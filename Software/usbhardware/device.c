/*
 * a314usb.device - USB hardware driver for Poseidon via A314.
 *
 * Architecture:
 *  - init_device opens a314.device (needs Process context)
 *  - A device task is spawned which creates its own message ports and IORequests
 *    and handles async USB transfers. It connects to the "usbhw" service and
 *    allocates the shared buffer at the first transfer, and resets the stream
 *    and frees the buffer when the last opener closes or the stream dies.
 *  - BeginIO queues transfer requests to the device task via op_req_mp.
 *  - The device task uses separate read_ior/write_ior for concurrent I/O:
 *    DoIO for writes (short), and a read (SendIO) for the host's replies that
 *    stays pending while connected. Replies carry the request id.
 *  - Interrupt transfers stay pending on the host until the device has data,
 *    as on real USB, and several can be pending at once; their data travels
 *    in the request and reply messages. Control and bulk transfers go one at
 *    a time through the shared buffer.
 *  - Poseidon stores wValue/wIndex/wLength as little-endian (USB wire order);
 *    we swap them to big-endian for our protocol.
 *  - Transfer data goes through a shared memory buffer (AllocMemA314) that the
 *    host can read/write via A314 read_mem/write_mem.
 */

#include <exec/types.h>
#include <exec/execbase.h>
#include <exec/devices.h>
#include <exec/errors.h>
#include <exec/libraries.h>
#include <exec/memory.h>
#include <exec/ports.h>
#include <exec/tasks.h>
#include <exec/lists.h>
#include <proto/exec.h>
#include <clib/alib_protos.h>
#include <utility/tagitem.h>
#include <proto/utility.h>
#include <string.h>

#include "device.h"
#include "usbhw.h"
#include "usb_protocol.h"
#include "debug.h"
#include "../a314device/a314.h"
#include "../a314device/proto_a314.h"

// Device ID strings
const char device_name[] = DEVICE_NAME;
const char id_string[] = DEVICE_NAME " 1.1 (13.11.2026)";

// Verbose per-transfer logging
#define VERBOSE 0
#if VERBOSE
#define VLOG(...) kprintf(__VA_ARGS__)
#else
#define VLOG(...) do {} while (0)
#endif

#define SysBase (*(struct ExecBase **)4)

static struct UsbDevice *global_dev_base = NULL;
#define UtilityBase (global_dev_base->utility_base)
#define A314Base (global_dev_base->a314_base)

#define SERVICE_NAME "usbhw"

// Signal bits (CTRL_D/E/F are break signals — safe for user use too)
#define SIGB_A314       SIGBREAKB_CTRL_D
#define SIGB_OP_REQ     SIGBREAKB_CTRL_E
#define SIGB_A314_WRITE SIGBREAKB_CTRL_F

#define USB_DATA_BUF_SIZE 4096   // shared with the other A314 services
#define TASK_STACK_SIZE   8192

// Global USB state
UWORD usb_state = UHSF_OPERATIONAL;

// Forward declarations
static struct UsbDevice *init_device(__reg("a6") struct ExecBase *sys_base,
                                      __reg("a0") BPTR seg_list,
                                      __reg("d0") struct UsbDevice *dev);
static struct UsbDevice *dev_open(__reg("a6") struct UsbDevice *dev,
                                   __reg("a1") struct IOUsbHWReq *ioreq,
                                   __reg("d0") ULONG unit,
                                   __reg("d1") ULONG flags);
static BPTR dev_close(__reg("a6") struct UsbDevice *dev,
                       __reg("a1") struct IOUsbHWReq *ioreq);
static BPTR dev_expunge(__reg("a6") struct UsbDevice *dev);
static void dev_begin_io(__reg("a6") struct UsbDevice *dev,
                          __reg("a1") struct IOUsbHWReq *ioreq);
static ULONG dev_abort_io(__reg("a6") struct UsbDevice *dev,
                           __reg("a1") struct IOUsbHWReq *ioreq);

static void task_entry(void);
static void handle_a314_response(struct UsbDevice *dev);
static void handle_op_requests(struct UsbDevice *dev);
static void handle_aborts(struct UsbDevice *dev);
static void start_buffer_xfer(struct UsbDevice *dev);
static void build_data_xfer_req(UBYTE *req_buf, UBYTE kind,
                                 struct UsbDevice *dev,
                                 struct IOUsbHWReq *ioreq,
                                 UBYTE endpoint, UWORD length,
                                 ULONG req_id);
static void send_bulk_piece(struct UsbDevice *dev, struct IOUsbHWReq *ioreq);
static void process_async_control_xfer(struct UsbDevice *dev, struct IOUsbHWReq *ioreq);
static void process_async_bulk_xfer(struct UsbDevice *dev, struct IOUsbHWReq *ioreq);


// ============================================================================
// Helpers
// ============================================================================

static void init_message_port(struct MsgPort **mp_ptr, BYTE signal, struct Task *task)
{
    struct MsgPort *mp = AllocMem(sizeof(struct MsgPort), MEMF_PUBLIC | MEMF_CLEAR);
    if (mp)
    {
        mp->mp_Node.ln_Type = NT_MSGPORT;
        mp->mp_Flags = PA_SIGNAL;
        mp->mp_SigBit = signal;
        mp->mp_SigTask = task;
        mp->mp_MsgList.lh_Head = (struct Node *)&mp->mp_MsgList.lh_Tail;
        mp->mp_MsgList.lh_TailPred = (struct Node *)&mp->mp_MsgList.lh_Head;
        mp->mp_MsgList.lh_Tail = NULL;
    }
    *mp_ptr = mp;
}

static void init_a314_ioreq(struct A314_IORequest **ior_ptr, struct MsgPort *mp)
{
    struct A314_IORequest *ior = AllocMem(sizeof(struct A314_IORequest),
                                           MEMF_PUBLIC | MEMF_CLEAR);
    if (ior)
    {
        ior->a314_Request.io_Message.mn_Node.ln_Type = NT_REPLYMSG;
        ior->a314_Request.io_Message.mn_ReplyPort = mp;
        ior->a314_Request.io_Message.mn_Length = sizeof(struct A314_IORequest);
    }
    *ior_ptr = ior;
}


// ============================================================================
// Device init (called from init task context)
// ============================================================================

static struct UsbDevice *init_device(__reg("a6") struct ExecBase *sys_base,
                                      __reg("a0") BPTR seg_list,
                                      __reg("d0") struct UsbDevice *dev)
{
    dev->saved_seg_list = seg_list;
    dev->utility_base = NULL;
    dev->a314_base = NULL;
    dev->a314_mp = NULL;
    dev->write_mp = NULL;
    dev->op_req_mp = NULL;
    dev->read_ior = NULL;
    dev->write_ior = NULL;
    dev->a314_socket = -1;
    dev->task = FindTask(NULL);
    dev->task_running = FALSE;
    dev->device_task = NULL;
    dev->task_stack = NULL;
    dev->expunge_task = NULL;
    dev->abort_count = 0;
    dev->task_ready = FALSE;
    dev->connected = FALSE;
    dev->connect_failed = FALSE;
    dev->pending_read = FALSE;
    dev->current_ioreq = NULL;
    dev->xfer_offset = 0;
    dev->xfer_piece_max = 0;
    dev->dataBufferAddr = INVALID_A314_ADDRESS;
    dev->dataBufferSize = 0;
    dev->next_req_id = 1;

    dev->lib.lib_Node.ln_Type = NT_DEVICE;
    dev->lib.lib_Node.ln_Name = (char *)device_name;
    dev->lib.lib_Flags = LIBF_SUMUSED | LIBF_CHANGED;
    dev->lib.lib_Version = DEVICE_VERSION;
    dev->lib.lib_Revision = DEVICE_REVISION;
    dev->lib.lib_IdString = (APTR)id_string;

    global_dev_base = dev;

    dev->utility_base = OpenLibrary("utility.library", 39);
    if (!dev->utility_base)
    {
        kprintf("a314usb.device: Failed to open utility.library");
        return NULL;
    }

    kprintf("a314usb.device: init_device start");

    // op_req_mp for Poseidon -> device task messages
    init_message_port(&dev->op_req_mp, SIGB_OP_REQ, dev->task);
    if (!dev->op_req_mp)
    {
        kprintf("a314usb.device: Failed to create op_req msg port");
        goto fail;
    }

    // Open a314.device from init context. The device task can't do it
    // because AddTask'd tasks lack the process context OpenDevice needs.
    {
        struct MsgPort *tmp_mp = NULL;
        struct A314_IORequest *tmp_ior = NULL;

        init_message_port(&tmp_mp, SIGB_A314, dev->task);
        if (!tmp_mp) { kprintf("a314usb.device: tmp msg port fail"); goto fail; }

        init_a314_ioreq(&tmp_ior, tmp_mp);
        if (!tmp_ior)
        {
            FreeMem(tmp_mp, sizeof(struct MsgPort));
            kprintf("a314usb.device: tmp ioreq fail");
            goto fail;
        }

        kprintf("a314usb.device: Opening a314.device");
        if (OpenDevice((char *)A314_NAME, 0, &tmp_ior->a314_Request, 0))
        {
            kprintf("a314usb.device: Failed to open a314.device");
            FreeMem(tmp_ior, sizeof(struct A314_IORequest));
            FreeMem(tmp_mp, sizeof(struct MsgPort));
            goto fail;
        }

        dev->a314_base = &(tmp_ior->a314_Request.io_Device->dd_Library);
        kprintf("a314usb.device: a314.device opened, base=0x%08lx",
                (ULONG)dev->a314_base);

        FreeMem(tmp_ior, sizeof(struct A314_IORequest));
        FreeMem(tmp_mp, sizeof(struct MsgPort));
    }

    // Allocate task stack
    dev->task_stack = AllocMem(TASK_STACK_SIZE, MEMF_PUBLIC | MEMF_CLEAR);
    if (!dev->task_stack)
    {
        kprintf("a314usb.device: Failed to alloc task stack");
        goto fail;
    }

    dev->device_task = AllocMem(sizeof(struct Task), MEMF_PUBLIC | MEMF_CLEAR);
    if (!dev->device_task)
    {
        kprintf("a314usb.device: Failed to alloc task struct");
        goto fail_free_stack;
    }

    dev->device_task->tc_Node.ln_Type = NT_TASK;
    dev->device_task->tc_Node.ln_Pri = 0;
    dev->device_task->tc_Node.ln_Name = "a314usb.device task";
    dev->device_task->tc_SPLower = dev->task_stack;
    dev->device_task->tc_SPUpper = (APTR)((UBYTE *)dev->task_stack + TASK_STACK_SIZE);
    dev->device_task->tc_SPReg = dev->device_task->tc_SPUpper;
    dev->device_task->tc_UserData = (APTR)dev;

    dev->op_req_mp->mp_SigTask = dev->device_task;

    AddTask(dev->device_task, (void *)task_entry, NULL);
    dev->task_running = TRUE;

    // Wait for task to finish its init
    kprintf("a314usb.device: waiting for task init");
    Wait(SIGF_SINGLE);

    if (!dev->task_ready)
    {
        kprintf("a314usb.device: Device task failed to start");
        goto fail_free_stack;
    }

    kprintf("a314usb.device: initialized");
    return dev;

fail_free_stack:
    if (dev->device_task)
    {
        FreeMem(dev->device_task, sizeof(struct Task));
        dev->device_task = NULL;
    }
    if (dev->task_stack)
    {
        FreeMem(dev->task_stack, TASK_STACK_SIZE);
        dev->task_stack = NULL;
    }

fail:
    if (dev->op_req_mp) { FreeMem(dev->op_req_mp, sizeof(struct MsgPort)); dev->op_req_mp = NULL; }
    if (dev->utility_base) { CloseLibrary(dev->utility_base); dev->utility_base = NULL; }
    return NULL;
}


// ============================================================================
// Device open / close / expunge
// ============================================================================

static struct UsbDevice *dev_open(__reg("a6") struct UsbDevice *dev,
                                   __reg("a1") struct IOUsbHWReq *ioreq,
                                   __reg("d0") ULONG unit,
                                   __reg("d1") ULONG flags)
{
    kprintf("a314usb.device: open unit=%ld", unit);

    if (unit != 0)
    {
        ioreq->iouh_Req.io_Error = IOERR_OPENFAIL;
        return NULL;
    }

    ioreq->iouh_Req.io_Unit = NULL;
    ioreq->iouh_Req.io_Error = 0;
    dev->lib.lib_OpenCnt++;
    dev->lib.lib_Flags &= ~LIBF_DELEXP;
    return dev;
}

static BPTR dev_close(__reg("a6") struct UsbDevice *dev,
                       __reg("a1") struct IOUsbHWReq *ioreq)
{
    kprintf("a314usb.device: close");

    ioreq->iouh_Req.io_Device = NULL;
    ioreq->iouh_Req.io_Unit = NULL;
    dev->lib.lib_OpenCnt--;

    // The device task disconnects once nobody has the device open.
    if (dev->lib.lib_OpenCnt == 0)
        Signal(dev->device_task, 1L << SIGB_OP_REQ);

    if (dev->lib.lib_OpenCnt == 0 && (dev->lib.lib_Flags & LIBF_DELEXP))
        return dev_expunge(dev);
    return 0;
}

static BPTR dev_expunge(__reg("a6") struct UsbDevice *dev)
{
    kprintf("a314usb.device: expunge");

    if (dev->lib.lib_OpenCnt != 0)
    {
        dev->lib.lib_Flags |= LIBF_DELEXP;
        return 0;
    }

    // Expunge runs under Forbid(); Wait() lets the device task run until it has
    // disconnected and exited.
    if (dev->task_running && dev->device_task)
    {
        dev->expunge_task = FindTask(NULL);
        SetSignal(0, SIGF_SINGLE);
        Signal(dev->device_task, SIGBREAKF_CTRL_C);
        Wait(SIGF_SINGLE);
        dev->task_running = FALSE;
    }

    if (dev->a314_base && dev->read_ior)
    {
        CloseDevice(&dev->read_ior->a314_Request);
        dev->a314_base = NULL;
    }

    if (dev->write_ior) { FreeMem(dev->write_ior, sizeof(struct A314_IORequest)); dev->write_ior = NULL; }
    if (dev->read_ior)  { FreeMem(dev->read_ior, sizeof(struct A314_IORequest));  dev->read_ior = NULL; }
    if (dev->op_req_mp) { FreeMem(dev->op_req_mp, sizeof(struct MsgPort)); dev->op_req_mp = NULL; }
    if (dev->write_mp)  { FreeMem(dev->write_mp, sizeof(struct MsgPort));  dev->write_mp = NULL; }
    if (dev->a314_mp)   { FreeMem(dev->a314_mp, sizeof(struct MsgPort));   dev->a314_mp = NULL; }
    if (dev->task_stack) { FreeMem(dev->task_stack, TASK_STACK_SIZE); dev->task_stack = NULL; }
    if (dev->device_task) { FreeMem(dev->device_task, sizeof(struct Task)); dev->device_task = NULL; }

    if (dev->utility_base)
    {
        CloseLibrary(dev->utility_base);
        dev->utility_base = NULL;
    }

    BPTR seg_list = dev->saved_seg_list;
    Remove(&dev->lib.lib_Node);
    FreeMem((char *)dev - dev->lib.lib_NegSize,
            dev->lib.lib_NegSize + dev->lib.lib_PosSize);
    return seg_list;
}


// ============================================================================
// BeginIO
// ============================================================================

static void dev_begin_io(__reg("a6") struct UsbDevice *dev,
                          __reg("a1") struct IOUsbHWReq *ioreq)
{
    UWORD cmd = ioreq->iouh_Req.io_Command;

    VLOG("a314usb.device: BeginIO cmd=%ld", (ULONG)cmd);

    ioreq->iouh_State = usb_state;
    ioreq->iouh_Req.io_Error = UHIOERR_NO_ERROR;

    // Transfer commands go to the device task
    if (cmd == UHCMD_CONTROLXFER || cmd == UHCMD_INTXFER || cmd == UHCMD_BULKXFER)
    {
        ioreq->iouh_Req.io_Flags &= ~IOF_QUICK;
        PutMsg(dev->op_req_mp, &ioreq->iouh_Req.io_Message);
        return;
    }

    // Other commands run synchronously here
    switch (cmd)
    {
        case CMD_RESET:
        case CMD_FLUSH:
            break;
        case UHCMD_QUERYDEVICE:
            cmdQueryDevice(ioreq, 0, dev);
            break;
        case UHCMD_USBRESET:
        case UHCMD_USBRESUME:
            usb_state = UHSF_OPERATIONAL;
            ioreq->iouh_State = usb_state;
            break;
        case UHCMD_USBSUSPEND:
            usb_state = UHSF_SUSPENDED;
            ioreq->iouh_State = usb_state;
            break;
        case UHCMD_USBOPER:
            usb_state = UHSF_OPERATIONAL;
            ioreq->iouh_State = usb_state;
            break;
        case UHCMD_ISOXFER:
            ioreq->iouh_Req.io_Error = IOERR_NOCMD;
            break;
        default:
            kprintf("a314usb.device: Unknown cmd %ld", (ULONG)cmd);
            ioreq->iouh_Req.io_Error = IOERR_NOCMD;
            break;
    }

    if (!(ioreq->iouh_Req.io_Flags & IOF_QUICK))
        ReplyMsg(&ioreq->iouh_Req.io_Message);
}

// The device task does the abort: it alone touches the transfer lists.
static ULONG dev_abort_io(__reg("a6") struct UsbDevice *dev,
                           __reg("a1") struct IOUsbHWReq *ioreq)
{
    Forbid();
    if (ioreq->iouh_Req.io_Message.mn_Node.ln_Type != NT_REPLYMSG &&
        dev->abort_count < ABORT_MAX)
    {
        dev->abort_reqs[dev->abort_count++] = ioreq;
    }
    Permit();
    Signal(dev->device_task, 1L << SIGB_OP_REQ);
    return 0;
}


// ============================================================================
// Device task
// ============================================================================

static void put_be32(UBYTE *p, ULONG v)
{
    p[0] = (v >> 24) & 0xFF;
    p[1] = (v >> 16) & 0xFF;
    p[2] = (v >> 8) & 0xFF;
    p[3] = v & 0xFF;
}

static ULONG get_be32(const UBYTE *p)
{
    return ((ULONG)p[0] << 24) | ((ULONG)p[1] << 16) | ((ULONG)p[2] << 8) | p[3];
}

// Reply to a transfer. A pending abort of it is dropped, so that it can't hit
// the request's next use.
static void reply_ioreq(struct UsbDevice *dev, struct IOUsbHWReq *ioreq,
                        BYTE error, ULONG actual)
{
    Forbid();
    UWORD j = 0;
    for (UWORD i = 0; i < dev->abort_count; i++)
    {
        if (dev->abort_reqs[i] != ioreq)
            dev->abort_reqs[j++] = dev->abort_reqs[i];
    }
    dev->abort_count = j;
    Permit();

    ioreq->iouh_Req.io_Error = error;
    ioreq->iouh_Actual = actual;
    ReplyMsg(&ioreq->iouh_Req.io_Message);
}

static void post_read(struct UsbDevice *dev)
{
    dev->read_ior->a314_Request.io_Command = A314_READ;
    dev->read_ior->a314_Socket = dev->a314_socket;
    dev->read_ior->a314_Buffer = (STRPTR)dev->res_msg_buf;
    dev->read_ior->a314_Length = sizeof(dev->res_msg_buf);
    SendIO(&dev->read_ior->a314_Request);
    dev->pending_read = TRUE;
}

// Connect to the usbhw service, allocate the shared data buffer, and keep a
// read for the host's replies pending from then on.
static BOOL connect_service(struct UsbDevice *dev)
{
    dev->read_ior->a314_Request.io_Command = A314_CONNECT;
    dev->read_ior->a314_Socket = dev->a314_socket;
    dev->read_ior->a314_Buffer = (STRPTR)SERVICE_NAME;
    dev->read_ior->a314_Length = strlen(SERVICE_NAME);
    DoIO(&dev->read_ior->a314_Request);

    if (dev->read_ior->a314_Request.io_Error != A314_CONNECT_OK)
    {
        if (!dev->connect_failed)
            kprintf("a314usb.device: connect to %s failed err=%ld", SERVICE_NAME,
                    (LONG)dev->read_ior->a314_Request.io_Error);
        dev->connect_failed = TRUE;
        return FALSE;
    }

    dev->dataBufferSize = USB_DATA_BUF_SIZE;
    dev->dataBufferAddr = AllocMemA314(USB_DATA_BUF_SIZE);
    if (dev->dataBufferAddr == INVALID_A314_ADDRESS)
    {
        if (!dev->connect_failed)
            kprintf("a314usb.device: AllocMemA314 failed");
        dev->connect_failed = TRUE;
        dev->write_ior->a314_Request.io_Command = A314_RESET;
        dev->write_ior->a314_Socket = dev->a314_socket;
        DoIO(&dev->write_ior->a314_Request);
        return FALSE;
    }

    kprintf("a314usb.device: Connected to %s, data buffer at 0x%08lx (%ld bytes)",
            SERVICE_NAME, dev->dataBufferAddr, dev->dataBufferSize);
    dev->connected = TRUE;
    dev->connect_failed = FALSE;
    post_read(dev);
    return TRUE;
}

// Reset the stream, free the shared data buffer, and fail every transfer that
// was started or waits for the buffer.
static void disconnect_service(struct UsbDevice *dev)
{
    if (!dev->connected)
        return;

    if (dev->pending_read)
    {
        AbortIO(&dev->read_ior->a314_Request);
        WaitIO(&dev->read_ior->a314_Request);
        dev->pending_read = FALSE;
    }

    dev->write_ior->a314_Request.io_Command = A314_RESET;
    dev->write_ior->a314_Socket = dev->a314_socket;
    DoIO(&dev->write_ior->a314_Request);

    FreeMemA314(dev->dataBufferAddr, dev->dataBufferSize);
    dev->dataBufferAddr = INVALID_A314_ADDRESS;
    dev->connected = FALSE;
    kprintf("a314usb.device: Disconnected from %s", SERVICE_NAME);

    if (dev->current_ioreq)
    {
        struct IOUsbHWReq *ioreq = dev->current_ioreq;
        dev->current_ioreq = NULL;
        reply_ioreq(dev, ioreq, UHIOERR_HOSTERROR, dev->xfer_offset);
    }

    struct IOUsbHWReq *ioreq;
    while ((ioreq = (struct IOUsbHWReq *)RemHead((struct List *)&dev->waiting)))
        reply_ioreq(dev, ioreq, UHIOERR_HOSTERROR, 0);

    for (UWORD i = 0; i < INT_PARKED_MAX; i++)
    {
        if (dev->parked[i].ioreq)
        {
            ioreq = dev->parked[i].ioreq;
            dev->parked[i].ioreq = NULL;
            reply_ioreq(dev, ioreq, UHIOERR_HOSTERROR, 0);
        }
    }
}

// Write one request to the host. A failed write means the stream is gone.
static BOOL send_request(struct UsbDevice *dev, UBYTE *req_buf, UWORD req_len)
{
    dev->write_ior->a314_Request.io_Command = A314_WRITE;
    dev->write_ior->a314_Socket = dev->a314_socket;
    dev->write_ior->a314_Buffer = (STRPTR)req_buf;
    dev->write_ior->a314_Length = req_len;
    DoIO(&dev->write_ior->a314_Request);

    if (dev->write_ior->a314_Request.io_Error != A314_WRITE_OK)
    {
        kprintf("a314usb.device: A314_WRITE err=%ld",
                (LONG)dev->write_ior->a314_Request.io_Error);
        disconnect_service(dev);
        return FALSE;
    }
    return TRUE;
}

static BOOL nothing_in_flight(struct UsbDevice *dev)
{
    if (dev->current_ioreq || !IsListEmpty((struct List *)&dev->waiting))
        return FALSE;
    for (UWORD i = 0; i < INT_PARKED_MAX; i++)
    {
        if (dev->parked[i].ioreq)
            return FALSE;
    }
    return TRUE;
}

static void task_entry(void)
{
    struct Task *this_task = FindTask(NULL);
    struct UsbDevice *dev = (struct UsbDevice *)this_task->tc_UserData;
    ULONG signals, wait_mask;

    kprintf("a314usb.device: task entry");

    // Create message ports owned by this task
    init_message_port(&dev->a314_mp, SIGB_A314, this_task);
    init_message_port(&dev->write_mp, SIGB_A314_WRITE, this_task);
    if (!dev->a314_mp || !dev->write_mp)
    {
        kprintf("a314usb.device: task: msg port fail");
        goto fail;
    }

    // Create IORequests (read for async, write for sync)
    init_a314_ioreq(&dev->read_ior, dev->a314_mp);
    init_a314_ioreq(&dev->write_ior, dev->write_mp);
    if (!dev->read_ior || !dev->write_ior)
    {
        kprintf("a314usb.device: task: ioreq fail");
        goto fail;
    }

    // Use the already-opened a314.device base
    dev->read_ior->a314_Request.io_Device = (struct Device *)dev->a314_base;
    dev->write_ior->a314_Request.io_Device = (struct Device *)dev->a314_base;

    dev->a314_socket = 1;
    dev->connected = FALSE;
    dev->pending_read = FALSE;
    dev->current_ioreq = NULL;
    NewList((struct List *)&dev->waiting);
    dev->task_ready = TRUE;

    // Signal init task that we're ready
    Signal(dev->task, SIGF_SINGLE);

    kprintf("a314usb.device: task ready");
    wait_mask = (1L << SIGB_A314) | (1L << SIGB_OP_REQ) | SIGBREAKF_CTRL_C;

    while (TRUE)
    {
        signals = Wait(wait_mask);

        if (signals & SIGBREAKF_CTRL_C)
        {
            kprintf("a314usb.device: task: shutdown");
            break;
        }

        if (signals & (1L << SIGB_A314))
            handle_a314_response(dev);

        handle_op_requests(dev);
        handle_aborts(dev);
        start_buffer_xfer(dev);

        if (dev->lib.lib_OpenCnt == 0 && nothing_in_flight(dev))
            disconnect_service(dev);
    }

    disconnect_service(dev);

    kprintf("a314usb.device: task exiting");

    // The expunging task frees this task's stack once it runs again, which is
    // after this task has been removed.
    Forbid();
    Signal(dev->expunge_task, SIGF_SINGLE);
    return;

fail:
    kprintf("a314usb.device: task init failed");
    Forbid();
    Signal(dev->task, SIGF_SINGLE);
}


// ============================================================================
// Host replies, requests and aborts
// ============================================================================

// A reply to the control or bulk transfer that has the shared buffer.
static void handle_buffer_reply(struct UsbDevice *dev, UBYTE kind, UBYTE status,
                                UWORD actual_length, ULONG req_id)
{
    struct IOUsbHWReq *ioreq = dev->current_ioreq;

    if (!ioreq || kind != dev->current_resp_type || req_id != dev->current_req_id)
    {
        kprintf("a314usb.device: unexpected reply kind=%ld id=%ld",
                (ULONG)kind, req_id);
        return;
    }

    switch (status)
    {
        case USB_STATUS_OK:
        {
            UWORD copy_len = actual_length;
            if (copy_len > dev->current_max_len)
                copy_len = dev->current_max_len;
            if (dev->current_is_in && copy_len > 0 && ioreq->iouh_Data)
                ReadMemA314((UBYTE *)ioreq->iouh_Data + dev->xfer_offset,
                            dev->dataBufferAddr, copy_len);
            dev->xfer_offset += copy_len;
            // A full piece of a bulk transfer is followed by the next one; a
            // short IN piece ends the transfer.
            if (kind == USB_BULK_TRANSFER_RES &&
                copy_len == dev->current_max_len &&
                dev->xfer_offset < ioreq->iouh_Length)
            {
                send_bulk_piece(dev, ioreq);
                return;
            }
            dev->current_ioreq = NULL;
            reply_ioreq(dev, ioreq, UHIOERR_NO_ERROR, dev->xfer_offset);
            break;
        }
        case USB_STATUS_STALL:
            dev->current_ioreq = NULL;
            reply_ioreq(dev, ioreq, UHIOERR_STALL, dev->xfer_offset);
            break;
        case USB_STATUS_NAK_TIMEOUT:
            dev->current_ioreq = NULL;
            reply_ioreq(dev, ioreq, UHIOERR_NAKTIMEOUT, dev->xfer_offset);
            break;
        default:
            dev->current_ioreq = NULL;
            reply_ioreq(dev, ioreq, UHIOERR_HOSTERROR, dev->xfer_offset);
            break;
    }
}

// A reply to an interrupt transfer; its IN data comes with it.
static void handle_int_reply(struct UsbDevice *dev, UBYTE status,
                             UWORD actual_length, ULONG req_id, UBYTE *data)
{
    struct ParkedXfer *p = NULL;
    for (UWORD i = 0; i < INT_PARKED_MAX; i++)
    {
        if (dev->parked[i].ioreq && dev->parked[i].req_id == req_id)
        {
            p = &dev->parked[i];
            break;
        }
    }
    if (!p)
        return;     // aborted

    struct IOUsbHWReq *ioreq = p->ioreq;
    p->ioreq = NULL;

    switch (status)
    {
        case USB_STATUS_OK:
        {
            UWORD copy_len = actual_length;
            if (copy_len > p->length)
                copy_len = p->length;
            if (p->is_in && copy_len > 0 && ioreq->iouh_Data)
                CopyMem(data, ioreq->iouh_Data, copy_len);
            reply_ioreq(dev, ioreq, UHIOERR_NO_ERROR, copy_len);
            break;
        }
        case USB_STATUS_STALL:
            reply_ioreq(dev, ioreq, UHIOERR_STALL, 0);
            break;
        case USB_STATUS_NAK_TIMEOUT:
            reply_ioreq(dev, ioreq, UHIOERR_NAKTIMEOUT, 0);
            break;
        default:
            reply_ioreq(dev, ioreq, UHIOERR_HOSTERROR, 0);
            break;
    }
}

static void handle_a314_response(struct UsbDevice *dev)
{
    if (!dev->pending_read)
        return;
    if (!GetMsg(dev->a314_mp))
        return;

    WaitIO(&dev->read_ior->a314_Request);
    dev->pending_read = FALSE;

    if (dev->read_ior->a314_Request.io_Error != A314_READ_OK)
    {
        kprintf("a314usb.device: stream lost err=%ld",
                (LONG)dev->read_ior->a314_Request.io_Error);
        disconnect_service(dev);
        return;
    }

    if (dev->read_ior->a314_Length < 8)
    {
        kprintf("a314usb.device: short reply len=%ld", dev->read_ior->a314_Length);
    }
    else
    {
        UBYTE *res = dev->res_msg_buf;
        UBYTE kind = res[0];
        UBYTE status = res[1];
        UWORD actual_length = (res[2] << 8) | res[3];
        ULONG req_id = get_be32(&res[4]);

        if (kind == USB_INT_TRANSFER_RES)
        {
            if (8 + (ULONG)actual_length > dev->read_ior->a314_Length)
                actual_length = dev->read_ior->a314_Length - 8;
            handle_int_reply(dev, status, actual_length, req_id, &res[8]);
        }
        else
        {
            handle_buffer_reply(dev, kind, status, actual_length, req_id);
        }
    }

    if (dev->connected && !dev->pending_read)
        post_read(dev);
}

// Interrupt transfers go to the host at once and stay there until the device
// has data; their data travels in the request and the reply, not the buffer.
static void start_int_xfer(struct UsbDevice *dev, struct IOUsbHWReq *ioreq)
{
    UBYTE req_buf[16 + USB_INT_DATA_MAX];
    UBYTE is_in = (ioreq->iouh_Dir == UHDIR_IN);
    UBYTE endpoint = (UBYTE)(ioreq->iouh_Endpoint | (is_in ? 0x80 : 0x00));
    ULONG length = ioreq->iouh_Length;

    if (is_in && length > USB_INT_DATA_MAX)
        length = USB_INT_DATA_MAX;
    if (!is_in && length > USB_INT_DATA_MAX)
    {
        reply_ioreq(dev, ioreq, UHIOERR_BADPARAMS, 0);
        return;
    }

    struct ParkedXfer *p = NULL;
    for (UWORD i = 0; i < INT_PARKED_MAX; i++)
    {
        if (!dev->parked[i].ioreq)
        {
            p = &dev->parked[i];
            break;
        }
    }
    if (!p)
    {
        kprintf("a314usb.device: too many interrupt transfers");
        reply_ioreq(dev, ioreq, UHIOERR_HOSTERROR, 0);
        return;
    }

    ULONG req_id = dev->next_req_id++;
    build_data_xfer_req(req_buf, USB_INT_TRANSFER_REQ, dev, ioreq, endpoint, length, req_id);
    UWORD req_len = 16;
    if (!is_in && length > 0 && ioreq->iouh_Data)
    {
        CopyMem(ioreq->iouh_Data, &req_buf[16], length);
        req_len += length;
    }

    p->ioreq = ioreq;
    p->req_id = req_id;
    p->length = length;
    p->is_in = is_in;
    send_request(dev, req_buf, req_len);
}

// Take the new requests: interrupt transfers start, control and bulk transfers
// queue for the shared buffer.
static void handle_op_requests(struct UsbDevice *dev)
{
    while (TRUE)
    {
        // Under Forbid() a request is always on a list: dev_abort_io() can
        // only queue it for handle_aborts().
        Forbid();
        struct IOUsbHWReq *ioreq = (struct IOUsbHWReq *)GetMsg(dev->op_req_mp);
        if (ioreq && ioreq->iouh_Req.io_Command != UHCMD_INTXFER)
            AddTail((struct List *)&dev->waiting, &ioreq->iouh_Req.io_Message.mn_Node);
        Permit();

        if (!ioreq)
            return;
        if (ioreq->iouh_Req.io_Command != UHCMD_INTXFER)
            continue;

        if (!dev->connected && !connect_service(dev))
        {
            reply_ioreq(dev, ioreq, UHIOERR_USBOFFLINE, 0);
            continue;
        }
        start_int_xfer(dev, ioreq);
    }
}

static BOOL remove_if_listed(struct List *list, struct IOUsbHWReq *ioreq)
{
    BOOL found = FALSE;
    Forbid();
    for (struct Node *n = list->lh_Head; n->ln_Succ; n = n->ln_Succ)
    {
        if (n == &ioreq->iouh_Req.io_Message.mn_Node)
        {
            Remove(n);
            found = TRUE;
            break;
        }
    }
    Permit();
    return found;
}

// Aborted requests: one not started yet is taken off its list, a pending
// interrupt transfer is cancelled on the host too. A control or bulk transfer
// in progress finishes.
static void handle_aborts(struct UsbDevice *dev)
{
    while (TRUE)
    {
        Forbid();
        struct IOUsbHWReq *ioreq = NULL;
        if (dev->abort_count > 0)
            ioreq = dev->abort_reqs[--dev->abort_count];
        Permit();
        if (!ioreq)
            return;

        if (remove_if_listed(&dev->op_req_mp->mp_MsgList, ioreq) ||
            remove_if_listed((struct List *)&dev->waiting, ioreq))
        {
            reply_ioreq(dev, ioreq, IOERR_ABORTED, 0);
            continue;
        }

        for (UWORD i = 0; i < INT_PARKED_MAX; i++)
        {
            if (dev->parked[i].ioreq == ioreq)
            {
                UBYTE req_buf[8];
                req_buf[0] = USB_ABORT_TRANSFER_REQ;
                req_buf[1] = 0;
                req_buf[2] = 0;
                req_buf[3] = 0;
                put_be32(&req_buf[4], dev->parked[i].req_id);
                dev->parked[i].ioreq = NULL;
                reply_ioreq(dev, ioreq, IOERR_ABORTED, 0);
                if (dev->connected)
                    send_request(dev, req_buf, sizeof(req_buf));
                break;
            }
        }
    }
}

// Start the next control or bulk transfer once the shared buffer is free.
static void start_buffer_xfer(struct UsbDevice *dev)
{
    while (!dev->current_ioreq)
    {
        Forbid();
        struct IOUsbHWReq *ioreq = (struct IOUsbHWReq *)RemHead((struct List *)&dev->waiting);
        Permit();
        if (!ioreq)
            return;

        if (!dev->connected && !connect_service(dev))
        {
            reply_ioreq(dev, ioreq, UHIOERR_USBOFFLINE, 0);
            continue;
        }

        dev->xfer_offset = 0;
        if (ioreq->iouh_Req.io_Command == UHCMD_CONTROLXFER)
            process_async_control_xfer(dev, ioreq);
        else
            process_async_bulk_xfer(dev, ioreq);
    }
}

// Send a control or bulk request that uses the shared buffer, and wait for its
// reply in handle_buffer_reply().
static void send_buffer_request(struct UsbDevice *dev, struct IOUsbHWReq *ioreq,
                                UBYTE *req_buf, UWORD req_len, ULONG req_id,
                                UBYTE resp_type, UBYTE is_in, UWORD max_len)
{
    dev->current_ioreq = ioreq;
    dev->current_req_id = req_id;
    dev->current_resp_type = resp_type;
    dev->current_is_in = is_in;
    dev->current_max_len = max_len;
    send_request(dev, req_buf, req_len);
}


// ============================================================================
// Transfer builders
// ============================================================================

// Swap a 16-bit value between LE (Poseidon's wire order) and native BE.
static UWORD swap16(UWORD v)
{
    return (UWORD)((v >> 8) | (v << 8));
}

static void process_async_control_xfer(struct UsbDevice *dev, struct IOUsbHWReq *ioreq)
{
    UBYTE req_buf[18];
    ULONG req_id;
    UBYTE is_in = (ioreq->iouh_SetupData.bmRequestType & 0x80) != 0;
    // Poseidon stores wLength as little-endian USB wire order.
    UWORD wLength = swap16(ioreq->iouh_SetupData.wLength);

    // The host writes IN data to the shared buffer, so never ask for more.
    if (is_in && wLength > dev->dataBufferSize)
        wLength = dev->dataBufferSize;

    if (!is_in && wLength > 0 && ioreq->iouh_Data)
    {
        if (wLength > dev->dataBufferSize)
        {
            reply_ioreq(dev, ioreq, UHIOERR_BADPARAMS, 0);
            return;
        }
        WriteMemA314(dev->dataBufferAddr, ioreq->iouh_Data, wLength);
    }

    req_id = dev->next_req_id++;

    req_buf[0] = USB_CONTROL_TRANSFER_REQ;
    req_buf[1] = (UBYTE)ioreq->iouh_DevAddr;
    req_buf[2] = ioreq->iouh_SetupData.bmRequestType;
    req_buf[3] = ioreq->iouh_SetupData.bRequest;
    // Swap LE fields to BE for our protocol
    req_buf[4] = ioreq->iouh_SetupData.wValue & 0xFF;
    req_buf[5] = (ioreq->iouh_SetupData.wValue >> 8) & 0xFF;
    req_buf[6] = ioreq->iouh_SetupData.wIndex & 0xFF;
    req_buf[7] = (ioreq->iouh_SetupData.wIndex >> 8) & 0xFF;
    req_buf[8] = (wLength >> 8) & 0xFF;
    req_buf[9] = wLength & 0xFF;
    // Shared buffer address (BE)
    put_be32(&req_buf[10], dev->dataBufferAddr);
    // Request ID (BE)
    put_be32(&req_buf[14], req_id);

    send_buffer_request(dev, ioreq, req_buf, 18, req_id,
                        USB_CONTROL_TRANSFER_RES, is_in, wLength);
}

static void build_data_xfer_req(UBYTE *req_buf, UBYTE kind,
                                 struct UsbDevice *dev,
                                 struct IOUsbHWReq *ioreq,
                                 UBYTE endpoint, UWORD length,
                                 ULONG req_id)
{
    req_buf[0] = kind;
    req_buf[1] = (UBYTE)ioreq->iouh_DevAddr;
    req_buf[2] = endpoint;
    req_buf[3] = 0;
    req_buf[4] = (ioreq->iouh_MaxPktSize >> 8) & 0xFF;
    req_buf[5] = ioreq->iouh_MaxPktSize & 0xFF;
    req_buf[6] = (length >> 8) & 0xFF;
    req_buf[7] = length & 0xFF;
    put_be32(&req_buf[8], dev->dataBufferAddr);
    put_be32(&req_buf[12], req_id);
}

// Send the next piece of a bulk transfer, starting at xfer_offset.
static void send_bulk_piece(struct UsbDevice *dev, struct IOUsbHWReq *ioreq)
{
    UBYTE req_buf[16];
    UBYTE is_in = (ioreq->iouh_Dir == UHDIR_IN);
    UBYTE endpoint = (UBYTE)(ioreq->iouh_Endpoint | (is_in ? 0x80 : 0x00));
    ULONG length = ioreq->iouh_Length - dev->xfer_offset;

    if (length > dev->xfer_piece_max)
        length = dev->xfer_piece_max;

    if (!is_in && length > 0 && ioreq->iouh_Data)
        WriteMemA314(dev->dataBufferAddr, (UBYTE *)ioreq->iouh_Data + dev->xfer_offset, length);

    ULONG req_id = dev->next_req_id++;
    build_data_xfer_req(req_buf, USB_BULK_TRANSFER_REQ, dev, ioreq, endpoint, length, req_id);
    send_buffer_request(dev, ioreq, req_buf, 16, req_id,
                        USB_BULK_TRANSFER_RES, is_in, length);
}

// A transfer larger than the shared buffer goes in pieces that are whole
// packets, so only the last piece can be short.
static void process_async_bulk_xfer(struct UsbDevice *dev, struct IOUsbHWReq *ioreq)
{
    ULONG max_pkt = ioreq->iouh_MaxPktSize;

    // Whole packets that fit the buffer; no division, as no division routines
    // are linked in.
    if (max_pkt > 0 && max_pkt <= dev->dataBufferSize)
    {
        dev->xfer_piece_max = max_pkt;
        while (dev->xfer_piece_max + max_pkt <= dev->dataBufferSize)
            dev->xfer_piece_max += max_pkt;
    }
    else
    {
        dev->xfer_piece_max = dev->dataBufferSize;
    }

    send_bulk_piece(dev, ioreq);
}


// ============================================================================
// Library vectors & auto_init_tables (used by RTF_AUTOINIT)
// ============================================================================

static const APTR device_vectors[] =
{
    (APTR)dev_open,
    (APTR)dev_close,
    (APTR)dev_expunge,
    NULL,
    (APTR)dev_begin_io,
    (APTR)dev_abort_io,
    (APTR)-1
};

ULONG auto_init_tables[] =
{
    (ULONG)sizeof(struct UsbDevice),
    (ULONG)device_vectors,
    0,
    (ULONG)init_device
};
