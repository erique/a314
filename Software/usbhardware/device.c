/*
 * a314usb.device - USB hardware driver for Poseidon via A314.
 *
 * Architecture:
 *  - init_device opens a314.device (needs Process context)
 *  - A device task is spawned which creates its own message ports and IORequests,
 *    connects to the "usbhw" service, allocates a shared buffer, and handles
 *    async USB transfers.
 *  - BeginIO queues transfer requests to the device task via op_req_mp.
 *  - The device task uses separate read_ior/write_ior for concurrent I/O:
 *    DoIO for writes (short), SendIO for reads (response), main loop waits on
 *    both SIGB_A314 (read complete) and SIGB_OP_REQ (new request).
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
const char id_string[] = DEVICE_NAME " 0.6 (2026)";

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

#define USB_DATA_BUF_SIZE 32768
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
static void send_xfer_request(struct UsbDevice *dev, struct IOUsbHWReq *ioreq,
                               UBYTE *req_buf, UWORD req_len,
                               UBYTE resp_type, UBYTE is_in, UWORD max_len);
static void process_async_control_xfer(struct UsbDevice *dev, struct IOUsbHWReq *ioreq);
static void process_async_int_xfer(struct UsbDevice *dev, struct IOUsbHWReq *ioreq);
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
    dev->connected = FALSE;
    dev->pending_read = FALSE;
    dev->current_ioreq = NULL;
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

    if (!dev->connected)
    {
        kprintf("a314usb.device: Device task failed to connect");
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

    // Signal task to stop, spin-wait for it to remove itself.
    if (dev->task_running && dev->device_task)
    {
        Signal(dev->device_task, SIGBREAKF_CTRL_C);
        volatile ULONG i;
        while (FindTask("a314usb.device task"))
            for (i = 0; i < 100000; i++) { /* tight loop, yields via scheduler */ }
        dev->task_running = FALSE;
    }

    if (dev->dataBufferAddr != INVALID_A314_ADDRESS)
    {
        FreeMemA314(dev->dataBufferAddr, dev->dataBufferSize);
        dev->dataBufferAddr = INVALID_A314_ADDRESS;
    }

    if (dev->a314_socket != -1 && dev->a314_base && dev->read_ior)
    {
        dev->read_ior->a314_Request.io_Command = A314_RESET;
        dev->read_ior->a314_Socket = dev->a314_socket;
        DoIO(&dev->read_ior->a314_Request);
        dev->a314_socket = -1;
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

static ULONG dev_abort_io(__reg("a6") struct UsbDevice *dev,
                           __reg("a1") struct IOUsbHWReq *ioreq)
{
    return IOERR_NOCMD;
}


// ============================================================================
// Device task
// ============================================================================

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

    // Connect to usbhw service
    dev->read_ior->a314_Request.io_Command = A314_CONNECT;
    dev->read_ior->a314_Socket = 1;
    dev->read_ior->a314_Buffer = (STRPTR)SERVICE_NAME;
    dev->read_ior->a314_Length = strlen(SERVICE_NAME);
    kprintf("a314usb.device: task: connecting to %s", SERVICE_NAME);
    DoIO(&dev->read_ior->a314_Request);

    if (dev->read_ior->a314_Request.io_Error != A314_CONNECT_OK)
    {
        kprintf("a314usb.device: task: connect failed err=%ld",
                (LONG)dev->read_ior->a314_Request.io_Error);
        goto fail;
    }

    dev->a314_socket = 1;
    dev->write_ior->a314_Socket = dev->a314_socket;
    kprintf("a314usb.device: Connected to %s", SERVICE_NAME);

    // Allocate shared memory data buffer
    dev->dataBufferSize = USB_DATA_BUF_SIZE;
    dev->dataBufferAddr = AllocMemA314(USB_DATA_BUF_SIZE);
    if (dev->dataBufferAddr == INVALID_A314_ADDRESS)
    {
        kprintf("a314usb.device: task: AllocMemA314 fail");
        goto fail;
    }
    kprintf("a314usb.device: data buffer at 0x%08lx (%ld bytes)",
            dev->dataBufferAddr, dev->dataBufferSize);

    dev->connected = TRUE;
    dev->pending_read = FALSE;
    dev->current_ioreq = NULL;

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

        // Always try to pick up queued op requests; the SIGB_OP_REQ bit
        // may have been consumed earlier while a transfer was in flight.
        handle_op_requests(dev);
    }

    if (dev->pending_read)
    {
        AbortIO(&dev->read_ior->a314_Request);
        WaitIO(&dev->read_ior->a314_Request);
        dev->pending_read = FALSE;
    }

    kprintf("a314usb.device: task exiting");
    return;

fail:
    kprintf("a314usb.device: task init failed");
    Signal(dev->task, SIGF_SINGLE);
}


// ============================================================================
// Response handler + transfer dispatcher
// ============================================================================

static void handle_a314_response(struct UsbDevice *dev)
{
    struct IOUsbHWReq *ioreq;
    UBYTE status;
    UWORD actual_length, copy_len;

    if (!dev->pending_read)
        return;
    if (!GetMsg(dev->a314_mp))
        return;

    WaitIO(&dev->read_ior->a314_Request);
    dev->pending_read = FALSE;

    ioreq = dev->current_ioreq;
    dev->current_ioreq = NULL;

    if (!ioreq)
        return;

    if (dev->read_ior->a314_Request.io_Error != 0 ||
        dev->read_ior->a314_Length < 8 ||
        dev->res_msg_buf[0] != dev->current_resp_type)
    {
        kprintf("a314usb.device: bad resp err=%ld kind=%ld want=%ld",
                (LONG)dev->read_ior->a314_Request.io_Error,
                (ULONG)dev->res_msg_buf[0],
                (ULONG)dev->current_resp_type);
        ioreq->iouh_Req.io_Error = UHIOERR_HOSTERROR;
        ReplyMsg(&ioreq->iouh_Req.io_Message);
        return;
    }

    status = dev->res_msg_buf[1];
    actual_length = (dev->res_msg_buf[2] << 8) | dev->res_msg_buf[3];

    switch (status)
    {
        case USB_STATUS_OK:
            copy_len = actual_length;
            if (copy_len > dev->current_max_len)
                copy_len = dev->current_max_len;
            ioreq->iouh_Req.io_Error = UHIOERR_NO_ERROR;
            ioreq->iouh_Actual = copy_len;
            if (dev->current_is_in && copy_len > 0 && ioreq->iouh_Data)
                ReadMemA314(ioreq->iouh_Data, dev->dataBufferAddr, copy_len);
            break;
        case USB_STATUS_STALL:
            ioreq->iouh_Req.io_Error = UHIOERR_STALL;
            break;
        case USB_STATUS_NAK_TIMEOUT:
            ioreq->iouh_Req.io_Error = UHIOERR_NAKTIMEOUT;
            break;
        default:
            ioreq->iouh_Req.io_Error = UHIOERR_HOSTERROR;
            break;
    }

    ReplyMsg(&ioreq->iouh_Req.io_Message);
}

// Send A314 WRITE (sync), then SendIO an async A314 READ.
// When the read completes we'll process the response.
static void send_xfer_request(struct UsbDevice *dev, struct IOUsbHWReq *ioreq,
                               UBYTE *req_buf, UWORD req_len,
                               UBYTE resp_type, UBYTE is_in, UWORD max_len)
{
    dev->write_ior->a314_Request.io_Command = A314_WRITE;
    dev->write_ior->a314_Socket = dev->a314_socket;
    dev->write_ior->a314_Buffer = (STRPTR)req_buf;
    dev->write_ior->a314_Length = req_len;

    DoIO(&dev->write_ior->a314_Request);

    if (dev->write_ior->a314_Request.io_Error != 0)
    {
        kprintf("a314usb.device: A314_WRITE err=%ld",
                (LONG)dev->write_ior->a314_Request.io_Error);
        ioreq->iouh_Req.io_Error = UHIOERR_HOSTERROR;
        ReplyMsg(&ioreq->iouh_Req.io_Message);
        return;
    }

    dev->current_ioreq = ioreq;
    dev->current_resp_type = resp_type;
    dev->current_is_in = is_in;
    dev->current_max_len = max_len;

    dev->read_ior->a314_Request.io_Command = A314_READ;
    dev->read_ior->a314_Socket = dev->a314_socket;
    dev->read_ior->a314_Buffer = (STRPTR)dev->res_msg_buf;
    dev->read_ior->a314_Length = sizeof(dev->res_msg_buf);
    SendIO(&dev->read_ior->a314_Request);
    dev->pending_read = TRUE;
}

// Dequeue one Poseidon request (if any) and start a transfer.
// Skips if a transfer is currently in flight.
static void handle_op_requests(struct UsbDevice *dev)
{
    struct IOUsbHWReq *ioreq;

    if (dev->pending_read)
        return;

    ioreq = (struct IOUsbHWReq *)GetMsg(dev->op_req_mp);
    if (!ioreq)
        return;

    switch (ioreq->iouh_Req.io_Command)
    {
        case UHCMD_CONTROLXFER:
            process_async_control_xfer(dev, ioreq);
            break;
        case UHCMD_INTXFER:
            process_async_int_xfer(dev, ioreq);
            break;
        case UHCMD_BULKXFER:
            process_async_bulk_xfer(dev, ioreq);
            break;
        default:
            ioreq->iouh_Req.io_Error = IOERR_NOCMD;
            ReplyMsg(&ioreq->iouh_Req.io_Message);
            break;
    }
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

    if (!is_in && wLength > 0 && ioreq->iouh_Data)
    {
        if (wLength > dev->dataBufferSize)
        {
            ioreq->iouh_Req.io_Error = UHIOERR_BADPARAMS;
            ReplyMsg(&ioreq->iouh_Req.io_Message);
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
    req_buf[8] = ioreq->iouh_SetupData.wLength & 0xFF;
    req_buf[9] = (ioreq->iouh_SetupData.wLength >> 8) & 0xFF;
    // Shared buffer address (BE)
    req_buf[10] = (dev->dataBufferAddr >> 24) & 0xFF;
    req_buf[11] = (dev->dataBufferAddr >> 16) & 0xFF;
    req_buf[12] = (dev->dataBufferAddr >> 8) & 0xFF;
    req_buf[13] = dev->dataBufferAddr & 0xFF;
    // Request ID (BE)
    req_buf[14] = (req_id >> 24) & 0xFF;
    req_buf[15] = (req_id >> 16) & 0xFF;
    req_buf[16] = (req_id >> 8) & 0xFF;
    req_buf[17] = req_id & 0xFF;

    send_xfer_request(dev, ioreq, req_buf, 18,
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
    req_buf[8] = (dev->dataBufferAddr >> 24) & 0xFF;
    req_buf[9] = (dev->dataBufferAddr >> 16) & 0xFF;
    req_buf[10] = (dev->dataBufferAddr >> 8) & 0xFF;
    req_buf[11] = dev->dataBufferAddr & 0xFF;
    req_buf[12] = (req_id >> 24) & 0xFF;
    req_buf[13] = (req_id >> 16) & 0xFF;
    req_buf[14] = (req_id >> 8) & 0xFF;
    req_buf[15] = req_id & 0xFF;
}

static void process_async_int_xfer(struct UsbDevice *dev, struct IOUsbHWReq *ioreq)
{
    UBYTE req_buf[16];
    ULONG req_id;
    UBYTE is_in = (ioreq->iouh_Dir == 2);  // UHDIR_IN
    UBYTE endpoint = (UBYTE)(ioreq->iouh_Endpoint | (is_in ? 0x80 : 0x00));
    UWORD length = (UWORD)ioreq->iouh_Length;

    if (!is_in && length > 0 && ioreq->iouh_Data)
    {
        if (length > dev->dataBufferSize)
        {
            ioreq->iouh_Req.io_Error = UHIOERR_BADPARAMS;
            ReplyMsg(&ioreq->iouh_Req.io_Message);
            return;
        }
        WriteMemA314(dev->dataBufferAddr, ioreq->iouh_Data, length);
    }

    req_id = dev->next_req_id++;
    build_data_xfer_req(req_buf, USB_INT_TRANSFER_REQ, dev, ioreq, endpoint, length, req_id);
    send_xfer_request(dev, ioreq, req_buf, 16,
                       USB_INT_TRANSFER_RES, is_in, length);
}

static void process_async_bulk_xfer(struct UsbDevice *dev, struct IOUsbHWReq *ioreq)
{
    UBYTE req_buf[16];
    ULONG req_id;
    UBYTE is_in = (ioreq->iouh_Dir == 2);
    UBYTE endpoint = (UBYTE)(ioreq->iouh_Endpoint | (is_in ? 0x80 : 0x00));
    UWORD length = (UWORD)ioreq->iouh_Length;

    if (!is_in && length > 0 && ioreq->iouh_Data)
    {
        if (length > dev->dataBufferSize)
        {
            ioreq->iouh_Req.io_Error = UHIOERR_BADPARAMS;
            ReplyMsg(&ioreq->iouh_Req.io_Message);
            return;
        }
        WriteMemA314(dev->dataBufferAddr, ioreq->iouh_Data, length);
    }

    req_id = dev->next_req_id++;
    build_data_xfer_req(req_buf, USB_BULK_TRANSFER_REQ, dev, ioreq, endpoint, length, req_id);
    send_xfer_request(dev, ioreq, req_buf, 16,
                       USB_BULK_TRANSFER_RES, is_in, length);
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
