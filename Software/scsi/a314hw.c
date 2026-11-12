#include <exec/types.h>
#include <exec/memory.h>
#include <exec/io.h>
#include <exec/errors.h>

#include <dos/dos.h> // TICKS_PER_SECOND

#include <devices/timer.h>
#include <devices/scsidisk.h>

#include <proto/exec.h>


#include <string.h>

#include "../a314device/a314.h"
#include "../a314device/proto_a314.h"

#include "a314scsi.h"
#include "a314scsi_protocol.h"
#include "scsi.h"
#include "alib.h"
#include "kprintf.h"

#define SysBase (*(struct ExecBase**)4)

// Redirect the proto_a314.h helper macros to the per-device library base.
#undef A314Base
#define A314Base (base->a314base)

// A host that stops answering must not hang the io task: every a314 request
// is timed, and the socket is reset when one runs out. A request that fails
// marks the link down; the next one reconnects, on a new socket id, as a
// socket can stay in a314.device until its reset has gone out.
#define REQ_TIMEOUT_SECS        10
#define CONNECT_TIMEOUT_SECS    3
#define RESET_TIMEOUT_SECS      2
#define LINK_TIMEOUT            (-1)

static const char service_name[] = "scsi";
static const char a314_name[] = A314_NAME;

static ULONG get_socket_id(void)
{
    struct timerequest tr;

    memset(&tr, 0, sizeof(tr));
    tr.tr_node.io_Message.mn_Node.ln_Type = NT_REPLYMSG;

    if (OpenDevice((STRPTR)TIMERNAME, UNIT_VBLANK, (struct IORequest*)&tr, 0))
    {
        return 0x53435349UL; // 'SCSI'
    }

    tr.tr_node.io_Command = TR_GETSYSTIME;
    DoIO((struct IORequest*)&tr);
    CloseDevice((struct IORequest*)&tr);

    return (tr.tr_time.tv_secs * TICKS_PER_SECOND) ^ 0x53435349UL;
}

// Open a314.device and reserve resources. This MUST run in init_device's caller
// context (a normal process), NOT the freshly-created io task: opening the device
// from a just-added task races with a314.device's own startup and hangs.
BOOL a314_open_device(struct A314ScsiBase* base)
{
    // OpenDevice needs a reply port, but the io task owns the real one; use a
    // throwaway port here and let a314_connect() install the io task's port.
    struct MsgPort* tmp = CreatePort(NULL, 0);
    if (!tmp)
    {
        return FALSE;
    }

    memset(&base->a314ior, 0, sizeof(base->a314ior));
    base->a314ior.a314_Request.io_Message.mn_ReplyPort = tmp;
    base->a314ior.a314_Request.io_Message.mn_Length = sizeof(base->a314ior);

    if (OpenDevice((STRPTR)a314_name, 0, &base->a314ior.a314_Request, 0))
    {
        kprintf("a314scsi: OpenDevice(a314.device) FAILED\n");
        DeletePort(tmp);
        return FALSE;
    }
    base->a314base = &base->a314ior.a314_Request.io_Device->dd_Library;

    DeletePort(tmp);

    base->socket = get_socket_id();

    base->bounce = AllocMemA314(A314SCSI_MAX_XFER);
    if (base->bounce == INVALID_A314_ADDRESS)
    {
        kprintf("a314scsi: AllocMemA314 FAILED\n");
        CloseDevice(&base->a314ior.a314_Request);
        base->a314base = NULL;
        return FALSE;
    }

    memset(&base->tmotr, 0, sizeof(base->tmotr));
    base->tmotr.tr_node.io_Message.mn_Length = sizeof(base->tmotr);
    if (OpenDevice((STRPTR)TIMERNAME, UNIT_VBLANK, (struct IORequest*)&base->tmotr, 0))
    {
        kprintf("a314scsi: OpenDevice(timer.device) FAILED\n");
        FreeMemA314(base->bounce, A314SCSI_MAX_XFER);
        CloseDevice(&base->a314ior.a314_Request);
        base->a314base = NULL;
        return FALSE;
    }

    base->rstior = base->a314ior;

    return TRUE;
}

// Wait for io, at most secs seconds. FALSE: it is still outstanding.
static BOOL wait_io_timed(struct A314ScsiBase* base, struct IORequest* io, ULONG secs)
{
    struct IORequest* tmo = (struct IORequest*)&base->tmotr;
    tmo->io_Command = TR_ADDREQUEST;
    base->tmotr.tr_time.tv_secs = secs;
    base->tmotr.tr_time.tv_micro = 0;
    SendIO(tmo);

    for (;;)
    {
        if (CheckIO(io))
        {
            WaitIO(io);
            AbortIO(tmo);
            WaitIO(tmo);
            return TRUE;
        }
        if (CheckIO(tmo))
        {
            WaitIO(tmo);
            return FALSE;
        }
        Wait(1UL << base->a314port->mp_SigBit);
    }
}

// Reset the socket from rstior, so neither side keeps it. a314.device then
// replies a314ior too, if it is outstanding (iorpending).
static void reset_socket(struct A314ScsiBase* base, BOOL iorpending)
{
    base->rstior.a314_Request.io_Command = A314_RESET;
    base->rstior.a314_Request.io_Error = 0;
    base->rstior.a314_Socket = base->socket;
    base->rstior.a314_Buffer = NULL;
    base->rstior.a314_Length = 0;
    SendIO(&base->rstior.a314_Request);

    BOOL rstback = wait_io_timed(base, &base->rstior.a314_Request, RESET_TIMEOUT_SECS);
    BOOL iorback = !iorpending || wait_io_timed(base, &base->a314ior.a314_Request, RESET_TIMEOUT_SECS);
    if (!rstback || !iorback)
    {
        // The requests may still be replied later, so they are never reused.
        kprintf("a314scsi: a314.device does not reply; link disabled\n");
        base->stuck = 1;
    }
}

// Send an a314 command on the socket and wait for it, at most secs seconds.
// Returns the a314 io_Error, or LINK_TIMEOUT (the socket has then been reset).
static LONG a314_cmd(struct A314ScsiBase* base, UWORD command, APTR buffer, ULONG length, ULONG secs)
{
    if (base->stuck)
    {
        return LINK_TIMEOUT;
    }

    base->a314ior.a314_Request.io_Command = command;
    base->a314ior.a314_Request.io_Error = 0;
    base->a314ior.a314_Socket = base->socket;
    base->a314ior.a314_Buffer = (STRPTR)buffer;
    base->a314ior.a314_Length = length;
    SendIO(&base->a314ior.a314_Request);

    if (wait_io_timed(base, &base->a314ior.a314_Request, secs))
    {
        return base->a314ior.a314_Request.io_Error;
    }

    kprintf("a314scsi: a314 command %ld timed out\n", (ULONG)command);
    reset_socket(base, TRUE);
    return LINK_TIMEOUT;
}

// Connect to the host 'scsi' service. Runs in the io task, which owns the reply
// port used for this and every subsequent a314 request.
BOOL a314_connect(struct A314ScsiBase* base)
{
    if (!base->a314port)
    {
        base->a314port = CreatePort(NULL, 0);
        if (!base->a314port)
        {
            return FALSE;
        }
        base->a314ior.a314_Request.io_Message.mn_ReplyPort = base->a314port;
        base->rstior.a314_Request.io_Message.mn_ReplyPort = base->a314port;
        base->tmotr.tr_node.io_Message.mn_ReplyPort = base->a314port;
    }

    base->socket++;

    LONG err = a314_cmd(base, A314_CONNECT, (APTR)service_name, strlen(service_name), CONNECT_TIMEOUT_SECS);
    if (err != A314_CONNECT_OK)
    {
        kprintf("a314scsi: CONNECT failed err=%ld\n", err);
        return FALSE;
    }

    base->connected = 1;
    kprintf("a314scsi: connected to 'scsi' (socket=%lx)\n", base->socket);
    return TRUE;
}

// One request/response round trip. On failure the link is marked down, and
// the socket is reset unless the host or a timeout already did.
static LONG a314_txn(struct A314ScsiBase* base, struct A314ScsiReq* req, struct A314ScsiRes* res)
{
    LONG err = a314_cmd(base, A314_WRITE, req, sizeof(struct A314ScsiReq), REQ_TIMEOUT_SECS);
    if (err == A314_WRITE_OK)
    {
        err = a314_cmd(base, A314_READ, res, sizeof(struct A314ScsiRes), REQ_TIMEOUT_SECS);
        if (err == A314_READ_OK)
        {
            return 0;
        }
        if (err != A314_READ_RESET && err != LINK_TIMEOUT)
        {
            reset_socket(base, FALSE);
        }
    }
    else if (err != A314_WRITE_RESET && err != LINK_TIMEOUT)
    {
        reset_socket(base, FALSE);
    }

    kprintf("a314scsi: txn err=%ld\n", err);
    base->connected = 0;
    return IOERR_ABORTED;
}

LONG a314_scsi(struct A314ScsiBase* base, struct Drive* d, struct SCSICmd* cmd)
{
    struct A314ScsiReq req;
    struct A314ScsiRes res;

    if (cmd->scsi_Length > A314SCSI_MAX_XFER)
    {
        return IOERR_ABORTED; // caller must chunk
    }

    UBYTE dir;
    if (cmd->scsi_Length == 0)
    {
        dir = A314SCSI_DIR_NONE;
    }
    else if (cmd->scsi_Flags & SCSIF_READ)
    {
        dir = A314SCSI_DIR_READ;
    }
    else
    {
        dir = A314SCSI_DIR_WRITE;
    }

    // Resolve the a314-space address of the caller buffer, else use the bounce.
    ULONG addr = 0;
    BOOL bounced = FALSE;
    if (cmd->scsi_Length)
    {
        addr = TranslateAddressA314((APTR)cmd->scsi_Data);
        if (addr == INVALID_A314_ADDRESS)
        {
            addr = base->bounce;
            bounced = TRUE;
            if (dir == A314SCSI_DIR_WRITE)
            {
                WriteMemA314(base->bounce, (APTR)cmd->scsi_Data, cmd->scsi_Length);
            }
        }
    }

    memset(&req, 0, sizeof(req));
    req.kind = A314SCSI_CMD_REQ;
    req.unit = d ? (UBYTE)d->number : 0;
    req.cdb_len = (UBYTE)cmd->scsi_CmdLength;
    if (req.cdb_len > A314SCSI_MAX_CDB)
    {
        req.cdb_len = A314SCSI_MAX_CDB;
    }
    memcpy(req.cdb, cmd->scsi_Command, req.cdb_len);
    req.direction = dir;
    req.data_length = cmd->scsi_Length;
    req.address = addr;

    // A lost link is reconnected, and the command sent once more: the host
    // may have restarted, and a READ or WRITE can safely be repeated.
    LONG err = IOERR_ABORTED;
    for (int attempt = 0; attempt < 2 && err; attempt++)
    {
        if (!base->connected && !a314_connect(base))
        {
            return IOERR_ABORTED;
        }
        err = a314_txn(base, &req, &res);
    }
    if (err)
    {
        return err;
    }

    if (res.kind != A314SCSI_CMD_RES)
    {
        kprintf("a314scsi: bad res kind=%ld\n", (ULONG)res.kind);
        return IOERR_ABORTED;
    }

    cmd->scsi_Status = res.scsi_status;
    cmd->scsi_Actual = (res.actual_length > cmd->scsi_Length) ? cmd->scsi_Length : res.actual_length;

    if (dir == A314SCSI_DIR_READ && bounced && res.actual_length)
    {
        ULONG n = res.actual_length;
        if (n > cmd->scsi_Length)
        {
            n = cmd->scsi_Length;
        }
        ReadMemA314((APTR)cmd->scsi_Data, base->bounce, n);
    }

    if (res.scsi_status != SCSI_STATUS_GOOD)
    {
        if (cmd->scsi_SenseData && cmd->scsi_SenseLength >= 14)
        {
            memset(cmd->scsi_SenseData, 0, cmd->scsi_SenseLength);
            cmd->scsi_SenseData[0] = SCSI_SENSE_CURRENT_FIXED;
            cmd->scsi_SenseData[2] = res.sense_key;
            cmd->scsi_SenseData[7] = SCSI_SENSE_ADD_LEN;
            cmd->scsi_SenseData[12] = res.asc;
            cmd->scsi_SenseData[13] = res.ascq;
            cmd->scsi_SenseActual = 14;
        }
        return HFERR_BadStatus;
    }

    return 0;
}
