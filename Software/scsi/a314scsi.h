#ifndef A314SCSI_H
#define A314SCSI_H

#include <exec/types.h>
#include <exec/devices.h>
#include <exec/io.h>
#include <exec/ports.h>
#include <exec/tasks.h>
#include <exec/lists.h>
#include <exec/libraries.h>
#include <exec/interrupts.h>

#include <dos/dos.h> // BPTR
#include <dos/dosextens.h> // struct Process
#include <dos/filehandler.h> // struct DeviceNode

#include <devices/scsidisk.h> // struct SCSICmd
#include <devices/timer.h> // struct timerequest

#include "../a314device/a314.h" // struct A314_IORequest
#include "a314scsi_protocol.h" // struct A314ScsiRes

#define DEVICE_NAME "a314scsi.device"
#define DEVICE_VERSION 1
#define DEVICE_REVISION 0

// How many drive units the host can serve (unit number -> HDF).
#define A314SCSI_MAX_DRIVES 4

// Amiga unit numbering follows the scsi.device convention: unit = LUN*10 + ID.
// Each host disk is exposed as its own LUN (ID 0), so valid units are 0, 10,
// 20, ... - which is exactly what HDToolBox probes.
#define A314SCSI_UNIT_STEP 10

#define TASK_PRIORITY 10
#define TASK_STACK_SIZE 4096

// Largest single a314 transfer == the a314-memory bounce buffer.
#define A314SCSI_MAX_XFER (4UL * 1024)

// Every unit is removable: the host's hdf can be removed or replaced.
#define ALL_UNITS_MASK ((1UL << A314SCSI_MAX_DRIVES) - 1)

// Drive flags.
#define DRVF_PRESENT (1 << 0)
#define DRVF_WRITEPROT (1 << 1)
#define DRVF_REMOVABLE (1 << 2)
#define DRVF_MOUNTED (1 << 3)

struct Drive
{
    UWORD           number;         // unit number
    UWORD           opencount;
    UWORD           flags;          // DRVF_*
    UBYTE           devtype;        // INQUIRY peripheral device type
    UBYTE           blockshift;     // log2(blocksize)
    ULONG           blocksize;
    ULONG           blocks;         // total block count
    UBYTE           sense[24];      // last auto/request sense

    // Medium changes (TD_CHANGENUM, TD_ADDCHANGEINT, TD_REMOVE)
    ULONG           changecount;
    struct MinList  changeints;     // TD_ADDCHANGEINT requests, io_Data = Interrupt
    struct Interrupt* removeint;    // TD_REMOVE
};

struct A314ScsiBase
{
    struct Device           device;
    struct ExecBase*        sysbase;
    BPTR                    seglist;

    struct Drive*           drives[A314SCSI_MAX_DRIVES];
    struct ConfigDev*       configdev;      // makes the BootNodes bootable (see romtag.asm)

    struct MsgPort*         reqport;        // command queue to the io task
    struct Task             iotask;         // embedded io task (tc_UserData -> base)
    // The io task calls CreateNewProc() (see start_mounter), and dos.library
    // may write the caller's Process fields, so it gets room for them.
    UBYTE                   iotaskpad[sizeof(struct Process) - sizeof(struct Task)];
    APTR                    stack;

    // a314 transport (opened and used only by the io task)
    struct MsgPort*         a314port;       // replies of a314ior, rstior and tmotr
    struct A314_IORequest   a314ior;        // CONNECT and WRITE
    struct A314_IORequest   readior;        // the READ, outstanding while connected
    struct A314_IORequest   rstior;         // resets the socket while others are out
    struct A314ScsiRes      rbuf;           // what readior reads: a response or an event
    UBYTE                   readpending;
    struct timerequest      tmotr;          // a314 request timeouts
    struct Library*         a314base;
    ULONG                   socket;
    ULONG                   bounce;         // a314-space bounce buffer
    UBYTE                   connected;
    UBYTE                   stuck;          // a314.device stopped replying

    // Reconnect timer, running while the link is down
    struct timerequest      rescantr;
    UBYTE                   rescanpending;

    // Units to ask about their medium: from host events (changedunits) and
    // from UNIT ATTENTION seen on a command (uaunits)
    UBYTE                   changedunits;
    UBYTE                   uaunits;

    // io-task startup handshake back to init_device
    struct Task*            inittask;
    ULONG                   initsig;
    volatile UBYTE          started;
};

// romtag.asm
extern UBYTE diag_area[];

// device.c
extern const char device_name[];
struct Drive* find_drive(struct A314ScsiBase* base, UWORD number);
struct Drive* make_drive(struct A314ScsiBase* base, UWORD number);

// scsicmd.c
LONG scsi_test_ready(struct A314ScsiBase* base, struct Drive* d);
LONG scsi_inquiry(struct A314ScsiBase* base, struct Drive* d);
LONG scsi_read_capacity(struct A314ScsiBase* base, struct Drive* d);
LONG scsi_mode_sense_wp(struct A314ScsiBase* base, struct Drive* d);
LONG scsi_rw(struct A314ScsiBase* base, struct Drive* d, ULONG lba, ULONG blocks, APTR buf, BOOL write);

// a314hw.c
BOOL a314_open_device(struct A314ScsiBase* base);
BOOL a314_connect(struct A314ScsiBase* base);
void a314_poll(struct A314ScsiBase* base);
LONG a314_scsi(struct A314ScsiBase* base, struct Drive* d, struct SCSICmd* cmd);

// cli.c
void mount_boot_nodes(struct Library* ExpansionBase);

// rdb.c
BOOL mount_drive(struct A314ScsiBase* base, struct Drive* d);
void free_partition(struct DeviceNode* dn);

// lseg.c
BPTR load_seglist(struct A314ScsiBase* base, struct Drive* d, ULONG first_block);

#endif
