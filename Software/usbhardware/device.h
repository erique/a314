/*
 * a314usb.device - USB Hardware driver for Poseidon via A314
 */

#ifndef A314USB_DEVICE_H
#define A314USB_DEVICE_H

#include <exec/types.h>
#include <exec/libraries.h>
#include <exec/io.h>
#include <libraries/dos.h>
#include <utility/tagitem.h>

// Device information
#define DEVICE_NAME             "a314usb.device"
#define DEVICE_VERSION          1
#define DEVICE_REVISION         0

// USB Hardware Commands (Poseidon standard)
#define UHCMD_QUERYDEVICE       (CMD_NONSTD + 0)  // 9
#define UHCMD_USBRESET          (CMD_NONSTD + 1)  // 10
#define UHCMD_USBRESUME         (CMD_NONSTD + 2)  // 11
#define UHCMD_USBSUSPEND        CMD_STOP          // 6
#define UHCMD_USBOPER           CMD_START         // 7
#define UHCMD_CONTROLXFER       (CMD_NONSTD + 3)  // 12
#define UHCMD_ISOXFER           (CMD_NONSTD + 4)  // 13
#define UHCMD_INTXFER           (CMD_NONSTD + 5)  // 14
#define UHCMD_BULKXFER          (CMD_NONSTD + 6)  // 15

// Error codes
#define UHIOERR_NO_ERROR        0
#define UHIOERR_USBOFFLINE      1
#define UHIOERR_NAK             2
#define UHIOERR_HOSTERROR       3
#define UHIOERR_STALL           4
#define UHIOERR_TIMEOUT         6
#define UHIOERR_NAKTIMEOUT     10
#define UHIOERR_BADPARAMS      11
#define UHIOERR_OUTOFMEMORY   12

// iouh_State bits
#define UHSB_OPERATIONAL        0
#define UHSB_RESUMING           1
#define UHSB_SUSPENDED          2
#define UHSB_RESET              3

#define UHSF_OPERATIONAL        (1<<UHSB_OPERATIONAL)
#define UHSF_RESUMING           (1<<UHSB_RESUMING)
#define UHSF_SUSPENDED          (1<<UHSB_SUSPENDED)
#define UHSF_RESET              (1<<UHSB_RESET)

// Tags for UHCMD_QUERYDEVICE
#define UHA_Dummy               (TAG_USER + 0x4711)
#define UHA_State               (UHA_Dummy + 0x01)
#define UHA_Manufacturer        (UHA_Dummy + 0x10)
#define UHA_ProductName         (UHA_Dummy + 0x11)
#define UHA_Version             (UHA_Dummy + 0x12)
#define UHA_Revision            (UHA_Dummy + 0x13)
#define UHA_Description         (UHA_Dummy + 0x14)
#define UHA_Copyright           (UHA_Dummy + 0x15)
#define UHA_DriverVersion       (UHA_Dummy + 0x20)

// USB Setup Data structure
struct UsbSetupData
{
    UBYTE bmRequestType;
    UBYTE bRequest;
    UWORD wValue;    // little-endian (USB wire order)
    UWORD wIndex;    // little-endian
    UWORD wLength;   // little-endian
};

// Poseidon IOUsbHWReq structure
struct IOUsbHWReq
{
    struct IORequest iouh_Req;
    UWORD iouh_Flags;
    UWORD iouh_State;
    UWORD iouh_Dir;
    UWORD iouh_DevAddr;
    UWORD iouh_Endpoint;
    UWORD iouh_MaxPktSize;
    ULONG iouh_Actual;
    ULONG iouh_Length;
    APTR  iouh_Data;
    UWORD iouh_Interval;
    ULONG iouh_NakTimeout;
    struct UsbSetupData iouh_SetupData;
    APTR  iouh_UserData;
    UWORD iouh_ExtError;
};

// Forward declarations
struct A314_IORequest;
struct MsgPort;
struct Task;
struct MinList;

// Device structure
struct UsbDevice
{
    struct Library lib;
    BPTR saved_seg_list;
    struct Library *utility_base;

    // A314 communication
    struct Library *a314_base;
    struct MsgPort *a314_mp;          // read completion port
    struct MsgPort *write_mp;         // write completion port
    struct A314_IORequest *read_ior;  // async reads
    struct A314_IORequest *write_ior; // sync writes
    LONG a314_socket;

    // Device task
    struct Task *task;                // init task
    struct Task *device_task;         // background task
    struct MsgPort *op_req_mp;        // Poseidon -> device task
    BOOL task_running;

    // Async task infrastructure
    APTR task_stack;
    ULONG next_req_id;

    // A314 communication state
    BOOL connected;
    BOOL pending_read;
    UBYTE res_msg_buf[256];

    // Current in-flight transfer
    struct IOUsbHWReq *current_ioreq;
    UBYTE current_resp_type;
    UBYTE current_is_in;
    UWORD current_max_len;

    // Shared memory data buffer
    ULONG dataBufferAddr;
    ULONG dataBufferSize;
};

// Globals
extern const char device_name[];
extern const char id_string[];
extern ULONG auto_init_tables[];
extern UWORD usb_state;

#endif
