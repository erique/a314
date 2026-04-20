/*
 * Poseidon command helpers (only QueryDevice is non-trivial)
 */

#include <exec/types.h>
#include <exec/errors.h>
#include <utility/tagitem.h>
#include <proto/exec.h>
#include <proto/utility.h>

#include "device.h"
#include "usbhw.h"
#include "debug.h"

#define SysBase (*(struct ExecBase **)4)

// Need UtilityBase in scope for tag functions (FindTagItem/GetTagData).
static struct UsbDevice *global_usb_dev = NULL;
#define UtilityBase (global_usb_dev->utility_base)

static const char manufacturer[] = "A314 Project";
static const char product_name[] = "A314 USB Bridge";
static const char description[] = "USB hardware bridge via A314 protocol";
static const char copyright[] = "\xa9 2026 A314 Project";

LONG cmdQueryDevice(struct IOUsbHWReq *ioreq, ULONG unit, struct UsbDevice *base)
{
    struct TagItem *tags = (struct TagItem *)ioreq->iouh_Data;
    struct TagItem *ti;
    ULONG count = 0;

    global_usb_dev = base;

    if (!tags)
    {
        ioreq->iouh_Req.io_Error = UHIOERR_BADPARAMS;
        return -1;
    }

    for (ti = tags; ti->ti_Tag != TAG_END; ti++)
    {
        switch (ti->ti_Tag)
        {
            case UHA_Manufacturer:
                *((STRPTR *)ti->ti_Data) = (STRPTR)manufacturer;
                count++;
                break;
            case UHA_ProductName:
                *((STRPTR *)ti->ti_Data) = (STRPTR)product_name;
                count++;
                break;
            case UHA_Description:
                *((STRPTR *)ti->ti_Data) = (STRPTR)description;
                count++;
                break;
            case UHA_Copyright:
                *((STRPTR *)ti->ti_Data) = (STRPTR)copyright;
                count++;
                break;
            case UHA_Version:
                *((ULONG *)ti->ti_Data) = DEVICE_VERSION;
                count++;
                break;
            case UHA_Revision:
                *((ULONG *)ti->ti_Data) = DEVICE_REVISION;
                count++;
                break;
            case UHA_DriverVersion:
                *((ULONG *)ti->ti_Data) = (DEVICE_VERSION << 8) | DEVICE_REVISION;
                count++;
                break;
            case UHA_State:
                *((ULONG *)ti->ti_Data) = ioreq->iouh_State;
                count++;
                break;
            default:
                break;
        }
    }

    return 0;
}
