#ifndef USBHW_H
#define USBHW_H

#include "device.h"

LONG cmdQueryDevice(struct IOUsbHWReq *ioreq, ULONG unit, struct UsbDevice *base);

#endif
