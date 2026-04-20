/*
 * USB Bridge Protocol Definitions
 *
 * Protocol for bridging USB hardware between Poseidon (Amiga) and libusb (host)
 * via the A314 communication layer.
 *
 * All multi-byte fields use big-endian (network) byte order.
 */

#ifndef USB_PROTOCOL_H
#define USB_PROTOCOL_H

#include <exec/types.h>

// Message Types (first byte of MSG_DATA payload)
#define USB_ROOT_HUB_STATUS_REQ    1
#define USB_ROOT_HUB_STATUS_RES    2
#define USB_DEVICE_ATTACHED        3
#define USB_DEVICE_DETACHED        4
#define USB_CONTROL_TRANSFER_REQ   5
#define USB_CONTROL_TRANSFER_RES   6
#define USB_BULK_TRANSFER_REQ      7
#define USB_BULK_TRANSFER_RES      8
#define USB_INT_TRANSFER_REQ       9
#define USB_INT_TRANSFER_RES      10
#define USB_RESET_DEVICE_REQ      11
#define USB_RESET_DEVICE_RES      12
#define USB_ABORT_TRANSFER_REQ    13
#define USB_ABORT_TRANSFER_RES    14

// Transfer Status Codes
#define USB_STATUS_OK              0
#define USB_STATUS_STALL           1
#define USB_STATUS_NAK_TIMEOUT     2
#define USB_STATUS_ERROR           3

// Device Speed Codes
#define USB_SPEED_FULL             0
#define USB_SPEED_LOW              1
#define USB_SPEED_HIGH             2

// Port Status Flags
#define USB_PORT_CONNECTION        0x0001
#define USB_PORT_ENABLE            0x0002
#define USB_PORT_SUSPEND           0x0004
#define USB_PORT_OVER_CURRENT      0x0008
#define USB_PORT_RESET             0x0010
#define USB_PORT_POWER             0x0100
#define USB_PORT_LOW_SPEED         0x0200
#define USB_PORT_HIGH_SPEED        0x0400

// Port Change Flags
#define USB_PORT_C_CONNECTION      0x0001
#define USB_PORT_C_ENABLE          0x0002
#define USB_PORT_C_SUSPEND         0x0004
#define USB_PORT_C_OVER_CURRENT    0x0008
#define USB_PORT_C_RESET           0x0010

#endif
