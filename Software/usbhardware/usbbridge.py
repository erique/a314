#!/usr/bin/env python3
"""
USB Bridge Service - Host side

Bridges USB hardware between Poseidon (Amiga) and libusb (host) via A314.
Control and bulk transfers: the Amiga allocates a buffer in A314 shared
memory and sends its address along with each request. The host reads/writes
that buffer via read_mem/write_mem for IN/OUT data, and sends back a short
status response via MSG_DATA. The Amiga sends one at a time.

Interrupt transfers: the data travels in the request (OUT) and the response
(IN). An IN transfer stays pending until the device has data, as on real USB,
so several can be pending at once; the Amiga cancels one with
USB_ABORT_TRANSFER_REQ.
"""

import sys
import os
import errno
import signal
import select
import threading
import collections
import struct
import json
import logging
import time
import usb.core
import usb.util

from a314d import A314d

from usb_protocol import *

logging.basicConfig(
    format='%(levelname)s, %(asctime)s, %(name)s, line %(lineno)d: %(message)s',
    level=logging.INFO,
    handlers=[
        logging.FileHandler('/tmp/usbbridge.log'),
        logging.StreamHandler(),
    ],
)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)  # DEBUG for per-transfer logging

SERVICE_NAME = 'usbhw'
CONFIG_FILE = '/etc/opt/a314/usbbridge.conf'

# Standard USB requests
USB_REQ_GET_STATUS = 0x00
USB_REQ_CLEAR_FEATURE = 0x01
USB_REQ_SET_FEATURE = 0x03
USB_REQ_SET_ADDRESS = 0x05
USB_REQ_GET_DESCRIPTOR = 0x06
USB_REQ_SET_CONFIGURATION = 0x09

USB_DT_DEVICE = 0x01
USB_DT_CONFIG = 0x02
USB_DT_STRING = 0x03
USB_DT_INTERFACE = 0x04
USB_DT_ENDPOINT = 0x05
USB_DT_HUB = 0x29

# Hub port features
USB_PORT_FEAT_CONNECTION = 0
USB_PORT_FEAT_ENABLE = 1
USB_PORT_FEAT_SUSPEND = 2
USB_PORT_FEAT_OVER_CURRENT = 3
USB_PORT_FEAT_RESET = 4
USB_PORT_FEAT_POWER = 8
USB_PORT_FEAT_LOWSPEED = 9
USB_PORT_FEAT_C_CONNECTION = 16
USB_PORT_FEAT_C_ENABLE = 17
USB_PORT_FEAT_C_SUSPEND = 18
USB_PORT_FEAT_C_OVER_CURRENT = 19
USB_PORT_FEAT_C_RESET = 20

# Packet size to round IN reads to when the Amiga doesn't give one
DEFAULT_MAX_PACKET = 64

# An interrupt reader's read timeout; a stop request waits for it at most
INT_READ_TIMEOUT_MS = 500
# Reports an interrupt reader holds for the Amiga; older ones are dropped
INT_QUEUE_MAX = 64

# How long attached devices show as disconnected when a client connects
REPLUG_DELAY_S = 1.0
# How often the host's USB devices are checked for changes
SCAN_INTERVAL_S = 2.0
# The longest the main loop waits for a message
POLL_INTERVAL_S = 0.1

# CLEAR_FEATURE(ENDPOINT_HALT): host-to-device, standard, endpoint recipient
USB_RT_ENDPOINT_OUT = 0x02
USB_FEAT_ENDPOINT_HALT = 0


def in_read_length(length, max_pkt):
    """Round an IN read up to whole packets. A device may always send a full
    packet, and a read buffer smaller than that overruns."""
    if max_pkt <= 0:
        max_pkt = DEFAULT_MAX_PACKET
    return max(max_pkt, -(-length // max_pkt) * max_pkt)


class InterruptReader:
    """Keeps a read pending on one interrupt IN endpoint and queues what
    arrives, so the Amiga's polls are answered at once and a report that
    arrives between them isn't lost."""

    def __init__(self, dev, endpoint, max_pkt, wake_fd):
        self.dev = dev
        self.endpoint = endpoint
        self.max_pkt = max_pkt
        self.wake_fd = wake_fd
        self.queue = collections.deque(maxlen=INT_QUEUE_MAX)
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stopping.is_set():
            try:
                data = bytes(self.dev.read(self.endpoint, self.max_pkt,
                                           timeout=INT_READ_TIMEOUT_MS))
                self.queue.append((USB_STATUS_OK, data))
                self.wake()
            except usb.core.USBTimeoutError:
                pass
            except usb.core.USBError as e:
                if e.errno == errno.EPIPE:
                    self.queue.append((USB_STATUS_STALL, b''))
                else:
                    self.queue.append((USB_STATUS_ERROR, b''))
                self.wake()
                if e.errno == errno.ENODEV:
                    return
                # Wait for the Amiga to clear the halt instead of spinning
                self.stopping.wait(INT_READ_TIMEOUT_MS / 1000)

    def wake(self):
        """Wake the main loop, which answers the pending transfers."""
        try:
            os.write(self.wake_fd, b'\0')
        except BlockingIOError:
            pass    # already woken

    def get(self):
        """The next (status, data) report, or None if there is none."""
        try:
            return self.queue.popleft()
        except IndexError:
            return None

    def stop(self):
        self.stopping.set()
        self.thread.join()


def str_desc(s):
    """Build a USB string descriptor from a Python str."""
    encoded = s.encode('utf-16-le')
    return bytes([len(encoded) + 2, 0x03]) + encoded


class RootHub:
    """Emulates a 4-port USB hub at device address 1."""

    def __init__(self, port_count=4):
        self.port_count = port_count
        # port_status[0] unused; ports are 1..port_count
        self.port_status = [0] * (port_count + 1)
        self.port_change = [0] * (port_count + 1)
        self.devices = {}  # port -> device

        # Power up all ports
        for i in range(1, port_count + 1):
            self.port_status[i] = USB_PORT_POWER

    def get_device_descriptor(self):
        return bytes([
            0x12,        # bLength
            USB_DT_DEVICE,
            0x10, 0x01,  # bcdUSB 1.10
            0x09,        # bDeviceClass = Hub
            0x00, 0x00,  # subclass, protocol
            0x08,        # bMaxPacketSize0
            0x27, 0x06,  # idVendor (0x0627)
            0x00, 0x00,  # idProduct
            0x00, 0x01,  # bcdDevice
            0x01,        # iManufacturer
            0x02,        # iProduct
            0x00,        # iSerial
            0x01,        # bNumConfigurations
        ])

    def get_config_descriptor(self):
        config = bytes([
            0x09, USB_DT_CONFIG,
            0x19, 0x00,  # wTotalLength = 25
            0x01,        # bNumInterfaces
            0x01,        # bConfigurationValue
            0x00,        # iConfiguration
            0xC0,        # bmAttributes (self-powered)
            0x00,        # bMaxPower
        ])
        interface = bytes([
            0x09, USB_DT_INTERFACE,
            0x00, 0x00,  # interface/alt
            0x01,        # bNumEndpoints
            0x09,        # bInterfaceClass (Hub)
            0x00, 0x00,  # subclass, protocol
            0x00,        # iInterface
        ])
        endpoint = bytes([
            0x07, USB_DT_ENDPOINT,
            0x81,        # EP1 IN
            0x03,        # Interrupt
            0x01, 0x00,  # wMaxPacketSize=1
            0xFF,        # bInterval
        ])
        return config + interface + endpoint

    def get_hub_descriptor(self):
        return bytes([
            0x09, USB_DT_HUB,
            self.port_count,
            0x00, 0x00,  # wHubCharacteristics
            0x32,        # bPwrOn2PwrGood
            0x00,        # bHubContrCurrent
            0x00, 0xFF,  # device removable, port power mask
        ])

    def get_string_descriptor(self, index):
        table = {
            0: b'\x04\x03\x09\x04',  # Language: English (US)
            1: str_desc('A314 USB Bridge'),
            2: str_desc('A314 Virtual Hub'),
        }
        return table.get(index, b'\x00\x00')

    def handle_control_transfer(self, bmRT, bReq, wValue, wIndex, wLength):
        """Handle control request directed at the hub. Returns (data, status)."""
        req_type = (bmRT >> 5) & 0x03
        recipient = bmRT & 0x1F

        # Standard device requests
        if req_type == 0 and recipient == 0:
            if bReq == USB_REQ_GET_DESCRIPTOR:
                dt = (wValue >> 8) & 0xFF
                di = wValue & 0xFF
                if dt == USB_DT_DEVICE:
                    return (self.get_device_descriptor(), USB_STATUS_OK)
                elif dt == USB_DT_CONFIG:
                    return (self.get_config_descriptor(), USB_STATUS_OK)
                elif dt == USB_DT_STRING:
                    return (self.get_string_descriptor(di), USB_STATUS_OK)
            elif bReq == USB_REQ_SET_CONFIGURATION:
                return (b'', USB_STATUS_OK)
            elif bReq == USB_REQ_GET_STATUS:
                return (bytes([0x01, 0x00]), USB_STATUS_OK)

        # Class hub requests
        elif req_type == 1 and recipient == 0:
            if bReq == USB_REQ_GET_DESCRIPTOR:
                dt = (wValue >> 8) & 0xFF
                if dt == USB_DT_HUB:
                    return (self.get_hub_descriptor(), USB_STATUS_OK)
            elif bReq == USB_REQ_GET_STATUS:
                return (bytes([0, 0, 0, 0]), USB_STATUS_OK)

        # Class port requests (recipient = Other = 3)
        elif req_type == 1 and recipient == 3:
            port = wIndex & 0xFF
            if port < 1 or port > self.port_count:
                return (b'', USB_STATUS_STALL)
            if bReq == USB_REQ_GET_STATUS:
                data = struct.pack('<HH', self.port_status[port], self.port_change[port])
                return (data, USB_STATUS_OK)
            elif bReq == USB_REQ_SET_FEATURE:
                feat = wValue & 0xFF
                if feat == USB_PORT_FEAT_RESET:
                    self.port_status[port] |= USB_PORT_RESET
                    self.port_change[port] |= USB_PORT_C_RESET
                    # Simulate reset completion
                    self.port_status[port] &= ~USB_PORT_RESET
                    self.port_status[port] |= USB_PORT_ENABLE
                    return (b'', USB_STATUS_OK)
                elif feat == USB_PORT_FEAT_POWER:
                    self.port_status[port] |= USB_PORT_POWER
                    return (b'', USB_STATUS_OK)
                elif feat == USB_PORT_FEAT_ENABLE:
                    self.port_status[port] |= USB_PORT_ENABLE
                    return (b'', USB_STATUS_OK)
            elif bReq == USB_REQ_CLEAR_FEATURE:
                feat = wValue & 0xFF
                mapping = {
                    USB_PORT_FEAT_C_CONNECTION: USB_PORT_C_CONNECTION,
                    USB_PORT_FEAT_C_RESET: USB_PORT_C_RESET,
                    USB_PORT_FEAT_C_ENABLE: USB_PORT_C_ENABLE,
                    USB_PORT_FEAT_C_SUSPEND: USB_PORT_C_SUSPEND,
                    USB_PORT_FEAT_C_OVER_CURRENT: USB_PORT_C_OVER_CURRENT,
                }
                if feat in mapping:
                    self.port_change[port] &= ~mapping[feat]
                # Succeed silently on other clear-feature requests
                return (b'', USB_STATUS_OK)

        logger.debug(f"RootHub: unhandled bmRT=0x{bmRT:02x} bReq={bReq}")
        return (b'', USB_STATUS_STALL)

    def read_status_change(self):
        """Build the hub interrupt endpoint bitmap. Returns None if no changes."""
        bitmap = 0
        for port in range(1, self.port_count + 1):
            if self.port_change[port] != 0:
                bitmap |= (1 << port)
        if bitmap == 0:
            return None
        return bytes([bitmap])

    def device_attached(self, port, device):
        if port < 1 or port > self.port_count:
            return
        self.port_status[port] |= USB_PORT_CONNECTION
        self.port_change[port] |= USB_PORT_C_CONNECTION
        self.devices[port] = device
        logger.info(f"RootHub: device attached to port {port}")

    def device_detached(self, port):
        if port < 1 or port > self.port_count:
            return
        self.port_status[port] &= ~(USB_PORT_CONNECTION | USB_PORT_ENABLE)
        self.port_change[port] |= USB_PORT_C_CONNECTION
        if port in self.devices:
            del self.devices[port]
        logger.info(f"RootHub: device detached from port {port}")


class UsbBridgeService:
    def __init__(self):
        self.a314d = A314d(SERVICE_NAME)
        self.current_stream_id = None

        self.config = self.load_config()
        self.filters = self.config.get('filters', [{'vid': '*', 'pid': '*'}])
        self.port_count = self.config.get('port_count', 4)

        self.root_hub = RootHub(self.port_count)
        self.devices = self.root_hub.devices   # port -> libusb device
        self.dev_addr_map = {}                 # usb_addr -> (port, dev)
        self.enumerating_device = None         # (port, dev) being reset + assigned address

        self.next_dev_addr = 2

        self.claimed = {}       # libusb device -> claimed interface numbers
        self.detached = {}      # libusb device -> interfaces taken from their kernel driver
        self.int_readers = {}   # (libusb device, endpoint) -> InterruptReader

        # Pending interrupt IN transfers, oldest first: dicts with req_id,
        # dev_addr, endpoint and length
        self.pending_int = []

        # Messages that arrived while waiting for a memory read/write reply
        self.backlog = collections.deque()

        # Devices shown as disconnected after a client connected: port -> dev
        self.replug = {}
        self.replug_at = 0

        # The interrupt readers wake the main loop through this pipe
        self.wake_r, self.wake_w = os.pipe()
        os.set_blocking(self.wake_r, False)
        os.set_blocking(self.wake_w, False)

    def load_config(self):
        paths = [
            CONFIG_FILE,
            os.path.join(os.path.dirname(os.path.abspath(__file__)), CONFIG_FILE),
        ]
        for p in paths:
            if os.path.exists(p):
                try:
                    with open(p) as f:
                        return json.load(f)
                except Exception as e:
                    logger.warning(f"Could not read {p}: {e}")
        return {}

    # ---- filtering ----
    def matches_filter(self, vid, pid):
        for f in self.filters:
            fv = f.get('vid', '*')
            fp = f.get('pid', '*')
            if (fv == '*' or fv == f"{vid:04x}") and (fp == '*' or fp == f"{pid:04x}"):
                return True
        return False

    # ---- A314 memory ----
    # a314d's MSG_DATA can arrive while the reply to a memory read or write is
    # awaited, since the Amiga has interrupt transfers pending; those messages
    # are kept for the main loop.
    def wait_for_reply(self, ptype):
        while True:
            stream_id, t, payload = self.a314d.wait_for_msg()
            if t == ptype:
                return payload
            self.backlog.append((stream_id, t, payload))

    def read_mem(self, address, length):
        self.a314d.send_read_mem_req(address, length)
        return self.wait_for_reply(self.a314d.MSG_READ_MEM_RES)

    def write_mem(self, address, data):
        self.a314d.send_write_mem_req(address, data)
        self.wait_for_reply(self.a314d.MSG_WRITE_MEM_RES)

    # ---- claimed interfaces ----
    def release_device(self, dev, give_back=False):
        """Stop the interrupt readers, release the claimed interfaces and free
        libusb's resources, so that the device can be configured again. With
        give_back, the kernel drivers get their interfaces back."""
        if dev is None or hasattr(dev, 'handle_control_transfer'):
            return
        for key in [k for k in self.int_readers if k[0] is dev]:
            self.int_readers.pop(key).stop()
        for intf in self.claimed.pop(dev, []):
            try:
                usb.util.release_interface(dev, intf)
            except usb.core.USBError:
                pass
        if give_back:
            for intf in self.detached.pop(dev, []):
                try:
                    dev.attach_kernel_driver(intf)
                    logger.info(f"Gave intf {intf} back to its kernel driver")
                except (usb.core.USBError, NotImplementedError) as e:
                    logger.warning(f"Could not give intf {intf} back to its kernel driver: {e}")
        try:
            usb.util.dispose_resources(dev)
        except usb.core.USBError:
            pass

    def release_all_devices(self, give_back=False):
        for dev in list(self.devices.values()) + list(self.replug.values()):
            self.release_device(dev, give_back)

    def find_free_port(self):
        for port in range(1, self.port_count + 1):
            if port not in self.devices:
                return port
        return None

    # ---- device scan / hotplug ----
    def scan_usb_devices(self):
        try:
            for dev in usb.core.find(find_all=True):
                # Skip hubs (9) and application-specific (Billboard etc., 0xFE)
                if dev.bDeviceClass in (9, 0xFE):
                    continue

                vid_pid = (dev.idVendor, dev.idProduct)
                already = False
                for pd in self.devices.values():
                    if pd is None:
                        continue
                    try:
                        if (pd.idVendor, pd.idProduct) == vid_pid:
                            already = True
                            break
                    except Exception:
                        pass
                if already:
                    continue

                if not self.matches_filter(dev.idVendor, dev.idProduct):
                    continue

                port = self.find_free_port()
                if port is None:
                    logger.debug(f"No free ports for {dev.idVendor:04x}:{dev.idProduct:04x}")
                    continue

                self.devices[port] = dev
                self.root_hub.device_attached(port, dev)
                logger.info(f"Device attached port={port} vid={dev.idVendor:04x}:{dev.idProduct:04x}")
        except Exception as e:
            logger.error(f"scan_usb_devices: {e}")

    def check_device_changes(self):
        """Detect insertion/removal of real USB devices. Track by VID:PID since
        bus/address can change after detach_kernel_driver."""
        current_vidpids = {}
        try:
            for dev in usb.core.find(find_all=True):
                if dev.bDeviceClass in (9, 0xFE):
                    continue
                if self.matches_filter(dev.idVendor, dev.idProduct):
                    current_vidpids[(dev.idVendor, dev.idProduct)] = dev
        except Exception as e:
            logger.error(f"check_device_changes scan: {e}")
            return

        had_removal = False
        for port in list(self.devices.keys()):
            dev = self.devices[port]
            if hasattr(dev, 'handle_control_transfer'):
                continue  # virtual device
            try:
                key = (dev.idVendor, dev.idProduct)
                if key not in current_vidpids:
                    logger.info(f"Device removed from port {port}: {key[0]:04x}:{key[1]:04x}")
                    self.release_device(dev)
                    self.detached.pop(dev, None)
                    for addr, (p, d) in list(self.dev_addr_map.items()):
                        if p == port:
                            del self.dev_addr_map[addr]
                    self.root_hub.device_detached(port)
                    if port in self.devices:
                        del self.devices[port]
                    had_removal = True
            except Exception:
                pass

        if had_removal:
            # Let hub.class see the disconnect before inserting a new device
            return

        attached = set()
        for port, dev in self.devices.items():
            if not hasattr(dev, 'handle_control_transfer'):
                try:
                    attached.add((dev.idVendor, dev.idProduct))
                except Exception:
                    pass
        for key, dev in current_vidpids.items():
            if key not in attached:
                port = self.find_free_port()
                if port is None:
                    continue
                self.devices[port] = dev
                self.root_hub.device_attached(port, dev)
                logger.info(f"Device inserted port={port}: {key[0]:04x}:{key[1]:04x}")

    # ---- Control transfer handling ----
    def handle_control_transfer_req(self, data):
        kind, dev_addr, bmRT, bReq = struct.unpack('>BBBB', data[0:4])
        wValue, wIndex, wLength = struct.unpack('>HHH', data[4:10])
        data_addr, req_id = struct.unpack('>II', data[10:18])

        is_in = (bmRT & 0x80) != 0
        logger.debug(
            f"Control: dev={dev_addr} bmRT={bmRT:02x} bReq={bReq:02x} "
            f"wVal={wValue:04x} wIdx={wIndex:04x} wLen={wLength} req_id={req_id}")

        try:
            # SET_ADDRESS to the device being enumerated
            if dev_addr == 0 and bReq == USB_REQ_SET_ADDRESS:
                new_addr = wValue & 0xFF
                if self.enumerating_device:
                    port, dev = self.enumerating_device
                    self.dev_addr_map[new_addr] = (port, dev)
                    self.enumerating_device = None
                    logger.info(f"SET_ADDRESS: port {port} -> addr {new_addr}")
                self.send_control_transfer_res(req_id, USB_STATUS_OK, 0)
                return

            # Address 0 during enumeration -> the device being enumerated (or root hub)
            if dev_addr == 0:
                if self.enumerating_device:
                    port, dev = self.enumerating_device
                    self._handle_dev_control(dev, bmRT, bReq, wValue, wIndex, wLength,
                                              data_addr, req_id, is_in, dev_addr=0)
                else:
                    # Root hub before it gets its address
                    data_out, status = self.root_hub.handle_control_transfer(
                        bmRT, bReq, wValue, wIndex, wLength)
                    if is_in and data_out:
                        data_out = data_out[:wLength]
                        if data_out:
                            self.write_mem(data_addr, data_out)
                    self.send_control_transfer_res(req_id, status,
                                                    len(data_out) if data_out else 0)
                return

            # Address 1 -> root hub; also detect port reset to start enumeration
            if dev_addr == 1:
                if bmRT == 0x23 and bReq == 0x03:  # SET_PORT_FEATURE
                    if (wValue & 0xFF) == USB_PORT_FEAT_RESET:
                        port = wIndex & 0xFF
                        if port in self.devices:
                            self.release_device(self.devices[port])
                            for addr, (p, d) in list(self.dev_addr_map.items()):
                                if p == port:
                                    del self.dev_addr_map[addr]
                            self.enumerating_device = (port, self.devices[port])
                            logger.info(f"Port {port} reset -> enumerate at addr 0")
                data_out, status = self.root_hub.handle_control_transfer(
                    bmRT, bReq, wValue, wIndex, wLength)
                if is_in and data_out:
                    data_out = data_out[:wLength]
                    if data_out:
                        self.write_mem(data_addr, data_out)
                self.send_control_transfer_res(req_id, status,
                                                len(data_out) if data_out else 0)
                return

            # Other addresses -> real libusb device
            port, dev = self.dev_addr_map.get(dev_addr, (None, None))
            if dev is None:
                logger.debug(f"Unknown dev_addr {dev_addr}")
                self.send_control_transfer_res(req_id, USB_STATUS_ERROR, 0)
                return
            self._handle_dev_control(dev, bmRT, bReq, wValue, wIndex, wLength,
                                      data_addr, req_id, is_in, dev_addr=dev_addr)
        except Exception as e:
            logger.error(f"Control transfer error: {e}")
            self.send_control_transfer_res(req_id, USB_STATUS_ERROR, 0)

    def _handle_dev_control(self, dev, bmRT, bReq, wValue, wIndex, wLength,
                              data_addr, req_id, is_in, dev_addr):
        # Special handling for SET_CONFIGURATION on real devices
        if bmRT == 0x00 and bReq == USB_REQ_SET_CONFIGURATION \
                and not hasattr(dev, 'handle_control_transfer'):
            config_val = wValue
            logger.info(f"SET_CONFIGURATION {config_val} for addr {dev_addr}")
            try:
                # A configured device is configured again after the Amiga
                # goes offline and online, or resets it.
                self.release_device(dev)

                # Detach kernel driver before configuring (needed on Linux,
                # and usually needed on macOS to actually claim the device).
                for cfg in dev:
                    for intf in cfg:
                        try:
                            if dev.is_kernel_driver_active(intf.bInterfaceNumber):
                                dev.detach_kernel_driver(intf.bInterfaceNumber)
                                self.detached.setdefault(dev, []).append(intf.bInterfaceNumber)
                                logger.info(f"Detached kernel driver from intf {intf.bInterfaceNumber}")
                        except (usb.core.USBError, NotImplementedError):
                            pass

                dev.set_configuration(config_val)
                cfg = dev.get_active_configuration()
                for intf in cfg:
                    usb.util.claim_interface(dev, intf.bInterfaceNumber)
                    self.claimed.setdefault(dev, []).append(intf.bInterfaceNumber)
                    logger.info(f"Claimed intf {intf.bInterfaceNumber}")
                    # Force HID boot protocol on HID Mouse (3/1/2)
                    if (intf.bInterfaceClass == 3 and
                        intf.bInterfaceSubClass == 1 and
                        intf.bInterfaceProtocol == 2):
                        try:
                            # SET_PROTOCOL(boot=0)
                            dev.ctrl_transfer(0x21, 0x0B, 0,
                                               intf.bInterfaceNumber, timeout=1000)
                            logger.info(f"Boot protocol set on intf {intf.bInterfaceNumber}")
                        except usb.core.USBError as e:
                            logger.warning(f"SET_PROTOCOL failed: {e}")
                for intf in cfg:
                    for ep in intf:
                        if (usb.util.endpoint_type(ep.bmAttributes) == usb.util.ENDPOINT_TYPE_INTR and
                                usb.util.endpoint_direction(ep.bEndpointAddress) == usb.util.ENDPOINT_IN):
                            self.int_readers[(dev, ep.bEndpointAddress)] = InterruptReader(
                                dev, ep.bEndpointAddress, ep.wMaxPacketSize, self.wake_w)
                            logger.info(f"Reading interrupt endpoint {ep.bEndpointAddress:02x}")
            except usb.core.USBError as e:
                logger.error(f"SET_CONFIGURATION failed: {e}")
                self.send_control_transfer_res(req_id, USB_STATUS_ERROR, 0)
                return
            self.send_control_transfer_res(req_id, USB_STATUS_OK, 0)
            return

        # CLEAR_FEATURE(ENDPOINT_HALT) through libusb, which also resets the
        # host side's data toggle; a plain control transfer leaves it, and the
        # next packets on that endpoint are dropped.
        if bmRT == USB_RT_ENDPOINT_OUT and bReq == USB_REQ_CLEAR_FEATURE \
                and wValue == USB_FEAT_ENDPOINT_HALT \
                and not hasattr(dev, 'handle_control_transfer'):
            dev.clear_halt(wIndex & 0xFF)
            logger.info(f"Cleared halt on ep {wIndex & 0xFF:02x}")
            self.send_control_transfer_res(req_id, USB_STATUS_OK, 0)
            return

        # Virtual device
        if hasattr(dev, 'handle_control_transfer'):
            if not is_in and wLength > 0:
                out = self.read_mem(data_addr, wLength)
                data_out, status = dev.handle_control_transfer(
                    bmRT, bReq, wValue, wIndex, out)
            else:
                data_out, status = dev.handle_control_transfer(
                    bmRT, bReq, wValue, wIndex, wLength)
            if is_in and data_out:
                data_out = data_out[:wLength]
                if data_out:
                    self.write_mem(data_addr, data_out)
            self.send_control_transfer_res(req_id, status,
                                            len(data_out) if data_out else 0)
            return

        # Real libusb device
        if is_in:
            result = dev.ctrl_transfer(bmRT, bReq, wValue, wIndex, wLength, timeout=5000)
            data_out = bytes(result)[:wLength]
            if data_out:
                self.write_mem(data_addr, data_out)
            self.send_control_transfer_res(req_id, USB_STATUS_OK, len(data_out))
        else:
            out = None
            if wLength > 0:
                out = self.read_mem(data_addr, wLength)
            res = dev.ctrl_transfer(bmRT, bReq, wValue, wIndex, out or b'', timeout=5000)
            self.send_control_transfer_res(req_id, USB_STATUS_OK, res or 0)

    def send_control_transfer_res(self, req_id, status, actual_length):
        msg = struct.pack('>BBHI', USB_CONTROL_TRANSFER_RES, status, actual_length, req_id)
        self.a314d.send_data(self.current_stream_id, msg)

    # ---- Interrupt transfer handling ----
    def handle_interrupt_transfer_req(self, data):
        kind, dev_addr, endpoint, flags = struct.unpack('>BBBB', data[0:4])
        max_pkt, length = struct.unpack('>HH', data[4:8])
        data_addr, req_id = struct.unpack('>II', data[8:16])
        is_in = (endpoint & 0x80) != 0

        if is_in:
            self.pending_int.append({'req_id': req_id, 'dev_addr': dev_addr,
                                     'endpoint': endpoint,
                                     'length': min(length, USB_INT_DATA_MAX)})
            self.answer_pending_int()
            return

        port, dev = self.dev_addr_map.get(dev_addr, (None, None))
        if dev is None:
            logger.debug(f"Interrupt: unknown dev_addr {dev_addr}")
            self.send_interrupt_transfer_res(req_id, USB_STATUS_ERROR)
            return
        if hasattr(dev, 'read_interrupt'):
            self.send_interrupt_transfer_res(req_id, USB_STATUS_STALL)
            return
        try:
            res = dev.write(endpoint, data[16:16 + length], timeout=5000)
            self.send_interrupt_transfer_res(req_id, USB_STATUS_OK, actual_length=res or 0)
        except usb.core.USBTimeoutError:
            self.send_interrupt_transfer_res(req_id, USB_STATUS_NAK_TIMEOUT)
        except usb.core.USBError as e:
            if e.errno == errno.EPIPE:
                self.send_interrupt_transfer_res(req_id, USB_STATUS_STALL)
            else:
                logger.error(f"USB error in interrupt transfer: {e}")
                self.send_interrupt_transfer_res(req_id, USB_STATUS_ERROR)

    def poll_int(self, p):
        """The (status, data) answer for a pending interrupt IN transfer, or
        None while the device has nothing."""
        dev_addr, endpoint, length = p['dev_addr'], p['endpoint'], p['length']

        # Root hub status change endpoint
        if dev_addr == 1 and endpoint == 0x81:
            bm = self.root_hub.read_status_change()
            return (USB_STATUS_OK, bm[:length]) if bm else None

        port, dev = self.dev_addr_map.get(dev_addr, (None, None))
        if dev is None:
            logger.debug(f"Interrupt: unknown dev_addr {dev_addr}")
            return (USB_STATUS_ERROR, b'')

        # Virtual device
        if hasattr(dev, 'read_interrupt'):
            data_out, status = dev.read_interrupt(endpoint, length)
            if status == USB_STATUS_OK and data_out:
                return (USB_STATUS_OK, data_out[:length])
            if status in (USB_STATUS_OK, USB_STATUS_NAK_TIMEOUT):
                return None
            return (status, b'')

        reader = self.int_readers.get((dev, endpoint))
        if reader is None:
            logger.warning(f"Interrupt: ep {endpoint:02x} of dev_addr {dev_addr} isn't read")
            return (USB_STATUS_ERROR, b'')
        report = reader.get()
        if report is None:
            return None
        status, data_bytes = report
        if status != USB_STATUS_OK:
            return (status, b'')
        logger.debug(f"IntIN dev={dev_addr} ep={endpoint:02x} "
                     f"req_len={length} got={len(data_bytes)}: {data_bytes.hex()}")

        # If the device is sending HID++ 7-byte reports but Poseidon
        # is in boot protocol (3-byte request), convert.
        if (length == 3 and len(data_bytes) >= 7 and data_bytes[0] == 0x01):
            buttons = data_bytes[1]
            dx = int.from_bytes(data_bytes[2:4], 'little', signed=True)
            dy = int.from_bytes(data_bytes[4:6], 'little', signed=True)
            dx = max(-127, min(127, dx))
            dy = max(-127, min(127, dy))
            data_bytes = bytes([buttons, dx & 0xFF, dy & 0xFF])

        return (USB_STATUS_OK, data_bytes[:length])

    def answer_pending_int(self):
        """Answer each pending interrupt IN transfer that has data now."""
        if self.current_stream_id is None:
            return
        for p in list(self.pending_int):
            try:
                answer = self.poll_int(p)
            except Exception as e:
                logger.error(f"Error in interrupt transfer: {e}")
                answer = (USB_STATUS_ERROR, b'')
            if answer is None:
                continue
            self.pending_int.remove(p)
            status, data_out = answer
            self.send_interrupt_transfer_res(p['req_id'], status, data_out)

    def handle_abort_transfer_req(self, data):
        req_id, = struct.unpack('>I', data[4:8])
        self.pending_int = [p for p in self.pending_int if p['req_id'] != req_id]

    def send_interrupt_transfer_res(self, req_id, status, data_out=b'', actual_length=None):
        if actual_length is None:
            actual_length = len(data_out)
        msg = struct.pack('>BBHI', USB_INT_TRANSFER_RES, status, actual_length, req_id) + data_out
        self.a314d.send_data(self.current_stream_id, msg)

    # ---- Bulk transfer handling ----
    def handle_bulk_transfer_req(self, data):
        kind, dev_addr, endpoint, flags = struct.unpack('>BBBB', data[0:4])
        max_pkt, length = struct.unpack('>HH', data[4:8])
        data_addr, req_id = struct.unpack('>II', data[8:16])
        is_in = (endpoint & 0x80) != 0

        port, dev = self.dev_addr_map.get(dev_addr, (None, None))
        if dev is None:
            logger.debug(f"Bulk: unknown dev_addr {dev_addr}")
            self.send_bulk_transfer_res(req_id, USB_STATUS_ERROR, 0)
            return

        try:
            if is_in:
                result = dev.read(endpoint, in_read_length(length, max_pkt), timeout=5000)
                data_out = bytes(result)
                if len(data_out) > length:
                    logger.warning(f"Bulk IN overrun: ep={endpoint:02x} asked {length}, got {len(data_out)}")
                    self.send_bulk_transfer_res(req_id, USB_STATUS_ERROR, 0)
                    return
                if data_out:
                    self.write_mem(data_addr, data_out)
                self.send_bulk_transfer_res(req_id, USB_STATUS_OK, len(data_out))
            else:
                out = self.read_mem(data_addr, length)
                res = dev.write(endpoint, out, timeout=5000)
                self.send_bulk_transfer_res(req_id, USB_STATUS_OK, res or 0)
        except usb.core.USBError as e:
            if getattr(e, 'errno', None) in (60, 110):
                self.send_bulk_transfer_res(req_id, USB_STATUS_NAK_TIMEOUT, 0)
            elif getattr(e, 'errno', None) == 32:
                self.send_bulk_transfer_res(req_id, USB_STATUS_STALL, 0)
            else:
                logger.error(f"USB error in bulk transfer: {e}")
                self.send_bulk_transfer_res(req_id, USB_STATUS_ERROR, 0)
        except Exception as e:
            logger.error(f"Error in bulk transfer: {e}")
            self.send_bulk_transfer_res(req_id, USB_STATUS_ERROR, 0)

    def send_bulk_transfer_res(self, req_id, status, actual_length):
        msg = struct.pack('>BBHI', USB_BULK_TRANSFER_RES, status, actual_length, req_id)
        self.a314d.send_data(self.current_stream_id, msg)

    # ---- Message dispatch ----
    def client_connected(self):
        """Show the attached devices as disconnected for a moment: after an
        a314d restart, Poseidon still has them at their old addresses, and
        this makes it enumerate them again."""
        self.dev_addr_map.clear()
        self.enumerating_device = None
        self.pending_int.clear()
        for port, dev in list(self.devices.items()):
            self.release_device(dev)
            self.root_hub.device_detached(port)
            self.replug[port] = dev
        self.replug_at = time.time() + REPLUG_DELAY_S

    def replug_devices(self):
        if self.replug and time.time() >= self.replug_at:
            for port, dev in self.replug.items():
                self.root_hub.device_attached(port, dev)
            self.replug.clear()

    def client_disconnected(self):
        self.current_stream_id = None
        self.pending_int.clear()
        self.release_all_devices(give_back=True)
        self.dev_addr_map.clear()

    def process_a314_message(self, stream_id, ptype, payload):
        if ptype == self.a314d.MSG_CONNECT:
            service = payload.decode('latin-1', errors='replace')
            if service == SERVICE_NAME and self.current_stream_id is None:
                self.current_stream_id = stream_id
                self.a314d.send_connect_response(stream_id, 0)
                logger.info(f"Client connected to {SERVICE_NAME}")
                self.client_connected()
            else:
                self.a314d.send_connect_response(stream_id, 1)
        elif self.current_stream_id == stream_id:
            if ptype == self.a314d.MSG_DATA and len(payload) > 0:
                kind = payload[0]
                if kind == USB_CONTROL_TRANSFER_REQ:
                    self.handle_control_transfer_req(payload)
                elif kind == USB_INT_TRANSFER_REQ:
                    self.handle_interrupt_transfer_req(payload)
                elif kind == USB_BULK_TRANSFER_REQ:
                    self.handle_bulk_transfer_req(payload)
                elif kind == USB_ABORT_TRANSFER_REQ:
                    self.handle_abort_transfer_req(payload)
                else:
                    logger.warning(f"Unknown payload kind {kind}")
            elif ptype == self.a314d.MSG_RESET:
                logger.info("Client disconnected")
                self.client_disconnected()

    def run(self):
        logger.info(f"USB Bridge Service starting (port_count={self.port_count})")
        self.scan_usb_devices()

        last_scan = time.time()
        while True:
            while self.backlog:
                self.process_a314_message(*self.backlog.popleft())

            rl, _, _ = select.select([self.a314d, self.wake_r], [], [], POLL_INTERVAL_S)
            if self.wake_r in rl:
                try:
                    os.read(self.wake_r, 4096)
                except BlockingIOError:
                    pass
            if self.a314d in rl:
                stream_id, ptype, payload = self.a314d.wait_for_msg()
                self.process_a314_message(stream_id, ptype, payload)

            self.replug_devices()
            now = time.time()
            if now - last_scan > SCAN_INTERVAL_S and not self.replug:
                self.check_device_changes()
                last_scan = now
            self.answer_pending_int()


def main():
    logger.info("USB Bridge Service initializing")
    # a314d and systemd stop services with SIGTERM
    signal.signal(signal.SIGTERM, lambda signo, frame: sys.exit(0))
    service = None
    try:
        service = UsbBridgeService()
        service.run()
    except KeyboardInterrupt:
        logger.info("Interrupted")
    except Exception as e:
        logger.error(f"Service error: {e}", exc_info=True)
    finally:
        if service is not None:
            service.release_all_devices(give_back=True)


if __name__ == '__main__':
    main()
