# USB Bridge

The USB bridge lets the Amiga use USB devices plugged into the Raspberry Pi,
through the Poseidon USB stack. On the Amiga, `a314usb.device` is a Poseidon
hardware driver; it shows Poseidon a root hub whose ports hold the Pi's USB
devices. On the Pi, the `usbhw` service (`usbbridge.py`) passes the transfers
to the devices with libusb.

The Amiga needs Poseidon (tested with 4.5). HID devices (keyboards, mice,
Logitech Unifying receivers) and USB mass storage are tested.

## Installing

On the Pi, `install-pi.sh` installs the service and registers it in
`/etc/opt/a314/a314d.conf`. It also:

- writes `/etc/udev/rules.d/99-a314usb.rules`, so that the a314 user can open
  the devices that `usbbridge.conf` lets through. Run `install-pi.sh` again
  after changing the filters in `usbbridge.conf`.
- blacklists the `hid_logitech_dj` and `hid_logitech_hidpp` kernel modules
  (`/etc/modprobe.d/a314usb-blacklist.conf`). They switch a Logitech Unifying
  receiver into DJ mode, whose reports Poseidon's `hid.class` can't decode.
  The blacklist takes effect after a reboot.

On the Amiga, copy `a314usb.device` to `DEVS:USBHardware/` and add it to
Poseidon as a hardware device, unit 0, either in Trident or with

    AddUSBHardware DEVICE=DEVS:USBHardware/a314usb.device

Give the full path: with the bare device name, Poseidon adds a second entry
for the same device.

## Configuration

`/etc/opt/a314/usbbridge.conf` selects which of the Pi's USB devices the Amiga
gets. Each filter matches a vendor id and a product id, in hex; `*` matches
any. Hubs are never passed through.

    {
        "port_count": 4,
        "filters": [
            {"comment": "Logitech Unifying receiver", "vid": "046d", "pid": "c52b"}
        ]
    }

A device the Amiga gets is taken from its Linux driver while the Amiga uses
it, and given back when Poseidon takes the hardware offline. Only drivers the
bridge took are given back; an interface that had no driver bound stays
unbound.

Dismount the volumes of a USB drive (`Assign <volume>: DISMOUNT`) before taking
the hardware offline. Offline aborts the transfers in progress, and a file
system still using the drive shows read or write errors.

`port_count` is the number of ports on the Amiga's root hub, and so the most
devices the Amiga can have at once.

## Memory

`a314usb.device` takes a 4 KB buffer of A314 memory for control and bulk
transfer data while Poseidon has the hardware online, and frees it when it
goes offline. Interrupt transfer data doesn't use it.
