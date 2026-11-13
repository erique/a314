#!/usr/bin/env python3
"""
Print udev rules that let a group open the USB devices that usbbridge.conf
lets through, so usbbridge can use them without running as root.

Usage: udev-rules.py <usbbridge.conf> <group>
"""

import sys
import json

RULE_PREFIX = 'SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device"'


def main():
    if len(sys.argv) != 3:
        print(__doc__.strip(), file=sys.stderr)
        sys.exit(1)

    conf_file, group = sys.argv[1], sys.argv[2]
    with open(conf_file) as f:
        filters = json.load(f).get('filters', [{'vid': '*', 'pid': '*'}])

    print(f'# Generated from {conf_file} by udev-rules.py')
    for f in filters:
        rule = RULE_PREFIX
        vid = f.get('vid', '*')
        pid = f.get('pid', '*')
        if vid != '*':
            rule += f', ATTR{{idVendor}}=="{vid}"'
        if pid != '*':
            rule += f', ATTR{{idProduct}}=="{pid}"'
        print(f'{rule}, GROUP="{group}", MODE="0660"')


if __name__ == '__main__':
    main()
