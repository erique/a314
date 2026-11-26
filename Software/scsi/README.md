# a314scsi

a314scsi.device gives the Amiga up to four SCSI disks, each one a hard disk
image (hdf) on the Raspberry Pi. The images have a Rigid Disk Block, and
their partitions are mounted as DOS devices.

## Pi side

The `scsi` service (`scsi.py`) serves the images. `/etc/opt/a314/scsi.conf`
names them:

    {
        "units": [
            {"unit": 0, "filename": "/home/pi/a314hdfs/Harddisk.hdf", "rw": true, "block_size": 512}
        ]
    }

`unit` is 0 to 3. A unit without `"rw": true` is write protected.
`block_size` is 512 if left out.

`"cdrom": true` makes the unit a CD-ROM drive, always write protected, for a
CD image (ISO 9660) with `"block_size": 2048`.

The images go in `~/a314hdfs`.

Every unit is removable. `scsi.py` checks the images every two seconds: an
hdf that is removed, copied in, replaced or overwritten is seen by the Amiga
as a disk change. After editing `scsi.conf`, reload it with

    sudo pkill -HUP -f /opt/a314/scsi.py

A unit that is added, removed or changed is then seen by the Amiga as a disk
change in that unit.

## Amiga side

Copy `a314scsi.device` to `DEVS:`. Running it as a command
(`DEVS:a314scsi.device`, e.g. in `S:User-Startup`) loads the device and
mounts the partitions. Made resident before DOS starts (e.g. with
LoadModule), it mounts them at boot, and a partition with a boot priority can
be booted from.

When a disk changes, the partitions of the new disk are mounted, and those
the new disk doesn't have are removed. A DOS name already in use gets a
suffix: `DH0` becomes `DH0_1`.

Filesystems in the RDB are added to FileSystem.resource, unless it already
has the same or a newer version, and are shared by all partitions of that
DOS type. A partition whose filesystem isn't in its RDB uses the one in
FileSystem.resource, such as the ROM's FFS.

A CD image has no RDB. When FileSystem.resource has a CD filesystem (DosType
`CD01`), the image is mounted as `CD<unit>:`, and it can be booted from with
boot priority -10 or by choosing it in the Early Startup menu. ODFileSystem
(Aminet `disk/cdrom/ODFileSystem`) adds itself to FileSystem.resource when it
is made resident with the devices:

    LoadModule DEVS:a314.device DEVS:a314scsi.device L:ODFileSystem

Otherwise a CD image is mounted with a mountlist for a CD filesystem, such as
`DEVS:DOSDrivers/CD0` of AmigaOS 3.2, with `Device = a314scsi.device` and
`Unit` the unit number times 10: unit 1 is `Unit = 10`.
