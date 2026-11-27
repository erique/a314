#include <exec/types.h>
#include <exec/memory.h>
#include <exec/nodes.h>
#include <exec/lists.h>

#include <libraries/dos.h>
#include <libraries/dosextens.h>
#include <libraries/filehandler.h>
#include <libraries/expansion.h>
#include <libraries/expansionbase.h>

#include <devices/hardblocks.h>
#include <resources/filesysres.h>

#include <proto/exec.h>

#include <stddef.h>
#include <string.h>

#include "a314scsi.h"
#include "scsi.h"
#include "kprintf.h"

#define SysBase (*(struct ExecBase**)4)

#define RDB_SCAN_LIMIT 16 // the RDB must live within the first 16 blocks

// A CD image has no RDB: the whole disc is one partition for the CD
// filesystem (DosType CD01) in FileSystem.resource.
#define CD_DOSTYPE      0x43443031UL    // 'CD01'
#define ISO_PVD_OFFSET  (16UL * 2048)   // byte offset of the ISO 9660 primary volume descriptor
#define ISO_PVD_ID      "CD001"         // bytes 1-5 of the volume descriptor
#define CD_BOOTPRI      (-10)           // below the usual hard disk partitions
#define CD_BUFFERS      16
#define CD_MAXTRANSFER  0x100000UL
#define CD_MASK         0x7FFFFFFEUL

#define FSE_PATCH_FIELDS ((offsetof(struct FileSysEntry, fse_GlobalVec) - offsetof(struct FileSysEntry, fse_Type)) / sizeof(ULONG) + 1)
#define FSE_PATCH_MASK   ((1UL << FSE_PATCH_FIELDS) - 1)

struct MountEntry
{
    struct          DeviceNode dev_node;
    struct          FileSysStartupMsg fssm;
    ULONG           env_vec[20];
    char            drive_name[36];  // BCPL string
    char            exec_name[24];   // BCPL string
    struct BootNode boot_node;
    struct Drive*   drive;
    ULONG           changecount;     // the drive's, when the partition was read
};

static ULONG sum_block(const ULONG* p, ULONG longs)
{
    ULONG sum = 0;
    for (ULONG i = 0; i < longs; i++)
    {
        sum += p[i];
    }
    return sum;
}

static BOOL read_block(struct A314ScsiBase* base, struct Drive* d, ULONG block, APTR buf)
{
    return scsi_rw(base, d, block, 1, buf, FALSE) == 0;
}

// Copy a BCPL-style string (len byte + chars) into a BCPL field.
static void set_bcpl(char* dst, const char* src, UBYTE len)
{
    dst[0] = len;
    for (UBYTE i = 0; i < len; i++)
    {
        dst[1 + i] = src[i];
    }
}

// The newest FileSystem.resource entry for this DOSType. Call under Forbid.
static struct FileSysEntry* find_filesystem(struct FileSysResource* fsr, ULONG dostype)
{
    struct FileSysEntry* best = NULL;
    for (struct Node* n = fsr->fsr_FileSysEntries.lh_Head; n->ln_Succ; n = n->ln_Succ)
    {
        struct FileSysEntry* fse = (struct FileSysEntry*)n;
        if (fse->fse_DosType == dostype && (!best || fse->fse_Version > best->fse_Version))
        {
            best = fse;
        }
    }
    return best;
}

// Walk the RDB FileSysHeaderBlock list and add each filesystem that
// FileSystem.resource doesn't have in this or a newer version, so every
// filesystem is loaded once and shared by all partitions using it.
static ULONG load_filesystems(struct A314ScsiBase* base, struct Drive* d, ULONG fshList, struct FileSysResource* fsr)
{
    APTR buf = AllocMem(d->blocksize, MEMF_PUBLIC | MEMF_CLEAR);
    if (!buf)
    {
        return 0;
    }

    ULONG n = 0;
    ULONG b = fshList;
    while (b != 0xFFFFFFFFUL && b != 0)
    {
        if (!read_block(base, d, b, buf))
        {
            break;
        }

        struct FileSysHeaderBlock* fhb = (struct FileSysHeaderBlock*)buf;
        if (fhb->fhb_ID != IDNAME_FILESYSHEADER)
        {
            break;
        }

        ULONG next = fhb->fhb_Next;
        kprintf("a314scsi: FSHD dostype=%lx version=%lx segblock=%ld\n", fhb->fhb_DosType, fhb->fhb_Version, fhb->fhb_SegListBlocks);

        Forbid();
        struct FileSysEntry* have = find_filesystem(fsr, fhb->fhb_DosType);
        BOOL needed = !have || have->fse_Version < fhb->fhb_Version;
        Permit();

        if (needed)
        {
            struct FileSysEntry* fse = (struct FileSysEntry*)AllocMem(sizeof(struct FileSysEntry), MEMF_PUBLIC | MEMF_CLEAR);
            if (fse)
            {
                // The FileSysEntry fields from fse_DosType on are laid out as
                // the FileSysHeaderBlock's from fhb_DosType on.
                CopyMem(&fhb->fhb_DosType, &fse->fse_DosType, sizeof(struct FileSysEntry) - offsetof(struct FileSysEntry, fse_DosType));
                fse->fse_Node.ln_Name = (char*)device_name;
                fse->fse_SegList = load_seglist(base, d, fhb->fhb_SegListBlocks);
                if (fse->fse_SegList)
                {
                    Forbid();
                    AddHead(&fsr->fsr_FileSysEntries, &fse->fse_Node);
                    Permit();
                    n++;
                }
                else
                {
                    FreeMem(fse, sizeof(struct FileSysEntry));
                }
            }
        }

        b = next;
    }

    FreeMem(buf, d->blocksize);
    return n;
}

// Are these BCPL names the same? DOS names are compared without regard to case.
BOOL same_name(const char* a, const char* b)
{
    if (a[0] != b[0])
    {
        return FALSE;
    }
    for (UBYTE i = 1; i <= (UBYTE)a[0]; i++)
    {
        char ca = (a[i] >= 'a' && a[i] <= 'z') ? a[i] - ('a' - 'A') : a[i];
        char cb = (b[i] >= 'a' && b[i] <= 'z') ? b[i] - ('a' - 'A') : b[i];
        if (ca != cb)
        {
            return FALSE;
        }
    }
    return TRUE;
}

// Does a BootNode on the MountList have this BCPL name? Call under Forbid.
static BOOL boot_name_taken(struct ExpansionBase* expansion, const char* bname)
{
    for (struct Node* n = expansion->MountList.lh_Head; n->ln_Succ; n = n->ln_Succ)
    {
        struct DeviceNode* dn = (struct DeviceNode*)((struct BootNode*)n)->bn_DeviceNode;
        if (dn && dn->dn_Name && same_name((const char*)BADDR(dn->dn_Name), bname))
        {
            return TRUE;
        }
    }
    return FALSE;
}

// DOS mounts the MountList's BootNodes at boot without checking their names,
// so a name another BootNode has gets a suffix: "DH0" -> "DH0_1" .. "DH0_9".
// Call under Forbid. FALSE: no free name.
static BOOL unique_boot_name(struct ExpansionBase* expansion, char* bname)
{
    if (!boot_name_taken(expansion, bname))
    {
        return TRUE;
    }
    UBYTE len = bname[0];
    if (len + 2 > MAX_DRIVE_NAME)
    {
        return FALSE;
    }
    bname[0] = len + 2;
    bname[len + 1] = '_';
    for (char c = '1'; c <= '9'; c++)
    {
        bname[len + 2] = c;
        if (!boot_name_taken(expansion, bname))
        {
            return TRUE;
        }
    }
    return FALSE;
}

// Add a partition to the MountList: its DosEnvec, its drive name as a BCPL
// string, and whether it can be booted from.
static BOOL mount_partition(struct A314ScsiBase* base, struct Drive* d, const ULONG* env,
                            const UBYTE* drive_name, BOOL bootable,
                            struct FileSysResource* fsr, struct ExpansionBase* expansion)
{
    struct MountEntry* me = (struct MountEntry*)AllocMem(sizeof(struct MountEntry), MEMF_PUBLIC | MEMF_CLEAR);
    if (!me)
    {
        return FALSE;
    }

    ULONG env_longs = env[0] + 1; // de_TableSize + 1
    if (env_longs > 20)
    {
        env_longs = 20;
    }
    for (ULONG i = 0; i < env_longs; i++)
    {
        me->env_vec[i] = env[i];
    }

    if (me->env_vec[0] > env_longs - 1)
        me->env_vec[0] = env_longs - 1;

    // Drive name (from the partition) and exec device name, as BCPL strings.
    UBYTE dn_len = drive_name[0];
    if (dn_len > MAX_DRIVE_NAME)
    {
        dn_len = MAX_DRIVE_NAME;
    }
    set_bcpl(me->drive_name, (const char*)&drive_name[1], dn_len);
    set_bcpl(me->exec_name, device_name, (UBYTE)strlen(device_name));

    me->drive = d;
    me->changecount = d->changecount;

    me->fssm.fssm_Unit          = d->number * A314SCSI_UNIT_STEP;
    me->fssm.fssm_Device        = MKBADDR(me->exec_name);
    me->fssm.fssm_Environ       = MKBADDR(me->env_vec);
    me->fssm.fssm_Flags         = 0;

    me->dev_node.dn_Type        = 0;
    me->dev_node.dn_StackSize   = 600;
    me->dev_node.dn_Priority    = 10;
    me->dev_node.dn_Startup     = MKBADDR(&me->fssm);
    me->dev_node.dn_Name        = MKBADDR(me->drive_name);
    me->dev_node.dn_GlobalVec   = -1;

    // If FileSystem.resource has a filesystem for the partition's DOSType,
    // point the DeviceNode at it (patching the fields its entry requests).
    ULONG dostype = env[DE_DOSTYPE];
    Forbid();
    struct FileSysEntry* fse = fsr ? find_filesystem(fsr, dostype) : NULL;
    if (fse)
    {
        ULONG* src = &fse->fse_Type;
        ULONG* dst = (ULONG*)&me->dev_node.dn_Type;
        for (ULONG patch = fse->fse_PatchFlags & FSE_PATCH_MASK; patch; patch >>= 1)
        {
            if (patch & 1)
            {
                *dst = *src;
            }
            src++;
            dst++;
        }
    }
    Permit();
    if (fse)
    {
        kprintf("a314scsi: partition FS dostype=%lx -> seglist %lx\n", dostype, (ULONG)me->dev_node.dn_SegList);
    }

    LONG bootpri = (LONG)env[DE_BOOTPRI];

    me->boot_node.bn_Node.ln_Type   = NT_BOOTNODE;
    me->boot_node.bn_Node.ln_Pri    = bootable ? (BYTE)bootpri : -128;
    me->boot_node.bn_Node.ln_Name   = (char*)base->configdev;
    me->boot_node.bn_DeviceNode     = (APTR)&me->dev_node;

    kprintf("a314scsi: mount partition (bootpri=%ld bootable=%ld)\n", bootpri, (ULONG)bootable);

    // Once DOS runs, the mount process renames against the DOS list instead.
    Forbid();
    BOOL dosup = FindName(&SysBase->LibList, (STRPTR)DOSNAME) != NULL;
    BOOL named = dosup || unique_boot_name(expansion, me->drive_name);
    if (named)
    {
        Enqueue(&expansion->MountList, &me->boot_node.bn_Node);
    }
    Permit();
    if (!named)
    {
        kprintf("a314scsi: no free name for a partition\n");
        FreeMem(me, sizeof(struct MountEntry));
    }
    return named;
}

// Was this DeviceNode made by mount_partition()?
BOOL is_mount_entry(struct DeviceNode* dn)
{
    struct MountEntry* me = (struct MountEntry*)dn;
    return dn->dn_Startup == MKBADDR(&me->fssm)
        && me->fssm.fssm_Device == MKBADDR(me->exec_name)
        && me->fssm.fssm_Environ == MKBADDR(me->env_vec);
}

// Free a partition that mount_partition() made and that is not mounted.
void free_partition(struct DeviceNode* dn)
{
    FreeMem(dn, sizeof(struct MountEntry));
}

// The drive of a DeviceNode made by mount_partition(), or NULL.
struct Drive* partition_drive(struct DeviceNode* dn)
{
    return is_mount_entry(dn) ? ((struct MountEntry*)dn)->drive : NULL;
}

// Was the partition read from its drive's current medium?
BOOL partition_current(struct DeviceNode* dn)
{
    struct MountEntry* me = (struct MountEntry*)dn;
    return me->changecount == me->drive->changecount;
}

// Does the medium hold an ISO 9660 file system? buf takes one block.
static BOOL is_iso9660(struct A314ScsiBase* base, struct Drive* d, UBYTE* buf)
{
    return read_block(base, d, ISO_PVD_OFFSET >> d->blockshift, buf)
        && memcmp(buf + 1, ISO_PVD_ID, sizeof(ISO_PVD_ID) - 1) == 0;
}

// Add the disc as partition CD<unit>, bootable, if FileSystem.resource has a
// CD filesystem to mount it with.
static BOOL mount_cdrom(struct A314ScsiBase* base, struct Drive* d, struct ExpansionBase* expansion)
{
    struct FileSysResource* fsr = (struct FileSysResource*)OpenResource((STRPTR)FSRNAME);
    Forbid();
    BOOL have_fs = fsr && find_filesystem(fsr, CD_DOSTYPE);
    Permit();
    if (!have_fs)
    {
        kprintf("a314scsi: no CD filesystem for unit %ld\n", (ULONG)d->number);
        return FALSE;
    }

    ULONG env[DE_DOSTYPE + 1];
    memset(env, 0, sizeof(env));
    env[DE_TABLESIZE]    = DE_DOSTYPE;
    env[DE_SIZEBLOCK]    = d->blocksize / sizeof(ULONG);
    env[DE_NUMHEADS]     = 1;
    env[DE_SECSPERBLK]   = 1;
    env[DE_BLKSPERTRACK] = 1;
    env[DE_UPPERCYL]     = d->blocks - 1;
    env[DE_NUMBUFFERS]   = CD_BUFFERS;
    env[DE_BUFMEMTYPE]   = MEMF_PUBLIC;
    env[DE_MAXTRANSFER]  = CD_MAXTRANSFER;
    env[DE_MASK]         = CD_MASK;
    env[DE_BOOTPRI]      = (ULONG)CD_BOOTPRI;
    env[DE_DOSTYPE]      = CD_DOSTYPE;

    UBYTE name[] = { 3, 'C', 'D', (UBYTE)('0' + d->number) };
    kprintf("a314scsi: ISO 9660 on unit %ld\n", (ULONG)d->number);
    return mount_partition(base, d, env, name, TRUE, fsr, expansion);
}

// Add the drive's partitions to the MountList: those in its RDB, or the
// disc of a CD-ROM unit. TRUE if any were added.
BOOL mount_drive(struct A314ScsiBase* base, struct Drive* d)
{
    BOOL added = FALSE;

    kprintf("a314scsi: mount_drive unit=%ld blocksize=%ld\n", (ULONG)d->number, d->blocksize);

    if (d->flags & DRVF_MOUNTED)
    {
        return FALSE;
    }

    // One block buffer serves the whole routine: first the RDB scan, then each
    // partition block in turn.
    APTR buf = AllocMem(d->blocksize, MEMF_PUBLIC | MEMF_CLEAR);
    if (!buf)
    {
        kprintf("a314scsi: mount_drive AllocMem FAILED\n");
        return FALSE;
    }

    // Find the RigidDiskBlock within the first few sectors.
    struct RigidDiskBlock* rdb = NULL;
    for (ULONG block = 0; block < RDB_SCAN_LIMIT; block++)
    {
        if (!read_block(base, d, block, buf))
        {
            kprintf("a314scsi: read block %ld FAILED\n", block);
            break;
        }

        struct RigidDiskBlock* r = (struct RigidDiskBlock*)buf;
        if (r->rdb_ID == IDNAME_RIGIDDISK)
        {
            ULONG longs = r->rdb_SummedLongs;
            if (longs > d->blocksize / 4)
                longs = d->blocksize / 4;

            if (sum_block((ULONG*)buf, longs) == 0)
            {
                rdb = r;
                break;
            }
        }
    }

    if (rdb)
    {
        kprintf("a314scsi: RDB found on unit %ld\n", (ULONG)d->number);

        // Grab the chain heads before buf gets reused for partition blocks.
        ULONG fsh_list = rdb->rdb_FileSysHeaderList;
        ULONG part_block = rdb->rdb_PartitionList;

        struct FileSysResource* fsr = (struct FileSysResource*)OpenResource((STRPTR)FSRNAME);
        if (fsr)
        {
            ULONG fscount = load_filesystems(base, d, fsh_list, fsr);
            kprintf("a314scsi: %ld filesystem(s) loaded\n", fscount);
        }

        // Enqueue every partition on the expansion MountList (opened once).
        struct ExpansionBase* expansion = (struct ExpansionBase*)OpenLibrary((STRPTR)EXPANSIONNAME, 0);
        if (expansion)
        {
            while (part_block != 0xFFFFFFFFUL && part_block != 0)
            {
                if (!read_block(base, d, part_block, buf))
                {
                    break;
                }

                struct PartitionBlock* pb = (struct PartitionBlock*)buf;
                if (pb->pb_ID != IDNAME_PARTITION)
                {
                    break;
                }

                ULONG next = pb->pb_Next;
                if (!(pb->pb_Flags & PBFF_NOMOUNT))
                {
                    BOOL bootable = (pb->pb_Flags & PBFF_BOOTABLE) ? TRUE : FALSE;
                    if (mount_partition(base, d, (const ULONG*)pb->pb_Environment,
                                        (const UBYTE*)pb->pb_DriveName, bootable, fsr, expansion))
                    {
                        added = TRUE;
                    }
                }
                part_block = next;
            }
            CloseLibrary((struct Library*)expansion);
        }

        d->flags |= DRVF_MOUNTED;
    }
    else if (d->devtype == SCSI_PDT_CDROM && is_iso9660(base, d, buf))
    {
        struct ExpansionBase* expansion = (struct ExpansionBase*)OpenLibrary((STRPTR)EXPANSIONNAME, 0);
        if (expansion)
        {
            added = mount_cdrom(base, d, expansion);
            CloseLibrary((struct Library*)expansion);
        }
        d->flags |= DRVF_MOUNTED;
    }
    else
    {
        kprintf("a314scsi: no RDB on unit %ld\n", (ULONG)d->number);
    }

    FreeMem(buf, d->blocksize);
    return added;
}
