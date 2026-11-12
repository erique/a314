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
#include "kprintf.h"

#define SysBase (*(struct ExecBase**)4)

#define RDB_SCAN_LIMIT 16 // the RDB must live within the first 16 blocks

// fhb_PatchFlags bits (which DeviceNode fields the FSHB overrides).
#define FSPF_TYPE       (1 << 0)
#define FSPF_STACKSIZE  (1 << 4)
#define FSPF_PRIORITY   (1 << 5)
#define FSPF_GLOBALVEC  (1 << 8)

struct MountEntry
{
    struct          DeviceNode dev_node;
    struct          FileSysStartupMsg fssm;
    ULONG           env_vec[20];
    char            drive_name[36];  // BCPL string
    char            exec_name[24];   // BCPL string
    struct BootNode boot_node;
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

static BOOL mount_partition(struct A314ScsiBase* base, struct Drive* d, struct PartitionBlock* pb,
                            struct FileSysResource* fsr, struct ExpansionBase* expansion)
{
    struct MountEntry* me = (struct MountEntry*)AllocMem(sizeof(struct MountEntry), MEMF_PUBLIC | MEMF_CLEAR);
    if (!me)
    {
        return FALSE;
    }

    const ULONG* env = (const ULONG*)pb->pb_Environment;
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
    UBYTE dn_len = ((const UBYTE*)pb->pb_DriveName)[0];
    if (dn_len > 34)
    {
        dn_len = 34;
    }
    set_bcpl(me->drive_name, (const char*)&pb->pb_DriveName[1], dn_len);
    set_bcpl(me->exec_name, device_name, (UBYTE)strlen(device_name));

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
        me->dev_node.dn_SegList = fse->fse_SegList;
        if (fse->fse_PatchFlags & FSPF_GLOBALVEC)
        {
            me->dev_node.dn_GlobalVec = fse->fse_GlobalVec;
        }
        if (fse->fse_PatchFlags & FSPF_TYPE)
        {
            me->dev_node.dn_Type = fse->fse_Type;
        }
        if (fse->fse_PatchFlags & FSPF_STACKSIZE)
        {
            me->dev_node.dn_StackSize = fse->fse_StackSize;
        }
        if (fse->fse_PatchFlags & FSPF_PRIORITY)
        {
            me->dev_node.dn_Priority = fse->fse_Priority;
        }
    }
    Permit();
    if (fse)
    {
        kprintf("a314scsi: partition FS dostype=%lx -> seglist %lx\n", dostype, (ULONG)me->dev_node.dn_SegList);
    }

    LONG bootpri = (LONG)env[DE_BOOTPRI];
    BOOL bootable = (pb->pb_Flags & PBFF_BOOTABLE) ? TRUE : FALSE;

    me->boot_node.bn_Node.ln_Type   = NT_BOOTNODE;
    me->boot_node.bn_Node.ln_Pri    = bootable ? (BYTE)bootpri : -128;
    me->boot_node.bn_Node.ln_Name   = (char*)base->configdev;
    me->boot_node.bn_DeviceNode     = (APTR)&me->dev_node;

    kprintf("a314scsi: mount partition (bootpri=%ld bootable=%ld)\n", bootpri, (ULONG)bootable);

    Forbid();
    Enqueue(&expansion->MountList, &me->boot_node.bn_Node);
    Permit();
    return TRUE;
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

// Add the drive's RDB partitions to the MountList. TRUE if any were added.
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
                    if (mount_partition(base, d, pb, fsr, expansion))
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
    else
    {
        kprintf("a314scsi: no RDB on unit %ld\n", (ULONG)d->number);
    }

    FreeMem(buf, d->blocksize);
    return added;
}
