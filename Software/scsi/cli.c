#include <exec/types.h>
#include <exec/memory.h>
#include <exec/io.h>
#include <exec/nodes.h>
#include <exec/lists.h>

#include <libraries/dos.h>
#include <libraries/dosextens.h>
#include <libraries/filehandler.h>
#include <libraries/expansion.h>
#include <libraries/expansionbase.h>

#include <dos/dosextens.h>

#include <proto/exec.h>
#include <proto/expansion.h>
#include <proto/dos.h>

#include <string.h>

#include "a314scsi.h"
#include "alib.h"
#include "kprintf.h"

#define SysBase (*(struct ExecBase**)4)

#define MAX_PENDING 16
#define MAX_DRIVE_NAME 34   // longest BCPL drive name rdb.c stores

// Is this DeviceNode one of ours (its exec device name == a314scsi.device)?
static BOOL is_ours(struct DeviceNode* dn)
{
    if (!dn || !dn->dn_Startup)
    {
        return FALSE;
    }

    // dn_Startup is not always a FileSysStartupMsg in other handlers' nodes.
    struct FileSysStartupMsg* fssm = (struct FileSysStartupMsg*)BADDR(dn->dn_Startup);
    if (!TypeOfMem(fssm) || !fssm->fssm_Device || !TypeOfMem(BADDR(fssm->fssm_Device)))
    {
        return FALSE;
    }

    const UBYTE* name = (const UBYTE*)BADDR(fssm->fssm_Device);
    UBYTE len = (UBYTE)strlen(DEVICE_NAME);
    return name[0] == len && memcmp(&name[1], DEVICE_NAME, len) == 0;
}

// Does this DOS device mount the same partition of the same unit as dn? The
// DosEnvec fields that place and format the partition must all match.
static BOOL same_partition(struct DosList* dol, struct DeviceNode* dn)
{
    BPTR startup = dol->dol_misc.dol_handler.dol_Startup;
    if (!startup || !TypeOfMem(BADDR(startup)) || !is_ours((struct DeviceNode*)dol))
    {
        return FALSE;
    }

    struct FileSysStartupMsg* a = (struct FileSysStartupMsg*)BADDR(startup);
    struct FileSysStartupMsg* b = (struct FileSysStartupMsg*)BADDR(dn->dn_Startup);
    if (a->fssm_Unit != b->fssm_Unit || !a->fssm_Environ || !TypeOfMem(BADDR(a->fssm_Environ)))
    {
        return FALSE;
    }

    static const UBYTE fields[] = { DE_SIZEBLOCK, DE_NUMHEADS, DE_BLKSPERTRACK, DE_LOWCYL, DE_UPPERCYL, DE_DOSTYPE };
    const ULONG* ea = (const ULONG*)BADDR(a->fssm_Environ);
    const ULONG* eb = (const ULONG*)BADDR(b->fssm_Environ);
    for (UWORD i = 0; i < sizeof(fields); i++)
    {
        UBYTE f = fields[i];
        if (ea[DE_TABLESIZE] < f || eb[DE_TABLESIZE] < f || ea[f] != eb[f])
        {
            return FALSE;
        }
    }
    return TRUE;
}

// Make the BCPL name unique among DOS devices: "DH0" -> "DH0_1" .. "DH0_9".
// FALSE: no free name.
static BOOL unique_name(struct Library* DOSBase, UBYTE* bname, UBYTE maxlen)
{
    UBYTE len = bname[0];
    char name[40];
    if (len + 2 >= sizeof(name) || len + 2 > maxlen)
    {
        return FALSE;
    }
    memcpy(name, &bname[1], len);
    name[len] = '_';
    name[len + 2] = 0;

    for (char c = '1'; c <= '9'; c++)
    {
        name[len + 1] = c;
        struct DosList* dl = LockDosList(LDF_DEVICES | LDF_READ);
        BOOL taken = FindDosEntry(dl, (STRPTR)name, LDF_DEVICES) != NULL;
        UnLockDosList(LDF_DEVICES | LDF_READ);
        if (!taken)
        {
            bname[0] = len + 2;
            memcpy(&bname[1], name, len + 2);
            return TRUE;
        }
    }
    return FALSE;
}

// Is this DeviceNode on the locked DOS list that starts at dl?
static BOOL on_dos_list(struct Library* DOSBase, struct DosList* dl, struct DeviceNode* dn)
{
    while ((dl = NextDosEntry(dl, LDF_DEVICES)))
    {
        if ((struct DeviceNode*)dl == dn)
        {
            return TRUE;
        }
    }
    return FALSE;
}

// Is this DeviceNode itself on the DOS list? DOS leaves the BootNodes it
// mounted at boot on the MountList.
static BOOL in_dos_list(struct Library* DOSBase, struct DeviceNode* dn)
{
    struct DosList* dl = LockDosList(LDF_DEVICES | LDF_READ);
    BOOL found = on_dos_list(DOSBase, dl, dn);
    UnLockDosList(LDF_DEVICES | LDF_READ);
    return found;
}

// Does a volume, or a lock on one, still belong to this handler?
static BOOL handler_in_use(struct Library* DOSBase, struct MsgPort* port)
{
    BOOL used = FALSE;
    struct DosList* dl = LockDosList(LDF_VOLUMES | LDF_READ);
    while (!used && (dl = NextDosEntry(dl, LDF_VOLUMES)))
    {
        BPTR lock = dl->dol_misc.dol_volume.dol_LockList;
        used = dl->dol_Task == port || (lock && ((struct FileLock*)BADDR(lock))->fl_Task == port);
    }
    UnLockDosList(LDF_VOLUMES | LDF_READ);
    return used;
}

// Remove a DOS device of ours whose partition the unit's new medium doesn't
// have. A running handler is inhibited and asked to quit; one that is still
// in use, or won't quit, keeps its device.
static void retire_device(struct Library* DOSBase, struct DeviceNode* dn)
{
    char name[40];
    UBYTE* bname = (UBYTE*)BADDR(dn->dn_Name);
    UBYTE len = bname[0] < sizeof(name) - 1 ? bname[0] : sizeof(name) - 2;
    memcpy(name, &bname[1], len);
    name[len] = ':';
    name[len + 1] = 0;

    // The packets go to the handler's port: a volume may have the device's
    // name, so looking the device up by name could find the volume instead.
    struct MsgPort* port = dn->dn_Task;
    if (port)
    {
        if (!DoPkt(port, ACTION_INHIBIT, DOSTRUE, 0, 0, 0, 0))
        {
            kprintf("a314scsi: %s stays, not inhibited\n", name);
            return;
        }
        // Let Workbench and others let go of the volume that just went away.
        Delay(TICKS_PER_SECOND);
        if (handler_in_use(DOSBase, port) || !DoPkt(port, ACTION_DIE, 0, 0, 0, 0, 0))
        {
            DoPkt(port, ACTION_INHIBIT, DOSFALSE, 0, 0, 0, 0);
            kprintf("a314scsi: %s stays, its handler is in use or won't quit\n", name);
            return;
        }
    }

    struct DosList* dl = LockDosList(LDF_DEVICES | LDF_WRITE);
    BOOL removed = on_dos_list(DOSBase, dl, dn) && dn->dn_Task == port && RemDosEntry((struct DosList*)dn);
    UnLockDosList(LDF_DEVICES | LDF_WRITE);

    kprintf("a314scsi: %s %s\n", name, removed ? "retired" : "stays");

    // A handler that quit may still be finishing with its DeviceNode, so only
    // a node whose handler never ran is freed.
    if (removed && !port)
    {
        free_partition(dn);
    }
}

// Retire our DOS devices on the units in pending whose partitions the new
// media no longer have.
static void retire_orphans(struct Library* DOSBase, struct DeviceNode** pending, UWORD count)
{
    struct DeviceNode* orphans[MAX_PENDING];
    UWORD n = 0;

    struct DosList* dl = LockDosList(LDF_DEVICES | LDF_READ);
    while (n < MAX_PENDING && (dl = NextDosEntry(dl, LDF_DEVICES)))
    {
        struct DeviceNode* dn = (struct DeviceNode*)dl;
        if (!is_ours(dn) || !is_mount_entry(dn))
        {
            continue;
        }

        ULONG unit = ((struct FileSysStartupMsg*)BADDR(dn->dn_Startup))->fssm_Unit;
        BOOL unitchanged = FALSE;
        BOOL kept = FALSE;
        for (UWORD i = 0; i < count; i++)
        {
            if (((struct FileSysStartupMsg*)BADDR(pending[i]->dn_Startup))->fssm_Unit == unit)
            {
                unitchanged = TRUE;
                kept = kept || same_partition(dl, pending[i]);
            }
        }
        if (unitchanged && !kept)
        {
            orphans[n++] = dn;
        }
    }
    UnLockDosList(LDF_DEVICES | LDF_READ);

    for (UWORD i = 0; i < n; i++)
    {
        retire_device(DOSBase, orphans[i]);
    }
}

int memcmp(const void* a, const void* b, size_t n)
{
    const UBYTE* pa = (const UBYTE*)a;
    const UBYTE* pb = (const UBYTE*)b;
    for (size_t i = 0; i < n; i++)
    {
        if (pa[i] != pb[i])
        {
            return (int)pa[i] - (int)pb[i];
        }
    }
    return 0;
}

// Mount the partitions whose BootNodes are on the MountList. Each one is
// detached under Forbid, then AddDosNode'd outside it (AddDosNode with
// ADNF_STARTPROC creates a process), so running this again mounts nothing twice.
// Runs are serialised by the device's mountsem. It must run in a process.
void mount_boot_nodes(struct Library* ExpansionBase, struct A314ScsiBase* base)
{
    ObtainSemaphore(&base->mountsem);

    struct DeviceNode* pending[MAX_PENDING];
    LONG pri[MAX_PENDING];
    UWORD count = 0;

    struct List* ml = &((struct ExpansionBase*)ExpansionBase)->MountList;

    Forbid();
    struct Node* n = ml->lh_Head;
    while (n->ln_Succ && count < MAX_PENDING)
    {
        struct Node* next = n->ln_Succ;
        struct BootNode* bn = (struct BootNode*)n;
        if (is_ours((struct DeviceNode*)bn->bn_DeviceNode))
        {
            Remove(n);
            pending[count] = (struct DeviceNode*)bn->bn_DeviceNode;
            pri[count] = (LONG)(BYTE)bn->bn_Node.ln_Pri;
            count++;
        }
        n = next;
    }
    Permit();

    // A partition still mounted from an earlier medium is skipped: its
    // filesystem takes the new medium through the change interrupt. Another
    // device with the same name gets the partition renamed.
    struct Library* DOSBase = OpenLibrary((STRPTR)DOSNAME, 36);
    if (DOSBase)
    {
        retire_orphans(DOSBase, pending, count);
    }

    UWORD added = 0;
    UWORD atboot = 0;
    for (UWORD i = 0; i < count; i++)
    {
        struct DeviceNode* dn = pending[i];
        UBYTE* bname = (UBYTE*)BADDR(dn->dn_Name);
        if (DOSBase)
        {
            char name[40];
            UBYTE len = bname[0] < sizeof(name) ? bname[0] : sizeof(name) - 1;
            memcpy(name, &bname[1], len);
            name[len] = 0;

            if (in_dos_list(DOSBase, dn))
            {
                kprintf("a314scsi: %s was mounted at boot\n", name);
                atboot++;
                continue;
            }

            struct DosList* dl = LockDosList(LDF_DEVICES | LDF_READ);
            struct DosList* found = FindDosEntry(dl, (STRPTR)name, LDF_DEVICES);
            BOOL same = found && same_partition(found, dn);
            UnLockDosList(LDF_DEVICES | LDF_READ);

            if (same)
            {
                kprintf("a314scsi: %s is mounted already\n", name);
                free_partition(dn);
                continue;
            }
            if (found && !unique_name(DOSBase, bname, MAX_DRIVE_NAME))
            {
                kprintf("a314scsi: no free name for %s\n", name);
                free_partition(dn);
                continue;
            }
        }
        kprintf("a314scsi: AddDosNode pri=%ld\n", pri[i]);
        AddDosNode(pri[i], ADNF_STARTPROC, dn);
        added++;
    }
    if (DOSBase)
    {
        CloseLibrary(DOSBase);
    }

    kprintf("a314scsi: mounted %ld of %ld node(s)\n", (LONG)added, (LONG)(count - atboot));

    ReleaseSemaphore(&base->mountsem);
}

LONG cli_main(void)
{
    kprintf("a314scsi: cli enter\n");
    struct Library* ExpansionBase = OpenLibrary((STRPTR)EXPANSIONNAME, 0);
    if (!ExpansionBase)
    {
        return RETURN_FAIL;
    }

    struct MsgPort* port = CreatePort(NULL, 0);
    if (!port)
    {
        CloseLibrary(ExpansionBase);
        return RETURN_FAIL;
    }

    // Opening the device runs its io task, which discovers and enqueues the
    // partitions. If it wasn't resident yet this also loads it.
    struct IOStdReq ior;
    memset(&ior, 0, sizeof(ior));
    ior.io_Message.mn_ReplyPort = port;
    ior.io_Message.mn_Length = sizeof(ior);

    BOOL opened = FALSE;
    for (UWORD u = 0; u < A314SCSI_MAX_DRIVES; u++)
    {
        if (!OpenDevice((STRPTR)DEVICE_NAME, u * A314SCSI_UNIT_STEP, (struct IORequest*)&ior, 0))
        {
            opened = TRUE;
            break;
        }
    }
    if (!opened)
    {
        kprintf("a314scsi: cli OpenDevice FAILED\n");
        DeletePort(port);
        CloseLibrary(ExpansionBase);
        return RETURN_FAIL;
    }

    mount_boot_nodes(ExpansionBase, (struct A314ScsiBase*)ior.io_Device);

    CloseDevice((struct IORequest*)&ior);
    DeletePort(port);
    CloseLibrary(ExpansionBase);

    return RETURN_OK;
}
