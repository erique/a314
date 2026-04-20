/*
 * Serial debug output via RawDoFmt + RawPutChar.
 */

#include <exec/types.h>
#include <exec/execbase.h>
#include <proto/exec.h>
#include <stdarg.h>

#include "debug.h"

#define SysBase (*(struct ExecBase **)4)

static void RawPutChar(__reg("d0") ULONG c, __reg("a6") struct ExecBase *sb) = "\tjsr\t-516(a6)\n";

static void raw_putch(__reg("d0") ULONG c, __reg("a3") struct ExecBase *sb)
{
    if (c == '\n')
        RawPutChar('\r', sb);
    RawPutChar(c, sb);
}

void kprintf(const char *fmt, ...)
{
    va_list args;
    struct ExecBase *sb = SysBase;

    va_start(args, fmt);
    RawDoFmt((STRPTR)fmt, (APTR)args, (void (*)())raw_putch, (APTR)sb);
    va_end(args);

    RawPutChar('\r', sb);
    RawPutChar('\n', sb);
}
