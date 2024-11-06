#include <exec/types.h>
#include <exec/memory.h>
#include <exec/tasks.h>
#include <hardware/intbits.h>

#include <proto/exec.h>

#include <string.h>

#include "a314.h"
#include "device.h"
#include "protocol.h"
#include "startup.h"
#include "debug.h"
#include "pi_if.h"

#define SysBase (*(struct ExecBase **)4)

int probe_pi_interface(struct A314Device *dev)
{
	// FindConfig(TF, TF4060)
	// check autoconf "rom" version
}

void setup_pi_interface(struct A314Device *dev)
{
	// AddMemList(0xe9xxxx, 15kb, MEMF_A314)
	// Add IRQ Server (clear Amiga IRQ, AddIntHandler)
	// Update Base Address and Event Head/Tail
}


void read_from_r2a(struct A314Device *dev, UBYTE *dst, UBYTE offset, int length)
{
	UBYTE *r2a_buffer = dev->ca->r2a_buffer;
	for (int i = 0; i < length; i++)
		*dst++ = r2a_buffer[offset++];
}

void write_to_a2r(struct A314Device *dev, UBYTE type, UBYTE stream_id, UBYTE length, UBYTE *data)
{
	struct ComArea *ca = dev->ca;
	UBYTE index = ca->cap.a2r_tail;
	UBYTE *a2r_buffer = ca->a2r_buffer;
	a2r_buffer[index++] = length;
	a2r_buffer[index++] = type;
	a2r_buffer[index++] = stream_id;
	for (int i = 0; i < (int)length; i++)
		a2r_buffer[index++] = *data++;
	ca->cap.a2r_tail = index;
}


static ULONG cpu_to_a314_address(__reg("a6") struct A314Device *dev, __reg("a0") void *address)
{
	// convert to E9 base
	return 0;
}

static void *a314_to_cpu_address(__reg("a6") struct A314Device *dev, __reg("d0") ULONG address)
{
	// convert from E9 base
	return 0;
}

ULONG a314base_translate_address(__reg("a6") struct A314Device *dev, __reg("a0") void *address)
{
	return cpu_to_a314_address(dev, address);
}

ULONG a314base_alloc_mem(__reg("a6") struct A314Device *dev, __reg("d0") ULONG length)
{
	void *p = AllocMem(length, MEMF_A314);
	if (!p)
		return INVALID_A314_ADDRESS;
	return cpu_to_a314_address(dev, p);
}

void a314base_free_mem(__reg("a6") struct A314Device *dev, __reg("d0") ULONG address, __reg("d1") ULONG length)
{
	void *p = a314_to_cpu_address(dev, address);
	FreeMem(p, length);
}

void a314base_write_mem(__reg("a6") struct A314Device *dev, __reg("d0") ULONG address, __reg("a0") UBYTE *src, __reg("d1") ULONG length)
{
	UBYTE *dst = a314_to_cpu_address(dev, address);
	memcpy(dst, src, length);
}

void a314base_read_mem(__reg("a6") struct A314Device *dev, __reg("a0") UBYTE *dst, __reg("d0") ULONG address, __reg("d1") ULONG length)
{
	UBYTE *src = a314_to_cpu_address(dev, address);
	memcpy(dst, src, length);
}

// unused? (cp version)
// extern void read_pi_cap(struct A314Device *dev)
// extern void write_amiga_cap(struct A314Device *dev);

void write_cp_nibble(int index, UBYTE value)
{
	// E9 config + reg
}
UBYTE read_cp_nibble(int index)
{
	// E9 config + reg
	return 0;
}

void write_cmem_safe(int index, UBYTE value)
{
	// Disable
	// write_cp_nibble
	// Enable
}


UBYTE read_cmem_safe(int index)
{
	// Disable
	// read_cp_nibble
	// Enable
	return 0;
}
