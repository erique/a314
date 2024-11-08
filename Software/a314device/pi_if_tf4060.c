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
#include "cmem.h"

#include <libraries/expansion.h>
#include <libraries/expansionbase.h>
#include <proto/expansion.h>
#include "tfconfig.h"

#define SysBase (*(struct ExecBase **)4)

/*
0xE903F0 - 0E903FF  CMEM Registers (upper 4 bits only)
0xE90400 - 0E93FFF  SRAM
*/

static volatile UBYTE* cmem = 0;
//#define CMEM_PTR(reg) ((volatile UBYTE *)(cpa + (reg << 2)))

#define CMEM       0x03F0
#define SRAM_START 0x0400
#define SRAM_END   0x4000


int probe_pi_interface(struct A314Device *dev)
{
    dev->tf_config = NULL;

    struct ExpansionBase* ExpansionBase = (struct ExpansionBase*)OpenLibrary(EXPANSIONNAME, 39);
    kprintf("ExpansionBase = %08lx\n", ExpansionBase);
    if (!ExpansionBase)
    {
        kprintf("OpenLibrary() failed\n");
        return FALSE;
    }
    struct ConfigDev* configDev = FindConfigDev(NULL, TF_MANU_ID, 0x94);
    kprintf("configDev = %08lx\n", configDev);
    if (!configDev)
    {
        kprintf("FindConfigDev() failed\n");
    }
    else
    {
	    struct TFConfig* tfConfig = configDev->cd_BoardAddr;
	    kprintf("tfConfig = %08lx\n", tfConfig);
	    if (!tfConfig)
	    {
	        kprintf("cd_BoardAddr is null\n");
	    }
	    dev->tf_config = tfConfig;
    }

    CloseLibrary((struct Library*)ExpansionBase);

    if (!dev->tf_config)
    {
    	dbg_error("Unable to find AutoConf board");
    	return FALSE;
    }

    cmem = (void*)(((intptr_t)dev->tf_config) + CMEM);

    const void* sram = (void*)(((intptr_t)dev->tf_config) + SRAM_START);
    const ULONG sram_size = SRAM_END - SRAM_START;

	AddMemList( sram_size, MEMF_A314, -128, sram, "A314-TF4060" );

	dev->ca = (struct ComArea *)AllocMem(sizeof(struct ComArea), MEMF_A314 | MEMF_CLEAR);
	if (dev->ca == NULL)
	{
		dbg_error("Unable to allocate A314 memory for com area\n");
		return FALSE;
	}

	return TRUE;
}

extern void IntServer();

static void add_interrupt_handlers(struct A314Device *dev)
{
	memset(&dev->vertb_interrupt, 0, sizeof(struct Interrupt));
	dev->vertb_interrupt.is_Node.ln_Type = NT_INTERRUPT;
	dev->vertb_interrupt.is_Node.ln_Pri = -60;
	dev->vertb_interrupt.is_Node.ln_Name = device_name;
	dev->vertb_interrupt.is_Data = (APTR)&dev->task;
	dev->vertb_interrupt.is_Code = IntServer;

	AddIntServer(INTB_VERTB, &dev->vertb_interrupt);

	memset(&dev->int_x_interrupt, 0, sizeof(struct Interrupt));
	dev->int_x_interrupt.is_Node.ln_Type = NT_INTERRUPT;
	dev->int_x_interrupt.is_Node.ln_Pri = 0;
	dev->int_x_interrupt.is_Node.ln_Name = device_name;
	dev->int_x_interrupt.is_Data = (APTR)&dev->task;
	dev->int_x_interrupt.is_Code = IntServer;

	LONG int_num = dev->interrupt_number == 6 ? INTB_EXTER : INTB_PORTS;
	AddIntServer(int_num, &dev->int_x_interrupt);
}

void setup_pi_interface(struct A314Device *dev)
{
	write_cmem_safe(A_ENABLE_ADDRESS, 0);
	read_cmem_safe(A_EVENTS_ADDRESS);

	write_base_address(a314base_translate_address(dev, dev->ca));

	write_cmem_safe(R_EVENTS_ADDRESS, R_EVENT_BASE_ADDRESS);

	add_interrupt_handlers(dev);

	write_cmem_safe(A_ENABLE_ADDRESS, A_EVENT_R2A_TAIL);
}


void read_from_r2a(struct A314Device *dev, UBYTE *dst, UBYTE offset, int length)
{
	kprintf("read_from_r2a: length = %ld ; dst = %lx\n", length, dst);
	UBYTE *r2a_buffer = dev->ca->r2a_buffer;
	DumpBuffer(&r2a_buffer[offset], length);
	for (int i = 0; i < length; i++)
		*dst++ = r2a_buffer[offset++];
}

void write_to_a2r(struct A314Device *dev, UBYTE type, UBYTE stream_id, UBYTE length, UBYTE *data)
{
	kprintf("write_to_a2r: type=%lx ; stream=%lx ; length = %ld ; data = %lx\n", type, stream_id, length, data);
	DumpBuffer(data, length);

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
    const void* sram_lo = (void*)(((intptr_t)dev->tf_config) + SRAM_START);
    const void* sram_hi = (void*)(((intptr_t)dev->tf_config) + SRAM_END);

	if (sram_lo <= address && address < sram_hi)
	{
		ULONG ret = (intptr_t)address - (intptr_t)sram_lo;
		kprintf("cpu_to_a314: %08lx => %08lx\n", address, ret);
		return (intptr_t)address - (intptr_t)sram_lo;
	}

	kprintf("cpu_to_a314: %08lx is not valid!\n", address);

	return INVALID_A314_ADDRESS;
}

static void *a314_to_cpu_address(__reg("a6") struct A314Device *dev, __reg("d0") ULONG address)
{
    const ULONG sram_lo = SRAM_START;
    const ULONG sram_hi = SRAM_END;

	if (sram_lo <= address && address < sram_hi)
	{
		void* ret = (void*)((intptr_t)dev->tf_config + SRAM_START + address);
		kprintf("a314_to_cpu: %08lx <= %08lx\n", ret, address);
		return (void*)((intptr_t)dev->tf_config + SRAM_START + address);
	}

	kprintf("a314_to_cpu: %08lx is not valid!\n", address);

	return 0/*INVALID_A314_ADDRESS*/;
}

ULONG a314base_translate_address(__reg("a6") struct A314Device *dev, __reg("a0") void *address)
{
	return cpu_to_a314_address(dev, address);
}

ULONG a314base_alloc_mem(__reg("a6") struct A314Device *dev, __reg("d0") ULONG length)
{
	kprintf("a314base_alloc_mem: %ld bytes\n", length);
	void *p = AllocMem(length, MEMF_A314);
	if (!p)
		return INVALID_A314_ADDRESS;
	return cpu_to_a314_address(dev, p);
}

void a314base_free_mem(__reg("a6") struct A314Device *dev, __reg("d0") ULONG address, __reg("d1") ULONG length)
{
	kprintf("a314base_free_mem: %08lx / %ld bytes\n", address, length);
	void *p = a314_to_cpu_address(dev, address);
	FreeMem(p, length);
}

void a314base_write_mem(__reg("a6") struct A314Device *dev, __reg("d0") ULONG address, __reg("a0") UBYTE *src, __reg("d1") ULONG length)
{
	kprintf("a314base_write_mem: %08lx / %ld bytes\n", address, length);
	UBYTE *dst = a314_to_cpu_address(dev, address);
	memcpy(dst, src, length);
	DumpBuffer(dst, length);
}

void a314base_read_mem(__reg("a6") struct A314Device *dev, __reg("a0") UBYTE *dst, __reg("d0") ULONG address, __reg("d1") ULONG length)
{
	kprintf("a314base_read_mem: %08lx / %ld bytes\n", address, length);
	UBYTE *src = a314_to_cpu_address(dev, address);
	memcpy(dst, src, length);
	DumpBuffer(dst, length);
}

// unused? (cp version)
// extern void read_pi_cap(struct A314Device *dev)
// extern void write_amiga_cap(struct A314Device *dev);

void write_cp_nibble(int index, UBYTE value)
{
	kprintf("write_cp_nibble: [%ld] <= %lx\n", index, value);
	volatile UBYTE *p = cmem;
	p += index;
	*p = value & 0xf;
}
UBYTE read_cp_nibble(int index)
{
	volatile UBYTE *p = cmem;
	p += index;
	UBYTE ret = *p & 0xf;
	kprintf("read_cp_nibble: [%ld] => %lx\n", index, ret);
	return ret;
}

void write_cmem_safe(int index, UBYTE value)
{
	Disable();
	UBYTE prev_regd = read_cp_nibble(13);
	write_cp_nibble(13, prev_regd | 8);
	write_cp_nibble(index, value);
	write_cp_nibble(13, prev_regd);
	Enable();
}

UBYTE read_cmem_safe(int index)
{
	Disable();
	UBYTE prev_regd = read_cp_nibble(13);
	write_cp_nibble(13, prev_regd | 8);
	UBYTE value = read_cp_nibble(index);
	write_cp_nibble(13, prev_regd);
	Enable();
	return value;
}

#define BASE_ADDRESS_LEN	6

void write_base_address(ULONG ba)
{
	ba |= 1;

	Disable();
	UBYTE prev_regd = read_cp_nibble(13);
	write_cp_nibble(13, prev_regd | 8);

	write_cp_nibble(0, 0);

	for (int i = BASE_ADDRESS_LEN - 1; i >= 0; i--)
	{
		ULONG v = (ba >> (i * 4)) & 0xf;
		write_cp_nibble(i, (UBYTE)v);
	}

	write_cp_nibble(13, prev_regd);
	Enable();
}

ULONG read_fw_flags()
{
	Disable();
	UBYTE prev_regd = read_cp_nibble(13);
	write_cp_nibble(13, prev_regd | 8);

	write_cp_nibble(10, 0);

	ULONG flags = 0;
	for (int i = 0; i < 4; i++)
		flags |= ((ULONG)read_cp_nibble(10)) << (4 * i);

	write_cp_nibble(13, prev_regd);
	Enable();

	return flags;
}
