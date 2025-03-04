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
0xE903F4            SINT
0xE90400 - 0E93FFF  SRAM
*/

#define SINT       0x03F4
#define SRAM_START 0x0400
#define SRAM_END   0x4000

// SINT:
//
// Bit 7 Set/Clear on write 
// Bit 6 Amiga Interrupts Enabled (INT2). 1 = Enabled, 0 = Disabled
// Bit 5 RPi (Remote Interrupt). 1 = Pending, 0 = Not Pending (Amiga Can set but not Clear, RPi Can clear but not set)
// Bit 4 Amiga Interrupt.  1 = Pending, 0 = Not Pending (Amiga can clear but not set, RPi Can set but not clear)
// Bit 3 Unused
// Bit 2 Unused
// Bit 1 Unused
// Bit 0 Reset Event 

#define REG_IRQ_SET             0x80
#define REG_IRQ_CLR             0x00
#define REG_IRQ_INTENA          0x40
#define REG_IRQ_RPI             0x20
#define REG_IRQ_AMIGA           0x10
#define REG_IRQ_RESET           0x01

void set_pi_irq(struct A314Device *dev)
{
	volatile UBYTE* sint = (void*)(((intptr_t)dev->tf_config) + SINT);
	*sint = REG_IRQ_SET | REG_IRQ_RPI;
}

void clear_amiga_irq(struct A314Device *dev)
{
	volatile UBYTE* sint = (void*)(((intptr_t)dev->tf_config) + SINT);
	*sint = REG_IRQ_AMIGA;
}

static void clear_reset(struct A314Device *dev)
{
	volatile UBYTE* sint = (void*)(((intptr_t)dev->tf_config) + SINT);
	*sint = REG_IRQ_RESET;
}

void enable_amiga_irq(struct A314Device *dev)
{
	volatile UBYTE* sint = (void*)(((intptr_t)dev->tf_config) + SINT);
	*sint = REG_IRQ_SET | REG_IRQ_INTENA;
}

static void spiBegin(struct TFConfig* tfConfig)             { tfConfig->TF_SpiCtrl = 0x00; }
static void spiEnd(struct TFConfig* tfConfig)               { tfConfig->TF_SpiCtrl = 0xFF; }

#define MAPP_CACHEINHIBIT     (1<<6)
#define MAPP_IO               (1<<30)

uint32_t SetMMU(__reg("a0") void* addr, __reg("d0") uint32_t size, __reg("d1") uint32_t flags, __reg("a6") struct ExecBase*);

static void *a314_to_cpu_address(__reg("a6") struct A314Device *dev, __reg("d0") ULONG address);

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

    const void* sram = (void*)(((intptr_t)dev->tf_config) + SRAM_START);
    const ULONG sram_size = SRAM_END - SRAM_START;

	// dev->ca = (struct ComArea *)AllocMem(sizeof(struct ComArea), MEMF_A314 | MEMF_CLEAR);
	dev->ca = sram;
	if (dev->ca == NULL)
	{
		dbg_error("Unable to allocate A314 memory for com area\n");
		return FALSE;
	}
	memset(dev->ca, 0, sizeof(struct ComArea));

	const uint32_t ca_cutout = 1024;
	if (ca_cutout < sizeof(struct ComArea))
	{
		dbg_error("A314 memory for com area to big!\n");
		return FALSE;
	}

	const void* memstart = (void*)((intptr_t)sram + ca_cutout);
	const ULONG memsize = sram_size - ca_cutout;
	AddMemList( memsize, MEMF_A314, -128, memstart, "A314-TF4060" );

	SetMMU(dev->tf_config, 0x4000, MAPP_IO|MAPP_CACHEINHIBIT, SysBase);

	return TRUE;
}

extern void IntServer();

static void add_interrupt_handlers(struct A314Device *dev)
{
	// memset(&dev->vertb_interrupt, 0, sizeof(struct Interrupt));
	// dev->vertb_interrupt.is_Node.ln_Type = NT_INTERRUPT;
	// dev->vertb_interrupt.is_Node.ln_Pri = -60;
	// dev->vertb_interrupt.is_Node.ln_Name = device_name;
	// dev->vertb_interrupt.is_Data = (APTR)&dev->task;
	// dev->vertb_interrupt.is_Code = IntServer;

	// AddIntServer(INTB_VERTB, &dev->vertb_interrupt);

	memset(&dev->int_x_interrupt, 0, sizeof(struct Interrupt));
	dev->int_x_interrupt.is_Node.ln_Type = NT_INTERRUPT;
	dev->int_x_interrupt.is_Node.ln_Pri = 0;
	dev->int_x_interrupt.is_Node.ln_Name = (char *)device_name;
	dev->int_x_interrupt.is_Data = (APTR)&dev->task;
	dev->int_x_interrupt.is_Code = IntServer;

	dev->interrupt_number = 2;
	LONG int_num = dev->interrupt_number == 6 ? INTB_EXTER : INTB_PORTS;
	AddIntServer(int_num, &dev->int_x_interrupt);
}

void setup_pi_interface(struct A314Device *dev)
{
	clear_amiga_irq(dev);

	add_interrupt_handlers(dev);
	enable_amiga_irq(dev);

	clear_reset(dev);
}


void read_from_r2a(struct A314Device *dev, UBYTE *dst, UBYTE offset, int length)
{
	// kprintf("read_from_r2a: length = %ld ; dst = %lx\n", length, dst);
	UBYTE *r2a_buffer = dev->ca->r2a_buffer;
	// DumpBuffer(&r2a_buffer[offset], length);
	for (int i = 0; i < length; i++)
		*dst++ = r2a_buffer[offset++];
}

void write_to_a2r(struct A314Device *dev, UBYTE type, UBYTE stream_id, UBYTE length, UBYTE *data)
{
	// kprintf("write_to_a2r: type=%lx ; stream=%lx ; length = %ld ; data = %lx\n", type, stream_id, length, data);
	// DumpBuffer(data, length);

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
    const ULONG sram_lo = 0;
    const ULONG sram_hi = SRAM_END - SRAM_START;

	if (address == INVALID_A314_ADDRESS)
	{
		kprintf("a314_to_cpu: using %08lx is not valid!\n", address);
		return 0;
	}

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
	length = ( length + 63 ) & (~63);
	length += 256;
	void *p = AllocMem(length, MEMF_A314 | MEMF_CLEAR);
	if (!p)
		return INVALID_A314_ADDRESS;
	memset(p, 0xcd, length);
	uint32_t* g = (uint32_t*)p;
	*g++ = (uint32_t)p;
	*g++ = length;
	g += (128 / sizeof(uint32_t)) - 2;
	return cpu_to_a314_address(dev, g);
}

void a314base_free_mem(__reg("a6") struct A314Device *dev, __reg("d0") ULONG address, __reg("d1") ULONG length)
{
	kprintf("a314base_free_mem: %08lx / %ld bytes\n", address, length);
	length = ( length + 63 ) & (~63);
	length += 256;
	void *p = a314_to_cpu_address(dev, address);
	uint32_t* g = (uint32_t*)p;
	g -= (128 / sizeof(uint32_t));
	if (g[0] != (uint32_t)p)
	{
		kprintf("***************** %08lx != %08lx\n", p, g[0]);
	}
	if (g[1] != length)
	{
		kprintf("***************** %ld != %ld\n", length, g[1]);
	}
	p = (void*)*g;
	FreeMem(p, length);
}

void a314base_write_mem(__reg("a6") struct A314Device *dev, __reg("d0") ULONG address, __reg("a0") UBYTE *src, __reg("d1") ULONG length)
{
	// kprintf("a314base_write_mem: %08lx / %ld bytes\n", address, length);
	UBYTE *dst = a314_to_cpu_address(dev, address);
	memcpy(dst, src, length);
	// DumpBuffer(dst, length);
}

void a314base_read_mem(__reg("a6") struct A314Device *dev, __reg("a0") UBYTE *dst, __reg("d0") ULONG address, __reg("d1") ULONG length)
{
	// kprintf("a314base_read_mem: %08lx / %ld bytes\n", address, length);
	UBYTE *src = a314_to_cpu_address(dev, address);
	memcpy(dst, src, length);
	// DumpBuffer(dst, length);
}
