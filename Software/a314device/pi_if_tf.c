#include <exec/types.h>
#include <exec/memory.h>
#include <exec/tasks.h>
#include <hardware/intbits.h>

#include <proto/exec.h>
#include <devices/timer.h>

#include <proto/alib.h>

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

#define SysBase (*(struct ExecBase **)4)

#define TF_MANU_ID 0x13D8
#define PRODUCTID 0x83

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

static int check_pi_irq(struct A314Device *dev)
{
	volatile UBYTE* sint = (void*)(((intptr_t)dev->tf_config) + SINT);
	return (*sint & REG_IRQ_RPI) != 0 ? TRUE : FALSE;
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

#define MAPP_CACHEINHIBIT     (1<<6)
#define MAPP_IO               (1<<30)

uint32_t SetMMU(__reg("a0") void* addr, __reg("d0") uint32_t size, __reg("d1") uint32_t flags, __reg("a6") struct ExecBase*);

static void *a314_to_cpu_address(__reg("a6") struct A314Device *dev, __reg("d0") ULONG address);

static int delay_1s()
{
	int success = FALSE;

	struct timerequest tr;
	memset(&tr, 0x00, sizeof(struct timerequest));

	struct MsgPort mp;
	memset(&mp, 0x00, sizeof(struct MsgPort));

	mp.mp_Node.ln_Type = NT_MSGPORT;
	mp.mp_Flags = PA_SIGNAL;
	mp.mp_SigTask = FindTask(NULL);
	mp.mp_SigBit = SIGB_SINGLE;
	NewList(&mp.mp_MsgList);

	if (OpenDevice(TIMERNAME, UNIT_VBLANK, (struct IORequest *)&tr, 0))
		return FALSE;

	tr.tr_node.io_Message.mn_Node.ln_Type = NT_REPLYMSG;
	tr.tr_node.io_Message.mn_ReplyPort = &mp;
	tr.tr_node.io_Message.mn_Length = sizeof(sizeof(struct timerequest));
	tr.tr_node.io_Command = TR_ADDREQUEST;
	tr.tr_time.tv_secs = 1;
	DoIO((struct IORequest *)&tr);

	return TRUE;
}

int probe_pi_interface(struct A314Device *dev)
{
    dev->tf_config = NULL;

    struct ExpansionBase* ExpansionBase = (struct ExpansionBase*)OpenLibrary(EXPANSIONNAME, 39);
    if (!ExpansionBase)
    {
		dbg_error("Unable to OpenLibrary('expansion')");
        return FALSE;
    }
    struct ConfigDev* configDev = FindConfigDev(NULL, TF_MANU_ID, 0x94);
    dbg_trace("configDev = $l", configDev);
    if (!configDev)
    {
		dbg_error("FindConfigDev() failed");
    }
    else
    {
	    struct TFConfig* tfConfig = configDev->cd_BoardAddr;
	    dbg_info("tfConfig = $l", tfConfig);
	    if (!tfConfig)
	    {
	        dbg_warning("cd_BoardAddr is null");
	    }
	    dev->tf_config = tfConfig;
    }

    CloseLibrary((struct Library*)ExpansionBase);

    if (!dev->tf_config)
    {
    	dbg_error("Unable to find AutoConf board");
    	return FALSE;
    }

	dbg_info("Probing PI IRQ...");
	set_pi_irq(dev);
	for(short timeout = 3; timeout >= 0; timeout--)
	{
		if (!check_pi_irq(dev))
			break;
		dbg_info("PI IRQ not cleared; delaying...");
		delay_1s();
	}
	if (check_pi_irq(dev))
	{
		dbg_error("No a314d running on the PI ?!");
		return FALSE;
	}

	dbg_info("a314d is running!");

    const void* sram = (void*)(((intptr_t)dev->tf_config) + SRAM_START);
    const ULONG sram_size = SRAM_END - SRAM_START;

	// dev->ca = (struct ComArea *)AllocMem(sizeof(struct ComArea), MEMF_A314 | MEMF_CLEAR);
	dev->ca = sram;
	if (dev->ca == NULL)
	{
		dbg_error("Unable to allocate A314 memory for com area");
		return FALSE;
	}
	memset(dev->ca, 0, sizeof(struct ComArea));

	const uint32_t ca_cutout = 1024;
	if (ca_cutout < sizeof(struct ComArea))
	{
		dbg_error("A314 memory for com area too big!");
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
	memset(&dev->int_x_interrupt, 0, sizeof(struct Interrupt));
	dev->int_x_interrupt.is_Node.ln_Type = NT_INTERRUPT;
	dev->int_x_interrupt.is_Node.ln_Pri = 0;
	dev->int_x_interrupt.is_Node.ln_Name = (char *)device_name;
	dev->int_x_interrupt.is_Data = (APTR)dev;
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
    const void* sram_lo = (void*)(((intptr_t)dev->tf_config) + SRAM_START);
    const void* sram_hi = (void*)(((intptr_t)dev->tf_config) + SRAM_END);

	if (sram_lo <= address && address < sram_hi)
	{
		ULONG ret = (intptr_t)address - (intptr_t)sram_lo;
		dbg_trace("cpu_to_a314: $l => $l", address, ret);
		return (intptr_t)address - (intptr_t)sram_lo;
	}

	dbg_error("cpu_to_a314: $l is not valid!", address);

	return INVALID_A314_ADDRESS;
}

static void *a314_to_cpu_address(__reg("a6") struct A314Device *dev, __reg("d0") ULONG address)
{
    const ULONG sram_lo = 0;
    const ULONG sram_hi = SRAM_END - SRAM_START;

	if (address == INVALID_A314_ADDRESS)
	{
		dbg_error("a314_to_cpu: using $l is not valid!", address);
		return 0;
	}

	if (sram_lo <= address && address < sram_hi)
	{
		void* ret = (void*)((intptr_t)dev->tf_config + SRAM_START + address);
		dbg_trace("a314_to_cpu: $l <= $l", ret, address);
		return (void*)((intptr_t)dev->tf_config + SRAM_START + address);
	}

	dbg_error("a314_to_cpu: $l is not valid!", address);

	return 0/*INVALID_A314_ADDRESS*/;
}

ULONG a314base_translate_address(__reg("a6") struct A314Device *dev, __reg("a0") void *address)
{
	return cpu_to_a314_address(dev, address);
}

ULONG a314base_alloc_mem(__reg("a6") struct A314Device *dev, __reg("d0") ULONG length)
{
	dbg_trace("a314base_alloc_mem: $l bytes", length);
	length = ( length + 63 ) & (~63);
	void *p = AllocMem(length, MEMF_A314 | MEMF_CLEAR);
	if (!p)
		return INVALID_A314_ADDRESS;
	return cpu_to_a314_address(dev,  p);
}

void a314base_free_mem(__reg("a6") struct A314Device *dev, __reg("d0") ULONG address, __reg("d1") ULONG length)
{
	dbg_trace("a314base_free_mem: $l bytes", address, length);
	length = ( length + 63 ) & (~63);
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
