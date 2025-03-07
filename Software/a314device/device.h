#ifndef __A314_DEVICE_H
#define __A314_DEVICE_H

#include <exec/types.h>
#include <exec/libraries.h>
#include <exec/tasks.h>
#include <exec/ports.h>
#include <exec/interrupts.h>
#include <libraries/dos.h>

#include "protocol.h"

struct A314Device
{
	struct Library lib; // sizeof(struct Library) == 34

	BPTR saved_seg_list;
	BOOL running;

#if defined(MODEL_TD) || defined(MODEL_FE)
	struct ComArea *ca; // offsetof(ca) == 40
#elif defined(MODEL_CP)
	ULONG clockport_address; // offsetof(clockport_address) == 40
#elif defined(MODEL_TF)
	APTR tf_config;  // offsetof(tf_config) == 40
#endif

	struct Task task; // offsetof(task) == 44

#if defined(MODEL_TD)
	ULONG bank_address[4];
	UWORD is_a600;

	ULONG fw_flags;
#elif defined(MODEL_FE)
	ULONG a314_mem_address;
#endif

#if defined(MODEL_TD) || defined(MODEL_FE)
	struct Interrupt vertb_interrupt;
#endif

	struct Interrupt int_x_interrupt;
	UWORD interrupt_number;

#if defined(MODEL_CP)
	struct ComAreaPtrs cap;

	void *first_chunk;
#elif defined(MODEL_TF)
	struct ComArea *ca;
#endif

	struct MsgPort task_mp;

	struct List active_sockets;

	struct Socket *send_queue_head;
	struct Socket *send_queue_tail;

	UBYTE next_stream_id;
};

#if defined(MODEL_TD) || defined(MODEL_FE) || defined(MODEL_TF)
#define CAP_PTR(dev) (&(dev->ca->cap))
#elif defined(MODEL_CP)
#define CAP_PTR(dev) (&(dev->cap))
#endif

extern const char device_name[];
extern const char id_string[];

#endif /* __A314_DEVICE_H */
