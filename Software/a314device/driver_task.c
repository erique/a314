/*
 * Copyright (c) 2018 Niklas Ekström
 */

#include <exec/types.h>
#include <exec/interrupts.h>
#include <exec/lists.h>
#include <exec/memory.h>
#include <exec/nodes.h>
#include <exec/ports.h>
#include <exec/io.h>
#include <exec/errors.h>
#include <exec/libraries.h>
#include <exec/devices.h>
#include <exec/execbase.h>

#include <libraries/dos.h>

#include <proto/exec.h>

#include <string.h>

#include "a314.h"
#include "debug.h"
#include "device.h"
#include "pi_if.h"
#include "protocol.h"
#include "sockets.h"
#include "startup.h"

#if defined(MODEL_TD)
#include "cmem.h"
#elif defined(MODEL_FE)
#include "handshake_fe.h"
#endif

// VBCC unnecessairly warns when a loop has a single iteration.
#pragma dontwarn 208

#define SysBase (*(struct ExecBase **)4)

static int used_in_r2a(struct ComAreaPtrs *cap)
{
	return (cap->r2a_tail - cap->r2a_head) & 255;
}

static int used_in_a2r(struct ComAreaPtrs *cap)
{
	return (cap->a2r_tail - cap->a2r_head) & 255;
}

static BOOL room_in_a2r(struct ComAreaPtrs *cap, int len)
{
	return used_in_a2r(cap) + 3 + len <= 255;
}

static void close_socket(struct A314Device *dev, struct Socket *s, BOOL should_send_reset)
{
	dbg_trace("Enter: close_socket");

	// A CONNECT still in the send queue was never written to a2r, so the Pi
	// has never seen this stream: there is nothing to reset, and keeping the
	// socket until a PKT_RESET goes out would leak it (and its stream id)
	// for as long as the Pi is not reading a2r.
	if (s->pending_connect != NULL && (s->flags & SOCKET_IN_SEND_QUEUE))
		should_send_reset = FALSE;

	if (s->pending_connect != NULL)
	{
		struct A314_IORequest *ior = s->pending_connect;
		ior->a314_Request.io_Error = A314_CONNECT_RESET;
		ReplyMsg((struct Message *)ior);

		s->pending_connect = NULL;
	}

	if (s->pending_read != NULL)
	{
		struct A314_IORequest *ior = s->pending_read;
		ior->a314_Length = 0;
		ior->a314_Request.io_Error = A314_READ_RESET;
		ReplyMsg((struct Message *)ior);

		s->pending_read = NULL;
	}

	if (s->pending_write != NULL)
	{
		struct A314_IORequest *ior = s->pending_write;
		ior->a314_Length = 0;
		ior->a314_Request.io_Error = A314_WRITE_RESET; // A314_EOS_RESET == A314_WRITE_RESET
		ReplyMsg((struct Message *)ior);

		s->pending_write = NULL;
	}

	if (s->rq_head != NULL)
	{
		struct QueuedData *qd = s->rq_head;
		while (qd != NULL)
		{
			struct QueuedData *next = qd->next;
			FreeMem(qd, sizeof(struct QueuedData) + qd->length);
			qd = next;
		}
		s->rq_head = NULL;
		s->rq_tail = NULL;
	}

	remove_from_send_queue(dev, s);

	// No operations can be pending when SOCKET_CLOSED is set.
	// However, may not be able to delete socket yet, because is waiting to send PKT_RESET.
	s->flags |= SOCKET_CLOSED;

	BOOL should_delete_socket = TRUE;

	if (should_send_reset)
	{
		if (dev->send_queue_head == NULL && room_in_a2r(CAP_PTR(dev), 0))
		{
			write_to_a2r(dev, PKT_RESET, s->stream_id, 0, NULL);
		}
		else
		{
			s->flags |= SOCKET_SHOULD_SEND_RESET;
			add_to_send_queue(dev, s, 0);
			should_delete_socket = FALSE;
		}
	}

	if (should_delete_socket)
		delete_socket(dev, s);
}

static void handle_pkt_connect_response(struct A314Device *dev, UBYTE offset, UBYTE length, struct Socket *s)
{
	dbg_trace("Enter: handle_pkt_connect_response");

	if (s->pending_connect == NULL)
	{
		dbg_error("SERIOUS ERROR: received a CONNECT RESPONSE even though no connect was pending");
		// Should reset stream?
	}
	else if (length != 1)
	{
		dbg_error("SERIOUS ERROR: received a CONNECT RESPONSE whose length was not 1");
		// Should reset stream?
	}
	else
	{
		UBYTE result;
		read_from_r2a(dev, &result, offset, 1);

		if (result == 0)
		{
			struct A314_IORequest *ior = s->pending_connect;
			ior->a314_Request.io_Error = A314_CONNECT_OK;
			ReplyMsg((struct Message *)ior);

			s->pending_connect = NULL;
		}
		else
		{
			struct A314_IORequest *ior = s->pending_connect;
			ior->a314_Request.io_Error = A314_CONNECT_UNKNOWN_SERVICE;
			ReplyMsg((struct Message *)ior);

			s->pending_connect = NULL;

			close_socket(dev, s, FALSE);
		}
	}
}

static void handle_pkt_data(struct A314Device *dev, UBYTE offset, UBYTE length, struct Socket *s)
{
	dbg_trace("Enter: handle_pkt_data");

	if (s->pending_read != NULL)
	{
		struct A314_IORequest *ior = s->pending_read;

		if (ior->a314_Length < length)
			close_socket(dev, s, TRUE);
		else
		{
			UBYTE *dst = ior->a314_Buffer;
			read_from_r2a(dev, dst, offset, length);
			ior->a314_Length = length;
			ior->a314_Request.io_Error = A314_READ_OK;
			ReplyMsg((struct Message *)ior);

			s->pending_read = NULL;
		}
	}
	else
	{
		struct QueuedData *qd = (struct QueuedData *)AllocMem(sizeof(struct QueuedData) + length, 0);
		qd->next = NULL,
		qd->length = length;

		UBYTE *dst = qd->data;
		read_from_r2a(dev, dst, offset, length);

		if (s->rq_head == NULL)
			s->rq_head = qd;
		else
			s->rq_tail->next = qd;
		s->rq_tail = qd;
	}
}

static void handle_pkt_eos(struct A314Device *dev, struct Socket *s)
{
	dbg_trace("Enter: handle_pkt_eos");

	s->flags |= SOCKET_RCVD_EOS_FROM_RPI;

	if (s->pending_read != NULL)
	{
		struct A314_IORequest *ior = s->pending_read;
		ior->a314_Length = 0;
		ior->a314_Request.io_Error = A314_READ_EOS;
		ReplyMsg((struct Message *)ior);

		s->pending_read = NULL;

		s->flags |= SOCKET_SENT_EOS_TO_APP;

		if (s->flags & SOCKET_SENT_EOS_TO_RPI)
			close_socket(dev, s, FALSE);
	}
}

static void handle_r2a_packet(struct A314Device *dev, UBYTE type, UBYTE stream_id, UBYTE offset, UBYTE length)
{
	dbg_trace("Enter: handle_r2a_packet, type=$b, stream_id=$b, offset=$b, length=$b", type, stream_id, offset, length);

	struct Socket *s = find_socket_by_stream_id(dev, stream_id);

	// A socket whose CONNECT is still in the send queue has a stream id the Pi
	// has not seen: any packet for it (RESET, CONNECT_RESPONSE, DATA) is left
	// over from a channel with the same id before an Amiga reset.
	if (s != NULL && s->pending_connect != NULL && (s->flags & SOCKET_IN_SEND_QUEUE))
		return;

	if (s != NULL && type == PKT_RESET)
	{
		close_socket(dev, s, FALSE);
		return;
	}

	if (s == NULL || (s->flags & SOCKET_CLOSED))
	{
		// Ignore this packet. The only packet that can do anything useful on a closed
		// channel is CONNECT, which is not handled at this time.
		return;
	}

	// Until its CONNECT is answered a stream has no data: a314d sends the
	// CONNECT_RESPONSE first. DATA or EOS now is left over from a channel
	// with the same id before an Amiga reset, so it is not for this socket.
	if (s->pending_connect != NULL && type != PKT_CONNECT_RESPONSE)
		return;

	if (type == PKT_CONNECT_RESPONSE)
	{
		handle_pkt_connect_response(dev, offset, length, s);
	}
	else if (type == PKT_DATA)
	{
		handle_pkt_data(dev, offset, length, s);
	}
	else if (type == PKT_EOS)
	{
		handle_pkt_eos(dev, s);
	}
}

// Close every socket, so the requests pending on them are replied: a314d is
// gone (TF), or r2a was corrupt. Each one is reset: its PKT_RESET goes into a2r
// at once if there is room, otherwise the socket and its stream id are kept
// until it can be sent. Either way a2r has the PKT_RESET ahead of any CONNECT
// that reuses the id, so a314d closes the old channel first.
static void close_all_sockets(struct A314Device *dev)
{
	struct Node *node = dev->active_sockets.lh_Head;
	while (node->ln_Succ != NULL)
	{
		struct Socket *s = (struct Socket *)node;
		node = node->ln_Succ;

		if (!(s->flags & SOCKET_CLOSED))
			close_socket(dev, s, TRUE);
	}
}

static void handle_packets_received_r2a(struct A314Device *dev)
{
	dbg_trace("Enter: handle_packets_received_r2a");

	int used;
	while ((used = used_in_r2a(CAP_PTR(dev))) != 0)
	{
		UBYTE index = CAP_PTR(dev)->r2a_head;

		struct PktHdr hdr;
		if (used < sizeof(hdr))
			hdr.length = 255;	// can't be a whole packet
		else
			read_from_r2a(dev, (UBYTE *)&hdr, index, sizeof(hdr));

		// A packet longer than what is in the ring is corrupt: a byte-sized head
		// moved by it would end up anywhere (or nowhere, at 256), and this loop
		// would never end. Drop the rest of the ring instead.
		if (sizeof(hdr) + hdr.length > used)
		{
			// What was dropped may have been for any socket, so reset them all:
			// none of them may wait for a packet that is gone.
			dbg_error("Malformed packet in r2a: dropping the rest of it, closing all sockets");
			CAP_PTR(dev)->r2a_head = CAP_PTR(dev)->r2a_tail;
			close_all_sockets(dev);
			break;
		}

		index += sizeof(hdr);

		handle_r2a_packet(dev, hdr.type, hdr.stream_id, index, hdr.length);

		index += hdr.length;
		CAP_PTR(dev)->r2a_head = index;
	}
}

static void handle_room_in_a2r(struct A314Device *dev)
{
	dbg_trace("Enter: handle_room_in_a2r");

	struct ComAreaPtrs *cap = CAP_PTR(dev);

	while (dev->send_queue_head != NULL)
	{
		struct Socket *s = dev->send_queue_head;

		if (!room_in_a2r(cap, s->send_queue_required_length))
			break;

		remove_from_send_queue(dev, s);

		if (s->pending_connect != NULL)
		{
			struct A314_IORequest *ior = s->pending_connect;
			int len = ior->a314_Length;
			write_to_a2r(dev, PKT_CONNECT, s->stream_id, (UBYTE)len, ior->a314_Buffer);
#if defined(MODEL_TF)
			s->connect_sent = dev->poll.ticks;
#endif
		}
		else if (s->pending_write != NULL)
		{
			struct A314_IORequest *ior = s->pending_write;
			int len = ior->a314_Length;

			if (ior->a314_Request.io_Command == A314_WRITE)
			{
				write_to_a2r(dev, PKT_DATA, s->stream_id, (UBYTE)len, ior->a314_Buffer);

				ior->a314_Request.io_Error = A314_WRITE_OK;
				ReplyMsg((struct Message *)ior);

				s->pending_write = NULL;
			}
			else // A314_EOS
			{
				write_to_a2r(dev, PKT_EOS, s->stream_id, 0, NULL);

				ior->a314_Request.io_Error = A314_EOS_OK;
				ReplyMsg((struct Message *)ior);

				s->pending_write = NULL;

				s->flags |= SOCKET_SENT_EOS_TO_RPI;

				if (s->flags & SOCKET_SENT_EOS_TO_APP)
					close_socket(dev, s, FALSE);
			}
		}
		else if (s->flags & SOCKET_SHOULD_SEND_RESET)
		{
			write_to_a2r(dev, PKT_RESET, s->stream_id, 0, NULL);
			delete_socket(dev, s);
		}
		else
		{
			dbg_error("SERIOUS ERROR: Socket was in send queue but has nothing to send");
		}
	}
}

static void handle_app_connect(struct A314Device *dev, struct A314_IORequest *ior, struct Socket *s)
{
	dbg_trace("Enter: handle_app_connect");

	if (s != NULL)
	{
		ior->a314_Request.io_Error = A314_CONNECT_SOCKET_IN_USE;
		ReplyMsg((struct Message *)ior);
	}
	else if (ior->a314_Length + 3 > 255)
	{
		ior->a314_Request.io_Error = A314_CONNECT_RESET;
		ReplyMsg((struct Message *)ior);
	}
	else
	{
		s = create_socket(dev, ior->a314_Request.io_Message.mn_ReplyPort->mp_SigTask, ior->a314_Socket);
		if (s == NULL)
		{
			// No memory, or all stream ids in use.
			ior->a314_Request.io_Error = A314_CONNECT_RESET;
			ReplyMsg((struct Message *)ior);
			return;
		}

		s->pending_connect = ior;
		s->flags = 0;

		int len = ior->a314_Length;
		if (dev->send_queue_head == NULL && room_in_a2r(CAP_PTR(dev), len))
		{
			write_to_a2r(dev, PKT_CONNECT, s->stream_id, (UBYTE)len, ior->a314_Buffer);
#if defined(MODEL_TF)
			s->connect_sent = dev->poll.ticks;
#endif
		}
		else
		{
			add_to_send_queue(dev, s, len);
		}
	}
}

static void handle_app_read(struct A314Device *dev, struct A314_IORequest *ior, struct Socket *s)
{
	dbg_trace("Received a READ request from application");

	if (s == NULL || (s->flags & SOCKET_CLOSED))
	{
		ior->a314_Length = 0;
		ior->a314_Request.io_Error = A314_READ_RESET;
		ReplyMsg((struct Message *)ior);
	}
	else
	{
		if (s->pending_connect != NULL || s->pending_read != NULL)
		{
			ior->a314_Length = 0;
			ior->a314_Request.io_Error = A314_READ_RESET;
			ReplyMsg((struct Message *)ior);

			close_socket(dev, s, TRUE);
		}
		else if (s->rq_head != NULL)
		{
			struct QueuedData *qd = s->rq_head;
			int len = qd->length;

			if (ior->a314_Length < len)
			{
				ior->a314_Length = 0;
				ior->a314_Request.io_Error = A314_READ_RESET;
				ReplyMsg((struct Message *)ior);

				close_socket(dev, s, TRUE);
			}
			else
			{
				s->rq_head = qd->next;
				if (s->rq_head == NULL)
					s->rq_tail = NULL;

				memcpy(ior->a314_Buffer, qd->data, len);
				FreeMem(qd, sizeof(struct QueuedData) + len);

				ior->a314_Length = len;
				ior->a314_Request.io_Error = A314_READ_OK;
				ReplyMsg((struct Message *)ior);
			}
		}
		else if (s->flags & SOCKET_RCVD_EOS_FROM_RPI)
		{
			ior->a314_Length = 0;
			ior->a314_Request.io_Error = A314_READ_EOS;
			ReplyMsg((struct Message *)ior);

			s->flags |= SOCKET_SENT_EOS_TO_APP;

			if (s->flags & SOCKET_SENT_EOS_TO_RPI)
				close_socket(dev, s, FALSE);
		}
		else
			s->pending_read = ior;
	}
}

static void handle_app_write(struct A314Device *dev, struct A314_IORequest *ior, struct Socket *s)
{
	dbg_trace("Received a WRITE request from application");

	if (s == NULL || (s->flags & SOCKET_CLOSED))
	{
		ior->a314_Length = 0;
		ior->a314_Request.io_Error = A314_WRITE_RESET;
		ReplyMsg((struct Message *)ior);
	}
	else
	{
		int len = ior->a314_Length;
		if (s->pending_connect != NULL || s->pending_write != NULL || (s->flags & SOCKET_RCVD_EOS_FROM_APP) || len + 3 > 255)
		{
			ior->a314_Length = 0;
			ior->a314_Request.io_Error = A314_WRITE_RESET;
			ReplyMsg((struct Message *)ior);

			close_socket(dev, s, TRUE);
		}
		else
		{
			if (dev->send_queue_head == NULL && room_in_a2r(CAP_PTR(dev), len))
			{
				write_to_a2r(dev, PKT_DATA, s->stream_id, (UBYTE)len, ior->a314_Buffer);

				ior->a314_Request.io_Error = A314_WRITE_OK;
				ReplyMsg((struct Message *)ior);
			}
			else
			{
				s->pending_write = ior;
				add_to_send_queue(dev, s, len);
			}
		}
	}
}

static void handle_app_eos(struct A314Device *dev, struct A314_IORequest *ior, struct Socket *s)
{
	dbg_trace("Received an EOS request from application");

	if (s == NULL || (s->flags & SOCKET_CLOSED))
	{
		ior->a314_Request.io_Error = A314_EOS_RESET;
		ReplyMsg((struct Message *)ior);
	}
	else
	{
		if (s->pending_connect != NULL || s->pending_write != NULL || (s->flags & SOCKET_RCVD_EOS_FROM_APP))
		{
			ior->a314_Length = 0;
			ior->a314_Request.io_Error = A314_EOS_RESET;
			ReplyMsg((struct Message *)ior);

			close_socket(dev, s, TRUE);
		}
		else
		{
			s->flags |= SOCKET_RCVD_EOS_FROM_APP;

			if (dev->send_queue_head == NULL && room_in_a2r(CAP_PTR(dev), 0))
			{
				write_to_a2r(dev, PKT_EOS, s->stream_id, 0, NULL);

				ior->a314_Request.io_Error = A314_EOS_OK;
				ReplyMsg((struct Message *)ior);

				s->flags |= SOCKET_SENT_EOS_TO_RPI;

				if (s->flags & SOCKET_SENT_EOS_TO_APP)
					close_socket(dev, s, FALSE);
			}
			else
			{
				s->pending_write = ior;
				add_to_send_queue(dev, s, 0);
			}
		}
	}
}

static void handle_app_reset(struct A314Device *dev, struct A314_IORequest *ior, struct Socket *s)
{
	dbg_trace("Enter: handle_app_reset");

	if (s == NULL || (s->flags & SOCKET_CLOSED))
	{
		ior->a314_Request.io_Error = A314_RESET_OK;
		ReplyMsg((struct Message *)ior);
	}
	else
	{
		ior->a314_Request.io_Error = A314_RESET_OK;
		ReplyMsg((struct Message *)ior);

		close_socket(dev, s, TRUE);
	}
}

#if defined(MODEL_TF)
// A request marked by AbortIO() (A314_IOF_ABORT) is replied with IOERR_ABORTED.
static BOOL reply_if_aborted(struct A314_IORequest *ior)
{
	if (ior == NULL || !(ior->a314_Request.io_Flags & A314_IOF_ABORT))
		return FALSE;

	ior->a314_Length = 0;
	ior->a314_Request.io_Error = IOERR_ABORTED;
	ReplyMsg((struct Message *)ior);
	return TRUE;
}
#endif

static void handle_app_request(struct A314Device *dev, struct A314_IORequest *ior)
{
	dbg_trace("Enter: handle_app_request");

#if defined(MODEL_TF)
	// Aborted before the driver task took it: nothing was done for it yet.
	if (reply_if_aborted(ior))
		return;
#endif

	struct Socket *s = find_socket(dev, ior->a314_Request.io_Message.mn_ReplyPort->mp_SigTask, ior->a314_Socket);

	switch (ior->a314_Request.io_Command)
	{
	case A314_CONNECT:
		handle_app_connect(dev, ior, s);
		break;
	case A314_READ:
		handle_app_read(dev, ior, s);
		break;
	case A314_WRITE:
		handle_app_write(dev, ior, s);
		break;
	case A314_EOS:
		handle_app_eos(dev, ior, s);
		break;
	case A314_RESET:
		handle_app_reset(dev, ior, s);
		break;
	default:
		ior->a314_Request.io_Error = IOERR_NOCMD;
		ReplyMsg((struct Message *)ior);
		break;
	}
}

#if defined(MODEL_TD)

void task_main()
{
	struct A314Device *dev = (struct A314Device *)FindTask(NULL)->tc_UserData;
	struct ComAreaPtrs *cap = CAP_PTR(dev);

	while (TRUE)
	{
		dbg_trace("Invoking Wait()");

		ULONG signal = Wait(SIGF_MSGPORT | SIGF_INT);

		dbg_trace("Returned from Wait() with signal=$l", signal);

		UBYTE prev_a2r_tail = cap->a2r_tail;
		UBYTE prev_r2a_head = cap->r2a_head;

		if (signal & SIGF_MSGPORT)
		{
			write_cmem_safe(A_ENABLE_ADDRESS, 0);

			struct Message *msg;
			while (msg = GetMsg(&dev->task_mp))
				handle_app_request(dev, (struct A314_IORequest *)msg);
		}

		UBYTE a_enable = 0;
		while (a_enable == 0)
		{
			handle_packets_received_r2a(dev);
			handle_room_in_a2r(dev);

			UBYTE r_events = 0;
			if (cap->a2r_tail != prev_a2r_tail)
				r_events |= R_EVENT_A2R_TAIL;
			if (cap->r2a_head != prev_r2a_head)
				r_events |= R_EVENT_R2A_HEAD;

			Disable();
			UBYTE prev_regd = read_cp_nibble(13);
			write_cp_nibble(13, prev_regd | 8);
			read_cp_nibble(A_EVENTS_ADDRESS);

			if (cap->r2a_head == cap->r2a_tail)
			{
				if (dev->send_queue_head == NULL)
					a_enable = A_EVENT_R2A_TAIL;
				else if (!room_in_a2r(cap, dev->send_queue_head->send_queue_required_length))
					a_enable = A_EVENT_R2A_TAIL | A_EVENT_A2R_HEAD;

				if (a_enable != 0)
				{
					write_cp_nibble(A_ENABLE_ADDRESS, a_enable);
					if (r_events != 0)
						write_cp_nibble(R_EVENTS_ADDRESS, r_events);
				}
			}

			write_cp_nibble(13, prev_regd);
			Enable();
		}
	}

	// There is currently no way to unload a314.device.
}

#elif defined(MODEL_FE)

void task_main()
{
	struct A314Device *dev = (struct A314Device *)FindTask(NULL)->tc_UserData;
	struct ComAreaPtrs *cap = CAP_PTR(dev);

	while (TRUE)
	{
		dbg_trace("Invoking Wait()");

		ULONG signal = Wait(SIGF_MSGPORT | SIGF_INT);

		dbg_trace("Returned from Wait() with signal=$l", signal);

		// Clear enable and event interrupt bits.
		dev->ca->interrupt = 0x0303;

		UBYTE prev_a2r_tail = cap->a2r_tail;
		UBYTE prev_r2a_head = cap->r2a_head;

		if (signal & SIGF_MSGPORT)
		{
			struct Message *msg;
			while (msg = GetMsg(&dev->task_mp))
				handle_app_request(dev, (struct A314_IORequest *)msg);
		}

		UWORD set_int_bits = 0;
		while (!set_int_bits)
		{
			handle_packets_received_r2a(dev);
			handle_room_in_a2r(dev);

			if (cap->r2a_head == cap->r2a_tail)
			{
				if (dev->send_queue_head == NULL)
					set_int_bits = IRQ_R2A_TAIL << 8;
				else if (!room_in_a2r(cap, dev->send_queue_head->send_queue_required_length))
					set_int_bits = (IRQ_R2A_TAIL | IRQ_A2R_HEAD) << 8;
			}
		}

		if (cap->a2r_tail != prev_a2r_tail)
			set_int_bits |= IRQ_A2R_TAIL;
		if (cap->r2a_head != prev_r2a_head)
			set_int_bits |= IRQ_R2A_HEAD;

		dev->ca->interrupt = 0x8000 | set_int_bits;
	}

	// There is currently no way to unload a314.device.
}

#elif defined(MODEL_CP)

void task_main()
{
	struct A314Device *dev = (struct A314Device *)FindTask(NULL)->tc_UserData;

	while (TRUE)
	{
		dbg_trace("Invoking Wait()");

		ULONG signal = Wait(SIGF_MSGPORT | SIGF_INT);

		dbg_trace("Returned from Wait() with signal=$l", signal);

		read_pi_cap(dev);

		dbg_trace("Read CAP, r2a_tail=$b, a2r_head=$b", dev->cap.r2a_tail, dev->cap.a2r_head);

		UBYTE prev_a2r_tail = dev->cap.a2r_tail;
		UBYTE prev_r2a_head = dev->cap.r2a_head;

		// TODO: Perhaps have two separate events for r2a_tail and a2r_head.
		//       Perhaps also have enable/disable those events separately.

		if (signal & SIGF_MSGPORT)
		{
			struct Message *msg;
			while (msg = GetMsg(&dev->task_mp))
				handle_app_request(dev, (struct A314_IORequest *)msg);
		}

		// TODO: May want to read r2a_tail and a2r_head from shared memory again,
		// and process anything left, in order to interrupt less.

		handle_packets_received_r2a(dev);
		handle_room_in_a2r(dev);

		if (prev_a2r_tail != dev->cap.a2r_tail || prev_r2a_head != dev->cap.r2a_head)
		{
			dbg_trace("Writing CAP, a2r_tail=$b, r2a_head=$b", dev->cap.a2r_tail, dev->cap.r2a_head);

			write_amiga_cap(dev);

			set_pi_irq(dev);
		}
	}

	// There is currently no way to unload a314.device.
}

#elif defined(MODEL_TF)

// Dead Pi. a314d clears the Pi interrupt (SINT RPI) whenever it looks at the
// com area, and moves a2r_head or r2a_tail when it takes or sends packets.
// While sockets are open the driver raises RPI at least every KICK_PI_FRAMES.
// RPI is sampled at the start of each pass, before that pass raises it again.
// If for DEAD_PI_FRAMES it is still set and neither ring moved, a314d is gone
// (stopped, crashed, or the Pi rebooted), even when a2r is empty: it may have
// taken a CONNECT or the last DATA before it died. Every socket is then
// closed, so no request waits on it for ever. About 1 s and 5 s at 50 Hz.
#define KICK_PI_FRAMES 50
#define DEAD_PI_FRAMES 250

// A CONNECT that a314d took but that no service answers: a314d itself is
// alive, so the check above can't see it. After this many frames (about 20 s
// at 50 Hz, room for an on-demand service to start) the socket is reset and
// the client gets A314_CONNECT_RESET.
#define CONNECT_TIMEOUT_FRAMES 1000

static void handle_connect_timeouts(struct A314Device *dev)
{
	struct Node *node = dev->active_sockets.lh_Head;
	while (node->ln_Succ != NULL)
	{
		struct Socket *s = (struct Socket *)node;
		node = node->ln_Succ;

		if (!(s->flags & SOCKET_CLOSED) && s->pending_connect != NULL &&
		    !(s->flags & SOCKET_IN_SEND_QUEUE) &&
		    dev->poll.ticks - s->connect_sent > CONNECT_TIMEOUT_FRAMES)
		{
			dbg_warning("CONNECT not answered: resetting its socket");
			close_socket(dev, s, TRUE);
		}
	}
}

// Requests marked by AbortIO() while pending on a socket: reply them with
// IOERR_ABORTED and reset the socket, as the stream state is no longer known.
// (Marked requests still in task_mp are replied by handle_app_request().)
static void handle_aborts(struct A314Device *dev)
{
	struct Node *node = dev->active_sockets.lh_Head;
	while (node->ln_Succ != NULL)
	{
		struct Socket *s = (struct Socket *)node;
		node = node->ln_Succ;

		if (s->flags & SOCKET_CLOSED)
			continue;

		// Decided before pending_connect is cleared (see close_socket()).
		BOOL connect_unsent = s->pending_connect != NULL && (s->flags & SOCKET_IN_SEND_QUEUE);

		BOOL aborted = FALSE;
		if (reply_if_aborted(s->pending_connect))
		{
			s->pending_connect = NULL;
			aborted = TRUE;
		}
		if (reply_if_aborted(s->pending_read))
		{
			s->pending_read = NULL;
			aborted = TRUE;
		}
		if (reply_if_aborted(s->pending_write))
		{
			s->pending_write = NULL;
			aborted = TRUE;
		}

		if (aborted)
		{
			dbg_info("AbortIO: request aborted, resetting its socket");
			close_socket(dev, s, !connect_unsent);
		}
	}
}

// TRUE if a socket is open. A closed one that still waits to send its PKT_RESET
// has no requests pending, so nothing waits on the Pi for it.
static BOOL any_open_socket(struct A314Device *dev)
{
	for (struct Node *node = dev->active_sockets.lh_Head; node->ln_Succ != NULL; node = node->ln_Succ)
	{
		if (!(((struct Socket *)node)->flags & SOCKET_CLOSED))
			return TRUE;
	}
	return FALSE;
}

void task_main()
{
	struct A314Device *dev = (struct A314Device *)FindTask(NULL)->tc_UserData;
	struct ComAreaPtrs *cap = CAP_PTR(dev);

	BOOL irq_waiting = FALSE;	// RPI set, no ring progress, since irq_since
	ULONG irq_since = 0;
	ULONG last_kick = 0;
	UBYTE seen_a2r_head = cap->a2r_head;
	UBYTE seen_r2a_tail = cap->r2a_tail;

	while (TRUE)
	{
		// Poll while anything could be waiting on the Pi.
		dev->poll.active = any_open_socket(dev);

		dbg_trace("Invoking Wait()");

		ULONG signal = Wait(SIGF_MSGPORT | SIGF_INT);

		dbg_trace("Returned from Wait() with signal=$l", signal);

		// For the dead-Pi check: sampled before this pass raises RPI again.
		BOOL pi_irq_set = check_pi_irq(dev);
		BOOL pi_moved = cap->a2r_head != seen_a2r_head || cap->r2a_tail != seen_r2a_tail;
		seen_a2r_head = cap->a2r_head;
		seen_r2a_tail = cap->r2a_tail;

		UBYTE prev_a2r_tail = cap->a2r_tail;
		UBYTE prev_r2a_head = cap->r2a_head;

		if (signal & SIGF_MSGPORT)
		{
			struct Message *msg;
			while (msg = GetMsg(&dev->task_mp))
				handle_app_request(dev, (struct A314_IORequest *)msg);
		}

		handle_packets_received_r2a(dev);
		handle_room_in_a2r(dev);

		// After the packets, so a request that completed in the meantime is
		// replied normally instead of aborted.
		handle_aborts(dev);
		handle_connect_timeouts(dev);

		// Also while a2r still holds data for the Pi: a314d only looks at the
		// com area after an interrupt, so a lost one (or an a314d that started
		// while a2r was full) would otherwise leave both sides waiting.
		if (prev_a2r_tail != cap->a2r_tail || prev_r2a_head != cap->r2a_head ||
		    cap->a2r_head != cap->a2r_tail)
		{
			dbg_trace("Signal PI IRQ");
			set_pi_irq(dev);
		}

		// Dead Pi (see DEAD_PI_FRAMES).
		BOOL no_sockets = !any_open_socket(dev);
		if (no_sockets || pi_moved || !pi_irq_set)
		{
			irq_waiting = FALSE;
			if (!no_sockets && !pi_irq_set && dev->poll.ticks - last_kick >= KICK_PI_FRAMES)
			{
				last_kick = dev->poll.ticks;
				set_pi_irq(dev);
			}
		}
		else if (!irq_waiting)
		{
			irq_waiting = TRUE;
			irq_since = dev->poll.ticks;
		}
		else if (dev->poll.ticks - irq_since > DEAD_PI_FRAMES)
		{
			dbg_warning("a314d has not answered for 5 s: closing all sockets");
			close_all_sockets(dev);
			irq_waiting = FALSE;
		}
	}

	// There is currently no way to unload a314.device.
}

#endif
