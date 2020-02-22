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

#include <hardware/intbits.h>

#include <libraries/dos.h>

#include <proto/exec.h>

#include <string.h>

#include "a314.h"
#include "debug.h"
#include "cmem.h"
#include "device.h"
#include "protocol.h"
#include "sockets.h"
#include "fix_mem_region.h"
#include "startup.h"

int used_in_r2a()
{
	return (ca->r2a_tail - ca->r2a_head) & 255;
}

int used_in_a2r()
{
	return (ca->a2r_tail - ca->a2r_head) & 255;
}

BOOL room_in_a2r(int len)
{
	return used_in_a2r() + 3 + len <= 255;
}

void append_a2r_packet(UBYTE type, UBYTE stream_id, UBYTE length, UBYTE *data)
{
	UBYTE index = ca->a2r_tail;
	ca->a2r_buffer[index++] = length;
	ca->a2r_buffer[index++] = type;
	ca->a2r_buffer[index++] = stream_id;
	for (int i = 0; i < (int)length; i++)
		ca->a2r_buffer[index++] = *data++;
	ca->a2r_tail = index;
}

void close_socket(struct Socket *s, BOOL should_send_reset)
{
	debug_printf("Called close socket\n");

	if (s->pending_connect != NULL)
	{
		struct A314_IORequest *ior = s->pending_connect;
		ior->a314_Request.io_Error = A314_CONNECT_RESET;
		ReplyMsg((struct Message *)ior);
		debug_printf("Reply request 1\n");
		s->pending_connect = NULL;
	}

	if (s->pending_read != NULL)
	{
		struct A314_IORequest *ior = s->pending_read;
		ior->a314_Length = 0;
		ior->a314_Request.io_Error = A314_READ_RESET;
		ReplyMsg((struct Message *)ior);
		debug_printf("Reply request 2\n");
		s->pending_read = NULL;
	}

	if (s->pending_write != NULL)
	{
		struct A314_IORequest *ior = s->pending_write;
		ior->a314_Length = 0;
		ior->a314_Request.io_Error = A314_WRITE_RESET; // A314_EOS_RESET == A314_WRITE_RESET
		ReplyMsg((struct Message *)ior);
		debug_printf("Reply request 3\n");
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

	remove_from_send_queue(s);

	// No operations can be pending when SOCKET_CLOSED is set.
	// However, may not be able to delete socket yet, because is waiting to send PKT_RESET.
	s->flags |= SOCKET_CLOSED;

	BOOL should_delete_socket = TRUE;

	if (should_send_reset)
	{
		if (sq_head == NULL && room_in_a2r(0))
		{
			append_a2r_packet(PKT_RESET, s->stream_id, 0, NULL);
		}
		else
		{
			s->flags |= SOCKET_SHOULD_SEND_RESET;
			s->send_queue_required_length = 0;
			add_to_send_queue(s);
			should_delete_socket = FALSE;
		}
	}

	if (should_delete_socket)
		delete_socket(s);
}

// When a message is received on R2A it is written to this buffer,
// to avoid dealing with the issue that R2A is a circular buffer.
// This is somewhat inefficient, so may want to change that to read from R2A directly.
UBYTE received_packet[256];

void handle_received_packet_r2a(UBYTE type, UBYTE stream_id, UBYTE length)
{
	struct Socket *s = find_socket_by_stream_id(stream_id);

	if (s != NULL && type == PKT_RESET)
	{
		debug_printf("Received a RESET packet from rpi\n");
		close_socket(s, FALSE);
		return;
	}

	if (s == NULL || (s->flags & SOCKET_CLOSED))
	{
		// Ignore this packet. The only packet that can do anything useful on a closed
		// channel is CONNECT, which is not handled at this time.
		return;
	}

	if (type == PKT_CONNECT_RESPONSE)
	{
		debug_printf("Received a CONNECT RESPONSE packet from rpi\n");

		if (s->pending_connect == NULL)
			debug_printf("SERIOUS ERROR: received a CONNECT RESPONSE even though no connect was pending\n");
		else if (length != 1)
			debug_printf("SERIOUS ERROR: received a CONNECT RESPONSE whose length was not 1\n");
		else
		{
			UBYTE result = received_packet[0];
			if (result == 0)
			{
				struct A314_IORequest *ior = s->pending_connect;
				ior->a314_Request.io_Error = A314_CONNECT_OK;
				ReplyMsg((struct Message *)ior);
				debug_printf("Reply request 4\n");
				s->pending_connect = NULL;
			}
			else
			{
				struct A314_IORequest *ior = s->pending_connect;
				ior->a314_Request.io_Error = A314_CONNECT_UNKNOWN_SERVICE;
				ReplyMsg((struct Message *)ior);
				debug_printf("Reply request 5\n");
				s->pending_connect = NULL;

				close_socket(s, FALSE);
			}
		}
	}
	else if (type == PKT_DATA)
	{
		debug_printf("Received a DATA packet from rasp\n");

		if (s->pending_read != NULL)
		{
			struct A314_IORequest *ior = s->pending_read;

			if (ior->a314_Length < length)
				close_socket(s, TRUE);
			else
			{
				memcpy(ior->a314_Buffer, received_packet, length);
				ior->a314_Length = length;
				ior->a314_Request.io_Error = A314_READ_OK;
				ReplyMsg((struct Message *)ior);
				debug_printf("Reply request 6\n");
				s->pending_read = NULL;
			}
		}
		else
		{
			struct QueuedData *qd = (struct QueuedData *)AllocMem(sizeof(struct QueuedData) + length, 0);
			qd->next = NULL,
			qd->length = length;
			memcpy(qd->data, received_packet, length);

			if (s->rq_head == NULL)
				s->rq_head = qd;
			else
				s->rq_tail->next = qd;
			s->rq_tail = qd;
		}
	}
	else if (type == PKT_EOS)
	{
		debug_printf("Received a EOS packet from rpi\n");

		s->flags |= SOCKET_RCVD_EOS_FROM_RPI;

		if (s->pending_read != NULL)
		{
			struct A314_IORequest *ior = s->pending_read;
			ior->a314_Length = 0;
			ior->a314_Request.io_Error = A314_READ_EOS;
			ReplyMsg((struct Message *)ior);
			debug_printf("Reply request 7\n");
			s->pending_read = NULL;

			s->flags |= SOCKET_SENT_EOS_TO_APP;

			if (s->flags & SOCKET_SENT_EOS_TO_RPI)
				close_socket(s, FALSE);
		}
	}
}

void handle_packets_received_r2a()
{
	while (used_in_r2a() != 0)
	{
		UBYTE index = ca->r2a_head;

		UBYTE len = ca->r2a_buffer[index++];
		UBYTE type = ca->r2a_buffer[index++];
		UBYTE stream_id = ca->r2a_buffer[index++];

		for (int i = 0; i < len; i++)
			received_packet[i] = ca->r2a_buffer[index++];

		ca->r2a_head = index;

		handle_received_packet_r2a(type, stream_id, len);
	}
}

void handle_room_in_a2r()
{
	while (sq_head != NULL)
	{
		struct Socket *s = sq_head;

		if (!room_in_a2r(s->send_queue_required_length))
			break;

		remove_from_send_queue(s);

		if (s->pending_connect != NULL)
		{
			struct A314_IORequest *ior = s->pending_connect;
			int len = ior->a314_Length;
			append_a2r_packet(PKT_CONNECT, s->stream_id, (UBYTE)len, ior->a314_Buffer);
		}
		else if (s->pending_write != NULL)
		{
			struct A314_IORequest *ior = s->pending_write;
			int len = ior->a314_Length;

			if (ior->a314_Request.io_Command == A314_WRITE)
			{
				append_a2r_packet(PKT_DATA, s->stream_id, (UBYTE)len, ior->a314_Buffer);

				ior->a314_Request.io_Error = A314_WRITE_OK;
				ReplyMsg((struct Message *)ior);
				debug_printf("Reply request 8\n");
				s->pending_write = NULL;
			}
			else // A314_EOS
			{
				append_a2r_packet(PKT_EOS, s->stream_id, 0, NULL);

				ior->a314_Request.io_Error = A314_EOS_OK;
				ReplyMsg((struct Message *)ior);
				debug_printf("Reply request 9\n");
				s->pending_write = NULL;

				s->flags |= SOCKET_SENT_EOS_TO_RPI;

				if (s->flags & SOCKET_SENT_EOS_TO_APP)
					close_socket(s, FALSE);
			}
		}
		else if (s->flags & SOCKET_SHOULD_SEND_RESET)
		{
			append_a2r_packet(PKT_RESET, s->stream_id, 0, NULL);
			delete_socket(s);
		}
		else
		{
			debug_printf("SERIOUS ERROR: Was in send queue but has nothing to send\n");
		}
	}
}

void handle_received_app_request(struct A314_IORequest *ior)
{
	struct Socket *s = find_socket(ior->a314_Request.io_Message.mn_ReplyPort->mp_SigTask, ior->a314_Socket);

	if (ior->a314_Request.io_Command == A314_CONNECT)
	{
		debug_printf("Received a CONNECT request from application\n");
		if (s != NULL)
		{
			ior->a314_Request.io_Error = A314_CONNECT_SOCKET_IN_USE;
			ReplyMsg((struct Message *)ior);
			debug_printf("Reply request 10\n");
		}
		else if (ior->a314_Length + 3 > 255)
		{
			ior->a314_Request.io_Error = A314_CONNECT_RESET;
			ReplyMsg((struct Message *)ior);
			debug_printf("Reply request 11\n");
		}
		else
		{
			s = create_socket(ior->a314_Request.io_Message.mn_ReplyPort->mp_SigTask, ior->a314_Socket);

			s->pending_connect = ior;
			s->flags = 0;

			int len = ior->a314_Length;
			if (sq_head == NULL && room_in_a2r(len))
			{
				append_a2r_packet(PKT_CONNECT, s->stream_id, (UBYTE)len, ior->a314_Buffer);
			}
			else
			{
				s->send_queue_required_length = len;
				add_to_send_queue(s);
			}
		}
	}
	else if (ior->a314_Request.io_Command == A314_READ)
	{
		debug_printf("Received a READ request from application\n");
		if (s == NULL || (s->flags & SOCKET_CLOSED))
		{
			ior->a314_Length = 0;
			ior->a314_Request.io_Error = A314_READ_RESET;
			ReplyMsg((struct Message *)ior);
			debug_printf("Reply request 12\n");
		}
		else
		{
			if (s->pending_connect != NULL || s->pending_read != NULL)
			{
				ior->a314_Length = 0;
				ior->a314_Request.io_Error = A314_READ_RESET;
				ReplyMsg((struct Message *)ior);
				debug_printf("Reply request 13\n");

				close_socket(s, TRUE);
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
					debug_printf("Reply request 14\n");

					close_socket(s, TRUE);
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
					debug_printf("Reply request 15\n");
				}
			}
			else if (s->flags & SOCKET_RCVD_EOS_FROM_RPI)
			{
				ior->a314_Length = 0;
				ior->a314_Request.io_Error = A314_READ_EOS;
				ReplyMsg((struct Message *)ior);
				debug_printf("Reply request 16\n");

				s->flags |= SOCKET_SENT_EOS_TO_APP;

				if (s->flags & SOCKET_SENT_EOS_TO_RPI)
					close_socket(s, FALSE);
			}
			else
				s->pending_read = ior;
		}
	}
	else if (ior->a314_Request.io_Command == A314_WRITE)
	{
		debug_printf("Received a WRITE request from application\n");
		if (s == NULL || (s->flags & SOCKET_CLOSED))
		{
			ior->a314_Length = 0;
			ior->a314_Request.io_Error = A314_WRITE_RESET;
			ReplyMsg((struct Message *)ior);
			debug_printf("Reply request 17\n");
		}
		else
		{
			int len = ior->a314_Length;
			if (s->pending_connect != NULL || s->pending_write != NULL || (s->flags & SOCKET_RCVD_EOS_FROM_APP) || len + 3 > 255)
			{
				ior->a314_Length = 0;
				ior->a314_Request.io_Error = A314_WRITE_RESET;
				ReplyMsg((struct Message *)ior);
				debug_printf("Reply request 18\n");

				close_socket(s, TRUE);
			}
			else
			{
				if (sq_head == NULL && room_in_a2r(len))
				{
					append_a2r_packet(PKT_DATA, s->stream_id, (UBYTE)len, ior->a314_Buffer);

					ior->a314_Request.io_Error = A314_WRITE_OK;
					ReplyMsg((struct Message *)ior);
					debug_printf("Reply request 19\n");
				}
				else
				{
					s->pending_write = ior;
					s->send_queue_required_length = len;
					add_to_send_queue(s);
				}
			}
		}
	}
	else if (ior->a314_Request.io_Command == A314_EOS)
	{
		debug_printf("Received an EOS request from application\n");
		if (s == NULL || (s->flags & SOCKET_CLOSED))
		{
			ior->a314_Request.io_Error = A314_EOS_RESET;
			ReplyMsg((struct Message *)ior);
			debug_printf("Reply request 20\n");
		}
		else
		{
			if (s->pending_connect != NULL || s->pending_write != NULL || (s->flags & SOCKET_RCVD_EOS_FROM_APP))
			{
				ior->a314_Length = 0;
				ior->a314_Request.io_Error = A314_EOS_RESET;
				ReplyMsg((struct Message *)ior);
				debug_printf("Reply request 21\n");

				close_socket(s, TRUE);
			}
			else
			{
				s->flags |= SOCKET_RCVD_EOS_FROM_APP;

				if (sq_head == NULL && room_in_a2r(0))
				{
					append_a2r_packet(PKT_EOS, s->stream_id, 0, NULL);

					ior->a314_Request.io_Error = A314_EOS_OK;
					ReplyMsg((struct Message *)ior);
					debug_printf("Reply request 22\n");

					s->flags |= SOCKET_SENT_EOS_TO_RPI;

					if (s->flags & SOCKET_SENT_EOS_TO_APP)
						close_socket(s, FALSE);
				}
				else
				{
					s->pending_write = ior;
					s->send_queue_required_length = 0;
					add_to_send_queue(s);
				}
			}
		}
	}
	else if (ior->a314_Request.io_Command == A314_RESET)
	{
		debug_printf("Received a RESET request from application\n");
		if (s == NULL || (s->flags & SOCKET_CLOSED))
		{
			ior->a314_Request.io_Error = A314_RESET_OK;
			ReplyMsg((struct Message *)ior);
			debug_printf("Reply request 23\n");
		}
		else
		{
			ior->a314_Request.io_Error = A314_RESET_OK;
			ReplyMsg((struct Message *)ior);
			debug_printf("Reply request 24\n");

			close_socket(s, TRUE);
		}
	}
	else
	{
		ior->a314_Request.io_Error = IOERR_NOCMD;
		ReplyMsg((struct Message *)ior);
	}
}

extern void IntServer();
struct Interrupt vertb_interrupt;
struct Interrupt ports_interrupt;

void task_main()
{
	write_cmem_safe(A_ENABLE_ADDRESS, 0);
	read_cmem_safe(A_EVENTS_ADDRESS);

	write_base_address(translate_address_a314(ca));

	write_cmem_safe(R_EVENTS_ADDRESS, R_EVENT_BASE_ADDRESS);

	vertb_interrupt.is_Node.ln_Type = NT_INTERRUPT;
	vertb_interrupt.is_Node.ln_Pri = -60;
	vertb_interrupt.is_Node.ln_Name = device_name;
	vertb_interrupt.is_Data = (APTR)task;
	vertb_interrupt.is_Code = IntServer;

	AddIntServer(INTB_VERTB, &vertb_interrupt);


	ports_interrupt.is_Node.ln_Type = NT_INTERRUPT;
	ports_interrupt.is_Node.ln_Pri = 0;
	ports_interrupt.is_Node.ln_Name = device_name;
	ports_interrupt.is_Data = (APTR)task;
	ports_interrupt.is_Code = IntServer;

	AddIntServer(INTB_PORTS, &ports_interrupt);

	write_cmem_safe(A_ENABLE_ADDRESS, A_EVENT_R2A_TAIL);

	while (1)
	{
		debug_printf("Waiting for signal\n");

		ULONG signal = Wait(SIGF_MSGPORT | SIGF_INT);

		UBYTE prev_a2r_tail = ca->a2r_tail;
		UBYTE prev_r2a_head = ca->r2a_head;

		if (signal & SIGF_MSGPORT)
		{
			write_cmem_safe(A_ENABLE_ADDRESS, 0);

			struct Message *msg;
			while (msg = GetMsg(&task_mp))
				handle_received_app_request((struct A314_IORequest *)msg);
		}

		UBYTE a_enable = 0;
		while (a_enable == 0)
		{
			handle_packets_received_r2a();
			handle_room_in_a2r();

			UBYTE r_events = 0;
			if (ca->a2r_tail != prev_a2r_tail)
				r_events |= R_EVENT_A2R_TAIL;
			if (ca->r2a_head != prev_r2a_head)
				r_events |= R_EVENT_R2A_HEAD;

			Disable();
			UBYTE prev_regd = read_cp_nibble(13);
			write_cp_nibble(13, prev_regd | 8);
			read_cp_nibble(A_EVENTS_ADDRESS);

			if (ca->r2a_head == ca->r2a_tail)
			{
				if (sq_head == NULL)
					a_enable = A_EVENT_R2A_TAIL;
				else if (!room_in_a2r(sq_head->send_queue_required_length))
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

	debug_printf("Shutting down\n");

	RemIntServer(INTB_PORTS, &ports_interrupt);
	RemIntServer(INTB_VERTB, &vertb_interrupt);
	FreeMem(ca, sizeof(struct ComArea));

	// Stack and task structure should be reclaimed.
}
