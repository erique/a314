#include <exec/types.h>
#include <exec/ports.h>
#include <exec/tasks.h>
#include <exec/nodes.h>
#include <exec/lists.h>
#include <exec/execbase.h>
#include <exec/memory.h>
#include <exec/interrupts.h>

#include <devices/input.h>
#include <devices/inputevent.h>
#include <libraries/dos.h>

#include <proto/alib.h>
#include <proto/exec.h>

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "../a314device/a314.h"

#define SERVICE_NAME "echo"

static struct MsgPort *mp;
static ULONG socket;

static struct A314_IORequest *cmsg;
static struct A314_IORequest *rmsg;
static struct A314_IORequest *wmsg;

static UBYTE arbuf[256];
static UBYTE awbuf[256];

static BOOL pending_a314_read = FALSE;
static BOOL pending_a314_write = FALSE;
static BOOL pending_a314_reset = FALSE;

static BOOL stream_closed = FALSE;

static ULONG ping_count = 0xbadc0ffe;

#pragma pack(push, 1)
struct EchoMessage
{
	ULONG code;
};
#pragma pack(pop)

static void start_a314_cmd(struct A314_IORequest *msg, UWORD command, char *buffer, int length)
{
	msg->a314_Request.io_Command = command;
	msg->a314_Request.io_Error = 0;

	msg->a314_Socket = socket;
	msg->a314_Buffer = buffer;
	msg->a314_Length = length;

	SendIO((struct IORequest *)msg);
}

static LONG a314_connect(char *name)
{
	socket = time(NULL);
	start_a314_cmd(cmsg, A314_CONNECT, name, strlen(name));
	return WaitIO((struct IORequest *)cmsg);
}

static void start_a314_read()
{
	start_a314_cmd(rmsg, A314_READ, arbuf, 255);
	pending_a314_read = TRUE;
}

static void start_a314_write(int length)
{
	start_a314_cmd(wmsg, A314_WRITE, awbuf, length);
	pending_a314_write = TRUE;
}

static LONG sync_a314_write(int length)
{
	start_a314_write(length);
	pending_a314_write = FALSE;
	return WaitIO((struct IORequest *)wmsg);
}

static void start_a314_reset()
{
	start_a314_cmd(cmsg, A314_RESET, NULL, 0);
	pending_a314_reset = TRUE;
}

static LONG sync_a314_reset()
{
	start_a314_reset();
	pending_a314_reset = FALSE;
	return WaitIO((struct IORequest *)cmsg);
}

static void send_ping(ULONG ping)
{
//	printf("sending = %08lx\n", ping); fflush(stdout);
	awbuf[0] = (ping >>  0) & 0xff;
	awbuf[1] = (ping >>  8) & 0xff;
	awbuf[2] = (ping >> 16) & 0xff;
	awbuf[3] = (ping >> 24) & 0xff;
	sync_a314_write(4);
}

static ULONG read_pong()
{
	ULONG result =
		(arbuf[0] <<  0) |
		(arbuf[1] <<  8) |
		(arbuf[2] << 16) |
		(arbuf[3] << 24);

//	printf("result = %08lx\n", result); fflush(stdout);
	return result;
}

static void handle_ping_pong()
{
	ULONG pong = read_pong();
	if (pong != ping_count)
	{
		printf("unexpected pong %08lx ; expected %08lx\n", pong, ping_count);
		fflush(stdout);
	}
	ping_count++;
	if ((ping_count & 0xff) == 0x0)
	{
		printf("ping count = %08lx\n", ping_count);
		fflush(stdout);
	}
//	if (ping_count < 10)
	{
		send_ping(ping_count);
	}
}

static void handle_a314_read_completed()
{
	pending_a314_read = FALSE;

	if (stream_closed)
		return;

	int res = rmsg->a314_Request.io_Error;
	if (res == A314_READ_OK)
	{
		handle_ping_pong();
		start_a314_read();
	}
	else if (res == A314_READ_EOS)
	{
		start_a314_reset();
		stream_closed = TRUE;
	}
	else if (res == A314_READ_RESET)
		stream_closed = TRUE;
}



int main()
{
//	LONG old_priority = SetTaskPri(FindTask(NULL), 60);

	mp = CreatePort(NULL, 0);
	cmsg = (struct A314_IORequest *)CreateExtIO(mp, sizeof(struct A314_IORequest));

	if (OpenDevice(A314_NAME, 0, (struct IORequest *)cmsg, 0) != 0)
	{
		printf("Unable to open a314.device\n");
		goto fail_out1;
	}

	wmsg = (struct A314_IORequest *)CreateExtIO(mp, sizeof(struct A314_IORequest));
	rmsg = (struct A314_IORequest *)CreateExtIO(mp, sizeof(struct A314_IORequest));
	memcpy(wmsg, cmsg, sizeof(struct A314_IORequest));
	memcpy(rmsg, cmsg, sizeof(struct A314_IORequest));

	if (a314_connect(SERVICE_NAME) != A314_CONNECT_OK)
	{
		printf("Unable to connect to " SERVICE_NAME " service\n");
		goto fail_out2;
	}

	start_a314_read();

	ping_count = 0;
	send_ping(ping_count);

	ULONG portsig = 1 << mp->mp_SigBit;

	printf("Press ctrl-c to exit...\n");

	while (TRUE)
	{
		ULONG signal = Wait(portsig | SIGBREAKF_CTRL_C);

		if (signal & portsig)
		{
			struct Message *msg;
			while (msg = GetMsg(mp))
			{
				if (msg == (struct Message *)rmsg)
					handle_a314_read_completed();
				else if (msg == (struct Message *)wmsg)
					pending_a314_write = FALSE;
				else if (msg == (struct Message *)cmsg)
					pending_a314_reset = FALSE;
			}
		}

		if (signal & SIGBREAKF_CTRL_C)
		{
			start_a314_reset();
			stream_closed = TRUE;
		}

		if (stream_closed && !pending_a314_read && !pending_a314_write && !pending_a314_reset)
			break;
	}

fail_out2:
	CloseDevice((struct IORequest *)cmsg);
	DeleteExtIO((struct IORequest *)rmsg);
	DeleteExtIO((struct IORequest *)wmsg);
fail_out1:
	DeleteExtIO((struct IORequest *)cmsg);
	DeletePort(mp);
//	SetTaskPri(FindTask(NULL), old_priority);
	return 0;
}
