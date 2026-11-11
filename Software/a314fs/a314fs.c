/*
 * Copyright (c) 2018 Niklas Ekström
 */

#include <exec/types.h>
#include <exec/ports.h>
#include <exec/nodes.h>
#include <exec/libraries.h>

#include <devices/timer.h>
#include <devices/input.h>
#include <devices/inputevent.h>

#include <libraries/dos.h>
#include <libraries/dosextens.h>
#include <libraries/filehandler.h>

#include <proto/exec.h>
#include <proto/dos.h>

#include <string.h>
#include <stddef.h>
#include <stdarg.h>

#include "../a314device/a314.h"
#include "../a314device/proto_a314.h"

#include "messages.h"

#define ID_314_DISK (('3' << 24) | ('1' << 16) | ('4' << 8))

#define REQ_RES_BUF_SIZE 256
#define BUFFER_SIZE 4096

// Not defined if using NDK13
#ifndef ACTION_IS_FILESYSTEM
#define ACTION_IS_FILESYSTEM 1027
#endif

#ifndef ACTION_EXAMINE_FH
#define ACTION_EXAMINE_FH 1034
#endif

#ifndef ACTION_SAME_LOCK
#define ACTION_SAME_LOCK 40
#endif

// Grab a reserved action type which seems to be unused
#define ACTION_UNSUPPORTED 65535

struct ExecBase *SysBase;
struct DosLibrary *DOSBase;
struct MsgPort *mp;

struct DeviceList *my_volume;

const char version_string[] = "$VER: a314fs 1.1 (11.11.2026)";

char default_volume_name[] = "\006PiDisk";

char device_name[32]; // "\004PI0:"

struct MsgPort *a314_mp;
struct A314_IORequest *a314_ior;
// The WRITE/READ buffer of the current a314_ior: allocated with it, never on
// the stack, as an abandoned request (see reset_socket()) may still use it.
ULONG *a314_msg;

// Link resilience: PI0: must never hang the Amiga. With no Pi,
// or a Pi that stops answering, PI0: behaves like a drive with no disk, and
// a314fs reconnects in the background.
#define REQ_TIMEOUT_SECS	10	// one request/response round trip
#define CONNECT_TIMEOUT_SECS	3
#define RESET_TIMEOUT_SECS	2
#define RETRY_SECS		2	// between connect attempts
#define RETRY_OPEN_SECS		10	// between OpenDevice attempts (no a314.device yet)
#define LINK_TIMEOUT		(-1)	// a314_cmd_timed(): no answer in time

struct MsgPort *a314_rst_mp;		// a second request on the same socket, to reset it
struct A314_IORequest *a314_rst_ior;
struct MsgPort *tmo_mp;			// request timeouts
struct timerequest *tmo_tr;
struct MsgPort *retry_mp;		// reconnect timer
struct timerequest *retry_tr;
BOOL retry_pending = FALSE;		// retry_tr is running (a retry wait or a connect timeout)
BOOL connecting = FALSE;		// a CONNECT is outstanding on a314_ior
struct MsgPort *input_mp;		// disk inserted/removed events for Workbench
struct IOStdReq *input_ior;
struct InputEvent disk_ie;		// the event input_ior sends (see send_disk_event())
BOOL input_busy = FALSE;		// input_ior has been sent and not yet collected
UBYTE disk_event_pending = 0;		// an event class still to send
BOOL insert_event_wanted = FALSE;	// send DISKINSERTED once the volume is listed

BOOL a314_open = FALSE;			// a314.device is open and our requests are ours
BOOL a314_missing = FALSE;		// gave up on a314.device (see open_a314())
BOOL buffers_ready = FALSE;		// request/data buffers set up in A314 memory
BOOL connected = FALSE;
BOOL volume_in_list = FALSE;
ULONG conn_gen = 0;			// bumped on every connect; older locks/handles are stale

// Locks and file handles remember the connection they came from, so a lock or
// handle from before a drop is never sent to the Pi.
struct MyLock
{
	struct FileLock fl;
	ULONG gen;
};

// fh_Arg1 of our file handles. The open ones are kept in a list, so a handle
// that is not ours any more (a second Close(), a packet reused after
// ACTION_END) is recognised instead of used.
struct MyFile
{
	struct MyFile *next;
	LONG arg1;			// the Pi's handle
	ULONG gen;
};

struct MyFile *my_files = NULL;

static struct MyFile *find_my_file(LONG fh_arg1)
{
	for (struct MyFile *mf = my_files; mf != NULL; mf = mf->next)
		if ((LONG)mf == fh_arg1)
			return mf;
	return NULL;
}

static void free_my_file(struct MyFile *mf)
{
	for (struct MyFile **pp = &my_files; *pp != NULL; pp = &(*pp)->next)
	{
		if (*pp == mf)
		{
			*pp = mf->next;
			FreeMem(mf, sizeof(struct MyFile));
			return;
		}
	}
}

// The Pi's handle for a file packet (dp_Arg1 = fh_Arg1, checked by
// handle_packet()). The packet itself is not changed: a program may send the
// same packet again.
static LONG file_arg1(struct DosPacket *dp)
{
	return ((struct MyFile *)dp->dp_Arg1)->arg1;
}

void create_and_add_volume();
void remove_volume_if_unused();
void sync_volume();
void send_disk_event(UBYTE ie_class);

struct Library *A314Base;

long socket;
ULONG next_socket = 0;		// see start_connect()

// Requests are built and responses read through request_buffer. It points
// at a314_request_buffer (A314 memory, read and written by the Pi) when that
// exists, and otherwise, and for every response made up locally, at
// local_buffer: a late write by the Pi must never land on those.
char local_buffer[REQ_RES_BUF_SIZE];
char *a314_request_buffer = NULL;
char *request_buffer = local_buffer;
BOOL request_buffer_in_a314_memory = FALSE;

// These are allocated in A314 memory.
ULONG request_buffer_address;
ULONG data_buffer_address;

void MyNewList(struct List *l)
{
    l->lh_Head = (struct Node *)&(l->lh_Tail);
    l->lh_Tail = NULL;
    l->lh_TailPred = (struct Node *)&(l->lh_Head);
}

struct MsgPort *MyCreatePort(char *name, long pri)
{
	int sigbit = AllocSignal(-1);
	if (sigbit == -1)
		return NULL;

	struct MsgPort *port = (struct MsgPort *)AllocMem(sizeof(struct MsgPort), MEMF_PUBLIC | MEMF_CLEAR);
	if (!port)
	{
		FreeSignal(sigbit);
		return NULL;
	}

	port->mp_Node.ln_Name = name;
	port->mp_Node.ln_Pri = pri;
	port->mp_Node.ln_Type = NT_MSGPORT;
	port->mp_Flags = PA_SIGNAL;
	port->mp_SigBit = sigbit;
	port->mp_SigTask = FindTask(NULL);

	if (name)
		AddPort(port);
	else
		MyNewList(&(port->mp_MsgList));

	return port;
}

struct IORequest *MyCreateExtIO(struct MsgPort *ioReplyPort, ULONG size)
{
	if (!ioReplyPort)
		return NULL;

	struct IORequest *ioReq = (struct IORequest *)AllocMem(size, MEMF_PUBLIC | MEMF_CLEAR);
	if (!ioReq)
		return NULL;

	ioReq->io_Message.mn_Node.ln_Type = NT_MESSAGE;
	ioReq->io_Message.mn_Length = size;
	ioReq->io_Message.mn_ReplyPort = ioReplyPort;

	return ioReq;
}

#define DEBUG 0

#if !DEBUG
void dbg_init()
{
}

void dbg(const char* fmt, ...)
{
}
#else
struct FileHandle *dbg_log;
struct MsgPort *dbg_replyport;
struct StandardPacket *dbg_sp;
char dbg_buf[256];

void dbg_init()
{
	dbg_log = (struct FileHandle *)BADDR(Open("CON:200/0/440/256/a314fs", MODE_NEWFILE));
	dbg_replyport = MyCreatePort(NULL, 0);
	dbg_sp = (struct StandardPacket *)AllocMem(sizeof(struct StandardPacket), MEMF_PUBLIC | MEMF_CLEAR);
}

void dbg_out(int l)
{
	dbg_sp->sp_Msg.mn_Node.ln_Name = (char *)&(dbg_sp->sp_Pkt);
	dbg_sp->sp_Pkt.dp_Link = &(dbg_sp->sp_Msg);
	dbg_sp->sp_Pkt.dp_Port = dbg_replyport;
	dbg_sp->sp_Pkt.dp_Type = ACTION_WRITE;
	dbg_sp->sp_Pkt.dp_Arg1 = (long)dbg_log->fh_Arg1;
	dbg_sp->sp_Pkt.dp_Arg2 = (long)dbg_buf;
	dbg_sp->sp_Pkt.dp_Arg3 = (long)l - 1;

	PutMsg((struct MsgPort *)dbg_log->fh_Type, (struct Message *)dbg_sp);
	WaitPort(dbg_replyport);
	GetMsg(dbg_replyport);
}

void dbg(const char* fmt, ...)
{
	char numbuf[16];

	const char *p = fmt;
	char *q = dbg_buf;

	va_list args;
	va_start(args, fmt);
	while (*p != 0)
	{
		char c = *p++;
		if (c == '$')
		{
			c = *p++;
			if (c == 'b')
			{
				UBYTE x = va_arg(args, UBYTE);
				*q++ = '$';
				for (int i = 0; i < 2; i++)
				{
					int ni = (x >> ((1 - i) * 4)) & 0xf;
					*q++ = (ni >= 10) ? ('a' + (ni - 10)) : ('0' + ni);
				}
			}
			else if (c == 'w')
			{
				UWORD x = va_arg(args, UWORD);
				*q++ = '$';
				for (int i = 0; i < 4; i++)
				{
					int ni = (x >> ((3 - i) * 4)) & 0xf;
					*q++ = (ni >= 10) ? ('a' + (ni - 10)) : ('0' + ni);
				}
			}
			else if (c == 'l')
			{
				ULONG x = va_arg(args, ULONG);
				*q++ = '$';
				for (int i = 0; i < 8; i++)
				{
					int ni = (x >> ((7 - i) * 4)) & 0xf;
					*q++ = (ni >= 10) ? ('a' + (ni - 10)) : ('0' + ni);
				}
			}
			else if (c == 'S')
			{
				unsigned char *s = (unsigned char *)va_arg(args, ULONG);
				int l = *s++;
				for (int i = 0; i < l; i++)
					*q++ = *s++;
			}
			else if (c == 's')
			{
				unsigned char *s = (unsigned char *)va_arg(args, ULONG);
				while (*s)
					*q++ = *s++;
				*q++ = 0;
			}
		}
		else
		{
			*q++ = c;
		}
	}
	*q++ = 0;

	va_end(args);

	dbg_out(q - dbg_buf);
}
#endif

char *DosAllocMem(int len)
{
	long *p = (long *)AllocMem(len + 4, MEMF_PUBLIC | MEMF_CLEAR);
	*p++ = len;
	return((char *)p);
}

void DosFreeMem(char *p)
{
	long *lp = (long *)p;
	long len = *--lp;
	FreeMem((char *)lp, len + 4);
}

void reply_packet(struct DosPacket *dp)
{
	struct MsgPort *reply_port = dp->dp_Port;
	dp->dp_Port = mp;
	PutMsg(reply_port, dp->dp_Link);
}

// Routines for talking to the A314 driver. Every request has a timeout:
// nothing here may wait for ever on the Pi.

static void start_timer(struct timerequest *t, ULONG secs)
{
	t->tr_node.io_Command = TR_ADDREQUEST;
	t->tr_time.tv_secs = secs;
	t->tr_time.tv_micro = 0;
	SendIO((struct IORequest *)t);
}

static void stop_timer(struct timerequest *t)
{
	if (!CheckIO((struct IORequest *)t))
		AbortIO((struct IORequest *)t);
	WaitIO((struct IORequest *)t);
}

// Wait for a request that has been sent, at most secs seconds.
// TRUE: it completed. FALSE: it is still pending.
static BOOL wait_io_timed(struct IORequest *ior, ULONG secs)
{
	ULONG sigs = (1UL << ior->io_Message.mn_ReplyPort->mp_SigBit) | (1UL << tmo_mp->mp_SigBit);

	start_timer(tmo_tr, secs);
	while (1)
	{
		if (CheckIO(ior))
		{
			WaitIO(ior);
			stop_timer(tmo_tr);
			return TRUE;
		}
		if (CheckIO((struct IORequest *)tmo_tr))
		{
			WaitIO((struct IORequest *)tmo_tr);
			return FALSE;
		}
		Wait(sigs);
	}
}

static void abandon_requests(BOOL ior_out, BOOL rst_out);

// Reset our socket from the second request, so neither side keeps it. Older
// a314.device versions have no AbortIO; a reset makes any version reply a
// request still pending on the socket (close_socket() in driver_task.c), so
// ior_pending says whether a314_ior is to be waited for too.
static void reset_socket(BOOL ior_pending)
{
	a314_rst_ior->a314_Request.io_Command = A314_RESET;
	a314_rst_ior->a314_Request.io_Error = 0;
	a314_rst_ior->a314_Socket = socket;
	a314_rst_ior->a314_Buffer = NULL;
	a314_rst_ior->a314_Length = 0;
	SendIO((struct IORequest *)a314_rst_ior);

	BOOL rst_back = wait_io_timed((struct IORequest *)a314_rst_ior, RESET_TIMEOUT_SECS);
	BOOL ior_back = !ior_pending || wait_io_timed((struct IORequest *)a314_ior, RESET_TIMEOUT_SECS);
	if (!rst_back || !ior_back)
	{
		// a314.device itself is not answering. Its requests may still be
		// replied later, so never reuse them: open_a314() makes new ones.
		dbg("a314.device did not reply; abandoning its requests\n");
		abandon_requests(ior_pending && !ior_back, !rst_back);
	}
}

// Send a command on our socket and wait for it, at most secs seconds.
// Returns the a314 io_Error, or LINK_TIMEOUT (the socket has then been reset).
LONG a314_cmd_timed(UWORD command, char *buffer, int length, ULONG secs)
{
	if (!a314_open)
		return LINK_TIMEOUT;

	a314_ior->a314_Request.io_Command = command;
	a314_ior->a314_Request.io_Error = 0;
	a314_ior->a314_Socket = socket;
	a314_ior->a314_Buffer = buffer;
	a314_ior->a314_Length = length;
	SendIO((struct IORequest *)a314_ior);

	if (wait_io_timed((struct IORequest *)a314_ior, secs))
		return a314_ior->a314_Request.io_Error;

	dbg("a314 command $l timed out; resetting the socket\n", (ULONG)command);
	reset_socket(TRUE);
	return LINK_TIMEOUT;
}

static void MyDeletePort(struct MsgPort *port)
{
	if (port == NULL)
		return;
	FreeSignal(port->mp_SigBit);
	FreeMem(port, sizeof(struct MsgPort));
}

// Free the current request set. Nothing of it may be outstanding.
static void free_requests(BOOL close_device)
{
	if (close_device)
		CloseDevice((struct IORequest *)a314_ior);
	if (a314_ior != NULL)
		FreeMem(a314_ior, sizeof(struct A314_IORequest));
	if (a314_rst_ior != NULL)
		FreeMem(a314_rst_ior, sizeof(struct A314_IORequest));
	if (a314_msg != NULL)
		FreeMem(a314_msg, 2 * sizeof(ULONG));
	MyDeletePort(a314_mp);
	MyDeletePort(a314_rst_mp);
	a314_ior = a314_rst_ior = NULL;
	a314_msg = NULL;
	a314_mp = a314_rst_mp = NULL;
}

// Request sets given up after a314.device did not answer a RESET in time. They
// may still be replied, so they are kept (with their buffer and device open)
// and freed by reap_abandoned() once every outstanding request is back.
#define MAX_ABANDONED 4
struct AbandonedSet
{
	struct MsgPort *mp, *rst_mp;
	struct A314_IORequest *ior, *rst_ior;
	ULONG *msg;
	BOOL ior_out, rst_out;		// still outstanding (not yet WaitIO()ed)
};
struct AbandonedSet abandoned[MAX_ABANDONED];
int n_abandoned = 0;

static void abandon_requests(BOOL ior_out, BOOL rst_out)
{
	struct AbandonedSet *a = &abandoned[n_abandoned++];	// open_a314() keeps room
	a->mp = a314_mp;
	a->rst_mp = a314_rst_mp;
	a->ior = a314_ior;
	a->rst_ior = a314_rst_ior;
	a->msg = a314_msg;
	a->ior_out = ior_out;
	a->rst_out = rst_out;

	a314_ior = a314_rst_ior = NULL;
	a314_msg = NULL;
	a314_mp = a314_rst_mp = NULL;
	a314_open = FALSE;
}

// The reply ports of abandoned requests still outstanding, for the main loop's
// Wait(): their replies must wake it, so they are freed even when PI0: is idle.
static ULONG abandoned_sigs()
{
	ULONG sigs = 0;
	for (int i = 0; i < n_abandoned; i++)
	{
		if (abandoned[i].ior_out)
			sigs |= 1UL << abandoned[i].mp->mp_SigBit;
		if (abandoned[i].rst_out)
			sigs |= 1UL << abandoned[i].rst_mp->mp_SigBit;
	}
	return sigs;
}

// Called on every wake-up of the main loop.
static void reap_abandoned()
{
	for (int i = 0; i < n_abandoned; )
	{
		struct AbandonedSet *a = &abandoned[i];
		if (a->ior_out && CheckIO((struct IORequest *)a->ior))
		{
			WaitIO((struct IORequest *)a->ior);
			a->ior_out = FALSE;
		}
		if (a->rst_out && CheckIO((struct IORequest *)a->rst_ior))
		{
			WaitIO((struct IORequest *)a->rst_ior);
			a->rst_out = FALSE;
		}
		if (a->ior_out || a->rst_out)
		{
			i++;
			continue;
		}

		dbg("Abandoned a314 requests are back; freeing them\n");
		CloseDevice((struct IORequest *)a->ior);
		FreeMem(a->ior, sizeof(struct A314_IORequest));
		FreeMem(a->rst_ior, sizeof(struct A314_IORequest));
		FreeMem(a->msg, 2 * sizeof(ULONG));
		MyDeletePort(a->mp);
		MyDeletePort(a->rst_mp);
		*a = abandoned[--n_abandoned];
	}
}

// Request and data buffers in A314 memory, on the first successful open.
static BOOL setup_buffers()
{
	if (buffers_ready)
		return TRUE;

	ULONG data = AllocMemA314(BUFFER_SIZE);
	if (data == INVALID_A314_ADDRESS)
		return FALSE;

	char *b = AllocMem(REQ_RES_BUF_SIZE, MEMF_A314);
	ULONG b_address = b != NULL ? TranslateAddressA314(b) : INVALID_A314_ADDRESS;
	if (b != NULL && b_address == INVALID_A314_ADDRESS)
	{
		FreeMem(b, REQ_RES_BUF_SIZE);
		b = NULL;
	}

	if (b != NULL)
	{
		a314_request_buffer = b;
		request_buffer = b;
		request_buffer_in_a314_memory = TRUE;
		request_buffer_address = b_address;
	}
	else
	{
		ULONG req = AllocMemA314(REQ_RES_BUF_SIZE);
		if (req == INVALID_A314_ADDRESS)
		{
			FreeMemA314(data, BUFFER_SIZE);
			return FALSE;
		}
		request_buffer_in_a314_memory = FALSE;
		request_buffer_address = req;
	}

	data_buffer_address = data;
	buffers_ready = TRUE;
	return TRUE;
}

BOOL open_a314()
{
	reap_abandoned();
	if (n_abandoned >= MAX_ABANDONED)
	{
		dbg("open_a314: earlier requests are not back yet, will retry\n");
		return FALSE;
	}

	a314_mp = MyCreatePort(NULL, 0);
	a314_rst_mp = MyCreatePort(NULL, 0);
	a314_ior = (struct A314_IORequest *)MyCreateExtIO(a314_mp, sizeof(struct A314_IORequest));
	a314_rst_ior = (struct A314_IORequest *)MyCreateExtIO(a314_rst_mp, sizeof(struct A314_IORequest));
	a314_msg = (ULONG *)AllocMem(2 * sizeof(ULONG), MEMF_PUBLIC);

	if (a314_ior == NULL || a314_rst_ior == NULL || a314_msg == NULL ||
	    OpenDevice(A314_NAME, 0, (struct IORequest *)a314_ior, 0) != 0)
	{
		// Not there yet (no TF board found, no a314.device file, or an
		// older a314.device that refuses to open without a314d): try again
		// later. Nothing was handed to the device, so free it all.
		dbg("open_a314: OpenDevice failed, will retry\n");
		free_requests(FALSE);
		return FALSE;
	}

	A314Base = &(a314_ior->a314_Request.io_Device->dd_Library);
	if (!setup_buffers())
	{
		// No A314 memory for the buffers: try again later. Nothing has been
		// sent yet, so the requests can be freed.
		dbg("open_a314: no A314 memory for the buffers, will retry\n");
		free_requests(TRUE);
		return FALSE;
	}

	// The reset request uses the same unit.
	a314_rst_ior->a314_Request.io_Device = a314_ior->a314_Request.io_Device;
	a314_rst_ior->a314_Request.io_Unit = a314_ior->a314_Request.io_Unit;

	a314_open = TRUE;
	return TRUE;
}

void arm_retry()
{
	if (retry_pending || a314_missing)
		return;
	start_timer(retry_tr, a314_open ? RETRY_SECS : RETRY_OPEN_SECS);
	retry_pending = TRUE;
}

void go_online()
{
	conn_gen++;
	connected = TRUE;
	insert_event_wanted = TRUE;	// sent by sync_volume() once PiDisk is listed
	create_and_add_volume();
	dbg("Connected (generation $l)\n", conn_gen);
}

void go_offline()
{
	if (!connected)
		return;

	connected = FALSE;
	insert_event_wanted = FALSE;
	if (my_volume != NULL)
		my_volume->dl_Task = NULL;
	remove_volume_if_unused();
	send_disk_event(IECLASS_DISKREMOVED);
	arm_retry();
	dbg("Link to the Pi lost\n");
}

// Start a CONNECT without waiting for it: the main loop sees it complete
// (a314_mp) or time out (retry_tr). Meanwhile every packet fails at once
// with ERROR_NO_DISK, so nothing waits on the Pi.
void start_connect()
{
	if (a314_missing || connecting || connected)
		return;
	if (!a314_open && !open_a314())
	{
		arm_retry();
		return;
	}

	// A new socket id for every attempt: a closed socket can stay in
	// a314.device until its PKT_RESET goes out, and would still match.
	if (next_socket == 0)
	{
		struct DateStamp ds;
		DateStamp(&ds);
		next_socket = ((ULONG)ds.ds_Minute << 12) + ds.ds_Tick + 1;
	}
	socket = next_socket++;

	a314_ior->a314_Request.io_Command = A314_CONNECT;
	a314_ior->a314_Request.io_Error = 0;
	a314_ior->a314_Socket = socket;
	a314_ior->a314_Buffer = "a314fs";
	a314_ior->a314_Length = 6;
	SendIO((struct IORequest *)a314_ior);
	connecting = TRUE;
	dbg("connect: start, socket $l\n", (ULONG)socket);

	if (retry_pending)
		stop_timer(retry_tr);
	start_timer(retry_tr, CONNECT_TIMEOUT_SECS);
	retry_pending = TRUE;
}

// Called from the main loop on every wake-up.
void poll_connection()
{
	if (connecting)
	{
		if (CheckIO((struct IORequest *)a314_ior))
		{
			WaitIO((struct IORequest *)a314_ior);
			connecting = FALSE;
			stop_timer(retry_tr);
			retry_pending = FALSE;

			dbg("connect: done, error $l\n", (ULONG)a314_ior->a314_Request.io_Error);
			if (a314_ior->a314_Request.io_Error == A314_CONNECT_OK)
				go_online();
			else
				arm_retry();
		}
		else if (CheckIO((struct IORequest *)retry_tr))
		{
			// No answer: reset the socket so a314.device replies the CONNECT.
			WaitIO((struct IORequest *)retry_tr);
			retry_pending = FALSE;

			dbg("connect: timeout, resetting the socket\n");
			reset_socket(TRUE);
			connecting = FALSE;
			arm_retry();
		}
		return;
	}

	if (!connected && retry_pending && CheckIO((struct IORequest *)retry_tr))
	{
		WaitIO((struct IORequest *)retry_tr);
		retry_pending = FALSE;
		start_connect();
	}

	// While connected retry_tr only waits for a volume list change (sync_volume()).
	if (connected && retry_pending && CheckIO((struct IORequest *)retry_tr))
	{
		WaitIO((struct IORequest *)retry_tr);
		retry_pending = FALSE;
		sync_volume();
	}
}

// The volume is listed while connected, and while locks on it are still held
// (with dl_Task = NULL then, so DOS asks for the volume when they are used).
//
// The handler must never wait for a DOS lock: a task that holds one while it
// waits on PI0: would deadlock it. So there is no AddDosEntry()/RemDosEntry(),
// which can wait on the entry and delete locks. On V36+ the node is linked in
// or out by hand under the volume list lock, taken with AttemptLockDosList().
// If the lock is busy the change waits (volume_sync_pending) and is retried on
// the next wake-up, which is within a second while connected (retry_tr). On
// 1.3, which has no lock, it is done under Forbid(). The node is never freed,
// so a task walking the list while it is unlinked still finds a valid dl_Next.
BOOL volume_sync_pending = FALSE;

// FALSE if the DOS list lock is busy.
static BOOL link_volume(BOOL in)
{
	BOOL v36 = DOSBase->dl_lib.lib_Version >= 36;
	if (v36)
	{
		struct DosList *dl = AttemptLockDosList(LDF_VOLUMES | LDF_WRITE);
		if ((ULONG)dl <= 1)	// busy; some versions return 1, not NULL
			return FALSE;
	}
	else
		Forbid();

	struct RootNode *root = (struct RootNode *)DOSBase->dl_Root;
	struct DosInfo *info = (struct DosInfo *)BADDR(root->rn_Info);
	BPTR me = MKBADDR(my_volume);

	if (in)
	{
		my_volume->dl_Next = info->di_DevInfo;
		info->di_DevInfo = me;
	}
	else if (info->di_DevInfo == me)
		info->di_DevInfo = my_volume->dl_Next;
	else
	{
		struct DeviceList *prev = (struct DeviceList *)BADDR(info->di_DevInfo);
		while (prev != NULL && prev->dl_Next != me)
			prev = (struct DeviceList *)BADDR(prev->dl_Next);
		if (prev != NULL)
			prev->dl_Next = my_volume->dl_Next;
	}

	if (v36)
		UnLockDosList(LDF_VOLUMES | LDF_WRITE);
	else
		Permit();

	volume_in_list = in;
	return TRUE;
}

void sync_volume()
{
	if (my_volume == NULL)
		return;

	BOOL want = connected || my_volume->dl_LockList != 0;
	volume_sync_pending = want != volume_in_list && !link_volume(want);

	// Workbench looks for the volume when it gets the event, so not before.
	if (connected && volume_in_list && insert_event_wanted)
	{
		insert_event_wanted = FALSE;
		send_disk_event(IECLASS_DISKINSERTED);
	}

	if (volume_sync_pending && connected && !retry_pending)
	{
		start_timer(retry_tr, 1);	// see poll_connection()
		retry_pending = TRUE;
	}
}

void create_and_add_volume()
{
	if (my_volume == NULL)
	{
		my_volume = (struct DeviceList *)DosAllocMem(sizeof(struct DeviceList));
		my_volume->dl_Name = MKBADDR(default_volume_name);
		my_volume->dl_Type = DLT_VOLUME;
		my_volume->dl_DiskType = ID_314_DISK;
		DateStamp(&my_volume->dl_VolumeDate);
	}

	my_volume->dl_Task = mp;
	sync_volume();
}

// Take the volume out of the device list once nothing holds a lock on it.
void remove_volume_if_unused()
{
	sync_volume();
}

// Disk events are sent without waiting for them, so an input handler that
// waits on PI0: can't deadlock the handler. One event is kept until it can be
// sent (the previous one is still in progress); a newer one replaces it, as
// only the current state matters. The main loop calls flush_disk_event().
void flush_disk_event()
{
	if (disk_event_pending == 0 || input_ior == NULL)
		return;

	if (input_busy)
	{
		if (!CheckIO((struct IORequest *)input_ior))
			return;
		WaitIO((struct IORequest *)input_ior);
		input_busy = FALSE;
	}

	memset(&disk_ie, 0, sizeof(disk_ie));
	disk_ie.ie_Class = disk_event_pending;
	disk_event_pending = 0;

	input_ior->io_Command = IND_WRITEEVENT;
	input_ior->io_Data = &disk_ie;
	input_ior->io_Length = sizeof(struct InputEvent);
	input_ior->io_Flags = 0;
	SendIO((struct IORequest *)input_ior);
	input_busy = TRUE;
}

void send_disk_event(UBYTE ie_class)
{
	disk_event_pending = ie_class;
	flush_disk_event();
}

void startup_fs_handler(struct DosPacket *dp)
{
	unsigned char *name = (unsigned char *)BADDR(dp->dp_Arg1);
	struct FileSysStartupMsg *fssm = (struct FileSysStartupMsg *)BADDR(dp->dp_Arg2);
	struct DeviceNode *node = (struct DeviceNode *)BADDR(dp->dp_Arg3);

	memcpy(device_name, name, *name + 1);
	device_name[*name + 1] = 0;

	node->dn_Task = mp;

	dp->dp_Res1 = DOSTRUE;
	dp->dp_Res2 = 0;
	reply_packet(dp);

	dbg_init();

	dbg("ACTION_STARTUP\n");
	dbg("  device_name = $S\n", (ULONG)device_name);
	dbg("  fssm = $l\n", (ULONG)fssm);
	dbg("  node = $l\n", (ULONG)node);

	tmo_mp = MyCreatePort(NULL, 0);
	tmo_tr = (struct timerequest *)MyCreateExtIO(tmo_mp, sizeof(struct timerequest));
	retry_mp = MyCreatePort(NULL, 0);
	retry_tr = (struct timerequest *)MyCreateExtIO(retry_mp, sizeof(struct timerequest));
	BOOL timers = FALSE;
	if (tmo_tr != NULL && retry_tr != NULL &&
	    OpenDevice(TIMERNAME, UNIT_VBLANK, (struct IORequest *)tmo_tr, 0) == 0)
	{
		if (OpenDevice(TIMERNAME, UNIT_VBLANK, (struct IORequest *)retry_tr, 0) == 0)
			timers = TRUE;
		else
			CloseDevice((struct IORequest *)tmo_tr);
	}
	if (!timers)
	{
		// Without the timeouts a request could wait on the Pi for ever, so
		// never connect: PI0: stays an empty drive.
		dbg("Unable to open timer.device for the timeouts; PI0: stays empty\n");
		a314_missing = TRUE;
		if (tmo_tr != NULL)
			FreeMem(tmo_tr, sizeof(struct timerequest));
		if (retry_tr != NULL)
			FreeMem(retry_tr, sizeof(struct timerequest));
		MyDeletePort(tmo_mp);
		MyDeletePort(retry_mp);
		tmo_tr = retry_tr = NULL;
		tmo_mp = retry_mp = NULL;
	}

	input_mp = MyCreatePort(NULL, 0);
	input_ior = (struct IOStdReq *)MyCreateExtIO(input_mp, sizeof(struct IOStdReq));
	if (input_ior != NULL && OpenDevice("input.device", 0, (struct IORequest *)input_ior, 0) != 0)
		input_ior = NULL;

	// Connect now, waiting up to a second so a Mount followed straight away
	// by a command sees the volume. If the Pi is not there yet, PI0: is an
	// empty drive and the main loop keeps trying in the background.
	// Only the CONNECT and tmo_tr are waited for here: poll_connection() may
	// reset the socket, which uses tmo_tr itself, so it runs after stop_timer().
	start_connect();
	if (connecting)
	{
		ULONG sigs = (1UL << a314_mp->mp_SigBit) | (1UL << tmo_mp->mp_SigBit);
		start_timer(tmo_tr, 1);
		while (!CheckIO((struct IORequest *)a314_ior) && !CheckIO((struct IORequest *)tmo_tr))
			Wait(sigs);
		stop_timer(tmo_tr);
		poll_connection();
	}

	dbg("Startup done, connected = $l\n", (ULONG)connected);
}


// A BCPL string from a response (which ends at end) into a FileInfoBlock field
// of size bytes: the length byte and at most size - 1 characters (on V36+ DOS
// makes it a C string in place), and a NUL after them if there is room.
// Returns the length taken from the response, to find the next string in it.
static int copy_bstr(char *dst, const char *src, int size, const char *end)
{
	int avail = end - src - 1;			// characters the response holds
	int len = avail < 0 ? 0 : (unsigned char)src[0];
	if (len > avail)
		len = avail < 0 ? 0 : avail;
	int n = len < size - 1 ? len : size - 1;
	dst[0] = n;
	memcpy(dst + 1, src + 1, n);
	if (n + 1 < size)
		dst[n + 1] = 0;
	return len;
}

// The length of the last response (write_req_and_wait_for_res()).
ULONG res_len = 0;

// The smallest success response for each request type: a success with less is
// not a real response.
static ULONG min_success_len(short type)
{
	switch (type)
	{
	case ACTION_LOCATE_OBJECT:	return sizeof(struct LocateObjectResponse);
	case ACTION_COPY_DIR:		return sizeof(struct CopyDirResponse);
	case ACTION_PARENT:		return sizeof(struct ParentResponse);
	case ACTION_CREATE_DIR:		return sizeof(struct CreateDirResponse);
	case ACTION_FINDINPUT:
	case ACTION_FINDOUTPUT:
	case ACTION_FINDUPDATE:		return sizeof(struct FindXxxResponse);
	case ACTION_READ:		return sizeof(struct ReadResponse);
	case ACTION_WRITE:		return sizeof(struct WriteResponse);
	case ACTION_SEEK:		return sizeof(struct SeekResponse);
	case ACTION_INFO:		return sizeof(struct InfoResponse);
	case ACTION_DISK_INFO:		return sizeof(struct DiskInfoResponse);
	case ACTION_EXAMINE_OBJECT:
	case ACTION_EXAMINE_NEXT:
	case ACTION_EXAMINE_FH:		return offsetof(struct ExamineObjectResponse, data) + 2;	// two lengths
	default:			return sizeof(struct FreeLockResponse);
	}
}

// The name and the comment of an EXAMINE response must both lie within the
// bytes received (min_success_len() guarantees the name's length byte).
static BOOL examine_strings_ok(const char *data)
{
	int name = data - request_buffer;
	int comment = name + 1 + (unsigned char)data[0];
	return comment < (int)res_len &&
	       comment + 1 + (unsigned char)request_buffer[comment] <= (int)res_len;
}

// A request with names must fit the request buffer; DOS names can be up to 255
// bytes each. TRUE (and the packet replied) if it does not.
static BOOL request_too_long(struct DosPacket *dp, int len)
{
	if (len <= REQ_RES_BUF_SIZE)
		return FALSE;
	dbg("Request of $l bytes does not fit the buffer; refused\n", (ULONG)len);
	dp->dp_Res1 = DOSFALSE;
	dp->dp_Res2 = ERROR_INVALID_COMPONENT_NAME;
	reply_packet(dp);
	return TRUE;
}

// Every response starts with has_response, success, error_code.
static void fail_response(short error_code)
{
	request_buffer = local_buffer;	// never in A314 memory (see local_buffer)
	memset(request_buffer, 0, REQ_RES_BUF_SIZE);
	struct FreeLockResponse *res = (struct FreeLockResponse *)request_buffer;
	res->has_response = 1;
	res->success = 0;
	res->error_code = error_code;
}

// A response that can't be right is a link failure: the old stream is reset,
// the volume dropped, and the caller gets ERROR_NO_DISK.
static void bad_response()
{
	if (a314_open)
		reset_socket(FALSE);
	go_offline();
	fail_response(ERROR_NO_DISK);
}

// Send the request in request_buffer and wait for the response. If the Pi is
// not connected or does not answer in time, the response is a failure with
// ERROR_NO_DISK (DOS then offers Retry/Cancel), and a314fs reconnects in the
// background.
BOOL write_req_and_wait_for_res(int len)
{
	if (!connected)
	{
		fail_response(ERROR_NO_DISK);
		return FALSE;
	}

	// After a local failure response the request was built in local_buffer.
	if (request_buffer_in_a314_memory && request_buffer != a314_request_buffer)
	{
		memcpy(a314_request_buffer, request_buffer, len);
		request_buffer = a314_request_buffer;
	}

	if (!request_buffer_in_a314_memory)
		WriteMemA314(request_buffer_address, request_buffer, len);
	a314_msg[0] = request_buffer_address;
	a314_msg[1] = len;

	short type = ((struct FreeLockRequest *)request_buffer)->type;	// all requests start so

	LONG err = a314_cmd_timed(A314_WRITE, (char *)&a314_msg[0], 8, REQ_TIMEOUT_SECS);
	if (err == A314_WRITE_OK)
		err = a314_cmd_timed(A314_READ, (char *)&a314_msg[1], 4, REQ_TIMEOUT_SECS);

	// The request set is only looked at after a READ that completed: after a
	// timeout it may have been abandoned (reset_socket()).
	// The response length: at least has_response, success and error_code, and
	// no more than the buffer (unsigned, so a bad value can't pass as negative).
	res_len = 0;
	if (err == A314_READ_OK)
	{
		res_len = a314_msg[1];
		if (a314_ior->a314_Length != 4 ||
		    res_len < sizeof(struct FreeLockResponse) || res_len > REQ_RES_BUF_SIZE)
		{
			dbg("Bad response ($l bytes, length $l)\n", (ULONG)a314_ior->a314_Length, res_len);
			err = A314_READ_RESET;
		}
	}

	if (err != A314_READ_OK)
	{
		// A timeout has reset the socket already. Anything else (a reset or
		// EOS from the Pi, a short read) may leave it open on one side or the
		// other: reset it, or the old stream would stay behind.
		if (err == LINK_TIMEOUT)
		{
			go_offline();
			fail_response(ERROR_NO_DISK);
		}
		else
			bad_response();
		return FALSE;
	}

	if (!request_buffer_in_a314_memory)
		ReadMemA314(request_buffer, request_buffer_address, res_len);

	if (((struct FreeLockResponse *)request_buffer)->success && res_len < min_success_len(type))
	{
		dbg("Short success response ($l bytes) for type $l\n", res_len, (ULONG)type);
		bad_response();
		return FALSE;
	}
	return TRUE;
}

struct FileLock *create_and_add_file_lock(long key, long mode)
{
	struct MyLock *ml = (struct MyLock *)DosAllocMem(sizeof(struct MyLock));
	struct FileLock *lock = &ml->fl;
	ml->gen = conn_gen;

	lock->fl_Key = key;
	lock->fl_Access = mode;
	lock->fl_Task = mp;
	lock->fl_Volume = MKBADDR(my_volume);

	Forbid();
	lock->fl_Link = my_volume->dl_LockList;
	my_volume->dl_LockList = MKBADDR(lock);
	Permit();

	return lock;
}

void action_locate_object(struct DosPacket *dp)
{
	struct FileLock *parent = (struct FileLock *)BADDR(dp->dp_Arg1);
	unsigned char *name = (unsigned char *)BADDR(dp->dp_Arg2);
	long mode = dp->dp_Arg3;

	dbg("ACTION_LOCATE_OBJECT\n");
	dbg("  parent lock = $l\n", parent);
	dbg("  name = $S\n", name);
	dbg("  mode = $s\n", mode == SHARED_LOCK ? "SHARED_LOCK" : "EXCLUSIVE_LOCK");

	struct LocateObjectRequest *req = (struct LocateObjectRequest *)request_buffer;
	req->has_response = 0;
	req->type = dp->dp_Type;
	req->key = parent == NULL ? 0 : parent->fl_Key;
	req->mode = mode;

	int nlen = *name;
	if (request_too_long(dp, sizeof(struct LocateObjectRequest) + nlen))
		return;
	memcpy(req->name, name, nlen + 1);

	write_req_and_wait_for_res(sizeof(struct LocateObjectRequest) + nlen);

	struct LocateObjectResponse *res = (struct LocateObjectResponse *)request_buffer;
	if (!res->success)
	{
		dbg("  Failed, error code $l\n", (LONG)res->error_code);
		dp->dp_Res1 = DOSFALSE;
		dp->dp_Res2 = res->error_code;
	}
	else
	{
		struct FileLock *lock = create_and_add_file_lock(res->key, mode);

		dbg("  Returning lock $l\n", lock);
		dp->dp_Res1 = MKBADDR(lock);
		dp->dp_Res2 = 0;
	}

	reply_packet(dp);
}

void action_free_lock(struct DosPacket *dp)
{
	ULONG arg1 = dp->dp_Arg1;
	struct FileLock *lock = (struct FileLock *)BADDR(dp->dp_Arg1);

	dbg("ACTION_FREE_LOCK\n");
	dbg("  lock = $l\n", lock);

	if (lock != NULL)
	{
		// A lock from before a drop is only freed here: the Pi no longer
		// knows its key.
		if (connected && ((struct MyLock *)lock)->gen == conn_gen)
		{
			struct FreeLockRequest *req = (struct FreeLockRequest *)request_buffer;
			req->has_response = 0;
			req->type = dp->dp_Type;
			req->key = lock->fl_Key;

			write_req_and_wait_for_res(sizeof(struct FreeLockRequest));
		}

		// Ignore the response. Must succeed.
		//struct FreeLockResponse *res = (struct FreeLockResponse *)request_buffer;

		Forbid();
		if (my_volume->dl_LockList == arg1)
			my_volume->dl_LockList = lock->fl_Link;
		else
		{
			struct FileLock *prev = (struct FileLock *)BADDR(my_volume->dl_LockList);
			while (prev->fl_Link != arg1)
				prev = (struct FileLock *)BADDR(prev->fl_Link);
			prev->fl_Link = lock->fl_Link;
		}
		Permit();
		DosFreeMem((char *)lock);
		remove_volume_if_unused();
	}

	dp->dp_Res1 = DOSTRUE;
	dp->dp_Res2 = 0;
	reply_packet(dp);
}

void action_copy_dir(struct DosPacket *dp)
{
	struct FileLock *parent = (struct FileLock *)BADDR(dp->dp_Arg1);

	dbg("ACTION_COPY_DIR\n");
	dbg("  lock to duplicate = $l\n", parent);

	if (parent == NULL)
	{
		dp->dp_Res1 = 0;
		dp->dp_Res2 = 0;
	}
	else
	{
		struct CopyDirRequest *req = (struct CopyDirRequest *)request_buffer;
		req->has_response = 0;
		req->type = dp->dp_Type;
		req->key = parent->fl_Key;

		write_req_and_wait_for_res(sizeof(struct CopyDirRequest));

		struct CopyDirResponse *res = (struct CopyDirResponse *)request_buffer;
		if (!res->success)
		{
			dbg("  Failed, error code $l\n", (LONG)res->error_code);
			dp->dp_Res1 = DOSFALSE;
			dp->dp_Res2 = res->error_code;
		}
		else
		{
			struct FileLock *lock = create_and_add_file_lock(res->key, parent->fl_Access);

			dbg("  Returning lock $l\n", lock);
			dp->dp_Res1 = MKBADDR(lock);
			dp->dp_Res2 = 0;
		}
	}

	reply_packet(dp);
}

void action_parent(struct DosPacket *dp)
{
	struct FileLock *prev_lock = (struct FileLock *)BADDR(dp->dp_Arg1);

	dbg("ACTION_PARENT\n");
	dbg("  lock = $l\n", prev_lock);

	if (prev_lock == NULL)
	{
		dp->dp_Res1 = DOSFALSE;
		dp->dp_Res2 = ERROR_INVALID_LOCK;
	}
	else
	{
		struct ParentRequest *req = (struct ParentRequest *)request_buffer;
		req->has_response = 0;
		req->type = dp->dp_Type;
		req->key = prev_lock->fl_Key;

		write_req_and_wait_for_res(sizeof(struct ParentRequest));

		struct ParentResponse *res = (struct ParentResponse *)request_buffer;
		if (!res->success)
		{
			dbg("  Failed, error code $l\n", (LONG)res->error_code);
			dp->dp_Res1 = DOSFALSE;
			dp->dp_Res2 = res->error_code;
		}
		else if (res->key == 0)
		{
			dp->dp_Res1 = 0;
			dp->dp_Res2 = 0;
		}
		else
		{
			struct FileLock *lock = create_and_add_file_lock(res->key, SHARED_LOCK);

			dbg("  Returning lock $l\n", lock);
			dp->dp_Res1 = MKBADDR(lock);
			dp->dp_Res2 = 0;
		}
	}

	reply_packet(dp);
}

void action_examine_object(struct DosPacket *dp)
{
	struct FileLock *lock = (struct FileLock *)BADDR(dp->dp_Arg1);
	struct FileInfoBlock *fib = (struct FileInfoBlock *)BADDR(dp->dp_Arg2);

	dbg("ACTION_EXAMINE_OBJECT\n");
	dbg("  lock = $l\n", lock);
	dbg("  fib = $l\n", fib);

	struct ExamineObjectRequest *req = (struct ExamineObjectRequest *)request_buffer;
	req->has_response = 0;
	req->type = dp->dp_Type;
	req->key = lock == NULL ? 0 : lock->fl_Key;

	write_req_and_wait_for_res(sizeof(struct ExamineObjectRequest));

	struct ExamineObjectResponse *res = (struct ExamineObjectResponse *)request_buffer;
	if (res->success && !examine_strings_ok(res->data))
	{
		dbg("  Malformed EXAMINE response\n");
		bad_response();		// a link failure: the response is now the local one
		res = (struct ExamineObjectResponse *)request_buffer;
	}
	if (!res->success)
	{
		dbg("  Failed, error code $l\n", (LONG)res->error_code);
		dp->dp_Res1 = DOSFALSE;
		dp->dp_Res2 = res->error_code;
	}
	else
	{
		const char *end = request_buffer + res_len;
		int nlen = copy_bstr(fib->fib_FileName, &res->data[0], sizeof(fib->fib_FileName), end);
		copy_bstr(fib->fib_Comment, &res->data[1 + nlen], sizeof(fib->fib_Comment), end);

		fib->fib_DiskKey = res->disk_key;
		fib->fib_DirEntryType = res->entry_type;
		fib->fib_EntryType = res->entry_type;
		fib->fib_Protection = res->protection;
		fib->fib_Size = res->size;
		fib->fib_NumBlocks = (res->size + 511) >> 9;
		fib->fib_Date.ds_Days = res->date[0];
		fib->fib_Date.ds_Minute = res->date[1];
		fib->fib_Date.ds_Tick = res->date[2];

		dp->dp_Res1 = DOSTRUE;
		dp->dp_Res2 = 0;
	}

	reply_packet(dp);
}

void action_examine_next(struct DosPacket *dp)
{
	struct FileLock *lock = (struct FileLock *)BADDR(dp->dp_Arg1);
	struct FileInfoBlock *fib = (struct FileInfoBlock *)BADDR(dp->dp_Arg2);

	dbg("ACTION_EXAMINE_NEXT\n");
	dbg("  lock = $l\n", lock);
	dbg("  fib = $l\n", fib);

	struct ExamineNextRequest *req = (struct ExamineNextRequest *)request_buffer;
	req->has_response = 0;
	req->type = dp->dp_Type;
	req->key = lock == NULL ? 0 : lock->fl_Key;
	req->disk_key = fib->fib_DiskKey;

	write_req_and_wait_for_res(sizeof(struct ExamineNextRequest));

	struct ExamineNextResponse *res = (struct ExamineNextResponse *)request_buffer;
	if (res->success && !examine_strings_ok(res->data))
	{
		dbg("  Malformed EXAMINE response\n");
		bad_response();		// a link failure: the response is now the local one
		res = (struct ExamineNextResponse *)request_buffer;
	}
	if (!res->success)
	{
		dbg("  Failed, error code $l\n", (LONG)res->error_code);
		dp->dp_Res1 = DOSFALSE;
		dp->dp_Res2 = res->error_code;
	}
	else
	{
		const char *end = request_buffer + res_len;
		int nlen = copy_bstr(fib->fib_FileName, &res->data[0], sizeof(fib->fib_FileName), end);
		copy_bstr(fib->fib_Comment, &res->data[1 + nlen], sizeof(fib->fib_Comment), end);

		fib->fib_DiskKey = res->disk_key;
		fib->fib_DirEntryType = res->entry_type;
		fib->fib_EntryType = res->entry_type;
		fib->fib_Protection = res->protection;
		fib->fib_Size = res->size;
		fib->fib_NumBlocks = (res->size + 511) >> 9;
		fib->fib_Date.ds_Days = res->date[0];
		fib->fib_Date.ds_Minute = res->date[1];
		fib->fib_Date.ds_Tick = res->date[2];

		dp->dp_Res1 = DOSTRUE;
		dp->dp_Res2 = 0;
	}

	reply_packet(dp);
}

void action_examine_fh(struct DosPacket *dp)
{
	ULONG arg1 = file_arg1(dp);
	struct FileInfoBlock *fib = (struct FileInfoBlock *)BADDR(dp->dp_Arg2);

	dbg("ACTION_EXAMINE_FH\n");
	dbg("  arg1 = $l\n", arg1);
	dbg("  fib = $l\n", fib);

	struct ExamineFhRequest *req = (struct ExamineFhRequest *)request_buffer;
	req->has_response = 0;
	req->type = dp->dp_Type;
	req->arg1 = arg1;

	write_req_and_wait_for_res(sizeof(struct ExamineFhRequest));

	struct ExamineFhResponse *res = (struct ExamineFhResponse *)request_buffer;
	if (res->success && !examine_strings_ok(res->data))
	{
		dbg("  Malformed EXAMINE response\n");
		bad_response();		// a link failure: the response is now the local one
		res = (struct ExamineFhResponse *)request_buffer;
	}
	if (!res->success)
	{
		dbg("  Failed, error code $l\n", (LONG)res->error_code);
		dp->dp_Res1 = DOSFALSE;
		dp->dp_Res2 = res->error_code;
	}
	else
	{
		const char *end = request_buffer + res_len;
		int nlen = copy_bstr(fib->fib_FileName, &res->data[0], sizeof(fib->fib_FileName), end);
		copy_bstr(fib->fib_Comment, &res->data[1 + nlen], sizeof(fib->fib_Comment), end);

		fib->fib_DiskKey = res->disk_key;
		fib->fib_DirEntryType = res->entry_type;
		fib->fib_EntryType = res->entry_type;
		fib->fib_Protection = res->protection;
		fib->fib_Size = res->size;
		fib->fib_NumBlocks = (res->size + 511) >> 9;
		fib->fib_Date.ds_Days = res->date[0];
		fib->fib_Date.ds_Minute = res->date[1];
		fib->fib_Date.ds_Tick = res->date[2];

		dp->dp_Res1 = DOSTRUE;
		dp->dp_Res2 = 0;
	}

	reply_packet(dp);
}

void action_findxxx(struct DosPacket *dp)
{
	struct FileHandle *fh = (struct FileHandle *)BADDR(dp->dp_Arg1);
	struct FileLock *lock = (struct FileLock *)BADDR(dp->dp_Arg2);
	unsigned char *name = (unsigned char *)BADDR(dp->dp_Arg3);

	if (dp->dp_Type == ACTION_FINDUPDATE)
		dbg("ACTION_FINDUPDATE\n");
	else if (dp->dp_Type == ACTION_FINDINPUT)
		dbg("ACTION_FINDINPUT\n");
	else if (dp->dp_Type == ACTION_FINDOUTPUT)
		dbg("ACTION_FINDOUTPUT\n");

	dbg("  file handle = $l\n", fh);
	dbg("  lock = $l\n", lock);
	dbg("  name = $S\n", name);

	struct FindXxxRequest *req = (struct FindXxxRequest *)request_buffer;
	req->has_response = 0;
	req->type = dp->dp_Type;
	req->key = lock == NULL ? 0 : lock->fl_Key;

	int nlen = *name;
	if (request_too_long(dp, sizeof(struct FindXxxRequest) + nlen))
		return;
	memcpy(req->name, name, nlen + 1);

	write_req_and_wait_for_res(sizeof(struct FindXxxRequest) + nlen);

	struct FindXxxResponse *res = (struct FindXxxResponse *)request_buffer;
	if (!res->success)
	{
		dbg("  Failed, error code $l\n", (LONG)res->error_code);
		dp->dp_Res1 = DOSFALSE;
		dp->dp_Res2 = res->error_code;
	}
	else
	{
		struct MyFile *mf = (struct MyFile *)AllocMem(sizeof(struct MyFile), MEMF_PUBLIC);
		if (mf == NULL)
		{
			// Close it again on the Pi; nothing refers to it. The request
			// overwrites the response, so take the handle first.
			LONG pi_arg1 = res->arg1;
			struct EndRequest *ereq = (struct EndRequest *)request_buffer;
			ereq->has_response = 0;
			ereq->type = ACTION_END;
			ereq->arg1 = pi_arg1;
			write_req_and_wait_for_res(sizeof(struct EndRequest));

			dp->dp_Res1 = DOSFALSE;
			dp->dp_Res2 = ERROR_NO_FREE_STORE;
			reply_packet(dp);
			return;
		}
		mf->arg1 = res->arg1;
		mf->gen = conn_gen;
		mf->next = my_files;
		my_files = mf;

		fh->fh_Arg1 = (LONG)mf;
		fh->fh_Type = mp;
		fh->fh_Port = DOSFALSE;  // Not an interactive file.

		dp->dp_Res1 = DOSTRUE;
		dp->dp_Res2 = 0;
	}

	reply_packet(dp);
}

void action_read(struct DosPacket *dp)
{
	ULONG arg1 = file_arg1(dp);
	UBYTE *dst = (UBYTE *)dp->dp_Arg2;
	int length = dp->dp_Arg3;

	dbg("ACTION_READ\n");
	dbg("  arg1 = $l\n", arg1);
	dbg("  length = $l\n", length);

	if (length == 0)
	{
		dp->dp_Res1 = -1;
		dp->dp_Res2 = ERROR_INVALID_LOCK; // This is not the correct error.
		reply_packet(dp);
		return;
	}

	int total_read = 0;
	while (length)
	{
		int to_read = length;
		if (to_read > BUFFER_SIZE)
			to_read = BUFFER_SIZE;

		struct ReadRequest *req = (struct ReadRequest *)request_buffer;
		req->has_response = 0;
		req->type = dp->dp_Type;
		req->arg1 = arg1;
		req->address = data_buffer_address;
		req->length = to_read;

		write_req_and_wait_for_res(sizeof(struct ReadRequest));

		struct ReadResponse *res = (struct ReadResponse *)request_buffer;
		if (!res->success)
		{
			dbg("  Failed, error code $l\n", (LONG)res->error_code);
			dp->dp_Res1 = -1;
			dp->dp_Res2 = res->error_code;
			reply_packet(dp);
			return;
		}

		if (res->actual < 0 || res->actual > to_read)
		{
			// More than asked for would overrun the caller's buffer.
			dbg("  Bad READ result $l for $l\n", (LONG)res->actual, (LONG)to_read);
			bad_response();
			dp->dp_Res1 = -1;
			dp->dp_Res2 = ERROR_NO_DISK;
			reply_packet(dp);
			return;
		}

		if (res->actual)
		{
			ReadMemA314(dst, data_buffer_address, res->actual);
			dst += res->actual;
			total_read += res->actual;
			length -= res->actual;
		}

		if (res->actual < to_read)
			break;
	}

	dp->dp_Res1 = total_read;
	dp->dp_Res2 = 0;
	reply_packet(dp);
}

void action_write(struct DosPacket *dp)
{
	ULONG arg1 = file_arg1(dp);
	UBYTE *src = (UBYTE *)dp->dp_Arg2;
	int length = dp->dp_Arg3;

	dbg("ACTION_WRITE\n");
	dbg("  arg1 = $l\n", arg1);
	dbg("  length = $l\n", length);

	int total_written = 0;
	while (length)
	{
		int to_write = length;
		if (to_write > BUFFER_SIZE)
			to_write = BUFFER_SIZE;

		WriteMemA314(data_buffer_address, src, to_write);

		struct WriteRequest *req = (struct WriteRequest *)request_buffer;
		req->has_response = 0;
		req->type = dp->dp_Type;
		req->arg1 = arg1;
		req->address = data_buffer_address;
		req->length = to_write;

		write_req_and_wait_for_res(sizeof(struct WriteRequest));

		struct WriteResponse *res = (struct WriteResponse *)request_buffer;
		if (!res->success)
		{
			dbg("  Failed, error code $l\n", (LONG)res->error_code);
			dp->dp_Res1 = total_written != 0 ? total_written : -1;	// Write() is -1 on error
			dp->dp_Res2 = res->error_code;
			reply_packet(dp);
			return;
		}

		if (res->actual < 0 || res->actual > to_write)
		{
			dbg("  Bad WRITE result $l for $l\n", (LONG)res->actual, (LONG)to_write);
			bad_response();
			dp->dp_Res1 = total_written != 0 ? total_written : -1;
			dp->dp_Res2 = ERROR_NO_DISK;
			reply_packet(dp);
			return;
		}

		if (res->actual)
		{
			src += res->actual;
			total_written += res->actual;
			length -= res->actual;
		}

		if (res->actual < to_write)
			break;
	}

	dp->dp_Res1 = total_written;
	dp->dp_Res2 = 0;
	reply_packet(dp);
}

void action_seek(struct DosPacket *dp)
{
	ULONG arg1 = file_arg1(dp);
	LONG new_pos = dp->dp_Arg2;
	LONG mode = dp->dp_Arg3;

	dbg("ACTION_SEEK\n");
	dbg("  arg1 = $l\n", arg1);
	dbg("  new_pos = $l\n", new_pos);
	dbg("  mode = $l\n", mode);

	struct SeekRequest *req = (struct SeekRequest *)request_buffer;
	req->has_response = 0;
	req->type = dp->dp_Type;
	req->arg1 = arg1;
	req->new_pos = new_pos;
	req->mode = mode;

	write_req_and_wait_for_res(sizeof(struct SeekRequest));

	struct SeekResponse *res = (struct SeekResponse *)request_buffer;
	if (!res->success)
	{
		dbg("  Failed, error code $l\n", (LONG)res->error_code);
		dp->dp_Res1 = -1;
		dp->dp_Res2 = res->error_code;
	}
	else
	{
		dp->dp_Res1 = res->old_pos;
		dp->dp_Res2 = 0;
	}

	reply_packet(dp);
}

void action_end(struct DosPacket *dp)
{
	ULONG arg1 = file_arg1(dp);

	dbg("ACTION_END\n");
	dbg("  arg1 = $l\n", arg1);

	struct EndRequest *req = (struct EndRequest *)request_buffer;
	req->has_response = 0;
	req->type = dp->dp_Type;
	req->arg1 = arg1;

	write_req_and_wait_for_res(sizeof(struct EndRequest));

	// The handle is freed either way (handle_packet()); a close that did not
	// reach the Pi may have lost buffered data, so report it.
	struct EndResponse *res = (struct EndResponse *)request_buffer;
	dp->dp_Res1 = res->success ? DOSTRUE : DOSFALSE;
	dp->dp_Res2 = res->success ? 0 : res->error_code;
	reply_packet(dp);
}

void action_delete_object(struct DosPacket *dp)
{
	struct FileLock *lock = (struct FileLock *)BADDR(dp->dp_Arg1);
	unsigned char *name = (unsigned char *)BADDR(dp->dp_Arg2);

	dbg("ACTION_DELETE_OBJECT\n");
	dbg("  lock = $l\n", lock);
	dbg("  name = $S\n", name);

	struct DeleteObjectRequest *req = (struct DeleteObjectRequest *)request_buffer;
	req->has_response = 0;
	req->type = dp->dp_Type;
	req->key = lock == NULL ? 0 : lock->fl_Key;

	int nlen = *name;
	if (request_too_long(dp, sizeof(struct DeleteObjectRequest) + nlen))
		return;
	memcpy(req->name, name, nlen + 1);

	write_req_and_wait_for_res(sizeof(struct DeleteObjectRequest) + nlen);

	struct DeleteObjectResponse *res = (struct DeleteObjectResponse *)request_buffer;
	if (!res->success)
	{
		dbg("  Failed, error code $l\n", (LONG)res->error_code);
		dp->dp_Res1 = DOSFALSE;
		dp->dp_Res2 = res->error_code;
	}
	else
	{
		dp->dp_Res1 = DOSTRUE;
		dp->dp_Res2 = 0;
	}

	reply_packet(dp);
}

void action_rename_object(struct DosPacket *dp)
{
	struct FileLock *lock = (struct FileLock *)BADDR(dp->dp_Arg1);
	unsigned char *name = (unsigned char *)BADDR(dp->dp_Arg2);
	struct FileLock *target_dir = (struct FileLock *)BADDR(dp->dp_Arg3);
	unsigned char *new_name = (unsigned char *)BADDR(dp->dp_Arg4);

	dbg("ACTION_RENAME_OBJECT\n");
	dbg("  lock = $l\n", lock);
	dbg("  name = $S\n", name);
	dbg("  target directory = $l\n", lock);
	dbg("  new name = $S\n", new_name);

	struct RenameObjectRequest *req = (struct RenameObjectRequest *)request_buffer;
	req->has_response = 0;
	req->type = dp->dp_Type;
	req->key = lock == NULL ? 0 : lock->fl_Key;
	req->target_dir = target_dir == NULL ? 0 : target_dir->fl_Key;

	int nlen = *name;
	int nnlen = *new_name;
	if (request_too_long(dp, sizeof(struct RenameObjectRequest) + nlen + nnlen))
		return;

	req->name_len = nlen;
	req->new_name_len = nnlen;

	unsigned char *p = &(req->new_name_len) + 1;
	memcpy(p, name + 1, nlen);
	p += nlen;
	memcpy(p, new_name + 1, nnlen);

	write_req_and_wait_for_res(sizeof(struct RenameObjectRequest) + nlen + nnlen);

	struct RenameObjectResponse *res = (struct RenameObjectResponse *)request_buffer;
	if (!res->success)
	{
		dbg("  Failed, error code $l\n", (LONG)res->error_code);
		dp->dp_Res1 = DOSFALSE;
		dp->dp_Res2 = res->error_code;
	}
	else
	{
		dp->dp_Res1 = DOSTRUE;
		dp->dp_Res2 = 0;
	}

	reply_packet(dp);
}

void action_create_dir(struct DosPacket *dp)
{
	struct FileLock *lock = (struct FileLock *)BADDR(dp->dp_Arg1);
	unsigned char *name = (unsigned char *)BADDR(dp->dp_Arg2);

	dbg("ACTION_CREATE_DIR\n");
	dbg("  lock = $l\n", lock);
	dbg("  name = $S\n", name);

	struct CreateDirRequest *req = (struct CreateDirRequest *)request_buffer;
	req->has_response = 0;
	req->type = dp->dp_Type;
	req->key = lock == NULL ? 0 : lock->fl_Key;

	int nlen = *name;
	if (request_too_long(dp, sizeof(struct CreateDirRequest) + nlen))
		return;
	memcpy(req->name, name, nlen + 1);

	write_req_and_wait_for_res(sizeof(struct CreateDirRequest) + nlen);

	struct CreateDirResponse *res = (struct CreateDirResponse *)request_buffer;
	if (!res->success)
	{
		dbg("  Failed, error code $l\n", (LONG)res->error_code);
		dp->dp_Res1 = DOSFALSE;
		dp->dp_Res2 = res->error_code;
	}
	else
	{
		struct FileLock *lock = create_and_add_file_lock(res->key, SHARED_LOCK);

		dbg("  Returning lock $l\n", lock);
		dp->dp_Res1 = MKBADDR(lock);
		dp->dp_Res2 = 0;
	}

	reply_packet(dp);
}

void action_set_protect(struct DosPacket *dp)
{
	struct FileLock *lock = (struct FileLock *)BADDR(dp->dp_Arg2);
	unsigned char *name = (unsigned char *)BADDR(dp->dp_Arg3);
	long mask = dp->dp_Arg4;

	dbg("ACTION_SET_PROTECT\n");
	dbg("  lock = $l\n", lock);
	dbg("  name = $S\n", name);
	dbg("  mask = $l\n", mask);

	struct SetProtectRequest *req = (struct SetProtectRequest *)request_buffer;
	req->has_response = 0;
	req->type = dp->dp_Type;
	req->key = lock == NULL ? 0 : lock->fl_Key;
	req->mask = mask;

	int nlen = *name;
	if (request_too_long(dp, sizeof(struct SetProtectRequest) + nlen))
		return;
	memcpy(req->name, name, nlen + 1);

	write_req_and_wait_for_res(sizeof(struct SetProtectRequest) + nlen);

	struct SetProtectResponse *res = (struct SetProtectResponse *)request_buffer;
	if (!res->success)
	{
		dbg("  Failed, error code $l\n", (LONG)res->error_code);
		dp->dp_Res1 = DOSFALSE;
		dp->dp_Res2 = res->error_code;
	}
	else
	{
		dp->dp_Res1 = DOSTRUE;
		dp->dp_Res2 = 0;
	}

	reply_packet(dp);
}

void action_set_comment(struct DosPacket *dp)
{
	struct FileLock *lock = (struct FileLock *)BADDR(dp->dp_Arg2);
	unsigned char *name = (unsigned char *)BADDR(dp->dp_Arg3);
	unsigned char *comment = (unsigned char *)BADDR(dp->dp_Arg4);

	dbg("ACTION_SET_COMMENT\n");
	dbg("  lock = $l\n", lock);
	dbg("  name = $S\n", name);
	dbg("  comment = $S\n", comment);

	struct SetCommentRequest *req = (struct SetCommentRequest *)request_buffer;
	req->has_response = 0;
	req->type = dp->dp_Type;
	req->key = lock == NULL ? 0 : lock->fl_Key;

	int nlen = *name;
	int clen = *comment;
	if (request_too_long(dp, sizeof(struct SetCommentRequest) + nlen + clen))
		return;

	req->name_len = nlen;
	req->comment_len = clen;

	unsigned char *p = &(req->comment_len) + 1;
	memcpy(p, name + 1, nlen);
	p += nlen;
	memcpy(p, comment + 1, clen);

	write_req_and_wait_for_res(sizeof(struct SetCommentRequest) + nlen + clen);

	struct SetCommentResponse *res = (struct SetCommentResponse *)request_buffer;
	if (!res->success)
	{
		dbg("  Failed, error code $l\n", (LONG)res->error_code);
		dp->dp_Res1 = DOSFALSE;
		dp->dp_Res2 = res->error_code;
	}
	else
	{
		dp->dp_Res1 = DOSTRUE;
		dp->dp_Res2 = 0;
	}

	reply_packet(dp);
}

void action_same_lock(struct DosPacket *dp)
{
	struct FileLock *lock1 = (struct FileLock *)BADDR(dp->dp_Arg1);
	struct FileLock *lock2 = (struct FileLock *)BADDR(dp->dp_Arg2);

	dbg("ACTION_SAME_LOCK\n");
	dbg("  locks to compare = $l $l\n", lock1, lock2);

	struct SameLockRequest *req = (struct SameLockRequest *)request_buffer;
	req->has_response = 0;
	req->type = dp->dp_Type;
	req->key1 = lock1 == NULL ? 0 : lock1->fl_Key;
	req->key2 = lock2 == NULL ? 0 : lock2->fl_Key;

	write_req_and_wait_for_res(sizeof(struct SameLockRequest));

	struct SameLockResponse *res = (struct SameLockResponse *)request_buffer;
	dp->dp_Res1 = res->success ? DOSTRUE : DOSFALSE;
	dp->dp_Res2 = res->error_code;

	reply_packet(dp);
}

void fill_info_data(struct InfoData *id)
{
	memset(id, 0, sizeof(struct InfoData));
	id->id_BytesPerBlock = 512;
	if (!connected)
	{
		id->id_DiskState = ID_VALIDATED;
		id->id_DiskType = ID_NO_DISK_PRESENT;
		return;
	}
	id->id_DiskState = ID_VALIDATED;
	id->id_NumBlocks = 512 * 1024;
	id->id_NumBlocksUsed = 10;
	id->id_DiskType = ID_DOS_DISK;
	id->id_VolumeNode = MKBADDR(my_volume);
	id->id_InUse = DOSTRUE;
}

void action_info(struct DosPacket *dp)
{
	struct FileLock *lock = (struct FileLock *)BADDR(dp->dp_Arg1);
	struct InfoData *id = (struct InfoData *)BADDR(dp->dp_Arg2);

	dbg("ACTION_INFO\n");
	dbg("  lock = $l\n", lock);

	fill_info_data(id);
	dp->dp_Res1 = DOSTRUE;
	dp->dp_Res2 = 0;
	if (!connected)
	{
		reply_packet(dp);
		return;
	}

	struct InfoRequest *req = (struct InfoRequest *)request_buffer;
	req->has_response = 0;
	req->type = dp->dp_Type;
	req->key = lock == NULL ? 0 : lock->fl_Key;

	if (!write_req_and_wait_for_res(sizeof(struct InfoRequest)))
	{
		fill_info_data(id);	// the link dropped: no disk now
		reply_packet(dp);
		return;
	}

	struct InfoResponse *res = (struct InfoResponse *)request_buffer;
	if (!res->success)
	{
		dbg("  Failed, error code $l\n", (LONG)res->error_code);
	}
	else
	{
		id->id_NumBlocks = res->total_blocks;
		id->id_NumBlocksUsed = res->used_blocks;
		id->id_BytesPerBlock = res->block_size;
	}

	dp->dp_Res1 = DOSTRUE;
	dp->dp_Res2 = 0;
	reply_packet(dp);
}

void action_disk_info(struct DosPacket *dp)
{
	struct InfoData *id = (struct InfoData *)BADDR(dp->dp_Arg1);

	dbg("ACTION_DISK_INFO\n");

	fill_info_data(id);
	if (!connected)
	{
		dp->dp_Res1 = DOSTRUE;
		dp->dp_Res2 = 0;
		reply_packet(dp);
		return;
	}

	struct DiskInfoRequest *req = (struct DiskInfoRequest *)request_buffer;
	req->has_response = 0;
	req->type = dp->dp_Type;

	if (!write_req_and_wait_for_res(sizeof(struct DiskInfoRequest)))
	{
		fill_info_data(id);	// the link dropped: no disk now
		dp->dp_Res1 = DOSTRUE;
		dp->dp_Res2 = 0;
		reply_packet(dp);
		return;
	}

	struct DiskInfoResponse *res = (struct DiskInfoResponse *)request_buffer;
	if (!res->success)
	{
		dbg("  Failed, error code $l\n", (LONG)res->error_code);
	}
	else
	{
		id->id_NumBlocks = res->total_blocks;
		id->id_NumBlocksUsed = res->used_blocks;
		id->id_BytesPerBlock = res->block_size;
	}

	dp->dp_Res1 = DOSTRUE;
	dp->dp_Res2 = 0;
	reply_packet(dp);
}

void action_is_filesystem(struct DosPacket *dp)
{
	dbg("ACTION_IS_FILESYSTEM\n");

	dp->dp_Res1 = DOSTRUE;
	dp->dp_Res2 = 0;
	reply_packet(dp);
}

void action_unsupported(struct DosPacket *dp)
{
	dbg("ACTION_UNSUPPORTED\n");
	dbg("  Unsupported action: $l\n", (ULONG)dp->dp_Type);

	struct UnsupportedRequest *req = (struct UnsupportedRequest *)request_buffer;
	req->has_response = 0;
	req->type = ACTION_UNSUPPORTED;
	req->dp_Type = dp->dp_Type;

	write_req_and_wait_for_res(sizeof(struct UnsupportedRequest));

	struct UnsupportedResponse *res = (struct UnsupportedResponse *)request_buffer;
	dp->dp_Res1 = res->success ? DOSTRUE : DOSFALSE;
	dp->dp_Res2 = res->error_code;
	reply_packet(dp);
}

static void reply_error(struct DosPacket *dp, LONG res1, LONG error)
{
	dp->dp_Res1 = res1;
	dp->dp_Res2 = error;
	reply_packet(dp);
}

// TRUE if the lock (BPTR) is NULL or comes from the current connection.
static BOOL lock_current(ULONG bptr)
{
	struct MyLock *ml = (struct MyLock *)BADDR(bptr);
	return ml == NULL || ml->gen == conn_gen;
}

// Which packet arguments are locks: bit 0 = Arg1, bit 1 = Arg2, bit 2 = Arg3.
static ULONG lock_args(LONG type)
{
	switch (type)
	{
	case ACTION_LOCATE_OBJECT:
	case ACTION_COPY_DIR:
	case ACTION_PARENT:
	case ACTION_EXAMINE_OBJECT:
	case ACTION_EXAMINE_NEXT:
	case ACTION_DELETE_OBJECT:
	case ACTION_CREATE_DIR:
	case ACTION_INFO:
		return 1;
	case ACTION_FINDUPDATE:
	case ACTION_FINDINPUT:
	case ACTION_FINDOUTPUT:
	case ACTION_SET_PROTECT:
	case ACTION_SET_COMMENT:
		return 2;
	case ACTION_RENAME_OBJECT:
		return 1 | 4;
	case ACTION_SAME_LOCK:
		return 1 | 2;
	default:
		return 0;
	}
}

static BOOL is_file_action(LONG type)
{
	return type == ACTION_READ || type == ACTION_WRITE || type == ACTION_SEEK ||
	       type == ACTION_END || type == ACTION_EXAMINE_FH;
}

static void dispatch_action(struct DosPacket *dp)
{
	switch (dp->dp_Type)
	{
	case ACTION_LOCATE_OBJECT: action_locate_object(dp); break;
	case ACTION_FREE_LOCK: action_free_lock(dp); break;
	case ACTION_COPY_DIR: action_copy_dir(dp); break;
	case ACTION_PARENT: action_parent(dp); break;
	case ACTION_EXAMINE_OBJECT: action_examine_object(dp); break;
	case ACTION_EXAMINE_NEXT: action_examine_next(dp); break;
	case ACTION_EXAMINE_FH: action_examine_fh(dp); break;

	case ACTION_FINDUPDATE: action_findxxx(dp); break;
	case ACTION_FINDINPUT: action_findxxx(dp); break;
	case ACTION_FINDOUTPUT: action_findxxx(dp); break;
	case ACTION_READ: action_read(dp); break;
	case ACTION_WRITE: action_write(dp); break;
	case ACTION_SEEK: action_seek(dp); break;
	case ACTION_END: action_end(dp); break;
	//case ACTION_TRUNCATE: action_truncate(dp); break;

	case ACTION_DELETE_OBJECT: action_delete_object(dp); break;
	case ACTION_RENAME_OBJECT: action_rename_object(dp); break;
	case ACTION_CREATE_DIR: action_create_dir(dp); break;

	case ACTION_SET_PROTECT: action_set_protect(dp); break;
	case ACTION_SET_COMMENT: action_set_comment(dp); break;
	//case ACTION_SET_DATE: action_set_date(dp); break;

	case ACTION_SAME_LOCK: action_same_lock(dp); break;
	case ACTION_DISK_INFO: action_disk_info(dp); break;
	case ACTION_INFO: action_info(dp); break;

	case ACTION_IS_FILESYSTEM: action_is_filesystem(dp); break;

	/*
	case ACTION_CURRENT_VOLUME: action_current_volume(dp); break;
	case ACTION_RENAME_DISK: action_rename_disk(dp); break;

	case ACTION_DIE: //action_die(dp); break;
	case ACTION_MORE_CACHE: //action_more_cache(dp); break;
	case ACTION_FLUSH: //action_flush(dp); break;
	case ACTION_INHIBIT: //action_inhibit(dp); break;
	case ACTION_WRITE_PROTECT: //action_write_protect(dp); break;
	*/

	default: action_unsupported(dp); break;
	}
}

// Offline (no Pi): fail with ERROR_NO_DISK, so DOS offers Retry/Cancel.
// A lock or file handle from before a drop: free it, or fail it with
// ERROR_INVALID_LOCK; never send its old key to the Pi.
void handle_packet(struct DosPacket *dp)
{
	LONG type = dp->dp_Type;
	LONG fail_res1 = (type == ACTION_READ || type == ACTION_WRITE || type == ACTION_SEEK) ? -1 : DOSFALSE;

	switch (type)
	{
	case ACTION_IS_FILESYSTEM:
	case ACTION_FREE_LOCK:		// handles stale locks and offline itself
		dispatch_action(dp);
		return;
	case ACTION_INFO:
	case ACTION_DISK_INFO:
		if (!connected || lock_current(dp->dp_Arg1) || type == ACTION_DISK_INFO)
		{
			dispatch_action(dp);
			return;
		}
		break;
	}

	if (is_file_action(type))
	{
		struct MyFile *mf = find_my_file(dp->dp_Arg1);
		if (mf == NULL)
		{
			// Not one of ours (any more): closing it again is harmless.
			if (type == ACTION_END)
				reply_error(dp, DOSTRUE, 0);
			else
				reply_error(dp, fail_res1, ERROR_INVALID_LOCK);
			return;
		}

		BOOL current = connected && mf->gen == conn_gen;
		if (type == ACTION_END && !current)
		{
			// The link dropped while it was open: what was written before is on
			// the Pi (a314fs.py closes its files), but say the close failed.
			free_my_file(mf);
			reply_error(dp, DOSFALSE, connected ? ERROR_INVALID_LOCK : ERROR_NO_DISK);
			return;
		}
		if (!connected)
		{
			reply_error(dp, fail_res1, ERROR_NO_DISK);
			return;
		}
		if (!current)
		{
			reply_error(dp, fail_res1, ERROR_INVALID_LOCK);
			return;
		}

		dispatch_action(dp);
		if (type == ACTION_END)
			free_my_file(mf);
		return;
	}

	if (!connected)
	{
		reply_error(dp, DOSFALSE, ERROR_NO_DISK);
		return;
	}

	ULONG m = lock_args(type);
	if (((m & 1) && !lock_current(dp->dp_Arg1)) ||
	    ((m & 2) && !lock_current(dp->dp_Arg2)) ||
	    ((m & 4) && !lock_current(dp->dp_Arg3)))
	{
		reply_error(dp, DOSFALSE, ERROR_INVALID_LOCK);
		return;
	}

	dispatch_action(dp);
}

void start(__reg("a0") struct DosPacket *startup_packet)
{
	SysBase = *(struct ExecBase **)4;
	DOSBase = (struct DosLibrary *)OpenLibrary(DOSNAME, 0);

	mp = MyCreatePort(NULL, 0);

	startup_fs_handler(startup_packet);

	ULONG mp_sig = 1UL << mp->mp_SigBit;
	ULONG retry_sig = retry_mp != NULL ? (1UL << retry_mp->mp_SigBit) : 0;

	while (1)
	{
		ULONG a314_sig = a314_open ? (1UL << a314_mp->mp_SigBit) : 0;
		ULONG input_sig = (disk_event_pending && input_busy) ? (1UL << input_mp->mp_SigBit) : 0;
		Wait(mp_sig | retry_sig | a314_sig | input_sig | abandoned_sigs());

		poll_connection();
		reap_abandoned();
		if (volume_sync_pending)
			sync_volume();
		flush_disk_event();

		struct StandardPacket *sp;
		while ((sp = (struct StandardPacket *)GetMsg(mp)) != NULL)
			handle_packet((struct DosPacket *)(sp->sp_Msg.mn_Node.ln_Name));
	}

	dbg("Shutting down\n");

	// More cleaning up is necessary if the handler is to exit.
	CloseLibrary((struct Library *)DOSBase);
}
