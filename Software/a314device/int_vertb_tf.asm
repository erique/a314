;
; Fallback poll for the TF model. VERTB server; a1 = &dev->poll.
; Counts frames, and while active signals the driver task every POLL_FRAMES
; frames, so a lost INT2 costs a short delay instead of a hang.
;
	XDEF	_VertbServerTF
	CODE

SIGB_INT	equ	14
SIGF_INT	equ	(1<<SIGB_INT)
POLL_FRAMES	equ	5		; 10 Hz on PAL, 12 Hz on NTSC

; Must match struct TFPoll in device.h.
tfp_Task	equ	0
tfp_Active	equ	4		; poll while non-zero
tfp_Count	equ	6		; frames to the next poll
tfp_Ticks	equ	8		; frames since start, always counted

_VertbServerTF:
		addq.l	#1,tfp_Ticks(a1)
		tst.w	tfp_Active(a1)
		beq.s	.done
		subq.w	#1,tfp_Count(a1)
		bgt.s	.done
		move.w	#POLL_FRAMES,tfp_Count(a1)

		move.l	tfp_Task(a1),a1
		move.l	#SIGF_INT,d0
		move.l	$4.w,a6
		jsr	-324(a6)		; Signal()

.done:		moveq	#0,d0			; Z set: the other VERTB servers run too
		rts
