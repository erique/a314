;
; Fallback poll for the TF model. VERTB server; a1 = &dev->poll:
;   0 task, 4 active (word), 6 count (word), 8 ticks (long)
; Counts frames, and while active signals the driver task every POLL_FRAMES
; frames, so a lost INT2 costs a short delay instead of a hang.
;
	XDEF	_VertbServerTF
	CODE

SIGB_INT	equ	14
SIGF_INT	equ	(1<<SIGB_INT)
POLL_FRAMES	equ	5		; 10 Hz on PAL, 12 Hz on NTSC

_VertbServerTF:
		addq.l	#1,8(a1)		; ticks
		tst.w	4(a1)			; active?
		beq.s	.done
		subq.w	#1,6(a1)
		bgt.s	.done
		move.w	#POLL_FRAMES,6(a1)

		move.l	(a1),a1			; task
		move.l	#SIGF_INT,d0
		move.l	$4.w,a6
		jsr	-324(a6)		; Signal()

.done:		moveq	#0,d0			; Z set: the other VERTB servers run too
		rts
