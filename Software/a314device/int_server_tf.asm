	XDEF	_IntServer
	CODE

ENABLE_KPRINTF

	include "kprintf.i"

SINT		equ	$3f4

BOARD_OFFSET	equ	40		; == offsetof(struct A314Device, tf_config)
TASK_OFFSET		equ	44		; == offsetof(struct A314Device, task)

SIGB_INT	equ	14
SIGF_INT	equ	(1<<SIGB_INT)

		; a1 points to device struct
_IntServer: move.l	BOARD_OFFSET(a1),a5

		lea 	SINT(a5),a5
		move.b	(a5),d0
		and.b	#1<<4,d0		; Bit 4 Amiga Interrupt.  1 = Pending, 0 = Not Pending
		bne.s	should_signal

		moveq	#0,d0			; Z clear
		rts

should_signal:
		move.b	#1<<4,(a5)		; clear bit 4

		move.l	$4.w,a6
		move.l	#SIGF_INT,d0
		lea.l	TASK_OFFSET(a1),a1	; pointer to driver task
		jsr		-324(a6)		; Signal()

		moveq	#-1,d0			; Z set
		rts
