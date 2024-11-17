	XDEF	_IntServer
	CODE

ENABLE_KPRINTF

	include "kprintf.i"

R_EVENTS_ADDRESS	equ	12
R_ENABLE_ADDRESS	equ	13
A_EVENTS_ADDRESS	equ	14
A_ENABLE_ADDRESS	equ	15

A_EVENT_R2A_TAIL        equ	1
A_EVENT_A2R_HEAD        equ	2

INTENA		equ	$dff09a
CMEM		equ	$e903f0

SIGB_INT	equ	14
SIGF_INT	equ	(1<<SIGB_INT)

		; a1 points to driver task
_IntServer:	lea.l	CMEM,a5

		move	#$4000,INTENA


;		kprintf "$$ irq"

		move.l	$4.w,a6
		move.l	a5,a0
		moveq.l	#$10,d0
		move.l	#(1<<11),d1 ;CACRF_ClearD
		jsr	    -642(a6)	; CacheClearE()

		move.b	A_EVENTS_ADDRESS(a5),d0	; A_EVENTS
		and.b	A_ENABLE_ADDRESS(a5),d0	; A_ENABLE
		and.b	#$f,d0
		bne.s	should_signal

		move	#$c000,INTENA
		moveq	#0,d0
		rts

should_signal:
;		kprintf "$$ irq %lx",d0
		move.b	#0,A_ENABLE_ADDRESS(a5)	; A_ENABLE

		move	#$c000,INTENA

;.loop	
;		move.w  $dff006,$dff180
;		btst 	#6,$bfe001
;		bne.b  .loop

;		kprintf "$$ signal"

		move.l	$4.w,a6
		move.l	#SIGF_INT,d0
		; a1 = pointer to driver task
		jsr	-324(a6)	; Signal()

		moveq	#0,d0
		rts
