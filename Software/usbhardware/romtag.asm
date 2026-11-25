	XREF	_device_name
	XREF	_id_string
	XREF	_auto_init_tables

	incdir	"/opt/ndk32/Include_I"

	include	exec/nodes.i
	include	exec/resident.i

VERSION:	equ	1
PRIORITY:	equ	0

		section	code,code

		moveq	#-1,d0
		rts

romtag:
		dc.w	RTC_MATCHWORD
		dc.l	romtag
		dc.l	endcode
		dc.b	RTF_AUTOINIT
		dc.b	VERSION
		dc.b	NT_DEVICE
		dc.b	PRIORITY
		dc.l	_device_name
		dc.l	_id_string
		dc.l	_auto_init_tables
endcode:

		cnop	0,4
