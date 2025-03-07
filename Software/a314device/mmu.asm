    incdir "/opt/ndk32/Include_I"

	include	devices/timer.i
	include	exec/exec.i
	include	utility/hooks.i
    include utility/tagitem.i

	include	libraries/expansion.i
	include	libraries/configvars.i

	include	lvo/exec_lib.i
	include	lvo/expansion_lib.i
	include	lvo/timer_lib.i

	XDEF	_SetMMU
	CODE

kprintf	MACRO
	ENDM
	
_SetMMU	; ( addr:a0 size:d0 flags:d1 exec:a6 )
	; returns old flags in d0/d1
	; ( d0-d1/a0-a1 are scratch )

MAPP_CACHEINHIBIT       equ     (1<<6)
MAPP_COPYBACK           equ     (1<<13)
MAPP_IMPRECISE          equ     (1<<21)
MAPP_NONSERIALIZED      equ     (1<<29)
MAPP_IO                 equ     (1<<30)

_LVOGetMapping              	EQU	-36
_LVOReleaseMapping          	EQU	-42
_LVOGetPageSize             	EQU	-48
_LVOGetMMUType              	EQU	-54
_LVOLockMMUContext          	EQU	-72
_LVOUnlockMMUContext        	EQU	-78
_LVOSetPropertiesA          	EQU	-84
_LVOGetPropertiesA          	EQU	-90
_LVORebuildTree             	EQU	-96
_LVOSuperContext            	EQU	-144
_LVODefaultContext          	EQU	-150
_LVOLockContextList         	EQU	-210
_LVOUnlockContextList       	EQU	-216
_LVOSetPropertyList         	EQU	-228
_LVORebuildTreesA           	EQU	-360

	kprintf	"SetMMU(addr:%lx size:%lx flags:%lx)",a0,d0,d1

		movem.l	d2-d7/a2-a5,-(sp)

		movem.l	d0/a0,-(sp)		; (sp),4(sp) = size,addr
		move.l	d1,a3			; a3 = flags

		moveq.l	#43,d0
		lea	.mmuName,a1
		CALLLIB	_LVOOpenLibrary
	kprintf	"mmu.library        = %lx",d0
		tst.l	d0
		bne.b	.mmulib_ok

		pea	.exit(pc)
		bra	.failed

	; Verify MMU presence
.mmulib_ok	move.l	d0,a6
		CALLLIB	_LVOGetMMUType
		tst.b	d0
		bne.b	.mmu_ok

	kprintf	"GetMMUType         = <none>"
		pea	.nommu(pc)
		bra	.failed

.mmu_ok
		sub.b	#'0',d0
	kprintf	"GetMMUType         = 680%ld0",d0

	; Get contexts
		CALLLIB	_LVODefaultContext
	kprintf	"DefaultContext     = %lx",d0
		movea.l	d0,a5			; a5 = ctx
		move.l	d0,a0
		CALLLIB	_LVOSuperContext
	kprintf	"SuperContext       = %lx",d0
		movea.l	d0,a4			; a4 = sctx

		move.l	a5,d0
		CALLLIB	_LVOGetPageSize
	kprintf	"GetPageSize (ctx)  = %lx (%ld bytes)",d0,d0
		move.l	d0,d7			; d7 = pagesize

		move.l	a4,d0
		CALLLIB	_LVOGetPageSize
	kprintf	"GetPageSize (sctx) = %lx (%ld bytes)",d0,d0
		cmp.l	d0,d7
		beq.b	.page_ok

		pea	.nommu(pc)
		bra	.failed

.page_ok
	; adjust address and size to match page size
		movem.l	(sp),d0/d1

	kprintf	"Requested region   = %lx,%lx",d1,d0

		subq.l	#1,d7
		move.l	d7,d6
		not.l	d6
		add.l	d1,d0
		add.l	d7,d1
		and.l	d6,d1
		and.l	d6,d0
		sub.l	d1,d0

	kprintf	"Aligned region     = %lx,%lx",d1,d0

		tst.l	d0
		bne.b	.sizeok

	kprintf	"Size is 0!"
		pea	.nommu(pc)
		bra	.failed

.sizeok
		movem.l	d0/d1,(sp)		; (sp),4(sp) = adjusted size/addr

	; Lock contexts
		CALLLIB	_LVOLockContextList
		movea.l	a5,a0
		CALLLIB	_LVOLockMMUContext (ctx)
		movea.l	a4,a0
		CALLLIB	_LVOLockMMUContext (sctx)

	; Get mapping
		move.l	a5,a0
		CALLLIB	_LVOGetMapping (ctx)
	kprintf	"ctx mapping        = %lx",d0
		move.l	d0,d5			; d5 = ctx mapping
		bne.b	.ctx_ok

		pea	.unlock(pc)
		bra	.failed

.ctx_ok		move.l	a4,a0
		CALLLIB	_LVOGetMapping (sctx)
	kprintf	"sctx mapping       = %lx",d0
		move.l	d0,d4			; d4 = ctx mapping
		bne.b	.sctx_ok

		pea	.release(pc)
		bra	.failed

.sctx_ok	movem.l	(sp),d6/d7		; d6 = size, d7 = addr

		move.l	a5,a0
		move.l	d7,a1
		lea	.tagDone,a2
	kprintf	"GetPropertiesA     = %lx,%lx,%lx",a0,a1,(a2)
		CALLLIB	_LVOGetPropertiesA (ctx,from,TAG_DONE)
	kprintf	"GetPropertiesA     => %lx",d0
		move.l	d0,(sp)			; (sp) = old flags (ctx)

		move.l	a5,a0
		move.l	a3,d1
		move.l	#MAPP_IO|MAPP_CACHEINHIBIT|MAPP_IMPRECISE|MAPP_NONSERIALIZED|MAPP_COPYBACK,d2
		move.l	d6,d0
		move.l	d7,a1
		lea	.tagDone(pc),a2
	kprintf	"SetPropertiesA     = %lx,%lx,%lx %lx,%lx %lx",a0,d1,d2,a1,d0,(a2)
		CALLLIB	_LVOSetPropertiesA (ctx,flags,mask,from,size,TAG_DONE)
		tst.b	d0
		beq	.revert

		move.l	a4,a0
		move.l	d7,a1
		movem.l	(sp),d0/a1
		lea	.tagDone,a2
	kprintf	"GetPropertiesA     = %lx,%lx,%lx",a0,a1,(a2)
		CALLLIB	_LVOGetPropertiesA (sctx,from,TAG_DONE)
	kprintf	"GetPropertiesA     => %lx",d0
		move.l	d0,4(sp)		; 4(sp) = old flags (sctx)

		move.l	a4,a0
		move.l	a3,d1
		move.l	#MAPP_IO|MAPP_CACHEINHIBIT|MAPP_IMPRECISE|MAPP_NONSERIALIZED|MAPP_COPYBACK,d2
		move.l	d6,d0
		move.l	d7,a1
		lea	.tagDone(pc),a2
	kprintf	"SetPropertiesA     = %lx,%lx,%lx %lx,%lx %lx",a0,d1,d2,a1,d0,(a2)
		CALLLIB	_LVOSetPropertiesA (sctx,flags,mask,from,size,TAG_DONE)
		tst.b	d0
		beq	.revert

		sub.w	#4*3,sp
		move.l	a5,(sp)
		move.l	a4,4(sp)
		clr.l	8(sp)
		move.l	sp,a0
	kprintf	"RebuildTreesA      = %lx,%lx,%lx",(a0),4(a0),8(a0)
		CALLLIB	_LVORebuildTreesA (ctx,sctx,NULL)
		add.w	#4*3,sp
		tst.b	d0
		bne	.success

.revert
		pea	.cleanup(pc)
		move.l	a5,a0
		move.l	d5,a1
	kprintf	"SetPropertyList    = %lx,%lx",a0,a1
		CALLLIB _LVOSetPropertyList (ctx,ctxl)
		move.l	a4,a0
		move.l	d4,a1
	kprintf	"SetPropertyList    = %lx,%lx",a0,a1
		CALLLIB _LVOSetPropertyList (sctx,sctxl)

.failed
	kprintf	"Failure!"
		moveq.l	#-1,d0
		move.l	d0,4(sp)		; set return values
		move.l	d0,8(sp)
		rts

.success
	kprintf	"Success!"

.cleanup
		move.l	a4,a0
		move.l	d4,a1
;	kprintf	"ReleaseMapping     = %lx,%lx",a0,a1
		CALLLIB	_LVOReleaseMapping (sctx,sctxl)

.release
		move.l	a5,a0
		move.l	d5,a1
;	kprintf	"ReleaseMapping     = %lx,%lx",a0,a1
		CALLLIB	_LVOReleaseMapping (ctx,ctxl)

.unlock

	; Unlock contexts
		movea.l	a4,a0
;	kprintf	"UnlockMMUContext   = %lx",a0
		CALLLIB	_LVOUnlockMMUContext (sctx)
		movea.l	a5,a0
;	kprintf	"UnlockMMUContext   = %lx",a0
		CALLLIB	_LVOUnlockMMUContext (ctx)
;	kprintf	"UnlockContextList"
		CALLLIB	_LVOUnlockContextList

.nommu

		move.l	a6,a1
		move.l	4.w,a6
;	kprintf	"CloseLibrary       = %lx",a1
		CALLLIB	_LVOCloseLibrary

.exit:
		movem.l	(sp)+,d0/d1
		movem.l	(sp)+,d2-d7/a2-a5
	kprintf "DONE (%08lx / %08lx)",d0,d1
		rts

.mmuName	dc.b	"mmu.library",0
		even
.tagDone	dc.l	TAG_DONE
