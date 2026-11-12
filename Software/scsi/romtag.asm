    XREF    _device_name
    XREF    _id_string
    XREF    _auto_init_tables
    XREF    _cli_main
    XDEF    _diag_area

    incdir  "/opt/ndk32/Include_I"

    include exec/resident.i
    include libraries/configregs.i
    include lvo/exec_lib.i

NT_DEVICE:      equ    3
VERSION:        equ    1
PRIORITY:       equ    -5

        section    CODE,code

        jsr     _cli_main
        rts

romtag:
        dc.w    RTC_MATCHWORD
        dc.l    romtag
        dc.l    endcode
        dc.b    RTF_AUTOINIT|RTF_COLDSTART
        dc.b    VERSION
        dc.b    NT_DEVICE
        dc.b    PRIORITY
        dc.l    _device_name
        dc.l    _id_string
        dc.l    _auto_init_tables

; The DiagArea of the ConfigDev that a314scsi's BootNodes carry, so that strap
; can boot from a partition: it calls BootPoint (SysBase in a6), which starts
; dos.library, and dos then boots from the BootNode at the head of the MountList.
        cnop    0,2
_diag_area:
        dc.b    DAC_WORDWIDE|DAC_CONFIGTIME     ; da_Config
        dc.b    0                               ; da_Flags
        dc.w    diag_end-_diag_area             ; da_Size
        dc.w    0                               ; da_DiagPoint
        dc.w    boot_point-_diag_area           ; da_BootPoint
        dc.w    0                               ; da_Name
        dc.w    0                               ; da_Reserved01
        dc.w    0                               ; da_Reserved02

boot_point:
        lea     dos_name(pc),a1
        jsr     _LVOFindResident(a6)
        tst.l   d0
        beq.s   .nodos
        move.l  d0,a0
        move.l  RT_INIT(a0),a0
        jsr     (a0)
.nodos:
        rts

dos_name:
        dc.b    'dos.library',0
        cnop    0,2
diag_end:
endcode:
