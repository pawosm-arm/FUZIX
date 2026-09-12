# 1 "buffers.S"
;
;	External buffer support logic
;

	.common
# 1 "kernelu.def"
; FUZIX mnemonics for memory addresses etc

U_DATA__TOTALSIZE	.equ	0x200	; 256+256 bytes @ D000
Z80_TYPE		.equ	0	; just a old good Z80
USE_FANCY_MONITOR	.equ	1	; disabling this saves around approx 0.5KB

Z80_MMU_HOOKS		.equ 0



PROGBASE		.equ	0x0000
PROGLOAD		.equ	0x0100

; Mnemonics for I/O ports etc

CONSOLE_RATE		.equ	115200

CPU_CLOCK_KHZ		.equ	10000

; Z80 CTC ports
CTC_CH0		.equ	0x88	; CTC channel 0 and interrupt vector
CTC_CH1		.equ	0x89	; CTC channel 1 (periodic interrupts)
CTC_CH2		.equ	0x8A	; CTC channel 2
CTC_CH3		.equ	0x8B	; CTC channel 3

; 37C65 FDC ports
FDC_CCR		.equ	0x48	; Configuration Control Register (W/O)
FDC_MSR		.equ	0x50	; 8272 Main Status Register (R/O)
FDC_DATA	.equ	0x51	; 8272 Data Port (R/W)
FDC_DOR		.equ	0x58	; Digital Output Register (W/O)
FDC_TC		.equ	0x58	; Pulse terminal count (R/O)

; MMU Ports
MPGSEL_0	.equ	0x78	; Bank_0 page select register (W/O)
MPGSEL_1	.equ	0x79	; Bank_1 page select register (W/O)
MPGSEL_2	.equ	0x7A	; Bank_2 page select register (W/O)
MPGSEL_3	.equ	0x7B	; Bank_3 page select register (W/O)
MPGENA		.equ	0x7C	; memory paging enable register, bit 0 (W/O)
# 1 "../../cpu-z80u/kernel-z80.def"
 
# 26
 
# 44
 
# 10 "buffers.S"
	.export _do_blkzero
	.export _do_blkcopyk
	.export _do_blkcopyul
	.export _do_blkcopyuh

	.export _bsrc
	.export _bdest
	.export _blen

	.export _workbuf

;
;	Ugly - but we need to rework memory management and stuff to fix it
;
_workbuf:
	.ds 1024

_bsrc:
	.word 0
_bdest:
	.word 0
_blen:
	.word 0

_do_blkzero:
	call map_buffers
	ld de,(_bsrc)
	inc de
	ld (hl),#0
	ld bc,#511
	ldir
	jp map_kernel

_do_blkcopyk:
	call map_buffers
	ld hl,(_bsrc)
	ld de,(_bdest)
	ld bc,(_blen)
	ldir
	jp map_kernel

_do_blkcopyul:
	call map_buf_user
	ld hl,(_bsrc)
	ld de,(_bdest)
	ld bc,(_blen)
	ldir
	jp map_kernel

_do_blkcopyuh:
	call map_buf_user_h
	ld hl,(_bsrc)
	ld de,(_bdest)
	ld bc,(_blen)
	ldir
	jp map_kernel
