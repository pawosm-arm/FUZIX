# 1 "crt0.S"
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
# 3 "crt0.S"
        ; startup code
	.code
init:
	di

	; setup the memory paging for kernel
        ld a, 33
        out (MPGSEL_1), a       ; map page 33 at 0x4000
        inc a
        out (MPGSEL_2), a       ; map page 34 at 0x8000
	inc a
        out (MPGSEL_3), a       ; map page 35 at 0xC000

mappedok:
        ; switch to stack in high memory
        ld sp, kstack_top

        ; Zero the data area
        ld hl, __bss
        ld de, __bss + 1
        ld bc, __bss_size - 1
        ld (hl), 0
        ldir

        ; Hardware setup
        call init_hardware

        ; Call the C main routine
        call _fuzix_main
    
        ; fuzix_main() shouldn't return, but if it does...
        di
stop:   halt
        jr stop

	; This gets overwriten by a serial buffer
	.abs
	.org 0x100

	jp init
