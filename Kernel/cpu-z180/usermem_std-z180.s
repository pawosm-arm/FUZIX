# 1 "cpu-z180/usermem_std-z180.S"
;
;	For the Z180 we use the DMA engine for user copies except for the
;	byte/word requests. At the moment we don't re-route any small
;	requests to the uget/uput API if they are below the DMA break even
;	point or just small.
;
;	The DMA engine stalls the CPU so the fact we di/ei around this does
;	not actually make much real difference. If DMA latency is a problem
;	then it needs doing in blocks here and in z180.s for fork() which is
;	by far the longer case.
;
;	TODO: consider BANKED support
;
	.z180
# 1 "cpu-z180/../build/kernelu.def"
; UZI mnemonics for memory addresses etc

; Move down to 0xF600 to fit the monitor in
U_DATA__TOTALSIZE           .equ 0x200        ; 256+256 bytes @ F800
Z80_TYPE                    .equ 2

OS_BANK                     .equ 0x00         ; value from include/kernel.h

; N8VEM Mark IV mnemonics
FIRST_RAM_BANK              .equ 0x80         ; low 512K of physical memory is ROM/ECB window.
Z180_IO_BASE                .equ 0x40
MARK4_IO_BASE               .equ 0x80

; No standard clock speed for the Mark IV board, but this is a common choice.
; disabling this saves around approx 0.5KB

CPU_CLOCK_KHZ               .equ 36864        ; 18.432MHz * 2
Z180_TIMER_SCALE            .equ 20           ; CPU clocks per timer tick
TICKSPERSEC                 .equ 40           ; timer interrupt rate (Hz)

PROGBASE		    .equ 0x0000
PROGLOAD		    .equ 0x0100
# 27
PPIDE_RD_LINE	.equ	0x40
PPIDE_WR_LINE	.equ	0x20
PPIDE_PPI_BUS_READ	.equ	0x92
PPIDE_PPI_BUS_WRITE	.equ	0x80

ppi_port_a	.equ	0x88
ppi_port_b	.equ	0x89
ppi_port_c	.equ	0x8A
ppi_control	.equ	0x8B

ppide_data	.equ	0x08
# 1 "cpu-z180/../cpu-z180/z180.def"
; ASCI serial ports
ASCI_CNTLA0                 .equ Z180_IO_BASE+0x00     ; ASCI control register A channel 0
ASCI_CNTLA1                 .equ Z180_IO_BASE+0x01     ; ASCI control register A channel 1
ASCI_CNTLB0                 .equ Z180_IO_BASE+0x02     ; ASCI control register B channel 0
ASCI_CNTLB1                 .equ Z180_IO_BASE+0x03     ; ASCI control register B channel 0
ASCI_STAT0                  .equ Z180_IO_BASE+0x04     ; ASCI status register    channel 0
ASCI_STAT1                  .equ Z180_IO_BASE+0x05     ; ASCI status register    channel 1
ASCI_TDR0                   .equ Z180_IO_BASE+0x06     ; ASCI transmit data reg, channel 0
ASCI_TDR1                   .equ Z180_IO_BASE+0x07     ; ASCI transmit data reg, channel 1
ASCI_RDR0                   .equ Z180_IO_BASE+0x08     ; ASCI receive data reg,  channel 0
ASCI_RDR1                   .equ Z180_IO_BASE+0x09     ; ASCI receive data reg,  channel 0
ASCI_ASEXT0                 .equ Z180_IO_BASE+0x12     ; ASCI extension register channel 0
ASCI_ASEXT1                 .equ Z180_IO_BASE+0x13     ; ASCI extension register channel 1
ASCI_ASTC0L                 .equ Z180_IO_BASE+0x1A     ; ASCI time constant register channel 0 low
ASCI_ASTC0H                 .equ Z180_IO_BASE+0x1B     ; ASCI time constant register channel 0 high
ASCI_ASTC1L                 .equ Z180_IO_BASE+0x1C     ; ASCI time constant register channel 1 low
ASCI_ASTC1H                 .equ Z180_IO_BASE+0x1D     ; ASCI time constant register channel 1 high

; Z180 MMU
MMU_CBR                     .equ Z180_IO_BASE+0x38     ; common1 base register
MMU_BBR                     .equ Z180_IO_BASE+0x39     ; bank base register
MMU_CBAR                    .equ Z180_IO_BASE+0x3A     ; common/bank area register

; Z180 DMA engine
DMA_SAR0L                   .equ Z180_IO_BASE+0x20     ; DMA source address reg, channel 0L
DMA_SAR0H                   .equ Z180_IO_BASE+0x21     ; DMA source address reg, channel 0H
DMA_SAR0B                   .equ Z180_IO_BASE+0x22     ; DMA source address reg, channel 0B
DMA_DAR0L                   .equ Z180_IO_BASE+0x23     ; DMA dest address reg,   channel 0L
DMA_DAR0H                   .equ Z180_IO_BASE+0x24     ; DMA dest address reg,   channel 0H
DMA_DAR0B                   .equ Z180_IO_BASE+0x25     ; DMA dest address reg,   channel 0B
DMA_BCR0L                   .equ Z180_IO_BASE+0x26     ; DMA byte count reg,     channel 0L
DMA_BCR0H                   .equ Z180_IO_BASE+0x27     ; DMA byte count reg,     channel 0H
DMA_MAR1L                   .equ Z180_IO_BASE+0x28     ; DMA memory address reg, channel 1L
DMA_MAR1H                   .equ Z180_IO_BASE+0x29     ; DMA memory address reg, channel 1H
DMA_MAR1B                   .equ Z180_IO_BASE+0x2A     ; DMA memory address reg, channel 1B
DMA_IAR1L                   .equ Z180_IO_BASE+0x2B     ; DMA I/O address reg,    channel 1L
DMA_IAR1H                   .equ Z180_IO_BASE+0x2C     ; DMA I/O address reg,    channel 1H
DMA_BCR1L                   .equ Z180_IO_BASE+0x2E     ; DMA byte count reg,     channel 1L
DMA_BCR1H                   .equ Z180_IO_BASE+0x2F     ; DMA byte count reg,     channel 1H
DMA_DSTAT                   .equ Z180_IO_BASE+0x30     ; DMA status register
DMA_DMODE                   .equ Z180_IO_BASE+0x31     ; DMA mode register
DMA_DCNTL                   .equ Z180_IO_BASE+0x32     ; DMA/WAIT control register

; Z180 Timer
TIME_TMDR0L                 .equ Z180_IO_BASE+0x0C     ; Timer data register,    channel 0L
TIME_TMDR0H                 .equ Z180_IO_BASE+0x0D     ; Timer data register,    channel 0H
TIME_RLDR0L                 .equ Z180_IO_BASE+0x0E     ; Timer reload register,  channel 0L
TIME_RLDR0H                 .equ Z180_IO_BASE+0x0F     ; Timer reload register,  channel 0H
TIME_TCR                    .equ Z180_IO_BASE+0x10     ; Timer control register
TIME_TMDR1L                 .equ Z180_IO_BASE+0x14     ; Timer data register,    channel 1L
TIME_TMDR1H                 .equ Z180_IO_BASE+0x15     ; Timer data register,    channel 1H
TIME_RLDR1L                 .equ Z180_IO_BASE+0x16     ; Timer reload register,  channel 1L
TIME_RLDR1H                 .equ Z180_IO_BASE+0x17     ; Timer reload register,  channel 1H
TIME_FRC                    .equ Z180_IO_BASE+0x18     ; Timer Free running counter

; Z180 Interrupts
INT_IL                      .equ Z180_IO_BASE+0x33     ; Interrupt vector low register
INT_ITC                     .equ Z180_IO_BASE+0x34     ; Interrupt vector low register

; Refresh control
MEM_RCR			    .equ Z180_IO_BASE+0x36	; Refresh control

; ESCC serial ports (Z80182)
ESCC_CTRL_A                 .equ 0xE0                   ; ESCC Channel A control register
ESCC_DATA_A                 .equ 0xE1                   ; ESCC Channel A data register
ESCC_CTRL_B                 .equ 0xE2                   ; ESCC Channel B control register
ESCC_DATA_B                 .equ 0xE3                   ; ESCC Channel B data register

PORT_A_DDR                  .equ 0xED                   ; Port A data direction register
PORT_A_DATA                 .equ 0xEE                   ; Port A data register
PORT_B_DDR                  .equ 0xE4                   ; Port B data direction register
PORT_B_DATA                 .equ 0xE5                   ; Port B data register
PORT_C_DDR                  .equ 0xDD                   ; Port C data direction register
PORT_C_DATA                 .equ 0xDE                   ; Port C data register

Z182_SYSCONFIG              .equ 0xEF                   ; System Configuration Register
Z182_RAMUBR                 .equ 0xE6                   ; RAM upper boundary register
Z182_RAMLBR                 .equ 0xE7                   ; RAM lower boundary register
Z182_ROMBR                  .equ 0xE8                   ; ROM boundary register

; Debugging
DEBUGBANK   .equ 0
DEBUGCOMMON .equ 0
# 1 "cpu-z180/../cpu-z80u/kernel-z80.def"
 
# 26
 
# 44
 
# 20 "cpu-z180/usermem_std-z180.S"
        ; exported symbols
        .export __uget
        .export __ugetc
        .export __ugetw

        .export __uput
        .export __uputc
        .export __uputw
        .export __uzero

	.code

OS_RAM1	.equ	FIRST_RAM_BANK + 0           
;
;	Compute the DMA pages to use. This isn't quite as trivial as it
;	looks because the kernel common flips per process
;
;
;	Copy BC bytes from HL to DE using the DMA engine
;
dma_to_kernel:
	ld	a,0x02		; memory inc to memory inc
	out0	(DMA_DMODE),a
	push	bc
	ld	a,(_udata + 2     )	; Bank code >> 4 is the upper 4 bits
	rra
	rra
	rra
	rra
	ld	c,a		; save src bank
	ld	b,a		; possible dst bank if common
	ld	a,d
	cp	>__common	; deal with common oddities
	jr	nc, dma_op
	ld	b, OS_RAM1 / 16
	jr	dma_op

dma_from_kernel:
	ld	a,0x02		; memory inc to memory inc
dma_user_fixed:
	out0	(DMA_DMODE),a	; burst mem to mem
	push bc
	ld	a,(_udata + 2     )	; get our bank
	rra
	rra
	rra
	rra
	ld	c,a		; save src bank if common
	ld	b,a		; save dst bank
	ld	a,h
	cp	>__common
	jr	nc, dma_op
	ld	c, OS_RAM1 / 16	; not common
dma_op:
	; load banks
	out0	(DMA_SAR0B),c
	out0	(DMA_DAR0B),b
	; recover length
	pop	bc
	; load DMA engine addresses and length
	out0	(DMA_BCR0H),b
	out0	(DMA_BCR0L),c
	out0	(DMA_DAR0H),d
	out0	(DMA_DAR0L),e
	out0	(DMA_SAR0H),h
	out0	(DMA_SAR0L),l
	ld	a,0x40		; burst transfer
	out0	(DMA_DSTAT),a
	; DMA stalls the ret until done
	ret


uputget:
	ld	ix, 10
	add	ix, sp
        ; load DE with the byte count
        ld	c, (ix + 4) ; byte count
        ld	b, (ix + 5)
        ; load HL with the source address
        ld	l, (ix + 0) ; src address
        ld	h, (ix + 1)
        ; load DE with destination address (in userspace)
        ld	e, (ix + 2)
        ld	d, (ix + 3)
	; check if zero byte operation
	ld	a, b
	or	c
	di
	ret

__uput:
	push	ix
	ld	a,i
	push	af
	push	bc
	call	uputget		; source in HL dest in DE, count in BC
	; Z means nothing to copy
	call	nz, dma_from_kernel
upop:
	pop	bc
	pop	af
	pop	ix
	ld	hl, 0
	ret	po
	ei
	ret

__uget:
	push	ix
	ld	a,i
	push	af
	push	bc
	call	uputget		; source in HL dest in DE, count in BC
	call	nz, dma_to_kernel
	jr	upop

;
;	Write first byte to 0 then DMA it over
;
# 167
	.common
;
;	We don't use the DMA for this at the moment. Some debug is needed
;	there. It's also not clear it is a speed win anyway.
;
__uzero:
	push	bc
	ld	hl,4
	add	hl,sp
	ld	e,(hl)
	inc	hl
	ld	d,(hl)
	inc	hl
	ld	c,(hl)
	inc	hl
	ld	b,(hl)
	ex	de,hl
	ld	a, b	; check for 0 copy
	or	c
	jr	z, pop_out
	call	map_proc_always
	ld	(hl), 0
	dec	bc
	ld	a, b
	or	c
	jr	z, pop_out
	ld	e, l
	ld	d, h
	inc	de
	ldir
pop_out:
	pop	bc
	jp	map_kernel_restore



;
;	We need these in common as they bank switch
;
	.common
__uputc:
	push	bc
	ld	hl,4
	add	hl,sp
	ld	e,(hl)
	inc	hl
	inc	hl
	ld	a,(hl)
	inc	hl
	ld	h,(hl)
	ld	l,a
	call	map_proc_always
	ld	(hl), e
uputc_out:
	pop	bc
	jp	map_kernel_restore	; map the kernel back below common

__uputw:
	push	bc
	ld	hl,4
	add	hl,sp
	ld	e,(hl)
	inc	hl
	ld	d,(hl)
	inc	hl
	ld	a,(hl)
	inc	hl
	ld	h,(hl)
	ld	l,a
	call	map_proc_always
	ld	(hl), e
	inc	hl
	ld	(hl), d
	jr	uputc_out

__ugetc:
	pop	de
	pop	hl
	push	hl
	push	de
	call	map_proc_always
        ld	l, (hl)
	ld	h, 0
	jp	map_kernel_restore

__ugetw:
	pop	de
	pop	hl
	push	hl
	push	de
	call	map_proc_always
        ld	a, (hl)
	inc	hl
	ld	h, (hl)
	ld	l, a
	jp	map_kernel_restore
