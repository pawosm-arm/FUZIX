# 1 "../../dev/devrd_z180_hw.S"
; 2017-01-04 William R Sowerbutts

        .z180

        ; exported symbols
        .export _rd_plt_copy
        .export _rd_cpy_count
        .export _rd_reverse
        .export _rd_dst_userspace
        .export _rd_dst_address
        .export _rd_src_address
        .export _devmem_read
        .export _devmem_write
# 1 "../../dev/../build/kernelu.def"
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
# 1 "../../dev/../cpu-z80u/kernel-z80.def"
 
# 26
 
# 44
 
# 1 "../../dev/../cpu-z180/z180.def"
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
# 19 "../../dev/devrd_z180_hw.S"
; FIXME: should be in z180.def or similar

OS_RAM1	.equ	FIRST_RAM_BANK + 0           

	.code

_devmem_write:
        ld a, 1
        ld (_rd_reverse), a             ; 1 = write
        jr _devmem_go

_devmem_read:
        xor a
        ld (_rd_reverse), a             ; 0 = read
        inc a
_devmem_go:
        ld (_rd_dst_userspace), a       ; 1 = userspace
        ; load the other parameters
        ld hl, (_udata + 98    )
        ld (_rd_dst_address), hl
        ld hl, (_udata + 100   )
        ld (_rd_cpy_count), hl
        ld hl, (_udata + 102   )
        ld (_rd_src_address), hl
        ld hl, (_udata + 102   +2)
        ld (_rd_src_address+2), hl
        ; FALL THROUGH INTO _rd_plt_copy

;=========================================================================
; _rd_page_copy - Copy data from one physical page to another
; See notes in devrd.h for input parameters
; This code is Z180 specific so can safely use ld a,i
;=========================================================================
_rd_plt_copy:
        ; save interrupt flag on stack then disable interrupts
        ld a, i
        push af
        di

        ; load source page number
        ld de, (_rd_src_address+1) ; and +2
        ld a,  (_rd_src_address+0)
        ld b, a

        ; compute destination
        ld a,(_rd_dst_userspace)        ; are we loading into userspace memory?
        or a
        jr nz, rd_translate_userspace
        ld hl, OS_RAM1/16
        jr rd_done_translate
rd_translate_userspace:
        ld hl,(_udata + 2     )          ; load page number
        add hl, hl                      ; shift left 4 bits
        add hl, hl
        add hl, hl
        add hl, hl
rd_done_translate:
        ; add in page offset
        ld a,(_rd_dst_address+1)        ; top 8 bits of address
        add a, l
        ld l, a
        adc a, h
        sub l
        ld h, a                         ; result in hl
        ld a,(_rd_dst_address+0)
        ld c, a

        ld a,(_rd_reverse)
        or a
        jr z,not_reversed

        ex de, hl
        out0 (DMA_SAR0L),c
        out0 (DMA_DAR0L),b
        jr topbits
not_reversed:
        out0 (DMA_SAR0L),b
        out0 (DMA_DAR0L),c
topbits:
        out0 (DMA_SAR0B),d
        out0 (DMA_SAR0H),e
        out0 (DMA_DAR0B),h
        out0 (DMA_DAR0H),l

        ld hl,(_rd_cpy_count)
        out0 (DMA_BCR0L),l
        out0 (DMA_BCR0H),h

        ; make dma go
        ld bc, 0x0240
        out0 (DMA_DMODE), b     ; 0x02 - memory to memory, burst mode
        out0 (DMA_DSTAT), c     ; 0x40 - enable DMA channel 0
        ; CPU stalls until DMA burst completes

        ; recover interrupt flag from stack and restore ints if required
        pop af
        ret po
        ei
        ret                     ; return with HL=_rd_cpy_count, as required by char device drivers

; variables
_rd_cpy_count:
        .word     0	; uint16_t
_rd_reverse:
        .byte     0	; bool
_rd_dst_userspace:
        .byte     0	; bool
_rd_dst_address:
        .word     0	; uint16_t
_rd_src_address:
        .byte     0	; uint32_t
        .byte     0
        .byte     0
        .byte     0
;=========================================================================
