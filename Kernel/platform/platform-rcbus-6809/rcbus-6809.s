# 0 "rcbus-6809.S"
# 0 "<built-in>"
# 0 "<command-line>"
# 1 "/usr/include/stdc-predef.h" 1 3
# 0 "<command-line>" 2
# 1 "rcbus-6809.S"
 ;
 ; common RC2014 6809 code
 ;

        ; exported symbols
        .export init_early
        .export init_hardware
        .export _program_vectors
 .export _need_resched

 ; exported
 .export _bufpool

 ; exported debugging tools
 .export _plt_monitor
 .export _plt_reboot
 .export outchar
 .export ___hard_di
 .export ___hard_ei
 .export ___hard_irqrestore

# 1 "kernel.def" 1
; FUZIX mnemonics for memory addresses etc

U_DATA equ 0xFB00 ; (this is struct u_data from kernel.h)
U_DATA__TOTALSIZE equ 0x200 ; 256+256 (we don't save istack)

U_DATA_STASH equ 0xBE00 ; BE00-BFFF

IDEDATA equ 0xFE10

PROGBASE equ 0x0100 ; programs and data start here

NBUFS equ 5

; This assumes a 1.8432MHz E clock to get 10Hz system timer.
CLKVAL equ ((184320 / 8) - 1)
# 23 "rcbus-6809.S" 2
# 1 "../../cpu-6809/kernel09.def" 1
; Keep these in sync with struct u_data!!
U_DATA__U_PTAB equ 0 ; struct p_tab*
U_DATA__U_PAGE equ 2 ; uint16_t
U_DATA__U_PAGE2 equ 4 ; uint16_t
U_DATA__U_INSYS equ 6 ; bool
U_DATA__U_CALLNO equ 7 ; uint8_t
U_DATA__U_SYSCALL_SP equ 8 ; void *
U_DATA__U_RETVAL equ 10 ; int16_t
U_DATA__U_ERROR equ 12 ; int16_t
U_DATA__U_SP equ 14 ; void *
U_DATA__U_ININTERRUPT equ 16 ; bool
U_DATA__U_CURSIG equ 17 ; int8_t
U_DATA__U_ARGN equ 18 ; uint16_t
U_DATA__U_ARGN1 equ 20 ; uint16_t
U_DATA__U_ARGN2 equ 22 ; uint16_t
U_DATA__U_ARGN3 equ 24 ; uint16_t
U_DATA__U_ISP equ 26 ; void * (initial stack pointer when _exec()ing)
U_DATA__U_TOP equ 28 ; uint16_t
U_DATA__U_BREAK equ 30 ; uint16_t
U_DATA__U_CODEBASE equ 32 ; uint16_t
U_DATA__U_SIGVEC equ 34 ; table of function pointers (void *)

; Keep these in sync with struct p_tab!!
P_TAB__P_STATUS_OFFSET equ 0
P_TAB__P_FLAGS_OFFSET equ 1
P_TAB__P_TTY_OFFSET equ 2
P_TAB__P_PID_OFFSET equ 3
P_TAB__P_PAGE_OFFSET equ 15

P_RUNNING equ 1 ; value from include/kernel.h
P_READY equ 2 ; value from include/kernel.h

PFL_BATCH equ 4 ; value from include/kernel.h

OS_BANK equ 0 ; value from include/kernel.h

EAGAIN equ 11 ; value from include/kernel.h


; Keep in sync with struct blkbuf
BUFSIZE equ 520
# 24 "rcbus-6809.S" 2

 .common

vectors:
 .word 0 ; reserved
 .word badswi_handler ; SWI3
 .word badswi_handler ; SWI2
 .word firq_handler ; FIR
 .word interrupt_handler ; IRQ
 .word unix_syscall_entry ; SWI
 .word nmi_handler ; NMI
 .word 0 ; RESET (never used)

 .buffers
 ;
 ; We use the linker to place these just below
 ; the discard area
 ;
_bufpool:
 .ds BUFSIZE*NBUFS

 ; And expose the discard buffer count to C - but in discard
 ; so we can blow it away and precomputed at link time
 ;
 ; TODO: this wants to change to blow away anything below
 ; common.
 ;
 .discard

init_early:
 rts

init_hardware:
 jsr size_ram
 ldx #$FE60 ; 6840 PTM at I/O 0x60
 lda #1
 sta 1,x ; Enable CR1, set CR2 to be
    ; square wave no IRQ, refclk, 16bit
 lda #0x01
 sta ,x ; Reset the chip, CR1 config doesn't matter
    ; providing output is disabled
 ldd #CLKVAL
 std 6,x ; Timer 3
 clr 1,x ; CR2 square, no IRQ, no out, no input,
    ; CR3 accessible
 lda #0x43 ; counter mode, count E clocks, prescale
    ; IRQ on
 ldx #vectors
 ldu #0xFFF0
 ldd ,x++
 std ,u++
 ldd ,x++
 std ,u++
 ldd ,x++
 std ,u++
 ldd ,x++
 std ,u++
 ldd ,x++
 std ,u++
 ldd ,x++
 std ,u++
 ldd ,x++
 std ,u++
 ldd ,x
 std ,u
 sta ,x

 lda #0x01
 sta 1,x ; Back to CR1
 clr ,x ; out of reset
 jmp set_vector

        .common

_plt_reboot:
 ; TODO
_plt_monitor:
 orcc #0x10
 bra _plt_monitor

___hard_di:
 tfr cc,b ; return the old irq state
 orcc #0x10
 rts
___hard_ei:
 andcc #0xef
 rts

___hard_irqrestore: ; B holds the data
 ldb 3,s
 tfr b,cc
 rts


        .common

;
; Our vectors are in a single fixed bank so no work is needed. Will
; change if we move to properly using the paging.
;
_program_vectors:
 ldx 2,s
 lda ,x
 sta 0xFE78 ; map low page
 jsr set_vector
 lda #0x20
 sta 0xFE78
 rts

set_vector:
 lda #0x7E
 sta 0
 ldd #null_handler
 sta 1
 rts

;
; Nothing to do here - we don't use SWI or FIRQ
;
firq_handler:
badswi_handler:
 rti

;
; debug via serial console port 1
;
outchar:
 ldb $FEC5
 andb #0x20
 beq outchar
 sta $FEC0
 rts

 .commondata

_need_resched:
 .byte 0
