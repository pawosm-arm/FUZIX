# 0 "mem-rcbus.S"
# 0 "<built-in>"
# 0 "<command-line>"
# 1 "/usr/include/stdc-predef.h" 1 3
# 0 "<command-line>" 2
# 1 "mem-rcbus.S"
;
; memory banking for RC2104
;

 ; exported
 .export size_ram
 .export map_kernel
 .export map_proc
 .export map_proc_always
 .export map_save
 .export map_restore
 .export map_for_swap

 .export cur_map

 .export __ugetc
 .export __ugetw
 .export __uget
 .export __uputc
 .export __uputw
 .export __uput
 .export __uzero

 .export _copy_common

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
# 27 "mem-rcbus.S" 2
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
# 28 "mem-rcbus.S" 2

 .discard

; Sets ramsize, procmem, membanks
size_ram:
 ldd #512
 std _ramsize
 subd #48 ; whatever the kernel occupies (changes when we move
 std _procmem
 rts

 .common

map_proc:
 cmpx #0
 bne map_procu
map_kernel:
 pshs d
 ldd #0x2021
 std cur_map
 std 0xFE78
 incb
 stb cur_map+2
 stb 0xFE7A
 puls d,pc

map_procu:
 pshs d
 ldd ,x++
 std cur_map
 std $FE78
 ldb ,x
 stb cur_map+2
 stb $FE7A
 puls d,pc

map_proc_always:
 pshs d
 ldd _udata + U_DATA__U_PAGE
 std cur_map
 std 0xFE78
 ldb _udata + U_DATA__U_PAGE+2
 stb cur_map+2
 stb 0xFE7A
 puls d,pc

map_save:
 pshs d
 ldd cur_map
 std saved_map
 ldb cur_map+2
 stb saved_map+2
 puls d,pc

map_restore:
 pshs d
 ldd saved_map
 std cur_map
 std 0xFE78
 ldb saved_map+2
 stb cur_map+2
 stb 0xFE7A
 puls d,pc

map_for_swap:
 sta cur_map+1
 sta 0xFE79
 rts

_copy_common:
 ldb 3,s
 ; B holds the page to stuff it in, ints are off, and we will
 ; clean up all the maps in the caller later. Will need
 ; review if we allow interrupts during fork copy
 stb $FE79
 ldx #__common ; Start of common space to copy
 ldy -0x8000,x ; we are mapping a Cxxx page at 4xxx
commoncp:
 ldd ,x++
 std ,y++
 cmpx #$FE00 ; I/O window
 beq skipio
 cmpx #0 ; copy until we did all the vectors
 bne commoncp
 jmp map_kernel
skipio:
 leax 0x100,x
 leay 0x100,y
 bra commoncp

__ugetc:
 ldx 2,s
 jsr map_proc_always
 ldb ,x
 clra
 jmp map_kernel
__ugetw:
 ldx 2,s
 jsr map_proc_always
 ldd ,x
 jmp map_kernel
__uget:
 pshs u
 ldx 4,s ; user
 ldu 6,s ; dst
 ldy 8,s ; size
ugetl:
 jsr map_proc_always
 lda ,x+
 jsr map_kernel
 sta ,u+
 leay -1,y
 bne ugetl
 ldd #0
 puls u,pc

__uputc:
 ldx 2,s
 ldb 5,s
 jsr map_proc_always
 stb ,x
 jsr map_kernel
 ldd #0
 rts
__uputw:
 ldx 2,s
 ldd 4,s
 jsr map_proc_always
 std ,x
 jsr map_kernel
 ldd #0
 rts
;
; Might be worth doing word sized loops for uput/uget ?
;
__uput:
 pshs u
 ldx 4,s ; src
 ldu 6,s ; dst
 ldy 8,s ; size
uputl:
 jsr map_kernel
 lda ,x+
 jsr map_proc_always
 sta ,u+
 leay -1,y
 bne uputl
 jsr map_kernel
 ldx #0
 puls u,pc

__uzero:
 ldx 2,s
 ldy 4,s
 jsr map_proc_always
 clra
 lsrb
 bcc evenc
 sta ,x+
 leay -1,y
 beq zdone
evenc:
 clrb
uzloop:
 std ,x++
 leay -2,y
 bne uzloop
zdone:
 jsr map_kernel
 ldd #0
 rts

 .commondata

cur_map:
 .word 0
 .byte 0
saved_map:
 .word 0
 .byte 0
