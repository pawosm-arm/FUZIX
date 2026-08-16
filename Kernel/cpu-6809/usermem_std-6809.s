# 0 "cpu-6809/usermem_std-6809.S"
# 0 "<built-in>"
# 0 "<command-line>"
# 1 "/usr/include/stdc-predef.h" 1 3
# 0 "<command-line>" 2
# 1 "cpu-6809/usermem_std-6809.S"
# 1 "cpu-6809/../build/kernel.def" 1
; FUZIX mnemonics for memory addresses etc

U_DATA equ 0xFB00 ; (this is struct u_data from kernel.h)
U_DATA__TOTALSIZE equ 0x200 ; 256+256 (we don't save istack)

U_DATA_STASH equ 0xBE00 ; BE00-BFFF

IDEDATA equ 0xFE10

PROGBASE equ 0x0100 ; programs and data start here

NBUFS equ 5

; This assumes a 1.8432MHz E clock to get 10Hz system timer.
CLKVAL equ ((184320 / 8) - 1)
# 2 "cpu-6809/usermem_std-6809.S" 2
# 1 "cpu-6809/kernel09.def" 1
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
# 3 "cpu-6809/usermem_std-6809.S" 2

;
; Using the C helpers for now
;
