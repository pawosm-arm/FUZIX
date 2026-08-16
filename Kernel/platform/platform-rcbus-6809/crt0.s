# 0 "crt0.S"
# 0 "<built-in>"
# 0 "<command-line>"
# 1 "/usr/include/stdc-predef.h" 1 3
# 0 "<command-line>" 2
# 1 "crt0.S"
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
# 2 "crt0.S" 2

  .code

  ; On entry 0000-BFFF are in situ. Cxxx holds the common for
  ; Fxxx

  .word 0x6809
start:
  orcc #0x10 ; interrupts definitely off
  lds #kstack_top ; note we'll wipe the stack later

  ldx #$C000
  ldy #$F000
copy1:
  ldd ,x++
  std ,y++
  cmpy #0000
  beq move_done
  ; Don't copy into the I/O window
  cmpy #$FE00
  bne copy1
  leax $100,x
  leay $100,y
  bra copy1
move_done: jmp premain

  .discard

premain:
  clra
  ldx #_udata
udata_wipe: sta ,x+
  cmpx #_udata+U_DATA__TOTALSIZE
  blo udata_wipe
  ldx #__bss
  ldy #__bss_size
bss_wipe: sta ,x+
  leay -1,y
  bne bss_wipe
  jsr init_early
  jsr init_hardware
  jmp main

  .code

main: jsr _fuzix_main
  orcc #0x10 ; we should never get here
stop: bra stop
