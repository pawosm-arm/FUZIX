; CoCoSDC core routines.
;
; SPDX-License-Identifier: CC0-1.0
;
; By Ciaran Anscomb, 2025-2026.  This file is dedicated to the public
; domain (Creative Commons "CC0 1.0 Universal"), but do note that this
; dedication may not apply to accompanying files.

; Define COCO=1 before including to use CoCo RSDOS register layout.  Else
; will assume DragonDOS layout.

; Provides:

; devopen
;
;   Call this first.  X register must contain the LSN of the next sector
;   to read.
;
; devclose
;
;  Call this when done.
;
; devread
;
;   Read next sector.

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

; CoCoSDC registers

	ifndef COCO

; Register definitions for CoCoSDC in DragonDOS mode.

CTRLATCH	equ $ff48	; controller latch (write)
CMDREG		equ $ff40	; command register (write)
STATREG		equ $ff40	; status register (read)
PREG1		equ $ff41	; param register 1
PREG2		equ $ff42	; param register 2
PREG3		equ $ff43	; param register 3
DATREGA		equ PREG2	; first data register
DATREGB		equ PREG3	; second data register

CMDMODE		equ $0b

	else

; Register definitions for CoCoSDC in RSDOS mode.

CTRLATCH	equ $ff40	; controller latch (write)
CMDREG		equ $ff48	; command register (write)
STATREG		equ $ff48	; status register (read)
PREG1		equ $ff49	; param register 1
PREG2		equ $ff4a	; param register 2
PREG3		equ $ff4b	; param register 3
DATREGA		equ PREG2	; first data register
DATREGB		equ PREG3	; second data register

CMDMODE		equ $43

	endif

; Status register masks
BUSY		equ $01
READY		equ $02
FAILED		equ $80

; Command values
CMDREAD		equ $80
CMDWRITE	equ $a0
CMDEX		equ $c0
CMDEXD		equ $e0

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

devopen
	stx cocosdc_lsn,pcr
	; fall through
devclose
	clr CTRLATCH
	rts

devread
	pshs a,b,u
	pshs x
	ldd #(CMDMODE*256)|CMDREAD
	sta CTRLATCH		; enable cocosdc enhanced mode
	exg a,a			; delay
	exg a,a
	bsr cocosdc_while_busy
	clr PREG1		; 24-bit LSN (only use 16 bits)
	ldx cocosdc_lsn,pcr
	stx PREG2		; X is still LSN from above
	leax 1,x
	stx cocosdc_lsn,pcr
	stb CMDREG		; enhanced read sector
	exg a,a			; delay
	exg a,a
	puls x
@l10	lda STATREG		; wait for READY flag
	bita #READY
	beq @l10
	lda #$80		; 128 2-byte fetches
@l20	ldu PREG2
	stu ,x++
	deca
	bne @l20
	bsr cocosdc_while_busy
	clr CMDREG
	puls a,b,u,pc

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

; Helpers

; Wait while CoCoSDC is BUSY.  On timeout, report error.
cocosdc_while_busy
	pshs x
	ldx #$e000		; timeout delay
@l10	leax -1,x
	beq @l20
	lda STATREG
	lsra
	bcs @l10
	puls x,pc
@l20	leas 2,s
	leax msg_busy,pcr
	lbra error

msg_busy
	fcb 10
	fcc /BUSY TIMEOUT/
	fcb 0

cocosdc_lsn
	fdb 0
