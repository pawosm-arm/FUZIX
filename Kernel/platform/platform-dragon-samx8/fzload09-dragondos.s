; DragonDOS core routines.
;
; SPDX-License-Identifier: CC0-1.0
;
; By Ciaran Anscomb, 2025-2026.  This file is dedicated to the public
; domain (Creative Commons "CC0 1.0 Universal"), but do note that this
; dedication may not apply to accompanying files.

; Assumes single-sided 18 * 256 sectors per track.

; Provides:

; devopen
;
;   Call this first.  Configures nmi handler, resets the controller, sets
;   up for FDC operations then restores to track 0.  X register must
;   contain the LSN of the next sector to read.
;
; devclose
;
;  Call this when done.
;
; devread
;
;   Seek to next track if necessary, then read next sector.

; Uses:

; set_nmi_handler
;
;   Point NMI vector to X.

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

; DragonDOS registers

; FDC interface
reg_fdc_status		equ $ff40
reg_fdc_command		equ $ff40
reg_fdc_track		equ $ff41
reg_fdc_sector		equ $ff42
reg_fdc_data		equ $ff43

; DragonDOS control
reg_ddos_control	equ $ff48

; FDC commands
fdc_c_restore		equ $00
fdc_c_seek		equ $10
fdc_c_read_sector	equ $88
fdc_c_force_interrupt	equ $d0

; FDC seek options
fdc_c_verify_track	equ $04
fdc_c_step_6ms		equ $00
fdc_c_step_12ms		equ $01
fdc_c_step_20ms		equ $02
fdc_c_step_30ms		equ $03

; FDC force interrupt options
fdc_c_fi_no_intrq	equ $00

; FDC status bits
fdc_s_not_ready		equ $80
fdc_s_write_protect	equ $40
fdc_s_hld		equ $20
fdc_s_seek_error	equ $10
fdc_s_crc_error		equ $08
fdc_s_track0		equ $04
fdc_s_index_pulse	equ $02
fdc_s_busy		equ $01
fdc_s_rnf		equ $10
fdc_s_lost_data		equ $04
fdc_s_drq		equ $02
fdc_s_record_type	equ $20

; DragonDOS control bits
ddos_c_nmi_enable	equ $20
ddos_c_precomp		equ $10
ddos_c_density		equ $08
ddos_c_motor_on		equ $04
ddos_c_drive_mask	equ $03
ddos_c_single_density	equ $08
ddos_c_double_density	equ $00

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

; Open device.
;
; entry: X = last sector LSN
; exit: A = status from restore command
; destroyed: b,x
devopen	pshs a
	; Naive LSN to track/sector conversion.  Assumes 18 sectors per
	; track, single-sided.
	clra
@l10	cmpx #18
	blo @l20
	inca
	leax -18,x
	bra @l10
@l20	sta ddos_track,pcr
	tfr x,d
	incb
	stb ddos_sector,pcr

	; Configure dummy NMI handler
	leax ddos_nmi_handler,pcr
	bsr set_nmi_handler

	; Base value for the DragonDOS control register
	clr ddos_control,pcr

	; DragonDOS reset controller
	lda #fdc_c_force_interrupt|fdc_c_fi_no_intrq
	sta reg_fdc_command
	lda #10
@l0	deca
	bne @l0			; approx 56us delay
	lda reg_fdc_status

	; Setup for FDC operation
	ldb ddos_control,pcr
	orb #ddos_c_motor_on
	stb reg_ddos_control
	bsr ddos_restore
	puls a,pc

; Close device.

devclose
	; Tidy up after FDC operation
	pshs a
	lda ddos_control,pcr
	sta reg_ddos_control
	puls a,pc

; Read next sector from device.

; entry: x = destination
; exit: x = next byte in destination
devread
	pshs a,b
	lda ddos_track,pcr
	bsr ddos_seek
	ldd ddos_track,pcr	; A=track, B=sector
	pshs b
	incb			; next sector
	cmpb #18
	bls @l10
	inca			; next track
	ldb #1
@l10	std ddos_track,pcr
	puls a			; A=sector
	bsr ddos_read_sector
	puls a,b,pc

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

ddos_restore
	clra			; fdc_c_restore
	bra ddos_type1

; entry: A = desired track
; exit: A = status register
; Turns on write precompensation above track 16
ddos_seek
	sta reg_fdc_data
	cmpa #$10
	bls @l10
	lda ddos_control,pcr
	ora #ddos_c_precomp|ddos_c_motor_on
	sta reg_ddos_control
@l10	lda #fdc_c_seek|fdc_c_verify_track
	; fall through

ddos_type1
	sta reg_fdc_command	; issue FDC command
@l0	lda reg_fdc_status
	bita #fdc_s_busy	; still BUSY?
	bne @l0			; if so keep polling
	rts

; entry: A = desired sector, X = destination
; exit: A = status register, X = next byte in destination
ddos_read_sector
	sta reg_fdc_sector
	lda #fdc_c_read_sector
	sta reg_fdc_command	; issue command
@l10	lda reg_fdc_status	; read status
	bita #fdc_s_drq		; DRQ set?
	beq @l20		; if not, test BUSY
	lda reg_fdc_data	; byte ready: read data
	sta ,x+			; store data in memory
	bra @l10
@l20	bita #fdc_s_busy	; BUSY set?
	bne @l10		; if so, keep polling
	rts

; Dummy NMI handler.
ddos_nmi_handler
	rti

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

; Work variables.

ddos_control
	fcb 0
ddos_track
	fcb 0
ddos_sector
	fcb 0
