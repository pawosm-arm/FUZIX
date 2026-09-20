	.export _dw_operation
	.export _dw_transaction
	.export _dw_reset

	.export map_proc_a
	.export map_proc_always
	.export map_kernel

	.common

; FIXME conversions debug, regsaves etc
;
;	Implement dw_transaction for the TRS80 port
;
_dw_transaction:
	push ix
	ld ix,#0
	add ix,sp
	ld a,(ix + 10)
	or a
	push af
	call nz, map_proc_always
	ld h,(ix + 3)
	ld l,(ix + 2)
	ld b,(ix + 5)
	ld c,(ix + 4)
	ld de, txbyte
	call dw_op
	ld h,(ix + 7)
	ld l,(ix + 6)
	ld b,(ix + 9)
	ld c,(ix + 8)
	ld de, rxbyte
	call dw_op
	ld hl, #0
	jr nc, read_good
	; We don't distinguish framing and short frames FIXME
	dec hl
read_good:
	pop af
	call nz, map_kernel
	pop ix
	ret

	; dw_reset(driveptr)
_dw_reset:
	ret

dw_op:
	ld (op + 1),de
dw_op_loop:
	ld a,b
	or c
	ret z
op:	call rxbyte		; self modifies
	ret c
	ld (hl),a
	inc hl
	dec bc
	jr dw_op_loop

	; dw_operation(cmdblock)
_dw_operation:
	pop bc		; return
	pop iy		; cmdblock
	push iy
	push bc

	ld b, (iy + 7)	; page map LSB
	call map_proc_a
	ld h, (iy + 3)	; pointer
	ld l, (iy + 4)
	ld b, (iy + 1)
	ld c, (iy + 2)
	xor a
	cp	(iy + 0)
	ld	a,(iy + 5)
	jr nz, is_write
	call dw_read_sector
dw_op_ret:
	ld hl, #0
	jr nc, dw_op_ok
	dec hl
dw_op_ok:
	jp map_kernel
is_write:
	call dw_write_sector
	jr dw_op_ret

; Entry A = command
; Exit BCD = 0

dw_hdr:
	call txbyte
	ld a, (de)	; drive
	call txbyte
	xor a		; LSN 23-16 = 0
	call txbyte
	ld a, b		; sector high
	call txbyte
	ld a, c		; sector low	
	call txbyte
	ld b, #0
	ld d, b		;checkum starts 0
	ld e, b
	ret

dw_write_sector:
	ld a, #0x57		; WRITE
	call dw_hdr
payload1:
	ld a, e
	add a, (hl)
	ld e, a
	ld a, d
	adc a, 0
	ld d, a
	ld a, (hl)
	inc hl
	call txbyte
	djnz payload1
	ld a, d
	call txbyte
	ld a, e
	call txbyte
	call rxbyte
	ret

dw_read_sector:
	ld a, #0xD2		; READEX
	call dw_hdr
	call rxbyte
	ret c
	or a
	jr nz, error_ret
;
;	256 receive bytes
;
payload2:
	call rxbyte
	ret c
	ld (hl), a
	inc hl
	ld a, (hl)
	add a, e
	ld e, a
	ld a, #0
	add a, d
	ld d, a
	djnz payload2
	ld a, d
	call txbyte
	ld a, e
	call txbyte
	call rxbyte
	ret c
	or a
	ret z
	; We could be smarter about error codes and reporting ?
error_ret:
	scf
	ret

;
;	Serial logic, polled interrupts off so we can do a reliable 19200
;	baud (hardware limit)
;
txbyte:
	in a, (0xEA)
	bit 6, a
	jr z, txbyte
	out (0xEB), a
	ret
;
;	We poll for receive in this mode as interrupts are off and we want
;	*speed*
;
rxbyte:
	; Loop time is 56 clocks so ~=72K loops/second
	; we want to spin for 0.25 (Drivewire limit)
	ld bc, #0x46
rxbytel:
	in a, (0xEA)		; 11
	bit 7, a		; 8
	jr nz, gotbyte		; 7(12)
	dec bc			; 10
	ld a, b			; 4
	or c			; 4
	jr nz, rxbytel		; 7(12)
	scf
	ret
gotbyte:
	in a, (0xEB)
	or a
	ret

