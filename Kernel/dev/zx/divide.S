
	.export _devide_read_data
	.export _devide_write_data

	.common

#define IDE_REG_DATA	0xA3

ide_setup:
	ld	bc, IDE_REG_DATA		; port and count
	ld	a, (_td_raw)
	or	a
	jp	z, map_buffers
	dec	a
	jp	z, map_proc_always
	ld	a, (_td_page)
	jp	map_for_swap

_devide_read_data:
	pop	de
	pop	hl
	push	hl
	push	de
	push	bc
	call	ide_setup
	ld 	a,0x05
	out	(0xfe),a
	inir                                    ; transfer first 256 bytes
	ld 	a,0x02
	out	(0xfe),a
	inir                                    ; transfer second 256 bytes
	ld 	a, (_vtborder)
	out	(0xfe),a
	pop	bc
	jp	map_kernel_restore

_devide_write_data:
	pop	de
	pop	hl
	push	hl
	push	de
	push	bc
	call	ide_setup
	ld 	a,0x05
	out	(0xfe),a
	otir                                    ; transfer first 256 bytes
	ld 	a,0x02
	out	(0xfe),a
	otir                                    ; transfer second 256 bytes
	ld 	a, (_vtborder)
	out	(0xfe),a
	pop	bc
	jp	map_kernel_restore
