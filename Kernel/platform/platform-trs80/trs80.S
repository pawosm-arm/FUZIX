;
;	TRS 80  hardware support
;
	; exported symbols
	.export init_early
	.export init_hardware
	.export interrupt_handler
	.export _program_vectors
	.export plt_interrupt_all

	; hard disk helpers
	.export _hd_xfer_in
	.export _hd_xfer_out
	; and the page from the C code
	.export _hd_page

	; exported debugging tools
	.export _plt_monitor
	.export _plt_reboot
	.export outchar

	; imported symbols
	.export _ramsize
	.export _procmem
	.export istack_top
	.export istack_switched_sp
	.export unix_syscall_entry
	.export outcharhex
	.export fd_nmi_handler
	.export null_handler
	.export map_kernel
	.export map_proc
	.export map_proc_a
	.export _opreg
	.export _modout
	.export _int_disabled

	.export map_proc_always
	.export map_kernel

	.export _bufpool

#include "kernelu.def"
#include "../../cpu-z80u/kernel-z80.def"

	.buffers
_bufpool:
	.ds BUFSIZE * NBUFS

	.common

_int_disabled:
	.byte	1

_plt_monitor:
	di
	halt

plt_interrupt_all:
	ret

;
;	We write the following into low memory
;	out (0x84),a
;	inc a
;	out (0x9c),a
;	.. then resets into ROM (see the technical reference manual)
;
_plt_reboot:
	di
	ld	hl,0x84D3		; out (0x84),a
	ld	(0),hl
	ld	hl,0xD33C		; inc a, out (
	ld	(2),hl
	ld	a,0x9C			; 9c),a
	ld	(4),a
	xor a
	rst 0

	.code

_plt_idle:
	halt
	ret

_ctc6845:			; registers in order
	.byte 99, 80, 85, 10, 25, 4, 24, 24, 0, 9, 101, 9, 0, 0, 0, 0
init_early:
	ld	a, 0x24		; uart rx, timer
	out	(0xE0), a
	ld	a, (_opreg)
	out	(0x84), a
	ld	a, (_modout)
	out	(0xEC), a

	; load the 6845 parameters
	ld	hl, _ctc6845
	ld	bc, 0x88
crtcloop:
	out	(c), b		; register
	ld	a, (hl)
	out	(0x89), a	; data
	inc	hl
	inc	b
	bit	4,b
	jr	z, crtcloop

   	; clear screen
	ld	hl, 0xF800
	ld	(hl), '*'		; debugging aid in top left
	inc	hl
	ld	de, 0xF802
	ld	bc, 1998
	ld	(hl), ' '
	ldir
	ret

	.common

_program_vectors:
	; we are called, with interrupts disabled, by both newproc() and crt0
	; will exit with interrupts off
	di ; just to be sure
	pop	de ; temporarily store return address
	pop	hl ; function argument -- base page number
	push	hl ; put stack back as it was
	push	de

	call	map_proc

	; write zeroes across all vectors
	ld	hl, 0
	ld	de, 1
	ld	bc, 0x007f	; program first 0x80 bytes only
	ld	(hl), 0x00
	ldir

	; now install the interrupt vector at 0x0038
	ld	a, 0xC3 ; JP instruction
	ld	(0x0038), a
	ld	hl, interrupt_handler
	ld	(0x0039), hl

	ld	(0x0000), a   
	ld	hl, null_handler   ;   to Our Trap Handler
	ld	(0x0001), hl

	ld	(0x0066), a  ; Set vector for NMI
	ld	hl, fd_nmi_handler
	ld	(0x0067), hl
	jp	map_kernel
	
; outchar: Wait for UART TX idle, then print the char in A
; destroys: AF
outchar:
	; FIXME: wait idle (works fine on emu as is of course)
	out	(0xEB), a
	ret

;
;	Swap helpers
;
_hd_xfer_in:
	pop	de
	pop	hl
	push	hl
	push	de
	ld	a, (_hd_page)
	or	a
	call 	nz, map_proc_a
	ld	bc, #0xC8		; 256 bytes from 0xC8
	inir
	jp	map_kernel

_hd_xfer_out:
	pop	de
	pop	hl
	push	hl
	push	de
	ld	a, (_hd_page)
	or	a
	call	nz, map_proc_a
	ld	bc, #0xC8		; 256 bytes to 0xC8
	otir
xfer_out_wait:				; busy wait after OTIR
	ex	(sp), hl
	ex	(sp), hl
	in	a,(0xCF)
	rlca
	jr	c, xfer_out_wait
	jp	map_kernel

;
;	Video screen flip
;

	.export	_vtswap
__vtswap:
	push	bc
	ld	hl, 0xf800
	ld	de, _vtbackbuf
	ld	bc, 80*24
exchit:
	push	bc
	ld	a, (de)	; Could be optimised but its only 2K
	ld	c, (hl)	; Probably worth doing eventuallly
	ex	de, hl
	ld	(hl), c
	ld	(de), a
	inc	hl
	inc	de
	pop	bc
	dec	bc
	ld	a, b
	or	c
	jr	nz, exchit
	pop	bc
	ret

;
;	Graphics
;
	.export _gfx_write
	.export _gfx_read
	.export _gfx_draw
	.export _gfx_exg
	.export _gfx_blit

	.export _ctrl_cache

_gfx_write:
	pop	de
	pop	hl
	push	hl
	push	de
	call	map_proc_always
	push	ix
	push	bc
	push	hl
	pop	ix
	ld	de,8
	add	hl,de

	ld	e,(ix + 6)	; Lines to do
	ld	d,(ix + 4)	; Y

	ld	a,(_ctrl_cache)
	or	0x40		; clock X on write
	out	(0x83),a
	ld	c,0x82			; data

wnextline:
	ld	a,d
	out	(0x81),a	; set Y
	ld	b,(ix + 4)	; width
	ld	a,(ix+ 0)	; X left
	out	(0x80),a
	otir			; write bytes
	inc	d		; move down
	dec	e		; count one off
	jr	nz, wnextline
	pop	bc
	pop	ix
	ld	hl,0
	jp	map_kernel


_gfx_read:
	pop	de
	pop	hl
	push	hl
	push	de
	call	map_proc_always
	push	ix
	push	bc
	push	hl
	pop	ix
	ld	de,8
	add	hl,de

	ld	e,(ix + 4)	; Lines to do
	ld	d,(ix + 0)	; Y

	ld a,(_ctrl_cache)
	or	0x10		; clock X on read
	out	(0x83),a
	ld	c,0x82		; data

rnextline:
	ld	a,d
	out	(0x81),a	; set Y
	ld	b,(ix + 6)	; width
	ld	a,(ix + 2)	; X left
	out	(0x80),a
	inir			; write bytes
	inc	d		; move down
	dec	e		; count one off
	jr	nz, rnextline
	pop	bc
	pop	ix
	ld	hl,0
	jp	map_kernel

_gfx_exg:
	pop	hl
	pop	de
	call	map_proc_always
	push	ix
	push	bc
	push	hl
	pop	ix
	ld	de,8
	add	hl,de

	ld	e,(ix + 4)	; Lines to do
	ld	d,(ix + 0)	; Y

	ld	a,(_ctrl_cache)
	or	0x20		; clock X on write
	out	(0x83),a
	ld	c,0x82		; data

enextline:
	ld	a,d
	out	(0x81),a	; set Y
	ld	b,(ix + 6)	; width
	ld	a,(ix + 2)	; X left
	out	(0x80),a
ebyte:
	ld	a,(hl)		; get new
	ini			; read byte to memory
	out	(0x82),a	; write new and inc
	djnz	ebyte
	inc	d		; move down
	dec	e		; count one off
	jr	nz, enextline
	pop	bc
	pop	ix
	ld	hl,0
	jp	map_kernel

_gfx_draw:
	pop	de
	pop	hl
	push	hl
	push	de
	call	map_proc_always	; Length, Y, X
	ld	de,	6			
	add	hl,de
	push	ix
	push	bc
	push	hl
	pop	ix
	ld	a,(_ctrl_cache)
	or	0x40		; clock X on write
	out	(0x83),a
	ld	d,(ix + 4)	; X
	ld	e,(ix + 2)	; Y
nextopline:
	; Set position
	ld	a,d
	out	(0x80),a
	ld	a,e
	out	(0x81),a
nextop:
	xor	a
	ld	b,(hl)
	cp	b
	jr	z,line_done
	inc	hl
	ld	c,(hl)
	inc	hl
oploop:
	in	a,(0x82)	; read no inc
	and	c
	xor	(hl)
	out	(0x82),a	; write, inc
	djnz	oploop
	inc	hl
	jr	nextop
line_done:
	inc	e		; down a line
	xor	a
	cp	(hl)
	jr	nz, nextopline
	pop	bc
	pop	ix
	ld	hl,0
	jp	map_kernel

_gfx_blit:
	pop	de
	pop	hl
	push	hl
	push	de
	push	ix
	push	bc
	push	hl
	pop	ix

	ld	d,(ix + 10)	; width
	res	7,d

	exx
	ld	a,(_ctrl_cache)
	or	0x10		; alt bc are the config sets
	ld	b,a
	xor	0x30
	ld	c,a
	exx

	ld	a,(ix + 0)	; Compare Y
	cp	(ix + 4)
	jr	c, blitrd
	jr	nz, blitlu
	ld	a,(ix + 2)	; and X
	cp	(ix + 6)
	jr	c, blitrd

blitlu:
	ld	e,(ix + 8)	; start on bottom line and work up
	dec	e
blitlu_n:
	exx
	ld	a,b
	out	(0x83),a
	exx
	ld	a,(ix + 2)	; source X
	out	(0x80),a
	ld	a,e
	add	a, (ix + 0)	; add Y source
	out	(0x81),a
	ld	b,d		; width of line
	ld	hl,scratch
	inir
	exx
	ld	a,(ix + 6)
	out	(0x80),a
	ld	a,e
	add	a, (ix + 4)	; add Y dest
	out	(0x81),a
	ld	a,c
	out	(0x83),a
	exx	
	ld	b,d
	ld	hl,scratch
	otir
	dec	e
	jr	nz, blitlu_n
	pop	ix
	ld	hl,0
	ret
	
blitrd:
	ld	e,0		; line counter
blitrd_n:
	exx
	ld	a,b
	out	(0x83),a
	exx
	ld	a,d		; source X
	out	(0x80),a
	ld	a,e
	add 	a, (ix + 0)	; add Y source
	out	(0x81),a
	ld	b,(ix + 8)	; width of line
	ld	hl,scratch
	inir
	exx
	ld	a,d
	out	(0x80),a
	ld	a,e
	add	a, (ix + 4)	; add Y dest
	out	(0x81),a
	ld	a,c
	out	(0x83),a
	exx	
	ld	b,(ix + 8)
	ld	hl,scratch
	otir
	inc	e
	ld	a, (ix + 8)	; height match ?
	cp	e
	jr	nz, blitrd_n
	pop	ix
	ld	hl,0
	ret

scratch:
	.ds 128
