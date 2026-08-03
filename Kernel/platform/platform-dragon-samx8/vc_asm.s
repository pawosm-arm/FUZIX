;
; low-level routines for VC handling
;

	.module vc_asm

	.globl _set_vid_mode
	.globl _set_vc_mode
	.globl _vc_clear
	.globl _vc_write_char
	.globl _vc_read_char
	.globl _vc_memset
	.globl _vc_scroll_up
	.globl _vc_scroll_down

	include "kernel.def"

	.area .video

_set_vc_mode:
	; SAM V=0 (SG4)
	sta 0xffc0
	sta 0xffc2
	sta 0xffc4
	; SAMx8 base = (VC_BASE_ABS + B*512) / 32
	; = VC_FREG + B*16
	lslb
	lslb
	lslb
	lslb
	clra
	addd #VC_FREG
	std 0xff38
	; VDG mode 0 (SG4)
	lda 0xff22
	anda #0x07
	sta 0xff22
	rts

_set_vid_mode:
	ldd #VIDEO_FREG
	std 0xff38
	jmp _vid256x192		; set resolution

_vc_clear:
	tfr b,a
	asla
	clrb
	ldx #VC_BASE+512
	leax d,x
	pshs x
	leax -512,x
	jsr _map_video
	ldd #0x2020
cllp:	std ,x++
	cmpx ,s
	blo cllp
	leas 2,s
	jmp _unmap_video

_vc_write_char:
	jsr _map_video
	stb ,x
	jmp _unmap_video

_vc_read_char:
	jsr _map_video
	ldb ,x
	jmp _unmap_video

vtbase:	lda _curtty
	deca
	asla
	clrb
	ldx #VC_BASE
	leax d,x
	;leay 512,x
	rts

_vc_scroll_up:
	pshs x,y		; x: make space for end address
	bsr vtbase
	leay 480,x
	sty ,s
	leay 32,x
	jsr _map_video
scup:	ldd ,y++
	std ,x++
	cmpx ,s
	blo scup
	jsr _unmap_video
	puls x,y,pc

_vc_scroll_down:
	pshs x,u
	bsr vtbase
	leau 512,x
	leax 32,x
	stx ,s
	leax -32,u
	jsr _map_video
scdn:	ldd ,--x
	pshu d
	cmpx ,s
	bhi scdn
	jsr _unmap_video
	puls x,y,pc

_vc_memset:
	pshs y
	ldy 4,s
	jsr _map_video
vcms:	stb ,x+
	leay -1,y
	bne vcms
	jsr _unmap_video
	puls y,pc

