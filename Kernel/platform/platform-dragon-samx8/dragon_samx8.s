;
; SAMx8 platform
;

	.module dragon_samx8

	; exported
	.globl _mpi_present
	.globl _mpi_set_slot
	.globl _cart_hash
	.globl _cart_analyze_hdb
	.globl _bufpool
	.globl _discard_size
	.globl _copy_common
	.globl _framedet
	.globl blkdev_rawflg
	.globl blkdev_unrawflg

	; imported
	.globl unix_syscall_entry
	.globl fd_nmi_handler
	.globl size_ram
	.globl null_handler
	.globl _vid256x192
	.globl _vtoutput

	; exported debugging tools
	.globl _plt_monitor
	.globl _plt_reboot
	.globl outchar
	.globl ___hard_di
	.globl ___hard_ei
	.globl ___hard_irqrestore

	include "kernel.def"
	include "../../cpu-6809/kernel09.def"


	.area .vectors
	;
	; Will be read-only at $FFE0 when COMMON enabled
	;
	.ds 0x10		; 16 spare bytes!
	.dw badswi_handler	; 6809 Reserved / 6309 Trap
	.dw badswi_handler	; SWI3
	.dw badswi_handler	; SWI2
	.dw firq_handler	; FIRQ
	.dw interrupt_handler	; IRQ
	.dw unix_syscall_entry	; SWI
	.dw fd_nmi_handler	; NMI
	.dw badswi_handler	; Restart


	.area .buffers
	;
	; We use the linker to place these just below
	; the discard area
	;
_bufpool:
	.ds BUFSIZE*NBUFS


	; And expose the discard buffer count to C - but in discard
	; so we can blow it away and precomputed at link time
	.area .discard
_discard_size:
	.db __sectionlen_.discard__/BUFSIZE

init_early:
	ldx #null_handler
	stx 1
	lda #0x7e
	sta 0
	rts

init_hardware:
	sta 0xffd9	; fast mode
	sta 0xffdf	; 64K RAM (loader should already have done this)
	jsr size_ram
	; Turn on PIA CB1 (FSync interrupt)
	lda 0xff03
	ora #1
	sta 0xff03
	; 50Hz or 60Hz?
	ldx #0
	lda 0xff02
waitvb0
	lda 0xff03
	bpl waitvb0	; wait for vsync
	lda 0xff02
waitvb2:
	leax 1,x	; time until vsync starts
	lda 0xff03
	bpl waitvb2
	stx _framedet
	jsr _vid256x192
	jmp _vtinit	; tail call

_framedet:
	.word 0

; old p6809.s stuff below

	; exported symbols
	.globl init_early
	.globl init_hardware
	.globl _program_vectors
	.globl _need_resched


	; imported symbols
	.globl _ramsize
	.globl _procmem
	.globl unix_syscall_entry
	.globl fd_nmi_handler

	.area .commondata

; in a place where internal memory is kept alone

_plt_reboot:
	ldd #0x3f3f
	orcc #0x10
	sta 0x0071	; cold boot flag for ROM reset handler
	; TODO: properly deconfigure.  may need a trampoline function
	; copied somewhere.  but for now...
	jmp [0xfffe]	; BASIC ROM & vectors are back

	.area .common

_plt_monitor:
	orcc #0x10
	bra _plt_monitor

___hard_di:
	tfr cc,b	; return the old irq state
	orcc #0x10
	rts
___hard_ei:
	andcc #0xef
	rts

___hard_irqrestore:	; B holds the data
	tfr b,cc
	rts


;
;------------------------------------------------------------------------------
; COMMON MEMORY PROCEDURES FOLLOW


	.area .common

; Called by pagemap_realloc and makeproc
_program_vectors:
	pshs a,b
	lda ,x
	sta 0xff34
	lda #0x7e
	sta 0
	ldd #null_handler
	std 1
	clr 0xff34
	puls a,b,pc


;
; Helpers for the MPI and Cartridge Detect
;

	.area .text
;
; oldslot = mpi_set_slot(uint8_t newslot)
;
_mpi_set_slot:
	tfr b,a
	ldb 0xff7f
	sta 0xff7f
	rts
;
; int8_t mpi_present(void)
;
_mpi_present:
	lda 0xff7f	; Save bits
	tfr a,b
	lsrb
	lsrb
	lsrb
	lsrb
	eorb 0xff7f
	andb #0x03	; We expect to see the bits 5-4 and 1-0 matching
	bne nompi	; not guaranteed but a good rule of thumb for us
	ldb #0xff	; Will get back 33 from an MPI cartridge
	stb 0xff7f	; if the emulator is right on this
	ldb 0xff7f
	andb #0x33
	cmpb #0x33
	bne nompi
	clr 0xff7f	; Switch to slot 0
	ldb 0xff7f
	andb #0x33	; We can't trust the high bits
	bne nompi
	incb
	sta 0xff7f	; Our becker port for debug will be on the default
	; slot so put it back for now
	rts	; B = 0
nompi:	ldb #0
	sta 0xff7f	; Restore bits just in case
	rts

; With a COMMON bit in SAMx8, we never need to copy_common()
_copy_common:
	rts

	.area .text
;
; Joystick helper
;
; jsread(buffer)
;
; Returns a buffer of words in the format
; right left/right, button
; right up/down, button
; left left/right, button
; left up/down, button
;
	.globl _jsread

_jsread:
	; Buffer is in X on entry
	pshs u
	lda #0xff
	sta 0xff02	; Keyboard scan lines off
	lda #0x08	; Select right joystick
	sta 0xff23	; Sound off a moment
	bsr jstwo
	lda #0x09
	bsr jstwo
	puls u
	rts
jstwo:
	sta 0xff03	; P0 CR B - select joystick L or R
	lda #0x04
	sta 0xff01	; X
	bsr jsfind
	lda #0x0c	; Y
	sta 0xff01
	; Fall through
jsfind:
	ldu #jstmp
	; Binary search the joystick DAC position
	lda #0x20
	sta ,u	; start in the middle and binary search
jssearch:
	lsr ,u
	beq jsdone
	sta 0xff20
	tst 0xff20
	bpl jsover
	adda ,u
	bra jssearch
jsover:
	suba ,u
	bra jssearch
jsdone:
	ldb 0xff20	; save fire button in bit 0
	std ,x++
	rts

	.area .data
jstmp:
	.byte 0

	.area .common
;
; FIXME:
;
firq_handler:
badswi_handler:
swi2_handler:
swi3_handler:
	rti

;
; debug via printer port
;
outchar:
	sta 0xff02
	lda 0xff20
	ora #0x02
	sta 0xff20
	anda #0xfd
	sta 0xff20
	rts

	.area .commondata

_need_resched:	.db 0
