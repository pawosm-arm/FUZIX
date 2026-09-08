# 1 "tricks.S"
# 1 "kernelu.def"
; FUZIX mnemonics for memory addresses etc

U_DATA__TOTALSIZE	.equ	0x200	; 256+256 bytes @ 0xC000
Z80_TYPE		.equ	0	; CMOS

Z80_MMU_HOOKS		.equ 0

CONFIG_SWAP		.equ 1

PROGBASE		.equ	0x0000
PROGLOAD		.equ	0x0100

; Mnemonics for I/O ports etc

CONSOLE_RATE		.equ	9600

CPU_CLOCK_KHZ		.equ	4000

B_RST	.equ	0x80
B_AACK	.equ	0x10
B_ASEL	.equ	0x04
B_ABUS	.equ	0x01

B_BUSY	.equ	0x40
B_MSG	.equ	0x10
B_CD	.equ	0x08
B_IO	.equ	0x04

B_PHAS	.equ	0x08

ncr_base	.equ	0x20
ncr_data	.equ	ncr_base
ncr_cmd		.equ	ncr_base + 1
ncr_mod		.equ	ncr_base + 2
ncr_tgt		.equ	ncr_base + 3
ncr_bus		.equ	ncr_base + 4
ncr_st		.equ	ncr_base + 5
ncr_dma_w	.equ	ncr_base + 5
ncr_int		.equ	ncr_base + 7
ncr_dma_r	.equ	ncr_base + 7
ncr_dack	.equ	ncr_base + 8
DARTA_D		.equ	0x80
DARTA_C		.equ	0x84
DARTB_D		.equ	0x88
DARTB_C		.equ	0x8C

CTC_CH0		.equ	0x40
CTC_CH1		.equ	0x50
CTC_CH2		.equ	0x60
CTC_CH3		.equ	0x70
# 1 "../../cpu-z80u/kernel-z80.def"
 
# 26
 
# 44
 
# 1 "../../lib/z80usingle.S"
;
;	Generic handling for single process in memory Z80 systems.
;
;	This code could really do with some optimizing.
;
;	FIXME: IRQ enable logic during swap
;
        .export _plt_switchout
        .export _switchin
        .export _dofork

	.common

; Switchout switches out the current process, finds another that is READY,
; possibly the same process, and switches it in.  When a process is
; restarted after calling switchout, it thinks it has just returned
; from switchout().
;
; This function can have no arguments or auto variables.
_plt_switchout:
        di
        ; save machine state

        ld hl, 0 ; return code set here is ignored, but _switchin can 

        ; return from either _switchout OR _dofork, so they must both write 
        ; 14     with the following on the stack:
        push hl ; return code
	push bc
        push ix
        push iy
        ld (_udata + 14    ), sp ; this is where the SP is restored in _switchin

        ; find another process to run (may select this one again)
        call _getproc

        push hl
        call _switchin

        ; we should never get here
        call _plt_monitor

badswitchmsg: .ascii "_switchin: FAIL"
.byte 13, 10, 0
swapped: .ascii "_switchin: SWAPPED"
.byte 13, 10, 0

_switchin:
        di
	ld a,#1
	ld (_int_disabled),a
        pop bc  ; return address  - ok to trash BC here
        pop de  ; new process pointer
;
;	FIXME: do we actually *need* to restore the stack !
;
        push de ; restore stack
        push bc ; restore stack

	push de
        ld hl, 15 
	add hl, de	; process ptr
	pop de

        ld a, (hl)

	or a
	jr nz, not_swapped
	;
	;	We are still on the departing processes stack, which is
	;	fine for now.
	;
	ld sp, _swapstack
	;
	;	The process we are switching from is not resident. In
	;	practice that means it exited. We don't need to write
	;	it back out as it's dead.
	;
	ld ix,(_udata + 0     )
	ld a, (ix + 15 )

	or a
	jr z, not_resident
	;
	;	DE = process we are switching to
	;
	push de	; save process
	; We will always swap out the current process
	ld hl, (_udata + 0     )
	push hl
	call _swapout
	pop hl
	pop de	; process to swap to
not_resident:
	push de	; called function may modify on stack arg so make
	push de	; two copies
	call _swapper
	pop de
	pop de

not_swapped:        
        ; check u_data->u_ptab matches what we wanted
        ld hl, (_udata + 0     ) ; u_data->u_ptab
        or a                    ; clear carry flag
        sbc hl, de              ; subtract, result will be zero if DE==HL
        jr nz, switchinfail

	; wants optimising up a bit
	ld ix, (_udata + 0     )
        ; next_process->p_status = 1           
        ld (ix + 0 ), 1           

	; Fix the moved page pointers
	; Just do one byte as that is all we use on this platform
	ld a, (ix + 15 )
	ld (_udata + 2     ), a
        ; runticks = 0
        ld hl, 0
        ld (_runticks), hl

        ; restore machine state -- note we may be returning from either
        ; _switchout or _dofork
        ld sp, (_udata + 14    )

        pop iy
        pop ix
	pop bc
        pop hl ; return code

        ; enable interrupts, if the ISR isn't already running
        ld a, (_udata + 16    )
	; put th einterrupt status flag back correctly
	ld (_int_disabled),a
        or a
        ret nz ; in ISR, leave interrupts off
        ei
        ret ; return with interrupts on

switchinfail:
	call outhl
        ld hl, badswitchmsg
        call outstring
	; something went wrong and we didn't switch in what we asked for
        jp _plt_monitor

fork_proc_ptr:
	.word 0 ; (C type is struct p_tab *) -- address of child process p_tab entry

;
;	Called from _fork. We are in a syscall, the uarea is live as the
;	parent uarea. The kernel is the mapped object.
;
_dofork:
        ; always disconnect the vehicle battery before performing maintenance
        di ; should already be the case ... belt and braces.

        pop de  ; return address
        pop hl  ; new process p_tab*
        push hl
        push de

        ld (fork_proc_ptr), hl

        ; Save the stack pointer and critical registers.
        ; When this process (the parent) is switched back in, it will be as if
        ; it returns with the value of the child's pid.
	ld hl,#0
        push hl		;	#0 child
	push bc
        push ix
        push iy

        ; save kernel stack pointer -- when it comes back in the child we'll be in
        ; _switchin which will immediately return (appearing to be _dofork()
	; returning) and with HL (ie return code) containing the child PID.
        ; Hurray.
        ld (_udata + 14    ), sp

        ; now we're in a safe state for _switchin to return in the parent
	; process.


        ; Make a new process table entry, etc.

	; Copy the parent properties into the temporary udata copy
	call _tmpbuf
	ld a,h
	or l
	jp z,ohpoo
	push hl
	ex de,hl
	ld hl, _udata
	ld bc, U_DATA__TOTALSIZE
	ldir

	; Recover the buffer pointer
	pop hl
	push hl

	; Make the child udata out of the temporary buffer
	push hl
        ld hl, (fork_proc_ptr)
        push hl
        call _makeproc
        pop bc
	pop bc

        ; in the child process, fork() returns zero.
	;
	; And we exit, with the kernel mapped, the child assembled in the
	; copy area as if it had done a switchout, the parent meanwhile
	; continues happily on

        ; now we're in a safe state for _switchin to return in the child
	; process swap out the image and the new udata

	; Stack the buffer as a second argument
	pop hl
	push hl
	push hl

	ld hl, (fork_proc_ptr)
	push hl
	call _swapout_new
	pop hl
	pop hl

	; Buffer pointer is sitting top of stack still
	; so use it as an argument to tmpfree
	call _tmpfree

	pop hl

	ld hl, (fork_proc_ptr)
        ; prepare return value in parent process -- HL = p->p_pid;
        ld de, 3 
        add hl, de
        ld a, (hl)
        inc hl
        ld h, (hl)
        ld l, a
        ; now the copy operation is complete we can get rid of the stuff
        ; _switchin will be expecting from our copy of the stack.
        pop iy
        pop ix
        pop bc
	pop de	; and throw the pid info away
	; Return pid of child we forked into swap
        ret

ohpoo:
	ld hl,#nobufs
	call outstring
	jp _plt_monitor

nobufs:
	.ascii 'nobufs'
	.byte 0
;
;	We can keep a stack in common because we will complete our
;	use of it before we switch common block. In this case we have
;	a true common so it's even easier.
;
	.ds 128
_swapstack:
# 1 "../../lib/z80uuser1.s"
;
;	Alternative user memory copiers for systems where all the kernel
;	data that is ever moved from user space is visible in the user
;	mapping.
;

        ; exported symbols
        .export __uget
        .export __ugetc
        .export __ugetw

        .export __uput
        .export __uputc
        .export __uputw

        .export __uzero

;
;	We need these in common as they bank switch
;
	.common
;
;	The basic operations are copied from the standard one. Only the
;	blk transfers are different. uputget is a bit different as we are
;	not doing 8bit loop pairs.
;
uputget:
        ; load DE with the byte count
        ld c, (ix + 8) ; byte count
        ld b, (ix + 9)
	ld a, b
	or c
	ret z		; no work
        ; load HL with the source address
        ld l, (ix + 4) ; src address
        ld h, (ix + 5)
        ; load DE with destination address (in userspace)
        ld e, (ix + 6)
        ld d, (ix + 7)
	ret	; 	Z is still false

__uputc:
	ld hl,2
	add hl,sp
	ld e,(hl)
	inc hl
	inc hl
	ld a,(hl)
	inc hl
	ld h,(hl)
	ld l,a
	call map_proc_always
	ld (hl), e
	jp map_kernel			; map the kernel back below common

__uputw:
	ld hl,2
	add hl,sp
	ld e,(hl)
	inc hl
	ld d,(hl)	; value
	inc hl
	ld a,(hl)
	inc hl
	ld h,(hl)
	ld l,a
	call map_proc_always
	ld (hl), e
	inc hl
	ld (hl), d
	jp map_kernel

__ugetc:
	pop de
	pop hl
	push hl
	push de
	call map_proc_always
        ld l, (hl)
	ld h, 0
	jp map_kernel

__ugetw:
	pop de
	pop hl
	push hl
	push de
	call map_proc_always
        ld a, (hl)
	inc hl
	ld h, (hl)
	ld l, a
	jp map_kernel

__uput:
	push ix
	ld ix, 0
	add ix, sp
	push bc
	call uputget			; source in HL dest in DE, count in BC
	jr z, uput_out			; but count is at this point magic
	call map_proc_always
	ldir
uput_out:
	call map_kernel
	pop bc
	pop ix
	ld hl, 0
	ret

__uget:
	push ix
	ld ix, 0
	add ix, sp
	push bc
	call uputget			; source in HL dest in DE, count in BC
	jr z, uput_out			; but count is at this point magic
	call map_proc_always
	ldir
	jr uput_out

;
__uzero:
	ld hl,2
	add hl,sp
	push bc
	ld e,(hl)
	inc hl
	ld d,(hl)
	inc hl
	ld c,(hl)
	inc hl
	ld b,(hl)
	ld a, b	; check for 0 copy
	or c
	jr z, popout
	call map_proc_always
	ld l,e
	ld h,d
	ld (hl), 0
	dec bc
	ld a, b
	or c
	jr z, popout
	inc de
	ldir
popout:
	pop bc
	jp map_kernel
