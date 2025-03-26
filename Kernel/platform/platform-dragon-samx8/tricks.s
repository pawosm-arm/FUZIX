;
;	6809 version
;
	.module tricks

	; imported
	.globl _makeproc
	.globl _chksigs
	.globl _getproc
	.globl _plt_monitor
	.globl _inint
	.globl map_kernel
	.globl map_proc_always
	.globl copybank
	.globl _nready
	.globl _plt_idle
	.globl _udata

	; exported
	.globl _plt_switchout
	.globl _switchin
	.globl _dofork
	.globl _ramtop

	include "kernel.def"
	include "../../cpu-6809/kernel09.def"

	.area .commondata

	; ramtop must be in common although not used here
_ramtop:
	.dw 0

newpp:	.dw 0

	.area .common

; Switchout switches out the current process, finds another that is READY,
; possibly the same process, and switches it in.  When a process is
; restarted after calling switchout, it thinks it has just returned
; from switchout().
;
_plt_switchout:
	orcc #0x10	; irq off

	; save machine state, including Y and U used by our C code
	ldd #0 ; return code set here is ignored, but _switchin can
	; return from either _switchout OR _dofork, so they must both write
	; U_DATA__U_SP with the following on the stack:
	pshs d,y,u
	sts U_DATA__U_SP	; this is where the SP is restored in _switchin

	; Stash the uarea into process memory bank
	; FIXME could map in only the bank that holds the stash
	lda U_DATA__U_PAGE+3
	sta 0xff34
	ldx #_udata
	ldy #U_DATA_STASH&0x3fff
stash:	ldd ,x++
	std ,y++
	cmpx #_udata+U_DATA__TOTALSIZE
	bne stash
	; get process table in
	clr 0xff34

	; find another (or same) process to run, returned in X
	jsr _getproc
	jsr _switchin
	; we should never get here
	jsr _plt_monitor

badswitchmsg:
	.ascii "_switchin: FAIL"
	.db 13
	.db 10
	.db 0


; new process pointer is in X
_switchin:
	orcc #0x10	; irq off

	stx newpp

	ldd P_TAB__P_PAGE_OFFSET,x	; 0 if swapped
	bne not_swapped

	; process was swapped out?
	cmpx #0
	bne not_swapped
	jsr _swapper	; void swapper(ptptr p)
	ldx newpp

not_swapped:
	lda P_TAB__P_PAGE_OFFSET+3,x
	sta 0xff34

	; fetch uarea from process memory
	ldx #U_DATA_STASH&0x3fff
	ldy #_udata
stashb:	ldd ,x++
	std ,y++
	cmpx #(U_DATA_STASH+U_DATA__TOTALSIZE)&0x3fff
	bne stashb

	; we have now new stacks so get new stack pointer before any jsr
	lds U_DATA__U_SP

	; get back kernel page so that we see process table
	clr 0xff34

nostash:
	ldx newpp
	; check u_data->u_ptab matches what we wanted
	cmpx U_DATA__U_PTAB
	bne switchinfail

	lda #P_RUNNING
	sta P_TAB__P_STATUS_OFFSET,x

	; fix any moved page pointers
	ldd P_TAB__P_PAGE_OFFSET,x
	std U_DATA__U_PAGE
	ldd P_TAB__P_PAGE_OFFSET+2,x
	std U_DATA__U_PAGE2

	ldx #0
	stx _runticks

	; restore machine state -- note we may be returning from either
	; _switchout or _dofork
	lds U_DATA__U_SP
	puls x,y,u ; return code and saved U and Y

	; enable interrupts, if the ISR isn't already running
	lda U_DATA__U_ININTERRUPT
	bne swtchdone ; in ISR, leave interrupts off
	andcc #0xef
swtchdone:
	rts

switchinfail:
	jsr outx
	ldx #badswitchmsg
	jsr outstring
	; something went wrong and we didn't switch in what we asked for
	jmp _plt_monitor

	.area .data

fork_proc_ptr:	.dw 0	; (C type is struct p_tab *)
		; -- address of child process p_tab entry

	.area .common
;
;	Called from _fork. We are in a syscall, the uarea is live as the
;	parent uarea. The kernel is the mapped object.
;
_dofork:
	; always disconnect the vehicle battery before performing maintenance
	orcc #0x10	 ; should already be the case ... belt and braces.

	; new process in X, get parent pid into X

	stx fork_proc_ptr
	ldx P_TAB__P_PID_OFFSET,x

	; Save the stack pointer and critical registers (Y and U used by C).
	; When this process (the parent) is switched back in, it will be as if
	; it returns with the value of the child's pid.
	pshs x,y,u	; x has p->p_pid from above, the return value in the parent

	; save kernel stack pointer -- when it comes back in the parent we'll be in
	; _switchin which will immediately return (appearing to be _dofork()
	; returning) and with X (ie return code) containing the child PID.
	; Hurray.
	sts U_DATA__U_SP

	; now we're in a safe state for _switchin to return in the parent
	; process.

	jsr fork_copy	; copy process memory to new bank
	; and save parents uarea

	; We are now in the kernel child context

	; now the copy operation is complete we can get rid of the stuff
	; _switchin will be expecting from our copy of the stack.
	leas 2,s

	ldx #_udata
	pshs x
	ldx fork_proc_ptr
	jsr _makeproc
	leas 2,s

	; any calls to map process will now map the childs memory

	; in the child process, fork() returns zero.
	ldx #0
	; runticks = 0;
	stx _runticks
	;
	; And we exit, with the kernel mapped, the child now being deemed
	; to be the live uarea. The parent is frozen in time and space as
	; if it had done a switchout().
	puls y,u,pc

fork_copy:
	; copy process memory to the new banks and stash parent uarea to old banks
	lda #3				; counter, decremented until negative
	pshs a
	ldx fork_proc_ptr
	leax P_TAB__P_PAGE_OFFSET,x	; new bank map
	ldy #U_DATA__U_PAGE		; old bank map
cpbank:	lda ,y+				; src
	cmpa ,y				; repeated bank = last
	bne notlst
	clr ,s
notlst:	ldb ,x+				; dst
	bsr copybank			; clobbers only A,B
	dec ,s
	bpl cpbank
	leas 1,s			; drop counter

	; stash parent uarea (including kernel stack)
	lda U_DATA__U_PAGE+3
	sta 0xff34
	ldx #_udata
	ldu #U_DATA_STASH&0x3fff
stashf:	ldd ,x++
	std ,u++
	cmpx #_udata+U_DATA__TOTALSIZE
	bne stashf
	clr 0xff34

	ldx fork_proc_ptr
	leax P_TAB__P_PAGE_OFFSET,x	; child's map
	jmp mmu_remap_x			; exit with child mapped in user task
	; --- we are now on the stack copy, parent stack is locked away ---
	; this stack is copied so safe to return on

; fast bank copy for fork (called with kernel mapped)
; src bank in A, dst bank in B

; we simply copy the full 16KB bank here

copybank:
	pshs dp,x,y,u
	std 0xff35		; map src at 0x4000, dst at 0x8000
	sts save_sp
	lds #0xc000		; top of DEST
	ldu #0x8000+7		; top of SRC
smash:	leau -14,u
	pulu dp,d,x,y		; transfer 7 bytes at a time
	pshs dp,d,x,y		; 6 times.. 42 bytes per loop
	leau -14,u
	pulu dp,d,x,y
	pshs dp,d,x,y
	leau -14,u
	pulu dp,d,x,y
	pshs dp,d,x,y
	leau -14,u
	pulu dp,d,x,y
	pshs dp,d,x,y
	leau -14,u
	pulu dp,d,x,y
	pshs dp,d,x,y
	leau -14,u
	pulu dp,d,x,y
	pshs dp,d,x,y
	cmpu #0x4004+42+7	; bottom of SRC? (leave space for interrupt)
	bne smash		; no repeat
	leax -7,u		; pick up SRC in X
	ldu save_sp		; restore stack...
	exg u,s			; ... and DEST now in U
safe:	ldd ,--x		; move last 46 bytes with a normal stack
	pshu d
	cmpx #0x4000		; reached bottom of SRC?
	bne safe
	ldd #0x0102
	std 0xff35		; restore kernel map
	puls dp,x,y,u,pc

	.area .commondata

save_sp:
	.dw 0
