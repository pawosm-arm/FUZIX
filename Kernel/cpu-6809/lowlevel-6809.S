;
;	Common elements of low level interrupt and other handling. We
; collect this here to minimise the amount of platform specific gloop
; involved in a port
;
;	Based upon code (C) 2013 William R Sowerbutts
;

	; debugging aids
	.export outcharhex
	.export outd
	.export outx
	.export outy
	.export outnewline
	.export outstring
	.export outstringhex
	.export outnibble

        ; exported symbols
        .export unix_syscall_entry
	.export null_handler
	.export unix_syscall_entry
	.export dispatch_process_signal
        .export _doexec
        .export trap_illegal
	.export nmi_handler
	.export interrupt_handler
	.export _sys_cpu
	.export _sys_cpu_feat
	.export _sys_stubs
	.export _set_cpu_type

#include "../build/kernel.def"
#include "kernel09.def"

        .common

; entry point for Fuzix system calls
;
; Called by swi, which has already saved our CPU state for us
;
unix_syscall_entry:
	leax 14,s	; 12 stacked by the swi + return address of caller
	ldu #_udata + U_DATA__U_ARGN
	ldd ,x++	; first argument in swi stacked X
	std ,u++
	ldd ,x++	; second argument from caller's stack
	std ,u++
	ldd ,x++ 	; third
	std ,u++
	ldd ,x++ 	; fourth
	std ,u++
	ldd 1,s		; stacked D register -> syscall number in B
	stb _udata + U_DATA__U_CALLNO
        ; save process stack pointer (in user page)
        sts _udata + U_DATA__U_SYSCALL_SP
        ; switch to kernel stack (makes our stack valid again)
        lds #kstack_top

	; we are in syscall state
	lda #1
	sta _udata + U_DATA__U_INSYS

        ; map in kernel keeping common
	jsr map_kernel

        ; re-enable interrupts
        andcc #0xef

        ; now pass control to C
        jsr _unix_syscall

        orcc #0x10
	; let the interrupt logic know we are not in kernel mode any more
	clr _udata + U_DATA__U_INSYS

        ; map process memory back in based on common (common may have
        ; changed on a task switch)
        jsr map_proc_always

        ; switch back to user stack
        lds _udata + U_DATA__U_SYSCALL_SP

	; stack is now valid but user stack
        ; check for signals, call the handlers
        jsr dispatch_process_signal

        ; check if error condition to be signalled on return
        ldd _udata + U_DATA__U_ERROR
	beq not_error
	ldx #-1	
        ; error code in d, result in x
        bra unix_return

not_error:
        ; no error to signal! return syscall return value instead of error code
        ldx _udata + U_DATA__U_RETVAL
unix_return:
	; we never make a syscall from in kernel space
	stx 4,s		; replace stacked values before rti
	std 1,s
	rti
;
;	We must rts on the right bank !
;
dispatch_process_signal:
        ; check if any signal outstanding
        ldb _udata + U_DATA__U_CURSIG
        beq dosigrts

	; The signal handler is entitled to make syscalls which will
	; in turn trash these two
	ldx _udata + U_DATA__U_RETVAL
	ldy _udata + U_DATA__U_ERROR
	pshs x,y
        ; put number in X as the argument for the signal handler
	; so extend it to 16bit
	clra
	tfr d,x

	lslb		;	2 bytes per entry
        ; load the address of signal handler function
	ldy #_udata + U_DATA__U_SIGVEC
	leay b,y
	ldu ,y		; now u = udata.u_sigvec[cursig]

        ; udata.u_cursig = 0;
	clr _udata + U_DATA__U_CURSIG

        ; restore signal handler to the default.
        ; udata.u_sigvec[cursig] = SIG_DFL;
        ; SIG_DFL = 0
	clr ,y+
	clr ,y

        andcc #0xef
	jsr ,u

signal_return:
        orcc #0x10
	puls x,y
	stx _udata + U_DATA__U_RETVAL
	sty _udata + U_DATA__U_ERROR
dosigrts:
        rts

_doexec:
	ldx 2,s
	; x is the jump address
        orcc #0x10
	; this is a funny extra path out of syscall so we must also cover
	; the exit from kernel here

	; map task into address space
	; preserves x
        jsr map_proc_always

	; base address
	ldy _udata + U_DATA__U_CODEBASE
        ; u_data.u_insys = false
        clr _udata + U_DATA__U_INSYS
	; At this point the stack goes invalid
        lds _udata + U_DATA__U_ISP
	andcc #0xef			; IRQs on
        jmp ,x

;
;	Very simple IRQ handler, we get interrupts and we may have to
;	poll ttys from it. The more logic we could move to common here the
;	better.
;


;
;	Called when interrupts have been re-enabled within the timer
;	interrupt. We hand it to the platform re-interrupt handler. If
;	none is expected then it can panic, or if the platform is clever
;	it can do the needed work.
;
reinterrupt:
	jsr _plt_reinterrupt
	; Signals and other magic will happen when the first level of
	; interrupt handling returns
	rti

interrupt_handler:
	; If the platform interrupt code re-enabled interrupts then
	; we are on the interrupt stack already and platform author
	; is assumed to know what they are doing 8)
	tst _udata + U_DATA__U_ININTERRUPT
	bne reinterrupt

	; Do not use the stack before the switch...
	; FIXME: add profil support here (need to keep profil ptrs
	; unbanked if so ?)

	lda #1
        sta _udata + U_DATA__U_ININTERRUPT

        ; switch stacks
        sts istack_switched_sp
        lds #istack_top

	jsr map_save

	ldb _udata + U_DATA__U_INSYS	; In a system call ?
	bne in_kernel

        ; we're not in kernel mode, check for signals and fault
	lda #0x7e
	cmpa 0		; JMP at 0?
	beq nofault	; yes?  not a fault
	sta 0		; fault - put JMP back
	jsr map_kernel
	ldb #11		; SIGSEGV
	jsr trap_signal	; signal the user with a fault

nofault:
in_kernel:
        jsr map_kernel

	;
	; If the kernel decides to task switch it will set
	; _need_resched, and will only do so if the caller was in
	; user space so has a free kernel stack

        jsr _plt_interrupt

        ldx istack_switched_sp	; stack back
        clr _udata + U_DATA__U_ININTERRUPT
        lda _udata + U_DATA__U_INSYS
        bne intret_x
	lda _need_resched
	beq no_switch

	clr _need_resched
	stx _udata + U_DATA__U_SYSCALL_SP	; save again somewhere safe for
					; preemption
	; Pre emption occurs on the task stack. Conceptually its a
	; not quite a syscall
	lds #kstack_top
	jsr _chksigs		; check signal state
	;
	ldx _udata + U_DATA__U_PTAB
	; Move to ready state
	lda P_TAB__P_STATUS_OFFSET,x
	cmpa #P_RUNNING
	bne not_running
	lda #P_READY
	sta P_TAB__P_STATUS_OFFSET,x
not_running:
	; Sleep on the kernel stack, IRQs will get re-enabled if need
	; be
	jsr _plt_switchout
	;
	; We will resume here after the pre-emption. Get back onto
	; the user stack and map ourself in
	jsr map_proc_always
	lds _udata + U_DATA__U_SYSCALL_SP
	bra intdone

	    ; Not task switching - the easy and usual path
no_switch:   
	; On a return from an interrupt restore the old mapping as it
	; will vary during kernel activity and we need to put it put
	; it back as it was before the interrupt
	; pre-emption is handled differently...
	jsr map_restore
	lds istack_switched_sp

intdone: 
        ; we're not in kernel mode, check for signals
	; runs off the user stack
	pshs y
        jsr dispatch_process_signal
	puls y

intret:
        rti
	;	From kernel
	;	Restore the user mapping then
	;	switch to the user stack ptr
intret_x:
	jsr map_restore
	tfr x,s
	bra intret

;  Enter with B being the signal to send ourself
trap_signal:
	ldx _udata + U_DATA__U_PTAB	;  ssig(pid, B)
	jmp _ssig

;  Called from process context (hopefully)
null_handler:
	; kernel jump to NULL is bad
	lda _udata + U_DATA__U_INSYS
	bne trap_illegal
	; user is merely not good
	; check order of push arguments !!
	ldx #7			; SIGBUS
	pshs d,x		; D only to fill stack (no caller)
	ldx _udata + U_DATA__U_PTAB
	ldb #39			; Function 39 = kill
	swi			; kill (getpid(), SIGBUS)
	leas 4,s
	ldx #0
	clrb			; Function 0 = exit
	swi			; exit (0)

illegalmsg: .ascii "[trap_illegal]"
        .byte 13,10,0

trap_illegal:
	ldx #illegalmsg
	jsr outstring
	jsr _plt_monitor

dpsmsg:	.ascii "[dispsig]"
        .byte 13,10,0


nmimsg: .ascii "[NMI]"
        .byte 13,10,0

nmi_handler:
	lds #istack_top		; We aren't coming back so this is ok
	jsr map_kernel
        ldx #nmimsg
	jsr outstring
        jsr _plt_monitor

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;	CPU type management
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; We don't use the stubs as we have a proper architected syscalling
; interface via swi. Set it here so the binary gets 16bytes of free
; but uninteresting noise
_sys_stubs:

	.commondata

_sys_cpu:
	.byte 4
_sys_cpu_feat:
	.byte 0

	.common
;
;	Check for a 6309 (as per The 6309 Book)
;
_set_cpu_type:
	pshs d
	.word 0x1043
	cmpb 1,s
	puls d
	beq is8
	lda #1
	sta _sys_cpu_feat
is8:
	rts


; outstring: Print the string at X until 0 byte is found
; destroys: A, X
outstring:
	lda ,x+
	beq outstrdone
        jsr outchar
        bra outstring
outstrdone:
	rts

; print the string at (X) in hex (continues until 0 byte seen)
outstringhex:
	lda ,x+
	beq outstrdone
        jsr outcharhex
	lda #32
	jsr outchar
        bra outstringhex

; output a newline
outnewline:
        lda #0x0d  ; output newline
        jsr outchar
        lda #0x0a
        jsr outchar
        rts

outx:  ; prints X
	pshs d
	tfr x,d
	bsr outd
	puls d,pc

outy:  ; prints Y
	pshs d
	tfr y,d
	bsr outd
	puls d,pc

outd:  ; prints D in hex.
	pshs b
	bsr outcharhex
	puls a
	; FALL THROUGH

; print the byte in A as a two-character hex value
outcharhex:
	pshs a
        lsra
        lsra
        lsra
        lsra
        bsr outnibble
	puls a
	; FALL THROUGH

; print the nibble in the low four bits of A
outnibble:
        anda #0x0f ; mask off low four bits
        cmpa #9
        ble num    ; less than 10?
        adda #0x07 ; start at 'A' (10+7+0x30=0x41='A')
num:    adda #0x30 ; start at '0' (0x30='0')
        jsr outchar
        rts


; Compiler helpers

	.export _swab

_swab:	ldd	2,s
	exg	a,b
	rts

	.export _memset

_memset:
	pshs	u
	ldu	4,s		; pointer
	ldx	6,s		; count
	beq	nowork
msl:	clr	,u+
	leax	-1,x
	bne	msl
nowork:
	ldd	4,s
	puls	u,pc

	.export _memcpy

_memcpy:
	pshs	u
	ldu	4,s		; dest
	ldx	6,s		; src
	ldy	8,s		; count
	beq	nowork
mcl:	lda	,x+
	sta	,u+
	leay	-1,y
	bne	mcl
	ldd	4,s
	puls	u,pc

	.export _strlen

_strlen:
	ldd	#0xFFFF
	ldx	2,s
next:
	addd	#1
	tst	,x+
	bne	next
	rts
