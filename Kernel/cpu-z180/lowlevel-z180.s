# 1 "cpu-z180/lowlevel-z180.S"
# 1 "cpu-z180/../cpu-z80u/lowlevel-z80u.S"
;
;	Common elements of low level interrupt and other handling. We
; collect this here to minimise the amount of platform specific gloop
; involved in a port
;
;	Based upon code (C) 2013 William R Sowerbutts
;

	; debugging aids
	.export outcharhex
	.export outbc
	.export outde
	.export outhl
	.export outnewline
	.export outstring
	.export outstringhex
	.export outnibble

        ; exported symbols
	.export null_handler
	.export unix_syscall_entry
        .export _doexec
	.export nmi_handler
	.export interrupt_legacy
	.export interrupt_handler
	.export synchronous_fault
	.export ___hard_ei
	.export ___hard_di
	.export ___hard_irqrestore
	.export _out
	.export _in
	.export _out16
	.export _in16
	.export _sys_cpu
	.export _sys_cpu_feat
	.export _sys_stubs
	.export _set_cpu_type

	.export mmu_irq_ret
# 1 "cpu-z180/../cpu-z80u/../build/kernelu.def"
; UZI mnemonics for memory addresses etc

; Move down to 0xF600 to fit the monitor in
U_DATA__TOTALSIZE           .equ 0x200        ; 256+256 bytes @ F800
Z80_TYPE                    .equ 2

OS_BANK                     .equ 0x00         ; value from include/kernel.h

; N8VEM Mark IV mnemonics
FIRST_RAM_BANK              .equ 0x80         ; low 512K of physical memory is ROM/ECB window.
Z180_IO_BASE                .equ 0x40
MARK4_IO_BASE               .equ 0x80

; No standard clock speed for the Mark IV board, but this is a common choice.
; disabling this saves around approx 0.5KB

CPU_CLOCK_KHZ               .equ 36864        ; 18.432MHz * 2
Z180_TIMER_SCALE            .equ 20           ; CPU clocks per timer tick
TICKSPERSEC                 .equ 40           ; timer interrupt rate (Hz)

PROGBASE		    .equ 0x0000
PROGLOAD		    .equ 0x0100
# 27
PPIDE_RD_LINE	.equ	0x40
PPIDE_WR_LINE	.equ	0x20
PPIDE_PPI_BUS_READ	.equ	0x92
PPIDE_PPI_BUS_WRITE	.equ	0x80

ppi_port_a	.equ	0x88
ppi_port_b	.equ	0x89
ppi_port_c	.equ	0x8A
ppi_control	.equ	0x8B

ppide_data	.equ	0x08
# 42 "cpu-z180/../cpu-z80u/lowlevel-z80u.S"
	; This up and down makes sure we can include it from cpu-z180.
# 1 "cpu-z180/../cpu-z80u/../cpu-z80u/kernel-z80.def"
 
# 26
 
# 44
 
# 56 "cpu-z180/../cpu-z80u/lowlevel-z80u.S"
	.common
;
;	Called on the user stack in order to process signals that
;	are pending. A user process can longjmp out of this loop so
;	care is needed. Call with interrupts disabled and user mapped.
;
;	Returns with interrupts disabled and user mapped, but may
;	enable interrupts and change mappings.
;
deliver_signals:
	; Pending signal
	ld a, (_udata + 17    )
	or a
	ret z

deliver_signals_2:
	ld l, a
	ld h, 0
	push hl		; signal number as C argument to the handler

	; Handler to use
	add hl, hl
	ld de, _udata + 34    
	add hl, de
	ld e, (hl)
	inc hl
	ld d,(hl)

	ld bc, signal_return
	push bc		; bc is passed in as the return vector

	ld c,a		; Save signal number in C as well for 8080 style

	; Indicate processed
	xor a
	ld (_udata + 17    ), a
	; and we will handle the signal with interrupts on so clear the
	; flag
	ld (_int_disabled),a

	; Semantics for now: signal delivery clears handler
	ld (hl), a
	dec hl
	ld (hl), a


	ei



	ld hl,(PROGLOAD+16); return to user space. This will then return via
			; the return path handler stacked above via BC
	jp (hl)
;
;	Syscall signal return path
;
signal_return:
	pop hl		; argument
	di



	;
	;	We must keep IRQ disabled in the kernel mapped
	;	element of this processing, as we don't want to
	;	set INSYS flags here.
	;
	ld (_udata + 8     ), sp
	ld sp, kstack_top
	;
	;	Ensure chksigs and friends see the right status
	;
	ld a,1
	ld (_int_disabled),a
	call map_kernel_di
	
	call _chksigs
	
	call map_proc_always_di
	ld sp, (_udata + 8     )
	jr deliver_signals


;
;	Syscall processing path
;
unix_syscall_entry:
	; We know the previous state was EI and that we won't do anything
	; clever until we EI again, so we can avoid the helpers on the fast
	; path.
        di
        ; store processor state. We destroy AF, AF', DE, HL
        exx
        push bc
        push de
        push hl
        exx
	push bc
        push ix
        push iy

        ; locate function call arguments on the userspace stack
        ld hl, 16     ; 12 bytes machine state, plus 2 bytes return address x 2
        add hl, sp
# 164
        ; save system call number
        ld (_udata + 7     ), a
        ; advance to syscall arguments
        ; copy arguments to common memory
        ld de, _udata + 18    

	ldi
	ldi
	ldi
	ldi
	ldi
	ldi
	ldi
	ldi

	ld a, 1
	ld (_udata + 6     ), a

        ; save process stack pointer
        ld (_udata + 8     ), sp
        ; switch to kernel stack
        ld sp, kstack_top

        ; map in kernel keeping common
	call map_kernel_di

        ; re-enable interrupts
        ei

	
        ; now pass control to C
        call _unix_syscall
	

	;
	; WARNING: There are two special cases to beware of here
	; 1. fork() will return twice from _unix_syscall
	; 2. execve() will not return here but will hit _doexec()
	;
	; The fork case returns with a different U_DATA mapped so the
	; U_DATA referencing code is fine, but globals are usually not

        di	; Again we know we won't mess up calling di/ei directly


	; FIXME: another spot we di but have the flag wrong and call stuff
	; although we probably just need a rule that _di versions can't
	; rely on it!
	call map_proc_always_di

	xor a
	ld (_udata + 6     ), a

	; Back to the user stack
	ld sp, (_udata + 8     )

	ld hl, (_udata + 12    )
	ld de, (_udata + 10    )

	ld a, (_udata + 17    )
	or a

	; Fast path the normal case
	jr nz, via_signal

	; Restore stacks and go
	;
	; Should we change the ABI and just return in DE/HL ?
	;
unix_return:
	ld a, h
	or l
	jr z, not_error
	scf		; carry flag on return state for errors
	jr unix_pop

not_error:
	ex de, hl	; return the retval instead
	;
	; Undo the stacking and go back to user space
	;
unix_pop:



        ; restore machine state
        pop iy
        pop ix
	pop bc
        exx
        pop hl
        pop de
        pop bc
        exx
        ei
        ret ; must immediately follow EI


via_signal:
	; Get off the kernel syscall stack before we start signal
	; handling. Our signal handlers may themselves elect to make system
	; calls. This means we must also save the error/return code
	ld hl, (_udata + 12    )
	push hl
	ld hl, (_udata + 10    )
	push hl

	; Signal processing. This may longjmp back into userland
	call deliver_signals_2

	; If not then we recover the syscall return values and
	; exit via the syscall return path
	pop de			; retval
	pop hl			; errno
	jr unix_return


;
;	Final component of execve()
;
_doexec:
        di
        call map_proc_always_di

        pop hl ; return address



        pop de ; start address

        ld hl, (_udata + 26    )
        ld sp, hl      ; Initialize user stack, below main() parameters and the environment

        ; u_data.u_insys = false
        xor a
        ld (_udata + 6     ), a

        ex de, hl
# 306
	; for the relocation engine - tell it where it is
	; we always start in the low 256 bytes of our binary so we can
	; just generate the relocation base accordingly
	ld d,h
	ld e,0
        ei
        jp (hl)

;
;	We took a NULL pointer - either a branch to NULL or on Z180
;	an illegal. For now use a blunt instrument. We could in theory
;	do a nicer signal throw etc.
;
null_handler:
	di
	; kernel jump to NULL is bad
	ld a, (_udata + 6     )
	or a
	jp nz, trap_illegal
	ld a, (_udata + 16    )
	or a
	jp nz, trap_illegal
	; user is merely not good - handle it synchronously
	ld hl,9		; SIGKILL
synchronous_fault:
	ld sp,kstack_top
	call map_kernel_di
	push hl
	
	call _doexit
	
trap_illegal:
        ld hl, illegalmsg
traphl:
        call outstring
        call _plt_monitor

illegalmsg: .ascii "[illegal]"
.byte 13, 10, 0

nmimsg: .ascii "[NMI]"
.byte 13,10,0

nmi_handler:



	call map_kernel_di
        ld hl, nmimsg
	jr traphl

;
;	Interrupt handler. Not quite the same as syscalls, we need to
;	stack everything and we must get off the IRQ stack and then
;	process need_resched and signals
;
interrupt_legacy:
# 371
interrupt_handler:
        ; store machine state
        ex af,af'
        push af
intvec:
        ex af,af'
        exx
        push bc
        push de
        push hl
        exx
        push af
        push bc
        push de
        push hl
        push ix
        push iy
	;
	; This is a bit exciting - if our MMU enforces r/o then the entire
	; stack state might be bogus!
	;
# 396
mmu_irq_ret:

	; Some platforms (MSX for example) have devices we *must*
	; service irrespective of kernel state in order to shut them
	; up. This code must be in common and use small amounts of stack
	call plt_interrupt_all
	; FIXME: add profil support here (need to keep profil ptrs
	; unbanked if so ?)
# 416
	; Get onto the IRQ stack
	ld (istack_switched_sp), sp
	ld sp, istack_top

	call map_save_kernel

	ld a,1
	; So we know that this task should resume with IRQs off
	ld (_udata + 16    ), a
	; Load the interrupt flag properly. It got an implicit di from
	; the IRQ being taken
	ld (_int_disabled),a

	; 	C temporaries
	ld hl,(__tmp)
	push hl
	ld hl,(__hireg)
	push hl
	ld hl,(__tmp2)
	push hl
	ld hl,(__tmp2+2)
	push hl
	ld hl,(__tmp3)
	push hl
	ld hl,(__tmp3+2)
	push hl
	ld hl,(__retaddr)
	push hl

	
	call _plt_interrupt
	

	pop hl
	ld (__retaddr),hl
	pop hl
	ld (__tmp3+2),hl
	pop hl
	ld (__tmp3),hl
	pop hl
	ld (__tmp2+2),hl
	pop hl
	ld (__tmp2),hl
	pop hl
	ld (__hireg),hl
	pop hl
	ld (__tmp),hl

	ld a, (_need_resched)
	or a
	jr nz, preemption

	; Back to the old memory map
	call map_restore

	;
	; Back on user stack
	;
	ld sp, (istack_switched_sp)

intout:
	xor a
	ld (_udata + 16    ), a
	;
	;	Z180 internal interrupts do not reti
	;
# 487
	ld hl, intret
	push hl
	reti			; We have now 'left' the interrupt
				; and the controllers have seen the
				; reti M1 cycle. However we still
				; have DI set
intret:
	di
	ld a, (_udata + 6     )
	or a
	jr nz, interrupt_pop


	ld a,(0)
	cp #0xC3
	jp nz, null_pointer_trap
	; Loop through any pending signals. These could longjmp out
	; of the handler so ensure everything is fixed before this !


	call deliver_signals

	; Then unstack and go.
interrupt_pop:
	xor a
	ld (_int_disabled),a



        pop iy
        pop ix
        pop hl
        pop de
        pop bc
        pop af
        exx
        pop hl
        pop de
        pop bc
        exx
        ex af, af'
        pop af
        ex af, af'
        ei			; Must be instruction before ret
	ret			; runs in the ei interrupt shadow

;
;	At the point we fire we are back on the user stack and logically
;	speaking have just finished the interrupt. If the low byte was
;	corrupt we assume the worst and just blow the process away
;
null_pointer_trap:
	ld sp,kstack_top	; Need to be off the user stack
				; can't use the interrupt stack as we might
				; IRQ during this
	call map_kernel_di	; to deliver the kill
	ld a, 0xC3		; Repair
	ld (0), a
	ld hl, 9		; SIGKILL (take no prisoners here)
trap_signal:
	push hl
	ld hl,(_udata + 0     )
	push hl
	
        call _ssig
	
	; Now fall into pre-emption from which we will not return
;
;	Pre-emption. We need to get off the interrupt stack, switch task
;	and clean up the IRQ state carefully
;

	.export preemption

preemption:
	xor a
	ld (_need_resched), a	; Task done

	; Back to the old memory map
	call map_restore

	ld hl, (istack_switched_sp)
	ld (_udata + 8     ), hl

	ld sp, kstack_top	; We don't pre-empt in a syscall
				; so this is fine
# 578
	ld hl, intret2
	push hl
	reti			; We have now 'left' the interrupt
				; and the controllers have seen the
				; reti M1 cycle. However we still
				; have DI set
	di			; see undocumented Z80 notes on RETI
	;
	; We are now on the syscall stack (which is fine, we don't
	; pre-empt mid syscall so therefore it is free.  We will now
	; task switch. The process being pre-empted will disappear into
	; switchout() and whoever is next will come out of the same -
	; hence the need to reti

	;
intret2:call map_kernel_di
	;
	; Semantically we are doing a null syscall for pre-empt. We need
	; to record ourselves as in a syscall so we can't be recursively
	; pre-empted when switchout re-enables interrupts.
	;
	ld a, 1
	ld (_udata + 6     ), a
	;
	; Check for signals
	;
	
	call _chksigs
	
	;
	; Process status is offset 0
	;
	ld hl, (_udata + 0     )
	ld a,1           
	cp (hl)
	jr nz, not_running
	ld (hl), 2           
	inc hl
	set #2	        ,(hl)
not_running:
	
	call _plt_switchout
	
	;
	; We are no longer in an interrupt or a syscall
	;
	xor a
	ld (_udata + 16    ), a
	ld (_udata + 6     ), a
	;
	; We have been rescheduled, remap ourself and go back to user
	; space via signal handling
	;
	call map_proc_always_di ; Get our user mapping back


	; We were pre-empted but have now been rescheduled
	; User stack
	ld sp, (_udata + 8     )
	ld a, (_udata + 17    )
	or a
	call nz, deliver_signals_2
	;
	; pop the stack and go
	;



	jp interrupt_pop

;
;	Debugging helpers
;

	.common

; outstring: Print the string at (HL) until 0 byte is found
; destroys: AF HL
outstring:
        ld a, (hl)     ; load next character
        and a          ; test if zero
        ret z          ; return when we find a 0 byte
        call outchar
        inc hl         ; next char please
        jr outstring

; print the string at (HL) in hex (continues until 0 byte seen)
outstringhex:
        ld a, (hl)     ; load next character
        and a          ; test if zero
        ret z          ; return when we find a 0 byte
        call outcharhex
        ld a, 0x20 ; space
        call outchar
        inc hl         ; next char please
        jr outstringhex

; output a newline
outnewline:
        ld a, 0x0d  ; output newline
        call outchar
        ld a, 0x0a
        jp outchar

outhl:  ; prints HL in hex.
	push af
        ld a, h
        call outcharhex
        ld a, l
        call outcharhex
	pop af
        ret

outbc:  ; prints BC in hex.
	push af
        ld a, b
        call outcharhex
        ld a, c
        call outcharhex
	pop af
        ret

outde:  ; prints DE in hex.
	push af
        ld a, d
        call outcharhex
        ld a, e
        call outcharhex
	pop af
        ret

; print the byte in A as a two-character hex value
outcharhex:
        push bc
	push af
        ld c, a  ; copy value
        ; print the top nibble
        rra
        rra
        rra
        rra
        call outnibble
        ; print the bottom nibble
        ld a, c
        call outnibble
	pop af
        pop bc
        ret

; print the nibble in the low four bits of A
outnibble:
        and #0x0f ; mask off low four bits
        cp #10
        jr c, numeral ; less than 10?
        add a, 0x07 ; start at 'A' (10+7+0x30=0x41='A')
numeral:add a, 0x30 ; start at '0' (0x30='0')
        jp outchar

;
;	I/O helpers for cases we don't use peepholes to write them out
;
;	Must not trash BC
;
;	out(addr, val)		- not the heathen x86 version
;
_out:
_out16:
	push	bc



	ld	hl,4

	add	hl,sp
	ld	c,(hl)
	inc	hl
	ld	b,(hl)
	inc	hl
	ld	e,(hl)
	out	(c),e
	pop	bc
	ret

;
;	Read a port
;
_in16:
_in:
	push	bc



	ld	hl,4

	add	hl,sp
	ld	c,(hl)
	inc	hl
	ld	b,(hl)
	in	l, (c)
	ld	h,0
	pop	bc
	ret

;
;	Deal with all the NMOS Z80 bugs and the buggy emulators by
;	simply tracing our own interrupt status. It's cheaper this way
;	but does mean any code that is using di and friends directly needs
;	to be a lot more careful. We can also make irqflags_t 8bit and
;	fastcall the irqrestore later on FIXME
;
___hard_ei:
	xor a
	ld (_int_disabled),a
	ei
	ret

___hard_di:
	ld hl,_int_disabled
	di
	ld a,(hl)
	ld (hl),1
	ld l,a
	ret

___hard_irqrestore:



	ld hl,2

	add hl,sp
	ld a,(hl)
	di
	ld (_int_disabled),a
	or a
	ret nz
	ei
	ret

	.literal

_sys_stubs:
	jp unix_syscall_entry
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	.data

_sys_cpu:
	.byte 0
_sys_cpu_feat:
	.byte 0

	.discard

_set_cpu_type:
	ld h,2		; Assume Z80
	xor a
	dec a
	daa
	jr c,is_z80
	ld h,6		; Nope Z180
is_z80:
	ld l,1		; 8080 family
	ld (_sys_cpu),hl	; Write cpu and cpu feat
	ret
# 859
	.code


;
;	Private bank correct helpers
;
;
		.export _memcpy
_memcpy:
		push	bc



		ld	hl,9

		add	hl,sp
		ld	b,(hl)	; Count
		dec	hl
		ld	c,(hl)
		dec	hl
		ld	d,(hl)	; Source
		dec	hl
		ld	e,(hl)
		dec	hl
		ld	a,(hl)	; Destination
		dec	hl
		ld	l,(hl)
		ld	h,a

		ld	a,b
		or	c
		jr	z,done

		push	hl
		ex	de,hl
		ldir
		pop	hl
done:
		pop	bc
		ret

;
;	Memset
;
;	TODO: rewrite into Z80 style with a set and copy
;
		.export _memset
_memset:
		push	bc



		ld	hl,4		; Allow for the push of BC

		add	hl,sp
		ld	e,(hl)
		inc	hl
		ld	d,(hl)		; Pointer
		push	de		; Return is the passed pointer
		inc	hl
		ld	a,(hl)		; fill byte
		inc	hl		; skip fill high
		inc	hl
		ld	c,(hl)
		inc	hl
		ld	b,(hl)		; length into BC

		ld	l,a		; We need to free up A for the loop check
		ex	de,hl		; now have HL as the pointer and E as the fill byte
		jp	loopin

loop:
		ld	(hl),e
		inc	hl
		dec	bc
loopin:
		ld	a,b
		or	c
		jr	nz,loop
		pop	hl		; Address passed in
		pop	bc		; Restore BC
		ret

		.export _strlen
_strlen:



		ld	hl,2		; Allow for the push of BC

		add	hl,sp
		ld	e,(hl)
		inc	hl
		ld	d,(hl)		; Pointer
		ex	de,hl
		xor	a
		ld	de, 0xFFFF
sloop:		inc	de
		cp	(hl)
		inc	hl
		jr	nz, sloop
		ex	de,hl
		ret

;
;	memcpy
;
		.export _memcmp
_memcmp:
		push	bc



		ld	hl,9

		add	hl,sp
		ld	b,(hl)	; Count
		dec	hl
		ld	c,(hl)
		dec	hl
		ld	d,(hl)	; Source 1
		dec	hl
		ld	e,(hl)
		dec	hl
		ld	a,(hl)	; Source 2
		dec	hl
		ld	l,(hl)
		ld	h,a

next:
		ld	a,b
		or	c
		jr	z,ret0

		ld	a,(de)		; get src 2
		cp	(hl)		; check v src 1
		jr	nz, mismatch	; and C if src2 < src1
		inc	de
		inc	hl
		dec	bc
		jr	next
mismatch:
		ld	hl,1
		jr	c, mis_low
		ld	hl,-1
mis_low:
		pop	bc
		ret
ret0:		ld	hl,0
		pop	bc
		ret
