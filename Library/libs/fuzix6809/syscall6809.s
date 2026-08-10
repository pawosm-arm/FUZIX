	.export __syscall

	.code
; FIXME: stop using swi swap X and D return

__syscall:
	swi
	cmpd #0			; D holds errno, if any
	beq noerr1
	std _errno		; X is -1 in this case
noerr1:
	tfr x,d
	rts
