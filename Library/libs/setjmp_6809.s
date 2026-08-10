; setjmp / longjmp for 6809 FUZIX
; Copyright 2015 Tormod Volden

	; exported
	.export _setjmp
	.code

; int setjmp(jmp_buf)
_setjmp:
	ldd ,s		; return address
	sty ,x++
	stu ,x++
	sts ,x++
	std ,x
	ldx #0
	rts
