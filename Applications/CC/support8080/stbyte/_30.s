
	.export __stbyte30

	.setcpu 8080
	.code
__stbyte30:
	mov a,l
	lxi h,30

	mov m,a
	mov l,a
	ret