
	.export __stbyte15

	.setcpu 8080
	.code
__stbyte15:
	mov a,l
	lxi h,15

	mov m,a
	mov l,a
	ret