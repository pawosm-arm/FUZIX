
	.export __stbyte9

	.setcpu 8080
	.code
__stbyte9:
	mov a,l
	lxi h,9

	mov m,a
	mov l,a
	ret