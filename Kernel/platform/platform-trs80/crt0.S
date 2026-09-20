	        ; Ordering of segments for the linker.
	        ; WRS: Note we list all our segments here, even though
	        ; we don't use them all, because their ordering is set
	        ; when they are first seen.	
#if 0
	.area _CODE
	.area _CODE2
	.area _VIDEO
	.area _CONST
	.area _INITIALIZED
	.area _DATA
	.area _BSEG
	.area _BSS
	.area _HEAP
	; note that areas below here may be overwritten by the heap at runtime, so
	; put initialisation stuff in here
	.area _GSINIT
	.area _GSFINAL
	; Buffers must be directly before discard as they will
	; expand over it
	.area _BUFFERS
	.area _DISCARD
	.area _COMMONMEM
	.area _COMMONDATA
	; Doesn't matter if these go over the I/O space as they are
	; removed at the end of the build
	.area _INITIALIZER
#endif
       ; startup code
	.code
;
;	Once the loader completes it jumps here
;
start:
	ld	sp, kstack_top
	; move the common memory where it belongs    
	ld	hl, __bss
	ld	de, __common
	ld	bc, __common_size
	ldir
	ld	de, __commondata
	ld	bc, __commondata_size
	ldir
	; then the discard
; Discard can just be linked in but is next to the buffers
; FIXME switch to lddr
	ld	de, __discard
	ld	bc, __discard_size
	ldir
	; then zero the data area
	ld	hl, __bss
	ld	de, __bss + 1
	ld	bc, __bss_size - 1
	ld	(hl), 0
	ldir
;	Zero buffers area
	ld	hl, __buffers
	ld	de, __buffers + 1
	ld	bc, __buffers_size - 1
	ld	(hl), 0
	ldir
	call	init_early
	call	init_hardware
	call	_fuzix_main
	di
stop:	halt
	jr stop
