; fzload09 - Fuzix loader for Dragon and CoCo
;
; SPDX-FileCopyrightText: Copyright 2025-2026 Ciaran Anscomb
; SPDX-License-Identifier: GPL-2.0-or-later

; BOOT will load 16 sectors of data (LSNs 2--17) from $2600+.  The first
; part of the payload can follow the loader code.  From then on, we read a
; sector (256 bytes) at a time from LSN 18+ using the configured driver.
;
; The payload is in CoCo DECB binary format.  In order to populate a whole
; 64K map, we first ensure our loader is running outside the first four
; pages (relocate to $8000-$bfff).  Then for each DECB chunk, we map the
; appropriate page to $4000-$7fff and copy data into the mapped area.
;
; Assuming that RAM below $0200 is free in the target map, we use that
; area for video while the loader runs.  When we read an EXEC chunk, we
; copy a small bounce routine to this area that resets the page mapping
; and jumps to the payload's EXEC address.

; Select ONE driver:
;
; Define DRV_DRAGONDOS=1 to load from a DragonDOS floppy controller.
;
; Define DRV_COCOSDC=1 to load from CoCoSDC.

; Define COCO=1 when building for the CoCo 3.  Page numbers are doubled
; up, and the DOS command loads 18 sectors from track 34 (LSNs 612--629),
; but otherwise it operates in exactly the same way.  In particular, once
; data from the boot track is exhausted, it's expected that the rest of
; the data will be in in the same place at the beginning of the disk as it
; would be for DragonDOS (starting at LSN 20, as we already have two more
; sectors of data).  Defining COCO also changes the register layout used
; by the CoCoSDC driver.

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

 ifndef COCO

; Dragon: BOOT will have loaded 16 sectors, continue from LSN 18.
btrk_nsecs	equ 16
next_lsn	equ 18

 else

; CoCo: DOS will have loaded 18 sectors, continue from LSN 20.
btrk_nsecs	equ 18
next_lsn	equ 20

 endif

; Screen addresses while loading
con_top		equ $0000
con_end		equ $0200

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

; First stage.  This is executed by BOOT (or DOS).

	org $2600
	fcc /OS/	; BOOT magic
	
	orcc #$50	; mask interrupts
	lda #$ff
	tfr a,dp
	setdp $ff
	clr $0071	; cold boot on reset
	sta $ffdf	; 64K mode

 ifdef COCO
	; video base address = 0
	sta $ffc6
	sta $ffc8
	; task 1 memory map, and area 2 of task 0
	ldd #$0001
	std $ffa8
	ldd #$0203
	std $ffaa
	ldd #$0809
	std $ffa4
	std $ffac
	ldd #$0607
	std $ffae
 else
	; video base address = 0
	clra
	clrb
	std $ff38
	; task 1 memory map, and area 2 of task 0
	ldb #$01
	std $ff34
	ldd #$0403
	sta $ff32
	std $ff36
 endif

	; relocate rest of loader to $8000+ as page 4
	ldx #reloc_start
	ldu #$8000
@l0	lda ,x+
	sta ,u+
	cmpx #reloc_end
	blo @l0
	; jump to newly-relocated loader...
	jmp $8000

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

; Second stage.  Copied to $8000+ as page 4 by the first stage.  This now
; reads in the DECB data and positions it within pages 0--3, mapping to
; $4000--$7fff as a work area.

reloc_start

	lds #$c000

	leax sector_buf,pcr
	stx buf_ptr,pcr
	ldx #next_lsn
	lbsr devopen

 ifdef COCO
	lda #1
	sta $ff91	; task 1
 else
	sta $ffd5	; task 1
 endif

	lbsr con_cls
	leax msg_loading,pcr
	lbsr con_strout

	; read in decb stream
read_loop
	bsr read_byte
	beq data_chunk
	cmpb #$ff	; EXEC chunk?
	lbeq exec_chunk
	; report error and halt
	lbsr con_cls
	leax msg_fmt_err,pcr
	lbra error
@l0	bra @l0	; error - infinite loop

; Various messages
msg_loading
	fcb $0C,$0F,$01,$04,$09,$0E,$07,$20,$06,$15,$1A,$09,$18,$00	; fci /LOADING FUZIX/,0
msg_fmt_err
	fcb $0A,$06,$0F,$12,$0D,$01,$14,$20,$05,$12,$12,$0F,$12,$00	; fci 10,/FORMAT ERROR/,0

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

; DATA chunk.

data_chunk
	lda #$2e
	lbsr con_chrout	; print a '.'
	lbsr con_anim_crs
	bsr read_word	; read chunk length
	tfr d,y		; y = chunk length
	bsr read_word	; read chunk destination address
	clr ,-s
	lsla
	rol ,s
	lsla
	rol ,s
	lsra
	lsra
	ora #$40
	tfr d,x		; x = dst addr translated to work area
@l10	lda ,s
 ifdef COCO
	lsla
	sta $ffaa
	inca
	sta $ffab	; work area = dst page
 else
	sta $ff35	; work area = dst page
 endif
@l20	bsr read_byte
	stb ,x+
	dec con_crs_timer,pcr
	bne @l30
	bsr con_anim_crs
@l30	leay -1,y
	beq @l40
	cmpx #$8000
	blo @l20
	ldx #$4000	; continue at beginning of work area
	inc ,s		; increment work page
	bra @l10
@l40	leas 1,s
	bra read_loop

con_crs_timer
	fcb 0

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

; If no bytes left in buffer, read the next sector.  Returns next byte in
; buffer in B.

read_word
	bsr read_byte
	tfr b,a
	; fall through
read_byte
	pshs x
	ldx buf_nbytes,pcr
	bne fetch_byte
	; read next lsn
	leax sector_buf,pcr
	stx buf_ptr,pcr
	lbsr devread
	; x = x - sector_buf (hoop-jumping pic version)
	pshs d
	tfr x,d
	subd buf_ptr,pcr
	tfr d,x
	puls d
fetch_byte
	leax -1,x
	stx buf_nbytes,pcr
	ldx buf_ptr,pcr
	ldb ,x+
	stx buf_ptr,pcr
	tstb
	puls x,pc

; Buffer handling
buf_ptr	fdb $0000	; initialised to sector_buf
buf_nbytes
	fdb track_nbytes

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

; EXEC chunk.
;
; Sets up a routine in addresses < $0100 that:
;
; - resets $4000-$7fff (work area) to page 1
; - resets $8000-$bfff (loader execution area) to page 2
; - JMPs to the EXEC address in the chunk

exec_chunk	
	bsr read_word	; skip 2 bytes
	bsr read_word	; EXEC address
	std $00fe
	lbsr devclose
	clra
	tfr a,dp
	setdp $00
	lda #$7e	; "JMP ext"
	sta $00fd	; 00fd| JMP exec_addr
 ifdef COCO
	ldd #$0203
	std $ffaa	; work area -> page 1
	ldd #$ccfd	; "LDD imm", "STD ext"
	sta $00f7
	stb $00fa
	ldd #$0405
	std $00f8	; 00f7| LDD #$0405
	ldd #$ffac
	std $00fb	; 00fa| STD $ffac
	jmp $00f7
 else
	ldd #$86b7	; "LDA imm", "STA ext"
	sta $00f8
	stb $00fa
	ldd #$0102
	sta $ff35	; work area -> page 1
	stb $00f9	; 00f8| LDA #$02
	ldd #$ff36
	std $00fb	; 00fa| STD $ff36
	jmp $00f8
 endif
	setdp $ff

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

; Just enough console functionality to print messages and animate a
; spinning cursor.

; animate cursor
con_anim_crs
	pshs a,x
	inc con_crs_idx,pcr
	lda con_crs_idx,pcr
	anda #3
	leax con_crs,pcr
	lda a,x
	sta [con_pos,pcr]
	puls a,x,pc

; print error and halt
error	bsr con_strout
@l0	bra @l0

; print string
; entry: x = string
; exit: x = byte after nul-terminator
con_strout
	pshs a
@l10	lda ,x+
	beq @l20
	bsr con_chrout
	bra @l10
@l20	puls a,pc

; print character
; destroyed: a (possibly)
con_chrout
	pshs x
	ldx con_pos,pcr
	cmpa #$0a
	beq @l10
	sta ,x+
	bra @l20
	; newline
@l10	bsr con_clr_eol_x
@l20	cmpx #con_end
	bhs @l30
	stx con_pos,pcr
	puls x,pc
; scroll up
; destroyed: a
con_scrup
	pshs x
@l30	ldx #con_top
@l40	lda 32,x
	sta ,x+
	cmpx #con_end-32
	blo @l40
	stx con_pos,pcr
	bsr con_clr_eol_x
	puls x,pc

; clear to end of line
; exit: X = byte after current line
; destroyed: a
con_clr_eol
	ldx con_pos,pcr
	; fall through
; entry: X = con_pos
con_clr_eol_x
	pshs b
@l0	lda #$20
	sta ,x+
	tfr x,d
	bitb #$1f
	bne @l0
	puls b,pc

; clear screen
; destroyed: a
con_cls
	pshs x
	ldx #con_top
	stx con_pos,pcr
	lda #$20
@l0	sta ,x+
	cmpx #con_end
	blo @l0
	puls x,pc

; Animated cursor: / - \ !
con_crs	fcb $2f,$2d,$1c,$21

; Console variables
con_pos	fdb $0000
con_crs_idx
	fcb 0

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

; Miscellaneous

set_nmi_handler
	pshs a
	stx $010a
	lda #$7e
	sta $0109
	puls a,pc

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	; Include selected driver code

 ifdef DRV_COCOSDC
	include "fzload09-cocosdc.s"
 else
 ifdef DRV_DRAGONDOS
	include "fzload09-dragondos.s"
 else
devopen
devclose
devread
	assert 0,"No driver configured"
 endif
 endif

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

; Although it wastes a bit of space, it's nice if the kernel ends up
; sector-aligned in the disk image.
	align 256,0

sector_buf

track_nbytes	equ $2600+256*btrk_nsecs-*

reloc_end	equ *+track_nbytes
