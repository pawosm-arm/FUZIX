### Fuzix build rules for Dragon + SAMx8 platform

## - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
## Common platform sources

# Steal what we can from dragon-nx32 and coco3
vpath %.s ../platform-dragon-nx32:../platform-coco3
vpath %.c ../platform-dragon-nx32:../platform-coco3

# C sources (build in place)
CSRCS = devices.c
CSRCS += main.c
CSRCS += devfd.c devinput.c devtty.c devlpr.c
CSRCS += libc.c

# C sources for discard segment (build in place)
CDSRCS = discard.c

# Video-related C sources (build in place)
VSRCS = vc.c

# C sources (build here from ../../dev)
DSRCS =

# C sources for discard segment (build here from ../../dev)
DDSRCS =

# Assembly sources (build in place).  No need to break out by segment, as this
# can be specifyied in the source file.
ASRCS = crt0.s dragon_samx8.s
ASRCS += commonmem.s mem_samx8.s tricks.s usermem_sam.s
ASRCS += multihead.s vc_asm.s video.s
ASRCS += floppy.s

EXTRA_ASRCS =

## - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
## Targets

DISKIMAGES = diskimage-ddos

ifndef SUBTARGET
SUBTARGET = sdc
endif

# Real hardware - CoCoSDC
ifeq ($(SUBTARGET),sdc)
ifndef WANT_COCOSDC
WANT_COCOSDC = yes
endif
DISKIMAGES += diskimage-sdc
endif

# Real hardware - IDE (untested)
ifeq ($(SUBTARGET),ide)
ifndef WANT_COCOIDE
WANT_COCOIDE = yes
endif
ifndef WANT_DRIVEWIRE
WANT_DRIVEWIRE = yes
endif
DISKIMAGES += diskimage-ide
endif

# Emulation - IDE and becker port
ifeq ($(SUBTARGET),emu)
ifndef WANT_COCOIDE
WANT_COCOIDE = yes
endif
ifndef WANT_DRIVEWIRE
WANT_DRIVEWIRE = yes
endif
ifndef WANT_BECKER
WANT_BECKER = yes
endif
endif

## - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
## Driver sources

# CoCoSDC support (tinydisk)

ifeq ($(WANT_COCOSDC),yes)
CSRCS += devsdc.c
ASRCS += sdc.s
CROSS_CCOPTS += -DCONFIG_COCOSDC
WANT_TINYDISK = yes
endif

# IDE support (tinydisk)

ifeq ($(WANT_COCOIDE),yes)
EXTRA_ASRCS += ../platform-coco3/ide.s
DSRCS += ../../dev/tinyide.c
DDSRCS += ../../dev/tinyide_discard.c
CROSS_CCOPTS += -DCONFIG_TD_IDE -DCONFIG_COCOIDE
WANT_TINYDISK = yes
endif

# Tinydisk support

ifeq ($(WANT_TINYDISK),yes)
ASRCS += td_dragon.s
DSRCS += ../../dev/tinydisk.c
DDSRCS += ../../dev/tinydisk_discard.c
CROSS_CCOPTS += -DCONFIG_TINYDISK
endif

# DriveWire support

ifeq ($(WANT_DRIVEWIRE),yes)
CSRCS += ttydw.c
DSRCS += ../../dev/devdw.c
EXTRA_ASRCS += ../../dev/drivewire/drivewire.s
CROSS_CCOPTS += -DCONFIG_DRIVEWIRE
endif

## - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
## Build rules

COBJS = $(CSRCS:.c=.o)
CDOBJS = $(CDSRCS:.c=.o)
VOBJS = $(VSRCS:.c=.o)
AOBJS = $(ASRCS:.s=.o)
DOBJS = $(patsubst %.c,%.o,$(notdir $(DSRCS)))
DDOBJS = $(patsubst %.c,%.o,$(notdir $(DDSRCS)))
EXTRA_OBJS = $(patsubst %.s,%.o,$(notdir $(EXTRA_ASRCS)))
OBJS  = $(COBJS) $(CDOBJS) $(VOBJS) $(AOBJS) $(DOBJS) $(DDOBJS) $(EXTRA_OBJS)

CROSS_CCOPTS += -I../../dev/ -I../platform-dragon-nx32/ -I../platform-coco3/

.PHONY: all
all: $(OBJS)
	echo TARGET=$(TARGET) SUBTARGET=$(SUBTARGET) OBJS=$(OBJS)

$(COBJS): %.o: %.c
	$(CROSS_CC) $(CROSS_CCOPTS) $(CROSS_CC_SEG2) -c $<

$(CDOBJS): %.o: %.c
	$(CROSS_CC) $(CROSS_CCOPTS) $(CROSS_CC_SEGDISC) -c $<

$(VOBJS): %.o: %.c
	$(CROSS_CC) $(CROSS_CCOPTS) $(CROSS_CC_VIDEO) -c $<

$(DOBJS): %.o: ../../dev/%.c
	$(CROSS_CC) $(CROSS_CCOPTS) $(CROSS_CC_SEG2) -c $<

$(DDOBJS): %.o: ../../dev/%.c
	$(CROSS_CC) $(CROSS_CCOPTS) $(CROSS_CC_SEGDISC) -c $<

$(AOBJS): %.o: %.s
	$(CROSS_AS) $(ASOPTS) $< -o $@

## - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
## Special build rules

ide.o: ../platform-coco3/ide.s
	$(CROSS_AS) $(ASOPTS) $< -o $@

ifeq ($(WANT_BECKER),yes)
drivewire.o: ASOPTS += --defsym BECKER=1 --defsym BCKSTAT=0xff49 --defsym BCKPORT=0xff4a
endif

drivewire.o: ../../dev/drivewire/drivewire.s
	$(CROSS_AS) $(ASOPTS) $< -o $@

## - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
## Kernel image

# Base
KOBJS = ../../start.o ../../version.o ../../cpu-6809/lowlevel-6809.o

# Memory
KOBJS += ../../bank16k.o ../../memalloc_none.o ../../mm.o ../../process.o \
	../../swap.o ../../usermem.o

# IO
KOBJS += ../../devinput.o ../../devio.o ../../devsys.o ../../kdata.o \
	../../timer.o ../../tty.o ../../vt.o ../../font8x8.o

# The rest (filesystem, syscalls)
KOBJS += ../../blk512.o ../../filesys.o ../../inode.o \
	../../syscall_exec.o ../../syscall_exec16.o ../../syscall_fs.o \
	../../syscall_fs2.o ../../syscall_fs3.o ../../syscall_other.o \
	../../syscall_proc.o

../../fuzix.bin: $(KOBJS) $(OBJS)
	$(CROSS_LD) -o $@ -Map=../../fuzix.map --script=fuzix.link --oformat=decb \
	$(KOBJS) $(OBJS)
	../../tools/pad256 ../../fuzix.bin
	../../tools/lw-checkmap ../../fuzix.map
	echo TARGET=$(TARGET) SUBTARGET=$(SUBTARGET)

## - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
## Disk images

IMAGES = $(FUZIX_ROOT)/Images/$(TARGET)

CLEAN_FZLOAD09 =

fzload09-sdc.bin: fzload09.s
	lwasm -r -DDRV_COCOSDC=1 -o$@ $<

fzload09-sdc.img: fzload09-sdc.bin ../../fuzix.bin
	cat fzload09-sdc.bin ../../fuzix.bin > $@

fzload09-sdc.dsk: fzload09-sdc.img
	dd if=/dev/zero of=$@ bs=256 count=720 conv=sync
	dd if=$< of=$@ bs=256 seek=2 conv=sync,notrunc

CLEAN_FZLOAD09 += fzload09-sdc.bin fzload09-sdc.img fzload09-sdc.dsk

fzload09-emu.bin: fzload09.s
	lwasm -r -DDRV_DRAGONDOS=1 -o$@ $<

fzload09-emu.img: fzload09-emu.bin
	cat fzload09-emu.bin ../../fuzix.bin > $@

fzload09-emu.dsk: fzload09-emu.img
	dd if=/dev/zero of=$@ bs=256 count=720 conv=sync
	dd if=$< of=$@ bs=256 seek=2 conv=sync,notrunc

CLEAN_FZLOAD09 += fzload09-emu.bin fzload09-emu.img fzload09-emu.dsk

.PHONY: image
image: ../../fuzix.bin

.PHONY: diskimage-sdc
diskimage-sdc: fzload09-sdc.dsk
	# Make a blank disk image
	dd if=/dev/zero of=$(IMAGES)/fuzix-sdc.img bs=512 count=39360
	# Copy boot floppy image to the very beginning
	dd if=fzload09-sdc.dsk of=$(IMAGES)/fuzix-sdc.img bs=256 conv=sync,notrunc
	# Add partition table (overwrite first 512 bytes of boot floppy image)
	dd if=$(FUZIX_ROOT)/Standalone/filesystem-src/parttab.20M of=$(IMAGES)/fuzix-sdc.img bs=512 count=1 conv=sync,notrunc
	# Add the filesystem
	dd if=$(IMAGES)/filesys8.img of=$(IMAGES)/fuzix-sdc.img bs=512 seek=2048 conv=notrunc

.PHONY: diskimage-ide
diskimage-ide:
	# Make a blank disk image
	dd if=/dev/zero of=$(IMAGES)/fuzix-ide.img bs=512 count=39360
	# Add partition table (overwrite first 512 bytes of boot floppy image)
	dd if=$(FUZIX_ROOT)/Standalone/filesystem-src/parttab.20M of=$(IMAGES)/fuzix-ide.img bs=512 count=1 conv=sync,notrunc
	# Add the filesystem
	dd if=$(IMAGES)/filesys8.img of=$(IMAGES)/fuzix-ide.img bs=512 seek=2048 conv=notrunc

.PHONY: diskimage-ddos
diskimage-ddos: fzload09-emu.dsk
	# Just copy it
	dd if=$< of=$(IMAGES)/fuzix-ddos.DSK bs=256 conv=sync

.PHONY: diskimage
diskimage: $(DISKIMAGES)

## - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
## Tidy up

clean:
	rm -f $(CLEAN_FZLOAD09)
	rm -f *.o core *~
