tools/visualizefcc: tools/visualizefcc.c

tools/pack85: tools/pack85.c

tools/packdiscard: tools/packdiscard.c

tools/doubleup: tools/doubleup.c

tools/makejv3: tools/makejv3.c

cpm-loader-fcc/cpmload.bin: cpm-loader-fcc/cpmload.S cpm-loader-fcc/fuzixload.S cpm-loader-fcc/makecpmloader.c
	+$(MAKE) -C cpm-loader-fcc

fuzix.bin: target $(OBJS) tools/pack85 tools/packdiscard tools/visualizefcc tools/doubleup cpm-loader-fcc/cpmload.bin tools/makejv3
	+$(MAKE) -C platform/platform-$(TARGET) image
	(cd platform/platform-$(TARGET); ../../tools/visualizefcc <../../fuzix.map)
	tools/hogfather fuzix.map | sort -nr >fuzix.hogs
