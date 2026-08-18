fuzix.bin: target $(OBJS) tools/decbdragon tools/decb-image tools/visualize6809 tools/diskpad
	+make -C platform/platform-$(TARGET) image
	tools/visualizefcc < fuzix.map
	tools/hogfather fuzix.map | sort -nr >fuzix.hogs
