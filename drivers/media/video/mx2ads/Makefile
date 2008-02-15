
O_TARGET := v4l2-mx2ads.o

# All of the (potential) objects that export symbols.
# This list comes from 'grep -l EXPORT_SYMBOL *.[hc]'.

export-objs :=

mx2ads-pp-objs  := ppcore.o ppscale.o
# Each configuration option enables a list of files.

obj-y := v4l2.o camif_mx2ads.o prpscale.o

obj-$(CONFIG_VIDEO_MX2ADS_OV9640) += camera_ov9640.o ov9640_sbus.o
obj-$(CONFIG_VIDEO_MX2ADS_MISOC0343) += camera_misoc0343.o misoc0343_sbus.o

obj-m := $(O_TARGET)

obj-$(CONFIG_VIDEO_MX2ADS_PP) += mx2ads-pp.o

include $(TOPDIR)/Rules.make

mx2ads-pp.o: $(mx2ads-pp-objs)
	$(LD) $(LD_RFLAG) -r -o $@ $(mx2ads-pp-objs)


