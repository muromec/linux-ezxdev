
O_TARGET := v4l2-omap.o

# All of the (potential) objects that export symbols.
# This list comes from 'grep -l EXPORT_SYMBOL *.[hc]'.

export-objs :=

# Each configuration option enables a list of files.

obj-y := v4l2.o

ifdef CONFIG_ARCH_OMAP1510
ifdef CONFIG_OMAP_INNOVATOR
obj-y += camif_innovator.o camera_ov6x30.o sbus_sccb.o
else
obj-y += camif_evm.o camera_sanyo.o camera_ov6x30.o sbus_old_i2c.o
endif
else
obj-y += camif_innovator.o camera_ov6x30.o camera_ov9640.o sbus_new_i2c.o
endif

obj-m := $(O_TARGET)

include $(TOPDIR)/Rules.make
