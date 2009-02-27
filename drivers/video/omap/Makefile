
O_TARGET := omapfb.o

# All of the (potential) objects that export symbols.
# This list comes from 'grep -l EXPORT_SYMBOL *.[hc]'.

export-objs    :=  omap.o

# Each configuration option enables a list of files.

obj-y := omap.o
obj-m := $(O_TARGET)

include $(TOPDIR)/Rules.make
