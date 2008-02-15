O_TARGET := mx21fb.o

# All of the (potential) objects that export symbols.
# This list comes from 'grep -l EXPORT_SYMBOL *.[hc]'.

export-objs    := mx21tvout.o

# Each configuration option enables a list of files.

obj-y := dbmx21fb.o
ifeq ($(CONFIG_MX21_TVOUT),y)
    obj-y += mx21tvout.o
endif
obj-m := $(O_TARGET)

include $(TOPDIR)/Rules.make

