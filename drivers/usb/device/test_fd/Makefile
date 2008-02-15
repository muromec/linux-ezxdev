#
# SA1100 Function driver for a network USB Device
#
# Copyright (C) 2001 Lineo, Inc.
# Copyright (C) 2001 Hewlett-Packard Co.


O_TARGET	:= test_fd_drv.o
list-multi	:= test_fd.o

test_fd-objs	:= test-fd.o

# Object file lists.

obj-y		:=
obj-m		:=
obj-n		:=
obj-		:=

# Each configuration option enables a list of files.

obj-$(CONFIG_USBD_TEST)   += test_fd.o

# Extract lists of the multi-part drivers.
# The 'int-*' lists are the intermediate files used to build the multi's.

multi-y		:= $(filter $(list-multi), $(obj-y))
multi-m		:= $(filter $(list-multi), $(obj-m))
int-y		:= $(sort $(foreach m, $(multi-y), $($(basename $(m))-objs)))
int-m		:= $(sort $(foreach m, $(multi-m), $($(basename $(m))-objs)))

# Files that are both resident and modular: remove from modular.

obj-m		:= $(filter-out $(obj-y), $(obj-m))
int-m		:= $(filter-out $(int-y), $(int-m))

# Translate to Rules.make lists.

O_OBJS		:= $(filter-out $(export-objs), $(obj-y))
OX_OBJS		:= $(filter     $(export-objs), $(obj-y))
M_OBJS		:= $(sort $(filter-out $(export-objs), $(obj-m)))
MX_OBJS		:= $(sort $(filter     $(export-objs), $(obj-m)))
MI_OBJS		:= $(sort $(filter-out $(export-objs), $(int-m)))
MIX_OBJS	:= $(sort $(filter     $(export-objs), $(int-m)))

# The global Rules.make.

include $(TOPDIR)/Rules.make

# Link rules for multi-part drivers.

test_fd.o: $(test_fd-objs)
	$(LD) -r -o $@ $(test_fd-objs)

# dependencies:

test-fd.o: ../usbd.h ../usbd-bus.h ../usbd-func.h


