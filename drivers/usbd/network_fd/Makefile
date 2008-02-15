#
# Network Function Driver
#
# Copyright (C) 2002-2003 Belcarra

# Copyright (c) 2003 Motorola.
# 2003-May-06 - Modified for EZXBASE By Zhao Liang <w20146@motorola.com>


O_TARGET	:= network_fd_drv.o
list-multi	:= network_fd.o

network_fd-objs	:= network.o basic.o basic2.o blan.o cdc.o safe.o fermat.o

# Objects that export symbols.
export-objs	:= network.o


# Object file lists.

obj-y		:=
obj-m		:=
obj-n		:=
obj-		:=

# Each configuration option enables a list of files.

obj-$(CONFIG_USBD_NETWORK)   += network_fd.o

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

USBD=$(TOPDIR)/drivers/usbd
NETWORKD=$(USBD)/network_fd
include $(TOPDIR)/Rules.make
EXTRA_CFLAGS += -I$(NETWORKD) -I$(USBD) -Wno-unused -Wno-format #-Werror 
EXTRA_CFLAGS_nostdinc += -I$(NETWORKD) -I$(USBD) -Wno-unused -Wno-format #-Werror

# Link rules for multi-part drivers.

network_fd.o: $(network_fd-objs)
	$(LD) -r -o $@ $(network_fd-objs)

# dependencies:

network.o: network.h $(USBD)/usbd.h $(USBD)/usbd-bus.h $(USBD)/usbd-func.h


