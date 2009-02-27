#
# Function driver for a CDC ACM USB Device
#
# Copyright (c) 2003 Belcarra

# Copyright (c) 2003 Motorola.
# 2003-May-06 - Modified for EZXBASE By Zhao Liang <w20146@motorola.com>

# Multipart objects.

O_TARGET	:= acm_fd_drv.o
list-multi	:= acm_fd.o 

acm_fd-objs	:= acm.o trace.o

# Objects that export symbols.
export-objs	:= acm.o

# Object file lists.

obj-y	:=
obj-m	:=
obj-n	:=
obj-	:=

# Each configuration option enables a list of files.

obj-$(CONFIG_USBD_ACM)   += acm_fd.o

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
ACMD=$(USBD)/acm_fd
include $(TOPDIR)/Rules.make
EXTRA_CFLAGS += -I$(ACMD) -I$(USBD) -Wno-unused -Wno-format  
EXTRA_CFLAGS_nostdinc += -I$(ACMD) -I$(USBD) -Wno-unused -Wno-format

# Link rules for multi-part drivers.

acm_fd.o: $(acm_fd-objs)
	$(LD) -r -o $@ $(acm_fd-objs)

# dependencies:

acm.o: $(USBD)/usbd.h $(USBD)/usbd-bus.h $(USBD)/usbd-func.h

