#
# USBD Function driver for a Config11 USB Device
#
# Copyright (C) 2001 Lineo, Inc.
# Copyright (C) 2001 Hewlett-Packard Co.

# Copyright (C) 2003 Motorola Inc.
# 2003-Dec-06 - Modified for EZXBASE By Zhao Liang <w20146@motorola.com>

# Multipart objects.

O_TARGET	:= cfg11_fd_drv.o
list-multi	:= cfg11_fd.o

cfg11_fd-objs	:= cfg11.o 


export-objs     := cfg11.o

# Object file lists.

obj-y	:=
obj-m	:=
obj-n	:=
obj-	:=

# Each configuration option enables a list of files.

obj-$(CONFIG_USBD_CFG11)   += cfg11_fd.o

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
CFG11D=$(USBD)/cfg11_fd
include $(TOPDIR)/Rules.make
# Link rules for multi-part drivers.
EXTRA_CFLAGS += -I$(CFG11D) -I$(USBD) -Wno-unused -Wno-format
EXTRA_CFLAGS_nostdinc += -I$(CFG11D) -I$(USBD) -Wno-unused -Wno-format


cfg11_fd.o: $(cfg11_fd-objs)
	$(LD) -r -o $@ $(subst dummy,,$^)

# dependencies:

cfg11.o: $(USBD)/usbd.h $(USBD)/usbd-bus.h $(USBD)/usbd-func.h 

