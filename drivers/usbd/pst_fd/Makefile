#
# PST device layer for USB Device subsystem.
#
# Copyright (C) 2001 Lineo, Inc.
# Copyright (C) 2001 Hewlett-Packard Co.
# Copyright (C) 2003 RTSoft.
#
# Copyright (C) 2003 Motorola.
# 2003-May-06 - Modified for EZXBASE By Zhao Liang

O_TARGET	:= pst_fd_drv.o

export-objs	:=pst-fd.o
# Multipart object and list of it's object files.
list-multi	:= pst_fd.o
pst_fd-objs	:= pst-fd.o ptf_buf.o ptfproto.o

# Objects that export symbols.
export-objs	:= pst-fd.o

# Object file lists.

obj-y	:=
obj-m	:=
obj-n	:=
obj-	:=

# Each configuration option enables a list of files.

obj-$(CONFIG_USBD_PST)   += pst_fd.o

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
PSTD=$(USBD)/pst_fd
include $(TOPDIR)/Rules.make
EXTRA_CFLAGS += -I$(PSTD) -I$(USBD) -Wno-unused -Wno-format #-Werror 
EXTRA_CFLAGS_nostdinc += -I$(PSTD) -I$(USBD) -Wno-unused -Wno-format #-Werror

# Link rules for multi-part drivers.

pst_fd.o: $(pst_fd-objs)
	$(LD) -r -o $@ $(pst_fd-objs)

# dependencies:

pst.o: $(USBD)/usbd.h $(USBD)/usbd-bus.h $(USBD)/usbd-func.h

