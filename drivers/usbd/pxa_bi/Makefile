#
# Makefile for the kernel USBD (device not host) drivers.
#
# Copyright (c) 2002-2003 Belcarra

# Copyright (c) 2003 Motorola.
# 2003-May-06 - Modified for EZXBASE By Zhao Liang <w20146@motorola.com>

# Subdirs.
# This is a bit complex, because some subdirs are for
# proprietary code, and are simply not present in a
# general distribution.

# The all-CAPS *_DIRS get nuked in the new versions
# of Rules.make, so use only the subdir-* methods.
subdir-y 	:=
subdir-m 	:=
subdir-n 	:=
subdir- 	:=

# The target object and module list name.

O_TARGET	:= pxa_bi_drv.o

# Objects that export symbols.

export-objs	:= 

# Multipart objects.

pxa_bi-objs	:= pxa.o usbd-bi.o trace.o

# Optional parts of multipart objects.

# Object file lists.

#obj-y		:= usbd-bi.o 
obj-y		:=
obj-m		:=
obj-n		:=
obj-		:=

# Each configuration option enables a list of files.

obj-$(CONFIG_USBD_PXA_BUS)		+= pxa_bi.o

# Object files in subdirectories


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
USBD=$(TOPDIR)/drivers/usbd
BI_DIR=$(USBD)/pxa_bi
#EXTRA_CFLAGS += -Wno-missing-prototypes -Wno-unused -Wno-format -Werror -I$(USBD) -I$(BI_DIR)
#EXTRA_CFLAGS_nostdinc += -Wno-missing-prototypes -Wno-unused -Wno-format -Werror -I$(USBD) -I$(BI_DIR)

EXTRA_CFLAGS += -Wno-missing-prototypes -Wno-unused -Wno-format -I$(USBD) -I$(BI_DIR)
EXTRA_CFLAGS_nostdinc += -Wno-missing-prototypes -Wno-unused -Wno-format -I$(USBD) -I$(BI_DIR)


vpath %.c $(USBD)

# Link rules for multi-part drivers.

pxa_bi.o: $(pxa_bi-objs)
	$(LD) -r -o $@ $(pxa_bi-objs)

# dependencies:

# local


