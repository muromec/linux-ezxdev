#
# Function driver for a Mouse USB Device
#
# Copyright (c) 2003 Belcarra

# Multipart objects.

O_TARGET	:= mouse_fd_drv.o
list-multi	:= mouse_fd.o 

mouse_fd-objs	:= mouse.o 

# Objects that export symbols.
export-objs	:= mouse.o

# Object file lists.

obj-y	:=
obj-m	:=
obj-n	:=
obj-	:=

# Each configuration option enables a list of files.

obj-$(CONFIG_USBD_MOUSE)   += mouse_fd.o

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
MOUSED=$(USBD)/mouse_fd
include $(TOPDIR)/Rules.make
EXTRA_CFLAGS += -I$(MOUSED) -I$(USBD) -Wno-unused -Wno-format -Werror 
EXTRA_CFLAGS_nostdinc += -I$(MOUSED) -I$(USBD) -Wno-unused -Wno-format -Werror

# Link rules for multi-part drivers.

mouse_fd.o: $(mouse_fd-objs)
	$(LD) -r -o $@ $(mouse_fd-objs)

# dependencies:

mouse.o: $(USBD)/usbd.h $(USBD)/usbd-bus.h $(USBD)/usbd-func.h


