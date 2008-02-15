#
# Motorola DSP LOG device layer for USB Device subsystem.
#
# Copyright (C) 2004 Motorola.

O_TARGET	:= dsplog_fd_drv.o

export-objs	:= dsplog.o
# Multipart object and list of it's object files.
list-multi	:= dsplog_fd.o
dsplog_fd-objs	:= dsplog.o

# Object file lists.

obj-y		:=
obj-m		:=
obj-n		:=
obj-		:=

# Each configuration option enables a list of files.
obj-$(CONFIG_USBD_DSP_LOG)   +=  dsplog_fd.o

# The global Rules.make.
include $(TOPDIR)/Rules.make

# Link rules for multi-part drivers.
dsplog_fd.o: $(dsplog_fd-objs)
	$(LD) -r -o $@ $(dsplog_fd-objs)




