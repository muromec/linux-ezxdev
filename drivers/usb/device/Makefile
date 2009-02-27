#
# Makefile for the kernel USBD (device not host) drivers.
#
# Copyright (C) 2001 Lineo, Inc.
# Copyright (C) 2001 Hewlett-Packard Co.


O_TARGET	:= usbd_drv.o


subdir-y 	:= 	bi
subdir-m 	:=	bi
subdir-n 	:=
subdir- 	:=

subdir-$(CONFIG_USBD_NET) += net_fd
subdir-$(CONFIG_USBD_SERIAL) += serial_fd
subdir-$(CONFIG_USBD_MOUSE) += mouse_fd
subdir-$(CONFIG_USBD_STORAGE) += storage_fd
subdir-$(CONFIG_USBD_TEST) += test_fd

#subdir-$(CONFIG_USBD_GENERIC_BUS) += gen_bi
#subdir-$(CONFIG_USBD_L7205_BUS) += l7205_bi
#subdir-$(CONFIG_USBD_SA1100_BUS) += sa1100_bi
#subdir-$(CONFIG_USBD_SL11_BUS) += sl11_bi
#subdir-$(CONFIG_USBD_SUPERH_BUS) += superh_bi

# The target object and module list name.

# Objects that export symbols.

export-objs	:= usbd.o usbd-bus.o usbd-debug.o usbd-func.o

# Multipart objects.

list-multi		:= usbdcore.o usbdmonitor.o usbdserial.o
usbdcore-objs		:= usbd.o ep0.o usbd-debug.o hotplug.o usbd-func.o usbd-bus.o
usbdmonitor-objs	:= usbd-monitor.o hotplug.o
usbdserial-objs		:= usbd-serialnumber.o hotplug.o

# Optional parts of multipart objects.

# Object file lists.

obj-y		:=
obj-m		:=
obj-n		:=
obj-		:=

# Each configuration option enables a list of files.

obj-$(CONFIG_USBD)		+= usbdcore.o 
obj-$(CONFIG_USBD_MONITOR)	+= usbdmonitor.o 
obj-$(CONFIG_USBD_MONITOR)	+= usbdserial.o 

# Object files in subdirectories
ifeq ($(CONFIG_USBD_NET),y)
        obj-y += net_fd/net_fd_drv.o
endif
ifeq ($(CONFIG_USBD_SERIAL),y)
        obj-y += serial_fd/serial_fd_drv.o
endif
ifeq ($(CONFIG_USBD_STORAGE),y)
        obj-y += storage_fd/storage_fd_drv.o
endif
ifeq ($(CONFIG_USBD),y)
        obj-y += bi/usbddrv.o
endif



# The global Rules.make.

include $(TOPDIR)/Rules.make

# Link rules for multi-part drivers.

usbdmonitor.o: $(usbdmonitor-objs)
	$(LD) -r -o $@ $(usbdmonitor-objs)

usbdserial.o: $(usbdserial-objs)
	$(LD) -r -o $@ $(usbdserial-objs)

usbdcore.o: $(usbdcore-objs)
	$(LD) -r -o $@ $(usbdcore-objs)

# dependencies:

usbd-monitor.o: usbd-export.h usbd-build.h
usbd-serial.o: usbd-export.h usbd-build.h
usbd.o: usbd-export.h usbd-build.h

usbd-export.h:
	echo "#define USBD_EXPORT_DATE    \"`date '+%Y-%m-%d %H:%M'`\""  > $@

usbd-build.h:
	echo "#define USBD_BUILD          \"000\"" > $@
	


