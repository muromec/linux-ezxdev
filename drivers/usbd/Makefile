#
# Makefile for the kernel USBD (device not host) drivers.
#
# Copyright (c) 2002 Belcarra
# Copyright (C) 2001 Lineo, Inc.
# Copyright (C) 2001 Hewlett-Packard Co.

# Subdirs.
# This is a bit complex, because some subdirs are for
# proprietary code, and are simply not present in a
# general distribution.

#
# Copyright (C) 2005 - Motorola
#
# modified by a16157 for EZXBASE product
#


TOPDIR ?= ../../..

# The all-CAPS *_DIRS get nuked in the new versions
# of Rules.make, so use only the subdir-* methods.
subdir-y 	:= 
subdir-m 	:=
subdir-n 	:=
subdir- 	:=

# Function Drivers
subdir-$(CONFIG_USBD_ACM) += acm_fd			
subdir-$(CONFIG_USBD_MOUSE) += mouse_fd			
subdir-$(CONFIG_USBD_NETWORK) += network_fd		
subdir-$(CONFIG_USBD_PST) += pst_fd		
subdir-$(CONFIG_USBD_CFG11) += cfg11_fd		
subdir-$(CONFIG_USBD_STORAGE) += storage_fd

#subdir-$(CONFIG_USBD_AUDIO) += audio_fd			
#subdir-$(CONFIG_USBD_EG) += eg_fd			
#subdir-$(CONFIG_USBD_KEYBOARD) += keyboard_fd			
#subdir-$(CONFIG_USBD_CDC) += cdc_fd		

# Bus Interface Drivers
subdir-$(CONFIG_USBD_PXA_BUS) += pxa_bi			
subdir-$(CONFIG_USBD_WMMX_BUS) += wmmx_bi			
subdir-$(CONFIG_USBD_ISP_BUS) += isp_bi			

#subdir-$(CONFIG_USBD_OMAP_BUS) += omap_bi			
#subdir-$(CONFIG_USBD_TC86C001_BUS) += tc86c001_bi			

#subdir-$(CONFIG_USBD_GENERIC_BUS) += gen_bi		
#subdir-$(CONFIG_USBD_SL11_BUS) += sl11_bi		
subdir-$(CONFIG_USBD_DSP_LOG) +=dsplog_fd
subdir-$(CONFIG_USBD_NM) += nm_fd

# The target object and module list name.

O_TARGET	:= usbdev.o

# Objects that export symbols.

export-objs	:= usbd.o usbd-bops.o usbd-fops.o usbd-bi.o ep0.o

# Multipart objects.

list-multi		:= usbdcore.o 
usbdcore-objs		:= usbd.o ep0.o usbd-fops.o usbd-bops.o 
usbdprocfs-objs		:= usbd-procfs.o


# Optional parts of multipart objects.

# Object file lists.

obj-y		:=
obj-m		:=
obj-n		:=
obj-		:=

# Each configuration option enables a list of files.

obj-$(CONFIG_USBD)		+= usbdcore.o 
obj-$(CONFIG_USBD_PROCFSM)	+= usbdprocfs.o 

# Object files in subdirectories

#
obj-$(CONFIG_USBD_MONITOR) += monitor/monitor_drv.o

obj-$(CONFIG_USBD_ACM) += acm_fd/acm_fd_drv.o
obj-$(CONFIG_USBD_MOUSE) += mouse_fd/mouse_fd_drv.o
obj-$(CONFIG_USBD_NETWORK) += network_fd/network_fd_drv.o
obj-$(CONFIG_USBD_PST) += pst_fd/pst_fd_drv.o
obj-$(CONFIG_USBD_CFG11) += cfg11_fd/cfg11_fd_drv.o
obj-$(CONFIG_USBD_STORAGE) += storage_fd/storage_fd_drv.o

#obj-$(CONFIG_USBD_AUDIO) += audio_fd/audio_fd_drv.o
#obj-$(CONFIG_USBD_EG) += eg_fd/eg_fd_drv.o
#obj-$(CONFIG_USBD_KEYBOARD) += keyboard_fd/keyboard_fd_drv.o

# Bus Interface Drivers
obj-$(CONFIG_USBD_PXA_BUS) += pxa_bi/pxa_bi_drv.o
obj-$(CONFIG_USBD_WMMX_BUS) += wmmx_bi/wmmx_bi_drv.o
obj-$(CONFIG_USBD_ISP_BUS) += isp_bi/isp_bi_drv.o

#obj-$(CONFIG_USBD_OMAP_BUS) += omap_bi/omap_bi_drv.o
#obj-$(CONFIG_USBD_TC86C001_BUS) += tc86c001_bi/tc86c001_bi_drv.o

#obj-$(CONFIG_USBD_GENERIC_BUS) += gen_bi/gen_bi_drv.o
#obj-$(CONFIG_USBD_SL11_BUS) += sl11_bi/sl11_bi_drv.o

ifeq ($(CONFIG_USBD_DSP_LOG),y)
	obj-y +=dsplog_fd/dsplog_fd_drv.o
endif

# Yech.  This isn't the best way to do this, but there isn't a config flag
# common to all the possible bus interfaces

ifeq ($(CONFIG_USBD_NM),y)
	obj-y += nm_fd/nm_fd_drv.o	
endif

# usbdezx.o should be linked after all usbd modules - w20146, Apr 20
obj-$(CONFIG_ARCH_EZX) += usbdezx.o

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
EXTRA_CFLAGS += -Wno-format -Wall

# Link rules for multi-part drivers.

usbdprocfs.o: $(usbdprocfs-objs)
	$(LD) -r -o $@ $(usbdprocfs-objs)

usbdcore.o: $(usbdcore-objs)
	$(LD) -r -o $@ $(usbdcore-objs)

# dependencies:

usbd.o: usbd-build.h

usbd-build.h:
	echo "#define USBD_BUILD          \"000\"" > $@
	
# local

%.h:%.p
	release inc build < $< > $@
	cp $@ $<

release.h: release.p

inc-build:
	release inc build < release.p > release.h
	cp release.h release.p


menuconfig:
	cd $(TOPDIR); make menuconfig

xconfig:
	cd $(TOPDIR); make xconfig



