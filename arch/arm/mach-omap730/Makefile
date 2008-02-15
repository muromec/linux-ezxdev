#
# Makefile for the OMAP730-specific parts of Linux kernel.
#
# Copyright (c) 2004 MPC-Data Limited (http://www.mpc-data.co.uk)
# Dave Peverley <dpeverley@mpc-data.co.uk>
#
# Copyright (C) 2004 MontaVista Software, Inc.
#  <source@mvista.com>
#


USE_STANDARD_AS_RULE := true

O_TARGET		:= omap730.o

export-objs		:= irq.o
export-objs		+= dma-omap.o
export-objs		+= ck.o
export-objs		+= perseus2.o
export-objs		+= generic.o
export-objs             += omap-uwire.o

# MVL-CEE added perseus2-ldm.o, perseus2_dpm.o
export-objs		+= perseus2-ldm.o omap730_dpm.o perseus2_dpm.o


# Object file lists.
obj-y 			:= ck.o
obj-m			:=
obj-n			:=
obj-			:=

# Common support (must be linked before board specific support)
obj-y += generic.o irq.o dma-omap.o time.o omap-uwire.o

# MVL-CEE added perseus2-ldm.o
obj-y				+= perseus2-ldm.o

# Specific board support
obj-$(CONFIG_MACH_OMAP_PERSEUS2) += perseus2.o

# LEDs support
leds-y := leds.o
leds-$(CONFIG_MACH_OMAP_PERSEUS2) += leds-perseus2.o
obj-$(CONFIG_LEDS) += $(leds-y)

obj-$(CONFIG_OMAP_PWT)		+= pwt.o
obj-$(CONFIG_OMAP730_PM)	+= pm.o sleep.o

obj-$(CONFIG_KGDB_SERIAL)	+= kgdb-serial.o

# MVL-CEE added perseus2_dpm.o
obj-$(CONFIG_DPM)		+= omap730_dpm.o
obj-$(CONFIG_PERSEUS2_DPM)	+= perseus2_dpm.o

include $(TOPDIR)/Rules.make
