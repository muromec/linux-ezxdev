#
# Makefile for the OMAP1610-specific parts of Linux kernel.
#
# Copyright (C) 2004 MontaVista Software, Inc.
#  <source@mvista.com>
# Copyright 2001 RidgeRun, Inc (glonnon@ridgerun.com) info@ridgerun.com
# http://www.ridgerun.com
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).


USE_STANDARD_AS_RULE := true

O_TARGET		:= omap1610.o

export-objs		:= irq.o
export-objs		+= dma-omap.o
export-objs		+= ck.o
export-objs		+= innovator.o
export-objs		+= generic.o
export-objs             += omap1610_uwire.o

# MVL-CEE added innovator-ldm.o, innovator_dpm.o
export-objs		+= innovator-ldm.o omap1610_dpm.o innovator_dpm.o

# Object file lists.
obj-y 			:= ck.o
obj-m			:=
obj-n			:=
obj-			:=

# Common support (must be linked before board specific support)
obj-y += generic.o irq.o dma-omap.o time.o omap1610_uwire.o

# MVL-CEE added innovator-ldm.o
obj-y			+= innovator-ldm.o

# Specific board support
obj-$(CONFIG_OMAP_INNOVATOR) += innovator.o

# LEDs support
leds-y := leds.o
leds-$(CONFIG_OMAP_INNOVATOR) += leds-innovator.o
obj-$(CONFIG_LEDS) += $(leds-y)

obj-$(CONFIG_OMAP_PWT)		+= pwt.o
obj-$(CONFIG_OMAP1610_PM)	+= pm.o sleep.o

obj-$(CONFIG_KGDB_SERIAL)	+= kgdb-serial.o

# MVL-CEE added innovator_dpm.o
obj-$(CONFIG_DPM)		+= omap1610_dpm.o
obj-$(CONFIG_INNOVATOR_DPM)	+= innovator_dpm.o

include $(TOPDIR)/Rules.make
