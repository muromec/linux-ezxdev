#
# Makefile for the linux kernel.
#
# Copyright 2001 RidgeRun, Inc (glonnon@ridgerun.com) info@ridgerun.com
# http://www.ridgerun.com
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).


USE_STANDARD_AS_RULE := true

O_TARGET		:= omap1510.o

export-objs		:= irq.o
export-objs		+= dma-omap.o
export-objs		+= ck.o
export-objs		+= innovator.o
export-objs		+= generic.o
# MVL-CEE added innovator-ldm.o, innovator_dpm.o
export-objs		+= innovator-ldm.o omap1510_dpm.o innovator_dpm.o


# Object file lists.
obj-y 			:= ck.o
obj-m			:=
obj-n			:=
obj-			:=

# Common support (must be linked before board specific support)
obj-y += generic.o irq.o dma-omap.o time.o

# MVL-CEE added innovator-ldm.o
obj-y				+= innovator-ldm.o

# Specific board support
obj-$(CONFIG_OMAP_INNOVATOR) += innovator.o

# LEDs support
leds-y := leds.o
leds-$(CONFIG_OMAP_INNOVATOR) += leds-innovator.o
obj-$(CONFIG_LEDS) += $(leds-y)

obj-$(CONFIG_OMAP_PWT)		+= pwt.o
obj-$(CONFIG_OMAP1510_PM)	+= pm.o sleep.o

obj-$(CONFIG_KGDB_SERIAL)	+= kgdb-serial.o

# MVL-CEE added innovator_dpm.o
obj-$(CONFIG_DPM)		+= omap1510_dpm.o
obj-$(CONFIG_INNOVATOR_DPM)	+= innovator_dpm.o


# Files that are both resident and modular; remove from modular.

obj-m		:= $(filter-out $(obj-y), $(obj-m))

# Translate to Rules.make lists.

O_OBJS		:= $(filter-out $(export-objs), $(obj-y))
OX_OBJS 	:= $(filter	$(export-objs), $(obj-y))
M_OBJS		:= $(sort $(filter-out $(export-objs), $(obj-m)))
MX_OBJS 	:= $(sort $(filter     $(export-objs), $(obj-m)))

include $(TOPDIR)/Rules.make
