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

O_TARGET		:= omap710.o

# Object file lists.
obj-y			:=
obj-m			:=
obj-n			:=
obj-			:=

export-objs		:= fpga.o
export-objs		+= dma-omap.o
export-objs		+= timer.o
export-objs		+= ck.o

obj-y				+= time.o
obj-y				+= arch.o
obj-y				+= mm.o
obj-y				+= fpga.o
obj-y				+= dma-omap.o
obj-y				+= irq.o
obj-$(CONFIG_LEDS)		+= 
obj-$(CONFIG_OMAP_TIMER)	+= timer.o
obj-$(CONFIG_OMAP_TIMER_PROC)	+= timer_proc.o
obj-$(CONFIG_OMAP_PWT)		+= pwt.o
obj-$(CONFIG_PM)		+= ck.o
obj-$(CONFIG_IRAM_ALLOC)	+= iram_test.o

obj-$(CONFIG_KGDB_SERIAL)	+= kgdb-serial.o

obj-$(CONFIG_DEBUG_OMAP710_REGS)	+= registers.o


# Files that are both resident and modular; remove from modular.

obj-m		:= $(filter-out $(obj-y), $(obj-m))

# Translate to Rules.make lists.

O_OBJS		:= $(filter-out $(export-objs), $(obj-y))
OX_OBJS 	:= $(filter	$(export-objs), $(obj-y))
M_OBJS		:= $(sort $(filter-out $(export-objs), $(obj-m)))
MX_OBJS 	:= $(sort $(filter     $(export-objs), $(obj-m)))

include $(TOPDIR)/Rules.make
