#
# Makefile for the MX2-specific portions of the kernel.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).

USE_STANDARD_AS_RULE := true

O_TARGET		:= mx2ads.o

# Object file lists.

obj-y			:= arch.o cpu.o irq.o mm.o time.o dma.o gpio.o mx21-ldm.o generic.o
obj-m			:=
obj-n			:=
obj-$(CONFIG_PM)	+= mx2ads-pm.o
obj-			:=
obj-$(CONFIG_DPM) += mx21-dpm.o

# KGDB architecture dependent serial stub
obj-$(CONFIG_KGDB_SERIAL)	+= kgdb-serial.o

export-objs :=time.o dma.o cpu.o gpio.o mx21-ldm.o generic.o mx21-dpm.o

include $(TOPDIR)/Rules.make
