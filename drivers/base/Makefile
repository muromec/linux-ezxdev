# Makefile for the Linux device tree

obj-y		:= core.o sys.o interface.o power.o bus.o \
			driver.o class.o intf.o platform.o \
			cpu.o firmware.o

# obj-$(CONFIG_NUMA)	+= node.o  memblk.o

# linux-pm obj-y		+= fs/
subdir-y		+= fs

obj-$(CONFIG_HOTPLUG)	+= hotplug.o

export-objs	:= core.o power.o sys.o bus.o driver.o \
			class.o intf.o platform.o firmware.o

O_TARGET := base.o

include $(TOPDIR)/Rules.make
