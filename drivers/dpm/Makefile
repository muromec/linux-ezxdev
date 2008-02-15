#
# Makefile for the kernel DPM driver.
#

O_TARGET := dpm-drv.o

export-objs	:= dpm.o dpm-opt.o dpm-syscall.o dpm-stats.o

obj-$(CONFIG_DPM)	+= dpm.o dpm-opt.o dpm-syscall.o
obj-$(CONFIG_PROC_FS)	+= proc.o
obj-$(CONFIG_DPM_TRACE)	+= trace.o
obj-$(CONFIG_DPM_STATS)	+= dpm-stats.o
obj-$(CONFIG_DPM_IDLE)	+= dpm-idle.o

include $(TOPDIR)/Rules.make
