#
# Makefile for the Eventing Mechanism implementation.
#

O_TARGET	:= event.o

obj-y   := ev_core.o user_api.o kernel_api.o group_api.o access_api.o \
	   helper_funcs.o debug.o

export-objs := ev_core.o kernel_api.o group_api.o

include $(TOPDIR)/Rules.make

