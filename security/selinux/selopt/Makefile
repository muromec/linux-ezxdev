#
# Makefile for building selopt (SELinux labeled networking with CIPSO/FIPS-188
# IP options).
#

EXTRA_CFLAGS += -I../include -I..

O_TARGET := selopt.o

selopt-objs := selopt_core.o perimtab.o flnetlink.o cache.o queue.o

obj-y := $(selopt-objs)
obj-m := $(O_TARGET)

all: $(O_TARGET)

include $(TOPDIR)/Rules.make

