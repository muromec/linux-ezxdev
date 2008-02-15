#
# Makefile for the Linux kernel device drivers.
#
# 15 Sep 2000, Christoph Hellwig <hch@caldera.de>
# Rewritten to use lists instead of if-statements.
#


mod-subdirs :=	

O_TARGET := ha.o

ifeq ($(CONFIG_EVENT_BROKER),y)
  obj-y += event/event.o
endif

subdir-y :=
subdir-m :=


subdir-$(CONFIG_EVENT_BROKER)		+= event

include $(TOPDIR)/Rules.make

