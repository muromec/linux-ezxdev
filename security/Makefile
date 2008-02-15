#
# Makefile for the kernel security code
#
# Copyright (C) 2005  Motorola Inc.
#
# 2005-Mar - Motorola - Modified by Zhu Zhifu for Security Access Control Module
#

# The target object and module list name.
O_TARGET	:= vmlinux-obj.o

# subdirectory list
#subdir-$(CONFIG_SECURITY_SELINUX)	+= selinux
#subdir-$(CONFIG_SECURITY_SELINUX_SELOPT)+= selinux/selopt
#subdir-$(CONFIG_SECURITY_DTE)		+= dte
#subdir-$(CONFIG_LIDS)		+= lids 
subdir-$(CONFIG_SECURITY_MOTOAC)	+= motoac

# Objects that export symbols
export-objs	:= security.o capability.o

# if we don't select a security model, use the default capabilities
ifneq ($(CONFIG_SECURITY),y)
obj-y		+= capability.o
endif

# Object file lists
obj-$(CONFIG_SECURITY)		+= security.o dummy.o

# Must precede capabilities in order to stack properly.
ifeq ($(CONFIG_SECURITY_SELINUX),y)
	obj-$(CONFIG_SECURITY_SELINUX)	+= selinux/selinux-obj.o
endif
obj-$(CONFIG_SECURITY_MOTOAC)           += motoac/motoac.o
obj-$(CONFIG_SECURITY_CAPABILITIES)	+= capability.o
obj-$(CONFIG_SECURITY_OWLSM)		+= owlsm.o
ifeq ($(CONFIG_SECURITY_DTE),y)
	obj-y += dte/dte_plug.o
endif
ifeq ($(CONFIG_LIDS),y)
	obj-y += lids/lids.o
endif

include $(TOPDIR)/Rules.make
