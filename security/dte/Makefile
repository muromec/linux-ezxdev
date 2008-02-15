#
# Makefile for the DTE code
#

O_TARGET := vmlinux-obj.o

obj-$(CONFIG_SECURITY_DTE)	:= dte_plug.o

dte-objs			:= dte.o inode.o mount.o read_policy.o task.o \
					module.o path.o syscall.o

include $(TOPDIR)/Rules.make

dte_plug.o: $(dte-objs)
	$(LD) -r -o $@ $(dte-objs)
