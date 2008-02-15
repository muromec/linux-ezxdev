#
# Makefile for the linux pram-filesystem routines.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#
# Note 2! The CFLAGS definitions are now in the main makefile...

O_TARGET := pramfs.o

export-objs := super.o

obj-y    := balloc.o dir.o file.o fsync.o inode.o ioctl.o namei.o \
		super.o symlink.o
ifndef CONFIG_PRAMFS_NOWP
obj-y	+= wprotect.o
endif

obj-m    := $(O_TARGET)

include $(TOPDIR)/Rules.make
