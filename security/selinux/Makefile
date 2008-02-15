#
# Makefile for building the SELinux module as part of the kernel tree.
#

O_TARGET := selinux-obj.o

subdir-$(CONFIG_SECURITY_SELINUX) := flask ss arch/$(ARCH)

selinux-objs := avc.o psid.o hooks.o syscalls.o arch/$(ARCH)/security.o ss/ss.o 
ifeq ($(CONFIG_SECURITY_SELINUX_NSID),y)
	selinux-objs += nsid.o
	export-objs += nsid.o
endif

ifeq ($(CONFIG_SECURITY_SELINUX_SELOPT),y)
	subdir-$(CONFIG_SECURITY_SELINUX) += selopt
        selinux-objs += selopt/selopt.o
endif

obj-y := $(selinux-objs)

EXTRA_CFLAGS += -Iinclude 

include $(TOPDIR)/Rules.make

ss/ss.o:
	make -C ss ss.o

selopt/selopt.o:
	make -C selopt selopt.o

arch/$(ARCH)/security.o:
	make -C arch/$(ARCH) security.o

# make sure symlinks are built
asm-deps := $(selinux-objs) 
$(asm-deps): include/asm

include/asm:
	rm -f include/asm 
	ln -sf asm-$(ARCH) include/asm
