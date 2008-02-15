#
# Makefile for building the SELinux security server as part of the kernel tree.
#

EXTRA_CFLAGS += -I../include -include global.h -I..

O_TARGET := ss.o

obj-y := ebitmap.o hashtab.o symtab.o sidtab.o avtab.o policydb_inflate.o policydb.o services.o syscalls.o 
#obj-m := $(O_TARGET)

ifeq ($(CONFIG_SECURITY_SELINUX_MLS),y)
obj-y += mls.o
endif

all: $(O_TARGET) 

global-deps := $(obj-y) 
$(global-deps): global.h

clean:
	rm -f $(O_TARGET) $(O_OBJS) 

include $(TOPDIR)/Rules.make

