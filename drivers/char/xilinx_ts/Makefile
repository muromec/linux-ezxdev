#
# Makefile for the Xilinx touchscreen driver
#

ifeq ($(CONFIG_PPC32),y)
EXTRA_CFLAGS		+= -I$(TOPDIR)/arch/ppc/platforms/xilinx_ocp
endif

list-multi		:= xilinx_ts.o

# The Linux adapter for the Xilinx driver code.
xilinx_ts-objs		+= adapter.o

# The Xilinx OS independent code.
xilinx_ts-objs		+= xtouchscreen.o xtouchscreen_g.o \
			   xtouchscreen_intr.o xtouchscreen_l.o

obj-$(CONFIG_XILINX_TS) := xilinx_ts.o

xilinx_ts.o: $(xilinx_ts-objs)
	$(LD) -r -o $@ $(xilinx_ts-objs)

include $(TOPDIR)/Rules.make
