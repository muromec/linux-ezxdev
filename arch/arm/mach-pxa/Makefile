#
# Makefile for the linux kernel.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).

USE_STANDARD_AS_RULE := true

O_TARGET		:= pxa.o

obj-y :=
obj-m :=
obj-n :=
obj-  :=

export-objs := generic.o irq.o dma.o sa1111.o sram.o pxabus_ldm.o

# Common support (must be linked before board specific support)
obj-y += generic.o irq.o dma.o
obj-$(CONFIG_SA1111) += sa1111.o

# Specific board support
obj-$(CONFIG_ARCH_LUBBOCK) += lubbock.o
obj-$(CONFIG_ARCH_MAINSTONE) += mainstone.o pxabus_ldm.o
obj-$(CONFIG_ARCH_PXA_IDP) += idp.o
obj-$(CONFIG_ARCH_PXA_CERF) += cerf.o

# Support for blinky lights
leds-y := leds.o
leds-$(CONFIG_ARCH_LUBBOCK) += leds-lubbock.o
leds-$(CONFIG_ARCH_MAINSTONE) += leds-mainstone.o
leds-$(CONFIG_ARCH_PXA_IDP) += leds-idp.o
leds-$(CONFIG_ARCH_PXA_CERF) += leds-cerf.o

obj-$(CONFIG_LEDS) += $(leds-y)

# Misc features
obj-$(CONFIG_PM) += pm.o sleep.o

#+++what if PM and DPM are both on??
obj-$(CONFIG_DPM) += pm.o sleep.o

obj-$(CONFIG_KGDB_SERIAL) += kgdb-serial.o
obj-$(CONFIG_CPU_BULVERDE) += sram.o
obj-$(CONFIG_DPM) += bulverde_dpm.o mainstone_dpm.o bulverde_freq.o bulverde_voltage.o

include $(TOPDIR)/Rules.make
