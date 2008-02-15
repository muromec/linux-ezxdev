#
# Makefile for the kernel spi bus driver.
#

O_TARGET := spi.o

export-objs	:= spi-core.o

# Init order: core, chardev, bit adapters, pcf adapters

obj-$(CONFIG_SPI)		+= spi-core.o
# obj-$(CONFIG_SPI_CHARDEV)	+= spi-dev.o

# Bit adapters

# PCF adapters

include $(TOPDIR)/Rules.make

