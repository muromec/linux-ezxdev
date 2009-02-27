# linux/drivers/video/ezx_lcd/Makefile
# Makefile for EZX LCD Frame Buffer Driver
# 
# Copyright (C) 2004-2005 - Motorola
# Makefile for the Linux video drivers.
# 5 Aug 1999, James Simmons, <mailto:jsimmons@edgeglobal.com>
# Rewritten to use lists instead of if-statements.
#
# 2004/06/28  Susan
#    Restruct LCD driver
# 2005/12/12  Wang limei
#    Remove oahu,add sumatra;
#    Update martinique, make it share LCD FB with barbados;


O_TARGET := ezx_lcd.o

export-objs    := pxafb.o 

obj-y      += pxafb.o

# files are picked up by PRODUCT CONFIG
obj-$(CONFIG_ARCH_EZX_BARBADOS)       += lcd_barbados.o
obj-$(CONFIG_ARCH_EZX_HAINAN)        += lcd_hainan.o
obj-$(CONFIG_ARCH_EZX_SUMATRA)        += lcd_sumatra.o
#obj-$(CONFIG_ARCH_EZX_OAHU)       += lcd_oahu.o
# Bartinique use the same LCD panel as Barbados, they share the same code
obj-$(CONFIG_ARCH_EZX_MARTINIQUE)            += lcd_barbados.o

# Files are picked out by specific PANEL vendor
# obj-$(CONFIG_LCD_TFT_72R89341N)           += 72r89341n.o
# obj-$(CONFIG_LCD_SMART_72R89405Y01)       += 72r89405y01.o

include $(TOPDIR)/Rules.make

clean:
	rm -f core *.o *.a *.s


