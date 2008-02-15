#
# Makefile for misc devices that really don't fit anywhere else.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#
# Note 2! The CFLAGS definitions are now inherited from the
# parent makes..
# 
# Copyright (C) 2005 Motorola, Inc.
# 
# - Add EzX touch screen support.
# - Add EzX keypad support.
# - Add EzX keylight and PCAP RTC support.
# - Add EzX log and fmradio support.
#
#*********************************** history *****************************
#    author          date         CR          title
#    weiqiang lin    2005-06-30  LIBgg39140   update pcap interface to spi mode
#    update
#    Zhenfang Wang  2005-08-18 LIBgg73078  remove no use files' object

O_TARGET                        := misc.o

export-objs			:= ezx-ts.o keylight.o log.o pcap_rtc.o

obj-$(CONFIG_ARCH_EZX_TS)       += ezx-ts.o 

obj-$(CONFIG_ARCH_EZX)          += ezx_keypad.o


obj-$(CONFIG_FMRADIO_E680)	+= fmradio.o
obj-$(CONFIG_MOT_POWER_IC_PCAP_RTC)     += pcap_rtc.o
obj-$(CONFIG_MOT_POWER_IC_KEYLIGHT)     += keylight.o
obj-$(CONFIG_HAINAN_LED)	+= pwm-led.o
obj-$(CONFIG_SUMATRA_LED)  	+= pwm-led.o

ifeq ($(CONFIG_PANIC_LOG),y)
obj-$(CONFIG_PANIC_LOG)		+= log.o
ifeq ($(CONFIG_ARCH_EZX),y)
obj-$(CONFIG_ARCH_EZX)		+= ezx-log.o
endif
endif

include $(TOPDIR)/Rules.make


fastdep:
