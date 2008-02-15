#
# Makefile for the Linux kernel device drivers.
#
# 15 Sep 2000, Christoph Hellwig <hch@infradead.org>
# Rewritten to use lists instead of if-statements.
#

#
# Copyright (C) 2005 Motorola Inc.
#
# modified by a15285, for EZX platform
#

mod-subdirs :=	dio hil mtd sbus video macintosh usb input telephony sgi ide \
		message/i2o message/fusion scsi md ieee1394 pnp isdn atm \
		fc4 net/hamradio i2c l3 acpi bluetooth serial maple vme \
		sensors usb/device
mod-subdirs +=	spi mmc
mod-subdirs +=  usbd
# MVL-CEE added base
subdir-y :=	parport char block net sound misc media cdrom hotplug base
subdir-m :=	$(subdir-y)


subdir-$(CONFIG_DIO)		+= dio
subdir-$(CONFIG_PCI)		+= pci
subdir-$(CONFIG_GSC)		+= gsc
subdir-$(CONFIG_HIL)		+= hil
subdir-$(CONFIG_PCMCIA)		+= pcmcia
subdir-$(CONFIG_MTD)		+= mtd
subdir-$(CONFIG_SBUS)		+= sbus
subdir-$(CONFIG_ZORRO)		+= zorro
subdir-$(CONFIG_NUBUS)		+= nubus
subdir-$(CONFIG_TC)		+= tc
subdir-$(CONFIG_VT)		+= video
subdir-$(CONFIG_MAC)		+= macintosh
subdir-$(CONFIG_PPC32)		+= macintosh
subdir-$(CONFIG_USB)		+= usb
#subdir-$(CONFIG_USBD)           += usb/device
subdir-$(CONFIG_USBD)           += usbd
subdir-$(CONFIG_INPUT)		+= input
subdir-$(CONFIG_PHONE)		+= telephony
subdir-$(CONFIG_SGI)		+= sgi
subdir-$(CONFIG_IDE)		+= ide
subdir-$(CONFIG_SCSI)		+= scsi
subdir-$(CONFIG_I2O)		+= message/i2o
subdir-$(CONFIG_FUSION)		+= message/fusion
subdir-$(CONFIG_MD)		+= md
subdir-$(CONFIG_IEEE1394)	+= ieee1394
subdir-$(CONFIG_PNP)		+= pnp
subdir-$(CONFIG_ISDN_BOOL)	+= isdn
subdir-$(CONFIG_ATM)		+= atm
subdir-$(CONFIG_FC4)		+= fc4
subdir-$(CONFIG_TRACE)		+= trace

# CONFIG_HAMRADIO can be set without CONFIG_NETDEVICE being set  -- ch
subdir-$(CONFIG_HAMRADIO)	+= net/hamradio
subdir-$(CONFIG_SERIAL_CORE)	+= serial
subdir-$(CONFIG_I2C)		+= i2c
subdir-$(CONFIG_SENSORS) 	+= sensors
subdir-$(CONFIG_L3)		+= l3
subdir-$(CONFIG_SPI)		+= spi
subdir-$(CONFIG_ACPI)		+= acpi
subdir-$(CONFIG_MOT_POWER_IC)	+= power_ic

subdir-$(CONFIG_MODEM_CONNECTIONS)	+= modem

subdir-$(CONFIG_BLUEZ)		+= bluetooth
subdir-$(CONFIG_MMC)		+= mmc
subdir-$(CONFIG_SSI)		+= ssi
subdir-$(CONFIG_MAPLE)		+= maple
subdir-$(CONFIG_VMEBUS)		+= vme

# MVL-CEE added dpm
subdir-$(CONFIG_DPM)		+= dpm

MARVELL_SUBDIR_FLAG=n

ifeq ($(CONFIG_MARVELL_WIFI), m)
  subdir-m   += net/wireless/8385
  MARVELL_SUBDIR_FLAG = y
endif

ifeq ($(CONFIG_MARVELL_SDIO), m)
  ifeq ($(MARVELL_SUBDIR_FLAG), n)
     subdir-m   += net/wireless/8385
  endif
endif

MARVELL_MFG_SUBDIR_FLAG=n

ifeq ($(CONFIG_MARVELL_WIFI_MFG), m)
  subdir-m   += net/wireless/8385_mfg
  MARVELL_MFG_SUBDIR_FLAG = y
endif

ifeq ($(CONFIG_MARVELL_SDIO_MFG), m)
  ifeq ($(MARVELL_MFG_SUBDIR_FLAG), n)
     subdir-m   += net/wireless/8385_mfg
  endif
endif

include $(TOPDIR)/Rules.make
