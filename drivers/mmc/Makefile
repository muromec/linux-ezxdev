#
# Makefile for the kernel mmc device drivers.
#
# Copyright (c) 2004-2005 Motorola Inc.
#
# Revision History:
#                    Modification    
# Changed by            Date             Description of Changes
#----------------   ------------      -------------------------
# zhou qiong         2004/1/13            change for ezx
# jiang Lili         2005/5/11            change for adding new config

O_TARGET := mmc.o

ifeq ($(CONFIG_ARCH_EZX_HAINAN),y)
export-objs     := mmc_core.o bvd-ezx_mmc.o
else
export-objs	:= mmc_core.o
endif

ifneq ($(CONFIG_USR_BLK_DEV), y)
  obj-$(CONFIG_MMC) += mmc_base.o
endif
obj-$(CONFIG_OMAP_MMC) += omap_mmc.o
obj-$(CONFIG_MAINSTONE_MMC) += mstone_mmc.o
obj-$(CONFIG_MX2MMC) += mx2_mmc.o
obj-$(CONFIG_EZX_MMC) += ezx_mmc.o
obj-$(CONFIG_S3CMMC) += samsung_mmc.o
obj-$(CONFIG_USR_BLK_DEV) += ezx_mmc.o

# Declare multi-part drivers.
list-multi := mmc_base.o mstone_mmc.o ezx_mmc.o

ifneq ($(CONFIG_USR_BLK_DEV), y)
  mmc_base-objs	:= mmc_protocol.o mmc_core.o mmc_media.o
endif

mstone_mmc-objs	:= bvd_mmc.o bvd-mstone_mmc.o
samsung_mmc-objs	:= s3c_mmc.o

ifeq ($(CONFIG_ARCH_S3C24A0),y)
samsung_mmc-objs +=  mmc_s3c24a0.o
endif

ifeq ($(CONFIG_ARCH_S3C2440),y)
samsung_mmc-objs +=  mmc_s3c2440.o
endif

ifeq ($(CONFIG_ARCH_EZX_TRIFLASH), y)
  ezx_mmc-objs	:= bvd_mmc.o bvd-ezx_mmc.o
endif
ifeq ($(CONFIG_USR_BLK_DEV), y)
  ezx_mmc-objs := usr_blk_dev.o
endif
ifeq ($(CONFIG_ARCH_EZX_SD), y)
#  mmc_base-objs	+= spi_protocol.o
   ezx_mmc-objs	:= bvd_mmc.o bvd-e680_mmc.o
endif

include $(TOPDIR)/Rules.make

# Link rules for multi-part drivers.

mmc_base.o: $(mmc_base-objs)
	$(LD) -r -o $@ $(mmc_base-objs)

mstone_mmc.o: $(mstone_mmc-objs)
	$(LD) -r -o $@ $(mstone_mmc-objs)

ezx_mmc.o: $(ezx_mmc-objs)
	$(LD) -r -o $@ $(ezx_mmc-objs)

samsung_mmc.o: $(samsung_mmc-objs)
	$(LD) -r -o $@ $(samsung_mmc-objs)
