#
# Makefile for the video capture/playback device drivers.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#
# Note 2! The CFLAGS definitions are now inherited from the
# parent makes..
#
#
# Copyright (C) 2003-2005 Motorola Inc.
# History:
#   Wang Fei,     Motorola, Dec 16, 2003, add Agilent ADCM2700 driver
#   Wang Wenxing, Motorola, Mar 15, 2004, add OV9640 driver
#   Ma Zhiqiang,  Motorola, Apr 13, 2004, add Micron MT9M111 driver
#   Wang Fei,     Motorola, Jul 14, 2004, add Micron MT9V111 driver
#   Mu Chenliang, Motorola, Jul 24, 2004, add OV9650 driver
#   Mu Chenliang, Motorola, Aug 03, 2004, add Agilent ADCM3800 driver
#   Mu Chenliang, Motorola, Mar 24, 2005, add MT9M111 light sensor device driver
#   Mu Chenliang, Motorola, May 31, 2005, add Micron MI2010SOC driver
# 

# Object file lists.

obj-y		:=
obj-m		:=
obj-n		:=
obj-		:=

SUB_DIRS     := 
MOD_SUB_DIRS := $(SUB_DIRS)
ALL_SUB_DIRS := $(SUB_DIRS)

O_TARGET := video.o

# All of the (potential) objects that export symbols.
# This list comes from 'grep -l EXPORT_SYMBOL *.[hc]'.

export-objs     :=	i2c-old.o videodev.o bttv-if.o cpia.o \
			 v4l2-common.o v4l1-compat.o

list-multi	:=	bttv.o zoran.o h3600_backpaq_camera.o
bttv-objs	:=	bttv-driver.o bttv-cards.o bttv-if.o

pxa-camera-objs :=	pxa_camera.o
  ifdef CONFIG_CAMERA_ADCM2700
    pxa-camera-objs +=	adcm2700.o adcm2700_hw.o mt9v111.o
  endif
  ifdef CONFIG_CAMERA_OV9640
    pxa-camera-objs +=  ov9640.o ov9640_hw.o
  endif
  ifdef CONFIG_CAMERA_OV9650
    pxa-camera-objs +=  ov9650.o ov9650_hw.o
  endif
  ifdef CONFIG_CAMERA_MT9M111
    pxa-camera-objs +=  mt9m111.o mt9m111_hw.o
  endif
  ifdef CONFIG_CAMERA_MI2010SOC
    pxa-camera-objs +=  mi2010soc.o mi2010soc_hw.o
  endif
  ifdef CONFIG_CAMERA_ADCM3800
    pxa-camera-objs +=  adcm3800.o adcm3800_hw.o
  endif
zoran-objs      :=	zr36120.o zr36120_i2c.o zr36120_mem.o
h3600-objs      :=      h3600_backpaq_core.o h3600_backpaq_algo.o

obj-$(CONFIG_VIDEO_DEV) += videodev.o v4l2-common.o v4l1-compat.o

obj-$(CONFIG_BUS_I2C) += i2c-old.o
obj-$(CONFIG_VIDEO_BT848) += bttv.o msp3400.o tvaudio.o \
	tda7432.o tda9875.o tuner.o
obj-$(CONFIG_SOUND_TVMIXER) += tvmixer.o

obj-$(CONFIG_VIDEO_ZR36120) += zoran.o
obj-$(CONFIG_I2C_PARPORT) += i2c-parport.o i2c-old.o
obj-$(CONFIG_VIDEO_SAA5249) += saa5249.o i2c-old.o
obj-$(CONFIG_VIDEO_CQCAM) += c-qcam.o
obj-$(CONFIG_VIDEO_BWQCAM) += bw-qcam.o
obj-$(CONFIG_VIDEO_W9966) += w9966.o
obj-$(CONFIG_VIDEO_ZORAN) += zr36067.o i2c-old.o
obj-$(CONFIG_VIDEO_ZORAN_BUZ) += saa7111.o saa7185.o
obj-$(CONFIG_VIDEO_ZORAN_DC10) += saa7110.o adv7175.o
obj-$(CONFIG_VIDEO_ZORAN_LML33) += bt819.o bt856.o
obj-$(CONFIG_VIDEO_CYBERPRO) += cyberpro.o i2c-old.o saa7111.o
obj-$(CONFIG_VIDEO_LML33) += bt856.o bt819.o
obj-$(CONFIG_VIDEO_PMS) += pms.o
obj-$(CONFIG_VIDEO_PLANB) += planb.o
obj-$(CONFIG_VIDEO_VINO) += vino.o
obj-$(CONFIG_VIDEO_STRADIS) += stradis.o
obj-$(CONFIG_VIDEO_CPIA) += cpia.o
obj-$(CONFIG_VIDEO_CPIA_PP) += cpia_pp.o
obj-$(CONFIG_VIDEO_CPIA_USB) += cpia_usb.o
obj-$(CONFIG_VIDEO_MEYE) += meye.o
obj-$(CONFIG_TUNER_3036) += tuner-3036.o
obj-$(CONFIG_PXA_CAMERA) += camera.o
obj-$(CONFIG_VIDEO_H3600_BACKPAQ) += h3600_backpaq_camera.o
obj-$(CONFIG_CAMERA_LIGHT_SENSOR) += light_sensor_cam.o

subdir-$(CONFIG_VIDEO_OMAP) += omap
ifeq ($(CONFIG_VIDEO_OMAP),y)
obj-y += omap/v4l2-omap.o
endif

# Extract lists of the multi-part drivers.
# The 'int-*' lists are the intermediate files used to build the multi's.

multi-y         := $(filter $(list-multi), $(obj-y))
multi-m         := $(filter $(list-multi), $(obj-m))
int-y           := $(sort $(foreach m, $(multi-y), $($(basename $(m))-objs)))
int-m           := $(sort $(foreach m, $(multi-m), $($(basename $(m))-objs)))

# Files that are both resident and modular: remove from modular.

obj-m           := $(filter-out $(obj-y), $(obj-m))
int-m           := $(filter-out $(int-y), $(int-m))

# Take multi-part drivers out of obj-y and put components in.

obj-y           := $(filter-out $(list-multi), $(obj-y)) $(int-y)

include $(TOPDIR)/Rules.make

fastdep:

zoran.o: zr36120.o zr36120_i2c.o zr36120_mem.o
	$(LD) $(LD_RFLAG) -r -o $@ zr36120.o zr36120_i2c.o zr36120_mem.o

bttv.o: $(bttv-objs)
	$(LD) $(LD_RFLAG) -r -o $@ $(bttv-objs)
camera.o: $(pxa-camera-objs)
	$(LD) $(LD_RFLAG) -r -o $@ $(pxa-camera-objs)
h3600_backpaq_camera.o: $(h3600-objs)
	$(LD) $(LD_RFLAG) -r -o $@ $(h3600-objs)
