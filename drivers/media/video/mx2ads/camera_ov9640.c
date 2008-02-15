/*
 * drivers/media/video/mx2ads/camera_ov9640.c
 *
 * Implementation of Camera for OmniVision OV9640 digital camera for MX2ADS
 * platform. This driver is copy of ov9640 camera driver for omap platform.
 * The difference is initializing of the camera. In original driver COM10
 * register initialize to 0x02
 *
 * Author: MontaVista Software, Inc.
 *              stevel@mvista.com or source@mvista.com
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */


#include <linux/config.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/videodev.h>
#include <asm/uaccess.h>
#include <asm/hardware/ov9640.h>
#define MODULE_NAME "ov9640"
#include "common.h"
#include "camif.h"

extern const struct image_size mx2ads_image_size[]; 
extern const int mx2ads_pixfmt_depth[];

static struct camera * this;

#define DEF_GAIN         31
#define DEF_AUTOGAIN      1
#define DEF_EXPOSURE    154
#define DEF_AEC           1
#define DEF_FREEZE_AGCAEC 0
#define DEF_BLUE        153
#define DEF_RED         (255 - DEF_BLUE)
#define DEF_AWB           1
#define DEF_HFLIP         0
#define DEF_VFLIP         0

/* Our own specific controls */
#define V4L2_CID_FREEZE_AGCAEC V4L2_CID_PRIVATE_BASE+0
#define V4L2_CID_AUTOEXPOSURE V4L2_CID_PRIVATE_BASE+1
#define V4L2_CID_LAST_PRIV  V4L2_CID_AUTOEXPOSURE

static int current_frame_period = 333667; /* .1 usec (30 fps) */
static int current_xclk = 24000000; /* Hz */

/*  Video controls  */
static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
	u8 reg;
	u8 mask;
	u8 start_bit;
} control[] = {
	{ { V4L2_CID_GAIN, "Gain", 0, 63, 1, DEF_GAIN,
	    V4L2_CTRL_TYPE_INTEGER },
	  0, OV9640_GAIN, 0x3f, 0 },
	{ { V4L2_CID_AUTOGAIN, "Auto Gain", 0, 1, 0, DEF_AUTOGAIN,
	    V4L2_CTRL_TYPE_BOOLEAN },
	  0, OV9640_COM8, 0x04, 2 },
	{ { V4L2_CID_EXPOSURE, "Exposure", 0, 255, 1, DEF_EXPOSURE,
	    V4L2_CTRL_TYPE_INTEGER },
	  0, OV9640_AECH, 0xff, 0 },
	{ { V4L2_CID_AUTOEXPOSURE, "Auto Exposure", 0, 1, 0, DEF_AEC,
	    V4L2_CTRL_TYPE_BOOLEAN },
	  0, OV9640_COM8, 0x01, 0 },
	{ { V4L2_CID_FREEZE_AGCAEC, "Freeze AGC/AEC", 0,1,0, DEF_FREEZE_AGCAEC,
	    V4L2_CTRL_TYPE_BOOLEAN },
	  0, OV9640_COM9, 0x01, 0 },
	{ { V4L2_CID_RED_BALANCE, "Red Balance", 0, 255, 1, DEF_RED,
	    V4L2_CTRL_TYPE_INTEGER },
	  0, OV9640_RED, 0xff, 0 },
	{ { V4L2_CID_BLUE_BALANCE, "Blue Balance", 0, 255, 1, DEF_BLUE,
	    V4L2_CTRL_TYPE_INTEGER },
	  0, OV9640_BLUE, 0xff, 0 },
	{ { V4L2_CID_AUTO_WHITE_BALANCE, "Auto White Balance", 0,1,0, DEF_AWB,
	    V4L2_CTRL_TYPE_BOOLEAN },
	  0, OV9640_COM8, 0x02, 1 },
	{ { V4L2_CID_HFLIP, "Mirror Image", 0, 1, 0, DEF_HFLIP,
	    V4L2_CTRL_TYPE_BOOLEAN },
	  0, OV9640_MVFP, 0x20, 5 },
	{ { V4L2_CID_VFLIP, "Vertical Flip", 0, 1, 0, DEF_VFLIP,
	    V4L2_CTRL_TYPE_BOOLEAN },
	  0, OV9640_MVFP, 0x10, 4 },
};

#define NUM_CONTROLS (sizeof(control)/sizeof(control[0]))

/* register initialization tables for OV9640 */

#define OV9640_REG_TERM 0xFF	/* terminating list entry for reg */
#define OV9640_VAL_TERM 0xFF	/* terminating list entry for val */

/* define a structure for ov9640 register initialization values */
struct ov9640_reg {
	unsigned char reg;
	unsigned char val;
};

/* common OV9640 register initialization for all image sizes, pixel formats, 
 * and frame rates.
 */
const static struct ov9640_reg ov9640_common[] = {
	{ 0x12, 0x80 }, { 0x11, 0x80 }, { 0x13, 0x88 }, /* COM7,CLKRC,COM8 */
	{ 0x01, 0x80 }, { 0x02, 0x80 }, { 0x04, 0x00 },	/* BLUE, RED, COM1 */
	{ 0x0E, 0x89 }, { 0x0F, 0x4F }, { 0x14, 0x4A },	/* COM5, COM6, COM9 */
	{ 0x16, 0x02 }, { 0x1B, 0x01 }, { 0x24, 0x70 },	/* ?, PSHFT, AEW */
	{ 0x25, 0x68 }, { 0x26, 0xD3 }, { 0x27, 0x90 },	/* AEB, VPT, BBIAS */
	{ 0x2A, 0x00 }, { 0x2B, 0x00 }, { 0x32, 0x24 },	/* EXHCH, EXHCL, HREF */
	{ 0x33, 0x02 }, { 0x37, 0x02 }, { 0x38, 0x13 },	/* CHLF, ADC, ACOM */
	{ 0x39, 0xF0 }, { 0x3A, 0x00 }, { 0x3B, 0x01 },	/* OFON, TSLB, COM11 */
	{ 0x3D, 0x90 }, { 0x3E, 0x02 }, { 0x3F, 0xF2 },	/* COM13, COM14, EDGE */
	{ 0x41, 0x02 }, { 0x42, 0xC9 },			/* COM16, COM17 */
	{ 0x43, 0xF0 }, { 0x44, 0x10 }, { 0x45, 0x6C },	/* ?, ?, ? */
	{ 0x46, 0x6C }, { 0x47, 0x44 }, { 0x48, 0x44 },	/* ?, ?, ? */
	{ 0x49, 0x03 }, { 0x59, 0x49 }, { 0x5A, 0x94 },	/* ?, ?, ? */
	{ 0x5B, 0x46 }, { 0x5C, 0x84 }, { 0x5D, 0x5C },	/* ?, ?, ? */
	{ 0x5E, 0x08 }, { 0x5F, 0x00 }, { 0x60, 0x14 },	/* ?, ?, ? */
	{ 0x61, 0xCE },					/* ? */
	{ 0x62, 0x70 }, { 0x63, 0x00 }, { 0x64, 0x04 },	/* LCC1, LCC2, LCC3 */
	{ 0x65, 0x00 }, { 0x66, 0x00 },			/* LCC4, LCC5 */
	{ 0x69, 0x00 }, { 0x6A, 0x3E }, { 0x6B, 0x3F },	/* HV, MBD, DBLV */
	{ 0x6C, 0x40 }, { 0x6D, 0x30 }, { 0x6E, 0x4B },	/* GSP1, GSP2, GSP3 */
	{ 0x6F, 0x60 }, { 0x70, 0x70 }, { 0x71, 0x70 },	/* GSP4, GSP5, GSP6 */
	{ 0x72, 0x70 }, { 0x73, 0x70 }, { 0x74, 0x60 },	/* GSP7, GSP8, GSP9 */
	{ 0x75, 0x60 }, { 0x76, 0x50 }, { 0x77, 0x48 },	/* GSP10,GSP11,GSP12 */
	{ 0x78, 0x3A }, { 0x79, 0x2E }, { 0x7A, 0x28 },	/* GSP13,GSP14,GSP15 */
	{ 0x7B, 0x22 }, { 0x7C, 0x04 }, { 0x7D, 0x07 },	/* GSP16,GST1, GST2 */
	{ 0x7E, 0x10 }, { 0x7F, 0x28 }, { 0x80, 0x36 },	/* GST3, GST4, GST5 */
	{ 0x81, 0x44 }, { 0x82, 0x52 }, { 0x83, 0x60 },	/* GST6, GST7, GST8 */
	{ 0x84, 0x6C }, { 0x85, 0x78 }, { 0x86, 0x8C },	/* GST9, GST10,GST11 */
	{ 0x87, 0x9E }, { 0x88, 0xBB }, { 0x89, 0xD2 },	/* GST12,GST13,GST14 */
	{ 0x8A, 0xE6 }, { 0x13, 0x8F }, { 0x15, 0x02 },	/* GST15,COM8,COM10 */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};

/* OV9640 register configuration for all combinations of pixel format and 
 * image size
 */
	/* YUV (YCbCr) QQCIF */
const static struct ov9640_reg qqcif_yuv[] = {
	{ 0x12, 0x08 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) QQVGA */
const static struct ov9640_reg qqvga_yuv[] = {
	{ 0x12, 0x10 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) QCIF */
const static struct ov9640_reg qcif_yuv[] = {
	{ 0x12, 0x08 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) QVGA */
const static struct ov9640_reg qvga_yuv[] = {
	{ 0x12, 0x10 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) CIF */
const static struct ov9640_reg cif_yuv[] = {
	{ 0x12, 0x20 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) VGA */
const static struct ov9640_reg vga_yuv[] = {
	{ 0x12, 0x40 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) SXGA */
const static struct ov9640_reg sxga_yuv[] = {
	{ 0x12, 0x00 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 QQCIF */
const static struct ov9640_reg qqcif_565[] = {
	{ 0x12, 0x0C }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 QQVGA */
const static struct ov9640_reg qqvga_565[] = {
	{ 0x12, 0x14 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 QCIF */
const static struct ov9640_reg qcif_565[] = {
	{ 0x12, 0x0C }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 QVGA */
const static struct ov9640_reg qvga_565[] = {
	{ 0x12, 0x14 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 CIF */
const static struct ov9640_reg cif_565[] = {
	{ 0x12, 0x24 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 VGA */
const static struct ov9640_reg vga_565[] = {
	{ 0x12, 0x44 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 SXGA */
const static struct ov9640_reg sxga_565[] = {
	{ 0x12, 0x04 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 QQCIF */
const static struct ov9640_reg qqcif_555[] = {
	{ 0x12, 0x0C }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 QQVGA */
const static struct ov9640_reg qqvga_555[] = {
	{ 0x12, 0x14 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 QCIF */
const static struct ov9640_reg qcif_555[] = {
	{ 0x12, 0x0C }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 QVGA */
const static struct ov9640_reg qvga_555[] = {
	{ 0x12, 0x14 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 CIF */
const static struct ov9640_reg cif_555[] = {
	{ 0x12, 0x24 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 VGA */
const static struct ov9640_reg vga_555[] = {
	{ 0x12, 0x44 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 SXGA */
const static struct ov9640_reg sxga_555[] = {
	{ 0x12, 0x04 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};

const static struct ov9640_reg *
	ov9640_reg_init[NUM_PIXEL_FORMATS][NUM_IMAGE_SIZES] =
{
 { qqcif_yuv, qqvga_yuv, qcif_yuv, qvga_yuv, cif_yuv, vga_yuv, sxga_yuv },
 { qqcif_565, qqvga_565, qcif_565, qvga_565, cif_565, vga_565, sxga_565 },
 { qqcif_555, qqvga_555, qcif_555, qvga_555, cif_555, vga_555, sxga_555 },
};


/* Initialize a list of OV9640 registers.
 * The list of registers is terminated by the pair of values 
 * { OV9640_REG_TERM, OV9640_VAL_TERM }.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov_write_reglist(const struct ov9640_reg reglist[])
{
	struct camera_serial_bus * sbus = this->camif->sbus;
	const struct ov9640_reg *next = reglist;
	int rc;
	
	while (!((next->reg == OV9640_REG_TERM) 
		&& (next->val == OV9640_VAL_TERM)))
	{
		rc = sbus->write(next->reg, (u8*)&next->val, 1);
		if (rc)
			return rc;
		next++;
	}
	return 0;
}

static int
find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE || id > V4L2_CID_LAST_PRIV)
		return -EDOM;
	for (i = NUM_CONTROLS - 1; i >= 0; i--)
		if (control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

static int
ov9640_querymenu(struct v4l2_querymenu *qm)
{
	/* No menu controls have been defined */
	return -EINVAL;
}

static int
ov9640_query_control(struct v4l2_queryctrl *qc)
{
	int i;

	i = find_vctrl(qc->id);
	if (i == -EINVAL) {
		qc->flags = V4L2_CTRL_FLAG_DISABLED;
		return 0;
	}
	if (i < 0)
		return -EINVAL;

	/*  V4L2 filled in category and group, preserve them */
	control[i].qc.category = qc->category;
	memcpy(control[i].qc.group, qc->group, sizeof(qc->group));

	*qc = control[i].qc;
	return 0;
}


static int ov9640_write_control(struct vcontrol * lvc, u8 value)
{
	struct camera_serial_bus * sbus = this->camif->sbus;
	u8 oldval, newval;
	u8 reg = lvc->reg;
	u8 mask = lvc->mask;
	int rc;
	
	value <<= lvc->start_bit;

	if (mask == 0xff) {
		newval = value;
	} else {
		if ((rc = sbus->read(reg, &oldval, 1)))
			return rc;
		
		oldval &= ~mask;              /* Clear the masked bits */
		value &= mask;                /* Enforce mask on value */
		newval = oldval | value;      /* Set the desired bits */
	}
	
	if ((rc = sbus->write(reg, &newval, 1)))
		return rc;
	if ((rc = sbus->read(reg, &newval, 1)))
		return rc;

	return (newval & mask) >> lvc->start_bit;
}

static int ov9640_read_control(struct vcontrol * lvc)
{
	struct camera_serial_bus * sbus = this->camif->sbus;
	u8 val;
	u8 reg = lvc->reg;
	u8 mask = lvc->mask;
	int rc;
	
	if ((rc = sbus->read(reg, &val, 1)))
		return rc;
	return (val & mask) >> lvc->start_bit;
}

static int
ov9640_control(struct v4l2_control *vc, int write)
{
	int i, val;
	struct vcontrol * lvc;
	
	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;

	lvc = &control[i];
	
	if (write)
		val = ov9640_write_control(lvc, vc->value);
	else
		val = ov9640_read_control(lvc);
	
	if (val >= 0) {
		vc->value = lvc->current_value = val;
		return 0;
	} else
		return val;
}


static int
ov9640_set_control(struct v4l2_control *vc)
{
	return ov9640_control(vc, 1);
}
static int
ov9640_get_control(struct v4l2_control *vc)
{
	return ov9640_control(vc, 0);
}


static int
ov9640_restore_current_controls(void)
{
	struct v4l2_control vc;
	int i, rc=0;
	
	for (i=0; i<NUM_CONTROLS; i++) {
		vc.id = control[i].qc.id;
		vc.value = control[i].current_value;
		if ((rc = ov9640_set_control(&vc)))
			break;
	}

	return rc;
}

static int
ov9640_save_current_controls(void)
{
	struct v4l2_control vc;
	int i, rc=0;
	
	for (i=0; i<NUM_CONTROLS; i++) {
		vc.id = control[i].qc.id;
		if ((rc = ov9640_get_control(&vc)))
			break;
	}

	return rc;
}

struct fract {
	unsigned long numerator;
	unsigned long denominator;
};	

/* Calculate the internal clock divisor (value of the CLKRC register) of the 
 * OV9640 given the image size, the frequency (in Hz) of its XCLK input and a 
 * desired frame period (in seconds).  The frame period 'fper' is expressed as 
 * a fraction.  The frame period is an input/output parameter.
 * Returns the value of the OV9640 CLKRC register that will yield the frame 
 * period returned in 'fper' at the specified xclk frequency.  The 
 * returned period will be as close to the requested period as possible.
 */
static unsigned char
ov9640_clkrc(unsigned long xclk, struct fract *fper)
{
	unsigned long fpm, fpm_max;	/* frames per minute */
	unsigned long divisor;
	const unsigned long divisor_max = 64;
	const static unsigned long clks_per_frame[] = 
		{ 200000, 400000, 200000, 400000, 400000, 800000, 3200000 };
	/*         QQCIF   QQVGA    QCIF    QVGA     CIF     VGA     SXGA
	 *actually 199680,400000, 199680, 400000, 399360, 800000, 3200000
	 */ 

	if (fper->numerator > 0)
		fpm = (fper->denominator*60)/fper->numerator;
	else
		fpm = 0xffffffff;
	fpm_max = (xclk*60)/clks_per_frame[this->imgfmt];
	if (fpm_max == 0)
		fpm_max = 1;
	if (fpm > fpm_max)
		fpm = fpm_max;
	if (fpm == 0)
		fpm = 1;
	divisor = fpm_max/fpm;
	if (divisor > divisor_max)
		divisor = divisor_max;
	fper->numerator = divisor*60;
	fper->denominator = fpm_max;

	/* try to reduce the fraction */
	while (!(fper->denominator % 5) && !(fper->numerator % 5)) {
		fper->numerator /= 5;
		fper->denominator /= 5;
	}
	while (!(fper->denominator % 3) && !(fper->numerator % 3)) {
		fper->numerator /= 3;
		fper->denominator /= 3;
	}
	while (!(fper->denominator % 2) && !(fper->numerator % 2)) {
		fper->numerator /= 2;
		fper->denominator /= 2;
	}
	if (fper->numerator < fper->denominator) {
		if (!(fper->denominator % fper->numerator)) {
			fper->denominator /= fper->numerator;
			fper->numerator = 1;
		}
	}
	else {
		if (!(fper->numerator % fper->denominator)) {
			fper->numerator /= fper->denominator;
			fper->denominator = 1;
		}
	}

	/* we set bit 7 in CLKRC to enable the digital PLL */
	return (0x80 | (divisor - 1));
}


static inline void cvt_fp_to_fract(struct fract * fper, int fp)
{
	fper->numerator = fp;
	fper->denominator = 10000000;
}
static inline int cvt_fract_to_fp(struct fract * fper)
{
	return (fper->numerator * 10000000) / fper->denominator;
}

static int ov9640_set_fp(int fp, int xclk, int test)
{
	struct camera_serial_bus * sbus = this->camif->sbus;
	struct fract fper;
	int retfp;
	u8 clkrc;
	
	cvt_fp_to_fract(&fper, fp);
	/* configure frame rate */
	clkrc = ov9640_clkrc(xclk, &fper);
	retfp = cvt_fract_to_fp(&fper);

	if (!test) {
		int err = sbus->write(OV9640_CLKRC, &clkrc, 1);
		if (err)
			return err;
		current_frame_period = retfp;
		current_xclk = xclk;
	}

	return retfp;
}

static int ov9640_get_fp(void)
{
	return current_frame_period;
}


/*
 * Configure the OV9640 for the current image size, pixel format, and
 * frame rate. Returns zero if successful, or non-zero otherwise.
 */
static int
ov9640_configure(void)
{
	int err;

	/* common register initialization */
	err = ov_write_reglist(ov9640_common);
	if (err)
		return err;

	/* configure image size and pixel format */
	err = ov_write_reglist(ov9640_reg_init[this->pixfmt][this->imgfmt]);
	if (err)
		return err;

	/* restore the current controls */
	err = ov9640_restore_current_controls();
	if (err)
		return err;

	/*
	 * and finally set the internal clock to generate the
	 * current frame rate for this image format
	 */
	err = ov9640_set_fp(current_frame_period, current_xclk, 0);
	if (err < 0)
		return err;

	return 0;
}


/* Convert a Video for Linux pixelformat to an enum pixel_format value.
 */
static int
ov9640_find_format(struct v4l2_pix_format* fmt)
{
	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		return YUV;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
	case V4L2_PIX_FMT_BGR24: /* will have to convert to BGR24 */
		return RGB565;
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB555X:
		return RGB555;
	}

	return -1;
}

/*
 * Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 */
static int
ov9640_find_size(struct v4l2_pix_format* fmt)
{
        int isize;

        for (isize = QQCIF; isize < SXGA; isize++) {
                if ((mx2ads_image_size[isize].height >= fmt->height) &&
		    (mx2ads_image_size[isize].width >= fmt->width)) {
			fmt->width = mx2ads_image_size[isize].width;
			fmt->height = mx2ads_image_size[isize].height;
                        return isize;
                }
        }

	fmt->width = mx2ads_image_size[SXGA].width;
	fmt->height = mx2ads_image_size[SXGA].height;
        return SXGA;
}


static int ov9640_setup(struct v4l2_pix_format* fmt)
{
	enum pixel_format pixfmt;
	enum image_format imgfmt;

	if ((pixfmt = ov9640_find_format(fmt)) < 0)
		return -EINVAL;
	
	imgfmt = ov9640_find_size(fmt);

	this->imgfmt = imgfmt;
	this->pixfmt = pixfmt;

	return ov9640_configure();
}


static int ov9640_open(void)
{
	return ov9640_configure();
}

static int ov9640_close(void)
{
	/*
	 * save the current controls so they can be restored on
	 * next open.
	 */
	return ov9640_save_current_controls();
}

static int ov9640_init(void)
{
	int i;

	for (i=0; i<NUM_CONTROLS; i++)
		control[i].current_value = control[i].qc.default_value;
	
	this->imgfmt = QVGA;
	this->pixfmt = RGB565;
	return 0;
}

static void ov9640_cleanup(void)
{
	/* nothing to do */
}


static int ov9640_detect(void)
{
	struct camera_serial_bus * sbus;
	u8 midh, midl, pid, ver;
	int rc;
	
	this = &camera_ov9640;
	sbus = this->camif->sbus;
	
	sbus->set_devid(CAMERA_OV9640_DEV_ID);

	if ((rc = sbus->read(OV9640_MIDH, &midh, 1)))
		return rc;
	if ((rc = sbus->read(OV9640_MIDL, &midl, 1)))
		return rc;
	if ((rc = sbus->read(OV9640_PID, &pid, 1)))
		return rc;
	if ((rc = sbus->read(OV9640_VER, &ver, 1)))
		return rc;

	if ((midh != OV9640_MIDH_MAGIC) 
		|| (midl != OV9640_MIDL_MAGIC)
		|| (pid != OV9640_PID_MAGIC))
	{
		/* We didn't read the values we expected, so 
		 * this must not be an OV9640.
		 */
		return -ENODEV;
	}

	return 0;
}

struct camera camera_ov9640 = {
	imgfmt: QVGA,
	pixfmt: RGB565,
	
	detect:  ov9640_detect,
	init:    ov9640_init,
	cleanup: ov9640_cleanup,

	open:    ov9640_open,
	close:   ov9640_close,
	set_format:   ov9640_setup,
	set_frame_period: ov9640_set_fp,
	get_frame_period: ov9640_get_fp,
	
	query_control: ov9640_query_control,
	get_control:   ov9640_get_control,
	set_control:   ov9640_set_control,
	query_menu:    ov9640_querymenu,
};

