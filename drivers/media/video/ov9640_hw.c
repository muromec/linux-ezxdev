/* 
    ov9640_hw.c - Omnivision 9640 CMOS sensor driver 

    Copyright (C) 2003, Intel Corporation
    Copyright (C) 2004 Motorola Inc.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

/*
Revision History:
                            Modification     Tracking
Author                 Date          Number     Description of Changes
----------------   ------------    ----------   -------------------------
Wang Wenxing)      03/16/2004                   Porting to EZX platform   
Wang Wenxing)      07/30/2004                   Improve, tunning
*/

#include <linux/types.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/wrapper.h>
#include <linux/delay.h>
#include <linux/videodev.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/dma.h>
#include <asm/irq.h>

#include <linux/i2c.h>
#include "camera.h"
#include "ov9640.h"
#include "ov9640_hw.h"

ov9640 g_ov;
/***********************************************************************
*  Attention: This is platform related!
***********************************************************************/
volatile int ov9640_step = 0;

/***********************************************************************
*  Register Settings
***********************************************************************/
static u8 gDelta[] = {
	OV9640_COM7,		//0x12
	OV9640_COM1,		//0x04
//      OV9640_CLKRC,           //0x11
	OV9640_COM4,		//0x0d
	OV9640_COM3,		//0x0c
	OV9640_REGEND		// End of list delimiter.        
};
const static u8 gYUV_QQVGA[] = {
	OV9640_COM10, 0x20,
	OV9640_COM7, 0x10,
	OV9640_COM1, 0x24,
	OV9640_CLKRC, 0x01,

	// From OmniVision
	OV9640_COM8, 0x8D,
	OV9640_BLUE, 0x80,
	OV9640_RED, 0x80,
	OV9640_COM3, 0x5,	//0x04
	OV9640_COM4, 0xC0,
	OV9640_COM6, 0x4E,	//0x4F
	OV9640_COM9, 0x4A,	//0x0A
	OV9640_PSHFT, 0x1,
	OV9640_AEW, 0x78,
	OV9640_AEB, 0x70,
	OV9640_BBIAS, 0x90,
	OV9640_CHLF, 0x8,
	OV9640_VIDO, 0xC9,
	OV9640_TSLB, 0x8,
	OV9640_COM11, 0x8,
	OV9640_COM12, 0x46,
	OV9640_COM13, 0x90,
	OV9640_COM14, 0x2,
	OV9640_EDGE, 0xA4,
	OV9640_COM16, 0x2,
	OV9640_COM17, 0xC8,
	OV9640_AWBTH1, 0xF0,
	OV9640_AWBTH2, 0x10,
	OV9640_AWBTH3, 0x5C,
	OV9640_AWBTH4, 0x70,
	OV9640_AWBTH5, 0x46,
	OV9640_AWBTH6, 0x4D,
	OV9640_MTX1, 0x63,
	OV9640_MTX2, 0x4E,
	OV9640_MTX3, 0x15,
	OV9640_MTX4, 0x1D,
	OV9640_MTX5, 0xBE,
	OV9640_MTX6, 0xDC,
	OV9640_MTXS, 0xF,
	OV9640_AWBC1, 0x1F,
	OV9640_AWBC2, 0x55,
	OV9640_AWBC3, 0x43,
	OV9640_AWBC4, 0x7C,
	OV9640_AWBC5, 0x54,
	OV9640_AWBC6, 0xA,
	OV9640_AWBC7, 0x5,
	OV9640_AWBC8, 0x14,
	OV9640_AWBC9, 0xCE,
	OV9640_DBLV, 0x3A,
	OV9640_GST0, 0x2,
	OV9640_GSP0, 0x40,
	OV9640_GSP1, 0x30,
	OV9640_GSP2, 0x4B,
	OV9640_GSP3, 0x60,
	OV9640_GSP4, 0x70,
	OV9640_GSP5, 0x70,
	OV9640_GSP6, 0x70,
	OV9640_GSP7, 0x70,
	OV9640_GSP8, 0x60,
	OV9640_GSP9, 0x60,
	OV9640_GSP10, 0x50,
	OV9640_GSP11, 0x48,
	OV9640_GSP12, 0x3A,
	OV9640_GSP13, 0x2E,
	OV9640_GSP14, 0x28,
	OV9640_GSP15, 0x22,
	OV9640_GST0, 0x4,
	OV9640_GST1, 0x7,
	OV9640_GST2, 0x10,
	OV9640_GST3, 0x28,
	OV9640_GST4, 0x36,
	OV9640_GST5, 0x44,
	OV9640_GST6, 0x52,
	OV9640_GST7, 0x60,
	OV9640_GST8, 0x6C,
	OV9640_GST9, 0x78,
	OV9640_GST10, 0x8C,
	OV9640_GST11, 0x9E,
	OV9640_GST12, 0xBB,
	OV9640_GST13, 0xD2,
	OV9640_GST14, 0xE6,
	OV9640_COM8, 0x8F,
	OV9640_REGEND, 0x00	// End of list delimiter.
};

const static u8 gYUV_QQCIF[] = {
	OV9640_COM10, 0x20,
	OV9640_COM7, 0x08,
	OV9640_COM1, 0x24,
	OV9640_CLKRC, 0x01,

	// From OmniVision
	OV9640_COM8, 0x8D,
	OV9640_BLUE, 0x80,
	OV9640_RED, 0x80,
	OV9640_COM3, 0x5,	//0x04
	OV9640_COM4, 0xC0,
	OV9640_COM6, 0x4E,	//0x4F
	OV9640_COM9, 0x4A,	//0x0A
	OV9640_PSHFT, 0x1,
	OV9640_AEW, 0x78,
	OV9640_AEB, 0x70,
	OV9640_BBIAS, 0x90,
	OV9640_CHLF, 0x8,
	OV9640_VIDO, 0xC9,
	OV9640_TSLB, 0x8,
	OV9640_COM11, 0x8,
	OV9640_COM12, 0x46,
	OV9640_COM13, 0x90,
	OV9640_COM14, 0x2,
	OV9640_EDGE, 0xA4,
	OV9640_COM16, 0x2,
	OV9640_COM17, 0xC8,
	OV9640_AWBTH1, 0xF0,
	OV9640_AWBTH2, 0x10,
	OV9640_AWBTH3, 0x5C,
	OV9640_AWBTH4, 0x70,
	OV9640_AWBTH5, 0x46,
	OV9640_AWBTH6, 0x4D,
	OV9640_MTX1, 0x63,
	OV9640_MTX2, 0x4E,
	OV9640_MTX3, 0x15,
	OV9640_MTX4, 0x1D,
	OV9640_MTX5, 0xBE,
	OV9640_MTX6, 0xDC,
	OV9640_MTXS, 0xF,
	OV9640_AWBC1, 0x1F,
	OV9640_AWBC2, 0x55,
	OV9640_AWBC3, 0x43,
	OV9640_AWBC4, 0x7C,
	OV9640_AWBC5, 0x54,
	OV9640_AWBC6, 0xA,
	OV9640_AWBC7, 0x5,
	OV9640_AWBC8, 0x14,
	OV9640_AWBC9, 0xCE,
	OV9640_DBLV, 0x3A,
	OV9640_GST0, 0x2,
	OV9640_GSP0, 0x40,
	OV9640_GSP1, 0x30,
	OV9640_GSP2, 0x4B,
	OV9640_GSP3, 0x60,
	OV9640_GSP4, 0x70,
	OV9640_GSP5, 0x70,
	OV9640_GSP6, 0x70,
	OV9640_GSP7, 0x70,
	OV9640_GSP8, 0x60,
	OV9640_GSP9, 0x60,
	OV9640_GSP10, 0x50,
	OV9640_GSP11, 0x48,
	OV9640_GSP12, 0x3A,
	OV9640_GSP13, 0x2E,
	OV9640_GSP14, 0x28,
	OV9640_GSP15, 0x22,
	OV9640_GST0, 0x4,
	OV9640_GST1, 0x7,
	OV9640_GST2, 0x10,
	OV9640_GST3, 0x28,
	OV9640_GST4, 0x36,
	OV9640_GST5, 0x44,
	OV9640_GST6, 0x52,
	OV9640_GST7, 0x60,
	OV9640_GST8, 0x6C,
	OV9640_GST9, 0x78,
	OV9640_GST10, 0x8C,
	OV9640_GST11, 0x9E,
	OV9640_GST12, 0xBB,
	OV9640_GST13, 0xD2,
	OV9640_GST14, 0xE6,
	OV9640_COM8, 0x8F,
	OV9640_REGEND, 0x00	// End of list delimiter.
};

const static u8 gYUV_QVGA1[] = {
	OV9640_COM10, 0x20,
	OV9640_COM7, 0x10,
	OV9640_COM1, 0x00,
	OV9640_CLKRC, 0x01,

	// From OmniVision
	OV9640_COM8, 0x8D,
	OV9640_BLUE, 0x80,
	OV9640_RED, 0x80,
	OV9640_COM3, 0x5,	//0x04
	OV9640_COM4, 0xC0,
	OV9640_COM6, 0x4E,	//0x4F
	OV9640_COM9, 0x4A,	//0x0A
	OV9640_PSHFT, 0x1,
	OV9640_AEW, 0x78,
	OV9640_AEB, 0x70,
	OV9640_BBIAS, 0x90,
	OV9640_CHLF, 0x8,
	OV9640_VIDO, 0xC9,
	OV9640_TSLB, 0x8,
	OV9640_COM11, 0x8,
	OV9640_COM12, 0x46,
	OV9640_COM13, 0x90,
	OV9640_COM14, 0x2,
	OV9640_EDGE, 0xA4,
	OV9640_COM16, 0x2,
	OV9640_COM17, 0xC8,
	OV9640_AWBTH1, 0xF0,
	OV9640_AWBTH2, 0x10,
	OV9640_AWBTH3, 0x5C,
	OV9640_AWBTH4, 0x70,
	OV9640_AWBTH5, 0x46,
	OV9640_AWBTH6, 0x4D,
	OV9640_MTX1, 0x63,
	OV9640_MTX2, 0x4E,
	OV9640_MTX3, 0x15,
	OV9640_MTX4, 0x1D,
	OV9640_MTX5, 0xBE,
	OV9640_MTX6, 0xDC,
	OV9640_MTXS, 0xF,
	/*
	   OV9640_MTX1     , 0x3c,
	   OV9640_MTX2     , 0x32,
	   OV9640_MTX3     , 0x09,
	   OV9640_MTX4     , 0x13,
	   OV9640_MTX5     , 0x39,
	   OV9640_MTX6     , 0x4c,
	 */

	OV9640_AWBC1, 0x1F,
	OV9640_AWBC2, 0x55,
	OV9640_AWBC3, 0x43,
	OV9640_AWBC4, 0x7C,
	OV9640_AWBC5, 0x54,
	OV9640_AWBC6, 0xA,
	OV9640_AWBC7, 0x5,
	OV9640_AWBC8, 0x14,
	OV9640_AWBC9, 0xCE,
	OV9640_DBLV, 0x3A,
	OV9640_GST0, 0x2,
	OV9640_GSP0, 0x40,
	OV9640_GSP1, 0x30,
	OV9640_GSP2, 0x4B,
	OV9640_GSP3, 0x60,
	OV9640_GSP4, 0x70,
	OV9640_GSP5, 0x70,
	OV9640_GSP6, 0x70,
	OV9640_GSP7, 0x70,
	OV9640_GSP8, 0x60,
	OV9640_GSP9, 0x60,
	OV9640_GSP10, 0x50,
	OV9640_GSP11, 0x48,
	OV9640_GSP12, 0x3A,
	OV9640_GSP13, 0x2E,
	OV9640_GSP14, 0x28,
	OV9640_GSP15, 0x22,
	OV9640_GST0, 0x4,
	OV9640_GST1, 0x7,
	OV9640_GST2, 0x10,
	OV9640_GST3, 0x28,
	OV9640_GST4, 0x36,
	OV9640_GST5, 0x44,
	OV9640_GST6, 0x52,
	OV9640_GST7, 0x60,
	OV9640_GST8, 0x6C,
	OV9640_GST9, 0x78,
	OV9640_GST10, 0x8C,
	OV9640_GST11, 0x9E,
	OV9640_GST12, 0xBB,
	OV9640_GST13, 0xD2,
	OV9640_GST14, 0xE6,
	OV9640_COM8, 0x8F,
	OV9640_REGEND, 0x00	// End of list delimiter.
};

const static u8 gYUV_VGA[] = {
	0x15, 0x20,
	0x12, 0x40,
	0x04, 0x00,
	0x11, 0x83,
	//0x01  , 0x80,
	//0x02  , 0x80,
	0x0C, 0x05,
	0x0D, 0xC0,
	0x0E, 0x01,
	0x0F, 0x4F,
	0x13, 0x82,		//0x80
	0x14, 0x4A,
	0x1B, 0x01,
	0x24, 0x78,
	0x25, 0x70,
	0x27, 0x90,
	0x33, 0x08,
	0x36, 0xC9,
	0x3A, 0x08,
	0x3B, 0x00,
	0x3C, 0x46,
	0x3D, 0x90,
	0x3E, 0x02,
	0x3F, 0xF2,
	0x41, 0x02,
	0x42, 0xC8,
	0x43, 0xF0,
	0x44, 0x10,
	0x45, 0x5C,
	0x46, 0x70,
	0x47, 0x46,
	0x48, 0x4D,
	0x4F, 0x3C,
	0x50, 0x32,
	0x51, 0x09,
	0x52, 0x13,
	0x53, 0x39,
	0x54, 0x4C,
	0x59, 0x1F,
	0x5A, 0x55,
	0x5B, 0x43,
	0x5C, 0x7C,
	0x5D, 0x54,
	0x5E, 0x0A,
	0x5F, 0x01,
	0x60, 0x94,
	0x61, 0xCE,
	0x6B, 0x3A,
	0x6C, 0x40,
	0x6D, 0x30,
	0x6E, 0x4B,
	0x6F, 0x60,
	0x70, 0x70,
	0x71, 0x70,
	0x72, 0x70,
	0x73, 0x70,
	0x74, 0x60,
	0x75, 0x60,
	0x76, 0x50,
	0x77, 0x48,
	0x78, 0x3A,
	0x79, 0x2E,
	0x7A, 0x28,
	0x7B, 0x22,
	0x7C, 0x04,
	0x7D, 0x07,
	0x7E, 0x10,
	0x7F, 0x28,
	0x80, 0x36,
	0x81, 0x44,
	0x82, 0x52,
	0x83, 0x60,
	0x84, 0x6C,
	0x85, 0x78,
	0x86, 0x8C,
	0x87, 0x9E,
	0x88, 0xBB,
	0x89, 0xD2,
	0x8A, 0xE6,
	0x13, 0xaF,
	OV9640_REGEND, 0x00	// End of list delimiter.
};
const static u8 gYUV_QVGA[] = {
	0x15, 0x20,
	0x12, 0x10,
	0x04, 0x00,
	0x11, 0x81,
	0x01, 0x80,
	0x02, 0x24,
	0x0C, 0x05,
	0x0D, 0xC0,
	0x0E, 0x01,
	0x0F, 0x4F,
	0x13, 0x80,		//0x82
	0x14, 0x8A,
	0x1B, 0x01,
	0x24, 0x90,
	0x25, 0x80,
	0x2A, 0x00,
	0x2b, 0x00,
	0x27, 0x90,
	0x33, 0x08,
	0x36, 0xC9,
	0x3A, 0x08,
	0x3B, 0x00,
	0x3C, 0x46,
	0x3D, 0x90,
	0x3E, 0x02,
	0x3F, 0xa4,
	0x41, 0x02,
	0x42, 0xC8,
	0x43, 0xF0,
	0x44, 0x10,
	0x45, 0x5C,
	0x46, 0x70,
	0x47, 0x46,
	0x48, 0x4D,
	0x4F, 0x3C,
	0x50, 0x32,
	0x51, 0x09,
	0x52, 0x13,
	0x53, 0x39,
	0x54, 0x4C,
	0x59, 0x1F,
	0x5A, 0x55,
	0x5B, 0x43,
	0x5C, 0x7C,
	0x5D, 0x54,
	0x5E, 0x0A,
	0x5F, 0x01,
	0x60, 0x94,
	0x61, 0xCE,
	0x6B, 0x3A,
	0x6C, 0x50,
	0x6D, 0x60,
	0x6E, 0x58,
	0x6F, 0x58,
	0x70, 0x58,
	0x71, 0x50,
	0x72, 0x50,
	0x73, 0x50,
	0x74, 0x50,
	0x75, 0x50,
	0x76, 0x4c,
	0x77, 0x4c,
	0x78, 0x45,
	0x79, 0x3c,
	0x7A, 0x2c,
	0x7B, 0x24,
	0x7C, 0x05,
	0x7D, 0x0b,
	0x7E, 0x16,
	0x7F, 0x2c,
	0x80, 0x37,
	0x81, 0x41,
	0x82, 0x4b,
	0x83, 0x55,
	0x84, 0x5f,
	0x85, 0x69,
	0x86, 0x7C,
	0x87, 0x8f,
	0x88, 0xB1,
	0x89, 0xcf,
	0x8A, 0xE5,
	0x13, 0xad,
	OV9640_REGEND, 0x00	// End of list delimiter.
};
const static u8 gYUV_CIF[] = {
	0x15, 0x20,
	0x12, 0x20,
	0x04, 0x00,
	0x11, 0x87,
	//0x01  , 0x80,
	//0x02  , 0x80,
	0x0C, 0x05,
	0x0D, 0xC0,
	0x0E, 0x01,
	0x0F, 0x4F,
	0x13, 0x80,
	0x14, 0x4A,
	0x1B, 0x01,
	0x24, 0x78,
	0x25, 0x70,
	0x27, 0x90,
	0x33, 0x08,
	0x36, 0xC9,
	0x3A, 0x08,
	0x3B, 0x00,
	0x3C, 0x46,
	0x3D, 0x90,
	0x3E, 0x02,
	0x3F, 0xF2,
	0x41, 0x02,
	0x42, 0xC8,
	0x43, 0xF0,
	0x44, 0x10,
	0x45, 0x5C,
	0x46, 0x70,
	0x47, 0x46,
	0x48, 0x4D,
	0x4F, 0x3C,
	0x50, 0x32,
	0x51, 0x09,
	0x52, 0x13,
	0x53, 0x39,
	0x54, 0x4C,
	0x59, 0x1F,
	0x5A, 0x55,
	0x5B, 0x43,
	0x5C, 0x7C,
	0x5D, 0x54,
	0x5E, 0x0A,
	0x5F, 0x01,
	0x60, 0x94,
	0x61, 0xCE,
	0x6B, 0x3A,
	0x6C, 0x40,
	0x6D, 0x30,
	0x6E, 0x4B,
	0x6F, 0x60,
	0x70, 0x70,
	0x71, 0x70,
	0x72, 0x70,
	0x73, 0x70,
	0x74, 0x60,
	0x75, 0x60,
	0x76, 0x50,
	0x77, 0x48,
	0x78, 0x3A,
	0x79, 0x2E,
	0x7A, 0x28,
	0x7B, 0x22,
	0x7C, 0x04,
	0x7D, 0x07,
	0x7E, 0x10,
	0x7F, 0x28,
	0x80, 0x36,
	0x81, 0x44,
	0x82, 0x52,
	0x83, 0x60,
	0x84, 0x6C,
	0x85, 0x78,
	0x86, 0x8C,
	0x87, 0x9E,
	0x88, 0xBB,
	0x89, 0xD2,
	0x8A, 0xE6,
	0x13, 0xaF,
	OV9640_REGEND, 0x00	// End of list delimiter.
};

#if 0
#endif
const static u8 gYUV_QCIF[] = {
	0x15, 0x20,
	0x12, 0x08,
	0x04, 0x00,
	0x11, 0x87,
	//0x01  , 0x80,
	//0x02  , 0x80,
	0x0C, 0x05,
	0x0D, 0xC0,
	0x0E, 0x01,
	0x0F, 0x4F,
	0x13, 0x80,
	0x14, 0x4A,
	0x1B, 0x01,
	0x24, 0x78,
	0x25, 0x70,
	0x27, 0x90,
	0x33, 0x08,
	0x36, 0xC9,
	0x3A, 0x08,
	0x3B, 0x00,
	0x3C, 0x46,
	0x3D, 0x90,
	0x3E, 0x02,
	0x3F, 0xF2,
	0x41, 0x02,
	0x42, 0xC8,
	0x43, 0xF0,
	0x44, 0x10,
	0x45, 0x5C,
	0x46, 0x70,
	0x47, 0x46,
	0x48, 0x4D,
	0x4F, 0x3C,
	0x50, 0x32,
	0x51, 0x09,
	0x52, 0x13,
	0x53, 0x39,
	0x54, 0x4C,
	0x59, 0x1F,
	0x5A, 0x55,
	0x5B, 0x43,
	0x5C, 0x7C,
	0x5D, 0x54,
	0x5E, 0x0A,
	0x5F, 0x01,
	0x60, 0x94,
	0x61, 0xCE,
	0x6B, 0x3A,
	0x6C, 0x40,
	0x6D, 0x30,
	0x6E, 0x4B,
	0x6F, 0x60,
	0x70, 0x70,
	0x71, 0x70,
	0x72, 0x70,
	0x73, 0x70,
	0x74, 0x60,
	0x75, 0x60,
	0x76, 0x50,
	0x77, 0x48,
	0x78, 0x3A,
	0x79, 0x2E,
	0x7A, 0x28,
	0x7B, 0x22,
	0x7C, 0x04,
	0x7D, 0x07,
	0x7E, 0x10,
	0x7F, 0x28,
	0x80, 0x36,
	0x81, 0x44,
	0x82, 0x52,
	0x83, 0x60,
	0x84, 0x6C,
	0x85, 0x78,
	0x86, 0x8C,
	0x87, 0x9E,
	0x88, 0xBB,
	0x89, 0xD2,
	0x8A, 0xE6,
	0x13, 0xaF,
	OV9640_REGEND, 0x00	// End of list delimiter.
};
const static u8 gYUV_QCIF1[] = {
	OV9640_COM10, 0x20,
	OV9640_COM7, 0x08,
	OV9640_COM1, 0x00,
	OV9640_CLKRC, 0x01,

	// From OmniVision
	OV9640_COM8, 0x8D,
	OV9640_BLUE, 0x80,
	OV9640_RED, 0x80,
	OV9640_COM3, 0x5,	//0x04
	OV9640_COM4, 0xC0,
	OV9640_COM6, 0x4E,	//0x4F
	OV9640_COM9, 0x4A,	//0x0A
	OV9640_PSHFT, 0x1,
	OV9640_AEW, 0x78,
	OV9640_AEB, 0x70,
	OV9640_BBIAS, 0x90,
	OV9640_CHLF, 0x8,
	OV9640_VIDO, 0xC9,
	OV9640_TSLB, 0x8,
	OV9640_COM11, 0x8,
	OV9640_COM12, 0x46,
	OV9640_COM13, 0x90,
	OV9640_COM14, 0x2,
	OV9640_EDGE, 0xA4,
	OV9640_COM16, 0x2,
	OV9640_COM17, 0xC8,
	OV9640_AWBTH1, 0xF0,
	OV9640_AWBTH2, 0x10,
	OV9640_AWBTH3, 0x5C,
	OV9640_AWBTH4, 0x70,
	OV9640_AWBTH5, 0x46,
	OV9640_AWBTH6, 0x4D,
	OV9640_MTX1, 0x63,
	OV9640_MTX2, 0x4E,
	OV9640_MTX3, 0x15,
	OV9640_MTX4, 0x1D,
	OV9640_MTX5, 0xBE,
	OV9640_MTX6, 0xDC,
	OV9640_MTXS, 0xF,
	OV9640_AWBC1, 0x1F,
	OV9640_AWBC2, 0x55,
	OV9640_AWBC3, 0x43,
	OV9640_AWBC4, 0x7C,
	OV9640_AWBC5, 0x54,
	OV9640_AWBC6, 0xA,
	OV9640_AWBC7, 0x5,
	OV9640_AWBC8, 0x14,
	OV9640_AWBC9, 0xCE,
	OV9640_DBLV, 0x3A,
	OV9640_GST0, 0x2,
	OV9640_GSP0, 0x40,
	OV9640_GSP1, 0x30,
	OV9640_GSP2, 0x4B,
	OV9640_GSP3, 0x60,
	OV9640_GSP4, 0x70,
	OV9640_GSP5, 0x70,
	OV9640_GSP6, 0x70,
	OV9640_GSP7, 0x70,
	OV9640_GSP8, 0x60,
	OV9640_GSP9, 0x60,
	OV9640_GSP10, 0x50,
	OV9640_GSP11, 0x48,
	OV9640_GSP12, 0x3A,
	OV9640_GSP13, 0x2E,
	OV9640_GSP14, 0x28,
	OV9640_GSP15, 0x22,
	OV9640_GST0, 0x4,
	OV9640_GST1, 0x7,
	OV9640_GST2, 0x10,
	OV9640_GST3, 0x28,
	OV9640_GST4, 0x36,
	OV9640_GST5, 0x44,
	OV9640_GST6, 0x52,
	OV9640_GST7, 0x60,
	OV9640_GST8, 0x6C,
	OV9640_GST9, 0x78,
	OV9640_GST10, 0x8C,
	OV9640_GST11, 0x9E,
	OV9640_GST12, 0xBB,
	OV9640_GST13, 0xD2,
	OV9640_GST14, 0xE6,
	OV9640_COM8, 0x8F,
	OV9640_REGEND, 0x00	// End of list delimiter.
};


const static u8 gYUV_VGA1[] = {
	OV9640_COM10, 0x20,
	OV9640_COM7, 0x40,
	OV9640_COM1, 0x00,
	OV9640_CLKRC, 0x00,

	// From OmniVision
	OV9640_COM8, 0x8D,
	OV9640_BLUE, 0x80,
	OV9640_RED, 0x80,
	OV9640_COM3, 0x5,	//0x04
	OV9640_COM4, 0x40,
	OV9640_COM6, 0x4E,	//0x4F
	OV9640_COM9, 0x4A,	//0x0A
	OV9640_PSHFT, 0x1,
	OV9640_AEW, 0x78,
	OV9640_AEB, 0x70,
	OV9640_BBIAS, 0x90,
	OV9640_CHLF, 0x8,
	OV9640_VIDO, 0xC9,
	OV9640_TSLB, 0x8,
	OV9640_COM11, 0x8,
	OV9640_COM12, 0x46,
	OV9640_COM13, 0x90,
	OV9640_COM14, 0x2,
	OV9640_EDGE, 0xA4,
	OV9640_COM16, 0x2,
	OV9640_COM17, 0xC8,
	OV9640_AWBTH1, 0xF0,
	OV9640_AWBTH2, 0x10,
	OV9640_AWBTH3, 0x5C,
	OV9640_AWBTH4, 0x70,
	OV9640_AWBTH5, 0x46,
	OV9640_AWBTH6, 0x4D,
	OV9640_MTX1, 0x63,
	OV9640_MTX2, 0x4E,
	OV9640_MTX3, 0x15,
	OV9640_MTX4, 0x1D,
	OV9640_MTX5, 0xBE,
	OV9640_MTX6, 0xDC,
	OV9640_MTXS, 0xF,
	OV9640_AWBC1, 0x1F,
	OV9640_AWBC2, 0x55,
	OV9640_AWBC3, 0x43,
	OV9640_AWBC4, 0x7C,
	OV9640_AWBC5, 0x54,
	OV9640_AWBC6, 0xA,
	OV9640_AWBC7, 0x5,
	OV9640_AWBC8, 0x14,
	OV9640_AWBC9, 0xCE,
	OV9640_DBLV, 0x3A,
	OV9640_GST0, 0x2,
	OV9640_GSP0, 0x40,
	OV9640_GSP1, 0x30,
	OV9640_GSP2, 0x4B,
	OV9640_GSP3, 0x60,
	OV9640_GSP4, 0x70,
	OV9640_GSP5, 0x70,
	OV9640_GSP6, 0x70,
	OV9640_GSP7, 0x70,
	OV9640_GSP8, 0x60,
	OV9640_GSP9, 0x60,
	OV9640_GSP10, 0x50,
	OV9640_GSP11, 0x48,
	OV9640_GSP12, 0x3A,
	OV9640_GSP13, 0x2E,
	OV9640_GSP14, 0x28,
	OV9640_GSP15, 0x22,
	OV9640_GST0, 0x4,
	OV9640_GST1, 0x7,
	OV9640_GST2, 0x10,
	OV9640_GST3, 0x28,
	OV9640_GST4, 0x36,
	OV9640_GST5, 0x44,
	OV9640_GST6, 0x52,
	OV9640_GST7, 0x60,
	OV9640_GST8, 0x6C,
	OV9640_GST9, 0x78,
	OV9640_GST10, 0x8C,
	OV9640_GST11, 0x9E,
	OV9640_GST12, 0xBB,
	OV9640_GST13, 0xD2,
	OV9640_GST14, 0xE6,
	OV9640_COM8, 0x8F,
	OV9640_REGEND, 0x00	// End of list delimiter.        
};

const static u8 gYUV_CIF1[] = {
	OV9640_COM10, 0x20,
	OV9640_COM7, 0x20,
	OV9640_COM1, 0x00,
	OV9640_CLKRC, 0x00,

	// From OmniVision
	OV9640_COM8, 0x8D,
	OV9640_BLUE, 0x80,
	OV9640_RED, 0x80,
	OV9640_COM3, 0x5,	//0x04
	OV9640_COM4, 0x40,
	OV9640_COM6, 0x4E,	//0x4F
	OV9640_COM9, 0x4A,	//0x0A
	OV9640_PSHFT, 0x1,
	OV9640_AEW, 0x78,
	OV9640_AEB, 0x70,
	OV9640_BBIAS, 0x90,
	OV9640_CHLF, 0x8,
	OV9640_VIDO, 0xC9,
	OV9640_TSLB, 0x8,
	OV9640_COM11, 0x8,
	OV9640_COM12, 0x46,
	OV9640_COM13, 0x90,
	OV9640_COM14, 0x2,
	OV9640_EDGE, 0xA4,
	OV9640_COM16, 0x2,
	OV9640_COM17, 0xC8,
	OV9640_AWBTH1, 0xF0,
	OV9640_AWBTH2, 0x10,
	OV9640_AWBTH3, 0x5C,
	OV9640_AWBTH4, 0x70,
	OV9640_AWBTH5, 0x46,
	OV9640_AWBTH6, 0x4D,
	OV9640_MTX1, 0x63,
	OV9640_MTX2, 0x4E,
	OV9640_MTX3, 0x15,
	OV9640_MTX4, 0x1D,
	OV9640_MTX5, 0xBE,
	OV9640_MTX6, 0xDC,
	OV9640_MTXS, 0xF,
	OV9640_AWBC1, 0x1F,
	OV9640_AWBC2, 0x55,
	OV9640_AWBC3, 0x43,
	OV9640_AWBC4, 0x7C,
	OV9640_AWBC5, 0x54,
	OV9640_AWBC6, 0xA,
	OV9640_AWBC7, 0x5,
	OV9640_AWBC8, 0x14,
	OV9640_AWBC9, 0xCE,
	OV9640_DBLV, 0x3A,
	OV9640_GST0, 0x2,
	OV9640_GSP0, 0x40,
	OV9640_GSP1, 0x30,
	OV9640_GSP2, 0x4B,
	OV9640_GSP3, 0x60,
	OV9640_GSP4, 0x70,
	OV9640_GSP5, 0x70,
	OV9640_GSP6, 0x70,
	OV9640_GSP7, 0x70,
	OV9640_GSP8, 0x60,
	OV9640_GSP9, 0x60,
	OV9640_GSP10, 0x50,
	OV9640_GSP11, 0x48,
	OV9640_GSP12, 0x3A,
	OV9640_GSP13, 0x2E,
	OV9640_GSP14, 0x28,
	OV9640_GSP15, 0x22,
	OV9640_GST0, 0x4,
	OV9640_GST1, 0x7,
	OV9640_GST2, 0x10,
	OV9640_GST3, 0x28,
	OV9640_GST4, 0x36,
	OV9640_GST5, 0x44,
	OV9640_GST6, 0x52,
	OV9640_GST7, 0x60,
	OV9640_GST8, 0x6C,
	OV9640_GST9, 0x78,
	OV9640_GST10, 0x8C,
	OV9640_GST11, 0x9E,
	OV9640_GST12, 0xBB,
	OV9640_GST13, 0xD2,
	OV9640_GST14, 0xE6,
	OV9640_COM8, 0x8F,
	OV9640_REGEND, 0x00	// End of list delimiter.        
};

const static u8 gYUV_SXGA[] = {
	OV9640_COM10, 0x20,
	OV9640_COM7, 0x00,
	OV9640_COM1, 0x00,
	OV9640_CLKRC, 0x80,

	// From OmniVision
	OV9640_COM8, 0x8D,
	OV9640_BLUE, 0x80,
	OV9640_RED, 0x80,
	OV9640_COM3, 0x1,	//0x04
	OV9640_COM4, 0x40,
	OV9640_COM6, 0x4E,	//0x4F
	OV9640_COM9, 0x4A,	//0x0A
	OV9640_PSHFT, 0x1,
	OV9640_AEW, 0x78,
	OV9640_AEB, 0x70,
	OV9640_BBIAS, 0x90,
	OV9640_CHLF, 0x8,
	OV9640_VIDO, 0xC9,
	OV9640_TSLB, 0x0,	//0x8
	OV9640_COM11, 0x8,
	OV9640_COM12, 0x46,
	OV9640_COM13, 0x90,
	OV9640_COM14, 0x2,
	OV9640_EDGE, 0xA4,
	OV9640_COM16, 0x2,
	OV9640_COM17, 0xC8,
	OV9640_AWBTH1, 0xF0,
	OV9640_AWBTH2, 0x10,
	OV9640_AWBTH3, 0x5C,
	OV9640_AWBTH4, 0x70,
	OV9640_AWBTH5, 0x46,
	OV9640_AWBTH6, 0x4D,
	OV9640_MTX1, 0x63,
	OV9640_MTX2, 0x4E,
	OV9640_MTX3, 0x15,
	OV9640_MTX4, 0x1D,
	OV9640_MTX5, 0xBE,
	OV9640_MTX6, 0xDC,
	OV9640_MTXS, 0xF,
	OV9640_AWBC1, 0x1F,
	OV9640_AWBC2, 0x55,
	OV9640_AWBC3, 0x43,
	OV9640_AWBC4, 0x7C,
	OV9640_AWBC5, 0x54,
	OV9640_AWBC6, 0xA,
	OV9640_AWBC7, 0x5,
	OV9640_AWBC8, 0x14,
	OV9640_AWBC9, 0xCE,
	OV9640_DBLV, 0x3A,
	OV9640_GST0, 0x2,
	OV9640_GSP0, 0x40,
	OV9640_GSP1, 0x30,
	OV9640_GSP2, 0x4B,
	OV9640_GSP3, 0x60,
	OV9640_GSP4, 0x70,
	OV9640_GSP5, 0x70,
	OV9640_GSP6, 0x70,
	OV9640_GSP7, 0x70,
	OV9640_GSP8, 0x60,
	OV9640_GSP9, 0x60,
	OV9640_GSP10, 0x50,
	OV9640_GSP11, 0x48,
	OV9640_GSP12, 0x3A,
	OV9640_GSP13, 0x2E,
	OV9640_GSP14, 0x28,
	OV9640_GSP15, 0x22,
	OV9640_GST0, 0x4,
	OV9640_GST1, 0x7,
	OV9640_GST2, 0x10,
	OV9640_GST3, 0x28,
	OV9640_GST4, 0x36,
	OV9640_GST5, 0x44,
	OV9640_GST6, 0x52,
	OV9640_GST7, 0x60,
	OV9640_GST8, 0x6C,
	OV9640_GST9, 0x78,
	OV9640_GST10, 0x8C,
	OV9640_GST11, 0x9E,
	OV9640_GST12, 0xBB,
	OV9640_GST13, 0xD2,
	OV9640_GST14, 0xE6,
	OV9640_COM8, 0x8F,
	OV9640_REGEND, 0x00	// End of list delimiter.        
};


const static u8 gRGB_QQVGA[] = {
	OV9640_COM7, 0x80,
	OV9640_REGEND, 0x00	// End of list delimiter.
};

const static u8 gRGB_QVGA[] = {
	OV9640_COM7, 0x80,
	OV9640_REGEND, 0x00	// End of list delimiter.
};

const static u8 gRGB_QCIF[] = {
	OV9640_COM7, 0x80,
	OV9640_REGEND, 0x00	// End of list delimiter.
};


const static u8 gRGB_VGA[] = {
	OV9640_COM7, 0x80,
	OV9640_REGEND, 0x00	// End of list delimiter.
};

const static u8 gRGB_CIF[] = {
	OV9640_COM7, 0x80,
	OV9640_REGEND, 0x00	// End of list delimiter.
};

const static u8 gRGB_SXGA[] = {
	OV9640_COM7, 0x80,
	OV9640_REGEND, 0x00	// End of list delimiter.
};
const static u8 gYUV_QVGA_v3[] = {
	0x11, 0x83,
	0x12, 0x10,
//      0x13, 0xA8,
	0x01, 0x80,
	0x02, 0x80,
	0x04, 0x00,
	0x0C, 0x05,
	0x0D, 0xC0,
	0x0E, 0x81,
	0x0F, 0x4F,
	0x14, 0x8A,
	0x16, 0x02,
	0x1B, 0x01,
	0x24, 0x70,
	0x25, 0x68,
	0x26, 0xD1,
	0x27, 0x88,
	0x2A, 0x00,
	0x2B, 0x00,
	0x2c, 0x88,
	0x33, 0x02,
	0x37, 0x05,
	0x38, 0x13,
	0x39, 0xF0,
	0x3A, 0x0C,
	0x3B, 0x01,
	0x3C, 0x46,
	0x3D, 0x90,
	0x3E, 0x02,
	0x3F, 0xa4,
	0x41, 0x02,
	0x42, 0xC8,

	/*
	   0x43, 0xF0,
	   0x44, 0x10,
	   0x45, 0x5C,
	   0x46, 0x70,
	   0x47, 0x46,
	   0x48, 0x4d,

	   0x43, 0xF0,
	   0x44, 0x10,
	   0x45, 0x6C,
	   0x46, 0x6c,
	   0x47, 0x44,
	   0x48, 0x44,
	 */

	0x43, 0xF0,
	0x44, 0x10,
	0x45, 0x20,
	0x46, 0x20,
	0x47, 0x20,
	0x48, 0x20,

	0x49, 0x03,
	0x4f, 0x3c,
	0x50, 0x32,
	0x51, 0x09,
	0x52, 0x13,
	0x53, 0x39,
	0x54, 0x4c,

	/*
	   0x59, 0x1f,
	   0x5A, 0x55,
	   0x5B, 0x43,
	   0x5C, 0x7c,
	   0x5D, 0x54,
	   0x5E, 0x0a,
	   0x5F, 0x01,
	   0x60, 0x94,
	   0x61, 0xCE,

	   0x59, 0x49,
	   0x5A, 0x94,
	   0x5B, 0x46,
	   0x5C, 0x84,
	   0x5D, 0x5c,
	   0x5E, 0x08,
	   0x5F, 0x00,
	   0x60, 0x14,
	   0x61, 0xCE,
	 */
	0x59, 0x27,
	0x5A, 0x72,
	0x5B, 0x56,
	0x5C, 0x7a,
	0x5D, 0x5d,
	0x5E, 0x17,
	0x5F, 0x00,
	0x60, 0x14,
	0x61, 0xCE,
	0x62, 0x78,
	0x63, 0x00,
	0x64, 0x04,
	0x65, 0x50,
	0x66, 0x01,
	0x69, 0x00,
	0x6A, 0x3d,		//for QVGA 3e-->3d, 
	0x6B, 0x3F,
	/*
	   0x6C, 0x50,
	   0x6D, 0x60,
	   0x6E, 0x58,
	   0x6F, 0x58,
	   0x70, 0x58,
	   0x71, 0x50,
	   0x72, 0x50,
	   0x73, 0x50,
	   0x74, 0x50,
	   0x75, 0x50,
	   0x76, 0x4c,
	   0x77, 0x4c,
	   0x78, 0x45,
	   0x79, 0x3c,
	   0x7A, 0x2c,
	   0x7B, 0x24,
	   0x7C, 0x05,
	   0x7D, 0x0b,
	   0x7E, 0x16,
	   0x7F, 0x2c,
	   0x80, 0x37,
	   0x81, 0x41,
	   0x82, 0x4b,
	   0x83, 0x55,
	   0x84, 0x5f,
	   0x85, 0x69,
	   0x86, 0x7C,
	   0x87, 0x8f,
	   0x88, 0xB1,
	   0x89, 0xcf,
	   0x8A, 0xE5,
	 */
	0x6C, 0x40,
	0x6D, 0x30,
	0x6E, 0x4B,
	0x6F, 0x60,
	0x70, 0x70,
	0x71, 0x70,
	0x72, 0x70,
	0x73, 0x70,
	0x74, 0x60,
	0x75, 0x60,
	0x76, 0x50,
	0x77, 0x48,
	0x78, 0x3A,
	0x79, 0x2E,
	0x7A, 0x28,
	0x7B, 0x22,
	0x7C, 0x04,
	0x7D, 0x07,
	0x7E, 0x10,
	0x7F, 0x28,
	0x80, 0x36,
	0x81, 0x44,
	0x82, 0x52,
	0x83, 0x60,
	0x84, 0x6C,
	0x85, 0x78,
	0x86, 0x8C,
	0x87, 0x9E,
	0x88, 0xBB,
	0x89, 0xD2,
	0x8A, 0xE6,
	/*
	 */
	0x13, 0xAF,
	OV9640_REGEND, 0x00	// End of list delimiter.
};

const static u8 gSensorSlaveAddr = 0x30;

int i2c_init(void)
{
	return i2c_ov9640_init();
}

int i2c_deinit(void)
{
	return i2c_ov9640_cleanup();
}

/***********************************************************************
*  Private/helper api
***********************************************************************/
static int prv_get_reg_value(u8 * regP, u8 regAddr, u8 * regValueP)
{
	unsigned int index = 0;
	u8 curReg = 0;
	while (curReg < OV9640_REGEND) {
		curReg = regP[index << 1];
		if (curReg == regAddr) {
			*regValueP = regP[(index << 1) + 1];
			return 0;
		}
		index++;
	}

	return -1;

}

static int prv_set_reg_value(u8 * regP, u8 regAddr, u8 regValue)
{
	unsigned int index = 0;
	u8 curReg = 0;

	while (curReg < OV9640_REGEND) {
		curReg = regP[index << 1];
		if (curReg == regAddr) {
			regP[(index << 1) + 1] = regValue;
			return 0;
		}
		index++;
	}

	return -1;

}

/***********************************************************************
*  Sensor read/write 
***********************************************************************/
static int prv_read_sensor_reg(const u8 subAddress, u8 * bufP)
{
	return ov9640_read(subAddress, bufP);
}

static int prv_write_sensor_reg(const u8 subAddress, u8 * bufP)
{
	return ov9640_write(subAddress, *bufP);
}

static int prv_RMW_sensor_reg(const u8 subAddress, u8 * bufP, u8 andMask, u8 orMask)
{
	int status;
	status = prv_read_sensor_reg(subAddress, bufP);
	if (!status) {
		*bufP &= andMask;
		*bufP |= orMask;
		status = prv_write_sensor_reg(subAddress, bufP);
	}
	return status;
}

int ov9640_read_sensor_reg(const u8 subAddress, u8 * bufP)
{
	return prv_read_sensor_reg(subAddress, bufP);
}

int ov9640_write_sensor_reg(const u8 subAddress, u8 * bufP)
{
	return prv_write_sensor_reg(subAddress, bufP);
}

int ov9640_set_regs(u8 * regP)
{
	u32 curReg = 0;
	int status = 0;

	// The list is a register number followed by the value.
	while (regP[curReg << 1] < OV9640_REGEND) {
		status = prv_write_sensor_reg(regP[curReg << 1], &regP[(curReg << 1) + 1]);
		if (curReg == 0)
			ov9640_wait(1);

		curReg++;
	}
	return status;
}

int ov9640_read_all_regs(u8 * bufP, u32 numRegs)
{
	u32 curReg;

	for (curReg = 0; curReg < numRegs; curReg++, bufP++)
		prv_read_sensor_reg(curReg, bufP);

	return 0;
}

/***********************************************************************
*  Power & Reset
***********************************************************************/
void ov9640_power_down(int powerDown)
{
	// OV9640 PWRDWN, 0 = NORMAL, 1=POWER DOWN
	//GPDR1 |= GPIO_bit(GPIO_CAM_EN);
	//OV9640 reset CIF_RST, 0 = NORMAL, 1=RESET
	//GPDR0 |= GPIO_bit(GPIO_CAM_RST);
	if (powerDown == 1) {
		mdelay(200);
		ov9640_soft_reset();
		ov9640_write(0x39, 0xf4);
		ov9640_write(0x1e, 0x80);
		ov9640_write(0x6b, 0x3f);
		ov9640_write(0x36, 0x49);
		ov9640_write(0x12, 0x05);
		mdelay(800);
		GPSR1 = GPIO_bit(GPIO_CAM_EN);
	}
	else {
		ov9640_write(0x39, 0xf0);
		ov9640_write(0x1e, 0x00);
		ov9640_write(0x6b, 0x3f);
		ov9640_write(0x36, 0x49);
		ov9640_write(0x12, 0x10);
		GPCR1 = GPIO_bit(GPIO_CAM_EN);
		//GPSR0 = GPIO_bit(GPIO_CAM_RST);
		mdelay(20);
		//GPCR0 = GPIO_bit(GPIO_CAM_RST);
	}
	mdelay(100);
}

void ov9640_soft_reset(void)
{
	u8 regValue;
	regValue = 0x80;
	prv_write_sensor_reg(OV9640_COM7, &regValue);
	mdelay(10);
	return;
}

void ov9640_reset(void)
{
	// do nothing on Zoar
	return 0;
}

void ov9640_wait(int ms)
{
	mdelay(ms);
}

int ov9640_output_stoped()
{
	ov9640 *pov;
	pov = &g_ov;
	return pov->stoped;
}

void ov9640_set_start()
{
	ov9640 *pov;
	pov = &g_ov;

	pov->stoped = 0;
}

void ov9640_set_stop(p_camera_context_t cam_ctx)
{
	ov9640 *pov;
	pov = &g_ov;

	pov->stoped = 1;
}

/***********************************************************************
*  Settings
***********************************************************************/
int ov9640_version_revision(u8 * pCmRevision, u8 * pSensorRevision)
{
	prv_read_sensor_reg(OV9640_PID, pCmRevision);
	prv_read_sensor_reg(OV9640_VER, pSensorRevision);
	return 0;
}

void ov9640_set_HSYNC(void)
{
	u8 val;

	// Makes HREF become HSYNC
	prv_read_sensor_reg(OV9640_COM10, &val);
	val |= 0x40;
	prv_write_sensor_reg(OV9640_COM10, &val);
}

void ov9640_auto_function_on(void)
{
	u8 val;
	DPRINTK("in function %s\n", __FUNCTION__);
	prv_read_sensor_reg(OV9640_COM8, &val);
	val |= 0x07;
	prv_write_sensor_reg(OV9640_COM8, &val);
}

void ov9640_auto_function_off(void)
{
	u8 val;
	DPRINTK("in function %s\n", __FUNCTION__);
	prv_read_sensor_reg(OV9640_COM8, &val);
	val &= ~0x07;
	prv_write_sensor_reg(OV9640_COM8, &val);
}


/***********************************************************************
*  Viewfinder, still 
***********************************************************************/
int ov9640_viewfinder_on(void)
{
	u8 com3;

	prv_read_sensor_reg(OV9640_COM3, &com3);
	com3 &= ~0x01;
	prv_write_sensor_reg(OV9640_COM3, &com3);

	return OV_ERR_NONE;
}


int ov9640_viewfinder_off(void)
{
	u8 com3;

	prv_read_sensor_reg(OV9640_COM3, &com3);
	com3 |= 0x01;
	prv_write_sensor_reg(OV9640_COM3, &com3);

	mdelay(200);
	return OV_ERR_NONE;
}


int ov9640_halt_video_output(void)
{
	u8 com3;

	// Set the camera to only output 1 frame.
	prv_read_sensor_reg(OV9640_COM3, &com3);
	com3 |= 1;
	prv_write_sensor_reg(OV9640_COM3, &com3);

	return OV_ERR_NONE;
}

int ov9640_resume_to_full_output_mode(void)
{
	u8 mode;

	// Output still frames continuously
	// Turn off single capture mode COM3.
	prv_RMW_sensor_reg(OV9640_COM3, (&mode), ((u8) ~ 1), 0);
	return OV_ERR_NONE;
}

int ov9640_get_single_image(void)
{
	u8 mode;

	prv_RMW_sensor_reg(OV9640_COM3, &mode, (u8) ~ 1, 1);
	return OV_ERR_NONE;
}
static u8 *ov9640_get_regs_list(u32 captureSizeFormat, u32 colorFormat)
{
	char *formatNameP;
	u8 *defaultDataP = NULL;
	ov9640 *pov;

	pov = &g_ov;


	// Get the default setting.
	if (colorFormat == OV_FORMAT_YUV_422) {
		switch (captureSizeFormat) {
		case OV_SIZE_QQVGA:
			defaultDataP = gYUV_QQVGA;
			formatNameP = "QQVGA.422";
			break;
		case OV_SIZE_QQCIF:
			defaultDataP = gYUV_QQCIF;
			formatNameP = "QQCIF.422";
			break;
		case OV_SIZE_QVGA:
			defaultDataP = gYUV_QVGA;
			formatNameP = "QVGA.422";
			//version 3 support:
			if (pov->version == ((PID_OV_v3 << 8) | (PID_9640_v3))) {
				DPRINTK("ver 3 sensor!\n");
				defaultDataP = gYUV_QVGA_v3;
			}
			break;
		case OV_SIZE_QCIF:
			defaultDataP = gYUV_QCIF;
			formatNameP = "QCIF.422";
			break;
		case OV_SIZE_VGA:
			defaultDataP = gYUV_VGA;
			formatNameP = "VGA.422";
			break;
		case OV_SIZE_CIF:
			defaultDataP = gYUV_CIF;
			formatNameP = "CIF.422";
			break;
		case OV_SIZE_SXGA:
			defaultDataP = gYUV_SXGA;
			formatNameP = "SXGA.422";
			break;
		default:
			return NULL;
		}
	}

	if (colorFormat == OV_FORMAT_RGB_565) {
		switch (captureSizeFormat) {
		case OV_SIZE_QQVGA:
			defaultDataP = gRGB_QQVGA;
			formatNameP = "QQVGA.RGB";
			break;
		case OV_SIZE_QCIF:
			defaultDataP = gRGB_QCIF;
			formatNameP = "QCIF.RGB";
			break;

		case OV_SIZE_QVGA:
			defaultDataP = gRGB_QVGA;
			formatNameP = "QVGA.RGB";
			break;
		case OV_SIZE_VGA:
			defaultDataP = gRGB_VGA;
			formatNameP = "VGA.RGB";
			break;
		case OV_SIZE_CIF:
			defaultDataP = gRGB_CIF;
			formatNameP = "CIF.RGB";
			break;
		case OV_SIZE_SXGA:
			defaultDataP = gRGB_SXGA;
			formatNameP = "SXGA.RGB";
			break;
		default:
			return NULL;
		}
	}
	return defaultDataP;
}

/***********************************************************************
*  Format 
***********************************************************************/
int ov9640_set_format(u32 captureSizeFormat, u32 colorFormat)
{
	int status = OV_ERR_PARAMETER;
	u8 *regsP, regValue;
	unsigned int index = 0;
	u8 curReg = 0;
	static u32 prev_colorFormat = OV_FORMAT_NONE, prev_size = OV_SIZE_NONE;

	if (prev_colorFormat == colorFormat && captureSizeFormat == prev_size)
		return 0;
	if ((captureSizeFormat == OV_SIZE_NONE) && (colorFormat == OV_FORMAT_NONE))
		goto no_set;

	if ((prev_colorFormat == OV_FORMAT_NONE) && (prev_size == OV_SIZE_NONE)) {
		regsP = (u8 *) ov9640_get_regs_list(captureSizeFormat, colorFormat);
		// Get the pointer to the basic setting.  The pointer must be freed after exiting.

		if (regsP == NULL)
			return OV_ERR_PARAMETER;
		ov9640_soft_reset();
		// Blast the entire parameter tree into the part.
		status = ov9640_set_regs(regsP);
	}
	else {
		status = ov9640_switch_format(captureSizeFormat, colorFormat);
	}
      no_set:
	prev_colorFormat = colorFormat;
	prev_size = captureSizeFormat;
	return status;
}

void ov9640_save_gains()
{
	u8 gain, aech, aecl, laec, blue, red;
	ov9640 *pov;
	u8 regValue;
	u32 current_size;

	pov = &g_ov;

	// Get current size
	prv_read_sensor_reg(OV9640_COM7, &regValue);
	switch (regValue) {
	case 0x00:
		current_size = OV_SIZE_SXGA;
		break;
	case 0x08:
		prv_read_sensor_reg(OV9640_COM1, &regValue);
		if (regValue & 0x20)
			current_size = OV_SIZE_QQCIF;
		else
			current_size = OV_SIZE_QCIF;
		break;
	case 0x10:
		prv_read_sensor_reg(OV9640_COM1, &regValue);
		if (regValue & 0x20)
			current_size = OV_SIZE_QQVGA;
		else
			current_size = OV_SIZE_QVGA;
		break;
	case 0x20:
		current_size = OV_SIZE_CIF;
		break;
	case 0x40:
		current_size = OV_SIZE_VGA;
		break;
	default:
		current_size = OV_SIZE_SXGA;
		break;
	}
	pov->pre_size = current_size;

	// Get the awb, gain, exposure values
	prv_read_sensor_reg(OV9640_BLUE, &blue);
	prv_read_sensor_reg(OV9640_RED, &red);
	prv_read_sensor_reg(OV9640_GAIN, &gain);
	gain &= 0x3F;
	prv_read_sensor_reg(OV9640_AECH, &aech);
	prv_read_sensor_reg(OV9640_COM1, &aecl);
	aecl &= 0x3;
	prv_read_sensor_reg(OV9640_LAEC, &laec);

	pov->gain = gain;
	pov->blue_gain = blue;
	pov->red_gain = red;
	pov->exp_value = (aech << 2) | aecl;
	pov->exp_time = laec;

}

void ov9640_adjust_gains(u32 prev_size, u32 cur_size)
{
	u8 expMultiple = 1;
	u8 gain;
	ov9640 *pov;

	pov = &g_ov;
	if (prev_size == OV_SIZE_QVGA) {
		if (cur_size == OV_SIZE_VGA) {
			if (pov->gain & 0x20) {
				/* hight gain capture 7.5 fps */
				pov->pclock = 0x83;
				pov->gain = pov->gain & ~0x20;
				pov->adjusted_exp_value = pov->exp_value * 2;
			}
			else {
				/* capture 15 fps */
				pov->pclock = 0x81;
				pov->adjusted_exp_value = pov->exp_value * 2;
			}
			if (pov->night_mode == 1) {
				//night mode 
				//pov->pclock = 0x87;
			}
			else if (pov->night_mode == 2) {	//action mode
				pov->pclock = 0x80;
			}
		}
		else if (cur_size == OV_SIZE_SXGA) {
			if (pov->gain & 0x20) {
				/* 8x gain capture 7.5 fps to 4x gain */
				pov->pclock = 0x80;
				pov->gain = pov->gain & ~0x20;
				pov->adjusted_exp_value = pov->exp_value * 4;
			}
			else if (pov->gain & 0x10) {
				/* 4x gain capture 7.5 fps to 2x gain */
				pov->pclock = 0x80;
				pov->gain = pov->gain & ~0x10;
				pov->adjusted_exp_value = pov->exp_value * 4;
			}
			else {
				pov->pclock = 0x80;
				pov->adjusted_exp_value = pov->exp_value * 2;
			}
			if (pov->night_mode == 1) {
				//night mode 
				//pov->pclock = 0x81;
			}
			else if (pov->night_mode == 2) {	//action mode
				pov->pclock = 0x80;
			}
		}
		else if (cur_size == OV_SIZE_QVGA) {
			if (pov->gain & 0x20) {
				/* hight gain capture 7.5 fps */
				pov->pclock = 0x87;
				pov->gain = pov->gain & ~0x20;
				pov->adjusted_exp_value = pov->exp_value;
			}
			else {
				/* capture 15 fps */
				pov->pclock = 0x83;
				pov->adjusted_exp_value = pov->exp_value;
			}
			if (pov->night_mode == 1) {
				//night mode 
				//pov->pclock = 0x89;
			}
			else if (pov->night_mode == 2) {	//action mode
				pov->pclock = 0x81;
			}
		}
		pov->red_gain=pov->red_gain*9 /10;
		pov->blue_gain=pov->blue_gain *21 /20;
	}
}

static void ov9640_upload_gains()
{
	ov9640 *pov;
	u32 expValue;
	u8 gain, aech, aecl, laec, blue, red;
	u8 regValue;

	pov = &g_ov;

	gain = pov->gain;
	blue = pov->blue_gain;
	red = pov->red_gain;
	expValue = pov->adjusted_exp_value;
	// Set awb
	prv_write_sensor_reg(OV9640_BLUE, &blue);
	prv_write_sensor_reg(OV9640_RED, &red);

	// Set gain 
	prv_write_sensor_reg(OV9640_GAIN, &gain);
	// Set exposure
	prv_read_sensor_reg(OV9640_COM1, &regValue);
	regValue = (regValue & 0xFC) | (expValue & 0x03);
	prv_write_sensor_reg(OV9640_COM1, &regValue);
	expValue >>= 2;
	if (expValue > 0xFF)
		regValue = 0xFF;
	else
		regValue = expValue;
	prv_write_sensor_reg(OV9640_AECH, &regValue);
	prv_write_sensor_reg(OV9640_CLKRC, &pov->pclock);
}

int ov9640_switch_format(u32 captureSizeFormat, u32 colorFormat)
{
	int status;
	u32 index, curReg;
	u8 *regsP;
	u8 regValue;
	ov9640 *pov;

	pov = &g_ov;

	regsP = (u8 *) ov9640_get_regs_list(captureSizeFormat, colorFormat);
	// Get the pointer to the basic setting.  The pointer must be freed after exiting.

	if (regsP == NULL)
		return OV_ERR_PARAMETER;
	// Apply the delta registers.
	index = curReg = 0;
	while ((curReg = gDelta[index]) < OV9640_REGEND) {
		status = prv_get_reg_value(regsP, curReg, &regValue);
		if (status == 0) {
			prv_write_sensor_reg(curReg, &regValue);
		}
		else
			break;
		index++;
	}
	mdelay(50);
	if (pov->night_mode == 1) {
		//night mode 
		ov9640_set_night_mode();
	}
	else if (pov->night_mode == 2) {	//action mode
		ov9640_set_action_mode();
	}
	else
		ov9640_set_auto_mode();

	return status;
}

int
ov9640_set_dma_pages(pxa_dma_desc ** pdes,
		     pxa_dma_desc ** des_physical, int num, struct page *array[], int total_size, int dsadr, int flags)
{
	int remain_size, des_transfer_size;
	int j, target_page_num = num;
	pxa_dma_desc *cur_des_virtual = *pdes;
	pxa_dma_desc *cur_des_physical = *des_physical;

	// in each iteration, generate one dma chain for one frame
	remain_size = total_size;

	for (j = 0; j < num; j++) {
		// set descriptor
		if (remain_size > PAGE_SIZE)
			des_transfer_size = PAGE_SIZE;
		else
			des_transfer_size = remain_size;
		cur_des_virtual->ddadr = (unsigned) cur_des_physical + sizeof(pxa_dma_desc);
		cur_des_virtual->dsadr = dsadr;	// FIFO0 physical address
		cur_des_virtual->dtadr = page_to_bus(array[j]);
		cur_des_virtual->dcmd = des_transfer_size | flags;

		// advance pointers
		remain_size -= des_transfer_size;
		cur_des_virtual++;
		cur_des_physical++;
		target_page_num++;
	}
	*pdes = cur_des_virtual;
	*des_physical = cur_des_physical;
}

int
ov9640_set_dma_page1(pxa_dma_desc ** pdes,
		     pxa_dma_desc ** des_physical, int num, struct page *page1, int total_size, int dsadr, int flags)
{
	int remain_size, des_transfer_size;
	int j, target_page_num = num;
	pxa_dma_desc *cur_des_virtual = *pdes;
	pxa_dma_desc *cur_des_physical = *des_physical;
	int dump_page;

	// in each iteration, generate one dma chain for one frame
	remain_size = total_size;
	dump_page = page_to_bus(page1);
	DPRINTK("dump_page=%x", dump_page);

	for (j = 0; j < num; j++) {
		// set descriptor
		if (remain_size > PAGE_SIZE)
			des_transfer_size = PAGE_SIZE;
		else
			des_transfer_size = remain_size;
		cur_des_virtual->ddadr = (unsigned) cur_des_physical + sizeof(pxa_dma_desc);
		cur_des_virtual->dsadr = dsadr;	// FIFO0 physical address
		cur_des_virtual->dtadr = dump_page;
		cur_des_virtual->dcmd = des_transfer_size | flags;

		// advance pointers
		remain_size -= des_transfer_size;
		cur_des_virtual++;
		cur_des_physical++;
		target_page_num++;
	}
	*pdes = cur_des_virtual;
	*des_physical = cur_des_physical;
}

int ov9640_update_still_dma_chain(p_camera_context_t cam_ctx)
{
	pxa_dma_desc *cur_des_virtual, *cur_des_physical, *last_des_virtual = NULL;
	int des_transfer_size, remain_size;
	unsigned int i, j;

	int target_page_num;

	DPRINTK("ov9640_update_still_dma_chain\n");
	// clear descriptor pointers
	cam_ctx->fifo0_descriptors_virtual = cam_ctx->fifo0_descriptors_physical = 0;
	cam_ctx->fifo1_descriptors_virtual = cam_ctx->fifo1_descriptors_physical = 0;
	cam_ctx->fifo2_descriptors_virtual = cam_ctx->fifo2_descriptors_physical = 0;

	// calculate how many descriptors are needed per frame
	cam_ctx->fifo0_num_descriptors = cam_ctx->pages_per_fifo0;

	cam_ctx->fifo1_num_descriptors = cam_ctx->pages_per_fifo1;

	cam_ctx->fifo2_num_descriptors = cam_ctx->pages_per_fifo2;

	// check if enough memory to generate descriptors
	DPRINTK("in %s, cam_ctx->block_number =%d\n", __FUNCTION__, cam_ctx->block_number);
	if ((cam_ctx->fifo0_num_descriptors + cam_ctx->fifo1_num_descriptors +
	     cam_ctx->fifo2_num_descriptors) * cam_ctx->block_number > cam_ctx->dma_descriptors_size)
		return -1;

	// generate fifo0 dma chains
	cam_ctx->fifo0_descriptors_virtual = (unsigned) cam_ctx->dma_descriptors_virtual;
	cam_ctx->fifo0_descriptors_physical = (unsigned) cam_ctx->dma_descriptors_physical;
	cur_des_virtual = (pxa_dma_desc *) cam_ctx->fifo0_descriptors_virtual;
	cur_des_physical = (pxa_dma_desc *) cam_ctx->fifo0_descriptors_physical;

	DPRINTK("pages_allocated=%d,fifo0_descriptors_virtual=%x\n", cam_ctx->pages_allocated, cur_des_virtual);

	for (i = 0; i < 2; i++) {
		// in each iteration, generate one dma chain for one frame
		remain_size = cam_ctx->fifo0_transfer_size;
		ov9640_set_dma_page1(&cur_des_virtual, &cur_des_physical,
				     cam_ctx->fifo0_num_descriptors,
				     cam_ctx->page_array[cam_ctx->
							 pages_allocated -
							 1], remain_size, CIBR0_PHY, DCMD_FLOWSRC | DCMD_BURST32);

	}
	DPRINTK("after ov9640_set_dma_page1=%d\n", cam_ctx->pages_allocated);
	for (i = 0; i < 1; i++) {
		// in each iteration, generate one dma chain for one frame
		remain_size = cam_ctx->fifo0_transfer_size;

		// assume the blocks are stored consecutively
		target_page_num = cam_ctx->pages_per_block * i;
		DPRINTK("target_page_num=%d\n", target_page_num);
		ov9640_set_dma_pages(&cur_des_virtual, &cur_des_physical,
				     cam_ctx->fifo0_num_descriptors,
				     &cam_ctx->page_array[target_page_num],
				     remain_size, CIBR0_PHY, DCMD_FLOWSRC | DCMD_BURST32 | DCMD_INCTRGADDR);

		// stop the dma transfer on one frame captured
		last_des_virtual = cur_des_virtual - 1;
	}
	last_des_virtual->ddadr = ((unsigned) cam_ctx->fifo0_descriptors_physical);
	last_des_virtual->ddadr |= 0x1;
	last_des_virtual->dcmd |= DCMD_ENDIRQEN;

	// generate fifo1 dma chains
	if (cam_ctx->fifo1_transfer_size) {
		// record fifo1 descriptors' start address
		cam_ctx->fifo1_descriptors_virtual = (unsigned) cur_des_virtual;
		cam_ctx->fifo1_descriptors_physical = (unsigned) cur_des_physical;

		for (i = 0; i < 2; i++) {
			// in each iteration, generate one dma chain for one frame
			remain_size = cam_ctx->fifo1_transfer_size;
			ov9640_set_dma_page1(&cur_des_virtual,
					     &cur_des_physical,
					     cam_ctx->
					     fifo1_num_descriptors,
					     cam_ctx->page_array[cam_ctx->
								 pages_allocated
								 - 2],
					     remain_size, CIBR1_PHY, DCMD_FLOWSRC | DCMD_BURST32);

			// stop the dma transfer on one frame captured
			last_des_virtual = cur_des_virtual - 1;
			//last_des_virtual->ddadr |= 0x1;
		}
		for (i = 0; i < 1; i++) {
			// in each iteration, generate one dma chain for one frame
			remain_size = cam_ctx->fifo1_transfer_size;

			target_page_num = cam_ctx->pages_per_block * i + cam_ctx->pages_per_fifo0;
			ov9640_set_dma_pages(&cur_des_virtual, &cur_des_physical,
					     cam_ctx->fifo1_num_descriptors,
					     &cam_ctx->page_array[target_page_num],
					     remain_size, CIBR1_PHY, DCMD_FLOWSRC | DCMD_BURST32 | DCMD_INCTRGADDR);

			// stop the dma transfer on one frame captured
			last_des_virtual = cur_des_virtual - 1;
		}
		last_des_virtual->ddadr = ((unsigned) cam_ctx->fifo1_descriptors_physical);
		last_des_virtual->ddadr |= 0x1;
	}
	// generate fifo2 dma chains
	if (cam_ctx->fifo2_transfer_size) {
		// record fifo1 descriptors' start address
		cam_ctx->fifo2_descriptors_virtual = (unsigned) cur_des_virtual;
		cam_ctx->fifo2_descriptors_physical = (unsigned) cur_des_physical;

		for (i = 0; i < 2; i++) {
			// in each iteration, generate one dma chain for one frame
			remain_size = cam_ctx->fifo2_transfer_size;
			ov9640_set_dma_page1(&cur_des_virtual,
					     &cur_des_physical,
					     cam_ctx->
					     fifo2_num_descriptors,
					     cam_ctx->page_array[cam_ctx->
								 pages_allocated
								 - 3],
					     remain_size, CIBR2_PHY, DCMD_FLOWSRC | DCMD_BURST32);

			// stop the dma transfer on one frame captured
			last_des_virtual = cur_des_virtual - 1;
			//last_des_virtual->ddadr |= 0x1;
		}
		DPRINTK("last target_page_num=%d\n", target_page_num + cam_ctx->fifo2_num_descriptors);
		for (i = 0; i < 1; i++) {
			// in each iteration, generate one dma chain for one frame
			remain_size = cam_ctx->fifo2_transfer_size;
			target_page_num =
			    cam_ctx->pages_per_block * i + cam_ctx->pages_per_fifo0 + cam_ctx->pages_per_fifo1;
			ov9640_set_dma_pages(&cur_des_virtual, &cur_des_physical,
					     cam_ctx->fifo2_num_descriptors,
					     &cam_ctx->page_array[target_page_num],
					     remain_size, CIBR2_PHY, DCMD_FLOWSRC | DCMD_BURST32 | DCMD_INCTRGADDR);

			// stop the dma transfer on one frame captured
			last_des_virtual = cur_des_virtual - 1;
			DPRINTK("last target_page_num=%d\n", target_page_num + cam_ctx->fifo2_num_descriptors);
		}
		last_des_virtual->ddadr = ((unsigned) cam_ctx->fifo2_descriptors_physical);
		last_des_virtual->ddadr |= 0x1;
	}
	return 0;
}

int ov9640_prepare_capture(p_camera_context_t cam_ctx, u32 captureSizeFormat, u32 colorFormat)
{
	static int first = 1;
	int status;
	//u32 expMultiple, expValue;
	ov9640 *pov;

	pov = &g_ov;

	stop_dma_transfer(cam_ctx);
	ov9640_switch_format(captureSizeFormat, colorFormat);
	ov9640_adjust_gains(pov->pre_size, captureSizeFormat);
	ov9640_upload_gains();
	if (captureSizeFormat == OV_SIZE_VGA) {
		//ov9640_update_still_dma_chain(cam_ctx);
		ov9640_stop_third_des(cam_ctx);
		/*
		 */
	}
	else if (captureSizeFormat == OV_SIZE_SXGA) {
		camera_set_int_mask(cam_ctx, 0x3fd | 0x0400);
		ov9640_update_still_dma_chain(cam_ctx);
		//ov9640_halt_video_output();
		//ci_clear_int_status(0xFFFFFFFF);
		//DPRINTK("before camera_sleep \n");
		//camera_sleep();
	}
	ci_reset_fifo();
	ci_clear_int_status(0xFFFFFFFF);
	start_dma_transfer(cam_ctx, 1);
	DPRINTK("after ov9640_prepare_capture \n");
}

int ov9640_set_special_effect(int style)
{
	int ret = 0;
	u32 index, curReg;
	u8 *regsP;
	u8 regValue;

	DPRINTK("in function %s parameter=%d\n", __FUNCTION__, style);
	curReg = 0x3a;
	ov9640_read(0x3a, &regValue);
	regValue = regValue & 0xf;
	switch (style) {
	case V4l_STYLE_NORMAL:
		DPRINTK("V4l_STYLE_NORMAL\n");
		regValue &= 0x7f;
		//ov9640_write(0x3a, 0x08);
		ov9640_write(0x3a, regValue);
		ov9640_write(0x67, 0x80);
		ov9640_write(0x68, 0x80);
		break;
	case V4l_STYLE_BLACK_WHITE:
		DPRINTK("V4l_STYLE_BLACK_WHITE\n");
		regValue |= 0x10;
		//ov9640_write(0x3a, 0x18);
		ov9640_write(0x3a, regValue);
		ov9640_write(0x67, 0x80);
		ov9640_write(0x68, 0x80);
		break;
	case V4l_STYLE_SEPIA:
		DPRINTK("V4l_STYLE_SEPIA\n");
		regValue |= 0x10;
		//ov9640_write(0x3a, 0x18);
		ov9640_write(0x3a, regValue);
		ov9640_write(0x67, 0xa0);
		ov9640_write(0x68, 0x40);
		break;
		//case V4l_STYLE_BULISH:
		DPRINTK("V4l_STYLE_BULISH\n");
		ov9640_write(0x3a, 0x18);
		ov9640_write(0x67, 0x80);
		ov9640_write(0x68, 0xc0);
		break;
	default:
		DPRINTK("case default ????????????????????\n");
		//ret=OV_ERR_PARAMETER;
	}
	return ret;

}

int ov9640_set_brightness(int bright)
{
	int ret = 0;
	const u8 BN[] = {
		//BN-3
		0x0f, 0x4f,
		0x27, 0xe8,
		0x28, 0xe0,
		0x29, 0xe0,
		0x2c, 0xe0,
		//BN-2
		0x0f, 0x4f,
		0x27, 0xc8,
		0x28, 0xc0,
		0x29, 0xc0,
		0x2c, 0xc0,
		//BN-1
		0x0f, 0x4f,
		0x27, 0xa8,
		0x28, 0xa0,
		0x29, 0xa0,
		0x2c, 0xa0,
		//BN-0
		0x0f, 0x4f,
		0x27, 0x88,
		0x28, 0x80,
		0x29, 0x80,
		0x2c, 0x88,
		//BN+1
		0x0f, 0x4f,
		0x27, 0x28,
		0x28, 0x20,
		0x29, 0x20,
		0x2c, 0x20,
		//BN+2
		0x0f, 0x4f,
		0x27, 0x48,
		0x28, 0x40,
		0x29, 0x40,
		0x2c, 0x40,
		//BN+3
		0x0f, 0x4f,
		0x27, 0x68,
		0x28, 0x60,
		0x29, 0x60,
		0x2c, 0x60,
	};
	u8 *regs;
	int n = 5;
	int i;

	DPRINTK("in function %s bright =%d \n", __FUNCTION__, bright);
	if (bright < -3)
		bright = -3;
	if (bright > 3)
		bright = 3;
	//bright = -4 .. 4
	regs = &BN[(bright + 3) * n * 2];
	for (i = 0; i < n * 2; i += 2)
		ret |= ov9640_write(regs[i], regs[i + 1]);
	return OV_ERR_NONE;

}
int ov9640_set_expose_compensation(int bright)
{
	int ret = 0;
	const u8 EV[] = {
		//EV-3
		0x24, 0x1c,
		0x25, 0x14,
		//EV-3
		0x24, 0x28,
		0x25, 0x20,
		//EV-2
		0x24, 0x38,
		0x25, 0x30,
		//EV-1
		0x24, 0x50,
		0x25, 0x48,
		//EV-0
		0x24, 0x70,
		0x25, 0x68,
		//EV+1
		0x24, 0x90,
		0x25, 0x88,
		//EV+2
		0x24, 0xb4,
		0x25, 0xac,
		//EV+3
		0x24, 0xc8,
		0x25, 0xc0,
		//EV+4
		0x24, 0xe0,
		0x25, 0xd8,
	};
	u8 *regs;
	int n = 2;
	int i;

	DPRINTK("in function %s bright =%d \n", __FUNCTION__, bright);
	if (bright < -4)
		bright = -4;
	if (bright > 4)
		bright = 4;
	//bright = -4 .. 4
	regs = &EV[(bright + 4) * n * 2];
	for (i = 0; i < n * 2; i += 2)
		ret |= ov9640_write(regs[i], regs[i + 1]);
//      if ( ret) 
//              return OV_ERR_PARAMETER;
	return OV_ERR_NONE;

}

int ov9640_set_white_balance(V4l_PIC_WB light)
{
	int ret = 0;
	const u8 WB_v3[] = {
		//V4l_WB_DIRECT_SUN
		0x13, 0xad,
		0x01, 0x80,
		0x02, 0x80,
		0x45, 0x20,
		0x46, 0x20,
		0x47, 0x58,
		0x48, 0x58,
		0x5C, 0x7a,
		0x5D, 0x5d,
		0x5F, 0x00,
		0x60, 0x14,
		0x13, 0xaf,
		0x14, 0x4a,
		//V4l_WB_INCANDESCENT home mode
		0x13, 0xad,
		0x01, 0x80,
		0x02, 0x80,
		0x45, 0x64,
		0x46, 0x64,
		0x47, 0x20,
		0x48, 0x20,
		0x5C, 0x7a,
		0x5D, 0x48,
		0x5F, 0x00,
		0x60, 0x14,
		0x13, 0xaf,
		0x14, 0x8a,
		//V4l_WB_FLUORESCENT office mode
		0x13, 0xad,
		0x01, 0x80,
		0x02, 0x80,
		0x45, 0x64,
		0x46, 0x64,
		0x47, 0x58,
		0x48, 0x58,
		0x5C, 0x7a,
		0x5D, 0x5d,
		0x5F, 0x00,
		0x60, 0x14,
		0x13, 0xaf,
		0x14, 0x8a,
		// auto
		0x13, 0xaf,
		0x01, 0x80,
		0x02, 0x80,
/*
		0x43, 0xF0,
		0x44, 0x10,
		0x45, 0x6C,
		0x46, 0x6c,
		0x47, 0x44,
		0x48, 0x44,
		0x59, 0x49,
		0x5A, 0x94,
		0x5B, 0x46,
		0x5C, 0x84,
		0x5D, 0x5c,
		0x5E, 0x08,
		0x5F, 0x00,
		0x60, 0x14,
		0x61, 0xCE,
	*/
		0x43, 0xF0,
		0x44, 0x10,
		0x45, 0x20,
		0x46, 0x20,
		0x47, 0x20,
		0x48, 0x20,
		0x59, 0x27,
		0x5A, 0x72,
		0x5B, 0x56,
		0x5C, 0x7a,
		0x5D, 0x5d,
		0x5E, 0x17,
		0x5F, 0x00,
		0x60, 0x14,
		0x61, 0xCE,

		0x14, 0x8a,
	};
	u8 *regs;
	int n = 13;
	int i = 0;
	ov9640 *pov;

	pov = &g_ov;

	DPRINTK("in function %s ,parameter=%d\n", __FUNCTION__, light);
	switch (light) {
	case V4l_WB_DIRECT_SUN:
		//ov9640_set_color_saturation(3);
		DPRINTK("V4l_WB_DIRECT_SUN\n");
		i = 0 * n * 2;
		//ov9640_set_night_mode();
		break;
	case V4l_WB_INCANDESCENT:
		//ov9640_set_color_saturation(2);
		DPRINTK("V4l_WB_INCANDESCENT\n");
		i = 1 * n * 2;
		//ov9640_set_action_mode();
		break;
	case V4l_WB_FLUORESCENT:
		//ov9640_set_color_saturation(2);
		DPRINTK("V4l_WB_FLUORESCENT\n");
		i = 2 * n * 2;
		break;
	default:
		/* auto */
		//ov9640_set_color_saturation(2);
		DPRINTK("case default ????????????????????\n");
		i = 3 * n * 2;
		n = 19;
		//ov9640_set_auto_mode();
		break;
	}
	regs = &WB_v3[i];
	if (pov->version == ((PID_OV_v3 << 8) | (PID_9640_v3))) {
		DPRINTK("ver 3 sensor!\n");
		regs = &WB_v3[i];
	}

	for (i = 0; i < n * 2; i += 2)
		ret |= ov9640_write(regs[i], regs[i + 1]);
	return OV_ERR_NONE;
}

int ov9640_set_night_mode()
{
	const u8 NM_v3[] = {
		//auto
		0x11, 0x89,
		//night mode
		//action mode
	};
	ov9640 *pov;

	pov = &g_ov;
	pov->night_mode = 1;
	//ov9640_write(0x11, 0x89);
	ov9640_write(0x3b, 0xe1);

}

int ov9640_set_action_mode()
{
	ov9640 *pov;

	pov = &g_ov;
	pov->night_mode = 2;
	ov9640_write(0x11, 0x81);
	ov9640_write(0x3b, 0x01);
	ov9640_write(0x2d, 0x0);
	ov9640_write(0x2e, 0x0);


}

int ov9640_set_auto_mode()
{
	ov9640 *pov;

	pov = &g_ov;
	pov->night_mode = 0;
	ov9640_write(0x11, 0x83);
	ov9640_write(0x3b, 0x01);
	ov9640_write(0x2d, 0x0);
	ov9640_write(0x2e, 0x0);
}

int ov9640_set_light_environment(V4l_PIC_WB light)
{
	int ret = 0;
	const u8 LE_v3[] = {
		//OutDoor
		0x3b, 0x00,
		0x2a, 0,
		0x2b, 0,
		0x6a, 0x3d,
		//Indoor 60Hz(Default)
		0x3b, 0x01,
		0x2a, 0,
		0x2b, 0,
		0x6a, 0x3d,
		//Indoor 50Hz
		0x3b, 0x01,
		0x2a, 0x10,
		0x2b, 0x40,
		0x6a, 0x3d,
	};
	const u8 LE_v2[] = {
		//OutDoor
		0x13, 0x8d,
		0x2a, 0,
		0x2b, 0,
		0x6a, 0x3d,
		//Indoor 60Hz(Default)
		0x13, 0x8d,
		0x2a, 0,
		0x2b, 0,
		0x6a, 0x3d,
		//Indoor 50Hz
		0x13, 0x8d,
		0x2a, 0x10,
		0x2b, 0x14,
		0x6a, 0x3d,
	};
	u8 *regs;
	int n = 4;
	int i = 0;
	ov9640 *pov;

	pov = &g_ov;
	DPRINTK("in function %s ,parameter=%d\n", __FUNCTION__, light);
	switch (light) {
	case 0:
		i = 0 * n * 2;
		break;
	case 60:
		i = 1 * n * 2;
		break;
	case 50:
		i = 2 * n * 2;
		break;
	default:
		i = 0;
		break;
	}

	regs = &LE_v2[i];
	if (pov->version == ((PID_OV_v3 << 8) | (PID_9640_v3))) {
		DPRINTK("ver 3 sensor!\n");
		regs = &LE_v3[i];
	}


	for (i = 0; i < n * 2; i += 2)
		ret |= ov9640_write(regs[i], regs[i + 1]);
	return OV_ERR_NONE;
}

int ov9640_set_color_saturation(int saturation)
{
	const u8 CS[] = {
		//Saturation: 0.25
		0x4f, 0x14,
		0x50, 0x10,
		0x51, 0x3,
		0x52, 0x6,
		0x53, 0x13,
		0x54, 0x19,
		//Saturation 0.5
		0x4f, 0x28,
		0x50, 0x22,
		0x51, 0x6,
		0x52, 0xc,
		0x53, 0x26,
		0x54, 0x33,
		//Saturation 0.75 (Default)
		0x4f, 0x3c,
		0x50, 0x32,
		0x51, 0x9,
		0x52, 0x13,
		0x53, 0x39,
		0x54, 0x4c,
		//Saturation 1.0
		0x4f, 0x50,
		0x50, 0x43,
		0x51, 0xd,
		0x52, 0x19,
		0x53, 0x4d,
		0x54, 0x66,
		//Saturation 1.25
		0x4f, 0x64,
		0x50, 0x53,
		0x51, 0x10,
		0x52, 0x1f,
		0x53, 0x5f,
		0x54, 0x7f,
	};
	u8 *regs;
	int n = 6;
	int i;

	DPRINTK("in function %s ,parameter=%d\n", __FUNCTION__, saturation);
	if (saturation < 0)
		saturation = 0;
	if (saturation > 4)
		saturation = 4;

	regs = &CS[saturation * n * 2];
	for (i = 0; i < n * 2; i += 2)
		ov9640_write(regs[i], regs[i + 1]);
	return OV_ERR_NONE;

}

int ov9640_set_contrast(int contrast)
{
	const u8 CO[] = {
		//Low contrast
		0x6C, 0x20,
		0x6D, 0x50,
		0x6E, 0xc0,
		0x6F, 0xa8,
		0x70, 0x88,
		0x71, 0x80,
		0x72, 0x78,
		0x73, 0x70,
		0x74, 0x68,
		0x75, 0x58,
		0x76, 0x40,
		0x77, 0x30,
		0x78, 0x28,
		0x79, 0x20,
		0x7A, 0x1e,
		0x7B, 0x18,
		0x7C, 0x04,
		0x7D, 0x07,
		0x7E, 0x1f,
		0x7F, 0x49,
		0x80, 0x5a,
		0x81, 0x6a,
		0x82, 0x79,
		0x83, 0x87,
		0x84, 0x94,
		0x85, 0x9f,
		0x86, 0xaf,
		0x87, 0xbb,
		0x88, 0xcf,
		0x89, 0xdf,
		0x8A, 0xee,
		//Middle contrast (default)
		0x6C, 0x40,
		0x6D, 0x30,
		0x6E, 0x4B,
		0x6F, 0x60,
		0x70, 0x70,
		0x71, 0x70,
		0x72, 0x70,
		0x73, 0x70,
		0x74, 0x60,
		0x75, 0x60,
		0x76, 0x50,
		0x77, 0x48,
		0x78, 0x3A,
		0x79, 0x2E,
		0x7A, 0x28,
		0x7B, 0x22,
		0x7C, 0x04,
		0x7D, 0x07,
		0x7E, 0x10,
		0x7F, 0x28,
		0x80, 0x36,
		0x81, 0x44,
		0x82, 0x52,
		0x83, 0x60,
		0x84, 0x6C,
		0x85, 0x78,
		0x86, 0x8C,
		0x87, 0x9E,
		0x88, 0xBB,
		0x89, 0xD2,
		0x8A, 0xE6,
		//High contrast
		0x6c, 0x50,
		0x6d, 0x60,
		0x6e, 0x58,
		0x6f, 0x58,
		0x70, 0x58,
		0x71, 0x50,
		0x72, 0x50,
		0x73, 0x50,
		0x74, 0x50,
		0x75, 0x50,
		0x76, 0x4c,
		0x77, 0x4c,
		0x78, 0x45,
		0x79, 0x3c,
		0x7a, 0x2c,
		0x7b, 0x24,
		0x7c, 0x05,
		0x7d, 0x0b,
		0x7e, 0x16,
		0x7f, 0x2c,
		0x80, 0x37,
		0x81, 0x41,
		0x82, 0x4b,
		0x83, 0x55,
		0x84, 0x5f,
		0x85, 0x69,
		0x86, 0x7c,
		0x87, 0x8f,
		0x88, 0xb1,
		0x89, 0xcf,
		0x8a, 0xe5,
	};

	u8 *regs;
	int n = 31;
	int i;

	DPRINTK("in function %s parameter=%d \n", __FUNCTION__, contrast);
	if (contrast < 0)
		contrast = 0;
	if (contrast > 2)
		contrast = 2;

	regs = &CO[contrast * n * 2];
	for (i = 0; i < n * 2; i += 2)
		ov9640_write(regs[i], regs[i + 1]);
	return OV_ERR_NONE;

}

int ov9640_set_back_light(int b_light)
{
	const u8 BL[] = {
		0x3B, 0,
		0x3B, 8,
		0x3B, 0x10,
		0x3B, 0x18,
	};
	u8 *regs;
	int n = 1;
	int i;

	DPRINTK("in function %s parameter=%d\n", __FUNCTION__, b_light);
	if (b_light < 0)
		b_light = 0;
	if (b_light > 3)
		b_light = 3;

	regs = &BL[b_light * n * 2];
	for (i = 0; i < n * 2; i += 2)
		ov9640_write(regs[i], regs[i + 1]);
	return OV_ERR_NONE;
}

int ov9640_stop_third_des(p_camera_context_t cam_ctx)
{
	pxa_dma_desc *pdesc, *ptmp;
	unsigned int phy_addr;
	int i;

	// stop the dma transfer on one frame captured
	pdesc = (pxa_dma_desc *) (cam_ctx->fifo0_descriptors_virtual);
	ptmp = pdesc + cam_ctx->fifo0_num_descriptors * 2;
	for (i = 0; i < cam_ctx->fifo0_num_descriptors; i++)
		(ptmp + i)->dtadr = (pdesc + i)->dtadr;

	pdesc += cam_ctx->fifo0_num_descriptors - 1;
	pdesc->dcmd = (pdesc->dcmd & DCMD_LENGTH) | DCMD_FLOWSRC | DCMD_BURST32 | DCMD_INCTRGADDR;
	pdesc += cam_ctx->fifo0_num_descriptors;
	pdesc->dcmd = (pdesc->dcmd & DCMD_LENGTH) | DCMD_FLOWSRC | DCMD_BURST32 | DCMD_INCTRGADDR;
	pdesc += cam_ctx->fifo0_num_descriptors;
	pdesc->ddadr |= 0x1;

	pdesc = (pxa_dma_desc *) (cam_ctx->fifo1_descriptors_virtual);
	ptmp = pdesc + cam_ctx->fifo1_num_descriptors * 2;
	for (i = 0; i < cam_ctx->fifo1_num_descriptors; i++)
		(ptmp + i)->dtadr = (pdesc + i)->dtadr;
	pdesc += cam_ctx->fifo1_num_descriptors * 3 - 1;
	pdesc->ddadr |= 0x1;

	pdesc = (pxa_dma_desc *) (cam_ctx->fifo2_descriptors_virtual);
	ptmp = pdesc + cam_ctx->fifo2_num_descriptors * 2;
	for (i = 0; i < cam_ctx->fifo2_num_descriptors; i++)
		(ptmp + i)->dtadr = (pdesc + i)->dtadr;
	pdesc += cam_ctx->fifo2_num_descriptors * 3 - 1;
	pdesc->ddadr |= 0x1;
}

int ov9640_stop_third_des1(p_camera_context_t cam_ctx)
{
	pxa_dma_desc *pdesc, *ptmp;
	unsigned int phy_addr;
	int i;

	// stop the dma transfer on one frame captured
	pdesc = (pxa_dma_desc *) (cam_ctx->fifo0_descriptors_virtual);
	ptmp = pdesc + cam_ctx->fifo0_num_descriptors * 2;
	/*
	   for (i=0;i<cam_ctx->fifo0_num_descriptors;i++)
	   (ptmp+i)->dtadr = (pdesc+i)->dtadr;
	 */
	pdesc += cam_ctx->fifo0_num_descriptors - 1;
	pdesc->dcmd = DCMD_FLOWSRC | DCMD_BURST32;
	pdesc += cam_ctx->fifo0_num_descriptors;
	pdesc->dcmd = DCMD_FLOWSRC | DCMD_BURST32;
	pdesc += cam_ctx->fifo0_num_descriptors;
	pdesc->ddadr |= 0x1;

	pdesc = (pxa_dma_desc *) (cam_ctx->fifo1_descriptors_virtual);
	ptmp = pdesc + cam_ctx->fifo1_num_descriptors * 2;
	/*
	   for (i=0;i<cam_ctx->fifo1_num_descriptors;i++)
	   (ptmp+i)->dtadr = (pdesc+i)->dtadr;
	 */
	pdesc += cam_ctx->fifo1_num_descriptors * 3 - 1;
	pdesc->ddadr |= 0x1;

	pdesc = (pxa_dma_desc *) (cam_ctx->fifo2_descriptors_virtual);
	ptmp = pdesc + cam_ctx->fifo2_num_descriptors * 2;
	/*
	   for (i=0;i<cam_ctx->fifo2_num_descriptors;i++)
	   (ptmp+i)->dtadr = (pdesc+i)->dtadr;
	 */
	pdesc += cam_ctx->fifo2_num_descriptors * 3 - 1;
	pdesc->ddadr |= 0x1;
}

int find_window(struct video_window *vw, int *sub_win)
{
	int ret = OV_SIZE_NONE;

	*sub_win = 1;
	if (vw->width > 1280 || vw->height > 960 || vw->width < 88 || vw->height < 72) {
		ret = OV_SIZE_NONE;
	}
	else if (vw->width == 1280 && vw->height == 960) {
		*sub_win = 0;
		ret = OV_SIZE_SXGA;
	}
	else if (vw->width >= 640 && vw->height >= 480) {
		if (vw->width == 640 && vw->height == 480)
			*sub_win = 0;
		ret = OV_SIZE_VGA;
	}
	else if (vw->width >= 352 && vw->height >= 288) {
		if (vw->width == 352 && vw->height == 288)
			*sub_win = 0;
		ret = OV_SIZE_CIF;
	}
	else if (vw->width >= 320 && vw->height >= 240) {
		if (vw->width == 320 && vw->height == 240)
			*sub_win = 0;
		ret = OV_SIZE_QVGA;
	}
	else if (vw->width >= 176 && vw->height >= 144) {
		if (vw->width == 176 && vw->height == 144)
			*sub_win = 0;
		ret = OV_SIZE_QCIF;
	}
	else if (vw->width >= 160 && vw->height >= 120) {
		if (vw->width == 160 && vw->height == 120)
			*sub_win = 0;
		ret = OV_SIZE_QQVGA;
	}
	else if (vw->width >= 88 && vw->height >= 72) {
		if (vw->width == 88 && vw->height == 72)
			*sub_win = 0;
		ret = OV_SIZE_QQCIF;
	}
	DPRINTK("in %s,ret = %d, subwin=%d\n", __FUNCTION__, ret, *sub_win);
	return ret;
}

int ov9640_set_window(struct video_window *vw)
{
	int ret = 0;
	int x_start, x_end;
	int y_start, y_end;
	struct video_window window;
	int width;
	int height;
	int sub_win;
	ov9640 *pov;

	pov = &g_ov;
	vw->width = (vw->width + 7) & (~0x7);
	vw->height = (vw->height + 7) & (~0x7);
	vw->x = vw->y = 0;
	x_end = window.width = (vw->width + vw->x);
	y_end = window.height = (vw->height + vw->y);
	DPRINTK("in %s, vw-x =%d, vw-y=%d,vw->width=%d,vw->height=%d\n",
		__FUNCTION__, vw->x, vw->y, vw->width, vw->height);
	ret = find_window(&window, &sub_win);
	if (ret <= OV_SIZE_NONE)
		return -1;

	ret = ov9640_set_format(ret, OV_FORMAT_YUV_422);
	if (ret < 0)
		return -1;

	/*
	   if ((sub_win == 1) || (vw->x != 0) || (vw->y != 0) || (pov->sub_win != sub_win)) {
	   u8 href = 0;
	   u8 vref = 0;
	   ov9640_read(0x32, &href);
	   href = href & 0xc0 | (vw->width & 0x7) << 3 | (vw->x & 0x7);
	   ov9640_write(0x17, vw->x >> 3);
	   ov9640_write(0x18, x_end >> 3);
	   ov9640_write(0x19, vw->y >> 2);
	   ov9640_write(0x20, y_end >> 2);
	   ov9640_read(0x3, &vref);
	   vref = vref & 0xf0 | (vw->height & 0x3) << 2 | (vw->y & 0x3);
	   pov->sub_win = sub_win;
	   }
	 */
	pov->win = *vw;
	return ret;
}

int ov9640_get_window(struct video_window *vw)
{
	ov9640 *pov;
	pov = &g_ov;
	*vw = pov->win;
	return 0;
}

struct win_size {
	int width;
	int height;
};
int ov9640_set_sensor_size(void *w_size)
{
	struct win_size size;
	ov9640 *pov;
	pov = &g_ov;
	if (copy_from_user(&size, w_size, sizeof(struct win_size))) {
		return -EFAULT;
	}
//make it in an even of multiple of 8
	size.height = (size.height + 7) / 8 * 8;
	pov->sensor_width = size.width;
	pov->sensor_height = size.height;
	return 0;
}

int ov9640_get_sensor_size(void *w_size)
{
}
int ov9640_set_output_size(void *w_size)
{
	struct win_size size;
	ov9640 *pov;
	pov = &g_ov;
	if (copy_from_user(&size, w_size, sizeof(struct win_size))) {
		return -EFAULT;
	}
//make it in an even of multiple of 8
	size.height = (size.height + 7) / 8 * 8;
	pov->sensor_width = size.width;
	pov->sensor_height = size.height;
	return 0;
}

int ov9640_get_output_size(void *w_size)
{
}
int test_divider(int res, int fps)
{
	int max_hz = 48 * 1000000;
	int div = 1;
	int i;
	int ov_fps[5] = { 3, 7, 15, 30, 60 };
	u32 value = 320 * 240;
	/*
	   switch (prevSize) {
	   case OV_SIZE_QQCIF:
	   value = 88 * 72;
	   break;
	   case OV_SIZE_QQVGA:
	   value = 176 * 144;
	   break;
	   case OV_SIZE_QCIF:
	   value = 160 * 120;
	   break;
	   case OV_SIZE_QVGA:
	   value = 320 * 240;
	   break;
	   case OV_SIZE_CIF:
	   value = 352 * 288;
	   break;
	   case OV_SIZE_VGA:
	   value = 640 * 480;
	   break;
	   case OV_SIZE_SXGA:
	   value = 1280 * 960;
	   break;
	   }

	   while (max_hz / res / div > fps)
	   div++;
	   if (div > 64)
	   div = 64;
	   return (div - 1);

	   for (i =0;i<5;i++)
	   if ( fps < ov_fps[i] ) 
	   continue;
	 */
	if (fps == 0)
		return 0;
	if (fps > 60)
		fps = 60;
	return (div = 60 / fps - 1);
}

int ov9640_set_fps(int fps, int min_fps)
{
	u32 res = 0;
	u8 value;
	if (fps < 0) {
		DPRINTK("in %s fps = %d divider value =%d\n", __FUNCTION__, fps, res);
		fps = 15;
	}
	res = test_divider(0, fps);
	ov9640_read(OV9640_CLKRC, &value);
	value = (value & 0xc0) | res;
	ov9640_write(OV9640_CLKRC, value);
	DPRINTK("in %s fps = %d divider value =%d\n", __FUNCTION__, fps, res);
	/*
	   ov9640_set_night_mode();
	   ov9640_set_action_mode();
	   ov9640_set_auto_mode();
	 */
	return 0;
}

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

void write_balance()
{
	u8 yav, uav, vav;
	static int bg = 0x80;
	static int rg = 0x24;
	int awb_mode;
	ov9640 *pov;

	pov = &g_ov;

	if (pov->version == ((PID_OV_v3 << 8) | (PID_9640_v3))) {
		DPRINTK("ver 3 sensor!\n");
		return;
	}


	prv_read_sensor_reg(0x5, &vav);
	prv_read_sensor_reg(0x6, &yav);
	prv_read_sensor_reg(0x8, &uav);
	DPRINTK("in %s, v =%x ,y =%x,u =%x\n)", __FUNCTION__, vav, yav, uav);
	if (yav >= 0x20 && yav <= 0xc0) {
		if (vav < 50)
			rg = MIN(255, rg + 40);
		else if (vav < 80)
			rg = MIN(255, rg + 20);
		else if (vav < 100)
			rg = MIN(255, rg + 5);
		else if (vav < 120)
			rg = MIN(255, rg + 1);
		else if (vav > 206)
			rg = MAX(0, rg - 40);
		else if (vav > 180)
			rg = MAX(0, rg - 20);
		else if (vav > 156)
			rg = MAX(0, rg - 5);
		else if (vav > 136)
			rg = MAX(0, rg - 1);

		if (uav < 50)
			bg = MIN(255, bg + 40);
		else if (uav < 80)
			bg = MIN(255, bg + 20);
		else if (uav < 100)
			bg = MIN(255, bg + 5);
		else if (uav < 120)
			bg = MIN(255, bg + 1);
		else if (uav > 206)
			bg = MAX(0, bg - 40);
		else if (uav > 180)
			bg = MAX(0, bg - 20);
		else if (uav > 156)
			bg = MAX(0, bg - 5);
		else if (uav > 136)
			bg = MAX(0, bg - 1);
	}
	ov9640_write(0x01, bg);
	ov9640_write(0x2, rg);
}
