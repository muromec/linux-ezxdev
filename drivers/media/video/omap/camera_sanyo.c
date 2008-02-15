/*
 * camera_sanyo.c
 *
 * Implementation of Camera for Sanyo digital camera.
 *
 * Copyright (C) 2003 MontaVista Software, Inc.  All rights reserved.
 * Copyright (C) 2002 RidgeRun, Inc.  All rights reserved.
 * Copyright (C) 2002 Texas Instruments  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/config.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/videodev.h>
#include <asm/uaccess.h>
#define MODULE_NAME "sanyo"
#include "common.h"
#include "camif.h"

#define SANYO_BPP 2

#define HORZCONST_RISING_VGA   164  // as per CCD chip specs.
#define HORZCONST_FALLING_VGA  804  // as per CCD chip specs.
#define HORZCONST_RISING_QVGA  167  // as per CCD chip specs.
#define HORZCONST_FALLING_QVGA 807  // as per CCD chip specs.
#define VERTCONST_RISING_VGA     8  // as per CCD chip specs.
#define VERTCONST_FALLING_VGA  488  // as per CCD chip specs.
#define MAXWIDTH_VGA           640  // as per CCD chip specs.
#define MAXHEIGHT_VGA          480  // as per CCD chip specs.
#define MAXWIDTH_VGAQ          320  // as per CCD chip specs.
#define MAXHEIGHT_VGAQ         240  // as per CCD chip specs.

static struct camera * this;

static volatile int PHOTO_COUNTER;

// **************************
// Routine:
// Description:
// **************************
static int convert_sanyo_image(u8* src, void* dest,
			       int to_user,
			       int dest_stride,
			       struct v4l2_pix_format* fmt)
{
	int x, y;
	int line_gap;
	int dest_BPP = fmt->depth / 8;
	int width = omap_image_size[this->imgfmt].width;
	int height = omap_image_size[this->imgfmt].height;
	
	line_gap = dest_stride - (width * dest_BPP);

	src += 4;    // added dakoo
	
	/*
	 * Correct the rgb565 pixel ordering.
	 *
	 * Note:
	 *   The camera chip streamed RGB565 bytes to the omap1510 camera
	 *   interface module in the order:
	 *          GB1(1st pixel lo_byte)
	 *          RG1(1st pixel hi_byte)
	 *          GB2(2nd pixel lo_byte)
	 *          RG2(2nd pixel hi_byte)
	 *          etc...
	 *   but due to the way each four byte stream was placed on the RHEA
	 *   bus when the DMA read them and then subsequently stored them
	 *   into our buffer (with 32bits rd/wr), we will now encounter the
	 *   bytes in memory in this order.
	 *          GB2(2nd pixel lo_byte)
	 *          RG2(2nd pixel hi_byte)
	 *          GB1(1st pixel lo_byte)
	 *          RG1(1st pixel hi_byte)
	 *          etc...
	 *   Hence we see that each 16bit pixel has its bytes ordered correctly
	 *   in memory to support 16bit little endian reading of the pixel,
	 *   that is good, unfortunately the two pixels within a 32bit word
	 *   are swapped and we'll need to consider that as we display the
	 *   image here.
	 */
	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_RGB565:
		// Convert camera data into rgb565.
		for (y = 0; y < height; y++) {
			for (x = 0 ; x < width/2; x++) {
				u16 rgb1, rgb2;
				rgb2 = *((u16*)src)++;
				rgb1 = *((u16*)src)++;
				if (to_user) {
					put_user(rgb1, ((u16*)dest)++);
					put_user(rgb2, ((u16*)dest)++);
				} else {
					*((u16*)dest)++ = rgb1;
					*((u16*)dest)++ = rgb2;
				}
			}
			dest += line_gap;
		}
		break;
	case V4L2_PIX_FMT_BGR24:
		// Convert camera data into BGR24.
		for (y = 0; y < height; y++) {
			for (x = 0 ; x < width/2; x++) {
				u8 B1, G1, R1;
				u8 B2, G2, R2;
				u8 S;

				S = *src++;
				G2 = S>>5;
				// The 5 bits of blue made into 8.
				B2 = S<<3;
				S = *src++;
				// The 6 bits of green made into 8.
				G2 = (G2 | ((S&0x7)<<3))<<2;
				// The 5 bits of red made into 8.
				R2 = S&0xf8;
				
				S = *src++;
				G1 = S>>5;
				// The 5 bits of blue made into 8.
				B1 = S<<3;
				S = *src++;
				// The 6 bits of green made into 8.
				G1 = (G1 | ((S&0x7)<<3))<<2;
				// The 5 bits of red made into 8.
				R1 = S&0xf8;

				/*
				 * And finally write them to the target
				 * buffer. Note the following 16bit writes
				 * result in the bytes B1,G1,R1,B2,G2,R2
				 * being stored sequentially in memory.
				 */
				if (to_user) {
					put_user((G1<<8) | B1,
						 ((u16*)dest)++);
					put_user((B2<<8) | R1,
						 ((u16*)dest)++);
					put_user((R2<<8) | G2,
						 ((u16*)dest)++);
				} else {
					*((u16*)dest)++ = (G1<<8) | (B1);
					*((u16*)dest)++ = (B2<<8) | (R1);
					*((u16*)dest)++ = (R2<<8) | (G2);
				}
			}

			dest += line_gap;
		}
		break;
	default:
		err("unsupported conversion request.\n");
		return -ENXIO;
	}

	return 0;
}


// **************************
// Routine:
// Description:
//   The setup that can be done essentially once per power-cycle.
// **************************
static int open_sanyo(void)
{
	struct camera_serial_bus * sbus = this->camif->sbus;

#if 1  // This set is for memory mode on  QVGA
	sbus->write_verify(0x0,0x0); // Register Reset activated (step1of2)
	sbus->write_verify(0x0,0x1); // Register Reset deactivated (step2of2).
	sbus->write_verify(0xF8,0x0); // Circuit Reset activated (step1of2)
	sbus->write_verify(0xF8,0x1);  //Circuit Reset deactivated (step2of2)
	sbus->write_verify(0x9,0x0);   // Drv Setup (step2of3)
	sbus->write_verify(0x9,0x1);  // Drv Setup (step3of3)
	sbus->write_verify(0x9,0x25);   // Drv Setup (step1of3)
	sbus->write_verify(0x9,0x3F);   // Drv Setup (step2of3)
	sbus->write_verify(0x9,0x45);  // Drv Setup (step3of3)
	sbus->write_verify(0x9,0x57);   // Drv Setup (step1of3)
	sbus->write_verify(0x9,0x6F);   // Drv Setup (step2of3)
	sbus->write_verify(0x9,0x7E);  // Drv Setup (step3of3)
	sbus->write_verify(0x9,0x84);   // Drv Setup (step1of3)
	sbus->write_verify(0xB,0x0);   // Drv Setup
	sbus->write_verify(0xC,0x20);  // ---------20---------------should it be 0x30?
	sbus->write_verify(0xD,0x58);   // ---------58--------------should it be 0x52 ?
	sbus->write_verify(0xE,0x0);   // Drv Setup
	sbus->write_verify(0xF,0xC3);  // changed changes in august
	sbus->write_verify(0x10,0x1);   // Drv Setup
	sbus->write_verify(0x11,0xFB);   // Drv Setup -------------------
	sbus->write_verify(0x12,0x6);  // Drv Setup ---------------------
	sbus->write_verify(0x13,0x18);  // changed changes in august
	sbus->write_verify(0x14,0x4);  // changed changes in august
	sbus->write_verify(0x15,0x15);  // changed changes in august
	sbus->write_verify(0x16,0xC0);   // Drv Setup
	sbus->write_verify(0x1A,0x20);   // changed changes in august
	sbus->write_verify(0x1E,0x0);  // -------0 means PCLK input clock
	sbus->write_verify(0x1F,0x11);   // -----no of lines 224?
	sbus->write_verify(0x20,0xF1);   // ----no of lines
	sbus->write_verify(0x21,0x1);  // Drv Setup
 	sbus->write_verify(0x22,0x2);  // changed changes in august
	sbus->write_verify(0x29,0x8);   // Drv Setup (step2of3)
	sbus->write_verify(0x2A,0x9);  // Drv Setup (step3of3)
	sbus->write_verify(0x2B,0x7);   // Drv Setup (step1of3)
	sbus->write_verify(0x2F,0x24);   //
	sbus->write_verify(0x38,0x1);   // thats for cref instead of href
	sbus->write_verify(0x3A,0xA0);   // Drv Setup
	sbus->write_verify(0x3B,0x61);  // Drv Setup (step3of3)
	sbus->write_verify(0x3C,0x16);   // Drv Setup (step1of3)
	sbus->write_verify(0x3D,0x16);   // Drv Setup (step2of3)
	sbus->write_verify(0x3E,0x19);  // Drv Setup (step3of3)
	sbus->write_verify(0x3F,0xBC);   // Drv Setup (step1of3)
	sbus->write_verify(0x40,0x2);   // Drv Setup (step2of3)
	sbus->write_verify(0x41,0x0);  // Drv Setup (step3of3)
	sbus->write_verify(0x51,0x2);   // Drv Setup (step1of3)
	sbus->write_verify(0x52,0xC);   // Drv Setup (step2of3)
	sbus->write_verify(0x53,0xE);  // Drv Setup (step3of3)
	sbus->write_verify(0x54,0x17);   // Drv Setup (step1of3)
	sbus->write_verify(0x55,0x50);   // Drv Setup (step2of3)
	sbus->write_verify(0x56,0x7F);  // Drv Setup (step3of3)
	sbus->write_verify(0x57,0x50);   // Drv Setup (step1of3)
	sbus->write_verify(0x58,0x0);   // Drv Setup (step2of3)
	sbus->write_verify(0x5B,0x6);  // Drv Setup (step3of3)
	sbus->write_verify(0x5C,0x2);   // Drv Setup (step1of3)
	sbus->write_verify(0x5D,0x18);   // Drv Setup (step2of3)
	sbus->write_verify(0x5F,0xF);  // Drv Setup (step3of3)
	sbus->write_verify(0x64,0x1D);   // Drv Setup (step1of3)
	sbus->write_verify(0x65,0x34);   // Drv Setup (step2of3)
	sbus->write_verify(0x66,0x33);  // Drv Setup (step3of3)
	sbus->write_verify(0x67,0x14);   // Drv Setup (step1of3)
	sbus->write_verify(0x68,0x2D);   // Drv Setup (step2of3)
	sbus->write_verify(0x69,0x45);  // Drv Setup (step3of3)
	sbus->write_verify(0x6A,0x64);   // Drv Setup (step1of3)
	sbus->write_verify(0x6B,0x96);   // Drv Setup (step2of3)
	sbus->write_verify(0x6C,0x12);  // Drv Setup (step3of3)
	sbus->write_verify(0x6D,0x24);   // Drv Setup (step1of3)
	sbus->write_verify(0x6E,0x23);   // Drv Setup (step2of3)
	sbus->write_verify(0x6F,0x21);  // Drv Setup (step3of3)
	sbus->write_verify(0x70,0x5);   // Drv Setup (step1of3)
	sbus->write_verify(0x71,0xF);   // Drv Setup (step2of3)
	sbus->write_verify(0x75,0xA);  // Drv Setup (step3of3)
	sbus->write_verify(0x77,0xAF);  // Drv Setup (step1of3)
	sbus->write_verify(0x78,0x75);   // Drv Setup (step2of3)
	sbus->write_verify(0x7C,0x3E);  // Drv Setup (step3of3)
	sbus->write_verify(0x7D,0x54);   // Drv Setup (step1of3)
	sbus->write_verify(0x7E,0x7B);   // Drv Setup (step2of3)
	sbus->write_verify(0x7F,0x87);  // Drv Setup (step3of3)
	sbus->write_verify(0x86,0xE0);   // Drv Setup (step1of3)
	sbus->write_verify(0x87,0xEE);   // Drv Setup (step2of3)
	sbus->write_verify(0x88,0xE);  // Drv Setup (step3of3)
	sbus->write_verify(0x89,0x1D);   // Drv Setup (step1of3)
	sbus->write_verify(0x8A,0x90);   // Drv Setup (step2of3)
	sbus->write_verify(0x8B,0xB0);  // Drv Setup (step3of3)
	sbus->write_verify(0x8C,0x76);   // Drv Setup (step1of3)
	sbus->write_verify(0x8D,0x6C);   // Drv Setup (step2of3)
	sbus->write_verify(0x8E,0x39);  // Drv Setup (step3of3)
	sbus->write_verify(0x8F,0x72);   // Drv Setup (step1of3)
	sbus->write_verify(0x90,0x7D);   // Drv Setup (step2of3)
	sbus->write_verify(0x92,0x0);  // Drv Setup (step3of3)
	sbus->write_verify(0x93,0xF);  // Iris hold time
	sbus->write_verify(0x95,0x5);   // Drv Setup (step2of3)
	sbus->write_verify(0x99,0x7B);  // Drv Setup (step3of3)
	sbus->write_verify(0x9A,0x5A);   // Drv Setup (step1of3)
	sbus->write_verify(0x9B,0xC8);   // Drv Setup (step2of3)
	sbus->write_verify(0xB4,0x22);  // Drv Setup (step3of3)
	sbus->write_verify(0xB5,0x22);   // Drv Setup (step1of3)
	sbus->write_verify(0xB6,0x22);   // Drv Setup (step2of3)
	sbus->write_verify(0xB7,0x22);  // Drv Setup (step3of3)
	sbus->write_verify(0xBE,0x52);   // Drv Setup (step1of3)
	sbus->write_verify(0xC2,0x2);   // Drv Setup (step2of3)
	sbus->write_verify(0xCD,0x1);  // Drv Setup (step3of3)
	sbus->write_verify(0xD2,0xF);   // Drv Setup (step1of3)
	sbus->write_verify(0xD4,0x3);   // Drv Setup (step2of3)
	sbus->write_verify(0xD5,0xC0);  // Drv Setup (step3of3)
	sbus->write_verify(0xD8,0xC2);   // Drv Setup (step1of3)
	sbus->write_verify(0xD9,0x7);   // Drv Setup (step2of3)
	sbus->write_verify(0xDA,0x21);  // Drv Setup (step3of3)
	sbus->write_verify(0xDC,0x11);   // Drv Setup (step1of3)
	sbus->write_verify(0x9,0x15);   // Drv Setup (step2of3)
	sbus->write_verify(0xB9,0x11);  // Drv Setup (step3of3)
	sbus->write_verify(0xB8,0xC2);   // Drv Setup (step1of3)
	sbus->write_verify(0x9,0x3);   // Drv Setup (step2of3)
	mdelay(10);
	sbus->write_verify(0x9,0x11);  // Drv Setup (step3of3)

	this->imgfmt = QVGA;
	this->pixfmt = RGB565;
#endif

	return 0;
}

static int close_sanyo(void)
{
	return 0;
}

static int sanyo_set_fp(int fp, int exclk, int test)
{
	return -EINVAL;
}
static int sanyo_get_fp(void)
{
	return -EINVAL;
}

static int init_sanyo(void)
{
	return 0;
}

static void cleanup_sanyo(void)
{
}

// **************************
// Routine:
// Description:
//   This is the setup that must occur to adjust
//   reg settings as per any new VIDIOC_S_FMT command
//   the client may have just perform to the driver.
//   In sanyo we can program the the windowing
// 	 once we setup the overall mode(QVGA or VGA).
//   To setup the windowing we will have to calculate
//   some initial settings just as omni is doing.
// **************************
static int setup_sanyo(struct v4l2_pix_format* fmt)
{
	struct camera_serial_bus * sbus = this->camif->sbus;
#if 0
	unsigned int H_start_REGLOW ;
	unsigned int H_start_REGHIGH;
	unsigned int H_end_REGLOW;
	unsigned int H_end_REGHIGH;
	unsigned int V_start_REGLOW;
	unsigned int V_start_REGHIGH;
	unsigned int V_end_REGLOW;
	unsigned int V_end_REGHIGH;
#endif
	
	// Next, Configure the remote camera module.
	// Calculate a centered image within the full CCD grid.

	if (fmt->width >= 640) {

		this->imgfmt = VGA;
		fmt->width = 640;
		fmt->height = 480;

		// Set it to VGA at around 8 fps, but here we need to put a global
		// counter that increaments with each use. We will check this counter
		// if we need to go back to QVGA and its subsets and will have to do
		// complete programming again because the viewfinder modes are the subset
		// of QVGA(for better qualitity) and the format changes only deal with
		// a subsetof the master registers in the camera DSP.
		PHOTO_COUNTER++;

		sbus->write_verify(0x0,0x0); // Register Reset activated (step1of2)
		sbus->write_verify(0x0,0x1); // Register Reset deactivated (step2of2).
		sbus->write_verify(0xF8,0x0); // Circuit Reset activated (step1of2)
		sbus->write_verify(0xF8,0x1);  //Circuit Reset deactivated (step2of2)
		sbus->write_verify(0x9,0x0);   // Drv Setup (step2of3)
		sbus->write_verify(0x9,0x1);  // Drv Setup (step3of3)
		sbus->write_verify(0x9,0x25);   // Drv Setup (step1of3)
		sbus->write_verify(0x9,0x3F);   // Drv Setup (step2of3)
		sbus->write_verify(0x9,0x45);  // Drv Setup (step3of3)
		sbus->write_verify(0x9,0x57);   // Drv Setup (step1of3)
		sbus->write_verify(0x9,0x6F);   // Drv Setup (step2of3)
		sbus->write_verify(0x9,0x7E);  // Drv Setup (step3of3)
		sbus->write_verify(0x9,0x84);   // Drv Setup (step1of3)
		sbus->write_verify(0xB,0x0);   // Drv Setup
		sbus->write_verify(0xC,0x20);  // ---------20---------------should it be 0x30?
		sbus->write_verify(0xD,0x58);   // ---------58--------------should it be 0x52 ?
		sbus->write_verify(0xE,0x0);   // Drv Setup
		sbus->write_verify(0xF,0xD3);  // changed from D7 to D3 to slow to 4fps
		sbus->write_verify(0x10,0x0);   // changed from 1 for half of pclk
		sbus->write_verify(0x11,0xFB);   // Drv Setup -------------------
		sbus->write_verify(0x12,0x6);  // Drv Setup ---------------------
		sbus->write_verify(0x13,0x5C);  // changed from 18 to 5C to slow to 4fps
		sbus->write_verify(0x14,0xF);  // changed from 4 to F to slow to 4fps
		sbus->write_verify(0x15,0x1A);  // changed from 15 to 1A to slow to 4fps
		sbus->write_verify(0x16,0xC0);   // Drv Setup
		sbus->write_verify(0x1A,0x0);   // changed from 0
		sbus->write_verify(0x1E,0x0);  // ------------------ 0 means PCLK input clock
		sbus->write_verify(0x1F,0x11);   // ------------------------no of lines 224?
		sbus->write_verify(0x20,0xF1);   // -----------------------no of lines
		sbus->write_verify(0x21,0x1);  // Drv Setup
		sbus->write_verify(0x22,0x3);  // changed changes in august
		sbus->write_verify(0x29,0x8);   // Drv Setup (step2of3)
		sbus->write_verify(0x2A,0x9);  // Drv Setup (step3of3)
		sbus->write_verify(0x2B,0x7);   // Drv Setup (step1of3)
		sbus->write_verify(0x2F,0x24);   //

		sbus->write_verify(0x30,0xA4);   // changed from nothing to A4 to slow to 4fps

		//sbus->write_verify(0x31,);   // H end of window

		sbus->write_verify(0x32,0x24);   // H start of window

		sbus->write_verify(0x33,0x3);   // H end of window

		sbus->write_verify(0x34,0x8);  // V start of window

		//sbus->write_verify(0x35,);   // H end of window

		sbus->write_verify(0x36,0xE8);  // V start of window

		//sbus->write_verify(0x37,);   // H end of window


		sbus->write_verify(0x38,0x1);   // -----------thats for cref instead of href
		sbus->write_verify(0x3A,0xA0);   // Drv Setup
		sbus->write_verify(0x3B,0x61);  // Drv Setup (step3of3)
		sbus->write_verify(0x3C,0x16);   // Drv Setup (step1of3)
		sbus->write_verify(0x3D,0x16);   // Drv Setup (step2of3)
		sbus->write_verify(0x3E,0x19);  // Drv Setup (step3of3)
		sbus->write_verify(0x3F,0xBC);   // Drv Setup (step1of3)
		sbus->write_verify(0x40,0x2);   // Drv Setup (step2of3)
		sbus->write_verify(0x41,0x0);  // Drv Setup (step3of3)
		sbus->write_verify(0x51,0x2);   // Drv Setup (step1of3)
		sbus->write_verify(0x52,0xC);   // Drv Setup (step2of3)
		sbus->write_verify(0x53,0xE);  // Drv Setup (step3of3)
		sbus->write_verify(0x54,0x17);   // Drv Setup (step1of3)
		sbus->write_verify(0x55,0x50);   // Drv Setup (step2of3)
		sbus->write_verify(0x56,0x7F);  // Drv Setup (step3of3)
		sbus->write_verify(0x57,0x50);   // Drv Setup (step1of3)
		sbus->write_verify(0x58,0x0);   // Drv Setup (step2of3)
		sbus->write_verify(0x5B,0x6);  // Drv Setup (step3of3)
		sbus->write_verify(0x5C,0x2);   // Drv Setup (step1of3)
		sbus->write_verify(0x5D,0x18);   // Drv Setup (step2of3)
		sbus->write_verify(0x5F,0xF);  // Drv Setup (step3of3)
		sbus->write_verify(0x64,0x1D);   // Drv Setup (step1of3)
		sbus->write_verify(0x65,0x34);   // Drv Setup (step2of3)
		sbus->write_verify(0x66,0x33);  // Drv Setup (step3of3)
		sbus->write_verify(0x67,0x14);   // Drv Setup (step1of3)

		sbus->write_verify(0x68,0x2D);   // Drv Setup (step2of3)
		sbus->write_verify(0x69,0x45);  // Drv Setup (step3of3)
		sbus->write_verify(0x6A,0x64);   // Drv Setup (step1of3)
		sbus->write_verify(0x6B,0x96);   // Drv Setup (step2of3)
		sbus->write_verify(0x6C,0x12);  // Drv Setup (step3of3)
		sbus->write_verify(0x6D,0x24);   // Drv Setup (step1of3)
		sbus->write_verify(0x6E,0x23);   // Drv Setup (step2of3)
		sbus->write_verify(0x6F,0x21);  // Drv Setup (step3of3)
		sbus->write_verify(0x70,0x5);   // Drv Setup (step1of3)
		sbus->write_verify(0x71,0xF);   // Drv Setup (step2of3)
		sbus->write_verify(0x75,0xA);  // Drv Setup (step3of3)
		sbus->write_verify(0x77,0xAF);  // Drv Setup (step1of3)
		sbus->write_verify(0x78,0x7D);   // changed from 75
		sbus->write_verify(0x7C,0x3E);  // Drv Setup (step3of3)
		sbus->write_verify(0x7D,0x54);   // Drv Setup (step1of3)
		sbus->write_verify(0x7E,0x7B);   // Drv Setup (step2of3)
		sbus->write_verify(0x7F,0x87);  // Drv Setup (step3of3)
		sbus->write_verify(0x82,0x3F);   // changed from -- to -- to slow to 4fps
		sbus->write_verify(0x86,0xE0);   // Drv Setup (step1of3)
		sbus->write_verify(0x87,0xEE);   // Drv Setup (step2of3)
		sbus->write_verify(0x88,0xE);  // Drv Setup (step3of3)
		sbus->write_verify(0x89,0x1D);   // Drv Setup (step1of3)
		sbus->write_verify(0x8A,0x90);   // Drv Setup (step2of3)
		sbus->write_verify(0x8B,0xB0);  // Drv Setup (step3of3)
		sbus->write_verify(0x8C,0x76);   // Drv Setup (step1of3)
		sbus->write_verify(0x8D,0x6C);   // Drv Setup (step2of3)
		sbus->write_verify(0x8E,0x39);  // Drv Setup (step3of3)
		sbus->write_verify(0x8F,0x72);   // Drv Setup (step1of3)
		sbus->write_verify(0x90,0x7D);   // Drv Setup (step2of3)
		sbus->write_verify(0x92,0x0);  // Drv Setup (step3of3)
		sbus->write_verify(0x93,0xF);  // changed from bellow for 15 fps in Memory mode ONN
		sbus->write_verify(0x95,0x5);   // Drv Setup (step2of3)
		sbus->write_verify(0x99,0x7B);  // Drv Setup (step3of3)
		sbus->write_verify(0x9A,0x5A);   // Drv Setup (step1of3)
		sbus->write_verify(0x9B,0xC8);   // Drv Setup (step2of3)
		sbus->write_verify(0xB4,0x22);  // Drv Setup (step3of3)
		sbus->write_verify(0xB5,0x22);   // Drv Setup (step1of3)
		sbus->write_verify(0xB6,0x22);   // Drv Setup (step2of3)
		sbus->write_verify(0xB7,0x22);  // Drv Setup (step3of3)
		sbus->write_verify(0xBE,0x52);   // Drv Setup (step1of3)
		sbus->write_verify(0xC2,0x2);   // Drv Setup (step2of3)
		sbus->write_verify(0xCD,0x1);  // Drv Setup (step3of3)
		sbus->write_verify(0xD2,0xF);   // Drv Setup (step1of3)
		sbus->write_verify(0xD4,0x3);   // Drv Setup (step2of3)
		sbus->write_verify(0xD5,0xC0);  // Drv Setup (step3of3)
		sbus->write_verify(0xD8,0xC2);   // Drv Setup (step1of3)
		sbus->write_verify(0xD9,0x7);   // Drv Setup (step2of3)
		sbus->write_verify(0xDA,0x21);  // Drv Setup (step3of3)
		sbus->write_verify(0xDC,0x11);   // Drv Setup (step1of3)
		sbus->write_verify(0x9,0x15);   // Drv Setup (step2of3)
		sbus->write_verify(0xB9,0x11);  // Drv Setup (step3of3)
		sbus->write_verify(0xB8,0xC2);   // Drv Setup (step1of3)
		sbus->write_verify(0x9,0x3);   // Drv Setup (step2of3)
		mdelay(10);
		sbus->write_verify(0x9,0x11);  // Drv Setup (step3of3)

	} else if (fmt->width >= 320) {

		this->imgfmt = QVGA;
		fmt->width = 320;
		fmt->height = 240;

		//	sbus->write_verify(0xF,0xC3); 	// .
		//  sbus->write_verify(0x13,0x1E); 	// frame rate.
		//  sbus->write_verify(0x14,0x05); 	// frame rate.
		//  sbus->write_verify(0x15,0x16); 	// frame rate.
		//  sbus->write_verify(0x1A,0x20); 	// frame rate
		//  sbus->write_verify(0x78,0x75); 	// frame rate
		//
		//  sbus->write_verify(0x82,0x37); 	// frame rate
		//  sbus->write_verify(0x90,0x7D); 	// frame rate.
		//  sbus->write_verify(0x93,0x0C); 	// frame rate.

		if (PHOTO_COUNTER > 0) {
			// needed in case we have been in VGA mode
			init_sanyo();
			PHOTO_COUNTER--;
		}

		//sbus->write_verify(0x13,0x12); // frame rate.
		//sbus->write_verify(0x14,0x3);  // frame rate, 20 fps
		/*sbus->write_verify(0x93,0x14); frame rate,
		  can only use if memory mode is off*/

		sbus->write_verify(0x30,0xA7); 	// horiz start of window.
		sbus->write_verify(0x31,0x0); 	// horiz start of window.
		sbus->write_verify(0x32,0x27); 	// horiz end of window.
		sbus->write_verify(0x33,0x3); 	// horiz end of window.
		sbus->write_verify(0x34,0x28); 	// vert start of window 320x208
		sbus->write_verify(0x35,0x0); 	// vert start of window.
		sbus->write_verify(0x36,0xC8); 	// vert end of window.
		sbus->write_verify(0x37,0x1); 	// vert end of window.

	} else {

		fmt->width = 208;
		fmt->height = 208;

		if (PHOTO_COUNTER > 0) {
			// needed in case we have been in VGA mode
			init_sanyo();
			PHOTO_COUNTER--;
		}

		//sbus->write_verify(0x13,0x12); // frame rate.
		//sbus->write_verify(0x14,0x3);  // frame rate. // 20 fps
		// can only use if memory mode is off
		//sbus->write_verify(0x93,0x14); // frame rate. 

		sbus->write_verify(0x30,0x17);
		sbus->write_verify(0x31,0x1);
		sbus->write_verify(0x32,0xB7);
		sbus->write_verify(0x33,0x2);
		sbus->write_verify(0x34,0x28);  /// 208 by 208
		sbus->write_verify(0x35,0x0);
		sbus->write_verify(0x36,0xC8);
		sbus->write_verify(0x37,0x1);
		//////	SANYO_OneTimeSetup();
	}

	return 0;
} //// End of function


static int detect_sanyo(void)
{
	struct camera_serial_bus * sbus;

	this = &camera_sanyo;
	sbus = this->camif->sbus;
	
	// Just an arbitrary camera command,
	// which we know exists on the Sanyo
	// device but not on the other.
	sbus->set_devid(CAMERA_SANYO_DEV_ID);

	return sbus->write_verify(0xB8, 0xC2);
}

static int
sanyo_querymenu(struct v4l2_querymenu *qm)
{
	return -EINVAL;
}

static int
sanyo_query_control(struct v4l2_queryctrl *qc)
{
	return -EINVAL;
}

static int
sanyo_set_control(struct v4l2_control *vc)
{
	return -EINVAL;
}

static int
sanyo_get_control(struct v4l2_control *vc)
{
	return -EINVAL;
}

struct camera camera_sanyo = {
	detect:  detect_sanyo,
	init:    init_sanyo,
	cleanup: cleanup_sanyo,

	open:    open_sanyo,
	close:   close_sanyo,
	set_format:   setup_sanyo,
	set_frame_period: sanyo_set_fp,
	get_frame_period: sanyo_get_fp,
	
	convert_image: convert_sanyo_image,

	query_control: sanyo_query_control,
	get_control:   sanyo_get_control,
	set_control:   sanyo_set_control,
	query_menu:    sanyo_querymenu,
};
