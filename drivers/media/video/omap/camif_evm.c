/*
 * File: camif_evm.c
 *
 * Description:
 *   Implementation of Camera Interface for OMAP1510 EVM platform.
 *                            f
 *   This driver was highly leveraged from Bill Dirk's
 *   v4l2 example generic driver. Modified by RidgeRun
 *   to work with the omap1509 camera module.
 *
 * Author: RidgeRun, Inc <skranz@ridgerun.com>
 * Created 2002, Copyright (C) 2002 RidgeRun, Inc.  All rights reserved.
 * Created 2002, Copyright (C) 2002 Texas Instruments  All rights reserved.
 * Copyright (C) 2003 MontaVista Software, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * Original Author: Bill Dirks <bdirks@pacbell.net>
 *                  based on code by Alan Cox, <alan@cymru.net>
 */

#include <linux/config.h> /* retrieve the CONFIG_* macros */
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/videodev.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/dma.h>
#define MODULE_NAME "camif"
#include "common.h"
#include "camif.h"

static struct camera_interface * this;

static void (*capture_callback)(void *);
static void* callback_data;
static u8* capture_buffer;
static int capture_size;

static void CAM_cleanfifo(void);
static void CAM_intr(int irq, void *client_data, struct pt_regs *regs);
static void DMA_intr(void *client_data);

static int snapshot_active = 0;
static int streaming_active = 0;

#define CAM_MODE     ((volatile unsigned int *)0xFFFB6808)
#define CAM_GPIO_RST ((volatile unsigned int *)0xFFFB6814)

static dma_regs_t *dma_regs = NULL;


static int camif_abort(void)
{
	// Disable cam_intr() interrupts.
	*CAM_MODE &= ~0x301f0;
	
	omap_reset_dma(dma_regs); // terminate any in-progress DMA.
	
	// *revisit-skranz* in the interest of power
	// management we should also put logic here
	// that uses I2C to tell the camera Module
	// to stop processing images -- or maybe assert
	// and hold indefinitely the camera module's
	// reset line. I believe the "*CAM_GPIO_RST=0x0;"
	// would accomplish that.
	snapshot_active = streaming_active = 0;
	return 0;
}

static int camif_snapshot(u8* buf, int size)
{
        ENTRY();

	capture_buffer = buf;
	capture_size = size;
	
	if (snapshot_active || streaming_active)
                camif_abort();

	// Enable cam_intr() interrupts.
	
	*CAM_MODE |= 0x0110;  // Set DMA Mode and Unmask EN_V_UP
//          *CAM_MODE |= 0x00020110;  // Set DMA Mode and Unmask EN_V_UP

	*CAM_GPIO_RST |= 0x01;  // Take Camera interface out of reset

        snapshot_active = 1;
	return 0;
}

static int camif_start_streaming(u8* buf, int size)
{
	ENTRY();
	
	capture_buffer = buf;
	capture_size = size;
	
	if (snapshot_active || streaming_active)
		camif_abort();
	
	// Enable cam_intr() interrupts.
	
	*CAM_MODE |= 0x0110;  // Set DMA Mode and Unmask EN_V_UP
//          *CAM_MODE |= 0x00020110;  // Set DMA Mode and Unmask EN_V_UP
	
	*CAM_GPIO_RST |= 0x01;  // Take Camera interface out of reset
	
	streaming_active = 1;
	return 0;
}



/*
 *
 *      V I D E O   F O R   L I N U X   I N T E R F A C I N G
 *
 */

// **************************
// Routine:
// Description:
// **************************
static int camif_open(void)
{
	int status;

	if (!this->camera)
		return 0;

	// Request and enable Camera interrupt
	status = request_irq(INT_CAMERA, &CAM_intr,
			     SA_INTERRUPT, "camera", NULL);
	if (status < 0) {
		err("No IRQ available for camera");
		return status;
	}

	if (!dma_regs) {
		if (omap_request_dma(eCameraRx, "camera", DMA_intr,
				     NULL, &dma_regs)) {
			err("No DMA available for camera");
			return -ENODEV;
		}
		omap_dma_setup(eCameraRx, eDmaIn);
		
		dma_regs->csdp   = 0x000E;
		// sync camera fifo, no autoinit, high pri,
		dma_regs->ccr    = 0x4054;
		// frame sync, src const, dst post inc

		dma_regs->cicr   = 0x0020;   	// enable block interrupt
		dma_regs->cssa_l = 0x6810;   	// low; camif data reg
		dma_regs->cssa_u = 0xFFFB;   	// hi;  camif data reg

//      dma_regs->cfn = 1;
//      dma_regs->cen = ((Camera_Data_Size) / 4);
//      dma_regs->cen = 1;
//      dma_regs->cfn = (((Camera_Data_Size) / 4) / dma_regs->cen);
//      dma_regs->cen = 2;
		//dma_regs->cen = 32;
		//dma_regs->cfn = (((Camera_Data_Size) / 4) / dma_regs->cen);

	}

	return this->camera->open();
}

static int camif_close(void)
{
	camif_abort();
	if (this->camera)
		this->camera->close();
	free_irq(INT_CAMERA, NULL);
	omap_free_dma((dma_regs_t *)dma_regs);
	return 0;
}

// ------------------hw----------------------- //

#define CAM_BASE              ((volatile unsigned int *)0xFFFB6800)
#define CAM_CTRLCLOCK         ((volatile unsigned int *)0xFFFB6800)
#define CAM_IT_STATUS         ((volatile unsigned int *)0xFFFB6804)
#define CAM_STATUS            ((volatile unsigned int *)0xFFFB680C)

static void dump_dma_regs(void)
{
	u16 *i;

	for (i=(u16 *)dma_regs; i < (((u16 *)dma_regs) + 13); i++)
		dbg("dma_reg:  0x%08x = 0x%04x\n", (unsigned int)i, (u16)(*i));
}

static void dump_camera_regs(void)
{
	u32 *i;

	for (i=(u32 *)CAM_BASE; i < (((u32 *)CAM_BASE) + 7); i++)
		dbg("cam_reg:  0x%08x = 0x%08x\n", (unsigned int)i, (u32)(*i));
}

static inline void CAM_cleanfifo(void)
{
#define CAMDATA ((volatile unsigned int *)0xFFFB6810)
#define CAM_PEAK_COUNTER ((volatile unsigned int *)0xFFFB6818)

	int i;
	volatile unsigned int val;

	for (i=0;i<128;i++)
		val = *CAMDATA;
	*CAM_PEAK_COUNTER = 0;
//   *CAM_MODE |= (1<<18);
//   *CAM_MODE &=~(1<<18);
}

// **************************
// Routine:
// Description:
// **************************
static void CAM_intr(int irq, void *client_data, struct pt_regs *regs)
{
	unsigned int status;

	// This is a clear-on-read register so we'll read it only
	// once and perform all tests on the single cached value.
	status = *CAM_IT_STATUS;
  
	//DBG("Status= 0x%08x\n", status);
//  dump_camera_regs();

	if (DATA_XFER & status) {
		//DBG("DATA_XFER\n");
	}
	if (FIFO_FULL & status) {
		//DBG("FIFO_FULL\n");
//    CAM_cleanfifo();
//    status = *CAM_IT_STATUS;
	}
	if (H_DOWN & status) {
		//DBG("H_DOWN\n");
	}
	if (H_UP & status) {
		//DBG("H_UP\n");
	}
	if (V_DOWN & status) {
		//DBG("V_DOWN\n");
	}
	if (V_UP & status) {
		//DBG("V_UP\n");
//    dump_dma_regs();

		if (snapshot_active || streaming_active) {
			u32* phyBuff =
				(u32 *)virt_to_phys((void*)capture_buffer);
			//DBG("StartCamera");
 
			CAM_cleanfifo();
			status = *CAM_IT_STATUS;
			
			// start a new capture.
			omap_start_dma(dma_regs,
				       (dma_addr_t)phyBuff,
				       (u_int)capture_size);
			
			// Note: At this point the pump is furthur primed
			//       for picking up an image. Processing will
			//       continue at DMA_intr() interrupt.
		}
	}
}

// **************************
// Routine:
// Description:
// **************************
static void DMA_intr(void *client_data)
{
	// Note: This represents the last step
	// in capturing a raw image from the h/w.

	snapshot_active = 0;
	// callback to V4L2 layer
	capture_callback(callback_data);
}


static int camif_init(void (*callback)(void *), void* data)
{
	ENTRY();

	this = &camif_evm;

	capture_callback = callback;
	callback_data = data;
	
//  *CAM_GPIO_RST = 0x00; // hold reset.
	*CAM_CTRLCLOCK = 0xA0;  // LCLK_EN | MCLK_EN
//  *CAM_MODE = 0x0200;     // THRESHOLD = 1
//  *CAM_MODE = 0x0400;     // THRESHOLD = 2
// 	*CAM_MODE = 0x0800;     // THRESHOLD = 4
// 	*CAM_MODE = 0x1000;     // THRESHOLD = 8
//  *CAM_MODE = 0x2000;     // THRESHOLD = 16
//  *CAM_MODE = 0x4000;     // THRESHOLD = 32
//  *CAM_MODE = 0x8000;     // THRESHOLD = 64
	//DBG("Threshold = 4\n");
	CAM_cleanfifo();
//  *CAM_CTRLCLOCK |= 0x0010;  // Enable external clock

	*CAM_MODE |= 0x8; 		// turn on byte swap in cam intf fifo.
	// so that we can avoid doing it in SW
	// This probably is off by default and we don't need to
	// explicitly turn it of for omni as swap is not needed there.

//  *CAM_GPIO_RST = 0x00;
//  *CAM_GPIO_RST = 0x01;

	return 0;
}

static void camif_cleanup(void) 
{
	struct camera_serial_bus * sbus = this->sbus;
	info("Unloading\n");
	if (this->camera)
		this->camera->cleanup();
	sbus->cleanup();
}

// **************************
// Routine: // Description:
// **************************
static struct camera * camif_camera_detect(void)
{
	struct camera * cam;

	ENTRY();

	this->camera = NULL;

	// first, set serial bus to I2C
	this->sbus = &camera_sbus_old_i2c;
	// and init the bus
	if (this->sbus->init()) {
		err("error initializing 1509-mode I2C bus\n");
		return NULL;
	}

	// -------
	// try and detect if a Sanyo camera is out there.
	// -------
	cam = &camera_sanyo;
	cam->camif = this;

	if (cam->detect() == 0) {
		this->camera = cam;
		this->camera->init();
		return cam;
	}
	
	// -------
	// Try and detect if an OmniVision camera is out there.
	// -------
	cam = &camera_ov6x30;
	cam->camif = this;
	
	if (cam->detect() == 0) {
		this->camera = cam;
		this->camera->init();
		return cam;
	}

	return NULL; // camera not present.
}

static void* camif_alloc_image_buffer(int size)
{
	return (void*)__get_dma_pages(GFP_KERNEL, get_order(size));
}

static void camif_free_image_buffer(void* buffer, int size)
{
	free_pages((unsigned long)buffer, get_order(size));
}


struct camera_interface camif_evm = {
	camera_detect:      camif_camera_detect,
	alloc_image_buffer: camif_alloc_image_buffer,
	free_image_buffer:  camif_free_image_buffer,
	init:               camif_init,
	cleanup:            camif_cleanup,
	open:               camif_open,
	close:              camif_close,
	snapshot:           camif_snapshot,
	start_streaming:    camif_start_streaming,
	abort:              camif_abort,
};
