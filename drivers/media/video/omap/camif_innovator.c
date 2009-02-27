/*
 * camif_innovator.c
 *
 * Implementation of Camera Interface for OMAP1510 Innovator platform.
 *
 * Copyright (C) 2003-2004 MontaVista Software, Inc.
 *
 * Author: MontaVista Software, Inc.
 *              stevel@mvista.com or source@mvista.com
 *
 *  Modifications:
 *
 *  Oct 2003: MontaVista Software Inc. source@mvista.com
 *  Added OMAP1610 support
 *
 *  May 2004: MontaVista Software Inc. stevel@mvista.com
 *    - stripped out old NO_DMA support.
 *    - added support for OV9640 camera detection.
 *    - some minor fixes to the camera API.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/videodev.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/dma.h>
#ifdef CONFIG_ARCH_OMAP730
#include <asm/arch/omap730_config.h>
#endif
#define MODULE_NAME "camif"
#include "common.h"
#include "camif.h"

static struct camera_interface * this;

static void (*capture_callback)(void *);
static void* callback_data;
static u8* capture_buffer;

#undef MEASURE_FR

// DMA global control register
static volatile u16 *        dma_gcr = (u16 *)(OMAP_DMA_BASE + 0x400);
static volatile dma_regs_t * camera_dma_regs = NULL;

static volatile camera_regs_t * camera_regs = (camera_regs_t *)CAMERA_BASE;

static int snapshot_active = 0;
static int streaming_active = 0;
static int camera_module_present = 0;
static int current_exclk = 24*10;  // MHz * 10

static spinlock_t camif_lock;
static wait_queue_head_t vsync_wait;

#define FIFO_TRIGGER_LVL 32
#define DMA_DEST_AMODE  AMODE_POST_INC
#define DMA_FRAME_INDEX 0
#define DMA_ELEM_INDEX  0
#define DMA_ELEM_SIZE   4
#define DMA_DEST_PORT   PORT_EMIFF

static inline u32 get_imgsize_bits(void)
{
	if (this->camera) {
		switch (this->camera->imgfmt) {
		case VGA:
			return IMGSIZE_VGA;
		case CIF:
			return IMGSIZE_CIF;
		case QVGA:
			return IMGSIZE_QVGA;
		default:
			return IMGSIZE_QCIF;
		}
	}

	return IMGSIZE_QCIF;
}

static inline void camif_mode_set(u32 mask)
{
	u32 itstat = camera_regs->it_status ;
	itstat = 0;
	camera_regs->mode |= mask;
}

static inline void camif_mode_clear(u32 mask)
{
	u32 itstat = camera_regs->it_status ;
	itstat = 0;
	camera_regs->mode &= ~mask;
}

static inline void camif_cleanfifo(void)
{
	ENTRY();
	camera_regs->ctrlclock &= ~LCLK_EN;
	camif_mode_set(RAZ_FIFO);
	udelay(10);
	camif_mode_clear(RAZ_FIFO);
	camera_regs->ctrlclock |= LCLK_EN;
	EXIT();
}


static inline u32 mhzx10_to_foscmod(int mhzx10)
{
	switch (mhzx10) {
	case 60:
		return FOSCMOD_6MHz;
	case 80:
		return FOSCMOD_8MHz;
	case 96:
		return FOSCMOD_9_6MHz;
	case 120:
		return FOSCMOD_12MHz;
	case 240:
	default:
		/* gotta use DPLL source for 24 MHz */
		return FOSCMOD_24MHz | DPLL_EN;
	}
}

static inline void camif_start(int exclk)
{
	u32 clkbits = mhzx10_to_foscmod(exclk);
	
	// take camera interface out of reset
	camera_regs->gpio = CAM_RST;

	// reset mode
	camera_regs->mode = (get_imgsize_bits() |
			     (FIFO_TRIGGER_LVL << THRESHOLD_BIT));
	
	// start the camera interface clocks
	camera_regs->ctrlclock =
		(MCLK_EN | LCLK_EN | CAMEXCLK_EN | clkbits);

	mdelay(10);

	current_exclk = exclk;
}


static inline void camif_stop(void)
{
	camera_regs->ctrlclock = 0; // stop clocks
	camera_regs->gpio = 0;	    // put camera interface in reset
	mdelay(10);
}

#ifdef MEASURE_FR
extern unsigned long do_getmachinecycles(void);
extern unsigned long machinecycles_to_usecs(unsigned long mputicks);
static unsigned long dmac_sum;
static unsigned long dmac_delta;
static unsigned long dmac_N;
#endif

static void dma_callback(void *client_data)
{
	unsigned long flags;
	
	spin_lock_irqsave(&camif_lock, flags);
	snapshot_active = 0;
	if (streaming_active) {
		camif_cleanfifo();
		camera_dma_regs->ccr |= DCCR_EN; // restart DMA
	} else {
		camif_mode_clear(EN_DMA | EN_FIFO_FULL);
	}
	spin_unlock_irqrestore(&camif_lock, flags);

#ifdef MEASURE_FR
	if (dmac_delta) {
		dmac_sum += machinecycles_to_usecs(do_getmachinecycles()
						   - dmac_delta);
		dmac_N++;
	}
	dmac_delta = do_getmachinecycles();
#endif
	
	// callback to V4L2 layer
	capture_callback(callback_data);

	EXIT();
}

// SystemDMA init
static int dma_init(void)
{
	ENTRY();

	*dma_gcr = 0x0004;  // dma free running

	EXIT();
	return 0;
}

static void dma_start(void)
{
	dma_addr_t physbuf;
	int width = omap_image_size[this->camera->imgfmt].width;
	int height = omap_image_size[this->camera->imgfmt].height;
	int Bpp = (omap_pixfmt_depth[this->camera->pixfmt] + 7) >> 3;
	
	ENTRY();

	if (!this->camera)
		return;
	camera_dma_regs->csdp =
		DATA_TYPE_S32 |
		(PORT_TIPB << DCSDP_SRC_PORT_BIT) |
		(DMA_DEST_PORT << DCSDP_DEST_PORT_BIT);
	//dbg("%04x --> CSDP\n", camera_dma_regs->csdp);
	
	/*
	 * sync on camera fifo, no autoinit, high pri,
	 * frame sync, src const, dst post inc
	 */
	camera_dma_regs->ccr =
		((DMA_DEST_AMODE << DCCR_DST_AMODE_BIT) |
		 (AMODE_CONST << DCCR_SRC_AMODE_BIT) |
		 DCCR_PRIO | DCCR_FS | eCameraRx);
	if (streaming_active)
		camera_dma_regs->ccr |= (DCCR_AI | DCCR_REPEAT);
	//dbg("%04x --> CCR\n", camera_dma_regs->ccr);
	
	camera_dma_regs->cicr   = 0x0020; // enable block interrupt
	
	camera_dma_regs->cssa_l = (CAM_CAMDATA_REG & 0xffff);
	//dbg("%04x --> CSSA_L\n", camera_dma_regs->cssa_l);
	camera_dma_regs->cssa_u = ((CAM_CAMDATA_REG>>16) & 0xffff);
	//dbg("%04x --> CSSA_U\n", camera_dma_regs->cssa_u);

	/*
	 * elements per frame must be FIFO trigger level when
	 * using frame sync
	 */
	camera_dma_regs->cen = FIFO_TRIGGER_LVL;
	// TODO: make sure this divides to a whole number.
	camera_dma_regs->cfn =
		(width * height * Bpp) / (DMA_ELEM_SIZE * FIFO_TRIGGER_LVL);

#ifdef CONFIG_ARCH_OMAP1510
	camera_dma_regs->cfi = DMA_FRAME_INDEX;
	camera_dma_regs->cei = DMA_ELEM_INDEX;
#else
	camera_dma_regs->csfi = DMA_FRAME_INDEX;
	camera_dma_regs->csei = DMA_ELEM_INDEX;
	camera_dma_regs->cdfi = DMA_FRAME_INDEX;
	camera_dma_regs->cdei = DMA_ELEM_INDEX;
#endif

	physbuf = (dma_addr_t)virt_to_phys(capture_buffer);

#if 1
	//dbg("regs->csr = 0x%08x\n", camera_dma_regs->csr);
	camera_dma_regs->cdsa_l = physbuf & 0xffff;
	camera_dma_regs->cdsa_u = (physbuf >> 16) & 0xffff;
	//dbg("%04x --> CDSA_L\n", camera_dma_regs->cdsa_l);
	//dbg("%04x --> CDSA_U\n", camera_dma_regs->cdsa_u);
	camera_dma_regs->ccr |= DCCR_EN;	// start transfer;
#else
	omap_start_dma((dma_regs_t *)camera_dma_regs, physbuf, 0);
#endif

	EXIT();
}


static void camera_interrupt(int irq, void *client_data,
			     struct pt_regs *regs)
{
	u32 itstat;

	ENTRY();
	
	spin_lock(&camif_lock);

	itstat = camera_regs->it_status;
	
	if (itstat & V_UP) {
		if (snapshot_active || streaming_active) {
			camif_cleanfifo();
			camif_mode_clear(EN_V_UP);
			camif_mode_set(EN_DMA | EN_FIFO_FULL);
			dma_start();
		}

		wake_up_interruptible(&vsync_wait);
	}
	
	if (itstat & V_DOWN) {
		wake_up_interruptible(&vsync_wait);
	}

	if (itstat & H_UP) {
		dbg("H_UP set\n");
	}
	if (itstat & H_DOWN) {
		dbg("H_DOWN set\n");
	}
	if (itstat & FIFO_FULL) {
		camif_cleanfifo();
		dbg("FIFO_FULL set\n");
	}

	spin_unlock(&camif_lock);
	EXIT();
}

static void camif_wait_for_vsync_edge(u32 edge_mask)
{
	ENTRY();

	camif_mode_set(edge_mask);

	// wait for VSYNC edge
	do {
		interruptible_sleep_on(&vsync_wait);
	} while (signal_pending(current));

	camif_mode_clear(edge_mask);
	camif_cleanfifo();
	EXIT();
}

static int camif_snapshot(u8* buf, int size)
{
	unsigned long flags;

	spin_lock_irqsave(&camif_lock, flags);

	if (!this->camera) {
		spin_unlock_irqrestore(&camif_lock, flags);
		return -ENODEV;
	}
	if (snapshot_active) {
		dbg("already active!\n");
		spin_unlock_irqrestore(&camif_lock, flags);
		return 0;
	}
	if (streaming_active) {
		dbg("streaming is active!\n");
		spin_unlock_irqrestore(&camif_lock, flags);
		return -EINVAL;
	}
	
	capture_buffer = buf;

	snapshot_active = 1;
	camif_mode_set(EN_V_UP);

	//dbg("mode = 0x%08x\n", camera_regs->mode);

	spin_unlock_irqrestore(&camif_lock, flags);
	return 0;
}

static int camif_start_streaming(u8* buf, int size)
{
	unsigned long flags;

	spin_lock_irqsave(&camif_lock, flags);

	if (!this->camera) {
		spin_unlock_irqrestore(&camif_lock, flags);
		return -ENODEV;
	}
	if (streaming_active) {
		dbg("already active!\n");
		spin_unlock_irqrestore(&camif_lock, flags);
		return 0;
	}
	if (snapshot_active) {
		dbg("snapshot is active!\n");
		spin_unlock_irqrestore(&camif_lock, flags);
		return -EINVAL;
	}
	
	capture_buffer = buf;
	
	streaming_active = 1;
	camif_mode_set(EN_V_UP);
	
	spin_unlock_irqrestore(&camif_lock, flags);
	return 0;
}

static int camif_abort(void)
{
	unsigned long flags;

	spin_lock_irqsave(&camif_lock, flags);

	camif_mode_clear(EN_V_UP | EN_DMA | EN_V_DOWN |
			 EN_NIRQ | EN_FIFO_FULL);

	// terminate any in-progress DMA.
	if (camera_dma_regs)
		omap_reset_dma((dma_regs_t *)camera_dma_regs);

	snapshot_active = streaming_active = 0;
	spin_unlock_irqrestore(&camif_lock, flags);
	return 0;
}


static int camif_set_fp(int fp)
{
	int ret, exclk_mhzx10;

	if (!this->camera)
		return -ENODEV;

	if (fp == this->camera->get_frame_period())
		return fp;
	
	exclk_mhzx10 = fp < 500000 ? 24*10 : 12*10;
	
	// first test this frame rate with camera
	ret = this->camera->set_frame_period(fp, exclk_mhzx10 * 100000, 1);

	if (ret >= 0) {
		// frame rate passed tests, apply it.
		camif_abort();
		if (exclk_mhzx10 != current_exclk) {
			this->camera->close();
			camif_stop();
			camif_start(exclk_mhzx10);
			if ((ret = this->camera->open()) < 0) {
				err("error from camera open\n");
				return ret;
			}
		}

		ret = this->camera->set_frame_period(fp,
						     exclk_mhzx10 * 100000,
						     0);

		/*
		 * wait a few frames for camera to stabilize
		 */
		camif_wait_for_vsync_edge(EN_V_DOWN);
		camif_wait_for_vsync_edge(EN_V_DOWN);
		camif_wait_for_vsync_edge(EN_V_DOWN);
	}
	
	return ret;
}


static void CameraFPGADisable(void)
{
#ifdef CONFIG_ARCH_OMAP1510
	// turn off power to camera module
	outb((inb(INNOVATOR_FPGA_CAM_USB_CONTROL) | 0x04),
	     INNOVATOR_FPGA_CAM_USB_CONTROL);
#endif
}

static void CameraFPGAEnable(void)
{
#ifdef CONFIG_ARCH_OMAP1510
	CameraFPGADisable();
	udelay(100);

	// turn on power to camera module
	outb((inb(INNOVATOR_FPGA_CAM_USB_CONTROL) & ~0x04),
	     INNOVATOR_FPGA_CAM_USB_CONTROL);
	udelay(100);

	// reset the camera module
        outb(inb(OMAP1510P1_FPGA_RST) & ~(1<<4), OMAP1510P1_FPGA_RST);
        udelay(100);
        outb(inb(OMAP1510P1_FPGA_RST) | (1<<4), OMAP1510P1_FPGA_RST);
        udelay(100);
#endif
}


static int camif_open(void)
{
	int ret;
	
	if (!this->camera)
		return 0;
	
	// Camera interrupt
	if ((ret = request_irq(INT_CAMERA, camera_interrupt, SA_INTERRUPT,
			       "camera", NULL))) {
		err("Failed to acquire camera interrupt\n");
		return ret;
	}
	
	if ((ret = omap_request_dma(eCameraRx, "camera dma", dma_callback,
				    NULL, (dma_regs_t **)&camera_dma_regs))) {
		err("No DMA available for camera\n");
		free_irq(INT_CAMERA, NULL);
		camera_dma_regs = NULL;
		return ret;
	}

	//dbg("Camera DMA at %p\n", camera_dma_regs);
	omap_dma_setup(eCameraRx, eDmaIn);

	CameraFPGAEnable();
	camif_start(current_exclk);

	if ((ret = this->camera->open())) {
		err("Error from Camera open\n");
		camif_stop();
		CameraFPGADisable();
		free_irq(INT_CAMERA, NULL);
		omap_free_dma((dma_regs_t *)camera_dma_regs);
		camera_dma_regs = NULL;
		return ret;
	}
		
#ifdef MEASURE_FR
	dmac_sum = dmac_delta = dmac_N = 0;
#endif
	/*
	 * wait a few frames for camera to stabilize
	 */
	camif_wait_for_vsync_edge(EN_V_DOWN);
	camif_wait_for_vsync_edge(EN_V_DOWN);
	camif_wait_for_vsync_edge(EN_V_DOWN);

	return ret;
}

static int camif_close(void)
{
	camif_abort();
	if (this->camera)
		this->camera->close();
	camif_stop();
	CameraFPGADisable();
	
	free_irq(INT_CAMERA, NULL);

	omap_free_dma((dma_regs_t *)camera_dma_regs);
	camera_dma_regs = NULL;

#ifdef MEASURE_FR
	dbg("%lu frames in %lu usecs\n", dmac_N, dmac_sum);
#endif
	
	return 0;
}

static int camif_init(void (*callback)(void *), void* data)
{
#ifdef CONFIG_ARCH_OMAP1510
	u8 camera_pos;
#endif

	this = &camif_innovator;

	capture_callback = callback;
	callback_data = data;
	
#ifdef CONFIG_ARCH_OMAP1510
	outl(inl(FUNC_MUX_CTRL_0) & ~(1<<28), FUNC_MUX_CTRL_0);
	outl(inl(FUNC_MUX_CTRL_4) & ~(0x1ff<<21), FUNC_MUX_CTRL_4);
	outl(0, FUNC_MUX_CTRL_5);
#endif

#ifdef CONFIG_ARCH_OMAP1610
	/* set pin mux control for camera functionality */
	outl(inl(FUNC_MUX_CTRL_4) & ~(0x1ff<<21), FUNC_MUX_CTRL_4);
	outl(0, FUNC_MUX_CTRL_5);
	outl((inl(FUNC_MUX_CTRL_A) & ~0x7) | 0x6, FUNC_MUX_CTRL_A);

	/* Enable internal pull-down resistors on Camera interface pins */
	outl(inl(PULL_DWN_CTRL_0) & ~(0x1FFF<<17), PULL_DWN_CTRL_0);
	outl(inl(PU_PD_SEL_0) & ~(0x1FFF<<17), PU_PD_SEL_0);

	/* Enable internal pull-down on BCLK */
	outl(inl(PULL_DWN_CTRL_2) & ~(0x1<<6), PULL_DWN_CTRL_2);
	outl(inl(PU_PD_SEL_2) & ~(0x1<<6), PU_PD_SEL_2);

	/* activate above settings */
	outl(0xeaef, COMP_MODE_CTRL_0);

	/* reset camera interface */
	outl(inl(RESET_CONTROL) & ~(1<<5), RESET_CONTROL);
	mdelay(1);
	outl(inl(RESET_CONTROL) | (1<<5), RESET_CONTROL);
	mdelay(1);
#endif

#ifdef CONFIG_ARCH_OMAP730
	/* pin mux setup is done in board-specific configuration file, 
	 * e.g. perseus2.c.
	 */

	/* DPLL clock request */
	*((volatile u16 *)SOFT_REQ_REG) |= SOFT_CAM_DPLL_MCK0_REQ;
	*((volatile u16 *)SOFT_DISABLE_REQ_REG) &= ~((u16)DIS_CAM_DPLL_MCLK_REQ);
	
	/* DPLL clock enable request */
	OMAP730_CONFIGURE(PCC_CAM_CLK_REQ, ACTIVE);

	/* Enable the camera clock */
	*((volatile u16 *)(CAM_CLK_CTRL)) = SYSTEM_CLK_EN | CAM_CLOCK_EN;
	mdelay(1);
#endif

	//enable peripheral reset
	*ARM_RSTCT2 |= (1<<EN_PER);
	mdelay(1);
	
	//enable peripheral clock
	*ARM_IDLECT2 |= (1<<EN_XORPCK); 
	mdelay(1);

	spin_lock_init(&camif_lock);
	init_waitqueue_head(&vsync_wait);

	dma_init();

#ifdef CONFIG_ARCH_OMAP1510
	camera_module_present = 0;
	camera_pos = inb(INNOVATOR_FPGA_CAM_USB_CONTROL) & 0x03;
	if (camera_pos == 0x03) {
		info("camera module not present\n");
		// this is not an error
		return 0;
	}
	info("camera module detected, facing %s\n",
	     camera_pos == 2 ? "forward" : "backward");
#endif
	camera_module_present = 1;
	
	return 0;
}


static void camif_cleanup(void) 
{
	info("Unloading\n");
	CameraFPGADisable();
	if (this->camera)
		this->camera->cleanup();
	if (this->sbus) {
		this->sbus->cleanup();
	}
}


// **************************
// Routine: // Description:
// **************************
static struct camera * camif_camera_detect(void)
{
	struct camera * cam = NULL;
	
	ENTRY();

	this->camera = NULL;

	if (!camera_module_present)
		return NULL;
	
	CameraFPGAEnable();
	camif_start(current_exclk);

	// first, select serial bus module
#ifdef CONFIG_ARCH_OMAP1510
	this->sbus = &camera_sbus_sccb;
#else
	this->sbus = &camera_sbus_new_i2c; //I2C in normal mode
#endif
	// and init the camera serial bus
	if (this->sbus->init()) {
		err("error initializing SCCB\n");
		goto cam_detect_exit;
	}

#if defined(CONFIG_OMAP_H2) || defined (CONFIG_MACH_OMAP_PERSEUS2)
	// -------
	// Try and detect if an OmniVision 9640 camera is out there.
	// -------
	cam = &camera_ov9640;
	cam->camif = this;
	
	if (cam->detect() == 0) {
		info("OV9640 camera detected\n");
		this->camera = cam;
		this->camera->init();
		goto cam_detect_exit;
	}
#endif
	
	// -------
	// Try and detect if an OmniVision 6x30 camera is out there.
	// -------
	cam = &camera_ov6x30;
	cam->camif = this;
	
	if (cam->detect() == 0) {
		info("OV6X30 camera detected\n");
		this->camera = cam;
		this->camera->init();
		goto cam_detect_exit;
	}

	this->sbus->cleanup();
	cam = NULL;

 cam_detect_exit:
	camif_stop();
	CameraFPGADisable();
	return cam;
}


static void* camif_alloc_image_buffer(int size)
{
	return (void*)__get_dma_pages(GFP_KERNEL, get_order(size));
}

static void camif_free_image_buffer(void* buffer, int size)
{
	free_pages((unsigned long)buffer, get_order(size));
}


struct camera_interface camif_innovator = {
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
	set_frame_period:   camif_set_fp,
};

