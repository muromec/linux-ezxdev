/* 
    ov9640.c - Omnivision 9640 CMOS sensor driver 

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
#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/pagemap.h>
#include <linux/wrapper.h>
#include <linux/videodev.h>

#include <asm/pgtable.h>
#include <asm/dma.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>

#include "camera.h"
#include "ov9640.h"
#include "ov9640_hw.h"


#define MAX_WIDTH	1280
#define MAX_HEIGHT	960
#define MIN_WIDTH	88
#define MIN_HEIGHT	72

#define MAX_BPP		16
#define WIDTH_DEFT	320
#define HEIGHT_DEFT	240
#define FRAMERATE_DEFT	0xf

/*
 * It is required to have at least 3 frames in buffer
 * in current implementation
 */
#define FRAMES_IN_BUFFER	3
#define MIN_FRAMES_IN_BUFFER	3
#define MAX_FRAME_SIZE		(MAX_WIDTH * MAX_HEIGHT * (MAX_BPP >> 3))
#define BUF_SIZE_DEFT		(MAX_FRAME_SIZE )
#define SINGLE_DESC_TRANS_MAX  	PAGE_SIZE
#define MAX_DESC_NUM		((MAX_FRAME_SIZE / SINGLE_DESC_TRANS_MAX + 1) *\
				 MIN_FRAMES_IN_BUFFER)

#define MAX_BLOCK_NUM	20
extern ov9640 g_ov;

camera_function_t ov9640_func = {
	.init = camera_func_ov9640_init,
	.deinit = camera_func_ov9640_deinit,
	.set_capture_format = camera_func_ov9640_set_capture_format,
	.start_capture = camera_func_ov9640_start_capture,
	.stop_capture = camera_func_ov9640_stop_capture,
	.command = camera_func_ov9640_command,
	.pm_management = camera_func_ov9640_pm,
};

/***********************************************************************
 *
 * OV9640 Functions
 *
 ***********************************************************************/
static void ov9640_gpio_init()
{

    set_GPIO_mode(GPIO_CIF_DD0_MD);     /* CIF_DD[0] */
    set_GPIO_mode(GPIO_CIF_DD1_MD);     /* CIF_DD[1] */
    set_GPIO_mode(GPIO_CIF_DD2_MD);     /* CIF_DD[2] */
    set_GPIO_mode(GPIO_CIF_DD3_MD);     /* CIF_DD[3] */
    set_GPIO_mode(GPIO_CIF_DD4_MD);     /* CIF_DD[4] */
    set_GPIO_mode(GPIO_CIF_DD5_MD);     /* CIF_DD[5] */
    set_GPIO_mode(GPIO_CIF_DD6_MD);     /* CIF_DD[6] */
    set_GPIO_mode(GPIO_CIF_DD7_MD);     /* CIF_DD[7] */
    set_GPIO_mode(GPIO_CIF_MCLK_MD);    /* CIF_MCLK */
    set_GPIO_mode(GPIO_CIF_PCLK_MD);    /* CIF_PCLK */
    set_GPIO_mode(GPIO_CIF_LV_MD);      /* CIF_LV */
    set_GPIO_mode(GPIO_CIF_FV_MD);      /* CIF_FV */
    set_GPIO_mode(GPIO_CAM_EN_MD);      /*CIF_PD */
#ifdef GPIO_CAM_RST
    set_GPIO_mode(GPIO_CAM_RST_MD);     /*CIF_RST */
#endif

    return;

}

void ov9640_set_clock(unsigned int clk_regs_base, int pclk_enable, int mclk_enable, unsigned int mclk_khz)
{
	unsigned int ciclk = 0, value, div, cccr_l, K;

	// determine the LCLK frequency programmed into the CCCR.
	cccr_l = (CCCR & 0x0000001F);

	if (cccr_l < 8)		// L = [2 - 7]
		ciclk = (13 * cccr_l) * 100;
	else if (cccr_l < 17)	// L = [8 - 16]
		ciclk = ((13 * cccr_l) * 100) >> 1;
	else if (cccr_l < 32)	// L = [17 - 31]
		ciclk = ((13 * cccr_l) * 100) >> 2;
	DPRINTK(KERN_WARNING "the mclk_khz = %d \n", mclk_khz);

	// want a divisor that gives us a clock rate as close to, but not more than the given mclk.
	div = (ciclk + mclk_khz) / (2 * mclk_khz) - 1;

	// write cicr4
	value = CICR4;
	value &= ~(CI_CICR4_PCLK_EN | CI_CICR4_MCLK_EN | CI_CICR4_DIV_SMASK << CI_CICR4_DIV_SHIFT);
	value |= (pclk_enable) ? CI_CICR4_PCLK_EN : 0;
	value |= (mclk_enable) ? CI_CICR4_MCLK_EN : 0;
	value |= div << CI_CICR4_DIV_SHIFT;
	CICR4 = value;
	return;
}

int camera_func_ov9640_init(p_camera_context_t cam_ctx)
{
	u8 cm_rev, cm_pid;
	u32 timeout;
	int status;
	ov9640 *pov;

	pov = &g_ov;
	memset(pov, 0, sizeof(ov9640));

	pov->pre_size = OV_SIZE_NONE;
	pov->win.width = cam_ctx->capture_width;
	pov->win.height = cam_ctx->capture_height;
	// init context status
	cam_ctx->dma_channels[0] = 0xFF;
	cam_ctx->dma_channels[1] = 0xFF;
	cam_ctx->dma_channels[2] = 0xFF;

	cam_ctx->capture_width = WIDTH_DEFT;
	cam_ctx->capture_height = HEIGHT_DEFT;

	cam_ctx->capture_input_format = CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR;
	cam_ctx->capture_output_format = CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR;

	cam_ctx->frame_rate = cam_ctx->fps = FRAMERATE_DEFT;

	cam_ctx->mini_fps = FRAMERATE_DEFT - 2;

	cam_ctx->buf_size = BUF_SIZE_DEFT;
	cam_ctx->dma_descriptors_size = MAX_DESC_NUM;
	DPRINTK(KERN_WARNING "dma_descriptors_size =%d,cam_ctx->buf_size=%d\n", cam_ctx->dma_descriptors_size,
		cam_ctx->buf_size);
	cam_ctx->vc.maxwidth = 1280;
	cam_ctx->vc.maxheight = 960;
	cam_ctx->vc.minwidth = MIN_WIDTH;
	cam_ctx->vc.minheight = MIN_HEIGHT;
	//DPRINTK( KERN_WARNING" before ov9640_gpio_init\n");
	ov9640_gpio_init();
	ci_init();
	// Configure CI according to OV9640's hardware        
	// master parallel with 8 data pins
	ci_set_mode(CI_MODE_MP, CI_DATA_WIDTH8);
	// enable pixel clock(sensor will provide pclock) and master clock = 26MHZ
	//ci_set_clock(cam_ctx->clk_reg_base, 1, 1, 26);
	ov9640_set_clock(cam_ctx->clk_reg_base, 1, 1, 2400);
	pov->mclock = 24;

	// data sample on rising and h,vsync active high
	ci_set_polarity(0, 0, 0);

	// fifo control
	// CISetFIFO(cam_ctx->ci_reg_base, 4000, XLLP_CI_FIFO_THL_32, XLLP_TRUE, XLLP_TRUE);
	ci_set_fifo(0, CI_FIFO_THL_32, 1, 1);

	// OV9640 Power on sequence
	// Take out of Power down mode, PWRDWN=1, NORMAL=0
	// Assert Reset
	// Delay
	// Remove reset
	// Delay
	ov9640_power_down(0);
	ov9640_reset();
	mdelay(1);

	// init I2C.
	status = i2c_init();
	if (status)
		return status;

	// 2 Turn on M_CLK using xx MHz and wait for 150 ms. ??
	ci_enable(1);
	mdelay(20);

	// read out version
	//timeout = 50;
	timeout = 5;
	do {
		cm_pid = cm_rev = 0;
		status = ov9640_version_revision(&cm_pid, &cm_rev);

		// Check to make sure we are working with an OV9640
		if (cm_pid == PID_OV && cm_rev == PID_9640) {
			int ver = (PID_OV << 8) | (PID_9640);
			pov->version = ver;
		}
		else if (cm_pid == PID_OV_v3 || cm_rev == PID_9640_v3) {
			int ver = (PID_OV_v3 << 8) | (PID_9640_v3);
			pov->version = ver;
		}
		else {
			ov9640_power_down(1);
			ov9640_power_down(0);
			ov9640_reset();
			mdelay(1);
		}
		printk("in fun camera_func_ov9640_init version=%x\n", pov->version);
		if (--timeout == 0)
			return -1;
	}
	while (cm_pid != PID_OV);

	cam_ctx->sensor_type = CAMERA_TYPE_OMNIVISION_9640;
	// turn sensor output off
	ov9640_viewfinder_off();

	return 0;
}

int camera_func_ov9640_deinit(p_camera_context_t cam_ctx)
{
	//init the prev_xx value.
	ov9640_set_format(OV_SIZE_NONE, OV_FORMAT_NONE);
	mdelay(20);
	// power off the external module
	ov9640_power_down(1);

	return 0;
}

int camera_func_ov9640_sleep(p_camera_context_t cam_ctx)
{
	camera_func_ov9640_deinit(cam_ctx);
}

int camera_func_ov9640_wake(p_camera_context_t cam_ctx)
{
	camera_func_ov9640_init(cam_ctx);
}

static get_ov_format(p_camera_context_t cam_ctx, u32 * size_format, u32 * color_format)
{
	u32 ovSizeFormat, ovFormat;

	// Set the current mode
	switch (cam_ctx->capture_input_format) {
	case CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR:
	case CAMERA_IMAGE_FORMAT_YCBCR422_PACKED:
		ovFormat = OV_FORMAT_YUV_422;
		printk("in get_ov_format ovFormat = OV_FORMAT_YUV_422\n");
		break;
	case CAMERA_IMAGE_FORMAT_RGB565:
		ovFormat = OV_FORMAT_RGB_565;
		break;
	default:
		printk(KERN_INFO "The Format doesn't support by OV9640 Sensor \n");
		return -1;
	}
	if (cam_ctx->capture_width == 88 && cam_ctx->capture_height == 72)
		ovSizeFormat = OV_SIZE_QQCIF;
	else if (cam_ctx->capture_width == 160 && cam_ctx->capture_height == 120)
		ovSizeFormat = OV_SIZE_QQVGA;
	else if (cam_ctx->capture_width == 176 && cam_ctx->capture_height == 144)
		ovSizeFormat = OV_SIZE_QCIF;
	else if (cam_ctx->capture_width == 320 && cam_ctx->capture_height == 240)
		ovSizeFormat = OV_SIZE_QVGA;
	else if (cam_ctx->capture_width == 352 && cam_ctx->capture_height == 288)
		ovSizeFormat = OV_SIZE_CIF;
	else if (cam_ctx->capture_width == 640 && cam_ctx->capture_height == 480)
		ovSizeFormat = OV_SIZE_VGA;
	else if (cam_ctx->capture_width == 1280 && cam_ctx->capture_height == 960)
		ovSizeFormat = OV_SIZE_SXGA;
	else {
		return -1;
	}
	if (ovSizeFormat == OV_SIZE_QVGA)
		printk("ovSizeFormat =OV_SIZE_QVGA \n");
	*size_format = ovSizeFormat;
	*color_format = ovFormat;
	return 0;
}

int camera_func_ov9640_set_capture_format(p_camera_context_t cam_ctx)
{
	CI_MP_TIMING timing;
	int status = -1;
	u32 ovSizeFormat, ovFormat;
	ov9640 *pov;

	pov = &g_ov;

	status = get_ov_format(cam_ctx, &ovSizeFormat, &ovFormat);
	if (status)
		return -1;

	status = ov9640_set_format(ovSizeFormat, ovFormat);
	if (status)
		return -1;

	// set capture width/height and timing 
	//timing.BFW = pov->win.x;
	//timing.BLW = pov->win.y;
	timing.BFW = 0;
	timing.BLW = 0;
	//ci_configure_mp(pov->win.width - 1, pov->win.height - 1, &timing);

	return status;
}

#define COPY_BUFFERS(pdes, p_page, size) \
do { \
	unsigned int len; \
	unsigned int remain_size = size; \
	while (remain_size > 0) { \
		if (remain_size > PAGE_SIZE) \
			len = PAGE_SIZE; \
		else \
			len = remain_size; \
		if (memcpy(page_address(*pdes), page_address(*p_page), len)) \
			return -EFAULT; \
		remain_size -= len; \
		pdes ++;\
		p_page++; \
	} \
} while (0);

int camera_func_ov9640_start_capture(p_camera_context_t cam_ctx, unsigned int frames)
{
	int status = -1;
	u32 ovSizeFormat, ovFormat;
	ov9640 *pov;
	u8 value;

	pov = &g_ov;

	ci_disable(1);
	mdelay(1);
	ci_enable(1);
	// clear ci fifo
	ci_reset_fifo();
	ci_clear_int_status(0xFFFFFFFF);

	camera_set_int_mask(cam_ctx, 0x3ff | 0x0400);
	// turn auto function on only doing continues capture
	if (frames == 0) {
//              CISR |= (1 << 3);
//              while (!(CISR & (1 << 3)));     //Wait a EOF then begin start DMA
		ov9640_auto_function_on();
		// start dma
		start_dma_transfer(cam_ctx, 0);
	}
	else {
		status = get_ov_format(cam_ctx, &ovSizeFormat, &ovFormat);
		if (status)
			return -1;
		if (pov->pre_size == OV_SIZE_NONE)
			return -1;
		printk("in function %s, cam_ctx->block_tail == 0\n", __FUNCTION__);
		if (pov->pre_size == ovSizeFormat) {
			if (cam_ctx->block_tail == 0) {
			}
			else {
				struct page **p_page;
				struct page **pdes;

				printk("in function %s, cam_ctx->block_tail != 0\n", __FUNCTION__);
				p_page = &cam_ctx->page_array[cam_ctx->block_tail * cam_ctx->pages_per_block];
				pdes = &cam_ctx->page_array[cam_ctx->block_tail * cam_ctx->pages_per_block];
				COPY_BUFFERS(pdes, p_page, cam_ctx->fifo0_transfer_size);
				COPY_BUFFERS(pdes, p_page, cam_ctx->fifo1_transfer_size);
				COPY_BUFFERS(pdes, p_page, cam_ctx->fifo2_transfer_size);
			}
			set_still_image_ready(1);
			return 0;
		}
		else {
			ov9640_auto_function_off();
			cam_ctx->block_tail = cam_ctx->block_header = 0;
			ov9640_prepare_capture(cam_ctx, ovSizeFormat, ovFormat);
		}
		// turn viewfinder on
		ov9640_viewfinder_on();
	}
	// turn viewfinder on
	ov9640_viewfinder_on();
	ov9640_read(0xc, &value);
	ov9640_set_start();
	return 0;
}

int Ov9640AutoFunctionOn()
{
	u8 regValue;

	printk("in function %s \n", __FUNCTION__);
	ov9640_read(0x13, &regValue);
	if ((regValue & 0x7) == 7)
		return 1;
	return 0;
}

int camera_func_ov9640_stop_capture(p_camera_context_t cam_ctx)
{
	if (ov9640_output_stoped())
		return 0;
	printk("in camera_func_ov9640_stop_capture\n");
	//if (Ov9640AutoFunctionOn())
	ov9640_save_gains();
	// turn auto function off
	//ov9640_auto_function_off();

	// turn viewfinder off
	ov9640_viewfinder_off();
	stop_dma_transfer(cam_ctx);
	ov9640_set_stop(cam_ctx);
	return 0;
}

int camera_func_ov9640_command(p_camera_context_t cam_ctx, unsigned int cmd, void *param)
{
	int ret = 0;

	printk("in function %s, param =%d, \n", __FUNCTION__, param);
	switch (cmd) {
		/* get capture size */
	case VIDIOCGWIN:
	{
		struct video_window vw;
		vw.width = cam_ctx->capture_width;
		vw.height = cam_ctx->capture_height;
		ov9640_get_window(&vw);
		if (copy_to_user(param, &vw, sizeof(struct video_window)))
			ret = -EFAULT;
		break;
	}

		/* set capture size. */
	case VIDIOCSWIN:
	{
		struct video_window vw;
		if (copy_from_user(&vw, param, sizeof(vw))) {
			ret = -EFAULT;
			break;
		}
		printk("in %s, vw-x =%d, vw-y=%d,vw.width=%d,vw.height=%d\n",
		       __FUNCTION__, vw.x, vw.y, vw.width, vw.height);
		if (vw.width > 1280 || vw.height > 960 || vw.width < MIN_WIDTH || vw.height < MIN_HEIGHT) {
			ret = -EFAULT;
			break;
		}
		/*
		 */
		ov9640_set_window(&vw);
		cam_ctx->capture_width = vw.width;
		cam_ctx->capture_height = vw.height;
		camera_set_capture_format(cam_ctx);
		break;
	}
		/*set picture style */
	case WCAM_VIDIOCSSTYLE:
	{
		V4l_PIC_STYLE capture_style;
		capture_style = (V4l_PIC_STYLE) param;
		cam_ctx->capture_style = (V4l_PIC_STYLE) param;
		//ret = ov9640_set_special_effect(capture_style);
		printk("in camera_func_ov9640_command ret=%d\n", ret);
		if (cam_ctx->capture_style != V4l_STYLE_BLACK_WHITE && cam_ctx->capture_style != V4l_STYLE_SEPIA) {
		}
	}
		break;
		/*set picture light */
	case WCAM_VIDIOCSLIGHT:
		cam_ctx->capture_light = (int) param;
		ret = ov9640_set_white_balance(param);
		break;
		/*set picture brightness */
	case WCAM_VIDIOCSBRIGHT:
		cam_ctx->capture_bright = (int) param;
		//ret = ov9640_set_expose_compensation((int) param);
		ret = ov9640_set_brightness((int) param);
		break;
		/*set sensor size */
	case WCAM_VIDIOCSSSIZE:
		return ov9640_set_sensor_size(cam_ctx, param);

		/*get sensor size */
	case WCAM_VIDIOCGSSIZE:
		return ov9640_get_sensor_size(cam_ctx, param);

		/*set output size */
	case WCAM_VIDIOCSOSIZE:
		return ov9640_set_output_size(cam_ctx, param);

		/*get output size */
	case WCAM_VIDIOCGOSIZE:
		return ov9640_get_output_size(cam_ctx, param);
#if 0
#endif
		/*set video mode fps */
	case WCAM_VIDIOCSFPS:
	{
		struct {
			int fps, minfps;
		} cam_fps;
		DPRINTK("WCAM_VIDIOCSFPS");
		if (copy_from_user(&cam_fps, param, sizeof(int) * 2)) {
			return -EFAULT;
		}
		cam_ctx->fps = cam_fps.fps;
		cam_ctx->mini_fps = cam_fps.minfps;
		return ov9640_set_fps(cam_fps.fps, cam_fps.minfps);
	}
		return -1;
	case WCAM_VIDIOCSNIGHTMODE:
	{
		struct {
			int mode, maxexpottime;
		} cam_mode;
		int mode;
		if (copy_from_user(&cam_mode, param, sizeof(cam_mode))) {
			return -EFAULT;
		}
		mode = cam_mode.mode;
		if (mode == V4l_NM_NIGHT)
			ov9640_set_night_mode();
		if (mode == V4l_NM_ACTION)
			ov9640_set_action_mode();
		if (mode == V4l_NM_AUTO)
			ov9640_set_auto_mode();
	}
		break;
	case WCAM_VIDIOCSCONTRAST:
	{
		DPRINTK("WCAM_VIDIOCSCONTRAST  parameter=%d",param);
		ret = ov9640_set_contrast((int)param/50);
		break;
	}
	case WCAM_VIDIOCSFLICKER:
	{
		ret = ov9640_set_light_environment(param);
		break;
	}
	default:
		printk("in %s default case -----------------cmd =%d param=%d\n", __FUNCTION__, cmd, param);
		ret = -1;
	}
	return ret;
}
int camera_func_ov9640_pm(p_camera_context_t cam_ctx, int suspend)
{
	static int resume_dma = 0;
	if (suspend) {
		if (cam_ctx != NULL) {
			if (cam_ctx->dma_started) {
				dbg_print("camera running, suspended");
				stop_dma_transfer(cam_ctx);
				resume_dma = 1;
			}
		}

		disable_irq(IRQ_CAMERA);
		CKEN &= ~CKEN24_CAMERA;
	}
	else {
		CKEN |= CKEN24_CAMERA;
		enable_irq(IRQ_CAMERA);

		if (cam_ctx != NULL) {
			dbg_print("camera running, resumed");
			camera_init(cam_ctx);
			//ov9640_restore_property(cam_ctx, 0);


			if (resume_dma == 1) {
				camera_start_video_capture(cam_ctx, 0);
				resume_dma = 0;
			}
		}
	}
	return 0;
}
