/*
 *  adcm2650.c
 *
 *  ADCM 2650 Camera Module driver.
 *
 *  Copyright (C) 2003, Intel Corporation
 *  Copyright (C) 2003, Montavista Software Inc.
 *
 *  Author: Intel Corporation Inc.
 *          MontaVista Software, Inc.
 *           source@mvista.com
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
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

#include <asm/pgtable.h>
#include <asm/dma.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>

#include "pxa_camera.h"
#include "adcm2650.h"
#include "adcm2650_hw.h"

#define MCLK_DEFT	7  	/* 7 MHz */

/* Use PLL increase the Sensor and Image Processor clock frequencies */
#define ADCM2650_USE_PLL

#ifdef ADCM2650_USE_PLL
/* Default VCO/MCLK ratio is 7.4 */
#define ADCM2650_PLL_DIV_HI	0x4909 /* VCO/MCLK ratio is 74/10 */
#define ADCM2650_PLL_DIV_LO	0x2404 /* VCO/MCLK ratio is 37/5 */
#endif

/***********************************************************************
 *
 * ADCM2650 Functions
 *
 ***********************************************************************/
int camera_func_adcm2650_init(  p_camera_context_t camera_context )
{
	u8 sensor_rev;
	u16 cm_rev, regv;
	int status;

	// Configure CI according to ADCM2650's hardware
	// master parallel with 8 data pins
	ci_set_mode(CI_MODE_MP, CI_DATA_WIDTH8);

	// enable pixel clock(sensor will provide pclock) and master clock = 7MHZ
	ci_set_clock(camera_context->clk_reg_base, 1, 1, MCLK_DEFT);

	// data sample on rising and h,vsync active high
	ci_set_polarity(0, 0, 0);

	// fifo control
	ci_set_fifo(0, CI_FIFO_THL_32, 1, 1); // quality

	// ADCM2650 Power on sequence
	// 1 Turn on M_VCC and wait for 20 ms.
	// initilization on Board Level
	// In MiscWrite Reg1(Phys Addr: 0x0800_0080):
	//   Bit 15: CAMERA_ON (1:On  0: Off)
	//   Bit 14: CAMERA_SEL (1:Routed to Camera Interface  0: To SSP1, USIM, PWM1)
	MST_MSCWR1 |= (MST_MSCWR1_CAMERA_ON | MST_MSCWR1_CAMERA_SEL);
	mdelay(20);
	
	// init I2C.
	i2c_init();

	// 2 Turn on M_CLK using xx MHz and wait for 150 ms.
	ci_enable(1);
	mdelay(150);

	// read out version
	status = adcm2650_version_revision(&cm_rev, &sensor_rev);
	if (cm_rev != PIPE_REV || sensor_rev != SENSOR_REV) 
		return STATUS_FAILURE;

	// power on the module
	// note: pll is NOT enabled, ADCM is using external clock
	status = adcm2650_power_on(MCLK_DEFT);// external mclk = 7M HZ 
	
	// set default output format
	status = adcm2650_switch_to_normal(VIEWFINDER_MODE);
	status = adcm2650_switch_to_normal(STILLFRAME_MODE);
	status = adcm2650_viewfinder_cfg_output(O_FORMAT_422_B_YCbYCr);
	status = adcm2650_stillframe_cfg_output(O_FORMAT_422_B_YCbYCr);

	// clear video sub-sampling and set v-mirror and h-mirror
	status = adcm2650_pipeline_read(VIDEO_CONFIG, &regv); 
	regv &= ~VIDEO_CONFIG_SS;   
	regv |= VIDEO_CONFIG_V_MIRROR | VIDEO_CONFIG_H_MIRROR;  
	regv |= VIDEO_CONFIG_V_MIRROR;  
	adcm2650_pipeline_write(VIDEO_CONFIG, regv);

#ifdef ADCM2650_USE_PLL
	// set VCO to default 7.4xMCLK
	adcm2650_pipeline_read(PLL_CTRL_1, &regv);
	adcm2650_pipeline_write(PLL_CTRL_1, ADCM2650_PLL_DIV_HI);
	adcm2650_pipeline_read(PLL_CTRL_2, &regv);
	adcm2650_pipeline_write(PLL_CTRL_2, ADCM2650_PLL_DIV_LO);
	adcm2650_pipeline_write(CMD_2, 0x0300);

	/* wait for new VCO clock stabilization */
	do {
		mdelay(1);
		adcm2650_pipeline_read(CMD_2, &regv);
	} while ((regv & 0x0300) != 0);
#endif

	return 0;
}

int camera_func_adcm2650_deinit(  p_camera_context_t camera_context )
{
	// power off the external module
	adcm2650_power_off(); 
	
	// disable CI
	ci_disable(1);  // quick disable
	i2c_deinit();
	// cut down at Board Level
	MST_MSCWR1 &= ~(MST_MSCWR1_CAMERA_ON | MST_MSCWR1_CAMERA_SEL);
	return 0;
}

int camera_func_adcm2650_set_capture_format(  p_camera_context_t camera_context )
{
	adcm_window_size wsize;
	u16 adcm_format;

	// set sensor input/output window
	wsize.width = camera_context->capture_width;
	wsize.height = camera_context->capture_height;
	adcm2650_viewfinder_input_size(&wsize);
	adcm2650_viewfinder_output_size(&wsize);
 	adcm2650_stillframe_input_size(&wsize);
	adcm2650_stillframe_output_size(&wsize);
	
#if 0
	fwrow = (((640 - wsize.height) / 2) + 4) / 4;
	fwcol = (((480 - wsize.width ) / 2) + 24) / 4;
	lwrow = (((640 - wsize.height) / 2) + wsize.height + 24 ) / 4;
	lwcol = (((480 - wsize.width ) / 2) + wsize.width + 44 ) / 4;
	
	adcm2650_sensor_write_ws(FWROW, fwrow);
	adcm2650_sensor_write_ws(FWCOL, fwcol);
	adcm2650_sensor_write_ws(LWROW, lwrow);
	adcm2650_sensor_write_ws(LWCOL, lwcol);
#endif
	// set sensor format
	switch(camera_context->capture_input_format) {
	case CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR:
	case CAMERA_IMAGE_FORMAT_YCBCR422_PACKED:
		adcm_format = O_FORMAT_422_B_YCbYCr;
		break;
	case CAMERA_IMAGE_FORMAT_RGB565:
		adcm_format = O_FORMAT_565_RGB;
		break;
	case CAMERA_IMAGE_FORMAT_RGB888_PACKED:
	case CAMERA_IMAGE_FORMAT_RGB888_PLANAR:
		adcm_format = O_FORMAT_888RGB;
		break;
	default:
		adcm_format = O_FORMAT_422_B_YCbYCr;
		break;
	}
	adcm2650_viewfinder_cfg_output(adcm_format);
	adcm2650_stillframe_cfg_output(adcm_format);

	return 0;
}

int camera_func_adcm2650_start_capture(  p_camera_context_t camera_context, unsigned int frames )
{   
	// frames=0 means continues capture	
	if (frames == 0) {
		// set viewfinder to infinite output
		adcm2650_resume_to_full_output_mode();
		adcm2650_viewfinder_on();
	}
	else {
		// halt viewfinder output
		adcm2650_halt_video_output();
		// limit output frames
		adcm2650_pipeline_write(UART_CREDITS, frames);
	}	
	
	// turn viewfinder on
	adcm2650_viewfinder_on();

	return 0;
}

int camera_func_adcm2650_stop_capture(  p_camera_context_t camera_context )
{
	adcm2650_viewfinder_off();
	mdelay(200);
	return 0;
}
