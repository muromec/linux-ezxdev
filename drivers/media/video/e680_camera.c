/*
 * Copyright (C) 2004 Motorola Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Revision History:
 *                             Modification     Tracking
 * Author                 Date          Number     Description of Changes
 * ----------------   ------------    ----------   -------------------------
 * wangfei(w20239)      04/15/2004                  Created   
 * 
 * ==================================================================================
 *                                  INCLUDE FILES
 * ==================================================================================
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

#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/pagemap.h>
#include <linux/wrapper.h>
#include <linux/videodev.h>
#include <linux/pci.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/wait.h>

#include "camera.h"
#include "adcm2700.h"

#define MCLK_DEFT	        13             /* Default Master clock*/

extern int i2c_adcm2700_init(void);
extern int i2c_mt9v111_init(void);
//////////////////////////////////////////////////////////////////////////////////////
//adcm2700 functions
//
int camera_func_adcm2700_init(p_camera_context_t);
int camera_func_adcm2700_deinit(p_camera_context_t);
int camera_func_adcm2700_docommand(p_camera_context_t cam_ctx, unsigned int cmd, void *param);
int camera_func_adcm2700_set_capture_format(p_camera_context_t);
int camera_func_adcm2700_start_capture(p_camera_context_t, unsigned int frames);
int camera_func_adcm2700_stop_capture(p_camera_context_t);

int camera_func_adcm2700_pm_management(p_camera_context_t, int);

//////////////////////////////////////////////////////////////////////////////////////
//mt9v111 functions
//
int camera_func_mt9v111_init(p_camera_context_t);
int camera_func_mt9v111_deinit(p_camera_context_t);
int camera_func_mt9v111_docommand(p_camera_context_t cam_ctx, unsigned int cmd, void *param);
int camera_func_mt9v111_set_capture_format(p_camera_context_t);
int camera_func_mt9v111_start_capture(p_camera_context_t, unsigned int frames);
int camera_func_mt9v111_stop_capture(p_camera_context_t);

int camera_func_mt9v111_pm_management(p_camera_context_t, int);


extern int camera_func_init(p_camera_context_t cam_ctx);


camera_function_t  camera_func = 
{
   init:                camera_func_init,
};

int camera_func_init(p_camera_context_t cam_ctx)
{
     // Configure CI according to hardware
  	// master parallel with 8 data pins
	ci_set_mode(CI_MODE_MP, CI_DATA_WIDTH8); 

	// enable pixel clock(sensor will provide pclock) and master clock = 26MHZ
	ci_set_clock(cam_ctx->clk_reg_base, 1, 1, MCLK_DEFT);

	// data sample on rising and h,vsync active high
	ci_set_polarity(0, 0, 0);
	
	// fifo control
	ci_set_fifo(0, CI_FIFO_THL_32, 1, 1); // quality

	// Turn on M_CLK using xx MHz and wait for 150 ms.
	ci_enable(1);
	mdelay(150);
    
    cam_ctx->mclk = MCLK_DEFT;
    dbg_print("detect...");    
    if(i2c_adcm2700_init() == 0)
    {
      dbg_print("Agilent ADCM2700 camera module detected!");
      cam_ctx->sensor_type = CAMERA_TYPE_ADCM_2700;
      
      camera_func.deinit             = camera_func_adcm2700_deinit;
      camera_func.command          = camera_func_adcm2700_docommand;
      camera_func.set_capture_format = camera_func_adcm2700_set_capture_format;
      camera_func.start_capture      = camera_func_adcm2700_start_capture; 
      camera_func.stop_capture       = camera_func_adcm2700_stop_capture;
      camera_func.pm_management      = camera_func_adcm2700_pm_management;

      return camera_func_adcm2700_init(cam_ctx);
    }
    else if(i2c_mt9v111_init() == 0)
    {
       dbg_print("Micro MT9V111 camera module detected!");
       cam_ctx->sensor_type = CAMERA_TYPE_MT9V111;

       camera_func.deinit             = camera_func_mt9v111_deinit;
       camera_func.command          = camera_func_mt9v111_docommand;
       camera_func.set_capture_format = camera_func_mt9v111_set_capture_format;
       camera_func.start_capture      = camera_func_mt9v111_start_capture; 
       camera_func.stop_capture       = camera_func_mt9v111_stop_capture;
       camera_func.pm_management      = camera_func_mt9v111_pm_management;

      return camera_func_mt9v111_init(cam_ctx);
    }
    return -1;
}

