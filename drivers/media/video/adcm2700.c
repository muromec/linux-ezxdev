/*
 *  adcm2700.c
 *
 *  Agilent ADCM 2700 Camera Module driver.
 *
 *  Copyright (C) 2003-2004 Motorola Inc.
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
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 
Revision History:
                            Modification     Tracking
Author                 Date          Number     Description of Changes
----------------   ------------    ----------   -------------------------
wangfei(w20239)      12/15/2003     LIBdd35749   Created   
wangfei(w20239)      02/05/2004     LIBdd74309   Set frame rate in video mode
wangfei(w20239)     02/26/2004      LIBdd81055   New chip id support
                                                 Update algorithm for DMA transfer
                                                 Update strategy for memory management
                                                 Fix still picture capture failed sometime
                                                 New Agilent sensor chip ID support
                                                 Make output height in an even multiple of 8

*/

/*================================================================================
                                 INCLUDE FILES
==================================================================================*/
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
#include "adcm2700_hw.h"


/***********************************************************************
 *
 * ADCM2700  and MT9v111 common functions
 *
 ***********************************************************************/
#define MAX_WIDTH          640
#define MAX_HEIGHT         480
#define MIN_WIDTH          64
#define MIN_HEIGHT         64
#define WIDTH_DEFT         240
#define HEIGHT_DEFT        320
#define FRAMERATE_DEFT	   15
#define MCLK_DEFT          6             /* Default Master clock*/
#define BUF_SIZE_DEFT      ((PAGE_ALIGN(MAX_WIDTH * MAX_HEIGHT) + (PAGE_ALIGN(MAX_WIDTH*MAX_HEIGHT/2)*2)))

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

int camera_func_init(p_camera_context_t cam_ctx);


camera_function_t  e680_camera_func = 
{
   init:                camera_func_init,
};

int camera_func_init(p_camera_context_t cam_ctx)
{
    int ret = 0;
    // init context status
    cam_ctx->dma_channels[0] = 0xFF;
    cam_ctx->dma_channels[1] = 0xFF;
    cam_ctx->dma_channels[2] = 0xFF;
    
    cam_ctx->capture_width  = WIDTH_DEFT;
    cam_ctx->capture_height = HEIGHT_DEFT;
    
    cam_ctx->capture_input_format  = CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR;
    cam_ctx->capture_output_format = CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR;
    
    cam_ctx->frame_rate = cam_ctx->fps = FRAMERATE_DEFT;
    
    cam_ctx->mini_fps = FRAMERATE_DEFT-5;
    
    cam_ctx->mclk = MCLK_DEFT;
    cam_ctx->flicker_freq = 50;
    
    cam_ctx->buf_size     = BUF_SIZE_DEFT;
    cam_ctx->dma_descriptors_size = (cam_ctx->buf_size/PAGE_SIZE + 10);
    cam_ctx->vc.maxwidth  = MAX_WIDTH;
    cam_ctx->vc.maxheight = MAX_HEIGHT;
    cam_ctx->vc.minwidth  = MIN_WIDTH; 
    cam_ctx->vc.minheight = MIN_HEIGHT;
       
    camera_gpio_init();
    ci_init();
    
    // Configure CI according to hardware
  	// master parallel with 8 data pins
   	ci_set_mode(CI_MODE_MP, CI_DATA_WIDTH8); 

  	// enable pixel clock(sensor will provide pclock)
   	ci_set_clock(cam_ctx->clk_reg_base, 1, 1, cam_ctx->mclk);

   	// data sample on rising and h,vsync active high
   	ci_set_polarity(0, 0, 0);
	
   	// fifo control
   	ci_set_fifo(0, CI_FIFO_THL_32, 1, 1); // quality

   	// Turn on M_CLK using xx MHz and wait for 150 ms.
    ci_enable(1);
    
    mdelay(150);
    
    if(i2c_adcm2700_init() == 0)
    {
        cam_ctx->sensor_type = CAMERA_TYPE_ADCM_2700;
        e680_camera_func.deinit             = camera_func_adcm2700_deinit;
        e680_camera_func.command            = camera_func_adcm2700_docommand;
        e680_camera_func.set_capture_format = camera_func_adcm2700_set_capture_format;
        e680_camera_func.start_capture      = camera_func_adcm2700_start_capture; 
        e680_camera_func.stop_capture       = camera_func_adcm2700_stop_capture;
        e680_camera_func.pm_management      = camera_func_adcm2700_pm_management;

        if((ret =  camera_func_adcm2700_init(cam_ctx)) < 0)
        {
         dbg_print("adcm2700 init error! capture format!");
         return -1;
        }
        ddbg_print("Agilent ADCM2700 camera module detected!");
    }
    else if(i2c_mt9v111_init() == 0)
    {
        cam_ctx->sensor_type = CAMERA_TYPE_MT9V111;

        e680_camera_func.deinit             = camera_func_mt9v111_deinit;
        e680_camera_func.command            = camera_func_mt9v111_docommand;
        e680_camera_func.set_capture_format = camera_func_mt9v111_set_capture_format;
        e680_camera_func.start_capture      = camera_func_mt9v111_start_capture; 
        e680_camera_func.stop_capture       = camera_func_mt9v111_stop_capture;
        e680_camera_func.pm_management      = camera_func_mt9v111_pm_management;
        if((ret =  camera_func_mt9v111_init(cam_ctx)) < 0)
        {
         dbg_print("mt9v111 init error! capture format!");
         return -1;
        }
        ddbg_print("Micro MT9V111 camera module detected!");
    }
    else
    {
       dbg_print("no camera sensor detected!!!\n");
       return -1;
    }

    return 0;
}

/***********************************************************************
 *
 * ADCM2700 Functions
 *
 ***********************************************************************/
int camera_func_adcm2700_init(  p_camera_context_t camera_context )
{
    u16 sensor_rev, cm_rev;
    int i;
    adcm2700_power_on(camera_context->mclk);
    //read out version
    //adcm2700_version_revision(&cm_rev, &sensor_rev);
    return 0;
}

int camera_func_adcm2700_deinit(  p_camera_context_t camera_context )
{
    /* power off the external module */
    camera_func_adcm2700_stop_capture(camera_context);
    /* disable CI */
    ci_disable(1);
	
    i2c_adcm2700_cleanup();
   	camera_gpio_deinit();
    return 0;
}

int camera_func_adcm2700_set_capture_format(p_camera_context_t camera_context)
{
    u16 adcm_format;
    adcm_window_size size;

	   //set sensor format
    switch(camera_context->capture_input_format)
    {
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
    size.width  = camera_context->capture_width;
    size.height = camera_context->capture_height;
    //adcm2700_output_size(&size);
  	//adcm2700_viewfinder_cfg_output(adcm_format);
   return 0;
}

int camera_func_adcm2700_start_capture(p_camera_context_t cam_ctx, unsigned int frames)
{   
    int   cisr;
    int   wait_count;
    int   error_count = 0;
    static int error_times = 0;
 
start:    
  
    // frames=0 means video mode capture	
    if(frames == 0) 
    {
       // ddbg_print("video capture!"); 
       wait_count = 1;
       adcm2700_restore_property(cam_ctx, frames);
       adcm2700_viewfinder_on();
       ci_disable(1);
       ci_enable(1);       
 
    }
    else 
    {
       // ddbg_print("still capture");
       
        wait_count = 1;
        adcm2700_restore_property(cam_ctx, frames);
        adcm2700_snapshot_trigger();
        ci_disable(1);
        ci_enable(1);       

    }

    dbg_print("wait for EOF %d time", wait_count);
    unsigned int start_time = 0xFFFFF * 20 * wait_count;
    unsigned int flags;
    
  //  local_irq_save(flags);
    CISR |= CI_CISR_EOF;       
    CISR |= CI_CISR_SOF;       
    cisr=CISR;
    while(wait_count)
    {
       if(cisr & CI_CISR_EOF)
       {
          //wait_count -- ;
          CISR |= CI_CISR_EOF;
       }   
       
       if(cisr & CI_CISR_SOF)
       {
          wait_count -- ;
          CISR |= CI_CISR_SOF;
       }   
              
       cisr=CISR;
       if(!(--start_time))
       {
          goto wait_EOF_error;
       }

    }
    ci_disable(1);
    ci_enable(1);
    ci_reset_fifo();
    ci_clear_int_status(0xFFFFFFFF);
    mdelay(1);
    start_dma_transfer(cam_ctx, cam_ctx->block_header);
//    local_irq_restore(flags);
    dbg_print("wait ok..%d", start_time);
    return 0;
    
wait_EOF_error:
   // local_irq_restore(flags);
    error_times++;
    if(error_count++ >= 3)
    {
      return -1;
    }
    dbg_print("wait EOF error! error_count = %d", error_count);
    dbg_print("wait EOF error! error_times = %d", error_times);    
    dbg_print("Reset CIF and camera...");
    ci_disable(1);
   // master parallel with 8 data pins
    ci_set_mode(CI_MODE_MP, CI_DATA_WIDTH8); 
  	 // enable pixel clock(sensor will provide pclock)
    ci_set_clock(cam_ctx->clk_reg_base, 1, 1, cam_ctx->mclk);
  	 // data sample on rising and h,vsync active high
    ci_set_polarity(0, 0, 0);
    // fifo control
    ci_set_fifo(0, CI_FIFO_THL_32, 1, 1); // quality
    // Turn on M_CLK using xx MHz and wait for 150 ms.
    ci_enable(1);
    mdelay(10);
    adcm2700_power_on(cam_ctx->mclk);
    goto start;
}

int camera_func_adcm2700_stop_capture(p_camera_context_t cam_ctx)
{
    adcm2700_viewfinder_off();
    stop_dma_transfer(cam_ctx);
    return 0;
}

int camera_func_adcm2700_pm_management(p_camera_context_t cam_ctx, int suspend)
{
    static int resume_dma = 0;
    if(suspend)
    {
        if(cam_ctx != NULL )
        {
            if(cam_ctx->dma_started) 
            {
                ddbg_print("camera running, suspended");
                stop_dma_transfer(cam_ctx);
                resume_dma = 1;
            }
        }

        disable_irq(IRQ_CAMERA);
        CKEN &= ~CKEN24_CAMERA;
    }
    else
    {
        CKEN |= CKEN24_CAMERA;
        enable_irq(IRQ_CAMERA);

        if(cam_ctx != NULL)
        {  
            ddbg_print("camera running, resumed");
            camera_init(cam_ctx);
            if(resume_dma == 1)
            {
                camera_start_video_capture(cam_ctx, 0);
                resume_dma = 0;
            }
        }
    }
   return 0;
}

/*set picture brightness*/
static int pxa_cam_WCAM_VIDIOCSBRIGHT(p_camera_context_t cam_ctx, void * param)
{
   ddbg_print("WCAM_VIDIOCSBRIGHT");
   int ret = 0;
   if(cam_ctx->capture_bright != (int)param)
   {
     cam_ctx->capture_bright = (int)param;
     ret = adcm2700_set_bright(cam_ctx->capture_bright);
   }
   return ret;
}

/*set picture style*/  
static int pxa_cam_WCAM_VIDIOCSSTYLE(p_camera_context_t cam_ctx, void * param)
{
  ddbg_print("WCAM_VIDIOCSSTYLE");
  int ret = 0;
  if(cam_ctx->capture_style != (V4l_PIC_STYLE)param)
  {
     cam_ctx->capture_style = (V4l_PIC_STYLE)param;
     if(cam_ctx->dma_started == 1)
      {
         camera_func_adcm2700_stop_capture(cam_ctx);
         ret = camera_func_adcm2700_start_capture(cam_ctx, 0);
      } 

  }
  
  return ret;
}

        
/*set picture light*/     
static int pxa_cam_WCAM_VIDIOCSLIGHT(p_camera_context_t cam_ctx, void * param)
{
   
   dbg_print("WCAM_VIDIOCSLIGHT");
   int ret = 0;
   if(cam_ctx->capture_light != (V4l_PIC_WB)param)
   {
      cam_ctx->capture_light = (V4l_PIC_WB)param;
      if(cam_ctx->dma_started == 1)
      {
         camera_func_adcm2700_stop_capture(cam_ctx);
         ret = camera_func_adcm2700_start_capture(cam_ctx, 0);
      }
 
   }
   return ret;
}


//set  output size
static int pxa_cam_WCAM_VIDIOCSOSIZE(p_camera_context_t cam_ctx, void * param)
{

   //ddbg_print("WCAM_VIDIOCSOSIZE");
   
   adcm_window_size size;
   CI_MP_TIMING     timing;
   int ret = 0;
 
   if(copy_from_user(&size, param, sizeof(adcm_window_size))) 
   {
        return  -EFAULT;
   }
   if(cam_ctx->dma_started == 1)
   {
        return -EFAULT;
   }
   //make it in an even of multiple of 8
   size.width  = (size.width +7)/8 * 8;
   size.height = (size.height+7)/8 * 8;
   dbg_print("w=%d h=%d", size.width, size.height);
   if(cam_ctx->capture_width != size.width || cam_ctx->capture_height != size.height)
   {
     cam_ctx->capture_width  = size.width;
     cam_ctx->capture_height = size.height;
     ret = adcm2700_output_size(&size);
   
     timing.BFW = timing.BLW = 0;

     ci_configure_mp(cam_ctx->capture_width-1, cam_ctx->capture_height-1, &timing);
     camera_ring_buf_init(cam_ctx);
   }
   
   return ret;
}

/*Set  sensor size*/  
static int pxa_cam_WCAM_VIDIOCSSSIZE(p_camera_context_t cam_ctx, void * param)
{
  //ddbg_print("WCAM_VIDIOCSSSIZE");
  adcm_window_size size;
  int ret = 0;
  
  if(copy_from_user(&size, param, sizeof(adcm_window_size))) 
  {
        return  -EFAULT;
  }
  if(cam_ctx->dma_started == 1)
  {
        return -EFAULT;
  }
  //make it in an even of multiple of 8
  size.width  = (size.width +7)/8 * 8;
  size.height = (size.height+7)/8 * 8;
  
  if(cam_ctx->sensor_width != size.width ||  cam_ctx->sensor_height != size.height)
  {
    cam_ctx->sensor_width = size.width;
    cam_ctx->sensor_height = size.height;
    ret = adcm2700_input_size(&size);
    dbg_print("w = %d h = %d", size.width, size.height);
  }
  return ret;
}

static int pxa_cam_WCAM_VIDIOCSFPS(p_camera_context_t cam_ctx, void * param)
{
    //ddbg_print("WCAM_VIDIOCSFPS");
    
    struct {int fps, minfps;} cam_fps;
    int ret = 0;
    
    if(copy_from_user(&cam_fps, param, sizeof(int) * 2)) 
    {
        return  -EFAULT;
    }
    if(cam_fps.fps < 12)
    {
       cam_fps.fps = 12;
    }
     
    if(cam_ctx->fps != cam_fps.fps || cam_ctx->mini_fps != cam_fps.minfps)
    {
        cam_ctx->fps = cam_fps.fps;
        cam_ctx->mini_fps = cam_fps.minfps;
        if(cam_ctx->dma_started == 1)
        {
          camera_func_adcm2700_stop_capture(cam_ctx);
          ret = camera_func_adcm2700_start_capture(cam_ctx, 0);
        }    
    }
    return ret;
}

static int pxa_camera_WCAM_VIDIOCGCAMREG(p_camera_context_t cam_ctx, void * param)
{
    int reg_value, offset;
    //ddbg_print("WCAM_VIDIOCGCAMREG");
    if(copy_from_user(&offset, param, sizeof(int))) 
    {
        return -EFAULT;
    }
    reg_value = (int)adcm2700_reg_read((u16)offset);

    if(copy_to_user(param, &reg_value, sizeof(int))) 
    {
        return -EFAULT;
    } 

    return 0;
}
/*set flicker frequency*/
static int pxa_cam_WCAM_VIDIOCSFLICKER(p_camera_context_t cam_ctx, void * param)
{
   dbg_print("WCAM_VIDIOCSFLICKER");
   cam_ctx->flicker_freq = (int)param;

   return adcm2700_set_flicker(cam_ctx->flicker_freq);
}


static int pxa_camera_WCAM_VIDIOCSCAMREG(p_camera_context_t cam_ctx, void * param)
{
    struct reg_set_s{int val1, val2} reg_s;
    //ddbg_print("WCAM_VIDIOCSCAMREG");

    if(copy_from_user(&reg_s, param, sizeof(int) * 2)) 
    {
        return  -EFAULT;
    }
    adcm2700_write((u16)reg_s.val1, (u16)reg_s.val2);
    return 0;
} 

int camera_func_adcm2700_docommand(p_camera_context_t cam_ctx, unsigned int cmd, void *param)
{
   switch(cmd)
   {
     /*read adcm2700 registers*/
    case WCAM_VIDIOCGCAMREG:
         return pxa_camera_WCAM_VIDIOCGCAMREG(cam_ctx, param);

    /*write adcm2700 registers*/
    case WCAM_VIDIOCSCAMREG:
          return pxa_camera_WCAM_VIDIOCSCAMREG(cam_ctx, param);
        
    /*set sensor size */  
    case WCAM_VIDIOCSSSIZE:
         return pxa_cam_WCAM_VIDIOCSSSIZE(cam_ctx, param);

    /*set output size*/
    case WCAM_VIDIOCSOSIZE:
         return pxa_cam_WCAM_VIDIOCSOSIZE(cam_ctx, param);
         
          
    /*set video mode fps*/
    case WCAM_VIDIOCSFPS:
         return pxa_cam_WCAM_VIDIOCSFPS(cam_ctx, param);
            
    /*set picture style*/  
    case WCAM_VIDIOCSSTYLE:
         return pxa_cam_WCAM_VIDIOCSSTYLE(cam_ctx, param);
         
    /*set picture light*/     
    case WCAM_VIDIOCSLIGHT:
         return pxa_cam_WCAM_VIDIOCSLIGHT(cam_ctx, param);
    
    /*set picture brightness*/
    case WCAM_VIDIOCSBRIGHT:
         return pxa_cam_WCAM_VIDIOCSBRIGHT(cam_ctx, param);
         
    /*set flicker frequency*/
    case WCAM_VIDIOCSFLICKER:
         return pxa_cam_WCAM_VIDIOCSFLICKER(cam_ctx, param);

    default:
         {
           dbg_print("Error cmd=0x%x", cmd);
           return -1;
         }
  }
  return 0;
 
 }

