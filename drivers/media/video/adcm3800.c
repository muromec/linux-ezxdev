/*
 *  adcm3800.c
 *
 *  Agilent ADCM 3800 Camera Module driver.
 *
 *  Copyright (C) 2005 Motorola Inc.
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
 *  Revision History:
 *                             Modification     Tracking
 *  Author                 Date          Number     Description of Changes
 *  ----------------   ------------    ----------   -------------------------
 *  Mu Chenliang        06/14/2004      LIBee41682   Created, modified from adcm2700.c
 *  Mu Chenliang        09/29/2004      LIBff19648   Update
 * 
 */

/*
 * ==================================================================================
 *                                  INCLUDE FILES
 * ==================================================================================
 */

#include <linux/types.h>
#include <linux/config.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <asm/hardware.h>

#include "camera.h"
#include "adcm3800.h"
#include "adcm3800_hw.h"


/***********************************************************************
 *
 * ADCM3800  common functions
 *
 ***********************************************************************/
#define MAX_WIDTH          1280
#define MAX_HEIGHT         1024
#define MIN_WIDTH          64
#define MIN_HEIGHT         56
#define WIDTH_DEFT         320
#define HEIGHT_DEFT        240
#define S_WIDTH_DEFT       1280
#define S_HEIGHT_DEFT      960
#define FRAMERATE_DEFT	   15
#define MCLK_DEFT          48             /* Default Master clock*/
#define BUF_SIZE_DEFT      ((PAGE_ALIGN(MAX_WIDTH * MAX_HEIGHT) + (PAGE_ALIGN(MAX_WIDTH*MAX_HEIGHT/2)*2)))

extern int i2c_adcm3800_init(void);
extern int i2c_adcm3800_cleanup(void);

//////////////////////////////////////////////////////////////////////////////////////
//adcm3800 functions
//
static int camera_func_adcm3800_init(p_camera_context_t);
static int camera_func_adcm3800_deinit(p_camera_context_t);
static int camera_func_adcm3800_docommand(p_camera_context_t cam_ctx, unsigned int cmd, void *param);
static int camera_func_adcm3800_set_capture_format(p_camera_context_t);
static int camera_func_adcm3800_start_capture(p_camera_context_t, unsigned int frames);
static int camera_func_adcm3800_stop_capture(p_camera_context_t);

static int camera_func_adcm3800_pm_management(p_camera_context_t, int);

static int adcm3800_sensor_init(p_camera_context_t cam_ctx);


camera_function_t  camera_adcm3800_func = 
{
    init:                camera_func_adcm3800_init,
    deinit:              camera_func_adcm3800_deinit,
    command:             camera_func_adcm3800_docommand,
    set_capture_format:  camera_func_adcm3800_set_capture_format,
    start_capture:       camera_func_adcm3800_start_capture,
    stop_capture:        camera_func_adcm3800_stop_capture,
    pm_management:       camera_func_adcm3800_pm_management
};

static int camera_func_adcm3800_init(p_camera_context_t cam_ctx)
{
    int ret = 0;
    // init context status
    cam_ctx->dma_channels[0] = 0xFF;
    cam_ctx->dma_channels[1] = 0xFF;
    cam_ctx->dma_channels[2] = 0xFF;
    
    cam_ctx->capture_width  = WIDTH_DEFT;
    cam_ctx->capture_height = HEIGHT_DEFT;
    cam_ctx->sensor_width  = S_WIDTH_DEFT;
    cam_ctx->sensor_height = S_HEIGHT_DEFT;
    
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
    
    mdelay(10);    // TODO: reduce it
    
    if(i2c_adcm3800_init() == 0)
    {
        cam_ctx->sensor_type = CAMERA_TYPE_ADCM3800;

        //mdelay(100);
    
        if((ret =  adcm3800_sensor_init(cam_ctx)) < 0)
        {
            dbg_print("adcm3800 hardware init error!");
            return -1;
        }
        ddbg_print("Agilent ADCM3800 camera module detected!");
    }
    else
        return -1;

    return 0;
}

/***********************************************************************
 *
 * ADCM3800 Functions
 *
 ***********************************************************************/
static int adcm3800_sensor_init(  p_camera_context_t camera_context )
{
    //u32 adcm_mclk = camera_context->mclk * 195000 / 192;   // SYS_CLK is 195Mhz
    u32 adcm_mclk = camera_context->mclk * 1000;
    adcm3800_power_on(adcm_mclk);
    return 0;
}

static int camera_func_adcm3800_deinit(  p_camera_context_t camera_context )
{
    dbg_print("adcm3800 off!");

    /* power off the external module */
    camera_func_adcm3800_stop_capture(camera_context);

    adcm3800_power_off();
    i2c_adcm3800_cleanup();

    /* disable CI */
    ci_disable(1);
	
   	camera_gpio_deinit();
    return 0;
}

static int camera_func_adcm3800_set_capture_format(p_camera_context_t camera_context)
{
    u16 adcm_format;
    window_size size;

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
    adcm3800_set_output_format(adcm_format);

    size.width  = camera_context->capture_width;
    size.height = camera_context->capture_height;
    adcm3800_set_output_size(size.width, size.height);

    return 0;
}

static int camera_func_adcm3800_start_capture(p_camera_context_t cam_ctx, unsigned int frames)
{
    int   ret;
    int   cisr;
    int   wait_count;
    static int error_times = 0;
 
    // frames=0 means video mode capture	
    if(frames == 0) 
    {
        // ddbg_print("video capture!"); 
        wait_count = 1;
    }
    else 
    {
        // ddbg_print("still capture");
        wait_count = 1;
    }
    ret = adcm3800_reconfigure(cam_ctx, frames);
    if(ret<0)
        return ret;
    adcm3800_viewfinder_on();

    ci_disable(1);
    ci_enable(1);

    dbg_print("wait for SOF %d time", wait_count);
    unsigned int start_time = 0xFFFFF * 20 * wait_count;

    CISR |= CI_CISR_SOF;
    CISR |= CI_CISR_EOF;
    cisr=CISR;
    while(wait_count)
    {
       if(cisr & CI_CISR_EOF)
       {
          dbg_print("get EOF %d:%d", wait_count, start_time);
          CISR |= CI_CISR_EOF;
       }          
       if(cisr & CI_CISR_SOF)
       {
          dbg_print("get SOF %d:%d", wait_count, start_time);
          wait_count -- ;
          CISR |= CI_CISR_SOF;
       }          
       cisr=CISR;
       if(!(--start_time))
       {
          goto wait_SOF_error;
       }

    }
    //mdelay(1);
    ci_disable(1);
    ci_enable(1);
    ci_reset_fifo();
    //ci_clear_int_status(0xFFFFFFFF);
    start_dma_transfer(cam_ctx, cam_ctx->block_header);
    dbg_print("wait ok..%d", start_time);
    return 0;
    
wait_SOF_error:
    error_times++;
    dbg_print("wait SOF error! error_times = %d", error_times);    
    return -EIO;
}

static int camera_func_adcm3800_stop_capture(p_camera_context_t cam_ctx)
{
    adcm3800_viewfinder_off();
    stop_dma_transfer(cam_ctx);
    return 0;
}

static int camera_func_adcm3800_pm_management(p_camera_context_t cam_ctx, int suspend)
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
    int ret = 0;
    int bright;
    bright = (int)param;
    ddbg_print("WCAM_VIDIOCSBRIGHT %d", bright);

    cam_ctx->capture_bright = bright;
    ret = adcm3800_set_bright(bright);

    if(cam_ctx->dma_started == 1 && ret>0)
    {
        camera_func_adcm3800_stop_capture(cam_ctx);
        ret = camera_func_adcm3800_start_capture(cam_ctx, 0);
        return ret;
    }
    return 0;
}

/*set picture style*/  
static int pxa_cam_WCAM_VIDIOCSSTYLE(p_camera_context_t cam_ctx, void * param)
{
    int ret = 0;
    V4l_PIC_STYLE style;
    style = (V4l_PIC_STYLE)param;
    ddbg_print("WCAM_VIDIOCSSTYLE %d", style);

    cam_ctx->capture_style = style;
    ret = adcm3800_set_style(style);
    if(cam_ctx->dma_started == 1 && ret>0)
    {
        camera_func_adcm3800_stop_capture(cam_ctx);
        ret = camera_func_adcm3800_start_capture(cam_ctx, 0);
        return ret;
    } 
    return 0;
}

        
/*set picture light*/     
static int pxa_cam_WCAM_VIDIOCSLIGHT(p_camera_context_t cam_ctx, void * param)
{
    int ret = 0;
    V4l_PIC_WB light;
    light = (V4l_PIC_WB)param;
    dbg_print("WCAM_VIDIOCSLIGHT %d", light);

    cam_ctx->capture_light = light;
    ret = adcm3800_set_light(light);
    if(cam_ctx->dma_started == 1)
    {
        camera_func_adcm3800_stop_capture(cam_ctx);
        ret = camera_func_adcm3800_start_capture(cam_ctx, 0);
        return ret;
    }
    return 0;
}


//set  output size
static int pxa_cam_WCAM_VIDIOCSOSIZE(p_camera_context_t cam_ctx, void * param)
{
   //ddbg_print("WCAM_VIDIOCSOSIZE");
   
   window_size size;
 
   if(copy_from_user(&size, param, sizeof(window_size))) 
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
       dbg_print("error: the camera output size should match CSWIN size");
       return -EINVAL;
   }
   
   return 0;
}

/*Set  sensor size*/  
static int pxa_cam_WCAM_VIDIOCSSSIZE(p_camera_context_t cam_ctx, void * param)
{
    //ddbg_print("WCAM_VIDIOCSSSIZE");
    window_size size;
    int ret;
  
    if(copy_from_user(&size, param, sizeof(window_size))) 
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
  
    cam_ctx->sensor_width = size.width;
    cam_ctx->sensor_height = size.height;
    dbg_print("w = %d h = %d", size.width, size.height);

    ret = adcm3800_set_sensor_size(size.width, size.height);
    if(ret<0)
        return ret;

    return 0;
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
    if(cam_fps.fps < 5)
    {
       cam_fps.fps = 5;
    }
     
    ddbg_print("WCAM_VIDIOCSFPS %d:%d", cam_ctx->fps, cam_fps.fps);
    if(cam_ctx->fps != cam_fps.fps || cam_ctx->mini_fps != cam_fps.minfps)
    {
        cam_ctx->fps = cam_fps.fps;
        cam_ctx->mini_fps = cam_fps.minfps;

        ret = adcm3800_set_fps(cam_ctx->fps * 10);
        if(ret<0)
            return ret;

        if(cam_ctx->dma_started == 1 && ret>1)
        {
            camera_func_adcm3800_stop_capture(cam_ctx);
            ret = camera_func_adcm3800_start_capture(cam_ctx, 0);
            return ret;
        }
    }
    return 0;
}

static int pxa_cam_WCAM_VIDIOCSNIGHTMODE(p_camera_context_t cam_ctx, void * param)
{
    int ret;
    struct {
        V4l_NM mode;
        int maxtime;
    } expo_mode;
    if (copy_from_user(&expo_mode, param, sizeof(expo_mode))) {
        return -EFAULT;
    }
    ddbg_print("WCAM_VIDIOCSNIGHTMODE %d:%d", expo_mode.mode, expo_mode.maxtime);

    ret = adcm3800_set_exposure_mode(expo_mode.mode, expo_mode.maxtime);
    if(cam_ctx->dma_started == 1 && ret>0)
    {
        camera_func_adcm3800_stop_capture(cam_ctx);
        ret = camera_func_adcm3800_start_capture(cam_ctx, 0);
        return ret;
    }
    return 0;
}

/*set flicker frequency*/
static int pxa_cam_WCAM_VIDIOCSFLICKER(p_camera_context_t cam_ctx, void * param)
{
    int ret;
    dbg_print("WCAM_VIDIOCSFLICKER");
    cam_ctx->flicker_freq = (int)param;

    if(cam_ctx->dma_started == 1)
        return -EPERM;

    ret = adcm3800_set_flicker(cam_ctx->flicker_freq);
    if(ret<0)
        return ret;

    return 0;
}

static int pxa_camera_WCAM_VIDIOCGCAMREG(p_camera_context_t cam_ctx, void * param)
{
    int ret, offset;
    u16 value;
    //ddbg_print("WCAM_VIDIOCGCAMREG");
    if(copy_from_user(&offset, param, sizeof(int))) 
    {
        return -EFAULT;
    }
    ret = i2c_adcm3800_read((u16)offset, &value);
    if(ret<0)
        return ret;

    if(copy_to_user(param, &value, sizeof(int))) 
    {
        return -EFAULT;
    } 

    return 0;
}

static int pxa_camera_WCAM_VIDIOCSCAMREG(p_camera_context_t cam_ctx, void * param)
{
    struct reg_set_s{int val1, val2;} reg_s;
    //ddbg_print("WCAM_VIDIOCSCAMREG");

    if(copy_from_user(&reg_s, param, sizeof(int) * 2)) 
    {
        return  -EFAULT;
    }
    i2c_adcm3800_write((u16)reg_s.val1, (u16)reg_s.val2);
    return 0;
} 

static int camera_func_adcm3800_docommand(p_camera_context_t cam_ctx, unsigned int cmd, void *param)
{
   switch(cmd)
   {
     /*read adcm3800 registers*/
    case WCAM_VIDIOCGCAMREG:
         return pxa_camera_WCAM_VIDIOCGCAMREG(cam_ctx, param);

    /*write adcm3800 registers*/
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
            
    case WCAM_VIDIOCSNIGHTMODE:
         return pxa_cam_WCAM_VIDIOCSNIGHTMODE(cam_ctx, param);
            
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

