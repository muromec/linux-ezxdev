/*
 *  mt9m111.c
 *
 *  Micron MI-1310SOC Camera Module driver.
 *
 *  Copyright (C) 2004-2005 Motorola Inc.
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
Ma Zhiqiang          06/29/2004                 Change for auto-detect

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
#include "mt9m111.h"
#include "mt9m111_hw.h"

#define MCLK_DEFT           (26)         /* Default Master clock*/
#define MSCWR1_CAMERA_ON    (0x1 << 15)     /* Camera Interface Power Control */
#define MSCWR1_CAMERA_SEL   (0x1 << 14)     /* Camera Interface Mux control */

#define MAX_WIDTH     1280
#define MAX_HEIGHT    1024

#define MIN_WIDTH     40
#define MIN_HEIGHT    30

#define VIEW_FINDER_WIDTH_DEFT     320
#define VIEW_FINDER_HEIGHT_DEFT    240

#define FRAMERATE_DEFT    15
#define BUF_SIZE_DEFT     ((PAGE_ALIGN(MAX_WIDTH * MAX_HEIGHT) + (PAGE_ALIGN(MAX_WIDTH*MAX_HEIGHT/2)*2)))

static unsigned int mclk_out_hz;

extern int i2c_mt9m111_cleanup(void);
extern int i2c_mt9m111_init(void);

/***********************************************************************
 *
 * MT9M111 Functions
 *
 ***********************************************************************/
int camera_func_mt9m111_init(p_camera_context_t);
int camera_func_mt9m111_deinit(p_camera_context_t);
int camera_func_mt9m111_docommand(p_camera_context_t cam_ctx, unsigned int cmd, void *param);
int camera_func_mt9m111_set_capture_format(p_camera_context_t);
int camera_func_mt9m111_start_capture(p_camera_context_t, unsigned int frames);
int camera_func_mt9m111_stop_capture(p_camera_context_t);
int camera_func_mt9m111_pm_management(p_camera_context_t, int);


camera_function_t  mt9m111_func = 
{
    init:                camera_func_mt9m111_init,
    deinit:              camera_func_mt9m111_deinit,
    command:             camera_func_mt9m111_docommand,
    set_capture_format:  camera_func_mt9m111_set_capture_format,
    start_capture:       camera_func_mt9m111_start_capture,
    stop_capture:        camera_func_mt9m111_stop_capture,
    pm_management:       camera_func_mt9m111_pm_management
};

static void mt9m111_init(int dma_en)
{
    camera_gpio_init();     // only init GPIO mode

    GPCR(GPIO_CAM_EN)  = GPIO_bit(GPIO_CAM_EN);    /*set STANDBY to low*/
    GPCR(GPIO_CAM_RST) = GPIO_bit(GPIO_CAM_RST);   /*set RESET to low*/

    PGSR(GPIO_CAM_EN) |= GPIO_bit(GPIO_CAM_EN);    /*set suspend status to high*/
    PGSR(GPIO_CAM_RST)|= GPIO_bit(GPIO_CAM_RST);   /*set suspend status to high*/
    

    // Turn on M_CLK
    ci_enable(dma_en);

    // For reset to take place, hard standby should be low and clkin should be on
    udelay(100);            // delay 100us
    GPSR(GPIO_CAM_RST) = GPIO_bit(GPIO_CAM_RST);   /*set RESET to high*/
    // Wait for 10 CLKIN rising edges before using ship
    udelay(1);              // delay 1us
}

static void mt9m111_hw_deinit(void)
{
    GPSR(GPIO_CAM_EN) = GPIO_bit(GPIO_CAM_EN);      // set STANDBY to HIGH
    // Wait for 24 CLKIN after set STANDBY to hight
    udelay(2);

    /* disable CI, turn off MCLK */
    ci_disable(1);
}

static void mt9m111_deinit(void)
{
    mt9m111_reg_write(0x1B3, 0);
    mdelay(5);

    i2c_mt9m111_cleanup();

    mt9m111_hw_deinit();
}

int camera_func_mt9m111_standby(void)
{
    int ret;
    // init context status
    ci_init();

    // enable master clock
    ci_set_clock(0, 0, 1, MCLK_DEFT);

    mt9m111_init(0);

    ret = i2c_mt9m111_init();
    if(ret < 0)
    {   // i2c failed
        err_print("error: i2c initialize fail!");
        mt9m111_hw_deinit();
        ci_deinit();
        return ret;
    }

    udelay(10);

    mt9m111_reg_write(0x1B3, 0);
    mdelay(5);

    i2c_mt9m111_cleanup();

    GPSR(GPIO_CAM_EN) = GPIO_bit(GPIO_CAM_EN);      // set STANDBY to HIGH
    // Wait for 24 CLKIN after set STANDBY to hight
    udelay(2);

    /* disable CI, turn off MCLK */
    ci_disable(1);
    ci_deinit();
    return 0;
}

int camera_func_mt9m111_init(  p_camera_context_t cam_ctx )
{
    u16 device_id = 0 ;
    u16 revision = 0 ;
    int ret;

    // init context status
    
    cam_ctx->sensor_width  = MAX_WIDTH;
    cam_ctx->sensor_height = MAX_HEIGHT;
    
    cam_ctx->capture_width  = VIEW_FINDER_WIDTH_DEFT;
    cam_ctx->capture_height = VIEW_FINDER_HEIGHT_DEFT;
    
    cam_ctx->still_width  = MAX_WIDTH;
    cam_ctx->still_height = MAX_HEIGHT;
    
    cam_ctx->frame_rate = cam_ctx->fps = FRAMERATE_DEFT;
    
    cam_ctx->mini_fps = FRAMERATE_DEFT;
    
    cam_ctx->buf_size     = BUF_SIZE_DEFT;
    cam_ctx->dma_descriptors_size = (cam_ctx->buf_size/PAGE_SIZE + 10);
    strcpy (cam_ctx->vc.name, "Micron MT9M111");
    cam_ctx->vc.maxwidth  = MAX_WIDTH;
    cam_ctx->vc.maxheight = MAX_HEIGHT;
    cam_ctx->vc.minwidth  = MIN_WIDTH; 
    cam_ctx->vc.minheight = MIN_HEIGHT;

    ci_init();
 
    // Configure CI according to MT9M111's hardware
    // master parallel with 8 data pins
    ci_set_mode(CI_MODE_MP, CI_DATA_WIDTH8); 

    // enable pixel clock(sensor will provide pclock) 
    mclk_out_hz = ci_set_clock(cam_ctx->clk_reg_base, 1, 1, MCLK_DEFT);
    if(mclk_out_hz != MCLK_DEFT*1000000)
    {
        err_print("error: ci clock is not correct!");
        return -EIO;
    }
    cam_ctx->mclk = mclk_out_hz/1000000;

    // data sample on rising and h,vsync active high
    ci_set_polarity(1, 0, 0);
    
    // fifo control
    ci_set_fifo(0, CI_FIFO_THL_32, 1, 1); // quality

    mt9m111_init(1);

    ret = i2c_mt9m111_init();
    if(ret < 0)
    {   // i2c failed
        err_print("error: i2c initialize fail!");
        mt9m111_hw_deinit();
        return ret;
    }

    // read out device id
    mt9m111_get_device_id(&device_id, &revision);
    if(device_id != 0x143A) 
    {
        err_print("error: unknown device id %04x!", device_id);
        //ci_disable(1);
        //camera_gpio_deinit();
        return -ENOTSUPP;
    }
    dbg_print("Micron MT9M111 device id is %04x, rev%d!", device_id, revision);

    cam_ctx->sensor_type = CAMERA_TYPE_MT9M111;

    mt9m111_default_settings();
    
    ddbg_print("mt9m111 init success!");

    return 0;
}

int camera_func_mt9m111_deinit(  p_camera_context_t camera_context )
{
    mt9m111_deinit();

    return 0;
}

#define ASPECT_FLOAT_MULTIPLE   10000

int camera_func_mt9m111_set_capture_format(  p_camera_context_t camera_context )
{
    micron_window_size wsize;
    u16 micron_format;
    int aspect, sen_aspect, w, h, swidth, sheight, azoom, bzoom;
    ddbg_print("enter");

    if(camera_context->still_image_mode)
        return 0;

    // calculate the sensor window
    w = camera_context->capture_width;
    h = camera_context->capture_height;
    if(w < MIN_WIDTH || h < MIN_HEIGHT || w > MAX_SENSOR_A_WIDTH || h > MAX_SENSOR_A_HEIGHT)
        return -EINVAL;

    // max widht/height
    swidth = MAX_SENSOR_A_WIDTH;
    sheight = MAX_SENSOR_A_HEIGHT;
    // sensor aspect ratio
    sen_aspect = ASPECT_FLOAT_MULTIPLE * MAX_SENSOR_A_HEIGHT/MAX_SENSOR_A_WIDTH;
    // output aspect ratio
    aspect = ASPECT_FLOAT_MULTIPLE * h / w;
    if(aspect < sen_aspect)
        sheight = swidth * aspect / ASPECT_FLOAT_MULTIPLE;
    else if(aspect > sen_aspect)
        swidth = ASPECT_FLOAT_MULTIPLE * sheight / aspect;
    ddbg_print("video: sensor size %d:%d", swidth, sheight);
    camera_context->sensor_width = swidth;
    camera_context->sensor_height = sheight;

    // set sensor zoom for video mode
    azoom = camera_context->capture_digital_zoom;
    bzoom = camera_context->still_digital_zoom;
    ddbg_print("input video zoom %d, photo zoom %d", azoom, bzoom);

    if(azoom < CAMERA_ZOOM_LEVEL_MULTIPLE)
        azoom = CAMERA_ZOOM_LEVEL_MULTIPLE;
    if(azoom > CAMERA_ZOOM_LEVEL_MULTIPLE)
    {
        swidth = swidth * CAMERA_ZOOM_LEVEL_MULTIPLE / azoom;
        sheight = sheight * CAMERA_ZOOM_LEVEL_MULTIPLE / azoom;
        if(swidth/2 < camera_context->capture_width)
        {
            swidth = camera_context->capture_width * 2;
            azoom = camera_context->sensor_width * CAMERA_ZOOM_LEVEL_MULTIPLE / swidth;
            camera_context->capture_digital_zoom = azoom;
        }
        if(sheight/2 < camera_context->capture_height)
        {
            sheight = camera_context->capture_height * 2;
        }
    }

    // set sensor input window
    wsize.width = swidth;
    wsize.height = sheight;
    mt9m111_sensor_size(&wsize);
    ddbg_print("video mode: zoom %d, sensor w:h %d:%d", azoom, swidth, sheight);

    // calculate the sensor window
    w = camera_context->still_width;
    h = camera_context->still_height;
    if(w < MIN_WIDTH || h < MIN_HEIGHT || w > MAX_SENSOR_B_WIDTH || h > MAX_SENSOR_B_HEIGHT)
        return -EINVAL;

    // max widht/height
    swidth = MAX_SENSOR_B_WIDTH;
    sheight = MAX_SENSOR_B_HEIGHT;
    // sensor aspect ratio
    sen_aspect = ASPECT_FLOAT_MULTIPLE * MAX_SENSOR_B_HEIGHT/MAX_SENSOR_B_WIDTH;
    // output aspect ratio
    aspect = ASPECT_FLOAT_MULTIPLE * h / w;
    if(aspect < sen_aspect)
        sheight = swidth * aspect / ASPECT_FLOAT_MULTIPLE;
    else if(aspect > sen_aspect)
        swidth = ASPECT_FLOAT_MULTIPLE * sheight / aspect;
    ddbg_print("photo: sensor size %d:%d", swidth, sheight);
    camera_context->sensor_width = swidth;
    camera_context->sensor_height = sheight;

    // set sensor zoom for capture mode
    if(bzoom < CAMERA_ZOOM_LEVEL_MULTIPLE)
        bzoom = CAMERA_ZOOM_LEVEL_MULTIPLE;
    if(bzoom > CAMERA_ZOOM_LEVEL_MULTIPLE)
    {
        swidth = swidth * CAMERA_ZOOM_LEVEL_MULTIPLE / bzoom;
        sheight = sheight * CAMERA_ZOOM_LEVEL_MULTIPLE / bzoom;
        if(swidth < camera_context->still_width)
        {
            swidth = camera_context->still_width;
            bzoom = camera_context->sensor_width * CAMERA_ZOOM_LEVEL_MULTIPLE / swidth;
            camera_context->still_digital_zoom = bzoom;
        }
        if(sheight < camera_context->still_height)
        {
            sheight = camera_context->still_height;
        }
    }

    wsize.width = swidth;
    wsize.height = sheight;
    mt9m111_capture_sensor_size(&wsize);
    ddbg_print("photo mode: zoom %d, sensor w:h %d:%d", bzoom, swidth, sheight);

    // set sensor output window
    wsize.width = camera_context->capture_width;
    wsize.height = camera_context->capture_height;
    mt9m111_output_size(&wsize);

    wsize.width = camera_context->still_width;
    wsize.height = camera_context->still_height;
    mt9m111_capture_size(&wsize);

    // set sensor format
    switch(camera_context->capture_input_format) {
    case CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR:
    case CAMERA_IMAGE_FORMAT_YCBCR422_PACKED:
        micron_format = O_FORMAT_422_YCbYCr;
        break;
    case CAMERA_IMAGE_FORMAT_RGB565:
        micron_format = O_FORMAT_565_RGB;
        break;
    case CAMERA_IMAGE_FORMAT_RGB555:
        micron_format = O_FORMAT_555_RGB;
        break;
    case CAMERA_IMAGE_FORMAT_RGB444:
        micron_format = O_FORMAT_444_RGB;
        break;
    default:
        micron_format = O_FORMAT_422_YCbYCr;
        break;
    }
    mt9m111_output_format(micron_format);

    return 0;
}

int camera_func_mt9m111_start_capture(  p_camera_context_t cam_ctx, unsigned int frames )
{
    int waitingFrame = 0;

    //ci_reset_fifo();
    ci_clear_int_status(0xFFFFFFFF);

    // frames=0 means video mode capture    
    if (frames == 0) 
    {
        ddbg_print("video capture!"); 
        mt9m111_viewfinder_on();
    }
    else 
    {
        ddbg_print("still capture");
        mt9m111_snapshot_trigger();
    }

    if(frames == 1) //Wait 1 frames to begin capture photo
    {
        waitingFrame = 1;
    } 
    else
    { 
        waitingFrame = 1;
    }
    camera_skip_frame(cam_ctx, waitingFrame);
    return 0;
}

int camera_func_mt9m111_stop_capture(  p_camera_context_t camera_context )
{
    mt9m111_viewfinder_off();
    stop_dma_transfer(camera_context);

    return 0;
}

int camera_func_mt9m111_pm_management(p_camera_context_t cam_ctx, int suspend)
{
    return 0;
}

static int pxa_camera_WCAM_VIDIOCGCAMREG(p_camera_context_t cam_ctx, void * param)
{
    int reg_value, offset;
    ddbg_print("WCAM_VIDIOCGCAMREG");
    if(copy_from_user(&offset, param, sizeof(int))) 
    {
        return -EFAULT;
    }
    reg_value = (int)mt9m111_reg_read((u16)offset);

    if(copy_to_user(param, &reg_value, sizeof(int))) 
    {
        return -EFAULT;
    } 

    return 0;
}
static int pxa_camera_WCAM_VIDIOCSCAMREG(p_camera_context_t cam_ctx, void * param)
{
    struct reg_set_s{int val1; int val2;} reg_s;
    ddbg_print("WCAM_VIDIOCSCAMREG");

    if(copy_from_user(&reg_s, param, sizeof(int) * 2)) 
    {
        return  -EFAULT;
    }
    mt9m111_reg_write((u16)reg_s.val1, (u16)reg_s.val2);
    return 0;
} 
 
static int pxa_cam_WCAM_VIDIOCSFPS(p_camera_context_t cam_ctx, void * param)
{
    struct {int fps, minfps;} cam_fps;
    ddbg_print("WCAM_VIDIOCSFPS");
    if(copy_from_user(&cam_fps, param, sizeof(int) * 2)) 
    {
        return  -EFAULT;
    }
    cam_ctx->fps = cam_fps.fps;
    cam_ctx->mini_fps = cam_fps.minfps;
    mt9m111_set_fps(cam_fps.fps, cam_fps.minfps);
    return 0;
}


/*Set  sensor size*/  
static int pxa_cam_WCAM_VIDIOCSSSIZE(p_camera_context_t cam_ctx, void * param)
{
  micron_window_size size;
  
  if(copy_from_user(&size, param, sizeof(micron_window_size))) 
  {
        return  -EFAULT;
  }
  
  size.width = (size.width+3)/4 * 4;
  size.height = (size.height+3)/4 * 4;
  cam_ctx->sensor_width = size.width;
  cam_ctx->sensor_height = size.height;

  ddbg_print("w=%d h=%d", size.width, size.height);
  return 0;
}

//set  output size
static int pxa_cam_WCAM_VIDIOCSOSIZE(p_camera_context_t cam_ctx, void * param)
{
   micron_window_size size;
  
   if(copy_from_user(&size, param, sizeof(micron_window_size))) 
   {
        return  -EFAULT;
   }

   //make it in an even number
   size.width = (size.width+1)/2 * 2;
   size.height = (size.height+1)/2 * 2;
   
   cam_ctx->capture_width  = size.width;
   cam_ctx->capture_height = size.height;
   
   ddbg_print("w=%d h=%d", size.width, size.height);
   return 0;
}
    
/*set picture style*/  
static int pxa_cam_WCAM_VIDIOCSSTYLE(p_camera_context_t cam_ctx, void * param)
{
  ddbg_print("WCAM_VIDIOCSSTYLE");
  cam_ctx->capture_style = (V4l_PIC_STYLE)param;
  
  return mt9m111_set_style(cam_ctx->capture_style);
}

/*set picture light*/     
static int pxa_cam_WCAM_VIDIOCSLIGHT(p_camera_context_t cam_ctx, void * param)
{
   ddbg_print("WCAM_VIDIOCSLIGHT");
   cam_ctx->capture_light = (V4l_PIC_WB)param;

   return  mt9m111_set_light((V4l_PIC_WB)param);
}
    
/*set picture brightness*/
static int pxa_cam_WCAM_VIDIOCSBRIGHT(p_camera_context_t cam_ctx, void * param)
{
   ddbg_print("WCAM_VIDIOCSBRIGHT");
   cam_ctx->capture_bright = (int)param;

   return  mt9m111_set_bright((int)param);
}

/*set picture contrast*/
/*static int pxa_cam_WCAM_VIDIOCSCONTRAST(p_camera_context_t cam_ctx, void * param)
{
   ddbg_print("WCAM_VIDIOCSCONTRAST");
   cam_ctx->capture_contrast = ((int)param-50)/12;

   return  mt9m111_set_contrast(cam_ctx->capture_contrast);
}*/

/*set flicker frequency*/
static int pxa_cam_WCAM_VIDIOCSFLICKER(p_camera_context_t cam_ctx, void * param)
{
   ddbg_print("WCAM_VIDIOCSFLICKER");
   cam_ctx->flicker_freq = (int)param;

   return  mt9m111_set_flicker(cam_ctx->flicker_freq);
}


/*set night mode*/
static int pxa_cam_WCAM_VIDIOCSNIGHTMODE(p_camera_context_t cam_ctx, void * param)
{
    struct {u32 mode, maxexpotime; } cam_mode;
    u32 maxexpotime;
    
    if (copy_from_user(&cam_mode, param, sizeof(cam_mode))) 
    {
        return -EFAULT;
    }

    maxexpotime = cam_mode.maxexpotime;
    if(maxexpotime == 0)
    {
        return -EFAULT;
    }

    switch (cam_mode.mode)
    {
        case V4l_NM_NIGHT:
        case V4l_NM_ACTION:
        case V4l_NM_AUTO:
            cam_ctx->mini_fps = (1000000+maxexpotime/2)/maxexpotime;
            mt9m111_set_autoexposure_zone(cam_ctx->mini_fps);
            break;
        default:
            return -EFAULT;
    }

    return 0;
}

static int mt9m111_get_exposure_para(struct V4l_EXPOSURE_PARA *expo_para);

static int pxa_cam_WCAM_VIDIOCGEXPOPARA(p_camera_context_t cam_ctx, void * param)
{
    ddbg_print("WCAM_VIDIOCGEXPOPARA");
    struct V4l_EXPOSURE_PARA expo_para;

    mt9m111_get_exposure_para(&expo_para);

    if(copy_to_user(param, &expo_para, sizeof(expo_para))) 
    {
        return -EFAULT;
    } 

    return 0;
}

/*set picture mirroring*/
static int mt9m111_WCAM_VIDIOCSMIRROR(p_camera_context_t cam_ctx, void * param)
{
    int mirror, rows, columns;
    mirror = (int)param;

    ddbg_print("WCAM_VIDIOCSMIRROR %x", mirror);

    if(mirror & CAMERA_MIRROR_VERTICALLY)
        rows = 1;
    else
        rows = 0;

    if(mirror & CAMERA_MIRROR_HORIZONTALLY)
        columns = 1;
    else
        columns = 0;

    return  mt9m111_set_mirror(rows, columns);
}

/*set strobe flash enable/disable*/
static int mt9m111_WCAM_VIDIOCSSTROBEFLASH(p_camera_context_t cam_ctx, void * param)
{
    int strobe = (int)param;
    ddbg_print("WCAM_VIDIOCSSTROBEFLASH %d", strobe);

    return  mt9m111_set_strobe_flash(strobe);
}

int camera_func_mt9m111_docommand(p_camera_context_t cam_ctx, unsigned int cmd, void *param)
{
   switch(cmd)
   {
     /*read mt9m111 registers*/
    case WCAM_VIDIOCGCAMREG:
         return pxa_camera_WCAM_VIDIOCGCAMREG(cam_ctx, param);

    /*write mt9m111 registers*/
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
    
    /*set picture contrast*/
    //case WCAM_VIDIOCSCONTRAST:
    //     return pxa_cam_WCAM_VIDIOCSCONTRAST(cam_ctx, param);

    /*set flicker frequency*/
    case WCAM_VIDIOCSFLICKER:
         return pxa_cam_WCAM_VIDIOCSFLICKER(cam_ctx, param);

    case WCAM_VIDIOCSNIGHTMODE:
         return pxa_cam_WCAM_VIDIOCSNIGHTMODE(cam_ctx, param);

    case WCAM_VIDIOCGEXPOPARA:
         return pxa_cam_WCAM_VIDIOCGEXPOPARA(cam_ctx, param);

    case WCAM_VIDIOCSMIRROR:
         return mt9m111_WCAM_VIDIOCSMIRROR(cam_ctx, param);

    case WCAM_VIDIOCSSTROBEFLASH:
         return mt9m111_WCAM_VIDIOCSSTROBEFLASH(cam_ctx, param);

    default:
         {
           err_print("Error cmd=%d", cmd);
           return -1;
         }
    }
    return 0;
 
}

/* luma ratio, 256x */ 
static unsigned int mt9m111_get_luma_ratio(void)
{
    u16 r22e, r24c;
    unsigned int ratio;

    r22e = mt9m111_reg_read(0x22e);     // AE Target Control
    r24c = mt9m111_reg_read(0x24c);     // AE Current Luma Monitor

    ratio = r24c / 0x4a;                // div default target value

    ddbg_print("R22e(0x%04x) R24c(0x%04x), ratio(%d)",
                r22e, r24c, ratio);
    return ratio;
}

/* gain 256x, range: >=64  */ 
static unsigned int mt9m111_get_gain(void)
{
    u16 r02f, r041, r23f, r262;
    unsigned int global, digital, vref, gain = 1;

    r02f = mt9m111_reg_read(0x02f);     // Global Analog Gain
    r041 = mt9m111_reg_read(0x041);     // Vref ADCs
    r23f = mt9m111_reg_read(0x23f);     // AE Zone Index
    r262 = mt9m111_reg_read(0x262);     // AE digital Gains Monitor

    /* global gain: range 32 -- 2032 */
    global = r02f & 0x007f;             // initial gain, 0.03125, 32x
    if(r02f & 0x0080)
        global *= 2;                    // bit7, 2x
    if(r02f & 0x0100)
        global *= 2;                    // bit8, 2x
    if(r02f & 0x0200)
        global *= 2;                    // bit9, 2x
    if(r02f & 0x0400)
        global *= 2;                    // bit10, 2x

    /* digital gain: range 256 -- 65025 */
    digital = (r262&0x00ff) * ((r262&0xff00)/256);    // 16x, 16x

    /* vref: range 16 -- 32 */
    vref = 16;   // TODO: calculate r041

    /* gain: range 256 --  */
    gain = (global*digital/32) * vref/16;

    ddbg_print("R02f(0x%04x) R041(0x%04x) R23f(0x%04x) R262(0x%04x), global(%d), digital(%d), vref(%d), gain(%d)",
                r02f, r041, r23f, r262, global, digital, vref, gain);
    if(gain<64)
        gain = 64;
    return gain;
}


/* shutter time: unit us */ 
static unsigned int mt9m111_get_shutter_time(void)
{
    u16 r021;
    u16 r004, r003, r007, r008;
    u16 r009, r00c;
    unsigned int a, q, row, time;
    unsigned int powermode = 2;     // initialize to Full Power
    unsigned int skip = 1;          // initialize to no skip (normal)

    r021 = mt9m111_reg_read(0x021);     // readout mode A
    if(r021 & 0x0400)
        powermode = 4;  // Low Power
    if(r021 & 0x000C)
        skip = 2;       // skip on

    r004 = mt9m111_reg_read(0x004);     // column width
    r003 = mt9m111_reg_read(0x003);     // row height
    r007 = mt9m111_reg_read(0x007);     // horizontal Blanking A
    r008 = mt9m111_reg_read(0x008);     // vertical Blanking A

    r009 = mt9m111_reg_read(0x009);     // shutter width
    r00c = mt9m111_reg_read(0x00C);     // shutter delay

    a = r004/skip+8;
    q = r007;
    row = a+q;
    // TODO: add shutter delay
    time = row * r009 * powermode * 1000 / (mclk_out_hz/1000); // us
    ddbg_print("R04(%d) R03(%d) R07(%d) R08(%d) R09(%d), time(%d us)",
                r004, r003, r007, r008, r009, time);
    if(time==0)
        time = 1;
    return time;
}

#define LUMA_COEFFICIENT 65        // should be less than 1000

static int mt9m111_get_luminance(int *lux_integer, int *lux_decimal, 
                                struct V4l_EXPOSURE_PARA *expo_para)
{
    unsigned int shutter, gain, luma_ratio;
    unsigned int lux_int, lux_dec;

    shutter = mt9m111_get_shutter_time();
    gain = mt9m111_get_gain();
    luma_ratio = mt9m111_get_luma_ratio();

    // should avoid integer overflow
    // luma_ratio, range is 0-1000
    // shutter, 1-1000000
    // gain, 64-
    lux_dec =  0;
    if(shutter<4)
    {   // shutter time < 4us
        unsigned int a1;
        a1 = LUMA_COEFFICIENT * (1000000 / 256) / shutter;
        lux_int = (luma_ratio * a1 / gain) * 256;
    }
    else if(shutter<16)
    {   // 4us < shutter time < 16us
        unsigned int a1;
        a1 = LUMA_COEFFICIENT * (1000000 / 64) / shutter;
        lux_int = (luma_ratio * a1 / gain) * 64;
    }
    else if(shutter<1024)
    {   // 16us < shutter time < 1ms
        unsigned int a1;
        a1 = LUMA_COEFFICIENT * (1000000 / 16) / shutter;
        lux_int = luma_ratio * a1 / (gain/16);
    }
    else if(shutter<131072) {
        // 1ms < shutter time < 130ms
        unsigned int a1;
        a1 = LUMA_COEFFICIENT * 1000000 * 4 / shutter;
        lux_int =  luma_ratio * a1 / 4 / gain;
        if(lux_int<50)
        {
            unsigned int a2;
            a2 = 1000 / 4 * luma_ratio * a1 / gain;
            lux_dec = a2 - lux_int*1000;
        }
    }
    else {
        // shutter time > 130ms
        unsigned int a1;
        a1 = LUMA_COEFFICIENT * 1000000 / (shutter/500);
        lux_int =  luma_ratio * a1 / 500 / gain;
        if(lux_int<50)
        {
            unsigned int a2;
            a2 = 1000 / 500 * luma_ratio * a1 / gain;
            lux_dec = a2 - lux_int*1000;
        }
    }

    ddbg_print("shutter(%d)us gain(%d) ratio(%d), lux(%d.%03d)",
                shutter, gain, luma_ratio, lux_int, lux_dec);
  
    if(lux_integer)
        *lux_integer = lux_int;
    if(lux_decimal)
        *lux_decimal = lux_dec;
    if(expo_para) {
        expo_para->luminance = lux_int;
        expo_para->shutter = shutter;
        expo_para->ISOSpeed = gain;     // TODO: temp solution
    }
    return lux_int;
}

static int mt9m111_get_exposure_para(struct V4l_EXPOSURE_PARA *expo_para)
{
    return mt9m111_get_luminance(NULL, NULL, expo_para);
}


/* camera light sensor functions */

#define MCLK_LIGHT_SENSOR           52         /* Default Master clock*/
static unsigned int start_frameno = 0;

static int mt9m111_initialize_lightsensor(void)
{
    //u16 value = 0;

    mt9m111_reg_write(0x00D, 0x0029);   /* Reset sensor and SOC */
    /* Wait a bit... */
    mt9m111_reg_write(0x00D, 0x0008);   /* Release reset */

    // enable AE, disable AWB flicker lens corr and bypass CCM
    // "large window" back light compensation control
    mt9m111_reg_write(0x106, 0x7010);
    //mt9m111_reg_write(0x226, 0x8000);     AE window boundaries
    //mt9m111_reg_write(0x227, 0x8000);
    // set AE fast adaptation
    u16 r22f = mt9m111_reg_read(0x22F);
    mt9m111_reg_write(0x22F, (r22f&0xfe20) | 0x100);    // AE Adjustment Speed A

    // sensor 2x zoom window, sensor output 640x512
    mt9m111_reg_write(0x1A7, 0x0140); //XSIZE A: No bigger than sensor window
    mt9m111_reg_write(0x1AA, 0x0100); //YSIZE A: No bigger than sensor window
    mt9m111_reg_write(0x1A6, 0x0280); //XZOOM A: No bigger than sensor window
    mt9m111_reg_write(0x1A9, 0x0200); //YZOOM A: No bigger than sensor window
    mt9m111_reg_write(0x1A5, 0x46c0); //XPAN A:Center Origin at - 320
    mt9m111_reg_write(0x1A8, 0x4700); //YPAN A: Center Origin at -256
    
    u16 r00d = 0;
    r00d = mt9m111_reg_read(0x00D);
    mt9m111_reg_write(0x00D, 0x8000|r00d); //SYNC CHANGES:Suspend changes

    mt9m111_reg_write(0x003, 0x0200); //ROW WIDTH (Window height)
    mt9m111_reg_write(0x004, 0x0280); //COLUMN WIDTH
    mt9m111_reg_write(0x001, 0x010C); //ROW START 12+256

    mt9m111_reg_write(0x002, 0x015C); //COLUMN START 28+320
    mt9m111_reg_write(0x007, 0x0061); //HBLANK A for 48 MHz Osc
    mt9m111_reg_write(0x008, 0x0011); //VBLANK A
    mt9m111_reg_write(0x00D, r00d); //SYNC CHNGES: Apply changes now

    //mt9m111_reg_write(0x021, 0x000C); //READ_MODE_A full power plus skip
    mt9m111_reg_write(0x021, 0x040C); //READ_MODE_A low power plus skip

    mt9m111_reg_write(0x009, 256);      //shutter width to get ?ms exposure (this could go lower)
    //mt9m111_reg_write(0x02F, 0x0020); //reset analog gains
    //mt9m111_reg_write(0x041, 0x00d7); //default DAC vref

    mt9m111_reg_write(0x237, 0x0200);   // to 16 // AE Zone Limits
    mt9m111_reg_write(0x257, 256);      // AE full-frame time, 60Hz, Context A
    mt9m111_reg_write(0x258, 306);      // AE full-frame time, 50Hz, Context A
    // limit ADC to no Vref changes
    mt9m111_reg_write(0x23D, 0x17DD);   // AE ADC Gain Control

    start_frameno = mt9m111_reg_read(0x19A);    // Frame Counter

    return 0;
}

#define LIGHT_SENSOR_FRAMES_TO_WAIT 8

int light_sensor_mt9m111_get_luma(int *lux_integer, int *lux_decimal)
{
    int frameno, count;
    unsigned int mclk_hz;

    count = 400;
    frameno = mt9m111_reg_read(0x19A);    // Frame Counter
    while(frameno < start_frameno+LIGHT_SENSOR_FRAMES_TO_WAIT && --count>0)
    {
        if(mclk_out_hz!=(mclk_hz=ci_get_clock()))
        {
            err_print("error: ci clock was changed to %d!", mclk_hz);
            return -EIO;
        }
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(2);
        //mt9m111_get_luminance(lux_integer, lux_decimal); // for debug
        frameno = mt9m111_reg_read(0x19A);    // Frame Counter
    }
    if(mclk_out_hz!=(mclk_hz=ci_get_clock()))
    {
        err_print("error: ci clock was changed to %d!", mclk_hz);
        return -EIO;
    }

    return mt9m111_get_luminance(lux_integer, lux_decimal, NULL);
}

int light_sensor_mt9m111_init(void)
{
    u16 device_id = 0 ;
    u16 revision = 0 ;
    int ret;

    // init context status
    ci_init();
 
    // enable master clock 
    mclk_out_hz = ci_set_clock(0, 0, 1, MCLK_LIGHT_SENSOR);
    if(mclk_out_hz != MCLK_LIGHT_SENSOR*1000000)
    {
        err_print("error: ci clock is not correct!");
        //return -EIO;
    }

    mt9m111_init(0);

    ret = i2c_mt9m111_init();
    if(ret < 0)
    {   // i2c failed
        err_print("error: i2c initialize fail!");
        mt9m111_hw_deinit();
        return ret;
    }

    // read out device id
    mt9m111_get_device_id(&device_id, &revision);
    if(device_id != 0x143A) 
    {
        err_print("error: unknown device id %04x!", device_id);
        mt9m111_deinit();
        return -ENOTSUPP;
    }

    mt9m111_initialize_lightsensor();
    
    ddbg_print("mt9m111 init success!");
    return 0;
}
    
int light_sensor_mt9m111_deinit(void)
{
    mt9m111_deinit();
    ci_deinit();
    return 0;
}

