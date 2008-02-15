/*
 *  mi2010soc.c
 *
 *  Micron MI-2010SOC Camera Module driver.
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
Revision History:
                            Modification     Tracking
Author                 Date          Number     Description of Changes
----------------   ------------    ----------   -------------------------
Mu Chenliang        04/30/2005      LIBgg02659   Created, modified from mt9m111.c
 *
*/

/*
==================================================================================
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
#include "mi2010soc.h"
#include "mi2010soc_hw.h"

#define LOG_TIME_STAMP   //If defined, the time stamp log will be printed out

#define MCLK_DEFT           (26)         /* Default Master clock*/
#define MSCWR1_CAMERA_ON    (0x1 << 15)     /* Camera Interface Power Control */
#define MSCWR1_CAMERA_SEL   (0x1 << 14)     /* Camera Interface Mux control */

#define MAX_WIDTH     MAX_SENSOR_B_WIDTH
#define MAX_HEIGHT    MAX_SENSOR_B_HEIGHT

#define MIN_WIDTH     64
#define MIN_HEIGHT    48

#define FRAMERATE_DEFT    15
#define BUF_SIZE_DEFT     (PAGE_ALIGN(MAX_WIDTH * MAX_HEIGHT * 3 / 4))  // 1.44M

#define HARDWARE_STANDBY 1 

static unsigned int mclk_out_hz;

extern int i2c_mi2010soc_cleanup(void);
extern int i2c_mi2010soc_init(void);

/***********************************************************************
 *
 * MI2010SOC Functions
 *
 ***********************************************************************/
int camera_func_mi2010soc_init(p_camera_context_t);
int camera_func_mi2010soc_deinit(p_camera_context_t);
int camera_func_mi2010soc_docommand(p_camera_context_t cam_ctx, unsigned int cmd, void *param);
int camera_func_mi2010soc_set_capture_format(p_camera_context_t);
int camera_func_mi2010soc_start_capture(p_camera_context_t, unsigned int frames);
int camera_func_mi2010soc_stop_capture(p_camera_context_t);
int camera_func_mi2010soc_pm_management(p_camera_context_t, int);


camera_function_t  mi2010soc_func = 
{
    init:                camera_func_mi2010soc_init,
    deinit:              camera_func_mi2010soc_deinit,
    command:             camera_func_mi2010soc_docommand,
    set_capture_format:  camera_func_mi2010soc_set_capture_format,
    start_capture:       camera_func_mi2010soc_start_capture,
    stop_capture:        camera_func_mi2010soc_stop_capture,
    pm_management:       camera_func_mi2010soc_pm_management
};

static void mi2010soc_init(int dma_en)
{
    camera_gpio_init();     // only init GPIO mode

    // Turn on M_CLK
    ci_enable(dma_en);

    udelay(1);              // delay 1 us

    // to exit hardware standby, hard standby should be low and clkin should be on first

    GPCR(GPIO_CAM_EN)  = GPIO_bit(GPIO_CAM_EN);    /*set STANDBY to low*/

}

static void mi2010soc_hw_deinit(void)
{
#if HARDWARE_STANDBY
    GPSR(GPIO_CAM_EN) = GPIO_bit(GPIO_CAM_EN);      // set STANDBY to HIGH
#endif
    // Wait for 24 CLKIN after set STANDBY to hight
    udelay(200); /* manual says need 24 CLKIN but 20, 50 or 100 does not work */

    /* disable CI, turn off MCLK */
    ci_disable(1);

    ci_deinit();

    set_GPIO_mode( GPIO_CIF_MCLK); /*trun off MCLK*/
    GPCR(GPIO_CIF_MCLK) = GPIO_bit(GPIO_CIF_MCLK);   /* Set GPIO_CIF_MCLK to low */

}

static void mi2010soc_deinit(void)
{
    mi2010soc_standby_enter();

    i2c_mi2010soc_cleanup();

    mi2010soc_hw_deinit();

}

int camera_func_mi2010soc_standby(void)
{
    int ret;
    u16 device_id = 0 ;
    u16 revision = 0 ;
    // init context status
    ddbg_print("Micron MI2010SOC standby!");

    ci_init();

    // enable master clock
    ci_set_clock(0, 0, 1, MCLK_DEFT);

    camera_gpio_init();     // only init GPIO mode

    GPCR(GPIO_CAM_EN)  = GPIO_bit(GPIO_CAM_EN);    /*set STANDBY to low*/
    GPCR(GPIO_CAM_RST) = GPIO_bit(GPIO_CAM_RST);   /*set RESET to low*/

#if HARDWARE_STANDBY
    PGSR(GPIO_CAM_EN) |= GPIO_bit(GPIO_CAM_EN);    /*set suspend status to high*/
#endif
    PGSR(GPIO_CAM_RST)|= GPIO_bit(GPIO_CAM_RST);   /*set suspend status to high*/

    // Turn on M_CLK
    ci_enable(0);

    // For reset to take place, hard standby should be low and clkin should be on
    udelay(10);              // delay 10 us
    GPSR(GPIO_CAM_RST) = GPIO_bit(GPIO_CAM_RST);   /*set RESET to high*/

    // wait 10 CLK after reset
    udelay(2);

    // write register 
    
    ret = i2c_mi2010soc_init();
    if(ret >= 0)
    {
        mi2010soc_get_device_id(&device_id, &revision);
        if(device_id != 0x1519) 
        {
            err_print("error: unknown device id %04x!", device_id);
        }
        else
        {
            ddbg_print("Loading patch");
            mi2010soc_reset_init();
            ddbg_print("Enter standby");
            mi2010soc_standby_enter();
        }
    }
    else
        err_print("error: i2c initialize fail!");
    i2c_mi2010soc_cleanup();

    mi2010soc_hw_deinit();

    return ret;
}

int camera_func_mi2010soc_init(  p_camera_context_t cam_ctx )
{
    u16 device_id = 0 ;
    u16 revision = 0 ;
    int ret;

    // init context status for sensor
    
    cam_ctx->sensor_width  = MAX_WIDTH;
    cam_ctx->sensor_height = MAX_HEIGHT;
    
    cam_ctx->capture_width  = DEFT_SENSOR_A_WIDTH;
    cam_ctx->capture_height = DEFT_SENSOR_A_HEIGHT;
    
    cam_ctx->still_width  = MAX_WIDTH;
    cam_ctx->still_height = MAX_HEIGHT;
    
    cam_ctx->frame_rate = cam_ctx->fps = FRAMERATE_DEFT;
    cam_ctx->mini_fps = FRAMERATE_DEFT;
    
    cam_ctx->buf_size     = BUF_SIZE_DEFT;
    cam_ctx->dma_descriptors_size = (cam_ctx->buf_size/PAGE_SIZE + 10);
    strcpy (cam_ctx->vc.name, "Micron MI2010SOC");
    cam_ctx->vc.maxwidth  = MAX_WIDTH;
    cam_ctx->vc.maxheight = MAX_HEIGHT;
    cam_ctx->vc.minwidth  = MIN_WIDTH; 
    cam_ctx->vc.minheight = MIN_HEIGHT;

    ci_init();
 
    // Configure CI according to MI2010SOC's hardware
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

    // data sample on falling and h,vsync active high
    ci_set_polarity(1, 0, 0);
    
    // fifo control
    ci_set_fifo(0, CI_FIFO_THL_32, 1, 1); // quality

    mi2010soc_init(1);

    ret = i2c_mi2010soc_init();
    if(ret < 0)
    {   // i2c failed
        err_print("error: i2c initialize fail!");
        mi2010soc_hw_deinit();
        return ret;
    }

    // read out device id
    mi2010soc_get_device_id(&device_id, &revision);
    if(device_id != 0x1519) 
    {
        err_print("error: unknown device id %04x!", device_id);
        //ci_disable(1);
        //camera_gpio_deinit();
        //return -ENOTSUPP;
    }
    dbg_print("Micron MI2010SOC device id is %04x, rev%d!", device_id, revision);

    cam_ctx->sensor_type = CAMERA_TYPE_MI2010SOC;

    mi2010soc_standby_exit();

    mi2010soc_default_settings();

    ddbg_print("mi2010soc init success!");

    return 0;
}
    
int camera_func_mi2010soc_deinit(  p_camera_context_t camera_context )
{
    mi2010soc_deinit();

    return 0;
}

#define ASPECT_FLOAT_MULTIPLE   10000

int camera_func_mi2010soc_set_capture_format(  p_camera_context_t camera_context )
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
        if(swidth < camera_context->capture_width)
        {
            swidth = camera_context->capture_width;
            azoom = camera_context->sensor_width * CAMERA_ZOOM_LEVEL_MULTIPLE / swidth;
            camera_context->capture_digital_zoom = azoom;
        }
        if(sheight < camera_context->capture_height)
        {
            sheight = camera_context->capture_height;
        }
    }

    // set sensor input window
    wsize.width = swidth;
    wsize.height = sheight;
    mi2010soc_sensor_size(&wsize);
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
    mi2010soc_capture_sensor_size(&wsize);
    ddbg_print("photo mode: zoom %d, sensor w:h %d:%d", bzoom, swidth, sheight);

    // set sensor output window
    wsize.width = camera_context->capture_width;
    wsize.height = camera_context->capture_height;
    mi2010soc_output_size(&wsize);

    wsize.width = camera_context->still_width;
    wsize.height = camera_context->still_height;
    mi2010soc_capture_size(&wsize);

    // set sensor preview format
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
    case CAMERA_IMAGE_FORMAT_JPEG:
    case CAMERA_IMAGE_FORMAT_JPEG_MICRON:
        micron_format = O_FORMAT_JPEG;
        break;
    default:
        micron_format = O_FORMAT_422_YCbYCr;
        break;
    }
    mi2010soc_output_format(micron_format);

    camera_context->spoof_mode = 0;
    // set sensor capture format
    switch(camera_context->still_input_format) {
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
    case CAMERA_IMAGE_FORMAT_JPEG:
    case CAMERA_IMAGE_FORMAT_JPEG_MICRON:
        micron_format = O_FORMAT_JPEG;
        camera_context->spoof_mode = 1;
        break;
    default:
        micron_format = O_FORMAT_422_YCbYCr;
        break;
    }
    mi2010soc_capture_format(micron_format);

    if(camera_context->spoof_mode)
    {
        wsize.width = camera_context->still_width;
        wsize.height = camera_context->still_height * 3 / 4;
        wsize.width &= 0xffffe0;

        mi2010soc_set_spoof_size(&wsize);
        ddbg_print("spoof size %d:%d", wsize.width, wsize.height);

        camera_context->spoof_width = wsize.width;
        camera_context->spoof_height = wsize.height;
    }

    return 0;
}

int camera_func_mi2010soc_start_capture(  p_camera_context_t cam_ctx, unsigned int frames )
{
    int waitingFrame = 0;

    ci_clear_int_status(0xFFFFFFFF);

    // frames=0 means video mode capture    
    if (frames == 0) 
    {
        ddbg_print("video capture!"); 
        mi2010soc_viewfinder_on();
    }
    else 
    {
        ddbg_print("still capture");
        mi2010soc_snapshot_trigger();
    }

    if(frames == 1) //Wait 1 frames to begin capture photo
    {
        waitingFrame = 0;
    } 
    else
    { 
        waitingFrame = 0;
    }
    camera_skip_frame(cam_ctx, waitingFrame);

    return 0;
}

int camera_func_mi2010soc_stop_capture(  p_camera_context_t camera_context )
{
    mi2010soc_viewfinder_off();
    stop_dma_transfer(camera_context);

    return 0;
}

int camera_func_mi2010soc_pm_management(p_camera_context_t cam_ctx, int suspend)
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
    reg_value = (int)mi2010soc_reg_read((u16)offset);

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
    mi2010soc_reg_write((u16)reg_s.val1, (u16)reg_s.val2);
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
    mi2010soc_set_fps(cam_ctx->fps);
    return 0;
}


/*Set  sensor size*/  
static int pxa_cam_WCAM_VIDIOCSSSIZE(p_camera_context_t cam_ctx, void * param)
{
  micron_window_size size;
  ddbg_print("WCAM_VIDIOCSSSIZE");
  
  if(copy_from_user(&size, param, sizeof(micron_window_size))) 
  {
        return  -EFAULT;
  }
  
  size.width = (size.width+3)/4 * 4;
  size.height = (size.height+3)/4 * 4;
  cam_ctx->sensor_width = size.width;
  cam_ctx->sensor_height = size.height;
  //mi2010soc_sensor_size(&size);
  return 0;
}

//set  output size
static int pxa_cam_WCAM_VIDIOCSOSIZE(p_camera_context_t cam_ctx, void * param)
{
   micron_window_size size;
   ddbg_print("WCAM_VIDIOCSOSIZE");
  
   if(copy_from_user(&size, param, sizeof(micron_window_size))) 
   {
        return  -EFAULT;
   }

   //make it in an even number
   size.width = (size.width+1)/2 * 2;
   size.height = (size.height+1)/2 * 2;
   //mi2010soc_output_size(&size);
   
   cam_ctx->capture_width  = size.width;
   cam_ctx->capture_height = size.height;
   
   ddbg_print("w:h = %d:%d", cam_ctx->capture_width, cam_ctx->capture_height);
   
   return 0;
}
    
/*set picture style*/  
static int pxa_cam_WCAM_VIDIOCSSTYLE(p_camera_context_t cam_ctx, void * param)
{
  ddbg_print("WCAM_VIDIOCSSTYLE");
  cam_ctx->capture_style = (V4l_PIC_STYLE)param;
  
  return mi2010soc_set_style(cam_ctx->capture_style);
}

/*set picture light*/     
static int pxa_cam_WCAM_VIDIOCSLIGHT(p_camera_context_t cam_ctx, void * param)
{
   ddbg_print("WCAM_VIDIOCSLIGHT");
   cam_ctx->capture_light = (V4l_PIC_WB)param;

   return  mi2010soc_set_light((V4l_PIC_WB)param);
}
    
/*set picture brightness*/
static int pxa_cam_WCAM_VIDIOCSBRIGHT(p_camera_context_t cam_ctx, void * param)
{
   ddbg_print("WCAM_VIDIOCSBRIGHT");
   cam_ctx->capture_bright = (int)param;

   return  mi2010soc_set_bright((int)param);
}

/*set picture contrast*/
/*static int pxa_cam_WCAM_VIDIOCSCONTRAST(p_camera_context_t cam_ctx, void * param)
{
   ddbg_print("WCAM_VIDIOCSCONTRAST");
   cam_ctx->capture_contrast = ((int)param-50)/12;

   return  mi2010soc_set_contrast(cam_ctx->capture_contrast);
}*/

/*set flicker frequency*/
static int pxa_cam_WCAM_VIDIOCSFLICKER(p_camera_context_t cam_ctx, void * param)
{
   ddbg_print("WCAM_VIDIOCSFLICKER");
   cam_ctx->flicker_freq = (int)param;

   return  mi2010soc_set_flicker(cam_ctx->flicker_freq);
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
            mi2010soc_set_exposure_mode(cam_mode.mode, maxexpotime);
            break;
        default:
            return -EFAULT;
    }

    return 0;
}

//static int mi2010soc_get_exposure_para(struct V4l_EXPOSURE_PARA *expo_para);

static int pxa_cam_WCAM_VIDIOCGEXPOPARA(p_camera_context_t cam_ctx, void * param)
{
    ddbg_print("WCAM_VIDIOCGEXPOPARA");
    struct V4l_EXPOSURE_PARA expo_para;

    //mi2010soc_get_exposure_para(&expo_para);

    if(copy_to_user(param, &expo_para, sizeof(expo_para))) 
    {
        return -EFAULT;
    } 

    return 0;
}

/*set jpeg quality*/
static int mi2010soc_WCAM_VIDIOCSJPEGQUALITY(p_camera_context_t cam_ctx, void * param)
{
   return  mi2010soc_set_jpeg_scale(cam_ctx->jpeg_quality);
}

/*set picture mirroring*/
static int mi2010soc_WCAM_VIDIOCSMIRROR(p_camera_context_t cam_ctx, void * param)
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

    return  mi2010soc_set_mirror(rows, columns);
}


int camera_func_mi2010soc_docommand(p_camera_context_t cam_ctx, unsigned int cmd, void *param)
{
   switch(cmd)
   {
     /*read mi2010soc registers*/
    case WCAM_VIDIOCGCAMREG:
         return pxa_camera_WCAM_VIDIOCGCAMREG(cam_ctx, param);

    /*write mi2010soc registers*/
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
         return mi2010soc_WCAM_VIDIOCSMIRROR(cam_ctx, param);

    case WCAM_VIDIOCSJPEGQUALITY:
         return mi2010soc_WCAM_VIDIOCSJPEGQUALITY(cam_ctx, param);

    default:
         {
           err_print("Error cmd=%d", cmd);
           return -1;
         }
    }
    return 0;
 
}

