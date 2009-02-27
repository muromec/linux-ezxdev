/*
 *  mt9v111.c
 *
 *  Micron MT9V111 VGA Camera Module driver.
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
wangfei(w20239)      04/14/2003                 Created   

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

#include "adcm2700.h"
#include "adcm2700_hw.h"

#define ADDRSPACE_CORE          0x04              /* code to select core registers 0-170*/
#define ADDRSPACE_IFP           0x01              /* code to select IFP/SOC registers 0-255*/
#define MT9V111_SENSOR_WIDTH    640
#define MT9V111_SENSOR_HEIGHT   480

extern int mt9v111_write(u8 addrSpace, u16 addr, u16 value);
extern int mt9v111_read(u8 addrSpace, u16 addr);
extern int i2c_mt9v111_cleanup(void);
extern int camera_ring_buf_init(p_camera_context_t);
extern void adcm2700_wait(u32);
extern void start_dma_transfer(p_camera_context_t,unsigned);
extern void stop_dma_transfer(p_camera_context_t);
extern int camera_func_mt9v111_stop_capture(p_camera_context_t camera_context);

static int mt9v111_restore_property(p_camera_context_t);

static inline int mt9v111_read_core_reg(u16 addr)
{
    return mt9v111_read(ADDRSPACE_CORE, addr);
}

static inline int mt9v111_read_ifp_reg(u16 addr)
{
    return mt9v111_read(ADDRSPACE_IFP, addr);
}
static inline int mt9v111_write_core_reg(u16 addr, u16 value)
{
    return mt9v111_write(ADDRSPACE_CORE, addr, value);
}
static inline int mt9v111_write_ifp_reg(u16 addr, u16 value)
{
    return mt9v111_write(ADDRSPACE_IFP, addr, value);
}
static int mt9v111_input_size(adcm_window_size * size)
{
    u16 pan_ver,pan_hor;
    pan_hor=(MT9V111_SENSOR_WIDTH - size->width)>>1;
    pan_ver=(MT9V111_SENSOR_HEIGHT- size->height)>>1;
    
    mt9v111_write_ifp_reg(0xa5,0x8000|pan_hor);
    mt9v111_write_ifp_reg(0xa8,0x8000|pan_ver);
    mt9v111_write_ifp_reg(0xa6,0x8000|size->width);
    mt9v111_write_ifp_reg(0xa9,size->height);
    dbg_print("set sensor width : %d  height : %d\n",size->width , size->height);
    //dbg_print("horizontal pan :%d  vertical pan %d \n",pan_hor,pan_ver);
    return 0;
}
static int mt9v111_set_fps(int fps, int minfps)
{    
    //write to r55 (0x37) [9..5] to change minimum fps
    u16 blank_v,r55,zone_low,zone_high;
    dbg_print("set fps : %d min fps : %d\n",fps,minfps);
    
    //Set vertical blank to adjust frame rate
    blank_v = (15-fps)*496/fps + 4;
    mt9v111_write_core_reg(0x06,blank_v);
    
    //shut width
    zone_low=0;
    zone_high=fps*8/minfps;
    r55 = zone_low|(zone_high<<5);
    mt9v111_write_ifp_reg(0x37, r55);
    
    return 0;
}
static int mt9v111_set_bright(int bright)
{
    const u16 target[]=
    {
        25,
        35,
        50,
        75,
        100,
        120,
        140,
        160,
        180
    };
    if(bright > 4|| bright < -4)
    {
        return -EFAULT;
    }
    mt9v111_write_ifp_reg(0x2e,(16<<8)|target[bright+4]);
    return 0;
}
static int mt9v111_output_size(adcm_window_size * size)
{
    //0xA7  Horizontal size of output 
    //0xAA  Vertical size of output 
    //refer to mt9v111-datasheet.pdf page 45 and page 46
    mt9v111_write_ifp_reg(0xA7, 0x8000|size->width);
    mt9v111_write_ifp_reg(0xAA, size->height);
    dbg_print("output size :%d * %d \n",size->width,size->height);
    adcm2700_wait(50);
    return 0;
}
static int mt9v111_set_blackwhite(int black_white)
{
    u16 value = mt9v111_read_ifp_reg(0x08);
    if(black_white)
    {
        value |= 1<<5;
    }
    else
    {
        value &= ~(1<<5);
    }
    
    mt9v111_write_ifp_reg(0x08, value);
    return 0;
}
//set color correction matrix
static int mt9v111_set_ccm(int index)
{
    int i;
    u16 r6;
    const u16 regs[]=
    {
        72,                                  //digital gain
        2,                                   //sign
        3,4,                                 //scale
        9,10,11,12,13,14,15,16,17,           //magnitude
        21,22,23,24,25,26,27,28,29,30,       //delta matrix
        94,95,96
    };
    const u16 value[][26]=
    {
        //default
        {
            
            0,
            0x6e,
            0x2923,0x0524,                 
            146,22,8,171,147,88,77,169,160,
            373,22,67,12,0,21,31,22,152,76,
            26684,12296,2
           /*    
            0,
            110,
            10531,1316,
            125,108,58,129,141,112,95,128,138,
            145,6,231,221,15,65,208,72,59,43,
            26953,11283,2
           */
        },
        //direct sun
        {
        /*
            128,
            0xea,
            0x3922,0x4e4,
            0x8d,0x71,0xc5,0x9d,0xf8,0x53,0x0d,0xa7,0x90,
            0,0,0,0,0,0,0,0,0,0
        */
            128,
            174,
            14627,1828,
            128,7,52,122,215,7,59,99,232,
            0,0,0,0,0,0,0,0,0,0,
            21331,0,0

        },
        //incandescent
        {
        /*
            128,
            0xee,
            0x2922,0x4a4,
            0x8c,0xcc,0x6e,0xa6,0x86,0x72,0x06,0x85,0xc6,
            0,0,0,0,0,0,0,0,0,0
        */
            128,
            110,
            10532,1316,
            245,244,168,137,174,216,131,158,160,
            0,0,0,0,0,0,0,0,0,0,
            32832,0,0
        },
        //fluorescent
        {
            /*
            0,
            0xea,
            0x3922,0x4e4,
            0x8d,0xba,0x7b,0x9c,0xfe,0x61,0x08,0xb9,0x9a,
            0,0,0,0,0,0,0,0,0,0
            */
            128,
            166,
            10531,1316,
            197,160,77,230,132,3,122,91,131,
            0,0,0,0,0,0,0,0,0,0,
            32844,0,0
       
        },
        //sepia
        {
            128,
            36,
            10516,2276,
            9,142,17,24,134,43,10,147,17,
            0,0,0,0,0,0,0,0,0,0,
            20560,0,0
        },
    };
    
    for(i=0; i<26; i++)
    {
        mt9v111_write_ifp_reg(regs[i],value[index][i]);
    }
    
    r6=mt9v111_read_ifp_reg(0x06);
    r6 |= 1<<15;
    mt9v111_write_ifp_reg(0x06,r6);
    r6 &= ~(1<<15);
    mt9v111_write_ifp_reg(0x06,r6);
    return 0;
}
static int mt9v111_set_gamma(int index)
{
    const u16 regs[]=
    {
        0x58,0x53,0x54,0x55,0x56,0x57,
    };
    const u16 value[][11]=
    {
        //default
        {
            //0x0,0x09,0x0f,0x1b,0x2f,0x52,0x71,0x8f,0xab,0xc6,0xe0
            0,10,18,31,54,94,130,164,196,226,255
        },
        //solarize
        {
           // 224,23,17,128,220,175,105,53,33,8,0
           0,4,8,16,32,64,96,128,160,192,224
        },
        //neg art
        {
           // 224,204,194,178,174,118,89,64,41,20,0
           0,4,8,16,32,64,96,128,160,192,224
        }
    };
    int i;
    dbg_print("mt9v111_set_gamma to index :%d \n",index);
    mt9v111_write_ifp_reg(regs[0],value[index][0]);
    for(i=0; i<5; i++)
    {
        mt9v111_write_ifp_reg(regs[i+1],value[index][2*i+1]|(value[index][2*i+2]<<8));
    }
    return 0;
}
static int mt9v111_set_light(V4l_PIC_WB light)
{
    int index;
    switch(light)
    {
        case V4l_WB_DIRECT_SUN:
            index = 1;
            break;
        case V4l_WB_INCANDESCENT:
            index = 2;
            break;
        case V4l_WB_FLUORESCENT:
            index = 3;
            break;
        case V4l_WB_AUTO:
            index = 0;
            break;    
        default:
            dbg_print("incorrect light type!\n");
            return -EFAULT;
    }
    return mt9v111_set_ccm(index);
}
static int mt9v111_set_style(V4l_PIC_STYLE style)
{
    if(style != V4l_STYLE_BLACK_WHITE)
    {      
        mt9v111_set_blackwhite(0);  
    }
    switch(style)
    {
    case V4l_STYLE_BLACK_WHITE:
        mt9v111_set_gamma(0);
        mt9v111_set_blackwhite(1);  
        break;
    case V4l_STYLE_SEPIA:
        mt9v111_set_gamma(0);
        mt9v111_set_ccm(4);
        break;
    case V4l_STYLE_SOLARIZE:
        mt9v111_set_gamma(1);
        break;
    case V4l_STYLE_NEG_ART:
        mt9v111_set_gamma(2);
        break;
    default:
        mt9v111_set_gamma(0);
        break;
    }
    return 0;
}

static inline int mt9v111_set_effects(p_camera_context_t cam_ctx)
{
    mt9v111_set_light(cam_ctx->capture_light);
    mt9v111_set_style(cam_ctx->capture_style);
    return 0;
}
static int mt9v111_reset(void)
{
   //stop and reset
    mt9v111_write_core_reg(0x07, 0x0);
    adcm2700_wait(20);
    mt9v111_write_core_reg(0x0D, 0x1);
    adcm2700_wait(5);
    mt9v111_write_core_reg(0x0D, 0x0);
    adcm2700_wait(5);
    mt9v111_write_ifp_reg(0x07, 0x1);
    adcm2700_wait(5);
    mt9v111_write_ifp_reg(0x07, 0x0);

    return 0;
}
static int mt9v111_set_flicker(int flicker)
{
/*
    if (flicker == 50)
    {
        mt9v111_write(0x5b,0x01);
    }
    else if (flicker ==60)
    {
        mt9v111_write(0x5b,0x03);
    }
    else
    {
        return -EFAULT;
    }
*/
    return 0;
}
/***********************************************************************
 *
 * MT9V111 Functions
 *
 ***********************************************************************/
int camera_func_mt9v111_init( p_camera_context_t cam_ctx )
{
    u16 addr;
    u16 reg_value;
    const u16 values2[] = 
    {
        0x0006, 0xCE12, 0x00F0, 0x370D, 0xE20C, 0xFDF5,
        0x280B, 0xDE0D, 0x00F4, 0x280A, 0xBD20, 0xF8E4,
        0x1E0A, 0x0050, 0xD812, 0xFCEC, 0x1407, 0x0028,
        0xD413, 0xFEEC, 0x1405, 0x0028
    };
    
    ci_disable(1);
    cam_ctx->mclk=12;
    ci_set_clock(cam_ctx->clk_reg_base, 1, 1, cam_ctx->mclk);
    ci_enable(1);
    adcm2700_wait(10);
    mt9v111_reset();
    /* lens shading */
    for(addr = 0x80; addr <= 0x95; addr++)
    {
        mt9v111_write_ifp_reg(addr, values2[addr-0x80]);
    }
    //accelerate AE speed
    //mt9v111_write_ifp_reg(0x2f,0x80);
    /* 100% sharpening */
    //mt9v111_write_ifp_reg(0x05,0x0c);
//codes from jason
    //lens shading enable
    //*
    reg_value = mt9v111_read_ifp_reg(0x08);
    reg_value |= 0x0100;
    mt9v111_write_ifp_reg(0x08,reg_value);
    //*/
    mt9v111_write_core_reg(0x09,280);
    mt9v111_write_core_reg(0x21,0xe401);
    reg_value = mt9v111_read_core_reg(0x07);
    reg_value &= ~(0x0010);
    mt9v111_write_core_reg(0x07,reg_value);
    mt9v111_write_core_reg(0x2f,0xe7b6);
    mt9v111_write_ifp_reg(0x39,282);
    mt9v111_write_ifp_reg(0x59,280);
    mt9v111_write_ifp_reg(0x5a,336);
    mt9v111_write_ifp_reg(0x5c,4624);
    mt9v111_write_ifp_reg(0x5d,5652);
    mt9v111_write_ifp_reg(0x25,0x4514);
    //mt9v111_write_ifp_reg(0x25,0x6d14);
    mt9v111_write_ifp_reg(0x34,0x0);
    mt9v111_write_ifp_reg(0x35,0xff00);
    mt9v111_write_ifp_reg(0x33,0x1411);
    mt9v111_write_ifp_reg(0x3e,0x0fff);
    mt9v111_write_ifp_reg(0x3b,0x042c);
    mt9v111_write_ifp_reg(0x3d,0x01da);
    mt9v111_write_ifp_reg(0x38,0x0878);
    mt9v111_write_ifp_reg(0x40,0x1e10);
    mt9v111_write_ifp_reg(0x41,0x1417);
    mt9v111_write_ifp_reg(0x42,0x1213);
    mt9v111_write_ifp_reg(0x43,0x1112);
    mt9v111_write_ifp_reg(0x44,0x7110);
    mt9v111_write_ifp_reg(0x45,0x7473);
    
    dbg_print("Mt9v111 init success!");
    return 0;
}

int camera_func_mt9v111_deinit(  p_camera_context_t camera_context )
{
    //stop camera run
    camera_func_mt9v111_stop_capture(camera_context);
    ci_disable(1);
    i2c_mt9v111_cleanup();
    return 0;
}

int camera_func_mt9v111_set_capture_format(p_camera_context_t camera_context)
{
    adcm_window_size wsize;
    u16 fmt_v1, fmt_v2;
    //set sensor input/output window
    wsize.width = camera_context->capture_width;
    wsize.height = camera_context->capture_height;
    mt9v111_output_size(&wsize);
    dbg_print("output size width=%d, height=%d", wsize.width, wsize.height);
    /*
    0x08 output format contorl     bit 12 
                                        1 RGB
                                        0 YCbCr
    0x3A output format control 2   bit 6-7
                                        00 16-bit RGB565
                                        01 15-bit RGB555
                                        10 12-bit RGB444x
                                        11 12-bit RGB444
    refer to mt9v111-datasheet.pdf page 32 and page 39
    */
    u16 old1 = mt9v111_read_ifp_reg(0x08);
    u16 old2 = mt9v111_read_ifp_reg(0x3A);
    
    // set sensor format
    switch(camera_context->capture_input_format) 
    {
      case CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR:
      case CAMERA_IMAGE_FORMAT_YCBCR422_PACKED:
           dbg_print("output format YCBCR422");
           fmt_v1 = 0;
           fmt_v2 = 0;
           break;
        
      case CAMERA_IMAGE_FORMAT_RGB565:
          dbg_print("output format RGB565");
          fmt_v1 = (1<<12);
          fmt_v2 = 0;
          break;
        
     case CAMERA_IMAGE_FORMAT_RGB444:
          dbg_print("output format RGB444");
          fmt_v1 = (1<<12);
          fmt_v2 = (0x03<<6);
          break;
    
     case CAMERA_IMAGE_FORMAT_RGB555:
          dbg_print("output format RGB555");
          fmt_v1 = (1<<12);
          fmt_v2 = (0x01<<6);
          break;
        
     default:
          dbg_print("unsupported format!");
          return -EFAULT;
    }
    
    old1 = (old1 & ~(1<<12)) | fmt_v1;
    old2 = ((old2 & ~(3<<6)) & (~0x03)) | fmt_v2; // | (2<<3)/*out test ramp*/;
    mt9v111_write_ifp_reg(0x08, old1);
    mt9v111_write_ifp_reg(0x3A, old2);
    return 0;
}

int camera_func_mt9v111_start_capture(p_camera_context_t cam_ctx, unsigned int frames)
{   
    int cisr;
    int wait_count = 1;
    unsigned int start_time;
    unsigned int flags;
    int error_count = 0;
    //mt9v111_restore_property(cam_ctx);
restart:    
    start_time = 0x1000000;
    if(frames == 1)
    {
        wait_count = 2;
    }
    //mt9v111_write_core_reg(0x07, 0x0002);
    dbg_print("wait for EOF %d time", wait_count);
    ci_disable(1);
    ci_enable(1);

    start_time *= wait_count;
    CISR|=CI_CISR_EOF;
    cisr=CISR;
    //local_irq_save(flags);
    //mdelay(200);
    while(wait_count)
    {
        if(cisr & CI_CISR_EOF)
        {
            wait_count -- ;
            CISR|=CI_CISR_EOF;
        }
        cisr=CISR;
        if(!(--start_time))
        {
           goto start_error;
        }
    }
   // ci_clear_int_status(0xFFFFFFFF);
    mdelay(1);
    ci_reset_fifo();
    ci_disable(1);
    ci_enable(1);
    start_dma_transfer(cam_ctx, 0);
    //local_irq_restore(flags);
    return 0;
    
start_error:
    dbg_print("start error \n");
    if(error_count++ > 3)
    {
       return -1;
    }
    ci_disable(1);
    ci_enable(1);
    adcm2700_wait(10);
    mt9v111_reset();
    mt9v111_restore_property(cam_ctx);
    goto restart;
    
}

int camera_func_mt9v111_stop_capture(  p_camera_context_t camera_context )
{
    /*
    u16 old = mt9v111_read_core_reg(0x07);
    old &= ~0x02;
    mt9v111_write_core_reg(0x07, old);
    */
 //   stop_dma_transfer(camera_context);
    return 0;
}
/*set output size*/
static int pxa_cam_WCAM_VIDIOCSOSIZE(p_camera_context_t cam_ctx, void * param)
{
  
   adcm_window_size size;
   CI_MP_TIMING     timing;
   int ret = 0;
   unsigned int  value;
   memset(&timing, 0, sizeof(timing));
   dbg_print("WCAM_VIDIOCSOSIZE");
   if(copy_from_user(&size, param, sizeof(adcm_window_size))) 
   {
        return  -EFAULT;
   }
   size.height = (size.height+7)/8 * 8;
   size.width  = (size.width +7)/8 * 8;
   if(size.height != cam_ctx->capture_height || size.width != cam_ctx->capture_width)
   {
       cam_ctx->capture_width  = size.width;
       cam_ctx->capture_height = size.height;
       ret = mt9v111_output_size(&size);
       timing.BFW = 0;
       timing.BLW = 0;
       ci_disable(1);
       ci_configure_mp(cam_ctx->capture_width-1, cam_ctx->capture_height-1, &timing);
       camera_ring_buf_init(cam_ctx);
       ci_enable(1);
       //mdelay(150);
   }   
   return ret;
}

/*read mt9v111 registers*/
static int pxa_camera_WCAM_VIDIOCGCAMREG(p_camera_context_t cam_ctx, void *param)
{
  dbg_print("");
  int data, reg_value;
  if(copy_from_user(&data, param, sizeof(data))) 
  {
        return  -EFAULT;
  }
  u16 space = (u16)(data>>16);
  u16 addr  = (u16)data;
  
  if(space == ADDRSPACE_CORE)
  {
    reg_value = mt9v111_read_core_reg(addr);
  }
  else if(space == ADDRSPACE_IFP)
  {
    reg_value = mt9v111_read_ifp_reg(addr);
  }
  else
  {
    return -EFAULT;
  }
  
  if(copy_to_user(param, &reg_value, sizeof(int))) 
  {
       return -EFAULT;
  }
  return 0;
}

/*write mt9v111 registers*/
static int pxa_camera_WCAM_VIDIOCSCAMREG(p_camera_context_t cam_ctx, void *param)
{
  dbg_print("");
  
  struct reg_set_s {int  val1, val2;}reg_value;
  
  if(copy_from_user(&reg_value, param, sizeof(reg_value))) 
  {
        return  -EFAULT;
  }
  
  u16 space = (u16)(reg_value.val1 >> 16);
  u16 addr  = (u16)(reg_value.val1);
  
  if(space == ADDRSPACE_CORE)
  {
    mt9v111_write_core_reg(addr, (u16)(reg_value.val2));
  }
  else if(space == ADDRSPACE_IFP)
  {
    mt9v111_write_ifp_reg(addr, (u16)(reg_value.val2));
  }
  else
  {
    return -EFAULT;
  }
  
  return 0;
}

/*set sensor size*/
static int pxa_cam_WCAM_VIDIOCSSSIZE(p_camera_context_t cam_ctx, void *param)
{
  dbg_print("");
  adcm_window_size size;
  int ret = 0;
  if(copy_from_user(&size, param, sizeof(adcm_window_size))) 
  {
        return  -EFAULT;
  }
  size.width  = ((size.width+7)/8) * 8;
  size.height = ((size.height+7)/8) *8;
  if(size.width != cam_ctx->sensor_width || size.height != cam_ctx->sensor_height)
  {
      cam_ctx->sensor_width  = size.width;
      cam_ctx->sensor_height = size.height;
  
      if(size.width > MT9V111_SENSOR_WIDTH || size.height > MT9V111_SENSOR_HEIGHT)
      {
        dbg_print("error input size width %d height %d",size.width, size.height);
        return -EFAULT;
      }
  
      ret = mt9v111_input_size(&size);
  }
  return ret;
}

/*set fps*/
static int pxa_cam_WCAM_VIDIOCSFPS(p_camera_context_t cam_ctx, void *param)
{
  dbg_print("WCAM_VIDIOCSFPS");
  struct {int fps, minfps;} cam_fps;
  int ret = 0;

  if(copy_from_user(&cam_fps, param, sizeof(int) * 2)) 
  {
        return  -EFAULT;
  }
  
  if(cam_fps.minfps < 5)
  {
     cam_fps.minfps = 5;
  }
  if(cam_fps.fps >15)
  {
     cam_fps.fps = 15;
  }
  
  if(cam_fps.fps != cam_ctx->fps || cam_fps.minfps != cam_ctx->mini_fps)
  {
     cam_ctx->fps =  cam_fps.fps;
     cam_ctx->mini_fps = cam_fps.minfps;
     ret = mt9v111_set_fps(cam_ctx->fps, cam_ctx->mini_fps);
  }
  
  return ret;
}
/*set image style*/
static int pxa_cam_WCAM_VIDIOCSSTYLE(p_camera_context_t cam_ctx, void *param)
{
    dbg_print("WCAM_VIDIOCSSTYLE");
    int ret=0;
    if(cam_ctx->capture_style != (V4l_PIC_STYLE)param)
    {
        cam_ctx->capture_style = (V4l_PIC_STYLE)param;
        ret = mt9v111_set_effects(cam_ctx);
    }
    return ret;
}
/*set video light*/
static int pxa_cam_WCAM_VIDIOCSLIGHT(p_camera_context_t cam_ctx, void *param)
{
    dbg_print("WACM_VIDIOCSLIGHT");
    int ret=0;
    if(cam_ctx->capture_light!= (V4l_PIC_WB)param)
    {
        cam_ctx->capture_light=(V4l_PIC_WB)param;
        ret = mt9v111_set_effects(cam_ctx);
    }
    return ret;
}
/*set picture brightness*/
static int pxa_cam_WCAM_VIDIOCSBRIGHT(p_camera_context_t cam_ctx, void *param)
{
    dbg_print("WCAM_VIDIOCSBRIGHT");
    int ret =0;
    if( cam_ctx->capture_bright !=  (int)param)
    {
        cam_ctx->capture_bright = (int)param;
        ret=mt9v111_set_bright(cam_ctx->capture_bright);
    }
    return ret;
}
static int pxa_cam_WCAM_VIDIOCSFLICKER(p_camera_context_t cam_ctx, void * param)
{
   dbg_print("WCAM_VIDIOCSFLICKER");
   cam_ctx->flicker_freq = (int)param;
   return  mt9v111_set_flicker(cam_ctx->flicker_freq);
}
static int mt9v111_restore_property(p_camera_context_t cam_ctx)
{    
    adcm_window_size size;
    
    size.width = cam_ctx->sensor_width;
    size.height= cam_ctx->sensor_height;
    mt9v111_input_size(&size);
    
    camera_func_mt9v111_set_capture_format(cam_ctx);
    
    mt9v111_set_fps(cam_ctx->fps,cam_ctx->mini_fps);
    
    mt9v111_set_bright(cam_ctx->capture_bright);
    
    mt9v111_set_effects(cam_ctx);
    
    return 0;
}
int camera_func_mt9v111_pm_management(p_camera_context_t cam_ctx, int suspend)
{
   static int resume_dma = 0;
   if(suspend)
   {
    if(cam_ctx != NULL )
    {
        if(cam_ctx->dma_started)
        {
            dbg_print("camera running, suspended");
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
       if(cam_ctx != NULL)
       {
           dbg_print("camera running, resumed");
           camera_init(cam_ctx);
           mt9v111_restore_property(cam_ctx);
           if(resume_dma == 1)
           {
               camera_start_video_capture(cam_ctx, 0);
               resume_dma = 0;
           }
       }
       enable_irq(IRQ_CAMERA);
   } 
   return  0;
}

int camera_func_mt9v111_docommand(p_camera_context_t cam_ctx, unsigned int cmd, void *param)
{
   switch(cmd)
   {
       /*read mt9v111 registers*/
    case WCAM_VIDIOCGCAMREG:
         return pxa_camera_WCAM_VIDIOCGCAMREG(cam_ctx, param);

        /*write mt9v111 registers*/
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
           return -EFAULT;
         }
    }
  return 0;

}
