/*
 *  adcm2700_hw.c
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
wangfei(w20239)     12/15/2003      LIBdd35749    Created   

wangfei(w20239)     02/05/2004      LIBdd74309    Set frame rate in video mode

wangfei(w20239)     02/26/2004      LIBdd81055    New chip id support
                                                  Update algorithm for DMA transfer
                                                  Update strategy for memory management
                                                  Fix still picture capture failed sometime
                                                  New Agilent sensor chip ID support
                                                  Make output height in an even multiple of 8
                                                 
wangfei(w20239)     03/08/2004      LIBdd84578    Photo effects setting

*/

/*================================================================================
                                 INCLUDE FILES
==================================================================================*/
#include <linux/types.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/dma.h>
#include <asm/irq.h>

#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/wrapper.h>
#include <linux/delay.h>
#include <linux/i2c.h> 

#define  MAX_FPS  20

#include "adcm2700_hw.h"
#include "camera.h"


extern int adcm2700_read(u16 addr, u16 *pvalue);
extern int adcm2700_write(u16 addr, u16 value);
extern int adcm2700_read_byte(unsigned short addr, unsigned char * value);


 
#define wait_sreg_update()  { dbg_print("wait for sensor update simple registers"); \
                              adcm2700_write(SREG_CONTROL,   0x05);                 \
                              int   retry = adcm2700__TIMEOUT<<2;                   \
                              while(--retry)                                        \
                              {                                                     \
                                u16 v = adcm2700_reg_read(SREG_CONTROL);            \
                                mdelay(1);                                          \
                                if(v == 0xFF)  break;                               \
                                if(!(v & 0x04)) break;                              \
                              }                                                     \
                              dbg_print("retry = %d", retry);                       \
                            }
 
void adcm2700_wait(u32 ms)
{
    if(ms > 10)
    {
     set_current_state(TASK_INTERRUPTIBLE);
     schedule_timeout(ms/10);
    }
}

/****************************************************************************
*									                                        *
*        I2C Management		 					                            *
*									                                        *
*****************************************************************************/
u16 adcm2700_reg_read(u16 reg_addr)
{
    u16 value;
    int ret = adcm2700_read(reg_addr, &value);
    if(ret < 0)
    {
      return 0xFF;
    }
    return value;
}

void adcm2700_reg_write(u16 reg_addr, u16 reg_value)
{
 
    /*experts registers or SREG_CONTROL write directly*/
    if(reg_addr > 0x26 || reg_addr == SREG_CONTROL)
    {
      adcm2700_write(reg_addr, reg_value);
      return; 
    }

    /* write sample control register step:
    1 Stop the camera ¨C write 0x0000 to the CONTROL register.
    2 Change the relevant register. 
    3 Set the CONFIG bit in the CONTROL register, write 0x0004.
    4 Wait for the CONFIG bit in CONTROL to clear. */
    adcm2700_write(SREG_CONTROL, 0);
    adcm2700_write(reg_addr, reg_value);
    wait_sreg_update();
    adcm2700_write(SREG_CONTROL, 0x01);
  
}
 
/*restore capture property*/
int adcm2700_restore_property(p_camera_context_t cam_ctx, int frames)
{
    dbg_print("sensor width %d", cam_ctx->sensor_width);
    dbg_print("sensor height %d", cam_ctx->sensor_height);
    dbg_print("capture_width %d", cam_ctx->capture_width);
    dbg_print("capture_height %d", cam_ctx->capture_height);
    ddbg_print("fps %d mini fps %d", cam_ctx->fps, cam_ctx->mini_fps);
    ddbg_print("light %d", cam_ctx->capture_light);
    ddbg_print("capture_bright %d", cam_ctx->capture_bright);
    ddbg_print("capture_style %d", cam_ctx->capture_style);


    u16 awb_gain_grn1 = adcm2700_reg_read(EREG_APS_COEF_GRN1);
    u16 awb_gain_grn2 = adcm2700_reg_read(EREG_APS_COEF_GRN2);
    u16 awb_gain_red  = adcm2700_reg_read(EREG_APS_COEF_RED);
    u16 awb_gain_blue = adcm2700_reg_read(EREG_APS_COEF_BLUE);
    
    u8 ae_gain1, ae_gain2, ae_gain3, ae_gain4;
    adcm2700_read_byte(0x080F, &ae_gain1);
    adcm2700_read_byte(0x0810, &ae_gain2);
    adcm2700_read_byte(0x0811, &ae_gain3);
    adcm2700_read_byte(0x0812, &ae_gain4);
    
    u8 r813, r814, r815, r80e;
    adcm2700_read_byte(0x813, &r813);
    adcm2700_read_byte(0x814, &r814);
    adcm2700_read_byte(0x815, &r815);
    adcm2700_read_byte(0x80e, &r80e);
    dbg_print("r813= %x, r814=%x r815 = %x", r813, r814, r815);
    
    u16 r80, r84, rp;
    adcm2700_read(0x80, &r80);
    adcm2700_read(0x84, &r84);
    adcm2700_read(0x114, &rp);
    u16 clk_div = r80+1;
    u16 sen_clk_div = (r84 == 0) ? 1 : (r84<<1);
    u32 texp = ((((u32)r814)<<8) + r813) * rp + 4 * r815;
    texp *= clk_div * sen_clk_div * r80e;
    dbg_print("before rp = 0x%x r80 = 0x%x r84 = 0x%x", rp, r80, r84);
    dbg_print("before clk_div = 0x%x sen_clk_div = 0x%x r80e = 0x%x", clk_div, sen_clk_div, r80e);
       //update simple registers
    adcm2700_write(SREG_CONTROL, 0);
    //sensor width and sensor height
    adcm2700_write(SREG_SIZE, 0x707);  
    adcm2700_write(SREG_SENSOR_WID_V, (u16)(cam_ctx->sensor_width));
    adcm2700_write(SREG_SENSOR_HGT_V, (u16)(cam_ctx->sensor_height));

    //output width and output height
    adcm2700_write(SREG_OUTPUT_WID_V, (u16)(cam_ctx->capture_width));
    adcm2700_write(SREG_OUTPUT_HGT_V, (u16)(cam_ctx->capture_height));

    //default fps
    adcm2700_write(SREG_FRAME_RATE, cam_ctx->fps*10);
    wait_sreg_update();
    adcm2700_set_light(cam_ctx->capture_light);
    adcm2700_write(EREG_AE_ETIME_MAX, 100000/(cam_ctx->mini_fps));
       
      //still mode and not night mode
    if(frames == 1)
    {
        adcm2700_write(EREG_AF_CTRL1, 0x10);
        adcm2700_read_byte(0x80e, &r80e);
        adcm2700_read(0x80, &r80);
        adcm2700_read(0x84, &r84);
        adcm2700_read(0x114, &rp);
        clk_div = r80+1;
        sen_clk_div = (r84 == 0) ? 1 : (r84<<1);
        texp /= (clk_div * sen_clk_div * r80e);
        u16 ttexp = (texp/rp);
        r815 = (u8)((texp - ttexp * rp)>>2);
        r813 = (u8)(ttexp & 0xFF);
        r814 = (u8)(ttexp >> 8);
        dbg_print("after r813= %x, r814=%x r815 = %x", r813, r814, r815);
        adcm2700_write_byte(0x813, r813);
        adcm2700_write_byte(0x814, r814);
        adcm2700_write_byte(0x815, r815);
        dbg_print("after rp = 0x%x r80 = 0x%x r84 = 0x%x", rp, r80, r84);
        dbg_print("after clk_div = 0x%x sen_clk_div = 0x%x r80e = 0x%x", clk_div, sen_clk_div, r80e);

        
        adcm2700_write_byte(0x080F, ae_gain1);
        adcm2700_write_byte(0x0810, ae_gain2);
        adcm2700_write_byte(0x0811, ae_gain3);
        adcm2700_write_byte(0x0812, ae_gain4);
 
        adcm2700_write(EREG_APS_COEF_GRN1, awb_gain_grn1);
        adcm2700_write(EREG_APS_COEF_GRN2, awb_gain_grn2);
        adcm2700_write(EREG_APS_COEF_RED,  awb_gain_red);
        adcm2700_write(EREG_APS_COEF_BLUE, awb_gain_blue);
        
    }
    else
    {

    }
   
           
    //expert registers
    adcm2700_set_bright(cam_ctx->capture_bright);

    if(cam_ctx->capture_style == V4l_STYLE_NORMAL && frames == 1)
    {
         adcm2700_set_gamma(3);
    }
    else
    {
        adcm2700_set_style(cam_ctx->capture_style);
    }

    adcm2700_write(SREG_CONTROL, 0x01);
    return ADCM_ERR_NONE;

}


///////////////////////////////////////////////////////////////
//
//   Programming Guide Chapter 1: Basic Programming
//
///////////////////////////////////////////////////////////////

int adcm2700_power_on( u8 clk )
{
    /*Follow these steps to correctly power on the ADCM-2700:
    1 Turn on VCC voltage (2.8 volts) and wait 20 milliseconds.
    2 Turn on MCLK (13 MHz is the default value) and wait 150 milliseconds.
    3 Read register 0x0004; if the return value equals 0x0001, power-up is complete. */
    int   retry = adcm2700__TIMEOUT<<2;
    while(--retry)
    {
        if(adcm2700_reg_read(SREG_STATUS) == 0x0001)
        {
             dbg_print("adcm2700 Power-up complete!!"); 
             break;
        }
        adcm2700_wait(2);
    }
    
 
    
    /* Program the mafster clock */
    adcm2700_master_clock(clk);

    /* Configure  anti-vignetting */
    adcm2700_color_init();
    
    return ADCM_ERR_NONE;
}


int adcm2700_power_off()
{
	/*stop camera*/
	adcm2700_write(SREG_CONTROL, 0);
	return ADCM_ERR_NONE;   
}


/////////////////////////////////////////////////////////////////////////////////////
//   
//  Programming Guide Chapter 2: Configuration Methods 
//
/////////////////////////////////////////////////////////////////////////////////////


int adcm2700_version_revision(u16 * cm_revision, u16 *sensor_revision)
{
    //Camera module version is 0x060
    *cm_revision = adcm2700_reg_read(SREG_ID);
    dbg_print("adcm2700 SREG_ID is 0x%x", *cm_revision);
     
    //Image sensor version is 0x60
    *sensor_revision = adcm2700_reg_read(EREG_IDENT);
    dbg_print("adcm2700 EREG_IDENT is 0x%x", *sensor_revision);       
    return ADCM_ERR_NONE;
}

int adcm2700_viewfinder_on()
{
    //adcm2700_write(SREG_CONTROL, 0x01);
    ddbg_print("camera video mode start!"); 
    return ADCM_ERR_NONE;
}



int adcm2700_viewfinder_off()
{
    //adcm2700_write(SREG_CONTROL, 0);
    ddbg_print("camera video mode stop!");
      // adcm2700_store_af_regs();
    return ADCM_ERR_NONE;
}


int adcm2700_snapshot_trigger()
{
 /*   
    u16 status = adcm2700_reg_read(EREG_AF_STATUS);
    int   retry = adcm2700__TIMEOUT<<1;
    while(--retry)
    {
      if(status & 0x08)
         break;
      status = adcm2700_reg_read(EREG_AF_STATUS);   
    }
    dbg_print("retry = %d", retry);
 
   */
	return ADCM_ERR_NONE;
}

int adcm2700_master_clock(u8 clk)
{
  
    //0x2712  2X
    //0x1D09  3X   //default
    //0x2709  4X
    //0xFC53
    u16 div = 0x1D09;

    clk *= (((div>>8) + 1)/((div&0xFF) + 1));
    
    
    //adcm2700_reg_write(EREG_SEN_CLK_DIV, 1);

    /*enable pll*/
    adcm2700_write(SREG_CONTROL, 0);

    adcm2700_write(SREG_CLK_FREQ, (u16)clk * 1000);
    adcm2700_write(EREG_PLL_DIV_L, div);
    adcm2700_write(EREG_PLL_DIV_S, div);
    adcm2700_write(EREG_PLL_CTRL,  0x0025);  //0x002D
    
    wait_sreg_update();
    //adcm2700_reg_write(EREG_I_CLK_DIV, 0);
    adcm2700_write(SREG_CONTROL, 1);

    return ADCM_ERR_NONE;
}


int adcm2700_input_size(adcm_window_size * window)
{
  
   /* write sample control register step:
    1 Stop the camera ¨C write 0x0000 to the CONTROL register.
    2 Change the relevant register. 
    3 Set the CONFIG bit in the CONTROL register, write 0x0004.
    4 Wait for the CONFIG bit in CONTROL to clear.  */
 /*
   adcm2700_write(SREG_CONTROL, 0);

      adcm2700_write(SREG_SIZE, 0x707);  
      adcm2700_write(SREG_SENSOR_WID_V, window->width);
      adcm2700_write(SREG_SENSOR_HGT_V, window->height);
      
      //adcm2700_write(SREG_SENSOR_WID_S, window->width);
      //adcm2700_write(SREG_SENSOR_HGT_S, window->height);
      
   wait_sreg_update();
 */
   return ADCM_ERR_NONE;
 }
int adcm2700_output_size(adcm_window_size * window)
{
   
   /* write sample control register step:
    1 Stop the camera ¨C write 0x0000 to the CONTROL register.
    2 Change the relevant register. 
    3 Set the CONFIG bit in the CONTROL register, write 0x0004.
    4 Wait for the CONFIG bit in CONTROL to clear.  */
/*
   adcm2700_write(SREG_CONTROL, 0);
  
       adcm2700_write(SREG_SIZE, 0x707);  
       adcm2700_write(SREG_OUTPUT_WID_V, window->width);
       adcm2700_write(SREG_OUTPUT_HGT_V, window->height);
//     adcm2700_write(SREG_OUTPUT_WID_S, window->width);
//     adcm2700_write(SREG_OUTPUT_HGT_S, window->height);

   wait_sreg_update();
 */
   return ADCM_ERR_NONE;
}


int adcm2700_set_fps(u16 fps, u16 minfps)
{
    if(fps > MAX_FPS  || minfps > fps)
    {
        return ADCM_ERR_PARAMETER;
    }
    
    adcm2700_reg_write(SREG_FRAME_RATE, fps*10);
    adcm2700_reg_write(EREG_AE_ETIME_MAX, 100000/(minfps));
    
    return ADCM_ERR_NONE;
}


int adcm2700_stillframe_cfg_output(u16 format)
{

    u16   oldcfg =   adcm2700_reg_read(SREG_OUTPUT_FORMAT);
    
    oldcfg = (oldcfg & 0xF0FF) | ((format & 0x000F) << 8);
    adcm2700_reg_write( SREG_OUTPUT_FORMAT, oldcfg);
    
    return ADCM_ERR_NONE;
}

int adcm2700_viewfinder_cfg_output(u16 format)
{

      u16   oldcfg = adcm2700_reg_read( SREG_OUTPUT_FORMAT);
      
      oldcfg = (oldcfg & 0xFFF0) | (format & 0x000F);
      adcm2700_reg_write( SREG_OUTPUT_FORMAT, oldcfg );
      return ADCM_ERR_NONE;
}



void adcm2700_dump_register(u16 startRegAddr, u16 endRegAddr, u16* buffer)
{
	u16 addr;
	
	for(addr = startRegAddr; addr <= endRegAddr; addr+=2)
	{
	   adcm2700_read(addr, buffer++);
	}
}

int adcm2700_set_gamma(int table)
{
  const u16 regs[] = 
  { 
    0x1400, 0x1402, 0x1404, 0x1406, 0x1408, 0x140a, 0x140c, 0x140e, 0x1410, 0x1412, 0x1414,  
    0x1416, 0x1418, 0x141a, 0x141c, 0x141e, 0x1420, 0x1422, 0x1424, 0x1426, 0x1428, 0x142a,  
    0x142c, 0x142e, 0x1430, 0x1432, 0x1434, 0x1436, 0x1438, 0x143a, 0x143c, 0x143e, 0x1440
  };
  

  const u16 regsv[] = 
  {
  EREG_TM_COEF_00_V, EREG_TM_COEF_01_V, EREG_TM_COEF_02_V, EREG_TM_COEF_03_V, 
  EREG_TM_COEF_04_V, EREG_TM_COEF_05_V, EREG_TM_COEF_06_V, EREG_TM_COEF_07_V, 
  EREG_TM_COEF_08_V, EREG_TM_COEF_09_V, EREG_TM_COEF_10_V, EREG_TM_COEF_11_V, 
  EREG_TM_COEF_12_V, EREG_TM_COEF_13_V, EREG_TM_COEF_14_V, EREG_TM_COEF_15_V,
  EREG_TM_COEF_16_V, EREG_TM_COEF_17_V, EREG_TM_COEF_18_V, EREG_TM_COEF_19_V,
  EREG_TM_COEF_20_V, EREG_TM_COEF_21_V, EREG_TM_COEF_22_V, EREG_TM_COEF_23_V,
  EREG_TM_COEF_24_V, EREG_TM_COEF_25_V, EREG_TM_COEF_26_V, EREG_TM_COEF_27_V,
  EREG_TM_COEF_28_V, EREG_TM_COEF_29_V, EREG_TM_COEF_30_V, EREG_TM_COEF_31_V,
  EREG_TM_COEF_32_V
  };
  const u16 regss[] = 
  {
  EREG_TM_COEF_00_S, EREG_TM_COEF_01_S, EREG_TM_COEF_02_S, EREG_TM_COEF_03_S, 
  EREG_TM_COEF_04_S, EREG_TM_COEF_05_S, EREG_TM_COEF_06_S, EREG_TM_COEF_07_S, 
  EREG_TM_COEF_08_S, EREG_TM_COEF_09_S, EREG_TM_COEF_10_S, EREG_TM_COEF_11_S, 
  EREG_TM_COEF_12_S, EREG_TM_COEF_13_S, EREG_TM_COEF_14_S, EREG_TM_COEF_15_S,
  EREG_TM_COEF_16_S, EREG_TM_COEF_17_S, EREG_TM_COEF_18_S, EREG_TM_COEF_19_S,
  EREG_TM_COEF_20_S, EREG_TM_COEF_21_S, EREG_TM_COEF_22_S, EREG_TM_COEF_23_S,
  EREG_TM_COEF_24_S, EREG_TM_COEF_25_S, EREG_TM_COEF_26_S, EREG_TM_COEF_27_S,
  EREG_TM_COEF_28_S, EREG_TM_COEF_29_S, EREG_TM_COEF_30_S, EREG_TM_COEF_31_S,
  EREG_TM_COEF_32_S
  };

  const u16 value[][33] =   
  {
  //default
    0x0000, 0x003c, 0x0052, 0x0063, 0x0070, 0x007c, 0x0087, 0x0091, 0x0094, 0x00ab, 0x00b9, 
    0x00c7, 0x00d3, 0x00ea, 0x00fe, 0x0111, 0x0122, 0x0141, 0x015d, 0x0176, 0x018d, 0x01b8,
    0x01de, 0x0201, 0x0221, 0x025b, 0x028f, 0x02bf, 0x02eb, 0x033b, 0x0382, 0x03c3, 0x0400,
    /*
    0x0000, 0x004C, 0x0065, 0x0078, 0x0087, 0x0094, 0x00A0, 0x00ab, 0x00b5, 0x00c6, 0x00d6, 
    0x00e4, 0x00f1, 0x0109, 0x011E, 0x0131, 0x0142, 0x0161, 0x017D, 0x0197, 0x01AE, 0x01d8,
    0x01fd, 0x021F, 0x023E, 0x0276, 0x02a8, 0x02d5, 0x02ff, 0x0349, 0x038C, 0x03C8, 0x0400, 
    */   
  
   
   //solarize
     0x0400, 0x03e9, 0x03a9, 0x0344, 0x02c3, 0x0232, 0x019c, 0x010e, 0x0095, 0x003c, 0x0009,
     0x0002, 0x0026, 0x0074, 0x00e3, 0x016b, 0x01ff, 0x0294, 0x031c, 0x038b, 0x03d9, 0x03fd,
     0x03f6, 0x03c3, 0x036a, 0x02f1, 0x0263, 0x01cd, 0x013c, 0x00bb, 0x0056, 0x0016, 0x0000,
     
   //neg.art
     0x03ff, 0x03e8, 0x03cd, 0x03b9, 0x03a9, 0x039b, 0x038e, 0x0383, 0x0379, 0x0366, 0x0356, 
     0x0347, 0x0339, 0x0320, 0x030a, 0x02f6, 0x02e4, 0x02c2, 0x02a5, 0x028a, 0x0272, 0x0245,
     0x021e, 0x01fa, 0x01da, 0x019e, 0x016a, 0x013a, 0x010e, 0x00c0, 0x007a, 0x003a, 0x0000,
    
   
   /*
    //2.6
    0x0000, 0x005C, 0x0079, 0x008d, 0x009e, 0x00ac, 0x00b9, 0x00c4, 0x00ce, 0x00e1, 0x00f1, 
    0x0100, 0x010e, 0x0126, 0x013b, 0x014e, 0x0160, 0x0180, 0x019c, 0x01b5, 0x01cc, 0x01f5,
    0x0219, 0x023a, 0x0258, 0x028e, 0x02be, 0x02e9, 0x0310, 0x0356, 0x0394, 0x03cc, 0x0400, 
   
    //2.4
    0x0000, 0x004C, 0x0065, 0x0078, 0x0087, 0x0094, 0x00A0, 0x00ab, 0x00b5, 0x00c6, 0x00d6, 
    0x00e4, 0x00f1, 0x0109, 0x011E, 0x0131, 0x0142, 0x0161, 0x017D, 0x0197, 0x01AE, 0x01d8,
    0x01fd, 0x021F, 0x023E, 0x0276, 0x02a8, 0x02d5, 0x02ff, 0x0349, 0x038C, 0x03C8, 0x0400, 
    
    //2.3
    0x0000, 0x0043, 0x005b, 0x006d, 0x007c, 0x0088, 0x0094, 0x009e, 0x00a7, 0x00b8, 0x00c8,
    0x00d6, 0x00e2, 0x00fa, 0x010e, 0x0121, 0x0132, 0x0151, 0x016d, 0x0187, 0x019e, 0x01c8,
    0x01ee, 0x0210, 0x0230, 0x0269, 0x029c, 0x02ca, 0x02f5, 0x0342, 0x0387, 0x03c6, 0x0400,
     */
    //2.2
    0x0000, 0x003c, 0x0052, 0x0063, 0x0070, 0x007c, 0x0087, 0x0091, 0x0094, 0x00ab, 0x00b9, 
    0x00c7, 0x00d3, 0x00ea, 0x00fe, 0x0111, 0x0122, 0x0141, 0x015d, 0x0176, 0x018d, 0x01b8,
    0x01de, 0x0201, 0x0221, 0x025b, 0x028f, 0x02bf, 0x02eb, 0x033b, 0x0382, 0x03c3, 0x0400,
     
    //2.1
    0x0000, 0x0034, 0x0049, 0x0058, 0x0065, 0x0070, 0x007b, 0x0084, 0x008d, 0x009d, 0x00ab, 
    0x00b8, 0x00c4, 0x00da, 0x00ee, 0x0100, 0x0111, 0x0130, 0x014b, 0x0164, 0x017c, 0x01a7,
    0x01cd, 0x01f0, 0x0211, 0x024c, 0x0281, 0x02b2, 0x02e0, 0x0332, 0x037c, 0x03c0, 0x0400,
    
    //2.0
    0x0000, 0x002d, 0x0040, 0x004e, 0x005a, 0x0065, 0x006e, 0x0077, 0x0080, 0x008f, 0x009c,
    0x00a9, 0x00b5, 0x00ca, 0x00dd, 0x00ef, 0x0100, 0x011e, 0x0139, 0x0152, 0x016a, 0x0194,
    0x01bb, 0x01de, 0x0200, 0x023c, 0x0273, 0x02a5, 0x02d4, 0x0329, 0x0376, 0x03bd, 0x0400,
   
    //1.9
    0x0000, 0x0026, 0x0037, 0x0044, 0x004f, 0x0059, 0x0062, 0x006a, 0x0072, 0x0081, 0x008e, 
    0x009a, 0x00a5, 0x00b9, 0x00cc, 0x00dd, 0x00ed, 0x010b, 0x0126, 0x013f, 0x0156, 0x0181, 
    0x01a8, 0x01cc, 0x01ed, 0x022b, 0x0263, 0x0296, 0x02c6, 0x031f, 0x0370, 0x03ba, 0x0400,
    
    //1.8 ap30
    0x0000, 0x0020, 0x002f, 0x003a, 0x0045, 0x004e, 0x0056, 0x005e, 0x0065, 0x0073, 0x007f, 
    0x008a, 0x0095, 0x00a9, 0x00bb, 0x00cb, 0x00db, 0x00f8, 0x0112, 0x012b, 0x0142, 0x016d, 
    0x0194, 0x01b8, 0x01da, 0x0218, 0x0251, 0x0286, 0x02b8, 0x0314, 0x0368, 0x03b6, 0x0400,
    //default
 
    0x0000, 0x0017, 0x0032, 0x0046, 0x0056, 0x0064, 0x0071, 0x007c, 0x0086, 0x0099, 0x00a9,
    0x00b8, 0x00c6, 0x00df, 0x00f5, 0x0109, 0x011b, 0x013d, 0x015a, 0x0175, 0x018d, 0x01ba,
    0x01e1, 0x0205, 0x0225, 0x0261, 0x0295, 0x02c5, 0x02f1, 0x033f, 0x0385, 0x03c5, 0x0400
   
  };
  
  int i, ret;
  /*
  for(i = 0; i < 33; i++)
  {
     ret = adcm2700_write(regss[i], value[table][i]);
     if(ret < 0)
     {
       dbg_print("adcm2700 write error!");
       return -1;
     }
  }
  */
  
  for(i = 0; i < 33; i++)
  {
     ret = adcm2700_write(regsv[i], value[table][i]);
     if(ret < 0)
     {
       dbg_print("adcm2700 write error!");
       return -1;
     }
  }
  
  for(i = 0; i < 33; i++)
  {
     ret = adcm2700_write(regs[i],  value[table][i]);
     if(ret < 0)
     {
       dbg_print("adcm2700 write error!");
       return -1;
     }
  }
  return ADCM_ERR_NONE;
}
/*set picture style(normal/black white/sepia/solarize/neg.art)*/
int adcm2700_set_style(V4l_PIC_STYLE style)
{
  const u16 regs1[] = {EREG_CC_COEF_00, EREG_CC_COEF_01, EREG_CC_COEF_02,
                       EREG_CC_COEF_10, EREG_CC_COEF_11, EREG_CC_COEF_12,
                       EREG_CC_COEF_20, EREG_CC_COEF_21, EREG_CC_COEF_22};
                       
  // 0 - normal    1 - black white   2 - sepia
  const u16 value1[][9] = 
  {
   //{0x02f9, 0x0f03, 0x0f02, 0x0f4f, 0x025c, 0x0f54, 0x0fe0, 0x0e4a, 0x02d5}, //normal settings
   
   {0x01f0, 0x0f88, 0x0f88,
    0x0f88, 0x01f0, 0x0f88,
    0x0f88, 0x0f88, 0x01f0},
   
   {0x003b, 0x00c8, 0x0fcf, 0x003b, 0x00c8, 0x0fcf, 0x003b, 0x00c8, 0x0fcf}, //black white
   {0x005c, 0x0131, 0x0fb6, 0x0030, 0x00a2, 0x0fd9, 0x001a, 0x0058, 0x0feb}, //sepia   
  }; 
   

  u16 * reg_value;
  u16 reg_count;
  switch(style)
  {
  case V4l_STYLE_BLACK_WHITE:
       adcm2700_set_gamma(0);
       reg_value = value1[1];
       reg_count = 9;
       break;
  case V4l_STYLE_SEPIA:
       adcm2700_set_gamma(0);
       reg_value = value1[2];
       reg_count = 9;
       break;
  case V4l_STYLE_SOLARIZE:
       adcm2700_set_gamma(1);
       return ADCM_ERR_NONE; 
  case V4l_STYLE_NEG_ART:
       adcm2700_set_gamma(2);
       return ADCM_ERR_NONE; 
  default:
       adcm2700_set_gamma(0);
     //  adcm2700_reg_write(SREG_ILLUM, illum);
       return ADCM_ERR_NONE;
  }

  int i;
  for(i = 0; i < reg_count; i++)
  {
    adcm2700_write(regs1[i], reg_value[i]);
  }
  return ADCM_ERR_NONE;
}

/*set picture light(auto/direct sun/incandescent/fluorescent)*/     
u16 adcm2700_get_light_v(V4l_PIC_WB light)
{ /*
  const u16 regs1[] = {EREG_CC_COEF_00, EREG_CC_COEF_01, EREG_CC_COEF_02,
                       EREG_CC_COEF_10, EREG_CC_COEF_11, EREG_CC_COEF_12,
                       EREG_CC_COEF_20, EREG_CC_COEF_21, EREG_CC_COEF_22};
                       
  const u16 value1[][9] = 
  {
   {0x02f9, 0x0f03, 0x0f02, 0x0f4f, 0x025c, 0x0f54, 0x0fe0, 0x0e4a, 0x02d5}, //default
   {0x0235, 0x0f8f, 0x0f3b, 0x0f63, 0x01f0, 0x0fad, 0x000d, 0x0eb2, 0x0241}, //day
   {0x0235, 0x0f46, 0x0f85, 0x0f64, 0x01fc, 0x0f9f, 0x0008, 0x0e8d, 0x026b}, //fluorescent
   {0x023a, 0x0f34, 0x0f92, 0x0f5a, 0x0218, 0x0f8e, 0x0ffa, 0x0deb, 0x031b}, //tungsten   
  }; 
  int i, index;
  switch(light)
  {
   case V4l_WB_DIRECT_SUN:
       index = 1;
       break;       
   case V4l_WB_INCANDESCENT:
       index = 3;
       break;       
   case V4l_WB_FLUORESCENT:
       index = 2;
       break;       
   default:
       index = 0;
       break;
  }
  for(i = 0; i< 9; i++)
  {
      adcm2700_write(regs1[i], value1[index][i]);
  }

  */

  u16 value = adcm2700_reg_read(SREG_ILLUM);
  value &= ~0x0007;
  switch(light)
  {
  case V4l_WB_DIRECT_SUN:
       value |= 0x0002;
       break;       
  case V4l_WB_INCANDESCENT:
       value |= 0x0006;
       break;       
  case V4l_WB_FLUORESCENT:
       value |= 0x0004;
       break;       
  default:
       break;
   }
   return value;
}
/* Sunny - 5500K daylight */
const u16 agilent_sunny_color[] =
{0x0214, 0x0F6E, 0x0F7C,
 0x0F58, 0x022D, 0x0F79,
 0x0FF4, 0x0EB3, 0x0257};

/* Cloudy - 7500K daylight */
const u16 agilent_cloudy_color[] =
{0x021B, 0x0F8D, 0x0F56,
 0x0F5C, 0x0226, 0x0F7D,
 0x0FF5, 0x0EE7, 0x0223};

/* Indoor - home tungsten */
const u16 agilent_home_tungsten_color[] =
{0x0243, 0x0EDD, 0x0FDF,
 0x0F3B, 0x0273, 0x0F50,
 0x0FA4, 0x0D9F, 0x03BB};

/* Indoor - office cool white fluorescent */
const u16 agilent_office_cool_white_fluorescent_color[] =
{0x01CF, 0x0F6E, 0x0FC1,
 0x0F72, 0x020D, 0x0F7F,
 0x0FEF, 0x0E5A, 0x02B6};

/* Night tungsten */
const u16 agilent_night_tungsten_color[] =
{0x0243, 0x0EDD, 0x0FDF,
 0x0F3B, 0x0273, 0x0F50,
 0x0FA4, 0x0D9F, 0x03BB};

/* Automatic (generic default setting) 6500K daylight */
const u16 agilent_automatic_color[] =
{0x02F4, 0x0F27, 0x0F30,
 0x0F0C, 0x02F2, 0x0F4E,
 0x0FE8, 0x0E2E, 0x0335};
 
int adcm2700_set_light(V4l_PIC_WB light)
{
  // u16 value = adcm2700_get_light_v(light);
  // adcm2700_reg_write(SREG_ILLUM, value);
  
  const u16 regs1[] = {EREG_CC_COEF_00, EREG_CC_COEF_01, EREG_CC_COEF_02,
                       EREG_CC_COEF_10, EREG_CC_COEF_11, EREG_CC_COEF_12,
                       EREG_CC_COEF_20, EREG_CC_COEF_21, EREG_CC_COEF_22};
  const u16 agilent_default[] = {0x02f9, 0x0f03, 0x0f02, 0x0f4f, 0x025c, 0x0f54, 0x0fe0, 0x0e4a, 0x02d5};
     /*                  
  const u16 value1[][9] = 
  {
   {0x02f9, 0x0f03, 0x0f02, 0x0f4f, 0x025c, 0x0f54, 0x0fe0, 0x0e4a, 0x02d5}, //default
   {0x0235, 0x0f8f, 0x0f3b, 0x0f63, 0x01f0, 0x0fad, 0x000d, 0x0eb2, 0x0241}, //day
   {0x0235, 0x0f46, 0x0f85, 0x0f64, 0x01fc, 0x0f9f, 0x0008, 0x0e8d, 0x026b}, //fluorescent
   {0x023a, 0x0f34, 0x0f92, 0x0f5a, 0x0218, 0x0f8e, 0x0ffa, 0x0deb, 0x031b}, //tungsten   
  }; 
  */
  int i;
  const u16 * values = NULL;
  u16 v = adcm2700_reg_read(EREG_AF_CTRL1);
  switch(light)
  {
   case V4l_WB_DIRECT_SUN:
       values = agilent_sunny_color;
       adcm2700_write(EREG_AF_CTRL1, v&(~0x0002)); 
       break;       
   case V4l_WB_INCANDESCENT:
       values = agilent_night_tungsten_color;
       adcm2700_write(EREG_AF_CTRL1, v&(~0x0002)); 
       break;       
   case V4l_WB_FLUORESCENT:
       values = agilent_office_cool_white_fluorescent_color;
       adcm2700_write(EREG_AF_CTRL1, v&(~0x0002)); 
       break;       
   default:
       values = agilent_automatic_color;
       adcm2700_write(EREG_AF_CTRL1, v|0x0002);
       break;
  }
  if(values != agilent_automatic_color)
  {
   for(i = 0; i < 9; i++)
   {
      adcm2700_write(regs1[i], values[i]);
   }
  }
  return 0;
}
        
/*set picture brightness*/
int adcm2700_set_bright(int bright)
{
  const u16 target[] = 
   { 
       0x0010,      // -2.0 EV
       0x0015,      // -1.5 EV
       0x0020,      // -1.0 EV
       0x0030,      // -0.5 EV
       0x0040,      //  0.0 EV
       0x0060,      // +0.5 EV
       0x0080,      // +1.0 EV
       0x00C0,      // +1.5 EV
       0x00FF       // +2.0 EV
   };

  if(bright < -4 || bright > 4)
  {
    return ADCM_ERR_PARAMETER;
  }
  adcm2700_write(EREG_AE_TARGET, target[bright+4]);
  return ADCM_ERR_NONE;
}

#define ADCM2700_PROC_CTRL_V_REG        0x0112
#define ADCM2700_PROC_CTRL_S_REG        0x0132

#define ADCM2700_AV_CENTER_COL_REG      0x106e
#define ADCM2700_AV_CENTER_ROW_REG      0x1070
#define ADCM2700_AV_OVAL_FACT_REG       0x1094


#define ADCM2700_PROC_CTRL_V_REG        0x0112
#define ADCM2700_PROC_CTRL_S_REG        0x0132

#define ADCM2700_AV_LEFT_TOP_REG        0x106a
#define ADCM2700_AV_RIGHT_BOT_REG       0x106c
#define ADCM2700_AV_CENTER_COL_REG      0x106e
#define ADCM2700_AV_CENTER_ROW_REG      0x1070
#define ADCM2700_AV_OVAL_FACT_REG       0x1094

#define ADCM2700_AV_RED_RAM_REG         0x1800
#define ADCM2700_AV_GREEN_RAM_REG       0x1840
#define ADCM2700_AV_BLUE_RAM_REG        0x1880

unsigned short AV_BLUE_2700[32] =
{
    0x47, 0x49, 0x4b, 0x4d, 0x50, 0x52, 0x55, 0x58,
    0x5a, 0x5d, 0x60, 0x63, 0x66, 0x69, 0x6d, 0x71,
    0x75, 0x79, 0x7d, 0x82, 0x87, 0x8d, 0x93, 0x9a,
    0xa1, 0xa9, 0xb2, 0xbd, 0xc8, 0xd6, 0xe5, 0xf7
};

unsigned short AV_GREEN_2700[32] =
{
    0x44, 0x48, 0x4b, 0x4f, 0x52, 0x56,  0x5a,  0x5e,
    0x63, 0x67, 0x6c, 0x70, 0x75, 0x7b,  0x80,  0x86,
    0x8c, 0x92, 0x98, 0x9f, 0xa6, 0xae,  0xb5,  0xbd,
    0xc6, 0xcf, 0xd8, 0xe2, 0xed, 0xf8,  0x103, 0x10f
};

unsigned short AV_RED_2700[32] =
{
    0x43, 0x47, 0x4b, 0x50, 0x54,  0x59,  0x5e,  0x63,
    0x69, 0x6f, 0x75, 0x7b, 0x82,  0x89,  0x90,  0x97,
    0x9e, 0xa6, 0xae, 0xb6, 0xbe,  0xc6,  0xce,  0xd6,
    0xde, 0xe6, 0xee, 0xf6, 0xfe,  0x103, 0x104, 0x103
};


void adcm2700_color_init()
{
    u16  regdata;
    int  count;

    adcm2700_reg_write(ADCM2700_AV_CENTER_COL_REG, 0x0146);
    adcm2700_reg_write(ADCM2700_AV_CENTER_ROW_REG, 0x00fc);
    adcm2700_reg_write(ADCM2700_AV_OVAL_FACT_REG,  0x10e);

    for(count = 0; count < 32; count++)
    {
        adcm2700_reg_write(ADCM2700_AV_RED_RAM_REG+count*2,   AV_RED_2700[count]);
    }

    for(count = 0; count < 32; count++)
    {    
        adcm2700_reg_write(ADCM2700_AV_GREEN_RAM_REG+count*2, AV_GREEN_2700[count]);
    }
    
    for(count = 0; count < 32; count++)
    {
        adcm2700_reg_write(ADCM2700_AV_BLUE_RAM_REG+count*2,  AV_BLUE_2700[count]);
    }


    /* video mode sharpening enable*/
    regdata = adcm2700_reg_read(ADCM2700_PROC_CTRL_V_REG);
    adcm2700_reg_write(ADCM2700_PROC_CTRL_V_REG, regdata|0x4800);

    /* still mode sharpening enable*/
    regdata = adcm2700_reg_read(ADCM2700_PROC_CTRL_S_REG);
    adcm2700_reg_write(ADCM2700_PROC_CTRL_S_REG, regdata|0x4800);


    /* disable automatic dark*/
    regdata = 0x64;
    adcm2700_reg_write(0x081C, regdata);
}

int adcm2700_set_flicker(int flicker)
{
   
    u16 old = adcm2700_reg_read(EREG_AF_CTRL2);
    u16 old1 = adcm2700_reg_read(SREG_ILLUM);
    old1 &= ~(0x18);
    dbg_print("set flicker to %d", flicker);
    if(flicker == 50)
    {
        adcm2700_reg_write(SREG_ILLUM, old1|0x08);
        adcm2700_write(EREG_AF_CTRL2, old&(~0x02));
        //1000 = 100000/50/2
        adcm2700_write(EREG_AE_ETIME_DFLT, 1000);
    }
    else if(flicker == 60)
    {
         adcm2700_reg_write(SREG_ILLUM, old1|0x10);
         adcm2700_write(EREG_AF_CTRL2, old|0x02);
         // 833 = 100000/60/2 
         adcm2700_write(EREG_AE_ETIME_DFLT, 833);  
    }
    else
    {
        return ADCM_ERR_PARAMETER;
    }
    return 0;
}

