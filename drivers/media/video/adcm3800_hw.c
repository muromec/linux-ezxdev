/*
 *  adcm3800_hw.c
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
 *  Mu Chenliang        06/14/2004      LIBee41682   Created, modified from adcm2700_hw.c
 *  Mu Chenliang        12/08/2004      LIBff42749   Update
 *
*/

/*
 * ==================================================================================
 *                                  INCLUDE FILES
 * ==================================================================================
 */

#include <linux/types.h>
#include <linux/delay.h>

#include "camera.h"
#include "adcm3800_hw.h"

#define  MAX_FPS  300   // 30 fps

static adcm3800_context_t  adcm3800_context;

#define adcm3800_read(addr, pvalue)             \
    i2c_adcm3800_read(addr, pvalue);            \
    ddbg_print("i2c read: a(0x%04x) v(0x%04x)", addr, *(pvalue));
 
#define adcm3800_write(addr, value)             \
    i2c_adcm3800_write(addr, value);            \
    ddbg_print("i2c write: a(0x%04x) v(0x%04x)", addr, value);
 
#define adcm3800_read_byte(addr, pvalue)        \
    i2c_adcm3800_read_byte(addr, pvalue);       \
    ddbg_print("i2c read b: a(0x%04x) v(0x%02x)", addr, *(pvalue));
 
#define adcm3800_write_byte(addr, value)        \
    i2c_adcm3800_write_byte(addr, value);       \
    ddbg_print("i2c write b: a(0x%04x) v(0x%02x)", addr, value);
 
static int adcm3800_reg_read(u16 reg_addr)
{
    u16 value;
    int ret = adcm3800_read(reg_addr, &value);
    if(ret < 0)
    {
        return ret;
    }
    return value;
}

// These routine writes to consectutive 16 bit registers
// with addresses that increment by 2.
static int adcm3800_write_regs (u16 regAddr, u16 *regData, u16 nRegs)
{
    int i, ret;
    for(i = 0; i < nRegs; i++)
    {
        ret = adcm3800_write(regAddr+i*2, regData[i]);
        if(ret < 0)
        {
            dbg_print("adcm3800 write error %d:%04x!", i, regAddr+i*2);
            return -1;
        }
    }
    return i;
}

static int adcm3800_writeb_regs (u16 regAddr, u8 *regData, u16 nRegs)
{
    int i, ret;
    for(i = 0; i < nRegs; i++)
    {
        ret = adcm3800_write_byte(regAddr+i, regData[i]);
        if(ret < 0)
        {
            dbg_print("adcm3800 write error %d:%04x!", i, regAddr+i*2);
            return -1;
        }
    }
    return i;
}
     
void adcm3800_dump_awb_regs(void)
{
#ifdef DEBUG
    int i;
    unsigned short value;
    unsigned char valueb;

    // read some PI/AWB registers
    adcm3800_read(0x003e, &value);  // current CC matrix for PI
    dbg_print("003e: %04x (PI index)", value);
    adcm3800_read(0x1064, &value);  // current red gains
    dbg_print("1064: %04x (red gain)", value);
    adcm3800_read(0x1066, &value);  // current blue gains
    dbg_print("1066: %04x (blue gain)", value);

    // read only current registers
    adcm3800_read(0x0244, &value);  // C_GAIN,  current gain
    dbg_print("0244: %04x (current gain)", value);
    adcm3800_read(0x0246, &value);  // C_ETIME, current exposure time
    dbg_print("0246: %04x (current exposure time)", value);
    adcm3800_read(0x024a, &value);  // C_RG_RATIO,  current red/green ratio
    dbg_print("024a: %04x (current red/green ratio)", value);
    adcm3800_read(0x024c, &value);  // C_BG_RATIO,  current blue/green ratio
    dbg_print("024c: %04x (current blue/green ratio)", value);
#endif
}

static int adcm3800_print_status(void)
{
    adcm3800_dump_awb_regs();

#if DEBUG > 1
    int i;
    unsigned short value;
    unsigned char valueb;

    adcm3800_read(0x0002, &value);
    ddbg_print("control: %04x", value);
    adcm3800_read(0x0004, &value);
    ddbg_print("status: %04x", value);
    adcm3800_read(0x000c, &value);
    ddbg_print("exposure time: %d ms", value/100);
    adcm3800_read(0x0016, &value);
    ddbg_print("frame rate: %d/10 fps", value);

    // read only current registers
    adcm3800_read(0x0248, &value);  // C_BNF_TIME,  current flicker time

    // PLL & clock registers
    adcm3800_read(0x100a, &value);
    adcm3800_read(0x3000, &value);
    adcm3800_read(0x3004, &value);
    adcm3800_read(0x3006, &value);

    adcm3800_read(0x114, &value);

    // Auto function registers
    adcm3800_read(EREG_AE_TARGET, &value);
    adcm3800_read(0x017c, &value);
    adcm3800_read(0x017e, &value);

    adcm3800_read(EREG_AF_CTRL1, &value);
    adcm3800_read(EREG_AF_CTRL2, &value);
    adcm3800_read(EREG_APS_COEF_GRN1, &value);
    adcm3800_read(EREG_APS_COEF_GRN2, &value);
    adcm3800_read(EREG_APS_COEF_RED, &value);
    adcm3800_read(EREG_APS_COEF_BLUE, &value);

    // statistics registers
    //adcm3800_read(0x1076, &value);
    //adcm3800_read(0x1074, &value);

    // Sensor exposure registers
    adcm3800_read_byte(0x080F, &valueb);
    adcm3800_read_byte(0x0810, &valueb);
    adcm3800_read_byte(0x0811, &valueb);
    adcm3800_read_byte(0x0812, &valueb);
    
    adcm3800_read_byte(0x0813, &valueb);
    adcm3800_read_byte(0x0814, &valueb);
    adcm3800_read_byte(0x0815, &valueb);
    adcm3800_read_byte(0x080e, &valueb);
    
    // read CC_COEF
    for(i=0; i<9; i++)
    {    adcm3800_read(0x1028+i*2, &value); }
    // read NACC Bright CC
    for(i=0; i<9; i++)
    {    adcm3800_read(0x0270+i*2, &value); }
    // read gamma
    for(i=0; i<33; i++)
    {    adcm3800_read(0x1400+i*2, &value); }
    // read CSC
    for(i=0; i<10; i++)
    {    adcm3800_read(0x0190+i*2, &value); }
    for(i=0; i<10; i++)
    {    adcm3800_read(0x1046+i*2, &value); }
#endif
    return 0;
}

static void adcm3800_wait(u32 ms)
{
    if(ms > 10)
    {
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(ms/10);
    }
}

static int wait_sreg_update(void)
{
    dbg_print("wait for sensor update simple registers");
    adcm3800_write(SREG_CONTROL, 0x04);
    adcm3800_wait(10);
    int retry = adcm3800__TIMEOUT*4;
    while(--retry)
    {
        u16 v;
        int ret;
        ret = adcm3800_read(SREG_CONTROL, &v);
        if(ret<0)
            return ret;

        if(!(v & 0x04))
        { 
            dbg_print("retry = %d", retry);
            return 0;
        }
        adcm3800_wait(2);
    }
    dbg_print("error:timeout to SCL, retry = %d", retry);
    return -EIO;
}
 
static int adcm3800_firmware_upgrade(void);
static int adcm3800_init_regs(void);

static void adcm3800_init_context(void)
{
    memset(&adcm3800_context, 0, sizeof(adcm3800_context));

    adcm3800_context.SCL_restart = 1;        // force restart at first time
    adcm3800_context.format = O_FORMAT_NONE;
    adcm3800_context.max_expotime = 100000;  // 100ms
}

///////////////////////////////////////////////////////////////
//
//   Programming Guide Chapter 1: Basic Programming
//
///////////////////////////////////////////////////////////////

int adcm3800_master_clock(u32 clk)
{
#if 1
    int i;
    unsigned short value;
    unsigned int   clock;
    //0x5017  3.375X    // 24M*3.375 = 81M
    //0x502f  1.6875X   // 48M*1.6875 = 81M
    //0x4f2f            // 48.75*80/48 = 81.25M
    //u16 div = 0x4f2f;   // 48.75*80/48 = 81.25M
    u16 div = 0x502f;   // 48.75*81/48 = 82.265625M

    clock = clk*((div>>8) + 1) / ((div&0xFF) + 1);
    
    adcm3800_read(EREG_PLL_CTRL,  &value);
    dbg_print("1 clk %d, clock %d, PLL CTRL : %04x", clk, clock, value);
    /*enable pll*/
    adcm3800_write(0x300a, clock/4000-1);
    adcm3800_write(EREG_PLL_DIV_L, div);
    adcm3800_write(EREG_PLL_DIV_S, div);
    adcm3800_write(EREG_PLL_CTRL,  0x0015);  //0x002D
    adcm3800_write(0x3040, 0x0100);
    
    adcm3800_write(0x010c, 0x0006);
    adcm3800_write(SREG_CLK_FREQ, (u16)(clock / 10));

    for(i=0; i<1000; i++)
    {
        adcm3800_read(EREG_PLL_CTRL,  &value);
        dbg_print("2 PLL CTRL : %04x", value);
        if(value&0x80)
            break;
        mdelay(2);
    }
    dbg_print("3 wait %d ms : ", i*2);

#else
    adcm3800_write(SREG_CLK_FREQ, (u16)(clk / 10));
#endif

    return 0;
}

int adcm3800_power_on( u32 clk )
{
    adcm3800_init_context();

    //read out version
    //u16 sensor_rev, cm_rev;
    //adcm3800_version_revision(&cm_rev, &sensor_rev);

    /*Follow these steps to correctly power on the ADCM-2700:
    1 Turn on VCC voltage (2.8 volts) and wait 20 milliseconds.
    2 Turn on MCLK (13 MHz is the default value) and wait 150 milliseconds.
    3 Read register 0x0004; if the return value equals 0x0001, power-up is complete. */
    int   retry = adcm3800__TIMEOUT/2;
    int   timeout = 0;
    while(--retry)
    {
        int ret;
        u16 value;
        ret = adcm3800_read(SREG_STATUS, &value);
        if(value == 0x0001)
        {
             dbg_print("adcm3800 Power-up complete!! time %dms", timeout); 
             break;
        }
        adcm3800_wait(2);
        timeout += 2;
    }
    if(retry==0)
    {
        dbg_print("error: timeout! Power-up failed!");
        return -EIO;
    }

    // turn off camera
    adcm3800_write(SREG_CONTROL, 0x0);

    // Load the firmware upgrade
    adcm3800_firmware_upgrade();

    // initialize registers
    adcm3800_init_regs();

    /* Program the mafster clock */
    adcm3800_master_clock(clk);

    return 0;
}


int adcm3800_power_off()
{
	/*stop camera*/
	adcm3800_write(SREG_CONTROL, 0);
	return 0;   
}


/////////////////////////////////////////////////////////////////////////////////////
//   
//  Programming Guide Chapter 2: Configuration Methods 
//
/////////////////////////////////////////////////////////////////////////////////////


static int adcm3800_version_revision(u16 * cm_revision, u16 *sensor_revision)
{
    u8 bvalue;

    //Camera module version is 0x060
    adcm3800_read(SREG_ID, cm_revision);
    dbg_print("adcm3800 SREG_ID is 0x%x", *cm_revision);
     
    //Image sensor version is 0x60
    adcm3800_read_byte(EREG_IDENT, &bvalue);
    *sensor_revision = bvalue;
    dbg_print("adcm3800 EREG_IDENT is 0x%x", *sensor_revision);

    return 0;
}

int adcm3800_viewfinder_on()
{
    ddbg_print("camera video mode start!"); 
    adcm3800_print_status();
    return 0;
}

int adcm3800_viewfinder_off()
{
    adcm3800_print_status();

    ddbg_print("camera video mode stop!");
    return 0;
}

static int adcm3800_register_sensor_size(u16 width, u16 height)
{
    adcm3800_write(SREG_SENSOR_WID_V, width);
    adcm3800_write(SREG_SENSOR_HGT_V, height);
    return 0;
}

int adcm3800_set_sensor_size(u16 width, u16 height)
{
    if( width!=adcm3800_context.sensor_w ||
        height!=adcm3800_context.sensor_h )
    {
        adcm3800_context.sensor_w = width;
        adcm3800_context.sensor_h = height;
        adcm3800_context.SCL_partial = 1;
        return 1;
    }
    return 0;
}

static int adcm3800_register_output_size(u16 width, u16 height)
{
    adcm3800_write(SREG_OUTPUT_WID_V, width);
    adcm3800_write(SREG_OUTPUT_HGT_V, height);

    return 0;
}

int adcm3800_set_output_size(u16 width, u16 height)
{
    if( width!=adcm3800_context.output_w || 
        height!=adcm3800_context.output_h )
    {
        adcm3800_context.output_w = width;
        adcm3800_context.output_h = height;
        adcm3800_context.SCL_partial = 1;
        return 1;
    }
    return 0;
}


static int adcm3800_register_output_format(u16 format)
{
    u16   newcfg;
    u16   oldcfg;
    adcm3800_read(SREG_OUTPUT_FORMAT, &oldcfg);
      
    newcfg = (oldcfg & 0xFFF0) | (format & 0x000F);
    if(newcfg!=oldcfg)
    {
        adcm3800_write( SREG_OUTPUT_FORMAT, newcfg );
    }

    return 0;
}

int adcm3800_set_output_format(u16 format)
{
    if( format!=adcm3800_context.format )
    {
        adcm3800_context.format = format;
        adcm3800_context.SCL_partial = 1;   // TODO: not confirmed
        return 1;
    }
    return 0;
}

static int adcm3800_register_fps(p_adcm3800_context_t adcm_ctx)
{
    if( (adcm_ctx->fps>70) && (adcm_ctx->output_w > 640 || adcm_ctx->output_h > 480) )
    {
        //if capture size > VGA, the frame rate shall be low
        // this is limit of CIF PCLK < 24.25Mhz
        adcm3800_write(SREG_FRAME_RATE, 75);  // 7.5 fps
    }
    else
    {
        //normal fps
        adcm3800_write(SREG_FRAME_RATE, adcm_ctx->fps);
    }
    return 0;
}

int adcm3800_set_fps(u16 fps)
{
    if(fps > MAX_FPS)
    {
        return -EINVAL;
    }
    
    if( fps!=adcm3800_context.fps )
    {
        adcm3800_context.fps = fps;
        adcm3800_context.SCL_partial = 1;   // TODO: not confirmed
        return 1;
    }
    return 0;
}

static int adcm3800_register_gamma(int table)
{
  dbg_print("set gamma table %d", table);

  const u16 value[][33] =   
  {
    //0 default sRGB
    {
    0x0000, 0x0017, 0x0032, 0x0046, 0x0056, 0x0064, 0x0071, 0x007c, 0x0086, 0x0099, 0x00a9,
    0x00b8, 0x00c6, 0x00df, 0x00f5, 0x0109, 0x011b, 0x013d, 0x015a, 0x0175, 0x018d, 0x01ba,
    0x01e1, 0x0205, 0x0225, 0x0261, 0x0295, 0x02c5, 0x02f1, 0x033f, 0x0385, 0x03c5, 0x0400},
     
    //1 2.6
    {
    0x0000, 0x005c, 0x0079, 0x008d, 0x009e, 0x00ac, 0x00b9, 0x00c4, 0x00ce, 0x00e1, 0x00f1, 
    0x0100, 0x010e, 0x0126, 0x013b, 0x014e, 0x0160, 0x0180, 0x019c, 0x01b5, 0x01cc, 0x01f5,
    0x0219, 0x023a, 0x0258, 0x028e, 0x02be, 0x02e9, 0x0310, 0x0356, 0x0394, 0x03cc, 0x0400}, 
    
    //2 2.5
    {
    0x0000, 0x0054, 0x006f, 0x0083, 0x0093, 0x00a0, 0x00ac, 0x00b7, 0x00c2, 0x00d4, 0x00e4, 
    0x00f2, 0x00ff, 0x0117, 0x012d, 0x0140, 0x0151, 0x0171, 0x018d, 0x01a6, 0x01bd, 0x01e7,
    0x020c, 0x022d, 0x024c, 0x0283, 0x02b3, 0x02df, 0x0308, 0x0350, 0x0390, 0x03ca, 0x0400}, 
    
    //3 2.4
    {
    0x0000, 0x004C, 0x0065, 0x0078, 0x0087, 0x0094, 0x00A0, 0x00ab, 0x00b5, 0x00c6, 0x00d6, 
    0x00e4, 0x00f1, 0x0109, 0x011E, 0x0131, 0x0142, 0x0161, 0x017D, 0x0197, 0x01AE, 0x01d8,
    0x01fd, 0x021F, 0x023E, 0x0276, 0x02a8, 0x02d5, 0x02ff, 0x0349, 0x038C, 0x03C8, 0x0400}, 
    
    //4 2.3
    {
    0x0000, 0x0043, 0x005b, 0x006d, 0x007c, 0x0088, 0x0094, 0x009e, 0x00a7, 0x00b8, 0x00c8,
    0x00d6, 0x00e2, 0x00fa, 0x010e, 0x0121, 0x0132, 0x0151, 0x016d, 0x0187, 0x019e, 0x01c8,
    0x01ee, 0x0210, 0x0230, 0x0269, 0x029c, 0x02ca, 0x02f5, 0x0342, 0x0387, 0x03c6, 0x0400},

    //5 2.2
    {
    0x0000, 0x003c, 0x0052, 0x0063, 0x0070, 0x007c, 0x0087, 0x0091, 0x0094, 0x00ab, 0x00b9, 
    0x00c7, 0x00d3, 0x00ea, 0x00fe, 0x0111, 0x0122, 0x0141, 0x015d, 0x0176, 0x018d, 0x01b8,
    0x01de, 0x0201, 0x0221, 0x025b, 0x028f, 0x02bf, 0x02eb, 0x033b, 0x0382, 0x03c3, 0x0400},
  
    //6 2.1
    {
    0x0000, 0x0034, 0x0049, 0x0058, 0x0065, 0x0070, 0x007b, 0x0084, 0x008d, 0x009d, 0x00ab, 
    0x00b8, 0x00c4, 0x00da, 0x00ee, 0x0100, 0x0111, 0x0130, 0x014b, 0x0164, 0x017c, 0x01a7,
    0x01cd, 0x01f0, 0x0211, 0x024c, 0x0281, 0x02b2, 0x02e0, 0x0332, 0x037c, 0x03c0, 0x0400},
     
    //7 2.0
    {
    0x0000, 0x002d, 0x0040, 0x004e, 0x005a, 0x0065, 0x006e, 0x0077, 0x0080, 0x008f, 0x009c,
    0x00a9, 0x00b5, 0x00ca, 0x00dd, 0x00ef, 0x0100, 0x011e, 0x0139, 0x0152, 0x016a, 0x0194,
    0x01bb, 0x01de, 0x0200, 0x023c, 0x0273, 0x02a5, 0x02d4, 0x0329, 0x0376, 0x03bd, 0x0400},
   
    //8 1.9
    {
    0x0000, 0x0026, 0x0037, 0x0044, 0x004f, 0x0059, 0x0062, 0x006a, 0x0072, 0x0081, 0x008e, 
    0x009a, 0x00a5, 0x00b9, 0x00cc, 0x00dd, 0x00ed, 0x010b, 0x0126, 0x013f, 0x0156, 0x0181, 
    0x01a8, 0x01cc, 0x01ed, 0x022b, 0x0263, 0x0296, 0x02c6, 0x031f, 0x0370, 0x03ba, 0x0400},
    
    //9 1.8
    {
    0x0000, 0x0020, 0x002f, 0x003a, 0x0045, 0x004e, 0x0056, 0x005e, 0x0065, 0x0073, 0x007f, 
    0x008a, 0x0095, 0x00a9, 0x00bb, 0x00cb, 0x00db, 0x00f8, 0x0112, 0x012b, 0x0142, 0x016d, 
    0x0194, 0x01b8, 0x01da, 0x0218, 0x0251, 0x0286, 0x02b8, 0x0314, 0x0368, 0x03b6, 0x0400},
   
    //10 1.6
    {
    0x0000, 0x0014, 0x0020, 0x0029, 0x0031, 0x0038, 0x003f, 0x0046, 0x004c, 0x0057, 0x0062,
    0x006b, 0x0075, 0x0086, 0x0097, 0x00a6, 0x00b5, 0x00d0, 0x00e9, 0x0100, 0x0117, 0x0140,
    0x0167, 0x018c, 0x01ae, 0x01ee, 0x022a, 0x0262, 0x0297, 0x02fb, 0x0357, 0x03ae, 0x0400}

  };
  
  int i, ret;
  /*
  for(i = 0; i < 33; i++)
  {
     adcm3800_write(0x1400+i*2, value[table][i]);
  }
  */
  
  for(i = 0; i < 33; i++)
  {
     ret = adcm3800_write(EREG_TM_COEF_00_V+i*2, value[table][i]);
  }

  return 0;
}

static int adcm3800_register_gamma_solarize(void)
{
  const u16 value[33] =   
  {
   //solarize
     0x0400, 0x03e9, 0x03a9, 0x0344, 0x02c3, 0x0232, 0x019c, 0x010e, 0x0095, 0x003c, 0x0009,
     0x0002, 0x0026, 0x0074, 0x00e3, 0x016b, 0x01ff, 0x0294, 0x031c, 0x038b, 0x03d9, 0x03fd,
     0x03f6, 0x03c3, 0x036a, 0x02f1, 0x0263, 0x01cd, 0x013c, 0x00bb, 0x0056, 0x0016, 0x0000
  };

  int i, ret;
  /*
  for(i = 0; i < 33; i++)
  {
     adcm3800_write(0x1400+i*2, value[i]);
  }
  */
  
  for(i = 0; i < 33; i++)
  {
     ret = adcm3800_write(EREG_TM_COEF_00_V+i*2, value[i]);
  }

  return 0;
}

/***************** Image Styles ************************/
/* No special effect */
static const u16 agilent_adcm3800_normal_effect[] =
{0x0026, 0x004B, 0x000F,
 0x01ED, 0x01DB, 0x0038,
 0x004F, 0x01BE, 0x01F3,
 0x0000};

/* Antique effect */
static const u16 agilent_adcm3800_antique_effect[] =
{0x0026, 0x004B, 0x000F,
 0x01F8, 0x01F8, 0x01F8,
 0x0008, 0x0008, 0x0008, 
 0x0000};

/* Black and White effect */
static const u16 agilent_adcm3800_black_and_white_effect[] =
{0x0026, 0x004B, 0x000F,
 0x0000, 0x0000, 0x0000,
 0x0000, 0x0000, 0x0000,
 0x0000};

/* Color Negative effect */
static const u16 agilent_adcm3800_color_negative_effect[] =
{0x01da, 0x01bf, 0x01f1,
 0x0013, 0x0025, 0x01c8,
 0x01b1, 0x0042, 0x000d,
 0x00ff};

#if 0
/* saturation 50% effect */
static const u16 agilent_adcm3800_saturation_50_effect[] =
{0x0026, 0x004B, 0x000F,
 0x01e4, 0x01c9, 0x0054,
 0x0076, 0x019d, 0x01ed,
 0x0000};

/* saturation 100% effect */
static const u16 agilent_adcm3800_saturation_100_effect[] =
{0x0026, 0x004B, 0x000F,
 0x01da, 0x01b6, 0x0070,
 0x009d, 0x017c, 0x01e6,
 0x0000};

/* Reddish effect */
static const u16 agilent_adcm3800_red_effect[] =
{0x0026, 0x004B, 0x000F,
 0x01e4, 0x01db, 0x0038,
 0x0076, 0x01be, 0x01f3,
 0x0000};

/* Greenish effect */
static const u16 agilent_adcm3800_green_effect[] =
{0x0026, 0x004B, 0x000F,
 0x01ed, 0x01c9, 0x0038,
 0x004f, 0x019d, 0x01F3,
 0x0000};

/* Bluish effect */
static const u16 agilent_adcm3800_blue_effect[] =
{0x0026, 0x004B, 0x000F,
 0x01ed, 0x01db, 0x0054,
 0x004f, 0x01be, 0x01ed,
 0x0000};

/* Cyan effect */
static const u16 agilent_adcm3800_cyan_effect[] =
{0x0026, 0x004B, 0x000F,
 0x01f7, 0x01db, 0x0038,
 0x0027, 0x01be, 0x01f3,
 0x0000};
#endif

/* End additional matrices */
/*******************************************************/


/*set picture style(normal/black white/sepia/solarize/neg.art)*/
static int adcm3800_register_style(V4l_PIC_STYLE style)
{
    u16 * reg_value;
    u16 reg_count = 10;
    reg_value = (u16 *)agilent_adcm3800_normal_effect;
    switch(style)
    {
        case V4l_STYLE_BLACK_WHITE:
            reg_value = (u16 *)agilent_adcm3800_black_and_white_effect;
            break;
        case V4l_STYLE_SEPIA:
            reg_value = (u16 *)agilent_adcm3800_antique_effect;
            break;
        case V4l_STYLE_SOLARIZE:
            break;
        case V4l_STYLE_NEG_ART:
            reg_value = (u16 *)agilent_adcm3800_color_negative_effect;
            break;
        default:
            break;
    }

    int i;
    for(i = 0; i < reg_count; i++)
    {
        adcm3800_write(EREG_CSC_00_V+i*2, reg_value[i]);
    }
    return 0;
}

/*set picture style(normal/black white/sepia/solarize/neg.art)*/
int adcm3800_set_style(V4l_PIC_STYLE style)
{
    if( style!=adcm3800_context.style )
    {
        adcm3800_context.style = style;
        adcm3800_context.SCL_restart = 1;
        return 1;
    }
    return 0;
}

/******* Agilent Color Matrices ***************************/

/*
White Balance gains are located in registers 0x1064 (Red Gain) and 0x1066 (Blue Gains)

Color Correction Coeffecients are located in registers 0x1028-0x1038

* To force write White Balance coeffecients, make sure AWB is off (bit 1 in 0x140)

* To force write Color Correction coeffiecients, make sure NACC is off (bit 3 in 0x140)


The following are the register writes for various iluminants:

*************************************************
Illuminant D50
*************************************************
w 1064 12B
w 1066 143
w 1028 23E EDF FE3 F3E 2A8 F1A FBC E23 321
------------------------ 
*************************************************
Illuminant D55
*************************************************
w 1064 134
w 1066 13B
w 1028 233 EF7 FD7 F3F 29B F26 FC1 E40 2FF
------------------------ 
*************************************************
Illuminant D65
*************************************************
w 1064 141
w 1066 12D
w 1028 223 F17 FC6 F41 289 F36 FC6 E6A 2D0
------------------------ 
*************************************************
Illuminant D75
*************************************************
w 1064 14B
w 1066 123
w 1028 21A F2C FBA F42 27D F40 FC7 E86 2B3
------------------------ 
*************************************************
Illuminant D90
*************************************************
w 1064 155
w 1066 118
w 1028 212 F40 FAF F44 272 F4A FC7 EA2 298
------------------------ 
*************************************************
Illuminant FluorTriphosphor_4000K
*************************************************
w 1064 111
w 1066 16A
w 1028 251 E44 06B F63 281 F1B FB7 E12 338
------------------------ 
*************************************************
Illuminant Fluor_PhilipsUltraLume
*************************************************
w 1064 F3
w 1066 190
w 1028 212 DCA 125 F81 23D F42 F8E D81 3F1
------------------------ 
*************************************************
Illuminant blackbody2600	
*************************************************
w 1064 E4
w 1066 176
w 1028 240 DDC 0E4 F49 2BC EFC F44 D57 465
------------------------ 
*************************************************
Illuminant blackbody3200
*************************************************
w 1064 FA
w 1066 166
w 1028 217 E74 075 F50 27D F33 F8C DEA 38A
------------------------ 
*************************************************
Illuminant blackbody3500
*************************************************
w 1064 104
w 1066 15E
w 1028 20A EA2 055 F53 26C F41 F9E E19 349
------------------------ 
*************************************************
Illuminant blackbody3800
*************************************************
w 1064 10E
w 1066 157
w 1028 1FF EC5 03D F55 25F F4C FAB E3D 318
------------------------ 
*************************************************
Illuminant blackbody4200
*************************************************
w 1064 118
w 1066 14E
w 1028 1F4 EE7 025 F57 252 F56 FB7 E63 2E7
------------------------ 
*************************************************
Illuminant blackbody4600
*************************************************
w 1064 121
w 1066 146
w 1028 1EB F02 013 F59 249 F5E FBE E80 2C2
------------------------ 
*************************************************
Illuminant boothCool_4070K
*************************************************
w 1064 126
w 1066 16C
w 1028 201 ED8 027 F6D 274 F1F FAE E0F 343
------------------------ 
*************************************************
Illuminant boothHorizon
*************************************************
w 1064 D5
w 1066 17D
w 1028 264 D43 159 F47 313 EA6 EF5 CD9 531
------------------------ 
*************************************************
Illuminant boothInca
*************************************************
w 1064 ED
w 1066 170
w 1028 22C E29 0AB F4C 29A F1A F69 DA0 3F7
------------------------ 
*************************************************
Illuminant fluorescent_3335K
*************************************************
w 1064 109
w 1066 186
w 1028 1DB E6A 0BB F83 229 F54 FA2 DE4 37A
------------------------ 
*************************************************
Illuminant halogen_2250K
*************************************************
w 1064 D5
w 1066 184
w 1028 213 D7F 16E F5C 2C2 EE2 F01 D43 4BC
------------------------ 
*/

/* Sunlight 
*************************************************
Illuminant D65
*************************************************
w 1064 141
w 1066 12D
w 1028 223 F17 FC6 F41 289 F36 FC6 E6A 2D0
*/
static const u16 agilent_adcm3800_sunny_color_gain[] =
{0x0141, 0x012D};
static const u16 agilent_adcm3800_sunny_color[] =
{0x0223, 0x0F17, 0x0FC6,
 0x0F41, 0x0289, 0x0F36,
 0x0FC6, 0x0E6A, 0x02D0};

/* Cloudy 
*************************************************
Illuminant D75
*************************************************
w 1064 14B
w 1066 123
w 1028 21A F2C FBA F42 27D F40 FC7 E86 2B3
*/
static const u16 agilent_adcm3800_cloudy_color_gain[] =
{0x014B, 0x0123};
static const u16 agilent_adcm3800_cloudy_color[] =
{0x021A, 0x0F2C, 0x0FBA, 
 0x0F42, 0x027D, 0x0F40, 
 0x0FC7, 0x0E86, 0x02B3};


/* Indoor - home tungsten
*************************************************
Illuminant blackbody3200
*************************************************
w 1064 FA
w 1066 166
w 1028 217 E74 075 F50 27D F33 F8C DEA 38A
*/
static const u16 agilent_adcm3800_home_tungsten_color_gain[] =
{0x0FA, 0x0166};
static const u16 agilent_adcm3800_home_tungsten_color[] =
{0x0217, 0x0E74, 0x0075, 
 0x0F50, 0x027D, 0x0F33, 
 0x0F8C, 0x0DEA, 0x038A};


/* Indoor - office cool white fluorescent 
*************************************************
Illuminant fluorescent_3335K
*************************************************
w 1064 109
w 1066 186
w 1028 1DB E6A 0BB F83 229 F54 FA2 DE4 37A
 * */
static const u16 agilent_adcm3800_office_fluorescent_color_gain[] =
{0x0109, 0x0186};
static const u16 agilent_adcm3800_office_fluorescent_color[] =
{0x01DB, 0x0E6A, 0x00BB, 
 0x0F83, 0x0229, 0x0F54, 
 0x0FA2, 0x0DE4, 0x037A};

/* Night tungsten
static const u16 agilent_adcm3800_night_tungsten_color[] =
{0x034f, 0x0cdd, 0x00d3, 
 0x0f47, 0x033c, 0x0e73, 
 0x0f31, 0x0b2a, 0x06a3};
*/

/* Automatic (generic default setting) 5500K sun-light
*************************************************
Illuminant D55
*************************************************
w 1064 134
w 1066 13B
w 1028 233 EF7 FD7 F3F 29B F26 FC1 E40 2FF
*/
static const u16 agilent_adcm3800_automatic_color_gain[] =
{0x0134, 0x013B};
static const u16 agilent_adcm3800_automatic_color[] =
{0x0233, 0x0EF7, 0x0FD7,
 0x0F3F, 0x029B, 0x0F26,
 0x0FC1, 0x0E40, 0x02FF};
/*******************************************************/


static int adcm3800_register_light(V4l_PIC_WB light)
{
    int i;
    int awb = 0;
    u16* color_cc;
    u16* color_gain;
    switch(light)
    {
      case V4l_WB_DIRECT_SUN:
        awb = 0;
        color_cc = (u16 *)agilent_adcm3800_sunny_color;
        color_gain = (u16 *)agilent_adcm3800_sunny_color_gain;
        break;       
      case V4l_WB_INCANDESCENT:
        awb = 0;
        color_cc = (u16 *)agilent_adcm3800_home_tungsten_color;
        color_gain = (u16 *)agilent_adcm3800_home_tungsten_color_gain;
        break;       
      case V4l_WB_FLUORESCENT:
        awb = 1;
        color_cc = (u16 *)agilent_adcm3800_office_fluorescent_color;
        color_gain = (u16 *)agilent_adcm3800_office_fluorescent_color_gain;
        break;       
      default:
        awb = 1;
        color_cc = (u16 *)agilent_adcm3800_automatic_color;
        color_gain = (u16 *)agilent_adcm3800_automatic_color_gain;
        break;
    }

    u16 value_r140;
    adcm3800_read(EREG_AF_CTRL1, &value_r140);
    // disable AWB NACC
    adcm3800_write(EREG_AF_CTRL1, value_r140&(~0x02)&(~0x08));

    for(i = 0; i< 9; i++)
    {
        adcm3800_write(EREG_CC_COEF_00+i*2, color_cc[i]);
    }
#if 1
    for(i = 0; i< 9; i++)
    {
        u16 value;
        adcm3800_read(0x0270+i*2, &value);
        adcm3800_write(0x0270+i*2, (value&0xf000)|color_cc[i]);
    }
#endif

    for(i = 0; i< 2; i++)
    {
        adcm3800_write(0x1064+i*2, color_gain[i]);
    }

    // tighter awb tolerances ???
    // w 17c 108 110
    //adcm3800_write(EREG_AWB_TOL_ACQ, 0x0108);
    //adcm3800_write(EREG_AWB_TOL_MON, 0x0110);

    if(awb) {
        adcm3800_write(EREG_AF_CTRL1, value_r140|0x02);   // enable AWB
    }
    else {
        adcm3800_write(EREG_AF_CTRL1, value_r140&(~0x02));   // disable AWB
    }
    return 0;
}
        
int adcm3800_set_light(V4l_PIC_WB light)
{
    if( light!=adcm3800_context.light )
    {
        adcm3800_context.light = light;
        adcm3800_context.SCL_restart = 1;
        return 1;
    }
    return 0;
}

/*set exposure compensation */
static int adcm3800_register_exp_comp(int bright)
{
    const u16 exp_adj[] = 
    { 
        0x0108,      // -2.0 EV
        0x0106,      // -1.5 EV
        0x0104,      // -1.0 EV
        0x0102,      // -0.5 EV
        0x0000,      //  0.0 EV
        0x0002,      // +0.5 EV
        0x0004,      // +1.0 EV
        0x0006,      // +1.5 EV
        0x0008       // +2.0 EV
    };

    if(bright < -4 || bright > 4)
    {
        return -EINVAL;
    }

    int value = exp_adj[bright+4];
    value |= 0x0200;    // set center zone exposure
    adcm3800_write(SREG_EXP_ADJ, value);
    return 0;
}

/*set picture brightness*/
static int adcm3800_register_bright(int bright)
{
    const u16 target[] = 
    { 
        16,  //0x0010,      // -2.0 EV
        23,  //0x0017,      // -1.5 EV
        32,  //0x0020,      // -1.0 EV
        45,  //0x002d,      // -0.5 EV
        64,  //0x0040,      //  0.0 EV
        90,  //0x005a,      // +0.5 EV
        128, //0x0080,      // +1.0 EV
        181, //0x00b5,      // +1.5 EV
        240  //0x00f0       // +2.0 EV
    };

    if(bright < -4 || bright > 4)
    {
        return -EINVAL;
    }
    adcm3800_write(EREG_AE_TARGET, target[bright+4]);
    return 0;
}

/*set picture brightness*/
int adcm3800_set_bright(int bright)
{
    adcm3800_context.bright = bright;
    adcm3800_register_bright(bright);
    return 0;
}

static int adcm3800_register_exposure_mode(V4l_NM mode, int maxexpotime)
{
    adcm3800_write(EREG_AE_ETIME_MAX, maxexpotime/10);
    return 0;
}

/*set exposure mode: normal/night */
int adcm3800_set_exposure_mode(V4l_NM mode, int maxexpotime)
{
    if(maxexpotime<10 && maxexpotime>500000) // 500ms > time > 10us
        return -EINVAL;

    if(adcm3800_context.max_expotime != maxexpotime)
    {
        adcm3800_context.max_expotime = maxexpotime;
        adcm3800_context.SCL_restart = 1;
        return 1;
    }
    return 0;
}

int adcm3800_set_flicker(int flicker)
{
    adcm3800_context.flicker_freq = flicker;
    u16 old;
    u16 old1;
    adcm3800_read(EREG_AF_CTRL2, &old);
    adcm3800_read(SREG_ILLUM, &old1);
    old1 &= ~(0x18);
    dbg_print("set flicker to %d", flicker);
    if(flicker == 50)
    {
        adcm3800_write(SREG_ILLUM, old1|0x08);
        adcm3800_write(EREG_AF_CTRL2, old&(~0x02));
        //1000 = 100000/50/2
        adcm3800_write(EREG_AE_ETIME_DFLT, 1000);
    }
    else if(flicker == 60)
    {
        adcm3800_write(SREG_ILLUM, old1|0x10);
        adcm3800_write(EREG_AF_CTRL2, old|0x02);
        // 833 = 100000/60/2 
        adcm3800_write(EREG_AE_ETIME_DFLT, 833);  
    }
    else
    {
        return -EINVAL;
    }
    return 0;
}

// SCL restart
static int adcm3800_SCL_restart(p_camera_context_t cam_ctx, int frames)
{
    int ret;

    /*stop camera*/
    adcm3800_write(SREG_CONTROL, 0);

    //update simple registers
    adcm3800_register_fps(&adcm3800_context);

    adcm3800_register_output_format(adcm3800_context.format);

    adcm3800_write(SREG_SIZE, 0x0f0f);  // perfer image quality, disallow subsampling
    //sensor width and sensor height
    adcm3800_register_sensor_size(adcm3800_context.sensor_w, adcm3800_context.sensor_h);
    //output width and output height
    adcm3800_register_output_size(adcm3800_context.output_w, adcm3800_context.output_h);

    if(adcm3800_context.style == V4l_STYLE_SOLARIZE ) {
        adcm3800_register_gamma_solarize();
    }
    else if(frames == 1) {
        adcm3800_register_gamma(0);  // gamma sRGB, still mode
    }
    else {
        adcm3800_register_gamma(5);  // gamma 2.0, video mode
    }

    //adcm3800_print_status();

    // SCL
    ret = wait_sreg_update();
    if(ret<0)
        return ret;

    //adcm3800_print_status();

    // set AE_GAIN to improve pink at center issue
    // w 150 240 280 500 280
#if 1
    adcm3800_write(EREG_AE_GAIN_MIN,    0x0240);
    adcm3800_write(EREG_AE_GAIN_MIN_P,  0x0280);
    adcm3800_write(EREG_AE_GAIN_MAX,    0x0500);
    adcm3800_write(EREG_AE_GAIN_DFLT,   0x0280);
#endif

    adcm3800_register_style(adcm3800_context.style);

    // re-initialize auto functions
    u16 v_ctrl2;
    adcm3800_read(EREG_AF_CTRL2, &v_ctrl2);
    adcm3800_write(EREG_AF_CTRL2, v_ctrl2|0x0040);

    //adcm3800_print_status();

    // set max exposure time
    adcm3800_register_exposure_mode(adcm3800_context.expo_mode,
                                    adcm3800_context.max_expotime);

    // set brightness
    adcm3800_register_bright(adcm3800_context.bright);

    // set white balance
    adcm3800_register_light(adcm3800_context.light);

    // start video mode
    adcm3800_write(SREG_CONTROL, 0x01);

    //adcm3800_print_status();

    adcm3800_write(EREG_AF_CTRL2, v_ctrl2);

    return 0;
}

// SCL partial
int adcm3800_SCL_partial(p_camera_context_t cam_ctx, int frames)
{
    //update simple registers
    adcm3800_register_fps(&adcm3800_context);

    adcm3800_write(SREG_SIZE, 0x0f0f);  // perfer image quality, disallow subsampling
    //sensor width and sensor height
    adcm3800_register_sensor_size(adcm3800_context.sensor_w, adcm3800_context.sensor_h);

    //output width and output height
    adcm3800_register_output_size(adcm3800_context.output_w, adcm3800_context.output_h);

    if(adcm3800_context.style == V4l_STYLE_SOLARIZE ) {
        adcm3800_register_gamma_solarize();
    }
    else if(frames == 1) {
        adcm3800_register_gamma(0);  // gamma sRGB, still mode
    }
    else {
        adcm3800_register_gamma(5);  // gamma 2.0, video mode
    }

    // partial simple control
    // still mode

    //adcm3800_print_status();

    // partial config
    adcm3800_write(SREG_CONTROL, 0x21);

    //adcm3800_print_status();

    return 0;
}

int adcm3800_reconfigure(p_camera_context_t cam_ctx, int frames)
{
    dbg_print("sensor width %d", cam_ctx->sensor_width);
    dbg_print("sensor height %d", cam_ctx->sensor_height);
    dbg_print("capture_width %d", cam_ctx->capture_width);
    dbg_print("capture_height %d", cam_ctx->capture_height);
    ddbg_print("fps %d", cam_ctx->fps);
    ddbg_print("light %d", cam_ctx->capture_light);
    ddbg_print("capture_bright %d", cam_ctx->capture_bright);
    ddbg_print("capture_style %d", cam_ctx->capture_style);

    int ret = 0;
    if(adcm3800_context.SCL_restart != 0)
    {
        ret = adcm3800_SCL_restart(cam_ctx, frames);
        adcm3800_context.SCL_restart = 0;
    }
    else if(adcm3800_context.SCL_partial != 0)
    {
        ret = adcm3800_SCL_partial(cam_ctx, frames);
        adcm3800_context.SCL_partial = 0;
    }

    return ret;
}

//
// This is an example of how to implement a firmware patch table.
// The patch data contained here is the ADCM-3800 patch version 3.
//
// The first array is the starting register address of each contiguous block
// of data to write.  The second array is the number of words to write for
// each block.  These arrays are terminated with a zero entry.  Some other
// method could also be used to know how many patch blocks there are,
// such as using the sizeof() operator, or a separate defined constant.
//
// The third array contains the patch data to be written, formatted according
// to the length of each block.  The length of the third array should equal
// the sum of the entries in the second array.
//

// patch code ver.7, 10/20/2004
static unsigned short patch3800addr[] = {
    0x0186, 0x484c, 0x488c, 0x4838, 0x4818, 0x4898,
    0x4980, 0x002c, 0x0034, 0x4820, 0x486C,
	0x4860, 0x4804, 0x4854, 0x487C, 0x4894, 0x482C, 0x4834, 0x4800,
	0x4868, 0x4C80, 0x0062, 0x006A, 0x0186,
    0};
static unsigned short patch3800len [] = {
    3,      2,      2,      2,      2,      2,
    352,    4,      4,      2,	    2,
	2,	    2,      2,      2,      2,      2,      2,      2,
	2,	    1104,   4,      2,      2,
    0};
static unsigned short patch3800data[] =
{
    // #Load patch 
    // 0x0186, 3
    0x0000, 0x0000, 0x0000,
    // 484C, 2
    0x0002, 0x0012,
    // 488C, 2
    0x0002, 0x437e,
    // 4838, 2
    0x0002, 0x4385,
    // 4818, 2
    0x0002, 0x0564,
    // 4898, 2
    0x0002, 0x43a0,

    // #PI patch 
    // 0x4980, 352
    0x0000, 0x00C0, 0x0000, 0x00EA, 0x0000, 0x0212, 0x0000, 0xFF40,
	0x0000, 0xFFAF, 0x0000, 0xFF44, 0x0000, 0x0272, 0x0000, 0xFF4A,
	0x0000, 0xFFC7, 0x0000, 0xFEA2, 0x0000, 0x0298, 0x0000, 0x00C6,
	0x0000, 0x00E1, 0x0000, 0x021A, 0x0000, 0xFF2C, 0x0000, 0xFFBA,
	0x0000, 0xFF42, 0x0000, 0x027D, 0x0000, 0xFF40, 0x0000, 0xFFC7,
	0x0000, 0xFE86, 0x0000, 0x02B3, 0x0000, 0x00CC, 0x0000, 0x00DA,
	0x0000, 0x0223, 0x0000, 0xFF17, 0x0000, 0xFFC6, 0x0000, 0xFF41,
	0x0000, 0x0289, 0x0000, 0xFF36, 0x0000, 0xFFC6, 0x0000, 0xFE6A,
	0x0000, 0x02D0, 0x0000, 0x00D5, 0x0000, 0x00D0, 0x0000, 0x0233,
	0x0000, 0xFEF7, 0x0000, 0xFFD7, 0x0000, 0xFF3F, 0x0000, 0x029B,
	0x0000, 0xFF26, 0x0000, 0xFFC1, 0x0000, 0xFE40, 0x0000, 0x02FF,
	0x0000, 0x00DB, 0x0000, 0x00CB, 0x0000, 0x023E, 0x0000, 0xFEDF,
	0x0000, 0xFFE3, 0x0000, 0xFF3E, 0x0000, 0x02A8, 0x0000, 0xFF1A,
	0x0000, 0xFFBC, 0x0000, 0xFE23, 0x0000, 0x0321, 0x0000, 0x00E3,
	0x0000, 0x00C9, 0x0000, 0x01EB, 0x0000, 0xFF02, 0x0000, 0x0013,
	0x0000, 0xFF59, 0x0000, 0x0249, 0x0000, 0xFF5E, 0x0000, 0xFFBE,
	0x0000, 0xFE80, 0x0000, 0x02C2, 0x0000, 0x00EA, 0x0000, 0x00C4,
	0x0000, 0x01F4, 0x0000, 0xFEE7, 0x0000, 0x0025, 0x0000, 0xFF57,
	0x0000, 0x0252, 0x0000, 0xFF56, 0x0000, 0xFFB7, 0x0000, 0xFE63,
	0x0000, 0x02E7, 0x0000, 0x00F3, 0x0000, 0x00BF, 0x0000, 0x01FF,
	0x0000, 0xFEC5, 0x0000, 0x003D, 0x0000, 0xFF55, 0x0000, 0x025F,
	0x0000, 0xFF4C, 0x0000, 0xFFAB, 0x0000, 0xFE3D, 0x0000, 0x0318,
	0x0000, 0x00DF, 0x0000, 0x00B4, 0x0000, 0x0201, 0x0000, 0xFED8,
	0x0000, 0x0027, 0x0000, 0xFF6D, 0x0000, 0x0274, 0x0000, 0xFF1F,
	0x0000, 0xFFAE, 0x0000, 0xFE0F, 0x0000, 0x0343, 0x0000, 0x00FC,
	0x0000, 0x00BB, 0x0000, 0x020A, 0x0000, 0xFEA2, 0x0000, 0x0055,
	0x0000, 0xFF53, 0x0000, 0x026C, 0x0000, 0xFF41, 0x0000, 0xFF9E,
	0x0000, 0xFE19, 0x0000, 0x0349, 0x0000, 0x0106, 0x0000, 0x00B7,
	0x0000, 0x0217, 0x0000, 0xFE74, 0x0000, 0x0075, 0x0000, 0xFF50,
	0x0000, 0x027D, 0x0000, 0xFF33, 0x0000, 0xFF8C, 0x0000, 0xFDEA,
	0x0000, 0x038A, 0x0000, 0x00F7, 0x0000, 0x00A8, 0x0000, 0x01DB,
	0x0000, 0xFE6A, 0x0000, 0x00BB, 0x0000, 0xFF83, 0x0000, 0x0229,
	0x0000, 0xFF54, 0x0000, 0xFFA2, 0x0000, 0xFDE4, 0x0000, 0x037A,
	0x0000, 0x0114, 0x0000, 0x00B2, 0x0000, 0x022C, 0x0000, 0xFE29,
	0x0000, 0x00AB, 0x0000, 0xFF4C, 0x0000, 0x029A, 0x0000, 0xFF1A,
	0x0000, 0xFF69, 0x0000, 0xFDA0, 0x0000, 0x03F7, 0x0000, 0x010E,
	0x0000, 0x00A4, 0x0000, 0x0212, 0x0000, 0xFDCA, 0x0000, 0x0125,
	0x0000, 0xFF81, 0x0000, 0x023D, 0x0000, 0xFF42, 0x0000, 0xFF8E,
	0x0000, 0xFD81, 0x0000, 0x03F1, 0x0000, 0x0120, 0x0000, 0x00AF,
	0x0000, 0x0240, 0x0000, 0xFDDC, 0x0000, 0x00E4, 0x0000, 0xFF49,
	0x0000, 0x02BC, 0x0000, 0xFEFC, 0x0000, 0xFF44, 0x0000, 0xFD57,
	0x0000, 0x0465, 0x0000, 0x0134, 0x0000, 0x00AC, 0x0000, 0x0264,
	0x0000, 0xFD43, 0x0000, 0x0159, 0x0000, 0xFF47, 0x0000, 0x0313,
	0x0000, 0xFEA6, 0x0000, 0xFEF5, 0x0000, 0xFCD9, 0x0000, 0x0531,
    // 002c, 4
	0x0080, 0x001A, 0x0034, 0x1388,
    // 0034, 4
	0x03E8, 0x0000, 0x07D0, 0x0001,
	// 4820, 2, 
    0x0002, 0x4320,
	// 486C, 2
    0x0002, 0x432F,
	// 4860, 2
    0x0002, 0x44C0,
	// 4804, 2
    0x0002, 0x44DF,
	// 4854, 2
    0x0002, 0x1D8B,
	// 487C, 2
    0x0002, 0x4510,
	// 4894, 2
    0x0002, 0x4516,
	// 482C, 2
    0x0002, 0x451E,
	// 4834, 2
    0x0002, 0x4522,
	// 4800, 2
    0x0002, 0x4536,
	// 4868, 2
    0x0002, 0x4538,
	// 4C80, 1104
    0x0003, 0x1DB3, 0x0000, 0x0632, 0x0009, 0x0006, 0x0000, 0x661F,
	0x0009, 0x0004, 0x0000, 0x6620, 0x0009, 0x0118, 0x0000, 0x6725,
	0x0009, 0x014e, 0x0000, 0x6726, 0x0009, 0x0118, 0x0000, 0x6532,
	0x0009, 0x014e, 0x0000, 0x6533, 0x0000, 0x01FF, 0x0000, 0x0958,
	0x0000, 0x0960, 0x0000, 0x0968, 0x0000, 0x0970, 0x0000, 0x0978,
	0x0000, 0x4F23, 0x0000, 0x4722, 0x0000, 0x024B, 0x0000, 0x01A3,
	0x0000, 0x091E, 0x000B, 0xFE26, 0x0009, 0x8480, 0x0000, 0x08CB,
	0x000B, 0xFE27, 0x0009, 0x001E, 0x0000, 0x08CB, 0x0009, 0xFE26,
    // 4D00
	0x0000, 0x6621, 0x0000, 0x7A22, 0x0009, 0xFE26, 0x0000, 0x6623,
	0x0003, 0x449B, 0x000B, 0xFE27, 0x0000, 0x085B, 0x000A, 0x0000,
	0x0000, 0x0153, 0x0006, 0x434B, 0x0002, 0x434E, 0x000B, 0xFE26,
	0x0009, 0xFFFF, 0x0000, 0x08CB, 0x0009, 0x0010, 0x0000, 0x662A,
	0x000B, 0xFE26, 0x0000, 0x0873, 0x0000, 0x5620, 0x000B, 0x0000,
	0x0000, 0x4A19, 0x0000, 0x0156, 0x0007, 0x435B, 0x000A, 0x0001,
	0x0000, 0x0155, 0x0004, 0x435B, 0x0002, 0x4363, 0x0000, 0x5219,
	0x0000, 0x042C, 0x0000, 0x4A19, 0x0000, 0x0054, 0x0000, 0x0166,
	0x0007, 0x4362, 0x0002, 0x4363, 0x0002, 0x4364, 0x0000, 0x0183,
	0x0000, 0x031B, 0x0005, 0x436A, 0x0009, 0x0205, 0x0000, 0x662A,
	0x000D, 0x0001, 0x0002, 0x43AE, 0x000B, 0x0000, 0x0000, 0x4A1A,
	0x0000, 0x0156, 0x0007, 0x4372, 0x000A, 0x0002, 0x0000, 0x0155,
	0x0004, 0x4372, 0x0002, 0x437A, 0x0000, 0x521A, 0x0000, 0x042C,
	0x0000, 0x4A1A, 0x0000, 0x0054, 0x0000, 0x0166, 0x0007, 0x4379,
	0x0002, 0x437A, 0x0002, 0x437B, 0x0000, 0x0183, 0x0000, 0x031B,
	0x0005, 0x4381, 0x0009, 0x0003, 0x0000, 0x662A, 0x000D, 0x0002,
    // 4E00
	0x0002, 0x43AE, 0x000B, 0x0000, 0x0000, 0x4A1B, 0x0000, 0x0156,
	0x0007, 0x4389, 0x000A, 0x0003, 0x0000, 0x0155, 0x0004, 0x4389,
	0x0002, 0x4391, 0x0000, 0x521B, 0x0000, 0x042C, 0x0000, 0x4A1B,
	0x0000, 0x0054, 0x0000, 0x0166, 0x0007, 0x4390, 0x0002, 0x4391,
	0x0002, 0x4392, 0x0000, 0x0183, 0x0000, 0x031B, 0x0005, 0x4396,
	0x000D, 0x0003, 0x0002, 0x43AE, 0x000B, 0x0000, 0x0000, 0x4A1C,
	0x0000, 0x0156, 0x0007, 0x439E, 0x000A, 0x0004, 0x0000, 0x0155,
	0x0004, 0x439E, 0x0002, 0x43A6, 0x0000, 0x521C, 0x0000, 0x042C,
	0x0000, 0x4A1C, 0x0000, 0x0054, 0x0000, 0x0166, 0x0007, 0x43A5,
	0x0002, 0x43A6, 0x0002, 0x43A7, 0x0000, 0x0183, 0x0000, 0x031B,
	0x0005, 0x43AD, 0x000D, 0x0004, 0x0009, 0x0010, 0x0000, 0x662A,
	0x0002, 0x43AE, 0x000D, 0x0005, 0x0000, 0x7620, 0x0000, 0x4F4D,
	0x0000, 0x474C, 0x0000, 0x028B, 0x0000, 0x4725, 0x0000, 0x028B,
	0x0000, 0x091F, 0x0000, 0x4F4E, 0x0000, 0x474C, 0x0000, 0x028B,
	0x0000, 0x4726, 0x0000, 0x028B, 0x0000, 0x6E25, 0x0000, 0x462A,
	0x0000, 0x0439, 0x0000, 0x6629, 0x0000, 0x4629, 0x000A, 0x000B,
    // 4F00
    0x0000, 0x0251, 0x0000, 0x01D3, 0x000A, 0x4260, 0x0000, 0x005A,
	0x0000, 0x6A2B, 0x0000, 0x4E2B, 0x0000, 0x0853, 0x0000, 0x6A21,
	0x0000, 0x7E22, 0x0000, 0x4E2B, 0x0000, 0x0183, 0x0000, 0x0853,
	0x0000, 0x6A23, 0x0000, 0x4625, 0x0000, 0x6624, 0x0003, 0x4485,
	0x0000, 0x6628, 0x0000, 0x5629, 0x0000, 0x0185, 0x0000, 0x4E2A,
	0x0000, 0x2CFF, 0x0000, 0x015D, 0x0007, 0x43D8, 0x0002, 0x43F0,
	0x0000, 0x462B, 0x000A, 0x000B, 0x0000, 0x0051, 0x0000, 0x662B,
	0x0000, 0x4E2B, 0x0000, 0x0853, 0x0000, 0x6A21, 0x0000, 0x7E22,
	0x0000, 0x4E2B, 0x0000, 0x0183, 0x0000, 0x0853, 0x0000, 0x6A23,
	0x0000, 0x4625, 0x0000, 0x6624, 0x0003, 0x4485, 0x0000, 0x090E,
	0x0000, 0x4A28, 0x0000, 0x0156, 0x0007, 0x43EC, 0x0002, 0x43EE,
	0x0000, 0x7A28, 0x0000, 0x7629, 0x0000, 0x0185, 0x0002, 0x43D3,
	0x0000, 0x461F, 0x000A, 0x000B, 0x0000, 0x0251, 0x0000, 0x01D3,
	0x000A, 0x4260, 0x0000, 0x005A, 0x0000, 0x6A2B, 0x0000, 0x4E2B,
	0x0000, 0x0853, 0x0000, 0x6A21, 0x0000, 0x7E22, 0x0000, 0x4E2B,
	0x0000, 0x0183, 0x0000, 0x0853, 0x0000, 0x6A23, 0x0000, 0x4625,
	// 5000
    0x0000, 0x6624, 0x0003, 0x4485, 0x0000, 0x090E, 0x0000, 0x4629,
	0x0000, 0x4A1F, 0x0000, 0x0151, 0x0004, 0x4409, 0x0000, 0x0602,
	0x0002, 0x4416, 0x0000, 0x4A28, 0x0000, 0x02B2, 0x0000, 0x0913,
	0x0000, 0x4A16, 0x0000, 0x0153, 0x0006, 0x4410, 0x0002, 0x4415,
	0x0000, 0x461F, 0x0000, 0x6629, 0x0000, 0x7A28, 0x0000, 0x0602,
	0x0002, 0x4416, 0x0000, 0x0622, 0x0000, 0x0652, 0x0005, 0x4422,
	0x0000, 0x4628, 0x0000, 0x4A17, 0x0000, 0x0151, 0x0007, 0x441D,
	0x0002, 0x4421, 0x0000, 0x0612, 0x0009, 0x0000, 0x0000, 0x661E,
	0x0000, 0x0622, 0x0002, 0x442B, 0x0000, 0x4628, 0x0000, 0x4A18,
	0x0000, 0x0151, 0x0006, 0x4427, 0x0002, 0x442B, 0x0000, 0x0632,
	0x0009, 0x0001, 0x0000, 0x661E, 0x0000, 0x0622, 0x000B, 0x0000,
	0x0000, 0x4629, 0x000A, 0x0005, 0x0000, 0x0151, 0x0007, 0x4435,
	0x0000, 0x4620, 0x000A, 0x0002, 0x0000, 0x0151, 0x0006, 0x4435,
	0x0000, 0x0183, 0x0000, 0x031B, 0x0005, 0x443B, 0x0000, 0x0632,
	0x0000, 0x0622, 0x0009, 0x0002, 0x0000, 0x6629, 0x0000, 0x0642,
	0x0005, 0x447B, 0x0000, 0x0605, 0x0000, 0x0606, 0x0000, 0x4629,
	// 5100
    0x000A, 0x000B, 0x0000, 0x0251, 0x0000, 0x01D3, 0x000A, 0x4260,
	0x0000, 0x0053, 0x000A, 0x000A, 0x0000, 0x0053, 0x0000, 0x6E2B,
	0x0000, 0x8EA0, 0x0005, 0x444D, 0x0009, 0xFF40, 0x0000, 0x662C,
	0x0002, 0x444F, 0x0009, 0xFD1C, 0x0000, 0x662C, 0x000D, 0x0001,
	0x000A, 0x000A, 0x0000, 0x0155, 0x0007, 0x4454, 0x0002, 0x4460,
	0x0000, 0x4E2C, 0x0000, 0x0193, 0x0000, 0x6E2C, 0x0000, 0x0183,
	0x0000, 0x522B, 0x0000, 0x0194, 0x0000, 0x722B, 0x0000, 0x0184,
	0x0000, 0x0864, 0x0000, 0x08E3, 0x0000, 0x0185, 0x0002, 0x4450,
	0x0000, 0x0652, 0x0005, 0x4472, 0x0000, 0x4E2B, 0x0000, 0x0193,
	0x0000, 0x6E2B, 0x0000, 0x0183, 0x0000, 0x085B, 0x000A, 0x0100,
	0x0000, 0x029A, 0x0000, 0x0913, 0x0000, 0x6F26, 0x0000, 0x4E2B,
	0x0000, 0x085B, 0x000A, 0x0100, 0x0000, 0x029A, 0x0000, 0x0913,
	0x0000, 0x6F25, 0x0002, 0x447A, 0x000A, 0x0100, 0x0000, 0x02BA,
	0x0000, 0x0913, 0x0000, 0x6F25, 0x000B, 0x0100, 0x0000, 0x4625,
	0x0000, 0x028B, 0x0000, 0x6F26, 0x0002, 0x447D, 0x0000, 0x0625,
	0x0000, 0x0626, 0x0000, 0x4629, 0x0000, 0x661F, 0x0000, 0x0838,
    // 5200
    0x0000, 0x0830, 0x0000, 0x0828, 0x0000, 0x0820, 0x0000, 0x0818,
	0x0000, 0x01FF, 0x0000, 0x0958, 0x0000, 0x4621, 0x0000, 0x4A22,
	0x0003, 0x4493, 0x0000, 0x090B, 0x0000, 0x4623, 0x0000, 0x4A24,
	0x0003, 0x4493, 0x0000, 0x0059, 0x0000, 0x01FB, 0x0005, 0x4491,
	0x0009, 0xFFFF, 0x0000, 0x0818, 0x0000, 0x01FF, 0x0000, 0x008A,
	0x0000, 0x0212, 0x0000, 0x01D1, 0x0000, 0x01C2, 0x0000, 0x0352,
	0x0005, 0x449A, 0x0009, 0xFFFF, 0x0000, 0x01FF, 0x0000, 0x0958,
	0x0000, 0x0960, 0x0000, 0x0978, 0x0000, 0x4A21, 0x0000, 0x080A,
	0x0000, 0x0852, 0x0000, 0x4E22, 0x000C, 0x0000, 0x000F, 0x0020,
	0x0000, 0x060A, 0x0000, 0x0404, 0x0000, 0x01FB, 0x0005, 0x44A9,
	0x0000, 0x062A, 0x0000, 0x05AF, 0x0005, 0x44AC, 0x0000, 0x0540,
	0x0000, 0x0402, 0x0000, 0x059F, 0x0005, 0x44B0, 0x0000, 0x0520,
	0x0000, 0x0401, 0x0000, 0x064A, 0x0004, 0x44B5, 0x0000, 0x015C,
	0x0007, 0x44B7, 0x0000, 0x00DC, 0x0000, 0x0510, 0x0000, 0x0197,
	0x0004, 0x44A4, 0x0000, 0x4E23, 0x0000, 0x088B, 0x0000, 0x08D3,
	0x0000, 0x0838, 0x0000, 0x0820, 0x0000, 0x0818, 0x0000, 0x01FF,
	// 5300
    0x0000, 0x060D, 0x0000, 0x513F, 0x000F, 0x0008, 0x0000, 0x00FC,
	0x0000, 0x5540, 0x0000, 0x00FD, 0x0000, 0x026C, 0x0000, 0x01A7,
	0x0000, 0x52A5, 0x0000, 0x0414, 0x0000, 0x0267, 0x0000, 0x01A7,
	0x0000, 0x7E50, 0x000F, 0x0000, 0x0000, 0x7E51, 0x0000, 0x7E53,
	0x0000, 0x7E55, 0x000B, 0x0000, 0x0002, 0x44D4, 0x0000, 0x0183,
	0x0000, 0x5EA6, 0x0000, 0x017B, 0x0006, 0x21A8, 0x000F, 0x4230,
	0x0000, 0x001F, 0x0000, 0x0877, 0x0000, 0x5253, 0x0000, 0x0074,
	0x0000, 0x7254, 0x000F, 0xFE31, 0x0002, 0x218D, 0x0000, 0x5E01,
	0x0000, 0x05F5, 0x0005, 0x004C, 0x0000, 0x04F5, 0x0000, 0x7E01,
	0x0003, 0x0213, 0x0000, 0x46AF, 0x0000, 0x0948, 0x0000, 0x4722,
	0x0000, 0x0948, 0x0000, 0x4723, 0x0000, 0x0948, 0x0000, 0x060A,
	0x0003, 0x05DC, 0x0000, 0x062A, 0x0003, 0x05DC, 0x0000, 0x060A,
	0x0003, 0x06EC, 0x0000, 0x675E, 0x0000, 0x062A, 0x0003, 0x06EC,
	0x0000, 0x675F, 0x0009, 0x44FA, 0x0000, 0x0948, 0x0009, 0xFFFB,
	0x0000, 0x0048, 0x0002, 0x0564, 0x0000, 0x0808, 0x0000, 0x6723,
	0x0000, 0x0808, 0x0000, 0x6722, 0x0000, 0x0808, 0x0000, 0x66AF,
	// 5400
    0x0003, 0x02BC, 0x0000, 0x7E67, 0x0000, 0x7A68, 0x0000, 0x7269,
	0x0003, 0x1C3B, 0x0000, 0x7E8A, 0x0003, 0x1C55, 0x0000, 0x7E8B,
	0x0000, 0x5EA4, 0x0000, 0x7D07, 0x0000, 0x5F23, 0x0003, 0x1CD1,
	0x0003, 0x01AF, 0x0003, 0x1B8A, 0x0003, 0x1B90, 0x0002, 0x004C,
	0x0000, 0x0640, 0x0004, 0x4514, 0x0000, 0x0641, 0x0005, 0x4515,
	0x0000, 0x0634, 0x0002, 0x225D, 0x0000, 0x0654, 0x0005, 0x451A,
	0x0000, 0x8EA2, 0x0004, 0x451C, 0x0000, 0x060D, 0x0000, 0x01FF,
	0x0000, 0x0614, 0x0002, 0x2163, 0x0003, 0x021E, 0x0000, 0x5E98,
	0x0000, 0x7D04, 0x0000, 0x01FF, 0x0000, 0x466E, 0x0000, 0x240F,
	0x000A, 0x0001, 0x0000, 0x014A, 0x0004, 0x010A, 0x0000, 0x4601,
	0x0000, 0x0591, 0x0004, 0x4531, 0x0000, 0x0590, 0x0004, 0x012C,
	0x0003, 0x0213, 0x0000, 0x4602, 0x0000, 0x0490, 0x0000, 0x6602,
	0x0002, 0x0017, 0x0000, 0xBAA1, 0x0004, 0x4535, 0x0000, 0x8EA2,
	0x0005, 0x012C, 0x0002, 0x0082, 0x0008, 0x4570, 0x0002, 0x001E,
	0x0000, 0x4723, 0x0000, 0x4AAD, 0x0000, 0x0151, 0x0007, 0x1E8F,
	0x0006, 0x4545, 0x0000, 0x4722, 0x0000, 0x4AAA, 0x0000, 0x0151,
	// 5500
    0x0007, 0x1E8F, 0x0009, 0x0100, 0x0000, 0x010C, 0x0007, 0x1E8F,
	0x0000, 0x01FF, 0x0000, 0x6B23, 0x0000, 0x0620, 0x0000, 0x01FF,
	// 0062, 4
    0x0200, 0x0600, 0x0C00, 0x1800,
	// 006A, 2
    0x2800, 0x3800,
	// 0186, 2
    0x2903, 0x8D20
};


static int adcm3800Patch(void)
{
    int err;
    int ipatch, idata;
    for (ipatch = 0, idata = 0; patch3800addr[ipatch] != 0; ipatch++)
    {
        int addr = patch3800addr[ipatch];
        int len = patch3800len[ipatch];
        ddbg_print("write %d length to addr %x", len, addr);
        err = adcm3800_write_regs (addr, &patch3800data[idata], len);
        if (err < 0) 
        {
            dbg_print("error: fail to upgrade patch code");
            return (err);
        }
        idata += len;
    }
    dbg_print("ok: write %d patch code", idata);
    if(idata*2 != sizeof(patch3800data))
    {
        dbg_print("error: patch len wrong! %d:%d", idata, sizeof(patch3800data)/2);
        return -1;
    }
    return 0;
}


static int adcm3800_firmware_upgrade(void)
{
    u8  valueb;
    u16 valuew;

    //adcm3800_write(0x3040, 0x0100);

    int to = 200;
    while(to>0)
    {
        adcm3800_read(0x0004, &valuew);
        if(valuew==0x0000)
            break;
        mdelay(5);
        to -= 1;
    }

    dbg_print("start patch");
    adcm3800Patch();
    dbg_print("end patch");
    adcm3800_read_byte(0x084b, &valueb);

    return 0;
}
    
static int adcm3800_init_regs(void)
{
    u16 valuew;
/*
#Setting pixel ram and sensor registers
#set block 14
w  0a04 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0005 0005 0805 0005 0007 0007 0007 0007 0107 0107 0187 0187 0187 0183 0183 0183

#set block 15
w  0a80 0183 0183 0183 0183 0183 0183 0183 0183
w  0a90 0181 0001 0000 0000 0000 0000 0008 0008
w  0aa0 0008 0008 0079 0079 0079 0079 0079 0079
w  0ab0 0079 0079 0079 0079 0079 0079 0079 0079

#set block 16
w  0b00 0079 0079 0079 0079 0079 0071 0035 0035
w  0b10 0035 0035 0037 0837 0037 0037 0137 0137
w  0b20 01b7 01b7 01b3 01b3 01b3 01b3 01b3 01b3
w  0b30 01b3 01b3 01b3 01b3 01b3 01b3 01b1 0071

#set block 17
w  0b80 0071 0071 0071 0071 0071 0071 0061 0021 0020 0000 0000 0000 0000 0000
*/
    static u16 reg_0a04[] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0005, 0x0005, 0x0805, 0x0005, 0x0007, 0x0007, 0x0007, 0x0007, 0x0107, 0x0107, 0x0187, 0x0187, 0x0187, 0x0183, 0x0183, 0x0183};
    adcm3800_write_regs (0x0a04, reg_0a04, 30);

    static u16 reg_0a80[] = {
        0x0183, 0x0183, 0x0183, 0x0183, 0x0183, 0x0183, 0x0183, 0x0183,
        0x0181, 0x0001, 0x0000, 0x0000, 0x0000, 0x0000, 0x0008, 0x0008,
        0x0008, 0x0008, 0x0079, 0x0079, 0x0079, 0x0079, 0x0079, 0x0079,
        0x0079, 0x0079, 0x0079, 0x0079, 0x0079, 0x0079, 0x0079, 0x0079};
    adcm3800_write_regs (0x0a80, reg_0a80, 32);

    static u16 reg_0b00[] = {
        0x0079, 0x0079, 0x0079, 0x0079, 0x0079, 0x0071, 0x0035, 0x0035,
        0x0035, 0x0035, 0x0037, 0x0837, 0x0037, 0x0037, 0x0137, 0x0137,
        0x01b7, 0x01b7, 0x01b3, 0x01b3, 0x01b3, 0x01b3, 0x01b3, 0x01b3,
        0x01b3, 0x01b3, 0x01b3, 0x01b3, 0x01b3, 0x01b3, 0x01b1, 0x0071};
    adcm3800_write_regs (0x0b00, reg_0b00, 32);

    static u16 reg_0b80[] = {
        0x0071, 0x0071, 0x0071, 0x0071, 0x0071, 0x0071, 0x0061, 0x0021, 0x0020, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
    adcm3800_write_regs (0x0b80, reg_0b80, 14);

/*
# Sensor settings
w8 081e 00
w8 081f 00
w8 0820 00
w8 0821 00
w8 0822 00
w8 0823 00
w8 0824 00
w8 0825 00
w8 0828 02
w8 0845 b0
w8 0848 06
w8 0849 03
w8 084d 20
*/
    adcm3800_write_byte(0x081e, 0x00);
    adcm3800_write_byte(0x081f, 0x00);
    adcm3800_write_byte(0x0820, 0x00);
    adcm3800_write_byte(0x0821, 0x00);
    adcm3800_write_byte(0x0822, 0x00);
    adcm3800_write_byte(0x0823, 0x00);
    adcm3800_write_byte(0x0824, 0x00);
    adcm3800_write_byte(0x0825, 0x00);
    adcm3800_write_byte(0x0828, 0x02);
    adcm3800_write_byte(0x0845, 0xb0);
    adcm3800_write_byte(0x0848, 0x06);
    adcm3800_write_byte(0x0849, 0x03);
    adcm3800_write_byte(0x084D, 0x20);
/*
# Purple sky fix
w8 84f F8
*/
    adcm3800_write_byte(0x084f, 0xF8);

/*
#r  0112 1	# Return:8280
w  0112 c680	# PROC_CTRL_V
#r  0132 1	# Return:0280
w  0132 4680	# PROC_CTRL_S
w  014a 0015	# Auto Black Level Target
#r  0008 1	# Return:0b84
// delete w  0008 0b04	# Image Size and Orientation, still: SXGA, Video: QQVGA
*/
    // turn on AV
    // sharpness
    //adcm3800_write(0x0112, 0x4e80);   // high sharpness
    //adcm3800_write(0x0132, 0x4e80);
    adcm3800_write(0x0112, 0x4a80);   // medium sharpness
    adcm3800_write(0x0132, 0x4a80);
    //adcm3800_write(0x0112, 0x4680);   // low sharpness
    //adcm3800_write(0x0132, 0x4680);
    //adcm3800_write(0x0112, 0x4280);     // no sharpness
    //adcm3800_write(0x0132, 0x4280);
    adcm3800_write(0x014a, 0x0015);

/*
# AWB setting
w  0174 010d	# AWB Default Red/Green Ratio
w  017a 0185	# AWB Default Blue/Green Ratio
w  017c 0108	# AWB Tolerance Acquire
w  017e 0110	# AWB Tolerance Monitor
*/
    // AWB
    adcm3800_write(0x0174, 0x010d);
    adcm3800_write(0x017a, 0x0185);
    adcm3800_write(0x017c, 0x0108);
    adcm3800_write(0x017e, 0x0110);

    // 0x00e0, 6
    static u16 reg_00e0[] = {
        0x003c, 0x00c8, 0x0200, 0x0130, 0x0180, 0x003c
    };
    adcm3800_write_regs (0x00e0, reg_00e0, 6);

/*
#set block 00, AWB setting
w  0028 028f 0333 a3d7 001a 0021 c350 0bb8 01f4 0064 0001 0001
#set block 00
w  0062 0200 0600 0c00 1800 2800 3800
*/
#if 0
    static u16 reg_0028[] = {
        0x028f, 0x0333, 0xa3d7, 0x001a, 0x0021, 0xc350, 0x0bb8, 0x01f4, 0x0064, 0x0001, 0x0001};
    adcm3800_write_regs (0x0028, reg_0028, 11);
    static u16 reg_0062[] = {
        0x0200, 0x0600, 0x0c00, 0x1800, 0x2800, 0x3800};
    adcm3800_write_regs (0x0062, reg_0062, 6);
#endif

/*
# 70% Vignetting Correction (7/27/04 - BB)
w 10a6 248 1d0 248 1d0 260 160 0f6 0f6 0f2

w 1800 0F1 102 111 120 12F 13F 14B 157 165 171 17E 18A 197 1A3 1AB 1B4 1BB 1C3 1C6 1CB 1CA 1C6 1C2 1BB 1B4 1AE 1A9 1A3 19E 198 193 18D
w 1840 0E0 0EF 0FC 108 113 11F 12A 136 141 14D 158 161 16D 176 17E 186 18D 191 196 198 198 197 195 191 18A 183 17A 170 165 159 14C 13F
w 1880 0F8 106 110 11A 124 12D 136 140 14B 154 15D 167 16F 178 180 188 18F 196 19C 1A0 1A4 1A6 1A8 1A7 1A6 1A4 1A0 19C 195 18E 186 17C
*/
    static u16 reg_10a6[] = {
        0x0248, 0x01d0, 0x0248, 0x01d0, 0x0260, 0x0160, 0x00f6, 0x00f6, 0x00f2
    };
    adcm3800_write_regs (0x10a6, reg_10a6, 9);
    // Red vignetting data
    static u16 reg_1800[] = {
        0x00F1, 0x0102, 0x0111, 0x0120, 0x012F, 0x013F, 0x014B, 0x0157,
        0x0165, 0x0171, 0x017E, 0x018A, 0x0197, 0x01A3, 0x01AB, 0x01B4,
        0x01BB, 0x01C3, 0x01C6, 0x01CB, 0x01CA, 0x01C6, 0x01C2, 0x01BB,
        0x01B4, 0x01AE, 0x01A9, 0x01A3, 0x019E, 0x0198, 0x0193, 0x018D
    };
    adcm3800_write_regs (0x1800, reg_1800, 32);
    // Green vignetting data
    static u16 reg_1840[] = {
        0x00E0, 0x00EF, 0x00FC, 0x0108, 0x0113, 0x011F, 0x012A, 0x0136,
        0x0141, 0x014D, 0x0158, 0x0161, 0x016D, 0x0176, 0x017E, 0x0186,
        0x018D, 0x0191, 0x0196, 0x0198, 0x0198, 0x0197, 0x0195, 0x0191,
        0x018A, 0x0183, 0x017A, 0x0170, 0x0165, 0x0159, 0x014C, 0x013F
    };
    adcm3800_write_regs (0x1840, reg_1840, 32);
    // Blue vignetting data
    static u16 reg_1880[] = {
        0x00F8, 0x0106, 0x0110, 0x011A, 0x0124, 0x012D, 0x0136, 0x0140,
        0x014B, 0x0154, 0x015D, 0x0167, 0x016F, 0x0178, 0x0180, 0x0188,
        0x018F, 0x0196, 0x019C, 0x01A0, 0x01A4, 0x01A6, 0x01A8, 0x01A7,
        0x01A6, 0x01A4, 0x01A0, 0x019C, 0x0195, 0x018E, 0x0186, 0x017C
    };
    adcm3800_write_regs (0x1880, reg_1880, 32);

    // BPA
    // BPA 5 Line Threshold, default to 0x0020 
    adcm3800_write(0x1094, 0x0010);

/*
#G1/G2 Threshold
w 101a 23f
*/
    adcm3800_write(0x101a, 0x023f);
/*
#NACC Settings - Dark Table
w 282 2B3 FE0B 41 FF69 2B1 FEE4 FF6D FC69 528
*/
    static u16 reg_0282[] = {
        0x2B3, 0xFE0B, 0x41, 0xFF69, 0x2B1, 0xFEE4, 0xFF6D, 0xFC69, 0x528
    };
    adcm3800_write_regs (0x0282, reg_0282, 9);
/*
#NACC Settings - NACC Table
w 250 96 0 0 100
*/
    static u16 reg_0250[] = {
        0x0096, 0x0000, 0x0000, 0x0100, 0x0ea6, 0x0060, 0x09c4, 0x0080,
        0x04e2, 0x00a0, 0x0271, 0x00c0, 0x0000, 0x0100
    };
    adcm3800_write_regs (0x0250, reg_0250, 14);
/*
#PI control
w 2e A4 EC
*/
    adcm3800_write(0x002e, 0x00A4);
    adcm3800_write(0x0030, 0x00EC);

/*
#Sunlight oscillation
#Set bit 4 of 0x142
# auto function control, select Xenon flash mode and enabled auto exposure deliberate overexposure
w 142 1011

#Turn NACC on
w 140 1B
*/
    //adcm3800_write(0x0142, 0x1011);
    adcm3800_write(EREG_AF_CTRL2, 0x1011);
    //adcm3800_write(0x0140, 0x001B);
    adcm3800_write(EREG_AF_CTRL1, 0x001b);   // enable AE AWB NACC ABL

    dbg_print("end");
    return 0;
}

