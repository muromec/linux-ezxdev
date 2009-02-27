/*
 *  mt9m111_hw.c
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
Ma Zhiqiang         6/30/2004                    Change for auto-detect
Ma Zhiqiang         9/03/2004                    Add the update from Peter 
                                                 (worked out with Micron/Flextronix and LV CTO team)

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

#include "mt9m111_hw.h"
#include "camera.h"


extern int mt9m111_read(u16 addr, u16 *pvalue);
extern int mt9m111_write(u16 addr, u16 value);


static u16 sensorAWidth = MAX_SENSOR_A_WIDTH;
static u16 sensorAHeight = MAX_SENSOR_A_HEIGHT;
static u16 sensorBWidth = MAX_SENSOR_B_WIDTH;
static u16 sensorBHeight = MAX_SENSOR_B_HEIGHT;
static u16 outWidth = DEFT_SENSOR_A_WIDTH;
static u16 outHeight = DEFT_SENSOR_A_HEIGHT;
static u16 captureWidth = DEFT_SENSOR_B_WIDTH;
static u16 captureHeight = DEFT_SENSOR_B_HEIGHT;


static u16 preferredFlickerMask = 0;
static int inSkipMode = 1;
static int didSnapPhoto = 0;
static u16 minFPS = -1;
static u16 maxFPS = -1;

static u8 isHalfSensorSize = 0;

#ifdef  CONFIG_CAMERA_ROTATE_180
/* default to set rows and columns mirror if camera sensor rotate 180 degree */
static const int rotate_mirror_rows = 1;
static const int rotate_mirror_columns = 1;
#else
static const int rotate_mirror_rows = 0;
static const int rotate_mirror_columns = 0;
#endif
/* set through ioctl by application */
static int mirror_rows;
static int mirror_columns;

/* 0-disable strobe flash, 1-enable strobe flash */
static int strobe_flash;

// light tables

/* Jason CCM (80% saturation)*/
const static u16 autoLight[] =
{ 0x00EE, 0x3923, 0x0724, 0x00CF, 0x004F, 0x0006, 0x004D, 0x00EC,
  0x005B, 0x0018, 0x0080, 0x00ED, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x684F, 0x3D29,
  0x0002 };

// ScK1K5 271B -> 271A : K1 3 -> 2 so shift K1 right
const static u16 incandescentLight[] =
{ 0x00EE, 0x3923, 0x0724, 0x00CF, 0x004F, 0x0006, 0x004D, 0x00EC,
  0x005B, 0x0018, 0x0080, 0x00ED, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x863B, 0x0000,
  0x0000 };

// ScK1K5 291B -> 271A : K1 3--> 2 & K4 4 -> 3 so shift K1/D1 and K4/D4 right
// ScK6K9 04E4 -> 02A4 : K8 3 -> 2 & K9 2 -> 1 K8/D8 and K9/D9 shift right
 const static u16 tl84Light[] =
{ 0x00EE, 0x3923, 0x0724, 0x00CF, 0x004F, 0x0006, 0x004D, 0x00EC,
  0x005B, 0x0018, 0x0080, 0x00ED, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x753A, 0x0000,
  0x0000 };

// ScK1K5 291A -> 271A : K4 4 -> 3 so shift K4 and D4 right:
// ScK6K9 04E4 -> 02A4 : K8 3 -> 2 & K9 2 -> 1 K8/D8 and K9/D9 shift right
const static u16 d65Light[] =
{ 0x00EE, 0x3923, 0x0724, 0x00CF, 0x004F, 0x0006, 0x004D, 0x00EC,
  0x005B, 0x0018, 0x0080, 0x00ED, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4A63, 0x0000,
  0x0000 };

const static u16 cloudyLight[] =
{ 0x00EE, 0x3923, 0x0724, 0x00CF, 0x004F, 0x0006, 0x004D, 0x00EC,
  0x005B, 0x0018, 0x0080, 0x00ED, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x5358, 0x0000,
  0x0000 };

// KSigns ScK1K5  ScK6K9, K1      K2      K3      K4      K5
// K6     K7      K8      K9      DSigns  D1      D2      D3
// D4     D5      D6      D7      D8      D9      K GRat  D GRat


/* global initialize flag varible definition */
static V4l_PIC_WB       g_light = -1 ; 
static int              g_earlyAWB = 0 ;
static int              g_flicker = 0 ;
static int              g_earlySetAE = 0 ;
static unsigned long    g_ulAWBJiffies ;
inline void             DisableAWB( void ) ;
inline void             early_set_flicker( void ) ;
inline void             early_set_autoexposure( void ) ;
inline void             mt9m111_early_setting( void ) ;

inline void mt9m111_early_setting()
{
    //Enable flicker detection
    mt9m111_write(0x106, 0x708E);   // Operating Mode Control -- P.13 0x0708C

    DisableAWB() ;
    g_ulAWBJiffies = jiffies ;

    early_set_flicker() ;

    early_set_autoexposure() ;
}

inline void DisableAWB( void )
{
    u16 v106 ;

    v106 = mt9m111_reg_read(0x106);
    v106 = v106 & ~0x0002;
    mt9m111_write(0x106, v106);     // disable AWB

    g_earlyAWB = 1 ;
}

inline void early_set_flicker()
{
    mt9m111_write(0x25B, 0x0001);   // manual mode 50Hz
}

inline void early_set_autoexposure()
{
    u16 limitAE = ((120/15) << 5) & 0x03E0;
    mt9m111_write(0x237, limitAE);
    g_earlySetAE = 1 ;
}

/*****************************************************************************
*
*****************************************************************************/
static void mt9m111_reset(void)
{
    minFPS = -1;
    maxFPS = -1;
    inSkipMode = 1;
    didSnapPhoto = 0;
    isHalfSensorSize = 0;
    /* default mirror */
    mirror_rows = rotate_mirror_rows;
    mirror_columns = rotate_mirror_columns;
    /* default to disable strobe flash */
    strobe_flash = 0;

    g_light = -1 ; 
    g_earlyAWB = 0 ;
    g_flicker = 0 ;
    g_earlySetAE = 0 ;
}

/*****************************************************************************
*									     *
*        I2C Management		 					     *
*									     *
*****************************************************************************/
u16 mt9m111_reg_read(u16 reg_addr)
{
    u16 value=0;
    mt9m111_read(reg_addr, &value);
    return value;
}

void mt9m111_reg_write(u16 reg_addr, u16 reg_value)
{
    mt9m111_write(reg_addr, reg_value);
}

static void mt9m111_reg_write_020(u16 readmode)
{
    readmode = (readmode & 0xfffc) | mirror_rows | (mirror_columns<<1);
    mt9m111_write(0x020, readmode);
}

/////////////////////////////////////////////////////////////////////////////////////
//   
//  Programming Guide : Configuration Methods 
//
/////////////////////////////////////////////////////////////////////////////////////

int mt9m111_get_device_id(u16 *id, u16 *rev)
{
    u16 revision;
    /*Device ID is in register 0x0 */
    *id = mt9m111_reg_read(0x0);

    mt9m111_read(0x0F9, &revision);
    revision |= 0x0080;
    mt9m111_write(0x0F9, revision);
    mt9m111_read(0x0FD, &revision);
    revision = (revision&0x0FE0) >> 5;
    *rev = revision;

    //ddbg_print("mt9m111 device ID is 0x%x, rev%d", *id, revision);       
    return MICRON_ERR_NONE;
}

int mt9m111_viewfinder_on()
{
    if((outWidth*2 > sensorAWidth) || (outHeight*2 > sensorAHeight))
    {
        ddbg_print("mt9m111_viewfinder_on: making sure skip mode is off");

	// make sure skip mode is off

	    if (inSkipMode || didSnapPhoto)
	    {
            ddbg_print("mt9m111_viewfinder_on: skip mode is on -- turning in off");
	    //
	    // set to full power, turn off subsampling
	    //     switch to context B for read mode, full frame, and max line delay
 	    //

    	    mt9m111_write(0x2D2, 0x0041);	// turn off subsampling and turn on full power mode
    	    mt9m111_write(0x2CC, 0x4);		// Context Ctl Program Select -- use default program
    	    u16 v = mt9m111_reg_read(0x2CB);	// read Context Ctl Program Advance
    	    mt9m111_write(0x2CB, v|1);		// advance program

    	    didSnapPhoto = 0;
	        inSkipMode = 0;

            //
            // after program advance, we need to poll Global Context Ctl to wait for 
            // sensor read mode context has switched to context B
            //

	        int count = 0;
	        while ((mt9m111_reg_read(0x00C8) != 0x000B) && (++count < 20))
	        {
	            //mdelay(10);
                set_current_state(TASK_INTERRUPTIBLE);
                schedule_timeout(1);
            }
        }

    	//mt9m111_update_size(0);
    }
    else
    {
    	//mt9m111_update_size(0);

	// make sure skip mode is on

        ddbg_print("mt9m111_viewfinder_on: making sure skip mode is on");

	    if (!inSkipMode || didSnapPhoto)
	    { 
            ddbg_print("mt9m111_viewfinder_on: skip mode is off -- turning in on");

	    //
	    // set to low power, turn on subsampling
            //     switch to context A for read mode, full frame, and max line delay

    	    mt9m111_write(0x2D2, 0x0000);	// turn off subsampling and turn on full power mode
    	    mt9m111_write(0x2CC, 0x4);		// Context Ctl Program Select -- use default program
    	    u16 v = mt9m111_reg_read(0x2CB);	// read Context Ctl Program Advance
    	    mt9m111_write(0x2CB, v|1);		// advance program

	        inSkipMode = 1;
    	    didSnapPhoto = 0;
	    }
    }

    return MICRON_ERR_NONE;
}

int mt9m111_viewfinder_off()
{
    ddbg_print("mt9m111_viewfinder_off");
    return MICRON_ERR_NONE;
}

int mt9m111_snapshot_trigger()
{
    ddbg_print("mt9m111_snapshot_trigger");

    //mt9m111_update_size(1);

    mt9m111_write(0x2D2, 0x007F);		// Context return to default => all in context B
#ifdef CONFIG_CAMERA_STROBE_FLASH
    if(strobe_flash) {
        /* enable strobe flash */
        // Program Select = default => run program to LED flash
        mt9m111_write(0x2CC, 0x0001);
    }
    else {
        /* disable strobe flash */
        // Program Select = default => run program to default
        mt9m111_write(0x2CC, 0x0004);
    }
#else
    // Program Select = default => run program to default
    mt9m111_write(0x2CC, 0x0004);
#endif
    u16 v = mt9m111_reg_read(0x2CB);		
    mt9m111_write(0x2CB, v | 1);	        // Program Advance = RUN/GO

    didSnapPhoto = 1;

    return MICRON_ERR_NONE;
}

int mt9m111_snapshot_complete()
{
    ddbg_print("mt9m111_snapshot_complete");
    return MICRON_ERR_NONE;
}

int mt9m111_sensor_size(micron_window_size * window)
{
    sensorAWidth = window->width;
    sensorAHeight = window->height;

    ddbg_print("sensor A: %dx%d", sensorAWidth, sensorAHeight);
    /* set the sensor size */ 

    //context A
    mt9m111_write(0x1A6, sensorAWidth);
    mt9m111_write(0x1A9, sensorAHeight);

    return MICRON_ERR_NONE;
}

int mt9m111_capture_sensor_size(micron_window_size * window)
{
    sensorBWidth = window->width;
    sensorBHeight = window->height;

    ddbg_print("sensor B: %dx%d", sensorBWidth, sensorBHeight);
    /* set the sensor size */ 

    //context B
    mt9m111_write(0x1A0, sensorBWidth);
    mt9m111_write(0x1A3, sensorBHeight);

    return MICRON_ERR_NONE;
}

int mt9m111_output_size(micron_window_size * window)
{
    outWidth = window->width;
    outHeight = window->height;
    ddbg_print("preview: %dx%d", outWidth, outHeight);

    //context A
    mt9m111_write(0x1A7, outWidth);
    mt9m111_write(0x1AA, outHeight);

    return MICRON_ERR_NONE;
}

int mt9m111_capture_size(micron_window_size * window)
{
    captureWidth = window->width;
    captureHeight = window->height;
    ddbg_print("capture: %dx%d", captureWidth, captureHeight);

    //context B
    mt9m111_write(0x1A1, captureWidth);
    mt9m111_write(0x1A4, captureHeight);

    return MICRON_ERR_NONE;
}

int mt9m111_set_fps(u16 newMaxFPS, u16 newMinFPS)
{
    ddbg_print("mt9m111_set_fps: newMaxFPS=%d, newMinFPS=%d", newMaxFPS, newMinFPS);

    //
    // clamp newMaxFPS to a legal range
    //

    const int MIN_FPS = 5;
    const int MAX_FPS = 15;

    newMaxFPS = (newMaxFPS > MAX_FPS) ? MAX_FPS : newMaxFPS;
    newMaxFPS = (newMaxFPS < MIN_FPS) ? MIN_FPS : newMaxFPS;

    ddbg_print("mt9m111_set_fps: newMaxFPS=%d, newMinFPS=%d", newMaxFPS, newMinFPS);

    //
    // if the maxFPS has changed, update the vblank
    //

    if (newMaxFPS != maxFPS)
    {

        //
        // enforce max fps by setting the vertical blanking
        //
        // The table below assumes:
        //    MCLK = 26 MHz
        //    target line time = 119 micro seconds
        //    HBLANK in low power = 126
        //    HBLANK in full power = 260
        //
        // The following values came from Micron during a site visit on 9/1.
        //
        // If the input clock changes, these settings also need to change.
        //
        // The basic idea is that a flicker-detection friendly framerate close
        // to the requested frame rate is chosen.
        //

        const u16 vblanksLowPower[] = 
        { 
            1123,	/* 5 : 5.11   */ 
            782, 	/* 6 : 6.45   */
            627, 	/* 7 : 7.32   */ 
            474, 	/* 8 : 8.45   */
            364, 	/* 9 : 9.5    */
            284, 	/* 10: 10.45  */
            207, 	/* 11: 11.55  */
            166, 	/* 12: 12.25  */
            118, 	/* 13: 13.17  */
            71, 	/* 14: 14.21  */
            18   	/* 15: 15.61  */
        };

        const u16 vblanksFullPower[] = 
        { 
            611, 	/* 5 : 5.11 */  
            270, 	/* 6 : 6.45 */  
            115,        /* 7 : 7.32 */ 
            5, 	        /* 8 : 8.10 */  
            5, 	        /* 9 : 8.10 */ 
            5, 	        /* 10: 8.10 */
            5, 	        /* 11: 8.10 */
            5, 	        /* 12: 8.10 */ 
            5, 	        /* 13: 8.10 */ 
            5, 	        /* 14: 8.10 */ 
            5 		/* 15: 8.10 */
        };

        //
        // use the following hblanks to achieve a line rate of 127us
        //

        const u16 readModeLowPower = 0x040C;
        const u16 readModeFullPower = 0x0300;
        const u16 hblankLowPower = 126;
        const u16 hblankFullPower = 260;
        const u16 aeLineSizeLowPower = 1548;
        const u16 aeLineSizeFullPower = 1548;
        const u16 aeShutterDelayLimitLowPower = 1548;
        const u16 aeShutterDelayLimitFullPower = 1548;
        const u16 aeFlickerFullFrameTime60HzLowPower = 280;
        const u16 aeFlickerFullFrameTime60HzFullPower = 280;
        const u16 aeFlickerFullFrameTime50HzLowPower = 336;
        const u16 aeFlickerFullFrameTime50HzFullPower = 336;
        const u16 flickerSearchRange50HZ = 0x1712;
        const u16 flickerSearchRange60HZ = 0x120D;
        // const u16 flickerParameter = 0x1E1C; // 7/8 trials
        const u16 flickerParameter = 0x0D1C; 	// 3/4 trials

        mt9m111_write(0x005, hblankFullPower);
        mt9m111_write(0x006, vblanksFullPower[newMaxFPS-5]);
        mt9m111_write(0x007, hblankLowPower);
        mt9m111_write(0x008, vblanksLowPower[newMaxFPS-5]);
        //mt9m111_write(0x020, readModeFullPower);
        mt9m111_reg_write_020(readModeFullPower);
        mt9m111_write(0x021, readModeLowPower);
        mt9m111_write(0x239, aeLineSizeLowPower);
        mt9m111_write(0x23A, aeLineSizeFullPower);
        mt9m111_write(0x23B, aeShutterDelayLimitLowPower);
        mt9m111_write(0x23C, aeShutterDelayLimitFullPower);
        mt9m111_write(0x257, aeFlickerFullFrameTime60HzLowPower);
        mt9m111_write(0x258, aeFlickerFullFrameTime50HzLowPower);
        mt9m111_write(0x259, aeFlickerFullFrameTime60HzFullPower);
        mt9m111_write(0x25A, aeFlickerFullFrameTime50HzFullPower);
        mt9m111_write(0x25C, flickerSearchRange60HZ);
        mt9m111_write(0x25D, flickerSearchRange50HZ);
        mt9m111_write(0x264, flickerParameter);
 
        maxFPS = newMaxFPS;
    }

    return MICRON_ERR_NONE;

}

int mt9m111_set_autoexposure_zone(u16 newMinFPS)
{
    const int MIN_FPS = 5;
    const int MAX_FPS = 15;

    if ( newMinFPS == minFPS ) {
        return MICRON_ERR_NONE ;
    }

    if ( g_earlySetAE ) 
    {
        /* clear global flag */
        g_earlySetAE = 0 ;

        if ( 15 == newMinFPS )
        {
            return MICRON_ERR_NONE ;
        }
    }
    //
    // This driver only implements two minimum framerates: 5fps and 15fps.
    //
    // The mt9m111 can support every min fps between 5fps and 15fps, but the AE settings 
    // will become very complex.
    // 

    if (newMinFPS > 5)
    {
        newMinFPS = MAX_FPS;
    }
    else
    {
        newMinFPS = MIN_FPS;
    }

    // if the minFPS has changed, update the AE limit
    //

    if (newMinFPS != minFPS)
    {
        //
        // enforce min fps by setting max exposure time
        //

        if (newMinFPS == 5)
        {
            // set min fps to 5 fps -> AE Zone 24
            u16 limitAE = ((120/5) << 5) & 0x03E0;
            mt9m111_write(0x237, limitAE);

            //
            // work-around for AE strangeness
            // symptom: start in dark, preview gets darker, darker, very dark, black, then is OK.
            //

            if (mt9m111_reg_read(0x23F) == 8)
            {
                u16 v;
                v = mt9m111_reg_read(0x22E);	// read target luma register

                // by setting target luma to 0, trick AE into zone <= 8
                mt9m111_write(0x22E, 0x0000);	// set target luma to 0

                //
                // wait until AE zone is in bounds (or time exceeds 1500ms)
                //
                // 1500ms is a long time!
                //

                int count=0;
                while ((mt9m111_reg_read(0x23F) == 8) && (++count < 30))
                {
                    //mdelay(50);
                    set_current_state(TASK_INTERRUPTIBLE);
                    schedule_timeout(5);
                }

                mt9m111_write(0x22E, v);	// rewrite target luma register
            }
        }
        else
        {
            // set min fps to 15 fps -> AE Zone 8
            u16 limitAE = ((120/15) << 5) & 0x03E0;
            mt9m111_write(0x237, limitAE);
            ddbg_print("AE zone limit is %0x\n", limitAE);

            //
            // work-around for AE strangeness
            // symptom: start in dark, preview gets darker, darker, very dark, black, then is OK.
            //

            if (mt9m111_reg_read(0x23F) > 8)
            {
                u16 v;
                v = mt9m111_reg_read(0x22E);	// read target luma register

                // by setting target luma to 0, trick AE into zone <= 8
                mt9m111_write(0x22E, 0x0000);	// set target luma to 0

                //
                // wait until AE zone is in bounds (or time exceeds 1500ms)
                //
                // 1500ms is a long time!
                //

                int count=0;
                while ((mt9m111_reg_read(0x23F) > 8) && (++count < 30))
                {
                    //mdelay(50);
                    set_current_state(TASK_INTERRUPTIBLE);
                    schedule_timeout(5);

                }

                mt9m111_write(0x22E, v);	// rewrite target luma register
            }
        }

        minFPS = newMinFPS;
    }

    return MICRON_ERR_NONE;
}

int mt9m111_output_format(u16 format)
{
    u16 value;

    if(format == O_FORMAT_422_YCbYCr)
    {
        mt9m111_write(0x13A, 0);
    }
    else
    {
        value = (1<<8)|((format-1)<<6);
        mt9m111_write(0x13A, value);
    }
    return MICRON_ERR_NONE;
}

void mt9m111_dump_register(u16 startRegAddr, u16 endRegAddr, u16* buffer)
{
    u16 addr,value;
	
    for(addr = startRegAddr; addr <= endRegAddr; addr++)
    {
        unsigned long flags;
        local_irq_save(flags);
	   
        mt9m111_read(addr, &value);
	  
        local_irq_restore(flags);

        dbg_print("mt9m111 register: 0x%04x = %04x",addr,value);
	   
        if(buffer)
        {
            *buffer = value;
            buffer++;
        }
    }
}

//
// TODO: make this an IOCTL that is called by the application some time after set_flicker() is
// called
//

int mt9m111_set_flicker_2(void)
{
    ddbg_print("mt9m111_set_flicker_2: %04x", preferredFlickerMask);

    int count=0;

    //
    // read flicker_control_1 until it verifies that flicker is set to the preferred rate.
    //

    while(((mt9m111_reg_read(0x25B) & 0x8000) != preferredFlickerMask) && (++count < 50))
    {
        //mdelay(20);
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(2);
    }

    mt9m111_write(0x25B, 0x0002);	// back to auto mode

    return MICRON_ERR_NONE;
}

int mt9m111_set_flicker(int flicker)
{
    dbg_print("mt9m111_set_flicker: %d", flicker);

    if ( g_flicker == flicker ) {
        return MICRON_ERR_NONE ;
    }

    preferredFlickerMask = (flicker == 50) ? 0x0000 : 0x8000;

    if (flicker == 50)
    {
        /* g_flicker is 0 means first call set_flicker after
        *   early_set_flicker is called
        */
        if ( 60 == g_flicker ) {
            mt9m111_write(0x25B, 0x0001);   // manual mode 50Hz
        }
    }
    else if (flicker == 60)
    {
        mt9m111_write(0x25B, 0x0003);   // manual mode 60Hz
    }
    else
    { 
        return MICRON_ERR_PARAMETER;
    }

    mt9m111_set_flicker_2();

    g_flicker = flicker ;

    return MICRON_ERR_NONE;
}

int mt9m111_set_contrast(int contrast)
{
    ddbg_print("mt9m111_set_contrast: %d (skipped)", contrast);
    return MICRON_ERR_NONE;

#if 0
    //
    // we don't support setting the contrast in the initial release of A780
    //

    const u16 gammaTable[] = 
    {
        //0x3B2C,0x664D,0x9F87,0xC3B3,0xE0D2, //0.40
        //0x3224,0x664D,0x9F87,0xC3B3,0xE0D2, //0.45
        0x2A1D,0x543B,0x9277,0xBDA9,0xE0CF, //0.50
        0x2318,0x4C34,0x8C70,0xBAA4,0xE0CD, //0.55
        0x1E14,0x462E,0x876A,0xB7A0,0xE0CC, //0.60
        0x1910,0x3F28,0x8163,0xB39B,0xE0CA, //0.65
        0x150D,0x3923,0x7B5D,0xB097,0xE0C9, //0.70
        0x120A,0x341E,0x7657,0xAE93,0xE0C7, //0.75
        0x0F08,0x2F1B,0x7152,0xAB8F,0xE0C6, //0.80
        0x0D07,0x2A17,0x6D4D,0xA88B,0xE0C4, //0.85
        0x0B05,0x2614,0x6848,0xA587,0xE0C2, //0.90
        //0x0804,0x2010,0x6040,0xA080,0xE0C0, //1.00
        //0x0502,0x1A0C,0x5838,0x9A79,0xE0BD, //1.10
        //0x0401,0x1509,0x5131,0x9572,0xE0BA, //1.20
    };

    if(contrast < -4 || contrast > 4)
    {
        return MICRON_ERR_PARAMETER;
    }

    u16 i, regA=0x153, regB=0x1DC;

    for(i=0; i<5; i++)
    {
        mt9m111_write(regA++, gammaTable[(contrast+4)*5+i]);
        mt9m111_write(regB++, gammaTable[(contrast+4)*5+i]);
    } 

    return MICRON_ERR_NONE;
#endif

}

/*set picture style(normal/black white/sepia/solarize/neg.art)*/
int mt9m111_set_style(V4l_PIC_STYLE style)
{
    ddbg_print("mt9m111_set_style: %d", style);

  switch(style)
  {
  case V4l_STYLE_BLACK_WHITE:
       mt9m111_write(0x1E2,0x7001);
       break;
  case V4l_STYLE_SEPIA:
       mt9m111_write(0x1E2,0x7002);
       mt9m111_write(0x1E3,0xB023);
       break;
  case V4l_STYLE_SOLARIZE:
       mt9m111_write(0x1E2,0x7004);
       break;
  case V4l_STYLE_NEG_ART:
       mt9m111_write(0x1E2,0x7003);
       break;

    case V4l_STYLE_BLUISH:
        mt9m111_write( 0x1E2 , 0x7002 ) ;
        mt9m111_write( 0x1E3 , 0x40a0 ) ;
        break ;

    case V4l_STYLE_REDDISH:
        mt9m111_write( 0x1E2 , 0x7002 ) ;
        mt9m111_write( 0x1E3 , 0xa040 ) ;
        break ;

    case V4l_STYLE_GREENISH:
        mt9m111_write( 0x1E2 , 0x7002 ) ;
        mt9m111_write( 0x1E3 , 0xc0c0 ) ;
        break ;

  default:
       mt9m111_write(0x1E2,0x7000);
       break;
  }

  return MICRON_ERR_NONE;
}

        
/*set picture light(auto/direct sun/incandescent/fluorescent)*/     
int mt9m111_set_light(V4l_PIC_WB light)
{ 
    u16 *m;
    u16 value, v106;
    int awb, i;
    unsigned long   diff_jiffies ;

    if ( g_light == light ) {
        return MICRON_ERR_NONE ;
    }

    awb = 0;
    switch(light)
    {
        case V4l_WB_DIRECT_SUN:

            ddbg_print("mt9m111_set_light: %d (direct sun)", light);
            m = (u16 *)d65Light;
            break;       

        case V4l_WB_CLOUDY:

            ddbg_print( "mt9m111_set_light: %d (cloudy)", light ) ;
            m = ( u16 * ) cloudyLight ;
            break ;

        case V4l_WB_INCANDESCENT:

            ddbg_print("mt9m111_set_light: %d (incandescent)", light);
            m = (u16 *)incandescentLight;
            break;       

        case V4l_WB_FLUORESCENT:

            ddbg_print("mt9m111_set_light: %d (fluorescent)", light);
            m = (u16 *)tl84Light;
            //awb = 1;
            break;

        default:
    
            ddbg_print("mt9m111_set_light: %d (default)", light);
            m = (u16 *)autoLight;
            awb = 1;
            break;

    }

    if ( g_earlyAWB ) {
        diff_jiffies = jiffies - g_ulAWBJiffies ;
        if ( ( diff_jiffies ) < 30 ) {
            set_current_state( TASK_INTERRUPTIBLE ) ;
            schedule_timeout( 30 - diff_jiffies ) ;
        }
        g_earlyAWB = 0 ;    /* clear early AWB flag */
    } else {
        DisableAWB() ;
        set_current_state( TASK_INTERRUPTIBLE ) ;
        schedule_timeout( 30 ) ;
    }

    v106 = mt9m111_reg_read(0x106);

    mt9m111_write(0x202, m[0]);  //BASE_MATRIX_SIGNS
    mt9m111_write(0x209, m[3]);  //BASE_MATRIX_COEF_K1
    mt9m111_write(0x20A, m[4]);  //BASE_MATRIX_COEF_K2
    mt9m111_write(0x20B, m[5]);  //BASE_MATRIX_COEF_K3
    mt9m111_write(0x20C, m[6]);  //BASE_MATRIX_COEF_K4
    mt9m111_write(0x20D, m[7]);  //BASE_MATRIX_COEF_K5
    mt9m111_write(0x20E, m[8]);  //BASE_MATRIX_COEF_K6
    mt9m111_write(0x20F, m[9]);  //BASE_MATRIX_COEF_K7
    mt9m111_write(0x210, m[10]); //BASE_MATRIX_COEF_K8
    mt9m111_write(0x211, m[11]); //BASE_MATRIX_COEF_K9
    mt9m111_write(0x215, m[12]); //DELTA_COEF_SIGNS
    mt9m111_write(0x216, m[13]); //DELTA_MATRIX_COEF_D1
    mt9m111_write(0x217, m[14]); //DELTA_MATRIX_COEF_D2
    mt9m111_write(0x218, m[15]); //DELTA_MATRIX_COEF_D3
    mt9m111_write(0x219, m[16]); //DELTA_MATRIX_COEF_D4
    mt9m111_write(0x21A, m[17]); //DELTA_MATRIX_COEF_D5
    mt9m111_write(0x21B, m[18]); //DELTA_MATRIX_COEF_D6
    mt9m111_write(0x21C, m[19]); //DELTA_MATRIX_COEF_D7
    mt9m111_write(0x21D, m[20]); //DELTA_MATRIX_COEF_D8
    mt9m111_write(0x21E, m[21]); //DELTA_MATRIX_COEF_D9
    mt9m111_write(0x25E, m[22]); //RATIO_BASE_REG
    mt9m111_write(0x25F, m[23]); //RATIO_DELTA_REG
    mt9m111_write(0x260, m[24]); //SIGNS_DELTA_REG
    mt9m111_write(0x203, m[1]);  //BASE_MATRIX_SCALE_K1_K5
    mt9m111_write(0x204, m[2]);  //BASE_MATRIX_SCALE_K6_K9

    //Toggle Manual White Balance to force loading the new table
    if(awb)
    {   // Auto white balance
        /* Set WB red/blue gains to unity */
        mt9m111_read(0x148, &value);
        mt9m111_write(0x148, (value & ~0x0080));

        /* Disable the AWB bit */
        /* Set the Manual WB bit to load the new settings */
        v106 = (v106 | 0x8000) & ~0x0002;
        mt9m111_write(0x106, v106);

        i = 0;
        do
        {
            mt9m111_read(0x261, &value );
            ddbg_print("R261(0x%04x)", value);
            //mdelay(70);
            set_current_state(TASK_INTERRUPTIBLE);
            schedule_timeout(7);
        }while((value != m[22]) && ((++i) < 100));

        /* Clear the Manual WB bit */
        /* Set the AWB bit */
        v106 = (v106 & ~0x8000) | 0x0002;
        mt9m111_write(0x106, v106);
    }
    else {
        /* Set WB red/blue gains to unity */
        mt9m111_read(0x148, &value);
        mt9m111_write(0x148, (value | 0x0080));

        /* Disable the AWB bit */
        /* Set the Manual WB bit to load the new settings */
        v106 = (v106 | 0x8000) & ~0x0002;
        mt9m111_write(0x106, v106);
    }

    return MICRON_ERR_NONE;
}

    
/*set picture brightness*/
int mt9m111_set_bright(int bright)
{
    ddbg_print("mt9m111_set_bright (gamma) %d", bright);

    const u16 target[] = 
    { 
        21,      // -1.7 EV     *2^(-1.7) -> *0.3078
        28,      // -1.3 EV     *2^(-1.3) -> *0.4061
        34,      // -1.0 EV     *2^(-1.0) -> *0.5
        48,      // -0.5 EV     *2^(-0.5) -> *0.7071
        68,      //  0.0 EV     *1.0
        96,      // +0.5 EV     *2^(+0.5) -> *1.4142
        136,     // +1.0 EV     *2^(+1.0) -> *2.0
        167,     // +1.3 EV     *2^(+1.3) -> *2.4623
        221      // +1.7 EV     *2^(+1.7) -> *3.2490
    };

    if(bright < -4 || bright > 4)
    {
        return MICRON_ERR_PARAMETER;
    }

    /*set luma value of AE target*/
    mt9m111_write(0x22E, 0x0500 + target[bright+4]);
    return MICRON_ERR_NONE;
}

/*set picture mirroring*/
int mt9m111_set_mirror(int rows, int columns)
{
    u16 reg;
    if(rows==0)
        mirror_rows = rotate_mirror_rows;   // set to default rotate mirror
    else
        mirror_rows = (!rotate_mirror_rows)&0x01;

    if(columns==0)
        mirror_columns = rotate_mirror_columns;
    else
        mirror_columns = (!rotate_mirror_columns)&0x01;

    ddbg_print("set mirror rows:%d columns:%d, sensor mirror rows:%d columns:%d",
                rows, columns, mirror_rows, mirror_columns);

    reg = mt9m111_reg_read(0x020);
    mt9m111_reg_write_020(reg);

    return 0;
}

/*set strobe flash*/
int mt9m111_set_strobe_flash(int strobe)
{
    strobe_flash = strobe;
    return 0;
}

static void mt9m111_lens_shading_settings(void)
{
    u16 value;

    // lens correction for Micron es2 rev2 + flex module
    mt9m111_write(0x180, 0x0007);    // LENS_ADJ_PARAM_0_5
    mt9m111_write(0x181, 0xD810);    // LENS_ADJ_VERT_RED_0
    mt9m111_write(0x182, 0xF0E6);    // LENS_ADJ_VERT_RED_1_2
    mt9m111_write(0x183, 0x01F9);    // LENS_ADJ_VERT_RED_3_4
    mt9m111_write(0x184, 0xE20C);    // LENS_ADJ_VERT_GREEN_0
    mt9m111_write(0x185, 0xF6EC);    // LENS_ADJ_VERT_GREEN_1_2
    mt9m111_write(0x186, 0x00FA);    // LENS_ADJ_VERT_GREEN_3_4
    mt9m111_write(0x187, 0xE00D);    // LENS_ADJ_VERT_BLUE_0
    mt9m111_write(0x188, 0xF4EA);    // LENS_ADJ_VERT_BLUE_1_2
    mt9m111_write(0x189, 0xFFFA);    // LENS_ADJ_VERT_BLUE_3_4
    mt9m111_write(0x18A, 0xBB21);    // LENS_ADJ_HORIZ_RED_0
    mt9m111_write(0x18B, 0xE4D2);    // LENS_ADJ_HORIZ_RED_1_2
    mt9m111_write(0x18C, 0xF7EE);    // LENS_ADJ_HORIZ_RED_3_4
    mt9m111_write(0x18D, 0x0002);    // LENS_ADJ_HORIZ_RED_5
    mt9m111_write(0x18E, 0xD816);    // LENS_ADJ_HORIZ_GREEN_0
    mt9m111_write(0x18F, 0xECE2);    // LENS_ADJ_HORIZ_GREEN_1_2
    mt9m111_write(0x190, 0xF8F2);    // LENS_ADJ_HORIZ_GREEN_3_4
    mt9m111_write(0x191, 0x00FF);    // LENS_ADJ_HORIZ_GREEN_5
    mt9m111_write(0x192, 0xD713);    // LENS_ADJ_HORIZ_BLUE_0
    mt9m111_write(0x193, 0xF0E7);    // LENS_ADJ_HORIZ_BLUE_1_2
    mt9m111_write(0x194, 0xFAF5);    // LENS_ADJ_HORIZ_BLUE_3_4
    mt9m111_write(0x195, 0x0001);    // LENS_ADJ_HORIZ_BLUE_5
    mt9m111_write(0x1B6, 0x1109);    // LENS_ADJ_VERT_RED_5_6
    mt9m111_write(0x1B7, 0x3C20);    // LENS_ADJ_VERT_RED_7_8
    mt9m111_write(0x1B8, 0x0B06);    // LENS_ADJ_VERT_GREEN_5_6
    mt9m111_write(0x1B9, 0x2914);    // LENS_ADJ_VERT_GREEN_7_8
    mt9m111_write(0x1BA, 0x0A03);    // LENS_ADJ_VERT_BLUE_5_6
    mt9m111_write(0x1BB, 0x2312);    // LENS_ADJ_VERT_BLUE_7_8
    mt9m111_write(0x1BC, 0x1409);    // LENS_ADJ_HORIZ_RED_6_7
    mt9m111_write(0x1BD, 0x3221);    // LENS_ADJ_HORIZ_RED_8_9
    mt9m111_write(0x1BE, 0x0058);    // LENS_ADJ_HORIZ_RED_10
    mt9m111_write(0x1BF, 0x0E06);    // LENS_ADJ_HORIZ_GREEN_6_7
    mt9m111_write(0x1C0, 0x1E16);    // LENS_ADJ_HORIZ_GREEN_8_9
    mt9m111_write(0x1C1, 0x0034);    // LENS_ADJ_HORIZ_GREEN_10
    mt9m111_write(0x1C2, 0x0B05);    // LENS_ADJ_HORIZ_BLUE_6_7
    mt9m111_write(0x1C3, 0x1B12);    // LENS_ADJ_HORIZ_BLUE_8_9
    mt9m111_write(0x1C4, 0x002C);    // LENS_ADJ_HORIZ_BLUE_10
    
    //turn on lens correction
    mt9m111_read(0x106, &value);   // Operating Mode Control -- P.13 0x708E 
    value |= 0x0400;
    mt9m111_write(0x106, value);
}

int mt9m111_default_settings()
{
    ddbg_print("mt9m111_default_settings");

    mt9m111_reset();

    // Soft Reset -- Reset both sensor AND SOC (and leave sensor digital logic enabled)

    mt9m111_write(0x00D, 0x29);   // Reset sensor and SOC -- P.54
    // i2c will delay enough here for reset to finish (1us is enough)
    mt9m111_write(0x00D, 0x08);   // Release reset -- P.54

    mt9m111_early_setting() ;

    /* Row Noise control part in and around day mode */
    mt9m111_write(0x23D, 0x17DD);
    mt9m111_write(0x05F, 0x3630);
    mt9m111_write(0x030, 0x043E);
    mt9m111_write(0x13B, 0x043E);
 
    /* Configure the part for the night mode to get rid of AE strangeness issue */
    /* Transition zone = 8 */
    mt9m111_write(0x238, 0x0840);   // Shutter priority transition zone
    /* AE gains table updates */
    mt9m111_write(0x282, 0x03FE);
    mt9m111_write(0x285, 0x0061);   // AE Gain table 
    mt9m111_write(0x286, 0x0080);   // AE Gain table
    mt9m111_write(0x287, 0x0061);   // AE Gain table
    mt9m111_write(0x288, 0x0061);   // AE Gain table
    mt9m111_write(0x289, 0x03E2);   // AE Gain table

    /* Set ambient lighting to default  (automatic) */
    //mt9m111_set_light(V4l_WB_AUTO); //the app will call ioctl to set light

    //Enable flicker detection
    /* mt9m111_write(0x106, 0x708E);*/   // Operating Mode Control -- P.13 0x0708C

    // Enable 2D defect correction in both contexts 
    mt9m111_write(0x14C, 0x0001);   // Defect Buffer Context A -- P.13 0x0000 -- Enable 2D defect correct context A 
    mt9m111_write(0x14D, 0x0001);   // Defect Buffer Context B -- P.13 0x0000 -- Enable 2D defect correct context B 

    /* Enable Eclipse mode  */
    //mt9m111_write(0x034, 0xC039);   // Reserved -- P.54 0xC019 in es3 -- turn on eclipse

    /* Output Mode Select:
       Do not invert the pixel clock
       YCbCr 4:2:2 output, 16 bits per pixel */
    mt9m111_write(0x13A, 0x0000);   // Output Format Control 2A -- P.13 0x0000 
    mt9m111_write(0x19B, 0x0000);   // Output Format Control 2B -- P.14 0x0000

    // Set centered origin 
    mt9m111_write(0x1a8, 0x4000);   // Reducer YPan A -- P.14 0x0000 -- Cntxt A Y Crop centering 
    mt9m111_write(0x1a5, 0x4000);   // Reducer XPan A -- P.14 0x0000 -- Cntxt A X Crop centering 
    mt9m111_write(0x1a2, 0x4000);   // Reducer YPan B -- P.14 0x0000 -- Cntxt B Y Crop centering 
    mt9m111_write(0x19F, 0x4000);   // Reducer XPan B -- P.14 0x0000 -- Cntxt B X Crop centering 

    // Sensor is running in semi auto mode only 
    mt9m111_write(0x2CC, 0x0004);   // Context Ctl Pgm Select -- P.19 0x0000 

    //Auto sharpening
    mt9m111_write(0x105, 0x000B);   // Aperature Correction (sharpening) gain -- 0x0003 
                                    // 75% sharpening, auto-sharpening on
    //Enable classic interpolation at full res
    mt9m111_write(0x1AF, 0x0018);   // Reducer Zoom Control -- P.14 0x0010
    
    //AWB settings for 12mm lens, no ulens shift
    mt9m111_write(0x21F, 0x0090);
    mt9m111_write(0x222, 0xb060);   // Red Gain AWB Limits -- P.16 0x0D960 -- overconstrained currently? (sunburned->0x8870)
    mt9m111_write(0x223, 0xb070);   // Blue Gain AWB Limits -- P.16 0x0D960 -- overconstrained currently?
    mt9m111_write(0x228, 0xEF02);   // AWB gain control speed  (keep gain low for stable AWB)
    //mt9m111_write(0x229, 0x847C);   // Reserved -- P.16 0x8D73 -- Thresholds (tied into digital gains above)

    // set luma offset and clip for both contexts
    mt9m111_write(0x134, 0x0000);   // Luma offset (Brightness Offset)
    mt9m111_write(0x135, 0xFF01);   // Luma clip (Brightness Offset)

    // Widen the AE Center Boundaries to reduce overexposure
    mt9m111_write(0x22B, 0x8000);   // AE Center Horizontal boundary
    mt9m111_write(0x22C, 0x8000);   // AE Center Vertical boundary

    // set Saturation control
    mt9m111_write(0x125, 0x002D);   // Saturation 150%

    /* load hard-coded 0.45 gamma table (Context A)
       and black correction set to 13 
       and JPEG range */
    mt9m111_write(0x153, 0x0F04);
    mt9m111_write(0x154, 0x592D);
    mt9m111_write(0x155, 0xA988);
    mt9m111_write(0x156, 0xD9C3);
    mt9m111_write(0x157, 0xFFED);
    mt9m111_write(0x158, 0x0000);
    // load hard-coded 0.6 gamma table (Context B)
    mt9m111_write(0x1DC, 0x0F04);
    mt9m111_write(0x1DD, 0x592D);
    mt9m111_write(0x1DE, 0xA988);
    mt9m111_write(0x1DF, 0xD9C3);
    mt9m111_write(0x1E0, 0xFFED);
    mt9m111_write(0x1E1, 0x0000);

    /* Load and Enable lens shading settings */
    mt9m111_lens_shading_settings();

    // setup strobe flash led
#ifdef CONFIG_CAMERA_STROBE_FLASH
    mt9m111_write(0x2CE, 0x3E9B);
#endif

    return MICRON_ERR_NONE;
}

