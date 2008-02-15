/*
 *  mi2010soc_hw.c
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
Mu Chenliang        4/30/2005                   Created, for EZX platform
 *
*/

/*
 * ==================================================================================
 *                                  INCLUDE FILES
 * ==================================================================================
 */

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

#include "mi2010soc_hw.h"
#include "camera.h"


extern int i2c_mi2010soc_read(u16 addr, u16 *pvalue);
extern int i2c_mi2010soc_write(u16 addr, u16 value);

static int sensor_id = -1;              // ID=0x1519 MT9D111(MI2010SOC)
static int sensor_revision = -1;        // 1=Rev1, 2=Rev2, 3=Rev3
static int sensor_pll = 0;              // 1=enable PLL

static u16 sensorAWidth = MAX_SENSOR_A_WIDTH;
static u16 sensorAHeight = MAX_SENSOR_A_HEIGHT;
static u16 sensorBWidth = MAX_SENSOR_B_WIDTH;
static u16 sensorBHeight = MAX_SENSOR_B_HEIGHT;
static u16 outWidth = DEFT_SENSOR_A_WIDTH;
static u16 outHeight = DEFT_SENSOR_A_HEIGHT;
static u16 captureWidth = DEFT_SENSOR_B_WIDTH;
static u16 captureHeight = DEFT_SENSOR_B_HEIGHT;
static u16 spoofWidth = MAX_SENSOR_B_WIDTH/2;
static u16 spoofHeight = MAX_SENSOR_B_HEIGHT;

static int sensorContext;               // =0, Context A; =1, Context B
static int sensorShouldRefresh;         // =1, Should refresh
static int sensorShouldRefreshMode;     // =1, Should refresh mode
static int sensorRunning;               // =1, Running

static int sensor_fps = 15;             // sensor preview FPS
static int sensor_flicker_freq = 60;    // flicker freq for sensor

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

/*****************************************************************************
*
*****************************************************************************/
static void mi2010soc_reset_value(void)
{
    sensorContext = 0;
    sensorShouldRefresh = 0;
    sensorShouldRefreshMode = 0;
    sensorRunning = 0;
    sensor_fps = 15;             // sensor preview FPS
    sensor_flicker_freq = 60;    // flicker freq for sensor
    sensor_pll = 0;
    /* default mirror */
    mirror_rows = rotate_mirror_rows;
    mirror_columns = rotate_mirror_columns;
}

/*****************************************************************************
*									     *
*        I2C Management		 					     *
*									     *
*****************************************************************************/
u16 mi2010soc_reg_read(u16 reg_addr)
{
    u16 value=0;
    i2c_mi2010soc_read(reg_addr, &value);
    ddbg_print("i2c r:(0x%04x) = 0x%04x", reg_addr, value);
    return value;
}

void mi2010soc_reg_write(u16 reg_addr, u16 reg_value)
{
    ddbg_print("i2c w:(0x%04x) = 0x%04x", reg_addr, reg_value);
    i2c_mi2010soc_write(reg_addr, reg_value);
}

#define MI_VAR_WORD         0x0000  // b15=0, 16-bit access
#define MI_VAR_BYTE         0x8000  // b15=1,  8-bit access
#define MI_VAR_LOGICAL      0x2000  // Bits[14:13]=01, logical access
#define MI_VAR_ID_SMASK     0x001F  // Bits[12:8], driver ID
#define MI_VAR_ID_SHIFT     8       // Bits[12:8], driver ID
#define MI_VAR_OFFSET_MASK  0x00FF  // Bits[7:0], driver variable offset

u16 mi2010soc_var_read16(u8 id, u8 offset)
{
    u16 r198, value;
    r198 = MI_VAR_WORD | MI_VAR_LOGICAL | ((id & MI_VAR_ID_SMASK)<<MI_VAR_ID_SHIFT) | offset;
    i2c_mi2010soc_write(0x1c6, r198);
    i2c_mi2010soc_read(0x1c8, &value);
    ddbg_print("%d:%d(%04x) = 0x%04x", id, offset, offset, value);
    return value;
}

u8 mi2010soc_var_read8(u8 id, u8 offset)
{
    u16 r198, value;
    r198 = MI_VAR_BYTE | MI_VAR_LOGICAL | ((id & MI_VAR_ID_SMASK)<<MI_VAR_ID_SHIFT) | offset;
    i2c_mi2010soc_write(0x1c6, r198);
    i2c_mi2010soc_read(0x1c8, &value);
    ddbg_print("%d:%d(%04x) = 0x%02x", id, offset, offset, value);
    return value;
}

void mi2010soc_var_write16(u8 id, u8 offset, u16 value)
{
    u16 r198;
    r198 = MI_VAR_WORD | MI_VAR_LOGICAL | ((id & MI_VAR_ID_SMASK)<<MI_VAR_ID_SHIFT) | offset;
    i2c_mi2010soc_write(0x1c6, r198);
    i2c_mi2010soc_write(0x1c8, value);
    ddbg_print("%d:%d(%04x) <- 0x%04x", id, offset, offset, value);
}

void mi2010soc_var_write8(u8 id, u8 offset, u8 value)
{
    u16 r198;
    r198 = MI_VAR_BYTE | MI_VAR_LOGICAL | ((id & MI_VAR_ID_SMASK)<<MI_VAR_ID_SHIFT) | offset;
    i2c_mi2010soc_write(0x1c6, r198);
    i2c_mi2010soc_write(0x1c8, (u16)value);
    ddbg_print("%d:%d(%04x) <- 0x%02x", id, offset, offset, value);
}

static void mi2010soc_reg_write_020(u16 readmode)
{
    readmode = (readmode & 0xfffc) | mirror_rows | (mirror_columns<<1);
    mi2010soc_reg_write(0x020, readmode);
}

#if 0
static void mi2010soc_dump_reg_sensor(void)
{
    mi2010soc_reg_read(0x001);
    mi2010soc_reg_read(0x002);
    mi2010soc_reg_read(0x003);
    mi2010soc_reg_read(0x004);
    mi2010soc_reg_read(0x005);
    mi2010soc_reg_read(0x006);
    mi2010soc_reg_read(0x007);
    mi2010soc_reg_read(0x008);
    mi2010soc_reg_read(0x009);
    mi2010soc_reg_read(0x00A);
    mi2010soc_reg_read(0x020);
    mi2010soc_reg_read(0x021);
    mi2010soc_reg_read(0x0F2);
}

static void mi2010soc_dump_reg_ifp(void)
{
    mi2010soc_reg_read(0x108);
    mi2010soc_reg_read(0x111);
    mi2010soc_reg_read(0x112);
    mi2010soc_reg_read(0x113);
    mi2010soc_reg_read(0x114);
    mi2010soc_reg_read(0x116);
    mi2010soc_reg_read(0x117);
}

static void mi2010soc_dump_var_sequence(void)
{
    mi2010soc_var_read8(1, 3);
    mi2010soc_var_read8(1, 4);
}

static void mi2010soc_dump_var_mode(void)
{
    mi2010soc_var_read8(7, 2);
    mi2010soc_var_read16(7, 3);
    mi2010soc_var_read16(7, 5);
    mi2010soc_var_read16(7, 7);
    mi2010soc_var_read16(7, 9);
    mi2010soc_var_read16(7, 15);
    mi2010soc_var_read16(7, 17);
    mi2010soc_var_read16(7, 19);
    mi2010soc_var_read16(7, 21);
    mi2010soc_var_read16(7, 27);
    mi2010soc_var_read16(7, 29);
    mi2010soc_var_read16(7, 31);
    mi2010soc_var_read16(7, 33);
    mi2010soc_var_read16(7, 39);
    mi2010soc_var_read16(7, 41);
    mi2010soc_var_read16(7, 43);
    mi2010soc_var_read16(7, 45);
    mi2010soc_var_read16(7, 53);
    mi2010soc_var_read16(7, 55);
    mi2010soc_var_read16(7, 57);
    mi2010soc_var_read16(7, 59);
}

static void mi2010soc_dump_debug_regs(void)
{
    mi2010soc_dump_reg_sensor();
    mi2010soc_dump_reg_ifp();
    mi2010soc_dump_var_sequence();
    mi2010soc_dump_var_mode();
}
#endif

/////////////////////////////////////////////////////////////////////////////////////
//   
//  Programming Guide : Configuration Methods 
//
/////////////////////////////////////////////////////////////////////////////////////

int mi2010soc_get_device_id(u16 *id, u16 *rev)
{
    u16 value;
    if(sensor_id==-1)
    {
        /*Device ID is in register 0x0 */
        value = mi2010soc_reg_read(0x0);
        sensor_id = value;
    }
    if(id!=NULL)
        *id = sensor_id;

    if(sensor_revision==-1)
    {
        value = mi2010soc_reg_read(0x0F9);
        value |= 0x0080;
        mi2010soc_reg_write(0x00F9, value);
        value = mi2010soc_reg_read(0x0FD);
        value = (value&0x0FE0) >> 5;
        sensor_revision = value;
    }
    if(rev!=NULL)
        *rev = sensor_revision;

    //ddbg_print("mi2010soc device ID is 0x%x, rev%d", sensor_id, sensor_revision);       
    return MICRON_ERR_NONE;
}

static void mi2010soc_wait_state(int state)
{
    int cur_state;

    cur_state = mi2010soc_var_read8(1, 4);        // SEQ_STATE
    while(cur_state!=state) {                     // is desired state?
        cur_state = mi2010soc_var_read8(1, 4);    // SEQ_STATE
    }
}

static int mi2010soc_set_mode(int context)
{
    /* set the sensor context */

    if(context == 0)
    {
        // VAR8 = 1, 32, 0x0000     // SEQ_CAP_MODE
        mi2010soc_var_write8(1, 32, 0);
        // VAR8 = 1, 3, 0x0001      // SEQ_CMD Do Preview
        mi2010soc_var_write8(1, 3, 1);

        // wait preview state
        mi2010soc_wait_state(3);

        sensorContext = 0;
    }
    else
    {
        // VAR8 = 1, 32, 0x0002     // SEQ_CAP_MODE
        mi2010soc_var_write8(1, 32, 0x0002);
        // VAR8 = 1, 3, 0x0002      // SEQ_CMD Do Capture
        mi2010soc_var_write8(1, 3, 2);

        // wait capture state
        mi2010soc_wait_state(7);

        sensorContext = 1;
    }

    return MICRON_ERR_NONE;
}

static void mi2010soc_cmd_refresh(void)
{
    int cmd, state;
    state = mi2010soc_var_read8(1, 4);                  // SEQ_STATE

    // VAR8 = 1, 3, 0x0005      // SEQ_CMD Refresh
    mi2010soc_var_write8(1, 3, 5);

    cmd = mi2010soc_var_read8(1, 3);            // SEQ_CMD
    while(cmd!=0) {                             // is end?
        cmd = mi2010soc_var_read8(1, 3);        // SEQ_CMD
    }
    sensorShouldRefresh = 0;
}

static void mi2010soc_cmd_refresh_mode(void)
{
    int cmd, state;
    state = mi2010soc_var_read8(1, 4);                  // SEQ_STATE

    // VAR  = 1, 3, 0x006        //Refresh Sequencer Mode = 6
    mi2010soc_var_write8(1, 3, 6);
    cmd = mi2010soc_var_read8(1, 3);            // SEQ_CMD
    while(cmd!=0) {
        cmd = mi2010soc_var_read8(1, 3);        // SEQ_CMD
    }
    sensorShouldRefreshMode = 0;
}

int mi2010soc_viewfinder_on()
{
    // check refresh mode command
    if(sensorShouldRefreshMode)
    {
        mi2010soc_cmd_refresh_mode();
    }
    // check refresh command
    if(sensorShouldRefresh)
    {
        mi2010soc_cmd_refresh();
    }

    if(sensorContext)
    {   // change to preview mode (context 0) if context is not preview
        mi2010soc_set_mode(0);
    }
    else
    {   // ensure the sensor mode is preview
        mi2010soc_wait_state(3);
    }
    sensorRunning = 1;

    return MICRON_ERR_NONE;
}

int mi2010soc_viewfinder_off()
{
    ddbg_print("mi2010soc_viewfinder_off");

    return MICRON_ERR_NONE;
}

int mi2010soc_snapshot_trigger()
{
    ddbg_print("mi2010soc_snapshot_trigger");

    mi2010soc_set_mode(1);

    return MICRON_ERR_NONE;
}

int mi2010soc_snapshot_complete()
{
    ddbg_print("mi2010soc_snapshot_complete");
    return MICRON_ERR_NONE;
}

int mi2010soc_sensor_size(micron_window_size * window)
{
    sensorAWidth = window->width;
    sensorAHeight = window->height;
    ddbg_print("sensor A: %dx%d", sensorAWidth, sensorAHeight);

    // VAR  = 7, 39, 0x0000        //Crop_X0 (A) = 0
    // VAR  = 7, 41, 0x0320        //Crop_X1 (A) = 800
    // VAR  = 7, 43, 0x0000        //Crop_Y0 (A) = 0
    // VAR  = 7, 45, 0x0258        //Crop_Y1 (A) = 600
    mi2010soc_var_write16(7, 39, (MAX_SENSOR_A_WIDTH - sensorAWidth)/2 );
    mi2010soc_var_write16(7, 41, (MAX_SENSOR_A_WIDTH + sensorAWidth)/2 );
    mi2010soc_var_write16(7, 43, (MAX_SENSOR_A_HEIGHT - sensorAHeight)/2 );
    mi2010soc_var_write16(7, 45, (MAX_SENSOR_A_HEIGHT + sensorAHeight)/2 );

    sensorShouldRefresh = 1;
    return MICRON_ERR_NONE;
}

int mi2010soc_capture_sensor_size(micron_window_size * window)
{
    sensorBWidth = window->width;
    sensorBHeight = window->height;
    ddbg_print("sensor B: %dx%d", sensorBWidth, sensorBHeight);

    // VAR  = 7, 53, 0x0000        //Crop_X0 (B) = 0
    // VAR  = 7, 55, 0x0640        //Crop_X1 (B) = 1600
    // VAR  = 7, 57, 0x0000        //Crop_Y0 (B) = 0
    // VAR  = 7, 59, 0x04B0        //Crop_Y1 (B) = 1200
    mi2010soc_var_write16(7, 53, (MAX_SENSOR_B_WIDTH - sensorBWidth)/2 );
    mi2010soc_var_write16(7, 55, (MAX_SENSOR_B_WIDTH + sensorBWidth)/2 );
    mi2010soc_var_write16(7, 57, (MAX_SENSOR_B_HEIGHT - sensorBHeight)/2 );
    mi2010soc_var_write16(7, 59, (MAX_SENSOR_B_HEIGHT + sensorBHeight)/2 );

    sensorShouldRefresh = 1;
    return MICRON_ERR_NONE;
}

int mi2010soc_output_size(micron_window_size * window)
{
    outWidth = window->width;
    outHeight = window->height;
    ddbg_print("preview: %dx%d", outWidth, outHeight);

    //context A
    mi2010soc_var_write16(7, 3, outWidth);
    mi2010soc_var_write16(7, 5, outHeight);

    sensorShouldRefresh = 1;
    return MICRON_ERR_NONE;
}

int mi2010soc_capture_size(micron_window_size * window)
{
    captureWidth = window->width;
    captureHeight = window->height;
    ddbg_print("capture: %dx%d", captureWidth, captureHeight);

    //context B
    mi2010soc_var_write16(7, 7, captureWidth);
    mi2010soc_var_write16(7, 9, captureHeight);

    sensorShouldRefresh = 1;
    return MICRON_ERR_NONE;
}

int mi2010soc_set_spoof_size( micron_window_size * window)
{
    spoofWidth = window->width;
    spoofHeight = window->height;
    ddbg_print("spoof: %dx%d", spoofWidth, spoofHeight);

    // JPEG spoof width, height
    mi2010soc_var_write16(7, 121, spoofWidth);
    mi2010soc_var_write16(7, 123, spoofHeight);

    sensorShouldRefresh = 1;
    return MICRON_ERR_NONE;
}

int mi2010soc_get_spoof_size( micron_window_size * window)
{
    window->width = spoofWidth;
    window->height = spoofHeight;

    return MICRON_ERR_NONE;
}

int mi2010soc_set_exposure_mode(int mode, u32 maxexpotime)
{
    int maxzone, th23, time_1zone;
    if(sensor_flicker_freq == 60)
    {
        time_1zone = 8333;      // 8.333ms for 60hz flicker
        th23 = 8;               // 8x8.333ms = 66.66ms, 15fps
    }
    else
    {
        time_1zone = 9966;      // 9.966ms for 50hz flicker
        th23 = 6;               // 6x9.966ms = 59.8ms, 16.7fps
    }
    maxzone = maxexpotime / time_1zone;

    // zone range: 1-24
    if(maxzone>24)
        maxzone = 24;
    if(maxzone<=0)
        maxzone = 1;

    // th23 should be less than zone
    if(th23>maxzone)
        th23 = maxzone;

    ddbg_print("max time:%dus, max zone:%d, th23 zone:%d", maxexpotime, maxzone, th23);
    // VAR  = 2, 14, 0x008        //AE_MAX_INDEX = 8
    mi2010soc_var_write8(2, 14, maxzone);
    // VAR  = 2, 23, 0x008        //IndexTH23 = 8
    mi2010soc_var_write8(2, 23, th23);
    // write current zone index to max zone to take AE effect 
    // when change max zone to higher value
    // VAR  = 2, 30, 0x008        //AE_INDEX = 8
    mi2010soc_var_write8(2, 30, maxzone);

    if(mode==V4l_NM_NIGHT)
    {   // night mode
        // VAR8=2, 0x10, 0x0080    // AE_MAX_VIRTGAIN
        mi2010soc_var_write8(2, 0x10, 0x0080);
        // VAR8=2, 0x18, 0x0078    // AE_MAXGAIN23
        mi2010soc_var_write8(2, 0x18, 0x0078);
        // VAR=2, 0x14, 0x0060     // AE_MAX_DGAIN_AE1
        mi2010soc_var_write16(2, 0x14, 0x0080);
        // VAR8=2, 0x16, 0x0018    // AE_MAX_DGAIN_AE2
        mi2010soc_var_write8(2, 0x16, 0x0020);
        // VAR8=1, 0x18, 0x0040    // SEQ_LLSAT1
        mi2010soc_var_write8(1, 0x18, 0x0040);
    }
    else
    {   // normal mode
        // VAR8=2, 0x10, 0x0060    // AE_MAX_VIRTGAIN
        mi2010soc_var_write8(2, 0x10, 0x0080);
        // VAR8=2, 0x18, 0x005A    // AE_MAXGAIN23
        mi2010soc_var_write8(2, 0x18, 0x0040);
        // VAR=2, 0x14, 0x0060     // AE_MAX_DGAIN_AE1
        mi2010soc_var_write16(2, 0x14, 0x0080);
        // VAR8=2, 0x16, 0x0018    // AE_MAX_DGAIN_AE2
        mi2010soc_var_write8(2, 0x16, 0x0020);
        // VAR8=1, 0x18, 0x0080    // SEQ_LLSAT1
        mi2010soc_var_write8(1, 0x18, 0x0080);
    }

    if(sensorRunning)
        mi2010soc_cmd_refresh();

    return MICRON_ERR_NONE;
}

int mi2010soc_output_format(u16 format)
{
    // default output YCbCr format on preview/video mode 
    return MICRON_ERR_NONE;
}

int mi2010soc_capture_format(u16 format)
{
    switch(format)
    {
    case O_FORMAT_422_YCbYCr:
        mi2010soc_var_write16(7, 11, 0x0030);   // Context B, Bypass JPEG
        mi2010soc_var_write16(7, 114, 0x0027);
        mi2010soc_var_write16(7, 116, 0x0001);
        mi2010soc_var_write8(7, 118, 0x21);
        break;
    case O_FORMAT_JPEG:
        mi2010soc_var_write16(7, 11, 0x0010);   // Context B, enable JPEG
        mi2010soc_var_write16(7, 114, 0x0067);
        mi2010soc_var_write16(7, 116, 0x0406);
        mi2010soc_var_write8(7, 118, 0x02);
        break;
    default:
        err_print("unsupported format %d", format);
        break;
    }
    return MICRON_ERR_NONE;
}

struct _mi2010soc_var_monitor {
    u16     vmt;    // void *, 16bit pointer
    char    cmd;
    u16     arg1;
    u16     arg2;
    char    msgCount;
    long    msg;
    u8      var;
};

struct _mi2010soc_var_sequencer {
    u16     vmt;    // void *, 16bit pointer
    u8      mode;
    u8      cmd;
    u8      state;
    u8      stepMode;
    struct {
        u8  flashType;
        u8  aeContBuff;
        u8  aeContStep;
        u8  aeFastbuff;
        // ...
    } sharedParams;
};

void mi2010soc_dump_register(u16 startRegAddr, u16 endRegAddr, u16* buffer)
{
    u16 addr,value;
	
    for(addr = startRegAddr; addr <= endRegAddr; addr++)
    {
        value = mi2010soc_reg_read(addr);

        dbg_print("mi2010soc register: 0x%04x = %04x",addr,value);
	   
        if(buffer)
        {
            *buffer = value;
            buffer++;
        }
    }
}

/*
    enforce fps by setting the vertical blanking

    The table below assumes:
        MCLK = 52 MHz
        target row time = 81.6923 us
        VBLANK in low power = ???
        VBLANK in full power = ???

    If the input clock changes, these settings also need to change.

    The basic idea is that a flicker-detection friendly framerate close
    to the requested frame rate is chosen.
    
    For the flicker avoidance driver to operate correctly, the sensor frame rate
    may not be exactly 15 fps or 30 fps.
*/

static const u16 vblanksA[] = 
{
/* Values from 5 fps to 20 fps */
/* vBlanking      fps  actual */
    1788,	/* 5 : 5.11   */ 
    1290, 	/* 6 : 6.45   */
    1064, 	/* 7 : 7.32   */ 
    841, 	/* 8 : 8.45   */
    681, 	/* 9 : 9.5    */
    563, 	/* 10: 10.45  */
    452, 	/* 11: 11.55  */
    391, 	/* 12: 12.25  */
    340, 	/* 13: 13.17  */
    278, 	/* 14: 14.21  */
    202, 	/* 15: 15.11  */
    140, 	/* 16: 16.37  */
    103, 	/* 17: 17.22  */
    67, 	/* 18: 18.13  */
    36,   	/* 19: 19.00 */
    11   	/* 20: 19.776 */
};

int mi2010soc_set_fps(u16 fps)
{
    u16 vblankA;
    ddbg_print("new FPS=%d, previous FPS=%d", fps, sensor_fps);

    //
    // if the FPS has changed, update the vblank
    //

    if (fps != sensor_fps)
    {

        if(fps>20)
            fps = 20;
        else if(fps<5)
            fps = 5;
        //
        // use the vblanking from table to achieve fps
        //
        vblankA = vblanksA[fps-5];

        // REG = 0, 0x08, 0x00D0        //VBLANK (A) = 208
        mi2010soc_reg_write(0x008, vblankA);

        sensor_fps = fps;
    }

    return 0;
}

int mi2010soc_set_flicker(int flicker)
{
    u8 fd_mode;
    ddbg_print(" %d", flicker);
    sensor_flicker_freq = flicker;

    fd_mode = mi2010soc_var_read8(4, 4);    // FD_MODE
    fd_mode |= 0x80;                        // Manual mode 
    if(flicker==50)
    {
        mi2010soc_var_write8(4, 4, fd_mode | 0x40);     // set bit 6
    }
    else if(flicker==60)
    {
        mi2010soc_var_write8(4, 4, fd_mode & (~0x40));  // clear bit 6
    }
    else
        return -EINVAL;

    sensorShouldRefresh = 1;
    return 0;
}

int mi2010soc_set_contrast(int contrast)
{
    ddbg_print("mi2010soc_set_contrast: %d (skipped)", contrast);
    return MICRON_ERR_NONE;

}

/*set picture style(normal/black white/sepia/solarize/neg.art)*/
int mi2010soc_set_style(V4l_PIC_STYLE style)
{
    u16 effects_value;
    ddbg_print("mi2010soc_set_style: %d", style);

    switch(style)
    {
    case V4l_STYLE_BLACK_WHITE:
        effects_value = 0x6441;
        break;
    case V4l_STYLE_SEPIA:
        effects_value = 0x6442;
        break;
    case V4l_STYLE_SOLARIZE:
        effects_value = 0x3844;
        break;
    case V4l_STYLE_NEG_ART:
        effects_value = 0x6443;
        break;
    default:
        effects_value = 0x6440;
        break;
    }

    mi2010soc_var_write16(7, 127, effects_value);      // MODE_SPEC_EFFECTS_A
    mi2010soc_var_write16(7, 129, effects_value);      // MODE_SPEC_EFFECTS_B

    if(sensorRunning)
        mi2010soc_cmd_refresh();

    return MICRON_ERR_NONE;
}

        
/*set picture light(auto/direct sun/incandescent/fluorescent)*/     
int mi2010soc_set_light(V4l_PIC_WB light)
{ 
    u8 awb_mode;

    awb_mode = mi2010soc_var_read8(3, 83);      // AWB Mode
    switch(light)
    {
        case V4l_WB_DIRECT_SUN:
            ddbg_print(" %d (direct sun)", light);
            mi2010soc_var_write8(3, 83, awb_mode | 0x20);
            mi2010soc_var_write8(3, 81, 127);   // AWB CCM position
            break;       

        case V4l_WB_INCANDESCENT:
            ddbg_print(" %d (incandescent)", light);
            mi2010soc_var_write8(3, 83, awb_mode | 0x20);
            mi2010soc_var_write8(3, 81, 0);     // AWB CCM position
            break;       

        case V4l_WB_FLUORESCENT:
            ddbg_print(" %d (fluorescent)", light);
            mi2010soc_var_write8(3, 83, awb_mode | 0x20);
            mi2010soc_var_write8(3, 81, 64);    // AWB CCM position
            break;

        default:
            mi2010soc_var_write8(3, 83, awb_mode & (~0x20));
            ddbg_print(" %d (default)", light);
            break;
    }

    return MICRON_ERR_NONE;
}

    
/*set picture brightness*/
int mi2010soc_set_bright(int bright)
{
    ddbg_print("mi2010soc_set_bright %d", bright);

    static const u16 target[][2] = 
    {
        {18, 3},    // -1.7 EV     *2^(-1.7) -> *0.3078
        {24, 3},    // -1.3 EV     *2^(-1.3) -> *0.4061
        {30, 3},    // -1.0 EV     *2^(-1.0) -> *0.5
        {42, 5},    // -0.5 EV     *2^(-0.5) -> *0.7071
        {60, 8},    //  0.0 EV     *1.0
        {85, 10},   // +0.5 EV     *2^(+0.5) -> *1.4142
        {120,12},   // +1.0 EV     *2^(+1.0) -> *2.0
        {148,15},   // +1.3 EV     *2^(+1.3) -> *2.4623
        {195,20}    // +1.7 EV     *2^(+1.7) -> *3.2490
    };

    if(bright < -4 || bright > 4)
    {
        return MICRON_ERR_PARAMETER;
    }

    /*set luma value of AE target*/
    mi2010soc_var_write8(2, 6, target[bright+4][0]);
    /* AE gate */
    mi2010soc_var_write8(2, 7, target[bright+4][1]);

    return MICRON_ERR_NONE;
}

/*set jpeg scaling factor */
int mi2010soc_set_jpeg_scale(int scale)
{
    int qscale1, qscale2, qscale3;
    ddbg_print("%d", scale);

    if(scale<=0 || scale>127)
        return -EINVAL;

    /* scaling factor for first set of Q-tables */
    qscale1 = scale;
    if(qscale1>127)
        qscale1 = 127;

    qscale2 = qscale1 ;
    qscale3 = qscale1 ;
#if 0
    /* scaling factor for second set of Q-tables */
    qscale2 = (int)qscale1 * 3/2;   // 1.5 x qscale1
    if(qscale2>127)
        qscale2 = 127;

    /* scaling factor for third set of Q-tables */
    qscale3 = qscale1 * 2;          // 2 x qscale1
    if(qscale3>127)
        qscale3 = 127;
#endif

    mi2010soc_var_write8(9, 10, qscale1 | 0x80);
    mi2010soc_var_write8(9, 11, qscale2 | 0x80);
    mi2010soc_var_write8(9, 12, qscale3 | 0x80);

    return 0;
}

/*set picture mirroring*/
int mi2010soc_set_mirror(int rows, int columns)
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

    reg = mi2010soc_reg_read(0x020);
    mi2010soc_reg_write_020(reg);

    return 0;
}

static void mi2010soc_lens_shading_settings(void)
{
/*
Lens correction settings from Altus, 12/06/2005

[Lens Correction 95%]
REG=2, 0x80, 0x01A0 //LENS_CORRECTION_CONTROL
REG=2, 0x81, 0x6432 //ZONE_BOUNDS_X1_X2
REG=2, 0x82, 0x3296 //ZONE_BOUNDS_X0_X3
REG=2, 0x83, 0x9664 //ZONE_BOUNDS_X4_X5
REG=2, 0x84, 0x5028 //ZONE_BOUNDS_Y1_Y2
REG=2, 0x85, 0x2878 //ZONE_BOUNDS_Y0_Y3
REG=2, 0x86, 0x7850 //ZONE_BOUNDS_Y4_Y5
REG=2, 0x87, 0x0000 //CENTER_OFFSET
REG=2, 0x88, 0x013C //FX_RED
REG=2, 0x8B, 0x00BA //FY_RED
REG=2, 0x8E, 0x0CA8 //DF_DX_RED
REG=2, 0x91, 0x0E71 //DF_DY_RED
REG=2, 0x94, 0xBE2F //SECOND_DERIV_ZONE_0_RED
REG=2, 0x97, 0x4B17 //SECOND_DERIV_ZONE_1_RED
REG=2, 0x9A, 0x1B21 //SECOND_DERIV_ZONE_2_RED
REG=2, 0x9D, 0x2626 //SECOND_DERIV_ZONE_3_RED
REG=2, 0xA0, 0x2423 //SECOND_DERIV_ZONE_4_RED
REG=2, 0xA3, 0x2928 //SECOND_DERIV_ZONE_5_RED
REG=2, 0xA6, 0xF706 //SECOND_DERIV_ZONE_6_RED
REG=2, 0xA9, 0x7242 //SECOND_DERIV_ZONE_7_RED
REG=2, 0x89, 0x00DB //FX_GREEN
REG=2, 0x8C, 0x0083 //FY_GREEN
REG=2, 0x8F, 0x0E0B //DF_DX_GREEN
REG=2, 0x92, 0x0F3B //DF_DY_GREEN
REG=2, 0x95, 0xD713 //SECOND_DERIV_ZONE_0_GREEN
REG=2, 0x98, 0x180A //SECOND_DERIV_ZONE_1_GREEN
REG=2, 0x9B, 0x1B17 //SECOND_DERIV_ZONE_2_GREEN
REG=2, 0x9E, 0x1E21 //SECOND_DERIV_ZONE_3_GREEN
REG=2, 0xA1, 0x211D //SECOND_DERIV_ZONE_4_GREEN
REG=2, 0xA4, 0x1D1B //SECOND_DERIV_ZONE_5_GREEN
REG=2, 0xA7, 0x0405 //SECOND_DERIV_ZONE_6_GREEN
REG=2, 0xAA, 0x1413 //SECOND_DERIV_ZONE_7_GREEN
REG=2, 0x8A, 0x00B7 //FX_BLUE
REG=2, 0x8D, 0x0072 //FY_BLUE
REG=2, 0x90, 0x0E53 //DF_DX_BLUE
REG=2, 0x93, 0x0FD8 //DF_DY_BLUE
REG=2, 0x96, 0xCE11 //SECOND_DERIV_ZONE_0_BLUE
REG=2, 0x99, 0x1F09 //SECOND_DERIV_ZONE_1_BLUE
REG=2, 0x9C, 0x1614 //SECOND_DERIV_ZONE_2_BLUE
REG=2, 0x9F, 0x191C //SECOND_DERIV_ZONE_3_BLUE
REG=2, 0xA2, 0x1718 //SECOND_DERIV_ZONE_4_BLUE
REG=2, 0xA5, 0x1B18 //SECOND_DERIV_ZONE_5_BLUE
REG=2, 0xA8, 0x0304 //SECOND_DERIV_ZONE_6_BLUE
REG=2, 0xAB, 0x161A //SECOND_DERIV_ZONE_7_BLUE
REG=2, 0xAC, 0x8180 //X2_FACTORS
REG=2, 0xAD, 0x0000 //GLOBAL_OFFSET_FXY_FUNCTION
REG=2, 0xAE, 0x03FF //K_FACTOR_IN_K_FX_FY
STATE=Lens Correction Falloff, 95
STATE=Lens Correction Center X, 800
STATE=Lens Correction Center Y, 600
BITFIELD=1, 0x08, 0x0004, 1 //LENS_CORRECTION

*/

    u16 u0x108 ;

    mi2010soc_reg_write( 0x280 , 0x01A0 ) ;
    mi2010soc_reg_write( 0x281 , 0x6432 ) ;
    mi2010soc_reg_write( 0x282 , 0x3296 ) ;
    mi2010soc_reg_write( 0x283 , 0x9664 ) ;
    mi2010soc_reg_write( 0x284 , 0x5028 ) ;
    mi2010soc_reg_write( 0x285 , 0x2878 ) ;
    mi2010soc_reg_write( 0x286 , 0x7850 ) ;
    mi2010soc_reg_write( 0x287 , 0x0000 ) ;
    mi2010soc_reg_write( 0x288 , 0x013C ) ;
    mi2010soc_reg_write( 0x28B , 0x00BA ) ;
    mi2010soc_reg_write( 0x28E , 0x0CA8 ) ;
    mi2010soc_reg_write( 0x291 , 0x0E71 ) ;
    mi2010soc_reg_write( 0x294 , 0xBE2F ) ;
    mi2010soc_reg_write( 0x297 , 0x4B17 ) ;
    mi2010soc_reg_write( 0x29A , 0x1B21 ) ;
    mi2010soc_reg_write( 0x29D , 0x2626 ) ;
    mi2010soc_reg_write( 0x2A0 , 0x2423 ) ;
    mi2010soc_reg_write( 0x2A3 , 0x2928 ) ;
    mi2010soc_reg_write( 0x2A6 , 0xF706 ) ;
    mi2010soc_reg_write( 0x2A9 , 0x7242 ) ;
    mi2010soc_reg_write( 0x289 , 0x00DB ) ;
    mi2010soc_reg_write( 0x28C , 0x0083 ) ;
    mi2010soc_reg_write( 0x28F , 0x0E0B ) ;
    mi2010soc_reg_write( 0x292 , 0x0F3B ) ;
    mi2010soc_reg_write( 0x295 , 0xD713 ) ;
    mi2010soc_reg_write( 0x298 , 0x180A ) ;
    mi2010soc_reg_write( 0x29B , 0x1B17 ) ;
    mi2010soc_reg_write( 0x29E , 0x1E21 ) ;
    mi2010soc_reg_write( 0x2A1 , 0x211D ) ;
    mi2010soc_reg_write( 0x2A4 , 0x1D1B ) ;
    mi2010soc_reg_write( 0x2A7 , 0x0405 ) ;
    mi2010soc_reg_write( 0x2AA , 0x1413 ) ;
    mi2010soc_reg_write( 0x28A , 0x00B7 ) ;
    mi2010soc_reg_write( 0x28D , 0x0072 ) ;
    mi2010soc_reg_write( 0x290 , 0x0E53 ) ;
    mi2010soc_reg_write( 0x293 , 0x0FD8 ) ;
    mi2010soc_reg_write( 0x296 , 0xCE11 ) ;
    mi2010soc_reg_write( 0x299 , 0x1F09 ) ;
    mi2010soc_reg_write( 0x29C , 0x1614 ) ;
    mi2010soc_reg_write( 0x29F , 0x191C ) ;
    mi2010soc_reg_write( 0x2A2 , 0x1718 ) ;
    mi2010soc_reg_write( 0x2A5 , 0x1B18 ) ;
    mi2010soc_reg_write( 0x2A8 , 0x0304 ) ;
    mi2010soc_reg_write( 0x2AB , 0x161A ) ;
    mi2010soc_reg_write( 0x2AC , 0x8180 ) ;
    mi2010soc_reg_write( 0x2AD , 0x0000 ) ;
    mi2010soc_reg_write( 0x2AE , 0x03FF ) ;

    u0x108 = mi2010soc_reg_read( 0x108 ) ;
    mi2010soc_reg_write( 0x108 , u0x108 | 0x0004 ) ;
}

static void mi2010soc_set_pll(void)
{
    // REG = 0, 0x66, 0x1001        //PLL Control 1 = 4097
    // REG = 0, 0x67, 0x0501        //PLL Control 2 = 1281
    // REG = 0, 0x65, 0xA000        //Clock CNTRL: PLL ON = 40960
    // REG = 0, 0x65, 0x2000        //Clock CNTRL: USE PLL = 8192
#if 0
    // Enable PLL 26 Mhz Ext/39 Mhz out
    // M = 18, N = 2, P = 1, PFD = 8.667, Fvco = 156
    mi2010soc_reg_write(0x066, 0x1202);
    mi2010soc_reg_write(0x067, 0x0501);
#endif

#if 1
    // Enable PLL 26 Mhz Ext/52 Mhz out
    // M = 16, N = 1, P = 1, PFD = 13, Fvco = 208
    mi2010soc_reg_write(0x066, 0x1001);
    mi2010soc_reg_write(0x067, 0x0501);
#endif

#if 0
    // Enable PLL 26 Mhz Ext/65 Mhz out
    // M = 20, N = 3, P = 0, PFD = 6.5, Fvco = 130
    mi2010soc_reg_write(0x066, 0x1403);
    mi2010soc_reg_write(0x067, 0x0500);
#endif

#if 0
    // Enable PLL 26 Mhz Ext/78 Mhz out
    // M = 18, N = 2, P = 0, PFD = 8.667, Fvco = 156
    mi2010soc_reg_write(0x066, 0x1202);
    mi2010soc_reg_write(0x067, 0x0500);
#endif

    mi2010soc_reg_write(0x065, 0xA000);
    udelay(150);
    mi2010soc_reg_write(0x065, 0x2000);

    sensor_pll = 1;
}

int mi2010soc_get_jpeg_status(int *plength, int *pstatus)
{
    u16 r202, r203, r204;
    u16 var9_16;
    u8  var9_15;
    var9_15 = mi2010soc_var_read8(9, 15);
    var9_16 = mi2010soc_var_read16(9, 16);

    r202 = mi2010soc_reg_read(0x202);
    r203 = mi2010soc_reg_read(0x203);
    r204 = mi2010soc_reg_read(0x204);
    if(plength)
        *plength = ((int)var9_15 << 8) + (int)var9_16;
    if(pstatus)
        *pstatus = r202 & 0x00ff;
    return 0;
}

int mi2010soc_standby_enter(void)
{
    int state;
    int count;
    u16 value;

    state = mi2010soc_var_read8(1, 4);          // SEQ_STATE
    if(state!=3)
        mi2010soc_set_mode(0);                  // enter prview mode

    // VAR  = 1, 3, 0x003        //Do Standby Sequencer = 3, enter standby
    mi2010soc_var_write8(1, 3, 3);

    // Waiting state to standby mode
    count = 500000;
    state = mi2010soc_var_read8(1, 4);          // SEQ_STATE
    while(state!=9 && count>0) {
        state = mi2010soc_var_read8(1, 4);      // SEQ_STATE
        count--;
    }

    if(sensor_pll)
    {
        // bypass PLL
        mi2010soc_reg_write(0x065, 0xE000);
        udelay(150);
    }

    // PAD_SLEW, control the bidirectional pads
    value = mi2010soc_reg_read(0x10a);
    mi2010soc_reg_write(0x10a, value | 0x0080);

    // Drive output pins
    value = mi2010soc_reg_read(0x00D);
    mi2010soc_reg_write(0x00D, value | 0x0040);

    // configure GPIO as outputs
    // follow TN-09-34: MT9D111 Standby Sequence Introduction, Page 2
    mi2010soc_reg_write(0x1c6, 0x9078);     // GPIO_DIR_H pads 11:8
    mi2010soc_reg_write(0x1c8, 0x0000);     // 11:8 as output
    mi2010soc_reg_write(0x1c6, 0x9079);     // GPIO_DIR_L pads 7:0
    mi2010soc_reg_write(0x1c8, 0x0000);     // 7:2 as output, 1:0 as input
    mi2010soc_reg_write(0x1c6, 0x9070);     // GPIO_DATA_H pads 11:8
    mi2010soc_reg_write(0x1c8, 0x0000);
    mi2010soc_reg_write(0x1c6, 0x9071);     // GPIO_DATA_L pads 7:0
    mi2010soc_reg_write(0x1c8, 0x0000);

    // soft standby
#if 0
    value = mi2010soc_reg_read(0x00D);
    mi2010soc_reg_write(0x00D, value | 0x0002);
    mdelay(2);
#endif

    return 0;
}

int mi2010soc_standby_exit(void)
{
    int state;
    int count;
    u16 value;

    // disable soft standby
#if 0
    value = mi2010soc_reg_read(0x00D);
    mi2010soc_reg_write(0x00D, value & (~0x0002));
    mdelay(2);
#endif

    // PAD_SLEW, control the bidirectional pads
    value = mi2010soc_reg_read(0x10a);
    mi2010soc_reg_write(0x10a, value & (~0x0080));

    // Drive output pins
    value = mi2010soc_reg_read(0x00D);
    mi2010soc_reg_write(0x00D, value & (~0x0040));

#if 0
    // is not necessary to configure all GPIO as inputs
    mi2010soc_reg_write(0x1c6, 0x9078);     // GPIO_DIR_H pads 11:8
    mi2010soc_reg_write(0x1c8, 0x000F);     // 11:8 as input
    mi2010soc_reg_write(0x1c6, 0x9079);     // GPIO_DIR_L pads 7:0
    mi2010soc_reg_write(0x1c8, 0x00FF);     // 7:2 as input, 1:0 as input
#endif

    if(sensor_pll)
    {
        // enalbe PLL
        mdelay(2); //required by sensor
        mi2010soc_reg_write(0x065, 0xA000);
        udelay(150);
        mi2010soc_reg_write(0x065, 0x2000);
    }

    state = mi2010soc_var_read8(1, 4);          // SEQ_STATE
    if(state!=3)
    {
        // VAR  = 1, 3, 0x001        //Do Preview Sequencer = 1, enter preview
        mi2010soc_var_write8(1, 3, 1);

        // Waiting state to standby mode
        count = 500000;
        while(state!=3 && count>0) {
            state = mi2010soc_var_read8(1, 4);      // SEQ_STATE
            count--;
        }
    }
    return 0;
}

int mi2010soc_colorcorrectionmatrix( void )
{
    // VAR=3, 0x06, 0x0257 //AWB_CCM_L_0
    mi2010soc_var_write16( 3 , 0x06 , 0x0257 ) ;
    // VAR=3, 0x08, 0xFFFFFFAB //AWB_CCM_L_1
    mi2010soc_var_write16( 3 , 0x08 , 0xFFAB ) ;
    // VAR=3, 0x0A, 0xFFFFFF29 //AWB_CCM_L_2
    mi2010soc_var_write16( 3 , 0x0A , 0xFF29 ) ;
    // VAR=3, 0x0C, 0xFFFFFF43 //AWB_CCM_L_3
    mi2010soc_var_write16( 3 , 0x0C , 0xFF43 ) ;
    // VAR=3, 0x0E, 0x0249 //AWB_CCM_L_4
    mi2010soc_var_write16( 3 , 0x0E , 0x0249 ) ;
    // VAR=3, 0x10, 0xFFFFFFA1 //AWB_CCM_L_5
    mi2010soc_var_write16( 3 , 0x10 , 0xFFA1 ) ;
    //VAR=3, 0x12, 0xFFFFFEF3 //AWB_CCM_L_6
    mi2010soc_var_write16( 3 , 0x12 , 0xFEF3 ) ;
    // VAR=3, 0x14, 0xFFFFFD4D //AWB_CCM_L_7
    mi2010soc_var_write16( 3 , 0x14 , 0xFD4D ) ;
    // VAR=3, 0x16, 0x0508 //AWB_CCM_L_8
    mi2010soc_var_write16( 3 , 0x16 , 0x0508 ) ;
    // VAR=3, 0x18, 0x0022 //AWB_CCM_L_9
    mi2010soc_var_write16( 3 , 0x18 , 0x0022 ) ;
    // VAR=3, 0x1A, 0x0035 //AWB_CCM_L_10
    mi2010soc_var_write16( 3 , 0x1A , 0x0035 ) ;
    // VAR=3, 0x1C, 0xFFFFFF61 //AWB_CCM_RL_0
    mi2010soc_var_write16( 3 , 0x1C , 0xFF61 ) ;
    // VAR=3, 0x1E, 0x001A //AWB_CCM_RL_1
    mi2010soc_var_write16( 3 , 0x1E , 0x1A ) ;
    // VAR=3, 0x20, 0x0085 //AWB_CCM_RL_2
    mi2010soc_var_write16( 3 , 0x20 , 0x0085 ) ;
    // VAR=3, 0x22, 0x0049 //AWB_CCM_RL_3
    mi2010soc_var_write16( 3 , 0x22 , 0x0049 ) ;
    // VAR=3, 0x24, 0xFFFFFFD4 //AWB_CCM_RL_4
    mi2010soc_var_write16( 3 , 0x24 , 0xFFD4 ) ;
    // VAR=3, 0x26, 0xFFFFFFE9 //AWB_CCM_RL_5
    mi2010soc_var_write16( 3 , 0x26 , 0xFFE9 ) ;
    // VAR=3, 0x28, 0x00DE //AWB_CCM_RL_6
    mi2010soc_var_write16( 3 , 0x28 , 0x00DE ) ;
    // VAR=3, 0x2A, 0x019B //AWB_CCM_RL_7
    mi2010soc_var_write16( 3 , 0x2A , 0x019B ) ;
    // VAR=3, 0x2C, 0xFFFFFD6F //AWB_CCM_RL_8
    mi2010soc_var_write16( 3 , 0x2C , 0xFD6F ) ;
    // VAR=3, 0x2E, 0x000A //AWB_CCM_RL_9
    mi2010soc_var_write16( 3 , 0x2E , 0x000A ) ;
    // VAR=3, 0x30, 0xFFFFFFF1 //AWB_CCM_RL_10
    mi2010soc_var_write16( 3 , 0x30 , 0xFFF1 ) ;

    // VAR8=3, 0x61, 0x00E2    // AWB_TG_MIN0
    mi2010soc_var_write8( 3 , 0x61 , 0x00E2 ) ;
    // VAR8=3, 0x62, 0x00F0    // AWB_TG_MAX0
    mi2010soc_var_write8( 3 , 0x62 , 0x00F0 ) ;

    return 0 ;
}

int mi2010soc_default_settings()
{
    u16 value;
    ddbg_print("mi2010soc_default_settings");

    // REG = 0, 0x05, 0x0204        //HBLANK (B) = 516
    mi2010soc_reg_write(0x005, 0x0204);
    // REG = 0, 0x06, 0x000B        //VBLANK (B) = 11
    mi2010soc_reg_write(0x006, 0x000B);
    // REG = 0, 0x07, 0x00FE        //HBLANK (A) = 254
    mi2010soc_reg_write(0x007, 0x00FE);
    // REG = 0, 0x08, 0x00D0        //VBLANK (A) = 202
    mi2010soc_reg_write(0x008, 202);// default to 15fps
    // REG = 0, 0x20, 0x0300        //Read Mode (B) = 768
    mi2010soc_reg_write_020(0x0300);//plus mirror 
    // REG = 0, 0x21, 0x8400        //Read Mode (A) = 33792
    mi2010soc_reg_write(0x021, 0x8400);

    // VAR  = 1, 34, 0x001        //Enter Preview: Auto Exposure = 1
    // VAR  = 1, 35, 0x000        //Enter Preview: Flicker Detection = 0
    mi2010soc_var_write8(1, 35, 0x000);
    // VAR  = 1, 36, 0x001        //Enter Preview: Auto White Balance = 1
    // VAR  = 1, 37, 0x000        //Enter Preview: Auto Focus = 0
    // VAR  = 1, 38, 0x001        //Enter Preview: Histogram = 1
    // VAR  = 1, 39, 0x000        //Enter Preview: Strobe Control = 0
    // VAR  = 1, 40, 0x000        //Enter Preview: Skip Control = 0
    // VAR  = 1, 41, 0x003        //In Preview: Auto Exposure = 3
    // VAR  = 1, 42, 0x002        //In Preview: Flicker Detection = 2
    mi2010soc_var_write8(1, 42, 0x002);
    // VAR  = 1, 43, 0x003        //In Preview: Auto White Balance = 3
    // VAR  = 1, 44, 0x000        //In Preview: Auto Focus = 0
    // VAR  = 1, 45, 0x003        //In Preview: Histogram = 3
    // VAR  = 1, 46, 0x000        //In Preview: Strobe Control = 0
    // VAR  = 1, 47, 0x000        //In Preview: Skip Control = 0
    // VAR  = 1, 48, 0x004        //Exit Preview: Auto Exposure = 4
    mi2010soc_var_write8(1, 48, 0x000);
    // VAR  = 1, 49, 0x000        //Exit Preview: Flicker Detection = 0
    // VAR  = 1, 50, 0x001        //Exit Preview: Auto White Balance = 1
    mi2010soc_var_write8(1, 50, 0x000);
    // VAR  = 1, 51, 0x000        //Exit Preview: Auto Focus = 0
    // VAR  = 1, 52, 0x001        //Exit Preview: Histogram = 1
    mi2010soc_var_write8(1, 52, 0x000);
    // VAR  = 1, 53, 0x000        //Exit Preview: Strobe Control = 0
    // VAR  = 1, 54, 0x000        //Exit Preview: Skip Control = 0
    mi2010soc_var_write8(1, 54, 0x040);
    // VAR  = 1, 55, 0x000        //Capture: Auto Exposure = 0
    // VAR  = 1, 56, 0x000        //Capture: Flicker Detection = 0
    // VAR  = 1, 57, 0x000        //Capture: Auto White Balance = 0
    // VAR  = 1, 58, 0x000        //Capture: Auto Focus = 0
    // VAR  = 1, 59, 0x000        //Capture: Histogram = 0
    // VAR  = 1, 60, 0x000        //Capture: Strobe Control = 0
    // VAR  = 1, 61, 0x000        //Capture: Skip Control = 0

    // VAR  = 7, 3, 0x0140        //Output Width (A) = 320
    mi2010soc_var_write16(7, 3, outWidth);
    // VAR  = 7, 5, 0x00F0        //Output Height (A) = 240
    mi2010soc_var_write16(7, 5, outHeight);
    // VAR  = 7, 7, 0x0640        //Output Width (B) = 1600
    mi2010soc_var_write16(7, 7, captureWidth);
    // VAR  = 7, 9, 0x04B0        //Output Height (B) = 1200
    mi2010soc_var_write16(7, 9, captureHeight);
    // VAR  = 7, 11, 0x0030        //mode_config = 48
    // VAR  = 7, 15, 0x01C        //Row Start (A) = 28
    // VAR  = 7, 17, 0x03C        //Column Start (A) = 60
    // VAR  = 7, 19, 0x4B0        //Row Height (A) = 1200
    // VAR  = 7, 21, 0x640        //Column Width (A) = 1600
    // VAR  = 7, 23, 0x000        //Extra Delay (A) = 0
    mi2010soc_var_write16(7, 23, 0);
    // VAR  = 7, 25, 0x0011        //Row Speed (A) = 17
    mi2010soc_var_write16(7, 25, 0x0011);
    // VAR  = 7, 27, 0x01C        //Row Start (B) = 28
    // VAR  = 7, 29, 0x03C        //Column Start (B) = 60
    // VAR  = 7, 31, 0x4B0        //Row Height (B) = 1200
    // VAR  = 7, 33, 0x640        //Column Width (B) = 1600
    // VAR  = 7, 35, 0x000        //Extra Delay (B) = 0
    mi2010soc_var_write16(7, 35, 0);
    // VAR  = 7, 37, 0x0011        //Row Speed (B) = 17
    mi2010soc_var_write16(7, 37, 0x0011);
    // VAR  = 7, 39, 0x0000        //Crop_X0 (A) = 0
    // VAR  = 7, 41, 0x0320        //Crop_X1 (A) = 800
    // VAR  = 7, 43, 0x0000        //Crop_Y0 (A) = 0
    // VAR  = 7, 45, 0x0258        //Crop_Y1 (A) = 600
    // VAR  = 7, 53, 0x0000        //Crop_X0 (B) = 0
    // VAR  = 7, 55, 0x0640        //Crop_X1 (B) = 1600
    // VAR  = 7, 57, 0x0000        //Crop_Y0 (B) = 0
    // VAR  = 7, 59, 0x04B0        //Crop_Y1 (B) = 1200

    // VAR8 = 7, 67, 0x42        //Gamma and Contrast Settings (A) = 66
    // VAR8 = 7, 68, 0x42        //Gamma and Contrast Settings (B) = 66


    // ;Custom gamma tables...

    // VAR  = 7, 69, 0x000        //Gamma Table 0 (A) = 0
    mi2010soc_var_write8(7,  69, 0x000);
    // VAR  = 7, 70, 0x014        //Gamma Table 1 (A) = 20
    mi2010soc_var_write8(7,  70, 0x014);
    // VAR  = 7, 71, 0x023        //Gamma Table 2 (A) = 35
    mi2010soc_var_write8(7,  71, 0x023);
    // VAR  = 7, 72, 0x03A        //Gamma Table 3 (A) = 58
    mi2010soc_var_write8(7,  72, 0x03A);
    // VAR  = 7, 73, 0x05E        //Gamma Table 4 (A) = 94
    mi2010soc_var_write8(7,  73, 0x05E);
    // VAR  = 7, 74, 0x076        //Gamma Table 5 (A) = 118
    mi2010soc_var_write8(7,  74, 0x076);
    // VAR  = 7, 75, 0x088        //Gamma Table 6 (A) = 136
    mi2010soc_var_write8(7,  75, 0x088);
    // VAR  = 7, 76, 0x096        //Gamma Table 7 (A) = 150
    mi2010soc_var_write8(7,  76, 0x096);
    // VAR  = 7, 77, 0x0A3        //Gamma Table 8 (A) = 163
    mi2010soc_var_write8(7,  77, 0x0A3);
    // VAR  = 7, 78, 0x0AF        //Gamma Table 9 (A) = 175
    mi2010soc_var_write8(7,  78, 0x0AF);
    // VAR  = 7, 79, 0x0BA        //Gamma Table 10 (A) = 186
    mi2010soc_var_write8(7,  79, 0x0BA);
    // VAR  = 7, 80, 0x0C4        //Gamma Table 11 (A) = 196
    mi2010soc_var_write8(7,  80, 0x0C4);
    // VAR  = 7, 81, 0x0CE        //Gamma Table 12 (A) = 206
    mi2010soc_var_write8(7,  81, 0x0CE);
    // VAR  = 7, 82, 0x0D7        //Gamma Table 13 (A) = 215
    mi2010soc_var_write8(7,  82, 0x0D7);
    // VAR  = 7, 83, 0x0E0        //Gamma Table 14 (A) = 224
    mi2010soc_var_write8(7,  83, 0x0E0);
    // VAR  = 7, 84, 0x0E8        //Gamma Table 15 (A) = 232
    mi2010soc_var_write8(7,  84, 0x0E8);
    // VAR  = 7, 85, 0x0F0        //Gamma Table 16 (A) = 240
    mi2010soc_var_write8(7,  85, 0x0F0);
    // VAR  = 7, 86, 0x0F8        //Gamma Table 17 (A) = 248
    mi2010soc_var_write8(7,  86, 0x0F8);
    // VAR  = 7, 87, 0x0FF        //Gamma Table 18 (A) = 255
    mi2010soc_var_write8(7,  87, 0x0FF);
    // VAR  = 7, 88, 0x000        //Gamma Table 0 (B) = 0
    mi2010soc_var_write8(7,  88, 0x000);
    // VAR  = 7, 89, 0x014        //Gamma Table 1 (B) = 20
    mi2010soc_var_write8(7,  89, 0x014);
    // VAR  = 7, 90, 0x023        //Gamma Table 2 (B) = 35
    mi2010soc_var_write8(7,  90, 0x023);
    // VAR  = 7, 91, 0x03A        //Gamma Table 3 (B) = 58
    mi2010soc_var_write8(7,  91, 0x03A);
    // VAR  = 7, 92, 0x05E        //Gamma Table 4 (B) = 94
    mi2010soc_var_write8(7,  92, 0x05E);
    // VAR  = 7, 93, 0x076        //Gamma Table 5 (B) = 118
    mi2010soc_var_write8(7,  93, 0x076);
    // VAR  = 7, 94, 0x088        //Gamma Table 6 (B) = 136
    mi2010soc_var_write8(7,  94, 0x088);
    // VAR  = 7, 95, 0x096        //Gamma Table 7 (B) = 150
    mi2010soc_var_write8(7,  95, 0x096);
    // VAR  = 7, 96, 0x0A3        //Gamma Table 8 (B) = 163
    mi2010soc_var_write8(7,  96, 0x0A3);
    // VAR  = 7, 97, 0x0AF        //Gamma Table 9 (B) = 175
    mi2010soc_var_write8(7,  97, 0x0AF);
    // VAR  = 7, 98, 0x0BA        //Gamma Table 10 (B) = 186
    mi2010soc_var_write8(7,  98, 0x0BA);
    // VAR  = 7, 99, 0x0C4        //Gamma Table 11 (B) = 196
    mi2010soc_var_write8(7,  99, 0x0C4);
    // VAR  = 7, 100, 0x0CE        //Gamma Table 12 (B) = 206
    mi2010soc_var_write8(7, 100, 0x0CE);
    // VAR  = 7, 101, 0x0D7        //Gamma Table 13 (B) = 215
    mi2010soc_var_write8(7, 101, 0x0D7);
    // VAR  = 7, 102, 0x0E0        //Gamma Table 14 (B) = 224
    mi2010soc_var_write8(7, 102, 0x0E0);
    // VAR  = 7, 103, 0x0E8        //Gamma Table 15 (B) = 232
    mi2010soc_var_write8(7, 103, 0x0E8);
    // VAR  = 7, 104, 0x0F0        //Gamma Table 16 (B) = 240
    mi2010soc_var_write8(7, 104, 0x0F0);
    // VAR  = 7, 105, 0x0F8        //Gamma Table 17 (B) = 248
    mi2010soc_var_write8(7, 105, 0x0F8);
    // VAR  = 7, 106, 0x0FF        //Gamma Table 18 (B) = 255
    mi2010soc_var_write8(7, 106, 0x0FF);


    // VAR  = 7, 109, 0xE0E2        //FIFO_Conf1 (A) = 57570
    // VAR  = 7, 111, 0x0E1        //FIFO_Conf2 (A) = 225
    // VAR  = 7, 114, 0x0067      //FIFO_Conf0 (B) = 0x0067
    //mi2010soc_var_write16(7, 114, 0x0067);
    // VAR  = 7, 116, 0xE0E1        //FIFO_Conf1 (B) = 57569
    //mi2010soc_var_write16(7, 116, 0xE0E1);
    // VAR  = 7, 118, 0x0E1        //FIFO_Conf2 (B) = 225
    //mi2010soc_var_write8(7, 118, 0x0E1);
    // VAR  = 7, 119, 0x0606        //MODE_FIFO_SPOOF_LEN_TIMING (B)
    //mi2010soc_var_write16(7, 119, 0x0606);

    // VAR  = 2, 11, 0x0192        //Max R12 (B)(Shutter Delay) = 402
    mi2010soc_var_write16(2, 11, 402);
    // VAR  = 2, 14, 0x008        //AE_MAX_INDEX = 8
    mi2010soc_var_write8(2, 14, 8);
    // VAR  = 2, 23, 0x008        //IndexTH23 = 8
    mi2010soc_var_write8(2, 23, 8);
    // VAR  = 2, 40, 0x020F        //RowTime (msclk per)/4 = 527
    mi2010soc_var_write16(2, 40, 527);
    // VAR  = 2, 47, 0x0066        //R9 Step = 102
    mi2010soc_var_write16(2, 47, 102);

    // VAR8=2, 0x10, 0x0060    // AE_MAX_VIRTGAIN
    mi2010soc_var_write8(2, 0x10, 0x0080);
    // VAR8=2, 0x18, 0x005A    // AE_MAXGAIN23
    mi2010soc_var_write8(2, 0x18, 0x0078);
    // VAR=2, 0x14, 0x0060     // AE_MAX_DGAIN_AE1
    mi2010soc_var_write16(2, 0x14, 0x0080);
    // VAR8=2, 0x16, 0x0018    // AE_MAX_DGAIN_AE2
    mi2010soc_var_write8(2, 0x16, 0x0020);

    // VAR  = 4, 8, 0x013        //search_f1_50 = 19
    mi2010soc_var_write8(4, 8, 19);
    // VAR  = 4, 9, 0x015        //search_f2_50 = 21
    mi2010soc_var_write8(4, 8, 21);
    // VAR  = 4, 10, 0x017        //search_f1_60 = 23
    mi2010soc_var_write8(4, 10, 23);
    // VAR  = 4, 11, 0x019        //search_f2_60 = 25
    mi2010soc_var_write8(4, 11, 25);
    // VAR  = 4, 17, 0x0066        //R9_Step_60 = 102
    mi2010soc_var_write16(4, 17, 102);
    // VAR  = 4, 19, 0x007A        //R9_Step_50 = 122
    mi2010soc_var_write16(4, 19, 122);

    // VAR  = 9, 8, 0x0000        //JPEG_RESTART_INT = 0 
    mi2010soc_var_write16(9, 8,  0 ) ;

    mi2010soc_lens_shading_settings() ;
    mi2010soc_colorcorrectionmatrix() ;

    // refresh mode after change mode parameters
    sensorShouldRefreshMode = 1;
    //mi2010soc_cmd_refresh_mode();

    // set output test pattern for test
    //mi2010soc_reg_write(0x148, 0x0003);

    /* Color Pipeline Control
       Do not invert the pixel clock */
    value = mi2010soc_reg_read(0x108);
    mi2010soc_reg_write(0x108, (value & 0xFFBF));

    sensorShouldRefresh = 1;

    return MICRON_ERR_NONE;
}

#if 0

; This file was generated by: SOC2010 Register Wizard ver 0.145
; 
; [MODE A PARAMETERS]
; 
; Max Frame Time: 33.3333 msec
; Max Frame Clocks: 866666.6 clocks (26 MHz)
; No. of ADCs: 1
; Skip Mode: 1x cols, 1x rows, Bin Mode: Yes
; Active Sensor Columns: 808 pixels / 1616 clocks
; Active Sensor Rows: 608 rows
; Horiz Blanking: 254 pixels / 508 clocks
; Vert Blanking: 11 rows
; Extra Delay: 0 clocks
; 
; Actual Frame Clocks: 1314756 clocks
; Row Time: 81.692 usec / 2124 clocks
; Frame time: 50.567538 msec
; Frames per Sec: 19.776 fps   (cannot match)
;     (PLL output minimum: 78.885 MHz )
; Max Shutter Delay: 402
; 50Hz Flicker Period: 122.41 lines
; 60Hz Flicker Period: 102.01 lines
; 
; [MODE B PARAMETERS]
; 
; Max Frame Time: 66.6667 msec
; Max Frame Clocks: 1733333.3 clocks (26 MHz)
; No. of ADCs: 2
; Skip Mode: 1x cols, 1x rows, Bin Mode: No
; Active Sensor Columns: 1608 pixels
; Active Sensor Rows: 1208 rows
; Horiz Blanking: 516 pixels
; Vert Blanking: 11 rows
; Extra Delay: 0 clocks
; 
; Actual Frame Clocks: 2589156 clocks
; Row Time: 81.692 usec / 2124 clocks
; Frame time: 99.582923 msec
; Frames per Sec: 10.042 fps   (cannot match)
;     (PLL output minimum: 77.675 MHz )
; Max Shutter Delay: 1663
; 50Hz Flicker Period: 122.41 lines
; 60Hz Flicker Period: 102.01 lines
; 
; 



[SOC2010 Register Wizard ver 0.145 Defaults]
REG = 0, 0x05, 0x0204        //HBLANK (B) = 516
REG = 0, 0x06, 0x000B        //VBLANK (B) = 11
REG = 0, 0x07, 0x00FE        //HBLANK (A) = 254
REG = 0, 0x08, 0x000B        //VBLANK (A) = 11
REG = 0, 0x20, 0x0300        //Read Mode (B) = 768
REG = 0, 0x21, 0x8400        //Read Mode (A) = 33792
REG = 0, 0x66, 0x1001        //PLL Control 1 = 4097
REG = 0, 0x67, 0x501        //PLL Control 2 = 1281
REG = 0, 0x65, 0xA000        //Clock CNTRL: PLL ON = 40960
REG = 0, 0x65, 0x2000        //Clock CNTRL: USE PLL = 8192


;Sequencer States...

VAR  = 1, 34, 0x001        //Enter Preview: Auto Exposure = 1
VAR  = 1, 35, 0x000        //Enter Preview: Flicker Detection = 0
VAR  = 1, 36, 0x001        //Enter Preview: Auto White Balance = 1
VAR  = 1, 37, 0x000        //Enter Preview: Auto Focus = 0
VAR  = 1, 38, 0x001        //Enter Preview: Histogram = 1
VAR  = 1, 39, 0x000        //Enter Preview: Strobe Control = 0
VAR  = 1, 40, 0x000        //Enter Preview: Skip Control = 0
VAR  = 1, 41, 0x003        //In Preview: Auto Exposure = 3
VAR  = 1, 42, 0x002        //In Preview: Flicker Detection = 2
VAR  = 1, 43, 0x003        //In Preview: Auto White Balance = 3
VAR  = 1, 44, 0x000        //In Preview: Auto Focus = 0
VAR  = 1, 45, 0x003        //In Preview: Histogram = 3
VAR  = 1, 46, 0x000        //In Preview: Strobe Control = 0
VAR  = 1, 47, 0x000        //In Preview: Skip Control = 0
VAR  = 1, 48, 0x004        //Exit Preview: Auto Exposure = 4
VAR  = 1, 49, 0x000        //Exit Preview: Flicker Detection = 0
VAR  = 1, 50, 0x001        //Exit Preview: Auto White Balance = 1
VAR  = 1, 51, 0x000        //Exit Preview: Auto Focus = 0
VAR  = 1, 52, 0x001        //Exit Preview: Histogram = 1
VAR  = 1, 53, 0x000        //Exit Preview: Strobe Control = 0
VAR  = 1, 54, 0x000        //Exit Preview: Skip Control = 0
VAR  = 1, 55, 0x000        //Capture: Auto Exposure = 0
VAR  = 1, 56, 0x000        //Capture: Flicker Detection = 0
VAR  = 1, 57, 0x000        //Capture: Auto White Balance = 0
VAR  = 1, 58, 0x000        //Capture: Auto Focus = 0
VAR  = 1, 59, 0x000        //Capture: Histogram = 0
VAR  = 1, 60, 0x000        //Capture: Strobe Control = 0
VAR  = 1, 61, 0x000        //Capture: Skip Control = 0


VAR  = 7, 3, 0x0320        //Output Width (A) = 800
VAR  = 7, 5, 0x0258        //Output Height (A) = 600
VAR  = 7, 7, 0x0640        //Output Width (B) = 1600
VAR  = 7, 9, 0x04B0        //Output Height (B) = 1200
VAR  = 7, 11, 0x0030        //mode_config = 48
VAR  = 7, 15, 0x01C        //Row Start (A) = 28
VAR  = 7, 17, 0x03C        //Column Start (A) = 60
VAR  = 7, 19, 0x4B0        //Row Height (A) = 1200
VAR  = 7, 21, 0x640        //Column Width (A) = 1600
VAR  = 7, 23, 0x000        //Extra Delay (A) = 0
VAR  = 7, 25, 0x0011        //Row Speed (A) = 17
VAR  = 7, 27, 0x01C        //Row Start (B) = 28
VAR  = 7, 29, 0x03C        //Column Start (B) = 60
VAR  = 7, 31, 0x4B0        //Row Height (B) = 1200
VAR  = 7, 33, 0x640        //Column Width (B) = 1600
VAR  = 7, 35, 0x000        //Extra Delay (B) = 0
VAR  = 7, 37, 0x0011        //Row Speed (B) = 17
VAR  = 7, 39, 0x0000        //Crop_X0 (A) = 0
VAR  = 7, 41, 0x0320        //Crop_X1 (A) = 800
VAR  = 7, 43, 0x0000        //Crop_Y0 (A) = 0
VAR  = 7, 45, 0x0258        //Crop_Y1 (A) = 600
VAR  = 7, 53, 0x0000        //Crop_X0 (B) = 0
VAR  = 7, 55, 0x0640        //Crop_X1 (B) = 1600
VAR  = 7, 57, 0x0000        //Crop_Y0 (B) = 0
VAR  = 7, 59, 0x04B0        //Crop_Y1 (B) = 1200
VAR8 = 7, 67, 0x42        //Gamma and Contrast Settings (A) = 66
VAR8 = 7, 68, 0x42        //Gamma and Contrast Settings (B) = 66


;Custom gamma tables...

VAR  = 7, 69, 0x000        //Gamma Table 0 (A) = 0
VAR  = 7, 70, 0x014        //Gamma Table 1 (A) = 20
VAR  = 7, 71, 0x023        //Gamma Table 2 (A) = 35
VAR  = 7, 72, 0x03A        //Gamma Table 3 (A) = 58
VAR  = 7, 73, 0x05E        //Gamma Table 4 (A) = 94
VAR  = 7, 74, 0x076        //Gamma Table 5 (A) = 118
VAR  = 7, 75, 0x088        //Gamma Table 6 (A) = 136
VAR  = 7, 76, 0x096        //Gamma Table 7 (A) = 150
VAR  = 7, 77, 0x0A3        //Gamma Table 8 (A) = 163
VAR  = 7, 78, 0x0AF        //Gamma Table 9 (A) = 175
VAR  = 7, 79, 0x0BA        //Gamma Table 10 (A) = 186
VAR  = 7, 80, 0x0C4        //Gamma Table 11 (A) = 196
VAR  = 7, 81, 0x0CE        //Gamma Table 12 (A) = 206
VAR  = 7, 82, 0x0D7        //Gamma Table 13 (A) = 215
VAR  = 7, 83, 0x0E0        //Gamma Table 14 (A) = 224
VAR  = 7, 84, 0x0E8        //Gamma Table 15 (A) = 232
VAR  = 7, 85, 0x0F0        //Gamma Table 16 (A) = 240
VAR  = 7, 86, 0x0F8        //Gamma Table 17 (A) = 248
VAR  = 7, 87, 0x0FF        //Gamma Table 18 (A) = 255
VAR  = 7, 88, 0x000        //Gamma Table 0 (B) = 0
VAR  = 7, 89, 0x014        //Gamma Table 1 (B) = 20
VAR  = 7, 90, 0x023        //Gamma Table 2 (B) = 35
VAR  = 7, 91, 0x03A        //Gamma Table 3 (B) = 58
VAR  = 7, 92, 0x05E        //Gamma Table 4 (B) = 94
VAR  = 7, 93, 0x076        //Gamma Table 5 (B) = 118
VAR  = 7, 94, 0x088        //Gamma Table 6 (B) = 136
VAR  = 7, 95, 0x096        //Gamma Table 7 (B) = 150
VAR  = 7, 96, 0x0A3        //Gamma Table 8 (B) = 163
VAR  = 7, 97, 0x0AF        //Gamma Table 9 (B) = 175
VAR  = 7, 98, 0x0BA        //Gamma Table 10 (B) = 186
VAR  = 7, 99, 0x0C4        //Gamma Table 11 (B) = 196
VAR  = 7, 100, 0x0CE        //Gamma Table 12 (B) = 206
VAR  = 7, 101, 0x0D7        //Gamma Table 13 (B) = 215
VAR  = 7, 102, 0x0E0        //Gamma Table 14 (B) = 224
VAR  = 7, 103, 0x0E8        //Gamma Table 15 (B) = 232
VAR  = 7, 104, 0x0F0        //Gamma Table 16 (B) = 240
VAR  = 7, 105, 0x0F8        //Gamma Table 17 (B) = 248
VAR  = 7, 106, 0x0FF        //Gamma Table 18 (B) = 255


VAR  = 7, 109, 0xE0E2        //FIFO_Conf1 (A) = 57570
VAR  = 7, 111, 0x0E1        //FIFO_Conf2 (A) = 225
VAR  = 7, 116, 0xE0E1        //FIFO_Conf1 (B) = 57569
VAR  = 7, 118, 0x0E1        //FIFO_Conf2 (B) = 225
VAR  = 2, 11, 0x0192        //Max R12 (B)(Shutter Delay) = 402
VAR  = 2, 23, 0x008        //IndexTH23 = 8
VAR  = 2, 40, 0x020F        //RowTime (msclk per)/4 = 527
VAR  = 2, 47, 0x0066        //R9 Step = 102
VAR  = 4, 8, 0x013        //search_f1_50 = 19
VAR  = 4, 9, 0x015        //search_f2_50 = 21
VAR  = 4, 10, 0x017        //search_f1_60 = 23
VAR  = 4, 11, 0x019        //search_f2_60 = 25
VAR  = 4, 17, 0x0066        //R9_Step_60 = 102
VAR  = 4, 19, 0x007A        //R9_Step_50 = 122
VAR  = 1, 3, 0x005        //Refresh Sequencer = 5
DELAY = 500
VAR  = 1, 3, 0x006        //Refresh Sequencer Mode = 6
#endif


// Software Patch for SOC2010 REV3 sensors only
//
// Please note: 
//
// Patch should be loaded after power-up while the PLL is off.
// Patch stays in memory in-and-out of standby; therefore, there is no need to reload the patch unless
// power is disconnected or a hard/soft reset is performed.
//
//
// Register Log created on Thursday, September 01, 2005 : 15:30:42
// [Register Log 09/01/05 15:30:42]

# if 0
REG=1, 0xC6, 0x104C 	// MCU_ADDRESS
REG=1, 0xC8, 0x0000 	// MCU_DATA_0
REG=1, 0xC6, 0x0310 	// MCU_ADDRESS
REG=1, 0xC8, 0x3C3C 	// MCU_DATA_0
REG=1, 0xC9, 0xCC01 	// MCU_DATA_1
REG=1, 0xCA, 0x33BD 	// MCU_DATA_2
REG=1, 0xCB, 0xD43F 	// MCU_DATA_3
REG=1, 0xCC, 0x30ED 	// MCU_DATA_4
REG=1, 0xCD, 0x02DC 	// MCU_DATA_5
REG=1, 0xCE, 0xB7A3 	// MCU_DATA_6
REG=1, 0xCF, 0x0223 	// MCU_DATA_7
REG=1, 0xC6, 0x0320 	// MCU_ADDRESS
REG=1, 0xC8, 0x0DF6 	// MCU_DATA_0
REG=1, 0xC9, 0x02BD 	// MCU_DATA_1
REG=1, 0xCA, 0xF102 	// MCU_DATA_2
REG=1, 0xCB, 0xBE23 	// MCU_DATA_3
REG=1, 0xCC, 0x107A 	// MCU_DATA_4
REG=1, 0xCD, 0x02BD 	// MCU_DATA_5
REG=1, 0xCE, 0x200B 	// MCU_DATA_6
REG=1, 0xCF, 0xF602 	// MCU_DATA_7
REG=1, 0xC6, 0x0330 	// MCU_ADDRESS
REG=1, 0xC8, 0xBDF1 	// MCU_DATA_0
REG=1, 0xC9, 0x02BF 	// MCU_DATA_1
REG=1, 0xCA, 0x2403 	// MCU_DATA_2
REG=1, 0xCB, 0x7C02 	// MCU_DATA_3
REG=1, 0xCC, 0xBDCC 	// MCU_DATA_4
REG=1, 0xCD, 0x011F 	// MCU_DATA_5
REG=1, 0xCE, 0xED00 	// MCU_DATA_6
REG=1, 0xCF, 0xF602 	// MCU_DATA_7
REG=1, 0xC6, 0x0340 	// MCU_ADDRESS
REG=1, 0xC8, 0xBD4F 	// MCU_DATA_0
REG=1, 0xC9, 0xBDD4 	// MCU_DATA_1
REG=1, 0xCA, 0x2BCC 	// MCU_DATA_2
REG=1, 0xCB, 0x0130 	// MCU_DATA_3
REG=1, 0xCC, 0xBDD4 	// MCU_DATA_4
REG=1, 0xCD, 0x3FD7 	// MCU_DATA_5
REG=1, 0xCE, 0xB026 	// MCU_DATA_6
REG=1, 0xCF, 0x04C6 	// MCU_DATA_7
REG=1, 0xC6, 0x0350 	// MCU_ADDRESS
REG=1, 0xC8, 0x01D7 	// MCU_DATA_0
REG=1, 0xC9, 0xB0CC 	// MCU_DATA_1
REG=1, 0xCA, 0x0131 	// MCU_DATA_2
REG=1, 0xCB, 0xBDD4 	// MCU_DATA_3
REG=1, 0xCC, 0x3FD7 	// MCU_DATA_4
REG=1, 0xCD, 0xB126 	// MCU_DATA_5
REG=1, 0xCE, 0x04C6 	// MCU_DATA_6
REG=1, 0xCF, 0x01D7 	// MCU_DATA_7
REG=1, 0xC6, 0x0360 	// MCU_ADDRESS
REG=1, 0xC8, 0xB1CC 	// MCU_DATA_0
REG=1, 0xC9, 0x0132 	// MCU_DATA_1
REG=1, 0xCA, 0xBDD4 	// MCU_DATA_2
REG=1, 0xCB, 0x3FD7 	// MCU_DATA_3
REG=1, 0xCC, 0xB25D 	// MCU_DATA_4
REG=1, 0xCD, 0x2604 	// MCU_DATA_5
REG=1, 0xCE, 0xC601 	// MCU_DATA_6
REG=1, 0xCF, 0xD7B2 	// MCU_DATA_7
REG=1, 0xC6, 0x0370 	// MCU_ADDRESS
REG=1, 0xC8, 0x5F38 	// MCU_DATA_0
REG=1, 0xC9, 0x3839 	// MCU_DATA_1
REG=1, 0xC6, 0x0400 	// MCU_ADDRESS
REG=1, 0xC8, 0x308F 	// MCU_DATA_0
REG=1, 0xC9, 0xC3FF 	// MCU_DATA_1
REG=1, 0xCA, 0xEF8F 	// MCU_DATA_2
REG=1, 0xCB, 0x35F6 	// MCU_DATA_3
REG=1, 0xCC, 0x01D0 	// MCU_DATA_4
REG=1, 0xCD, 0x860C 	// MCU_DATA_5
REG=1, 0xCE, 0x3DC3 	// MCU_DATA_6
REG=1, 0xCF, 0x01DD 	// MCU_DATA_7
REG=1, 0xC6, 0x0410 	// MCU_ADDRESS
REG=1, 0xC8, 0x8FEC 	// MCU_DATA_0
REG=1, 0xC9, 0x0630 	// MCU_DATA_1
REG=1, 0xCA, 0xED07 	// MCU_DATA_2
REG=1, 0xCB, 0xF601 	// MCU_DATA_3
REG=1, 0xCC, 0xD086 	// MCU_DATA_4
REG=1, 0xCD, 0x0C3D 	// MCU_DATA_5
REG=1, 0xCE, 0xC301 	// MCU_DATA_6
REG=1, 0xCF, 0xDD8F 	// MCU_DATA_7
REG=1, 0xC6, 0x0420 	// MCU_ADDRESS
REG=1, 0xC8, 0xEC04 	// MCU_DATA_0
REG=1, 0xC9, 0x30ED 	// MCU_DATA_1
REG=1, 0xCA, 0x05F6 	// MCU_DATA_2
REG=1, 0xCB, 0x01D0 	// MCU_DATA_3
REG=1, 0xCC, 0x4FED 	// MCU_DATA_4
REG=1, 0xCD, 0x02CC 	// MCU_DATA_5
REG=1, 0xCE, 0x0021 	// MCU_DATA_6
REG=1, 0xCF, 0xA302 	// MCU_DATA_7
REG=1, 0xC6, 0x0430 	// MCU_ADDRESS
REG=1, 0xC8, 0xBDD4 	// MCU_DATA_0
REG=1, 0xC9, 0x3F30 	// MCU_DATA_1
REG=1, 0xCA, 0xED0F 	// MCU_DATA_2
REG=1, 0xCB, 0x1F0F 	// MCU_DATA_3
REG=1, 0xCC, 0x800A 	// MCU_DATA_4
REG=1, 0xCD, 0xEC07 	// MCU_DATA_5
REG=1, 0xCE, 0x04ED 	// MCU_DATA_6
REG=1, 0xCF, 0x07EC 	// MCU_DATA_7
REG=1, 0xC6, 0x0440 	// MCU_ADDRESS
REG=1, 0xC8, 0x0504 	// MCU_DATA_0
REG=1, 0xC9, 0xED05 	// MCU_DATA_1
REG=1, 0xCA, 0xD65A 	// MCU_DATA_2
REG=1, 0xCB, 0xC40F 	// MCU_DATA_3
REG=1, 0xCC, 0xE704 	// MCU_DATA_4
REG=1, 0xCD, 0xEC07 	// MCU_DATA_5
REG=1, 0xCE, 0xED02 	// MCU_DATA_6
REG=1, 0xCF, 0xE604 	// MCU_DATA_7
REG=1, 0xC6, 0x0450 	// MCU_ADDRESS
REG=1, 0xC8, 0x4FED 	// MCU_DATA_0
REG=1, 0xC9, 0x00CC 	// MCU_DATA_1
REG=1, 0xCA, 0x0010 	// MCU_DATA_2
REG=1, 0xCB, 0xBDD3 	// MCU_DATA_3
REG=1, 0xCC, 0x4330 	// MCU_DATA_4
REG=1, 0xCD, 0xEC02 	// MCU_DATA_5
REG=1, 0xCE, 0xED0D 	// MCU_DATA_6
REG=1, 0xCF, 0xD65A 	// MCU_DATA_7
REG=1, 0xC6, 0x0460 	// MCU_ADDRESS
REG=1, 0xC8, 0x5454 	// MCU_DATA_0
REG=1, 0xC9, 0x5454 	// MCU_DATA_1
REG=1, 0xCA, 0xE704 	// MCU_DATA_2
REG=1, 0xCB, 0xEC05 	// MCU_DATA_3
REG=1, 0xCC, 0xED02 	// MCU_DATA_4
REG=1, 0xCD, 0xE604 	// MCU_DATA_5
REG=1, 0xCE, 0x4FED 	// MCU_DATA_6
REG=1, 0xCF, 0x00CC 	// MCU_DATA_7
REG=1, 0xC6, 0x0470 	// MCU_ADDRESS
REG=1, 0xC8, 0x0010 	// MCU_DATA_0
REG=1, 0xC9, 0xBDD3 	// MCU_DATA_1
REG=1, 0xCA, 0x4330 	// MCU_DATA_2
REG=1, 0xCB, 0xEC02 	// MCU_DATA_3
REG=1, 0xCC, 0xED0B 	// MCU_DATA_4
REG=1, 0xCD, 0xD65B 	// MCU_DATA_5
REG=1, 0xCE, 0xC40F 	// MCU_DATA_6
REG=1, 0xCF, 0xCB01 	// MCU_DATA_7
REG=1, 0xC6, 0x0480 	// MCU_ADDRESS
REG=1, 0xC8, 0xE704 	// MCU_DATA_0
REG=1, 0xC9, 0xEC07 	// MCU_DATA_1
REG=1, 0xCA, 0xED02 	// MCU_DATA_2
REG=1, 0xCB, 0xE604 	// MCU_DATA_3
REG=1, 0xCC, 0x4FED 	// MCU_DATA_4
REG=1, 0xCD, 0x00CC 	// MCU_DATA_5
REG=1, 0xCE, 0x0010 	// MCU_DATA_6
REG=1, 0xCF, 0xBDD3 	// MCU_DATA_7
REG=1, 0xC6, 0x0490 	// MCU_ADDRESS
REG=1, 0xC8, 0x4330 	// MCU_DATA_0
REG=1, 0xC9, 0xEC02 	// MCU_DATA_1
REG=1, 0xCA, 0xED07 	// MCU_DATA_2
REG=1, 0xCB, 0xD65B 	// MCU_DATA_3
REG=1, 0xCC, 0x5454 	// MCU_DATA_4
REG=1, 0xCD, 0x5454 	// MCU_DATA_5
REG=1, 0xCE, 0xCB01 	// MCU_DATA_6
REG=1, 0xCF, 0xE704 	// MCU_DATA_7
REG=1, 0xC6, 0x04A0 	// MCU_ADDRESS
REG=1, 0xC8, 0xEC05 	// MCU_DATA_0
REG=1, 0xC9, 0xED02 	// MCU_DATA_1
REG=1, 0xCA, 0xE604 	// MCU_DATA_2
REG=1, 0xCB, 0x4FED 	// MCU_DATA_3
REG=1, 0xCC, 0x00CC 	// MCU_DATA_4
REG=1, 0xCD, 0x0010 	// MCU_DATA_5
REG=1, 0xCE, 0xBDD3 	// MCU_DATA_6
REG=1, 0xCF, 0x4330 	// MCU_DATA_7
REG=1, 0xC6, 0x04B0 	// MCU_ADDRESS
REG=1, 0xC8, 0xEC02 	// MCU_DATA_0
REG=1, 0xC9, 0xED05 	// MCU_DATA_1
REG=1, 0xCA, 0xE30B 	// MCU_DATA_2
REG=1, 0xCB, 0xED09 	// MCU_DATA_3
REG=1, 0xCC, 0xC300 	// MCU_DATA_4
REG=1, 0xCD, 0x0ADD 	// MCU_DATA_5
REG=1, 0xCE, 0x5CEC 	// MCU_DATA_6
REG=1, 0xCF, 0x0DE3 	// MCU_DATA_7
REG=1, 0xC6, 0x04C0 	// MCU_ADDRESS
REG=1, 0xC8, 0x07ED 	// MCU_DATA_0
REG=1, 0xC9, 0x02EC 	// MCU_DATA_1
REG=1, 0xCA, 0x0DED 	// MCU_DATA_2
REG=1, 0xCB, 0x00C6 	// MCU_DATA_3
REG=1, 0xCC, 0x08BD 	// MCU_DATA_4
REG=1, 0xCD, 0xD319 	// MCU_DATA_5
REG=1, 0xCE, 0x30ED 	// MCU_DATA_6
REG=1, 0xCF, 0x0FCC 	// MCU_DATA_7
REG=1, 0xC6, 0x04D0 	// MCU_ADDRESS
REG=1, 0xC8, 0x012D 	// MCU_DATA_0
REG=1, 0xC9, 0xED00 	// MCU_DATA_1
REG=1, 0xCA, 0xEC0F 	// MCU_DATA_2
REG=1, 0xCB, 0xBDD4 	// MCU_DATA_3
REG=1, 0xCC, 0x2B30 	// MCU_DATA_4
REG=1, 0xCD, 0xEC09 	// MCU_DATA_5
REG=1, 0xCE, 0xED02 	// MCU_DATA_6
REG=1, 0xCF, 0xEC0B 	// MCU_DATA_7
REG=1, 0xC6, 0x04E0 	// MCU_ADDRESS
REG=1, 0xC8, 0xED00 	// MCU_DATA_0
REG=1, 0xC9, 0xC608 	// MCU_DATA_1
REG=1, 0xCA, 0xBDD3 	// MCU_DATA_2
REG=1, 0xCB, 0x1930 	// MCU_DATA_3
REG=1, 0xCC, 0xED0F 	// MCU_DATA_4
REG=1, 0xCD, 0xCC01 	// MCU_DATA_5
REG=1, 0xCE, 0x2EED 	// MCU_DATA_6
REG=1, 0xCF, 0x00EC 	// MCU_DATA_7
REG=1, 0xC6, 0x04F0 	// MCU_ADDRESS
REG=1, 0xC8, 0x0FBD 	// MCU_DATA_0
REG=1, 0xC9, 0xD42B 	// MCU_DATA_1
REG=1, 0xCA, 0x30C6 	// MCU_DATA_2
REG=1, 0xCB, 0x113A 	// MCU_DATA_3
REG=1, 0xCC, 0x3539 	// MCU_DATA_4
REG=1, 0xCD, 0x308F 	// MCU_DATA_5
REG=1, 0xCE, 0xC3FF 	// MCU_DATA_6
REG=1, 0xCF, 0xED8F 	// MCU_DATA_7
REG=1, 0xC6, 0x0500 	// MCU_ADDRESS
REG=1, 0xC8, 0x35F6 	// MCU_DATA_0
REG=1, 0xC9, 0x01D0 	// MCU_DATA_1
REG=1, 0xCA, 0x860E 	// MCU_DATA_2
REG=1, 0xCB, 0x3DC3 	// MCU_DATA_3
REG=1, 0xCC, 0x01F5 	// MCU_DATA_4
REG=1, 0xCD, 0x8FEC 	// MCU_DATA_5
REG=1, 0xCE, 0x0030 	// MCU_DATA_6
REG=1, 0xCF, 0xED06 	// MCU_DATA_7
REG=1, 0xC6, 0x0510 	// MCU_ADDRESS
REG=1, 0xC8, 0xF601 	// MCU_DATA_0
REG=1, 0xC9, 0xD086 	// MCU_DATA_1
REG=1, 0xCA, 0x0E3D 	// MCU_DATA_2
REG=1, 0xCB, 0xC301 	// MCU_DATA_3
REG=1, 0xCC, 0xF58F 	// MCU_DATA_4
REG=1, 0xCD, 0xEC04 	// MCU_DATA_5
REG=1, 0xCE, 0x30ED 	// MCU_DATA_6
REG=1, 0xCF, 0x04EC 	// MCU_DATA_7
REG=1, 0xC6, 0x0520 	// MCU_ADDRESS
REG=1, 0xC8, 0x15ED 	// MCU_DATA_0
REG=1, 0xC9, 0x0BEC 	// MCU_DATA_1
REG=1, 0xCA, 0x17ED 	// MCU_DATA_2
REG=1, 0xCB, 0x09D6 	// MCU_DATA_3
REG=1, 0xCC, 0x02C4 	// MCU_DATA_4
REG=1, 0xCD, 0x0FE7 	// MCU_DATA_5
REG=1, 0xCE, 0x08EC 	// MCU_DATA_6
REG=1, 0xCF, 0x0BED 	// MCU_DATA_7
REG=1, 0xC6, 0x0530 	// MCU_ADDRESS
REG=1, 0xC8, 0x02E6 	// MCU_DATA_0
REG=1, 0xC9, 0x084F 	// MCU_DATA_1
REG=1, 0xCA, 0xED00 	// MCU_DATA_2
REG=1, 0xCB, 0xCC00 	// MCU_DATA_3
REG=1, 0xCC, 0x10BD 	// MCU_DATA_4
REG=1, 0xCD, 0xD343 	// MCU_DATA_5
REG=1, 0xCE, 0x30EC 	// MCU_DATA_6
REG=1, 0xCF, 0x02ED 	// MCU_DATA_7
REG=1, 0xC6, 0x0540 	// MCU_ADDRESS
REG=1, 0xC8, 0x0FE3 	// MCU_DATA_0
REG=1, 0xC9, 0x06ED 	// MCU_DATA_1
REG=1, 0xCA, 0x0FD6 	// MCU_DATA_2
REG=1, 0xCB, 0x0254 	// MCU_DATA_3
REG=1, 0xCC, 0x5454 	// MCU_DATA_4
REG=1, 0xCD, 0x54E7 	// MCU_DATA_5
REG=1, 0xCE, 0x08EC 	// MCU_DATA_6
REG=1, 0xCF, 0x09ED 	// MCU_DATA_7
REG=1, 0xC6, 0x0550 	// MCU_ADDRESS
REG=1, 0xC8, 0x02E6 	// MCU_DATA_0
REG=1, 0xC9, 0x084F 	// MCU_DATA_1
REG=1, 0xCA, 0xED00 	// MCU_DATA_2
REG=1, 0xCB, 0xCC00 	// MCU_DATA_3
REG=1, 0xCC, 0x10BD 	// MCU_DATA_4
REG=1, 0xCD, 0xD343 	// MCU_DATA_5
REG=1, 0xCE, 0x30EC 	// MCU_DATA_6
REG=1, 0xCF, 0x02ED 	// MCU_DATA_7
REG=1, 0xC6, 0x0560 	// MCU_ADDRESS
REG=1, 0xC8, 0x0DE3 	// MCU_DATA_0
REG=1, 0xC9, 0x04ED 	// MCU_DATA_1
REG=1, 0xCA, 0x0DD6 	// MCU_DATA_2
REG=1, 0xCB, 0x03C4 	// MCU_DATA_3
REG=1, 0xCC, 0x0FCB 	// MCU_DATA_4
REG=1, 0xCD, 0x01E7 	// MCU_DATA_5
REG=1, 0xCE, 0x08EC 	// MCU_DATA_6
REG=1, 0xCF, 0x0BED 	// MCU_DATA_7
REG=1, 0xC6, 0x0570 	// MCU_ADDRESS
REG=1, 0xC8, 0x02E6 	// MCU_DATA_0
REG=1, 0xC9, 0x084F 	// MCU_DATA_1
REG=1, 0xCA, 0xED00 	// MCU_DATA_2
REG=1, 0xCB, 0xCC00 	// MCU_DATA_3
REG=1, 0xCC, 0x40BD 	// MCU_DATA_4
REG=1, 0xCD, 0xD343 	// MCU_DATA_5
REG=1, 0xCE, 0x30EC 	// MCU_DATA_6
REG=1, 0xCF, 0x02ED 	// MCU_DATA_7
REG=1, 0xC6, 0x0580 	// MCU_ADDRESS
REG=1, 0xC8, 0x0BD6 	// MCU_DATA_0
REG=1, 0xC9, 0x0354 	// MCU_DATA_1
REG=1, 0xCA, 0x5454 	// MCU_DATA_2
REG=1, 0xCB, 0x54CB 	// MCU_DATA_3
REG=1, 0xCC, 0x01E7 	// MCU_DATA_4
REG=1, 0xCD, 0x08EC 	// MCU_DATA_5
REG=1, 0xCE, 0x09ED 	// MCU_DATA_6
REG=1, 0xCF, 0x02E6 	// MCU_DATA_7
REG=1, 0xC6, 0x0590 	// MCU_ADDRESS
REG=1, 0xC8, 0x084F 	// MCU_DATA_0
REG=1, 0xC9, 0xED00 	// MCU_DATA_1
REG=1, 0xCA, 0xCC00 	// MCU_DATA_2
REG=1, 0xCB, 0x40BD 	// MCU_DATA_3
REG=1, 0xCC, 0xD343 	// MCU_DATA_4
REG=1, 0xCD, 0x30EC 	// MCU_DATA_5
REG=1, 0xCE, 0x02ED 	// MCU_DATA_6
REG=1, 0xCF, 0x0905 	// MCU_DATA_7
REG=1, 0xC6, 0x05A0 	// MCU_ADDRESS
REG=1, 0xC8, 0x05E3 	// MCU_DATA_0
REG=1, 0xC9, 0x0DC3 	// MCU_DATA_1
REG=1, 0xCA, 0x000A 	// MCU_DATA_2
REG=1, 0xCB, 0xDD04 	// MCU_DATA_3
REG=1, 0xCC, 0xEC0D 	// MCU_DATA_4
REG=1, 0xCD, 0xED02 	// MCU_DATA_5
REG=1, 0xCE, 0xEC0F 	// MCU_DATA_6
REG=1, 0xCF, 0xED00 	// MCU_DATA_7
REG=1, 0xC6, 0x05B0 	// MCU_ADDRESS
REG=1, 0xC8, 0xC608 	// MCU_DATA_0
REG=1, 0xC9, 0xBDD3 	// MCU_DATA_1
REG=1, 0xCA, 0x1930 	// MCU_DATA_2
REG=1, 0xCB, 0xED11 	// MCU_DATA_3
REG=1, 0xCC, 0xCC02 	// MCU_DATA_4
REG=1, 0xCD, 0xC0ED 	// MCU_DATA_5
REG=1, 0xCE, 0x00EC 	// MCU_DATA_6
REG=1, 0xCF, 0x11BD 	// MCU_DATA_7
REG=1, 0xC6, 0x05C0 	// MCU_ADDRESS
REG=1, 0xC8, 0xD42B 	// MCU_DATA_0
REG=1, 0xC9, 0x30EC 	// MCU_DATA_1
REG=1, 0xCA, 0x09ED 	// MCU_DATA_2
REG=1, 0xCB, 0x02EC 	// MCU_DATA_3
REG=1, 0xCC, 0x0BED 	// MCU_DATA_4
REG=1, 0xCD, 0x00C6 	// MCU_DATA_5
REG=1, 0xCE, 0x02BD 	// MCU_DATA_6
REG=1, 0xCF, 0xD319 	// MCU_DATA_7
REG=1, 0xC6, 0x05D0 	// MCU_ADDRESS
REG=1, 0xC8, 0x30ED 	// MCU_DATA_0
REG=1, 0xC9, 0x11CC 	// MCU_DATA_1
REG=1, 0xCA, 0x02C1 	// MCU_DATA_2
REG=1, 0xCB, 0xED00 	// MCU_DATA_3
REG=1, 0xCC, 0xEC11 	// MCU_DATA_4
REG=1, 0xCD, 0xBDD4 	// MCU_DATA_5
REG=1, 0xCE, 0x2B7F 	// MCU_DATA_6
REG=1, 0xCF, 0x10C4 	// MCU_DATA_7
REG=1, 0xC6, 0x05E0 	// MCU_ADDRESS
REG=1, 0xC8, 0x30EC 	// MCU_DATA_0
REG=1, 0xC9, 0x09C4 	// MCU_DATA_1
REG=1, 0xCA, 0xFEFD 	// MCU_DATA_2
REG=1, 0xCB, 0x10C5 	// MCU_DATA_3
REG=1, 0xCC, 0xEC0B 	// MCU_DATA_4
REG=1, 0xCD, 0xC4FE 	// MCU_DATA_5
REG=1, 0xCE, 0xFD10 	// MCU_DATA_6
REG=1, 0xCF, 0xC701 	// MCU_DATA_7
REG=1, 0xC6, 0x05F0 	// MCU_ADDRESS
REG=1, 0xC8, 0x0101 	// MCU_DATA_0
REG=1, 0xC9, 0xCC02 	// MCU_DATA_1
REG=1, 0xCA, 0xC2ED 	// MCU_DATA_2
REG=1, 0xCB, 0x00FC 	// MCU_DATA_3
REG=1, 0xCC, 0x10C2 	// MCU_DATA_4
REG=1, 0xCD, 0xBDD4 	// MCU_DATA_5
REG=1, 0xCE, 0x2BFC 	// MCU_DATA_6
REG=1, 0xCF, 0x10C0 	// MCU_DATA_7
REG=1, 0xC6, 0x0600 	// MCU_ADDRESS
REG=1, 0xC8, 0xCA06 	// MCU_DATA_0
REG=1, 0xC9, 0x30ED 	// MCU_DATA_1
REG=1, 0xCA, 0x11CC 	// MCU_DATA_2
REG=1, 0xCB, 0x02C3 	// MCU_DATA_3
REG=1, 0xCC, 0xED00 	// MCU_DATA_4
REG=1, 0xCD, 0xEC11 	// MCU_DATA_5
REG=1, 0xCE, 0xBDD4 	// MCU_DATA_6
REG=1, 0xCF, 0x2B30 	// MCU_DATA_7
REG=1, 0xC6, 0x0610 	// MCU_ADDRESS
REG=1, 0xC8, 0xC613 	// MCU_DATA_0
REG=1, 0xC9, 0x3A35 	// MCU_DATA_1
REG=1, 0xCA, 0x393C 	// MCU_DATA_2
REG=1, 0xCB, 0xDC25 	// MCU_DATA_3
REG=1, 0xCC, 0x30ED 	// MCU_DATA_4
REG=1, 0xCD, 0x00BD 	// MCU_DATA_5
REG=1, 0xCE, 0x81BE 	// MCU_DATA_6
REG=1, 0xCF, 0x7D00 	// MCU_DATA_7
REG=1, 0xC6, 0x0620 	// MCU_ADDRESS
REG=1, 0xC8, 0x1E27 	// MCU_DATA_0
REG=1, 0xC9, 0x227F 	// MCU_DATA_1
REG=1, 0xCA, 0x10C4 	// MCU_DATA_2
REG=1, 0xCB, 0xD61E 	// MCU_DATA_3
REG=1, 0xCC, 0x4FFD 	// MCU_DATA_4
REG=1, 0xCD, 0x10C5 	// MCU_DATA_5
REG=1, 0xCE, 0xDC2F 	// MCU_DATA_6
REG=1, 0xCF, 0xFD10 	// MCU_DATA_7
REG=1, 0xC6, 0x0630 	// MCU_ADDRESS
REG=1, 0xC8, 0xC701 	// MCU_DATA_0
REG=1, 0xC9, 0x0101 	// MCU_DATA_1
REG=1, 0xCA, 0xFC10 	// MCU_DATA_2
REG=1, 0xCB, 0xC2DD 	// MCU_DATA_3
REG=1, 0xCC, 0x25D6 	// MCU_DATA_4
REG=1, 0xCD, 0x31C1 	// MCU_DATA_5
REG=1, 0xCE, 0x0224 	// MCU_DATA_6
REG=1, 0xCF, 0x0BC6 	// MCU_DATA_7
REG=1, 0xC6, 0x0640 	// MCU_ADDRESS
REG=1, 0xC8, 0x02D7 	// MCU_DATA_0
REG=1, 0xC9, 0x3120 	// MCU_DATA_1
REG=1, 0xCA, 0x0530 	// MCU_DATA_2
REG=1, 0xCB, 0xEC00 	// MCU_DATA_3
REG=1, 0xCC, 0xDD25 	// MCU_DATA_4
REG=1, 0xCD, 0x3839 	// MCU_DATA_5
REG=1, 0xCE, 0x373C 	// MCU_DATA_6
REG=1, 0xCF, 0x3CD6 	// MCU_DATA_7
REG=1, 0xC6, 0x0650 	// MCU_ADDRESS
REG=1, 0xC8, 0x1E30 	// MCU_DATA_0
REG=1, 0xC9, 0xE701 	// MCU_DATA_1
REG=1, 0xCA, 0xDC25 	// MCU_DATA_2
REG=1, 0xCB, 0xED02 	// MCU_DATA_3
REG=1, 0xCC, 0xE607 	// MCU_DATA_4
REG=1, 0xCD, 0xE700 	// MCU_DATA_5
REG=1, 0xCE, 0xE604 	// MCU_DATA_6
REG=1, 0xCF, 0xBD87 	// MCU_DATA_7
REG=1, 0xC6, 0x0660 	// MCU_ADDRESS
REG=1, 0xC8, 0x01D6 	// MCU_DATA_0
REG=1, 0xC9, 0x1FD1 	// MCU_DATA_1
REG=1, 0xCA, 0x1023 	// MCU_DATA_2
REG=1, 0xCB, 0x04D6 	// MCU_DATA_3
REG=1, 0xCC, 0x10D7 	// MCU_DATA_4
REG=1, 0xCD, 0x1F30 	// MCU_DATA_5
REG=1, 0xCE, 0xE607 	// MCU_DATA_6
REG=1, 0xCF, 0xC101 	// MCU_DATA_7
REG=1, 0xC6, 0x0670 	// MCU_ADDRESS
REG=1, 0xC8, 0x2612 	// MCU_DATA_0
REG=1, 0xC9, 0xD61E 	// MCU_DATA_1
REG=1, 0xCA, 0xE101 	// MCU_DATA_2
REG=1, 0xCB, 0x240C 	// MCU_DATA_3
REG=1, 0xCC, 0xD610 	// MCU_DATA_4
REG=1, 0xCD, 0xD71F 	// MCU_DATA_5
REG=1, 0xCE, 0xE601 	// MCU_DATA_6
REG=1, 0xCF, 0xD71E 	// MCU_DATA_7
REG=1, 0xC6, 0x0680 	// MCU_ADDRESS
REG=1, 0xC8, 0xEC02 	// MCU_DATA_0
REG=1, 0xC9, 0xDD25 	// MCU_DATA_1
REG=1, 0xCA, 0x3838 	// MCU_DATA_2
REG=1, 0xCB, 0x3139 	// MCU_DATA_3
REG=1, 0xCC, 0x3CDE 	// MCU_DATA_4
REG=1, 0xCD, 0x00EE 	// MCU_DATA_5
REG=1, 0xCE, 0x12AD 	// MCU_DATA_6
REG=1, 0xCF, 0x00D6 	// MCU_DATA_7
REG=1, 0xC6, 0x0690 	// MCU_ADDRESS
REG=1, 0xC8, 0x4630 	// MCU_DATA_0
REG=1, 0xC9, 0xE701 	// MCU_DATA_1
REG=1, 0xCA, 0xC601 	// MCU_DATA_2
REG=1, 0xCB, 0xE700 	// MCU_DATA_3
REG=1, 0xCC, 0xE600 	// MCU_DATA_4
REG=1, 0xCD, 0x4F8F 	// MCU_DATA_5
REG=1, 0xCE, 0xE646 	// MCU_DATA_6
REG=1, 0xCF, 0x30EB 	// MCU_DATA_7
REG=1, 0xC6, 0x06A0 	// MCU_ADDRESS
REG=1, 0xC8, 0x01E7 	// MCU_DATA_0
REG=1, 0xC9, 0x016C 	// MCU_DATA_1
REG=1, 0xCA, 0x00E6 	// MCU_DATA_2
REG=1, 0xCB, 0x00C1 	// MCU_DATA_3
REG=1, 0xCC, 0x0525 	// MCU_DATA_4
REG=1, 0xCD, 0xED4F 	// MCU_DATA_5
REG=1, 0xCE, 0xE601 	// MCU_DATA_6
REG=1, 0xCF, 0x2A01 	// MCU_DATA_7
REG=1, 0xC6, 0x06B0 	// MCU_ADDRESS
REG=1, 0xC8, 0x43CE 	// MCU_DATA_0
REG=1, 0xC9, 0x0005 	// MCU_DATA_1
REG=1, 0xCA, 0xBD07 	// MCU_DATA_2
REG=1, 0xCB, 0xD0D7 	// MCU_DATA_3
REG=1, 0xCC, 0x4E30 	// MCU_DATA_4
REG=1, 0xCD, 0x6F01 	// MCU_DATA_5
REG=1, 0xCE, 0x96D5 	// MCU_DATA_6
REG=1, 0xCF, 0x112F 	// MCU_DATA_7
REG=1, 0xC6, 0x06C0 	// MCU_ADDRESS
REG=1, 0xC8, 0x04C6 	// MCU_DATA_0
REG=1, 0xC9, 0x01E7 	// MCU_DATA_1
REG=1, 0xCA, 0x01E6 	// MCU_DATA_2
REG=1, 0xCB, 0x0138 	// MCU_DATA_3
REG=1, 0xCC, 0x393C 	// MCU_DATA_4
REG=1, 0xCD, 0x3C3C 	// MCU_DATA_5
REG=1, 0xCE, 0x3C34 	// MCU_DATA_6
REG=1, 0xCF, 0xC620 	// MCU_DATA_7
REG=1, 0xC6, 0x06D0 	// MCU_ADDRESS
REG=1, 0xC8, 0xF702 	// MCU_DATA_0
REG=1, 0xC9, 0xBD7F 	// MCU_DATA_1
REG=1, 0xCA, 0x02BE 	// MCU_DATA_2
REG=1, 0xCB, 0xF702 	// MCU_DATA_3
REG=1, 0xCC, 0xBFC6 	// MCU_DATA_4
REG=1, 0xCD, 0xF6D7 	// MCU_DATA_5
REG=1, 0xCE, 0xBACC 	// MCU_DATA_6
REG=1, 0xCF, 0x02AB 	// MCU_DATA_7
REG=1, 0xC6, 0x06E0 	// MCU_ADDRESS
REG=1, 0xC8, 0x30ED 	// MCU_DATA_0
REG=1, 0xC9, 0x06FE 	// MCU_DATA_1
REG=1, 0xCA, 0x1050 	// MCU_DATA_2
REG=1, 0xCB, 0xEC06 	// MCU_DATA_3
REG=1, 0xCC, 0xFD02 	// MCU_DATA_4
REG=1, 0xCD, 0xA7FE 	// MCU_DATA_5
REG=1, 0xCE, 0x02A7 	// MCU_DATA_6
REG=1, 0xCF, 0xEC00 	// MCU_DATA_7
REG=1, 0xC6, 0x06F0 	// MCU_ADDRESS
REG=1, 0xC8, 0xFD02 	// MCU_DATA_0
REG=1, 0xC9, 0xA930 	// MCU_DATA_1
REG=1, 0xCA, 0x6F08 	// MCU_DATA_2
REG=1, 0xCB, 0xE608 	// MCU_DATA_3
REG=1, 0xCC, 0x4F05 	// MCU_DATA_4
REG=1, 0xCD, 0xF302 	// MCU_DATA_5
REG=1, 0xCE, 0xA98F 	// MCU_DATA_6
REG=1, 0xCF, 0xEC00 	// MCU_DATA_7
REG=1, 0xC6, 0x0700 	// MCU_ADDRESS
REG=1, 0xC8, 0x30ED 	// MCU_DATA_0
REG=1, 0xC9, 0x00E6 	// MCU_DATA_1
REG=1, 0xCA, 0x084F 	// MCU_DATA_2
REG=1, 0xCB, 0x05E3 	// MCU_DATA_3
REG=1, 0xCC, 0x0618 	// MCU_DATA_4
REG=1, 0xCD, 0x8FEC 	// MCU_DATA_5
REG=1, 0xCE, 0x0018 	// MCU_DATA_6
REG=1, 0xCF, 0xED00 	// MCU_DATA_7
REG=1, 0xC6, 0x0710 	// MCU_ADDRESS
REG=1, 0xC8, 0x6C08 	// MCU_DATA_0
REG=1, 0xC9, 0xE608 	// MCU_DATA_1
REG=1, 0xCA, 0xC109 	// MCU_DATA_2
REG=1, 0xCB, 0x25DE 	// MCU_DATA_3
REG=1, 0xCC, 0xEE06 	// MCU_DATA_4
REG=1, 0xCD, 0xCC03 	// MCU_DATA_5
REG=1, 0xCE, 0x10ED 	// MCU_DATA_6
REG=1, 0xCF, 0x0230 	// MCU_DATA_7
REG=1, 0xC6, 0x0720 	// MCU_ADDRESS
REG=1, 0xC8, 0xEE06 	// MCU_DATA_0
REG=1, 0xC9, 0xCC04 	// MCU_DATA_1
REG=1, 0xCA, 0x00ED 	// MCU_DATA_2
REG=1, 0xCB, 0x04CC 	// MCU_DATA_3
REG=1, 0xCC, 0x02AB 	// MCU_DATA_4
REG=1, 0xCD, 0xDD58 	// MCU_DATA_5
REG=1, 0xCE, 0xCC02 	// MCU_DATA_6
REG=1, 0xCF, 0xC430 	// MCU_DATA_7
REG=1, 0xC6, 0x0730 	// MCU_ADDRESS
REG=1, 0xC8, 0xED04 	// MCU_DATA_0
REG=1, 0xC9, 0xFE10 	// MCU_DATA_1
REG=1, 0xCA, 0x50EC 	// MCU_DATA_2
REG=1, 0xCB, 0x04FD 	// MCU_DATA_3
REG=1, 0xCC, 0x02C0 	// MCU_DATA_4
REG=1, 0xCD, 0xFE02 	// MCU_DATA_5
REG=1, 0xCE, 0xC0EC 	// MCU_DATA_6
REG=1, 0xCF, 0x00FD 	// MCU_DATA_7
REG=1, 0xC6, 0x0740 	// MCU_ADDRESS
REG=1, 0xC8, 0x02C2 	// MCU_DATA_0
REG=1, 0xC9, 0x306F 	// MCU_DATA_1
REG=1, 0xCA, 0x08E6 	// MCU_DATA_2
REG=1, 0xCB, 0x084F 	// MCU_DATA_3
REG=1, 0xCC, 0x05F3 	// MCU_DATA_4
REG=1, 0xCD, 0x02C2 	// MCU_DATA_5
REG=1, 0xCE, 0x8FEC 	// MCU_DATA_6
REG=1, 0xCF, 0x0030 	// MCU_DATA_7
REG=1, 0xC6, 0x0750 	// MCU_ADDRESS
REG=1, 0xC8, 0xED00 	// MCU_DATA_0
REG=1, 0xC9, 0xE608 	// MCU_DATA_1
REG=1, 0xCA, 0x4F05 	// MCU_DATA_2
REG=1, 0xCB, 0xE304 	// MCU_DATA_3
REG=1, 0xCC, 0x188F 	// MCU_DATA_4
REG=1, 0xCD, 0xEC00 	// MCU_DATA_5
REG=1, 0xCE, 0x18ED 	// MCU_DATA_6
REG=1, 0xCF, 0x006C 	// MCU_DATA_7
REG=1, 0xC6, 0x0760 	// MCU_ADDRESS
REG=1, 0xC8, 0x08E6 	// MCU_DATA_0
REG=1, 0xC9, 0x08C1 	// MCU_DATA_1
REG=1, 0xCA, 0x0E25 	// MCU_DATA_2
REG=1, 0xCB, 0xDEEE 	// MCU_DATA_3
REG=1, 0xCC, 0x04CC 	// MCU_DATA_4
REG=1, 0xCD, 0x04FA 	// MCU_DATA_5
REG=1, 0xCE, 0xED04 	// MCU_DATA_6
REG=1, 0xCF, 0x30EE 	// MCU_DATA_7
REG=1, 0xC6, 0x0770 	// MCU_ADDRESS
REG=1, 0xC8, 0x04CC 	// MCU_DATA_0
REG=1, 0xC9, 0x0615 	// MCU_DATA_1
REG=1, 0xCA, 0xED0A 	// MCU_DATA_2
REG=1, 0xCB, 0x30EE 	// MCU_DATA_3
REG=1, 0xCC, 0x04CC 	// MCU_DATA_4
REG=1, 0xCD, 0x064C 	// MCU_DATA_5
REG=1, 0xCE, 0xED0C 	// MCU_DATA_6
REG=1, 0xCF, 0xCC02 	// MCU_DATA_7
REG=1, 0xC6, 0x0780 	// MCU_ADDRESS
REG=1, 0xC8, 0xC4DD 	// MCU_DATA_0
REG=1, 0xC9, 0x00CC 	// MCU_DATA_1
REG=1, 0xCA, 0x02E4 	// MCU_DATA_2
REG=1, 0xCB, 0x30ED 	// MCU_DATA_3
REG=1, 0xCC, 0x02FE 	// MCU_DATA_4
REG=1, 0xCD, 0x1050 	// MCU_DATA_5
REG=1, 0xCE, 0xEC02 	// MCU_DATA_6
REG=1, 0xCF, 0xFD02 	// MCU_DATA_7
REG=1, 0xC6, 0x0790 	// MCU_ADDRESS
REG=1, 0xC8, 0xE0FE 	// MCU_DATA_0
REG=1, 0xC9, 0x02E0 	// MCU_DATA_1
REG=1, 0xCA, 0xEC00 	// MCU_DATA_2
REG=1, 0xCB, 0xFD02 	// MCU_DATA_3
REG=1, 0xCC, 0xE230 	// MCU_DATA_4
REG=1, 0xCD, 0x6F08 	// MCU_DATA_5
REG=1, 0xCE, 0xE608 	// MCU_DATA_6
REG=1, 0xCF, 0x4F05 	// MCU_DATA_7
REG=1, 0xC6, 0x07A0 	// MCU_ADDRESS
REG=1, 0xC8, 0xF302 	// MCU_DATA_0
REG=1, 0xC9, 0xE28F 	// MCU_DATA_1
REG=1, 0xCA, 0xEC00 	// MCU_DATA_2
REG=1, 0xCB, 0x30ED 	// MCU_DATA_3
REG=1, 0xCC, 0x00E6 	// MCU_DATA_4
REG=1, 0xCD, 0x084F 	// MCU_DATA_5
REG=1, 0xCE, 0x05E3 	// MCU_DATA_6
REG=1, 0xCF, 0x0218 	// MCU_DATA_7
REG=1, 0xC6, 0x07B0 	// MCU_ADDRESS
REG=1, 0xC8, 0x8FEC 	// MCU_DATA_0
REG=1, 0xC9, 0x0018 	// MCU_DATA_1
REG=1, 0xCA, 0xED00 	// MCU_DATA_2
REG=1, 0xCB, 0x6C08 	// MCU_DATA_3
REG=1, 0xCC, 0xE608 	// MCU_DATA_4
REG=1, 0xCD, 0xC112 	// MCU_DATA_5
REG=1, 0xCE, 0x25DE 	// MCU_DATA_6
REG=1, 0xCF, 0xEE02 	// MCU_DATA_7
REG=1, 0xC6, 0x07C0 	// MCU_ADDRESS
REG=1, 0xC8, 0xCC06 	// MCU_DATA_0
REG=1, 0xC9, 0x88ED 	// MCU_DATA_1
REG=1, 0xCA, 0x1CCC 	// MCU_DATA_2
REG=1, 0xCB, 0x02E4 	// MCU_DATA_3
REG=1, 0xCC, 0xDDC2 	// MCU_DATA_4
REG=1, 0xCD, 0x30C6 	// MCU_DATA_5
REG=1, 0xCE, 0x093A 	// MCU_DATA_6
REG=1, 0xCF, 0x3539 	// MCU_DATA_7
REG=1, 0xC6, 0x07D0 	// MCU_ADDRESS
REG=1, 0xC8, 0x8F4D 	// MCU_DATA_0
REG=1, 0xC9, 0x2C13 	// MCU_DATA_1
REG=1, 0xCA, 0x4353 	// MCU_DATA_2
REG=1, 0xCB, 0x8F08 	// MCU_DATA_3
REG=1, 0xCC, 0x4D2C 	// MCU_DATA_4
REG=1, 0xCD, 0x0643 	// MCU_DATA_5
REG=1, 0xCE, 0x5302 	// MCU_DATA_6
REG=1, 0xCF, 0x088F 	// MCU_DATA_7
REG=1, 0xC6, 0x07E0 	// MCU_ADDRESS
REG=1, 0xC8, 0x3902 	// MCU_DATA_0
REG=1, 0xC9, 0x098F 	// MCU_DATA_1
REG=1, 0xCA, 0x4353 	// MCU_DATA_2
REG=1, 0xCB, 0x398F 	// MCU_DATA_3
REG=1, 0xCC, 0x4D2C 	// MCU_DATA_4
REG=1, 0xCD, 0x0743 	// MCU_DATA_5
REG=1, 0xCE, 0x5302 	// MCU_DATA_6
REG=1, 0xCF, 0x8F43 	// MCU_DATA_7
REG=1, 0xC6, 0x07F0 	// MCU_ADDRESS
REG=1, 0xC8, 0x5339 	// MCU_DATA_0
REG=1, 0xC9, 0x028F 	// MCU_DATA_1
REG=1, 0xC6, 0x87F4 	// MCU_ADDRESS
REG=1, 0xC8, 0x0039 	// MCU_DATA_0
VAR=0, 0x03, 0x06C9 	// MON_ARG1
VAR8=0, 0x02, 0x0001 	// MON_CMD
#endif

static const u16 mi2010soc_patch12345_addr_104C[] = {
    0x104C,             // start MCU_ADDRESS
    1,                  // data size
    // REG=1, 0xC6, 0x104C 	// MCU_ADDRESS
    0x0000
};

static const u16 mi2010soc_patch12345_addr_0310[] = {
    0x0310,             // start MCU_ADDRESS
    0x32,               // data size
    // REG=1, 0xC6, 0x0310 	// MCU_ADDRESS
    0x3C3C, 0xCC01, 0x33BD, 0xD43F, 0x30ED, 0x02DC, 0xB7A3, 0x0223,
    // REG=1, 0xC6, 0x0320 	// MCU_ADDRESS
    0x0DF6, 0x02BD, 0xF102, 0xBE23, 0x107A, 0x02BD, 0x200B, 0xF602,
    // REG=1, 0xC6, 0x0330 	// MCU_ADDRESS
    0xBDF1, 0x02BF, 0x2403, 0x7C02, 0xBDCC, 0x011F, 0xED00, 0xF602,
    // REG=1, 0xC6, 0x0340 	// MCU_ADDRESS
    0xBD4F, 0xBDD4, 0x2BCC, 0x0130, 0xBDD4, 0x3FD7, 0xB026, 0x04C6,
    // REG=1, 0xC6, 0x0350 	// MCU_ADDRESS
    0x01D7, 0xB0CC, 0x0131, 0xBDD4, 0x3FD7, 0xB126, 0x04C6, 0x01D7,
    // REG=1, 0xC6, 0x0360 	// MCU_ADDRESS
    0xB1CC, 0x0132, 0xBDD4, 0x3FD7, 0xB25D, 0x2604, 0xC601, 0xD7B2,
    // REG=1, 0xC6, 0x0370 	// MCU_ADDRESS
    0x5F38, 0x3839
};

static const u16 mi2010soc_patch12345_addr_0400[] = {
    0x0400,             // start MCU_ADDRESS
    0x1FA,              // data size
    // REG=1, 0xC6, 0x0400 	// MCU_ADDRESS
    0x308F, 0xC3FF, 0xEF8F, 0x35F6, 0x01D0, 0x860C, 0x3DC3, 0x01DD,
    // REG=1, 0xC6, 0x0410 	// MCU_ADDRESS
    0x8FEC, 0x0630, 0xED07, 0xF601, 0xD086, 0x0C3D, 0xC301, 0xDD8F,
    // REG=1, 0xC6, 0x0420 	// MCU_ADDRESS
    0xEC04, 0x30ED, 0x05F6, 0x01D0, 0x4FED, 0x02CC, 0x0021, 0xA302,
    // REG=1, 0xC6, 0x0430 	// MCU_ADDRESS
    0xBDD4, 0x3F30, 0xED0F, 0x1F0F, 0x800A, 0xEC07, 0x04ED, 0x07EC,
    // REG=1, 0xC6, 0x0440 	// MCU_ADDRESS
    0x0504, 0xED05, 0xD65A, 0xC40F, 0xE704, 0xEC07, 0xED02, 0xE604,
    // REG=1, 0xC6, 0x0450 	// MCU_ADDRESS
    0x4FED, 0x00CC, 0x0010, 0xBDD3, 0x4330, 0xEC02, 0xED0D, 0xD65A,
    // REG=1, 0xC6, 0x0460 	// MCU_ADDRESS
    0x5454, 0x5454, 0xE704, 0xEC05, 0xED02, 0xE604, 0x4FED, 0x00CC,
    // REG=1, 0xC6, 0x0470 	// MCU_ADDRESS
    0x0010, 0xBDD3, 0x4330, 0xEC02, 0xED0B, 0xD65B, 0xC40F, 0xCB01,
    // REG=1, 0xC6, 0x0480 	// MCU_ADDRESS
    0xE704, 0xEC07, 0xED02, 0xE604, 0x4FED, 0x00CC, 0x0010, 0xBDD3,
    // REG=1, 0xC6, 0x0490 	// MCU_ADDRESS
    0x4330, 0xEC02, 0xED07, 0xD65B, 0x5454, 0x5454, 0xCB01, 0xE704,
    // REG=1, 0xC6, 0x04A0 	// MCU_ADDRESS
    0xEC05, 0xED02, 0xE604, 0x4FED, 0x00CC, 0x0010, 0xBDD3, 0x4330,
    // REG=1, 0xC6, 0x04B0 	// MCU_ADDRESS
    0xEC02, 0xED05, 0xE30B, 0xED09, 0xC300, 0x0ADD, 0x5CEC, 0x0DE3,
    // REG=1, 0xC6, 0x04C0 	// MCU_ADDRESS
    0x07ED, 0x02EC, 0x0DED, 0x00C6, 0x08BD, 0xD319, 0x30ED, 0x0FCC,
    // REG=1, 0xC6, 0x04D0 	// MCU_ADDRESS
    0x012D, 0xED00, 0xEC0F, 0xBDD4, 0x2B30, 0xEC09, 0xED02, 0xEC0B,
    // REG=1, 0xC6, 0x04E0 	// MCU_ADDRESS
    0xED00, 0xC608, 0xBDD3, 0x1930, 0xED0F, 0xCC01, 0x2EED, 0x00EC,
    // REG=1, 0xC6, 0x04F0 	// MCU_ADDRESS
    0x0FBD, 0xD42B, 0x30C6, 0x113A, 0x3539, 0x308F, 0xC3FF, 0xED8F,
    // REG=1, 0xC6, 0x0500 	// MCU_ADDRESS
    0x35F6, 0x01D0, 0x860E, 0x3DC3, 0x01F5, 0x8FEC, 0x0030, 0xED06,
    // REG=1, 0xC6, 0x0510 	// MCU_ADDRESS
    0xF601, 0xD086, 0x0E3D, 0xC301, 0xF58F, 0xEC04, 0x30ED, 0x04EC,
    // REG=1, 0xC6, 0x0520 	// MCU_ADDRESS
    0x15ED, 0x0BEC, 0x17ED, 0x09D6, 0x02C4, 0x0FE7, 0x08EC, 0x0BED,
    // REG=1, 0xC6, 0x0530 	// MCU_ADDRESS
    0x02E6, 0x084F, 0xED00, 0xCC00, 0x10BD, 0xD343, 0x30EC, 0x02ED,
    // REG=1, 0xC6, 0x0540 	// MCU_ADDRESS
    0x0FE3, 0x06ED, 0x0FD6, 0x0254, 0x5454, 0x54E7, 0x08EC, 0x09ED,
    // REG=1, 0xC6, 0x0550 	// MCU_ADDRESS
    0x02E6, 0x084F, 0xED00, 0xCC00, 0x10BD, 0xD343, 0x30EC, 0x02ED,
    // REG=1, 0xC6, 0x0560 	// MCU_ADDRESS
    0x0DE3, 0x04ED, 0x0DD6, 0x03C4, 0x0FCB, 0x01E7, 0x08EC, 0x0BED,
    // REG=1, 0xC6, 0x0570 	// MCU_ADDRESS
    0x02E6, 0x084F, 0xED00, 0xCC00, 0x40BD, 0xD343, 0x30EC, 0x02ED,
    // REG=1, 0xC6, 0x0580 	// MCU_ADDRESS
    0x0BD6, 0x0354, 0x5454, 0x54CB, 0x01E7, 0x08EC, 0x09ED, 0x02E6,
    // REG=1, 0xC6, 0x0590 	// MCU_ADDRESS
    0x084F, 0xED00, 0xCC00, 0x40BD, 0xD343, 0x30EC, 0x02ED, 0x0905,
    // REG=1, 0xC6, 0x05A0 	// MCU_ADDRESS
    0x05E3, 0x0DC3, 0x000A, 0xDD04, 0xEC0D, 0xED02, 0xEC0F, 0xED00,
    // REG=1, 0xC6, 0x05B0 	// MCU_ADDRESS
    0xC608, 0xBDD3, 0x1930, 0xED11, 0xCC02, 0xC0ED, 0x00EC, 0x11BD,
    // REG=1, 0xC6, 0x05C0 	// MCU_ADDRESS
    0xD42B, 0x30EC, 0x09ED, 0x02EC, 0x0BED, 0x00C6, 0x02BD, 0xD319,
    // REG=1, 0xC6, 0x05D0 	// MCU_ADDRESS
    0x30ED, 0x11CC, 0x02C1, 0xED00, 0xEC11, 0xBDD4, 0x2B7F, 0x10C4,
    // REG=1, 0xC6, 0x05E0 	// MCU_ADDRESS
    0x30EC, 0x09C4, 0xFEFD, 0x10C5, 0xEC0B, 0xC4FE, 0xFD10, 0xC701,
    // REG=1, 0xC6, 0x05F0 	// MCU_ADDRESS
    0x0101, 0xCC02, 0xC2ED, 0x00FC, 0x10C2, 0xBDD4, 0x2BFC, 0x10C0,
    // REG=1, 0xC6, 0x0600 	// MCU_ADDRESS
    0xCA06, 0x30ED, 0x11CC, 0x02C3, 0xED00, 0xEC11, 0xBDD4, 0x2B30,
    // REG=1, 0xC6, 0x0610 	// MCU_ADDRESS
    0xC613, 0x3A35, 0x393C, 0xDC25, 0x30ED, 0x00BD, 0x81BE, 0x7D00,
    // REG=1, 0xC6, 0x0620 	// MCU_ADDRESS
    0x1E27, 0x227F, 0x10C4, 0xD61E, 0x4FFD, 0x10C5, 0xDC2F, 0xFD10,
    // REG=1, 0xC6, 0x0630 	// MCU_ADDRESS
    0xC701, 0x0101, 0xFC10, 0xC2DD, 0x25D6, 0x31C1, 0x0224, 0x0BC6,
    // REG=1, 0xC6, 0x0640 	// MCU_ADDRESS
    0x02D7, 0x3120, 0x0530, 0xEC00, 0xDD25, 0x3839, 0x373C, 0x3CD6,
    // REG=1, 0xC6, 0x0650 	// MCU_ADDRESS
    0x1E30, 0xE701, 0xDC25, 0xED02, 0xE607, 0xE700, 0xE604, 0xBD87,
    // REG=1, 0xC6, 0x0660 	// MCU_ADDRESS
    0x01D6, 0x1FD1, 0x1023, 0x04D6, 0x10D7, 0x1F30, 0xE607, 0xC101,
    // REG=1, 0xC6, 0x0670 	// MCU_ADDRESS
    0x2612, 0xD61E, 0xE101, 0x240C, 0xD610, 0xD71F, 0xE601, 0xD71E,
    // REG=1, 0xC6, 0x0680 	// MCU_ADDRESS
    0xEC02, 0xDD25, 0x3838, 0x3139, 0x3CDE, 0x00EE, 0x12AD, 0x00D6,
    // REG=1, 0xC6, 0x0690 	// MCU_ADDRESS
    0x4630, 0xE701, 0xC601, 0xE700, 0xE600, 0x4F8F, 0xE646, 0x30EB,
    // REG=1, 0xC6, 0x06A0 	// MCU_ADDRESS
    0x01E7, 0x016C, 0x00E6, 0x00C1, 0x0525, 0xED4F, 0xE601, 0x2A01,
    // REG=1, 0xC6, 0x06B0 	// MCU_ADDRESS
    0x43CE, 0x0005, 0xBD07, 0xD0D7, 0x4E30, 0x6F01, 0x96D5, 0x112F,
    // REG=1, 0xC6, 0x06C0 	// MCU_ADDRESS
    0x04C6, 0x01E7, 0x01E6, 0x0138, 0x393C, 0x3C3C, 0x3C34, 0xC620,
    // REG=1, 0xC6, 0x06D0 	// MCU_ADDRESS
    0xF702, 0xBD7F, 0x02BE, 0xF702, 0xBFC6, 0xF6D7, 0xBACC, 0x02AB,
    // REG=1, 0xC6, 0x06E0 	// MCU_ADDRESS
    0x30ED, 0x06FE, 0x1050, 0xEC06, 0xFD02, 0xA7FE, 0x02A7, 0xEC00,
    // REG=1, 0xC6, 0x06F0 	// MCU_ADDRESS
    0xFD02, 0xA930, 0x6F08, 0xE608, 0x4F05, 0xF302, 0xA98F, 0xEC00,
    // REG=1, 0xC6, 0x0700 	// MCU_ADDRESS
    0x30ED, 0x00E6, 0x084F, 0x05E3, 0x0618, 0x8FEC, 0x0018, 0xED00,
    // REG=1, 0xC6, 0x0710 	// MCU_ADDRESS
    0x6C08, 0xE608, 0xC109, 0x25DE, 0xEE06, 0xCC03, 0x10ED, 0x0230,
    // REG=1, 0xC6, 0x0720 	// MCU_ADDRESS
    0xEE06, 0xCC04, 0x00ED, 0x04CC, 0x02AB, 0xDD58, 0xCC02, 0xC430,
    // REG=1, 0xC6, 0x0730 	// MCU_ADDRESS
    0xED04, 0xFE10, 0x50EC, 0x04FD, 0x02C0, 0xFE02, 0xC0EC, 0x00FD,
    // REG=1, 0xC6, 0x0740 	// MCU_ADDRESS
    0x02C2, 0x306F, 0x08E6, 0x084F, 0x05F3, 0x02C2, 0x8FEC, 0x0030,
    // REG=1, 0xC6, 0x0750 	// MCU_ADDRESS
    0xED00, 0xE608, 0x4F05, 0xE304, 0x188F, 0xEC00, 0x18ED, 0x006C,
    // REG=1, 0xC6, 0x0760 	// MCU_ADDRESS
    0x08E6, 0x08C1, 0x0E25, 0xDEEE, 0x04CC, 0x04FA, 0xED04, 0x30EE,
    // REG=1, 0xC6, 0x0770 	// MCU_ADDRESS
    0x04CC, 0x0615, 0xED0A, 0x30EE, 0x04CC, 0x064C, 0xED0C, 0xCC02,
    // REG=1, 0xC6, 0x0780 	// MCU_ADDRESS
    0xC4DD, 0x00CC, 0x02E4, 0x30ED, 0x02FE, 0x1050, 0xEC02, 0xFD02,
    // REG=1, 0xC6, 0x0790 	// MCU_ADDRESS
    0xE0FE, 0x02E0, 0xEC00, 0xFD02, 0xE230, 0x6F08, 0xE608, 0x4F05,
    // REG=1, 0xC6, 0x07A0 	// MCU_ADDRESS
    0xF302, 0xE28F, 0xEC00, 0x30ED, 0x00E6, 0x084F, 0x05E3, 0x0218,
    // REG=1, 0xC6, 0x07B0 	// MCU_ADDRESS
    0x8FEC, 0x0018, 0xED00, 0x6C08, 0xE608, 0xC112, 0x25DE, 0xEE02,
    // REG=1, 0xC6, 0x07C0 	// MCU_ADDRESS
    0xCC06, 0x88ED, 0x1CCC, 0x02E4, 0xDDC2, 0x30C6, 0x093A, 0x3539,
    // REG=1, 0xC6, 0x07D0 	// MCU_ADDRESS
    0x8F4D, 0x2C13, 0x4353, 0x8F08, 0x4D2C, 0x0643, 0x5302, 0x088F,
    // REG=1, 0xC6, 0x07E0 	// MCU_ADDRESS
    0x3902, 0x098F, 0x4353, 0x398F, 0x4D2C, 0x0743, 0x5302, 0x8F43,
    // REG=1, 0xC6, 0x07F0 	// MCU_ADDRESS
    0x5339, 0x028F
};

static const u16 mi2010soc_patch12345_addr_87F4[] = {
    0x87F4,             // start MCU_ADDRESS
    1,                  // data size
    // REG=1, 0xC6, 0x87F4 	// MCU_ADDRESS
    0x0039
};

static void mi2010soc_load_patch_segment(const u16* patch)
{
    u16 start, size, data, offreg;
    const u16 *pdata;
    int offset;
    
    start = patch[0];   // start MCU_ADDRESS
    size = patch[1];    // data size
    pdata = patch+2;     // data pointer
    offset = 0;         // MCU_DATA_0,7
    while(size>0)
    {
        if(offset==0)
        {
            mi2010soc_reg_write(0x1C6, start);  // write MCU_ADDRESS
            //printk(KERN_INFO "REG=1, 0xC6, 0x%04X \t// MCU_ADDRESS\n", start);
        }
        data = *pdata++;
        offreg = 0x1C8 + offset;
        mi2010soc_reg_write(offreg, data);  // write MCU_ADDRESS
        //printk(KERN_INFO "REG=1, 0x%02X, 0x%04X \t// MCU_DATA_%d\n", offreg&0xff, data, offset);

        start += 2;
        size--;
        offset++;
        if(offset==8)
            offset = 0;
    }
    return;
}

static void mi2010soc_load_patch(void)
{
    dbg_print("MI2010SOC Rev3 patch12345");
    
    mi2010soc_load_patch_segment(mi2010soc_patch12345_addr_104C);
    mi2010soc_load_patch_segment(mi2010soc_patch12345_addr_0310);
    mi2010soc_load_patch_segment(mi2010soc_patch12345_addr_0400);
    mi2010soc_load_patch_segment(mi2010soc_patch12345_addr_87F4);
    
    // VAR=0, 0x03, 0x06C9 	// MON_ARG1
    mi2010soc_var_write16(0, 3, 0x06C9);
    // VAR8=0, 0x02, 0x0001 	// MON_CMD
    mi2010soc_var_write8(0, 2, 0x0001);

    return;
}

int mi2010soc_reset_init(void)
{
    // clear values
    mi2010soc_reset_value();

    // get chip id and revision and save
    mi2010soc_get_device_id(NULL, NULL);

    if(sensor_revision==3)
        mi2010soc_load_patch();     // the patch only used on Rev3

    mi2010soc_set_pll();
    return 0;
}

