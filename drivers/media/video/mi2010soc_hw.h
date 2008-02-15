/*
 *  mi2010soc_hw.h
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
Mu Chenliang        4/30/2005                    created, for EZX platform

*/

#ifndef _PXA_MI2010SOC_HW_H__
#define _PXA_MI2010SOC_HW_H__

#include "camera.h"

/***********************************************************************
 * 
 * Constants & Structures
 *
 ***********************************************************************/
 
#define MAX_SENSOR_B_WIDTH      1600
#define MAX_SENSOR_B_HEIGHT     1200
#define MAX_SENSOR_A_WIDTH      (MAX_SENSOR_B_WIDTH/2) 
#define MAX_SENSOR_A_HEIGHT     (MAX_SENSOR_B_HEIGHT/2)

#define DEFT_SENSOR_B_WIDTH     1600
#define DEFT_SENSOR_B_HEIGHT    1200
#define DEFT_SENSOR_A_WIDTH     320
#define DEFT_SENSOR_A_HEIGHT    240

/* MICRON_WINDOWSIZE */
typedef struct {
    u16 width;
    u16 height;
} micron_window_size;


// Return codes
#define MICRON_ERR_NONE        0
#define MICRON_ERR_TIMEOUT    -1
#define MICRON_ERR_PARAMETER  -2  


// Output Format
#define O_FORMAT_422_YCbYCr   0
#define O_FORMAT_565_RGB      1
#define O_FORMAT_555_RGB      2
#define O_FORMAT_444_RGB      3
#define O_FORMAT_JPEG         4
#define O_FORMAT_JPEG_STATUS  5

                                                                            
/***********************************************************************                   
 *                                                                                         
 * Function Prototype                 
 *                                    
 ***********************************************************************/

u16  mi2010soc_reg_read(u16 reg_addr);
void mi2010soc_reg_write(u16 reg_addr, u16 reg_value);


// Configuration Procedures
int mi2010soc_get_device_id(u16 *id, u16 *rev);

int mi2010soc_viewfinder_on( void );
int mi2010soc_viewfinder_off( void );
int mi2010soc_snapshot_trigger( void );
int mi2010soc_snapshot_complete( void );

int mi2010soc_set_fps(u16 fps);
int mi2010soc_set_exposure_mode(int mode, u32 maxexpotime);

int mi2010soc_sensor_size(micron_window_size * win);
int mi2010soc_output_size(micron_window_size * win);
int mi2010soc_output_format(u16 format);
int mi2010soc_capture_sensor_size(micron_window_size * win);
int mi2010soc_capture_size(micron_window_size * win);
int mi2010soc_capture_format(u16 format);
int mi2010soc_set_spoof_size(micron_window_size * win);
int mi2010soc_get_spoof_size(micron_window_size * win);

int mi2010soc_set_contrast(int contrast);
int mi2010soc_set_style(V4l_PIC_STYLE style);
int mi2010soc_set_light(V4l_PIC_WB light);
int mi2010soc_set_bright(int bright);
int mi2010soc_set_flicker(int bright);
int mi2010soc_set_mirror(int rows, int columns);
int mi2010soc_set_jpeg_scale(int scale);

int mi2010soc_get_jpeg_status(int *length, int *status);

int mi2010soc_default_settings( void );
int mi2010soc_standby_enter(void);
int mi2010soc_standby_exit(void);
int mi2010soc_reset_init(void);

void mi2010soc_dump_register(u16 startRegAddr, u16 endRegAddr, u16* buffer);
void mi2010soc_dump_debug_regs(void);

#endif /* _PXA_MI2010SOC_HW_H__ */

