/*
 *  mt9m111_hw.h
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
Ma Zhiqiang        6/30/2004                    Change for auto-detect

*/

/*================================================================================
                                 INCLUDE FILES
==================================================================================*/

#ifndef _PXA_MT9M111_HW_H__
#define _PXA_MT9M111_HW_H__

#include "camera.h"

/***********************************************************************
 * 
 * Constants & Structures
 *
 ***********************************************************************/
 
#define MAX_SENSOR_B_WIDTH      1280
#define MAX_SENSOR_B_HEIGHT     1024
#define MAX_SENSOR_A_WIDTH      MAX_SENSOR_B_WIDTH
#define MAX_SENSOR_A_HEIGHT     MAX_SENSOR_B_HEIGHT

#define DEFT_SENSOR_B_WIDTH     1280
#define DEFT_SENSOR_B_HEIGHT    1024
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

                                                                            
/***********************************************************************                   
 *                                                                                         
 * Function Prototype                 
 *                                    
 ***********************************************************************/

u16  mt9m111_reg_read(u16 reg_addr);
void mt9m111_reg_write(u16 reg_addr, u16 reg_value);


// Configuration Procedures
int mt9m111_get_device_id(u16 *id, u16 *rev);

int mt9m111_viewfinder_on( void );
int mt9m111_viewfinder_off( void );
int mt9m111_snapshot_trigger( void );
int mt9m111_snapshot_complete( void );

int mt9m111_set_fps(u16 fps, u16 minfps);
int mt9m111_set_autoexposure_zone(u16 minfps);

int mt9m111_sensor_size(micron_window_size * win);
int mt9m111_capture_sensor_size(micron_window_size * win);
int mt9m111_output_size(micron_window_size * win);
int mt9m111_capture_size(micron_window_size * win);
int mt9m111_output_format(u16 format);

int mt9m111_set_contrast(int contrast);
int mt9m111_set_style(V4l_PIC_STYLE style);
int mt9m111_set_light(V4l_PIC_WB light);
int mt9m111_set_bright(int bright);
int mt9m111_set_flicker(int bright);
int mt9m111_set_mirror(int rows, int columns);
int mt9m111_set_strobe_flash(int enable);   // 0-disable, 1-enable

int mt9m111_default_settings( void );

void mt9m111_dump_register(u16 startRegAddr, u16 endRegAddr, u16* buffer);

#endif /* _PXA_MT9M111_HW_H__ */

