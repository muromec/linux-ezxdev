/* 
    ov9640.h - Omnivision 9640 CMOS sensor driver 

    Copyright (C) 2003, Intel Corporation
    Copyright (C) 2004 Motorola Inc.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

/*
Revision History:
                            Modification     Tracking
Author                 Date          Number     Description of Changes
----------------   ------------    ----------   -------------------------
Wang Wenxing)      03/16/2004                   Porting to EZX platform
Wang Wenxing)      07/30/2004                   Improve, tunning
*/

#ifndef _OV_9640_H_
#define _OV_9640_H_

#include <linux/videodev.h>
#include "camera.h"
typedef struct {
	u32 version;
	u32 stoped;
	u32 mclock;
	u32 pre_size;
	u32 exp_value;
	u32 exp_time;
	u32 adjusted_exp_value;
	u32 adjusted_exp_time;
	u8 pclock;
	u8 gain;
	u8 blue_gain;
	u8 red_gain;
	u8 y_average;
	u32 sensor_width;
	u32 sensor_height;
	u32 sub_win;
	u32 night_mode;
	struct video_window win;
}ov9640;
	
//////////////////////////////////////////////////////////////////////////////////////
//
//          Prototypes
//
//////////////////////////////////////////////////////////////////////////////////////

int camera_func_ov9640_init( p_camera_context_t );

int camera_func_ov9640_deinit( p_camera_context_t );

int camera_func_ov9640_sleep(  p_camera_context_t camera_context );

int camera_func_ov9640_wake(  p_camera_context_t camera_context );

int camera_func_ov9640_set_capture_format( p_camera_context_t );

int camera_func_ov9640_start_capture( p_camera_context_t, unsigned int frames );

int camera_func_ov9640_stop_capture( p_camera_context_t );

int camera_func_ov9640_pm(p_camera_context_t cam_ctx, int suspend);
#endif
