/*
 *  mt9m111.h
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

*/

/*================================================================================
                                 INCLUDE FILES
==================================================================================*/

#ifndef _MT9M111_H_
#define _MT9M111_H_

#include "camera.h"

//////////////////////////////////////////////////////////////////////////////////////
//
//          Prototypes
//
//////////////////////////////////////////////////////////////////////////////////////

int camera_func_mt9m111_init(p_camera_context_t);
int camera_func_mt9m111_deinit(p_camera_context_t);
int camera_func_mt9m111_set_capture_format(p_camera_context_t);
int camera_func_mt9m111_start_capture(p_camera_context_t, unsigned int frames);
int camera_func_mt9m111_stop_capture(p_camera_context_t);
int camera_func_mt9m111_pm_management(p_camera_context_t cam_ctx, int suspend);
int camera_func_mt9m111_docommand(p_camera_context_t cam_ctx, unsigned int cmd, void *param);
int camera_func_mt9m111_standby(void);

extern int light_sensor_mt9m111_init(void);
extern int light_sensor_mt9m111_deinit(void);
extern int light_sensor_mt9m111_get_luma(int *lux_int, int *lux_dec);

#endif /* _MT9M111_H_ */


