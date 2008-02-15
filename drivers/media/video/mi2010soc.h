/*
 *  mi2010soc.h
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

/*================================================================================
                                 INCLUDE FILES
==================================================================================*/

#ifndef _MI2010SOC_H_
#define _MI2010SOC_H_

#include "camera.h"

//////////////////////////////////////////////////////////////////////////////////////
//
//          Prototypes
//
//////////////////////////////////////////////////////////////////////////////////////

int camera_func_mi2010soc_init(p_camera_context_t);
int camera_func_mi2010soc_deinit(p_camera_context_t);
int camera_func_mi2010soc_set_capture_format(p_camera_context_t);
int camera_func_mi2010soc_start_capture(p_camera_context_t, unsigned int frames);
int camera_func_mi2010soc_stop_capture(p_camera_context_t);
int camera_func_mi2010soc_pm_management(p_camera_context_t cam_ctx, int suspend);
int camera_func_mi2010soc_docommand(p_camera_context_t cam_ctx, unsigned int cmd, void *param);

#endif /* _MI2010SOC_H_ */


