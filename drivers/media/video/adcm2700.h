/*
 *  adcm2700.h
 *
 *  Agilent ADCM 2700 Camera Module driver.
 *
 *  Copyright (C) 2003 Motorola Inc.
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
wangfei(w20239)      12/15/2003     LIBdd35749    Created   

*/

/*================================================================================
                                 INCLUDE FILES
==================================================================================*/

#ifndef _ADCM2700_H_
#define _ADCM2700_H_

#include "camera.h"

//////////////////////////////////////////////////////////////////////////////////////
//
//          Prototypes
//
//////////////////////////////////////////////////////////////////////////////////////

int camera_func_adcm2700_init(p_camera_context_t);
int camera_func_adcm2700_deinit(p_camera_context_t);
int camera_func_adcm2700_set_capture_format(p_camera_context_t);
int camera_func_adcm2700_start_capture(p_camera_context_t, unsigned int frames);
int camera_func_adcm2700_stop_capture(p_camera_context_t);

#endif /* _ADCM2700_H_ */


