/*
 *  adcm2650.h
 *
 *  ADCM 2650 Camera Module driver header.
 *
 *  Copyright (C) 2003, Intel Corporation
 *  Copyright (C) 2003, Montavista Software Inc.
 *
 *  Author: Intel Corporation Inc.
 *          MontaVista Software, Inc.
 *           source@mvista.com
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
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#ifndef __ADCM2650_H__
#define __ADCM2650_H__

#include "pxa_camera.h"

//////////////////////////////////////////////////////////////////////////////////////
//
//          Prototypes
//
//////////////////////////////////////////////////////////////////////////////////////

int camera_func_adcm2650_init( p_camera_context_t );
int camera_func_adcm2650_deinit( p_camera_context_t );
int camera_func_adcm2650_set_capture_format( p_camera_context_t );
int camera_func_adcm2650_start_capture( p_camera_context_t, unsigned int frames );
int camera_func_adcm2650_stop_capture( p_camera_context_t );

#endif /* __ADCM2650_H__ */

