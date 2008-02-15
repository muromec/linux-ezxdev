/*
 * Copyright (C) 2004-2005 Motorola, Inc.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  
 * 02111-1307, USA
 *
 */

/*!
 * @file lights_backlight.h
 *
 * @brief Global symbols definitions for lights_backlights.c interface.
 *
 * @ingroup poweric_lights
 */

/* 
 * Created by Sierra Team of Motorola
 */

#ifndef _LIGHTS_BACKLIGHTS_H
#define _LIGHTS_BACKLIGHTS_H
/*==================================================================================================
                                         INCLUDE FILES
==================================================================================================*/
#include <stdbool.h>
/*==================================================================================================
                                           CONSTANTS
==================================================================================================*/
    
/*==================================================================================================
                                            MACROS
==================================================================================================*/

/*==================================================================================================
                                             ENUMS
==================================================================================================*/
/* Defines the enum for the different regions */
typedef enum
{
    LIGHTS_BACKLIGHT_KEYPAD = 0,
    LIGHTS_BACKLIGHT_DISPLAY,
    LIGHTS_BACKLIGHT_CLI_DISPLAY,
    LIGHTS_BACKLIGHT_NAV,
    LIGHTS_BACKLIGHT_ALL
}LIGHTS_BACKLIGHT_T;

/* Defines the brightsness level, currently we set 256 brightness level*/
typedef unsigned char LIGHTS_BACKLIGHT_BRIGHTNESS_T;
/*==================================================================================================
                                 STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/
/* The structure is for the user space ioctl call */
typedef struct
{
    LIGHTS_BACKLIGHT_T bl_select;  
    LIGHTS_BACKLIGHT_BRIGHTNESS_T bl_brightness;
}LIGHTS_BACKLIGHT_SET_T;
    
/*==================================================================================================
                                 GLOBAL VARIABLE DECLARATION
==================================================================================================*/

/*==================================================================================================
                                     FUNCTION PROTOTYPES
==================================================================================================*/
void lights_backlightset
(
    LIGHTS_BACKLIGHT_T bl_select,  
    LIGHTS_BACKLIGHT_BRIGHTNESS_T bl_brightness   
);
#endif

