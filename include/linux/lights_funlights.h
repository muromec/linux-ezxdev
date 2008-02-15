/*
 * Copyright 2004-2005 Motorola, Inc.
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
*/

/* 
 * Maintained by Sierra Team of Motorola.
 */
 
/*!
 * @file lights_funlights.h
 *
 * @brief Global symbol definitions for the fun lights interface.
 *
 * @ingroup poweric_lights
 */

#ifndef _LIGHTS_FUNLIGHTS_H
#define _LIGHTS_FUNLIGHTS_H

/*==================================================================================================
                                         INCLUDE FILES
==================================================================================================*/
#include <stdbool.h>
#include <stdarg.h>

/*==================================================================================================
                                           CONSTANTS
==================================================================================================*/

/*
 * The maximum number of regions which can be controled by HAPI.
 */
#define LIGHTS_FL_MAX_REGIONS     8


/*
 * Used in conjunction with LIGHTS_FL_Control_Set to set or remove control of regions.  Region 1
 * is bit 0 and region 8 is bit 7 as defined below.
 */
#define LIGHTS_FL_REGION01_MSK    0x01
#define LIGHTS_FL_REGION02_MSK    0x02
#define LIGHTS_FL_REGION03_MSK    0x04
#define LIGHTS_FL_REGION04_MSK    0x08
#define LIGHTS_FL_REGION05_MSK    0x10
#define LIGHTS_FL_REGION06_MSK    0x20
#define LIGHTS_FL_REGION07_MSK    0x40
#define LIGHTS_FL_REGION08_MSK    0x80
#define LIGHTS_FL_ALL_REGIONS_MSK 0xff

/*
 * Define the basic colors which are supported by all of the tri colored LED drivers to be used
 * conjunction with LIGHTS_FL_COLOR_T.
 */
#define LIGHTS_FL_COLOR_BLACK   0x000000
#define LIGHTS_FL_COLOR_BLUE    0x0000ff
#define LIGHTS_FL_COLOR_GREEN   0x00ff00
#define LIGHTS_FL_COLOR_CYAN    0x00ffff
#define LIGHTS_FL_COLOR_RED     0xff0000  
#define LIGHTS_FL_COLOR_MAGENTA 0xff00ff
#define LIGHTS_FL_COLOR_YELLOW  0xffff00
#define LIGHTS_FL_COLOR_WHITE   0xffffff

/*
 * Define the number of times a color value must be shifted in order to isolate only
 * the colors intensity.
 */
#define LIGHTS_FL_COLOR_BLUE_SFT  0
#define LIGHTS_FL_COLOR_GREEN_SFT 8
#define LIGHTS_FL_COLOR_RED_SFT   16

/*
 * Define some fixed regions which are used by the apps or DL to control backlights.  Please
 * note, not all of these are valid for all phones.  See the region control table
 * (LIGHTS_FL_region_ctl_tb) in dl_led_fun_lights.c for the specifics.  These were changed to
 * constants in order to keep all configuration data in the source file and not spread it out.
 */
#define LIGHTS_FL_REGION_DISPLAY_BL     1
#define LIGHTS_FL_REGION_KEYPAD_NAV_BL  2
#define LIGHTS_FL_REGION_KEYPAD_NUM_BL  2
#define LIGHTS_FL_REGION_TRI_COLOR_1    3
#define LIGHTS_FL_REGION_TRI_COLOR_2    4
#define LIGHTS_FL_REGION_CLI_DISPLAY_BL 5
#define LIGHTS_FL_REGION_CAMERA_FLASH   6
#define LIGHTS_FL_REGION_WIFI_STATUS    7
#define LIGHTS_FL_REGION_SOL_LED        8

/*
 * Define differen mode for camera flash
 */

#define LIGHTS_FL_CAMERA_TORCH   0x444444
#define LIGHTS_FL_CAMERA_REDEYE  0x888888
#define LIGHTS_FL_CAMERA_FLASH   0xffffff

/*==================================================================================================
                                            MACROS
==================================================================================================*/

/*==================================================================================================
                                             ENUMS
==================================================================================================*/

/*
 * All of the apps which are allowed to control the LED's listed in increasing priority.
 */
typedef enum
{
   LIGHTS_FL_APP_CTL_DEFAULT = 0,
   LIGHTS_FL_APP_CTL_KJAVA,
   LIGHTS_FL_APP_CTL_FUN_LIGHTS,
   LIGHTS_FL_APP_CTL_TST_CMDS,
   LIGHTS_FL_APP_CTL_END
} LIGHTS_FL_APP_CTL_T;

/*==================================================================================================
                                 STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/
/*
 * Define the type of fun light LED color   The format of the type is to use the lower 24 bits
 * for color in the format rrrrrrrrggggggggbbbbbbbb where each position is a bit with r being the red
 * value, g being the green value and b being the blue value. If a region is not a
 * tri-color LED, then any non BLACK (0) value will result in the LED being turned on.
 *
 * Note:
 *   If the size of this typedef is made greater than the size of an unsigned int the
 *   code in LIGHTS_FL_vupdate will need to be updated.
 */
typedef unsigned int LIGHTS_FL_COLOR_T;

/*
 * Type used to define which region number will be affected for functions which take a region
 * value not a mask.
 *
 * Note: If the size of this typedef is made greater than the size of an unsigned int the
 * code in LIGHTS_FL_vupdate will need to be updated.
 */
typedef unsigned char LIGHTS_FL_REGION_T;

/*
 * Define type which can be used to hold a bit mask with 1 bit per region.
 */
typedef unsigned char LIGHTS_FL_REGION_MSK_T;

/*
 * Define the type of struct for the user space request to set the fun lights mode
 */
typedef struct
{
    LIGHTS_FL_APP_CTL_T nApp;         
    LIGHTS_FL_REGION_MSK_T  nRegionMsk;
    LIGHTS_FL_REGION_MSK_T return_mask;
}LIGHTS_FL_SET_T;

/*
 * Define the type of struct for the user space request to update funlight
 */
typedef struct
{
    LIGHTS_FL_APP_CTL_T nApp;         
    LIGHTS_FL_REGION_T nRegion;
    LIGHTS_FL_COLOR_T nColor;
    LIGHTS_FL_REGION_MSK_T return_mask;
}LIGHTS_FL_UPDATE_T;

#ifdef __KERNEL__
/*
 * The structure below is used to pass information to the functions which are responsible for enabling
 * the LEDs for different regions. 
 */
typedef struct
{
    unsigned char port_id;              /* The gpio port number */
    unsigned char nLed;                 /* LED ID which is passed to the  set_tri_color_led. */
    unsigned char nSwitchedValue;       /* This value is used to enable or disabled the LED */
} LIGHTS_FL_LED_CTL_T;

/*
 * Structure used to configure the regions for different products.  See the configuration tables
 * below for more details.
 */
typedef struct
{
    bool (* const pFcn)(LIGHTS_FL_LED_CTL_T *, LIGHTS_FL_COLOR_T);
    LIGHTS_FL_LED_CTL_T xCtlData;  /* See LIGHTS_FL_LED_CTL_T for a description. */
} LIGHTS_FL_REGION_CFG_T;
#endif

/*==================================================================================================
                                 GLOBAL VARIABLE DECLARATION
==================================================================================================*/

/*==================================================================================================
                                     FUNCTION PROTOTYPES
==================================================================================================*/

/*uint8_t LIGHTS_FL_get_current_draw(void);*/
extern LIGHTS_FL_REGION_MSK_T lights_fl_set_control 
(
   LIGHTS_FL_APP_CTL_T nApp,                
   LIGHTS_FL_REGION_MSK_T nRegionMask        
);

extern LIGHTS_FL_REGION_MSK_T lights_fl_vupdate  
(
    LIGHTS_FL_APP_CTL_T nApp,           
    unsigned char nRegions, 
    va_list pData                        
);

#ifdef __KERNEL__

extern LIGHTS_FL_REGION_MSK_T lights_fl_update   
(
    LIGHTS_FL_APP_CTL_T nApp,            
    unsigned char nRegions,
    ...
);

extern int lights_ioctl(unsigned int cmd, unsigned long arg);

#else
extern LIGHTS_FL_REGION_MSK_T lights_fl_update   
(
    LIGHTS_FL_APP_CTL_T nApp,            
    LIGHTS_FL_REGION_T nRegion,
    LIGHTS_FL_COLOR_T  nColor
);
extern int lights_init(void);
extern int lights_close(void);
#endif /* __KERNEL__ */

#endif
