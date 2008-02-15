/*
 * Copyright 2004 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2005 - Motorola
 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 * Motorola 2005-Feb-28 - Rewrote the software for PCAP.
 *
 */

/*!
 * @file lights_backlight.c
 *
 * @brief Backlight setting
 *
 *  This is the main file of the backlight interface which conrols the backlight setting.
 *
 * @ingroup poweric_lights
 */

#include <linux/power_ic.h>
#include <linux/lights_backlight.h>
#include <linux/lights_funlights.h>
#include <linux/module.h>


/*******************************************************************************************
                                           CONSTANTS
 ******************************************************************************************/

/********************************************************************************************
                                 STRUCTURES AND OTHER TYPEDEFS
*******************************************************************************************/

/*******************************************************************************************
 * GLOBAL FUNCTIONS
 ******************************************************************************************/
 /*!
  * @brief  Backlight setting
  *
  * This function activates/deactivates (with intensity for the main and cli displays)
  * the selected backlight(s).
  *
  * @param bl_select      Indicates which backlights to enable/disable
  * @param bl_brightness  Levels of brightness.
  * @return     none
  */

void lights_backlightset(LIGHTS_BACKLIGHT_T bl_select, LIGHTS_BACKLIGHT_BRIGHTNESS_T bl_brightness)
{

    LIGHTS_FL_COLOR_T nColor;
    
    /* if bl_select is out of range, bl_select is
       set to all backlights */
    if (bl_select > LIGHTS_BACKLIGHT_ALL)
    {
        bl_select = LIGHTS_BACKLIGHT_ALL;
    }

    /*Convert the brightness level to real color */
    nColor = (bl_brightness << LIGHTS_FL_COLOR_RED_SFT)|
        (bl_brightness << LIGHTS_FL_COLOR_GREEN_SFT)|
        (bl_brightness<< LIGHTS_FL_COLOR_BLUE_SFT);
  
    /* Update the stored color for the selected backlight(s).
       LIGHTA_FL_update uses color not intensity so store
       the display brightness as a color (shade of white) */
    switch (bl_select)
    {
        case LIGHTS_BACKLIGHT_KEYPAD:                   
            lights_fl_update(LIGHTS_FL_APP_CTL_DEFAULT, 1,
                             LIGHTS_FL_REGION_KEYPAD_NUM_BL,
                             nColor);
            
            /*The navigation light is considered as part of keypad light, it is turned on/off with
              keypad light */                
        case LIGHTS_BACKLIGHT_NAV:
            lights_fl_update(LIGHTS_FL_APP_CTL_DEFAULT, 1,
                             LIGHTS_FL_REGION_KEYPAD_NAV_BL,
                             nColor);
            break;
                
        case LIGHTS_BACKLIGHT_DISPLAY:
            lights_fl_update(LIGHTS_FL_APP_CTL_DEFAULT, 1,
                             LIGHTS_FL_REGION_DISPLAY_BL,
                             nColor );
            break;

        case LIGHTS_BACKLIGHT_CLI_DISPLAY:
            lights_fl_update(LIGHTS_FL_APP_CTL_DEFAULT, 1,
                             LIGHTS_FL_REGION_CLI_DISPLAY_BL,
                             nColor);
            break;
                
        case LIGHTS_BACKLIGHT_ALL:
            lights_fl_update(LIGHTS_FL_APP_CTL_DEFAULT, 4,
                             LIGHTS_FL_REGION_KEYPAD_NUM_BL,
                             nColor,
                             LIGHTS_FL_REGION_KEYPAD_NAV_BL,
                             nColor,
                             LIGHTS_FL_REGION_CLI_DISPLAY_BL,
                             nColor,
                             LIGHTS_FL_REGION_DISPLAY_BL,
                             nColor
                             );
            break;
    }
}


/*==================================================================================================
                                     EXPORTED SYMBOLS
==================================================================================================*/

#ifndef DOXYGEN_SHOULD_SKIP_THIS
EXPORT_SYMBOL(lights_backlightset);
#endif   
 
