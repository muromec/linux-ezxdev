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
 * Motorola 2005-Jun-16 - Updated the region table for new hardware.
 *
 */


/*!
 * @file lights_funlights_atlas.c
 *
 * @ingroup poweric_lights
 *
 * @brief Funlight module
 *
 *  In this file, there are interface functions between the funlights driver and outside world.
 *  These functions implement a cached priority based method of controlling fun lights regions.
 *  Upon powerup the regions are assigned to the default app, which can be looked at as the old functionality.
 *  This allows the keypad and display backlights to operate as they do today, when they are not in use by fun
 *  lights or KJAVA.  If a new application needs to be added it must be done in the corresponding
 *  header file.

 */
#include <stdbool.h>
#include <linux/kernel.h>
#include <linux/power_ic.h>
#include <linux/lights_funlights.h>
#include <linux/lights_backlight.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include "../core/os_independent.h"

/*******************************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************************/
static bool lights_fl_region_main_display(LIGHTS_FL_LED_CTL_T *pCtlData,LIGHTS_FL_COLOR_T nColor);
static bool lights_fl_region_cli_display(LIGHTS_FL_LED_CTL_T *pCtlData,LIGHTS_FL_COLOR_T nColor);
static bool lights_fl_region_keypad(LIGHTS_FL_LED_CTL_T *pCtlData,LIGHTS_FL_COLOR_T nColor);
static bool lights_fl_region_tri_color(LIGHTS_FL_LED_CTL_T *pCtlData,LIGHTS_FL_COLOR_T nColor);
static bool lights_fl_region_sol_led(LIGHTS_FL_LED_CTL_T *pCtlData,LIGHTS_FL_COLOR_T nColor); 

/*******************************************************************************************
 * LOCAL CONSTANTS
 ******************************************************************************************/
/* ATLAS Registers */
/*******************************************************************************************
 * LOCAL CONSTANTS
 ******************************************************************************************/
/* ATLAS Registers */
/* Register LED Control 0 */
#define LIGHTS_FL_CHRG_LED_EN                18

/*Register LED Control 2*/
#define LIGHTS_FL_MAIN_DISPLAY_DC_MASK       0x001E07
#define LIGHTS_FL_CLI_DISPLAY_DC_MASK        0x00E038
#define LIGHTS_FL_KEYPAD_DC_MASK             0x1E01C0

/*Register LED Control 3, 4, and 5*/
#define LIGHTS_FL_TRI_COLOR_RED_DC_INDEX     6
#define LIGHTS_FL_TRI_COLOR_RED_DC_MASK      0x0007C0
#define LIGHTS_FL_TRI_COLOR_GREEN_DC_INDEX   11
#define LIGHTS_FL_TRI_COLOR_GREEN_DC_MASK    0x00F800
#define LIGHTS_FL_TRI_COLOR_BLUE_DC_INDEX    16
#define LIGHTS_FL_TRI_COLOR_BLUE_DC_MASK     0x1F0000

#define LIGHTS_FL_TRI_COLOR_DC_MASK          0x1FFFC0

#define LIGHTS_FL_DUTY_CYCLE_SHIFT           3

/*******************************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************************/

/*
 * Table of control functions for region LED's.  The table is set up to easily allow different
 * products to conditionally select different control functions based on available hardware.  If
 * flex control of the regions is necessary, it should be done in the functions which are listed
 * in this table.
 */
LIGHTS_FL_REGION_CFG_T LIGHTS_FL_region_ctl_tb[LIGHTS_FL_MAX_REGIONS] =
{
    { lights_fl_region_main_display,{0,0, 0}}, /*Region 1 */
    { lights_fl_region_keypad,      {0,0, 1}}, /*Region 2*/
    { lights_fl_region_tri_color,   {0,1, 0}}, /*Region 3*/
    { lights_fl_region_tri_color,   {0,2, 0}}, /*Region 4*/
    { lights_fl_region_cli_display, {0,0, 0}}, /*Region 5*/
    { NULL,                         {0,0, 0}}, /*Region 6*/
    { NULL,                         {0,0, 0}}, /*Region 7*/
    { lights_fl_region_sol_led,     {0,0, 0}}  /*Region 8*/ 
};

/*******************************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************************/
 /*!
  * @brief Set main display
  *
  *  Function to handle a request for the main display backlight on devices which support variable
  *  backlight intensity and have display backlights controlled by either a GPIO of a power ic.
  *  The backlight intensity is set in lights_led_backlights.c and converted to a hardware
  *  value in this routine.
  *
  * @param   pCtlData   Pointer to the control data for the region.
  * @param   nColor     The color to set the region to.
  *
  * @return     true  region updated
  *             false  region not updated
  */

static bool lights_fl_region_main_display 
(
    LIGHTS_FL_LED_CTL_T *pCtlData, 
    LIGHTS_FL_COLOR_T nColor      
)
{
    int error;
    tracemsg(_k_d("=>Update the main display with color %d"), nColor);

    if(nColor)
    {
        /*For the main display and CLI display, 00-on,01-off*/
        error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                      LIGHTS_FL_CLI_DISPLAY_DC_MASK,
                                      0); 
    
        error |= power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                       LIGHTS_FL_MAIN_DISPLAY_DC_MASK,
                                       LIGHTS_FL_MAIN_DISPLAY_DC_MASK);
    }
    else
    {
        error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                      LIGHTS_FL_MAIN_DISPLAY_DC_MASK,
                                      0);
    } 
    return (error != 0);
}

/*!
  * @brief Set CLI display
  *        Function to handle a request for the cli display backlight on devices which support variable
  *        backlight intensity and have CLI display backlights controlled by either a GPIO port or a power
  *        IC.
  *
  * @param  pCtlData     Pointer to the control data for the region. 
  * @param  nColor       The color to set the region to. 
  *
  * @return     true - region updated
  *             false - region not updated
  */
 
static bool lights_fl_region_cli_display
(
    LIGHTS_FL_LED_CTL_T *pCtlData, 
    LIGHTS_FL_COLOR_T nColor       
)   
{   
    int error;
    tracemsg(_k_d("=>Update the cli display %d"), nColor);
    
    if(nColor)
    {
        error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                      LIGHTS_FL_MAIN_DISPLAY_DC_MASK,
                                      0);
        error |= power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                       LIGHTS_FL_CLI_DISPLAY_DC_MASK,
                                       LIGHTS_FL_CLI_DISPLAY_DC_MASK);
    }
    else
    {
        error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                      LIGHTS_FL_CLI_DISPLAY_DC_MASK,
                                      0);
    }
    
    return (error != 0);
}

/*!
  * @brief Update keypad light
  *        Function to handle keypad light update. 
  *
  * @param  pCtlData     Pointer to the control data for the region. 
  * @param  nColor       The color to set the region to. 
  *
  * @return     true - region updated
  *             false - region not updated
  */
 
static bool lights_fl_region_keypad
(
    LIGHTS_FL_LED_CTL_T *pCtlData, 
    LIGHTS_FL_COLOR_T nColor
)    
{
    int error;
    tracemsg(_k_d("=>Update the keypad display %d"), nColor);
    
    if(nColor)
    {
        error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                      LIGHTS_FL_KEYPAD_DC_MASK,
                                      LIGHTS_FL_KEYPAD_DC_MASK);
    }
    else
    {
        error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_2,
                                      LIGHTS_FL_KEYPAD_DC_MASK,
                                      0);
    }
    
    return (error != 0);
}

/*!
  * @brief Update tri color light
  *       Function to handle the enabling tri color leds which have the
  *       regions combined.
  * @param  pCtlData     Pointer to the control data for the region. 
  * @param  nColor       The color to set the region to. 
  *
  * @return     true - region updated
  *             false - region not updated
  */

static bool lights_fl_region_tri_color 
(
    LIGHTS_FL_LED_CTL_T *pCtlData, 
    LIGHTS_FL_COLOR_T nColor       
)
{
    unsigned int color_mask;
    int error;
    
    tracemsg(_k_d("=>Update the tricolor display %d"), nColor);
                 
    color_mask = (((nColor & LIGHTS_FL_COLOR_BLUE) >> LIGHTS_FL_COLOR_BLUE_SFT)
                  >> LIGHTS_FL_DUTY_CYCLE_SHIFT)  << LIGHTS_FL_TRI_COLOR_BLUE_DC_INDEX;
    color_mask |= (((nColor & LIGHTS_FL_COLOR_GREEN) >> LIGHTS_FL_COLOR_GREEN_SFT) 
                  >> LIGHTS_FL_DUTY_CYCLE_SHIFT) << LIGHTS_FL_TRI_COLOR_GREEN_DC_INDEX;
    color_mask |= (((nColor & LIGHTS_FL_COLOR_RED) >> LIGHTS_FL_COLOR_RED_SFT) 
                  >> LIGHTS_FL_DUTY_CYCLE_SHIFT) << LIGHTS_FL_TRI_COLOR_RED_DC_INDEX;
                                    
    if(pCtlData->nLed == 1)
    {
        error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_3, LIGHTS_FL_TRI_COLOR_DC_MASK, color_mask);
    }
    else
    {
        error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_4, LIGHTS_FL_TRI_COLOR_DC_MASK, color_mask);
        error |= power_ic_set_reg_mask(POWER_IC_REG_ATLAS_LED_CONTROL_5, LIGHTS_FL_TRI_COLOR_DC_MASK, color_mask);
    }
 
    return (error != 0);
}

/*!
  * @brief Update the sol led
  *       Returns the number of the highest priority app which has control the region requested.
  *
  * @param  pCtlData     Pointer to the control data for the region. 
  * @param  nColor       The color to set the region to.
  *
  * @return     The applycation priority
  */
static bool lights_fl_region_sol_led
(
    LIGHTS_FL_LED_CTL_T *pCtlData, 
    LIGHTS_FL_COLOR_T nColor       
)
{
    int error;
    tracemsg(_k_d("=>Update the sol led with color %d"), nColor);
    
    error = power_ic_set_reg_bit(POWER_IC_REG_ATLAS_CHARGER_0,LIGHTS_FL_CHRG_LED_EN,(nColor != 0));
        
    return (error != 0);
}
