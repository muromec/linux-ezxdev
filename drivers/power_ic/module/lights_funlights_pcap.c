/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/module/lights_funlights_pcap.c
 * 
 * Copyright (C) 2005 Motorola, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation.
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
 * 2005-Nov-16 - Design the PCAP specific interface for the funlight driver
 */


/*!
 * @file lights_funlights_pcap.c
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
#include "../core/fl_register.h"
#include "../core/os_independent.h"


/*******************************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************************/
static bool lights_fl_region_main_display(LIGHTS_FL_LED_CTL_T *pCtlData,LIGHTS_FL_COLOR_T nColor);
static bool lights_fl_region_cli_display(LIGHTS_FL_LED_CTL_T *pCtlData,LIGHTS_FL_COLOR_T nColor);
static bool lights_fl_region_keypad(LIGHTS_FL_LED_CTL_T *pCtlData,LIGHTS_FL_COLOR_T nColor);
static bool lights_fl_region_tri_color(LIGHTS_FL_LED_CTL_T *pCtlData,LIGHTS_FL_COLOR_T nColor);
static bool lights_fl_region_camera_flash(LIGHTS_FL_LED_CTL_T *pCtlData,LIGHTS_FL_COLOR_T nColor); 
static bool lights_fl_region_sol_led(LIGHTS_FL_LED_CTL_T *pCtlData,LIGHTS_FL_COLOR_T nColor); 

/*******************************************************************************************
 * LOCAL CONSTANTS
 ******************************************************************************************/
/*Following definition is the mask bits of fl registers for different region*/

/* PCAP2 Registers */
/*Register AUX_VREG */
#define LIGHTS_FL_PCAP_VAUX4_EN            12

/* EOC Registers */
/* Register Power Control 0 */
#define LIGHTS_FL_EOC_CHRG_LED_EN          18

/* LP3944 Regsiters */
/*Register LS1*/
#define LIGHTS_FL_MAIN_DISPLAY_INDEX       0
#define LIGHTS_FL_MAIN_DISPLAY_NUM_BITS    2
#define LIGHTS_FL_CAMERA_INDEX             4
#define LIGHTS_FL_CAMERA_NUM_BITS          4

/*Register LS0*/
#define LIGHTS_FL_TRI_COLOR_RED_INDEX      0
#define LIGHTS_FL_TRI_COLOR_RED_NUM_BITS   2
#define LIGHTS_FL_TRI_COLOR_GREEN_INDEX    2
#define LIGHTS_FL_TRI_COLOR_GREEN_NUM_BITS 2
#define LIGHTS_FL_TRI_COLOR_BLUE_INDEX     4
#define LIGHTS_FL_TRI_COLOR_BLUE_NUM_BITS  2
#define LIGHTS_FL_CLI_DISPLAY_INDEX        6
#define LIGHTS_FL_CLI_DISPLAY_NUM_BITS     2

#define DISPLAY_ON        0x00
#define DISPLAY_OFF       0x01
#define LED_ON            0x01
#define LED_OFF           0x00
#define CAMERA_FLASH_ON   0x01
#define CAMERA_TORCH_ON   0x05
#define CAMERA_OFF        0x00

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
    { lights_fl_region_camera_flash,{0,0, 0}}, /*Region 6*/
    { NULL,                         {0,0, 0}}, /*Region 7*/
    { NULL,                         {0,0, 0}}  /*Region 8*/
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

    /*Because of the hardware limitation on Barbados, it needs turn off the CLI display
      when the main display need on*/
    if(nColor)
    {
        /*For the main display and CLI display, 00-on,01-off*/
        error = power_ic_set_reg_value(POWER_IC_REG_FL_LS0,
                                       LIGHTS_FL_CLI_DISPLAY_INDEX,
                                       DISPLAY_OFF,
                                       LIGHTS_FL_CLI_DISPLAY_NUM_BITS); 
    
        error |= power_ic_set_reg_value(POWER_IC_REG_FL_LS1,
                                        LIGHTS_FL_MAIN_DISPLAY_INDEX,
                                        DISPLAY_ON,
                                        LIGHTS_FL_MAIN_DISPLAY_NUM_BITS);
    }
    else
    {
       error = power_ic_set_reg_value(POWER_IC_REG_FL_LS1,
                                      LIGHTS_FL_MAIN_DISPLAY_INDEX,
                                      DISPLAY_OFF,
                                      LIGHTS_FL_MAIN_DISPLAY_NUM_BITS);
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
        error = power_ic_set_reg_value(POWER_IC_REG_FL_LS1,
                                       LIGHTS_FL_MAIN_DISPLAY_INDEX,
                                       DISPLAY_OFF,
                                       LIGHTS_FL_MAIN_DISPLAY_NUM_BITS);
        error |= power_ic_set_reg_value(POWER_IC_REG_FL_LS0,
                                        LIGHTS_FL_CLI_DISPLAY_INDEX,
                                        DISPLAY_ON,
                                        LIGHTS_FL_CLI_DISPLAY_NUM_BITS);
    }
    else
    {
        error = power_ic_set_reg_value(POWER_IC_REG_FL_LS0,
                                       LIGHTS_FL_CLI_DISPLAY_INDEX,
                                       DISPLAY_OFF,
                                       LIGHTS_FL_CLI_DISPLAY_NUM_BITS);
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
    error = power_ic_set_reg_bit(POWER_IC_REG_PCAP_AUX_VREG, LIGHTS_FL_PCAP_VAUX4_EN,(nColor != LIGHTS_FL_COLOR_BLACK));
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
    unsigned char val = 0;
    int error;
    
    tracemsg(_k_d("=>Update the tricolor display %d"), nColor);
                 
    if((nColor & LIGHTS_FL_COLOR_BLUE)!=0)
    {
        val |= (LED_ON << LIGHTS_FL_TRI_COLOR_BLUE_INDEX);
    }
    if((nColor & LIGHTS_FL_COLOR_GREEN)!=0)
    {
        val |= (LED_ON << LIGHTS_FL_TRI_COLOR_GREEN_INDEX);
    }
    if((nColor & LIGHTS_FL_COLOR_RED)!=0)
    {
        val |= (LED_ON << LIGHTS_FL_TRI_COLOR_RED_INDEX);
    } 

    error = power_ic_set_reg_value(POWER_IC_REG_FL_LS0,
                                   LIGHTS_FL_TRI_COLOR_RED_INDEX,
                                   val,
                                   LIGHTS_FL_TRI_COLOR_BLUE_NUM_BITS+
                                   LIGHTS_FL_TRI_COLOR_GREEN_NUM_BITS+
                                   LIGHTS_FL_TRI_COLOR_RED_NUM_BITS);
    return (error != 0);
}


/*!
  * @brief Update the camera flash
  *       Returns the number of the highest priority app which has control the region requested.
  *
  * @param  pCtlData     Pointer to the control data for the region. 
  * @param  nColor       The color to set the region to.
  *
  * @return     The applycation priority
  */
static bool lights_fl_region_camera_flash
(
    LIGHTS_FL_LED_CTL_T *pCtlData, 
    LIGHTS_FL_COLOR_T nColor       
)
{
    int error;
    tracemsg(_k_d("=>Update the camera flash with color %d"), nColor);

    if(nColor == LIGHTS_FL_CAMERA_FLASH )
    {
        /*Turn on the camera flash*/
        error = power_ic_set_reg_value(POWER_IC_REG_FL_LS1,
                                       LIGHTS_FL_CAMERA_INDEX,
                                       CAMERA_FLASH_ON,
                                       LIGHTS_FL_CAMERA_NUM_BITS);
    }
    else if (nColor == LIGHTS_FL_CAMERA_TORCH)
    {
        /*Turn on the camera torch mode*/
        error = power_ic_set_reg_value(POWER_IC_REG_FL_LS1,
                                       LIGHTS_FL_CAMERA_INDEX,
                                       CAMERA_TORCH_ON,
                                       LIGHTS_FL_CAMERA_NUM_BITS);
    }
    else 
    {
        /*Turn off the camera flash*/
        error = power_ic_set_reg_value(POWER_IC_REG_FL_LS1,
                                       LIGHTS_FL_CAMERA_INDEX,
                                       CAMERA_OFF,
                                       LIGHTS_FL_CAMERA_NUM_BITS);
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
    error = power_ic_set_reg_bit(POWER_IC_REG_EOC_POWER_CONTROL_0,LIGHTS_FL_EOC_CHRG_LED_EN,(nColor != LIGHTS_FL_COLOR_BLACK));

    return (error != 0);
}

