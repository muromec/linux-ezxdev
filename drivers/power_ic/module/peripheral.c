/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/module/peripheral.c
 * 
 * Description - Main file of the power IC peripheral interface.
 *
 * Copyright (C) 2004-2005 Motorola, Inc. All Rights Reserved.
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
 * 2004-Dec-06 - Design of the power IC peripheral interface.
 * 
 * 2005-Mar-01 - Add WLAN support
 *
 * 2005-Apr-28 - Provided support to change SIM voltage values
 *
 * 2005-Sep-13 - Added Camera Support
 */

/*!
 * @file peripheral.c
 *
 * @brief This is the main file of the power IC peripheral interface.
 *
 * @ingroup poweric_periph
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/errno.h>

#include <linux/power_ic.h>

#include "../core/os_independent.h"

/*******************************************************************************************
 * CONSTANTS
 ******************************************************************************************/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
/* Bluetooth is controlled by VAUX1 in the AUX_VREG register this is the old configuration. */
#ifdef CONFIG_MOT_POWER_IC_BLUETOOTH_SPLY_VAUX1
#define BLUETOOTH_ONOFF_REG       POWER_IC_REG_PCAP_AUX_VREG
#define BLUETOOTH_ONOFF_INDEX     1
#define BLUETOOTH_ONOFF_NUM_BITS  1
/* Bluetooth is controlled by V6 in the VREG2 register this is the new configuration. */
#else 
#define BLUETOOTH_ONOFF_REG       POWER_IC_REG_PCAP_VREG2
#define BLUETOOTH_ONOFF_INDEX     1
#define BLUETOOTH_ONOFF_NUM_BITS  1
#endif

/* To support the Flash card which is controlled the AUX_VREG register. */
#define FLASH_CARD_ONOFF_REG      POWER_IC_REG_PCAP_AUX_VREG
/* When cards are to be used and the supply needs to support the higher current use VAUX2 */
#ifdef CONFIG_MOT_POWER_IC_MEM_CARD_SPLY_SD
#define FLASH_CARD_ONOFF_INDEX    4
#define FLASH_CARD_ONOFF_NUM_BITS 1
/* This is the default _TRANSFLASH setting which corresponds to VAUX3 */ 
#else
#define FLASH_CARD_ONOFF_INDEX    7
#define FLASH_CARD_ONOFF_NUM_BITS 1
#endif

/* The vibrator is turned on and off by V_VIB_EN in the AUX_VREG register. */
#ifdef CONFIG_MOT_POWER_IC_PCAP2
#define VIBRATOR_ONOFF_REG        POWER_IC_REG_PCAP_AUX_VREG
#define VIBRATOR_ONOFF_INDEX      19
#define VIBRATOR_ONOFF_NUM_BITS   1
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
#define VIBRATOR_ONOFF_REG        POWER_IC_REG_ATLAS_REG_MODE_1
#define VIBRATOR_ONOFF_INDEX      11
#define VIBRATOR_ONOFF_NUM_BITS   1
#endif

/* The vibrator level is controlled by V_VIB_[0..1] in the AUX_VREG register. */
#ifdef CONFIG_MOT_POWER_IC_PCAP2
#define VIBRATOR_LEVEL_REG        POWER_IC_REG_PCAP_AUX_VREG
#define VIBRATOR_LEVEL_INDEX      20
#define VIBRATOR_LEVEL_NUM_BITS   2
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
#define VIBRATOR_LEVEL_REG        POWER_IC_REG_ATLAS_REG_SET_1
#define VIBRATOR_LEVEL_INDEX      0
#define VIBRATOR_LEVEL_NUM_BITS   2
#endif

/* USB Cable Attached */
#ifdef CONFIG_MOT_POWER_IC_PCAP2
#define USB_CABLE_ATTACH          POWER_IC_EVENT_EOC_VBUS_3V4
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
#define USB_CABLE_ATTACH          POWER_IC_EVENT_ATLAS_USBI
#endif

/* The USB pull-up is controlled by DP_1K5_PU in the Connectivity Control register of EOC */
#ifdef CONFIG_MOT_POWER_IC_PCAP2
#define USB_PU_REG                POWER_IC_REG_EOC_CONN_CONTROL
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
#define USB_PU_REG                POWER_IC_REG_ATLAS_USB_0
#endif
#define USB_PU_INDEX              2
#define USB_PU_NUM_BITS           1

/* WLAN requires two supplies one for the logic section and one for the RF section. 
   This is the old configuration where the LOGIC supply is controlled by V6 in the VREG2 register. */
#ifdef CONFIG_MOT_POWER_IC_BLUETOOTH_SPLY_VAUX1   
#define WLAN_LOGIC_ONOFF_REG         POWER_IC_REG_PCAP_VREG2
#define WLAN_LOGIC_ONOFF_INDEX       1
#define WLAN_LOGIC_ONOFF_NUM_BITS    1
#else
/* This is the new configuration where the LOGIC supply is controlled by VAUX1 in the AUX_VREG register */
#define WLAN_LOGIC_ONOFF_REG         POWER_IC_REG_PCAP_AUX_VREG
#define WLAN_LOGIC_ONOFF_INDEX       1
#define WLAN_LOGIC_ONOFF_NUM_BITS    1
#endif
/* Currently the supply for the RF section is VAUX2 which is in the AUX_VREG register */
#define WLAN_RF_ONOFF_REG      POWER_IC_REG_PCAP_AUX_VREG
#define WLAN_RF_ONOFF_INDEX    4
#define WLAN_RF_ONOFF_NUM_BITS 1


/* SIM voltage is controlled by VSIM in the Regulator Setting 0 on SCM-A11 */
#define SIM_REG_SET_0_REG         POWER_IC_REG_ATLAS_REG_SET_0
#define SIM_REG_SET_0_VSIM_INDEX  14
#define SIM_REG_SET_0_VESIM_INDEX 15
#define SIM_REG_SET_0_NUM_BITS    1

/* The camera bits in order to turn on and off. Only used for Atlas */
#define CAMERA_ONOFF_REG        POWER_IC_REG_ATLAS_REG_MODE_1
#define CAMERA_ONOFF_INDEX      6
#define CAMERA_ONOFF_NUM_BITS   1

#endif

/*******************************************************************************************
 * TYPES
 ******************************************************************************************/
 
 
/*******************************************************************************************
 * GLOBAL FUNCTIONS
 ******************************************************************************************/
 
 /*!
 * This function is called by power IC clients to turn Bluetooth's power supply on or off.
 *
 * @param        on         Turns Bluetooth on or off (POWER_IC_PERIPH_ON or POWER_IC_PERIPH_OFF)
 *
 * @return       This function returns 0 if successful.
 */
int power_ic_periph_set_bluetooth_on(int on)
{
    /* Any input above zero is on.. */
    if(on > 0)
    {
        on = 1;
    }
    else
    {
        on = 0;
    }
    
    return power_ic_set_reg_value(BLUETOOTH_ONOFF_REG, BLUETOOTH_ONOFF_INDEX, on,
                                  BLUETOOTH_ONOFF_NUM_BITS);
}

/*!
 * This function is called by power IC clients to turn the flash card power supply on or off.
 *
 * @param        on         Turns the flash card on or off (POWER_IC_PERIPH_ON or POWER_IC_PERIPH_OFF)
 *
 * @return       This function returns 0 if successful.
 */
int power_ic_periph_set_flash_card_on(int on)
{
    
    /* Any input above zero is on.. */
    if( on > 0)
    {
        on = 1;
    }
    else
    {
        on = 0;
    }
        
    return power_ic_set_reg_value(FLASH_CARD_ONOFF_REG, FLASH_CARD_ONOFF_INDEX, on,
                                    FLASH_CARD_ONOFF_NUM_BITS);
  
}


/*!
 * This function is called by power IC clients to set the vibrator supply level.
 *
 * @param        level      The level that the vibrator should be set to, from 0 (low) to 3 (high).
 *
 * @return       This function returns 0 if successful.
 */
int power_ic_periph_set_vibrator_level(int level)
{
    /* Don't allow any attempts to set any other value than on or off. */
    if((level < 0) || (level >= (1 << VIBRATOR_LEVEL_NUM_BITS) ))
    {
        return -EINVAL;
    }
    
    return power_ic_set_reg_value(VIBRATOR_LEVEL_REG, VIBRATOR_LEVEL_INDEX, level,
                                  VIBRATOR_LEVEL_NUM_BITS);
}

/*!
 * This function is called by power IC clients turn the vibrator on or off.
 *
 * @param        on         Turns the vibrator on or off (POWER_IC_PERIPH_ON or POWER_IC_PERIPH_OFF)
 *
 * @return       This function returns 0 if successful.
 */
int power_ic_periph_set_vibrator_on(int on)
{
    /* Any input above zero is on.. */
    if(on > 0)
    {
        on = 1;
    }
    else
    {
        on = 0;
    }
    
    return power_ic_set_reg_value(VIBRATOR_ONOFF_REG, VIBRATOR_ONOFF_INDEX, on,
                                  VIBRATOR_ONOFF_NUM_BITS);
}

/*!
 * @brief Returns 1 if USB cable is currently connected
 *
 * This function determines whether a USB cable is connected or not.  Currently, this
 * is a very simple check that looks at the USB 3.4v sense bit in the EMU One Chip.
 * At some point, this function should not be required and the hotplug interface with
 * the USB driver will be the only indication of when a cable is inserted/removed.
 *
 * @return 1 if USB cable is connected, 0 if not connected
 */

int power_ic_periph_is_usb_cable_connected (void)
{
    return power_ic_event_sense_read (USB_CABLE_ATTACH);
}

/*!
 * @brief Returns 1 if the USB pull-up is enabled
 *
 * This function checks the EMU One Chip Connectivity Control register to determine
 * if the USB pull-up has been enabled (bit 2, DP_1K5_PU).  The function returns the
 * state of the bit (1 = enabled, 0 = not enabled).
 *
 * @return 1 if USB pull-up enabled, 0 if not
 */

int power_ic_periph_is_usb_pull_up_enabled (void)
{
    int value;

    /* Read the bit from the EMU One Chip */
    if (power_ic_get_reg_value (USB_PU_REG, USB_PU_INDEX, &value, USB_PU_NUM_BITS) != 0)
    {
        value = 0;
    }

    return value;
}

 /*!
 * This function is called by power IC clients to turn the two WLAN's power supplies LOGIC and RF on or off.
 *
 * @param        on         Turns WLAN on or off (POWER_IC_PERIPH_ON or POWER_IC_PERIPH_OFF)
 *
 * @return       This function returns 0 if successful.
 */
int power_ic_periph_set_wlan_on(int on)
{
    
    int val;
    
    /* Any input above zero is on.. */
    /* It is required that the LOGIC be powered-up before RF section when turning ON and shut down 
    after the RF section when shutting off */     
    if(on > 0)
    {
    
        val = power_ic_set_reg_value(WLAN_LOGIC_ONOFF_REG, WLAN_LOGIC_ONOFF_INDEX, 1,
                                 WLAN_LOGIC_ONOFF_NUM_BITS);

        val |= power_ic_set_reg_value(WLAN_RF_ONOFF_REG, WLAN_RF_ONOFF_INDEX, 1,
                                     WLAN_RF_ONOFF_NUM_BITS);
    }
    else
    {
        val = power_ic_set_reg_value(WLAN_RF_ONOFF_REG, WLAN_RF_ONOFF_INDEX, 0,
                                     WLAN_RF_ONOFF_NUM_BITS);

        val |= power_ic_set_reg_value(WLAN_LOGIC_ONOFF_REG, WLAN_LOGIC_ONOFF_INDEX, 0,
                                 WLAN_LOGIC_ONOFF_NUM_BITS);
    }
    
     
    return val;
    
}

 /*!
 * This function is called by power IC clients to when the WLAN is going to be placed into a 
 * low power state the LOGIC section is turned on, in case it is off, and the RF is turned off.
 *
 * @return       This function returns 0 if successful.
 */
int power_ic_periph_set_wlan_low_power_state_on(void)
{
    int val;

    val = power_ic_set_reg_value(WLAN_LOGIC_ONOFF_REG, WLAN_LOGIC_ONOFF_INDEX, 1,
                                 WLAN_LOGIC_ONOFF_NUM_BITS);
    
    val |= power_ic_set_reg_value(WLAN_RF_ONOFF_REG, WLAN_RF_ONOFF_INDEX, 0,
                                     WLAN_RF_ONOFF_NUM_BITS);
    
    return val;
    
}

/*!
 * This function is called by power IC clients to change the SIM voltage values.
 *
 * @param        sim_card_num   Used later for different types of SIM
 * @param        volt           Changes SIM voltage to 1.8 or 3.0 volts
 *
 * @return       This function returns 0 if successful.
 *
 * @note         If volt is 0, then SIM voltage will be 1.8 volts.  If volt is 1, then SIM voltage 
 *               is 3.0 volts
 */
int power_ic_periph_set_sim_voltage(unsigned char sim_card_num, POWER_IC_SIM_VOLTAGE_T volt)
{
    /* If 0, then set VSIM, otherwise set VESIM */
    if(sim_card_num == 0)
    {
        return (power_ic_set_reg_value(SIM_REG_SET_0_REG, SIM_REG_SET_0_VSIM_INDEX, volt,
                                 SIM_REG_SET_0_NUM_BITS));
    }
    else
    {  
        return (power_ic_set_reg_value(SIM_REG_SET_0_REG, SIM_REG_SET_0_VESIM_INDEX, volt,
                                 SIM_REG_SET_0_NUM_BITS));
    }      
}

/*!
 * @brief Set the state of the USB pull-up
 *
 * This function sets the state of the USB pull-up in the EMU One Chip Connectivity
 * Control register.
 *
 * @param on  0 to turn the pull-up off, anything else turns it on
 */

void power_ic_periph_set_usb_pull_up (int on)
{
    /* Set the USB pull-up bit in the EMU One Chip register */
    (void)power_ic_set_reg_bit (USB_PU_REG, USB_PU_INDEX, (on) ? 1 : 0);
}

/*!
 * This function is called by power IC clients turn the camera on or off.
 *
 * @param        on         Turns the camera on or off (POWER_IC_PERIPH_ON or POWER_IC_PERIPH_OFF)
 *
 * @return       This function returns 0 if successful.
 *
 * @note         This function is only used for Atlas.  PCAP camera is powered off of AP_IO_REG which is always on.
 */
int power_ic_periph_set_camera_on(int on)
{
    /* Any input above zero is on.. */
    if(on > 0)
    {
        on = 1;
    }
    else
    {
        on = 0;
    }
    
    return power_ic_set_reg_value(CAMERA_ONOFF_REG, CAMERA_ONOFF_INDEX, on,
                                  CAMERA_ONOFF_NUM_BITS);
}

/*!
 * This function is the ioctl() interface handler for all peripheral operations. It is not called
 * directly through an ioctl() call on the power IC device, but is executed from the core ioctl
 * handler for all ioctl requests in the range for peripherals.
 *
 * @param        cmd         the ioctl() command
 * @param        arg         the ioctl() argument
 *
 * @return       This function returns 0 if successful.
 */
int periph_ioctl(unsigned int cmd, unsigned long arg)
{
    int retval = 0;
    int data = (int) arg;
    
    /* Get the actual command from the ioctl request. */
    unsigned int cmd_num = _IOC_NR(cmd);

    tracemsg(_k_d("peripheral ioctl(), request 0x%X (cmd %d)"),(int) cmd, (int)cmd_num);
    
    /* Handle the request. */
    switch(cmd)
    {
        case POWER_IC_IOCTL_PERIPH_SET_BLUETOOTH_ON:
            retval = power_ic_periph_set_bluetooth_on(data);
            break;

        case POWER_IC_IOCTL_PERIPH_SET_FLASH_CARD_ON:
            retval = power_ic_periph_set_flash_card_on(data);
            break;
                
        case POWER_IC_IOCTL_PERIPH_SET_VIBRATOR_LEVEL:
            retval = power_ic_periph_set_vibrator_level(data);
            break;

        case POWER_IC_IOCTL_PERIPH_SET_VIBRATOR_ON:
            retval = power_ic_periph_set_vibrator_on(data);
            break;

        case POWER_IC_IOCTL_PERIPH_SET_WLAN_ON:
            retval = power_ic_periph_set_wlan_on(data);
            break;
	    
        case POWER_IC_IOCTL_PERIPH_SET_WLAN_LOW_POWER_STATE_ON:
            retval = power_ic_periph_set_wlan_low_power_state_on();
            break;
            
        case POWER_IC_IOCTL_PERIPH_SET_CAMERA_ON:
            retval = power_ic_periph_set_camera_on(data);
            break;

        default: /* This shouldn't be able to happen, but just in case... */
            tracemsg(_k_d("=> 0x%X unsupported peripheral ioctl command"), (int) cmd);
            retval = -EINVAL;
            break;
    }

    return retval;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
EXPORT_SYMBOL(power_ic_periph_set_bluetooth_on);
EXPORT_SYMBOL(power_ic_periph_set_flash_card_on);
EXPORT_SYMBOL(power_ic_periph_set_vibrator_level);
EXPORT_SYMBOL(power_ic_periph_set_vibrator_on);
EXPORT_SYMBOL(power_ic_periph_is_usb_cable_connected);
EXPORT_SYMBOL(power_ic_periph_is_usb_pull_up_enabled);
EXPORT_SYMBOL(power_ic_periph_set_usb_pull_up);
EXPORT_SYMBOL(power_ic_periph_set_wlan_on);
EXPORT_SYMBOL(power_ic_periph_set_wlan_low_power_state_on);
EXPORT_SYMBOL(power_ic_periph_set_sim_voltage);
EXPORT_SYMBOL(power_ic_periph_set_camera_on);
#endif
