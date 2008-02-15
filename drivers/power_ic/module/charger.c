/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/module/charger.c
 *
 * Description - Power IC charging control interface.
 *
 * Copyright (C) 2005 - Motorola
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
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
 * Motorola 2005-Feb-28 - File Creation
 * Motorola 2005-Feb-28 - Add power path control and OV protection
 * Motorola 2005-Dec-13 - Finalized the software
 *
 */
 
 /*!
 * @file charger.c
 *
 * @ingroup poweric_charger
 * 
 * @brief Power IC charging control interface.
 *
 * This module implements the interface for controlling the charging hardware.
 * This provides basic control over charging voltage and current and the 
 * configuration of the power path. This module does not include any charging
 * algorithm.
 * 
 * @todo Need to figure out how to use overvoltage hardware for EMU one-chip. 
 * power_ic_charger_get_overvoltage() and power_ic_charger_reset_overvoltage()
 * are provided as an equivalent for these controls in AUL, but they don't seem
 * to do anything - sense bit doesn't go high when charger overvoltage occurs
 * (though EMU does shut off the charger) and charging resumes autonomously
 * without resetting the hardware.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/delay.h>
#include <asm/errno.h>
#include <linux/power_ic.h>
#include <asm/uaccess.h>

#include "../core/os_independent.h"

#include "charger.h"

/******************************************************************************
* Constants
******************************************************************************/ 
#ifndef DOXYGEN_SHOULD_SKIP_THIS

/* Bits in EMU one-chip's power control register 0. */
#define EMU_VCHRG_MASK              0x00000007
#define EMU_ICHRG_MASK              0x00000078
#define EMU_ICHRG_SHIFT             3
#define EMU_ICHRG_TR_MASK           0x00000380
#define EMU_ICHRG_TR_SHIFT          7
#define EMU_FET_OVRD_MASK           0x00000400
#define EMU_FET_CTRL_MASK           0x00000800

#endif

/*! Below 3.0V, the initial power path will be set to dual-path mode if a charger
 *  is attached). Otherwise, current-share mode will be selected. */
#define POWER_PATH_BATT_THRESHOLD   0x01A4

/* The safest way to detect a charger early in powerup is to look at its output.
 * Anything above 4.0V on the charger line will be treated as a charger attached. */
#define POWER_PATH_CHRG_THRESHOLD   0x0074

/* The power control register will be set up like this depending on battery voltage. 
 * The curr share init value is the one originally specified by the hardware team. */
#define POWER_PATH_INIT_CURR_SHARE  0x0000C01
#define POWER_PATH_INIT_DUAL_PATH   0x0000801

#ifdef CONFIG_MOT_POWER_IC_PCAP2
#define VBUS_OVERVOLTAGE_EVENT      POWER_IC_EVENT_EOC_VBUSOV
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
#define VBUS_OVERVOLTAGE_EVENT      POWER_IC_EVENT_ATLAS_CHGOVI
#endif

/******************************************************************************
* Local Variables
******************************************************************************/

/******************************************************************************
* Internal Global driver functions.
*******************************************************************************/

/*!
 * @brief initialises the charger hardware.
 *
 * This function is used to set the initial power path. if the battery voltage 
 * is too low to support the phone, the path is set to dual-path to allow the 
 * charger to supply the phone while slowly charging up the battery. Otherwise,
 * current-sharing is selected as the battery can make up any shortfall.
 *
 * @bug Ideally, this function should be passed the power-up reason so that 
 * the path is also set according to whether a charger is present or not. 
 * However, a lot of powerups are currently detected as power cuts, which 
 * have a higher priority than the charger, so this cannot be added yet.
 *
 * @todo Requirements issue: does driver need to set power path based on charger
 * and battery voltage at powerup, or can it wait until SBCM starts a few seconds later?
 * Init code commented out for now.
 */
void charger_init(void)
{
    int batt;
    int charger;

    /* Get readings of the battery and charger. */
    /*
    if((power_ic_atod_single_channel(POWER_IC_ATOD_CHANNEL_BATT, &batt) == 0) &&
       (power_ic_atod_single_channel(POWER_IC_ATOD_CHANNEL_MOBPORTB, &charger) == 0))
    {
        if((charger > POWER_PATH_CHRG_THRESHOLD) && (batt < POWER_PATH_BATT_THRESHOLD))
        {
            power_ic_write_reg_value(POWER_IC_REG_EOC_POWER_CONTROL_0, POWER_PATH_INIT_DUAL_PATH);
        }
        else
        {
            power_ic_write_reg_value(POWER_IC_REG_EOC_POWER_CONTROL_0, POWER_PATH_INIT_CURR_SHARE);
        }
    }*/
}

/*!
 * @brief ioctl() handler for the charger control interface.
 *
 * This function is the ioctl() interface handler for all charger control operations. It is 
 * not called directly through an ioctl() call on the power IC device, but is executed 
 * from the core ioctl handler for all ioctl requests in the range for charger operations.
 *
 * @param        cmd       ioctl() request received.
 * @param        arg       additional information about request, specific to each request.
 *
 * @return 0 if successful.
 */
int charger_ioctl(unsigned int cmd, unsigned long arg)
{
    int error;
    int temp;     
    tracemsg(_k_d("Charger control ioctl(), request 0x%X (cmd 0x%X)"),(int) cmd, _IOC_NR(cmd));
    
    /* Handle the request. Note that no charger control operations require a structure to be
     * passed - all operations take a single parameter passed by value. */
    switch(cmd)
    {
        case POWER_IC_IOCTL_CHARGER_SET_CHARGE_VOLTAGE:
            return(power_ic_charger_set_charge_voltage((int)arg));
            break;
            

        case POWER_IC_IOCTL_CHARGER_SET_CHARGE_CURRENT:
            return(power_ic_charger_set_charge_current((int)arg));
            break;
            

        case POWER_IC_IOCTL_CHARGER_SET_TRICKLE_CURRENT:
            return(power_ic_charger_set_trickle_current((int)arg));
            break;
            

        case POWER_IC_IOCTL_CHARGER_SET_POWER_PATH:
            return(power_ic_charger_set_power_path((int)arg));
            break;

        case POWER_IC_IOCTL_CHARGER_GET_OVERVOLTAGE:
            /* Read the state of overvoltage from the hardware. */
            if((error = power_ic_charger_get_overvoltage(&temp)))
            {
                return error;
            }
            
            /* Return the read state back to the caller. */
            if(put_user(temp, (int *)arg))
            {
                return -EFAULT;
            }
            break;
            
        case POWER_IC_IOCTL_CHARGER_RESET_OVERVOLTAGE:
            return(power_ic_charger_reset_overvoltage());
            break;

        default: /* This shouldn't be able to happen, but just in case... */
            tracemsg(_k_d("=> 0x%X unsupported charger ioctl command"), (int) cmd);
            return -ENOTTY;
            break;
    
    }
    return 0;
}

 /******************************************************************************
* Global Functions
*******************************************************************************/

/*!
 * @brief Sets the charge voltage.
 *
 * This function sets the maximum voltage that the battery will charged up to.
 *
 * @param        charge_voltage       Maximum charge voltage to be set.
 *
 * @return returns 0 if successful.
 */
int power_ic_charger_set_charge_voltage(int charge_voltage)
{   
    tracemsg(_k_d("Charger: setting VCHRG 0x%X"), charge_voltage);
    return(power_ic_set_reg_mask(POWER_IC_REG_EOC_POWER_CONTROL_0, EMU_VCHRG_MASK, charge_voltage));
}

/*!
 * @brief Sets the charge current.
 *
 * This function sets the maximum current allowed for charging the battery.
 *
 * @param        charge_current       Main charge current to be set.
 *
 * @note When called, this function will also set the trickle charge current to
 * zero. This ensures than the main charge and trickle charge aren't on at the 
 * same time.
 *
 * @note The hardware has the final say in how much current is pushed into the
 * battery. The hardware will respect the limit set, but the actual charge 
 * current may be less than the limit.
 *
 * @return returns 0 if successful.
 */
int power_ic_charger_set_charge_current(int charge_current)
{
    int mask;


    int setup = (charge_current << EMU_ICHRG_SHIFT) & EMU_ICHRG_MASK;
    /* If the ICHRG reg is being set to a non-zero value, then we should 
       clear the trickle charge bits. If it is a zero value, then leave the
       TRKLCHRG reg as is.
    */
    if (charge_current !=0)
    {
        mask = EMU_ICHRG_MASK | EMU_ICHRG_TR_MASK;
    }
    else
    {
        mask = EMU_ICHRG_MASK;
    }

    return(power_ic_set_reg_mask(POWER_IC_REG_EOC_POWER_CONTROL_0, mask, setup));
}

/*!
 * @brief Sets the trickle charge current.
 *
 * This function sets the maximum current allowed for trickle charging.
 *
 * @param        charge_current       Trickle charge current to be set.
 *
 * @pre The power path must be set to dual-path prior to enabling the trickle
 * charger.
 *
 * @note When called, this function will also set the main charge current to
 * zero. This ensures than the main charge and trickle charge aren't on at the 
 * same time.
 *
 * @note The hardware has the final say in how much current is pushed into the
 * battery. The hardware will respect the limit set, but the actual charge 
 * current may be less than the limit.
 *
 * @return returns 0 if successful.
 */
int power_ic_charger_set_trickle_current(int charge_current)
{
    int mask;

    int setup = (charge_current << EMU_ICHRG_TR_SHIFT) & EMU_ICHRG_TR_MASK;

    /* If theTRKLCHRG reg is being set to a non-zero value, then we should 
       clear the main current regulator. If it is a zero value, then leave the
       ICHRG reg as is.
    */
    if (charge_current !=0)
    {
        mask = EMU_ICHRG_MASK | EMU_ICHRG_TR_MASK;
    }
    else
    {
        mask = EMU_ICHRG_TR_MASK;
    }

    return(power_ic_set_reg_mask(POWER_IC_REG_EOC_POWER_CONTROL_0, mask, setup));
}

/*!
 * @brief Sets the power path.
 *
 * This function configures the path used to supply current from the charger. In
 * dual-path mode, the charger is connected to the phone's supply and can operate
 * the phone without a battery so long as the current drawn isn't too high. In 
 * current-share mode, a battery must be present for the phone to operate.
 *
 * @param        path       New setting for power path.
 *
 * @pre The power path must be set to dual-path prior to enabling the trickle
 * charger.
 *
 * @return returns 0 if successful.
 */
int power_ic_charger_set_power_path(POWER_IC_CHARGER_POWER_PATH_T path)
{
    /* Both FET override and FET control are used to control path. */
    int mask = EMU_FET_OVRD_MASK | EMU_FET_CTRL_MASK;
    int setup;
    
    switch(path)
    {
        /* Hardware-controlled dual-path mode. */
        case POWER_IC_CHARGER_POWER_DUAL_PATH:
            setup = 0;
            break;
        
        /* Hardware-controlled current-share mode. */    
        case POWER_IC_CHARGER_POWER_CURRENT_SHARE:
            setup = EMU_FET_CTRL_MASK;
            break;
                        
        /* Software-overridden dual-path mode. */    
        case POWER_IC_CHARGER_POWER_DUAL_PATH_SW_OVERRIDE:
            setup = EMU_FET_OVRD_MASK;
            break;   
                 
        /* Software-overridden current-share mode. */
        case POWER_IC_CHARGER_POWER_CURRENT_SHARE_SW_OVERRIDE:
            setup = EMU_FET_OVRD_MASK | EMU_FET_CTRL_MASK;
            break;
            
        default:
            tracemsg(_k_d("   power path %d is invalid."), path);
            return -EINVAL;
    }
    
    tracemsg(_k_a("########################## Charger: setting power path 0x%X (masked with 0x%X)"), setup, mask);
    
    return(power_ic_set_reg_mask(POWER_IC_REG_EOC_POWER_CONTROL_0, mask, setup));
}


/*!
 * @brief Reads overvoltage state.
 *
 * This function reads the charger overvoltage sense bit from the hardware and returns it
 * to the caller.
 *
 * @param        overvoltage       For a successful read, set to zero if no overvoltage detected,
 *                                 greater than 0 if an overvoltage condition has been seen.
 *
 * @return returns 0 if successful.
 *
 * @note At this time, this doesn't seem to do much. The sense bit is not set when a charger
 * overvoltage condition occurs, even when EMU one-chip has already detected a problem and
 * cut the charge current.
 */
int power_ic_charger_get_overvoltage(int * overvoltage)
{
    int error = 0;
    
    *overvoltage = power_ic_event_sense_read(VBUS_OVERVOLTAGE_EVENT);
    
    if(*overvoltage < 0)
    {
        error = *overvoltage;
    }
    
    return error;
}


/*!
 * @brief Resets the overvoltage hardware.
 *
 * This function is used to reset the overvoltage hardware after a problem is detected.
 * Once an overvoltage condition occurs, charging will be disabled in hardware until it is
 * reset.
 *
 * @return returns 0 if successful.
 *
 * @note At this time, this doesn't seem to do a whole lot. EMU one-chip appears to reset
 * itself after an overvoltage condition occurs on the charger.
 */
int power_ic_charger_reset_overvoltage(void)
{
    return(power_ic_event_clear(VBUS_OVERVOLTAGE_EVENT));
}
