/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/module/power_management.c
 *
 * Description - This module provides interfaces to control various power-related
 * functions, (e.g. changing the processor core voltage).
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
 * 2004-Dec-17 - Design of interfaces to control power-related functions.
 */

/*!
 * @file power_management.c
 *
 * @ingroup poweric_power_management
 *
 * @brief This module provides interfaces to control various power-related
 * functions, (e.g. changing the processor core voltage).
 */

#include <linux/kernel.h>
#include <linux/power_ic.h>
#include <asm/delay.h>
#include <linux/errno.h>
#include "power_management.h"
#include "../core/os_independent.h"

/*******************************************************************************************
 * CONSTANTS
 ******************************************************************************************/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
/* The Adjunct's core voltage is using PCAP2 controlled by SW1 in the SWCTRL register. */
#define POWER_MANAGEMENT_REG      	POWER_IC_REG_PCAP_SWCTRL
#define POWER_MANAGEMENT_INDEX    	2
#define POWER_MANAGEMENT_NUM_BITS 	4
#endif

/*! The number of settings available for the regulator supplying the core. */
#define PCAP_SW1_ARRAY_SIZE	  	16

/*! Bulverde's core is rated to this maxumum voltage (in mV). */
#define CORE_MAXIMUM_RATING       	1705

/*! The software must delay for this duration after changing the core voltage (in microsecs). */
#define PCAP_SW1_STABILIZING_DELAY	150

/*******************************************************************************************
 * TYPES
 ******************************************************************************************/
/*! When using PCAP2 the core voltage will be controlled by SW1. The highest value in this 
 * array must be located in the last position so the function can correctly verify that the
 * requested voltage is not larger then the part can supply  */
static const unsigned int PCAP_SW1_SETTINGS[PCAP_SW1_ARRAY_SIZE] = 
{ 
    900, 950, 1000, 1050,1100, 1150, 1200, 1250, 1300, 1350, 1400, 1450, 1500, 1600, 1875, 2250, 4400
    
};
 
/*******************************************************************************************
 * GLOBAL FUNCTIONS
 ******************************************************************************************/
 
 /*!
 * This function is called by power IC clients too adjust the adjunct's core voltage.
 *
 * @param        millivolts     The supply to the core will be set to this or the closest value above it   
 *
 * @return       This function returns 0 if successful.
 */
 int power_ic_set_core_voltage (int millivolts)
{
    int value = 0;
    int val;
      
    /* Do not allow voltage too be programed that exceeds the cores maximum voltage rating or are larger then the table can supply. */
    if((millivolts >= CORE_MAXIMUM_RATING) || (millivolts > PCAP_SW1_SETTINGS[PCAP_SW1_ARRAY_SIZE-1] ))
    {
        return -EINVAL;
    }

    while(millivolts > PCAP_SW1_SETTINGS[value])
    {
	value++;
    }

    if(PCAP_SW1_SETTINGS[value] >= CORE_MAXIMUM_RATING)
    {
        return -EINVAL; 
    }	
		
    /* Write determined voltage setting to power ic*/
    val = power_ic_set_reg_value(POWER_MANAGEMENT_REG,POWER_MANAGEMENT_INDEX,value,POWER_MANAGEMENT_NUM_BITS);

    /* After adjusting the output level of the power management IC the function will wait for the supply to
    stabilze before reporting that the adjustment has been made */  
    
    udelay (PCAP_SW1_STABILIZING_DELAY);
    
    return val;  
      
}

/*!
 * This function is the ioctl() interface handler for all power management operations. It is not called
 * directly through an ioctl() call on the power IC device, but is executed from the core ioctl
 * handler for all ioctl requests in the range for power management.
 *
 * @param        cmd    The command being called 
 * @param        arg    This sets the target voltage that the PCAP will be provide to the core 
 * 
 * @return       This function returns 0 if successful.
 */
int pmm_ioctl(unsigned int cmd, unsigned int arg)
{
    int data = (int) arg;
    int retval;
    
    /* Get the actual command from the ioctl request. */
    unsigned int cmd_num = _IOC_NR(cmd);

    tracemsg(_k_d("power management ioctl(), request 0x%X (cmd %d)"),(int) cmd, (int)cmd_num);
    
    /* Handle the request. */
    switch(cmd)
    {
        case POWER_IC_IOCTL_PMM_SET_CORE_VOLTAGE:
            /* pass the data to the function and return the results back the the caller in user space. */
            retval = power_ic_set_core_voltage(data);
            break;

             
        default: /* This shouldn't be able to happen, but just in case... */
            tracemsg(_k_d("=> 0x%X unsupported power management ioctl command"), (int) cmd);
            return -EINVAL;
            break;
  
        }

    return retval;
}
