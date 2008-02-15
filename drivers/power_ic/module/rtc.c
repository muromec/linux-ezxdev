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
 * Motorola 2005-May-12 - Added support for Atlas
 * Motorola 2005-Jun-16 - Finalized code.
 *
 */

/*!
 * @file rtc.c
 *
 * @brief This is the main file of the power IC RTC interface.
 *
 * @ingroup poweric_rtc
 */

/*==================================================================================================
                                                                               
    Module Name:  rtc.c

    General Description:
    
    Module used to read the RTC registers from PCAP2.

====================================================================================================
                                        INCLUDE FILES
==================================================================================================*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
#include <linux/compatmac.h>
#else
#include <asm/uaccess.h>
#endif

#include <linux/power_ic.h>

#include "rtc.h"
#include "../core/os_independent.h"

/*==================================================================================================
                                 LOCAL FUNCTION PROTOTYPES
==================================================================================================*/

/*==================================================================================================
                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/

/*==================================================================================================
                                     LOCAL CONSTANTS
==================================================================================================*/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
#ifdef CONFIG_MOT_POWER_IC_PCAP2
#define POWER_IC_POWER_CUT_BIT             14
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
#define POWER_IC_POWER_CUT_BIT              8
#endif
#define POWER_IC_POWER_CUT_NUM_BITS         4
#define POWER_IC_TIME_REG_BIT               0
#define POWER_IC_TOD_NUM_BITS              17
#define POWER_IC_DAY_NUM_BITS              15
#define POWER_IC_NUM_SEC_PER_DAY        86400

#ifdef CONFIG_MOT_POWER_IC_PCAP2
#define RTC_TOD_REG                     POWER_IC_REG_PCAP_RTC_TOD
#define RTC_DAY_REG                     POWER_IC_REG_PCAP_RTC_DAY
#define RTC_TODA_REG                    POWER_IC_REG_PCAP_RTC_TODA
#define RTC_DAYA_REG                    POWER_IC_REG_PCAP_RTC_DAYA
#define RTC_POWER_CUT_REG               POWER_IC_REG_PCAP_RTC_DAY
#define RTC_TODA_EVENT                  POWER_IC_EVENT_PCAP_TODAI
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
#define RTC_TOD_REG                     POWER_IC_REG_ATLAS_RTC_TIME
#define RTC_DAY_REG                     POWER_IC_REG_ATLAS_RTC_DAY
#define RTC_TODA_REG                    POWER_IC_REG_ATLAS_RTC_ALARM
#define RTC_DAYA_REG                    POWER_IC_REG_ATLAS_RTC_DAY_ALARM
#define RTC_POWER_CUT_REG               POWER_IC_REG_ATLAS_PWR_CONTROL_1
#define RTC_TODA_EVENT                  POWER_IC_EVENT_ATLAS_TODAI
#endif
#endif /* Doxygen */

/*==================================================================================================
                                        LOCAL MACROS
==================================================================================================*/

/*==================================================================================================
                                      LOCAL VARIABLES
==================================================================================================*/

/*==================================================================================================
                                     GLOBAL VARIABLES
==================================================================================================*/

/*==================================================================================================
                                     EXPORTED SYMBOLS
==================================================================================================*/

#ifndef DOXYGEN_SHOULD_SKIP_THIS
EXPORT_SYMBOL(power_ic_rtc_set_time);
EXPORT_SYMBOL(power_ic_rtc_get_time);
EXPORT_SYMBOL(power_ic_rtc_set_time_alarm);
EXPORT_SYMBOL(power_ic_rtc_get_time_alarm);
EXPORT_SYMBOL(power_ic_get_num_power_cuts);
#endif

/*!
 * @brief The ioctl() handler for the RTC interface
 *
 * This function is the ioctl() interface handler for all RTC operations.
 * It is not called directly through an ioctl() call on the power IC device,
 * but is executed from the core ioctl() handler for all ioctl() requests in the
 * range for the RTC.
 *
 * @param     cmd   the ioctl() command
 * @param     arg   the ioctl() argument
 *
 * @return 0 if successful
 */

int rtc_ioctl(unsigned int cmd, unsigned long arg)
{
    struct timeval power_ic_time;
    struct timeval * usr_spc_time_val = (struct timeval *)arg;
    int err = 0;

    /* Handle the request. */
    switch(cmd)
    {
        case POWER_IC_IOCTL_GET_TIME:
	    /* Read the TOD and DAY registers and set the data in the timeval
	       structure to the format the Linux uses.*/
	    err = power_ic_rtc_get_time(&power_ic_time);
	    if(copy_to_user(usr_spc_time_val, &power_ic_time, sizeof(power_ic_time)))
	    {
	       err = -EFAULT;
	    }
	    break;
        case POWER_IC_IOCTL_GET_ALARM_TIME:
	    /* Read the TODA and DAYA registers and set the data in the timeval
	       structure to the format the Linux uses.*/
	    err = power_ic_rtc_get_time_alarm(&power_ic_time);
	    if(copy_to_user(usr_spc_time_val, &power_ic_time, sizeof(power_ic_time)))
	    {
	       err = -EFAULT;
	    }
	    break;
        case POWER_IC_IOCTL_SET_TIME:
	    /* Write to  the TOD and DAY registers based on the data in the timeval struct */
	    if(copy_from_user(&power_ic_time,usr_spc_time_val, sizeof(power_ic_time)))
	    {
	       err = -EFAULT;
	    }
	    err = power_ic_rtc_set_time(&power_ic_time);
	    break;	
        case POWER_IC_IOCTL_SET_ALARM_TIME:
	    /* Write to  the TODA and DAYA registers based on the data in the timeval struct */
	    if (copy_from_user(&power_ic_time,usr_spc_time_val,sizeof(power_ic_time)))
	    {
	       err = -EFAULT;
	    }	      
	    err = power_ic_rtc_set_time_alarm(&power_ic_time);
	    break;	
        default: /* This shouldn't be able to happen, but just in case... */
            tracemsg(_k_d("0x%X unsupported ioctl command"), (int) cmd);
            err =  -ENOTTY;
	    break;
    }
    
    return err;
}

/*!
 * @brief Sets the RTC time
 *
 * This function sets the value in the RTC_TOD and RTC_DAY registers based on the
 * number of seconds that have passed since January, 1 1970 00:00:00 UTC.
 *
 * @param power_ic_time pointer to the time in seconds stored in memory
 *
 * @return 0 if successful
 */

int power_ic_rtc_set_time(struct timeval *power_ic_time)
{
    int err = 0;
    if (power_ic_time->tv_usec > 500000)
    {
      power_ic_time->tv_sec += 1;
    }
    err = power_ic_set_reg_value(RTC_TOD_REG,POWER_IC_TIME_REG_BIT,
				 power_ic_time->tv_sec % POWER_IC_NUM_SEC_PER_DAY,  POWER_IC_TOD_NUM_BITS );
    err = power_ic_set_reg_value(RTC_DAY_REG,POWER_IC_TIME_REG_BIT,
				 power_ic_time->tv_sec / POWER_IC_NUM_SEC_PER_DAY, POWER_IC_DAY_NUM_BITS);
    tracemsg(_k_d("Set RTC Time \n RTC_TOD = %d \n RTC_DAY = %d \n tv_sec = %d Error = %d \n"),((int)power_ic_time->tv_sec % POWER_IC_NUM_SEC_PER_DAY) , 
	     ((int)power_ic_time->tv_sec / POWER_IC_NUM_SEC_PER_DAY),((int)power_ic_time->tv_sec), err );
    return err;
}

/*!
 * @brief Gets the RTC time
 *
 * This function retrieves the value in the RTC_TOD and RTC_DAY registers.
 * Those values are converted into the number of seconds that have elapsed
 * since January, 1 1970 00:00:00 UTC.
 *
 * @param power_ic_time pointer to the time in seconds stored in memory
 *
 * @return 0 if successful
 */

int power_ic_rtc_get_time(struct timeval *power_ic_time)
{
    int tod_reg_val = 0;
    int day_reg_val = 0;
    int err = 0;
    
    err = power_ic_get_reg_value(RTC_TOD_REG, POWER_IC_TIME_REG_BIT, 
				 &tod_reg_val, POWER_IC_TOD_NUM_BITS);
    err = power_ic_get_reg_value(RTC_DAY_REG, POWER_IC_TIME_REG_BIT,
				 &day_reg_val, POWER_IC_DAY_NUM_BITS);
    power_ic_time->tv_sec = tod_reg_val + day_reg_val * POWER_IC_NUM_SEC_PER_DAY;
    power_ic_time->tv_usec = 0;
    printk("Get RTC Time \n RTC_TOD = %d \n RTC_DAY = %d \n tv_sec = %d Error = %d \n", tod_reg_val, 
	   day_reg_val,(int)power_ic_time->tv_sec, err );
    return err;
}

/*!
 * @brief Sets the RTC alarm time
 *
 * This function sets the value in the RTC_TODA and RTC_DAYA registers based on the
 * number of seconds that have passed since January, 1 1970 00:00:00 UTC.
 *
 * @param power_ic_time pointer to the time in seconds stored in memory
 *
 * @return 0 if successful
 */

int power_ic_rtc_set_time_alarm(struct timeval *power_ic_time)
{

    int err = 0;
    
    if (power_ic_time->tv_usec > 500000)
    {
       power_ic_time->tv_sec += 1;
    }
    err = power_ic_set_reg_value(RTC_TODA_REG,POWER_IC_TIME_REG_BIT,
				 power_ic_time->tv_sec % POWER_IC_NUM_SEC_PER_DAY,  POWER_IC_TOD_NUM_BITS );
    err = power_ic_set_reg_value(RTC_DAYA_REG,POWER_IC_TIME_REG_BIT,
				 power_ic_time->tv_sec / POWER_IC_NUM_SEC_PER_DAY, POWER_IC_DAY_NUM_BITS);

    err |= power_ic_event_unmask(RTC_TODA_EVENT);
    
    tracemsg(_k_d("Set RTC Alarm Time \nRTC_TODA = %d \n RTC_DAYA = %d \n tv_sec = %d Error = %d \n"),(int)power_ic_time->tv_sec % POWER_IC_NUM_SEC_PER_DAY , 
	     (int)power_ic_time->tv_sec / POWER_IC_NUM_SEC_PER_DAY,(int)power_ic_time->tv_sec, err );
    return err;
}

/*!
 * @brief Gets the RTC alarm time
 *
 * This function retrieves the value in the RTC_TODA and RTC_DAYA registers.
 * Those values are converted into the number of seconds that have elapsed
 * since January, 1 1970 00:00:00 UTC.
 *
 * @param power_ic_time pointer to the time in seconds stored in memory
 *
 * @return 0 if successful
 */

int power_ic_rtc_get_time_alarm(struct timeval *power_ic_time)
{
    int toda_reg_val = 0;
    int daya_reg_val = 0;
    int err = 0;
   
    err = power_ic_get_reg_value(RTC_TODA_REG, POWER_IC_TIME_REG_BIT, 
				 &toda_reg_val, POWER_IC_TOD_NUM_BITS);
    err = power_ic_get_reg_value(RTC_DAYA_REG, POWER_IC_TIME_REG_BIT,
				 &daya_reg_val, POWER_IC_DAY_NUM_BITS);

    power_ic_time->tv_sec = toda_reg_val + daya_reg_val * POWER_IC_NUM_SEC_PER_DAY;
    power_ic_time->tv_usec = 0;
 
    tracemsg(_k_d("Get RTC Alarm Time \n RTC_TODA = %d \n RTC_DAYA = %d \n tv_sec = %d Error = %d \n"), toda_reg_val, 
	     daya_reg_val,(int)power_ic_time->tv_sec, err );
    return err;
}

/*!
 * @brief Gets the value of the power cut counter
 *
 * This function reads the power cut counter from the power IC.
 *
 * @param power_cuts pointer to location in which to store the power cut counter value
 *
 * @return 0 if successful
 */

int power_ic_get_num_power_cuts(int * power_cuts)
{
    int err = 0;
    
    err = power_ic_get_reg_value(RTC_POWER_CUT_REG, POWER_IC_POWER_CUT_BIT, 
                 		 power_cuts, POWER_IC_POWER_CUT_NUM_BITS);
    return (err);
}
