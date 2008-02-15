/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/module/touchscreen.c
 * 
 * Description - This file contains all of the functions and data structures required to implement the
 * interface module between touchscreen manager from user space and power IC driver.
 *
 * Copyright (C) 2005 Motorola, Inc. All Rights Reserved.
 */

/* 
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
 * 2005-Apr-27 - Design of the touchscreen module
 *
 * 2005-Jul-13 - Added robustness to the touchscreen module
 */

/*!
 * @file touchscreen.c
 *
 * @ingroup poweric_touchscreen
 *
 * @brief Power IC touchscreen module
 *
 * This file contains all of the functions and data structures required to implement the
 * interface module between touchscreen manager from user space and power IC driver. This touchscreen
 * module gives the user more flexibility to control the touchscreen registers and tell when the 
 * touchscreen is being used.
 *
 * Note that for programmed poll intervals of less than 1 jiffy in duration, the timeout
 * calculated and passed to wait_event_timeout() will be zero jiffies. This is deliberate.
 * We are assuming that a timeout of zero doesn't mean that the thread will not sleep, but rather
 * that the wait will be less than 1 whole jiffy in duration and that the thread will then wake 
 * at the start of the next jiffy.
 *
 * This means that while there may be cases where the period between two position updates will
 * be significantly less than 1 jiffy (i.e. when polling starts and the jiffy is already half
 * over), for the most part we expect updates to occur fairly evenly spaced, 1 per jiffy.
 *
 * This works because the amount of work in performing the AtoD conversions and calling
 * the position callback function all adds up to significantly less than a jiffy in duration. 
 * The AtoD measurements are both completed in a total of < 2ms, and the callback should be
 * negligible in length. If these ever consume the majority of a jiffy, performance will 
 * drop off significantly.
 */

/*====================================================================================================
                                        INCLUDE FILES
==================================================================================================*/
#include <asm/errno.h>
#include <asm/semaphore.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/locks.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/power_ic.h>
#include <stdbool.h>

#include "touchscreen.h"
#include "../core/event.h"
#include "../core/os_independent.h"
#include "../core/thread.h"

/*==================================================================================================
                                     MACROS AND CONSTANTS
==================================================================================================*/
#define REPEAT_INT_MIN         10                      /*!< Minimum allowable period, in ms. */

#define EVENT_TSI              POWER_IC_EVENT_PCAP_TSI /*!< Register used to subscribe event. */   

/*! This will convert a timeout in millisecs to jiffies, always rounding down. */
#define TO_JIFFIES(msec) (((msec) * HZ) / 1000)

/*! This will convert a number of jiffies, back to milliseconds, rounding down. */
#define TO_MILLISECS(jiffy) (((jiffy) * 1000) / HZ)
/*==================================================================================================
                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/

/*! States to determine what Touchscreen thread is doing. */
typedef enum
{
    POWER_IC_TS_IDLE,
    POWER_IC_TS_POLLING,
    POWER_IC_TS_DISABLED
} POWER_IC_TS_STATE_T;

/*==================================================================================================
                                     LOCAL CONSTANTS
==================================================================================================*/

/*==================================================================================================
                                        LOCAL MACROS
==================================================================================================*/

/*==================================================================================================
                                      LOCAL VARIABLES
==================================================================================================*/

/*! Period between samples of the position in milliseconds. Initialised much larger than minimum
 *  so default use doesn't take much CPU time. */
static int poll_interval = TO_JIFFIES(REPEAT_INT_MIN * 10);

/*! Touchscreen Event Array */
static void * touchscreen_events[POWER_IC_TS_NUM] = {NULL, NULL, NULL};

/*! This flag will be used to check the state of the thread. */
static POWER_IC_TS_STATE_T  touchscreen_thread_state = POWER_IC_TS_DISABLED;

#ifndef DOXYGEN_SHOULD_SKIP_THIS /* Doxygen does a really bad job of documenting this stuff... */
/*! Processes wait for a touchscreen pressed event on this queue. */
DECLARE_WAIT_QUEUE_HEAD(touchscreen_wait_queue);
#endif

/*==================================================================================================
                                     GLOBAL VARIABLES
==================================================================================================*/

/*==================================================================================================
                                     LOCAL FUNCTIONS
==================================================================================================*/

/*!
 * @brief Implements the kernel thread for power IC touchscreen handling.
 *
 * The function that implements the kernel thread for touchscreen handling. This
 * function will be used to handle the repeated polling of touchscreen position. 
 *
 * @param unused An unused parameter
 *
 * @return the function should never return
 */

static int touchscreen_thread_loop (void *unused)
{
    int error;
    int x, y, pressure;

    /* Usual thread setup. */
    thread_common_setup("ktscrd");
    if(thread_set_realtime_priority(THREAD_PRIORITY_TOUCHSCREEN) != 0)
    {
        tracemsg(_a("Event thread - error setting thread priority."));
    }

    while(1)
    {
        /* Disable hardware. */
        power_ic_atod_set_touchscreen_mode(POWER_IC_TS_MODE_DISABLED);
        
        /* Wait indefinitely for the touchscreen to be enabled. */
        wait_event(touchscreen_wait_queue, (touchscreen_thread_state != POWER_IC_TS_DISABLED));
        
        /* Clear touchscreen event */
        power_ic_event_clear(EVENT_TSI);
    
        /* unmask interrupt */
        power_ic_event_unmask(EVENT_TSI);
            
        /* Set up the hardware for the wait. */
        power_ic_atod_set_touchscreen_mode(POWER_IC_TS_MODE_LOW_PWR_MODE);
       
        /* Wait indefinitely for the touchscreen to be pressed or hardware to be disabled. */
        wait_event(touchscreen_wait_queue, (touchscreen_thread_state != POWER_IC_TS_IDLE));
        
        /* Mask touchscreen event */
        power_ic_event_mask(EVENT_TSI);
        
        if(touchscreen_thread_state == POWER_IC_TS_POLLING)
        {        
            /* Set hardware to full power. */
            power_ic_atod_set_touchscreen_mode(POWER_IC_TS_MODE_FULL_PWR_MODE);
                    
            /* Get touchscreen position */
            power_ic_atod_touchscreen_position_conversion(&x, &y);
                
            /* Call touchscreen pressed callback */
            if(touchscreen_events[POWER_IC_TS_PRESSED] != NULL)
            {
                (*((POWER_IC_TS_PRESS_POSITION_T)touchscreen_events[POWER_IC_TS_PRESSED]))(x,y);
            }
            else
            {
                tracemsg(_k_d("Pressed callback function is NULL"));
            }
       
            /* Polling loop */
            do
            {
                /* Disable hardware. */
                power_ic_atod_set_touchscreen_mode(POWER_IC_TS_MODE_DISABLED);
        
                /* Wait for polling period */
                wait_event_timeout(touchscreen_wait_queue, touchscreen_thread_state != POWER_IC_TS_POLLING, poll_interval);
                
                if(touchscreen_thread_state != POWER_IC_TS_POLLING)
                {
                    break;
                }
                
                /* Set hardware to full power. */
                power_ic_atod_set_touchscreen_mode(POWER_IC_TS_MODE_FULL_PWR_MODE);
       
                /* Get touchscreen pressure */
                error = power_ic_atod_touchscreen_pressure_conversion(&pressure);
                
                if(error != 0)
                {
                    pressure = 0;
                    tracemsg(_k_d("Error during pressure conversion...Pressure was %d at position %d,%d \
                        with an error of %d"), pressure, x, y, error);
                }
                
                if(pressure > 0)
                {
                    /* Get touchscreen position */
                    error = power_ic_atod_touchscreen_position_conversion(&x, &y);
                
                    if(error != 0)
                    {
                        tracemsg(_k_d("Error %d during position conversion"), error);
                    }
                    else
                    {
                        /* Call touchscreen position callback */
                        if(touchscreen_events[POWER_IC_TS_POSITION] != NULL)
                        {
                            (*((POWER_IC_TS_PRESS_POSITION_T)touchscreen_events[POWER_IC_TS_POSITION]))(x,y);
                        }
                        else
                        {
                            tracemsg(_k_d("Position callback was NULL"));
                        }
                    }
                }
            } 
            while (pressure > 0);
        
            /* Call touchscreen released callback */
            if(touchscreen_events[POWER_IC_TS_RELEASE] != NULL)
            {
                (*((POWER_IC_TS_RELEASE_T)touchscreen_events[POWER_IC_TS_RELEASE]))();
            }
            else
            {
                tracemsg(_k_d("Release callback was NULL"));
            }
        }
        
        if(touchscreen_thread_state == POWER_IC_TS_POLLING)
        {
            /* Set state to being Idle since done polling and not told to disable */
            touchscreen_thread_state = POWER_IC_TS_IDLE;
        }
    }
    
    return error;
}

/*!
 * @brief Touchscreen event handler.
 *
 * This function is the handler for the TSI interrupt that indicates the touchscreen
 * was pressed while in standby mode.
 *
 * @param        unused     The event type (not used).
 *
 * @return 1 to indicate the event was handled.
 */
static int touchscreen_event_handler(POWER_IC_EVENT_T unused)
{
    tracemsg(_k_d("Touchscreen event received."));
   
    /* We only need to wake up the process(es) that are waiting for a notification
     * that the touchscreen was pressed. */
    touchscreen_thread_state = POWER_IC_TS_POLLING;
    wake_up(&touchscreen_wait_queue);
    
    return 1;
}

/*==================================================================================================
                                     GLOBAL FUNCTIONS
==================================================================================================*/

/*!
 * @brief Initializes the power IC touchscreen handling
 *
 * This function initializes the power IC touchscreen handling. 
 *
 * @return nothing
 */
#ifdef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT
extern int ezx_ts_tsi_handler(POWER_IC_EVENT_T unused);
#endif /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */

void __init touchscreen_init(void)
{
#ifdef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT
    /* Register the handler for the TSI touchscreen event. */
    power_ic_event_subscribe(EVENT_TSI, &ezx_ts_tsi_handler);   

#else
    /* Register the handler for the TSI touchscreen event. */
    power_ic_event_subscribe(EVENT_TSI, &touchscreen_event_handler);   
    
    /* Start the touchscreen kernel thread */
    kernel_thread (touchscreen_thread_loop, NULL, 0); 
#endif /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */
}
 
/*!
 * @brief Registers a touchscreen callback
 *
 * This function registers the touchscreen callback and the corresponding function
 * that will be called from the thread.
 *
 * @param  callback     Touchscreen callback telling if touchscreen is pressed, position, or release.
 * @param  reg_ts_fcn   Function pointer to function that will be called from kernel thread.
 *
 * @return POWER_IC_TS_CALLBACK_T  The callback that was originally there
 */ 
void * power_ic_touchscreen_register_callback(POWER_IC_TS_CALLBACK_T callback, void * reg_ts_fcn)
{
    void * old_value = touchscreen_events[callback];
    
    touchscreen_events[callback] = reg_ts_fcn;
            
    return (old_value);
}

/*!
 * @brief Initializes the polling interval for touchscreen handling
 *    
 * This function is used to set the period between samples of the position in milliseconds
 * The minimum allowable period of REPEAT_INT_MIN will be set for the repeat interval so that all of the 
 * available CPU time is not consumed.
 *
 * @param  period   Period between position samples in milliseconds
 *
 * @return Period (in ms) accepted. If specified period was too low, then this will 
 * indicate that the period was limited to the returned value.
 *
 * @note Depending on the settings of the Linux kernel, the actual period used may not be
 * exactly that specified. The timing resolution is limited by the rate that jiffies occur
 * at in the kernel.
 */
int power_ic_touchscreen_set_repeat_interval(int period)
{
    if(period < REPEAT_INT_MIN)
    {
       tracemsg(_k_w("Warning --------------------------------------------------"));
       tracemsg(_k_w("Requested touchscreen repeat interval of %d ms is too short"), period);
       tracemsg(_k_w(" - restricting interval to minimum period %d ms."), REPEAT_INT_MIN);
       tracemsg(_k_w("----------------------------------------------------------"));
       
       period = REPEAT_INT_MIN;
    }
    
    poll_interval = TO_JIFFIES(period);
    
    /* If period was an exact number of jiffies, we need to reduce the period by 1 count. 
     * This is needed to keep performance up - if a jiffy-exact poll interval is specified,
     * then we will do work in the first jiffy of that interval and then wait for the other
     * remaining jiffies. This is to do with the wait_event_timeout() behaviour described in
     * the notes for this file. */
    if(TO_MILLISECS(poll_interval) == period)
    {
        poll_interval--;
    }
    
    tracemsg(_k_d("Repeat interval set to %d jiffies for %d millisecs"), poll_interval, period);
    
    return period;
}

/*!
 * @brief Enables/Disables Touchscreen.
 *
 * This function will set the thread state and wake up the touchscreen queue
 * *
 * @pre While not specifically required, a callback function should generally be
 * registered before unmasking an event.
 *
 * @return nothing
 */
void power_ic_touchscreen_enable(bool on)
{
    int pressure;
    touchscreen_thread_state = POWER_IC_TS_DISABLED;
    
    if(on)
    {
        touchscreen_thread_state = POWER_IC_TS_IDLE;
        
        /* Get touchscreen pressure;  This must be done to fix the case where a stylus is already
           on the touchscreen...pressure is being applied...before the touchscreen is enabled. If
           this is the case, then we want touchscreen to automatically start polling for position
           and pressure measurements. */
        power_ic_atod_touchscreen_pressure_conversion(&pressure);
                
        if(pressure > 0)
        {
            /* Touchscreen is enabled and has been pressed...set to polling */
            touchscreen_thread_state = POWER_IC_TS_POLLING;
        }
    }
     
    wake_up(&touchscreen_wait_queue);
}

/*!
 * @brief The ioctl() handler for the Touchscreen interface
 *
 * This function is the ioctl() interface handler for all Touchscreen operations.
 * It is not called directly through an ioctl() call on the power IC device,
 * but is executed from the core ioctl() handler for all ioctl() requests in the
 * range for the Touchscreen.
 *
 * @note   These ioctls are just used for testing.  If called, nothing successful will
 *         come out of this
 *
 * @param     cmd   the ioctl() command
 * @param     arg   the ioctl() argument
 *
 * @return 0 if successful
 */

int touchscreen_ioctl(unsigned int cmd, unsigned long arg)
{
    int err = 0;
    int data = (int) arg;
    
    /* Handle the request. */
    switch(cmd)
    {
        case POWER_IC_IOCTL_TS_ENABLE:
            tracemsg(_k_d("Enabling touchscreen"));
            power_ic_touchscreen_enable(true);
	        break;

        case POWER_IC_IOCTL_TS_DISABLE:
            tracemsg(_k_d("Disabling touchscreen"));
            power_ic_touchscreen_enable(false);
	        break;

        case POWER_IC_IOCTL_TS_SET_INTERVAL:
            tracemsg(_k_d("Setting touchscreen interval"));
            err = power_ic_touchscreen_set_repeat_interval(data);
	        break;	

        default: /* This shouldn't be able to happen, but just in case... */
            tracemsg(_k_d("0x%X unsupported ioctl command"), (int) cmd);
            err =  -ENOTTY;
	        break;
    }

    return err;
}
