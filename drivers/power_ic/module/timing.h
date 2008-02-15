#ifndef __POWER_IC_TIMING_H__
#define __POWER_IC_TIMING_H__
/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/module/timing.h
 * 
 * Description - This is the header of definitions and prototypes for 
 *               the debug timing code.
 *
 * Copyright (C) 2005 - Motorola, Inc. All Rights Reserved.
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
 * 2005-Jun-22 - Design code to support timing analysis for the driver. 
 */
/*!
 * @file timing.h
 *
 * @ingroup poweric_debug
 *
 * @brief This is the header of definitions and prototypes for the debug timing code.
 */ 

#include <linux/time.h>
#include <stdbool.h>
 /*==================================================================================================
                                        CONSTANTS
==================================================================================================*/
/*! The maximum start and stop times stored for any given timer. 
 *  This is pretty arbitrary - feel free to increase it if you need to. */
#define TIMING_MAX_TIMES_STORED 1000
/*==================================================================================================
                                         MACROS
==================================================================================================*/
/*! Used to create a new timer. This ensures that the timer is 
 *  not put on the stack, as they can get pretty big. */
#define TIMING_NEW_TIMER(new_timer) static TIMING_TIMER_T new_timer

/*==================================================================================================
                                         TYPEDEFS 
==================================================================================================*/

/*! One pair of start/stop times for a timer. Not to be used directly. */
typedef struct
{
    struct timeval start_time;
    struct timeval stop_time;
} TIMING_SINGLE_TIME_T;

/*! Type of a single timer. */
typedef struct
{
    int num_times_to_store;  /*!< The number of times to be held. */
    int num_times_recorded;  /*!< The number of times that have been captured. */
    bool timer_started;      /*!< Remembers if the timer has been started or is stopped. */

    TIMING_SINGLE_TIME_T times[TIMING_MAX_TIMES_STORED];
} TIMING_TIMER_T;

/*==================================================================================================
                                 GLOBAL FUNCTION PROTOTYPES
==================================================================================================*/
extern void timing_init(TIMING_TIMER_T * timer, int times_to_store);
extern int timing_start_timer(TIMING_TIMER_T * timer);
extern int timing_stop_timer(TIMING_TIMER_T * timer);
extern void timing_print_info(TIMING_TIMER_T * timer, bool show_all);
extern void timing_print_rate_info(TIMING_TIMER_T * timer, bool show_all);
extern int timing_get_num_times_stored(TIMING_TIMER_T * timer);

#endif /* __POWER_IC_TIMING_H__ */
