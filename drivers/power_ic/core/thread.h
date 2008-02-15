/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/core/thread.h
 * 
 * Description - header file contains common thread handling functions and definitions.
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
 * 2005-Jul-20 - Design of the common thread handling functions and definitions.
 */

#ifndef __POWER_IC_THREAD_H__
#define __POWER_IC_THREAD_H__

/*!
 * @file thread.h
 *
 * @ingroup poweric_core
 *
 * @brief Central location for thread-related code.
 *
 * This header contains definitions for thread-handling code that is common to
 * all threads created by the power IC driver.
 */
 
  
#include <linux/sched.h>
 
/*
 * @name Thread Priorities
 *
 * This section is the location for all kernel thread priorities for the power IC driver.
 * They are kept here so that they are all in a single place, which should make figuring 
 * out relative priorities between threads easier, as well as explaining any rationale
 * behind their settings.
 *
 * From high to low priority, the current thread priority order is:
 *
 * - Event thread
 * - Touchscreen thread
 * - Debounce thread
 * - EMU state machine thread
 */
 
/* @{ */

/*! 
 * @brief Priority for event handling thread.
 *
 * The thread that deals with events for the driver needs to have a high priority in order
 * to keep event latency to a minimum. At this time, this is the highest-priority thread
 * in the driver  */
#define THREAD_PRIORITY_EVENT      (MAX_RT_PRIO - 1)

/*! 
 * @brief Priority for touchscreen thread.
 *
 * The touchscreen thread will either be completely asleep or will wake periodically 
 * (typically once per jiffy) in order to update the position of the touchscreen press.
 * Since this is driven directly by the user, this needs to be a high priority thread. 
 * This is the next highest-priority thread in the driver. */
#define THREAD_PRIORITY_TOUCHSCREEN (MAX_RT_PRIO - 2)

/*! 
 * @brief Priority for debouncing thread.
 *
 * The debounce thread will usually run soon after an event occurs in order to debounce
 * a signal. Since debouncing takes a significant amount of time, this isn't quite as 
 * critical, so this has a lower priority than the touchscreen and event threads. */
#define THREAD_PRIORITY_DEBOUNCE    (MAX_RT_PRIO - 3)

/*! 
 * @brief Priority for EMU state machine thread.
 *
 * The EMU state machine will run on EMU bus-related events and will involve a lot of
 * switching controls on and off and debouncing. All this will likely take a while,
 * so this is the lowest priority thread for the driver, although this thread should
 * still have a higher than normal priority so that it isn't blocked by the screensaver
 * or whatever. */
#define THREAD_PRIORITY_EMU         (MAX_RT_PRIO - 10)

/* @} End of thread priorities --------------------------------------------------------------------*/


extern void thread_common_setup(char * thread_name);
extern int thread_set_realtime_priority(int priority);

#endif /* __POWER_IC_THREAD_PRIORITY__ */
