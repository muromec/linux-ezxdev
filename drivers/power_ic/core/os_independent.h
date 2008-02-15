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
 * Motorola 2005-Sep-01 - Implementation of wait event with timeout. 
 * Motorola 2005-Feb-28 - File re-written from scratch.
 *
 */

#ifndef __OS_INDEPENDENT_H__
#define __OS_INDEPENDENT_H__

/*!
 * @file os_independent.h
 *
 * @brief Contains utilties for outputting information to the console
 *
 * @ingroup poweric_core
 */
 
#include <linux/time.h>
#include <linux/version.h>
#include <linux/wait.h>

/*! Implementation of __wait_event() with timeout not present in 2.4 kernel.
 *
 *  NOTE: Modified use of schedule_timeout based on Linux v2.6.12.5 kernel.
 */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
#define __wait_event_timeout(wq, condition, ret)                      \
do {                                                                  \
    wait_queue_t __wait;                                              \
    unsigned long expire;                                             \
    init_waitqueue_entry(&__wait, current);                           \
                                                                      \
    add_wait_queue(&wq, &__wait);                                     \
    for (;;) {                                                        \
	set_current_state(TASK_UNINTERRUPTIBLE);                      \
        if (condition)                                                \
            break;                                                    \
        ret = schedule_timeout(ret);                                  \
        if (!ret)                                                     \
            break;                                                    \
    }                                                                 \
    current->state = TASK_RUNNING;                                    \
    remove_wait_queue(&wq, &__wait);                                  \
} while (0)

 /*! Implementation of wait_event() with timeout not present in 2.4 kernel.
  *
  *  NOTE: Copied from Linux v2.6.12.5 kernel.
  */
#define wait_event_timeout(wq, condition, timeout)                    \
({                                                                    \
      long __ret = timeout;                                           \
      if (!(condition))                                               \
          __wait_event_timeout(wq, condition, __ret);                 \
      __ret;                                                          \
})
#endif /* End of Linux 2.4-only compile. */

/* Only compile trace message printing if build needs it. */
#ifdef CONFIG_MOT_POWER_IC_TRACEMSG
#define tracemsg(fmt,args...)  printk(fmt,##args)
#else
#define tracemsg(fmt,args...)
#endif

#define _a(a)           "Power IC: "a" \n"
#define _k_a(a) _a(a)
#define _k_d(a) KERN_DEBUG _a(a)
#define _k_i(a) KERN_INFO _a(a)
#define _k_w(a) KERN_WARNING _a(a)

/* Wrappers for looking at timing. Gets turned on/off with tracemsgs to avoid
 * a bunch of conditional compiles everywhere.. */
#ifdef CONFIG_MOT_POWER_IC_TRACEMSG
#define dbg_gettimeofday(time)  do_gettimeofday((time))
#define dbg_timeval(time) struct timeval time
#else
#define dbg_gettimeofday(time)
#define dbg_timeval(time) 
#endif

#endif /* __OS_INDEPENDENT_H__ */
