/*
 * Copyright 2004 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2004 Motorola, Inc. All Rights Reserved.
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
 * Motorola 2004-Dec-06 - Redesign of the Power IC event handler */

#ifndef __EVENT_H__
#define __EVENT_H__

/*!
 * @file event.h
 *
 * @brief This file contains prototypes for power IC core-internal event handling functions.
 *
 * @ingroup poweric_core
 */ 

#include <linux/power_ic.h>
#include <linux/version.h>

/* These definitions regarding interrupt handling are only available in linux 2.4.23 and after... */
#ifndef DOXYGEN_SHOULD_SKIP_THIS
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,4,23))
typedef void irqreturn_t;
#define IRQ_NONE
#define IRQ_HANDLED
#define IRQ_RETVAL(x)
#endif
#endif

#ifdef CONFIG_MOT_POWER_IC_PCAP2
#define POWER_IC_EVENT_NUM_EVENTS POWER_IC_EVENT_NUM_EVENTS_PCAP
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
#define POWER_IC_EVENT_NUM_EVENTS POWER_IC_EVENT_NUM_EVENTS_ATLAS
#endif

void power_ic_event_initialize (void);

#endif /* __EVENT_H__ */
