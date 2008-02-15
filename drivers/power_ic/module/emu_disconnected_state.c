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
 * Motorola 2005-Nov-02 - Finalize the software.
 *
 */

/*!
 * @file emu_disconnected_state.c
 *
 * @ingroup poweric_emu
 *
 * @brief EMU accessory detection Disconnected State functions.
 *
 * The Disconnected State is a terminal state used when there is no accessory
 * attached to the EMU bus.
 *
 * The only transition out of the Disconnected state is to the Connecting
 * state. This transition is caused by either a VBUS or ID change
 * interrupts.
 */

#include <linux/sched.h>
#include "emu.h"
#include "../core/os_independent.h"

/******************************************************************************
* Global functions
******************************************************************************/

/*!
 * @brief Disconnected State entry function
 *
 * The entry into the Disconnected State is used to configure the hardware
 * in the lowest-power mode that still allows for the detection of the insertion
 * of an accessory.
 
 * @param prev_state        the previous EMU state
 * @param polling_interval  a return parameter that holds the "polling rate"
 */
void disconnected_enter(EMU_STATE_T prev_state, int *polling_interval)
{    
    /* Unmask IDI and CHGDETI */
    power_ic_event_unmask(EMU_INT_VBUS);
    power_ic_event_unmask(EMU_INT_ID);

    /* Config hardware in the lowest-power mode */
    emu_default_register_settings(polling_interval);
    
    /* Now wait for an interrupt */
    *polling_interval = EMU_POLL_WAIT_EVENT;
}

/*!
 * @brief Disconnected State handler function
 *
 * Since the Disconnected state is a terminal state, it is only concerned with
 * receiving events from the power IC that indicate that an accessory may have
 * possibly been connected to the phone.  These events are the Charger Detection
 * and ID Change events.  When these events are received, the only required
 * processing is to transition to the Connecting: Debounce state to allow for
 * accessory identification.
 *
 * @param polling_interval a return parameter that holds the "polling rate"
 *
 * @return next_state tells the state machine what the next EMU state is
 */
EMU_STATE_T disconnected_handler(int *polling_interval)
{
    /* Just to be consistant */
    *polling_interval = EMU_POLL_CONTINUE;
    
    /* Received an interrrupt go debounce the bus */
    return EMU_STATE_CONNECTING__DEBOUNCE_DEV_TYPE;
}
