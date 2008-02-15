/*
 * Copyright 2004 - 2005 Motorola, Inc. All Rights Reserved.
 */

/* 
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
 */

/*!
 * @file emu_disconnecting_state.c
 *
 * @ingroup poweric_emu
 *
 * @brief EMU state machine disconnecting state functions.
 *
 * The Disconnecting state is a transient state used when a device has been removed
 * from the EMU bus and the software is in the process of debouncing and confirming
 * the removal of the device.
 *
 * There are three possible transitions out of the Disconnecting state: the
 * Disconnected, Connecting, or Connected states. The Disconnected state is entered
 * when the software has determined that the previously connected accessory has been
 * removed and was not replaced with a different accessory. The Connecting state is
 * entered when the software has determined that the previously connected accessory
 * has been removed and possibly replaced by a different accessory. The Connected
 * state is entered if the software determines that the previously connected accessory
 * was not removed (in which case the Disconnecting state was probably entered in
 * error).
 *
 */

#include <linux/sched.h>
#include <linux/power_ic.h>
#include "emu.h"
#include "../core/os_independent.h"

/******************************************************************************
* Global functions
******************************************************************************/

/*!
 * @brief Disconnecting State entry function.
 *
 * When the Disconnecting state is entered, it indicates that one of the other
 * states believes that the previously connected accessory may have been removed.
 * It is the job of the Disconnecting state to verify that the previous accessory
 * is no longer attached.  The entry function for the state is responsible for
 * configuring the hardware in the way that will allow for this to begin.
 *
 * @param prev_state        the previous EMU state
 * @param polling_interval  a return parameter that holds the "polling rate"
 */
void disconnecting_enter(EMU_STATE_T prev_state, int *polling_interval)
{
    /* disable reverse current */
    EMU_SET_REVERSE_MODE(DISABLE);

    /* We can't deal with interrupt firing while debouncing the removal
       of the device */
    power_ic_event_mask(EMU_INT_ID);
    power_ic_event_mask(EMU_INT_VBUS);

    /* Go directly to the disconnecting_handler */
    *polling_interval = EMU_POLL_CONTINUE;
}

/*!
 * @brief Handles the disconnecting state.
 *
 * @param polling_interval a return parameter that holds the "polling rate"
 *
 * @return next_state tells the state machine what the next EMU state is
 */
EMU_STATE_T disconnecting_handler(int *polling_interval)
{
    EMU_DEV_TYPE_T curr_device_type;
    
    /* Wait for the device type to be debounced before continuing */
    if (!(is_device_type_debounced(&curr_device_type)))
    {
        /* Come back again after the polling interval has expired */
        *polling_interval = EMU_POLL_DFLT_DEB_DELAY;
        return EMU_STATE_DISCONNECTING;
    }

    /* Finished debouncing, reset the counter */
    emu_debounce_counter = 0;

    /* Go directly to the next state */
    *polling_interval = EMU_POLL_CONTINUE;

    /* There is not device connected or the device has been removed,
       so go to the disconnected state */
    if (curr_device_type == EMU_DEV_TYPE_NONE)
    {
        return EMU_STATE_DISCONNECTED;
    }
    
    /* The current device type is the same as the previous device type. We would
       end up here when there was a glitch on the VBUS or ID lines, or if a partially
       inserted SPD has been fully inserted, (ie. the ID, D+ and D- pins have made contact
       with the EMU connector). */
    if (curr_device_type == emu_current_device_type)
    {
        /* A partially inserted SPD is always reported as a USB cable. The ID interrupt
           is left unmasked for the USB cable. If the partially inserted SPD is fully inserted
           the ID interrupt is generated and we end up here. This condition will decide if
           the previous device was a USB cable and it still is, or if the previous device
           was a USB cable and no it doesn't look like one. If the currently connected device
           was an USB cable, the device type debounced as an SPD and the ID is not floating then
           the currently connected device must have been fully inserted. This check is needed to
           keep the USB cable from losing enumeration in the case there was a glitch on the
           VBUS or ID line */
        if ((emu_current_device == MOTO_ACCY_TYPE_CABLE_USB) &&
            (curr_device_type == EMU_DEV_TYPE_SPD) &&
            (get_bus_state(EMU_BUS_SIGNAL_ID_FLOAT) == EMU_BUS_SIGNAL_STATE_DISABLED))
        {
            return EMU_STATE_CONNECTING__DEBOUNCE_DEV_TYPE;
        }
        else
        {
            /* Glitch on the VBUS or ID line just go back to the previous state */
            return emu_prev_state;
        }
    }

    /* If the device type and EMU bus state don't make sense start over */
    return EMU_STATE_CONNECTING__DEBOUNCE_DEV_TYPE;  
}

/*!
 * @brief Disconnecting State exit function.
 *
 * If the disconnecting state determined that the previously connected accessory
 * is no longer connected (i.e., the next state is Disconnected or Connecting),
 * the removal of the previous accessory needs to be signaled using the kernel
 * interface.  No hardware reconfiguration is required here as it will be
 * taken care of by the next state.
 *
 * @param next_state        the next EMU state
 * @param polling_interval  a return parameter that holds the "polling rate"
 */
void disconnecting_exit(EMU_STATE_T next_state, int *polling_interval)
{
    /* If we are NOT going directly back to the Connected State,
       notify the applications the current device has been removed
       and reset the device info */
    if ((next_state != EMU_STATE_CONNECTED__POLL_SPD_REMOVAL) &&
        (next_state != EMU_STATE_CONNECTED__FACTORY) &&
        (next_state != EMU_STATE_CONNECTED__HEADSET))
    {
        tracemsg(_k_d("EMU: disconnecting_exit: call user app disconnect interface"));
        
        moto_accy_notify_remove(emu_current_device);
            
        emu_current_device = MOTO_ACCY_TYPE_NONE;
        emu_current_device_type = EMU_DEV_TYPE_NONE;
    }
    
    *polling_interval = EMU_POLL_CONTINUE;
}
