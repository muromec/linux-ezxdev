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
 * Motorola 2005-Jun-30 - Updated the device_config_table. 
 * Motorola 2005-Dec-19 - Finalize the software.
 *
 */

/*!
 * @file emu_connected_state.c
 *
 * @ingroup poweric_emu
 *
 * @brief EMU state machine Connected State functions.
 *
 * The Connected state is used when a device is attached to the EMU bus after the
 * device has been identified. In most cases, the state is a terminal state.
 *
 * The only transition out of the Connected state is to the Disconnecting state. For
 * non-cable class devices, this transition is caused by either the Charger Detect
 * or ID Change interrupts.
 *
 */

#include <linux/keypad.h>
#include <linux/sched.h>
#include <linux/power_ic.h>
#ifdef CONFIG_MOT_POWER_IC_PCAP2
#include <linux/ezxusbd.h>
#endif
#include "emu.h"

#include "../core/os_independent.h"

/* This table must match the MOTO_ACCY_T enum */
static const struct
{
    BOOL vbus_mask  : 1; /*MASK -> VBUS masked*/
    BOOL id_mask    : 1; /*MASK -> ID masked*/
    BOOL id_pd      : 1; /*ENABLE -> ID pulled down*/
    BOOL charger_pu : 1; /*ENABLE -> D+ pulled up*/
    BOOL vbus_5k_pd : 1; /*ENABLE -> VBUS pulled down*/
    EMU_XCVR_T xcvr;
    EMU_CONN_MODE_T conn_mode;
}  device_config_table[] =
{
    /*MOTO_ACCY_T                           VBUS    ID      ID PD    D+ PU    VBUS5KPD  XCVR                   CONN MODE                */
    /*MOTO_ACCY_TYPE_NONE*/               { UNMASK, UNMASK, DISABLE, ENABLE,  ENABLE,   EMU_XCVR_OFF,          EMU_CONN_MODE_USB        },
    
    /*MOTO_ACCY_TYPE_INVALID*/            { UNMASK, UNMASK, DISABLE, ENABLE,  ENABLE,   EMU_XCVR_OFF,          EMU_CONN_MODE_USB        },
    
    /*MOTO_ACCY_TYPE_NOT_SUPPORTED*/      { UNMASK, UNMASK, DISABLE, ENABLE,  ENABLE,   EMU_XCVR_OFF,          EMU_CONN_MODE_USB        },
    
    /*MOTO_ACCY_TYPE_CHARGER_MID*/        { MASK,   MASK,   DISABLE, ENABLE,  ENABLE,   EMU_XCVR_OFF,          EMU_CONN_MODE_USB        },
    
    /*MOTO_ACCY_TYPE_CHARGER_MID_MPX*/    { MASK,   MASK,   DISABLE, ENABLE,  ENABLE,   EMU_XCVR_OFF,          EMU_CONN_MODE_USB        },
    
    /*MOTO_ACCY_TYPE_CHARGER_FAST*/       { MASK,   MASK,   DISABLE, ENABLE,  ENABLE,   EMU_XCVR_OFF,          EMU_CONN_MODE_USB        },
    
    /*MOTO_ACCY_TYPE_CHARGER_FAST_MPX*/   { MASK,   MASK,   DISABLE, ENABLE,  ENABLE,   EMU_XCVR_OFF,          EMU_CONN_MODE_USB        },
    
    /*MOTO_ACCY_TYPE_CHARGER_FAST_3G*/    { MASK,   MASK,   ENABLE,  ENABLE,  ENABLE,   EMU_XCVR_OFF,          EMU_CONN_MODE_USB        },
#ifdef CONFIG_MOT_POWER_IC_PCAP2    
    /*MOTO_ACCY_TYPE_CARKIT_MID*/         { MASK,   MASK,   DISABLE, ENABLE,  ENABLE,   EMU_XCVR_SPD_AUDIO,    EMU_CONN_MODE_MONO_AUDIO },
    
    /*MOTO_ACCY_TYPE_CARKIT_FAST*/        { MASK,   MASK,   DISABLE, ENABLE,  ENABLE,   EMU_XCVR_SPD_AUDIO,    EMU_CONN_MODE_MONO_AUDIO },
#elif defined(CONFIG_MOT_POWER_IC_ATLAS) 
    /*MOTO_ACCY_TYPE_CARKIT_MID*/         { MASK,   MASK,   DISABLE, ENABLE,  ENABLE,   EMU_XCVR_OFF,          EMU_CONN_MODE_MONO_AUDIO },
    
    /*MOTO_ACCY_TYPE_CARKIT_FAST*/        { MASK,   MASK,   DISABLE, ENABLE,  ENABLE,   EMU_XCVR_OFF,          EMU_CONN_MODE_MONO_AUDIO },
    
#endif    
    /*MOTO_ACCY_TYPE_CARKIT_SMART*/       { UNMASK, UNMASK, DISABLE, ENABLE,  ENABLE,   EMU_XCVR_OFF,          EMU_CONN_MODE_USB        },

    /*MOTO_ACCY_TYPE_CABLE_USB*/          { MASK,   UNMASK, DISABLE, DISABLE, DISABLE,  EMU_XCVR_USB_HOST,     EMU_CONN_MODE_USB        },
    
    /*MOTO_ACCY_TYPE_CABLE_REGRESSION*/   { MASK,   MASK,   DISABLE, DISABLE, DISABLE,  EMU_XCVR_USB_HOST,     EMU_CONN_MODE_USB        },
    
    /*MOTO_ACCY_TYPE_CABLE_FACTORY*/      { MASK,   MASK,   DISABLE, DISABLE, DISABLE,  EMU_XCVR_FACTORY_MODE, EMU_CONN_MODE_USB        },
    
    /*MOTO_ACCY_TYPE_HEADSET_MONO*/       { UNMASK, UNMASK, DISABLE, ENABLE,  ENABLE,   EMU_XCVR_OFF,          EMU_CONN_MODE_USB        },
    
    /*MOTO_ACCY_TYPE_HEADSET_STEREO*/     { UNMASK, UNMASK, DISABLE, ENABLE,  ENABLE,   EMU_XCVR_OFF,          EMU_CONN_MODE_USB        },
#ifdef CONFIG_MOT_POWER_IC_PCAP2    
    /*MOTO_ACCY_TYPE_HEADSET_EMU_MONO*/   { MASK,   UNMASK, DISABLE, ENABLE,  DISABLE,  EMU_XCVR_PPD_AUDIO,    EMU_CONN_MODE_MONO_AUDIO },
    
    /*MOTO_ACCY_TYPE_HEADSET_EMU_STEREO*/ { MASK,   UNMASK, DISABLE, ENABLE,  DISABLE,  EMU_XCVR_PPD_AUDIO,    EMU_CONN_MODE_MONO_AUDIO }
#elif defined(CONFIG_MOT_POWER_IC_ATLAS) 
    /*MOTO_ACCY_TYPE_HEADSET_EMU_MONO*/   { MASK,   UNMASK, DISABLE, ENABLE,  DISABLE,  EMU_XCVR_OFF,          EMU_CONN_MODE_MONO_AUDIO },
    
    /*MOTO_ACCY_TYPE_HEADSET_EMU_STEREO*/ { MASK,   UNMASK, DISABLE, ENABLE,  DISABLE,  EMU_XCVR_OFF,          EMU_CONN_MODE_MONO_AUDIO }
#endif
};

/******************************************************************************
* Global functions
******************************************************************************/

/*!
 * @brief Used to notify the apps of the current device and configures the
 *        current device appropriatly.
 * 
 *
 * @param prev_state the previous EMU state
 * @param polling_interval a return parameter that holds the "polling rate"
 */
void device_config_enter(EMU_STATE_T prev_state, int *polling_interval)
{
    if(emu_audio_update)
    {
        emu_util_set_emu_headset_mode(emu_audio_mode);
    }
    /* Do not disable power to the device if the device was not reported disconnected */
    if (emu_prev_state != EMU_STATE_DISCONNECTING)
    {
        /* Make sure that we are not supplying power to the accessory */
        EMU_SET_REVERSE_MODE(DISABLE);

        /* The ID line is used to for the Send/End key on headsets, keep
           track of the ID line's state. */
        if ((emu_current_device == MOTO_ACCY_TYPE_HEADSET_EMU_MONO) ||
            (emu_current_device == MOTO_ACCY_TYPE_HEADSET_EMU_STEREO))
        {
            emu_id_state = get_bus_state(EMU_BUS_SIGNAL_ID);
            emu_util_set_emu_headset_mode(MOTO_ACCY_HEADSET_MODE_NONE);
        }
        
#ifdef CONFIG_MOT_POWER_IC_PCAP2 /* Charger support not yet available for Atlas. */
        /* Workaround for EMU one-chip charging from factory cable. */
        else if(emu_current_device == MOTO_ACCY_TYPE_CABLE_FACTORY)
        {
            /* Workaround is to turn off ICHRG (again). */
            power_ic_charger_set_charge_current(0);
            tracemsg(_k_a("Factory cable workaround: turned off charger."));
        }
        
        if((emu_current_device == MOTO_ACCY_TYPE_CABLE_USB)||
           (emu_current_device == MOTO_ACCY_TYPE_CABLE_FACTORY)||
           (emu_current_device == MOTO_ACCY_TYPE_CABLE_REGRESSION))
        {
            tracemsg(_k_a("Set the GPIOs for usb."));
            usbd_set_gpio_function(1);
        }
#endif
    
        /* Configure the connected device */
        EMU_SET_ID_PULL_DOWN(device_config_table[emu_current_device].id_pd);
        EMU_SET_DPLUS_150K_PULL_UP(device_config_table[emu_current_device].charger_pu);
        EMU_SET_VBUS_5K_PULL_DOWN(device_config_table[emu_current_device].vbus_5k_pd);
        emu_configure_usb_xcvr(device_config_table[emu_current_device].xcvr);
        EMU_SET_EMU_CONN_MODE(device_config_table[emu_current_device].conn_mode);
    }

    /* Configure the EMU interrupts correctly for the connected device */
    if (device_config_table[emu_current_device].vbus_mask == MASK)
    {
        power_ic_event_mask(EMU_INT_VBUS);
    }
    else
    {
        power_ic_event_unmask(EMU_INT_VBUS);
    }

    if (device_config_table[emu_current_device].id_mask == MASK)
    {
        power_ic_event_mask(EMU_INT_ID);
    }
    else
    {
        power_ic_event_unmask(EMU_INT_ID);
    }

    tracemsg(_k_d("EMU: device_config_enter: notify user interface; CONNECTED DEVICE is %d"),
             emu_current_device);
    
    /* Notify the apps of the insertion of the EMU device */
    moto_accy_notify_insert(emu_current_device);

    /* If we are in the poll SPD state then go directly to the handler
       and start polling for the removal, otherwise just wait for an
       interrupt */
    *polling_interval = (emu_state == EMU_STATE_CONNECTED__POLL_SPD_REMOVAL) ?
        EMU_POLL_CONTINUE : EMU_POLL_WAIT_EVENT;
}

/*!
 * @brief This function polls for the removal of an SPD
 *
 * @param polling_interval a return parameter that holds the "polling rate"
 *
 * @return next_state tells the state machine what the next EMU state is
 */
EMU_STATE_T poll_spd_removal_handler(int *polling_interval)
{
    /* The I2C code seems to powers down before PCAP does and since the error code from get_device_type
       and get_bus_state is never returned, we really dont know if the reads we are getting
       back are good.  In this case, when powering down the phone with an accessory attached,
       at powerdown because the reads are not successful, we presume there is no accessory attached
       and the phone does not go into charge only mode if a charger is attached.
       Therefore, a check to see if something can be successfully read from EMU One Chip has to be
       done to make sure it is not shutdown. */  
    if (((get_device_type() != EMU_DEV_TYPE_SPD) ||
         ((emu_current_device == MOTO_ACCY_TYPE_CABLE_USB) &&
          (get_bus_state(EMU_BUS_SIGNAL_ID_FLOAT) == EMU_BUS_SIGNAL_STATE_DISABLED))) &&
        (power_ic_event_sense_read(EMU_SENSE_ID_FLOAT) >= 0))
    {
        /* If the device type is no longer SPD or if a USB cable is
           thought to have been attached but the id is no floating
           then go find out what happened */
        *polling_interval = EMU_POLL_CONTINUE;
        return EMU_STATE_DISCONNECTING;
    }

    /* Keep polling until the device type is no longer SPD */
    *polling_interval = EMU_POLL_SPD_REMOVAL_DELAY;
    return EMU_STATE_CONNECTED__POLL_SPD_REMOVAL;
}

/*!
 * @brief This function transitions to disconnecting state when an interrupt
 *        is recieved.
 *
 * @param polling_interval a return parameter that holds the "polling rate"
 *
 * @return next_state tells the state machine what the next EMU state is
 *
 * @note This handler is used for the factory cable, invalid and not supported
 *       devices.
 */
EMU_STATE_T factory_handler(int *polling_interval)
{
    *polling_interval = EMU_POLL_CONTINUE;
    return EMU_STATE_DISCONNECTING;
}

/*!
 * @brief Used to handle activity on the bus when a basic device is connected
 *
 * The Connected: basic device handler state is used to handle send/end key
 * events for mono or stereo headsets. If the event is not send/end key related
 * this function will determine what the next state should be.
 *
 * @param polling_interval a return parameter that holds the "polling rate"
 *
 * @return next_state tells the state machine what the next EMU state is
 */
EMU_STATE_T headset_handler(int *polling_interval)
{
    EMU_BUS_SIGNAL_STATE_T new_id_state;
    int id_handler = 0;
    /*Define the handler for different ID state.*/
#define EMU_ID_HANDLER_NONE          0
#define EMU_ID_HANDLER_DISCONNECTING 0x0001
#define EMU_ID_HANDLER_SEND_END_DOWN 0x0002
#define EMU_ID_HANDLER_SEND_END_UP   0x0004    
    static const int headset_handler[EMU_BUS_SIGNAL_STATE_ID_RESISTOR-EMU_BUS_SIGNAL_STATE_UNKNOWN][EMU_BUS_SIGNAL_STATE_ID_RESISTOR-EMU_BUS_SIGNAL_STATE_UNKNOWN] =
        {
            /* emu_id_state                  new_id_state */
            {
                /*EMU_BUS_SIGNAL_STATE_ID_FLOATING,EMU_BUS_SIGNAL_STATE_ID_FLOATING*/
                EMU_ID_HANDLER_NONE,
                /*EMU_BUS_SIGNAL_STATE_ID_FLOATING,EMU_BUS_SIGNAL_STATE_ID_GROUNDED */
                EMU_ID_HANDLER_SEND_END_DOWN,
                /*EMU_BUS_SIGNAL_STATE_ID_FLOATING,EMU_BUS_SIGNAL_STATE_ID_RESISTOR*/
                EMU_ID_HANDLER_DISCONNECTING      
            },
            
            {
                /*EMU_BUS_SIGNAL_STATE_ID_GROUNDED EMU_BUS_SIGNAL_STATE_ID_FLOATING*/
                EMU_ID_HANDLER_DISCONNECTING|EMU_ID_HANDLER_SEND_END_UP,
                /*EMU_BUS_SIGNAL_STATE_ID_GROUNDED EMU_BUS_SIGNAL_STATE_ID_GROUNDED */
                EMU_ID_HANDLER_NONE,
                /*EMU_BUS_SIGNAL_STATE_ID_GROUNDED EMU_BUS_SIGNAL_STATE_ID_RESISTOR*/
                EMU_ID_HANDLER_SEND_END_UP
            },
            {  
                /*EMU_BUS_SIGNAL_STATE_ID_RESISTOR  EMU_BUS_SIGNAL_STATE_ID_FLOATING*/
                EMU_ID_HANDLER_DISCONNECTING,
                /*EMU_BUS_SIGNAL_STATE_ID_RESISTOR  EMU_BUS_SIGNAL_STATE_ID_GROUNDED */
                EMU_ID_HANDLER_SEND_END_DOWN,
                /*EMU_BUS_SIGNAL_STATE_ID_RESISTOR  EMU_BUS_SIGNAL_STATE_ID_RESISTOR*/
                EMU_ID_HANDLER_NONE
            }
            
        };
                
    tracemsg(_k_d("basic_device_handler:"));
    
    if ((emu_current_device == MOTO_ACCY_TYPE_HEADSET_EMU_MONO) ||
        (emu_current_device == MOTO_ACCY_TYPE_HEADSET_EMU_STEREO))
    {
        if (is_bus_state_debounced(EMU_BUS_SIGNAL_ID,
                                   &new_id_state,
                                   EMU_POLL_DEB_DONE_CNT))
        {   
            /* ID is debounced reset the counter */
            emu_debounce_counter = 0;
            id_handler = headset_handler[emu_id_state - EMU_BUS_SIGNAL_STATE_ID_FLOATING][new_id_state - EMU_BUS_SIGNAL_STATE_ID_FLOATING];
            
            if((id_handler & EMU_ID_HANDLER_SEND_END_UP)!=0)
            {
                /* send/end key released, send message to apps */
                tracemsg(_k_d("EMU: basic_device_handler: send/end key released"));
                emu_headset_key_handler(KEYUP);
            }
            if((id_handler & EMU_ID_HANDLER_SEND_END_DOWN)!=0)
            {
                /* send/end key pressed, send message to apps */
                tracemsg(_k_d("EMU: basic_device_handler: send/end key pressed"));
                emu_headset_key_handler(KEYDOWN);
            }
         
            if((id_handler & EMU_ID_HANDLER_DISCONNECTING) !=0)
            {
                tracemsg(_k_d("EMU: basic_device_handler: ID is floating")); 
                /* If ID is floating at any point go to the directly to
                   the disconnecting state */
                *polling_interval = EMU_POLL_CONTINUE;
                return EMU_STATE_DISCONNECTING;
            }
            else
            {
                /* update the id_state */
                emu_id_state = new_id_state;

                /* Don't forget to unmask the ID interrupt */
                power_ic_event_unmask(EMU_INT_ID);

                /* Go wait for the next interrupt */
                *polling_interval = EMU_POLL_WAIT_EVENT;
                return EMU_STATE_CONNECTED__HEADSET;
            }
        }
        
        else
        {
            /* Continue debouncing ID */
            *polling_interval = EMU_POLL_FAST_POLL_DELAY;
            return EMU_STATE_CONNECTED__HEADSET;
        }
    }
    /*else not a headset */

    *polling_interval = EMU_POLL_CONTINUE;
    return EMU_STATE_DISCONNECTING;
}
