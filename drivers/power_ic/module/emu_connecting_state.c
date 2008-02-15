/*
 * Copyright 2004-2005 Motorola, Inc. All Rights Reserved.
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
 * @file emu_connecting_state.c
 *
 * @ingroup poweric_emu
 *
 * @brief EMU accessory detection Connecting State functions.
 *
 * The Connecting State is a transient state used when a new device has been
 * attached to the EMU bus and the software is in the process of debouncing
 * and identifying the device.
 * 
 * There are two transitions out of the Connecting State: to the Disconnected
 * State or the Connected State. In the first case, if the software has determined
 * that there is no device attached to the EMU bus after the debounce period, the
 * Disconnected State is re-entered. In the second case, if the software has
 * determined that either a valid, identified device is attached to the EMU bus or
 * an invalid or un-identifiable device is attached to the EMU bus, the Connected
 * state is entered.
 *
 */

#include <linux/sched.h>
#ifdef CONFIG_MOT_POWER_IC_PCAP2
#include <linux/ezxusbd.h>
#endif
#include "emu.h"

#include "../core/os_independent.h"

/******************************************************************************
* Local structures
******************************************************************************/
/* This structure/array is dependant on the order
   of the EMU_ID_RESISTOR_T enum */
static const struct
{
    MOTO_ACCY_TYPE_T device;
    EMU_STATE_T next_state;
}spd_id_translation_table[] =
{
    /*EMU_ID_RESISTOR_OPEN*/        { MOTO_ACCY_TYPE_CABLE_USB,        EMU_STATE_CONNECTED__POLL_SPD_REMOVAL },
    
    /*EMU_ID_RESISTOR_FACTORY*/     { MOTO_ACCY_TYPE_CABLE_FACTORY,    EMU_STATE_CONNECTED__FACTORY          },
    
    /*EMU_ID_RESISTOR_GROUND*/      { MOTO_ACCY_TYPE_INVALID,          EMU_STATE_CONNECTED__INVALID          },
    
    /*EMU_ID_RESISTOR_440K*/        { MOTO_ACCY_TYPE_CARKIT_FAST,      EMU_STATE_CONNECTING__UNPOWERED_SIHF  },
    
    /*EMU_ID_RESISTOR_200K*/        { MOTO_ACCY_TYPE_CABLE_REGRESSION, EMU_STATE_CONNECTED__POLL_SPD_REMOVAL },
    
    /*EMU_ID_RESISTOR_100K*/        { MOTO_ACCY_TYPE_CHARGER_MID_MPX,  EMU_STATE_CONNECTED__POLL_SPD_REMOVAL },
    
    /*EMU_ID_RESISTOR_10K*/         { MOTO_ACCY_TYPE_CHARGER_FAST_MPX, EMU_STATE_CONNECTED__POLL_SPD_REMOVAL },

    /*EMU_ID_RESISTOR_1K*/          { MOTO_ACCY_TYPE_INVALID,          EMU_STATE_CONNECTED__INVALID          },
  
    /*EMU_ID_RESISTOR_OPEN_SE1*/    { MOTO_ACCY_TYPE_NOT_SUPPORTED,    EMU_STATE_CONNECTED__NOT_SUPPORTED    },
    
    /*EMU_ID_RESISTOR_FACTORY_SE1*/ { MOTO_ACCY_TYPE_INVALID,          EMU_STATE_CONNECTED__INVALID          },
    
    /*EMU_ID_RESISTOR_GROUND_SE1*/  { MOTO_ACCY_TYPE_INVALID,          EMU_STATE_CONNECTED__INVALID          },
    
    /*EMU_ID_RESISTOR_440K_SE1*/    { MOTO_ACCY_TYPE_CHARGER_FAST,     EMU_STATE_CONNECTING__SPD_POLL        },
    
    /*EMU_ID_RESISTOR_200K_SE1*/    { MOTO_ACCY_TYPE_CHARGER_MID,      EMU_STATE_CONNECTING__SPD_POLL        },
    
    /*EMU_ID_RESISTOR_100K_SE1*/    { MOTO_ACCY_TYPE_INVALID,          EMU_STATE_CONNECTED__INVALID          },
    
    /*EMU_ID_RESISTOR_10K_SE1*/     { MOTO_ACCY_TYPE_INVALID,          EMU_STATE_CONNECTED__INVALID          },

    /*EMU_ID_RESISTOR_1K_SE1*/      { MOTO_ACCY_TYPE_INVALID,          EMU_STATE_CONNECTED__INVALID          }
};

/******************************************************************************
* Global functions
******************************************************************************/
/*!
 * @brief Used to configure the device for debouncing.
 *
 * When the Connecting State: debounce  is entered, it indicates
 * that one of the other states believes that an accessory may be attached to the
 * phone.  It is the job of the Connecting: Debounce state to determine if an
 * accessory is attached and then to identify the accessory.  Some accessories may
 * be able to be identified in this state (i.e., no additional steps are needed in
 * order to uniquely identify the accessory) where other accessories require
 * additional processing before they can be identified.  This additional processing
 * is handled in the other states in the Connecting family.
 *
 * The entry function is responsible for configuring the hardware in the way that
 * will allow for the debouncing of accessory insertion and to begin the process
 * of accessory identification.  Additionally, a number of debounce variables will
 * need to be reset. 
 *
 * @param prev_state the previous EMU state
 * @param polling_interval a return parameter that holds the "polling rate"
 */
void debounce_dev_type_enter(EMU_STATE_T prev_state, int *polling_interval)
{
    /* Mask ID and VBUS interrupts */
    power_ic_event_mask(EMU_INT_VBUS);
    power_ic_event_mask(EMU_INT_ID);
    
    *polling_interval = EMU_POLL_CONTINUE;
    
    emu_default_register_settings(polling_interval);
}

/*!
 * @brief Used to debounce the device type
 *
 * The debouncing of the accessory is done using the base accessory type
 * (self-powered versus phone-powered).  The purpose of the event handler is to
 * wait until the base accessory type has stabilized before continuing with
 * accessory identification.  This is accomplished by having the event handler
 * execute every 100 milliseconds until the base accessory type has not changed
 * for 300 milliseconds.
 * Once the debounce is complete, the event handler will perform the initial
 * accessory identification and transition to the appropriate next state.
 * 
 * @param polling_interval a return parameter that points to the "polling rate"
 *
 * @return next_state tells the state machine what the next EMU state is
 */
EMU_STATE_T debounce_dev_type_handler(int *polling_interval)
{
    EMU_ID_RESISTOR_T id_res;
    EMU_DEV_TYPE_T device_type;
    EMU_STATE_T next_state;
    
    /* Debounce the device type */
    if ((is_device_type_debounced(&device_type)) == FALSE)
    {
        /* Come back to this state after polling interval expires */
        *polling_interval = EMU_POLL_DFLT_DEB_DELAY;
        
        return EMU_STATE_CONNECTING__DEBOUNCE_DEV_TYPE;      
    }

    /* Finished debouncing, reset the counter */
    emu_debounce_counter = 0;
    
    /* Store the device type */
    emu_current_device_type = device_type;
    
    /* Now that the device type is debounced... */
    switch (device_type)
    {
        case EMU_DEV_TYPE_PPD:
            tracemsg(_k_d("EMU: debounce_dev_type_handler: EMU_DEV_TYPE_PPD"));
            
            /* Verify that ID res is 100k */
            if (get_id_res_value() != EMU_ID_RESISTOR_100K)
            {                
                /*if ID is not 100k, the accy is invalid */
                emu_current_device = MOTO_ACCY_TYPE_INVALID;
                next_state =  EMU_STATE_CONNECTED__INVALID;
                tracemsg(_k_d("EMU: debounce_dev_type_handler: ID != 100k"));
            }
            else
            {
                /* Go to the first PPD polling state */
                next_state = EMU_STATE_CONNECTING__PPD_VALIDATE;
            }
            break;
            
        case EMU_DEV_TYPE_SPD:
            tracemsg(_k_d("EMU: debounce_dev_type_handler: EMU_DEV_TYPE_SPD"));
            
            /* Get the resistance on ID */
            id_res = get_id_res_value();
            
            /* Is the device single ended one */
            if (get_bus_state(EMU_BUS_SIGNAL_SE1))
            {
                id_res += EMU_ID_RESISTOR_SE1;
            }

            emu_current_device = spd_id_translation_table[id_res].device;
            next_state = spd_id_translation_table[id_res].next_state;

            /* If we were previously in the unpowered SIHF state, and the
               device is still not behaving (looks like an unpowered sihf),
               set the device as invalid and transition to the connected state */
            if ((emu_prev_state == EMU_STATE_CONNECTING__UNPOWERED_SIHF) &&
                (next_state == EMU_STATE_CONNECTING__UNPOWERED_SIHF))
            {
                emu_current_device = MOTO_ACCY_TYPE_INVALID;
                next_state = EMU_STATE_CONNECTED__INVALID;
            }
            break;
            
        case EMU_DEV_TYPE_NOT_SUPPORTED:
            tracemsg(_k_d("EMU: debounce_dev_type_handler: EMU_DEV_TYPE_NOT_SUPPORTED"));
            
            emu_current_device = MOTO_ACCY_TYPE_NOT_SUPPORTED;
            next_state = EMU_STATE_CONNECTED__NOT_SUPPORTED;
            break;
            
        case EMU_DEV_TYPE_INVALID:
            tracemsg(_k_d("EMU: debounce_dev_type_handler: EMU_DEV_TYPE_INVALID"));
                
            emu_current_device = MOTO_ACCY_TYPE_INVALID;
            next_state = EMU_STATE_CONNECTED__INVALID;
            break;
            
        case EMU_DEV_TYPE_NONE:
            tracemsg(_k_d("EMU: debounce_dev_type_handler: EMU_DEV_TYPE_NONE"));
        default:
            emu_current_device = MOTO_ACCY_TYPE_NONE;
            next_state = EMU_STATE_DISCONNECTED;
            break;
    }

    tracemsg(_k_d("EMU: debounce_dev_type_handler: current_device = %d, next_state = %d"),
             emu_current_device, next_state);

    *polling_interval = EMU_POLL_CONTINUE;
    
    return next_state;
}

/*!
 * @brief Connecting State: spd_delay_enter function
 *
 * The purpose of the Connecting: Self-powered Delay state is to identify the
 * difference between a charger-type accessory and a car kit-type accessory.
 * This is done by executing the following steps:
 *   1.Disable 150k weak D+ pull-up (charger detect pull-up).
 *   2.Enable USB transceiver
 *   3.Wait 5 milliseconds
 *   4.Read state of D- line.
 * If the D- line is low, the accessory is a charger.  If the D- line is high, the
 * accessory is a car kit.
 *
 * This state is only ever entered from the Connecting: Debounce state.  The entry
 * function will perform the first two steps in the procedure listed above and
 * then set up the timer to allow the event handler to be called after the 5
 * millisecond delay.
 *
 * @param prev_state the previous EMU state
 * @param polling_interval a return parameter that holds the "polling rate"
 */
void spd_delay_enter(EMU_STATE_T prev_state, int *polling_interval)
{
    /* Configure the transceiver */
    emu_configure_usb_xcvr(EMU_XCVR_SPD_DETECT);

    /*Disable the dplus pull up */
    EMU_SET_DPLUS_150K_PULL_UP(DISABLE);
    
    /* Delay the spd poll handler state */
    *polling_interval = EMU_POLL_SETTLE_DELAY;
}

/*!
 * @brief Connecting State: spd_delay_handler function
 *
 * The event handler for this state will only be called after the 5 millisecond
 * delay and is responsible for checking the state of the D- line.
 *
 * @param polling_interval a return parameter that holds the "polling rate"
 *
 * @return next_state tells the state machine what the next EMU state is
 */
EMU_STATE_T spd_delay_handler(int *polling_interval)
{
    int vbus;
    EMU_STATE_T next_state = EMU_STATE_CONNECTED__POLL_SPD_REMOVAL;
        
    /* Verify that the device type didn't change while polling */
    if (get_device_type() != EMU_DEV_TYPE_SPD)
    {
        tracemsg(_k_d("EMU: spd_delay_handler:device type changed while polling!"));

        *polling_interval = EMU_POLL_CONTINUE;
        
        /* Something changed debounce the accy again */
        return EMU_STATE_CONNECTING__DEBOUNCE_DEV_TYPE;
    }
    
    /* If dminus is high then this charger is a SIHF */
    if (get_bus_state(EMU_BUS_SIGNAL_DMINUS))
    {
        if (emu_current_device == MOTO_ACCY_TYPE_CHARGER_FAST)
        {
            emu_current_device = MOTO_ACCY_TYPE_CARKIT_FAST;
        }
        else
        {
            emu_current_device = MOTO_ACCY_TYPE_CARKIT_MID;
        }
    }
    else
    {
        if (emu_current_device == MOTO_ACCY_TYPE_CHARGER_FAST)
        {
            /* Convert the Batt+ channel to get the VBUS voltage */
            if (power_ic_atod_single_channel(EMU_A2D_VBUS, &vbus) != 0)
            {
                vbus = 0;
            }
            
            if (vbus >= EMU_VBUS_5VOLTS)
            {
                /*current_device is already set correctly */
                /*current_device = MOTO_ACCY_TYPE_CHARGER_FAST;*/
            }
            else if (vbus >= EMU_VBUS_4_5VOLTS)
            {
                emu_current_device = MOTO_ACCY_TYPE_CHARGER_FAST_3G;
            }
            else /*if (vbus < VBUS_4_5VOLTS)*/
            {
                emu_current_device = MOTO_ACCY_TYPE_INVALID;
                next_state = EMU_STATE_CONNECTED__INVALID;
            }
        }
        /*else if (emu_current_device == MOTO_ACCY_TYPE_CHARGER_MID) */
        /* Nothing to do here for Mid rate chargers */
    }

    tracemsg(_k_d("EMU: current_device = %d"),
             emu_current_device);

    *polling_interval = EMU_POLL_CONTINUE;
    
    /* Go to the determined connected substate */
    return next_state;
}

/*!
 * @brief Used to configure the interrupt lines and sets the polling interval
 *        for detection of a SIHF
 *
 * This state is used handle the special case of the insertion of a SIHF that is
 * not functioning correctly due to the fact that it was plugged into the phone
 * before power was applied to the SIHF.  The SIHF accessories are responsible for
 * driving the D- line high, causing the phone to see a SE1 when the accessory is
 * attached.  However, when power is first applied to the accessory, it may take
 * up to a few seconds before the D- line rises to a level where the SE1 condition
 * is detected by the phone.  If the accessory identification starts before the
 * SE1 is seen, the SIHF accessory will appear to be an invalid accessory.  This
 * state is used to allow for this poorly behaving accessory to be detected
 * correctly by waiting for the SE1 condition before proceeding with the accessory
 * identification.
 *
 * The entry function will configure the timer to expire after 5 seconds.
 * This will allow the SIHF to stabilize the bus.
 *
 * @param prev_state the previous EMU state
 * @param polling_interval a return parameter that holds the "polling rate"
 *        
 */
void unpowered_sihf_enter(EMU_STATE_T prev_state, int *polling_interval)
{
    /* Wait for any EMU bus change  */
    power_ic_event_unmask(EMU_INT_SE1);
    power_ic_event_unmask(EMU_INT_VBUS);
    power_ic_event_unmask(EMU_INT_ID);
    
    /* Wait for the SIHF delay to time out or for a change in ID, VBUS or SE1 */
    *polling_interval = EMU_POLL_UNPOWERED_SIHF_DELAY;
}

/*!
 * @brief Used to mask the EMU bus interrupts and always transitions directly
 *        to the Connecting State: debounce.
 *
 * The handler will be called if an ID, VBUS or SE1 interrupt occur, or if the
 * unpowered SIHF delay expires. If one of the interrupts is seen or the unpowered
 * SIHF delay expires, the state will be change to the Connecting State: debounce
 * in order to complete the identification of the accessory.
 *
 * @param polling_interval a return parameter that holds the "polling rate"
 *
 * @return next_state tells the state machine what the next EMU state is
 */
EMU_STATE_T unpowered_sihf_handler(int *polling_interval)
{
    /* Mask the interrupts */
    power_ic_event_mask(EMU_INT_SE1);
    power_ic_event_mask(EMU_INT_VBUS);
    power_ic_event_mask(EMU_INT_ID);

    *polling_interval = EMU_POLL_CONTINUE;

    /* Always go back to debouncing */
    return EMU_STATE_CONNECTING__DEBOUNCE_DEV_TYPE;
}


/*!
 * @brief Configures the transceiver and lets the hardware stabilize.
 *
 * The Connecting: Phone-powered Verification Delay state is used to verify that
 * the connected phone-powered device is a valid phone-powered device.  This state
 * is executed prior to enabling power to the phone-powered accessory.  Enabling
 * power to an accessory could cause problems unless the accessory is verified to
 * be a real phone-powered accessory.
 * The steps to verify that an accessory is a real phone-powered accessory are:
 *  1.Enable the USB transceiver
 *  2.Wait 5 milliseconds
 *  3.Read the state of the D+ line
 * A real EMU phone-powered accessory will keep the D+ line grounded until power
 * is applied to it.  So, if the D+ line is high, the accessory is an invalid
 * accessory.  If the D+ line is low, the accessory is a real phone-powered
 * accessory.
 *
 * The entry function for the state will enable the USB transceiver and configure
 * the timer to expire after 5 milliseconds.  This will cause the event handler
 * function to be called after this delay to check the state of the D+ line.
 *
 * @param prev_state        the previous EMU state
 * @param polling_interval  a return parameter that holds the "polling rate"
 */
void ppd_validate_enter(EMU_STATE_T prev_state, int *polling_interval)
{
    /* configure the USB transciever */
    emu_configure_usb_xcvr(EMU_XCVR_PPD_DETECT);

    /* set timer */
    *polling_interval = EMU_POLL_SETTLE_DELAY;
}

/*!
 * @brief Validates that the PPD is a supported device
 *
 * The event handler for this state will be called after a delay to ensure that
 * the output of the USB transceiver has stabilized.  If the D+ line is low, the
 * accessory is a valid phone-powered accessory, so the state is changed to the
 * Connecting State: PPD validate state to identify the accessory.  If the D+ line
 * is high, the accessory is invalid, so the state is changed to the Connected
 * State indicating the invalid accessory.  As a separate verification step, the
 * event handler will also verify that the accessory still appears to be a
 * phone-powered accessory (as opposed to a self-powered accessory).  If the
 * accessory does not appear to be a phone-powered accessory, the state is reset
 * to the Connecting: debounce_dev_type state to attempt to re-debounce the
 * insertion of the accessory.
 *
 * @param polling_interval a return parameter that holds the "polling rate"
 *
 * @return next_state tells the state machine what the next EMU state is
 */
EMU_STATE_T ppd_validate_handler(int *polling_interval)
{
    if (get_device_type() != EMU_DEV_TYPE_PPD)
    {
        tracemsg(_k_d("EMU: ppd_validate_handler: device_type changed! next_state = EMU_STATE_CONNECTING__DEBOUNCE_DEV_TYPE"));
        
        /* Something changed, debounce the device type again */
        *polling_interval = EMU_POLL_CONTINUE;
        
        return EMU_STATE_CONNECTING__DEBOUNCE_DEV_TYPE;
    }

    if (!get_bus_state(EMU_BUS_SIGNAL_DPLUS))
    {
        tracemsg(_k_d("EMU: ppd_validate_handler: D+ is low, next_state = EMU_STATE_CONNECTING__PPD_IDENTIFY"));
        
        /* If D+ is low and the device type has not changed
           go on to the next layer of PPD detection */
        *polling_interval = EMU_POLL_CONTINUE;
        
        return EMU_STATE_CONNECTING__PRE_PPD_IDENTIFY;
    }

    tracemsg(_k_d("EMU: ppd_validate_handler : D+ is high, current_device = MOTO_ACCY_TYPE_INVALID, next_state = EMU_STATE_CONNECTED__INVALID"));
    
    /* D+ is not low at this point so the device is invalid */
    emu_current_device = MOTO_ACCY_TYPE_INVALID;

    *polling_interval = EMU_POLL_CONTINUE;
    
    return EMU_STATE_CONNECTED__INVALID;
}

/*!
 * @brief Enables power to the PPD accessory, then wait for the VBUS line to settle
 *
 * The purpose of this state is to identify the type of phone-powered accessory
 * attached to the phone.  This identification is performed using the following
 * steps:
 *   1.Enable power to the accessory
 *   2.Wait 200 milliseconds.
 *   3.Read the state of the D+ and D- lines.
 * The state of the D+ and D- lines indicates the type of accessory that is
 * connected based on the following table:
 *
 *   D+ D- Accessory       
 *   0  0  Unknown     
 *   0  1  Unknown     
 *   1  0  EMU mono headset    
 *   1  1  Unknown   
 * Table 4: Phone-powered accessory identification
 *
 * The entry function for the state will enable the power to the accessory and
 * configure the timer to expire after 200 milliseconds.  This will cause the
 * event handler function to be called after this delay to check the state of the
 * D+ and D- lines.
 *
 * @param prev_state the previous EMU state
 * @param polling_interval a return parameter that holds the "polling rate"
 */
void pre_ppd_identify_enter(EMU_STATE_T prev_state, int *polling_interval)
{
    tracemsg(_k_d("EMU: pre_ppd_identify_enter: EMU_DEV_TYPE_PPD"));
     /* Power VBUS */
    EMU_SET_REVERSE_MODE(ENABLE);
    
    /* Set timer to wait for D+ and D- lines to stabilize */
    *polling_interval = EMU_POLL_REV_MODE_DELAY;
}

/*!
 * @brief Used to determine what type of PPD this is initially.
 *
 * The event handler function reads the state of the D+ and D- lines to determine
 * what type of accessory is connected. The state is then changed to the
 * Connected state along with an indication of the identification of the accessory.
 *
 * @param polling_interval a return parameter that holds the "polling rate"
 *
 * @return next_state tells the state machine what the next EMU state is
 */
EMU_STATE_T pre_ppd_identify_handler(int *polling_interval)
{
    *polling_interval = EMU_POLL_CONTINUE;
    switch (get_bus_state(EMU_BUS_SIGNAL_DP_DM))
    {
        case EMU_BUS_SIGNAL_STATE_DP_DM_10:   

            tracemsg(_k_d("Set the GPIO for ST headset"));
            EMU_SET_HEADSET_PULL_UP(ENABLE);
            return EMU_STATE_CONNECTING__PPD_IDENTIFY;
            break;
              
        case EMU_BUS_SIGNAL_STATE_DP_DM_00:
        case EMU_BUS_SIGNAL_STATE_DP_DM_01:
        case EMU_BUS_SIGNAL_STATE_DP_DM_11:
        default:
            emu_current_device = MOTO_ACCY_TYPE_NOT_SUPPORTED;
            return EMU_STATE_CONNECTED__NOT_SUPPORTED;
            break;
    }
}
/*!
 * @brief Wait for ID line settle down
 *
 * The purpose of this state is to identify the type of phone-powered accessory
 * attached to the phone.  This identification is performed using the following
 * steps:
 *   1.Wait 200 milliseconds.
 *   2.Read the state of the D+ and D- lines.
 * The state of the D+ and D- lines indicates the type of accessory that is
 * connected based on the following table:
 *
 *   D+ D- Accessory       
 *   0  0  EMU stereo headset    
 *   0  1  Unknown     
 *   1  0  EMU mono headset    
 *   1  1  Unknown   
 *
 * The entry function for the state will configure the timer to expire after 200 milliseconds.
 * This will cause the event handler function to be called after this delay to check the state
 * of the D+ and D- lines.
 *
 * @param prev_state the previous EMU state
 * @param polling_interval a return parameter that holds the "polling rate"
 */
void ppd_identify_enter(EMU_STATE_T prev_state, int *polling_interval)
{
    /* Set timer to wait for D+ and D- lines to stabilize */
    *polling_interval = EMU_POLL_HEADSET_DELAY;
}

/*!
 * @brief Used to determine what type of PPD this is.
 *
 * The event handler function reads the state of the D+ and D- lines to determine
 * what type of accessory is connected. The state is then changed to the
 * Connected state along with an indication of the identification of the accessory.
 *
 * @param polling_interval a return parameter that holds the "polling rate"
 *
 * @return next_state tells the state machine what the next EMU state is
 */
EMU_STATE_T ppd_identify_handler(int *polling_interval)
{
    *polling_interval = EMU_POLL_CONTINUE;
    
    switch (get_bus_state(EMU_BUS_SIGNAL_DP_DM))
    {
        case EMU_BUS_SIGNAL_STATE_DP_DM_10:
            emu_current_device = MOTO_ACCY_TYPE_HEADSET_EMU_MONO;
            return EMU_STATE_CONNECTED__HEADSET;
            break;
            
        case EMU_BUS_SIGNAL_STATE_DP_DM_00:
            tracemsg(_k_d("The headset is detected as EMU ST headset"));
            emu_current_device = MOTO_ACCY_TYPE_HEADSET_EMU_STEREO;
            return EMU_STATE_CONNECTED__HEADSET;
            break;
            
        case EMU_BUS_SIGNAL_STATE_DP_DM_01:
        case EMU_BUS_SIGNAL_STATE_DP_DM_11:
        default:
            emu_current_device = MOTO_ACCY_TYPE_NOT_SUPPORTED;
            return EMU_STATE_CONNECTED__NOT_SUPPORTED;
            break;
    }
}

