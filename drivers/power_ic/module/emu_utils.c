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
 * Motorola 2005-May-12 - Cleaned up the VBUS structure with new setting
 * Motorola 2005-Jun-09 - Added the set headset mode function
 * Motorola 2005-Dec-13 - Finalize the software.
 *
 */

/*!
 * @file emu_utils.c
 *
 * @ingroup poweric_emu
 *
 * @brief EMU state machine utility functions
 *
 * This file defines the functions that are needed to assist the
 * EMU state functions in determining the state of the EMU bus.
 *
 */

#include <linux/power_ic.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/moto_accy.h>
#include "atod.h"
#include "emu.h"

#include "../core/os_independent.h"

/* Initialize global variable */
int emu_debounce_counter = 0;
BOOL emu_audio_update = FALSE;
MOTO_ACCY_HEADSET_MODE_T emu_audio_mode = MOTO_ACCY_HEADSET_MODE_NONE;
/******************************************************************************
* Global functions
******************************************************************************/
/* Hardware specific inline function */
/*!
 * @brief configure_usb_xcvr
 *
 * This utility function is used to configure the transceiver, which includes
 * enabling/disabling the transceiver, setting the VUSB input source, setting the
 * VUSB output voltage and setting the state of the VUSB regulator
 *
 * @param state - what state the transceiver should be set to
 */
void emu_configure_usb_xcvr(EMU_XCVR_T state)
{
    static const struct
    {
        BOOL usbsuspend;
        BOOL usbxcvren;
        BOOL vusben;
        EMU_VREG_IN_T input_source;
        EMU_VREG_OUT_T voltage;
    } vusb_states[EMU_XCVR__NUM] =
    {
        /* EMU_XCVR_OFF */          { TRUE,  FALSE, TRUE,  EMU_VREG_IN_BPLUS,  EMU_VREG_OUT_3_3V   },
        
        /* EMU_XCVR_PPD_DETECT */   { TRUE,  TRUE,  TRUE,  EMU_VREG_IN_VINBUS, EMU_VREG_OUT_3_3V   },

        /* EMU_XCVR_SPD_DETECT */   { TRUE,  TRUE,  TRUE,  EMU_VREG_IN_VBUS,   EMU_VREG_OUT_3_3V   },

        /* EMU_XCVR_PPD */          { TRUE,  TRUE,  TRUE,  EMU_VREG_IN_BPLUS,  EMU_VREG_OUT_3_3V   },

        /* EMU_XCVR_PPD_AUDIO */    { TRUE,  TRUE,  TRUE,  EMU_VREG_IN_BPLUS,  EMU_VREG_OUT_3_3V   },

        /* EMU_XCVR_SPD */          { TRUE,  TRUE,  TRUE,  EMU_VREG_IN_VBUS,   EMU_VREG_OUT_3_3V   },

        /* EMU_XCVR_SPD_AUDIO */    { TRUE,  TRUE,  TRUE,  EMU_VREG_IN_VBUS,   EMU_VREG_OUT_3_3V   },

        /* EMU_XCVR_USB_HOST */     { FALSE, TRUE,  TRUE,  EMU_VREG_IN_VINBUS, EMU_VREG_OUT_3_3V   },

        /* EMU_XCVR_FACTORY_MODE */ { FALSE, TRUE,  TRUE,  EMU_VREG_IN_VINBUS, EMU_VREG_OUT_3_3V   },
    };

    /* If the state is out of range, assume the off state */
    if (state >= EMU_XCVR__NUM)
    {
        state = EMU_XCVR_OFF;
    }

    /* Set the USB suspend bit */
    EMU_SET_USB_SUSPEND(vusb_states[state].usbsuspend);
    
    /* Set the VUSB input source */
    EMU_SET_VUSB_INPUT_SOURCE(vusb_states[state].input_source);
    
    /* Set VUSB output voltage */
    EMU_SET_VUSB_OUTPUT_VOLTAGE(vusb_states[state].voltage);
    
    /* Set the state of the VUSB regulator */
    EMU_SET_VUSB_REGULATOR_STATE(vusb_states[state].vusben);
    
    /* Set the state of the transceiver */
    EMU_SET_TRANSCEIVER_STATE(vusb_states[state].usbxcvren);
}

/*!
 * @brief Used to debounce the device type.
 *
 * This utility function helps the state functions debounce the device type. 
 * 
 * @param device_type - Used to return the current device type (SPD, PPD).
 *
 * @return TRUE if the device type is debounced,
 *         FALSE if the device type is not debounced
 */
BOOL is_device_type_debounced(EMU_DEV_TYPE_T *device_type)
{
    static EMU_DEV_TYPE_T prev_device_type = EMU_DEV_TYPE_NONE;
    EMU_DEV_TYPE_T curr_device_type = get_device_type();
    
    if (device_type != NULL)
    {
        /* Set device type if it is wanted */
        *device_type = curr_device_type;
    }
    
    if (prev_device_type == curr_device_type)
    {
        emu_debounce_counter = emu_debounce_counter + 1;
    }
    else
    {
        emu_debounce_counter = 1;
        prev_device_type = curr_device_type;
    }
    
    if (emu_debounce_counter >= EMU_POLL_DEB_DONE_CNT)
    {
        /* The device type is considered debounced,
           reset prev_device_type */
        prev_device_type = EMU_DEV_TYPE_NONE;

        /* The counter is reset in the state machine when transitioning from
           the current state to a different state. If the next state is the
           same as the current state, then the counter must be cleared by the
           state function */
        
        tracemsg(_k_d("EMU: is_device_type_debounced: device_type is debounced"));
        return TRUE;
    }

    tracemsg(_k_d("EMU: is_device_type_debounced: device_type is not debounced"));
    return FALSE;
}

/*!
 * @brief This utility funciton get the device type.
 *
 * If VBUS4V4 is set the device is an SPD, otherwise the CHRG_CURR
 * sense bit, USB2V0 sense bit, ID_FLOAT sense bit and the previous
 * device type are used to determine the current device type.
 *
 * @return The device type
 */
EMU_DEV_TYPE_T get_device_type(void)
{
    EMU_DEV_TYPE_T dev;
    BOOL idfloat = FALSE;

    /* This table must match the EMU_DEV_TYPE_T enum */
    static const char* emu_dev_type_name_tbl[EMU_DEV_TYPE__NUM] =
    {
        "EMU_DEV_TYPE_NONE",
        "EMU_DEV_TYPE_SPD",
        "EMU_DEV_TYPE_PPD",
        "EMU_DEV_TYPE_NOT_SUPPORTED",
        "EMU_DEV_TYPE_INVALID"
    };
    
    /* Update the VBUS det global */
    emu_vbus_det_state = get_bus_state(EMU_BUS_SIGNAL_VBUS);

    /* If VBUS is at or above 4.4 volts then there is no doubt that this is an SPD */
    if (emu_vbus_det_state == EMU_BUS_SIGNAL_STATE_VBUS4V4)
    {
        /* USB4V4 = 1 */
        dev = EMU_DEV_TYPE_SPD;
    }
    /* If VBUS is below 4.4 volts and the CHRGCURR sense bit is 0, then
       it is not possible for a device to be connected.
       The follwing table describe how the CHRGCURR sense bit behaves.
       
       I>20mA  chrgreg | CHRG_CURR sense
      -----------------+-----------------
         0         0   |  1 
         0         1   |  0 only happens if nothing is connected
         1         X   |  1
    */
    else if (!power_ic_event_sense_read(EMU_INT_CHRG_CURR))
    {
        /* USB4V4 = 0, CHRGCURR = 0 */
        dev = EMU_DEV_TYPE_NONE;
    }
    else
    {
        /* USB4V4 = 0, CHRGCURR = 1 */
        idfloat = get_bus_state(EMU_BUS_SIGNAL_ID_FLOAT);

        /* We need to use the previously detected device type to determine the new
           device type from here */
        switch (emu_current_device_type)
        {
            case EMU_DEV_TYPE_NONE:
            case EMU_DEV_TYPE_PPD:
                dev = (idfloat ? EMU_DEV_TYPE_NONE : EMU_DEV_TYPE_PPD);
                break;
                
            case EMU_DEV_TYPE_SPD:
                /* The charger is collapsed so report the dev type as SPD */
                if (emu_vbus_det_state == EMU_BUS_SIGNAL_STATE_VBUS2V0)
                {
                    dev = EMU_DEV_TYPE_SPD;
                }
                else if (idfloat)
                {
                    /* USB4V4 = 0, CHRGCURR = 1, USB2V0 = 0 IDFLOAT = 1 */
                    dev = EMU_DEV_TYPE_NONE;
                }
                else
                {
                    dev = EMU_DEV_TYPE_PPD;
                }
                break;
                
            default:
                dev = EMU_DEV_TYPE_INVALID;
                break;
        }
    }

    tracemsg(_k_d("EMU: get_device_type: %s"), emu_dev_type_name_tbl[dev]);

    return dev;
}

/*!
 * @brief Used to debounce the bus state.
 *
 * This utility function helps the state functions debounce the bus state. 
 * 
 * @param signal    - The bus signal that needs to be debounced.
 * @param bus_state - Used to return the current state of the selected bus signal
 *        counter   - Holds the current number of debounces that have occured. This
 *                    value is incremented in this function, but it is the job of
 *                    the calling function to initialize, clear and
 *                    remember (static) the counter
 * @param counter_limit - Specifies the number of time to debounce the bus signal.
 *
 * @return TRUE if the selected bus signal is debounced,
 *         FALSE if the selected bus signal is not debounced
 */
BOOL is_bus_state_debounced(EMU_BUS_SIGNAL_T signal,
                            EMU_BUS_SIGNAL_STATE_T *bus_state,
                            int counter_limit)
{
    static EMU_BUS_SIGNAL_STATE_T prev_bus_state = EMU_BUS_SIGNAL_STATE_UNKNOWN;
    EMU_BUS_SIGNAL_STATE_T curr_bus_state = get_bus_state(signal);

    if (bus_state != NULL)
    {
        *bus_state = curr_bus_state;
    }
    
    if (prev_bus_state == curr_bus_state)
    {
        emu_debounce_counter = emu_debounce_counter + 1;
    }
    else
    {
        emu_debounce_counter = 1;
        
        prev_bus_state = curr_bus_state;
    }
    
    if (emu_debounce_counter >= counter_limit)
    {
        /* If the bus state is considered debounced
           then reset prev_bus_state */
        prev_bus_state = EMU_BUS_SIGNAL_STATE_UNKNOWN;

        /* The counter is reset in the state machine when transitioning from
           the current state to a different state. If the next state is the
           same as the current state, then the counter must be cleared by the
           state function */
        
        tracemsg(_k_d("EMU: is_bus_state_debounced: bus_state is debounced"));
        return TRUE;
    }
    
    tracemsg(_k_d("EMU: is_bus_state_debounced: bus_state is not debounced, "));
    return FALSE;
}

/*!
 * @brief Returns the resistor value of the ID line
 *
 * This utility function gets the resistor value of the ID line. If ID is
 * floating or if ID sense bits are both set (factory mode) the A2D
 * measurement is not used to determine the resistor value. Otherwise, an
 * A2D measurement is made on the ID line. This A2D measurement is then compared
 * to the threasholds defined for the current hardware configuration to determine
 * what resistor value to return.
 *
 * @return One of the resistor values defined in EMU_ID_RESISTOR_T
 */
EMU_ID_RESISTOR_T get_id_res_value(void)
{
    EMU_ID_COUNTS_T id_atod_value;
    EMU_ID_RESISTOR_T result = EMU_ID_RESISTOR_GROUND;
    int i = 0;

    static const char* emu_id_resistor_name_tbl[EMU_ID_RESISTOR__NUM] =
    {
        "EMU_ID_RESISTOR_OPEN",
        "EMU_ID_RESISTOR_FACTORY",
        "EMU_ID_RESISTOR_GROUND",
        "EMU_ID_RESISTOR_440K",
        "EMU_ID_RESISTOR_200K",
        "EMU_ID_RESISTOR_100K",
        "EMU_ID_RESISTOR_10K",
        "EMU_ID_RESISTOR_1K",   
        "EMU_ID_RESISTOR_OPEN_SE1", 
        "EMU_ID_RESISTOR_FACTORY_SE1",
        "EMU_ID_RESISTOR_GROUND_SE1",
        "EMU_ID_RESISTOR_440K_SE1",
        "EMU_ID_RESISTOR_200K_SE1",
        "EMU_ID_RESISTOR_100K_SE1",
        "EMU_ID_RESISTOR_10K_SE1",
        "EMU_ID_RESISTOR_1K_SE1"
    };
    
    static const struct
    {
        EMU_ID_COUNTS_T threshold;
        EMU_ID_RESISTOR_T res_value;
    } sig_state_to_id_res_trans_tbl[EMU_ID_COUNTS__NUM] =
    {
        { EMU_ID_COUNTS_OPEN_MIN, EMU_ID_RESISTOR_OPEN },
        { EMU_ID_COUNTS_440K_MIN, EMU_ID_RESISTOR_440K },
        { EMU_ID_COUNTS_200K_MIN, EMU_ID_RESISTOR_200K },
        { EMU_ID_COUNTS_100K_MIN, EMU_ID_RESISTOR_100K },
        { EMU_ID_COUNTS_10K_MIN,  EMU_ID_RESISTOR_10K  },
        { EMU_ID_COUNTS_1K_MIN,   EMU_ID_RESISTOR_1K   }
    };

    if (get_bus_state(EMU_BUS_SIGNAL_ID) == EMU_BUS_SIGNAL_STATE_ID_FACTORY)
    {
        result = EMU_ID_RESISTOR_FACTORY;
    }
    else if (get_bus_state(EMU_BUS_SIGNAL_ID_FLOAT) == EMU_BUS_SIGNAL_STATE_ENABLED)
    {
        result = EMU_ID_RESISTOR_OPEN;
    }
    else
    {
        /* Enable the 5uA current source before taking
           the A2D measurement on ID */
        EMU_SET_ID_PULL_UP_CONTROL(ENABLE);
        
        /* Get A2D reading on the ID line */
        if (power_ic_atod_single_channel(EMU_A2D_ID, &id_atod_value) != 0)
        {
            id_atod_value = 0;
        }
    
        tracemsg(_k_d("EMU: get_id_res_value: ID A2D = 0x%03X"), id_atod_value);

        /* The 5uA current source is only used when measuring ID values
           that are >= 102K */
        EMU_SET_ID_PULL_UP_CONTROL(DISABLE);
        
        /* If the measurement on ID is below the EMU_ID_COUNTS_102K_MIN
           disable the ID current source and re-convert the ID line */
        if (id_atod_value < EMU_ID_COUNTS_100K_MIN)
        {
            tracemsg(_k_d("EMU: get_id_res_value: ID < 102K"));

            /* Get A2D reading on the ID line again */
            if (power_ic_atod_single_channel(EMU_A2D_ID, &id_atod_value) != 0)
            {
                id_atod_value = 0;
            }
        
            tracemsg(_k_d("EMU: get_id_res_value: ID A2D = 0x%03X"), id_atod_value);
        }

        /* Get the resistor value based on the a2d measurement. If the a2d measurement
           is less than the 1K min value then the resistor value is considered grounded*/
        while (i < EMU_ID_COUNTS__NUM)
        {
            if (id_atod_value >= sig_state_to_id_res_trans_tbl[i].threshold)
            {
                result = sig_state_to_id_res_trans_tbl[i].res_value;
                break;
            }
            i++;
        }
    }

    tracemsg(_k_d("EMU: get_id_res_value: id resistor value = %s"),
             emu_id_resistor_name_tbl[result]);

    return result;
}

/*!
 * @brief This utility function gets the state of the passed in EMU bus signal.
 *
 * @param signal - Specifies which bus signal's state is needed.
 *
 * @return The state of the selected EMU bus signal.
 */
EMU_BUS_SIGNAL_STATE_T get_bus_state(EMU_BUS_SIGNAL_T signal)
{
    EMU_BUS_SIGNAL_STATE_T id_float = EMU_BUS_SIGNAL_STATE_DISABLED;
    EMU_BUS_SIGNAL_STATE_T id_gnd = EMU_BUS_SIGNAL_STATE_DISABLED;
    EMU_BUS_SIGNAL_STATE_T result;

    /* This table must match the EMU_BUS_SIGNAL_STATE_T enum */
    static const char* emu_bus_signal_state_name_tbl[EMU_BUS_SIGNAL_STATE__NUM] =
    {
        "EMU_BUS_SIGNAL_STATE_DISABLED",
        "EMU_BUS_SIGNAL_STATE_ENABLED",
        "EMU_BUS_SIGNAL_STATE_UNKNOWN",
        "EMU_BUS_SIGNAL_STATE_ID_FLOATING",
        "EMU_BUS_SIGNAL_STATE_ID_GROUNDED", 
        "EMU_BUS_SIGNAL_STATE_ID_RESISTOR",
        "EMU_BUS_SIGNAL_STATE_ID_FACTORY",
        "EMU_BUS_SIGNAL_STATE_DP_DM_00",
        "EMU_BUS_SIGNAL_STATE_DP_DM_01",
        "EMU_BUS_SIGNAL_STATE_DP_DM_10",
        "EMU_BUS_SIGNAL_STATE_DP_DM_11",
        "EMU_BUS_SIGNAL_STATE_VBUS0V8",
        "EMU_BUS_SIGNAL_STATE_VBUS2V0",
        "EMU_BUS_SIGNAL_STATE_VBUS4V4",
    };

    /* This table must match the EMU_BUS_SIGNAL_T enum */
    static const char* emu_bus_signal_name_tbl[EMU_BUS_SIGNAL__NUM] =
    {
        "EMU_BUS_SIGNAL_DP_DM",
        "EMU_BUS_SIGNAL_DPLUS",
        "EMU_BUS_SIGNAL_DMINUS",
        "EMU_BUS_SIGNAL_SE1",
        "EMU_BUS_SIGNAL_VBUS",
        "EMU_BUS_SIGNAL_ID",
        "EMU_BUS_SIGNAL_ID_FLOAT",
        "EMU_BUS_SIGNAL_ID_GROUND"
    };
    
    static const EMU_BUS_SIGNAL_STATE_T dp_dm_table[2][2] =
        {{EMU_BUS_SIGNAL_STATE_DP_DM_00, EMU_BUS_SIGNAL_STATE_DP_DM_01},
         {EMU_BUS_SIGNAL_STATE_DP_DM_10, EMU_BUS_SIGNAL_STATE_DP_DM_11}};

    static const EMU_BUS_SIGNAL_STATE_T idf_idg_table[2][2] =
        {{EMU_BUS_SIGNAL_STATE_ID_RESISTOR, EMU_BUS_SIGNAL_STATE_ID_GROUNDED},
         {EMU_BUS_SIGNAL_STATE_ID_FLOATING, EMU_BUS_SIGNAL_STATE_ID_FACTORY}};

    static const EMU_BUS_SIGNAL_STATE_T vbus_table[8] =
        {EMU_BUS_SIGNAL_STATE_DISABLED, /* if VBUS < 0.8V, VBUS is considered disabled */
         EMU_BUS_SIGNAL_STATE_UNKNOWN,  /* not possible */
         EMU_BUS_SIGNAL_STATE_UNKNOWN,  /* not possible */
         EMU_BUS_SIGNAL_STATE_UNKNOWN,  /* not possible */
         EMU_BUS_SIGNAL_STATE_DISABLED, /* if VBUS < 2.0V, VBUS is considered disabled */
         EMU_BUS_SIGNAL_STATE_UNKNOWN,  /* not possible */
         EMU_BUS_SIGNAL_STATE_VBUS2V0,  /* if 2.0V <= VBUS < 4.4V */
         EMU_BUS_SIGNAL_STATE_VBUS4V4}; /* if 4.4V <= VBUS < 7.0V */
    
    switch (signal)
    {
        case EMU_BUS_SIGNAL_DP_DM:
            result = dp_dm_table[EMU_GET_D_PLUS()][EMU_GET_D_MINUS()];
            break;
            
        case EMU_BUS_SIGNAL_DPLUS:
            result = EMU_GET_D_PLUS();
            break;
            
        case EMU_BUS_SIGNAL_DMINUS:
            result = EMU_GET_D_MINUS();
            break;
            
        case EMU_BUS_SIGNAL_SE1:
            power_ic_event_clear(EMU_INT_SE1);
#ifdef CONFIG_CPU_BULVERDE            
            if (power_ic_event_sense_read(EMU_INT_SE1) == 0)
            {
                /* The SE1 sense bit will always be zero if
                   the voltage on VBUS drops below 3.4 volts,
                   so use D+ and D- to verify the SE1 sense */
                if (EMU_GET_D_PLUS() && EMU_GET_D_MINUS())
                {
                    result = EMU_BUS_SIGNAL_STATE_ENABLED;
                }
                else
                {
                    result = EMU_BUS_SIGNAL_STATE_DISABLED;
                }
            }
            else
            {
                result = EMU_BUS_SIGNAL_STATE_ENABLED;
            }
#elif defined (CONFIG_ARCH_SCMA11)
            result = power_ic_event_sense_read(EMU_INT_SE1);
#endif
            break;
            
        case EMU_BUS_SIGNAL_VBUS:
            power_ic_event_clear(EMU_INT_VBUS);
            
            result = vbus_table[EMU_GET_VBUS_SENSE()];
            break;
            
        case EMU_BUS_SIGNAL_ID:
            power_ic_event_clear(EMU_SENSE_ID_FLOAT);
            
            id_float = power_ic_event_sense_read(EMU_SENSE_ID_FLOAT) ?
                EMU_BUS_SIGNAL_STATE_ENABLED : EMU_BUS_SIGNAL_STATE_DISABLED;
            
            id_gnd = power_ic_event_sense_read(EMU_SENSE_ID_GND) ?
                EMU_BUS_SIGNAL_STATE_ENABLED : EMU_BUS_SIGNAL_STATE_DISABLED;
            
            result = idf_idg_table[id_float][id_gnd];
            break;

        case EMU_BUS_SIGNAL_ID_FLOAT:
            power_ic_event_clear(EMU_SENSE_ID_FLOAT);
            
            result = power_ic_event_sense_read(EMU_SENSE_ID_FLOAT) ?
                EMU_BUS_SIGNAL_STATE_ENABLED : EMU_BUS_SIGNAL_STATE_DISABLED;
            break;
            
        case EMU_BUS_SIGNAL_ID_GROUND:
            power_ic_event_clear(EMU_SENSE_ID_FLOAT);
            
            result = power_ic_event_sense_read(EMU_SENSE_ID_GND) ?
                EMU_BUS_SIGNAL_STATE_ENABLED : EMU_BUS_SIGNAL_STATE_DISABLED;
            break;
            
        default:
            result = EMU_BUS_SIGNAL_STATE_UNKNOWN;
            break;
    }

    tracemsg(_k_d("EMU: get_bus_state : %s = %s"),
             emu_bus_signal_name_tbl[signal],
             emu_bus_signal_state_name_tbl[result]);
    
    return result;
}

/*!
 * @brief This utility function sets the audio mode in Atlas and stereo emu headset
 *        pull up.
 *
 * @param mode - The headset mode
 *
 */
void emu_util_set_emu_headset_mode(MOTO_ACCY_HEADSET_MODE_T mode)
{
    if(emu_state == EMU_STATE_CONNECTED__HEADSET)
    {
        switch(mode)
        {   
            case MOTO_ACCY_HEADSET_MODE_NONE:
                EMU_SET_REVERSE_MODE(FALSE);
                mdelay(10);
                EMU_SET_EMU_CONN_MODE(EMU_CONN_MODE_USB);
                EMU_SET_HEADSET_PULL_UP(DISABLE);
                /*Following is the work around for the emu headset send/end key*/
                EMU_SET_HS_SEND_END_REGS(ENABLE);
                EMU_SET_VUSB_INPUT_SOURCE(DISABLE);
                break;
        
            case MOTO_ACCY_HEADSET_MODE_MONO:
                EMU_SET_VUSB_INPUT_SOURCE(EMU_VREG_IN_BPLUS);
                EMU_SET_HS_SEND_END_REGS(DISABLE);
                EMU_SET_EMU_CONN_MODE(EMU_CONN_MODE_MONO_AUDIO);
                EMU_SET_HEADSET_PULL_UP(DISABLE);
                mdelay(1);
                EMU_SET_REVERSE_MODE(TRUE);
                break;
            
            case MOTO_ACCY_HEADSET_MODE_STEREO:
                EMU_SET_VUSB_INPUT_SOURCE(EMU_VREG_IN_BPLUS);
                EMU_SET_HS_SEND_END_REGS(DISABLE);
                EMU_SET_EMU_CONN_MODE(EMU_CONN_MODE_STEREO_AUDIO);
                EMU_SET_HEADSET_PULL_UP(ENABLE);
                mdelay(1);
                EMU_SET_REVERSE_MODE(TRUE);
                break;
        
            default:
                tracemsg(_k_d("EMU: Audio Mode %d not supported"), mode);
        }
        emu_audio_update =FALSE;
    }
    else
    {
        emu_audio_update = TRUE;
        emu_audio_mode = mode;
    }
   
}
   

/*!
 * @brief Initial EMU register settings
 *
 * This function is used to initialize the EMU registers back to a safe state
 * upon removal and also at the beginning of detection
 *
 * @param polling_interval - polling rate
 */
void emu_default_register_settings(int *polling_interval)
{
    /* Make sure that we are using the ID pull up */
    EMU_SET_ID_PULL_UP_CONTROL(DISABLE);

#ifdef CONFIG_MOT_POWER_IC_PCAP2
     /*Reset the usb GPIOs*/
    usbd_set_gpio_function(0);
#endif
   
    /* Enable the 150K D+ pull up */
    EMU_SET_DPLUS_150K_PULL_UP(ENABLE);
    
    /* Disable the ID pull down */
    EMU_SET_ID_PULL_DOWN(DISABLE);
    
    /* Set the conn mode */
    EMU_SET_EMU_CONN_MODE(EMU_CONN_MODE_USB);

    /*Reset headset send/end key registers.*/
    EMU_SET_HS_SEND_END_REGS(DISABLE);
    /*Disable the headset pull up */
    EMU_SET_HEADSET_PULL_UP(DISABLE);
  
    /* Set the USB transceiver to off before debouncing */
    emu_configure_usb_xcvr(EMU_XCVR_OFF);
    
    /* Disable the USB pull up */
    power_ic_periph_set_usb_pull_up(DISABLE);

    /* If switched B+ is enabled disable it */
    if (EMU_GET_REVERSE_MODE())
    {
        EMU_SET_REVERSE_MODE(DISABLE);
        
        /* We have to wait 200 mS for the hardware to
           settle after disabling reverse mode */
        *polling_interval = EMU_POLL_REV_MODE_DELAY;
    }
}
