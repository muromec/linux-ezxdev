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
 * Motorola 2005-Jun-19 - Added VBUS interrupt handler
 * Motorola 2005-Dec-13 - Finalize the software.
 *
 */

/*!
 * @file emu_state_machine.c
 *
 * @ingroup poweric_emu
 *
 * @brief EMU accessory detection state machine engine
 *
 * States are implemented in the state machine by a combination of up to three
 * functions: a state entry function, a state exit function, and a state event
 * handler function.  As the names imply, the entry function is called when the
 * state is entered, the exit function is called when the state is exited, and the
 * event handler function is called whenever an event or timer expiration occurs.
 * It is not necessary for all states to implement all three of the functions, but
 * each state must have at least an event handler function.
 */

#include <linux/sched.h>
#include <linux/version.h>

#include "emu.h"
#include "../core/os_independent.h"
#include "../core/thread.h"

/*! Structure defining format of state table entries */
typedef struct
{
    /*! Pointer to state event/timeout handler function */
    EMU_STATE_T (*handler)(int *polling_interval);

    /*! Pointer to state entry function */
    void (*entry_function)(EMU_STATE_T prev_state, int *polling_interval);

    /*! Pointer to state exit function */
    void (*exit_function)(EMU_STATE_T next_state, int *polling_interval);
} STATE_TABLE_ENTRY_T;

/*! EMU accessory detection state table */
static const STATE_TABLE_ENTRY_T state_table[EMU_STATE__NUM_STATES] =
{
    /*Handler Function           Entry Function          Exit Function      */
    /**************************DISCONNECTED STATE****************************/
    { disconnected_handler,      disconnected_enter,      NULL               },
    /**************************CONNECTING SUB STATES*************************/
    { debounce_dev_type_handler, debounce_dev_type_enter, NULL               },
    { spd_delay_handler,         spd_delay_enter,         NULL               },
    { unpowered_sihf_handler,    unpowered_sihf_enter,    NULL               },
    { ppd_validate_handler,      ppd_validate_enter,      NULL               },
    { pre_ppd_identify_handler,  pre_ppd_identify_enter,  NULL               },
    { ppd_identify_handler,      ppd_identify_enter,      NULL               },
    /**************************CONNECTED SUB STATES**************************/
    { poll_spd_removal_handler,  device_config_enter,     NULL               },
    { factory_handler,           device_config_enter,     NULL               },
    { headset_handler,           device_config_enter,     NULL               },
    /**************************DISCONNECTING STATE***************************/
    { disconnecting_handler,     disconnecting_enter,     disconnecting_exit }
};

/* This table must match the EMU_STATE_T enum */
static const char* function_name_tbl[EMU_STATE__NUM_STATES] =
{
    "Disonnected",
    "Connecting debounce",
    "Connecting SPD delay",
    "Connecting unpowered SIHF",
    "Connecting PPD validate",
    "Connecting PPD pre_verify",
    "Connecting PPD verify",
    "Connected SPD poll removal ",
    "Connected factory/invalid/not supported",
    "Connected headset",
    "Disconnecting"
};

/******************************************************************************
* Global variables
******************************************************************************/

/*! Holds the current state */
EMU_STATE_T emu_state;

/*! Holds the previous state */
EMU_STATE_T emu_prev_state;

/*! Holds what we believe the currently connected device is */
MOTO_ACCY_TYPE_T emu_current_device = MOTO_ACCY_TYPE_NONE;

/*! Holds the current device type */
EMU_DEV_TYPE_T emu_current_device_type = EMU_DEV_TYPE_NONE;

/*! Used to keep track of the send/end key position */
EMU_BUS_SIGNAL_STATE_T emu_id_state = EMU_BUS_SIGNAL_STATE_UNKNOWN;

/*! Used to keep track of the state of VBUS */
EMU_BUS_SIGNAL_STATE_T emu_vbus_det_state = EMU_BUS_SIGNAL_STATE_UNKNOWN;

/*! Wait event flag */
BOOL emu_event_flag = FALSE;

/*! Wait queue for interrupt/thread communication */
DECLARE_WAIT_QUEUE_HEAD(emu_thread_wq);

/******************************************************************************
* Global functions
******************************************************************************/
/*!
 * @brief Event handler for the power IC SE1 and ID interrupts
 *
 * This function is the event handler for the power IC SE1 and ID interrupts.
 * When the events occur, the job of the interrupt handler is to ensure that
 * the accessory detection state machine thread is running to handle the
 * event.  This is accomplished through two mechanisms:
 *
 *   - Set a flag to indicate that an event occurred.  The thread clears
 *     the flag every time it wakes up and will always check to make sure
 *     that the flag is clear before going to sleep.  This ensures that
 *     interrupts are never missed by the thread.
 *
 *   - Wake up the thread if it is sleeping.  Setting the flag doesn't
 *     automatically wake up the thread, so if the thread is sleeping (and
 *     on the wait queue), it will be woken up after setting the flag.
 *     If the thread wasn't sleeping, the call to wake_up() does nothing.
 *
 * @param unused an unused parameter indicating which even occurred
 *
 * @return 1, indicating that the event has been handled
 */
static int emu_int_handler(POWER_IC_EVENT_T unused)
{
    tracemsg(_k_d("EMU: emu_int_handler: interrupt received"));
    
    /* Set the flag indicating that an event occurred */
    emu_event_flag = TRUE;

    /* Wake up the thread if it is sleeping */
    wake_up(&emu_thread_wq);

    /* Indicate that the event has been handled */
    return 1;
}

/*!
 * @brief Event handler for the power IC VBUS interrupt
 *
 * This function is the event handler for the power IC VBUS_DET interrupt.
 * When the events occur, the job of the interrupt handler is to ensure that
 * the accessory detection state machine thread is running to handle the
 * event.  This is accomplished through two mechanisms:
 *
 *   - Set a flag to indicate that an event occurred.  The thread clears
 *     the flag every time it wakes up and will always check to make sure
 *     that the flag is clear before going to sleep.  This ensures that
 *     interrupts are never missed by the thread.
 *
 *   - Wake up the thread if it is sleeping.  Setting the flag doesn't
 *     automatically wake up the thread, so if the thread is sleeping (and
 *     on the wait queue), it will be woken up after setting the flag.
 *     If the thread wasn't sleeping, the call to wake_up() does nothing.
 *
 * @param unused an unused parameter indicating which even occurred
 *
 * @return 1, indicating that the event has been handled
 */
static int emu_vbus_det_int_handler(POWER_IC_EVENT_T unused)
{
    EMU_BUS_SIGNAL_STATE_T new_vbus_det_state;
        
    tracemsg(_k_d("EMU: emu_vbus_det_int_handler: interrupt received"));

    new_vbus_det_state = get_bus_state(EMU_BUS_SIGNAL_VBUS);
    
    /* If VBUS 4V4 sense bit has changed, an SPD has been removed, inserted, or it has collapsed, so
       start the state machine. If VBUS 4V4 is currently low and VBUS 2V0 was
       high and is now low, a collapsed SPD has been removed so start the state machine.*/
    if (new_vbus_det_state != emu_vbus_det_state)
    {
        /* Set the flag indicating that an event occurred */
        emu_event_flag = TRUE;

        /* Wake up the thread if it is sleeping */
        wake_up(&emu_thread_wq);
    }
    else
    {
        /* Make sure that the interrupt is enabled before leaving in case this interrupt
           was generated by VBUS crossing the 0.8V or 2.0V thresholds */
        power_ic_event_unmask(EMU_INT_VBUS);
    }

    /* Only enable the the charge current interrupt if a collapsed SPD is connected */
    if (new_vbus_det_state == EMU_BUS_SIGNAL_STATE_VBUS2V0)
    {
        power_ic_event_clear(EMU_INT_CHRG_CURR);
        power_ic_event_unmask(EMU_INT_CHRG_CURR);
    }
    else
    {
        power_ic_event_mask(EMU_INT_CHRG_CURR);
    }
    
    /* Update stored VBUS_DET state, this variable is also updated in the
       get_device_type function */
    emu_vbus_det_state = new_vbus_det_state;
    
    /* Indicate that the event has been handled */
    return 1;
}

/*!
 * @brief Main EMU accessory detection state machine thread
 *
 * This function implements the main EMU accessory detection state machine
 * thread loop.  The state machine itself is implemented as a simple loop.
 * In each iteration of the loop, a state handler function is called.  When
 * the return value from the state handler function indicates a state change,
 * the exit function for the current state is executed followed by the entry
 * function for the new state.
 *
 * Each function that is called (entry, exit, or the handler function) has an
 * opportunity to change the "polling rate".  This polling rate can be one of
 * three values: a timeout, an indication to wait for the next event, or an
 * indication that there should be no delay before calling the state handler
 * function again.  Based on the return value, the state machine engine will
 * determine if it should sleep before running the next iteration of the state
 * machine.
 *
 * @param unused An unused parameter
 *
 * @return 0, but function should never return
 */

static int emu_state_machine_thread(void *unused)
{
    int polling_interval;
    EMU_STATE_T next_state;

    /* Usual thread setup. */
    thread_common_setup("kemud");
    if(thread_set_realtime_priority(THREAD_PRIORITY_EMU) != 0)
    {
        tracemsg(_a("EMU thread - error setting thread priority."));
    }
    
    /* Force state machine variables to be intitialized*/
    emu_state = EMU_STATE__NUM_STATES;
    
    while (1)
    {
        
        /* Make sure that the state is in the proper range */
        if (emu_state >= EMU_STATE__NUM_STATES)
        {
            polling_interval = EMU_POLL_WAIT_EVENT;
            emu_prev_state = EMU_STATE_DISCONNECTED;
            emu_state = EMU_STATE_CONNECTING__DEBOUNCE_DEV_TYPE;

            /* Call the state entry function if it exists */
            if (state_table[emu_state].entry_function)
            {
                tracemsg(_k_d("EMU: %s enter"), function_name_tbl[emu_state]);
                state_table[emu_state].entry_function(emu_prev_state,
                                                      &polling_interval);
            }
        }
        
        /* Call the state handler function */
        tracemsg(_k_d("EMU: %s handler"), function_name_tbl[emu_state]);
        next_state = state_table[emu_state].handler(&polling_interval);
        
        /* Handle a state change if one was requested */
        if (next_state != emu_state)
        {
            /* Reset the debounce counter when transitioning to a new state */
            emu_debounce_counter = 0;
            
            /* Call the state exit function if it exists */
            if (state_table[emu_state].exit_function != NULL)
            {
                /* Exit the current state */
                tracemsg(_k_d("EMU: %s exit"), function_name_tbl[emu_state]);
                state_table[emu_state].exit_function(next_state, &polling_interval);
            }

            /* Store the previous state */
            emu_prev_state = emu_state;
            
            /* Update the state variable */
            emu_state = next_state;

            /* Call the state entry function if it exists */
            if (state_table[next_state].entry_function != NULL)
            {
                /* Enter the new state */
                tracemsg(_k_d("EMU: %s enter"), function_name_tbl[next_state]);
                state_table[next_state].entry_function(emu_prev_state,
                                                       &polling_interval);
            }
        }

        if (polling_interval >= EMU_POLL_MIN_DELAY)
        {
            tracemsg(_k_d("EMU: emu_state_machine_thread: sleep for %d"), polling_interval);
            
            /* Sleep for polling interval */
            wait_event_timeout(emu_thread_wq, emu_event_flag, polling_interval);

            tracemsg(_k_d("EMU: emu_state_machine_thread: done sleeping"));
                
            /* Reset the event flag */
            emu_event_flag = FALSE;
        }
        else if (polling_interval == EMU_POLL_WAIT_EVENT)
        {
            tracemsg(_k_d("EMU: emu_state_machine_thread: waiting for an interrupt"));
            
            /* Wait for the next event if we haven't already received it */
            wait_event(emu_thread_wq, emu_event_flag);

            tracemsg(_k_d("EMU: emu_state_machine_thread: received an interrupt"));
            
            /* Reset the event flag */
            emu_event_flag = FALSE;
        }
        /* else don't sleep */
    }

    /* Should never reach this point */
    return 0;
}

/*!
 * @brief Initializes the EMU accessory detection state machine
 *
 * The function performs any initialization required to get the EMU
 * accessory detection state machine running.  This includes registering
 * for the required power IC events and starting the kernel thread.
 */
void __init emu_init(void)
{
    tracemsg(_k_d("EMU: emu_init: initializing state machine"));
    
#ifdef CONFIG_MOT_POWER_IC_ATLAS
    iomux_config_mux(AP_GPIO_AP_C16, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC3); 
    iomux_config_mux(AP_GPIO_AP_C17, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_FUNC3); 
    gpio_config(GPIO_AP_C_PORT, 16, false, GPIO_INT_NONE);
    gpio_config(GPIO_AP_C_PORT, 17, false, GPIO_INT_NONE);
#endif

    /* Subscribe to the EMU interrupt events */
    EMU_SUBSCRIBE_VBUS_INTERRUPT(emu_vbus_det_int_handler);
    EMU_SUBSCRIBE_ID_INTERRUPT(emu_int_handler);
    EMU_SUBSCRIBE_SE1_INTERRUPT(emu_int_handler);
    EMU_SUBSCRIBE_CHRGCURR_INTERRUPT(emu_int_handler);

#ifdef CONFIG_CPU_BULVERDE
    /*Configure the GPIO for emu stereo headset as output*/
    EMU_CONFIG_HEADSET_MODE();
#endif    
    
    /* Create thread */
    kernel_thread(emu_state_machine_thread, NULL, 0);
}
