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
 * Motorola 2005-Jun-16 - Added ATLAS support
 * Motorola 2005-Oct-15 - Finalized the software.
 *
 */

/*!
 * @file debounce.c
 *
 * @ingroup poweric_debounce
 *
 * @brief This is the main file of the power IC debouncing routines
 *
 * This file handles the debouncing of various interrupts from the power IC.
 * The module is desgined to be as generic as possible, with a table that
 * defines the interrupts that are to be monitored (debounced) along with
 * a callback function to be called when a change to the signal has been
 * fully debounced.
 *
 * Currently, this file handles debouncing of the power key, barrel headset,
 * and barrel headset send/end key.  In the future, stereo headset detection
 * will need to be added as well.
 */

/*******************************************************************************
* Includes
*******************************************************************************/
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/keypad.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/delay.h>

#include <asm/hardware.h>

#include <linux/power_ic.h>
#include <linux/moto_accy.h>


#include "../core/event.h"
#include "../core/os_independent.h"
#include "../core/thread.h"

/*******************************************************************************
* Macros and Constants
*******************************************************************************/
#define TO_JIFFIES(msec) (1 + (((msec) * HZ) / 1000))

#define QUEUE_MAX_EVENTS 16

#ifdef CONFIG_MOT_POWER_IC_PCAP2
#define POWER_IC_EVENT_MB2I         POWER_IC_EVENT_PCAP_MB2I
#define POWER_IC_EVENT_ONOFFI       POWER_IC_EVENT_PCAP_ONOFFI
#define POWER_IC_EVENT_HEADSETI     POWER_IC_EVENT_PCAP_A1I
#define A1ID_RX_INDEX 18
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
#define POWER_IC_EVENT_MB2I         POWER_IC_EVENT_ATLAS_MC2BI
#define POWER_IC_EVENT_ONOFFI       POWER_IC_EVENT_ATLAS_ONOFD1I
#define POWER_IC_EVENT_HEADSETI     POWER_IC_EVENT_ATLAS_HSDETI
#endif

/*******************************************************************************
* Type definitions
*******************************************************************************/

/*! Structure defining a power IC event to debounce */
typedef struct debounce_event_t
{
    POWER_IC_EVENT_T event;    /*!< power IC event to watch */

    int enabled;               /*!< 1 = event is enabled, 0 = disabled */

    int poll_period;           /*!< polling period (in milliseconds) */
    int poll_count;            /*!< number of times to poll */

    int polarity;              /*!< Active high or low */       
    void (*callback)(struct debounce_event_t *, int status); /*!< function to call when polling complete */

    /* Entries after this point should not be initialized in debounce_events */

    int poll_current;          /*!< the current polling counter */
    int previous_status;       /*!< previous state of the event */

    struct timer_list timer;   /*!< timer used for this event */
} DEBOUNCE_EVENT_T;

enum
{
    DEBOUNCE_EVENT_POWER,
    DEBOUNCE_EVENT_HEADSET,
    DEBOUNCE_EVENT_MB2I
};

/*******************************************************************************
* Local Variables
*******************************************************************************/

/* doxygen is confused by the following definition since it looks like a function */
#ifndef DOXYGEN_SHOULD_SKIP_THIS

/* Declare a wait queue used to signal arriving events to the handler thread */
static DECLARE_WAIT_QUEUE_HEAD(debounce_wait);

#endif

/*! Spin lock to prevent simultaneous access to the queue */
static spinlock_t queue_lock = SPIN_LOCK_UNLOCKED;

/*! Array location of the head of the queue */
static int queue_head = 0;

/*! Array location of the tail of the queue */
static int queue_tail = 0;

/*! The event queue array */
static POWER_IC_EVENT_T event_queue[QUEUE_MAX_EVENTS];

/* Event table (defined later in the file) */
static DEBOUNCE_EVENT_T debounce_events[];

/*******************************************************************************
* Function Prototypes
*******************************************************************************/


/*******************************************************************************
* Local Functions
*******************************************************************************/

/*!
 * @brief the kernel thread responsible for debouncing
 *
 * This function implements the thread responsible for the majority of the
 * debounce handling.  The thread consists of an infinite loop that will
 * wait for interrupts and/or timer expirations.
 *
 * When an event arrives, it is placed into a queue and then the thread is
 * woken up.  When the thread wakes up, it takes events off of the queue and
 * handles them.  For each event handled, the current interrupt sense bit is
 * compared with the previous state of the sense bit.  If the sense bit stays
 * in the same state for X consecutive iterations (where X is defined on a
 * per-event basis), the callback function is called for the event.
 *
 * When not debouncing, the only thing that would cause the thread to start
 * running is an interrupt from the power IC.  While it is in the process or
 * debouncing an event, the event's interrupt is masked and a timer is used
 * to periodically poll the state of the sense bit.  Once debouncing is
 * complete, the interrupt is unmasked.
 *
 * In order to try to eliminate any interrupt race conditions, the interrupt
 * flag is always cleared whenever the interrupt sense is read.  Then, when
 * the interrupt is unmasked, the status is NOT cleared.  This will cause
 * the interrupt to fire immediately if the interrupt sense were to have
 * changed between the time it was last read and the time that the
 * interrupt was unmasked.
 *
 * @param unused An unused parameter
 *
 * @returns 0, but should never actually return
 */
static int debounce_thread (void *unused)
{
    POWER_IC_EVENT_T queued_event;
    int i;
    int status;

    /* Usual thread setup. */
    thread_common_setup("kdebounced");
    if(thread_set_realtime_priority(THREAD_PRIORITY_DEBOUNCE) != 0)
    {
        tracemsg(_a("Debounce thread - error setting thread priority."));
    }

    /* Loop forever waiting for interrupts and timer expirations */
    while (1)
    {
        
        /* Iterate over the queue of pending events */
        while (queue_head != queue_tail)
        {
            /* Acquire the queue lock so that we can take an event off of the queue */
            spin_lock(&queue_lock);

            /* Grab the first entry and remove it from the queue */
            queued_event = event_queue[queue_head];
            queue_head = (queue_head + 1) % QUEUE_MAX_EVENTS;

            /* Unlock the queue */
            spin_unlock (&queue_lock);
        
            /* Locate the event entry in the debounce event table */
            i = 0;
            while ((debounce_events[i].event < POWER_IC_EVENT_NUM_EVENTS) &&
                   (debounce_events[i].event != queued_event))
            {
                i++;
            }

            /* Only process the event if we found it in the table */
            if (debounce_events[i].event != POWER_IC_EVENT_NUM_EVENTS &&
                debounce_events[i].enabled)
            {
                /* Read the sense bit from the power IC */
                power_ic_event_clear (debounce_events[i].event);
                status = power_ic_event_sense_read (debounce_events[i].event);

                /* If the status is the same as the previous status, increment the counter */
                if (status == debounce_events[i].previous_status &&
                    debounce_events[i].poll_current < debounce_events[i].poll_count)
                {
          	  debounce_events[i].poll_current++;  
                 }

                /* Else, reset the counter and the previous status variable */
                else
                {
                    debounce_events[i].poll_current = 1; 
                    debounce_events[i].previous_status = status;
                    
                }

                /* Check to see if the polling counter has reached its limit */
                if (debounce_events[i].poll_current >= debounce_events[i].poll_count)
                {
                    /* Since we're done debouncing, call the callback function */
                    debounce_events[i].callback(&(debounce_events[i]), status);

                    /* Reset the variables */
                    debounce_events[i].poll_current = 0;
                    debounce_events[i].previous_status = -1;

                    /* Reenable the interrupt */
                    power_ic_event_unmask (debounce_events[i].event);
                }

                /* Else, need to keep polling, so start a timer */
                else
                {
                    /* Set the expiration time */
                    debounce_events[i].timer.expires =
                        jiffies + TO_JIFFIES(debounce_events[i].poll_period);

                    /* Add the timer to the list */
                    add_timer (&(debounce_events[i].timer));
                }
            }
        }
        
        /* Sleep if there are no more events waiting on the queue */
        wait_event (debounce_wait, (queue_head != queue_tail));
    }

    return 0;
}

/*!
 * @brief adds an event to the event queue
 *
 * This function adds an event to the event queue for the debouncing thread.
 * The ordering of the events in the queue doesn't really matter too much
 * since all of the pending events will be handled by the thread during one
 * iteration.
 *
 * This function checks to see if the queue is full by checking to see if
 * incrementing the tail pointer will make the tail equal to the head
 * pointer.  When the tail and head are equal, it means that the queue
 * is empty, so if incrementing the tail would make it equal to the head,
 * it means that the queue is full and new events cannot be added.
 *
 * The queue is currently large enough so that it is impossible for the
 * queue to fill up.  If in the future many additional events are added
 * into the event table, the queue size may need to be increased.
 *
 * @param event power IC event to add
 */
static void add_event (POWER_IC_EVENT_T event)
{
    int next_event = (queue_tail + 1) % QUEUE_MAX_EVENTS;

    /* Acquire the lock protecting the thread queue */
    spin_lock (&queue_lock);

    /* Add the event to the queue if it isn't already full */
    if (next_event != queue_head)
    {
        /* Store the event into the queue */
        event_queue[queue_tail] = event;

        /* Update the tail pointer */
        queue_tail = next_event;
    }

    /* Release the lock */
    spin_unlock (&queue_lock);

    /* Wake up the thread if thread is sleeping */
    wake_up (&debounce_wait);
}

/*!
 * @brief the event handler for all of the power IC events being debounced
 *
 * This is the power IC event handler for the events registered by the debouncing
 * code.  The function simply adds the event to the queue (which will cause the
 * thread to be woken up to process the event).
 *
 * @param event power IC that occurred
 *
 * @return 1 to indicate that the event has been handled
 */
static int debounce_interrupt_handler (POWER_IC_EVENT_T event)
{
    /* Add the event to the queue */
    add_event(event);
    return 1;
}

/*!
 * @brief the handler for a debounce timer expiration
 *
 * This is the timer event handler for the events registered by the debouncing
 * code.  The function simply adds the event to the queue (which will cause th
 * thread to be woken up to process the timer expiration).  Timers are not
 * cyclic, so when the timer expires, it must be restarted by hand before
 * it will "fire" again.  This is handled by the debouncing thread.
 *
 * @param i entry in the event table that this timer expiration is for
 */
static void debounce_timer_handler (unsigned long i)
{
    /* Add the event to the queue */
    add_event(debounce_events[i].event);
}

/*!
 * @brief callback function to indicate power key status
 *
 * This function is called when the status of the power key changes.  The
 * function will report the new state of the power key to the keypad handling
 * code for furture processing.
 *
 * @param event pointer to the location in the event table
 * @param status current status of the power key
 */
static void power_key_debounced (DEBOUNCE_EVENT_T *event, int status)
{
    /* Status is reverse-polarity: 1 = not pressed */
    if (status != debounce_events[DEBOUNCE_EVENT_POWER].polarity)
    {
        tracemsg(_k_d("power_key_debounced: key is released"));
        power_key_event(KEYUP);
    }
    else
    {
        tracemsg(_k_d("power_key_debounced: key is pressed"));
        power_key_event(KEYDOWN);
    }	
}

/*!
 * @brief callback function to indicate headset status
 *
 * This function is called when the status of the headset changes.  The
 * function will report the new state of the accessory to the accessory driver
 * code.  The accessory driver will handle notifying any interested applications
 * of the event.
 *
 * The function also handles enabling and disabling of the headset send/end key
 * interrupt.  In order to prevent problems with spurious send/end key interrupts,
 * the send/end key interrupt (MB2) is not enabled until the headset has been
 * inserted and fully debounced.  Likewise, when the headset is removed, the
 * send/end key interrupt will be disabled.
 *
 * @param event pointer to the location in the event table
 * @param status current status of the headset
 */
#ifdef CONFIG_MOT_POWER_IC_BARREL_HEADSET_STEREO_3MM5

//#define HDEBUG(fmt,args...)		printk("3MM5 Debug: ");printk(fmt,##args)
#define HDEBUG(...)			do {} while (0)

#define HEADSET_DET_HIGH	(0x29b)		/* 1.5 Volts */
#define HEADSET_DET_LOW		(0x0de)		/* 0.5 Volts */

static void headset_3mm5_debounced(DEBOUNCE_EVENT_T *event, int status)
{
	int i = 0;
	int ad6_val = 0;

	/* For sumatra p4 to recognize standard 3.5mm headset */
	set_GPIO_mode(GPIO_HDST_VGS_BOOST|GPIO_OUT);
	set_GPIO(GPIO_HDST_VGS_BOOST);

	/* Search for the MB2 event (send/end key) in the table */
	while (debounce_events[i].event != POWER_IC_EVENT_NUM_EVENTS &&
	       debounce_events[i].event != POWER_IC_EVENT_MB2I) {
		i++;
	}

	/* Status is reverse-polarity: 1 = removed */
	if (status != debounce_events[DEBOUNCE_EVENT_HEADSET].polarity ) {
		HDEBUG("4-pole headset removed.\n");
		/* Notify the accessory driver that the 3.5mm stereo headset has been removed */
		/* It is safe to remove a accessory not attached */
		moto_accy_notify_remove(MOTO_ACCY_TYPE_3MM5_HEADSET_STEREO_MIC);
		moto_accy_notify_remove(MOTO_ACCY_TYPE_3MM5_HEADSET_STEREO);
		tracemsg(_k_d("headset_debounced: 4-pole stereo headset is removed"));

		/* If the MB2 event was in the table, disable it and disable the interrupt */
		if (debounce_events[i].event == POWER_IC_EVENT_MB2I) {
			/* Disable the event in the table */
			debounce_events[i].enabled = 0;
			/* Mask the interrupt */
			power_ic_event_mask(POWER_IC_EVENT_MB2I);
		}
	}
	/* Else, stereo headset is attached */
	else {
		/* Enable MIC_BIAS2 */
		power_ic_set_reg_bit(POWER_IC_REG_PCAP_TX_AUD_AMPS, 10, 1);
		/* Delay for the AD6 to be stable, 10ms will be OK */
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ/100 + 1);
		/* Measure headset detect pin PCAP2 AD6 */
		if (power_ic_atod_single_channel(POWER_IC_ATOD_CHANNEL_AD6, &ad6_val)) {
			printk(KERN_ERR"headset_debounced: Error reading from PCAP2 AD6\n");
			return;
		}
		HDEBUG("AD6: 0x%x\n", ad6_val);
		if (ad6_val > HEADSET_DET_HIGH || ad6_val < HEADSET_DET_LOW) {
			/* Standard 3.5mm headset or iPod 3.5mm headset detected */
			HDEBUG("Standard or iPod 3.5mm headset is attached.\n");
			moto_accy_notify_insert(MOTO_ACCY_TYPE_3MM5_HEADSET_STEREO);
			tracemsg(_k_d("headset_debounced: stardard or iPod 3.5mm headset is inserted"));
		} else {
			/* 4-pole stereo 3.5mm headset with mic detected*/
			HDEBUG("4-pole stereo 3.5mm headset with mic is attached.\n");
			/* Notify the accessory driver that the 4-pole stereo 3.5mm headset with microphone is attached */
			moto_accy_notify_insert(MOTO_ACCY_TYPE_3MM5_HEADSET_STEREO_MIC);
			tracemsg(_k_d("headset_debounced: 4-pole stereo 3.5mm headset with microphone is inserted"));

			/* If the MB2 event was in the table, enable it and enable the interrupt */
			if (debounce_events[i].event == POWER_IC_EVENT_MB2I) {
				/* Enable the event in the table */
				debounce_events[i].enabled = 1;
				/* Reset the debounce variables so that debouncing starts from the beginning */
				debounce_events[i].poll_current = 0;
				debounce_events[i].previous_status = -1;
				/* Clear and unmask the interrupt */
				power_ic_event_clear(POWER_IC_EVENT_MB2I);
				power_ic_event_unmask(POWER_IC_EVENT_MB2I);
			}
		}
	}
}
#else
static void headset_debounced (DEBOUNCE_EVENT_T *event, int status)
{
    int i = 0;
    
    /* Search for the MB2 event (send/end key) in the table */
    while (debounce_events[i].event != POWER_IC_EVENT_NUM_EVENTS &&
           debounce_events[i].event != POWER_IC_EVENT_MB2I)
    {
        i++;
    }
    
    /* Status is reverse-polarity: 1 = removed */
    if (status != debounce_events[DEBOUNCE_EVENT_HEADSET].polarity)
    {
        /* Notify the applications that the headset has been removed */

#ifdef CONFIG_MOT_POWER_IC_BARREL_HEADSET_MONO
        moto_accy_notify_remove(MOTO_ACCY_TYPE_HEADSET_MONO);
        tracemsg(_k_d("headset_debounced: mono headset is removed"));
#else
        power_ic_set_reg_bit(POWER_IC_REG_PCAP_RX_AUD_AMPS,A1ID_RX_INDEX,0);  /* added by e5907c to resolve A1 interrupt issue */ 
        moto_accy_notify_remove(MOTO_ACCY_TYPE_HEADSET_STEREO);
        tracemsg(_k_d("headset_debounced: stereo headset is removed"));
#endif
        /* If the MB2 event was in the table, disable it and disable the interrupt */
        if (debounce_events[i].event == POWER_IC_EVENT_MB2I)
        {
            /* Disable the event in the table */
            debounce_events[i].enabled = 0;

            /* Mask the interrupt */
            power_ic_event_mask(POWER_IC_EVENT_MB2I);
        }
    }

    /* Else, headset is attached */
    else
    {
        /* Notify the applications that the headset is attached */
#ifdef CONFIG_MOT_POWER_IC_BARREL_HEADSET_MONO
        moto_accy_notify_insert(MOTO_ACCY_TYPE_HEADSET_MONO);
        tracemsg(_k_d("headset_debounced: mono headset is inserted"));
#else			 
        power_ic_set_reg_bit(POWER_IC_REG_PCAP_RX_AUD_AMPS,A1ID_RX_INDEX,1);  /* added by e5907c to resolve A1 interrupt issue */                
        moto_accy_notify_insert(MOTO_ACCY_TYPE_HEADSET_STEREO);
        tracemsg(_k_d("headset_debounced: stereo headset is inserted"));
#endif
        /* If the MB2 event was in the table, enable it and enable the interrupt */
        if (debounce_events[i].event == POWER_IC_EVENT_MB2I)
        {
            /* Enable the event in the table */
            debounce_events[i].enabled = 1;

            /* Reset the debounce variables so that debouncing starts from the beginning */
            debounce_events[i].poll_current = 0;
            debounce_events[i].previous_status = -1;

            /* Clear and unmask the interrupt */
            power_ic_event_clear(POWER_IC_EVENT_MB2I);
            power_ic_event_unmask(POWER_IC_EVENT_MB2I);
        }
    }
}
#endif

/*!
 * @brief callback function to indicate headset send/end key status
 *
 * This function is called when the state of the headset send/end key changes.
 * The function will report the new state of the send/end key to the keypad 
 * handling code for furture processing.
 * 
 * @param event pointer to the location in the event table
 * @param status current status of the headset
 */
static void headset_key_debounced (DEBOUNCE_EVENT_T *event, int status)
{
    if (status != debounce_events[DEBOUNCE_EVENT_MB2I].polarity)
    {
        tracemsg(_k_d("headset_key_debounced: headset send/end key is released"));
        headset_key_handler(KEYUP);
    }
    else
    {
        tracemsg(_k_d("headset_key_debounced: headset send/end is pressed"));
        headset_key_handler(KEYDOWN);
    }
}

/*******************************************************************************
* Global Functions
*******************************************************************************/
/*! Event table */
static DEBOUNCE_EVENT_T debounce_events[] =
{
    /* event                 enabled  time  count   polarity  callback */
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    { POWER_IC_EVENT_ONOFFI,     1,       10,   3,     0,      power_key_debounced },	
    /* Hardware version that require that the headset interupts remain masked should be added here */ 
#if defined(CONFIG_ARCH_EZX_BARBADOS) || defined(CONFIG_ARCH_EZX_MARTINIQUE)
#else     
#ifdef CONFIG_MOT_POWER_IC_BARREL_HEADSET_STEREO_3MM5
    { POWER_IC_EVENT_HEADSETI,	 1,	  75,	10,    0,      headset_3mm5_debounced },
#else
    { POWER_IC_EVENT_HEADSETI,   1,       75,   10,    0,      headset_debounced },
#endif
    { POWER_IC_EVENT_MB2I,       0,       15,   10,    0,      headset_key_debounced },
#endif
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    { POWER_IC_EVENT_ONOFFI,     1,       10,   3,     0,      power_key_debounced },
    { POWER_IC_EVENT_HEADSETI,   1,       75,   10,    1,      headset_debounced },
    { POWER_IC_EVENT_MB2I,       0,       15,   10,    0,      headset_key_debounced },
#endif

    /* End of table -- insert new entries before this one */
    { POWER_IC_EVENT_NUM_EVENTS, 0, 0, 0, 0, NULL }
};

/*!
 * @brief function to initialize the power IC debouncing thread
 *
 * This function does the initialization required to start the power IC
 * debouncing thread.  During the initialization, an event for each of the
 * enabled events from the event table is added into the queue to be
 * processed by the debouncing thread.  This is done to allow for power-up
 * determination of the state of each of the events (headset status, etc.)
 * rather than just having to wait for an interrupt from the power IC before
 * the state of the event can be determined.
 */
void power_ic_debounce_init (void)
{
    int i = 0;

    /* Loop through the set of registered events */
    while (debounce_events[i].event < POWER_IC_EVENT_NUM_EVENTS)
    {
        /* Reset the variables for this event */
        debounce_events[i].poll_current = 0;
        debounce_events[i].previous_status = -1;

        /* Initialize the timer data for this entry */
        init_timer(&(debounce_events[i].timer));
        debounce_events[i].timer.data = i;
        debounce_events[i].timer.function = debounce_timer_handler;

        /* Register an event handler */
        power_ic_event_subscribe (debounce_events[i].event, debounce_interrupt_handler);

        /* If the entry is enabled, force an event for this entry, just to get things started */
        if (debounce_events[i].enabled)
        {
            add_event(debounce_events[i].event);
        }

        /* Move to the next event in the table */
        i++;
    }

    /* Start the debouncer kernel thread */
    kernel_thread (debounce_thread, NULL, 0);
}
