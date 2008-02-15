/*
 * Copyright 2004 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2004-2006 Motorola, Inc. All Rights Reserved.
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
 * Motorola 2004-Dec-06 - Redesign of the Power IC event handler 
 * 
 * Motorola 2006-Dec-10 - Finalize design of the event handler
 */

/*!
 * @file event.c
 *
 * @ingroup poweric_core
 *
 * @brief Power IC event (interrupt) handling
 *
 * This file contains all of the functions and data structures required to
 * implement the power IC event handling.  This includes:
 *
 *    - The actual interrupt handler for PCAP and EMU One Chip
 *    - The bottom half interrupt handler for PCAP and EMU One Chip
 *    - The public, kernel-level interface for event handling.
 *
 * The power IC (both PCAP and EMU One Chip) have three registers related to
 * interrupts: the interrupt status register, the interrupt sense register, and
 * the interrupt mask register.  The status register is a latched indication
 * that the interrupt condition occurred.  That is, when the condition that causes
 * the interrupt occurs, the interrupt status bit is set and can only be cleared
 * by a write to the interrupt status register.  For some interrupts, there is
 * internal hardware debouncing that will occcur before the interrupt status bit
 * will be set.
 *
 * The interrupt sense register is used to indicate the current, unlatched state
 * of the event.  The bits in this register are read-only and can change at any
 * time.
 *
 * The interrupt mask register is used to indicate which interrupt status bits
 * can cause the interrupt line to the processor to be asserted.  Each zero bit
 * in the interrupt mask register indicates that when the corresponding interrupt
 * status bit is set, the external interrupt line will be asserted.  Note that
 * the interrupt mask bits do not prevent the interrupt status bits from being
 * set -- they only affect the external interrupt line.
 *
 * The way that interrupt handling is managed in this driver is through two levels
 * of interrupt handling.  There is a top-half interrupt handler that is called
 * whenever there is a rising edge on the external interrupt line going into the
 * processor.  This interrupt handler is executed when all other interrupts are
 * disabled, so it needs to run as quickly as possible.  The only thing that the
 * top-half interrupt handler does is to "schedule" the bottom-half interrupt
 * handler.
 *
 * The bottom-half interrupt handler is run in "bottom-half" or task context,
 * meaning that interrupts have been re-enabled and are allowed to interrupt the
 * bottom-half interrupt handler.  The bottom-half interrupt handler is responsible
 * for:
 *
 *    - Clearing the interrupt condition in the power IC (to allow the interrupt
 *      line to go back low.
 *    - Dispatching the interrupt event to registered callback functions.
 *
 */

#include <linux/list.h>
#include <linux/power_ic.h>
#include <linux/bitops.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/module.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>

#ifdef CONFIG_CPU_BULVERDE
#include <asm/arch/pxa-regs.h>
#endif /* Bulverde */

#ifdef CONFIG_ARCH_SCMA11
#include <asm/arch/gpio.h>
#endif /* SCM-A11 */

#include "event.h"
#include "os_independent.h"
#include "thread.h"

#ifdef CONFIG_MOT_POWER_IC_PCAP2
#include "eoc_register.h"
#include "pcap_register.h"
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
#include "atlas_register.h"
#endif /* ATLAS */

#ifndef DOXYGEN_SHOULD_SKIP_THIS
EXPORT_SYMBOL(power_ic_event_subscribe);
EXPORT_SYMBOL(power_ic_event_unsubscribe);
EXPORT_SYMBOL(power_ic_event_mask);
EXPORT_SYMBOL(power_ic_event_unmask);
EXPORT_SYMBOL(power_ic_event_clear);
EXPORT_SYMBOL(power_ic_event_sense_read);
#endif

/******************************************************************************
* Local type definitions
******************************************************************************/

/* Define a type for implementing a list of callback functions */
typedef struct
{
    struct list_head list; /*!< the list head for the list of callbacks */
    POWER_IC_EVENT_CALLBACK_T callback; /*!< the callback function pointer */
} POWER_IC_EVENT_CALLBACK_LIST_T;

/******************************************************************************
* Local variables
******************************************************************************/

/*! Define the event lists */
struct list_head power_ic_events[POWER_IC_EVENT_NUM_EVENTS];

/*! Flag to indicate pending interrupt conditions */ 
static int interrupt_flag = 0;

/* Declare a wait queue used to signal arriving interrupts to the interrupt handler thread */
static DECLARE_WAIT_QUEUE_HEAD(interrupt_thread_wait);

/******************************************************************************
* Local function prototypes
******************************************************************************/
static void power_ic_bh_handler(unsigned long unused);

/******************************************************************************
* Local functions
******************************************************************************/

/*!
 * @brief Executes the registered callback functions for a given event
 *
 * Iterates over the list of registered callback functions (saved in
 * #power_ic_events[]) and calls each of the callback functions.  The
 * return value from the callback indicates if the event was handled
 * by the callback and if additional callbacks should be called.  That
 * is, if a callback function returns a non-zero value, the event is
 * considered handled and no other callbacks are called for the event.
 *
 * The list of callbacks is implemented using the Linux standard
 * doubly-linked list implementation.
 *
 * @param       event    the event that occurred
 *
 * @return      nothing
 */

static void execute_handlers (POWER_IC_EVENT_T event)
{
    struct list_head *p;
    POWER_IC_EVENT_CALLBACK_LIST_T *t;

    list_for_each (p, &(power_ic_events[event]))
    {
        t = list_entry (p, POWER_IC_EVENT_CALLBACK_LIST_T, list);

        /* If the callback returns a non-zero value, it handled the event */
        if (t->callback(event) != 0)
        {
            break;
        }
    }
}


#ifdef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT
extern int spi_int_req( void (*pProcedure)(void) );
extern int spi_int_reg_read (int reg, unsigned int *value_ptr);
extern int spi_int_reg_write (int reg, unsigned int value);
extern int spi_int_set_reg_bit(POWER_IC_REGISTER_T reg, int index, int value);

/*!
 * @brief Masks (disables) an event.
 *
 * This function sets the interrupt mask bit for the given event.  This will
 * prevent interrupts from being generated for the event when it occurrs. 
 * This function only support pcap operation.
 * 
 * @param        event   the event type to mask
 *
 * @return 0 on success
 */
int spi_int_event_mask (POWER_IC_EVENT_T event) 
{
    /* Verify that the event number is not out of range */
    if ( (event >= POWER_IC_EVENT_PCAP_FIRST)
       &&(event <= POWER_IC_EVENT_PCAP_LAST))
    {
        /* Set the interrupt mask bit to 1 to mask the interrupt */
        return spi_int_set_reg_bit (
            POWER_IC_REG_PCAP_IMR,
            event - POWER_IC_EVENT_PCAP_FIRST,
            1);
    }
    else
    {
        return -EINVAL;
    }

}

/*!
 * @brief Unmasks (enables) an event.
 *
 * This function clears the interrupt mask bit for the given event.  This will
 * allow interrupts to be generated for the event when the event occurs.  Once
 * the event is unmasked, the event callback could be called at any time (including
 * before this function returns back to the caller).
 * This function only support pcap operation.
 *
 * @note If the interrupt is pending and masked, when it is unmasked, the callback
 * will be called immediately.
 *
 * @pre While not specifically required, a callback function should generally be
 * registered before unmasking an event.
 *
 * @param        event   the event type to unmask
 *
 * @return 0 on success
 */
int spi_int_event_unmask (POWER_IC_EVENT_T event)
{
    /* Verify that the event number is not out of range */
    if ( (event >= POWER_IC_EVENT_PCAP_FIRST)
       &&(event <= POWER_IC_EVENT_PCAP_LAST))
    {
        /* Set the interrupt mask bit to 0 to unmask the interrupt */
        return spi_int_set_reg_bit (
            POWER_IC_REG_PCAP_IMR,
            event - POWER_IC_EVENT_PCAP_FIRST,
            0);
    }
    else
    {
        return -EINVAL;
    }
}

/*!
 * @brief Irq handler for pcap interrupt.
 *
 * This function handles pcap interrupt request.
 */
void pcap2_power_ic_bh_handler(void)
{
    unsigned int isr0;
    unsigned int imr0;
    unsigned int enabled_ints0;
    unsigned int int_vec;

    /* Loop while PCAP continues to assert its interrupt line */
    while (GPLR(1) & GPIO_bit(1))
    {
        /* Read the interrupt status and mask registers */
        spi_int_reg_read (POWER_IC_REG_PCAP_ISR, &isr0);
        spi_int_reg_read (POWER_IC_REG_PCAP_IMR, &imr0);

        /* Mask all interrupts that we are about to service */
        spi_int_reg_write (POWER_IC_REG_PCAP_IMR, imr0 | isr0);
        
        /* Get the set of enabled interrupt bits */
        enabled_ints0 = isr0 & ~imr0;
        /*
        printk("isr0=0x%x;imr0=0x%x;enabled_ints0=0x%x\n", 
               isr0, imr0, enabled_ints0);
        */
        /* Loop to handle the interrupts */
        while (enabled_ints0 != 0)
        {
            /* Find the interrupt number of the highest priority interrupt */
            int_vec = ffs(enabled_ints0) - 1;

            /* Run the handlers for the event */
            execute_handlers(int_vec + POWER_IC_EVENT_PCAP_FIRST);

            /* Clear the bit for the interrupt we just serviced */
            enabled_ints0 &= ~(1 << int_vec);
        }

        /* Clear the interrupt status bits */
        spi_int_reg_write (POWER_IC_REG_PCAP_ISR, isr0);
    }
}

/*!
 * @brief Interrupt handler for PCAP interrupts
 *
 * This is the interrupt handler for the PCAP interrupts.
 * The purpose of the interrupt handler is to handle pcap interrupt 
 * as fast as possible.
 *
 * @note This function runs with context switching and all other interrupts
 * disabled, so it needs to run as fast as possible.
 *
 * @param        irq        the irq number
 * @param        dev_id     the pointer on the device
 * @param        regs       the interrupt parameters
 *
 * @return       The function returns IRQ_RETVAL(1) when handled.
 */

static irqreturn_t spi_int_pcap2_irq_handler (int irq, void *dev_id, struct pt_regs *regs)
{
#ifdef CONFIG_ARCH_SCMA11
    /* Clear interrupt */
    edio_clear_int(ED_INT5);
    edio_get_int(ED_INT5);
#endif /* SCM-A11 */
    
    spi_int_req( pcap2_power_ic_bh_handler );

    return IRQ_RETVAL(1);
}



#endif /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */



/*!
 * @brief Interrupt handler for PCAP and EMU One Chip interrupts
 *
 * This is the interrupt handler for the PCAP and EMU One Chip interrupts.
 * The purpose of the interrupt handler is to schedule the bottom half
 * interrupt handler.  Due to problems with the I2C driver failing
 * to operate in interrupt (or bottom half) context, the bottom half of
 * the interrupt is implemented as a kernel thread.  The interrupt handler
 * wakes up the thread (which is waiting on a wait queue).
 *
 * @note This function runs with context switching and all other interrupts
 * disabled, so it needs to run as fast as possible.
 *
 * @param        irq        the irq number
 * @param        dev_id     the pointer on the device
 * @param        regs       the interrupt parameters
 *
 * @return       The function returns IRQ_RETVAL(1) when handled.
 */

static irqreturn_t power_ic_irq_handler (int irq, void *dev_id, struct pt_regs *regs)
{
#ifdef CONFIG_ARCH_SCMA11
    /* Clear interrupt */
    edio_clear_int(ED_INT5);
    edio_get_int(ED_INT5);
#endif /* SCM-A11 */
    
    /* Set the interrupt flag to prevent the thread from sleeping */
    interrupt_flag = 1;

    /* Wake up the "bottom half" interrupt handler thread */
    wake_up(&interrupt_thread_wait);

    return IRQ_RETVAL(1);
}

/*!
 * @brief Implements the kernel thread for power IC "bottom half" interrupt handling.
 *
 * The function that implements the kernel thread for interrupt handling.  This
 * is used to "simulate" a bottom half because the I2C driver only works in task
 * context, not in interrupt or bottom half context.  The function simply executes
 * the same bottom half function that would normally run in the bottom half tasklet.
 *
 * @param unused An unused parameter
 *
 * @return the function should never return
 */

static int interrupt_thread_loop (void *unused)
{
    /* Usual thread setup. */
    thread_common_setup("keventd");
    if(thread_set_realtime_priority(THREAD_PRIORITY_EVENT) != 0)
    {
        tracemsg(_a("Event thread - error setting thread priority."));
    }

    while(1)
    {
        /* Sleep if an interrupt isn't pending again */
        wait_event (interrupt_thread_wait, interrupt_flag);

        /* Reset the interrupt flag */
        interrupt_flag = 0;

        /* Handle the interrupt */
        power_ic_bh_handler(0);
    }

    return 0;
}

/*!
 * @brief The bottom half of the power IC interrupt handler.
 *
 * This function implements the bottom half of the power IC interrupt handler.
 * It handles both PCAP and EMU One Chip interrupt processing.
 *
 * For PCAP interrupt handling, the function loops as long as PCAP continues to
 * assert its interrupt line to the processor.  In each iteration of the loop,
 * the interrupt status and interrupt mask registers are read from PCAP to determine
 * the set of outstanding interrupts.  These interrupts are then "dispatched" to
 * the registered callback functions.  All of the pending interrupts are then
 * cleared and masked.
 *
 * EMU One Chip interrupts are handled identically to PCAP interrupts.
 *
 * @note This function normally runs in bottom-half context, meaning that interrupts
 * are enabled, but user processes are being preempted.  This needs to execute quickly
 * to allow control to return back to user-space as soon as possible, but it is not
 * absolutely time critical.
 *
 * @param unused An unused parameter
 *
 * @return nothing
 */

static void power_ic_bh_handler(unsigned long unused)
{
    unsigned int isr0;
    unsigned int imr0;
    unsigned int enabled_ints0;
    unsigned int int_vec;

#ifdef CONFIG_MOT_POWER_IC_PCAP2

#ifndef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT
    /* Loop while PCAP continues to assert its interrupt line */
    while (GPLR(1) & GPIO_bit(1))
    {
        /* Read the interrupt status and mask registers */
        power_ic_read_reg (POWER_IC_REG_PCAP_ISR, &isr0);
        power_ic_read_reg (POWER_IC_REG_PCAP_IMR, &imr0);

        /* Mask all interrupts that we are about to service */
        power_ic_write_reg_value (POWER_IC_REG_PCAP_IMR, imr0 | isr0);
        
        /* Get the set of enabled interrupt bits */
        enabled_ints0 = isr0 & ~imr0;

        /* Loop to handle the interrupts */
        while (enabled_ints0 != 0)
        {
            /* Find the interrupt number of the highest priority interrupt */
            int_vec = ffs(enabled_ints0) - 1;

            /* Run the handlers for the event */
            execute_handlers(int_vec + POWER_IC_EVENT_PCAP_FIRST);

            /* Clear the bit for the interrupt we just serviced */
            enabled_ints0 &= ~(1 << int_vec);
        }

        /* Clear the interrupt status bits */
        power_ic_write_reg_value (POWER_IC_REG_PCAP_ISR, isr0);
    }
#endif /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */

    /* Loop while EMU One Chip continues to assert its interrupt line */
    while (GPLR(10) & GPIO_bit(10))
    {
        /* Read the interrupt status and mask registers */
        power_ic_read_reg (POWER_IC_REG_EOC_INT_STATUS, &isr0);
        power_ic_read_reg (POWER_IC_REG_EOC_INT_MASK, &imr0);

        /* Mask all interrupts that we are about to service */
        power_ic_write_reg_value (POWER_IC_REG_EOC_INT_MASK, imr0 | isr0);
        
        /* Get the set of enabled interrupt bits */
        enabled_ints0 = isr0 & ~imr0;

        /* Loop to handle the interrupts */
        while (enabled_ints0 != 0)
        {
            /* Find the interrupt number of the highest priority interrupt */
            int_vec = ffs(enabled_ints0) - 1;

            /* Run the handlers for the event */
            execute_handlers(int_vec + POWER_IC_EVENT_EOC_FIRST);

            /* Clear the bit for the interrupt we just serviced */
            enabled_ints0 &= ~(1 << int_vec);
        }

        /* Clear the interrupt status bits */
        power_ic_write_reg_value (POWER_IC_REG_EOC_INT_STATUS, isr0);
    }
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    unsigned int isr1;
    unsigned int imr1;
    unsigned int enabled_ints1;
#ifdef CONFIG_ARCH_SCMA11
    /* Loop while ATLAS continues to assert its interrupt line */
    while (edio_get_data(ED_INT5))
#else
    /* Build error out because Atlas is currently not implemented for Bulverde, so we do not
       know which GPIO will need to be constantly checked for the while loop.  Until Atlas is
       implemented, this conditional and error must stay */
    #error "Bulverde with ATLAS: GPIO not defined"
#endif /* SCM-A11 */
    {
        /* Read the interrupt status and mask registers */
        power_ic_read_reg (POWER_IC_REG_ATLAS_INT_STAT_0, &isr0);
        power_ic_read_reg (POWER_IC_REG_ATLAS_INT_MASK_0, &imr0);
        power_ic_read_reg (POWER_IC_REG_ATLAS_INT_STAT_1, &isr1);
        power_ic_read_reg (POWER_IC_REG_ATLAS_INT_MASK_1, &imr1);

        /* Mask all interrupts that we are about to service */
        power_ic_write_reg_value (POWER_IC_REG_ATLAS_INT_MASK_0, imr0 | isr0);
        power_ic_write_reg_value (POWER_IC_REG_ATLAS_INT_MASK_1, imr1 | isr1);
        
        /* Get the set of enabled interrupt bits */
        enabled_ints0 = isr0 & ~imr0;
        enabled_ints1 = isr1 & ~imr1;
        
        /* Loop to handle the interrupts for Register 0 */
        while (enabled_ints0 != 0)
        {
            /* Find the interrupt number of the highest priority interrupt */
            int_vec = ffs(enabled_ints0) - 1;

            /* Run the handlers for the event for Interrupt Status Register 0 */
            execute_handlers(int_vec + POWER_IC_EVENT_ATLAS_FIRST_REG);

            /* Clear the bit for the interrupt we just serviced */
            enabled_ints0 &= ~(1 << int_vec);
        }
        
        /* Loop to handle the interrupts for Register 1 */
        while (enabled_ints1 != 0)
        {
            /* Find the interrupt number of the highest priority interrupt */
            int_vec = ffs(enabled_ints1) - 1;

            /* Run the handlers for the event for Interrupt Status Register 1 */
            execute_handlers(int_vec + POWER_IC_EVENT_ATLAS_SECOND_REG);

            /* Clear the bit for the interrupt we just serviced */
            enabled_ints1 &= ~(1 << int_vec);
        }

        /* Clear the interrupt status bits */
        power_ic_write_reg_value (POWER_IC_REG_ATLAS_INT_STAT_0, isr0);
        power_ic_write_reg_value (POWER_IC_REG_ATLAS_INT_STAT_1, isr1);
    }
#endif /* ATLAS */
}

/******************************************************************************
* Global functions
******************************************************************************/

/*!
 * @brief Initializes the power IC event handling
 *
 * This function initializes the power IC event handling.  This includes:
 *     - Initializing the doubly-linked lists in the #power_ic_events array
 *     - Configuring the GPIO interrupt lines
 *     - Registering the GPIO interrupt handlers
 *
 * This function is also responsible for creating the kernel thread that will be
 * use as a temporary replacement for the bottom half interrupt tasklet due to
 * issues with the I2C driver.
 *
 * @return nothing
 */
void power_ic_event_initialize (void)
{
    int i;
    
    /* Initialize the lists for power IC events */
    for (i = 0; i < POWER_IC_EVENT_NUM_EVENTS; i++)
    {
        INIT_LIST_HEAD(&(power_ic_events[i]));
    }

    /* Start our kernel thread */
    kernel_thread(interrupt_thread_loop, NULL, 0);

#ifdef CONFIG_CPU_BULVERDE
    /* Configure and register the PCAP interrupt */
    set_GPIO_mode(GPIO1_RST);
    set_GPIO_IRQ_edge(GPIO1_RST, GPIO_RISING_EDGE);
#ifdef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT
    request_irq(IRQ_GPIO(1), spi_int_pcap2_irq_handler, 0, "PCAP irq", NULL);
#else
    request_irq(IRQ_GPIO(1), power_ic_irq_handler, 0, "PCAP irq", NULL);
#endif /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */

    /* Configure and register the EMU One Chip interrupt */
    set_GPIO_mode(GPIO10_RTCCLK);
    set_GPIO_IRQ_edge(GPIO10_RTCCLK, GPIO_RISING_EDGE);
    request_irq(IRQ_GPIO(10), power_ic_irq_handler, 0, "EMU One Chip irq", NULL);
#endif /* Bulverde */

#ifdef CONFIG_ARCH_SCMA11
    /* Configure mux */
    iomux_config_mux(AP_ED_INT5, OUTPUTCONFIG_FUNC1, INPUTCONFIG_FUNC1); 
    /* Configure and register the ATLAS interrupt */
    edio_config(ED_INT5, false, EDIO_INT_RISE_EDGE);
    request_irq(INT_EXT_INT5, power_ic_irq_handler, SA_INTERRUPT, "Atlas irq", 0);
#endif /* SCM-A11 */
}

/*!
 * @brief Registers a callback function to be called when an event occurs
 *
 * This function is used to subscribe to an event.  The callback function is
 * stored in a dynamically-allocated structure and then added to a doubly-linked
 * list implemented using the standard Linux doubly-linked list implementation.
 * Multiple callbacks may be added for each event.  Duplicated callback function
 * checking is not implemented.
 *
 * @param        event     the event type
 * @param        callback  the function to call when event occurrs
 *
 * @return 0 when successful or -ENOMEM if memory allocation failed
 */

int power_ic_event_subscribe (POWER_IC_EVENT_T event, POWER_IC_EVENT_CALLBACK_T callback)
{
    POWER_IC_EVENT_CALLBACK_LIST_T *temp;

    /* Verify that the event number is not out of range */
    if (event >= POWER_IC_EVENT_NUM_EVENTS)
    {
        return -EINVAL;
    }

    /* Create a new linked list entry */
    temp = kmalloc(sizeof(POWER_IC_EVENT_CALLBACK_LIST_T), GFP_KERNEL);
    if (temp == NULL)
    {
        return -ENOMEM;
    }

    /* Initialize the fields of the list */
    temp->callback = callback;
    INIT_LIST_HEAD(&temp->list);

    /* Add the entry to the list for the requested event */
    list_add (&temp->list, &(power_ic_events[event]));

    return 0;
}

/*!
 * @brief Unregisters a callback function for a given event
 *
 * The function iterates over the list of callback functions registered for the
 * given event.  If one is found matching the provided callback function pointer,
 * the callback is removed from the list.  If the callback function is included
 * in the list more than once, all instances are removed.  Memory allocated for
 * the list entry is also freed when the entry is removed.
 *
 * @param        event     the event type
 * @param        callback  the callback function to unregister
 *
 * @return always returns 0
 */

int power_ic_event_unsubscribe (POWER_IC_EVENT_T event, POWER_IC_EVENT_CALLBACK_T callback)
{
    struct list_head *p;
    struct list_head *n;
    POWER_IC_EVENT_CALLBACK_LIST_T *t;

    /* Verify that the event number is not out of range */
    if (event >= POWER_IC_EVENT_NUM_EVENTS)
    {
        return -EINVAL;
    }

    /* Find the entry in the list */
    list_for_each_safe (p, n, &(power_ic_events[event]))
    {
        t = list_entry (p, POWER_IC_EVENT_CALLBACK_LIST_T, list);
        if (t->callback == callback)
        {
            /* Remove the entry from the list and free the memory */
            list_del(p);
            kfree(t);
        }
    }

    return 0;
}

/*!
 * @brief Unmasks (enables) an event.
 *
 * This function clears the interrupt mask bit for the given event.  This will
 * allow interrupts to be generated for the event when the event occurs.  Once
 * the event is unmasked, the event callback could be called at any time (including
 * before this function returns back to the caller).
 *
 * @note If the interrupt is pending and masked, when it is unmasked, the callback
 * will be called immediately.
 *
 * @pre While not specifically required, a callback function should generally be
 * registered before unmasking an event.
 *
 * @param        event   the event type to unmask
 *
 * @return 0 on success
 */

int power_ic_event_unmask (POWER_IC_EVENT_T event)
{
    /* Verify that the event number is not out of range */
    if (event >= POWER_IC_EVENT_NUM_EVENTS)
    {
        return -EINVAL;
    }
    
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    else if (event >= POWER_IC_EVENT_EOC_FIRST)
    {
        /* Set the interrupt mask bit to 0 to unmask the interrupt */
        return power_ic_set_reg_bit (
            POWER_IC_REG_EOC_INT_MASK,
            event - POWER_IC_EVENT_EOC_FIRST,
            0);
    }
    else
    {
        /* Set the interrupt mask bit to 0 to unmask the interrupt */
        return power_ic_set_reg_bit (
            POWER_IC_REG_PCAP_IMR,
            event - POWER_IC_EVENT_PCAP_FIRST,
            0);
    }
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    else if (event >= POWER_IC_EVENT_ATLAS_SECOND_REG)
    {
        /* Set the interrupt mask bit to 0 to unmask the interrupt -- From Interrupt Mask 1 */
        return power_ic_set_reg_bit (
            POWER_IC_REG_ATLAS_INT_MASK_1,
            event - POWER_IC_EVENT_ATLAS_SECOND_REG,
            0);
    }
    else
    {
        /* Set the interrupt mask bit to 0 to unmask the interrupt -- From Interrupt Mask 0 */
        return power_ic_set_reg_bit (
            POWER_IC_REG_ATLAS_INT_MASK_0,
            event - POWER_IC_EVENT_ATLAS_FIRST_REG,
            0);
    }
#endif
}

/*!
 * @brief Masks (disables) an event.
 *
 * This function sets the interrupt mask bit for the given event.  This will
 * prevent interrupts from being generated for the event when it occurrs.
 *
 * @param        event   the event type to mask
 *
 * @return 0 on success
 */

int power_ic_event_mask (POWER_IC_EVENT_T event) 
{
    /* Verify that the event number is not out of range */
    if (event >= POWER_IC_EVENT_NUM_EVENTS)
    {
        return -EINVAL;
    }

#ifdef CONFIG_MOT_POWER_IC_PCAP2
    else if (event >= POWER_IC_EVENT_EOC_FIRST)
    {
        /* Set the interrupt mask bit to 1 to mask the interrupt */
        return power_ic_set_reg_bit (
            POWER_IC_REG_EOC_INT_MASK,
            event - POWER_IC_EVENT_EOC_FIRST,
            1);
    }
    else
    {
        /* Set the interrupt mask bit to 1 to mask the interrupt */
        return power_ic_set_reg_bit (
            POWER_IC_REG_PCAP_IMR,
            event - POWER_IC_EVENT_PCAP_FIRST,
            1);
    }
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    else if (event >= POWER_IC_EVENT_ATLAS_SECOND_REG)
    {
        /* Set the interrupt mask bit to 1 to mask the interrupt -- from Interrupt Mask 1 */
        return power_ic_set_reg_bit (
            POWER_IC_REG_ATLAS_INT_MASK_1,
            event - POWER_IC_EVENT_ATLAS_SECOND_REG,
            1);
    }
    else
    {
        /* Set the interrupt mask bit to 1 to mask the interrupt -- from Interrupt Mask 0*/
        return power_ic_set_reg_bit (
            POWER_IC_REG_ATLAS_INT_MASK_0,
            event - POWER_IC_EVENT_ATLAS_FIRST_REG,
            1);
    }
#endif /* ATLAS */
}

/*!
 * @brief Clears an event (interrupt) flag in the power IC.
 *
 * This function clears the interrupt status flag in the power IC for the given
 * event.  This function would generally be called before an event is unmasked to
 * ensure that any previously pending event doesn't cause the callback to
 * called immediately.
 *
 * @note When the event ocurrs, the event flag is automatically cleared after
 * all of the callback functions have been executed.
 *
 * @param        event   the event type to clear.
 *
 * @return 0 on success
 */

int power_ic_event_clear (POWER_IC_EVENT_T event) 
{
    /* Verify that the event number is not out of range */
    if (event >= POWER_IC_EVENT_NUM_EVENTS)
    {
        return -EINVAL;
    }

#ifdef CONFIG_MOT_POWER_IC_PCAP2
    else if (event >= POWER_IC_EVENT_EOC_FIRST)
    {
        /* Set the interrupt status bit to 1 to clear the flag */
        return power_ic_set_reg_bit (
            POWER_IC_REG_EOC_INT_STATUS,
            event - POWER_IC_EVENT_EOC_FIRST,
            1);
    }
    else
    {
        /* Set the interrupt status bit to 1 to clear the flag */
        return power_ic_set_reg_bit (
            POWER_IC_REG_PCAP_ISR,
            event - POWER_IC_EVENT_PCAP_FIRST,
            1);
    }
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    else if (event >= POWER_IC_EVENT_ATLAS_SECOND_REG)
    {
        /* Set the interrupt status bit to 1 to clear the flag -- From Interrupt Status 1 */
        return power_ic_set_reg_bit (
            POWER_IC_REG_ATLAS_INT_STAT_1,
            event - POWER_IC_EVENT_ATLAS_SECOND_REG,
            1);
    }
    else 
    {
        /* Set the interrupt status bit to 1 to clear the flag -- From Interrupt Status 0 */
        return power_ic_set_reg_bit (
            POWER_IC_REG_ATLAS_INT_STAT_0,
            event - POWER_IC_EVENT_ATLAS_FIRST_REG,
            1);
    }
#endif /* ATLAS */
}

/*!
 * @brief Reads an interrupt sense flag from the power IC.
 *
 * This function reads the interrupt sense flag in the power IC for the given
 * event.
 *
 * @param        event   the event type to read
 *
 * @return the state of the interrupt sense or <0 on failure.
 */

int power_ic_event_sense_read (POWER_IC_EVENT_T event)
{
    int retval;
    int value;

    /* Verify that the event number is not out of range */
    if (event >= POWER_IC_EVENT_NUM_EVENTS)
    {
        return -EINVAL;
    }

#ifdef CONFIG_MOT_POWER_IC_PCAP2
    if (event >= POWER_IC_EVENT_EOC_FIRST)
    {
        /* In EMU one-chip, the bit for overvoltage detection in the sense register
         * does not match up with the one in the interrupt register. Since the POWER_IC_EVENT_T
         * enum lines up with the interrupt bit, some translation is needed. Talk about a 
         * marvelous piece of IC design. */
        if(event == POWER_IC_EVENT_EOC_VBUSOV)
        {
            event = 9;
        }
        
        /* For the same reason, there is no sense bit for reverse mode. It's occupied by
         * overvoltage. */
        else if(event == POWER_IC_EVENT_EOC_RVRS_MODE)
        {
            return -EINVAL;
        }
    
        /* Read the interrupt sense bit */
        retval = power_ic_get_reg_value (
            POWER_IC_REG_EOC_INT_SENSE,
            event - POWER_IC_EVENT_EOC_FIRST,
            &value,
            1);
    }
    else
    {
        /* Read the interrupt sense bit */
        retval = power_ic_get_reg_value (
            POWER_IC_REG_PCAP_PSTAT,
            event - POWER_IC_EVENT_PCAP_FIRST,
            &value,
            1);
    }
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    if (event >= POWER_IC_EVENT_ATLAS_SECOND_REG)
    {
        /* Read the interrupt sense 1 bit from register 4 */
        retval = power_ic_get_reg_value (
            POWER_IC_REG_ATLAS_INT_SENSE_1,
            event - POWER_IC_EVENT_ATLAS_SECOND_REG,
            &value,
            1);
    }
    else
    {
        /* Read the interrupt sense 0 bit from register 2 */
        retval = power_ic_get_reg_value (
            POWER_IC_REG_ATLAS_INT_SENSE_0,
            event - POWER_IC_EVENT_ATLAS_FIRST_REG,
            &value,
            1);
    } 
#endif /* ATLAS */

    /* Return the error or the value of the interrupt sense bit */
    return (retval < 0) ? retval : value;
}
