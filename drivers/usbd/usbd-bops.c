/*
 * usbd/usbd-bops.c - USB Device Prototype
 *
 *      Copyright (c) 2004 Belcarra
 *
 * Adapted from earlier work:
 *      Copyright (c) 2002, 2003 Belcarra
 *      Copyright (c) 2000, 2001, 2002 Lineo
 *      Copyright (c) 2001 Hewlett Packard
 *
 * By: 
 *      Stuart Lynne <sl@belcarra.com>, 
 *      Tom Rushworth <tbr@belcarra.com>, 
 *      Bruce Balden <balden@belcarra.com>
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/*
 * Copyright (C) 2004 - Motorola
 *
 * 2004-Dec-06 - Modified for EZXBASE By Zhao Liang <w20146@motorola.com>
 *
 */

#include <linux/config.h>
#include <linux/module.h>

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/smp_lock.h>
#include <linux/ctype.h>
#include <linux/timer.h>
#include <linux/string.h>

#include "usbd-chap9.h"
#include "usbd-mem.h"
#include "usbd.h"
#include "usbd-func.h"
#include "usbd-bus.h"

#include <linux/ezxusbd.h>

/* Private defs from usbd-func.c *************************************************************** */
/* Function driver enable/disable
 *
 * Called by usbd_enable_function/usbd_disable_function to call the selected
 * function drivers function_enable or function_disable function.
 */
int usbd_function_enable (struct usb_bus_instance *, struct usb_function_instance *);
void usbd_function_disable(struct usb_function_instance *);
void usbd_func_event_irq(struct usb_bus_instance *, struct usb_function_instance *, usb_device_event_t , int );
int usbd_strings_init (void);
void usbd_strings_exit(void);
void usbd_urb_callback (struct urb *urb, int rc);

/* List support functions ******************************************************************** */

/*
 * Structure member address manipulation macros.
 * These are used by client code (code using the urb_link routines), since
 * the urb_link structure is embedded in the client data structures.
 *
 * Note: a macro offsetof equivalent to member_offset is defined in stddef.h
 *       but this is kept here for the sake of portability.
 *
 * p2surround returns a pointer to the surrounding structure given
 * type of the surrounding structure, the name memb of the structure
 * member pointed at by ptr.  For example, if you have:
 * 
 *      struct foo {
 *          int x;
 *          float y;
 *          char z;
 *      } thingy;
 *  
 *      char *cp = &thingy.z;
 *
 * then
 *      &thingy == p2surround(struct foo, z, cp)
 */

#define _cv_(ptr)                 ((char*)(void*)(ptr))
#define member_offset(type,memb)  (_cv_(&(((type*)0)->memb))-(char*)0)
#define p2surround(type,memb,ptr) ((type*)(void*)(_cv_(ptr)-member_offset(type,memb)))

static inline struct usb_function_driver *list_entry_function (const struct list_head *le)
{
	return list_entry (le, struct usb_function_driver, drivers);
}


/*
 * Append an urb_link (or a whole list of
 * urb_links) to the tail of another list
 * of urb_links.
 */
static __inline__ void urb_append_irq (urb_link * hd, struct urb *urb)
{
	if (hd && urb) {
		urb_link *new = &urb->link;

#ifdef C2L_SINGLETON_ONLY
		// This _assumes_ the new urb is a singleton,
		// but allows it to have an uninitialized link.
		//printk(KERN_DEBUG"urb_append: hd: %p n:%p p:%p new: %p n:%p p:%p\n", hd, hd->next, hd->prev, new, new->next, new->prev);

		new->prev = hd->prev;
		new->next = hd;

		hd->prev->next = new;
		hd->prev = new;
#else
		// This allows the new urb to be a list of urbs,
		// with new pointing at the first, but the link
		// must be initialized.
		// Order is important here...
		urb_link *pul = hd->prev;
		new->prev->next = hd;
		hd->prev = new->prev;
		new->prev = pul;
		pul->next = new;
#endif
	}
}

/*
 * Return the first urb_link in a list with a distinguished
 * head "hd", or NULL if the list is empty.  This will also
 * work as a predicate, returning NULL if empty, and non-NULL
 * otherwise.
 *
 * Called from interrupt.
 */
static __inline__ urb_link *first_urb_link_irq (urb_link * hd)
{
	urb_link *nx;
        return (!hd || !(nx = hd->next) || (nx == hd)) ? NULL : nx;
}

/*
 * Return the first urb in a list with a distinguished
 * head "hd", or NULL if the list is empty.
 *
 * Called from interrupt.
 */
static __inline__ struct urb *first_urb_irq (urb_link * hd)
{
	urb_link *nx;
	return (!(nx = first_urb_link_irq (hd))) ? NULL : p2surround (struct urb, link, nx);
}

/*
 * Detach and return the first urb in a list with a distinguished
 * head "hd", or NULL if the list is empty.
 */
struct urb *usbd_first_urb_detached_irq (urb_link * hd)
{
	struct urb *urb;
	urb_link *ul;
	RETURN_NULL_IF (!(urb = first_urb_irq (hd)));
	ul = &urb->link;
	ul->next->prev = ul->prev;
	ul->prev->next = ul->next;
	ul->prev = ul->next = ul;
	return urb;
}

static __inline__ void urb_append(urb_link *hd, struct urb *urb)
{
	unsigned long flags;
	local_irq_save (flags);
	urb_append_irq(hd, urb);
	local_irq_restore (flags);
}

/* usbd_urb_sent_finished_irq - tell function that an urb has been transmitted.
 *
 * Must be called from an interrupt or with interrupts disabled.
 *
 * Used by a USB Bus driver to pass a sent urb back to the function 
 * driver via the endpoints done queue.
 */
void usbd_urb_sent_finished_irq (struct urb *urb, int rc)
{
        //printk(KERN_INFO"%s: urb: %p\n", __FUNCTION__, urb);
        urb->status = rc;
        urb->jiffies = jiffies;
        if (USBD_OK != urb->bus->status) 
                usbd_urb_callback(urb, urb->status);
        else {
                urb_append_irq (&(urb->endpoint->done), urb);
                RETURN_IF ( urb->bus->device_bh.sync);
                queue_task (&urb->bus->device_bh, &tq_immediate);
                mark_bh (IMMEDIATE_BH);
        }
}


/* usbd_urb_recv_finished_irq - tell function that an urb has been received.
 *
 * Must be called from an interrupt or with interrupts disabled.
 *
 * Used by a USB Bus driver to pass a sent urb back to the function 
 * driver via the endpoints done queue.
 */
void usbd_urb_recv_finished_irq (struct urb *urb, int rc)
{
        urb->status = rc;
        urb->jiffies = jiffies;
        if (USBD_OK != urb->bus->status) 
                usbd_urb_callback (urb, rc);
        else {
                urb_append_irq (&(urb->endpoint->rcv), urb);
                RETURN_IF (urb->bus->device_bh.sync);
                queue_task (&urb->bus->device_bh, &tq_immediate);
                mark_bh (IMMEDIATE_BH);
        }
}


/* first_urb_detached -
 *
 * Detach and return the first urb in a list with a distinguished
 * head "hd", or NULL if the list is empty.
 *
 */
static __inline__ struct urb *first_urb_detached (urb_link * hd)
{
	struct urb *urb;
	unsigned long flags;
	local_irq_save (flags);
	urb = usbd_first_urb_detached_irq (hd);
	local_irq_restore (flags);
	return urb;
}


/* Private defs from usbd-fops.c *************************************************************** */


extern struct usb_function_driver ep0_driver;
extern struct list_head usbd_function_drivers;
extern struct usb_bus_instance *usbd_bus_instance;

#define LANGID_ENGLISH          "\011"
#define LANGID_US_ENGLISH       "\004"
#define LANGIDs  LANGID_US_ENGLISH LANGID_ENGLISH

/* usbd_flush_endpoint_irq - flush urbs from endpoint
 *
 * Iterate across the approrpiate tx or rcv list and cancel any outstanding urbs.
 */
void usbd_flush_endpoint_irq (struct usb_endpoint_instance *endpoint)
{
	struct urb *urb;
        //printk(KERN_INFO"%s: bEndpointAddress: %d\n", __FUNCTION__, endpoint->bEndpointAddress);
	endpoint->last = endpoint->sent = 0;	// w20146, Jan 26, 2005
        if ((urb = endpoint->tx_urb)) 
                usbd_cancel_urb_irq(urb);

        for (; (urb = usbd_first_urb_detached_irq (&endpoint->tx)); usbd_cancel_urb_irq(urb));

        if ((urb = endpoint->rcv_urb)) 
                usbd_cancel_urb_irq(urb);

        for (; (urb = usbd_first_urb_detached_irq (&endpoint->rdy)); usbd_cancel_urb_irq(urb));
}

/* usbd_flush_endpoint - flush urbs from endpoint
 *
 * Iterate across the approrpiate tx or rcv list and cancel any outstanding urbs.
 */
void usbd_flush_endpoint (struct usb_endpoint_instance *endpoint)
{
        unsigned long flags;
        local_irq_save (flags);
        usbd_flush_endpoint_irq(endpoint);
        local_irq_restore (flags);
}

/* usbd_endpoint_halted
 *
 * Return non-zero if endpoint is halted.
 */
int usbd_endpoint_halted (struct usb_function_instance *function, int endpoint)
{
        //printk(KERN_INFO"%s:\n", __FUNCTION__);
        return function->bus->driver->bops->endpoint_halted (function->bus, endpoint);
}


/* usbd_device_feature - set usb device feature
 *
 * Return non-zero if error
 */
int usbd_device_feature (struct usb_function_instance *function, int endpoint, int feature)
{
        //printk(KERN_INFO"%s:\n", __FUNCTION__);
        return function->bus->driver->bops->device_feature (function->bus, endpoint, feature);
}

/* usbd_cancel_urb_irq - cancel an urb being sent
 *
 * Return non-zero if error
 */
int usbd_cancel_urb_irq (struct urb *urb)
{
        //printk(KERN_INFO"%s: urb: %p\n", __FUNCTION__, urb);
        // XXX should we do usbd_dealloc_urb(urb);
        return urb->bus->driver->bops->cancel_urb_irq (urb);
}

/* usbd_send_urb - submit a urb to send
 *
 * Used by a USB Function driver to submit data to be sent in an urb to the
 * appropriate USB Bus driver via the endpoints transmit queue.
 *
 * Return non-zero if error
 */
int usbd_send_urb (struct urb *urb)
{
        //printk(KERN_INFO"%s: urb: %p length: %d\n", __FUNCTION__, urb, urb->actual_length);
	//RETURN_EINVAL_IF (USBD_OK != urb->bus->status);
	RETURN_EINVAL_IF (urb->endpoint->bEndpointAddress && (USBD_OK != urb->bus->status));
	urb->status = SEND_IN_QUEUE;
	urb->jiffies = jiffies;
	urb_append (&(urb->endpoint->tx), urb);
        return urb->bus->driver->bops->start_endpoint_in(urb->bus, urb->endpoint);
}

/* usbd_start_recv - recycle a received urb
 *
 * Used by a USB Function interface driver to recycle an urb.
 *
 * Return non-zero if error
 */
int usbd_start_recv (struct urb *urb)
{
        //printk(KERN_INFO"%s: urb: %p\n", __FUNCTION__, urb);
	RETURN_EINVAL_IF (urb->endpoint->bEndpointAddress && (USBD_OK != urb->bus->status));
	urb->actual_length = 0;
	urb->status = RECV_IN_QUEUE;
	urb->jiffies = jiffies;
	urb_append (&(urb->endpoint->rdy), urb);
        urb->bus->driver->bops->start_endpoint_out(urb->bus, urb->endpoint);
        //printk(KERN_INFO"%s: finis\n", __FUNCTION__, urb);
        return 0;
}

/* usbd_alloc_string_zero - allocate a string descriptor and return index number
 *
 * Find an empty slot in index string array, create a corresponding descriptor
 * and return the slot number.
 */
__u8 usbd_alloc_string_zero (char *str)
{
	__u8 bLength;
	__u16 *wData;
	struct usb_string_descriptor *string;

        RETURN_ZERO_IF(usb_strings[0] != NULL);

        bLength = sizeof (struct usb_string_descriptor) + strlen (str);

        RETURN_ZERO_IF(!(string = ckmalloc (bLength, GFP_KERNEL)));

        string->bLength = bLength;
        string->bDescriptorType = USB_DT_STRING;

        for (wData = string->wData; *str; str += 2) {
                *wData = (__u16) ((str[0] << 8 | str[1]));
                wData++;
        }
        usb_strings[0] = string; // store in string index array
        return 0;
}


int usbd_strings_init (void)
{
        //printk(KERN_INFO"%s: usb_strings: %p\n", __FUNCTION__, usb_strings);
	RETURN_ZERO_IF(usb_strings);
        usbd_maxstrings = MIN(usbd_maxstrings, 254);
	
	RETURN_EINVAL_IF (!(usb_strings = ckmalloc (sizeof (struct usb_string_descriptor *) * usbd_maxstrings, GFP_KERNEL)));
#ifdef BLAN_IP_ADDR_GNPO
	// allocate space for the language ID
	if (usbd_update_string_at_index (STRINDEX_LANGID, LANGIDs) != 0) {
#else
	if (usbd_alloc_string_zero (LANGIDs) != 0) {
#endif
		lkfree (usb_strings);
		return -1;
	}
	return 0;
}

void usbd_strings_exit(void)
{
        int i;
        //printk(KERN_INFO"%s: usb_strings: %p\n", __FUNCTION__, usb_strings);
        RETURN_IF (!usb_strings);
        for (i = 0; i < usbd_maxstrings; i++) 
                usbd_dealloc_string(i);
        lkfree (usb_strings);
        usb_strings = NULL;
}

static usb_device_state_t event_states[DEVICE_CLOSE] = {
        STATE_UNKNOWN, STATE_INIT, STATE_ATTACHED, STATE_POWERED,
        STATE_DEFAULT, STATE_ADDRESSED, STATE_CONFIGURED, STATE_UNKNOWN,
        STATE_UNKNOWN, STATE_UNKNOWN, STATE_ADDRESSED, STATE_SUSPENDED,
        STATE_UNKNOWN, STATE_POWERED, STATE_ATTACHED, 
};

static usb_device_status_t event_status[DEVICE_CLOSE+1] = {
        USBD_UNKNOWN,   // DEVICE_UNKNOWN
        USBD_OPENING,   // DEVICE_INIT
        USBD_OPENING,   // DEVICE_CREATE
        USBD_OPENING,   // DEVICE_HUB_CONFIGURED
        USBD_RESETING,  // DEVICE_RESET
        USBD_OK,        // DEVICE_ADDRESS_ASSIGNED
        USBD_OK,        // DEVICE_CONFIGURED
        USBD_OK,        // DEVICE_SET_INTERFACE
        USBD_OK,        // DEVICE_SET_FEATURE
        USBD_OK,        // DEVICE_CLEAR_FEATURE
        USBD_OK,        // DEVICE_DE_CONFIGURED
        USBD_SUSPENDED, // DEVICE_BUS_INACTIVE
        USBD_OK,        // DEVICE_BUS_ACTIVITY
        USBD_RESETING,  // DEVICE_POWER_INTERRUPTION
        USBD_RESETING,  // DEVICE_HUB_RESET
        USBD_CLOSING,   // DEVICE_DESTROY
        USBD_CLOSED,    // DEVICE_CLOSE
};

/* usbd_device_event_irq - called to respond to various usb events
 *
 * Used by a Bus driver to indicate an event.
 */
#ifdef CONFIG_ARCH_EZX
#define POWER_CURRENT_UNIT	2	// multiplue of 2mA from USB SPEC
#define POWER_CHARGING_CURRENT	500
#endif

void usbd_bus_event_irq (struct usb_bus_instance *bus, usb_device_event_t event, int data)
{
        RETURN_IF(!bus);

        //printk(KERN_INFO"%s:\n", __FUNCTION__);
        //printk(KERN_INFO"%s: --> event: %d status: %d state: %d\n", __FUNCTION__, event, bus->status, bus->device_state);
	switch (event) {
	case DEVICE_BUS_INACTIVE:
                bus->suspended_state = bus->device_state;
                //printk(KERN_INFO"%s: INACTIVE\n", __FUNCTION__);
                /* FALL THROUGH */
        default:
                bus->device_state = event_states[event];
                //printk(KERN_INFO"%s: DEFAUL\n", __FUNCTION__);
                break;
	case DEVICE_UNKNOWN:
                //printk(KERN_INFO"%s: UNKNOWN\n", __FUNCTION__);
		break;
	case DEVICE_BUS_ACTIVITY:
                bus->device_state = bus->suspended_state;
#ifdef CONFIG_ARCH_EZX
		bus->suspended_state = STATE_INIT;
#endif
                //printk(KERN_INFO"%s: ACTIVITY\n", __FUNCTION__);
		break;

	case DEVICE_SET_INTERFACE:
	case DEVICE_SET_FEATURE:
	case DEVICE_CLEAR_FEATURE:
                //printk(KERN_INFO"%s: SET\n", __FUNCTION__);
		break;
	}

        switch (event) {
        case DEVICE_BUS_ACTIVITY:
        case DEVICE_BUS_INACTIVE:
                BREAK_IF(USBD_CLOSING != bus->status);
                /* FALL THROUGH */
        default:
                bus->status = event_status[event];
                //printk(KERN_INFO"%s: DEFAULT\n", __FUNCTION__);
                break;
        }
        //printk(KERN_INFO"%s: <-- event: %d status: %d state: %d\n", __FUNCTION__, event, bus->status, bus->device_state);

        // if we lost configuration then get rid of alternate settings
        if ((bus->device_state != STATE_CONFIGURED) && bus->bNumInterfaces && bus->alternates) {
                bus->bNumInterfaces = 0;
                lkfree(bus->alternates);
                bus->alternates = NULL;
        }
        bus->driver->bops->device_event (bus, event, data);
        usbd_func_event_irq (bus, bus->ep0, event, data);
        usbd_func_event_irq (bus, bus->function_instance, event, data);

#ifdef CONFIG_ARCH_EZXBASE
	// report event after function driver reports to make the 500mA as the last indicator
	switch(event)
	{
		case DEVICE_CONFIGURED:
			if(1)
			{
#ifndef CONFIG_USBD_ISP_BUS

				int power_current;
				struct usb_configuration_descriptor *cfg_desc;

				cfg_desc = (&bus->function_instance->function_driver->configuration_instance_array[0])->configuration_descriptor;
				power_current = (int)(cfg_desc->bMaxPower);
				//printk(KERN_INFO"%s: DEVICE_CONFIGURED emu charger %d mA\n", __FUNCTION__, power_current * POWER_CURRENT_UNIT);
				if(power_current == POWER_CURRENT_500MA_SETTING)
				{
					queue_motusbd_event(USB_CURRENT_500MA);
				}
				else if(power_current == POWER_CURRENT_100MA_SETTING)
				{
					queue_motusbd_event(USB_CURRENT_100MA);
				}
				else
				{
					queue_motusbd_event(USB_CURRENT_0MA);
				}
#else
				/* If the Raw_ConfigurationValue is zero, it means a invalid configuration value and the current drawing 
				   shall also be 0 .
				   TODO: If the two lines of codes below are applicable to Bulverde USB 1.1, we need further verification.
				   USBD should be indenpent of low level HW driver.
				 */
				if (!(bus->Raw_ConfigurationValue))  {
					queue_motusbd_event(USB_CURRENT_0MA);
				} else {
					int power_current;
					struct usb_configuration_descriptor *cfg_desc;

					cfg_desc = (&bus->function_instance->function_driver->configuration_instance_array[0])->configuration_descriptor;
					power_current = (int)(cfg_desc->bMaxPower);
					//printk(KERN_INFO"%s: DEVICE_CONFIGURED emu charger %d mA\n", __FUNCTION__, power_current * POWER_CURRENT_UNIT);
					if(power_current == POWER_CURRENT_500MA_SETTING) {
						queue_motusbd_event(USB_CURRENT_500MA);
					} else if(power_current == POWER_CURRENT_100MA_SETTING)	{
						queue_motusbd_event(USB_CURRENT_100MA);
					} else	{
						queue_motusbd_event(USB_CURRENT_0MA);
					}
				}
#endif
			}
			break;
		case DEVICE_DE_CONFIGURED:
			//printk(KERN_INFO"%s: DEVICE_DE_CONFIGURED tell BP cable is removed to stop NetMonitor\n", __FUNCTION__);
			// what key event?
			break;

		default:
			break;
	}
	
#endif
}


/* usbd_bus_event
 */
void usbd_bus_event (struct usb_bus_instance *bus, usb_device_event_t event, int data)
{
	unsigned long flags;
	local_irq_save (flags);
	usbd_bus_event_irq (bus, event, data);
	local_irq_restore (flags);
}


/* usbd_attached - return cable status
 *
 * Return non-zero if cable attached
 */
int usbd_attached (struct usb_bus_instance *bus)
{
	return bus->driver->bops->bus_attached (bus);
}

/* usbd_connected - return pullup resistor control status
 *  *
 *   * Return non-zero if pullup enabled
 *    */
int usbd_connected (struct usb_bus_instance *bus)
{
        return bus->driver->bops->bus_connected (bus);
}


/* usbb bus bottom half ********************************************************************** */

/*
 * Note that all of the functions from here to the end of the file protect against 
 * overlapped operation using the usbd_bus_sem
 *
 *      usbd_device_bh
 *
 *      usbd_register_bus
 *      usbd_deregister_bus
 *      usbd_enable_function_irq
 *      usbd_disable_function
 */
DECLARE_MUTEX(usbd_bus_sem);


/* usbd_device_bh - 
 *
 * Bottom half handler to process sent or received urbs.
 */
void usbd_device_bh (void *data)
{
        int i;
        struct usb_bus_instance *bus = data;
        struct usb_endpoint_instance *endpoint;

        RETURN_IF (!bus || !(endpoint = bus->endpoint_array));
        down(&usbd_bus_sem);
        // process received and sent urbs
        for (i = 0; i < bus->endpoints; i++, endpoint++) {
                struct urb *urb;
                for (; (urb = first_urb_detached (&endpoint->rcv )); usbd_urb_callback (urb, urb->status));
                for (; (urb = first_urb_detached (&endpoint->done)); usbd_urb_callback (urb, urb->status));
        }
        if (USBD_CLOSING == bus->status) 
                bus->device_bh.data = NULL;
        up(&usbd_bus_sem);
}


/* usb-device USB BUS INTERFACE generic functions ******************************************** */

/*
 * Initialize an urb_link to be a single element list.
 * If the urb_link is being used as a distinguished list head
 * the list is empty when the head is the only link in the list.
 */
static __inline__ void urb_link_init (urb_link * ul)
{
        ul->prev = ul->next = ul;
}

/* usbd_register_bus - called by a USB BUS INTERFACE driver to register a bus driver
 *
 * Used by a USB Bus interface driver to register itself with the usb device layer.
 *
 * Return non-zero if error
 */
struct usb_bus_instance *usbd_register_bus (struct usb_bus_driver *driver)
{
        int i;
        struct usb_bus_instance *bus = NULL;

        //printk(KERN_INFO"%s: DOWN USBD_BUS_SEM\n", __FUNCTION__);
        down(&usbd_bus_sem);
	MOD_INC_USE_COUNT;  // QQQ should this be before the down()?

        THROW_IF(usbd_bus_instance, error);
        THROW_IF((bus = ckmalloc (sizeof (struct usb_bus_instance), GFP_ATOMIC)) == NULL, error);

        bus->driver = driver;
        bus->endpoints = bus->driver->max_endpoints;

        THROW_IF(!(bus->endpoint_array = ckmalloc(sizeof (struct usb_endpoint_instance) * bus->endpoints, GFP_ATOMIC)), error); 

        for (i = 0; i < bus->endpoints; i++) {
                struct usb_endpoint_instance *endpoint = bus->endpoint_array + i;
                endpoint->physical_endpoint = i;
                urb_link_init (&endpoint->rcv);
                urb_link_init (&endpoint->rdy);
                urb_link_init (&endpoint->tx);
                urb_link_init (&endpoint->done);
        }

        bus->admin[usbd_admin_enable] = driver->bops->bus_enable;
        bus->admin[usbd_admin_disable] = driver->bops->bus_disable;
        bus->admin[usbd_admin_disconnect] = driver->bops->bus_disconnect;
        bus->admin[usbd_admin_connect] = driver->bops->bus_connect;
        bus->admin[usbd_admin_pm_off] = driver->bops->bus_pm_off;
        bus->admin[usbd_admin_pm_on] = driver->bops->bus_pm_on;
        bus->admin[usbd_admin_serial_number] = driver->bops->bus_serial_number;

	THROW_IF((bus->ep0 = ckmalloc (sizeof (struct usb_function_instance), GFP_ATOMIC)) == NULL, error)
	bus->ep0->function_driver = &ep0_driver;
        bus->ep0->endpointsRequested = 1;
        THROW_IF(!(bus->ep0->endpoint_map_array = ckmalloc(sizeof(struct usb_endpoint_map) * 1, GFP_KERNEL)), error);

	bus->device_state = STATE_CREATED;
	bus->status = USBD_OPENING;

        usbd_bus_instance = bus;

        CATCH(error) {
                if (bus) {
                        if (bus->endpoint_array) 
                                lkfree(bus->endpoint_array);
                	bus->endpoints = 0;
                        lkfree(bus);
                }
                printk(KERN_INFO"%s: FAILED\n", __FUNCTION__);
                bus =  NULL;
		MOD_DEC_USE_COUNT;
        }

        up(&usbd_bus_sem);
        //printk(KERN_INFO"%s: UP USBD_BUS_SEM\n", __FUNCTION__);
	return bus;
}

/* usbd_deregister_bus - called by a USB BUS INTERFACE driver to deregister a bus driver
 *
 * Used by a USB Bus interface driver to de-register itself with the usb device
 * layer.
 */
void usbd_deregister_bus (struct usb_bus_instance *bus)
{
        //printk(KERN_INFO"%s: DOWN USBD_BUS_SEM\n", __FUNCTION__);
        down(&usbd_bus_sem);

        usbd_bus_instance = NULL;

        if (bus->ep0) {
                if (bus->ep0->endpoint_map_array)
                        lkfree(bus->ep0->endpoint_map_array);
                lkfree(bus->ep0);
        }

        lkfree (bus->arg);
	lkfree (bus->endpoint_array);
        bus->endpoints = 0;
	lkfree (bus);
	MOD_DEC_USE_COUNT;  // QQQ should this be after the up()?
        up(&usbd_bus_sem);
        //printk(KERN_INFO"%s: UP USBD_BUS_SEM\n", __FUNCTION__);
}


/* usb-device USB Device generic functions *************************************************** */

/* usbd_enable_function_irq - called to enable the desired function
 *
 * Used by a USB Bus interface driver to create a virtual device.
 *
 * Return non-zero if error
 */
int usbd_enable_function_irq (struct usb_bus_instance *bus, char *arg)
{
	struct usb_function_instance *function = NULL;
	struct list_head *lhd;
        int len = 0;
        int i;
        int rc = -EINVAL;
        int epn;

        //printk(KERN_INFO"%s: DOWN USBD_BUS_SEM\n", __FUNCTION__);
        down(&usbd_bus_sem);

        if (arg && bus->arg) 
                lkfree(bus->arg);

        if (arg) {
                bus->arg = lstrdup(arg);
                len = strlen(arg);
        }

	// initialize the strings pool
	THROW_IF(usbd_strings_init (), error);

        for (i = 1; i < bus->endpoints; i++) {
                struct usb_endpoint_instance *endpoint = bus->endpoint_array + i;
                endpoint->rcv_urb = endpoint->tx_urb = NULL;
        }               

	list_for_each (lhd, &usbd_function_drivers) {
		struct usb_function_driver *function_driver;
		function_driver = list_entry_function (lhd);

                //printk(KERN_INFO"%s: check: [%s] %s\n", __FUNCTION__,
		//       (arg?arg:""), function_driver->name);

                // mode - single, either the first or a named function
                CONTINUE_IF(arg && len && strncmp(function_driver->name, arg, len));

                //printk(KERN_INFO"%s: found: %s\n", __FUNCTION__, function_driver->name);

                THROW_IF (!(function = ckmalloc (sizeof (struct usb_function_instance), GFP_ATOMIC)), error);

                function->function_driver = function_driver;  // XXX
                THROW_IF(!(function->endpoint_map_array = bus->driver->bops->request_endpoints(
                                        function->function_driver->device_description->endpointsRequested,
                                        function->function_driver->device_description->requestedEndpoints
                                        )),error_free);
                function->endpointsRequested = function->function_driver->device_description->endpointsRequested;
                THROW_IF(usbd_function_enable (bus, function), error_free);
                break;
	}

        if (NULL == function && NULL != arg && 0 != len) {
                //printk(KERN_INFO"Unknown function driver (len=%d) [%s], known drivers:\n",len,arg);
                list_for_each (lhd, &usbd_function_drivers) {
                        //struct usb_function_driver *function_driver = list_entry_function (lhd);
                        //printk(KERN_INFO"   [%s]\n", function_driver->name);
                }
        }

        THROW_IF (!(bus->function_instance = function), error);

        THROW_IF(bus->driver->bops->set_endpoints( function->function_driver->device_description->endpointsRequested,
                                        function->endpoint_map_array), error);
	// device bottom half
	bus->device_bh.routine = usbd_device_bh;
	bus->device_bh.data = bus;
	bus->status = USBD_OK;
	bus->bus_state = usbd_bus_state_enabled;
        bus->ep0->endpoint_map_array->endpoint = bus->endpoint_array;
        THROW_IF (usbd_function_enable (bus, bus->ep0), error);

        //printk(KERN_INFO"%s: %d endpoint: %p map: %p\n", __FUNCTION__, 0,
        //                bus->ep0->endpoint_map_array->endpoint, 
        //                bus->ep0->endpoint_map_array);

        // iterate across the logical endpoint map to copy appropriate information 
        // into the physical endpoint instance array

        for (epn = 0; epn < bus->function_instance->endpointsRequested; epn++) {

                struct usb_endpoint_map *endpoint_map = bus->function_instance->endpoint_map_array + epn;
                int physicalEndpoint = endpoint_map->physicalEndpoint[0];
                struct usb_endpoint_instance *endpoint = bus->endpoint_array + physicalEndpoint;

                //printk(KERN_INFO"%s: %d endpoint: %p map: %p bEndpointAddress: %02x\n", __FUNCTION__, 
                //                epn, endpoint, endpoint_map, endpoint_map->bEndpointAddress[0]);

                endpoint_map->endpoint = endpoint;
                endpoint->bEndpointAddress = endpoint_map->bEndpointAddress[0];
                endpoint->bmAttributes = endpoint_map->bmAttributes[0];

                switch(endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) {
                case USB_DIR_IN:
                        endpoint->wMaxPacketSize = endpoint_map->wMaxPacketSize[0];
                        endpoint->last = 0;
                        endpoint->tx_urb = NULL;
                        break;

                case USB_DIR_OUT:
                        endpoint->rcv_transferSize = endpoint_map->transferSize[0];
                        endpoint->wMaxPacketSize = endpoint_map->wMaxPacketSize[0];
                        endpoint->rcv_urb = NULL;
                        break;
                }
        }

        rc = 0;

        CATCH(error_free) {
		lkfree(function);
		THROW_IF(1, error);
        }

        CATCH(error) {
                printk(KERN_INFO"%s: FAILED\n", __FUNCTION__);
                usbd_strings_exit();
        }
        up(&usbd_bus_sem);
        //printk(KERN_INFO"%s: UP USBD_BUS_SEM\n", __FUNCTION__);
        return rc;
}

/* usbd_disable_function - called to disable the current function
 *
 * Used by a USB Bus interface driver to destroy a virtual device.
 */
void usbd_disable_function (struct usb_bus_instance *bus)
{
        //printk(KERN_INFO"%s: DOWN USBD_BUS_SEM\n", __FUNCTION__);
        down(&usbd_bus_sem);
	// prevent any more bottom half scheduling
        bus->status = USBD_CLOSING;
        up(&usbd_bus_sem);
        //printk(KERN_INFO"%s: UP USBD_BUS_SEM\n", __FUNCTION__);

	// wait for pending device bottom half to finish
	while (bus->device_bh.data /*|| device->function_bh.data */) {

                //printk(KERN_INFO"%s: waiting for usbd_device_bh %ld %p interrupt: %d\n", __FUNCTION__, 
                //                bus->device_bh.sync, bus->device_bh.data, in_interrupt());

                // This can probably be either, but for consistency's sake...
                queue_task(&bus->device_bh, &tq_immediate);
                mark_bh (IMMEDIATE_BH);
                // schedule_task(&device->device_bh);

		schedule_timeout (2000 * HZ);
	}

        //printk(KERN_INFO"%s: DOWN USBD_BUS_SEM\n", __FUNCTION__);
        down(&usbd_bus_sem);

	// tell the function driver to close
	usbd_function_disable (bus->ep0);
	usbd_function_disable (bus->function_instance);

        // free alternates memory
        if (/*bus->bNumInterfaces &&*/ bus->alternates) {
                bus->bNumInterfaces = 0;
                lkfree(bus->alternates);
                bus->alternates = NULL;
        }
        if (bus->function_instance) {
                if (bus->function_instance->endpoint_map_array) 
                        lkfree(bus->function_instance->endpoint_map_array);
                lkfree(bus->function_instance);
                bus->function_instance = NULL;
        }
        usbd_strings_exit();
        bus->status = USBD_CLOSED;
        up(&usbd_bus_sem);
        //printk(KERN_INFO"%s: UP USBD_BUS_SEM\n", __FUNCTION__);
}

usb_device_state_t usbd_device_state(struct usb_function_instance *function)
{
        //printk(KERN_INFO"%s: %d\n", __FUNCTION__, function->bus->device_state);
        return (function && function->bus) ? function->bus->device_state : STATE_UNKNOWN;
}
usb_device_state_t usbd_bus_state(struct usb_function_instance *function)
{
        //printk(KERN_INFO"%s: %d\n", __FUNCTION__, function->bus->bus_state);
        return (function && function->bus) ? function->bus->bus_state : usbd_bus_state_unknown;
}
usb_device_status_t usbd_bus_status(struct usb_function_instance *function)
{
        //printk(KERN_INFO"%s: %d\n", __FUNCTION__, function->bus->status);
        return (function && function->bus) ?  function->bus->status : USBD_UNKNOWN;
}

EXPORT_SYMBOL(usbd_register_bus);
EXPORT_SYMBOL(usbd_deregister_bus);
EXPORT_SYMBOL(usbd_enable_function_irq);
EXPORT_SYMBOL(usbd_disable_function);
EXPORT_SYMBOL(usbd_attached);
EXPORT_SYMBOL(usbd_send_urb);
EXPORT_SYMBOL(usbd_start_recv);
EXPORT_SYMBOL(usbd_flush_endpoint);
EXPORT_SYMBOL(usbd_cancel_urb_irq);
EXPORT_SYMBOL(usbd_bus_event_irq);
EXPORT_SYMBOL(usbd_bus_event);
EXPORT_SYMBOL(usbd_urb_sent_finished_irq);
EXPORT_SYMBOL(usbd_urb_recv_finished_irq);
EXPORT_SYMBOL(usbd_device_state);
EXPORT_SYMBOL(usbd_bus_state);
EXPORT_SYMBOL(usbd_bus_status);
EXPORT_SYMBOL(usbd_first_urb_detached_irq);

