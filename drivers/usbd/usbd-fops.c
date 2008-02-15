/*
 * usbd/usbd-fops.c - USB Function support
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
 * Copyright (C) 2003 - Motorola
 *
 * 2003-Dec-06 - Modified for EZXBASE By Zhao Liang <w20146@motorola.com>
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


struct usb_string_descriptor **usb_strings;

extern struct list_head usbd_function_drivers;
extern struct usb_bus_instance *usbd_bus_instance;


/* usbd_get_string - find and return a string descriptor
 *
 * Find an indexed string and return a pointer to a it.
 */
struct usb_string_descriptor *usbd_get_string (__u8 index)
{
	RETURN_NULL_IF (index >= usbd_maxstrings);
	return usb_strings[index];
}

#ifdef BLAN_IP_ADDR_GNPO
/* usbd_update_string_at_index - change the string descriptor at the specified index
 *
 * create a corresponding descriptor and replace the existing item at the specified index
 */
__u8 usbd_update_string_at_index (__u8 index, char *str)
{
	struct usb_string_descriptor *string;
	__u8 bLength;
	__u16 *wData;

	RETURN_ZERO_IF(!str || !strlen (str));
	RETURN_ZERO_IF(index >= usbd_maxstrings);

	//printk (KERN_INFO "%s: index %d, str: %s\n", __FUNCTION__, index, str);
	if(index == 0)
		bLength = sizeof (struct usb_string_descriptor) + strlen (str);
	else
		bLength = sizeof (struct usb_string_descriptor) + 2 * strlen (str);

	// if a string descriptor already exists at that location
	if ((string = usb_strings[index]) != NULL)
	{
		// free the old string
		lkfree(string);
	}

	RETURN_ZERO_IF(!(string = ckmalloc (bLength, GFP_KERNEL)));

	string->bLength = bLength;
	string->bDescriptorType = USB_DT_STRING;

	if(index == 0)
	{
		for (wData = string->wData; *str; str += 2) 
		{
			*wData = (__u16) ((str[0] << 8 | str[1]));
			wData++;
		}
	}
	else
	{
		for (wData = string->wData; *str;) 
			*wData++ = (__u16) (*str++);
	}

	usb_strings[index] = string;       // store in string index array

	return index;
}
#endif

/* usbd_alloc_string - allocate a string descriptor and return index number
 *
 * Find an empty slot in index string array, create a corresponding descriptor
 * and return the slot number.
 */
#ifdef BLAN_IP_ADDR_GNPO
__u8 usbd_alloc_string (char *str)
{
	int i;

	RETURN_ZERO_IF(!str || !strlen (str));

	// find an empty string descriptor slot
	// "0" is consumed by the language ID, 
	// "1" is the optional IP address
	// "2" is for the product string (as required by the PST group)
	for (i = 3; i < usbd_maxstrings; i++) 
	{
		CONTINUE_IF (usb_strings[i] != NULL);

		// add the item in the first unused spot
		return usbd_update_string_at_index (i, str);
	}
	return 0;
}
#else
__u8 usbd_alloc_string (char *str)
{
	int i;
	struct usb_string_descriptor *string;
	__u8 bLength;
	__u16 *wData;

	RETURN_ZERO_IF(!str || !strlen (str));

        //printk(KERN_INFO"%s: %s\n", __FUNCTION__, str);

	// find an empty string descriptor slot
	for (i = 1; i < usbd_maxstrings; i++) {

		CONTINUE_IF (usb_strings[i] != NULL);

                bLength = sizeof (struct usb_string_descriptor) + 2 * strlen (str);

                RETURN_ZERO_IF(!(string = ckmalloc (bLength, GFP_KERNEL)));

                string->bLength = bLength;
                string->bDescriptorType = USB_DT_STRING;

                for (wData = string->wData; *str;) 
                        *wData++ = (__u16) (*str++);

                // store in string index array
                usb_strings[i] = string;
                return i;
        }
	return 0;
}
#endif

/* usbd_dealloc_string - deallocate a string descriptor
 *
 * Find and remove an allocated string.
 */
void usbd_dealloc_string (__u8 index)
{
	struct usb_string_descriptor *string;

	if ((index < usbd_maxstrings) && (string = usb_strings[index])) {
		usb_strings[index] = NULL;
		lkfree (string);
	}
}

/* usbd_alloc_urb - allocate an URB appropriate for specified endpoint
 *
 * Allocate an urb structure. The usb device urb structure is used to
 * contain all data associated with a transfer, including a setup packet for
 * control transfers.
 */
struct urb *usbd_alloc_urb (struct usb_function_instance *function, int endpoint_index,
                int length, int (*callback) (struct urb *, int))
{
        struct urb *urb = NULL;
        struct usb_endpoint_map *endpoint_map;
        struct usb_endpoint_instance *endpoint;
        unsigned long flags;

        //printk(KERN_INFO"%s: %s\n", __FUNCTION__, function->function_driver->name);
        RETURN_NULL_IF(!function);
        RETURN_NULL_IF(!(endpoint_map = function->endpoint_map_array));
        RETURN_NULL_IF(!(endpoint = endpoint_map[endpoint_index].endpoint));

        local_irq_save(flags);
        THROW_IF (!(urb = ckmalloc (sizeof (struct urb), GFP_ATOMIC)), error); 
        urb->endpoint = endpoint;
        urb->bus = function->bus;
        urb->function_instance = function;
        urb->link.prev = urb->link.next = &urb->link;
        urb->callback = callback;
        urb->buffer_length = urb->actual_length = 0;

	/* initialize the dma_mode into zero */
        urb->dma_mode = 0;

        if (length) {
                urb->request_length = length;

                /* For receive we always overallocate to ensure that receiving another
                 * full sized packet when we are only expecting a short packet will
                 * not overflow the buffer
                 */
                if (!endpoint->bEndpointAddress || endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) {
                        length = ((length / endpoint->wMaxPacketSize) + 1) * endpoint->wMaxPacketSize;
                }
                urb->buffer_length = length;

                if (urb->endpoint && urb->endpoint->bEndpointAddress && urb->function_instance && 
                                urb->function_instance->function_driver->fops->alloc_urb_data) 
                {
                        THROW_IF(urb->function_instance->function_driver->fops->alloc_urb_data (urb, length), error); 
                }
                else {
                        THROW_IF(!(urb->buffer = ckmalloc (length, GFP_ATOMIC)), error);
                }
        }

        CATCH(error) {
                printk(KERN_ERR"%s: dealloc %p\n", __FUNCTION__, urb); 
                usbd_dealloc_urb(urb);
                urb = NULL;
        }
        local_irq_restore(flags);
        //printk(KERN_INFO"%s: finis %p\n", __FUNCTION__, urb);
        return urb;
}

struct urb *usbd_alloc_urb_ep0 (struct usb_function_instance *function, int length, int (*callback) (struct urb *, int))
{
        return usbd_alloc_urb(function->bus->ep0, 0, length, callback);
}

/* usbd_dealloc_urb - deallocate an URB and associated buffer
 *
 * Deallocate an urb structure and associated data.
 */
void usbd_dealloc_urb (struct urb *urb)
{
        RETURN_IF (!urb);
        if (urb->buffer) 
                (urb->function_instance && urb->function_instance->function_driver->fops->dealloc_urb_data) ?
                        urb->function_instance->function_driver->fops->dealloc_urb_data : lkfree (urb->buffer);
        lkfree (urb);
}

/* usbd_flush_endpoint_address 
 */
void usbd_flush_endpoint_index (struct usb_function_instance *function, int endpoint_index)
{
        struct usb_endpoint_map *endpoint_map;
        struct usb_endpoint_instance *endpoint;

        //printk(KERN_INFO"%s: %s\n", __FUNCTION__, function->function_driver->name);
        RETURN_IF(!function);
        RETURN_IF(!(endpoint_map = function->endpoint_map_array));
        RETURN_IF(!(endpoint = endpoint_map[endpoint_index].endpoint));
        usbd_flush_endpoint (endpoint);
}

/* usbd_urb_callback - tell function that an urb has been transmitted.
 *
 * Must be called from an interrupt or with interrupts disabled.
 *
 * Used by a USB Bus driver to pass a sent urb back to the function 
 * driver via the endpoints done queue.
 */
void usbd_urb_callback (struct urb *urb, int rc)
{
        RETURN_IF (!urb);
        if (!urb->callback || urb->callback(urb,rc)) 
                usbd_dealloc_urb(urb);
}


/* usbd_recv_setup_irq - process a received urb
 *
 * Used by a USB Bus interface driver to pass received data in a URB to the
 * appropriate USB Function driver.
 */
int usbd_recv_setup_irq (struct usb_function_instance *function, struct usb_device_request *request)
{
        return function->function_driver->fops->recv_setup_irq(request);
}

void *usbd_function_get_privdata(struct usb_function_instance *function)
{
	return(function->privdata);
}

void usbd_function_set_privdata(struct usb_function_instance *function, void *privdata)
{
	function->privdata = privdata;
}

int usbd_endpoint_wMaxPacketSize(struct usb_function_instance *function, int endpoint_index, int hs)
{
        struct usb_endpoint_map *endpoint_map;
        RETURN_ZERO_IF(!(endpoint_map = function->endpoint_map_array));
        return le16_to_cpu(endpoint_map[endpoint_index].wMaxPacketSize[hs]);
}

#ifndef CONFIG_ARCH_EZX
int usbd_endpoint_wMaxPacketSize_ep0(struct usbd_function_instance *function, int hs)
{
        struct usbd_endpoint_map *endpoint_map;
        RETURN_ZERO_IF(!(endpoint_map = function->bus->ep0->endpoint_map_array));
        return le16_to_cpu(endpoint_map[0].wMaxPacketSize[hs]);
}
#endif

int usbd_endpoint_bEndpointAddress(struct usb_function_instance *function, int endpoint_index, int hs)
{
        struct usb_endpoint_map *endpoint_map;
        RETURN_ZERO_IF(!(endpoint_map = function->endpoint_map_array));
        return endpoint_map[endpoint_index].bEndpointAddress[hs];
}

int usbd_endpoint_transferSize(struct usb_function_instance *function, int endpoint_index, int hs)
{
        struct usb_endpoint_map *endpoint_map;
        RETURN_ZERO_IF(!(endpoint_map = function->endpoint_map_array));
        return endpoint_map[endpoint_index].transferSize[hs];
}

void usbd_endpoint_update(struct usb_function_instance *function, int endpoint_index, 
                struct usb_endpoint_descriptor *endpoint, int hs)
{
        endpoint->bEndpointAddress = usbd_endpoint_bEndpointAddress(function, endpoint_index, hs);
        endpoint->wMaxPacketSize = usbd_endpoint_wMaxPacketSize(function, endpoint_index, hs);
}

int usbd_endpoint_interface(struct usb_function_instance *function, int endpoint_index)
{
        struct usb_endpoint_map *endpoint_map;
        RETURN_ZERO_IF(!(endpoint_map = function->endpoint_map_array));
        return endpoint_map[endpoint_index].interface;
}

int usbd_interface_AltSetting(struct usb_function_instance *function, int interface_index)
{
        // XXX TODO - per function modifications for composite devices
        return function->bus->alternates[interface_index];
}

int usbd_ConfigurationValue(struct usb_function_instance *function)
{
        // XXX TODO - per function modifications for composite devices
        return function->bus->ConfigurationValue;
}

int usbd_high_speed(struct usb_function_instance *function)
{
        // XXX TODO - per function modifications for composite devices
        return function->bus->HighSpeedFlag;
}

/* usb-device USB FUNCTION generic functions ************************************************* */

/* alloc_function_alternates - allocate alternate instance array
 *
 * Return a pointer to an array of alternate instances. Each instance contains a pointer
 * to a filled in alternate descriptor and a pointer to the endpoint descriptor array.
 *
 * Returning NULL will cause the caller to cleanup all previously allocated memory.
 */
static struct usb_alternate_instance *alloc_function_alternates (int bInterfaceNumber,
                struct usb_interface_description *interface_description,
                int alternates, struct usb_alternate_description *alternate_description_array)
{
	int i;
	struct usb_alternate_instance *alternate_instance_array;

	// allocate array of alternate instances
	RETURN_NULL_IF(!(alternate_instance_array = ckmalloc (sizeof (struct usb_alternate_instance) * alternates, GFP_KERNEL)));
	
	// iterate across the alternate descriptions
	for (i = 0; i < alternates; i++) {

		struct usb_alternate_description *alternate_description = alternate_description_array + i;
		struct usb_alternate_instance *alternate_instance = alternate_instance_array + i;
		struct usb_interface_descriptor *interface_descriptor;

                interface_descriptor = alternate_description->interface_descriptor;
                if (interface_descriptor->bInterfaceNumber != bInterfaceNumber)	{
                        //printk(KERN_INFO"%s: bInterfaceNumber mis-match: %d should be %d\n", __FUNCTION__,
                        //                interface_descriptor->bInterfaceNumber, bInterfaceNumber);
                        lkfree(alternate_instance_array);
                        return NULL;
                }
                
		interface_descriptor->bNumEndpoints = alternate_description->endpoints;
		interface_descriptor->iInterface = usbd_alloc_string (alternate_description->iInterface );

                alternate_instance->class_list = alternate_description->class_list;
		alternate_instance->classes = alternate_description->classes;
		alternate_instance->endpoint_list = alternate_description->endpoint_list; 
		alternate_instance->endpoint_indexes = alternate_description->endpoint_indexes; 

		// save number of alternates, classes and endpoints for this alternate
		alternate_instance->endpoints = alternate_description->endpoints;
		alternate_instance->interface_descriptor = interface_descriptor;
	}
	return alternate_instance_array;
}

/* alloc_function_interfaces - allocate interface instance array
 *
 * Return a pointer to an array of interface instances. Each instance contains a pointer
 * to a filled in interface descriptor and a pointer to the endpoint descriptor array.
 *
 * Returning NULL will cause the caller to cleanup all previously allocated memory.
 */
static struct usb_interface_instance *alloc_function_interfaces (int bNumInterfaces, 
                struct usb_interface_description *interface_description_array)
{
	int interface;
	struct usb_interface_instance *interface_instance_array;

	// allocate array of interface instances
	RETURN_NULL_IF(!(interface_instance_array = 
                                ckmalloc (sizeof (struct usb_interface_instance) * bNumInterfaces, GFP_KERNEL)));
	
	// iterate across the interface descriptions
	for (interface = 0; interface < bNumInterfaces; interface++) {

		struct usb_interface_description *interface_description = interface_description_array + interface;
		struct usb_interface_instance *interface_instance = interface_instance_array + interface;

		THROW_IF( !(interface_instance->alternates_instance_array = 
                        alloc_function_alternates (interface, interface_description, 
                                        interface_description->alternates, 
					interface_description->alternate_list)), error);
		interface_instance->alternates = interface_description->alternates;
	}
	
	return interface_instance_array;

	CATCH(error) {
		for(; interface > 0; interface--) {
			lkfree(interface_instance_array[interface - 1].alternates_instance_array);	
		}
		lkfree(interface_instance_array);
		return NULL;
	}
}


/* alloc_function_configurations - allocate configuration instance array
 *
 * Return a pointer to an array of configuration instances. Each instance contains a pointer
 * to a filled in configuration descriptor and a pointer to the interface instances array.
 *
 * Returning NULL will cause the caller to cleanup all previously allocated memory.
 */
static struct usb_configuration_instance *alloc_function_configurations (int bNumConfigurations, 
                struct usb_configuration_description *configuration_description_array)
{
	int i;
	struct usb_configuration_instance *configuration_instance_array;

	// allocate array
	RETURN_NULL_IF(!(configuration_instance_array = 
                                ckmalloc (sizeof (struct usb_configuration_instance) * bNumConfigurations, GFP_KERNEL)));
	// fill in array
	for (i = 0; i < bNumConfigurations; i++) {
		int j;
		int length;

		struct usb_configuration_description *configuration_description = configuration_description_array + i;
		struct usb_configuration_descriptor *configuration_descriptor;

                configuration_descriptor = configuration_description->configuration_descriptor;

		// setup fields in configuration descriptor
		// XXX c.f. 9.4.7 zero is default, so config MUST BE 1 to n, not 0 to n-1
		// XXX N.B. the configuration itself is fetched 0 to n-1.

		configuration_descriptor->bConfigurationValue = i + 1;
		configuration_descriptor->wTotalLength = 0;
		configuration_descriptor->bNumInterfaces = configuration_description->bNumInterfaces;
		configuration_descriptor->iConfiguration = usbd_alloc_string (configuration_description->iConfiguration);

		// save the configuration descriptor in the configuration instance array
		configuration_instance_array[i].configuration_descriptor = configuration_descriptor;

		THROW_IF (!(configuration_instance_array[i].interface_instance_array = 
                                        alloc_function_interfaces (configuration_description->bNumInterfaces, 
                                                configuration_description->interface_list)), error);
		length = sizeof(struct usb_configuration_descriptor);

		for (j = 0; j < configuration_descriptor->bNumInterfaces; j++) {

			int alternate;
			struct usb_interface_instance *interface_instance = 
                                configuration_instance_array[i].interface_instance_array + j;

                        //printk(KERN_INFO"%s: len: %02x:%02d configuration\n", __FUNCTION__, length, length);
			for (alternate = 0; alternate < interface_instance->alternates; alternate++) {
				int class;
                                int endpoint;
				struct usb_alternate_instance *alternate_instance = 
                                        interface_instance->alternates_instance_array + alternate;

				length += sizeof (struct usb_interface_descriptor);

                                //printk(KERN_INFO"%s: len: %02x:%02d interface\n", __FUNCTION__, length, length);
				for (class = 0; class < alternate_instance->classes; class++) {
                                        struct usb_generic_class_descriptor * class_descriptor = 
                                                *(alternate_instance->class_list + class);
					length += class_descriptor->bLength;
                                        //printk(KERN_INFO"%s: len: %02x:%02d class\n", __FUNCTION__, length, length);
				}

				for (endpoint = 0; endpoint < alternate_instance->endpoints; endpoint++) {
                                        struct usb_endpoint_descriptor * endpoint_descriptor = 
                                                *(alternate_instance->endpoint_list + endpoint);
					length += endpoint_descriptor->bLength;
                                        //printk(KERN_INFO"%s: len: %02x:%02d endpoint\n", __FUNCTION__, length, length);
				}
			}
		}
		configuration_descriptor->wTotalLength = cpu_to_le16 (length);
		configuration_instance_array[i].bNumInterfaces = configuration_description->bNumInterfaces;
                //printk(KERN_INFO"%s: configuration_instance_array[%d]: %p\n", __FUNCTION__, i, 
                //                &configuration_instance_array[i]);
	}
	return configuration_instance_array;

	CATCH(error) {
		for (; i > 0; i--) {
			lkfree(configuration_instance_array[i-1].interface_instance_array);
		}	
		lkfree(configuration_instance_array);
		return NULL;	
	}
}


void usbd_func_event_irq(struct usb_bus_instance *bus, struct usb_function_instance *function, 
                usb_device_event_t event , int data)
{
        function->function_driver->fops->event_irq(function, event, data);
}


/* usbd_register_function - register a usb function driver
 *
 * Used by a USB Function driver to register itself with the usb device layer.
 *
 * It will create a usb_function_instance structure.
 *
 * The user friendly configuration/interface/endpoint descriptions are compiled into
 * the equivalent ready to use descriptor records.
 *
 * All function drivers must register before any bus interface drivers.
 *
 */
int usbd_register_function (struct usb_function_driver *function_driver)
{
	MOD_INC_USE_COUNT;
	list_add_tail (&function_driver->drivers, &usbd_function_drivers);
	return 0;
}


/* usbd_deregister_function - called by a USB FUNCTION driver to deregister itself
 *
 * Called by a USB Function driver De-register a usb function driver.
 */
void usbd_deregister_function (struct usb_function_driver *function_driver)
{
	list_del (&function_driver->drivers);
	MOD_DEC_USE_COUNT;
}


int usbd_function_enable (struct usb_bus_instance *bus, struct usb_function_instance *function)
{
        struct usb_function_driver *function_driver = function->function_driver;
	struct usb_device_description *device_description;
	struct usb_device_descriptor *device_descriptor;
#ifdef CONFIG_USBD_HIGH_SPEED
	/* qualifier descriptor is supported for high speed */
	struct usb_device_qualifier_descriptor *device_qualifier_descriptor;
#endif
        int rc = 0;

        function->bus = bus;

        RETURN_EINVAL_IF(function_driver->fops->function_enable (function));
	RETURN_ZERO_IF(!(device_description = function_driver->device_description));
	RETURN_ZERO_IF(!(device_descriptor = device_description->device_descriptor));

#ifdef CONFIG_USBD_HIGH_SPEED
	/* get the qualifier descriptor */
	device_qualifier_descriptor = device_description->device_qualifier_descriptor;
#endif

	device_descriptor->bMaxPacketSize0 = bus->driver->maxpacketsize;
	device_descriptor->iManufacturer = usbd_alloc_string (device_description->iManufacturer);
	device_descriptor->iProduct = usbd_alloc_string (device_description->iProduct);

#ifdef BLAN_IP_ADDR_GNPO
	// ICK: The PST team requires that the product string be at index 2,
	// due to a deficiency in all shipping version of Microsoft Windows.
	// For all other uses of the product string, the index doesn't matter,
	// so this change shouldn't affect any other functionality.
	device_descriptor->iProduct = usbd_update_string_at_index (STRINDEX_PRODUCT, device_description->iProduct);
#endif
	
	if (bus->serial_number_str && strlen (bus->serial_number_str)) 
		device_descriptor->iSerialNumber = usbd_alloc_string (bus->serial_number_str);
        else  
		device_descriptor->iSerialNumber = usbd_alloc_string (device_description->iSerialNumber);

	device_descriptor->bNumConfigurations = function_driver->bNumConfigurations;
	device_descriptor->idVendor = function_driver->idVendor;
	device_descriptor->idProduct = function_driver->idProduct;
	device_descriptor->bcdDevice = function_driver->bcdDevice;

	// allocate the configuration descriptor array
	RETURN_EINVAL_IF (!(function_driver->configuration_instance_array = 
                                alloc_function_configurations (function_driver->bNumConfigurations, 
                                        function_driver->configuration_description)));
	function_driver->device_descriptor = device_descriptor;

#ifdef CONFIG_USBD_HIGH_SPEED
	/* if there is the qualifier descriptor, update it */
	if(device_qualifier_descriptor) {
		device_qualifier_descriptor->bMaxPacketSize0 = bus->driver->maxpacketsize;
		function_driver->device_qualifier_descriptor = device_qualifier_descriptor;
	}
#endif
        return rc;
}


void usbd_function_disable (struct usb_function_instance *function)
{
	int configuration;
        struct usb_function_driver *function_driver = function->function_driver;
	struct usb_configuration_instance *configuration_instance_array = function_driver->configuration_instance_array;

        function->function_driver->fops->function_disable (function);

        RETURN_IF(!function_driver->configuration_instance_array);

	// iterate across the descriptors list and de-allocate all structures
	if (function_driver->configuration_instance_array) {
		for (configuration = 0; configuration < function_driver->bNumConfigurations; configuration++) {
			int interface;
			struct usb_configuration_instance *configuration_instance = configuration_instance_array + configuration;

			for (interface = 0; interface < configuration_instance->bNumInterfaces; interface++) {

				//int alternate;
				struct usb_interface_instance *interface_instance = 
                                        configuration_instance->interface_instance_array + interface;

				lkfree (interface_instance->alternates_instance_array);
			}
			lkfree (configuration_instance->interface_instance_array);
			//usbd_dealloc_descriptor_strings( (struct usb_descriptor *)
                        //                configuration_instance_array[configuration].configuration_descriptor);
		}
	}
	
        //usbd_dealloc_descriptor_strings ((struct usb_descriptor *) function_driver->device_descriptor);
	lkfree (configuration_instance_array);

        function->bus = NULL;
}

EXPORT_SYMBOL(usbd_register_function );
EXPORT_SYMBOL(usbd_deregister_function);
EXPORT_SYMBOL(usbd_alloc_string);
#ifdef BLAN_IP_ADDR_GNPO
EXPORT_SYMBOL(usbd_update_string_at_index);
#endif
EXPORT_SYMBOL(usbd_dealloc_string);
EXPORT_SYMBOL(usbd_get_string);
EXPORT_SYMBOL(usbd_alloc_urb);
EXPORT_SYMBOL(usbd_alloc_urb_ep0);
EXPORT_SYMBOL(usbd_dealloc_urb);
EXPORT_SYMBOL(usbd_flush_endpoint_index);
EXPORT_SYMBOL(usbd_recv_setup_irq);
EXPORT_SYMBOL(usbd_urb_callback);
EXPORT_SYMBOL(usbd_endpoint_halted);
EXPORT_SYMBOL(usbd_device_feature);
EXPORT_SYMBOL(usbd_function_get_privdata);
EXPORT_SYMBOL(usbd_function_set_privdata);

EXPORT_SYMBOL(usbd_endpoint_wMaxPacketSize);
EXPORT_SYMBOL(usbd_endpoint_bEndpointAddress);
EXPORT_SYMBOL(usbd_endpoint_interface);
EXPORT_SYMBOL(usbd_endpoint_transferSize);
EXPORT_SYMBOL(usbd_interface_AltSetting);
EXPORT_SYMBOL(usbd_ConfigurationValue);
EXPORT_SYMBOL(usbd_high_speed);
EXPORT_SYMBOL(usbd_endpoint_update);
