/*
 * usbd/ep0.c
 *
 *      Copyright (c) 2004 Belcarra
 *
 * Adapted from earlier work:
 *      Copyright (c) 2002, 2003, 2004 Belcarra
 *      Copyright (c) 2000, 2001, 2002 Lineo
 *      Copyright (c) 2001 Hewlett Packard
 *
 * By: 
 *      Stuart Lynne <sl@belcarra.com>, 
 *      Tom Rushworth <tbr@belcarra.com>, 
 *      Bruce Balden <balden@lineo.com>
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
 * This function driver implements support for all of the USB 2.0 Chapter
 * nine requests.
 *
 * Any request that is not required by Chapter nine is passed to the other
 * function drivers recv_setup routine.
 *
 */

/*
 * Copyright (C) 2004-2005 - Motorola
 *
 * 2004-Dec-06 - Change for EZXBASE By Zhao Liang <w20146@motorola.com>
 *
 * 2005-JUN-01 - Passed USB-IF Test By Li Xin <a16157@motorola.com>
 */

#include <linux/config.h>
#include <linux/module.h>

//EXPORT_NO_SYMBOLS;

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
#include "usbd-bus.h"           // for usbd_recv_setup_irq() definition
#include "usbd-admin.h"


struct usb_function_instance * ep0_function;
#ifdef CONFIG_USBD_HIGH_SPEED
#define  USB_HS_TEST_MODE   0x02
extern int udc_hs_test_mode(int mode_type);
#endif
/* C.f. 9.4 Standard Requests and table 9.3
 *
 * Encode valid requests into a bitmap for each recipient type for 
 * both directions.
 *
 */

#define STD(x) (1<<(x+1))

/* h2d_standard_requests and d2h_standard_requests
 *
 * These tables list all of the valid Chapter Nine requests. Any request
 * not listed in these tables will NOT be processed by the EP0 function and 
 * will instead be passed to the appropriate function driver.
 */
u32 h2d_standard_requests[4] = {
        // 0 - Device
        STD(USB_REQ_CLEAR_FEATURE) |
        STD(USB_REQ_SET_FEATURE) |
        STD(USB_REQ_SET_ADDRESS) |
        STD(USB_REQ_SET_DESCRIPTOR) |
#ifndef CONFIG_ARCH_EZX
        STD(USB_REQ_GET_CONFIGURATION) |
#endif
        STD(USB_REQ_SET_CONFIGURATION) ,
        // 1 - Interface
        STD(USB_REQ_CLEAR_FEATURE) |
        STD(USB_REQ_SET_FEATURE) |
        STD(USB_REQ_SET_INTERFACE) ,
        // 2 - Endpoint
        STD(USB_REQ_CLEAR_FEATURE) |
        STD(USB_REQ_SET_FEATURE) ,
        // 3 - Other
        0,
};

u32 d2h_standard_requests[4] = {
        // 0 - Device
        STD(USB_REQ_GET_STATUS) |
#ifdef CONFIG_ARCH_EZX
        STD(USB_REQ_GET_CONFIGURATION) |
#endif
        STD(USB_REQ_GET_DESCRIPTOR) ,
        // 1 - Interface
        STD(USB_REQ_GET_STATUS) |
        STD(USB_REQ_GET_INTERFACE) ,
        // 2 - Endpoint
        STD(USB_REQ_GET_STATUS) |
        STD(USB_REQ_SYNCH_FRAME) ,
        // 3 - Other
        0,
};


/* Endpoint ZEro Configuration *************************************************************** */

/* ep0_event_irq - respond to USB event
 *
 * Process USB events.
 */
static void ep0_event_irq (struct usb_function_instance *function, usb_device_event_t event, int dummy )
{
	switch (event) {
	case DEVICE_CREATE:
        default:
                break;
        }
}

/* copy_config - copy data into urb buffer
 */
static int copy_config (u8 *cp, void *data, int actual_length, int max_buf)
{
	int available = max_buf - actual_length;
        int length = MIN(*(u8 *)data, available);

	RETURN_ZERO_IF (!length);
	memcpy (cp, data, length);
        return length;
}

/* copy_config - copy data into urb buffer
 */
static int copy_endpoint (struct usb_function_instance *function, u8 *cp,
                struct usb_endpoint_descriptor *endpoint, int endpoint_index, int actual_length, int max_buf, int hs)
{
	int available = max_buf - actual_length;
	int length = MIN(endpoint->bLength, available);
        struct usb_endpoint_descriptor endpoint_copy;

        RETURN_ZERO_IF (!length);
	memcpy (&endpoint_copy, endpoint, endpoint->bLength);
        usbd_endpoint_update(function, endpoint_index, &endpoint_copy, hs);
	memcpy (cp, &endpoint_copy, length);
        return length;
}

/* usbd_get_descriptor - copy descriptor into urb buffer
 *
 * Return non-zero for error.
 */

#ifdef CONFIG_ARCH_EZX
#include "../misc/ssp_pcap.h"
int usbd_enum_flag;
extern int usbcable_status;
#endif

int usbd_get_descriptor (struct usb_bus_instance *bus, u8 *buffer, int max, int descriptor_type, int index)
{
        struct usb_function_driver *function_driver = bus->function_instance->function_driver;
        int actual_length = 0;

	switch (descriptor_type) {
	case USB_DESCRIPTOR_TYPE_DEVICE:
		{
			struct usb_device_descriptor *device_descriptor = function_driver->device_descriptor;

			// copy descriptor for this device
			actual_length += copy_config (buffer + actual_length, device_descriptor, actual_length, max);

			// correct the correct control endpoint 0 max packet size into the descriptor
			device_descriptor = (struct usb_device_descriptor *) buffer;
			device_descriptor->bMaxPacketSize0 = bus->driver->maxpacketsize;
		}
		break;

#ifdef CONFIG_USBD_HIGH_SPEED
	case USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER:      // c.f. 9.6.2 Device Qualifier
                {
			struct usb_device_qualifier_descriptor *device_qualifier_descriptor = 
                                function_driver->device_qualifier_descriptor;
			if(device_qualifier_descriptor != NULL)	{
			// copy descriptor for this device
			actual_length += copy_config (buffer + actual_length, device_qualifier_descriptor, actual_length, max);
			} else {
				return -EINVAL;
			}
		}
                break;

	case USB_DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION:
	case USB_DESCRIPTOR_TYPE_CONFIGURATION:
		{
                        int hs = bus->HighSpeedFlag ?  descriptor_type == USB_DESCRIPTOR_TYPE_CONFIGURATION:
                                descriptor_type == USB_DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION;
#else
	case USB_DESCRIPTOR_TYPE_CONFIGURATION:
		{
                        int hs = 0;
#endif
			int interface;

                        struct usb_configuration_instance *configuration_instance = 
                                &function_driver->configuration_instance_array[index];

			struct usb_configuration_descriptor *configuration_descriptor = 
                                configuration_instance->configuration_descriptor;

                        RETURN_EINVAL_IF (!configuration_descriptor);
			RETURN_EINVAL_IF (index > function_driver->device_descriptor->bNumConfigurations);

			actual_length += copy_config (buffer + actual_length, configuration_descriptor, actual_length, max);

			// iterate across bNumInterfaces for specified configuration
			for (interface = 0; interface < configuration_descriptor->bNumInterfaces; interface++) {

				int alternate;
				struct usb_interface_instance *interface_instance =
                                        configuration_instance->interface_instance_array + interface;

				// iterate across interface alternates
				for (alternate = 0; alternate < interface_instance->alternates; alternate++) {

					int class;
					int endpoint;

					struct usb_alternate_instance *alternate_instance =
                                                interface_instance->alternates_instance_array + alternate;

                                        //struct usb_interface_descriptor *usb_interface_descriptor;

					// copy descriptor for this interface
					actual_length += copy_config (buffer + actual_length, 
                                                        alternate_instance->interface_descriptor, actual_length, max);

					// iterate across classes for this alternate interface
					for (class = 0; class < alternate_instance->classes; class++) 
						actual_length += copy_config (buffer + actual_length, 
                                                                *(alternate_instance->class_list + class), actual_length, max);

					// iterate across endpoints for this alternate interface
					//interface_descriptor = alternate_instance->interface_descriptor;

                                        for (endpoint = 0; endpoint < alternate_instance->endpoints ; endpoint++) {

                                                //printk(KERN_INFO"%s: endpoint: %d index: %d\n",
                                                //                __FUNCTION__, endpoint, 
                                                //                alternate_instance->endpoint_indexes[endpoint]
                                                //                );
                                                actual_length += copy_endpoint (bus->function_instance, 
                                                                buffer + actual_length, 
                                                                *(( alternate_instance->endpoint_list) + endpoint), 
                                                                alternate_instance->endpoint_indexes[endpoint],
                                                                actual_length, max, hs);
                                        }
                                }
                        }
                }
                break;

	case USB_DESCRIPTOR_TYPE_STRING:
		{
			struct usb_string_descriptor *string_descriptor;
			RETURN_EINVAL_IF (!(string_descriptor = usbd_get_string (index)));
			actual_length += copy_config (buffer + actual_length, string_descriptor, actual_length, max);
		}
		break;

	default:
		return -EINVAL;
	}
	return actual_length;
}

/* ep0_recv_setup_irq - process a device request
 *
 * Process a received device request. If not a Chapter nine request pass it
 * to the other loaded function driver recv_setup_irq() function.
 *
 * Return non-zero to indicate failure.
 */
static int ep0_recv_setup_irq (struct usb_device_request *request)
{
        struct usb_function_instance *function = ep0_function;
        struct usb_bus_instance *bus = ep0_function->bus;

#if 0
	printk(KERN_INFO"%s: bus: %p bmRequestType:%02x bRequest:%02x wValue:%04x wIndex:%04x wLength:%04x %d\n", __FUNCTION__,
	        function->bus, request->bmRequestType, request->bRequest, 
	        le16_to_cpu(request->wValue), le16_to_cpu(request->wIndex), le16_to_cpu(request->wLength),
                request->bRequest);
#endif

	// handle USB Standard Request only (c.f. USB Spec table 9-2, D6..5 must be 0)

        THROW_IF ((request->bmRequestType & USB_REQ_TYPE_MASK) != 0, non_standard);

        THROW_IF(!( (request->bmRequestType & USB_DIR_IN ?  d2h_standard_requests : h2d_standard_requests) 
                        [request->bmRequestType & 0x3] & STD(request->bRequest)), non_standard ); 
        
        CATCH(non_standard) {
                return usbd_recv_setup_irq(bus->function_instance, request);
        }

        switch (bus->device_state) {
        case STATE_CREATED:
        case STATE_ATTACHED:
        case STATE_POWERED:
                return -EINVAL;

        case STATE_INIT:
        case STATE_DEFAULT:
                switch (request->bRequest) {
                case USB_REQ_GET_STATUS:
                case USB_REQ_GET_INTERFACE:
		case USB_REQ_SYNCH_FRAME:
		case USB_REQ_CLEAR_FEATURE:
		case USB_REQ_SET_FEATURE:
		case USB_REQ_SET_DESCRIPTOR:
		case USB_REQ_SET_INTERFACE:
                        printk(KERN_INFO"%s: bad device_state\n", __FUNCTION__);
			return -EINVAL;

		case USB_REQ_SET_CONFIGURATION:
		case USB_REQ_SET_ADDRESS:
		case USB_REQ_GET_DESCRIPTOR:
		case USB_REQ_GET_CONFIGURATION:
			break;
		}
	case STATE_ADDRESSED:
	case STATE_CONFIGURED:
	case STATE_SUSPENDED:
		break;
	case STATE_UNKNOWN:
                        printk(KERN_INFO"%s: suspended or unknown\n", __FUNCTION__);
		return -EINVAL;
	}

	// handle all requests that return data (direction bit set on bm RequestType)
	if ((request->bmRequestType & USB_REQ_DIRECTION_MASK)) {

                struct urb *urb;
                int rc = 0;
		switch (request->bRequest) {
		case USB_REQ_SYNCH_FRAME:
		case USB_REQ_CLEAR_FEATURE:
		case USB_REQ_SET_FEATURE:
		case USB_REQ_SET_ADDRESS:
		case USB_REQ_SET_DESCRIPTOR:
		case USB_REQ_SET_CONFIGURATION:
		case USB_REQ_SET_INTERFACE:
                        printk(KERN_INFO"%s: bad direction\n", __FUNCTION__);
			return -EINVAL;
		}

                RETURN_EINVAL_IF(!le16_to_cpu(request->wLength));

                // allocate urb, no callback, urb will be automatically de-allocated
                RETURN_EINVAL_IF(!(urb = usbd_alloc_urb (function, 0, le16_to_cpu(request->wLength), NULL)));

		switch (request->bRequest) {

		case USB_REQ_GET_STATUS:
                        urb->actual_length = 2;
                        urb->buffer[0] = urb->buffer[1] = 0;
                        switch (request->bmRequestType & USB_REQ_RECIPIENT_MASK) {
                        case USB_REQ_RECIPIENT_DEVICE:
#ifdef CONFIG_ARCH_EZX
                                urb->buffer[0] = 0;
#else
                                urb->buffer[0] = USB_STATUS_SELFPOWERED;
#endif
                                break;
                        case USB_REQ_RECIPIENT_INTERFACE:
                                break;
                        case USB_REQ_RECIPIENT_ENDPOINT:
                                urb->buffer[0] = usbd_endpoint_halted (function, le16_to_cpu(request->wIndex));
                                break;
                        case USB_REQ_RECIPIENT_OTHER:
                                urb->actual_length = 0;
                        default:
                                break;
                        }
                        rc = 0;

                        break;

                case USB_REQ_GET_DESCRIPTOR:
#ifdef CONFIG_ARCH_EZX
			if((le16_to_cpu (request->wValue >> 8)) == USB_DESCRIPTOR_TYPE_DEVICE && 
					(bus->device_state != DEVICE_CONFIGURED))
			{
				// some function drivers(PST) will still send GET DEVICE DESC command after SET CFG.
				//printk(KERN_INFO"%s: jiffies %d usbd_enum_flag %d state %d\n", __FUNCTION__, (int)jiffies, 
				//		usbd_enum_flag, bus->device_state);	// usbd_bus->device_state???
				usbd_enum_flag = 1;
				if(usbcable_status == 0)
				{
					usbcable_status = 1;
				}
			}
#endif
                        rc = usbd_get_descriptor (bus, urb->buffer, 
                                        le16_to_cpu (request->wLength), 
                                        le16_to_cpu (request->wValue >> 8), 
                                        le16_to_cpu (request->wValue) & 0xff);
                        if (rc != -EINVAL) {
                                urb->actual_length = rc;
                                rc = 0;
                        }
                        break;

                case USB_REQ_GET_CONFIGURATION:
                        urb->actual_length = 1;
#ifdef CONFIG_ARCH_EZX
		 	urb->buffer[0] = bus->Raw_ConfigurationValue; 
#else
		 	urb->buffer[0] = bus->ConfigurationValue; 
#endif
                        break;

                case USB_REQ_GET_INTERFACE:
                        RETURN_EINVAL_IF(le16_to_cpu(request->wIndex) > bus->bNumInterfaces);
                        urb->actual_length = 1;	
                        urb->buffer[0] = bus->alternates[le16_to_cpu(request->wIndex)];
                        break;
                default:
                        rc = 1;
                }

                //printk(KERN_INFO"%s: actual: %d packetsize: %d wIndex: %d rc: %d\n", __FUNCTION__,
                //                urb->actual_length, urb->bus->driver->maxpacketsize, le16_to_cpu(request->wLength), rc);

                if (!(urb->actual_length % urb->bus->driver->maxpacketsize) && 
                                (urb->actual_length < le16_to_cpu(request->wLength))) 
                {
                        //printk(KERN_INFO"%s: NEED ZLP\n", __FUNCTION__);
                        urb->flags |= USBD_URB_SENDZLP;
                }
                if (!rc)
                        RETURN_ZERO_IF(!usbd_send_urb(urb));
                //printk(KERN_INFO"%s: failed urb: %p\n", __FUNCTION__, urb);
                usbd_dealloc_urb(urb);
                return -EINVAL;
        }
        // handle the requests that do not return data
        else {

                switch (request->bRequest) {

                case USB_REQ_CLEAR_FEATURE:
                case USB_REQ_SET_FEATURE:
                        switch (request->bmRequestType & USB_REQ_RECIPIENT_MASK) {
                        case USB_REQ_RECIPIENT_DEVICE:
                                // XXX DEVICE_REMOTE_WAKEUP or TEST_MODE would be added here
                                // XXX fall through for now as we do not support either
#ifdef CONFIG_USBD_HIGH_SPEED
				if(request->wValue == USB_HS_TEST_MODE)    {
					return udc_hs_test_mode((request->wIndex & 0xFF00) >> 8);
				} else 	{
					return -EINVAL;
				}
#endif
                        case USB_REQ_RECIPIENT_INTERFACE:
                        case USB_REQ_RECIPIENT_OTHER:
                        default:
                                return -EINVAL;

                        case USB_REQ_RECIPIENT_ENDPOINT:
                                if (le16_to_cpu(request->wValue) == USB_ENDPOINT_HALT) 
                                        return usbd_device_feature (function, le16_to_cpu (request->wIndex) & 0x7f,
                                                        request->bRequest == USB_REQ_SET_FEATURE);

                                else 
                                        return -EINVAL ;
                        }

                case USB_REQ_SET_ADDRESS:
                        // check if this is a re-address, reset first if it is (this shouldn't be possible)
                        RETURN_EINVAL_IF (bus->device_state != STATE_DEFAULT);
                        usbd_bus_event (bus, DEVICE_ADDRESS_ASSIGNED, le16_to_cpu(request->wValue));
                        return 0;

                case USB_REQ_SET_DESCRIPTOR:	
                        // XXX should we support this?
                        // This would require allocating a rcv urb and using usbd_start_recv()
                        return -EINVAL;

                case USB_REQ_SET_CONFIGURATION:
                        {
                                struct usb_function_driver *function_driver = bus->function_instance->function_driver;
                                int bNumConfigurations = function_driver->bNumConfigurations;
                                int bNumInterfaces;
                                u8 ConfigurationValue;


                                struct usb_configuration_instance *configuration_instance;
                                struct usb_configuration_descriptor *configuration_descriptor;

                                // get rid of previous interface and alternates 
                                if (bus->bNumInterfaces && bus->alternates) {
                                        bus->bNumInterfaces = 0;
                                        lkfree(bus->alternates);
                                        bus->alternates = NULL;
                                }

                                // c.f. 9.4.7 - the top half of wValue is reserved
                                //
                                // c.f. 9.4.7 - zero is the default or addressed state, in our case this
                                // is the same is configuration zero, but will be fixed in usbd.c when used.
                                
                                ConfigurationValue = le16_to_cpu (request->wValue) & 0x7f;

                                RETURN_EINVAL_IF(ConfigurationValue > bNumConfigurations);
                                 
#ifdef CONFIG_ARCH_EZX
                                bus->Raw_ConfigurationValue = ConfigurationValue;
#endif

#ifdef CONFIG_ARCH_EZX		// w20146 - bug fix - Apr 30, 2004
				ConfigurationValue = ConfigurationValue ? (ConfigurationValue - 1) : 0;
#else
				ConfigurationValue = ConfigurationValue ? 0 : ConfigurationValue -1;
#endif
                                
                                configuration_instance = &function_driver->configuration_instance_array[ConfigurationValue];
                                configuration_descriptor = configuration_instance->configuration_descriptor;

                                RETURN_EINVAL_IF(!configuration_descriptor);

                                bNumInterfaces = configuration_instance->bNumInterfaces;

                                bus->ConfigurationValue = ConfigurationValue + 1;

                                // reset interface and alternate settings

                                RETURN_EINVAL_IF (!(bus->alternates = ckmalloc(bNumInterfaces, GFP_ATOMIC)));
                                bus->bNumInterfaces = bNumInterfaces;
				
#ifdef CONFIG_ARCH_EZX
				usbd_enum_flag = 0;	// enumeration end
#endif
                                //usbd_bus->device_event (bus, DEVICE_CONFIGURED, 0);
                                usbd_bus_event (bus, DEVICE_CONFIGURED, 0);
                                return 0;
                        }

		case USB_REQ_SET_INTERFACE:
                        {
                                int interface = le16_to_cpu(request->wIndex);

                                RETURN_EINVAL_IF(interface > bus->bNumInterfaces);

                                bus->alternates[interface] = le16_to_cpu(request->wValue);
                                usbd_bus_event (bus, DEVICE_SET_INTERFACE, 0);
                                return 0;
                        }

                case USB_REQ_GET_STATUS:
		case USB_REQ_GET_DESCRIPTOR:
		case USB_REQ_GET_CONFIGURATION:
		case USB_REQ_GET_INTERFACE:
		case USB_REQ_SYNCH_FRAME:	// XXX should never see this (?)
                        printk(KERN_INFO"%s: unknown\n", __FUNCTION__);
			return -EINVAL;
		}
	}
	return -EINVAL;
}


/* ep0_function_enable - enable the endpoint zero function
 *
 * Return non-zero on failure.
 */
static int ep0_function_enable (struct usb_function_instance *function)
{
        ep0_function = function;
        return 0;
}

/* ep0_function_disable - disable the endpoint zero function
 */
static void ep0_function_disable (struct usb_function_instance *function)
{
        ep0_function = NULL;
}       


static struct usb_function_operations ep0_ops = {
	event_irq:	ep0_event_irq,
	recv_setup_irq:	ep0_recv_setup_irq,
        function_enable: ep0_function_enable,
        function_disable: ep0_function_disable,
};

struct usb_function_driver ep0_driver = {
	name:		"EP0",
	fops:		&ep0_ops,
};

EXPORT_SYMBOL(usbd_get_descriptor);
