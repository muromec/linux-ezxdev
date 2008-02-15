/*
 * linux/drivers/usbd/ep0.c
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 * Copyright (c) 2001 Hewlett Packard
 *
 * By: 
 *      Stuart Lynne <sl@lineo.com>, 
 *      Tom Rushworth <tbr@lineo.com>, 
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
 * This is the builtin ep0 control function. It implements all required functionality
 * for responding to control requests (SETUP packets).
 *
 * XXX 
 *
 * Currently we do not pass any SETUP packets (or other) to the configured
 * function driver. This may need to change.
 *
 * XXX
 */

#include <linux/config.h>
#include <linux/module.h>

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <asm/uaccess.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/etherdevice.h>
#include <net/arp.h>
#include <linux/rtnetlink.h>
#include <linux/smp_lock.h>
#include <linux/ctype.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/atmdev.h>
#include <linux/pkt_sched.h>

#include "usbd.h"
#include "usbd-debug.h"
#include "usbd-func.h"
#include "usbd-bus.h"
#include "usbd-inline.h"

extern int dbgflg_usbdcore_ep0;
#define dbg_ep0(lvl,fmt,args...) dbgPRINT(dbgflg_usbdcore_ep0,lvl,fmt,##args)

/* EP0 Configuration Set ********************************************************************* */

#if 0

// XXX ep0 does not have or need any configuration

static struct usb_endpoint_description ep0_endpoints[] = {
      {bEndpointAddress: 0x00, attributes: 0, max_size: 8, polling_interval:0}
};


static struct usb_interface_description ep0_interfaces[] = {
      {class: 0, sub_class: 0, protocol:0,
	      endpoints:sizeof (ep0_endpoints) / sizeof (struct usb_endpoint_description),
      endpoint_list:ep0_endpoints},
};

struct usb_configuration_description ep0_description[] = {
      {name: "EP0", attributes: 0, max_power:0,
	      interfaces:sizeof (ep0_interfaces) /
	 sizeof (struct usb_interface_description),
      interface_list:ep0_interfaces},
};

#endif

/**
 * ep0_event - respond to USB event
 * @device:
 * @event:
 *
 * Called by lower layers to indicate some event has taken place. Typically
 * this is reset, suspend, resume and close.
 */
static void ep0_event (struct usb_device_instance *device,
		       usb_device_event_t event, int dummy /* to make fn signature correct */ )
{
        // do nothing
}

/**
 * ep0_get_status - fill in URB data with appropriate status
 * @device:
 * @urb:
 * @index:
 * @requesttype:
 *
 */
static int ep0_get_status (struct usb_device_instance *device, struct urb *urb, int index, int requesttype)
{
	char *cp;

	urb->actual_length = 2;
	cp = urb->buffer;
        cp[0] = cp[1] = 0;

	switch (requesttype) {
	case USB_REQ_RECIPIENT_DEVICE:
		cp[0] = USB_STATUS_SELFPOWERED;
		break;
	case USB_REQ_RECIPIENT_INTERFACE:
		break;
	case USB_REQ_RECIPIENT_ENDPOINT:
		cp[0] = usbd_endpoint_halted (device, index);
		break;
	case USB_REQ_RECIPIENT_OTHER:
		urb->actual_length = 0;
        default:
                break;
	}
	dbg_ep0(2, "%02x %02x", cp[0], cp[1]);
	return 0;
}

/**
 * ep0_get_one
 * @device:
 * @urb:
 * @result:
 *
 * Set a single byte value in the urb send buffer. Return non-zero to signal
 * a request error.
 */
static int ep0_get_one (struct usb_device_instance *device, struct urb *urb, __u8 result)
{
	urb->actual_length = 1;	// XXX 2?
	((char *) urb->buffer)[0] = result;
	return 0;
}

/**
 * copy_config
 * @urb: pointer to urb
 * @data: pointer to configuration data
 * @length: length of data
 *
 * Copy configuration data to urb transfer buffer if there is room for it.
 */
static void copy_config (struct urb *urb, void *data, int max_length, int max_buf)
{
	int available;
	int length;

	//dbg_ep0(3, "-> actual: %d buf: %d max_buf: %d max_length: %d data: %p", 
	//        urb->actual_length, urb->buffer_length, max_buf, max_length, data);

	if (!data) {
		dbg_ep0 (1, "data is NULL");
		return;
	}
	if (!(length = *(unsigned char *) data)) {
		dbg_ep0 (1, "length is zero");
		return;
	}

	if (length > max_length) {
		dbg_ep0 (1, "length: %d >= max_length: %d", length, max_length);
		return;
	}
	//dbg_ep0(1, "   actual: %d buf: %d max_buf: %d max_length: %d length: %d", 
	//        urb->actual_length, urb->buffer_length, max_buf, max_length, length);

	if ((available = /*urb->buffer_length */ max_buf - urb->actual_length) <= 0) {
		return;
	}
	//dbg_ep0(1, "actual: %d buf: %d max_buf: %d length: %d available: %d", 
	//        urb->actual_length, urb->buffer_length, max_buf, length, available);

	if (length > available) {
		length = available;
	}
	//dbg_ep0(1, "actual: %d buf: %d max_buf: %d length: %d available: %d", 
	//        urb->actual_length, urb->buffer_length, max_buf, length, available);

	memcpy (urb->buffer + urb->actual_length, data, length);
	urb->actual_length += length;

	//dbg_ep0(3, "<- actual: %d buf: %d max_buf: %d max_length: %d available: %d", 
	//        urb->actual_length, urb->buffer_length, max_buf, max_length, available);
}

/**
 * ep0_get_descriptor
 * @device:
 * @urb:
 * @max:
 * @descriptor_type:
 * @index:
 *
 * Called by ep0_rx_process for a get descriptor device command. Determine what
 * descriptor is being requested, copy to send buffer. Return zero if ok to send,
 * return non-zero to signal a request error.
 */
static int ep0_get_descriptor (struct usb_device_instance *device, struct urb *urb, int max,
			       int descriptor_type, int index)
{
	int port = 0;		// XXX compound device
	char *cp;

	//dbg_ep0(3, "max: %x type: %x index: %x", max, descriptor_type, index);

	if (!urb || !urb->buffer || !urb->buffer_length || (urb->buffer_length < 255)) {
		dbg_ep0 (2, "invalid urb %p", urb);
		return -EINVAL;
	}

	// setup tx urb
	urb->actual_length = 0;
	cp = urb->buffer;

        dbg_ep0(2, "%s", USBD_DEVICE_DESCRIPTORS(descriptor_type));

	switch (descriptor_type) {
	case USB_DESCRIPTOR_TYPE_DEVICE:
		{
			struct usb_device_descriptor *device_descriptor;

			if (!(device_descriptor = usbd_device_device_descriptor (device, port))) {
				return -EINVAL;
			}
			// copy descriptor for this device
			copy_config (urb, device_descriptor, sizeof (struct usb_device_descriptor),
				     max);

			// correct the correct control endpoint 0 max packet size into the descriptor
			device_descriptor = (struct usb_device_descriptor *) urb->buffer;
			device_descriptor->bMaxPacketSize0 =
			    urb->device->bus->driver->maxpacketsize;

		}
		//dbg_ep0(3, "copied device configuration, actual_length: %x", urb->actual_length);
		break;

	case USB_DESCRIPTOR_TYPE_CONFIGURATION:
		{
			int bNumInterface;
			struct usb_configuration_descriptor *configuration_descriptor;
			struct usb_device_descriptor *device_descriptor;
			if (!(device_descriptor = usbd_device_device_descriptor (device, port))) {
				return -EINVAL;
			}
			//dbg_ep0(2, "%d %d", index, device_descriptor->bNumConfigurations);
			if (index > device_descriptor->bNumConfigurations) {
				dbg_ep0 (0, "index too large: %d > %d", index, device_descriptor->bNumConfigurations);
				return -EINVAL;
			}

			if (!
			    (configuration_descriptor =
			     usbd_device_configuration_descriptor (device, port, index))) {
				dbg_ep0 (0, "usbd_device_configuration_descriptor failed: %d", index);
				return -EINVAL;
			}
			copy_config (urb, configuration_descriptor,
				     sizeof (struct usb_configuration_descriptor), max);


			// iterate across interfaces for specified configuration
			for (bNumInterface = 0;
			     bNumInterface < configuration_descriptor->bNumInterfaces;
			     bNumInterface++) {

				int bAlternateSetting;
				struct usb_interface_instance *interface_instance;

				//dbg_ep0(3, "[%d] bNumInterfaces: %d", bNumInterface, configuration_descriptor->bNumInterfaces);

				if (!
				    (interface_instance =
				     usbd_device_interface_instance (device, port, index,
								     bNumInterface))) {
					dbg_ep0 (3, "[%d] interface_instance NULL", bNumInterface);
					return -EINVAL;
				}
				// iterate across interface alternates
				for (bAlternateSetting = 0;
				     bAlternateSetting < interface_instance->alternates;
				     bAlternateSetting++) {
					int class;
					int bNumEndpoint;
					struct usb_interface_descriptor *interface_descriptor;

					struct usb_alternate_instance *alternate_instance;

					//dbg_ep0(3, "[%d:%d] alternates: %d", bNumInterface, bAlternateSetting, interface_instance->alternates);

					if (!
					    (alternate_instance =
					     usbd_device_alternate_instance (device, port, index, bNumInterface,
									     bAlternateSetting)))
                                        {
						dbg_ep0 (3, "[%d] alternate_instance NULL",
							 bNumInterface);
						return -EINVAL;
					}
					// copy descriptor for this interface
					copy_config (urb, alternate_instance->interface_descriptor,
						     sizeof (struct usb_interface_descriptor), max);

					//dbg_ep0(3, "[%d:%d] classes: %d endpoints: %d", bNumInterface, bAlternateSetting, 
					//        alternate_instance->classes, alternate_instance->endpoints);

					// iterate across classes for this alternate interface
					for (class = 0; class < alternate_instance->classes;
					     class++) {
						struct usb_class_descriptor *class_descriptor;
						//dbg_ep0(3, "[%d:%d:%d] classes: %d", bNumInterface, bAlternateSetting, 
						//        class, alternate_instance->classes);
						if (!(class_descriptor =
						      usbd_device_class_descriptor_index (device,
											  port,
											  index,
											  bNumInterface,
											  bAlternateSetting,
											  class))) {
							dbg_ep0 (3, "[%d] class NULL", class);
							return -EINVAL;
						}
						// copy descriptor for this class
						copy_config (urb, class_descriptor,
							     sizeof (struct usb_class_descriptor),
							     max);
					}

					// iterate across endpoints for this alternate interface
					interface_descriptor =
					    alternate_instance->interface_descriptor;
					for (bNumEndpoint = 0;
					     bNumEndpoint < alternate_instance->endpoints;
					     bNumEndpoint++) {
						struct usb_endpoint_descriptor *endpoint_descriptor;
						//dbg_ep0(3, "[%d:%d:%d] endpoint: %d", bNumInterface, bAlternateSetting, 
						//        bNumEndpoint, interface_descriptor->bNumEndpoints);
						if (!(endpoint_descriptor =
						      usbd_device_endpoint_descriptor_index (device,
											     port,
											     index,
											     bNumInterface,
											     bAlternateSetting,
											     bNumEndpoint)))
						{
							dbg_ep0 (3, "[%d] endpoint NULL",
								 bNumEndpoint);
							return -EINVAL;
						}
						// copy descriptor for this endpoint
						copy_config (urb, endpoint_descriptor,
							     sizeof (struct
								     usb_endpoint_descriptor), max);
					}
				}
			}
			//dbg_ep0(3, "lengths: %d %d", le16_to_cpu(configuration_descriptor->wTotalLength), urb->actual_length);
		}
		break;

	case USB_DESCRIPTOR_TYPE_STRING:
		{
			struct usb_string_descriptor *string_descriptor;
			if (!(string_descriptor = usbd_get_string (index))) {
				return -EINVAL;
			}
			//dbg_ep0(3, "string_descriptor: %p", string_descriptor);
			copy_config (urb, string_descriptor, string_descriptor->bLength, max);
		}
		break;
	case USB_DESCRIPTOR_TYPE_INTERFACE:
		return -EINVAL;
	case USB_DESCRIPTOR_TYPE_ENDPOINT:
		return -EINVAL;
	case USB_DESCRIPTOR_TYPE_HID:
        {
		int bNumInterface = le16_to_cpu(urb->device_request.wIndex);
		int bAlternateSetting = 0;
		int class = 0;
		struct usb_class_descriptor *class_descriptor;
		if (!(class_descriptor = 
		      usbd_device_class_descriptor_index(
			      device, port, 0, bNumInterface,
			      bAlternateSetting, class)) ||
		    class_descriptor->descriptor.hid.bDescriptorType !=
		    USB_DT_HID) 
		{
			dbg_ep0(3, "[%d] interface is not HID", bNumInterface);
			return -EINVAL;
		}
		// copy descriptor for this class
		copy_config(urb, class_descriptor,
			    class_descriptor->descriptor.hid.bLength, max);
        }
        break;
	case USB_DESCRIPTOR_TYPE_REPORT:
	{
		int bNumInterface = le16_to_cpu(urb->device_request.wIndex);
		int bAlternateSetting = 0;
		int class = 0;
		struct usb_class_report_descriptor *report_descriptor;
		if (!(report_descriptor = 
		      usbd_device_class_report_descriptor_index(
			      device, port, 0, bNumInterface,
			      bAlternateSetting, class)) ||
		    report_descriptor->bDescriptorType != USB_DT_REPORT) 
		{
			dbg_ep0(3, "[%d] descriptor is not REPORT",
				bNumInterface);
			return -EINVAL;
		}
		// copy report descriptor for this class
		//copy_config(urb, &report_descriptor->bData[0], report_descriptor->wLength, max);
		if (max - urb->actual_length > 0) {
			int length = min_t(int, report_descriptor->wLength,
					   max - urb->actual_length);
			memcpy(urb->buffer + urb->actual_length,
			       &report_descriptor->bData[0], length);
			urb->actual_length += length;
		}
        }
        break;
	default:
		return -EINVAL;
	}


	//dbg_ep0(1, "urb: buffer: %p buffer_length: %2d actual_length: %2d packet size: %2d", 
	//        urb->buffer, urb->buffer_length, urb->actual_length, device->bus->endpoint_array[0].tx_packetSize);
/*
    if ((urb->actual_length < max) && !(urb->actual_length % device->bus->endpoint_array[0].tx_packetSize)) {
        dbg_ep0(0, "adding null byte");
        urb->buffer[urb->actual_length++] = 0;
        dbg_ep0(0, "urb: buffer_length: %2d actual_length: %2d packet size: %2d", 
                urb->buffer_length, urb->actual_length device->bus->endpoint_array[0].tx_packetSize);
    }
*/
	return 0;

}

/**
 * ep0_recv_setup - called to indicate URB has been received
 * @urb: pointer to struct urb
 *
 * Check if this is a setup packet, process the device request, put results
 * back into the urb and return zero or non-zero to indicate success (DATA)
 * or failure (STALL).
 *
 */
static int ep0_recv_setup (struct urb *urb)
{
	//struct usb_device_request *request = urb->buffer;
	//struct usb_device_instance *device = urb->device;

	struct usb_device_request *request;
	struct usb_device_instance *device;
	int address;

	if (!urb || !urb->device) {
		dbg_ep0 (3, "invalid URB %p", urb);
		return -EINVAL;
	}

	request = &urb->device_request;
	device = urb->device;

	//dbg_ep0(3, "urb: %p device: %p", urb, urb->device);


	//dbg_ep0(2, "-       -       -       -       -       -       -       -       -       -");

	dbg_ep0(2, "bmRequestType:%02x bRequest:%02x wValue:%04x wIndex:%04x wLength:%04x %s",
	        request->bmRequestType, 
                request->bRequest, 
	        le16_to_cpu(request->wValue), 
	        le16_to_cpu(request->wIndex), 
	        le16_to_cpu(request->wLength),
                USBD_DEVICE_REQUESTS(request->bRequest));

	// handle USB Standard Request (c.f. USB Spec table 9-2)
	if ((request->bmRequestType & USB_REQ_TYPE_MASK) != 0) {
		dbg_ep0 (1, "non standard request: %x", request->bmRequestType & USB_REQ_TYPE_MASK);
		return 0;	// XXX
	}
#if 0
	// check that this is for endpoint zero and is a SETUP packet
	if (urb->endpoint->endpoint_address != 0) {
		dbg_ep0 (1, "endpoint non zero: %d or not setup packet: %x",
			 urb->endpoint->endpoint_address, urb->pid);
		return -EINVAL;
	}
#endif

	switch (device->device_state) {
	case STATE_CREATED:
	case STATE_ATTACHED:
	case STATE_POWERED:
		dbg_ep0 (1, "request %s not allowed in this state: %s", 
                                USBD_DEVICE_REQUESTS(request->bRequest), 
                                usbd_device_states[device->device_state]);
		return -EINVAL;

	case STATE_INIT:
	case STATE_DEFAULT:
		switch (request->bRequest) {
		case USB_REQ_GET_STATUS:
		case USB_REQ_GET_INTERFACE:
		case USB_REQ_SYNCH_FRAME:	// XXX should never see this (?)
		case USB_REQ_CLEAR_FEATURE:
		case USB_REQ_SET_FEATURE:
		case USB_REQ_SET_DESCRIPTOR:
		// case USB_REQ_SET_CONFIGURATION:
		case USB_REQ_SET_INTERFACE:
			dbg_ep0 (1, "request %s not allowed in DEFAULT state: %s",
				 USBD_DEVICE_REQUESTS(request->bRequest), usbd_device_states[device->device_state]);
			return -EINVAL;

		case USB_REQ_SET_CONFIGURATION:
		case USB_REQ_SET_ADDRESS:
		case USB_REQ_GET_DESCRIPTOR:
		case USB_REQ_GET_CONFIGURATION:
			break;
		}
	case STATE_ADDRESSED:
	case STATE_CONFIGURED:
		break;
	case STATE_UNKNOWN:
		dbg_ep0 (1, "request %s not allowed in UNKNOWN state: %s", 
                                USBD_DEVICE_REQUESTS(request->bRequest), usbd_device_states[device->device_state]);
		return -EINVAL;
	}

	// handle all requests that return data (direction bit set on bm RequestType)
	if ((request->bmRequestType & USB_REQ_DIRECTION_MASK)) {

		//dbg_ep0(3, "Device-to-Host");

		switch (request->bRequest) {

		case USB_REQ_GET_STATUS:
			return ep0_get_status (device, urb, request->wIndex, request->bmRequestType & USB_REQ_RECIPIENT_MASK);

		case USB_REQ_GET_DESCRIPTOR:
			return ep0_get_descriptor (device, urb,
                                        le16_to_cpu (request->wLength),
                                        le16_to_cpu (request->wValue) >> 8,
                                        le16_to_cpu (request->wValue) & 0xff);

		case USB_REQ_GET_CONFIGURATION:
			return ep0_get_one (device, urb, device->configuration);

		case USB_REQ_GET_INTERFACE:
			return ep0_get_one (device, urb, device->alternate);

		case USB_REQ_SYNCH_FRAME:	// XXX should never see this (?)
			return -EINVAL;

		case USB_REQ_CLEAR_FEATURE:
		case USB_REQ_SET_FEATURE:
		case USB_REQ_SET_ADDRESS:
		case USB_REQ_SET_DESCRIPTOR:
		case USB_REQ_SET_CONFIGURATION:
		case USB_REQ_SET_INTERFACE:
			return -EINVAL;
		}
	}
	// handle the requests that do not return data
	else {


		//dbg_ep0(3, "Host-to-Device");
		switch (request->bRequest) {

		case USB_REQ_CLEAR_FEATURE:
		case USB_REQ_SET_FEATURE:
                        dbg_ep0(0, "Host-to-Device");
			switch (request->bmRequestType & USB_REQ_RECIPIENT_MASK) {
			case USB_REQ_RECIPIENT_DEVICE:
                                // XXX DEVICE_REMOTE_WAKEUP or TEST_MODE would be added here
                                // XXX fall through for now as we do not support either
			case USB_REQ_RECIPIENT_INTERFACE:
			case USB_REQ_RECIPIENT_OTHER:
                                dbg_ep0 (0, "request %s not", USBD_DEVICE_REQUESTS(request->bRequest));
                        default:
				return -EINVAL;

                        case USB_REQ_RECIPIENT_ENDPOINT:
                                dbg_ep0(0, "ENDPOINT: %x", le16_to_cpu(request->wValue));
                                if (le16_to_cpu(request->wValue) == USB_ENDPOINT_HALT) {
                                        return usbd_device_feature (device, le16_to_cpu (request->wIndex),
                                                        request->bRequest == USB_REQ_SET_FEATURE);
                                }
                                else {
                                        dbg_ep0 (1, "request %s bad wValue: %04x", 
                                                        USBD_DEVICE_REQUESTS(request->bRequest), le16_to_cpu(request->wValue));
                                        return -EINVAL ;
                                }
                        }

                case USB_REQ_SET_ADDRESS:
                        // check if this is a re-address, reset first if it is (this shouldn't be possible)
                        if (device->device_state != STATE_DEFAULT) {
                                dbg_ep0 (1, "set_address: %02x state: %s",
					 le16_to_cpu (request->wValue), usbd_device_states[device->device_state]);
				return -EINVAL;
			}
			address = le16_to_cpu (request->wValue);
			if ((address & 0x7f) != address) {
				dbg_ep0 (1, "invalid address %04x %04x", address, address & 0x7f);
				return -EINVAL;
			}
			device->address = address;

			//dbg_ep0(2, "address: %d %d %d", 
			//        request->wValue, le16_to_cpu(request->wValue), device->address);

			usbd_device_event (device, DEVICE_ADDRESS_ASSIGNED, 0);
			return 0;

		case USB_REQ_SET_DESCRIPTOR:	// XXX should we support this?
			dbg_ep0 (0, "set descriptor: NOT SUPPORTED");
			return -EINVAL;

		case USB_REQ_SET_CONFIGURATION:
			// c.f. 9.4.7 - the top half of wValue is reserved
			//
			if ((device->configuration = le16_to_cpu (request->wValue) & 0x7f) != 0) {
				// c.f. 9.4.7 - zero is the default or addressed state, in our case this
				// is the same is configuration zero
				device->configuration = 0;  // TBR - ??????
			}
			// reset interface and alternate settings
			device->interface = device->alternate = 0;

			//dbg_ep0(2, "set configuration: %d", device->configuration);
			usbd_device_event (device, DEVICE_CONFIGURED, 0);
			return 0;

		case USB_REQ_SET_INTERFACE:
			device->interface = le16_to_cpu (request->wIndex);
			device->alternate = le16_to_cpu (request->wValue);
			//dbg_ep0(2, "set interface: %d alternate: %d", device->interface, device->alternate);
			usbd_device_event (device, DEVICE_SET_INTERFACE, 0);
			return 0;

		case USB_REQ_GET_STATUS:
		case USB_REQ_GET_DESCRIPTOR:
		case USB_REQ_GET_CONFIGURATION:
		case USB_REQ_GET_INTERFACE:
		case USB_REQ_SYNCH_FRAME:	// XXX should never see this (?)
			return -EINVAL;
		}
	}
	return -EINVAL;
}

/**
 * ep0_urb_sent - called to indicate URB transmit finished
 * @urb: pointer to struct urb
 * @rc: result
 */
static int ep0_urb_sent (struct urb *urb, int rc)
{
	//dbg_ep0(2, "%s rc: %d", urb->device->name, rc);
	//usbd_dealloc_urb(urb);
	return 0;
}

static struct usb_function_operations ep0_ops = {
	event:		ep0_event,
	recv_setup:	ep0_recv_setup,
	urb_sent:	ep0_urb_sent,
};

struct usb_function_driver ep0_driver = {
	name:		"EP0",
	ops:		&ep0_ops,
};
