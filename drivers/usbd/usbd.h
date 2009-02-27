/*
 * usbd/usbd.h 
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
 * This program is free software; you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program; if
 * not, write to the Free Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
/*
 * Copyright (C) 2005-2006 Motorola Inc.
 *
 *  2005-Nov-11  add Belcarra usbd patch, 	Zhao liang
 *  2006-Jan-12  modify for test command 	Wang Jordan
 *  2006-May-12  modify for usb2.0, 		Li xin
 *
 */

/*
 * This file contains the USB Device Core Common definitions. It also contains definitions for
 * functions that are commonly used by both Function Drivers and Bus Interface Drivers.
 *
 * Specifically:
 *
 * 	o public functions exported by the USB Core layer 
 *
 * 	o common structures and definitions 
 *
 */


/* 
 * Device Events 
 *
 * These are defined in the USB Spec (c.f USB Spec 2.0 Figure 9-1).
 *
 * There are additional events defined to handle some extra actions we need to have handled.
 *
 */
typedef enum usb_device_event {

	DEVICE_UNKNOWN,		// bi - unknown event
	DEVICE_INIT,		// bi  - initialize
	DEVICE_CREATE,		// bi  - 
	DEVICE_HUB_CONFIGURED,	// bi  - bus has been plugged int
	DEVICE_RESET,		// bi  - hub has powered our port

	DEVICE_ADDRESS_ASSIGNED,// ep0 - set address setup received
	DEVICE_CONFIGURED,	// ep0 - set configure setup received
	DEVICE_SET_INTERFACE,	// ep0 - set interface setup received

	DEVICE_SET_FEATURE,	// ep0 - set feature setup received
	DEVICE_CLEAR_FEATURE,	// ep0 - clear feature setup received

	DEVICE_DE_CONFIGURED,	// ep0 - set configure setup received for ??

	DEVICE_BUS_INACTIVE,	// bi  - bus in inactive (no SOF packets)
	DEVICE_BUS_ACTIVITY,	// bi  - bus is active again

	DEVICE_POWER_INTERRUPTION,// bi  - hub has depowered our port
	DEVICE_HUB_RESET,	// bi  - bus has been unplugged
	DEVICE_DESTROY,		// bi  - device instance should be destroyed
	DEVICE_CLOSE,		// bi  - device instance should be destroyed

} usb_device_event_t;



struct usb_bus_instance;
struct usb_function_instance;

typedef struct urb_link {
	struct urb_link *next;
	struct urb_link *prev;
} urb_link;

/* URB Status
 *
 */
typedef enum usbd_urb_status {
        SEND_IN_QUEUE,
	SEND_IN_PROGRESS,
	SEND_FINISHED_OK,
	SEND_FINISHED_ERROR,
	SEND_FINISHED_CANCELLED,
	RECV_IN_QUEUE,
	RECV_IN_PROGRESS,
	RECV_OK,
	RECV_ERROR,
	RECV_CANCELLED
} usbd_urb_status_t;



/* USB Data structure 
 *
 * This is used for both sending and receiving data. 
 *
 * The callback function is used to let the function driver know when transmitted data has been
 * sent.
 *
 * The callback function is set by the alloc_recv function when an urb is allocated for
 * receiving data for an endpoint and used to call the function driver to inform it that data
 * has arrived.
 *
 * Note that for OUT urbs the buffer is always allocated to a multiple of the packetsize that is
 * 1 larger than the requested size. This prevents overflow if the host unexpectedly sends a
 * full sized packet when we are expecting a short one (the host is always right..)
 */

struct urb {

	struct usb_bus_instance 	*bus;
	struct usb_function_instance 	*function_instance;
	struct usb_endpoint_instance 	*endpoint;

	int 				(*callback) (struct urb *, int);

	u8 			        *buffer;	// data received (OUT) or being sent (IN)
	u32 				request_length; // maximum data expected for OUT
	u32 				buffer_length;	// allocated size of buffer
	u32 				actual_length;	// actual data received (OUT or being sent (IN)
        u32 				flags;
        
	void 				*privdata;

	struct urb_link 		link;	
	usbd_urb_status_t 		status;		// what is the current status of the urb
	time_t 				jiffies;	// timestamp for when urb was started or finished
	u16 				framenum;	// SOF framenum when urb was finished

	/* the following parameters are used for ISP1583 DMA mode*/
	u8 				dma_mode;	// 1: DMA is used, 0: DMA can not be used
	u32 				dma_physaddr;	// physical address of current data buffer
};

#define USBD_URB_SENDZLP        0x01


/* 
 * Device State (c.f USB Spec 2.0 Figure 9-1)
 *
 * What state the usb device is in. 
 *
 * Note the state does not change if the device is suspended, we simply set a
 * flag to show that it is suspended.
 *
 */
typedef enum usb_device_state {
	STATE_INIT,		// just initialized
	STATE_CREATED,		// just created
	STATE_ATTACHED,		// we are attached 
	STATE_POWERED,		// we have seen power indication (electrical bus signal)
	STATE_DEFAULT,		// we been reset
	STATE_ADDRESSED,	// we have been addressed (in default configuration)
	STATE_CONFIGURED,	// we have seen a set configuration device command
	STATE_SUSPENDED,
	STATE_UNKNOWN,		// destroyed
} usb_device_state_t;

/*
 * Device status
 *
 * Overall state
 */
typedef enum usb_device_status {
	USBD_OPENING,		// we are currently opening
	USBD_RESETING,		// we are currently opening
	USBD_OK,		// ok to use
	USBD_SUSPENDED,		// we are currently suspended
	USBD_CLOSING,		// we are currently closing
	USBD_CLOSED,		// we are currently closing
	USBD_UNKNOWN,
} usb_device_status_t;

/*
 * Bus state
 *
 * enabled or disabled
 */
typedef enum usbd_bus_state {
        usbd_bus_state_unknown,
        usbd_bus_state_disabled,
        usbd_bus_state_enabled
} usbd_bus_state_t;



/* Endpoint Request
 *
 * An array of these structures is initialized by each function driver to specify the endpoints
 * it requires. 
 *
 * The bus interface driver will attempt to fulfill these requests with the actual endpoints it
 * has available.
 *
 * Note that in most cases the bEndpointAddress should be left as zero except for specialized
 * function drivers that both require a specific value and know ahead of time that the specified
 * value is legal for the bus interface driver being used.
 */
struct usb_endpoint_request {
	u8 configuration;		// configuration endpoint will be in
	u8 interface;			// interface endpoint will be in
	u8 alternate;			// altsetting for this request
	u8 bmAttributes;		// endpoint type AND direction
	u16 fs_requestedTransferSize;	// max full speed transfer size for this endpoint
	u16 hs_requestedTransferSize;	// max high speed transfer size for this endpoint
        u8 bEndpointAddress;            // specific bEndpointAddress function driver requires
	u8 physical;			// physical endpoint used
};



struct usb_function_operations;
struct usb_function_driver;



/*
 * USB Bus Interface Driver structures
 *
 * Driver description:
 * 
 *  struct usb_bus_operations
 *  struct usb_bus_driver
 *
 */

struct usb_bus_operations;
struct usb_bus_driver;
extern int usbd_maxstrings;


