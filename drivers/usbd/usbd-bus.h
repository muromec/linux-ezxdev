/*
 * usbd/usbd-bus.c - USB Device Bus Interface Driver Interface
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

/*
 * This file contains the USB Bus Interface Driver Interface definitions.
 *
 * This is the interface between the bottom of the USB Core and the top of
 * the Bus Interace Drivers and is comprised of:
 *
 * 	o public functions exported by the USB Core layer 
 *
 * 	o structures and functions passed by the Function Driver to the USB
 * 	Core layer
 *
 * USB Bus Interface Drivers are structured such that the upper edge
 * implements interfaces to the USB Core layer to provide low level USB
 * services and the lower edge interfaces to the actual USB Hardware
 * (typically called the USB Device Controller or UDC.)
 *
 */

struct usb_endpoint_map; 

/* Operations that the slave layer or function driver can use to interact
 * with the bus interface driver. 
 *
 * The send_urb() function is used by the usb-device endpoint 0 driver and
 * function drivers to submit data to be sent.
 *
 * The cancel_urb() function is used by the usb-device endpoint 0 driver and
 * function drivers to remove previously queued data to be sent.
 *
 * The endpoint_halted() function is used by the ep0 control function to
 * check if an endpoint is halted.
 *
 * The device_feature() function is used by the ep0 control function to
 * set/reset device features on an endpoint.
 *
 * The device_event() function is used by the usb device core to tell the
 * bus interface driver about various events.
 */
struct usb_bus_operations {
	int (*bus_enable) (struct usb_bus_instance *, char *);
	int (*bus_disable) (struct usb_bus_instance *, char *);
	int (*bus_disconnect) (struct usb_bus_instance *, char *);
	int (*bus_connect) (struct usb_bus_instance *, char *);
	int (*bus_pm_off) (struct usb_bus_instance *, char *);
	int (*bus_pm_on) (struct usb_bus_instance *, char *);
	int (*bus_serial_number) (struct usb_bus_instance *, char *);

	int (*bus_attached) (struct usb_bus_instance *);
        int (*bus_connected) (struct usb_bus_instance *);
	int (*start_endpoint_in) (struct usb_bus_instance *, struct usb_endpoint_instance *);
	int (*start_endpoint_out) (struct usb_bus_instance *, struct usb_endpoint_instance *);
	int (*cancel_urb_irq) (struct urb *);
	int (*endpoint_halted) (struct usb_bus_instance *, int);
	int (*device_feature) (struct usb_bus_instance *, int, int);
	int (*device_event) (struct usb_bus_instance *, usb_device_event_t, int);
	struct usb_endpoint_map *(*request_endpoints) (int, struct usb_endpoint_request *);
        int (*set_endpoints) (int , struct usb_endpoint_map *);
};


/* Endpoint Map
 *
 * An array of these structures is created by the bus interface driver to
 * show what endpoints have been configured for the function driver.
 */
struct usb_endpoint_map {
        u8 configuration;
        u8 interface;
        u8 alternate;
	u8 bEndpointAddress[2];		// logical endpoint address
	u16 wMaxPacketSize[2];		// packetsSize for requested endpoint
	u8 bmAttributes[2];		// requested endpoint type
	u16 transferSize[2];		// transferSize for bulk transfers
	u8 physicalEndpoint[2];		// physical endpoint number
	struct usb_endpoint_instance 	*endpoint;
};


/* Endpoint configuration
 *
 * Per endpoint configuration data. Used to track which function driver owns
 * an endpoint.
 *
 */
struct usb_endpoint_instance {
	int 				bEndpointAddress;	// logical endpoint address 
	int 				physical_endpoint;	// physical endpoint address - bus interface specific
        int 				bmAttributes;       	// endpoint type
	u16 				wMaxPacketSize;		// packet size for requested endpoint

	// control
        int 				status;             	// halted
	int 				state;			// available for use by bus interface driver

	// receive side
	struct urb_link 		rcv;			// received urbs
	struct urb_link 		rdy;			// empty urbs ready to receive
	struct urb 			*rcv_urb;		// active urb
	__u32 				rcv_transferSize;	// maximum transfer size from function driver
	int 				rcv_error;		// current bulk-in has an error

	// transmit side
	struct urb_link 		tx;			// urbs ready to transmit
	struct urb_link 		done;			// transmitted urbs
	struct urb 			*tx_urb;		// active urb

	__u32 				sent;			// data already sent
	__u32 				last;			// data sent in last packet XXX do we need this
};

/* endpoint zero states
 */
#define WAIT_FOR_SETUP          0
#define DATA_STATE_XMIT         1
#define DATA_STATE_NEED_ZLP     2
#define WAIT_FOR_OUT_STATUS     3
#define DATA_STATE_RECV         4
#define DATA_STATE_PENDING_XMIT 5


/* Bus Interface data structure
 *
 * Keep track of specific bus interface. 
 *
 * This is passed to the usb-device layer when registering. It contains all
 * required information about each real bus interface found such that the
 * usb-device layer can create and maintain a usb-device structure.
 *
 * Note that bus interface registration is incumbent on finding specific
 * actual real bus interfaces. There will be a registration for each such
 * device found.
 *
 * The max_tx_endpoints and max_rx_endpoints are the maximum number of
 * possible endpoints that this bus interface can support. The default
 * endpoint 0 is not included in these counts.
 *
 */
struct usb_bus_driver {
	char				*name;
	u8				max_endpoints;	// maximimum number of rx enpoints
	u8				maxpacketsize;
	u8				HighSpeedCapable;
	struct usb_device_description	*device_description;
	struct usb_bus_operations	*bops;
};


typedef enum usbd_admin {
        usbd_admin_enable = 0,
        usbd_admin_disable = 1,
        usbd_admin_connect = 2,
        usbd_admin_disconnect = 3,
        usbd_admin_pm_off = 4,
        usbd_admin_pm_on = 5,
        usbd_admin_serial_number = 6
} usbd_admin_t;

typedef int (*usbd_admin_proc_t) (struct usb_bus_instance *, char *);


/* Function configuration structure
 *
 * This is allocated for each configured instance of a function driver.
 *
 * It stores pointers to the usb_function_driver for the appropriate function,
 * and pointers to the USB HOST requested usb_configuration_description and
 * usb_interface_description.
 *
 * The privdata pointer may be used by the function driver to store private
 * per instance state information.
 *
 */
struct usb_function_instance {
	struct usb_bus_instance 	*bus;
	struct usb_function_driver 	*function_driver;
	void 				*privdata;		// private data for the function
	__u8 				endpointsRequested;	// number of requested endpoints
	struct usb_endpoint_map 	*endpoint_map_array;	// map of endpoints requested by function driver
};


/* Bus Interface configuration structure
 *
 * This is allocated for each configured instance of a bus interface driver.
 *
 * It contains a pointer to the appropriate bus interface driver.
 *
 * The privdata pointer may be used by the bus interface driver to store private
 * per instance state information.
 */
struct usb_bus_instance {

	struct usb_bus_driver           *driver;

        usbd_admin_proc_t               admin [8];

        int                             endpoints;
	struct usb_endpoint_instance    *endpoint_array;	// array of available configured endpoints

	char                            *serial_number_str;

	usb_device_status_t             status;	                // device status
        usbd_bus_state_t                bus_state;
	usb_device_state_t              device_state;		// current USB Device state
	usb_device_state_t              suspended_state;	// previous USB Device state

	struct usb_function_instance    *ep0;	                // ep0 configuration
	struct usb_function_instance    *function_instance;


        u8                              HighSpeedFlag;
	u8                              ConfigurationValue;	// current set configuration (zero is default)
        u8                              bNumInterfaces;        	// number of interfaces in the current configuration
        u8                              *alternates;         	// array[0..interfaces-1] of alternate settings for each interface

	struct tq_struct                device_bh;		// runs as bottom half, equivalent to interrupt time

	void                            *privdata;		// private data for the bus interface
	char				*arg;

#ifdef CONFIG_ARCH_EZX
	u8                              Raw_ConfigurationValue;	// added for compliance test (zero is default)
#endif
};

/* bus driver registration
 *
 * Called by bus interface drivers to register themselves when loaded
 * or de-register when unloading.
 */
struct usb_bus_instance *usbd_register_bus (struct usb_bus_driver *);
void usbd_deregister_bus (struct usb_bus_instance *);



/* Enable/Disable Function
 *
 * Called by a bus interface driver to select and enable a specific function 
 * driver.
 */
int usbd_enable_function_irq (struct usb_bus_instance *, char *);
void usbd_disable_function (struct usb_bus_instance *);


void usbd_flush_endpoint_irq (struct usb_endpoint_instance *);
void usbd_flush_endpoint (struct usb_endpoint_instance *);

/* 
 * usbd_configure_device is used by function drivers (usually the control endpoint)
 * to change the device configuration.
 *
 * usbd_device_event is used by bus interface drivers to tell the higher layers that
 * certain events have taken place.
 */
void usbd_bus_event_irq (struct usb_bus_instance *, usb_device_event_t, int);
void usbd_bus_event (struct usb_bus_instance *, usb_device_event_t, int);


/**
 * usbd_recv_setup_irq - process a received urb
 * @urb: pointer to an urb structure
 *
 * Used by a USB Bus interface driver to pass received data in a URB to the
 * appropriate USB Function driver.
 *
 * This function must return 0 for success and -EINVAL if the request
 * is to be stalled.
 *
 * Not that if the SETUP is Host to Device with a non-zero wLength then there
 * *MUST* be a valid receive urb queued OR the request must be stalled.
 */
int usbd_recv_setup_irq (struct usb_function_instance*, struct usb_device_request *);


void usbd_urb_sent_finished_irq (struct urb *, int );
void usbd_urb_recv_finished_irq (struct urb *, int );


/*
 * Detach and return the first urb in a list with a distinguished
 * head "hd", or NULL if the list is empty.
 */
struct urb *usbd_first_urb_detached_irq (urb_link * hd);

/*
 * Get a descriptor
 */
int usbd_get_descriptor (struct usb_bus_instance *bus, u8 *buffer, int max, int descriptor_type, int index);

