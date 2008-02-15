/*
 * usbd/usbd-func.h - USB Device Function Driver Interface
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

/*
 * This file contains the USB Function Driver Interface definitions.
 *
 * This is the interface between the bottom of the USB Function drivers and
 * the top of the USB Core and is comprised of:
 *
 * 	o public functions exported by the USB Core layer 
 *
 * 	o structures and functions passed by the Bus Interface Driver to the
 * 	USB Core layer
 *
 *
 * USB Function Drivers are structured such that the upper edge implements
 * some specific function (network, serial, mass storage etc) and the lower
 * edge interfaces to the USB Device Core for USB services.
 *
 */

/*
 * USB Function Driver structures
 *
 * Descriptors:
 *   struct usb_endpoint_description
 *   struct usb_interface_description 
 *   struct usb_configuration_description
 *
 * Driver description:
 *   struct usb_function_driver
 *   struct usb_function_operations
 *
 */

struct usb_function_operations {
	int (*function_enable) (struct usb_function_instance *);
	void (*function_disable) (struct usb_function_instance *);
	void (*event_irq) (struct usb_function_instance *, usb_device_event_t, int);
	int (*recv_setup_irq) (struct usb_device_request *);
	int (*alloc_urb_data) (struct urb *, int);
	void (*dealloc_urb_data) (struct urb *);
};


/*
 * function driver definitions
 */
struct usb_alternate_instance {
	struct usb_interface_descriptor *interface_descriptor;
	int 				classes;
	struct usb_generic_class_descriptor 	**class_list;
	int 				endpoints;
        struct usb_endpoint_descriptor  **endpoint_list;
	u8				*endpoint_indexes;
};


/*
 * usb device description structures
 */

struct usb_alternate_description {
	struct usb_interface_descriptor *interface_descriptor;
	char 				*iInterface;

	// list of CDC class descriptions for this alternate interface
	u8 				classes;
	struct usb_generic_class_descriptor 	**class_list;

	// list of endpoint descriptions for this alternate interface
	u8 				endpoints;
        struct usb_endpoint_descriptor  **endpoint_list;

	// list of indexes into endpoint request map for each endpoint descriptor
	u8				*endpoint_indexes;
};

struct usb_interface_description {
	// list of alternate interface descriptions for this interface
	u8 				alternates;
	struct usb_alternate_description *alternate_list;
};

struct usb_configuration_description {
        struct usb_configuration_descriptor *configuration_descriptor;
	char 				*iConfiguration;
	// list of interface descriptons for this configuration
	u8 				bNumInterfaces;
	struct usb_interface_description *interface_list;
	int 				configuration_type;
};

struct usb_device_description {
        struct usb_device_descriptor 	*device_descriptor;
	struct usb_device_qualifier_descriptor *device_qualifier_descriptor;
	char 				*iManufacturer;
	char 				*iProduct;
	char 				*iSerialNumber;
        u8 				endpointsRequested;
	struct usb_endpoint_request 	*requestedEndpoints;
	struct usb_function_endpoint_config *endpointConfig;
};


struct usb_interface_instance {
	u8 				alternates;
	struct usb_alternate_instance 	*alternates_instance_array;
};

struct usb_configuration_instance {
	int 				bNumInterfaces;
	struct usb_configuration_descriptor *configuration_descriptor;
	struct usb_interface_instance 	*interface_instance_array;
	struct usb_function_driver 	*function_driver;
};

#ifdef CONFIG_ARCH_EZXBASE
#define BLAN_IP_ADDR_GNPO
#endif

extern struct usb_string_descriptor **usb_strings;
__u8 usbd_alloc_string (char *);
#ifdef BLAN_IP_ADDR_GNPO
__u8 usbd_update_string_at_index (__u8, char *);
#endif
void usbd_dealloc_string (__u8 );
void usbd_dealloc_descriptor_strings (struct usb_descriptor *);
struct usb_string_descriptor *usbd_get_string (__u8);




/* Function Driver data structure
 *
 * Function driver and its configuration descriptors. 
 *
 * This is passed to the usb-device layer when registering. It contains all
 * required information about the function driver for the usb-device layer
 * to use the function drivers configuration data and to configure this
 * function driver an active configuration.
 *
 * Note that each function driver registers itself on a speculative basis.
 * Whether a function driver is actually configured will depend on the USB
 * HOST selecting one of the function drivers configurations. 
 *
 * This may be done multiple times WRT to either a single bus interface
 * instance or WRT to multiple bus interface instances. In other words a
 * multiple configurations may be selected for a specific bus interface. Or
 * the same configuration may be selected for multiple bus interfaces. 
 *
 */
struct usb_function_driver {
	const char 			*name;
	struct usb_function_operations 	*fops;	// functions 

	// device & configuration descriptions 
	struct usb_device_description 	*device_description;
	struct usb_configuration_description *configuration_description;
	int 				bNumConfigurations;

	u16				idVendor;
	u16				idProduct;
	u16				bcdDevice;

	// constructed descriptors
	struct usb_device_descriptor 	*device_descriptor;
	struct usb_device_qualifier_descriptor *device_qualifier_descriptor;
	struct usb_configuration_instance *configuration_instance_array;

	struct list_head 		drivers;	// linked list 
};


struct usb_function_instance;

/* function driver registration
 *
 * Called by function drivers to register themselves when loaded
 * or de-register when unloading.
 */
//int usbd_strings_init(void);
int usbd_register_function (struct usb_function_driver *);
void usbd_deregister_function (struct usb_function_driver *);

/* Access to function privdata
 */
void *usbd_function_get_privdata(struct usb_function_instance *function);
void usbd_function_set_privdata(struct usb_function_instance *function, void *privdata);
/*
 * Called to queue urb's for send or recv
 */
int usbd_send_urb (struct urb *urb);
int usbd_start_recv (struct urb *);

/*
 * queury endpoint astatus
 */
int usbd_endpoint_halted (struct usb_function_instance *, int);
int usbd_device_feature (struct usb_function_instance *, int, int);


/*
 * query bus state and status
 */
usb_device_state_t usbd_device_state(struct usb_function_instance *);
usbd_bus_state_t usbd_bus_state(struct usb_function_instance *);
usb_device_status_t usbd_bus_status(struct usb_function_instance *);


/* urb allocation
 *
 * Used to allocate and de-allocate urb structures.
 * usb_cancel_urb is used to cancel a previously submitted data urb for sending.
 */
struct urb *usbd_alloc_urb (struct usb_function_instance *, int, int length, int (*callback) (struct urb *, int));
struct urb *usbd_alloc_urb_ep0 (struct usb_function_instance *, int length, int (*callback) (struct urb *, int));
void usbd_dealloc_urb (struct urb *urb);
int usbd_alloc_urb_data (struct urb *, int);
int usbd_cancel_urb_irq (struct urb *);
void usbd_flush_endpoint_index (struct usb_function_instance *, int );


/* endpoint information
 *
 * Used by function drivers to get specific endpoint information
 */
int usbd_endpoint_wMaxPacketSize(struct usb_function_instance *, int, int);
#ifndef CONFIG_ARCH_EZX
int usbd_endpoint_wMaxPacketSize_ep0(struct usbd_function_instance *, int);
#endif
int usbd_endpoint_bEndpointAddress(struct usb_function_instance *, int, int);
int usbd_endpoint_transferSize(struct usb_function_instance *, int, int);
int usbd_endpoint_interface(struct usb_function_instance *, int);
int usbd_interface_AltSetting(struct usb_function_instance *, int);
int usbd_ConfigurationValue(struct usb_function_instance *);
void usbd_endpoint_update(struct usb_function_instance *, int , struct usb_endpoint_descriptor *, int);
int usbd_high_speed(struct usb_function_instance *);

#ifdef BLAN_IP_ADDR_GNPO
/*
 *  * well-known, preallocated string indices
 *   */
#define STRINDEX_LANGID     0
#define STRINDEX_IPADDR     1
#define STRINDEX_PRODUCT    2
#endif
