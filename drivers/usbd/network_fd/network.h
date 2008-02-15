/*
 * usbd/network_fd/network.h - Network Function Driver
 *
 *      Copyright (c) 2002, 2003, 2004 Belcarra
 *
 * By: 
 *      Chris Lynne <cl@belcarra.com>
 *      Stuart Lynne <sl@belcarra.com>
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

#ifndef NETWORK_FD_H
#define NETWORK_FD_H 1

typedef enum network_encapsulation {
        simple_net, simple_crc, 
} network_encapsulation_t;

typedef enum network_hotplug_status {
	hotplug_unkown,
	hotplug_attached,
	hotplug_detached
} network_hotplug_status_t;

typedef enum network_type {
        network_unknown,
        network_blan,
        network_safe,
        network_cdc,
        network_basic,
        network_basic2,
} network_type_t;

struct usb_network_private {

        struct net_device_stats stats;  /* network device statistics */

	int flags;
	struct usb_function_instance *function;
	unsigned int maxtransfer;
        rwlock_t rwlock;

	network_hotplug_status_t hotplug_status;
        network_type_t network_type;

        int state;

        int mtu; 
	int crc;
#if defined(CONFIG_USBD_NETWORK_BLAN_FERMAT)
        int fermat;
#endif

        unsigned int stopped; 
        unsigned int restarts;

        unsigned int max_queue_entries;
        unsigned int max_queue_bytes;

        unsigned int queued_entries;
        unsigned int queued_bytes;

        time_t avg_queue_entries;

        time_t jiffies;
        unsigned long samples;

        int have_interrupt;

        struct urb *int_urb;

        network_encapsulation_t encapsulation;

	struct tq_struct notification_bh;

#ifdef CONFIG_HOTPLUG
	struct tq_struct hotplug_bh;
#endif
};

// XXX this needs to be co-ordinated with rndis.c maximum's
#define MAXFRAMESIZE 2000

#if !defined(CONFIG_USBD_MAXPOWER)
	#define CONFIG_USBD_MAXPOWER                    0
#endif

#if !defined(CONFIG_USBD_MANUFACTURER)
	#define CONFIG_USBD_MANUFACTURER              	"Belcarra"
#endif


#if !defined(CONFIG_USBD_SERIAL_NUMBER_STR)
	#define CONFIG_USBD_SERIAL_NUMBER_STR           ""
#endif

/*
 * Lineo specific 
 */

#define VENDOR_SPECIFIC_CLASS           0xff
#define VENDOR_SPECIFIC_SUBCLASS        0xff
#define VENDOR_SPECIFIC_PROTOCOL        0xff

/*
 * Lineo Classes
 */
#define LINEO_CLASS                     0xff

#define LINEO_SUBCLASS_BASIC_NET          0x01
#define LINEO_SUBCLASS_BASIC_SERIAL       0x02

/*
 * Lineo Protocols
 */
#define LINEO_BASIC_NET_CRC             0x01
#define LINEO_BASIC_NET_CRC_PADDED      0x02

#define LINEO_BASIC_SERIAL_CRC          0x01
#define LINEO_BASIC_SERIAL_CRC_PADDED   0x02


/*
 * endpoint and interface indexes
 */
#define BULK_OUT        0x00
#define BULK_IN         0x01
#define INT_IN          0x02
#define ENDPOINTS       0x03

#define COMM_INTF       0x00
#define DATA_INTF       0x01


/* bmDataCapabilities */
#define BMDATA_CRC              	0x01
#define BMDATA_PADBEFORE        	0x02
#define BMDATA_PADAFTER         	0x04
#define BMDATA_FERMAT           	0x08
#define BMDATA_HOSTNAME         	0x10

/* bmNetworkCapabilities */
#define BMNETWORK_SET_PACKET_OK 	0x01
#define BMNETWORK_NOBRIDGE 	0x02


/*
 * BLAN Data Plane
 */
//#define CONFIG_USBD_NETWORK_PADBYTES 	8
//#define CONFIG_USBD_NETWORK_PADAFTER 	1
//#undef CONFIG_USBD_NETWORK_PADBEFORE 	
//#define CONFIG_USBD_NETWORK_CRC 	1


extern __u8 network_requested_endpoints[ENDPOINTS+1];
extern __u16 network_requested_transferSizes[ENDPOINTS+1];
extern struct usb_network_private Usb_network_private;
extern __u8 local_dev_addr[ETH_ALEN];
extern __u8 remote_dev_addr[ETH_ALEN];

extern struct usb_function_operations network_fd_function_ops;
extern struct usb_network_private Usb_network_private;

struct usb_class_safe_networking_mdlm_descriptor {
        __u8 bFunctionLength;		// 0x06
        __u8 bDescriptorType;		// 0x24
        __u8 bDescriptorSubtype;        // 0x13
        __u8 bGuidDescriptorType;	// 0x00
        __u8 bmNetworkCapabilities;
        __u8 bmDataCapabilities;
} __attribute__ ((packed));

struct usb_class_blan_networking_mdlm_descriptor {
        __u8 bFunctionLength;		// 0x07
        __u8 bDescriptorType;		// 0x24
        __u8 bDescriptorSubtype;        // 0x13
        __u8 bGuidDescriptorType;	// 0x01
        __u8 bmNetworkCapabilities;
        __u8 bmDataCapabilities;
        __u8 bPad;
} __attribute__ ((packed));





#endif /* NETWORK_FD_H */
