/*
 * linux/drivers/usbd/net_fd/netproto.h 
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

//#define NP_ENABLED         0x01
//#define NP_REGISTERED      0x01

/* netproto_dev support routines ***************************************************************** */


#define MAX_NP_CMDS     16	// length of queue of commands for bh

#define NPC_CREATE      0x01	// not presently used
#define NPC_REGISTER    0x02
#define NPC_ENABLE      0x03
#define NPC_DISABLE     0x04
#define NPC_UNREGISTER  0x05
#define NPC_CLOSE       0x07

   /* These should really be in the bus interface, but in order to
      avoid putting a whole task structure into there like the
      netproto_registration_task, it's included here. */

#define NPC_SUSPEND     0x08	// bus inactive - host has suspended
#define NPC_RESUME      0x09	// bus active - host has resumed

   /* Values for device state */

#define NP_REGISTERED   0x01	// device has been registered
#define NP_ENABLED      0x02
#define NP_DESTROYED    0x04
#define	NP_CLOSING	0x08
#define	NP_LOCKED	0x10


//#define NP_ENABLE       0x10
//#define NP_DISABLE      0x20
//#define NP_ENABLED      0x40

/* per network interface device structure
 */

struct netproto_dev {
	struct net_device *net_dev;	// network device information
	struct net_device_stats stats;	// network device statistics

	int interface;		// interface number
	int mac_was_set;


	int state;
	int ops_in_progress;
	int cmd_queue_count;
	unsigned char queued_cmds[MAX_NP_CMDS];

	int (*xmit_skb) (int, struct sk_buff *);	// use to send a network buffer
	int (*set_addr) (int, void *, int);	// set address
	int (*tx_timeout) (int);	// timeout

	int addr_set;		// addr set by ioctl flag
	int addr_mac;		// addr provided flag

	// XXX is this sufficent should we just use
	// module netproto_rwlock for everything
	// lock for reading/writing
	rwlock_t rwlock;

	int mtu;

	unsigned int stopped;
	unsigned int restarts;

	unsigned int max_queue_entries;
	unsigned int max_queue_bytes;

	unsigned int queued_entries;
	unsigned int queued_bytes;

	time_t avg_queue_entries;

	time_t jiffies;
	unsigned long samples;

};


extern char *netproto_procname;	// name to use for proc filesystem
extern int netproto_devices;	// maximum number of interfaces that can be created
extern struct netproto_dev **netproto_device_array;	// pointer to array of pointers to netproto dev structures
extern rwlock_t netproto_rwlock;	// lock for netproto device array access

#include "../usbd-debug.h"

// Some debug macros for the inlines...
extern int dbgflg_usbdfd_prx;
extern int dbgflg_usbdfd_ptx;
#ifndef dbg_prx
#define dbg_prx(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_prx,lvl,fmt,##args)
#endif
#ifndef dbg_ptx
#define dbg_ptx(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_ptx,lvl,fmt,##args)
#endif

extern debug_option *netproto_get_dbg_table (void);


/* proc file system */
#define NET_DEVICE_CONDITION_UNKNOWN 0
#define NET_DEVICE_CONDITION_RESET   1
#define NET_DEVICE_CONDITION_OK      2
#define NET_DEVICE_CONDITION_FAIL    3


/**
 * netproto
 *
 * This is a simple linux network driver library that is suitable for
 * implementing layered network drivers. I.e. networking over adsl or usb.
 *
 */


/**
 * netproto_modinit - initialize netproto library
 * @int: number of interfaces to allow
 *
 * Initialize the netproto library setting the maximum number of network
 * interfaces that may be created.
 */
int netproto_modinit (char *, int);

/* *
 * netproto_create - create network interface
 * @name: name
 * @int (*)(struct sk_buff *)
 * @int (*)(void *,int)
 * @void * - pointer to mac address buffer
 * @int - length of mac address buffer
 * @mtu: maximum mtu
 * @max_queue_entries:
 * @max_queue_bytes:
 * @flags
 *
 * Create a network interface, providing an xmit skb function, an option
 * function to set network addreses and an optional mac address.
 *
 * A returned value greater than zero indicates success and can be used with
 * subsequent function calls to indicate the created network interface.
 */

int netproto_create (char *,
		     int (*)(int, struct sk_buff *),
		     int (*)(int, void *, int), int (*)(int), void *, int, int, int, int, int);

/* *
 * netproto_command -
 * @device
 */
//void netproto_command(int, int);


/* *
 * netproto_enable -
 * @int
 * @flags
 */
void netproto_enable (int, int);

/* *
 * netproto_recv - receive a network buffer
 * @int - network interface
 * @struct sk_buff * - network buffer
 *
 * Call to pass up a network buffer.
 */

static __inline__ int netproto_recv (int, struct sk_buff *);


/* *
 * netproto_done - transmit done
 * @int - network interface
 * @struct sk_buff * - network buffer
 * @int - non-zero indicates failure
 *
 * Call to indicate that a network buffer has been transferred.
 */

static __inline__ int netproto_done (int, struct sk_buff *, int);


/* *
 * netproto_control - control network interface
 * @int - network interface
 * @int - operation
 *
 * Control a network interface, 
 */

int netproto_control (int, int);


/* *
 * netproto_destroy - destroy a network interface
 * @int - network interface
 *
 * Call to tear down a previously created network interface
 */

int netproto_destroy (int);

/* *
* netproto_rx_dropped
* @int - network interface
*
 */
void netproto_rx_dropped (int);

/* *
* netproto_rx_length_error
* @int - network interface
*
 */
void netproto_rx_length_error (int);

/* *
* netproto_rx_over_error
* @int - network interface
*
 */
void netproto_rx_over_error (int);

/* *
* netproto_rx_crc_error
* @int - network interface
*
 */
void netproto_rx_crc_error (int);

/* *
* netproto_rx_frame_error
* @int - network interface
*
 */
void netproto_rx_frame_error (int);

/* *
* netproto_rx_fifo_error
* @int - network interface
*
 */
void netproto_rx_fifo_error (int);

/* *
* netproto_rx_missed_error
* @int - network interface
*
 */
void netproto_rx_missed_error (int);

/* *
 * netproto_rx_packets
 * @int - network interface
 * @len:
 *
 */
void netproto_rx_packets (int, int);

/* *
 * netproto_tx_dropped
 * @int - network interface
 *
 */
void netproto_tx_dropped (int);

/* *
 * netproto_tx_carrier_errors
 * @int - network interface
 *
 */
void netproto_tx_carrier_errors (int);

/* *
 * netproto_tx_fifo_errors
 * @int - network interface
 *
 */
void netproto_tx_fifo_errors (int);

/* *
 * netproto_tx_packets
 * @int - network interface
 * @len:
 *
 */
void netproto_tx_packets (int, int);

/* *
 * netproto_name
 * @int - network interface
 *
 */
char *netproto_name (int interface);

/* *
* netproto_mtu
* @int - network interface
*
 */
int netproto_mtu (int interface);

/* *
 * netproto_modexit
 *
 * Call to unload the library.
 */

/* *
 * netproto_on
 * @int - network interface
 *
 */
void netproto_on (int interface);

/* *
 * netproto_off
 * @int - network interface
 *
 */
void netproto_off (int interface);


int netproto_modexit (void);


/**
 * netproto_recv - receive a network buffer
 * @interface: network interface
 * @skb: pointer to network buffer
 *
 * Call to pass up a network buffer.
 *
 * Called by netproto_recv() to process a received skb and push it to the
 * network layer. Return non-zero if the skb was not freed.
 */

static __inline__ int netproto_recv (int interface, struct sk_buff *skb)
{
	struct netproto_dev *device;
	struct net_device *net_dev;
	int rc;

	//printk(KERN_DEBUG"netproto_recv\n");
	//dbg_prx(0, "receiving %x %d", skb, skb->len);

	if ((interface >= netproto_devices) || (device = netproto_device_array[interface]) == NULL) {
		dbg_prx (0, "bad interface: %d", interface);
		return -EINVAL;
	}

	if (device->state != (NP_REGISTERED | NP_ENABLED)) {
		dbg_prx (0, "not registered: state: %x", device->state);
		return -EINVAL;
	}

	if (!(net_dev = device->net_dev)) {
		dbg_prx (0, "bad net_dev: %p", net_dev);
		return -EINVAL;
	}
	// return -EINVAL;

	if (!netif_device_present (net_dev)) {
		dbg_prx (0, "device not present");
		return -EINVAL;
	}

	if (!netif_carrier_ok (net_dev)) {
		dbg_prx (0, "no carrier");
		return -EINVAL;
	}
	// if the net device is down, refuse the skb
	if (!(net_dev->flags & IFF_UP)) {
		dbg_prx (0, "not up net_dev->flags: %x", net_dev->flags);
		netproto_rx_dropped (interface);
		return -EINVAL;
	}

	skb->dev = net_dev;
	skb->pkt_type = PACKET_HOST;
	skb->protocol = eth_type_trans (skb, net_dev);
	skb->ip_summed = CHECKSUM_UNNECESSARY;

	dbg_prx (5, "skb: %p head: %p data: %p tail: %p end: %p len: %d", skb, skb->head, skb->data,
		 skb->tail, skb->end, skb->len);

	// pass it up to kernel networking layer
	if ((rc = netif_rx (skb))) {
		dbg_prx (0, "netif_rx rc: %d", rc);
	}

	netproto_rx_packets (interface, skb->len);

	{
		extern int net_device_condition;
		if (net_device_condition == NET_DEVICE_CONDITION_RESET)
			net_device_condition = NET_DEVICE_CONDITION_OK;
	}

	return 0;
}

/**
 * netproto_done - transmit done
 * @interface: network interface
 * @skb: network buffer
 * @rc: non-zero indicates failure
 *
 * Call to indicate that a network buffer has been transferred.
 *
 * The network layer will be restarted. 
 */

static __inline__ int netproto_done (int interface, struct sk_buff *skb, int rc)
{
	struct netproto_dev *device;
	struct net_device *net_dev;
	time_t elapsed = 0;

	dbg_ptx (5, "skb: %p", skb);

	if (!skb) {
		return -EINVAL;
	}

	if ((interface >= netproto_devices) || (device = netproto_device_array[interface]) == NULL) {
		dbg_prx (0, "bad interface: %d", interface);
		return -EINVAL;
	}

	if (!(net_dev = device->net_dev)) {
		dbg_prx (0, "bad net_dev: %p", net_dev);
		return -EINVAL;
	}

	if (rc) {
		netproto_tx_dropped (interface);
	}

	{			// lock and update stats
		unsigned long flags;
		write_lock_irqsave (&device->rwlock, flags);

		device->avg_queue_entries += device->queued_entries;
		device->queued_entries--;
		if (skb) {
			device->samples++;
			device->jiffies += jiffies - *(time_t *) (&skb->cb);
			device->queued_bytes -= skb->len;
			elapsed = jiffies - *(time_t *) (&skb->cb);
		}
		write_unlock_irqrestore (&device->rwlock, flags);
	}

	if (skb) {
		dev_kfree_skb_any (skb);
	}

	if (netif_queue_stopped (net_dev)) {
		unsigned long flags;
		netif_wake_queue (net_dev);
		write_lock_irqsave (&device->rwlock, flags);
		device->restarts++;
		write_unlock_irqrestore (&device->rwlock, flags);
	}
	return 0;
}

/* netproto_dev_alloc_skb - allocate an skb suitable for IP with alignment
 * @len:
 *
 * IP headers need to be aligned on 16 byte boundary.
 */
static __inline__ struct sk_buff *netproto_dev_alloc_skb (unsigned int length)
{
	struct sk_buff *skb;
	if ((skb = dev_alloc_skb (length + 2))) {
		skb_reserve (skb, 2);
	}
	dbg_prx (5, "skb: %p len:%d adding 2 for IP alignment", skb, length);
	return skb;
}
