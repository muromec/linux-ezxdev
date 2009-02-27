/*
 * linux/drivers/usbd/net_fd/netproto.c - network prototype library
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

/* netproto
 *
 * This is a simple linux network driver library that is suitable for
 * implementing layered network drivers. 
 *
 * E.g. networking over adsl or usb.
 *
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

#include "../usbd-debug.h"

#ifndef MODULE
#undef GET_USE_COUNT
#undef THIS_MODULE
#define GET_USE_COUNT(foo) 0
#define THIS_MODULE 0
#endif

/* Debug switches ****************************************************************************** */

int dbgflg_usbdfd_pinit = 0;
int dbgflg_usbdfd_poc = 0;
int dbgflg_usbdfd_prx = 0;
int dbgflg_usbdfd_ptx = 0;
int dbgflg_usbdfd_pmgmt = 0;

static debug_option dbg_table[] = {
	{&dbgflg_usbdfd_pinit, NULL, "init", "initialization/termination handling"},
	{&dbgflg_usbdfd_poc, NULL, "oc", "open/close handling"},
	{&dbgflg_usbdfd_prx, NULL, "rx", "receive (from host)"},
	{&dbgflg_usbdfd_ptx, NULL, "tx", "transmit (to host)"},
	{&dbgflg_usbdfd_pmgmt, NULL, "mgmt", "management (ioctl) handling"},
	{NULL, NULL, NULL, NULL}
};

#define dbg_pinit(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_pinit,lvl,fmt,##args)
#define dbg_poc(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_poc,lvl,fmt,##args)
#define dbg_prx(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_prx,lvl,fmt,##args)
#define dbg_ptx(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_ptx,lvl,fmt,##args)
#define dbg_pmgmt(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_pmgmt,lvl,fmt,##args)

debug_option *netproto_get_dbg_table (void)
{
	return (dbg_table);
}


// This needs to be after the dbg_* macros because of inlines....
#include "netproto.h"


char *netproto_procname;	// name to use for proc filesystem
int netproto_devices;		// maximum number of interfaces that can be created
struct netproto_dev **netproto_device_array;	// pointer to array of pointers to netproto dev structures
rwlock_t netproto_rwlock = RW_LOCK_UNLOCKED;	// lock for netproto device array access


/* Network Device support functions *************************************************************** 
 *
 * net_dev.get_stats = netproto_get_stats;
 * net_dev.set_mac_address = netproto_set_mac_addr 
 * net_dev.hard_start_xmit = netproto_start_xmit 
 * net_dev.do_ioctl = netproto_do_ioctl 
 * net_dev.set_multicast = netproto_set_multicast   // 
 * net_dev.open = netproto_open                     // tell management thread to start ATM connection 
 * net_dev.stop = netproto_stop                     // tell management thread to close ATM connection 
 * net_dev.tx_timeout = netproto_tx_timeout         // tell the driver that tx has timed out
 * net_dev.set_config = netproto_set_config         // set ATM configuration details
 */

/* *
 * netproto_init - network device open function 
 * @net_dev: pointer to &struct net_device 
 *
 * Initialize the device. 
 */
static int netproto_init (struct net_device *net_dev)
{
	dbg_pinit (1, "%s", net_dev->name);
	//if (netif_running(net_dev)) {
	//    return -EBUSY;
	//}

	return 0;
}

/* *
 * netproto_uninit - network device uninit function 
 * @net_dev: pointer to &struct net_device 
 *
 * UnInitialize the device. 
 */
static void netproto_uninit (struct net_device *net_dev)
{
	dbg_pinit (1, "%s", net_dev->name);
	return;
}

/* *
 * netproto_open - network device open function 
 * @net_dev: pointer to &struct net_device 
 *
 * Start the device. 
 */
static int netproto_open (struct net_device *net_dev)
{
	//struct netproto_dev *dev = net_dev->priv;

	dbg_poc (1, "%s", net_dev->name);

	netif_wake_queue (net_dev);

	dbg_poc (2, "OK %s", net_dev->name);

	return 0;
}


/* *
 * netproto_stop - network device stop function
 * @net_dev: pointer to &struct net_device 
 *
 * Stop the device. 
 */
static int netproto_stop (struct net_device *net_dev)
{
	//struct netproto_dev *device = net_dev->priv;

	dbg_poc (1, "%s", net_dev->name);
	netif_stop_queue (net_dev);

	return 0;
}


/* *
 * netproto_get_stats - network device get stats function
 * @net_dev: pointer to &struct net_device 
 *
 * Retreive network device stats structure.
 */
static struct net_device_stats *netproto_get_stats (struct net_device *net_dev)
{
	struct netproto_dev *device = net_dev->priv;
	return &device->stats;
}


#if 0
/* *
 * netproto_set_multicast -
 * @net_dev: pointer to &struct net_device 
 *
 * Set device to match multicast status
 */
static void netproto_set_multicast (struct net_device *net_dev)
{
	//struct netproto_dev *device = net_dev->priv;

	if (net_dev->flags & IFF_PROMISC) {
		// enable promiscuous mode
	}
#if 0
	else if ((net_dev->flags & IFF_ALLMULTI) || (net_dev->mc_count > HW_MAX_ADDRS)) {
		// disable promiscous mode, set filter
	} else if (net_dev->mc_count) {
		// walk address list and load filters
	}
#endif
	else {
		// disable promiscuous mode
	}
	return;
}
#endif

/* *
 * netproto_set_mac_addr - network device set mac address function
 * @net_dev: pointer to &struct net_device 
 * @p: pointer to sockaddr structure
 *
 * Set mac address.
 */
static int netproto_set_mac_addr (struct net_device *net_dev, void *p)
{
	struct netproto_dev *device = net_dev->priv;
	struct sockaddr *addr = p;

	if (netif_running (net_dev)) {
		return -EBUSY;
	}

	{
		unsigned long flags;
		write_lock_irqsave (&device->rwlock, flags);
		memcpy (net_dev->dev_addr, addr->sa_data, net_dev->addr_len);
		device->addr_set = 1;
		write_unlock_irqrestore (&device->rwlock, flags);
	}

	// push down
	if (device->set_addr) {
		device->set_addr (device->interface, p, ETH_ALEN);
	}

	return 0;
}

/* *
 * netproto_tx_timeout - network device transmit timeout function
 * @net_dev: pointer to &struct net_device 
 *
 * Called by network layer to tell the driver that transmit has timed out.
 */
static void netproto_tx_timeout (struct net_device *net_dev)
{
	struct netproto_dev *device = net_dev->priv;

	dbg_ptx (7, "");

	if (device->tx_timeout) {
		device->tx_timeout (device->interface);
	}
}


/* *
 * netproto_set_config - network device set config function
 * @net_dev: pointer to &struct net_device 
 * @map: pointer to &struct ifmap 
 *
 * Only allow things to be set if not running.
 */
static int netproto_set_config (struct net_device *net_dev, struct ifmap *map)
{
	if (netif_running (net_dev)) {
		return -EBUSY;
	}
	if (map->mem_start) {
		net_dev->mem_start = map->mem_start;
	}
	if (map->irq) {
		net_dev->irq = map->irq;
	}
	if (map->base_addr) {
		net_dev->base_addr = map->base_addr;
	}
	// XXX copy to device
	return 0;
}


/* *
 * netproto_change_mtu - network device set config function
 * @net_dev: pointer to &struct net_device 
 * @mtu: max transmission unit
 *
 * Set MTU, if running we can only change it to something less
 * than or equal to MTU when PVC opened.
 */
static int netproto_change_mtu (struct net_device *net_dev, int mtu)
{
	struct netproto_dev *dev = net_dev->priv;

	if (netif_running (net_dev)) {
		if (mtu > dev->mtu) {
			return -EBUSY;
		}
		net_dev->mtu = mtu;
		return 0;
	}
	net_dev->mtu = mtu;
	// XXX copy to device
	return 0;
}


/* *
 * netproto_start_xmit - network device start xmit function
 * @skb: pointer to &struct sk_buff to send
 * @net_dev: pointer to &struct net_device 
 *
 * Called by kernel network layer to transmit a frame on this interface,
 * grab locks and pass to netproto_do_xmit().
 *
 * The network layer flow control is managed to prevent more than 
 * device->max_queue_entries from being outstanding. See also 
 * netproto_done().
 * 
 * 
 */
static int netproto_start_xmit (struct sk_buff *skb, struct net_device *net_dev)
{
	struct netproto_dev *device = net_dev->priv;
	int len = skb->len;
	int rc;

	dbg_ptx (7, "");
	dbg_ptx (7, "skb: %p", skb);

	if (!netif_carrier_ok (net_dev)) {
		dbg_ptx (0, "no carrier %p", skb);
		dev_kfree_skb_any (skb);
		//netif_stop_queue(net_dev);
		return 0;
	}
	// stop queue, it will be restart only when we are ready for another skb
	netif_stop_queue (net_dev);

	// lock and update some stats
	{
		unsigned long flags;
		write_lock_irqsave (&device->rwlock, flags);
		device->stopped++;
		device->queued_entries++;
		device->queued_bytes += skb->len;
		write_unlock_irqrestore (&device->rwlock, flags);
	}

	// Set the timestamp for tx timeout
	net_dev->trans_start = jiffies;

	// XXX should we use skb->stamp here
	*(time_t *) (&skb->cb) = jiffies;

#if 0
	dbg_ptx (7, "skb: %p head: %p data: %p tail: %p len: %d",
		 skb, skb->head, skb->data, skb->tail, skb->len);
	dbgPRINTmem (dbgflg_usbdfd_ptx, 7, skb->data, skb->len);
#endif
	// push it down to next layer, handle results
	if ((rc = device->xmit_skb (device->interface, skb))) {

		switch (rc) {

		case -EINVAL:
		case -EUNATCH:
			dbg_ptx (0, "not attached, send failed: %d", rc);
			netproto_tx_carrier_errors (device->interface);
			netif_wake_queue (net_dev);
			break;

		case -ENOMEM:
			dbg_ptx (0, "no mem, send failed: %d", rc);
			netproto_tx_fifo_errors (device->interface);
			netif_wake_queue (net_dev);
			break;

		case -ECOMM:
			dbg_ptx (0, "comm failure, send failed: %d %p", rc, net_dev);
			netproto_tx_dropped (device->interface);
			break;

		}
		dev_kfree_skb_any (skb);
	} else {

		// possibly restart queue
		netproto_tx_packets (device->interface, len);

		{
			read_lock (&device->rwlock);
			if ((device->queued_entries < device->max_queue_entries)
			    && (device->queued_bytes < device->max_queue_bytes)) {
				read_unlock (&device->rwlock);
				netif_wake_queue (net_dev);
			}
			read_unlock (&device->rwlock);
		}
	}
	return 0;
}




/* Ioctl support functions ************************************************************************ 
 */


/* *
* netproto_do_ioctl - network device do_ioctl function
* @net_dev: pointer to &struct net_device 
* @rp: pointer to &struct ifreq
* @cmd: ioctl cmd
* 
* This handles ioctls actually performed on our device - we must return
* -ENOIOCTLCMD for any unrecognized ioctl
 */
static int netproto_do_ioctl (struct net_device *net_dev, struct ifreq *rp, int cmd)
{
	//int             rc;

	dbg_pmgmt (1, "%s cmd=%d", net_dev->name, cmd);

	return -ENOIOCTLCMD;
}


/* Proc Filesystem support ********************************************************************* */


/* *
* netproto_proc_read - implement proc file system read.
* @file: file
* @buf: buffer
* @count: character count
* @pos: file position
*
* Standard proc file system read function.
*
* We let upper layers iterate for us, *pos will indicate which device to return
* statistics for.
 */
static ssize_t netproto_proc_read (struct file *file, char *buf, size_t count, loff_t * pos)
{
	unsigned long page;
	int len = 0;
	int index;

	struct netproto_dev *dev;
	int i;

	// get a page, max 4095 bytes of data...
	if (!(page = get_free_page (GFP_ATOMIC))) {
		return -ENOMEM;
	}

	len = 0;
	index = (*pos)++;
	if (index == 0) {
		len += sprintf ((char *) page + len,
				"          MAC              ESI   VCC         Skbs Forwarded    "
				"Stopped     Deferred     TimeInQ uS           Avg Q    QueueLen   MaxQ       Conn\n");
		len += sprintf ((char *) page + len,
				"%48slo       hi                           lpkt:cel  hpkt:cel     lo   hi   lo  hi    lo  hi\n",
				"");
	}
	// lock and iterate across devices to get access to N'th device, return it's stats
	read_lock (&netproto_rwlock);

	for (i = 0; i < netproto_devices; i++) {

		dev = netproto_device_array[i];

		if (index-- == 0) {
			// max 4095 bytes of data...
			len +=
			    sprintf ((char *) page + len,
				     "%.16s %2d [%02X:%02X:%02X:%02X:%02X:%02X]",
				     dev->net_dev->name, dev->interface, dev->net_dev->dev_addr[0],
				     dev->net_dev->dev_addr[1], dev->net_dev->dev_addr[2],
				     dev->net_dev->dev_addr[3], dev->net_dev->dev_addr[4],
				     dev->net_dev->dev_addr[5]);

			len += sprintf ((char *) page + len, " %s ", dev->addr_mac ? "y" : "n");

			len += sprintf ((char *) page + len, "[%02d.%02d.%02d]",
					(int) (dev->net_dev->mem_start >> 24) & 0xff,
					(int) (dev->net_dev->mem_start >> 16) & 0xff,
					(int) (dev->net_dev->mem_start) & 0xffff);

			len +=
			    sprintf ((char *) page + len, " [%6d %2d]", dev->stopped,
				     dev->stopped - dev->restarts);
			len +=
			    sprintf ((char *) page + len, " [%4ld]",
				     dev->samples ? ((dev->jiffies * 1000) / dev->samples) : 0);

			len +=
			    sprintf ((char *) page + len, " [%3ld]",
				     dev->samples ? ((dev->avg_queue_entries) / dev->samples) : 0);

			len += sprintf ((char *) page + len, " [%3d]", dev->queued_entries);
			len += sprintf ((char *) page + len, " [%3d]", dev->max_queue_entries);

			len += sprintf ((char *) page + len, "J[%6ld]", dev->jiffies);
			len += sprintf ((char *) page + len, "S[%6ld]", dev->samples);

			len += sprintf ((char *) page + len, "\n");
			break;
		}
	}
	read_unlock (&netproto_rwlock);

	if (len > count) {
		len = -EINVAL;
	}

	else if (len > 0 && copy_to_user (buf, (char *) page, len)) {
		len = -EFAULT;
	}

	free_page (page);
	return len;
}

static struct file_operations netproto_proc_operations = {
	read:netproto_proc_read,
};


/* Library Interface functions ***************************************************************** */


/* *
* netproto_rx_dropped
* @int - network interface
*
 */
void netproto_rx_dropped (int interface)
{
	struct netproto_dev *device;

	if ((interface < netproto_devices) && (device = netproto_device_array[interface])) {
		device->stats.rx_dropped++;
		dbg_pinit (1, "%d %ld %ld", interface,
			   device->stats.rx_dropped, device->stats.rx_errors);
	}
}

/* *
* netproto_rx_length_error
* @int - network interface
*
 */
void netproto_rx_length_error (int interface)
{
	struct netproto_dev *device;

	if ((interface < netproto_devices) && (device = netproto_device_array[interface])) {
		device->stats.rx_length_errors++;
		device->stats.rx_dropped++;
		device->stats.rx_errors++;
		dbg_pinit (1, "%d %ld %ld %ld", interface,
			   device->stats.rx_length_errors, device->stats.rx_dropped,
			   device->stats.rx_errors);
	}
}

/* *
* netproto_rx_over_error
* @int - network interface
*
 */
void netproto_rx_over_error (int interface)
{
	struct netproto_dev *device;

	if ((interface < netproto_devices) && (device = netproto_device_array[interface])) {
		device->stats.rx_over_errors++;
		device->stats.rx_dropped++;
		device->stats.rx_errors++;
		dbg_pinit (1, "%d %ld %ld %ld", interface, device->stats.rx_over_errors,
			   device->stats.rx_dropped, device->stats.rx_errors);
	}
}

/* *
* netproto_rx_crc_error
* @int - network interface
*
 */
void netproto_rx_crc_error (int interface)
{
	struct netproto_dev *device;
	//return;

	if ((interface < netproto_devices) && (device = netproto_device_array[interface])) {
		device->stats.rx_crc_errors++;
		device->stats.rx_dropped++;
		device->stats.rx_errors++;
		dbg_pinit (3, "%d %ld %ld %ld", interface, device->stats.rx_crc_errors,
			   device->stats.rx_dropped, device->stats.rx_errors);
	}
}

/* *
* netproto_rx_frame_error
* @int - network interface
*
 */
void netproto_rx_frame_error (int interface)
{
	struct netproto_dev *device;
	//return;

	if ((interface < netproto_devices) && (device = netproto_device_array[interface])) {
		device->stats.rx_frame_errors++;
		device->stats.rx_dropped++;
		device->stats.rx_errors++;
		dbg_pinit (1, "%d %ld %ld %ld", interface,
			   device->stats.rx_frame_errors, device->stats.rx_dropped,
			   device->stats.rx_errors);
	}
}

/* *
* netproto_rx_fifo_error
* @int - network interface
*
 */
void netproto_rx_fifo_error (int interface)
{
	struct netproto_dev *device;
	//return;

	if ((interface < netproto_devices) && (device = netproto_device_array[interface])) {
		device->stats.rx_fifo_errors++;
		device->stats.rx_dropped++;
		device->stats.rx_errors++;
		dbg_pinit (1, "%d %ld %ld %ld", interface, device->stats.rx_fifo_errors,
			   device->stats.rx_dropped, device->stats.rx_errors);
	}
}

/* *
* netproto_rx_missed_error
* @int - network interface
*
 */
void netproto_rx_missed_error (int interface)
{
	struct netproto_dev *device;
	//return;

	if ((interface < netproto_devices) && (device = netproto_device_array[interface])) {
		device->stats.rx_missed_errors++;
		device->stats.rx_dropped++;
		device->stats.rx_errors++;
		dbg_pinit (1, "%d %ld %ld %ld", interface,
			   device->stats.rx_missed_errors, device->stats.rx_dropped,
			   device->stats.rx_errors);
	}
}


/* *
* netproto_rx_packets
* @int - network interface
*
 */
void netproto_rx_packets (int interface, int len)
{
	struct netproto_dev *device;
	//return;

	if ((interface < netproto_devices) && (device = netproto_device_array[interface])) {
		device->stats.rx_packets++;
		device->stats.rx_bytes += len;
	}
}


/* *
* netproto_tx_dropped
* @int - network interface
*
 */
void netproto_tx_dropped (int interface)
{
	struct netproto_dev *device;
	//return;

	if ((interface < netproto_devices) && (device = netproto_device_array[interface])) {
		device->stats.tx_dropped++;
	}
}

/* *
* netproto_tx_carrier_error
* @int - network interface
*
 */
void netproto_tx_carrier_errors (int interface)
{
	struct netproto_dev *device;
	//return;

	if ((interface < netproto_devices) && (device = netproto_device_array[interface])) {
		device->stats.tx_errors++;
		device->stats.tx_carrier_errors++;
	}
}

/* *
* netproto_tx_fifo_error
* @int - network interface
*
 */
void netproto_tx_fifo_errors (int interface)
{
	struct netproto_dev *device;
	//return;

	if ((interface < netproto_devices) && (device = netproto_device_array[interface])) {
		device->stats.tx_errors++;
		device->stats.tx_fifo_errors++;
	}
}

/* *
* netproto_tx_packets
* @int - network interface
*
 */
void netproto_tx_packets (int interface, int len)
{
	struct netproto_dev *device;
	//return;

	if ((interface < netproto_devices) && (device = netproto_device_array[interface])) {
		device->stats.tx_packets++;
		device->stats.tx_bytes += len;
	}
}

/* *
* netproto_name
* @int - network interface
*
 */
char *netproto_name (int interface)
{
	struct netproto_dev *device;
	if ((interface < netproto_devices) && (device = netproto_device_array[interface]) && (device->net_dev)) {
		return device->net_dev->name;
	}
	return NULL;
}

/* *
* netproto_mtu
* @int - network interface
*
 */
int netproto_mtu (int interface)
{
	struct netproto_dev *device;
	if (0 <= interface && (interface < netproto_devices) &&
            (device = netproto_device_array[interface]) && (device->net_dev)) {
		return device->net_dev->mtu;
	}
	return 0;
}

/* *
* netproto_on
* @int - network interface
*
 */
void netproto_on (int interface)
{
	struct netproto_dev *device;
	dbg_pinit (1, "%d", interface);
	if ((interface < netproto_devices) && (device = netproto_device_array[interface]) && (device->net_dev)) {
                dbg_pinit (1, "calling netif_carrier_on" );
		netif_carrier_on (device->net_dev);
		netif_wake_queue (device->net_dev);
	}
}

/* *
* netproto_off
* @int - network interface
*
 */
void netproto_off (int interface)
{
	struct netproto_dev *device;
	dbg_pinit (1, "%d", interface);
	if ((interface < netproto_devices) && (device = netproto_device_array[interface]) && (device->net_dev)) {
		netif_stop_queue (device->net_dev);
		netif_carrier_off (device->net_dev);
	}
}



/**
* netproto_modinit - initialize netproto library
* @name: name for proc file system
* @num: number of interfaces to allow
*
* Initialize the netproto library setting the maximum number of network
* interfaces that may be created.
 */

int netproto_modinit (char *name, int maximum_interfaces)
{
	struct proc_dir_entry *p;
	struct netproto_dev **nda;	// pointer to array of pointers to netproto dev structures

	dbg_pinit (1, "%s maximum_interfaces: %d", name, maximum_interfaces);

	if (!name || !strlen (name)) {
		dbg_pinit (0, "name null or zero length");
		return -EINVAL;
	}
	// verify that netproto_devices not previously set, then allocate array and set it

	if ((nda =
	     kmalloc (sizeof (struct netproto_dev *) * maximum_interfaces, GFP_ATOMIC)) == NULL) {
		dbg_pinit (0, "kmalloc failed");
		return -EINVAL;
	}
	memset (nda, 0, sizeof (struct netproto_dev *) * maximum_interfaces);

	{
		unsigned long flags;
		write_lock_irqsave (&netproto_rwlock, flags);
		if (netproto_devices) {
			write_unlock_irqrestore (&netproto_rwlock, flags);
			kfree (nda);
			dbg_pinit (0, "netproto_devices already set");
			return -EINVAL;
		}
		netproto_device_array = nda;
		netproto_devices = maximum_interfaces;
		write_unlock_irqrestore (&netproto_rwlock, flags);
	}

	return 0;

	// create proc filesystem entry
	if ((p = create_proc_entry (name, 0, NULL)) == NULL)
		return -ENOMEM;
	p->proc_fops = &netproto_proc_operations;

	return 0;
}



/**
* netproto_create - create network interface
* @name: name of interface (use " " to get default "ethN")
* @xmit_skb: call back to transmit skb
* @set_addr: callback to set mac address
* @addr: - pointer to mac address buffer
* @addrlen: - length of address buffer
* @mtu: length of mac address buffer
* @max_queue_entries: max number of outstanding skbs allow
* @max_queue_bytes: max number of outstanding bytes allow
* @flags:
*
* Create a network interface, providing an xmit skb function and an
* optional mac address.
*
* A returned value greater or equal to zero indicates success and can be
* used with subsequent function calls to indicate the created network
* interface.
 */

int netproto_create (char *name,
		     int (*xmit_skb) (int, struct sk_buff *),
		     int (*set_addr) (int, void *, int),
		     int (*tx_timeout) (int),
		     void *addr,
		     int addrlen, int mtu, int max_queue_entries, int max_queue_bytes, int flags)
{
	struct netproto_dev *device;
	struct net_device *net_dev;
	int i;
	unsigned char *cp = addr;

	dbg_pinit (1, "name: %s", name);
	dbg_pinit (1, "addr: %x:%x:%x:%x:%x:%x len: %d",
		   cp[0], cp[1], cp[2], cp[3], cp[4], cp[5], addrlen);

	if (!name || !strlen (name) || !xmit_skb) {
		dbg_pinit (0, "invalid args");
		return -EINVAL;
	}

	if ((net_dev = kmalloc (sizeof (struct net_device), GFP_ATOMIC)) == NULL) {
		dbg_pinit (0, "name: %s kmalloc failed", name);
		return -ENOMEM;
	}
	if ((device = kmalloc (sizeof (struct netproto_dev), GFP_ATOMIC)) == NULL) {
		dbg_pinit (0, "name: %s kmalloc failed", name);
		kfree (net_dev);
		return -ENOMEM;
	}
	dbg_pinit (1, "kmalloc device: %p net_dev: %p", device, net_dev);

	memset (net_dev, 0, sizeof (struct net_device));
	memset (device, 0, sizeof (struct netproto_dev));

	device->net_dev = net_dev;
	device->ops_in_progress = 0;
	device->cmd_queue_count = 0;
	device->state = 0;

	device->xmit_skb = xmit_skb;
	device->set_addr = set_addr;
	device->tx_timeout = tx_timeout;

	// trim name to less then net_dev.name
	if ((i = strlen (name)) >= sizeof (net_dev->name)) {
		i = sizeof (net_dev->name) - 2;
	}
	memcpy (net_dev->name, name, i);	// XXX 

	ether_setup (net_dev);

	if (addrlen && addrlen <= ETH_ALEN) {
		memcpy (&(net_dev->dev_addr), addr, addrlen);
		device->mac_was_set = 1;
	}
	// set net_dev 
	net_dev->priv = device;
	net_dev->do_ioctl = netproto_do_ioctl;
	net_dev->set_config = netproto_set_config;
	net_dev->set_mac_address = netproto_set_mac_addr;
	net_dev->hard_start_xmit = netproto_start_xmit;
	net_dev->get_stats = netproto_get_stats;
	//net_dev->set_multicast = netproto_set_multicast;
	net_dev->change_mtu = netproto_change_mtu;
	net_dev->init = netproto_init;
	net_dev->uninit = netproto_uninit;
	net_dev->open = netproto_open;
	net_dev->stop = netproto_stop;

	net_dev->tx_timeout = netproto_tx_timeout;

	device->mtu = mtu ? mtu : net_dev->mtu;

	device->max_queue_entries = max_queue_entries ? max_queue_entries : 1;
	device->max_queue_bytes = max_queue_bytes ? max_queue_bytes : 2000;

	// get global lock, 
	// derive device number by seeing how many devices we already have
	{
		unsigned long flags;
		write_lock_irqsave (&netproto_rwlock, flags);
		for (i = 0; i < netproto_devices; i++) {
			if (netproto_device_array[i] == NULL) {
				break;
			}
		}
		if (i == netproto_devices) {
			dbg_pinit (0, "name: %s cannot find netproto_devices", name);
			write_unlock_irqrestore (&netproto_rwlock, flags);
			dbg_pinit (2, "kfree device: %p", device);
			kfree (net_dev);	///// QQQ
			kfree (device);	///// QQQ
			return -ENOMEM;
		}
		device->interface = i;
		netproto_device_array[i] = device;
		write_unlock_irqrestore (&netproto_rwlock, flags);

	}

	if (!register_netdev (net_dev)) {
		device->state |= NP_REGISTERED | NP_ENABLED;
		dbg_pinit (1, "REGISTERED DEVICE->STATE: #%x", device->state);
	} else {
		dbg_pinit (0, "REGISTER_NETDEV FAILED");
	}

	// set state to inactive and request to desired value
	//netif_device_detach(net_dev);

	//netif_carrier_off(net_dev);

	//netif_stop_queue(net_dev);

	return i;
}



/**
* netproto_control - control network interface
* @interface: network interface
* @operation: operation
*
* Control a network interface, 
 */

int netproto_control (int interface, int operation)
{
	dbg_pmgmt (1, "interface: %d operation:%d", interface, operation);

	return 0;
}


/**
* netproto_destroy - destroy a network interface
* @interface: network interface
*
* Call to tear down a previously created network interface
 */

int netproto_destroy (int interface)
{
	struct netproto_dev *device;
	struct net_device *net_dev;

	dbg_pinit (1, "interface: %d", interface);

	{
		unsigned long flags;
		write_lock_irqsave (&netproto_rwlock, flags);
		if ((interface >= netproto_devices) || !(device = netproto_device_array[interface])) {
			write_lock_irqsave (&netproto_rwlock, flags);
			dbg_pinit (0, "interface: %d netproto_device_array bad", interface);
			return 0;
		}
		netproto_device_array[interface] = NULL;
		write_lock_irqsave (&netproto_rwlock, flags);

	}

	net_dev = device->net_dev;
	device->net_dev = NULL;

	dbg_pinit (1, "state: %x", device->state);

	device->state &= ~NP_ENABLED;

	if ((device->state & NP_REGISTERED) && net_dev) {
		device->state &= ~NP_REGISTERED;
		dbg_pinit (1, "unregistering: %p", net_dev);
		unregister_netdev (net_dev);
		kfree (net_dev);
	}

	kfree (device);

	return 0;
}


/**
* netproto_modexit
*
* Call to unload the library.
 */

int netproto_modexit (void)
{
	int devices;
	struct netproto_dev **device_array;

	dbg_pinit (1, "");

	if (netproto_devices) {
		return 0;
	}
	// remove proc filesystem entry
	//remove_proc_entry(netproto_procname, NULL);
	netproto_procname = NULL;

	// XXX need to lock here

	devices = netproto_devices;
	netproto_devices = 0;

	//for (i = 0; i < devices; i++) {
	//    netproto_destroy(i);
	//}

	{
		unsigned long flags;
		write_lock_irqsave (&netproto_rwlock, flags);
		device_array = netproto_device_array;
		netproto_device_array = NULL;
		write_lock_irqsave (&netproto_rwlock, flags);
	}
	kfree (netproto_device_array);

	return 0;
}
