/*
 * usbd/network_fd/network.c - Network Function Driver
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
 *
 * This network function driver intended to interoperate with 
 * Belcarra's USBLAN Class drivers. 
 *
 * These are available for Windows, Linux and Mac OSX. For more 
 * information and to download a copy for testing:
 *
 *      http://www.belcarra.com/usblan/
 *
 *
 * Notes
 *
 * 1. If compiled into the kernel this driver can be used with NFSROOT to
 * provide the ROOT filesystem. Please note that the kernel NFSROOT support
 * (circa 2.4.20) can have problems if there are multiple interfaces. So 
 * it is best to ensure that there are no other network interfaces compiled
 * in.
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
#include <linux/kernel.h>
#include <linux/version.h>

#include <usbd-export.h>
#include <usbd-build.h>

MODULE_AUTHOR ("sl@belcarra.com, balden@belcarra.com");


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,17)
MODULE_LICENSE("GPL");
#endif

MODULE_DESCRIPTION ("USB Network Function");


#include <linux/init.h>
#include <linux/list.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/etherdevice.h>
#include <net/arp.h>
#include <linux/rtnetlink.h>
#include <linux/smp_lock.h>
#include <linux/ctype.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/atmdev.h>
#include <linux/pkt_sched.h>
#include <linux/random.h>
#include <linux/utsname.h>

#include <linux/ip.h>
#include <linux/if_ether.h>
#include <linux/in.h>
#include <linux/inetdevice.h>

#include <linux/kmod.h>

#include <asm/uaccess.h>
#include <asm/system.h>

#include <usbd-chap9.h>
#include <usbd-mem.h>
#include <usbd.h>
#include <usbd-func.h>

#if 1//def CONFIG_ARCH_EZX_BARBADOS
#include <linux/ezxusbd.h>
#endif

USBD_MODULE_INFO ("network_fd 2.0-beta");

#include "network.h"
#ifdef CONFIG_USBD_NETWORK_BLAN_FERMAT
#include "fermat.h"
#endif

#if defined(CONFIG_USBD_NETWORK_CDC)

static u32 cdc = 0;
MODULE_PARM (cdc, "i");
MODULE_PARM_DESC (cdc, "Enable CDC mode");
void cdc_init(struct usb_function_instance *);

extern struct usb_function_driver cdc_function_driver;

#if defined(CONFIG_USBD_NETWORK_NETWORK_CDC_MYDHCPD)
extern u_int8_t * checkfordhcp(struct net_device *net_device, u_int8_t *, int, int* );
#endif

#endif

#ifdef CONFIG_USBD_NETWORK_BASIC
static u32 basic = 0;
MODULE_PARM (basic, "i");
MODULE_PARM_DESC (basic, "Enable BASIC mode");
void basic_init(struct usb_function_instance *);
extern struct usb_function_driver basic_function_driver;
#endif

#ifdef CONFIG_USBD_NETWORK_BASIC2
static u32 basic2 = 0;
MODULE_PARM (basic, "i");
MODULE_PARM_DESC (basic2, "Enable BASIC2 mode");
void basic2_init(struct usb_function_instance *);
extern struct usb_function_driver basic2_function_driver;
#endif


#ifdef CONFIG_USBD_NETWORK_SAFE
static u32 safe = 0;
MODULE_PARM (safe, "i");
MODULE_PARM_DESC (safe, "Enable SAFE mode");
void safe_init(struct usb_function_instance *);
extern struct usb_function_driver safe_function_driver;
#endif

#ifdef CONFIG_USBD_NETWORK_BLAN
static u32 blan = 0;
MODULE_PARM (blan, "i");
MODULE_PARM_DESC (blan, "Enable BLAN mode");
void blan_init(struct usb_function_instance *);
extern struct usb_function_driver blan_function_driver;
#endif

static struct usb_function_driver *function_driver = NULL;


/* Module Parameters ************************************************************************* */

wait_queue_head_t usb_netif_wq;

#ifdef CONFIG_ARCH_EZX
//#define CONFIG_USBD_NET_NFS_SUPPORT
//#ifdef CONFIG_USBD_NET_NFS_SUPPORT
int usb_is_configured;
int motusbd_nfs_flag = 0;
//#endif	// CONFIG_USBD_NET_NFS_SUPPORT
#endif	// CONFIG_ARCH_EZX

#define CONFIG_USBD_NETWORK_ALLOW_SETID           1



#ifdef CONFIG_USBD_NETWORK_ALLOW_SETID
// override vendor ID
static u32 vendor_id;
MODULE_PARM (vendor_id, "i");
MODULE_PARM_DESC (vendor_id, "vendor id");

// override product ID
static u32 product_id;
MODULE_PARM (product_id, "i");
MODULE_PARM_DESC (product_id, "product id");
#endif

// override local mac address
#ifdef CONFIG_USBD_NETWORK_LOCAL_MACADDR
static char *local_mac_address_str = CONFIG_USBD_NETWORK_LOCAL_MACADDR;
#else
static char *local_mac_address_str;
#endif
MODULE_PARM (local_mac_address_str, "s");
MODULE_PARM_DESC (local_mac_address_str, "Local MAC");

// override remote mac address
#ifdef CONFIG_USBD_NETWORK_REMOTE_MACADDR
static char *remote_mac_address_str = CONFIG_USBD_NETWORK_REMOTE_MACADDR;
#else
static char *remote_mac_address_str;
#endif
MODULE_PARM (remote_mac_address_str, "s");
MODULE_PARM_DESC (remote_mac_address_str, "Remote MAC");


#ifdef CONFIG_USBD_NETWORK_EP0TEST
static u32 ep0test;
MODULE_PARM (ep0test, "i");
MODULE_PARM_DESC (ep0test, "Test EP0 String Handling - set to ep0 packetsize [8,16,32,64]");
#endif


static u32 zeroconf = 0;
MODULE_PARM (zeroconf, "i");
MODULE_PARM_DESC (zeroconf, "Use usbz%d for network name");

#define NETWORK_CREATED         0x01
#define NETWORK_REGISTERED      0x02
#define NETWORK_DESTROYING      0x04
#define NETWORK_ENABLED         0x08
#define NETWORK_ATTACHED        0x10
#define NETWORK_OPEN            0x20


#define MCCI_ENABLE_CRC         0x03
#define BELCARRA_GETMAC         0x04

#define BELCARRA_SETTIME        0x04
#define BELCARRA_SETIP          0x05
#define BELCARRA_SETMSK         0x06
#define BELCARRA_SETROUTER      0x07
#define BELCARRA_SETDNS         0x08
#define BELCARRA_PING           0x09
#define BELCARRA_SETFERMAT      0x0a
#define BELCARRA_HOSTNAME       0x0b


#define RFC868_OFFSET_TO_EPOCH  0x83AA7E80      // RFC868 - seconds from midnight on 1 Jan 1900 GMT to Midnight 1 Jan 1970 GMT


__u8 local_dev_addr[ETH_ALEN];

__u8 remote_dev_addr[ETH_ALEN];
static __u8 zeros[ETH_ALEN];

#ifdef CONFIG_ARCH_EZX
__u32 usblan_ip_addr = -1;
#else
static __u32 ip_addr;
#endif

static __u32 router_ip;
static __u32 network_mask;
static __u32 dns_server_ip;

/* Prevent overlapping of bus administrative functions:
 *
 *      network_function_enable
 *      network_function_disable
 *      network_hard_start_xmit
 */
DECLARE_MUTEX(usbd_network_sem);


struct net_device Network_net_device;
struct usb_network_private Usb_network_private;

static void network_send_int_blan(struct usb_function_instance *function, int connected );
static void notification_schedule_bh (void);
static int network_urb_sent_bulk (struct urb *urb, int urb_rc);
static int network_urb_sent_int (struct urb *urb, int urb_rc);


//_________________________________________________________________________________________________

/*
 * Synchronization
 *
 *
 * Notification bottom half
 *   
 *  This is a scheduled task that will send an interrupt notification. Because it
 *  is called from the task scheduler context it needs to verify that the device is
 *  still usable.
 *
 *      static int network_send_int_blan(struct usb_device_instance *, int )
 *      static void notification_bh (void *)
 *      void notification_schedule_bh (void)
 *
 *
 * Netdevice functions
 *
 *   These are called by the Linux network layer. They must be protected by irq locks
 *   if necessary to prevent interruption by IRQ level events.
 *
 *      int network_init (struct net_device *net_device)
 *      void network_uninit (struct net_device *net_device)
 *      int network_open (struct net_device *net_device)
 *      int network_stop (struct net_device *net_device)
 *      struct net_device_stats *network_get_stats (struct net_device *net_device)
 *      int network_set_mac_addr (struct net_device *net_device, void *p)
 *      void network_tx_timeout (struct net_device *net_device)
 *      int network_set_config (struct net_device *net_device, struct ifmap *map)
 *      int network_stop (struct net_device *net_device)
 *      int network_hard_start_xmit (struct sk_buff *skb, struct net_device *net_device)
 *      int network_do_ioctl (struct net_device *net_device, struct ifreq *rp, int cmd)
 *
 *
 * Data bottom half functions
 *
 *  These are called from the bus bottom half handler. 
 *
 *      static int network_recv (struct usb_network_private *, struct net_device *, struct sk_buff *)
 *      int network_recv_urb (struct urb *)
 *      int network_urb_sent (struct urb *, int )
 *      
 *
 * Hotplug bottom half:
 *
 *  This is a scheduled task that will send do a hotplug call. Because it is
 *  called from the task scheduler context it needs to verify that the
 *  device is still usable.
 *
 *      static int hotplug_attach (__u32 ip, __u32 mask, __u32 router, int attach)
 *      static void hotplug_bh (void *data)
 *      void hotplug_schedule_bh (void)
 *
 *
 * Irq level functions:
 *
 *   These are called at interrupt time do process or respond to USB setup
 *   commands. 
 *
 *      int network_recv_setup_irq (struct usb_device_request *)
 *      void network_event_irq (struct usb_function_instance *function, usb_device_event_t event, int data)
 *
 *      
 * Enable and disable functions:
 *
 *      void network_function_enable (struct usb_function_instance *, struct usb_function_instance *)
 *      void network_function_disable (struct usb_function_instance *function)
 *
 *
 * Driver initialization and exit:
 *
 *      static  int network_create (void)
 *      static  void network_destroy (void)
 *
 *      int network_modinit (void)
 *      void network_modexit (void)
 */

//_________________________________________________________________________________________________

/*
 * If the following are defined we implement the crc32_copy routine using
 * Duff's device. This will unroll the copy loop by either 4 or 8. Do not
 * use these without profiling to test if it actually helps on any specific
 * device.
 */
#undef CONFIG_USBD_NETWORK_CRC_DUFF4
#undef CONFIG_USBD_NETWORK_CRC_DUFF8

static __u32 *network_crc32_table;

#define CRC32_INIT   0xffffffff      // Initial FCS value
#define CRC32_GOOD   0xdebb20e3      // Good final FCS value

#define CRC32_POLY   0xedb88320      // Polynomial for table generation
        
#define COMPUTE_FCS(val, c) (((val) >> 8) ^ network_crc32_table[((val) ^ (c)) & 0xff])

//_________________________________________________________________________________________________
//                                      crc32_copy

/**
 * Generate the crc32 table
 *
 * return non-zero if malloc fails
 */
static int make_crc_table(void)
{
        u32 n;
        RETURN_ZERO_IF(network_crc32_table);
        RETURN_ENOMEM_IF(!(network_crc32_table = (u32 *)ckmalloc(256*4, GFP_KERNEL)));
        for (n = 0; n < 256; n++) {
                int k;
                u32 c = n;
                for (k = 0; k < 8; k++) {
                        c =  (c & 1) ? (CRC32_POLY ^ (c >> 1)) : (c >> 1);
                }
                network_crc32_table[n] = c;
        }
        return 0;
}

#if !defined(CONFIG_USBD_NETWORK_CRC_DUFF4) && !defined(CONFIG_USBD_NETWORK_CRC_DUFF8)
/**
 * Copies a specified number of bytes, computing the 32-bit CRC FCS as it does so.
 *
 * dst   Pointer to the destination memory area.
 * src   Pointer to the source memory area.
 * len   Number of bytes to copy.
 * val   Starting value for the CRC FCS.
 *
 * Returns      Final value of the CRC FCS.
 *
 * @sa crc32_pad
 */
static __u32 __inline__ crc32_copy (__u8 *dst, __u8 *src, int len, __u32 val)
{
        for (; len-- > 0; val = COMPUTE_FCS (val, *dst++ = *src++));
        return val;
}

#else /* DUFFn */

/**
 * Copies a specified number of bytes, computing the 32-bit CRC FCS as it does so.
 *
 * dst   Pointer to the destination memory area.
 * src   Pointer to the source memory area.
 * len   Number of bytes to copy.
 * val   Starting value for the CRC FCS.
 *
 * Returns      Final value of the CRC FCS.
 *
 * @sa crc32_pad
 */
static __u32 crc32_copy (__u8 *dst, __u8 *src, int len, __u32 val)
{
#if defined(CONFIG_USBD_NETWORK_CRC_DUFF8)
        int n = (len + 7) / 8;
        switch (len % 8) 
#elif defined(CONFIG_USBD_NETWORK_CRC_DUFF4)
                int n = (len + 3) / 4;
        switch (len % 4) 
#endif
        {
        case 0: do {
                        val = COMPUTE_FCS (val, *dst++ = *src++);
#if defined(CONFIG_USBD_NETWORK_CRC_DUFF8)
                case 7:
                        val = COMPUTE_FCS (val, *dst++ = *src++);
                case 6:
                        val = COMPUTE_FCS (val, *dst++ = *src++);
                case 5:
                        val = COMPUTE_FCS (val, *dst++ = *src++);
                case 4:
                        val = COMPUTE_FCS (val, *dst++ = *src++);
#endif
                case 3:
                        val = COMPUTE_FCS (val, *dst++ = *src++);
                case 2:
                        val = COMPUTE_FCS (val, *dst++ = *src++);
                case 1:
                        val = COMPUTE_FCS (val, *dst++ = *src++);
                } while (--n > 0);
        }
        return val;
}
#endif /* DUFFn */


//_________________________________________________________________________________________________
//                                      crc32_pad

/* crc32_pad - pad and calculate crc32
 *
 * Returns - CRC FCS
 */
static __u32 __inline__ crc32_pad (__u8 *dst, int len, __u32 val)
{
        for (; len-- > 0; val = COMPUTE_FCS (val, *dst++ = '\0'));
        return val;
}

//_________________________________________________________________________________________________
//                                      network_send_int
//

/* network_send_int_blan - send an interrupt notification response
 *
 * Generates a response urb on the notification (INTERRUPT) endpoint.
 * Return a non-zero result code to STALL the transaction.
 *
 * This is called from either a scheduled task or from the process context
 * that calls network_open() or network_close().
 *
 */
static void network_send_int_blan(struct usb_function_instance *function, int connected )
{
        struct urb *urb;
        struct cdc_notification_descriptor *cdc;
        int rc;
        struct usb_network_private *network_private = &Usb_network_private;

        //printk(KERN_INFO"%s: device: %p\n", __FUNCTION__, device);

        /*
         * This needs to lock out interrupts as network_event_irq can
         * change the NETWORK_ATTACHED status
         */
        unsigned long flags;
        local_irq_save(flags);
        do {
                BREAK_IF(!function);


                BREAK_IF(network_private->network_type != network_blan);
                BREAK_IF(!network_private->have_interrupt);

                BREAK_IF(!(network_private->flags & NETWORK_ATTACHED));

                //printk(KERN_INFO"%s: connected: %d network: %d %d\n", __FUNCTION__, connected, 
                //                network_private->network_type, network_blan);

                BREAK_IF(usbd_bus_status(function) != USBD_OK);

#ifndef CONFIG_ARCH_EZX
                if (network_private->int_urb) {
                        printk(KERN_INFO"%s: int_urb: %p\n", __FUNCTION__, network_private->int_urb);
                        usbd_cancel_urb_irq(network_private->int_urb);
                        network_private->int_urb = NULL;
                }
#endif

                BREAK_IF(!(urb = usbd_alloc_urb (function, INT_IN, 
                                                sizeof(struct cdc_notification_descriptor), network_urb_sent_int)));
                
                urb->actual_length = sizeof(struct cdc_notification_descriptor);
                memset(urb->buffer, 0, sizeof(struct cdc_notification_descriptor));

                cdc = (struct cdc_notification_descriptor *)urb->buffer;

                cdc->bmRequestType = 0xa1;
                cdc->bNotification = 0x00;
                cdc->wValue = connected ? 0x01 : 0x00;
                cdc->wIndex = 0x01; // XXX interface - check that this is correct

#ifndef CONFIG_ARCH_EZX
                network_private->int_urb = urb;
                //printk(KERN_INFO"%s: int_urb: %p\n", __FUNCTION__, urb);
#endif

                BREAK_IF (!(rc = usbd_send_urb (urb)));

                printk(KERN_ERR"%s: usbd_send_urb failed err: %x\n", __FUNCTION__, rc);
                urb->privdata = NULL;

#ifndef CONFIG_ARCH_EZX
                network_private->int_urb = NULL;
#endif

                usbd_dealloc_urb (urb);

        } while(0);
        local_irq_restore(flags);
}


/* notification_bh - Bottom half handler to send a notification status 
 *
 * Send a notification with open/close status
 *
 * It should not be possible for this to be called more than once at a time
 * as it is only called via schedule_task() which protects against a second
 * invocation.
 */
static void notification_bh (void *data)
{
        network_send_int_blan(Usb_network_private.function, Usb_network_private.flags & NETWORK_OPEN);
        MOD_DEC_USE_COUNT;
}

/* notification_schedule_bh - schedule a call for notification_bh
 */
static void notification_schedule_bh (void)
{
        MOD_INC_USE_COUNT;
        if (!schedule_task (&Usb_network_private.notification_bh)) 
                MOD_DEC_USE_COUNT;
}


//______________________________________Network Layer Functions____________________________________

/*
 * In general because the functions are called from an independant layer it is necessary
 * to verify that the device is still ok and to lock interrupts out to prevent in-advertant
 * closures while in progress.
 */

/* network_init -
 *
 * Initializes the specified network device.
 *
 * Returns non-zero for failure.
 */
static int network_init (struct net_device *net_device)
{
        //printk(KERN_INFO"%s:\n", __FUNCTION__);
	return 0;
}


/* network_uninit -
 *
 * Uninitializes the specified network device.
 */
static void network_uninit (struct net_device *net_device)
{
        //printk(KERN_INFO"%s:\n", __FUNCTION__);
	return;
}


/* network_open -
 *
 * Opens the specified network device.
 *
 * Returns non-zero for failure.
 */
static int network_open (struct net_device *net_device)
{
        struct usb_network_private *network_private = &Usb_network_private;

        // XXX should this be before or after the preceeding netif_running() test?
	MOD_INC_USE_COUNT;
        network_private->flags |= NETWORK_OPEN;
	netif_wake_queue (net_device);
        network_send_int_blan(network_private->function, 1);

#ifdef CONFIG_ARCH_EZX
//#ifdef CONFIG_USBD_NET_NFS_SUPPORT
        if ((!usb_is_configured) && (motusbd_nfs_flag)) {
                if (!in_interrupt()) {
                        printk(KERN_ERR"Please replug USB cable and then ifconfig host interface.\n");
                        interruptible_sleep_on(&usb_netif_wq);
                }
                else {
                        printk(KERN_ERR"Wanring! In interrupt\n");
                }
        }
//#endif	// CONFIG_USBD_NET_NFS_SUPPORT
#endif	// CONFIG_ARCH_EZX
	return 0;
}


/* network_stop -
 *
 * Stops the specified network device.
 *
 * Returns non-zero for failure.
 */
static int network_stop (struct net_device *net_device)
{
        struct usb_network_private *network_private = &Usb_network_private;

        //printk(KERN_INFO"%s:\n", __FUNCTION__);

	//netif_stop_queue (net_device);

        network_private->flags &= ~NETWORK_OPEN;
        network_send_int_blan(network_private->function, 0);

	MOD_DEC_USE_COUNT;
	return 0;
}


/* network_get_stats -
 *
 * Gets statistics from the specified network device.
 *
 * Returns pointer to net_device_stats structure with the required information.
 */
static struct net_device_stats *network_get_stats (struct net_device *net_device)
{
        struct usb_network_private *network_private = &Usb_network_private;
	return &network_private->stats;
}


/* network_set_mac_addr
 *
 * Sets the MAC address of the specified network device. Fails if the device is in use.
 *
 * Returns non-zero for failure.
 */
static int network_set_mac_addr (struct net_device *net_device, void *p)
{
	struct sockaddr *addr = p;
        unsigned long flags;

        //printk(KERN_INFO"%s:\n", __FUNCTION__);

	RETURN_EBUSY_IF(netif_running (net_device));
        local_irq_save(flags);
        memcpy (net_device->dev_addr, addr->sa_data, net_device->addr_len);
        local_irq_restore(flags);
	return 0;
}


/* network_tx_timeout -
 *
 * Tells the specified network device that its current transmit attempt has timed out.
 */
static void network_tx_timeout (struct net_device *net_device)
{
        struct usb_network_private *network_private = &Usb_network_private;
#if 0
        //printk(KERN_ERR"%s:\n", __FUNCTION__);
        network_private->stats.tx_errors++;
        network_private->stats.tx_dropped++;
        usbd_cancel_urb_irq (network_private->bus, NULL);
#endif
#if 0
        // XXX attempt to wakeup the host...
        if ((network_private->network_type == network_blan) && (network_private->flags & NETWORK_OPEN)) {
                notification_schedule_bh();
        }
#endif
}


/** network_set_config -
 *
 * Sets the specified network device's configuration. Fails if the device is in use.
 *
 * map           An ifmap structure containing configuration values.
 *                      Those values which are non-zero/non-null update the corresponding fields
 *                      in net_device.
 *
 * Returns non-zero for failure.
 */
static int network_set_config (struct net_device *net_device, struct ifmap *map)
{
	RETURN_EBUSY_IF(netif_running (net_device));
	if (map->base_addr) 
		net_device->base_addr = map->base_addr;
	if (map->mem_start) 
		net_device->mem_start = map->mem_start;
	if (map->irq) 
		net_device->irq = map->irq;
	return 0;
}


/* network_change_mtu -
 *
 * Sets the specified network device's MTU. Fails if the new value is larger and
 * the device is in use.
 *
 * Returns non-zero for failure.
 */
static int network_change_mtu (struct net_device *net_device, int mtu)
{
        struct usb_network_private *network_private = &Usb_network_private;

        //printk(KERN_INFO"%s:\n", __FUNCTION__);

	RETURN_EBUSY_IF(netif_running (net_device));
	RETURN_EBUSY_IF(mtu > network_private->mtu);

	net_device->mtu = mtu;
	return 0;
}

//_________________________________________________________________________________________________
//                                      network_hard_start_xmit

/* network_hard_start_xmit - start sending an skb
 *
 * Starts a network device's transmit function.
 * Called by kernel network layer to transmit a frame on this device.
 * Grab locks and pass to @c netproto_do_xmit().
 * The network layer flow control is managed to prevent more than 
 * @c device->max_queue_entries from being outstanding.
 *
 * skb           Pointer to the sk_buff structure to be sent.
 * net_device    Specifies the device by pointing to its net_device struct.
 *
 * Return non-zero (1) if busy.
 */
static int network_hard_start_xmit (struct sk_buff *skb, struct net_device *net_device)
{
        struct usb_network_private *network_private = &Usb_network_private;
        struct usb_function_instance *function = network_private->function;
	struct urb *urb = NULL;
	int len = skb->len;
	int rc = 1;
        int in_pkt_sz;

        down(&usbd_network_sem);
        do {


#if 0
                printk(KERN_INFO"%s: %s len: %d encap: %d\n", __FUNCTION__, net_device->name, skb->len, network_private->encapsulation);
                printk(KERN_INFO"start_xmit: len: %x head: %p data: %p tail: %p\n", 
                                skb->len, skb->head, skb->data, skb->tail);
                {
                        __u8 *cp = skb->data;
                        int i;
                        for (i = 0; i < skb->len; i++) {
                                if ((i%32) == 0) {
                                        printk("\ntx[%2x] ", i);
                                }
                                printk("%02x ", *cp++);
                        }
                        printk("\n");
                }
#endif

                THROW_IF(!(network_private->flags & NETWORK_ATTACHED), not_ok);
                THROW_IF(!netif_carrier_ok (net_device), not_ok);
                THROW_IF(usbd_bus_status(function) != USBD_OK, not_ok);

#if defined(CONFIG_USBD_NETWORK_CDC)
                // verify interface is enabled - non-zero altsetting means data is enabled
                THROW_IF(!usbd_interface_AltSetting(function, DATA_INTF), not_ok);
#endif
                in_pkt_sz = usbd_endpoint_wMaxPacketSize(function, BULK_IN, usbd_high_speed(function));

                // stop queue, it will be restart only when we are ready for another skb
                netif_stop_queue (net_device);

                // lock and update some stats
                network_private->stopped++;
                network_private->queued_entries++;
                network_private->queued_bytes += skb->len;


                // Set the timestamp for tx timeout
                net_device->trans_start = jiffies;

                switch (network_private->encapsulation) {
                case simple_crc:
                        //printk(KERN_INFO"%s: BASIC_CRC\n", __FUNCTION__);
                        {
                                u32 crc;

                                // allocate urb 5 bytes larger than required
                                if (!(urb = usbd_alloc_urb (function, BULK_IN, 
                                                                skb->len + 5 + in_pkt_sz, network_urb_sent_bulk ))) 
                                {
                                        printk(KERN_ERR"%s: urb alloc failed len: %d endpoint: %02x\n", 
                                                        __FUNCTION__, skb->len, 
                                                        usbd_endpoint_bEndpointAddress(function, BULK_IN, 
                                                                usbd_high_speed(function)));
                                        return -ENOMEM;
                                }

                                // copy and crc skb->len bytes
                                crc = crc32_copy(urb->buffer, skb->data, skb->len, CRC32_INIT);
                                urb->actual_length = skb->len;

                                // add a pad byte if required to ensure a short packet, usbdnet driver
                                // will correctly handle pad byte before or after CRC, but the MCCI driver
                                // wants it before the CRC.
                                if ((urb->actual_length % in_pkt_sz) == (in_pkt_sz - 4)) {
                                        crc = crc32_pad(urb->buffer + urb->actual_length, 1, crc);
                                        urb->actual_length++;
                                }

                                // munge and append crc
                                crc = ~crc;
                                urb->buffer[urb->actual_length++] = crc & 0xff;
                                urb->buffer[urb->actual_length++] = (crc >> 8) & 0xff;
                                urb->buffer[urb->actual_length++] = (crc >> 16) & 0xff;
                                urb->buffer[urb->actual_length++] = (crc >> 24) & 0xff;

                                break;
                        }
                default:
                        break;

                }
                if (!urb) {
                        printk(KERN_ERR"%s: unknown encapsulation\n", __FUNCTION__);
                        rc = -EINVAL;
                        break;
                }

                // save skb for netproto_done
                urb->privdata = (void *) skb;
#if 0
                printk(KERN_INFO"start_xmit: len: %d : %d data: %p\n", skb->len, urb->actual_length, urb->buffer);
                {
                        __u8 *cp = urb->buffer;
                        int i;
                        for (i = 0; i < urb->actual_length; i++) {
                                if ((i%32) == 0) {
                                        printk("\ntx[%2x] ", i);
                                }
                                printk("%02x ", *cp++);
                        }
                        printk("\n");
                }
#endif
#if defined(CONFIG_USBD_NETWORK_BLAN_FERMAT)
                if (network_private->fermat) {
                        fermat_encode(urb->buffer, urb->actual_length);
                }
#endif

                //printk(KERN_INFO"%s: urb: %p\n", __FUNCTION__, urb);
                if ((rc = usbd_send_urb (urb))) {

                        printk(KERN_ERR"%s: FAILED: %d\n", __FUNCTION__, rc);
                        urb->privdata = NULL;
                        usbd_dealloc_urb (urb);

                        switch (rc) {

                        case -EINVAL:
                        case -EUNATCH:
                                printk(KERN_ERR"%s: not attached, send failed: %d\n", __FUNCTION__, rc);
                                network_private->stats.tx_errors++;
                                network_private->stats.tx_carrier_errors++;
                                netif_wake_queue (net_device);
                                break;

                        case -ENOMEM:
                                printk(KERN_ERR"%s: no mem, send failed: %d\n", __FUNCTION__, rc);
                                network_private->stats.tx_errors++;
                                network_private->stats.tx_fifo_errors++;
                                netif_wake_queue (net_device);
                                break;

                        case -ECOMM:
                                printk(KERN_ERR"%s: comm failure, send failed: %d %p\n", __FUNCTION__, rc, net_device);
                                network_private->stats.tx_dropped++;
                                break;

                        }
                        dev_kfree_skb_any (skb);
                        //rc = NET_XMIT_DROP; XXX this is what we should do, blows up on some 2.4.20 kernels
                        rc = 0;
                        break;
                }

                // XXX should we restart network queue
                //printk(KERN_INFO"%s: OK: %d\n", __FUNCTION__, rc);
                network_private->stats.tx_packets++;
                network_private->stats.tx_bytes += len;

                if ((network_private->queued_entries < network_private->max_queue_entries) && 
                                (network_private->queued_bytes < network_private->max_queue_bytes)) 
                        netif_wake_queue (net_device);
                
                CATCH(not_ok) {
                        dev_kfree_skb_any (skb);
                        network_private->stats.tx_dropped++;
                        //netif_stop_queue(net_device); // XXX
                        //rc = NET_XMIT_DROP; XXX this is what we should do, blows up on some 2.4.20 kernels
                        rc = 0;
                        break;
                }

        } while (0);
        up(&usbd_network_sem);
        return rc;
}


/* network_do_ioctl - perform an ioctl call 
 *
 * Carries out IOCTL commands for the specified network device.
 *
 * rp            Points to an ifreq structure containing the IOCTL parameter(s).
 * cmd           The IOCTL command.
 *
 * Returns non-zero for failure.
 */
static int network_do_ioctl (struct net_device *net_device, struct ifreq *rp, int cmd)
{
	return -ENOIOCTLCMD;
}

//_________________________________________________________________________________________________

struct net_device Network_net_device = {
        get_stats: network_get_stats,
        tx_timeout: network_tx_timeout,
        do_ioctl: network_do_ioctl,
        set_config: network_set_config,
        set_mac_address: network_set_mac_addr,
        hard_start_xmit: network_hard_start_xmit,
        change_mtu: network_change_mtu,
        init: network_init,
        uninit: network_uninit,
        open: network_open,
        stop: network_stop,
        priv: &Usb_network_private,
};

//_________________________________________________________________________________________________


/* network_recv - function to process an received data URB
 *
 * Passes received data to the network layer. Passes skb to network layer.
 *
 * Returns non-zero for failure.
 */
static __inline__ int network_recv (struct usb_network_private *network_private, 
                struct net_device *net_device, struct sk_buff *skb)
{
	int rc;
#if 0
        printk(KERN_INFO"%s: len: %x head: %p data: %p tail: %p\n", __FUNCTION__, 
                        skb->len, skb->head, skb->data, skb->tail);
        {
                __u8 *cp = skb->data;
                int i;
                for (i = 0; i < skb->len; i++) {
                        if ((i%32) == 0) {
                                printk("\nrx[%2x] ", i);
                        }
                        printk("%02x ", *cp++);
                }
                printk("\n");
        }
#endif

        // refuse if no device present
        if (!netif_device_present (net_device)) {
                printk(KERN_INFO"%s: device not present\n", __FUNCTION__);
                return -EINVAL;
        }

        // refuse if no carrier
        if (!netif_carrier_ok (net_device)) {
                printk(KERN_INFO"%s: no carrier\n", __FUNCTION__);
		return -EINVAL;
	}

	// refuse if the net device is down
	if (!(net_device->flags & IFF_UP)) {
		//printk(KERN_INFO"%s: not up net_dev->flags: %x\n", __FUNCTION__, net_device->flags);
		network_private->stats.rx_dropped++;
		return -EINVAL;
	}

	skb->dev = net_device;
	skb->pkt_type = PACKET_HOST;
	skb->protocol = eth_type_trans (skb, net_device);
	skb->ip_summed = CHECKSUM_UNNECESSARY;

        //printk(KERN_INFO"%s: len: %x head: %p data: %p tail: %p\n", __FUNCTION__, 
        //                skb->len, skb->head, skb->data, skb->tail);


	// pass it up to kernel networking layer
	if ((rc = netif_rx (skb))) {
		//printk(KERN_INFO"%s: netif_rx rc: %d\n", __FUNCTION__, rc);
	}
        network_private->stats.rx_bytes += skb->len;
        network_private->stats.rx_packets++;

	return 0;
}

//_________________________________________________________________________________________________

/* network_recv_urb - callback to process a received URB
 *
 * Returns non-zero for failure.
 */
static int network_recv_urb (struct urb *urb, int rc)
{
        struct net_device *net_device;
        struct usb_network_private *network_private = &Usb_network_private;
	struct usb_function_instance *function = network_private->function;

	struct sk_buff *skb = NULL;
        int out_pkt_sz;

        RETURN_EINVAL_IF(!(network_private->flags & NETWORK_ATTACHED));
        RETURN_EINVAL_IF(!(net_device = &Network_net_device));

        out_pkt_sz = usbd_endpoint_wMaxPacketSize(function, BULK_OUT, usbd_high_speed(function));

#if 0
        printk(KERN_INFO"%s: urb: %p len: %d maxtransfer: %d encap: %d\n", __FUNCTION__, 
                        urb, urb->actual_length, network_private->maxtransfer, network_private->encapsulation);

        {
                __u8 *cp = urb->buffer;
                int i;
                for (i = 0; i < urb->actual_length; i++) {
                        if ((i%32) == 0) {
                                printk("\n[%2x] ", i);
                        }
                        printk("%02x ", *cp++);
                }
                printk("\n");
        }
#endif

        THROW_IF(urb->status != RECV_OK, error);

        // Is CDC active (we have received CONTROL WRITE setup packets indicating real CDC host)
        switch (network_private->encapsulation) {
                int len;
        case simple_crc:

                len = urb->actual_length;

                // allocate skb of appropriate length, reserve 2 to align ip
                THROW_IF(!(skb = dev_alloc_skb (len + 2)), error);
                skb_reserve(skb, 2);

#if defined(CONFIG_USBD_NETWORK_BLAN_PADAFTER)
                {
                        /* This version simply checks for a correct CRC along the 
                         * entire packet. Some UDC's have trouble with some packet
                         * sizes, this allows us to add pad bytes after the CRC.
                         */

                        u8 *dst = skb_put(skb, len - 1);
                        u8 *src = urb->buffer;
                        int copied;
                        u32 crc;

                        // XXX this should work, but the MIPS optimizer seems to get it wrong....
                        //copied = (len < out_pkt_sz) ? 0 : ((len / out_pkt_sz) - 1) * out_pkt_sz;
                        
                        if (len < out_pkt_sz*2) 
                                copied = 0;
                        else {
                                int pkts = ((len - out_pkt_sz) / out_pkt_sz);
                                copied = (pkts - 1) * out_pkt_sz;
                        }

                        len -= copied;
                        crc = CRC32_INIT;
                        for (; copied-- > 0 ; crc = COMPUTE_FCS (crc, *dst++ = *src++));

                        for (; (len-- > 0) && (CRC32_GOOD != crc); crc = COMPUTE_FCS (crc, *dst++ = *src++));

                        skb_trim(skb, skb->len - len - 4);

                        if (CRC32_GOOD != crc) {
                                //printk(KERN_INFO"%s: AAA frame: %03x\n", __FUNCTION__, urb->framenum);
                                THROW_IF(network_private->crc, crc_error);
                        }
                        else 
                                network_private->crc = 1;
                }
#else
                /* 
                 * The CRC can be sent in two ways when the size of the transfer 
                 * ends up being a multiple of the packetsize:
                 *
                 *                                           |
                 *                <data> <CRC><CRC><CRC><CRC>|<???>     case 1
                 *                <data> <NUL><CRC><CRC><CRC>|<CRC>     case 2
                 *           <data> <NUL><CRC><CRC><CRC><CRC>|          case 3
                 *     <data> <NUL><CRC><CRC><CRC>|<CRC>     |          case 4
                 *                                           |
                 *        
                 * This complicates CRC checking, there are four scenarios:
                 *
                 *      1. length is 1 more than multiple of packetsize with a trailing byte
                 *      2. length is 1 more than multiple of packetsize 
                 *      3. length is multiple of packetsize
                 *      4. none of the above
                 *
                 * Finally, even though we always compute CRC, we do not actually throw
                 * things away until and unless we have previously seen a good CRC.
                 * This allows backwards compatibility with hosts that do not support
                 * adding a CRC to the frame.
                 *
                 */

                // test if 1 more than packetsize multiple
                if (1 == (len % out_pkt_sz)) {

                        // copy and CRC up to the packetsize boundary
                        u32 crc = crc32_copy(skb_put(skb, len - 1), urb->buffer, len - 1, CRC32_INIT);

                        // if the CRC is good then this is case 1
                        if (CRC32_GOOD != crc) {

                                crc = crc32_copy(skb_put(skb, 1), urb->buffer + len - 1, 1, crc);

                                if (CRC32_GOOD != crc) {
                                        //crc_errors[len%64]++;
                                        printk(KERN_INFO"%s: A CRC error %08x %03x\n", __FUNCTION__, crc, urb->framenum);
                                        THROW_IF(network_private->crc, crc_error);
                                }
                                else 
                                        network_private->crc = 1;
                        }
                        else 
                                network_private->crc = 1;
                }
                else {
                        u32 crc = crc32_copy(skb_put(skb, len), urb->buffer, len, CRC32_INIT);

                        if (CRC32_GOOD != crc) {
                                //crc_errors[len%64]++;
                                //printk(KERN_INFO"%s: CCC\n", __FUNCTION__);
                                THROW_IF(network_private->crc, crc_error);
                        }
                        else 
                                network_private->crc = 1;
                }
                // trim IFF we are paying attention to crc
                if (network_private->crc) 
                        skb_trim(skb, skb->len - 4);
#endif

                // pass it up, free skb if non zero
                THROW_IF(network_recv (network_private, net_device, skb), skb_error);
                
                break;
        default:
                break;
        }

        // catch a simple error, just increment missed error and general error
        CATCH(error) {

                network_private->stats.rx_frame_errors++;
                network_private->stats.rx_errors++;

                // catch error where skb may need to be released
                CATCH(skb_error) {

                        // catch a CRC error
                        
                        // XXX We need to track whether we have seen a correct CRC, until then 
                        // we ignore CRC errors.

                        CATCH(crc_error) {
#if 0
                                printk(KERN_INFO"%s: urb: %p status: %d len: %d maxtransfer: %d encap: %d\n", __FUNCTION__, 
                                                urb, urb->status, urb->actual_length, network_private->maxtransfer, 
                                                network_private->encapsulation);

                                {
                                        __u8 *cp = urb->buffer;
                                        int i;
                                        for (i = 0; i < urb->actual_length; i++) {
                                                if ((i%32) == 0) {
                                                        printk("\n[%2x] ", i);
                                                }
                                                printk("%02x ", *cp++);
                                        }
                                        printk("\n");
                                }
#endif
                                network_private->stats.rx_crc_errors++;
                                network_private->stats.rx_errors++;
                        }

                        // catch an overrun error
                        // (Only used if CONFIG_USBD_NETWORK_CDC is defined.)
                        //CATCH(fifo_error) {
                        //        network_private->stats.rx_fifo_errors++;
                        //        network_private->stats.rx_errors++;
                        //}

                        // if skb defined free it
                        if (skb) 
                                dev_kfree_skb_any (skb);
                }
                network_private->stats.rx_dropped++;
                //return -EINVAL;
        }
        //printk(KERN_INFO"%s: restart: %p\n", __FUNCTION__, urb);
        return (usbd_start_recv (urb));
}

//_________________________________________________________________________________________________
//                                      network_urb_sent

/* network_urb_sent_bulk - callback function to process a sent URB
 *
 * Handles notification that an urb has been sent (successfully or otherwise).
 *
 * urb   Pointer to the urb that has been sent.
 * rc    Result code from the send operation.
 *
 * Returns non-zero for failure.
 */
static int network_urb_sent_bulk (struct urb *urb, int urb_rc)
{
        struct sk_buff *skb;
        struct net_device *net_device;
        unsigned long flags;
        struct usb_network_private *network_private = &Usb_network_private;
        struct usb_function_instance *function = network_private->function;
        int rc = -EINVAL;

        //printk(KERN_INFO"%s: urb: %p device: %p address: %x urb_rc: %d\n", __FUNCTION__,
        //                urb, urb->device, urb->endpoint->bEndpointAddress, urb_rc);

        local_irq_save(flags);
        do {

                BREAK_IF(!urb);
                BREAK_IF(!(function = urb->function_instance));

                switch (urb_rc) {
                case SEND_FINISHED_ERROR:
                        network_private->stats.tx_errors++;
                        network_private->stats.tx_dropped++;
                        break;
                case SEND_FINISHED_CANCELLED:
                        network_private->stats.tx_errors++;
                        network_private->stats.tx_carrier_errors++;
                        break;
                default:
                        break;
                }

                // XXX should we zap skb first if error?
                RETURN_EINVAL_IF(!(network_private->flags & NETWORK_CREATED));

                // retrieve skb pointer and unlink from urb pointers
                skb = (struct sk_buff *) urb->privdata;

                urb->privdata = NULL;
                usbd_dealloc_urb (urb);

                // tell netproto we are done with the skb, it will test for NULL
                // netproto_done (interface, skb, urb_rc != SEND_FINISHED_OK);

                BREAK_IF(!skb);
                BREAK_IF(!(net_device = &Network_net_device));


                network_private->avg_queue_entries += network_private->queued_entries;
                network_private->queued_entries--;
                network_private->samples++;
                network_private->jiffies += jiffies - *(time_t *) (&skb->cb);
                network_private->queued_bytes -= skb->len;

                dev_kfree_skb_any (skb);

                if (netif_queue_stopped (net_device)) {
                        netif_wake_queue (net_device);
                        network_private->restarts++;
                }
                rc = 0;

        } while (0);
        local_irq_restore(flags);
        return rc;
}

/* network_urb_sent_int - callback for sent URB
 *
 * Handles notification that an urb has been sent (successfully or otherwise).
 *
 * Returns non-zero for failure.
 */
static int network_urb_sent_int (struct urb *urb, int urb_rc)
{
        struct usb_function_instance *function;
        struct sk_buff *skb;
        struct net_device *net_device;
        unsigned long flags;
        int rc = -EINVAL;
        struct usb_network_private *network_private = &Usb_network_private;

        //printk(KERN_INFO"%s: urb: %p device: %p address: %x urb_rc: %d\n", __FUNCTION__,
        //                urb, urb->device, urb->endpoint->bEndpointAddress, urb_rc);

        local_irq_save(flags);
        do {

                BREAK_IF(!urb);
                BREAK_IF(!(function = urb->function_instance));
                //RETURN_EINVAL_IF(!(network_private->flags & NETWORK_ATTACHED));

                //printk(KERN_INFO"%s: reseting int_urb: %p\n", __FUNCTION__, network_private->int_urb);
                usbd_dealloc_urb (urb);
#ifndef CONFIG_ARCH_EZX
                network_private->int_urb = NULL;
#endif
                rc = 0;

        } while (0);
        local_irq_restore(flags);
        return rc;
}


//_________________________________________________________________________________________________
//                                      network_recv_setup_irq
//
/* network_urb_received_ep0 - callback for sent URB
 *
 * Handles notification that an urb has been sent (successfully or otherwise).
 *
 * Returns non-zero for failure.
 */
static int network_urb_received_ep0 (struct urb *urb, int urb_rc)
{
        printk(KERN_INFO"%s: urb: %p status: %d\n", __FUNCTION__, urb, urb->status);

        RETURN_EINVAL_IF (RECV_OK != urb->status);

        printk(KERN_INFO"%s: %s\n", __FUNCTION__, urb->buffer);

        return -EINVAL;         // caller will de-allocate
}

/* network_recv_setup_irq - process a received SETUP URB
 *
 * Processes a received setup packet and CONTROL WRITE data.
 * Results for a CONTROL READ are placed in urb->buffer.
 *
 * Returns non-zero for failure.
 */
static int network_recv_setup_irq (struct usb_device_request *request)
{
        struct usb_network_private *network_private = &Usb_network_private;
        struct usb_function_instance *function = network_private->function;
        struct urb *urb;


        // Verify that this is a USB Class request per CDC specification or a vendor request.
        RETURN_ZERO_IF (!(request->bmRequestType & (USB_REQ_TYPE_CLASS | USB_REQ_TYPE_VENDOR)));

        // Determine the request direction and process accordingly
        switch (request->bmRequestType & (USB_REQ_DIRECTION_MASK | USB_REQ_TYPE_MASK)) {

        case USB_REQ_HOST2DEVICE | USB_REQ_TYPE_VENDOR:

                switch (request->bRequest) {
                case MCCI_ENABLE_CRC:
                        if (make_crc_table()) 
                                return -EINVAL;
                        network_private->encapsulation = simple_crc;
                        return 0;

                case BELCARRA_PING:
                        //printk(KERN_INFO"%s: H2D VENDOR IP: %08x\n", __FUNCTION__, ip_addr);
                        if ((network_private->network_type == network_blan)) 
                                notification_schedule_bh();
                        break;

#if !defined(CONFIG_USBD_NETWORK_BLAN_DO_NOT_SETTIME) || !defined(CONFIG_USBD_NETWORK_SAFE_DO_NOT_SETTIME) 
                case BELCARRA_SETTIME:
                        {
                                struct timeval tv;
#ifndef CONFIG_ARCH_EZX
#error
                                // wIndex and wLength contain RFC868 time - seconds since midnight 1 jan 1900

                                tv.tv_sec = ntohl( request->wValue << 16 | request->wIndex);
                                tv.tv_usec = 0;

                                // convert to Unix time - seconds since midnight 1 jan 1970
                                
                                tv.tv_sec -= RFC868_OFFSET_TO_EPOCH; 

                                //printk(KERN_INFO"%s: H2D VENDOR TIME: %08x\n", __FUNCTION__, tv.tv_sec);

                                // set the time
                                do_settimeofday(&tv);
#endif
                        } break;
#endif
                case BELCARRA_SETIP:
                        usblan_ip_addr = ntohl( request->wValue << 16 | request->wIndex);
                        break;

                case BELCARRA_SETMSK:
                        network_mask = ntohl( request->wValue << 16 | request->wIndex);
                        break;

                case BELCARRA_SETROUTER:
                        router_ip = ntohl( request->wValue << 16 | request->wIndex);
                        break;

                case BELCARRA_SETDNS:
                        dns_server_ip = ntohl( request->wValue << 16 | request->wIndex);
                        break;
#ifdef CONFIG_USBD_NETWORK_BLAN_FERMAT
                case BELCARRA_SETFERMAT:
                        network_private->fermat = 1;
                        break;
#endif
#ifdef CONFIG_USBD_NETWORK_BLAN_HOSTNAME
                case BELCARRA_HOSTNAME:
                        //printk(KERN_INFO"%s: HOSTNAME\n", __FUNCTION__);
                        RETURN_EINVAL_IF(!(urb = usbd_alloc_urb_ep0(function, le16_to_cpu(request->wLength), 
                                                        network_urb_received_ep0) ));
                        RETURN_ZERO_IF(!usbd_start_recv(urb));                  // return if no error
                        usbd_dealloc_urb(urb);                                  // de-alloc if error
                        return -EINVAL;
#endif
                }
                return 0;
#if 0
        case USB_REQ_DEVICE2HOST | USB_REQ_TYPE_VENDOR:
                urb->actual_length = 0;
                switch (request->bRequest) {
                case BELCARRA_GETMAC:
                        {
                                // copy and free the original buffer
                                memcpy(urb->buffer, Network_net_device.dev_addr, ETH_ALEN);
                                urb->actual_length = ETH_ALEN;
                                return 0;
                        }
                }
#endif
                return 0;
        default:
                break;
        }
        return -EINVAL;
}

//______________________________________ Hotplug Functions ________________________________________

#ifdef CONFIG_HOTPLUG

#define AGENT "network_fd"

/* hotplug_attach - call hotplug 
 */
static int hotplug_attach (__u32 ip, __u32 mask, __u32 router, int attach)
{
        static int count = 0;
        char *argv[3];
        char *envp[10];
        char ifname[20+12 + IFNAMSIZ];
        int i;
        char count_str[20];

        RETURN_EINVAL_IF(!hotplug_path[0]);

        argv[0] = hotplug_path;
        argv[1] = AGENT;
        argv[2] = 0;

        sprintf (ifname, "INTERFACE=%s", Network_net_device.name);
        sprintf (count_str, "COUNT=%d", count++);

        i = 0;
        envp[i++] = "HOME=/";
        envp[i++] = "PATH=/sbin:/bin:/usr/sbin:/usr/bin";
        envp[i++] = ifname;


        if (attach) {
                unsigned char *cp;
                char ip_str[20+32];
                char mask_str[20+32];
                char router_str[20+32];
                __u32 nh;

#ifdef CONFIG_ARCH_EZX
		if(ip == 0)
		{
			// When connecting with Linux Host PC, the IP is set to fixed address: 192.168.1.2
			ip = (192<<24) | (168<<16) | (1<<8) | (2);
			usblan_ip_addr = ip;
		}
#endif
                nh = htonl(ip);
                cp = (unsigned char*) &nh;
                sprintf (ip_str, "IP=%d.%d.%d.%d", cp[0], cp[1], cp[2], cp[3]);

#ifdef BLAN_IP_ADDR_GNPO
		// update the special string descriptor with the IP addr
		usbd_update_string_at_index(STRINDEX_IPADDR, ip_str);
#endif
		
                nh = htonl(mask);
                cp = (unsigned char*) &nh;
                sprintf (mask_str, "MASK=%d.%d.%d.%d", cp[0], cp[1], cp[2], cp[3]);

                nh = htonl(router);
                cp = (unsigned char*) &nh;
                sprintf (router_str, "ROUTER=%d.%d.%d.%d", cp[0], cp[1], cp[2], cp[3]);

                //printk (KERN_INFO "%s: attach %s %s %s\n", __FUNCTION__, ifname, ip_str, count_str);

                envp[i++] = "ACTION=attach";
                envp[i++] = ip_str;
                envp[i++] = mask_str;
                envp[i++] = router_str;
        }
        else {
                //printk (KERN_INFO "%s: detach %s %s\n", __FUNCTION__, ifname, count_str);
                envp[i++] = "ACTION=detach";
		
#ifdef CONFIG_ARCH_EZX
		usblan_ip_addr = 0;
#endif
        }

        envp[i++] = count_str;
        envp[i++] = 0;

        return call_usermodehelper (argv[0], argv, envp);
}


/* hotplug_bh - bottom half handler to call hotplug script to signal ATTACH or DETACH
 *
 * Check connected status and load/unload as appropriate.
 *
 * It should not be possible for this to be called more than once at a time
 * as it is only called via schedule_task() which protects against a second
 * invocation.
 */
static void hotplug_bh (void *data)
{
        struct usb_network_private *network_private = &Usb_network_private;
        struct usb_function_instance *function = network_private->function;

        //printk(KERN_INFO"%s: BUS state: %d status: %d\n", __FUNCTION__, usbd_device_state(function), usbd_bus_status(function));

        if (function && (USBD_OK == usbd_bus_status(function)) && (STATE_CONFIGURED == usbd_device_state(function))) {
                if (hotplug_attached != network_private->hotplug_status) {
                        //printk(KERN_INFO"%s: ATTACH\n", __FUNCTION__);
                        network_private->hotplug_status = hotplug_attached;
                        hotplug_attach (usblan_ip_addr, network_mask, router_ip, 1);
                }
        }
        else {
                if (hotplug_detached != network_private->hotplug_status) {
                        //printk(KERN_INFO"%s: DETACH\n", __FUNCTION__);
                        network_private->hotplug_status = hotplug_detached;
                        hotplug_attach (usblan_ip_addr, network_mask, router_ip, 0);
                }
        }
        MOD_DEC_USE_COUNT;
}

/* hotplug_schedule_bh - schedule a call to hotplug bottom half
 */
static void hotplug_schedule_bh (void)
{
        MOD_INC_USE_COUNT;
        if (!schedule_task (&Usb_network_private.hotplug_bh)) 
                MOD_DEC_USE_COUNT;
}
#endif				/* CONFIG_HOTPLUG */


//_________________________________________________________________________________________________

#ifdef CONFIG_USBD_NETWORK_START_SINGLE
#define NETWORK_START_URBS 1
#else
#define NETWORK_START_URBS 2
#endif

/* network_start_recv - start recv urb(s)
 */
void network_start_recv(struct usb_function_instance *function)
{
        int i;
        for (i = 0; i < NETWORK_START_URBS; i++) {
                struct urb *urb;
                BREAK_IF(!(urb = usbd_alloc_urb (function, BULK_OUT, 
                                                usbd_endpoint_transferSize(function, BULK_OUT, usbd_high_speed(function)), 
                                                network_recv_urb)));
                //printk(KERN_INFO"%s: i: %d start: %p\n", __FUNCTION__, i, urb);
                if (usbd_start_recv(urb)) 
                        usbd_dealloc_urb(urb);
        }
}

/* network_event_irg - Processes a USB event.
 */
static void network_event_irq (struct usb_function_instance *function, usb_device_event_t event, int data)
{
        struct usb_network_private *network_private = &Usb_network_private;

        switch (event) {

        case DEVICE_RESET:
        case DEVICE_DESTROY:	
        case DEVICE_BUS_INACTIVE:
                //printk(KERN_INFO"%s: RST %x\n", __FUNCTION__, ip_addr);
                {
                        //printk(KERN_INFO"%s:\n", __FUNCTION__);
                        // Return if argument is null.

                        // XXX flush

                        network_private->flags &= ~NETWORK_ATTACHED;
#ifndef CONFIG_ARCH_EZX
                        network_private->int_urb = NULL;
#endif

                        // Disable our net-device.
                        // Apparently it doesn't matter if we should do this more than once.

                        netif_stop_queue(&Network_net_device);
                        netif_carrier_off(&Network_net_device);

                        // If we aren't already tearing things down, do it now.
                        if (!(network_private->flags & NETWORK_DESTROYING)) {
                                network_private->flags |= NETWORK_DESTROYING;
                                //network_private->device = NULL;
                        }
                }
                break;

        case DEVICE_CONFIGURED:
        case DEVICE_BUS_ACTIVITY:
                //printk(KERN_INFO"%s: CFG %x\n", __FUNCTION__, ip_addr);
                network_private->flags |= NETWORK_ATTACHED;

                if ((network_private->network_type == network_blan) && (network_private->flags & NETWORK_OPEN)) 
                        notification_schedule_bh();
                
                netif_carrier_on (&Network_net_device);
                netif_wake_queue (&Network_net_device);

                network_start_recv(function);

#ifdef CONFIG_ARCH_EZX
//#ifdef CONFIG_USBD_NET_NFS_SUPPORT
                if ((!usb_is_configured) && (motusbd_nfs_flag)) {
                        wake_up(&usb_netif_wq);
                        usb_is_configured = 1;
                }
//#endif	// CONFIG_USBD_NET_NFS_SUPPORT
#endif	// CONFIG_ARCH_EZX
		
#if 1//def CONFIG_ARCH_EZX_BARBADOS
		if(event == DEVICE_CONFIGURED)
			queue_motusbd_event(USB_BLAN_READY);
#endif	// CONFIG_ARCH_EZX_BARBADOS
                break;

	case DEVICE_SET_INTERFACE:
                // XXX if CDC then we can check device->alternates[1] and see if we should
                // enable/disable data flow.
                // XXX verify ep0.c SET_CONFIGURATION and SET_INTERFACE implmentation are
                // complete before using this
		break;

        default:
                return;
        }
#ifdef CONFIG_HOTPLUG
        hotplug_schedule_bh();
#endif
}

//_________________________________________________________________________________________________

/* network_function_enable - enable the function driver
 *
 * Called for usbd_function_enable() from usbd_register_device()
 */

static int network_function_enable (struct usb_function_instance *function)
{
        //printk(KERN_INFO"%s: DOWN\n", __FUNCTION__);
        down(&usbd_network_sem);
	MOD_INC_USE_COUNT;  // QQQ Should this be _before_ the down()?

        // set the network device address from the local device address
        memcpy(Network_net_device.dev_addr, local_dev_addr, ETH_ALEN);

        //Usb_network_private.bus = function->bus;
        Usb_network_private.function = function;
        Usb_network_private.have_interrupt = usbd_endpoint_bEndpointAddress(function, INT_IN, usbd_high_speed(function)) ? 1 : 0;

        Usb_network_private.flags |= NETWORK_ENABLED;

#if defined(CONFIG_USBD_NETWORK_CDC)
        cdc_init(function);
#endif                          /* CONFIG_USBD_NETWORK_CDC */

#ifdef CONFIG_USBD_NETWORK_BASIC
        basic_init(function);
#endif

#ifdef CONFIG_USBD_NETWORK_BASIC2
        basic2_init(function);
#endif

#ifdef CONFIG_USBD_NETWORK_SAFE
        safe_init(function);
#endif
#ifdef CONFIG_USBD_NETWORK_BLAN
        blan_init(function);
#endif
        up(&usbd_network_sem);
        //printk(KERN_INFO"%s: UP\n", __FUNCTION__);
        return 0;
}

/* network_functino_disable - disable the function driver
 *
 */
static void network_function_disable (struct usb_function_instance *function)
{
        //printk(KERN_INFO"%s: DOWN\n", __FUNCTION__);
        down(&usbd_network_sem);
        Usb_network_private.flags &= ~NETWORK_ENABLED;
        //Usb_network_private.bus = NULL;
        Usb_network_private.function = NULL;
	MOD_DEC_USE_COUNT;  // QQQ Should this be _after_ the up()?
        up(&usbd_network_sem);
        //printk(KERN_INFO"%s: UP\n", __FUNCTION__);
}

struct usb_function_operations network_fd_function_ops = {
	recv_setup_irq:    network_recv_setup_irq,
	event_irq:     network_event_irq,
	function_enable: network_function_enable,
	function_disable: network_function_disable,
};

//_________________________________________________________________________________________________

/* network_create - create and initialize network private structure
 *
 * Returns non-zero for failure.
 */
static int network_create (void)
{
	struct usb_network_private *network_private = &Usb_network_private;

        //printk(KERN_INFO"%s:\n", __FUNCTION__);

	// Set some fields to generic defaults and register the network device with the kernel networking code

	memset(&network_private->stats, 0, sizeof network_private->stats);

        ether_setup (&Network_net_device);
        RETURN_EINVAL_IF (register_netdev(&Network_net_device));

        netif_stop_queue (&Network_net_device);
        netif_carrier_off (&Network_net_device);
        Network_net_device.flags &= ~IFF_UP;

        network_private->flags |= NETWORK_CREATED;

        network_private->maxtransfer = MAXFRAMESIZE + 4 + 64;

        network_private->flags |= NETWORK_REGISTERED;

        network_private->network_type = network_unknown;

        //printk(KERN_INFO"%s: finis\n", __FUNCTION__);
        return 0;
}

/* network_destroy - destroy network private struture
 *
 * Destroys the network interface referenced by the global variable @c network_private.
 */
static void network_destroy (void)
{
        if (Usb_network_private.flags & NETWORK_REGISTERED) {
                netif_stop_queue (&Network_net_device);
                netif_carrier_off (&Network_net_device);
                unregister_netdev (&Network_net_device);
        }
        Usb_network_private.flags = 0;
}


//______________________________________module_init and module_exit________________________________

/* hexdigit -
 *
 * Converts characters in [0-9A-F] to 0..15, characters in [a-f] to 42..47, and all others to 0.
 */
static __u8 hexdigit (char c)
{
	return isxdigit (c) ? (isdigit (c) ? (c - '0') : (c - 'A' + 10)) : 0;
}

/* set_address -
 */
void set_address(char *mac_address_str, __u8 *dev_addr)
{
        int i;
        if (mac_address_str && strlen(mac_address_str)) {
                for (i = 0; i < ETH_ALEN; i++) {
                        dev_addr[i] = 
                                hexdigit (mac_address_str[i * 2]) << 4 | 
                                hexdigit (mac_address_str[i * 2 + 1]);
                }
        }
        else {
                get_random_bytes(dev_addr, ETH_ALEN);
                dev_addr[0] = (dev_addr[0] & 0xfe) | 0x02;
        }
}

/* macstrtest -
 */
int macstrtest(char *mac_address_str)
{
        int l = 0;

        if (mac_address_str) {
                l = strlen(mac_address_str);
        }
        return ((l != 0) && (l != 12));
}

/* network_modinit - driver intialization
 *
 * Returns non-zero for failure.
 */
static int network_modinit (void)
{
        network_type_t  network_type = network_unknown;

        init_waitqueue_head(&usb_netif_wq);


        printk(KERN_INFO "Copyright (c) 2002-2004 Belcarra Technologies; www.belcarra.com; sl@belcarra.com\n");
        printk(KERN_INFO "%s: %s vendor_id: %04x product_id: %04x\n", __FUNCTION__, __usbd_module_info, vendor_id, product_id);

#ifdef CONFIG_USBD_NETWORK_BLAN_FERMAT
        //printk(KERN_INFO "%s: fermat\n", __FUNCTION__);
        fermat_init();
#endif

#if defined(CONFIG_USBD_NETWORK_CDC) 
        if (cdc) {
                //printk(KERN_INFO "%s: cdc\n", __FUNCTION__);
                network_type = network_cdc;
        }
#endif

#ifdef CONFIG_USBD_NETWORK_BASIC
        if (basic) {
                THROW_IF (network_type != network_unknown, select_error);
                //printk(KERN_INFO "%s: basic\n", __FUNCTION__);
                network_type = network_basic;
        }
#endif

#ifdef CONFIG_USBD_NETWORK_BASIC2
        if (basic2) {
                THROW_IF (network_type != network_unknown, select_error);
                //printk(KERN_INFO "%s: basic2\n", __FUNCTION__);
                network_type = network_basic2;
        }
#endif

#ifdef CONFIG_USBD_NETWORK_BLAN
        if (blan) {
                //printk(KERN_INFO "%s: blan\n", __FUNCTION__);
                THROW_IF (network_type != network_unknown, select_error);
                network_type = network_blan;
        }
#endif

#if defined(CONFIG_USBD_NETWORK_CDC)
        if (network_type == network_unknown) {
                //printk(KERN_INFO "%s: cdc\n", __FUNCTION__);
                network_type = network_cdc;
        }
#endif

        // still unknown - check for other bNumConfigurations

        if (network_type == network_unknown) {
#if defined(CONFIG_USBD_NETWORK_BASIC)
                //printk(KERN_INFO "%s: basic\n", __FUNCTION__);
                THROW_IF (network_type != network_unknown, select_error);
                network_type = network_basic;
#endif
#if defined(CONFIG_USBD_NETWORK_BASIC2)
                //printk(KERN_INFO "%s: basic2\n", __FUNCTION__);
                THROW_IF (network_type != network_unknown, select_error);
                network_type = network_basic2;
#endif
#if defined(CONFIG_USBD_NETWORK_SAFE)
                //printk(KERN_INFO "%s: safe\n", __FUNCTION__);
                THROW_IF (network_type != network_unknown, select_error);
                network_type = network_safe;
#endif
#if defined(CONFIG_USBD_NETWORK_BLAN)
                //printk(KERN_INFO "%s: blan\n", __FUNCTION__);
                THROW_IF (network_type != network_unknown, select_error);
                network_type = network_blan;
#endif
        }

        // sanity check
        THROW_IF (network_type == network_unknown, select_error);

        // select the function driver descriptors based on network_type

        switch (network_type) {

#if defined(CONFIG_USBD_NETWORK_BASIC)
        case network_basic:
                //printk(KERN_INFO "%s: basic\n", __FUNCTION__);
                function_driver = &basic_function_driver;
                break;
#endif

#if defined(CONFIG_USBD_NETWORK_BASIC2)
        case network_basic2:
                //printk(KERN_INFO "%s: basic2\n", __FUNCTION__);
                function_driver = &basic2_function_driver;
                break;
#endif

#if defined(CONFIG_USBD_NETWORK_SAFE)
        case network_safe:
                //printk(KERN_INFO "%s: blan bNumConfigurations: %d\n", __FUNCTION__, blan_function_driver.bNumConfigurations);
                function_driver = &safe_function_driver;
                break;
#endif
#if defined(CONFIG_USBD_NETWORK_BLAN)
        case network_blan:
                //printk(KERN_INFO "%s: blan bNumConfigurations: %d\n", __FUNCTION__, blan_function_driver.bNumConfigurations);
                function_driver = &blan_function_driver;
                break;
#endif

#if defined(CONFIG_USBD_NETWORK_CDC)
        case network_cdc:
                function_driver = &cdc_function_driver;
                break;
#endif
        default:
                THROW(select_error);
                break;
        }

        strncpy(Network_net_device.name, network_type == zeroconf ? "usbz0" : (network_blan ? "usbl0" : "usbb0"), 6);


        THROW_IF (!function_driver, select_error);

        CATCH(select_error) {
                printk(KERN_INFO "%s: configuration selection error\n", __FUNCTION__);
		return -EINVAL;
        }


#ifdef CONFIG_USBD_NETWORK_EP0TEST
        /*
         * ep0test - test that bus interface can do ZLP on endpoint zero
         *
         * This will artificially force iProduct string descriptor to be
         * exactly the same as the endpoint zero packetsize.  When the host
         * requests this string it will request it not knowing the strength
         * and will use a max length of 0xff. The bus interface driver must
         * send a ZLP to terminate the transaction.
         *
         * The iProduct descriptor is used because both the Linux and
         * Windows usb implmentations fetch this in a default enumeration. 
         *
         */
        if (ep0test) {
                switch (ep0test) {
                case 8:  function_driver->device_description->iProduct = "012"; break;
                case 16: function_driver->device_description->iProduct = "0123456"; break;
                case 32: function_driver->device_description->iProduct = "0123456789abcde"; break;
                case 64: function_driver->device_description->iProduct = "0123456789abcdef0123456789abcde"; break;
                default: printk(KERN_ERR"%s: ep0test: bad value: %d, must be one of 8, 16, 32 or 64\n", 
                                         __FUNCTION__, ep0test); return -EINVAL;
                         break;
                }
                printk(KERN_INFO"%s: ep0test: iProduct set to: %s\n", __FUNCTION__, 
                                function_driver->device_description->iProduct);
        }
        else 
                printk(KERN_INFO"%s: ep0test: not set\n", __FUNCTION__);
#endif /* CONFIG_USBD_NETWORK_EP0TEST */


#ifdef CONFIG_USBD_NETWORK_ALLOW_SETID
        //printk(KERN_INFO"%s: checking idVendor: %04x idProduct: %04x\n", __FUNCTION__, vendor_id, product_id);

        if (vendor_id) 
                function_driver->idVendor = cpu_to_le16(vendor_id);
        
        if (product_id) 
                function_driver->idProduct = cpu_to_le16(product_id);
#endif

        if ((macstrtest(local_mac_address_str) || macstrtest(remote_mac_address_str))) {
                printk(KERN_INFO"%s: bad size %s %s\n", __FUNCTION__, local_mac_address_str, remote_mac_address_str);
		return -EINVAL;
        }

        set_address(local_mac_address_str, local_dev_addr);
        set_address(remote_mac_address_str, remote_dev_addr);

        RETURN_EINVAL_IF(network_create());
        
        Usb_network_private.network_type = network_type;

        Usb_network_private.notification_bh.routine = notification_bh;
        Usb_network_private.notification_bh.data = NULL;

#ifdef CONFIG_HOTPLUG
        Usb_network_private.hotplug_bh.routine = hotplug_bh;
        Usb_network_private.hotplug_bh.data = NULL;
#endif

        memcpy(Network_net_device.dev_addr, local_dev_addr, ETH_ALEN);

        THROW_IF(make_crc_table(), error);
        
        Usb_network_private.encapsulation = simple_crc;

	THROW_IF(usbd_register_function (function_driver), error);

#ifdef CONFIG_ARCH_EZX
	usblan_ip_addr = 0;
#endif
	
	return 0;

        CATCH(error) {
                network_destroy();
		return -EINVAL;
        }
}

//_________________________________________________________________________________________________

/* network_modexit - driver exit
 *
 * Cleans up the module. Deregisters the function driver and destroys the network object.
 */
static void network_modexit (void)
{
	//printk(KERN_INFO"%s: exiting\n", __FUNCTION__);
        
        while (Usb_network_private.notification_bh.sync) {
                printk(KERN_ERR"%s: waiting for notificationhotplug bh\n", __FUNCTION__);
                schedule_timeout(10 * HZ);
        }

#ifdef CONFIG_HOTPLUG
        while (Usb_network_private.hotplug_bh.sync) {
                printk(KERN_ERR"%s: waiting for hotplug bh\n", __FUNCTION__);
                schedule_timeout(10 * HZ);
        }
#endif

	usbd_deregister_function (function_driver);
        network_destroy();
        if (network_crc32_table) {
                lkfree(network_crc32_table);
                network_crc32_table = NULL;
        }

#ifdef CONFIG_ARCH_EZX
	usblan_ip_addr = -1;
#endif
}

//_________________________________________________________________________________________________

module_init (network_modinit);
module_exit (network_modexit);

