/*
 * usbd/acm_fd/acm.c
 *
 *      Copyright (c) 2003, 2004 Belcarra
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
 *
 * Note: this function driver requires the following endpoints:
 *
 *      BULK-IN
 *      BULK-OUT
 *      INTERRUPT-IN
 *
 * This function driver cannot be used on devices (such as the StrongArm
 * SA1100) that do not have and interrupt endpoint.
 *
 */

/*
 * Copyright (C) 2004 - Motorola
 *
 * 2004-Dec-06 - Flow control for EZXBASE By Zhao Liang <w20146@motorola.com>
 *
 */



#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include <usbd-export.h>
#include <usbd-build.h>

MODULE_AUTHOR ("sl@belcarra.com, tbr@belcarra.com");

MODULE_DESCRIPTION ("Belcarra CDC-ACM Function");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,17)
MODULE_LICENSE("GPL");
#endif

#define ACM_TRACE_NAME "acm_trace"

#undef USE_TICKER
#undef MCEL

#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/ctype.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <asm/atomic.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/smp_lock.h>
#include <linux/slab.h>

#include <usbd-chap9.h>
#include <usbd-mem.h>
#include <usbd.h>
#include <usbd-func.h>

#include <usbd-bus.h>		// UDC

#include "trace.h"

#include <linux/ezxusbd.h>

#ifdef CONFIG_ARCH_EZXBASE
#include <linux/modem/connection_monitor.h>
#endif

// Define the low order 16 bits of an urb's memory address as it's ID for tracing.
#define urbID(urb) (0xffff & (u32) (void *) urb)
int acm_interrupts;

USBD_MODULE_INFO ("acm_fd 2.1-beta");

#ifdef CONFIG_ARCH_EZX
#define MCEL

#define MAX_QUEUED_BYTES        256
#define MAX_QUEUED_URBS         10	// 64???

#define MAX_RECV_URBS           2      // Max for receiving data
#define RECV_RING_SIZE          (MAX_RECV_URBS+1)

#else

#define MAX_QUEUED_BYTES        256
#define MAX_QUEUED_URBS         10     // Max for write

#define MAX_RECV_URBS           2      // Max for receiving data
#define RECV_RING_SIZE          (MAX_RECV_URBS+1)

#endif

// Endpoint indexes in acm_endpoint_requests[] and the endpoint map.
#define BULK_OUT        0x00
#define BULK_IN         0x01
#define INT_IN          0x02

#ifdef CONFIG_USBD_NM
#define NM_NEED_INT_IN  0x04
#endif

#define ENDPOINTS	0x03


#define COMM_INTF       0x00
#define DATA_INTF       0x01


#ifdef CONFIG_USBD_NM
int get_acm_interrupt_endpoint(void);
extern int motusbd_status;
#endif


/* Module Parameters ************************************************************************* */

static u32 vendor_id;
static u32 product_id;
static u32 max_queued_urbs = MAX_QUEUED_URBS;
static u32 max_queued_bytes = MAX_QUEUED_BYTES;

MODULE_PARM (vendor_id, "i");
MODULE_PARM (product_id, "i");
MODULE_PARM (max_queued_urbs, "i");
MODULE_PARM (max_queued_bytes, "i");

MODULE_PARM_DESC (vendor_id, "Device Vendor ID");
MODULE_PARM_DESC (product_id, "Device Product ID");
MODULE_PARM_DESC (max_queued_urbs, "Maximum TX Queued Urbs");
MODULE_PARM_DESC (max_queued_bytes, "Maximum TX Queued Bytes");

/*
 * CDC ACM Configuration
 *
 * Endpoint, Class, Interface, Configuration and Device descriptors/descriptions
 */

/* Endpoints */
static __u8 acm_alt_1[] = { 0x07, USB_DT_ENDPOINT, OUT, BULK,      0, 0x00, 0x00, };
static __u8 acm_alt_2[] = { 0x07, USB_DT_ENDPOINT, IN,  BULK,     0,  0x00, 0x00, };
static struct usb_endpoint_descriptor *acm_alt_endpoints[] = { 
        (struct usb_endpoint_descriptor *) acm_alt_1, 
        (struct usb_endpoint_descriptor *) acm_alt_2, };
u8 acm_alt_indexes[] = { BULK_OUT, BULK_IN, };

static __u8 acm_comm_1[] = { 0x07, USB_DT_ENDPOINT, IN,  INTERRUPT, 0, 0x00, 0x0a, };
static struct usb_endpoint_descriptor *acm_comm_endpoints[] = { (struct usb_endpoint_descriptor *) acm_comm_1 };
u8 acm_comm_indexes[] = { INT_IN, };

static __u8 cdc_class_1[] = { 0x05, CS_INTERFACE, USB_ST_HEADER, 0x01, 0x01, /* CLASS_BDC_VERSION, CLASS_BDC_VERSION */ };
static __u8 cdc_class_2[] = { 0x05, CS_INTERFACE, USB_ST_CMF, 0x03, 0x01, /* bMasterInterface: 0, bSlaveInterface: 1 */ };
static __u8 cdc_class_3[] = { 0x05, CS_INTERFACE, USB_ST_UF, 0x00, 0x01, /* bMasterInterface: 0, bSlaveInterface: 1 */ };

/* ACMF - c.f. Table 28
 * currenty set to 0x2 - Support Set_Line_Coding etc, 
 *
 * XXX Should we also set 0x4 - Supports Network_Notification?
 */
static __u8 cdc_class_4[] = { 0x04, CS_INTERFACE, USB_ST_ACMF, 0x02, };

static struct usb_generic_class_descriptor *cdc_comm_class_descriptors[] = 
        { (struct usb_generic_class_descriptor *) cdc_class_1, 
        (struct usb_generic_class_descriptor *) cdc_class_2, 
        (struct usb_generic_class_descriptor *) cdc_class_3, 
        (struct usb_generic_class_descriptor *) cdc_class_4, };


/* Alternate Descriptors */
// First two bytes are identical in all: bLength, bDescriptorType (0x09 0x04)
static __u8 cdc_comm_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
        0x09, USB_DT_INTERFACE, COMM_INTF, 0x00, 0x00, // bInterfaceNumber, bAlternateSetting, bNumEndpoints
        COMMUNICATIONS_INTERFACE_CLASS, COMMUNICATIONS_ACM_SUBCLASS, 0x01, 0x00, };

static __u8 cdc_data_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
        0x09, USB_DT_INTERFACE, DATA_INTF, 0x00, 0x00, // bInterfaceNumber, bAlternateSetting, bNumEndpoints
        DATA_INTERFACE_CLASS, COMMUNICATIONS_NO_SUBCLASS, COMMUNICATIONS_NO_PROTOCOL, 0x00, };

/* Alternate Descriptions */
//static struct usb_alternate_description cdc_comm_alternate_descriptions[] = {
struct usb_alternate_description cdc_comm_alternate_descriptions[] = {
      { iInterface: CONFIG_USBD_ACM_COMM_INTF,
              interface_descriptor: (struct usb_interface_descriptor *)&cdc_comm_alternate_descriptor,
	      classes:sizeof (cdc_comm_class_descriptors) / sizeof (struct usb_generic_class_descriptor *),
              class_list: cdc_comm_class_descriptors,
              endpoint_list: acm_comm_endpoints,
	      endpoints:sizeof (acm_comm_endpoints) / sizeof(struct usb_endpoint_descriptor *),
              endpoint_indexes: acm_comm_indexes,
      }, };

//static struct usb_alternate_description cdc_data_alternate_descriptions[] = {
struct usb_alternate_description cdc_data_alternate_descriptions[] = {
      { iInterface: CONFIG_USBD_ACM_DATA_INTF,
              interface_descriptor: (struct usb_interface_descriptor *)&cdc_data_alternate_descriptor,
              endpoint_list: acm_alt_endpoints,
	      endpoints:sizeof (acm_alt_endpoints) / sizeof(struct usb_endpoint_descriptor *),
              endpoint_indexes: acm_alt_indexes,
              }, };

/* Interface Descriptions */
static struct usb_interface_description cdc_interfaces[] = {
      { 
                alternates:sizeof (cdc_comm_alternate_descriptions) / sizeof (struct usb_alternate_description),
                alternate_list:cdc_comm_alternate_descriptions,
      },

      { 
                alternates:sizeof (cdc_data_alternate_descriptions) / sizeof (struct usb_alternate_description),
                alternate_list:cdc_data_alternate_descriptions,
      },
};


/* Configuration Descriptor and Description */
static __u8 cdc_configuration_descriptor[sizeof(struct usb_configuration_descriptor)] = {
        0x09, USB_DT_CONFIG, 0x00, 0x00, sizeof (cdc_interfaces) / sizeof (struct usb_interface_description),
        0x01, 0x00, BMATTRIBUTE, BMAXPOWER, };

struct usb_configuration_description acm_description[] = {
      { iConfiguration: CONFIG_USBD_ACM_DESC,
              configuration_descriptor: (struct usb_configuration_descriptor *)cdc_configuration_descriptor,
	      bNumInterfaces:sizeof (cdc_interfaces) / sizeof (struct usb_interface_description),
              interface_list:cdc_interfaces,}, };

static struct usb_device_descriptor acm_device_descriptor = {
	bLength: sizeof(struct usb_device_descriptor),
	bDescriptorType: USB_DT_DEVICE,
	bcdUSB: __constant_cpu_to_le16(USB_BCD_VERSION),
	bDeviceClass: COMMUNICATIONS_DEVICE_CLASS,
	bDeviceSubClass: 0x02,
	bDeviceProtocol: 0x00,
	bMaxPacketSize0: 0x00,
	idVendor: __constant_cpu_to_le16(CONFIG_USBD_ACM_VENDORID),
	idProduct: __constant_cpu_to_le16(CONFIG_USBD_ACM_PRODUCTID),
	bcdDevice: __constant_cpu_to_le16(CONFIG_USBD_ACM_BCDDEVICE),
};

#ifdef CONFIG_USBD_HIGH_SPEED
static struct usb_device_qualifier_descriptor acm_device_qualifier_descriptor = {
	bLength: sizeof(struct usb_device_qualifier_descriptor),
	bDescriptorType: USB_DT_DEVICE_QUALIFIER,
	bcdUSB: __constant_cpu_to_le16(USB_BCD_VERSION),
	bDeviceClass: COMMUNICATIONS_DEVICE_CLASS,
	bDeviceSubClass: 0x02,
	bDeviceProtocol: 0x00,
	bMaxPacketSize0: 0x00,
};
#endif /* CONFIG_USBD_HIGH_SPEED */

static struct usb_endpoint_request acm_endpoint_requests[ENDPOINTS+1] = {
        { 1, 1, 0, USB_DIR_OUT | USB_ENDPOINT_BULK, 64, 512, },
        { 1, 1, 0, USB_DIR_IN | USB_ENDPOINT_BULK, 64 /* * 4 */, 512, },
        { 1, 0, 0, USB_DIR_IN | USB_ENDPOINT_INTERRUPT, 16, 64, },
        { 0, },
};

#ifdef CONFIG_USBD_OTG
#error
static struct usb_otg_descriptor acm_otg_descriptor = {
	bLength : sizeof(struct usb_otg_descriptor),
	bDescriptorType: USB_DT_OTG,
	bmAttributes: 0,
};
#endif /* CONFIG_USBD_OTG */

struct usb_device_description acm_device_description = {
        device_descriptor: &acm_device_descriptor,
#ifdef CONFIG_USBD_HIGH_SPEED
	device_qualifier_descriptor: &acm_device_qualifier_descriptor,
#endif /* CONFIG_USBD_HIGH_SPEED */
#ifdef CONFIG_USBD_OTG
	otg_descriptor: &acm_otg_descriptor,
#endif /* CONFIG_USBD_OTG */
	iManufacturer: CONFIG_USBD_ACM_MANUFACTURER,
	iProduct: CONFIG_USBD_ACM_PRODUCT_NAME,
#if !defined(CONFIG_USBD_NO_SERIAL_NUMBER) && defined(CONFIG_USBD_SERIAL_NUMBER_STR)
	iSerialNumber: CONFIG_USBD_SERIAL_NUMBER_STR, 
#endif
        endpointsRequested: ENDPOINTS,
        requestedEndpoints: acm_endpoint_requests,
};

/* Missing atomic functions ******************************************************************** */

static __inline__ int atomic_post_inc(volatile atomic_t *v)
{
	unsigned long flags;
	int result;
	local_irq_save(flags);
	result = (v->counter)++;
	local_irq_restore(flags);
	return(result);
}

static __inline__ int atomic_pre_dec(volatile atomic_t *v)
{
	unsigned long flags;
	int result;
	local_irq_save(flags);
	result = --(v->counter);
	local_irq_restore(flags);
	return(result);
}

/* ACM ***************************************************************************************** */
/*
 * Output and Input control lines and line errors.
 */
#define LINE_OUT_DTR            0x01
#define LINE_OUT_RTS            0x02
#define LINE_IN_DCD             0x01
#define LINE_IN_DSR             0x02
#define LINE_IN_BRK             0x04
#define LINE_IN_RI              0x08
#define LINE_IN_FRAMING         0x10
#define LINE_IN_PARITY          0x20
#define LINE_IN_OVERRUN         0x40

/* ******************************************************************************************* */

#define ACM_TTY_MAJOR   166
#define ACM_TTY_MINOR   0
#define ACM_TTY_MINORS  1

struct acm_private {
        struct usb_function_instance *function;
        struct tty_driver *tty_driver;
        int tty_driver_registered;              // non-zero if tty_driver registered
        int usb_driver_registered;              // non-zero if usb function registered

        struct tty_struct *tty;                 // non-null if tty open
        struct tq_struct wqueue;                // task queue for writer wakeup 
        struct tq_struct hqueue;                // task queue for hangup
        wait_queue_head_t open_wait;            // wait queue for blocking open
	int open_wait_count;			// count of (possible) blocked
        int exiting;				// True if module exiting

        unsigned char throttle;                 // non-zero if we are throttled
        unsigned char clocal;                   // non-zero if clocal set
        unsigned char connected;                // non-zero if connected to host (configured)

        unsigned int writesize;                 // packetsize * 4
        unsigned int ctrlin;                    // line state device sends to host
        unsigned int ctrlout;                   // line state device received from host

	int exclusive;
        atomic_t used;
        atomic_t queued_bytes;
        atomic_t queued_urbs;

	int recv_urbs;
	struct tq_struct recv_tqueue;

	/*TBR debug receive flow control */
	unsigned long bytes_received;
	unsigned long bytes_forwarded;
	/*TBR end debug */
};

static struct acm_private acm_private;

static int acm_send_int_notification(struct usb_function_instance *, int , int );
static int acm_urb_sent_bulk (struct urb *urb, int rc);
static int acm_urb_sent_int (struct urb *urb, int rc);
static int acm_recv_urb (struct urb *urb, int rc);

/* Serial Functions **************************************************************************** */

static void acm_schedule(struct tq_struct *queue)
{
	TRACE_MSG1("task %p",queue);
        RETURN_IF(!queue->data || queue->sync);
        MOD_INC_USE_COUNT;
        queue_task(queue, &tq_immediate);
        mark_bh(IMMEDIATE_BH);
	TRACE_MSG1("task %p scheduled and marked",queue);
}

                                
static int block_until_ready(
	struct tty_struct *tty,
	struct file *filp,
	struct acm_private *acm)
{                               
        unsigned long flags;    
        int rc = 0;             
                        
        // FUTURE: check tty for non-blocking open...
        
        local_irq_save(flags);
	TRACE_MSG1("owc=%d --> at entry",acm->open_wait_count);
        acm->open_wait_count += 1;
        for (;;) {
                if (tty_hung_up_p(filp)) {
			TRACE_MSG("tty_hung_up_p()");
                        rc = -ERESTARTSYS;
                        break;
                }
                if (signal_pending(current)) {
			TRACE_MSG("signal_pending()");
                        rc = -ERESTARTSYS;
                        break;  
                }               
                if (acm->exiting) {
			TRACE_MSG("module exiting()");
                        rc = -ENODEV;
                        break;  
                }               
                if (acm->ctrlout & LINE_OUT_DTR) { 
                        // OK, there's somebody on the other end, let's go...
			TRACE_MSG("found DTR");
                        break;
                }
		TRACE_MSG1("owc=%d sleeping...",acm->open_wait_count);
                interruptible_sleep_on(&acm->open_wait);
		TRACE_MSG1("owc=%d got WAKEUP",acm->open_wait_count);
        }
        acm->open_wait_count -= 1;
	TRACE_MSG1("owc=%d <-- at exit",acm->open_wait_count);
        local_irq_restore(flags);
        return(rc);
}

static void acm_hangup(void *private)
{
        //struct acm_private *acm = &acm_private;
        struct acm_private *acm = (struct acm_private *) private;
        struct tty_struct *tty = (NULL == acm) ? NULL : acm->tty;
	TRACE_MSG("entered");

        if (tty && !acm->clocal) 
                tty_hangup(tty);

        wake_up_interruptible(&acm->open_wait);

        MOD_DEC_USE_COUNT;
        //printk (KERN_INFO"%s: MOD: %d connected: %d tty: %p clocal: %x\n", 
        //                __FUNCTION__, MOD_IN_USE, acm->connected, tty, acm->clocal);
	TRACE_MSG("exited");
}

static int acm_recv_urb (struct urb *urb, int rc);
void acm_start_recv_urbs(struct tty_struct *tty) 
{
	struct acm_private *acm = &acm_private;
	struct usb_function_instance *function = acm->function;
	unsigned long flags;
	int try_again;

	TRACE_MSG1("START RECV: tty: %x", (int)tty);
	TRACE_MSG2("START RECV: connected: %x recv_urbs: %d", acm->connected, acm->recv_urbs);

	RETURN_IF(!function);
	local_irq_save(flags);
	if (tty && acm->connected && !acm->throttle) {
		// flow control of ACM driver is based on checking free space of upper layer, normally flip layer.
		// In TTY architecture, the buffer size for flip layer is 512B(TTY_FLIPBUF_SIZE),
		// while the TTY_THRESHOLD_THROTTLE in ld(line discipline) layer is 128B.
		// Copying data from flip layer to ld layer by calling tty_flip_buffer_push() might lose data.
		// It is not a good solution to modify source code of tty layer, esepcially increase TTY_THRESHOLD_THROTTLE.
		// So the better solution is to check the free space of both flip layer and ld layer in ACM driver.
		// -- w20146
		int free_space = TTY_FLIPBUF_SIZE - tty->flip.count;
		if(tty->ldisc.receive_room)
		{
			int ld_free = tty->ldisc.receive_room(tty);
			free_space = MIN(free_space, ld_free);
		}
		TRACE_MSG2("START RECV: throttled: %d flip count: %d", acm->throttle, tty->flip.count);
		while ( ( (acm->recv_urbs + 1) *
			usbd_endpoint_transferSize (function, BULK_OUT, usbd_high_speed(function)) ) <= free_space) {	
			struct urb *urb;
			BREAK_IF(!(urb = usbd_alloc_urb(function, BULK_OUT, 
						usbd_endpoint_transferSize(function, BULK_OUT, usbd_high_speed(function)),
						acm_recv_urb)));
			acm->recv_urbs++;
			TRACE_MSG1("START RECV: %d", acm->recv_urbs);
			CONTINUE_UNLESS (usbd_start_recv(urb));
			acm->recv_urbs--;
			TRACE_MSG1("START RECV: %d", acm->recv_urbs);
			usbd_dealloc_urb(urb);
			break;
		}
	}
	try_again = (0 >= acm->recv_urbs);
	local_irq_restore(flags);
	if (try_again) {
		/* There are no recv urbs queued, but there needs to be at least one
		 * in order to keep driving the tty_flip_buff_push, so try agian later. */
		TRACE_MSG1("queueing acm_start_recv sync=%d",acm->recv_tqueue.sync);
		queue_task(&acm->recv_tqueue, &tq_timer);
	}
}

static void acm_start_recv(void *private)
{
	/* This routine is queued by acm_start_recv_urbs() when it
	 * fails to get any recv urbs queued and wants to try again
	 * at a later point. */
	struct acm_private *acm = (struct acm_private *) private;
	struct tty_struct *tty = (NULL == acm) ? NULL : acm->tty;
	TRACE_MSG("entered");

	if (tty && !acm->throttle) {
		TRACE_MSG("attempting to flip");
		tty_flip_buffer_push(tty);
		TRACE_MSG("attempting to start");
		acm_start_recv_urbs(tty);
	}

	TRACE_MSG("exited");
}

static int acm_tty_open(struct tty_struct *tty, struct file *filp)
{
        struct acm_private *acm = &acm_private;
        //struct usb_function_instance *function = acm->function;
        int used;
	int nonblocking;
        int rc = 0;
	unsigned long flags;

	TRACE_MSG2("used=%d MOD=%d",atomic_read(&acm->used),MOD_IN_USE);
        //printk (KERN_INFO"%s: bus: %p used: %d MOD: %d\n",
	//	 __FUNCTION__, acm->function, atomic_read(&acm->used), MOD_IN_USE);
        nonblocking = filp->f_flags & O_NONBLOCK;
        /* Lock and increment used counter, save current value.
           Check for exclusive open at the same time. */
	local_irq_save(flags);
	/* The value of acm->used controls MOD_{INC/DEC}_USE_COUNT, so
           it has to be incremented unconditionally, and no early return
           made until after USE_COUNT has been adjusted to match. */
        used = atomic_post_inc(&acm->used);
#ifdef MCEL
        //filp->f_flags |= O_EXCL;  // QQQ Can we persuade MCEL to add this to their app? => w20146 comments!!!
        if (nonblocking && !(acm->connected)) {
		// QQQ Is MCEL actually using this "feature"?  (See below for printk)
                rc = -EINVAL;
        } else
#endif
        if (filp->f_flags & O_EXCL) {
		/* This is intended to be an exclusive open, so
                   make sure no one has the device open already, and
                   set the exclusive flag so no one can open it later. */
		if (used > 0) {
			// Someone already has it.
			rc = -EBUSY;
		} else {
			acm->exclusive = 1;
			set_bit(TTY_EXCLUSIVE, &tty->flags);
		}
	} else if (acm->exclusive != 0 && !suser()) {
		// Only the superuser can do a normal open of an O_EXCL tty
		rc = -EBUSY;
	}
	local_irq_restore(flags);
	if (0 == used) {
		// The value before incrementing was 0, this is the first open.
	        MOD_INC_USE_COUNT;
	}
	// OK, now it's safe to make an early return.
	if (0 != rc) {
#ifdef MCEL
		// This can dissappear when the "feature" above does.
		//if (-EINVAL == rc)
                //	printk(KERN_INFO "\nusb cable not connected!\n");
#endif
		return(rc);
	}

	acm_start_recv_urbs(tty);
	
	/* To truly emulate the old dual-device approach of having a non-blocking
           device (e.g cu0) and a blocking device (e.g. tty0) we would need to
           track blocking and non-blocking opens separately.  We don't.  This
           may lead to funny behavior in the multiple open case. */
	if (0 == used) {
		// First open.
		TRACE_MSG2("FIRST OPEN nb=%x xo=%x",nonblocking,acm->exclusive);
	       	//printk (KERN_INFO"%s: FIRST OPEN used_%sblock: s=%d, a=%d x=%d\n",
		//	 __FUNCTION__,(nonblocking?"non":""), used, atomic_read(&acm->used),
		//	 acm->exclusive);
		tty->driver_data = acm;
	        acm->tty = tty;
	        tty->low_latency = 1;
		acm->bytes_received = 0;
		acm->bytes_forwarded = 0;
		acm->throttle = 0;
	        acm->ctrlin = LINE_IN_DCD | LINE_IN_DSR;
	        if (NULL != acm->function) {
			TRACE_MSG1("sending notification ctrlin=%x",acm->ctrlin);
	       	        //printk (KERN_INFO"%s: sending ctrlin notification\n",__FUNCTION__);
	                rc = acm_send_int_notification(acm->function, CDC_NOTIFICATION_SERIAL_STATE,
			                               acm->ctrlin);
		}
	}

        if (0 == rc && !nonblocking) {
        	// Blocking open - all callers block until DTR shows up.
                rc = block_until_ready(tty,filp,acm);
	}
                
        /* The tty layer calls acm_tty_close() even if this open fails,
           so any cleanup (rc != 0) will be done there. */
	TRACE_MSG2("used=%d rc=%d",atomic_read(&acm->used),rc);
        return(rc);
}

static void acm_tty_close(struct tty_struct *tty, struct file *filp)
{
        struct acm_private *acm;
        struct usb_function_instance *function;	
        int used;

	if(tty && tty->driver_data)
		acm = tty->driver_data;
	else
		acm = &acm_private;

	function = acm->function;
	
	TRACE_MSG2("used=%d MOD=%d",atomic_read(&acm->used),MOD_IN_USE);
        //printk (KERN_INFO"%s: function: %p used: %d MOD: %d\n", 
	//		__FUNCTION__, function, atomic_read(&acm->used), MOD_IN_USE);	// aaa

        // lock and decrement used counter, save result
        used = atomic_pre_dec(&acm->used);

        // finished unless this is the last close
        if (used <= 0) {
                // This is the last close, clean up

                acm->tty = NULL;
                acm->ctrlin = 0x0;
		acm->throttle = 0;

                if (acm->function) {
                        acm_send_int_notification(function, CDC_NOTIFICATION_SERIAL_STATE, acm->ctrlin);
                }
                /* This should never happen if this is the last close,
                   but it can't hurt to check. */
                if (acm->open_wait_count) {
                        wake_up_interruptible(&acm->open_wait);
                }
		if (acm->exclusive) {
			acm->exclusive = 0;
			if(NULL != tty)
				clear_bit(TTY_EXCLUSIVE, &tty->flags);
		}
                MOD_DEC_USE_COUNT;
		TRACE_MSG1("LAST CLOSE r-f=%d",(acm->bytes_received-acm->bytes_forwarded));
                //printk (KERN_INFO"%s: LAST CLOSE used: %d MOD: %d bytes_received: %lu forwarded: %lu\n",
		//	 __FUNCTION__, atomic_read(&acm->used), MOD_IN_USE, acm->bytes_received,
		//	acm->bytes_forwarded);

		//usbd_flush_endpoint_index(function, BULK_IN);
		//usbd_flush_endpoint_index(function, BULK_OUT);
        }
#if 0
	TRACE_MSG("exited");
#else
	TRACE_MSG1("exited: used = %d", atomic_read(&acm->used));
#endif
}

/* Transmit Function - called by serproto ****************************************************** */

static int acm_tty_write(struct tty_struct *tty, int from_user, const unsigned char *buf, int count)
{
        struct acm_private *acm = tty->driver_data;
        struct usb_function_instance *function = acm->function;
	struct urb *urb;

	TRACE_MSG1("count=%d",count);
        //printk (KERN_INFO"%s: used: %d count: %d\n", __FUNCTION__, atomic_read(&acm->used), count);

        RETURN_EINVAL_IF(!function);
//	TRACE_MSG("function OK");

        // sanity check and are we connect
        RETURN_EINVAL_IF(!atomic_read(&acm->used)); 
//	TRACE_MSG("used OK");
        RETURN_ZERO_IF (!count || !acm->connected);
//	TRACE_MSG("connected OK");
	
	TRACE_MSG1("queued_urbs=%d", atomic_read(&acm->queued_urbs));
        RETURN_ZERO_IF(max_queued_urbs <= atomic_read(&acm->queued_urbs));
	TRACE_MSG("max_queued_urbs OK");

        // allocate a write urb
        count = MIN(count, acm->writesize);

        // XXX verify that we don't attempt to send wMaxPacketSize packets...
        if ( !(count % usbd_endpoint_wMaxPacketSize (function, BULK_OUT, usbd_high_speed(function) ) ) )  //QQSV
                count--;
        
        //printk(KERN_INFO"%s:\n", __FUNCTION__);
	RETURN_ENOMEM_IF (!(urb = usbd_alloc_urb (function, BULK_IN, count, acm_urb_sent_bulk)));
	TRACE_MSG("alloc_urb OK");

        if (from_user) 
                copy_from_user(urb->buffer, buf, count);
        else 
                memcpy(urb->buffer, buf, count);
        urb->privdata = acm;
        urb->actual_length = count;
        atomic_add(count, &acm->queued_bytes);
        atomic_inc(&acm->queued_urbs);
        usbd_send_urb(urb);
	TRACE_MSG2("urbID#%04x --> %d",urbID(urb),count);
        return count;
}

static int acm_tty_write_room(struct tty_struct *tty)
{       
        struct acm_private *acm = &acm_private;
	int rc;

	TRACE_MSG("entered");
        //printk (KERN_INFO"%s: used: %d queued: %d %d room: %d\n", __FUNCTION__, atomic_read(&acm->used),
        //                atomic_read(&acm->queued_urbs), atomic_read(&acm->queued_bytes),
        //                (max_queued_urbs > atomic_read(&acm->queued_urbs)) && 
        //                (max_queued_bytes > atomic_read(&acm->queued_bytes)) ? acm->writesize : 0);

        RETURN_EINVAL_IF(!atomic_read(&acm->used)); 
	TRACE_MSG("used OK");
        rc = (!acm->function || max_queued_urbs <= atomic_read(&acm->queued_urbs) ||
              max_queued_bytes <= atomic_read(&acm->queued_bytes) ) ?  0 : acm->writesize ;
	TRACE_MSG1("--> %d",rc);
        return(rc);
}       

static int acm_tty_chars_in_buffer(struct tty_struct *tty)
{       
        struct acm_private *acm = &acm_private;
	int rc;
#ifndef CONFIG_ARCH_EZX
//	TRACE_MSG("entered");
#endif
        RETURN_EINVAL_IF(!atomic_read(&acm->used)); 
//	TRACE_MSG("used OK");
        RETURN_ZERO_IF(!acm->function); 
//	TRACE_MSG("function OK");
        rc = atomic_read(&acm->queued_bytes);
//	TRACE_MSG1("--> %d",rc);
        return(rc);
}

/* acm_blocked is used to trigger debugging output in the tty layer
 *    after throttling has occured. */
//extern int acm_blocked;
static int throttle_count = 0;
static int unthrottle_count = 0;

static void acm_tty_throttle(struct tty_struct *tty)
{       
        struct acm_private *acm = &acm_private;
	throttle_count += 1;
	TRACE_MSG1("entered %d", throttle_count);
        //uuu printk (KERN_INFO"%s:\n", __FUNCTION__);
	acm->throttle = 1;
	//acm_blocked = 1;
	TRACE_MSG1("exited %d", throttle_count);
}               

static void acm_tty_unthrottle(struct tty_struct *tty)
{               
        struct acm_private *acm = &acm_private;
	struct urb *urb;
	unthrottle_count += 1;
	TRACE_MSG1("entered %d", unthrottle_count);
        //uuu printk (KERN_INFO"%s:\n", __FUNCTION__);
	acm->throttle = 0;
	/* This function is called while the TTY_DONT_FLIP flag is still
	   set, so there is no point trying to push the flip buffer.  Just
	   try to queue some recv urbs, and keep trying until we do manage
	   to get some queued. */
	tty_schedule_flip(tty);
	acm_start_recv_urbs(tty);
	TRACE_MSG1("exited %d", unthrottle_count);
}       

static int acm_tty_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg)
{       
        struct acm_private *acm = &acm_private;
	struct usb_function_instance *function = acm->function;
        unsigned int mask;
        unsigned int newctrl;

	TRACE_MSG("entered");
        //printk (KERN_INFO"%s: used: %d\n", __FUNCTION__, atomic_read(&acm->used));
        RETURN_EINVAL_IF(!atomic_read(&acm->used));

#ifdef CONFIG_ARCH_EZX
	switch(cmd) {
		case TCGETS:
		case TCFLSH:
		case TCSETS:
			//printk (KERN_INFO"%s: ignored cmd: %x\n", __FUNCTION__, cmd);
			return 0;

		case TIOCMGET: 
#if defined (CONFIG_ARCH_EZX_E680) || defined (CONFIG_ARCH_EZX_A780)
			// Per application(modem engine)'s request
			return put_user( ((acm->ctrlout & LINE_OUT_DTR) ? TIOCM_DTR : 0) | TIOCM_CTS, (unsigned long *) arg);
#else//elif CONFIG_ARCH_EZX_BARBADOS
			// It is the right way to do ioctl.
			return put_user((acm->ctrlout & LINE_OUT_DTR ? TIOCM_DTR : 0) |
					(acm->ctrlout & LINE_OUT_RTS ? TIOCM_RTS : 0) |
					(acm->ctrlin  & LINE_IN_DSR ? TIOCM_DSR : 0) |
					(acm->ctrlin  & LINE_IN_RI  ? TIOCM_RI  : 0) |
					(acm->ctrlin  & LINE_IN_DCD ? TIOCM_CD  : 0) |
					TIOCM_CTS, (unsigned long *) arg);
#endif

		case TIOCMSET:
		case TIOCMBIS:
		case TIOCMBIC:
			//printk (KERN_INFO"%s: TIOCM{SET,BIS,BIC}: arg 0x%x\n", __FUNCTION__, arg);
			RETURN_EFAULT_IF (get_user(mask, (unsigned long *) arg));
			//printk (KERN_INFO"%s: TIOCM{SET,BIS,BIC}: %04x, ctrlin 0x%x\n", __FUNCTION__, mask, acm->ctrlin);
			newctrl = acm->ctrlin;		// ctrlout???
#if defined (CONFIG_ARCH_EZX_E680) || defined (CONFIG_ARCH_EZX_A780)
			// Per application(modem engine)'s request
			if(0)
				mask = (mask & TIOCM_DTR) ? (LINE_IN_DCD|LINE_IN_DSR) : 0;
				/* | (mask & TIOCM_RTS ? LINE_OUT_RTS : 0)*/
			else
				mask = (mask & TIOCM_DTR ? LINE_OUT_DTR : 0) | (mask & TIOCM_RTS ? LINE_OUT_RTS : 0);
#else//elif CONFIG_ARCH_EZX_BARBADOS
			// It is the right way to do ioctl.
			mask = (mask & TIOCM_CD ? LINE_IN_DCD : 0) | 
				(mask & TIOCM_DSR ? LINE_IN_DSR : 0) |
				(mask & TIOCM_RI ? LINE_IN_RI : 0);
#endif
			switch(cmd) {
				case TIOCMSET: newctrl = mask; break;
				case TIOCMBIS: newctrl |= mask; break;
				case TIOCMBIC: newctrl &= ~mask; break;
			}
			RETURN_ZERO_IF(acm->ctrlin == newctrl);

			printk (KERN_INFO"%s: newctrl: %04x, mask 0x%x\n", __FUNCTION__, newctrl, mask);
			acm->ctrlin = newctrl;
			return acm_send_int_notification(function, CDC_NOTIFICATION_SERIAL_STATE, acm->ctrlin);
	}
#endif		// CONFIG_ARCH_EZX

	TRACE_MSG("--> -ENOIOCTLCMD");
        return -ENOIOCTLCMD;
}

static void acm_tty_set_termios(struct tty_struct *tty, struct termios *termios_old)
{
        struct acm_private *acm = &acm_private;
        struct termios *termios;

	TRACE_MSG("entered");
        //printk (KERN_INFO"%s: tty: %p\n", __FUNCTION__, tty);

        RETURN_IF(!atomic_read(&acm->used) || !tty || !tty->termios);

        termios = tty->termios;
        acm->clocal = termios->c_cflag & CLOCAL;

        //printk (KERN_INFO"%s: clocal: %d\n", __FUNCTION__, acm->clocal);
	TRACE_MSG("exited");
}

/* ********************************************************************************************* */

static void acm_wakeup_writers(void *private)
{
        struct acm_private *acm = &acm_private;
        struct tty_struct *tty = acm->tty;

	TRACE_MSG("entered");
        MOD_DEC_USE_COUNT;
        
        //printk (KERN_INFO"%s: MOD: %d connected: %d tty: %p\n", __FUNCTION__, MOD_IN_USE, acm->connected, tty);

        RETURN_IF(!acm->connected || !atomic_read(&acm->used) || !tty);
	TRACE_MSG("connected and used OK");

        if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) && tty->ldisc.write_wakeup) {
		TRACE_MSG("ldisc wakeup");
                (tty->ldisc.write_wakeup)(tty);
	}
        
        // printk (KERN_INFO"%s: write_wait wakeup\n", __FUNCTION__);
        wake_up_interruptible(&tty->write_wait);
	TRACE_MSG("exited");
}


/* ********************************************************************************************* */

static int acm_tty_refcount;
static struct tty_struct *acm_tty_table[ACM_TTY_MINORS];
static struct termios *acm_tty_termios[ACM_TTY_MINORS];
static struct termios *acm_tty_termios_locked[ACM_TTY_MINORS];

static struct tty_driver acm_tty_driver = {
        magic:                  TTY_DRIVER_MAGIC,
        type:                   TTY_DRIVER_TYPE_SERIAL,
        subtype:                SERIAL_TYPE_NORMAL,
        flags:                  TTY_DRIVER_REAL_RAW | TTY_DRIVER_RESET_TERMIOS, //TTY_DRIVER_NO_DEVFS 
        driver_name:            "acm-CDC",
        name:                   "usb/acm/%d",
        major:                  ACM_TTY_MAJOR,
        num:                    ACM_TTY_MINORS,
        minor_start:            0, 

        open:                   acm_tty_open,
        close:                  acm_tty_close,
        write:                  acm_tty_write,
        write_room:             acm_tty_write_room,
        ioctl:                  acm_tty_ioctl,
        throttle:               acm_tty_throttle,
        unthrottle:             acm_tty_unthrottle,
        chars_in_buffer:        acm_tty_chars_in_buffer,
        set_termios:            acm_tty_set_termios,

        refcount:               &acm_tty_refcount,
        table:                  acm_tty_table,
        termios:                acm_tty_termios,
        termios_locked:         acm_tty_termios_locked,
};      


/* Transmit INTERRUPT ************************************************************************** */

/**
 * Generates a response urb on the notification (INTERRUPT) endpoint.
 * Return a non-zero result code to STALL the transaction.
 * CALLED from interrupt context.
 */
static int acm_send_int_notification(struct usb_function_instance *function, int bnotification, int data)
{
	struct acm_private *acm = &acm_private;
        struct urb *urb = NULL;
        struct cdc_notification_descriptor *notification;
        unsigned long flags;
        int rc;

        //printk(KERN_INFO"%s: function: %p connected: %d DTR: %d\n", __FUNCTION__, 
        //                function, acm->connected, acm->ctrlout & LINE_OUT_DTR);

        local_irq_save(flags);
	TRACE_MSG("entered");

        do {
                BREAK_IF(!function);
                //BREAK_IF(!acm->connected || !acm->ctrlout & LINE_OUT_DTR);
                BREAK_IF(!acm->connected);

                //uuu printk(KERN_INFO"%s:\n", __FUNCTION__);
                #ifdef CONFIG_USBD_NM
                   BREAK_IF(!(urb = usbd_alloc_urb (function, get_acm_interrupt_endpoint(), 
                                                    sizeof(struct cdc_notification_descriptor), acm_urb_sent_int))); 
                #else 
                    BREAK_IF(!(urb = usbd_alloc_urb (function, INT_IN,
                                                     sizeof(struct cdc_notification_descriptor), acm_urb_sent_int)));
                                
                #endif
                //printk(KERN_INFO"%s: BBB\n", __FUNCTION__);

                memset(urb->buffer, 0, urb->buffer_length);
                urb->actual_length = sizeof(struct cdc_notification_descriptor);
                urb->privdata = &acm_private;

                // fill in notification structure
                notification = (struct cdc_notification_descriptor *) urb->buffer;

                notification->bmRequestType = USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE;
                notification->bNotification = bnotification;

                switch (bnotification) {
                case CDC_NOTIFICATION_NETWORK_CONNECTION:
                        notification->wValue = data;
                        break;
                case CDC_NOTIFICATION_SERIAL_STATE:
                        notification->wLength = cpu_to_le16(2);
                        *((unsigned short *)notification->data) = cpu_to_le16(data);
                        break;
                }

                //printk(KERN_INFO"%s: CCC\n", __FUNCTION__);
                BREAK_IF(!(rc = usbd_send_urb (urb)));

                //uuu printk(KERN_ERR"%s: usbd_send_urb failed err: %x\n", __FUNCTION__, rc);
                urb->privdata = NULL;
                usbd_dealloc_urb (urb);

        } while(0);

	TRACE_MSG1("urbID#%04x --> 0",urbID(urb));
        local_irq_restore(flags);
        //printk(KERN_INFO"%s: DDD\n", __FUNCTION__);
        return 0;
}

/* Callback functions for TX urb completion (function chosen when urb is allocated) ***********/

/* acm_urb_sent_bulk - called to indicate URB transmit finished
 * @urb: pointer to struct urb
 * @rc: result
 */
static int acm_urb_sent_bulk (struct urb *urb, int rc)
{
	struct acm_private *acm = &acm_private;
        struct usb_function_instance *function;

	acm_interrupts++;
	TRACE_MSG2("entered urbID#%04x rc=%d",urbID(urb),rc);

	if (!urb || !(function = urb->function_instance)) {
		TRACE_MSG1("urbID#%04x --> -EINVAL",urbID(urb));
		return(-EINVAL);
	}
	TRACE_MSG1("IN length=%d",urb->actual_length);

        atomic_sub(urb->actual_length, &acm->queued_bytes);
        atomic_dec(&acm->queued_urbs);
        urb->privdata = NULL;
        usbd_dealloc_urb (urb);
        acm_schedule(&acm->wqueue);
	TRACE_MSG1("urbID#%04x --> 0",urbID(urb));
        return 0;
}

/* acm_urb_sent_int - called to indicate URB transmit finished
 * @urb: pointer to struct urb
 * @rc: result
 */
static int acm_urb_sent_int (struct urb *urb, int rc)
{
        struct acm_private *acm = &acm_private;
        struct usb_function_instance *function;

	acm_interrupts++;
	TRACE_MSG2("entered urbID#%04x rc=%d",urbID(urb),rc);
        //printk(KERN_INFO"%s: urb: %p function: %p rc: %d\n", __FUNCTION__, urb, urb->function_instance, rc);
	if (!urb || !(function = urb->function_instance)) {
		TRACE_MSG1("urbID#%04x --> -EINVAL",urbID(urb));
		return(-EINVAL);
	}
	TRACE_MSG1("INT length=%d",urb->actual_length);

        usbd_dealloc_urb(urb);
	TRACE_MSG1("urbID#%04x --> 0",urbID(urb));
        return 0;
}

/* USB Device Functions ************************************************************************ */

/* acm_event_irq - process a device event
 *
 */


void acm_event_irq (struct usb_function_instance *function, usb_device_event_t event, int data)
{
        struct acm_private *acm = &acm_private;
        int i;

	acm_interrupts++;
	TRACE_MSG1("entered ev=%d",event);
	switch (event) {
	case DEVICE_CONFIGURED:
		TRACE_MSG("CONFIGURED");
#ifdef CONFIG_MODEM_CONNECTIONS
		queue_motusbd_event(USB_ACM_READY);
		//printk (KERN_INFO "%s: modem_connection_status_inform_kernel OK\n", __FUNCTION__);
		modem_connection_status_inform_kernel(CONNECTION_USB, CONNECTION_OK);
#endif

                acm->connected = 1;
                acm_schedule(&acm->wqueue);
		acm_start_recv_urbs(acm->tty);
		//printk(KERN_INFO"%s: DEVICE_CONFIGURED finished\n", __FUNCTION__);
                break;

	case DEVICE_RESET:
	case DEVICE_DE_CONFIGURED:
	//case DEVICE_BUS_INACTIVE:		// Fix me: w20146 -- 
		TRACE_MSG("RESET");
                BREAK_IF(!acm->connected);
		TRACE_MSG("RESET continue");
                acm->connected = 0;

#ifdef CONFIG_MODEM_CONNECTIONS
		//printk (KERN_INFO "%s: modem_connection_status_inform_kernel broken\n", __FUNCTION__);
		modem_connection_status_inform_kernel(CONNECTION_USB, CONNECTION_BROKEN);
#endif

		//printk(KERN_INFO"%s: connected %d, event %d\n", __FUNCTION__, acm->connected, event);
                acm_schedule(&acm->hqueue);
                // XXX flush
		// Release any queued urbs
		break;

        default: break;
	}

	TRACE_MSG("exited");
}

static int acm_recv_urb (struct urb *urb, int rc)
{
	/* Return 0 if urb has been accepted,
	 * return 1 and expect caller to deal with urb release otherwise. */
	struct acm_private *acm = &acm_private;
	struct tty_struct *tty = acm->tty;
	struct urb *furb;
	unsigned long flags;
	int i;
	u8 *cp;

	acm_interrupts++;

	local_irq_save(flags);
	acm->recv_urbs--;               // this could probably be atomic operation....
	local_irq_restore(flags);

	//printk(KERN_INFO"%s: urb: %p rc=%d\n", __FUNCTION__, urb, rc);
	if (RECV_CANCELLED == rc) {
		TRACE_MSG1("cancelled URB=%p",urb);
		return -EINVAL;
	}
	if (!tty) {
		TRACE_MSG1("no tty URB=%p",urb);
		return -EINVAL;
	}
	if (RECV_OK != rc) {
		TRACE_MSG1("rejected URB=%p",urb);
		usbd_dealloc_urb(urb);
		acm_start_recv_urbs(tty);
		return(0);
	}
	
/* acm_start_recv_urbs() will never queue more urbs than there is currently
 * * room in the flip buffer for. So we are guaranteed that any data actually
 * * received can be given to the upper layers without worrying if we will 
 * * actually have room. 
 * *
 * * XXX I think that if the tty layer does get behind and we reach a point where
 * * there are no outstanding receive urbs, then we will also have been throttled
 * * so we will have an oppourtunity to queue more receive urbs when we get unthrottled.
 * *
 * * XXX If the above assumption is not true that a timer will be needed to periodically
 * * check if we can restart.
 * *
 * * YYY What can happen is that the tty_flip_buffer_push() call can fail for
 * * a variety of reasons (locked out while a read call is in progress, ldisc
 * * buffer is full, and probably others).  We need to ensure that we keep
 * * trying to push the flip buffer until it does go, and there is room for
 * * more receive urbs.  acm_start_recv_urbs() will queue a task to push and
 * * try again if it can't queue any now.
 * *
 * * ZZZ Yet another failure mode is to call tty_flip_buffer_push() too many
 * * times while throttled.  This will result in the ldisc layer silently
 * * dropping data inside the n_tty_receive_buf() routine.  (Acutal numbers
 * * for the 2.4.20 kernel this was discovered on are 128 bytes left in the
 * * ldisc buffer when throttle is called, and upto 7 64 byte urbs outstanding.
 * * The urbs will fit into the flip buffer, but NOT the ldisc buffer.)  This
 * * means we must not call tty_flip_buffer_push() when throttled.  We will
 * * count on the acm_tty_unthrottle() call to kick off the final push.
 * */
	cp = urb->buffer;
#define WORD_COPY 1
#if defined(WORD_COPY)
	i = urb->actual_length;
	local_irq_save(flags);
	memcpy(tty->flip.char_buf_ptr,cp,i);
	memset(tty->flip.flag_buf_ptr,TTY_NORMAL,i);
	tty->flip.count += i;
	tty->flip.char_buf_ptr += i;
	tty->flip.flag_buf_ptr += i;
	acm->bytes_forwarded += i;
	local_irq_restore(flags);
#else
	for (i = 0; i < urb->actual_length; i++) {
		/* tty_flip_char() has no lock out from any calls
		 * to tty_flip_buffer_push() that may have been queued
		 * by acm_unthrottle() or acm_start_recv_urbs(), so... */
		local_irq_save(flags);
		tty_insert_flip_char(tty, *cp++, TTY_NORMAL);
		local_irq_restore(flags);
	}
	acm->bytes_forwarded += urb->acutal_length;
#endif

	if (!acm->throttle) {
		tty_flip_buffer_push(tty);
	}

	acm->bytes_received += urb->actual_length;
	TRACE_MSG1("bytes_received: %d",acm->bytes_received);
	usbd_dealloc_urb(urb);

	acm_start_recv_urbs(tty);

	return(0);
}


/* acm_line_coding_urb_received - callback for sent URB
 *
 * Handles notification that an urb has been sent (successfully or otherwise).
 *
 * Returns non-zero for failure.
 */
static int acm_line_coding_urb_received (struct urb *urb, int urb_rc)
{
	TRACE_MSG2("urbID#%04x rc=%d",urbID(urb),urb_rc);

	RETURN_EINVAL_IF (RECV_OK != urb_rc);

	return -EINVAL;         // caller will de-allocate
}

/* acm_recv_setup_irq - called to indicate urb has been received
 */
int acm_recv_setup_irq (struct usb_device_request *request)
{
        struct acm_private *acm = &acm_private;
	struct usb_function_instance *function = acm->function;

	acm_interrupts++;
	TRACE_SETUP(request);

        // verify that this is a usb class request per cdc-acm specification or a vendor request.
        if (!(request->bmRequestType & (USB_REQ_TYPE_CLASS | USB_REQ_TYPE_VENDOR))) {
		TRACE_MSG("--> 0");
		return(0);
	}
       
        // determine the request direction and process accordingly
        switch (request->bmRequestType & (USB_REQ_DIRECTION_MASK | USB_REQ_TYPE_MASK)) {

        case USB_REQ_HOST2DEVICE | USB_REQ_TYPE_CLASS:
                switch (request->bRequest) {
                case CDC_CLASS_REQUEST_SEND_ENCAPSULATED: break;
                case CDC_CLASS_REQUEST_SET_COMM_FEATURE: break;
                case CDC_CLASS_REQUEST_CLEAR_COMM_FEATURE: break;
                case CDC_CLASS_REQUEST_SET_LINE_CODING:
			{
				struct urb *urb;
				int len = le16_to_cpu(request->wLength);
				TRACE_MSG1("SET_LINE_CODING wLength=%d",len);
				if (len <= 0) {
					TRACE_MSG("(len<=0)--> 0");
					return(0);
				}
				// Set up an ep0 recv urb for the rest of it.
				urb = usbd_alloc_urb_ep0(function, len, acm_line_coding_urb_received);
				if (NULL == urb) {
					TRACE_MSG("no mem for ep0 recv urb");
					return(-EINVAL);
				}
				if (usbd_start_recv(urb)) {
					TRACE_MSG("usbd_start_recv() failed");
					usbd_dealloc_urb(urb);          // de-alloc if error
					TRACE_MSG("--> -EINVAL");
					return(-EINVAL);
				}
                        }
                        break;
                case CDC_CLASS_REQUEST_SET_CONTROL_STATE:
                        {
                                struct acm_private *acm = &acm_private;
				unsigned int prev_ctrlout = acm->ctrlout;
                                acm->ctrlout = le16_to_cpu(request->wValue);
                                
				TRACE_MSG1("set control state, tty=%p",acm->tty);
                                //uuu printk(KERN_INFO"%s: tty: %p clocal: %x ctrlout: %02x DTR: %x\n", 
                                //uuu                 __FUNCTION__, acm->tty, acm->clocal, acm->ctrlout, acm->ctrlout & LINE_OUT_DTR);
                                
                                // schedule writers or hangup IFF open
                                BREAK_IF(!acm->tty);
				TRACE_MSG1("set control state, ctrlout#%04x",acm->ctrlout);
				// make sure there really is a state change
				if ((acm->ctrlout ^ prev_ctrlout) & LINE_OUT_DTR) {
					TRACE_MSG1("DTR state changed -> %x",(acm->ctrlout&LINE_OUT_DTR));
                                	//printk(KERN_INFO"%s: tty: %p DTR state changed -> %u\n", 
                                        //        __FUNCTION__, acm->tty, (acm->ctrlout&LINE_OUT_DTR));
#ifdef CONFIG_ARCH_EZXBASE
					if((acm->ctrlout & LINE_OUT_DTR))
					{
						printk(KERN_INFO"%s: DTR high, wake up wqueue\n", __FUNCTION__);
						acm_schedule(&acm->wqueue);
					}
					else
					{
						printk(KERN_INFO"%s: DTR low, no tty_hangup!!!\n", __FUNCTION__);
						if(kill_sl((acm->tty)->session, SIGHUP, 1) != 0)
							printk("error info: fail to send SIGHUP to session lead\n");
					}
#else
                                	acm_schedule(((acm->ctrlout & LINE_OUT_DTR) ? &acm->wqueue : &acm->hqueue));
#endif
					// wake up blocked opens
					if (acm->open_wait_count > 0) {
						wake_up_interruptible(&acm->open_wait);
					}
				} // end of state change operation

                                // send notification if we have DCD
				TRACE_MSG1("checking DCD ctrlin#%04x",acm->ctrlin);
                                BREAK_IF(!(acm->ctrlin & (LINE_IN_DCD | LINE_IN_DSR)));
				TRACE_MSG1("tty=%p sending (DCD|DSR) notification",acm->tty);
                                acm_send_int_notification(function, CDC_NOTIFICATION_SERIAL_STATE, acm->ctrlin);
                        }
                        break;

                case CDC_CLASS_REQUEST_SEND_BREAK: break;
                default: break;
                }
		TRACE_MSG("--> 0");
                return 0;

        case USB_REQ_DEVICE2HOST | USB_REQ_TYPE_CLASS:
                switch (request->bRequest) {
                case CDC_CLASS_REQUEST_GET_ENCAPSULATED: break;
                case CDC_CLASS_REQUEST_GET_COMM_FEATURE: break;
                case CDC_CLASS_REQUEST_GET_LINE_CODING: 
                        {
                                struct urb *urb;
                                struct cdc_acm_line_coding *results;
				int len = le16_to_cpu(request->wLength);
				if (len != sizeof(struct cdc_acm_line_coding)) {
					TRACE_MSG2("(len=%d!=sz=%d--> -EINVAL",len,sizeof(struct cdc_acm_line_coding));
                                	return -EINVAL;
				}

                                if (!(urb = usbd_alloc_urb_ep0(function, len, NULL))) {
					TRACE_MSG("(nomem)--> -EINVAL");
                                	return -EINVAL;
				}
                                results = (struct cdc_acm_line_coding *)urb->buffer;
#ifdef CONFIG_ARCH_EZX
                                results->dwDTERate = cpu_to_le32(0x1c200); // 115200
#else
				results->dwDTERate = cpu_to_le16(0x1c200); // 115200
#endif
                                results ->bDataBits = 0x08;
                                urb->actual_length = len;
				TRACE_MSG1("sending line coding urb=%p",(u32)(void*)urb);
                                if (!usbd_send_urb(urb)) {
					TRACE_MSG("--> 0");
					return(0);
				}
                                usbd_dealloc_urb(urb);
				TRACE_MSG("(send failed)--> -EINVAL");
                                return -EINVAL;
                        }
                default: break;
                }
		TRACE_MSG("--> 0");
                return 0;

        default: break;
        }
	TRACE_MSG("--> 0");
        return 0;
}

int acm_function_enable (struct usb_function_instance *function)
{
        struct acm_private *acm = &acm_private;
	acm_interrupts++;
	TRACE_MSG("entered");
	MOD_INC_USE_COUNT;
        acm->function = function;

	if ( usbd_high_speed(function) )
		acm->writesize = usbd_endpoint_wMaxPacketSize ( function, BULK_OUT, 1 );
	else
	        acm->writesize = usbd_endpoint_wMaxPacketSize ( function, BULK_OUT, 0 ) * 4;

	TRACE_MSG("-> 0");
        return 0;
}

void acm_function_disable (struct usb_function_instance *function)
{
        struct acm_private *acm = &acm_private;
	acm_interrupts++;
	TRACE_MSG("entered");
        acm->writesize = 0;
        acm->function = NULL;
        MOD_DEC_USE_COUNT;
	TRACE_MSG("exited");
}


static struct usb_function_operations function_ops = {
	event_irq: acm_event_irq,
	recv_setup_irq: acm_recv_setup_irq,
        function_enable: acm_function_enable,
        function_disable: acm_function_disable,
};

static struct usb_function_driver function_driver = {
	name: "acm-CDC",
	fops:&function_ops,
	device_description:&acm_device_description,
	bNumConfigurations:sizeof (acm_description) / sizeof (struct usb_configuration_description),
	configuration_description:acm_description,
	idVendor: __constant_cpu_to_le16(CONFIG_USBD_ACM_VENDORID),
	idProduct: __constant_cpu_to_le16(CONFIG_USBD_ACM_PRODUCTID),
	bcdDevice: __constant_cpu_to_le16(CONFIG_USBD_ACM_BCDDEVICE),
};


#if defined(USE_TICKER)
/* usb ticker ********************************************************************************** */

#define retrytime 10
                
int ticker_terminating;
int ticker_timer_set;

static DECLARE_MUTEX_LOCKED (ticker_sem_start);
static DECLARE_MUTEX_LOCKED (ticker_sem_work);

void ticker_tick (unsigned long data)
{
        ticker_timer_set = 0;
        up (&ticker_sem_work);
}

void udc_ticker_poke (void)
{
        up (&ticker_sem_work);
}

int ticker_thread (void *data)
{
        struct timer_list ticker;
        // detach
        lock_kernel ();
        exit_mm (current);
        exit_files (current);
        exit_fs (current);

        // setsid equivalent, used at start of kernel thread, no error checks needed, or at least none made :). 
        current->leader = 1;
        current->session = current->pgrp = current->pid;
        current->tty = NULL;
        current->tty_old_pgrp = 0;
        sprintf (current->comm, "acm_fd");

        // setup signal handler
        current->exit_signal = SIGCHLD;
        spin_lock (&current->sigmask_lock);
        flush_signals (current);
        spin_unlock (&current->sigmask_lock);

        // run at a high priority, ahead of sync and friends
        current->policy = SCHED_OTHER;
        unlock_kernel ();

        // setup timer
        init_timer (&ticker);
        ticker.data = 0;
        ticker.function = ticker_tick;

        // let startup continue
        up (&ticker_sem_start);
        
        // process loop
        for (ticker_timer_set = ticker_terminating = 0; !ticker_terminating;) {

                struct acm_private * acm = &acm_private;

                if (!ticker_timer_set) {
                        mod_timer (&ticker, jiffies + HZ * retrytime);
                }

                // wait for someone to tell us to do something
                down (&ticker_sem_work);

                // sanity checks before proceeding
                BREAK_IF(ticker_terminating);
                CONTINUE_IF(!(function = acm->function));
                CONTINUE_IF(USBD_OK != bus->status);

                // do what we need to do
                acm_send_int_notification(function, CDC_NOTIFICATION_SERIAL_STATE, acm->ctrlin);
        }

        // remove timer, let the process stopping us know we are done and return
        del_timer (&ticker);
        up (&ticker_sem_start);
        return 0;
}
#endif


/* USB Module init/exit ************************************************************************ */
/*
 * acm_modinit - module init
 *
 */
static int acm_modinit (void)
{
	int i;
        printk (KERN_INFO "Copyright (c) 2003-2004 sl@belcarra.com\n");

	if (0 != acm_trace_init(ACM_TRACE_NAME)) {
                printk(KERN_ERR"%s: ERROR tracing configured, but init failed.\n", __FUNCTION__);
                return -EINVAL;
	}
	TRACE_MSG("entered");

        if (vendor_id) 
                function_driver.idVendor = cpu_to_le16(vendor_id);
        if (product_id) 
                function_driver.idProduct = cpu_to_le16(product_id);
	printk (KERN_INFO "%s: %s vendor_id: %04x product_id: %04x\n", __FUNCTION__,
		 __usbd_module_info, function_driver.idVendor, function_driver.idProduct);

        // initialize private structure
        acm_private.tty_driver = &acm_tty_driver;
        acm_private.wqueue.routine = acm_wakeup_writers;
        acm_private.wqueue.data = &acm_private;
        acm_private.hqueue.routine = acm_hangup;
        acm_private.hqueue.data = &acm_private;
	acm_private.recv_tqueue.routine = &acm_start_recv;
	acm_private.recv_tqueue.data = &acm_private;

        init_waitqueue_head(&acm_private.open_wait);

        // register as tty driver
        acm_tty_driver.init_termios = tty_std_termios;
        acm_tty_driver.init_termios.c_cflag = B115200 | CS8 | CREAD | HUPCL | CLOCAL;
        acm_tty_driver.init_termios.c_lflag = 0;	//&= ~(ECHO | ICANON);
	acm_tty_driver.init_termios.c_iflag = 0;
	acm_tty_driver.init_termios.c_oflag = 0;
        THROW_IF(tty_register_driver(&acm_tty_driver), error);

        tty_register_devfs(&acm_tty_driver, 0, ACM_TTY_MINOR);
        acm_private.tty_driver_registered++;

	// register as usb function driver
	THROW_IF (usbd_register_function (&function_driver), error);
        acm_private.usb_driver_registered++;
	
#if defined(USE_TICKER)
        // kickoff_thread - start management thread
        //ticker_terminating = 0;
        //kernel_thread (&ticker_thread, NULL, 0);
        //down (&ticker_sem_start);
#endif

        CATCH(error) {
                printk(KERN_ERR"%s: ERROR\n", __FUNCTION__);

                if (acm_private.tty_driver_registered) {
                        tty_unregister_driver(&acm_tty_driver);
                        acm_private.tty_driver_registered = 0;
                }
                if (acm_private.usb_driver_registered) {
                        usbd_deregister_function (&function_driver);
                        acm_private.usb_driver_registered = 0;
                }
		TRACE_MSG("--> -EINVAL");
                return -EINVAL;
        }
	TRACE_MSG("--> 0");
	return 0;
}

void acm_wait_task(struct tq_struct *queue)
{
	TRACE_MSG1("entered data=%p",queue->data);
        RETURN_IF(!queue->data);
        queue->data = NULL;
        while (queue->sync) {
                //uuu printk(KERN_INFO"%s: waiting for queue: %p\n", __FUNCTION__, queue);
		TRACE_MSG1("waiting for queue: %p",queue);
                schedule_timeout(HZ);
        }
	TRACE_MSG("exited");
}

/* acm_modexit - module cleanup
 */
static void acm_modexit (void)
{
	unsigned long flags;
	struct urb *urb;
	TRACE_MSG("entered");

#if defined(USE_TICKER)
        // killoff_thread - stop management thread
        //if (!ticker_terminating) {
        //        ticker_terminating = 1;
        //        up (&ticker_sem_work);
        //        down (&ticker_sem_start);
        //}
#endif

	// Wake up any pending opens after setting the exiting flag.
	local_irq_save(flags);
	acm_private.exiting = 1;
	if (acm_private.open_wait_count > 0) {
		wake_up_interruptible(&acm_private.open_wait);
	}
	local_irq_restore(flags);

        // verify no tasks are running
        acm_wait_task(&acm_private.wqueue);
        acm_wait_task(&acm_private.hqueue);

	run_task_queue(&tq_timer);
	acm_wait_task(&acm_private.recv_tqueue);
	
        // de-register as tty  and usb drivers
        if (acm_private.tty_driver_registered) {
                tty_unregister_driver(&acm_tty_driver);
        }
        if (acm_private.usb_driver_registered) {
                usbd_deregister_function (&function_driver);
       }

	acm_trace_exit(ACM_TRACE_NAME);
}
#ifdef CONFIG_USBD_NM
int get_acm_interrupt_endpoint(void)
{
    
    if(motusbd_status==USB_STATUS_NM)
    {
        return NM_NEED_INT_IN;
    }
    return INT_IN;
}
#endif

module_init (acm_modinit);
module_exit (acm_modexit);
