/*
 * linux/drivers/usbd/l7205_bi/udc.c -- L7205 USB controller driver. 
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
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


// Debug enabling defines
//#define TRIGGER       1
//#define FLAG_F1ERR    1
// End of debug enabling defines

#include <linux/config.h>
#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>

#include <asm/atomic.h>
#include <asm/io.h>

#include <linux/proc_fs.h>
#include <linux/tqueue.h>

#include <linux/netdevice.h>
#include <linux/version.h>
#include <linux/pci.h>
#include <linux/cache.h>

#include <asm/dma.h>
#include <asm/mach/dma.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/hardware.h>

#include <asm/types.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>

#include <linux/delay.h>

#include "../usbd.h"
#include "../usbd-func.h"
#include "../usbd-bus.h"
#include "../usbd-inline.h"
#include "usbd-bi.h"

#ifdef CONFIG_IRIS
#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>
#include <asm/arch/fpga.h>
#include <asm/arch/iris_serialno.h>
#endif

#include "hardware.h"
#include "l7205.h"
#include "udc.h"

int sof_saw_tx_active;
int sof_saw_rx_f1ne;		// track if we have seen F1NE at SOF
int host_sus_interrupt_disabled;	// set if SUS interrupt received and disabled, waiting for SOF
int host_reset_interrupt_disabled;	// set if HRST interrupt received and disabled, waiting for SOF
int ep2_active;			// set if transmit on ep2 is active, used to track missing interrupts

extern unsigned int udc_interrupts;
extern unsigned int udc_interrupts_last;

static struct usb_device_instance *udc_device;	// for interrupt handler

struct usb_device_instance *ep1_device;
struct usb_endpoint_instance *ep1_endpoint;


#ifdef CONFIG_IRIS
#define USB_PULLUP_GPIO bitPB6
// USB pullup resistor control.
// USB cable and AC power connect status
extern int iris_read_usb_sync (void);
extern int iris_read_ac_sync (void);
extern void iris_AUXPLL_ON_for_usb (void);
extern void iris_AUXPLL_OFF_for_usb (void);
#endif

/*
 * ep_endpoints - map physical endpoints to logical endpoints
 */
static struct usb_endpoint_instance *ep_endpoints[UDC_MAX_ENDPOINTS];

struct usb_device_instance *ep2_device;
struct usb_endpoint_instance *ep2_endpoint;

#define MIN(a,b) ((a)<(b))?(a):(b)

/* ep1 functions ******************************************************************************* */

/**
 * ep1_clear
 *
 * Clear ep1 fifo by reading all data and resetting F1BCNT to 32
 */
static /* __inline__ */ void ep1_clear (void)
{
	int j;
	volatile unsigned long junk;
	for (j = 0; j < 8; j++) {
		junk = IO_USBF_FIFO1;
	}
	IO_USBF_F1BCNT = 32;
}

/*
 * Endpoint 1 interrupts will be directed here.
 *
 */
static __inline__ void ep1_int_hndlr (void)
{
	if (ep1_endpoint) {

		if (!ep1_endpoint->rcv_urb) {

			ep1_endpoint->rcv_urb = first_urb_detached (&ep1_endpoint->rdy);

			// XXX could we call usbd_fill_rcv() here?
		}

		if (ep1_endpoint->rcv_urb) {

			int len = IO_USBF_F1BCNT;
			unsigned long *lp =
			    (unsigned long *) (ep1_endpoint->rcv_urb->buffer +
					       ep1_endpoint->rcv_urb->actual_length);

			// copy 4 bytes at a time, fetching more than available seems ok, use unrolled loop
			lp[0] = IO_USBF_FIFO1;
			lp[1] = IO_USBF_FIFO1;
			lp[2] = IO_USBF_FIFO1;
			lp[3] = IO_USBF_FIFO1;

			lp[4] = IO_USBF_FIFO1;
			lp[5] = IO_USBF_FIFO1;
			lp[6] = IO_USBF_FIFO1;
			lp[7] = IO_USBF_FIFO1;

			usbd_rcv_complete_irq (ep1_endpoint, len, 0);

			// reset F1BCNT
			IO_USBF_F1BCNT = 32;
			return;
		}
		// fall through if error of any type
		usbd_rcv_complete_irq (ep1_endpoint, 0, 0);
	}
	ep1_clear ();
}

/*
 * Endpoint 1 interrupts will be directed here if there was an error.
 *
 */
void ep1_int_hndlr_error (void)
{
#if 1
	int i;
	volatile unsigned char *cp = (volatile unsigned char *) IO_USBF_FIFO_RX;
	printk (KERN_DEBUG "ep1_int_hndlr: ERROR ");
	for (i = 0; i < 32; i++) {
		printk ("%02x ", cp[i]);
	}
	printk ("\n");
#endif

	if (ep1_endpoint) {
		usbd_rcv_complete_irq (ep1_endpoint, 0, 0);
	}
	ep1_clear ();
}

/* ep2 functions ******************************************************************************* */

static int __inline__ ep2_fill (unsigned char *cp, int len, int delay)
{
	unsigned long *buf = (unsigned long *) cp;
	volatile unsigned long *p2 = (volatile unsigned long *) IO_USBF_FIFO_TX;

	// unrolled loop to fill fifo
	IO_USBF_FIFO2 = buf[0];
	IO_USBF_FIFO2 = buf[1];
	IO_USBF_FIFO2 = buf[2];
	IO_USBF_FIFO2 = buf[3];
	IO_USBF_FIFO2 = buf[4];
	IO_USBF_FIFO2 = buf[5];
	IO_USBF_FIFO2 = buf[6];
	IO_USBF_FIFO2 = buf[7];

	if ((buf[0] != p2[0]) || (buf[1] != p2[1]) || (buf[2] != p2[2]) || (buf[3] != p2[3]) ||
	    (buf[4] != p2[4]) || (buf[5] != p2[5]) || (buf[6] != p2[6]) || (buf[7] != p2[7])) {
		udelay (8);	// 
		// reset fifo read pointer
		l7205_toggle (IO_USBF_CONTROL, USBF_CONTROL_F2CLR);

		// unrolled loop to fill fifo
		IO_USBF_FIFO2 = buf[0];
		IO_USBF_FIFO2 = buf[1];
		IO_USBF_FIFO2 = buf[2];
		IO_USBF_FIFO2 = buf[3];
		IO_USBF_FIFO2 = buf[4];
		IO_USBF_FIFO2 = buf[5];
		IO_USBF_FIFO2 = buf[6];
		IO_USBF_FIFO2 = buf[7];

		return (buf[0] != p2[0]) || (buf[1] != p2[1]) || (buf[2] != p2[2])
		    || (buf[3] != p2[3]) || (buf[4] != p2[4]) || (buf[5] != p2[5])
		    || (buf[6] != p2[6]) || (buf[7] != p2[7]);
	}
	return 0;
}

/**
 * ep2_start
 *
 * Fill ep2 fifo with data.
 */
static void ep2_start (void)
{
	int len;

	if (ep2_endpoint->tx_urb &&
	    (len = MIN ((ep2_endpoint->tx_urb->actual_length - ep2_endpoint->sent),
			ep2_endpoint->tx_urb->endpoint->tx_packetSize))) {
		ep2_active = 1;

		// fill the FIFO, if this fails send a short packet to end of this
		// attempt of the BULK transfer and force the restart of this urb at
		// th beginning, the host will see a CRC error and drop the
		// intermediate results

		if (ep2_fill (ep2_endpoint->tx_urb->buffer + ep2_endpoint->sent, len, 0)) {

			// ep2_fill failed, reset bulk transfer and decrement len to
			// send current usb packet one byte short to guarantee host will
			// terminate bulk transfer and it will fail CRC check

			IO_USBF_F2BCNT = len - 1;
			ep2_endpoint->last = ep2_endpoint->sent = 0;
		} else {
			ep2_endpoint->last = IO_USBF_F2BCNT = len;
		}
	} else {
		ep2_active = 0;
	}
}

/**
 * ep2_int_hndlr - transmit interrupt handler, fast version
 */
static __inline__ void ep2_int_hndlr (unsigned int status)
{
	// if we have active buffer, umap the buffer and update position if previous send was successful
	if (!(status & (USBF_STATUS_F2ERR | USBF_STATUS_F2BSY))) {
		if (ep2_endpoint->tx_urb) {

			ep2_endpoint->sent += ep2_endpoint->last;
			ep2_endpoint->last = 0;

			// if current buffer is finished call urb sent and advance to next urb 
			if ((ep2_endpoint->tx_urb->actual_length - ep2_endpoint->sent) <= 0) {
				usbd_urb_sent_irq (ep2_endpoint->tx_urb, SEND_FINISHED_OK);
				ep2_endpoint->tx_urb = NULL;
			}

			ep2_start ();
		}
	}
}

/**
 * ep2_int_hndlr_error - transmit interrupt handler, slow version
 */
static void ep2_int_hndlr_error (unsigned int status, int error, int skip)
{
	if (!skip) {
		if ((status & (USBF_STATUS_F2ERR | USBF_STATUS_F2BSY | USBF_STATUS_F2NE)) || error) {

			if (!(status & USBF_STATUS_F2BSY)) {

				dbg_tx (0, "[%d]: F2ERR %08lx", udc_interrupts, IO_USBF_STATUS);

				l7205_toggle (IO_USBF_CONTROL, USBF_CONTROL_F2CLR);
			}
		} else {
			ep2_int_hndlr (status);
		}
	}
}

/*
 * l7205
 */
void l7205_dump_fifos (char *msg, int max)
{
	int i;

	dbg_udc (9, " ");
	dbg_udc (9, "*********** %s ***********", msg);

	if (7 <= dbgflg_usbdbi_udc) {
		for (i = 0; i < max; i++) {
			if ((i % 32) == 0) {
				printk ("\nFF[%02x]: ", i);
			}
			printk ("%02x ", __IOB (USBF_FIFOS + i));
		}
		printk ("\n");
	}
}

void l7205_regs (char *msg)
{
	if (1 <= dbgflg_usbdbi_udc) {
		int i;
		printk (KERN_DEBUG "\n");
		printk (KERN_DEBUG "*********** %s ***********\n", msg);
		printk (KERN_DEBUG "\n");
		printk (KERN_DEBUG "udc: Rev:  %08lx Ctl:  %08lx Sts:  %08lx\n",
			IO_USBF_REVISION, IO_USBF_CONTROL, IO_USBF_STATUS);
		printk (KERN_DEBUG "udc: IntE: %08lx IntD: %08lx Ep0:  %08lx\n",
			IO_USBF_INTENA, IO_USBF_INTDIS, IO_USBF_ENDPTBUF0);
		printk (KERN_DEBUG "udc: Ep1:  %08lx Ep2:  %08lx Ep3:  %08lx\n",
			IO_USBF_ENDPTBUF1, IO_USBF_ENDPTBUF2, IO_USBF_ENDPTBUF3);
		printk (KERN_DEBUG "udc: CFG:  %08lx ST0:  %08lx ST1:  %08lx\n",
			IO_USBF_CONFIGBUF1, IO_USBF_STRINGBUF0, IO_USBF_STRINGBUF1);
		printk (KERN_DEBUG "udc: ST2:  %08lx ST3:  %08lx ST4:  %08lx\n",
			IO_USBF_STRINGBUF2, IO_USBF_STRINGBUF3, IO_USBF_STRINGBUF4);
		printk (KERN_DEBUG "\n");

		printk (KERN_DEBUG "CLK ENA: %08lx AUX: %08lx SEL: %08lx\n",
			IO_SYS_CLOCK_ENABLE, IO_SYS_CLOCK_AUX, IO_SYS_CLOCK_SELECT);
		printk (KERN_DEBUG "\n");

		printk (KERN_DEBUG
			"udc: CFG:  %02lx:%02lx    ST0:  %02lx:%02lx    ST1:  %02lx:%02lx\n",
			__IOL (USBF_CONFIGBUF1) >> 9, __IOL (USBF_CONFIGBUF1) & 0x1ff,
			__IOL (USBF_STRINGBUF0) >> 9, __IOL (USBF_STRINGBUF0) & 0x1ff,
			__IOL (USBF_STRINGBUF1) >> 9, __IOL (USBF_STRINGBUF1) & 0x1ff);

		printk (KERN_DEBUG
			"udc: ST2:  %02lx:%02lx    ST3:  %02lx:%02lx    ST4:  %02lx:%02lx\n",
			__IOL (USBF_STRINGBUF2) >> 9, __IOL (USBF_STRINGBUF2) & 0x1ff,
			__IOL (USBF_STRINGBUF3) >> 9, __IOL (USBF_STRINGBUF3) & 0x1ff,
			__IOL (USBF_STRINGBUF4) >> 9, __IOL (USBF_STRINGBUF4) & 0x1ff);

		for (i = 0; i < 168; i++) {
			if ((i % 32) == 0) {
				printk ("\nDS[%02x]: ", i);
			}
			printk ("%02x ", __IOB (USBF_DESCRIPTORS + i));
		}
		printk ("\n");
		printk ("\n");
	}
}

void udc_regs (void)
{
	if (1 <= dbgflg_usbdbi_tick) {
		printk (KERN_DEBUG
			"%u Ctl: %08lx Sts: %08lx Raw: %08lx Ena: %08lx EP0: %08lx EP1: %08lx EP2: %08lx "
			"int: %2d\n",
			udc_interrupts,
			IO_USBF_CONTROL, IO_USBF_STATUS, IO_USBF_RAWSTATUS, IO_USBF_INTENA,
			IO_USBF_F0BCNT, IO_USBF_F1BCNT, IO_USBF_F2BCNT,
			udc_interrupts - udc_interrupts_last);
		udc_interrupts_last = udc_interrupts;
	}
}


/*
 * l7205_copy_descriptor
 *
 * Copy descriptor to shared SRAM and optionally set descriptor register to point
 * to where it is located.
 *
 * Note that reg CAN be NULL (there is no device descriptor register it is 
 * always at zero offset)
 */
static int l7205_copy_descriptor (unsigned char *descriptors, volatile unsigned long reg,
				  int offset, struct usb_descriptor *descriptor, int size)
{
	// set default 
	if (reg) {
		__IOL (reg) = 0;
	}

	if (!descriptors || !descriptor) {
		return offset;
	}

	if (!size) {
		if (!(size = descriptor->descriptor.generic.bLength)) {
			return offset;
		}
	}

	if (((offset + size) >= (USBF_DESCRIPTORS_MAX - 4))) {
		dbg_udc (0, "string too large to copy: offset: %x size: %d", offset, size);
		if (reg) {
			__IOL (reg) = (4 << 9) | (USBF_DESCRIPTORS_MAX - 4);
		}
		descriptors[USBF_DESCRIPTORS_MAX - 4] = 4;
		descriptors[USBF_DESCRIPTORS_MAX - 3] = 3;
		descriptors[USBF_DESCRIPTORS_MAX - 2] = 'A';
		descriptors[USBF_DESCRIPTORS_MAX - 1] = 0;
		return offset;
	}

	dbg_udc (3, "reg: %08lx offset: %02x descriptor: %p size: %02x", reg, offset, descriptor,
		 size);

	// set size and offset if we have a register
	if (reg) {
		__IOL (reg) = (size << 9) | offset;
		dbg_udc (3, "reg: %8lx %8lx size: %2lx off: %2lx", reg, __IOL (reg),
			 __IOL (reg) >> 9, __IOL (reg) & 0x1ff);
	}

	memcpy (descriptors + offset, descriptor, size);

	return (offset + size + 3) & 0x1fc;	// 4
}


static int l7205_copy_request (unsigned char *descriptors, struct usb_device_instance *device,
			       volatile unsigned long reg, int offset, int request)
{
	struct urb *udc_urb;
	usb_device_state_t device_state;

	/*
	 * fake a setup request to get descriptors
	 */
	if (!device || !(udc_urb = usbd_alloc_urb (device, NULL, 0, 512))) {
		dbg_udc (0, "cannot alloc urb");
		return offset;
	}

	udc_urb->device_request.bmRequestType = 0x80;
	udc_urb->device_request.bRequest = USB_REQ_GET_DESCRIPTOR;
	udc_urb->device_request.wValue = cpu_to_le16 ((request << 8));
	udc_urb->device_request.wLength = cpu_to_le16 (200);	// max 168?

	device_state = device->device_state;
	device->device_state = STATE_ADDRESSED;

	if (usbd_recv_setup (udc_urb)) {
		dbg_udc (0, "cannot process setup request");
		usbd_dealloc_urb (udc_urb);
		return offset;
	}

	device->device_state = device_state;
	offset =
	    l7205_copy_descriptor (descriptors, reg, offset,
				   (struct usb_descriptor *) udc_urb->buffer,
				   udc_urb->actual_length);
	usbd_dealloc_urb (udc_urb);

	return offset;
}

/**
 * udc_init - initialize L7205 USB Controller
 * 
 * Get ready to use the L7205 USB Controller.
 *
 * Register an interrupt handler and initialize dma.
 */
int udc_init (void)
{
	// This should never happen!
	if (IO_USBF_REVISION != USBF_REVISION_11) {
		dbg_udc (0, "incorrect or not found version: %02lx should be %02x",
			 IO_USBF_REVISION, USBF_REVISION_11);
		return -EINVAL;
	}
	// disable the udc just in case
	udc_disable_interrupts (0);
	udc_disconnect ();
	udc_disable ();
	return 0;
}

/*
 * Switch off the L7205 USB Function
 */
void udc_disable (void)
{
	udc_device = NULL;

	// disable interrupts
	IO_USBF_INTDIS =
	    USBF_INTDIS_DNSUS |
	    USBF_INTDIS_DNF0ERR |
	    USBF_INTDIS_DNF1OR |
	    USBF_INTDIS_DNF1ERR |
	    USBF_INTDIS_DNF2UR |
	    USBF_INTDIS_DNF2ERR |
	    USBF_INTDIS_DNF3ERR |
	    USBF_INTDIS_DNGRSM |
	    USBF_INTDIS_DNF0RQ |
	    USBF_INTDIS_DNF1RQ |
	    USBF_INTDIS_DNF2RQ | USBF_INTDIS_DNF3RQ | USBF_INTDIS_DNSOF | USBF_INTDIS_DNHRST;

	// set USBFUNC_EN
	IO_SYS_CLOCK_ENABLE &= ~SYS_CLOCK_USBFUNC_EN;

	// reset USB function
	// IO_USBF_CONTROL |= USBF_CONTROL_FRST;

	IO_USBF_CONTROL = 0;

	// turn off clocks
#ifdef xCONFIG_IRIS
	iris_AUXPLL_OFF_for_usb ();
#else
	IO_SYS_CLOCK_ENABLE &= ~(SYS_CLOCK_AUXPLL_EN | SYS_CLOCK_AUXCLK_EN);
#endif
}

/*
 * initialize L7205 USB Function
 */
void l7205_init (struct usb_device_instance *device)
{
	//int i;
	int offset;
	unsigned char *descriptors;

	// XXX disable everything
	IO_USBF_CONTROL = 0;
	udelay (20);

#if defined(CONFIG_L7205SDB)
	/*
	 * Linkup reports that there may be a problem with video conflicting with
	 * USB Function, this will disable it.
	 */

	dbg_udc (2, "1. CLCDCON: %08lx SYS_CLOCK_ENABLE: %08lx", IO_CLCDCON, IO_SYS_CLOCK_ENABLE);

	// disable LCD power (bit 22) and wait 20MS
	IO_CLCDCON &= ~0x20000;
	udelay (100);
	dbg_udc (2, "2. CLCDCON: %08lx SYS_CLOCK_ENABLE: %08lx", IO_CLCDCON, IO_SYS_CLOCK_ENABLE);

	// disable LCD (bit 0)
	IO_CLCDCON &= ~0x1;
	dbg_udc (2, "3. CLCDCON: %08lx SYS_CLOCK_ENABLE: %08lx", IO_CLCDCON, IO_SYS_CLOCK_ENABLE);

	// disable Vee and backlight bits 1, 2
	IO_SYS_CLOCK_ENABLE &= ~0x3;
	dbg_udc (2, "4. CLCDCON: %08lx SYS_CLOCK_ENABLE: %08lx", IO_CLCDCON, IO_SYS_CLOCK_ENABLE);
#endif

	/*
	 * Setup FIRCLK to use internal clock - c.f. 13.1.2 Clocks
	 */
	dbg_udc (5, "setting clocks (internal PLL)");

	// clear AUXOSCMUX, AUXPLLMUX and AUXPLLMUL bits of CLOCK_AUX
	IO_SYS_CLOCK_AUX = 0;

	// clear AUX0SCMUX and set AUXPLLMUL to 13 for 48Mhz
	IO_SYS_CLOCK_AUX |= SYS_CLOCK_AUXPLLMUL_48;

	// set FIRAUX_SEL
	IO_SYS_CLOCK_SELECT |= SYS_CLOCK_FIRAUX_SEL;

	// set AUXPLL_EN and AUXCLK_EN 
#ifdef xCONFIG_IRIS
	iris_AUXPLL_ON_for_usb ();
#else
	IO_SYS_CLOCK_ENABLE |= SYS_CLOCK_AUXPLL_EN | SYS_CLOCK_AUXCLK_EN;
#endif

	// wait 8us
	udelay (10);

	// set USBFUNC_EN to enable clock
	IO_SYS_CLOCK_ENABLE |= SYS_CLOCK_USBFUNC_EN;

	// wait 8us
	udelay (10);

	// enable USB Function
	IO_USBF_CONTROL |= USBF_CONTROL_ENBL;
	udelay (20);

	// set program modes
	IO_USBF_CONTROL |= USBF_CONTROL_F1MOD_IO | USBF_CONTROL_F2MOD_IO;

	// Force USB function core reset
	// l7205_toggle(IO_USBF_CONTROL, USBF_CONTROL_FRST);

	// Reset and clear all FIFO's
	// l7205_toggle(IO_USBF_CONTROL, USBF_CONTROL_F0CLR | USBF_CONTROL_F1CLR | USBF_CONTROL_F2CLR | USBF_CONTROL_F3CLR);

	// allocate tmp buffer
	if ((descriptors = kmalloc (USBF_DESCRIPTORS_MAX, GFP_ATOMIC))) {

		// clear the descriptor space
		memset (descriptors, 0, USBF_DESCRIPTORS_MAX);

		// XXX memset((void *)(__IOA(USBF_DESCRIPTORS)), 0, USBF_DESCRIPTORS_MAX);

		// copy in device, configuration, langid and up to four strings
		offset = l7205_copy_request (descriptors, device, 0, 0, USB_DESCRIPTOR_TYPE_DEVICE);
		offset =
		    l7205_copy_request (descriptors, device, USBF_CONFIGBUF1, offset,
					USB_DESCRIPTOR_TYPE_CONFIGURATION);
		offset =
		    l7205_copy_descriptor (descriptors, USBF_STRINGBUF0, offset,
					   (struct usb_descriptor *) usbd_get_string (0), 0);
		offset =
		    l7205_copy_descriptor (descriptors, USBF_STRINGBUF1, offset,
					   (struct usb_descriptor *) usbd_get_string (1), 0);
		offset =
		    l7205_copy_descriptor (descriptors, USBF_STRINGBUF2, offset,
					   (struct usb_descriptor *) usbd_get_string (2), 0);
		offset =
		    l7205_copy_descriptor (descriptors, USBF_STRINGBUF3, offset,
					   (struct usb_descriptor *) usbd_get_string (3), 0);
		offset =
		    l7205_copy_descriptor (descriptors, USBF_STRINGBUF4, offset,
					   (struct usb_descriptor *) usbd_get_string (4), 0);

		// copy descriptors to shared RAM
		memcpy ((unsigned char *) __IOA (USBF_DESCRIPTORS), descriptors,
			USBF_DESCRIPTORS_MAX);
		kfree (descriptors);
	} else {
		// should never happen
		dbg_udc (0, "cannot malloc descriptors");
	}

	/*
	 * Configuration and Intialization - c.f. 13.1.4 and c.f. 13.1.1
	 */
	IO_USBF_ENDPTBUF0 = USBF_ENDPTBUF0_EP0MSIZE_8;
	IO_USBF_ENDPTBUF1 = USBF_ENDPTBUF1_EP1TYPE_BULK | 0x20;
	IO_USBF_ENDPTBUF2 = USBF_ENDPTBUF2_EP2TYPE_BULK | 0x20;
	IO_USBF_ENDPTBUF3 = USBF_ENDPTBUF3_EP3TYPE_INT;

	IO_USBF_F1TOUT = 1;
	IO_USBF_F1BCNT = 0x20;
	IO_USBF_F2BCNT = 0;

	// Force USB function core reset
	l7205_toggle (IO_USBF_CONTROL, USBF_CONTROL_FRST);

	// Reset and clear all FIFO's
	l7205_toggle (IO_USBF_CONTROL,
		      USBF_CONTROL_F0CLR | USBF_CONTROL_F1CLR | USBF_CONTROL_F2CLR |
		      USBF_CONTROL_F3CLR);

	// Clear interrupts
	IO_USBF_INTCLR = 0x3fff;

	// set initialization done bit to indicate descriptors and registers are ready
	IO_USBF_CONTROL |= USBF_CONTROL_INTD;

	// enable interrupts
	IO_USBF_INTENA =
	    USBF_INTENA_ENSUS |
	    USBF_INTENA_ENF0ERR |
	    USBF_INTENA_ENF1OR |
	    USBF_INTENA_ENF1ERR |
	    USBF_INTENA_ENF2UR |
	    USBF_INTENA_ENF2ERR |
	    USBF_INTENA_ENF3ERR |
	    USBF_INTENA_ENGRSM |
	    USBF_INTENA_ENF0RQ |
	    USBF_INTENA_ENF1RQ |
	    USBF_INTENA_ENF2RQ | USBF_INTENA_ENF3RQ | USBF_INTENA_ENSOF | USBF_INTENA_ENHRST;

	// wait 
	udelay (60);		// XXX

	l7205_regs ("finished");
}


/*
 * Switch on the L7205 USB Function
 */
void udc_enable (struct usb_device_instance *device)
{
	udc_device = device;
	l7205_init (udc_device);	// XXX do when Vbus is present....
	return;
}

#if defined(TRIGGER)

static unsigned int max_triggers = 64;

static void send_usb_trigger (unsigned int status, char *id)
{
	/* This code is to insert a "trigger" into the outgoing data stream
	   so that the a USB protocol anlyser can capture the data surrounding
	   it.  It is only used when debugging strange hardware states.
	   The "trigger" is data that looks like "0xdeadbeef". */
	if (max_triggers == 0) {
		printk (KERN_DEBUG "%s & no more triggers\n", id);
		return;
	}
	if (status & USBF_STATUS_F2BSY) {
		printk (KERN_DEBUG "%s & F2BSY\n", id);
	} else {
		static char flag_bytes[48] = {
			0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef,
			    0xde, 0xad, 0xbe, 0xef,
			0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef,
			    0xde, 0xad, 0xbe, 0xef,
			0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef,
			    0xde, 0xad, 0xbe, 0xef
		};
		unsigned long *slp;
		unsigned long *txf = IO_USBF_FIFO_TX;
		int i, j, fifo_good;
		slp =
		    (unsigned long *) (void *) (0xfffffffc &
						(unsigned long) (3 + (void *) flag_bytes));
		j = 0;
		do {
			for (i = 0; i < 8; i++) {
				IO_USBF_FIFO2 = slp[i];
			}
			fifo_good = 1;
			for (i = 0; i < 8; i++) {
				fifo_good &= (slp[i] == txf[i]);
			}
		} while (++j < 10 && !fifo_good);
		if (fifo_good) {
			IO_USBF_F2BCNT = 32;
			printk (KERN_DEBUG "deadbeef %u %s\n", max_triggers, id);
			max_triggers -= 1;
		} else {
			// too busy receiving?
			printk (KERN_DEBUG "%s & F1BSY\n", id);
		}
	}
}
#endif

#define STATUS_NO_INTR_MASK   ~(USBF_STATUS_F1NE | USBF_STATUS_F2NF | USBF_STATUS_F2NE | USBF_STATUS_F2BSY | USBF_STATUS_VCCMD);


/**
 * udc_int_hndlr - interrupt handler
 *
 */
void udc_int_hndlr (int irq, void *dev_id, struct pt_regs *regs)
{
	int loopcount;
	unsigned int rawstatus;
	int ep2_int_hndlr_called = 0;

	udc_interrupts++;
	// XXX rawstatus = IO_USBF_RAWSTATUS;
	for (loopcount = 0; 0 != (rawstatus = IO_USBF_RAWSTATUS) && (loopcount < 4); loopcount++) {
		unsigned int status = IO_USBF_STATUS;

		IO_USBF_INTCLR = rawstatus;

		/*
		 * Handle high priority interrupts first to reduce latency and
		 * overhead.
		 */

		if (rawstatus & USBF_RAWSTATUS_RF1RQ) {
			ep1_int_hndlr ();
			sof_saw_rx_f1ne = 0;
		}

		if (rawstatus & USBF_RAWSTATUS_RF2RQ) {
			ep2_int_hndlr (status);
			ep2_int_hndlr_called++;
			sof_saw_tx_active = 0;
		}

		/*
		 * check if we have one of the un-common interrupts, this is faster
		 * than testing for each one sequentially. Note that we will do the
		 * SOF test first, and then another general test for the really
		 * un-common interrupts.
		 */
		if (rawstatus &
		    (USBF_RAWSTATUS_RSUS | USBF_RAWSTATUS_RF0ERR | USBF_RAWSTATUS_RF1OR |
		     USBF_RAWSTATUS_RF1ERR | USBF_RAWSTATUS_RF2ERR | USBF_RAWSTATUS_RF3ERR |
		     USBF_RAWSTATUS_RGRSM | USBF_RAWSTATUS_RF0RQ | USBF_RAWSTATUS_RF3RQ |
		     USBF_RAWSTATUS_RHRST | USBF_RAWSTATUS_RSOF)) {
			/*
			 * SOF - Start of frame interrupt, use as a safe place to check
			 * for missed transmit, receive interrupts, turn back on SUS or
			 * RST interrupts etc.
			 */
			if (rawstatus & USBF_RAWSTATUS_RSOF) {

				/*
				 * Simulate an rx interrupt, if we have not seen a rcv
				 * interrupt recently.
				 */
				if (IO_USBF_STATUS & USBF_STATUS_F1NE) {
					if ((sof_saw_rx_f1ne++ > 2)) {
						dbg_tick (1, "[%d]: F1NE %ld", udc_interrupts,
							  IO_USBF_F1BCNT);
						ep1_clear ();
						sof_saw_rx_f1ne = 0;
					}
				} else {
					sof_saw_rx_f1ne = 0;
				}

				/*
				 * This is required in case we miss a tx interrupt or we
				 * abandoned a tx attempt due to contention with rx fifo
				 */
				if ((sof_saw_tx_active++ > 10) && ep2_active) {
					ep2_int_hndlr_error (status, 0, ep2_int_hndlr_called++);
					sof_saw_tx_active = 0;
				}

				/*
				 * The SUS and HRST interrupts disable themselves, now is
				 * the time to re-enable them.
				 */
				if (host_sus_interrupt_disabled) {
					IO_USBF_INTENA = USBF_INTENA_ENSUS;
					host_sus_interrupt_disabled = 0;
					usbd_device_event (udc_device, DEVICE_BUS_ACTIVITY, 0);
				}
				if (host_reset_interrupt_disabled) {
					IO_USBF_INTENA = USBF_INTENA_ENHRST;
					host_reset_interrupt_disabled = 0;
					usbd_device_event (udc_device, DEVICE_ADDRESS_ASSIGNED, 0);
					udc_device->configuration = 0;
					usbd_device_event (udc_device, DEVICE_CONFIGURED, 0);
					udc_device->interface = 0;
					udc_device->alternate = 0;
					usbd_device_event (udc_device, DEVICE_SET_INTERFACE, 0);
				}

				/*
				 * If these three bits are on all is lost, we will
				 * disconnect.
				 */
				if ((status & USBF_STATUS_F2BSY) && (status & USBF_STATUS_F2NE)
				    && (status & USBF_STATUS_F2NF)) {
					// disconnect and reset udc
					// XXX udc_disconnect();
					// XXX udc_connect();
				}
			}

			/*
			 * check if we have a really uncommon interrupt, this saves
			 * sequential testing of all of these during SOF only
			 * interrupts.
			 */
			if (rawstatus &
			    (USBF_RAWSTATUS_RSUS | USBF_RAWSTATUS_RF0ERR | USBF_RAWSTATUS_RF1OR |
			     USBF_RAWSTATUS_RF1ERR | USBF_RAWSTATUS_RF2ERR | USBF_RAWSTATUS_RF3ERR |
			     USBF_RAWSTATUS_RGRSM | USBF_RAWSTATUS_RF0RQ | USBF_RAWSTATUS_RF3RQ |
			     USBF_RAWSTATUS_RHRST)) {
				/*
				 * SUS - suspend interrupt, issued device event and disable SUS interrupt
				 * until we see an SOF interrupt
				 */
				if (rawstatus & USBF_RAWSTATUS_RSUS) {
					dbg_intr (0, "[%u]: SUS Sts: %08x Raw: %08x",
						  udc_interrupts, status, rawstatus);
					// disable host interrupt
					IO_USBF_INTDIS = USBF_INTDIS_DNSUS;
					host_sus_interrupt_disabled++;
					usbd_device_event (udc_device, DEVICE_BUS_INACTIVE, 0);
				}

				/*
				 * HRST - Host Reset interrupt, issued device event and disable SUS interrupt
				 * until we see an SOF interrupt
				 */
				if (rawstatus & USBF_RAWSTATUS_RHRST) {
					// L7210 AB version would give multiple HRST when the first is cleared,
					// ignore them until SOF interrupt

					dbg_intr (0, "[%u]: HRST", udc_interrupts);
					// XXX l7205_toggle(IO_USBF_CONTROL, USBF_CONTROL_FRST);

					// reset udc
					l7205_init (udc_device);	// XXX 

					// disable host interrupt
					IO_USBF_INTDIS = USBF_INTDIS_DNHRST;
					host_reset_interrupt_disabled++;
					usbd_device_event (udc_device, DEVICE_RESET, 0);
				}

				/*
				 * F1ERR - FIFO 1 error, a receive error, bad news, usually once
				 * this has happened we cannot continue or reset to normal
				 * state. In desparation we will drop connection and hope that
				 * HRST will restore things.
				 */
				if (rawstatus & USBF_RAWSTATUS_RF1ERR) {
					dbg_intr (0, "[%u]: F1ERR Sts: %08x Raw: %08x F1Bcnt: %ld",
						  udc_interrupts, status, rawstatus,
						  IO_USBF_F1BCNT);
					sof_saw_rx_f1ne = 0;
#if defined(FLAG_F1ERR) && defined(TRIGGER)
					send_usb_trigger (status, "F1ERR");
#endif
					udc_disconnect ();
					ep1_int_hndlr_error ();
					udelay (100);
					udc_connect ();
					udelay (100);
					udc_disconnect ();

					// XXX udc_disconnect();
					// XXX l7205_init(udc_device); // XXX 
				}

				/*
				 * Misc Error interrupts - have never seen any of these.
				 */
				if (rawstatus & USBF_RAWSTATUS_RF0ERR) {
					dbg_intr (0, "[%u]: F0ERR Sts: %08x Raw: %08x",
						  udc_interrupts, status, rawstatus);
				}
				if (rawstatus & USBF_RAWSTATUS_RF1OR) {
					dbg_intr (0, "[%u]: F1OR Sts: %08x Raw: %08x",
						  udc_interrupts, status, rawstatus);
					ep1_int_hndlr_error ();
					sof_saw_rx_f1ne = 0;
				}
				if (rawstatus & USBF_RAWSTATUS_RF2UR) {
					dbg_intr (0, "[%u]: F2UR Sts: %08x Raw: %08x",
						  udc_interrupts, status, rawstatus);
					sof_saw_tx_active = 0;
					ep2_int_hndlr_error (status, 1, ep2_int_hndlr_called++);
				}
				if (rawstatus & USBF_RAWSTATUS_RF2ERR) {
					dbg_intr (0, "[%u]: F2ERR Sts: %08x Raw: %08x",
						  udc_interrupts, status, rawstatus);
					sof_saw_tx_active = 0;
					ep2_int_hndlr_error (status, 1, ep2_int_hndlr_called++);
				}
				if (rawstatus & USBF_RAWSTATUS_RF3ERR) {
					dbg_intr (0, "[%u]: F3ERR Sts: %08x Raw: %08x",
						  udc_interrupts, status, rawstatus);
				}
				if (rawstatus & USBF_RAWSTATUS_RGRSM) {
					dbg_intr (0, "[%u]: GRSM Sts: %08x Raw: %08x",
						  udc_interrupts, status, rawstatus);
				}
				if (rawstatus & USBF_RAWSTATUS_RF0RQ) {
					dbg_intr (0, "[%u]: F0RQ Sts: %08x Raw: %08x",
						  udc_interrupts, status, rawstatus);
				}
				if (rawstatus & USBF_RAWSTATUS_RF3RQ) {
					dbg_intr (0, "[%u]: F3RQ Sts: %08x Raw: %08x",
						  udc_interrupts, status, rawstatus);
				}
			}
		}
	}			// end of loopcount loop
}

/* ********************************************************************************************* */
/*
 * Start of public functions.
 */

/**
 * udc_request_udc_irq - request UDC interrupt
 *
 * Return non-zero if not successful.
 */
int udc_request_udc_irq ()
{
	// request IRQ 
	if (request_irq
	    (IRQ_USBF, udc_int_hndlr, SA_INTERRUPT | SA_SAMPLE_RANDOM, "L7205 USBD Bus Interface",
	     NULL) != 0) {
		dbg_init (0, "Couldn't request USB irq");
		return -EINVAL;
	}
	return 0;
}

/**
 * udc_start_in_irq - start transmit
 * @eendpoint: endpoint instance
 *
 * Called by bus interface driver to see if we need to start a data transmission.
 */
void udc_start_in_irq (struct usb_endpoint_instance *endpoint)
{
	if (!(IO_USBF_STATUS & USBF_STATUS_F2BSY)) {
		ep2_start ();
	}
}

/**
 * udc_stall_ep - stall endpoint
 * @ep: physical endpoint
 *
 * Stall the endpoint.
 */
void udc_stall_ep (unsigned int ep)
{
}

/**
 * udc_reset_ep - reset endpoint
 * @ep: physical endpoint
 * reset the endpoint.
 *
 * returns : 0 if ok, -1 otherwise
 */
void udc_reset_ep (unsigned int ep)
{
	if (ep < UDC_MAX_ENDPOINTS) {
		// reset

		switch (ep) {
		case 0:
		case 1:
		case 0x82:
			/* do nothing */
			break;
		}
	}
}

/**
 * udc_endpoint_halted - is endpoint halted
 * @ep:
 *
 * Return non-zero if endpoint is halted
 */
int udc_endpoint_halted (unsigned int ep)
{
	return 0;
}

/**
 * udc_set_address - set the USB address for this device
 * @address:
 *
 * Called from control endpoint function after it decodes a set address setup packet.
 */
void udc_set_address (unsigned char address)
{
}


static int crc32 (char *s, int length)
{
	/* indices */
	int pByte;
	int pBit;

	const unsigned long poly = 0xedb88320;
	unsigned long crc_value = 0xffffffff;

	for (pByte = 0; pByte < length; pByte++) {
		unsigned char c = *(s++);
		for (pBit = 0; pBit < 8; pBit++) {
			crc_value = (crc_value >> 1) ^ (((crc_value ^ c) & 0x01) ? poly : 0);
			c >>= 1;
		}
	}
	return crc_value;
}

/**
 * udc_serial_init - set a serial number if available
 */
int __init udc_serial_init (struct usb_bus_instance *bus)
{
#ifdef CONFIG_IRIS
	iris_serial_number_struct serial;

	if (!iris_get_serial_number (&serial)
	    && (bus->serial_number_str = kmalloc (36, GFP_KERNEL))) {
		sprintf (bus->serial_number_str, "%08x-%08x-%08x-%08x", serial.b3, serial.b2,
			 serial.b1, serial.b0);
		bus->serial_number = crc32 (bus->serial_number_str, 32);
		return 0;
	}
#endif
	return -EINVAL;
}

/**
 * udc_max_endpoints - max physical endpoints 
 *
 * Return number of physical endpoints.
 */
int udc_max_endpoints (void)
{
	return UDC_MAX_ENDPOINTS;
}

/**
 * udc_check_ep - check logical endpoint 
 * @lep:
 *
 * Return physical endpoint number to use for this logical endpoint or zero if not valid.
 */
int udc_check_ep (int logical_endpoint, int packetsize)
{
	if (packetsize > 32) {
		return 0;
	}
	switch (logical_endpoint) {
	case 1:
		return 1;
	case 0x82:
		return 2;
	case 0x83:
		return 3;
	default:
		return 0;
	}
}

/**
 * udc_set_ep - setup endpoint 
 * @ep:
 * @endpoint:
 *
 * Associate a physical endpoint with endpoint_instance
 */
void udc_setup_ep (struct usb_device_instance *device, unsigned int ep,
		   struct usb_endpoint_instance *endpoint)
{
	dbg_init (1, "ep: %d", ep);
	if (ep < UDC_MAX_ENDPOINTS) {
		ep_endpoints[ep] = endpoint;
		switch (ep) {
		case 0:	// Control
			break;
		case 1:	// OUT
			usbd_fill_rcv (device, endpoint, 10);	// XXX
			endpoint->rcv_urb = first_urb_detached (&endpoint->rdy);
			ep1_device = device;
			ep1_endpoint = endpoint;
			break;
		case 2:	// IN
			ep2_device = device;
			ep2_endpoint = endpoint;
			break;
		}
	}
}

/**
 * udc_disable_ep - disable endpoint
 * @ep:
 *
 * Disable specified endpoint 
 */
void udc_disable_ep (unsigned int ep)
{
	if (ep < UDC_MAX_ENDPOINTS) {
		struct usb_endpoint_instance *endpoint;

		switch (ep) {
		case 0:	// Control
			break;
		case 1:	// OUT
			ep1_endpoint = NULL;
			break;
		case 2:	// IN
			ep2_device = NULL;
			ep2_endpoint = NULL;
			break;
		}

		if ((endpoint = ep_endpoints[ep])) {
			ep_endpoints[ep] = NULL;
			usbd_flush_ep (endpoint);
		}

	}
}

/**
 * udc_connected - is the USB cable connected
 *
 * Return non-zeron if cable is connected.
 */
int udc_connected ()
{
#ifdef CONFIG_IRIS
	// Return TRUE if USB cable shows Vbus _and_ AC power present
	dbg_pur (0, "Vbus: %x AC: %x", iris_read_usb_sync (), iris_read_ac_sync ());
	return (iris_read_usb_sync () && iris_read_ac_sync ());
#else
	return 1;
#endif
}

/**
 * udc_connect - enable pullup resistor
 *
 * Turn on the USB connection by enabling the pullup resistor.
 */
void udc_connect (void)
{
#ifdef CONFIG_IRIS
	// Enable the pullup resistor.
	dbg_pur (0, "Enabling IRIS USB pullup resistor");
	SET_PB_OUT (USB_PULLUP_GPIO);
	SET_PBDR_HI (USB_PULLUP_GPIO);
	SET_PBSBSR_LO (USB_PULLUP_GPIO);
#endif
}

/**
 * udc_disconnect - disable pullup resistor
 *
 * Turn off the USB connection by disabling the pullup resistor.
 */
void udc_disconnect (void)
{
#ifdef CONFIG_IRIS
	// Disable the pullup resistor.
	dbg_pur (0, "Disabling IRIS USB pullup resistor");
	SET_PB_OUT (USB_PULLUP_GPIO);
	SET_PBDR_LO (USB_PULLUP_GPIO);
	SET_PBSBSR_LO (USB_PULLUP_GPIO);
#endif
}

/**
 * udc_all_interrupts - enable interrupts
 *
 * Switch on UDC interrupts.
 *
 */
void udc_all_interrupts (struct usb_device_instance *device)
{
	dbg_init (1, " ");
	// set interrupt mask
}

/**
 * udc_suspended_interrupts - enable suspended interrupts
 *
 * Switch on only UDC resume interrupt.
 *
 */
void udc_suspended_interrupts (struct usb_device_instance *device)
{
	dbg_init (1, " ");
	// set interrupt mask
}

/**
 * udc_disable_interrupts - disable interrupts.
 *
 * switch off interrupts
 */
void udc_disable_interrupts (struct usb_device_instance *device)
{
	dbg_init (1, " ");
	// reset interrupt mask
}

/**
 * udc_ep0_packetsize - return ep0 packetsize
 */
int udc_ep0_packetsize (void)
{
	return EP0_PACKETSIZE;
}

/**
 * udc_startup_events - allow udc code to do any additional startup
 */
void udc_startup_events (struct usb_device_instance *device)
{
	usbd_device_event (device, DEVICE_INIT, 0);
	usbd_device_event (device, DEVICE_CREATE, 0);
	usbd_device_event (device, DEVICE_HUB_CONFIGURED, 0);

	// XXX moved to first SOF after HRST
	//usbd_device_event(device, DEVICE_RESET, 0);            // XXX should be done from device event
	// XXX usbd_device_event(device, DEVICE_ADDRESS_ASSIGNED, 0);
	//device->configuration = 0; 
	// XXX usbd_device_event(device, DEVICE_CONFIGURED, 0);
	//device->interface = 1; 
	//device->alternate = 0;
	// XXX usbd_device_event(device, DEVICE_SET_INTERFACE, 0);
}

/**
 * udc_name - return name of USB Device Controller
 */
char *udc_name (void)
{
	return UDC_NAME;
}

/**
 * udc_request_cable_irq - request Cable interrupt
 *
 * Return non-zero if not successful.
 */
int udc_request_cable_irq ()
{
	return 0;
}

#ifdef CONFIG_USBD_PROCFS
/* Proc Filesystem *************************************************************************** */

/* *
 * l7205_proc_read - implement proc file system read.
 * @file
 * @buf
 * @count
 * @pos
 *
 * Standard proc file system read function.
 */
static ssize_t l7205_proc_read (struct file *file, char *buf, size_t count, loff_t * pos)
{
	unsigned long page;
	int len = 0;
	int index;

	MOD_INC_USE_COUNT;
	// get a page, max 4095 bytes of data...
	if (!(page = get_free_page (GFP_KERNEL))) {
		MOD_DEC_USE_COUNT;
		return -ENOMEM;
	}

	len = 0;
	index = (*pos)++;

	if (index == 0) {
		len += sprintf ((char *) page + len, "l7205 UDC status\n");
	}

	if (index == 1) {
		len +=
		    sprintf ((char *) page + len, "%s\n",
			     udc_connected ()? "Connected" : "Not connected");
	}

	if (len > count) {
		len = -EINVAL;
	} else if (len > 0 && copy_to_user (buf, (char *) page, len)) {
		len = -EFAULT;
	}
	free_page (page);
	MOD_DEC_USE_COUNT;
	return len;
}

/* *
 * l7205_proc_write - implement proc file system write.
 * @file
 * @buf
 * @count
 * @pos
 *
 * Proc file system write function, used to signal monitor actions complete.
 * (Hotplug script (or whatever) writes to the file to signal the completion
 * of the script.)  An ugly hack.
 */
static ssize_t l7205_proc_write (struct file *file, const char *buf, size_t count, loff_t * pos)
{
	size_t n = count;
	char command[64];
	char *cp = command;
	int i = 0;
	MOD_INC_USE_COUNT;
	//printk(KERN_DEBUG "%s: count=%u\n",__FUNCTION__,count);
	while ((n > 0) && (i < 64)) {
		// Not too efficient, but it shouldn't matter
		if (copy_from_user (cp++, buf + (count - n), 1)) {
			count = -EFAULT;
			break;
		}
		*cp = '\0';
		i++;
		n -= 1;
		//printk(KERN_DEBUG "%s: %u/%u %02x\n",__FUNCTION__,count-n,count,c);
	}
	if (!strncmp (command, "plug", 4)) {
		udc_connect ();
	} else if (!strncmp (command, "unplug", 6)) {
		udc_disconnect ();
	}
	MOD_DEC_USE_COUNT;
	return (count);
}

static struct file_operations l7205_proc_operations_functions = {
	read:l7205_proc_read,
	write:l7205_proc_write,
};

#endif

/**
 * udc_request_udc_io - request UDC io region
 *
 * Return non-zero if not successful.
 */
int udc_request_io ()
{
#ifdef CONFIG_USBD_PROCFS
	{
		struct proc_dir_entry *p;

		// create proc filesystem entries
		if ((p = create_proc_entry ("l7205", 0, 0)) == NULL) {
			return -ENOMEM;
		}
		p->proc_fops = &l7205_proc_operations_functions;
	}
#endif
	return 0;
}

/**
 * udc_release_udc_irq - release UDC irq
 */
void udc_release_udc_irq ()
{
	free_irq (IRQ_USBF, NULL);
}

/**
 * udc_release_cable_irq - release Cable irq
 */
void udc_release_cable_irq ()
{
}

/**
 * udc_release_release_io - release UDC io region
 */
void udc_release_io ()
{
#ifdef CONFIG_USBD_PROCFS
	remove_proc_entry ("l7205", NULL);
#endif

}
