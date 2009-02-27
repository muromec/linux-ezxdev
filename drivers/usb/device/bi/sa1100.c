/*
 * linux/drivers/usbd/sa1100_bi/udc.c
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
 * Please note that this driver works reasonably well with StrongARM parts
 * running at 206Mhz. 
 *
 * It is not possible to run the USB with DMA on the StrongARM running at 133Mhz.
 * Running without DMA is possible and reasonably reliable. This can be done by
 * restricting the USB packetsize to 16bytes for your bulk endpoints.
 */ 



#include <linux/config.h>
#include <linux/module.h>

#include "../usbd-export.h"
#include "../usbd-build.h"
#include "../usbd-module.h"

MODULE_AUTHOR ("sl@lineo.com, tbr@lineo.com");
MODULE_DESCRIPTION ("USB Device SA-1100 Bus Interface");

USBD_MODULE_INFO ("sa1100_bi 0.2");

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>

#include <asm/atomic.h>
#include <asm/io.h>

#include <linux/proc_fs.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,6)
#define USE_ADD_DEL_TIMER_FOR_USBADDR_CHECK 1
#include <linux/timer.h>
#else
#undef USE_ADD_DEL_TIMER_FOR_USBADDR_CHECK
#include <linux/tqueue.h>
#endif

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


#include "../usbd-debug.h"

#include "../usbd.h"
#include "../usbd-func.h"
#include "../usbd-bus.h"
#include "../usbd-inline.h"
#include "usbd-bi.h"

#include "sa1100.h"


#if defined(CONFIG_SA1100_COLLIE)       // XXX change to 5500
#include <asm-arm/arch-sa1100/collie.h>
#include <asm/ucb1200.h>
#include <asm/arch/tc35143.h>
#endif



#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

#define ACTIVEA 1
#define ACTIVEB 2

static struct usb_device_instance *udc_device;	// required for the interrupt handler

/*
 * ep_endpoints - map physical endpoints to logical endpoints
 */
static struct usb_endpoint_instance *ep_endpoints[UDC_MAX_ENDPOINTS];

//static struct urb ep0_urb;
unsigned char usb_address;

extern unsigned int udc_interrupts;
extern unsigned int udc_interrupts_last;

unsigned int ep0_interrupts;
unsigned int tx_interrupts;
unsigned int rx_interrupts;
unsigned int sus_interrupts;
unsigned int res_interrupts;
unsigned int udc_address_errors;
unsigned int udc_ticks;
unsigned int udc_fixed;

unsigned int ep0_interrupts_last;
unsigned int rx_interrupts_last;
unsigned int tx_interrupts_last;

unsigned int udc_rpe_errors;
unsigned int udc_ep1_errors;
unsigned int udc_ep2_errors;
unsigned int udc_ep2_tpe;
unsigned int udc_ep2_tur;
unsigned int udc_ep2_sst;
unsigned int udc_ep2_fst;




int usbd_rcv_dma;
int usbd_tx_dma;

static int udc_saw_sof;

#ifdef USE_ADD_DEL_TIMER_FOR_USBADDR_CHECK
static struct timer_list sa1100_usb_dev_addr_check;
static int usb_addr_check_initialized = 0;
#define CHECK_INTERVAL   1
#else
static struct tq_struct sa1100_tq;
#endif


/*
 * DMA control register structure
 */
typedef struct {
	volatile u_long DDAR;
	volatile u_long SetDCSR;
	volatile u_long ClrDCSR;
	volatile u_long RdDCSR;
	volatile dma_addr_t DBSA;
	volatile u_long DBTA;
	volatile dma_addr_t DBSB;
	volatile u_long DBTB;
} dma_regs_t;

/* ********************************************************************************************* */
/* IO
 */

volatile int udc (volatile unsigned int *regaddr)
{
	volatile unsigned int value;
	int ok;
	for (ok = 1000, value = *(regaddr); value != *(regaddr) && ok--; value = *(regaddr));
	if (!ok) {
		dbg_udc (0, "NOT OK: %p %x", regaddr, value);
	}
	return value;
}

static __inline__ volatile int _udc (volatile unsigned int *regaddr)
{
	volatile unsigned int value;
	int ok;
	for (ok = 1000, value = *(regaddr); value != *(regaddr) && ok--; value = *(regaddr));
	if (!ok) {
		printk (KERN_ERR "NOT OK: %p %x\n", regaddr, value);
	}
	return value;
}


static __inline__ void _sa1100_bi_dma_run (dmach_t channel, int active)
{
	dma_regs_t *regs = (dma_regs_t *) io_p2v (_DDAR (channel));
	if (active == ACTIVEA) {
		regs->SetDCSR = DCSR_STRTA | DCSR_RUN;
	} else {
		regs->SetDCSR = DCSR_STRTB | DCSR_RUN;
	}
}

static __inline__ int _sa1100_bi_dma_queue_buffer_irq (dmach_t channel, dma_addr_t data, int size)
{
	int status;
	dma_regs_t *regs = (dma_regs_t *) io_p2v (_DDAR (channel));

	status = regs->RdDCSR;
	if (((status & DCSR_BIU) && (status & DCSR_STRTB)) || (!(status & DCSR_BIU) && !(status & DCSR_STRTA))) {
		regs->ClrDCSR = DCSR_DONEA | DCSR_STRTA;
		regs->DBSA = data;
		regs->DBTA = size;
		// Once is good, twice is better....
		regs->DBSA = data;
		regs->DBTA = size;
		return ACTIVEA;
	} else {
		regs->ClrDCSR = DCSR_DONEB | DCSR_STRTB;
		regs->DBSB = data;
		regs->DBTB = size;
		// Once is good, twice is better....
		regs->DBSB = data;
		regs->DBTB = size;
		return ACTIVEB;
	}
}

static __inline__ int _sa1100_bi_dma_flush_all_irq (dmach_t channel)
{
	dma_regs_t *regs = (dma_regs_t *) io_p2v (_DDAR (channel));
	regs->ClrDCSR = DCSR_STRTA | DCSR_STRTB | DCSR_RUN | DCSR_IE;
	return 0;
}

static __inline__ int _sa1100_bi_dma_stop_get_current_irq (dmach_t channel, dma_addr_t * addr, int active)
{
	dma_regs_t *regs = (dma_regs_t *) io_p2v (_DDAR (channel));

	// addr sometimes can be set incorrectly, the caller must do bounds checking
	switch (active) {
	case ACTIVEA:
		regs->ClrDCSR = DCSR_RUN | DCSR_STRTA | DCSR_RUN | DCSR_IE;
		*addr = regs->DBSA;	// not reliable
		return 0;
	case ACTIVEB:
		regs->ClrDCSR = DCSR_RUN | DCSR_STRTB | DCSR_RUN | DCSR_IE;
		*addr = regs->DBSB;	// not reliable
		return 0;
	default:
		*addr = 0;
		return -ENXIO;
	}
}



/* ********************************************************************************************* */


static u32 getCPUID (char **stepping)
{
	u32 cpuID;

	__asm__ __volatile__ (" mrc   p15, 0, %0, c0, c0":"=r" (cpuID)
			      :);

	if (NULL != stepping) {
		switch (cpuID & 0xf) {
		case 0:
			*stepping = "A0";
			break;
		case 4:
			*stepping = "B0";
			break;
		case 5:
			*stepping = "B1";
			break;
		case 6:
			*stepping = "B2";
			break;
		case 8:
			*stepping = "B4";
			break;
		case 9:
			*stepping = "B5";
			break;
		default:
			*stepping = "??";
			dbg_udc (0, "stepping unknown, ID#%x", (cpuID & 0xf));
		}
	}

	return (cpuID);
}



void udc_fix_errata_29 (char *msg)
{
	int ok;
	u32 cpuID;		// from init.c

	cpuID = getCPUID (NULL);

	// Set errata 29 fix bit, if possible
	if (cpuID == 0 || (cpuID & 0xF) >= 9) {
		// Unknown CPU, or B5 stepping and above
		// set errata 29 fix enable (for B5 and above, B4 will ignore)
		int udc_cr;
		for (ok = 10; ok > 0; ok--) {
			*(UDCCR) |= /*UDCCR_ERR29 */ 0x80;
			// Do some dummy reads....
			udc_cr = *(UDCCR);
			udc_cr = *(UDCCR);
			udc_cr = *(UDCCR);
			if (udc (UDCCR) & /*UDCCR_ERR29 */ 0x80) {
				dbg_udc (0,
					 "%s: set errata 29 fix bit worked, UDCCR#%02x ok=%d cpuID#%08x",
					 msg, udc (UDCCR), ok, cpuID);
				ok = -2;
				break;
			}
		}
		if (ok != -2) {
			dbg_udc (0, "%s: set errata 29 fix bit failed, UDCCR#%02x ok=%d cpuID#%08x",
				 msg, udc (UDCCR), ok, cpuID);
		}
	} else {
		dbg_udc (0, "%s: errata 29 fix bit not available, cpuID#%08x", msg, cpuID);
	}
}

/* ********************************************************************************************* */

/**
 * sa1100_tick - clock timer task 
 * @data:
 *  
 * Run from global clock tick to check if we are suspended.
 */
#ifdef USE_ADD_DEL_TIMER_FOR_USBADDR_CHECK
static void sa1100_tick (unsigned long data)
#else
static void sa1100_tick (void *data)
#endif
{
	udc_ticks++;

	// is driver active
	if (data) {

		if (_udc (UDCAR) != usb_address) {
			dbg_udc (0, "sa1100_tick: ADDRESS ERROR DETECTED %02x %02x", *(UDCAR), usb_address);
			udc_address_errors++;
			udc_fixed++;
		}
		*(UDCAR) = usb_address;

		// re-queue task
#ifdef USE_ADD_DEL_TIMER_FOR_USBADDR_CHECK
		sa1100_usb_dev_addr_check.expires = jiffies + CHECK_INTERVAL;
		add_timer (&sa1100_usb_dev_addr_check);
#else
		queue_task (&sa1100_tq, &tq_timer);
#endif

	}
}


/* ********************************************************************************************* */
/* Interrupt Handler
 */

/**
 * int_hndlr_cable - interrupt handler for cable
 */
static void int_hndlr_cable (int irq, void *dev_id, struct pt_regs *regs)
{
#ifdef CONFIG_SA1100_USBCABLE_GPIO
	dbg_udc (1, "udc_cradle_interrupt:");
	udc_cable_event ();
#endif
}


/**
 * int_hndlr_device - interrupt handler
 *
 */
static void int_hndlr_device (int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned int status;

	status = *(UDCSR);
	*(UDCSR) = status;

	udc_interrupts++;

	//dbg_udc(0, "");
	dbg_intr(2, "[%d]: CSR: %02x CCR: %02x CAR: %02x:%02x", udc_interrupts, status, *(UDCCR), *(UDCAR), usb_address);

	// Handle common interrupts first, IN (tx) and OUT (recv)

	if (status & UDCSR_RIR) {
		ep1_int_hndlr (status);
	}

	if (status & UDCSR_TIR) {
		ep2_int_hndlr (status, 1);
	}
	// handle less common interrupts
	if (status & (UDCSR_EIR | UDCSR_RSTIR | UDCSR_SUSIR | UDCSR_RESIR)) {
		if (status & UDCSR_EIR) {
			ep0_int_hndlr (status);
		}

		if (status & UDCSR_RSTIR) {
			dbg_intr (1, "[%d] DEVICE_RESET: CSR: %02x CS0: %02x CAR: %02x", 
                                        udc_interrupts, *(UDCSR), *(UDCCS0), *(UDCAR));
			usbd_device_event (udc_device, DEVICE_RESET, 0);
		}

		if (status & UDCSR_SUSIR) {
			dbg_intr (1, "[%d] SUSPEND address: %02x irq: %02x status: %02x", 
                                        udc_interrupts, *(UDCAR), irq, status);
			sus_interrupts++;
#if defined(CONFIG_SA1100_COLLIE)	// XXX change to 5500
			usbd_device_event (udc_device, DEVICE_BUS_INACTIVE, 0);
#else
			usbd_device_event (udc_device, DEVICE_BUS_INACTIVE, 0);
			//usbd_device_event (udc_device, DEVICE_RESET, 0);
#endif
			udc_suspended_interrupts (udc_device);
			udc_ticker_poke ();
		}

		if (status & UDCSR_RESIR) {
			dbg_intr (1, "[%d] RESUME address: %02x irq: %02x status: %02x", udc_interrupts, *(UDCAR), irq, status);
                        *(UDCAR) = usb_address;
			res_interrupts++;
#if defined(CONFIG_SA1100_COLLIE)	// XXX change to 5500
			usbd_device_event (udc_device, DEVICE_BUS_ACTIVITY, 0);
#endif
			udc_all_interrupts (udc_device);
			udc_ticker_poke ();
		}
	}
	// Check that the UDC has not forgotton it's address, force it back to correct value
	//if (_udc (UDCAR) != usb_address) {
	//	udc_address_errors++;
	//}
	*(UDCAR) = usb_address;

}


/* ********************************************************************************************* */
/*
 * Start of public functions.
 */

/**
 * udc_init - initialize
 *
 * Return non-zero if we cannot see device.
 **/
int udc_init (void)
{
	// reset

	return 0;
}

/**
 * udc_start_in_irq - start transmit
 * @endpoint:
 *
 * Called with interrupts disabled.
 */
void udc_start_in_irq (struct usb_endpoint_instance *endpoint)
{
	ep2_int_hndlr (0, 0);
}


void udc_stall_ep0 (void)
{
	int ok = 0;
	// QQQ eh?
	dbg_udc (0, "stalling ep0 (UDCCS0_FST,UDCCS0_FST,UDCCS0_SST)");

	// write 1 to set FST
	SET_AND_TEST ((*(UDCCS0) &= UDCCS0_FST), !(udc (UDCCS0) & UDCCS0_FST), ok);
	if (!ok) {
		dbg_udc (0, "cannot stall !(UDCCS0&UDCCS0_FST) UDCCS0: %02x", *(UDCCS0));
	}
	// write 0 to reset FST
	SET_AND_TEST ((*(UDCCS0) &= ~UDCCS0_FST), (udc (UDCCS0) & UDCCS0_FST), ok);
	if (!ok) {
		dbg_udc (0, "cannot stall (UDCCS0&UDCCS0_FST) UDCCS0: %02x", *(UDCCS0));
	}
	// write 1 to reset SST
	SET_AND_TEST ((*(UDCCS0) = UDCCS0_SST), (udc (UDCCS0) & UDCCS0_SST), ok);
	if (!ok) {
		dbg_udc (0, "cannot stall (UDCCS0&UDCCS0_SST) UDCCS0: %02x", *(UDCCS0));
	}
}

static void stall_ep_n (int ep, volatile unsigned int *regaddr, int fst)
{
	int ok;
	dbg_udc (0, "stalling ep %d (FST)", ep);

	// write 1 to set FST
	SET_AND_TEST ((*(regaddr) = fst), !(udc (regaddr) & fst), ok);
	if (!ok) {
		dbg_udc (0, "cannot stall !(reg&fst) UDCCS%d: %02x", ep, *(regaddr));
	}
}

static void reset_ep_n (int ep, volatile unsigned int *regaddr, int fst, int sst)
{
	int ok;
	dbg_udc (1, "reset ep %d (FST)", ep);

	// write 0 to reset FST
	SET_AND_TEST ((*(regaddr) &= ~fst), (udc (regaddr) & fst), ok);
	if (!ok) {
		dbg_udc (0, "cannot stall !(reg&fst) UDCCS%d: %02x", ep, *(regaddr));
	}
	// write 1 to reset SST
	SET_AND_TEST ((*(regaddr) = sst), (udc (regaddr) & sst), ok);
	if (!ok) {
		dbg_udc (0, "cannot stall (reg&sst) UDCCS%d: %02x", ep, *(UDCCS0));
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
	dbg_udc (0, "STALLING %d (FST)", ep);
	switch (ep) {
	case 1:
		stall_ep_n (1, UDCCS1, UDCCS1_FST);
		break;
	case 2:
		stall_ep_n (2, UDCCS2, UDCCS2_FST);
		break;
	}
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
	dbg_udc (1, "RESETING %d (FST)", ep);
	switch (ep) {
	case 0:
		reset_ep_n (1, UDCCS0, UDCCS0_FST, UDCCS0_SST);
		break;
	case 1:
		reset_ep_n (1, UDCCS1, UDCCS1_FST, UDCCS1_SST);
		break;
	case 2:
		reset_ep_n (2, UDCCS2, UDCCS2_FST, UDCCS2_SST);
		break;
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
	switch (ep) {
	case 0:
		return (udc (UDCCS0) & UDCCS0_FST) != 0;
	case 1:
		return (udc (UDCCS1) & UDCCS1_FST) != 0;
	case 2:
		return (udc (UDCCS2) & UDCCS2_FST) != 0;
	}
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
	usb_address = address;
#if 0
        int ok;
	// address can be setup, udc will wait until ack received
	dbg_udc(1, "%02d", address);
        IOIOIO(UDCAR, usb_address, *(UDCAR) = address , (udc(UDCAR) != address), ok);

        if (*(UDCAR) != address) {
                dbg_udc(0, "AA %02d %02d ok: %d", udc(UDCAR), address, ok);
                udelay(20);
                IOIOIO(UDCAR, usb_address, *(UDCAR) = address , (udc(UDCAR) != address), ok);
                if (*(UDCAR) != address) {
                        dbg_udc(0, "BB %02d %02d ok: %d", udc(UDCAR), address, ok);
                        udelay(40);
                        IOIOIO(UDCAR, usb_address, *(UDCAR) = address , (udc(UDCAR) != address), ok);
                }
        }
        dbg_udc(0, "CC %02d %02d ok: %d", udc(UDCAR), address, ok);
#endif
}

#ifdef CONFIG_SA1100_BITSY
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
#endif

/**
 * udc_serial_init - set a serial number if available
 */
int __init udc_serial_init (struct usb_bus_instance *bus)
{
#if defined(SHOW_CPU_STEPPING)
	char *stepping;
	u32 id = getCPUID (&stepping);

	dbg_init (0, "CPU ID #%08x stepping %s",id,stepping);
#else
	dbg_init (2, "serial number");
#endif

#ifdef CONFIG_SA1100_BITSY
	if (machine_is_bitsy ()) {
		char serial_number[22];
		int i;
		int j;
		memset (&serial_number, 0, sizeof (serial_number));
		for (i = 0, j = 0; i < 20; i++, j++) {
			char buf[4];
			h3600_eeprom_read (5 + i, buf, 2);
			serial_number[j] = buf[1];
		}
		serial_number[j] = '\0';
		bus->serial_number = crc32 (serial_number, 22);
		if (bus->serial_number_str = kmalloc (strlen (serial_number) + 1, GFP_KERNEL)) {
			strcpy (bus->serial_number_str, serial_number);
		}
		//dbg_udc(0,  "serial: %s %08x", bus->serial_number_str, bus->serial_number);
		return 0;
	}
#endif

#ifdef CONFIG_SA1100_CALYPSO
	if (machine_is_calypso ()) {
		__u32 eerom_serial;
		int i;
		if ((i =
		     CalypsoIicGet (IIC_ADDRESS_SERIAL0, (unsigned char *) &eerom_serial,
				    sizeof (eerom_serial)))) {
			bus->serial_number = eerom_serial;
		}
		if (bus->serial_number_str = kmalloc (9, GFP_KERNEL)) {
			sprintf (bus->serial_number_str, "%08X", eerom_serial & 0xffffffff);
		}
		//dbg_udc(0,  "serial: %s %08x", bus->serial_number_str, bus->serial_number);
		return 0;
	}
#endif
	return -EINVAL;

}


/* ********************************************************************************************* */

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
 * @logical_endpoint:
 *
 * Return physical endpoint number to use for this logical endpoint or zero if not valid.
 */
int udc_check_ep (int logical_endpoint, int packetsize)
{
	if (packetsize > 64) {
		return 0;
	}
	switch (logical_endpoint) {
	case 1:
		return 1;
	case 0x82:
		return 2;
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
	dbg_udc (1, "[%d]:", ep);

	if (ep < UDC_MAX_ENDPOINTS) {

		ep_endpoints[ep] = endpoint;
                dbg_udc (1, "[%d] ep_endpoint %p endpoint: %p", ep, ep_endpoints[ep], endpoint);


		if (endpoint) {
			switch (ep) {
			case 0:	// Control
				ep0_enable (device, endpoint);
				break;

			case 1:	// OUT
                                dbg_udc (1, "[%d]: CHECKING rcv_urb: %p", ep, endpoint->rcv_urb);
				// XXX XXX 
                                if (!endpoint->rcv_urb) {
                                        usbd_fill_rcv (device, endpoint, 5);
                                        endpoint->rcv_urb = first_urb_detached (&endpoint->rdy);
                                        dbg_udc (1, "[%d]: SETTING rcv_urb: %p", ep, endpoint->rcv_urb);
                                }
				ep1_enable (device, endpoint, usbd_rcv_dma);
				break;

			case 2:	// IN
				ep2_enable (device, endpoint, usbd_tx_dma);
				break;
			}
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
	dbg_udc (1, "[%d]:", ep);
	if (ep < UDC_MAX_ENDPOINTS) {
		struct usb_endpoint_instance *endpoint;

		if ((endpoint = ep_endpoints[ep])) {
			ep_endpoints[ep] = NULL;
			usbd_flush_ep (endpoint);
		}
		switch (ep) {
		case 0:
			ep0_disable ();
			break;
		case 1:
			ep1_disable ();
			break;
		case 2:
			ep2_disable ();
			break;
		}
	}
}

/* ********************************************************************************************* */

/**
 * udc_incradle - is the USB cable connected
 *
 * Return non-zeron if cable is connected.
 */
int udc_connected ()
{
	int rc = 1;
#ifdef CONFIG_SA1100_USBCABLE_GPIO

#ifdef CONFIG_SA1100_USBCABLE_ACTIVE_HIGH
	rc = (GPLR & GPIO_GPIO (23)) != 0;
	dbg_udc (1, "udc_connected: ACTIVE_HIGH: %d", rc);
#else
	rc = (GPLR & GPIO_GPIO (23)) == 0;
	dbg_udc (1, "udc_connected: ACTIVE_LOW: %d", rc);
#endif
#else
#warning UDC_CONNECTED not implemented
#endif
	return rc;
}


/**
 * udc_connect - enable pullup resistor
 *
 * Turn on the USB connection by enabling the pullup resistor.
 */
void udc_connect (void)
{

#if defined(CONFIG_SA1100_COLLIE)	// XXX change to 5500

        #warning COLLIE CONNECT
        if (ucb1200_test_io(TC35143_GPIO_VERSION0) != 0 ||
                        ucb1200_test_io(TC35143_GPIO_VERSION1) != 0) {
                SCP_REG_GPCR |= SCP_GPCR_PA19;    /* direction : out mode */
                SCP_REG_GPWR |= SCP_GPCR_PA19;    /* set Hi */
                dbg_udc(1, "udc_connect: %d GPCR: %x GPWR: %x GPRR: %x",
                                SCP_GPCR_PA19, SCP_REG_GPCR, SCP_REG_GPWR, SCP_REG_GPRR);
        }

#elif defined(CONFIG_SA1100_CONNECT_GPIO)

	GAFR |= GPIO_GPIO (CONFIG_SA1100_CONNECT_GPIO);
	GPDR |= GPIO_GPIO (CONFIG_SA1100_CONNECT_GPIO);

        #ifdef CONFIG_SA1100_CONNECT_ACTIVE_HIGH
	GPSR = GPIO_GPIO (CONFIG_SA1100_CONNECT_GPIO);
	dbg_udc (1, "udc_connect: %d ACTIVE_HIGH %x %x %x", CONFIG_SA1100_CONNECT_GPIO,
		 GPIO_GPIO (CONFIG_SA1100_CONNECT_GPIO), GPDR, GPLR);
        #else
	GPCR = GPIO_GPIO (CONFIG_SA1100_CONNECT_GPIO);
	dbg_udc (1, "udc_connect: %d ACTIVE_LOW %x %x %x", CONFIG_SA1100_CONNECT_GPIO,
		 GPIO_GPIO (CONFIG_SA1100_CONNECT_GPIO), GPDR, GPLR);
        #endif

#else				/* defined(CONFIG_SA1100_CONNECT_GPIO) */

#warning UDC_CONNECT not implemented

#endif

}


/**
 * udc_disconnect - disable pullup resistor
 *
 * Turn off the USB connection by disabling the pullup resistor.
 */
void udc_disconnect (void)
{

#if defined(CONFIG_SA1100_COLLIE)	// XXX change to 5500

        if (ucb1200_test_io(TC35143_GPIO_VERSION0) != 0 ||
                        ucb1200_test_io(TC35143_GPIO_VERSION1) != 0) {
                SCP_REG_GPWR &= ~SCP_GPCR_PA19;    /* set Hi */
                SCP_REG_GPCR &= ~SCP_GPCR_PA19;    /* direction : out mode */

                dbg_udc(1, "udc_disconnect: %d GPCR: %x GPWR: %x GPRR: %x",
                                SCP_GPCR_PA19, SCP_REG_GPCR, SCP_REG_GPWR, SCP_REG_GPRR);
        }


#elif defined(CONFIG_SA1100_CONNECT_GPIO)

#ifdef CONFIG_SA1100_CONNECT_ACTIVE_HIGH
	GPCR = GPIO_GPIO (CONFIG_SA1100_CONNECT_GPIO);
	dbg_udc (1, "udc_disconnect: %d ACTIVE_HIGH", CONFIG_SA1100_CONNECT_GPIO);
#else
	GPSR = GPIO_GPIO (CONFIG_SA1100_CONNECT_GPIO);
	dbg_udc (1, "udc_disconnect: %d ACTIVE_LOW", CONFIG_SA1100_CONNECT_GPIO);
#endif

	GPDR &= ~GPIO_GPIO (CONFIG_SA1100_CONNECT_GPIO);

#endif
}


/* ********************************************************************************************* */

static __inline__ void set_interrupts (int flags)
{
	int ok;
	//dbg_udc(1, "udc_set_interrupts: %02x", flags);

	// set interrupt mask
	SET_AND_TEST ((*(UDCCR) = flags), (udc (UDCCR) != flags), ok);
	if (!ok) {
		dbg_intr (0, "failed to set UDCCR %02x to %02x", *(UDCCR), flags);
	}
}

/**
 * udc_enable_interrupts - enable interrupts
 *
 * Switch on UDC interrupts.
 *
 */
void udc_all_interrupts (struct usb_device_instance *device)
{
	set_interrupts (0);
	dbg_udc (1, "%02x", *(UDCCR));

	// setup tick
#ifdef USE_ADD_DEL_TIMER_FOR_USBADDR_CHECK
	if (!usb_addr_check_initialized) {
		init_timer (&sa1100_usb_dev_addr_check);
		usb_addr_check_initialized = 1;
		sa1100_usb_dev_addr_check.function = sa1100_tick;
		sa1100_usb_dev_addr_check.data = 1;
		sa1100_usb_dev_addr_check.expires = jiffies + CHECK_INTERVAL;
		add_timer (&sa1100_usb_dev_addr_check);
	}
#else
	// XXX sa1100_tq.sync = 0;
	sa1100_tq.routine = sa1100_tick;
	sa1100_tq.data = &udc_saw_sof;
	queue_task (&sa1100_tq, &tq_timer);
#endif

}


/**
 * udc_suspended_interrupts - enable suspended interrupts
 *
 * Switch on only UDC resume interrupt.
 *
 */
void udc_suspended_interrupts (struct usb_device_instance *device)
{
	set_interrupts (UDCCR_SRM);
	dbg_udc (1, "%02x", *(UDCCR));
}


/**
 * udc_disable_interrupts - disable interrupts.
 *
 * switch off interrupts
 */
void udc_disable_interrupts (struct usb_device_instance *device)
{
	set_interrupts (UDCCR_SRM | UDCCR_EIM | UDCCR_RIM | UDCCR_TIM | UDCCR_SRM | UDCCR_REM);
	dbg_udc (1, "%02x", *(UDCCR));
}

/* ********************************************************************************************* */

/**
 * udc_ep0_packetsize - return ep0 packetsize
 */
int udc_ep0_packetsize (void)
{
	return EP0_PACKETSIZE;
}

/**
 * udc_enable - enable the UDC
 *
 * Switch on the UDC
 */
void udc_enable (struct usb_device_instance *device)
{
	int ok;
	dbg_udc (1, "************************* udc_enable:");

	//dbg_udc(0, "");
	//dbg_udc(0, "udc_enable: device: %p", device);

	// save the device structure pointer
	udc_device = device;

	// enable UDC
	for (ok = 0; (*(UDCCR) & UDCCR_UDA) && ok < 100000; ok++);
	if (ok > 10000) {
		dbg_udc (0, "Waiting too long to go inactive: UDCCR: %02x", *(UDCCR));
	}
	// set UDD
	SET_AND_TEST ((*(UDCCR) |= UDCCR_UDD), (!udc (UDCCR) & UDCCR_UDD), ok);
	if (!ok) {
		dbg_udc (0, "Waiting too long to disable: UDCCR: %02x", *(UDCCR));
	}
	// reset UDD
	SET_AND_TEST ((*(UDCCR) &= ~UDCCR_UDD), (udc (UDCCR) & UDCCR_UDD), ok);
	if (!ok) {
		dbg_udc (0, "can't enable udc, FAILED: %02x", *(UDCCR));
	}
}


/**
 * udc_disable - disable the UDC
 *
 * Switch off the UDC
 */
void udc_disable (void)
{
	int ok;
	dbg_udc (1, "************************* udc_disable:");

	// tell tick task to stop
#ifdef USE_ADD_DEL_TIMER_FOR_USBADDR_CHECK
	del_timer (&sa1100_usb_dev_addr_check);
	sa1100_usb_dev_addr_check.data = 0;
	usb_addr_check_initialized = 0;
#else
	sa1100_tq.data = NULL;
	while (sa1100_tq.sync) {
		//printk(KERN_DEBUG"waiting for sa1100_tq to stop\n");
		schedule_timeout (10 * HZ);
	}
#endif

	// disable UDC

	for (ok = 0; (*(UDCCR) & UDCCR_UDA) && ok < 100000; ok++);
	if (ok > 10000) {
		dbg_udc (0, "Waiting too long to go inactive: UDCCR: %02x", *(UDCCR));
	}
	// set UDD
	SET_AND_TEST ((*(UDCCR) |= UDCCR_UDD), (!udc (UDCCR) & UDCCR_UDD), ok);
	if (!ok) {
		dbg_udc (0, "Waiting too long to disable: UDCCR: %02x", *(UDCCR));
	}

	dbg_udc (7, "CCR: %02x", *(UDCCR));

	// reset device pointer
	udc_device = NULL;
}


/**
 * udc_startup - allow udc code to do any additional startup
 */
void udc_startup_events (struct usb_device_instance *device)
{
	usbd_device_event (device, DEVICE_INIT, 0);
	usbd_device_event (device, DEVICE_CREATE, 0);
	usbd_device_event (device, DEVICE_HUB_CONFIGURED, 0);
	usbd_device_event (device, DEVICE_RESET, 0);	// XXX should be done from device event
}


/* ********************************************************************************************* */

/**
 * udc_name - return name of USB Device Controller
 */
char *udc_name (void)
{
	return UDC_NAME;
}

/**
 * udc_request_udc_irq - request UDC interrupt
 *
 * Return non-zero if not successful.
 */
int udc_request_udc_irq ()
{
	// request IRQ and two dma channels
	// XXX GPDR &= ~GPIO_GPIO(23);
	// XXX set_GPIO_IRQ_edge(GPIO_GPIO(CONFIG_SA1100_USBCABLE_GPIO), GPIO_BOTH_EDGES);

	dbg_init (2, "requesting udc irq: %d %d", 13, IRQ_Ser0UDC);

	return request_irq (IRQ_Ser0UDC, int_hndlr_device, SA_INTERRUPT | SA_SAMPLE_RANDOM,
			    "SA1100 USBD Bus Interface", NULL);
}

/**
 * udc_request_cable_irq - request Cable interrupt
 *
 * Return non-zero if not successful.
 */
int udc_request_cable_irq ()
{
#ifdef CONFIG_SA1100_USBCABLE_GPIO
	dbg_init (2, "requesting cable irq: %d %d",
		  CONFIG_SA1100_USBCABLE_GPIO, SA1100_GPIO_TO_IRQ (CONFIG_SA1100_USBCABLE_GPIO));
	GPDR &= ~GPIO_GPIO (23);
	set_GPIO_IRQ_edge (GPIO_GPIO (CONFIG_SA1100_USBCABLE_GPIO), GPIO_BOTH_EDGES);
	return request_irq (SA1100_GPIO_TO_IRQ (CONFIG_SA1100_USBCABLE_GPIO), int_hndlr_cable,
			    SA_INTERRUPT | SA_SAMPLE_RANDOM, "SA1100 Monitor", NULL);
#else
	return 0;
#endif
}

/**
 * udc_request_udc_io - request UDC io region
 *
 * Return non-zero if not successful.
 */
int udc_request_io ()
{
	//sa1100_bi_init_dma ();

        usbd_rcv_dma = usbd_tx_dma = -1;

	if (sa1100_request_dma (&usbd_rcv_dma, "SA1100 USBD OUT - rcv dma", DMA_Ser0UDCRd)) {
		dbg_init (2, "channel %d taken", usbd_rcv_dma);
		return -EINVAL;
	}


	if (sa1100_request_dma (&usbd_tx_dma, "SA1100 USBD IN - tx dma", DMA_Ser0UDCWr)) {
		dbg_init (2, "channel %d taken", usbd_tx_dma);
		sa1100_free_dma (usbd_rcv_dma);
		return -EINVAL;
	}

	return 0;
}

/**
 * udc_release_udc_irq - release UDC irq
 */
void udc_release_udc_irq ()
{
	free_irq (IRQ_Ser0UDC, NULL);
}

/**
 * udc_release_cable_irq - release Cable irq
 */
void udc_release_cable_irq ()
{
#ifdef CONFIG_SA1100_USBCABLE_GPIO
	dbg_init (2, "freeing cable irq: %d", CONFIG_SA1100_USBCABLE_GPIO);
	free_irq (SA1100_GPIO_TO_IRQ (CONFIG_SA1100_USBCABLE_GPIO), NULL);
#endif
}

/**
 * udc_release_io - release UDC io region
 */
void udc_release_io ()
{
	sa1100_free_dma (usbd_rcv_dma);
	sa1100_free_dma (usbd_tx_dma);
}

/* ep0 dma engine functions ******************************************************************** */

/* 
 * EP0 State - See the Intel Application Note
 */

#define EP0_STATE_IDLE 0
#define EP0_STATE_IN_DATA_PHASE 1
#define EP0_STATE_END_IN 2

char *ep0_state_desc[] = {
	"idle", "in", "end"
};

static int ep0_state = EP0_STATE_IDLE;

//struct urb ep0_urb;
struct urb *ep0_urb;


static unsigned char *buffer;
static unsigned int buffer_cnt;


/*
 * There are two scenarios: 
 *
 *   1. Data to be sent is an exact multiple of the packetsize and less than
 *   the requested size. An empty packet needs to be sent.
 *
 *   2. Everything else. The last packet will be recognized as the last
 *   because it is less than the packetsize OR because we have sent exactly
 *   the required amount of data.
 */

static unsigned int send_zlp;

struct usb_endpoint_instance *ep0_endpoint;

char *send_zlp_description[] = {
	"exact", "forced"
};

int ep0_enable (struct usb_device_instance *device, struct usb_endpoint_instance *endpoint)
{
	ep0_endpoint = endpoint;

        dbg_ep0(0, "ep0_urb: %p", ep0_urb);
	if (!ep0_urb) {
		if (!(ep0_urb = usbd_alloc_urb (device, device->function_instance_array, 0, 512))) {
			dbg_ep0 (1, "ep0_enable: usbd_alloc_urb failed\n");
		}
	} else {
		dbg_ep0 (1, "ep0_enable: ep0_urb already allocated\n");
	}
	return 0;
}

void ep0_disable (void)
{
        dbg_ep0(0, "ep0_urb: %p", ep0_urb);
	if (ep0_urb) {
		usbd_dealloc_urb (ep0_urb);
		ep0_urb = 0;
	} else {
		dbg_ep0 (1, "ep0_disable: ep0_urb already NULL\n");
	}
}

int ep0_reset (void)
{
	dbgENTER (dbgflg_usbdbi_ep0, 1);
	ep0_state = EP0_STATE_IDLE;
	udc_reset_ep (0);
	return 0;
}

/* 
 * ep0_set_opr
 *
 * Set OPR and optionally DE.
 */
static void ep0_set_opr (void)
{
        int ok;
        IOIOIO(UDCCS0, UDCCS0_S0, *(UDCCS0) = UDCCS0_SO , (*(UDCCS0) & UDCCS0_OPR), ok);
        if (!ok) {
                dbg_ep0(0, "failed to reset OPR");
        }
        dbg_ep0(1,"CS0: %02x ok: %d", *(UDCCS0), ok);
}

/* 
 * ep0_set_opr_and_de
 *
 * Set OPR and optionally DE.
 */
static void ep0_set_opr_and_de (void)
{
        int ok;
        IOIOIO(UDCCS0, UDCCS0_S0, *(UDCCS0) = (UDCCS0_SO | UDCCS0_DE) , (*(UDCCS0) & UDCCS0_OPR), ok);
        if (!ok) {
                dbg_ep0(0, "failed to reset OPR");
        }
        dbg_ep0(1,"CS0: %02x ok: %d", *(UDCCS0), ok);
}


/*
 * ep0_read_data
 *
 * Retreive setup data from FIFO.
 */
__inline__ static int ep0_read_data (struct usb_device_request *request, int max)
{
	int fifo_count;
	int bytes_read;
	int i;
	int reads;
	unsigned char *cp = (unsigned char *) request;
        int udcwc;
        int udccs0;

        udccs0 = *(UDCCS0);
	udcwc = fifo_count = udc(UDCWC) & 0xff;

	if (fifo_count != max) {
		dbg_ep0 (0, "ep0_read_data: have %d bytes; want max\n", fifo_count);
	}

	reads = 0;
	bytes_read = 0;

	for (reads = 0, bytes_read = 0; udcwc && fifo_count--;) {
		i = 0;
		do {
			*cp = (unsigned char) *(UDCD0);
                        // XXX without this reads may be corrupt, typically the wLength field
                        // will contain bad data, which in turn will cause get descriptor config
                        // to return the full set of descriptors in response to limited first
                        // get descriptor 9, which will BSOD windows
			udelay(10);
			reads++;
			i++;
		} while ((((udcwc = udc(UDCWC) & 0xff)) > fifo_count) && (i < 10));

		if (i == 10) {
			dbg_ep0 (0, "failed: UDCWC: %2x %2x bytes: %d fifo: %d reads: %d no change\n", 
                                        udcwc, *(UDCWC), bytes_read, fifo_count, reads);
		}

                if (udcwc < (fifo_count - 1)) {
			dbg_ep0 (0, "failed: UDCWC: %2x %2x bytes: %d fifo: %d reads: %d too low\n", 
                                        udcwc, *(UDCWC), bytes_read, fifo_count, reads);
                }

		cp++;
		bytes_read++;

		if ((bytes_read >= max) && (fifo_count > 0)) {
			dbg_ep0 (0, "reading too much data\n");
			break;
		}
	}

        dbg_ep0 ((bytes_read == max)?2:0, 
                        "ep0_read_data: reads: %d want: %d fifo_count: %d bytes_read: %d CS0: %02x:%02x CWC: %02x",
                        reads, max, fifo_count, bytes_read, *(UDCCS0), udccs0, udcwc);

	return bytes_read;
}


/* *
 * ep0_read_request
 *
 * Called when in IDLE state to get a setup packet.
 */
static void ep0_read_request (unsigned int udccs0, unsigned int udccr)
{
	int i;
	int ok;

	if (!(udccr & UDCCR_UDA)) {
		dbg_ep0 (3, "warning, UDC not active");
	}
	// reset SST
	if (udccs0 & UDCCS0_SST) {
		IOIOIO (UDCCS0, UDCCS0_SST, *(UDCCS0) = UDCCS0_SST, *(UDCCS0) & UDCCS0_SST, ok);
		if (!ok) {
			dbg_ep0 (0, "failed to reset SST CS0: %02x ok: %d", *(UDCCS0), ok);
		}
	}
	// check UDC Endpoint 0 Control/Status register for OUT Packet Ready (OPR) (c.f. 11.8.7.1)
	if (!(udccs0 & UDCCS0_OPR)) {
		// wait for another interrupt
		dbg_ep0 (3, "wait for another interrupt CS0: %02x", udccs0);
		return;
	}

	// read a device request
	ep0_urb->actual_length = 0;
	if ((i = ep0_read_data (&ep0_urb->device_request, 8)) < 8) {
		ep0_set_opr ();
		dbg_ep0 (2, "not enough data: %d", i);
		return;
	}
	//ep0_need_data = le16_to_cpu(ep0_urb->device_request.wLength);
	dbg_ep0 (3, "bmRequestType:%02x bRequest:%02x wValue:%04x wIndex:%04x wLength:%04x",
		 ep0_urb->device_request.bmRequestType,
		 ep0_urb->device_request.bRequest,
		 le16_to_cpu (ep0_urb->device_request.wValue),
		 le16_to_cpu (ep0_urb->device_request.wIndex),
		 le16_to_cpu (ep0_urb->device_request.wLength));

	{
		char *cp = (char *) &ep0_urb->device_request;
		dbg_ep0 (1, "%02x %02x %02x %02x %02x %02x %02x %02x",
			 cp[0], cp[1], cp[2], cp[3], cp[4], cp[5], cp[6], cp[7]
		    );
	}

	// check if we have been initialized
	if (!ep0_urb->device) {
		ep0_set_opr_and_de ();
		udc_stall_ep (0);
		return;
	}

	// check direction of data transfer
        if ((ep0_urb->device_request.bmRequestType & USB_REQ_DIRECTION_MASK) == USB_REQ_HOST2DEVICE) {

                // if this is a set address save it
                if ((ep0_urb->device_request.bmRequestType == 0) && (ep0_urb->device_request.bRequest == USB_REQ_SET_ADDRESS)) {

                        usb_address = le16_to_cpu(ep0_urb->device_request.wValue) & 0xff;

                        /*
                         * very occasionally register write will fail, delay and redo twice
                         * to ensure that it actually got through, cannot check as UDC
                         * will not propagate until SO and DE are set. We also need a delay
                         * after setting SO and DE, exiting too soon can also result in
                         * problems if address has not propagated.
                         */


                        *(UDCAR) = usb_address;
                        udelay(2);
                        *(UDCAR) = usb_address;
                        udelay(2);
                        *(UDCAR) = usb_address;
                        ep0_set_opr_and_de();   
                        udelay(40);            

                        if (usbd_recv_setup(ep0_urb)) {
                                dbg_ep0(5, "usb_recv_setup failed, not stalling");
                        }
                }
                else {
                        //ep0_set_opr_and_de();
                        // submit urb to ep0, stall if non-zero result
                        if (usbd_recv_setup(ep0_urb)) {
                                dbg_ep0(1, "usb_recv_setup failed, stalling");
                                udc_stall_ep(0);
                        }
                        ep0_set_opr_and_de(); // tbr getting better results with this.
                }
                return;
	}

	// submit urb to ep0, stall if non-zero result
	if (usbd_recv_setup (ep0_urb)) {
		dbg_ep0 (2, "usb_recv_setup failed, stalling");
		ep0_set_opr_and_de ();
		udc_stall_ep (0);
		return;
	}

	// device reqeust has specified Device-to-Host, so we should be returning data

	// check request length, zero is not legal
	if (!le16_to_cpu (ep0_urb->device_request.wLength)) {
		dbg_ep0 (0, "wLength zero, stall");
		ep0_set_opr_and_de ();
		udc_stall_ep (0);
		return;
	}
	// check that we have some data to send back, zero should not be possible
	if (!ep0_urb->actual_length) {
		dbg_ep0 (0, "no data, stall");
		ep0_set_opr_and_de ();
		udc_stall_ep (0);
		return;
	}
	// everything looks sane, so set buffer pointers and setup for data transfer
	buffer = ep0_urb->buffer;
	buffer_cnt = ep0_urb->actual_length;

	/*
	 * IFF we are sending a multiple of the packet size AND less than was
	 * requested we will need to send an empty packet after all the data packets.
	 */
	if (!(buffer_cnt % 8) && (buffer_cnt < le16_to_cpu (ep0_urb->device_request.wLength))) {
		send_zlp = 1;
		dbg_ep0 (1, "DEVICE2HOST: SEND ZLP %x %x", ep0_urb->actual_length,
			 le16_to_cpu (ep0_urb->device_request.wLength));
	} else {
		send_zlp = 0;
		dbg_ep0 (2, "DEVICE2HOST: NO ZLP %x %x", ep0_urb->actual_length,
			 le16_to_cpu (ep0_urb->device_request.wLength));
	}

	ep0_state = EP0_STATE_IN_DATA_PHASE;
}

/* 
 * ep0_in
 *
 * Send some data to host.
 */
static void ep0_in (unsigned int udccs0)
{
	int transfer_cnt = 0;
	unsigned int fifo_count;
	volatile int writes = 0;
	unsigned long start = jiffies;
	int ok;

	udccs0 = *(UDCCS0);
	dbg_ep0 (2, "CS0: %02x", udccs0);

	// check for non empty FIFO
	if (*(UDCWC)) {
		dbg_ep0 (0, "FIFO not empty");
		return;
	}
	// process iff ep0 is not stalled, no premature end and IPR is clear
	if (udccs0 & (UDCCS0_SE | UDCCS0_SST)) {
		dbg_ep0 (1, "ep0_in: SE or SST set\n");
		return;
	}
	// first check that IPR is not set, if it is then we wait for it to clear.
	if (udccs0 & UDCCS0_IPR) {

		for (ok = 0; (*(UDCCS0) & UDCCS0_IPR) && ok < 2000; ok++) {
			udelay (25);	// XXX 
		}
		if (ok == 2000) {
			dbg_ep0 (0, "IPR clear timed out");
		}
		dbg_ep0 (0, "IPR was set count: %d", ok);
	}
	// get up to first 8 bytes of data
	transfer_cnt = MIN (8, buffer_cnt);
	buffer_cnt -= transfer_cnt;
	fifo_count = 0;

	do {
		int count = 0;
                int udcwc;
		do {
			*(UDCD0) = (unsigned long) buffer[fifo_count];
			count++;
			writes++;
		} while (((udcwc = udc (UDCWC)) == fifo_count) && (count < 2000));

		if (udcwc <= fifo_count) {
			dbg_ep0 (0,
				 "sending STALL, failed writing FIFO transfer: %d fifo_count: %x CS0: %02x CWC: %02x",
				 transfer_cnt, fifo_count, *(UDCCS0), udcwc);
			udc_stall_ep (0);
			ep0_state = EP0_STATE_IDLE;
			return;
		}

		fifo_count++;

	} while (fifo_count < transfer_cnt);

	dbg_ep0 (3, "elapsed: %lx writes: %d fifo_count: %d transfer_cnt: %d buffer_cnt: %d CS0: %02x CWC: %02x",
		 jiffies - start, writes, fifo_count, transfer_cnt, buffer_cnt, *(UDCCS0), *(UDCWC));

	buffer += transfer_cnt;

	// set data end, and start 
	if (buffer_cnt == 0) {
		ep0_state = EP0_STATE_END_IN;
	}

	IOIOIO (UDCCS0, UDCCS0_IPR,
		*(UDCCS0) = (UDCCS0_IPR | ((((!buffer_cnt) && (send_zlp != 1)) ? UDCCS0_DE : 0))),
		!(*(UDCCS0) & UDCCS0_IPR), ok);

	if (!ok) {
		dbg_ep0 (0, "failed to set IPR CS0: %02x ok: %d", *(UDCCS0), ok);
	}
}

/*
 * ep0_end_in
 *
 * Handle end of IN, possibly with ZLP.
 */
static void ep0_end_in (unsigned int udccs0)
{
	int ok;

	dbg_ep0 (2, " status: %02x CS0: %02x", udccs0, *(UDCCS0));

	// reset SST if necessary
	if (udccs0 & UDCCS0_SST) {
		IOIOIO (UDCCS0, UDCCS0_SST, *(UDCCS0) = UDCCS0_SST, *(UDCCS0) & UDCCS0_SST, ok);
		if (!ok) {
			dbg_ep0 (0, "failed to reset SST");
		}
	}
	// reset SE if necessary
	if (udccs0 & UDCCS0_SE) {
		IOIOIO (UDCCS0, UDCCS0_SSE, *(UDCCS0) = UDCCS0_SSE, *(UDCCS0) & UDCCS0_SSE, ok);
		if (!ok) {
			dbg_ep0 (0, "failed to reset SE");
		}
	}
	// ensure that IPR is not set and send zero length packet if necessary
	if (!(udc (UDCCS0) & UDCCS0_IPR) && (send_zlp == 1)) {
                IOIOIO (UDCCS0, UDCCS0_DE | UDCCS0_IPR, 
                                *(UDCCS0) = (UDCCS0_DE | UDCCS0_IPR), 
                                (*(UDCCS0) & (UDCCS0_DE | UDCCS0_IPR)) != (UDCCS0_DE | UDCCS0_IPR), ok);
                if (!ok) {
                        dbg_ep0 (0, "failed to set DE and IPR CS0: %02x", *(UDCCS0));
                }
                else {
                        dbg_ep0 (1, "set DE and IPR, ZLP sent CS0: %02x", *(UDCCS0));
                }
	}
	// set state to IDLE
	ep0_state = EP0_STATE_IDLE;
	send_zlp = 0;
}



/* 
 * ep0_int_hndlr
 *
 * Handle ep0 interrupts.
 *
 * Generally follows the example in the Intel Application Note.
 *
 */
void ep0_int_hndlr (unsigned int status)
{
	unsigned int udccs0;
	unsigned int udccr;

	udccs0 = udc (UDCCS0);
	udccr = udc (UDCCR);

	ep0_interrupts++;

	dbg_ep0 (4, "------------>[%d:%s:%s CCR: %02x CS0: %02x CWC: %02x CAR: %02x]",
		 ep0_interrupts, ep0_state_desc[ep0_state], send_zlp_description[send_zlp], udccr,
		 udccs0, *(UDCWC), *(UDCAR));

	if (udccs0 == UDCCS0_IPR) {
		dbg_ep0 (3, "IPR SET [%d:%02x]", ep0_state, udccs0);
		return;
	}

	if (udccs0 & UDCCS0_SST) {
		dbg_ep0 (3, "previously sent stall [%d:%02x]", ep0_state, udccs0);
	}
	// SE 
	if (udccs0 & UDCCS0_SE) {
		int ok;
		dbg_ep0 (3, "unloading, found UDCCS0_SE: CS0: %02x", udccs0);

		// reset SE
		IOIOIO (UDCCS0, UDCCS0_SSE, *(UDCCS0) = UDCCS0_SSE, *(UDCCS0) & UDCCS0_SE, ok);
		if (!ok) {
			dbg_ep0 (0, "failed to reset SE CS0: %02x ok: %d", *(UDCCS0), ok);
		}
		// clear SE before unloading any setup packets 
		if (udccs0 & UDCCS0_OPR) {
			// reset OPR
			*(UDCCS0) = UDCCS0_SO;
		}
		ep0_state = EP0_STATE_IDLE;
                if (send_zlp) {
			dbg_ep0 (0, "not sending ZLP CS0: %02x", *(UDCCS0));
                }
		send_zlp = 0;
	}

	if ((ep0_state != EP0_STATE_IDLE) && (udccs0 & UDCCS0_OPR)) {
		dbg_ep0 (3, "Forcing EP0_STATE_IDLE CS0: %02x", udccs0);
		ep0_state = EP0_STATE_IDLE;
	}

	switch (ep0_state) {
	case EP0_STATE_IDLE:
		dbg_ep0 (4, "state: EP0_STATE_IDLE CS0: %02x", udccs0);
		ep0_read_request (udccs0, udccr);
		if (ep0_state != EP0_STATE_IN_DATA_PHASE) {
			break;
		}
		// fall through to data phase
		ep0_set_opr ();

	case EP0_STATE_IN_DATA_PHASE:
		dbg_ep0 (4, "state: EP0_STATE_IN_DATA_PHASE %x", udccs0);
		ep0_in (udccs0);
		break;

	case EP0_STATE_END_IN:
		dbg_ep0 (4, "state: EP0_STATE_END_IN %x", udccs0);
		ep0_end_in (udccs0);
		break;
	}
	dbg_ep0 (4, "<------------[%s:%s CCR: %02x CS0: %02x CWC: %02x]",
		 ep0_state_desc[ep0_state], send_zlp_description[send_zlp], *(UDCCR), *(UDCCS0),
		 *(UDCWC));
}

/* ep1 dma engine functions ******************************************************************** */

static int dmachn_rx;		// rx dma channel
static int dma_enabled;		// dma channel has been setup
static int dma_is_active;		// dma engine is running
static int dma_rx_active;		// dma engine is running

static dma_addr_t dma_rx_curpos;
struct usb_device_instance *ep1_device;
struct usb_endpoint_instance *ep1_endpoint;

/* *
 * ep1_stop_dma - stop the DMA engine
 *
 * Stop the DMA engine. 
 */
static void ep1_stop_dma (struct urb *urb)
{
	if (urb && dma_enabled) {
		unsigned long flags;
		save_flags_cli (flags);
		if (!dma_is_active) {
			restore_flags (flags);
			dbg_rx (1, "dma OFF: %x %x %x", dmachn_rx, dma_enabled, dma_is_active);
			return;
		}

		dma_is_active = 0;
		restore_flags (flags);

		if (ep1_endpoint->rcv_packetSize > 20) {
			_sa1100_bi_dma_flush_all_irq(dmachn_rx);
		}
	}
}


/* *
 * ep1_start_dma - start the DMA engine running
 *
 * Start the DMA engine. This should allow incoming data to be stored
 * in the DMA receive buffer.
 *
 */
static void ep1_start_dma (struct urb *urb)
{
	if (urb && dma_enabled) {
		unsigned long flags;
		int ok;
		save_flags_cli (flags);
		if (dma_is_active) {
			restore_flags (flags);
			dbg_rx (1, "dma ON: %x %x %x", dmachn_rx, dma_enabled, dma_is_active);
			return;
		}

		dma_is_active = 1;

		dbg_rx (1, "urb: %p urb->buffer: %p actual_length: %d rcv_packetSize: %d",
			urb, urb->buffer, urb->actual_length, urb->endpoint->rcv_packetSize);

		if (!(dma_rx_curpos = pci_map_single (NULL, (void *) urb->buffer + urb->actual_length,
						   urb->endpoint->rcv_packetSize,
						   PCI_DMA_FROMDEVICE))) {
			restore_flags (flags);
			dbg_rx (0, "pci_map_single failed");
			return;
		}

		SET_AND_TEST (*(UDCCS1) = UDCCS1_RPC, _udc (UDCCS1) & UDCCS1_RPC, ok);


		// enable DMA only if packet to big for FIFO (twenty bytes)
		if (ep1_endpoint->rcv_packetSize > 20) {
			dma_rx_active = 
                            _sa1100_bi_dma_queue_buffer_irq (dmachn_rx, dma_rx_curpos, urb->endpoint->rcv_packetSize);
			_sa1100_bi_dma_run (dmachn_rx, dma_rx_active);
		}

		restore_flags (flags);
	}
}


/* ep1 public functions ************************************************************************ */

/**
 * ep1_init - initialize the endpoint
 *
 */
void ep1_enable (struct usb_device_instance *device, struct usb_endpoint_instance *endpoint,
		 int chn)
{
	if (endpoint) {
		int ok;
		ep1_device = device;
		ep1_endpoint = endpoint;
		dmachn_rx = chn;

                dbg_rx (1, "ep1_endpoint %p", ep1_endpoint);

		//usbd_fill_rcv(device, endpoint, 5);

		SET_AND_TEST (*(UDCOMP) =
			      ep1_endpoint->rcv_packetSize - 1,
			      udc (UDCOMP) != ep1_endpoint->rcv_packetSize - 1, ok);
		if (!ok) {
			dbg_rx (0, "FAILED setting pktsize %d", *(UDCOMP));
		}
		// setup the dma engine operating parameters
		dma_is_active = 0;
		dma_enabled = 1;
                if (!ep1_endpoint->rcv_urb) {
                        ep1_endpoint->rcv_urb = first_urb_detached (&ep1_endpoint->rdy);
                        dbg_rx (1, "SETTING ep1_endpoint->rcv_urb: %p", ep1_endpoint->rcv_urb);
                }
                else {
                        dbg_rx (1, "CHECKING ep1_endpoint->rcv_urb: %p", ep1_endpoint->rcv_urb);
                }
		ep1_start_dma (ep1_endpoint->rcv_urb);
	}
}


/**
 * ep1_reset - reset the endpoint
 *
 * Return non-zero if error.
 */
void ep1_reset (void)
{
	int ok;

	dbg_rx (1, "CHECKING ep1_endpoint->rcv_urb: %p", ep1_endpoint->rcv_urb);

	ep1_stop_dma (ep1_endpoint->rcv_urb);

	if (ep1_endpoint->rcv_urb) {
		ep1_endpoint->rcv_urb->actual_length = 0;
	}
	ep1_start_dma (ep1_endpoint->rcv_urb);

	udc_reset_ep (1);
	SET_AND_TEST (*(UDCOMP) =
		      ep1_endpoint->rcv_packetSize - 1,
		      udc (UDCOMP) != ep1_endpoint->rcv_packetSize - 1, ok);
	if (!ok) {
		dbg_rx (0, "FAILED setting pktsize %d", *(UDCOMP));
	}
}


/**
 * ep1_disable - disable endpoint for use
 *
 */
void ep1_disable ()
{
	dbg_rx (1, "dma_is_active: %d dma_enabled: %d", dma_is_active, dma_enabled);

	if (dma_is_active) {
		ep1_stop_dma (ep1_endpoint->rcv_urb);
	}

	if (!dma_enabled) {
		dbg_rx (1, "!dma_enabled: %x %x %x", dmachn_rx, dma_enabled, dma_is_active);
		return;
	}
	{
		unsigned long flags;
		save_flags_cli (flags);
		if (dma_is_active) {
			restore_flags (flags);
			dbg_rx (1, "dma_is_active: %x %x %x", dmachn_rx, dma_enabled, dma_is_active);
			ep1_stop_dma (ep1_endpoint->rcv_urb);
			usbd_dealloc_urb (ep1_endpoint->rcv_urb);
                        dbg_rx(1, "CLEARING rcv_urb %p", ep1_endpoint->rcv_urb);
			ep1_endpoint->rcv_urb = NULL;
		}
		dma_enabled = 0;
		restore_flags (flags);
	}

	usbd_flush_rcv (ep1_endpoint);
	ep1_endpoint = NULL;
}


/*
 * Endpoint 1 interrupts will be directed here.
 *
 * status : the contents of the UDC Status/Interrupt Register.
 */
void ep1_int_hndlr (unsigned int status)
{
	dma_addr_t dma_address;
	unsigned int udccs1;
	int ok;
	unsigned int len = 0;
	unsigned int urb_bad = 0;

	rx_interrupts++;

	if (!((udccs1 = *(UDCCS1)) & UDCCS1_RPC)) {
		SET_AND_TEST (*(UDCCS1) = UDCCS1_SST, _udc (UDCCS1) & UDCCS1_SST, ok);
		SET_AND_TEST (*(UDCCS1) = UDCCS1_RPC, _udc (UDCCS1) & UDCCS1_RPC, ok);
		return;
	}
	//dbg_rx(4, "udccs1: %02x", udccs1);

	dma_is_active = 0;

	// DMA or non-DMA
	// XXX - we should be able to collapse these together
	if (ep1_endpoint->rcv_packetSize <= 20) {

		// non-DMA version
		if (!(udccs1 & (UDCCS1_RPE))) {

			// get residual data from fifo
			if (ep1_endpoint->rcv_urb) {
				unsigned char *dmabuf;

				dmabuf = ep1_endpoint->rcv_urb->buffer + ep1_endpoint->rcv_urb->actual_length;

				// XXX Note that it is apparantly impossible to do this with 100%
				// reliability. Only copy data in if/while length is less than packet size

				for (;
				     (len < (unsigned) ep1_endpoint->rcv_packetSize)
				     && *(UDCCS1) & UDCCS1_RNE; len++) {
					*dmabuf++ = *(UDCDR);
					//udelay(1);
				}
				// record an error if the FIFO is not empty
				udc_ep1_errors += ((urb_bad = *(UDCCS1) & UDCCS1_RNE)) ? 1 : 0;
			} else {
				dbg_rx (0, "NO RCV URB");
			}
		} else {
			udc_rpe_errors++;
		}
	}
	// retrieve dma_address, note that this is not reliable
	else if (!_sa1100_bi_dma_stop_get_current_irq (dmachn_rx, &dma_address, dma_rx_active)) {

		// DMA version
		// stop dma engine, if we can get the current address, unmap, get residual data, and give to bottom half
		pci_unmap_single (NULL, dma_rx_curpos, ep1_endpoint->rcv_packetSize,
				  PCI_DMA_FROMDEVICE);

		if (!(udccs1 & (UDCCS1_RPE))) {

			// get residual data from fifo
			if (ep1_endpoint->rcv_urb) {
				unsigned char *dmabuf;

				len = dma_address - dma_rx_curpos;
				dmabuf = ep1_endpoint->rcv_urb->buffer + ep1_endpoint->rcv_urb->actual_length + len;

				/* WARNING - it is impossible empty the FIFO with 100% accuracy. 
				 *
				 * We only copy data in if/while length is less than packet size
				 *
				 * There are two problems. First fetching the dma address can fail. In this case len will
				 * be too large and we do not attempt to read any data.
				 *
				 * Second, reads sometimes fail to update the FIFO read address resulting in too much
				 * data being retrieved. Slightly delaying between each read helps to reduce this problem
				 * at the expense of increasing latency.
				 *
				 * This (and the similiar problem with the TX FIFO) are the reason that it is NOT SAFE
				 * to operate the SA1100 without a CRC across all bulk transfers. The normal USB CRC which
				 * the hardware uses to protect each packet being sent is not sufficent. The data is being
				 * damaged getting it into and out of the FIFO(s).
				 */

				for (;
				     (len < (unsigned) ep1_endpoint->rcv_packetSize)
				     && *(UDCCS1) & UDCCS1_RNE; len++) {
					*dmabuf++ = *(UDCDR);
					udelay (0);
				}

				// record an error if the FIFO is not empty
				udc_ep1_errors += ((urb_bad = *(UDCCS1) & UDCCS1_RNE)) ? 1 : 0;
			} else {
				dbg_rx (0, "NO RCV URB");
			}
		} else {
			udc_rpe_errors++;
		}
	} else {
		//dbg_rx(0, "nothing to unmap");
	}

	// let the upper levels handle the urb
	usbd_rcv_complete_irq (ep1_endpoint, len, urb_bad);

	// clear SST if necessary
	if (udccs1 & UDCCS1_SST) {
		SET_AND_TEST (*(UDCCS1) = UDCCS1_SST, _udc (UDCCS1) & UDCCS1_SST, ok);
		if (!ok) {
			dbg_rx (0, "could not clear SST: %02x", *(UDCCS1));
		}
	}
	// start DMA and reset RPC 
	SET_AND_TEST (*(UDCCS1) = UDCCS1_RPC, _udc (UDCCS1) & UDCCS1_RPC, ok);
	if (!ok) {
		dbg_rx (0, "could not clear RPC: %02x", *(UDCCS1));
	}

	if (dma_enabled && ep1_endpoint->rcv_urb) {
		dma_is_active = 1;

		// start DMA if packetsize to big for FIFO (twenty bytes)
		if ((dma_rx_curpos =
		     pci_map_single (NULL,
				     (void *) ep1_endpoint->rcv_urb->buffer +
				     ep1_endpoint->rcv_urb->actual_length,
				     ep1_endpoint->rcv_packetSize, PCI_DMA_FROMDEVICE))
		    && (ep1_endpoint->rcv_packetSize > 20)) 
                {
			dma_rx_active = _sa1100_bi_dma_queue_buffer_irq (dmachn_rx, dma_rx_curpos, ep1_endpoint->rcv_packetSize);
			_sa1100_bi_dma_run (dmachn_rx, dma_rx_active);
		}
	}
}

/* ep2 dma engine functions ******************************************************************** */



static int dmachn_tx;		// tx dma channel
static int dma_tx_active;
dma_addr_t dma_tx_curpos;

static struct usb_endpoint_instance *ep2_endpoint;

/* *
 * ep2_reset_dma - 
 * @restart:
 *
 * Reset the DMA engine parameters. 
 */
static void ep2_reset_dma (void)
{
	unsigned long flags;

	local_irq_save (flags);

	// stop current dma
	_sa1100_bi_dma_flush_all_irq (dmachn_tx);
	udelay (10);		// XXX

	ep2_endpoint->tx_urb = NULL;
	if (dma_tx_curpos) {
		pci_unmap_single (NULL, dma_tx_curpos, ep2_endpoint->last, PCI_DMA_TODEVICE);
	}
	dma_tx_curpos = (dma_addr_t) NULL;
	ep2_endpoint->tx_urb = NULL;
	ep2_endpoint->sent = 0;

	usbd_flush_tx (ep2_endpoint);

	local_irq_restore (flags);
}


/* ep2 public functions ************************************************************************ */

/**
 * ep2_enable -
 *
 * Initialize the dma variables. Called once.
 *
 */
int ep2_enable (struct usb_device_instance *device, struct usb_endpoint_instance *endpoint, int chn)
{
	dbgENTER (dbgflg_usbdbi_tx, 1);
	dmachn_tx = chn;
	ep2_disable ();
	//ep2_device = device;
	{
		unsigned long flags;
		local_irq_save (flags);
		ep2_endpoint = endpoint;
		dma_tx_curpos = 0;
		ep2_endpoint->sent = 0;
		ep2_endpoint->tx_urb = NULL;
		local_irq_restore (flags);
	}
	return 0;
}


/*
 * Initialise the DMA system to use the given chn for receiving
 * packets over endpoint 2.
 *
 * chn     : the dma channel to use.
 * returns : 0 if ok, -1 if channel couldn't be used.
 */
void ep2_disable (void)
{
	unsigned long flags;
	dbgENTER (dbgflg_usbdbi_tx, 1);
	local_irq_save (flags);
	if (ep2_endpoint) {
		ep2_reset_dma ();
		ep2_endpoint = NULL;
	}
	local_irq_restore (flags);
}


/*
 * reset any txpkts, it will be up to the client driver to free up the send
 * queue.
 */
void ep2_reset (void)
{
	dbgENTER (dbgflg_usbdbi_tx, 1);
	ep2_reset_dma ();
	udc_reset_ep (2);
}


/* Interrupt service routines ****************************************************************** */


/*
 *
 * Interrupt handler - called with interrupts disabled.
 */
void ep2_int_hndlr (unsigned int status, int in_interrupt)
{
	unsigned int udccs2;
	int ok;
	int restart = 0;

	udccs2 = *(UDCCS2);

	// do not do anything if FST is set
	if (udccs2 & UDCCS2_FST) {
		dbg_tx (0, "FST set! (%sint)", in_interrupt ? "" : "not-");
		udc_ep2_fst++;
		return;
	} else if (udccs2 & UDCCS2_SST) {
		dbg_tx (0, "SST set, stalling! (%sint)", in_interrupt ? "" : "not-");
		udc_stall_ep (2);
		return;
	}
	// if in interrupt we have to stop any current activity
	if (in_interrupt) {

		tx_interrupts++;

		if (!((udccs2 = *(UDCCS2)) & UDCCS2_TPC)) {
			return;
		}

		_sa1100_bi_dma_flush_all_irq (dmachn_tx);

		// if requred reset stall and restart 
		if (udccs2 & (UDCCS2_TPE | UDCCS2_TUR | UDCCS2_SST)) {

			if (udccs2 & UDCCS2_SST) {
				SET_AND_TEST (*(UDCCS2) =
					      UDCCS2_SST, _udc (UDCCS2) & UDCCS2_SST, ok);
				if (!ok) {
					dbg_tx (0, "Waiting too long to set SST %02x", *(UDCCS2));
				}
			}
			udc_ep2_errors++;
			restart = 1;
			if (udccs2 & UDCCS2_TPE) {
				udc_ep2_tpe++;
			}
			//QQQ was: if (udccs2 & UDCCS2_TUR | UDCCS2_SST) 
			if (udccs2 & UDCCS2_TUR) {
				udc_ep2_tur++;
			}
			if (udccs2 & UDCCS2_SST) {
				udc_ep2_sst++;
			}

		}
		// if we have an active buffer, umap the buffer and update position if previous send was successful
		if (dma_tx_curpos) {
			pci_unmap_single (NULL, dma_tx_curpos, ep2_endpoint->last, PCI_DMA_TODEVICE);
			dma_tx_curpos = (dma_addr_t) NULL;
		}
	}

	usbd_tx_complete_irq (ep2_endpoint, restart);

	// start dma if we have something to send
	if (ep2_endpoint->tx_urb
	    && ((ep2_endpoint->tx_urb->actual_length - ep2_endpoint->sent) > 0)) {
		int ok1, ok2;
		//int count;
		ep2_endpoint->last = MIN (ep2_endpoint->tx_urb->actual_length - ep2_endpoint->sent,
					  ep2_endpoint->tx_urb->endpoint->tx_packetSize);

		if (!
		    (dma_tx_curpos =
		     pci_map_single (NULL, ep2_endpoint->tx_urb->buffer + ep2_endpoint->sent,
				     ep2_endpoint->last, PCI_DMA_TODEVICE))) {
			dbg_tx (0, "pci_map_single failed");
		}

		/* The order of the following is critical....
		 * Reseting the TPC clears the FIFO, so the DMA engine cannot be started until after
		 * TPC is set. But this opens a window where the USB UCD can fail because it is
		 * waiting for the DMA engine to fill the FIFO and it receives an IN packet from
		 * the host. The best we can do is to minimize the window. [Note that this effect
		 * is most noticeable on slow, 133Mhz, processors.]
		 *
		 * 1. Set IMP and all DMA registers, but do not start DMA yet
		 * 2. reset TPC
		 * 3. start DMA
		 */
		SET_AND_TEST (*(UDCIMP) =
			      ep2_endpoint->last - 1, _udc (UDCIMP) != (ep2_endpoint->last - 1),
			      ok1);
		if (ep2_endpoint->last > 16) {
			dma_tx_active = _sa1100_bi_dma_queue_buffer_irq (dmachn_tx, dma_tx_curpos, ep2_endpoint->last);

			//for (count = 8; count--; *(UDCDR) = '\0');

			SET_AND_TEST (*(UDCCS2) = UDCCS2_TPC, _udc (UDCCS2) & UDCCS2_TPC, ok2);
			_sa1100_bi_dma_run (dmachn_tx, dma_tx_active);

			if (!ok1 || !ok2) {
				dbg_tx (0, "Waiting too long to set UDCIMP %02x or TPC: %02x",
					*(UDCIMP), *(UDCCS2));
			}
			dbg_tx (4, "queued %d bytes using DMA", ep2_endpoint->last);
		} else {

			// Entire packet is small enough to fit in FIFO, stuff it
			// in directly and forget DMA.

			unsigned char *dmabuf = (unsigned char *) dma_tx_curpos;
			int count = ep2_endpoint->last;

			SET_AND_TEST (*(UDCCS2) = UDCCS2_TPC, _udc (UDCCS2) & UDCCS2_TPC, ok2);
			for (; count--; *(UDCDR) = *dmabuf++);

			dbg_tx (4, "queued %d bytes directly to FIFO", ep2_endpoint->last);
		}
	}
}

/* Clock Tick Debug support ****************************************************************** */


#define RETRYTIME 10
#if defined(CONFIG_USBD_STALL_TIMEOUT)
  // If an URB waits more than...
#define USBD_STALL_TIMEOUT_SECONDS        CONFIG_USBD_STALL_TIMEOUT
#else
#define USBD_STALL_TIMEOUT_SECONDS        0	// Stall watchdog hobbled
#endif

#if defined(CONFIG_USBD_STALL_DISCONNECT_DURATION)
#define USBD_STALL_DISCONNECT_DURATION    CONFIG_USBD_STALL_DISCONNECT_DURATION
#else
#define USBD_STALL_DISCONNECT_DURATION    2	// seconds
#endif

char hexdigit (int hex)
{
	hex &= 0xf;
	return (hex < 0xa) ? ('0' + hex) : ('a' + hex - 0xa);
}

int hex2buf (char *buf, char *str, int num)
{
	char *cp = buf;
	while (*str) {
		*cp++ = *str++;
	}
	*cp++ = hexdigit (num >> 4);
	*cp++ = hexdigit (num & 0xf);
	*cp++ = ' ';
	return (cp - buf);
}

extern int send_length;
extern int send_todo;
extern int sent_last;

static void show_info (void)
{
	char buf[100];
	char *cp;
	volatile short int gpdr;
/*
    if (udc_interrupts_last == udc_interrupts) {
        udc_irq_enable();
    }
*/
	if (udc_interrupts != udc_interrupts_last) {
		printk (KERN_DEBUG "--------------\n");
	}
	// do some work
	memset (buf, 0, sizeof (buf));
	cp = buf;
	cp += hex2buf (cp, "CCR:", *(UDCCR));
	cp += hex2buf (cp, "CSR:", *(UDCSR));
	cp += hex2buf (cp, "CAR:", *(UDCAR));
	cp += hex2buf (cp, "CS0:", *(UDCCS0));
	cp += hex2buf (cp, "CS1:", *(UDCCS1));
	cp += hex2buf (cp, "CS2:", *(UDCCS2));
	cp += hex2buf (cp, "IMP:", *(UDCIMP));
	cp += hex2buf (cp, "OMP:", *(UDCOMP));
	cp += hex2buf (cp, "WC:", *(UDCWC));
	cp += hex2buf (cp, "SR:", *(UDCSR));

	gpdr = GPDR;
	printk (KERN_DEBUG
		"[%u] %s int:%2d ep0:%d rx:%d tx:%d sus:%d res:%d er:%d re:%d e1:%d e2:%d tpe:%d tur:%d sst:%d fst:%d tck:%d fix:%d\n",
		udc_interrupts, buf, udc_interrupts - udc_interrupts_last,
		ep0_interrupts - ep0_interrupts_last,
		rx_interrupts - rx_interrupts_last,
		tx_interrupts - tx_interrupts_last,
		sus_interrupts, res_interrupts,
		udc_address_errors,
		udc_rpe_errors,
		udc_ep1_errors,
		udc_ep2_errors,
		udc_ep2_tpe, udc_ep2_tur, udc_ep2_sst, udc_ep2_fst, udc_ticks, udc_fixed);
	udc_interrupts_last = udc_interrupts;
	ep0_interrupts_last = ep0_interrupts;
	rx_interrupts_last = rx_interrupts;
	tx_interrupts_last = tx_interrupts;
}

void udc_regs (void)
{
	if (_udc (UDCAR) != usb_address) {
		printk (KERN_DEBUG "ADDRESS ERROR DETECTED\n");
		udc_address_errors++;
	}
	*(UDCAR) = usb_address;

	show_info ();
}

