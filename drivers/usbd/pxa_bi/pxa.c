/*
 * usbd/pxa_bi/pxa.c -- Xscale USB Device Controller driver. 
 *
 *      Copyright (c) 2004 Belcarra
 *
 * Adapted from earlier work:
 *      Copyright (c) 2002, 2003 Belcarra
 *      Copyright (c) 2000, 2001, 2002 Lineo
 *
 * By: 
 *      Stuart Lynne <sl@belcarra.com>, 
 *      Tom Rushworth <tbr@belcarra.com>, 
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
 *
 * Notes
 *
 * 1. This code was developed on the Intel Lubbock with Cotulla PXA250 processor.
 *
 * 2. Currently this is being tested with the PXA-255, it should work with the older PXA-250
 *    but I cannot guarantee it.
 *
 * 3. IO on the PXA is slow, keep the number of accesses to a minimum.
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/netdevice.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/types.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <asm/hardware.h>
#include <asm/dma.h>

#include <usbd-export.h>
#include <usbd-build.h>
#include <usbd-chap9.h>
#include <usbd-mem.h>
#include <usbd.h>
#include <usbd-bus.h>
#include <trace.h>
#include <usbd-bi.h>
#include "asm/arch/pxa-regs.h"
#include "pxa.h"

MODULE_AUTHOR ("sl@belcarra.com, tbr@belcarra.com");
MODULE_DESCRIPTION ("Xscale USB Device Bus Interface");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,17)
MODULE_LICENSE("GPL");
#endif
USBD_MODULE_INFO ("pxa_bi 2.0-beta"); 
const char *usbd_bi_module_info(void)
{
	return __usbd_module_info;
}

u32 udc_interrupts;
static u32 udc_suspended;
u32 pxa_cpu_rev;
u32 pxa_have_udccfr;
u32 udc_in_int;

/* Map logical to physical
 */
typedef enum ep {
        ep_control, ep_bulk_in, ep_bulk_out, ep_iso_in, ep_iso_out, ep_interrupt
} ep_t;

/* PXA has lots of endpoints, but they have fixed address and type so logical to physical map is
 * limited to masking top bits so we can find appropriate info.
 */
__u8 epl2p[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, };
__u8 epp2l[16] = { 0x00, 0x81, 0x02, 0x83, 0x04, 0x85, 0x86, 0x07, 0x88, 0x09, 0x8a, 0x8b, 0x0c, 0x8d, 0x0e, 0x8f, };

u32 _UDDRN[16] = {
        0x40600080, 0x40600100, 0x40600180, 0x40600200, 0x40600400, 0x406000A0, 0x40600600, 0x40600680,
        0x40600700, 0x40600900, 0x406000C0, 0x40600B00, 0x40600B80, 0x40600C00, 0x40600E00, 0x406000E0, 
};

u32 _UBCRN[16] = {
        0,          0,          0x40600068, 0, 0x4060006c, 0,          0,          0x40600070,
        0,          0x40600074, 0,          0, 0x40600078, 0,          0x4060007c, 0, 
};

#define UDCCSN(x)       __REG2(0x40600010, (x) << 2)
#define UDDRN(x)        __REG(_UDDRN[x])
#define UBCRN(x)        __REG(_UBCRN[x])

struct ep_map {
        int             logical;
        ep_t            eptype;
        int             size;
        int             dma_chan;
};

static struct ep_map ep_maps[16] = {
        { logical: 0, eptype: ep_control,   size: 16, }, { logical: 1, eptype: ep_bulk_in,   size: 64, },
        { logical: 2, eptype: ep_bulk_out,  size: 64, }, { logical: 3, eptype: ep_iso_in,   size: 256, },
        { logical: 4, eptype: ep_iso_out,  size: 256, }, { logical: 5, eptype: ep_interrupt , size: 8, },
        { logical: 6, eptype: ep_bulk_in,   size: 64, }, { logical: 7, eptype: ep_bulk_out,  size: 64, }, 
        { logical: 8, eptype: ep_iso_in,   size: 256, }, { logical: 9, eptype: ep_iso_out,  size: 256, },
        { logical: 10, eptype: ep_interrupt, size: 8, }, { logical: 11, eptype: ep_bulk_in,  size: 64, },
        { logical: 12, eptype: ep_bulk_out, size: 64, }, { logical: 13, eptype: ep_iso_in,  size: 256, },
        { logical: 14, eptype: ep_iso_out, size: 256, }, { logical: 15, eptype: ep_interrupt, size: 8, },
};

static __inline__ void pxa_enable_ep_interrupt(int ep)
{
        ep &= 0xf;
        if (ep < 8) 
                UICR0 &= ~(1 << ep);
        else 
                UICR1 &= ~(1 << (ep - 7));
}

static __inline__ void pxa_disable_ep_interrupt(int ep)
{
        ep &= 0xf;
        if (ep < 8) 
                UICR0 = 1 << ep;
        else 
                UICR1 = 1 << (ep - 7);
}

static __inline__ void pxa_ep_reset_irs(int ep)
{
        ep &= 0xf;
        if (ep < 8) 
                USIR0 = (1 << ep);
        else 
                USIR1 = (1 << (ep - 7));
}
/* ********************************************************************************************* */
/* pxa_out_n
 */
static __inline__ void pxa_out_n(int ep, struct usb_endpoint_instance *endpoint, u32 cs)
{
        while ((cs = UDCCSN(ep)) & UDCCS_BO_RPC/*|UDCCS_B0_RFS*/) {
                struct urb *rcv_urb = bi_rcv_next_irq(endpoint);
                int len = 0;
                unsigned char *cp = endpoint->rcv_urb->buffer + endpoint->rcv_urb->actual_length;
                int count = MIN(UBCRN(ep) + 1, endpoint->wMaxPacketSize);
                if (!rcv_urb) {
                        pxa_disable_ep_interrupt(ep);
                        break;
                }
                for (cp = endpoint->rcv_urb->buffer + endpoint->rcv_urb->actual_length; count--; *cp++ = UDDRN(ep), len++);
                bi_rcv_complete_irq (endpoint, len, 0);
                UDCCSN(ep) = UDCCS_BO_RPC;
        }
}

/* pxa_in_n
 */
static void __inline__ pxa_in_n (unsigned int ep, struct usb_endpoint_instance *endpoint)
{
        int udccsn;
        int j = 0;
        while ((udccsn = UDCCSN(ep)) & (UDCCS_BI_TFS | UDCCS_BI_TPC | UDCCS_BI_TUR)) {
                struct urb *tx_urb;
                int last;
                int size;
                u8 *cp;
                TRACE_MSG32("IN: UDCCSN: %x", udccsn);
                BREAK_IF(j++>4);
                if (udccsn & UDCCS_BI_TPC) {                            // if TPC update tx urb and clear TPC
                        TRACE_MSG("IN: CLEAR BI_TPC");
                        UDCCSN(ep) = UDCCS_BI_TPC;                      // clear TPC
                }
                if (udccsn & UDCCS_BI_TUR) { 
                        TRACE_MSG("IN: CLEAR BI_TUR");
                        UDCCSN(ep) = UDCCS_BI_TUR;                      // clear underrun, not much we can do about it
                }
                CONTINUE_IF (!(udccsn & UDCCS_BI_TFS));
                TRACE_MSG("IN: BI_TFS SET");
                BREAK_IF (!(tx_urb = bi_tx_complete_irq(endpoint, 0)));    // nothing to send

                TRACE_MSG16("IN: length: %d sent: %d", tx_urb->actual_length, endpoint->sent);
                BREAK_IF (!(last = MIN (tx_urb->actual_length - (endpoint->sent + endpoint->last), endpoint->wMaxPacketSize)));
                for ( size = last, cp = tx_urb->buffer + endpoint->sent + endpoint->last; size--; UDDRN(ep) = *cp++); 
                if (( last < endpoint->wMaxPacketSize ) || ( (endpoint->tx_urb->actual_length - endpoint->sent ) == last )) 
                        UDCCSN(ep) = UDCCS_BI_TSP;
                endpoint->last = last;
        }
}
/* ********************************************************************************************* */
/* pxa_ep0_in 
 */
void pxa_ep0_in(struct usb_endpoint_instance *endpoint, u32 udccs0)
{
        struct urb *tx_urb = bi_tx_next_irq(endpoint);
        int size;
        u8 *cp;
        TRACE_MSG32("          --> xmit: UDCCS0: %02x", udccs0);
        if (!tx_urb) {
                UDCCS0 = UDCCS0_FTF | UDCCS0_OPR;
                endpoint->state = WAIT_FOR_SETUP;
                TRACE_MSG32("          <-- xmit premature status: UDCCS0: %02x", UDCCS0);
        }
        if ((udccs0 & UDCCS0_OPR) && !(UDCCS0 & UDCCS0_SA) ) {                  // check for premature status stage?
                UDCCS0 = UDCCS0_FTF | UDCCS0_OPR;                               // clear tx fifo and opr
                endpoint->state = WAIT_FOR_SETUP;
                endpoint->tx_urb = NULL;
                bi_tx_cancelled_irq(endpoint);
                TRACE_MSG32("          <-- xmit premature status: UDCCS0: %02x", UDCCS0);
                return;
        }
        if (udccs0 & UDCCS0_SST) {                                              // check for stall
                UDCCS0 = UDCCS0_SST | UDCCS0_FTF;                               // clear stall and tx fifo
                endpoint->state = WAIT_FOR_SETUP;
                bi_tx_cancelled_irq(endpoint);
                TRACE_MSG32("          <-- xmit stall: UDCCS0: %02x", UDCCS0);
                return;
        }
        endpoint->last = size = MIN (tx_urb->actual_length - endpoint->sent, endpoint->wMaxPacketSize);
        TRACE_MSG32("xmit: %d", size);
        for (cp = tx_urb->buffer + endpoint->sent ; size--; UDDRN(0) = *cp++);             // Stuff the FIFO
        if (endpoint->last < endpoint->wMaxPacketSize) {                        // short packet?
                UDCCS0 = UDCCS0_IPR;                                            // set IPR if this is a short packet
                endpoint->state = WAIT_FOR_OUT_STATUS;
                TRACE_MSG32("          <-- xmit wait for status: UDCCS0: %02x", UDCCS0);
        }
        TRACE_MSG32("          <-- xmit not finished: UDCCS0: %02x", UDCCS0);
}

/* pxa_out_ep0 -
 */
static __inline__ void pxa_out_ep0(int ep, struct usb_endpoint_instance *endpoint, u32 udccs0)
{
        struct urb *rcv_urb = bi_rcv_next_irq(endpoint);
        int len = 0;
        u8 *cp;
        TRACE_MSG32("          --> recv: UDCCS0: %02x", udccs0);
        RETURN_IF (!((udccs0 & UDCCS0_OPR) && !(UDCCS0 & UDCCS0_SA)));
        for (cp = endpoint->rcv_urb->buffer + endpoint->rcv_urb->actual_length;
                        (UDCCS0 & UDCCS0_RNE) && (len < endpoint->wMaxPacketSize); len++, *cp++ = UDDR0);
        TRACE_MSG16("         <--  ep0 recv: packetsize: %02x len: %02x", endpoint->wMaxPacketSize, len);
        if (!bi_rcv_complete_irq(endpoint, len, 0)) 
                endpoint->state = WAIT_FOR_SETUP;
        UDCCS0 = UDCCS0_IPR | UDCCS0_OPR;
}

/* pxa_ep0_setup
 */
void __inline__ pxa_ep0_setup(struct usb_endpoint_instance *endpoint, volatile u32 udccs0)
{
        struct usb_device_request request;
        int len = 0;
        int max = 8;
        u8 *cp = (u8 *)&request;
        RETURN_IF ((udccs0 & (UDCCS0_SA | UDCCS0_OPR | UDCCS0_RNE)) != (UDCCS0_SA | UDCCS0_OPR | UDCCS0_RNE));
        TRACE_MSG32("      --> setup: UDCCS0: %02x", udccs0);
        for (; max-- /*&& (UDCCS0 & UDCCS0_RNE)*/; *cp++ = UDDR0, len++);               // read setup packet
        udc_in_int = 1;
        if (bi_recv_setup_irq(&request)) {                                              // setup processing failed 
                UDCCS0 = UDCCS0_FST;
                endpoint->state = WAIT_FOR_SETUP;
                TRACE_MSG32("      --> bad setup FST: UDCCS0: %02x", udccs0);
                return;
        }
        udc_in_int = 0;
        if ((request.bmRequestType & USB_REQ_DIRECTION_MASK) == USB_REQ_HOST2DEVICE) {  // check direction
                endpoint->state = DATA_STATE_RECV;
                UDCCS0 = UDCCS0_IPR;
                TRACE_MSG32("      <-- H2D setup nodata: UDCCS0: %02x",UDCCS0);
        }
        else {                                                   // Control Read - we are sending data to the host
                endpoint->state = DATA_STATE_XMIT;
                TRACE_MSG32("      <-- D2H setup: UDCCS0: %02x",UDCCS0);
                RETURN_IF(!bi_tx_next_irq(endpoint));
                pxa_ep0_in(endpoint, UDCCS0);                                           // start sending
                UDCCS0 = UDCCS0_SA | UDCCS0_OPR;                                        // Clear SA and OPR bits
                TRACE_MSG32("      <-- setup data: CLEAR SA AND OPR UDCCS0: %02x", UDCCS0);
        }
}

/* pxa_ep0_int
 */
static __inline__ void pxa_ep0_int(struct usb_endpoint_instance *endpoint, volatile u32 udccs0)
{
        int j = 0;
        TRACE_MSG32("  --> ep0: UDCCS0: %02x", udccs0);
        if (udccs0 & UDCCS0_SST) {
                UDCCS0 = UDCCS0_SST;
                TRACE_MSG32("  --> ep0 clear SST: UDCCS0: %02x", udccs0);
                bi_tx_cancelled_irq(endpoint);
                return;
        }
        if (!endpoint) {
                UDCCS0 = UDCCS0_IPR | UDCCS0_OPR | UDCCS0_SA;
                TRACE_MSG32("  ep0 NULL: UDCCS0: %02x", UDCCS0);
                return;
        }
        if (bi_tx_complete_irq(endpoint, 0)) {
                if (bi_tx_sendzlp(endpoint))
                        endpoint->state = DATA_STATE_NEED_ZLP;
        }
        if ((endpoint->state != WAIT_FOR_SETUP) && (udccs0 & UDCCS0_SA)) {
                TRACE_MSG32("  --> ep0 early SA: UDCCS0: %02x", udccs0);
                endpoint->state = WAIT_FOR_SETUP;
                endpoint->tx_urb = NULL;
        }
        switch (endpoint->state) {
        case DATA_STATE_NEED_ZLP:
                UDCCS0 = UDCCS0_IPR;
                endpoint->state = WAIT_FOR_OUT_STATUS;
                break;
        case WAIT_FOR_OUT_STATUS:
                if ((udccs0 & (UDCCS0_OPR | UDCCS0_SA)) == UDCCS0_OPR) 
                        UDCCS0 |= UDCCS0_OPR;
                TRACE_MSG32("  --> ep0 WAIT for STATUS: UDCCS0: %02x", UDCCS0);
                endpoint->state = WAIT_FOR_SETUP;
        case WAIT_FOR_SETUP:
        case DATA_STATE_RECV:
                do {
                        TRACE_MSG32("  UDCCS0: %02x", UDCCS0);
                        switch (endpoint->state) {
                        case DATA_STATE_RECV:
                                pxa_out_ep0(0, endpoint, UDCCS0);
                                UDCCS0 |= UDCCS0_OPR;
                                break;
                        case WAIT_FOR_SETUP:
                                bi_tx_cancelled_irq(endpoint);
                                pxa_ep0_setup(endpoint, UDCCS0);
                                break;
                        }
                        if (udccs0 & UDCCS0_SST) {
                                UDCCS0 = UDCCS0_SST | UDCCS0_OPR | UDCCS0_SA;
                                TRACE_MSG32("  --> ep0 clear SST: UDCCS0: %02x", udccs0);
                                bi_tx_cancelled_irq(endpoint);
                                endpoint->state = WAIT_FOR_SETUP;
                                return;
                        }
                        if (j++ > 2) {
                                u32 udccs0 = UDCCS0;
                                TRACE_MSG32("  UDCCS0 wait: UDCCS0: %02x", udccs0);
                                if ((udccs0 & (UDCCS0_OPR | UDCCS0_SA | UDCCS0_RNE)) == (UDCCS0_OPR | UDCCS0_SA)) {
                                        UDCCS0 = UDCCS0_OPR | UDCCS0_SA;
                                        TRACE_MSG32("  ep0 force: UDCCS0: %02x", UDCCS0);
                                }
                                else {
                                        UDCCS0 = UDCCS0_OPR | UDCCS0_SA;
                                        TRACE_MSG32("  ep0 force and return: UDCCS0: %02x", UDCCS0);
                                        break;
                                }
                        }
                } while (UDCCS0 & (UDCCS0_OPR | UDCCS0_RNE) && (j++ < 100));
                break;
        case DATA_STATE_XMIT:
                pxa_ep0_in(endpoint, UDCCS0);
                break;
        }
        if (pxa_have_udccfr) 
                UDCCFR = UDCCFR_AREN;
}
/* ********************************************************************************************* */
/* pxa_int_hndlr - interrupt handler
 */
static void pxa_int_hndlr (int irq, void *dev_id, struct pt_regs *regs)
{
        u32 udccr;
	udc_interrupts++;
        TRACE_MSG16("------------------------> USIRO: %02x UDCCR: %02x", USIR0, UDCCR);
        if ((udccr = UDCCR) & (UDCCR_RSTIR | UDCCR_RESIR | UDCCR_SUSIR)) {              // uncommon interrupts
                if (udccr & UDCCR_RSTIR) {                                              // UDC Reset
                        TRACE_MSG32("------------------------> Reset: UDCCR: %02x", udccr);
                        udc_suspended = 0;
                        usbd_bus_event_irq (usbd_bus, DEVICE_RESET, 0);
                        usbd_bus_event_irq (usbd_bus, DEVICE_ADDRESS_ASSIGNED, 0);	
                        UDCCR |= UDCCR_RSTIR;
                }
                if (udccr & UDCCR_RESIR) {                                              // UDC Resume
                        TRACE_MSG32("------------------------> Resume: UDCCR: %02x", udccr);
                        if (udc_suspended) {
                                udc_suspended = 0;
                                usbd_bus_event_irq (usbd_bus, DEVICE_BUS_ACTIVITY, 0);
                        }
                        UDCCR |= UDCCR_RESIR;
                }
                if (udccr & UDCCR_SUSIR) {                                              // UDC Suspend
                        TRACE_MSG32("------------------------> Suspend: UDCCR: %02x", udccr);
                        if (!udc_suspended) {
                                udc_suspended = 1;
                                usbd_bus_event_irq (usbd_bus, DEVICE_BUS_INACTIVE, 0);
                        }
                        UDCCR |= UDCCR_SUSIR;
                }
        }
        for (;;) {                                                                      // endpoint interrupts
                int ep;
                u32 usirn;
                if ((usirn = USIR0)) 
                        ep = 0;
                else if ((usirn = USIR1))
                        ep = 8;
                else 
                        break;
                for (; usirn; usirn >>= 1, ep++) {
                        CONTINUE_IF (!(usirn & 1));
                        switch (ep_maps[ep].eptype) {
                        case ep_control:
                                pxa_ep0_int(usbd_bus->endpoint_array + 0, UDCCS0);
                                break;
                        case ep_bulk_in:
                        case ep_interrupt:
                                pxa_in_n(ep, usbd_bus->endpoint_array + ep);
                                break;
                        case ep_bulk_out:
                                pxa_out_n(ep, usbd_bus->endpoint_array + ep, UDCCSN(ep));
                                break;
                        case ep_iso_in:
                        case ep_iso_out:
                                break;
                        }
                        if (ep < 8) 
                                USIR0 = (1 << ep);
                        else 
                                USIR1 = (1 << (ep - 7));
                }
        }
        if (UFNRH & UFNHR_SIR)                                                  // sof interrupt
                UFNRH = UFNHR_SIR;
}
/* ********************************************************************************************* */
/* udc_start_endpoint_in - start transmit
 */ 
void udc_start_endpoint_in(struct usb_endpoint_instance *endpoint)
{       
        int epn = endpoint->bEndpointAddress & 0xf;
        TRACE_MSG16("START IN %02x %d", endpoint->bEndpointAddress, epn);
        TRACE_MSG32("START IN %p", (u32)endpoint->tx_urb);
        switch(endpoint->bmAttributes & USB_ENDPOINT_MASK) {
        case USB_ENDPOINT_CONTROL:
                if (!udc_in_int)
                        pxa_ep0_in(endpoint, UDCCS0);                                           // start sending
                break;
        case USB_ENDPOINT_BULK:
        case USB_ENDPOINT_INTERRUPT:
                //if (UDCCSN(epn) & UDCCS_BI_TFS) 
                        pxa_in_n (epn, endpoint);
                break;
        case USB_ENDPOINT_ISOCHRONOUS:
                break;
        }
}

/* udc_start_endpoint_out - start receive
 */
void udc_start_endpoint_out(struct usb_endpoint_instance *endpoint)
{
        int epn = endpoint->bEndpointAddress & 0xf;
        TRACE_MSG16("START OUT %02x %d", endpoint->bEndpointAddress, epn);
        TRACE_MSG32("START OUT %p", (u32)endpoint->rcv_urb);
        switch(endpoint->bmAttributes & USB_ENDPOINT_MASK) {
        case USB_ENDPOINT_BULK:
        case USB_ENDPOINT_INTERRUPT:
                pxa_enable_ep_interrupt(epn);
                break;
        case USB_ENDPOINT_ISOCHRONOUS:
                break;
        }
}

void udc_cancel_in_irq(struct urb *urb)
{
        int ep = urb->endpoint->bEndpointAddress & 0xf;
        TRACE_MSG32("IN FLUSH: %d", ep);
        UDCCSN(ep) = UDCCS_BI_FTF;
        UDCCSN(ep) = UDCCS_BI_TSP;
}

void udc_cancel_out_irq(struct urb *urb)
{
}

/* udc_init - initialize
 */
int udc_init (void)
{
        /* get pxa cpu rev - see arch/arm/mach-pxa/cpu-pxa.c
         */
        asm("mrc%? p15, 0, %0, c0, c0" : "=r" (pxa_cpu_rev));
        TRACE_MSG32("PXA CPU: %08x", pxa_cpu_rev);
        pxa_cpu_rev &= (CP15R0_NUM_MASK | CP15R0_REV_MASK);
        switch (pxa_cpu_rev) {
#undef PXA_FAKE_OLD
#ifndef PXA_FAKE_OLD
        case PXA255_A0:
                TRACE_MSG("PXA-255");
                pxa_have_udccfr = 1;
#endif
        }
        udc_disable_interrupts ();
	return 0;
}

/* udc_stall_ep - stall endpoint
 */
void udc_stall_ep (unsigned int ep)
{
}

/* udc_reset_ep - reset endpoint
 */
void udc_reset_ep (unsigned int ep)
{
}

/* udc_endpoint_halted - is endpoint halted
 */
int udc_endpoint_halted (unsigned int ep)
{
	return 0;
}

/* udc_set_address - set the USB address for this device
 */
void udc_set_address (unsigned char address)
{
}

/* udc_serial_init - set a serial number if available
 */
int udc_serial_init (void)
{
	return -EINVAL;
}

/* udc_max_endpoints - max physical endpoints 
 */
int udc_max_endpoints (void)
{
	return UDC_MAX_ENDPOINTS;
}

/* udc_set_ep - setup endpoint 
 */
void udc_setup_ep (unsigned int ep, struct usb_endpoint_instance *endpoint)
{
        pxa_enable_ep_interrupt(endpoint->bEndpointAddress);
}

/* udc_disable_ep - disable endpoint
 */
void udc_disable_ep (unsigned int ep)
{
        pxa_disable_ep_interrupt(ep);
}

/* udc_attached - is the USB cable connected
 */
int udc_attached ()
{
#ifdef CONFIG_ARCH_LUBBOCK
        TRACE_MSG32 ("ATTACHED: %x", LUB_MISC_RD);
        TRACE_MSG32 ("ATTACHED: %x", ((LUB_MISC_RD & (1 << 9)) == 0));
        return ((LUB_MISC_RD & (1 << 9)) == 0);
#else
	return 1;
#endif          
}

int udc_connected_status;

/* udc_connected - is the USB pullup enabled
 */
int udc_connected ()
{
        return udc_connected_status;
}

/* udc_connect - enable pullup resistor
 */
void udc_connect (void)
{
#ifdef CONFIG_SABINAL_DISCOVERY
        /* Set GPIO pin function to I/O */
        GPAFR1_U &= ~(0x3 << ((USBD_CONNECT_GPIO & 0xF) << 1));
        /* Set pin direction to output */
        GPDR(1) |= GPIO_GPIO(USBD_CONNECT_GPIO);
#if defined(USBD_CONNECT_HIGH)
	/* Set GPIO pin high to connect */
        printk(KERN_INFO"udc_connect: HIGH\n");
	GPSR(1) |= GPIO_GPIO(USBD_CONNECT_GPIO);
#else
	/* Set GPIO pin low to connect */
        printk(KERN_INFO"udc_connect: LOW\n");
	GPCR(1) |= GPIO_GPIO(USBD_CONNECT_GPIO);
#endif
#elif CONFIG_ARCH_H3900
        GPDR0 |= GPIO_H3900_USBP_PULLUP;
        GPSR0 |= GPIO_H3900_USBP_PULLUP;
#else
#endif
        udc_connected_status = 1;
}

/* udc_disconnect - disable pullup resistor
 */
void udc_disconnect (void)
{
#ifdef CONFIG_SABINAL_DISCOVERY
        /* Set GPIO pin function to I/O */
        GPAFR1_U &= ~(0x3 << ((USBD_CONNECT_GPIO & 0xF) << 1));
        /* Set pin direction to output */
        GPDR(1) |= GPIO_GPIO(USBD_CONNECT_GPIO);
#if defined(USBD_CONNECT_HIGH)
	/* Set GPIO pin low to disconnect */
	GPCR(1) |= GPIO_GPIO(USBD_CONNECT_GPIO);
#else
	/* Set GPIO pin high to disconnect */
	GPSR(1) |= GPIO_GPIO(USBD_CONNECT_GPIO);
#endif
#elif CONFIG_ARCH_H3900
        GPDR0 &= ~GPIO_H3900_USBP_PULLUP;
#else
#endif
        udc_connected_status = 0;
}

/* udc_framenum - get current framenum
 */
int udc_framenum (void)
{
        return (UFNRH & 0x7) << 8 | UFNRL;
}

/* udc_enable_interrupts - enable interrupts
 */
void udc_all_interrupts (void)
{
        int i;
        UDCCR &= ~UDCCR_SRM;                                    // enable suspend/resume interrupt
        UDCCR &= ~UDCCR_REM;                                    // enable reset interrupt
        pxa_enable_ep_interrupt(0);                             // always enable control endpoint
        for (i = 1; i < UDC_MAX_ENDPOINTS; i++) 
                if (usbd_bus->endpoint_array[i].bEndpointAddress) 
                        pxa_enable_ep_interrupt(usbd_bus->endpoint_array[i].bEndpointAddress);
}

/* udc_suspended_interrupts - enable suspended interrupts
 */
void udc_suspended_interrupts (void)
{
        UDCCR |= UDCCR_REM;     // disable reset interrupt
        UDCCR |= UDCCR_SRM;     // disable suspend/resume interrupt
}

/* udc_disable_interrupts - disable interrupts.
 */
void udc_disable_interrupts (void)
{
        UICR0 = UICR1 = 0xff;                   // disable endpoint interrupts
        UFNRH |= UFNHR_SIM;                     // disable sof interrupt
        UDCCR |= UDCCR_REM | UDCCR_SRM;         // disable suspend/resume and reset interrupt
}

/* udc_ep0_packetsize - return ep0 packetsize
 */
int udc_ep0_packetsize (void)
{
	return EP0_PACKETSIZE;
}

/* udc_enable - enable the UDC
 */
void udc_enable (void)
{
	CKEN |= CKEN11_USB;                                     // c.f. 3.6.2 Clock Enable Register
                                                                // c.f. 12.4.11 GPIOn and GPIOx
                                                                // enable cable interrupt
                                                                // c.f. 12.5 UDC Operation - after reset on EP0 interrupt 
                                                                // is enabled.
        UDCCR |= UDCCR_UDE | UDCCR_REM;                         // c.f. 12.6.1.1 UDC Enable
        if (pxa_have_udccfr) {
                TRACE_MSG("Ack Control Mode Enabled");
                UDCCFR = UDCCFR_ACM;                            // enable Ack Control Mode for new PXA
        }
}

/* udc_disable - disable the UDC
 */
void udc_disable (void)
{
        UDCCR &= ~( UDCCR_UDE | UDCCR_REM );                    // c.f. 12.6.1.1 UDC Enable
	CKEN &= ~CKEN11_USB;                                    // c.f. 3.6.2 Clock Enable Register
}

/* udc_startup - allow udc code to do any additional startup
 */
void udc_startup_events (void)
{
	usbd_bus_event_irq (usbd_bus, DEVICE_INIT, 0);
	usbd_bus_event_irq (usbd_bus, DEVICE_CREATE, 0);
	usbd_bus_event_irq (usbd_bus, DEVICE_HUB_CONFIGURED, 0);
	usbd_bus_event_irq (usbd_bus, DEVICE_RESET, 0);
}

/* udc_name - return name of USB Device Controller
 */
char *udc_name (void)
{
	return UDC_NAME;
}

/* udc_request_udc_irq - request UDC interrupt
 */
int udc_request_udc_irq ()
{
	return request_irq (IRQ_USB, pxa_int_hndlr, SA_INTERRUPT, UDC_NAME " USBD Bus Interface", NULL);
}

/* udc_request_cable_irq - request Cable interrupt
 */
int udc_request_cable_irq ()
{
	return -EINVAL;
}

/* udc_request_udc_io - request UDC io region
 */
int udc_request_io ()
{
	return 0;
}

/* udc_release_release_io - release UDC io region
 */
void udc_release_io ()
{
}

/* udc_release_udc_irq - release UDC irq
 */
void udc_release_udc_irq ()
{
	free_irq (IRQ_USB, NULL);
}

/* udc_release_cable_irq - release Cable irq
 */
void udc_release_cable_irq ()
{
}

/* udc_assign_endpoint -
 */
int udc_assign_endpoint( __u8 physicalEndpoint, int used[6], struct usb_endpoint_map *endpoint_map, __u8 bmAttributes,
                __u16 wMaxPacketSize, __u16 transferSize) 
{
        RETURN_EINVAL_IF(used[physicalEndpoint]);
        endpoint_map->bEndpointAddress = epp2l[physicalEndpoint];
        endpoint_map->physicalEndpoint = physicalEndpoint;
        endpoint_map->transferSize = transferSize;
        endpoint_map->bmAttributes = bmAttributes;
        endpoint_map->wMaxPacketSize = wMaxPacketSize;
        used[physicalEndpoint]++;
        return 0;
}

/* udc_request_endpoints -
 */
struct usb_endpoint_map * udc_request_endpoints(int endpointsRequested, struct usb_endpoint_request *requestedEndpoints)
{
        struct usb_endpoint_map *endpoint_map_array = NULL;
        struct usb_device_description *device_description;
        int i;
        int used[16];
        memset(used, 0, sizeof(used));
	RETURN_NULL_IF(!(endpoint_map_array = ckmalloc(sizeof(struct usb_endpoint_map) * endpointsRequested, GFP_KERNEL)));
        for (i = 0; i < endpointsRequested; i++) {
                struct usb_endpoint_map *endpoint_map = endpoint_map_array + i;
                __u8 bmAttributes = requestedEndpoints[i].bmAttributes;
                __u16 transferSize = requestedEndpoints[i].fs_requestedTransferSize;
                endpoint_map->bmAttributes = bmAttributes;
                switch(bmAttributes) {
                case USB_DIR_IN | USB_ENDPOINT_BULK:
                        CONTINUE_IF(!udc_assign_endpoint( 1, used, endpoint_map, bmAttributes, 0x40, transferSize));
                        CONTINUE_IF(!udc_assign_endpoint( 6, used, endpoint_map, bmAttributes, 0x40, transferSize));
                        CONTINUE_IF(!udc_assign_endpoint(11, used, endpoint_map, bmAttributes, 0x40, transferSize));
                        break;
                case USB_DIR_OUT | USB_ENDPOINT_BULK:
                        CONTINUE_IF(!udc_assign_endpoint( 2, used, endpoint_map, bmAttributes, 0x40, transferSize));
                        CONTINUE_IF(!udc_assign_endpoint( 7, used, endpoint_map, bmAttributes, 0x40, transferSize));
                        CONTINUE_IF(!udc_assign_endpoint(12, used, endpoint_map, bmAttributes, 0x40, transferSize));
                        break;
                case USB_DIR_IN | USB_ENDPOINT_ISOCHRONOUS:
                        CONTINUE_IF(!udc_assign_endpoint( 3, used, endpoint_map, bmAttributes, transferSize, transferSize));
                        CONTINUE_IF(!udc_assign_endpoint( 8, used, endpoint_map, bmAttributes, transferSize, transferSize));
                        CONTINUE_IF(!udc_assign_endpoint(13, used, endpoint_map, bmAttributes, transferSize, transferSize));
                        break;
                case USB_DIR_OUT | USB_ENDPOINT_ISOCHRONOUS:
                        CONTINUE_IF(!udc_assign_endpoint( 4, used, endpoint_map, bmAttributes, transferSize, transferSize));
                        CONTINUE_IF(!udc_assign_endpoint( 9, used, endpoint_map, bmAttributes, transferSize, transferSize));
                        CONTINUE_IF(!udc_assign_endpoint(14, used, endpoint_map, bmAttributes, transferSize, transferSize));
                        break;
                case USB_DIR_IN | USB_ENDPOINT_INTERRUPT:
                case USB_DIR_IN | USB_ENDPOINT_INTERRUPT | USB_ENDPOINT_OPT:
                        CONTINUE_IF(!udc_assign_endpoint( 5, used, endpoint_map, bmAttributes, 0x8, transferSize));
                        CONTINUE_IF(!udc_assign_endpoint(10, used, endpoint_map, bmAttributes, 0x8, transferSize));
                        CONTINUE_IF(!udc_assign_endpoint(15, used, endpoint_map, bmAttributes, 0x8, transferSize));
                        break;
                case USB_DIR_OUT | USB_ENDPOINT_INTERRUPT:
                case USB_DIR_OUT | USB_ENDPOINT_INTERRUPT | USB_ENDPOINT_OPT:
                        break;
                }
                CONTINUE_IF(bmAttributes & USB_ENDPOINT_OPT);
                THROW(error);
        }
#if 0
        for (i = 0; i < endpointsRequested; i++) {
                struct usb_endpoint_map *endpoint_map = endpoint_map_array + i;
                printk(KERN_INFO"%s: address: %02x physical: %02x request: %02x size: %04x transfer: %04x\n", __FUNCTION__,
                                endpoint_map->bEndpointAddress, endpoint_map->physicalEndpoint,
                                endpoint_map->bmAttributes, endpoint_map->wMaxPacketSize, endpoint_map->transferSize
                                );
        }
#endif
        CATCH(error) {
                printk(KERN_INFO"%s: FAILED\n", __FUNCTION__);
                if (endpoint_map_array) 
                        lkfree(endpoint_map_array);
                return NULL;
        }
        return endpoint_map_array;
}

int udc_set_endpoints(int endpointsRequested, struct usb_endpoint_map *endpoint_map_array)
{
        return 0;
}

