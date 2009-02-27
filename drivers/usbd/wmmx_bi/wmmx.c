/*
 * usbd/wmmx_bi/wmmx.c -- Bulverde USB Device Controller driver.
 *
 *      Copyright (c) 2004 Belcarra
 *
 * Adapted from earlier work:
 *      Copyright (c) 2002, 2003 Belcarra
 *      Copyright (c) 2000, 2001, 2002 Lineo
 *      Copyright (c) 2003 MontaVista Software, Inc.
 *
 * 2003/12/18
 * sl@belcarra.com, tbr@belcarra.com
 *      - softconnect and cable interrupt support
 *      - implemented work around for hardware endpoint limitations vis a vis set interface
 *      - generally cleanup and reduce I/O operations 
 *
 * 09/04/2003
 * Stanley Cai (stanley.cai@intel.com) ported the driver to Bulverde Mainstone.
 *
 * Copyright (C) 2003 - Motorola
 *
 * 2003-Dec-06 - Modified for EZXBASE By Zhao Liang <w20146@motorola.com>
 *
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
 * Notes.
 *
 * 0. This code was developed on the Intel Mainstone with Bulverde processor
 *
 * 1. Slow I/O
 *    Access to the UDC is slow, on the order of 1us to read/write I/O registers.  To reduce ISR
 *    overhead it is necessary to reduce accesses to the minimum. Note that TRACE_ macros read
 *    the OSCR register which may have the same problem. So keep TRACE_ calls to a minimum as
 *    well.
 *
 * 2. UDC Configuration
 *    The current Bulverde implementation of the UDC is designed to operate outside of the USB
 *    2.0 specification WRT to configurations and interfaces. See README-WMMX-PROBLEMS for
 *    details. 
 *
 *    We must determine the union of all possible logical endpoints and the union of all
 *    possible configurations (configuration, interface and alternate) and define a physical
 *    endpoint for each possible logical endpoint in each possible configuration.
 *
 *    We keep track of this in the wwmx_index structure that points at the current physical
 *    endpoints as mapped by the logical ones. This allows us to use the correct physical
 *    endpoints. We change the global pointer to this at each change of configuration.
 *
 * 3. Loss of data on config change
 *    Another side-effect of the previous problem is that all pending data in both directions
 *    for all endpoints (except possibly endpoint zero) is lost at the configuration change.
 *
 *    We should (but currently don't) flush all active endpoints and restart from the previous
 *    position all endopints.
 *
 * 4. ISO
 *    The UDC supports ISO endpoints, but this is not currently implemented.  ISO will probably
 *    require DMA support becuase of the I/O speed problem. ISO functions typically require
 *    non-zero Alternate settings so will require all of the above fixes.
 *
 * 5. Endpoint Addresses
 *    bEndpointAddress is allocated in order from 1-N where N is the number of endpoints
 *    requested by the function driver(s). The direction bit is added for IN endpoints. This
 *    simplifies allocation and allows for an simple implementation of the work-around descibed
 *    in para 2. above.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <asm/atomic.h>
#include <linux/vmalloc.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/types.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <asm/hardware.h>
#include <asm/dma.h>

#include <usbd-export.h>
#include <usbd-build.h>
#include <usbd-chap9.h>
#include <usbd-mem.h>
#include <usbd.h>
#include <usbd-func.h>
#include <usbd-bus.h>
#include "trace.h"
#include <usbd-bi.h>
#include "wmmx.h"

#ifdef CONFIG_ARCH_EZX
#include "../misc/ssp_pcap.h"
#include <linux/ezxusbd.h>
#endif

MODULE_DESCRIPTION ("Xscale (WMMX) USB Device Bus Interface");
MODULE_AUTHOR ("stanley.cai@intel.com, sl@belcarra.com, tbr@belcarra.com");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,17)
MODULE_LICENSE("GPL");
#endif
USBD_MODULE_INFO ("wmmx_bi 2.0-beta");
const char *usbd_bi_module_info(void)
{ return __usbd_module_info; }

static int udc_suspended;
unsigned int udc_interrupts;
int wmmx_alternates;                            // total number of config/interface/alternates
struct wmmx_index *wmmx_indexes;                // array of configuration information
struct wmmx_index *wmmx_index;                  // pointer to current configuration

extern void bulverde_udc_regs(void);  

/* wmmx_epn - return physical endpoint offset for endpoint
 */
int __inline__ wmmx_epn(struct usb_endpoint_instance *endpoint)
{
        int epn = (endpoint->bEndpointAddress & 0xf);
        return epn ? wmmx_index->offset + epn : 0;
}

/* wmmx_intmask - get interrupt mask for physical endpoint
 */
static __inline__ u32 wmmx_intmask(int phys_ep)
{
        return (phys_ep < 16) ?  (UDC_INT_FIFOERROR | UDC_INT_PACKETCMP) << (phys_ep << 1) :
                (UDC_INT_FIFOERROR | UDC_INT_PACKETCMP) << ((phys_ep - 16) << 1);
}

/* wmmx_enable_ep_interrupt - enable interrupt for physical endpoint
 */
static __inline__ void wmmx_enable_ep_interrupt(int phys_ep)
{
        ((phys_ep < 16) ? UDCICR0 : UDCICR1) |= wmmx_intmask(phys_ep);
}

/* wmmx_disable_ep_interrupt - enable interrupt for physical endpoint
 */
static __inline__ void wmmx_disable_ep_interrupt(int phys_ep)
{
        ((phys_ep < 16) ? UDCICR0 : UDCICR1) &= ~wmmx_intmask(phys_ep);
}
/* ********************************************************************************************** */
/* wmmx_out_bulk - receive BULK data
 */
static void wmmx_out_bulk(u32 ep, struct usb_endpoint_instance *endpoint)
{
        struct urb *rcv_urb = bi_rcv_next_irq(endpoint);                // get next available rcv_urb 
        volatile u32 * reg = PUDCDN(ep);
        u32 udccsn;
        if (unlikely(!rcv_urb)) {                                       // get next availabel rcv_urb 
                wmmx_disable_ep_interrupt(ep);                          // disable interrupts if there is no urb available;
                return;
        }
        while ((udccsn = UDCCSN(ep)) & UDCCSR_PC) {                             // loop to empty the two FIFOs
                u32 *dp = (u32*)(rcv_urb->buffer + rcv_urb->actual_length);
                int len = MIN(UDCBCN(ep), endpoint->wMaxPacketSize);
                int count;
                if (unlikely(!(udccsn & UDCCSR_PC))) {             // check for errors
                        UDCCSN(ep) = (udccsn & UDCCSR_WR_MASK) | UDCCSR_PC;
                        return;
                }
                for (count = len; count > 0; count -= 4, *dp++ = *reg);         // copy data 
                UDCCSN(ep) = UDCCSR_PC | (udccsn & UDCCSR_WR_MASK);             // clear PC and interrupt 
                if (unlikely(!(rcv_urb = bi_rcv_complete_irq (endpoint, len, 0)))) { // process received data in urb
                        wmmx_disable_ep_interrupt(ep);          // disable interrupts and exit if we end up without rcv_urb
                        return;
                }
        }
}

/* wmmx_start_in_bulk - start IN endpoint
 */
static void wmmx_start_in_bulk(u32 udccsn, unsigned int ep, struct usb_endpoint_instance *endpoint)
{
        struct urb *tx_urb;
        int size, last, remain;
        volatile u8 *reg = (volatile u8 *)PUDCDN(ep);
        u32 *dp;
        u8 *cp;
        RETURN_IF (!(tx_urb = endpoint->tx_urb));               // check if we have a tx_urb
        if (bi_tx_sendzlp(endpoint)) {
                UDCCSN(ep) = UDCCSR_SP;                         // send ZLP
                return;
        }
        size = last = MIN(tx_urb->actual_length - (endpoint->sent + endpoint->last), endpoint->wMaxPacketSize);
        dp = (u32*)(tx_urb->buffer + endpoint->sent + endpoint->last);
        for (remain = (last & 0x03), size &= ~0x03; size > 0; size -= 4, UDCDN(ep) = *dp++);    // 4 byte writes
        for (cp = (u8 *)dp; remain--; *reg = *cp++);                                            // remainder as 1 byte writes
        if ((last < endpoint->wMaxPacketSize) || ((endpoint->tx_urb->actual_length - endpoint->sent) == last)) // short packet?
                if (last != endpoint->wMaxPacketSize)
                        UDCCSN(ep) = (udccsn & UDCCSR_WR_MASK) | UDCCSR_SP;
        endpoint->last += last;
} 

/* wmmx_in_bulk - process IN endpoint interrupt
 */
static void wmmx_in_bulk(u32 ep, struct usb_endpoint_instance *endpoint)
{
        u32 udccsn = UDCCSN(ep);
        if (udccsn & UDCCSR_PC) {                                               // if TPC update tx urb and clear TPC 
                UDCCSN(ep) = (udccsn & UDCCSR_WR_MASK) | UDCCSR_PC;
                bi_tx_complete_irq(endpoint, 0);
        }
        if (udccsn & UDCCSR_FS)
                wmmx_start_in_bulk(udccsn, ep, endpoint);
        if (udccsn & UDCCSR_TRN)                                                // clear underrun, not much we can do about it 
                UDCCSN(ep) = (udccsn & UDCCSR_WR_MASK) | UDCCSR_TRN;
}
/* ********************************************************************************************** */
/* wmmx_out_ep0 - called to recv endpoint zero data for a CONTROL WRITE
 */
void wmmx_out_ep0(struct usb_endpoint_instance *endpoint)
{
        struct urb *rcv_urb = bi_rcv_next_irq(endpoint);
        volatile u32 udccsr0 = UDCCSR0;
        TRACE_MSG32("EP0 RECV: CSF0: %08x", udccsr0);
        if (!rcv_urb) {
                TRACE_MSG("FST");
                UDCCSR0 = UDCCSR0_FST;
                endpoint->state = WAIT_FOR_SETUP;
                return;
        }
        if (!(udccsr0 & UDCCSR0_OPC) && !(udccsr0 & UDCCSR0_IPR))               // check for premature status stage 
                endpoint->state = WAIT_FOR_SETUP; 
	if ((udccsr0 & UDCCSR0_OPC) && !(udccsr0 & UDCCSR0_SA)) {               // receive more data 
		u32* cp = (u32*) (rcv_urb->buffer + rcv_urb->actual_length);
                int len = MIN(UDCBCN(0), endpoint->wMaxPacketSize);
		int size = len;
		for (; size > 0; size-=4, *cp++ = UDCDN(0));
		UDCCSR0 |= UDCCSR0_OPC | UDCCSR0_IPR;   // set OPC and IPR to allow to enter a premature STATUS IN stage 
                if (!bi_rcv_complete_irq (endpoint, len, 0))
                        endpoint->state = WAIT_FOR_SETUP;
                UDCCSR0 |= UDCCSR0_IPR;                                         // Set IPR for end of packet
	}
}

/* wmmx_in_ep0 - called to handle endpoint zero transmit for a CONTROL READ
 */
void wmmx_in_ep0(struct usb_endpoint_instance *endpoint)
{
	int size;
	struct urb *tx_urb = bi_tx_next_irq(endpoint);
        volatile u32 udccsr0 = UDCCSR0;
	if ((udccsr0 & UDCCSR0_OPC) && !(udccsr0 & UDCCSR0_SA)) {               // check for premature status stage 
		UDCCSR0 |= UDCCSR0_FTF | UDCCSR0_OPC;                           // clear tx fifo and opr 
		endpoint->state = WAIT_FOR_SETUP;
                bi_tx_cancelled_irq (endpoint);
		return;
	}
	if (udccsr0 & UDCCSR0_SST) {                                            // clear if stalled
		UDCCSR0 |= UDCCSR0_SST | UDCCSR0_FTF;                           // clear stall and tx fifo 
		endpoint->state = WAIT_FOR_SETUP;
                bi_tx_cancelled_irq (endpoint);
		return;
	}
	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++Jordan :ZLP handler*/
//	if (!tx_urb) {
//              UDCCSR0 |= UDCCSR0_IPR;                                         // Set IPR for end of packet
	if(bi_tx_sendzlp(endpoint)){
		UDCCSR0 |= UDCCSR0_IPR;                                         // Set IPR for ZLP
	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++End*/
                endpoint->state = WAIT_FOR_OUT_STATUS;
                return;
        }
	if(!tx_urb){
		UDCCSR0 |= UDCCSR0_IPR;
		endpoint->state = WAIT_FOR_OUT_STATUS;
		return;
	}
        endpoint->last = size = MIN(tx_urb->actual_length - endpoint->sent, endpoint->wMaxPacketSize);
        if ((size > 0)) {
                volatile u8 *reg = (volatile u8 *)PUDCDN(0);
                u32* dp = (u32*)(tx_urb->buffer + endpoint->sent);
                int remain = size & 0x03;
                u8 *cp;
                TRACE_MSG("SENDING");
                for (size &= ~0x3; size; size -= 4, UDCDN(0) = *dp++);
                for (cp = (u8 *)dp; remain--; *reg = *cp++);
		/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++Jordan handle end zlp case*/
		if((endpoint->last == endpoint->wMaxPacketSize) &&     
		    ((endpoint->last + endpoint->sent) == endpoint->tx_urb->actual_length)) 
		{
			endpoint->state = DATA_STATE_NEED_ZLP;	
			return;
		}
		/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++end */
                RETURN_IF (endpoint->last == endpoint->wMaxPacketSize);
		/*+++++++++++++++++++++++++++++++++++++++++++++ Jordan*/
        	UDCCSR0 |= UDCCSR0_IPR;                                         // Set IPR for end of packet
        }
		/*+++++++++++++++++++++++++++++++++++++++++++++ end*/
}

/* wmmx_ep0_setup - called when endpoint zero is waiting for a setup packet
 */
void wmmx_ep0_setup(u32 udccsr0, struct usb_endpoint_instance *endpoint)
{
        int count;
        u32 *dp;
        int len = UDCBCN(0);
        struct usb_device_request request;
        bi_tx_cancelled_irq(endpoint);
        bi_rcv_cancelled_irq(endpoint);
        TRACE_MSG32("EP0 SETUP: UDCCSR0: %04x", udccsr0);

        RETURN_IF((udccsr0 & (UDCCSR0_SA | UDCCSR0_OPC | UDCCSR0_RNE)) != (UDCCSR0_SA | UDCCSR0_OPC | UDCCSR0_RNE));
        memset(&request, 0, sizeof(request));
        for (dp = (u32 *)&request, count = MIN(8, len); count > 0; count -= 4, len -= 4, *dp++ = UDCDN(0));
        for (; len > 0; len -= 4, count = UDCDN(0));                            // flushing ep0 FIFO
        if (bi_recv_setup_irq(&request)) { 
                UDCCSR0 = UDCCSR0_FST;
                endpoint->state = WAIT_FOR_SETUP;
                TRACE_MSG("    --> bad SETUP FST");
                return;
        }
        if ((request.bmRequestType & USB_REQ_DIRECTION_MASK) == USB_REQ_HOST2DEVICE) {
                UDCCSR0 |= UDCCSR0_IPR;
                if (request.wLength) 
                        endpoint->state = DATA_STATE_RECV;
                return;
        } 
        if (!le16_to_cpu(request.wLength)) {                                    // Control Read 
                TRACE_MSG("EP0 STALL");
                udc_stall_ep(0);
                return;
        }
        if (!bi_tx_next_irq(endpoint)) {                                        // no tx_urb, disable ep0 
                endpoint->state = DATA_STATE_PENDING_XMIT;
                UDCCSR0 = UDCCSR0_SA | UDCCSR0_OPC;                             // Clear SA and OPR bits ??
                return;
        }
        endpoint->state = DATA_STATE_XMIT;
        wmmx_in_ep0(endpoint);
        UDCCSR0 = UDCCSR0_SA | UDCCSR0_OPC;                                     // Clear SA and OPR bits 
}

/* wmmx_ep0_int - main endpoint zero interrupt handler
 */
static void wmmx_ep0_int(u32 ep, struct usb_endpoint_instance *endpoint)
{
        int j = 0;
        u32 udccsr0 = UDCCSR0;
        TRACE_MSG32("EP0 INT CSR0: %08x", udccsr0);
        if ((endpoint->state != DATA_STATE_PENDING_XMIT) && (udccsr0 & UDCCSR0_SST)) {
                UDCCSR0 |= UDCCSR0_SST;
                return;
        }
        if ((DATA_STATE_XMIT == endpoint->state) && !bi_tx_complete_irq (endpoint, 0)) // update tx_urb 
                endpoint->state = WAIT_FOR_OUT_STATUS;
        if ((DATA_STATE_PENDING_XMIT != endpoint->state) && (WAIT_FOR_SETUP != endpoint->state) && (udccsr0 & UDCCSR0_SA)) 
                endpoint->state = WAIT_FOR_SETUP;
        switch (endpoint->state) {
	/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++Jordan end zlp case*/
	case DATA_STATE_NEED_ZLP:  
		UDCCSR0 |= UDCCSR0_IPR;
		endpoint->state = WAIT_FOR_OUT_STATUS;
		break;
        /*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++end*/
        case WAIT_FOR_OUT_STATUS:
                TRACE_MSG("WAIT_FOR_OUT_STATUS");
                if ((udccsr0 & (UDCCSR0_OPC | UDCCSR0_SA)) == UDCCSR0_OPC) 
                        UDCCSR0 |= UDCCSR0_OPC;
                endpoint->state = WAIT_FOR_SETUP;
                /* FALL THROUGH */
        case WAIT_FOR_SETUP:
                TRACE_MSG("WAIT_FOR_SETUP");
                do {
                        wmmx_ep0_setup(UDCCSR0, endpoint);
                        RETURN_IF (DATA_STATE_PENDING_XMIT == endpoint->state);
                        if (udccsr0 & UDCCSR0_SST) {
                                UDCCSR0 |= UDCCSR0_SST | UDCCSR0_OPC | UDCCSR0_SA;
                                endpoint->state = WAIT_FOR_SETUP;
                                return;
                        }
                        CONTINUE_IF (j++ <= 2);                                 // continue if first or second time
                        UDCCSR0 |= (UDCCSR0_OPC | UDCCSR0_SA);                  // set OPC | SA
                        BREAK_IF ((UDCCSR0 & (UDCCSR0_OPC | UDCCSR0_SA | UDCCSR0_RNE)) != (UDCCSR0_OPC | UDCCSR0_SA));
                } while (UDCCSR0 & (UDCCSR0_OPC | UDCCSR0_RNE));
                break;
        case DATA_STATE_XMIT:
        case DATA_STATE_PENDING_XMIT:
                TRACE_MSG("DATA_STATE_XMIT");
                wmmx_in_ep0(endpoint);
                break;
        case DATA_STATE_RECV:
                TRACE_MSG("DATA_STATE_RECV");
                wmmx_out_ep0(endpoint);
                break;
        }
}
/* ********************************************************************************************** */
/* wmmx_notimp - for unimplemented endpoint types (e.g. iso)
 */
static void wmmx_notimp(u32 ep, struct usb_endpoint_instance *endpoint)
{
        return;
}

/* int_hndlrs - table of interrupt function handlers for each endpoint type
 */
typedef void (*int_hndlr_t) (u32, struct usb_endpoint_instance*);
int_hndlr_t int_hndlrs[8] = {
        wmmx_ep0_int, wmmx_notimp, wmmx_out_bulk, wmmx_out_bulk,
        wmmx_ep0_int, wmmx_notimp, wmmx_in_bulk, wmmx_in_bulk,
};

/* wmmx_int_hndlr - Bulverde UDC Interrupt Handler
 * Iterate until all endpoint interrupt sources in UDCISR0 and * UDCISR1 are gone.  Note that we
 * are guaranteed to see UDCISR1 and it is saved for later use.  It should be possible to use
 * ffs (find first set) for this but we have to look at two bits for each endpoint.
 */
#ifdef CONFIG_ARCH_EZX
extern int usbd_enum_flag;
extern struct timer_list reenum_timer;
#define RE_ENUM_TIMEOUT		(500*HZ/1000)	// 500ms for pull up/down restistor - from LWQ
#endif

static void wmmx_int_hndlr (int irq, void *dev_id, struct pt_regs *regs)
{
        u32 udcisr, udcisr1;
	udc_interrupts++;
        for (;;) {
                int ep, epmax;
                struct usb_endpoint_instance *endpoint;
                if ((udcisr = UDCISR0)) {
                        ep = 0;                                                 // start at ep 0
                        epmax = 16;                                             // end at   ep 15
                }
                else if ((udcisr = ((udcisr1 = UDCISR1) & 0xffff))) {
                        ep = 16;                                                // start at ep 16
                        epmax = 24;                                             // end at   ep 23
                }
                else 
                        break;
                for (endpoint = usbd_bus->endpoint_array + ep; udcisr != 0 && ep < epmax ; udcisr >>= 2, ep++, endpoint++) {
                        CONTINUE_IF(likely(!(udcisr & (UDC_INT_PACKETCMP | UDC_INT_FIFOERROR))));
                        ((ep < 16) ? UDCISR0 : UDCISR1) = wmmx_intmask(ep);     // reset interrupt status
                        int_hndlrs[ ((endpoint->bmAttributes & USB_ENDPOINT_DIR_MASK) >> 5) | (endpoint->bmAttributes & 0x3)]
                                (ep, endpoint);
                }
        }
        RETURN_IF (likely(!udcisr1));                                           // Check for less common interrupts
	if (udcisr1 & UDCISR1_IERS) {                                           // Reset
                TRACE_MSG("RESET");
		udc_suspended = 0;
#ifdef CONFIG_ARCH_EZX
		usbd_enum_flag = 0;             // Jul 6, 2004 -- w20146
		usbd_bus->suspended_state = STATE_INIT;
		usbd_bus->device_state = STATE_INIT;
#endif
		usbd_bus_event_irq (usbd_bus, DEVICE_RESET, 0);
		usbd_bus_event_irq (usbd_bus, DEVICE_ADDRESS_ASSIGNED, 0);	
		UDCISR1 = UDCISR1_IERS;
		//printk(KERN_INFO"%s: RESET clear usbd_enum_flag, suspended %d\n", __FUNCTION__, usbd_bus->suspended_state);
	}
	if (udcisr1 & UDCISR1_IERU) {                                           // Resume
                TRACE_MSG("RESUME");
		//printk(KERN_INFO"%s: RESUME device %d bus %d suspended %d\n", __FUNCTION__, 
		//		usbd_bus->device_state, usbd_bus->bus_state, usbd_bus->suspended_state);
		if (udc_suspended) {
			udc_suspended = 0;
			usbd_bus_event_irq (usbd_bus, DEVICE_BUS_ACTIVITY, 0);
		}
		UDCISR1 = UDCISR1_IERU;
	}
	if (udcisr1 & UDCISR1_IESU) {                                           // Suspend 
                TRACE_MSG("SUSPEND");

#ifdef CONFIG_ARCH_EZXBASE
		if(1)
		{
			int udccontrol_context_safe(int control_flag);

			//printk(KERN_INFO"%s: SUSPEND device %d bus %d suspended %d\n", __FUNCTION__, 
			//		usbd_bus->device_state, usbd_bus->bus_state, usbd_bus->suspended_state);
			if(usbd_enum_flag && ((usbd_bus->suspended_state == STATE_ADDRESSED) || (usbd_bus->suspended_state == STATE_INIT)) && (usbd_bus->device_state == STATE_ADDRESSED))
			{
				struct usb_configuration_descriptor *cfg_desc;

				usbd_enum_flag = 0;
				//printk(KERN_INFO"\n!!!!!!!\n%s: enumeration failure! Disable charging!\n", __FUNCTION__);
				cfg_desc = (&usbd_bus->function_instance->function_driver->configuration_instance_array[0])->configuration_descriptor;
				//printk(KERN_INFO"%s: bMaxPower %d\n!!!!!!!\n\n", __FUNCTION__, cfg_desc->bMaxPower);
				if(cfg_desc->bMaxPower <= POWER_CURRENT_100MA_SETTING)
				{
					//printk(KERN_INFO"\n\n%s: SUSPEND! Disable charging!\n\n", __FUNCTION__);
					queue_motusbd_event(USB_CURRENT_SUSPEND);
				}
				else
				{
					//printk(KERN_INFO"\n!!!%s: start re-enumeratio mechanism\n", __FUNCTION__);
					udccontrol_context_safe(0);
					mod_timer(&reenum_timer, jiffies + RE_ENUM_TIMEOUT);
				}
			}
			else if(usbd_bus->device_state == DEVICE_CONFIGURED)
			{
				//printk(KERN_INFO"\n\n%s: SUSPEND! \n\n\n", __FUNCTION__);
				queue_motusbd_event(USB_CURRENT_SUSPEND);
			}
		}
#endif	// CONFIG_ARCH_EZXBASE
		
		if (!udc_suspended) {
			udc_suspended = 1;
			usbd_bus_event_irq (usbd_bus, DEVICE_BUS_INACTIVE, 0);
		}
		UDCISR1 = UDCISR1_IESU;
	}
	if (udcisr1 & UDCISR1_IECC) {                                           // Configuration Change 
                struct usb_device_request request;
                u8 configuration = (UDCCR & UDCCR_ACN) >> UDCCR_ACN_S;
                u8 interface = (UDCCR & UDCCR_AIN) >> UDCCR_AIN_S;
                u8 alternate = (UDCCR & UDCCR_AAISN) >> UDCCR_AAISN_S;
                u8 index;
                TRACE_MSG8("CONFIG CHANGE CFG: %x intf: %x alt: %x", usbd_bus->ConfigurationValue, interface, alternate, 0); 
                wmmx_index = wmmx_indexes;                                      // search for correct wmmx_index
                for (index = 0; alternate < wmmx_alternates; alternate++) {
                        CONTINUE_IF(wmmx_index->configuration != configuration);
                        CONTINUE_IF(wmmx_index->interface != interface);
                        CONTINUE_IF(wmmx_index->alternate != alternate);
                        TRACE_MSG("FOUND WMMX_INDEX");
                }
                /* XXX TODO FIXME we need to interrogate all transmit (OUT) endpoints and
                 * be ready to restart them in their new physical endpoints after banging on SMAC
                 */
		UDCCR |= UDCCR_SMAC;                                            // change to the new active configuration

                /* We need to generate a fake SETUP packet and call usbd_recv_setup() to process
                 * it. This will allow all of the required processing to take place just as
                 * though we actually received a real setup packet.  If we are not configured or
                 * the configuration seen in UDCCR is not what is seen in
                 * usbd_bus->configuration then do SET CONFIG otherwise do SET INTERFACE
                 */
                memset(&request, 0, sizeof(request));
                if ((usbd_bus->device_state != STATE_CONFIGURED) || (usbd_bus->ConfigurationValue != configuration)) {
                        TRACE_MSG32("SET CONFIGURATION: %x", cpu_to_le16(configuration));
                        request.bRequest = USB_REQ_SET_CONFIGURATION;
                        request.wValue = cpu_to_le16(configuration);
                }
                else {
                        TRACE_MSG16("SET INTERFACE: %x %x", cpu_to_le16(interface), cpu_to_le16(alternate));
                        request.bRequest = USB_REQ_SET_INTERFACE;
                        request.wIndex = cpu_to_le16(interface);
                        request.wValue = cpu_to_le16(alternate);
                }
                bi_recv_setup_irq(&request);
		UDCISR1 = UDCISR1_IECC;
	}
}

/* wmmx_cable_int_hndlr - handle a cable status change interrupt
 */
static void wmmx_cable_int_hndlr (int irq, void *dev_id, struct pt_regs *regs)
{
        TRACE_MSG32("CABLE INT attached: %d", udc_attached());
        udc_cable_event_irq();
}
/* ********************************************************************************************** */
/* udc_start_endpoint_in - start transmit
 */                     
void udc_start_endpoint_in(struct usb_endpoint_instance *endpoint)
{                       
        u32 udccsn;
        int epn = wmmx_epn(endpoint);
        switch(endpoint->bmAttributes & USB_ENDPOINT_MASK) {
        case USB_ENDPOINT_CONTROL:
                if ((DATA_STATE_PENDING_XMIT == endpoint->state)) {
                        wmmx_enable_ep_interrupt(0);
                        endpoint->state = DATA_STATE_XMIT;
                        wmmx_in_ep0(endpoint);
                }
                break;
        case USB_ENDPOINT_BULK:
        case USB_ENDPOINT_INTERRUPT:
                wmmx_enable_ep_interrupt(epn);
                if ((udccsn = UDCCSN(epn)) & UDCCSR_FS) 
                        wmmx_start_in_bulk(udccsn, epn, endpoint);
                break;
        }
}

/* udc_start_endpoint_out - start receive
 */
void udc_start_endpoint_out(struct usb_endpoint_instance *endpoint)
{
        int epn = wmmx_epn(endpoint);
        wmmx_enable_ep_interrupt(epn);
        switch(endpoint->bmAttributes & USB_ENDPOINT_MASK) {
        case USB_ENDPOINT_BULK:
        case USB_ENDPOINT_INTERRUPT:
                wmmx_out_bulk(epn, usbd_bus->endpoint_array + epn);
                break;
        }
}

#ifdef CONFIG_ARCH_EZX
void udc_stop_endpoint_out(struct usb_endpoint_instance *endpoint)
{
	int epn = wmmx_epn(endpoint);
	wmmx_disable_ep_interrupt(epn);
}
#endif

/* udc_cancel_in_irq - cancel IN urb
 */
void udc_cancel_in_irq(struct urb *urb)
{
        int epn = wmmx_epn(urb->endpoint);
        wmmx_disable_ep_interrupt(epn);
}

/* udc_cancel_out_irq - cancel OUT urb
 */
void udc_cancel_out_irq(struct urb *urb)
{
        int epn = wmmx_epn(urb->endpoint);
        wmmx_disable_ep_interrupt(epn);
}

/* udc_init - initialize the WMMX UDC
 */
int udc_init (void)
{
	udc_disable_interrupts ();
	return 0;
}

/* udc_stall_ep - stall endpoint
 */
void udc_stall_ep (u32 phys_ep)
{
        UDCCSN(phys_ep) |= UDCCSR_FST;
}

/* udc_reset_ep - reset endpoint
 */
void udc_reset_ep (unsigned int phys_ep)
{
        UDCCSN(phys_ep) |= ((UDCCSN(phys_ep) & UDCCSR_DME) | UDCCSR_FEF);
}

/* udc_endpoint_halted - is endpoint halted
 */
int udc_endpoint_halted (unsigned int phys_ep)
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
int __init udc_serial_init (void)
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
void udc_setup_ep ( unsigned int ep, struct usb_endpoint_instance *endpoint)
{
        if ((ep == 0) || (endpoint->bEndpointAddress & 0x80)) 
                wmmx_enable_ep_interrupt(ep);
        else if (endpoint->bEndpointAddress) { 
                RETURN_IF(bi_rcv_next_irq(endpoint));
                wmmx_enable_ep_interrupt(ep);
        }
}

/* udc_disable_ep - disable endpoint
 */
void udc_disable_ep (unsigned int ep)
{
        wmmx_disable_ep_interrupt(0);
}

/* udc_attached - is the USB cable connected
 * Return non-zeron if cable is connected.
 *
 * udc_connected - is the USB pullup enabled
 * Return non-zeron if cable is connected.
 *
 * udc_connect - enable pullup resistor
 * Turn on the USB connection by enabling the pullup resistor.
 *
 * udc_disconnect - disable pullup resistor
 * Turn off the USB connection by disabling the pullup resistor.
 *
 */
#ifdef CONFIG_ARCH_MAINSTONE
int udc_attached (void)
{
        u32 mscrd = MST_MSCRD;                                                  // XXX C.f. Section 12.4.4
        TRACE_MSG16("ATTACHED MSCRD: %04x %04x", mscrd, (mscrd & MST_MSCRD_USB_CBL) ? 1 : 0);
        return (mscrd & MST_MSCRD_USB_CBL)? 1 : 0;
}

int udc_connected_status;

int udc_connected(void)
{
        return udc_connected_status;
}

void udc_connect (void)
{
        MST_MSCWR2 &= ~MST_MSCWR2_nUSBC_SC;
        TRACE_MSG32("CONNECT MSCWR2: %08x", MST_MSCWR2);
        udc_connected_status = 1;
}

void udc_disconnect (void)
{
        MST_MSCWR2 |= MST_MSCWR2_nUSBC_SC;
        TRACE_MSG32("DISCONNECT MSCWR2: %08x", MST_MSCWR2);
        udc_connected_status = 0;
}

#elif CONFIG_ARCH_EZX		// w20146 - port BMC code base to E680

extern void pulldown_usb(void);
extern void pullup_usb(void);
extern int powermode_flag;
extern void restore_powermode(void);

void udc_clock_enable(void);
void udc_clock_disable(void);

int udc_connected_status;
extern int usbcable_status;		// from PCAP INT

void clear_device_state(void)
{
	if(usbd_bus)
	{
		//printk(KERN_INFO"%s: \n", __FUNCTION__);
		usbd_bus->suspended_state = STATE_INIT;
		usbd_bus->device_state = STATE_INIT;
		restore_powermode();
		//powermode_flag = 0;
	}
	return ;
}

int udc_attached (void)
{
#if 0
	// From PCAP(Lin weiqiang):
	// SSP_PCAP_get_bit_from_PCAP(SSP_PCAP_ADJ_BIT_PSTAT_USBDET_1V) == SSP_PCAP_BIT_ONE  is unplug USB cable.
	// SSP_PCAP_get_bit_from_PCAP(SSP_PCAP_ADJ_BIT_PSTAT_USBDET_4V) == SSP_PCAP_BIT_ONE  is plug in a USB Type cable. 
	int ret;
	ret = (SSP_PCAP_get_bit_from_PCAP(SSP_PCAP_ADJ_BIT_PSTAT_USBDET_1V) == SSP_PCAP_BIT_ONE);
	printk(KERN_INFO"%s: from PCAP %d\n", __FUNCTION__, ret);
	return !ret;
#else
	return usbcable_status;
#endif
}

int udc_connected (void)
{
#if 1
	//printk(KERN_INFO"%s: status %d\n", __FUNCTION__, udc_connected_status);
	return udc_connected_status;
#else
	return 1;
#endif
}

void udc_connect (void)
{
	if(udc_connected_status == 0)
	{
		//printk(KERN_INFO"%s:\n", __FUNCTION__);
		pullup_usb();
		udc_connected_status = 1;

		// send event upper?
		// It will be better if the function drivers send its own event.
	}
}

void udc_disconnect (void)
{
	if(udc_connected_status == 1)
	{
		//printk(KERN_INFO"%s:\n", __FUNCTION__);
		pulldown_usb();
		udc_connected_status = 0;

		// clear USBD state machine
		clear_device_state();
	}
}
#endif

/* udc_framenum - get current framenum
 */
int udc_framenum (void)
{
        return UDCFNR;
}

/* udc_enable_interrupts - enable interrupts
 */
void udc_all_interrupts (void)
{
	UDCICR1 = (UDCICR1_IESU | UDCICR1_IERU | UDCICR1_IERS | UDCICR1_IECC) & ~UDCICR1_IESOF;
}

/* udc_suspended_interrupts - enable suspended interrupts
 */
void udc_suspended_interrupts (void)
{
	UDCICR1 = (UDCICR1 | UDCICR1_IERS | UDCICR1_IERU | UDCICR1_IECC | UDCICR1_IESU) & ~UDCICR1_IESOF;
}

/* udc_disable_interrupts - disable interrupts.
 */
void udc_disable_interrupts (void)
{
	UDCICR0 = UDCICR1 = 0x00000000;
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
#ifdef CONFIG_ARCH_EZX
	udc_clock_enable();
#else
	CKEN |= CKEN11_USB;                                     // enable UDC clock and enable UDC controller 
#endif
	UDCCR = UDCCR_UDE;
	RETURN_IF ((UDCCR & UDCCR_EMCE) != UDCCR_EMCE);
        TRACE_MSG32("UDC CONFIGURE ERROR CCR: %08x", UDCCR);
        UDCCR = UDCCR_EMCE;
}

//#define PM_USBDCLK
#undef PM_USBDCLK

#ifdef CONFIG_ARCH_EZX     //added by Jordan for the consideration of pm
extern int emucable_status;
void udc_clock_enable(void)
{
#ifdef PM_USBDCLK
	if(emucable_status)
		CKEN |= CKEN11_USB;
#else
	CKEN |= CKEN11_USB;
#endif
}	
#endif

/* udc_disable - disable the UDC
 */
void udc_disable (void)
{
        udc_disable_interrupts();
	UDCCR &= ~(UDCCR_UDE);                                  // disable UDC controller && UDC clock 
#ifdef CONFIG_ARCH_EZX
	udc_clock_disable();
#else
	CKEN &= ~CKEN11_USB;
#endif
}

#ifdef CONFIG_ARCH_EZX     //added by Jordan for the consideration of pm
void udc_clock_disable(void)
{
	CKEN &= ~CKEN11_USB;
}
#endif

/* udc_startup_events - allow udc code to do any additional startup
 */
void udc_startup_events (void)
{
	usbd_bus_event_irq(usbd_bus, DEVICE_INIT, 0);
	usbd_bus_event_irq(usbd_bus, DEVICE_CREATE, 0);
	usbd_bus_event_irq(usbd_bus, DEVICE_HUB_CONFIGURED, 0);
	usbd_bus_event_irq(usbd_bus, DEVICE_RESET, 0);
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
	return request_irq(IRQ_USB, wmmx_int_hndlr, SA_INTERRUPT, UDC_NAME " USBD Bus Interface", NULL);
}

/* udc_request_cable_irq - request Cable interrupt
 */
int udc_request_cable_irq()
{
#ifdef CONFIG_ARCH_MAINSTONE
	return request_irq(MAINSTONE_USBC_IRQ, wmmx_cable_int_hndlr, SA_INTERRUPT, UDC_NAME " USBD Bus Cable", NULL);
#elif CONFIG_ARCH_EZX			// w20146
	return 0;
#endif
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
        RETURN_IF (!wmmx_indexes);
        lkfree(wmmx_indexes);
        wmmx_indexes = NULL;
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
#ifdef CONFIG_ARCH_MAINSTONE
	free_irq (MAINSTONE_USBC_IRQ, NULL);
#elif CONFIG_ARCH_EZX			// w20146
	// none
#endif
}
/* ********************************************************************************************* */
/* udc_assign_endpoint - keep track of used endpoints
 */
int udc_assign_endpoint( u8 bEndpointAddress, struct usb_endpoint_map *endpoint_map, u8 bmAttributes,
                u16 wMaxPacketSize, u16 transferSize, u8 configuration, u8 interface, u8 alternate) 
{
        TRACE_MSG16("ASSIGN %02x %02x", bEndpointAddress, bmAttributes);
        endpoint_map->configuration = configuration;
        endpoint_map->interface = interface;
        endpoint_map->alternate = alternate;
        endpoint_map->bEndpointAddress[0] = bEndpointAddress;
        endpoint_map->physicalEndpoint[0] = bEndpointAddress & 0x7f;
        endpoint_map->transferSize[0] = transferSize;
        endpoint_map->bmAttributes[0] = bmAttributes;
        endpoint_map->wMaxPacketSize[0] = wMaxPacketSize;
        return 0;
}

/* udc_request_endpoints - process endpoint requests
 */
struct usb_endpoint_map * udc_request_endpoints(int endpointsRequested, struct usb_endpoint_request *requestedEndpoints)
{
        struct usb_endpoint_map *endpoint_map_array = NULL;
        struct usb_device_description *device_description;
        int i;
        int last_interface;
        int last_alternate;
        int alternates[23];
        int interface;
        int alternate;

        TRACE_MSG("Request Endpoints");
	RETURN_NULL_IF(!(endpoint_map_array = ckmalloc(sizeof(struct usb_endpoint_map) * endpointsRequested, GFP_KERNEL)));
        wmmx_alternates = 0;
        last_interface = 0;
        last_alternate = 0;

        alternates[0] = 0;
        for (i = 1; i < 23; i++) alternates[i] = -1;                            // Count interfaces and alternates
        for (i = 0; i < endpointsRequested; i++) {
                struct usb_endpoint_map *endpoint_map = endpoint_map_array + i;
                u8 interface = requestedEndpoints[i].interface;
                u8 alternate = requestedEndpoints[i].alternate;
                if (interface > 22) {
			lkfree(endpoint_map_array);
			return NULL;
		}
                alternates[interface] = MAX(alternates[interface], alternate);
                TRACE_MSG8("%02d: altenates: %d interface: %d alternate: %d", i, alternates[interface], interface, alternate);
        }
        for (i = 0; i < 23; i++) 
                if (!i || (alternates[i] > 0)) 
                        wmmx_alternates += alternates[i] + 1;
        
        TRACE_MSG16("alternates: %d requested: %d", wmmx_alternates, endpointsRequested);
        THROW_IF((wmmx_alternates * endpointsRequested) > 23, error);

        if (wmmx_indexes) lkfree(wmmx_indexes);                         // allocate the wmmx_indexes array
        THROW_IF(!(wmmx_indexes = ckmalloc(sizeof(struct wmmx_index) * (wmmx_alternates + 1), GFP_KERNEL)), error);

        for ( wmmx_index = wmmx_indexes, interface = 0; interface < wmmx_alternates; interface++) {
                if (!interface || (alternates[interface] > 0)) {
                        for (i = 0; i <= alternates[interface]; i++) {
                                wmmx_index->configuration = 1;
                                wmmx_index->interface = interface;
                                wmmx_index->alternate = i;
                                wmmx_index++;
                        }
                }
        }
        for ( wmmx_index = wmmx_indexes, alternate = 0; alternate < wmmx_alternates; alternate++) {
                // set offset to multiple of endpoints
                wmmx_index->offset = alternate * endpointsRequested;
                TRACE_MSG8("ALTERNATES cfg: %d int: %d alt: %d off: %d", 
                                wmmx_index->configuration, wmmx_index->interface,
                                wmmx_index->alternate, wmmx_index->offset);
                wmmx_index++;
        }
        wmmx_index->configuration = 0;
        for ( wmmx_index = wmmx_indexes, i = 0; i < endpointsRequested; i++) {
                struct usb_endpoint_map *endpoint_map = endpoint_map_array + i;
                u8 bmAttributes = requestedEndpoints[i].bmAttributes;
                u16 transferSize = requestedEndpoints[i].fs_requestedTransferSize;
                u8 configuration = requestedEndpoints[i].configuration;
                u8 interface = requestedEndpoints[i].interface;
                u8 alternate = requestedEndpoints[i].alternate;
                switch(endpoint_map->bmAttributes[0] = bmAttributes) {
                case USB_DIR_IN | USB_ENDPOINT_BULK:
                        CONTINUE_IF(!udc_assign_endpoint( USB_DIR_IN | (i + 1), endpoint_map, bmAttributes, 
                                                0x40, transferSize, configuration, interface, alternate));
                        break;
                case USB_DIR_OUT | USB_ENDPOINT_BULK:
                        CONTINUE_IF(!udc_assign_endpoint( USB_DIR_OUT | (i + 1), endpoint_map, bmAttributes, 
                                                0x40, transferSize, configuration, interface, alternate));
                        break;
                case USB_DIR_IN | USB_ENDPOINT_ISOCHRONOUS:
                        CONTINUE_IF(!udc_assign_endpoint( USB_DIR_IN | (i + 1), endpoint_map, bmAttributes, 
                                                transferSize, transferSize, configuration, interface, alternate));
                        break;
                case USB_DIR_OUT | USB_ENDPOINT_ISOCHRONOUS:
                        CONTINUE_IF(!udc_assign_endpoint( USB_DIR_OUT | (i + 1), endpoint_map, bmAttributes, 
                                                transferSize, transferSize, configuration, interface, alternate));
                        break;
                case USB_DIR_IN | USB_ENDPOINT_INTERRUPT:
                case USB_DIR_IN | USB_ENDPOINT_INTERRUPT | USB_ENDPOINT_OPT:
                        CONTINUE_IF(!udc_assign_endpoint( USB_DIR_IN | (i + 1), endpoint_map, bmAttributes, 
                                                0x10, transferSize, configuration, interface, alternate));
                        break;
                case USB_DIR_OUT | USB_ENDPOINT_INTERRUPT:
                case USB_DIR_OUT | USB_ENDPOINT_INTERRUPT | USB_ENDPOINT_OPT:
                        break;
                }
                CONTINUE_IF(bmAttributes & USB_ENDPOINT_OPT);
                THROW(error);
        }
#if 1
        for (i = 0; i < endpointsRequested; i++) {
                struct usb_endpoint_map *endpoint_map = endpoint_map_array + i;
                TRACE_MSG8("address: %02x physical: %02x request: %02x size: %04x", 
                                endpoint_map->bEndpointAddress[0], 
                                endpoint_map->physicalEndpoint[0], 
                                endpoint_map->bmAttributes[0], 
                                endpoint_map->wMaxPacketSize[0]);
        }
#endif
        CATCH(error) {
                //printk(KERN_ERR"%s: FAILED\n", __FUNCTION__);
                if (endpoint_map_array)
			lkfree(endpoint_map_array);
                RETURN_NULL_IF (!wmmx_indexes);
                lkfree(wmmx_indexes);
                wmmx_index = NULL;
                return NULL;
        }
        return endpoint_map_array;
}

/* udc_set_endpoints - setup the physical endpoints for the endpoint map
 *
 * C.f. 12.4.2.2, 12.4.2.3, 12.4.3, and 12.5.9.
 *
 * The physical endpoints, 1..24, are completely configurable to any configuration, including
 * endpoint address, direction, configuration, interface, altsetting, size and type.
 *
 * To remediate the WMMX SET INTERFACE problem we need to configure all endpoints in all
 * interfaces.
 *
 * Note that an endpoint address to be used in more than one configuration or in more than once
 * in an interface (with different altsetting), it must be configured into a physical endpoint. 
 *
 * The total number of such allocated endpoints cannot exceed 23.
 */
int udc_set_endpoints(int endpointsRequested, struct usb_endpoint_map *endpoint_map_array)
{
        int alternate;
        for ( wmmx_index = wmmx_indexes, alternate = 0; alternate < wmmx_alternates; alternate++, wmmx_index++) {
                int i;
                int phys_ep = wmmx_index->offset = alternate * endpointsRequested;
                TRACE_MSG8("SET cfg: %d int: %d alt: %d off: %d", 
                                wmmx_index->configuration, wmmx_index->interface, wmmx_index->alternate, wmmx_index->offset);
                for (i = 0; i < endpointsRequested; i++) { 
                        struct usb_endpoint_map *endpoint_map = endpoint_map_array + i;
                        int epreq = endpoint_map->bmAttributes[0];
                        int eptype = epreq & USB_ENDPOINT_MASK;
                        int epdir = epreq & USB_ENDPOINT_DIR_MASK ? 0x8 : 0;
                        int epsize = endpoint_map->wMaxPacketSize[0];
                        u32 config_reg = 0;
                        phys_ep++;
                        TRACE_MSG32("PHYS EP: %d", phys_ep);
                        TRACE_MSG8("REQ: cfg: %x intf: %x alt: %x typ: %x", 
                                        endpoint_map->configuration, endpoint_map->interface,
                                        endpoint_map->alternate, endpoint_map->bmAttributes[0]);
                        TRACE_MSG8("REQ: cfg: %x intf: %x alt: %x off: %x", 
                                        wmmx_index->configuration, wmmx_index->interface,
                                        wmmx_index->alternate, wmmx_index->offset);
                        /* C.f. Table 12-19 - set and endpoint control register for each active endpoint
                         */
                        TRACE_MSG16("SET ep:  %02x size: %02x", endpoint_map->physicalEndpoint[0], epsize);
                        config_reg |= ((phys_ep << UDCCONR_EN_S) & UDCCONR_EN);                 // Endpoint number
                        TRACE_MSG32("SET cfg: %08x endpoint number", config_reg);
                        config_reg |= ((epsize << UDCCONR_MPS_S) & UDCCONR_MPS);                // Endpoint type and size
                        TRACE_MSG32("SET cfg: %08x Max", config_reg);
                        config_reg |= ((endpoint_map->bmAttributes[0] << UDCCONR_ET_S) & UDCCONR_ET); // Endpoint type and size
                        TRACE_MSG32("SET cfg: %08x type", config_reg);
                        config_reg |=                                   // Endpoint configuration, interface and altsetting
                                ((wmmx_index->configuration << UDCCONR_CN_S) & UDCCONR_CN) |
                                ((wmmx_index->interface << UDCCONR_IN_S) & UDCCONR_IN) |
                                ((wmmx_index->alternate << UDCCONR_AISN_S) & UDCCONR_AISN);
                        TRACE_MSG32("SET cfg: %08x cfg/int/alt", config_reg);

                        if (endpoint_map->bEndpointAddress[0] & USB_DIR_IN) {                      // Endpoint Direction
                                config_reg |= UDCCONR_ED;                                       // Direction: IN 
                                TRACE_MSG32("SET cfg: %08x IN | ED", config_reg);
                        } 
                        else {
                                config_reg &= ~UDCCONR_ED;                                      // Direction: OUT
                                TRACE_MSG32("SET cfg: %08x OUT & ~ED", config_reg);
                        }
                        config_reg |= UDCCONR_EE | UDCCONR_DE;                  // Double Buffering Enabled and Enable the EP 
                        TRACE_MSG32("SET cfg: %08x EE and DE", config_reg);
                        TRACE_MSG16("UDCCN: %02x %02x", i + 1, endpoint_map->physicalEndpoint[0]);
                        UDCCN(phys_ep) = config_reg;     // UDCCRN()                            // Set it 
                        TRACE_MSG32("UDCCCR  %08x", UDCCR);
                }
        }
        return 0;
}
/*
 * Purpose: dump all udc registers and bulverde registers for PM support and debug.
 * Date:    2004/04/27
 * Author: Jordan Wang
 */
void bulverde_udc_regs(void)
{
	printk("\n[%d]UDCCR[%02x]UDCICR[%02x %02x]UDCISR[%02x %02x]UDCFNR[%02x]UDCCSR0[%02x]UDCOTGICR[%02x]UP2OCR[%02x]\n", 
                udc_interrupts,
		UDCCR, 
		UDCICR0, UDCICR1, 
		UDCISR0, UDCISR1,
		UDCFNR, 
		UDCCSR0,
		UDCOTGICR,
		UP2OCR);

	printk("\nICIP[%02x]ICMR[%02x]ICLR[%02x]ICFP[%02x]ICPR[%02x]ICCR[%02x]\n",
		ICIP,
		ICMR,
		ICLR,
		ICFP,
		ICPR,
		ICCR);
}

