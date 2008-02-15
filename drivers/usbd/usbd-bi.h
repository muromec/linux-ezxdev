/*
 * usbd/usbd-bi.h
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
 * Copyright (C) 2005 Motorola Inc.
 *
 *  2005-Nov-11  change for usb2.0,  Li xin
 */

#define MAX_DEVICES 1

/* struct udc_bi_data
 *
 * private data structure for this bus interface driver
 */
struct bi_data {
	int num;
	struct tq_struct cradle_bh;
};


extern unsigned int udc_interrupts;
extern struct usb_bus_instance *usbd_bus;


/* udc_start_endpoint - start endpoint sending or receivign
 *
 * Called with interrupts disabled.
 */
void udc_start_endpoint_in(struct usb_endpoint_instance *);
void udc_start_endpoint_out(struct usb_endpoint_instance *);


/* udc_cancel_in_irq - start transmit
 * udc_cancel_out_irq - start transmit
 *
 * Called with interrupts disabled.
 */
void udc_cancel_in_irq (struct urb *urb);
void udc_cancel_out_irq (struct urb *urb);

/* udc_stall_ep - stall endpoint
 *
 * Stall the endpoint.
 */
void udc_stall_ep (unsigned int ep);

/* udc_reset_ep - reset endpoint
 *
 * returns : 0 if ok, -1 otherwise
 */
void udc_reset_ep (unsigned int ep);

/* udc_endpoint_halted - is endpoint halted
 *
 * Return non-zero if endpoint is halted
 */
int udc_endpoint_halted (unsigned int ep);

/* udc_set_address - set the USB address for this device
 *
 * Called from control endpoint function after it decodes a set address setup packet.
 */
void udc_set_address (unsigned char address);

/* udc_serial_init - set a serial number if available
 */
int __init udc_serial_init (void);

/* udc_max_endpoints - max physical endpoints 
 *
 * Return number of physical endpoints.
 */
int udc_max_endpoints (void);

/* udc_set_ep - setup endpoint 
 *
 * Associate a physical endpoint with endpoint_instance
 */
void udc_setup_ep (unsigned int ep, struct usb_endpoint_instance *endpoint);


/* udc_disable_ep - disable endpoint
 *
 * Disable specified endpoint 
 */
void udc_disable_ep (unsigned int ep);


/* udc_attached - is the USB cable connected
 *
 * Return non-zeron if cable is connected.
 */
int udc_attached (void);

/* udc_connected - is the USB pullup resistor enabled
 *
 * Return non-zeron if pullup resistor enabled
 */
int udc_connected (void);


/* udc_connect - enable pullup resistor
 *
 * Turn on the USB connection by enabling the pullup resistor.
 */
void udc_connect (void);


/* udc_disconnect - disable pullup resistor
 *
 * Turn off the USB connection by disabling the pullup resistor.
 */
void udc_disconnect (void);

/* udc_framenum - fetch framenum
 *
 */
int udc_framenum (void);

/* udc_all_interrupts - enable interrupts
 *
 * Switch on UDC interrupts.
 *
 */
void udc_all_interrupts (void);


/* udc_suspended_interrupts - enable suspended interrupts
 *
 * Switch on only UDC resume interrupt.
 *
 */
void udc_suspended_interrupts (void);

/* udc_disable_interrupts - disable interrupts.
 *
 * switch off interrupts
 */
void udc_disable_interrupts (void);

/* udc_ep0_packetsize - return ep0 packetsize
 */
int udc_ep0_packetsize (void);

/* udc_enable - enable the UDC
 *
 * Switch on the UDC
 */
void udc_enable (void);

/* udc_disable - disable the UDC
 *
 * Switch off the UDC
 */
void udc_disable (void);

/* udc_startup - allow udc code to do any additional startup
 */
void udc_startup (void);

/* udc_startup - allow udc code to do any additional startup
 */
void udc_startup_events (void);

/* udc_init - initialize USB Controller
 * 
 * Get ready to use the USB Controller.
 *
 * Register an interrupt handler and IO region. Return non-zero for error.
 */
int udc_init (void);

/* udc_exit - Stop using the USB Controller
 *
 * Stop using the USB Controller.
 *
 * Shutdown and free dma channels, de-register the interrupt handler.
 */
void udc_exit (void);

/* udc_regs - dump registers
 *
 * Dump registers with printk
 */
void udc_regs (void);

/* udc_name - return name of USB Device Controller
 */
char *udc_name (void);

/* udc_request_udc_irq - request UDC interrupt
 *
 * Return non-zero if not successful.
 */
int udc_request_udc_irq (void);

/* udc_request_cable_irq - request Cable interrupt
 *
 * Return non-zero if not successful.
 */
int udc_request_cable_irq (void);

/* udc_request_udc_io - request UDC io region
 *
 * Return non-zero if not successful.
 */
int udc_request_io (void);

/* udc_release_udc_irq - release UDC irq
 */
void udc_release_udc_irq (void);

/* udc_release_cable_irq - release Cable irq
 */
void udc_release_cable_irq (void);

/* udc_release_io - release UDC io region
 */
void udc_release_io (void);

/* udc_connect_event - called from cradle interrupt handler
 */
void udc_cable_event (void);

void udc_ticker_poke (void);

struct usb_endpoint_map *udc_request_endpoints(int, struct usb_endpoint_request *);
int udc_set_endpoints(int , struct usb_endpoint_map *);

void udc_pm_suspend(void);
void udc_pm_resume(void);

void udc_cable_event_irq(void);
void udc_cable_event(void);

/* bi_rcv_next_irq - complete a receive
 *
 * Called from rcv interrupt to complete.
 */
static __inline__ struct urb * bi_rcv_next_irq (struct usb_endpoint_instance *endpoint)
{
	if (!endpoint->rcv_urb)
		if ((endpoint->rcv_urb = usbd_first_urb_detached_irq (&endpoint->rdy)))
			endpoint->rcv_urb->status = RECV_IN_PROGRESS;
        TRACE_MSG32("BI RCV_URB: %p", (int)endpoint->rcv_urb);
	return endpoint->rcv_urb;
}


/* bi_rcv_complete_irq - complete a receive
 *
 * Called from rcv interrupt to complete.
 */
struct urb * bi_rcv_complete_irq (struct usb_endpoint_instance *endpoint, int len, int urb_bad);
static __inline__ struct urb * _bi_rcv_complete_irq (struct usb_endpoint_instance *endpoint, int len, int urb_bad)
{
	struct urb *rcv_urb;

	// if we had an urb then update actual_length, dispatch if neccessary
	if (likely((int)(rcv_urb = endpoint->rcv_urb))) {

		//printk(KERN_ERR"%s: actual: %d buffer: %d\n", 
		//		__FUNCTION__, rcv_urb->actual_length, rcv_urb->buffer_length);
                
                TRACE_MSG8("BI RCV COMPLETE: actual: %d len: %d bad: %d: status: %d",
                                rcv_urb->actual_length, len, urb_bad, rcv_urb->status);

		TRACE_MSG8("BI RCV COMPLETE: request: %d buffer: %d packet: %d transfer: %d", 
				rcv_urb->request_length, rcv_urb->buffer_length, 
				endpoint->wMaxPacketSize, endpoint->rcv_transferSize);

		// check the urb is ok, are we adding data less than the packetsize
		if (!urb_bad && !endpoint->rcv_error && (rcv_urb->bus->status == USBD_OK) && ((len <= endpoint->wMaxPacketSize) || (rcv_urb->dma_mode))) {

			// increment the received data size
			rcv_urb->actual_length += len;

			// if the current received data is short (less than full packetsize) which
			// indicates the end of the bulk transfer, we have received the maximum
			// transfersize, or if we do not have enough room to receive another packet 
			// then pass this data up to the function driver

			// XXX this needs to be fixed, for example the MSC driver 
			// has varying maximum sizes

			if (
					((len < endpoint->wMaxPacketSize) || (rcv_urb->dma_mode) ||
					 (rcv_urb->actual_length >= endpoint->rcv_transferSize) || 
					 (rcv_urb->actual_length >= rcv_urb->request_length) || 
					 (rcv_urb->actual_length + endpoint->wMaxPacketSize > rcv_urb->buffer_length)))
			{
#if 1
				int i;
				for (i = 0; i < rcv_urb->actual_length; TRACE_RECV(rcv_urb->buffer + i), i+= 8);
#endif
				endpoint->rcv_urb = NULL;
				rcv_urb->jiffies = jiffies;
				rcv_urb->framenum = udc_framenum ();
                                TRACE_MSG32("BI RCV COMPLETE: finished length: %d", rcv_urb->actual_length);
				TRACE_MSG32("BI RCV COMPLETE: framenum: %x", (int) rcv_urb->framenum);
                                usbd_urb_recv_finished_irq (rcv_urb, RECV_OK);
				rcv_urb = NULL;
			}
		} 
		else {
			rcv_urb->actual_length = 0;
			//endpoint->rcv_error = 1;
		}
	} 

	// if we don't have an urb see if we can get one
	return bi_rcv_next_irq(endpoint);
}

static __inline__ void bi_rcv_cancelled_irq(struct usb_endpoint_instance *endpoint)
{
        struct urb *rcv_urb;

	TRACE_MSG32("BI RCV CANCELLED: %p", (int) endpoint->rcv_urb);
        RETURN_IF (!(rcv_urb = endpoint->rcv_urb));
//	printk(KERN_INFO"%s: rcv_urb: %p\n", __FUNCTION__, endpoint->rcv_urb);
        usbd_urb_recv_finished_irq(rcv_urb, RECV_CANCELLED);
	endpoint->sent = endpoint->last = 0;
        //endpoint->rcv_urb = bi_rcv_next_irq(endpoint);
        endpoint->rcv_urb = NULL;
	printk(KERN_INFO"%s: rcv_urb: %p\n", __FUNCTION__, endpoint->rcv_urb);
}


/* bi_tx_next_irq - complete a receive
 *
 * Called from tx interrupt to complete.
 */
static __inline__ struct urb * bi_tx_next_irq (struct usb_endpoint_instance *endpoint)
{
	if (!endpoint->tx_urb)
		if ((endpoint->tx_urb = usbd_first_urb_detached_irq (&endpoint->tx))) {
#if 1
			int i;
			TRACE_MSG16("NEXT TX: length: %d flags: %x", endpoint->tx_urb->actual_length, endpoint->tx_urb->flags);
			for (i = 0; i < endpoint->tx_urb->actual_length; TRACE_SENT(endpoint->tx_urb->buffer + i), i+= 8);
#endif
			endpoint->tx_urb->status = SEND_IN_PROGRESS;
		}
	TRACE_MSG32("BI TX NEXT TX_URB: %p", (int)endpoint->tx_urb);
	return endpoint->tx_urb;
}


/* bi_tx_complete_irq - complete a transmit
 *
 * Called from tx interrupt to complete.
 */
struct urb * bi_tx_complete_irq (struct usb_endpoint_instance *endpoint, int restart);
static __inline__ struct urb * _bi_tx_complete_irq (struct usb_endpoint_instance *endpoint, int restart)
{
        struct urb *tx_urb;
	//RETURN_NULL_IF(!endpoint);

        //printk(KERN_INFO"%s: endpoint: %p %02x rc: %d\n", __FUNCTION__, endpoint, endpoint->bEndpointAddress, restart);

        // if we have a tx_urb advance or reset, finish if complete
        if ((likely((int)tx_urb = endpoint->tx_urb))) {

                TRACE_MSG32("BI TX CURRENT TX_URB: %p", (int)endpoint->tx_urb);
                TRACE_MSG8("BI TX COMPLETE: actual: %d sent: %d last: %d: status: %d",
                                tx_urb->actual_length, endpoint->sent, endpoint->last, tx_urb->status);

                if (likely(!restart)) {
                        int sent = endpoint->last;
                        endpoint->sent += sent;
                        endpoint->last -= sent;
                }
                else 
                        endpoint->last = 0;

                //printk(KERN_INFO"%s: act: %d last: %d sent: %d status: %d flags: %x\n", __FUNCTION__,
                //		endpoint->tx_urb->actual_length, endpoint->last, endpoint->sent, 
		//		tx_urb->status, tx_urb->flags);

		// XXX is the test for bEndpointAddress still appropriate for CONTROL WRITES
                if (((tx_urb->actual_length - endpoint->sent) <= 0) && !(tx_urb->flags & USBD_URB_SENDZLP) ) {
                        //if (endpoint->bEndpointAddress) 
                        tx_urb->jiffies = jiffies;
			tx_urb->framenum = udc_framenum ();
                        TRACE_MSG32("BI TX COMPLETE: finished tx_urb: %p", (int)tx_urb);
                        TRACE_MSG32("BI TX COMPLETE: framenum: %x", (int)tx_urb->framenum);
                        usbd_urb_sent_finished_irq (tx_urb, SEND_FINISHED_OK);
                        endpoint->tx_urb = NULL;
                        endpoint->last = endpoint->sent = 0;
                }
        }
        return bi_tx_next_irq(endpoint);
}

static __inline__ void bi_tx_cancelled_irq(struct usb_endpoint_instance *endpoint)
{
        struct urb *tx_urb;

	TRACE_MSG32("BI TX CANCELLED: %p", (int) endpoint->tx_urb);
        RETURN_IF (!(tx_urb = endpoint->tx_urb));
	//printk(KERN_INFO"%s: tx_urb: %p\n", __FUNCTION__, endpoint->tx_urb);
        usbd_urb_sent_finished_irq(tx_urb, SEND_FINISHED_CANCELLED);
	endpoint->sent = endpoint->last = 0;
        //endpoint->tx_urb = bi_tx_next_irq(endpoint);
        endpoint->tx_urb = NULL;
	//printk(KERN_INFO"%s: tx_urb: %p\n", __FUNCTION__, endpoint->tx_urb);
}

static __inline__ int bi_tx_sendzlp(struct usb_endpoint_instance *endpoint)
{
	struct urb *tx_urb = endpoint->tx_urb;
	RETURN_ZERO_IF (!tx_urb || (tx_urb->actual_length != endpoint->sent) || !(tx_urb->flags & USBD_URB_SENDZLP)); 
        tx_urb->flags &= ~USBD_URB_SENDZLP;
        return 1;
}

/* bi_rcv_complete_irq - complete a receive
 *
 * Called from rcv interrupt to complete.
 */
static __inline__ void bi_rcv_fast_complete_irq (struct usb_endpoint_instance *endpoint, struct urb *rcv_urb)
{
	TRACE_MSG32("BI RCV FAST COMPLETE: %d", rcv_urb->actual_length);
        usbd_urb_recv_finished_irq(rcv_urb, RECV_OK);
} 

/* bi_recv_setup - process a device request
 * 
 * Note that we verify if a receive urb has been queued for H2D with non-zero wLength
 * and return -EINVAL to stall if the upper layers have not properly tested for and 
 * setup a receive urb in this case.
 */
static __inline__ int bi_recv_setup_irq(struct usb_device_request *request)
{
        struct usb_endpoint_instance *endpoint = usbd_bus->endpoint_array + 0;
	TRACE_SETUP(request);
	RETURN_EINVAL_IF (usbd_recv_setup_irq(usbd_bus->ep0, request));         // fail if already failed
        RETURN_ZERO_IF ((request->bmRequestType & USB_REQ_DIRECTION_MASK) == USB_REQ_DEVICE2HOST);      
        RETURN_ZERO_IF (!le16_to_cpu(request->wLength));
        RETURN_EINVAL_IF (!endpoint->rcv_urb);
        return 0;
}

/* bi_ep0_reset_irq - reset ep0 endpoint 
 */
static void __inline__ bi_ep0_reset_endpoint_irq(struct usb_endpoint_instance *endpoint)
{
        bi_tx_cancelled_irq(endpoint);
        bi_rcv_cancelled_irq(endpoint);
        endpoint->sent = endpoint->last = 0;
}
