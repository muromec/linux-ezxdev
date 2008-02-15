/*
 * linux/drivers/usbd/bi/mx2.c
 * USB Device Controller driver,
 * platform-dependent part for Motorola i.MX21.
 * To be used with usbd-bi.c and ../../usb-mx2otg
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 * Copyright (c) 2003 MontaVista Software Inc. <source@mvista.com>
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

#include <linux/config.h>
#include <linux/module.h>

#include "../usbd-export.h"
#include "../usbd-build.h"
#include "../usbd-module.h"
#include "../usbd-debug.h"
#include "../usbd-arch.h"

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/init.h>

#include <asm/atomic.h>
#include <asm/io.h>

#include <linux/netdevice.h>

#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/system.h>

#include <asm/types.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>
#include <linux/delay.h>

#include "../usbd.h"
#include "../usbd-func.h"
#include "../usbd-bus.h"
#include "../usbd-inline.h"
#include "usbd-bi.h"
#include "mx2.h"
#include "../../usb-mx2otg.h"

static int mx2udc_otg_enabled;
static int storage_flag = 0;
static int winhost = 0;
static int USBBusResetDetected = 0;
u32 EP_active;			/*initialise all eps to be inactive */

void
toggle_ready_ep(u8 ep_num, u8 ready)
{

	u8 rAlreadyReady;
	u32 mask = 1 << ep_num;

	rAlreadyReady = check_ep_ready(ep_num);
	if (rAlreadyReady && ready) {
		/*$display("INFO: toggle_ready_ep was called but EP %0x (%x) was already ready! It will not be modified.", ep_num, ep_state); */
	} else {
		if (!rAlreadyReady && !ready) {
			/*$display("INFO: toggle_ready_ep was called but EP %0x (%x) was already un-ready! It will not be modified.", ep_num, ep_state); */
		} else {
			OTG_FUNC_EP_RDY = mask;	/* write 1 to toggle current state */
		}
	}
}

void
EPConfig(u32 ep, u32 dir, u32 stall, u32 setup, u32 MPS, u32 format, u32 xbsa,
	 u32 ybsa, u32 Bufsize, u32 Tbyte)
{

	u32 DWORD0 = 0;
	u32 DWORD1 = 0;
	u32 DWORD3 = 0;
	u32 localEp = ep;
	u32 DWORD0_addr, DWORD1_addr, DWORD2_addr, DWORD3_addr;
	u8 rAlreadyReady;

	/*Unready the ep if it is currently Ready */
	rAlreadyReady = check_ep_ready(((ep * 2) + dir));
	if (rAlreadyReady) {
		/*$display("INFO: EPConfig was called but EP %0x (%x) was already enabled! Clearing ready.", endpt_num, direction); */
		toggle_ready_ep(((ep * 2) + dir), 0);
	}

	DWORD0_addr = OTG_EP_BASE + (16 * ((localEp * 2) + dir));
	DWORD1_addr = DWORD0_addr + 4;
	DWORD2_addr = DWORD1_addr + 4;
	DWORD3_addr = DWORD2_addr + 4;

	/*Configuring DWORD0 */
	DWORD0 =
	    (stall << 31) | (setup << 30) | (0 << 26) | (MPS << 16) | (format <<
								       14) | 0;
	__REG32(DWORD0_addr) = DWORD0;

	/*Configuring DWORD1 */
	DWORD1 = (ybsa << 18) | (0 << 16) | (xbsa << 2) | 0;
	__REG32(DWORD1_addr) = DWORD1;

	/*Configuring DWORD3 */
	DWORD3 = (Bufsize << 21) | Tbyte;
	__REG32(DWORD3_addr) = DWORD3;

	/*Configure EP Table Entry */

	/*devId parameter not used in EP config */
	/*activedirection not used in EP's environment */

}

u8
check_ep_ready(u8 qEpNumberDirection)
{

	u8 mask = 1 << qEpNumberDirection;
	if (OTG_FUNC_EP_RDY & mask)
		return 1;	/* ready */
	else
		return 0;	/* not ready */
}

void
Enable_EP(u8 ep_num_mask)
{

	u32 ep_state = 0;

	ep_state = OTG_FUNC_EP_EN;

	if (ep_state & ep_num_mask) {
	} else {
		OTG_FUNC_EP_EN = ep_state | ep_num_mask;	/* enable the EP if it's not enabled */
	}
}

void
Ready_EP_OUT(u8 ep)
{

	u8 ep_num = (ep * 2) + 0;
	u32 mask = 1 << ep_num;

	toggle_ready_ep(ep_num, 1);
	EP_active = EP_active | mask;	/* set the EP active for DGC purpose */
}

static void
FunctionSetup(void)
{
	if (!mx2udc_otg_enabled) return;

	if (OTG_FUNC_CND_STAT & BIT0)	/* RESET DET set */
		OTG_FUNC_CND_STAT = 0x0;

	if (OTG_FUNC_SINT_STAT & BIT0)
		OTG_FUNC_SINT_STAT = BIT0;

	/*  Configure EP0 for CTRL SETUP (8 bytes) */
	EPConfig(0, EP_OUT_DIR, EP_NO_STALL, EP_NO_SETUP, 8, CTL_TYPE, 0, 0, 8,
		 8);
	Enable_EP(EP0OUT);	/* Enable EP0-OUT to accept CTRL SETUP */
	Ready_EP_OUT(0);	/* Ready EP0 */
	Enable_EP(EP0IN);	/* Enable EP0-IN to accept CTRL DATA, if ready not set will return NAK */
	Enable_EP(EP1IN);
	Enable_EP(EP2OUT);

	/* interrupts */
	OTG_FUNC_EP_DEN = EP0OUT | EP2OUT;
	OTG_FUNC_SINT_STAT = BIT3 | BIT2 | BIT0;
	OTG_FUNC_SINT_STEN = BIT3 | BIT2 | BIT0;

}


void
SendNullPacket(void)
{

	EPConfig(EP0, EP_IN_DIR, EP_NO_STALL, EP_NO_SETUP, 8, CTL_TYPE, 0, 0, 8,
		 0);

	WriteEPToggleBit(EP0, EP_IN_DIR, 1);	/* Toggle Status Stage In PID from DATA0 to DATA1 */

	ClearXYBufferInt();

	/* send IN with Empty packet to complete the status stage */
	/* set xfill to indicate data memory filled of EP0 IN */
	OTG_FUNC_XFILL_STAT = EP0IN;

	Ready_EP_IN(0);

	/* check for EP0 IN transfer done to send the Empty packet */
	while (!(OTG_FUNC_EP_DSTAT & EP0IN)) ;

	/* clear EP0 IN transfer done */
	OTG_FUNC_EP_DSTAT |= EP0IN;
	ClearXYBufferInt();

	unready_ep_in(EP0);

}

void
SetClearFillStatus(u8 mask)
{
	OTG_FUNC_XFILL_STAT = mask;	/*write to toggle the correct EP's X buffer status */
}

void
ClearXYBufferInt()
{
	u32 temp = 0;

	temp = OTG_FUNC_XINT_STAT;
	OTG_FUNC_XINT_STAT = temp;

	temp = OTG_FUNC_YINT_STAT;
	OTG_FUNC_YINT_STAT = temp;

}

void
WriteEPToggleBit(u8 ep, u8 dir, u8 toggle_val)
{
	u8 ep_num = (ep * 2) + dir;
	u32 temp;
	u32 toggle_mask;
	u32 ep_mask = 1 << ep_num;

	toggle_mask = (toggle_val & 1) << ep_num;

	temp = OTG_FUNC_EP_TOGGLE;
	if ((temp & ep_mask) != toggle_mask) {
	/* write to the register only if necessary */
#ifdef CONFIG_MX2TO1
		/* in TO1 this register is a regular register */
		OTG_FUNC_EP_TOGGLE = (temp & ~ep_mask) | toggle_mask;
#else
		/* in TO2 this register is a toggle register */
		OTG_FUNC_EP_TOGGLE = ep_mask;
#endif
	}
}
void
Ready_EP_IN(u8 ep)
{

	u8 ep_num = (ep * 2) + 1;
	u32 mask = 1 << ep_num;
	toggle_ready_ep(ep_num, 1);
	EP_active = EP_active | mask;	/* set the EP active for DGC purpose */
}

void
unready_ep_in(u8 ep)
{

	u8 ep_num = (ep * 2) + 1;

	toggle_ready_ep(ep_num, 0);
	Clear_active_ep(ep_num);
}

void
unready_ep_out(u8 ep)
{

	u8 ep_num = (ep * 2) + 0;

	toggle_ready_ep(ep_num, 0);
	Clear_active_ep(ep_num);
}

void
Clear_active_ep(u8 ep_num)
{
	u32 mask = 1 << ep_num;

	if (EP_active & mask) {	/* if "1", clear to "0" */
		EP_active -= mask;
	}			/*else, do nothing as the Ep is already inactive. */
}

void
EP0OutForRequest(void)
{
	/*  Configure EP0 for CTRL SETUP (8 bytes) */
	EPConfig(0, EP_OUT_DIR, EP_NO_STALL, EP_NO_SETUP, 8, CTL_TYPE, 0, 0, 8,
		 8);

}

void
EP0Out_StatusStageTransferDone(void)
{

	/*  Configure EP0 for CTRL SETUP (0 bytes) */
	EPConfig(0, EP_OUT_DIR, EP_NO_STALL, EP_NO_SETUP, 8, CTL_TYPE, 0, 0, 7,
		 0);
	Ready_EP_OUT(0);	/* Ready EP0 */

	while (!(OTG_FUNC_EP_DSTAT & EP0OUT)) ;

	OTG_FUNC_EP_DSTAT |= EP0OUT;

	EP0OutForRequest();

}

MODULE_AUTHOR("MontaVista Software Inc");
MODULE_DESCRIPTION("Motorola i.MX21 USB Function");
MODULE_LICENSE("GPL");

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

static struct usb_device_instance *udc_device;

/*
 * ep_endpoints - map physical endpoints to logical endpoints
 */
static struct usb_endpoint_instance *ep_endpoints[UDC_MAX_ENDPOINTS];

static struct urb *ep0_urb;
extern unsigned int udc_interrupts;

/* ********************************************************************************************* */

static void __inline__
mx2_ep0_start(unsigned int ep, struct usb_endpoint_instance *endpoint,
	      int needzlp)
{

	u32 EpStartAddress = 0;
	u32 ep_num = 0;
	u8 dir = 0;
	int k;
	unsigned int *bp;

	if (endpoint->tx_urb) {
		struct urb *urb = endpoint->tx_urb;

		EPConfig(EP0, EP_IN_DIR, EP_NO_STALL, EP_NO_SETUP,
			 8 /* packet size */ , CTL_TYPE, (0 * 16),
			 200 /* ybsa */ ,
			 256 /* bufsize */ ,
			 urb->actual_length /* bytes to transfer */ );

		WriteEPToggleBit(EP0, EP_IN_DIR, 1);

		dir = EP_OUT_DIR;
		ep_num = (ep * 2) + dir;

		EpStartAddress =
		    ((__REG32(OTG_EP_BASE + (16 * 1) + 4)) & 0x0000FFFF);

		bp = (unsigned int *) urb->buffer;
		for (k = 0; k < (urb->actual_length + 3) / 4; k++) {
			__REG32(OTG_DATA_BASE + EpStartAddress + (k * 0x4)) =
			    *bp++;
		}

		ClearXYBufferInt();
		Ready_EP_IN(0);

		SetClearFillStatus(EP0IN);

		if (needzlp) {
			while (!(OTG_FUNC_EP_DSTAT & EP0IN)) ;
			OTG_FUNC_EP_DSTAT |= EP0IN;
			ClearXYBufferInt();
			unready_ep_in(EP0);
			EPConfig(EP0, EP_IN_DIR, 0, 0, 8, CTL_TYPE, (0 * 16),
				 200, 8, 0);
			SetClearFillStatus(EP0IN);
			ClearXYBufferInt();
			Ready_EP_IN(0);
		}

		while (!(OTG_FUNC_EP_DSTAT & EP0IN)) ;

		OTG_FUNC_EP_DSTAT |= EP0IN;
		ClearXYBufferInt();

		unready_ep_in(EP0);

		EP0Out_StatusStageTransferDone();	/* ZLP */

	}
}

void
ep0_out_interrupt(void)
{

	int i;
	unsigned char *cp = (unsigned char *) &ep0_urb->device_request;
	struct usb_endpoint_instance *endpoint;

	int j = 0;
	u32 ep_num = 0;
	u32 ep = 0;
	u8 dir = 0;
	u32 mask = 0;
	u32 TempBuffer = 0;
	u32 EpStartAddress = 0;
	static int needzlp = 0;

	endpoint = ep_endpoints[0];

	ep = 0;
	dir = EP_OUT_DIR;
	ep_num = (ep * 2) + dir;
	mask = 1 << ep_num;

	EpStartAddress =
	    ((__REG32((OTG_EP_BASE) + (16 * ep_num) + 4)) & 0x0000FFFF);

	for (i = 0; i < 2; i++) {
		TempBuffer = __REG32(OTG_DATA_BASE + EpStartAddress + i * 4);
		for (j = 0; j < 4; j++) {
			*cp = (u8) ((TempBuffer >> 8 * j) & 0xFF);
			cp++;
		}
	}

	SetClearFillStatus(mask);

	if (ep0_urb->device_request.bRequest == 0x05) {
		SendNullPacket();
	}
	if (ep0_urb->device_request.bRequest == 0x09) {
		SendNullPacket();
	}

	if (storage_flag) {
		if (ep0_urb->device_request.bmRequestType == 0x80 && ep0_urb->device_request.bRequest == 0x06 && __swab16(ep0_urb->device_request.wValue) == 0x01 && ep0_urb->device_request.wLength == 0x40) {	/* Win 2000 bug */
			winhost = 1;
			ep0_urb->device_request.wLength = 0x8;
		}

		if (ep0_urb->device_request.wLength >= 0xff  && (__swab16(ep0_urb->device_request.wValue) & 0xf) != 3) { /* not GET STRING packet */
			ep0_urb->device_request.wLength = 32;
			needzlp = 1;
		}

	}

	if (usbd_recv_setup(ep0_urb)) {
		printk(KERN_ERR "BAD SETUP");
		return;
	}

	if ((ep0_urb->device_request.
	     bmRequestType & USB_REQ_DIRECTION_MASK) == USB_REQ_HOST2DEVICE) {

		/* Control Write - we are receiving more data from the host */
		ep0_urb->actual_length = 0;

	} else {

		/* Control Read - we are sending data to the host */

		if (!(ep0_urb->device_request.wLength)) {

			udc_stall_ep(0);
			return;
		}
		/* verify that we have non-zero length response */
		if (!ep0_urb->actual_length) {
			udc_stall_ep(0);
			return;
		}
		/*  start sending */
		endpoint->tx_urb = ep0_urb;
		endpoint->sent = 0;
		endpoint->last = 0;
		endpoint->state = 1;

		mx2_ep0_start(0, endpoint, needzlp);
		needzlp = 0;

	}

}

void
ep1_in_interrupt(void)
{
	struct usb_endpoint_instance *endpoint;
	endpoint = ep_endpoints[1];

	OTG_FUNC_EP_DSTAT |= EP1IN;
	ClearXYBufferInt();

	unready_ep_in(EP1);

}

void
ep2_out_interrupt(void)
{
	struct usb_endpoint_instance *endpoint;
	u32 len = 0;
	unsigned int i = 0, j = 0;
	struct urb *rcv_urb;

	endpoint = ep_endpoints[2];

	u32 ep = 2;
	u8 dir = EP_OUT_DIR;
	u32 TempBuffer = 0;
	u32 EpStartAddress = 0;
	u32 ep_num = (ep * 2) + dir;;
	u32 mask = 1 << ep_num;
	static int bytecounter;
	unsigned int size;

	bytecounter = BYTECOUNT - (OTG_FUNC_DWORD3_EP2OUT & BYTECOUNTMASK);
	size = bytecounter;

	ClearXYBufferInt();
	unready_ep_out(EP2);

	EpStartAddress =
	    ((__REG32(OTG_EP_BASE + (16 * ep_num) + 4)) & 0x0000FFFF);

	if (endpoint) {
		if (!endpoint->rcv_urb) {
			endpoint->rcv_urb = first_urb_detached(&endpoint->rdy);
		}

		if (endpoint->rcv_urb) {
			rcv_urb = endpoint->rcv_urb;

			unsigned char *cp;
			unsigned int *bp = (unsigned int *)
			    (endpoint->rcv_urb->buffer +
			     endpoint->rcv_urb->actual_length);
			if (bp) {
				for (i = 0; size >= 4; i++, size -= 4) {
					TempBuffer =
					    __REG32(OTG_DATA_BASE +
						    EpStartAddress + i * 4);
					*bp++ = TempBuffer;
					if ((i + 1) % (32 / 4) == 0) {
						usbd_rcv_complete_irq(endpoint,
								      32, 0);
						len += 32;
						if (endpoint->rcv_urb
						    && rcv_urb !=
						    endpoint->rcv_urb) {
							rcv_urb =
							    endpoint->rcv_urb;
							bp = (unsigned int
							      *) (endpoint->
								  rcv_urb->
								  buffer +
								  endpoint->
								  rcv_urb->
								  actual_length);
						}
					}
				}

				cp = (u8 *) bp;
				TempBuffer =
				    __REG32(OTG_DATA_BASE + EpStartAddress +
					    i * 4);

				for (j = 0; j < size; j++) {
					*cp =
					    (u8) ((TempBuffer >> 8 * j) & 0xFF);
					cp++;
				}

				usbd_rcv_complete_irq(endpoint,
						      bytecounter - len, 0);
				if (endpoint->rcv_urb
				    && rcv_urb != endpoint->rcv_urb) {
					rcv_urb = endpoint->rcv_urb;
					bp = (unsigned int *) (endpoint->
							       rcv_urb->buffer +
							       endpoint->
							       rcv_urb->
							       actual_length);
				}

				if (storage_flag && endpoint->rcv_urb
				    && endpoint->rcv_urb->actual_length ==
				    STORAGE_BLOCK_SIZE) {
					usbd_rcv_complete_irq(endpoint, 0, 0);
					if (endpoint->rcv_urb
					    && rcv_urb != endpoint->rcv_urb) {
						rcv_urb = endpoint->rcv_urb;
						cp = endpoint->rcv_urb->buffer +
						    endpoint->rcv_urb->
						    actual_length;
					}
				}

			} else {
				printk(KERN_ERR
				       "read[%d] bad arguements\n",
				       udc_interrupts);
			}

		} else {

			printk(KERN_ERR " no rcv_urb\n");

		}

	}

	SetClearFillStatus(mask);
	ClearXYBufferInt();
	EPConfig(EP2, EP_OUT_DIR, EP_NO_STALL, EP_NO_SETUP, 32, BLK_TYPE, 0, 0,
		 BYTECOUNT, BYTECOUNT);
	Ready_EP_OUT(2);
}

void
ep2_out_dma(void)
{
	struct usb_endpoint_instance *endpoint;
	u32 len = 0;
	unsigned int i = 0;
	struct urb *rcv_urb;
	static int bytecounter;
	unsigned int size;
	unsigned char *cp = NULL;
	endpoint = ep_endpoints[2];

	bytecounter = BYTECOUNT - (OTG_FUNC_DWORD3_EP2OUT & BYTECOUNTMASK);
	size = bytecounter;

	OTG_FUNC_EP_DSTAT = EP2OUT;
	unready_ep_out(EP2);

	if (endpoint) {
		if (endpoint->rcv_urb) {
			rcv_urb = endpoint->rcv_urb;

			for (i = 0; i < size; i++) {
				if ((i + 1) % 32 == 0) {
					usbd_rcv_complete_irq(endpoint, 32, 0);
					len += 32;
				}
			}

			if (size - len > 0) {
				usbd_rcv_complete_irq(endpoint, size - len, 0);
			}

			if (endpoint->rcv_urb && rcv_urb != endpoint->rcv_urb) {
				rcv_urb = endpoint->rcv_urb;
				cp = endpoint->rcv_urb->buffer +
				    endpoint->rcv_urb->actual_length;

			}

			if (storage_flag && endpoint->rcv_urb
			    && endpoint->rcv_urb->actual_length ==
			    STORAGE_BLOCK_SIZE) {
				usbd_rcv_complete_irq(endpoint, 0, 0);
				if (endpoint->rcv_urb
				    && rcv_urb != endpoint->rcv_urb) {
					rcv_urb = endpoint->rcv_urb;
					cp = endpoint->rcv_urb->buffer +
					    endpoint->rcv_urb->actual_length;
				}
			}

			OTG_DMA_EP2_O_MSA = (unsigned long) cp;

		} else {

			printk(KERN_ERR " no rcv_urb\n");

		}

	}
	EPConfig(EP2, EP_OUT_DIR, EP_NO_STALL, EP_NO_SETUP, 32, BLK_TYPE, 0, 0,
		 BYTECOUNT, BYTECOUNT);
	OTG_DMA_EP_EN |= EP2OUT;
	Ready_EP_OUT(2);
}

static void __inline__
mx2_ep1_start(unsigned int ep, struct usb_endpoint_instance *endpoint,
	      int restart)
{

	u32 EpStartAddress = 0;
	int k;
	unsigned int *bp;

	if (endpoint->tx_urb) {
		struct urb *urb = endpoint->tx_urb;

		if (urb->actual_length == 0) {
			usbd_tx_complete_irq(endpoint, 0);
			return;
		}

		while (TRUE) {

			int last =
			    ((urb->actual_length -
			      (endpoint->sent + endpoint->last)) >
			     512) ? 512 : (urb->actual_length -
					   (endpoint->sent + endpoint->last));

			EPConfig(EP1, EP_IN_DIR, EP_NO_STALL, EP_NO_SETUP,
				 32 /* packet size */ , BLK_TYPE, 0x100,
				 0x200 /* ybsa */ ,
				 last /* bufsize */ ,
				 last /* bytes to transfer */ );

			int size = last;
			unsigned char *cp =
			    urb->buffer + endpoint->sent + endpoint->last;
			bp = (int *) cp;

			EpStartAddress =
			    ((__REG32(OTG_EP_BASE + (16 * 3) + 4)) &
			     0x0000FFFF);

			for (k = 0; k < (size + 3) / 4; k++) {

				__REG32(OTG_DATA_BASE + EpStartAddress +
					(k * 0x4)) = *bp++;
			}

			ClearXYBufferInt();
			Ready_EP_IN(1);

			SetClearFillStatus(EP1IN);

			endpoint->last += last;

			int timeout = 0xFFFF;
			while (!(OTG_FUNC_EP_DSTAT & EP1IN)) {
				timeout--;
				if (timeout == 0) {
					break;
				}
			}

			OTG_FUNC_EP_DSTAT |= EP1IN;
			ClearXYBufferInt();

			unready_ep_in(EP1);

			usbd_tx_complete_irq(endpoint, 0);

			if (endpoint->tx_urb == NULL) {
				break;
			}

		}
	}
	if (OTG_FUNC_XFILL_STAT & EP2OUT) {
		ep2_out_interrupt();
	}
}

static void __inline__
mx2_dma_ep1_start(unsigned int ep, struct usb_endpoint_instance *endpoint,
		  int restart)
{

	u32 timeout = TIMEOUT_VALUE;

	if (endpoint->tx_urb) {
		struct urb *urb = endpoint->tx_urb;

		if (urb->actual_length == 0) {
			usbd_tx_complete_irq(endpoint, 0);
			return;
		}

		while (TRUE) {

			int last =
			    ((urb->actual_length -
			      (endpoint->sent + endpoint->last)) >
			     512) ? 512 : (urb->actual_length -
					   (endpoint->sent + endpoint->last));

			EPConfig(EP1, EP_IN_DIR, EP_NO_STALL, EP_NO_SETUP,
				 32 /* packet size */ , BLK_TYPE, 0x100,
				 0x200 /* ybsa */ ,
				 last /* bufsize */ ,
				 last /* bytes to transfer */ );

			unsigned char *cp =
			    urb->buffer + endpoint->sent + endpoint->last;

			OTG_DMA_EP1_I_MSA = (unsigned long) cp;

			OTG_DMA_EP_EN |= EP1IN;

			Ready_EP_IN(1);

			timeout = TIMEOUT_VALUE;
			while (OTG_DMA_EP_EN & EP1IN) {
				timeout--;
				if (!timeout)
					break;
			}

			timeout = TIMEOUT_VALUE;
			while (!(OTG_FUNC_EP_DSTAT & EP1IN)) {
				timeout--;
				if (!timeout)
					break;
			}

			if (OTG_FUNC_EP_DSTAT & EP1IN) {
				OTG_FUNC_EP_DSTAT = EP1IN;
			}
			/* due to the MOT-ERRATA1 */
			ClearXYBufferInt();

			unready_ep_in(EP1);

			endpoint->last += last;
			dbg_tx(3, "endpoint->sent=%d", endpoint->sent);
			usbd_tx_complete_irq(endpoint, 0);

			if (endpoint->tx_urb == NULL) {
				break;
			}

		}
	}
	if (OTG_FUNC_XFILL_STAT & EP2OUT) {
		if (storage_flag) {
			ep2_out_dma();
		} else {
			ep2_out_interrupt();
		}
	}
}

void
mx2_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned int status = OTG_FUNC_EP_DSTAT;

	if (OTG_FUNC_SINT_STAT & BIT3) {	/* Done */
	if (status & EP0OUT) {
		OTG_FUNC_EP_DSTAT |= EP0OUT;
		ep0_out_interrupt();

	} else if (status & EP1IN) {
		udelay(100);
		ep1_in_interrupt();

	}
	if (OTG_FUNC_EP_DSTAT & EP2OUT) {
		if (storage_flag) {
			ep2_out_dma();
		} else {
			ep2_out_interrupt();
			OTG_FUNC_EP_DSTAT = EP2OUT;
		}
		}
		OTG_FUNC_SINT_STAT |= BIT3;	/* clear interrupts */

	}

	if (OTG_FUNC_SINT_STAT & BIT0) {	/* USBBusResetDetected */

		USBBusResetDetected = 1;
		usbd_device_event(udc_device, DEVICE_DESTROY, 0);
		usbd_device_event(udc_device, DEVICE_CREATE, 0);
		usbd_device_event(udc_device, DEVICE_RESET, 0);
		OTG_FUNC_CND_STAT |= BIT7;

		FunctionSetup();

		OTG_FUNC_SINT_STAT |= BIT0;	/* clear interrupts */
		USBBusResetDetected = 0;
	}
	if (OTG_FUNC_SINT_STAT & BIT2) {	/* SuspendDetected */
		OTG_FUNC_SINT_STAT |= BIT2;	/* clear interrupts */
	}
	if (OTG_FUNC_SINT_STAT & BIT4) {	/* SOF */
		OTG_FUNC_SINT_STAT |= BIT4;	/* clear interrupts */
	}

}

/* ********************************************************************************************* */
/*
 * Start of public functions.
 */

/**
 * udc_start_in_irq - start transmit
 * @eendpoint: endpoint instance
 *
 * Called by bus interface driver to see if we need to start a data transmission.
 */
void
udc_start_in_irq(struct usb_endpoint_instance *endpoint)
{

	if (storage_flag) {
		if (endpoint->tx_urb->actual_length >= 512)
			mx2_dma_ep1_start(endpoint->endpoint_address & 0x7f,
					  endpoint, 0);
		else
			mx2_ep1_start(endpoint->endpoint_address & 0x7f,
				      endpoint, 0);
	} else {
		mx2_ep1_start(endpoint->endpoint_address & 0x7f, endpoint, 0);
	}
}

/**
 * udc_init - initialize
 *
 * Return non-zero if we cannot see device.
 **/
int
udc_init(void)
{
	/*initialization is done in usb-mx2otg*/
	return 0;
}

/**
 * udc_start_in - start transmit
 * @eendpoint: endpoint instance
 *
 * Called by bus interface driver to see if we need to start a data transmission.
 */
void
udc_start_in(struct usb_endpoint_instance *endpoint)
{
	if (endpoint) {
		unsigned long flags;
		local_irq_save(flags);
		if (!endpoint->tx_urb) {
			mx2_ep1_start(endpoint->endpoint_address & 0x7f,
				      endpoint, 0);
		}
		local_irq_restore(flags);
	}
}

/**
 * udc_stall_ep - stall endpoint
 * @ep: physical endpoint
 *
 * Stall the endpoint.
 */
void
udc_stall_ep(unsigned int ep)
{

	u32 timeout = TIMEOUT_VALUE;

	dbg_stall(3, " udc_stall_ep \n");
	if (ep < UDC_MAX_ENDPOINTS) {

		if (storage_flag) {
			EPConfig(EP1, EP_IN_DIR, EP_NO_STALL, 0, 32, BLK_TYPE,
				 0x100, 0x200, 32, 0);
			ClearXYBufferInt();
			SetClearFillStatus(EP1IN);
			Ready_EP_IN(1);
			while (!(OTG_FUNC_EP_DSTAT & EP1IN)) {
				timeout--;
				if (!timeout)
					break;
			}

			OTG_FUNC_EP_DSTAT |= EP1IN;
			ClearXYBufferInt();
			unready_ep_in(EP1);
		}

	}
}

/**
 * udc_reset_ep - reset endpoint
 * @ep: physical endpoint
 * reset the endpoint.
 *
 * returns : 0 if ok, -1 otherwise
 */
void
udc_reset_ep(unsigned int ep)
{
	if (ep < UDC_MAX_ENDPOINTS) {
	}
}

/**
 * udc_endpoint_halted - is endpoint halted
 * @ep:
 *
 * Return non-zero if endpoint is halted
 */
int
udc_endpoint_halted(unsigned int ep)
{
	dbg_tx(3, " udc_endpoint_halted \n");
	return 0;
}

/**
 * udc_set_address - set the USB address for this device
 * @address:
 *
 * Called from control endpoint function after it decodes a set address setup packet.
 */
void
udc_set_address(unsigned char address)
{
	dbg_tx(2, " udc_set_address  %d \n", address);
	OTG_FUNC_DEV_ADDR = address;
}

/**
 * udc_serial_init - set a serial number if available
 */
int __init
udc_serial_init(struct usb_bus_instance *bus)
{
	return -EINVAL;
}

/* ********************************************************************************************* */

/**
 * udc_max_endpoints - max physical endpoints
 *
 * Return number of physical endpoints.
 */
int
udc_max_endpoints(void)
{
	dbg_tx(3, " udc_max_endpoints = UDC_MAX_ENDPOINTS \n");
	return UDC_MAX_ENDPOINTS;
}

/**
 * udc_check_ep - check logical endpoint
 * @lep:
 *
 * Return physical endpoint number to use for this logical endpoint or zero if not valid.
 */
int
udc_check_ep(int logical_endpoint, int packetsize)
{

	return (((logical_endpoint & 0xf) >= UDC_MAX_ENDPOINTS)
		|| (packetsize > 64)) ? 0 : (logical_endpoint & 0xf);
}

/**
 * udc_set_ep - setup endpoint
 * @ep:
 * @endpoint:
 *
 * Associate a physical endpoint with endpoint_instance
 */
void
udc_setup_ep(struct usb_device_instance *device, unsigned int ep,
	     struct usb_endpoint_instance *endpoint)
{
	dbg_init(3, " udc_setup_ep: ep=%d \n", ep);
	if (ep < UDC_MAX_ENDPOINTS) {

		ep_endpoints[ep] = endpoint;

		if (ep == 0) {
		} else if (ep == 1) {
			EPConfig(EP1, EP_IN_DIR, 0, 0, 32, BLK_TYPE, 0x100,
				 0x200, BYTECOUNT, BYTECOUNT);
		} else if (ep == 2) {

			dbg_init(3, " usbd_fill_rcv : ep=%d \n", ep);
			usbd_fill_rcv(device, endpoint, storage_flag ? 22 : 5);
			endpoint->rcv_urb = first_urb_detached(&endpoint->rdy);

			OTG_FUNC_IINT |= EP2OUT;

			EPConfig(EP2, EP_OUT_DIR, EP_NO_STALL, EP_NO_SETUP, 32,
				 BLK_TYPE, 0, 0, BYTECOUNT, BYTECOUNT);

			if (storage_flag) {
				unsigned char *cp =
				    endpoint->rcv_urb->buffer +
				    endpoint->rcv_urb->actual_length;

				OTG_DMA_EP2_O_MSA = (unsigned long) cp;
				OTG_DMA_EP_EN |= EP2OUT;

			}

			Ready_EP_OUT(2);
		}
	}
}

/**
 * udc_disable_ep - disable endpoint
 * @ep:
 *
 * Disable specified endpoint
 */
void
udc_disable_ep(unsigned int ep)
{
	dbg_tx(3, " udc_disable_ep \n");
	if (ep < UDC_MAX_ENDPOINTS) {
		struct usb_endpoint_instance *endpoint;

		if ((endpoint = ep_endpoints[ep])) {
			ep_endpoints[ep] = NULL;
			usbd_flush_ep(endpoint);
		}
	}
}

/* ********************************************************************************************* */

/**
 * udc_connected - is the USB cable connected
 *
 * Return non-zeron if cable is connected.
 */
int
udc_connected()
{
	dbg_tx(3, " udc_connected \n");
	return 1;
}

/**
 * udc_connect - enable pullup resistor
 *
 * Turn on the USB connection by enabling the pullup resistor.
 */
void
udc_connect(void)
{
	/*pull-up resistor is taken care of in mx2-usbotg*/
	dbg_tx(3, " udc_connect \n");
}

/**
 * udc_disconnect - disable pullup resistor
 *
 * Turn off the USB connection by disabling the pullup resistor.
 */
void
udc_disconnect(void)
{
	/*pull-up resistor is taken care of in mx2-usbotg*/
	dbg_tx(3, " udc_disconnect \n");
}

/* ********************************************************************************************* */

/**
 * udc_enable_interrupts - enable interrupts
 *
 * Switch on UDC interrupts.
 *
 */
void
udc_all_interrupts(struct usb_device_instance *device)
{
	dbg_tx(3, "udc_all_interrupts enable \n\n\n");

	if (!mx2udc_otg_enabled) return;

	OTG_FUNC_EP_DEN = EP0OUT | EP2OUT;
	OTG_FUNC_SINT_STAT = BIT3 | BIT2 | BIT0;
	OTG_FUNC_SINT_STEN = BIT3 | BIT2 | BIT0;

	dbg_tx(3, "udc_all_interrupts ok \n\n\n");
}

/**
 * udc_suspended_interrupts - enable suspended interrupts
 *
 * Switch on only UDC resume interrupt.
 *
 */
void
udc_suspended_interrupts(struct usb_device_instance *device)
{
	dbg_tx(3, "udc_suspended_interrupts enable \n");

}

/**
 * udc_disable_interrupts - disable interrupts.
 *
 * switch off interrupts
 */
void
udc_disable_interrupts(struct usb_device_instance *device)
{
	OTG_FUNC_SINT_STEN = 0;
}

/* ********************************************************************************************* */

/**
 * udc_ep0_packetsize - return ep0 packetsize
 */
int
udc_ep0_packetsize(void)
{
	return EP0_PACKETSIZE;
}

/**
 * udc_enable - enable the UDC
 *
 * Switch on the UDC
 */
void
udc_enable(struct usb_device_instance *device)
{

	udc_device = device;

	if (!ep0_urb) {
		if (!
		    (ep0_urb =
		     usbd_alloc_urb(device, device->function_instance_array, 0,
				    512))) {
			printk (KERN_ERR "udc_enable: usbd_alloc_urb failed\n");
		}
	} else {
		printk (KERN_ERR "udc_enable: ep0_urb already allocated\n");
	}

}

/**
 * udc_disable - disable the UDC
 *
 * Switch off the UDC
 */
void
udc_disable(void)
{
	dbg_tx(3, " udc_disable \n");

	if (ep0_urb) {
		usbd_dealloc_urb(ep0_urb);
		ep0_urb = NULL;
	}
}

/**
 * udc_startup - allow udc code to do any additional startup
 */
void
udc_startup_events(struct usb_device_instance *device)
{
	struct usb_function_instance *function;
	struct usb_configuration_description *configuration;
	struct usb_interface_description *interface;
	int i, j;

	dbg_usbe(3, " udc_startup_events:device->name=%s \n", device->name);

	function = device->function_instance_array;

	for (i = 0; i < function->function_driver->configurations; i++) {
		configuration =
		    function->function_driver->configuration_description + i;
		for (j = 0; j < configuration->interfaces; j++) {
			interface = configuration->interface_list + j;
			if (interface->bInterfaceClass ==
			    USB_CLASS_MASS_STORAGE) {
				storage_flag = 1;
			}
		}
	}

	usbd_device_event(device, DEVICE_INIT, 0);
	usbd_device_event(device, DEVICE_CREATE, 0);
	usbd_device_event(device, DEVICE_HUB_CONFIGURED, 0);
	usbd_device_event(device, DEVICE_RESET, 0);
}

/* ********************************************************************************************* */

static void mx2udc_disable(void)
{
	OTG_FUNC_SINT_STEN = 0;
	mx2udc_otg_enabled = 0;
}

static int mx2udc_enable(void)
{
	mx2udc_otg_enabled = 1;
	FunctionSetup();
	return 0;
}

static struct mx2otg_device_descr_st mx2udc_otg_descriptor = {
	.enable = &mx2udc_enable,
	.disable = &mx2udc_disable,
};

/**
 * udc_name - return name of USB Device Controller
 */
char *
udc_name(void)
{
	return UDC_NAME;
}

/**
 * udc_request_udc_irq - request UDC interrupt
 *
 * Return non-zero if not successful.
 */
int
udc_request_udc_irq()
{

	if (request_irq
	    (INT_USBFUNC, mx2_interrupt, SA_INTERRUPT, "USB DEV", 0) < 0)
		printk("Intr request for source %d failed !\n", INT_USBFUNC);
	return 0;
}

/**
 * udc_request_cable_irq - request Cable interrupt
 *
 * Return non-zero if not successful.
 */
int
udc_request_cable_irq()
{
	return 0;
}

/**
 * udc_request_udc_io - request UDC io region
 *
 * Return non-zero if not successful.
 */
int
udc_request_io()
{
	/*connect to the usb-mx2otg here*/
	return mx2otg_register_device(&mx2udc_otg_descriptor);
}

/**
 * udc_release_udc_irq - release UDC irq
 */
void
udc_release_udc_irq()
{
	free_irq(INT_USBFUNC, 0);
}

/**
 * udc_release_cable_irq - release Cable irq
 */
void
udc_release_cable_irq()
{
}

/**
 * udc_release_release_io - release UDC io region
 */
void
udc_release_io()
{
	/*disconnect from the usb-mx2otg here*/
	mx2otg_unregister_device(&mx2udc_otg_descriptor);
}

/**
 * udc_regs - dump registers
 *
 * Dump registers with printk
 */
void
udc_regs(void)
{
	printk("OTG_FUNC_CND_STAT=%x\n", OTG_FUNC_CND_STAT);
	printk("OTG_FUNC_DEV_ADDR=%x\n", OTG_FUNC_DEV_ADDR);
	printk("OTG_FUNC_SINT_STAT=%x\n", OTG_FUNC_SINT_STAT);
	printk("OTG_FUNC_SINT_STEN=%x\n", OTG_FUNC_SINT_STEN);
	printk("OTG_FUNC_XINT_STAT=%x\n", OTG_FUNC_XINT_STAT);
	printk("OTG_FUNC_YINT_STAT=%x\n", OTG_FUNC_YINT_STAT);
	printk("OTG_FUNC_XYINT_STEN=%x\n", OTG_FUNC_XYINT_STEN);
	printk("OTG_FUNC_XFILL_STAT=%x\n", OTG_FUNC_XFILL_STAT);
	printk("OTG_FUNC_YFILL_STAT=%x\n", OTG_FUNC_YFILL_STAT);

	printk("OTG_FUNC_EP_RDY=%x\n", OTG_FUNC_EP_RDY);
	printk("OTG_FUNC_EP_EN=%x\n", OTG_FUNC_EP_EN);
	printk("OTG_FUNC_IINT=%x\n", OTG_FUNC_IINT);
	printk("OTG_FUNC_EP_DSTAT=%x\n", OTG_FUNC_EP_DSTAT);
	printk("OTG_FUNC_EP_DEN=%x\n", OTG_FUNC_EP_DEN);
	printk("OTG_FUNC_EP_TOGGLE=%x\n", OTG_FUNC_EP_TOGGLE);
	printk("OTG_FUNC_FRM_NUM=%x\n", OTG_FUNC_FRM_NUM);
}
