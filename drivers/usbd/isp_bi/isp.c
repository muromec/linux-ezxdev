/*
 * usbd/isp_bi/isp.c -- ISP1583 USB Device Controller driver.
 *
 *      Copyright (c) 2005 Motorola
 *
 * By: 
 *      Richard Xiao <a2590c@motorola.com>, 
 *      Zhiming Yuan <a14194@motorola.com>, 
 *      Xin     Li   <a16157@motorola.com>, 
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
 */
/* Revision history:
 * 
 * 2005-July-12		created for USB2.0, Li xin
 * 2005-August-20	modify for nfs, Yuan Zhiming
 * 2005-Oct-18		modify for acm, Xiao Richard
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
#include <linux/pci.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/types.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <asm/hardware.h>
#include <asm/dma.h>

#include <linux/ezxusbd.h>

#include <usbd-export.h>
#include <usbd-build.h>
#include <usbd-chap9.h>
#include <usbd-mem.h>
#include <usbd.h>
#include <usbd-func.h>
#include <usbd-bus.h>
#include "trace.h"
#include <usbd-bi.h>
#include "isp.h"

MODULE_DESCRIPTION ("ISP1583 USB Device Bus Interface");
MODULE_AUTHOR ("a2590c@motorola.com,a14194@motorola.com,a16157@motorola.com");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,17)
MODULE_LICENSE("GPL");
#endif
USBD_MODULE_INFO ("isp_bi 1.0-beta");

/*
 * define ISP_DMA_ENABLE to enable ISP1583 DMA data transfer
 */
#define ISP_DMA_ENABLE

/* some static function declaration */
static void isp_ep_in_int (int epn, int flag, INT_ENABLE_LSB *l, INT_ENABLE_MSB *m);
static void isp_ep_out_int (int epn, int flag, INT_ENABLE_LSB *l, INT_ENABLE_MSB *m);

/* NFS mode flag */
int usbd_in_nfs_mode(void);


/* 
 * global parameters declaration 
 */
unsigned int udc_interrupts = 0;

/* 
 * local parameters 
 */
static int udc_suspended;
static int udc_in_int;
static int high_full_state;
static int isp_requested_eps_flag = 0;      
static int isp_requested_irq_flag = 0;
static int isp_set_addressed = 0;
static int g_hs_test_packet = 0;
static int isp_set_ep_type = 0;
static int udc_connected_status;
static int isp_reenum_flag = 0;

/* the following parameters are used to map the endpoint between ISP chip and UDC */
/* IN endpoints */
static int isp_in_map_udc_ep[UDC_MAX_IN_ENDPOINTS] = { 0, 0, 0, 0, 0, 0, 0, 0};
/* OUT endpoints */
static int isp_out_map_udc_ep[UDC_MAX_OUT_ENDPOINTS] = { 0, 0, 0, 0, 0, 0, 0, 0};

/* ISP1583 register pointer */
volatile D14_CNTRL_REG  *isp_regs = 0; 

/* ISP1583 USB device address mask */
#define ISP_ADDR_MASK    0x7F

/*
 * the following parameters are used for DMA mode
 */
#ifdef ISP_DMA_ENABLE

static int isp_dma_channel = -1;

/* when DMA is used, isp_dma_flag is 1, otherwise 0 */
static int isp_dma_flag = 0;

/* save the current ep for DMA */
static int isp_dma_ep   = 0;

/* save the current DMA data transfer length */
static int isp_dma_len  = 0;
static unsigned long  isp_dma_index_len  = 0;

/* define the each DMA transfer byte number  */
#define MAX_DMA_SIZE  4096

/* local function definition */
static void isp_epx_rx_set_dma(unsigned long phys_addr);
static void isp_epx_tx_set_dma(unsigned long phys_addr);
static void isp_dev_dma_irq(void);

#endif   /* ISP_DMA_ENABLE */

/* #define ISP_DBG(x...) printk(x) */
#define ISP_DBG(x...) 

/* 
 * local definition
 */
#define LOOP_FOREVER     while(1);

/* 
 * because there is some problem if only one ISP endpoint index register operation is used,
 * the following macro is used for this operation  
 */
#define ISP_SET_ENDPOINT_INDEX(x);       { \
	isp_regs->D14_ENDPT_INDEX = x;     \
	isp_regs->D14_ENDPT_INDEX = x;     \
}

#define ISP_SET_DMA_ENDPOINT(x);       { \
	isp_regs->D14_DMA_ENDPOINT = x;     \
	isp_regs->D14_DMA_ENDPOINT = x;     \
}

/*
 * get the usbd module information
 */
const char *usbd_bi_module_info (void)
{ 
	return __usbd_module_info; 
}

/*
 * dump ISP1583 registers
 */
static void isp_display_regs (void)
{
	ISP_DBG("\r\nISP Regs:");

	ISP_DBG("ADDRESS:0x%x,",isp_regs->D14_ADDRESS.VALUE&0xFFFF);
	ISP_DBG("EDNPT_I:0x%x,",isp_regs->D14_ENDPT_INDEX&0xFFFF);
	ISP_DBG("EPT_CTL:0x%x,",isp_regs->D14_CONTROL_FUNCTION.VALUE&0xFFFF);
	ISP_DBG("DMA_EPT_I:0x%x,",isp_regs->D14_DMA_ENDPOINT&0xFFFF);
	ISP_DBG("MAXPT:0x%x,",isp_regs->D14_ENDPT_MAXPKTSIZE.VALUE&0xFFFF);
	ISP_DBG("TYPE:0x%x,",isp_regs->D14_ENDPT_TYPE.VALUE&0xFFFF);
	ISP_DBG("MODE:0x%x,",isp_regs->D14_MODE.VALUE&0xFFFF);
	ISP_DBG("INTCFG:0x%x,",isp_regs->D14_INT_CONFIG.VALUE&0xFFFF);
	ISP_DBG("INTEN_L:0x%x,",isp_regs->D14_INT_ENABLE_LSB.VALUE&0xFFFF);
	ISP_DBG("INTEN_M:0x%x,",isp_regs->D14_INT_ENABLE_MSB.VALUE&0xFFFF);
	ISP_DBG("INT_L:0x%x,",isp_regs->D14_INT_LSB.VALUE&0xFFFF);
	ISP_DBG("INT_M:0x%x",isp_regs->D14_INT_MSB.VALUE&0xFFFF);
	ISP_DBG("BUF_ST:0x%x",isp_regs->D14_BUFFER_STATUS&0xFFFF);
	ISP_DBG("DMA_CFG:0x%x",isp_regs->D14_DMA_CONFIG.VALUE&0xFFFF);
	ISP_DBG("DMA_HW:0x%x",isp_regs->D14_DMA_HARDWARE.VALUE&0xFFFF);
	ISP_DBG("DMA_INT:0x%x",isp_regs->D14_DMA_INT.VALUE&0xFFFF);
	ISP_DBG("DMA_INT_EN:0x%x",isp_regs->D14_DMA_INT_ENABLE.VALUE&0xFFFF);
	ISP_DBG("DMA_CNT_LSB:0x%x",isp_regs->D14_DMA_TRANSFER_COUNTER_LSB&0xFFFF);
	ISP_DBG("DMA_CNT_MSB:0x%x",isp_regs->D14_DMA_TRANSFER_COUNTER_MSB&0xFFFF);

	ISP_DBG("\r\n");
}

/*
 * ISP1583 set endpoint IN type
 */
static void isp_set_epx_in (int epn)
{
	struct usb_endpoint_instance *endpoint;
	u16  ep_type = ISP_EPTYPE_BULK;  /* bulk */
	
	/* The input "epn" is the ISP IN endpoint index, the isp_in_map_udc_ep[] can be used to 
	    get the endpoint structure in UDC layer */
	endpoint = usbd_bus->endpoint_array + isp_in_map_udc_ep[epn];

	/* 
	 * the following is used to clear the endpoint IN buffer
	 * please refer to philip AN10045
	 */
	ISP_SET_ENDPOINT_INDEX((epn << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);
	isp_regs->D14_CONTROL_FUNCTION.VALUE = ISP_CTLFUN_VENDP;

	ISP_SET_ENDPOINT_INDEX((epn << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);
	isp_regs->D14_CONTROL_FUNCTION.VALUE = ISP_CTLFUN_VENDP;

	isp_regs->D14_ENDPT_MAXPKTSIZE.VALUE = endpoint->wMaxPacketSize;

	switch(endpoint->bmAttributes & USB_ENDPOINT_MASK) {
        case USB_ENDPOINT_BULK:
		ep_type = ISP_EPTYPE_BULK;
		break;
	case USB_ENDPOINT_INTERRUPT:
		ep_type = ISP_EPTYPE_INTERRUPT;
		break;
	default:
		break;
	}
	isp_regs->D14_ENDPT_TYPE.VALUE = ep_type ;
}

/*
 * ISP1583 set endpoint OUT type
 */
static void isp_set_epx_out (int epn)
{
	struct usb_endpoint_instance * endpoint;
	u16  ep_type = ISP_EPTYPE_BULK;  /* bulk */


	/* The input "epn" is the ISP OUT endpoint index, the isp_out_map_udc_ep[] can be used to 
	    get the endpoint structure in UDC layer */

	endpoint = usbd_bus->endpoint_array + isp_out_map_udc_ep[epn];

	ISP_SET_ENDPOINT_INDEX(epn << ISP_EPINDEX_EP_OFFSET);
	isp_regs->D14_ENDPT_MAXPKTSIZE.VALUE = endpoint->wMaxPacketSize;

	switch(endpoint->bmAttributes & USB_ENDPOINT_MASK) {
	case USB_ENDPOINT_BULK:
		ep_type = ISP_EPTYPE_BULK;
		break;
	case USB_ENDPOINT_INTERRUPT:
		ep_type = ISP_EPTYPE_INTERRUPT;
		break;
	default:
		break;
	}
	isp_regs->D14_ENDPT_TYPE.VALUE = ep_type ;
}

#ifdef ISP_DMA_ENABLE
                                                                                                  
/* 
 * PXA27x DMA channel interrupt handler 
 */
static void isp_pxa_dma_irq(int dmach, void* dev_id, struct pt_regs *regs)
{
	ISP_DBG("\r\nISP: PAX DMA interrupt handler\r\n");
}
                                                                                                  
/* 
 * PXA27x DMA channel initialization 
 */
static int isp_pxa_dma_init(void)
{
	ISP_DBG("\r\nISP: PAX dma init\r\n");

	/* allocate DMA channel */
	isp_dma_channel = pxa_request_dma ("ISP1583 DMA", DMA_PRIO_LOW, isp_pxa_dma_irq, NULL);
	if (isp_dma_channel < 0) {
		return 1;
	}

	ISP_DBG("\r\nISP: isp_dma_channel = %d\r\n",isp_dma_channel);
	return 0;
}

/* 
 * the following function is used to free PXA27x DMA channel 
 */                                                    
static void isp_pxa_dma_exit(void)
{
	ISP_DBG("\r\nISP: PAX dma exit\r\n");
	
	/* free DMA channel */
	if (isp_dma_channel >= 0)    {
		pxa_free_dma (isp_dma_channel);
	}
}

#endif /* ISP_DMA_ENABLE */

/*
 * ISP1583 dma registers initialization 
 */
static void isp_dev_dma_init (void)
{
	DMA_CONFIG cfg;
	DMA_HARDWARE dma_hw;
	DMA_INT_ENABLE dma_int;

	ISP_DBG("\r\nISP: dma init\r\n");

	/* reset DMA */
	isp_regs->D14_DMA_COMMAND = ISP_DMA_RESET;   

	/* initialize GDMA DMA mode */
	cfg.VALUE = isp_regs->D14_DMA_CONFIG.VALUE;    
	cfg.BITS.MODE = 0;
	cfg.BITS.DIS_XFER_CNT = 0;
	cfg.BITS.WIDTH = 1;
	cfg.BITS.IGNORE_IORDY = 0;
	cfg.BITS.PIO_MODE = 0;
	cfg.BITS.DMA_MODE = 0;
	isp_regs->D14_DMA_CONFIG.VALUE = cfg.VALUE;

	dma_hw.VALUE = isp_regs->D14_DMA_HARDWARE.VALUE;
	dma_hw.BITS.ENDIAN = 0;
	dma_hw.BITS.EOT_POL = 0;
	dma_hw.BITS.ACK_POL = 0;
	dma_hw.BITS.DREQ_POL = 1;
	dma_hw.BITS.WRITE_POL = 0;
	dma_hw.BITS.READ_POL = 0;
	isp_regs->D14_DMA_HARDWARE.VALUE = dma_hw.VALUE;
   
	/* enable DMA interrupt */
	dma_int.VALUE = 0;
	dma_int.BITS.IE_DMA_XFER_OK = 1;
	isp_regs->D14_DMA_INT_ENABLE.VALUE = dma_int.VALUE;

	/* set burst counter */
	isp_regs->D14_DMA_BURST_COUNTER = ISP_DMA_BURST_LEN;

#ifdef ISP_DMA_ENABLE
	isp_dma_flag  = 0;
#endif /* ISP_DMA_ENABLE */
}

#ifdef ISP_DMA_ENABLE
/*
 * ISP1583 DMA interrupt handler
 */
static void isp_dev_dma_irq(void)
{
	struct usb_endpoint_instance *endpoint;
	u8    ep = isp_dma_ep&(~USB_DIR_IN);
	CONTROL_REG ctrl;
	INT_ENABLE_LSB l; INT_ENABLE_MSB m;
	u8 *src,*tgt; u16 i;
                                                                                                  
	ISP_DBG("\r\nISP: dma irq\r\n");
                                                                                                  
	if((!isp_dma_flag) || (ep == 0))  return;
                                                                                                  
	/* clear DMA flag */
	isp_dma_flag = 0;

	/* TX interrupt */
	if (USB_DIR_IN & isp_dma_ep)   {
		endpoint = usbd_bus->endpoint_array + isp_in_map_udc_ep[ep];

		ISP_DBG("\r\nDMA IN endpoint interrupt ep=%d, len=%d\r\n",ep,isp_dma_len);
		if (isp_dma_index_len >= endpoint->tx_urb->actual_length)  {
			/* all data have been transmitted by DMA */
			if(endpoint->tx_urb->actual_length % endpoint->wMaxPacketSize)  {
				/* set dma endpoint into a random value which is different with endpoint index */
				isp_regs->D14_DMA_ENDPOINT = (ep << 1) + 5;

				ISP_SET_ENDPOINT_INDEX((ep << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);
				ctrl.VALUE = 0;
				ctrl.BITS.VENDP = 1;
				isp_regs->D14_CONTROL_FUNCTION.VALUE = ctrl.VALUE;
			}
			endpoint->last = endpoint->tx_urb->actual_length;
			bi_tx_complete_irq(endpoint, 0);
	
			if (!isp_dma_flag)
			{
				/* set dma endpoint into a random value which is different with endpoint index */
				isp_regs->D14_DMA_ENDPOINT = (ep << 1) + 5;

				/* enable endpoint interrupt */
				l.VALUE = isp_regs->D14_INT_ENABLE_LSB.VALUE;
				m.VALUE = isp_regs->D14_INT_ENABLE_MSB.VALUE;
				isp_ep_in_int (ep, 1, &l, &m);
				isp_regs->D14_INT_ENABLE_LSB.VALUE = l.VALUE;
				isp_regs->D14_INT_ENABLE_MSB.VALUE = m.VALUE;
			}
			return;
		}

		/* re-set the DMA to transfer the other data */
		isp_dma_len = endpoint->tx_urb->actual_length - isp_dma_index_len;
		if (isp_dma_len > MAX_DMA_SIZE)
			isp_dma_len = MAX_DMA_SIZE;
		unsigned char * buffer;
		dma_addr_t dma_addr;
		buffer = endpoint->tx_urb->buffer + isp_dma_index_len;
		dma_addr = virt_to_bus(buffer);
		isp_dma_index_len += isp_dma_len;
		consistent_sync(endpoint->tx_urb->buffer, MAX_DMA_SIZE, PCI_DMA_TODEVICE);
		/* enable RX dma transfer */
		isp_epx_tx_set_dma(dma_addr);
	}
	else     {   /* RX interrupt */
		endpoint = usbd_bus->endpoint_array + isp_out_map_udc_ep[ep];

		/* tell UDC framework, the data has been received */
		ISP_DBG("\r\nDMA OUT endpoint interrupt ep=%d, len=%d\r\n",isp_dma_ep,isp_dma_len);

		/* if all wished data have been received by DMA */
		if(isp_dma_index_len >= endpoint->rcv_urb->request_length)  {
			bi_rcv_complete_irq(endpoint, endpoint->rcv_urb->request_length, 0);

			if (!isp_dma_flag)
			{
				/* set dma endpoint into a random value which is different with endpoint index */
				isp_regs->D14_DMA_ENDPOINT = (ep << 1) + 5;

				/* enable endpoint interrupt */
				l.VALUE = isp_regs->D14_INT_ENABLE_LSB.VALUE;
				m.VALUE = isp_regs->D14_INT_ENABLE_MSB.VALUE;
				isp_ep_out_int (ep, 1, &l, &m);
				isp_regs->D14_INT_ENABLE_LSB.VALUE = l.VALUE;
				isp_regs->D14_INT_ENABLE_MSB.VALUE = m.VALUE;
			}
			return;
		}

		/* re-set the DMA channel to receive other data */
		isp_dma_len = endpoint->rcv_urb->request_length - isp_dma_index_len;
		if(isp_dma_len > MAX_DMA_SIZE)
			isp_dma_len = MAX_DMA_SIZE;
		dma_addr_t dma_addr = virt_to_bus(endpoint->rcv_urb->buffer + isp_dma_index_len);
		isp_dma_index_len += isp_dma_len;
		consistent_sync(endpoint->rcv_urb->buffer, MAX_DMA_SIZE, PCI_DMA_FROMDEVICE);
		/* enable RX dma transfer */
		isp_epx_rx_set_dma(dma_addr);
	}
}

/* the following registers may be used for DMA initialization */                                                             
#define DRQSR0 __REG(0x400000E0)
#define DALGN  __REG(0x400000A0)

/*
 * configure the DMA channel to receive data
 */
static void isp_epx_rx_set_dma(unsigned long phys_addr)
{
	ISP_DBG("\r\nISP: isp_epx_rx_set_dma ep=%d, len=%d, phys_addr=0x%x\r\n",isp_dma_ep,isp_dma_len,phys_addr);
                                                                                                  
	/* configure PXA DMA registers */
	DALGN  |= (1 << isp_dma_channel);
	DRCMR0 = 0x80 | isp_dma_channel;
	DCSR(isp_dma_channel) = DCSR_NODESC;
	DSADR(isp_dma_channel) = (ISP_REGBASE_ADDR + ISP_DMA_DATA_PORT_OFFSET);
	DTADR(isp_dma_channel) = phys_addr;
	DCMD(isp_dma_channel) = DCMD_INCTRGADDR | DCMD_FLOWSRC | DCMD_BURST32 | isp_dma_len;
	DCSR(isp_dma_channel) |= DCSR_RUN | DCSR_NODESC;
                                                                                                  
	/* configure ISP DMA registers */
	isp_regs->D14_ENDPT_INDEX =  (isp_dma_ep << 1) + 3;
	ISP_SET_DMA_ENDPOINT(isp_dma_ep << ISP_EPINDEX_EP_OFFSET);

	/* initialize DMA transfer counter */
	isp_regs->D14_DMA_TRANSFER_COUNTER_LSB = (u16)(isp_dma_len & 0xFFFF);
	isp_regs->D14_DMA_TRANSFER_COUNTER_MSB = (u16)((isp_dma_len >> 16) & 0xFFFF);

	/* enable ISP1583 DMA write */
	isp_regs->D14_DMA_COMMAND = ISP_GDMA_Write_Command;

	/* set flag to indicate the DMA channel has been occupied */
	isp_dma_flag = 1;
}

/*
 * the following function is used for debug
 */
static void display_pxa_dma_regs(void)
{
	ISP_DBG("\r\nDRCMR0:0x%x,", DRCMR0);
	ISP_DBG("DCSR:0x%x,", DCSR(isp_dma_channel));
	ISP_DBG("DSADR:0x%x,", DSADR(isp_dma_channel));
	ISP_DBG("DTADR:0x%x,", DTADR(isp_dma_channel));
	ISP_DBG("DCMD:0x%x,", DCMD(isp_dma_channel));
	ISP_DBG("DALGN:0x%x,", DALGN);
	ISP_DBG("DRQSR0:x%x\r\n", DRQSR0);
}

/*
 * configure the DMA transmit
 */                                                                                          
static void isp_epx_tx_set_dma(unsigned long phys_addr)
{
	u8 ep = isp_dma_ep&(~USB_DIR_IN);
                                                                                                  
	ISP_DBG("\r\nISP: isp_epx_tx_set_dma, ep=%d,len=%d,dma_ch=%d,phys_addr=0x%x\r\n",ep,isp_dma_len,isp_dma_channel,phys_addr);
                                                                                                  

	/* configure PXA DMA registers */
	DALGN  |= (1 << isp_dma_channel);
	DRCMR0 = 0x80 | isp_dma_channel;
	DCSR(isp_dma_channel) = DCSR_NODESC;
	DSADR(isp_dma_channel) = phys_addr;
	DTADR(isp_dma_channel) = (ISP_REGBASE_ADDR + ISP_DMA_DATA_PORT_OFFSET);
	DCMD(isp_dma_channel) = DCMD_INCSRCADDR | DCMD_FLOWTRG | DCMD_BURST32 | isp_dma_len;
	DCSR(isp_dma_channel) |= DCSR_RUN | DCSR_NODESC;
                                                                                                  
	/* configure ISP DMA registers */
	isp_regs->D14_ENDPT_INDEX =  ep + 5;
	ISP_SET_DMA_ENDPOINT((ep << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);

	/* initialize DMA transfer counter */
	isp_regs->D14_DMA_TRANSFER_COUNTER_LSB = (u16)(isp_dma_len & 0xFFFF);
	isp_regs->D14_DMA_TRANSFER_COUNTER_MSB = (u16)((isp_dma_len >> 16) & 0xFFFF);
	isp_regs->D14_DMA_COMMAND = ISP_GDMA_Read_Command;
                                                                                                  
	/* set flag to indicate the DMA channel has been occupied */
	isp_dma_flag = 1;
}
                                                                                                  
#endif /* ISP_DMA_ENABLE */

/* 
 * this function is used to enable/disable IN endpoint interrrupt 
 */
static void isp_ep_in_int (int epn, int flag, INT_ENABLE_LSB *l, INT_ENABLE_MSB *m)
{
	ISP_DBG("\r\nISP: isp_ep_in_int epn=%d, flag=%d\r\n",epn,flag);

	switch (epn) {
	case 0:
		l->BITS.IEP0TX = flag;
		break;
	case 1:
		l->BITS.IEP1TX = flag;
		break;
	case 2:
		l->BITS.IEP2TX = flag;
		break;
	case 3:
		m->BITS.IEP3TX = flag;
		break;
	case 4:
		m->BITS.IEP4TX = flag;
		break;
	case 5:
		m->BITS.IEP5TX = flag;
		break;
	case 6:
		m->BITS.IEP6TX = flag;
		break;
	case 7:
		m->BITS.IEP7TX = flag;
	default:
		break;
	}
}

/* 
 * this function is used to enable/disable OUT endpoint interrrupt 
 */
static void isp_ep_out_int (int epn, int flag, INT_ENABLE_LSB *l, INT_ENABLE_MSB *m)
{
	ISP_DBG("\r\nISP: isp_ep_out_int epn=%d, flag=%d\r\n",epn,flag);

	switch (epn) {
	case 0:
		l->BITS.IEP0RX = flag;
		break;
	case 1:
		l->BITS.IEP1RX = flag;
		break;
	case 2:
		l->BITS.IEP2RX = flag;
		break;
	case 3:
		m->BITS.IEP3RX = flag;
		break;
	case 4:
		m->BITS.IEP4RX = flag;
		break;
	case 5:
		m->BITS.IEP5RX = flag;
		break;
	case 6:
		m->BITS.IEP6RX = flag;
		break;
	case 7:
		m->BITS.IEP7RX = flag;
	default:
		break;
	}
}

/* 
 * ISP1583 Initialization 
 */
static void isp_init (void)
{
	USB_MODE m;
	INT_CONFIG i;

	ISP_DBG("\r\nISP: init\r\n");

	/* unlock the ISP158x register */
	isp_regs->D14_UNLOCK_DEVICE = ISP_UNLOCK_REGS;

	/* usb mode */
	m.VALUE = 0;
	m.BITS.DMACLKON = 1;
	m.BITS.CLKAON   = 1;
	m.BITS.GLINTENA = 1;
	m.BITS.SOFTCT   = 1;
	isp_regs->D14_MODE.VALUE   = m.VALUE;

	/* interrupt configure */
	i.VALUE = 0;
	i.BITS.DDBGMODOUT = 1;
	i.BITS.DDBGMODIN  = 1;
	i.BITS.CDBGMOD    = 1;
	isp_regs->D14_INT_CONFIG.VALUE = i.VALUE;

	/* init dma endpoint index */
	/* 0xe0 is only a random value which is used to initialize DMA endpoint */
	isp_regs->D14_DMA_ENDPOINT = 0xe0;
	
#ifdef ISP_DMA_ENABLE
	isp_dev_dma_init();
#endif /* ISP_DMA_ENABLE */

}

/* 
 * ISP1583 OUT endpoint interrupt handler 
 */
static void isp_epx_out (u8 ep)
{
	/* The input "ep" is the ISP OUT endpoint index, the isp_out_map_udc_ep[] can be used to 
	    get the endpoint structure in UDC layer */

	struct usb_endpoint_instance *endpoint = usbd_bus->endpoint_array + isp_out_map_udc_ep[ep];
	struct urb *rcv_urb;
	int len = 0, size, i, j, num;
	u8 *cp;
	u16 temp_i = 0;
	INT_ENABLE_LSB l; 
	INT_ENABLE_MSB m;
    
	/* select the setup endpoint and read in the device request */
	ISP_SET_ENDPOINT_INDEX(ep << ISP_EPINDEX_EP_OFFSET);

	ISP_DBG("\r\nISP: epx_out %d, %d\r\n",ep, isp_regs->D14_BUFFER_STATUS);

	if ((i = isp_regs->D14_BUFFER_STATUS) != 0) {
		if(i == 3) 
			num = 2;
		else
			num = 1;
		for (j = 0; j < num; j ++) {

			/* the following endpoint index setting is need, otherwise the data access error */
			ISP_SET_ENDPOINT_INDEX(ep << ISP_EPINDEX_EP_OFFSET);

			size = (isp_regs->D14_BUFFER_LENGTH) & 0xFFFF;
			rcv_urb = bi_rcv_next_irq (endpoint);

	 		/* does not have an available urb for this endpoint */
			if (!rcv_urb) {    
				l.VALUE = isp_regs->D14_INT_ENABLE_LSB.VALUE;
				m.VALUE = isp_regs->D14_INT_ENABLE_MSB.VALUE;

				isp_ep_out_int (ep, 0, &l, &m);
				isp_regs->D14_INT_ENABLE_LSB.VALUE = l.VALUE;
				isp_regs->D14_INT_ENABLE_MSB.VALUE = m.VALUE;
				ISP_DBG("\r\nrcv_urb is NULL\r\n");
				return;
			}

			if (rcv_urb->dma_mode)  {
				rcv_urb->dma_mode = 0;
			}
			cp = endpoint->rcv_urb->buffer + endpoint->rcv_urb->actual_length;
			for(len = 0; (len < size) && (len < endpoint->wMaxPacketSize); len ++, cp++) {
				if((len % 2) == 0) {
					temp_i = isp_regs->D14_DATA_PORT; 
					*cp = (u8)(temp_i & 0xFF); 
				} else {
					*cp = (u8)((temp_i & 0xFF00) >> 8); 
				}
			}
			rcv_urb =  bi_rcv_complete_irq (endpoint, len, 0); 
		}
	}
}

/* 
 * ISP1583 OUT endpoint interrupt handler 
 */
static void isp_ep0_out (u8 ep)
{
	/* The input "ep" is the ISP OUT endpoint index, the isp_out_map_udc_ep[] can be used to 
	    get the endpoint structure in UDC layer */

	struct usb_endpoint_instance *endpoint = usbd_bus->endpoint_array + isp_out_map_udc_ep[ep];
	struct urb *rcv_urb = bi_rcv_next_irq (endpoint);
	int len = 0, size, i, j, num;
	u8 *cp;
	u16 temp_i = 0;
	CONTROL_REG ctrl;
	INT_ENABLE_LSB l; 
	INT_ENABLE_MSB m;
        int recv_all_flag = 0;
 
	ISP_DBG("\r\nISP: ep0_out %d\r\n",ep);
    
	/* select the setup endpoint and read in the device request */
	ISP_SET_ENDPOINT_INDEX(ep << ISP_EPINDEX_EP_OFFSET);

	if ( isp_regs->D14_BUFFER_STATUS == 0) {
		/* set IN 0 ACK */
		ISP_SET_ENDPOINT_INDEX((0 << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);

		ctrl.VALUE = 0;
		ctrl.BITS.STATUS = 1;
		isp_regs->D14_CONTROL_FUNCTION.VALUE = ctrl.VALUE;
		return;
	}

	if ((i = isp_regs->D14_BUFFER_STATUS) != 0) {
		if(i == ISP_BUFFER_FULL)
			num = 2;     /* two buffers are available */
		else
			num = 1;     /* only one buffer is available */
		for (j = 0; j < num; j++) {
			size = (isp_regs->D14_BUFFER_LENGTH) & 0xFFFF;
			/* does not have an available urb for this endpoint */
		        if (!rcv_urb) {    
				l.VALUE = isp_regs->D14_INT_ENABLE_LSB.VALUE;
				m.VALUE = isp_regs->D14_INT_ENABLE_MSB.VALUE;

				isp_ep_out_int (ep, 0, &l, &m);
				isp_regs->D14_INT_ENABLE_LSB.VALUE = l.VALUE;
				isp_regs->D14_INT_ENABLE_MSB.VALUE = m.VALUE;
				return;
			}

			cp = endpoint->rcv_urb->buffer + endpoint->rcv_urb->actual_length;
			for(len = 0; (len < size) && (len < endpoint->wMaxPacketSize); len ++, cp++) {
				if((len % 2) == 0) {
					temp_i = isp_regs->D14_DATA_PORT; 
					*cp = (u8)(temp_i & 0xFF); 
				} else {
					*cp = (u8)((temp_i & 0xFF00) >> 8); 
				}
			}

			/* detect whether all requested data have been received */
			if((size == endpoint->wMaxPacketSize) && ((endpoint->rcv_urb->actual_length + size) >= endpoint->rcv_urb->request_length) )
			{
				/* set flag for preparing IN 0 ACK */
				recv_all_flag = 1;
			}

			rcv_urb =  bi_rcv_complete_irq (endpoint, len, 0); 

			if((size < endpoint->wMaxPacketSize) || (recv_all_flag))  {
				/* set IN 0 ACK */
	        		ISP_SET_ENDPOINT_INDEX((0 << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);

				ctrl.VALUE = 0;
				ctrl.BITS.STATUS = 1;
				isp_regs->D14_CONTROL_FUNCTION.VALUE = ctrl.VALUE;
				return;
			}
		}
	}
}

/* 
 * ISP1583 IN endpoint 0 interrupt handler 
 */
static void isp_ep0_in (u8 ep)
{
	/* The input "ep" is the ISP IN endpoint index, the isp_in_map_udc_ep[] can be used to 
	    get the endpoint structure in UDC layer */

	struct usb_endpoint_instance *endpoint = usbd_bus->endpoint_array + isp_out_map_udc_ep[ep];
	struct urb *tx_urb;
	int last, size;
	u8 *cp;
	u16 temp_i;
	INT_ENABLE_LSB l; 
	INT_ENABLE_MSB m;
	CONTROL_REG ctrl;
    
	ISP_DBG("\r\nISP: ep0_in %d\r\n", ep);

	isp_set_addressed = 0;

	if (!(tx_urb = bi_tx_next_irq(endpoint))) {
		/* nothing to send, OUT endpoint */
       		ISP_SET_ENDPOINT_INDEX(0 << ISP_EPINDEX_EP_OFFSET);

		ctrl.VALUE = 0;
		ctrl.BITS.STATUS = 1;
		isp_regs->D14_CONTROL_FUNCTION.VALUE = ctrl.VALUE;
		return;
	}
	
	/* if last is 0, that means a zero packet shall be sent out */
	last = MIN (tx_urb->actual_length - (endpoint->sent + endpoint->last), endpoint->wMaxPacketSize);

	ISP_SET_ENDPOINT_INDEX((ep << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);
	if (last != EP0_PACKETSIZE) {
		isp_regs->D14_BUFFER_LENGTH = (last & 0xFFFF);
	}
	
	for (size = 0, cp = tx_urb->buffer + endpoint->sent + endpoint->last; size < last; size += 2) {
		temp_i = *cp ++;
		temp_i += (*cp++) << 8;
		isp_regs->D14_DATA_PORT = temp_i;
	}

	ctrl.VALUE = 0;
	ctrl.BITS.DSEN   = 1;
	isp_regs->D14_CONTROL_FUNCTION.VALUE = ctrl.VALUE;

	/* call sendzlp API for zero packet process */
	bi_tx_sendzlp (endpoint);

	/* set endpoint last and call complete API */   
	endpoint->last = last;
	bi_tx_complete_irq(endpoint, 0);
}

/* 
 * ISP1583 IN endpoint interrupt handler 
 */
static void isp_epx_in (u8 ep)
{
	/* The input "ep" is the ISP IN endpoint index, the isp_in_map_udc_ep[] can be used to 
	    get the endpoint structure in UDC layer */

	struct usb_endpoint_instance *endpoint = usbd_bus->endpoint_array + isp_in_map_udc_ep[ep];
	struct urb *tx_urb;
	int last, size, i;
	u8 *cp;
	u16 temp_i;
	INT_ENABLE_LSB l; 
	INT_ENABLE_MSB m;

	isp_set_addressed = 0;

	ISP_SET_ENDPOINT_INDEX((ep << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);

	ISP_DBG("\r\nISP: epx_in %d, %d\r\n", ep, isp_regs->D14_BUFFER_STATUS);

	if ((i = isp_regs->D14_BUFFER_STATUS) != ISP_BUFFER_FULL)   {
		if (!(tx_urb = endpoint->tx_urb)) {    
			l.VALUE = isp_regs->D14_INT_ENABLE_LSB.VALUE;
			m.VALUE = isp_regs->D14_INT_ENABLE_MSB.VALUE;

			isp_ep_in_int (ep, 0, &l, &m);
			isp_regs->D14_INT_ENABLE_LSB.VALUE = l.VALUE;
			isp_regs->D14_INT_ENABLE_MSB.VALUE = m.VALUE;
			ISP_DBG("\r\nendpoint->tx_urb is NULL\r\n");
			return;
		}

		if (tx_urb->dma_mode)  {
			tx_urb->dma_mode = 0;
		}

		/* if last is 0, that means a zero packet shall be sent out */
		last = MIN (tx_urb->actual_length - (endpoint->sent + endpoint->last), endpoint->wMaxPacketSize);

		if(last != endpoint->wMaxPacketSize)
			isp_regs->D14_BUFFER_LENGTH = (last & 0xFFFF);

		for (size = 0, cp = tx_urb->buffer + endpoint->sent + endpoint->last; size < last; size += 2) {
			temp_i = *cp ++;
			temp_i += (*cp++) << 8;
			isp_regs->D14_DATA_PORT = temp_i;
		}

		ISP_DBG("\r\nlast = %d\r\n", last);

		/* call sendzlp API for zero packet process */
		bi_tx_sendzlp (endpoint);

		endpoint->last = last;
		tx_urb =  bi_tx_complete_irq(endpoint, 0); 
	}
}

/* 
 * ISP1583 setup interrupt data transfer 
 */
static void isp_ep0_setup (void)
{
	struct usb_device_request request;
	int len = 0, r_len, max = 8;
	u8 *cp = (u8 *)&request;
	u16 temp_i;
	CONTROL_REG ctrl;
	struct usb_endpoint_instance *endpoint = usbd_bus->endpoint_array + 0;

	ISP_DBG("\r\nISP: ep0_setup\r\n");

	ISP_SET_ENDPOINT_INDEX(ISP_EPINDEX_EP0SETUP);

	for (len = 0; len < (max / 2) ; len++) {
		temp_i = isp_regs->D14_DATA_PORT;
		*cp++ = temp_i & 0xFF;
		*cp++ = temp_i >> 8;
	}

	if (bi_recv_setup_irq (&request)) {   
		/* setup process failed */
		ISP_SET_ENDPOINT_INDEX((0 << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);

		ctrl.VALUE = 0;
		ctrl.BITS.STALL = 1;
		isp_regs->D14_CONTROL_FUNCTION.VALUE = ctrl.VALUE;
		return;
	}

	/*  if High Speed test mode ... */
	if (g_hs_test_packet) {
		udc_release_udc_irq();
		return;
	}

	/* set IN 0 ACK */
	if((request.wLength == 0) && (isp_set_addressed == 0)) {
		ISP_SET_ENDPOINT_INDEX((0 << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);

		ctrl.VALUE = 0;
		ctrl.BITS.STATUS = 1;
		isp_regs->D14_CONTROL_FUNCTION.VALUE = ctrl.VALUE;
	}

	if((request.wLength) && (!(request.bmRequestType & USB_REQ_DIRECTION_MASK)))  {
		ISP_SET_ENDPOINT_INDEX(0 << ISP_EPINDEX_EP_OFFSET);

		ctrl.VALUE = 0;
		ctrl.BITS.DSEN = 1;
		isp_regs->D14_CONTROL_FUNCTION.VALUE = ctrl.VALUE;
	}
}

/*
 * used for update the max packet size if the high speed host is detected
 */
static void update_endpoints_hs_maxpktsize (void)
{
	int epn;

	for (epn=0; epn < usbd_bus->function_instance->endpointsRequested; epn++) {
		struct usb_endpoint_map *endpoint_map = usbd_bus->function_instance->endpoint_map_array + epn;
		int physicalEndpoint = endpoint_map->physicalEndpoint[0];
		struct usb_endpoint_instance *endpoint = usbd_bus->endpoint_array + physicalEndpoint;

		switch(endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) {
		case USB_DIR_IN:
			endpoint->wMaxPacketSize = endpoint_map->wMaxPacketSize[1];
			break;
		case USB_DIR_OUT:
			endpoint->wMaxPacketSize = endpoint_map->wMaxPacketSize[1];
		default:
			break;
		}
	}
}

/*
 * ISP1583 interrupt handler
 */
#ifdef CONFIG_ARCH_EZX
extern int usbd_enum_flag;
extern struct timer_list reenum_timer;
#define RE_ENUM_TIMEOUT         (500*HZ/1000)   // 500ms for pull up/down restistor - from LWQ
#endif

static void isp_int_hndlr (int irq, void *dev_id, struct pt_regs *regs)
{
	INTERRUPT_STATUS_LSB int_status_lsb;
	INTERRUPT_STATUS_MSB int_status_msb;
	u32 flags;
	u16 mode;
	USB_MODE   mod;

	/* detect connect status firstly */
	if (!udc_connected_status) return;

	/* interrupt counter */
	udc_interrupts++;

	/* disable interrupt */
	mode = isp_regs->D14_MODE.VALUE;
	mode &= ISP_MODE_MASK_GLINTENA;
	isp_regs->D14_MODE.VALUE = mode;

	int_status_lsb.VALUE = isp_regs->D14_INT_LSB.VALUE;
	int_status_msb.VALUE = isp_regs->D14_INT_MSB.VALUE;
	int_status_lsb.VALUE &= isp_regs->D14_INT_ENABLE_LSB.VALUE;
	int_status_msb.VALUE &= isp_regs->D14_INT_ENABLE_MSB.VALUE;

	/* clear interrupt status */
	isp_regs->D14_INT_LSB.VALUE = int_status_lsb.VALUE;
	isp_regs->D14_INT_MSB.VALUE = int_status_msb.VALUE;
    
	ISP_DBG("\r\nISP: interrupt handler, 0x%x, 0x%x\r\n",int_status_lsb.VALUE, int_status_msb.VALUE);

	if (int_status_lsb.BITS.HS_STAT) {
		/* bus reset interrupt */
		usbd_bus->HighSpeedFlag = 1;
		update_endpoints_hs_maxpktsize ();
		isp_set_ep_type = 0;
		udc_all_interrupts ();
		if (high_full_state == ISP_FULL_SPEED) {
			high_full_state = ISP_HIGH_SPEED;
		}
	}

	if (int_status_lsb.BITS.RESET)   { 
		/* bus reset interrupt */
		isp_set_ep_type = 0;
		isp_init();    /* re-initialization isp1583 */
		udc_all_interrupts();
		udc_suspended = 0;
		usbd_bus->suspended_state = STATE_INIT;
		usbd_bus->device_state    = STATE_INIT;
		usbd_bus_event_irq (usbd_bus, DEVICE_RESET, 0);

#ifdef CONFIG_ARCH_EZXBASE
		/* disable charge if Reset is received */
		usbd_bus->Raw_ConfigurationValue = 0;
		queue_motusbd_event(USB_CURRENT_0MA);

#endif	// CONFIG_ARCH_EZXBASE

		if (isp_set_addressed) {
			usbd_bus_event_irq (usbd_bus, DEVICE_ADDRESS_ASSIGNED, 0);
		}

		mode = isp_regs->D14_MODE.VALUE;
		mode |= ISP_MODE_GLINTENA;
		isp_regs->D14_MODE.VALUE = mode;
		return;
	}

#ifdef ISP_DMA_ENABLE
	if(int_status_lsb.BITS.DMA)  { /* DMA interrupt */
		DMA_INT dma_int;
		dma_int.VALUE = isp_regs->D14_DMA_INT.VALUE;
		isp_regs->D14_DMA_INT.VALUE = dma_int.VALUE;
		isp_regs->D14_INT_LSB.VALUE = int_status_lsb.VALUE;
		isp_regs->D14_INT_MSB.VALUE = int_status_msb.VALUE;
		isp_dev_dma_irq();
	}
#endif

	if ((int_status_lsb.BITS.RESUME) && (!int_status_lsb.BITS.SUSP))  { 

#ifdef CONFIG_ARCH_EZXBASE
		/* according configuration, set charge */
		if (usbd_bus->Raw_ConfigurationValue)  {
			struct usb_configuration_descriptor *cfg_desc;
	
			usbd_enum_flag = 0;
			cfg_desc = (&usbd_bus->function_instance->function_driver->configuration_instance_array[0])->configuration_descriptor;
			if(cfg_desc->bMaxPower <= POWER_CURRENT_100MA_SETTING) 	{
				queue_motusbd_event(USB_CURRENT_100MA);
			}  else	{
				queue_motusbd_event(USB_CURRENT_500MA);
			}
		} else {
			queue_motusbd_event(USB_CURRENT_0MA);
		}
#endif	// CONFIG_ARCH_EZXBASE

		/* resume interrupt */
		isp_regs->D14_UNLOCK_DEVICE = ISP_UNLOCK_REGS;
		if (udc_suspended) {
			udc_suspended = 0;
			usbd_bus_event_irq (usbd_bus, DEVICE_BUS_ACTIVITY, 0);
		}
	}

	if (int_status_lsb.BITS.SUSP && (!int_status_lsb.BITS.RESUME)) { 
#ifdef CONFIG_ARCH_EZXBASE
		if (usbd_bus->Raw_ConfigurationValue)  {
			queue_motusbd_event(USB_CURRENT_0MA);
		} else {
			extern int udccontrol_context_safe(int control_flag);

			if(usbd_enum_flag && ((usbd_bus->suspended_state == STATE_ADDRESSED) || (usbd_bus->suspended_state == STATE_INIT)) && (usbd_bus->device_state == STATE_ADDRESSED))
			{
				struct usb_configuration_descriptor *cfg_desc;
				usbd_enum_flag = 0;
				cfg_desc = (&usbd_bus->function_instance->function_driver->configuration_instance_array[0])->configuration_descriptor;
				if(cfg_desc->bMaxPower <= POWER_CURRENT_100MA_SETTING) 	{
					queue_motusbd_event(USB_CURRENT_0MA);
				}  else	{
					usbd_bus->Raw_ConfigurationValue = 0;
					isp_reenum_flag = 1;
					udccontrol_context_safe(0);
					mod_timer(&reenum_timer, jiffies + RE_ENUM_TIMEOUT);
				}
			} else if(usbd_bus->device_state == DEVICE_CONFIGURED) {
				queue_motusbd_event(USB_CURRENT_0MA);
			}
		}
#endif	// CONFIG_ARCH_EZXBASE

		/* suspend interrupt */
		if (!udc_suspended) {
			udc_suspended = 1;
			usbd_bus_event_irq (usbd_bus, DEVICE_BUS_INACTIVE, 0);
		}
	}

	if (int_status_lsb.BITS.EP0SETUP)   {  
		/* setup token */
		isp_ep0_setup ();
	}

	if (int_status_lsb.BITS.EP0TX)   { 
		/* endpoint IN 0 */
		isp_ep0_in (0);
	}

	if (int_status_lsb.BITS.EP0RX)   {  
		/* endpoint OUT 0 */
		isp_ep0_out (0);
	}

	if (isp_requested_eps_flag == 0)   {
		mode = isp_regs->D14_MODE.VALUE;
		mode |= ISP_MODE_GLINTENA;
		isp_regs->D14_MODE.VALUE = mode;
		return;
	}

	if (int_status_lsb.BITS.EP1TX)   { 
		/* endpoint IN 1 */
		isp_epx_in (1);
	}

	if (int_status_lsb.BITS.EP1RX)   {  
		/* endpoint OUT 1 */
		isp_epx_out (1);
	}

	if (int_status_lsb.BITS.EP2TX)   { 
		/* endpoint IN 2 */
		isp_epx_in (2);
	}

	if (int_status_lsb.BITS.EP2RX)   {  
		/* endpoint OUT 2 */
		isp_epx_out (2);
	}

	if (int_status_msb.BITS.EP3TX)   { 
		/* endpoint IN 3 */
		isp_epx_in (3);
	}

	if (int_status_msb.BITS.EP3RX)   {  
		/* endpoint OUT 3 */
		isp_epx_out (3);
	}

	if (int_status_msb.BITS.EP4TX)   { 
		/* endpoint IN 4 */
		isp_epx_in (4);
	}

	if (int_status_msb.BITS.EP4RX)   {  
		/* endpoint OUT 4 */
		isp_epx_out (4);
	}

	if (int_status_msb.BITS.EP5TX)   { 
		/* endpoint IN 5 */
		isp_epx_in (5);
	}

	if (int_status_msb.BITS.EP5RX)   {  
		/* endpoint OUT 6 */
		isp_epx_out (5);
	}

	if (int_status_msb.BITS.EP6TX)   { 
		/* endpoint IN 6 */
		isp_epx_in (6);
	}

	if (int_status_msb.BITS.EP6RX)   {  
		/* endpoint OUT 6 */
		isp_epx_out (6);
	}

	if (int_status_msb.BITS.EP7TX)   { 
		/* endpoint IN 7 */
		isp_epx_in (7);
	}

	if (int_status_msb.BITS.EP7RX)   { 
		 /* endpoint OUT 7 */
		isp_epx_out (7);
	}

	/* enable interrupt */
	mode = isp_regs->D14_MODE.VALUE;
	mode |= ISP_MODE_GLINTENA;
	isp_regs->D14_MODE.VALUE = mode;
}

/* 
 * udc_start_endpoint_in - start transmit
 */                     
void udc_start_endpoint_in (struct usb_endpoint_instance *endpoint)
{                       
	int epn = endpoint->bEndpointAddress & 0xf;
	INT_ENABLE_LSB l; INT_ENABLE_MSB m;

	ISP_DBG("\r\nISP: udc_start_endpoint_in, %d\r\n", epn);

	if (g_hs_test_packet) 
		return;

	/* enable endpoint data transfer */
	if (epn != 0)    {
#ifdef ISP_DMA_ENABLE
		if ((endpoint->tx_urb) && (endpoint->tx_urb->dma_mode) && (isp_dma_flag == 0))  {
			/* disable endpoint interrupt */
			l.VALUE = isp_regs->D14_INT_ENABLE_LSB.VALUE;
			m.VALUE = isp_regs->D14_INT_ENABLE_MSB.VALUE;
			isp_ep_in_int(epn,0,&l,&m);
			isp_regs->D14_INT_ENABLE_LSB.VALUE = l.VALUE;
			isp_regs->D14_INT_ENABLE_MSB.VALUE = m.VALUE;
                                                                                                  
			ISP_DBG("\r\nISP: ep=%d, request_length =%d, physaddr= 0x%x\r\n",epn,endpoint->tx_urb->actual_length,endpoint->tx_urb->dma_physaddr);
                                                                                                  
			isp_dma_ep = epn | USB_DIR_IN;
			if (endpoint->tx_urb->actual_length > MAX_DMA_SIZE)
				isp_dma_len = MAX_DMA_SIZE;
			else
				isp_dma_len = endpoint->tx_urb->actual_length;
			isp_dma_index_len = isp_dma_len;
			
			dma_addr_t dma_addr = virt_to_bus(endpoint->tx_urb->buffer);
			consistent_sync(endpoint->tx_urb->buffer, MAX_DMA_SIZE, PCI_DMA_TODEVICE);

			isp_epx_tx_set_dma (dma_addr);
			return;
		}
#endif  /* ISP_DMA_ENABLE */
		/* DMA is not used, clear urb dma_mode */
		if (endpoint->tx_urb)
			endpoint->tx_urb->dma_mode = 0;

		isp_epx_in (epn);
	} else  {
		isp_ep0_in (epn);
	}

	/* enable endpoint interrupt */
	l.VALUE = isp_regs->D14_INT_ENABLE_LSB.VALUE;
	m.VALUE = isp_regs->D14_INT_ENABLE_MSB.VALUE;
	isp_ep_in_int (epn, 1, &l, &m);
	isp_regs->D14_INT_ENABLE_LSB.VALUE = l.VALUE;
	isp_regs->D14_INT_ENABLE_MSB.VALUE = m.VALUE;
}

/* 
 * udc_start_endpoint_out - start receive
 */
void udc_start_endpoint_out (struct usb_endpoint_instance *endpoint)
{
	int epn = endpoint->bEndpointAddress & 0xf;
	struct urb *rcv_urb;
	INT_ENABLE_LSB l; 
	INT_ENABLE_MSB m;
    
	ISP_DBG("\r\nISP: udc_start_endpoint_out, %d\r\n",epn);

	if (g_hs_test_packet) 
		return;

	if (epn)    {
#ifdef ISP_DMA_ENABLE
		if((endpoint->rcv_urb) && (endpoint->rcv_urb->dma_mode) && (isp_dma_flag == 0))  {
			l.VALUE = isp_regs->D14_INT_ENABLE_LSB.VALUE;
			m.VALUE = isp_regs->D14_INT_ENABLE_MSB.VALUE;
			isp_ep_out_int(epn,0,&l,&m);
			isp_regs->D14_INT_ENABLE_LSB.VALUE = l.VALUE;
			isp_regs->D14_INT_ENABLE_MSB.VALUE = m.VALUE;
                                                                                                  
			ISP_DBG("\r\nISP: ep=%d, request_length =%d, physaddr= 0x%x\r\n",epn,endpoint->rcv_urb->request_length,endpoint->rcv_urb->dma_physaddr);

			isp_dma_ep = epn;
			if(endpoint->rcv_urb->request_length > MAX_DMA_SIZE)
				isp_dma_len = MAX_DMA_SIZE;
			else
				isp_dma_len = endpoint->rcv_urb->request_length;
			isp_dma_index_len = isp_dma_len;
			dma_addr_t dma_addr = virt_to_bus(endpoint->rcv_urb->buffer);
			consistent_sync(endpoint->rcv_urb->buffer, MAX_DMA_SIZE, PCI_DMA_FROMDEVICE);
			isp_epx_rx_set_dma(dma_addr);
			return;
		}
#endif
		/* DMA is not used, clear urb dma_mode */
		if (endpoint->rcv_urb)
			endpoint->rcv_urb->dma_mode = 0;

		isp_epx_out(epn);
	}

	/* enable endpoint interrupt */
	l.VALUE = isp_regs->D14_INT_ENABLE_LSB.VALUE;
	m.VALUE = isp_regs->D14_INT_ENABLE_MSB.VALUE;
	isp_ep_out_int (epn, 1, &l, &m);
	isp_regs->D14_INT_ENABLE_LSB.VALUE = l.VALUE;
	isp_regs->D14_INT_ENABLE_MSB.VALUE = m.VALUE;
	ISP_DBG("\r\nISP: end udc_start_endpoint_out, %d\r\n",epn);
}

/*
 * ISP1583 endpoint OUT stop 
 */
void udc_stop_endpoint_out (struct usb_endpoint_instance *endpoint)
{
	int epn = endpoint->bEndpointAddress & 0xf;
	INT_ENABLE_LSB l; 
	INT_ENABLE_MSB m;

	ISP_DBG("\r\nISP: udc_stop_endpoint_out\r\n");

	if (g_hs_test_packet) 
		return;

	/* disable ep interrupt */
	l.VALUE = isp_regs->D14_INT_ENABLE_LSB.VALUE;
	m.VALUE = isp_regs->D14_INT_ENABLE_MSB.VALUE;
	isp_ep_out_int (epn, 0, &l, &m);
	isp_regs->D14_INT_ENABLE_LSB.VALUE = l.VALUE;
	isp_regs->D14_INT_ENABLE_MSB.VALUE = m.VALUE;
}

/* 
 * udc_cancel_in_irq - cancel IN urb
 */
void udc_cancel_in_irq (struct urb *urb)
{
	int ep = urb->endpoint->bEndpointAddress & 0xf;
	INT_ENABLE_LSB l; 
	INT_ENABLE_MSB m;

	ISP_DBG("\r\nISP: udc_cancel_in_irq\r\n");

	if (g_hs_test_packet) 
		return;

	/* disable ep interrupt */
	l.VALUE = isp_regs->D14_INT_ENABLE_LSB.VALUE;
	m.VALUE = isp_regs->D14_INT_ENABLE_MSB.VALUE;
	isp_ep_in_int (ep, 0, &l, &m);
	isp_regs->D14_INT_ENABLE_LSB.VALUE = l.VALUE;
	isp_regs->D14_INT_ENABLE_MSB.VALUE = m.VALUE;
}

/* 
 * udc_cancel_out_irq - cancel OUT urb
 */
void udc_cancel_out_irq(struct urb *urb)
{
	int ep = urb->endpoint->bEndpointAddress & 0xf;
	INT_ENABLE_LSB l; 
	INT_ENABLE_MSB m;

	ISP_DBG("\r\nISP: udc_cancel_out_irq\r\n");

	if (g_hs_test_packet) 
		return;

	/* disable ep interrupt */
	l.VALUE = isp_regs->D14_INT_ENABLE_LSB.VALUE;
	m.VALUE = isp_regs->D14_INT_ENABLE_MSB.VALUE;
	isp_ep_out_int (ep, 0, &l, &m);
	isp_regs->D14_INT_ENABLE_LSB.VALUE = l.VALUE;
	isp_regs->D14_INT_ENABLE_MSB.VALUE = m.VALUE;
}

/* 
 * udc_init - initialize the isp UDC
 */
int udc_init (void)
{
	ISP_DBG("\r\nISP: udc_init\r\n");

	/* disconnect ISP soft connect before switch */
	USB_MODE mod;

	mod.VALUE = isp_regs->D14_MODE.VALUE;
	mod.BITS.SOFTCT   = 0;
	isp_regs->D14_MODE.VALUE   = mod.VALUE;

	/* switch to USB2.0 */
	set_GPIO(GPIO_USB20_SWITCH);

	/* delay 30ms to make sure the signal is stability */
	set_current_state(TASK_INTERRUPTIBLE);    
	schedule_timeout(3*HZ/100);                 
 
	/* reset the ISP1583 */
	mod.VALUE = isp_regs->D14_MODE.VALUE;
	mod.BITS.SFRESET = 1;
	isp_regs->D14_MODE.VALUE = mod.VALUE;
	udelay(500);
	mod.BITS.SFRESET = 0;
	isp_regs->D14_MODE.VALUE = mod.VALUE;

	/* initialize the ISP1583 */
	isp_init();
	return 0;
}

/* 
 * udc_stall_ep - stall endpoint
 */
void udc_stall_ep (u32 phys_ep)
{
    ISP_DBG("\r\nISP: udc_stall_ep\r\n");
}

/* 
 * udc_reset_ep - reset endpoint
 */
void udc_reset_ep (unsigned int phys_ep)
{
    ISP_DBG("\r\nISP: udc_reset_ep %d\r\n",phys_ep);
}

/* 
 * udc_endpoint_halted - is endpoint halted
 */
int udc_endpoint_halted (unsigned int phys_ep)
{
    ISP_DBG("\r\nISP: udc_endpoint_halted\r\n");
    return 0;
}

/* 
 * udc_set_address - set the USB address for this device
 */
void udc_set_address (unsigned char address)
{
	ISP_DBG("\r\nISP: udc_set_address: %d\r\n",address & ISP_ADDR_MASK);

	/* unlock the ISP158x register */
	isp_regs->D14_UNLOCK_DEVICE = ISP_UNLOCK_REGS;
	isp_regs->D14_ADDRESS.BITS.DEVADDR = (address & ISP_ADDR_MASK);

	/* detect whether a non-zero address */
	if ((address & ISP_ADDR_MASK) != 0)    {
		ISP_SET_ENDPOINT_INDEX((0 << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);

		CONTROL_REG ctrl;
		ctrl.VALUE = 0;
		ctrl.BITS.STATUS = 1;
		isp_regs->D14_CONTROL_FUNCTION.VALUE = ctrl.VALUE;

		isp_set_addressed = 1;
	}
}

/* 
 * udc_serial_init - set a serial number if available
 */
int __init udc_serial_init (void)
{
	ISP_DBG("\r\nISP: udc_serial_init\r\n");
	return -EINVAL;
}

/* 
 * udc_max_endpoints - max physical endpoints 
 */
int udc_max_endpoints (void)
{
    ISP_DBG("\r\nISP: udc_max_endpoints\r\n");
    return UDC_MAX_ENDPOINTS;
}

/* 
 * udc_set_ep - setup endpoint 
 */
void udc_setup_ep (unsigned int ep, struct usb_endpoint_instance *endpoint)
{
	int i;
	INT_ENABLE_LSB l; 
	INT_ENABLE_MSB m;

	ISP_DBG("\r\nISP: udc_set_ep ep = %d, ep->address = %x\r\n", ep, endpoint->bEndpointAddress);

	l.VALUE = isp_regs->D14_INT_ENABLE_LSB.VALUE;
	m.VALUE = isp_regs->D14_INT_ENABLE_MSB.VALUE;

	if ((ep == 0) || (endpoint->bEndpointAddress & 0x80))  {
		i = endpoint->bEndpointAddress & 0x0F;
		isp_ep_in_int (i, 1, &l, &m);
	} else if (endpoint->bEndpointAddress)  { 
		RETURN_IF(bi_rcv_next_irq (endpoint));
		i = endpoint->bEndpointAddress & 0x0F;
		isp_ep_out_int (i, 1, &l, &m);
	}

	isp_regs->D14_INT_ENABLE_LSB.VALUE = l.VALUE;
	isp_regs->D14_INT_ENABLE_MSB.VALUE = m.VALUE;
}

/* 
 * udc_disable_ep - disable endpoint
 */
void udc_disable_ep (unsigned int ep)
{
	INT_ENABLE_LSB l; 
	INT_ENABLE_MSB m;
	u8 epn;
	struct usb_endpoint_instance *endpoint;

	ISP_DBG("\r\nISP: udc_disable_ep: %d\r\n",ep);

	endpoint = usbd_bus->endpoint_array + ep;
	epn = endpoint->bEndpointAddress;

	l.VALUE = isp_regs->D14_INT_ENABLE_LSB.VALUE;
	m.VALUE = isp_regs->D14_INT_ENABLE_MSB.VALUE;
	if ((epn & USB_ENDPOINT_DIR_MASK) != USB_DIR_IN) {
		isp_ep_out_int (epn & (~USB_DIR_OUT), 0, &l, &m);
	} else {
		isp_ep_in_int (epn & (~USB_DIR_IN), 0, &l, &m);
	}

	isp_regs->D14_INT_ENABLE_LSB.VALUE = l.VALUE;
	isp_regs->D14_INT_ENABLE_MSB.VALUE = m.VALUE;
}

/* 
 * udc_attached - is the USB cable connected
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
extern int usbcable_status;     /* from PCAP INT */
extern int powermode_flag;
extern void pulldown_usb (void);
extern void pullup_usb (void);
extern void restore_powermode (void);

void clear_device_state (void)
{
	if (usbd_bus)  {
		usbd_bus->suspended_state = STATE_INIT;
		usbd_bus->device_state = STATE_INIT;
	}
}

int udc_attached (void)
{
	ISP_DBG("\r\nISP: udc_attached %d\r\n",usbcable_status);

	return usbcable_status;
}

int udc_connected (void)
{
	ISP_DBG("\r\nISP: udc_connected %d\r\n",udc_connected_status);

	return udc_connected_status;
}

void udc_connect (void)
{
	ISP_DBG("\r\nISP: udc_connect %d\r\n",udc_connected_status);

	/* connect ISP soft connect before switch */
	USB_MODE mod;

	if (isp_reenum_flag) {
		mod.VALUE = isp_regs->D14_MODE.VALUE;
		mod.BITS.SOFTCT   = 1;
		isp_regs->D14_MODE.VALUE   = mod.VALUE;
	}

	if (udc_connected_status == 0)   {
		udc_connected_status = 1;
	}

}

void udc_disconnect (void)
{
	ISP_DBG("\r\nISP: udc_disconnect %d\r\n",udc_connected_status);

	/* disconnect ISP soft connect */
	USB_MODE mod;

	if (isp_reenum_flag) {
		mod.VALUE = isp_regs->D14_MODE.VALUE;
		mod.BITS.SOFTCT   = 0;
		isp_regs->D14_MODE.VALUE   = mod.VALUE;
	}

	if (udc_connected_status == 1)  {
		udc_connected_status = 0;
		clear_device_state ();
	}
}

/* 
 * udc_framenum - get current framenum
 */
int udc_framenum (void)
{
	ISP_DBG("\r\nISP: udc_framenum %d\r\n",isp_regs->D14_FRAME_NUMBER.VALUE & 0xFFFF);
	return ((isp_regs->D14_FRAME_NUMBER.VALUE) & ISP_FNO_MASK);
}

/* 
 * udc_enable_interrupts - enable interrupts
 */
void udc_all_interrupts (void)
{
	int i,j;
	u16 ep_type;
	INT_ENABLE_LSB l; 
	INT_ENABLE_MSB m;
    
	ISP_DBG("\r\nISP: udc_all_interrupts \r\n");

	/* enable all default interrupt */
	l.VALUE = 0; 
	m.VALUE = 0;   
	l.BITS.IESUSP = 1;   /* suspend */
	l.BITS.IERESM = 1;   /* resume */
	l.BITS.IERST = 1;    /* reset */
	l.BITS.IEP0SETUP = 1;  /* setup */
	l.BITS.IEHS_STA = 1;  /* setup */
	l.BITS.IEP0RX = 1;  /* out endpoint 0 */
	l.BITS.IEP0TX = 1;  /* in endpoint 0 */

#ifdef ISP_DMA_ENABLE
	l.BITS.IEDMA = 1;  /* DMA interrupt */
#endif
 
	isp_regs->D14_INT_ENABLE_LSB.VALUE = l.VALUE;
	isp_regs->D14_INT_ENABLE_MSB.VALUE = m.VALUE;

	/* set endpoint type */
	if ((isp_requested_eps_flag) && (!isp_set_ep_type)) {      
		for (i = 1; i < UDC_MAX_ENDPOINTS; i++)  {
			j = usbd_bus->endpoint_array[i].bEndpointAddress;
			if (j)  {
				if(usbd_bus->endpoint_array[i].bmAttributes & USB_DIR_IN)  {
					isp_set_epx_in (j & 0x0f);
				} else {
					isp_set_epx_out (j & 0x0f);
				}
			} 
		}

		for (i = 1; i < UDC_MAX_ENDPOINTS; i++)  {
			j = usbd_bus->endpoint_array[i].bEndpointAddress;
			if (j)   {
				if(usbd_bus->endpoint_array[i].bmAttributes & USB_DIR_IN)  {
					/* 
					 * the following is used to clear the endpoint IN buffer
					 * please refer to philip AN10045
					 */
					ISP_SET_ENDPOINT_INDEX(((j&0x0f) << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);

					ep_type = isp_regs->D14_ENDPT_TYPE.VALUE ;
					ep_type |= ISP_EPT_ENABLE;
					isp_regs->D14_ENDPT_TYPE.VALUE = ep_type ;
					isp_regs->D14_CONTROL_FUNCTION.VALUE = ISP_CTLFUN_VENDP;

					ISP_SET_ENDPOINT_INDEX(((j&0x0f) << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);
					isp_regs->D14_CONTROL_FUNCTION.VALUE = ISP_CTLFUN_VENDP;
             
				} else {
					ISP_SET_ENDPOINT_INDEX((j&0x0f) << ISP_EPINDEX_EP_OFFSET);

					ep_type = isp_regs->D14_ENDPT_TYPE.VALUE ;
					ep_type |= ISP_EPT_ENABLE;
					isp_regs->D14_ENDPT_TYPE.VALUE = ep_type ;
				}
			} 
		}
		isp_set_ep_type = 1;
	}

	/* enable ISP1583 */
	isp_regs->D14_ADDRESS.BITS.DEVEN = 1;
}

/* 
 * udc_suspended_interrupts - enable suspended interrupts
 */
void udc_suspended_interrupts (void)
{
	ISP_DBG("\r\nISP: udc_suspended_interrupts \r\n");
}

/*
 * initialize the bulverde relevant GPIOs
 */
static void isp_hw_gpio_init(void)
{
	u32 temp;

	
	/* Enable CS3 and set MSC1 register */
	set_GPIO_mode(GPIO_USB20_NCS3 | GPIO_ALT_FN_2_OUT);

	temp = MSC1;

	/* MSC1 is used to configure the CS3 memory access clock 
	 * This value shall be adjusted according the the system memory clock.
	 * if system memory clock is 312M, 0x758C can be used. If the system memory
	 * clock is 208M, this value can be changed into 0x44AC in order to increase
	 * the performance.
	 */
	MSC1 = (temp & 0x0000FFFF)|0x758C0000; 

	/* 0x7FFC is the maximum CS3 configure value */
	/* MSC1 = (temp & 0x0000FFFF)|0x7FFC0000; */

	set_GPIO_mode(GPIO_HDD_USB20_READY|GPIO_ALT_FN_1_IN);
	set_GPIO_mode(GPIO_NPWE|GPIO_ALT_FN_2_OUT);
#ifdef ISP_DMA_ENABLE
	set_GPIO_mode(GPIO_USB20_DREQ0|GPIO_ALT_FN_1_IN);
#endif

	/* initialization interrupt GPIO pin */
	set_GPIO_mode(GPIO_USB20_INT | GPIO_IN);
	set_GPIO_IRQ_edge(GPIO_USB20_INT,GPIO_FALLING_EDGE);

	/* configure GPIO to enable ISP chip */
	set_GPIO_mode(GPIO_USB20_SWITCH | GPIO_OUT);
	clr_GPIO(GPIO_USB20_SWITCH);

	/* initialization interrupt GPIO pin */
	isp_regs = (volatile D14_CNTRL_REG  *)ioremap_nocache((unsigned long)ISP_REGBASE_ADDR, 0x400);

}

/* 
 * udc_disable_interrupts - disable interrupts.
 */
void udc_disable_interrupts (void)
{
	USB_MODE mod;
	u16 i;
    
	ISP_DBG("\r\nISP: udc_disable_interrupts \r\n");

	/* using iomap the ISP registers */
	if (isp_regs == 0)    {
		isp_hw_gpio_init();
		isp_regs->D14_UNLOCK_DEVICE = 0xAA37;
		mod.VALUE = isp_regs->D14_MODE.VALUE;
		mod.BITS.SFRESET = 1;
		isp_regs->D14_MODE.VALUE = mod.VALUE;
		mod.BITS.SFRESET = 0;
		isp_regs->D14_MODE.VALUE = mod.VALUE;
	} 

	/* clear all interrupt status */
	i = isp_regs->D14_INT_LSB.VALUE;
	isp_regs->D14_INT_LSB.VALUE = i;
	i = isp_regs->D14_INT_MSB.VALUE;
	isp_regs->D14_INT_MSB.VALUE = i;

#ifdef ISP_DMA_ENABLE
	DMA_INT dma_int;
	dma_int.VALUE = isp_regs->D14_DMA_INT.VALUE;
	isp_regs->D14_DMA_INT.VALUE = dma_int.VALUE;
#endif

	/* disable all interrupt */
	isp_regs->D14_INT_ENABLE_LSB.VALUE = 0;
	isp_regs->D14_INT_ENABLE_MSB.VALUE = 0; 
}

/* 
 * udc_ep0_packetsize - return ep0 packetsize
 */
int udc_ep0_packetsize (void)
{
	ISP_DBG("\r\nISP: udc_ep0_packetsize %d\r\n",EP0_PACKETSIZE);

	return EP0_PACKETSIZE;
}

/* 
 * udc_enable - enable the UDC
 */
void udc_enable (void)
{
	ISP_DBG("\r\nISP: udc_enable \r\n");
}

/* udc_disable - disable the UDC
 */
void udc_disable (void)
{
	ISP_DBG("\r\nISP: udc_disenable \r\n");

	isp_requested_eps_flag  = 0;     
	isp_set_ep_type         = 0;
	g_hs_test_packet        = 0;
	isp_set_addressed       = 0;
	usbd_bus->HighSpeedFlag = 0;

	isp_reenum_flag = 0;
	
	/* reset all endpoint map parameters */
	memset(isp_in_map_udc_ep, 0, sizeof(isp_in_map_udc_ep));
	memset(isp_out_map_udc_ep, 0, sizeof(isp_out_map_udc_ep));

	/* disconect the ISP1583 soft connect before switch to USB1.1 */
	USB_MODE m;
	m.VALUE = isp_regs->D14_MODE.VALUE;
	m.BITS.SOFTCT   = 0;
	isp_regs->D14_MODE.VALUE   = m.VALUE;

	/* switch to USB1.1 */
	clr_GPIO(GPIO_USB20_SWITCH);
}

/* 
 * udc_startup_events - allow udc code to do any additional startup
 */
void udc_startup_events (void)
{
	ISP_DBG("\r\nISP: udc_startup_events \r\n");

	usbd_bus_event_irq (usbd_bus, DEVICE_INIT, 0);
	usbd_bus_event_irq (usbd_bus, DEVICE_CREATE, 0);
	usbd_bus_event_irq (usbd_bus, DEVICE_HUB_CONFIGURED, 0);
}

/* 
 * udc_name - return name of USB Device Controller
 */
char *udc_name (void)
{
	ISP_DBG("\r\nISP: udc_name \r\n");

	return UDC_NAME;
}

/* 
 * udc_request_udc_irq - request UDC interrupt
 */
int udc_request_udc_irq (void)
{
	int ret;


	
	/*
	 * MSC1 is modified after AP sleep/wakeup. Setup it again to make sure ISP1583 is working after AP sleep/wakep.
	 */
	MSC1 = (MSC1 & 0x0000FFFF)|0x758C0000;
					  

	/* In NFS mode, wait until the USB cale is connected so that ISP1583 is powered.
	 * By checking if the CHIP ID is correct, we know if ISP1583 is powered.
	 */
	if (usbd_in_nfs_mode()) {
		printk ("Please insert USB cable to phone.\n");
		while(isp_regs->D14_CHIP_ID_LSB != 0x8230) ;
	}

	ret = request_irq(IRQ_ISP_USB, isp_int_hndlr, SA_INTERRUPT, UDC_NAME " USBD Bus Interface", NULL);

	ISP_DBG("\r\nISP: udc_request_irq %d\r\n",ret);

	isp_requested_irq_flag = 1;
	udc_all_interrupts();

#ifdef ISP_DMA_ENABLE
	if (isp_pxa_dma_init())     {
		ISP_DBG("\r\nAllocate PXA DMA channel and memory error\r\n");
		return 1;
	}
#endif

	return ret;
}

/* 
 * udc_request_cable_irq - request Cable interrupt
 */
int udc_request_cable_irq(void)
{
	ISP_DBG("\r\nISP: udc_request_cable_irq \r\n");

	return 0;
}

/* 
 * udc_request_udc_io - request UDC io region
 */
int udc_request_io (void)
{
	ISP_DBG("\r\nISP: udc_request_io \r\n");

	return 0;
}

/* 
 * udc_release_release_io - release UDC io region
 */
void udc_release_io (void)
{
	ISP_DBG("\r\nISP: udc_release_io \r\n");
}

/* 
 * udc_release_udc_irq - release UDC irq
 */
void udc_release_udc_irq (void)
{
	ISP_DBG("\r\nISP: udc_release_irq \r\n");

	free_irq (IRQ_ISP_USB, NULL);
	isp_requested_irq_flag = 0;

#ifdef ISP_DMA_ENABLE
	/* release DMA */
	isp_pxa_dma_exit();
#endif

}

/* 
 * udc_release_cable_irq - release Cable irq
 */
void udc_release_cable_irq (void)
{
	ISP_DBG("\r\nISP: udc_release_cable \r\n");
}

/* 
 * udc_assign_endpoint - keep track of used endpoints
 */
int udc_assign_endpoint( u8 bEndpointAddress, u8 bPhysEndpointAddress, struct usb_endpoint_map *endpoint_map, u8 bmAttributes, u16 wMaxPacketSize_fs, u16 wMaxPacketSize_hs, u16 transferSize_fs, u16 transferSize_hs, u8 configuration, u8 interface, u8 alternate) 
{
	u8  i;
        
	ISP_DBG("\r\nISP: udc_assign_endpoint 0x%x\r\n",bEndpointAddress);

	endpoint_map->configuration = configuration;
	endpoint_map->interface = interface;
	endpoint_map->alternate = alternate;

	/* the bEndpointAddress is for ISP endpoint */
	endpoint_map->bEndpointAddress[0] = bEndpointAddress;
	endpoint_map->bEndpointAddress[1] = bEndpointAddress;

	/* the bPhyEndpointAddress is the the endpoint index for UDC endpoint array */
	if (bPhysEndpointAddress & USB_DIR_IN)   {
		i = bPhysEndpointAddress & (~USB_DIR_IN);
		endpoint_map->physicalEndpoint[0] = i;
		endpoint_map->physicalEndpoint[1] = i;
	} else  {
		i = bPhysEndpointAddress & (~USB_DIR_OUT);
		endpoint_map->physicalEndpoint[0] = i;
		endpoint_map->physicalEndpoint[1] = i;
	}

	endpoint_map->transferSize[0] = transferSize_fs;
	endpoint_map->transferSize[1] = transferSize_hs;
	endpoint_map->bmAttributes[0] = bmAttributes;
	endpoint_map->bmAttributes[1] = bmAttributes;
	endpoint_map->wMaxPacketSize[0] = wMaxPacketSize_fs;
	endpoint_map->wMaxPacketSize[1] = wMaxPacketSize_hs;
	return 0;
}

/* 
 * udc_request_endpoints - process endpoint requests
 */
struct usb_endpoint_map * udc_request_endpoints(int endpointsRequested, struct usb_endpoint_request *requestedEndpoints)
{
	struct usb_endpoint_map *endpoint_map_array = NULL;
	struct usb_device_description *device_description;
	int i,ret, in_ep_num, out_ep_num;

	ISP_DBG("\r\nISP: udc_request_endpoints %d \r\n",endpointsRequested);

	isp_requested_eps_flag = 1;     

	in_ep_num = 0;
	out_ep_num = 0;

	for (i = 0; i < endpointsRequested; i++) {
		if (requestedEndpoints[i].bmAttributes & USB_DIR_IN) {
			in_ep_num ++;
		} else {
			out_ep_num ++;
		}
	}

	/* detect endpoint supported numbers : IN < 7 and Out < 7;  (not contain endpoint 0) */
	THROW_IF((in_ep_num) >= UDC_MAX_IN_ENDPOINTS, error);
	THROW_IF((out_ep_num) >= UDC_MAX_OUT_ENDPOINTS, error);
        
	RETURN_NULL_IF(!(endpoint_map_array = ckmalloc(sizeof(struct usb_endpoint_map) * endpointsRequested, GFP_KERNEL)));

	in_ep_num = 1;   
	out_ep_num = 1;
 
	for (i = 0; i < endpointsRequested; i++)   {
		struct usb_endpoint_map *endpoint_map = endpoint_map_array + i;
		u8 bmAttributes = requestedEndpoints[i].bmAttributes;
		u16 transferSize_fs = requestedEndpoints[i].fs_requestedTransferSize;
		u16 transferSize_hs = requestedEndpoints[i].hs_requestedTransferSize;
		u8 configuration = requestedEndpoints[i].configuration;
		u8 interface = requestedEndpoints[i].interface;
		u8 alternate = requestedEndpoints[i].alternate;

		/* full speed, max packet size is 64, high speed max packet size is 512 */
	        switch (endpoint_map->bmAttributes[0] = bmAttributes)  {
		case USB_DIR_IN | USB_ENDPOINT_BULK:
			ret = udc_assign_endpoint (USB_DIR_IN | in_ep_num, USB_DIR_IN | (i + 1),endpoint_map, bmAttributes, USB_FS_BULK_MAXPKTSIZE, USB_HS_BULK_MAXPKTSIZE, transferSize_fs,transferSize_hs, configuration,interface,alternate);

			/* save the UDC endpoint array index for this endpoint */
			isp_in_map_udc_ep[in_ep_num] = i + 1;

			in_ep_num ++;
			if (ret == 0) 
				continue;
			break;
		case USB_DIR_OUT | USB_ENDPOINT_BULK:
			ret = udc_assign_endpoint (USB_DIR_OUT | out_ep_num, USB_DIR_OUT | (i + 1), endpoint_map, bmAttributes, USB_FS_BULK_MAXPKTSIZE, USB_HS_BULK_MAXPKTSIZE, transferSize_fs, transferSize_hs, configuration, interface, alternate);

			/* save the UDC endpoint array index for this endpoint */
			isp_out_map_udc_ep[out_ep_num] = i + 1;

			out_ep_num ++;
			if (ret == 0) 
				continue;
			break;
		case USB_DIR_IN | USB_ENDPOINT_ISOCHRONOUS:
			ret = udc_assign_endpoint (USB_DIR_IN | in_ep_num, USB_DIR_IN | (i + 1), endpoint_map, bmAttributes,USB_FS_ISO_MAXPKTSIZE, USB_HS_ISO_MAXPKTSIZE, transferSize_fs, transferSize_hs, configuration, interface, alternate);

			/* save the UDC endpoint array index for this endpoint */
			isp_in_map_udc_ep[in_ep_num] = i + 1;

			in_ep_num ++;
			if (ret == 0) 
				continue;
			break;
		case USB_DIR_OUT | USB_ENDPOINT_ISOCHRONOUS:
			ret = udc_assign_endpoint (USB_DIR_OUT | out_ep_num, USB_DIR_OUT | (i + 1), endpoint_map, bmAttributes, USB_FS_ISO_MAXPKTSIZE, USB_HS_ISO_MAXPKTSIZE, transferSize_fs, transferSize_hs, configuration, interface, alternate);

			/* save the UDC endpoint array index for this endpoint */
			isp_out_map_udc_ep[out_ep_num] = i + 1;

			out_ep_num ++;
			if (ret == 0) 
				continue;
			break;
		case USB_DIR_IN | USB_ENDPOINT_INTERRUPT:
		case USB_DIR_IN | USB_ENDPOINT_INTERRUPT | USB_ENDPOINT_OPT:
			ret = udc_assign_endpoint (USB_DIR_IN | in_ep_num, USB_DIR_IN | (i + 1), endpoint_map, bmAttributes, USB_FS_INT_MAXPKTSIZE, USB_HS_INT_MAXPKTSIZE, transferSize_fs,transferSize_hs, configuration, interface, alternate);

			/* save the UDC endpoint array index for this endpoint */
			isp_in_map_udc_ep[in_ep_num] = i + 1;

			in_ep_num ++;
			if (ret == 0) 
				continue;
			break;
		case USB_DIR_OUT | USB_ENDPOINT_INTERRUPT:
		case USB_DIR_OUT | USB_ENDPOINT_INTERRUPT | USB_ENDPOINT_OPT:
		default:
			break;
		}
		CONTINUE_IF(bmAttributes & USB_ENDPOINT_OPT);
		THROW(error);
	}

	CATCH(error) {
		lkfree (endpoint_map_array);
		return NULL;
	}

	return endpoint_map_array;
}

/* 
 * udc_set_endpoints - setup the physical endpoints for the endpoint map
 * The total number of such allocated endpoints cannot exceed 7 In and 7 Out.
 */
int udc_set_endpoints (int endpointsRequested, struct usb_endpoint_map *endpoint_map_array)
{
	u8 phys_in_ep;
	u8 phys_out_ep;
	u16 i;
        
	ISP_DBG("\r\nISP: udc_set_endpoints %d \r\n",endpointsRequested);

	phys_in_ep = 0; 
	phys_out_ep = 0;

	for (i = 0; i < endpointsRequested; i++) { 
		struct usb_endpoint_map *endpoint_map = endpoint_map_array + i;
		int epreq = endpoint_map->bmAttributes[0];
		int eptype = epreq & USB_ENDPOINT_MASK;
		int epdir = epreq & USB_ENDPOINT_DIR_MASK ? 0x8 : 0;
		int epsize = endpoint_map->wMaxPacketSize[0];
        
		if (endpoint_map->bEndpointAddress[0] & USB_DIR_IN) { 
			/* Endpoint Direction */
			phys_in_ep = (endpoint_map->bEndpointAddress[0]) & (~USB_DIR_IN);
			/* alloc physical in endpoint */
			ISP_SET_ENDPOINT_INDEX((phys_in_ep << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);

			isp_regs->D14_ENDPT_MAXPKTSIZE.VALUE = epsize;  
		}  else  {
			/* alloc physical out endpoint */
			phys_out_ep  = endpoint_map->bEndpointAddress[0];
			ISP_SET_ENDPOINT_INDEX(phys_out_ep << ISP_EPINDEX_EP_OFFSET);
			isp_regs->D14_ENDPT_MAXPKTSIZE.VALUE = epsize;  
		}

		ENDPT_TYPE t;
		t.VALUE = 0;
		t.BITS.ENDPTYP = eptype;
		t.BITS.ENABLE  = 1;
		t.BITS.DBLBUF  = 1;
		t.BITS.NOEMPKT = 1;
		isp_regs->D14_ENDPT_TYPE.VALUE = t.VALUE;
	}
	return 0;
}

/* 
 * define the test packet 
 */
#define USB20_TEST_PACKET_SIZE   56

static unsigned short isp_hs_test_packet[] = { 
	0x00c3,
	0x0000,0x0000,0x0000,0x0000,
	0xaaaa,0xaaaa,0xaaaa,0xaaaa,
	0xeeee,0xeeee,0xeeee,0xeeee,
	0xfffe,0xffff,0xffff,0xffff,0xffff,0xffff,
	0xbf7f,0xefdf,0xfbf7,0xfcfd,0xbf7e,0xefdf,0xfbf7,0x7efd,0xceb6
};

/* 
 * the function is used only for test 
 */
static void hs_test_delay (void)
{
	int i, j;

	for(i=0;i<1000;i++)
	for(j=0;j<1000;j++);
}

/*
 * high speed test function
 */
int udc_hs_test_mode (int mode_type)
{
	CONTROL_REG ctrl;
	int i;

	ISP_DBG("\r\nISP: udc_hs_test_mode, %d\r\n",mode_type);

	isp_set_addressed = 0;
	g_hs_test_packet = 1;

	switch (mode_type)   {
	case 1:  /* Test_J */
		/* endpoint 0 IN */
		ISP_SET_ENDPOINT_INDEX((0 << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);
		ctrl.VALUE = 0;
		ctrl.BITS.STATUS = 1;
		isp_regs->D14_CONTROL_FUNCTION.VALUE = ctrl.VALUE;
		hs_test_delay ();
		isp_regs->D14_TEST_MODE.BITS.FORCEHS = 1;		
		isp_regs->D14_TEST_MODE.BITS.JSTATE = 1;	

		/* wait forever for high speed test */
		LOOP_FOREVER
		break;
	case 2:  /* Test_K */
		/* endpoint 0 IN */
		ISP_SET_ENDPOINT_INDEX((0 << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);
		ctrl.VALUE = 0;
		ctrl.BITS.STATUS = 1;
		isp_regs->D14_CONTROL_FUNCTION.VALUE = ctrl.VALUE;
		hs_test_delay ();
		isp_regs->D14_TEST_MODE.BITS.FORCEHS = 1;		
		isp_regs->D14_TEST_MODE.BITS.KSTATE = 1;		

		/* wait forever for high speed test */
		LOOP_FOREVER
		break;
        case 3:  /* Test_SE0_NAK */
		/* endpoint 0 IN */
		ISP_SET_ENDPOINT_INDEX((0 << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);
		ctrl.VALUE = 0;
		ctrl.BITS.STATUS = 1;
		isp_regs->D14_CONTROL_FUNCTION.VALUE = ctrl.VALUE;
		hs_test_delay ();
		isp_regs->D14_TEST_MODE.BITS.FORCEHS = 1;		
		isp_regs->D14_TEST_MODE.BITS.SE0_NAK = 1;		

		/* wait forever for high speed test */
		LOOP_FOREVER
		break;	
        case 4:  /* Test_Packet */
		/* endpoint 0 IN */
		ISP_SET_ENDPOINT_INDEX((0 << ISP_EPINDEX_EP_OFFSET) | ISP_EPINDEX_IN_DIR);
		ctrl.VALUE = 0;
		ctrl.BITS.STATUS = 1;
		isp_regs->D14_CONTROL_FUNCTION.VALUE = ctrl.VALUE;
		isp_regs->D14_BUFFER_LENGTH = 56;

		for (i = 0; i < (USB20_TEST_PACKET_SIZE / 2); i++) {
			isp_regs->D14_DATA_PORT = isp_hs_test_packet[i]; 
		}

		ctrl.VALUE = isp_regs->D14_CONTROL_FUNCTION.VALUE;
		ctrl.BITS.DSEN = 1;
		isp_regs->D14_CONTROL_FUNCTION.VALUE = ctrl.VALUE;
		hs_test_delay ();
		isp_regs->D14_TEST_MODE.BITS.FORCEHS = 1;
		isp_regs->D14_TEST_MODE.BITS.PRBS = 1;

		/* wait forever for high speed test */
		LOOP_FOREVER
		break;	
        case 5:  /* Test_Force_Enable */
		/* endpoint 0 OUT */
		ISP_SET_ENDPOINT_INDEX(0 << ISP_EPINDEX_EP_OFFSET);
		ctrl.VALUE = 0;
		ctrl.BITS.STATUS = 1;
		isp_regs->D14_CONTROL_FUNCTION.VALUE = ctrl.VALUE;

		/* wait forever for high speed test */
		LOOP_FOREVER
		break;	
        default: 
		break;
    }
    return -EINVAL;
}

