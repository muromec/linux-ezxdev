/*
 * linux/drivers/usbd/bi/tc86c001.c -- USB Device Controller driver. 
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 *
 * By: 
 *      Stuart Lynne <sl@lineo.com>, 
 *      Tom Rushworth <tbr@lineo.com>, 
 *      Bruce Balden <balden@lineo.com>
 *
 * Copyright (C) 2002 Toshiba Corporation
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

/***************************************************************************/

#include <linux/config.h>
#include <linux/module.h>

#include "../usbd-export.h"
#include "../usbd-build.h"
#include "../usbd-module.h"

MODULE_AUTHOR ("sl@lineo.com, tbr@lineo.com, TOSHIBA Corporation");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION ("USB Device TC86C001 Bus Interface");

USBD_MODULE_INFO ("tc86c001_bi 0.1-alpha");

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>

#include <asm/atomic.h>
#include <asm/io.h>

#include <linux/netdevice.h>

#include <asm/irq.h>
#include <asm/system.h>

#include <asm/types.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/delay.h>

#include "../usbd.h"
#include "../usbd-debug.h"
#include "../usbd-func.h"
#include "../usbd-bus.h"
#include "../usbd-inline.h"
#include "usbd-bi.h"

#define EP0_PACKETSIZE  0x8

#define UDC_MAX_ENDPOINTS       4

#define UDC_NAME        "TC86C001 USBD"


/* offset 0x000-0x1ff */
struct tc_udc_regs
{
    volatile u32 IntStatus;
    volatile u32 IntEnable;
    volatile u32 MstSetting;
    volatile u32 MstWrStart;
    volatile u32 MstWrEnd;        /* 0x010 */
    volatile u32 MstWrCurr;
    volatile u32 MstRdStart;
    volatile u32 MstRdEnd;
    volatile u32 MstRdCurr;        /* 0x020 */
    volatile u32 PowerDetect;
};

/* offset 0x200-0x7ff */
struct tc_udc_udcregs
{
    volatile u32 EPxFIFO[8];        /* 0x00 */
    volatile u32 EPxMode[8];        /* 0x20 (0:reserved) */
    volatile u32 EPxStatus[8];        /* 0x40 */
    volatile u32 EPxSizeLA[8];        /* 0x60 */
    volatile u32 EPxSizeLB[8];        /* 0x80 (0:reserved) */
    volatile u32 EPxSizeHA[8];        /* 0xa0 (0:reserved) */
    volatile u32 EPxSizeHB[8];        /* 0xc0 (0:reserved) */
    volatile u32 unused1[8];
    volatile u32 bmReqType;        /* 0x100 */
    volatile u32 bRequest;
    volatile u32 wValueL;
    volatile u32 wValueH;
    volatile u32 wIndexL;
    volatile u32 wIndexH;
    volatile u32 wLengthL;
    volatile u32 wLengthH;
    volatile u32 SetupRecv;        /* 0x120 */
    volatile u32 CurrConfig;
    volatile u32 StdRequest;
    volatile u32 Request;
    volatile u32 DataSet[2];
    volatile u32 UsbState;
    volatile u32 EOP;
    volatile u32 Command;        /* 0x140 */
    volatile u32 EPxSingle[2];
    volatile u32 EPxBCS[2];
    volatile u32 unused2;
    volatile u32 IntControl;
    volatile u32 unused3;
    volatile u32 StdReqMode;        /* 0x160 */
    volatile u32 ReqMode;
    volatile u32 unused4[6];
    volatile u32 PortStatus;        /* 0x180 */
    volatile u32 unused5[2];
    volatile u32 Address;
    volatile u32 BuffTest;
    volatile u32 unused6;
    volatile u32 UsbReady;
    volatile u32 unused7;
    volatile u32 SetDescStall;        /* 0x1a0 */
};

/* MstSetting.MST_connection == 0 */
#define UDC_MSTWR_ENDPOINT        1
#define UDC_MSTRD_ENDPOINT        2

#define UDC_DESCRAM_SIZE        0x80

/* offset 0x800-0x9ff */
struct tc_udc_ramregs
{
    volatile u32 DescRam[UDC_DESCRAM_SIZE];
};

/* IntStatus/IntEnable */
#define INT_SUSPEND        0x00001
#define INT_USBRESET        0x00002
#define INT_ENDPOINT0        0x00004
#define INT_SETUP        0x00008
#define INT_STATUS        0x00010
#define INT_STATUSNAK        0x00020
#define INT_EP1DATASET        0x00040
#define INT_EP2DATASET        0x00080
#define INT_EP3DATASET        0x00100
#define INT_EPxDATASET(n)        (0x00040 << ((n) - 1))
#define INT_EP1NAK        0x00200
#define INT_EP2NAK        0x00400
#define INT_EP3NAK        0x00800
#define INT_EPnNAK(n)        (0x00200 < ((n) - 1))
#define INT_SOF        0x01000
#define INT_ERR        0x02000
#define INT_MSTWRSET        0x04000
#define INT_MSTWREND        0x08000
#define INT_MSTWRTMOUT        0x10000
#define INT_MSTRDEND        0x20000
#define INT_SYSERROR        0x40000
#define INT_PWRDETECT        0x80000

/* MstSetting */
#define MST_EOPB_DIS        0x0800
#define MST_EOPB_ENA        0x0400
#define MST_TIMEOUT_DIS        0x0200
#define MST_TIMEOUT_ENA        0x0100
#define MST_RD_EOPB        0x0080
#define MST_RD_RESET        0x0040
#define MST_WR_RESET        0x0020
#define MST_RD_ENA        0x0004        /* 1:start, 0:ignore */
#define MST_WR_ENA        0x0002        /* 1:start, 0:ignore */
#define MST_CONNECTION        0x0001
#define MST_RW_BITS        0x0f67        /* read/write bits */

/* PowerDetect */
#define PW_DETECT        0x04
#define PW_RESETB        0x02
#define PW_PLLUPENB        0x01

/* udcregs StdRequest/StdReqMode */
#define STDREQ_S_INTERFACE        0x80
#define STDREQ_G_INTERFACE        0x40
#define STDREQ_S_CONFIG                0x20
#define STDREQ_G_CONFIG                0x10
#define STDREQ_G_DESCRIPT        0x08
#define STDREQ_S_FEATURE        0x04
#define STDREQ_C_FEATURE        0x02
#define STDREQ_G_STATUS                0x01

/* udcregs EPxStatus */
#define EPxSTATUS_TOGGLE        0x40
#define EPxSTATUS_SUSPEND        0x20
#define EPxSTATUS_EP_MASK        0x1c
#define EPxSTATUS_EP_READY        0x00
#define EPxSTATUS_EP_DATAIN        0x04
#define EPxSTATUS_EP_FULL        0x08
#define EPxSTATUS_EP_TX_ERR        0x0c
#define EPxSTATUS_EP_RX_ERR        0x10
#define EPxSTATUS_EP_BUSY        0x14
#define EPxSTATUS_EP_STALL        0x18
#define EPxSTATUS_EP_INVALID        0x1c
#define EPxSTATUS_FIFO_DISABLE        0x02
#define EPxSTATUS_STAGE_ERROR        0x01

/* udcregs Command */
#define COMMAND_SETDATA0        2
#define COMMAND_RESET        3
#define COMMAND_STALL        4
#define COMMAND_INVALID        5
#define COMMAND_EP(n)        ((n) << 4)

/* udcregs UsbState */
#define USBSTATE_CONFIGURED        0x04
#define USBSTATE_ADDRESSED        0x02
#define USBSTATE_DEFAULT        0x01

static struct pci_dev *udc_pci_dev;        /* only one instance */
static struct tc_udc_regs *tc_regs;
static struct tc_udc_udcregs *tc_udcregs;
static struct tc_udc_ramregs *tc_ramregs;

static struct usb_device_instance *udc_device;        // required for the interrupt handler

/*
 * ep_endpoints - map physical endpoints to logical endpoints
 */
static struct usb_endpoint_instance *ep_endpoints[UDC_MAX_ENDPOINTS];
static dma_addr_t ep_dmaaddrs[UDC_MAX_ENDPOINTS];

static struct urb *ep0_urb;

extern unsigned int udc_interrupts;

static int nodma;
MODULE_PARM (nodma, "i");
MODULE_PARM_DESC (nodma, "Disable DMA");

/* *********************************************************************** */
/* IO
 */

/**
 * tc_write_buffer - write a buffer to the tc fifo
 * @ep: endpoint
 * @b: pointer to buffer to write
 * @size: number of bytes to write
 */
static void
tc_write_buffer (unsigned char ep, unsigned char *b, unsigned char size)
{
    while (size--) {
        writel (*b++, &tc_udcregs->EPxFIFO[ep]);
    }
}

/**
 * tc_read_buffer - fill a buffer from the tc fifo
 * @ep: endpoint
 * @b: pointer to buffer to fill
 * @size: number of bytes to read
 */
static void
tc_read_buffer (unsigned char ep, unsigned char *b, unsigned char size)
{
    while (size--) {
        *b++ = readl (&tc_udcregs->EPxFIFO[ep]);
    }
}

/**
 * tc_read_epxsize - read data count in FIFO of the endpoint.
 * @ep: endpoint
 */
static int
tc_read_epxsize (unsigned int ep)
{
    int low = readl (&tc_udcregs->EPxSizeLA[ep]);
    int size;
    /* SizeH: DATA[9:7], SizeH: DATA[6:0] */
    if (low & 0x80) {
        size = ((readl (&tc_udcregs->EPxSizeHA[ep]) & 0x03) << 7) |
            (low & 0x7f);
    } else {
        low = readl (&tc_udcregs->EPxSizeLB[ep]);
        size = ((readl (&tc_udcregs->EPxSizeHB[ep]) & 0x03) << 7) |
            (low & 0x7f);
    }
    return size;
}

static void
copy_descram (const void *data, int len, int *ofs)
{
    const unsigned char *p = (const unsigned char *) data;
    int i;
    if (*ofs <= UDC_DESCRAM_SIZE && *ofs + len > UDC_DESCRAM_SIZE)
        printk (KERN_ERR UDC_NAME ": too big descriptor\n");
    if (*ofs + len <= UDC_DESCRAM_SIZE) {
        for (i = 0; i < len; i++)
            writel (*p++, &tc_ramregs->DescRam[(*ofs) + i]);
    }
    *ofs += len;
}

/* *********************************************************************** */
/* Control (endpoint zero)
 */

/**
 * tc_in_ep0 - start transmit
 * @ep:
 */
static void
tc_in_ep0 (struct usb_endpoint_instance *endpoint)
{
    if (!endpoint)
        return;
    if (endpoint->tx_urb) {
        struct urb *urb = endpoint->tx_urb;
        dbg_ep0 (2, "length: %d sent %d",
            endpoint->tx_urb->actual_length, endpoint->sent);

        if ((urb->actual_length - endpoint->sent) > 0) {
            if (readl (&tc_udcregs->DataSet[0]) & 1) {
                printk (KERN_ERR UDC_NAME ": ep0 FIFO not empty\n");
                return;
            }
            endpoint->last =
                min_t (int, urb->actual_length - endpoint->sent,
                endpoint->tx_packetSize);
            tc_write_buffer (0, urb->buffer + endpoint->sent, endpoint->last);
            if (endpoint->last < endpoint->tx_packetSize) {
                dbg_ep0 (2, "EOP");
                writel (~1, &tc_udcregs->EOP);
            }
        } else {
            // XXX ZLP
            endpoint->last = 0;
            dbg_ep0 (2, "EOP");
            writel (~1, &tc_udcregs->EOP);
        }
    } else if (endpoint->last == 0 ||
        endpoint->last == endpoint->tx_packetSize) {
        // XXX ZLP
        dbg_ep0 (2, "EOP (last %d)", endpoint->last);
        endpoint->last = 0;
        writel (~1, &tc_udcregs->EOP);
    }
}
static void
tc_out_ep0 (struct usb_endpoint_instance *endpoint)
{
    printk (KERN_ERR UDC_NAME "tc_out_ep0: not supported.\n");
}

static void
tc_ep0_setup (void)
{
    struct usb_device_request *req = &ep0_urb->device_request;
    u8 *buf = (u8 *) req;
    struct usb_endpoint_instance *endpoint = ep_endpoints[0];

    if (!endpoint) {
        dbg_udc (0, "no endpoint 0");
        return;
    }
    /* store in little endian (see ep0_recv_setup) */
    buf[0] = readl (&tc_udcregs->bmReqType);
    buf[1] = readl (&tc_udcregs->bRequest);
    buf[2] = readl (&tc_udcregs->wValueL);
    buf[3] = readl (&tc_udcregs->wValueH);
    buf[4] = readl (&tc_udcregs->wIndexL);
    buf[5] = readl (&tc_udcregs->wIndexH);
    buf[6] = readl (&tc_udcregs->wLengthL);
    buf[7] = readl (&tc_udcregs->wLengthH);
    writel (0, &tc_udcregs->SetupRecv);
    dbg_ep0 (1,
        "bmRequestType:%02x bRequest:%02x wValue:%04x wIndex:%04x wLength:%04x",
        req->bmRequestType, req->bRequest, le16_to_cpu (req->wValue),
        le16_to_cpu (req->wIndex), le16_to_cpu (req->wLength));

    if (udc_device->device_state == STATE_DEFAULT &&
        ((req->bmRequestType & USB_REQ_DIRECTION_MASK) == USB_REQ_HOST2DEVICE
            || (req->bmRequestType & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS)
        && (readl (&tc_udcregs->Address) & 0xff)) {
        /* first Host-to-Device setup or Class setup packet */
        u8 new_address = readl (&tc_udcregs->Address) & 0xff;
        /* SET_ADDRESS are processed by hardware only */
        dbg_ep0 (1, "address assigned (%x)", new_address);
        if (udc_device->address && udc_device->address != new_address)
            printk (KERN_ERR "address changed\n");
        udc_device->address = new_address;
        usbd_device_event (udc_device, DEVICE_ADDRESS_ASSIGNED, 0);
    }

    if (usbd_recv_setup (ep0_urb)) {
        dbg_ep0 (0, "usb_recv_setup failed, stalling");
        udc_stall_ep (0);
        return;
    }
    // check data direction
    if ((req->bmRequestType & USB_REQ_DIRECTION_MASK) == USB_REQ_HOST2DEVICE) {
        // should we setup to receive data
        if (le16_to_cpu (req->wLength)) {
            dbg_ep0 (1, "setup to read data %d", le16_to_cpu (req->wLength));
            endpoint->rcv_urb = ep0_urb;
            endpoint->rcv_urb->actual_length = 0;
            tc_out_ep0 (endpoint);
            return;
        }
        if (req->bRequest == USB_REQ_SET_CONFIGURATION) {
            switch (req->wValue) {
            case 0:
                /* configured --> addressed */
                usbd_device_event (udc_device, DEVICE_DE_CONFIGURED, 0);
                writel (0, &tc_udcregs->UsbState);
                dbg_udc (1, "UsbState %x", readl (&tc_udcregs->UsbState));
                break;
            default:
                /* addressed --> configured */
                //usbd_device_event(udc_device, DEVICE_CONFIGURED, 0);
                writel (USBSTATE_CONFIGURED, &tc_udcregs->UsbState);
                dbg_udc (1, "UsbState %x", readl (&tc_udcregs->UsbState));
            }
        }
        dbg_ep0 (2, "EOP");
        writel (~1, &tc_udcregs->EOP);
        return;
    }
    // we should be sending data back
    // check request length, zero is not legal
    if (!le16_to_cpu (ep0_urb->device_request.wLength)) {
        dbg_ep0 (0, "wLength zero, stall");
        udc_stall_ep (0);
        return;
    }
    // check that we have some data to send back, zero should not be possible
    if (!ep0_urb->actual_length) {
        dbg_ep0 (0, "no data, stall");
        udc_stall_ep (0);
        return;
    }
    // start sending
    endpoint->tx_urb = ep0_urb;
    endpoint->sent = 0;
    endpoint->last = 0;
    tc_in_ep0 (endpoint);
}

/* *********************************************************************** */
/* Bulk OUT (recv)
 */

static void
tc_mstrd_start (u32 start, u32 len, int eop)
{
    u32 mst = readl (&tc_regs->MstSetting);
    mst &= MST_RW_BITS & ~(MST_WR_ENA | MST_EOPB_ENA | MST_EOPB_DIS);
    mst |= eop ? MST_EOPB_ENA : MST_EOPB_DIS;
    mst |= MST_RD_ENA;
    dbg_tx (2, "mstrd 0x%x 0x%x 0x%x", start, len, mst);
    writel (start, &tc_regs->MstRdStart);
    writel (start + len - 1, &tc_regs->MstRdEnd);
    writel (mst, &tc_regs->MstSetting);
}
static void
tc_mstwr_start (u32 start, u32 len)
{
    u32 mst = readl (&tc_regs->MstSetting);
    mst &= MST_RW_BITS & ~(MST_RD_ENA);
    mst |= MST_WR_ENA;
    dbg_rx (2, "mstwr 0x%x 0x%x 0x%x", start, len, mst);
    writel (start, &tc_regs->MstWrStart);
    writel (start + len - 1, &tc_regs->MstWrEnd);
    writel (mst | MST_WR_ENA, &tc_regs->MstSetting);
}

static unsigned long tc_do_task_ep;
static void tc_start_out (unsigned int ep);
static void
tc_do_task (void *data)                // runs as process
{
    unsigned long flags;
    unsigned int ep;
    if (!udc_device || udc_device->status != USBD_OK)
        return;
    for (ep = 0; ep < UDC_MAX_ENDPOINTS; ep++) {
        if (test_and_clear_bit (ep, &tc_do_task_ep)) {
            local_irq_save (flags);
            while (!ep_endpoints[ep]->rcv_urb &&
                !(ep_endpoints[ep]->rcv_urb =
                    first_urb_detached_irq (&(ep_endpoints[ep]->rdy)))) {
                local_irq_restore (flags);
                /* usbd_recycle_urb will be called via device_bh */
                run_task_queue (&tq_immediate);
                local_irq_save (flags);
            }
            dbg_rx (1, "ep%d rcv urb available.", ep);
            tc_start_out (ep);
            local_irq_restore (flags);
        }
    }
}
static struct tq_struct tc_task = {
    routine:tc_do_task
};

static void
tc_start_out (unsigned int ep)
{
    struct usb_endpoint_instance *endpoint = ep_endpoints[ep];
    struct urb *urb = endpoint->rcv_urb;
    int size;

    if (!urb) {
        dbg_rx (2, "no rcv_urb");
        dbg_rx (1, "ep%d no rcv_urb available.  try again.", ep);
        set_bit (ep, &tc_do_task_ep);
        schedule_task (&tc_task);
        // XXX could we call usbd_fill_rcv() here?
        return;
    }

    if (!nodma && ep == UDC_MSTWR_ENDPOINT) {
        /* DMA endpoint */
        if (urb->actual_length == 0) {
            if (ep_dmaaddrs[ep]) {
                printk (KERN_ERR UDC_NAME ": receive DMA busy.\n");
                //return;
            }
            ep_dmaaddrs[ep] =
                pci_map_single (udc_pci_dev,
                urb->buffer, urb->buffer_length, PCI_DMA_FROMDEVICE);
            dbg_rx (2, "rx dmaaddr(%d) %x", ep, ep_dmaaddrs[ep]);
            if (!ep_dmaaddrs[ep]) {
                printk (KERN_ERR UDC_NAME ": pci_map_single failed.\n");
                usbd_rcv_complete_irq (endpoint, 0, 1);
                return;
            }
        }
        /* start DMA (master write) */
        /* check short packet without following three SOFs... */
        size = tc_read_epxsize (ep);
        if (size == endpoint->rcv_packetSize) {
            /* fulll packet: transfer continuous packets */
            size = min ((int) (urb->buffer_length - urb->actual_length),
                endpoint->rcv_transferSize);
        }
        tc_mstwr_start (ep_dmaaddrs[ep] + urb->actual_length, size);
        return;
    }
    /* check EPx_DSETA */
    if (!(readl (&tc_udcregs->DataSet[0]) & (1 << (ep * 2)))) {
        printk (KERN_ERR UDC_NAME ": ep%d FIFO empty\n", ep);
        return;
    }
    size = tc_read_epxsize (ep);
    if (size == 0) {
        printk (KERN_ERR UDC_NAME ": ep%d SIZE 0\n", ep);
        return;
    }
    dbg_rx (2, "ep%d rx size 0x%x (status %x)",
        ep, size, readl (&tc_udcregs->EPxStatus[ep]));
    tc_read_buffer (ep, urb->buffer + urb->actual_length, size);
    usbd_rcv_complete_irq (endpoint, size, 0);
}

static void
tc_end_out (unsigned int ep, int tout)
{
    struct usb_endpoint_instance *endpoint = ep_endpoints[ep];
    struct urb *urb = endpoint->rcv_urb;
    int size;
    int org_buffer_length;

    if (!ep_dmaaddrs[ep] || !urb) {
        dbg_rx (0, "not mstwr");
        return;
    }
    size = readl (&tc_regs->MstWrCurr) + 1 - ep_dmaaddrs[ep];
    dbg_rx (2, "mstwr end (size 0x%x tout %d)", size, tout);
    org_buffer_length = urb->buffer_length;
    while (size > 0) {
        int len = min (size, endpoint->rcv_packetSize);
        if (endpoint->rcv_urb != urb) {
            dbg_rx (0, "BUG: rcv_urb consumed!!!");
            break;
        }
        usbd_rcv_complete_irq (endpoint, len, 0);
        size -= len;
        if (size == 0 && len == endpoint->rcv_packetSize &&
            endpoint->rcv_urb == urb) {
            /* null packet (send current rcv_urb to function driver) */
            usbd_rcv_complete_irq (endpoint, 0, 0);
        }
    }
    if (endpoint->rcv_urb != urb) {
        pci_unmap_single (udc_pci_dev, ep_dmaaddrs[ep], org_buffer_length,
            PCI_DMA_FROMDEVICE);
        ep_dmaaddrs[ep] = 0;
    }
}

/* *********************************************************************** */
/* Bulk IN (tx)
 */

/**
 * start_in - start transmit
 * @ep:
 */
static void
tc_start_in (unsigned int ep)
{
    struct usb_endpoint_instance *endpoint = ep_endpoints[ep];
    struct urb *urb = endpoint->tx_urb;

    if (!nodma) {
        if (ep_dmaaddrs[ep]) {
            dbg_tx (0, "ep%d DMA busy", ep);
            return;
        }
        if (readl (&tc_regs->MstSetting) & MST_RD_ENA) {
            dbg_tx (0, "ep%d DMARD busy", ep);
            return;
        }
    }
    usbd_tx_complete_irq (endpoint, 0);

    /* send next urb */
    if ((urb = endpoint->tx_urb) != NULL) {
        if (!nodma && ep == UDC_MSTRD_ENDPOINT) {
            /* start DMA (master read) */
            if (endpoint->sent == 0) {
                ep_dmaaddrs[ep] =
                    pci_map_single (udc_pci_dev,
                    urb->buffer, urb->buffer_length, PCI_DMA_TODEVICE);
                dbg_tx (2, "tx dmaaddr(%d) %x", ep, ep_dmaaddrs[ep]);
                if (!ep_dmaaddrs[ep]) {
                    printk (KERN_ERR UDC_NAME ": pci_map_single failed.\n");
                    endpoint->last = urb->actual_length;
                    usbd_tx_complete_irq (endpoint, 0);
                    return;
                }
            }
            /* invoke master read for whole urb buffer */
            endpoint->last = urb->actual_length - endpoint->sent;
            if (endpoint->last == 0) {
                /* send NULL packet */
                dbg_tx (2, "ep%d EOPB", ep);
                writel ((readl (&tc_regs->
                            MstSetting) & MST_RW_BITS) | MST_RD_EOPB,
                    &tc_regs->MstSetting);
                while (readl (&tc_regs->MstSetting) & MST_RD_EOPB);
            } else {
                /* disable EOP if transfersize is multiple of packetsize */
                int eop = (endpoint->last % endpoint->tx_packetSize != 0);
                tc_mstrd_start (ep_dmaaddrs[ep] + endpoint->sent,
                    endpoint->last, eop);
            }
            return;
        }
        /* check EPx_DSETA */
        if (readl (&tc_udcregs->DataSet[0]) & (1 << (ep * 2))) {
            printk (KERN_ERR UDC_NAME ": ep%d FIFO not empty"
                " (urb length %d sent %d)\n",
                ep, urb->actual_length, endpoint->sent);
            return;
        }
        if ((urb->actual_length - endpoint->sent) > 0) {
            endpoint->last =
                min_t (int, urb->actual_length - endpoint->sent,
                endpoint->tx_packetSize);
            dbg_tx (2, "ep%d tx size 0x%x", ep, endpoint->last);
            tc_write_buffer (ep, urb->buffer + endpoint->sent,
                endpoint->last);
            if (endpoint->last < endpoint->tx_packetSize) {
                dbg_tx (2, "EOP %d", ep);
                writel (~(1 << ep), &tc_udcregs->EOP);
            }
        } else {
            // XXX ZLP
            endpoint->last = 0;
            dbg_tx (2, "EOP %d", ep);
            writel (~(1 << ep), &tc_udcregs->EOP);
        }
    } else if (nodma && (endpoint->last == endpoint->tx_packetSize)) {
        // XXX ZLP
        dbg_tx (2, "EOP %d (last %d)", ep, endpoint->last);
        endpoint->last = 0;
        writel (~(1 << ep), &tc_udcregs->EOP);
    }
}

static void
tc_end_in (unsigned int ep)
{
    struct usb_endpoint_instance *endpoint = ep_endpoints[ep];
    struct urb *urb = endpoint->tx_urb;

    if (!ep_dmaaddrs[ep] || !urb)
        return;
    dbg_tx (2, "mstrd end (size 0x%x)",
        readl (&tc_regs->MstRdCurr) + 1 - ep_dmaaddrs[ep]);
    if (endpoint->sent + endpoint->last >= urb->actual_length) {
        pci_unmap_single (udc_pci_dev, ep_dmaaddrs[ep], urb->buffer_length,
            PCI_DMA_TODEVICE);
        ep_dmaaddrs[ep] = 0;
    }
    /* complete current tx packet and start next transmit (if exists) */
    tc_start_in (ep);
}

/* *********************************************************************** */
/* Interrupt Handler
 */

static void
tc_setup_descram_if (struct usb_device_instance *device,
    int port, int nconf, int nint, int *ofs)
{
    struct usb_interface_instance *iinst;
    int nalt;
    iinst = usbd_device_interface_instance (device, port, nconf, nint);
    if (!iinst)
        return;
    for (nalt = 0; nalt < iinst->alternates; nalt++) {
        struct usb_interface_descriptor *idesc;
        struct usb_alternate_instance *ainst;
        int nclass, nep;
        ainst = usbd_device_alternate_instance (device, port,
            nconf, nint, nalt);
        if (!ainst)
            break;
        idesc = ainst->interface_descriptor;
        copy_descram (idesc, sizeof (*idesc), ofs);
        for (nclass = 0; nclass < ainst->classes; nclass++) {
            struct usb_class_descriptor *cldesc;
            cldesc = usbd_device_class_descriptor_index (device,
                port, nconf, nint, nalt, nclass);
            if (!cldesc)
                break;
            copy_descram (cldesc, cldesc->descriptor.generic.bFunctionLength,
                ofs);
        }
        for (nep = 0; nep < ainst->endpoints; nep++) {
            struct usb_endpoint_descriptor *edesc;
            edesc = usbd_device_endpoint_descriptor_index (device,
                port, nconf, nint, nalt, nep);
            if (!edesc)
                break;
            copy_descram (edesc, sizeof (*edesc), ofs);
        }
    }
}

static void
tc_setup_descram_str (int *ofs)
{
    u8 bLength[4] = { 0, 0, 0, 0 };
    copy_descram (&bLength, 4, ofs);
}

static void
tc_setup_descram (void)
{
    struct usb_device_descriptor *ddesc;
    int ofs = 0, nconf;
    int i;

    ddesc = usbd_device_device_descriptor (udc_device, 0);
    if (!ddesc)
        return;
    copy_descram (ddesc, sizeof (*ddesc), &ofs);
    for (nconf = 0; nconf < ddesc->bNumConfigurations; nconf++) {
        struct usb_configuration_descriptor *cdesc;
        int nint;
        cdesc = usbd_device_configuration_descriptor (udc_device, 0, nconf);
        if (!cdesc)
            break;
        copy_descram (cdesc, sizeof (*cdesc), &ofs);
        for (nint = 0; nint < cdesc->bNumInterfaces; nint++) {
            tc_setup_descram_if (udc_device, 0, nconf, nint, &ofs);
        }
    }

    tc_setup_descram_str (&ofs);

    for (i = ofs; i < UDC_DESCRAM_SIZE; i++)
        writel (0, &tc_ramregs->DescRam[i]);

    for (i = 0; i < ofs; i += 8)
        dbg_udc (2, "Descram: %02x %02x %02x %02x %02x %02x %02x %02x",
            readl (&tc_ramregs->DescRam[i + 0]),
            readl (&tc_ramregs->DescRam[i + 1]),
            readl (&tc_ramregs->DescRam[i + 2]),
            readl (&tc_ramregs->DescRam[i + 3]),
            readl (&tc_ramregs->DescRam[i + 4]),
            readl (&tc_ramregs->DescRam[i + 5]),
            readl (&tc_ramregs->DescRam[i + 6]),
            readl (&tc_ramregs->DescRam[i + 7]));
}

static void
tc_power_detected (void)
{
    int i;
    u8 bcs, single;

    if (readl (&tc_regs->PowerDetect) & PW_DETECT) {
        /* power detected */
        dbg_udc (1, "Power detected");
        usbd_device_event_irq (udc_device, DEVICE_HUB_CONFIGURED, 0);

        /* assert reset */
        writel (0, &tc_regs->PowerDetect);
        readl (&tc_regs->IntStatus);        /* wbflush */
        udelay (1);
        /* deassert reset (USB_RESET still disabled) */
        writel (MST_EOPB_ENA | MST_TIMEOUT_ENA, &tc_regs->MstSetting);
        writel (PW_RESETB, &tc_regs->PowerDetect);
        writel (STDREQ_S_CONFIG | STDREQ_G_CONFIG | STDREQ_G_DESCRIPT,
            &tc_udcregs->StdReqMode);
        /* EP[2:1]:DMA access, EP[3]:CPU access (single) */
        bcs = single = 0;
        for (i = 1; i < 4; i++) {
            single |= 0x11 << i;
            bcs |= ((i == 3 || nodma) ? 0x11 /*CPU*/ : 0x10 /*DMA*/) << i;
        }
        writel (single, &tc_udcregs->EPxSingle[0]);
        writel (bcs, &tc_udcregs->EPxBCS[0]);
        tc_setup_descram ();
        /* enable USB_RESET detection */
        writel (0, &tc_udcregs->UsbReady);
    } else {
        /* power down */
        dbg_udc (1, "Power down");
        usbd_device_event_irq (udc_device, DEVICE_RESET, 0);
        usbd_device_event_irq (udc_device, DEVICE_POWER_INTERRUPTION, 0);
        usbd_device_event_irq (udc_device, DEVICE_HUB_RESET, 0);
    }
}

/**
 * int_hndlr - interrupt handler
 *
 */
static void
tc_int_hndlr (int irq, void *dev_id, struct pt_regs *regs)
{
    int count, i;
    u32 stat, mask;
    udc_interrupts++;

    for (count = 0; count < 10; count++) {
        stat = readl (&tc_regs->IntStatus);
        mask = readl (&tc_regs->IntEnable);
        if ((stat & mask) == 0)
            break;
        dbg_udc (3, "stat %x mask %x", stat, mask);
        stat &= mask;
        if (stat & INT_ERR) {
            writel (~INT_ERR, &tc_regs->IntStatus);
            printk (KERN_ERR UDC_NAME ": ERR interrupt.\n");
            continue;
        }
        if (stat & INT_SYSERROR) {
            writel (~INT_SYSERROR, &tc_regs->IntStatus);
            printk (KERN_ERR UDC_NAME ": SYSERROR interrupt.\n");
            continue;
        }
        if (stat & INT_PWRDETECT) {
            writel (~INT_PWRDETECT, &tc_regs->IntStatus);
            if (!udc_device)
                continue;
            //udc_cable_event();
            tc_power_detected ();
            continue;
        }
        if (stat & INT_USBRESET) {
            writel (~INT_USBRESET, &tc_regs->IntStatus);
            if (!udc_device)
                continue;
            /* USBRESET received */
            usbd_device_event_irq (udc_device, DEVICE_RESET, 0);
            /* enable pull-up */
            dbg_udc (1, "Enabling pullup");
            writel (PW_RESETB | PW_PLLUPENB, &tc_regs->PowerDetect);
            continue;
        }
        if (stat & INT_SUSPEND) {
            writel (~INT_SUSPEND, &tc_regs->IntStatus);
            if (!udc_device)
                continue;
#if 0
            if (udc_device->device_state == STATE_UNKNOWN ||
                udc_device->device_state == STATE_INIT ||
                udc_device->device_state == STATE_ATTACHED)
                continue;
#endif
            if (readl (&tc_udcregs->EPxStatus[0]) & EPxSTATUS_SUSPEND) {
                dbg_udc (1, "Suspend");
                usbd_device_event (udc_device, DEVICE_BUS_INACTIVE, 0);
            } else {
                dbg_udc (1, "Resume");
                usbd_device_event (udc_device, DEVICE_BUS_ACTIVITY, 0);
            }
            continue;
        }
        if (stat & INT_SETUP) {
            dbg_udc (1, "Setup");
            writel (~INT_SETUP, &tc_regs->IntStatus);
            tc_ep0_setup ();
            continue;
        }
        if (stat & INT_ENDPOINT0) {
            writel (~INT_ENDPOINT0, &tc_regs->IntStatus);
            usbd_tx_complete_irq (ep_endpoints[0], 0);
            tc_in_ep0 (ep_endpoints[0]);
            // XXX how about control write?
            continue;
        }
        if (stat & INT_MSTWRTMOUT) {
            writel (~INT_MSTWRTMOUT, &tc_regs->IntStatus);
            tc_end_out (UDC_MSTWR_ENDPOINT, 1);
            continue;
        }
        if (stat & INT_MSTWREND) {
            writel (~INT_MSTWREND, &tc_regs->IntStatus);
            tc_end_out (UDC_MSTWR_ENDPOINT, 0);
            continue;
        }
        if (stat & INT_MSTWRSET) {
            writel (~INT_MSTWRSET, &tc_regs->IntStatus);
            /* start master write */
            tc_start_out (UDC_MSTWR_ENDPOINT);
            continue;
        }
        if (stat & INT_MSTRDEND) {
            writel (~INT_MSTRDEND, &tc_regs->IntStatus);
            tc_end_in (UDC_MSTRD_ENDPOINT);
            continue;
        }
        for (i = 1; i < 4; i++) {
            if (stat & INT_EPxDATASET (i)) {
                writel (~INT_EPxDATASET (i), &tc_regs->IntStatus);
                if (!ep_endpoints[i])
                    continue;
                if (ep_endpoints[i]->endpoint_address & USB_DIR_IN) {
                    tc_start_in (i);
                } else {
                    tc_start_out (i);
                }
            }
        }
    }
}


/* *********************************************************************** */

static int __devinit
udc_probe (struct pci_dev *pdev, const struct pci_device_id *ent)
{
    void *mmio;

    if (udc_pci_dev) {
        /* multiple instance is not supported */
        return -ENODEV;
    }
    /* make sure PCI base addr 0 is MMIO */
    if (!(pci_resource_flags (pdev, 0) & IORESOURCE_MEM)) {
        printk (KERN_ERR UDC_NAME
            ": region #0 not an MMIO resource, aborting\n");
        return -ENODEV;
    }
    if (request_irq (pdev->irq, &tc_int_hndlr,
            SA_SHIRQ | SA_SAMPLE_RANDOM, UDC_NAME, pdev)) {
        printk (KERN_ERR UDC_NAME " could not request irq %d\n", pdev->irq);
        return -EBUSY;
    }
    if (pci_request_regions (pdev, UDC_NAME)) {
        printk (KERN_ERR UDC_NAME " could not request regions\n");
        free_irq (pdev->irq, pdev);
        return -EBUSY;
    }
    pci_set_master (pdev);

    mmio = ioremap (pci_resource_start (pdev, 0), pci_resource_len (pdev, 0));
    if (mmio == NULL) {
        printk (KERN_ERR UDC_NAME " could not remap 0x%08lx(0x%lx)\n",
            pci_resource_start (pdev, 0), pci_resource_len (pdev, 0));
        free_irq (pdev->irq, pdev);
        pci_release_regions (pdev);
        return -EBUSY;
    }
    tc_regs = mmio;
    tc_udcregs = (struct tc_udc_udcregs *) (mmio + 0x200);
    tc_ramregs = (struct tc_udc_ramregs *) (mmio + 0x800);

    printk (KERN_DEBUG UDC_NAME " found at %p irq %d (%s)\n",
        mmio, pdev->irq, nodma ? "PIO" : "DMA");

    udc_pci_dev = pdev;
    return 0;
}

static void __devexit
udc_remove (struct pci_dev *pdev)
{
    if (tc_regs)
        iounmap (tc_regs);
    pci_release_regions (pdev);
    free_irq (pdev->irq, pdev);
    tc_regs = NULL;
    tc_udcregs = NULL;
    tc_ramregs = NULL;
    udc_pci_dev = NULL;
}


static struct pci_device_id udc_pci_tbl[] __devinitdata = {
    {PCI_VENDOR_ID_TOSHIBA_2, PCI_DEVICE_ID_TOSHIBA_TC86C001_USBD, PCI_ANY_ID,
            PCI_ANY_ID, 0, 0, 0},
    {0,}
};

MODULE_DEVICE_TABLE (pci, udc_pci_tbl);
static struct pci_driver udc_pci_driver = {
    name:UDC_NAME,
    id_table:udc_pci_tbl,
    probe:udc_probe,
    remove:__devexit_p (udc_remove),
};


/* *********************************************************************** */
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
udc_start_in_irq (struct usb_endpoint_instance *endpoint)
{
    int i;
    for (i = 1; i < UDC_MAX_ENDPOINTS; i++)
        if (ep_endpoints[i] == endpoint)
            tc_start_in (i);
}

/**
 * udc_init - initialize
 *
 * Return non-zero if we cannot see device.
 **/

int
udc_init (void)
{
    return pci_module_init (&udc_pci_driver);
}


#if 0                                /* not used */
/**
 * udc_start_in - start transmit
 * @eendpoint: endpoint instance
 *
 * Called by bus interface driver to see if we need to start a data transmission.
 */
void
udc_start_in (struct usb_endpoint_instance *endpoint)
{
}
#endif


/**
 * udc_stall_ep - stall endpoint
 * @ep: physical endpoint
 *
 * Stall the endpoint.
 */
void
udc_stall_ep (unsigned int ep)
{
    if (ep < UDC_MAX_ENDPOINTS) {
        writel (COMMAND_EP (ep) | COMMAND_STALL, &tc_udcregs->Command);
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
udc_reset_ep (unsigned int ep)
{
    if (ep < UDC_MAX_ENDPOINTS) {
        writel (COMMAND_EP (ep) | COMMAND_RESET, &tc_udcregs->Command);
    }
}


#if 0                                /* not used */
/**
 * udc_endpoint_halted - is endpoint halted
 * @ep:
 *
 * Return non-zero if endpoint is halted
 */
int
udc_endpoint_halted (unsigned int ep)
{
    return 0;
}
#endif


/**
 * udc_set_address - set the USB address for this device
 * @address:
 *
 * Called from control endpoint function after it decodes a set address setup packet.
 */
void
udc_set_address (unsigned char address)
{
    // address cannot be setup until ack received
    /* tc_udcregs->Address register is read-only */
}

/**
 * udc_serial_init - set a serial number if available
 */
int __init
udc_serial_init (struct usb_bus_instance *bus)
{
    return -EINVAL;
}

/* *********************************************************************** */

/**
 * udc_max_endpoints - max physical endpoints 
 *
 * Return number of physical endpoints.
 */
int
udc_max_endpoints (void)
{
    return UDC_MAX_ENDPOINTS;
}


/**
 * udc_check_ep - check logical endpoint 
 * @lep:
 *
 * Return physical endpoint number to use for this logical endpoint or zero if not valid.
 */
int
udc_check_ep (int logical_endpoint, int packetsize)
{
    int ep = logical_endpoint & 0xf;
    if (ep >= UDC_MAX_ENDPOINTS || packetsize > 64)
        return 0;
    /* ASSUMPTION: MstSetting.MST_connection == 0 */
    /* MSTWR endpoint must be OUT */
    if (ep == UDC_MSTWR_ENDPOINT && (logical_endpoint & USB_DIR_IN))
        return 0;
    /* MSTRD endpoint must be IN */
    if (ep == UDC_MSTRD_ENDPOINT && !(logical_endpoint & USB_DIR_IN))
        return 0;
    return ep;
}


/**
 * udc_set_ep - setup endpoint 
 * @ep:
 * @endpoint:
 *
 * Associate a physical endpoint with endpoint_instance
 */
void
udc_setup_ep (struct usb_device_instance *device, unsigned int ep,
    struct usb_endpoint_instance *endpoint)
{
    if (ep < UDC_MAX_ENDPOINTS) {
        ep_endpoints[ep] = endpoint;
        dbg_udc (1, "endpoint %d %p", ep, endpoint);
        // ep0
        if (ep == 0) {
        }
        // IN
        else if (endpoint->endpoint_address & 0x80) {
            int mode = endpoint->tx_attributes & USB_ENDPOINT_XFERTYPE_MASK;
            int payload = 0;
            while (8 << payload < endpoint->tx_packetSize)
                payload++;
            writel ((payload << 3) | (mode << 1) | 1,
                &tc_udcregs->EPxMode[ep]);
            dbg_udc (1, "EP%dMode %x", ep, readl (&tc_udcregs->EPxMode[ep]));
        }
        // OUT
        else if (endpoint->endpoint_address) {
            int mode = endpoint->rcv_attributes & USB_ENDPOINT_XFERTYPE_MASK;
            int payload = 0;
            while (8 << payload < endpoint->rcv_packetSize)
                payload++;
            writel ((payload << 3) | (mode << 1) | 0,
                &tc_udcregs->EPxMode[ep]);
            if (nodma) {
                int payload_size = (8 << payload);
                if (endpoint->rcv_transferSize % payload_size == 0 &&
                    endpoint->rcv_transferSize > payload_size)
                    endpoint->rcv_transferSize = payload_size;
                dbg_rx (1, "ep%d rcv_transferSize %d", ep, payload_size);
            }
            usbd_fill_rcv (device, endpoint, 64);        /* increased for storage */
            endpoint->rcv_urb = first_urb_detached (&endpoint->rdy);
            dbg_udc (1, "EP%dMode %x", ep, readl (&tc_udcregs->EPxMode[ep]));
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
udc_disable_ep (unsigned int ep)
{
    if (ep < UDC_MAX_ENDPOINTS) {
        struct usb_endpoint_instance *endpoint;

        if ((endpoint = ep_endpoints[ep])) {
            dbg_udc (1, "disabling endpoint %d %p", ep, endpoint);
            ep_endpoints[ep] = NULL;
            usbd_flush_ep (endpoint);
        }
    }
}

/* *********************************************************************** */

/**
 * udc_connected - is the USB cable connected
 *
 * Return non-zeron if cable is connected.
 */
int
udc_connected ()
{
    return readl (&tc_regs->PowerDetect) & PW_DETECT;
}


/**
 * udc_connect - enable pullup resistor
 *
 * Turn on the USB connection by enabling the pullup resistor.
 */
void
udc_connect (void)
{
    /* pullup will enabled on INT_USBRESET interrupt */
}


/**
 * udc_disconnect - disable pullup resistor
 *
 * Turn off the USB connection by disabling the pullup resistor.
 */
void
udc_disconnect (void)
{
    dbg_udc (1, "Disabling pullup");
    writel (PW_RESETB, &tc_regs->PowerDetect);
}

/* *********************************************************************** */

/**
 * udc_all_interrupts - enable interrupts
 *
 * Switch on UDC interrupts.
 *
 */
void
udc_all_interrupts (struct usb_device_instance *device)
{
    // set interrupt mask
    u32 val = INT_PWRDETECT | INT_SYSERROR |
        INT_ERR | INT_SETUP |
        INT_EPxDATASET (3) | INT_ENDPOINT0 | INT_USBRESET | INT_SUSPEND;
    /* ASSUMPTION: MstSetting.MST_connection == 0 */
    /* endpoint 1 must be OUT */
    if (nodma)
        val |= INT_EPxDATASET (1) | INT_EPxDATASET (2);
    else
        val |= INT_MSTWRSET | INT_MSTWREND | INT_MSTWRTMOUT | INT_MSTRDEND;
    writel (val, &tc_regs->IntEnable);
    dbg_udc (1, "udc_enable_interrupts");
}


/**
 * udc_suspended_interrupts - enable suspended interrupts
 *
 * Switch on only UDC resume interrupt.
 *
 */
void
udc_suspended_interrupts (struct usb_device_instance *device)
{
    dbg_udc (1, "udc_suspended_interrupts");
    // set interrupt mask
    writel (INT_SUSPEND | INT_PWRDETECT | INT_USBRESET, &tc_regs->IntEnable);
}


/**
 * udc_disable_interrupts - disable interrupts.
 *
 * switch off interrupts
 */
void
udc_disable_interrupts (struct usb_device_instance *device)
{
    printk (KERN_DEBUG "udc_disable_interrupts:\n");
    // reset interrupt mask
    writel (0, &tc_regs->IntEnable);
    tc_do_task_ep = 0;
}

/* *********************************************************************** */

/**
 * udc_ep0_packetsize - return ep0 packetsize
 */
int
udc_ep0_packetsize (void)
{
    return EP0_PACKETSIZE;
}

/**
 * udc_enable - enable the UDC
 *
 * Switch on the UDC
 */
void
udc_enable (struct usb_device_instance *device)
{
    int i;

    dbg_udc (1, "enable");
    // save the device structure pointer
    udc_device = device;

    // ep0 urb
    if (!ep0_urb) {
        ep0_urb =
            usbd_alloc_urb (device, device->function_instance_array, 0, 512);
        if (!ep0_urb) {
            printk (KERN_ERR "ep0_enable: usbd_alloc_urb failed\n");
        }
    } else {
        printk (KERN_ERR "udc_enable: ep0_urb already allocated\n");
    }

    for (i = 0; i < UDC_MAX_ENDPOINTS; i++)
        ep_dmaaddrs[i] = 0;
    // enable UDC

    /* reset and initialize */
    writel (0, &tc_regs->PowerDetect);
    writel (0, &tc_regs->IntEnable);
    /* need to read tc registers... */
    writel (PW_RESETB, &tc_regs->PowerDetect);

    tc_power_detected ();
}


/**
 * udc_disable - disable the UDC
 *
 * Switch off the UDC
 */
void
udc_disable (void)
{
    dbg_udc (1, "disable");
    // disable UDC
    writel (0, &tc_regs->PowerDetect);
    writel (0, &tc_regs->IntEnable);
    tc_do_task_ep = 0;
    /* need to read tc registers... */
    writel (PW_RESETB, &tc_regs->PowerDetect);

    // reset device pointer
    udc_device = NULL;

    // ep0 urb
    if (ep0_urb) {
        usbd_dealloc_urb (ep0_urb);
        ep0_urb = NULL;
    }
}


/**
 * udc_startup - allow udc code to do any additional startup
 */
void
udc_startup_events (struct usb_device_instance *device)
{
    usbd_device_event (device, DEVICE_INIT, 0);
    usbd_device_event (device, DEVICE_CREATE, 0);
#if 0
    usbd_device_event (device, DEVICE_HUB_CONFIGURED, 0);
    usbd_device_event (device, DEVICE_RESET, 0);        // XXX should be done from device event
#endif
}


/* *********************************************************************** */

/**
 * udc_name - return name of USB Device Controller
 */
char *
udc_name (void)
{
    return UDC_NAME;
}

/**
 * udc_request_udc_irq - request UDC interrupt
 *
 * Return non-zero if not successful.
 */
int
udc_request_udc_irq ()
{
    return 0;                        /* request_irq is called in udc_probe */
}

/**
 * udc_request_cable_irq - request Cable interrupt
 *
 * Return non-zero if not successful.
 */
int
udc_request_cable_irq ()
{
    return -1;
}

/**
 * udc_request_udc_io - request UDC io region
 *
 * Return non-zero if not successful.
 */
int
udc_request_io ()
{
    return 0;                        /* request_region is called in udc_probe */
}

/**
 * udc_release_udc_irq - release UDC irq
 */
void
udc_release_udc_irq ()
{
}

/**
 * udc_release_cable_irq - release Cable irq
 */
void
udc_release_cable_irq ()
{
}

/**
 * udc_release_release_io - release UDC io region
 */
void
udc_release_io ()
{
    if (tc_task.sync) {
        flush_scheduled_tasks ();
    }
    pci_unregister_driver (&udc_pci_driver);
}


/**
 * udc_regs - dump registers
 *
 * Dump registers with printk
 */
void
udc_regs (void)
{
    printk ("\n");
}

/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
