/******************************************************************************

	usb-mx2hcd.h
	driver for Motorola Dragonball MX2 on-chip USB Host (part of USB OTG)
	header file, for local use only.

	Copyright (c) 2004 MontaVista Software, Inc. <source@mvista.com>
	-----------------------------------------------------------------------
	virtual root hub code is almost unchanged from usb-ohci.c "ver. 5.3":
	"URB OHCI HCD (Host Controller Driver) for USB."

	"(C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>"
	"(C) Copyright 2000-2001 David Brownell <dbrownell@users.sourceforge.net>"

	"[ Initialisation is based on Linus'  ]"
	"[ uhci code and gregs ohci fragments ]"
	"[ (C) Copyright 1999 Linus Torvalds  ]"
	"[ (C) Copyright 1999 Gregory P. Smith]"
	-----------------------------------------------------------------------
	URB scheduling, and URB debug print is based on hc_simple.c:
	"simple generic USB HCD frontend Version 0.9.5 (10/28/2001)"
	"for embedded HCs (SL811HS)"
	-----------------------------------------------------------------------

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; either version 2
	of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
	
********************************************************************************/

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/completion.h>

#include <asm/irq.h>
#include <asm/system.h>
#include <asm/arch/hardware.h>
#include <asm/arch/irqs.h>
#include <asm/arch/gpio.h>
#include <asm/proc/cache.h>
#include <asm/io.h>
#include <linux/pci.h>

#include <linux/usb.h>
#include "hcd.h"
#include "usb-mx2otg.h"

#undef MX2HCD_USE_DIRECT_DMA
#undef MX2HCD_USE_DIRECT_ISO_DMA

#undef DEBUG
/*#define DEBUG*/
/*#define dbg(format, arg...) printk(KERN_ERR __FILE__ ": " format "\n" , ## arg)*/

#define MODULE_NAME "MX2 USB HOST"

#define MX2HCD_UNLINK_URB_TIMEOUT (HZ/10)	/*like in OHCI */

/* control transfer states */
#define US_CTRL_SETUP	2
#define US_CTRL_DATA	1
#define US_CTRL_ACK	0

/* bulk transfer main state and 0-length packet */
#define US_BULK		1
#define US_BULK0	0

/* condition (error) CC codes classification table */
static int cc_to_error[16] = {

/* mapping of the MX2HCI CC status to error codes */
	/* No  Error  */ USB_ST_NOERROR,
	/* CRC Error  */ USB_ST_CRC,
	/* Bit Stuff  */ USB_ST_BITSTUFF,
	/* Data Togg  */ USB_ST_BITSTUFF,
	/* Stall      */ USB_ST_STALL,
	/* DevNotResp */ USB_ST_NORESPONSE,
	/* PIDFailure */ USB_ST_BITSTUFF,
	/* Reserved   */ USB_ST_NORESPONSE,
	/* DataOver   */ USB_ST_DATAOVERRUN,
	/* DataUnder  */ USB_ST_DATAUNDERRUN,
	/* ACK Handshake (debug) */ USB_ST_NOERROR,
	/* NACK Handshake */ USB_ST_NORESPONSE,
	/* BufferOver */ USB_ST_BUFFEROVERRUN,
	/* BuffUnder  */ USB_ST_BUFFERUNDERRUN,
	/* Shedule Overrun */ USB_ST_INTERNALERROR,
	/* Not Access */ USB_ST_NORESPONSE
};

/* condition (error) CC codes and mapping (OHCI like) */

#define TD_CC_NOERROR		0x00
#define TD_CC_CRC		0x01
#define TD_CC_BITSTUFFING	0x02
#define TD_CC_DATATOGGLEM	0x03
#define TD_CC_STALL		0x04
#define TD_DEVNOTRESP		0x05
#define TD_PIDCHECKFAIL		0x06
/*#define TD_UNEXPECTEDPID	0x07 - reserved, not active on MX2*/
#define TD_DATAOVERRUN		0x08
#define TD_DATAUNDERRUN		0x09
#define TD_BUFFEROVERRUN	0x0C
#define TD_BUFFERUNDERRUN	0x0D
#define TD_NOTACCESSED		0x0F

/* Local CC Codes */
#define MX2HCD_ETD_ACK           0xA
#define MX2HCD_ETD_NACK          0xB
#define MX2HCD_ETD_SCHOVERRUN   0xE

/*ETD format description*/
#define MX2HCD_FMT_CTRL   0x0
#define MX2HCD_FMT_ISO    0x1
#define MX2HCD_FMT_BULK   0x2
#define MX2HCD_FMT_INT    0x3

static char fmt_urb_to_etd[4] = {
/*PIPE_ISOCHRONOUS*/ MX2HCD_FMT_ISO,
/*PIPE_INTERRUPT*/ MX2HCD_FMT_INT,
/*PIPE_CONTROL*/ MX2HCD_FMT_CTRL,
/*PIPE_BULK*/ MX2HCD_FMT_BULK
};

/*ETD direction description*/
#define MX2HCD_DIR_SETUP 0x0	/*output to endpoint, useful for CTRL transfers */
#define MX2HCD_DIR_OUT   0x1
#define MX2HCD_DIR_IN    0x2

/*ETD toggle packet ID description*/
#define MX2HCD_TGL_DATA0  0x02
#define MX2HCD_TGL_DATA1  0x03
#define MX2HCD_TGL_CARRY  0x00

#define MX2HCD_MAX_ETD 32

#define NUM_EDS 32

typedef struct epd {
	struct urb *pipe_head;
	struct list_head urb_queue;
	struct timer_list timeout;
} epd_t;

struct hci_device {
	epd_t ed[NUM_EDS];
};

typedef struct urb_priv_st {
	int etd_num;		/*indicates the etd number for the URB (iso: even # packets only) */
	int etd_num1;		/*indicates an etd number for the ISO URB (odd # packets only) */
	int iso_index;		/*indicates iso packet being processed in ETD #etd_num */
	int iso_index1;		/*indicates iso packet being processed in ETD #etd_num1 */
	struct completion done;	/*used for unlinking URBs */
	int unlinked;		/*used for unlinking URBs */
	struct timer_list timeout;	/*used for unlink timeout */
} urb_priv_t;

#define usb_to_hci(usb)	((struct hci_device *)(usb)->hcpriv)

struct virt_root_hub {
	int devnum;		/* Address of Root Hub endpoint */
	void *urb;		/* interrupt URB of root hub */
	int send;		/* active flag */
	int interval;		/* intervall of roothub interrupt transfers */
	struct timer_list rh_int_timer;	/* intervall timer for rh interrupt EP */
};

typedef struct hci {
	struct virt_root_hub rh;	/* roothub */
	wait_queue_head_t waitq;	/* deletion of URBs and devices needs a waitqueue */

	struct list_head ctrl_list;	/* set of ctrl endpoints */
	struct list_head bulk_list;	/* set of bulk endpoints */
	struct list_head iso_list;	/* set of isoc endpoints */
	struct list_head intr_list;	/* set of int endpoints */

	struct urb *etd_urb[MX2HCD_MAX_ETD];	/*points to the URB for this ETD number. 0 if ETD is free */
	int etd_len[MX2HCD_MAX_ETD];	/* non-iso - DMA buffer length */
	char *etd_buf[MX2HCD_MAX_ETD];	/* non-iso - DMA buffer address */

	struct usb_bus *bus;	/* our bus */

	int active_urbs;	/*counts the number of URBs being actively processed. Used for enabling/disabling HC ints */
	int disabled;
} hci_t;

/*
 * Maximum number of root hub ports.  
 */
#define MAX_ROOT_PORTS	3	/* maximum MX2 USB Host root hub ports */

/* MX2 HCI CONTROL AND STATUS REGISTER MASKS */

/*
 * HcControl (control) register masks
 */
#define HCI_CTRL_CBSR	(3 << 0)	/* control/bulk service ratio */
#define HCI_CTRL_HCFS	(3 << 2)	/* host controller functional state */
#define HCI_CTRL_RWE	(1 << 4)	/* remote wakeup enable */

/* pre-shifted values for HCFS */
#	define HCI_USB_RESET	(0 << 2)
#	define HCI_USB_RESUME	(1 << 2)
#	define HCI_USB_OPER	(2 << 2)
#	define HCI_USB_SUSPEND	(3 << 2)

#define HCI_HCR	(1 << 31)	/* host controller reset */
#define HCI_SOC  	(3 << 16)	/* scheduling overrun count */

/*
 * masks used with interrupt registers:
 */

#define HCI_INTR_SO	(1 << 0)	/* scheduling overrun */
#define HCI_INTR_WDH	(1 << 1)	/* ETD done */
#define HCI_INTR_SF	(1 << 2)	/* start frame */
#define HCI_INTR_RD	(1 << 3)	/* resume detect */
#define HCI_INTR_UE	(1 << 4)	/* unrecoverable error */
#define HCI_INTR_FNO	(1 << 5)	/* frame number overflow */
#define HCI_INTR_RHSC	(1 << 6)	/* root hub status change */

/* USB HUB CONSTANTS (not MX2-specific; see hub.h and USB spec) */

/* destination of request */
#define RH_INTERFACE		0x01
#define RH_ENDPOINT		0x02
#define RH_OTHER		0x03

#define RH_CLASS		0x20
#define RH_VENDOR		0x40

/* Requests: bRequest << 8 | bmRequestType */
#define RH_GET_STATUS		0x0080
#define RH_CLEAR_FEATURE	0x0100
#define RH_SET_FEATURE		0x0300
#define RH_SET_ADDRESS		0x0500
#define RH_GET_DESCRIPTOR	0x0680
#define RH_SET_DESCRIPTOR	0x0700
#define RH_GET_CONFIGURATION	0x0880
#define RH_SET_CONFIGURATION	0x0900
#define RH_GET_STATE		0x0280
#define RH_GET_INTERFACE	0x0A80
#define RH_SET_INTERFACE	0x0B00
#define RH_SYNC_FRAME		0x0C80
/* Our Vendor Specific Request */
#define RH_SET_EP		0x2000

/* Hub port features */
#define RH_PORT_CONNECTION	0x00
#define RH_PORT_ENABLE		0x01
#define RH_PORT_SUSPEND		0x02
#define RH_PORT_OVER_CURRENT	0x03
#define RH_PORT_RESET		0x04
#define RH_PORT_POWER		0x08
#define RH_PORT_LOW_SPEED	0x09

#define RH_C_PORT_CONNECTION	0x10
#define RH_C_PORT_ENABLE	0x11
#define RH_C_PORT_SUSPEND	0x12
#define RH_C_PORT_OVER_CURRENT	0x13
#define RH_C_PORT_RESET		0x14

/* Hub features */
#define RH_C_HUB_LOCAL_POWER	0x00
#define RH_C_HUB_OVER_CURRENT	0x01

#define RH_DEVICE_REMOTE_WAKEUP	0x00
#define RH_ENDPOINT_STALL	0x01

/* MX2 HCI ROOT HUB REGISTER MASKS */

/* roothub.portstatus [i] bits */
#define RH_PS_CCS            0x00000001	/* current connect status */
#define RH_PS_PES            0x00000002	/* port enable status */
#define RH_PS_PSS            0x00000004	/* port suspend status */
#define RH_PS_POCI           0x00000008	/* port over current indicator */
#define RH_PS_PRS            0x00000010	/* port reset status */
#define RH_PS_PPS            0x00000100	/* port power status */
#define RH_PS_LSDA           0x00000200	/* low speed device attached */
#define RH_PS_CSC            0x00010000	/* connect status change */
#define RH_PS_PESC           0x00020000	/* port enable status change */
#define RH_PS_PSSC           0x00040000	/* port suspend status change */
#define RH_PS_OCIC           0x00080000	/* over current indicator change */
#define RH_PS_PRSC           0x00100000	/* port reset status change */

/* roothub.status bits */
#define RH_HS_LPS	     0x00000001	/* local power status */
#define RH_HS_OCI	     0x00000002	/* over current indicator */
#define RH_HS_DRWE	     0x00008000	/* device remote wakeup enable */
#define RH_HS_LPSC	     0x00010000	/* local power status change */
#define RH_HS_OCIC	     0x00020000	/* over current indicator change */
#define RH_HS_CRWE	     0x80000000	/* clear remote wakeup enable */

/* roothub.b masks */
#define RH_B_DR		0x000000ff	/* device removable flags */
#define RH_B_PPCM	0x00ff0000	/* port power control mask */

/* roothub.a masks */
#define	RH_A_NDP	(0xff << 0)	/* number of downstream ports */
#define	RH_A_PSM	(1 << 9)	/* power switching mode */
#define	RH_A_NPS	(1 << 8)	/* no power switching */
#define	RH_A_DT		(1 << 10)	/* device type (mbz) */
#define	RH_A_OCPM	(1 << 11)	/* over current protection mode */
#define	RH_A_NOCP	(1 << 12)	/* no over current protection */
#define	RH_A_POTPGT	(0xff << 24)	/* power on to power good time */

/*FORWARD FUNCTION DECLARATIONS*/

/* urb interface functions */

static int hci_unlink_urb(struct urb *urb);
static int hci_get_current_frame_number(struct usb_device *usb_dev);

/* root hub */
static int rh_init_int_timer(struct urb *urb);
static int rh_submit_urb(struct urb *urb);
static int rh_unlink_urb(struct urb *urb);

/* schedule functions */
static int sh_add_packet(hci_t * hci, struct urb *urb);

/* hc specific functions */

static inline void hc_stop_int(void);
static inline void hc_start_int(void);
static inline struct urb *hc_parse_trans(hci_t * hci, int etd_num,
					 int *actbytes, int *cc);
static inline int hc_add_iso_trans(hci_t * hci, struct urb *urb);
static inline int hc_add_noniso_trans(hci_t * hci, struct urb *urb, int state);
static void hc_abort_transfer(hci_t * hci, int etd_num);
static void hc_next_iso_trans(hci_t * hci, struct urb *urb, int iso_index,
			      int etd_num);
static inline void set_iso_etd_word_0(int etd_num, struct urb *urb);

static hci_t *mx2hci_struct;

static char mx2hci_irq_inited;
static char mx2hci_otg_inited;

static int __init mx2ads_usbhost_init(void);
static void __init mx2ads_usbhost_exit(void);

int mx2otg_host_enable(void);
void mx2otg_host_disable(void);
void mx2otg_host_dma_error(void);

static struct mx2otg_host_descr_st mx2hcd_otg_descriptor = {
	.enable = &mx2otg_host_enable,
	.disable = &mx2otg_host_disable,
	.dma_err = &mx2otg_host_dma_error,
};

/*find an unused ETD*/
static int
mx2hcd_get_free_ETD_num(hci_t * hci)
{
	int etd_num;
	u32 bit_mask;

	bit_mask = 1;
	for (etd_num = 0; etd_num < MX2HCD_MAX_ETD; etd_num++) {	/*check all ETDs in use */
		if (hci->etd_urb[etd_num] == 0)	/*if etd is not active */
			break;
		bit_mask <<= 1;
	}

	if (etd_num == MX2HCD_MAX_ETD) {	/*no available ETD found */
		err("no available ETD");	/*debug */
		return -ENODEV;
	}
	return etd_num;
}

#ifdef DEBUG
/* print the main components of an URB
 * small: 0) header + data packets 1) just header */

static void
urb_print(struct urb *urb, char *str, int small)
{
	unsigned int pipe = urb->pipe;
	int i, len;

	if (!urb->dev || !urb->dev->bus) {
		dbg("%s URB: no dev", str);
		return;
	}

	printk
	    ("%s URB:[%4x] dev:%2d,ep:%2d-%c,type:%s,flags:%4x,len:%d/%d,stat:%d(%x)\n",
	     str, hci_get_current_frame_number(urb->dev), usb_pipedevice(pipe),
	     usb_pipeendpoint(pipe), usb_pipeout(pipe) ? 'O' : 'I',
	     usb_pipetype(pipe) < 2 ? (usb_pipeint(pipe) ? "INTR" : "ISOC")
	     : (usb_pipecontrol(pipe) ? "CTRL" : "BULK"), urb->transfer_flags,
	     urb->actual_length, urb->transfer_buffer_length, urb->status,
	     urb->status);
	if (!small) {
		if (usb_pipecontrol(pipe)) {
			printk(__FILE__ ": cmd(8):");
			for (i = 0; i < 8; i++)
				printk(" %02x",
				       ((__u8 *) urb->setup_packet)[i]);
			printk("\n");
		}
		if (urb->transfer_buffer_length > 0 && urb->transfer_buffer) {
			printk(__FILE__ ": data(%d/%d):", urb->actual_length,
			       urb->transfer_buffer_length);
			len =
			    usb_pipeout(pipe) ? urb->
			    transfer_buffer_length : urb->actual_length;
			for (i = 0; i < 2096 && i < len; i++)
				printk(" %02x",
				       ((__u8 *) urb->transfer_buffer)[i]);
			printk("%s stat:%d\n", i < len ? "..." : "",
			       urb->status);
		}
	}
}

static void
etd_dump(int etd_num, char *inp_str)
{
	u32 etd_addr = OTG_ETD_BASE + (etd_num << 4);
	printk("%s OTG_HOST_ETD_STAT %x\n", inp_str, OTG_HOST_ETD_STAT);
	printk("%s OTG_HOST_ETD_DONE %x\n", inp_str, OTG_HOST_ETD_DONE);
	printk("%s OTG_HOST_ETD_EN %x\n", inp_str, OTG_HOST_ETD_EN);
	printk("%s ETD %d WORD0 %x\n", inp_str, etd_num, __REG32(etd_addr));
	printk("%s ETD %d WORD1 %x\n", inp_str, etd_num, __REG32(etd_addr + 4));
	printk("%s ETD %d WORD2 %x\n", inp_str, etd_num, __REG32(etd_addr + 8));
	printk("%s ETD %d WORD3 %x\n", inp_str, etd_num,
	       __REG32(etd_addr + 12));
}

#endif

#define mx2hcd_irq_restore(flags) do { \
local_irq_restore(flags); \
} while (0)

#define mx2hcd_irq_save_disable(flags) do { \
local_irq_save(flags);\
} while (0)
