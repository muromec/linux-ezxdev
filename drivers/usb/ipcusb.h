/*
 *  Copyright (C) 2003-2004 - Motorola
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  History:
 *  Created by Levis(Levis@motorola.com) for mux driver use in 11/18/2003
 *  Remove tty_struct and add the usb_flip_buf struct by Levis(Levis@motorola.com
 * ) in 12/7/2004
 */

#define USB_FLIPBUF_SIZE	2048

struct usb_flip_buffer
{
	char *char_buf_ptr;
	int count;
	int buf_num;
	unsigned char char_buf[2*USB_FLIPBUF_SIZE];
};

extern struct tty_driver *usb_for_mux_driver;
extern struct usb_flip_buffer *usb_for_mux_flip_buf;
extern void (*usb_mux_dispatcher)(struct tty_struct *tty);
extern void (*usb_mux_sender)(void);

#undef BVD_DEBUG		
//#define BVD_DEBUG		

#ifdef BVD_DEBUG
#define bvd_dbg(format, arg...) printk(__FILE__ ": " format "\n" , ## arg)
#else
#define bvd_dbg(format, arg...) do {} while (0)
#endif

#define USB_RESUME_SUSPEND_DEBUG
#ifdef USB_RESUME_SUSPEND_DEBUG
#define resume_suspend_dbg(format, arg...) printk(__FILE__ ": " format "\n" , ## arg)
#else
#define resume_suspend_dbg(format, arg...) do {} while (0)
#endif

#define IS_EP_BULK(ep)  ((ep).bmAttributes == USB_ENDPOINT_XFER_BULK ? 1 : 0)
#define IS_EP_BULK_IN(ep) (IS_EP_BULK(ep) && ((ep).bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN)
#define IS_EP_BULK_OUT(ep) (IS_EP_BULK(ep) && ((ep).bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT)


#define MOTO_IPC_VID		0x22b8
#define MOTO_IPC_PID		0x3006
#define IPC_USB_XMIT_SIZE	4096
#define IPC_URB_SIZE		512
#define IPC_USB_WRITE_INIT 	0
#define IPC_USB_WRITE_XMIT	1
#define IPC_USB_PROBE_READY	3
#define IPC_USB_PROBE_NOT_READY	4
#define DBG_MAX_BUF_SIZE	1024
#define ICL_EVENT_INTERVAL	(HZ) 

#ifdef CONFIG_ARCH_EZX_MARTINIQUE
#define IPC_USB_SUSPEND_INTERVAL	1000
#else
#define IPC_USB_SUSPEND_INTERVAL	5000
#endif
#define WAKE_UP_BP_UDELAY	125


