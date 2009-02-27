/*
 * Copyright (C) 2003-2005 Motorola
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * 2003-Nov-25 Created for Motorola EzX platform, Zhuang Xiaofan
 */

#include <linux/init.h>


#define USB_FUNC_ACM	1
#define USB_FUNC_PST	2
#define USB_FUNC_NET	3

static int usb_flag = USB_FUNC_ACM;

extern void pcap_switch_on_usb(void);
extern void pcap_switch_off_usb(void);

static int __init usbd_func_switch(char *line)
{
	if (strncmp (line, "acm", 3) == 0) usb_flag=USB_FUNC_ACM;
	if (strncmp (line, "pst", 3) == 0) usb_flag=USB_FUNC_PST;
	if (strncmp (line, "net", 3) == 0) usb_flag=USB_FUNC_NET;
	return 1;
}
__setup("usbd=", usbd_func_switch);

void pulldown_usb()
{
	if((usb_flag == USB_FUNC_ACM) || (usb_flag == USB_FUNC_PST)) {
		pcap_switch_off_usb();
	}
}

void pullup_usb()
{
	if((usb_flag == USB_FUNC_ACM) || (usb_flag == USB_FUNC_PST)) {
		pcap_switch_on_usb();
	}
}

void __init usbd_func_init()
{
	switch (usb_flag) {
	case USB_FUNC_NET:
		network_modinit();
		break;
	case USB_FUNC_PST:
		pst_modinit();
		break;
	case USB_FUNC_ACM:
	default:
		acm_modinit();
		break;
	}
	bi_modinit();
}

