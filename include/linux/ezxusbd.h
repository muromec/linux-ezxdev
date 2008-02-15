/*
 * Copyright (C) 2005 Motorola Inc.
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
 */
/*
 * create by w20146, for EZX platform
 */ 

#ifndef __EZX_USBD_H
#define __EZX_USBD_H

#define MOTUSBD_PROCFILE		"/proc/motusbd"

// macros for USB status
#define USB_STATUS_NONE		(-1)
#define USB_STATUS_MODEM	0
#define USB_STATUS_NET		1
#define USB_STATUS_USBLAN	USB_STATUS_NET
#define USB_STATUS_CFG11	2
#define USB_STATUS_PST		3
#define USB_STATUS_STORAGE	4
#define USB_STATUS_DSPLOG       5
#define USB_STATUS_NM           6

#define USB_STATUS_STRING_NONE		"MotNone"
#define USB_STATUS_STRING_PST		"MotPst"
#define USB_STATUS_STRING_CFG11		"MotCdc"
#define USB_STATUS_STRING_NET		"MotNet"
#define USB_STATUS_STRING_USBLAN	USB_STATUS_STRING_NET
#define USB_STATUS_STRING_MODEM		"MotACM"
#define USB_STATUS_STRING_STORAGE	"MotMas"
#define USB_STATUS_STRING_DSPLOG        "MotDSPLog"
#define USB_STATUS_STRING_NM            "MotNM"


#define MOTUSBD_DEV_MAJOR_NUM		222
#define MOTUSBD_DEV_NAME		"motusbd"		// "/dev/motusbd"

//e12707 motusbd ioctl command used by switch usb mode
#define MOTUSBD_IOCTL_SET_USBMODE 1
#define MOTUSBD_IOCTL_GET_USBMODE 2



typedef enum
{
	USB_ACM_READY = 	USB_STATUS_MODEM,
	USB_BLAN_READY = 	USB_STATUS_USBLAN,
	USB_CFG11_READY = 	USB_STATUS_CFG11,
	USB_PST_READY = 	USB_STATUS_PST,
	USB_STORAGE_READY =	USB_STATUS_STORAGE,
	USB_DSPLOG_READY =	USB_STATUS_DSPLOG,
	USB_NM_READY =		USB_STATUS_NM,
	USB_CURRENT_500MA,
	USB_CURRENT_100MA,
	USB_CURRENT_0MA,		// enumeration ok with <100m current
	USB_CURRENT_SUSPEND		// USB SUSPEND, no current from VBUS
}MOTUSBD_MSG_T;

int usbd_in_nfs_mode(void);

void usbd_set_gpio_function(int enable);

int queue_motusbd_event(MOTUSBD_MSG_T event);

#endif		// __EZX_USBD_H
