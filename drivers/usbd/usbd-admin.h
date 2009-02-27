/*
 * usbd/usbd-admin.h 
 *
 * Copyright (c) 2004 Belcarra
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
 * This defines the public interface to administer the USB Device Software.
 *
 * Specifically it allows the USB Device software to be:
 *
 * 	enabled		enable a specific function driver for use
 * 	disabled	disable the USB Device Software
 *
 * 	disconnected	disable the USB Pullup Resistor (also known as soft-connect)
 * 	connected	enable the USB Pullup Resistor
 *
 * 	pm_off		perform required functions for Power Management off
 * 	pm_on		perform required functions for Power Management on
 *
 * 
 */

int usbd_enable(char *);
int usbd_enable_irq(char *);
int usbd_disable(char *);
int usbd_disable_irq(char *);

int usbd_disconnect(char *);
int usbd_connect(char *);
int usbd_pm_on(char *);
int usbd_pm_off(char *);

#ifdef CONFIG_USBD_DEPRECATED
int usbd_load(char *);
int usbd_unload(char *);
int usbd_replug(char *);
int usbd_unplug(char *);
#endif

