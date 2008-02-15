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


