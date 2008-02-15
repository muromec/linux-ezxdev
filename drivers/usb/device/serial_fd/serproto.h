/*
 * serial_fd/serproto.h
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 * Copyright (c) 2001 Hewlett Packard
 *
 * By: 
 *      Stuart Lynne <sl@lineo.com>, 
 *      Tom Rushworth <tbr@lineo.com>, 
 *      Bruce Balden <balden@lineo.com>
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

#ifndef SERPROTO_H
#define SERPROTO_H 1
#include "../usbd-debug.h"

#ifdef CONFIG_DEVFS_FS
#define USBD_TTY_NAME "usb/tts/%d"
#else
#define USBD_TTY_NAME "ttyUSBx"
#endif

extern debug_option *serproto_get_dbg_table (void);
extern int serproto_modinit (char *, int);
extern int serproto_create (char *, int (*xmit_data) (int, unsigned char *, int), int, int, int,
			    int, int);
extern int serproto_done (int, void *, int, int);
extern int serproto_recv (int, unsigned char *, int);
extern int serproto_control (int, int);
#define SERPROTO_DISCONNECT 0
#define SERPROTO_CONNECT    1
extern int serproto_destroy (int);
extern void serproto_modexit (void);

#endif
