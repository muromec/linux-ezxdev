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
 *
 *
 *  History:
 *  Richard Xiao      April 11, 2005     add dsplog ioctl for ezx platform
 *
 */

#ifndef __EZX_DSPLOG_USB_H
#define __EZX_DSPLOG_USB_H

#define DL_DRV_NAME		"usbdl"
#define DSPLOG_MINOR            0x15

/* ioctl command */
#define DSPLOG_ENABLED          0
#define DSPLOG_DISABLED         1

#endif		// __EZX_DSPLOG_USB_H
