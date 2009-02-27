/*
 * usbd/wmmx_bi/wmmx.h
 *
 *      Copyright (c) 2004 Belcarra
 *
 * Adapted from earlier work:
 *      Copyright (c) 2002, 2003 Belcarra
 *      Copyright (c) 2000, 2001, 2002 Lineo
 *      Copyright (c) 2003 MontaVista Software, Inc.
 *
 * By: 
 *      Stuart Lynne <sl@lineo.com>, 
 *      Tom Rushworth <tbr@lineo.com>, 
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

#define EP0_PACKETSIZE  0x10

#define UDC_MAX_ENDPOINTS       24
#define MAX_LOGICAL_ENDPOINTS   16
#define MAX_LOGICAL_CONFIGURATIONS 4

#define UDC_NAME        "WMMX"


/* When we re-configure the device we search a table to find
 * the offset to the first physical endpoint for that configuration.
 */
struct wmmx_index {
        u8      configuration;
        u8      interface;
        u8      alternate;
        u8      offset;
};

