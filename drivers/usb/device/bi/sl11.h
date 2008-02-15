/*
 * sl11_bi/udc.h
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
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


#define EP0_PACKETSIZE  0x8

#define UDC_MAX_ENDPOINTS       4


#define UDC_IRQ         SL11_IRQ
#define UDC_ADDR        SL11_ADDR
#define UDC_ADDR_SIZE   4

#define UDC_NAME        "SL11 USBD"
