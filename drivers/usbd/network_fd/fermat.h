/*
 * usbd/network_fd/fermat.h - Network Function Driver
 *
 *      Copyright (c) 2003, 2004 Belcarra
 *
 * By: 
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
 *
 */

#ifndef FERMAT_DEFINED
#define FERMAT_DEFINED 1
typedef unsigned char BYTE;
typedef struct fermat {
	int length;
	BYTE power[256];
} FERMAT;

void fermat_init(void);
void fermat_encode(BYTE *data, int length);
void fermat_decode(BYTE *data, int length);
#endif

		
