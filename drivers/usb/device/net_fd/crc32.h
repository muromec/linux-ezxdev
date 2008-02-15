/*
 * linux/drivers/usbd/net_fd/crc32.h
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 * Copyright (c) 2001 Hewlett Packard
 * Copyright (c) 2003 MontaVista Software Inc.
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


extern __u32 net_fd_crc32_table[256];

#define CRC32_INITFCS	0xffffffff	// Initial FCS value
#define CRC32_GOODFCS	0xdebb20e3	// Good final FCS value

#define CRC32_FCS(fcs, c) (((fcs) >> 8) ^ net_fd_crc32_table[((fcs) ^ (c)) & 0xff])

/**
 * fcs_memcpy32 - memcpy and calculate fcs
 * @dp:
 * @sp:
 * @len:
 * @fcs:
 *
 * Perform a memcpy and calculate fcs using ppp 32bit CRC algorithm.
 */
static __u32 __inline__ fcs_memcpy32 (unsigned char *dp, unsigned char *sp, int len, __u32 fcs)
{
	for (; len-- > 0; fcs = CRC32_FCS (fcs, *dp++ = *sp++));
	return fcs;
}

/**
 * fcs_pad32 - pad and calculate fcs
 * @dp:
 * @len:
 * @fcs:
 *
 * Pad and calculate fcs using ppp 32bit CRC algorithm.
 */
static __u32 __inline__ fcs_pad32 (unsigned char *dp, int len, __u32 fcs)
{
	for (; len-- > 0; fcs = CRC32_FCS (fcs, *dp++ = '\0'));
	return fcs;
}

/**     
 * fcs_compute32 - memcpy and calculate fcs
 * @sp:
 * @len:
 * @fcs:
 *
 * Perform a memcpy and calculate fcs using ppp 32bit CRC algorithm.
 */
static __u32 __inline__ fcs_compute32 (unsigned char *sp, int len, __u32 fcs)
{
	for (; len-- > 0; fcs = CRC32_FCS (fcs, *sp++));
	return fcs;
}
