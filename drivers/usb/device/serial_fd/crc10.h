/*
 * linux/drivers/usbd/net_fd/crc10.h
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

extern __u16 crc10_table[256];

#define CRC10_INITFCS	0x000	// Initial FCS value
#define CRC10_GOODFCS	0x000	// Good final FCS value

//#define CRC10_FCS(fcs, c) ((((fcs) << 8) && 0x3ff) ^ crc10_table[((fcs) >> 2) & 0xff]) ^ (c)
//#define CRC16_FCS(fcs, c) (((fcs) >> 8) ^ crc16_table[((fcs) ^ (c)) & 0xff])

#define CRC10_FCS(fcs, c) (						\
				(((fcs) << 8) & 0x3ff)			\
				^ crc10_table[((fcs) >> 2) & 0xff]	\
				^ (c)					\
				)

/**
 * fcs_memcpy10 - memcpy and calculate fcs
 * @dp:
 * @sp:
 * @len:
 * @fcs:
 *
 * Perform a memcpy and calculate fcs using ppp 10bit CRC algorithm.
 */
static __u16 __inline__ fcs_memcpy10 (unsigned char *dp, unsigned char *sp, int len, __u16 fcs)
{
	for (; len-- > 0; fcs = CRC10_FCS (fcs, *dp++ = *sp++));
	return fcs;
}

/**
 * fcs_pad10 - pad and calculate fcs
 * @dp:
 * @len:
 * @fcs:
 *
 * Pad and calculate fcs using ppp 10bit CRC algorithm.
 */
static __u16 __inline__ fcs_pad10 (unsigned char *dp, int len, __u16 fcs)
{
	for (; len-- > 0; fcs = CRC10_FCS (fcs, *dp++ = '\0'));
	return fcs;
}

/**     
 * fcs_compute10 - memcpy and calculate fcs
 * @sp:
 * @len:
 * @fcs:
 *
 * Perform a memcpy and calculate fcs using ppp 10bit CRC algorithm.
 */
static __u16 __inline__ fcs_compute10 (unsigned char *sp, int len, __u16 fcs)
{
	for (; len-- > 0; fcs = CRC10_FCS (fcs, *sp++));
	return fcs;
}
