/*
 * usbd/pxa_bi/pxa.h
 *
 *      Copyright (c) 2004 Belcarra
 *
 * Adapted from earlier work:
 *      Copyright (c) 2002, 2003 Belcarra
 *      Copyright (c) 2000, 2001, 2002 Lineo
 *
 * By: 
 *      Stuart Lynne <sl@belcarra.com>, 
 *      Tom Rushworth <tbr@belcarra.com>, 
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

#define UDC_MAX_ENDPOINTS       6

#define UDC_NAME        "PXA"

#ifdef CONFIG_SABINAL_DISCOVERY

/* SHARP A3 specific GPIO assignments */
#define USBD_CONNECT_GPIO       50

/*
 * DVT and DVT2 are defined as HIGH, others (PreMV) are LOW
 */
#if defined(CONFIG_SABINAL_DISCOVERY_DVT) || defined(CONFIG_SABINAL_DISCOVERY_DVT2)
#define USBD_CONNECT_HIGH        1   /* Pin high for connect (undef for pin low) */
#warning USBD_CONNECT_HIGH DEFINED
#else
#undef USBD_CONNECT_HIGH
#warning USBD_CONNECT_HIGH UNDEF
#endif

#define CKEN11_USB	USB_CE		 /* USB Unit Clock Enable */

#endif /* CONFIG_SABINAL_DISCOVERY */




/*
 * CP15 - Coprocessor 15 Register 0 - ID Register Defintion
 * C.f. PXA255 2.2.4
 */

#define CP15R0_NUM_MASK         0x000003f0      /* Product Number Mask */
#define CP15R0_REV_MASK         0x0000000f

#define PXA255_A0               0x00000106      

#define CP15R0_VENDOR_MASK      0xffffe000

#if     defined(CONFIG_ARCH_PXA)
#define CP15R0_XSCALE_VALUE     0x69052000      /* intel/arm/xscale */

#elif   defined(CONFIG_ARCH_IXP425)

#define CP15R0_XSCALE_VALUE     0x69054000      /* intel/arm/ixp425 */

#endif 


/*
 * Reserved Register Definitions
 */
#define UDCRSVD0        __REG(0x40600004)  /* UDC Reserved Register 0 */
#define UDCRSVD1        __REG(0x40600008)  /* UDC Reserved Register 1 */
#define UDCRSVD2        __REG(0x4060000c)  /* UDC Reserved Register 2 */

//#define UFNHR           __REG(0x40600060)  /* UDC Frame Number High Register */
//#define UFNLR           __REG(0x40600064)  /* UDC Frame Number Low Register */

#define	UDCCFR		__REG(0x40600008)	/* UDC Control Function Register - PXA-255 */

#define	UDCCFR_AREN	(1<<7)			/* ACK Response Mode - PXA-255 */
#define	UDCCFR_ACM	(1<<2)			/* ACK Control Mode - PXA-255 */

