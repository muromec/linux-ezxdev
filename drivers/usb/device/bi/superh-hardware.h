/*
 * superh_bi/hardware.h
 *
 * Copyright (c) 2002 Lineo
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


/*
 * Standby Control Register (STBCR3) c.f. 9.2.3
 */

#define MSTP14          0x10

/* 
 * EXCPG Control Register (EXCPGCR) c.f. Section 11.3.1
 */

#define USBDIVS_EL0	0x00
#define USBDIVS_EL1	0x01
#define USBDIVS_EL2	0x02

#define	USBCKS_EL1	0x04
#define	USBCKS_EL2	0x10
#define	USBCKS_EL3	0x20

#define USBDIV_11	0x00
#define USBDIV_12	0x01
#define USBDIV_13	0x02

#define	USBCKS_PC	0x00
#define	USBCKS_IC	0x20
#define	USBCKS_BC	0x24
#define	USBCKS_EC	0x30


/*
 * Extra Pin Function Controller (EXPFC) c.f. Section 22.2.1
 */

#define	USB_TRANS_TRAN	0x00
#define	USB_TRANS_DIG	0x02

#define	USB_SEL_HOST	0x00
#define	USB_SEL_FUNC	0x01


/*
 * USBDMA Setting Register (USBDMAR) c.f. Section 23.5.19
 */

#define EP1_DMAE        0x01
#define EP2_DMAE        0x02
#define PULLUP_E        0x04

/*
 * USB Interrupt Flag Register 0 (USBIFR0) c.f. Section 23.5.7
 */

#define BRST            0x80
#define EP1_FULL        0x40
#define EP2_TR          0x20
#define EP2_EMPTY       0x10
#define SETUP_TS        0x08
#define EP0o_TS         0x04
#define EP0i_TR         0x02
#define EP0i_TS         0x01


/*
 * USB Interrupt Flag Register 1 (USBIFR0) c.f. Section 23.5.8
 */

#define VBUSMN          0x08
#define EP3_TR          0x04
#define EP3_TS          0x02
#define VBUSF           0x01

/*
 * USB Trigger Register (USBTRG) c.f. Section 23.5.9
 */

#define EP0i_PKTE       0x01
#define EP0o_PKTE       0x02
#define EP0s_PKTE       0x04

#define EP2_PKTE        0x10
#define EP1_PKTE        0x20
#define EP3_PKTE        0x40


/*
 * USBFIFO Clear Register (USBFCLR) c.f. Section 23.5.10
 */

#define EP3_CLEAR       0x40
#define EP1_CLEAR       0x20
#define EP2_CLEAR       0x10
#define EP0o_CLEAR      0x02
#define EP0i_CLEAR      0x01


/*
 * Port Control Registers (PNCR) c.f. Section 26.2
 */


#define PN_PB0_OF       0x0000
#define PN_PB0_PO       0x0001
#define PN_PB0_PI_ON    0x0002
#define PN_PB0_PI_OFF   0x0003
#define PN_PB0_MSK     ~0x0003

#define PN_PB1_OF       0x0000
#define PN_PB1_PO       0x0004
#define PN_PB1_PI_ON    0x0008
#define PN_PB1_PI_OFF   0x000c
#define PN_PB1_MSK     ~0x000c

#define PN_PB2_OF       0x0000
#define PN_PB2_PO       0x0010
#define PN_PB2_PI_ON    0x0020
#define PN_PB2_PI_OFF   0x0030
#define PN_PB2_MSK     ~0x0030

#define PN_PB3_OF       0x0000
#define PN_PB3_PO       0x0040
#define PN_PB3_PI_ON    0x0080
#define PN_PB3_PI_OFF   0x00c0
#define PN_PB3_MSK     ~0x00c0

#define PN_PB4_OF       0x0000
#define PN_PB4_PO       0x0100
#define PN_PB4_PI_ON    0x0200
#define PN_PB4_PI_OFF   0x0300
#define PN_PB4_MSK     ~0x0300

#define PN_PB5_OF       0x0000
#define PN_PB5_PO       0x0400
#define PN_PB5_PI_ON    0x0800
#define PN_PB5_PI_OFF   0x0c00
#define PN_PB5_MSK     ~0x0c00

#define PN_PB6_OF       0x0000
#define PN_PB6_PO       0x1000
#define PN_PB6_PI_ON    0x2000
#define PN_PB6_PI_OFF   0x3000
#define PN_PB6_MSK     ~0x3000

#define PN_PB7_OF       0x0000
#define PN_PB7_PO       0x4000
#define PN_PB7_PI_ON    0x8000
#define PN_PB7_PI_OFF   0xc000
#define PN_PB7_MSK     ~0xc000
