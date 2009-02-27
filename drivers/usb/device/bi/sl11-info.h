/*
 * sl11_bi/sl11.h
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

#define SL11_ADDR       0x390
#define SL11_ADDR_SIZE  4

#define SL11_IRQ        3

#define PC_ADDR         390h
#define outportb        _outp
#define inportb         _inp

/*-------------------------------------------------------------------------
 * EP0 use for configuration and Vendor Specific command interface
 *-------------------------------------------------------------------------
 */
#define EP0Buf          0x40	/* SL11 memory start at 0x40 */
#define EP0Len          0x40	/* Length of config buffer EP0Buf */
#define EP1Buf          0x60
#define EP1Len          0x40

/*
 * We have three 64 byte buffers. The first buffer is shared by
 * the control endpoint, all Bulk IN endpoints and Interrupt endpoints.
 *
 * The other two buffers are used as ping pong buffers for a single
 * OUT bulk endpoint.
 *
 * TODO Allow an IN bulk endpoint to borrow one of the ping pong
 * buffers so it can setup ping pong send buffers. This would result
 * in the OUT bulk endpoint only having a single buffer. But if
 * both IN and OUT are in use at the same time then there should
 * be sufficent time for a single OUT buffer to be emptied in time.
 *
 * TODO Allow two IN bulk endpoints by setting them up as A-B and B-A
 * such that each can operate.
 */

#define EP_S_BUF        0x40	// shared buffer, Bulk IN, Interrupt, Control

#define EP_A_BUF        0x80	// ping pong A - OUT
#define EP_B_BUF        0xc0	// ping pong B - OUT


/*-------------------------------------------------------------------------
 * SL11 memory from 80h-ffh use as ping-pong buffer for endpoint1-endpoint3
 * These buffers are share for endpoint 1-endpoint 3.
 * For DMA: endpoint3 will be used
 *-------------------------------------------------------------------------
 */
#define uBufA           0x80	/* buffer A address for DATA0 */
#define uBufB           0xc0	/* buffer B address for DATA1 */
#define uXferLen        0x40	/* xfer length */
#define sMemSize        0xc0	/* Total SL11 memory size */

/*-------------------------------------------------------------------------
 * SL11 Register Control memory map
 *-------------------------------------------------------------------------
 */
#define CtrlReg         0x05
#define IntEna          0x06
#define USBAdd          0x07

#define IntStatus       0x0d
#define DATASet         0x0e

#define SOFLow          0x15
#define SOFHigh         0x16

#define sDMACntLow      0x35
#define sDMACntHigh     0x36

#define IntMask         0x57	/* Reset|DMA|EP0|EP2|EP1 for IntEna */
#define HostMask        0x47	/* Host request command  for IntStatus */
#define ReadMask        0xd7	/* Read mask interrupt   for IntStatus */

#define EP0_Control     0x00

#define EP1_Control     0x10
#define EP2_Control     0x20
#define EP3_Control     0x30

#define EPN_A           0x00
#define EPN_B           0x08

#define EPN_Control      0x00
#define EPN_Address      0x01
#define EPN_XferLen      0x02
#define EPN_Status       0x03
#define EPN_Counter      0x04

/*
 * EP control register masks
 */
#define EPN_CTRL_ARM            0x01
#define EPN_CTRL_ENABLE         0x02

#define EPN_CTRL_DIRECTION      0x04
#define EPN_CTRL_DIRECTION_IN   0x04
#define EPN_CTRL_DIRECTION_OUT  0x00

#define EPN_CTRL_NEXTDATA       0x08
#define EPN_CTRL_ISO            0x10
#define EPN_CTRL_SENDSTALL      0x20
#define EPN_CTRL_SEQUENCE       0x40
#define EPN_CTRL_RESERVERED     0x80

/*
 * EP Packet status masks
 */
#define EPN_PSTS_ACK            0x01
#define EPN_PSTS_ERROR          0x02
#define EPN_PSTS_TIMEOUT        0x04
#define EPN_PSTS_SEQUENCE       0x08
#define EPN_PSTS_SETUP          0x10
#define EPN_PSTS_OVERFLOW       0x20
#define EPN_PSTS_NAK            0x40
#define EPN_PSTS_STALL          0x80


/*
 * Control Register masks
 */
#define CTRL_USB_ENABLE         0x01
#define CTRL_DMA_ENABLE         0x02
#define CTRL_USB_RESET          0x08

/*
 * Interrupt enable register mask
 */
#define INT_EP0_DONE            0x01
#define INT_EP1_DONE            0x02
#define INT_EP2_DONE            0x04
#define INT_EP3_DONE            0x08
#define INT_DMA_DONE            0x10
#define INT_SOF_RECEIVED        0x20
#define INT_USB_RESET           0x40
#define INT_DMA_STATUS          0x80

/*
 * Current Data Set Register masks
 */
#define CDS_EP0                 0x01
#define CDS_EP1                 0x02
#define CDS_EP2                 0x04
#define CDS_EP3                 0x08
#define CDS_RESERVED            0x10


#if  0
/**
 * sl11write_byte - write a byte to the sl11
 * @a: sl11 address
 * @b: byte to write
 */
__inline__ void sl11write_byte (unsigned char, unsigned char);

/**
 * sl11write_buffer - write a buffer to the sl11 using auto-increment mode
 * @a: sl11 address
 * @b: pointer to buffer to write
 * @size: number of bytes to write
 */
__inline__ void sl11write_buffer (unsigned char, unsigned char *, unsigned char);

/**
 * sl11read_byte - read a byte from the sl11 and return
 * @a: sl11 address
 */
__inline__ unsigned char sl11read_byte (unsigned char);

/**
 * sl11read_buffer - fill a buffer from the sl11 using auto-increment mode
 * @a: sl11 address
 * @b: pointer to buffer to fill
 * @size: number of bytes to read
 */
__inline__ void sl11read_buffer (unsigned char, unsigned char *, unsigned char);


/**
 * sl11write_epn - write a byte to a sl11 endpoint register
 * @a: endpoint 
 * @b: A or B
 * @c: endpoint register
 * @d: byte to write
 */
__inline__ void sl11write_epn (unsigned char, unsigned char, unsigned char);

/**
 * sl11read_epn - read a byte from a sl11 endpoint register
 * @a: endpoint 
 * @b: A or B
 * @c: endpoint register
 * @d: byte to write
 */
__inline__ void sl11read_epn (unsigned char, unsigned char);

#endif
