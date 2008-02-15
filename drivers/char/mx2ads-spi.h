/******************************************************************************
	mx2ads-spi.h
	driver for Motorola  MX21 ASP - Touchscreen support

	Copyright (C) 2003 Motorola Semiconductors HK Ltd

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; either version 2
	of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
	
********************************************************************************/
#ifndef SPI_H_INC
#define SPI_H_INC

#define TPNL_IRQ 					8	//INT_GPIO
#define TPNL_INTR_MODE				SA_INTERRUPT|SA_SHIRQ
#define TPNL_PEN_UPVALUE			-999

#define SPI_TE_INT_BIT		0x00000001
#define	SPI_TSHFE_INT_BIT	0x00000008
#define SPI_XCH_BIT			0x00000200
#define SPI_RR_INT_BIT		0x00000010
#define SPI_XCH_MASK		0xfffffdff
#define SPI_EN_BIT			0x00000400

#define EDGE_MIN_LIMIT  (100)
#define EDGE_MAX_LIMIT  (4000)

#define QT_IPAQ

typedef struct {
#ifdef QT_IPAQ
	unsigned short pressure;
#else
	unsigned char pressure;
#endif
	unsigned short x;
	unsigned short y;
#ifdef QT_IPAQ
	unsigned short pad;
#endif
	struct timeval stamp;
} ts_event_t;

#define  MAX_ID 	0x14

#ifdef QT_IPAQ
#define PENUP		0x0
#define PENDOWN		0xffff
#else
#define PENUP		0x0
#define PENDOWN		0xff
#endif

#endif
