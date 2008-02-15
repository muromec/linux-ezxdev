/*
 *  linux/include/asm-arm/arch-mx2ads/hardware.h
 *
 *  Copyright (C) 1999 ARM Limited.
 *  Copyright (C) 2004 MontaVista Software Inc.
 *  Copyright (C) 2004 Motorola Semiconductors HK Ltd
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */
#ifndef __ASM_ARCH_HARDWARE_H__
#define __ASM_ARCH_HARDWARE_H__

#include <asm/sizes.h>

/* External crystal clocks definitions */
#define OSC_32768HZ_INPUT
#undef OSC_32000HZ_INPUT 	

/* Shared per clock frequencies */
#define MX2_PRECLK1_FREQ 48000000 /*UART, GPT, PWM*/
#define MX2_PRECLK2_FREQ 48000000 /*MMC/SD, SPI*/

/* Internal modules reserved space */

#define MX2_IO_BASE             	0x10000000
#define MX2_IO_SIZE             	SZ_1M
#define MX2_IO_IOBASE           	0xE4000000

#define MX2_CSI_BASE             	0x80000000
#define MX2_CSI_SIZE             	SZ_4K
#define MX2_CSI_IOBASE           	0xE4100000

#define MX2_BMI_BASE             	0xA0000000
#define MX2_BMI_SIZE             	SZ_4K
#define MX2_BMI_IOBASE           	0xE4101000

/* PCMCIA/CF IO and memory space defintions */
#define PCMCIA_IOMEM_PARTITION_SIZE	SZ_1K

#define PCMCIA_IOMEM_SPACE		(4 * PCMCIA_IOMEM_PARTITION_SIZE)

#define MX2_PCMCIA_IO_BASE           	0xD4000000
#define MX2_PCMCIA_IO_SIZE           	PCMCIA_IOMEM_SPACE
#define MX2_PCMCIA_IO_IOBASE           	0xE4102000

/*
 * According to MX2 spec, EIM register is mapped to 0xDF001000,
 * not in register area!! it's so strange, I have to map EIM another space
 * it is include NFC address space(DF003000 DF003FFF)
 * Frank Li Change MX2ADS_EMI_BASE from 0xDF001000 to 0xDF000000
*/

#define MX2_NFC_BASE             	0xDF000000
#define MX2_NFC_SIZE             	SZ_1M
#define MX2_NFC_IOBASE           	0xE4400000

/* PCMCIA/CF Controller module */
#define MX2_PCMCIA_BASE             	0xDF002000
#define MX2_PCMCIA_SIZE             	SZ_4K
#define MX2_PCMCIA_IOBASE           	0xE4203000

/* Mx2 ADS SDRAM is from 0xC0000000, 64M, we use the last 16M as RAM disk for usb*/

#define	MX2ADS_SDRAM_DISK_BASE		0xC3000000	/* SDRAM disk base (last 16M of SDRAM) */
#define MX2ADS_SDRAM_DISK_SIZE  	SZ_16M
#define	MX2ADS_SDRAM_DISK_IOBASE	0xE0000000

#define MX2ADS_PER_BASE         	0xCC000000	/* cs1   for periperal device IO */
#define MX2ADS_PER_SIZE         	SZ_16M
#define MX2ADS_PER_IOBASE       	0xE1000000

/* This Macro only for map register space to 0xe4000000 */
#define IO_ADDRESS(x) 		(((x) - MX2_IO_BASE) + MX2_IO_IOBASE)

#define NFC_IO_ADDRESS(x) 	(((x) - MX2_NFC_BASE)+ MX2_NFC_IOBASE)

#ifndef __ASSEMBLY__

/*
 * The __REGx() versions give the same results as the ones above,  except
 * that we are fooling gcc somehow so it generates far better and smaller
 * assembly code for access to contigous registers.
 *
 * The 4096 bytes macro size is set specifically to take
 * advantage of the ARM ISA "register +/- 12-bit immediate offset addressing
 * mode". See the ARM ARM "5.2 Addressing Mode 2 - Load and Store Word or
 * Unsigned Byte" for details on "Load and Store Word or Unsigned Byte -
 * Immediate offset" addressing modes.
 */
#include <asm/types.h>

typedef struct {
	volatile u8 offset[4096];
} __regbase8;
#define __REG8(x)	((__regbase8 *)((x)&~4095))->offset[(x)&4095]

typedef struct {
	volatile u16 offset[2048];
} __regbase16;
#define __REG16(x)	((__regbase16 *)((x)&~4095))->offset[((x)&4095)>>1]

typedef struct {
	volatile u32 offset[1024];
} __regbase32;
#define __REG32(x)	((__regbase32 *)((x)&~4095))->offset[((x)&4095)>>2]

/* Let's kick gcc's ass again... */
#define __REG32_2(x,y)	\
	( __builtin_constant_p(y) ? (__REG32((x) + (y))) \
				  : (*(volatile u32 *)((u32)&__REG32(x) + (y))) )

#else

#define __REG8(x)	(x)
#define __REG16(x)	(x)
#define __REG32(x)	(x)
#define __REG32_2(x,y)	((x)+(y))

#endif

#include <asm/arch/mx2-regs.h>

#endif
