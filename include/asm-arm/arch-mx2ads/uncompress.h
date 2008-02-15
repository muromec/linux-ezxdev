/*
 * linux/include/asm-arm/arch-mx2ads/uncompress.h
 *
 * Author: <source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Initially based on linux-2.4.21-rmk2/include/asm-arm/arch-mx1ads/uncompress.h
 * Copyright (C) 1999 ARM Limited
 * Copyright (C) Shane Nay (shane@minirl.com)
 *
 */

#ifndef __ASM_ARCH_UNCOMPRESS_H__
#define __ASM_ARCH_UNCOMPRESS_H__

#define UART1_BASE_ADDR 0x1000A000
#define UART1_URXD	(*(volatile u32 *)(UART1_BASE_ADDR+0x00))
#define UART1_UTXD	(*(volatile u8 *)(UART1_BASE_ADDR+0x40))
#define UART1_UCR1	(*(volatile u32 *)(UART1_BASE_ADDR+0x80))
#define UART1_UCR2	(*(volatile u32 *)(UART1_BASE_ADDR+0x84))
#define UART1_UCR3	(*(volatile u32 *)(UART1_BASE_ADDR+0x88))
#define UART1_UCR4	(*(volatile u32 *)(UART1_BASE_ADDR+0x8C))
#define UART1_UFCR	(*(volatile u32 *)(UART1_BASE_ADDR+0x90))
#define UART1_USR1	(*(volatile u32 *)(UART1_BASE_ADDR+0x94))
#define UART1_USR2	(*(volatile u32 *)(UART1_BASE_ADDR+0x98))
#define UART1_UESC	(*(volatile u32 *)(UART1_BASE_ADDR+0x9C))
#define UART1_UTIM	(*(volatile u32 *)(UART1_BASE_ADDR+0xA0))
#define UART1_UBIR	(*(volatile u32 *)(UART1_BASE_ADDR+0xA4))
#define UART1_UBMR	(*(volatile u32 *)(UART1_BASE_ADDR+0xA8))
#define UART1_UBRC	(*(volatile u32 *)(UART1_BASE_ADDR+0xAC))
#define UART1_ONEMS	(*(volatile u32 *)(UART1_BASE_ADDR+0xB0))
#define UART1_UTS	(*(volatile u32 *)(UART1_BASE_ADDR+0xB4))

#define TXFEBIT         (14)

/*
 * This does not append a newline
 */
static void
puts(const char *s)
{
	while (*s) {
		while (!(UART1_USR2 & (1 << TXFEBIT)))
			barrier();

		UART1_UTXD = *s;

		if (*s == '\n') {
			while (!(UART1_USR2 & (1 << TXFEBIT)))
				barrier();

			UART1_UTXD = '\r';
		}
		s++;
	}

	while (!(UART1_USR2 & (1 << TXFEBIT)))
		barrier();
}

/*
 * nothing to do
 */
#define arch_decomp_setup()

#define arch_decomp_wdog()

#endif
