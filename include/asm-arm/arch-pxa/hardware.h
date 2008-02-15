/*
 *  linux/include/asm-arm/arch-pxa/hardware.h
 *
 *  Author:	Nicolas Pitre
 *  Created:	Jun 15, 2001
 *  Copyright:	MontaVista Software Inc.
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/*
 * Copyright (C) 2005 Motorola Inc.
 *
 * modified by w19962, for EZX platform
 */

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <linux/config.h>
#include <asm/mach-types.h>

/*
 * These are statically mapped PCMCIA IO space for designs using it as a
 * generic IO bus, typically with ISA parts, hardwired IDE interfaces, etc.
 * The actual PCMCIA code is mapping required IO region at run time.
 */

#ifdef CONFIG_CPU_BULVERDE
#define PCMCIA_IO_0_BASE	0xe6000000
#define PCMCIA_IO_1_BASE	0xe7000000
#else
#define PCMCIA_IO_0_BASE	0xf6000000
#define PCMCIA_IO_1_BASE	0xf7000000
#endif

/*
 * We requires absolute addresses.
 */
#define PCIO_BASE		0

/*
 * Workarounds for at least 2 errata so far require this.
 * The mapping is set in mach-pxa/generic.c.
 */
#define UNCACHED_PHYS_0		0xff000000
#define UNCACHED_ADDR		UNCACHED_PHYS_0


#ifdef CONFIG_CPU_BULVERDE
#define io_p2v(x)	( ((x) | 0xae000000) ^ (~((x) >> 1) & 0x06000000) )
#define io_v2p( x )	( ((x) & 0x51ffffff) ^ ( ((x) & 0x06000000) << 1) )
#else
/*
 * Intel PXA internal I/O mappings:
 *
 * 0x40000000 - 0x41ffffff <--> 0xf8000000 - 0xf9ffffff
 * 0x44000000 - 0x45ffffff <--> 0xfa000000 - 0xfbffffff
 * 0x48000000 - 0x49ffffff <--> 0xfc000000 - 0xfdffffff
 */
#define io_p2v(x)	( ((x) | 0xbe000000) ^ (~((x) >> 1) & 0x06000000) )
#define io_v2p( x )	( ((x) & 0x41ffffff) ^ ( ((x) & 0x06000000) << 1) )
#endif

#ifndef __ASSEMBLY__

#if 0
# define __REG(x)	(*((volatile u32 *)io_p2v(x)))
#else
/*
 * This __REG() version gives the same results as the one above,  except
 * that we are fooling gcc somehow so it generates far better and smaller
 * assembly code for access to contigous registers.  It's a shame that gcc
 * doesn't guess this by itself.
 */
#include <asm/types.h>
typedef struct { volatile u32 offset[4096]; } __regbase;
# define __REGP(x)	((__regbase *)((x)&~4095))->offset[((x)&4095)>>2]
# define __REG(x)	__REGP(io_p2v(x))
#endif

/* Let's kick gcc's ass again... */
# define __REG2(x,y)	\
	( __builtin_constant_p(y) ? (__REG((x) + (y))) \
				  : (*(volatile u32 *)((u32)&__REG(x) + (y))) )

# define __PREG(x)	(io_v2p((u32)&(x)))

#else

# define __REG(x)	io_p2v(x)
# define __PREG(x)	io_v2p(x)

#endif

#include "pxa-regs.h"

#ifndef __ASSEMBLY__

/*
 * GPIO edge detection for IRQs:
 * IRQs are generated on Falling-Edge, Rising-Edge, or both.
 * This must be called *before* the corresponding IRQ is registered.
 * Use this instead of directly setting GRER/GFER.
 */
#define GPIO_FALLING_EDGE       1
#define GPIO_RISING_EDGE        2
#define GPIO_BOTH_EDGES         3
extern void set_GPIO_IRQ_edge( int gpio_nr, int edge_mask );

/*
 * Handy routine to set GPIO alternate functions
 */
extern void set_GPIO_mode( int gpio_mode );

/*
 * return current memclk frequency in units of 10kHz
 */
extern unsigned int get_memclk_frequency_10khz(void);

#endif


/*
 * Implementation specifics
 */
#include "lubbock.h"
#include "mainstone.h"
#include "idp.h"
#include "cerf.h"
#include "ezx.h"

#endif  /* _ASM_ARCH_HARDWARE_H */
