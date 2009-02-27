/*
 * linux/include/asm-arm/arch-pxa/irq.h
 * 
 * Author:	Nicolas Pitre
 * Created:	Jun 15, 2001
 * Copyright:	MontaVista Software Inc.
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

#ifdef CONFIG_ARCH_EZX
extern int pm_handle_irq(int irq);
#define fixup_irq(x)	pm_handle_irq(x)
#else
#define fixup_irq(x)	(x)
#endif

/*
 * This prototype is required for cascading of multiplexed interrupts.
 * Since it doesn't exist elsewhere, we'll put it here for now.
 */
extern void do_IRQ(int irq, struct pt_regs *regs);
