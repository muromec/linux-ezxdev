/*
 * include/asm-arm/arch-omap/irq.h
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 *   <source@mvista.com>
 * Initially based on linux/include/asm-arm/arch-sa1100/irq.h
 * Author: Nicolas Pitre
 */

#define fixup_irq(x)	(x)

/*
 * This prototype is required for cascading of multiplexed interrupts.
 * Since it doesn't exist elsewhere, we'll put it here for now.
 */
extern void do_IRQ(int irq, struct pt_regs *regs);

#define IFL_LOWLEVEL 1
#define IFL_HIGHLEVEL 2
#define IFL_FRONTEDGE 4
#define IFL_REAREDGE 8

extern int omap1610_tune_irq(unsigned int irq, u32 flags);
