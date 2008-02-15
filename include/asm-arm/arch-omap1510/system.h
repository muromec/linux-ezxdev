/*
 *  linux/include/asm-arm/arch-ti925/system.h
 *
 *  Copyright (C) 1999 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *  Copyright (C) 2000 RidgeRun, Inc. (http://www.ridgerun.com)
 *                Greg Lonnon (glonnon@ridgerun.com)
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include "hardware.h"

static void arch_idle(void)
{
	if (!hlt_counter) {
		int flags;
		local_irq_save(flags);
		if (!current->need_resched)
			cpu_do_idle();
		local_irq_restore(flags);
	}
}

static void arch_reset(char mode)
{
	*(volatile u16 *)(ARM_RSTCT1) = 1;
}
#endif
