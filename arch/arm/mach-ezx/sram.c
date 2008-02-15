/*
 * linux/arch/arm/mach-wmmx/sram.c
 *
 * Bulverde Internal Memory stuff 
 *
 * Created:	Sep 05, 2003
 * Copyright:	MontaVista Software Inc.
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
 * Copyright (C) 2005 Motorola Inc.
 *
 * 2005-May-11  copy from arch/arm/mach-pxa/sram.c and porting to EZX platform,  Zhuang Xiaofan
 *
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/config.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/spinlock.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/hardware.h>

#define SRAM_DELAY_STANDBY_TIME 255

static spinlock_t sram_spinlock;
static volatile int sram_locked = 0;

static int __init sram_init (void) {
	/* Enable clock */
	CKEN |= CKEN20_IM;

	/* All the banks are
	 * - Automatic wakeup enabled
	 * - Enter Stand-by after 255 ms 
	 */
	IMPMCR = IMPMCR_PC3_AUTO_MODE |
		 IMPMCR_PC2_AUTO_MODE |
		 IMPMCR_PC1_AUTO_MODE |
		 IMPMCR_PC0_AUTO_MODE |
		 IMPMCR_AW3 | IMPMCR_AW2 |
		 IMPMCR_AW1 | IMPMCR_AW0 |
		 SRAM_DELAY_STANDBY_TIME;

	return 0;
}

int sram_access_obtain(unsigned long *pmem, unsigned long *psize) {

	int flags;
	
	spin_lock_irqsave(&sram_spinlock, flags);
	
	if (sram_locked != 0) 	{
		spin_unlock_irqrestore(&sram_spinlock, flags);
		return -EAGAIN;
	}

	sram_locked = 1;

	spin_unlock_irqrestore(&sram_spinlock, flags);
	
	*pmem = SRAM_MEM_VIRT;
	*psize = SRAM_SIZE;
	
	return 0;
}

int sram_access_release(unsigned long *pmem, unsigned long *psize)
{
	if ((*pmem != SRAM_MEM_VIRT) ||
	    (*psize != SRAM_SIZE)){
		return -EINVAL;
	}
	
	*pmem = 0;
	*psize = 0;

	sram_locked = 0;
	
	return 0;
}

EXPORT_SYMBOL(sram_access_obtain);
EXPORT_SYMBOL(sram_access_release);

__initcall(sram_init);

