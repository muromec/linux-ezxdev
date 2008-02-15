/*
 * arch/arm/mach-pxa/sram.c
 *
 * Bulverde SRAM management
 *
 * Author: source@mvista.com
 * Copyright 2003, 2004 MontaVista Software Inc.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/config.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/slab.h>

#include <asm/hardware.h>

#define SRAM_DELAY_STANDBY_TIME 255

typedef struct {
	struct list_head list;
	unsigned long start;
	unsigned long size;
} sram_block_t;

static spinlock_t sram_lock = SPIN_LOCK_UNLOCKED;
static LIST_HEAD(sram_blocks);

static int __init sram_init(void) {
	/* Enable clock */
	CKEN |= CKEN20_IM;

	/* All the banks are:
	 * - automatic wakeup enabled
	 * - will enter Stand-by after 255 ms
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

unsigned long sram_alloc(unsigned long size)
{
	struct list_head *list;
	sram_block_t *sb, *new;
	unsigned long start = SRAM_MEM_VIRT, before_block = 0, flags;

	spin_lock_irqsave(&sram_lock, flags);
	list_for_each(list, &sram_blocks) {
		sb = list_entry(list, sram_block_t, list);
		if (start + size <= sb->start) {
			/* Found space before this block */
			before_block++;
			break;
		}
		else
			start = sb->start + sb->size;
	}
	if (start + size > SRAM_MEM_VIRT + SRAM_SIZE) {
		/* No space after the last block */
		start = 0;
		goto end;
	}
	/* OK to allocate from start */
	new = kmalloc(sizeof (sram_block_t), GFP_KERNEL);
	if (!new) {
		start = 0;
		goto end;
	}
	INIT_LIST_HEAD(&new->list);
	new->start = start;
	new->size = size;
	if (before_block)
		__list_add(&new->list, list->prev, list);
	else
		list_add_tail(&new->list, &sram_blocks);
  end:
	spin_unlock_irqrestore(&sram_lock, flags);
	return start;
}

void sram_free(unsigned long addr)
{
	sram_block_t *sb;
	unsigned long flags;
	struct list_head *list;

	spin_lock_irqsave(&sram_lock, flags);
	list_for_each(list, &sram_blocks) {
		sb = list_entry(list, sram_block_t, list);
		if (sb->start == addr) {
			list_del(&sb->list);
			kfree(sb);
			break;
		}
	}
	spin_unlock_irqrestore(&sram_lock, flags);
}

EXPORT_SYMBOL(sram_alloc);
EXPORT_SYMBOL(sram_free);

__initcall(sram_init);
