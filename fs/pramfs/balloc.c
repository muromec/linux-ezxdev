/*
 * balloc.c
 *
 * The blocks allocation and deallocation routines.
 *
 * Author: Steve Longerbeam <stevel@mvista.com, or source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc.
 * Copyright 2003 Sony Corporation
 * Copyright 2003 Matsushita Electric Industrial Co., Ltd.
 *
 * This software is being distributed under the terms of the GNU General Public
 * License version 2.  Some or all of the technology encompassed by this
 * software may be subject to one or more patents pending as of the date of
 * this notice.  No additional patent license will be required for GPL
 * implementations of the technology.  If you want to create a non-GPL
 * implementation of the technology encompassed by this software, please
 * contact legal@mvista.com for details including licensing terms and fees.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#include <linux/config.h>
#include <linux/fs.h>
#include <linux/pram_fs.h>
#include <linux/locks.h>
#include <linux/quotaops.h>
#include <asm/bitops.h>


/*
 * This just marks in-use the blocks that make up the bitmap.
 * The bitmap must be writeable before calling.
 */
void pram_init_bitmap(struct super_block * sb)
{
	struct pram_super_block * ps = pram_get_super(sb);
	u32* bitmap = pram_get_bitmap(sb);
	int blocks = ps->s_bitmap_blocks;

	memset(bitmap, 0, blocks<<sb->s_blocksize_bits);
	
	while (blocks >= 32) {
		*bitmap++ = 0xffffffff;
		blocks -= 32;
	}

	if (blocks)
		*bitmap = (1<<blocks) - 1;
}


/* Free absolute blocknr */
void pram_free_block (struct super_block * sb, int blocknr)
{
	struct pram_super_block * ps;
	pram_off_t bitmap_block;
	unsigned long flags;
	int bitmap_bnr;
	void* bitmap;
	void* bp;

	lock_super (sb);

	bitmap = pram_get_bitmap(sb);
	/*
	 * find the block within the bitmap that contains the inuse bit
	 * for the block we need to free. We need to unlock this bitmap
	 * block to clear the inuse bit.
	 */
	bitmap_bnr = blocknr >> (3 + sb->s_blocksize_bits);
	bitmap_block = pram_get_block_off(sb, bitmap_bnr);
	bp = pram_get_block(sb, bitmap_block);
	
	pram_lock_block(sb, bp);
	clear_bit(blocknr, bitmap); // mark the block free
	pram_unlock_block(sb, bp);

	ps = pram_get_super(sb);
	pram_lock_super(ps);
	if (blocknr < ps->s_free_blocknr_hint)
		ps->s_free_blocknr_hint = blocknr;
	ps->s_free_blocks_count++;
	pram_unlock_super(ps);

	unlock_super (sb);
}


/*
 * allocate a block and return it's absolute blocknr. Zeroes out the
 * block if zero set.
 */
int pram_new_block (struct super_block * sb, int* blocknr, int zero)
{
	struct pram_super_block * ps;
	pram_off_t bitmap_block;
	unsigned long flags;
	int bnr, bitmap_bnr, errval;
	void* bitmap;
	void* bp;
	
	lock_super (sb);
	ps = pram_get_super(sb);
	bitmap = pram_get_bitmap(sb);

	if (ps->s_free_blocks_count) {
		/* find the oldest unused block */
		bnr = find_next_zero_bit(bitmap,
					 ps->s_blocks_count,
					 ps->s_free_blocknr_hint);
		
		if (bnr < ps->s_bitmap_blocks || bnr >= ps->s_blocks_count) {
			pram_err("no free blocks found!\n");
			errval = -ENOSPC;
			goto fail;
		}

		pram_dbg ("allocating blocknr %d\n", bnr);
		pram_lock_super(ps);
		ps->s_free_blocks_count--;
		ps->s_free_blocknr_hint =
			(bnr < ps->s_blocks_count-1) ? bnr+1 : 0;
                pram_unlock_super(ps);
	} else {
		pram_err("all blocks allocated\n");
		errval = -ENOSPC;
		goto fail;
	}

	/*
	 * find the block within the bitmap that contains the inuse bit
	 * for the unused block we just found. We need to unlock it to
	 * set the inuse bit.
	 */
	bitmap_bnr = bnr >> (3 + sb->s_blocksize_bits);
	bitmap_block = pram_get_block_off(sb, bitmap_bnr);
	bp = pram_get_block(sb, bitmap_block);
	
	pram_lock_block(sb, bp);
	set_bit(bnr, bitmap); // mark the new block in use
	pram_unlock_block(sb, bp);

	if (zero) {
		bp = pram_get_block(sb, pram_get_block_off(sb, bnr));
		pram_lock_block(sb, bp);
		memset(bp, 0, sb->s_blocksize);
		pram_unlock_block(sb, bp);
	}
		
	*blocknr = bnr;
	errval = 0;
 fail:
	unlock_super (sb);
	return errval;
}


unsigned long pram_count_free_blocks (struct super_block * sb)
{
	struct pram_super_block * ps = pram_get_super(sb);
	return ps->s_free_blocks_count;
}
