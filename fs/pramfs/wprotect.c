/*
 * wprotect.c
 *
 * Write protection for the filesystem pages.
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
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/pram_fs.h>
#include <linux/mm.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>

// init_mm.page_table_lock must be held before calling!
static void pram_page_writeable(unsigned long addr, int rw)
{
	pgd_t *pgdp;
	pmd_t *pmdp;
	pte_t *ptep;
	
	pgdp = pgd_offset_k(addr);
	if (!pgd_none(*pgdp)) {
		pmdp = pmd_offset(pgdp, addr);
		if (!pmd_none(*pmdp)) {
			pte_t pte;
			ptep = pte_offset(pmdp, addr);
			pte = *ptep;
			if (pte_present(pte)) {
				pte = rw ? pte_mkwrite(pte) :
					pte_wrprotect(pte);
				set_pte(ptep, pte);
			}
		}
	}
}


// init_mm.page_table_lock must be held before calling!
void pram_writeable(void * vaddr, unsigned long size, int rw)
{
        unsigned long addr = (unsigned long)vaddr & PAGE_MASK;
	unsigned long end = (unsigned long)vaddr + size;
	unsigned long start = addr;

	do {
		pram_page_writeable(addr, rw);
		addr += PAGE_SIZE;
	} while (addr && (addr < end));


	/*
	 * NOTE: flush_tlb_page/range() routines must support
	 * flushing memory regions owned by init_mm, or the
	 * flush will fail, causing kernel page protection
	 * faults.
	 *
	 * NOTE2: we will always flush just one page (one TLB
	 * entry) except possibly in one case: when a new
	 * filesystem is initialized at mount time, when pram_read_super
	 * calls pram_lock_range to make the super block, inode
	 * table, and bitmap writeable.
	 */
	if (end <= start + PAGE_SIZE)
		flush_tlb_page(find_vma(&init_mm,start), start);
	else
		flush_tlb_range(&init_mm, start, end);
}
