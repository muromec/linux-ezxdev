#ifndef _MIPS_PGTABLE_3LEVEL_H
#define _MIPS_PGTABLE_3LEVEL_H

/*
 * Not really a 3 level page table but we follow most of the x86 PAE code.
 */

#define PMD_SHIFT	21
#define PTRS_PER_PTE	512
#define PTRS_PER_PMD	1
#define PTRS_PER_PGD	2048
#define PGD_ORDER	1

#if !defined (_LANGUAGE_ASSEMBLY)
#define pte_ERROR(e) \
	printk("%s:%d: bad pte %p(%08lx%08lx).\n", __FILE__, __LINE__, &(e), (e).pte_high, (e).pte_low)
#define pmd_ERROR(e) \
	printk("%s:%d: bad pmd %p(%016Lx).\n", __FILE__, __LINE__, &(e), pmd_val(e))
#define pgd_ERROR(e) \
	printk("%s:%d: bad pgd %p(%016Lx).\n", __FILE__, __LINE__, &(e), pgd_val(e))

/*
 * MIPS32 Note
 * pte_low contains the 12 low bits only.  This includes the 6 lsb bits
 * which contain software control bits, and the next 6 attribute bits 
 * which are actually written in the entrylo[0,1] registers (G,V,D,Cache Mask).
 * pte_high contains the 36 bit physical address and the 6 hardware 
 * attribute bits (G,V,D, Cache Mask). The entry is already fully setup
 * so in the tlb refill handler we do not need to shift right 6.
 */

/* Rules for using set_pte: the pte being assigned *must* be
 * either not present or in a state where the hardware will
 * not attempt to update the pte.  In places where this is
 * not possible, use pte_get_and_clear to obtain the old pte
 * value and then use set_pte to update it.  -ben
 */
static inline void set_pte(pte_t *ptep, pte_t pte)
{
	ptep->pte_high = (pte.pte_high & ~0x3f) | ((pte.pte_low>>6) & 0x3f);
	ptep->pte_low = pte.pte_low;
}

static inline int pte_same(pte_t a, pte_t b)
{
	return a.pte_low == b.pte_low && a.pte_high == b.pte_high;
}

#define pte_page(x)    (mem_map+(((x).pte_high >> 6)))
#define pte_none(x)    (!(x).pte_low && !(x).pte_high)

static inline pte_t 
__mk_pte(unsigned long page_nr, pgprot_t pgprot)
{
	pte_t pte;

	pte.pte_high = (page_nr << 6) | (pgprot_val(pgprot) >> 6);
	pte.pte_low = pgprot_val(pgprot);
	return pte;
}
#endif

#endif /* _MIPS_PGTABLE_3LEVEL_H */
