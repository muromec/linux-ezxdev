#ifndef _MIPS_PGTABLE_2LEVEL_H
#define _MIPS_PGTABLE_2LEVEL_H

/*
 * traditional mips two-level paging structure:
 */

#if defined(CONFIG_64BIT_PHYS_ADDR)
#define PMD_SHIFT	21
#define PTRS_PER_PTE	512
#define PTRS_PER_PMD	1
#define PTRS_PER_PGD	2048
#define PGD_ORDER	1
#else
#define PMD_SHIFT	22
#define PTRS_PER_PTE	1024
#define PTRS_PER_PMD	1
#define PTRS_PER_PGD	1024
#define PGD_ORDER	0
#endif

#if !defined (_LANGUAGE_ASSEMBLY)
#define pte_ERROR(e) \
	printk("%s:%d: bad pte %08lx.\n", __FILE__, __LINE__, (e).pte_low)
#define pmd_ERROR(e) \
	printk("%s:%d: bad pmd %08lx.\n", __FILE__, __LINE__, pmd_val(e))
#define pgd_ERROR(e) \
	printk("%s:%d: bad pgd %p(%016Lx).\n", __FILE__, __LINE__, &(e), pgd_val(e))


/* Certain architectures need to do special things when pte's
 * within a page table are directly modified.  Thus, the following
 * hook is made available.
 */
static inline void set_pte(pte_t *ptep, pte_t pteval)
{
	*ptep = pteval;
}
static inline int pte_none(pte_t pte)    { return !(pte.pte_low); }

#ifdef CONFIG_CPU_VR41XX
#define pte_page(x)  (mem_map+((unsigned long)(((x).pte_low >> (PAGE_SHIFT+2)))))
#define __mk_pte(page_nr,pgprot) __pte(((page_nr) << (PAGE_SHIFT+2)) | pgprot_val(pgprot))
#else
#define pte_page(x)  (mem_map+((unsigned long)(((x).pte_low >> PAGE_SHIFT))))
#define __mk_pte(page_nr,pgprot) __pte(((page_nr) << PAGE_SHIFT) | pgprot_val(pgprot))
#endif

#endif

#endif /* _MIPS_PGTABLE_2LEVEL_H */
