#ifndef _ASM_PGTABLE_H
#define _ASM_PGTABLE_H

/*
 * include/asm-xtensa/pgtable.h
 *
 * Derived from MIPS.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 *	Authors:	Marc Gauthier
 *			Joe Taylor <joe@tensilica.com, joetylr@yahoo.com>
 */

#include <asm/addrspace.h>
#include <asm/page.h>
#include <asm/xtutil.h>
#include <xtensa/config/core.h>

#ifndef _LANGUAGE_ASSEMBLY

#include <linux/linkage.h>
#include <linux/config.h>


/* With physically tagged caches and appropriately sized caches (to
 * avoid aliasing problems), caches do not need flushing at all.  This
 * limitation is imposed on Xtensa Processors intending to run Linux.
 * Those using Xtensa Processors with potential aliasing problems must
 * rethink these macros.
 */

/* XTFIXME: Perhaps a #error statement would be useful here.  The
 * condition would be based on constants from the CHAL, and would
 * detect potential aliasing problems.
 */

#define flush_dcache_page(page)	        do { } while (0)
#define flush_cache_all()		do { } while (0)
#define flush_cache_mm(mm)		do { } while (0)
#define flush_cache_range(mm,start,end)	do { } while (0)
#define flush_cache_page(vma,page)	do { } while (0)
#define flush_cache_sigtramp(addr)	do { } while (0)
#define flush_page_to_ram(page)		do { } while (0)
#define flush_icache_range(start, end)	flush_cache_all()
#define flush_icache_page(vma, page)	do { } while (0)
#define flush_icache_user_range(vma, page, addr, len)	do { } while (0)

#define invalidate_dcache_range(start, end) 				\
  do {									\
	unsigned long v = (unsigned long)start & ~(XCHAL_DCACHE_LINESIZE - 1);		\
	asm ("99: dhi %0, 0\n\r"					\
	     "    addi %0, %0, " XTSTR(XCHAL_DCACHE_LINESIZE) "\n\r"	\
	     "    blt %0, %1, 99b\n\r" : "+r" (v) : "r" (end));	\
  } while (0)


/* Basically we have the same two-level (which is the logical three level
 * Linux page table layout folded) page tables as the i386.
 */

#endif /* !defined (_LANGUAGE_ASSEMBLY) */

/* XTFIXME:  Need to pull the following constant from the CHAL, if possible. */
#define WIRED_WAY_FOR_PAGE_TABLE  7

/* PMD_SHIFT determines the size of the area a second-level page table can map */
#define PMD_SHIFT	22
#define PMD_SIZE	(1UL << PMD_SHIFT)
#define PMD_MASK	(~(PMD_SIZE-1))

/* PGDIR_SHIFT determines what a third-level page table entry can map */
#define PGDIR_SHIFT	22
#define PGDIR_SIZE	(1UL << PGDIR_SHIFT)
#define PGDIR_MASK	(~(PGDIR_SIZE-1))

/* Entries per page directory level: we use two-level, so
 * we don't really have any PMD directory physically.
 */
#define PTRS_PER_PTE	1024
#define PTRS_PER_PMD	1
#define PTRS_PER_PGD	1024
#define USER_PTRS_PER_PGD	(TASK_SIZE/PGDIR_SIZE)
#define FIRST_USER_PGD_NR	0

/*
 * XTFIXME:  The CHAL should provide memory-map info on local memories and
 * XLMI ports that are identity mapped.  They are typically located just
 * below the kernel space at 0xD0000000, and they are relatively small
 * (today).  We want to verify (with #error directives), however, that
 * these memory-mapped processor features do not overlap the VMALLOC space
 * we define here.  [Sep 11, 2002]
 */

/* 0xC0000000-0xC8000000 reserved for vmalloc; below allows guard memory */
#define VMALLOC_START     0xC0010000
#define VMALLOC_VMADDR(x) ((unsigned long)(x))
#define VMALLOC_END       0xC7FF0000

/* Xtensa Linux config PTE layout:
 *	31-12:	PPN
 *	11-6:	software:
 *		11:	unused
 *		10:	page modified
 *		9:	page accessed
 *		8:	page writeable
 *		7:	page readable
 *		6:	page present
 *	5-4:	RING
 *	3-0:	CA
 *
 * Similar to the Alpha and MIPS ports, we need to keep track of the ref
 * and mod bits in software.  We have a software "yeah you can read
 * from this page" bit, and a hardware one which actually lets the
 * process read from the page.  On the same token we have a software
 * writable bit and the real hardware one which actually lets the
 * process write to the page, this keeps a mod bit via the hardware
 * dirty bit.
 */

#if (XCHAL_MMU_RINGS != 4)
#error Linux build assumes 4 ring levels.
#endif
#if (XCHAL_MMU_RING_BITS != 2)
#error We assume exactly two bits for RING.
#endif
#if (XCHAL_MMU_CA_BITS != 4)
#error We assume exactly four bits for CA.
#endif
#if (XCHAL_MMU_SR_BITS != 0)
#error We have no room for SR bits.
#endif

#define GLOBAL_RING                  3 /* global ring level */
#define USER_RING                    1 /* user ring level */
#define KERNEL_RING                  0 /* kernel ring level */

#define _PAGE_GLOBAL                (GLOBAL_RING<<4)	/* ring: global access (ring=3) */
/*define _PAGE_xxx                  (3<<4)*/		/* ring: ??? access (ring=2) */
#define _PAGE_USER                  (USER_RING<<4)	/* ring: user access (ring=1) */
#define _PAGE_KERNEL                (KERNEL_RING<<4)	/* ring: kernel access (ring=0) */

#define _PAGE_PRESENT               (1<<6)  /* software: (have translation / page not swapped out?) */
#define _PAGE_READ                  (1<<7)  /* software: page readable */
#define _PAGE_WRITE                 (1<<8)  /* software: page writable */
#define _PAGE_ACCESSED              (1<<9)  /* software: page accessed (read) */
#define _PAGE_MODIFIED              (1<<10) /* software: page modified (written/dirty) */

#define _PAGE_VALID                 (1<<0)	/* ca: !! replaced with CA!=invalid */
#define _PAGE_SILENT_READ           (1<<0)	/* ca: !! synonym                 */
#define _PAGE_DIRTY                 (1<<1)	/* ca: !! replaced with CA!=readonly (MIPS dirty bit) */
#define _PAGE_SILENT_WRITE          (1<<1)	/* ca: ... */
#define _CACHE_BYPASS               (0<<2)	/* ca: bypass cache non-speculative (no MP coherency) */
#define _CACHE_CACHABLE_NONCOHERENT (1<<2)	/* ca: cachable speculative     (no MP coherency) */
#define _CACHE_UNCACHED             (2<<2)	/* ca: uncached speculative (no MP coherency) */
#define _CACHE_CACHABLE_SPECIAL     (3<<2)	/* ca: special (isolate, no execute, ...) (no MP coherency) */
#define _CACHE_MASK                 (15<<0)	/* ca: all CA bits */

/*#define _CACHE_CACHABLE_NO_WA       (0<<0)*/  /* R4600 only              */
/*#define _CACHE_CACHABLE_WA          (1<<0)*/  /* R4600 only              */
/*#define _CACHE_CACHABLE_CE          (4<<0)*/  /* R4[04]00 only           */
/*#define _CACHE_CACHABLE_COW         (5<<0)*/  /* R4[04]00 only           */
/*#define _CACHE_CACHABLE_CUW         (6<<0)*/  /* R4[04]00 only           */
/*#define _CACHE_CACHABLE_ACCELERATED (7<<0)*/  /* R10000 only             */

#define __READABLE	(_PAGE_READ | _PAGE_SILENT_READ | _PAGE_ACCESSED)
#define __WRITEABLE	(_PAGE_WRITE | _PAGE_SILENT_WRITE | _PAGE_MODIFIED)

#define _PAGE_CHG_MASK  (PAGE_MASK | _PAGE_ACCESSED | _PAGE_MODIFIED | _CACHE_MASK)

/* PAGE_COPY is historical and identical to PAGE_READONLY.  See Linux
 * Kernel Internals, second edition, by Beck, et al, pg. 76.
 *
 * PAGE_KERNEL_ASM is an assembler version of PAGE_KERNEL.  The
 * assembler doesn't understand __pgprot, so we need a version without
 * it.  PAGE_KERNEL and PAGE_KERNEL_ASM should always have identical
 * content.
 */
#define PAGE_NONE		__pgprot(_PAGE_PRESENT |			    _PAGE_USER   | _CACHE_CACHABLE_NONCOHERENT)
#define PAGE_SHARED		__pgprot(_PAGE_PRESENT | _PAGE_READ | _PAGE_WRITE | _PAGE_USER   | _CACHE_CACHABLE_NONCOHERENT)
#define PAGE_COPY		__pgprot(_PAGE_PRESENT | _PAGE_READ |		    _PAGE_USER   | _CACHE_CACHABLE_NONCOHERENT)
#define PAGE_READONLY		__pgprot(_PAGE_PRESENT | _PAGE_READ |		    _PAGE_USER   | _CACHE_CACHABLE_NONCOHERENT)
#define PAGE_USERIO		__pgprot(_PAGE_PRESENT | _PAGE_READ | _PAGE_WRITE | _PAGE_USER   | _CACHE_BYPASS)
#define PAGE_KERNEL		__pgprot(_PAGE_PRESENT | __READABLE | __WRITEABLE | _PAGE_KERNEL | _CACHE_CACHABLE_NONCOHERENT)
#define PAGE_KERNEL_ASM		        (_PAGE_PRESENT | __READABLE | __WRITEABLE | _PAGE_KERNEL | _CACHE_CACHABLE_NONCOHERENT)
#define PAGE_KERNEL_UNCACHED	__pgprot(_PAGE_PRESENT | __READABLE | __WRITEABLE | _PAGE_KERNEL |_CACHE_BYPASS)

/*
 * On certain configurations of Xtensa MMUs (eg. the initial Linux config),
 * the MMU can't do page protection for execute, and considers that the same as
 * read.  Also, write permissions may imply read permissions.
 * What follows is the closest we can get by reasonable means..
 * See linux/mm/mmap.c for protection_map[] array that uses these definitions.  */
#define __P000	PAGE_NONE	/* private --- */
#define __P001	PAGE_READONLY	/* private --r */
#define __P010	PAGE_COPY	/* private -w- */
#define __P011	PAGE_COPY	/* private -wr */
#define __P100	PAGE_READONLY	/* private x-- */
#define __P101	PAGE_READONLY	/* private x-r */
#define __P110	PAGE_COPY	/* private xw- */
#define __P111	PAGE_COPY	/* private xwr */

#define __S000	PAGE_NONE	/* shared  --- */
#define __S001	PAGE_READONLY	/* shared  --r */
#define __S010	PAGE_SHARED	/* shared  -w- */
#define __S011	PAGE_SHARED	/* shared  -wr */
#define __S100	PAGE_READONLY	/* shared  x-- */
#define __S101	PAGE_READONLY	/* shared  x-r */
#define __S110	PAGE_SHARED	/* shared  xw- */
#define __S111	PAGE_SHARED	/* shared  xwr */

#if !defined (_LANGUAGE_ASSEMBLY)

#define pte_ERROR(e) \
	printk("%s:%d: bad pte %016lx.\n", __FILE__, __LINE__, pte_val(e))
#define pmd_ERROR(e) \
	printk("%s:%d: bad pmd %016lx.\n", __FILE__, __LINE__, pmd_val(e))
#define pgd_ERROR(e) \
	printk("%s:%d: bad pgd %016lx.\n", __FILE__, __LINE__, pgd_val(e))

/*
 * BAD_PAGETABLE is used when we need a bogus page-table, while
 * BAD_PAGE is used for a bogus page.
 *
 * ZERO_PAGE is a global shared page that is always zero: used
 * for zero-mapped memory areas etc..
 */
extern pte_t __bad_page(void);
extern pte_t *__bad_pagetable(void);

extern unsigned long empty_zero_page;
/*extern unsigned long zero_page_mask;*/

#define BAD_PAGETABLE __bad_pagetable()
#define BAD_PAGE __bad_page()

/*  No need to do page-colouring, for now:  -Marc  */
#define ZERO_PAGE(vaddr) (virt_to_page(empty_zero_page))
/*#define ZERO_PAGE(vaddr) \
	(virt_to_page(empty_zero_page + (((unsigned long)(vaddr)) & zero_page_mask)))*/

/* number of bits that fit into a memory pointer */
#define BITS_PER_PTR			(8*sizeof(unsigned long))

/* to align the pointer to a pointer address */
#define PTR_MASK			(~(sizeof(void*)-1))

/*
 * sizeof(void*) == (1 << SIZEOF_PTR_LOG2)
 */
#define SIZEOF_PTR_LOG2			2

extern pgd_t swapper_pg_dir[PAGE_SIZE/sizeof(pgd_t)];
extern pgd_t invalid_pte_table[PAGE_SIZE/sizeof(pgd_t)];
extern pte_t exception_pte_table[PAGE_SIZE/sizeof(pte_t)];

/* to find an entry in a page-table */
#define PAGE_PTR(address) \
((unsigned long)(address)>>(PAGE_SHIFT-SIZEOF_PTR_LOG2)&PTR_MASK&~PAGE_MASK)

extern void load_pgd(unsigned long pg_dir);

/*
 * Conversion functions: convert a page and protection to a page entry,
 * and a page entry and page directory to the page they refer to.
 */
extern inline unsigned long pmd_page(pmd_t pmd)
{
	return pmd_val(pmd);
}

extern inline void pmd_set(pmd_t * pmdp, pte_t * ptep)
{
	pmd_val(*pmdp) = (((unsigned long) ptep) & PAGE_MASK);
}

extern inline int pte_none(pte_t pte)    { return !pte_val(pte); }
extern inline int pte_present(pte_t pte) { return pte_val(pte) & _PAGE_PRESENT; }

/* Certain architectures need to do special things when pte's
 * within a page table are directly modified.  Thus, the following
 * hook is made available.
 */
extern inline void set_pte(pte_t *ptep, pte_t pteval)
{
	*ptep = pteval;
}

extern inline void pte_clear(pte_t *ptep)
{
	set_pte(ptep, __pte(0));
}

/*
 * Empty pgd/pmd entries point to the invalid_pte_table.
 */
extern inline int pmd_none(pmd_t pmd)
{
	return pmd_val(pmd) == (unsigned long) invalid_pte_table;
}

extern void * high_memory;

extern inline int pmd_bad(pmd_t pmd)
{
	return ((pmd_page(pmd) > (unsigned long) high_memory) ||
	        (pmd_page(pmd) < PAGE_OFFSET));
}

extern inline int pmd_present(pmd_t pmd)
{
	return (pmd_val(pmd) != ((unsigned long) invalid_pte_table));
}

extern inline void pmd_clear(pmd_t *pmdp)
{
	pmd_val(*pmdp) = ((unsigned long) invalid_pte_table);
}

/*
 * The "pgd_xxx()" functions here are trivial for a folded two-level
 * setup: the pgd is never bad, and a pmd always exists (as it's folded
 * into the pgd entry)
 */
extern inline int pgd_none(pgd_t pgd)		{ return 0; }
extern inline int pgd_bad(pgd_t pgd)		{ return 0; }
extern inline int pgd_present(pgd_t pgd)	{ return 1; }
extern inline void pgd_clear(pgd_t *pgdp)	{ }

#define pte_page(x)		(mem_map+(unsigned long)((pte_val(x) >> PAGE_SHIFT)))

/*
 * The following only work if pte_present() is true.
 * Undefined behaviour if not..
 */
extern inline int pte_read(pte_t pte)	{ return pte_val(pte) & _PAGE_READ; }
extern inline int pte_write(pte_t pte)	{ return pte_val(pte) & _PAGE_WRITE; }
extern inline int pte_dirty(pte_t pte)	{ return pte_val(pte) & _PAGE_MODIFIED; }
extern inline int pte_young(pte_t pte)	{ return pte_val(pte) & _PAGE_ACCESSED; }

extern inline pte_t pte_wrprotect(pte_t pte)
{
	pte_val(pte) &= ~(_PAGE_WRITE | _PAGE_SILENT_WRITE);
	return pte;
}

extern inline pte_t pte_rdprotect(pte_t pte)
{
	pte_val(pte) &= ~(_PAGE_READ | _PAGE_SILENT_READ);
	return pte;
}

extern inline pte_t pte_mkclean(pte_t pte)
{
	pte_val(pte) &= ~(_PAGE_MODIFIED|_PAGE_SILENT_WRITE);
	return pte;
}

extern inline pte_t pte_mkold(pte_t pte)
{
	pte_val(pte) &= ~(_PAGE_ACCESSED|_PAGE_SILENT_READ);
	return pte;
}

extern inline pte_t pte_mkwrite(pte_t pte)
{
	pte_val(pte) |= _PAGE_WRITE;
	if (pte_val(pte) & _PAGE_MODIFIED)
		pte_val(pte) |= _PAGE_SILENT_WRITE;
	return pte;
}

extern inline pte_t pte_mkread(pte_t pte)
{
	pte_val(pte) |= _PAGE_READ;
	if (pte_val(pte) & _PAGE_ACCESSED)
		pte_val(pte) |= _PAGE_SILENT_READ;
	return pte;
}

extern inline pte_t pte_mkdirty(pte_t pte)
{
	pte_val(pte) |= _PAGE_MODIFIED;
	if (pte_val(pte) & _PAGE_WRITE)
		pte_val(pte) |= _PAGE_SILENT_WRITE;
	return pte;
}

extern inline pte_t pte_mkyoung(pte_t pte)
{
	pte_val(pte) |= _PAGE_ACCESSED;
	if (pte_val(pte) & _PAGE_READ)
		pte_val(pte) |= _PAGE_SILENT_READ;
	return pte;
}

/*
 * Conversion functions: convert a page and protection to a page entry,
 * and a page entry and page directory to the page they refer to.
 *
 * WARNING: Assembly code in arch/xtensa/kernel/handlers.S mimics this
 * macro definition.  If you change this definition, you should review
 * the port in handlers.S.  Within that file, just search for
 * 'mk_pte'.
 */
#define mk_pte(page, pgprot)						\
({									\
	pte_t   __pte;							\
									\
	pte_val(__pte) = ((unsigned long)(page - mem_map) << PAGE_SHIFT) | \
	                 pgprot_val(pgprot);				\
									\
	__pte;								\
})

extern inline pte_t mk_pte_phys(unsigned long physpage, pgprot_t pgprot)
{
	return __pte(((physpage & PAGE_MASK) - PAGE_OFFSET) | pgprot_val(pgprot));
}

extern inline pte_t pte_modify(pte_t pte, pgprot_t newprot)
{
	return __pte((pte_val(pte) & _PAGE_CHG_MASK) | pgprot_val(newprot));
}

#define page_pte(page) page_pte_prot(page, __pgprot(0))

/* to find an entry in a kernel page-table-directory */
#define pgd_offset_k(address) pgd_offset(&init_mm, address)

#define pgd_index(address)	((address) >> PGDIR_SHIFT)

/* to find an entry in a page-table-directory */
extern inline pgd_t *pgd_offset(struct mm_struct *mm, unsigned long address)
{
	return mm->pgd + pgd_index(address);
}

/* Find an entry in the second-level page table.. */
extern inline pmd_t *pmd_offset(pgd_t *dir, unsigned long address)
{
	return (pmd_t *) dir;
}

/* Find an entry in the third-level page table.. */ 
extern inline pte_t *pte_offset(pmd_t * dir, unsigned long address)
{
	return (pte_t *) (pmd_page(*dir)) +
	       ((address >> PAGE_SHIFT) & (PTRS_PER_PTE - 1));
}

#endif /*  !defined (_LANGUAGE_ASSEMBLY) */


#ifdef _LANGUAGE_ASSEMBLY

/* Assembly macro _PGD_INDEX is the same as C version pgd_index().
 *
 * Entry Conditions:
 *	a3 = unsigned long address
 *
 * Exit Conditions:
 *	a3 = page directory index
 *	All other registers are preserved
 */

	.macro	_PGD_INDEX	reg
	extui	\reg, \reg, PGDIR_SHIFT, 32-PGDIR_SHIFT
	.endm


/* Assembly macro _PGD_OFFSET is the same as C version pgd_offset().
 *
 * Entry Conditions:
 *	a2 = struct mm_struct *mm
 *	a3 = unsigned long address
 *
 * Exit Conditions:
 *	a2 = pgd_offset(mm, addr)
 *	a3 = pgd_index(addr)
 */

	.macro  _PGD_OFFSET	mm, addr
	l32i	\mm, \mm, MM_PGD
	_PGD_INDEX	\addr
	addx4	\mm, \addr, \mm
	.endm


/* Assembly macro _PMD_OFFSET is the same as C version pmd_offset().
 * Currently, it's just a type case in C and a no-op in assembly.
 * There is no second-level page directory in this version.
 *
 * Entry Conditions:
 *	a2 = pgd_t *pgd
 *	a3 = unsigned long address
 *
 * Exit Conditions:
 *	None.  Macro does nothing.  */

	.macro  _PMD_OFFSET	pgd, addr
	.endm


#endif  /* _LANGUAGE_ASSEMBLY */


#if !defined (_LANGUAGE_ASSEMBLY)

/*
 * Initialize new page directory with pointers to invalid ptes
 */
extern void pgd_init(unsigned long page);

extern void __bad_pte(pmd_t *pmd);
extern void __bad_pte_kernel(pmd_t *pmd);

#define pte_free_kernel(pte)    free_pte_fast(pte)
#define pte_free(pte)           free_pte_fast(pte)
#define pgd_free(pgd)           free_pgd_fast(pgd)
#define pgd_alloc(mm)           get_pgd_fast()

extern int do_check_pgt_cache(int, int);
extern void paging_init(void);

#define SWP_TYPE(x)		(((x).val >> 1) & 0x3f)
#define SWP_OFFSET(x)		((x).val >> 8)
#define SWP_ENTRY(type,offset)	((swp_entry_t) { ((type) << 1) | ((offset) << 8) })
#define pte_to_swp_entry(pte)	((swp_entry_t) { pte_val(pte) })
#define swp_entry_to_pte(x)	((pte_t) { (x).val })


/* Needs to be defined here and not in linux/mm.h, as it is arch dependent */
#define PageSkip(page)		(0)
#define kern_addr_valid(addr)	(1)

/* TLB operations. */

/* XTFIXME: The following constants should come from the CHAL. */
#define ITLB_WAYS_LOG2      XCHAL_ITLB_WAY_BITS
#define DTLB_WAYS_LOG2      XCHAL_DTLB_WAY_BITS
#define ITLB_PROBE_SUCCESS  (1 << ITLB_WAYS_LOG2)
#define DTLB_PROBE_SUCCESS  (1 << DTLB_WAYS_LOG2)


extern inline void set_rasid_register (unsigned long val)
{
	__asm__ __volatile__ (" wsr   %0, "XTSTR(RASID)"\n\t"
			      " isync\n"
			      : : "a" (val));
}

extern inline unsigned long get_rasid_register (void)
{
	unsigned long tmp;
	__asm__ __volatile__ (" rsr   %0, "XTSTR(RASID)"\n\t"
			      " isync\n"
			      : "=a" (tmp));
	return tmp;
}

extern inline unsigned long itlb_probe(unsigned long addr)
{
	unsigned long tmp;
	__asm__ __volatile__("pitlb  %0, %1\n\t"
			     : "=a" (tmp)
			     : "a" (addr));
	return tmp;
}

extern inline unsigned long dtlb_probe(unsigned long addr)
{
	unsigned long tmp;
	__asm__ __volatile__("pdtlb  %0, %1\n\t"
			     : "=a" (tmp)
			     : "a" (addr));
	return tmp;
}

extern inline void invalidate_itlb_entry (unsigned long probe)
{
	__asm__ __volatile__("iitlb  %0\n\t"
			     "isync\n\t"
			     : : "a" (probe));
}

extern inline void invalidate_dtlb_entry (unsigned long probe)
{
	__asm__ __volatile__("idtlb  %0\n\t"
			     "dsync\n\t"
			     : : "a" (probe));
}

extern inline void set_itlbcfg_register (unsigned long val)
{
	__asm__ __volatile__("wsr  %0, "XTSTR(ITLBCFG)"\n\t"
			     "isync\n\t"
			     : : "a" (val));
}

extern inline void set_dtlbcfg_register (unsigned long val)
{
	__asm__ __volatile__("wsr  %0, "XTSTR(DTLBCFG)"\n\t"
			     "dsync\n\t"
			     : : "a" (val));
}

extern inline void set_ptevaddr_register (unsigned long val)
{
	__asm__ __volatile__(" wsr  %0, "XTSTR(PTEVADDR)"\n\t"
			     " isync\n"
			     : : "a" (val));
}

extern inline unsigned long read_ptevaddr_register (void)
{
	unsigned long tmp;
	__asm__ __volatile__("rsr  %0, "XTSTR(PTEVADDR)"\n\t"
			     "dsync\n"
			     : "=a" (tmp));
	return tmp;
}

extern inline void write_dtlb_entry (pte_t entry, int way)
{
	__asm__ __volatile__("wdtlb  %1, %0\n\t"
			     "dsync\n\t"
			     : : "r" (way), "r" (entry) );
}

extern inline void write_itlb_entry (pte_t entry, int way)
{
	__asm__ __volatile__("witlb  %1, %0\n\t"
			     "isync\n\t"
			     : : "r" (way), "r" (entry) );
}

extern inline unsigned long read_dtlb_virtual (int way)
{
	/* DO NOT USE THIS FUNCTION.  This instruction isn't part of
	 * the Xtensa ISA and exists only for test purposes on T1040.
	 * You may find it helpful for MMU debugging, however.
	 *
	 * 'at' is the unmodified input register
	 * 'as' is the output register, as follows (specific to the Linux config):
	 *
	 *      as[31..12] contain the virtual address
	 *      as[11..08] are meaningless
	 *      as[07..00] contain the asid
	 */

	unsigned long tmp;
	__asm__ __volatile__("rdtlb0  %0, %1\n\t"
			     : "=a" (tmp), "+a" (way)   );
	return tmp;
}

extern inline unsigned long read_dtlb_translation (int way)
{
	/* DO NOT USE THIS FUNCTION.  This instruction isn't part of
	 * the Xtensa ISA and exists only for test purposes on T1040.
	 * You may find it helpful for MMU debugging, however.
	 *
	 * 'at' is the unmodified input register
	 * 'as' is the output register, as follows (specific to the Linux config):
	 *
	 *      as[31..12] contain the physical address
	 *      as[11..04] are meaningless
	 *      as[03..00] contain the cache attribute
	 */

	unsigned long tmp;
	__asm__ __volatile__("rdtlb1  %0, %1\n\t"
			     : "=a" (tmp), "+a" (way)   );
	return tmp;
}

extern inline unsigned long read_itlb_virtual (int way)
{
	/* DO NOT USE THIS FUNCTION.  This instruction isn't part of
	 * the Xtensa ISA and exists only for test purposes on T1040.
	 * You may find it helpful for MMU debugging, however.
	 *
	 * 'at' is the unmodified input register
	 * 'as' is the output register, as follows (specific to the Linux config):
	 *
	 *      as[31..12] contain the virtual address
	 *      as[11..08] are meaningless
	 *      as[07..00] contain the asid
	 */

	unsigned long tmp;
	__asm__ __volatile__("ritlb0  %0, %1\n\t"
			     : "=a" (tmp), "+a" (way)   );
	return tmp;
}

extern inline unsigned long read_itlb_translation (int way)
{
	/* DO NOT USE THIS FUNCTION.  This instruction isn't part of
	 * the Xtensa ISA and exists only for test purposes on T1040.
	 * You may find it helpful for MMU debugging, however.
	 *
	 * 'at' is the unmodified input register
	 * 'as' is the output register, as follows (specific to the Linux config):
	 *
	 *      as[31..12] contain the physical address
	 *      as[11..04] are meaningless
	 *      as[03..00] contain the cache attribute
	 */

	unsigned long tmp;
	__asm__ __volatile__("ritlb1  %0, %1\n\t"
			     : "=a" (tmp), "+a" (way)   );
	return tmp;
}


extern inline void invalidate_page_table (void)
{
	invalidate_dtlb_entry (WIRED_WAY_FOR_PAGE_TABLE);
}

extern inline void invalidate_itlb_mapping (unsigned address)
{
	unsigned long tlb_entry;
	if ((tlb_entry = itlb_probe (address)) & ITLB_PROBE_SUCCESS)
		invalidate_itlb_entry (tlb_entry);
}

extern inline void invalidate_dtlb_mapping (unsigned address)
{
	unsigned long tlb_entry;
	if ((tlb_entry = dtlb_probe (address)) & DTLB_PROBE_SUCCESS)
		invalidate_dtlb_entry (tlb_entry);
}


/* The document "Linux Cache Flush Architecture" offers hints on
 * function update_mmu_cache, but it's misleading for Xtensa
 * processors. */

/* The kernel (in mm/memory.c) often invokes this macro when a page
 * previously wasn't mapped.  In handle_2nd_level_miss() when no page
 * is mapped, we map the exception_pte_table into the Page Table to
 * generate another exception that exposes the type of access.  After
 * mapping in the appropriate page (often by calling the kernel's
 * handle_mm_fault()), we need to remove the mapping of
 * exception_pte_table from the Page Table.  This macro does this
 * function.  The 2nd-level miss handler will later fill in the
 * correct mapping. */

extern inline void update_mmu_cache(struct vm_area_struct * vma,
	unsigned long address, pte_t pte)
{
	invalidate_page_table();
}


/* XTFIXME:  Possible optimization opportunity here. */
#include <asm-generic/pgtable.h>


#endif /* !defined (_LANGUAGE_ASSEMBLY) */

#define io_remap_page_range remap_page_range

/* No page table caches to init */
#define pgtable_cache_init()	do { } while (0)

#endif /* _ASM_PGTABLE_H */
