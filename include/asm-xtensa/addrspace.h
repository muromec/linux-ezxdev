#ifndef __ASM_XTENSA_ADDRSPACE_H
#define __ASM_XTENSA_ADDRSPACE_H

/*
 * include/asm-xtensa/addrspace.h
 *
 * Definitions for configurable address spaces of Xtensa MMUs.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 *	Authors:	Joe Taylor <joe@tensilica.com, joetylr@yahoo.com>
 *			Marc Gauthier
 *			Chris Zankel <zankel@tensilica.com, chris@zankel.net>
 */


/*
 *  WARNING: Not all Xtensa architectures have these definitions.
 *  Only minimally needed for Xtensa.
 */

/*  Get the compile-time HAL:  */
#include <xtensa/config/core.h>

/*  Define user VM space; not used yet, just here for reference for now:  */
#if XCHAL_SEG_MAPPABLE_VADDR != 0
#error "But, but... don't I put user space at vaddr zero, dang it?"
#endif
#if XCHAL_SEG_MAPPABLE_SIZE < 0x80000000
#error "Please, pretty please, let users have half the vm space..."
#endif

/* USER_VM_SIZE does not necessarily equal TASK_SIZE.  We bumped
 * TASK_SIZE down to 0x4000000 to simplify the handling of windowed
 * call instructions (currently limited to a range of 1 GByte).  User
 * tasks may very well reclaim the VM space from 0x40000000 to
 * 0x7fffffff in the future, so we do not want the kernel becoming
 * accustomed to having any of its stuff (e.g., page tables) in this
 * region.  This VM region is no-man's land for now. */

#define USER_VM_START		XCHAL_SEG_MAPPABLE_VADDR
#define USER_VM_SIZE		0x80000000

/*  Size of page table:  */
#define PGTABLE_SIZE_BITS	(32 - XCHAL_MMU_MIN_PTE_PAGE_SIZE + 2)
#define PGTABLE_SIZE		(1L << PGTABLE_SIZE_BITS)

/*  All kernel-mappable space:  */
#define KERNEL_ALLMAP_START	(USER_VM_START + USER_VM_SIZE)
#define KERNEL_ALLMAP_SIZE	(XCHAL_SEG_MAPPABLE_SIZE - KERNEL_ALLMAP_START)

/*  Carve out page table at start of kernel-mappable area:  */
#if KERNEL_ALLMAP_SIZE < PGTABLE_SIZE
#error "Gimme some space for page table!"
#endif
#define PGTABLE_START		KERNEL_ALLMAP_START
/*  Remaining kernel-mappable space:  */
#define KERNEL_MAPPED_START	(KERNEL_ALLMAP_START + PGTABLE_SIZE)
#define KERNEL_MAPPED_SIZE	(KERNEL_ALLMAP_SIZE - PGTABLE_SIZE)
#if KERNEL_MAPPED_SIZE < 0x01000000	/* 16 MB is arbitrary for now */
#error "Shouldn't the kernel have at least *some* mappable space?"
#endif

/*
 *  Some constants used elsewhere, but perhaps only in Xtensa header
 *  files, so maybe we can get rid of some and access compile-time HAL
 *  directly...
 *
 *  XTFIXME:  this assumes that system RAM is located at the
 *  very start of the kernel segments !!
 */
#define KERNEL_VM_LOW           XCHAL_KSEG_CACHED_VADDR
#define KERNEL_VM_HIGH          XCHAL_KSEG_BYPASS_VADDR
#define KERNEL_SPACE            XCHAL_KSEG_CACHED_VADDR

/*
 * Returns the physical/virtual addresses of the kernel space
 * (works with the cached kernel segment only, which is the
 *  one normally used for kernel operation).
 */

//			PHYSICAL	BYPASS		CACHED
//
//  bypass vaddr	bypass paddr	*		cached vaddr
//  cached vaddr	cached paddr	bypass vaddr	*
//  bypass paddr	*		bypass vaddr	cached vaddr
//  cached paddr	*		bypass vaddr	cached vaddr
//  other		*		*		*
 
#define PHYSADDR(a)							     \
    (((a) >= XCHAL_KSEG_BYPASS_VADDR					     \
      && (a) < XCHAL_KSEG_BYPASS_VADDR + XCHAL_KSEG_BYPASS_SIZE) ?	     \
        (a) - XCHAL_KSEG_BYPASS_VADDR + XCHAL_KSEG_BYPASS_PADDR :	     \
        ((a) >= XCHAL_KSEG_CACHED_VADDR					     \
         && a < XCHAL_KSEG_CACHED_VADDR + XCHAL_KSEG_CACHED_SIZE) ?	     \
            (a) - XCHAL_KSEG_CACHED_VADDR + XCHAL_KSEG_CACHED_PADDR : (a))

#define BYPASS_ADDR(a)							     \
    (((a) >= XCHAL_KSEG_BYPASS_PADDR					     \
      && (a) < XCHAL_KSEG_BYPASS_PADDR + XCHAL_KSEG_BYPASS_SIZE) ?	     \
        (a) - XCHAL_KSEG_BYPASS_PADDR + XCHAL_KSEG_BYPASS_VADDR :	     \
        ((a) >= XCHAL_KSEG_CACHED_PADDR					     \
         && (a) < XCHAL_KSEG_CACHED_PADDR + XCHAL_KSEG_CACHED_SIZE) ?	     \
            (a) - XCHAL_KSEG_CACHED_PADDR + XCHAL_KSEG_BYPASS_VADDR :	     \
            ((a) >= XCHAL_KSEG_CACHED_VADDR				     \
             && (a) < XCHAL_KSEG_CACHED_VADDR + XCHAL_KSEG_CACHED_SIZE) ?    \
                (a) - XCHAL_KSEG_CACHED_VADDR + XCHAL_KSEG_BYPASS_VADDR : (a))
 
#define CACHED_ADDR(a)							     \
    (((a) >= XCHAL_KSEG_BYPASS_PADDR					     \
      && (a) < XCHAL_KSEG_BYPASS_PADDR + XCHAL_KSEG_BYPASS_SIZE) ?	     \
        (a) - XCHAL_KSEG_BYPASS_PADDR + XCHAL_KSEG_CACHED_VADDR :	     \
        ((a) >= XCHAL_KSEG_CACHED_PADDR					     \
         && (a) < XCHAL_KSEG_CACHED_PADDR + XCHAL_KSEG_CACHED_SIZE) ?	     \
            (a) - XCHAL_KSEG_CACHED_PADDR + XCHAL_KSEG_CACHED_VADDR :	     \
            ((a) >= XCHAL_KSEG_BYPASS_VADDR				     \
             && (a) < XCHAL_KSEG_BYPASS_VADDR + XCHAL_KSEG_BYPASS_SIZE) ?    \
                (a) - XCHAL_KSEG_BYPASS_VADDR + XCHAL_KSEG_CACHED_VADDR : (a))

#define PHYSADDR_IO(a)							     \
    (((a) >= XCHAL_KIO_BYPASS_VADDR					     \
      && (a) < XCHAL_KIO_BYPASS_VADDR + XCHAL_KIO_BYPASS_SIZE) ?	     \
        (a) - XCHAL_KIO_BYPASS_VADDR + XCHAL_KIO_BYPASS_PADDR :		     \
        ((a) >= XCHAL_KIO_CACHED_VADDR					     \
         && a < XCHAL_KIO_CACHED_VADDR + XCHAL_KIO_CACHED_SIZE) ?	     \
            (a) - XCHAL_KIO_CACHED_VADDR + XCHAL_KIO_CACHED_PADDR : (a))
 
#define BYPASS_ADDR_IO(a)						     \
    (((a) >= XCHAL_KIO_BYPASS_PADDR					     \
      && (a) < XCHAL_KIO_BYPASS_PADDR + XCHAL_KIO_BYPASS_SIZE) ?	     \
        (a) - XCHAL_KIO_BYPASS_PADDR + XCHAL_KIO_BYPASS_VADDR :		     \
        ((a) >= XCHAL_KIO_CACHED_PADDR					     \
         && (a) < XCHAL_KIO_CACHED_PADDR + XCHAL_KIO_CACHED_SIZE) ?	     \
            (a) - XCHAL_KIO_CACHED_PADDR + XCHAL_KIO_BYPASS_VADDR :	     \
            ((a) >= XCHAL_KIO_CACHED_VADDR				     \
             && (a) < XCHAL_KIO_CACHED_VADDR + XCHAL_KIO_CACHED_SIZE) ?	     \
                (a) - XCHAL_KIO_CACHED_VADDR + XCHAL_KIO_BYPASS_VADDR : (a))
 
#define CACHED_ADDR_IO(a)						     \
    (((a) >= XCHAL_KIO_BYPASS_PADDR					     \
      && (a) < XCHAL_KIO_BYPASS_PADDR + XCHAL_KIO_BYPASS_SIZE) ?	     \
        (a) - XCHAL_KIO_BYPASS_PADDR + XCHAL_KIO_CACHED_VADDR :		     \
        ((a) >= XCHAL_KIO_CACHED_PADDR					     \
         && (a) < XCHAL_KIO_CACHED_PADDR + XCHAL_KIO_CACHED_SIZE) ?	     \
            (a) - XCHAL_KIO_CACHED_PADDR + XCHAL_KIO_CACHED_VADDR :	     \
            ((a) >= XCHAL_KIO_BYPASS_VADDR				     \
             && (a) < XCHAL_KIO_BYPASS_VADDR + XCHAL_KIO_BYPASS_SIZE) ?	     \
                (a) - XCHAL_KIO_BYPASS_VADDR + XCHAL_KIO_CACHED_VADDR : (a))

#if 0

#define PHYSADDR(a)		((((unsigned long)(a)) & (XCHAL_KSEG_CACHED_SIZE-1)) + XCHAL_KSEG_CACHED_PADDR)
#define VIRTADDR(a)             ((__typeof__(a))(((unsigned long)(a) & (XCHAL_KSEG_CACHED_SIZE-1)) + XCHAL_KSEG_CACHED_VADDR))
#define BYPASS_ADDR(a)		((((unsigned long)(a)) & (XCHAL_KSEG_CACHED_SIZE-1)) + XCHAL_KSEG_BYPASS_VADDR)
#define CACHED_ADDR(a)		((((unsigned long)(a)) & (XCHAL_KSEG_CACHED_SIZE-1)) + XCHAL_KSEG_CACHED_VADDR)

#endif


#endif /* __ASM_XTENSA_ADDRSPACE_H */





