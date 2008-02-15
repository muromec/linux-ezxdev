#ifndef __MMU_H
#define __MMU_H

#ifdef CONFIG_64BIT_PHYS_ADDR
typedef unsigned long long phys_addr_t;
extern phys_addr_t (*__ioremap_fixup)(phys_addr_t, phys_addr_t);
#define fixup_bigphys_addr(addr, size) __ioremap_fixup(addr, size)
#else
#include <asm-generic/mmu.h>
#endif


/* Default "unsigned long" context */
typedef unsigned long mm_context_t;

#endif
