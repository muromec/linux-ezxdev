#ifndef _ASM_GENERIC_MMU_H
#define _ASM_GENERIC_MMU_H

/*
 * Some cpu's (mips, ppc) have 32-bit virtual address space but
 * larger (36-bit) physical space. So the include/asm-[*]/mmu.h
 * have the option of redefining phys_addr_t. Also, fixup_bigphys_addr
 * can be redefined to "convert" a physaddr represented in 32 bits
 * to a phys_addr_t. How that's actually done is very platform-
 * specific, and is a hack no matter how it's done. It's a stop-gap
 * approach until the day the entire kernel deals with physical
 * addresses consistently.
 */
typedef unsigned long phys_addr_t;

#define fixup_bigphys_addr(addr, size) (addr)

#endif /* _ASM_GENERIC_MMU_H */
