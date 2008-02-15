/*
 *  linux/include/asm-xtensa/io.h
 *  
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 *      Author:         Christian Zankel <zankel@tensilica.com, chris@zankel.net>
 *
 */

#ifdef __KERNEL__
#ifndef __ASM_XTENSA_IO_H
#define __ASM_XTENSA_IO_H

#include <linux/config.h>
#include <asm/byteorder.h>

#include <linux/types.h>
#include <asm/addrspace.h>

/*
 * Change virtual addresses to physical addresses and vv.
 * These are trivial on the 1:1 Linux/Xtensa mapping
 */
extern inline unsigned long virt_to_phys(volatile void * address)
{
	return PHYSADDR((unsigned long)address);
}

extern inline void * phys_to_virt(unsigned long address)
{
	return (void*) CACHED_ADDR(address);
}

/*
 * IO bus memory addresses are also 1:1 with the physical address
 */
extern inline unsigned long virt_to_bus(volatile void * address)
{
	return PHYSADDR((unsigned long)address);
}

extern inline void * bus_to_virt (unsigned long address)
{
	return (void *) CACHED_ADDR(address);
}

/*
 * Change "struct page" to physical address.
 */
#define page_to_phys(page)	((page - mem_map) << PAGE_SHIFT)

extern inline void *ioremap(unsigned long offset, unsigned long size)
{
        return (void *) CACHED_ADDR_IO(offset);
}

extern inline void *ioremap_nocache(unsigned long offset, unsigned long size)
{
        return (void *) BYPASS_ADDR_IO(offset);
}

extern inline void iounmap(void *addr)
{
}


#define _IO_BASE 0

/*
 * Generic I/O
 */

#define readb(addr)	in_8((volatile u8*)(addr))
#define readw(addr)	in_le16((volatile u16*)(addr))
#define readl(addr)	in_le32((volatile u32*)(addr))
#define writeb(b,addr)	out_8((b),(volatile u8*)(addr))
#define writew(b,addr)	out_le16((b),(volatile u16*)(addr))
#define writel(b,addr)	out_le32((b),(volatile u32*)(addr))

/*
 * The raw_readx and raw_writex don't do byte swapping
 */

#define __raw_readb(addr)	(*(volatile unsigned char *)(addr))
#define __raw_readw(addr)	(*(volatile unsigned short *)(addr))
#define __raw_readl(addr)	(*(volatile unsigned int *)(addr))
#define __raw_writeb(v, addr)	(*(volatile unsigned char *)(addr) = (v))
#define __raw_writew(v, addr)	(*(volatile unsigned short *)(addr) = (v))
#define __raw_writel(v, addr)	(*(volatile unsigned int *)(addr) = (v))

/* These are the definitions for the x86 IO instructions
 * inb/inw/inl/outb/outw/outl, the "string" versions
 * insb/insw/insl/outsb/outsw/outsl, and the "pausing" versions
 * inb_p/inw_p/...
 * The macros don't do byte-swapping.
 */

#define inb(port)		in_8((u8 *)((port)+_IO_BASE))
#define outb(val, port)		out_8((val),(u8 *)((unsigned long)(port)+_IO_BASE))
#define inw(port)		in_le16((u16 *)((port)+_IO_BASE))
#define outw(val, port)		out_le16((val),(u16 *)((unsigned long)(port)+_IO_BASE))
#define inl(port)		in_le32((u32 *)((port)+_IO_BASE))
#define outl(val, port)		out_le32((val),(u32 *)((unsigned long)(port)))

#define inb_p(port)		inb((port))
#define outb_p(val, port)	outb((val), (port))
#define inw_p(port)		inw((port))
#define outw_p(val, port)	outw((val), (port))
#define inl_p(port)		inl((port))
#define outl_p(val, port)	outl((val), (port))

extern void outsb(unsigned int port, const void *from, int len);
extern void outsw(unsigned int port, const void *from, int len);
extern void outsl(unsigned int port, const void *from, int len);
extern void insb(unsigned int port, void *from, int len);
extern void insw(unsigned int port, void *from, int len);
extern void insl(unsigned int port, void *from, int len);

#define IO_SPACE_LIMIT ~0

#define memset_io(a,b,c)       memset((void *)(a),(b),(c))
#define memcpy_fromio(a,b,c)   memcpy((a),(void *)(b),(c))
#define memcpy_toio(a,b,c)      memcpy((void *)(a),(b),(c))

/* At this point the Xtensa doesn't provide byte swap instructions */

#ifdef __BIG_ENDIAN
#define in_8(addr) (*(u8*)(addr))
#define in_le16(addr) swab16(*(u16*)(addr))
#define in_le32(addr) swab32(*(u32*)(addr))
#define out_8(b, addr) *(u8*)(addr) = (b)
#define out_le16(b, addr) *(u16*)(addr) = swab16(b)
#define out_le32(b, addr) *(u32*)(addr) = swab32(b)
#else
#define in_8(addr)  (*(u8*)(addr))
#define in_le16(addr) (*(u16*)(addr))
#define in_le32(addr) (*(u32*)(addr))
#define out_8(b, addr) *(u8*)(addr) = (b)
#define out_le16(b, addr) *(u16*)(addr) = (b)
#define out_le32(b, addr) *(u32*)(addr) = (b)
#endif




#endif
#endif
