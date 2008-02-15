/*
 *  linux/include/asm-arm/arch-mx2ads/memory.h
 *
 *  Copyright (C) 1999 ARM Limited
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/*
 * Task size: 3GB
 */
#if (defined CONFIG_XIP_ROM)
#define TASK_SIZE	(0xbf000000UL)
#else
#define TASK_SIZE	(0xc0000000UL)
#endif
#define TASK_SIZE_26	(0x04000000UL)

/*
 * This decides where the kernel will search for a free chunk of vm
 * space during mmap's.
 */
#define TASK_UNMAPPED_BASE (0x40000000UL)

/*
 * Page offset: 3GB
 */
/* Note
 * This should be hot-swapable with a CONFIG_MX1ADS_SDRAM
 * or something similar, so we can swap between SRAM and
 * SDRAM running kernel.
 */

#define PAGE_OFFSET	(0xc0000000UL)
#define PHYS_OFFSET	(0xc0000000UL)

/*
 * On mx2, the sdram/sram is contiguous
 */
#define __virt_to_phys__is_a_macro
#define __virt_to_phys(vpage) (vpage)
#define __phys_to_virt__is_a_macro
#define __phys_to_virt(ppage) (ppage)

/*
 * Virtual view <-> DMA view memory address translations
 * virt_to_bus: Used to translate the virtual address to an
 *              address suitable to be passed to set_dma_addr
 * bus_to_virt: Used to convert an address for DMA operations
 *              to an address that the kernel can use.
 */
#define __virt_to_bus__is_a_macro
#define __virt_to_bus(x)	(x)
#define __bus_to_virt__is_a_macro
#define __bus_to_virt(x)	(x)

#define PHYS_TO_NID(addr)	(0)

#endif
