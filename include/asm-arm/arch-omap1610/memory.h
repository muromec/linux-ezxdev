/*
 * include/asm-arm/arch-omap/memory.h
 *
 * OMAP1610 memory map  
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 *   <source@mvista.com>
 * Copyright (C) 2000 RidgeRun, Inc.
 * Author: RidgeRun, Inc.
 *         Greg Lonnon (glonnon@ridgerun.com) or info@ridgerun.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * This file was derived from 
 * linux/include/asm-arm/arch-intergrator/memory.h
 *
 * Copyright (C) 1999 ARM Limited
 */

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#include <linux/config.h>

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
#define PAGE_OFFSET     (0xC0000000UL)
#define PHYS_OFFSET     (0x10000000UL)
/*
 * The DRAM is contiguous.
 */
#define __virt_to_phys__is_a_macro
#define __virt_to_phys(vpage) ((vpage) - PAGE_OFFSET + PHYS_OFFSET)
#define __phys_to_virt__is_a_macro
#define __phys_to_virt(ppage) ((ppage) + PAGE_OFFSET - PHYS_OFFSET)

#define __virt_to_bus__is_a_macro
#define __virt_to_bus(x) __virt_to_phys(x)
#define __bus_to_virt__is_a_macro
#define __bus_to_virt(x) __phys_to_virt(x)

#define PHYS_TO_NID(addr) (0)
#endif /* __ASM_ARCH_MEMORY_H */
