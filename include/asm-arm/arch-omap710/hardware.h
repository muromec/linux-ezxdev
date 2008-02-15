/*
 *  linux/include/asm-arm/arch-omap710/hardware.h
 *
 * BRIEF MODULE DESCRIPTION
 *   OMAP710 hardware map
 *
 * Copyright (C) 2001, 2002 RidgeRun, Inc. (http://www.ridgerun.com)
 * Author: RidgeRun, Inc.
 *         Greg Lonnon (glonnon@ridgerun.com) or info@ridgerun.com
 *         Brian Mahaffy (brianm@ridgerun.com) or info@ridgerun.com
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
 */
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H
 
#include <asm/arch/sizes.h>
#include <linux/config.h>

/*
 * Where in virtual memory the IO devices (timers, system controllers
 * and so on)
 * ???_BASE is the virtual address
 * ???_START is the physical address
 */
#define IO_BASE			0xFFFB0000                 // VA of IO 
#define IO_SIZE			0x00040000                 // How much?
#define IO_START		0xFFFB0000                 // PA of IO

// Next, the nCS1 portion of the memory map as indicated
// by the omap710 documentation will be the memory addresses
// used when referencing either the SD/MMC card slot or the
// CompactFlash card slot. As a practicle matter the s/w
// only uses the first few address of the 32MB nCS1 range.
#define nCS1_BASE  0xE4000000    // VA of IO 
#define nCS1_SIZE  SZ_4K + SZ_4K // How much?
#define nCS1_START 0x04000000    // PA of IO

/* maps in the FPGA registers and the ETHR registers */
#define FPGA_BASE      0xE8000000                 // VA
#define FPGA_SIZE      SZ_4K                      // SIZE
#define FPGA_START     0x08000000                 // PA   
#define FPGA_ETHR_BASE 0xE8000200 /* drivers/net/smc9194.c uses this */


#define EPLD_BASE       0xEBFFF000                // VA
#define EPLD_SIZE       SZ_16K                    // size
#define EPLD_START      0x0BFFF000                // PA

#define OMAP710_SRAM_BASE       0xD0000000                // VA
#define OMAP710_SRAM_SIZE       ( SZ_128K + SZ_64K )      // size
#define OMAP710_SRAM_START      0x20000000                // PA

#define OMAP_FLASH_0_BASE	0xD8000000	// VA
#define OMAP_FLASH_0_SIZE	SZ_16M

/*
 * Where's the flush address (for flushing D and I cache?)
 */
#define FLUSH_BASE              0xdf000000
#define FLUSH_BASE_PHYS		FLASH_BASE

#ifndef __ASSEMBLER__

#define PCIO_BASE		0

/*
 * RAM definitions
 */
#define MAPTOPHYS(a)		((unsigned long)(a) - PAGE_OFFSET)
#define KERNTOPHYS(a)		((unsigned long)(&a))
#define KERNEL_BASE             (0x10008000)  
#endif

/* macro to get at IO space when running virtually */
#define IO_ADDRESS(x) ((x)) 

#define PLATFORM_ID                     0x00000045

/*
 *  memory map
 */
#define FLASH_BASE           0x00000000
#define SDRAM_BASE           0x10000000

#include <asm/arch/omap710-regs.h>

#endif //__ASM_ARCH_HARDWARE_H
