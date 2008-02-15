/*
 *  linux/include/asm-arm/arch-ti925/ti925_evm/hardware.h
 *
 * BRIEF MODULE DESCRIPTION
 *   TI925 hardware map
 *
 * Copyright (C) 2000 RidgeRun, Inc. (http://www.ridgerun.com)
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
 */
#ifndef __ASM_ARCH_TI925_EVM_HARDWARE_H
#define __ASM_ARCH_TI925_EVM_HARDWARE_H

#include <asm/arch/sizes.h>

/*
 * Where in virtual memory the IO devices (timers, system controllers
 * and so on)
 */
#define IO_BASE			0xE0000000                 // VA of IO 
#define IO_SIZE			0x01000000                 // How much?
#define IO_START		0x08000000                 // PA of IO

#define TI925_RHEA_BASE	        0x00000000                 // VA of IO 
#define TI925_RHEA_SIZE		0x00100000                 // How much?
#define TI925_RHEA_START	0x08100000                 // PA of IO

#define TI925_ETHER_BASE	0x00000000                 // VA of IO 
#define TI925_ETHER_SIZE	0x00001000                 // How much?
#define TI925_ETHER_START	0x14000000                 // PA of IO

/*
 * Similar to above, but for PCI addresses (memory, IO, Config and the
 * V3 chip itself).  WARNING: this has to mirror definitions in platform.h
 */
#define PCI_MEMORY_VADDR        0xD0000000
#define PCI_CONFIG_VADDR        0xE1000000
#define PCI_V3_VADDR            0xE2000000
#define PCI_IO_VADDR            0xE3000000

/*
 * Where's the flush address (for flushing D and I cache?)
 */
#define FLUSH_BASE              0xdf000000
#define FLUSH_BASE_PHYS		TI925_FLASH_BASE

#ifndef __ASSEMBLER__

#define PCIO_BASE		IO_BASE

/*
 * RAM definitions
 */
#define MAPTOPHYS(a)		((unsigned long)(a) - PAGE_OFFSET)
#define KERNTOPHYS(a)		((unsigned long)(&a))
#define KERNEL_BASE		(0x04008000)

#endif

/* macro to get at IO space when running virtually */
#define IO_ADDRESS(x) ((x) + IO_BASE) 

#define PLATFORM_ID                     0x00000045

/*
 *  TI925 memory map
 */
#define TI925_FLASH_BASE           0x00000000
#define TI925_SDRAM_BASE           0x04000000
#define TI925_HDR0_SDRAM_BASE      0x80000000

/*
 *  Flash
 */
#define TI925_FLASH_START          TI925_FLASH_BASE
#define TI925_FLASH_SIZE           0x04000000
#define TI925_FLASH_BUSWIDTH       2
#define TI925_FLASH_SETUP_REGISTER 0x08000B00

/*
 * Traffic Controller
 */
#define TI925_TC_BASE              0x08000000
#define TI925_TC_START             TI925_TC_BASE
#define TI925_TC_SIZE              SZ_4K

#define TI925_TC_SETUP_REGISTER    (TI925_TC_START)

 

/* ----------------------------------------------------------------------------
 *  TI925 system registers
 * ----------------------------------------------------------------------------
 */

#define TI925_DPLL_BASE            0x08100000    /* Digital Phased Lock Loop */
#define TI925_CT_BASE              0x0810a000	 /* Counter/Timers */
#define TI925_IC_BASE              0x08102000	 /* Interrupt Controller */
#define TI925_RTC_BASE             0x15000000	 /* Real Time Clock */
#define TI925_UART0_BASE           0x08106000	 /* UART 0 */
#define TI925_UART1_BASE           0x08104000	 /* UART 1 */
#define TI925_KBD_BASE             0x18000000	 /* Keyboard */
#define TI925_MOUSE_BASE           0x19000000	 /* Mouse */

/* 
 *  LED's & Switches
 * 
 */
#define TI925_HIO_FPGA_START       0x11000000
#define TI925_HIO_FPGA_SIZE        SZ_4K

#define TI925_HOST_RESET           (TI925_HIO_FPGA_START + 0x0)
#define TI925_WATCHDOG_CLEAR       (TI925_HIO_FPGA_START + 0x2)
#define TI925_FP_LED               (TI925_HIO_FPGA_START + 0x3)
#define TI925_SPI_DIV              (TI925_HIO_FPGA_START + 0x4)
#define TI925_DIP_SWITCHES         (TI925_HIO_FPGA_START + 0x5)

#define TI925_LCD_PANEL_CONTROL    (TI925_HIO_FPGA_START + 0x7)

/*
 * LCD Panel
 */

#define TI925_TC_CIR_BUF_TOP_ADDR  (TI925_TC_START+0x400)
#define TI925_TC_CIR_BUF_END_ADDR  (TI925_TC_START+0x500)
#define TI925_TC_CIR_BUF_READ_ADDR (TI925_TC_START+0x600)

#define TI925_LCD_START       TI925_TC_BASE
#define TI925_LCD_CONTROL     (TI925_LCD_START+0x700)
#define TI925_LCD_STATUS      (TI925_LCD_START+0x710)
#define TI925_LCD_RESERVED1   (TI925_LCD_START+0x720)
#define TI925_LCD_RESERVED2   (TI925_LCD_START+0x730)
#define TI925_LCD_TIMING0     (TI925_LCD_START+0x740)
#define TI925_LCD_TIMING1     (TI925_LCD_START+0x750)
#define TI925_LCD_TIMING2     (TI925_LCD_START+0x760)
#define TI925_LCD_RESERVED3   (TI925_LCD_START+0x770)


/* ---------------------------------------------------------------------------
 *  TI925 Interrupt Controllers
 * ---------------------------------------------------------------------------
 * 
 *  Offsets from interrupt controller base 
 * 
 *  Core Module interrupt controller base is
 * 
 * 	TI925_HDR_IC 
 * 
 */
#define IRQ_ITR                 0x00
#define IRQ_MIR                 0x20
#define IRQ_SIR_IRQ             0x40
#define IRQ_SIR_FIQ             0x50
#define IRQ_CONTROL_REG         0x60
#define IRQ_ISR                 0x70
#define IRQ_ILR0                0x80

/*
 * Interrupts for the HIO
 */
#define TI925_HIO_IC            0x11000000
#define IRQ_HIO_IMR             0x0e
#define IRQ_HIO_ISR             0x0c

/*
 *  Timer definitions
 * 
 */

/* note: TI docs have timer0 mapped to timer 1 and timer1 to timer 2, sorry
 * the docs are inconsistent, so I am just follwoing the docs... -gdl
 */
#define TI925_TIMER0_BASE          (TI925_CT_BASE + 0x2000)
#define TI925_TIMER1_BASE          (TI925_CT_BASE)

/*
 *  Digital Phase Locked Loop (DPLL) definitions. See ti925_dpll.pdf.
 */
#define TI925_DPLL_DCER      (TI925_DPLL_BASE)
#define TI925_DPLL_CTL       (TI925_DPLL_BASE + 4)
#define TI925_DCER_SBI       0x001
#define TI925_DCER_LCD       0x002                 
#define TI925_DCER_DSP       0x004                 
#define TI925_DCER_IC        0x008
#define TI925_DCER_GPIO      0x010                 
#define TI925_DCER_Timer2    0x020
#define TI925_DCER_Timer1    0x040
#define TI925_DCER_UART2     0x080
#define TI925_DCER_UART1     0x100

#endif


