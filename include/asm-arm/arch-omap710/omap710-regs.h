/*
 *  linux/include/asm-arm/arch-omap710/omap710-regs.h
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
#ifndef __ASM_ARCH_OMAP_REGS_H
#define __ASM_ARCH_OMAP_REGS_H
 
#include <asm/arch/sizes.h>
#include <linux/config.h>

/*
  There are 2 sets of general I/O -->
  1. GPIO (shared between ARM & DSP, configured by ARM)
  2. MPUIO which can be used only by the ARM.
  
  Base address FFFB:5000 is where the ARM accesses the MPUIO control registers
  (see 7.2.2 of the TRM for MPUIO reg definitions).
  
  Base address E101:5000 is reserved for ARM access of the same MPUIO control
  regs, but via the DSP I/O map.  This address is unavailable on 710.
  
  Base address FFFC:E000 is where the ARM accesses the GPIO config registers
  directly via its own peripheral bus.
  
  Base address E101:E000 is where the ARM can access the same GPIO config
  registers, but the access takes place through the ARM port interface (called
  API or MPUI) via the DSP's peripheral bus (DSP I/O space).
  
  Therefore, the ARM should setup the GPIO regs thru the FFFC:E000 addresses
  instead of the E101:E000 addresses.  The DSP has only read access of the pin
  control register, so this may explain the inability to write to E101:E018.
  Try accessing pin control reg at FFFC:E018.
 */
#define OMAP710_GPIO_BASE       0xfffbc800
#define OMAP710_GPIO_START      OMAP710_GPIO_BASE
#define OMAP710_GPIO_SIZE       SZ_4K * 2

#define OMAP710_MCBSP1_BASE     0xE1011000               // VA
#define OMAP710_MCBSP1_SIZE     SZ_4K                    // size
#define OMAP710_MCBSP1_START    0xE1011000               // PA

#define OMAP710_MCBSP3_BASE     0xE1017000               // VA
#define OMAP710_MCBSP3_SIZE     SZ_4K                    // size
#define OMAP710_MCBSP3_START    0xE1017000               // PA

/*
 * PERSEUS_CONF
 */
#define PERSEUS_CONF            0xfffe1000
#define HWCONFIG_REG_BASE	0xfffe1000 /* REVISIT: aka used in arch/ide.h */
#define PERSEUS_DEV_ID0         (PERSEUS_CONF + 0x00)
#define PERSEUS_DEV_ID1         (PERSEUS_CONF + 0x02)
#define GSM_EXT_LEAD_CONF       (PERSEUS_CONF + 0x04)
#define GSM_EXT_ARM_CONF        (PERSEUS_CONF + 0x06)
#define GSM_ASIC_CONF           (PERSEUS_CONF + 0x08)
#define GSM_IO_CONF             (PERSEUS_CONF + 0x0A)
#define PERSEUS_OSC32K_GSM_CONF (PERSEUS_CONF + 0x0C)
#define GSM_MCU_TRACE_SW        (PERSEUS_CONF + 0x0E)
#define PERSEUS_DIE_ID0         (PERSEUS_CONF + 0x10)
#define PERSEUS_DIE_ID1         (PERSEUS_CONF + 0x12)
#define PERSEUS_DIE_ID2         (PERSEUS_CONF + 0x14)
#define PERSEUS_DIE_ID3         (PERSEUS_CONF + 0x16)
/* skip 0x18 */
#define PERSEUS_MODE2           (PERSEUS_CONF + 0x1A)
#define PERSEUS_MODE1           (PERSEUS_CONF + 0x1C)
#define PERSEUS_IO_CONF0        (PERSEUS_CONF + 0x1E)
#define PERSEUS_IO_CONF1        (PERSEUS_CONF + 0x20)
#define PERSEUS_IO_CONF2        (PERSEUS_CONF + 0x22)
#define PERSEUS_IO_CONF3        (PERSEUS_CONF + 0x24)
#define PERSEUS_IO_CONF4        (PERSEUS_CONF + 0x26)
#define PERSEUS_IO_CONF5        (PERSEUS_CONF + 0x28)
#define PERSEUS_IO_CONF6        (PERSEUS_CONF + 0x2A)
#define PERSEUS_IO_CONF7        (PERSEUS_CONF + 0x2C)
#define PERSEUS_IO_CONF8        (PERSEUS_CONF + 0x2E)
#define PERSEUS_IO_CONF9        (PERSEUS_CONF + 0x30)
#define PERSEUS_AUDIO_CONF      (PERSEUS_CONF + 0x32)
/* skip 0x34 */
#define PERSEUS_OSC32K_MPU_CONF (PERSEUS_CONF + 0x36)
#define FLASH_PROTECT           (PERSEUS_CONF + 0x38)

/* ----------------------------------------------------------------------------
 *  system registers
 * ----------------------------------------------------------------------------
 */

#define UART1_BASE        0xfffb0000    /* UART1 */
#define UART_MODEM	0xfffb0000    /* UART1 */
#define UART2_BASE        0xfffb0800    /* UART2 */
#define UART_IRDA	0xfffb0800    /* UART2 */
#define CF_CONTROLLER_BASE	0xfffbe000 /* REVISIT: used in arch/ide.h */
#define UART3_BASE        0xfffce800  
#define UART_MODEM2	0xfffce800  
#define COM_MCBSP2_BASE   0xffff1000    /* Com McBSP2 */
#define AUDIO_MCBSP_BASE  0xffff1800    /* Audio McBSP2 */
#define RTC_BASE          0xfffb4800    /* RTC */
#define ARMIO_BASE        0xfffb5000    /* keyboard/gpio */


/*
 * RTC
 */
#define OMAP710_RTC_SEC         RTC_BASE+0x00
#define OMAP710_RTC_MIN         RTC_BASE+0x01
#define OMAP710_RTC_HOUR        RTC_BASE+0x02
#define OMAP710_RTC_DAY         RTC_BASE+0x03
#define OMAP710_RTC_MONTH       RTC_BASE+0x04
#define OMAP710_RTC_YEAR        RTC_BASE+0x05
#define OMAP710_RTC_DOW         RTC_BASE+0x06
#define OMAP710_RTC_ALRM_SEC    RTC_BASE+0x08
#define OMAP710_RTC_ALRM_MIN    RTC_BASE+0x09
#define OMAP710_RTC_ALRM_HOUR   RTC_BASE+0x0A
#define OMAP710_RTC_ALRM_DAY    RTC_BASE+0x0B
#define OMAP710_RTC_ALRM_MONTH  RTC_BASE+0x0C
#define OMAP710_RTC_ARRM_YEAR   RTC_BASE+0x0D
#define OMAP710_RTC_CTRL        RTC_BASE+0x10
#define OMAP710_RTC_STAT        RTC_BASE+0x11
#define OMAP710_RTC_INTR        RTC_BASE+0x12
#define OMAP710_RTC_COMP_LSB    RTC_BASE+0x13
#define OMAP710_RTC_COMP_MSB    RTC_BASE+0x14


/*
 * LCD Panel
 */
#define OMAP710_LCD_BASE        0xFFFEC000        // VA of the LCD controller
#define OMAP710_LCD_CONTROL     (OMAP710_LCD_BASE)
#define OMAP710_LCD_TIMING0     (OMAP710_LCD_BASE+0x4)
#define OMAP710_LCD_TIMING1     (OMAP710_LCD_BASE+0x8)
#define OMAP710_LCD_TIMING2     (OMAP710_LCD_BASE+0xc)
#define OMAP710_LCD_STATUS      (OMAP710_LCD_BASE+0x10)
#define OMAP710_LCD_SUBPANEL    (OMAP710_LCD_BASE+0x14)

#define OMAP_LCD_CONTROL        OMAP710_LCD_CONTROL

/* 2.9.6 Traffic Controller Memory Interface Registers: */
#define EMIF_REG_BASE			0xfffecc00	/* REVISIT: any use? */
#define OMAP_IMIF_PRIO_REG		0xfffecc00
#define OMAP_EMIFS_PRIO_REG		0xfffecc04
#define OMAP_EMIFF_PRIO_REG		0xfffecc08
#define OMAP_EMIFS_CONFIG_REG		0xfffecc0c
#define OMAP_EMIFS_CS0_CONFIG		0xfffecc10
#define OMAP_EMIFS_CS1_CONFIG		0xfffecc14
#define OMAP_EMIFS_CS2_CONFIG		0xfffecc18
#define OMAP_EMIFS_CS3_CONFIG		0xfffecc1c
#define OMAP_EMIFF_SDRAM_CONFIG		0xfffecc20
#define OMAP_EMIFF_MRS			0xfffecc24
#define OMAP_TIMEOUT1			0xfffecc28
#define OMAP_TIMEOUT2			0xfffecc2c
#define OMAP_TIMEOUT3			0xfffecc30
#define OMAP_ENDIANISM			0xfffecc34

/* 2.9.10 EMIF Slow Interface Configuration Register (EMIFS_CONFIG_REG): */
#define OMAP_EMIFS_CONFIG_FR		(1 << 4)
#define OMAP_EMIFS_CONFIG_PDE		(1 << 3)
#define OMAP_EMIFS_CONFIG_PWD_EN	(1 << 2)
#define OMAP_EMIFS_CONFIG_BM		(1 << 1)
#define OMAP_EMIFS_CONFIG_WP		(1 << 0)


/*
 * Memory chunk set aside for the Framebuffer in SRAM
 */
#define SRAM_FRAMEBUFFER_MEMORY OMAP710_SRAM_BASE

/* 
 * DMA
 */

#define OMAP710_DMA_BASE 0xFFFED800
#define OMAP_DMA_BASE    OMAP710_DMA_BASE

/* Global Register selection */
#define NO_GLOBAL_DMA_ACCESS 0

/* Channel select field
 * NOTE: all other channels are linear, chan0 is 0, chan1 is 1, etc...
 */
#define LCD_CHANNEL 0xc

/* Register Select Field (LCD) */
#define DMA_LCD_CTRL        0
#define DMA_LCD_TOP_F1_L    1
#define DMA_LCD_TOP_F1_U    2
#define DMA_LCD_BOT_F1_L    3
#define DMA_LCD_BOT_F1_U    4

#define LCD_FRAME_MODE             (1<<0)
#define LCD_FRAME_IT_IE            (1<<1)
#define LCD_BUS_ERROR_IT_IE        (1<<2)
#define LCD_FRAME_1_IT_COND        (1<<3)
#define LCD_FRAME_2_IT_COND        (1<<4) 
#define LCD_BUS_ERROR_IT_COND      (1<<5)
#define LCD_SOURCE_IMIF            (1<<6)


/* ---------------------------------------------------------------------------
 *  Interrupt Handlers
 * ---------------------------------------------------------------------------
 * 
 */
#define OMAP710_IH1_BASE          0xfffecb00
#define OMAP710_IH2_BASE          0xfffe0000
#define OMAP710_ITR               0x0
#define OMAP710_MASK              0x4

#define INTERRUPT_HANDLER_BASE   OMAP710_IH1_BASE
#define INTERRUPT_INPUT_REGISTER OMAP710_ITR
#define INTERRUPT_MASK_REGISTER  OMAP710_MASK

#define IRQ_LEVEL_INT   1
#define IRQ_EDGE_INT 0

/* Um, it seems kinda of a waste to define a static table
 * for irq trigger level *_constants_*. Why not just use
 * a manifest constant with shift and mask based on the
 * irq number? Anyway, this is the start of such a method:
 *
 * IRQs 0 to 31:
 *	1111 0010 0111 1111 1111 1111 1100 1101
 *	1011 0011 1111 1111 1111 1110 0100 1111
 *	0xb3fffe4f
 *
 * IRQs 32 to 63:
 *	1000 0110 0011 1111 0000 0001 0111 1110
 *	0111 1110 1000 0000 1111 1100 0110 0001
 *	0x7e80fc61
 */

// OMAP710 INTERRUPT REGISTERS
#define IRQ_ITR			0x00
#define IRQ_MIR			0x04
#define IRQ_SIR_IRQ		0x10
#define IRQ_SIR_FIQ		0x14
#define IRQ_CONTROL_REG		0x18
#define IRQ_ISR			0x9c
#define IRQ_ILR0		0x1c

// INTERRUPT LEVEL REGISTER BITS
#define ILR_PRIORITY_MASK	(0x3c)
#define ILR_PRIORITY_SHIFT	(2)
#define ILR_LEVEL_TRIGGER	(1<<1)
#define ILR_FIQ			(1<<0)

/* ---------------------------------------------------------------------------
 *  OMAP710 FPGA
 * ---------------------------------------------------------------------------
 * 
 */

//#define FPGA_BASE      0xE8000000                 // VA
#define FPGA_REV_LOW           (FPGA_BASE + 0x0)   /* Revision */
#define FPGA_REV_HIGH          (FPGA_BASE + 0x1)   /* Revision */
#define FPGA_LCD_PANEL_CONTROL (FPGA_BASE + 0x2)
#define FPGA_LED_DIGIT         (FPGA_BASE + 0x3)
#define FPGA_ISR_LO            (FPGA_BASE + 6)   /* Interrupt Status Register (Low) */
#define FPGA_ISR_HI            (FPGA_BASE + 7)   /* Interrupt Status Register (High) */
#define FPGA_IMR_LO            (FPGA_BASE + 8)   /* Interrupt Mask Register (Low) */
#define FPGA_IMR_HI            (FPGA_BASE + 9)   /* Interrupt Mask Register (High) */
#define FPGA_CONTROL           (FPGA_BASE + 0xd)


/* ---------------------------------------------------------------------------
 *  OMAP710 TIMERS
 * ---------------------------------------------------------------------------
 * 
 */

#define OMAP710_32kHz_TIMER_BASE 0xfffb9000

#define OMAP710_TIMER1_BASE 0xfffec500
#define OMAP710_TIMER2_BASE 0xfffec600
#define OMAP710_TIMER3_BASE 0xfffec700

/* ---------------------------------------------------------------------------
 *  OMAP710 LED
 * ---------------------------------------------------------------------------
 * 
 */
#define OMAP710_LED_BASE 0xe8000003

/* ---------------------------------------------------------------------------
 *  OMAP710 GPIO (only)
 * ---------------------------------------------------------------------------
 * 
 */
#define GPIO_BASE1  OMAP710_GPIO_BASE
#define GPIO_BASE2  OMAP710_GPIO_BASE + 0x800
#define GPIO_BASE3  OMAP710_GPIO_BASE + 0x1000

#define GPIO1_DATA_INPUT_REG  (GPIO_BASE1 + 0x0)
#define GPIO1_DATA_OUTPUT_REG (GPIO_BASE1 + 0x4)
#define GPIO1_DIR_CONTROL_REG (GPIO_BASE1 + 0x8)
#define GPIO1_INT_CONTROL_REG (GPIO_BASE1 + 0xc)
#define GPIO1_INT_MASK_REG    (GPIO_BASE1 + 0x10)
#define GPIO1_INT_STATUS_REG  (GPIO_BASE1 + 0x14)

#define GPIO2_DATA_INPUT_REG  (GPIO_BASE2 + 0x0)
#define GPIO2_DATA_OUTPUT_REG (GPIO_BASE2 + 0x4)
#define GPIO2_DIR_CONTROL_REG (GPIO_BASE2 + 0x8)
#define GPIO2_INT_CONTROL_REG (GPIO_BASE2 + 0xc)
#define GPIO2_INT_MASK_REG    (GPIO_BASE2 + 0x10)
#define GPIO2_INT_STATUS_REG  (GPIO_BASE2 + 0x14)

#define GPIO3_DATA_INPUT_REG  (GPIO_BASE3 + 0x0)
#define GPIO3_DATA_OUTPUT_REG (GPIO_BASE3 + 0x4)
#define GPIO3_DIR_CONTROL_REG (GPIO_BASE3 + 0x8)
#define GPIO3_INT_CONTROL_REG (GPIO_BASE3 + 0xc)
#define GPIO3_INT_MASK_REG    (GPIO_BASE3 + 0x10)
#define GPIO3_INT_STATUS_REG  (GPIO_BASE3 + 0x14)

/* ---------------------------------------------------------------------------
 *  OMAP710 TIPB 
 * ---------------------------------------------------------------------------
 * 
 */
#define TIPB_PUBLIC_CNTL_BASE          0xfffed300
#define MPU_PUBLIC_TIPB_CNTL_REG       (TIPB_PUBLIC_CNTL_BASE + 0x8)
#define TIPB_PRIVATE_CNTL_BASE         0xfffeca00
#define MPU_PRIVATE_TIPB_CNTL_REG      (TIPB_PUBLIC_CNTL_BASE + 0x8)

/*****************************************************************************/
/* omap-specific section */

#define CLKGEN_RESET_BASE  (0xfffece00)
#define ARM_CKCTL          (volatile __u16 *)(CLKGEN_RESET_BASE + 0x0)
#define ARM_IDLECT1        (volatile __u16 *)(CLKGEN_RESET_BASE + 0x4)
#define ARM_IDLECT2        (volatile __u16 *)(CLKGEN_RESET_BASE + 0x8)
#define ARM_EWUPCT         (volatile __u16 *)(CLKGEN_RESET_BASE + 0xC)
#define ARM_RSTCT1         (volatile __u16 *)(CLKGEN_RESET_BASE + 0x10)
#define ARM_RSTCT2         (volatile __u16 *)(CLKGEN_RESET_BASE + 0x14)
#define ARM_SYSST          (volatile __u16 *)(CLKGEN_RESET_BASE + 0x18)

#define CK_CLKIN     12		/* MHz */
#define CK_RATEF     1
#define CK_IDLEF     2
#define CK_ENABLEF    4
#define CK_SELECTF   8
#define CK_DPLL1     ((volatile __u16 *)0xfffecf00)
#define SETARM_IDLE_SHIFT

/* ARM_CKCTL bit shifts */
#define PERDIV          0
#define LCDDIV          2
#define ARMDIV          4
#define DSPDIV          6
#define TCDIV           8
#define DSPMMUDIV       10
#define ARM_TIMXO       12
#define EN_DSPCK        13
#define ARM_INTHCK_SEL  14	/* REVISIT -- where is this used? */

/* ARM_IDLECT1 bit shifts */
#define IDLWDT_ARM      0
#define IDLXORP_ARM     1
#define IDLPER_ARM      2
#define IDLLCD_ARM      3
#define IDLLB_ARM       4
#define IDLHSAB_ARM     5
#define IDLIF_ARM       6
#define IDLDPLL_ARM     7
#define IDLAPI_ARM      8
#define IDLTIM_ARM      9
#define SETARM_IDLE     11

/* ARM_IDLECT2 bit shifts */
#define EN_WDTCK        0
#define EN_XORPCK       1
#define EN_PERCK        2
#define EN_LCDCK        3
#define EN_LBCK         4
#define EN_HSABCK       5
#define EN_APICK        6
#define EN_TIMCK        7
#define DMACK_REQ       8
#define EN_GPIOCK       9
#define EN_LBFREECK     10

#endif //__ASM_ARCH_OMAP_REGS_H
