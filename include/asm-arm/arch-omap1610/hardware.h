/*
 * linux/include/asm-arm/arch-omap/hardware.h
 *
 * OMAP1610 hardware map
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 *   <source@mvista.com>
 * Copyright (C) 2001 RidgeRun, Inc. (http://www.ridgerun.com)
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
#ifndef __ASM_ARCH_OMAP_HARDWARE_H
#define __ASM_ARCH_OMAP_HARDWARE_H
 
#include <asm/sizes.h>
#include <linux/config.h>
#ifndef __ASSEMBLER__
#include <asm/types.h>
#endif
#include <asm/mach-types.h>

/*
 * Where in virtual memory the IO devices (timers, system controllers
 * and so on)
 * ???_BASE is the virtual address
 * ???_START is the physical address
 */
#define IO_BASE		    0xFFFB0000                // VA of IO 
#define IO_SIZE		    0x40000                   // How much?
#define IO_START	    0xFFFB0000                // PA of IO

#define OMAP_DSP_BASE       0xE0000000                // VA
#define OMAP_DSP_SIZE       0x50000                   // size
#define OMAP_DSP_START      0xE0000000                // PA

#define OMAP_DSPREG_BASE    0xE1000000                // VA
#define OMAP_DSPREG_SIZE    SZ_128K                   // size
#define OMAP_DSPREG_START   0xE1000000                // PA

#define OMAP1610_SRAM_BASE  0xD0000000                // VA
#define OMAP1610_SRAM_SIZE  SZ_16K                    // size
#define OMAP1610_SRAM_START 0x20000000                // PA

/*
  There are 2 sets of general I/O -->
  1. GPIO (shared between ARM & DSP, configured by ARM)
  2. MPUIO which can be used only by the ARM.

  Base address FFFB:5000 is where the ARM accesses the MPUIO control registers
  (see 7.2.2 of the TRM for MPUIO reg definitions).

  Base address E101:5000 is reserved for ARM access of the same MPUIO control
  regs, but via the DSP I/O map.  This address is unavailable on 1610.

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
#define OMAP1610_GPIO_BASE       0xfffbe400
#define OMAP1610_GPIO_START      OMAP1610_GPIO_BASE
#define OMAP1610_GPIO_SIZE       SZ_1K

#define OMAP1610_GPIO2_BASE      0xfffbec00
#define OMAP1610_GPIO2_START     OMAP1610_GPIO2_BASE
#define OMAP1610_GPIO2_SIZE      SZ_1K
  
#define OMAP1610_MPUIO_BASE      0xfffb5000
#define OMAP1610_MPUIO_SIZE      SZ_2K
#define OMAP1610_MPUIO_START     0xfffb5000

#define OMAP1610_MCBSP1_BASE     0xE1011000               // VA
#define OMAP1610_MCBSP1_SIZE     SZ_4K                    // size
#define OMAP1610_MCBSP1_START    0xE1011000               // PA

#define OMAP1610_MCBSP2_BASE     0xFFFB1000

#define OMAP1610_MCBSP3_BASE     0xE1017000               // VA
#define OMAP1610_MCBSP3_SIZE     SZ_4K                    // size
#define OMAP1610_MCBSP3_START    0xE1017000               // PA

/*
 * Where's the flush address (for flushing D and I cache?)
 */
#define FLUSH_BASE		0xdf000000
#define FLUSH_BASE_PHYS	0x00000000

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

/* ----------------------------------------------------------------------------
 *  OMAP1610 system registers
 * ----------------------------------------------------------------------------
 */

#define OMAP1610_UART1_BASE        0xfffb0000    /* "BLUETOOTH-UART" */
#define OMAP1610_UART2_BASE        0xfffb0800    /* "MODEM-UART" */
#define OMAP1610_RTC_BASE          0xfffb4800    /* RTC */
#define OMAP1610_RTC_SIZE          128
#define OMAP1610_UART3_BASE        0xfffb9800    /* Shared MPU/DSP UART */
#define OMAP1610_COM_MCBSP2_BASE   0xffff1000    /* Com McBSP2 */
#define OMAP1610_AUDIO_MCBSP_BASE  0xffff1800    /* Audio McBSP2 */
#define OMAP1610_ARMIO_BASE        0xfffb5000    /* keyboard/gpio */

/* MPU I/O Registers:							*/
#define MPUIO_INPUT_LATCH_REG	(OMAP1610_ARMIO_BASE + 0x0)
#define MPUIO_OUTPUT_REG	(OMAP1610_ARMIO_BASE + 0x4)
#define MPUIO_IO_CNTL_REG	(OMAP1610_ARMIO_BASE + 0x8)
#define MPUIO_KBR_LATCH_REG	(OMAP1610_ARMIO_BASE + 0x10)
#define MPUIO_KBC_REG		(OMAP1610_ARMIO_BASE + 0x14)
#define MPUIO_EVENT_MODE_REG	(OMAP1610_ARMIO_BASE + 0x18)
#define MPUIO_INT_EDGE_REG	(OMAP1610_ARMIO_BASE + 0x1c)
#define MPUIO_KBD_INT_REG	(OMAP1610_ARMIO_BASE + 0x20)
#define MPUIO_INT_REG		(OMAP1610_ARMIO_BASE + 0x24)
#define MPUIO_KBD_MASKIT_REG	(OMAP1610_ARMIO_BASE + 0x28)
#define MPUIO_MASKIT_REG	(OMAP1610_ARMIO_BASE + 0x2c)
#define MPUIO_DEBOUNCING_REG	(OMAP1610_ARMIO_BASE + 0x30)
#define MPUIO_LATCH_REG		(OMAP1610_ARMIO_BASE + 0x38)

/* 
 * OMAP1610 UART3 Registers 
 */

#define OMAP_MPU_UART3_BASE  0xFFFB9800 /* UART3 through MPU bus */

/* UART3 Registers Maping through MPU bus */
 
#define UART3_RHR        (OMAP_MPU_UART3_BASE + 0)
#define UART3_THR        (OMAP_MPU_UART3_BASE + 0)
#define UART3_DLL        (OMAP_MPU_UART3_BASE + 0)
#define UART3_IER        (OMAP_MPU_UART3_BASE + 4)
#define UART3_DLH        (OMAP_MPU_UART3_BASE + 4)
#define UART3_IIR        (OMAP_MPU_UART3_BASE + 8)
#define UART3_FCR        (OMAP_MPU_UART3_BASE + 8)
#define UART3_EFR        (OMAP_MPU_UART3_BASE + 8)
#define UART3_LCR        (OMAP_MPU_UART3_BASE + 0x0C)  
#define UART3_MCR        (OMAP_MPU_UART3_BASE + 0x10)
#define UART3_XON1_ADDR1 (OMAP_MPU_UART3_BASE + 0x10)
#define UART3_XON2_ADDR2 (OMAP_MPU_UART3_BASE + 0x14)
#define UART3_LSR        (OMAP_MPU_UART3_BASE + 0x14)
#define UART3_TCR        (OMAP_MPU_UART3_BASE + 0x18) 
#define UART3_MSR        (OMAP_MPU_UART3_BASE + 0x18)
#define UART3_XOFF1      (OMAP_MPU_UART3_BASE + 0x18)
#define UART3_XOFF2      (OMAP_MPU_UART3_BASE + 0x1C)
#define UART3_SPR        (OMAP_MPU_UART3_BASE + 0x1C)  
#define UART3_TLR        (OMAP_MPU_UART3_BASE + 0x1C)
#define UART3_MDR1       (OMAP_MPU_UART3_BASE + 0x20)
#define UART3_MDR2       (OMAP_MPU_UART3_BASE + 0x24)
#define UART3_SFLSR      (OMAP_MPU_UART3_BASE + 0x28)
#define UART3_TXFLL      (OMAP_MPU_UART3_BASE + 0x28)
#define UART3_RESUME     (OMAP_MPU_UART3_BASE + 0x2C)
#define UART3_TXFLH      (OMAP_MPU_UART3_BASE + 0x2C)
#define UART3_SFREGL     (OMAP_MPU_UART3_BASE + 0x30)
#define UART3_RXFLL      (OMAP_MPU_UART3_BASE + 0x30)
#define UART3_SFREGH     (OMAP_MPU_UART3_BASE + 0x34)
#define UART3_RXFLH      (OMAP_MPU_UART3_BASE + 0x34)
#define UART3_BLR        (OMAP_MPU_UART3_BASE + 0x38)
#define UART3_ACREG      (OMAP_MPU_UART3_BASE + 0x3C)
#define UART3_DIV16      (OMAP_MPU_UART3_BASE + 0x3C)
#define UART3_SCR        (OMAP_MPU_UART3_BASE + 0x40)
#define UART3_SSR        (OMAP_MPU_UART3_BASE + 0x44)
#define UART3_EBLR       (OMAP_MPU_UART3_BASE + 0x48)
#define UART3_OSC_12M_SEL (OMAP_MPU_UART3_BASE + 0x4C)
#define UART3_MVR        (OMAP_MPU_UART3_BASE + 0x50)

/*
 * Configuration Registers
 */
#define FUNC_MUX_CTRL_0         0xfffe1000
#define FUNC_MUX_CTRL_1         0xfffe1004
#define FUNC_MUX_CTRL_2         0xfffe1008
#define COMP_MODE_CTRL_0        0xfffe100c
#define FUNC_MUX_CTRL_3         0xfffe1010
#define FUNC_MUX_CTRL_4         0xfffe1014
#define FUNC_MUX_CTRL_5         0xfffe1018
#define FUNC_MUX_CTRL_6         0xfffe101C
#define FUNC_MUX_CTRL_7         0xfffe1020
#define FUNC_MUX_CTRL_8         0xfffe1024
#define FUNC_MUX_CTRL_9         0xfffe1028
#define FUNC_MUX_CTRL_A         0xfffe102C
#define FUNC_MUX_CTRL_B         0xfffe1030
#define FUNC_MUX_CTRL_C         0xfffe1034
#define FUNC_MUX_CTRL_D         0xfffe1038

#define USB_TRANSCEIVER_CTRL    0xfffe1064

#define FUNC_MUX_CTRL_E		0xfffe1090
#define FUNC_MUX_CTRL_F		0xfffe1094
#define FUNC_MUX_CTRL_10	0xfffe1098
#define FUNC_MUX_CTRL_11	0xfffe109c
#define FUNC_MUX_CTRL_12	0xfffe10a0

#define PU_PD_SEL_0		0xfffe10b4
#define PU_PD_SEL_1		0xfffe10b8
#define PU_PD_SEL_2		0xfffe10bc
#define PU_PD_SEL_3		0xfffe10c0
#define PU_PD_SEL_4		0xfffe10c4

#define PULL_DWN_CTRL_0         0xfffe1040
#define PULL_DWN_CTRL_1         0xfffe1044
#define PULL_DWN_CTRL_2         0xfffe1048
#define PULL_DWN_CTRL_3         0xfffe104c
#define GATE_INH_CTRL_0         0xfffe1050
#define VOLTAGE_CTRL_0          0xfffe1060
#define TEST_DBG_CTRL_0         0xfffe1070

#define MOD_CONF_CTRL_0         0xfffe1080
#define MOD_CONF_CTRL_1         0xfffe1110

#define RESET_CONTROL           0xfffe1140

/* Some bit field definitions for a few OMAP1610 Configuration Registers: */
#define	I2C_MODE_CTRL		0
#define	CONF_MOD_I2C_SELECT_R	16

/*
 * Traffic Controller Memory Interface Registers
 */
#define TCMIF_BASE              0xfffecc00
#define IMIF_PRIO               (TCMIF_BASE + 0x00)
#define EMIFS_PRIO_REG          (TCMIF_BASE + 0x04)
#define EMIFF_PRIO_REG          (TCMIF_BASE + 0x08)
#define EMIFS_CONFIG_REG        (TCMIF_BASE + 0x0c)
#define EMIFS_CS0_CONFIG        (TCMIF_BASE + 0x10)
#define EMIFS_CS1_CONFIG        (TCMIF_BASE + 0x14)
#define EMIFS_CS2_CONFIG        (TCMIF_BASE + 0x18)
#define EMIFS_CS3_CONFIG        (TCMIF_BASE + 0x1c)
#define EMIFF_SDRAM_CONFIG      (TCMIF_BASE + 0x20)
#define EMIFF_MRS               (TCMIF_BASE + 0x24)
#define TC_TIMEOUT1             (TCMIF_BASE + 0x28)
#define TC_TIMEOUT2             (TCMIF_BASE + 0x2c)
#define TC_TIMEOUT3             (TCMIF_BASE + 0x30)
#define TC_ENDIANISM            (TCMIF_BASE + 0x34)
#define EMIFF_SDRAM_CONFIG_2    (TCMIF_BASE + 0x3c)
#define EMIF_CFG_DYNAMIC_WS     (TCMIF_BASE + 0x40)
#define EMIFS_CS0_CONFIG2        (TCMIF_BASE + 0x50)
#define EMIFS_CS1_CONFIG2        (TCMIF_BASE + 0x54)
#define EMIFS_CS2_CONFIG2        (TCMIF_BASE + 0x58)
#define EMIFS_CS3_CONFIG2        (TCMIF_BASE + 0x5c)


/*
 * LCD Panel
 */
#define TI925_LCD_BASE        0xFFFEC000        // VA of the LCD controller
#define TI925_LCD_CONTROL     (TI925_LCD_BASE)
#define TI925_LCD_TIMING0     (TI925_LCD_BASE+0x4)
#define TI925_LCD_TIMING1     (TI925_LCD_BASE+0x8)
#define TI925_LCD_TIMING2     (TI925_LCD_BASE+0xc)
#define TI925_LCD_STATUS      (TI925_LCD_BASE+0x10)
#define TI925_LCD_SUBPANEL    (TI925_LCD_BASE+0x14)
#define LCD_LINEINT           (TI925_LCD_BASE+0x18)
#define LCD_DISPLAYSTATUS     (TI925_LCD_BASE+0x1c)

#define OMAP_LCD_CONTROL      TI925_LCD_CONTROL

/* 
 * MMC/SD Host Controller Registers 
 */

#define OMAP_MMC_CMD     0xFFFB7800 /* MMC Command */
#define OMAP_MMC_ARGL    0xFFFB7804 /* MMC argument low */
#define OMAP_MMC_ARGH    0xFFFB7808 /* MMC argument high */
#define OMAP_MMC_CON     0xFFFB780C /* MMC system configuration */
#define OMAP_MMC_STAT    0xFFFB7810 /* MMC status */
#define OMAP_MMC_IE      0xFFFB7814 /* MMC system interrupt enable */
#define OMAP_MMC_CTO     0xFFFB7818 /* MMC command time-out */
#define OMAP_MMC_DTO     0xFFFB781C /* MMC data time-out */
#define OMAP_MMC_DATA    0xFFFB7820 /* MMC TX/RX FIFO data */
#define OMAP_MMC_BLEN    0xFFFB7824 /* MMC block length */
#define OMAP_MMC_NBLK    0xFFFB7828 /* MMC number of blocks */
#define OMAP_MMC_BUF     0xFFFB782C /* MMC buffer configuration */
#define OMAP_MMC_SPI     0xFFFB7830 /* MMC serial port interface */
#define OMAP_MMC_SDIO    0xFFFB7834 /* MMC SDIO mode configuration */
#define OMAP_MMC_SYST    0xFFFB7838 /* MMC system test */
#define OMAP_MMC_REV     0xFFFB783C /* MMC module version */
#define OMAP_MMC_RSP0    0xFFFB7840 /* MMC command response 0 */
#define OMAP_MMC_RSP1    0xFFFB7844 /* MMC command response 1 */
#define OMAP_MMC_RSP2    0xFFFB7848 /* MMC command response 2 */
#define OMAP_MMC_RSP3    0xFFFB784C /* MMC command response 3 */
#define OMAP_MMC_RSP4    0xFFFB7850 /* MMC command response 4 */
#define OMAP_MMC_RSP5    0xFFFB7854 /* MMC command response 5 */
#define OMAP_MMC_RSP6    0xFFFB7858 /* MMC command response 6 */
#define OMAP_MMC_RSP7    0xFFFB785C /* MMC command response 4 */

/* MMC masks */

#define OMAP_MMC_END_OF_CMD   (1 << 0)   /* End of command phase */
#define OMAP_MMC_CARD_BUSY    (1 << 2)   /* Card enter busy state */
#define OMAP_MMC_BLOCK_RS     (1 << 3)   /* Block received/sent */
#define OMAP_MMC_EOF_BUSY     (1 << 4)   /* Card exit busy state */
#define OMAP_MMC_DATA_TIMEOUT (1 << 5)   /* Data response time-out */
#define OMAP_MMC_DATA_CRC     (1 << 6)   /* Date CRC error */
#define OMAP_MMC_CMD_TIMEOUT  (1 << 7)   /* Command response time-out */
#define OMAP_MMC_CMD_CRC      (1 << 8)   /* Command CRC error */
#define OMAP_MMC_A_FULL       (1 << 10)  /* Buffer almost full */
#define OMAP_MMC_A_EMPTY      (1 << 11)  /* Buffer almost empty */
#define OMAP_MMC_OCR_BUSY     (1 << 12)  /* OCR busy */
#define OMAP_MMC_CARD_IRQ     (1 << 13)  /* Card IRQ received */
#define OMAP_MMC_CARD_ERR     (1 << 14)  /* Card status error in response */

/* 2.9.2 MPUI Interface Registers FFFE:C900 */

#define	MPUI_CTRL_REG		(volatile __u32 *)(0xfffec900)
#define	MPUI_DEBUG_ADDR		(volatile __u32 *)(0xfffec904)
#define	MPUI_DEBUG_DATA		(volatile __u32 *)(0xfffec908)
#define	MPUI_DEBUG_FLAG		(volatile __u16 *)(0xfffec90c)
#define	MPUI_STATUS_REG		(volatile __u16 *)(0xfffec910)
#define	MPUI_DSP_STATUS_REG	(volatile __u16 *)(0xfffec914)
#define	MPUI_DSP_BOOT_CONFIG	(volatile __u16 *)(0xfffec918)
#define	MPUI_DSP_API_CONFIG	(volatile __u16 *)(0xfffec91c)

/* 2.9.6 Traffic Controller Memory Interface Registers: */
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
#define SRAM_FRAMEBUFFER_MEMORY OMAP1610_SRAM_BASE

/* 
 * DMA
 */

#define OMAP1610_DMA_BASE 0xFFFED800
#define OMAP_DMA_BASE     OMAP1610_DMA_BASE
#define OMAP_DMA_GCR      OMAP1610_DMA_BASE+0x400
#define OMAP_DMA_GSCR     OMAP1610_DMA_BASE+0x404
#define OMAP_DMA_GRST     OMAP1610_DMA_BASE+0x408
#define OMAP_DMA_CROSS    0xFFFE10EC
/* Global Register selection */
#define NO_GLOBAL_DMA_ACCESS 0

/* Channel select field
 * NOTE: all other channels are linear, chan0 is 0, chan1 is 1, etc...
 */
#define LCD_CHANNEL 0x2F

/* Register Select Field (LCD) */
#define DMA_LCD_CSDP        0
#define DMA_LCD_CCR         1
#define DMA_LCD_CTRL        2
#define DMA_LCD_TOP_B1_L    4
#define DMA_LCD_TOP_B1_U    5
#define DMA_LCD_BOT_B1_L    6
#define DMA_LCD_BOT_B1_U    7
#define DMA_LCD_TOP_B2_L    8
#define DMA_LCD_TOP_B2_U    9
#define DMA_LCD_BOT_B2_L    10
#define DMA_LCD_BOT_B2_U    11
#define DMA_LCD_SRC_EI_B1   12
#define DMA_LCD_SRC_FI_B1_L 13
#define DMA_LCD_SRC_FI_B1_U 26
#define DMA_LCD_SRC_EI_B2   14
#define DMA_LCD_SRC_FI_B2_L 15
#define DMA_LCD_SRC_FI_B2_U 27
#define DMA_LCD_SRC_EN_B1   16
#define DMA_LCD_SRC_EN_B2   17
#define DMA_LCD_SRC_FN_B1   18
#define DMA_LCD_SRC_FN_B2   19
#define DMA_LCD_LCH_CTRL    21

#define LCD_FRAME_MODE             (1<<0)
#define LCD_FRAME_IT_IE            (1<<1)
#define LCD_BUS_ERROR_IT_IE        (1<<2)
#define LCD_FRAME_1_IT_COND        (1<<3)
#define LCD_FRAME_2_IT_COND        (1<<4) 
#define LCD_BUS_ERROR_IT_COND      (1<<5)
#define LCD_SOURCE_IMIF            (1<<6)

/*
 * Real-Time Clock
 */

#define OMAP_RTC_SECONDS_REG		(OMAP1610_RTC_BASE + 0x00)   
#define OMAP_RTC_MINUTES_REG		(OMAP1610_RTC_BASE + 0x04)   
#define OMAP_RTC_HOURS_REG		(OMAP1610_RTC_BASE + 0x08)   
#define OMAP_RTC_DAYS_REG		(OMAP1610_RTC_BASE + 0x0C)   
#define OMAP_RTC_MONTHS_REG		(OMAP1610_RTC_BASE + 0x10)   
#define OMAP_RTC_YEARS_REG		(OMAP1610_RTC_BASE + 0x14)   
#define OMAP_RTC_WEEKS_REG		(OMAP1610_RTC_BASE + 0x18)   
#define OMAP_RTC_ALARM_SECONDS_REG	(OMAP1610_RTC_BASE + 0x20)
#define OMAP_RTC_ALARM_MINUTES_REG	(OMAP1610_RTC_BASE + 0x24)
#define OMAP_RTC_ALARM_HOURS_REG	(OMAP1610_RTC_BASE + 0x28)
#define OMAP_RTC_ALARM_DAYS_REG		(OMAP1610_RTC_BASE + 0x2c)
#define OMAP_RTC_ALARM_MONTHS_REG	(OMAP1610_RTC_BASE + 0x30)
#define OMAP_RTC_ALARM_YEARS_REG	(OMAP1610_RTC_BASE + 0x34)
#define OMAP_RTC_CTRL_REG		(OMAP1610_RTC_BASE + 0x40)
#define OMAP_RTC_STATUS_REG		(OMAP1610_RTC_BASE + 0x44)
#define OMAP_RTC_INTERRUPTS_REG		(OMAP1610_RTC_BASE + 0x48)
#define OMAP_RTC_COMP_LSB_REG		(OMAP1610_RTC_BASE + 0x4c)
#define OMAP_RTC_COMP_MSB_REG		(OMAP1610_RTC_BASE + 0x50)

/* RTC Control Register bit fields: */

#define OMAP_RTC_CTRL_STOP		(1<<0)

/* RTC Status Register bit fields: */

#define OMAP_RTC_STATUS_POWER_UP	(1<<7)
#define OMAP_RTC_STATUS_ALARM		(1<<6)
#define OMAP_RTC_STATUS_1D_EVENT	(1<<5)
#define OMAP_RTC_STATUS_1H_EVENT	(1<<4)
#define OMAP_RTC_STATUS_1M_EVENT	(1<<3)
#define OMAP_RTC_STATUS_1S_EVENT	(1<<2)
#define OMAP_RTC_STATUS_RUN		(1<<1)
#define OMAP_RTC_STATUS_BUSY		(1<<0)

/* RTC Interrupt Register bit fields: */

#define OMAP_RTC_INTERRUPTS_IT_ALARM	(1<<3)
#define OMAP_RTC_INTERRUPTS_IT_TIMER	(1<<2)

/* uWire Registers: */

#define UWIRE_BASE    0xFFFB3000
#define UWIRE_IO_SIZE 0x20
#define UWIRE_TDR     UWIRE_BASE + 0x00
#define UWIRE_RDR     UWIRE_BASE + 0x00
#define UWIRE_CSR     UWIRE_BASE + 0x04
#define UWIRE_SR1     UWIRE_BASE + 0x08
#define UWIRE_SR2     UWIRE_BASE + 0x0C
#define UWIRE_SR3     UWIRE_BASE + 0x10
#define UWIRE_SR4     UWIRE_BASE + 0x14
#define UWIRE_SR5     UWIRE_BASE + 0x18

/* I2C Registers: */

#define I2C_BASE	(0xfffb3800UL)
#define I2C_IOSIZE	(0x40)
#define I2C_REV		(I2C_BASE + 0x00)
#define I2C_IE		(I2C_BASE + 0x04)
#define I2C_STAT	(I2C_BASE + 0x08)
#define I2C_SYSS	(I2C_BASE + 0x10)
#define I2C_BUF		(I2C_BASE + 0x14)
#define I2C_CNT		(I2C_BASE + 0x18)
#define I2C_DATA	(I2C_BASE + 0x1c)
#define I2C_SYSC	(I2C_BASE + 0x20)
#define I2C_CON		(I2C_BASE + 0x24)
#define I2C_OA		(I2C_BASE + 0x28)
#define I2C_SA		(I2C_BASE + 0x2c)
#define I2C_PSC		(I2C_BASE + 0x30)
#define I2C_SCLL	(I2C_BASE + 0x34)
#define I2C_SCLH	(I2C_BASE + 0x38)
#define I2C_SYSTEST	(I2C_BASE + 0x3c)

/* I2C Interrupt Enable Register (I2C_IE): */

#define I2C_IE_XRDY_IE	(1 << 4)	/* Transmit data ready interrupt enable */
#define I2C_IE_RRDY_IE	(1 << 3)	/* Receive data ready interrupt enable */
#define I2C_IE_ARDY_IE	(1 << 2)	/* Register access ready interrupt enable */
#define I2C_IE_NACK_IE	(1 << 1)	/* No acknowledgment interrupt enable */
#define I2C_IE_AL_IE	(1 << 0)	/* Arbitration lost interrupt enable */

/* I2C Status Register (I2C_STAT): */

#define I2C_STAT_SBD	(1 << 15)	/* Single byte data */
#define I2C_STAT_BB	(1 << 12)	/* Bus busy */
#define I2C_STAT_ROVR	(1 << 11)	/* Receive overrun */
#define I2C_STAT_XUDF	(1 << 10)	/* Transmit underflow */
#define I2C_STAT_AAS	(1 << 9)	/* Address as slave */
#define I2C_STAT_AD0	(1 << 8)	/* Address zero */
#define I2C_STAT_XRDY	(1 << 4)	/* Transmit data ready */
#define I2C_STAT_RRDY	(1 << 3)	/* Receive data ready */
#define I2C_STAT_ARDY	(1 << 2)	/* Register access ready */
#define I2C_STAT_NACK	(1 << 1)	/* No acknowledgment interrupt enable */
#define I2C_STAT_AL	(1 << 0)	/* Arbitration lost interrupt enable */

/* I2C Buffer Configuration Register (I2C_BUF): */

#define I2C_BUF_RDMA_EN		(1 << 15)	/* Receive DMA channel enable */
#define I2C_BUF_XDMA_EN		(1 << 7)	/* Transmit DMA channel enable */

/* I2C Configuration Register (I2C_CON): */

#define I2C_CON_EN	(1 << 15)	/* I2C module enable */
#define I2C_CON_BE	(1 << 14)	/* Big endian mode */
#define I2C_CON_STB	(1 << 11)	/* Start byte mode (master mode only) */
#define I2C_CON_MST	(1 << 10)	/* Master/slave mode */
#define I2C_CON_TRX	(1 << 9)	/* Transmitter/receiver mode (master mode only) */
#define I2C_CON_XA	(1 << 8)	/* Expand address */
#define I2C_CON_RM	(1 << 2)	/* Repeat mode (master mode only) */
#define I2C_CON_STP	(1 << 1)	/* Stop condition (master mode only) */
#define I2C_CON_STT	(1 << 0)	/* Start condition (master mode only) */

/* I2C System Test Register (I2C_SYSTEST): */

#define I2C_SYSTEST_ST_EN	(1 << 15)	/* System test enable */
#define I2C_SYSTEST_FREE	(1 << 14)	/* Free running mode (on breakpoint) */
#define I2C_SYSTEST_TMODE_MASK	(3 << 12)	/* Test mode select */
#define I2C_SYSTEST_TMODE_SHIFT	(12)		/* Test mode select */
#define I2C_SYSTEST_SCL_I	(1 << 3)	/* SCL line sense input value */
#define I2C_SYSTEST_SCL_O	(1 << 2)	/* SCL line drive output value */
#define I2C_SYSTEST_SDA_I	(1 << 1)	/* SDA line sense input value */
#define I2C_SYSTEST_SDA_O	(1 << 0)	/* SDA line drive output value */

/* I2C System Status register (I2C_SYSS): */

#define I2C_SYSS_RDONE		1		/* Reset Done*/

/* I2C System Configuration Register (I2C_SYSC): */

#define I2C_SYSC_SRST		(1 << 1)	/* Soft Reset */

/* ---------------------------------------------------------------------------
 *  OMAP1610 Interrupt Handlers
 * ---------------------------------------------------------------------------
 *
 */
#define OMAP_IH1_BASE            0xfffecb00
#define OMAP_IH2_0_BASE          0xfffe0000
#define OMAP_IH2_1_BASE          0xfffe0100
#define OMAP_IH2_2_BASE          0xfffe0200
#define OMAP_IH2_3_BASE          0xfffe0300
#define OMAP1610_ITR             0x0
#define OMAP1610_MASK            0x4
#define OMAP1610_SIR_IRQ         0x10
#define OMAP1610_CONTROL_IRQ     0x18

#define INTERRUPT_HANDLER_BASE   OMAP_IH1_BASE
#define INTERRUPT_INPUT_REGISTER OMAP1610_ITR
#define INTERRUPT_MASK_REGISTER  OMAP1610_MASK

/* ---------------------------------------------------------------------------
 *  OMAP1610 TIMERS
 * ---------------------------------------------------------------------------
 * 
 */

#define OMAP1610_32kHz_TIMER_BASE 0xfffb9000

// 32k Timer Registers
#define TIMER32k_CR     0x08
#define TIMER32k_TVR    0x00
#define TIMER32k_TCR    0x04

// 32k Timer Control Register definition
#define TIMER32k_TSS    (1<<0)
#define TIMER32k_TRB    (1<<1)
#define TIMER32k_INT    (1<<2)
#define TIMER32k_ARL    (1<<3)

// MPU Timer base addresses
#define OMAP1610_MPUTIMER_BASE 0xfffec500
#define OMAP1610_MPUTIMER_OFF  0x00000100

#define OMAP1610_TIMER1_BASE 0xfffec500
#define OMAP1610_TIMER2_BASE 0xfffec600
#define OMAP1610_TIMER3_BASE 0xfffec700
#define OMAP1610_WATCHDOG_BASE	0xfffec800
#define OMAP1610_WDT_TIMER_MODE	(OMAP1610_WATCHDOG_BASE + 0x8)

// MPU Timer Registers
#define CNTL_TIMER 0
#define LOAD_TIM   4
#define READ_TIM   8

//  CNTL_TIMER register bits
#define MPUTIM_FREE         (1<<6)
#define MPUTIM_CLOCK_ENABLE (1<<5)
#define MPUTIM_PTV_MASK     (0x7<<PTV_BIT)
#define MPUTIM_PTV_BIT      2
#define MPUTIM_AR           (1<<1)
#define MPUTIM_ST           (1<<0)

#define mputimer_base(n) \
    ((volatile mputimer_regs_t*)(OMAP1610_MPUTIMER_BASE + \
                                 (n)*OMAP1610_MPUTIMER_OFF))

/* ---------------------------------------------------------------------------
 *  OMAP1610 GPIO (only)
 * ---------------------------------------------------------------------------
 * 
 */
#define GPIO_REVISION_REG  (OMAP1610_GPIO_BASE + 0x0)
#define GPIO_SYSCONFIG_REG  (OMAP1610_GPIO_BASE + 0x10)
#define GPIO_SYSSTATUS_REG  (OMAP1610_GPIO_BASE + 0x14)
#define GPIO_IRQSTATUS1_REG  (OMAP1610_GPIO_BASE + 0x18)
#define GPIO_IRQENABLE1_REG  (OMAP1610_GPIO_BASE + 0x1c)
#define GPIO_IRQSTATUS2_REG  (OMAP1610_GPIO_BASE + 0x20)
#define GPIO_IRQENABLE2_REG  (OMAP1610_GPIO_BASE + 0x24)
#define GPIO_WAKEUPENABLE_REG  (OMAP1610_GPIO_BASE + 0x28)
#define GPIO_DATAIN_REG  (OMAP1610_GPIO_BASE + 0x2C)
#define GPIO_DATAOUT_REG (OMAP1610_GPIO_BASE + 0x30)
#define GPIO_DIRECTION_REG (OMAP1610_GPIO_BASE + 0x34)
#define GPIO_EDGE_CTRL1_REG (OMAP1610_GPIO_BASE + 0x38)
#define GPIO_EDGE_CTRL2_REG (OMAP1610_GPIO_BASE + 0x3c)
#define GPIO_CLEAR_IRQENABLE1_REG    (OMAP1610_GPIO_BASE + 0x9c)
#define GPIO_CLEAR_IRQENABLE2_REG    (OMAP1610_GPIO_BASE + 0xa4)
#define GPIO_CLEAR_WAKEUPENA_REG    (OMAP1610_GPIO_BASE + 0xa8)
#define GPIO_CLEAR_DATAOUT_REG    (OMAP1610_GPIO_BASE + 0xb0)
#define GPIO_SET_IRQENABLE1_REG    (OMAP1610_GPIO_BASE + 0xdc)
#define GPIO_SET_IRQENABLE2_REG    (OMAP1610_GPIO_BASE + 0xe4)
#define GPIO_SET_WAKEUPENA_REG    (OMAP1610_GPIO_BASE + 0xe8)
#define GPIO_SET_DATAOUT_REG    (OMAP1610_GPIO_BASE + 0xf0)

#define GPIO2_REVISION_REG  (OMAP1610_GPIO2_BASE + 0x0)
#define GPIO2_SYSCONFIG_REG  (OMAP1610_GPIO2_BASE + 0x10)
#define GPIO2_SYSSTATUS_REG  (OMAP1610_GPIO2_BASE + 0x14)
#define GPIO2_IRQSTATUS1_REG  (OMAP1610_GPIO2_BASE + 0x18)
#define GPIO2_IRQENABLE1_REG  (OMAP1610_GPIO2_BASE + 0x1c)
#define GPIO2_IRQSTATUS2_REG  (OMAP1610_GPIO2_BASE + 0x20)
#define GPIO2_IRQENABLE2_REG  (OMAP1610_GPIO2_BASE + 0x24)
#define GPIO2_WAKEUPENABLE_REG  (OMAP1610_GPIO2_BASE + 0x28)
#define GPIO2_DATAIN_REG  (OMAP1610_GPIO2_BASE + 0x2C)
#define GPIO2_DATAOUT_REG (OMAP1610_GPIO2_BASE + 0x30)
#define GPIO2_DIRECTION_REG (OMAP1610_GPIO2_BASE + 0x34)
#define GPIO2_EDGE_CTRL1_REG (OMAP1610_GPIO2_BASE + 0x38)
#define GPIO2_EDGE_CTRL2_REG (OMAP1610_GPIO2_BASE + 0x3c)
#define GPIO2_CLEAR_IRQENABLE1_REG    (OMAP1610_GPIO2_BASE + 0x9c)
#define GPIO2_CLEAR_IRQENABLE2_REG    (OMAP1610_GPIO2_BASE + 0xa4)
#define GPIO2_CLEAR_WAKEUPENA_REG    (OMAP1610_GPIO2_BASE + 0xa8)
#define GPIO2_CLEAR_DATAOUT_REG    (OMAP1610_GPIO2_BASE + 0xb0)
#define GPIO2_SET_IRQENABLE1_REG    (OMAP1610_GPIO2_BASE + 0xdc)
#define GPIO2_SET_IRQENABLE2_REG    (OMAP1610_GPIO2_BASE + 0xe4)
#define GPIO2_SET_WAKEUPENA_REG    (OMAP1610_GPIO2_BASE + 0xe8)
#define GPIO2_SET_DATAOUT_REG    (OMAP1610_GPIO2_BASE + 0xf0)

/* ---------------------------------------------------------------------------
 *  OMAP1610 TIPB (only)
 * ---------------------------------------------------------------------------
 * 
 */
#define TIPB_PUBLIC_CNTL_BASE          0xfffed300
#define MPU_PUBLIC_TIPB_CNTL_REG       (TIPB_PUBLIC_CNTL_BASE + 0x8)
#define TIPB_PRIVATE_CNTL_BASE         0xfffeca00
#define MPU_PRIVATE_TIPB_CNTL_REG      (TIPB_PRIVATE_CNTL_BASE + 0x8)

/*
 * ---------------------------------------------------------------------------
 *  OMAP1610 Camera Interface
 * ---------------------------------------------------------------------------
 */
#define CAMERA_BASE          (IO_BASE + 0x6800)
#define CAM_CTRLCLOCK_REG    (CAMERA_BASE + 0x00)
#define CAM_IT_STATUS_REG    (CAMERA_BASE + 0x04)
#define CAM_MODE_REG         (CAMERA_BASE + 0x08)
#define CAM_STATUS_REG       (CAMERA_BASE + 0x0C)
#define CAM_CAMDATA_REG      (CAMERA_BASE + 0x10)
#define CAM_GPIO_REG         (CAMERA_BASE + 0x14)
#define CAM_PEAK_CTR_REG     (CAMERA_BASE + 0x18)
#define CAMERA_IOSIZE        0x1C

#ifndef __ASSEMBLY__
typedef struct {
	__u32 ctrlclock;     // 00
	__u32 it_status;     // 04
	__u32 mode;          // 08
	__u32 status;        // 0C
	__u32 camdata;       // 10
	__u32 gpio;          // 14
	__u32 peak_counter;  // 18
} camera_regs_t;
#endif

/* CTRLCLOCK bit shifts */
#define FOSCMOD_BIT    0
#define FOSCMOD_MASK   (0x7 << FOSCMOD_BIT)
#define   FOSCMOD_12MHz    0x0
#define   FOSCMOD_6MHz     0x2
#define   FOSCMOD_9_6MHz   0x4
#define   FOSCMOD_24MHz    0x5
#define   FOSCMOD_8MHz     0x6
#define POLCLK         (1<<3)
#define CAMEXCLK_EN    (1<<4)
#define MCLK_EN        (1<<5)
#define DPLL_EN        (1<<6)
#define LCLK_EN        (1<<7)

/* IT_STATUS bit shifts */
#define V_UP           (1<<0)
#define V_DOWN         (1<<1)
#define H_UP           (1<<2)
#define H_DOWN         (1<<3)
#define FIFO_FULL      (1<<4)
#define DATA_XFER      (1<<5)

/* MODE bit shifts */
#define CAMOSC         (1<<0)
#define IMGSIZE_BIT    1
#define IMGSIZE_MASK   (0x3 << IMGSIZE_BIT)
#define   IMGSIZE_CIF      (0x0 << IMGSIZE_BIT)    /* 352x288 */
#define   IMGSIZE_QCIF     (0x1 << IMGSIZE_BIT)    /* 176x144 */
#define   IMGSIZE_VGA      (0x2 << IMGSIZE_BIT)    /* 640x480 */
#define   IMGSIZE_QVGA     (0x3 << IMGSIZE_BIT)    /* 320x240 */
#define ORDERCAMD      (1<<3)
#define EN_V_UP        (1<<4)
#define EN_V_DOWN      (1<<5)
#define EN_H_UP        (1<<6)
#define EN_H_DOWN      (1<<7)
#define EN_DMA         (1<<8)
#define THRESHOLD      (1<<9)
#define THRESHOLD_BIT  9
#define THRESHOLD_MASK (0x7f<<9)
#define EN_NIRQ        (1<<16)
#define EN_FIFO_FULL   (1<<17)
#define RAZ_FIFO       (1<<18)

/* STATUS bit shifts */
#define VSTATUS        (1<<0)
#define HSTATUS        (1<<1)

/* GPIO bit shifts */
#define CAM_RST        (1<<0)

/*
 * ---------------------------------------------------------------------------
 *  OTG (USB On The Go Interface)
 * ---------------------------------------------------------------------------
 */
#define OTG_BASE     0xfffb0400
#define OTG_IOSIZE   0x100
#define OTG_SYSCON_1 0xfffb0404
#define OTG_SYSCON_2 0xfffb0408
#define OTG_CTRL     0xfffb040c
#define OTG_IRQ_EN   0xfffb0410

/* ---------------------------------------------------------------------------
 *  Differentiating processor versions for those who care.
 * ---------------------------------------------------------------------------
 * 
 */
/*#define OMAP1610_ID_CODE_REG 0xfffed404*/

#ifndef __ASSEMBLY__
int cpu_type(void);
#endif

/*
 * EVM Implementation Specifics.
 * 
 * *** NOTE ***
 * Any definitions in these files should be prefixed by an identifier -
 * eg. OMAP1610P1_FLASH0_BASE .
 *
 */

/*****************************************************************************/

#define CLKGEN_RESET_BASE  (0xfffece00)
#define ARM_CKCTL          (volatile __u16 *)(CLKGEN_RESET_BASE + 0x0)    
#define ARM_IDLECT1        (volatile __u16 *)(CLKGEN_RESET_BASE + 0x4)    
#define ARM_IDLECT2        (volatile __u16 *)(CLKGEN_RESET_BASE + 0x8)    
#define ARM_EWUPCT         (volatile __u16 *)(CLKGEN_RESET_BASE + 0xC)    
#define ARM_RSTCT1         (volatile __u16 *)(CLKGEN_RESET_BASE + 0x10)    
#define ARM_RSTCT2         (volatile __u16 *)(CLKGEN_RESET_BASE + 0x14)    
#define ARM_SYSST          (volatile __u16 *)(CLKGEN_RESET_BASE + 0x18)
#define ARM_CKOUT1         (volatile __u16 *)(CLKGEN_RESET_BASE + 0x1C)
#define ARM_CKOUT2         (volatile __u16 *)(CLKGEN_RESET_BASE + 0x20)
#define ARM_IDLECT3        (volatile __u16 *)(CLKGEN_RESET_BASE + 0x24)
#define EN_OCPI_CK	(1 << 0)
#define IDLOCPI_ARM	(1 << 1)

#define DSP_IDLECT1     (OMAP_DSPREG_BASE+0x8004)
#define DSP_IDLECT2     (OMAP_DSPREG_BASE+0x8008)


#define OCPI_BASE	0xfffec320
#define OCPI_FAULT	(volatile __u16 *)(OCPI_BASE + 0x00)
#define OCPI_CMD_FAULT	(volatile __u16 *)(OCPI_BASE + 0x04)
#define OCPI_SINT0	(volatile __u16 *)(OCPI_BASE + 0x08)
#define OCPI_TABORT	(volatile __u16 *)(OCPI_BASE + 0x0c)
#define OCPI_SINT1	(volatile __u16 *)(OCPI_BASE + 0x10)
#define OCPI_PROT	(volatile __u16 *)(OCPI_BASE + 0x14)
#define OCPI_SEC	(volatile __u16 *)(OCPI_BASE + 0x18)

#define CK_DPLL1     ((volatile __u16 *)0xfffecf00)

/* ARM_CKCTL bit shifts */
#define PERDIV          0
#define LCDDIV          2
#define ARMDIV          4
#define DSPDIV          6
#define TCDIV           8
#define DSPMMUDIV       10
#define ARM_TIMXO       12
#define EN_DSPCK        13
#define ARM_INTHCK_SEL  14 /* REVISIT -- where is this used? */

#define ARM_CKCTL_RSRVD_BIT15		(1 << 15)
#define ARM_CKCTL_ARM_INTHCK_SEL	(1 << 14)
#define ARM_CKCTL_EN_DSPCK		(1 << 13)
#define ARM_CKCTL_ARM_TIMXO		(1 << 12)
#define ARM_CKCTL_DSPMMU_DIV1		(1 << 11)
#define ARM_CKCTL_DSPMMU_DIV2		(1 << 10)
#define ARM_CKCTL_TCDIV1		(1 << 9)
#define ARM_CKCTL_TCDIV2		(1 << 8)
#define ARM_CKCTL_DSPDIV1		(1 << 7)
#define ARM_CKCTL_DSPDIV0		(1 << 6)
#define ARM_CKCTL_ARMDIV1		(1 << 5)
#define ARM_CKCTL_ARMDIV0		(1 << 4)
#define ARM_CKCTL_LCDDIV1		(1 << 3)
#define ARM_CKCTL_LCDDIV0		(1 << 2)
#define ARM_CKCTL_PERDIV1		(1 << 1)
#define ARM_CKCTL_PERDIV0		(1 << 0)

/* ARM_IDLECT1 bit shifts */
#define IDLWDT_ARM      0
#define IDLXORP_ARM     1
#define IDLPER_ARM      2
/* IDLLCD_ARM (bit 3) doesn't exist on omap1610 */
/* IDLLB_ARM (bit 4) doesn't exist on omap1610 */
/* IDLHSAB_ARM (bit 5) doesn't exist on omap1610 */
#define IDLIF_ARM       6
#define IDLDPLL_ARM     7
#define IDLAPI_ARM      8
#define IDLTIM_ARM      9
#define WKUP_MODE      10
/* SETARM_IDLE (bit 11) doesn't exist on omap1610 */
#define IDL_CLKOUT_ARM 12

/* ARM_IDLECT2 bit shifts */
#define EN_WDTCK        0
#define EN_XORPCK       1
#define EN_PERCK        2
#define EN_LCDCK        3
/* EN_LBCK (bit 4) doesn't exist on omap1610 */
/* EN_HSABCK (bit 5) doesn't exist on omap1610 */
#define EN_APICK        6
#define EN_TIMCK        7
#define DMACK_REQ       8
/* EN_GPIOCK (bit 9) doesn't exist on omap1610 */
/* EN_LBFREECK (bit 10) doesn't exist on omap1610 */
#define EN_CKOUT_ARM   11

#define ARM_RSTCT1_SW_RST       (1 << 3)
#define ARM_RSTCT1_DSP_RST      (1 << 2)
#define ARM_RSTCT1_DSP_EN       (1 << 1)
#define ARM_RSTCT1_ARM_RST      (1 << 0)

/* ARM_RSTCT2 bit shifts */
#define EN_PER          0

#define ARM_SYSST_RSRVD_BIT15	(1 << 15)
#define ARM_SYSST_RSRVD_BIT14	(1 << 14)
#define ARM_SYSST_CLOCK_SELECT2	(1 << 13)
#define ARM_SYSST_CLOCK_SELECT1	(1 << 12)
#define ARM_SYSST_CLOCK_SELECT0	(1 << 11)
#define ARM_SYSST_RSRVD_BIT10	(1 << 10)
#define ARM_SYSST_RSRVD_BIT9	(1 << 9)
#define ARM_SYSST_RSRVD_BIT8	(1 << 8)
#define ARM_SYSST_RSRVD_BIT7	(1 << 7)
#define ARM_SYSST_IDLE_DSP	(1 << 6)
#define ARM_SYSST_POR		(1 << 5)
#define ARM_SYSST_EXT_RST	(1 << 4)
#define ARM_SYSST_ARM_MCRST	(1 << 3)
#define ARM_SYSST_ARM_WDRST	(1 << 2)
#define ARM_SYSST_GLOB_SWRST	(1 << 1)
#define ARM_SYSST_DSP_WDRST	(1 << 0)

/* ARM_CKOUT1 bit shifts */
#define TCLKOUT 4
#define DCLKOUT 2
#define ACLKOUT 0

/* Table 15-23. DPLL Control Registers: */

#define DPLL_CTL_REG		(volatile __u16 *)(0xfffecf00)
#define CK_DPLL1		((volatile __u16 *)0xfffecf00)

/* Table 15-24. Control Register (CTL_REG): */

#define DPLL_CTL_REG_IOB		(1 << 13)
#define DPLL_CTL_REG_PLL_MULT		Fld(5,0)

/*****************************************************************************/

// OMAP INTERRUPT REGISTERS
#define IRQ_ITR			0x00
#define IRQ_MIR			0x04
#define IRQ_SIR_IRQ		0x10
#define IRQ_SIR_FIQ		0x14
#define IRQ_CONTROL_REG		0x18
#define IRQ_ISR			0x9c
#define IRQ_ILR0		0x1c
#define OCP_CFG                 0xa4 

// INTERRUPT LEVEL REGISTER BITS
#define ILR_PRIORITY_MASK 	(0x3c)
#define ILR_PRIORITY_SHIFT	(2)
#define ILR_LEVEL_TRIGGER	(1<<1)
#define ILR_FIQ			(1<<0)

#define IRQ_LEVEL_INT		1
#define IRQ_EDGE_INT		0

// These definitions define an area of FLASH set aside
// for the use of MTD/JFFS2. This is the area of flash
// that a JFFS2 filesystem will reside which is mounted
// at boot with the "root=/dev/mtdblock/0 rw"
// command line option. The flash address used here must
// fall within the legal range defined by rrload for storing
// the filesystem component. This address will be sufficiently
// deep into the overall flash range to avoid the other
// components also stored in flash such as the bootloader,
// the bootloader params, and the kernel.
// The SW2 settings for the map below are:
// 1 off, 2 off, 3 on, 4 off.

// Intel flash_0, partitioned as expected by rrload.
#define OMAP_FLASH_0_BASE	0xD8000000	// VA
#define OMAP_FLASH_0_START	0x00000000	// PA
#define OMAP_FLASH_0_SIZE	SZ_32M

//Ethernet base address & interrupt
#define INT_ETHER               INT_GPIO_0
#define OMAP_ETHR_BASE		0xE8000000
#define ETHR_BASE		OMAP_ETHR_BASE
#define OMAP_ETHR_START		0x04000000
#define OMAP_ETHR_SIZE		SZ_32M

//ULPD
#define ULPD_BASE		0xFFFE0800
#define POWER_CTRL_REG  	(ULPD_BASE+0x50)
#define ULPD_CAM_CLK_CTRL	(ULPD_BASE+0x7C)
#define CLOCK_CTRL_REG		(ULPD_BASE+0x30)
#define SOFT_REQ_REG		(ULPD_BASE+0x34)
#define SOFT_DISABLE_REQ_REG	(ULPD_BASE+0x68)

#define FPGA_BASE               0xD0100000
#define FPGA_START              0x04000000
#define FPGA_SIZE               SZ_4K

#define PS2_DATA_REG            (FPGA_BASE+0x20)
#define PS2_CTRL_REG            (FPGA_BASE+0x22)

#endif /*  __ASM_ARCH_OMAP_HARDWARE_H */
