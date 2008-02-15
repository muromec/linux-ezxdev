/*
 *  linux/include/asm-arm/arch-omap730/hardware.h
 *
 *  BRIEF MODULE DESCRIPTION
 *    OMAP730 hardware map
 *
 *  This has been written to include as many of the OMAP730 register defs
 *  as is possible. I've migrated as much of the code that was using 
 *  anonymous constants as is feasible to try and work out what the code
 *  is doing, and reduce the chance of silly bugs in this area. 
 *
 *  I've tried to keep the names in common with the conventions in the TRM,
 *  but where a definition existed before I've tried to keep the old name
 *  for simplicity and clarity. Its a comprimise, but hey, isn't all the
 *  kernel code? :-/
 * 
 *  Copyright (C) 2004 MPC-Data Limited. (http://www.mpc-data.co.uk)
 *   Dave Peverley <dpeverley@mpc-data.co.uk>
 * 
 *  Inspired by the OMAP1610 hardware.h header :
 *   Copyright (C) 2004 MontaVista Software, Inc.
 *            <source@mvista.com>
 *    Copyright (C) 2001 RidgeRun, Inc. (http://www.ridgerun.com)
 *    Author: RidgeRun, Inc.
 *            Greg Lonnon (glonnon@ridgerun.com) or info@ridgerun.com
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

#ifndef SZ_2K
#define SZ_2K	0x00000800
#endif

/*
 * Where in virtual memory the IO devices (timers, system controllers
 * and so on)
 * ???_BASE is the virtual address
 * ???_START is the physical address
 */

/* EMIFS chip selects
 * The physical base addresses of CS0 and CS3 depend on which chip select is
 * used for booting.
 */
#define OMAP730_CS0_BOOT_CS0_START	0x00000000
#define OMAP730_CS0_BOOT_CS3_START	0x0C000000
#define OMAP730_CS0_SIZE		SZ_64M

#define OMAP730_CS1_START		0x04000000
#define OMAP730_CS1_SIZE		SZ_64M

#define OMAP730_CS2_START		0x08000000
#define OMAP730_CS2_SIZE		SZ_64M

#define OMAP730_CS3_BOOT_CS0_START	0x0C000000
#define OMAP730_CS3_BOOT_CS3_START	0x00000000
#define OMAP730_CS3_SIZE		SZ_64M

// Intel flash_0, partitioned as expected by rrload.
#define OMAP_FLASH_0_BASE          0xD8000000                 // VA
#define OMAP_FLASH_0_SIZE          SZ_32M

#define OMAP730_FPGA_BASE          0xE8000000                 // VA
#define OMAP730_FPGA_SIZE          SZ_4K                      // SIZE
#define OMAP730_FPGA_START         0x04000000                 // PA

#define IO_BASE                    0xFFFB0000                 // VA of IO 
#define IO_START                   0xFFFB0000                 // PA of IO
#define IO_SIZE                    0x40000                 // How much?
 
// SDRAM (CS4) phys base address & size
#define OMAP7xx_SDRAM_PHYS         0x10000000
#define OMAP7xx_SDRAM_SIZE         SZ_32M

#define OMAP730_SRAM_BASE          0xD0000000                 // VA
#define OMAP730_SRAM_START         0x20000000                 // PA
#define OMAP730_SRAM_SIZE          (SZ_128K + SZ_64K + SZ_8K) // size

#define OMAP730_UART1_BASE         0xFFFB0000                 // VA
#define OMAP730_UART1_START        OMAP730_UART1_BASE         // PA
#define OMAP730_UART1_SIZE         SZ_1K                      // size

#define OMAP730_USB_OTG_BASE       0xFFFB0400                 // VA
#define OMAP730_USB_OTG_START      OMAP730_USB_OTG_BASE       // PA
#define OMAP730_USB_OTG_SIZE       SZ_1K                      // size

/* Note, this is referred to interchangably as UART2 or UART3 in the TRM
 * So we choose to refer to it as UART3 to retain compatibility with the
 * OMAP1610.
 */
#define OMAP730_UART3_BASE         0xFFFB0800                 // VA
#define OMAP730_UART3_START        OMAP730_UART3_BASE         // PA
#define OMAP730_UART3_SIZE         SZ_2K                      // size

#define OMAP730_MCBSP1_BASE        0xFFFB1000                 // VA
#define OMAP730_MCBSP1_START       OMAP730_MCBSP1_BASE        // PA
#define OMAP730_MCBSP1_SIZE        SZ_2K                      // size

#define OMAP730_MCBSP2_BASE        0xFFFB1800                 // VA
#define OMAP730_MCBSP2_START       OMAP730_MCBSP2_BASE        // PA
#define OMAP730_MCBSP2_SIZE        SZ_2K                      // size

#define OMAP730_UWIRE_BASE         0xFFFB3000                 // VA
#define OMAP730_UWIRE_START        OMAP730_UWIRE_BASE         // PA
#define OMAP730_UWIRE_SIZE         SZ_2K                      // size

#define OMAP730_I2C_BASE           0xFFFB3800                 // VA
#define OMAP730_I2C_START          OMAP730_I2C_BASE           // PA
#define OMAP730_I2C_SIZE           SZ_2K                      // size

#define OMAP730_RTC_BASE           0xFFFB4800                 // VA
#define OMAP730_RTC_START          OMAP730_RTC_BASE           // PA
#define OMAP730_RTC_SIZE           SZ_2K                      // size

#define OMAP730_ARMIO_BASE         0xFFFB5000                 // VA
#define OMAP730_ARMIO_START        OMAP730_ARMIO_BASE         // PA
#define OMAP730_ARMIO_SIZE         SZ_2K                      // size

#define OMAP730_CAMERA_IF_BASE     0xFFFB6800                 // VA
#define OMAP730_CAMERA_IF_START    OMAP730_CAMERA_IF_BASE     // PA
#define OMAP730_CAMERA_IF_SIZE     SZ_2K                      // size

#define OMAP730_MMC_BASE           0xFFFB7800                 // VA
#define OMAP730_MMC_START          OMAP730_MMC_BASE           // PA
#define OMAP730_MMC_SIZE           SZ_2K                      // size

#define OMAP730_32kHz_TIMER_BASE   0xFFFB9000                 // VA
#define OMAP730_32kHz_TIMER_START  OMAP730_32kHz_TIMER_BASE   // PA
#define OMAP730_32kHz_TIMER_SIZE   SZ_2K                      // size

#define OMAP730_ICR_BASE           0xFFFBB800
#define OMAP730_ICR_START          OMAP730_ICR_BASE
#define OMAP730_ICR_SIZE           SZ_2K

/* 
 * Note that MPU GPIO is a special case : 
 * There are a further 5 GPIO banks after this one, each at 0x800 (2K) 
 * offsets which contain duplicates of the registers, but for the next 
 * set of MPU GPIO lines. (This is because there are 6 * 32 GPIOs!)
 */

#define OMAP730_GPIO_BASE          0xfffbc000
#define OMAP730_GPIO_START         OMAP730_GPIO_BASE
#define OMAP730_GPIO_SIZE          SZ_2K
#define OMAP730_GPIO_NB            32
#define OMAP730_GPIO_BANKS         6

#define OMAP730_IH2_BASE           0xFFFE0000                 // VA
#define OMAP730_IH2_START          OMAP730_IH2_BASE           // PA
#define OMAP730_IH2_SIZE           256                        // size

#define OMAP730_PCC_ULPD_BASE      0xFFFE0800                 // VA
#define OMAP730_PCC_ULPD_START     OMAP730_PCC_ULPD_BASE      // PA
#define OMAP730_PCC_ULPD_SIZE      SZ_2K                      // size

#define OMAP730_CONFIG_BASE        0xFFFE1000                 // VA
#define OMAP730_CONFIG_START       OMAP730_CONFIG_BASE        // PA
#define OMAP730_CONFIG_SIZE        SZ_2K                      // size

#define OMAP730_LCD_BASE           0xFFFEC000                 // VA
#define OMAP730_LCD_START          OMAP730_LCD_BASE           // PA
#define OMAP730_LCD_SIZE           256                        // size

#define OMAP730_OCPI_BASE          0xFFFEC320                 // VA
#define OMAP730_OCPI_START         OMAP730_OCPI_BASE          // PA
#define OMAP730_OCPI_SIZE          224                        // size

/* 
 * Again, OMAP730_TIMER_BASE is special - It is three identical 
 * banks of registers for three timers, 0x100 bytes apart.
 */
#define OMAP730_TIMER_BASE         0xFFFEC500                 // VA
#define OMAP730_TIMER_START        OMAP730_TIMER_BASE         // PA
#define OMAP730_TIMER_SIZE         256                        // size

#define OMAP730_WATCHDOG_BASE      0xFFFEC800                 // VA
#define OMAP730_WATCHDOG_START     OMAP730_WATCHDOG_BASE      // PA
#define OMAP730_WATCHDOG_SIZE      256                        // size

#define OMAP730_IH1_BASE           0xFFFECB00                 // VA
#define OMAP730_IH1_START          OMAP730_IH1_BASE           // PA
#define OMAP730_IH1_SIZE           256                        // size

#define OMAP730_TCMIF_BASE         0xFFFECC00                 // VA
#define OMAP730_TCMIF_START        OMAP730_TCMIF_BASE         // PA
#define OMAP730_TCMIF_SIZE         256                        // size

#define OMAP730_CLKM_BASE          0xFFFECE00                 // VA
#define OMAP730_CLKM_START         OMAP730_CLKM_BASE          // PA
#define OMAP730_CLKM_SIZE          256                        // size

#define OMAP730_DPLL1_BASE         0xFFFECF00                 // VA
#define OMAP730_DPLL1_START        OMAP730_DPLL1_BASE         // PA
#define OMAP730_DPLL1_SIZE         256                        // size

#define OMAP730_DMA_BASE           0xFFFED800                 // VA
#define OMAP730_DMA_START          OMAP730_DMA_BASE           // PA
#define OMAP730_DMA_SIZE           SZ_4K                      // size
 
#define OMAP730_EAC_BASE           0xFFFBB000                 // VA
#define OMAP730_EAC_START          OMAP730_EAC_BASE           // PA
#define OMAP730_EAC_SIZE           SZ_4K                      // size

/* Kernel physical addresses for mapping GSM memory
 * mapped in reserved area
 * GSM CS0 program area (in CS0 FLASH)
 */
#define GTI_GSM_CODE_ADDR_PHY_SRC       0x01E00000              // Where the code resides in Flash
#define GTI_GSM_CODE_ADDR_PHY_EXEC      OMAP7xx_GSM_SDRAM_PHYS  // Where it is executed from
#define GTI_GSM_CODE_SIZE               0x200000
// GSM CS1 data area (in CS4 SDRAM)
#define GTI_GSM_DATA_ADDR_PHY           (OMAP7xx_GSM_SDRAM_PHYS + GTI_GSM_CODE_SIZE)
#define GTI_GSM_DATA_SIZE               0x100000
// GSM (Code & Data) usage in SDRAM: reserved at boot time (cf. arch/arm/mm/init.c)
#define OMAP7xx_GSM_SDRAM_PHYS  (OMAP7xx_SDRAM_PHYS + OMAP7xx_SDRAM_SIZE - OMAP7xx_GSM_SDRAM_SIZE)
#define OMAP7xx_GSM_SDRAM_SIZE  (GTI_GSM_CODE_SIZE + GTI_GSM_DATA_SIZE)


/*
 * Where's the flush address (for flushing D and I cache?)
 */
#define FLUSH_BASE                 0xdf000000
#define FLUSH_BASE_PHYS            0x00000000

#ifndef __ASSEMBLER__

#define PCIO_BASE                  0

/*
 * RAM definitions
 */
#define MAPTOPHYS(a)               ((unsigned long)(a) - PAGE_OFFSET)
#define KERNTOPHYS(a)              ((unsigned long)(&a))
#define KERNEL_BASE                (0x10008000)  

#endif

/* macro to get at IO space when running virtually */
#define IO_ADDRESS(x)              ((x)) 

/* Macros for Register access from base addresses */
#define REG8(addr)                 ((volatile __u8 *)(addr))
#define REG16(addr)                ((volatile __u16 *)(addr))
#define REG32(addr)                ((volatile __u32 *)(addr))


/* ---------------------------------------------------------------------------
 * OMAP730 UART3 Register Definitions
 * ---------------------------------------------------------------------------
 */

#define UART3_RHR                  (OMAP730_UART3_BASE + 0x00)
#define UART3_THR                  (OMAP730_UART3_BASE + 0x00)
#define UART3_DLL                  (OMAP730_UART3_BASE + 0x00)
#define UART3_IER                  (OMAP730_UART3_BASE + 0x01)
#define UART3_DLH                  (OMAP730_UART3_BASE + 0x01)
#define UART3_IIR                  (OMAP730_UART3_BASE + 0x02)
#define UART3_FCR                  (OMAP730_UART3_BASE + 0x02)
#define UART3_EFR                  (OMAP730_UART3_BASE + 0x02)
#define UART3_LCR                  (OMAP730_UART3_BASE + 0x03)
#define UART3_MCR                  (OMAP730_UART3_BASE + 0x04)
#define UART3_XON1_ADDR1           (OMAP730_UART3_BASE + 0x04)
#define UART3_XON2_ADDR2           (OMAP730_UART3_BASE + 0x05)
#define UART3_LSR                  (OMAP730_UART3_BASE + 0x05)
#define UART3_TCR                  (OMAP730_UART3_BASE + 0x06)
#define UART3_MSR                  (OMAP730_UART3_BASE + 0x06)
#define UART3_XOFF1                (OMAP730_UART3_BASE + 0x06)
#define UART3_XOFF2                (OMAP730_UART3_BASE + 0x07)
#define UART3_SPR                  (OMAP730_UART3_BASE + 0x07)
#define UART3_TLR                  (OMAP730_UART3_BASE + 0x07)
#define UART3_MDR1                 (OMAP730_UART3_BASE + 0x08)
#define UART3_MDR2                 (OMAP730_UART3_BASE + 0x09)
#define UART3_SFLSR                (OMAP730_UART3_BASE + 0x0A)
#define UART3_TXFLL                (OMAP730_UART3_BASE + 0x0A)
#define UART3_RESUME               (OMAP730_UART3_BASE + 0x0B)
#define UART3_TXFLH                (OMAP730_UART3_BASE + 0x0B)
#define UART3_SFREGL               (OMAP730_UART3_BASE + 0x0C)
#define UART3_RXFLL                (OMAP730_UART3_BASE + 0x0C)
#define UART3_SFREGH               (OMAP730_UART3_BASE + 0x0D)
#define UART3_RXFLH                (OMAP730_UART3_BASE + 0x0D)
#define UART3_BLR                  (OMAP730_UART3_BASE + 0x0E)
#define UART3_ACREG                (OMAP730_UART3_BASE + 0x0F)
#define UART3_DIV16                (OMAP730_UART3_BASE + 0x0F)
#define UART3_SCR                  (OMAP730_UART3_BASE + 0x10)
#define UART3_SSR                  (OMAP730_UART3_BASE + 0x11)
#define UART3_EBLR                 (OMAP730_UART3_BASE + 0x12)
#define UART3_OSC_12M_SEL          (OMAP730_UART3_BASE + 0x13)
#define UART3_MVR                  (OMAP730_UART3_BASE + 0x14)


/* Register definitions for UART3_IER */

#define UART3_IER_TX_STATUS_IT          (1 << 5)
#define UART3_IER_EOF_IT                (1 << 7)

/* Register definitions for UART3_IIR */

#define UART3_IIR_TX_STATUS_IT          (1 << 5)
#define UART3_IIR_EOF_IT                (1 << 7)

/* Register definitions for UART3_FCR */

#define UART3_FCR_FIFO_EN               (1 << 0)
#define UART3_FCR_RX_FIFO_CLEAR         (1 << 1)
#define UART3_FCR_TX_FIFO_CLEAR         (1 << 2)
#define UART3_FCR_DMA_MODE              (1 << 3)
#define UART3_FCR_TX_FIFO_TRIG_POS      4
#define UART3_FCR_TX_FIFO_TRIG_MSK      0x3
#define UART3_FCR_RX_FIFO_TRIG_POS      6
#define UART3_FCR_RX_FIFO_TRIG_MSK      0x3
#define   UART3_FCR_FIFO_SPACES_8       0x0
#define   UART3_FCR_FIFO_SPACES_16      0x1
#define   UART3_FCR_FIFO_SPACES_56      0x2
#define   UART3_FCR_FIFO_SPACES_60      0x3




/* Register definitions for UART3_LCR */

#define UART3_LCR_CHAR_LENGTH_POS       0
#define UART3_LCR_CHAR_LENGTH_MSK       0x3
#define UART3_LCR_CHAR_LENGTH_5_BITS    0x0
#define UART3_LCR_CHAR_LENGTH_6_BITS    0x1
#define UART3_LCR_CHAR_LENGTH_7_BITS    0x2
#define UART3_LCR_CHAR_LENGTH_8_BITS    0x3
#define UART3_LCR_DIV_EN                (1 << 7)








/* Register definitions for UART3_MDR1 */

#define UART3_MDR1_MODE_SELECT_POS      0
#define UART3_MDR1_MODE_SELECT_MSK      0x7
#define MDR1_MODE_SELECT_UART_16X       0x0
#define MDR1_MODE_SELECT_SIR            0x1
#define MDR1_MODE_SELECT_UART_16X_AUTO  0x2
#define MDR1_MODE_SELECT_UART_13X_AUTO  0x3 
#define MDR1_MODE_SELECT_MIR            0x4
#define MDR1_MODE_SELECT_FIR            0x5
#define MDR1_MODE_SELECT_DISABLE        0x7
#define UART3_MDR1_SIP_MODE             (1 << 6)

/* Register definitions for UART3_MDR2 */

#define UART3_MDR2_IRTX_UNDERRUN        (1 << 0)



/* Register definitions for UART3_ACREG */

#define UART3_ACREG_DIS_TX_UNDERRUN     (1 << 4)
#define UART3_ACREG_DIS_IR_RX           (1 << 5)
#define UART3_ACREG_SD_MOD              (1 << 6)
#define UART3_ACREG_PULSE_TYPE          (1 << 7)

/* Register definitions for UART3_SCR */

#define UART3_SCR_TX_TRIG_GRANU1        (1 << 6)
#define UART3_SCR_RX_TRIG_GRANU1        (1 << 7)




/* ---------------------------------------------------------------------------
 * OMAP730 MPU I/O Register Definitions
 * Note : In the TRM, this can be referred to as either MPUIO or ARMIO
 *        depending on which part you're reading.
 * ---------------------------------------------------------------------------
 */

/* MPU IO Register offsets */

#define MPUIO_INPUT_LATCH          (OMAP730_ARMIO_BASE + 0x00)
#define MPUIO_OUTPUT_REG           (OMAP730_ARMIO_BASE + 0x02)
#define MPUIO_IO_CNTL              (OMAP730_ARMIO_BASE + 0x04)
#define MPUIO_KBR_LATCH            (OMAP730_ARMIO_BASE + 0x08)
#define MPUIO_KBC_REG              (OMAP730_ARMIO_BASE + 0x0a)
#define MPUIO_GPIO_EVENT_MODE_REG  (OMAP730_ARMIO_BASE + 0x0c)
#define MPUIO_GPIO_INT_EDGE_REG    (OMAP730_ARMIO_BASE + 0x0e)
#define MPUIO_KBD_INT              (OMAP730_ARMIO_BASE + 0x10)
#define MPUIO_GPIO_INT             (OMAP730_ARMIO_BASE + 0x12)
#define MPUIO_KBD_MASKIT           (OMAP730_ARMIO_BASE + 0x14)
#define MPUIO_GPIO_MASKIT          (OMAP730_ARMIO_BASE + 0x16)
#define MPUIO_GPIO_DEBOUNCING_REG  (OMAP730_ARMIO_BASE + 0x18)
#define MPUIO_GPIO_LATCH_REG       (OMAP730_ARMIO_BASE + 0x1a)


/* ---------------------------------------------------------------------------
 * OMAP730 Configuration Register Definitions
 * ---------------------------------------------------------------------------
 */

/* Config Register Offsets */

#define PERSEUS2_MPU_DEV_ID        (OMAP730_CONFIG_BASE + 0x00)
#define PERSEUS2_GSM_DEV_ID0       (OMAP730_CONFIG_BASE + 0x00)
#define PERSEUS2_GSM_DEV_ID1       (OMAP730_CONFIG_BASE + 0x02)
#define DSP_CONF                   (OMAP730_CONFIG_BASE + 0x04)
#define PERSEUS2_MPU_DIE_ID0       (OMAP730_CONFIG_BASE + 0x08)
#define GSM_ASIC_CONF              (OMAP730_CONFIG_BASE + 0x08)
#define PERSEUS2_MODE_1            (OMAP730_CONFIG_BASE + 0x10)
#define PERSEUS2_GSM_DIE_ID0       (OMAP730_CONFIG_BASE + 0x10)
#define PERSEUS2_GSM_DIE_ID1       (OMAP730_CONFIG_BASE + 0x12)
#define PERSEUS2_MODE_2            (OMAP730_CONFIG_BASE + 0x14)
#define PERSEUS2_GSM_DIE_ID2       (OMAP730_CONFIG_BASE + 0x14)
#define PERSEUS2_GSM_DIE_ID3       (OMAP730_CONFIG_BASE + 0x16)
#define PERSEUS2_ANALOG_CELLS_CONF (OMAP730_CONFIG_BASE + 0x18)
#define SECCTRL                    (OMAP730_CONFIG_BASE + 0x1C)
#define ECO_SPARE1                 (OMAP730_CONFIG_BASE + 0x20)
#define ECO_SPARE2                 (OMAP730_CONFIG_BASE + 0x24)
#define GSM_PBG_IRQ                (OMAP730_CONFIG_BASE + 0x28)
#define DMA_REQ_CONF               (OMAP730_CONFIG_BASE + 0x30)
#define PE_CONF_NO_DUAL            (OMAP730_CONFIG_BASE + 0x60)
#define PERSEUS2_IO_CONF0          (OMAP730_CONFIG_BASE + 0x70)
#define PERSEUS2_IO_CONF1          (OMAP730_CONFIG_BASE + 0x74)
#define PERSEUS2_IO_CONF2          (OMAP730_CONFIG_BASE + 0x78)
#define PERSEUS2_IO_CONF3          (OMAP730_CONFIG_BASE + 0x7c)
#define PERSEUS2_IO_CONF4          (OMAP730_CONFIG_BASE + 0x80)
#define PERSEUS2_IO_CONF5          (OMAP730_CONFIG_BASE + 0x84)
#define PERSEUS2_IO_CONF6          (OMAP730_CONFIG_BASE + 0x88)
#define PERSEUS2_IO_CONF7          (OMAP730_CONFIG_BASE + 0x8c)
#define PERSEUS2_IO_CONF8          (OMAP730_CONFIG_BASE + 0x90)
#define PERSEUS2_IO_CONF9          (OMAP730_CONFIG_BASE + 0x94)
#define PERSEUS2_IO_CONF10         (OMAP730_CONFIG_BASE + 0x98)
#define PERSEUS2_IO_CONF11         (OMAP730_CONFIG_BASE + 0x9c)
#define PERSEUS2_IO_CONF12         (OMAP730_CONFIG_BASE + 0xa0)
#define PERSEUS2_IO_CONF13         (OMAP730_CONFIG_BASE + 0xa4)
#define PERSEUS_PCC_CONF_REG       (OMAP730_CONFIG_BASE + 0xB4)
#define BIST_STATUS_INTERNAL       (OMAP730_CONFIG_BASE + 0xB8)
#define BIST_CONTROL               (OMAP730_CONFIG_BASE + 0xC0)
#define BIST_SECROM_SIG1_INTERNAL  (OMAP730_CONFIG_BASE + 0xD0)
#define BIST_SECROM_SIG2_INTERNAL  (OMAP730_CONFIG_BASE + 0xD4)
#define DEBUG1                     (OMAP730_CONFIG_BASE + 0xE0)
#define DEBUG2                     (OMAP730_CONFIG_BASE + 0xE4)
#define DEBUG_DMA_IRQ              (OMAP730_CONFIG_BASE + 0xE8)


/* ECO_SPARE1 register defines */
	
#define ECO_SPARE1_INITVALUE       0x2   /* Disable pull-ups of SDMC in mode 0 */

/* PERSEUS2_PCC_CONF_REG bit positions */

#define PCONF_MMC_DPLL_REQ         (1 << 2)
#define UART1_DPLL_REQ             (1 << 3)
#define UART3_DPLL_REQ             (1 << 4)
#define PCC_CAM_CLK_REQ            (1 << 7)

/*
 *  For bit fields within the PERSEUS2_IO_CONF* registers, see omap730_mux.h
 */




/* ---------------------------------------------------------------------------
 * OMAP730 TCMIF Register definitions
 * ---------------------------------------------------------------------------
 * 
 */

/* TCMIF / EMIFS Register offsets */

#define EMIFS_LRUREG               (OMAP730_TCMIF_BASE + 0x04)
#define EMIFS_CONFIG               (OMAP730_TCMIF_BASE + 0x0C)
#define OMAP_EMIFS_CONFIG_REG      EMIFS_CONFIG
#define FLASH_CFG_0                (OMAP730_TCMIF_BASE + 0x10)
#define FLASH_CFG_1                (OMAP730_TCMIF_BASE + 0x14)
#define FLASH_CFG_2                (OMAP730_TCMIF_BASE + 0x18)
#define FLASH_CFG_3                (OMAP730_TCMIF_BASE + 0x1C)
#define FL_CFG_DYN_WAIT            (OMAP730_TCMIF_BASE + 0x40)
#define EMIFS_TIMEOUT1_REG         (OMAP730_TCMIF_BASE + 0x28)
#define EMIFS_TIMEOUT2_REG         (OMAP730_TCMIF_BASE + 0x2C)
#define EMIFS_TIMEOUT3_REG         (OMAP730_TCMIF_BASE + 0x30)
#define EMIFS_ABORT_ADDR           (OMAP730_TCMIF_BASE + 0x44)
#define EMIFS_ABORT_TYPE           (OMAP730_TCMIF_BASE + 0x48)
#define EMIFS_ABORT_TOUT           (OMAP730_TCMIF_BASE + 0x4C)
#define FLASH_ACFG_0_1             (OMAP730_TCMIF_BASE + 0x50)
#define FLASH_ACFG_1_1             (OMAP730_TCMIF_BASE + 0x54)
#define FLASH_ACFG_2_1             (OMAP730_TCMIF_BASE + 0x58)
#define FLASH_ACFG_3_1             (OMAP730_TCMIF_BASE + 0x5C)

/* TCMIF / EMIFF Register offsets */

#define EMIFF_PRIORITY_REG         (OMAP730_TCMIF_BASE + 0x08)
#define EMIFF_SDRAM_CONFIG         (OMAP730_TCMIF_BASE + 0x20)
#define EMIFF_MRS                  (OMAP730_TCMIF_BASE + 0x24)
#define EMIFF_TIMEOUT1             (OMAP730_TCMIF_BASE + 0x8C)
#define EMIFF_TIMEOUT2             (OMAP730_TCMIF_BASE + 0x90)
#define EMIFF_TIMEOUT3             (OMAP730_TCMIF_BASE + 0x94)
#define EMIFF_CONFIG_REG32         (OMAP730_TCMIF_BASE + 0x3C)
#define EMIFF_MRS_NEW              (OMAP730_TCMIF_BASE + 0x70)
#define EMIFF_EMRS1                (OMAP730_TCMIF_BASE + 0x78)
#define EMIFF_EMRS2                (OMAP730_TCMIF_BASE + 0xC8)
#define SDRAM_OPERATION_REG        (OMAP730_TCMIF_BASE + 0x80)
#define SDRAM_MANUAL_CMD_REG       (OMAP730_TCMIF_BASE + 0x84)
#define EMIFF_ABORT_ADDRESS        (OMAP730_TCMIF_BASE + 0x98)
#define EMIFF_ABORT_TYPE           (OMAP730_TCMIF_BASE + 0x9C)
   
/* TCMIF Register Definitions */

#define OMAP_EMIFS_CONFIG_FR       (1 << 4)
#define OMAP_EMIFS_CONFIG_PDE      (1 << 3)
#define OMAP_EMIFS_CONFIG_PWD_EN   (1 << 2)
#define OMAP_EMIFS_CONFIG_BM       (1 << 1)
#define OMAP_EMIFS_CONFIG_WP       (1 << 0)


/* ---------------------------------------------------------------------------
 * OMAP730 LCD Register Definitions
 * ---------------------------------------------------------------------------
 */

/* LCD Register Offsets */

#define TI925_LCD_CONTROL          (OMAP730_LCD_BASE + 0x00)
#define TI925_LCD_TIMING0          (OMAP730_LCD_BASE + 0x04)
#define TI925_LCD_TIMING1          (OMAP730_LCD_BASE + 0x08)
#define TI925_LCD_TIMING2          (OMAP730_LCD_BASE + 0x0c)
#define TI925_LCD_STATUS           (OMAP730_LCD_BASE + 0x10)
#define TI925_LCD_SUBPANEL         (OMAP730_LCD_BASE + 0x14)
#define LCD_LINEINT                (OMAP730_LCD_BASE + 0x18)
#define LCD_DISPLAYSTATUS          (OMAP730_LCD_BASE + 0x1c)

#define OMAP_LCD_CONTROL           TI925_LCD_CONTROL


/* TI925_LCD_CONTROL Register Definitions */

#define LCD_EN_POS                 0
#define VSYNC_MASK_POS             2
#define LOADMASK_POS               4
#define DONEMASK_POS               3
#define LCDTFT_POS                 7
#define PXL_GATED_POS              11
#define FDD_POS                    12
#define FDD_MSK                    0xFF

/* TI925_LCD_TIMING0 Register Definitions */

#define PPL_POS                    0
#define PPL_MSK                    0x3FF
#define HSW_POS                    10
#define HSW_MSK                    0x3F
#define HFP_POS                    16
#define HFP_MSK                    0xFF
#define HBP_POS                    24
#define HBP_MSK                    0xFF

/* TI925_LCD_TIMING1 Register Definitions */

#define LPP_POS                    0
#define LPP_MSK                    0x3FF
#define VSW_POS                    10
#define VSW_MSK                    0x3F
#define VFP_POS                    16
#define VFP_MSK                    0xFF
#define VBP_POS                    24
#define VBP_MSK                    0xFF

/* TI925_LCD_TIMING2 Register Definitions */

#define PCD_POS                    0
#define PCD_MSK                    0xFF
#define ACB_POS                    8
#define ACB_MSK                    0xFF
#define ACBI_POS                   16
#define ACBI_MSK                   0x0F
#define IVS_POS                    20
#define IHS_POS                    21
#define IPC_POS                    22
#define IEO_POS                    23

/* TI925_LCD_STATUS Register Definitions */

#define LP_INT                     (1 << 6)
#define FUF_INT                    (1 << 5)
#define LINE_INT                   (1 << 4)
#define ABC_INT                    (1 << 3)
#define SYNC_LOST_INT              (1 << 2)
#define VS_INT                     (1 << 1)
#define DONE_INT                   (1 << 0)


/* Memory chunk set aside for the Framebuffer in SRAM  */

#define SRAM_FRAMEBUFFER_MEMORY    OMAP730_SRAM_BASE


/* ---------------------------------------------------------------------------
 * OMAP730 MMC Register Definitions
 * ---------------------------------------------------------------------------
 */

/* MMC Register Offsets */

#define OMAP_MMC_CMD               (OMAP730_MMC_BASE + 0x00)  /* MMC Command */
#define OMAP_MMC_ARGL              (OMAP730_MMC_BASE + 0x02)  /* MMC argument low */
#define OMAP_MMC_ARGH              (OMAP730_MMC_BASE + 0x04)  /* MMC argument high */
#define OMAP_MMC_CON               (OMAP730_MMC_BASE + 0x06)  /* MMC system configuration */
#define OMAP_MMC_STAT              (OMAP730_MMC_BASE + 0x08)  /* MMC status */
#define OMAP_MMC_IE                (OMAP730_MMC_BASE + 0x0A)  /* MMC system interrupt enable */
#define OMAP_MMC_CTO               (OMAP730_MMC_BASE + 0x0C)  /* MMC command time-out */
#define OMAP_MMC_DTO               (OMAP730_MMC_BASE + 0x0E)  /* MMC data time-out */
#define OMAP_MMC_DATA              (OMAP730_MMC_BASE + 0x10)  /* MMC TX/RX FIFO data */
#define OMAP_MMC_BLEN              (OMAP730_MMC_BASE + 0x12)  /* MMC block length */
#define OMAP_MMC_NBLK              (OMAP730_MMC_BASE + 0x14)  /* MMC number of blocks */
#define OMAP_MMC_BUF               (OMAP730_MMC_BASE + 0x16)  /* MMC buffer configuration */
#define OMAP_MMC_SPI               (OMAP730_MMC_BASE + 0x18)  /* MMC serial port interface */
#define OMAP_MMC_SDIO              (OMAP730_MMC_BASE + 0x1A)  /* MMC SDIO mode configuration */
#define OMAP_MMC_SYST              (OMAP730_MMC_BASE + 0x1C)  /* MMC system test */
#define OMAP_MMC_REV               (OMAP730_MMC_BASE + 0x1E)  /* MMC module version */
#define OMAP_MMC_RSP0              (OMAP730_MMC_BASE + 0x20)  /* MMC command response 0 */
#define OMAP_MMC_RSP1              (OMAP730_MMC_BASE + 0x22)  /* MMC command response 1 */
#define OMAP_MMC_RSP2              (OMAP730_MMC_BASE + 0x24)  /* MMC command response 2 */
#define OMAP_MMC_RSP3              (OMAP730_MMC_BASE + 0x26)  /* MMC command response 3 */
#define OMAP_MMC_RSP4              (OMAP730_MMC_BASE + 0x28)  /* MMC command response 4 */
#define OMAP_MMC_RSP5              (OMAP730_MMC_BASE + 0x2A)  /* MMC command response 5 */
#define OMAP_MMC_RSP6              (OMAP730_MMC_BASE + 0x2C)  /* MMC command response 6 */
#define OMAP_MMC_RSP7              (OMAP730_MMC_BASE + 0x2E)  /* MMC command response 7 */
#define OMAP_MMC_SYSC              (OMAP730_MMC_BASE + 0x34)  /* MMC System Control */
#define OMAP_MMC_SYSS              (OMAP730_MMC_BASE + 0x34)  /* MMC System Status */


/* OMAP_MMC_CMD Register Definitions */

#define MMC_CMD_DDIR               (1 << 15)  /* Data Direction (Read / Write) */
#define MMC_CMD_SHR                (1 << 14)  /* Stream Command or broadcast host response */
#define MMC_CMD_TYPE_POS           12         /* Command Type (BC, BCR, AC, ADTC) */
#define MMC_CMD_TYPE_MSK           0x3
#define MMC_CMD_BUSY               (1 << 11)  /* Command with busy response */
#define MMC_CMD_RSP_POS            8          /* Command responses */
#define MMC_CMD_RSP_MSK            0x7
#define MMC_CMD_INAB               (1 << 7)   /* Send Initialisation stream/data abort command */
#define MMC_CMD_ODTO               (1 << 6)   /* Card open drain mode / extended command time-out mode */
#define MMC_CMD_INDX_POS           5          /* Command Index */
#define MMC_CMD_INDX_MSK           0x3F

#define MMC_CMD_TYPE_BC            0x00
#define MMC_CMD_TYPE_BCR           0x01
#define MMC_CMD_TYPE_AC            0x02
#define MMC_CMD_TYPE_ADTC          0x03

#define MMC_CMD_RSP_NONE           0x00
#define MMC_CMD_RSP_R1             0x01
#define MMC_CMD_RSP_R2             0x02
#define MMC_CMD_RSP_R3             0x03
#define MMC_CMD_RSP_R4             0x04
#define MMC_CMD_RSP_R5             0x05
#define MMC_CMD_RSP_R6             0x06

/* OMAP_MMC_CON Register Definitions */

#define MMC_CON_DW                 (1 << 15)  /* Data Bus Width */
#define MMC_CON_MODE_POS           12         /* Operating mode select */
#define MMC_CON_MODE_MSK           0x03
#define MMC_CON_POWER_UP           (1 << 11)  /* Power up control */
#define MMC_CON_BE                 (1 << 10)  /* Big endian mode */
#define MMC_CON_CLKD_POS           0          /* Clock Divider */
#define MMC_CON_CLKD_MSK           0x3FF

#define MMC_CON_DW_1BIT            0
#define MMC_CON_DW_4BIT            1

#define MMC_CON_MODE_MMCSD         0x00
#define MMC_CON_MODE_SPI           0x01
#define MMC_CON_MODE_SYSTEST       0x02

/* OMAP_MMC_STAT Register Definitions */

#define MMC_STAT_END_OF_CMD        (1 << 0)   /* End of command phase */
#define MMC_STAT_CARD_DETECT       (1 << 1)   /* Card Detect on DAT3 */
#define MMC_STAT_CARD_BUSY         (1 << 2)   /* Card enter busy state */
#define MMC_STAT_BLOCK_RS          (1 << 3)   /* Block received/sent */
#define MMC_STAT_EOF_BUSY          (1 << 4)   /* Card exit busy state */
#define MMC_STAT_DATA_TIMEOUT      (1 << 5)   /* Data response time-out */
#define MMC_STAT_DATA_CRC          (1 << 6)   /* Date CRC error */
#define MMC_STAT_CMD_TIMEOUT       (1 << 7)   /* Command response time-out */
#define MMC_STAT_CMD_CRC           (1 << 8)   /* Command CRC error */
#define MMC_STAT_R_WAIT            (1 << 9)   /* Card Read Wait */
#define MMC_STAT_A_FULL            (1 << 10)  /* Buffer almost full */
#define MMC_STAT_A_EMPTY           (1 << 11)  /* Buffer almost empty */
#define MMC_STAT_OCR_BUSY          (1 << 12)  /* OCR busy */
#define MMC_STAT_CARD_IRQ          (1 << 13)  /* Card IRQ received */
#define MMC_STAT_CARD_ERR          (1 << 14)  /* Card status error in response */
#define MMC_STAT_CLR               0xFFFF     /* Mask to clear all statuses */

/* OMAP_MMC_IE Register definitions */

#define MMC_IE_CERR                (1 << 14)  /* Card status error */
#define MMC_IE_CIRQ                (1 << 13)  /* Card IRQ */
#define MMC_IE_OCRB                (1 << 12)  /* OCR Busy */
#define MMC_IE_AE                  (1 << 11)  /* Almost Empty */
#define MMC_IE_AF                  (1 << 10)  /* Almost Full */
#define MMC_IE_CRW                 (1 << 9)   /* Card read wait enable */
#define MMC_IE_CCRC                (1 << 8)   /* Command CRC error */
#define MMC_IE_CTO                 (1 << 7)   /* Command response timeout  */
#define MMC_IE_DCRC                (1 << 6)   /* Data CRC Error */
#define MMC_IE_DTO                 (1 << 5)   /* Data response timeout */
#define MMC_IE_EOFB                (1 << 4)   /* Card exit busy state */
#define MMC_IE_BRS                 (1 << 3)   /* Block received/sent */
#define MMC_IE_CB                  (1 << 2)   /* Card enter busy state */
#define MMC_IE_CD                  (1 << 1)   /* Card Detect */
#define MMC_IE_EOC                 (1 << 0)   /* End of command */

/* OMAP_MMC_BUF Register Definitions */

#define MMC_BUF_RXDE               (1 << 15)  /* Receive DMA channel enable */
#define MMC_BUF_AFL_POS            8          /* Buffer almost full level */
#define MMC_BUF_AFL_MSK            0x1F
#define MMC_BUF_TXDE               (1 << 7)   /* Transmit DMA channel enable */
#define MMC_BUF_AEL_POS            0          /* Buffer almost full level */
#define MMC_BUF_AEL_MSK            0x1F

/* OMAP_MMC_SYSS Register Definitions */

#define MMC_SYSS_RSTD              (1 << 0)   /* Reset completed */


/* ---------------------------------------------------------------------------
 * OMAP730 DMA Register definitions
 * ---------------------------------------------------------------------------
 */

#define OMAP_DMA_BASE              (OMAP730_DMA_BASE + 0x000)
#define OMAP_DMA_GCR               (OMAP730_DMA_BASE + 0x400)
#define OMAP_DMA_GSCR              (OMAP730_DMA_BASE + 0x404)
#define OMAP_DMA_GRST              (OMAP730_DMA_BASE + 0x408)

#define OMAP_DMA_GCR_REG		(OMAP_DMA_BASE + 0x400)
#define OMAP_DMA_GSCR_REG    		(OMAP_DMA_BASE + 0x404)
#define OMAP_DMA_GRST_REG    		(OMAP_DMA_BASE + 0x408)
#define OMAP_DMA_HW_ID_REG   		(OMAP_DMA_BASE + 0x442)
#define OMAP_DMA_PCH2_ID_REG 		(OMAP_DMA_BASE + 0x444)
#define OMAP_DMA_PCH0_ID     		(OMAP_DMA_BASE + 0x446)
#define OMAP_DMA_PCH1_ID     		(OMAP_DMA_BASE + 0x448)
#define OMAP_DMA_PCHG_ID     		(OMAP_DMA_BASE + 0x44a)
#define OMAP_DMA_PCHD_ID     		(OMAP_DMA_BASE + 0x44c)
#define OMAP_DMA_CAPS_0_U_REG		(OMAP_DMA_BASE + 0x44e)
#define OMAP_DMA_CAPS_0_L_REG		(OMAP_DMA_BASE + 0x450)
#define OMAP_DMA_CAPS_1_U_REG		(OMAP_DMA_BASE + 0x452)
#define OMAP_DMA_CAPS_1_L_REG		(OMAP_DMA_BASE + 0x454)
#define OMAP_DMA_CAPS_2_REG  		(OMAP_DMA_BASE + 0x456)
#define OMAP_DMA_CAPS_3_REG  		(OMAP_DMA_BASE + 0x458)
#define OMAP_DMA_CAPS_4_REG  		(OMAP_DMA_BASE + 0x45a)
#define OMAP_DMA_PCH2_SR_REG 		(OMAP_DMA_BASE + 0x460)
#define OMAP_DMA_PCH0_SR_REG 		(OMAP_DMA_BASE + 0x480)
#define OMAP_DMA_PCH1_SR_REG 		(OMAP_DMA_BASE + 0x482)
#define OMAP_DMA_PCHD_SR_REG 		(OMAP_DMA_BASE + 0x4c0)
/* Every LCh has its own set of the registers below */
#define OMAP_DMA_BASE_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x00)
#define OMAP_DMA_CSDP_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x00)
#define OMAP_DMA_CCR_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x02)
#define OMAP_DMA_CICR_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x04)
#define OMAP_DMA_CSR_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x06)
#define OMAP_DMA_CSSA_L_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x08)
#define OMAP_DMA_CSSA_U_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x0a)
#define OMAP_DMA_CDSA_L_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x0c)
#define OMAP_DMA_CDSA_U_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x0e)
#define OMAP_DMA_CEN_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x10)
#define OMAP_DMA_CFN_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x12)
#define OMAP_DMA_CFI_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x14)
#define OMAP_DMA_CEI_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x16)
#define OMAP_DMA_CSAC_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x18)
#define OMAP_DMA_CDAC_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x1a)
#define OMAP_DMA_CDEI_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x1c)
#define OMAP_DMA_CDFI_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x1e)
#define OMAP_DMA_COLOR_L_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x20)
#define OMAP_DMA_COLOR_U_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x22)
#define OMAP_DMA_CCR2_REG(n)		(OMAP_DMA_BASE + 0x40 * (n) + 0x24)
#define OMAP_DMA_CLNK_CTRL_REG(n)	(OMAP_DMA_BASE + 0x40 * (n) + 0x28)
#define OMAP_DMA_LCH_CTRL_REG(n)	(OMAP_DMA_BASE + 0x40 * (n) + 0x2a)

#define OMAP_DMA_TOUT_IRQ           	(1 << 0)
#define OMAP_DMA_DROP_IRQ           	(1 << 1)
#define OMAP_DMA_HALF_IRQ           	(1 << 2)
#define OMAP_DMA_FRAME_IRQ          	(1 << 3)
#define OMAP_DMA_LAST_IRQ           	(1 << 4)
#define OMAP_DMA_BLOCK_IRQ          	(1 << 5)
#define OMAP_DMA_SYNC_IRQ           	(1 << 6)

#define OMAP_DMA_DATA_TYPE_S8       	0x00
#define OMAP_DMA_DATA_TYPE_S16      	0x01
#define OMAP_DMA_DATA_TYPE_S32		0x02

#define OMAP_DMA_SYNC_ELEMENT       	0x00
#define OMAP_DMA_SYNC_FRAME         	0x01

#define OMAP_DMA_PORT_EMIFF         	0x00
#define OMAP_DMA_PORT_EMIFS         	0x01
#define OMAP_DMA_PORT_OCP_T1        	0x02
#define OMAP_DMA_PORT_TIPB          	0x03
#define OMAP_DMA_PORT_OCP_T2        	0x04
#define OMAP_DMA_PORT_MPUI          	0x05

#define OMAP_DMA_AMODE_CONSTANT     	0x00
#define OMAP_DMA_AMODE_POST_INC     	0x01
#define OMAP_DMA_AMODE_SINGLE_IDX   	0x02
#define OMAP_DMA_AMODE_DOUBLE_IDX   	0x03

#define OMAP_DMA_PACK_NO		0x00
#define OMAP_DMA_PACK_YES		0x01

#define	OMAP_DMA_BURST_NO		0x00
#define	OMAP_DMA_BURST_4		0x02
#define	OMAP_DMA_BURST_8		0x03

#define	OMAP_DMA_PRIO_LOW		0x00
#define	OMAP_DMA_PRIO_HIGH		0x01

/* DMA_GCR Register Definitions */

#define GCR_FREE                   (1 << 2)
#define GCR_CLK_AUTOGATING_ON      (1 << 3)
#define GCR_ROUND_ROBIN_DISABLE    (1 << 4)

#define NO_GLOBAL_DMA_ACCESS 		0

#define OMAP_DMA_GCR_FREE   		4
#define OMAP_DMA_CCR_EN         	(1 << 7)

#define OMAP_DMA_CLNK_CTRL_STOP		(1 << 14)
#define OMAP_DMA_CLNK_CTRL_EN		(1 << 15)

#define	OMAP_DMA_LCH_CTRL_2D		0
#define OMAP_DMA_LCH_CTRL_P		2

/* DMA_GSCR Register Definitions */

#define GSCR_OMAP_3_1_MAPPING_DISABLE   (1 << 3)

/* DMA_GRST Register Definitions */

#define GRST_SW_RESET              (1 << 0)


/* Global Register selection */

#define NO_GLOBAL_DMA_ACCESS       0


/* Channel select field
 * NOTE: all other channels are linear, chan0 is 0, chan1 is 1, etc...
 */
#define LCD_CHANNEL 0x2F

/* 
 * LCD DMA Registers
 */

/* Register Select Field (LCD) */

#define DMA_LCD_CSDP               0 
#define DMA_LCD_CCR                1 
#define DMA_LCD_CTRL               2 
#define DMA_LCD_TOP_B1_L           4 
#define DMA_LCD_TOP_B1_U           5 
#define DMA_LCD_BOT_B1_L           6 
#define DMA_LCD_BOT_B1_U           7 
#define DMA_LCD_TOP_B2_L           8 
#define DMA_LCD_TOP_B2_U           9 
#define DMA_LCD_BOT_B2_L           10
#define DMA_LCD_BOT_B2_U           11
#define DMA_LCD_SRC_EI_B1          12
#define DMA_LCD_SRC_FI_B1_L        13
#define DMA_LCD_SRC_FI_B1_U        26
#define DMA_LCD_SRC_EI_B2          14
#define DMA_LCD_SRC_FI_B2_L        15
#define DMA_LCD_SRC_FI_B2_U        27
#define DMA_LCD_SRC_EN_B1          16
#define DMA_LCD_SRC_EN_B2          17
#define DMA_LCD_SRC_FN_B1          18
#define DMA_LCD_SRC_FN_B2          19
#define DMA_LCD_LCH_CTRL           21

/* DMA_LCD_CTRL Register Definitions */

#define LCD_BLOCK_MODE             (1 << 0)
#define LCD_BLOCK_IT_IE            (1 << 1)
#define LCD_BUS_ERROR_IT_IE        (1 << 2)
#define LCD_FRAME_1_IT_COND        (1 << 3)
#define LCD_FRAME_2_IT_COND        (1 << 4) 
#define LCD_BUS_ERROR_IT_COND      (1 << 5)
#define LCD_SOURCE_IMIF            (1 << 6)

/* DMA_LCD_CCR Register Definitions */

#define CCR_HIGH_PRIO              (1 << 6)
#define CCR_EN                     (1 << 7)
#define CCR_AUTOINT                (1 << 8)
#define CCR_REPEAT                 (1 << 9)
#define OMAP3_1_COMPATIBLE_DISABLE (1 << 10)
#define SRC_AMODE_B1_POSTINC       (1 << 12)
#define SRC_AMODE_B2_DOUBLE        (3 << 14)

/* DMA_LCD_CSDP Register Definitions */

#define SRC_DATA_TYPE_16           (1 << 0)
#define SRC_SDRAM                  (0 << 2)
#define SRC_IMIF                   (1 << 2)
#define SRC_PACK                   (1 << 6)
#define SRC_BURST_EN_0             (0 << 7)
#define SRC_BURST_EN_4             (2 << 7)
#define SRC_BURST_EN_8             (3 << 7)
#define DST_SDRAM                  (0 << 9)
#define DST_IMIF                   (1 << 9)
#define DST_DATA_TYPE_16           (1 << 11)
#define DST_PACK                   (1 << 13)
#define DST_BURST_EN_0             (0 << 14)
#define DST_BURST_EN_4             (2 << 14)
#define DST_BURST_EN_8             (3 << 14)


/* ---------------------------------------------------------------------------
 *  OMAP730 EAC (Audio) Register definitions
 * ---------------------------------------------------------------------------
 */
#define EAC_CPCFR1			(OMAP730_EAC_BASE + 0x00)
#define EAC_CPCFR2			(OMAP730_EAC_BASE + 0x02)
#define EAC_CPCFR3			(OMAP730_EAC_BASE + 0x04)
#define EAC_CPCFR4			(OMAP730_EAC_BASE + 0x06)
#define EAC_CPTCTL			(OMAP730_EAC_BASE + 0x08)
#define EAC_CPTTADR			(OMAP730_EAC_BASE + 0x0A)
#define EAC_CPTDATL			(OMAP730_EAC_BASE + 0x0C)
#define EAC_CPTDATH			(OMAP730_EAC_BASE + 0x0E)
#define EAC_CPTVSLL			(OMAP730_EAC_BASE + 0x10)
#define EAC_CPTVSLH			(OMAP730_EAC_BASE + 0x12)
#define EAC_MPCTR			(OMAP730_EAC_BASE + 0x20)
#define EAC_MPMCCFR			(OMAP730_EAC_BASE + 0x22)
#define EAC_MPACCFR			(OMAP730_EAC_BASE + 0x24)
#define EAC_MPADLTR			(OMAP730_EAC_BASE + 0x26)
#define EAC_MPADMTR			(OMAP730_EAC_BASE + 0x28)
#define EAC_MPADLRR			(OMAP730_EAC_BASE + 0x2A)
#define EAC_MPADMRR			(OMAP730_EAC_BASE + 0x2C)
#define EAC_BPCTR			(OMAP730_EAC_BASE + 0x30)
#define EAC_BPMCCFR			(OMAP730_EAC_BASE + 0x32)
#define EAC_BPACCFR			(OMAP730_EAC_BASE + 0x34)
#define EAC_BPADLTR			(OMAP730_EAC_BASE + 0x36)
#define EAC_BPADMTR			(OMAP730_EAC_BASE + 0x38)
#define EAC_BPADLRR			(OMAP730_EAC_BASE + 0x3A)
#define EAC_BPADMRR			(OMAP730_EAC_BASE + 0x3C)
#define EAC_AMSCFR			(OMAP730_EAC_BASE + 0x40)
#define EAC_AMVCTR			(OMAP730_EAC_BASE + 0x42)
#define EAC_AM1VCTR			(OMAP730_EAC_BASE + 0x44)
#define EAC_AM2VCTR			(OMAP730_EAC_BASE + 0x46)
#define EAC_AM3VCTR			(OMAP730_EAC_BASE + 0x48)
#define EAC_ASTCTR			(OMAP730_EAC_BASE + 0x4A)
#define EAC_APD1LCR			(OMAP730_EAC_BASE + 0x4C)
#define EAC_APD1RCR			(OMAP730_EAC_BASE + 0x4E)
#define EAC_APD2LCR			(OMAP730_EAC_BASE + 0x50)
#define EAC_APD2RCR			(OMAP730_EAC_BASE + 0x52)
#define EAC_APD3LCR			(OMAP730_EAC_BASE + 0x54)
#define EAC_APD3RCR			(OMAP730_EAC_BASE + 0x56)
#define EAC_APD4R			(OMAP730_EAC_BASE + 0x58)
#define EAC_ADWDR			(OMAP730_EAC_BASE + 0x5A)
#define EAC_ADRDR			(OMAP730_EAC_BASE + 0x5C)
#define EAC_AGCFR			(OMAP730_EAC_BASE + 0x5E)
#define EAC_AGCTR			(OMAP730_EAC_BASE + 0x60)
#define EAC_AGCFR2			(OMAP730_EAC_BASE + 0x62)

#define MIXER_x_A_MASK			0x007F
#define MIXER_x_B_MASK			0x7F00
#define	MIXER_x_A_GAIN_OFFSET		0
#define	MIXER_x_B_GAIN_OFFSET		8

#define EAC_AGCTR_RESERVED		0x07F0
#define	EAC_AGCTR_EACPWD		0x0001
#define	EAC_AGCTR_AUDEN			0x0002
#define EAC_AGCTR_MCLK_EN		0x0008
#define EAC_AGCTR_DMAWEN		0x0800
#define EAC_AGCTR_DMAREN		0x1000

#define EAC_AGCFR_RESERVED		0xF800
#define EAC_AGCFR_B8_16			0x0200
#define EAC_AGCFR_MN_ST			0x0400
#define	EAC_AGCFR_AUD_CKSRC_12MHZ	0x0010
#define	EAC_AGCFR_AUD_CKSRC_13MHZ	0x002C
#define EAC_AGCFR_FSINT_MASK		0x00C0
#define EAC_AGCFR_FSINT_8KHZ		0x0000
#define EAC_AGCFR_FSINT_11KHZ		0x0040
#define EAC_AGCFR_FSINT_22KHZ		0x0080
#define EAC_AGCFR_FSINT_44KHZ		0x00C0

#define	EAC_AMSCFR_DEFAULT_SWITCHES	0x0BE7
//#define	EAC_AMSCFR_DEFAULT_SWITCHES	0x00A0

#define	EAC_AMVCTR_RD_DMA_OFFSET	0
#define	EAC_AMVCTR_WR_DMA_OFFSET	8

#define EAC_ASTCTR_ATTEN		0x0001

#define EAC_CPTCTL_RESERVED		0xFF00
#define EAC_CPTCTL_CRST			0x0001
#define EAC_CPTCTL_CPEN			0x0008
#define EAC_CPTCTL_TXE			0x0020
#define EAC_CPTCTL_RXF			0x0080

#define EAC_CPCFR1_MODE_I2S		0x000C

#define	EAC_CPCFR2_I2S_20BITS		0x001B

#define	EAC_CPCFR3_I2S_INPUT		0x00EB

#define EAC_CPCFR4_I2S_DIV7		0x0007

#define	EAC_MPMCCFR_DEFAULT_MASTER_NOCOMP_16BITS	0x01EF

#define EAC_MPCTR_DISABLEALL		0x0000
#define	EAC_MPCTR_PRE_MC_16		0x0008
#define	EAC_MPCTR_MC_EN			0x0080
#define	EAC_MPCTR_CKEN			0x0001

#define EAC_BPCTR_DISABLEALL		0x0000

#define EAC_BPMCCFR_DEFAULT_SLAVE_NOCOMP_16BITS		0x00EF

#define	EAC_BPCTR_PRE_MC_16		0x0008
#define	EAC_BPCTR_MC_EN			0x0080
#define	EAC_BPCTR_CKEN			0x0001

#define	SOFT_REQ_REG_EAC12M_DPLL_REQ	0x4000

#define PCC_PERIPH_SOURCE_EAC_CLK_SOURCE		0x0010
#define	CAM_CLK_CTRL_SYSTEM_CLK_EN	0x0004


/* ---------------------------------------------------------------------------
 * OMAP730 RTC Register definitions
 * ---------------------------------------------------------------------------
 */

#define OMAP_RTC_SECONDS_REG       (OMAP730_RTC_BASE + 0x00)   
#define OMAP_RTC_MINUTES_REG       (OMAP730_RTC_BASE + 0x01)   
#define OMAP_RTC_HOURS_REG         (OMAP730_RTC_BASE + 0x02)   
#define OMAP_RTC_DAYS_REG          (OMAP730_RTC_BASE + 0x03)   
#define OMAP_RTC_MONTHS_REG        (OMAP730_RTC_BASE + 0x04)   
#define OMAP_RTC_YEARS_REG         (OMAP730_RTC_BASE + 0x05)   
#define OMAP_RTC_WEEKS_REG         (OMAP730_RTC_BASE + 0x06)   
#define OMAP_RTC_RESERVED_07       (OMAP730_RTC_BASE + 0x07)   
#define OMAP_RTC_ALARM_SECONDS_REG (OMAP730_RTC_BASE + 0x08)
#define OMAP_RTC_ALARM_MINUTES_REG (OMAP730_RTC_BASE + 0x09)
#define OMAP_RTC_ALARM_HOURS_REG   (OMAP730_RTC_BASE + 0x0A)
#define OMAP_RTC_ALARM_DAYS_REG    (OMAP730_RTC_BASE + 0x0B)
#define OMAP_RTC_ALARM_MONTHS_REG  (OMAP730_RTC_BASE + 0x0C)
#define OMAP_RTC_ALARM_YEARS_REG   (OMAP730_RTC_BASE + 0x0D)
#define OMAP_RTC_RESERVED_0E       (OMAP730_RTC_BASE + 0x0E)   
#define OMAP_RTC_RESERVED_0F       (OMAP730_RTC_BASE + 0x0F)   
#define OMAP_RTC_CTRL_REG          (OMAP730_RTC_BASE + 0x10)
#define OMAP_RTC_STATUS_REG        (OMAP730_RTC_BASE + 0x11)
#define OMAP_RTC_INTERRUPTS_REG    (OMAP730_RTC_BASE + 0x12)
#define OMAP_RTC_COMP_LSB_REG      (OMAP730_RTC_BASE + 0x13)
#define OMAP_RTC_COMP_MSB_REG      (OMAP730_RTC_BASE + 0x14)
#define OMAP_RTC_RES_PROG_REG      (OMAP730_RTC_BASE + 0x14)

/* RTC Control Register bit fields : */

#define OMAP_RTC_CTRL_STOP         (1<<0)

/* RTC Status Register bit fields: */

#define OMAP_RTC_STATUS_POWER_UP   (1<<7)
#define OMAP_RTC_STATUS_ALARM      (1<<6)
#define OMAP_RTC_STATUS_1D_EVENT   (1<<5)
#define OMAP_RTC_STATUS_1H_EVENT   (1<<4)
#define OMAP_RTC_STATUS_1M_EVENT   (1<<3)
#define OMAP_RTC_STATUS_1S_EVENT   (1<<2)
#define OMAP_RTC_STATUS_RUN        (1<<1)
#define OMAP_RTC_STATUS_BUSY       (1<<0)

/* RTC Interrupt Register bit fields: */

#define OMAP_RTC_INTERRUPTS_IT_ALARM  (1<<3)
#define OMAP_RTC_INTERRUPTS_IT_TIMER  (1<<2)


/* ---------------------------------------------------------------------------
 * OMAP730 uWire Register definitions
 * ---------------------------------------------------------------------------
 */

#define UWIRE_TDR                  (OMAP730_UWIRE_BASE + 0x00)
#define UWIRE_RDR                  (OMAP730_UWIRE_BASE + 0x00)
#define UWIRE_CSR                  (OMAP730_UWIRE_BASE + 0x02)
#define UWIRE_SR1                  (OMAP730_UWIRE_BASE + 0x04)
#define UWIRE_SR2                  (OMAP730_UWIRE_BASE + 0x06)
#define UWIRE_SR3                  (OMAP730_UWIRE_BASE + 0x08)
#define UWIRE_SR4                  (OMAP730_UWIRE_BASE + 0x0A)
#define UWIRE_SR5                  (OMAP730_UWIRE_BASE + 0x0C)



/* UWIRE_CSR Register Definitions */

#define UWIRE_NB_BITS_RD_POS       0
#define UWIRE_NB_BITS_RD_MSK       0x1F
#define UWIRE_NB_BITS_WR_POS       5
#define UWIRE_NB_BITS_WR_MSK       0x1F
#define UWIRE_CSR_INDEX_POS        10
#define UWIRE_CSR_INDEX_MSK        0x3
#define UWIRE_CS_CMD_POS           12
#define UWIRE_START_POS            13
#define UWIRE_CSRB_POS             14
#define UWIRE_RDRB_POS             15

/* UWIRE_SR1 Register Definitions */

#define CS0_EDGE_RD_POS            0
#define CS0_EDGE_WR_POS            1
#define CS0CS_LVL_POS              2
#define CS0_FRQ_POS                3
#define CS0_FRQ_MSK                0x3
#define CS0_CHK_POS                5
#define CS1_EDGE_RD_POS            6
#define CS1_EDGE_WR_POS            7
#define CS1CS_LVL_POS              8
#define CS1_FRQ_POS                9
#define CS1_FRQ_MSK                0x3
#define CS1_CHK_POS                11

/* UWIRE_SR2 Register Definitions */

#define CS2_EDGE_RD_POS            0
#define CS2_EDGE_WR_POS            1
#define CS2CS_LVL_POS              2
#define CS2_FRQ_POS                3
#define CS2_FRQ_MSK                0x3
#define CS2_CHK_POS                5
#define CS3_EDGE_RD_POS            6
#define CS3_EDGE_WR_POS            7
#define CS3CS_LVL_POS              8
#define CS3_FRQ_POS                9
#define CS3_FRQ_MSK                0x3
#define CS3_CHK_POS                11

/* UWIRE_SR3 Register Definitions */
#define SR3_CLK_EN_POS             0
#define SR3_CLK_FREQ_POS           1
#define SR3_CLK_FREQ_MSK           0x3




/* ---------------------------------------------------------------------------
 * OMAP730 I2C Register definitions
 * ---------------------------------------------------------------------------
 */

#define I2C_REV                    (OMAP730_I2C_BASE + 0x00)
#define I2C_IE                     (OMAP730_I2C_BASE + 0x02)
#define I2C_STAT                   (OMAP730_I2C_BASE + 0x04)
#define I2C_SYSS                   (OMAP730_I2C_BASE + 0x08)
#define I2C_BUF                    (OMAP730_I2C_BASE + 0x0A)
#define I2C_CNT                    (OMAP730_I2C_BASE + 0x0C)
#define I2C_DATA                   (OMAP730_I2C_BASE + 0x0E)
#define I2C_SYSC                   (OMAP730_I2C_BASE + 0x10)
#define I2C_CON                    (OMAP730_I2C_BASE + 0x12)
#define I2C_OA                     (OMAP730_I2C_BASE + 0x14)

#define I2C_SA                     (OMAP730_I2C_BASE + 0x16)
#define I2C_PSC                    (OMAP730_I2C_BASE + 0x18)
#define I2C_SCLL                   (OMAP730_I2C_BASE + 0x1A)
#define I2C_SCLH                   (OMAP730_I2C_BASE + 0x1C)
#define I2C_SYSTEST                (OMAP730_I2C_BASE + 0x1E)

/* I2C_IE Register Bit Fields */

#define I2C_IE_XRDY_IE             (1 << 4)  /* Transmit data ready interrupt enable */
#define I2C_IE_RRDY_IE             (1 << 3)  /* Receive data ready interrupt enable */
#define I2C_IE_ARDY_IE             (1 << 2)  /* Register access ready interrupt enable */
#define I2C_IE_NACK_IE             (1 << 1)  /* No acknowledgment interrupt enable */
#define I2C_IE_AL_IE               (1 << 0)  /* Arbitration lost interrupt enable */

/* I2C_STAT Register Bit Fields */

#define I2C_STAT_SBD               (1 << 15)  /* Single byte data */
#define I2C_STAT_BB                (1 << 12)  /* Bus busy */
#define I2C_STAT_ROVR              (1 << 11)  /* Receive overrun */
#define I2C_STAT_XUDF              (1 << 10)  /* Transmit underflow */
#define I2C_STAT_AAS               (1 <<  9)  /* Address as slave */

#define I2C_STAT_GC                (1 << 5)   /* General Call */
#define I2C_STAT_XRDY              (1 << 4)   /* Transmit data ready */
#define I2C_STAT_RRDY              (1 << 3)   /* Receive data ready */
#define I2C_STAT_ARDY              (1 << 2)   /* Register access ready */
#define I2C_STAT_NACK              (1 << 1)   /* No acknowledgment interrupt enable */
#define I2C_STAT_AL                (1 << 0)   /* Arbitration lost interrupt enable */

/* I2C_BUF Register Bit Fields */

#define I2C_BUF_RDMA_EN            (1 << 15)  /* Receive DMA channel enable */
#define I2C_BUF_XDMA_EN            (1 <<  7)  /* Transmit DMA channel enable */

/* I2C_CON Register Bit Fields */

#define I2C_CON_EN                 (1 << 15)  /* I2C module enable */
#define I2C_CON_BE                 (1 << 14)  /* Big endian mode */
#define I2C_CON_STB                (1 << 11)  /* Start byte mode (master mode only) */
#define I2C_CON_MST                (1 << 10)  /* Master/slave mode */
#define I2C_CON_TRX                (1 <<  9)  /* Transmitter/receiver mode (master mode only) */
#define I2C_CON_XA                 (1 <<  8)  /* Expand address */
#define I2C_CON_STP                (1 <<  1)  /* Stop condition (master mode only) */
#define I2C_CON_STT                (1 <<  0)  /* Start condition (master mode only) */

/* I2C_SYSTEST Register Bit Fields */

#define I2C_SYSTEST_ST_EN          (1 << 15)  /* System test enable */
#define I2C_SYSTEST_FREE           (1 << 14)  /* Free running mode (on breakpoint) */
#define I2C_SYSTEST_TMODE_MASK     (3 << 12)  /* Test mode select */
#define I2C_SYSTEST_TMODE_SHIFT    (12)       /* Test mode select */
#define I2C_SYSTEST_SCL_I          (1 << 3)   /* SCL line sense input value */
#define I2C_SYSTEST_SCL_O          (1 << 2)   /* SCL line drive output value */
#define I2C_SYSTEST_SDA_I          (1 << 1)   /* SDA line sense input value */
#define I2C_SYSTEST_SDA_O          (1 << 0)   /* SDA line drive output value */

/* I2C_SYSS Register Bit Fields */

#define I2C_SYSS_RDONE             (1 << 0)   /* Reset Done*/

/* I2C_SYSC Register Bit Fields */

#define I2C_SYSC_SRST              (1 << 1)   /* Soft Reset */


/* ---------------------------------------------------------------------------
 * OMAP730 Interrupt Handlers (See TRM 6.4.2)
 * Note the IH2 Split. This is because there are 64 IH2 lines ; The ITR, MIR, 
 * ISR and ILRn are split into halves of 32 bits each, 0x100 bytes apart.
 * To understand this, see the code in arch/arm/mach-omap730/irq.c read_ih()
 * ---------------------------------------------------------------------------
 */

#define OMAP_IH1_BASE              OMAP730_IH1_BASE
#define OMAP_IH2_0_BASE            (OMAP730_IH2_BASE + 0x0000)
#define OMAP_IH2_1_BASE            (OMAP730_IH2_BASE + 0x0100)

/* OMAP Interrupt Register Offsets (IH1 & IH2) */

#define IRQ_ITR                    0x00  /* Interrupt Register */
#define IRQ_MIR                    0x04  /* Mask Interrupt Register */
#define IRQ_SIR_IRQ                0x10  /* Interrupt Encoded Source for IRQ Register */
#define IRQ_SIR_FIQ                0x14  /* Interrupt Encoded Source for FIA Register */
#define IRQ_CONTROL_REG            0x18  /* Interrupt Control Register */
#define IRQ_ILR0                   0x1C  /* Priority Level Register */
#define IRQ_ISR                    0x9C  /* Software Interrupt Set Register (aka ISIR) */
#define IRQ_ENHANCED_CNTL_REG      0xA0  /* Enhanced Control Register (IH1 Only) */
#define IRQ_STATUS                 0xA0  /* OCP Status Register (IH2 Only) */
#define IRQ_OCP_CFG                0xA4  /* OCP Configuration Register (IH2 Only) */
#define IRQ_INTH_REV               0xA8  /* Revision ID Register (IH2 Only) */

/* ILR Register Bit Fields */

#define ILR_PRIORITY_MASK          (0x3c)
#define ILR_PRIORITY_SHIFT         (2)
#define ILR_LEVEL_TRIGGER          (1 << 1)
#define ILR_FIQ                    (1 << 0)

/* IRQ_ILR0 Register Definitions */

#define IRQ_LEVEL_INT              1
#define IRQ_EDGE_INT               0

/* IRQ_CONTROL_REG Register Definitions */

#define NEW_IRQ_AGR                (1 << 0)
#define NEW_FIQ_AGR                (1 << 1)


/* ---------------------------------------------------------------------------
 * OMAP730 TIMER32K Register Definitions
 * ---------------------------------------------------------------------------
 */

/* TIMER32K Registers Offsets */
#define TIMER32k_TVR               0x00
#define TIMER32k_TCR               0x04
#define TIMER32k_CR                0x08


/* TIMER32K_CR Register Definitions */
#define TIMER32k_TSS               (1<<0)
#define TIMER32k_TRB               (1<<1)
#define TIMER32k_INT               (1<<2)
#define TIMER32k_ARL               (1<<3)


/* ---------------------------------------------------------------------------
 * OMAP730 TIMERn Register Definitions
 * Note : There are 3 MPU Timers ; TIMER1, TIMER2, TIMER3 that are similar
 * ---------------------------------------------------------------------------
 */

/* TIMERn Register offsets (Add to OMAP730_TIMERn_BASE) */
#define CNTL_TIMER                 0x00  /* MPU Control Timer */
#define LOAD_TIM                   0x04  /* MPU Load Timer */
#define READ_TIM                   0x08  /* MPU Read Timer */

/*  CNTL_TIMER Register Definitions */

#define MPUTIM_FREE                (1<<6)
#define MPUTIM_CLOCK_ENABLE        (1<<5)
#define MPUTIM_PTV_MASK            (0x7<<PTV_BIT)
#define MPUTIM_PTV_BIT             2
#define MPUTIM_AR                  (1<<1)
#define MPUTIM_ST                  (1<<0)


#ifndef __ASSEMBLY__

#define mputimer_base(n) \
    ((volatile mputimer_regs_t*)(OMAP730_TIMER_BASE + ((n) * OMAP730_TIMER_SIZE)))

#endif   /* ! __ASSEMBLY__ */


/* ---------------------------------------------------------------------------
 * OMAP730 MPU GPIO Register definitions
 * Note : In the TRM, this can be referred to as either MPUIO, GPIO or 
 *        MPU GPIO depending on which part you're reading.
 * ---------------------------------------------------------------------------
 */


/* MPU Register offsets */

#define GPIO_DATA_INPUT_REG        (OMAP730_GPIO_BASE + 0x00)
#define GPIO_DATA_OUTPUT_REG       (OMAP730_GPIO_BASE + 0x04)
#define GPIO_DATA_DIRECTION_REG    (OMAP730_GPIO_BASE + 0x08)
#define GPIO_INT_CONTROL_REG       (OMAP730_GPIO_BASE + 0x0c)
#define GPIO_INT_MASK_REG          (OMAP730_GPIO_BASE + 0x10)
#define GPIO_INT_STATUS_REG        (OMAP730_GPIO_BASE + 0x14)

/* GPIO_DATA_DIRECTION_REG Register Definitions */

#define GPIO_DIR_OUTPUT            0
#define GPIO_DIR_INPUT             1

/* MPU GPIO utility Macros */

#ifndef __ASSEMBLY__

#define GPIO_BANK_OFFSET(bank)      (bank * OMAP730_GPIO_SIZE)

#define GPIO_DATA_INPUT_REGn(bank)      (GPIO_DATA_INPUT_REG + GPIO_BANK_OFFSET(bank))
#define GPIO_DATA_OUTPUT_REGn(bank)     (GPIO_DATA_OUTPUT_REG + GPIO_BANK_OFFSET(bank))
#define GPIO_DATA_DIRECTION_REGn(bank)  (GPIO_DATA_DIRECTION_REG + GPIO_BANK_OFFSET(bank))
#define GPIO_INT_CONTROL_REGn(bank)     (GPIO_INT_CONTROL_REG  + GPIO_BANK_OFFSET(bank))
#define GPIO_INT_MASK_REGn(bank)        (GPIO_INT_MASK_REG + GPIO_BANK_OFFSET(bank))
#define GPIO_INT_STATUS_REGn(bank)      (GPIO_INT_STATUS_REG + GPIO_BANK_OFFSET(bank))

/* Set gpio 'pin' to GPIO_DIR_OUTPUT or GPIO_DIR_INPUT. */
static inline void GPIO_SET_DIR(__u32 pin, __u32 in)
{
  *(REG32(GPIO_DATA_DIRECTION_REG + ((pin / OMAP730_GPIO_NB) * OMAP730_GPIO_SIZE))) &= ~(1 << (pin % OMAP730_GPIO_NB));
  *(REG32(GPIO_DATA_DIRECTION_REG + ((pin / OMAP730_GPIO_NB) * OMAP730_GPIO_SIZE))) |= in << (pin % OMAP730_GPIO_NB);
}

static inline __u32 GPIO_GET_DATA_INPUT(__u32 pin)
{
  return *(REG32(GPIO_DATA_INPUT_REG + ((pin / OMAP730_GPIO_NB) * OMAP730_GPIO_SIZE)));
}
 
static inline __u32 GPIO_GET_DATA_OUTPUT(__u32 pin)
{
  return *(REG32(GPIO_DATA_OUTPUT_REG + ((pin / OMAP730_GPIO_NB) * OMAP730_GPIO_SIZE)));
}

/* Get the input state of GPIO 'pin' 1 for high, 0 for low */
static inline __u32 GPIO_GET_INPUT_STATE(__u32 pin)
{
  return (GPIO_GET_DATA_INPUT(pin) >> (pin % OMAP730_GPIO_NB)) & 1;
}

/* Get the output state of GPIO 'pin' 1 for high, 0 for low */
static inline __u32 GPIO_GET_OUTPUT_STATE(__u32 pin)
{
  return (GPIO_GET_DATA_OUTPUT(pin) >> (pin % OMAP730_GPIO_NB)) & 1;
}

/* Set the output state of GPIO 'pin' 1 for high, 0 for low */
static inline void GPIO_SET_STATE(__u32 pin, __u32 val)
{
  __u32 temp = GPIO_GET_DATA_INPUT(pin);
  
  temp &= ~(1 << (pin % OMAP730_GPIO_NB));
  temp |= (val & 1) << (pin % OMAP730_GPIO_NB);
  *(REG32(GPIO_DATA_OUTPUT_REG + ((pin / OMAP730_GPIO_NB) * OMAP730_GPIO_SIZE))) = temp;
}
#endif


/* ---------------------------------------------------------------------------
 * OMAP730 TIPB Register Definitions
 * ---------------------------------------------------------------------------
 * 
 */

#define TIPB_PUBLIC_CNTL_BASE      0xfffed300
#define MPU_PUBLIC_TIPB_CNTL_REG   (TIPB_PUBLIC_CNTL_BASE + 0x8)
#define TIPB_PRIVATE_CNTL_BASE     0xfffeca00
#define MPU_PRIVATE_TIPB_CNTL_REG  (TIPB_PRIVATE_CNTL_BASE + 0x8)


/*
 * ---------------------------------------------------------------------------
 * OMAP730 Camera Interface Register Definitions
 * ---------------------------------------------------------------------------
 */
#define CAMERA_BASE                OMAP730_CAMERA_IF_BASE

#define CAM_CTRLCLOCK_REG          (OMAP730_CAMERA_IF_BASE + 0x00)
#define CAM_IT_STATUS_REG          (OMAP730_CAMERA_IF_BASE + 0x04)
#define CAM_MODE_REG               (OMAP730_CAMERA_IF_BASE + 0x08)
#define CAM_STATUS_REG             (OMAP730_CAMERA_IF_BASE + 0x0C)
#define CAM_CAMDATA_REG            (OMAP730_CAMERA_IF_BASE + 0x10)
#define CAM_GPIO_REG               (OMAP730_CAMERA_IF_BASE + 0x14)
#define CAM_PEAK_CTR_REG           (OMAP730_CAMERA_IF_BASE + 0x18)

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

/* CAM_CTRLCLOCK Register Definitions */

#define FOSCMOD_BIT                0
#define FOSCMOD_MASK               (0x7 << FOSCMOD_BIT)
#define   FOSCMOD_12MHz            0x0
#define   FOSCMOD_6MHz             0x2
#define   FOSCMOD_9_6MHz           0x4
#define   FOSCMOD_24MHz            0x5
#define   FOSCMOD_8MHz             0x6
#define POLCLK                     (1<<3)
#define CAMEXCLK_EN                (1<<4)
#define MCLK_EN                    (1<<5)
#define DPLL_EN                    (1<<6)  /* Called APLL_EN in the TRM, but left for compat. */
#define LCLK_EN                    (1<<7)

/* CAM_IT_STATUS Register Definitions */

#define V_UP                       (1<<0)
#define V_DOWN                     (1<<1)
#define H_UP                       (1<<2)
#define H_DOWN                     (1<<3)
#define FIFO_FULL                  (1<<4)
#define DATA_XFER                  (1<<5)

/* CAM_MODE Register Definitions */

#define CAMOSC                     (1<<0)
#define IMGSIZE_BIT                1
#define IMGSIZE_MASK               (0x3 << IMGSIZE_BIT)
#define   IMGSIZE_CIF              (0x0 << IMGSIZE_BIT)    /* 352x288 */
#define   IMGSIZE_QCIF             (0x1 << IMGSIZE_BIT)    /* 176x144 */
#define   IMGSIZE_VGA              (0x2 << IMGSIZE_BIT)    /* 640x480 */
#define   IMGSIZE_QVGA             (0x3 << IMGSIZE_BIT)    /* 320x240 */
#define ORDERCAMD                  (1<<3)
#define EN_V_UP                    (1<<4)
#define EN_V_DOWN                  (1<<5)
#define EN_H_UP                    (1<<6)
#define EN_H_DOWN                  (1<<7)
#define EN_DMA                     (1<<8)
#define THRESHOLD                  (1<<9)
#define THRESHOLD_BIT              9
#define THRESHOLD_MASK             (0x7f<<9)
#define EN_NIRQ                    (1<<16)
#define EN_FIFO_FULL               (1<<17)
#define RAZ_FIFO                   (1<<18)

/* CAM_STATUS Register Definitions */

#define VSTATUS                    (1<<0)
#define HSTATUS                    (1<<1)

/* CAM_GPIO Register Definitions */

#define CAM_RST                    (1<<0)


/*
 * ---------------------------------------------------------------------------
 * OMAP730 USB OTG (USB On The Go Interface) Register Definitions
 * ---------------------------------------------------------------------------
 */

#define OTG_REV                    (OMAP730_USB_OTG_BASE + 0x00)  /* OTG Controller Revision Number */
#define OTG_SYSCON_1               (OMAP730_USB_OTG_BASE + 0x04)  /* OTG System Configuration group 1 */
#define OTG_SYSCON_2               (OMAP730_USB_OTG_BASE + 0x08)  /* OTG System Configuration group 2 */
#define OTG_CTRL                   (OMAP730_USB_OTG_BASE + 0x0c)  /* OTG Control */
#define OTG_IRQ_EN                 (OMAP730_USB_OTG_BASE + 0x10)  /* OTG Interrupt Enable */
#define OTG_IRQ_SRC                (OMAP730_USB_OTG_BASE + 0x14)  /* OTG Interrupt Source Identification */
#define OTG_VC                     (OMAP730_USB_OTG_BASE + 0xFC)  /* USB Vendor Code */

/* OTG_SYSCON_1 Register definitions */

#define USB2_TRX_MODE_POS			24
#define USB1_TRX_MODE_POS			20
#define USB0_TRX_MODE_POS			16

#define USB2_TRX_MODE_MASK			0x07
#define USB1_TRX_MODE_MASK			0x07
#define USB0_TRX_MODE_MASK			0x07

#define SIX_PIN_UNI_HIGH			0x00
#define FOUR_PIN_BI					0x01
#define THREE_PIN_BI				0x02
#define SIX_PIN_UNI					0x03

/* OTG_SYSCON_2 Register definitions */

#define OTG_EN						(1 << 31)
#define USBx_SYNCHRO				(1 << 30)
#define OTG_PADEN					(1 << 10)
#define HMC_PADEN					(1 << 9)
#define UHOST_EN					(1 << 8)

#define HMC_MODE_POS				0
#define HMC_MODE_MASK				0x3F

#define HMC_MODE_0					(0 << HMC_MODE_POS)
#define HMC_MODE_1					(1 << HMC_MODE_POS)


/*
 * ---------------------------------------------------------------------------
 * OMAP730 USB HOST ADDRESS
 * ---------------------------------------------------------------------------
 */
#define USB_HOST_BASE				0XFFFBA000
#define USB_HOST_SIZE				SZ_4K


/*
 * ---------------------------------------------------------------------------
 * OMAP730 PCC UPLD Register Definitions
 * ---------------------------------------------------------------------------
 */

#define CLOCK_CTRL_REG             (OMAP730_PCC_ULPD_BASE + 0x030)
#define SOFT_REQ_REG               (OMAP730_PCC_ULPD_BASE + 0x034)
#define SOFT_DISABLE_REQ_REG       (OMAP730_PCC_ULPD_BASE + 0x068)
#define CAM_CLK_CTRL               (OMAP730_PCC_ULPD_BASE + 0x07C)
#define PCC_CTRL_REG               (OMAP730_PCC_ULPD_BASE + 0x100)
#define PCC_PERIPH_CLOCK_SOURCE_SEL (OMAP730_PCC_ULPD_BASE + 0x108)

/* SOFT_REQ_REG Register Definitions */

#define SOFT_DPLL_REQ              (1 << 0)
#define SOFT_USB_REQ               (1 << 3)
#define USB_REQ_EN                 (1 << 4)
#define SOFT_CAM_DPLL_MCK0_REQ     (1 << 7)
#define SOFT_USB_OTG_DPLL_REQ      (1 << 8)
#define SOFT_UART1_DPLL_REQ        (1 << 9)
#define SOFT_UART3_DPLL_REQ        (1 << 11)


/* SOFT_DISABLE_REQ_REG Register Definitions */

#define DIS_CAM_DPLL_MCLK_REQ      (1 << 5)
#define DIS_USB_HOST_DPLL_REQ      (1 << 6)
#define DIS_UART1_DPLL_REQ         (1 << 7)
#define DIS_UART3_DPLL_REQ         (1 << 9)

/* CAM_CLK_CTRL Register Definitions */
#define CAM_CLOCK_EN               (1 << 0)
#define CAM_CLK_DIV                (1 << 1)
#define SYSTEM_CLK_EN              (1 << 2)

/* PCC_CTRL_REG Register Definitions */

#define CLK_SWITCH_CMD             (1 << 0)
#define APLL_ALWAYS_ON             (1 << 1)
#define SLC_OUT_DIV                (1 << 2)
#define ULPD_INPUT_SWITCH_STS      (1 << 3)
#define APLL96_INPUT_SWITCH_STS    (1 << 4)



/*
 * ---------------------------------------------------------------------------
 * OMAP730 CLKM Register Definitions
 * ---------------------------------------------------------------------------
 */

/* CLKM Register offsets */

#define ARM_CKCTL                  REG16(OMAP730_CLKM_BASE + 0x00)  /* MPU CLock Control Prescalar selection register */
#define ARM_IDLECT1                REG16(OMAP730_CLKM_BASE + 0x04)  /* MPU Idle Enable Control 1 */    
#define ARM_IDLECT2                REG16(OMAP730_CLKM_BASE + 0x08)  /* MPU Idle Enable Control 2 */    
#define ARM_EWUPCT                 REG16(OMAP730_CLKM_BASE + 0x0C)  /* MPU Restore Power delay */    
#define ARM_RSTCT1                 REG16(OMAP730_CLKM_BASE + 0x10)  /* Master Software reset */    
#define ARM_RSTCT2                 REG16(OMAP730_CLKM_BASE + 0x14)  /* Peripherals Reset */    
#define ARM_SYSST                  REG16(OMAP730_CLKM_BASE + 0x18)  /* MPU Clock Reset Status */
#define ARM_CKOUT1                 REG16(OMAP730_CLKM_BASE + 0x1C)  /* MPU Clock Out Definition */
#define ARM_CKOUT2                 REG16(OMAP730_CLKM_BASE + 0x20)  /* MPU reserved */
#define ARM_IDLECT3                REG16(OMAP730_CLKM_BASE + 0x24)  /* MPU Idle Enable Control 3 */

/* GSM Specific */
#define CLKM_CKCTL_OFFSET          0x00                             /* Register Offsets */

/* Some of the following Register bit definitions are given as positions and some as 
 * fields. This is to maintain compatibility with the existing 1510 code */

/* ARM_CKCTL Bit shift Register Definitions */

#define PERDIV                     0   /* Prescalar value of CK_GEN1 to MPU external peripheral Clock Domain */
#define LCDDIV                     2   /* Prescalar value of CK_GEN3 to LCD controller clock */
#define ARMDIV                     4   /* Prescalar value for the DSP Clock Domain */
#define DSPDIV                     6   /* Prescalar value of CK_GEN1 to MPU Clock Domain */
#define TCDIV                      8   /* Prescalar value of CK_GEN3 to TC Clock Domain */
#define DSPMMUDIV                  10  /* Prescalar value of CK_GEN2 to DSPMMU Clock Domain */
#define ARM_TIMXO                  12  /* Selects either CK_REF or CK_GEN1 to supply internal MPU timers */
#define ARM_INTHCK_SEL             14  /* Selects either ARM_CK or ARM_CK/2 to supply  ARM_INTH_CK */
#define ARM_CKCTL_DIV_MSK          0x03  /* Mask for clock prescalars */

/* ARM_IDLECT1 Bit shift Register Definitions */

#define IDLWDT_ARM                 0
#define IDLXORP_ARM                1
#define IDLPER_ARM                 2
#define IDLIF_ARM                  6
#define IDLDPLL_ARM                7
#define IDLTIM_ARM                 9
#define WKUP_MODE_ARM              10
#define IDL_CLKOUT_ARM             12

/* ARM_IDLECT2 Bit shift Register Definitions */

#define EN_WDTCK                   0
#define EN_XORPCK                  1
#define EN_PERCK                   2
#define EN_LCDCK                   3
#define EN_APICK                   6
#define EN_TIMCK                   7
#define DMACK_REQ                  8
/* Bit 9 is reserved. TRM says keep set to 0 */
#define EN_CKOUT_ARM               11

/* ARM_IDLECT3 Register Definitions */

#define EN_OCPI_CK                 (1 << 0)
#define IDLOCPI_ARM                (1 << 1)
#define EN_TC1_CK                  (1 << 2)
#define IDLTC1_ARM                 (1 << 3)
#define EN_TC2_CK                  (1 << 4)
#define IDLTC2_ARM                 (1 << 5)

/* ARM_RSTCT1 Register Definitions */

#define ARM_RSTCT1_SW_RST          (1 << 3)
#define ARM_RSTCT1_DSP_RST         (1 << 2)
#define ARM_RSTCT1_DSP_EN          (1 << 1)
#define ARM_RSTCT1_ARM_RST         (1 << 0)

/* ARM_RSTCT2 Register Definitions */

#define ARM_RSTCT2_PER_EN          (1 << 0)   /* Controls ARM PER_nRSET pins */
#define EN_PER                     0          // For compatibility...



/*
 * ---------------------------------------------------------------------------
 * OMAP730 OCPI Register Definitions
 * Note : The TRM uses different naming conventions here, but these are 
 * already in use in the omap1510 USB code, so left here for compatibility.
 * ---------------------------------------------------------------------------
 */

#define OCPI_FAULT                 REG32(OMAP730_OCPI_BASE + 0x00)  /* OCP Address Fault */
#define OCPI_CMD_FAULT             REG32(OMAP730_OCPI_BASE + 0x04)  /* OCP Master Command Fault */
#define OCPI_SINT0                 REG32(OMAP730_OCPI_BASE + 0x08)  /* OCP SINTERRUPT0 */
#define OCPI_TABORT                REG32(OMAP730_OCPI_BASE + 0x0c)  /* OCP type of abort */
#define OCPI_SINT1                 REG32(OMAP730_OCPI_BASE + 0x10)  /* OCP SINTERRUPT1 */
#define OCPI_PROT                  REG32(OMAP730_OCPI_BASE + 0x14)  /* OCP Protection */
#define OCPI_SEC                   REG32(OMAP730_OCPI_BASE + 0x18)  /* OCP-I Secure Mode */

/* OCPI Register masks */
#define OCPI_PROT_MASK				0xFF
#define OCPI_SEC_MASK				0x7F

/*
 * ---------------------------------------------------------------------------
 * OMAP730 DPLL Register Definitions
 * ---------------------------------------------------------------------------
 */

/* DPLL1 Register offsets */

#define DPLL1_CTL_REG              REG16(OMAP730_DPLL1_BASE + 0x00)

/* DPLL1_CTL_REG Register Definitions */

#define DPLL1_CTL_LS_DISABLE       (1 << 15)  /* Controls Level Shifter Power Down */
#define DPLL1_CTL_IAI              (1 << 14)  /* Initialise after Idle */
#define DPLL1_CTL_IOB              (1 << 13)  /* Initialise on Break */
#define DPLL1_CTL_TEST             (1 << 12)  /* Test Output Clock Control */
#define DPLL1_CTL_PLL_MULT_POS     7          /* DPLL Multiply Value */
#define DPLL1_CTL_PLL_MULT_MSK     0x1F
#define DPLL1_CTL_PLL_DIV_POS      5          /* DPLL Divide Value */
#define DPLL1_CTL_PLL_DIV_MSK      0x03
#define DPLL1_CTL_PLL_ENABLE       (1 << 4)   /* Enter Lock Mode */
#define DPLL1_CTL_BYPASS_DIV_POS   2          /* Clock Freq in bypass mode */
#define DPLL1_CTL_BYPASS_DIV_MSK   0x03          
#define DPLL1_CTL_BREAKLN          (1 << 1)   /* Indicates whether lock broken */
#define DPLL1_CTL_LOCK             (1 << 0)   /* Indicates if in lock mode */


/*
 * ---------------------------------------------------------------------------
 * OMAP730 ICR Register Definitions
 * ---------------------------------------------------------------------------
 */

#define M_ICR                      (OMAP730_ICR_BASE + 0x00)  /* MPU-S to GSM-S flags */
#define G_ICR                      (OMAP730_ICR_BASE + 0x02)  /* GSM-S to MPU-S flags */
#define M_CTL                      (OMAP730_ICR_BASE + 0x04)  /* MPU-S Control */
#define G_CTL                      (OMAP730_ICR_BASE + 0x06)  /* GSM-S Control */
#define PM_BA                      (OMAP730_ICR_BASE + 0x0A)  /* Program memory Base Address */
#define DM_BA                      (OMAP730_ICR_BASE + 0x0C)  /* Data memory Base Address */
#define RM_BA                      (OMAP730_ICR_BASE + 0x0E)  /* Random memory Base Address */
#define SSPI_TAS                   (OMAP730_ICR_BASE + 0x12)  /* Syren SPI test and set */


/* ---------------------------------------------------------------------------
 * Differentiating processor versions for those who care.
 * ---------------------------------------------------------------------------
 */


#define OMAP730                    0  /* Only one version of the OMAP730 currently */
 
#ifndef __ASSEMBLY__
int cpu_type(void);
#endif

#endif /*  __ASM_ARCH_OMAP_HARDWARE_H */
