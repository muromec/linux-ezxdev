/*
 *  linux/include/asm-arm/arch-omap/omap1510p1.h
 *
 * Copyright (C) 2001 RidgeRun, Inc. (http://www.ridgerun.com)
 * Author: RidgeRun, Inc.
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
#ifndef __ASM_ARCH_OMAP1510_OMAP1510P1_H
#define __ASM_ARCH_OMAP1510_OMAP1510P1_H

/*
 * NOTE:  ALL NEW DEFINITIONS IN THIS FILE NEED TO BE PREVIXED BY IDENTIFIER
 *        OMAP1510P1 since they are specific to the EVM and not the chip.
 */

/* ---------------------------------------------------------------------------
 *  OMAP1510 FPGA
 * ---------------------------------------------------------------------------
 * 
 */
/* maps in the FPGA registers and the ETHR registers */
#define OMAP1510P1_FPGA_BASE              0xE8000000                 // VA
#define OMAP1510P1_FPGA_SIZE              SZ_4K                      // SIZE
#define OMAP1510P1_FPGA_START             0x08000000                 // PA   

#define OMAP1510P1_FPGA_REV_LOW           (OMAP1510P1_FPGA_BASE + 0x0)   /* Revision */ 
#define OMAP1510P1_FPGA_REV_HIGH          (OMAP1510P1_FPGA_BASE + 0x1)   /* Revision */
#define OMAP1510P1_FPGA_LCD_PANEL_CONTROL (OMAP1510P1_FPGA_BASE + 0x2)
#define OMAP1510P1_FPGA_LED_DIGIT         (OMAP1510P1_FPGA_BASE + 0x3)
#define OMAP1510P1_FPGA_HID_SPI           (OMAP1510P1_FPGA_BASE + 0x4)
#define OMAP1510P1_FPGA_POWER             (OMAP1510P1_FPGA_BASE + 0x5)
#define OMAP1510P1_FPGA_ISR_LO            (OMAP1510P1_FPGA_BASE + 0x6)   /* Interrupt Status Register (Low) */ 
#define OMAP1510P1_FPGA_ISR_HI            (OMAP1510P1_FPGA_BASE + 0x7)   /* Interrupt Status Register (High) */
#define OMAP1510P1_FPGA_IMR_LO            (OMAP1510P1_FPGA_BASE + 0x8)   /* Interrupt Mask Register (Low) */
#define OMAP1510P1_FPGA_IMR_HI            (OMAP1510P1_FPGA_BASE + 0x9)   /* Interrupt Mask Register (High) */
#define OMAP1510P1_FPGA_HOST_RESET        (OMAP1510P1_FPGA_BASE + 0xa)
#define OMAP1510P1_FPGA_RST               (OMAP1510P1_FPGA_BASE + 0xb) /* Peripheral Reset */
#define OMAP1510P1_FPGA_AUDIO             (OMAP1510P1_FPGA_BASE + 0xc)
#define OMAP1510P1_FPGA_DIP               (OMAP1510P1_FPGA_BASE + 0xe)
#define OMAP1510P1_FPGA_FPGA_IO           (OMAP1510P1_FPGA_BASE + 0xf)
#define OMAP1510P1_FPGA_UART1             (OMAP1510P1_FPGA_BASE + 0x14)
#define OMAP1510P1_FPGA_UART2             (OMAP1510P1_FPGA_BASE + 0x15)
#define OMAP1510P1_FPGA_OMAP1510_STATUS   (OMAP1510P1_FPGA_BASE + 0x16)
#define OMAP1510P1_FPGA_BOARD_REV         (OMAP1510P1_FPGA_BASE + 0x18)
#define OMAP1510P1_PPT_DATA               (OMAP1510P1_FPGA_BASE + 0x100)
#define OMAP1510P1_PPT_STATUS             (OMAP1510P1_FPGA_BASE + 0x101)
#define OMAP1510P1_PPT_CONTROL            (OMAP1510P1_FPGA_BASE + 0x102)

#define OMAP1510P1_FPGA_TOUCHSCREEN	  (OMAP1510P1_FPGA_BASE + 0x204)
#define OMAP1510P1_FPGA_INFORMATION       (OMAP1510P1_FPGA_BASE + 0x205) 

#define OMAP1510P1_FPGA_ETHR_BASE         0xE8000300

// Power up Giga UART driver, turn on HID clock.
// Turn off BT power, since we're not using it and it
// draws power.
#define OMAP1510P1_FPGA_RESET_VALUE       0x42

#define OMAP1510P1_FPGA_PCR_IF_PD0         (1 << 7)
#define OMAP1510P1_FPGA_PCR_COM2_EN        (1 << 6)
#define OMAP1510P1_FPGA_PCR_COM1_EN        (1 << 5)
#define OMAP1510P1_FPGA_PCR_EXP_PD0        (1 << 4)
#define OMAP1510P1_FPGA_PCR_EXP_PD1        (1 << 3)
#define OMAP1510P1_FPGA_PCR_48MHZ_CLK      (1 << 2)
#define OMAP1510P1_FPGA_PCR_4MHZ_CLK       (1 << 1)
#define OMAP1510P1_FPGA_PCR_RSRVD_BIT0     (1 << 0)

#ifndef OMAP_SDRAM_DEVICE
#define OMAP_SDRAM_DEVICE                 D256M_1X16_4B
#endif

#define OMAP1510P1_IMIF_PRI_VALUE	  0x00
#define OMAP1510P1_EMIFS_PRI_VALUE	  0x00
#define OMAP1510P1_EMIFF_PRI_VALUE	  0x00

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
#define OMAP_FLASH_0_SIZE	SZ_16M

// Intel flash_1, used for cramfs or other flash file systems
#define OMAP_FLASH_1_BASE	0xD9000000	// VA
#define OMAP_FLASH_1_START	0x01000000	// PA
#define OMAP_FLASH_1_SIZE	SZ_16M

/* The FPGA IRQ is cascaded through GPIO_13 */
#define INT_FPGA     (IH_GPIO_BASE + 13)

/*
 * IRQ Numbers for interrupts muxed through the FPGA
 */
#define IH_FPGA_BASE    80
#define INT_FPGA_ATN    (IH_FPGA_BASE + 0)
#define INT_FPGA_ACK    (IH_FPGA_BASE + 1)
#define INT_FPGA2       (IH_FPGA_BASE + 2)
#define INT_FPGA3       (IH_FPGA_BASE + 3)
#define INT_FPGA4       (IH_FPGA_BASE + 4)
#define INT_FPGA5       (IH_FPGA_BASE + 5)
#define INT_FPGA6       (IH_FPGA_BASE + 6)
#define INT_FPGA7       (IH_FPGA_BASE + 7)
#define INT_FPGA8       (IH_FPGA_BASE + 8)
#define INT_FPGA9       (IH_FPGA_BASE + 9)
#define INT_FPGA10      (IH_FPGA_BASE + 10)
#define INT_FPGA11      (IH_FPGA_BASE + 11)
#define INT_FPGA12      (IH_FPGA_BASE + 12)
#define INT_ETHER       (IH_FPGA_BASE + 13)
#define INT_FPGAUART1   (IH_FPGA_BASE + 14)
#define INT_FPGAUART2   (IH_FPGA_BASE + 15)

#define MAXIRQNUM                       (IH_FPGA_BASE + 15)
#define MAXFIQNUM                       MAXIRQNUM
#define MAXSWINUM                       MAXIRQNUM

#define NR_IRQS                         (MAXIRQNUM + 1)


#ifndef __ASSEMBLY__
void fpga_write(unsigned char val, int reg);
unsigned char fpga_read(int reg);
#endif

#endif
