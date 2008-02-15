/*
 *  linux/include/asm-arm/arch-omap/irqs.h
 *
 *  Copyright (C) Greg Lonnon 2001
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
#ifndef __ASM_ARCH_OMAP1510_IRQS_H
#define __ASM_ARCH_OMAP1510_IRQS_H
/* ---------------------------------------------------------------------------
 *  Interrupts
 * ---------------------------------------------------------------------------
 */

/* These interrupt vectors are for the OMAP1510.
 * The interrupt vectors for the OMAP1509/OMAP1510DC are different.
 */

/* 
 *  IRQ Numbers for INT1
 * 
 */
#define INT_IH2_IRQ      0
#define INT_CAMERA       1 
#define INT_RES2         2 
#define INT_FIQ          3
#define INT_SPI_TX       4
#define INT_SPI_RX       5
#define INT_RTDX         6
#define INT_DSP_MMU_ABORT 7
#define INT_HOST         8
#define INT_ABORT        9
#define INT_DSP_MAILBOX1 10
#define INT_DSP_MAILBOX2 11
#define INT_RES12        12
#define INT_BRIDGE_PRIV  13
#define INT_GPIO         14
#define INT_UART3        15
#define INT_TIMER3       16
#define INT_LB_MMU       17
#define INT_RES18        18
#define INT_DMA_CH0_6    19
#define INT_DMA_CH1_7    20
#define INT_DMA_CH2_8    21
#define INT_DMA_CH3      22
#define INT_DMA_CH4      23
#define INT_DMA_CH5      24
#define INT_DMA_LCD      25
#define INT_TIMER1       26
#define INT_WD_TIMER     27
#define INT_BRIDGE_PUB   28
#define INT_LOCAL_BUS    29
#define INT_TIMER2       30
#define INT_LCD_CTRL     31

/* 
 * IRQ Numbers for Interrupt Handler 2
 */
#define IH2_BASE 32
 
#define INT_HW_ERRORS 0 + IH2_BASE
#define INT_KEYBOARD  1 + IH2_BASE
#define INT_uWireTX   2 + IH2_BASE
#define INT_uWireRX   3 + IH2_BASE
#define INT_I2C       4 + IH2_BASE
#define INT_MPUIO     5 + IH2_BASE
#define INT_McBSP3TX  10 + IH2_BASE
#define INT_McBSP3RX  11 + IH2_BASE
#define INT_McBSP1TX  12 + IH2_BASE
#define INT_McBSP1RX  13 + IH2_BASE
#define INT_UART1     14 + IH2_BASE // Bluetooth
#define INT_UART2     15 + IH2_BASE
#define INT_BT_MCS1TX 16 + IH2_BASE
#define INT_BT_MCS1RX 17 + IH2_BASE
#define INT_OS_32kHz_TIMER 22 + IH2_BASE
#define INT_MMC         23 + IH2_BASE
#define INT_GAUGE_32K   24 + IH2_BASE
#define INT_RTC_TIMER	(25 + IH2_BASE)
#define INT_RTC_ALARM	(26 + IH2_BASE)
#define INT_DSP_MMU     28 + IH2_BASE
#define INT_COM_SPI_RO  31 + IH2_BASE

#undef	OMAP1510_USE_32KHZ_TIMER
#ifdef OMAP1510_USE_32KHZ_TIMER
#define	TIMER_IRQ INT_OS_32kHz_TIMER
#else
#define TIMER_IRQ INT_TIMER2
#endif

/*
 * IRQ Numbers for interrupts muxed through GPIO
 */
#define IH_GPIO_BASE 64
#define INT_GPIO0    (IH_GPIO_BASE + 0)
#define INT_GPIO1    (IH_GPIO_BASE + 1)
#define INT_GPIO2    (IH_GPIO_BASE + 2)
#define INT_GPIO3    (IH_GPIO_BASE + 3)
#define INT_GPIO4    (IH_GPIO_BASE + 4)
#define INT_GPIO5    (IH_GPIO_BASE + 5)
#define INT_GPIO6    (IH_GPIO_BASE + 6)
#define INT_GPIO7    (IH_GPIO_BASE + 7)
#define INT_GPIO8    (IH_GPIO_BASE + 8)
#define INT_GPIO9    (IH_GPIO_BASE + 9)
#define INT_GPIO10   (IH_GPIO_BASE + 10)
#define INT_GPIO11   (IH_GPIO_BASE + 11)
#define INT_GPIO12   (IH_GPIO_BASE + 12)
#define INT_GPIO13   (IH_GPIO_BASE + 13)
#define INT_GPIO14   (IH_GPIO_BASE + 14)
#define INT_GPIO15   (IH_GPIO_BASE + 15)

#define IH_BOARD_BASE	(INT_GPIO15 + 1)

#ifndef __ASSEMBLY__
extern void omap1510_init_irq(void);
#endif

/* definition of NR_IRQS is in board-specific header file, which is
 * included via hardware.h
 */
#include <asm/arch/hardware.h>

#endif
