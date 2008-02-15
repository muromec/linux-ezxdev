/*
 *  linux/include/asm-arm/arch-ti925/omap710/irqs.h
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
#ifndef __ASM_ARCH_OMAP710_IRQS_H
#define __ASM_ARCH_OMAP710_IRQS_H
/* ---------------------------------------------------------------------------
 *  Interrupts
 * ---------------------------------------------------------------------------
 */

/* 
 *  IRQ Numbbers for INT1
 * 
 */
#define INT_IH2_FIQ		0
#define INT_IH2_IRQ		1
#define INT_USB_NIRQ		2 
#define INT_USB_SOF_NIRQ	3
#define INT_ICR			4
#define INT_EAC			5
#define INT_MPUIO1		6
#define INT_MPUIO2		7
#define INT_MPUIO3		8
#define INT_ABORT		9
// RESERVED on 710		10
// RESERVED on 710		11
// RESERVED on 710		12
#define INT_BRIDGE_PRIV		13
#define INT_GPIO		14
#define INT_UART3		15 
#define INT_UART_MODEM2		15 
#define INT_TIMER3		16
// RESERVED on 710		17
// RESERVED on 710		18
#define INT_DMA_CH0_6		19
#define INT_DMA_CH1_7		20
#define INT_DMA_CH2_8		21
#define INT_DMA_CH3		22
#define INT_DMA_CH4		23
#define INT_DMA_CH5		24
#define INT_DMA_LCD		25
#define INT_TIMER1		26
#define INT_WD_TIMER		27
#define INT_BRIDGE_PUB		28
// RESERVED on 710		29
#define INT_TIMER2		30
#define INT_LCD_CTRL		31

/* 
 * IRQ Numbers for Interrupt Handler 2
 */
#define IH2_BASE		32
 
#define INT_HW_ERRORS		0 + IH2_BASE
#define INT_PWR_FAIL		1 + IH2_BASE
// Next, Correction to h/w docs. h/w docs had INT_CFCD
// and INT_CFCD reversed from what is shown next.
#define INT_CFIREQ		2 + IH2_BASE
#define INT_CFCD		3 + IH2_BASE
#define INT_VSPI		4 + IH2_BASE
#define INT_USB_WAKE		5 + IH2_BASE
#define INT_MPU_EXT		6 + IH2_BASE
#define INT_SPI_100KHz		7 + IH2_BASE
// RESERVED on 710		8 + IH2_BASE
// RESERVED on 710		9 + IH2_BASE
// RESERVED on 710		10 + IH2_BASE
// RESERVED on 710		11 + IH2_BASE
#define INT_McBSP1TX		12 + IH2_BASE
#define INT_McBSP1RX		13 + IH2_BASE
#define INT_UART2		14 + IH2_BASE // backwards from omap1510
#define INT_UART_IRDA		14 + IH2_BASE // backwards from omap1510
#define INT_UART1		15 + IH2_BASE // ditto
#define INT_UART_MODEM		15 + IH2_BASE // backwards from omap1510
#define INT_BT_MCS1TX		16 + IH2_BASE
#define INT_BT_MCS1RX		17 + IH2_BASE
#define INT_BT_MCSIFE		18 + IH2_BASE
#define INT_COM_MCSi2TX		19 + IH2_BASE
#define INT_COM_MCSi2RX		20 + IH2_BASE
#define INT_COM_MCSi2FE		21 + IH2_BASE
#define INT_OS_32kHz_TIMER	22 + IH2_BASE
#define TIMER_IRQ		INT_OS_32kHz_TIMER
#define INT_MMC			23 + IH2_BASE
#define INT_GAUGE_32K		24 + IH2_BASE
#define INT_RTC_PER		25 + IH2_BASE
#define INT_RTC_ALRM		26 + IH2_BASE
#define INT_USB_GEN1		29 + IH2_BASE
#define INT_USB_SBI1		30 + IH2_BASE
//#define INT_COM_SPI_RO	31 + IH2_BASE

/*
 * IRQ Numbers for interrupts muxed through GPIO
 */
#define IH_GPIO_BASE	64
#define INT_GPIO0	(IH_GPIO_BASE + 0)
#define INT_GPIO1	(IH_GPIO_BASE + 1)
#define INT_GPIO2	(IH_GPIO_BASE + 2)
#define INT_GPIO3	(IH_GPIO_BASE + 3)
#define INT_GPIO4	(IH_GPIO_BASE + 4)
#define INT_GPIO5	(IH_GPIO_BASE + 5)
#define INT_GPIO6	(IH_GPIO_BASE + 6)
#define INT_GPIO7	(IH_GPIO_BASE + 7)
#define INT_FPGA	(IH_GPIO_BASE + 8)
#define INT_GPIO9	(IH_GPIO_BASE + 9)
#define INT_GPIO10	(IH_GPIO_BASE + 10)
#define INT_GPIO11	(IH_GPIO_BASE + 11)
#define INT_GPIO12	(IH_GPIO_BASE + 12)
#define INT_GPIO13	(IH_GPIO_BASE + 13)
#define INT_GPIO14	(IH_GPIO_BASE + 14)
#define INT_GPIO15	(IH_GPIO_BASE + 15)

/*
 * IRQ Numbers for interrupts muxed through the FPGA
 */
#define IH_FPGA_BASE	80
#define INT_FPGA0	(IH_FPGA_BASE + 0)
#define INT_FPGA1	(IH_FPGA_BASE + 1)
#define INT_FPGA2	(IH_FPGA_BASE + 2)
#define INT_FPGA3	(IH_FPGA_BASE + 3)
#define INT_FPGA4	(IH_FPGA_BASE + 4)
#define INT_FPGA5	(IH_FPGA_BASE + 5)
#define INT_FPGA6	(IH_FPGA_BASE + 6)
#define INT_FPGA7	(IH_FPGA_BASE + 7)
#define INT_FPGA8	(IH_FPGA_BASE + 8)
#define INT_FPGA9	(IH_FPGA_BASE + 9)
#define INT_FPGA10	(IH_FPGA_BASE + 10)
#define INT_FPGAIT_RI	(IH_FPGA_BASE + 11)
#define INT_FPGAIT_DCD	(IH_FPGA_BASE + 12)
#define INT_FPGA13	(IH_FPGA_BASE + 13)
#define INT_FPGA_ETHR	(IH_FPGA_BASE + 14)
#define INT_FPGAIT_EXT	(IH_FPGA_BASE + 15)

#define MAXIRQNUM	(IH_FPGA_BASE + 15)
#define MAXFIQNUM	MAXIRQNUM
#define MAXSWINUM	MAXIRQNUM

#define NR_IRQS		(MAXIRQNUM + 1)

#endif
