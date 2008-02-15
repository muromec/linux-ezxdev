/*
 *  linux/include/asm-arm/arch-omap/irqs.h
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 *   <source@mvista.com>
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
#ifndef __ASM_ARCH_OMAP1610_IRQS_H
#define __ASM_ARCH_OMAP1610_IRQS_H
/* ---------------------------------------------------------------------------
 *  Interrupts
 * ---------------------------------------------------------------------------
 */

/* These interrupt vectors are for the OMAP1610.
 * The interrupt vectors for the OMAP1509/OMAP1610DC are different.
 */

/* 
 *  IRQ Numbers for INT1
 * 
 */
#define INT_IH2_IRQ       0
#define INT_CAMERA        1
#define INT_IH2_FIQ       2
#define INT_FIQ           3
#define INT_McBSP2_TX        4
#define INT_McBSP2_RX        5
#define INT_RTDX          6
#define INT_DSP_MMU_ABORT 7
#define INT_HOST          8
#define INT_ABORT         9
#define INT_DSP_MAILBOX1  10
#define INT_DSP_MAILBOX2  11
#define INT_LCD_LINE      12
#define INT_BRIDGE_PRIV   13
#define INT_GPIO1         14
#define INT_UART3         15
#define INT_TIMER3        16
#define INT_GPTIMER1      17
#define INT_GPTIMER2      18
#define INT_DMA_CH0       19
#define INT_DMA_CH1       20
#define INT_DMA_CH2       21
#define INT_DMA_CH3       22
#define INT_DMA_CH4       23
#define INT_DMA_CH5       24
#define INT_DMA_LCD       25
#define INT_TIMER1        26
#define INT_WD_TIMER      27
#define INT_BRIDGE_PUB    28
#define INT_SSR_FIFO_CH0  29
#define INT_TIMER2        30
#define INT_LCD_CTRL      31

/* 
 * IRQ Numbers for Interrupt Handler 2
 */
#define IH2_BASE 32
 
#define INT_HW_ERRORS     (IH2_BASE + 0)
#define INT_KEYBOARD      (IH2_BASE + 1)
#define INT_uWireTX       (IH2_BASE + 2)
#define INT_uWireRX       (IH2_BASE + 3)
#define INT_I2C           (IH2_BASE + 4)
#define INT_MPUIO         (IH2_BASE + 5)
#define INT_USB_HHC1      (IH2_BASE + 6)
#define INT_USB_HHC2      (IH2_BASE + 7)
#define INT_USB_OTG       (IH2_BASE + 8)
#define INT_SoSSI_ATTN    (IH2_BASE + 9)
#define INT_McBSP3TX      (IH2_BASE + 10)
#define INT_McBSP3RX      (IH2_BASE + 11)
#define INT_McBSP1TX      (IH2_BASE + 12)
#define INT_McBSP1RX      (IH2_BASE + 13)
#define INT_UART1         (IH2_BASE + 14) // Bluetooth
#define INT_UART2         (IH2_BASE + 15)
#define INT_BT_MCS1TX     (IH2_BASE + 16)
#define INT_BT_MCS1RX     (IH2_BASE + 17)
#define INT_IH2_18        (IH2_BASE + 18) /* free */
#define INT_SoSSI_MATCH   (IH2_BASE + 19)
#define INT_USB_W2FC_Geni (IH2_BASE + 20)
#define INT_1WIRE         (IH2_BASE + 21)
#define INT_OS_32kHz_TIMER (IH2_BASE + 22)
#define INT_MMC_SDIO1     (IH2_BASE + 23)
#define INT_GAUGE_32K     (IH2_BASE + 24)
#define INT_RTC_TIMER     (IH2_BASE + 25)
#define INT_RTC_ALARM     (IH2_BASE + 26)
#define INT_MEM_STICK     (IH2_BASE + 27)
#define INT_DSP_MMU       (IH2_BASE + 28)
#define INT_USB_W2FC_ISO_ON     (IH2_BASE + 29)
#define INT_USB_W2FC_NON_ISO_ON (IH2_BASE + 30)
#define INT_COM_McBSP2_RO    (IH2_BASE + 31)
#define INT_STI_GLOBAL    (IH2_BASE + 32)
#define INT_STI_WAKEUP    (IH2_BASE + 33)
#define INT_GPTIMER3      (IH2_BASE + 34)
#define INT_GPTIMER4      (IH2_BASE + 35)
#define INT_GPTIMER5      (IH2_BASE + 36)
#define INT_GPTIMER6      (IH2_BASE + 37)
#define INT_GPTIMER7      (IH2_BASE + 38)
#define INT_GPTIMER8      (IH2_BASE + 39)
#define INT_GPIO2         (IH2_BASE + 40)
#define INT_GPIO3         (IH2_BASE + 41)
#define INT_MMC_SDIO2     (IH2_BASE + 42)
#define INT_CF            (IH2_BASE + 43)
#define INT_COMMRX        (IH2_BASE + 44)
#define INT_COMMTX        (IH2_BASE + 45)
#define INT_PERIF_WAKEUP  (IH2_BASE + 46)
#define INT_IH2_47        (IH2_BASE + 47) /* free */
#define INT_GPIO4         (IH2_BASE + 48)
#define INT_SPI           (IH2_BASE + 49)
#define INT_CCPSTATUS     (IH2_BASE + 50)
#define INT_CCP_FIFO_NE   (IH2_BASE + 51)
#define INT_CCP_ATTN      (IH2_BASE + 52)
#define INT_DMA_CH6       (IH2_BASE + 53)
#define INT_DMA_CH7       (IH2_BASE + 54)
#define INT_DMA_CH8       (IH2_BASE + 55)
#define INT_DMA_CH9       (IH2_BASE + 56)
#define INT_DMA_CH10      (IH2_BASE + 57)
#define INT_DMA_CH11      (IH2_BASE + 58)
#define INT_DMA_CH12      (IH2_BASE + 59)
#define INT_DMA_CH13      (IH2_BASE + 60)
#define INT_DMA_CH14      (IH2_BASE + 61)
#define INT_DMA_CH15      (IH2_BASE + 62)
#define INT_NAND_FLASH    (IH2_BASE + 63)
#define INT_USB_HHC2_SUSP (IH2_BASE + 64)
#define INT_SST_EMPTY_CH0 (IH2_BASE + 65)
#define INT_IH2_66        (IH2_BASE + 66) /* free */
#define INT_SSR_OVRRN_CH0 (IH2_BASE + 67)
#define INT_SST_EMPTY_CH1 (IH2_BASE + 68)
#define INT_SSR_FULL_CH1  (IH2_BASE + 69)
#define INT_SSR_OVRRN_CH1 (IH2_BASE + 70)
#define INT_SST_EMPTY_CH2 (IH2_BASE + 71)
#define INT_SSR_FULL_CH2  (IH2_BASE + 72)
#define INT_SSR_OVRRN_CH2 (IH2_BASE + 73)
#define INT_SST_EMPTY_CH3 (IH2_BASE + 74)
#define INT_SSR_FULL_CH3  (IH2_BASE + 75)
#define INT_SSR_OVRRN_CH3 (IH2_BASE + 76)
#define INT_SST_EMPTY_CH4 (IH2_BASE + 77)
#define INT_SSR_FULL_CH4  (IH2_BASE + 78)
#define INT_SSR_OVRRN_CH4 (IH2_BASE + 79)
#define INT_SST_EMPTY_CH5 (IH2_BASE + 80)
#define INT_SSR_FULL_CH5  (IH2_BASE + 81)
#define INT_SSR_OVRRN_CH5 (IH2_BASE + 82)
#define INT_SST_EMPTY_CH6 (IH2_BASE + 83)
#define INT_SSR_FULL_CH6  (IH2_BASE + 84)
#define INT_SSR_OVRRN_CH6 (IH2_BASE + 85)
#define INT_SST_EMPTY_CH7 (IH2_BASE + 86)
#define INT_SSR_FULL_CH7  (IH2_BASE + 87)
#define INT_SSR_OVRRN_CH7 (IH2_BASE + 88)
#define INT_SSR_ERR       (IH2_BASE + 89)
#define INT_SSR           (IH2_BASE + 90)
#define INT_SHA1_MD5      (IH2_BASE + 91)
#define INT_RNG           (IH2_BASE + 92)
#define INT_RNGIDLE       (IH2_BASE + 93)
#define INT_IH2_94        (IH2_BASE + 94) /* free */
#define INT_GDD_LCH0      (IH2_BASE + 95)
#define INT_GDD_LCH1      (IH2_BASE + 96)
#define INT_GDD_LCH2      (IH2_BASE + 97)
#define INT_GDD_LCH3      (IH2_BASE + 98)
#define INT_GDD_LCH4      (IH2_BASE + 99)
#define INT_GDD_LCH5      (IH2_BASE + 100)
#define INT_GDD_LCH6      (IH2_BASE + 101)
#define INT_GDD_LCH7      (IH2_BASE + 102)

#define IH2_LAST (IH2_BASE+127)

/*
 * IRQ Numbers for interrupts muxed through GPIO
 */
#define IH_GPIO_BASE (IH2_LAST+1)
#define INT_GPIO_0    (IH_GPIO_BASE + 0)
#define INT_GPIO_1    (IH_GPIO_BASE + 1)
#define INT_GPIO_2    (IH_GPIO_BASE + 2)
#define INT_GPIO_3    (IH_GPIO_BASE + 3)
#define INT_GPIO_4    (IH_GPIO_BASE + 4)
#define INT_GPIO_5    (IH_GPIO_BASE + 5)
#define INT_GPIO_6    (IH_GPIO_BASE + 6)
#define INT_GPIO_7    (IH_GPIO_BASE + 7)
#define INT_GPIO_8    (IH_GPIO_BASE + 8)
#define INT_GPIO_9    (IH_GPIO_BASE + 9)
#define INT_GPIO_10   (IH_GPIO_BASE + 10)
#define INT_GPIO_11   (IH_GPIO_BASE + 11)
#define INT_GPIO_12   (IH_GPIO_BASE + 12)
#define INT_GPIO_13   (IH_GPIO_BASE + 13)
#define INT_GPIO_14   (IH_GPIO_BASE + 14)
#define INT_GPIO_15   (IH_GPIO_BASE + 15)
#define INT_GPIO_16   (IH_GPIO_BASE + 16)
#define INT_GPIO_17   (IH_GPIO_BASE + 17)
#define INT_GPIO_18   (IH_GPIO_BASE + 18)
#define INT_GPIO_19   (IH_GPIO_BASE + 19)
#define INT_GPIO_20   (IH_GPIO_BASE + 20)
#define INT_GPIO_21   (IH_GPIO_BASE + 21)
#define INT_GPIO_22   (IH_GPIO_BASE + 22)
#define INT_GPIO_23   (IH_GPIO_BASE + 23)
#define INT_GPIO_24   (IH_GPIO_BASE + 24)
#define INT_GPIO_25   (IH_GPIO_BASE + 25)
#define INT_GPIO_26   (IH_GPIO_BASE + 26)
#define INT_GPIO_27   (IH_GPIO_BASE + 27)
#define INT_GPIO_28   (IH_GPIO_BASE + 28)
#define INT_GPIO_29   (IH_GPIO_BASE + 29)
#define INT_GPIO_30   (IH_GPIO_BASE + 30)
#define INT_GPIO_31   (IH_GPIO_BASE + 31)
#define INT_GPIO_32   (IH_GPIO_BASE + 32)
#define INT_GPIO_33   (IH_GPIO_BASE + 33)
#define INT_GPIO_34   (IH_GPIO_BASE + 34)
#define INT_GPIO_35   (IH_GPIO_BASE + 35)
#define INT_GPIO_36   (IH_GPIO_BASE + 36)
#define INT_GPIO_37   (IH_GPIO_BASE + 37)
#define INT_GPIO_38   (IH_GPIO_BASE + 38)
#define INT_GPIO_39   (IH_GPIO_BASE + 39)
#define INT_GPIO_40   (IH_GPIO_BASE + 40)
#define INT_GPIO_41   (IH_GPIO_BASE + 41)
#define INT_GPIO_42   (IH_GPIO_BASE + 42)
#define INT_GPIO_43   (IH_GPIO_BASE + 43)
#define INT_GPIO_44   (IH_GPIO_BASE + 44)
#define INT_GPIO_45   (IH_GPIO_BASE + 45)
#define INT_GPIO_46   (IH_GPIO_BASE + 46)
#define INT_GPIO_47   (IH_GPIO_BASE + 47)
#define INT_GPIO_48   (IH_GPIO_BASE + 48)
#define INT_GPIO_49   (IH_GPIO_BASE + 49)
#define INT_GPIO_50   (IH_GPIO_BASE + 50)
#define INT_GPIO_51   (IH_GPIO_BASE + 51)
#define INT_GPIO_52   (IH_GPIO_BASE + 52)
#define INT_GPIO_53   (IH_GPIO_BASE + 53)
#define INT_GPIO_54   (IH_GPIO_BASE + 54)
#define INT_GPIO_55   (IH_GPIO_BASE + 55)
#define INT_GPIO_56   (IH_GPIO_BASE + 56)
#define INT_GPIO_57   (IH_GPIO_BASE + 57)
#define INT_GPIO_58   (IH_GPIO_BASE + 58)
#define INT_GPIO_59   (IH_GPIO_BASE + 59)
#define INT_GPIO_60   (IH_GPIO_BASE + 60)
#define INT_GPIO_61   (IH_GPIO_BASE + 61)
#define INT_GPIO_62   (IH_GPIO_BASE + 62)
#define INT_GPIO_63   (IH_GPIO_BASE + 63)

#define INT_GPIO_LAST INT_GPIO_63

/*
 * IRQ Numbers for interrupts muxed through MPUIO
 */
#define IH_MPUIO_BASE (INT_GPIO_LAST+1)
#define INT_MPUIO_0    (IH_MPUIO_BASE + 0)
#define INT_MPUIO_1    (IH_MPUIO_BASE + 1)
#define INT_MPUIO_2    (IH_MPUIO_BASE + 2)
#define INT_MPUIO_3    (IH_MPUIO_BASE + 3)
#define INT_MPUIO_4    (IH_MPUIO_BASE + 4)
#define INT_MPUIO_5    (IH_MPUIO_BASE + 5)
#define INT_MPUIO_6    (IH_MPUIO_BASE + 6)
#define INT_MPUIO_7    (IH_MPUIO_BASE + 7)
#define INT_MPUIO_8    (IH_MPUIO_BASE + 8)
#define INT_MPUIO_9    (IH_MPUIO_BASE + 9)
#define INT_MPUIO_10    (IH_MPUIO_BASE + 10)
#define INT_MPUIO_11    (IH_MPUIO_BASE + 11)
#define INT_MPUIO_12    (IH_MPUIO_BASE + 12)
#define INT_MPUIO_13    (IH_MPUIO_BASE + 13)
#define INT_MPUIO_14    (IH_MPUIO_BASE + 14)
#define INT_MPUIO_15    (IH_MPUIO_BASE + 15)

#define INT_MPUIO_LAST INT_MPUIO_15

#define MAXIRQNUM                       (INT_MPUIO_LAST + 1)
#define MAXFIQNUM                       MAXIRQNUM
#define MAXSWINUM                       MAXIRQNUM

#define NR_IRQS                         (MAXIRQNUM + 1)

#ifndef __ASSEMBLY__
extern void omap1610_init_irq(void);
#endif

#endif /* __ASM_ARCH_OMAP1610_IRQS_H */
