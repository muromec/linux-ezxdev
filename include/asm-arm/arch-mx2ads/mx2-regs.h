/*
 *  linux/include/asm-arm/arch-mx2ads/mx2-regs.h
 *
 *  Motorola MX2 system registers
 *
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *  Copyright (C) 2003 Motorola Semiconductor SUZHOU Ltd
 *  Copyright (C) 2004 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef __ASM_ARCH_MX2_REGS_H__
#define __ASM_ARCH_MX2_REGS_H__

#define B_SET(b)			(1 << (b))
/*
 * AIPI 1 & 2
 */

#define _AIPI_BASE	0x10000000

/* input for x: 1 or 2*/
#define AIPI_BASE(x)	IO_ADDRESS(_AIPI_BASE + (((x) - 1) << 17))
#define AIPI_PSR0(x)	__REG32_2(IO_ADDRESS(_AIPI_BASE+0x00), ((x) - 1) << 17)	/*  32bit Peripheral Size Reg 0 */
#define AIPI_PSR1(x)	__REG32_2(IO_ADDRESS(_AIPI_BASE+0x04), ((x) - 1) << 17)	/*  32bit Peripheral Size Reg 1 */
#define AIPI_PAR(x)	__REG32_2(IO_ADDRESS(_AIPI_BASE+0x08), ((x) - 1) << 17)	/*  32bit Peripheral Access Reg */

/*
 * DMA
 */

#define _DMA_BASE	0x10001000	/* dma base address */
#define _DMA_CH_BASE	(_DMA_BASE+0x80)	/* base address for channel registers */
#define _DMA_2D_SZ_BASE	(_DMA_BASE+0x40)	/* base address for 2D size memory reg */
#define _DMA_TEST_BASE	(_DMA_BASE+0x480)	/* base address for test registers */

#define DMA_BASE		IO_ADDRESS(_DMA_BASE)
#define DMA_DCR			__REG32(IO_ADDRESS(_DMA_BASE))	/*  32bit dma control reg */
#define DMA_DISR		__REG32(IO_ADDRESS(_DMA_BASE+0x004))	/*  32bit dma interrupt status reg */
#define DMA_DIMR		__REG32(IO_ADDRESS(_DMA_BASE+0x008))	/*  32bit dma interrupt mask reg */
#define DMA_DBTOSR		__REG32(IO_ADDRESS(_DMA_BASE+0x00C))	/*  32bit dma burst timeout stat reg */
#define DMA_DRTOSR		__REG32(IO_ADDRESS(_DMA_BASE+0x010))	/*  32bit dma req timeout status reg */
#define DMA_DSESR		__REG32(IO_ADDRESS(_DMA_BASE+0x014))	/*  32bit dma transfer err status reg */
#define DMA_DBOSR		__REG32(IO_ADDRESS(_DMA_BASE+0x018))	/*  32bit dma buffer overflow stat reg */
#define DMA_DBTOCR		__REG32(IO_ADDRESS(_DMA_BASE+0x01C))	/*  32bit dma burst timeout ctrl reg */

/* input for x: 0 or 1*/
#define DMA_2D_SZ_BASE(x)	IO_ADDRESS(_DMA_2D_SZ_BASE++ ((x)<<2)|((x)<<3))
#define DMA_WSR(x)		__REG32_2(IO_ADDRESS(_DMA_2D_SZ_BASE+0x000),((x)<<2)|((x)<<3))	/*  32bit dma W-size reg */
#define DMA_XSR(x)		__REG32_2(IO_ADDRESS(_DMA_2D_SZ_BASE+0x004),((x)<<2)|((x)<<3))	/*  32bit dma X-size reg */
#define DMA_YSR(x)		__REG32_2(IO_ADDRESS(_DMA_2D_SZ_BASE+0x008),((x)<<2)|((x)<<3))	/*  32bit dma Y-size reg */

/* input for x: 0 .. MAX_DMA_CHANNELS-1*/
#define DMA_CH_BASE(x)		IO_ADDRESS(_DMA_CH_BASE +((x)<<6))
#define DMA_SAR(x)		__REG32_2(IO_ADDRESS(_DMA_CH_BASE),(x)<<6)
#define DMA_DAR(x)		__REG32_2(IO_ADDRESS(_DMA_CH_BASE+0x004),(x)<<6)
#define DMA_CNTR(x)		__REG32_2(IO_ADDRESS(_DMA_CH_BASE+0x008),(x)<<6)
#define DMA_CCR(x)		__REG32_2(IO_ADDRESS(_DMA_CH_BASE+0x00C),(x)<<6)	/*  32bit dma ch x control reg */
#define DMA_RSSR(x)		__REG32_2(IO_ADDRESS(_DMA_CH_BASE+0x010),(x)<<6)	/*  32bit dma ch x req source sel reg */
#define DMA_BLR(x)		__REG32_2(IO_ADDRESS(_DMA_CH_BASE+0x014),(x)<<6)	/*  32bit dma ch x burst lenght reg */
#define DMA_RTOR(x)		__REG32_2(IO_ADDRESS(_DMA_CH_BASE+0x018),(x)<<6)	/*  32bit dma ch x req time out reg */
#define DMA_BUCR(x)		__REG32_2(IO_ADDRESS(_DMA_CH_BASE+0x018),(x)<<6)	/*  32bit dma ch x bus utilization reg */
#define DMA_CCNR(x)		__REG32_2(IO_ADDRESS(_DMA_CH_BASE+0x01C),(x)<<6)

#define DMA_TEST_BASE		IO_ADDRESS(_DMA_TEST_BASE)
#define DMA_TCR			__REG32(IO_ADDRESS(_DMA_TEST_BASE+0x000))	/*  32bit dma test control reg */
#define DMA_TFIFOAR		__REG32(IO_ADDRESS(_DMA_TEST_BASE+0x004))	/*  32bit dma test fifo A reg */
#define DMA_TDRR		__REG32(IO_ADDRESS(_DMA_TEST_BASE+0x008))	/*  32bit dma test request reg */
#define DMA_TDIPR		__REG32(IO_ADDRESS(_DMA_TEST_BASE+0x00C))	/*  32bit dma test in progress reg */
#define DMA_TFIFOBR		__REG32(IO_ADDRESS(_DMA_TEST_BASE+0x010))	/*  32bit dma test fifo B reg */

/*
 * WDOG
 */

#define _WDOG_BASE	0x10002000

#define WDOG_BASE	IO_ADDRESS(_WDOG_BASE)
#define WDOG_WCR	__REG16(IO_ADDRESS(_WDOG_BASE+0x00))	/*  16bit watchdog control reg */
#define WDOG_WSR	__REG16(IO_ADDRESS(_WDOG_BASE+0x02))	/*  16bit watchdog service reg */
#define WDOG_WRSR	__REG16(IO_ADDRESS(_WDOG_BASE+0x04))	/*  16bit watchdog reset status reg */
#define WDOG_WPR	__REG16(IO_ADDRESS(_WDOG_BASE+0x06))	/*  16bit watchdog protect reg */

/*
 * GPT
 */

#define _GPT_BASE		0x10003000

#define GPT1 0
#define GPT2 1
#define GPT3 2

#define GPT_BASE(x)		IO_ADDRESS(_GPT_BASE+ ((x)<<12))
#define GPT_TCTL(x)		__REG32_2(IO_ADDRESS(_GPT_BASE+0x00),(x)<<12)	/*  32bit timer  control reg */
#define GPT_TPRER(x)		__REG32_2(IO_ADDRESS(_GPT_BASE+0x04),(x)<<12)	/*  32bit timer  prescaler reg */
#define GPT_TCMP(x)		__REG32_2(IO_ADDRESS(_GPT_BASE+0x08),(x)<<12)	/*  32bit timer  compare reg */
#define GPT_TCR(x)		__REG32_2(IO_ADDRESS(_GPT_BASE+0x0C),(x)<<12)	/*  32bit timer  capture reg */
#define GPT_TCN(x)		__REG32_2(IO_ADDRESS(_GPT_BASE+0x10),(x)<<12)	/*  32bit timer  counter reg */
#define GPT_TSTAT(x)		__REG32_2(IO_ADDRESS(_GPT_BASE+0x14),(x)<<12)	/*  32bit timer  status reg */

/*
 * PWM
 */

#define _PWM_BASE	0x10006000

#define PWM_BASE	IO_ADDRESS(_PWM_BASE)
#define PWM_PWMC	__REG32(IO_ADDRESS(_PWM_BASE+0x00))	/*  32bit pwm control reg */
#define PWM_PWMS	__REG32(IO_ADDRESS(_PWM_BASE+0x04))	/*  32bit pwm sample reg */
#define PWM_PWMP	__REG32(IO_ADDRESS(_PWM_BASE+0x08))	/*  32bit pwm period reg */
#define PWM_PWMCNT	__REG32(IO_ADDRESS(_PWM_BASE+0x0C))	/*  32bit pwm counter reg */
#define PWM_PWMTEST1	__REG32(IO_ADDRESS(_PWM_BASE+0x10))	/*  32bit pwm test reg */
#define PWMC                     0x00
#define PWMS                     0x04
#define PWMP                     0x08
#define PWMCNT                   0x0C
#define PWMTST                   0x10

/*
 * RTC
 */

#define _RTC_BASE		0x10007000
#define RTC_BASE		IO_ADDRESS(_RTC_BASE)
#define RTC_HOURMIN		__REG32(IO_ADDRESS(_RTC_BASE+0x00))	/*  32bit rtc hour/min counter reg */
#define RTC_SECOND		__REG32(IO_ADDRESS(_RTC_BASE+0x04))	/*  32bit rtc seconds counter reg */
#define RTC_ALRM_HM		__REG32(IO_ADDRESS(_RTC_BASE+0x08))	/*  32bit rtc alarm hour/min reg */
#define RTC_ALRM_SEC		__REG32(IO_ADDRESS(_RTC_BASE+0x0C))	/*  32bit rtc alarm seconds reg */
#define RTC_RTCCTL		__REG32(IO_ADDRESS(_RTC_BASE+0x10))	/*  32bit rtc control reg */
#define RTC_RTCISR		__REG32(IO_ADDRESS(_RTC_BASE+0x14))	/*  32bit rtc interrupt status reg */
#define RTC_RTCIENR		__REG32(IO_ADDRESS(_RTC_BASE+0x18))	/*  32bit rtc interrupt enable reg */
#define RTC_STPWCH		__REG32(IO_ADDRESS(_RTC_BASE+0x1C))	/*  32bit rtc stopwatch min reg */
#define RTC_DAYR		__REG32(IO_ADDRESS(_RTC_BASE+0x20))	/*  32bit rtc days counter reg */
#define RTC_DAYALARM		__REG32(IO_ADDRESS(_RTC_BASE+0x24))	/*  32bit rtc day alarm reg */
#define RTC_TEST1		__REG32(IO_ADDRESS(_RTC_BASE+0x28))	/*  32bit rtc test reg 1 */
#define RTC_TEST2		__REG32(IO_ADDRESS(_RTC_BASE+0x2C))	/*  32bit rtc test reg 2 */
#define RTC_TEST3		__REG32(IO_ADDRESS(_RTC_BASE+0x30))	/*  32bit rtc test reg 3 */

/*
 * KPP
 */

#define _KPP_BASE		0x10008000
#define KPP_BASE		IO_ADDRESS(_KPP_BASE)
#define KPP_KPCR		__REG16(IO_ADDRESS(_KPP_BASE+0x00))	/*  16bit kpp keypad control reg */
#define KPP_KPSR		__REG16(IO_ADDRESS(_KPP_BASE+0x02))	/*  16bit kpp keypad status reg */
#define KPP_KDDR		__REG16(IO_ADDRESS(_KPP_BASE+0x04))	/*  16bit kpp keypad data directon reg */
#define KPP_KPDR		__REG16(IO_ADDRESS(_KPP_BASE+0x06))	/*  16bit kpp keypad data reg */

/*
 * OWIRE
 */

#define _OWIRE_BASE		0x10009000
#define OWIRE_BASE		IO_ADDRESS(_OWIRE_BASE)
#define OWIRE_CTRL		__REG16(IO_ADDRESS(_OWIRE_BASE+0x00))	/*  16bit owire control reg */
#define OWIRE_TIME_DIV		__REG16(IO_ADDRESS(_OWIRE_BASE+0x02))	/*  16bit owire time divider reg */
#define OWIRE_RESET		__REG16(IO_ADDRESS(_OWIRE_BASE+0x04))	/*  16bit owire reset reg */

/*
 * UART 1 - 4
 */

#define _UART_BASE		0x1000A000

/* input for x: 0 .. 3*/
#define UART_BASE(x)		IO_ADDRESS(_UART_BASE+((x)<<12))
#define UART_URXD(x)		__REG32_2(IO_ADDRESS(_UART_BASE+0x00),(x)<<12)	/*  32bit uart x receiver reg */
#define UART_UTXD(x)		__REG32_2(IO_ADDRESS(_UART_BASE+0x40),(x)<<12)	/*  32bit uart x transmitter reg */
#define UART_UCR1(x)		__REG32_2(IO_ADDRESS(_UART_BASE+0x80),(x)<<12)	/*  32bit uart x control 1 reg */
#define UART_UCR2(x)		__REG32_2(IO_ADDRESS(_UART_BASE+0x84),(x)<<12)	/*  32bit uart x control 2 reg */
#define UART_UCR3(x)		__REG32_2(IO_ADDRESS(_UART_BASE+0x88),(x)<<12)	/*  32bit uart x control 3 reg */
#define UART_UCR4(x)		__REG32_2(IO_ADDRESS(_UART_BASE+0x8C),(x)<<12)	/*  32bit uart x control 4 reg */
#define UART_UFCR(x)		__REG32_2(IO_ADDRESS(_UART_BASE+0x90),(x)<<12)	/*  32bit uart x fifo control reg */
#define UART_USR1(x)		__REG32_2(IO_ADDRESS(_UART_BASE+0x94),(x)<<12)	/*  32bit uart x status 1 reg */
#define UART_USR2(x)		__REG32_2(IO_ADDRESS(_UART_BASE+0x98),(x)<<12)	/*  32bit uart x status 2 reg */
#define UART_UESC(x)		__REG32_2(IO_ADDRESS(_UART_BASE+0x9C),(x)<<12)	/*  32bit uart x escape char reg */
#define UART_UTIM(x)		__REG32_2(IO_ADDRESS(_UART_BASE+0xA0),(x)<<12)	/*  32bit uart x escape timer reg */
#define UART_UBIR(x)		__REG32_2(IO_ADDRESS(_UART_BASE+0xA4),(x)<<12)	/*  32bit uart x BRM incremental reg */
#define UART_UBMR(x)		__REG32_2(IO_ADDRESS(_UART_BASE+0xA8),(x)<<12)	/*  32bit uart x BRM modulator reg */
#define UART_UBRC(x)		__REG32_2(IO_ADDRESS(_UART_BASE+0xAC),(x)<<12)	/*  32bit uart x baud rate count reg */
#define UART_ONEMS(x)		__REG32_2(IO_ADDRESS(_UART_BASE+0xB0),(x)<<12)	/*  32bit uart x one ms reg */
#define UART_UTS(x)		__REG32_2(IO_ADDRESS(_UART_BASE+0xB4),(x)<<12)	/*  32bit uart x test reg */

#define   URXD_CHARRDY             (1 << 15)
#define   URXD_ERR                 (1 << 14)
#define   URXD_OVRRUN              (1 << 13)
#define   URXD_FRMERR              (1 << 12)
#define   URXD_BRK                 (1 << 11)
#define   URXD_PRERR               (1 << 10)
#define   URXD_RXDATA_BIT           0
#define   URXD_RXDATA_MASK      (0xFF << URXD_RXDATA_BIT)

#define   UTXD_TXDATA_BIT           0
#define   UTXD_TXDATA_MASK      (0xFF << UTXD_TXDATA_BIT)

#define   UCR1_ADEN                (1 << 15)
#define   UCR1_ADBR                (1 << 14)
#define   UCR1_TRDYEN              (1 << 13)
#define   UCR1_IDEN                (1 << 12)
#define   UCR1_ICD_BIT             10
#define   UCR1_ICD_MASK          (0x3 << UCR1_ICD_BIT)
#define   UCR1_ICD_4FRAME          (0 << UCR1_ICD_BIT)
#define   UCR1_ICD_8FRAME          (1 << UCR1_ICD_BIT)
#define   UCR1_ICD_16FRAME         (2 << UCR1_ICD_BIT)
#define   UCR1_ICD_32FRAME         (3 << UCR1_ICD_BIT)
#define   UCR1_RRDYEN              (1 <<  9)
#define   UCR1_RDMAEN              (1 <<  8)
#define   UCR1_IREN                (1 <<  7)
#define   UCR1_TXEMPTYEN           (1 <<  6)
#define   UCR1_RTSDEN              (1 <<  5)
#define   UCR1_SNDBRK              (1 <<  4)
#define   UCR1_TDMAEN              (1 <<  3)
#define   UCR1_UARTCLKEN           (1 <<  2)
#define   UCR1_DOZE                (1 <<  1)
#define   UCR1_UARTEN              (1 <<  0)

#define   UCR2_ESCI                (1 << 15)
#define   UCR2_IRTS                (1 << 14)
#define   UCR2_CTSC                (1 << 13)
#define   UCR2_CTS                 (1 << 12)
#define   UCR2_ESCEN               (1 << 11)
#define   UCR2_RTEC_BIT             9
#define   UCR2_RTEC_MASK         (0x3 << UCR2_RTEC_BIT)
#define   UCR2_PREN                (1 << 8)
#define     UCR2_NON_PARITY        (0 << 8)
#define   UCR2_PROE                (1 << 7)
#define     UCR2_PARITY_ODD        (1 << 7)
#define     UCR2_PARITY_EVEN       (0 << 7)
#define   UCR2_STPB                (1 << 6)
#define     UCR2_STOPBIT_1         (0 << 6)
#define     UCR2_STOPBIT_2         (1 << 6)
#define   UCR2_WS                  (1 << 5)
#define     UCR2_7BIT              (0 << 5)
#define     UCR2_8BIT              (1 << 5)
#define   UCR2_RTSEN               (1 << 4)
#define   UCR2_TXEN                (1 << 2)
#define   UCR2_RXEN                (1 << 1)
#define   UCR2_SRST                (1 << 0)

#define   UCR3_DPEC_BIT            14
#define   UCR3_DPEC_MASK         (0x3 << UCR3_DPEC_BIT)
#define   UCR3_DTREN               (1 << 13)	/* only on UART2 */
#define   UCR3_PARERREN            (1 << 12)
#define   UCR3_FRAERREN            (1 << 11)
#define   UCR3_DSR                 (1 << 10)	/* only on UART2 */
#define   UCR3_DCD                 (1 <<  9)	/* only on UART2 */
#define   UCR3_RI                  (1 <<  8)	/* only on UART2 */
#define   UCR3_TIMEOUTEN           (1 <<  7)
#define   UCR3_RXDSEN              (1 <<  6)
#define   UCR3_AIRINTEN            (1 <<  5)
#define   UCR3_AWAKEN              (1 <<  4)
#define   UCR3_REF25               (1 <<  3)
#define   UCR3_REF30               (1 <<  2)
#define   UCR3_INVT                (1 <<  1)
#define   UCR3_BPEN                (1 <<  0)

#define   UCR4_CTSTL_BIT           10
#define   UCR4_CTSTL_MASK       (0x3F << UCR4_CTSTL_BIT)
#define   UCR4_INVR                (1 << 9)
#define   UCR4_ENIRI               (1 << 8)
#define   UCR4_WKEN                (1 << 7)
#define   UCR4_REF16               (1 << 6)
#define   UCR4_IRSC                (1 << 5)
#define   UCR4_TCEN                (1 << 3)
#define   UCR4_BKEN                (1 << 2)
#define   UCR4_OREN                (1 << 1)
#define   UCR4_DREN                (1 << 0)

#define   UFCR_TXTL_BIT            10
#define   UFCR_TXTL_MASK        (0x3F << UFCR_TXTL_BIT)
#define   UFCR_RFDIV_BIT            7
#define   UFCR_RFDIV_MASK        (0x7 << UFCR_RFDIV_BIT)
#define   UFCR_RFDIV_7           (0x6 << UFCR_RFDIV_BIT)
#define   UFCR_RFDIV_6           (0x0 << UFCR_RFDIV_BIT)
#define   UFCR_RFDIV_5           (0x1 << UFCR_RFDIV_BIT)
#define   UFCR_RFDIV_4           (0x2 << UFCR_RFDIV_BIT)
#define   UFCR_RFDIV_3           (0x3 << UFCR_RFDIV_BIT)
#define   UFCR_RFDIV_2           (0x4 << UFCR_RFDIV_BIT)
#define   UFCR_RFDIV_1           (0x5 << UFCR_RFDIV_BIT)
#define   UFCR_RXTL_BIT             0
#define   UFCR_RXTL_MASK        (0x3F << UFCR_RXTL_BIT)

#define   USR1_PARITY_ERR          (1 << 15)
#define   USR1_RTSS                (1 << 14)
#define   USR1_TRDY                (1 << 13)
#define   USR1_RTSD                (1 << 12)
#define   USR1_ESCF                (1 << 11)
#define   USR1_FRAMERR             (1 << 10)
#define   USR1_RRDY                (1 << 9)
#define   USR1_TIMEOUT             (1 << 7)
#define   USR1_RXDS                (1 << 6)
#define   USR1_AIRINT              (1 << 5)
#define   USR1_AWAKE               (1 << 4)

#define   USR2_ADET                (1 << 15)
#define   USR2_TXFE                (1 << 14)
#define   USR2_DTRF                (1 << 13)
#define   USR2_IDLE                (1 << 12)
#define   USR2_IRINT               (1 << 8)
#define   USR2_WAKE                (1 << 7)
#define   USR2_RTSF                (1 << 4)
#define   USR2_TXDC                (1 << 3)
#define   USR2_BRCD                (1 << 2)
#define   USR2_ORE                 (1 << 1)
#define   USR2_RDR                 (1 << 0)

/*
 * ALTERNATIVE UART DEFFINITIONS
 */

#define UART1_BASE         IO_ADDRESS(0x1000A000)
#define UART2_BASE         IO_ADDRESS(0x1000B000)
#define UART3_BASE         IO_ADDRESS(0x1000C000)
#define UART4_BASE         IO_ADDRESS(0x1000D000)
#define UART_RXDATA              0x00
#define   UART_CHARRDY             (1 << 15)
#define   UART_ERR                 (1 << 14)
#define   UART_OVRRUN              (1 << 13)
#define   UART_FRMERR              (1 << 12)
#define   UART_BRK                 (1 << 11)
#define   UART_PRERR               (1 << 10)
#define   UART_RXDATA_BIT           0
#define   UART_RXDATA_MASK      (0xFF << UART_RXDATA_BIT)
#define UART_TXDATA              0x40
#define   UART_TXDATA_BIT           0
#define   UART_TXDATA_MASK      (0xFF << UART_TXDATA_BIT)
#define UART_CR1                 0x80
#define   UART_ADEN                (1 << 15)
#define   UART_ADBR                (1 << 14)
#define   UART_TRDYEN              (1 << 13)
#define   UART_IDEN                (1 << 12)
#define   UART_ICD_BIT             10
#define   UART_ICD_MASK          (0x3 << UART_ICD_BIT)
#define   UART_ICD_4FRAME          (0 << UART_ICD_BIT)
#define   UART_ICD_8FRAME          (1 << UART_ICD_BIT)
#define   UART_ICD_16FRAME         (2 << UART_ICD_BIT)
#define   UART_ICD_32FRAME         (3 << UART_ICD_BIT)
#define   UART_RRDYEN              (1 <<  9)
#define   UART_RDMAEN              (1 <<  8)
#define   UART_IREN                (1 <<  7)
#define   UART_TXEMPTYEN           (1 <<  6)
#define   UART_RTSDEN              (1 <<  5)
#define   UART_SNDBRK              (1 <<  4)
#define   UART_TDMAEN              (1 <<  3)
#define   UART_UARTCLKEN           (1 <<  2)
#define   UART_DOZE                (1 <<  1)
#define   UART_UARTEN              (1 <<  0)
#define UART_CR2                 0x84
#define   UART_ESCI                (1 << 15)
#define   UART_IRTS                (1 << 14)
#define   UART_CTSC                (1 << 13)
#define   UART_CTS                 (1 << 12)
#define   UART_ESCEN               (1 << 11)
#define   UART_RTEC_BIT             9
#define   UART_RTEC_MASK         (0x3 << UART_RTEC_BIT)
#define   UART_PREN                (1 << 8)
#define     UART_NON_PARITY        (0 << 8)
#define   UART_PROE                (1 << 7)
#define     UART_PARITY_ODD        (1 << 7)
#define     UART_PARITY_EVEN       (0 << 7)
#define   UART_STPB                (1 << 6)
#define     UART_STOPBIT_1         (0 << 6)
#define     UART_STOPBIT_2         (1 << 6)
#define   UART_WS                  (1 << 5)
#define     UART_7BIT              (0 << 5)
#define     UART_8BIT              (1 << 5)
#define   UART_RTSEN               (1 << 4)
#define   UART_TXEN                (1 << 2)
#define   UART_RXEN                (1 << 1)
#define   UART_SRST                (1 << 0)
#define UART_CR3                 0x88
#define   UART_DPEC_BIT            14	/* not used, should be w/r as 0 */
#define   UART_DPEC_MASK         (0x3 << UART_DPEC_BIT)
#define   UART_DTREN               (1 << 13)	/* not used, should be w/r as 0 */
#define   UART_PARERREN            (1 << 12)
#define   UART_FRAERREN            (1 << 11)
#define   UART_DSR                 (1 << 10)	/* not used, should be w/r as 0 */
#define   UART_DCD                 (1 <<  9)	/* not used, should be w/r as 0 */
#define   UART_RI                  (1 <<  8)	/* not used, should be w/r as 0 */
#define   UART_ADNIMP	           (1 <<  7)
#define   UART_RXDSEN              (1 <<  6)
#define   UART_AIRINTEN            (1 <<  5)
#define   UART_AWAKEN              (1 <<  4)
#define   UART_RXDMUXSEL           (1 <<  2)
#define   UART_INVT                (1 <<  1)
#define   UART_ACIEN               (1 <<  0)
#define UART_CR4                 0x8C
#define   UART_CTSTL_BIT           10
#define   UART_CTSTL_MASK       (0x3F << UART_CTSTL_BIT)
#define   UART_INVR                (1 << 9)
#define   UART_ENIRI               (1 << 8)
#define   UART_WKEN                (1 << 7)
#define   UART_REF16               (1 << 6)
#define   UART_IRSC                (1 << 5)
#define   UART_TCEN                (1 << 3)
#define   UART_BKEN                (1 << 2)
#define   UART_OREN                (1 << 1)
#define   UART_DREN                (1 << 0)
#define UART_FCR                0x90
#define   UART_TXTL_BIT            10
#define   UART_TXTL_MASK        (0x3F << UART_TXTL_BIT)
#define   UART_RFDIV_BIT            7
#define   UART_RFDIV_MASK        (0x7 << UART_RFDIV_BIT)
#define   UART_RFDIV_7           (0x6 << UART_RFDIV_BIT)
#define   UART_RFDIV_6           (0x0 << UART_RFDIV_BIT)
#define   UART_RFDIV_5           (0x1 << UART_RFDIV_BIT)
#define   UART_RFDIV_4           (0x2 << UART_RFDIV_BIT)
#define   UART_RFDIV_3           (0x3 << UART_RFDIV_BIT)
#define   UART_RFDIV_2           (0x4 << UART_RFDIV_BIT)
#define   UART_RFDIV_1           (0x5 << UART_RFDIV_BIT)
#define   UART_RXTL_BIT             0
#define   UART_RXTL_MASK        (0x3F << UART_RXTL_BIT)
#define UART_SR1                 0x94
#define   UART_PARITY_ERR          (1 << 15)
#define   UART_RTSS                (1 << 14)
#define   UART_TRDY                (1 << 13)
#define   UART_RTSD                (1 << 12)
#define   UART_ESCF                (1 << 11)
#define   UART_FRAMERR             (1 << 10)
#define   UART_RRDY                (1 << 9)
#define   UART_TIMEOUT             (1 << 7)
#define   UART_RXDS                (1 << 6)
#define   UART_AIRINT              (1 << 5)
#define   UART_AWAKE               (1 << 4)
#define UART_SR2                 0x98
#define   UART_ADET                (1 << 15)
#define   UART_TXFE                (1 << 14)
#define   UART_DTRF                (1 << 13)
#define   UART_IDLE                (1 << 12)
#define   UART_IRINT               (1 << 8)
#define   UART_WAKE                (1 << 7)
#define   UART_RTSF                (1 << 4)
#define   UART_TXDC                (1 << 3)
#define   UART_BRCD                (1 << 2)
#define   UART_ORE                 (1 << 1)
#define   UART_RDR                 (1 << 0)
#define UART_ESC                 0x9C
#define UART_TIM                 0xA0
#define UART_BIR                 0xA4
#define UART_BMR                 0xA8
#define UART_BRC                 0xAC
#define UART_BIPR1               0xB0
#define UART_BMPR1               0xB4
#define UART_BIPR2               0xB8
#define UART_BMPR2               0xBC
#define UART_BIPR3               0xC0
#define UART_BMPR3               0xC4
#define UART_BIPR4               0xC8
#define UART_BMPR4               0xCC
#define UART_TS                  0xD0

/*
 * CSPI 1 & 2
 */

#define _CSPI_BASE			0x1000E000

/* input for x: 1 or 2*/
#define CSPI_BASE(x)			IO_ADDRESS(_CSPI_BASE + (((x) - 1) << 12))
#define CSPI_RXDATAREG(x)		__REG32_2(IO_ADDRESS(_CSPI_BASE+0x00), ((x) - 1) << 12)	/*  32bit cspi receive data reg */
#define CSPI_TXDATAREG(x)		__REG32_2(IO_ADDRESS(_CSPI_BASE+0x04), ((x) - 1) << 12)	/*  32bit cspi transmit data reg */
#define CSPI_CONTROLREG(x)		__REG32_2(IO_ADDRESS(_CSPI_BASE+0x08), ((x) - 1) << 12)	/*  32bit cspi control reg */
#define CSPI_INTREG(x)			__REG32_2(IO_ADDRESS(_CSPI_BASE+0x0C), ((x) - 1) << 12)	/*  32bit cspi interrupt stat/ctr reg */
#define CSPI_TESTREG(x)			__REG32_2(IO_ADDRESS(_CSPI_BASE+0x10), ((x) - 1) << 12)	/*  32bit cspi test reg */
#define CSPI_PERIODREG(x)		__REG32_2(IO_ADDRESS(_CSPI_BASE+0x14), ((x) - 1) << 12)	/*  32bit cspi sample period ctrl reg */
#define CSPI_DMAREG(x)			__REG32_2(IO_ADDRESS(_CSPI_BASE+0x18), ((x) - 1) << 12)	/*  32bit cspi dma ctrl reg */
#define CSPI_RESETREG(x)		__REG32_2(IO_ADDRESS(_CSPI_BASE+0x1C), ((x) - 1) << 12)	/*  32bit cspi soft reset reg */

/*
 * SSI 1 & 2
 */

#define _SSI_BASE		0x10010000

/* input for x: 1 or 2*/
#define SSI_BASE(x)		IO_ADDRESS(_SSI_BASE + (((x) - 1) << 12))
#define SSI_STX0(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x00), ((x) - 1) << 12)	/*  32bit ssi tx reg 0 */
#define SSI_STX1(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x04), ((x) - 1) << 12)	/*  32bit ssi tx reg 1 */
#define SSI_SRX0(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x08), ((x) - 1) << 12)	/*  32bit ssi rx reg 0 */
#define SSI_SRX1(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x0C), ((x) - 1) << 12)	/*  32bit ssi rx reg 1 */
#define SSI_SCR(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x10), ((x) - 1) << 12)	/*  32bit ssi control reg */
#define SSI_SISR(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x14), ((x) - 1) << 12)	/*  32bit ssi intr status reg */
#define SSI_SIER(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x18), ((x) - 1) << 12)	/*  32bit ssi intr enable reg */
#define SSI_STCR(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x1C), ((x) - 1) << 12)	/*  32bit ssi tx config reg */
#define SSI_SRCR(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x20), ((x) - 1) << 12)	/*  32bit ssi rx config reg */
#define SSI_STCCR(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x24), ((x) - 1) << 12)	/*  32bit ssi tx clock control reg */
#define SSI_SRCCR(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x28), ((x) - 1) << 12)	/*  32bit ssi rx clock control reg */
#define SSI_SFCSR(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x2C), ((x) - 1) << 12)	/*  32bit ssi fifo control/status reg */
#define SSI_STR(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x30), ((x) - 1) << 12)	/*  32bit ssi test reg */
#define SSI_SOR(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x34), ((x) - 1) << 12)	/*  32bit ssi option reg */
#define SSI_SACNT(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x38), ((x) - 1) << 12)	/*  32bit ssi ac97 control reg */
#define SSI_SACADD(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x3C), ((x) - 1) << 12)	/*  32bit ssi ac97 cmd addr reg */
#define SSI_SACDAT(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x40), ((x) - 1) << 12)	/*  32bit ssi ac97 cmd data reg */
#define SSI_SATAG(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x44), ((x) - 1) << 12)	/*  32bit ssi ac97 tag reg */
#define SSI_STMSK(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x48), ((x) - 1) << 12)	/*  32bit ssi tx time slot mask reg */
#define SSI_SRMSK(x)		__REG32_2(IO_ADDRESS(_SSI_BASE+0x4C), ((x) - 1) << 12)	/*  32bit ssi rx time slot mask reg */

#define SSI_IO_SIZE			0x50

/* for use with dma */

#define _SSI_STX0(x)		(_SSI_BASE+0x00 + (((x) - 1) << 12))	/*  32bit ssi tx reg 0 */
#define _SSI_STX1(x)		(_SSI_BASE+0x04 + (((x) - 1) << 12))	/*  32bit ssi tx reg 1 */
#define _SSI_SRX0(x)		(_SSI_BASE+0x08 + (((x) - 1) << 12))	/*  32bit ssi rx reg 0 */
#define _SSI_SRX1(x)		(_SSI_BASE+0x0C + (((x) - 1) << 12))	/*  32bit ssi rx reg 1 */

/*
 * I2C
 */

#define _I2C_BASE	0x10012000

#define I2C_BASE	IO_ADDRESS(_I2C_BASE)
#define I2C_IADR	__REG32(IO_ADDRESS(_I2C_BASE+0x00))	/*  16bit i2c address reg */
#define I2C_IFDR	__REG32(IO_ADDRESS(_I2C_BASE+0x04))	/*  16bit i2c frequency divider reg */
#define I2C_I2CR	__REG32(IO_ADDRESS(_I2C_BASE+0x08))	/*  16bit i2c control reg */
#define I2C_I2SR	__REG32(IO_ADDRESS(_I2C_BASE+0x0C))	/*  16bit i2c status reg */
#define I2C_I2DR	__REG32(IO_ADDRESS(_I2C_BASE+0x10))	/*  16bit i2c data i/o reg */

#define I2C_IO_SIZE				0x14

/*control register bit definitions*/
#define I2CR_ENABLE (1<<7)
#define I2CR_INTEN (1<<6)
#define I2CR_MASTER (1<<5)
#define I2CR_TRANSMIT (1<<4)
#define I2CR_NOACK (1<<3)
#define I2CR_REPSTART (1<<2)

/*status register bit definitions*/
#define I2SR_DATAREADY (1<<7)
#define I2SR_ADDRASSLA (1<<6)
#define I2SR_BUSBUSY (1<<5)
#define I2SR_LOSTARB (1<<4)
#define I2SR_SLATRANS (1<<2)
#define I2SR_INTPEND (1<<1)
#define I2SR_ACKRCVD 1

/*
 * SDHC
 */

#define _SDHC_BASE			0x10013000

/* input for x: */
#define SDHC_1   0
#define SDHC_2   (1 << 12)

#define SDHC_BASE(x)			IO_ADDRESS(_SDHC_BASE + (x))
#define SDHC_STR_STP_CLK(x)		__REG32_2(IO_ADDRESS(_SDHC_BASE+0x00), (x))	/*  32bit sdhc control reg */
#define SDHC_STATUS(x)			__REG32_2(IO_ADDRESS(_SDHC_BASE+0x04), (x))	/*  32bit sdhc status reg */
#define SDHC_CLK_RATE(x)		__REG32_2(IO_ADDRESS(_SDHC_BASE+0x08), (x))	/*  32bit sdhc clock rate reg */
#define SDHC_CMD_DAT_CONT(x)		__REG32_2(IO_ADDRESS(_SDHC_BASE+0x0C), (x))	/*  32bit sdhc cmd/data control reg */
#define SDHC_RESPONSE_TO(x)		__REG32_2(IO_ADDRESS(_SDHC_BASE+0x10), (x))	/*  32bit sdhc response time out reg */
#define SDHC_READ_TO(x)			__REG32_2(IO_ADDRESS(_SDHC_BASE+0x14), (x))	/*  32bit sdhc read time out reg */
#define SDHC_BLK_LEN(x)			__REG32_2(IO_ADDRESS(_SDHC_BASE+0x18), (x))	/*  32bit sdhc block length reg */
#define SDHC_NOB(x)			__REG32_2(IO_ADDRESS(_SDHC_BASE+0x1C), (x))	/*  32bit sdhc number of blocks reg */
#define SDHC_REV_NO(x)			__REG32_2(IO_ADDRESS(_SDHC_BASE+0x20), (x))	/*  32bit sdhc revision number reg */
#define SDHC_INT_MASK(x)		__REG32_2(IO_ADDRESS(_SDHC_BASE+0x24), (x))	/*  32bit sdhc interrupt mask reg */
#define SDHC_CMD(x)			__REG32_2(IO_ADDRESS(_SDHC_BASE+0x28), (x))	/*  32bit sdhc command code reg */
#define SDHC_ARGH(x)			__REG32_2(IO_ADDRESS(_SDHC_BASE+0x2C), (x))	/*  32bit sdhc argument high reg */
#define SDHC_ARGL(x)			__REG32_2(IO_ADDRESS(_SDHC_BASE+0x30), (x))	/*  32bit sdhc argument low reg */
#define SDHC_RES_FIFO(x)		__REG32_2(IO_ADDRESS(_SDHC_BASE+0x34), (x))	/*  32bit sdhc response fifo reg */
#define SDHC_BUFFER_ACCESS(x)		__REG32_2(IO_ADDRESS(_SDHC_BASE+0x38), (x))	/*  32bit sdhc buffer access reg */

/*
 * GPIO
 */

#define _GPIO_BASE	0x10015000

/* input for x: 0 .. 5 */
#define GPIO_BASE(x)	IO_ADDRESS(_GPIO_BASE+((x) << 8))
#define GPIO_DDIR(x)	__REG32_2(IO_ADDRESS(_GPIO_BASE+0x00), (x) << 8)	/*  32bit gpio pta data direction reg */
#define GPIO_OCR1(x)	__REG32_2(IO_ADDRESS(_GPIO_BASE+0x04), (x) << 8)	/*  32bit gpio pta output config 1 reg */
#define GPIO_OCR2(x)	__REG32_2(IO_ADDRESS(_GPIO_BASE+0x08), (x) << 8)	/*  32bit gpio pta output config 2 reg */
#define GPIO_ICONFA1(x)	__REG32_2(IO_ADDRESS(_GPIO_BASE+0x0C), (x) << 8)	/*  32bit gpio pta input config A1 reg */
#define GPIO_ICONFA2(x)	__REG32_2(IO_ADDRESS(_GPIO_BASE+0x10), (x) << 8)	/*  32bit gpio pta input config A2 reg */
#define GPIO_ICONFB1(x)	__REG32_2(IO_ADDRESS(_GPIO_BASE+0x14), (x) << 8)	/*  32bit gpio pta input config B1 reg */
#define GPIO_ICONFB2(x)	__REG32_2(IO_ADDRESS(_GPIO_BASE+0x18), (x) << 8)	/*  32bit gpio pta input config B2 reg */
#define GPIO_DR(x)	__REG32_2(IO_ADDRESS(_GPIO_BASE+0x1C), (x) << 8)	/*  32bit gpio pta data reg */
#define GPIO_GIUS(x)	__REG32_2(IO_ADDRESS(_GPIO_BASE+0x20), (x) << 8)	/*  32bit gpio pta in use reg */
#define GPIO_SSR(x)	__REG32_2(IO_ADDRESS(_GPIO_BASE+0x24), (x) << 8)	/*  32bit gpio pta sample status reg */
#define GPIO_ICR1(x)	__REG32_2(IO_ADDRESS(_GPIO_BASE+0x28), (x) << 8)	/*  32bit gpio pta interrupt ctrl 1 reg */
#define GPIO_ICR2(x)	__REG32_2(IO_ADDRESS(_GPIO_BASE+0x2C), (x) << 8)	/*  32bit gpio pta interrupt ctrl 2 reg */
#define GPIO_IMR(x)	__REG32_2(IO_ADDRESS(_GPIO_BASE+0x30), (x) << 8)	/*  32bit gpio pta interrupt mask reg */
#define GPIO_ISR(x)	__REG32_2(IO_ADDRESS(_GPIO_BASE+0x34), (x) << 8)	/*  32bit gpio pta interrupt status reg */
#define GPIO_GPR(x)	__REG32_2(IO_ADDRESS(_GPIO_BASE+0x38), (x) << 8)	/*  32bit gpio pta general purpose reg */
#define GPIO_SWR(x)	__REG32_2(IO_ADDRESS(_GPIO_BASE+0x3C), (x) << 8)	/*  32bit gpio pta software reset reg */
#define GPIO_PUEN(x)	__REG32_2(IO_ADDRESS(_GPIO_BASE+0x40), (x) << 8)	/*  32bit gpio pta pull up enable reg */

#define _GPIO_REG_BASE	0x10015600
#define GPIO_REG_BASE	IO_ADDRESS(_GPIO_REG_BASE)
#define GPIO_PMASK	__REG32(IO_ADDRESS(_GPIO_REG_BASE+0x00))	/*  32bit gpio interrupt mask reg */

/*
 * AUDMUX
 */

#define _AUDMUX_BASE	0x10016000

#define AUDMUX_BASE	IO_ADDRESS(_AUDMUX_BASE)
#define AUDMUX_HPCR(x)	__REG32_2(IO_ADDRESS(_AUDMUX_BASE), ((x) - 1) << 2)	/*  32bit audmux host config reg 1-3 */
#define AUDMUX_PPCR(x)	__REG32_2(IO_ADDRESS(_AUDMUX_BASE + 0x10), ((x) - 1) << 2)	/*  32bit audmux pripheral config 1-3 */

/*
 * LCDC
 */

#define _LCDC_BASE		0x10021000

#define LCDC_BASE		IO_ADDRESS(_LCDC_BASE)
#define LCDC_LSSAR		__REG32(IO_ADDRESS(_LCDC_BASE+0x00))	/*  32bit lcdc screen start addr reg */
#define LCDC_LSR		__REG32(IO_ADDRESS(_LCDC_BASE+0x04))	/*  32bit lcdc size reg */
#define LCDC_LVPWR		__REG32(IO_ADDRESS(_LCDC_BASE+0x08))	/*  32bit lcdc virtual page width reg */
#define LCDC_LCPR		__REG32(IO_ADDRESS(_LCDC_BASE+0x0C))	/*  32bit lcd cursor position reg */
#define LCDC_LCWHBR		__REG32(IO_ADDRESS(_LCDC_BASE+0x10))	/*  32bit lcd cursor width/heigh/blink */
#define LCDC_LCCMR		__REG32(IO_ADDRESS(_LCDC_BASE+0x14))	/*  32bit lcd color cursor mapping reg */
#define LCDC_LPCR		__REG32(IO_ADDRESS(_LCDC_BASE+0x18))	/*  32bit lcdc panel config reg */
#define LCDC_LHCR		__REG32(IO_ADDRESS(_LCDC_BASE+0x1C))	/*  32bit lcdc horizontal config reg */
#define LCDC_LVCR		__REG32(IO_ADDRESS(_LCDC_BASE+0x20))	/*  32bit lcdc vertical config reg */
#define LCDC_LPOR		__REG32(IO_ADDRESS(_LCDC_BASE+0x24))	/*  32bit lcdc panning offset reg */
#define LCDC_LSCR		__REG32(IO_ADDRESS(_LCDC_BASE+0x28))	/*  32bit lcdc sharp config 1 reg */
#define LCDC_LPCCR		__REG32(IO_ADDRESS(_LCDC_BASE+0x2C))	/*  32bit lcdc pwm contrast ctrl reg */
#define LCDC_LDCR		__REG32(IO_ADDRESS(_LCDC_BASE+0x30))	/*  32bit lcdc dma control reg */
#define LCDC_LRMCR		__REG32(IO_ADDRESS(_LCDC_BASE+0x34))	/*  32bit lcdc refresh mode ctrl reg */
#define LCDC_LICR		__REG32(IO_ADDRESS(_LCDC_BASE+0x38))	/*  32bit lcdc interrupt config reg */
#define LCDC_LIER		__REG32(IO_ADDRESS(_LCDC_BASE+0x3C))	/*  32bit lcdc interrupt enable reg */
#define LCDC_LISR		__REG32(IO_ADDRESS(_LCDC_BASE+0x40))	/*  32bit lcdc interrupt status reg */
#define LCDC_LGWSAR		__REG32(IO_ADDRESS(_LCDC_BASE+0x50))	/*  32bit lcdc graphic win start add */
#define LCDC_LGWSR		__REG32(IO_ADDRESS(_LCDC_BASE+0x54))	/*  32bit lcdc graphic win size reg */
#define LCDC_LGWVPWR		__REG32(IO_ADDRESS(_LCDC_BASE+0x58))	/*  32bit lcdc graphic win virtual pg */
#define LCDC_LGWPOR		__REG32(IO_ADDRESS(_LCDC_BASE+0x5C))	/*  32bit lcdc graphic win pan offset */
#define LCDC_LGWPR		__REG32(IO_ADDRESS(_LCDC_BASE+0x60))	/*  32bit lcdc graphic win positon reg */
#define LCDC_LGWCR		__REG32(IO_ADDRESS(_LCDC_BASE+0x64))	/*  32bit lcdc graphic win control reg */
#define LCDC_LGWDCR		__REG32(IO_ADDRESS(_LCDC_BASE+0x68))	/*  32bit lcdc graphic win DMA control reg */

#define LCDC_BPLUT_BASE(regno)	__REG32_2(IO_ADDRESS(_LCDC_BASE+0x800), (regno)<<2)	/*  Background Plane LUT (800 - BFC) */
#define LCDC_GWLUT_BASE(regno)	__REG32_2(IO_ADDRESS(_LCDC_BASE+0xC00), (regno)<<2)	/*  Background Plane LUT (C00 - FFC) */

/*
 *  SLCDC
 */

#define _SLCDC_BASE		0x10022000

#define SLCDC_BASE		IO_ADDRESS(_SLCDC_BASE)
#define SLCDC_DBADDR		__REG32(IO_ADDRESS(_SLCDC_BASE+0x00))	/*  32bit slcdc data base addr */
#define SLCDC_DBUF_SIZE		__REG32(IO_ADDRESS(_SLCDC_BASE+0x04))	/*  32bit slcdc data buffer size high */
#define SLCDC_CBADDR		__REG32(IO_ADDRESS(_SLCDC_BASE+0x08))	/*  32bit slcdc cmd base addr high */
#define SLCDC_CBUF_SIZE		__REG32(IO_ADDRESS(_SLCDC_BASE+0x0C))	/*  32bit slcdc cmd buffer size high */
#define SLCDC_CBUF_SSIZE	__REG32(IO_ADDRESS(_SLCDC_BASE+0x10))	/*  32bit slcdc cmd string size */
#define SLCDC_FIFO_CONFIG	__REG32(IO_ADDRESS(_SLCDC_BASE+0x14))	/*  32bit slcdc fifo config reg */
#define SLCDC_LCD_CONFIG	__REG32(IO_ADDRESS(_SLCDC_BASE+0x18))	/*  32bit slcdc lcd controller config */
#define SLCDC_LCD_TXCONFIG	__REG32(IO_ADDRESS(_SLCDC_BASE+0x1C))	/*  32bit slcdc lcd transmit config reg */
#define SLCDC_LCD_CTRL_STAT	__REG32(IO_ADDRESS(_SLCDC_BASE+0x20))	/*  32bit slcdc lcd control/status reg */
#define SLCDC_LCD_CLKCONFIG	__REG32(IO_ADDRESS(_SLCDC_BASE+0x24))	/*  32bit slcdc lcd clock config reg */
#define SLCDC_LCD_WR_DATA	__REG32(IO_ADDRESS(_SLCDC_BASE+0x28))	/*  32bit slcdc lcd write data reg */

/*
 * SAHARA
 */

#define _SAHARA_BASE	0x10023000

/* CHA BASE ADDRESSES */
#define _SAHARA_TOP	(_SAHARA_BASE+0x0000)
#define _SAHARA_AESA	(_SAHARA_BASE+0x0100)
#define _SAHARA_DESA	(_SAHARA_BASE+0x0100)
#define _SAHARA_MDHA	(_SAHARA_BASE+0x0200)
#define _SAHARA_RNGA	(_SAHARA_BASE+0x0300)
#define _SAHARA_FIDO	(_SAHARA_BASE+0x0400)
#define _SAHARA_I_FIDO	(_SAHARA_BASE+0x0400)
#define _SAHARA_O_FIDO	(_SAHARA_BASE+0x0500)
#define _SAHARA_PKHA	(_SAHARA_BASE+0x0800)

#define SAHARA_BASE	IO_ADDRESS(_SAHARA_BASE)

/* SAHARA REGISTERS */
#define SAHARA_TOP		IO_ADDRESS(_SAHARA_TOP)
#define SAHARA_VER_ID		__REG32(IO_ADDRESS(_SAHARA_TOP+0x00))
#define SAHARA_DSC_ADR		__REG32(IO_ADDRESS(_SAHARA_TOP+0x04))
#define SAHARA_CONTROL		__REG32(IO_ADDRESS(_SAHARA_TOP+0x08))
#define SAHARA_COMMAND		__REG32(IO_ADDRESS(_SAHARA_TOP+0x0C))
#define SAHARA_STAT		__REG32(IO_ADDRESS(_SAHARA_TOP+0x10))
#define SAHARA_ERR_STAT		__REG32(IO_ADDRESS(_SAHARA_TOP+0x14))
#define SAHARA_FAULT_ADR	__REG32(IO_ADDRESS(_SAHARA_TOP+0x18))
#define SAHARA_C_DSC_ADR	__REG32(IO_ADDRESS(_SAHARA_TOP+0x1C))
#define SAHARA_I_DSC_ADR	__REG32(IO_ADDRESS(_SAHARA_TOP+0x20))
#define SAHARA_BUFF_LVL		__REG32(IO_ADDRESS(_SAHARA_TOP+0x24))
#define SAHARA_DSC_A		__REG32(IO_ADDRESS(_SAHARA_TOP+0x80))
#define SAHARA_DSC_B		__REG32(IO_ADDRESS(_SAHARA_TOP+0x84))
#define SAHARA_DSC_C		__REG32(IO_ADDRESS(_SAHARA_TOP+0x88))
#define SAHARA_DSC_D		__REG32(IO_ADDRESS(_SAHARA_TOP+0x8C))
#define SAHARA_DSC_E		__REG32(IO_ADDRESS(_SAHARA_TOP+0x90))
#define SAHARA_DSC_F		__REG32(IO_ADDRESS(_SAHARA_TOP+0x94))
#define SAHARA_LNK_1_A		__REG32(IO_ADDRESS(_SAHARA_TOP+0xA0))
#define SAHARA_LNK_1_B		__REG32(IO_ADDRESS(_SAHARA_TOP+0xA4))
#define SAHARA_LNK_1_C		__REG32(IO_ADDRESS(_SAHARA_TOP+0xA8))
#define SAHARA_LNK_2_A		__REG32(IO_ADDRESS(_SAHARA_TOP+0xB0))
#define SAHARA_LNK_2_B		__REG32(IO_ADDRESS(_SAHARA_TOP+0xB4))
#define SAHARA_LNK_2_C		__REG32(IO_ADDRESS(_SAHARA_TOP+0xB8))
#define SAHARA_FLOW_CTRL	__REG32(IO_ADDRESS(_SAHARA_TOP+0xC0))

/* COMMON CHA REGISTERS */
#define SAHARA_MODE		0x00
#define SAHARA_KEY_SIZE		0x04
#define SAHARA_DATA_SIZE	0x08
#define SAHARA_STATUS		0x0C
#define SAHARA_ERROR_STATUS	0x10
#define SAHARA_CHA_GO		0x14
#define SAHARA_CONTEXT		0x40
#define SAHARA_KEY		0x80

/* SAHARA_AESA REGISTERS */
#define SAHARA_AESA			IO_ADDRESS(_SAHARA_AESA)
#define SAHARA_AESA_MODE		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_MODE+0x00))
#define SAHARA_AESA_KEY_SIZE		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_KEY_SIZE+0x00))
#define SAHARA_AESA_DATA_SIZE		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_DATA_SIZE+0x00))
#define SAHARA_AESA_STAT		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_STATUS+0x00))
#define SAHARA_AESA_ERR_STAT		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_ERROR_STATUS+0x00))
#define SAHARA_AESA_CHA_GO		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CHA_GO+0x00))
#define SAHARA_AESA_CXT			__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CONTEXT+0x00))
#define SAHARA_AESA_KEY_1		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_KEY+0x00))
#define SAHARA_AESA_KEY_2		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_KEY+0x04))
#define SAHARA_AESA_KEY_3		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_KEY+0x08))
#define SAHARA_AESA_KEY_4		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_KEY+0x0C))
#define SAHARA_AESA_IV_1		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CONTEXT+0x00))
#define SAHARA_AESA_IV_2		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CONTEXT+0x04))
#define SAHARA_AESA_IV_3		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CONTEXT+0x08))
#define SAHARA_AESA_IV_4		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CONTEXT+0x0C))
#define SAHARA_AESA_IV_5		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CONTEXT+0x10))
#define SAHARA_AESA_IV_6		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CONTEXT+0x14))
#define SAHARA_AESA_IV_7		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CONTEXT+0x18))
#define SAHARA_AESA_IV_8		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CONTEXT+0x1C))
#define SAHARA_AESA_IV_9		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CONTEXT+0x20))
#define SAHARA_AESA_IV_10		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CONTEXT+0x24))
#define SAHARA_AESA_IV_11		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CONTEXT+0x28))
#define SAHARA_AESA_IV_12		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CONTEXT+0x2C))
#define SAHARA_AESA_IV_13		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CONTEXT+0x30))
#define SAHARA_AESA_IV_14		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CONTEXT+0x34))
#define SAHARA_AESA_IV_15		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CONTEXT+0x38))
#define SAHARA_AESA_IV_16		__REG32(IO_ADDRESS(_SAHARA_AESA+SAHARA_CONTEXT+0x3C))

/* SAHARA_DESA REGISTERS */
#define SAHARA_DESA			IO_ADDRESS(_SAHARA_DESA)
#define SAHARA_DESA_MODE		__REG32(IO_ADDRESS(_SAHARA_DESA+SAHARA_MODE+0x00))
#define SAHARA_DESA_KEY_SIZE		__REG32(IO_ADDRESS(_SAHARA_DESA+SAHARA_KEY_SIZE+0x00))
#define SAHARA_DESA_DATA_SIZE		__REG32(IO_ADDRESS(_SAHARA_DESA+SAHARA_DATA_SIZE+0x00))
#define SAHARA_DESA_STAT		__REG32(IO_ADDRESS(_SAHARA_DESA+SAHARA_STATUS+0x00))
#define SAHARA_DESA_ERR_STAT		__REG32(IO_ADDRESS(_SAHARA_DESA+SAHARA_ERROR_STATUS+0x00))
#define SAHARA_DESA_CHA_GO		__REG32(IO_ADDRESS(_SAHARA_DESA+SAHARA_CHA_GO+0x00))
#define SAHARA_DESA_KEY			__REG32(IO_ADDRESS(_SAHARA_DESA+SAHARA_KEY+0x00))
#define SAHARA_DESA_CXT			__REG32(IO_ADDRESS(_SAHARA_DESA+SAHARA_CONTEXT+0x00))
#define SAHARA_DESA_KEY_1		__REG32(IO_ADDRESS(_SAHARA_DESA+SAHARA_KEY+0x00))
#define SAHARA_DESA_KEY_2		__REG32(IO_ADDRESS(_SAHARA_DESA+SAHARA_KEY+0x04))
#define SAHARA_DESA_KEY_3		__REG32(IO_ADDRESS(_SAHARA_DESA+SAHARA_KEY+0x08))
#define SAHARA_DESA_KEY_4		__REG32(IO_ADDRESS(_SAHARA_DESA+SAHARA_KEY+0x0C))
#define SAHARA_DESA_KEY_5		__REG32(IO_ADDRESS(_SAHARA_DESA+SAHARA_KEY+0x10))
#define SAHARA_DESA_KEY_6		__REG32(IO_ADDRESS(_SAHARA_DESA+SAHARA_KEY+0x14))
#define SAHARA_DESA_IV_1		__REG32(IO_ADDRESS(_SAHARA_DESA+SAHARA_CONTEXT+0x00))
#define SAHARA_DESA_IV_2		__REG32(IO_ADDRESS(_SAHARA_DESA+SAHARA_CONTEXT+0x04))

/* SAHARA_MDHA REGISTERS */
#define SAHARA_MDHA			IO_ADDRESS(_SAHARA_MDHA)
#define SAHARA_MDHA_MODE		__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_MODE+0x00))
#define SAHARA_MDHA_KEY_SIZE		__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_KEY_SIZE+0x00))
#define SAHARA_MDHA_DATA_SIZE		__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_DATA_SIZE+0x00))
#define SAHARA_MDHA_STAT		__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_STATUS+0x00))
#define SAHARA_MDHA_ERR_STAT		__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_ERROR_STATUS+0x00))
#define SAHARA_MDHA_GO			__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_CHA_GO+0x00))
#define SAHARA_MDHA_KEY			__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_KEY+0x00))
#define SAHARA_MDHA_CXT			__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_CONTEXT+0x00))
#define SAHARA_MDHA_MD_A1		__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_KEY+0x00))
#define SAHARA_MDHA_MD_B1		__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_KEY+0x04))
#define SAHARA_MDHA_MD_C1		__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_KEY+0x08))
#define SAHARA_MDHA_MD_D1		__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_KEY+0x0C))
#define SAHARA_MDHA_MD_E1		__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_KEY+0x10))
#define SAHARA_MDHA_MD_A		__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_CONTEXT+0x00))
#define SAHARA_MDHA_MD_B		__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_CONTEXT+0x04))
#define SAHARA_MDHA_MD_C		__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_CONTEXT+0x08))
#define SAHARA_MDHA_MD_D		__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_CONTEXT+0x0C))
#define SAHARA_MDHA_MD_E		__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_CONTEXT+0x10))
#define SAHARA_MDHA_MD_CNT		__REG32(IO_ADDRESS(_SAHARA_MDHA+SAHARA_CONTEXT+0x14))

/* SAHARA_PKHA REGISTERS */
#define SAHARA_PKHA				IO_ADDRESS(_SAHARA_PKHA)
#define SAHARA_PKHA_PGM_COUNT			__REG32(IO_ADDRESS(_SAHARA_PKHA+SAHARA_MODE+0x000))
#define SAHARA_PKHA_KEY_SIZE			__REG32(IO_ADDRESS(_SAHARA_PKHA+SAHARA_KEY_SIZE+0x000))
#define SAHARA_PKHA_MOD_SIZE			__REG32(IO_ADDRESS(_SAHARA_PKHA+SAHARA_DATA_SIZE+0x000))
#define SAHARA_PKHA_STAT			__REG32(IO_ADDRESS(_SAHARA_PKHA+SAHARA_STATUS+0x000))
#define SAHARA_PKHA_ERR_STAT			__REG32(IO_ADDRESS(_SAHARA_PKHA+SAHARA_ERROR_STATUS+0x000))
#define SAHARA_PKHA_CHA_GO			__REG32(IO_ADDRESS(_SAHARA_PKHA+SAHARA_CHA_GO+0x000))
#define SAHARA_PKHA_A0_BASE			__REG32(IO_ADDRESS(_SAHARA_PKHA+0x400))
#define SAHARA_PKHA_A1_BASE			__REG32(IO_ADDRESS(_SAHARA_PKHA+0x440))
#define SAHARA_PKHA_A2_BASE			__REG32(IO_ADDRESS(_SAHARA_PKHA+0x480))
#define SAHARA_PKHA_A3_BASE			__REG32(IO_ADDRESS(_SAHARA_PKHA+0x4C0))
#define SAHARA_PKHA_B0_BASE			__REG32(IO_ADDRESS(_SAHARA_PKHA+0x500))
#define SAHARA_PKHA_B1_BASE			__REG32(IO_ADDRESS(_SAHARA_PKHA+0x540))
#define SAHARA_PKHA_B2_BASE			__REG32(IO_ADDRESS(_SAHARA_PKHA+0x580))
#define SAHARA_PKHA_B3_BASE			__REG32(IO_ADDRESS(_SAHARA_PKHA+0x5C0))
#define SAHARA_PKHA_N_BASE			__REG32(IO_ADDRESS(_SAHARA_PKHA+0x600))
#define SAHARA_PKHA_EXP_BASE			__REG32(IO_ADDRESS(_SAHARA_PKHA+0x700))

/* SAHARA_RNGA REGISTERS */
#define SAHARA_RNGA				IO_ADDRESS(_SAHARA_RNGA)
#define SAHARA_RNGA_MODE			__REG32(IO_ADDRESS(_SAHARA_RNGA+SAHARA_MODE+0x00))
#define SAHARA_RNGA_DATA_SIZE			__REG32(IO_ADDRESS(_SAHARA_RNGA+SAHARA_DATA_SIZE+0x00))
#define SAHARA_RNGA_STAT			__REG32(IO_ADDRESS(_SAHARA_RNGA+SAHARA_STATUS+0x00))
#define SAHARA_RNGA_ERR_STAT			__REG32(IO_ADDRESS(_SAHARA_RNGA+SAHARA_ERROR_STATUS+0x00))
#define SAHARA_RNGA_CHA_GO			__REG32(IO_ADDRESS(_SAHARA_RNGA+SAHARA_CHA_GO+0x00))

#define SAHARA_I_FIDO	IO_ADDRESS(_SAHARA_I_FIDO)

#define SAHARA_O_FIDO	IO_ADDRESS(_SAHARA_O_FIDO)

#define SAHARA_FIDO	IO_ADDRESS(_SAHARA_FIDO)

/*
 * USB OTG
 */

#define _OTG_BASE	0x10024000

#define _OTG_CORE_BASE	(_OTG_BASE+0x000)	/*  base location for core */
#define _OTG_FUNC_BASE	(_OTG_BASE+0x040)	/*  base location for function */
#define _OTG_HOST_BASE	(_OTG_BASE+0x080)	/*  base location for host */
#define _OTG_I2C_BASE	(_OTG_BASE+0x100)	/*  base location for I2C */
#define _OTG_DMA_BASE	(_OTG_BASE+0x800)	/*  base location for dma */

#define _OTG_ETD_BASE	(_OTG_BASE+0x200)	/*  base location for etd memory */
#define _OTG_EP_BASE	(_OTG_BASE+0x400)	/*  base location for ep memory */
#define _OTG_SYS_BASE	(_OTG_BASE+0x600)	/*  base location for system */
#define _OTG_DATA_BASE	(_OTG_BASE+0x1000)	/*  base location for data memory */

#define OTG_BASE	IO_ADDRESS(_OTG_BASE)
#define OTG_IO_SIZE	0x2000

#define OTG_ETD_BASE	IO_ADDRESS(_OTG_ETD_BASE)	/*  base location for etd memory */
#define OTG_EP_BASE	IO_ADDRESS(_OTG_EP_BASE)	/*  base location for ep memory */
#define OTG_SYS_BASE	IO_ADDRESS(_OTG_SYS_BASE)	/*  base location for system */
#define OTG_DATA_BASE	IO_ADDRESS(_OTG_DATA_BASE)	/*  base location for data memory */

#define OTG_SYS_CTRL	__REG32(IO_ADDRESS(_OTG_SYS_BASE+0x000))	/*  base location for system */

#define OTG_CORE_BASE			IO_ADDRESS(OTG_CORE_BASE)
#define OTG_CORE_HWMODE			__REG32(IO_ADDRESS(_OTG_CORE_BASE+0x00))	/*  32bit core hardware mode reg */
#define OTG_CORE_CINT_STAT		__REG32(IO_ADDRESS(_OTG_CORE_BASE+0x04))	/*  32bit core int status reg */
#define OTG_CORE_CINT_STEN		__REG32(IO_ADDRESS(_OTG_CORE_BASE+0x08))	/*  32bit core int enable reg */
#define OTG_CORE_CLK_CTRL		__REG32(IO_ADDRESS(_OTG_CORE_BASE+0x0C))	/*  32bit core clock control reg */
#define OTG_CORE_RST_CTRL		__REG32(IO_ADDRESS(_OTG_CORE_BASE+0x10))	/*  32bit core reset control reg */
#define OTG_CORE_FRM_INTVL		__REG32(IO_ADDRESS(_OTG_CORE_BASE+0x14))	/*  32bit core frame interval reg */
#define OTG_CORE_FRM_REMAIN		__REG32(IO_ADDRESS(_OTG_CORE_BASE+0x18))	/*  32bit core frame remaining reg */
#define OTG_CORE_HNP_CSTAT		__REG32(IO_ADDRESS(_OTG_CORE_BASE+0x1C))	/*  32bit core HNP current state reg */
#define OTG_CORE_HNP_TIMER1		__REG32(IO_ADDRESS(_OTG_CORE_BASE+0x20))	/*  32bit core HNP timer 1 reg */
#define OTG_CORE_HNP_TIMER2		__REG32(IO_ADDRESS(_OTG_CORE_BASE+0x24))	/*  32bit core HNP timer 2 reg */
#define OTG_CORE_HNP_T3PCR		__REG32(IO_ADDRESS(_OTG_CORE_BASE+0x28))	/*  32bit core HNP timer 3 pulse ctrl */
#define OTG_CORE_HINT_STAT		__REG32(IO_ADDRESS(_OTG_CORE_BASE+0x2C))	/*  32bit core HNP int status reg */
#define OTG_CORE_HINT_STEN		__REG32(IO_ADDRESS(_OTG_CORE_BASE+0x30))	/*  32bit core HNP int enable reg */

#define OTG_FUNC_BASE			IO_ADDRESS(_OTG_FUNC_BASE)
#define OTG_FUNC_CND_STAT		__REG32(IO_ADDRESS(_OTG_FUNC_BASE+0x00))	/*  32bit func command status reg */
#define OTG_FUNC_DEV_ADDR		__REG32(IO_ADDRESS(_OTG_FUNC_BASE+0x04))	/*  32bit func device address reg */
#define OTG_FUNC_SINT_STAT		__REG32(IO_ADDRESS(_OTG_FUNC_BASE+0x08))	/*  32bit func system int status reg */
#define OTG_FUNC_SINT_STEN		__REG32(IO_ADDRESS(_OTG_FUNC_BASE+0x0C))	/*  32bit func system int enable reg */
#define OTG_FUNC_XINT_STAT		__REG32(IO_ADDRESS(_OTG_FUNC_BASE+0x10))	/*  32bit func X buf int status reg */
#define OTG_FUNC_YINT_STAT		__REG32(IO_ADDRESS(_OTG_FUNC_BASE+0x14))	/*  32bit func Y buf int status reg */
#define OTG_FUNC_XYINT_STEN		__REG32(IO_ADDRESS(_OTG_FUNC_BASE+0x18))	/*  32bit func XY buf int enable reg */
#define OTG_FUNC_XFILL_STAT		__REG32(IO_ADDRESS(_OTG_FUNC_BASE+0x1C))	/*  32bit func X filled status reg */
#define OTG_FUNC_YFILL_STAT		__REG32(IO_ADDRESS(_OTG_FUNC_BASE+0x20))	/*  32bit func Y filled status reg */
#define OTG_FUNC_EP_EN			__REG32(IO_ADDRESS(_OTG_FUNC_BASE+0x24))	/*  32bit func endpoints enable reg */
#define OTG_FUNC_EP_RDY			__REG32(IO_ADDRESS(_OTG_FUNC_BASE+0x28))	/*  32bit func endpoints ready reg */
#define OTG_FUNC_IINT			__REG32(IO_ADDRESS(_OTG_FUNC_BASE+0x2C))	/*  32bit func immediate interrupt reg */
#define OTG_FUNC_EP_DSTAT		__REG32(IO_ADDRESS(_OTG_FUNC_BASE+0x30))	/*  32bit func endpoints done status */
#define OTG_FUNC_EP_DEN			__REG32(IO_ADDRESS(_OTG_FUNC_BASE+0x34))	/*  32bit func endpoints done enable */
#define OTG_FUNC_EP_TOGGLE		__REG32(IO_ADDRESS(_OTG_FUNC_BASE+0x38))	/*  32bit func endpoints toggle bits */
#define OTG_FUNC_FRM_NUM		__REG32(IO_ADDRESS(_OTG_FUNC_BASE+0x3C))	/*  32bit func frame number reg */

#define OTG_HOST_BASE			IO_ADDRESS(_OTG_HOST_BASE)
#define OTG_HOST_CTRL			__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x00))	/*  32bit host controller config reg */
#define OTG_HOST_SINT_STAT		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x08))	/*  32bit host system int status reg */
#define OTG_HOST_SINT_STEN		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x0C))	/*  32bit host system int enable reg */
#define OTG_HOST_XINT_STAT		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x18))	/*  32bit host X buf int status reg */
#define OTG_HOST_YINT_STAT		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x1C))	/*  32bit host Y buf int status reg */
#define OTG_HOST_XYINT_STEN		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x20))	/*  32bit host XY buf int enable reg */
#define OTG_HOST_XFILL_STAT		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x28))	/*  32bit host X filled status reg */
#define OTG_HOST_YFILL_STAT		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x2C))	/*  32bit host Y filled status reg */
#define OTG_HOST_ETD_EN			__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x40))	/*  32bit host ETD enables reg */
#define OTG_HOST_ETD_EN_CLR		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x48))	/*  32bit host direct routing reg. TO2 only */
#define OTG_HOST_IINT			__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x4C))	/*  32bit host immediate interrupt reg */
#define OTG_HOST_ETD_STAT		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x50))	/*  32bit host endpoints done status */
#define OTG_HOST_ETD_DONE		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x54))	/*  32bit host ETD done reg */
#define OTG_HOST_FRM_NUM		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x60))	/*  32bit host frame number reg */
#define OTG_HOST_LSP_THRESH		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x64))	/*  32bit host low speed threshold reg */
#define OTG_HOST_HUB_DESCA		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x68))	/*  32bit host root hub descriptor A */
#define OTG_HOST_HUB_DESCB		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x6C))	/*  32bit host root hub descriptor B */
#define OTG_HOST_HUB_STAT		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x70))	/*  32bit host root hub status reg */
#define OTG_HOST_PORT1_STAT		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x74))	/*  32bit host port 1 status bits */
#define OTG_HOST_PORT2_STAT		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x78))	/*  32bit host port 2 status bits */
#define OTG_HOST_PORT3_STAT		__REG32(IO_ADDRESS(_OTG_HOST_BASE+0x7c))	/*  32bit host port 3 status bits */
/* input for x: 0..2 */
#define OTG_HOST_PORT_STAT(x)		__REG32_2(IO_ADDRESS(_OTG_HOST_BASE+0x74),((x)<<2))	/*  32bit host port status bits */

#define OTG_DMA_BASE			IO_ADDRESS(_OTG_DMA_BASE)
#define OTG_DMA_REV_NUM			__REG32(IO_ADDRESS(_OTG_DMA_BASE+0x000))	/*  32bit dma revision number reg */
#define OTG_DMA_DINT_STAT		__REG32(IO_ADDRESS(_OTG_DMA_BASE+0x004))	/*  32bit dma int status reg */
#define OTG_DMA_DINT_STEN		__REG32(IO_ADDRESS(_OTG_DMA_BASE+0x008))	/*  32bit dma int enable reg */
#define OTG_DMA_ETD_ERR			__REG32(IO_ADDRESS(_OTG_DMA_BASE+0x00C))	/*  32bit dma ETD error status reg */
#define OTG_DMA_EP_ERR			__REG32(IO_ADDRESS(_OTG_DMA_BASE+0x010))	/*  32bit dma EP error status reg */
#define OTG_DMA_ETD_EN			__REG32(IO_ADDRESS(_OTG_DMA_BASE+0x020))	/*  32bit dma ETD DMA enable reg */
#define OTG_DMA_EP_EN			__REG32(IO_ADDRESS(_OTG_DMA_BASE+0x024))	/*  32bit dma EP DMA enable reg */
#define OTG_DMA_ETD_ENXREQ		__REG32(IO_ADDRESS(_OTG_DMA_BASE+0x028))	/*  32bit dma ETD DMA enable Xtrig req */
#define OTG_DMA_EP_ENXREQ		__REG32(IO_ADDRESS(_OTG_DMA_BASE+0x02C))	/*  32bit dma EP DMA enable Ytrig req */
#define OTG_DMA_ETD_ENXYREQ		__REG32(IO_ADDRESS(_OTG_DMA_BASE+0x030))	/*  32bit dma ETD DMA enble XYtrig req */
#define OTG_DMA_EP_ENXYREQ		__REG32(IO_ADDRESS(_OTG_DMA_BASE+0x034))	/*  32bit dma EP DMA enable XYtrig req */
#define OTG_DMA_ETD_BURST4		__REG32(IO_ADDRESS(_OTG_DMA_BASE+0x038))	/*  32bit dma ETD DMA enble burst4 reg */
#define OTG_DMA_EP_BURST4		__REG32(IO_ADDRESS(_OTG_DMA_BASE+0x03C))	/*  32bit dma EP DMA enable burst4 reg */
#define OTG_DMA_MISC_CTRL		__REG32(IO_ADDRESS(_OTG_DMA_BASE+0x040))	/*  32bit dma EP misc control reg */
#define OTG_DMA_ETD_CH_CLR		__REG32(IO_ADDRESS(_OTG_DMA_BASE+0x048))	/*  32bit dma ETD clear channel reg */
#define OTG_DMA_EP_CH_CLR		__REG32(IO_ADDRESS(_OTG_DMA_BASE+0x04C))	/*  32bit dma EP clear channel reg */

#define OTG_DMA_ETD_MSA(x)	__REG32_2(IO_ADDRESS(_OTG_DMA_BASE+0x100), (x) << 2)	/*  32bit dma ETD mem start addr reg */

#define OTG_DMA_EP_O_MSA(x)	__REG32_2(IO_ADDRESS(_OTG_DMA_BASE+0x180), (x) << 3)	/*  32bit dma EP0 o/p mem start addr */
#define OTG_DMA_EP_I_MSA(x)	__REG32_2(IO_ADDRESS(_OTG_DMA_BASE+0x184), (x) << 3)	/*  32bit dma EP0 i/p mem start addr */

#define OTG_DMA_ETD_BPTR(x)	__REG32_2(IO_ADDRESS(_OTG_DMA_BASE+0x200), (x) << 2)	/*  32bit dma ETD0 buf tx pointer reg */

#define OTG_DMA_EP_O_BPTR(x)	__REG32_2(IO_ADDRESS(_OTG_DMA_BASE+0x280), (x) << 3)	/*  32bit dma EP0 o/p buf tx pointer */
#define OTG_DMA_EP_I_BPTR(x)	__REG32_2(IO_ADDRESS(_OTG_DMA_BASE+0x284), (x) << 3)	/*  32bit dma EP0 i/p buf tx pointer */

#define OTG_I2C_BASE			IO_ADDRESS(_OTG_I2C_BASE)

/*ISP1301-01 USB OTG Transceiver I2C Register Addresses*/
#define OTG_TXCVR_VENDOR_ID_REG0		0x00
#define OTG_TXCVR_VENDOR_ID_REG1		0x01
#define OTG_TXCVR_PRODUCT_ID_REG0		0x02
#define OTG_TXCVR_PRODUCT_ID_REG1		0x03
#define OTG_TXCVR_MODE_REG1_SET         	0x04
#define OTG_TXCVR_MODE_REG1_CLR         	0x05
#define OTG_TXCVR_CTRL_REG1_SET         	0x06
#define OTG_TXCVR_CTRL_REG1_CLR         	0x07
#define OTG_TXCVR_INT_SRC_REG           	0x08
#define OTG_TXCVR_INT_LAT_REG_SET       	0x0a
#define OTG_TXCVR_INT_LAT_REG_CLR       	0x0b
#define OTG_TXCVR_INT_FALSE_REG_SET     	0x0c
#define OTG_TXCVR_INT_FALSE_REG_CLR     	0x0d
#define OTG_TXCVR_INT_TRUE_REG_SET      	0x0e
#define OTG_TXCVR_INT_TRUE_REG_CLR      	0x0f
#define OTG_TXCVR_CTRL_REG2_SET         	0x10
#define OTG_TXCVR_CTRL_REG2_CLR         	0x11
#define OTG_TXCVR_MODE_REG2_SET         	0x12
#define OTG_TXCVR_MODE_REG2_CLR         	0x13
#define OTG_TXCVR_BCD_DEV_REG0          	0x14
#define OTG_TXCVR_BCD_DEV_REG1          	0x15

/*macro to access the ISP1301-01 registers*/
#define OTG_I2C_TXCVR_REG(x)		__REG8(IO_ADDRESS(_OTG_I2C_BASE+(x)))	/*   8bit I2C reg */

#define OTG_I2C_OTG_XCVR_DEVAD		__REG8(IO_ADDRESS(_OTG_I2C_BASE+0x18))	/*   8bit I2C reg */
#define OTG_I2C_SEQ_OP_REG		__REG8(IO_ADDRESS(_OTG_I2C_BASE+0x19))	/*   8bit I2C reg */
#define OTG_I2C_SEQ_RD_STARTAD		__REG8(IO_ADDRESS(_OTG_I2C_BASE+0x1A))	/*   8bit I2C reg */
#define OTG_I2C_OP_CTRL_REG		__REG8(IO_ADDRESS(_OTG_I2C_BASE+0x1B))	/*   8bit I2C reg */
#define OTG_I2C_SCLK_TO_SCL_HPER	__REG8(IO_ADDRESS(_OTG_I2C_BASE+0x1E))	/*   8bit I2C reg */
#define OTG_I2C_MASTER_INT_REG		__REG8(IO_ADDRESS(_OTG_I2C_BASE+0x1F))	/*   8bit I2C reg */

/*
 * EMMA
 */

#define _EMMA_BASE	0x10026000

#define EMMA_BASE	IO_ADDRESS(_EMMA_BASE)

#define _EMMA_PP_BASE	(_EMMA_BASE+0x000)	/*  base location for post processor */
#define _EMMA_PRP_BASE	(_EMMA_BASE+0x400)	/*  base location for pre processor */
#define _EMMA_DEC_BASE	(_EMMA_BASE+0x800)	/*  base location for decoder */
#define _EMMA_ENC_BASE	(_EMMA_BASE+0xC00)	/*  base location for encoder */
#define EMMA_PP_IO_SIZE		(0x400)

#define EMMA_PP_BASE			IO_ADDRESS(_EMMA_PP_BASE)
#define EMMA_PP_CNTL			__REG32(IO_ADDRESS(_EMMA_PP_BASE+0x00))	/*  32bit post processor control reg */
#define EMMA_PP_INTRCTRL		__REG32(IO_ADDRESS(_EMMA_PP_BASE+0x04))	/*  32bit pp interrupt enable reg */
#define EMMA_PP_INTRSTATUS		__REG32(IO_ADDRESS(_EMMA_PP_BASE+0x08))	/*  32bit pp interrupt status reg */
#define EMMA_PP_SY_PTR			__REG32(IO_ADDRESS(_EMMA_PP_BASE+0x0C))	/*  32bit pp source Y data ptr reg */
#define EMMA_PP_SCB_PTR			__REG32(IO_ADDRESS(_EMMA_PP_BASE+0x10))	/*  32bit pp source CB data ptr reg */
#define EMMA_PP_SCR_PTR			__REG32(IO_ADDRESS(_EMMA_PP_BASE+0x14))	/*  32bit pp source CR data ptr reg */
#define EMMA_PP_DRGB_PTR		__REG32(IO_ADDRESS(_EMMA_PP_BASE+0x18))	/*  32bit pp dest RGB data ptr reg */
#define EMMA_PP_QUAN_PTR		__REG32(IO_ADDRESS(_EMMA_PP_BASE+0x1C))	/*  32bit pp quantizer data ptr reg */
#define EMMA_PP_PROC_PARA		__REG32(IO_ADDRESS(_EMMA_PP_BASE+0x20))	/*  32bit pp process frame param reg */
#define EMMA_PP_SFRM_WIDTH		__REG32(IO_ADDRESS(_EMMA_PP_BASE+0x24))	/*  32bit pp source frame width reg */
#define EMMA_PP_DDIS_WIDTH		__REG32(IO_ADDRESS(_EMMA_PP_BASE+0x28))	/*  32bit pp destinatn display siz reg */
#define EMMA_PP_DIMAGE_SIZE		__REG32(IO_ADDRESS(_EMMA_PP_BASE+0x2C))	/*  32bit pp destinatn image size reg */
#define EMMA_PP_DPIX_FMT		__REG32(IO_ADDRESS(_EMMA_PP_BASE+0x30))	/*  32bit pp dest pixel format ctr reg */
#define EMMA_PP_RSIZE_IDX		__REG32(IO_ADDRESS(_EMMA_PP_BASE+0x34))	/*  32bit pp resize table index reg */
#define EMMA_PP_CSC_COEF_123            __REG32(IO_ADDRESS(_EMMA_PP_BASE+0x38))	/*  32bit pp CSC coef 1, 2 and 3 */
#define EMMA_PP_CSC_COEF_4              __REG32(IO_ADDRESS(_EMMA_PP_BASE+0x3C))	/*  32bit pp CSC coefs 4 */
#define EMMA_PP_RSIZE_COEF		__REG32(IO_ADDRESS(_EMMA_PP_BASE+0x100))	/*  32bit pp resize coef table reg */

#define PP_CNTL_EN                      (0x00000001)
#define PP_CNTL_DEBLOCKEN               (0x00000002)
#define PP_CNTL_DERINGEN                (0x00000004)
#define PP_CNTL_CSCEN                   (0x00000010)
#define PP_CNTL_CSCEN_YUV422            (0x00000000)
#define PP_CNTL_CSCEN_RGB               (0x00000010)
#define PP_CNTL_CSC_TABLE_SEL           (0x00000060)
#define PP_CNTL_CSC_TABLE_SEL_A1        (0x00000000)
#define PP_CNTL_CSC_TABLE_SEL_A0        (0x00000020)
#define PP_CNTL_CSC_TABLE_SEL_B1        (0x00000040)
#define PP_CNTL_CSC_TABLE_SEL_B0        (0x00000060)
#define PP_CNTL_SWRST                   (0x00000100)
#define PP_CNTL_CSC_OUT                 (0x00000C00)
#define PP_CNTL_CSC_OUT_RGB888          (0x00000C00)
#define PP_CNTL_CSC_OUT_RGB565          (0x00000800)
#define PP_CNTL_BSDI                    (0x00001000)


#define PP_INTRCNTL_FRAMECOMPINTREN     (0x00000001)
#define PP_INTRCNTL_ERRINTR_EN          (0x00000004)

#define PP_INTRSTATUS_FRAME_INTR        (0x00000001)
#define PP_INTRSTATUS_ERR_INTR          (0x00000004)

/* Register offsets */
#define EMMA_PRP_BASE			IO_ADDRESS(_EMMA_PRP_BASE)
#define EMMA_PRP_IO_SIZE		(0x400)

#define PRP_CNTL			__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x00)))
#define PRP_INTRCNTL	 		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x04)))
#define PRP_INTRSTATUS			__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x08)))
#define PRP_SOURCE_Y_PTR		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x0C)))
#define PRP_SOURCE_CB_PTR		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x10)))
#define PRP_SOURCE_CR_PTR		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x14)))
#define PRP_DEST_RGB1_PTR		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x18)))
#define PRP_DEST_RGB2_PTR		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x1C)))
#define PRP_DEST_Y_PTR			__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x20)))
#define PRP_DEST_CB_PTR			__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x24)))
#define PRP_DEST_CR_PTR			__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x28)))
#define PRP_SOURCE_FRAME_SIZE		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x2C)))
#define PRP_DEST_FRAME_FORMAT_CNTL	__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x38)))
#define PRP_SOURCE_FRAME_FORMAT_CNTL	__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x34)))
#define PRP_CH1_DEST_FRAME_CNTL		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x30)))
#define PRP_CH1_OUT_IMAGE_SIZE		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x3C)))
#define PRP_CH2_OUT_IMAGE_SIZE		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x40)))
#define PRP_FRAME_COUNTER		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x44)))	/* TO1 only */
#define PRP_SOURCE_LINE_STRIDE		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x48)))	/* TO2 - moved */
#define PRP_RESIZECNTL			__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x4C)))	/* TO1 only */
#define PRP_LOCK_BIT_REG		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x50)))	/* TO1 only */
/* TO2 */

#define PRP2_SRC_STRIDE			__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x44)))
#define PRP2_CSC_COEF0			__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x48)))
#define PRP2_CSC_COEF1			__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x4C)))
#define PRP2_CSC_COEF2			__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x50)))
#define PRP2_RSZ1_HCOEF1		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x54)))
#define PRP2_RSZ1_HCOEF2		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x58)))
#define PRP2_RSZ1_HCTL			__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x5C)))
#define PRP2_RSZ1_VCOEF1		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x60)))
#define PRP2_RSZ1_VCOEF2		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x64)))
#define PRP2_RSZ1_VCTL			__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x68)))
#define PRP2_RSZ2_HCOEF1		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x6C)))
#define PRP2_RSZ2_HCOEF2		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x70)))
#define PRP2_RSZ2_HCTL			__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x74)))
#define PRP2_RSZ2_VCOEF1		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x78)))
#define PRP2_RSZ2_VCOEF2		__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x7C)))
#define PRP2_RSZ2_VCTL			__REG32(IO_ADDRESS(_EMMA_PRP_BASE+(0x80)))

#define PRP_CNTL_RSTVAL			0xF232
#define PRP_CNTL_EN			B_SET(0)
#define PRP_CNTL_CH1EN			B_SET(1)
#define PRP_CNTL_CH2EN			B_SET(2)
#define PRP_CNTL_CSI			B_SET(3)
/* input & output pixel formats */
#define	PRP_CNTL_IN_YUV420		0x00010
#define	PRP_CNTL_IN_YUV422		0x00030
#define	PRP_CNTL_IN_RGB16		0x00040
#define	PRP_CNTL_IN_RGB32		0x00060
#define PRP_CNTL_CH1_YUYV		0x00082
#define PRP_CNTL_CH1_RGB8		0x00102
#define PRP_CNTL_CH1_RGB16		0x00202
#define PRP_CNTL_CH1_RGB32		0x00302
#define PRP_CNTL_CH2_YUV420		0x00004
#define PRP_CNTL_CH2_YUYV		0x00404
#define PRP_CNTL_CH2_YUV0		0x00804
/* YUV matrix & range for conversion */
#define PRP_CNTL_CSC_0			0x05000
#define PRP_CNTL_CSC_B			0x0A000
/* other cntrl */
#define PRP_CNTL_RST			B_SET(16)
#define PRP_CNTL_WINEN			B_SET(18)
#define PRP_CNTL_LOOP			B_SET(19)
/* csi frame skip values */
/* input & output fifo sizes */
#define PRP_CNTL_FIFO_I128		0
#define PRP_CNTL_FIFO_I96		B_SET(23)
#define PRP_CNTL_FIFO_I64		B_SET(24)
#define PRP_CNTL_FIFO_I32		(B_SET(23) | B_SET(24))
#define PRP_CNTL_FIFO_O64		0
#define PRP_CNTL_FIFO_O48		B_SET(25)
#define PRP_CNTL_FIFO_O32		B_SET(26)
#define PRP_CNTL_FIFO_O16		(B_SET(25) | B_SET(26))
/* endian input & output & 16bit swap for input */
#define PRP_CNTL_BENDI			B_SET(27)
#define PRP_CNTL_BEND1			B_SET(28)
#define PRP_CNTL_BEND2			B_SET(28)
#define PRP_CNTL_SWAPI16		B_SET(30)
/* TO2 */
#define PRP2_CNTL_RSTVAL		0x28
#define PRP2_CNTL_CH1EN			B_SET(0)
#define PRP2_CNTL_CH2EN			B_SET(1)
#define PRP2_CNTL_CSI			B_SET(2)
#define PRP2_CNTL_IN_32			B_SET(3)
#define PRP2_CNTL_IN_RGB		B_SET(4)
#define PRP2_CNTL_IN_YUV420		0
#define PRP2_CNTL_IN_YUV422		PRP2_CNTL_IN_32
#define PRP2_CNTL_IN_RGB16		PRP2_CNTL_IN_RGB
#define PRP2_CNTL_IN_RGB32		(PRP2_CNTL_IN_RGB | PRP2_CNTL_IN_32)
#define PRP2_CNTL_CH1_RGB8		0
#define PRP2_CNTL_CH1_RGB16		B_SET(5)
#define PRP2_CNTL_CH1_RGB32		B_SET(6)
#define PRP2_CNTL_CH1_YUV422		(B_SET(5) | B_SET(6))
#define PRP2_CNTL_CH2_YUV420		0
#define PRP2_CNTL_CH2_YUV422		B_SET(7)
#define PRP2_CNTL_CH2_YUV444		B_SET(8)
#define PRP2_CNTL_CH1_LOOP		B_SET(9)
#define PRP2_CNTL_CH2_LOOP		B_SET(10)
#define PRP2_CNTL_AUTODROP		B_SET(11)
#define PRP2_CNTL_RST			B_SET(12)
#define PRP2_CNTL_CNTREN		B_SET(13)
#define PRP2_CNTL_WINEN			B_SET(14)
#define PRP2_CNTL_UNCHAIN		B_SET(15)
#define PRP2_CNTL_IN_SKIP_NONE		0
#define PRP2_CNTL_IN_SKIP_1_2		B_SET(16)
#define PRP2_CNTL_IN_SKIP_1_3		B_SET(17)
#define PRP2_CNTL_IN_SKIP_2_3		(B_SET(16) | B_SET(17))
#define PRP2_CNTL_IN_SKIP_1_4		B_SET(18)
#define PRP2_CNTL_IN_SKIP_3_4		(B_SET(16) | B_SET(18))
#define PRP2_CNTL_IN_SKIP_2_5		(B_SET(17) | B_SET(18))
#define PRP2_CNTL_IN_SKIP_3_5		(B_SET(16) | B_SET(17) | B_SET(18))
#define PRP2_CNTL_CH1_SKIP_NONE		0
#define PRP2_CNTL_CH1_SKIP_1_2		B_SET(19)
#define PRP2_CNTL_CH1_SKIP_1_3		B_SET(20)
#define PRP2_CNTL_CH1_SKIP_2_3		(B_SET(19) | B_SET(20))
#define PRP2_CNTL_CH1_SKIP_1_4		B_SET(21)
#define PRP2_CNTL_CH1_SKIP_3_4		(B_SET(19) | B_SET(21))
#define PRP2_CNTL_CH1_SKIP_2_5		(B_SET(20) | B_SET(21))
#define PRP2_CNTL_CH1_SKIP_3_5		(B_SET(19) | B_SET(20) | B_SET(21))
#define PRP2_CNTL_CH2_SKIP_NONE		0
#define PRP2_CNTL_CH2_SKIP_1_2		B_SET(22)
#define PRP2_CNTL_CH2_SKIP_1_3		B_SET(23)
#define PRP2_CNTL_CH2_SKIP_2_3		(B_SET(22) | B_SET(23))
#define PRP2_CNTL_CH2_SKIP_1_4		B_SET(24)
#define PRP2_CNTL_CH2_SKIP_3_4		(B_SET(22) | B_SET(24))
#define PRP2_CNTL_CH2_SKIP_2_5		(B_SET(23) | B_SET(24))
#define PRP2_CNTL_CH2_SKIP_3_5		(B_SET(22) | B_SET(23) | B_SET(24))
#define PRP2_CNTL_FIFO_I128		0
#define PRP2_CNTL_FIFO_I96		B_SET(25)
#define PRP2_CNTL_FIFO_I64		B_SET(26)
#define PRP2_CNTL_FIFO_I32		(B_SET(25) | B_SET(26))
#define PRP2_CNTL_FIFO_O64		0
#define PRP2_CNTL_FIFO_O48		B_SET(27)
#define PRP2_CNTL_FIFO_O32		B_SET(28)
#define PRP2_CNTL_FIFO_O16		(B_SET(27) | B_SET(28))
#define PRP2_CNTL_CH2B1			B_SET(29)
#define PRP2_CNTL_CH2B2			B_SET(30)
#define PRP2_CNTL_CH2_FLOWEN		B_SET(31)

#define PRP_INTRCNTL_RDERR		B_SET(0)
#define PRP_INTRCNTL_CH1ERR		B_SET(1)
#define PRP_INTRCNTL_CH2ERR		B_SET(2)
#define PRP_INTRCNTL_FRM		B_SET(3)
#define PRP2_ICR_CH1			B_SET(3)
#define PRP2_ICR_CH2			B_SET(5)
#define PRP2_ICR_OVR			B_SET(7)
#define PRP2_ICR_FLOW			B_SET(8)

#define PRP_INTRSTAT_RDERR		B_SET(0)
#define PRP_INTRSTAT_CH1ERR		B_SET(1)
#define PRP_INTRSTAT_CH2ERR		B_SET(2)
#define PRP_INTRSTAT_CH2BUF2		B_SET(3)
#define PRP_INTRSTAT_CH2BUF1		B_SET(4)
#define PRP_INTRSTAT_CH1BUF2		B_SET(5)
#define PRP_INTRSTAT_CH1BUF1		B_SET(6)
#define PRP2_ISR_OVR			B_SET(7)
#define PRP2_ISR_FLOW			B_SET(8)



#define EMMA_DEC_BASE			IO_ADDRESS(_EMMA_DEC_BASE)
#define EMMA_MPEG4DEC_DEREG		__REG32(IO_ADDRESS(_EMMA_DEC_BASE+0x00))
#define EMMA_MPEG4DEC_CTRLBASE		__REG32(IO_ADDRESS(_EMMA_DEC_BASE+0x04))
#define EMMA_MPEG4DEC_RLCBASE		__REG32(IO_ADDRESS(_EMMA_DEC_BASE+0x0C))
#define EMMA_MPEG4DEC_MCDOBASE		__REG32(IO_ADDRESS(_EMMA_DEC_BASE+0x14))
#define EMMA_MPEG4DEC_MCDIBASE		__REG32(IO_ADDRESS(_EMMA_DEC_BASE+0x18))
#define EMMA_MPEG4DEC_IDREG		__REG32(IO_ADDRESS(_EMMA_DEC_BASE+0x1C))

#define EMMA_ENC_BASE			IO_ADDRESS(_EMMA_ENC_BASE)
#define EMMA_MPEG4ENC_CONTREG0		__REG32(IO_ADDRESS(_EMMA_ENC_BASE+0x00))
#define EMMA_MPEG4ENC_CONTREG1 		__REG32(IO_ADDRESS(_EMMA_ENC_BASE+0x04))
#define EMMA_MPEG4ENC_CONTREG2		__REG32(IO_ADDRESS(_EMMA_ENC_BASE+0x08))
#define EMMA_MPEG4ENC_CONTREG3		__REG32(IO_ADDRESS(_EMMA_ENC_BASE+0x0C))
#define EMMA_MPEG4ENC_CONTREG4		__REG32(IO_ADDRESS(_EMMA_ENC_BASE+0x10))
#define EMMA_MPEG4ENC_INTERRUPT		__REG32(IO_ADDRESS(_EMMA_ENC_BASE+0x14))
#define EMMA_MPEG4ENC_NBLUMBASE		__REG32(IO_ADDRESS(_EMMA_ENC_BASE+0x18))
#define EMMA_MPEG4ENC_IDREG		__REG32(IO_ADDRESS(_EMMA_ENC_BASE+0x1C))
#define EMMA_MPEG4ENC_NBCHBASE		__REG32(IO_ADDRESS(_EMMA_ENC_BASE+0x20))
#define EMMA_MPEG4ENC_MBTYPEBASE	__REG32(IO_ADDRESS(_EMMA_ENC_BASE+0x24))
#define EMMA_MPEG4ENC_VLCCTRBASE	__REG32(IO_ADDRESS(_EMMA_ENC_BASE+0x28))
#define EMMA_MPEG4ENC_VLCDABASE		__REG32(IO_ADDRESS(_EMMA_ENC_BASE+0x2C))
#define EMMA_MPEG4ENC_SARLUMBASE	__REG32(IO_ADDRESS(_EMMA_ENC_BASE+0x30))
#define EMMA_MPEG4ENC_SARCHBASE		__REG32(IO_ADDRESS(_EMMA_ENC_BASE+0x34))
#define EMMA_MPEG4ENC_SAWLUMBASE	__REG32(IO_ADDRESS(_EMMA_ENC_BASE+0x38))
#define EMMA_MPEG4ENC_SAWCHBASE		__REG32(IO_ADDRESS(_EMMA_ENC_BASE+0x3C))

/*
 *  CLOCK RESET
 */

#define _CRM_BASE	0x10027000

#define CRM_BASE	IO_ADDRESS(_CRM_BASE)
#ifdef CONFIG_MX2TO1
#define CRM_CSCR	__REG32(IO_ADDRESS(_CRM_BASE+0x00))	/*  32bit Clock Source Control Reg */
#define CRM_MPCTL0	__REG32(IO_ADDRESS(_CRM_BASE+0x04))	/*  32bit MCU PLL Control Reg */
#define CRM_MPCTL1	__REG32(IO_ADDRESS(_CRM_BASE+0x08))	/*  32bit MCU PLL */
#define CRM_SPCTL0	__REG32(IO_ADDRESS(_CRM_BASE+0x0C))	/*  32bit Serial Perpheral PLL Ctrl 0 */
#define CRM_SPCTL1	__REG32(IO_ADDRESS(_CRM_BASE+0x10))	/*  32bit Serial Perpheral PLL Ctrl 1 */
#define CRM_OSC26MCTL	__REG32(IO_ADDRESS(_CRM_BASE+0x14))	/*  32bit Osc 26M register */
#define CRM_PCDR	__REG32(IO_ADDRESS(_CRM_BASE+0x18))	/*  32bit Serial Perpheral Clk Div Reg */
#define CRM_PCCR0	__REG32(IO_ADDRESS(_CRM_BASE+0x1C))	/*  32bit Perpheral Clk Control Reg 0 */
#define CRM_PCCR1	__REG32(IO_ADDRESS(_CRM_BASE+0x20))	/*  32bit Perpheral Clk Control Reg 1 */
#define CRM_RSR		__REG32(IO_ADDRESS(_CRM_BASE+0x800))	/*  32bit Reset Source Reg */
#else
#define CRM_CSCR	__REG32(IO_ADDRESS(_CRM_BASE+0x00))	/*  32bit Clock Source Control Reg */
#define CRM_MPCTL0	__REG32(IO_ADDRESS(_CRM_BASE+0x04))	/*  32bit MCU PLL Control Reg */
#define CRM_MPCTL1	__REG32(IO_ADDRESS(_CRM_BASE+0x08))	/*  32bit MCU PLL */
#define CRM_SPCTL0	__REG32(IO_ADDRESS(_CRM_BASE+0x0C))	/*  32bit Serial Perpheral PLL Ctrl 0 */
#define CRM_SPCTL1	__REG32(IO_ADDRESS(_CRM_BASE+0x10))	/*  32bit Serial Perpheral PLL Ctrl 1 */
#define CRM_OSC26MCTL	__REG32(IO_ADDRESS(_CRM_BASE+0x14))	/*  32bit Osc 26M register */
#define CRM_PCDR	__REG32(IO_ADDRESS(_CRM_BASE+0x18))	/*  32bit Perpheral Clk Divider Reg 0 */
#define CRM_PCDR0	__REG32(IO_ADDRESS(_CRM_BASE+0x18))	/*  32bit Perpheral Clk Divider Reg 0 */
#define CRM_PCDR1	__REG32(IO_ADDRESS(_CRM_BASE+0x1C))	/*  32bit Perpheral Clk Divider Reg 1 */
#define CRM_PCCR0	__REG32(IO_ADDRESS(_CRM_BASE+0x20))	/*  32bit Perpheral Clk Control Reg 0 */
#define CRM_PCCR1	__REG32(IO_ADDRESS(_CRM_BASE+0x24))	/*  32bit Perpheral Clk Control Reg 1 */
#define CRM_CCSR  	__REG32(IO_ADDRESS(_CRM_BASE+0x28))	/*  32bit Clk Control Status Reg */
#define CRM_PMCTL  	__REG32(IO_ADDRESS(_CRM_BASE+0x2C))	/*  PMOS Switch Control Reg*/
#define CRM_PMCOUNT   	__REG32(IO_ADDRESS(_CRM_BASE+0x30))	/*  PMOS Switch Counter Reg*/
#define CRM_WKGDCTL  	__REG32(IO_ADDRESS(_CRM_BASE+0x34))	/*  32bit Wakeup Guard Mode Control Reg */
#endif

/* Clock Source Control Register's definitions */

#ifdef CONFIG_MX2TO1
#define CSCR_CLKO_SEL		(0x7 << 29)
#define CSCR_CLKO_SEL_SHIFT	29
#endif

#define CSCR_USB_DIV		(0x7 << 26)
#define CSCR_USB_DIV_SHIFT	26

#define CSCR_SD_CNT		(0x3 << 24)
#define CSCR_SD_CNT_SHIFT	24

#define CSCR_SPLL_RESTART	(1 << 22)
#define CSCR_MPLL_RESTART	(1 << 21)

#define CSCR_SSI2_SEL		(1 << 20)
#define CSCR_SSI1_SEL		(1 << 19)
#define CSCR_FIR_SEL		(1 << 18)
#define CSCR_SP_SEL		(1 << 17)
#define CSCR_MCU_SEL 		(1 << 16)

#ifdef CONFIG_MX2TO1
#define CSCR_PRESC_SHIFT	14
#define CSCR_PRESC		(0x3 << 14)
#else
#define CSCR_PRESC_SHIFT	29
#define CSCR_PRESC		(0x7 << 29)
#endif

#define CSCR_BCLKDIV_SHIFT	10
#define CSCR_BCLKDIV		(0xf << 10)

#define CSCR_IPDIV_SHIFT	9
#define CSCR_IPDIV		(1 << 9)

#ifdef CONFIG_MX2TO2
#define CSCR_OSC26M_DIV1P5	(1 << 4)
#endif

#define CSCR_OSC26M_EN		(1 << 3)
#define CSCR_FPM_EN		(1 << 2)
#define CSCR_SPEN		(1 << 1)
#define CSCR_MPEN		(1 << 0)

/* MPLL Control Register 0 */

#define MPCTL0_CPLM		(1 << 31)
#define MPCTL0_CPLM_SHIFT	31
#define MPCTL0_PD		(0xf << 26)
#define MPCTL0_PD_SHIFT		26
#define MPCTL0_MFD		(0x3ff << 16)
#define MPCTL0_MFD_SHIFT	16
#define MPCTL0_MFI		(0xf << 10)
#define MPCTL0_MFI_SHIFT	10
#define MPCTL0_MFN		0x3ff
#define MPCTL0_MFN_SHIFT	0

/* MPLL Control Register 1 */

#define MPCTL1_LF		(1 << 15)
#define MPCTL1_LF_SHIFT		15
#define MPCTL1_BRMO		(1 << 6)
#define MPCTL1_BRMO_SHIFT	6

#if defined(CONFIG_MX2TO1)

/* Peripheral Clock Divider Register */
#define PCDR_FIRI_DIV		0x1f
#define PCDR_FIRI_DIV_SHIFT	0

#define PCDR_48MDIV		(0x7 << 5)
#define PCDR_48MDIV_SHIFT	5

#define PCDR_PERDIV1		(0xf << 8)
#define PCDR_PERDIV1_SHIFT	8

#define PCDR_NFCDIV		(0xf << 12)
#define PCDR_NFCDIV_SHIFT	12

#define PCDR_SSI1DIV		(0x3f << 16)
#define PCDR_SSI1DIV_SHIFT	16

#define PCDR_PERDIV2		(0xf << 22)
#define PCDR_PERDIV2_SHIFT	22

#define PCDR_SSI2DIV		(0x3f << 26)
#define PCDR_SSI2DIV_SHIFT	26

#elif defined(CONFIG_MX2TO2)

/* Peripheral Clock Divider Register 0 */

#define PCDR0_FIRI_DIV		0x1f
#define PCDR0_FIRI_DIV_SHIFT	0

#define PCDR0_48MDIV		(0x7 << 5)
#define PCDR0_48MDIV_SHIFT	5

#define PCDR0_NFCDIV		(0xf << 12)
#define PCDR0_NFCDIV_SHIFT	12

#define PCDR0_SSI1DIV		(0x3f << 16)
#define PCDR0_SSI1DIV_SHIFT	16

#define PCDR0_SSI2DIV		(0x3f << 26)
#define PCDR0_SSI2DIV_SHIFT	26

/* Peripheral Clock Divider Register 1 */

#define PCDR1_PERDIV1		0x3f
#define PCDR1_PERDIV1_SHIFT	0
#define PCDR1_PERDIV2		(0x3f << 8)
#define PCDR1_PERDIV2_SHIFT	8
#define PCDR1_PERDIV3		(0x3f << 16)
#define PCDR1_PERDIV3_SHIFT	16
#define PCDR1_PERDIV4		(0x3f << 24)
#define PCDR1_PERDIV4_SHIFT	24

#endif


/* Peripheral Clock Control Register 0 */

#define PCCR0_HCLK_CSI_EN	0x80000000
#define PCCR0_HCLK_DMA_EN	0x40000000
#define PCCR0_HCLK_BROM_EN	0x10000000
#define PCCR0_HCLK_EMMA_EN	0x08000000
#define PCCR0_HCLK_LCDC_EN	0x04000000
#define PCCR0_HCLK_SLCD_EN	0x02000000
#define PCCR0_HCLK_USBOTG_EN	0x01000000
#define PCCR0_HCLK_BMI_EN	0x00800000
#ifdef CONFIG_MX2TO2
#define PCCR0_PERCLK4_EN	0x00400000
#endif
#define PCCR0_SLCD_EN		0x00200000
#define PCCR0_FIRI_BAUD_EN	0x00100000
#define PCCR0_NFC_EN		0x00080000
#define PCCR0_LCD_PIXCLK_EN	0x00040000
#ifdef CONFIG_MX2TO2
#define PCCR0_PERCLK3_EN	PCCR0_LCD_PIXCLK_EN
#endif
#define PCCR0_SSI1_BAUD_EN	0x00020000
#define PCCR0_SSI2_BAUD_EN	0x00010000
#define PCCR0_EMMA_EN		0x00008000
#define PCCR0_USBOTG_EN		0x00004000
#define PCCR0_DMA_EN		0x00002000
#define PCCR0_I2C_EN		0x00001000
#define PCCR0_GPIO_EN		0x00000800
#define PCCR0_SDHC2_EN		0x00000400
#define PCCR0_SDHC1_EN		0x00000200
#define PCCR0_FIRI_EN		0x00000100
#define PCCR0_SSI2_EN		0x00000080
#define PCCR0_SSI1_EN		0x00000040
#define PCCR0_CSPI2_EN		0x00000020
#define PCCR0_CSPI1_EN		0x00000010
#define PCCR0_UART4_EN		0x00000008
#define PCCR0_UART3_EN		0x00000004
#define PCCR0_UART2_EN		0x00000002
#define PCCR0_UART1_EN		0x00000001

#define PCCR1_OWIRE_EN		0x80000000
#define PCCR1_KPP_EN		0x40000000
#define PCCR1_RTC_EN		0x20000000
#define PCCR1_PWM_EN		0x10000000
#define PCCR1_GPT3_EN		0x08000000
#define PCCR1_GPT2_EN		0x04000000
#define PCCR1_GPT1_EN		0x02000000
#define PCCR1_WDT_EN		0x01000000
#ifdef CONFIG_MX2TO2
#define PCCR1_CSPI3_EN		0x00800000
#define PCCR1_RTIC_EN		0x00400000
#define PCCR1_RNGA_EN		0x00200000
#endif

/*
 * SYSTEM
 */

#define _SYS_BASE	0x10027800
#define SYS_BASE	IO_ADDRESS(_SYS_BASE)
#define SYS_SIDR	__REG32(IO_ADDRESS(_SYS_BASE+0x04))	/*  128bit Silicon ID Reg */
#define SYS_SIDR1	__REG32(IO_ADDRESS(_SYS_BASE+0x04))	/*  128bit Silicon ID Reg word 1 */
#define SYS_SIDR2	__REG32(IO_ADDRESS(_SYS_BASE+0x08))	/*  128bit Silicon ID Reg word 2 */
#define SYS_SIDR3	__REG32(IO_ADDRESS(_SYS_BASE+0x0C))	/*  128bit Silicon ID Reg word 3 */
#define SYS_SIDR4	__REG32(IO_ADDRESS(_SYS_BASE+0x10))	/*  128bit Silicon ID Reg word 4 */
#define SYS_FMCR	__REG32(IO_ADDRESS(_SYS_BASE+0x14))	/*  Functional Muxing Control Reg */
#define SYS_GPCR	__REG32(IO_ADDRESS(_SYS_BASE+0x18))	/*  Global Peripheral Control Reg */
#define SYS_WBCR	__REG32(IO_ADDRESS(_SYS_BASE+0x1C))	/*  Well Bias Control Reg */
#define SYS_DSCR1	__REG32(IO_ADDRESS(_SYS_BASE+0x20))	/*  Drive Strength Crtl Reg 1 */
#define SYS_DSCR2	__REG32(IO_ADDRESS(_SYS_BASE+0x24))	/*  Drive Strength Crtl Reg 2 */
#define SYS_DSCR3	__REG32(IO_ADDRESS(_SYS_BASE+0x28))	/*  Drive Strength Crtl Reg 3 */
#define SYS_DSCR4	__REG32(IO_ADDRESS(_SYS_BASE+0x2C))	/*  Drive Strength Crtl Reg 4 */
#define SYS_DSCR5	__REG32(IO_ADDRESS(_SYS_BASE+0x30))	/*  Drive Strength Crtl Reg 5 */
#define SYS_DSCR6	__REG32(IO_ADDRESS(_SYS_BASE+0x34))	/*  Drive Strength Crtl Reg 6 */
#define SYS_DSCR7	__REG32(IO_ADDRESS(_SYS_BASE+0x38))	/*  Drive Strength Crtl Reg 7 */
#define SYS_DSCR8	__REG32(IO_ADDRESS(_SYS_BASE+0x3C))	/*  Drive Strength Crtl Reg 8 */
#define SYS_DSCR9	__REG32(IO_ADDRESS(_SYS_BASE+0x40))	/*  Drive Strength Crtl Reg 9 */
#define SYS_DSCR10	__REG32(IO_ADDRESS(_SYS_BASE+0x44))	/*  Drive Strength Crtl Reg 10 */
#define SYS_DSCR11	__REG32(IO_ADDRESS(_SYS_BASE+0x48))	/*  Drive Strength Crtl Reg 11 */
#define SYS_DSCR12	__REG32(IO_ADDRESS(_SYS_BASE+0x4C))	/*  Drive Strength Crtl Reg 12 */
#define SYS_PSCR	__REG32(IO_ADDRESS(_SYS_BASE+0x50))	/*  Priority Control/select Reg */

/*
 * FIRI
 */

#define _FIRI_BASE	0x10028000
#define FIRI_BASE	IO_ADDRESS(_FIRI_BASE)
#define FIRI_FIRITCR	__REG32(IO_ADDRESS(_FIRI_BASE+0x00))	/*  32bit firi tx control reg */
#define FIRI_FIRITCTR	__REG32(IO_ADDRESS(_FIRI_BASE+0x04))	/*  32bit firi tx count  reg */
#define FIRI_FIRIRCR	__REG32(IO_ADDRESS(_FIRI_BASE+0x08))	/*  32bit firi rx control reg */
#define FIRI_FIRITSR	__REG32(IO_ADDRESS(_FIRI_BASE+0x0C))	/*  32bit firi tx status reg */
#define FIRI_FIRIRSR	__REG32(IO_ADDRESS(_FIRI_BASE+0x10))	/*  32bit firi rx status reg */
#define FIRI_TFIFO	__REG32(IO_ADDRESS(_FIRI_BASE+0x14))	/*  32bit firi tx fifo reg */
#define FIRI_RFIFO	__REG32(IO_ADDRESS(_FIRI_BASE+0x18))	/*  32bit firi rx fifo reg */
#define FIRI_FIRICR	__REG32(IO_ADDRESS(_FIRI_BASE+0x1C))	/*  32bit firi control reg */

/*
 * JAM
 */

#define _JAM_BASE	0x1003E000
#define JAM_BASE	IO_ADDRESS(_JAM_BASE)
#define JAM_ARM9P_GPR0	__REG32(IO_ADDRESS(_JAM_BASE+0x00))	/*  32bit jam debug enable */
#define JAM_ARM9P_GPR4	__REG32(IO_ADDRESS(_JAM_BASE+0x10))	/*  32bit jam platform version */

/*
 * MAX
 */

#define _MAX_BASE	0x1003F000

/* input range for x: 0 .. 3 */
#define MAX_BASE(x)			IO_ADDRESS(_MAX_BASE + ((x) << 8))
#define MAX_SLV_MPR(x)			__REG32_2(IO_ADDRESS(_MAX_BASE+0x00), (x) << 8)	/*  32bit max slv master priority reg */
#define MAX_SLV_AMPR(x)			__REG32_2(IO_ADDRESS(_MAX_BASE+0x04), (x) << 8)	/*  32bit max slv0 alt priority reg */
#define MAX_SLV_SGPCR(x)		__REG32_2(IO_ADDRESS(_MAX_BASE+0x10), (x) << 8)	/*  32bit max slv0 general ctrl reg */
#define MAX_SLV_ASGPCR(x)		__REG32_2(IO_ADDRESS(_MAX_BASE+0x14), (x) << 8)	/*  32bit max slv0 alt generl ctrl reg */

#define MAX_MST_MGPCR(x)		__REG32_2(IO_ADDRESS(_MAX_BASE+0x800), (x) << 8)	/*  32bit max mst0-5 general ctrl reg */

/*
 * AITC
 */

#define _AITC_BASE	0x10040000
#define AITC_BASE	IO_ADDRESS(_AITC_BASE)

#define AITC_INTCNTL	__REG32(IO_ADDRESS(_AITC_BASE))	/*  32bit aitc int control reg */
#define AITC_NIMASK	__REG32(IO_ADDRESS(_AITC_BASE+0x04))	/*  32bit aitc int mask reg */
#define AITC_INTENNUM	__REG32(IO_ADDRESS(_AITC_BASE+0x08))	/*  32bit aitc int enable number reg */
#define AITC_INTDISNUM	__REG32(IO_ADDRESS(_AITC_BASE+0x0C))	/*  32bit aitc int disable number reg */
#define AITC_INTENABLEH	__REG32(IO_ADDRESS(_AITC_BASE+0x10))	/*  32bit aitc int enable reg high */
#define AITC_INTENABLEL	__REG32(IO_ADDRESS(_AITC_BASE+0x14))	/*  32bit aitc int enable reg low */
#define AITC_INTTYPEH	__REG32(IO_ADDRESS(_AITC_BASE+0x18))	/*  32bit aitc int type reg high */
#define AITC_INTTYPEL	__REG32(IO_ADDRESS(_AITC_BASE+0x1C))	/*  32bit aitc int type reg low */

#define AITC_NIPRIORITY(x)	__REG32_2(IO_ADDRESS(_AITC_BASE+0x20), (7-(x))<<2)	/*  32bit aitc norm int priority lvl7 */

#define AITC_NIVECSR	__REG32(IO_ADDRESS(_AITC_BASE+0x40))	/*  32bit aitc norm int vector/status */
#define AITC_FIVECSR	__REG32(IO_ADDRESS(_AITC_BASE+0x44))	/*  32bit aitc fast int vector/status */
#define AITC_INTSRCH	__REG32(IO_ADDRESS(_AITC_BASE+0x48))	/*  32bit aitc int source reg high */
#define AITC_INTSRCL	__REG32(IO_ADDRESS(_AITC_BASE+0x4C))	/*  32bit aitc int source reg low */
#define AITC_INTFRCH	__REG32(IO_ADDRESS(_AITC_BASE+0x50))	/*  32bit aitc int force reg high */
#define AITC_INTFRCL	__REG32(IO_ADDRESS(_AITC_BASE+0x54))	/*  32bit aitc int force reg low */
#define AITC_NIPNDH	__REG32(IO_ADDRESS(_AITC_BASE+0x58))	/*  32bit aitc norm int pending high */
#define AITC_NIPNDL	__REG32(IO_ADDRESS(_AITC_BASE+0x5C))	/*  32bit aitc norm int pending low */
#define AITC_FIPNDH	__REG32(IO_ADDRESS(_AITC_BASE+0x60))	/*  32bit aitc fast int pending high */
#define AITC_FIPNDL	__REG32(IO_ADDRESS(_AITC_BASE+0x64))	/*  32bit aitc fast int pending low */

/*
 *  ROMPATCH
 */

#define _ROMPATCH_BASE	0x10041000
#define ROMPATCH_BASE	IO_ADDRESS(_ROMPATCH_BASE)

#define ROMPATCH_D(x)	__REG32_2(IO_ADDRESS(_ROMPATCH_BASE+0x0B4),(15-x)<<2)	/*  32bit rompatch data reg 15 */

#define ROMPATCH_CNTL	__REG32(IO_ADDRESS(_ROMPATCH_BASE+0x0F4))	/*  32bit rompatch control reg */
#define ROMPATCH_ENH	__REG32(IO_ADDRESS(_ROMPATCH_BASE+0x0F8))	/*  32bit rompatch enable reg high */
#define ROMPATCH_ENL	__REG32(IO_ADDRESS(_ROMPATCH_BASE+0x0FC))	/*  32bit rompatch enable reg low */

#define ROMPATCH_A(x)	__REG32_2(IO_ADDRESS(_ROMPATCH_BASE+0x100),(x) << 2)	/*  32bit rompatch data reg 15 */

#define ROMPATCH_BRPT	__REG32(IO_ADDRESS(_ROMPATCH_BASE+0x200))	/*  32bit rompatch */
#define ROMPATCH_BADR	__REG32(IO_ADDRESS(_ROMPATCH_BASE+0x204))	/*  32bit rompatch base addr reg */
#define ROMPATCH_SR	__REG32(IO_ADDRESS(_ROMPATCH_BASE+0x208))	/*  32bit rompatch status reg */
#define ROMPATCH_ABSR	__REG32(IO_ADDRESS(_ROMPATCH_BASE+0x20C))	/*  32bit rompatch abort status reg */
#define ROMPATCH_DADR	__REG32(IO_ADDRESS(_ROMPATCH_BASE+0x210))	/*  32bit rompatch d-ahb addr abort */
#define ROMPATCH_IADR	__REG32(IO_ADDRESS(_ROMPATCH_BASE+0x214))	/*  32bit rompatch i-ahb addr abort */

/*
 * SMN
 */

#define _SMN_BASE	0x10042000
#define SMN_BASE	IO_ADDRESS(_SMN_BASE)
#define SMN_STATUS	__REG32(IO_ADDRESS(_SMN_BASE+0x00))	/*  32bit SMN status reg */
#define SMN_CONTROL	__REG32(IO_ADDRESS(_SMN_BASE+0x04))	/*  32bit SMN command reg */
#define SMN_SEQ_START	__REG32(IO_ADDRESS(_SMN_BASE+0x08))	/*  32bit SMN sequence start reg */
#define SMN_SEQ_END	__REG32(IO_ADDRESS(_SMN_BASE+0x0C))	/*  32bit SMN sequence end reg */
#define SMN_SEQ_CHK	__REG32(IO_ADDRESS(_SMN_BASE+0x10))	/*  32bit SMN sequence check reg */
#define SMN_BIT_CNT	__REG32(IO_ADDRESS(_SMN_BASE+0x14))	/*  32bit SMN bit count reg */
#define SMN_INC_SIZE	__REG32(IO_ADDRESS(_SMN_BASE+0x18))	/*  32bit SMN increment size reg */
#define SMN_BB_DEC	__REG32(IO_ADDRESS(_SMN_BASE+0x1C))	/*  32bit SMN bit bank decrement reg */
#define SMN_COMP_SIZE	__REG32(IO_ADDRESS(_SMN_BASE+0x20))	/*  32bit SMN compare size reg */
#define SMN_PT_CHK	__REG32(IO_ADDRESS(_SMN_BASE+0x24))	/*  32bit SMN plain text check reg */
#define SMN_CT_CHK	__REG32(IO_ADDRESS(_SMN_BASE+0x28))	/*  32bit SMN cipher text check reg */
#define SMN_TIMER_IV	__REG32(IO_ADDRESS(_SMN_BASE+0x2C))	/*  32bit SMN timer initial value reg */
#define SMN_TIMER_CTL	__REG32(IO_ADDRESS(_SMN_BASE+0x30))	/*  32bit SMN timer control reg */
#define SMN_DD_STATUS	__REG32(IO_ADDRESS(_SMN_BASE+0x34))	/*  32bit SMN debug detector reg */
#define SMN_TIMER	__REG32(IO_ADDRESS(_SMN_BASE+0x38))	/*  32bit SMN timer reg */

/*
 * SCM
 */

#define _SCM_BASE		0x10043000
#define SCM_BASE		IO_ADDRESS(_SCM_BASE)
#define SCM_RED_START		__REG32(IO_ADDRESS(_SCM_BASE+0x00))	/*  32bit SCM red memory start addr */
#define SCM_BLACK_START		__REG32(IO_ADDRESS(_SCM_BASE+0x004))	/*  32bit SCM black memory start addr */
#define SCM_LENGTH		__REG32(IO_ADDRESS(_SCM_BASE+0x008))	/*  32bit SCM num blks encrypted */
#define SCM_CONTROL		__REG32(IO_ADDRESS(_SCM_BASE+0x00C))	/*  32bit SCM control reg */
#define SCM_STATUS		__REG32(IO_ADDRESS(_SCM_BASE+0x010))	/*  32bit SCM status reg */
#define SCM_ERROR		__REG32(IO_ADDRESS(_SCM_BASE+0x014))	/*  32bit SCM error status reg */
#define SCM_INT_MASK		__REG32(IO_ADDRESS(_SCM_BASE+0x018))	/*  32bit SCM interrupt control reg */
#define SCM_CONFIGURATION	__REG32(IO_ADDRESS(_SCM_BASE+0x01C))	/*  32bit SCM configuration */
#define SCM_INIT_VECTOR_0	__REG32(IO_ADDRESS(_SCM_BASE+0x020))	/*  32bit SCM initialization vector high */
#define SCM_INIT_VECTOR_1	__REG32(IO_ADDRESS(_SCM_BASE+0x024))	/*  32bit SCM initialization vector low */
#define SCM_RED_MEM_BASE	__REG32(IO_ADDRESS(_SCM_BASE+0x400))	/*  32bit Red memory regs (400 - 7FF) */
#define SCM_BLACK_MEM_BASE	__REG32(IO_ADDRESS(_SCM_BASE+0x800))	/*  32bit Black memory regs (800 - BFF) */

/*
 * CSI
 */

#define _CSI_BASE		MX2_CSI_BASE
#define CSI_BASE		MX2_CSI_IOBASE
#define CSI_IO_SIZE		MX2_CSI_SIZE
#define CSI_CSICR1		__REG32(CSI_BASE+0x00)	/*  32bit csi control 1 reg */
#define CSI_CSICR2		__REG32(CSI_BASE+0x04)	/*  32bit csi control 2 reg */
#define CSI_CSISR		__REG32(CSI_BASE+0x08)	/*  32bit csi status reg */
#define CSI_CSISTATR		__REG32(CSI_BASE+0x0C)	/*  32bit csi fifo statistics reg */
#define CSI_CSIRXR		__REG32(CSI_BASE+0x10)	/*  32bit csi receive image reg */
#define CSI_CSIRXCNT		__REG32(CSI_BASE+0x14)	/*  32bit csi receive count reg */

#define _CSI_CSIRXR		_CSI_BASE+0x10	/*  32bit csi receive image reg */

#define   CSICR1_REDGE               0x00000002	/* Pixel data is latched at the falling edge of CSI_PIXCLK */
#define   CSICR1_INV_PCLK            0x00000004	/* CSI_PIXCLK is inverted before applied to circuitry */
#define   CSICR1_INV_DATA            0x00000008	/* CSI_D data lines are invereted */
#define   CSICR1_GCLK_MODE           0x00000010	/* Enables Gated clock mode */
#define   CSICR1_CLR_RXFIFO          0x00000020	/* Clear asynchronous RXFIFO */
#define   CSICR1_CLR_STATFIFO        0x00000040	/* Clear asynchronous STATFIFO */
#define   CSICR1_PACK_DIR_MSBFIRST   0x00000080	/* Pack 8-bit image date into 32-bit from MSB first */
#define   CSICR1_PACK_DIR_LSBFIRST   0x00000000	/* Pack 8-bit image date into 32-bit from LSB first */
#define   CSICR1_FCC_SCLR            0x00000100	/* RX FIFO & STAT FIFO are cleared on every SOF */
#define   CSICR1_MCLKEN              0x00000200	/* MCLK input to sensor enable */
#define   CSICR1_CCIR_EN             0x00000400	/* CCIR656 Interface is selected */
#define   CSICR1_HSYNC_POL_HIGH      0x00000800	/* HSYNC Polarity is active high */
#define   CSICR1_HSYNC_POL_LOW       0x00000000	/* HSYNC Polarity is active high */
#define   CSICR1_MCLKDIV_SHIFT      (12)
#define   CSICR1_MCLKDIV            (0xF << CSICR1_MCLKDIV_SHIFT)
#define   CSICR1_MCLKDIV_x(_num)     ((((_num) - 1) >> 1) << 12)
#define   CSICR1_MCLKDIV_2           CSICR1_MCLKDIV_x(2)	/* MCLK divided by 2 */
#define   CSICR1_MCLKDIV_4           CSICR1_MCLKDIV_x(4)	/* MCLK divided by 4 */
#define   CSICR1_MCLKDIV_6           CSICR1_MCLKDIV_x(6)	/* MCLK divided by 6 */
#define   CSICR1_MCLKDIV_32          CSICR1_MCLKDIV_x(32)	/* MCLK divided by 32 */

#define   CSICR1_SOF_INTEN           0x00010000	/* Start of frame interrupt enable */
#define   CSICR1_SOF_POL_RISE        0x00020000	/* Rising edge on SOF treated as SOF interrupt */
#define   CSICR1_SOF_POL_FALL        0x00000000	/* Falling edge on SOF treated as SOF interrupt */
#define   CSICR1_RXFF_INTEN          0x00040000	/* RxFIFO full interrupt enable */
#define   CSICR1_RXFF_LEVEL_4        0x00000000	/* RxFifo full level */
#define   CSICR1_RXFF_LEVEL_8        0x00080000
#define   CSICR1_RXFF_LEVEL_16       0x00100000
#define   CSICR1_RXFF_LEVEL_24       0x00180000
#define   CSICR1_STATFF_INTEN        0x00200000	/* StatFIFO full interrupt enable */
#define   CSICR1_STATFF_LEVEL_4      0x00000000	/* StatFIFO full level */
#define   CSICR1_STATFF_LEVEL_8      0x00400000
#define   CSICR1_STATFF_LEVEL_16     0x00800000
#define   CSICR1_STATFF_LEVEL_32     0x00c00000
#define   CSICR1_RXFFOR_INTEN        0x01000000	/* Rx FIFO overrun interrupt enable */
#define   CSICR1_STATFFOR_INTEN      0x02000000	/* Stat FIFO overrun interrupt enable */
#define   CSICR1_COF_INTEN           0x04000000	/* COF interrupt enable */
#define   CSICR1_CCIR_MODE_INTRL     0x08000000	/* CCIR interlace mode */
#define   CSICR1_CCIR_MODE_PROGR     0x00000000	/* CCIR progressive mode */
#define   CSICR1_PRP_IFEN            0x10000000	/* CSI to PrP bus enable */
#define   CSICR1_EOF_INTEN           0x20000000	/* End of Frame interrupt enable */
#define   CSICR1_EXT_VSYNC           0x40000000	/* External VSYNC mode enable */
#define   CSICR1_SWAP16_EN           0x80000000	/* Enable swapping of 16-bit data */

#define   CSISR_RXFF_READY           0x00000001	/* Data ready into RxFIFO */
#define   CSISR_COF_INT              0x00002000	/* Change of Frame (COF) interrupt */
#define   CSISR_SOF_INT              0x00010000	/* Start of Frame (SOF) interrupt */
#define   CSISR_EOF_INT              0x00020000	/* End of Frame (EOF) interrupt */
#define   CSISR_RXFF_FULL            0x00040000	/* RxFIFO is full */
#define   CSISR_STATFF_FULL          0x00200000	/* StatFIFO is full */
#define   CSISR_RXFFOR_INT           0x01000000	/* RxFIFO overflowed */
#define   CSISR_STATFFOR_INT         0x02000000	/* StatFIFO overflowed */

/*
 * BMI
 */

#define _BMI_BASE	0xA0000000
#define BMI_BASE	IO_ADDRESS(_BMI_BASE)

/* input range for x: 1 .. 2 */
#define BMI_BMICTRL(x)	__REG32_2(IO_ADDRESS(_BMI_BASE+0x00),(x-1)<<2)	/*  32bit bmi control 1-2 reg */

#define BMI_BMISTR		__REG32(IO_ADDRESS(_BMI_BASE+0x08))	/*  32bit bmi status reg */
#define BMI_BMIRXD		__REG32(IO_ADDRESS(_BMI_BASE+0x0C))	/*  32bit bmi Rx FIFO reg */
#define BMI_BMITXD		__REG32(IO_ADDRESS(_BMI_BASE+0x10))	/*  32bit bmi Tx FIFO reg */

/*
 * External Memory CSD0 - CSD01
 */

/* input range for x: 0 .. 1 */
#define CSD_BASE(x)		IO_ADDRESS(0xC0000000 + ((x) << 26))	/*  CS0 (64Mb) */
#define CSD_END_ADDR(x)		IO_ADDRESS(0xC3FFFFFF + ((x) << 26))

/*
 * External Memory CS0 - CS5
 */

/* input range for x: 0 .. 5 */
#define CS_BASE(x)		IO_ADDRESS(0xC8000000 + ((x) << 26))	/*  CS0 (64Mb) */
#define CS_END_ADDR(x)		IO_ADDRESS(0xCBFFFFFF + ((x) << 26))


/*
 * SDRAMC
 * $DF00_0000 to $DF00_0FFF
 */

/*Modified by Bill Chen,Dec 26th,2003...start */
/*We should not use IO_ADDRESS to map SDRAM control registers' IO address */

#define SDRAMC_BASE	MX2ADS_EMI_IOBASE
#define SDRAMC_SDCTL(x)	__REG32_2(MX2ADS_EMI_IOBASE, (x) << 2))	/*  32bit sdram 0 control reg */

#define SDRAMC_MISC	__REG32(MX2ADS_EMI_IOBASE+0x14)	/*  32bit sdram miscellaneous reg */
#define SDRAMC_SDRST	__REG32(MX2ADS_EMI_IOBASE+0x18)	/*  32bit sdram reset reg */

/*
 * WEIM
 */

/* input range for x: 0..5 */
#define WEIM_CSU(x)	__REG32(MX2ADS_EMI_IOBASE+0x1000 + ((x) << 3))	/*  32bit eim chip sel 0 upper ctr reg */
#define WEIM_CSL(x)	__REG32(MX2ADS_EMI_IOBASE+0x1000+0x04 + ((x) << 3))	/*  32bit eim chip sel 0 lower ctr reg */

/*
 * PCMCIA
 */

#define _PCMCIA_BASE	MX2_PCMCIA_BASE
#define PCMCIA_BASE	MX2_PCMCIA_IOBASE

#define PCMCIA_PIPR	__REG32(PCMCIA_BASE + 0x0)	/*  32bit pcmcia input pins reg */
#define PCMCIA_PSCR	__REG32(PCMCIA_BASE + 0x4)	/*  32bit pcmcia status change reg */
#define PCMCIA_PER	__REG32(PCMCIA_BASE + 0x8)	/*  32bit pcmcia enable reg */

#define PCMCIA_PBR(x)	__REG32_2(PCMCIA_BASE + 0xC, (x) << 3)		/*  32bit pcmcia base reg */
#define PCMCIA_POR(x)	__REG32_2(PCMCIA_BASE + 0x28, (x) << 3)	/*  32bit pcmcia option reg */
#define PCMCIA_POFR(x)	__REG32_2(PCMCIA_BASE + 0x44, (x) << 3)	/*  32bit pcmcia offset reg */

#define PCMCIA_PGCR	__REG32(PCMCIA_BASE + 0x60)	/*  32bit pcmcia general control reg */
#define PCMCIA_PGSR	__REG32(PCMCIA_BASE + 0x64)	/*  32bit pcmcia general status reg */


/* PCMCIA/CF IO & Memory space definitions */

/* Three PCMCIA partitions are defined - for IO, Attribute and Memory spaces */

#define _PCMCIA(x)		MX2_PCMCIA_IO_BASE
#define _PCMCIAIO(x)		_PCMCIA(x)
#define _PCMCIAAttr(x)		(_PCMCIA(x) + PCMCIA_IOMEM_PARTITION_SIZE)
#define _PCMCIAMem(x)		(_PCMCIA(x) + 2 * PCMCIA_IOMEM_PARTITION_SIZE)

/*
 * NFC
 */
#define _NFC_BASE			0xDF003000
#define NFC_IO_SIZE			0x1000
#define NFC_BASE			NFC_IO_ADDRESS(_NFC_BASE)

#define NFC_MAB_BASE(x)			NFC_IO_ADDRESS(_NFC_BASE+0x000+((x) << 9))	/*  main area buffer0 (3000 - 31FE) */
#define NFC_SAB_BASE(x)			NFC_IO_ADDRESS(_NFC_BASE+0x800+((x) << 4))	/*  spare area buffer0 (3800 - 380E) */

#define _NFC_REG_BASE                  (_NFC_BASE+0xE00)	/*  register area (3E00 - 3E1C) */

#define NFC_REG_BASE 			NFC_IO_ADDRESS(_NFC_REG_BASE)
#define NFC_BUFSIZE		      __REG16(NFC_IO_ADDRESS(_NFC_REG_BASE+0x00))	/*  16bit nfc internal sram size */
#define NFC_BLK_ADD_LOCK	      __REG16(NFC_IO_ADDRESS(_NFC_REG_BASE+0x02))	/*  16bit nfc block addr for lock chk */
#define NFC_RAM_BUF_ADDR	      __REG16(NFC_IO_ADDRESS(_NFC_REG_BASE+0x04))	/*  16bit nfc buffer number */
#define NFC_NAND_FLASH_ADDR           __REG16(NFC_IO_ADDRESS(_NFC_REG_BASE+0x06))	/*  16bit nfc nand flash address */
#define NFC_NAND_FLASH_CMD	      __REG16(NFC_IO_ADDRESS(_NFC_REG_BASE+0x08))	/*  16bit nfc nand flash command */
#define NFC_CONFIGURATION	      __REG16(NFC_IO_ADDRESS(_NFC_REG_BASE+0x0A))	/*  16bit nfc internal buf lock ctrl */
#define NFC_ECC_STAT_RES	      __REG16(NFC_IO_ADDRESS(_NFC_REG_BASE+0x0C))	/*  16bit nfc controller status/result */
#define NFC_ECC_RSLT_MA		      __REG16(NFC_IO_ADDRESS(_NFC_REG_BASE+0x0E))	/*  16bit nfc ecc err position in main */
#define NFC_ECC_RSLT_SA		      __REG16(NFC_IO_ADDRESS(_NFC_REG_BASE+0x10))	/*  16bit nfc ecc err pos in spare */
#define NFC_NF_WR_PROT		      __REG16(NFC_IO_ADDRESS(_NFC_REG_BASE+0x12))	/*  16bit nfc write protection */
#define NFC_ULOCK_START_BLK	      __REG16(NFC_IO_ADDRESS(_NFC_REG_BASE+0x14))	/*  16bit nfc start unlock location */
#define NFC_ULOCK_END_BLK	      __REG16(NFC_IO_ADDRESS(_NFC_REG_BASE+0x16))	/*  16bit nfc end unlock location */
#define NFC_NF_WR_PROT_STAT	      __REG16(NFC_IO_ADDRESS(_NFC_REG_BASE+0x18))	/*  16bit nfc write protection status */
#define NFC_NF_CONFIG1		      __REG16(NFC_IO_ADDRESS(_NFC_REG_BASE+0x1A))	/*  16bit nfc configuration 1 */
#define NFC_NF_CONFIG2		      __REG16(NFC_IO_ADDRESS(_NFC_REG_BASE+0x1C))	/*  16bit nfc configuration 2 */

/* 	END */
#endif
