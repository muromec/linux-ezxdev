/*
 * linux/drivers/usbd/l7205_bi/hardware.h -- L7205 USB controller driver. 
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 *
 * By: 
 *      Stuart Lynne <sl@lineo.com>, 
 *      Tom Rushworth <tbr@lineo.com>, 
 *      Bruce Balden <balden@lineo.com>
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */


#define __IOA(x) ( (x) < IO_START_2 ?  \
        (volatile unsigned char*) (IO_BASE + (x) - IO_START) : \
        (volatile unsigned char*) (IO_BASE_2 + (x) - IO_START_2) )


/* 
 * usb interrupt
 */

#define IRQ_USBF        32

/*
 * PMU Software
 */

#define SYSTEM_BASE                     0x80050000

#if 0
#define SYS_CONFIG_CURRENT              (SYSTEM_BASE+0x000)
#define SYS_CONFIG_NEXT                 (SYSTEM_BASE+0x004)
#define SYS_CONFIG_RUN                  (SYSTEM_BASE+0x00c)
#define SYS_CONFIG_COMM                 (SYSTEM_BASE+0x010)
#define SYS_CONFIG_SDRAM                (SYSTEM_BASE+0x014)

#define IO_SYS_CONFIG_CURRENT           __IOL(SYSTEM_BASE+0x000)
#define IO_SYS_CONFIG_NEXT              __IOL(SYSTEM_BASE+0x004)
#define IO_SYS_CONFIG_RUN               __IOL(SYSTEM_BASE+0x00c)
#define IO_SYS_CONFIG_COMM              __IOL(SYSTEM_BASE+0x010)
#define IO_SYS_CONFIG_SDRAM             __IOL(SYSTEM_BASE+0x014)

#endif


/* 
 * Current Configuration Register - SYS_CONFIG_CURRENT
 */

#define SYSC_CURR_NFASTBUS              0x00000001
#define SYSC_CURR_DSRF_SEL              0x00000002
#define SYSC_CURR_SDRB_SEL              0x00000004
#define SYSC_CURR_BCLK_DIV              0x00000018

#define SYSC_CURR_PLLMUX                0x00000080
#define SYSC_CURR_PLLEN                 0x00000100
#define SYSC_CURR_PLLMUL                0x00007e00
#define SYSC_CURR_OSCMUX                0x00080000
#define SYSC_CURR_OSCEN                 0x00100000
#define SYSC_CURR_TRANSOP               0x06000000


/*
 * Next Configuration Register - SYS_CONFIG_NEXT
 */

#define SYSC_NEXT_NFASTBUS              0x00000001
#define SYSC_NEXT_SDRF_SEL              0x00000002
#define SYSC_NEXT_SDRB_SEL              0x00000004
#define SYSC_NEXT_BCLK_DIV              0x00000018

#define SYSC_NEXT_SYSCLKEN              0x00000020
#define SYSC_NEXT_SDR_STOP              0x00000040

#define SYSC_NEXT_PLLMUX                0x00000080
#define SYSC_NEXT_PLLEN                 0x00000100
#define SYSC_NEXT_PLLMUL                0x00007e00
#define SYSC_NEXT_OSCMUX                0x00008000
#define SYSC_NEXT_OSCEN                 0x00010000

#define SYSC_NEXT_INTRET                0x00020000
#define SYSC_NEXT_CLOCKRECOVERY         0x01fc0000
#define SYSC_NEXT_TRANSOP               0x06000000

/* 
 * Run Configuration Register - SYS_CONFIG_RUN
 */

#define SYSC_RUN_NFASTBUS               0x00000001
#define SYSC_RUN_DSRF_SEL               0x00000002
#define SYSC_RUN_SDRB_SEL               0x00000004
#define SYSC_RUN_BCLK_DIV               0x00000018

#define SYSC_RUN_PLLMUX                 0x00000080
#define SYSC_RUN_PLLEN                  0x00000100
#define SYSC_RUN_PLLMUL                 0x00007e00
#define SYSC_RUN_OSCMUX                 0x00080000
#define SYSC_RUN_OSCEN                  0x00100000


/*
 * System Control
 */
#if 0
#define SYS_CLOCK_ENABLE                (SYSTEM_BASE+0x030)
#define SYS_CLOCK_ESYNC                 (SYSTEM_BASE+0x034)
#define SYS_CLOCK_SELECT                (SYSTEM_BASE+0x038)

#define IO_SYS_CLOCK_ENABLE             __IOL(SYSTEM_BASE+0x030)
#define IO_SYS_CLOCK_ESYNC              __IOL(SYSTEM_BASE+0x034)
#define IO_SYS_CLOCK_SELECT             __IOL(SYSTEM_BASE+0x038)
#endif
#define SYS_CLOCK_AUX                   (SYSTEM_BASE+0x03c)
#define IO_SYS_CLOCK_AUX                __IOL(SYSTEM_BASE+0x03c)


/*
 * Clock Enable Register - SYS_CLOCK_ENABLE
 */

#define SYS_CLOCK_SYN_EN                0x00000001
#define SYS_CLOCK_B18M_EN               0x00000002
#define SYS_CLOCK_3M6_EN                0x00000004

#define SYS_CLOCK_FIR_EN                0x00000020
#define SYS_CLOCK_MIRM_EN               0x00000040
#define SYS_CLOCK_UARTM_EN              0x00000080
#define SYS_CLOCK_SIBADC_EN             0x00000100
#define SYS_CLOCK_ALTD_EN               0x00000200
#define SYS_CLOCK_CLCLK_EN              0x00000400
#define SYS_CLOCK_STCLK_EN              0x00000800
#define SYS_CLOCK_AUXPLL_EN             0x00001000
#define SYS_CLOCK_AUXCLK_EN             0x00002000
#define SYS_CLOCK_18SYN_EN              0x00004000
#define SYS_CLOCK_SICSYN_EN             0x00008000
#define SYS_CLOCK_NDMA_EN               0x00010000
#define SYS_CLOCK_USBFUNC_EN            0x00020000
#define SYS_CLOCK_USBHOST_EN            0x00080000


/*
 * Clock Multiplexor and Divider Control Register - SYS_CLOCK_SELECT
 */

#define SYS_CLOCK_18M_DIV               0x00000001
#define SYS_CLOCK_MIR_SEL               0x00000002

#define SYS_CLOCK_UART_SEL              0x0000000c

#define SYS_CLOCK_ALT_CLCLK_SEL         0x00000010
#define SYS_CLOCK_MM_DIV                0x00000020
#define SYS_CLOCK_MM_SEL                0x00000030

#define SYS_CLOCK_ADC_SEL               0x00001f80

#define SYS_CLOCK_ALTD_SEL              0x00002000
#define SYS_CLOCK_CL_SEL                0x00004000
#define SYS_CLOCK_18SRC_SEL             0x00008000
#define SYS_CLOCK_18BY_SEL              0x00010000

#define SYS_CLOCK_SICDIV_SEL            0x000e0000

#define SYS_CLOCK_SICBY_SEL             0x00100000
#define SYS_CLOCK_SICSYN_SEL            0x00600000
#define SYS_CLOCK_18_AUX_SEL            0x00800000
#define SYS_CLOCK_FIRAUX_SEL            0x01000000
#define SYS_CLOCK_SICAUX_SEL            0x02000000

/*
 * Auxillary PLL Configuration Register - SYS_CLOCK_AUX
 */

#define SYS_CLOCK_AUXOSCMUX             0x00000001
#define SYS_CLOCK_AUXPLLMUX             0x00000002
#define SYS_CLOCK_AUXPLLMUL             0x000000fc
#define SYS_CLOCK_AUXPLLMUL_18          0x00000014
#define SYS_CLOCK_AUXPLLMUL_48          0x00000034


/*
 * USB Function Controller
 */

#define USB_FUNCTION_BASE               0x8004b000

#if 0

#define USBF_REVISION     		(USB_FUNCTION_BASE+0x000)
#define USBF_CONTROL      		(USB_FUNCTION_BASE+0x004)
#define USBF_STATUS       		(USB_FUNCTION_BASE+0x008)
#define USBF_RAWSTATUS    		(USB_FUNCTION_BASE+0x00c)

#define USBF_INTENA       		(USB_FUNCTION_BASE+0x010)
#define USBF_INTDIS       		(USB_FUNCTION_BASE+0x014)
#define USBF_INTCLR       		(USB_FUNCTION_BASE+0x018)
// reserved                             (USB_FUNCTION_BASE+0x01c)

#define USBF_CONFIGBUF1   		(USB_FUNCTION_BASE+0x020)
#define USBF_ENDPTBUF0    		(USB_FUNCTION_BASE+0x024)
#define USBF_ENDPTBUF1    		(USB_FUNCTION_BASE+0x028)
#define USBF_ENDPTBUF2    		(USB_FUNCTION_BASE+0x02c)

#define USBF_ENDPTBUF3    		(USB_FUNCTION_BASE+0x030)
#define USBF_STRINGBUF0   		(USB_FUNCTION_BASE+0x034)
#define USBF_STRINGBUF1   		(USB_FUNCTION_BASE+0x038)
#define USBF_STRINGBUF2   		(USB_FUNCTION_BASE+0x03c)

#define USBF_STRINGBUF3   		(USB_FUNCTION_BASE+0x040)
#define USBF_STRINGBUF4   		(USB_FUNCTION_BASE+0x044)
#define USBF_F0BCNT       		(USB_FUNCTION_BASE+0x048)
#define USBF_F1BCNT       		(USB_FUNCTION_BASE+0x04c)

#define USBF_F1TOUT       		(USB_FUNCTION_BASE+0x050)
#define USBF_F2BCNT       		(USB_FUNCTION_BASE+0x054)
#define USBF_F3BCNT       		(USB_FUNCTION_BASE+0x058)

#define USBF_F2TAIL       		(USB_FUNCTION_BASE+0x05c)


#define USBF_DESCRIPTORS  		(USB_FUNCTION_BASE+0x100)
#endif
#define USBF_DESCRIPTORS_MAX            168

#if 0
#define USBF_FIFO0        		(USB_FUNCTION_BASE+0x070)
#define USBF_FIFO1        		(USB_FUNCTION_BASE+0x080)
#define USBF_FIFO2        		(USB_FUNCTION_BASE+0x0a0)
#define USBF_FIFO3        		(USB_FUNCTION_BASE+0x0c0)
#endif

#define USBF_FIFOS     		        (USBF_DESCRIPTORS+USBF_DESCRIPTORS_MAX)

#define USBF_RFIFO0    		        (USBF_DESCRIPTORS+USBF_DESCRIPTORS_MAX+8)
#define USBF_RFIFO1    		        (USBF_DESCRIPTORS+USBF_DESCRIPTORS_MAX+8+8)
#define USBF_RFIFO2    		        (USBF_DESCRIPTORS+USBF_DESCRIPTORS_MAX+8+8+32)
#define USBF_RFIFO3    		        (USBF_DESCRIPTORS+USBF_DESCRIPTORS_MAX+8+8+32+32)


#define IO_USBF_REVISION     		__IOL(USB_FUNCTION_BASE+0x000)
#define IO_USBF_CONTROL      		__IOL(USB_FUNCTION_BASE+0x004)
#define IO_USBF_STATUS       		__IOL(USB_FUNCTION_BASE+0x008)
#define IO_USBF_RAWSTATUS    		__IOL(USB_FUNCTION_BASE+0x00c)

#define IO_USBF_INTENA       		__IOL(USB_FUNCTION_BASE+0x010)
#define IO_USBF_INTDIS       		__IOL(USB_FUNCTION_BASE+0x014)
#define IO_USBF_INTCLR       		__IOL(USB_FUNCTION_BASE+0x018)
// reserved                             __IOL(USB_FUNCTION_BASE+0x01c)

#define IO_USBF_CONFIGBUF1   		__IOL(USB_FUNCTION_BASE+0x020)
#define IO_USBF_ENDPTBUF0    		__IOL(USB_FUNCTION_BASE+0x024)
#define IO_USBF_ENDPTBUF1    		__IOL(USB_FUNCTION_BASE+0x028)
#define IO_USBF_ENDPTBUF2    		__IOL(USB_FUNCTION_BASE+0x02c)

#define IO_USBF_ENDPTBUF3    		__IOL(USB_FUNCTION_BASE+0x030)
#define IO_USBF_STRINGBUF0   		__IOL(USB_FUNCTION_BASE+0x034)
#define IO_USBF_STRINGBUF1   		__IOL(USB_FUNCTION_BASE+0x038)
#define IO_USBF_STRINGBUF2   		__IOL(USB_FUNCTION_BASE+0x03c)

#define IO_USBF_STRINGBUF3   		__IOL(USB_FUNCTION_BASE+0x040)
#define IO_USBF_STRINGBUF4   		__IOL(USB_FUNCTION_BASE+0x044)
#define IO_USBF_F0BCNT       		__IOL(USB_FUNCTION_BASE+0x048)
#define IO_USBF_F1BCNT       		__IOL(USB_FUNCTION_BASE+0x04c)

#define IO_USBF_F1TOUT       		__IOL(USB_FUNCTION_BASE+0x050)
#define IO_USBF_F2BCNT       		__IOL(USB_FUNCTION_BASE+0x054)
#define IO_USBF_F3BCNT       		__IOL(USB_FUNCTION_BASE+0x058)

#define IO_USBF_F2TAIL       		__IOL(USB_FUNCTION_BASE+0x05c)
#define IO_USBF_F2TAIL_1       		__IOL(USB_FUNCTION_BASE+0x05c)
#define IO_USBF_F2TAIL_2     		__IOL(USB_FUNCTION_BASE+0x060)
#define IO_USBF_F2TAIL_3     		__IOL(USB_FUNCTION_BASE+0x064)

#define IO_USBF_FIFO0        		__IOL(USB_FUNCTION_BASE+0x070)
#define IO_USBF_FIFO1        		__IOL(USB_FUNCTION_BASE+0x080)
#define IO_USBF_FIFO2        		__IOL(USB_FUNCTION_BASE+0x0a0)
#define IO_USBF_FIFO3        		__IOL(USB_FUNCTION_BASE+0x0c0)

#define IO_USBF_DESCRIPTORS  		__IOA(USB_FUNCTION_BASE+0x100)

#define IO_USBF_FIFOS     		__IOA(USB_FUNCTION_BASE+0x100+USBF_DESCRIPTORS_MAX)
#define IO_USBF_FIFO_RX   		__IOA(USB_FUNCTION_BASE+0x100+USBF_DESCRIPTORS_MAX+16)
#define IO_USBF_FIFO_TX   		__IOA(USB_FUNCTION_BASE+0x100+USBF_DESCRIPTORS_MAX+48)


/*
 * Revision Register - USBF_REVISION
 */

#define USBF_REVISION_11                0x3

/*
 * Control Register - USBF_CONTROL
 */

#define USBF_CONTROL_ENBL               0x00000001
#define USBF_CONTROL_INTD               0x00000002
#define USBF_CONTROL_FRST               0x00000004
#define USBF_CONTROL_RWAK               0x00000008

#define USBF_CONTROL_F0CLR              0x00000010
#define USBF_CONTROL_F1CLR              0x00000020
#define USBF_CONTROL_F2CLR              0x00000040
#define USBF_CONTROL_F3CLR              0x00000080

#define USBF_CONTROL_F1MOD              0x00000300

#define USBF_CONTROL_F1MOD_DMA          0x00000100
#define USBF_CONTROL_F1MOD_IO           0x00000200

#define USBF_CONTROL_F2MOD              0x00000c00

#define USBF_CONTROL_F2MOD_DMA          0x00000400
#define USBF_CONTROL_F2MOD_IO           0x00000800

#define USBF_CONTROL_STAL0              0x00001000
#define USBF_CONTROL_STAL1              0x00002000
#define USBF_CONTROL_STAL2              0x00004000
#define USBF_CONTROL_STAL3              0x00008000

#define USBF_CONTROL_F0L                0x00010000
#define USBF_CONTROL_F2L                0x00020000
#define USBF_CONTROL_F3L                0x00040000



/*
 * Status Register - USBF_STATUS
 */

#define USBF_STATUS_SUS                 0x00000001
#define USBF_STATUS_F0ERR               0x00000002
#define USBF_STATUS_F1OR                0x00000004
#define USBF_STATUS_F1ERR               0x00000008

#define USBF_STATUS_F2UR                0x00000010
#define USBF_STATUS_F2ERR               0x00000020
#define USBF_STATUS_F3ERR               0x00000040
#define USBF_STATUS_GRSM                0x00000080

#define USBF_STATUS_F0RQ                0x00000100
#define USBF_STATUS_F1RQ                0x00000200
#define USBF_STATUS_F2RQ                0x00000400
#define USBF_STATUS_F3RQ                0x00000800

#define USBF_STATUS_SOF                 0x00001000
#define USBF_STATUS_HRST                0x00002000
#define USBF_STATUS_F1NE                0x00004000
#define USBF_STATUS_F2NF                0x00008000

#define USBF_STATUS_F2NE                0x00010000
#define USBF_STATUS_F2BSY               0x00020000
#define USBF_STATUS_VCCMD               0x00040000


/*
 * Raw Status Register - USBF_RAWSTATUS
 */

#define USBF_RAWSTATUS_RSUS             0x00000001
#define USBF_RAWSTATUS_RF0ERR           0x00000002
#define USBF_RAWSTATUS_RF1OR            0x00000004
#define USBF_RAWSTATUS_RF1ERR           0x00000008

#define USBF_RAWSTATUS_RF2UR            0x00000010
#define USBF_RAWSTATUS_RF2ERR           0x00000020
#define USBF_RAWSTATUS_RF3ERR           0x00000040
#define USBF_RAWSTATUS_RGRSM            0x00000080

#define USBF_RAWSTATUS_RF0RQ            0x00000100
#define USBF_RAWSTATUS_RF1RQ            0x00000200
#define USBF_RAWSTATUS_RF2RQ            0x00000400
#define USBF_RAWSTATUS_RF3RQ            0x00000800

#define USBF_RAWSTATUS_RSOF             0x00001000
#define USBF_RAWSTATUS_RHRST            0x00002000


/*
 * Interrupt Enable Register - USBF_INTENA
 */

#define USBF_INTENA_ENSUS               0x00000001
#define USBF_INTENA_ENF0ERR             0x00000002
#define USBF_INTENA_ENF1OR              0x00000004
#define USBF_INTENA_ENF1ERR             0x00000008

#define USBF_INTENA_ENF2UR              0x00000010
#define USBF_INTENA_ENF2ERR             0x00000020
#define USBF_INTENA_ENF3ERR             0x00000040
#define USBF_INTENA_ENGRSM              0x00000080

#define USBF_INTENA_ENF0RQ              0x00000100
#define USBF_INTENA_ENF1RQ              0x00000200
#define USBF_INTENA_ENF2RQ              0x00000400
#define USBF_INTENA_ENF3RQ              0x00000800

#define USBF_INTENA_ENSOF               0x00001000
#define USBF_INTENA_ENHRST              0x00002000

/*
 * Interrupt Disable Register - USBF_INTDIS
 */

#define USBF_INTDIS_DNSUS               0x00000001
#define USBF_INTDIS_DNF0ERR             0x00000002
#define USBF_INTDIS_DNF1OR              0x00000004
#define USBF_INTDIS_DNF1ERR             0x00000008

#define USBF_INTDIS_DNF2UR              0x00000010
#define USBF_INTDIS_DNF2ERR             0x00000020
#define USBF_INTDIS_DNF3ERR             0x00000040
#define USBF_INTDIS_DNGRSM              0x00000080

#define USBF_INTDIS_DNF0RQ              0x00000100
#define USBF_INTDIS_DNF1RQ              0x00000200
#define USBF_INTDIS_DNF2RQ              0x00000400
#define USBF_INTDIS_DNF3RQ              0x00000800

#define USBF_INTDIS_DNSOF               0x00001000
#define USBF_INTDIS_DNHRST              0x00002000

/*
 * Interrupt Clear Register - USBF_INTCLR
 */

#define USBF_INTCLR_CSUS                0x00000001
#define USBF_INTCLR_CF0ERR              0x00000002
#define USBF_INTCLR_CF1OR               0x00000004
#define USBF_INTCLR_CF1ERR              0x00000008

#define USBF_INTCLR_CF2UR               0x00000010
#define USBF_INTCLR_CF2ERR              0x00000020
#define USBF_INTCLR_CF3ERR              0x00000040
#define USBF_INTCLR_CGRSM               0x00000080

#define USBF_INTCLR_CF0RQ               0x00000100
#define USBF_INTCLR_CF1RQ               0x00000200
#define USBF_INTCLR_CF2RQ               0x00000400
#define USBF_INTCLR_CF3RQ               0x00000800

#define USBF_INTCLR_CSOF                0x00001000
#define USBF_INTCLR_CHRST               0x00002000

/*
 * Endpoint 0 Buffer Register - USBF_ENDPTBUF0
 */

#define USBF_ENDPTBUF0_EP0MSIZE         0x00000001
#define USBF_ENDPTBUF0_EP0MSIZE_8       0x00000000
#define USBF_ENDPTBUF0_EP0MSIZE_16      0x00000001

/*
 * Endpoint 1 Buffer Register - USBF_ENDPTBUF1
 */

#define USBF_ENDPTBUF1_EP1MSIZE         0x000003ff

#define USBF_ENDPTBUF1_EP1TYPE          0x00000c00
#define USBF_ENDPTBUF1_EP1TYPE_BULK     0x00000800

#define USBF_ENDPTBUF1_EP1ASET          0x00003000
#define USBF_ENDPTBUF1_EP1XFCE          0x0000c000

/*
 * Endpoint 2 Buffer Register - USBF_ENDPTBUF2
 */

#define USBF_ENDPTBUF2_EP2MSIZE         0x000003ff

#define USBF_ENDPTBUF2_EP2TYPE          0x00000c00
#define USBF_ENDPTBUF2_EP2TYPE_BULK     0x00000800

#define USBF_ENDPTBUF2_EP2ASET          0x00003000
#define USBF_ENDPTBUF2_EP2XFCE          0x0000c000

/*
 * Endpoint 3 Buffer Register - USBF_ENDPTBUF3
 */

#define USBF_ENDPTBUF3_EP3TYPE          0x00000003
#define USBF_ENDPTBUF3_EP3TYPE_INT      0x00000003

#define USBF_ENDPTBUF3_EP3ASET          0x0000000c
#define USBF_ENDPTBUF3_EP3XFCE          0x00000030

/*
 * String 0 Register - USBF_STRINGBUF0
 */

#define USBF_STRINGBUF0_ST0ADR          0x000001ff
#define USBF_STRINGBUF0_ST0LNTH         0x0001fe00

/*
 * String 1 Register - USBF_STRINGBUF1
 */

#define USBF_STRINGBUF1_ST1ADR          0x000001ff
#define USBF_STRINGBUF1_ST1LNTH         0x0001fe00

/*
 * String 2 Register - USBF_STRINGBUF2
 */

#define USBF_STRINGBUF2_ST2ADR          0x000001ff
#define USBF_STRINGBUF2_ST2LNTH         0x0001fe00

/*
 * String 3 Register - USBF_STRINGBUF3
 */

#define USBF_STRINGBUF3_ST3ADR          0x000001ff
#define USBF_STRINGBUF3_ST3LNTH         0x0001fe00

/*
 * String 4 Register - USBF_STRINGBUF4
 */

#define USBF_STRINGBUF4_ST4ADR          0x000001ff
#define USBF_STRINGBUF4_ST4LNTH         0x0001fe00

/*
 * FIFO0 Byte Count Register - USBF_F0BCNT
 */

#define USBF_F0BCNT_F0BCNT              0x0000001f

/*
 * FIFO1 Byte Count Register - USBF_F1BCNT
 */

#define USBF_F1BCNT_F1BCNT              0x0000003f

/*
 * FIFO1 Time Out Register - USBF_F1TOUT
 */

#define USBF_F1TOUT_F1TOUT              0x0000000f

/*
 * FIFO2 Byte Count Register - USBF_F2BCNT
 */

#define USBF_F2BCNT_F2BCNT              0x0000003f

/*
 * FIFO3 Byte Count Register - USBF_F3BCNT
 */

#define USBF_F3BCNT_F3BCNT              0x0000003f




/*
         
                0       0000 0001
                1       0000 0002
                2       0000 0004
                3       0000 0008

                4       0000 0010
                5       0000 0020
                6       0000 0040
                7       0000 0080

                8       0000 0100
                9       0000 0200
                10      0000 0400
                11      0000 0800

                12      0000 1000
                13      0000 2000
                14      0000 4000
                15      0000 8000

                16      0001 0000
                17      0002 0000
                18      0004 0000
                19      0008 0000

                20      0010 0000
                21      0020 0000
                22      0040 0000
                23      0080 0000

                24      0100 0000
                25      0200 0000
                26      0400 0000
                27      0800 0000

                28      1000 0000
                29      2000 0000
                30      4000 0000
                31      8000 0000


                0 0000  4 0100  8 1000  c 1100
                1 0001  5 0101  9 1001  d 1101
                2 0010  6 0110  a 1010  e 1110
                3 0100  7 0111  b 1011  f 1111



              0000 0000 0000 0000 0000 0000 0000 0000
                 |    |    |    |    |    |    |    |
                28   24   20   16   12    8    4    0

*/
