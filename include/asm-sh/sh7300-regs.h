/*
 * include/asm/sh7300-regs.h
 *
 * sh7300 specific regs.
 *
 * Author: Dmitrij Frasenyak <sed@ru.mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __INCLUDE_ASM_SH7300_REGS_H__
#define __INCLUDE_ASM_SH7300_REGS_H__


/* TIMER */
#define TMU_TSTR        0xA412FE92      /* Byte access */

#define TMU0_TCOR       0xA412FE94      /* Long access */
#define TMU0_TCNT       0xA412FE98      /* Long access */
#define TMU0_TCR        0xA412FE9C      /* Word access */

#define TMU1_TCOR	0xA412FEA0	/* Long access */
#define TMU1_TCNT	0xA412FEA4	/* Long access */
#define TMU1_TCR	0xA412FEA8	/* Word access */

#define TMU2_TCOR       0xA412FEAC      /* Long access */
#define TMU2_TCNT       0xA412FEB0      /* Long access */
#define TMU2_TCR        0xA412FEB4      /* Word access */

/* Clock Pulse Generator aka CPG */
#define FRQCR           0xA415FF80      /* Word access */
#define FCLKCR          0xA40A0018      /* 8bit register;Word access*/
#define SCLKCR          0xA40A0014      /* 8bit register;Word access*/
#define VCLKCR          0xA40A0020      /* 8bit register;Word access*/

/* Standby control registers. TBD */ 
#define STBCR           0xA415FF82
#define STBCR2          0xA415FF88
#define STBCR3          0xA40A0000
#define STBCR4          0xA40A0004
#define STBCR5          0xA40A0010

#define STBCR_STBY      0x80              /*1 << 7*/
/*BSC*/

#define BSC_SDCR        0xA4FD0044 /* SDRAM Control Register */
#define SDCR_SMODE	0x0400
#define SDCR_AMODE	0xFBFF

/*WDT*/

#define WTCSR           0xA415FF86
#define WTCSR_MASK      0xA500
#define WTCNT           0xA415FF84
#define WTCNT_MASK      0x5A00

/* DMA */
#define DMAC_BASE       0xa4010000
#define DMAC_DMAOR      0xa4010060

#endif /*__INCLUDE_ASM_SH7300_REGS_H__*/
