/*
 * include/asm/sh73180-regs.h
 *
 * sh73180 specific regs.
 *
 * modified from include/asm/sh7300-regs.h by Maksim Syrchin <msyrchin@ru.mvista.com>
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __INCLUDE_ASM_SH73180_REGS_H__
#define __INCLUDE_ASM_SH73180_REGS_H__


/* TIMER */
#define TMU_TOCR        0xffd80000      
#define TMU_TSTR        0xffd80004      
                                                                               
#define TMU0_TCOR       0xffd80008      
#define TMU0_TCNT       0xffd8000c      
#define TMU0_TCR        0xffd80010      
                                                                                
#define TMU1_TCOR       0xffd80014      
#define TMU1_TCNT       0xffd80018      
#define TMU1_TCR        0xffd8001c      
                                                                                
#define TMU2_TCOR       0xffd80020      
#define TMU2_TCNT       0xffd80024      
#define TMU2_TCR        0xffd80028      

/* Clock Pulse Generator aka CPG */
#define FRQCR           0xA4150000      
#define DLLFRQ           0xA4150050
#define SCLKCR          0xA4150008    
#define VCLKCR          0xA4150004   

/* Standby control registers. TBD */ 
#define STBCR           0xA4150020
#define BAR             0xa4150040            
#define STBCR_STBY      (1<<7)
#define STBCR_RTBY      (1<<5)
#define STBCR_UTBY      (1<<4)


/*BSC*/

#define BSC_SDCR        0xFE400008 /* SDRAM Control Register */
#define SDCR_SMODE	0x00000400
#define SDCR_AMODE	0xFFFFFBFF

/*WDT*/

#define WTCSR           0xA4520004
#define WTCSR_MASK      0xA500
#define WTCNT           0xA4520000
#define WTCNT_MASK      0x5A00

/* DMA */
#define DMAC_DMAOR      0xFE008060

#define CCR		0xff00001c	/* Address of Cache Control Register */

#define INTC_INTPRI0	0xa4140010

#define	MSTPCR0	        (0xa4150030)
#define	MSTPCR2	        (0xa4150038)

#define MSTP204	        (1<<4)
#define MSTP202         (1<<2)

#define HIZCRA_PTA      (unsigned short)(~(1<<14))
#define PACR_PTA	(unsigned short)0x0000
#define PECR_PTE	(unsigned short)0x003f
#define PSELD_PTE3	(unsigned short)(~(1<<6))
#define DRVCR1_VIO	(unsigned short)(1<<0)	/* low level drive */

#endif /*__INCLUDE_ASM_SH73180_REGS_H__*/
