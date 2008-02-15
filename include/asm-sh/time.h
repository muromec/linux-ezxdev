/*
 * include/asm/time.h
 *
 * TMU regs.
 * copy-pasted from arch/sh/time.c 
 *
 * Author: Dmitrij Frasenyak <sed@ru.mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */


#ifndef __INCLUDE_ASM_TIME_H__
#define __INCLUDE_ASM_TIME_H__

#define TMU_TOCR_INIT	0x00	/* Don't output RTC clock */

#define TMU0_TCR_INIT	0x0020	/* Clock/4, rising edge; interrupt on */
#define TMU0_TCR_CALIB	0x0000	/* Clock/4, rising edge; no interrupt */
#define TMU0_TSTR_INIT	0x01	/* Bit to turn on TMU0 */

#define TMU1_TCR_INIT	0x0000	/* Clock/4, rising edge; no interrupt */
#define TMU1_TSTR_INIT  0x02	/* Bit to turn on TMU1 */

#define TMU2_TCR_INIT	0x0020	/* Clock/4, rising edge; interrupt on */
#define TMU2_TSTR_INIT  0x04	/* Bit to turn on TMU2 */

#if defined(__sh3__)
#if defined(CONFIG_CPU_SUBTYPE_SH7300)
#include <asm/sh7300-regs.h>
#else
#define TMU_TOCR	0xfffffe90	/* Byte access */
#define TMU_TSTR	0xfffffe92	/* Byte access */

#define TMU0_TCOR	0xfffffe94	/* Long access */
#define TMU0_TCNT	0xfffffe98	/* Long access */
#define TMU0_TCR	0xfffffe9c	/* Word access */

#define TMU1_TCOR	0xfffffea0	/* Long access */
#define TMU1_TCNT	0xfffffea4	/* Long access */
#define TMU1_TCR	0xfffffea8	/* Word access */

#define TMU2_TCOR	0xfffffeac	/* Long access */
#define TMU2_TCNT	0xfffffeb0	/* Long access */
#define TMU2_TCR	0xfffffeb4	/* Word access */

#define FRQCR		0xffffff80
#endif
#elif defined(__SH4__)
#define TMU_TOCR	0xffd80000	/* Byte access */
#define TMU_TSTR	0xffd80004	/* Byte access */

#define TMU0_TCOR	0xffd80008	/* Long access */
#define TMU0_TCNT	0xffd8000c	/* Long access */
#define TMU0_TCR	0xffd80010	/* Word access */

#define TMU1_TCOR	0xffd80014	/* Long access */
#define TMU1_TCNT	0xffd80018	/* Long access */
#define TMU1_TCR	0xffd8001c	/* Word access */

#define TMU2_TCOR	0xffd80020	/* Long access */
#define TMU2_TCNT	0xffd80024	/* Long access */
#define TMU2_TCR	0xffd80028	/* Word access */

#if defined(CONFIG_CPU_SUBTYPE_SH73180)
#include <asm/sh73180-regs.h>
#else
#define FRQCR		0xffc00000
#endif

/* Core Processor Version Register */
#define CCN_PVR		0xff000030
#define CCN_PVR_CHIP_SHIFT 24
#define CCN_PVR_CHIP_MASK  0xff
#define CCN_PVR_CHIP_ST40STB1 0x4

#ifdef CONFIG_CPU_SUBTYPE_ST40STB1
#define CLOCKGEN_MEMCLKCR 0xbb040038
#define MEMCLKCR_RATIO_MASK 0x7
#endif /* CONFIG_CPU_SUBTYPE_ST40STB1 */
#endif /* __sh3__ or __SH4__ */

#endif
