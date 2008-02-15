/* 
 * xtensa/config/core.h -- HAL definitions that are dependent on CORE configuration
 *
 *  This header file is sometimes referred to as the "compile-time HAL" or CHAL.
 *  It was generated for a specific Xtensa processor configuration.
 *
 *  Source for configuration-independent binaries (which link in a
 *  configuration-specific HAL library) must NEVER include this file.
 *  It is perfectly normal, however, for the HAL source itself to include this file.
 */

/*
 * Customer ID=40; Build=11206; Copyright (c) 2002 by Tensilica Inc.  ALL RIGHTS RESERVED.
 * These coded instructions, statements, and computer programs are the
 * copyrighted works and confidential proprietary information of Tensilica Inc.
 * They may not be modified, copied, reproduced, distributed, or disclosed to
 * third parties in any manner, medium, or form, in whole or in part, without
 * the prior written consent of Tensilica Inc.
 */


#ifndef XTENSA_CONFIG_CORE_H
#define XTENSA_CONFIG_CORE_H

#include <xtensa/hal.h>


/*----------------------------------------------------------------------
				GENERAL
  ----------------------------------------------------------------------*/

/*
 *  Separators for macros that expand into arrays.
 *  These can be predefined by files that #include this one,
 *  when different separators are required.
 */
/*  Element separator for macros that expand into 1-dimensional arrays:  */
#ifndef XCHAL_SEP
#define XCHAL_SEP			,
#endif
/*  Array separator for macros that expand into 2-dimensional arrays:  */
#ifndef XCHAL_SEP2
#define XCHAL_SEP2			},{
#endif


/*----------------------------------------------------------------------
				ENDIANNESS
  ----------------------------------------------------------------------*/

#define XCHAL_HAVE_BE			0
#define XCHAL_HAVE_LE			1
#define XCHAL_MEMORY_ORDER		XTHAL_LITTLEENDIAN


/*----------------------------------------------------------------------
				REGISTER WINDOWS
  ----------------------------------------------------------------------*/

#define XCHAL_HAVE_WINDOWED		1	/* 1 if windowed registers option configured, 0 otherwise */
#define XCHAL_NUM_AREGS			32	/* number of physical address regs */
#define XCHAL_NUM_AREGS_LOG2		5	/* log2(XCHAL_NUM_AREGS) */


/*----------------------------------------------------------------------
				INTERRUPTS
  ----------------------------------------------------------------------*/

#define XCHAL_HAVE_INTERRUPTS		1	/* 1 if interrupt option configured, 0 otherwise */
#define XCHAL_HAVE_HIGHLEVEL_INTERRUPTS	1	/* 1 if high-level interrupt option configured, 0 otherwise */
#define XCHAL_HAVE_NMI			1	/* 1 if NMI option configured, 0 otherwise */
#define XCHAL_NUM_INTERRUPTS		32	/* number of interrupts */
#define XCHAL_NUM_EXTINTERRUPTS		10	/* number of external interrupts */
#define XCHAL_NUM_INTLEVELS		6	/* number of interrupt levels (not including level zero!) */

/*  Masks of interrupts at each interrupt level:  */
#define XCHAL_INTLEVEL0_MASK		0x00000000
#define XCHAL_INTLEVEL1_MASK		0x0C44F022
#define XCHAL_INTLEVEL2_MASK		0x120A0050
#define XCHAL_INTLEVEL3_MASK		0x00000204
#define XCHAL_INTLEVEL4_MASK		0x81B00408
#define XCHAL_INTLEVEL5_MASK		0x60010901
#define XCHAL_INTLEVEL6_MASK		0x00000000
#define XCHAL_INTLEVEL7_MASK		0x00000080
#define XCHAL_INTLEVEL8_MASK		0x00000000
#define XCHAL_INTLEVEL9_MASK		0x00000000
#define XCHAL_INTLEVEL10_MASK		0x00000000
#define XCHAL_INTLEVEL11_MASK		0x00000000
#define XCHAL_INTLEVEL12_MASK		0x00000000
#define XCHAL_INTLEVEL13_MASK		0x00000000
#define XCHAL_INTLEVEL14_MASK		0x00000000
#define XCHAL_INTLEVEL15_MASK		0x00000000
/*  As an array of entries (eg. for C constant arrays):  */
#define XCHAL_INTLEVEL_MASKS		0x00000000	XCHAL_SEP \
					0x0C44F022	XCHAL_SEP \
					0x120A0050	XCHAL_SEP \
					0x00000204	XCHAL_SEP \
					0x81B00408	XCHAL_SEP \
					0x60010901	XCHAL_SEP \
					0x00000000	XCHAL_SEP \
					0x00000080	XCHAL_SEP \
					0x00000000	XCHAL_SEP \
					0x00000000	XCHAL_SEP \
					0x00000000	XCHAL_SEP \
					0x00000000	XCHAL_SEP \
					0x00000000	XCHAL_SEP \
					0x00000000	XCHAL_SEP \
					0x00000000	XCHAL_SEP \
					0x00000000

/*  Masks of interrupts at each range 1..n of interrupt levels:  */
#define XCHAL_INTLEVEL0_ANDBELOW_MASK	0x00000000
#define XCHAL_INTLEVEL1_ANDBELOW_MASK	0x0C44F022
#define XCHAL_INTLEVEL2_ANDBELOW_MASK	0x1E4EF072
#define XCHAL_INTLEVEL3_ANDBELOW_MASK	0x1E4EF276
#define XCHAL_INTLEVEL4_ANDBELOW_MASK	0x9FFEF67E
#define XCHAL_INTLEVEL5_ANDBELOW_MASK	0xFFFFFF7F
#define XCHAL_INTLEVEL6_ANDBELOW_MASK	0xFFFFFF7F
#define XCHAL_INTLEVEL7_ANDBELOW_MASK	0xFFFFFFFF
#define XCHAL_INTLEVEL8_ANDBELOW_MASK	0xFFFFFFFF
#define XCHAL_INTLEVEL9_ANDBELOW_MASK	0xFFFFFFFF
#define XCHAL_INTLEVEL10_ANDBELOW_MASK	0xFFFFFFFF
#define XCHAL_INTLEVEL11_ANDBELOW_MASK	0xFFFFFFFF
#define XCHAL_INTLEVEL12_ANDBELOW_MASK	0xFFFFFFFF
#define XCHAL_INTLEVEL13_ANDBELOW_MASK	0xFFFFFFFF
#define XCHAL_INTLEVEL14_ANDBELOW_MASK	0xFFFFFFFF
#define XCHAL_INTLEVEL15_ANDBELOW_MASK	0xFFFFFFFF
/*  As an array of entries (eg. for C constant arrays):  */
#define XCHAL_INTLEVEL_ANDBELOW_MASKS	0x00000000	XCHAL_SEP \
					0x0C44F022	XCHAL_SEP \
					0x1E4EF072	XCHAL_SEP \
					0x1E4EF276	XCHAL_SEP \
					0x9FFEF67E	XCHAL_SEP \
					0xFFFFFF7F	XCHAL_SEP \
					0xFFFFFF7F	XCHAL_SEP \
					0xFFFFFFFF	XCHAL_SEP \
					0xFFFFFFFF	XCHAL_SEP \
					0xFFFFFFFF	XCHAL_SEP \
					0xFFFFFFFF	XCHAL_SEP \
					0xFFFFFFFF	XCHAL_SEP \
					0xFFFFFFFF	XCHAL_SEP \
					0xFFFFFFFF	XCHAL_SEP \
					0xFFFFFFFF	XCHAL_SEP \
					0xFFFFFFFF

/*  Level of each interrupt:  */
#define XCHAL_INT0_LEVEL		5
#define XCHAL_INT1_LEVEL		1
#define XCHAL_INT2_LEVEL		3
#define XCHAL_INT3_LEVEL		4
#define XCHAL_INT4_LEVEL		2
#define XCHAL_INT5_LEVEL		1
#define XCHAL_INT6_LEVEL		2
#define XCHAL_INT7_LEVEL		7
#define XCHAL_INT8_LEVEL		5
#define XCHAL_INT9_LEVEL		3
#define XCHAL_INT10_LEVEL		4
#define XCHAL_INT11_LEVEL		5
#define XCHAL_INT12_LEVEL		1
#define XCHAL_INT13_LEVEL		1
#define XCHAL_INT14_LEVEL		1
#define XCHAL_INT15_LEVEL		1
#define XCHAL_INT16_LEVEL		5
#define XCHAL_INT17_LEVEL		2
#define XCHAL_INT18_LEVEL		1
#define XCHAL_INT19_LEVEL		2
#define XCHAL_INT20_LEVEL		4
#define XCHAL_INT21_LEVEL		4
#define XCHAL_INT22_LEVEL		1
#define XCHAL_INT23_LEVEL		4
#define XCHAL_INT24_LEVEL		4
#define XCHAL_INT25_LEVEL		2
#define XCHAL_INT26_LEVEL		1
#define XCHAL_INT27_LEVEL		1
#define XCHAL_INT28_LEVEL		2
#define XCHAL_INT29_LEVEL		5
#define XCHAL_INT30_LEVEL		5
#define XCHAL_INT31_LEVEL		4
#define XCHAL_NMILEVEL			7	/* NMI "interrupt level" (for use with EXCSAVE_n, EPS_n, EPC_n, RFI n) */
/*  As an array of entries (eg. for C constant arrays):  */
#define XCHAL_INT_LEVELS		5	XCHAL_SEP \
					1	XCHAL_SEP \
					3	XCHAL_SEP \
					4	XCHAL_SEP \
					2	XCHAL_SEP \
					1	XCHAL_SEP \
					2	XCHAL_SEP \
					7	XCHAL_SEP \
					5	XCHAL_SEP \
					3	XCHAL_SEP \
					4	XCHAL_SEP \
					5	XCHAL_SEP \
					1	XCHAL_SEP \
					1	XCHAL_SEP \
					1	XCHAL_SEP \
					1	XCHAL_SEP \
					5	XCHAL_SEP \
					2	XCHAL_SEP \
					1	XCHAL_SEP \
					2	XCHAL_SEP \
					4	XCHAL_SEP \
					4	XCHAL_SEP \
					1	XCHAL_SEP \
					4	XCHAL_SEP \
					4	XCHAL_SEP \
					2	XCHAL_SEP \
					1	XCHAL_SEP \
					1	XCHAL_SEP \
					2	XCHAL_SEP \
					5	XCHAL_SEP \
					5	XCHAL_SEP \
					4

/*  Type of each interrupt:  */
#define XCHAL_INT0_TYPE 	XTHAL_INTTYPE_EXTERN_LEVEL
#define XCHAL_INT1_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT2_TYPE 	XTHAL_INTTYPE_TIMER
#define XCHAL_INT3_TYPE 	XTHAL_INTTYPE_TIMER
#define XCHAL_INT4_TYPE 	XTHAL_INTTYPE_EXTERN_EDGE
#define XCHAL_INT5_TYPE 	XTHAL_INTTYPE_TIMER
#define XCHAL_INT6_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT7_TYPE 	XTHAL_INTTYPE_NMI
#define XCHAL_INT8_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT9_TYPE 	XTHAL_INTTYPE_EXTERN_LEVEL
#define XCHAL_INT10_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT11_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT12_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT13_TYPE 	XTHAL_INTTYPE_EXTERN_LEVEL
#define XCHAL_INT14_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT15_TYPE 	XTHAL_INTTYPE_EXTERN_LEVEL
#define XCHAL_INT16_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT17_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT18_TYPE 	XTHAL_INTTYPE_EXTERN_LEVEL
#define XCHAL_INT19_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT20_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT21_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT22_TYPE 	XTHAL_INTTYPE_EXTERN_LEVEL
#define XCHAL_INT23_TYPE 	XTHAL_INTTYPE_EXTERN_LEVEL
#define XCHAL_INT24_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT25_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT26_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT27_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT28_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT29_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT30_TYPE 	XTHAL_INTTYPE_EXTERN_LEVEL
#define XCHAL_INT31_TYPE 	XTHAL_INTTYPE_SOFTWARE
/*  As an array of entries (eg. for C constant arrays):  */
#define XCHAL_INT_TYPES		XTHAL_INTTYPE_EXTERN_LEVEL     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_TIMER     	XCHAL_SEP \
				XTHAL_INTTYPE_TIMER     	XCHAL_SEP \
				XTHAL_INTTYPE_EXTERN_EDGE     	XCHAL_SEP \
				XTHAL_INTTYPE_TIMER     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_NMI     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_EXTERN_LEVEL     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_EXTERN_LEVEL     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_EXTERN_LEVEL     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_EXTERN_LEVEL     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_EXTERN_LEVEL     	XCHAL_SEP \
				XTHAL_INTTYPE_EXTERN_LEVEL     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_EXTERN_LEVEL     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE

/*  Masks of interrupts for each type of interrupt:  */
#define XCHAL_INTTYPE_MASK_UNCONFIGURED	0x00000000
#define XCHAL_INTTYPE_MASK_SOFTWARE	0xBF3B5D42
#define XCHAL_INTTYPE_MASK_EXTERN_EDGE	0x00000010
#define XCHAL_INTTYPE_MASK_EXTERN_LEVEL	0x40C4A201
#define XCHAL_INTTYPE_MASK_TIMER	0x0000002C
#define XCHAL_INTTYPE_MASK_NMI		0x00000080
/*  As an array of entries (eg. for C constant arrays):  */
#define XCHAL_INTTYPE_MASKS		0x00000000	XCHAL_SEP \
					0xBF3B5D42	XCHAL_SEP \
					0x00000010	XCHAL_SEP \
					0x40C4A201	XCHAL_SEP \
					0x0000002C	XCHAL_SEP \
					0x00000080

/*  Interrupts assigned to each timer (CCOMPARE0 to CCOMPARE3), -1 if unassigned  */
#define XCHAL_TIMER0_INTERRUPT	5
#define XCHAL_TIMER1_INTERRUPT	2
#define XCHAL_TIMER2_INTERRUPT	3
#define XCHAL_TIMER3_INTERRUPT	XTHAL_TIMER_UNCONFIGURED
/*  As an array of entries (eg. for C constant arrays):  */
#define XCHAL_TIMER_INTERRUPTS	5	XCHAL_SEP \
				2	XCHAL_SEP \
				3	XCHAL_SEP \
				XTHAL_TIMER_UNCONFIGURED

/*  Interrupt assigned to NMI:  */
#define XCHAL_NMI_INTERRUPT		7	/* NMI interrupt number */

/*  Indexing macros:  */
#define _XCHAL_INTLEVEL_MASK(n)		XCHAL_INTLEVEL ## n ## _MASK
#define XCHAL_INTLEVEL_MASK(n)		_XCHAL_INTLEVEL_MASK(n)		/* n = 0 .. 15 */
#define _XCHAL_INTLEVEL_ANDBELOWMASK(n)	XCHAL_INTLEVEL ## n ## _ANDBELOW_MASK
#define XCHAL_INTLEVEL_ANDBELOW_MASK(n)	_XCHAL_INTLEVEL_ANDBELOWMASK(n)	/* n = 0 .. 15 */
#define _XCHAL_INT_LEVEL(n)		XCHAL_INT ## n ## _LEVEL
#define XCHAL_INT_LEVEL(n)		_XCHAL_INT_LEVEL(n)		/* n = 0 .. 31 */
#define _XCHAL_INT_TYPE(n)		XCHAL_INT ## n ## _TYPE
#define XCHAL_INT_TYPE(n)		_XCHAL_INT_TYPE(n)		/* n = 0 .. 31 */
#define _XCHAL_TIMER_INTERRUPT(n)	XCHAL_TIMER ## n ## _INTERRUPT
#define XCHAL_TIMER_INTERRUPT(n)	_XCHAL_TIMER_INTERRUPT(n)	/* n = 0 .. 3 */



/*  External interrupt vectors/levels:  */

/*  Core interrupt numbers mapped to each EXTERNAL interrupt number:  */
#define XCHAL_EXTINT0_NUM		0	/* (intlevel 5) */
#define XCHAL_EXTINT1_NUM		4	/* (intlevel 2) */
#define XCHAL_EXTINT2_NUM		7	/* (intlevel 7) */
#define XCHAL_EXTINT3_NUM		9	/* (intlevel 3) */
#define XCHAL_EXTINT4_NUM		13	/* (intlevel 1) */
#define XCHAL_EXTINT5_NUM		15	/* (intlevel 1) */
#define XCHAL_EXTINT6_NUM		18	/* (intlevel 1) */
#define XCHAL_EXTINT7_NUM		22	/* (intlevel 1) */
#define XCHAL_EXTINT8_NUM		23	/* (intlevel 4) */
#define XCHAL_EXTINT9_NUM		30	/* (intlevel 5) */

/*  Corresponding interrupt masks:  */
#define XCHAL_EXTINT0_MASK		0x00000001
#define XCHAL_EXTINT1_MASK		0x00000010
#define XCHAL_EXTINT2_MASK		0x00000080
#define XCHAL_EXTINT3_MASK		0x00000200
#define XCHAL_EXTINT4_MASK		0x00002000
#define XCHAL_EXTINT5_MASK		0x00008000
#define XCHAL_EXTINT6_MASK		0x00040000
#define XCHAL_EXTINT7_MASK		0x00400000
#define XCHAL_EXTINT8_MASK		0x00800000
#define XCHAL_EXTINT9_MASK		0x40000000

/*  Core config interrupt levels mapped to each external interrupt:  */
#define XCHAL_EXTINT0_LEVEL		5	/* (int number 0) */
#define XCHAL_EXTINT1_LEVEL		2	/* (int number 4) */
#define XCHAL_EXTINT2_LEVEL		7	/* (int number 7) */
#define XCHAL_EXTINT3_LEVEL		3	/* (int number 9) */
#define XCHAL_EXTINT4_LEVEL		1	/* (int number 13) */
#define XCHAL_EXTINT5_LEVEL		1	/* (int number 15) */
#define XCHAL_EXTINT6_LEVEL		1	/* (int number 18) */
#define XCHAL_EXTINT7_LEVEL		1	/* (int number 22) */
#define XCHAL_EXTINT8_LEVEL		4	/* (int number 23) */
#define XCHAL_EXTINT9_LEVEL		5	/* (int number 30) */


/*----------------------------------------------------------------------
			EXCEPTIONS and VECTORS
  ----------------------------------------------------------------------*/

#define XCHAL_HAVE_EXCEPTIONS		1	/* 1 if exception option configured, 0 otherwise */

#define XCHAL_XEA_VERSION		2	/* Xtensa Exception Architecture number: 1 for XEA1 (old), 2 for XEA2 (new) */
#define XCHAL_HAVE_XEA1			0	/* 1 if XEA1, 0 otherwise */
#define XCHAL_HAVE_XEA2			1	/* 1 if XEA2, 0 otherwise */
/*  For backward compatibility ONLY -- DO NOT USE (will be removed in future release):  */
#define XCHAL_HAVE_OLD_EXC_ARCH		XCHAL_HAVE_XEA1	/* (DEPRECATED) 1 if old exception architecture (XEA1), 0 otherwise (eg. XEA2) */
#define XCHAL_HAVE_EXCM			XCHAL_HAVE_XEA2	/* (DEPRECATED) 1 if PS.EXCM bit exists (currently equals XCHAL_HAVE_TLBS) */

#define XCHAL_RESET_VECTOR_VADDR	0xFE000020
#define XCHAL_RESET_VECTOR_PADDR	0xFE000020
#define XCHAL_USER_VECTOR_VADDR		0xD0000220
#define XCHAL_PROGRAMEXC_VECTOR_VADDR	XCHAL_USER_VECTOR_VADDR		/* for backward compatibility */
#define XCHAL_USEREXC_VECTOR_VADDR	XCHAL_USER_VECTOR_VADDR		/* for backward compatibility */
#define XCHAL_USER_VECTOR_PADDR		0x00000220
#define XCHAL_PROGRAMEXC_VECTOR_PADDR	XCHAL_USER_VECTOR_PADDR		/* for backward compatibility */
#define XCHAL_USEREXC_VECTOR_PADDR	XCHAL_USER_VECTOR_PADDR		/* for backward compatibility */
#define XCHAL_KERNEL_VECTOR_VADDR	0xD0000200
#define XCHAL_STACKEDEXC_VECTOR_VADDR	XCHAL_KERNEL_VECTOR_VADDR	/* for backward compatibility */
#define XCHAL_KERNELEXC_VECTOR_VADDR	XCHAL_KERNEL_VECTOR_VADDR	/* for backward compatibility */
#define XCHAL_KERNERL_VECTOR_PADDR	0x00000200
#define XCHAL_STACKEDEXC_VECTOR_PADDR	XCHAL_KERNEL_VECTOR_PADDR	/* for backward compatibility */
#define XCHAL_KERNELEXC_VECTOR_PADDR	XCHAL_KERNEL_VECTOR_PADDR	/* for backward compatibility */
#define XCHAL_DOUBLEEXC_VECTOR_VADDR	0xD00002B0
#define XCHAL_DOUBLEEXC_VECTOR_PADDR	0x000002B0
#define XCHAL_WINDOW_VECTORS_VADDR	0xD0000000
#define XCHAL_WINDOW_VECTORS_PADDR	0x00000000
#define XCHAL_INTLEVEL2_VECTOR_VADDR	0xD0000240
#define XCHAL_INTLEVEL2_VECTOR_PADDR	0x00000240
#define XCHAL_INTLEVEL3_VECTOR_VADDR	0xD0000250
#define XCHAL_INTLEVEL3_VECTOR_PADDR	0x00000250
#define XCHAL_INTLEVEL4_VECTOR_VADDR	0xD0000260
#define XCHAL_INTLEVEL4_VECTOR_PADDR	0x00000260
#define XCHAL_INTLEVEL5_VECTOR_VADDR	0xD0000270
#define XCHAL_INTLEVEL5_VECTOR_PADDR	0x00000270
#define XCHAL_INTLEVEL6_VECTOR_VADDR	0xFE000520
#define XCHAL_INTLEVEL6_VECTOR_PADDR	0xFE000520
#define XCHAL_DEBUG_VECTOR_VADDR	XCHAL_INTLEVEL6_VECTOR_VADDR
#define XCHAL_DEBUG_VECTOR_PADDR	XCHAL_INTLEVEL6_VECTOR_PADDR
#define XCHAL_NMI_VECTOR_VADDR		0xFE000510
#define XCHAL_NMI_VECTOR_PADDR		0xFE000510
#define XCHAL_INTLEVEL7_VECTOR_VADDR	XCHAL_NMI_VECTOR_VADDR
#define XCHAL_INTLEVEL7_VECTOR_PADDR	XCHAL_NMI_VECTOR_PADDR

/*  Indexing macros:  */
#define _XCHAL_INTLEVEL_VECTOR_VADDR(n)		XCHAL_INTLEVEL ## n ## _VECTOR_VADDR
#define XCHAL_INTLEVEL_VECTOR_VADDR(n)		_XCHAL_INTLEVEL_VECTOR_VADDR(n)		/* n = 0 .. 15 */

/*
 *  Level-1 Exception Causes:
 */
#define XCHAL_EXCCAUSE_ILLEGAL_INSTRUCTION		0	/* Illegal Instruction (IllegalInstruction) */
#define XCHAL_EXCCAUSE_SYSTEM_CALL			1	/* System Call (SystemCall) */
#define XCHAL_EXCCAUSE_INSTRUCTION_FETCH_ERROR		2	/* Instruction Fetch Error (InstructionFetchError) */
#define XCHAL_EXCCAUSE_LOAD_STORE_ERROR			3	/* Load Store Error (LoadStoreError) */
#define XCHAL_EXCCAUSE_LEVEL1_INTERRUPT			4	/* Level 1 Interrupt (Level1Interrupt) */
#define XCHAL_EXCCAUSE_ALLOCA				5	/* Stack Extension Assist (Alloca) */
#define XCHAL_EXCCAUSE_INTEGER_DIVIDE_BY_ZERO		6	/* Integer Divide by Zero (IntegerDivideByZero) */
#define XCHAL_EXCCAUSE_SPECULATION			7	/* Speculation (Speculation) */
#define XCHAL_EXCCAUSE_PRIVILEGED			8	/* Privileged Instruction (Privileged) */
#define XCHAL_EXCCAUSE_ITLB_MISS			16	/* ITlb Miss Exception (ITlbMiss) */
#define XCHAL_EXCCAUSE_ITLB_MULTIHIT			17	/* ITlb Mutltihit Exception (ITlbMultihit) */
#define XCHAL_EXCCAUSE_ITLB_PRIVILEGE			18	/* ITlb Privilege Exception (ITlbPrivilege) */
#define XCHAL_EXCCAUSE_ITLB_SIZE_RESTRICTION		19	/* ITlb Size Restriction Exception (ITlbSizeRestriction) */
#define XCHAL_EXCCAUSE_FETCH_CACHE_ATTRIBUTE		20	/* Fetch Cache Attribute Exception (FetchCacheAttribute) */
#define XCHAL_EXCCAUSE_DTLB_MISS			24	/* DTlb Miss Exception (DTlbMiss) */
#define XCHAL_EXCCAUSE_DTLB_MULTIHIT			25	/* DTlb Multihit Exception (DTlbMultihit) */
#define XCHAL_EXCCAUSE_DTLB_PRIVILEGE			26	/* DTlb Privilege Exception (DTlbPrivilege) */
#define XCHAL_EXCCAUSE_DTLB_SIZE_RESTRICTION		27	/* DTlb Size Restriction Exception (DTlbSizeRestriction) */
#define XCHAL_EXCCAUSE_LOAD_CACHE_ATTRIBUTE		28	/* Load Cache Attribute Exception (LoadCacheAttribute) */
#define XCHAL_EXCCAUSE_STORE_CACHE_ATTRIBUTE		29	/* Store Cache Attribute Exception (StoreCacheAttribute) */
#define XCHAL_EXCCAUSE_COPROCESSOR0_DISABLED		32	/* Coprocessor 0 disabled (Coprocessor0Disabled) */
#define XCHAL_EXCCAUSE_COPROCESSOR2_DISABLED		34	/* Coprocessor 2 disabled (Coprocessor2Disabled) */
#define XCHAL_EXCCAUSE_COPROCESSOR5_DISABLED		37	/* Coprocessor 5 disabled (Coprocessor5Disabled) */
#define XCHAL_EXCCAUSE_COPROCESSOR6_DISABLED		38	/* Coprocessor 6 disabled (Coprocessor6Disabled) */
#define XCHAL_EXCCAUSE_FLOATING_POINT			40	/* Floating Point Exception (FloatingPoint) */



/*----------------------------------------------------------------------
				TIMERS
  ----------------------------------------------------------------------*/

#define XCHAL_HAVE_CCOUNT		1	/* 1 if have CCOUNT, 0 otherwise */
/*#define XCHAL_HAVE_TIMERS		XCHAL_HAVE_CCOUNT*/
#define XCHAL_NUM_TIMERS		3	/* number of CCOMPAREn regs */



/*----------------------------------------------------------------------
				DEBUG
  ----------------------------------------------------------------------*/

#define XCHAL_HAVE_DEBUG		1	/* 1 if debug option configured, 0 otherwise */
#define XCHAL_HAVE_OCD			1	/* 1 if OnChipDebug option configured, 0 otherwise */
#define XCHAL_NUM_IBREAK		2	/* number of IBREAKn regs */
#define XCHAL_NUM_DBREAK		2	/* number of DBREAKn regs */
#define XCHAL_DEBUGLEVEL		6	/* debug interrupt level */
/*DebugExternalInterrupt		0		0|1*/
/*DebugUseDIRArray			0		0|1*/




/*----------------------------------------------------------------------
			COPROCESSORS and EXTRA STATE
  ----------------------------------------------------------------------*/

#define XCHAL_HAVE_CP			1	/* 1 if coprocessor option configured (CPENABLE present) */
#define XCHAL_CP_NUM			4	/* number of coprocessors */
#define XCHAL_CP_MAX			7	/* max coprocessor id plus one (0 if none) */
#define XCHAL_CP_MAXCFG			7	/* max allowed cp id plus one (per cfg) */
#define XCHAL_CP_MASK			0x65	/* bitmask of coprocessors by id */

/*  Space for coprocessors' state save areas:  */
#define XCHAL_CP0_SA_SIZE		72
#define XCHAL_CP1_SA_SIZE		0
#define XCHAL_CP2_SA_SIZE		68
#define XCHAL_CP3_SA_SIZE		0
#define XCHAL_CP4_SA_SIZE		0
#define XCHAL_CP5_SA_SIZE		136
#define XCHAL_CP6_SA_SIZE		36
#define XCHAL_CP7_SA_SIZE		0
/*  Minimum required alignments of CP state save areas:  */
#define XCHAL_CP0_SA_ALIGN		4
#define XCHAL_CP1_SA_ALIGN		1
#define XCHAL_CP2_SA_ALIGN		4
#define XCHAL_CP3_SA_ALIGN		1
#define XCHAL_CP4_SA_ALIGN		1
#define XCHAL_CP5_SA_ALIGN		8
#define XCHAL_CP6_SA_ALIGN		4
#define XCHAL_CP7_SA_ALIGN		1

/*  Indexing macros:  */
#define _XCHAL_CP_SA_SIZE(n)		XCHAL_CP ## n ## _SA_SIZE
#define XCHAL_CP_SA_SIZE(n)		_XCHAL_CP_SA_SIZE(n)	/* n = 0 .. 7 */
#define _XCHAL_CP_SA_ALIGN(n)		XCHAL_CP ## n ## _SA_ALIGN
#define XCHAL_CP_SA_ALIGN(n)		_XCHAL_CP_SA_ALIGN(n)	/* n = 0 .. 7 */


/*  Space for "extra" state (user special registers and non-cp TIE) save area:  */
#define XCHAL_EXTRA_SA_SIZE		28
#define XCHAL_EXTRA_SA_ALIGN		4

/*  Total save area size (extra + all coprocessors)  */
/*  (not useful until xthal_{save,restore}_all_extra() is implemented,  */
/*   but included for Tor2 beta; doesn't account for alignment!):  */
#define XCHAL_CPEXTRA_SA_SIZE_TOR2	340	/* Tor2Beta temporary definition -- do not use */

/*  Combined required alignment for all CP and EXTRA state save areas  */
/*  (does not include required alignment for any base config registers):  */
#define XCHAL_CPEXTRA_SA_ALIGN		8

/* ... */


#ifdef _ASMLANGUAGE
/*
 *  Assembly-language specific definitions (assembly macros, etc.).
 */
#include <xtensa/config/specreg.h>

/********************
 *  Macros to save and restore the non-coprocessor TIE portion of EXTRA state.
 */

/* (none) */


/********************
 *  Macros to create functions that save and restore all EXTRA (non-coprocessor) state
 *  (does not include zero-overhead loop registers and non-optional registers).
 */

	/*
	 *  Macro that expands to the body of a function that
	 *  stores the extra (non-coprocessor) optional/custom state.
	 *	Entry:	a2 = ptr to save area in which to save extra state
	 *	Exit:	any register a2-a15 (?) may have been clobbered.
	 */
	.macro	xchal_extra_store_funcbody
	// Start saving state for MAC16
	rsr	a3, ACCLO
	s32i	a3, a2, 0
	rsr	a3, ACCHI
	s32i	a3, a2, 4
	rsr	a3, MR_0
	s32i	a3, a2, 8
	rsr	a3, MR_1
	s32i	a3, a2, 12
	rsr	a3, MR_2
	s32i	a3, a2, 16
	rsr	a3, MR_3
	s32i	a3, a2, 20
	// End of saving state for MAC16
	// Start saving state for Booleans
	rsr	a3, BR
	s32i	a3, a2, 24
	// End of saving state for Booleans
	.endm


	/*
	 *  Macro that expands to the body of a function that
	 *  loads the extra (non-coprocessor) optional/custom state.
	 *	Entry:	a2 = ptr to save area from which to restore extra state
	 *	Exit:	any register a2-a15 (?) may have been clobbered.
	 */
	.macro	xchal_extra_load_funcbody
	// begin restore state for MAC16
	l32i	a3, a2, 0
	wsr	a3, ACCLO
	l32i	a3, a2, 4
	wsr	a3, ACCHI
	l32i	a3, a2, 8
	wsr	a3, MR_0
	l32i	a3, a2, 12
	wsr	a3, MR_1
	l32i	a3, a2, 16
	wsr	a3, MR_2
	l32i	a3, a2, 20
	wsr	a3, MR_3
	// end restore state for MAC16
	// begin restore state for Booleans
	l32i	a3, a2, 24
	wsr	a3, BR
	// end restore state for Booleans
	.endm


/********************
 *  Macros to save and restore the state of each TIE coprocessor.
 */

#define xchal_cp_cp0_store	xchal_cp0_store
#define xchal_cp_cp0_store_a2	xchal_cp0_store_a2
#define xchal_cp0_store_a2	xchal_cp0_store	a2, a3
// Name: xchal_cp0_store
// This macro saves the states of cp0
// Prototype: xchal_cp0_store a_0 a_1 
// Pointer to memory: a_0, aligned to 4 bytes
// Scratch register needed:  a_1 
// Example use:
//
// Clobbers:
//    a_0 a_1 

.macro xchal_cp0_store a_0 a_1 
rur232	\a_1
s32i	\a_1, \a_0, 0
rur233	\a_1
s32i	\a_1, \a_0, 4
addi	\a_0, \a_0, 8
ssi f0, \a_0,  0
ssi f1, \a_0,  4
ssi f2, \a_0,  8
ssi f3, \a_0,  12
ssi f4, \a_0,  16
ssi f5, \a_0,  20
ssi f6, \a_0,  24
ssi f7, \a_0,  28
ssi f8, \a_0,  32
ssi f9, \a_0,  36
ssi f10, \a_0,  40
ssi f11, \a_0,  44
ssi f12, \a_0,  48
ssi f13, \a_0,  52
ssi f14, \a_0,  56
ssi f15, \a_0,  60

.endm // xchal_cp0_store


#define xchal_cp_cp0_load	xchal_cp0_load
#define xchal_cp_cp0_load_a2	xchal_cp0_load_a2
#define xchal_cp0_load_a2	xchal_cp0_load	a2, a3
// Name: xchal_cp0_load
// This macro restores the states of cp0
// Prototype: xchal_cp0_load a_0 a_1 
// Pointer to memory: a_0, aligned to 4 bytes
// Scratch register needed:  a_1 
// Example use:
//
// Clobbers:
//    a_0 a_1 

.macro xchal_cp0_load a_0 a_1 
l32i	\a_1, \a_0, 0
wur232	\a_1
l32i	\a_1, \a_0, 4
wur233	\a_1
addi	\a_0, \a_0, 8
lsi f0, \a_0,  0
lsi f1, \a_0,  4
lsi f2, \a_0,  8
lsi f3, \a_0,  12
lsi f4, \a_0,  16
lsi f5, \a_0,  20
lsi f6, \a_0,  24
lsi f7, \a_0,  28
lsi f8, \a_0,  32
lsi f9, \a_0,  36
lsi f10, \a_0,  40
lsi f11, \a_0,  44
lsi f12, \a_0,  48
lsi f13, \a_0,  52
lsi f14, \a_0,  56
lsi f15, \a_0,  60

.endm // xchal_cp0_load


#define xchal_cp_cp6_store	xchal_cp6_store
#define xchal_cp_cp6_store_a2	xchal_cp6_store_a2
#define xchal_cp6_store_a2	xchal_cp6_store	a2, a3
// Name: xchal_cp6_store
// This macro saves the states of cp6
// Prototype: xchal_cp6_store a_0 a_1 
// Pointer to memory: a_0, aligned to 4 bytes
// Scratch register needed:  a_1 
// Example use:
//
// Clobbers:
//    a_0 a_1 

.macro xchal_cp6_store a_0 a_1 
rur0	\a_1
s32i	\a_1, \a_0, 0
addi	\a_0, \a_0, 4
i16_si i160, \a_0,  0
i16_si i161, \a_0,  2
i16_si i162, \a_0,  4
i16_si i163, \a_0,  6
i16_si i164, \a_0,  8
i16_si i165, \a_0,  10
i16_si i166, \a_0,  12
i16_si i167, \a_0,  14
i16_si i168, \a_0,  16
i16_si i169, \a_0,  18
i16_si i1610, \a_0,  20
i16_si i1611, \a_0,  22
i16_si i1612, \a_0,  24
i16_si i1613, \a_0,  26
i16_si i1614, \a_0,  28
i16_si i1615, \a_0,  30

.endm // xchal_cp6_store


#define xchal_cp_cp6_load	xchal_cp6_load
#define xchal_cp_cp6_load_a2	xchal_cp6_load_a2
#define xchal_cp6_load_a2	xchal_cp6_load	a2, a3, a4, a5
// Name: xchal_cp6_load
// This macro restores the states of cp6
// Prototype: xchal_cp6_load a_0 a_1 a_2 a_3 
// Pointer to memory: a_0, aligned to 4 bytes
// Scratch register needed:  a_1 a_2 a_3 
// Example use:
//
// Clobbers:
//    a_0 a_1 a_2 a_3 

.macro xchal_cp6_load a_0 a_1 a_2 a_3 
l32i	\a_1, \a_0, 0
rur0	\a_2
movi	\a_3,  0x0000ffff
xor	\a_1, \a_1, \a_2
and	\a_1, \a_1, \a_3
xor	\a_1, \a_1, \a_2
wur0	\a_1
addi	\a_0, \a_0, 4
i16_li i160, \a_0,  0
i16_li i161, \a_0,  2
i16_li i162, \a_0,  4
i16_li i163, \a_0,  6
i16_li i164, \a_0,  8
i16_li i165, \a_0,  10
i16_li i166, \a_0,  12
i16_li i167, \a_0,  14
i16_li i168, \a_0,  16
i16_li i169, \a_0,  18
i16_li i1610, \a_0,  20
i16_li i1611, \a_0,  22
i16_li i1612, \a_0,  24
i16_li i1613, \a_0,  26
i16_li i1614, \a_0,  28
i16_li i1615, \a_0,  30

.endm // xchal_cp6_load


#define xchal_cp_cp2_store	xchal_cp2_store
#define xchal_cp_cp2_store_a2	xchal_cp2_store_a2
#define xchal_cp2_store_a2	xchal_cp2_store	a2, a3
// Name: xchal_cp2_store
// This macro saves the states of cp2
// Prototype: xchal_cp2_store a_0 a_1 
// Pointer to memory: a_0, aligned to 4 bytes
// Scratch register needed:  a_1 
// Example use:
//
// Clobbers:
//    a_0 a_1 

.macro xchal_cp2_store a_0 a_1 
rur1	\a_1
s32i	\a_1, \a_0, 0
addi	\a_0, \a_0, 4
i32_si i320, \a_0,  0
i32_si i321, \a_0,  4
i32_si i322, \a_0,  8
i32_si i323, \a_0,  12
i32_si i324, \a_0,  16
i32_si i325, \a_0,  20
i32_si i326, \a_0,  24
i32_si i327, \a_0,  28
i32_si i328, \a_0,  32
i32_si i329, \a_0,  36
i32_si i3210, \a_0,  40
i32_si i3211, \a_0,  44
i32_si i3212, \a_0,  48
i32_si i3213, \a_0,  52
i32_si i3214, \a_0,  56
i32_si i3215, \a_0,  60

.endm // xchal_cp2_store


#define xchal_cp_cp2_load	xchal_cp2_load
#define xchal_cp_cp2_load_a2	xchal_cp2_load_a2
#define xchal_cp2_load_a2	xchal_cp2_load	a2, a3
// Name: xchal_cp2_load
// This macro restores the states of cp2
// Prototype: xchal_cp2_load a_0 a_1 
// Pointer to memory: a_0, aligned to 4 bytes
// Scratch register needed:  a_1 
// Example use:
//
// Clobbers:
//    a_0 a_1 

.macro xchal_cp2_load a_0 a_1 
l32i	\a_1, \a_0, 0
wur1	\a_1
addi	\a_0, \a_0, 4
i32_li i320, \a_0,  0
i32_li i321, \a_0,  4
i32_li i322, \a_0,  8
i32_li i323, \a_0,  12
i32_li i324, \a_0,  16
i32_li i325, \a_0,  20
i32_li i326, \a_0,  24
i32_li i327, \a_0,  28
i32_li i328, \a_0,  32
i32_li i329, \a_0,  36
i32_li i3210, \a_0,  40
i32_li i3211, \a_0,  44
i32_li i3212, \a_0,  48
i32_li i3213, \a_0,  52
i32_li i3214, \a_0,  56
i32_li i3215, \a_0,  60

.endm // xchal_cp2_load


#define xchal_cp_cp5_store	xchal_cp5_store
#define xchal_cp_cp5_store_a2	xchal_cp5_store_a2
#define xchal_cp5_store_a2	xchal_cp5_store	a2, a3
// Name: xchal_cp5_store
// This macro saves the states of cp5
// Prototype: xchal_cp5_store a_0 a_1 
// Pointer to memory: a_0, aligned to 8 bytes
// Scratch register needed:  a_1 
// Example use:
//
// Clobbers:
//    a_0 a_1 

.macro xchal_cp5_store a_0 a_1 
rur2	\a_1
s32i	\a_1, \a_0, 0
rur3	\a_1
s32i	\a_1, \a_0, 4
addi	\a_0, \a_0, 8
i64_si i640, \a_0,  0
i64_si i641, \a_0,  8
i64_si i642, \a_0,  16
i64_si i643, \a_0,  24
i64_si i644, \a_0,  32
i64_si i645, \a_0,  40
i64_si i646, \a_0,  48
i64_si i647, \a_0,  56
i64_si i648, \a_0,  64
i64_si i649, \a_0,  72
i64_si i6410, \a_0,  80
i64_si i6411, \a_0,  88
i64_si i6412, \a_0,  96
i64_si i6413, \a_0,  104
i64_si i6414, \a_0,  112
i64_si i6415, \a_0,  120

.endm // xchal_cp5_store


#define xchal_cp_cp5_load	xchal_cp5_load
#define xchal_cp_cp5_load_a2	xchal_cp5_load_a2
#define xchal_cp5_load_a2	xchal_cp5_load	a2, a3
// Name: xchal_cp5_load
// This macro restores the states of cp5
// Prototype: xchal_cp5_load a_0 a_1 
// Pointer to memory: a_0, aligned to 8 bytes
// Scratch register needed:  a_1 
// Example use:
//
// Clobbers:
//    a_0 a_1 

.macro xchal_cp5_load a_0 a_1 
l32i	\a_1, \a_0, 0
wur2	\a_1
l32i	\a_1, \a_0, 4
wur3	\a_1
addi	\a_0, \a_0, 8
i64_li i640, \a_0,  0
i64_li i641, \a_0,  8
i64_li i642, \a_0,  16
i64_li i643, \a_0,  24
i64_li i644, \a_0,  32
i64_li i645, \a_0,  40
i64_li i646, \a_0,  48
i64_li i647, \a_0,  56
i64_li i648, \a_0,  64
i64_li i649, \a_0,  72
i64_li i6410, \a_0,  80
i64_li i6411, \a_0,  88
i64_li i6412, \a_0,  96
i64_li i6413, \a_0,  104
i64_li i6414, \a_0,  112
i64_li i6415, \a_0,  120

.endm // xchal_cp5_load




/********************
 *  Macros to create functions that save and restore the state of *any* TIE coprocessor.
 */

	/*
	 *  Macro that expands to the body of a function
	 *  that stores the selected coprocessor's state (registers etc).
	 *	Entry:	a2 = ptr to save area in which to save cp state
	 *		a3 = coprocessor number
	 *	Exit:	any register a2-a15 (?) may have been clobbered.
	 */
	.macro	xchal_cpi_store_funcbody
	bnez	a3, 99f
	xchal_cp_cp0_store_a2
	j	.Lcpi_store_end_\@
99:
	bnei	a3, 6, 99f
	xchal_cp_cp6_store_a2
	j	.Lcpi_store_end_\@
99:
	bnei	a3, 2, 99f
	xchal_cp_cp2_store_a2
	j	.Lcpi_store_end_\@
99:
	bnei	a3, 5, 99f
	xchal_cp_cp5_store_a2
	j	.Lcpi_store_end_\@
99:
.Lcpi_store_end_\@:
	.endm


	/*
	 *  Macro that expands to the body of a function
	 *  that loads the selected coprocessor's state (registers etc).
	 *	Entry:	a2 = ptr to save area from which to restore cp state
	 *		a3 = coprocessor number
	 *	Exit:	any register a2-a15 (?) may have been clobbered.
	 */
	.macro	xchal_cpi_load_funcbody
	bnez	a3, 99f
	xchal_cp_cp0_load_a2
	j	.Lcpi_load_end_\@
99:
	bnei	a3, 6, 99f
	xchal_cp_cp6_load_a2
	j	.Lcpi_load_end_\@
99:
	bnei	a3, 2, 99f
	xchal_cp_cp2_load_a2
	j	.Lcpi_load_end_\@
99:
	bnei	a3, 5, 99f
	xchal_cp_cp5_load_a2
	j	.Lcpi_load_end_\@
99:
.Lcpi_load_end_\@:
	.endm

#endif /*_ASMLANGUAGE*/


/*----------------------------------------------------------------------
			INTERNAL I/D RAM/ROMs and XLMI
  ----------------------------------------------------------------------*/

#define XCHAL_NUM_INSTROM		0	/* number of core instruction ROMs configured */
#define XCHAL_NUM_INSTRAM		1	/* number of core instruction RAMs configured */
#define XCHAL_NUM_DATAROM		0	/* number of core data ROMs configured */
#define XCHAL_NUM_DATARAM		1	/* number of core data RAMs configured */
#define XCHAL_NUM_XLMI			1	/* number of core XLMI ports configured */
#define  XCHAL_NUM_IROM			XCHAL_NUM_INSTROM	/* (DEPRECATED) */
#define  XCHAL_NUM_IRAM			XCHAL_NUM_INSTRAM	/* (DEPRECATED) */
#define  XCHAL_NUM_DROM			XCHAL_NUM_DATAROM	/* (DEPRECATED) */
#define  XCHAL_NUM_DRAM			XCHAL_NUM_DATARAM	/* (DEPRECATED) */

/*  Instruction RAM 0:  */
#define XCHAL_INSTRAM0_VADDR		0xCFFFF000
#define XCHAL_INSTRAM0_PADDR		0xCFFFF000
#define XCHAL_INSTRAM0_SIZE		4096
#define XCHAL_IRAM0_VADDR		XCHAL_INSTRAM0_VADDR	/* (DEPRECATED) */
#define XCHAL_IRAM0_PADDR		XCHAL_INSTRAM0_PADDR	/* (DEPRECATED) */
#define XCHAL_IRAM0_SIZE		XCHAL_INSTRAM0_SIZE	/* (DEPRECATED) */

/*  Data RAM 0:  */
#define XCHAL_DATARAM0_VADDR		0xCFFFE000
#define XCHAL_DATARAM0_PADDR		0xCFFFE000
#define XCHAL_DATARAM0_SIZE		2048
#define XCHAL_DRAM0_VADDR		XCHAL_DATARAM0_VADDR	/* (DEPRECATED) */
#define XCHAL_DRAM0_PADDR		XCHAL_DATARAM0_PADDR	/* (DEPRECATED) */
#define XCHAL_DRAM0_SIZE		XCHAL_DATARAM0_SIZE	/* (DEPRECATED) */

/*  XLMI Port 0:  */
#define XCHAL_XLMI0_VADDR		0xCFF80000
#define XCHAL_XLMI0_PADDR		0xCFF80000
#define XCHAL_XLMI0_SIZE		262144



/*----------------------------------------------------------------------
				CACHE
  ----------------------------------------------------------------------*/

/*  Size of the cache lines in log2(bytes):  */
#define XCHAL_ICACHE_LINEWIDTH		5
#define XCHAL_DCACHE_LINEWIDTH		6
/*  Size of the cache lines in bytes:  */
#define XCHAL_ICACHE_LINESIZE		32
#define XCHAL_DCACHE_LINESIZE		64
/*  Max for both I-cache and D-cache (used for general alignment):  */
#define XCHAL_CACHE_LINEWIDTH_MAX	6
#define XCHAL_CACHE_LINESIZE_MAX	64

/*  Number of cache sets in log2(lines per way):  */
#define XCHAL_ICACHE_SETWIDTH		7
#define XCHAL_DCACHE_SETWIDTH		4
/*  Max for both I-cache and D-cache (used for general cache-coherency page alignment):  */
#define XCHAL_CACHE_SETWIDTH_MAX	7
#define XCHAL_CACHE_SETSIZE_MAX		128

/*  Cache set associativity (number of ways):  */
#define XCHAL_ICACHE_WAYS		1
#define XCHAL_DCACHE_WAYS		2

/*  Size of the caches in bytes (ways * 2^(linewidth + setwidth)):  */
#define XCHAL_ICACHE_SIZE		4096
#define XCHAL_DCACHE_SIZE		2048

/*  Cache features:  */
#define XCHAL_DCACHE_IS_WRITEBACK	1
/*  Whether cache locking feature is available:  */
#define XCHAL_ICACHE_LINE_LOCKABLE	0
#define XCHAL_DCACHE_LINE_LOCKABLE	1

/*  Number of (encoded) cache attribute bits:  */
#define XCHAL_CA_BITS			4	/* number of bits needed to hold cache attribute encoding */
/*  (The number of access mode bits (decoded cache attribute bits) is defined by the architecture; see xtensa/hal.h?)  */


/*  Cache Attribute encodings -- lists of access modes for each cache attribute:  */
#define XCHAL_FCA_LIST		XTHAL_FAM_EXCEPTION	XCHAL_SEP \
				XTHAL_FAM_BYPASS	XCHAL_SEP \
				XTHAL_FAM_EXCEPTION	XCHAL_SEP \
				XTHAL_FAM_BYPASS	XCHAL_SEP \
				XTHAL_FAM_EXCEPTION	XCHAL_SEP \
				XTHAL_FAM_CACHED	XCHAL_SEP \
				XTHAL_FAM_EXCEPTION	XCHAL_SEP \
				XTHAL_FAM_CACHED	XCHAL_SEP \
				XTHAL_FAM_EXCEPTION	XCHAL_SEP \
				XTHAL_FAM_CACHED	XCHAL_SEP \
				XTHAL_FAM_EXCEPTION	XCHAL_SEP \
				XTHAL_FAM_CACHED	XCHAL_SEP \
				XTHAL_FAM_EXCEPTION	XCHAL_SEP \
				XTHAL_FAM_EXCEPTION	XCHAL_SEP \
				XTHAL_FAM_EXCEPTION	XCHAL_SEP \
				XTHAL_FAM_EXCEPTION
#define XCHAL_LCA_LIST		XTHAL_LAM_EXCEPTION	XCHAL_SEP \
				XTHAL_LAM_BYPASSG	XCHAL_SEP \
				XTHAL_LAM_EXCEPTION	XCHAL_SEP \
				XTHAL_LAM_BYPASSG	XCHAL_SEP \
				XTHAL_LAM_EXCEPTION	XCHAL_SEP \
				XTHAL_LAM_CACHED	XCHAL_SEP \
				XTHAL_LAM_EXCEPTION	XCHAL_SEP \
				XTHAL_LAM_CACHED	XCHAL_SEP \
				XTHAL_LAM_EXCEPTION	XCHAL_SEP \
				XTHAL_LAM_CACHED	XCHAL_SEP \
				XTHAL_LAM_EXCEPTION	XCHAL_SEP \
				XTHAL_LAM_CACHED	XCHAL_SEP \
				XTHAL_LAM_EXCEPTION	XCHAL_SEP \
				XTHAL_LAM_ISOLATE	XCHAL_SEP \
				XTHAL_LAM_EXCEPTION	XCHAL_SEP \
				XTHAL_LAM_CACHED
#define XCHAL_SCA_LIST		XTHAL_SAM_EXCEPTION	XCHAL_SEP \
				XTHAL_SAM_EXCEPTION	XCHAL_SEP \
				XTHAL_SAM_EXCEPTION	XCHAL_SEP \
				XTHAL_SAM_BYPASS	XCHAL_SEP \
				XTHAL_SAM_EXCEPTION	XCHAL_SEP \
				XTHAL_SAM_EXCEPTION	XCHAL_SEP \
				XTHAL_SAM_EXCEPTION	XCHAL_SEP \
				XTHAL_SAM_WRITEBACK	XCHAL_SEP \
				XTHAL_SAM_EXCEPTION	XCHAL_SEP \
				XTHAL_SAM_EXCEPTION	XCHAL_SEP \
				XTHAL_SAM_EXCEPTION	XCHAL_SEP \
				XTHAL_SAM_WRITETHRU	XCHAL_SEP \
				XTHAL_SAM_EXCEPTION	XCHAL_SEP \
				XTHAL_SAM_ISOLATE	XCHAL_SEP \
				XTHAL_SAM_EXCEPTION	XCHAL_SEP \
				XTHAL_SAM_WRITEBACK

/*  Test:
	read/only: 0 + 1 + 2 + 4 + 5 + 6 + 8 + 9 + 10 + 12 + 14
	read/only: 0 + 1 + 2 + 4 + 5 + 6 + 8 + 9 + 10 + 12 + 14
	all:       0 + 1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9 + 10 + 11 + 12 + 13 + 14 + 15
	fault:     0 + 2 + 4 + 6 + 8 + 10 + 12 + 14
	r/w/x cached:  
	r/w/x dcached: 
	I-bypass:  1 + 3

	load guard bit set: 1 + 3
	load guard bit clr: 0 + 2 + 4 + 5 + 6 + 7 + 8 + 9 + 10 + 11 + 12 + 13 + 14 + 15
	hit-cache r/w/x: 7 + 11

	fams: 5
	fams: 0 / 6 / 18 / 1 / 2
	fams: Bypass / Isolate / Cached / Exception / NACached
 */

/*  MMU okay:  yes  */

/*----------------------------------------------------------------------
				MMU
  ----------------------------------------------------------------------*/

#define XCHAL_HAVE_CACHEATTR		0	/* 1 if CACHEATTR register present, 0 if TLBs present instead */
#define XCHAL_HAVE_TLBS			1	/* 1 if TLBs present, 0 if CACHEATTR present instead */
#define XCHAL_HAVE_MMU			XCHAL_HAVE_TLBS	/* (DEPRECATED; use XCHAL_HAVE_TLBS instead; will be removed in future release) */
#define XCHAL_HAVE_SPANNING_WAY		0	/* 1 if single way maps entire virtual address space in I+D */
#define XCHAL_HAVE_IDENTITY_MAP		0	/* 1 if virtual addr == physical addr always, 0 otherwise */
#define XCHAL_HAVE_MIMIC_CACHEATTR	0	/* 1 if have MMU that mimics a CACHEATTR config (CaMMU) */
#define XCHAL_HAVE_XLT_CACHEATTR	0	/* 1 if have MMU that mimics a CACHEATTR config, but with translation (CaXltMMU) */

#define XCHAL_MMU_ASID_BITS		8	/* number of bits in ASIDs (address space IDs) */
#define XCHAL_MMU_ASID_INVALID		0	/* ASID value indicating invalid address space */
#define XCHAL_MMU_ASID_KERNEL		1	/* ASID value indicating kernel (ring 0) address space */
#define XCHAL_MMU_RINGS			4	/* number of rings supported (1..4) */
#define XCHAL_MMU_RING_BITS		2	/* number of bits needed to hold ring number */
#define XCHAL_MMU_SR_BITS		0	/* number of size-restriction bits supported */
#define XCHAL_MMU_CA_BITS		4	/* number of bits needed to hold cache attribute encoding */
#define XCHAL_MMU_MAX_PTE_PAGE_SIZE	12	/* max page size in a PTE structure */
#define XCHAL_MMU_MIN_PTE_PAGE_SIZE	12	/* min page size in a PTE structure */

/*  Instruction TLB:  */
#define XCHAL_ITLB_WAY_BITS		3	/* number of bits holding the ways */
#define XCHAL_ITLB_WAYS			7	/* number of ways */
#define XCHAL_ITLB_ARF_WAYS		4	/* number of auto-refill ways */

/*  Data TLB:  */
#define XCHAL_DTLB_WAY_BITS		4	/* number of bits holding the ways */
#define XCHAL_DTLB_WAYS			10	/* number of ways */
#define XCHAL_DTLB_ARF_WAYS		4	/* number of auto-refill ways */

/* ... */

/*
 *  Determine whether we have a full MMU (with Page Table and Protection)
 *  usable for an MMU-based OS:
 */
#if XCHAL_HAVE_TLBS && !XCHAL_HAVE_SPANNING_WAY && XCHAL_ITLB_ARF_WAYS > 0 && XCHAL_DTLB_ARF_WAYS > 0 && XCHAL_MMU_RINGS >= 2
# define XCHAL_HAVE_PTP_MMU		1	/* have full MMU (with page table [autorefill] and protection) */
#else
# define XCHAL_HAVE_PTP_MMU		0	/* don't have full MMU */
#endif

/*
 *  For full MMUs, report kernel RAM segment and kernel I/O segment static page mappings:
 */
#if XCHAL_HAVE_PTP_MMU
#define XCHAL_KSEG_CACHED_VADDR		0xD0000000	/* virt.addr of kernel RAM cached static map */
#define XCHAL_KSEG_CACHED_PADDR		0x00000000	/* phys.addr of kseg_cached */
#define XCHAL_KSEG_CACHED_SIZE		0x08000000	/* size in bytes of kseg_cached (assumed power of 2!!!) */
#define XCHAL_KSEG_BYPASS_VADDR		0xD8000000	/* virt.addr of kernel RAM bypass (uncached) static map */
#define XCHAL_KSEG_BYPASS_PADDR		0x00000000	/* phys.addr of kseg_bypass */
#define XCHAL_KSEG_BYPASS_SIZE		0x08000000	/* size in bytes of kseg_bypass (assumed power of 2!!!) */

#define XCHAL_KIO_CACHED_VADDR		0xE0000000	/* virt.addr of kernel I/O cached static map */
#define XCHAL_KIO_CACHED_PADDR		0xF0000000	/* phys.addr of kio_cached */
#define XCHAL_KIO_CACHED_SIZE		0x10000000	/* size in bytes of kio_cached (assumed power of 2!!!) */
#define XCHAL_KIO_BYPASS_VADDR		0xF0000000	/* virt.addr of kernel I/O bypass (uncached) static map */
#define XCHAL_KIO_BYPASS_PADDR		0xF0000000	/* phys.addr of kio_bypass */
#define XCHAL_KIO_BYPASS_SIZE		0x10000000	/* size in bytes of kio_bypass (assumed power of 2!!!) */

#define XCHAL_SEG_MAPPABLE_VADDR	0x00000000	/* start of largest non-static-mapped virtual addr area */
#define XCHAL_SEG_MAPPABLE_SIZE		0xD0000000	/* size in bytes of  "  */
/* define XCHAL_SEG_MAPPABLE2_xxx if more areas present, sorted in order of descending size.  */
#endif


/*----------------------------------------------------------------------
				MISC
  ----------------------------------------------------------------------*/

#define XCHAL_NUM_WRITEBUFFER_ENTRIES	8	/* number of write buffer entries */

#define XCHAL_BUILD_UNIQUE_ID		0x00002BC6	/* software build-unique ID (22-bit) */

/*  These definitions describe the hardware targeted by this software:  */
#define XCHAL_HW_CONFIGID0		0xC1FFDFFE	/* config ID reg 0 value (upper 32 of 64 bits) */
#define XCHAL_HW_CONFIGID1		0x00402BC6	/* config ID reg 1 value (lower 32 of 64 bits) */
#define XCHAL_CONFIGID0			XCHAL_HW_CONFIGID0	/* for backward compatibility only -- don't use! */
#define XCHAL_CONFIGID1			XCHAL_HW_CONFIGID1	/* for backward compatibility only -- don't use! */
#define XCHAL_HW_RELEASE_MAJOR		1050	/* major release of targeted hardware */
#define XCHAL_HW_RELEASE_MINOR		0	/* minor release of targeted hardware */
#define XCHAL_HW_RELEASE_NAME		"T1050.0"	/* full release name of targeted hardware */
#define XTHAL_HW_REL_T1050	1
#define XTHAL_HW_REL_T1050_0	1
#define XCHAL_HW_CONFIGID_RELIABLE	1


/*
 *  Miscellaneous special register fields:
 */


/*  DBREAKC (special register number 160):  */
#define XCHAL_DBREAKC_VALIDMASK	0xC000003F	/* bits of DBREAKC that are defined */
/*  MASK field:  */
#define XCHAL_DBREAKC_MASK_BITS 	6		/* number of bits in MASK field */
#define XCHAL_DBREAKC_MASK_NUM  	64		/* max number of possible causes (2^bits) */
#define XCHAL_DBREAKC_MASK_SHIFT	0		/* position of MASK bits in DBREAKC, starting from lsbit */
#define XCHAL_DBREAKC_MASK_MASK 	0x0000003F	/* mask of bits in MASK field of DBREAKC */
/*  LOADBREAK field:  */
#define XCHAL_DBREAKC_LOADBREAK_BITS 	1		/* number of bits in LOADBREAK field */
#define XCHAL_DBREAKC_LOADBREAK_NUM  	2		/* max number of possible causes (2^bits) */
#define XCHAL_DBREAKC_LOADBREAK_SHIFT	30		/* position of LOADBREAK bits in DBREAKC, starting from lsbit */
#define XCHAL_DBREAKC_LOADBREAK_MASK 	0x40000000	/* mask of bits in LOADBREAK field of DBREAKC */
/*  STOREBREAK field:  */
#define XCHAL_DBREAKC_STOREBREAK_BITS 	1		/* number of bits in STOREBREAK field */
#define XCHAL_DBREAKC_STOREBREAK_NUM  	2		/* max number of possible causes (2^bits) */
#define XCHAL_DBREAKC_STOREBREAK_SHIFT	31		/* position of STOREBREAK bits in DBREAKC, starting from lsbit */
#define XCHAL_DBREAKC_STOREBREAK_MASK 	0x80000000	/* mask of bits in STOREBREAK field of DBREAKC */

/*  PS (special register number 230):  */
#define XCHAL_PS_VALIDMASK	0x00070FFF	/* bits of PS that are defined */
/*  INTLEVEL field:  */
#define XCHAL_PS_INTLEVEL_BITS 	4		/* number of bits in INTLEVEL field */
#define XCHAL_PS_INTLEVEL_NUM  	16		/* max number of possible causes (2^bits) */
#define XCHAL_PS_INTLEVEL_SHIFT	0		/* position of INTLEVEL bits in PS, starting from lsbit */
#define XCHAL_PS_INTLEVEL_MASK 	0x0000000F	/* mask of bits in INTLEVEL field of PS */
/*  EXCM field:  */
#define XCHAL_PS_EXCM_BITS 	1		/* number of bits in EXCM field */
#define XCHAL_PS_EXCM_NUM  	2		/* max number of possible causes (2^bits) */
#define XCHAL_PS_EXCM_SHIFT	4		/* position of EXCM bits in PS, starting from lsbit */
#define XCHAL_PS_EXCM_MASK 	0x00000010	/* mask of bits in EXCM field of PS */
/*  PROGSTACK field:  */
#define XCHAL_PS_PROGSTACK_BITS 	1		/* number of bits in PROGSTACK field */
#define XCHAL_PS_PROGSTACK_NUM  	2		/* max number of possible causes (2^bits) */
#define XCHAL_PS_PROGSTACK_SHIFT	5		/* position of PROGSTACK bits in PS, starting from lsbit */
#define XCHAL_PS_PROGSTACK_MASK 	0x00000020	/* mask of bits in PROGSTACK field of PS */
/*  RING field:  */
#define XCHAL_PS_RING_BITS 	2		/* number of bits in RING field */
#define XCHAL_PS_RING_NUM  	4		/* max number of possible causes (2^bits) */
#define XCHAL_PS_RING_SHIFT	6		/* position of RING bits in PS, starting from lsbit */
#define XCHAL_PS_RING_MASK 	0x000000C0	/* mask of bits in RING field of PS */
/*  OWB field:  */
#define XCHAL_PS_OWB_BITS 	4		/* number of bits in OWB field */
#define XCHAL_PS_OWB_NUM  	16		/* max number of possible causes (2^bits) */
#define XCHAL_PS_OWB_SHIFT	8		/* position of OWB bits in PS, starting from lsbit */
#define XCHAL_PS_OWB_MASK 	0x00000F00	/* mask of bits in OWB field of PS */
/*  CALLINC field:  */
#define XCHAL_PS_CALLINC_BITS 	2		/* number of bits in CALLINC field */
#define XCHAL_PS_CALLINC_NUM  	4		/* max number of possible causes (2^bits) */
#define XCHAL_PS_CALLINC_SHIFT	16		/* position of CALLINC bits in PS, starting from lsbit */
#define XCHAL_PS_CALLINC_MASK 	0x00030000	/* mask of bits in CALLINC field of PS */
/*  WOE field:  */
#define XCHAL_PS_WOE_BITS 	1		/* number of bits in WOE field */
#define XCHAL_PS_WOE_NUM  	2		/* max number of possible causes (2^bits) */
#define XCHAL_PS_WOE_SHIFT	18		/* position of WOE bits in PS, starting from lsbit */
#define XCHAL_PS_WOE_MASK 	0x00040000	/* mask of bits in WOE field of PS */

/*  EXCCAUSE (special register number 232):  */
#define XCHAL_EXCCAUSE_VALIDMASK	0x0000003F	/* bits of EXCCAUSE that are defined */
/*  EXCCAUSE field:  */
#define XCHAL_EXCCAUSE_BITS 		6		/* number of bits in EXCCAUSE register */
#define XCHAL_EXCCAUSE_NUM  		64		/* max number of possible causes (2^bits) */
#define XCHAL_EXCCAUSE_SHIFT		0		/* position of EXCCAUSE bits in register, starting from lsbit */
#define XCHAL_EXCCAUSE_MASK 		0x0000003F	/* mask of bits in EXCCAUSE register */

/*  DEBUGCAUSE (special register number 233):  */
#define XCHAL_DEBUGCAUSE_VALIDMASK	0x0000003F	/* bits of DEBUGCAUSE that are defined */
/*  ICOUNT field:  */
#define XCHAL_DEBUGCAUSE_ICOUNT_BITS 	1		/* number of bits in ICOUNT field */
#define XCHAL_DEBUGCAUSE_ICOUNT_NUM  	2		/* max number of possible causes (2^bits) */
#define XCHAL_DEBUGCAUSE_ICOUNT_SHIFT	0		/* position of ICOUNT bits in DEBUGCAUSE, starting from lsbit */
#define XCHAL_DEBUGCAUSE_ICOUNT_MASK 	0x00000001	/* mask of bits in ICOUNT field of DEBUGCAUSE */
/*  IBREAK field:  */
#define XCHAL_DEBUGCAUSE_IBREAK_BITS 	1		/* number of bits in IBREAK field */
#define XCHAL_DEBUGCAUSE_IBREAK_NUM  	2		/* max number of possible causes (2^bits) */
#define XCHAL_DEBUGCAUSE_IBREAK_SHIFT	1		/* position of IBREAK bits in DEBUGCAUSE, starting from lsbit */
#define XCHAL_DEBUGCAUSE_IBREAK_MASK 	0x00000002	/* mask of bits in IBREAK field of DEBUGCAUSE */
/*  DBREAK field:  */
#define XCHAL_DEBUGCAUSE_DBREAK_BITS 	1		/* number of bits in DBREAK field */
#define XCHAL_DEBUGCAUSE_DBREAK_NUM  	2		/* max number of possible causes (2^bits) */
#define XCHAL_DEBUGCAUSE_DBREAK_SHIFT	2		/* position of DBREAK bits in DEBUGCAUSE, starting from lsbit */
#define XCHAL_DEBUGCAUSE_DBREAK_MASK 	0x00000004	/* mask of bits in DBREAK field of DEBUGCAUSE */
/*  BREAK field:  */
#define XCHAL_DEBUGCAUSE_BREAK_BITS 	1		/* number of bits in BREAK field */
#define XCHAL_DEBUGCAUSE_BREAK_NUM  	2		/* max number of possible causes (2^bits) */
#define XCHAL_DEBUGCAUSE_BREAK_SHIFT	3		/* position of BREAK bits in DEBUGCAUSE, starting from lsbit */
#define XCHAL_DEBUGCAUSE_BREAK_MASK 	0x00000008	/* mask of bits in BREAK field of DEBUGCAUSE */
/*  BREAKN field:  */
#define XCHAL_DEBUGCAUSE_BREAKN_BITS 	1		/* number of bits in BREAKN field */
#define XCHAL_DEBUGCAUSE_BREAKN_NUM  	2		/* max number of possible causes (2^bits) */
#define XCHAL_DEBUGCAUSE_BREAKN_SHIFT	4		/* position of BREAKN bits in DEBUGCAUSE, starting from lsbit */
#define XCHAL_DEBUGCAUSE_BREAKN_MASK 	0x00000010	/* mask of bits in BREAKN field of DEBUGCAUSE */
/*  DEBUGINT field:  */
#define XCHAL_DEBUGCAUSE_DEBUGINT_BITS 	1		/* number of bits in DEBUGINT field */
#define XCHAL_DEBUGCAUSE_DEBUGINT_NUM  	2		/* max number of possible causes (2^bits) */
#define XCHAL_DEBUGCAUSE_DEBUGINT_SHIFT	5		/* position of DEBUGINT bits in DEBUGCAUSE, starting from lsbit */
#define XCHAL_DEBUGCAUSE_DEBUGINT_MASK 	0x00000020	/* mask of bits in DEBUGINT field of DEBUGCAUSE */



/*----------------------------------------------------------------------
				ISA
  ----------------------------------------------------------------------*/

#define XCHAL_HAVE_DENSITY		1	/* 1 if density option configured, 0 otherwise */
#define XCHAL_HAVE_BOOLEANS		1	/* 1 if booleans option configured, 0 otherwise */
#define XCHAL_HAVE_LOOPS		1	/* 1 if zero-overhead loops option configured, 0 otherwise */
/*  Misc instructions:  */
#define XCHAL_HAVE_NSA			1	/* 1 if NSA/NSAU instructions option configured, 0 otherwise */
#define XCHAL_HAVE_MINMAX		1	/* 1 if MIN/MAX instructions option configured, 0 otherwise */
#define XCHAL_HAVE_SEXT			1	/* 1 if sign-extend instruction option configured, 0 otherwise */
#define XCHAL_HAVE_CLAMPS		1	/* 1 if CLAMPS instruction option configured, 0 otherwise */
#define XCHAL_HAVE_MAC16		1	/* 1 if MAC16 option configured, 0 otherwise */
#define XCHAL_HAVE_MUL16		1	/* 1 if 16-bit integer multiply option configured, 0 otherwise */
#define XCHAL_HAVE_MUL32		0	/* 1 if 32-bit integer multiply option configured, 0 otherwise */
#define XCHAL_HAVE_MUL32_HIGH		0	/* 1 if MUL32 option includes MULUH and MULSH, 0 otherwise */
/*#define XCHAL_HAVE_POPC		0*/	/* 1 if CRC instruction option configured, 0 otherwise */
/*#define XCHAL_HAVE_CRC		0*/	/* 1 if POPC instruction option configured, 0 otherwise */

#define XCHAL_HAVE_FP			1	/* 1 if floating point option configured, 0 otherwise */
#define XCHAL_HAVE_SPECULATION		0	/* 1 if speculation option configured, 0 otherwise */
/*#define XCHAL_HAVE_MP_SYNC		0*/	/* 1 if multiprocessor sync. option configured, 0 otherwise */
#define XCHAL_HAVE_PRID			1	/* 1 if processor ID register configured, 0 otherwise */

#define XCHAL_NUM_MISC_REGS		4	/* number of miscellaneous registers (0..4) */


/*----------------------------------------------------------------------
				DERIVED
  ----------------------------------------------------------------------*/

#if XCHAL_HAVE_BE
#define XCHAL_INST_ILLN			0xD60F		/* 2-byte illegal instruction, msb-first */
#define XCHAL_INST_ILLN_BYTE0		0xD6		/* 2-byte illegal instruction, 1st byte */
#define XCHAL_INST_ILLN_BYTE1		0x0F		/* 2-byte illegal instruction, 2nd byte */
#else
#define XCHAL_INST_ILLN			0xF06D		/* 2-byte illegal instruction, lsb-first */
#define XCHAL_INST_ILLN_BYTE0		0x6D		/* 2-byte illegal instruction, 1st byte */
#define XCHAL_INST_ILLN_BYTE1		0xF0		/* 2-byte illegal instruction, 2nd byte */
#endif
/*  Belongs in xtensa/hal.h:  */
#define XTHAL_INST_ILL			0x000000	/* 3-byte illegal instruction */


#endif /*XTENSA_CONFIG_CORE_H*/

