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
 * Customer ID=40; Build=10966; Copyright (c) 2002 by Tensilica Inc.  ALL RIGHTS RESERVED.
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
#define XCHAL_NUM_AREGS			64	/* number of physical address regs */
#define XCHAL_NUM_AREGS_LOG2		6	/* log2(XCHAL_NUM_AREGS) */


/*----------------------------------------------------------------------
				INTERRUPTS
  ----------------------------------------------------------------------*/

#define XCHAL_HAVE_INTERRUPTS		1	/* 1 if interrupt option configured, 0 otherwise */
#define XCHAL_HAVE_HIGHLEVEL_INTERRUPTS	1	/* 1 if high-level interrupt option configured, 0 otherwise */
#define XCHAL_HAVE_NMI			0	/* 1 if NMI option configured, 0 otherwise */
#define XCHAL_NUM_INTERRUPTS		17	/* number of interrupts */
#define XCHAL_NUM_EXTINTERRUPTS		10	/* number of external interrupts */
#define XCHAL_NUM_INTLEVELS		4	/* number of interrupt levels (not including level zero!) */

/*  Masks of interrupts at each interrupt level:  */
#define XCHAL_INTLEVEL0_MASK		0x00000000
#define XCHAL_INTLEVEL1_MASK		0x000064F9
#define XCHAL_INTLEVEL2_MASK		0x00008902
#define XCHAL_INTLEVEL3_MASK		0x00011204
#define XCHAL_INTLEVEL4_MASK		0x00000000
#define XCHAL_INTLEVEL5_MASK		0x00000000
#define XCHAL_INTLEVEL6_MASK		0x00000000
#define XCHAL_INTLEVEL7_MASK		0x00000000
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
					0x000064F9	XCHAL_SEP \
					0x00008902	XCHAL_SEP \
					0x00011204	XCHAL_SEP \
					0x00000000	XCHAL_SEP \
					0x00000000	XCHAL_SEP \
					0x00000000	XCHAL_SEP \
					0x00000000	XCHAL_SEP \
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
#define XCHAL_INTLEVEL1_ANDBELOW_MASK	0x000064F9
#define XCHAL_INTLEVEL2_ANDBELOW_MASK	0x0000EDFB
#define XCHAL_INTLEVEL3_ANDBELOW_MASK	0x0001FFFF
#define XCHAL_INTLEVEL4_ANDBELOW_MASK	0x0001FFFF
#define XCHAL_INTLEVEL5_ANDBELOW_MASK	0x0001FFFF
#define XCHAL_INTLEVEL6_ANDBELOW_MASK	0x0001FFFF
#define XCHAL_INTLEVEL7_ANDBELOW_MASK	0x0001FFFF
#define XCHAL_INTLEVEL8_ANDBELOW_MASK	0x0001FFFF
#define XCHAL_INTLEVEL9_ANDBELOW_MASK	0x0001FFFF
#define XCHAL_INTLEVEL10_ANDBELOW_MASK	0x0001FFFF
#define XCHAL_INTLEVEL11_ANDBELOW_MASK	0x0001FFFF
#define XCHAL_INTLEVEL12_ANDBELOW_MASK	0x0001FFFF
#define XCHAL_INTLEVEL13_ANDBELOW_MASK	0x0001FFFF
#define XCHAL_INTLEVEL14_ANDBELOW_MASK	0x0001FFFF
#define XCHAL_INTLEVEL15_ANDBELOW_MASK	0x0001FFFF
/*  As an array of entries (eg. for C constant arrays):  */
#define XCHAL_INTLEVEL_ANDBELOW_MASKS	0x00000000	XCHAL_SEP \
					0x000064F9	XCHAL_SEP \
					0x0000EDFB	XCHAL_SEP \
					0x0001FFFF	XCHAL_SEP \
					0x0001FFFF	XCHAL_SEP \
					0x0001FFFF	XCHAL_SEP \
					0x0001FFFF	XCHAL_SEP \
					0x0001FFFF	XCHAL_SEP \
					0x0001FFFF	XCHAL_SEP \
					0x0001FFFF	XCHAL_SEP \
					0x0001FFFF	XCHAL_SEP \
					0x0001FFFF	XCHAL_SEP \
					0x0001FFFF	XCHAL_SEP \
					0x0001FFFF	XCHAL_SEP \
					0x0001FFFF	XCHAL_SEP \
					0x0001FFFF

/*  Level of each interrupt:  */
#define XCHAL_INT0_LEVEL		1
#define XCHAL_INT1_LEVEL		2
#define XCHAL_INT2_LEVEL		3
#define XCHAL_INT3_LEVEL		1
#define XCHAL_INT4_LEVEL		1
#define XCHAL_INT5_LEVEL		1
#define XCHAL_INT6_LEVEL		1
#define XCHAL_INT7_LEVEL		1
#define XCHAL_INT8_LEVEL		2
#define XCHAL_INT9_LEVEL		3
#define XCHAL_INT10_LEVEL		1
#define XCHAL_INT11_LEVEL		2
#define XCHAL_INT12_LEVEL		3
#define XCHAL_INT13_LEVEL		1
#define XCHAL_INT14_LEVEL		1
#define XCHAL_INT15_LEVEL		2
#define XCHAL_INT16_LEVEL		3
#define XCHAL_INT17_LEVEL		0
#define XCHAL_INT18_LEVEL		0
#define XCHAL_INT19_LEVEL		0
#define XCHAL_INT20_LEVEL		0
#define XCHAL_INT21_LEVEL		0
#define XCHAL_INT22_LEVEL		0
#define XCHAL_INT23_LEVEL		0
#define XCHAL_INT24_LEVEL		0
#define XCHAL_INT25_LEVEL		0
#define XCHAL_INT26_LEVEL		0
#define XCHAL_INT27_LEVEL		0
#define XCHAL_INT28_LEVEL		0
#define XCHAL_INT29_LEVEL		0
#define XCHAL_INT30_LEVEL		0
#define XCHAL_INT31_LEVEL		0
/*  As an array of entries (eg. for C constant arrays):  */
#define XCHAL_INT_LEVELS		1	XCHAL_SEP \
					2	XCHAL_SEP \
					3	XCHAL_SEP \
					1	XCHAL_SEP \
					1	XCHAL_SEP \
					1	XCHAL_SEP \
					1	XCHAL_SEP \
					1	XCHAL_SEP \
					2	XCHAL_SEP \
					3	XCHAL_SEP \
					1	XCHAL_SEP \
					2	XCHAL_SEP \
					3	XCHAL_SEP \
					1	XCHAL_SEP \
					1	XCHAL_SEP \
					2	XCHAL_SEP \
					3	XCHAL_SEP \
					0	XCHAL_SEP \
					0	XCHAL_SEP \
					0	XCHAL_SEP \
					0	XCHAL_SEP \
					0	XCHAL_SEP \
					0	XCHAL_SEP \
					0	XCHAL_SEP \
					0	XCHAL_SEP \
					0	XCHAL_SEP \
					0	XCHAL_SEP \
					0	XCHAL_SEP \
					0	XCHAL_SEP \
					0	XCHAL_SEP \
					0	XCHAL_SEP \
					0

/*  Type of each interrupt:  */
#define XCHAL_INT0_TYPE 	XTHAL_INTTYPE_EXTERN_LEVEL
#define XCHAL_INT1_TYPE 	XTHAL_INTTYPE_EXTERN_LEVEL
#define XCHAL_INT2_TYPE 	XTHAL_INTTYPE_EXTERN_LEVEL
#define XCHAL_INT3_TYPE 	XTHAL_INTTYPE_EXTERN_LEVEL
#define XCHAL_INT4_TYPE 	XTHAL_INTTYPE_EXTERN_LEVEL
#define XCHAL_INT5_TYPE 	XTHAL_INTTYPE_EXTERN_LEVEL
#define XCHAL_INT6_TYPE 	XTHAL_INTTYPE_EXTERN_LEVEL
#define XCHAL_INT7_TYPE 	XTHAL_INTTYPE_EXTERN_EDGE
#define XCHAL_INT8_TYPE 	XTHAL_INTTYPE_EXTERN_EDGE
#define XCHAL_INT9_TYPE 	XTHAL_INTTYPE_EXTERN_EDGE
#define XCHAL_INT10_TYPE 	XTHAL_INTTYPE_TIMER
#define XCHAL_INT11_TYPE 	XTHAL_INTTYPE_TIMER
#define XCHAL_INT12_TYPE 	XTHAL_INTTYPE_TIMER
#define XCHAL_INT13_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT14_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT15_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT16_TYPE 	XTHAL_INTTYPE_SOFTWARE
#define XCHAL_INT17_TYPE 	XTHAL_INTTYPE_UNCONFIGURED
#define XCHAL_INT18_TYPE 	XTHAL_INTTYPE_UNCONFIGURED
#define XCHAL_INT19_TYPE 	XTHAL_INTTYPE_UNCONFIGURED
#define XCHAL_INT20_TYPE 	XTHAL_INTTYPE_UNCONFIGURED
#define XCHAL_INT21_TYPE 	XTHAL_INTTYPE_UNCONFIGURED
#define XCHAL_INT22_TYPE 	XTHAL_INTTYPE_UNCONFIGURED
#define XCHAL_INT23_TYPE 	XTHAL_INTTYPE_UNCONFIGURED
#define XCHAL_INT24_TYPE 	XTHAL_INTTYPE_UNCONFIGURED
#define XCHAL_INT25_TYPE 	XTHAL_INTTYPE_UNCONFIGURED
#define XCHAL_INT26_TYPE 	XTHAL_INTTYPE_UNCONFIGURED
#define XCHAL_INT27_TYPE 	XTHAL_INTTYPE_UNCONFIGURED
#define XCHAL_INT28_TYPE 	XTHAL_INTTYPE_UNCONFIGURED
#define XCHAL_INT29_TYPE 	XTHAL_INTTYPE_UNCONFIGURED
#define XCHAL_INT30_TYPE 	XTHAL_INTTYPE_UNCONFIGURED
#define XCHAL_INT31_TYPE 	XTHAL_INTTYPE_UNCONFIGURED
/*  As an array of entries (eg. for C constant arrays):  */
#define XCHAL_INT_TYPES		XTHAL_INTTYPE_EXTERN_LEVEL     	XCHAL_SEP \
				XTHAL_INTTYPE_EXTERN_LEVEL     	XCHAL_SEP \
				XTHAL_INTTYPE_EXTERN_LEVEL     	XCHAL_SEP \
				XTHAL_INTTYPE_EXTERN_LEVEL     	XCHAL_SEP \
				XTHAL_INTTYPE_EXTERN_LEVEL     	XCHAL_SEP \
				XTHAL_INTTYPE_EXTERN_LEVEL     	XCHAL_SEP \
				XTHAL_INTTYPE_EXTERN_LEVEL     	XCHAL_SEP \
				XTHAL_INTTYPE_EXTERN_EDGE     	XCHAL_SEP \
				XTHAL_INTTYPE_EXTERN_EDGE     	XCHAL_SEP \
				XTHAL_INTTYPE_EXTERN_EDGE     	XCHAL_SEP \
				XTHAL_INTTYPE_TIMER     	XCHAL_SEP \
				XTHAL_INTTYPE_TIMER     	XCHAL_SEP \
				XTHAL_INTTYPE_TIMER     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_SOFTWARE     	XCHAL_SEP \
				XTHAL_INTTYPE_UNCONFIGURED     	XCHAL_SEP \
				XTHAL_INTTYPE_UNCONFIGURED     	XCHAL_SEP \
				XTHAL_INTTYPE_UNCONFIGURED     	XCHAL_SEP \
				XTHAL_INTTYPE_UNCONFIGURED     	XCHAL_SEP \
				XTHAL_INTTYPE_UNCONFIGURED     	XCHAL_SEP \
				XTHAL_INTTYPE_UNCONFIGURED     	XCHAL_SEP \
				XTHAL_INTTYPE_UNCONFIGURED     	XCHAL_SEP \
				XTHAL_INTTYPE_UNCONFIGURED     	XCHAL_SEP \
				XTHAL_INTTYPE_UNCONFIGURED     	XCHAL_SEP \
				XTHAL_INTTYPE_UNCONFIGURED     	XCHAL_SEP \
				XTHAL_INTTYPE_UNCONFIGURED     	XCHAL_SEP \
				XTHAL_INTTYPE_UNCONFIGURED     	XCHAL_SEP \
				XTHAL_INTTYPE_UNCONFIGURED     	XCHAL_SEP \
				XTHAL_INTTYPE_UNCONFIGURED     	XCHAL_SEP \
				XTHAL_INTTYPE_UNCONFIGURED

/*  Masks of interrupts for each type of interrupt:  */
#define XCHAL_INTTYPE_MASK_UNCONFIGURED	0xFFFE0000
#define XCHAL_INTTYPE_MASK_SOFTWARE	0x0001E000
#define XCHAL_INTTYPE_MASK_EXTERN_EDGE	0x00000380
#define XCHAL_INTTYPE_MASK_EXTERN_LEVEL	0x0000007F
#define XCHAL_INTTYPE_MASK_TIMER	0x00001C00
#define XCHAL_INTTYPE_MASK_NMI		0x00000000
/*  As an array of entries (eg. for C constant arrays):  */
#define XCHAL_INTTYPE_MASKS		0xFFFE0000	XCHAL_SEP \
					0x0001E000	XCHAL_SEP \
					0x00000380	XCHAL_SEP \
					0x0000007F	XCHAL_SEP \
					0x00001C00	XCHAL_SEP \
					0x00000000

/*  Interrupts assigned to each timer (CCOMPARE0 to CCOMPARE3), -1 if unassigned  */
#define XCHAL_TIMER0_INTERRUPT	10
#define XCHAL_TIMER1_INTERRUPT	11
#define XCHAL_TIMER2_INTERRUPT	12
#define XCHAL_TIMER3_INTERRUPT	XTHAL_TIMER_UNCONFIGURED
/*  As an array of entries (eg. for C constant arrays):  */
#define XCHAL_TIMER_INTERRUPTS	10	XCHAL_SEP \
				11	XCHAL_SEP \
				12	XCHAL_SEP \
				XTHAL_TIMER_UNCONFIGURED

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
#define XCHAL_EXTINT0_NUM		0	/* (intlevel 1) */
#define XCHAL_EXTINT1_NUM		1	/* (intlevel 2) */
#define XCHAL_EXTINT2_NUM		2	/* (intlevel 3) */
#define XCHAL_EXTINT3_NUM		3	/* (intlevel 1) */
#define XCHAL_EXTINT4_NUM		4	/* (intlevel 1) */
#define XCHAL_EXTINT5_NUM		5	/* (intlevel 1) */
#define XCHAL_EXTINT6_NUM		6	/* (intlevel 1) */
#define XCHAL_EXTINT7_NUM		7	/* (intlevel 1) */
#define XCHAL_EXTINT8_NUM		8	/* (intlevel 2) */
#define XCHAL_EXTINT9_NUM		9	/* (intlevel 3) */

/*  Corresponding interrupt masks:  */
#define XCHAL_EXTINT0_MASK		0x00000001
#define XCHAL_EXTINT1_MASK		0x00000002
#define XCHAL_EXTINT2_MASK		0x00000004
#define XCHAL_EXTINT3_MASK		0x00000008
#define XCHAL_EXTINT4_MASK		0x00000010
#define XCHAL_EXTINT5_MASK		0x00000020
#define XCHAL_EXTINT6_MASK		0x00000040
#define XCHAL_EXTINT7_MASK		0x00000080
#define XCHAL_EXTINT8_MASK		0x00000100
#define XCHAL_EXTINT9_MASK		0x00000200

/*  Core config interrupt levels mapped to each external interrupt:  */
#define XCHAL_EXTINT0_LEVEL		1	/* (int number 0) */
#define XCHAL_EXTINT1_LEVEL		2	/* (int number 1) */
#define XCHAL_EXTINT2_LEVEL		3	/* (int number 2) */
#define XCHAL_EXTINT3_LEVEL		1	/* (int number 3) */
#define XCHAL_EXTINT4_LEVEL		1	/* (int number 4) */
#define XCHAL_EXTINT5_LEVEL		1	/* (int number 5) */
#define XCHAL_EXTINT6_LEVEL		1	/* (int number 6) */
#define XCHAL_EXTINT7_LEVEL		1	/* (int number 7) */
#define XCHAL_EXTINT8_LEVEL		2	/* (int number 8) */
#define XCHAL_EXTINT9_LEVEL		3	/* (int number 9) */


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
#define XCHAL_DOUBLEEXC_VECTOR_VADDR	0xD0000290
#define XCHAL_DOUBLEEXC_VECTOR_PADDR	0x00000290
#define XCHAL_WINDOW_VECTORS_VADDR	0xD0000000
#define XCHAL_WINDOW_VECTORS_PADDR	0x00000000
#define XCHAL_INTLEVEL2_VECTOR_VADDR	0xD0000240
#define XCHAL_INTLEVEL2_VECTOR_PADDR	0x00000240
#define XCHAL_INTLEVEL3_VECTOR_VADDR	0xD0000250
#define XCHAL_INTLEVEL3_VECTOR_PADDR	0x00000250
#define XCHAL_INTLEVEL4_VECTOR_VADDR	0xFE000520
#define XCHAL_INTLEVEL4_VECTOR_PADDR	0xFE000520
#define XCHAL_DEBUG_VECTOR_VADDR	XCHAL_INTLEVEL4_VECTOR_VADDR
#define XCHAL_DEBUG_VECTOR_PADDR	XCHAL_INTLEVEL4_VECTOR_PADDR

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
#define XCHAL_DEBUGLEVEL		4	/* debug interrupt level */
/*DebugExternalInterrupt		0		0|1*/
/*DebugUseDIRArray			0		0|1*/




/*----------------------------------------------------------------------
			COPROCESSORS and EXTRA STATE
  ----------------------------------------------------------------------*/

#define XCHAL_HAVE_CP			0	/* 1 if coprocessor option configured (CPENABLE present) */
#define XCHAL_CP_NUM			0	/* number of coprocessors */
#define XCHAL_CP_MAX			0	/* max coprocessor id plus one (0 if none) */
#define XCHAL_CP_MAXCFG			0	/* max allowed cp id plus one (per cfg) */
#define XCHAL_CP_MASK			0x00	/* bitmask of coprocessors by id */

/*  Space for coprocessors' state save areas:  */
#define XCHAL_CP0_SA_SIZE		0
#define XCHAL_CP1_SA_SIZE		0
#define XCHAL_CP2_SA_SIZE		0
#define XCHAL_CP3_SA_SIZE		0
#define XCHAL_CP4_SA_SIZE		0
#define XCHAL_CP5_SA_SIZE		0
#define XCHAL_CP6_SA_SIZE		0
#define XCHAL_CP7_SA_SIZE		0
/*  Minimum required alignments of CP state save areas:  */
#define XCHAL_CP0_SA_ALIGN		1
#define XCHAL_CP1_SA_ALIGN		1
#define XCHAL_CP2_SA_ALIGN		1
#define XCHAL_CP3_SA_ALIGN		1
#define XCHAL_CP4_SA_ALIGN		1
#define XCHAL_CP5_SA_ALIGN		1
#define XCHAL_CP6_SA_ALIGN		1
#define XCHAL_CP7_SA_ALIGN		1

/*  Indexing macros:  */
#define _XCHAL_CP_SA_SIZE(n)		XCHAL_CP ## n ## _SA_SIZE
#define XCHAL_CP_SA_SIZE(n)		_XCHAL_CP_SA_SIZE(n)	/* n = 0 .. 7 */
#define _XCHAL_CP_SA_ALIGN(n)		XCHAL_CP ## n ## _SA_ALIGN
#define XCHAL_CP_SA_ALIGN(n)		_XCHAL_CP_SA_ALIGN(n)	/* n = 0 .. 7 */


/*  Space for "extra" state (user special registers and non-cp TIE) save area:  */
#define XCHAL_EXTRA_SA_SIZE		0
#define XCHAL_EXTRA_SA_ALIGN		1

/*  Total save area size (extra + all coprocessors)  */
/*  (not useful until xthal_{save,restore}_all_extra() is implemented,  */
/*   but included for Tor2 beta; doesn't account for alignment!):  */
#define XCHAL_CPEXTRA_SA_SIZE_TOR2	0	/* Tor2Beta temporary definition -- do not use */

/*  Combined required alignment for all CP and EXTRA state save areas  */
/*  (does not include required alignment for any base config registers):  */
#define XCHAL_CPEXTRA_SA_ALIGN		1

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
	.endm


	/*
	 *  Macro that expands to the body of a function that
	 *  loads the extra (non-coprocessor) optional/custom state.
	 *	Entry:	a2 = ptr to save area from which to restore extra state
	 *	Exit:	any register a2-a15 (?) may have been clobbered.
	 */
	.macro	xchal_extra_load_funcbody
	.endm


/********************
 *  Macros to save and restore the state of each TIE coprocessor.
 */



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
	.endm


	/*
	 *  Macro that expands to the body of a function
	 *  that loads the selected coprocessor's state (registers etc).
	 *	Entry:	a2 = ptr to save area from which to restore cp state
	 *		a3 = coprocessor number
	 *	Exit:	any register a2-a15 (?) may have been clobbered.
	 */
	.macro	xchal_cpi_load_funcbody
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
#define XCHAL_ICACHE_LINEWIDTH		4
#define XCHAL_DCACHE_LINEWIDTH		4
/*  Size of the cache lines in bytes:  */
#define XCHAL_ICACHE_LINESIZE		16
#define XCHAL_DCACHE_LINESIZE		16
/*  Max for both I-cache and D-cache (used for general alignment):  */
#define XCHAL_CACHE_LINEWIDTH_MAX	4
#define XCHAL_CACHE_LINESIZE_MAX	16

/*  Number of cache sets in log2(lines per way):  */
#define XCHAL_ICACHE_SETWIDTH		8
#define XCHAL_DCACHE_SETWIDTH		8
/*  Max for both I-cache and D-cache (used for general cache-coherency page alignment):  */
#define XCHAL_CACHE_SETWIDTH_MAX	8
#define XCHAL_CACHE_SETSIZE_MAX		256

/*  Cache set associativity (number of ways):  */
#define XCHAL_ICACHE_WAYS		2
#define XCHAL_DCACHE_WAYS		2

/*  Size of the caches in bytes (ways * 2^(linewidth + setwidth)):  */
#define XCHAL_ICACHE_SIZE		8192
#define XCHAL_DCACHE_SIZE		8192

/*  Cache features:  */
#define XCHAL_DCACHE_IS_WRITEBACK	0
/*  Whether cache locking feature is available:  */
#define XCHAL_ICACHE_LINE_LOCKABLE	0
#define XCHAL_DCACHE_LINE_LOCKABLE	0

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
				XTHAL_LAM_NACACHED	XCHAL_SEP \
				XTHAL_LAM_EXCEPTION	XCHAL_SEP \
				XTHAL_LAM_NACACHED	XCHAL_SEP \
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
				XTHAL_SAM_WRITETHRU	XCHAL_SEP \
				XTHAL_SAM_EXCEPTION	XCHAL_SEP \
				XTHAL_SAM_EXCEPTION	XCHAL_SEP \
				XTHAL_SAM_EXCEPTION	XCHAL_SEP \
				XTHAL_SAM_WRITETHRU	XCHAL_SEP \
				XTHAL_SAM_EXCEPTION	XCHAL_SEP \
				XTHAL_SAM_ISOLATE	XCHAL_SEP \
				XTHAL_SAM_EXCEPTION	XCHAL_SEP \
				XTHAL_SAM_WRITETHRU

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

#define XCHAL_NUM_WRITEBUFFER_ENTRIES	4	/* number of write buffer entries */

#define XCHAL_BUILD_UNIQUE_ID		0x00002AD6	/* software build-unique ID (22-bit) */

/*  These definitions describe the hardware targeted by this software:  */
#define XCHAL_HW_CONFIGID0		0xC10FD3FE	/* config ID reg 0 value (upper 32 of 64 bits) */
#define XCHAL_HW_CONFIGID1		0x00402AD6	/* config ID reg 1 value (lower 32 of 64 bits) */
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
#define XCHAL_HAVE_BOOLEANS		0	/* 1 if booleans option configured, 0 otherwise */
#define XCHAL_HAVE_LOOPS		1	/* 1 if zero-overhead loops option configured, 0 otherwise */
/*  Misc instructions:  */
#define XCHAL_HAVE_NSA			1	/* 1 if NSA/NSAU instructions option configured, 0 otherwise */
#define XCHAL_HAVE_MINMAX		0	/* 1 if MIN/MAX instructions option configured, 0 otherwise */
#define XCHAL_HAVE_SEXT			0	/* 1 if sign-extend instruction option configured, 0 otherwise */
#define XCHAL_HAVE_CLAMPS		0	/* 1 if CLAMPS instruction option configured, 0 otherwise */
#define XCHAL_HAVE_MAC16		0	/* 1 if MAC16 option configured, 0 otherwise */
#define XCHAL_HAVE_MUL16		0	/* 1 if 16-bit integer multiply option configured, 0 otherwise */
#define XCHAL_HAVE_MUL32		0	/* 1 if 32-bit integer multiply option configured, 0 otherwise */
#define XCHAL_HAVE_MUL32_HIGH		0	/* 1 if MUL32 option includes MULUH and MULSH, 0 otherwise */
/*#define XCHAL_HAVE_POPC		0*/	/* 1 if CRC instruction option configured, 0 otherwise */
/*#define XCHAL_HAVE_CRC		0*/	/* 1 if POPC instruction option configured, 0 otherwise */

#define XCHAL_HAVE_FP			0	/* 1 if floating point option configured, 0 otherwise */
#define XCHAL_HAVE_SPECULATION		0	/* 1 if speculation option configured, 0 otherwise */
/*#define XCHAL_HAVE_MP_SYNC		0*/	/* 1 if multiprocessor sync. option configured, 0 otherwise */
#define XCHAL_HAVE_PRID			1	/* 1 if processor ID register configured, 0 otherwise */

#define XCHAL_NUM_MISC_REGS		2	/* number of miscellaneous registers (0..4) */


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

