#ifndef __ASM_XTENSA_PTRACE_H
#define __ASM_XTENSA_PTRACE_H

/*
 * include/asm-xtensa/ptrace.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 *	Authors:	Joe Taylor <joe@tensilica.com, joetylr@yahoo.com>
 *			Marc Gauthier
 *			Kevin Chea
 */

#include <xtensa/config/core.h>

/* Registers used by ptrace */
#define REG_A_BASE	0xfc000000
#define REG_AR_BASE	0x04000000
#define REG_PC		0x14000000
#define REG_PS		0x080000e6
#define REG_WB		0x08000048
#define REG_WS		0x08000049
#define REG_LBEG	0x08000000
#define REG_LEND	0x08000001
#define REG_LCOUNT	0x08000002
#define REG_SAR		0x08000003
#define REG_DEPC	0x080000c0
#define	REG_EXCCAUSE	0x080000e8
#define REG_EXCVADDR	0x080000ee
#define SYSCALL_NR	0x1

/* Note: can return illegal A reg numbers, i.e. > 15 */
#define AR_REGNO_TO_A_REGNO(ar_regno, wb) \
	((int)(ar_regno - wb*4) >= 0 ? \
	ar_regno - wb*4 : \
	XCHAL_NUM_AREGS + ar_regno - wb*4)


/*  Offsets from EXCSAVE_1 value:  */
#define TABLE_SAVEA0	0
#define TABLE_SAVEA2	4
#define TABLE_SAVEA3	8
/*  (4 reserved bytes)  */
#define TABLE_CRITICAL	16	/* ptr to critical handler */
#define TABLE_ROM_WIND	20	/* ptr to ROM-vectored window-handler double exception handler */
#define TABLE_FIXUP     24      /* ptr to double-exception fixup code */
/*  (4 reserved bytes)  */
#define TABLE_OFS	32	/* start of user, kernel and double exception handler ptr tables */
/*  3 x 64-entries x 4-byte tables follow from TABLE_OFS.  */

/*  Exception causes/codes?:  */
#define EXC_CODE_USER		0x000
#define EXC_CODE_KERNEL		0x040
#define EXC_CODE_DOUBLE		0x080
#define EXC_CODE_UNHANDLED	0x100

/*  Exception types:  */			/*unhandled quickspill*/
#define EXCTYPE_USER			0	/*    0         0     */
#define EXCTYPE_USER_UNHANDLED		1	/*    1         0     */
#define EXCTYPE_KERNEL			2	/*    0         1     */
#define EXCTYPE_KERNEL_UNHANDLED	3	/*    1         1     */
#define EXCTYPE_DOUBLE_WIND_USER	4	/*    0         0     */
#define EXCTYPE_DOUBLE_WIND_KERNEL	8	/*    0         0     */
/*  Bits:  */
#define EXCTYPE_UNHANDLED_SHIFT		0
#define EXCTYPE_QUICKSPILL_SHIFT	1


#ifndef __ASSEMBLY__

/*
 * This struct defines the way the registers are stored on the
 * kernel stack during a system call or other kernel entry.
 */
struct pt_regs {
	unsigned aregs[16];
	unsigned pc;
	unsigned ps;
	unsigned depc;
	unsigned exccause;
	unsigned exctype;
	unsigned excvaddr;
	unsigned vpri;
	unsigned wmask;
	unsigned wb;		/* WINDOWBASE */
	unsigned ws;		/* WINDOWSTART */
	unsigned reserved0;	/*??? cpenable ???*/
	unsigned reserved1;
	unsigned lbeg;
	unsigned lend;
	unsigned lcount;
	unsigned sar;
	long syscall_nr;
	unsigned char
	    extra[(XCHAL_EXTRA_SA_SIZE+15)&(~15)] __attribute__ ((aligned(16)));
	
	unsigned long padding [4];
};

#ifdef __KERNEL__
#define user_mode(regs) (((regs)->ps & 0x00000020)!=0)
#define instruction_pointer(regs) ((regs)->pc)
extern void show_regs(struct pt_regs *);
#endif /* __KERNEL__ */

#endif /* __ASSEMBLY__ */
#endif /* __ASM_XTENSA_PTRACE_H */
