/*
 * include/asm-arm/arch-pxa/bulverde_dpm.h  
 * Bulverde-specific definitions for DPM
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (C) 2002, MontaVista Software <source@mvista.com>
 *
 * Based on arch/ppc/platforms/ibm405lp_dpm.h by Bishop Brock.
 */
/*
 * Copyright (C) 2005 Motorola Inc.
 *
 * modified by w19962, for EZX platform
 */
#ifndef __ASM_BULVERDE_DPM_H__
#define __ASM_BULVERDE_DPM_H__

/*
 * machine dependent operating state
 *
 * An operating state is a cpu execution state that has implications for power
 * management. The DPM will select operating points based largely on the
 * current operating state.
 *
 * DPM_STATES is the number of supported operating states. Valid operating
 * states are from 0 to DPM_STATES-1 but when setting an operating state the
 * kernel should only specify a state from the set of "base states" and should
 * do so by name.  During the context switch the new operating state is simply
 * extracted from current->dpm_state.
 *
 * task states:
 *
 * APIs that reference task states use the range -(DPM_TASK_STATE_LIMIT + 1)
 * through +DPM_TASK_STATE_LIMIT.  This value is added to DPM_TASK_STATE to
 * obtain the downward or upward adjusted task state value. The
 * -(DPM_TASK_STATE_LIMIT + 1) value is interpreted specially, and equates to
 * DPM_NO_STATE.
 *
 * Tasks inherit their task operating states across calls to
 * fork(). DPM_TASK_STATE is the default operating state for all tasks, and is
 * inherited from init.  Tasks can change (or have changed) their tasks states
 * using the DPM_SET_TASK_STATE variant of the sys_dpm() system call.  */

#define DPM_NO_STATE        -1

#define DPM_RELOCK_STATE     0
#define DPM_IDLE_TASK_STATE  1
#define DPM_IDLE_STATE       2
#define DPM_SLEEP_STATE      3
#define DPM_BASE_STATES      4

#define DPM_TASK_STATE_LIMIT 4
#define DPM_TASK_STATE       (DPM_BASE_STATES + DPM_TASK_STATE_LIMIT)
#define DPM_STATES           (DPM_TASK_STATE + DPM_TASK_STATE_LIMIT + 1)
#define DPM_TASK_STATES      (DPM_STATES - DPM_BASE_STATES)

#define DPM_STATE_NAMES                  \
{ "relock", "idle-task", "idle", "sleep",\
  "task-4", "task-3", "task-2", "task-1",\
  "task",                                \
  "task+1", "task+2", "task+3", "task+4" \
}

/* MD operating point parameters */
#define DPM_MD_V                0  /* Voltage */
#define DPM_MD_PLL_L            1  /* L */
#define DPM_MD_PLL_N            2  /* N	*/
#define DPM_MD_PLL_B            3  /* B */
#define DPM_MD_HALF_TURBO       4  /* Cuts turbo mode in half */
#define DPM_MD_CCCRA            5  /* The A bit in the CCCR is
				      for MEMC clocks */
#define DPM_MD_CPLL_ON          6  /* Are the CPLL on? */
#define DPM_MD_PPLL_ON          7  /* Are the PPLL on? */
#define DPM_MD_SLEEP_MODE       8  /* Sleep mode, from pm.h */

/* this is the number of specifiable operating point parameters */
#define DPM_PP_NBR 9

/* these are calculated operating point values (for specification in
   constraints, for example) */
#define DPM_MD_CPUFREQ          9  /* calculated value */
#define DPM_MD_PLL_LCD         10  /* calculated value */
#define DPM_MD_LPJS            11  /* calculated value */

#define DPM_MD_PARAM_NUM       13  /* How many operating point parameters? */

#ifdef __KERNEL__
#ifndef __ASSEMBLER__

#include <linux/types.h>
#include <linux/proc_fs.h>

#define DPM_MD_STATS
typedef __u64 dpm_md_count_t;
typedef __u64 dpm_md_time_t;

extern unsigned int read_mputimer1(void);
//#define dpm_md_time() read_mputimer1()
#define dpm_md_time() 0

/* mputimer 1 runs @ 6Mhz  6 ticks = 1 microsecond */
#define DPM_MD_HZ 6 * 1000000

/* Hardcode this for now. */

#define tb_ticks_per_second DPM_MD_HZ   

/* Disable *all* asynchronous interrupts for a super-critical section. */

/* TODO: Just regular save and restore for now, revisit later. */

#define critical_save_and_cli(flags) save_flags_cli(flags)
#define critical_restore_flags(flags) restore_flags(flags)

/* The register values only include the bits manipulated by the DPM
   system - other bits that also happen to reside in these registers are
   not represented here.  The layout of struct dpm_regs is used by
   assembly code; don't change it without updating constants further
   below (TODO). */

struct dpm_regs {
	unsigned int	cccr;
	unsigned int	clkcfg;
	unsigned int	voltage;	/*	This is not a register.	*/
};   

/* Instances of this structure define valid Bulverde operating points for DPM.
   Voltages are represented in mV, and frequencies are represented in KHz. */ 

struct dpm_md_opt {
	/* Specified values */
	int v;         /* Target voltage in mV */
	int l;         /* Run Mode to Oscillator ratio */
	int n;         /* Turbo-Mode to Run-Mode ratio */
	int b;         /* Fast Bus Mode */
	int half_turbo;/* Half Turbo bit */
	int cccra;     /* the 'A' bit of the CCCR register,
				   alternate MEMC clock */
	int cpll_enabled; /* CPLL enabled? */
	int ppll_enabled; /* PPLLs enabled? */
	int sleep_mode;

	/* Calculated values */
	unsigned int lcd;	/* in KHz */
	unsigned int lpj;	/* New value for loops_per_jiffy */
	unsigned int cpu;	/* CPU frequency in KHz */
	unsigned int turbo;     /* Turbo bit in clkcfg */

	struct dpm_regs regs;   /* Register values */
};

/* This makes it easy to print out a string of the operating point
   parameter in errors or in dpm_trace for constraint conflicts */
#define DPM_OPP_NAMES                                    \
{ "Voltage", "L", "N", "B", "Half-Turbo", "CCCR[A]",     \
  "CPLL ON","PPLL ON", "Sleep Mode", "", "CPU Freq", "LCD", "LPJs" \
}
extern char *dpm_opp_names[DPM_MD_PARAM_NUM];

void dpm_bulverde_board_setup(void);

typedef void (*dpm_fscaler)(struct dpm_regs *regs);

#define basic_idle(parms) bulverde_pm_idle()

/* Machine-dependent operating point creating/query/setting */

#endif /* __ASSEMBLER__ */

#endif /* __KERNEL__ */
#endif /* __ASM_BULVERDE_DPM_H__ */

