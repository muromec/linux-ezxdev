/*
 * include/asm-arm/arch-omap1510/innovator_dpm.h  
 * Innovator-specific definitions for DPM
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

#ifndef __ASM_INNOVATOR_DPM_H__
#define __ASM_INNOVATOR_DPM_H__

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
#define DPM_MD_V		0  
#define DPM_MD_DPLL_MULT	1  /* DPLL  freq */
#define DPM_MD_DPLL_DIV		2  /* DPLL  freq */
#define DPM_MD_ARM_DIV		3  /* CPU freq */
#define DPM_MD_TC_DIV		4  /* Traffic Controller freq */
#define DPM_MD_PER_DIV		5  /**/
#define DPM_MD_DSP_DIV		6  /**/
#define DPM_MD_DSPMMU_DIV	7  /**/
#define DPM_MD_LCD_DIV		8  /**/

#define DPM_PP_NBR 9

#ifdef __KERNEL__
#ifndef __ASSEMBLER__

#include <linux/types.h>
#include <linux/proc_fs.h>
#include <asm/arch-omap1510/innovator.h>
#include <asm/arch-omap1510/ck.h>
#include <asm/preem_latency.h>

#define DPM_MD_STATS
typedef __u64 dpm_md_count_t;
typedef __u64 dpm_md_time_t;

#define dpm_md_time() ~read_mputimer1()
	
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
	u16 arm_msk; /* Clock Divider mask */
	u16 arm_ctl; /* Clock Divider register */
	u16 dpll_ctl; /* DPLL 1 Multiplier and Divider Register */
};   

/* Instances of this structure define valid Innovator operating points for DPM.
   Voltages are represented in mV, and frequencies are represented in KHz. */ 

struct dpm_md_opt {
        unsigned int v;         /* Target voltage in mV */
	unsigned int dpll;	/* in KHz */
	unsigned int cpu;	/* CPU frequency in KHz */
	unsigned int tc;	/* in KHz */
	unsigned int per ;	/* in KHz */
	unsigned int dsp;	/* in KHz */
	unsigned int dspmmu;	/* in KHz */
	unsigned int lcd;	/* in KHz */
        unsigned int lpj;	/* New value for loops_per_jiffy */
	struct dpm_regs regs;   /* Register values */
};

void dpm_omap1510_board_setup(void);

typedef void (*dpm_fscaler)(struct dpm_regs *regs);

#define basic_idle(parms) omap1510_pm_idle()

/* Machine-dependent operating point creating/query/setting */

#endif /* __ASSEMBLER__ */

#endif /* __KERNEL__ */
#endif /* __ASM_INNOVATOR_DPM_H__ */












