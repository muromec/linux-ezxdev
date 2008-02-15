/*
 * include/asm-arm/arch-mx2ads/mx21_dpm.h
 * MX21-specific definitions for DPM
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
 * Copyright (C) 2004, MontaVista Software <source@mvista.com>
 *
 * Based on arch/ppc/platforms/ibm405lp_dpm.h by Bishop Brock.
 */

#ifndef __ASM_MX21_DPM_H__
#define __ASM_MX21_DPM_H__

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
#define DPM_MD_CPU              0
#define DPM_MD_FCLK             1
#define DPM_MD_HCLK             2

#define DPM_MD_PARAM_NUM       3	/* How many operating point parameters? */
#define DPM_PP_NBR      3

#ifdef __KERNEL__
#ifndef __ASSEMBLER__

#include <linux/types.h>
#include <linux/proc_fs.h>

#define tb_ticks_per_second DPM_MD_HZ	/*defined in dpm-stats.h */

/* *all* asynchronous interrupts for a super-critical section
 are disabled when needed within the MX2 fscale function */
#define critical_save_and_cli(flags) do { \
} while(0)

#define critical_restore_flags(flags) do { \
} while(0)

struct dpm_regs {
	int cpu;     /* CPU Mode */
	int presc;   /* PRESC field of the CRM_CSCR register. Defines FCLK freq */
	int bclkdiv; /* BCLKDIV field of the CRM_CSCR register. Defines HCLK freq */
};

/* Instances of this structure define valid operating points for DPM */
struct dpm_md_opt {
	/* Specified values */
	unsigned int v;		/* Target voltage in mV -unused, but required by DPM core */
	int cpu;		/* CPU mode.
				   Possible values are 0 or -1 - running, 1 - sleep */
	int fclk;		/* Desired FCLK. Affects PRESC field of the CRM_CSCR register.
				   Possible values are in KHz, -1 - keep present setting */
	int hclk;		/* Desired HCLK Affects BCLKDIV field of the CRM_CSCR register.
				   Possible values are in KHz, -1 - keep present setting */
	struct dpm_regs regs;	/* required by DPM core */
};

/* This makes it easy to print out a string of the operating point
   parameter in errors or in dpm_trace for constraint conflicts */
#define DPM_OPP_NAMES                                    \
{ "Sleep Mode", "FCLK", "HCLK" \
}
extern char *dpm_opp_names[DPM_MD_PARAM_NUM];

typedef void (*dpm_fscaler) (struct dpm_regs * regs);

/*this is used in dpm-idle.c.
dpm-idle loop is not supported for MX2
Instead, arch_idle is properly defined and called
from process.c*/
/*#define basic_idle(parms)*/

#endif				/* __ASSEMBLER__ */

#endif				/* __KERNEL__ */
#endif				/* __ASM_MX21_DPM_H__ */
