/*
 * include/asm-i386/dpm.h
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
 * Based on include/asm-ppc/dpm.h by Robert Paulsen.  Copyright (C)
 * 2002, International Business Machines Corporation, All Rights
 * Reserved.
 * */

#ifndef __ASM_DPM_H__
#define __ASM_DPM_H__

#ifndef __ASSEMBLER__
typedef int dpm_state_t;
typedef int dpm_md_pp_t;
#endif

#define __NR_sys_dpm 253

#ifdef __KERNEL__
#ifndef __ASSEMBLER__

struct dpm_opt;
struct dpm_md_idle_parms;

/* A table of processor-dependent routines, must be initialized by
   platform-dependent boot code.  None of the entries (that will actually be
   called) are allowed to be NULL if DPM is enabled. */

struct dpm_md {
	void	(*init)(void);
	int	(*init_opt)(struct dpm_opt *opt);
	int	(*set_opt)(struct dpm_opt *opt, unsigned flags);
	int	(*get_opt)(struct dpm_opt *opt);
	int	(*validate_opt)(struct dpm_opt *opt);
	void	(*idle_set_parms)(struct dpm_md_idle_parms *idle_parms);
	void	(*cleanup)(void);
};
	

/* A table of board-dependent routines, must be initialized by
   platform-dependent boot code.  None of the entries (that will actually be
   called) are allowed to be NULL if DPM is enabled. */

struct dpm_bd {

	/* Initialization function. */

	int (*init)(void);

	/* Exit function. */

	void (*exit)(void);

        /* Check a core voltage for validity on the board.  If the actual
           voltage that will be applied is different, modify the parameter. */

        int (*check_v)(unsigned *v);

        /* Get the current core voltage (in mV) */

        unsigned (*get_v)(void);

	/* Initiate a new core voltage setting prior to frequency scaling.
	   The return value is the number of uS to wait before proceeding with
	   frequency scaling. */

	unsigned (*set_v_pre)(unsigned cur_v, unsigned new_v, int flags);

	/* Initiate a new core voltage setting after frequency scaling.
	   The return value is the number of uS to wait before allowing another
	   change in the operating point (normally 0). */

	unsigned (*set_v_post)(unsigned cur_v, unsigned new_v, int flags);
};

/*  If the PLL multiplier changes the pll must relock.  According to TI, relock
 *  needs 12 us.  We use 20 us to be safe
 */
static inline unsigned
dpm_relock_wait(struct dpm_md_opt *cur, struct dpm_md_opt *new)
{
	/* this should really call a generic platform specific routine */
	return 0;
}

static inline void
dpm_relock_setup(struct dpm_md_opt *md_relock, struct dpm_md_opt *md_new)
{
}

struct dpm_md_idle_parms {
};

/* Machine-specific /proc */

#define DPM_MD_PROC_INIT(proc_dpm_md) dpm_generic_md_proc_init(proc_dpm_md)
#define DPM_MD_PROC_CLEANUP(proc_dpm_md) dpm_generic_md_proc_cleanup(proc_dpm_md)

void dpm_generic_md_proc_init(struct proc_dir_entry *proc_dpm_md);
void dpm_generic_md_proc_cleanup(struct proc_dir_entry *proc_dpm_md);

#endif /* __ASSEMBLER__ */

#endif /* __KERNEL__ */
#endif /* __ASM_DPM_H__ */
