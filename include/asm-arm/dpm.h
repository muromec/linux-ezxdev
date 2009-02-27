/*
 * include/asm-arm/dpm.h        Platform-dependent DPM defines for ARM
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
/*
 * Copyright (C) 2005 Motorola Inc.
 *
 * modified by w19962, for EZX platform
 */

#ifndef __ASM_DPM_H__
#define __ASM_DPM_H__

#ifdef CONFIG_ARCH_OMAP1510  
#include <asm/arch-omap1510/omap1510_dpm.h>
#endif

#ifdef CONFIG_ARCH_OMAP1610
#include <asm/arch-omap1610/omap1610_dpm.h>
#endif

#ifdef CONFIG_ARCH_OMAP730
#include <asm/arch-omap730/omap730_dpm.h>
#endif

#if defined(CONFIG_ARCH_PXA) && defined(CONFIG_CPU_BULVERDE)
#include <asm/arch-pxa/bulverde_dpm.h>
#endif

#ifdef CONFIG_ARCH_MX2ADS
#include <asm/arch-mx2ads/mx21-dpm.h>
#endif

#ifdef CONFIG_ARCH_S3C24A0
#include <asm/arch-s3c24a0/s3c24a0_dpm.h>
#endif

#ifdef CONFIG_ARCH_S3C2440
#include <asm/arch-s3c2440/s3c2440_dpm.h>
#endif

#ifndef __ASSEMBLER__
typedef int dpm_state_t;
typedef int dpm_md_pp_t;
#endif

#define __NR_sys_dpm (__NR_SYSCALL_BASE+ 239)

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
#ifdef CONFIG_ARCH_OMAP
	if (new->regs.dpll_ctl != (cur->regs.dpll_ctl & CK_DPLL_MASK)) 
		return 20;
#endif /* CONFIG_ARCH_OMAP */
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
