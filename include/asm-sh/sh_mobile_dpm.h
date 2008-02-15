/*
 * include/asm/sh_mobile_dpm.h
 *
 * DPM header file.
 *
 * Author: Dmitrij Frasenyak <sed@ru.mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */


#ifndef __ASM_SH_MOBILE_DPM_H__
#define __ASM_SH_MOBILE_DPM_H__

#include <asm/time.h>
#include <asm/sh7300-regs.h>
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
{ "relock", "idle-task", "idle", "sleep", \
  "task-4", "task-3", "task-2", "task-1",\
  "task",                                \
  "task+1", "task+2", "task+3", "task+4" \
}
	 
/* sh7300 machine-dependent operating point initialization vector
   should be located here if any. */

/* To be fixed */
#define tb_ticks_per_second DPM_MD_HZ   

#define DPM_MD_PLL_MULT      0	/* PLL multipler; 
				   Must be 1,2,3,4,6*/
#define DPM_MD_IBUS_DIV       1	/* These dividers are in the range 1,2,3,4,6 */
#define DPM_MD_PBUS_DIV       2	/* These dividers are in the range 1,2,3,4,6 */
#define DPM_MD_SIO_DIV        3	
#define DPM_MD_FCLK_DIV       4	
#define DPM_MD_VIO_DIV        5	

#define DPM_PP_NBR 6

#ifndef __ASSEMBLER__
typedef int dpm_state_t;
typedef int dpm_md_pp_t;


#endif /* __ASSEMBLER__ */

#ifdef __KERNEL__
#ifndef __ASSEMBLER__

#include <linux/types.h>
#include <linux/proc_fs.h>

#define STBCR4_FLCTL (unsigned char)(1 << 7)
#define STBCR5_SIU   (unsigned char)(1 << 1)


/* Disable *all* asynchronous interrupts for a super-critical section. */
/* TODO: Just regular save and restore for now, revisit later. */

#define critical_save_and_cli(flags) local_irq_save(flags)
#define critical_restore_flags(flags) restore_flags(flags)


#if 0 /*We use gettime for now. should be revisited later*/
#ifdef CONFIG_CLOCK_COUNTS_DOWN
#define dpm_md_time() ~((unsigned int)ctrl_inl(TMU1_TCNT))
#else
#define dpm_md_time() ((unsigned int)ctrl_inl(TMU1_TCNT))
#endif
#endif

static int fmult_table[] __attribute__((unused)) = { 1, 2, 3, 4, 6, 8, 12 };
static int frqcr_table[] __attribute__((unused)) = { 0, 0, 1, 2, 3, 0, 4, 0, 5, 0 ,0 ,0 ,6 };
/*PLL IBUS PBUS SIO FCLK VIO */
static dpm_md_pp_t default_md_pp[DPM_PP_NBR] __attribute__((unused)) = { 2,1,4,6,4,1 } ;


#define get_pll(frqcr) fmult_table[((frqcr & 0x0700) >> 8)]
#define get_ifc(frqcr) fmult_table[((frqcr & 0x0070) >> 4)]
#define get_pfc(frqcr) fmult_table[frqcr & 0x0007]

#define set_pll(mul) (frqcr_table[mul] << 8)
#define set_ifc(div) (frqcr_table[div] << 4)
#define set_pfc(div) (frqcr_table[div])

static u16 m0_frqcr_table[] __attribute__((unused)) = { 
	0x1000,0x1001,0x1003,
	0x1101,0x1103,0x1111,0x1113,
	0x1303,0x1305,0x1313,0x1315,0x1333,0x1335,
	0x1404,0x1406,0x1414,0x1416,0x1424,0x1426,0x1444,0x1446
};

/*Clock pulse generator control registers*/

struct dpm_md_cpg_regs {
	u16 frqcr;
	u8 fclkcr; /* 8 bit register.Write is performed by word access*/
	u8 sclkcr;
	u8  vclkcr;
};

/* Standby control registers*/

struct dpm_md_stb_regs {
	u8 cr;
	u8 cr1;
	u8 cr2;
	u8 cr3;
	u8 cr4;
	u8 cr5;
};

/*  
 *  The order is important; *do* not change unless you know what you're doing 
 *  see do_pscale for further reference.
 */

#define md_cr_frqcr  1
#define md_cr_fclk   2
#define md_cr_sclk   4
#define md_cr_vclk   6
#define md_cr_sleep  8
#define md_cr_wakeup 10
#define md_cr_scif  30 /* This is not bus; but when pll/pfc changes, 
			  we need to redo baud rate settings;
			  This should be revisited because 
			  baud rate change could take long time 
			  (up to 2 serial ticks for 9600 = (2/9600 sec )
		          */

struct dpm_regs {
	struct dpm_md_stb_regs stb;
	struct dpm_md_cpg_regs cpg;
	struct sh_cpuinfo *cpuinfo;            /* Pointer to cpu_boot_data */
	unsigned int mask;      /* Used by compute_fscaler */
	unsigned int lpj;	/* New value for loops_per_jiffy */
};	

struct dpm_md_opt {
	unsigned int v;
	struct dpm_regs regs;	               /* DPM regs (CPG and stbcr values) */
};

struct dpm_opt;
struct dpm_md_idle_parms;

/* A table of processor-dependent routines, must be initialized by
   platform-dependent boot code.  None of the entries (that will actually be
   called) are allowed to be NULL when DPM is finished. But for now NULL is ok. */

struct dpm_md {
        void    (*init)(void);
        int     (*init_opt)(struct dpm_opt *opt);
        int     (*set_opt)(struct dpm_opt *opt, int flags);
        int     (*get_opt)(struct dpm_opt *opt);
        void    (*idle_set_parms)(struct dpm_md_idle_parms *idle_parms);
	void    (*fully_define_opt) (struct dpm_md_opt *cur, struct dpm_md_opt *new);
	int     (*validate_opt)(struct dpm_opt *opt);
        void    (*cleanup)(void);
};

/* A table of board-dependent routines, must be initialized by
   platform-dependent boot code.  None of the entries are allowed to be NULL if
   DPM is enabled. Actually all values are NULL for now */

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
/* HW WDT timer is used to count time during PLL switching. */
static inline unsigned
dpm_relock_wait(struct dpm_md_opt *cur, struct dpm_md_opt *new)
{
		return 0;
}

typedef void (*dpm_fscaler)(struct dpm_regs *regs);

extern void dpm_idle(void);


/* Structure containing parameters controlling the power-management idle
   function.  
*/

struct dpm_md_idle_parms {
	int not_used_for_now;
};

int
return_from_idle_immediate(void);

/* Machine-specific /proc 
   We don't support /proc interface. 
   Lines bellow are necessary to compile kernel.
   Should be removed later.
*/

#define DPM_MD_PROC_INIT(proc_dpm_md) dpm_generic_md_proc_init(proc_dpm_md)
#define DPM_MD_PROC_CLEANUP(proc_dpm_md) dpm_generic_md_proc_cleanup(proc_dpm_md)

void dpm_generic_md_proc_init(struct proc_dir_entry *proc_dpm_md);
void dpm_generic_md_proc_cleanup(struct proc_dir_entry *proc_dpm_md);

/* If the platform supports the cryo standby mode, it must define this
   operating point.  This is a PLL bypass operating point which must have
   sufficient capabilities to support console logging.  This operating point is
   entered only just before a cryo shutdown, and is replaced immediately after
   the cryo resume. */

extern struct dpm_opt *dpm_standby_opt;

#endif /* __ASSEMBLER__ */


#endif /* __KERNEL__ */
#endif /* __ASM_SH_MOBILE_DPM_H__ */

/*
 * Local variables:
 * c-basic-offset: 8
 * End:
 */

