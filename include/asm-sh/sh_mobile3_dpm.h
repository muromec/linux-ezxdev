/*
 * include/asm/sh_mobile3_dpm.h
 *
 * DPM header file.
 *
 * modified from include/asm/sh_mobile_dpm.h by Maksim Syrchin <msyrchin@ru.mvista.com>
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */


#ifndef __ASM_SH_MOBILE3_DPM_H__
#define __ASM_SH_MOBILE3_DPM_H__

#include <asm/time.h>
#include <asm/cpu-regs.h>
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
	 
/* sh73180 machine-dependent operating point initialization vector
   should be located here if any. */

/* To be fixed */
#define tb_ticks_per_second DPM_MD_HZ   

#define DPM_MD_PLL_MULT      0	/* PLL multipler  Must be 1,2,4,6,8*/
#define DPM_MD_I_DIV       1	/* These dividers are in the range 1,2,3,4,6,8,12,16 */
#define DPM_MD_C_DIV       2	/* Not directly divisor (see FRQCR CFC bits) */
#define DPM_MD_B_DIV        3 /* These dividers are in the range 1,2,3,4,6,8,12,16 */	
#define DPM_MD_B3_DIV       4	/* These dividers are in the range 1,2,3,4,6,8,12,16 */
#define DPM_MD_P2_DIV        5	/* These dividers are in the range 1,2,3,4,6,8,12,16 */
#define DPM_MD_P1_DIV        6  /* These dividers are in the range 1,2,3,4,6,8,12,16 */
#define DPM_MD_VIO_DIV        7  /* These dividers are in the range 1-20 */                                                                       
#define DPM_MD_SIU_DIV        8  /* These dividers are in the range 1,2,3,4,6,8,12 */                                                                       
#define DPM_MD_DLL_MULT      9  /* DLL multipler  Must be 0x338,0x262,0x2DC,0x3EF*/

#define DPM_PP_NBR 10

#ifndef __ASSEMBLER__
typedef int dpm_state_t;
typedef int dpm_md_pp_t;


#endif /* __ASSEMBLER__ */

#ifdef __KERNEL__
#ifndef __ASSEMBLER__

#include <linux/types.h>
#include <linux/proc_fs.h>


/* Disable *all* asynchronous interrupts for a super-critical section. */
/* TODO: Just regular save and restore for now, revisit later. */

#define critical_save_and_cli(flags) local_irq_save(flags)
#define critical_restore_flags(flags) restore_flags(flags)


static int frqcr2div_table[] __attribute__((unused)) = { 1, 2, 3, 4, 6, 8, 12, 16 };
static int div2frqcr_table[] __attribute__((unused)) = { 0, 0, 1, 2, 3, 0, 4, 0, 5, 0 ,0 ,0 ,6, 0, 0, 0, 7 };

static dpm_md_pp_t default_md_pp[DPM_PP_NBR] __attribute__((unused)) = { 8,1,2,4,4,4,8,1,1,0x338 } ;

#define get_pll(frqcr) (((frqcr>>24)&0x00000007)+1)
#define get_ifc(frqcr) frqcr2div_table[((frqcr>>20)&0x00000007)]
#define get_cfc(frqcr) ((frqcr>>16) & 0x0000000f)
#define get_bfc(frqcr) frqcr2div_table[((frqcr>>12) & 0x00000007)]
#define get_b3fc(frqcr) frqcr2div_table[((frqcr>>8) & 0x00000007)]
#define get_p2fc(frqcr) frqcr2div_table[((frqcr>>4) & 0x00000007)]
#define get_p1fc(frqcr) frqcr2div_table[(frqcr & 0x00000007)]



#define set_pll(mul) ((mul-1)<<24)
#define set_ifc(div) (div2frqcr_table[div]<<20)
#define set_cfc(div) (div<<16)
#define set_bfc(div) (div2frqcr_table[div]<<12)
#define set_b3fc(div) (div2frqcr_table[div]<<8)
#define set_p2fc(div) (div2frqcr_table[div]<<4)
#define set_p1fc(div) (div2frqcr_table[div])



/*Clock pulse generator control registers*/

struct dpm_md_cpg_regs {
	u32 frqcr;
	u32 sclkcr;
	u32 vclkcr;
	u32 dllfrq;
};

/* Standby control registers*/

struct dpm_md_stb_regs {
	u32 stbcr;
	u32 mstpcr0;
	u32 mstpcr1;
};

/*  
 *  The order is important; *do* not change unless you know what you're doing 
 *  see do_pscale for further reference.
 */

#define md_cr_frqcr  1
#define md_cr_sclk   4
#define md_cr_vclk   6
#define md_cr_dll   7
#define md_cr_sleep  8
#define md_cr_wakeup 10

/* This is not bus; but when pll/pfc changes, 
  we need to redo baud rate settings;
  This should be revisited because 
  baud rate change could take long time 
  (up to 2 serial ticks for 9600 = (2/9600 sec ) */

#define md_cr_scif  30 

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
        int     (*set_opt)(struct dpm_opt *opt, unsigned flags);
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
#endif /* __ASM_SH_MOBILE3_DPM_H__ */

/*
 * Local variables:
 * c-basic-offset: 8
 * End:
 */

