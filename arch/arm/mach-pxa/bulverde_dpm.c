/*
 * arch/arm/mach-pxa/bulverde_dpm.c  
 * 
 * Bulverde-specific DPM support
 *
 * Author: <source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 */

#include <linux/config.h>
#include <linux/dpm.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/device.h>

#include <linux/delay.h>
#include <asm/hardirq.h>
#include <asm/page.h>
#include <asm/processor.h>
#include <asm/uaccess.h>
#include <asm/arch/hardware.h>

/* to get the CPUMODE_* enum */
#include <linux/pm.h>

#include "mainstone_dpm.h"
#include "bulverde_freq.h"
#include "bulverde_voltage.h"

#undef DEBUG

#ifdef DEBUG
#define DPRINT(args...) printk(args)
#else
#define DPRINT(args...) do {} while (0)
#endif

struct dpm_bd dpm_bd;
struct dpm_md dpm_md;

static int saved_loops_per_jiffy = 0;
static int saved_cpu_freq = 0;
/* from pm.c */
extern int cpu_mode_set(int mode);

/* For dpm_bulverde_scale and dpm_bulverde_validate_opt to go through
   the LDM device list */
extern struct list_head global_device_list;
#define to_dev(node) container_of(node,struct device,g_list)

#define BULVERDE_DEFAULT_VOLTAGE 1400

/* Array of strings of Operating Point Parameters */
char *dpm_opp_names[DPM_MD_PARAM_NUM] = DPM_OPP_NAMES;

static struct dpm_opt temp_opt = { name : "[System Operating Point]" };

#define DPM_FSCALER_NOP             0
#define DPM_FSCALER_CPUFREQ        (1 << 0)
#define DPM_FSCALER_SLEEP          (1 << 1)
#define DPM_FSCALER_STANDBY        (1 << 2)
#define DPM_FSCALER_DEEPSLEEP      (1 << 3)
#define DPM_FSCALER_WAKEUP         (1 << 4)
#define DPM_FSCALER_VOLTAGE        (1 << 5)
#define DPM_FSCALER_XPLLON         (1 << 6)
#define DPM_FSCALER_HALFTURBO_ON   (1 << 7)
#define DPM_FSCALER_HALFTURBO_OFF  (1 << 8)
#define DPM_FSCALER_TURBO_ON       (1 << 9)
#define DPM_FSCALER_TURBO_OFF      (1 << 10)

#define DPM_FSCALER_TURBO (DPM_FSCALER_TURBO_ON | DPM_FSCALER_TURBO_OFF)

#define DPM_FSCALER_ANY_SLEEPMODE  (DPM_FSCALER_SLEEP | \
                                   DPM_FSCALER_STANDBY | \
				   DPM_FSCALER_DEEPSLEEP)

#define CCCR_CPDIS_BIT_ON          (1 << 31)
#define CCCR_PPDIS_BIT_ON          (1 << 30)
#define CCCR_CPDIS_BIT_OFF         (0 << 31)
#define CCCR_PPDIS_BIT_OFF         (0 << 30)
#define CCCR_PLL_EARLY_EN_BIT_ON   (1 << 26)
#define CCSR_CPLL_LOCKED           (1 << 29) 
#define CCSR_PPLL_LOCKED           (1 << 28)

/* CLKCFG
   | 31------------------------------------------- | 3 | 2  | 1 | 0 |
   | --------------------------------------------- | B | HT | F | T |
*/
#define CLKCFG_B_BIT               (1 << 3)
#define CLKCFG_HT_BIT              (1 << 2)
#define CLKCFG_F_BIT               (1 << 1)
#define CLKCFG_T_BIT               1

static unsigned int dpm_fscaler_flags = 0;

static int dpm_bulverde_fscale(struct dpm_opt *new, unsigned flags);
static void dpm_bulverde_scale(void);

static unsigned long
calculate_memclk(unsigned long cccr, unsigned long clkcfg)
{
	unsigned long M, memclk;
	u32 L;
	
	L = cccr & 0x1f;
	if (cccr & (1 << 25)) {
		if  (clkcfg & CLKCFG_B_BIT)
			memclk = (L*13);
		else
			memclk = (L*13)/2;
	}
	else {
		if (L <= 10) M = 1;
		else if (L <= 20) M = 2;
		else M = 4;

		memclk = (L*13)/M;
	}

	return memclk;
}

static unsigned long
calculate_new_memclk(struct dpm_regs *regs)
{
	return calculate_memclk(regs->cccr, regs->clkcfg);
}

static unsigned long
calculate_cur_memclk(void)
{
	unsigned long cccr = CCCR;
	return calculate_memclk(cccr, bulverde_read_clkcfg());
}

static void mainstone_scale_cpufreq(struct dpm_regs *regs)
{
	unsigned long new_memclk, cur_memclk;
	u32 new_mdrefr, cur_mdrefr, read_mdrefr;
	int set_mdrefr = 0, scaling_up = 0;

	new_memclk = calculate_new_memclk(regs);
	cur_memclk = calculate_cur_memclk();

	new_mdrefr = cur_mdrefr = MDREFR;
	
	if (new_memclk != cur_memclk) {
		DPRINT("%s: changing memclk from %ld to %ld\n",
		       __FUNCTION__, cur_memclk, new_memclk);
		
		if (new_memclk <= 52) {
			/* SDCLK0 = MEMCLK */
			new_mdrefr  &= ~(MDREFR_K0DB2 | MDREFR_K0DB4);
		}
		else if (new_memclk <= 104) {
			/* SDCLK0 = MEMCLK/2 */
			new_mdrefr &= ~MDREFR_K0DB4;
			new_mdrefr |= MDREFR_K0DB2;
		}
		else {
			/* SDCLK0 = MEMCLK/4 */
			new_mdrefr |= MDREFR_K0DB4;
		}

		/* clock increasing or decreasing? */
		if (new_memclk > cur_memclk) scaling_up = 1;
	}
	
	/* set MDREFR if necessary */
	if (new_mdrefr != cur_mdrefr)
		set_mdrefr = 1;

	/* if memclk is scaling up, set MDREFR before freq change */
	if (set_mdrefr && scaling_up) {
		DPRINT("PRE-FREQ: Changing mdrefr from 0x%08X to 0x%08X\n",
		       cur_mdrefr, new_mdrefr);
		MDREFR = new_mdrefr;
		read_mdrefr = MDREFR;
	}

	/*
 	 * Just first set up CCCR and then set clkcfg.
	 */
	CCCR = regs->cccr;
	bulverde_set_freq(regs->clkcfg);

	/* if memclk is scaling down, set MDREFR after freq change */
	if (set_mdrefr && !scaling_up) {
		DPRINT("POST-FREQ: Changing mdrefr from 0x%08X to 0x%08X\n",
		       cur_mdrefr, new_mdrefr);
		MDREFR = new_mdrefr;
		read_mdrefr = MDREFR;
	}
}

static void mainstone_scale_voltage(struct dpm_regs *regs)
{
	bulverde_set_voltage(regs->voltage);
}

static void mainstone_scale_voltage_coupled(struct dpm_regs *regs)
{
	bulverde_prep_set_voltage(regs->voltage);
}

static void xpll_on(struct dpm_regs *regs, int fscaler_flags)
{
	int tmp_cccr, tmp_ccsr;
	int new_cpllon=0, new_ppllon=0, cur_cpllon=0;
	int  cur_ppllon=0, start_cpll=0, start_ppll=0;

	if (!(fscaler_flags & DPM_FSCALER_XPLLON)){
		return;
	}
	tmp_ccsr = CCSR;

	if( (regs->cccr & CCCR_CPDIS_BIT_ON) == 0)
		new_cpllon=1;
	if( (regs->cccr & CCCR_PPDIS_BIT_ON) == 0)
		new_ppllon=1;
	if(  (tmp_ccsr & (0x1 << 31)) == 0 )
		cur_cpllon=1;
	if(  (tmp_ccsr & (0x1 << 30)) == 0 )
		cur_ppllon=1;

	if( (new_cpllon == 1) && (cur_cpllon == 0) ){
	  start_cpll=1;
	}
	if( (new_ppllon == 1) && (cur_ppllon == 0) ){
	  start_ppll=1;
	}

	if( (start_cpll == 0) && (start_ppll == 0) ){
	  printk(KERN_WARNING
	  	"%s:%d  %s NO PLL REQUESTED TO START #2\n",__FILE__,__LINE__,__FUNCTION__);
	  return;
	}
	/* NOTE: the Yellow Book says that exiting 13M mode requires a
	  PLL relock, which takes at least 120uS, so the book suggests
	  the OS could use a timer to keep busy until it is time to
	  check the CCSR bits which must happen before changing the
	  frequency back.

	  For now, we'll just loop.
	*/

	/* 
	   From Yellow Book, page 3-31, section 3.5.7.5 13M Mode
	   
	   Exiting 13M Mode:

	   1. Remain in 13M mode, but early enable the PLL via
              CCCR[CPDIS, PPDIS]=11, and CCCR[PLL_EARLY_EN]=1. Doing
              so will allow the PLL to be started early.

	   2. Read CCCR and compare to make sure that the data was
              correctly written.

	   3. Check to see if CCS[CPLOCK] and CCSR[PPLOCK] bits are
              both set. Once these bits are both high, the PLLs are
              locked and you may move on.

	   4. Note that the CPU is still in 13M mode, but the PLLs are
              started.

	   5. Exit from 13M mode by writing CCCR[CPDIS, PPDIS]=00, but
              maintain CCCR[PLL_EARLY_EN]=1. This bit will be cleared
              by the imminent frequency change.
	   
	*/

	/* Step 1 */
	tmp_cccr = CCCR;
	if(start_cpll)
		tmp_cccr |= CCCR_CPDIS_BIT_ON;
	if(start_ppll)
		tmp_cccr |= CCCR_PPDIS_BIT_ON;
	tmp_cccr |= CCCR_PLL_EARLY_EN_BIT_ON;

	CCCR = tmp_cccr;

	/* Step 2 */
	tmp_cccr = CCCR;

	if ( (tmp_cccr & CCCR_PLL_EARLY_EN_BIT_ON) != CCCR_PLL_EARLY_EN_BIT_ON) {
		printk(KERN_WARNING
			"DPM: Warning: PLL_EARLY_EN is NOT on\n");
	}
	if (  (start_cpll==1) &&
		((tmp_cccr & CCCR_CPDIS_BIT_ON) != CCCR_CPDIS_BIT_ON)  ) {
		printk(KERN_WARNING
			"DPM: Warning: CPDIS is NOT on\n");
	}
	if (  (start_ppll==1) &&
		(tmp_cccr & CCCR_PPDIS_BIT_ON) != CCCR_PPDIS_BIT_ON  ) {
		printk(KERN_WARNING
			"DPM: Warning: PPDIS is NOT on\n");
	}



	/* Step 3 */
	{
		/* Note: the point of this is to "wait" for the lock
		   bits to be set; the Yellow Book says this may take
		   a while, but observation indicates that it is
		   instantaneous */

		long volatile int i=0;
		int cpll_complete=1;
		int ppll_complete=1;
		if(start_cpll==1)
			 cpll_complete=0;
		if(start_ppll==1)
			 ppll_complete=0;
		
		for (i=0; i<999999 /* arbitrary big value to prevent
				      looping forever */; i++) {
			tmp_ccsr = CCSR;

			if( (tmp_ccsr & CCSR_CPLL_LOCKED) ==  CCSR_CPLL_LOCKED ){
			  /*CPLL locked*/
			  cpll_complete=1;
			}
			if( (tmp_ccsr & CCSR_PPLL_LOCKED ) == CCSR_PPLL_LOCKED ){
			  /*PPLL locked*/
			  ppll_complete=1;
			}
			if( (cpll_complete == 1) && (ppll_complete == 1) ){
			  break;
			}
		}
	}

	/* Step 4: NOP */

	/* Step 5 */
	/* Do NOT clear the PLL disable bits here
	 * (just do it when write regs->cccr to CCCR )
	 */

	/* But leave EARLY_EN on; it will be cleared by the frequency change */
	regs->cccr |= CCCR_PLL_EARLY_EN_BIT_ON;

	/* Do not set it now */

	/* Step 6: Now go continue on with frequency change */
}

static void mainstone_fscaler(struct dpm_regs *regs)
{
	unsigned int cccr, clkcfg = 0;

	/* If no flags are set, don't waste time here, just return */
	if (dpm_fscaler_flags == DPM_FSCALER_NOP)
		return;

	/* If exiting 13M mode, do some extra work before changing the
	   CPU frequency or voltage */
	if(dpm_fscaler_flags & DPM_FSCALER_XPLLON)
		xpll_on(regs, dpm_fscaler_flags);

	/* if not sleeping, and have a voltage change */
	/* note that SLEEPMODE will handle voltage itself */
	if (((dpm_fscaler_flags & DPM_FSCALER_ANY_SLEEPMODE) == 0) &&
	    (dpm_fscaler_flags & DPM_FSCALER_VOLTAGE)) {
		if(dpm_fscaler_flags & DPM_FSCALER_CPUFREQ) {
			/* coupled voltage & freq change */
			mainstone_scale_voltage_coupled(regs);
		}
		else {
			/* Scale CPU voltage un-coupled with freq */
			mainstone_scale_voltage(regs);
		}
	}

	if(dpm_fscaler_flags & DPM_FSCALER_CPUFREQ)  /* Scale CPU freq*/
		mainstone_scale_cpufreq(regs);

	if (dpm_fscaler_flags & DPM_FSCALER_TURBO) {

		clkcfg = bulverde_read_clkcfg();

		/* Section 3.5.7 of the Yellow Book says that the F
		   bit will be left on after a FCS, so we need to
		   explicitly clear it. But do not change the B bit */
		clkcfg &= ~(CLKCFG_F_BIT);

		if (dpm_fscaler_flags & DPM_FSCALER_TURBO_ON) {
			clkcfg = clkcfg | (CLKCFG_T_BIT);
		}
		else {
			clkcfg = clkcfg & ~(CLKCFG_T_BIT);
		}

		/* enable */
		bulverde_set_freq(clkcfg);
	}

	if ((dpm_fscaler_flags & DPM_FSCALER_HALFTURBO_ON) ||
	    (dpm_fscaler_flags & DPM_FSCALER_HALFTURBO_OFF)) {
		if ((dpm_fscaler_flags & DPM_FSCALER_CPUFREQ) ||
		    (dpm_fscaler_flags & DPM_FSCALER_VOLTAGE)) {
			/*
			 * From the Yellow Book, p 3-106:
			 * 
			 * "Any two writes to CLKCFG or PWRMODE
			 * registers must be separated by siz 13-MHz
			 * cycles.  This requirement is achieved by
			 * doing the write to the CLKCFG or POWERMODE
			 * reigster, performing a read of CCCR, and
			 * then comparing the value in the CLKCFG or
			 * POWERMODE register to the written value
			 * until it matches."
			 *
			 * Since the setting of half turbo is a
			 * separate write to CLKCFG, we need to adhere
			 * to this requirement.
			 */
			cccr = CCCR;
			clkcfg = bulverde_read_clkcfg();
			while (clkcfg != regs->clkcfg)
				clkcfg = bulverde_read_clkcfg();
		}

		if (clkcfg == 0)
			clkcfg = regs->clkcfg;
		/* Turn off f-bit.

		   According to the Yellow Book, page 3-23, "If only
		   HT is set, F is clear, and B is not altered, then
		   the core PLL is not stopped." */

		clkcfg = clkcfg & ~(CLKCFG_F_BIT);
		/* set half turbo bit */
		if (dpm_fscaler_flags & DPM_FSCALER_HALFTURBO_ON) {
			clkcfg = clkcfg | (CLKCFG_HT_BIT);
		}
		else {
			clkcfg = clkcfg & ~(CLKCFG_HT_BIT);
		}

		/* enable */
		bulverde_set_freq(clkcfg);
	}

	/* Devices only need to scale on a core frequency
	   change. Half-Turbo changes are separate from the regular
	   frequency changes, so Half-Turbo changes do not need to
	   trigger a device recalculation.

	   NOTE: turbo-mode-only changes could someday also be
	   optimized like Half-Turbo (to not trigger a device
	   recalc). */
	if (dpm_fscaler_flags & DPM_FSCALER_CPUFREQ) {
		/* Allow drivers to update themselves if necessary */
		dpm_bulverde_scale();
	}

	if(dpm_fscaler_flags & DPM_FSCALER_ANY_SLEEPMODE) {

		/* NOTE: voltage needs i2c, so be sure to change
		   voltage *before* calling device_suspend */

		if(dpm_fscaler_flags & DPM_FSCALER_VOLTAGE) {
			/* Scale CPU voltage un-coupled with freq */
			mainstone_scale_voltage(regs);
		}

		DPRINT("\n\nPrior to sleep, MDREFR=0x%08X\n\n", MDREFR);
		device_suspend(0, SUSPEND_POWER_DOWN);

		if(dpm_fscaler_flags & DPM_FSCALER_SLEEP) {
			cpu_mode_set(CPUMODE_SLEEP);
		}
		else if(dpm_fscaler_flags & DPM_FSCALER_STANDBY){
			cpu_mode_set(CPUMODE_STANDBY);
		}
		else if(dpm_fscaler_flags & DPM_FSCALER_DEEPSLEEP){
			cpu_mode_set(CPUMODE_DEEPSLEEP);
		}
		
	 	/* Here when we wake up. */

		/* Power on devices again */
		device_resume(RESUME_POWER_ON);
		DPRINT("\n\nAfter sleep, MDREFR=0x%08X\n\n", MDREFR);

		/* Recursive call to switch back to to task state. */
		dpm_set_os(DPM_TASK_STATE);
	}
}

static dpm_fscaler fscalers[1] = { 
	mainstone_fscaler,
};

/* This routine computes the "forward" frequency scaler that moves the system
 * from the current operating point to the new operating point.  The resulting
 * fscaler is applied to the registers of the new operating point. 
 */

dpm_fscaler
compute_fscaler(struct dpm_md_opt *cur, struct dpm_md_opt *new)
{
	int current_n, ccsr;

	ccsr = CCSR;
	current_n = (ccsr & CCCR_N_MASK)>>7;

	dpm_fscaler_flags = DPM_FSCALER_NOP;

	/* If new CPU is 0, that means sleep */
	if (!new->cpu) {
		if (new->sleep_mode == CPUMODE_DEEPSLEEP) {
			dpm_fscaler_flags |= DPM_FSCALER_DEEPSLEEP;
		}
		else if (new->sleep_mode == CPUMODE_STANDBY) {
			dpm_fscaler_flags |= DPM_FSCALER_STANDBY;
		}
		else {
			dpm_fscaler_flags |= DPM_FSCALER_SLEEP;
		}
	}

	/* If exiting 13M mode, set the flag so we can do the extra
	   work to get out before the frequency change */
	if( ((cur->cpll_enabled == 0) && (new->cpll_enabled ==1)) ||
		((cur->ppll_enabled == 0) && (new->ppll_enabled ==1)) ){
		dpm_fscaler_flags |= DPM_FSCALER_XPLLON;
		/* Don't it better to read xpll_enabled from registers instead? */
	}


	/* if CPU is *something*, it means we are not going to sleep */
	if ((new->cpu) && 
	    /* And something has indeed changed */
	    ((new->regs.cccr != cur->regs.cccr) ||
	     (new->regs.clkcfg != cur->regs.clkcfg))) {

		/* Find out if it is *just* a turbo bit change */
		if ((cur->l == new->l) &&
		    (cur->cccra == new->cccra) &&
		    (cur->b == new->b) &&
		    (cur->half_turbo == new->half_turbo)) {
			
			/* If the real, current N is a turbo freq and
			   the new N is not a turbo freq, then set
			   TURBO_OFF and do not change N */
			if ((cur->n > 1) && (new->n == 2)) {
				dpm_fscaler_flags |= DPM_FSCALER_TURBO_OFF;
			}
			/* Else if the current operating point's N is
			   not-turbo and the new N is the desired
			   destination N, then set TURBO_ON
			*/
			else if ((cur->n == 2) && (new->n == current_n)) {
				/* Desired N must be what is current
				   set in the CCCR/CCSR */
				dpm_fscaler_flags |= DPM_FSCALER_TURBO_ON;
			}
			/* Else, fall through to regular FCS */
		}

		if (!(dpm_fscaler_flags & DPM_FSCALER_TURBO)) {
			/* It this is not a Turbo bit only change, it
			   must be a regular FCS  */
			dpm_fscaler_flags |= DPM_FSCALER_CPUFREQ;
		}

		loops_per_jiffy = new->lpj;
	}

	if (new->half_turbo != cur->half_turbo) {
		loops_per_jiffy = new->lpj;
		if (new->half_turbo)
			dpm_fscaler_flags |= DPM_FSCALER_HALFTURBO_ON;
		else 
			dpm_fscaler_flags |= DPM_FSCALER_HALFTURBO_OFF;
	}

	if (new->regs.voltage != cur->regs.voltage)
		dpm_fscaler_flags |= DPM_FSCALER_VOLTAGE;
	
	return fscalers[0];
}

/* Initialize the machine-dependent operating point from a list of parameters,
   which has already been installed in the pp field of the operating point.
   Some of the parameters may be specified with a value of -1 to indicate a
   default value. */

#define DPLL_DIV_MAX 4
#define DPLL_MULT_MAX 31

#define	PLL_L_MAX	31
#define	PLL_N_MAX	8

/* The MIN for L is 2 in the Yellow Book tables, but L=1 really means
   13M mode, so L min includes 1 */
#define	PLL_L_MIN	1
#define	PLL_N_MIN	2

/* NOTE: This inherited function (from OMAP) claims to be delicately
   balanced so as not to overflow...but on Bulverde, it does overflow,
   although the rounding is very, very small: less than .01% of the
   actual value.

   For example, going from 91Mhz to 403Mhz should result in the LPJ
   going from 453632 to 2008941, but it really goes to 2008800; a
   rounding error of just 0.0070%
*/
static unsigned long compute_lpj(unsigned long ref, u_int div, u_int mult)
{
	unsigned long new_jiffy_l, new_jiffy_h;

	/*
	 * Recalculate loops_per_jiffy.  We do it this way to
	 * avoid math overflow on 32-bit machines.  Maybe we
	 * should make this architecture dependent?  If you have
	 * a better way of doing this, please replace!
	 *
	 *    new = old * mult / div
	 */
	new_jiffy_h = ref / div;
	new_jiffy_l = (ref % div) / 100;
	new_jiffy_h *= mult;
	new_jiffy_l = new_jiffy_l * mult / div;

	return new_jiffy_h + new_jiffy_l * 100;
}

static void
calculate_lcd_freq(struct dpm_md_opt *opt)
{
	int k=1;         /* lcd divisor */
	
	/* L is verified to be between PLL_L_MAX and PLL_L_MIN in
	   dpm_bulverde_init_opt() */

	if (opt->l == -1) {
		opt->lcd = -1;
		return;
	}

	if (opt->l > 16) {
		/* When L=17-31, K=4 */
		k = 4;
	}
	else if (opt->l > 7) {
		/* When L=8-16, K=2 */
		k = 2;
	}

	/* Else, when L=2-7, K=1 */
		
	opt->lcd = 13000 * opt->l / k;
}

static void
calculate_reg_values(struct dpm_md_opt *opt)
{
	int f = 0;     /* frequency change bit */
	int turbo = 0; /* turbo mode bit; depends on N value */

	opt->regs.voltage = opt->v;

/* CCCR
   | 31| 30|29-28| 27| 26| 25|24-11| 10| 9 | 8 | 7 |6-5  | 4 | 3 | 2 | 1 | 0 |
   | C | P |     | L | P |   |     |               |     |                   |
   | P | P |     | C | L |   |     |               |     |                   |
   | D | D |resrv| D | L | A |resrv|  2 * N        |resrv|    L              |
   | I | I |     | 2 | . |   |     |               |     |                   |
   | S | S |     | 6 | . |   |     |               |     |                   |

   A: Alternate setting for MEMC clock
      0 = MEM clock frequency as specified in YB's table 3-7
      1 = MEM clock frq = System Bus Frequency
*/

/* CLKCFG
   | 31------------------------------------------- | 3 | 2  | 1 | 0 |
   | --------------------------------------------- | B | HT | F | T |

   B = Fast-Bus Mode  0: System Bus is half of run-mode
                      1: System Bus is equal to run-mode 
                      NOTE: only allowed when L <= 16

   HT = Half-Turbo    0: core frequency = run or turbo, depending on T bit
                      1: core frequency = turbo frequency / 2
                      NOTE: only allowed when 2N = 6 or 2N = 8

   F = Frequency change
                      0: No frequency change is performed
                      1: Do frequency-change 

   T = Turbo Mode     0: CPU operates at run Frequency
                      1: CPU operates at Turbo Frequency (when n2 > 2)
*/

	/* Set the CLKCFG with B, T, and HT */
	if (opt->b != -1 && opt->n != -1) {
		f = 1;

		/* When N2=2, Turbo Mode equals Run Mode, so it
		   does not really matter if this is >2 or >=2 */
		if (opt->n > 2) {
			turbo = 0x1;
		}

		opt->regs.clkcfg = (opt->b<<3) + (f << 1) + turbo;
	}
	else {
		f = 0x1;
		opt->regs.clkcfg = (f << 1);
	}
	
	/* What about when n2=0 ... it is not defined by the yellow
	   book */
	if (opt->n != -1) {
		/* N2 is 4 bits, L is 5 bits */
		opt->regs.cccr = ((opt->n & 0xF) << 7) + (opt->l & 0x1F);
	}

	if (opt->cccra > 0) {
		/* Turn on the CCCR[A] bit */
		opt->regs.cccr |= (1 << 25);
	}

	/* 13M Mode */
	if (opt->l == 1) {
		/* Turn on the pPDIS and cPDIS bits */
		DPRINT("calculate_reg_values: TO 13M MODE\n");
	}

	if ( (opt->l > 1) && (opt->cpll_enabled == 0) ) {
		printk(KERN_WARNING
			"DPM: internal error if l>1 CPLL must be On\n");
	}
	if( (opt->cpll_enabled == 1) && (opt->ppll_enabled == 0) ){
		printk(KERN_WARNING
			"DPM: internal error CPLL=On PPLL=Off is NOT supported in hardware\n");
	}
	if(opt->cpll_enabled == 0) {
		opt->regs.cccr |= (CCCR_CPDIS_BIT_ON);
	}
	if(opt->ppll_enabled == 0) {
		opt->regs.cccr |= (CCCR_PPDIS_BIT_ON);
	}
}

int
dpm_bulverde_init_opt(struct dpm_opt *opt)
{
	int v	       = opt->pp[DPM_MD_V];
	int l 	       = opt->pp[DPM_MD_PLL_L];
	int n2 	       = opt->pp[DPM_MD_PLL_N]; /* 2*N */
	int b 	       = opt->pp[DPM_MD_PLL_B]; /* Fast bus mode bit. */
	int half_turbo = opt->pp[DPM_MD_HALF_TURBO];
	int cccra      = opt->pp[DPM_MD_CCCRA]; /* Alternate setting
						  for the MEM clock */
	int cpll_on    = opt->pp[DPM_MD_CPLL_ON];
	int ppll_on    = opt->pp[DPM_MD_PPLL_ON];
	int sleep_mode = opt->pp[DPM_MD_SLEEP_MODE];
	struct dpm_md_opt *md_opt = &opt->md_opt;

	/* Up-front error checking.  If we fail any of these, then the
	   whole operating point is suspect and therefore invalid. */
	
	if (((l && n2) == 0) &&
	    (l || n2) != 0) {
		/* If one of L or N2 is 0, but they are not both 0 */
		printk(KERN_WARNING
		       "DPM: L/N (%d/%d) must both be 0 or both be non-zero\n",
		       l, n2);
		return -EINVAL;
	}

	if( (l > PLL_L_MIN) && ( cpll_on == 0 ) ){
		printk(KERN_WARNING
			"DPM: when l>0 (NOT 13M mode) CPLL must be On \n");
		return -EINVAL;
	}

	if( (cpll_on>0) && (ppll_on==0) ){
		printk(KERN_WARNING
			"DPM: illegal combination CPLL=On PPLL=Off\n");
		return -EINVAL;
	}

	/* Standard range checking */
	if (((l > 0) && (n2 > 0)) && /* Don't complain about 0, it means sleep */
	    ((l > PLL_L_MAX) ||
	     (n2 > PLL_N_MAX) ||
	     (l < PLL_L_MIN) ||
	     (n2 < PLL_N_MIN))) {
		/* Range checking */
		printk(KERN_WARNING
		       "DPM: L/N (%d/%d) out of range, L=1-31, N=2-8 \n",
		       l, n2);
		return -EINVAL;
	}

	/* If this is for 13M mode, do some more checking */
	if (l == PLL_L_MIN) {
		/* NOTE: the Yellow Book does not require any
		  particular setting for N, but we think it really
		  should be 2 */
		if (n2 != 2) {
			printk(KERN_WARNING
			       "DPM: When L=1 (13M Mode), N must be 2 (%d)\n",
			       n2);
			return -EINVAL;
		}

		if (cpll_on != 0) {
			printk(KERN_WARNING
			       "DPM: When L=1 (13M Mode), CPLL must be OFF (%d)\n",
			       cpll_on);
			return -EINVAL;
		}

		/* Page 3-32, section 3.5.7.5.2 of the Yellow Book
		   says, "Notes: Other bits in the CLKCFG can not be
		   changed while entering or exiting the 13M
		   mode. While in 13M mode, it is illegal to write to
		   CLKCFG's B, HT, or T bits" */
		if ((b > 0) || (half_turbo > 0)){
			printk(KERN_WARNING
			       "DPM: When L=1 (13M Mode), B (%d) and "
			       "Half-Turbo (%d) must be off\n",
			       b, half_turbo);
			return -EINVAL;
		}
	}

	if (half_turbo > 1) {
		printk(KERN_WARNING "DPM: Half-Turbo must be 0 or 1 (%d)\n",
		       half_turbo);
		return -EINVAL;
	}

	if (b > 1) {
		printk(KERN_WARNING "DPM: Fast-Bus Mode (B) must be 0 or 1 (%d)\n",
		       b);
		return -EINVAL;
	}

	/* 2800002.pdf 3.5.7.1 It is illegal to set B if CCCR[CPDIS] is set. */
	if( cpll_on==0 && b == 1){
		printk(KERN_WARNING
		       "DPM: fast bus (b=%d) must both be 0 if CPLL is Off\n",
		       b);
		return -EINVAL;
	}

	if (cccra > 1) {
		printk(KERN_WARNING
		       "DPM: CCCR[A] (alternate MEMC clock) must be 0 or 1 (%d)\n",
		       cccra);
		return -EINVAL;
	}

	/* This (when CCCR[A] is on and FastBus is on, L must be <=16)
	   is explicitly stated in text at the bottom of one of the
	   CPU frequency tables--the one where CCCR[A] is on */
	if ((b == 1) && (cccra == 1) && (l > 16)) {
		printk(KERN_WARNING
		       "DPM: when B=1 and CCCR[A]=1, L must be <= 16 (L is %d)\n", l);
		return -EINVAL;
	}

	/* This one is not explicitly stated the Yellow Book as a bad
	   thing (as the previous restriction is), but according to
	   the CPU frequency tables, fast bus mode *cannot* be
	   supported, even when CCCR[A] is not 1. */
	if ((b == 1) && (l > 16)) {
		printk(KERN_WARNING
		       "DPM: when B=1, L must be <= 16 (L is %d)\n", l);
		return -EINVAL;
	}

	if ((half_turbo == 1) && (n2 != 6) && (n2 != 8)) {
		printk(KERN_WARNING
		       "DPM: Half Turbo only allowed when N2 is 6 or 8\n"
		       "(N2 is %d)\n", n2);
		return -EINVAL;
	}

	/* Check Sleep Mode versus modes from pm.h */
	/* NOTE: CPUMODE_SENSE is not implemented */
	if ((l == 0) && (n2 == 0) && /* Only check if sleeping */
	    (sleep_mode != CPUMODE_STANDBY) &&
	    (sleep_mode != CPUMODE_SLEEP) &&
	    (sleep_mode != CPUMODE_DEEPSLEEP)) {
		printk(KERN_WARNING
		       "DPM: Sleep Mode value %d is not allowed"
		       " (only %d, %d, or %d)\n",
		       sleep_mode,
		       CPUMODE_STANDBY, CPUMODE_SLEEP, CPUMODE_DEEPSLEEP);
		return -EINVAL;
	}

	/* save the values for this operating point */
	md_opt->v = v;
	md_opt->l = l;
	md_opt->n = n2;
	md_opt->b = b;
	md_opt->cccra = cccra;
	md_opt->half_turbo = half_turbo;
	md_opt->cpll_enabled = cpll_on;
	md_opt->ppll_enabled = ppll_on;
	md_opt->sleep_mode = sleep_mode;
	calculate_lcd_freq(md_opt);

	if ((md_opt->l == -1) || (md_opt->n == -1)) {
		md_opt->cpu = -1;
	}
	else {
		/* shift 1 to divide by 2 because opt->n is 2*N */
		md_opt->cpu = (13000 * md_opt->l * md_opt->n) >> 1;
		if (md_opt->half_turbo == 1) {
			/* divide by 2 */
			md_opt->cpu = md_opt->cpu >> 1;
		}
	}

	return 0;
}

void dpm_bulverde_fully_define_opt(struct dpm_md_opt *cur, struct dpm_md_opt *new);

int
dpm_bulverde_set_opt(struct dpm_opt *new, unsigned flags)
{
	unsigned target_v;
	struct dpm_opt *cur;
	struct dpm_md_opt *md_cur, *md_new;

	/* Support for setting the operating point when DPM is not
	   running, and setting the first operating point. */

	if (!dpm_enabled || !dpm_active_opt) {
		DPRINT("dpm_default_set_opt: DPM not enabled or no active opt!\n");
		if (dpm_md_get_opt(&temp_opt)) {
			printk(KERN_ERR "dpm_default_set_opt: "
			      "DPM disabled and system "
			      "operating point is illegal!\n");
			return -EINVAL;
		}
		dpm_active_opt = &temp_opt;
	}
	cur = dpm_active_opt;

	DPRINT("dpm_default_set_opt: %s => %s\n", cur->name, new->name);

	md_cur = &cur->md_opt;
	md_new = &new->md_opt;

	/* fully define the new opt, if necessary, based on values
	   from the current opt */
	dpm_bulverde_fully_define_opt(md_cur, md_new);

	target_v = md_new->v;

	/* In accordance with Yellow Book section 3.7.6.3, "Coupling
	   Voltage Change with Frequency Change", always set the
	   voltage first (setting the FVC bit in the PCFR) and then do
	   the frequency change */

	dpm_bd_set_v_pre(md_cur->v, target_v, flags);
	return dpm_bulverde_fscale(new, flags);
}

static int
dpm_bulverde_fscale(struct dpm_opt *new, unsigned flags)
{
	struct dpm_opt *cur = dpm_active_opt;
	struct dpm_md_opt *md_cur = &cur->md_opt;
	struct dpm_md_opt *md_new = &new->md_opt;
	dpm_fscaler fscaler;
	unsigned s;

	fscaler = compute_fscaler(md_cur, md_new);
	critical_save_and_cli(s);

	fscaler(&md_new->regs);

#ifdef CONFIG_DPM_OPT_STATS
	dpm_update_stats(&new->stats, &dpm_active_opt->stats);
#endif
	dpm_active_opt = new;

	critical_restore_flags(s);

	mb();
	if (flags & DPM_UNLOCK)
		dpm_unlock();

	return 0;
}

/* Fully determine the current machine-dependent operating point, and fill in a
   structure presented by the caller. This should only be called when the
   dpm_sem is held. This call can return an error if the system is currently at
   an operating point that could not be constructed by dpm_md_init_opt(). */

void
bulverde_get_current_info(struct dpm_md_opt *opt)
{
	unsigned int tmp_cccr;
	unsigned int cpdis;
	unsigned int ppdis;

	/* You should read CCSR to see what's up...but there is no A
	   bit in the CCSR, so we'll grab it from the CCCR. */
	tmp_cccr = CCCR;
	opt->cccra = (tmp_cccr >> 25) & 0x1; /* cccr[A]: bit 25 */

	/* NOTE: the current voltage is not obtained, but will be left
	   as 0 in the opt which will mean no voltage change at all */

	opt->regs.cccr = CCSR;

	opt->l = opt->regs.cccr & CCCR_L_MASK;       /* Get L */
	opt->n = (opt->regs.cccr & CCCR_N_MASK)>>7;  /* Get 2N */
	
	/* This should never really be less than 2 */
	if (opt->n < 2) {
		opt->n=2;
	}

	opt->regs.clkcfg = bulverde_read_clkcfg();
	opt->b = (opt->regs.clkcfg >> 3) & 0x1; /* Fast Bus (b): bit 3 */
	opt->turbo = opt->regs.clkcfg & 0x1; /* Turbo is bit 1 */
	opt->half_turbo = (opt->regs.clkcfg >> 2) & 0x1; /* HalfTurbo: bit 2 */

	calculate_lcd_freq(opt);

	/* are the PLLs on? */
	cpdis = ((opt->regs.cccr >> 31) & 0x1);
	ppdis = ((opt->regs.cccr >> 30) & 0x1);
	if ((cpdis == 0) && (ppdis != 0)) {
		/* CPLL=On PPLL=Off is NOT supported with hardware*/
		printk(KERN_WARNING
			"DPM: cpdis and ppdis are not in sync!\n");
	}
	opt->cpll_enabled = (cpdis == 0);
	opt->ppll_enabled = (ppdis == 0);

	/* Shift 1 to divide by 2 (because opt->n is really 2*N */
	if (opt->turbo) {
		opt->cpu = (13000 * opt->l * opt->n) >> 1;
	}
	else {
		/* turbo bit is off, so skip N multiplier (no matter
		   what N really is) and use Run frequency (13K * L) */
		opt->cpu = 13000 * opt->l;
	}
}

int dpm_bulverde_get_opt(struct dpm_opt *opt)
{
	struct dpm_md_opt *md_opt = &opt->md_opt;

	bulverde_get_current_info(md_opt);	

	return 0;
}

void dpm_bulverde_fully_define_opt(struct dpm_md_opt *cur,
				   struct dpm_md_opt *new)
{
	if (new->v == -1)
          new->v = cur->v;
	if (new->l == -1)
          new->l = cur->l;
	if (new->n == -1)
          new->n = cur->n;
	if (new->b == -1)
          new->b = cur->b;
	if (new->half_turbo == -1)
          new->half_turbo = cur->half_turbo;
	if (new->cccra == -1)
          new->cccra = cur->cccra;
	if (new->cpll_enabled == -1)
          new->cpll_enabled = cur->cpll_enabled;
	if (new->ppll_enabled == -1)
          new->ppll_enabled = cur->ppll_enabled;
	if (new->sleep_mode == -1)
          new->sleep_mode = cur->sleep_mode;
	
	if (new->n > 2) {
		new->turbo = 1;
		/* turbo mode: 13K * L * (N/2) 

		   Shift at the end to divide N by 2 for Turbo mode or
		   by 4 for Half-Turbo mode ) 
		*/
		new->cpu = (13000 * new->l * new->n) >> 
			   ((new->half_turbo == 1)?2:1);
	}
	else {
		new->turbo = 0;
		/* run mode */
		new->cpu = 13000 * new->l;
	}

	/* lcd freq is derived from L */
	calculate_lcd_freq(new);

	calculate_reg_values(new);

	/* We want to keep a baseline loops_per_jiffy/cpu-freq ratio
	   to work off of for future calculations, especially when
	   emerging from sleep when there is no current cpu frequency
	   to calculate from (because cpu-freq of 0 means sleep). 
	*/
	if (!saved_loops_per_jiffy) {
		saved_loops_per_jiffy = loops_per_jiffy;
		saved_cpu_freq = cur->cpu;
	}

	if (new->cpu) {
		/* Normal change (not sleep), just compute. Always use
		   the "baseline" lpj and freq */
		new->lpj = compute_lpj(saved_loops_per_jiffy, saved_cpu_freq,
				       new->cpu);
	}
	else {
		/* If sleeping, keep the old LPJ */
		new->lpj = loops_per_jiffy;
	}
}

static int
valid_opt(struct constraints *constraints, struct dpm_opt *opt)
{
	struct dpm_md_opt *md_opt = &opt->md_opt;
	struct constraint_param *param;
	int i;

	if (!constraints->asserted) {
		return 1;
	}

	for (i = 0;
	     (i < constraints->count);
	     i++) {
		param = &constraints->param[i];
		
		switch (param->id) {
		case DPM_MD_V:
			if ((md_opt->v != -1) &&
			    ((param->min > md_opt->v) ||
			     (param->max < md_opt->v))) {

				/* dpm_trace for CONSTRAINT_ASSERTED
				   needs parameter ID, curr value, and
				   constraint min and max */
				dpm_trace(DPM_TRACE_CONSTRAINT_ASSERTED,
					  param->id, md_opt->v,
					  param->min, param->max);
				return 0;
			}
			break;
		case DPM_MD_PLL_L:
			if ((md_opt->l != -1) &&
			    ((param->min > md_opt->l) ||
			     (param->max < md_opt->l))) {
				dpm_trace(DPM_TRACE_CONSTRAINT_ASSERTED,
					  param->id, md_opt->l,
					  param->min, param->max);
				return 0;
			}
			break;
		case DPM_MD_PLL_N:
			if ((md_opt->n != -1) &&
			    ((param->min > md_opt->n) ||
			     (param->max < md_opt->n))) {
				dpm_trace(DPM_TRACE_CONSTRAINT_ASSERTED,
					  param->id, md_opt->n,
					  param->min, param->max);
				return 0;
			}
			break;
		case DPM_MD_PLL_B:
			if ((md_opt->b != -1) &&
			    ((param->min > md_opt->b) ||
			     (param->max < md_opt->b))) {
				dpm_trace(DPM_TRACE_CONSTRAINT_ASSERTED,
					  param->id, md_opt->b,
					  param->min, param->max);
				return 0;
			}
			break;
		case DPM_MD_HALF_TURBO:
			if ((md_opt->half_turbo != -1) &&
			    ((param->min > md_opt->half_turbo) ||
			     (param->max < md_opt->half_turbo))) {
				dpm_trace(DPM_TRACE_CONSTRAINT_ASSERTED,
					  param->id, md_opt->half_turbo,
					  param->min, param->max);
				return 0;
			}
			break;
		case DPM_MD_PLL_LCD:

			if ((md_opt->lcd != -1) &&
			    ((param->min > md_opt->lcd) ||
			     (param->max < md_opt->lcd))) {
				dpm_trace(DPM_TRACE_CONSTRAINT_ASSERTED,
					  param->id, md_opt->lcd,
					  param->min, param->max);
				return 0;
			}
			break;
		case DPM_MD_CCCRA:
			if ((md_opt->cccra != -1) &&
			    ((param->min > md_opt->cccra) ||
			     (param->max < md_opt->cccra))) {
				dpm_trace(DPM_TRACE_CONSTRAINT_ASSERTED,
					  param->id, md_opt->cccra,
					  param->min, param->max);
				return 0;
			}
			break;
		case DPM_MD_CPLL_ON:
			if ((md_opt->cpll_enabled != -1) &&
			    ((param->min > md_opt->cpll_enabled) ||
			     (param->max < md_opt->cpll_enabled))) {
				dpm_trace(DPM_TRACE_CONSTRAINT_ASSERTED,
					  param->id, md_opt->cpll_enabled,
					  param->min, param->max);
				return 0;
			}
			break;
		case DPM_MD_PPLL_ON:
			if ((md_opt->ppll_enabled != -1) &&
			    ((param->min > md_opt->ppll_enabled) ||
			     (param->max < md_opt->ppll_enabled))) {
				dpm_trace(DPM_TRACE_CONSTRAINT_ASSERTED,
					  param->id, md_opt->ppll_enabled,
					  param->min, param->max);
				return 0;
			}
			break;
		case DPM_MD_SLEEP_MODE:
			if ((md_opt->sleep_mode != -1) &&
			    ((param->min > md_opt->sleep_mode) ||
			     (param->max < md_opt->sleep_mode))) {
				dpm_trace(DPM_TRACE_CONSTRAINT_ASSERTED,
					  param->id, md_opt->sleep_mode,
					  param->min, param->max);
				return 0;
			}
			break;
		} /* end case switch */
	} /* end for loop */

	/* if we got this far, the opt is valid */
	return 1;
}

int dpm_bulverde_validate_opt(struct dpm_opt *opt)
{
	struct list_head *head;
	int force, valid = 1;
	struct device *dev = 0;

	force = dpm_get_force();

	list_for_each_prev(head, &global_device_list) {

		dev = to_dev(head);

		/* If this device has no constraints, go to the next */
		if (!dev->constraints) {
			continue;
		}
		
		if (!valid_opt(dev->constraints, opt)) {
			/* If constraints fail AND force mode is on
			   AND there is a suspend function, call
			   it. 

			   Note: call the suspend function directly
			   (not driver_powerdown) as this will leave
			   constraints asserted which is important to
			   determine if it is OK to resume this device
			   later.
			*/
			if (force && dev->driver->suspend) {
				
				/* Mark this device as SUSPEND_FOR_OP
				   which means it was forced to
				   suspend; it may be turned back on
				   when conditions are appropriate */
				dev->power_state = DPM_SUSPEND_FOR_OP;
				dev->driver->suspend(dev, 0, SUSPEND_POWER_DOWN);
			}
			else {
				/* either force mode is not on or this
				   driver has no suspend function */
				valid = 0;
			}
		}

		/* This is a special check to make sure that
		   constraints are still asserted for this device
		   (implying that a successful return from valid_opt
		   really means there is no constraints conflict and
		   the device may be re-enabled. 

		   If the constraints exist, but have been deasserted,
		   then DPM can never check the constraints for
		   validity and this suspended device will never be
		   re-enabled. And that is not good. */
		else if ((dev->power_state == DPM_SUSPEND_FOR_OP) &&
			 (!dev->constraints->asserted)) {
			printk(KERN_WARNING
				"DPM: Constraints deasserted for %s, "
			       "unable to automatically resume\n", dev->name);
		}

		/* Else constraints ARE valid and asserted, so check
		   if the device may be re-enabled and if so, mark it
		   for resuming (this will happen in the scale
		   function after the change to the new operating
		   point) */
		else if (dev->power_state == DPM_SUSPEND_FOR_OP) {
			dev->power_state = DPM_RESUME_FOR_OP;
		}
	}

	return valid;
}

/* This function will be called after a frequency change to a) let
   drivers adjust any clocks or calculations for the new frequency; b)
   re-enable devices that were forced to suspend for a previous
   operating point. */
static void dpm_bulverde_scale(void)
{
	struct list_head *head;
	struct device *dev = 0;

	list_for_each_prev(head, &global_device_list) {

		dev = to_dev(head);

		/* If this device has a driver and that driver has
		   a scale callback */
		if (dev->driver && dev->driver->scale) {
			/* Call it */
			dev->driver->scale(0, 0);
		}

		if ((dev->power_state == DPM_RESUME_FOR_OP) && 
		    dev->driver->resume) {
			dev->power_state = DPM_POWER_ON;
			dev->driver->resume(dev, DPM_POWER_ON);
		}
	}
}

/****************************************************************************
 *  DPM Idle Handler
 ****************************************************************************/

/* Check for pending external interrupts.  If so, the entry to a low-power
   idle is preempted. */

int return_from_idle_immediate(void)
{
	return 0;
}

/****************************************************************************
 * Machine-dependent /proc/driver/dpm/md entries
 ****************************************************************************/

static inline int 
p5d(char *buf, unsigned mhz)
{
	return sprintf(buf, "%5d", mhz ); /* Round */
}

static int
dpm_proc_print_opt(char *buf, struct dpm_opt *opt)
{
        int len = 0;
        struct dpm_md_opt *md_opt = &opt->md_opt;

        len += sprintf(buf + len, "%12s ", 
                       opt->name);
        len += p5d(buf + len, md_opt->v);
        len += p5d(buf + len, (md_opt->cpu)/1000);
        len += p5d(buf + len, md_opt->l);
        len += p5d(buf + len, md_opt->n);
        len += p5d(buf + len, md_opt->cccra);
        len += p5d(buf + len, md_opt->b);
        len += p5d(buf + len, md_opt->half_turbo);
        len += p5d(buf + len, md_opt->cpll_enabled);
        len += p5d(buf + len, md_opt->ppll_enabled);
        len += p5d(buf + len, md_opt->sleep_mode);
        len += p5d(buf + len, (md_opt->lcd)/1000);
	len += sprintf(buf + len, "\n");
        return len;
}

int
read_proc_dpm_md_opts(char *page, char **start, off_t offset,
		      int count, int *eof, void *data)
{
	int len = 0;
	int limit = offset + count;
	struct dpm_opt *opt;
	struct list_head *opt_list;
	
	/* FIXME: For now we assume that the complete table,
	 * formatted, fits within one page */
	if (offset >= PAGE_SIZE)
		return 0;

	if (dpm_lock_interruptible())
		return -ERESTARTSYS;

	if (!dpm_initialized)
		len += sprintf(page + len, "DPM is not initialized\n");
	else if (!dpm_enabled)
		len += sprintf(page + len, "DPM is disabled\n");
	else {
		len += sprintf(page + len,
			       "The active DPM policy is \"%s\"\n",
			       dpm_active_policy->name);
		len += sprintf(page + len, 
			       "The current operating point is \"%s\"\n",
			       dpm_active_opt->name);
	}

	if (dpm_initialized) {
		len += sprintf(page + len, 
			       "Table of all defined operating points, "
			       "frequencies in MHz:\n");

		len += sprintf(page + len, 
				"        Name  Vol   CPU    L    N    A    B   HT  PLL Sleep LCD\n");

		list_for_each(opt_list, &dpm_opts) {
			opt = list_entry(opt_list, struct dpm_opt, list);
			if (len >= PAGE_SIZE)
				BUG();
			if (len >= limit)
				break;
			len += dpm_proc_print_opt(page + len, opt);
		}
	}
	dpm_unlock();
	*eof = 1;
	if (offset >= len)
		return 0;
	*start = page + offset;
	return min(count, len - (int)offset);
}

/*
 *
 * /proc/driver/dpm/md/cmd (Write-only)
 *
 *  This is a catch-all, simple command processor for the Bulverde DPM
 *  implementation. These commands are for experimentation and development
 *  _only_, and may leave the system in an unstable state.
 *
 *  No commands defined now.
 *
 */

int 
write_proc_dpm_md_cmd (struct file *file, const char *buffer,
		       unsigned long count, void *data)
{
	char *buf, *tok, *s;
	char *whitespace = " \t\r\n";
	int ret = 0;

	if (current->uid != 0)
		return -EACCES;
	if (count == 0)
		return 0;
	if (!(buf = kmalloc(count + 1, GFP_KERNEL)))
		return -ENOMEM;
	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}
	buf[count] = '\0';
	s = buf + strspn(buf, whitespace);
	tok = strsep(&s, whitespace);
	
	if (strcmp(tok, "define-me") == 0) {
		;
	} else {
		ret = -EINVAL;
	}
		kfree(buf);
	if (ret == 0)
		return count;
	else 
		return ret;
}

/****************************************************************************
 * Initialization/Exit
 ****************************************************************************/

void
dpm_bulverde_cleanup(void)
{
	dpm_bd.exit();
	bulverde_freq_cleanup();
}

int __init
dpm_bulverde_init(void)
{
	printk("Mainstone bulverde Dynamic Power Management\n");
	
	dpm_md.init		= NULL;
	dpm_md.init_opt		= dpm_bulverde_init_opt;
	dpm_md.set_opt		= dpm_bulverde_set_opt;
	dpm_md.get_opt		= dpm_bulverde_get_opt;
	dpm_md.validate_opt	= dpm_bulverde_validate_opt;
	dpm_md.idle_set_parms	= NULL;
	dpm_md.cleanup		= dpm_bulverde_cleanup;

	dpm_bulverde_board_setup();
	dpm_bd.init();

	bulverde_clk_init();
	bulverde_vcs_init();

	return 0;
}
__initcall(dpm_bulverde_init);
