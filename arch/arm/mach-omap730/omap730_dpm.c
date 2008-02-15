/*
 * arch/arm/mach-omap730/omap730-dpm.c  OMAP730 specific DPM support
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
 * Copyright (C) 2002 MontaVista Software <source@mvista.com>.
 * 	Matthew Locke <mlocke@mvista.com>
 *
 * Modifications for the OMAP730 :
 * Copyright (c) 2004 MPC-Data Limited (http://www.mpc-data.co.uk)
 * Dave Peverley <dpeverley@mpc-data.co.uk>
 *
 * Based on arch/ppc/platforms/ibm405lp_dpm.c by Bishop Brock.
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

#include <linux/delay.h>
#include <asm/hardirq.h>
#include <asm/irq.h>
#include <asm/page.h>
#include <asm/processor.h>
#include <asm/uaccess.h>
#include <asm/arch/hardware.h>
#include <asm/arch/ck.h>
#include <linux/device.h>
#include <asm/arch/pm.h>

#undef DEBUG

#ifdef DEBUG
#define DPRINT(args...) printk(args)
#else
#define DPRINT(args...) do {} while (0)
#endif

struct dpm_bd dpm_bd;
struct dpm_md dpm_md;


/* 
 * This prototype is needed as we sneakily borrow the
 * read_ih() call that used to be static within irq.c!
 */
inline u32 read_ih(int level, int part, int reg);



static void omap730_scale_divisors(struct dpm_regs *regs)
{
	volatile u16 *arm_ctl;
	volatile u16 new_arm_ctl;

	arm_ctl = (volatile u16 *) ARM_CKCTL;

	new_arm_ctl = (*arm_ctl & ~(regs->arm_msk)) | regs->arm_ctl;
	*arm_ctl = new_arm_ctl;
}

static void omap730_scale_dpll(struct dpm_regs *regs)
{
	volatile u16 *pll;

	pll = (volatile u16 *) DPLL1_CTL_REG;

	*pll &= ~(CK_DPLL_MASK);
       *pll |= regs->dpll_ctl;
	{ int i; for(i=0;i<0xFF;i++) asm volatile ("nop"); }  
}

unsigned int dpm_fscaler_flags;
#define DPM_FSCALER_NOP 		0
#define DPM_FSCALER_DIVISORS	 	1
#define DPM_FSCALER_SLEEP 		2
#define DPM_FSCALER_WAKEUP 		4
#define DPM_FSCALER_DPLL	 	8
#define DPM_FSCALER_UP		 	16

static void omap730_fscaler(struct dpm_regs *regs)
{
	extern void omap_pm_suspend(void);

	if(dpm_fscaler_flags & DPM_FSCALER_NOP)
		return;

	/* We want to scale after devices are powered off
	 * if going to sleep
	 */
	if(dpm_fscaler_flags & DPM_FSCALER_SLEEP)
		device_suspend(0, SUSPEND_POWER_DOWN);

	if(!(dpm_fscaler_flags & DPM_FSCALER_UP) && dpm_fscaler_flags & DPM_FSCALER_DPLL)
		omap730_scale_dpll(regs);

	if(dpm_fscaler_flags & DPM_FSCALER_DIVISORS)
		omap730_scale_divisors(regs);

	if(dpm_fscaler_flags & DPM_FSCALER_UP && dpm_fscaler_flags & DPM_FSCALER_DPLL)
		omap730_scale_dpll(regs);
	/* Don't bring the devices back until frequencies restored */
	if(dpm_fscaler_flags & DPM_FSCALER_WAKEUP)
		device_resume(RESUME_POWER_ON);

	if(dpm_fscaler_flags & DPM_FSCALER_SLEEP) {

		omap_pm_suspend();

	 	/* Here when we wake up.  Recursive call to switch back to
	 	* to task state.
	 	*/

		dpm_set_os(DPM_TASK_STATE);
	}
}

static dpm_fscaler fscalers[1] = { 
	omap730_fscaler,
};

/* This routine computes the "forward" frequency scaler that moves the system
 * from the current operating point to the new operating point.  The resulting
 * fscaler is applied to the registers of the new operating point. 
 */

dpm_fscaler
compute_fscaler(struct dpm_md_opt *cur, struct dpm_md_opt *new)
{
	dpm_fscaler_flags = DPM_FSCALER_NOP;

	if (cur->cpu && ! new->cpu)
		dpm_fscaler_flags = DPM_FSCALER_SLEEP;
	else if (! cur->cpu && new->cpu)
		dpm_fscaler_flags = DPM_FSCALER_WAKEUP;

	if (new->regs.dpll_ctl != (cur->regs.dpll_ctl & CK_DPLL_MASK)) {
		dpm_fscaler_flags |= DPM_FSCALER_DPLL;
		loops_per_jiffy = new->lpj;
	}

	if (new->regs.arm_ctl != (cur->regs.arm_ctl & new->regs.arm_msk)) {
		dpm_fscaler_flags |= DPM_FSCALER_DIVISORS;
		loops_per_jiffy = new->lpj;
	}

	if (new->regs.dpll_ctl > (cur->regs.dpll_ctl & CK_DPLL_MASK) && dpm_fscaler_flags & DPM_FSCALER_DIVISORS)
		dpm_fscaler_flags |= DPM_FSCALER_UP;

	return fscalers[0];
}



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

/* Initialize the machine-dependent operating point from a list of parameters,
   which has already been installed in the pp field of the operating point.
   Some of the parameters may be specified with a value of -1 to indicate a
   default value. */

#define DPLL_DIV_MAX 4
#define DPLL_MULT_MAX 31

int divider_encode(int n) 
{
	int count = 0;
        if (n == 0 || n == -1)
		return n;

	if (n > 8) 	/* no divider greater than 8 is valid */
		n = 8;
	while(n) {
		n = n >> 1;
		count++;
	}
	return (count -1);
}

int
dpm_omap730_init_opt(struct dpm_opt *opt)
{
	static int divider_decode[] = {1,2,4,8};
	int v		= opt->pp[DPM_MD_V];
	int pll_mult	= opt->pp[DPM_MD_DPLL_MULT];
	int pll_div	= opt->pp[DPM_MD_DPLL_DIV];
	int arm_div	= opt->pp[DPM_MD_ARM_DIV];
	int tc_div	= opt->pp[DPM_MD_TC_DIV];
	int per_div	= opt->pp[DPM_MD_PER_DIV];
	int dsp_div	= opt->pp[DPM_MD_DSP_DIV];
	int dspmmu_div	= opt->pp[DPM_MD_DSPMMU_DIV];
	int lcd_div	= opt->pp[DPM_MD_LCD_DIV];

	struct dpm_md_opt *md_opt = &opt->md_opt;

	/* Let's do some upfront error checking.  If we fail any of these, then the
	 * whole operating point is suspect and therefore invalid.
	 */

	if ((pll_mult > DPLL_MULT_MAX) || (pll_mult < 0) ||
	    (pll_div > DPLL_DIV_MAX) || (pll_div < 0)) {

		printk(KERN_WARNING "dpm_md_init_opt: "
				 "PLL_MUL/PLL_DIV specification "
				 "(%u/%u) out of range for opt named %s\n",
				 pll_mult, pll_div, opt->name );
		return -EINVAL;
	}

/* make sure the operating point follows the syncronous scalable mode rules */

	if ((arm_div) && (arm_div > tc_div)) {
		printk(KERN_WARNING "dpm_md_init_opt: "
			 "CPU frequency can not be lower than TC frequency"
			 "CPU div %u and/or TC div %u out of range"
			 " for opt named %s\n",
			 arm_div, tc_div, opt->name );
		return -EINVAL;
	}

	if ((dspmmu_div) && (dspmmu_div > tc_div)) {
		printk(KERN_WARNING "dpm_md_init_opt: "
			 "DSP MMU frequency can not be lower than TC frequency"
			 "DSP MMU div %u and/or TC div %u out of range"
			 " for opt named %s\n",
			 dspmmu_div, tc_div, opt->name );
		return -EINVAL;
	}

	if ((dsp_div && dspmmu_div) && (dspmmu_div > dsp_div)) {
		printk(KERN_WARNING "dpm_md_init_opt: "
			 "DSP MMU frequency can not be lower than DSP frequency"
			 "DSP MMU div %u and/or DSP div %u out of range"
			 " for opt named %s\n",
			 dspmmu_div, dsp_div, opt->name );
		return -EINVAL;
	}

	/* 0 is a special case placeholder for dpll idle */
	if (pll_mult == 0 || pll_div == 0) {
		md_opt->regs.dpll_ctl = 0xffff;
		md_opt->dpll = ck_get_rate(ck_gen1);
	}
	else {
		md_opt->dpll = (CK_CLKIN * pll_mult)/pll_div;
		if (!ck_valid_rate(md_opt->dpll)) {
			printk(KERN_WARNING "dpm_md_init_opt: DPLL rate is invalid: "
				 "%u out of range for opt named \"%s\"\n",
				 md_opt->dpll, opt->name );
			return -EINVAL;
		}
		else
			md_opt->regs.dpll_ctl = (((pll_mult << 2) | 
						(pll_div -1)) << 5);
	}

 	/* the encode/decode will weed out invalid dividers */
	if (arm_div == 0) 		/* zero means sleep for cpu */
		md_opt->cpu = 0;	
	else if (arm_div > 0) {
		arm_div	= divider_encode(arm_div);
		md_opt->cpu = md_opt->dpll / divider_decode[arm_div];
		md_opt->regs.arm_ctl |= (arm_div << ARMDIV);
		md_opt->regs.arm_msk |= (3 << ARMDIV);
	}

	/*  Ignore 0 and -1 */
	if (tc_div > 0) {
		tc_div	= divider_encode(tc_div);
		md_opt->tc = md_opt->dpll / divider_decode[tc_div];
		md_opt->regs.arm_ctl |= (tc_div << TCDIV);
		md_opt->regs.arm_msk |= (3 << TCDIV);
	}

	if (per_div > 0) {
		per_div	= divider_encode(per_div);
		md_opt->per = md_opt->dpll / divider_decode[per_div];
		md_opt->regs.arm_ctl |= (per_div << PERDIV);
		md_opt->regs.arm_msk |= (3 << PERDIV);
	}

	if (dsp_div > 0) {
		dsp_div	= divider_encode(dsp_div);
		md_opt->dsp = md_opt->dpll / divider_decode[dsp_div];
		md_opt->regs.arm_ctl |= (dsp_div << DSPDIV);
		md_opt->regs.arm_msk |= (3 << DSPDIV);
	}

	if (dspmmu_div > 0) {
		dspmmu_div= divider_encode(dspmmu_div);
		md_opt->dspmmu = md_opt->dpll / divider_decode[dspmmu_div];
		md_opt->regs.arm_ctl |= (dspmmu_div << DSPMMUDIV);
		md_opt->regs.arm_msk |= (3 << DSPMMUDIV);
	}

	if (lcd_div > 0) {
		lcd_div	= divider_encode(lcd_div);
		md_opt->lcd = md_opt->dpll / divider_decode[lcd_div];
		md_opt->regs.arm_ctl |= (lcd_div << LCDDIV);
		md_opt->regs.arm_msk |= (3 << LCDDIV);
	}

	md_opt->v = v;

	md_opt->lpj = compute_lpj(loops_per_jiffy, ck_get_rate(arm_ck)*1000, md_opt->cpu*1000);

	return 0;
}

/* Fully determine the current machine-dependent operating point, and fill in a
   structure presented by the caller. This should only be called when the
   dpm_sem is held. This call can return an error if the system is currently at
   an operating point that could not be constructed by dpm_md_init_opt(). */

int
dpm_omap730_get_opt(struct dpm_opt *opt)
{
	struct dpm_md_opt *md_opt = &opt->md_opt;

	md_opt->v = 5000; /* hardwired for now. */
	md_opt->dpll = ck_get_rate(ck_gen1);
	md_opt->cpu = ck_get_rate(arm_ck);
	md_opt->tc = ck_get_rate(tc_ck);
	md_opt->per = ck_get_rate(mpuper_ck);
	md_opt->dsp = ck_get_rate(dsp_ck);
	md_opt->dspmmu = ck_get_rate(dspmmu_ck);
	md_opt->lcd = ck_get_rate(lcd_ck);
	md_opt->lpj = loops_per_jiffy;
	md_opt->regs.dpll_ctl = *((volatile u16 *) DPLL1_CTL_REG);
	md_opt->regs.arm_ctl = *((volatile u16 *) ARM_CKCTL);

	return 0;
}

/****************************************************************************
 *  DPM Idle Handler
 ****************************************************************************/

/* Check for pending external interrupts.  If so, the entry to a low-power
   idle is preempted. */

int
return_from_idle_immediate(void)
{
	if ((read_ih(0, 0, IRQ_ITR) & ~read_ih(0, 0, IRQ_MIR)) ||
	    (read_ih(0, 1, IRQ_ITR) & ~read_ih(0, 1, IRQ_MIR)) ||
	    (read_ih(1, 0, IRQ_ITR) & ~read_ih(1, 0, IRQ_MIR)) ||
	    (read_ih(1, 1, IRQ_ITR) & ~read_ih(1, 1, IRQ_MIR)))
		return 1;
	else
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

        len += sprintf(buf + len, "%12s %9llu", 
                       opt->name, opt->stats.count);
        len += p5d(buf + len, md_opt->dpll);
        len += p5d(buf + len, md_opt->cpu);
        len += p5d(buf + len, md_opt->tc);
        len += p5d(buf + len, md_opt->per);
        len += p5d(buf + len, md_opt->dsp);
        len += p5d(buf + len, md_opt->dspmmu);
        len += p5d(buf + len, md_opt->lcd);
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
#if 0 // TODO
	len += sprintf(page + len, "The system clock speed is");
	len += p61f(page + len, bip->sys_speed / 1000);
	len += sprintf(page + len, " MHz\n\n");
#endif

	if (dpm_initialized) {
		len += sprintf(page + len, 
			       "Table of all defined operating points, "
			       "frequencies in MHz:\n");

		len += sprintf(page + len, 
				" Name           Count  DPLL  CPU  TC  PER  DSP  DSPMMU   LCD\n");

		list_for_each(opt_list, &dpm_opts) {
			opt = list_entry(opt_list, struct dpm_opt, list);
			if (len >= PAGE_SIZE)
				BUG();
			if (len >= limit)
				break;
			len += dpm_proc_print_opt(page + len, opt);
		}
		len += sprintf(page +len,"\nCurrent values DPLL\tCPU\tTC\tPER\tDSP\tDSPMMU\tLCD\n");
		len += sprintf(page +len,"\n                %d\t%d\t%d\t%d\t%d\t%d\t%d\n",
				ck_get_rate(ck_gen1), ck_get_rate(arm_ck),
				ck_get_rate(tc_ck), ck_get_rate(mpuper_ck),
				ck_get_rate(dsp_ck), ck_get_rate(dspmmu_ck),
				ck_get_rate(lcd_ck));
		//calibrate_delay();
	}
	dpm_unlock();
	*eof = 1;
	if (offset >= len)
		return 0;
	*start = page + offset;
	return min(count, len - (int)offset);
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * /proc/driver/dpm/md/cmd (Write-only)
 *
 *  This is a catch-all, simple command processor for the OMAP730/Perseus2 DPM
 *  implementation. These commands are for experimentation and development
 *  _only_, and may leave the system in an unstable state.
 *
 *  No commands defined now.
 *
 ****************************************************************************/

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

#ifdef CONFIG_DPM_IDLE
/****************************************************************************
 * DPM Idle
 ****************************************************************************/
u32 basic_idle(struct dpm_idle_parms *idle_parms)
{
	omap730_pm_idle();
	return 0;
}
#endif	/* ifdef CONFIG_DPM_IDLE */

/****************************************************************************
 * Initialization/Exit
 ****************************************************************************/

void
dpm_omap_cleanup(void)
{
	dpm_bd.exit();
}

extern void (*pm_idle)(void);   

int __init
dpm_omap_init(void)
{
	printk("OMAP730/Perseus2 Dynamic Power Management\n");
	
	dpm_md.init		= NULL;
	dpm_md.init_opt		= dpm_omap730_init_opt;
	dpm_md.set_opt		= dpm_default_set_opt;
	dpm_md.get_opt		= dpm_omap730_get_opt;
	dpm_md.idle_set_parms	= NULL;
	dpm_md.cleanup		= dpm_omap_cleanup;

	dpm_omap730_board_setup();
	dpm_bd.init();
#ifdef CONFIG_DPM_UTIMER
	init_utimer(&set_opt_utimer);
#endif

	return 0;
}
__initcall(dpm_omap_init);

/*
 * Local variables:
 * c-basic-offset: 8
 * End:
 */

