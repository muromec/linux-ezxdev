/*
 * arch/arm/mach-mx2ads/mx21-dpm.c
 *
 * MX21-specific DPM support
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
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/device.h>

#include <linux/delay.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/arch/hardware.h>
#include <asm/arch/irqs.h>
#include <asm/arch/pll.h>

/* For dpm_mx21_scale and dpm_mx21_validate_opt to go through
   the LDM device list */
extern struct list_head global_device_list;
#define to_dev(node) container_of(node,struct device,g_list)

/* Array of strings of Operating Point Parameters */
char *dpm_opp_names[DPM_MD_PARAM_NUM] = DPM_OPP_NAMES;

static void
dpm_mx21_frequency_scale(struct dpm_regs *regs)
{
	struct list_head *head;
	struct device *dev = 0;
	char do_scale = 0;
	char presc_falling = 0;
	u32 flags;

	save_flags(flags);

	if ((u32) regs->presc != -1) {
		if ((u32) regs->presc <
		    ((CRM_CSCR & CSCR_PRESC) >> CSCR_PRESC_SHIFT)) {
			/*try to avoid potentially invalid (both freqs too high) combinations,
			   by checking whether this frequcy is rising. If yes, raise it afterwards */
			presc_falling = 1;

		} else {
			cli();

			CRM_CSCR =
			    (CRM_CSCR & ~CSCR_PRESC) | ((u32) regs->
							presc <<
							CSCR_PRESC_SHIFT);
			do_scale = 1;
		}
	}

	if ((u32) regs->bclkdiv != -1) {
		cli();

		CRM_CSCR =
		    (CRM_CSCR & ~CSCR_BCLKDIV) | ((u32) regs->
						  bclkdiv <<
						  CSCR_BCLKDIV_SHIFT);
		do_scale = 1;
	}

	if (((u32) regs->presc != -1) && (presc_falling)) {
		cli();

		CRM_CSCR =
		    (CRM_CSCR & ~CSCR_PRESC) | ((u32) regs->
						presc << CSCR_PRESC_SHIFT);
		do_scale = 1;
	}

	if (!do_scale) {
		restore_flags(flags);
		return;
	}

/* frequency change done.
   let drivers adjust any clocks or calculations for the new frequency and
   re-enable devices that were forced to suspend for a previous
   operating point. */

	list_for_each_prev(head, &global_device_list) {

		dev = to_dev(head);

		/* If this device has a driver and that driver has
		   a scale callback */
		if (dev->driver && dev->driver->scale) {
			/* Call it */
			dev->driver->scale(0, 0);
		}
	}

	restore_flags(flags);

	list_for_each_prev(head, &global_device_list) {

		dev = to_dev(head);

		if ((dev->power_state == DPM_RESUME_FOR_OP) &&
		    dev->driver->resume) {
			dev->power_state = DPM_POWER_ON;
			dev->driver->resume(dev, DPM_POWER_ON);
		}
	}
}

static inline void
dpm_mx21_sleep(void)
{
	/* Stop PLLs */
	CRM_CSCR &= ~CSCR_MPEN;

	/*enter wait-for-interrupt mode */
	cpu_do_idle();
}

static void
dpm_mx21_fscaler(struct dpm_regs *regs)
{

	if (regs->cpu == 1) {

		/* Power down all devices */
		device_suspend(0, SUSPEND_POWER_DOWN);

		/*sleep mode */
		dpm_mx21_sleep();

		/****we are awake again here!*****/

		dpm_mx21_frequency_scale(regs);

		/* Power on devices again */
		device_resume(RESUME_POWER_ON);

		/* Recursive call to switch back to to task state. */
		dpm_set_os(DPM_TASK_STATE);
	} else {
		dpm_mx21_frequency_scale(regs);
	}

}

static dpm_fscaler fscalers[1] = {
	dpm_mx21_fscaler,
};

/* This routine computes the "forward" frequency scaler that moves the system
 * from the current operating point to the new operating point.  The resulting
 * fscaler is applied to the registers of the new operating point.
 */
dpm_fscaler
compute_fscaler(struct dpm_md_opt *cur, struct dpm_md_opt *new)
{
/*determine what sort of action is needed,
configure and return pointer to the appropriate
scaling function accordingly*/

	u32 tmp_mpll, tmp_div;
	tmp_mpll = mx_module_get_clk(MPLL) / 1000;

	/*fscaling action will only see ->regs values */

	if ((new->fclk != -1) && (new->hclk != -1)) {
		/*set both fclk and hclk */
		tmp_div = tmp_mpll / new->fclk;
		new->regs.presc = tmp_div - 1;

		tmp_div = new->fclk / new->hclk;
		new->regs.bclkdiv = tmp_div - 1;
	} else {
		new->regs.bclkdiv = -1;
		new->regs.presc = -1;
	}

	new->regs.cpu = new->cpu;

	return fscalers[0];
}

static int
dpm_mx21_init_opt(struct dpm_opt *opt)
{
	int cpu = opt->pp[DPM_MD_CPU];
	int tmp_fclk = opt->pp[DPM_MD_FCLK];
	int tmp_hclk = opt->pp[DPM_MD_HCLK];
	int tmp_mpll, min_fclk, tmp_clk, min_hclk, tmp_div;
	struct dpm_md_opt *md_opt = &opt->md_opt;

	/* Let's do some upfront error checking.  If we fail any of these, then the
	 * whole operating point is suspect and therefore invalid.
	 */

	if ((cpu != 0) && (cpu != 1) && (cpu != -1)) {
		printk(KERN_ERR "MX2 DPM Error: "
		       "CPU state setting %u is out of range for opt named %s. "
		       "Possible settings: 1 - sleep, 0 or -1 - keep running\n",
		       cpu, opt->name);
		return -EINVAL;
	}

	tmp_mpll = mx_module_get_clk(MPLL) / 1000;
	min_fclk = tmp_mpll / (((u32) CSCR_PRESC >> CSCR_PRESC_SHIFT) + 1);

	if ((!((tmp_fclk >= min_fclk) && (tmp_fclk <= tmp_mpll))) &&
	    (tmp_fclk != -1)) {
		printk(KERN_ERR "MX2 DPM Error: "
		       "FCLK setting %d is out of range for opt named %s. "
		       "Possible settings: %d .. %d(MPLL). "
		       "Set -1 to FCLK and HCLK to keep current frequency settings\n",
		       tmp_fclk, opt->name, min_fclk, tmp_mpll);
		return -EINVAL;
	}

	if (tmp_fclk != -1) {

		tmp_div = tmp_mpll / tmp_fclk;
		tmp_clk = tmp_mpll / tmp_div;

		if (tmp_clk != tmp_fclk) {
			printk(KERN_WARNING "MX2 DPM Warning: "
			       "adjusting FCLK setting from %d to %d to match PRESC division result "
			       "for opt named %s\n", tmp_fclk, tmp_clk,
			       opt->name);
			tmp_fclk = tmp_clk;
		}

		min_hclk =
		    tmp_fclk / (((u32) CSCR_BCLKDIV >> CSCR_BCLKDIV_SHIFT) + 1);

		if ((!((tmp_hclk >= min_hclk) && (tmp_hclk <= tmp_fclk)))
		    && (tmp_hclk != -1)) {
			printk(KERN_ERR "MX2 DPM Error: "
			       "HCLK setting %d is out of range for opt named %s. "
			       "Possible settings: %d .. %d(FCLK). "
			       "Set -1 to FCLK and HCLK to keep current fequency settings\n",
			       tmp_hclk, opt->name, min_hclk, tmp_fclk);
			return -EINVAL;
		}

		if (tmp_hclk == -1) {
			printk(KERN_ERR "MX2 DPM Error: "
			       "Cannot set FCLK if HCLK setting is -1 (not defined) "
			       "for opt named %s\n", opt->name);
			return -EINVAL;
		}

		tmp_div = tmp_fclk / tmp_hclk;
		tmp_clk = tmp_fclk / tmp_div;

		if (tmp_clk != tmp_hclk) {
			printk(KERN_WARNING "MX2 DPM Warning: "
			       "adjusting HCLK setting from %d to %d to match BCLKDIV division result "
			       "for opt named %s\n", tmp_hclk, tmp_clk,
			       opt->name);
			tmp_hclk = tmp_clk;
		}

	} else {
		if (tmp_hclk != -1) {
			printk(KERN_ERR "MX2 DPM Error: "
			       "Cannot set HCLK if FCLK setting is -1 (not defined) "
			       "for opt named %s\n", opt->name);
			return -EINVAL;
		}
		/*no HCLK or FCLK defined - that's OK */
	}

	md_opt->cpu = cpu;
	md_opt->fclk = tmp_fclk;
	md_opt->hclk = tmp_hclk;

	return 0;
}

static int
dpm_mx21_get_opt(struct dpm_opt *opt)
{
	struct dpm_md_opt *md_opt = &opt->md_opt;

/* Fully determine the current machine-dependent operating point, and fill in a
   structure presented by the caller. This should only be called when the
   dpm_sem is held. This call can return an error if the system is currently at
   an operating point that could not be constructed by dpm_md_init_opt(). */

	md_opt->cpu = 0;	/*cpu is running by default */
	md_opt->fclk = mx_module_get_clk(FCLK) / 1000;
	md_opt->hclk = mx_module_get_clk(HCLK) / 1000;

	return 0;
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

	for (i = 0; (i < constraints->count); i++) {
		param = &constraints->param[i];

		switch (param->id) {
		case DPM_MD_FCLK:
			if ((md_opt->fclk != -1) &&
			    ((param->min > md_opt->fclk) ||
			     (param->max < md_opt->fclk))) {
				dpm_trace(DPM_TRACE_CONSTRAINT_ASSERTED,
					  param->id, md_opt->fclk,
					  param->min, param->max);
				return 0;
			}
			break;
		case DPM_MD_HCLK:
			if ((md_opt->hclk != -1) &&
			    ((param->min > md_opt->hclk) ||
			     (param->max < md_opt->hclk))) {
				dpm_trace(DPM_TRACE_CONSTRAINT_ASSERTED,
					  param->id, md_opt->hclk,
					  param->min, param->max);
				return 0;
			}
			break;
		}		/* end case switch */
	}			/* end for loop */

	/* if we got this far, the opt is valid */
	return 1;
}

int
dpm_mx21_validate_opt(struct dpm_opt *opt)
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
				dev->driver->suspend(dev, 0,
						     SUSPEND_POWER_DOWN);
			} else {
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
			printk("MX2 DPM: Constraints deasserted for %s, "
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

/****************************************************************************
 *  DPM Idle Handler
 ****************************************************************************/

/* Check for pending external interrupts.  If so, the entry to a low-power
   idle is preempted. */

int
return_from_idle_immediate(void)
{

	return 0;
}

/****************************************************************************
 * Machine-dependent /proc/driver/dpm/md entries
 ****************************************************************************/

static int
dpm_proc_print_opt(char *buf, struct dpm_opt *opt)
{
	struct dpm_md_opt *md_opt = &opt->md_opt;

	return sprintf(buf, "%12s\t%d\t%d\t%d\n", opt->name, md_opt->cpu,
		       md_opt->fclk, md_opt->hclk);
}

int
read_proc_dpm_md_opts(char *page, char **start, off_t offset,
		      int count, int *eof, void *data)
{

	int len = 0;
	int limit = offset + count;
	struct dpm_opt *opt;
	struct list_head *opt_list;

	/* ATTENTION: Assumption is that the entire table,
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
			       "frequencies in KHz:\n");

		len += sprintf(page + len, " Name\t\tCPU\tFCLK\tHCLK\n");

		list_for_each(opt_list, &dpm_opts) {
			opt = list_entry(opt_list, struct dpm_opt, list);
			if (len >= PAGE_SIZE)
				BUG();
			if (len >= limit)
				break;
			len += dpm_proc_print_opt(page + len, opt);
		}
		len +=
		    sprintf(page + len,
			    "\nCurrent values  MPLL\tSPLL\tFCLK\tHCLK\n");

		len +=
		    sprintf(page + len,
			    "                %d\t%d\t%d\t%d\n",
			    mx_module_get_clk(MPLL) / 1000,
			    mx_module_get_clk(SPLL) / 1000,
			    mx_module_get_clk(FCLK) / 1000,
			    mx_module_get_clk(HCLK) / 1000);

	}
	dpm_unlock();
	*eof = 1;
	if (offset >= len)
		return 0;
	*start = page + offset;
	return min(count, len - (int) offset);

}

/*
 * This is a dummy command input processor for
 * /proc/driver/dpm/md/cmd
 * No commands defined. For developer's use only.
 */

int
write_proc_dpm_md_cmd(struct file *file, const char *buffer,
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
	} else
		ret = -EINVAL;

	kfree(buf);
	if (ret == 0)
		return count;
	else

		return ret;
}

/****************************************************************************
 * Initialization/Exit
 ****************************************************************************/
static int
dpm_mx21_bd_init(void)
{
	return 0;
}

static void
dpm_mx21_bd_exit(void)
{

}

static void
dpm_mx21_cleanup(void)
{

}

struct dpm_bd dpm_bd = {
	.init = dpm_mx21_bd_init,
	.exit = dpm_mx21_bd_exit,
	.check_v = NULL,
	.set_v_pre = NULL,
	.set_v_post = NULL,
};

struct dpm_md dpm_md = {
	.init = NULL,
	.init_opt = dpm_mx21_init_opt,
	.set_opt = dpm_default_set_opt,
	.get_opt = dpm_mx21_get_opt,
	.validate_opt = dpm_mx21_validate_opt,
	.idle_set_parms = NULL,
	.cleanup = dpm_mx21_cleanup,
};

int __init
dpm_mx21_init(void)
{
	printk(KERN_INFO "MX21 Dynamic Power Management\n");
	return 0;
}

__initcall(dpm_mx21_init);
