/*
 * SA1100 Power Management Routines
 *
 * Copyright (c) 2001 Cliff Brake <cbrake@accelent.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 *
 * History:
 *
 * 2001-02-06:	Cliff Brake         Initial code
 *
 * 2001-02-25:	Sukjae Cho <sjcho@east.isi.edu> &
 * 		Chester Kuo <chester@linux.org.tw>
 * 			Save more value for the resume function! Support
 * 			Bitsy/Assabet/Freebird board
 *
 * 2001-08-29:	Nicolas Pitre <nico@cam.org>
 * 			Cleaned up, pushed platform dependent stuff
 * 			in the platform specific files.
 *
 * 2002-05-27:	Nicolas Pitre	Killed sleep.h and the kmalloced save array.
 * 				Storage is local on the stack now.
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/sysctl.h>
#include <linux/errno.h>
#include <linux/cpufreq.h>

#include <asm/hardware.h>
#include <asm/memory.h>
#include <asm/system.h>
#include <asm/leds.h>


#ifdef CONFIG_SA1100_H3XXX
#include <asm/arch/h3600_asic.h>
#endif

/*
 * Debug macros
 */
#undef DEBUG



static char pm_helper_path[128] = "/sbin/pm_helper";
extern int call_usermodehelper(char *path, char **argv, char **envp);

static void
run_sbin_pm_helper( pm_request_t action )
{
	int i;
	char *argv[3], *envp[8];

	if (!pm_helper_path[0])
		return;

	if ( action != PM_SUSPEND && action != PM_RESUME )
		return;

	i = 0;
	argv[i++] = pm_helper_path;
	argv[i++] = (action == PM_RESUME ? "resume" : "suspend");
	argv[i] = 0;

        printk(KERN_CRIT __FUNCTION__ ":%d pm_helper_path=%s\n", __LINE__, pm_helper_path);
	i = 0;
	/* minimal command environment */
	envp[i++] = "HOME=/";
	envp[i++] = "PATH=/sbin:/bin:/usr/sbin:/usr/bin";
	envp[i] = 0;

	/* other stuff we want to pass to /sbin/pm_helper */
	call_usermodehelper (argv [0], argv, envp);
}



extern void sa1100_cpu_suspend(void);
extern void sa1100_cpu_resume(void);

#define SAVE(x)		sleep_save[SLEEP_SAVE_##x] = x
#define RESTORE(x)	x = sleep_save[SLEEP_SAVE_##x]

/*
 * List of global SA11x0 peripheral registers to preserve.
 * More ones like CP and general purpose register values are preserved
 * with the stack location in sleep.S.
 */
enum {	SLEEP_SAVE_START = 0,

	SLEEP_SAVE_OSCR, SLEEP_SAVE_OIER,
	SLEEP_SAVE_OSMR0, SLEEP_SAVE_OSMR1, SLEEP_SAVE_OSMR2, SLEEP_SAVE_OSMR3,

	SLEEP_SAVE_GPDR, SLEEP_SAVE_GRER, SLEEP_SAVE_GFER, SLEEP_SAVE_GAFR,
	SLEEP_SAVE_PPDR, SLEEP_SAVE_PPSR, SLEEP_SAVE_PPAR, SLEEP_SAVE_PSDR,
#if	defined(CONFIG_SA1100_H3XXX)
	SLEEP_SAVE_PWER,
#endif

	SLEEP_SAVE_ICMR,
	SLEEP_SAVE_Ser1SDCR0,

	SLEEP_SAVE_SIZE
};


int pm_do_suspend(void)
{
	unsigned long sleep_save[SLEEP_SAVE_SIZE];

	cli();

	leds_event(led_stop);

	/* preserve current time */
	RCNR = xtime.tv_sec;

	/* save vital registers */
	SAVE(OSCR);
	SAVE(OSMR0);
	SAVE(OSMR1);
	SAVE(OSMR2);
	SAVE(OSMR3);
	SAVE(OIER);

	SAVE(GPDR);
	SAVE(GRER);
	SAVE(GFER);
	SAVE(GAFR);

	SAVE(PPDR);
	SAVE(PPSR);
	SAVE(PPAR);
	SAVE(PSDR);

	SAVE(Ser1SDCR0);

	SAVE(ICMR);
#ifdef CONFIG_SA1100_H3XXX
        SAVE(PWER);
#endif

	/* ... maybe a global variable initialized by arch code to set this? */
	GRER = PWER;
	// Ugly, but I need the AC inserted event
	// In the future, we're going to care about DCD and USB interrupts as well
	if ( machine_is_h3800()) {
#ifdef CONFIG_SA1100_H3XXX
		GFER = GPIO_H3800_AC_IN;
#endif
	} else {
	GFER = 0;
	}
	GEDR = GEDR;

	/* Clear previous reset status */
	RCSR = RCSR_HWR | RCSR_SWR | RCSR_WDR | RCSR_SMR;

	/* set resume return address */
	PSPR = virt_to_phys(sa1100_cpu_resume);

	/* go zzz */
	sa1100_cpu_suspend();

	/* ensure not to come back here if it wasn't intended */
	PSPR = 0;

#ifdef DEBUG
	printk(KERN_DEBUG "*** made it back from resume\n");
#endif

#ifdef CONFIG_SA1100_H3XXX
	if ( machine_is_h3xxx()) {
		ipaq_model_ops.gedr = GEDR;
		ipaq_model_ops.icpr = ICPR;
	}
#endif

	/* restore registers */
	RESTORE(GPDR);
	RESTORE(GRER);
	RESTORE(GFER);
	RESTORE(GAFR);

	/* clear any edge detect bit */
	GEDR = GEDR;

	RESTORE(PPDR);
	RESTORE(PPSR);
	RESTORE(PPAR);
	RESTORE(PSDR);

	RESTORE(Ser1SDCR0);

	PSSR = PSSR_PH;

	RESTORE(OSMR0);
	RESTORE(OSMR1);
	RESTORE(OSMR2);
	RESTORE(OSMR3);
	RESTORE(OSCR);
	RESTORE(OIER);

#ifdef CONFIG_SA1100_H3XXX
/* OSMR0 may have fired before we went to sleep, but after interrupts
   were shut off.  Set OSMR0 to something plausible */
	OSMR0 = OSCR + LATCH;
#endif
	ICLR = 0;
	ICCR = 1;
	RESTORE(ICMR);
#if	defined(CONFIG_SA1100_H3XXX)
	RESTORE(PWER);
#endif
#if defined (CONFIG_SA1100_JORNADA720) || defined (CONFIG_SA1100_JORNADA56X)
	PWER = 0; /* jca */
#endif
	/* restore current time */
	xtime.tv_sec = RCNR;

	leds_event(led_start);

	sti();

	/*
	 * Restore the CPU frequency settings.
	 */
#ifdef CONFIG_CPU_FREQ
	cpufreq_restore();
#endif
	return 0;
}

unsigned long sleep_phys_sp(void *sp)
{
	return virt_to_phys(sp);
}

/*
 * If pm_suggest_suspend_hook is non-NULL, it is called by pm_suggest_suspend.
 */
int (*pm_suggest_suspend_hook)(int state);
EXPORT_SYMBOL(pm_suggest_suspend_hook);

/*
 * If pm_use_sbin_pm_helper is nonzero, then run_sbin_pm_helper is called before suspend and after resume
 */
int pm_use_sbin_pm_helper = 1;
EXPORT_SYMBOL(pm_use_sbin_pm_helper);

int pm_suggest_suspend(void)
{
        if (pm_suggest_suspend_hook) {
                if (pm_suggest_suspend_hook(PM_SUSPEND))
                        return 0;
        }
        if (pm_use_sbin_pm_helper)
                run_sbin_pm_helper(PM_SUSPEND);
	return 0;
}
EXPORT_SYMBOL(pm_suggest_suspend);


/*
 * Send us to sleep.
 */
int pm_suspend(void)
{
	int retval;

	retval = pm_send_all(PM_SUSPEND, (void *)3);
	if ( retval )
		return retval;

#ifdef CONFIG_SA1100_H3XXX
	retval = h3600_power_management(PM_SUSPEND);
	if (retval) {
		pm_send_all(PM_RESUME, (void *)0);
		return retval;
	}
#endif

	retval = pm_do_suspend();

#ifdef CONFIG_SA1100_H3XXX
	/* Allow the power management routines to override resuming */
	while ( h3600_power_management(PM_RESUME) )
		retval = pm_do_suspend();
#endif

	pm_send_all(PM_RESUME, (void *)0);
        if (pm_use_sbin_pm_helper)
                run_sbin_pm_helper(PM_RESUME);
	return retval;
}
EXPORT_SYMBOL(pm_suspend);

/*
 * If sysctl_pm_do_suspend_hook is non-NULL, it is called by sysctl_pm_do_suspend.
 * If it returns a true value, then pm_suspend is not called. 
 * Use this to hook in apmd, for now.
 */
int (*pm_sysctl_suspend_hook)(int state);
EXPORT_SYMBOL(pm_sysctl_suspend_hook);

static int sysctl_pm_suspend(void)
{
        if (pm_sysctl_suspend_hook) {
                if (pm_sysctl_suspend_hook(PM_SUSPEND))
                        return 0;
        }
        return pm_suspend();
}

#ifdef CONFIG_SYSCTL
/*
 * ARGH!  ACPI people defined CTL_ACPI in linux/acpi.h rather than
 * linux/sysctl.h.
 *
 * This means our interface here won't survive long - it needs a new
 * interface.  Quick hack to get this working - use sysctl id 9999.
 */
#warning ACPI broke the kernel, this interface needs to be fixed up.
#define CTL_ACPI 9999
#define ACPI_S1_SLP_TYP 19

static struct ctl_table pm_table[] =
{
	{ACPI_S1_SLP_TYP, "suspend", NULL, 0, 0600, NULL, (proc_handler *)&sysctl_pm_suspend},
	{2, "helper", pm_helper_path, sizeof(pm_helper_path), 0644, NULL, (proc_handler *)&proc_dostring},
	{0}
};

static struct ctl_table pm_dir_table[] =
{
	{CTL_ACPI, "pm", NULL, 0, 0555, pm_table},
	{0}
};

/*
 * Initialize power interface
 */
static int __init pm_init(void)
{
	register_sysctl_table(pm_dir_table, 1);
	return 0;
}

__initcall(pm_init);

#endif
