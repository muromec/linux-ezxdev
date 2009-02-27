/*
 * PXA250/210 Power Management Routines
 *
 * Original code for the SA11x0:
 * Copyright (c) 2001 Cliff Brake <cbrake@accelent.com>
 *
 * Modified for the PXA250 by Nicolas Pitre:
 * Copyright (c) 2002 Monta Vista Software, Inc.
 *
 * Modified for the Ezx by Zhuang Xiaofan:
 * Copyright (c) 2004, 2005 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 *
 *
 * History:
 *  Date         Author         Reference       Comment
 * =========    ==========     =============    ===============
 * 08.30.2004   Jordan/Xiaofan LIBee37568     Intel pm workaround2
 * 12.22.2005   Liu Lin	       LIBhh64682 	GPIO99 should be low		
 *
 * 
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/sysctl.h>
#include <linux/errno.h>
#include <linux/timex.h>
#ifdef CONFIG_DPM
#include <linux/device.h>
#endif
#include <linux/delay.h>

#include <asm/hardware.h>
#include <asm/memory.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/leds.h>


/*
 * Debug macros
 */
#undef DEBUG

extern void pxa_cpu_suspend(unsigned int);
extern void pxa_cpu_resume(void);
extern void (*pm_idle)(void);

#define SAVE(x)		sleep_save[SLEEP_SAVE_##x] = x
#define RESTORE(x)	x = sleep_save[SLEEP_SAVE_##x]

#ifdef CONFIG_IPM_DEEPIDLE
int enable_deepidle = 1;

void deepidle_enable(void)
{
	++enable_deepidle;
}

void deepidle_disable(void)
{
	--enable_deepidle;
}

#endif

/*
 * List of global PXA peripheral registers to preserve.
 * More ones like CP and general purpose register values are preserved
 * with the stack pointer in sleep.S.
 */
enum {	SLEEP_SAVE_START = 0,

	SLEEP_SAVE_OSCR, SLEEP_SAVE_OIER,
	SLEEP_SAVE_OSMR0, SLEEP_SAVE_OSMR1, SLEEP_SAVE_OSMR2, SLEEP_SAVE_OSMR3,

	SLEEP_SAVE_PGSR0, SLEEP_SAVE_PGSR1, SLEEP_SAVE_PGSR2, SLEEP_SAVE_PGSR3,

	SLEEP_SAVE_GPDR0, SLEEP_SAVE_GPDR1, SLEEP_SAVE_GPDR2, SLEEP_SAVE_GPDR3,
	SLEEP_SAVE_GRER0, SLEEP_SAVE_GRER1, SLEEP_SAVE_GRER2, SLEEP_SAVE_GRER3,
	SLEEP_SAVE_GFER0, SLEEP_SAVE_GFER1, SLEEP_SAVE_GFER2, SLEEP_SAVE_GFER3,
	SLEEP_SAVE_GAFR0_L, SLEEP_SAVE_GAFR1_L, SLEEP_SAVE_GAFR2_L, SLEEP_SAVE_GAFR3_L,
	SLEEP_SAVE_GAFR0_U, SLEEP_SAVE_GAFR1_U, SLEEP_SAVE_GAFR2_U, SLEEP_SAVE_GAFR3_U,

	SLEEP_SAVE_STIER, SLEEP_SAVE_STLCR, SLEEP_SAVE_STMCR,
	SLEEP_SAVE_STSPR, SLEEP_SAVE_STISR,
	SLEEP_SAVE_STDLL, SLEEP_SAVE_STDLH,

	SLEEP_SAVE_ICMR,
	SLEEP_SAVE_CKEN,

	SLEEP_SAVE_CKSUM,

	SLEEP_SAVE_SIZE
};

int	pm_do_standby(void)
{
	unsigned long unused;

	cli();
	clf();

	printk(KERN_INFO "In function %s.\n", __FUNCTION__);

	/*	This also works fine by useing SW12 to wake it up.*/	
	/*	Set PSTR.	*/
	PSTR = 0x4;
	/*	Copied from PalmOS Group.	*/
	PWER = 0x80000002;
	PRER = 0x2;
	PFER = 0x2;
	PKWR = 0x000E0000;
	/*	Copied from PalmOS Group end.*/
	
#if 1
	/*	
	 *	Use below's config it will return back from
	 *	standby mode automatically.
	 */

	/*	Set PSTR.	*/
	PSTR = 0x4;
	/*	Program PWER, PKWR, PRER and PFER	*/
	/*	All can wake it up.	*/
	PWER = 0xCE00FE1F;	
	/*	
	 *	No high level on GPIO can wake up it.
 	 *	If allow high level on GPIO to wake it up,
 	 *	set PKWR to 0x000FFFFF 
 	 */
	PKWR = 0x0;			
	/*	Rising edge can wake it up.	*/	
	PRER = 0xFE1B;
	/*	Falling edge can wake it up.*/
	PFER = 0x0FE1B;
#endif	

	/*	Standby the CPU. */
	__asm__ __volatile__("\n\
\n\
\n\
   	ldr r0, 	[%1] \n\
    mov r1, 	#0x18 \n\
	@ orr	r1,	r0,	#0x08 \n\
    mov r2, 	#2      \n\
    b   1f \n\
\n\
   	.align  5 \n\
1: \n\
   	@ All needed values are now in registers. \n\
    @ These last instructions should be in cache \n\
    @ enter standby mode \n\
\n\
   	mcr p14, 0, r2, c7, c0, 0 \n\
\n\
    @ CPU will stop exec until be waked up. \n\
   	@ After be waken, just return. \n\
	str r1, [%1] \n\
	.rept 11 \n\
	nop \n\
	.endr"

	: "=&r" (unused)
	: "r" (&PSSR)
	: "r0", "r1", "r2");

	printk(KERN_INFO "Return from standby mode.\n");

	sti();
	return 0;
}

/*
 * Used to save & restore the RTSR, SWAR and other registers.
 * Flag:
 * 0 means save
 * 1 means restore
 */
#define SAVE_REG	0
#define RESTORE_REG	1

static inline void register_handle(unsigned short flag)
{
	unsigned long swar1 = 0;
	unsigned long swar2 = 0;
	unsigned long swcr = 0;
	static int need_restore = 0;


	switch(flag) {
		case SAVE_REG:
			if ((RTSR & RTSR_SWCE) != 0) 
				/* somebody else is using it, don't touch */
				return;

#if 0
			swar1 = SWAR1;
			swar2 = SWAR2;
			swcr  = SWCR;

			SWAR1 = SWAR2 = (99 |
					59 << 7 |
					59 << (7 + 6) |
					31 << (7 + 6 + 6));

			/* 
			 * Test results1:
			 * Must sleep for >50us before the SWCR is reset to 0. 
			 *
			 * !!Test results2:
			 * After RTSR_SWCE enabled, SWCR starts after 50ms. :(
			 * How long will it take before SWCR starts count? 
			 * Intel's spec doesn't say it.
			 */
			udelay(50);
			RTSR |= RTSR_SWCE;
			need_restore = 1;
#endif 
			/* To skip the first SWCR interrupt */
			SWCR = (1 |
				1 << 7 |
				1 << (7 + 6) |
				1 << (7 + 6 + 6));
			udelay(50);

			RTSR |= RTSR_SWCE;
			break;

		case RESTORE_REG:
			if (need_restore == 1) {
				SWAR1 = swar1;
				SWAR2 = swar2;
				SWCR  = swcr;
				udelay(50);
				RTSR &= ~RTSR_SWCE;

				need_restore = 0;
			}
			break;
		default:
			printk("register handle: fatal error.\n");
	}

	return;
}

static inline void time_save(unsigned long *rcnr, unsigned long *swcr)
{
	*rcnr = RCNR;
	register_handle(SAVE_REG);
	*swcr = SWCR;

	return;
}

/* 
 * Get the interval of two SWCR time format, quick and dirty.
 * For more info, please refer to Intel's spec and 
 * drivers/char/sa1100-rtc.c
 * 					---kai 10/21, 2005
 */
static inline unsigned long ms_interval(unsigned long end, unsigned long start)
{
/*
 * BUG!!
 * Test result shows that, the SWCR register will roll back to 0 after it 
 * reaches 23 hours 59 minutes 59 seconds and 99 hundredths, 
 * but not 31 hours 59 minutes 59 seconds and 99 hundredths (the spec wrong).
 *
 *                                      ---kai 10/21, 2005
 */
#define MAX_SWCR_MS (24 * 3600 * 1000)

	unsigned long hours_e = (end & 0xf80000) >> (6 + 6 + 7);
	unsigned long min_e = (end & 0x07e000) >> (6 + 7);
	unsigned long sec_e = (end & 0x001f80) >> 7;
	unsigned long hun_e = end & 0x00007f;

	unsigned long hours_s = (start & 0xf80000) >> (6 + 6 + 7);
	unsigned long min_s = (start & 0x07e000) >> (6 + 7);
	unsigned long sec_s = (start & 0x001f80) >> 7;
	unsigned long hun_s = start & 0x00007f;

	unsigned long end_ms = (hours_e * 3600 * 1000) +
				(min_e * 60 * 1000) +
				(sec_e * 1000) +
				(hun_e * 10);
	unsigned long start_ms = (hours_s * 3600 * 1000) +
				(min_s * 60 * 1000) +
				(sec_s * 1000) +
				(hun_s * 10);

	/*
	printk("(S)hour: %d, min: %d, sec: %d, hun: %d\n", 
		hours_s, min_s, sec_s, hun_s);
	printk("(E)hour: %d, min: %d, sec: %d, hun: %d\n", 
		hours_e, min_e, sec_e, hun_e);
	*/

	if (unlikely(end_ms < start_ms))
		return (MAX_SWCR_MS - start_ms + end_ms);
	else
		return (end_ms - start_ms);
}

static inline void time_restore(unsigned long s_rcnr, unsigned long s_swcr)
{
	unsigned long duration_ms = 0;
	unsigned long duration = RCNR - s_rcnr;
	unsigned long usec = 0, sec = 0;

	if (unlikely(duration >= 24 * 3600)) {
		xtime.tv_sec += duration;
		goto restore_reg;
	}

	duration_ms = ms_interval(SWCR, s_swcr);
	/* printk("+++   sleep duration %ld ms.\n", duration_ms); */

	usec = duration_ms * 1000;

	while (usec >= 1000000) {
		usec -= 1000000;
		sec++;
	}

	xtime.tv_sec += sec;
	xtime.tv_usec += usec;
	/* Easy, xtime will handle the out of bounds of usec. */

restore_reg:
	register_handle(RESTORE_REG);
	return;
}


int pm_do_suspend(unsigned int mode)
{
	unsigned long sleep_save[SLEEP_SAVE_SIZE];
	unsigned long checksum = 0;
        unsigned long start_rcnr, start_swcr;
	int i;

	cli();
	clf();

	GPSR(GPIO90_USB_P3_5) = GPIO_bit(GPIO90_USB_P3_5);
	GPSR(GPIO91_USB_P3_1) = GPIO_bit(GPIO91_USB_P3_1);
	GPCR(GPIO113_USB_P3_3) = GPIO_bit(GPIO113_USB_P3_3);
	GAFR2_U  &= 0xff0fffff;
	set_GPIO_mode(GPIO90_USB_P3_5 | GPIO_OUT );
	set_GPIO_mode(GPIO91_USB_P3_1 | GPIO_OUT );
	set_GPIO_mode(GPIO113_USB_P3_3 | GPIO_OUT );

	/* 
	 * Temporary solution.  This won't be necessary once
	 * we move pxa support into the serial driver
	 */

	SAVE(STIER);
	SAVE(STLCR);
	SAVE(STMCR);
	SAVE(STSPR);
	SAVE(STISR);
	STLCR |= 0x80;
	SAVE(STDLL);
	SAVE(STDLH);
	STLCR &= 0xef;

	/* preserve current time */
        time_save(&start_rcnr, &start_swcr);

	/* save vital registers */
	SAVE(OSCR);
	SAVE(OSMR0);
	SAVE(OSMR1);
	SAVE(OSMR2);
	SAVE(OSMR3);
	SAVE(OIER);

	SAVE(PGSR0); SAVE(PGSR1); SAVE(PGSR2); SAVE(PGSR3);

	SAVE(GPDR0); SAVE(GPDR1); SAVE(GPDR2); SAVE(GPDR3);
	SAVE(GRER0); SAVE(GRER1); SAVE(GRER2); SAVE(GRER3);
	SAVE(GFER0); SAVE(GFER1); SAVE(GFER2); SAVE(GFER3);
	SAVE(GAFR0_L); SAVE(GAFR0_U); 
	SAVE(GAFR1_L); SAVE(GAFR1_U);
	SAVE(GAFR2_L); SAVE(GAFR2_U);
	SAVE(GAFR3_L); SAVE(GAFR3_U);

	SAVE(ICMR);
	ICMR = 0;

	SAVE(CKEN);
	
	/*	Change accoding to Nick's experience.	*/
	CKEN = CKEN22_MEMC;
	/*	Change accoding to Nick's experience end.	*/

	/* Note: wake up source are set up in each machine specific files */

	/* clear GPIO transition detect  bits */
	GEDR0 = GEDR0; GEDR1 = GEDR1; GEDR2 = GEDR2; GEDR3 = GEDR3;

	/* Clear sleep reset status */
	RCSR = RCSR_SMR;

	/* set resume return address */
        *(unsigned long *)(phys_to_virt(RESUME_ADDR)) = virt_to_phys(pxa_cpu_resume);
        *(unsigned long *)(phys_to_virt(FLAG_ADDR)) = SLEEP_FLAG;

	/* before sleeping, calculate and save a checksum */
	for (i = 0; i < SLEEP_SAVE_SIZE - 1; i++)
		checksum += sleep_save[i];
	sleep_save[SLEEP_SAVE_CKSUM] = checksum;

	/* *** go zzz *** */
	pxa_cpu_suspend(mode);

	/* after sleeping, validate the checksum */
	checksum = 0;
	for (i = 0; i < SLEEP_SAVE_SIZE - 1; i++)
		checksum += sleep_save[i];

	/* if invalid, display message and wait for a hardware reset */
	if (checksum != sleep_save[SLEEP_SAVE_CKSUM]) {
		while (1);
	}

	/* ensure not to come back here if it wasn't intended */
        *(unsigned long *)(phys_to_virt(RESUME_ADDR)) = 0;
        *(unsigned long *)(phys_to_virt(FLAG_ADDR)) = OFF_FLAG;

        GPSR(GPIO30_USB_P3_2) = GPIO_bit(GPIO30_USB_P3_2);
        GPCR(GPIO31_USB_P3_6) = GPIO_bit(GPIO31_USB_P3_6);
        GPCR(GPIO56_USB_P3_4) = GPIO_bit(GPIO56_USB_P3_4);
	GPSR(GPIO90_USB_P3_5) = GPIO_bit(GPIO90_USB_P3_5);
	GPSR(GPIO91_USB_P3_1) = GPIO_bit(GPIO91_USB_P3_1);
	GPCR(GPIO113_USB_P3_3) = GPIO_bit(GPIO113_USB_P3_3);
	
	/*restore GPIO pin level*/
	GPSR(GPIO_WDI_AP)  = GPIO_bit(GPIO_WDI_AP);
#ifndef CONFIG_ARCH_EZX_SUMATRA
	GPSR(GPIO_CLI_RESETB) = GPIO_bit(GPIO_CLI_RESETB);
#endif	
	GPSR(GPIO_CAM_RST) = GPIO_bit(GPIO_CAM_RST);

	GPSR(GPIO_BT_RESET) = GPIO_bit(GPIO_BT_RESET);
 	GPCR(GPIO_BB_FLASHMODE_EN) = GPIO_bit(GPIO_BB_FLASHMODE_EN);
	GPSR(GPIO_CAM_EN) = GPIO_bit(GPIO_CAM_EN);
	GPSR(GPIO_SYS_RESTART) = GPIO_bit(GPIO_SYS_RESTART);
	GPSR(GPIO_BT_WAKEUP) = GPIO_bit(GPIO_BT_WAKEUP);
	GPSR(GPIO45_BTRTS) = GPIO_bit(GPIO45_BTRTS);/* avoid 10us glitch in BTRTS ,set high*/
        GPSR(GPIO43_BTTXD) = GPIO_bit(GPIO43_BTTXD);/*avoid 10us glitch in BTTXD ,set high*/
	GPCR(GPIO_LCD_OFF) = GPIO_bit(GPIO_LCD_OFF);
#ifndef CONFIG_ARCH_EZX_SUMATRA
	GPCR(GPIO_LCD_CM) = GPIO_bit(GPIO_LCD_CM);
#endif	
	GPSR(GPIO_AP_RDY) = GPIO_bit(GPIO_AP_RDY);
	GPCR(GPIO_USB_READY) = GPIO_bit(GPIO_USB_READY);/* e12707 clear gpio99 to low */
	GPSR(GPIO_BB_RESET) = GPIO_bit(GPIO_BB_RESET);
#ifdef CONFIG_ARCH_EZX_MARTINIQUE
	GPSR(GPIO_WLAN_PWDN_B) = GPIO_bit(GPIO_WLAN_PWDN_B);
        GPSR(GPIO_HOST_WLAN_WAKEB) = GPIO_bit(GPIO_HOST_WLAN_WAKEB);
        GPCR(GPIO_WLAN_RESETB) = GPIO_bit(GPIO_WLAN_RESETB);
        
        GPSR(GPIO_MMC_CMD) = GPIO_bit(GPIO_MMC_CMD);
        GPCR(GPIO_MMC_DATA0) = GPIO_bit(GPIO_MMC_DATA0);
        GPCR(GPIO_MMC_DATA1) = GPIO_bit(GPIO_MMC_DATA1);
        GPCR(GPIO_MMC_DATA2) = GPIO_bit(GPIO_MMC_DATA2);
        GPCR(GPIO_MMC_DATA3) = GPIO_bit(GPIO_MMC_DATA3);

#endif       

	/* restore registers */
	RESTORE(PGSR0); RESTORE(PGSR1); RESTORE(PGSR2); RESTORE(PGSR3);

	RESTORE(GPDR0); RESTORE(GPDR1); RESTORE(GPDR2); RESTORE(GPDR3);
	RESTORE(GRER0); RESTORE(GRER1); RESTORE(GRER2); RESTORE(GRER3);
	RESTORE(GFER0); RESTORE(GFER1); RESTORE(GFER2); RESTORE(GFER3);

	RESTORE(GAFR0_L); RESTORE(GAFR0_U);
	RESTORE(GAFR1_L); RESTORE(GAFR1_U);
	RESTORE(GAFR2_L); RESTORE(GAFR2_U);
	RESTORE(GAFR3_L); RESTORE(GAFR3_U);


	//PSSR = PSSR_PH;
	PSSR = PSSR_PH | PSSR_RDH;
	
	RESTORE(OSMR0);
	RESTORE(OSMR1);
	RESTORE(OSMR2);
	RESTORE(OSMR3);

	/* to avoid os timer interrupt lost.
	 *
	 * OSCR_MAGIC is the time after restoring OSCR till restore of OIER.
	 * Cause only after the restore of OIER, the match of OSCR and OSMR0
	 * could generate interrupt.
	 *
	 * From our test, each R/W of Bulverde registers will cost 1us, i.e.,
	 * about 3.5 oscrs (in a 3.5 M HZ system), here we use 4 oscrs.
	 */
#define OSCR_MAGIC (4 * 5)
	RESTORE(OSCR);
	if (unlikely(time_after(OSCR + OSCR_MAGIC, OSMR0))) {
		OSCR = OSMR0 - OSCR_MAGIC; 	/* force a match */
	}
	RESTORE(OIER);

	RESTORE(CKEN);

	/* restore current time */
	time_restore(start_rcnr, start_swcr);

	ICLR = 0;
	ICCR = 1;
	RESTORE(ICMR);

	/* 
	 * Temporary solution.  This won't be necessary once
	 * we move pxa support into the serial driver.
	 * Restore the FF UART.
	 */
#if 10
	RESTORE(STMCR);
	RESTORE(STSPR);
	RESTORE(STLCR);
	STLCR |= 0x80;
	RESTORE(STDLH);
	RESTORE(STDLL);
	RESTORE(STLCR);
	RESTORE(STISR);
	STFCR = 0x07;
	RESTORE(STIER);
#endif
#ifdef DEBUG
	printk(KERN_DEBUG "*** made it back from resume\n");
#endif
	return 0;
}

unsigned long sleep_phys_sp(void *sp)
{
	return virt_to_phys(sp);
}

#ifdef CONFIG_SYSCTL
/*
 * ARGH!  ACPI people defined CTL_ACPI in linux/acpi.h rather than
 * linux/sysctl.h.
 *
 * This means our interface here won't survive long - it needs a new
 * interface.  Quick hack to get this working - use sysctl id 9999.
 */
#define CTL_ACPI 9999
#define ACPI_S1_SLP_TYP 19
#define ACPI_DEEPIDLE 20

/*
 * Send us to sleep.
 */
/*	New added function for different CPU mode. 	*/
int  m13_enable = 0;
int cpu_mode_set(int mode)
{
	/*  Need further work to support different CPU modes.   */

	if( mode==CPUMODE_STANDBY )
		pm_do_standby();
	else if (mode==CPUMODE_DEEPSLEEP)
		pm_do_suspend(mode);
	else pm_do_suspend(CPUMODE_SLEEP);

	return 0;
}

static int
sysctl_pm_do_suspend(ctl_table *ctl, int write, struct file *filp,
        void *buffer, size_t *lenp)
{
    char buf[16], *p;
    int  len, left = *lenp;
   	int retval;

    if (!left || (filp->f_pos && !write)) {
        *lenp = 0;
        return 0;
    }

    if (write) {
       unsigned int mode;

        len = left;
        if (left > sizeof(buf))
            left = sizeof(buf);
        if (copy_from_user(buf, buffer, left))
            return -EFAULT;
        buf[sizeof(buf) - 1] = '\0';

        mode = simple_strtoul(buf, &p, 0);

       if( (mode==CPUMODE_SLEEP) || (mode==CPUMODE_DEEPSLEEP) ||
			(mode ==CPUMODE_STANDBY )||(mode ==0 ) )  { // for Sleep.
#ifdef CONFIG_DPM
	   device_suspend(0, SUSPEND_POWER_DOWN);
	   retval = 0;
#else /* CONFIG_DPM */
           retval = pm_send_all(PM_SUSPEND, (void *)3);
#endif
           if (retval == 0) {
               retval = cpu_mode_set(mode);
#ifdef CONFIG_DPM
	       device_resume(RESUME_POWER_ON);
#else /* CONFIG_DPM */
               pm_send_all(PM_RESUME, (void *)0);
#endif
           }
       }  
	   else {
           /*
            *  for modes that except RUN, SLEEP and DEEPSLEEP.
            *  Need not call pm_send_all to send out notifications?
            *  Maybe it is also needed, the most problem is,no device driver
            *  will reponse to PM_SENSE, PM_STANDBY, etc. so we need to
            *  change those device drivers on by one.  :(
            */
           retval = cpu_mode_set(mode);
       }
   } else {
        len = sprintf(buf, "%d\n", CPUMODE_RUN);
        if (len > left)
            len = left;
        if (copy_to_user(buffer, buf, len))
            return -EFAULT;
    }

    *lenp = len;
    filp->f_pos += len;
    return 0;
}

static int
sysctl_pm_do_suspend_sysctl(ctl_table *table, int *name, int nlen,
           void *oldval, size_t *oldlenp,
           void *newval, size_t newlen, void **context)
{

    if (oldval && oldlenp) {
        size_t oldlen;

        if (get_user(oldlen, oldlenp))
            return -EFAULT;

        if (oldlen != sizeof(unsigned int))
            return -EINVAL;
        if (put_user(CPUMODE_RUN, (unsigned int *)oldval) ||
            put_user(sizeof(unsigned int), oldlenp))
            return -EFAULT;
    }
    if (newval && newlen) {
        unsigned int mode;

        if (newlen != sizeof(unsigned int))
            return -EINVAL;

        if (get_user(mode, (unsigned int *)newval))
            return -EFAULT;

        cpu_mode_set(mode);
    }
    return 1;
}


static struct ctl_table pm_table[] =
{
	{
        ctl_name:   ACPI_S1_SLP_TYP,
        procname:   "suspend",
        mode:       0644,
        proc_handler:   sysctl_pm_do_suspend,
        strategy:   sysctl_pm_do_suspend_sysctl,
	},
#ifdef CONFIG_IPM_DEEPIDLE
	{
	ctl_name:   ACPI_DEEPIDLE,
	procname:   "deepidle",
	data:       &enable_deepidle,
	maxlen:     sizeof(int),
	mode:       0644,
	proc_handler:   proc_dointvec
	},
#endif
	{
	ctl_name:    ACPI_S1_SLP_TYP,
	procname:    "oscc", 
	data:	     (void *)(&(OSCC)),
	maxlen:      sizeof(int),
	mode:	     0644,
	proc_handler:    proc_dointvec
	},
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
