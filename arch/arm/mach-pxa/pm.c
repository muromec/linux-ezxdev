/*
 * PXA250/210 Power Management Routines
 *
 * Original code for the SA11x0:
 * Copyright (c) 2001 Cliff Brake <cbrake@accelent.com>
 *
 * Modified for the PXA250 by Nicolas Pitre:
 * Copyright (c) 2002 Monta Vista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/sysctl.h>
#include <linux/errno.h>
#include <linux/timer.h>
#include <linux/delay.h>
#ifdef CONFIG_DPM
#include <linux/device.h>
#endif

#include <asm/hardware.h>
#include <asm/memory.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/leds.h>


/*
 * Debug macros
 */
#undef DEBUG

extern void pxa_cpu_standby(void);
extern void pxa_cpu_suspend(unsigned int);
extern void pxa_cpu_resume(void);
extern void (*pm_idle)(void);

#define SAVE(x)		sleep_save[SLEEP_SAVE_##x] = x
#define RESTORE(x)	x = sleep_save[SLEEP_SAVE_##x]

/*      How long we will in sleep mode if duty cycle.   */
unsigned int  pm_sleeptime=0;  /*      In seconds.     */

/*
 * List of global PXA peripheral registers to preserve.
 * More ones like CP and general purpose register values are preserved
 * with the stack pointer in sleep.S.
 */
enum {	SLEEP_SAVE_START = 0,

	SLEEP_SAVE_OSCR, SLEEP_SAVE_OIER,
	SLEEP_SAVE_OSMR0, SLEEP_SAVE_OSMR1, SLEEP_SAVE_OSMR2, SLEEP_SAVE_OSMR3,

	SLEEP_SAVE_GPDR0, SLEEP_SAVE_GPDR1, SLEEP_SAVE_GPDR2,
	SLEEP_SAVE_GRER0, SLEEP_SAVE_GRER1, SLEEP_SAVE_GRER2,
	SLEEP_SAVE_GFER0, SLEEP_SAVE_GFER1, SLEEP_SAVE_GFER2,
	SLEEP_SAVE_GAFR0_L, SLEEP_SAVE_GAFR1_L, SLEEP_SAVE_GAFR2_L,
	SLEEP_SAVE_GAFR0_U, SLEEP_SAVE_GAFR1_U, SLEEP_SAVE_GAFR2_U,

#if defined(CONFIG_CPU_BULVERDE)
        SLEEP_SAVE_GPDR3, SLEEP_SAVE_GRER3, SLEEP_SAVE_GFER3,
        SLEEP_SAVE_GAFR3_L, SLEEP_SAVE_GAFR3_U,
	SLEEP_SAVE_KPC,
#endif

	SLEEP_SAVE_FFIER, SLEEP_SAVE_FFLCR, SLEEP_SAVE_FFMCR,
	SLEEP_SAVE_FFSPR, SLEEP_SAVE_FFISR,
	SLEEP_SAVE_FFDLL, SLEEP_SAVE_FFDLH,

	SLEEP_SAVE_ICMR,
	SLEEP_SAVE_CKEN,

#ifdef CONFIG_ARCH_MAINSTONE
    SLEEP_SAVE_MST_GPSWR,
    SLEEP_SAVE_MST_MSCWR1,
    SLEEP_SAVE_MST_MSCWR2,
    SLEEP_SAVE_MST_MSCWR3,
    SLEEP_SAVE_MST_MSCRD,
    SLEEP_SAVE_MST_INTMSKENA,
    SLEEP_SAVE_MST_INTSETCLR,
#endif

        SLEEP_SAVE_PWER,
        SLEEP_SAVE_PSTR,   
        SLEEP_SAVE_OMCR4,
        SLEEP_SAVE_OSCR4, 
        SLEEP_SAVE_OSMR4,
        SLEEP_SAVE_MDREFR,
        SLEEP_SAVE_OSSR,
        SLEEP_SAVE_PGSR0,
        SLEEP_SAVE_PGSR1,
        SLEEP_SAVE_PGSR2,
        SLEEP_SAVE_PGSR3,

	SLEEP_SAVE_CKSUM,

	SLEEP_SAVE_SIZE
};

int	pm_do_standby()
{
	unsigned long unused;

	cli();
	clf();

	leds_event(led_stop);

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
	leds_event(led_start);

	sti();
	return 0;
}

/* +++ */
#define DPCSR __REG(0x400000a4)   /* DMA Programmed I/O Control Status Reg */
#define DPCSR_BRG_SPLIT (1 << 31) /* Posted writes and split reads bit */
#define DPCSR_BRG_BUSY (1 << 0)   /* Bridge busy status bit */

int pm_do_suspend(unsigned int cpumode)
{
	unsigned long sleep_save[SLEEP_SAVE_SIZE];
	unsigned long checksum = 0;
	unsigned long unused = 0;
	int i;

	cli();
	clf();

	leds_event(led_stop);

	/* preserve current time */
	RCNR = xtime.tv_sec;

#if defined(CONFIG_ARCH_MAINSTONE)
    SAVE(MST_GPSWR);
    SAVE(MST_MSCWR1);
    SAVE(MST_MSCWR2);
    SAVE(MST_MSCWR3);
    SAVE(MST_MSCRD);
    SAVE(MST_INTMSKENA);
    SAVE(MST_INTSETCLR);
#endif

	/* 
	 * Temporary solution.  This won't be necessary once
	 * we move pxa support into the serial driver
	 * Save the FF UART 
	 */
	SAVE(FFIER);
	SAVE(FFLCR);
	SAVE(FFMCR);
	SAVE(FFSPR);
	SAVE(FFISR);
	FFLCR |= 0x80;
	SAVE(FFDLL);
	SAVE(FFDLH);
	FFLCR &= 0xef;

	/*For SDRAM sightings.	*/
	SAVE(MDREFR);
	/* save vital registers */
	SAVE(OSCR);
	SAVE(OSSR);
	SAVE(OSMR0);
	SAVE(OSMR1);
	SAVE(OSMR2);
	SAVE(OSMR3);
	SAVE(OIER);

	SAVE(GPDR0); SAVE(GPDR1); SAVE(GPDR2);
	SAVE(GRER0); SAVE(GRER1); SAVE(GRER2);
	SAVE(GFER0); SAVE(GFER1); SAVE(GFER2);
	SAVE(GAFR0_L); SAVE(GAFR0_U);
	SAVE(GAFR1_L); SAVE(GAFR1_U);
	SAVE(GAFR2_L); SAVE(GAFR2_U);

#if defined(CONFIG_CPU_BULVERDE)
	SAVE(GPDR3);
	SAVE(GRER3);
	SAVE(GFER3);
	SAVE(GAFR3_L); SAVE(GAFR3_U);
	SAVE(KPC);
#endif	
	
	SAVE(ICMR);
	ICMR = 0;

	SAVE(CKEN);
	
	/*	10-29 */
	SAVE(PGSR0);
	SAVE(PGSR1);
	SAVE(PGSR2);
	SAVE(PGSR3);
	
#if defined(CONFIG_CPU_BULVERDE)
	if( cpumode==CPUMODE_STANDBY ) 
		CKEN = CKEN22_MEMC | CKEN9_OSTIMER | CKEN16_LCD |CKEN0_PWM0;
	else
		CKEN = CKEN22_MEMC | CKEN9_OSTIMER;

	PCFR = 0x66;    
	/*	for use I SRAM as framebuffer.	*/
	//	PSLR |= 0X4;
	PSLR |= 0xF04; 
	/*	For Keypad wakeup.	*/
	KPC &=~KPC_ASACT;
	KPC |=KPC_AS;
	PGSR0 = 0x00008800;
	PGSR1 = 0x00000002;
	PGSR2 = 0x0001FC00;
	PGSR3 = 0x00001F81;

//	PWER  = 0x80000002;
	PWER  = 0xC0000002;
	PRER  = 0x2;
	PFER  = 0x2;

	PKWR  = 0x000FD000;
	/*	Need read PKWR back after set it.	*/
	PKWR;
#else
	CKEN =0;
#endif	

	/* Note: wake up source are set up in each machine specific files */

	/* clear GPIO transition detect  bits */
	GEDR0 = GEDR0; GEDR1 = GEDR1; GEDR2 = GEDR2;

	/* Clear sleep reset status */
	RCSR = RCSR_SMR;

	/* Clear edge-detect status register. */
	PEDR = 0xCE00FE1B;

	/* set resume return address */
	if( cpumode!=CPUMODE_STANDBY ) 
		PSPR = virt_to_phys(pxa_cpu_resume);

	if( cpumode==CPUMODE_STANDBY ) {
		SAVE(PSTR);
		SAVE(OMCR4);
		SAVE(OSCR4);
		SAVE(OSMR4);
		SAVE(OIER);
	}	

	/* before sleeping, calculate and save a checksum */
	for (i = 0; i < SLEEP_SAVE_SIZE - 1; i++)
		checksum += sleep_save[i];
	sleep_save[SLEEP_SAVE_CKSUM] = checksum;

	if( cpumode!=CPUMODE_STANDBY ) {
		/*for self-refresh sightings.	*/
		MDREFR &= 0xE3D7AFFF;
    	/* Check the sleeptime to see how we need to wake up,in ms. */
    	if( pm_sleeptime !=0 ) {   /* come back sometimes later.*/
        	if(pm_sleeptime < 65 ) {    /*65s,use periodic interrupt*/
				/*	Clear PICE bit in RTSR.	*/
            	RTSR &= ~RTSR_PICE;
				/*	set PIAR	*/
            	PIAR = pm_sleeptime*1000;/*set to sleep time,in ms.*/
            	udelay(10); /*  Ensure at least 2 CPY cycles pass.*/
				/*	Clear PIALE bit in RTSR	*/
            	RTSR &= ~RTSR_PIALE;
            	RTSR |= RTSR_PIALE;
            	RTSR |= RTSR_PICE;
        	}
        	else {  /*  we need to use RTC. */
        		/*  Need to use other RTC wakeep methods..  */
        	}
    	}
	}	

	
#if defined(CONFIG_CPU_BULVERDE)
	/*	Use OST 4 to wakeup system from standby */
	if( cpumode==CPUMODE_STANDBY ) {
		/*	
		 *	Here we use OS timer 4 as wakeup src. Maybe this is helpful
		 *	when we decide to use standby to replace idle.
		 */
		/*	All SRAM are on, PI is active with clock running.	*/
		PSTR = 0x00000F08;
		/*	For standby we use OSCR4 as prioridic wakeup source. */
		/* CRES field of OMCR is set to 011, ticks will be 1 seconds.*/
		OMCR4  = 0x0B;
		OSCR4 = 1;
		OSMR4  = OSCR4 + pm_sleeptime;
		OIER = (OIER&0xFFF)|0x10;
	}
#endif	

	switch ( cpumode ) {
		case CPUMODE_SLEEP:
		case CPUMODE_DEEPSLEEP:
			pxa_cpu_suspend(cpumode);
			break;

#if defined(CONFIG_CPU_BULVERDE)
		case CPUMODE_STANDBY:
   			/*  Standby the CPU. */
			pxa_cpu_standby();
			break;
#endif			
	}	/*	end of switch	*/

	/* after sleeping, validate the checksum */
	checksum = 0;
	for (i = 0; i < SLEEP_SAVE_SIZE - 1; i++)
		checksum += sleep_save[i];

	/* if invalid, display message and wait for a hardware reset */
	if (checksum != sleep_save[SLEEP_SAVE_CKSUM]) {
#ifdef CONFIG_ARCH_LUBBOCK
		LUB_HEXLED = 0xbadbadc5;
#endif
#ifdef CONFIG_ARCH_MAINSTONE
		leds_event(led_start);
		MST_LEDDAT1 = 0xbadbadc5;
#endif
		while (1);
	}

	/* ensure not to come back here if it wasn't intended */
	PSPR = 0;

	if( cpumode==CPUMODE_STANDBY ) {
		/*	Clear PEDR	*/
		RESTORE(OIER);
		RESTORE(MDREFR);
		RESTORE(PSTR);
		OSSR =0x10;
		OSMR4 = OSCR4 + 3;
		OSCR4 = OSCR4 + 4;
	}

	/* restore registers */
	RESTORE(PGSR0);
	RESTORE(PGSR1);
	RESTORE(PGSR2);
	RESTORE(PGSR3);

	RESTORE(GPDR0); RESTORE(GPDR1); RESTORE(GPDR2);
	RESTORE(GRER0); RESTORE(GRER1); RESTORE(GRER2);
	RESTORE(GFER0); RESTORE(GFER1); RESTORE(GFER2);
	RESTORE(GAFR0_L); RESTORE(GAFR0_U);
	RESTORE(GAFR1_L); RESTORE(GAFR1_U);
	RESTORE(GAFR2_L); RESTORE(GAFR2_U);

#if defined(CONFIG_CPU_BULVERDE)
	RESTORE(KPC);
	RESTORE(GAFR3_L); RESTORE(GAFR3_U);
	RESTORE(GPDR3);
	RESTORE(GRER3);
	RESTORE(GFER3);
#endif	

	PSSR = PSSR_PH | PSSR_OTGPH;

	RESTORE(OSMR0);
	RESTORE(OSMR1);
	RESTORE(OSMR2);
	RESTORE(OSMR3);
	OSCR = OSMR0 - LATCH + 1;
	RESTORE(OIER);

	RESTORE(CKEN);

	ICLR = 0;
	ICCR = 1;
	RESTORE(ICMR);

#if defined(CONFIG_ARCH_MAINSTONE)
    /*  Restore Mainstone Board registers.  */
    RESTORE(MST_GPSWR);
    RESTORE(MST_MSCWR1);
    RESTORE(MST_MSCWR2);
    RESTORE(MST_MSCWR3);
    RESTORE(MST_MSCRD);
    RESTORE(MST_INTMSKENA);
    RESTORE(MST_INTSETCLR);
#endif

	/* 
	 * Temporary solution.  This won't be necessary once
	 * we move pxa support into the serial driver.
	 * Restore the FF UART.
	 */
	RESTORE(FFMCR);
	RESTORE(FFSPR);
	RESTORE(FFLCR);
	FFLCR |= 0x80;
	RESTORE(FFDLH);
	RESTORE(FFDLL);
	RESTORE(FFLCR);
	RESTORE(FFISR);
	FFFCR = 0x07;
	RESTORE(FFIER);

	/* restore current time */
	xtime.tv_sec = RCNR;

#ifdef DEBUG
	printk(KERN_DEBUG "*** made it back from resume\n");
#endif

	leds_event(led_start);

	sti();

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
#warning ACPI broke the kernel, this interface needs to be fixed up.
#define CTL_ACPI 9999
#define ACPI_S1_SLP_TYP 19

/*
 * Send us to sleep.
 */
/*	New added function for different CPU mode. 	*/
int  m13_enable = 0;
int cpu_mode_set(int mode)
{
	/*  Need further work to support different CPU modes.   */

	pm_do_suspend(mode);
#if 0
	if( mode==CPUMODE_STANDBY )
		pm_do_standby();
	else if (mode==CPUMODE_DEEPSLEEP)
		pm_do_suspend(mode);
	else pm_do_suspend(CPUMODE_SLEEP);
#endif
	return 0;
}

static int
sysctl_pm_do_suspend(ctl_table *ctl, int write, struct file *filp,
        void *buffer, size_t *lenp)
{
    char buf[16], *p;
    int len, left = *lenp;
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
