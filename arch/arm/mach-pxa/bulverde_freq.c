/*
 * linux/arch/arm/mach-pxa/bulverde_freq.c
 *
 * Functions to change CPU frequencies on the Bulverde processor
 * adopted from Intel code for MontaVista Linux.
 *
 * Author: <source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <asm/io.h>

#include <asm/hardware.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>

#include "bulverde_freq.h"
#include <asm/arch-pxa/bulverde_dpm.h>

/* Since CPDIS and PPDIS is always the same, we use only one
   definition here. */
#define	PDIS	0   /* Core PLL and Peripheral PLL is enabled after FCS. */
#define DEBUGGING   0

/*
 * 	Available CPU frequency list for Bulverde.
 */
static unsigned int  cpufreq_matrix[N_NUM][L_NUM+1];
static void bulverde_freq_debug_info(void);
static volatile int *ramstart;

/*
 *  Init according to bulverde manual.
 */
static void  bulverde_initialize_freq_matrix(void)
{
    int n,l;

    memset(&cpufreq_matrix,0 ,sizeof(cpufreq_matrix));

    for(n=2; n< N_NUM+2 ; n++) {
        for(l=2; l<= L_NUM; l++) {
            cpufreq_matrix[n-2][l-2] = ( 13*n*l /2 ) * 1000 ;
            if( cpufreq_matrix[n-2][l-2] > BLVD_MAX_FREQ )
                cpufreq_matrix[n-2][l-2] =0;
        }
    }
}

/* 
 * This should be called with a valid freq point that was
 * obtained via bulverde_validate_speed
 */
void bulverde_set_freq(unsigned int CLKCFGValue)
{
	unsigned long flags;
	unsigned int unused;
	volatile int v;
	/* NOTE: IRQs are already turned off in dpm idle, so it is not
	   necessary to do it here. */
	local_irq_save(flags);

	/*
	 * force a tlb fault to get the mapping into the tlb 
	 * (otherwise this will occur below when the sdram is turned off and
	 * something-bad(tm) will happen)
	 */
	v = *(volatile unsigned long *)ramstart;
	*(volatile unsigned long *)ramstart = v;

	MST_LEDDAT1 = CLKCFGValue;
	
	__asm__ __volatile__(" \n\
		ldr	r4, [%1]			@load MDREFR \n\
		mcr	p14, 0, %2, c6, c0, 0		@ set CCLKCFG[FCS] \n\
		ldr	r5, =0xe3dfefff	\n\
		and	r4, r4, r5	\n\
		str	r4,  [%1]			@restore \n\
		"
		: "=&r" (unused)
		: "r" (&MDREFR), "r" (CLKCFGValue), "r" (ramstart)
		: "r4", "r5");

	MST_LEDDAT1 = 0x0002;

	/* NOTE: if we don't turn off IRQs up top, there is no point
	   to restoring them here. */
	local_irq_restore(flags);
	
	/* spit out some info about what happened */
	bulverde_freq_debug_info();
}

extern void bulverde_get_current_info(struct dpm_md_opt *);

static void bulverde_freq_debug_info(void)
{
	unsigned int sysbus, run, t, turbo, mem, m=1;
	struct dpm_md_opt opt;

	bulverde_get_current_info(&opt);

	run = 13000 * opt.l;
	turbo = (13000 * opt.l * opt.n) >> 1;
	sysbus = (opt.b)?run:(run/2);
	t = opt.regs.clkcfg & 0x1; 

	/* If CCCR[A] is on */
	if (opt.cccra) {
		mem = sysbus;
	}
	else {
		/* If A=0 */
		/* m initialized to 1 (for l=2-10) */
		if (opt.l > 10) m = 2; /* for l=11-20 */
		if (opt.l > 20) m = 4; /* for l=21-31 */
		mem = run / m;
	}
}

int	bulverde_get_freq(void)
{
	unsigned int freq, n,l, ccsr;

	ccsr =CCSR;

	l = ccsr & CCCR_L_MASK;      /* Get L */
	n = (ccsr & CCCR_N_MASK)>>7; /* Get 2N */
	
	if (n < 2)
		n=2;

	/* Shift to divide by 2 because N is really 2N */
	freq = (13000 * l * n) >> 1;	/*	in kHz*/

	return freq;
}

unsigned int  bulverde_read_clkcfg(void)
{
	unsigned int	value=0;
	unsigned int	un_used;

	__asm__ __volatile__("mrc	p14, 0, %1, c6, c0, 0" :"=&r" (un_used) :"r" (value));

	return value;
}

static int bulverde_init_freqs( void)
{
	int cpu_ver; 

	asm volatile("mrc%? p15, 0, %0, c0, c0" : "=r" (cpu_ver));

	/*	
	 *	Bulverde 	A0: 	0x69054110, 
	 *				A1 : 	0x69054111	
	 */
	if( (cpu_ver&0x0000f000)>>12==4 && (cpu_ver & 0xffff0000)>>16==0x6905 ) 
	/*	It is a xscale core bulverde chip.	*/
		return 1;
	return 0;
}

int bulverde_clk_init(void)
{
	unsigned int freq;

	/*
	 * In order to turn the sdram back on (see below) we need to
	 * r/w the sdram.  We need to do this without the cache and
	 * write buffer in the way.  So, we temporarily ioremap the
	 * first page of sdram as uncached i/o memory and use the
	 * aliased address
	 */

	/* map the first page of sdram to an uncached virtual page */
	ramstart = (int *)ioremap(PHYS_OFFSET, 4096);

	freq = bulverde_get_freq();	/*	in kHz*/
	printk(KERN_INFO "Init freq: %dkHz.\n", freq);

	bulverde_initialize_freq_matrix();

	if (bulverde_init_freqs())	{
		printk(KERN_INFO "CPU frequency change initialized.\n");
	}
	return 0;
}

void
bulverde_freq_cleanup(void)
{
	/* unmap the page we used */
	iounmap((void *)ramstart);
}	
