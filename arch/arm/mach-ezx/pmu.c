/*
 * This function provides the implementation of the access functions to 
 * the Performance Monitoring Unit on all CPUs based on the XScale core.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 *
 * Copyright (C) 2003 MontaVista Softwrae, Inc.
 * Maintainer: Deepak Saxena <dsaxena@mvista.com>
 *
 * Copyright (C) 2004 Intel Corporation.
 * Add Bulverde support by Cain Yuan <cain.yuan@intel.com>
 *
 * Copyright (C) 2005 Motorola Inc.
 * move PMNC clear operation, it's more precise to freeze the counters before read 	Zhuang xiaofan <w19962@motorola.com>
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <asm/atomic.h>
#include <asm/xscale-pmu.h>
#include <asm/arch/irqs.h>

#define	PMU_ENABLE	0x001	/* Enable counters */
#define PMN_RESET	0x002	/* Reset event counters */
#define	CCNT_RESET	0x004	/* Reset clock counter */
#define	PMU_RESET	CCNT_RESET | PMN_RESET
#define	INT_ENABLE  0x070   /* Interrupt enable */

#define	PMN3_OVERFLOW	0x10	/* Perfromance counter 0 overflow */
#define PMN2_OVERFLOW	0x08	/* Performance counter 1 overflow */
#define PMN1_OVERFLOW	0x04	/* Performance counter 1 overflow */
#define PMN0_OVERFLOW	0x02	/* Performance counter 1 overflow */
#define CCNT_OVERFLOW	0x01	/* Clock counter overflow */

static atomic_t usage = ATOMIC_INIT(0);
static unsigned long id = 0;
static u32 pmnc;
static u32 evtsel;
struct pmu_results results;
static void pmu_irq_handler(int, void *, struct pt_regs *);

int pmu_claim(void)
{
	int err = 0;

	if(atomic_read(&usage))
		return -EBUSY;

	err = request_irq(IRQ_PMU, pmu_irq_handler, SA_INTERRUPT, NULL, (void *)&results);
	if(err < 0) {
		printk(KERN_ERR "unable to request IRQ %d for 80200 PMU: %d\n", IRQ_PMU, err);
		return err;
	}
	atomic_inc(&usage);
	pmnc = 0;
	/*	Clear PMNC */
	asm ("mcr p14, 0, %0, c0, c1, 0" : : "r" (pmnc));
	return ++id;
}

int pmu_release(int claim_id)
{
	if(!atomic_read(&usage))
		return 0;

	if(claim_id != id)
		return -EPERM;

	free_irq(IRQ_PMU, (void *)&results);
	atomic_dec(&usage);
	return 0;
}

#ifdef CONFIG_CPU_BULVERDE
int pmu_start(u32 pmn0, u32 pmn1, u32 pmn2, u32 pmn3)
{
	u32 inten = 0x1F;

	memset( &results, 0, sizeof(results));
	evtsel = (pmn3 <<24) | (pmn2 <<16) | (pmn1 <<8) | pmn0;
	pmnc |= PMU_ENABLE | PMU_RESET;

	/*	Event select */
	asm ("mcr p14, 0, %0, c8, c1, 0" : : "r" (evtsel));
	/*	Enable interrupt */
	asm ("mcr p14, 0, %0, c4, c1, 0" : : "r" (inten));
	/*	PMU reset and enable */
	asm ("mcr p14, 0, %0, c0, c1, 0" : : "r" (pmnc));

	return 0;
}
#else
int pmu_start(u32 pmn0, u32 pmn1)
{
	memset( &results, 0, sizeof(results));
	evtsel = (pmn1 << 20) | (pmn0 << 12);
	pmnc |= PMU_ENABLE | PMU_RESET | INT_ENABLE;

	/*	Event select */
	asm ("mcr p14, 0, %0, c8, c1, 0" : : "r" (evtsel));
	/*	Reset PMU and enable it, interrupt enabled */
	asm ("mcr p14, 0, %0, c0, c1, 0" : : "r" (pmnc));

	return 0;
}
#endif

int pmu_stop(struct pmu_results *results)
{
	u32 ccnt, pmn0, pmn1, pmn2, pmn3;

	if(!pmnc)
		return -ENOSYS;

	/*	Clear PMNC */
	pmnc = 0;
	asm ("mcr p14, 0, %0, c0, c1, 0" : : "r" (pmnc));

	/*	Read CCNT */
	asm ("mrc p14, 0, %0, c1, c1, 0" : "=r" (ccnt));
	/*	Read PMN0/1/2/3 */
	asm ("mrc p14, 0, %0, c0, c2, 0" : "=r" (pmn0));
	asm ("mrc p14, 0, %0, c1, c2, 0" : "=r" (pmn1));
	asm ("mrc p14, 0, %0, c2, c2, 0" : "=r" (pmn2));
	asm ("mrc p14, 0, %0, c3, c2, 0" : "=r" (pmn3));

	results->ccnt = ccnt;
	results->pmn0 = pmn0;
	results->pmn1 = pmn1;
	results->pmn2 = pmn2;
	results->pmn3 = pmn3;

	return 0;
}

static void pmu_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	struct pmu_results *results = (struct pmu_results *)dev_id;
	unsigned int flag;

	/* Read FLAG and PMNC status */
	asm ("mrc p14, 0, %0, c5, c1, 0" : "=r" (flag));
	asm ("mrc p14, 0, %0, c0, c1, 0" : "=r" (pmnc));

	if(pmnc & PMN0_OVERFLOW) {
		results->pmn0_of++;
	}
	if(pmnc & PMN1_OVERFLOW) {
		results->pmn1_of++;
	}
	if(pmnc & PMN2_OVERFLOW) {	
		results->pmn2_of++;
	}
	if(pmnc & PMN3_OVERFLOW) {
		results->pmn3_of++;
	}
	if(pmnc & CCNT_OVERFLOW) {
		results->ccnt_of++;
	}
	/*	Clear them */
	asm ("mcr p14, 0, %0, c5, c1, 0" : : "r" (flag));
	asm ("mcr p14, 0, %0, c0, c1, 0" : : "r" (pmnc));
}

EXPORT_SYMBOL(pmu_claim);
EXPORT_SYMBOL(pmu_release);
EXPORT_SYMBOL(pmu_start);
EXPORT_SYMBOL(pmu_stop);

