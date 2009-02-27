/*
 * linux/arch/arm/mach-pxa/bulverde_voltage.c
 *
 * Bulverde voltage change driver.
 *
 * Author: <source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 */
/*
 * Copyright (C) 2005 Motorola Inc.
 *
 * 2005-May-11  copy from arch/arm/mach-pxa/bulverde_voltage.c and porting to EZX platform,  Zhuang Xiaofan
 * 2005-Nov-11  add Montavista 3.1 patch,  Zhuang Xiaofan
 *
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <asm/hardware.h>

/* For ioremap */
#include <asm/io.h>

#include "bulverde_voltage.h"

/*
 *  Transfer desired mv to required DAC value.
 *  Vcore = 1.3v - ( 712uv * DACIn )
 */
struct MvDAC {
	unsigned int    mv;
	unsigned int    DACIn;
};

static struct MvDAC mvDACtable[] = {
	{1425,    0 },
	{1400,   69 },
	{1300,  248 },
	{1200,  428 },
	{1100,  601 },
	{1000,  777 },
	{ 950,  872 },
	{ 868, 1010 },
	{ 861, 0xFFFFFFFF },
};
unsigned int bulverde_validate_voltage(unsigned int mv)
{
	/*
	 *	Just to check whether user specified mv
	 *	can be set to the CPU.
	 */
	if( (mv >= BLVD_MIN_VOL) && (mv <= BLVD_MAX_VOL))
	    	return mv;
	else
		return 0;	
}

/*
 *	According to bulverde's manual, set the core's voltage.
 */
void bulverde_set_voltage(unsigned int mv)
{
	vm_setvoltage(mv2DAC(mv));
}

/*
 * Prepare for a voltage change, possibly coupled with a frequency
 * change
 */
static void  dpm_power_change_cmd(unsigned int DACValue, int coupled);

void bulverde_prep_set_voltage(unsigned int mv)
{
	dpm_power_change_cmd(mv2DAC(mv), 1 /* coupled */);
}

unsigned int mv2DAC(unsigned int mv)
{
	int i, num;

	if (mvDACtable[0].mv <= mv) { /* Max or bigger */
		/* Return the first one */
		return mvDACtable[0].DACIn;
	}

	num = sizeof(mvDACtable) / sizeof(struct MvDAC);
	
	if (mvDACtable[num-1].mv >=  mv) { /* Min or smaller */
		/* Return the last one */
		return mvDACtable[num-1].DACIn;
	}
	
	/* The biggest and smallest value cases are covered, now the
	   loop may skip those */
	for(i=1; i<=(num-1); i++) {
		if ((mvDACtable[i].mv >= mv) && (mvDACtable[i+1].mv < mv)) {
			return mvDACtable[i].DACIn;
		}
	}

	/* Should never get here */
	return 0;
}


/*
 *	Functionality: Initialize PWR I2C. 
 *	Argument:      None
 *	Return:        void
*/
int bulverde_vcs_init(void)
{
	CKEN |= 0x1 << 15;
	CKEN |= 0x1 << 14;
	PCFR	= 0x60;

	printk(KERN_INFO "CPU voltage change initialized.\n");

	return 0;
}

void bulverde_change_voltage(void)
{
	unsigned long flags;
	volatile int *ramstart;
	unsigned int unused;

	/* map the first page of sdram to an uncached virtual page */
	ramstart = (int *)ioremap(PHYS_OFFSET, 4096);

	local_irq_save(flags);

	__asm__ __volatile__("\n\
    @ BV B0 WORKAROUND - Core hangs on voltage change at different\n\
    @ alignments and at different core clock frequencies\n\
    @ To ensure that no external fetches occur, we want to store the next\n\
    @ several instructions that occur after the voltage change inside\n\
    @ the cache. The load dependency stall near the retry label ensures \n\
    @ that any outstanding instruction cacheline loads are complete before \n\
    @ the mcr instruction is executed on the 2nd pass. This procedure \n\
    @ ensures us that the internal bus will not be busy. \n\
					\n\
    b	    2f				\n\
    nop					\n\
    .align  5				\n\
2:					\n\
    ldr     r0, [%1]			@ APB register read and compare \n\
    cmp     r0, #0			@ fence for pending slow apb reads \n\
					\n\
    mov     r0, #8			@ VC bit for PWRMODE \n\
    movs    r1, #1			@ don't execute mcr on 1st pass \n\
					\n\
    @ %1 points to uncacheable memory to force memory read \n\
					\n\
retry:					\n\
    ldreq   r3, [%2]			@ only stall on the 2nd pass\n\
    cmpeq   r3, #0			@ cmp causes fence on mem transfers\n\
    cmp     r1, #0			@ is this the 2nd pass? \n\
    mcreq   p14, 0, r0, c7, c0, 0	@ write to PWRMODE on 2nd pass only \n\
					\n\
    @ Read VC bit until it is 0, indicates that the VoltageChange is done.\n\
    @ On first pass, we never set the VC bit, so it will be clear already.\n\
					\n\
VoltageChange_loop:			\n\
    mrc     p14, 0, r3, c7, c0, 0	\n\
    tst     r3, #0x8			\n\
    bne     VoltageChange_loop		\n\
					\n\
    subs    r1, r1, #1		@ update conditional execution counter\n\
    beq     retry"

    : "=&r" (unused)
    : "r" (&CCCR), "r" (ramstart)
    : "r0", "r1", "r3" );

	local_irq_restore(flags);
	
	/* unmap the page we used */
	iounmap(ramstart);
}

static void clr_all_sqc(void)
{
    int i = 0;
    for (i = 0; i < 32; i++)
		PCMD(i) &= ~PCMD_SQC;
}

static void clr_all_mbc(void)
{
    int i = 0;
    for (i = 0; i < 32; i++)
		PCMD(i) &= ~PCMD_MBC;
}

static void clr_all_dce(void)
{
    int i = 0;
    for (i = 0; i < 32; i++)
		PCMD(i) &= ~PCMD_DCE;
}

static void set_mbc_bit(int ReadPointer, int NumOfBytes)
{
    PCMD0 |= PCMD_MBC;
    PCMD1 |= PCMD_MBC;
}

static void set_lc_bit(int ReadPointer, int NumOfBytes)
{
    PCMD0 |= PCMD_LC;
    PCMD1 |= PCMD_LC;
    PCMD2 |= PCMD_LC;
}

static void set_cmd_data(unsigned char *DataArray, int StartPoint, int size)
{
    PCMD0 &= 0xFFFFFF00;
    PCMD0 |= DataArray[0];
    PCMD1 &= 0xFFFFFF00;
    PCMD1 |= DataArray[1];
    PCMD2 &= 0xFFFFFF00;
    PCMD2 |= DataArray[2];
}

/* coupled indicates that this VCS is to be coupled with a FCS */
static void  dpm_power_change_cmd(unsigned int DACValue, int coupled)
{
    unsigned char dataArray[3];

    dataArray[0] = 0;				/*	Command 0	*/
    dataArray[1] = (DACValue & 0x000000FF);	/* 	data LSB	*/
    dataArray[2] = (DACValue & 0x0000FF00) >> 8;/*	data MSB	*/

    PVCR = 0;

    PCFR &= ~PCFR_FVC;
    PVCR &= 0xFFFFF07F;		/*	no delay is necessary	*/
    PVCR &= 0xFFFFFF80;		/*  clear slave address		*/
    PVCR |= 0x20;		/*	set slave address	*/

    PVCR &= 0xFE0FFFFF;		/*	clear read pointer 0	*/
    PVCR |= 0;

    /*	DCE and SQC are not necessary for single command */
    clr_all_sqc();	
    clr_all_dce();

    clr_all_mbc();
    set_mbc_bit(0, 2);

    /*	indicate the last byte of this command is holded in this register	*/
    PCMD2 &= ~PCMD_MBC;

    /*	indicate this is the first command and last command also	*/
    set_lc_bit(0, 3);

    /*	programming the command data bit 	*/
    set_cmd_data(dataArray, 0, 2);

    /* Enable Power I2C */
    PCFR |= PCFR_PI2CEN;

    if (coupled) {
	    /* Enable Power I2C and FVC */
	    PCFR |= PCFR_FVC;
    }
}

void vm_setvoltage(unsigned int  DACValue)
{
	dpm_power_change_cmd(DACValue, 0 /* not-coupled */);
	/* Execute voltage change sequence	*/
	bulverde_change_voltage(); /* set VC on the PWRMODE on CP14 */
}	
