/*
 * linux/arch/arm/mach-ezx/dvfs.c
 *
 * Copyright (C) 2003 Motorola Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author:     Zhuang Xiaofan
 * Created:    Nov 25, 2003
 * Support for the Motorola Ezx A780 Development Platform.
 *
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/apm_bios.h>
#include <linux/power_ic.h>
#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/semaphore.h>

/* counter for disabling ipm and deep idle */
static int disable_ipm = 0;

#ifdef CONFIG_IPM_DEEPIDLE 
extern void deepidle_enable(void);
extern void deepidle_disable(void);
#endif

static DECLARE_MUTEX(ipm_scale_sem);

#define SDRAM_REFINTV	64
#define BASE_CLK_KHZ    13000
#define SDRAM_CLK       104000
#define FLASH_CLK       52000

#define FSCALE_CPUFREQ		0
#define FSCALE_ENTERING_13M	(1<<0)
#define FSCALE_EXITING_13M	(1<<1)

static struct ipm_config current_config;
static struct clk_regs conf_regs;

/* for calculate jiffies */
extern unsigned long loops_per_jiffy;
static unsigned int fscale_flag = 0;
static unsigned long saved_loops_per_jiffy = 0;
static unsigned long saved_cpu_freq = 0;

/* jiffies for 13M and non 13M */
static unsigned long lpj_in_13M = 0;
static unsigned long lpj_out_13M = 0;

#define PLL_L_MAX	30
#define PLL_L_MIN	0
#define PLL_2N_MAX	6
#define PLL_2N_MIN	0
#define CORE_VLTG_MAX	2000
#define CORE_VLTG_MIN	0

#define CCCR_RESERVED	0x31fff860
#define CCCR_CPDIS	(1<<31)
#define CLKCFG_FCS	0x2

/* macros to check clock register bit */
#define cccr_l(x)	((x) & 0x1f)
#define cccr_2n(x)	(((x) & 0x380) >> 7)
#define cccr_a(x)	(((x)>>25) & 0x1)
#define cccr_cpdis(x)	(((x)>>31) & 0x1)
#define cccr_lcd26(x)	(((x)>>27) & 0x1)

#define clkcfg_t(x)	((x) & 0x1)
#define clkcfg_ht(x)	(((x)>>2) & 0x1)
#define clkcfg_b(x)	(((x)>>3) & 0x1)

static unsigned int calc_memc_reg(unsigned int membus_clk);
extern unsigned int get_clk_frequency_khz( int info);
extern unsigned int get_memclk_frequency_10khz(void);
extern int change_pclk(struct clk_regs *regs, int speedup);

unsigned long compute_lpj(unsigned long ref, unsigned long div, unsigned long mult)
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


#define CCCR_CPDIS_BIT_ON              (1 << 31)
#define CCCR_PPDIS_BIT_ON              (1 << 30)
#define CCCR_PLL_EARLY_EN_BIT_ON       (1 << 26)
#define CCCR_LCD_26_BIT_ON             (1 << 27)

int enter_13M_quickly(void)
{
	int cccr, clkcfg;
	unsigned int tmp;
	unsigned long flags;

	if (CCCR & (1 << 31))
               return 1;

	if (!lpj_in_13M) {
		lpj_in_13M = compute_lpj(loops_per_jiffy, get_clk_frequency_khz(0), BASE_CLK_KHZ);
	}

	save_flags_cli(flags);

	calc_memc_reg(BASE_CLK_KHZ);
	ARB_CNTRL=0x01000f00;

	cccr = CCCR;
	cccr |= CCCR_CPDIS_BIT_ON;
	cccr &= ~CCCR_PPDIS_BIT_ON;
	cccr |= CCCR_LCD_26_BIT_ON;
	CCCR = cccr;

        asm("mrc\tp14, 0, %0, c6, c0, 0" :"=r"(clkcfg));
        clkcfg &= 0xf;
        clkcfg |= (1 << 1);
        asm("mcr\tp14, 0, %0, c6, c0, 0" ::"r"(clkcfg));

        cccr = CCCR;
        do {
                asm("mrc\tp14, 0, %0, c6, c0, 0" :"=r"(tmp));
        } while ((tmp & 0xf) != clkcfg);

#if 0
#ifdef FCS_WITH_MEM
	/* change MDREFR, wait 1.6167us for silicon bug workaround */
        MDREFR |= 0xfff;
        tmp=MDREFR;

        tmp = OSCR;
        while (OSCR - tmp <= 6);

        MDREFR = conf_regs.mdrefr;
        tmp=MDREFR;
#endif
#endif
	lpj_out_13M = loops_per_jiffy;
	loops_per_jiffy = lpj_in_13M;
	restore_flags(flags);
	return 0;
}


void exit_13M_quickly(int mode)
{
	int cccr, ccsr, clkcfg;
	unsigned long oscr;
	unsigned long flags;
	unsigned int tmp;

	if (mode)
		return;

	save_flags_cli(flags);

	cccr = CCCR;
	cccr |= (CCCR_CPDIS_BIT_ON | CCCR_PLL_EARLY_EN_BIT_ON);
	CCCR = cccr;

	/* Step 2 */
	cccr = CCCR;
	if ((cccr & 0x84000000) != 0x84000000) { /* CPDIS, pll_early_en */
		printk("DPM: Warning: CPDIS or PLL_EARLY_EN not on\n");
	}
	
	oscr = OSCR;
	while (OSCR - oscr < 390);	/* delay 120 us */

	/* Step 3 */
	{
		/* Note: the point of this is to "wait" for the lock
		   bits to be set; the Yellow Book says this may take
		   a while, but observation indicates that it is
		   instantaneous */

		long volatile int i=0;

		for (i=0; i<999999 /* arbitrary big value to prevent
					looping forever */; i++) {
			ccsr = CCSR;
			if ((ccsr & 0x30000000) == 0x30000000) {
				/* They are both on */
				break;
			}
		}		
	}

	/* Step 4: NOP */

	/* Step 5 */
	/* Now clear the PLL disable bits */
	/* But leave EARLY_EN on; it will be cleared by the frequency change */
	cccr &= ~(CCCR_CPDIS_BIT_ON | CCCR_PPDIS_BIT_ON); /* Not ON */
	cccr |= CCCR_PLL_EARLY_EN_BIT_ON;
	CCCR = cccr;

#if 0
#ifdef FCS_WITH_MEM
        calc_memc_reg(get_memclk_frequency_10khz()*10);
        MDREFR = conf_regs.mdrefr;
#endif
#endif
        asm("mrc\tp14, 0, %0, c6, c0, 0" :"=r"(clkcfg));
        clkcfg &= 0xf;
        clkcfg |= (1 << 1);
        asm("mcr\tp14, 0, %0, c6, c0, 0" ::"r"(clkcfg));

        cccr = CCCR;
        do {
                asm("mrc\tp14, 0, %0, c6, c0, 0" :"=r"(tmp));
        } while ((tmp & 0xf) != clkcfg);

	ARB_CNTRL = 0x237;
	loops_per_jiffy = lpj_out_13M;
	restore_flags(flags);
}


static unsigned int calc_memc_reg(unsigned int membus_clk)
{
        unsigned int rows, dri;
        unsigned int mdrefr, mdcnfg;

        mdrefr = MDREFR & ~MDREFR_RESERVED & ~0xfff;
        mdcnfg = MDCNFG;

        rows = 2 << (10 + ((mdcnfg & 0x60) >> 5));
        dri = (SDRAM_REFINTV * membus_clk / rows - 31) / 32;

        if (membus_clk > 2*SDRAM_CLK)
                return -1;
        else if (membus_clk <= SDRAM_CLK)
                mdrefr &= ~(MDREFR_K2DB2 | MDREFR_K1DB2);
        else
                mdrefr |= (MDREFR_K2DB2 | MDREFR_K1DB2);

        if (membus_clk > 4*FLASH_CLK)
                return -1;
        else if (membus_clk <= FLASH_CLK)
                mdrefr &= ~(MDREFR_K0DB4 | MDREFR_K0DB2);
        else if (membus_clk <= 2*FLASH_CLK)
                mdrefr = (mdrefr & ~MDREFR_K0DB4) | MDREFR_K0DB2;
        else
                mdrefr = (mdrefr & ~MDREFR_K0DB2) | MDREFR_K0DB4;

        conf_regs.mdrefr = mdrefr | dri;

        return 0;
}


/* Here just do simply check */
static int validate_ipm_config(struct ipm_config *conf)
{
	unsigned int l, n2, v, a, lcd_26;
	unsigned int b, t=0, ht=0;
	unsigned int R;

	v  = conf->core_vltg;
	if ((v > CORE_VLTG_MAX) || (v < CORE_VLTG_MIN)) {
		printk(KERN_WARNING
		       "Voltage: core (%d mv) out of range, V=0-2000 \n", v);
		return -EINVAL;
	}

        if (calc_memc_reg(conf->mem_bus_freq)) {
                printk(KERN_WARNING "memory bus clock: cannot fit sdram/falsh clock\n");
                return -EINVAL;
        }

	/* 13M mode */
	if (conf->core_freq == BASE_CLK_KHZ) {

		if (conf->lcd_freq == 2*BASE_CLK_KHZ)
			lcd_26 = 1;
		else 
			lcd_26 = 0;

		fscale_flag = FSCALE_ENTERING_13M;
		conf_regs.cccr = (CCCR & ~CCCR_RESERVED) | (lcd_26<<27) | CCCR_CPDIS;
		__asm__ __volatile__( "mrc\tp14, 0, %0, c6, c0, 0" :: "r" (conf_regs.clkcfg) );
		conf_regs.clkcfg = (conf_regs.clkcfg & 0xf) | CLKCFG_FCS;	

		return 0;
	}

	b = (conf->fast_bus_mode ? 1 : 0);
	if (b)
		R = conf->sys_bus_freq;
	else
		R = conf->sys_bus_freq*2;

	l = R/BASE_CLK_KHZ;
	n2 = conf->turbo_ratio;
	if ((l > PLL_L_MAX) || (l < PLL_L_MIN)
		|| (n2 > PLL_2N_MAX) || (n2 < PLL_2N_MIN)) {
		/* range checking */
		printk(KERN_WARNING
		       "Frequency: L/2N (%d/%d) out of range, L=0-30, 2N=0-6 \n",
		       l, n2);
		return -EINVAL;
	}

	a = (conf->sys_bus_freq == conf->mem_bus_freq);
	
	if (conf->core_freq == (R*n2/2)) {
		t = 1;
	}
	else if (conf->core_freq == (R*n2/4)) {
		ht = 1;
	}

	conf_regs.cccr = (a<<25 | n2<<7 | l);
	conf_regs.clkcfg = (b<<3 | ht<<2 | CLKCFG_FCS | t);	

	return 0;
}

/* Frequency scaling function */
static int ipm_scale_freq(struct ipm_config *conf)
{
	unsigned long new_cpu_freq = conf->core_freq;
	int sys_bus_change = 0;

	/* keep a baseline loops_per_jiffy/cpu-freq ratio */
	if (!saved_loops_per_jiffy) {
		get_ipm_config(&current_config);
		saved_loops_per_jiffy = loops_per_jiffy;
		saved_cpu_freq = current_config.core_freq;
	}

	if (fscale_flag & FSCALE_ENTERING_13M) {
		enter_13M_quickly();
	} else if (fscale_flag & FSCALE_EXITING_13M) {
		exit_13M_quickly(0);
	} else {	
		if(conf->sys_bus_freq == current_config.sys_bus_freq){
				sys_bus_change = 0;
		}else if (conf->sys_bus_freq > current_config.sys_bus_freq){
				sys_bus_change = 1;
		}else{
				sys_bus_change = -1;
		}
	
	     if (change_pclk(&conf_regs, sys_bus_change))
			return -EFAULT;
		loops_per_jiffy = compute_lpj(saved_loops_per_jiffy, saved_cpu_freq, new_cpu_freq);
	}	

	if (fscale_flag & FSCALE_ENTERING_13M)
		fscale_flag = FSCALE_EXITING_13M;
	else
		fscale_flag = FSCALE_CPUFREQ;
	return 0;
}

/* Voltage scaling function */
static int ipm_scale_vltg(struct ipm_config *conf)
{
	return power_ic_set_core_voltage(conf->core_vltg);
}

/* Get current core voltage */
int get_core_voltage(void)
{
	return current_config.core_vltg;
}

/* Get ipm config according to current system state */
int get_ipm_config(struct ipm_config  *ret_conf)
{
	struct ipm_config conf;
	unsigned long ccsr, cccr, turbo, b, ht;
	unsigned int l, L, m, M, n2, N, S, k, cccra;

	memset(&conf, 0, sizeof(struct ipm_config));

	ccsr = CCSR;
	cccr = CCCR;

	cccra = cccr_a(cccr);

	/* Read clkcfg register: it has turbo, b, half-turbo (and f) */
	__asm__ __volatile__( "mrc\tp14, 0, %0, c6, c0, 0" : "=r" (turbo) );
	b = clkcfg_b(turbo);
	ht = clkcfg_ht(turbo);	/* By now assume not using */

	l  =  cccr_l(ccsr);
	n2 =  cccr_2n(ccsr);
	if (l == 31) {
		/* The calculation from the Yellow Book is incorrect:
		   it says M=4 for L=21-30 (which is easy to calculate
		   by subtracting 1 and then dividing by 10, but not
		   with 31, so we'll do it manually */
		m = 1 << 2;
	}
	else {
		m  =  1<<( (l-1)/10 );
	}

	if (l >= 2 && l <= 7) {
		k = 1;
	}
	else if (l >=8 && l <=16) {
		k = 2;
	} else
		k = 4;

	L = l * BASE_CLK_KHZ;
	N = (n2 * L) / 2;
	S = (b) ? L : L/2;
	if (cccra == 0)
		M = L/m;
	else
		M = (b) ? L : L/2;

	conf.fast_bus_mode = b;
	conf.turbo_ratio = n2;
	conf.core_freq = (clkcfg_t(turbo)) ? N : L;
	conf.sys_bus_freq = (b) ? L : L/2;
	conf.mem_bus_freq = M;
	conf.lcd_freq = L / k;
	conf.cpu_mode = clkcfg_t(turbo);	

	/* 13M mode */
	if (cccr_cpdis(ccsr)) {
		conf.core_freq = BASE_CLK_KHZ;
		conf.sys_bus_freq = BASE_CLK_KHZ;
		conf.mem_bus_freq = BASE_CLK_KHZ;
		conf.lcd_freq = BASE_CLK_KHZ<<cccr_lcd26(cccr);
	}

	memcpy(ret_conf, &conf,sizeof(struct ipm_config));
	return 0;
}
static inline int set_same_op(struct ipm_config *set_conf)
{
	return (set_conf->core_freq == current_config.core_freq);
}

/* Change system state as ipm config decribed */
int set_ipm_config(struct ipm_config  *set_conf)
{
	down(&ipm_scale_sem);

        if (set_same_op(set_conf)) {
                up(&ipm_scale_sem);
                return 0;
         }

#ifdef CHK_IPC
	if (ipc_is_active()) {
		up(&ipm_scale_sem);
		return -EFAULT;
	}
#endif
        if (disable_ipm) {
		up(&ipm_scale_sem);
                return -EBUSY;
	}

	if (validate_ipm_config(set_conf)) {
		up(&ipm_scale_sem);
		return -EINVAL;
	}

	/* if set higher frequency */ 
	if (set_conf->core_freq > current_config.core_freq) {
		if (ipm_scale_vltg(set_conf)) {
			up(&ipm_scale_sem);
			return -EFAULT;
		}

		if (ipm_scale_freq(set_conf)) {
			ipm_scale_vltg(&current_config);
			up(&ipm_scale_sem);
			return -EFAULT;
		}
	} else {
		if (ipm_scale_freq(set_conf)) {
			up(&ipm_scale_sem);
			return -EFAULT;
		}

		if (ipm_scale_vltg(set_conf)) {
			ipm_scale_freq(&current_config);
			up(&ipm_scale_sem);
			return -EFAULT;
		}
	}

	current_config = *set_conf;
	up(&ipm_scale_sem);

	return 0;
}

static int set_ipm_cam(void)
{
        int ret;
        struct ipm_config cam_op =
#if defined(CONFIG_ARCH_EZX_HAINAN) || defined(CONFIG_ARCH_EZX_SUMATRA)  
		{312000, 1350, 3, 1, 1, 208000, 208000, 104000, -1};	
#else
                {208000, 1300, 2, 0, 1, 208000, 208000, 104000, -1};
#endif
        if ((ret = set_ipm_config(&cam_op)))
                return ret;

        disable_ipm ++;
#ifdef CONFIG_IPM_DEEPIDLE 
        deepidle_disable();
#endif
        return 0;
}

static void put_ipm_cam(void)
{
        disable_ipm --;
#ifdef CONFIG_IPM_DEEPIDLE 
        deepidle_enable();
#endif
}

int cam_ipm_hook(void)
{
	return set_ipm_cam();
}

void cam_ipm_unhook(void)
{
	put_ipm_cam();
}

int ovl2_ipm_hook(void)
{
	return set_ipm_cam();
}

void ovl2_ipm_unhook(void)
{
	put_ipm_cam();
}
