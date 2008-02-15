/*
 * OMAP1610 Power Management Routines
 *
 * Original code for the SA11x0:
 * Copyright (c) 2001 Cliff Brake <cbrake@accelent.com>
 *
 * Modified for the PXA250 by Nicolas Pitre:
 * Copyright (c) 2002 Monta Vista Software, Inc.
 *
 * Modified for the OMAP1510 by David Singleton:
 * Copyright (c) 2002 Monta Vista Software, Inc.
 *  
 * Modified for the OMAP1610 by Alexandr Shkurenkov:
 * Copyright (c) 2003 Monta Vista Software, Inc.
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
#include <linux/proc_fs.h>   
#include <linux/sysctl.h>
#include <linux/errno.h>
#include <linux/serial_reg.h>
#include <linux/delay.h>
#include <linux/trace.h>
#include <linux/pm.h>
#include <linux/rtc.h>

#include <asm/memory.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/serial.h>
#include <asm/arch/hardware.h>
#include <asm/arch/pm.h>

static unsigned short arm_sleep_save[ARM_SLEEP_SAVE_SIZE];
static unsigned short ulpd_sleep_save[ULPD_SLEEP_SAVE_SIZE];
static unsigned int mpui_sleep_save[MPUI_SLEEP_SAVE_SIZE];

/*
 * Let's power down on idle, but only if we are really
 * idle, because once we start down the path of
 * going idle we continue to do idle even if we get
 * a clock tick interrupt . . 
 */
void omap1610_pm_idle(void)
{
	int (*func_ptr)(void);
	unsigned short mask16 = 0;

	/*
	 * If the DSP is being used let's just idle the CPU, the overhead
	 * to wake up from Big Sleep is big, milliseconds versus micro
	 * seconds for wait for interrupt.
	 */

	cli();
	clf();
	if (current->need_resched) {
		stf();
		sti();
		return;
	}
	mask16 = *ARM_SYSST;
	stf();
	sti();
	if ((mask16 & DSP_IDLE) == 0) {
		TRACE_PROCESS(TRACE_EV_PROCESS_IDLE_BEGIN, 0, 0);
		__asm__ volatile ("mcr	p15, 0, r0, c7, c0, 4");
	} else {
		TRACE_PROCESS(TRACE_EV_PROCESS_IDLE_BEGIN, 0, 0);
		func_ptr = (void *)(OMAP1610_SRAM_IDLE_SUSPEND);
		func_ptr();
	}

	return;
}

void omap_pm_suspend(void)
{
	unsigned short mask16 = 0;
	unsigned long mask32 = 0;
	int (*func_ptr)(void);
	unsigned long *addr;
	int i;
#ifdef CONFIG_OMAP_RTC
	struct timeval t;
	struct rtc_time rt;
	unsigned long rtc_offset;
	unsigned long mkt;
	extern void get_rtc_time(struct rtc_time *);

	/*
	 * lets figure out the offset of the RTC to our system time.
	 */
	do_gettimeofday(&t);
	get_rtc_time(&rt);
	mkt = mktime(rt.tm_year + 1900, rt.tm_mon + 1, rt.tm_mday, rt.tm_hour,
	    rt.tm_min, rt.tm_sec);
	rtc_offset = t.tv_sec = mkt;
#endif
	/*
	 * Step 0: turn off interrupts
	 */

	cli();
	clf();
	MPUI_SAVE(IRQ_MIR1);
	MPUI_SAVE(IRQ_MIR2_0);
	MPUI_SAVE(IRQ_MIR2_1);
	MPUI_SAVE(IRQ_MIR2_2);
	MPUI_SAVE(IRQ_MIR2_3);

	/*
	 * The omap is a strange/beautiful device.  The caches, memory
	 * and regsigter state are preserved across power saves.
	 * We have to save and restore very little register state to
	 * idle the omap.
	 */

	/*
	 * Save MPUI control registers for wake up.
	 */

	MPUI_SAVE(MPUI_CTRL_REG);
	MPUI_SAVE(MPUI_DSP_BOOT_CONFIG);
	MPUI_SAVE(MPUI_DSP_API_CONFIG);
	MPUI_SAVE(PM_EMIFS_CONFIG_REG);
	MPUI_SAVE(PM_EMIFF_SDRAM_CONFIG);

	/*
	 * Sleep algorithm:
	 *
	 * if (DSP_IS_NOT_RUNNING) {
	 * 	a) Enable DSP/MPUI clocks
	 *	b) setup default MPUI config
	 *	c) set boot mode for DSP 0x2
	 *	d) disable SARAM access by ARM
	 *	e) release reset on DSP (boot DSP)
	 *	f) wait for DSP to idle bit 6 of ARM_SYSST = DSP_IDLE (0x0020)
	 * } 
	 * if (DSP_IS_IDLE) {
	 *	if (LCD_IS_ON) {
	 *		leave LCD clock enabled EN_LCDCK in ARM_IDLECT2
	 *		ARM_IDLE_REQUEST &= ~EN_LCDCK
	 *	}
	 * }
	 * call omap1610_cpu_suspend which will:
	 * put ARM/LCD/TC/whatever clocks into the state you want them
	 * and then put SDRAM into self refresh and idle the ARM and its 
	 * unused peripherals into sleep using the ARM_IDLE bits.
	 *	
	 * Save ARM control registers
	 */

	ARM_SAVE(ARM_CKCTL);
	ARM_SAVE(ARM_IDLECT1);
	ARM_SAVE(ARM_IDLECT2);
	ARM_SAVE(ARM_EWUPCT);
	ARM_SAVE(ARM_RSTCT1);
	ARM_SAVE(ARM_RSTCT2);
	ARM_SAVE(ARM_SYSST);
	ULPD_SAVE(ULPD_CLOCK_CTRL_REG);
	ULPD_SAVE(ULPD_STATUS_REQ_REG);

	mask16 = *ARM_SYSST;
	if (mask16 & DSP_IDLE) {

		mask16 = *ARM_SYSST;
		/*
		 * DSP is not running, do steps a - f.
		 */
		mask16 = (1<<EN_DSPCK);				/* a */
		*ARM_CKCTL = mask16;
		mask16 = (1<<EN_APICK);
		*ARM_IDLECT2 = mask16;

		mask16 = 0x4;
		*ARM_RSTCT1 = mask16;				/* b */
		mask32 = DEFAULT_MPUI_CONFIG;
		*MPUI_CTRL_REG = mask32;

		mask32 = 0x0002;
		*MPUI_DSP_BOOT_CONFIG = mask32;			/* c*/

		mask32 = 0x0;
		*MPUI_DSP_API_CONFIG = mask32;			/* d */

		mask16 = DSP_RESET;
		*ARM_RSTCT1 = mask16;				/* e */
		mask16 = DSP_ENABLE;
		*ARM_RSTCT1 = mask16;

		mdelay(DSP_IDLE_DELAY);
		for (i = 0; i < SUFFICIENT_DSP_RESET_TIME; i++) {
			mask16 = (unsigned short)*ARM_SYSST;  /* f */
			if (mask16 & DSP_IDLE) {
				break;
			}
		}

		/*
		 * enable DSP clock to be shut off in idle
		 */

		mask16 = *ARM_CKCTL;
		mask16 |= ~(DSP_RESET);
		*ARM_CKCTL = mask16;
	}

	/*
	 * enable arm xor clock amd MPUI clock
         * and release peripheral from reset by
	 * writing 1 to PER_EN bit in ARM_RSTCT2, this is required
	 * for COM UART configuration to use UART2 to wake up.
	 */
	 
	mask16 = *ARM_IDLECT2;
	mask16 |= (ENABLE_XORCLK | ENABLE_APICK);
	*ARM_IDLECT2 = mask16;

	*ARM_RSTCT2 = 0x1;

	mask16 = MODEM_32K_EN;
	*ULPD_CLOCK_CTRL_REG = mask16;

	/*
	 * Turn off all interrupts except UART1
	 * Leave 2nd level cascae enabled, but disable all other
	 * level 1 interrupts.
	 */

	addr = (void *)(OMAP_IH1_BASE + IRQ_MIR);
	mask32 = *addr;
	mask32 = ~1;
	*addr = mask32;

	addr = (void *)(OMAP_IH2_0_BASE + IRQ_MIR);
	mask32 = *addr;
	mask32 = ~(1<<14);
	*addr = mask32;

	addr = (void *)(OMAP_IH2_1_BASE + IRQ_MIR);
	mask32 = *addr;
	mask32 = 0xFFFFFFFF;
	*addr = mask32;

	addr = (void *)(OMAP_IH2_2_BASE + IRQ_MIR);
	mask32 = *addr;
	mask32 = 0xFFFFFFFF;
	*addr = mask32;

	addr = (void *)(OMAP_IH2_3_BASE + IRQ_MIR);
	mask32 = *addr;
	mask32 = 0xFFFFFFFF;
	*addr = mask32;

 	outl(1, OMAP_IH1_BASE + OMAP1610_CONTROL_IRQ); // New IRQ agreement

	/*
	 * set bit 2,3 in the RSTCT1 so we can program the EMIF registers
	 * and reset the DSP
	 */ 
	mask16 = *ARM_RSTCT1;
	mask16 |= 0x6;
	*ARM_RSTCT1 = mask16; 

        /* disable ARM watchdog */
        addr = (void *) OMAP1610_WDT_TIMER_MODE;
        *addr =  0x00F5;
        *addr =  0x00A0;
	/*
	 * Since the omap1610_cpu_suspend routine has been copied to
	 * SRAM, we'll do an indirect procedure call to it and pass the
	 * contents of arm_idlect1 and arm_idlect2 so it can restore
	 * them when it wakes up and it will return to here:
	 */

	func_ptr = (void *)(OMAP1610_SRAM_API_SUSPEND);
        func_ptr();

	/*
	 * restore ARM state, except idlect1/2 which cpu_suspend did
	 */

	ARM_RESTORE(ARM_CKCTL);
	ARM_RESTORE(ARM_EWUPCT);
	ARM_RESTORE(ARM_RSTCT1);
	ARM_RESTORE(ARM_RSTCT2);

	/*
	 * Restore MPUI control registers.
	 */

	MPUI_RESTORE(MPUI_CTRL_REG);
	MPUI_RESTORE(MPUI_DSP_BOOT_CONFIG);
	MPUI_RESTORE(MPUI_DSP_API_CONFIG);
	MPUI_RESTORE(PM_EMIFF_SDRAM_CONFIG);
	MPUI_RESTORE(PM_EMIFS_CONFIG_REG);
	MPUI_RESTORE(IRQ_MIR1);
	MPUI_RESTORE(IRQ_MIR2_0);
	MPUI_RESTORE(IRQ_MIR2_1);
	MPUI_RESTORE(IRQ_MIR2_2);
	MPUI_RESTORE(IRQ_MIR2_3);

#ifdef CONFIG_OMAP_RTC
	/* 
	 * let's reset the system time to the RTC.  NOTE: this
	 * depends on the RTC staying powered up during sleep.
	 * the omap1510 innovator does not have this feature.
	 */
	do_gettimeofday(&t);
	get_rtc_time(&rt);
	mkt = mktime(rt.tm_year + 1900, rt.tm_mon + 1, rt.tm_mday, rt.tm_hour,
	    rt.tm_min, rt.tm_sec);
	rtc_offset = t.tv_sec = mkt;
	xtime.tv_sec = mkt + rtc_offset;
#endif
	sti();
	stf();

	return;
}

#ifdef CONFIG_PROC_FS
static int g_read_completed;
/*
 * Writing to /proc/pm puts the CPU in sleep mode
 */
static ssize_t
omap_pm_write_proc(struct file * file, const char * buf, size_t count,
    loff_t *ppos)
{
	if (!pm_send_all(PM_SUSPEND, (void *)3)) {
		omap_pm_suspend();
		pm_send_all(PM_RESUME, (void *)0);
	}
	return count;
}     

static int omap_pm_read_proc(
	char *page_buffer,
	char **my_first_byte,
	off_t virtual_start,
	int length,
	int *eof,
	void *data)
{
	int my_buffer_offset = 0;
	char * const my_base = page_buffer;

	ARM_SAVE(ARM_CKCTL);
	ARM_SAVE(ARM_IDLECT1);
	ARM_SAVE(ARM_IDLECT2);
	ARM_SAVE(ARM_EWUPCT);
	ARM_SAVE(ARM_RSTCT1);
	ARM_SAVE(ARM_RSTCT2);
	ARM_SAVE(ARM_SYSST);

	ULPD_SAVE(ULPD_IT_STATUS_REG);
	ULPD_SAVE(ULPD_CLOCK_CTRL_REG);
	ULPD_SAVE(ULPD_SOFT_REQ_REG);
	ULPD_SAVE(ULPD_STATUS_REQ_REG);
	ULPD_SAVE(ULPD_DPLL_CTRL_REG);
	ULPD_SAVE(ULPD_POWER_CTRL_REG);

	MPUI_SAVE(MPUI_CTRL_REG);
	MPUI_SAVE(MPUI_DSP_STATUS_REG);
	MPUI_SAVE(MPUI_DSP_BOOT_CONFIG);
	MPUI_SAVE(MPUI_DSP_API_CONFIG);
	MPUI_SAVE(PM_EMIFF_SDRAM_CONFIG);
	MPUI_SAVE(PM_EMIFS_CONFIG_REG);

	if (virtual_start == 0) {
		g_read_completed = 0;

		my_buffer_offset += sprintf(my_base + my_buffer_offset,
		   "ARM_CKCTL:            0x%-4x     (0x%p)\n"
		   "ARM_IDLECT1:          0x%-4x     (0x%p)\n"
		   "ARM_IDLECT2:          0x%-4x     (0x%p)\n"
		   "ARM_EWUPCT:           0x%-4x     (0x%p)\n"
		   "ARM_RSTCT1:           0x%-4x     (0x%p)\n"
		   "ARM_RSTCT2:           0x%-4x     (0x%p)\n"
		   "ARM_SYSST:            0x%-4x     (0x%p)\n"
		   "ULPD_IT_STATUS_REG:   0x%-4x     (0x%p)\n"
		   "ULPD_CLOCK_CTRL_REG:  0x%-4x     (0x%p)\n"
		   "ULPD_SOFT_REQ_REG:    0x%-4x     (0x%p)\n"
		   "ULPD_DPLL_CTRL_REG:   0x%-4x     (0x%p)\n"
		   "ULPD_STATUS_REQ_REG:  0x%-4x     (0x%p)\n"
		   "ULPD_POWER_CTRL_REG:  0x%-4x     (0x%p)\n"
		   "MPUI_CTRL_REG         0x%-8x (0x%p)\n"
		   "MPUI_DSP_STATUS_REG:  0x%-8x (0x%p)\n"
		   "MPUI_DSP_BOOT_CONFIG: 0x%-8x (0x%p)\n"
		   "MPUI_DSP_API_CONFIG:  0x%-8x (0x%p)\n" 
		   "PM_MPUI_SDRAM_CONFIG:    0x%-8x (0x%p)\n" 
		   "PM_MPUI_EMIFS_CONFIG:    0x%-8x (0x%p)\n", 
		   ARM_SHOW(ARM_CKCTL), ARM_CKCTL, 
		   ARM_SHOW(ARM_IDLECT1), ARM_IDLECT1, 
		   ARM_SHOW(ARM_IDLECT2), ARM_IDLECT2, 
		   ARM_SHOW(ARM_EWUPCT), ARM_EWUPCT, 
		   ARM_SHOW(ARM_RSTCT1), ARM_RSTCT1, 
		   ARM_SHOW(ARM_RSTCT2), ARM_RSTCT2,
		   ARM_SHOW(ARM_SYSST), ARM_SYSST,
		   ULPD_SHOW(ULPD_IT_STATUS_REG), ULPD_IT_STATUS_REG,
		   ULPD_SHOW(ULPD_CLOCK_CTRL_REG), ULPD_CLOCK_CTRL_REG,
		   ULPD_SHOW(ULPD_SOFT_REQ_REG), ULPD_SOFT_REQ_REG,
		   ULPD_SHOW(ULPD_DPLL_CTRL_REG), ULPD_DPLL_CTRL_REG,
		   ULPD_SHOW(ULPD_STATUS_REQ_REG), ULPD_STATUS_REQ_REG,
		   ULPD_SHOW(ULPD_POWER_CTRL_REG), ULPD_POWER_CTRL_REG,
		   MPUI_SHOW(MPUI_CTRL_REG), MPUI_CTRL_REG,
		   MPUI_SHOW(MPUI_DSP_STATUS_REG), MPUI_DSP_STATUS_REG,
		   MPUI_SHOW(MPUI_DSP_BOOT_CONFIG), MPUI_DSP_BOOT_CONFIG,
		   MPUI_SHOW(MPUI_DSP_API_CONFIG), MPUI_DSP_API_CONFIG,
		   MPUI_SHOW(PM_EMIFF_SDRAM_CONFIG), PM_EMIFF_SDRAM_CONFIG,
		   MPUI_SHOW(PM_EMIFS_CONFIG_REG), PM_EMIFS_CONFIG_REG);
		g_read_completed++;
	} else if (g_read_completed >= 1) {
		 *eof = 1;
		 return 0;
	}
	g_read_completed++;

	*my_first_byte = page_buffer;
	return  my_buffer_offset;
}

int __init
omap_pm_init(void)
{
	struct proc_dir_entry *entry;
	extern unsigned int omap1610_cpu_suspend_sz;
	extern unsigned int omap1610_idle_loop_suspend_sz;

 	pm_idle = omap1610_pm_idle;
	printk("Power Management for the OMAP1610 is initialized pm_idle 0x%p\n",
	       pm_idle);
	entry = create_proc_read_entry("pm", S_IWUSR | S_IRUGO, NULL,
				       omap_pm_read_proc, 0);
	if (entry)
		entry->write_proc = (write_proc_t *)omap_pm_write_proc;
	/*
	 * We copy the assembler sleep/wakeup routines to SRAM.
	 * These routines need to be in SRAM as that's the only
	 * memory the MPU can see when it wakes up.
	 */

	memcpy((void *)OMAP1610_SRAM_IDLE_SUSPEND, omap1610_idle_loop_suspend,
	    omap1610_idle_loop_suspend_sz);
	memcpy((void *)OMAP1610_SRAM_API_SUSPEND, omap1610_cpu_suspend,
	    omap1610_cpu_suspend_sz);

	return 0;
}
__initcall(omap_pm_init);

#endif /* CONFIG_PROC_FS */

#ifdef CONFIG_SYSCTL

/*
 * ARGH!  ACPI people defined CTL_ACPI in linux/acpi.h rather than
 * linux/sysctl.h.
 */

#define CTL_ACPI 9999   
#define APM_S1_SLP_TYP 19

/*
 * Send us to sleep.
 */

static int sysctl_omap_pm_suspend(void)
{
	int retval;

	retval = pm_send_all(PM_SUSPEND, (void *)3);

	if (retval == 0) {
		omap_pm_suspend();

		pm_send_all(PM_RESUME, (void *)0);
	}

	return retval;
}

static struct ctl_table pm_table[] =
{
	{APM_S1_SLP_TYP, "suspend", NULL, 0, 0600, NULL,
	    (proc_handler *)&sysctl_omap_pm_suspend},

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

#endif /* CONFIG_SYSCTL */
