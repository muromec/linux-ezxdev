/*
 *  linux/arch/arm/kernel/process.c
 *
 *  Copyright (C) 2004-2005 Motorola, Inc. - Port to EzX platform.
 *  Copyright (C) 1996-2000 Russell King - Converted to ARM.
 *  Origional Copyright (C) 1995  Linus Torvalds
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <stdarg.h>

#include <linux/config.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/stddef.h>
#include <linux/unistd.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/user.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/dpm.h>
#include <linux/apm_bios.h>

#include <linux/trace.h>

#include <asm/system.h>
#include <asm/leds.h>
#include <asm/uaccess.h>

extern const char *processor_modes[];
extern void setup_mm_for_reboot(char mode);

static volatile int hlt_counter;

#include <asm/arch/system.h>


void switch_to(struct task_struct *prev, struct task_struct *next,
 		struct task_struct *last)
 {
#ifdef CONFIG_ILATENCY
 	intr_ret_from_exception();
#endif
 	last = __switch_to(prev,next);
 	dpm_set_os(current->dpm_state);
 	mb();
}

void disable_hlt(void)
{
	hlt_counter++;
}

void enable_hlt(void)
{
	hlt_counter--;
}

static int __init nohlt_setup(char *__unused)
{
	hlt_counter = 1;
	return 1;
}

static int __init hlt_setup(char *__unused)
{
	hlt_counter = 0;
	return 1;
}

int get_hlt_counter(void)
{
	return hlt_counter;
}

__setup("nohlt", nohlt_setup);
__setup("hlt", hlt_setup);

void (*pm_power_off)(void);
void (*pm_idle)(void);

/*
 * This is our default idle handler.  We need to disable
 * interrupts here to ensure we don't miss a wakeup call.
 */
void default_idle(void)
{
#ifdef CONFIG_ILATENCY
	__intr_cli();
#else
	local_irq_disable();
#endif
	if (!current->need_resched && !hlt_counter) {
		TRACE_PROCESS(TRACE_EV_PROCESS_IDLE_BEGIN, 0, 0);
		arch_idle();
	}
#ifdef CONFIG_ILATENCY
	__intr_sti();
#else
	local_irq_enable();
#endif
}

/*
 * The idle thread.  We try to conserve power, while trying to keep
 * overall latency low.  The architecture specific idle is passed
 * a value to indicate the level of "idleness" of the system.
 */

void cpu_idle(void)
{
	/* endless idle loop with no priority at all */

	while (1) {
		while (!need_resched()) {
			void (*idle)(void) = pm_idle;
			if (!idle)
				idle = default_idle;
            /* Since EzX does not enable DPM, so dmp functions are commented out. Jingze, 6/15/2005  */
			/*  leds_event(led_idle_start); */ 
			/* dpm_set_os(DPM_IDLE_STATE); */ 
			idle();
			/* dpm_set_os(DPM_IDLE_TASK_STATE); */
			/* leds_event(led_idle_end);    */
		}
		schedule();
#ifndef CONFIG_NO_PGT_CACHE
		check_pgt_cache();
#endif
	}
}

static char reboot_mode = 'h';

int __init reboot_setup(char *str)
{
	reboot_mode = str[0];
	return 1;
}

__setup("reboot=", reboot_setup);

void machine_halt(void)
{
	leds_event(led_halted);
}

void machine_power_off(void)
{
	leds_event(led_halted);
	if (pm_power_off)
		pm_power_off();
	else
		printk(KERN_EMERG "Can not power down!!!\n");
}

void machine_restart(char * __unused)
{
#if defined(CONFIG_ARCH_OMAP730)
	/* Disable the DPLL to enable restart. Lower bit 4 of the DPLL register.
	 * 
	 * This bad behavior is due a bug of OMAP (OMAP7xx, OMAP16xx have this bug).
	 * If a watchdog reset or a SW reset occurs, then OMAP core is reset except 
	 * the DPLL.
	 *
	 * So, after this reset, OMAP core will come back into fully synchronous mode
	 * (TC frequency = ARM frequency = DPLL frequency) with DPLL running at full
	 * speed: This will hang TC (because it will run at 200MHz) and later P2.
	 */
	printk(KERN_INFO "Disable the DPLL\n");
	*(volatile int *)DPLL1_CTL_REG &=  ~(1 << 4);
#endif

	/*
	 * Clean and disable cache, and turn off interrupts
	 */
	cpu_proc_fin();

	/*
	 * Tell the mm system that we are going to reboot -
	 * we may need it to insert some 1:1 mappings so that
	 * soft boot works.
	 */
	setup_mm_for_reboot(reboot_mode);

	/*
	 * Now call the architecture specific reboot code.
	 */
	arch_reset(reboot_mode);

	/*
	 * Whoops - the architecture was unable to reboot.
	 * Tell the user!
	 */
	mdelay(1000);
	printk("Reboot failed -- System halted\n");
	while (1);
}

#ifdef CONFIG_ARCH_EZXBASE
void ezxpanic_machine_restart(char * __unused)
{
        /*
         * Clean and disable cache, and turn off interrupts
         */
        cpu_proc_fin();
                                                                                                                             
        /*
         * Tell the mm system that we are going to reboot -
         * we may need it to insert some 1:1 mappings so that
         * soft boot works.
         */
        setup_mm_for_reboot(reboot_mode);
                                                                                                                             
        /*
         * Now reset bp.
         */
        *(unsigned long *)(phys_to_virt(BPSIG_ADDR)) = NO_FLAG;
        if (!(GPLR(GPIO_BB_WDI) & GPIO_bit(GPIO_BB_WDI))) {
        	*(unsigned long *)(phys_to_virt(BPSIG_ADDR)) = WDI_FLAG;
        }
        GPCR(GPIO_BB_RESET) = GPIO_bit(GPIO_BB_RESET);

        /* Initialize the watchdog and let it fire */
        OWER = OWER_WME;
        OSSR = OSSR_M3;
        OSMR3 = OSCR + CLOCK_TICK_RATE/100;     /* ... in 10 ms */
        MDREFR |= MDREFR_SLFRSH;

        while(1);
}
#endif 

void show_regs(struct pt_regs * regs)
{
	unsigned long flags;

	flags = condition_codes(regs);

	printk("pc : [<%08lx>]    lr : [<%08lx>]    %s\n"
	       "sp : %08lx  ip : %08lx  fp : %08lx\n",
		instruction_pointer(regs),
		regs->ARM_lr, print_tainted(), regs->ARM_sp,
		regs->ARM_ip, regs->ARM_fp);
	printk("r10: %08lx  r9 : %08lx  r8 : %08lx\n",
		regs->ARM_r10, regs->ARM_r9,
		regs->ARM_r8);
	printk("r7 : %08lx  r6 : %08lx  r5 : %08lx  r4 : %08lx\n",
		regs->ARM_r7, regs->ARM_r6,
		regs->ARM_r5, regs->ARM_r4);
	printk("r3 : %08lx  r2 : %08lx  r1 : %08lx  r0 : %08lx\n",
		regs->ARM_r3, regs->ARM_r2,
		regs->ARM_r1, regs->ARM_r0);
	printk("Flags: %c%c%c%c",
		flags & CC_N_BIT ? 'N' : 'n',
		flags & CC_Z_BIT ? 'Z' : 'z',
		flags & CC_C_BIT ? 'C' : 'c',
		flags & CC_V_BIT ? 'V' : 'v');
	printk("  IRQs o%s  FIQs o%s  Mode %s%s  Segment %s\n",
		interrupts_enabled(regs) ? "n" : "ff",
		fast_interrupts_enabled(regs) ? "n" : "ff",
		processor_modes[processor_mode(regs)],
		thumb_mode(regs) ? " (T)" : "",
		get_fs() == get_ds() ? "kernel" : "user");
#if defined(CONFIG_CPU_32)
	{
		unsigned int ctrl, transbase, dac;
		  __asm__ (
		"	mrc p15, 0, %0, c1, c0\n"
		"	mrc p15, 0, %1, c2, c0\n"
		"	mrc p15, 0, %2, c3, c0\n"
		: "=r" (ctrl), "=r" (transbase), "=r" (dac));
		printk("Control: %04X  Table: %08X  DAC: %08X\n",
		  	ctrl, transbase, dac);
	}
#endif
}

void show_fpregs(struct user_fp *regs)
{
	int i;

	for (i = 0; i < 8; i++) {
		unsigned long *p;
		char type;

		p = (unsigned long *)(regs->fpregs + i);

		switch (regs->ftype[i]) {
			case 1: type = 'f'; break;
			case 2: type = 'd'; break;
			case 3: type = 'e'; break;
			default: type = '?'; break;
		}
		if (regs->init_flag)
			type = '?';

		printk("  f%d(%c): %08lx %08lx %08lx%c",
			i, type, p[0], p[1], p[2], i & 1 ? '\n' : ' ');
	}
			

	printk("FPSR: %08lx FPCR: %08lx\n",
		(unsigned long)regs->fpsr,
		(unsigned long)regs->fpcr);
}

/*
 * Task structure and kernel stack allocation.
 */
static struct task_struct *task_struct_head;
static unsigned int nr_task_struct;

#ifdef CONFIG_CPU_32
#define EXTRA_TASK_STRUCT	4
#else
#define EXTRA_TASK_STRUCT	0
#endif

struct task_struct *alloc_task_struct(void)
{
	struct task_struct *tsk;

	preempt_disable();
	if (EXTRA_TASK_STRUCT)
		tsk = task_struct_head;
	else
		tsk = NULL;

	if (tsk) {
		task_struct_head = tsk->next_task;
		nr_task_struct -= 1;
	} else
		tsk = ll_alloc_task_struct();

#ifdef CONFIG_SYSRQ
	/*
	 * The stack must be cleared if you want SYSRQ-T to
	 * give sensible stack usage information
	 */
	if (tsk) {
		char *p = (char *)tsk;
		memzero(p+KERNEL_STACK_SIZE, KERNEL_STACK_SIZE);
	}
#endif
	preempt_enable();
	return tsk;
}

void __free_task_struct(struct task_struct *p)
{
	preempt_disable();
	if (EXTRA_TASK_STRUCT && nr_task_struct < EXTRA_TASK_STRUCT) {
		p->next_task = task_struct_head;
		task_struct_head = p;
		nr_task_struct += 1;
	} else
		ll_free_task_struct(p);
	preempt_enable();
}

/*
 * Free current thread data structures etc..
 */
void exit_thread(void)
{
}

static void default_fp_init(union fp_state *fp)
{
	memset(fp, 0, sizeof(union fp_state));
}

void (*fp_init)(union fp_state *) = default_fp_init;

void flush_thread(void)
{
	struct task_struct *tsk = current;

	preempt_disable();
	tsk->flags &= ~PF_USEDFPU;
	tsk->used_math = 0;

	memset(&tsk->thread.debug, 0, sizeof(struct debug_info));
	fp_init(&tsk->thread.fpstate);
	preempt_enable();
}

extern void wmmx_task_release(struct task_struct *);

void release_thread(struct task_struct *dead_task)
{
#ifdef CONFIG_XSCALE_WMMX
	wmmx_task_release(dead_task);
#endif
}

asmlinkage void ret_from_fork(void) __asm__("ret_from_fork");

int
copy_thread(int nr, unsigned long clone_flags, unsigned long esp,
	    unsigned long unused, struct task_struct * p, struct pt_regs * regs)
{
	struct pt_regs *childregs;
	struct context_save_struct * save;

	preempt_disable();
	atomic_set(&p->thread.refcount, 1);

	childregs = ((struct pt_regs *)((unsigned long)p + 8192 - 8)) - 1;
	*childregs = *regs;
	childregs->ARM_r0 = 0;
	childregs->ARM_sp = esp;

	save = ((struct context_save_struct *)(childregs)) - 1;
	*save = INIT_CSS;
	save->pc |= (unsigned long)ret_from_fork;

	p->thread.save = save;
	preempt_enable();

	return 0;
}

/*
 * fill in the fpe structure for a core dump...
 */
int dump_task_fpu (struct pt_regs *regs, struct task_struct *tsk,
		   struct user_fp *fp)
{
	int used_math = tsk->used_math;

	if (used_math)
		memcpy(fp, &tsk->thread.fpstate.soft, sizeof (*fp));

	return used_math;
}

int dump_fpu (struct pt_regs *regs, struct user_fp *fp)
{
	return dump_task_fpu (regs, current, fp);
}

/*
 * fill in the user structure for a core dump..
 */
void dump_thread(struct pt_regs * regs, struct user * dump)
{
	struct task_struct *tsk = current;

	dump->magic = CMAGIC;
	dump->start_code = tsk->mm->start_code;
	dump->start_stack = regs->ARM_sp & ~(PAGE_SIZE - 1);

	dump->u_tsize = (tsk->mm->end_code - tsk->mm->start_code) >> PAGE_SHIFT;
	dump->u_dsize = (tsk->mm->brk - tsk->mm->start_data + PAGE_SIZE - 1) >> PAGE_SHIFT;
	dump->u_ssize = 0;

	dump->u_debugreg[0] = tsk->thread.debug.bp[0].address;
	dump->u_debugreg[1] = tsk->thread.debug.bp[1].address;
	dump->u_debugreg[2] = tsk->thread.debug.bp[0].insn.arm;
	dump->u_debugreg[3] = tsk->thread.debug.bp[1].insn.arm;
	dump->u_debugreg[4] = tsk->thread.debug.nsaved;

	if (dump->start_stack < 0x04000000)
		dump->u_ssize = (0x04000000 - dump->start_stack) >> PAGE_SHIFT;

	dump->regs = *regs;
	dump->u_fpvalid = dump_fpu (regs, &dump->u_fp);
}

/*
 * This is the mechanism for creating a new kernel thread.
 *
 * NOTE! Only a kernel-only process(ie the swapper or direct descendants
 * who haven't done an "execve()") should use this: it will work within
 * a system call from a "real" process, but the process memory space will
 * not be free'd until both the parent and the child have exited.
 */
pid_t kernel_thread(int (*fn)(void *), void *arg, unsigned long flags)
{
	pid_t __ret;

	__asm__ __volatile__(
	"orr	r0, %1, %2	@ kernel_thread sys_clone	\n\
	mov	r1, #0						\n\
	"__syscall(clone)"					\n\
	movs	%0, r0		@ if we are the child		\n\
	bne	1f						\n\
	mov	fp, #0		@ ensure that fp is zero	\n\
	mov	r0, %4						\n\
	mov	lr, pc						\n\
	mov	pc, %3						\n\
	b	sys_exit					\n\
1:	"
        : "=&r" (__ret)
        : "Ir" (flags), "I" (CLONE_VM), "r" (fn), "r" (arg)
	: "r0", "r1", "lr");
#if (CONFIG_TRACE || CONFIG_TRACE_MODULE)
	if (__ret > 0)
		TRACE_PROCESS(TRACE_EV_PROCESS_KTHREAD, __ret, (int) fn);
#endif
	
	return __ret;
}

/*
 * These bracket the sleeping functions..
 */
extern void scheduling_functions_start_here(void);
extern void scheduling_functions_end_here(void);
#define first_sched	((unsigned long) scheduling_functions_start_here)
#define last_sched	((unsigned long) scheduling_functions_end_here)

unsigned long get_wchan(struct task_struct *p)
{
	unsigned long fp, lr;
	unsigned long stack_page;
	int count = 0;

	preempt_disable();
	if (!p || p == current || p->state == TASK_RUNNING) {
		preempt_enable();
		return 0;
	}

	stack_page = 4096 + (unsigned long)p;
	fp = thread_saved_fp(&p->thread);
	do {
		if (fp < stack_page || fp > 4092+stack_page) {
			preempt_enable();
			return 0;
		}
		lr = pc_pointer (((unsigned long *)fp)[-1]);
		if (lr < first_sched || lr > last_sched) {
			preempt_enable();
			return lr;
		}
		fp = *(unsigned long *) (fp - 12);
	} while (count ++ < 16);
	preempt_enable();
	return 0;
}
