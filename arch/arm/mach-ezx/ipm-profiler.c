/*
 * Bulverde Performance profiler and Idle profiler Routines
 *
 * Copyright (c) 2003 Intel Corporation.
 *
 */
/*
 * Copyright (C) 2005 Motorola Inc.
 *
 * 2005-May-11  add intel PM patch,  Zhuang Xiaofan
 * 2005-Nov-11  bug fix for deep idle mode,  Li Qiang
 *
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/smp_lock.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/errno.h>
#include <linux/timer.h>
#include <asm/hardware.h>
#include <asm/dma.h>
#include <asm/semaphore.h>
#include <asm/unistd.h>
#include <asm/xscale-pmu.h>
#include <linux/apm_bios.h>


#ifdef CONFIG_IPM_DEEPIDLE
#include <linux/power_ic.h>
extern int enable_deepidle;
#define VCORE_13M	900		// 900mV
#endif

int mode = 1;

#define DEFAULTWINDOWSIZE 200
#define MAX_OSCR0         0x9FFFFFFF
/* 1ms in ticks (3.25x10^6tick/sec * 1/1000sec/msec)	*/
#define OSCR0_TICKS_1MS   3250        
//#define	DEBUG

/* Performance Profiler specific variables	*/
unsigned short pprofiler_event;
unsigned long CPU_utilization;
static DECLARE_MUTEX_LOCKED(ipm_sem);

extern int get_hlt_counter(void);
extern void  (*pipm_start_pmu)(void);
static void ip_process_result(void);
static int pp_process_result(struct pmu_results *, unsigned long);


int ipm_thread_exit = 0;
struct completion ipm_thread_over;
/* Idle Profiler specific variables	*/
unsigned int 	window_started		=0;          
unsigned int 	window_start_time	=0;
unsigned int 	aggregate_idle_time	=0;
unsigned int 	iprof_enabled		=1; 
unsigned int 	samplingwindow		=20;
unsigned long 	WindowSize 		= DEFAULTWINDOWSIZE;     /* units of ms	*/
static unsigned long window_stop_time=0;  
static unsigned long Max_OSCR = (unsigned long)MAX_OSCR0;
static unsigned long OSCR0_Ticks_1ms = (unsigned long) OSCR0_TICKS_1MS;

/*	Call this function if you want to post profiler events.	*/
void ipm_postevent(void)
{
	apm_event_notify(APM_EVENT_PROFILER, EVENT_PROF_IDLE, CPU_utilization);
	apm_event_notify(APM_EVENT_PROFILER, EVENT_PROF_PERF, pprofiler_event);
}		

void ipm_start_pmu(void)
{
	up(&ipm_sem);
}

/*
 *	ipm idle profiler idle thread.
 */
void ipm_ip_idle(void)
{
	unsigned long curr_idle_start_time = 0;
	unsigned long curr_idle_stop_time = 0;
	unsigned long curr_time_idle;

	local_irq_disable();        

	if(iprof_enabled)
		curr_idle_start_time = OSCR;   /* grab the idle start time	*/
	/* value of current in this context?	*/
	if(!current->need_resched && !get_hlt_counter()) {
#if defined(CONFIG_CPU_BULVERDE) && defined(CONFIG_IPM_DEEPIDLE)
		if (enable_deepidle > 0) {
			mode = enter_13M_quickly();
//			if (!mode && get_core_voltage())
//				power_ic_set_core_voltage(VCORE_13M);
		}
#endif

		cpu_do_idle();

#if defined(CONFIG_CPU_BULVERDE) && defined(CONFIG_IPM_DEEPIDLE)
	        if (enable_deepidle > 0) {
//			if (!mode && get_core_voltage())
//				power_ic_set_core_voltage(get_core_voltage());
			exit_13M_quickly(mode);
		}
#endif
	}

	if(iprof_enabled) {
		curr_idle_stop_time = OSCR;    /* grab the idle end time */
		/*
		 * calculate total time in idle for this entry
		 */
		if(curr_idle_stop_time > curr_idle_start_time)
			curr_time_idle = (curr_idle_stop_time - curr_idle_start_time);
		else
			curr_time_idle = (Max_OSCR - curr_idle_start_time) + curr_idle_stop_time;
		/* 
		 * Aggregate total idle time for this potentially 
		 * variable-sized window
		 */
		aggregate_idle_time += curr_time_idle;
	}
	local_irq_enable();
}

/*
 * WindowSize is defined in idle-prof.c so two profilers are using 
 * the same window size.
 */
static int ipm_thread(void *data)
{
	struct task_struct *tsk = current;
	int valid;

	daemonize();
	reparent_to_init();
	strcpy(tsk->comm, "ipmd");
	tsk->tty = NULL;
	/*
	 * We run it as a real-time thread. 
	 */
	tsk->policy = SCHED_FIFO;
	tsk->rt_priority = 1;

	/* only want to receive SIGKILL */
	spin_lock_irq(&tsk->sigmask_lock);
	siginitsetinv(&tsk->blocked, sigmask(SIGKILL));
	recalc_sigpending(tsk);
	spin_unlock_irq(&tsk->sigmask_lock);
	valid = 0;

	while(1)  {
		unsigned long	dwTime;
		struct pmu_results result;

		/* Get blocked.*/
		down_interruptible(&ipm_sem);
		if (ipm_thread_exit)
			break;
		/* Monitor events-Icache,Inst.Executed,Dcachemiss,DataDependencyStall */
		pmu_start(0x01,0x07,0x0b,0x02);
		dwTime = OSCR;

		unsigned long WindowSizeJiffies = (WindowSize/10);
		unsigned long timeout = (jiffies + WindowSizeJiffies);

		while((long)(timeout - jiffies) > 0)	{    
			/* 	
			 *	why?  expecting to wake prematurely?  We should not be using 
			 *	jiffies+timeout, since schedule_timeout() takes #jiffies
			 */
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(WindowSizeJiffies);  
			/* 
			 * Make the current task sleep until @timeout jiffies have elapsed.
			 *  (jiffie = 10ms)
			 * The routine will return immediately the current task state 
			 * has been set, see set_current_state().
			 */
		}
		if(signal_pending(tsk))
			break;

		/* Window has ended; calculate CPU% */
		window_stop_time = OSCR;
		pmu_stop(&result);
		dwTime = OSCR - dwTime;
#ifdef DEBUG		
		printk("dwTime : 0x%x.\n",  dwTime);
#endif		
		/*
		 * Both process function will not post event but set event type.
		 * We need to combine two profiler's event together and post one event.
		 */
		pp_process_result( &result, dwTime);
		ip_process_result( );
		/*	Post profiler event here. */
		ipm_postevent();
	}
	complete_and_exit(&ipm_thread_over, 0);
	return 0;
}
/*
 *	Check CPU utilization and set idleprofiler_event to say what happened.
 *	If noting detected by idle-profiler, it just return previous time's 
 *	result.
 */
static void ip_process_result()
{
	/*
	 * avoiding overflow
	 */
	CPU_utilization=(unsigned long)(100 - (100*(aggregate_idle_time/OSCR0_Ticks_1ms)) /
			(window_stop_time/OSCR0_Ticks_1ms - window_start_time/OSCR0_Ticks_1ms) );
#ifdef DEBUG		
	printk("Current CPU load: %d\%.\n", CPU_utilization);
#endif		
	/*
	 * Update global vars used by idle loop
	 *    We will reset the variables here, just in case we never idle again
	 */
	window_start_time = OSCR;   /* grab the window start time	*/
	aggregate_idle_time=0;      /* reset window idle counter	*/
}
#if 0
static void PrintPMUReg(struct  pmu_results 	*presult)
{
	printk("ccnf_of: 0x%x  ", presult->ccnt_of);
	printk("ccnt: 0x%x\n", presult->ccnt);
	printk("pmn0: 0x%x  ", presult->pmn0);
	printk("pmn1: 0x%x\n", presult->pmn1);
	printk("pmn2: 0x%x  ", presult->pmn2);
	printk("pmn3: 0x%x\n", presult->pmn3);
}
#endif
/*
 *  Performance profiler process function.
 *	Set pprofiler_event to say what happened.
 */
static int pp_process_result(struct pmu_results *results, unsigned long dwTime)
{
	unsigned int	pmn1_res =0;
	unsigned int	pmn2_res =0;
	unsigned int 	f = 200;	// assume the freq is 200M

#ifdef DEBUG		
	printk("In functin %s.\n", __FUNCTION__);
	PrintPMUReg(results);
#endif		
        pmn1_res = results->pmn1/(dwTime*f/325);
        pmn2_res = results->pmn2/(dwTime*f/325);

#ifdef DEBUG		
	printk("In pp_process_result:PMN1_RES: %d, PMN2_RES: %d.\n",pmn1_res,pmn2_res);
#endif		

	pprofiler_event = 0;
	if (pmn1_res > 50) {
		/* CPU bound */
		pprofiler_event |= PERF_CPU_BOUND;
	}

	if (pmn2_res > 10)  {
		/* memory Bound */
		pprofiler_event |= PERF_MEM_BOUND;
	}

	return 0;
}

/*
 * The callback for timer used by idle (and eventually perf) profilers
 */

/* 
 * 	Initialize the PMU profiler
 *	Start the PMU_Profiler Thread
 */
extern void (*pm_idle)(void);
static int __init ipm_init(void)
{
	unsigned int ret = 0;

	iprof_enabled=1;
	pm_idle = ipm_ip_idle;
	/*  Let the pointer point to ipm_start_pmu function.    */
	pipm_start_pmu = ipm_start_pmu;
	printk("Initialize IPM performance perfiler.\n");
	ipm_thread_exit = 0;
	init_completion(&ipm_thread_over);
	/*	Should we start the kernel thread now?	*/
	ret = kernel_thread(ipm_thread, NULL , 0);

	return 0;
}

/* Exit the PMU profiler	*/
static void __exit ipm_exit(void)
{
	iprof_enabled = 0;
	pm_idle = NULL;
	pipm_start_pmu = NULL;
	printk("Quit IPM profilers\n");
	ipm_thread_exit = 1;
	up(&ipm_sem);
	wait_for_completion(&ipm_thread_over);
}

module_init(ipm_init);
module_exit(ipm_exit);

