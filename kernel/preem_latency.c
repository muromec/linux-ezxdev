/*
 * FILE NAME ktools/preempt.c
 *
 * BRIEF MODULE DESCRIPTION
 * This file implements preemption latency instrumentation for discovering
 * the long preempt disable paths in the kernel.
 *
 * Author: David Singleton dsingleton@mvista.com MontaVista Software, Inc.
 *
 * 2001-2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * * 20011220 Robert M. Love
 * 	- add SH arch support
 * 	- tidy things to better support new arches
 * 	- add include/asm/preem_latency.h
 * 	- now synced with 2.4.17
 * 
 * * 20011115 Robert M. Love
 * 	- resync with 2.4.15-pre4 and the latest preempt
 * 	  kernel patch
 * 
 * * 20010923 Robert M. Love
 * 	- remove causes "lowlat" and "check"
 * 	- update for revised preemption patch
 * 	- remove defines for preempt_lock_check and window
 * 
 * * 20010919 Robert M. Love
 * 	whitespace cleanup, remove unneeded code, remove
 * 	ifdefs around latency_cause, etc
 * 
 * * 20010918 Robert M. Love
 * 	update for 2.4.10-pre11 and 2.4.9-ac13
 */

#include <asm/system.h>
#include <asm/current.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/preem_latency.h>

#define PLINE_SIZE	48
#define NUM_LOG_ENTRY	20
struct preempt_point {
	const char * fname;
	unsigned lineno;
	unsigned address;    
	unsigned task_id;
};

/* worst recorded latencies */
struct log {
	unsigned latency;
	int cause;
	struct preempt_point start;
	struct preempt_point end;
};

static unsigned logging = 0;	/* 0 - no logging; 1 - logging */

struct latency {
	unsigned samples;	/* total number preempt disable/enable pairs */
	unsigned start_count;
	int cause;
	struct preempt_point start_point;
	struct log entry[NUM_LOG_ENTRY]; /* worst recorded latencies */

};

struct latency latency_data[NR_CPUS] = { {0} };

static char *cause_name[] = {
	"Softirq",
	"spin_lock",
	"do_IRQ",
	"BKL",
	"reacqBKL",
	"unknown"
};

static void __noinstrument reset_latency_data(void)
{
	int i;
	int cpu;

	for (cpu = 0; cpu < smp_num_cpus; cpu++) {
		latency_data[cpu].samples = 0;
		for (i = 0; i < NUM_LOG_ENTRY; i++) {
			latency_data[cpu].entry[i].latency = 0;
			latency_data[cpu].entry[i].cause = 0;
			latency_data[cpu].entry[i].start.fname = NULL;
			latency_data[cpu].entry[i].start.lineno = 0;
			latency_data[cpu].entry[i].start.address = 0;
			latency_data[cpu].entry[i].start.task_id = 0;
		}
			
	}
	return;
}

asmlinkage void __noinstrument latency_start(const char *fname, unsigned lineno,
	int cause)
{
	struct latency *l = &latency_data[smp_processor_id()];

	if (logging == 0)
		return;

	if (current->preempt_count == 1) {
		readclock(l->start_count);
		if (cause == -99)
			l->cause = DO_IRQ;
		else
			l->cause = cause;
		l->start_point.address = (int)__builtin_return_address(0);
		l->start_point.fname = fname;
		l->start_point.lineno = lineno;
		l->start_point.task_id = current->pid;
        }
}

void __noinstrument latency_logentry(unsigned diff, const char *fname,
	 unsigned lineno, unsigned address)
{
	struct latency *l = &latency_data[smp_processor_id()];
	unsigned lowest = 0xffffffff;
	int i, index = 0;


	if (current->preempt_count != 1)
		return;

	for (i = 0; i < NUM_LOG_ENTRY; i++) {
		if (lowest > l->entry[i].latency) {
			lowest = l->entry[i].latency;
			index = i;
		}
		if ((lineno == l->entry[i].end.lineno) &&
		    (strncmp(fname, l->entry[i].end.fname, strlen(fname)) == 0)) {
			index = i;
			break;
		}
	}

	if (diff > l->entry[index].latency) {
		l->entry[index].latency = diff;
		if (l->cause == -99)
			l->entry[index].cause = DO_IRQ;
		else	
			l->entry[index].cause = l->cause; 

		l->entry[index].end.fname = fname;
		l->entry[index].end.lineno = lineno;
		l->entry[index].end.address = address;
		l->entry[index].end.task_id = current->pid;

		l->entry[index].start.fname = l->start_point.fname;
		l->entry[index].start.lineno = l->start_point.lineno;
		l->entry[index].start.address = l->start_point.address; 
		l->entry[index].start.task_id = l->start_point.task_id;
	}

	l->samples++;
}

/* Called at end of preemption time */
asmlinkage void __noinstrument latency_end(const char *fname, unsigned lineno)
{
	struct latency *l = &latency_data[smp_processor_id()];
	unsigned end_count;

	if (logging == 0)
		return;

	readclock(end_count);

	if (current->preempt_count != 1) {
		return;
	}
	latency_logentry(clock_diff(l->start_count, end_count), fname, lineno,
		(int)__builtin_return_address(0));
	return;
}

static ssize_t __noinstrument
latencytimes_write_proc(struct file * file, const char * buf, size_t count,
 	loff_t *ppos)
{
	logging = 0;
	reset_latency_data();
	logging = 1;
	return count;
}

static int bucket = 0;
static int __noinstrument 
latencytimes_read_proc(char *page_buffer, char **my_first_byte,
	off_t virtual_start, int length, int *eof, void *data)
{
	char *const my_base = page_buffer;
	int len = 0;
	int g_cpu = 0;
	struct latency *l = &latency_data[g_cpu];

	*my_first_byte = page_buffer;

	if (virtual_start == 0) {
	/* Just been opened */
		bucket = logging = 0;  /* stop logging */
		len += sprintf(my_base + len,
		    "    cpu %d worst %d latency times of %d samples measured.\n",
		    g_cpu, NUM_LOG_ENTRY, l->samples);
		len += sprintf(my_base + len,
		    "      usec      cause     start line/file       address   end line/file\n");

	} else if (bucket == -1) {
		if (++g_cpu >= smp_num_cpus) {
			*eof = 1;
			logging = 1;  /* start logging */
			return 0;
		} else {
			bucket = 0;
		}
	}
	for (; bucket < NUM_LOG_ENTRY && len + PLINE_SIZE < length; bucket++) {
		len += sprintf(my_base + len,
		    "%2d) %6d %10s     %5d/%-15s %8x %5d/%s\n", bucket,
		    (int)clock_to_usecs(l->entry[bucket].latency),
		    (l->entry[bucket].cause >= MAX_CAUSE) ? "unknown" : 
		    cause_name[l->entry[bucket].cause],
		    l->entry[bucket].start.lineno,
		    l->entry[bucket].start.fname == (char *)0 ?  "entry.S" : 
		    l->entry[bucket].start.fname,
		    l->entry[bucket].start.address,
		    l->entry[bucket].end.lineno,
		    l->entry[bucket].end.fname == (char *)0 ?  "entry.S" : 
		    l->entry[bucket].end.fname);
	}
	if (bucket >= NUM_LOG_ENTRY) {
		bucket = -1;
	}
	*my_first_byte = page_buffer;
	return  len;

}

int __noinstrument __init latencytimes_init(void)
{
#ifdef CONFIG_PROC_FS
        struct proc_dir_entry *entry;

	entry = create_proc_read_entry("latencytimes", 0, 0,
	    latencytimes_read_proc, 0);
        if (entry)
                entry->write_proc = (write_proc_t *)latencytimes_write_proc;
#endif

	readclock_init();
	reset_latency_data();
	return 0;
}

__initcall(latencytimes_init);
