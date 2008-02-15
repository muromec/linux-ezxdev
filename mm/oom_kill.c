/*
 *  linux/mm/oom_kill.c
 * 
 *  Copyright (C)  1998,2000  Rik van Riel
 *	Thanks go out to Claus Fischer for some serious inspiration and
 *	for goading me into coding this file...
 *
 *  The routines in this file are used to kill a process when
 *  we're seriously out of memory. This gets called from kswapd()
 *  in linux/mm/vmscan.c when we really run out of memory.
 *
 *  Since we won't call these routines often (on a well-configured
 *  machine) this file will double as a 'coding guide' and a signpost
 *  for newbie kernel hackers. It features several pointers to major
 *  kernel subsystems and hints as to where to find out what things do.
 */
/*
 *
 *
 * 2005-Apr-05  Motorola  Add security patch
 */

#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/swap.h>
#include <linux/swapctl.h>
#include <linux/timex.h>
#include <linux/security.h>

/* #define DEBUG */

#ifdef CONFIG_EMBEDDED_OOM_KILLER
#ifdef CONFIG_PRIORITIZED_OOM_KILL
#include <linux/proc_fs.h>
#include <linux/oom.h>
#include <asm/uaccess.h>
#include <linux/string.h>

static int oom_debug = 0;
#define DPRINT if (oom_debug) printk 
oom_procs_t oom_killable;
oom_procs_t oom_dontkill;

int
dontkill_read_proc(char *page_buffer, char **first_byte, off_t offset,
	int length, int *eof, void *data)
{
	int len = 0;
	char *const base = page_buffer;
	static int bucket = 0;
	oom_procs_t *entry = NULL;

	if (offset == 0) {
		/* 
		 * Just been opened so display the header information 
		 */
		len += sprintf(base + len, "OOM killer dontkill list\n");
	} else if (bucket == -1) {
		 *eof = 1;
		 return 0;
	}

	entry = &oom_dontkill;
	do {
		len += sprintf(base + len, "%s\n", entry->name);
		entry = entry->next;
	} while ((entry != NULL) && (len < length));

	if (entry == NULL) 
		bucket = -1;
	*first_byte = page_buffer;
	return  len;
}

ssize_t
dontkill_write_proc(struct file *file, const char *buf, size_t count,
    loff_t *ppos)
{
	oom_procs_t *next, *this, *entry;
	int i;

	if (count >= PROC_NAME_SIZE)
		count = PROC_NAME_SIZE - 1;
	if ((entry = (oom_procs_t *)kmalloc(sizeof (struct oom_procs),
	    GFP_KERNEL)) == NULL) {
		return -ENOMEM;
	}
	for (i = 0; i < PROC_NAME_SIZE; i++)
		entry->name[i] = '\0';
	if (copy_from_user(entry->name, buf, count)) {
		kfree(entry);
		return -EFAULT;
	}
	if (entry->name[count - 1] == '\n')
		entry->name[count - 1] = '\0';

	/*
	 * If they pass the string erase, then erase the entire list
	 */

	if (strncmp("erase", entry->name, 5) == 0) {
		DPRINT("erasing dontkill list ");
		next = oom_dontkill.next;
		/* erase first entry */
		for (i = 0; i < PROC_NAME_SIZE; i++)
			oom_dontkill.name[i] = '\0';
		/* erase the chain */
		while (next != NULL) {
			this = next;
			next = next->next;
			DPRINT("%s ", this->name);
			kfree(this);
		}
		oom_dontkill.next = NULL;
		kfree(entry);
		return count;
	}

	/*
 	 * else add the oom_procs struct to the linked list 
	 */

	if (count >= PROC_NAME_SIZE)
		count = PROC_NAME_SIZE - 1;
	/* get first entry in */
	if (oom_dontkill.name[0] == '\0') {
		DPRINT("adding %s %d to the first dontkill slot\n",
		    entry->name, count);
		strncpy(oom_dontkill.name, entry->name, count);
		oom_dontkill.len = count;
		oom_dontkill.next = NULL;
		return count;
	}
	/* chain all other entries to the end in FIFO fashion */
	this = &oom_dontkill;
	next = oom_dontkill.next;
	do {
		if (next == NULL) {
			this->next = entry;
			entry->next = NULL;
			entry->len = count;
			DPRINT("adding %s %d to chain 0x%x\n", this->name,
			    this->len, this);
			break;
		}
		this = next;
		next = next->next;
	} while (this != NULL);

	return count;
}

int
killable_read_proc(char *page_buffer, char **first_byte, off_t offset,
	int length, int *eof, void *data)
{
	int len = 0;
	char *const base = page_buffer;
	static int bucket = 0;
	oom_procs_t *entry = NULL;

	if (offset == 0) {
		/* 
		 * Just been opened so display the header information 
		 */
		len += sprintf(base + len, "OOM killer prioritized victims\n");
	} else if (bucket == -1) {
		 *eof = 1;
		 return 0;
	}

	entry = &oom_killable;
	do {
		len += sprintf(base + len, "%s\n", entry->name);
		entry = entry->next;
	} while ((entry != NULL) && (len < length));

	if (entry == NULL) 
		bucket = -1;
	*first_byte = page_buffer;
	return  len;
}

ssize_t
killable_write_proc(struct file *file, const char *buf, size_t count,
    loff_t *ppos)
{
	oom_procs_t *next, *this, *entry;
	int i;

	if (count >= PROC_NAME_SIZE)
		count = PROC_NAME_SIZE - 1;
	if ((entry = (oom_procs_t *)kmalloc(sizeof (struct oom_procs),
	    GFP_KERNEL)) == NULL) {
		return -ENOMEM;
	}
	for (i = 0; i < PROC_NAME_SIZE; i++)
		entry->name[i] = '\0';
	if (copy_from_user(entry->name, buf, count)) {
		kfree(entry);
		return -EFAULT;
	}
	if (entry->name[count - 1] == '\n')
		entry->name[count - 1] = '\0';

	/*
	 * If they pass the string erase, then erase the entire list
	 */

	if (strncmp("erase", entry->name, 5) == 0) {
		DPRINT("erasing killable list ");
		next = oom_killable.next;
		for (i = 0; i < PROC_NAME_SIZE; i++)
			oom_killable.name[i] = '\0';
		while (next != NULL) {
			this = next;
			next = next->next;
			DPRINT("%s ", this->name);
			kfree(this);
		}
		oom_killable.next = NULL;
		oom_killable.len = 0;
		kfree(entry);
		return count;
	}

	/*
 	 * else add the oom_procs struct to the linked list 
	 */

	if (count >= PROC_NAME_SIZE)
		count = PROC_NAME_SIZE - 1;
	/* add first entry */
	if (oom_killable.name[0] == '\0') {
		DPRINT("adding %s %d to first killabke stlot\n",
		    entry->name, count);
		strncpy(oom_killable.name, entry->name, count);
		oom_killable.len = count;
		oom_killable.next = NULL;
		return count;
	}
	/* chain other entris in the list in FIFO fashion. */
	this = &oom_killable;
	next = oom_killable.next;
	do {
		if (next == NULL) {
			this->next = entry;
			entry->next = NULL;
			entry->len = count;
			DPRINT("adding %s %d to chain 0x%x\n", entry->name,
			    entry->len, this);
			break;
		}
		this = next;
		next = next->next;
	} while (this != NULL);

	return count;

}
#endif /* CONFIG_PRIORITIZED_OOM_KILL */

/**
 * int_sqrt - oom_kill.c internal function, rough approximation to sqrt
 * @x: integer of which to calculate the sqrt
 * 
 * A very rough approximation to the sqrt() function.
 */
static unsigned int int_sqrt(unsigned int x)
{
	unsigned int out = x;
	while (x & ~(unsigned int)1) x >>=2, out >>=1;
	if (x) out -= out >> 2;
	return (out ? out : 1);
}	

/**
 * oom_badness - calculate a numeric value for how bad this task has been
 * @p: task struct of which task we should calculate
 *
 * The formula used is relatively simple and documented inline in the
 * function. The main rationale is that we want to select a good task
 * to kill when we run out of memory.
 *
 * Good in this context means that:
 * 1) we lose the minimum amount of work done
 * 2) we recover a large amount of memory
 * 3) we don't kill anything innocent of eating tons of memory
 * 4) we want to kill the minimum amount of processes (one)
 * 5) we try to kill the process the user expects us to kill, this
 *    algorithm has been meticulously tuned to meet the priniciple
 *    of least surprise ... (be careful when you change it)
 */

static int badness(struct task_struct *p)
{
	int points, cpu_time, run_time;

	if (!p->mm)
		return 0;
	/*
	 * The memory size of the process is the basis for the badness.
	 */
	points = p->mm->total_vm;

	/*
	 * CPU time is in seconds and run time is in minutes. There is no
	 * particular reason for this other than that it turned out to work
	 * very well in practice. This is not safe against jiffie wraps
	 * but we don't care _that_ much...
	 */
	cpu_time = (p->times.tms_utime + p->times.tms_stime) >> (SHIFT_HZ + 3);
	run_time = (jiffies - p->start_time) >> (SHIFT_HZ + 10);

	points /= int_sqrt(cpu_time);
	points /= int_sqrt(int_sqrt(run_time));

	/*
	 * Niced processes are most likely less important, so double
	 * their badness points.
	 */
	if (task_nice(p) > 0)
		points *= 2;

	/*
	 * Superuser processes are usually more important, so we make it
	 * less likely that we kill those.
	 */
	if (!security_capable(p,CAP_SYS_ADMIN) ||
				p->uid == 0 || p->euid == 0)
		points /= 4;

	/*
	 * We don't want to kill a process with direct hardware access.
	 * Not only could that mess up the hardware, but usually users
	 * tend to only have this flag set on applications they think
	 * of as important.
	 */
	if (!security_capable(p,CAP_SYS_RAWIO))
		points /= 4;
#ifdef DEBUG
	printk(KERN_DEBUG "OOMkill: task %d (%s) got %d points\n",
	p->pid, p->comm, points);
#endif
	return points;
}

/*
 * Simple selection loop. We chose the process with the highest
 * number of 'points'. We expect the caller will lock the tasklist.
 *
 * (not docbooked, we don't want this one cluttering up the manual)
 */
static struct task_struct * select_bad_process(void)
{
	int maxpoints = 0;
	struct task_struct *p = NULL;
	struct task_struct *chosen = NULL;

#ifdef CONFIG_PRIORITIZED_OOM_KILL
	oom_procs_t *entry = NULL;
	int skip = 0;

	/*
 	 * we're going to implement 2 lists in /proc.  One list contains
	 * the names of processes to NOT kill the other contains lists
	 * of processes the user says can be killed in extreme emergencies.
	 * /proc/sys/kernel/oom/dontkill, /proc/sys/kernel/oom/killable
	 * the last resort is to fall back on the normal hueristic.
	 * for each process in the killable list do
	 *	find the process and kill it.  If we've reclaimed 
	 * 	enough memory return after erasing the kill process's
	 *	name from the killable list.
	 * If we don't have enough memory after killing everyone in
	 * the killable list we go to the old hueristic, except
	 * we skip the ones in the don't kill list.
	 */

	if (oom_killable.name[0] == '\0')
		goto no_killable_list;
	entry = &oom_killable;
	for_each_task(p) {
		while (entry != NULL) {
			DPRINT("comparing %s and %s at %d to kill\n", p->comm,
			    entry->name, entry->len);
			if ((strncmp(p->comm, entry->name, entry->len)) == 0) {
				DPRINT("returning victim %s\n", entry->name);
				return(p);
			}
			entry = entry->next;
		}
	}
no_killable_list:
#endif /* CONFIG_PRIORITIZED_OOM_KILL */
	for_each_task(p) {
#ifdef CONFIG_PRIORITIZED_OOM_KILL
		entry = &oom_dontkill;
		while (entry != NULL) {
			DPRINT("comparing %s and %s at %d to not kill\n", 
			    p->comm, entry->name, entry->len);
			if (strncmp(p->comm, entry->name, entry->len) == 0) {
				skip = 1;
				DPRINT("skipping %s as a potential victim\n",
				    entry->name);
				break;
			}
			entry = entry->next;
		}
		if (skip) {
			skip = 0;
			continue;  /* to next task */
		} else
#endif /* CONFIG_PRIORITIZED_OOM_KILL */
		if (p->pid) {
			int points = badness(p);
			if (points > maxpoints) {
				chosen = p;
				maxpoints = points;
			}
		}
	}
	return chosen;
}

/**
 * We must be careful though to never send SIGKILL a process with
 * CAP_SYS_RAW_IO set, send SIGTERM instead (but it's unlikely that
 * we select a process with CAP_SYS_RAW_IO set).
 */
static void oom_kill_task(struct task_struct *p)
{
	printk(KERN_ERR "Out of Memory: Killed process %d (%s).\n", p->pid, p->comm);

	/*
	 * We give our sacrificial lamb high priority and access to
	 * all the memory it needs. That way it should be able to
	 * exit() and clear out its resources quickly...
	 */
	p->time_slice = HZ;
	p->flags |= PF_MEMALLOC | PF_MEMDIE;

	/* This process has hardware access, be more careful. */
	if (!security_capable(p,CAP_SYS_RAWIO)) {
		force_sig(SIGTERM, p);
	} else {
		force_sig(SIGKILL, p);
	}
}

/**
 * oom_kill - kill the "best" process when we run out of memory
 *
 * If we run out of memory, we have the choice between either
 * killing a random task (bad), letting the system crash (worse)
 * OR try to be smart about which process to kill. Note that we
 * don't have to be perfect here, we just have to be good.
 */
static void oom_kill(void)
{
	struct task_struct *p, *q;

	read_lock(&tasklist_lock);
	p = select_bad_process();

	/* Found nothing?!?! Either we hang forever, or we panic. */
	if (p == NULL)
		panic("Out of memory and no killable processes...\n");

	/* kill all processes that share the ->mm (i.e. all threads) */
	for_each_task(q) {
		if (q->mm == p->mm)
			oom_kill_task(q);
	}
	read_unlock(&tasklist_lock);

	/*
	 * Make kswapd go out of the way, so "p" has a good chance of
	 * killing itself before someone else gets the chance to ask
	 * for more memory.
	 */
	yield();
	return;
}
#endif

/**
 * out_of_memory - is the system out of memory?
 */
void out_of_memory(void)
{
#ifdef CONFIG_EMBEDDED_OOM_KILLER
	static unsigned long first, last, count, lastkill;
	unsigned long now, since;

	/*
	 * This is a trade off for embedded systems that typically
	 * have no swap devices.  By just returning here we run
	 * the risk of getblk looping forever and hanging the system. . .
	 * having your embedded application killed doesn't quite work,
	 * but neither does hanging the system.  If the apps don't
	 * completely exhaust memory they'll keep running forever, as our
	 * QA tests have shown.
	 */

	/*
	 * Enough swap space left?  Not OOM.
	 */
	if (nr_swap_pages > 0)
		return;

	now = jiffies;
	since = now - last;
	last = now;

	/*
	 * If it's been a long time since last failure,
	 * we're not oom.
	 */
	last = now;
	if (since > 5*HZ)
		goto reset;

	/*
	 * If we haven't tried for at least one second,
	 * we're not really oom.
	 */
	since = now - first;
	if (since < HZ)
		return;

	/*
	 * If we have gotten only a few failures,
	 * we're not really oom. 
	 */
	if (++count < 10)
		return;

	/*
	 * If we just killed a process, wait a while
	 * to give that task a chance to exit. This
	 * avoids killing multiple processes needlessly.
	 */
	since = now - lastkill;
	if (since < HZ*5)
		return;

	/*
	 * Ok, really out of memory. Kill something.
	 */
	lastkill = now;
	oom_kill();

reset:
	first = now;
	count = 0;
#endif /* CONFIG_OOM_KILLER */
}
