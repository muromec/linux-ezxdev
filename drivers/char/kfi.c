/*
 *  drivers/char/kfi.c
 *
 *  Kernel Function Instrumentation
 *
 *  Support for Function Instrumentation/Profiling feature of
 *  GCC (-finstrument-functions).
 *
 *  Author: Steve Longerbeam stevel@mvista.com MontaVista Software, Inc.
 *
 *  2001-2004 (c) MontaVista Software, Inc. This file is licensed under
 *  the terms of the GNU General Public License version 2. This program
 *  is licensed "as is" without any warranty of any kind, whether express
 *  or implied.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/spinlock.h>
#include <linux/smp_lock.h>
#include <linux/proc_fs.h>
#include <linux/kfi.h>
#include <asm/uaccess.h>
#include <asm/hardirq.h>
#include <asm/preem_latency.h>

#define KFI_MODULE_NAME "KFI"
#define PFX KFI_MODULE_NAME

#define err(format, arg...) printk(KERN_ERR PFX ": " format , ## arg)
#define info(format, arg...) printk(KERN_INFO PFX ": " format , ## arg)
#define warn(format, arg...) printk(KERN_WARNING PFX ": " format , ## arg)
#define emerg(format, arg...) printk(KERN_EMERG PFX ": " format , ## arg)

#define KFI_MINOR 51

#ifdef CONFIG_KFI_STATIC_RUN
extern struct kfi_run kfi_run0;
extern struct kfi_run* kfi_last_run;
extern const int kfi_num_runs;
static struct kfi_run* run_head = &kfi_run0;
static struct kfi_run* run_curr = &kfi_run0;
static struct kfi_run* run_tail = NULL;
#else
static struct kfi_run* run_head = NULL;
static struct kfi_run* run_curr = NULL;
static struct kfi_run* run_tail = NULL;
#endif // CONFIG_KFI_STATIC_RUN

static int next_run_id = 0;
static int in_entry_exit = 0;

//static spinlock_t kfi_lock = SPIN_LOCK_UNLOCKED;
DECLARE_WAIT_QUEUE_HEAD(kfi_wait);

extern unsigned long do_getmachinecycles(void);
extern unsigned long machinecycles_to_usecs(unsigned long cycles);

static unsigned long usecs_since_boot = 0;
static unsigned long last_machine_cycles = 0;

static inline unsigned long __noinstrument
update_usecs_since_boot(void)
{
	unsigned long machine_cycles, delta;

	machine_cycles = do_getmachinecycles();
	delta = machine_cycles - last_machine_cycles;
	delta = machinecycles_to_usecs(delta);

	usecs_since_boot += delta;
	
	last_machine_cycles = machine_cycles;
	return usecs_since_boot;
}

static inline struct kfi_entry* __noinstrument
new_entry(struct kfi_run* run)
{
	struct kfi_entry* entry;
	
	if (run->next_entry >= run->num_entries)
		return NULL;
	
	entry = &run->log[run->next_entry];
	run->next_entry++;
	return entry;
}


static inline int __noinstrument
find_entry(struct kfi_run* run, void *this_fn, unsigned int pid)
{
	int i;

	for (i = run->next_entry-1; i >= 0; i--) {
		struct kfi_entry* entry = &run->log[i];
		if (entry->va == this_fn &&
		    entry->pid == pid &&
		    entry->delta == 0)
			return i;
	}

	return -1;
}


static inline void __noinstrument
free_entry(struct kfi_run* run, int loc)
{
	int i;
	run->next_entry--;
	for (i = loc; i < run->next_entry; i++)
		run->log[i] = run->log[i+1];
}


static inline int __noinstrument
in_func_list(struct kfi_filters* filters, void* func)
{
	int i;

	for (i=0; i < filters->func_list_size; i++) {
		if (filters->func_list[i] == func)
			return 1;
	}

	return 0;
}


static inline int __noinstrument
test_filters(struct kfi_filters* filters, void *this_fn)
{
	int in_intr;
	
	if (filters->func_list && !in_func_list(filters, this_fn)) {
#ifdef KFI_DEBUG
		filters->cnt.func_list++;
#endif
		return 1;
	}

	in_intr = in_interrupt();

	if (filters->no_ints && in_intr) {
#ifdef KFI_DEBUG
		filters->cnt.no_ints++;
#endif
		return 1;
	}

	if (filters->only_ints && !in_intr) {
#ifdef KFI_DEBUG
		filters->cnt.only_ints++;
#endif
		return 1;
	}
	
	return 0;
}


static inline void __noinstrument
do_func_entry(struct kfi_run* run, void *this_fn, void *call_site)
{
	struct kfi_entry* entry;

	if (!(entry = new_entry(run))) {
		run->complete = 1;
		run->stop_trigger.mark = update_usecs_since_boot();
		run->stop_trigger.type = TRIGGER_LOG_FULL;
		return;
	}
	
	entry->va = this_fn;
	entry->call_site = call_site;
	entry->pid = in_interrupt() ? INTR_CONTEXT : current->pid;

	entry->delta = 0;
	entry->time = update_usecs_since_boot() - run->start_trigger.mark;
}

static inline void __noinstrument
do_func_exit(struct kfi_run* run, void *this_fn, void *call_site)
{
	struct kfi_entry* entry;
	unsigned long exittime;
	unsigned long delta;
	int entry_i;

	if ((entry_i = find_entry(run, this_fn, in_interrupt() ?
				  INTR_CONTEXT : current->pid)) < 0) {
#ifdef KFI_DEBUG
		run->notfound++;
#endif
		return;
	}
	
	entry = &run->log[entry_i];

	// calc delta
	exittime = update_usecs_since_boot() - run->start_trigger.mark;
	delta = exittime - entry->time;
	
	if ((run->filters.min_delta && delta < run->filters.min_delta) ||
	    (run->filters.max_delta && delta > run->filters.max_delta)) {
#ifdef KFI_DEBUG
		run->filters.cnt.delta++;
#endif
		free_entry(run, entry_i);
	} else {
		entry->delta = delta;
	}
}


static inline int __noinstrument test_trigger(struct kfi_run* run,
					      int start_trigger,
					      int func_entry,
					      void* func_addr)
{
	unsigned long time, base_time;
	int ret = 0;
	struct kfi_trigger* t;
		
	t = start_trigger ? &run->start_trigger : &run->stop_trigger;

	switch (t->type) {
	case TRIGGER_TIME:
		time = update_usecs_since_boot();
		if (start_trigger) {
			/* trigger start time based from boot */
			base_time = 0;
		} else {
			/* trigger stop time based from start trigger time */
			base_time = run->start_trigger.mark;
		}
		
		if (time >= base_time + t->time) {
			t->mark = time; // mark trigger time
			ret = 1;
		}
		break;
	case TRIGGER_FUNC_ENTRY:
		if (func_entry && func_addr == t->func_addr) {
			time = update_usecs_since_boot();
			t->mark = time; // mark trigger time
			ret = 1;
		}
		break;
	case TRIGGER_FUNC_EXIT:
		if (!func_entry && func_addr == t->func_addr) {
			time = update_usecs_since_boot();
			t->mark = time; // mark trigger time
			ret = 1;
		}
		break;
	default:
		break;
	}

	return ret;
}


static inline void __noinstrument
func_entry_exit(void *this_fn, void *call_site, int func_entry)
{
	unsigned long flags;
	struct kfi_run* run;
	
	local_irq_save(flags);
	if (in_entry_exit) {
		local_irq_restore(flags);
		return;
	}
	in_entry_exit = 1;
	
	update_usecs_since_boot();
	
	run = run_curr;

	if (!run || run->complete) {
		goto entry_exit_byebye;
	}
	
	if (!run->triggered) {
		if (!(run->triggered = test_trigger(run, 1,
						    func_entry,
						    this_fn))) {
			goto entry_exit_byebye;
		}
	}

	if (!test_filters(&run->filters, this_fn)) {
		if (func_entry)
			do_func_entry(run, this_fn, call_site);
		else
			do_func_exit(run, this_fn, call_site);
	}

	if (!run->complete) {
		run->complete = test_trigger(run, 0,
					     func_entry,
					     this_fn);
	}

	if (run->complete) {
		if (waitqueue_active(&kfi_wait))
			wake_up_interruptible(&kfi_wait);
		if (run->next != NULL)
			run_curr = run->next;
	}
	
 entry_exit_byebye:
	in_entry_exit = 0;
	local_irq_restore(flags);
}


void __noinstrument __cyg_profile_func_enter (void *this_fn, void *call_site)
{
	func_entry_exit(this_fn, call_site, 1);
}

void __noinstrument __cyg_profile_func_exit (void *this_fn, void *call_site)
{
	func_entry_exit(this_fn, call_site, 0);
}


#define dump_str(buf, len, fmt, arg...) \
    if (buf) len += sprintf(buf + len, fmt, ## arg); \
    else len += printk(KERN_EMERG fmt, ## arg)


static struct kfi_run* __noinstrument find_run(int id)
{
	struct kfi_run* run = run_head;

	while (run && run->id != id)
		run = run->next;

	return run;
}

static int __noinstrument
print_trigger(char* buf, int len, struct kfi_trigger* t, int start_trigger)
{
	char trigbuf[80];
	
	switch (t->type) {
	case TRIGGER_DEV:
		sprintf(trigbuf, "system call\n");
		break;
	case TRIGGER_TIME:
		sprintf(trigbuf, "time at %lu usec from %s\n",
		       t->time, start_trigger ? "boot" : "start trigger");
		break;
	case TRIGGER_FUNC_ENTRY:
		sprintf(trigbuf, "entry to function 0x%08lx\n",
			(unsigned long)t->func_addr);
		break;
	case TRIGGER_FUNC_EXIT:
		sprintf(trigbuf, "exit from function 0x%08lx\n",
			(unsigned long)t->func_addr);
		break;
	case TRIGGER_LOG_FULL:
		sprintf(trigbuf, "log full\n");
		break;
	default:
		sprintf(trigbuf, "?\n");
		break;
	}

	dump_str(buf, len, "Logging %s at %lu usec by %s",
		 (start_trigger ? "started" : "stopped"),
		 t->mark, trigbuf);

	return len;
}

int __noinstrument kfi_dump_log(char* buf)
{
	int i, len = 0;
	struct kfi_run* run = run_curr;
	struct kfi_filters* filters = &run->filters;

	if (!run) {
		dump_str(buf, len, "\nNo logging run registered\n");
		return len;
	}

	if (!run->triggered) {
		dump_str(buf, len, "\nLogging not yet triggered\n");
		return len;
	}

	if (!run->complete) {
		dump_str(buf, len, "\nLogging is running\n");
		return len;
	}

	dump_str(buf, len, "\nKernel Instrumentation Run ID %d\n\n",
		 run->id);
	
	dump_str(buf, len, "Filters:\n");
	if (filters->func_list_size) {
		dump_str(buf, len, "\t%d-entry function list\n",
			 filters->func_list_size);
	}
	if (filters->min_delta) {
		dump_str(buf, len, "\t%ld usecs minimum execution time\n",
			 filters->min_delta);
	}
	if (filters->max_delta) {
		dump_str(buf, len, "\t%ld usecs maximum execution time\n",
			 filters->max_delta);
	}
	if (filters->no_ints) {
		dump_str(buf, len, "\tno functions in interrupt context\n");
	}
	if (filters->only_ints) {
		dump_str(buf, len,
			 "\tno functions NOT in interrupt context\n");
	}
	if (filters->func_list) {
		dump_str(buf, len, "\tfunction list\n");
	}
	
#ifdef KFI_DEBUG
	dump_str(buf, len, "\nFilter Counters:\n");

	if (filters->min_delta || filters->max_delta) {
		dump_str(buf, len, "\nExecution time filter count = %d\n",
			 filters->cnt.delta);
	}
	if (filters->no_ints) {
		dump_str(buf, len,
			 "No Interrupt functions filter count = %d\n",
			 filters->cnt.no_ints);
	}
	if (filters->only_ints) {
		dump_str(buf, len,
			 "Only Interrupt functions filter count = %d\n",
			 filters->cnt.only_ints);
	}
	if (filters->func_list_size) {
		dump_str(buf, len, "Function List filter count = %d\n",
			 filters->cnt.func_list);
	}
	dump_str(buf, len, "Total entries filtered = %d\n",
		 filters->cnt.delta +
		 filters->cnt.no_ints +
		 filters->cnt.only_ints +
		 filters->cnt.func_list);
	dump_str(buf, len, "Entries not found = %d\n", run->notfound);
#endif	
	dump_str(buf, len, "\nNumber of entries after filters = %d\n\n",
		 run->next_entry);

	len += print_trigger(buf, len, &run->start_trigger, 1);
	len += print_trigger(buf, len, &run->stop_trigger, 0);
	
	/* print out header */
	dump_str(buf, len, "\n");
	dump_str(buf, len,
		 " Entry      Delta       PID      Function    Caller\n");
	dump_str(buf, len,
		 "--------   --------   --------   --------   --------\n");

	for (i=0; i < run->next_entry; i++) {
		dump_str(buf, len, "%8lu   %8lu   %7d%s   %08x   %08x\n",
			 run->log[i].time,
			 run->log[i].delta,
			 run->log[i].pid,
			 (run->log[i].pid == INTR_CONTEXT) ? "i" : " ",
			 (unsigned int)run->log[i].va,
			 (unsigned int)run->log[i].call_site);
	}

	return len;
}

int __noinstrument kfi_read_proc(char *buf, char **start, off_t fpos,
				 int length, int *eof, void *data)
{
	int len = kfi_dump_log(buf);
	
	if (fpos >= len) {
		*start = buf;
		*eof = 1;
		return 0;
	}
	*start = buf + fpos;
	if ((len -= fpos) > length)
		return length;
	*eof = 1;
	return len;
}


static void __noinstrument kfi_reset(void)
{
	unsigned long flags;
	struct kfi_run* run;
	
#ifdef CONFIG_KFI_STATIC_RUN
	run = kfi_last_run->next;
#else
	run = run_head;
#endif

	local_irq_save(flags);
	if (run_curr)
		run_curr->complete = 1;
	local_irq_restore(flags);

	while (run) {
		struct kfi_run* tmp = run;
		kfree(run->log);
		if (run->filters.func_list)
			kfree(run->filters.func_list);
		run = run->next;
		kfree(tmp);
	}

	next_run_id = 0;
	run_head = run_tail = run_curr = NULL;
}


static int __noinstrument kfi_ioctl(struct inode *inode,
				    struct file *file,
				    unsigned int cmd,
				    unsigned long arg)
{
	unsigned long flags, kfitimer;
	struct kfi_run* run;
	struct kfi_run urun;
	struct kfi_entry* ulog;
	void** ufunc_list;
	int ufunc_list_size;
	
	switch (cmd) {
	case KFI_RESET:
		kfi_reset();
		break;
	case KFI_NEW_RUN:
		if (verify_area(VERIFY_READ, (void *)arg,
				sizeof(struct kfi_run)) ||
		    verify_area(VERIFY_WRITE, (void *)arg,
				sizeof(struct kfi_run)))
			return -EFAULT;
		if ((run = (struct kfi_run*)kmalloc(sizeof(struct kfi_run),
						    GFP_KERNEL)) == NULL)
			return -ENOMEM;
		copy_from_user(&urun, (struct kfi_run*)arg,
			       sizeof(struct kfi_run));
		if (urun.num_entries > MAX_RUN_LOG_ENTRIES) {
			kfree(run);
			return -EINVAL;
		}
		*run = urun;
		run->id = next_run_id++;
		run->triggered = run->complete = 0;
		run->next_entry = 0;
#ifdef KFI_DEBUG
		run->notfound = 0;
		memset(&run->filters.cnt, 0, sizeof(run->filters.cnt));
#endif
		run->next = NULL;
		urun = *run;
		run->log = (struct kfi_entry*)
			kmalloc(sizeof(struct kfi_entry) * run->num_entries,
				GFP_KERNEL);
		if (run->log == NULL) {
			kfree(run);
			return -ENOMEM;
		}
		memset(run->log, 0,
		       sizeof(struct kfi_entry) * run->num_entries);
		
		if (urun.filters.func_list) {
			int size;
			if (urun.filters.func_list_size >
			    MAX_FUNC_LIST_ENTRIES) {
				kfree(run->log);
				kfree(run);
				return -EINVAL;
			}
			size = urun.filters.func_list_size * sizeof(void*);
			if (verify_area(VERIFY_READ,
					(void *)urun.filters.func_list,
					size)) {
				kfree(run->log);
				kfree(run);
				return -EFAULT;
			}
			run->filters.func_list = (void**)kmalloc(size,
								 GFP_KERNEL);
			if (run->filters.func_list == NULL) {
				kfree(run->log);
				kfree(run);
				return -ENOMEM;
			}
			copy_from_user(run->filters.func_list,
				       urun.filters.func_list,
				       size);
		}
	       
		/* new run is ready, return it to user */
		copy_to_user((struct kfi_run*)arg, &urun,
			     sizeof(struct kfi_run));
		/* tack it on */
		local_irq_save(flags);
		if (!run_tail) {
			run_head = run_tail = run_curr = run;
		} else {
			if (run_curr == run_tail && run_curr->complete)
				run_curr = run;
			run_tail->next = run;
			run_tail = run;
		}
		local_irq_restore(flags);
		break;
	case KFI_START:
		local_irq_save(flags);
		run = run_curr;
		if (!run || run->complete || run->triggered) {
			local_irq_restore(flags);
			return -EINVAL;
		}
		run->triggered = 1;
		run->start_trigger.mark = update_usecs_since_boot();
		run->start_trigger.type = TRIGGER_DEV;
		local_irq_restore(flags);
		if (put_user(run->id, (int *)arg))
			return -EFAULT;
		break;
	case KFI_STOP:
		local_irq_save(flags);
		run = run_curr;
		if (!run || run->complete) {
			local_irq_restore(flags);
			return -EINVAL;
		}
		run->complete = 1;
		run->stop_trigger.mark = update_usecs_since_boot();
		run->stop_trigger.type = TRIGGER_DEV;
		if (run->next != NULL)
			run_curr = run->next;
		local_irq_restore(flags);
		if (waitqueue_active(&kfi_wait))
			wake_up_interruptible(&kfi_wait);
		if (put_user(run->id, (int *)arg))
			return -EFAULT;
		break;
	case KFI_READ:
	case KFI_READ_CURR:
		if (verify_area(VERIFY_READ, (void *)arg,
				sizeof(struct kfi_run)) ||
		    verify_area(VERIFY_WRITE, (void *)arg,
				sizeof(struct kfi_run)))
			return -EFAULT;
		copy_from_user(&urun, (struct kfi_run*)arg,
			       sizeof(struct kfi_run));

		local_irq_save(flags);

		run = (cmd == KFI_READ_CURR) ? run_curr : find_run(urun.id);
		if (!run) {
			local_irq_restore(flags);
			return -EINVAL;
		}

		if (urun.log != NULL) {
			if (urun.num_entries < run->num_entries) {
				local_irq_restore(flags);
				return -EINVAL;
			}
			
			if (!run->complete) {
				local_irq_restore(flags);
				if (file->f_flags & O_NONBLOCK)
					return -EAGAIN;
				while (!run->complete) {
					interruptible_sleep_on(&kfi_wait);
					if (signal_pending(current))
						return -ERESTARTSYS;
				}
				local_irq_save(flags);
			}
		}

		ufunc_list_size = urun.filters.func_list_size;

		// save user pointers
		ulog = urun.log;
		ufunc_list = urun.filters.func_list;
		
		urun = *run; // copy run

		// restore user pointers
		urun.log = ulog;
		urun.filters.func_list = ufunc_list;
		urun.next = NULL;

		local_irq_restore(flags);

		copy_to_user((void*)arg, &urun, sizeof(struct kfi_run));

		if (urun.log != NULL) {
			int size = run->next_entry * sizeof(struct kfi_entry);
			if (verify_area(VERIFY_WRITE, (void*)urun.log, size))
				return -EFAULT;
			copy_to_user((void*)urun.log, run->log, size);
		}

		if (ufunc_list != NULL && run->filters.func_list != NULL) {
			int size;
			if (ufunc_list_size < run->filters.func_list_size)
				return -EINVAL;
			size = run->filters.func_list_size * sizeof(void*);
			if (verify_area(VERIFY_WRITE, ufunc_list, size))
				return -EFAULT;
			copy_to_user(ufunc_list, run->filters.func_list, size);
		}
		break;
	case KFI_READ_TIMER:
		local_irq_save(flags);
		kfitimer = update_usecs_since_boot();
		local_irq_restore(flags);
		if (put_user(kfitimer, (unsigned long *)arg))
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static unsigned int __noinstrument
kfi_poll(struct file * filp, poll_table * wait)
{
        poll_wait(filp, &kfi_wait, wait);
        if (run_curr && run_curr->complete)
                return POLLIN | POLLRDNORM;
        return 0;
}

int __noinstrument kfi_open(struct inode *minode, struct file *mfile)
{
	MOD_INC_USE_COUNT;
	return 0;
}

int __noinstrument kfi_release(struct inode *minode, struct file *mfile)
{
	MOD_DEC_USE_COUNT;
	return 0;
}

static loff_t __noinstrument kfi_llseek(struct file *mfile,
					loff_t offset, int origint)
{
	return -ESPIPE;
}

static struct file_operations kfi_fops = {
	owner:		THIS_MODULE,
	llseek:		kfi_llseek,
	poll:           kfi_poll,
	ioctl:		kfi_ioctl,
	open:		kfi_open,
	release:	kfi_release,
};

static struct miscdevice kfi_miscdev = {
	KFI_MINOR,
	"kfi",
	&kfi_fops
};

static int __init __noinstrument kfi_init(void)
{
	int ret = misc_register(&kfi_miscdev);
	if (ret) {
		err("Register misc driver failed, errno is %d\n", ret);
		return ret;
	}

#ifdef CONFIG_KFI_STATIC_RUN
	run_tail = kfi_last_run;
	next_run_id = kfi_num_runs;
#endif

	create_proc_read_entry("kfi", 0, NULL,
			       kfi_read_proc, NULL);
	return 0;
}

static void __exit __noinstrument kfi_exit(void)
{
        remove_proc_entry("kfi", NULL);
        misc_deregister(&kfi_miscdev);
}


module_init(kfi_init);
module_exit(kfi_exit);

EXPORT_SYMBOL(__cyg_profile_func_enter);
EXPORT_SYMBOL(__cyg_profile_func_exit);

