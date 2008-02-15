/*
 * drivers/dpm/trace.c  Dynamic Power Management Tracing
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (C) 2002, International Business Machines Corporation
 * All Rights Reserved
 *
 * Bishop Brock
 * IBM Research, Austin Center for Low-Power Computing
 * bcbrock@us.ibm.com
 * October, 2002
 *
 */
/*
  DPM event tracing is a configuration option.  Tracing DPM events incurs a
  non-trivial overhead in the DPM system which may impact critical latencies in
  certain cases.  This tracing facility is for debugging and development
  _only_, and has known "bugs" that would effect production use.

  Traces are collected and stored in a fixed-length circular buffer.  Each
  traced event creates an entry in this buffer, as long as the event type is
  allowed by the global dpm_trace_allow mask.  Reading /proc/driver/dpm/trace
  displays the traces.

  Bugs:

  In-memory trace structures may contain pointers to DPM components which may
  have already been freed at the time that traces are displayed. In a system
  where DPM objects are created and deleted often (as opposed to just once at
  initialization) this might lead to a system crash.

  Tracing is halted when traces are displayed by the /proc interface (rather
  than e.g., locking DPM during the interval).  If sombody comes along and
  restarts tracing during this period then things could get out of sync.
  There is no concept of exclusive ownership of the trace buffer or the tracing
  subsystem. 

  The /proc interface only returns one PAGE of formatted trace data.
*/

#include <linux/dpm.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <asm/semaphore.h>
#include <asm/uaccess.h>

#define DPM_TRACEBUF_LEN 1048	/* Must be > 1! */
#define DPM_TRACE_PARAMS    4

/* A classic circular buffer, 'empty' when the read-pointer is equal to the
   write-pointer. On overflow we silently overwrite the oldest entries. */

struct dpm_trace {
	unsigned event;		/* Event type */
	pid_t pid;		/* PID of process making request */
	dpm_md_time_t time;	/* Timestamp */
	unsigned long params[DPM_TRACE_PARAMS]; /* Event-specific params */
};

static struct dpm_trace dpm_tracebuf[DPM_TRACEBUF_LEN];

static volatile unsigned rp = 0;
static volatile unsigned wp = 0;

static int dpm_trace_enabled = 0;
static unsigned dpm_trace_allow = 0;
static int dpm_trace_halted = 0;

static spinlock_t dpm_trace_lock = SPIN_LOCK_UNLOCKED;
static DECLARE_MUTEX(dpm_trace_sem);


static struct dpm_trace *
bump_rp(void)
{
	struct dpm_trace *t = &dpm_tracebuf[rp];

	if (rp == wp)
		return NULL;
	if (++rp >= DPM_TRACEBUF_LEN)
		rp = 0;
	return t;
}


static struct dpm_trace *
bump_wp(void)
{
	struct dpm_trace *t = &dpm_tracebuf[wp];

	if (++wp >= DPM_TRACEBUF_LEN)
		wp = 0;
	if (rp == wp)
		bump_rp();
	return t;
}


static struct dpm_trace *
unbump_wp(void)
{
	if (rp == wp)
		return NULL;
	if (wp == 0)
		wp = DPM_TRACEBUF_LEN - 1;
	else
		wp--;
	return &dpm_tracebuf[wp];
}


void
dpm_trace(unsigned event, ...)
{
	struct dpm_trace *t;
	unsigned long flags;
	va_list list;
	int i;

	if (!dpm_enabled)
		return;

	spin_lock_irqsave(dpm_trace_lock, flags);
	if (dpm_trace_halted || !dpm_trace_enabled || 
	    !(event & dpm_trace_allow)) {
		spin_unlock_irqrestore(dpm_trace_lock, flags);
		return;
	}

	t = bump_wp();

	t->event = event;
	t->pid = current->pid;
	t->time = dpm_md_time();

	i = 0;
	va_start(list, event);
	switch (event) {

		/* Four args */

	case DPM_TRACE_CONSTRAINT_ASSERTED: /* pp id, min, max, opt */
		t->params[i++] = va_arg(list, unsigned long);

		/* Three args */

	case DPM_TRACE_SET_TASK_STATE: /* pid, task_state, ret */
		t->params[i++] = va_arg(list, unsigned long);
		
		/* Two args */

	case DPM_TRACE_SET_POLICY: /* name, ret */
		t->params[i++] = va_arg(list, unsigned long);
		
		/* One arg */

	case DPM_TRACE_SET_OS:	/* state */
	case DPM_TRACE_SET_OPT_ASYNC: /* opt */
	case DPM_TRACE_SET_OPT_SYNC: /* opt */
	case DPM_TRACE_UNLOCK:	/* needs_resync */
		t->params[i++] = va_arg(list, unsigned long);

		/* No args */

	case DPM_TRACE_RESYNC:
	case DPM_TRACE_START:
	case DPM_TRACE_STOP:
		break;
	default:
		printk(KERN_ERR "dpm_trace: ?? 0x%08x ??\n", event);
		break;
	}
	va_end(list);

	spin_unlock_irqrestore(dpm_trace_lock, flags);
}


void
dpm_trace_start(unsigned events)
{
	struct dpm_trace *t;
	unsigned long flags;

	spin_lock_irqsave(dpm_trace_lock, flags);

	dpm_trace_allow = events;

	if (!dpm_trace_halted && (dpm_trace_allow & DPM_TRACE_START)) {
		t = bump_wp();
		t->event = DPM_TRACE_START;
		t->pid = current->pid;
		t->time = dpm_md_time();
		t->params[0] = (unsigned long)events;
	}

	dpm_trace_enabled = 1;

	spin_unlock_irqrestore(dpm_trace_lock, flags);
}
  

void
dpm_trace_stop(void)
{
	struct dpm_trace *t;
	unsigned long flags;

	spin_lock_irqsave(dpm_trace_lock, flags);

	if (!dpm_trace_halted && (dpm_trace_allow & DPM_TRACE_STOP)) {
		t = bump_wp();
		t->event = DPM_TRACE_STOP;
		t->pid = current->pid;
		t->time = dpm_md_time();
		t->params[0] = 0;
	}

	dpm_trace_enabled = 0;

	spin_unlock_irqrestore(dpm_trace_lock, flags);
}
  

void
dpm_trace_reset(void)
{
	unsigned long flags;

	spin_lock_irqsave(dpm_trace_lock, flags);

	dpm_trace_enabled = 0;
	wp = rp = 0;

	spin_unlock_irqrestore(dpm_trace_lock, flags);
}
  

static void
_dpm_trace_halt(int i)
{
	unsigned long flags;
	spin_lock_irqsave(dpm_trace_lock, flags);
	dpm_trace_halted = i;
	spin_unlock_irqrestore(dpm_trace_lock, flags);
}

#define dpm_trace_halt() _dpm_trace_halt(1)
#define dpm_trace_resume() _dpm_trace_halt(0)
	

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * /proc/driver/dpm/trace (Read/Write)
 *
 * Read:  
 *
 * Display recent trace activity.  Trace entries are listed from the most
 * recent entry to the least recent entry. The format is:
 *
 * t_offset, pid: event
 *
 * Where t_offset is the 32-bit time offset from the previous entry, pid is
 * current->pid when the change was made, and the information for each event is
 * event-specific.
 *
 * Write: 
 *
 * Issue DPM tracing commands:
 *
 * reset         : Call dpm_trace_reset()
 * start <mask>  : Call dpm_trace_start(mask) - No mask == DPM_TRACE_ALL
 * stop          : Call dpm_trace_stop
 * filter <mask> : Set a mask to be used as a filter for the read proc.  The
 *                 default is DPM_TRACE_ALL.  The filter stays in effect until
 *                 changed by this call.
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#ifdef CONFIG_PROC_FS
static unsigned dpm_read_proc_filter = DPM_TRACE_ALL;

int
read_proc_dpm_trace(char *page, char **start, off_t offset, 
		    int count, int *eof, void *data)
{
	int len = 0, max = PAGE_SIZE - (2 * DPM_NAME_SIZE);
	unsigned save_rp, save_wp;
	struct dpm_trace *t;
	struct dpm_opt *opt;
	dpm_md_time_t now, t_offset;

	if (down_interruptible(&dpm_trace_sem))
		return -ERESTARTSYS;

	dpm_trace_halt();

	if (wp == rp) {
		len += sprintf(page + len, "Trace buffer is empty\n");
		dpm_trace_resume();
		up(&dpm_trace_sem);
		*eof = 1;
		return len;
	}

	save_rp = rp;
	save_wp = wp;

	if (!dpm_enabled)
		len += sprintf(page + len, "DPM is currently disabled\n");

	now = dpm_md_time();
	len += sprintf(page + len, "Time now is 0x%08x%08x (%d Hz)\n",
		       (u32)((now >> 32) & 0xffffffffU),
		       (u32)(now & 0xffffffffU),		       
		       DPM_MD_HZ);

	while ((t = unbump_wp()) != NULL) {
		
		if (!(t->event & dpm_read_proc_filter))
			continue;

		t_offset = now - t->time;
		now = t->time;
		if (t_offset != (u32)t_offset) {
			len += sprintf(page + len, 
				       "Now = 0x%08x%08x\n", 
				       (u32)((now >> 32) & 0xffffffffU),
				       (u32)(now & 0xffffffffU));
			t_offset = 0;
		}
		len += sprintf(page + len, "%10u, %6d: ",
			       (u32)t_offset, t->pid);

		switch (t->event) {

		case DPM_TRACE_SET_OPT_ASYNC:
			len += sprintf(page + len, "set_opt_async: ");
			if (t->params[0]) {
				opt = (struct dpm_opt *)t->params[0];
				len += sprintf(page + len, "%s", opt->name);
			} else
				len += sprintf(page + len, "No change");
			break;

		case DPM_TRACE_SET_OPT_SYNC:
			len += sprintf(page + len, "set_opt_sync: ");
			if (t->params[0]) {
				opt = (struct dpm_opt *)(t->params[0]);
				len += sprintf(page + len, "%s", opt->name);
			} else
				len += sprintf(page + len, "No change");
			break;

		case DPM_TRACE_RESYNC:
			len += sprintf(page + len, "resync");
			break;

		case DPM_TRACE_UNLOCK:
			len += sprintf(page + len, "unlock%s",
				       t->params[0] ? ", resync needed" : "");
			break;
			
		case DPM_TRACE_SET_OS:
			len += sprintf(page + len, "set_os(%s)",
				       t->params[0] == DPM_NO_STATE ?
				       "NO STATE" : 
				       dpm_state_names[t->params[0]]);
			break;

		case DPM_TRACE_SET_POLICY:
			len += sprintf(page + len, "set_policy(%s)",
				       (char *)(t->params[0]));
			if (t->params[1])
				len += sprintf(page + len, ", failed (%d)",
					       (int)(t->params[1]));
			break;

		case DPM_TRACE_SET_TASK_STATE:
			len += sprintf(page + len, "set_task_state(%d, %d)",
				       (int)(t->params[0]), 
				       (int)(t->params[1]));
			if (t->params[2])
				len += sprintf(page + len, ", failed (%d)",
					       (int)(t->params[2]));
			break;

		case DPM_TRACE_CONSTRAINT_ASSERTED:
			/* pid is arg[4]; will be automatically printed */
			len += sprintf(page + len,
				       "constraint asserted: param %d (%s), value=%d, min/max %d/%d",
				       (int)(t->params[0]),
				       dpm_opp_names[(int)(t->params[0])],
				       (int)(t->params[1]),
				       (int)(t->params[2]),
				       (int)(t->params[3]));
			break;

		case DPM_TRACE_START:
			len += sprintf(page + len, "Start tracing");
			break;

		case DPM_TRACE_STOP:
			len += sprintf(page + len, "Stop tracing");
			break;

		default:
			len += sprintf(page + len, "?? Event 0x%08x ??",
				       t->event);
			break;
		}
		len += sprintf(page + len, "\n");
		if (len > max)
			break;
	}
	if (t != NULL)
		len += sprintf(page + len, "More ...\n");

	rp = save_rp;
	wp = save_wp;

	dpm_trace_resume();
	up(&dpm_trace_sem);

	*eof = 1;
	return len;
}

int 
write_proc_dpm_trace(struct file *file, const char *buffer,
		     unsigned long count, void *data)
{
	char *buf, *tok, *s;
	char *whitespace = " \t\r\n";
	int ret = 0, do_filter = 0;
	unsigned mask;

	if (current->uid != 0)
		return -EACCES;
	if (count == 0)
		return 0;
	if (!(buf = kmalloc(count + 1, GFP_KERNEL)))
		return -ENOMEM;
	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}
	buf[count] = '\0';
	s = buf + strspn(buf, whitespace);
	tok = strsep(&s, whitespace);
	
	if (strcmp(tok, "reset") == 0) {
		dpm_trace_reset();
	} else if (strcmp(tok, "stop") == 0) {
		dpm_trace_stop();
	} else if ((strcmp(tok, "start") == 0) ||
		   (do_filter = (strcmp(tok, "filter") == 0))) {
		s = s + strspn(s, whitespace);
		tok = strsep(&s, whitespace);

		if (*tok == '\0')
			mask = DPM_TRACE_ALL;
		else
			mask = simple_strtol(tok, NULL, 0);

		if (do_filter)
			dpm_read_proc_filter = mask;
		else
			dpm_trace_start(mask);

	} else {
		ret = -EINVAL;
	}
	kfree(buf);
	if (ret == 0)
		return count;
	else 
		return ret;
}
#endif /* CONFIG_PROC_FS */
  
/*
 * Local variables:
 * c-basic-offset: 8
 * End:
 */

