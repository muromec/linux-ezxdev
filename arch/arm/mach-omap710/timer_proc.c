/*
 * File: timer_proc.c
 *
 * /proc/timer
 *
 * DESCRIPTION                                                                   
 *    This provides a proc file interface to the kernel timer API. It creates
 *    /proc/timer and populates it with one subdirectory per system timer.
 *    Under each subdirectory it provides the following files:
 *    
 *    run -- read/write the timer status to "on" or "off"
 *    mode -- read/write the timer mode to "oneshot" or "periodic"
 *    period -- read/write the timer period as "<decimal num> <units>"
 *    count -- read the count as "<rollovers> <current count>"
 *
 * Copyright (C) 2001 RidgeRun, Inc.
 * Author: <author> <author-email> <date>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS  PROVIDED  ``AS  IS''  AND   ANY  EXPRESS  OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Please report all bugs/problems to the author or <support@dsplinux.net>
 *
 * key: RRGPLCR (do not remove)
 *
 */

/*****************************************************************************
         I n c l u d e s                                                      
 *****************************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <asm/arch/timer.h>

typedef struct {
	struct proc_dir_entry *subdir;
	unsigned long rollover;
} timer_proc_info_t;

static timer_proc_info_t *__timer_proc_subdirs;

static void
__timer_proc_cb(int timer, void *data)
{
	timer_proc_info_t *info = (timer_proc_info_t *) data;

	info->rollover++;
}

/*****************************************************************************
        r u n
 *****************************************************************************/
static int
__timer_proc_run_read(struct file *file, char *buf, size_t len, loff_t * offset)
{
	struct proc_dir_entry *pde;
	char kbuf[16];
	int ret = 0, timer;

	if ((len < 4) || file->f_pos)
		goto exit;

	pde = (struct proc_dir_entry *) file->f_dentry->d_inode->u.generic_ip;
	if (!pde)
		return -ENOTDIR;
	timer = (int) pde->data;

	if ((ret = timer_is_running(timer)) < 0)
		goto exit;

	len = sprintf(kbuf, "%s\n", (ret ? "on" : "off"));
	copy_to_user(buf, kbuf, len);

	ret = len;
	file->f_pos += len;
      exit:
	return ret;
}
static ssize_t
__timer_proc_run_write(struct file *file, const char *buf,
		       size_t len, loff_t * offset)
{
	struct proc_dir_entry *pde;
	char kbuf[16], *p;
	int ret = 0, timer;

	if (!len || file->f_pos)
		return 0;

	if (len < strlen("on"))
		return -EINVAL;

	if (len > sizeof (kbuf))
		len = sizeof (kbuf);

	pde = (struct proc_dir_entry *) file->f_dentry->d_inode->u.generic_ip;
	if (!pde)
		return -ENOTDIR;
	timer = (int) pde->data;

	copy_from_user(kbuf, buf, len);
	p = kbuf;

	if (strnicmp(kbuf, "on", strlen("on")) == 0) {
		if ((ret = timer_start(timer)) < 0)
			return ret;
		goto done;
	}

	if (strnicmp(kbuf, "off", strlen("off")) == 0) {
		if ((ret = timer_stop(timer)) < 0)
			return ret;
		goto done;
	}

	return -EINVAL;

      done:
	file->f_pos += len;
	return len;
}
static int
__timer_proc_run_permission(struct inode *inode, int unused)
{
	return 0;		/* REVISIT */
}

static struct file_operations __timer_proc_run_fops = {
	read:__timer_proc_run_read,
	write:__timer_proc_run_write
};

static struct inode_operations __timer_proc_run_iops = {
	permission:&__timer_proc_run_permission
};

/*****************************************************************************
        m o d e
 *****************************************************************************/
static int
__timer_proc_mode_read(struct file *file, char *buf, size_t len,
		       loff_t * offset)
{
	struct proc_dir_entry *pde;
	char kbuf[16];
	int ret = 0, timer;
	timer_mode_t mode;

	if (!len || file->f_pos)
		goto exit;

	pde = (struct proc_dir_entry *) file->f_dentry->d_inode->u.generic_ip;
	if (!pde) {
		ret = -ENOTDIR;
		goto exit;
	}
	timer = (int) pde->data;

	if ((ret = timer_get_mode(timer, &mode)) < 0)
		goto exit;

	switch (mode) {
	case timer_mode_oneshot:
		len = sprintf(kbuf, "oneshot\n");
		break;
	case timer_mode_periodic:
		len = sprintf(kbuf, "periodic\n");
		break;
	default:
		len = sprintf(kbuf, "???\n");
		break;
	}

	copy_to_user(buf, kbuf, len);
	file->f_pos += len;
	ret = len;

      exit:
	return ret;
}
static ssize_t
__timer_proc_mode_write(struct file *file, const char *buf,
			size_t len, loff_t * offset)
{
	struct proc_dir_entry *pde;
	char kbuf[16];
	int ret = 0, timer;

	if (!len || file->f_pos)
		goto exit;

	pde = (struct proc_dir_entry *) file->f_dentry->d_inode->u.generic_ip;
	if (!pde) {
		ret = -ENOTDIR;
		goto exit;
	}
	timer = (int) pde->data;

	if (len > sizeof (kbuf))
		len = sizeof (kbuf);

	copy_from_user(kbuf, buf, len);

	if (strnicmp(kbuf, "oneshot", strlen("oneshot")) == 0) {
		if ((ret = timer_set_mode(timer, timer_mode_oneshot)) < 0)
			goto exit;
		goto done;
	}

	if (strnicmp(kbuf, "periodic", strlen("periodic")) == 0) {
		if ((ret = timer_set_mode(timer, timer_mode_periodic)) < 0)
			goto exit;
		goto done;
	}

	ret = -EINVAL;
	goto exit;
      done:
	file->f_pos += len;
	ret = len;
      exit:
	return ret;
}
static struct file_operations __timer_proc_mode_fops = {
	read:__timer_proc_mode_read,
	write:__timer_proc_mode_write
};
#define __timer_proc_mode_iops __timer_proc_run_iops

/*****************************************************************************
        p e r i o d
 *****************************************************************************/

static int
__timer_proc_period_read(struct file *file, char *buf, size_t len,
			 loff_t * offset)
{
	struct proc_dir_entry *pde;
	char kbuf[32];
	int ret = 0, timer, klen;
	unsigned long val;

	if (!len || file->f_pos)
		goto exit;

	pde = (struct proc_dir_entry *) file->f_dentry->d_inode->u.generic_ip;
	if (!pde) {
		ret = -ENOTDIR;
		goto exit;
	}
	timer = (int) pde->data;

	if ((ret = timer_get_period(timer, &val)) < 0)
		goto exit;

	klen = sprintf(kbuf, "%ld\n", val);

	if (klen > len)
		klen = len;

	copy_to_user(buf, kbuf, klen);
	file->f_pos += klen;
	ret = klen;

      exit:
	return ret;
}
static ssize_t
__timer_proc_period_write(struct file *file, const char *buf,
			  size_t len, loff_t * offset)
{
	struct proc_dir_entry *pde;
	char kbuf[16], *p;
	int ret = 0, left, timer;
	unsigned long val = 0;

	if (!len || file->f_pos)
		goto exit;

	pde = (struct proc_dir_entry *) file->f_dentry->d_inode->u.generic_ip;
	if (!pde) {
		ret = -ENOTDIR;
		goto exit;
	}
	timer = (int) pde->data;

	if (len > sizeof (kbuf))
		len = sizeof (kbuf);

	copy_from_user(kbuf, buf, len);
	p = kbuf;
	left = len;

	while (*p && left-- && (*p >= '0') && (*p <= '9')) {
		val = (val * 10) + (*p - '0');
		p++;
	}

	if ((ret = timer_set_period(timer, val)) < 0)
		goto exit;

	len -= left;
	file->f_pos += len;
	ret = len;
      exit:
	return ret;
}
static struct file_operations __timer_proc_period_fops = {
	read:__timer_proc_period_read,
	write:__timer_proc_period_write
};
#define __timer_proc_period_iops __timer_proc_run_iops

/*****************************************************************************
        c o u n t
 *****************************************************************************/
static int
__timer_proc_count_read(struct file *file, char *buf, size_t len,
			loff_t * offset)
{
	struct proc_dir_entry *pde;
	char kbuf[32];
	int ret = 0, timer, klen;
	unsigned long val;

	if (!len || file->f_pos)
		goto exit;

	pde = (struct proc_dir_entry *) file->f_dentry->d_inode->u.generic_ip;
	if (!pde) {
		ret = -ENOTDIR;
		goto exit;
	}
	timer = (int) pde->data;

	klen = sprintf(kbuf, "%ld:", __timer_proc_subdirs[timer].rollover);

	if ((ret = timer_get_time_remaining(timer, &val)) < 0)
		goto exit;

	klen += sprintf(kbuf + klen, "%ld\n", val);

	if (klen > len)
		klen = len;

	copy_to_user(buf, kbuf, klen);
	file->f_pos += klen;
	ret = klen;

      exit:
	return ret;
}
static ssize_t
__timer_proc_count_write(struct file *file, const char *buf,
			 size_t len, loff_t * offset)
{
	struct proc_dir_entry *pde;
	int ret = 0, timer, flags;

	if (!len || file->f_pos)
		goto exit;

	pde = (struct proc_dir_entry *) file->f_dentry->d_inode->u.generic_ip;
	if (!pde) {
		ret = -ENOTDIR;
		goto exit;
	}
	timer = (int) pde->data;

	/* reset the rollover count */
	local_irq_save(flags);
	__timer_proc_subdirs[timer].rollover = 0;
	local_irq_restore(flags);

	file->f_pos += len;
	ret = len;
      exit:
	return ret;
}

static struct file_operations __timer_proc_count_fops = {
	read:__timer_proc_count_read,
	write:__timer_proc_count_write,
};
#define __timer_proc_count_iops __timer_proc_run_iops
/*****************************************************************************
        a c t i o n
 *****************************************************************************/
static int
__timer_proc_action_read(struct file *file, char *buf, size_t len,
			 loff_t * offset)
{
	struct proc_dir_entry *pde;
	char kbuf[16];
	int ret = 0, timer;
	timer_action_t action;

	if (!len || file->f_pos)
		goto exit;

	pde = (struct proc_dir_entry *) file->f_dentry->d_inode->u.generic_ip;
	if (!pde) {
		ret = -ENOTDIR;
		goto exit;
	}
	timer = (int) pde->data;

	if ((ret = timer_get_action(timer, &action)) < 0)
		goto exit;

	switch (action) {
	case timer_action_interrupt:
		len = sprintf(kbuf, "interrupt\n");
		break;
	case timer_action_reset:
		len = sprintf(kbuf, "reset\n");
		break;
	default:
		len = sprintf(kbuf, "???\n");
		break;
	}

	copy_to_user(buf, kbuf, len);
	file->f_pos += len;
	ret = len;

      exit:
	return ret;
}
static ssize_t
__timer_proc_action_write(struct file *file, const char *buf,
			  size_t len, loff_t * offset)
{
	struct proc_dir_entry *pde;
	char kbuf[16];
	int ret = 0, timer;

	if (!len || file->f_pos)
		goto exit;

	pde = (struct proc_dir_entry *) file->f_dentry->d_inode->u.generic_ip;
	if (!pde) {
		ret = -ENOTDIR;
		goto exit;
	}
	timer = (int) pde->data;

	if (len > sizeof (kbuf))
		len = sizeof (kbuf);

	copy_from_user(kbuf, buf, len);

	if (strnicmp(kbuf, "interrupt", strlen("interrupt")) == 0) {
		if ((ret = timer_set_action(timer, timer_action_interrupt)) < 0)
			goto exit;
		goto done;
	}

	if (strnicmp(kbuf, "reset", strlen("reset")) == 0) {
		if ((ret = timer_set_action(timer, timer_action_reset)) < 0)
			goto exit;
		goto done;
	}

	ret = -EINVAL;
	goto exit;
      done:
	file->f_pos += len;
	ret = len;
      exit:
	return ret;
}
static struct file_operations __timer_proc_action_fops = {
	read:__timer_proc_action_read,
	write:__timer_proc_action_write
};
#define __timer_proc_action_iops __timer_proc_run_iops

/*****************************************************************************
         D i r e c t o r i e s
 *****************************************************************************/
static struct proc_dir_entry *__timer_proc_dir;

static int
__timer_proc_create_directory(void)
{
	__timer_proc_dir = create_proc_entry("timer", S_IFDIR, &proc_root);

	if (!__timer_proc_dir)
		return -ENOMEM;
	return 0;
}
static int
__timer_proc_create_entry(int timer, struct proc_dir_entry *p,
			  char *name, struct inode_operations *iops,
			  struct file_operations *fops)
{
	struct proc_dir_entry *pde;

	pde = create_proc_entry(name, S_IFREG, p);

	if (!pde)
		return -ENOMEM;

	pde->data = (void *) timer;
	pde->proc_fops = fops;
	pde->proc_iops = iops;

	return 0;
}
static int
__timer_proc_create_subdir(int timer)
{
	struct proc_dir_entry *pde;
	char buf[3];
	int feats;

	sprintf(buf, "%d", timer);

	pde = create_proc_entry(buf, S_IFDIR, __timer_proc_dir);
	if (!pde)
		return -ENOMEM;

	__timer_proc_subdirs[timer].subdir = pde;

	pde->data = (void *) timer;

	__timer_proc_create_entry(timer, pde, "run",
				  &__timer_proc_run_iops,
				  &__timer_proc_run_fops);

	__timer_proc_create_entry(timer, pde, "mode",
				  &__timer_proc_mode_iops,
				  &__timer_proc_mode_fops);

	__timer_proc_create_entry(timer, pde, "period",
				  &__timer_proc_period_iops,
				  &__timer_proc_period_fops);

	__timer_proc_create_entry(timer, pde, "count",
				  &__timer_proc_count_iops,
				  &__timer_proc_count_fops);

	timer_get_features(timer, &feats);
	if (feats & TIMERF_RST)
		__timer_proc_create_entry(timer, pde, "action",
					  &__timer_proc_action_iops,
					  &__timer_proc_action_fops);

	return 0;		/* REVISIT -- need to do some error cleanup here */
}
static void
__timer_proc_remove_subdir(int timer)
{
	struct proc_dir_entry *pde;
	char buf[3];

	pde = __timer_proc_subdirs[timer].subdir;

	remove_proc_entry("run", pde);
	remove_proc_entry("mode", pde);
	remove_proc_entry("period", pde);
	remove_proc_entry("count", pde);

	sprintf(buf, "%d", timer);
	remove_proc_entry(buf, __timer_proc_dir);
}

/*****************************************************************************
         M o d u l e                                                          
 *****************************************************************************/
int
init_module(void)
{
	int ret = 0, timer, n;

	printk("DSPLinux timer_proc (c) 2001 RidgeRun, Inc\n");

	if ((ret = __timer_proc_create_directory()) < 0)
		goto exit;

	if ((n = timer_get_num_timers()) <= 0)
		goto exit;

	__timer_proc_subdirs = kmalloc(n * sizeof (timer_proc_info_t),
				       GFP_KERNEL);
	memset(__timer_proc_subdirs, 0, n * sizeof (timer_proc_info_t));

	if (!__timer_proc_subdirs) {
		ret = -ENOMEM;
		goto bail;
	}

	for (timer = 0; timer < n; timer++) {
		__timer_proc_create_subdir(timer);
		timer_register_cb(timer, __timer_proc_cb,
				  (void *) &__timer_proc_subdirs[timer]);
	}

	goto exit;
      bail:
	remove_proc_entry("timer", &proc_root);
      exit:
	return ret;
}

void
cleanup_module(void)
{
	int n, timer;

	if ((n = timer_get_num_timers()) > 0) {
		for (timer = 0; timer < timer_get_num_timers(); timer++) {
			timer_unregister_cb(timer,
					    (void *)
					    &__timer_proc_subdirs[timer]);
			__timer_proc_remove_subdir(timer);
		}
		kfree(__timer_proc_subdirs);
	}

	remove_proc_entry("timer", &proc_root);
}
