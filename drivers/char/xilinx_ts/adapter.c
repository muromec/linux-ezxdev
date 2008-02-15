/*
 * adapter.c
 *
 * Xilinx Adapter component to interface touchscreen component to Linux
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2002 (c) MontaVista, Software, Inc.  This file is licensed under the terms
 * of the GNU General Public License version 2.1.  This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */

/*
 * Parts of this code were written using drivers/misc/ucb1x00-ts.c as a
 * reference.  ucb1x00-ts.c is:
 *  Copyright (C) 2001 Russell King, All Rights Reserved.
 */

/*
 * This driver is a bit unusual in that it is composed of two logical
 * parts where one part is the OS independent code and the other part is
 * the OS dependent code.  Xilinx provides their drivers split in this
 * fashion.  This file represents the Linux OS dependent part known as
 * the Linux adapter.  The other files in this directory are the OS
 * independent files as provided by Xilinx with no changes made to them.
 * The names exported by those files begin with XTouchscreen_.  All
 * functions in this file that are called by Linux have names that begin
 * with xts_.  Any other functions are static helper functions.
 */

/*
 * CURRENT STATE OF AFFAIRS:
 * The underlying driver and hardware have some quirks that need to be
 * ironed out.  I've seen good X and Y values with zero Z's.  That is
 * why this driver synthesizes Z as either zero or 255 and uses
 * XTouchscreen_GetPosition_2D instead of XTouchscreen_GetPosition_3D.
 * There seem to be linearity problems and troubles with dead zones
 * around the edges of the screen.  I believe that this could be a
 * result of the issues outlined in the Burr-Brown (now Texas
 * Instruments) "Touch Screen Controller Tips" application note for the
 * ADS7846 touch screen controller which is used by the Xilinx ML300.
 * The Xilinx touchscreen controls the ADS7846, but it doesn't give
 * software enough access to the ADS7846 to implement the strategies
 * outlined in the aforementioned application note.  Xilinx is going to
 * look into improving this.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/input.h>
#include <linux/devfs_fs_kernel.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <xbasic_types.h>
#include "xtouchscreen.h"
#include "xtouchscreen_i.h"

MODULE_AUTHOR("MontaVista Software, Inc. <source@mvista.com>");
MODULE_DESCRIPTION("Xilinx touchscreen driver");
MODULE_LICENSE("GPL");

/* Dynamically allocate a major number. */
static int xts_major = 0;
#define XTS_NAME "xilinx_ts"

/*
 * The definition of the following structure is copied from ucb1x00-ts.c so the
 * touchscreen will "just work" with tslib, etc.
 */
/*
 * This structure is nonsense - millisecs is not very useful
 * since the field size is too small.  Also, we SHOULD NOT
 * be exposing jiffies to user space directly.
 */
struct ts_event {
	u16 pressure;
	u16 x;
	u16 y;
	u16 pad;
	struct timeval stamp;
};

#define NR_EVENTS	16	/* Must be a power of two */

/* Our private per interface data. */
struct xts_dev {
	struct xts_dev *next_dev;	/* The next device in dev_list */
	int index;		/* Which interface is this */
	u32 save_BaseAddress;	/* Saved physical address */
	unsigned int irq;	/* device IRQ number */
	devfs_handle_t devfs_handle;
	struct semaphore sem;
	int use_count;		/* How many things have us open */
	wait_queue_head_t read_wait;
	struct fasync_struct *fasync;
	struct ts_event events[NR_EVENTS];
	u8 evt_head;
	u8 evt_tail;
	/*
	 * The underlying OS independent code needs space as well.  A
	 * pointer to the following XTouchscreen structure will be
	 * passed to any XTouchscreen_ function that requires it.
	 * However, we treat the data as an opaque object in this file
	 * (meaning that we never reference any of the fields inside of
	 * the structure).
	 */
	XTouchscreen Touchscreen;
	/*
	 * The following bit fields are used to keep track of what
	 * all has been done to initialize the xts_dev to make
	 * error handling out of probe() easier.
	 */
	unsigned int remapped:1;	/* Has ioremap() been called? */
	unsigned int pen_is_down:1;
	unsigned int pen_was_down:1;	/* Used to determine pen-up */
};

/*
 * List of devices we're handling and a lock to give us atomic access in
 * anticipation of someday being able to hot add/remove.  If this is done, the
 * code that shoots the list in xts_thread will need to have protection added.
 */
static struct xts_dev *dev_list = NULL;
static spinlock_t dev_lock = SPIN_LOCK_UNLOCKED;

static devfs_handle_t devfs_dir = NULL;
struct task_struct *xts_task = NULL;	/* xts_thread task pointer */
static struct completion task_sync;	/* xts_thread start/stop syncing */
static char task_shutdown = 0;	/* Set to non-zero when task should quit.  */
static wait_queue_head_t irq_wait;	/* xts_thread waiting for interrupt */

#define event_pending(dev)	((volatile u8)(dev)->evt_head != (dev)->evt_tail)
#define event_get(dev)		((dev)->events + (dev)->evt_tail)
#define event_pull(dev)		((dev)->evt_tail = ((dev)->evt_tail + 1) & (NR_EVENTS - 1))
#define event_clear(dev)	((dev)->evt_head = (dev)->evt_tail = 0)

/*
 * Add a new touch event as long as either x, y or z was different
 * from the last touch event.
 */
static inline void
event_add(struct xts_dev *dev, u16 pressure, u16 x, u16 y)
{
	int next_head, prior_head;

	next_head = (dev->evt_head + 1) & (NR_EVENTS - 1);
	prior_head = (dev->evt_head - 1) & (NR_EVENTS - 1);
	if (next_head != dev->evt_tail
	    && (dev->events[prior_head].pressure != pressure
		|| dev->events[prior_head].x != x
		|| dev->events[prior_head].y != y))
	{
		dev->events[dev->evt_head].pressure = pressure;
		dev->events[dev->evt_head].x = x;
		dev->events[dev->evt_head].y = y;
		do_gettimeofday(&dev->events[dev->evt_head].stamp);
		dev->evt_head = next_head;

		if (dev->fasync)
			kill_fasync(&dev->fasync, SIGIO, POLL_IN);
		wake_up_interruptible(&dev->read_wait);
	}
}

/* SAATODO: This function will be moved into the Xilinx code. */
/*****************************************************************************/
/**
*
* Lookup the device configuration based on the touchscreen instance.  The table
* XTouchscreen_ConfigTable contains the configuration info for each device in
* the system.
*
* @param Instance is the index of the interface being looked up.
*
* @return
*
* A pointer to the configuration table entry corresponding to the given
* device ID, or NULL if no match is found.
*
* @note
*
* None.
*
******************************************************************************/
XTouchscreen_Config *
XTouchscreen_GetConfig(int Instance)
{
	if (Instance < 0 || Instance >= XPAR_XTOUCHSCREEN_NUM_INSTANCES) {
		return NULL;
	}

	return &XTouchscreen_ConfigTable[Instance];
}

/*
 * This routine is registered with the OS as the function to call when
 * the touchscreen interrupts.  It in turn, calls the Xilinx OS
 * independent interrupt function.  The Xilinx OS independent interrupt
 * function will in turn call any callbacks that we have registered for
 * various conditions.
 */
static void
xts_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct xts_dev *dev = dev_id;
	XTouchscreen_InterruptHandler(&dev->Touchscreen);
}

static void
Handler(void *CallbackRef, u32 Event, unsigned int EventData)
{
	struct xts_dev *dev = (struct xts_dev *) CallbackRef;

	switch (Event) {
	case XTOUCHSCREEN_EVENT_PEN_DOWN:
		dev->pen_is_down = 1;
		wake_up(&irq_wait);	/* Schedule our task */
		break;
	case XTOUCHSCREEN_EVENT_PEN_UP:
		dev->pen_is_down = 0;
		break;
	default:
		printk(KERN_ERR "%s #%d: Unknown event %d.\n", XTS_NAME,
		       dev->index, Event);
		break;
	}
}

/*
 * This task waits until at least one touchscreen is touched.  It then loops
 * digitizing and generating events until no touchscreens are being touched.
 */
static int
xts_thread(void *arg)
{
	int any_pens_down;
	struct xts_dev *dev;
	struct task_struct *tsk = current;
	DECLARE_WAITQUEUE(wait, tsk);
	xts_task = tsk;

	daemonize();
	reparent_to_init();
	strcpy(xts_task->comm, XTS_NAME);
	xts_task->tty = NULL;

	/* only want to receive SIGKILL */
	spin_lock_irq(&xts_task->sigmask_lock);
	siginitsetinv(&xts_task->blocked, sigmask(SIGKILL));
	recalc_sigpending(xts_task);
	spin_unlock_irq(&xts_task->sigmask_lock);

	complete(&task_sync);

	add_wait_queue(&irq_wait, &wait);
	any_pens_down = 0;
	for (;;) {
		/*
		 * Block waiting for interrupt or if any pens are down, either
		 * an interrupt or timeout to sample again.
		 */
		set_current_state(TASK_INTERRUPTIBLE);
		if (any_pens_down)
			schedule_timeout(HZ / 100);
		while (signal_pending(tsk)) {
			siginfo_t info;

			/* Only honor the signal if we're cleaning up */
			if (task_shutdown)
				goto exit;
			/*
			 * Someone else sent us a kill (probably the
			 * shutdown scripts "Sending all processes the
			 * KILL signal").  Just dequeue it and ignore
			 * it.
			 */
			spin_lock_irq(&current->sigmask_lock);
			(void)dequeue_signal(&current->blocked, &info);
			spin_unlock_irq(&current->sigmask_lock);
		}
		schedule();

		any_pens_down = 0;
		for (dev = dev_list; dev; dev = dev->next_dev) {
			if (dev->pen_is_down) {
				u32 x, y;
				XTouchscreen_GetPosition_2D(&dev->Touchscreen,
							    &x, &y);
				event_add(dev, 255, (u16) x, (u16) y);
				dev->pen_was_down = 1;
				any_pens_down = 1;
			} else if (dev->pen_was_down) {
				event_add(dev, 0, 0, 0);
				dev->pen_was_down = 0;
			}
		}
	}

exit:
	remove_wait_queue(&irq_wait, &wait);

	xts_task = NULL;
	complete_and_exit(&task_sync, 0);
}

static ssize_t
xts_read(struct file *filp, char *buffer, size_t count, loff_t * ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	struct xts_dev *dev = filp->private_data;
	char *ptr = buffer;
	int err = 0;

	add_wait_queue(&dev->read_wait, &wait);
	while (count >= sizeof (struct ts_event)) {
		err = -ERESTARTSYS;
		if (signal_pending(current))
			break;

		if (event_pending(dev)) {
			struct ts_event *evt = event_get(dev);

			err = copy_to_user(ptr, evt, sizeof (struct ts_event));
			event_pull(dev);

			if (err)
				break;

			ptr += sizeof (struct ts_event);
			count -= sizeof (struct ts_event);
			continue;
		}

		set_current_state(TASK_INTERRUPTIBLE);
		err = -EAGAIN;
		if (filp->f_flags & O_NONBLOCK)
			break;
		schedule();
	}
	current->state = TASK_RUNNING;
	remove_wait_queue(&dev->read_wait, &wait);

	return ptr == buffer ? err : ptr - buffer;
}
static unsigned int
xts_poll(struct file *filp, poll_table * wait)
{
	struct xts_dev *dev = filp->private_data;
	int ret = 0;

	poll_wait(filp, &dev->read_wait, wait);
	if (event_pending(dev))
		ret = POLLIN | POLLRDNORM;

	return ret;
}
static int
xts_open(struct inode *inode, struct file *filp)
{
	struct xts_dev *dev = filp->private_data;

	if (down_interruptible(&dev->sem))
		return -EINTR;

	if (dev->use_count++ == 0) {
		/* This was the first opener; we need to get the IRQ. */
		int retval;
		retval = request_irq(dev->irq, xts_interrupt, 0, XTS_NAME, dev);
		if (retval) {
			printk(KERN_ERR
			       "%s #%d: Could not allocate interrupt %d.\n",
			       XTS_NAME, dev->index, dev->irq);
			dev->use_count--;
			up(&dev->sem);
			return retval;
		}
	}

	up(&dev->sem);
	return 0;
}
static int
xts_release(struct inode *inode, struct file *filp)
{
	struct xts_dev *dev = filp->private_data;

	/* If dev is NULL, devfs must not be being used; find the right dev. */
	if (!dev) {
		dev = dev_list;
		while (dev && dev->index != MINOR(inode->i_rdev))
			dev = dev->next_dev;
		if (!dev)
			return -ENODEV;
		else
			filp->private_data = dev;
	}

	if (down_interruptible(&dev->sem))
		return -EINTR;

	if (--dev->use_count == 0) {
		/* This was the last closer; get rid of the IRQ. */
		disable_irq(dev->irq);
		free_irq(dev->irq, dev);
	}

	up(&dev->sem);
	return 0;
}
static int
xts_fasync(int fd, struct file *filp, int on)
{
	struct xts_dev *dev = filp->private_data;

	return fasync_helper(fd, filp, on, &dev->fasync);
}

static void
remove_head_dev(void)
{
	struct xts_dev *dev;
	XTouchscreen_Config *cfg;

	/* Pull the head off of dev_list. */
	spin_lock(&dev_lock);
	dev = dev_list;
	dev_list = dev->next_dev;
	spin_unlock(&dev_lock);

	if (dev->devfs_handle)
		devfs_unregister(dev->devfs_handle);

	if (dev->remapped) {
		cfg = XTouchscreen_GetConfig(dev->index);
		iounmap((void *) cfg->BaseAddress);
		cfg->BaseAddress = dev->save_BaseAddress;
	};

	kfree(dev);
}

static struct file_operations fops = {
	owner:THIS_MODULE,
	read:xts_read,
	poll:xts_poll,
	open:xts_open,
	release:xts_release,
	fasync:xts_fasync,
};

static void			/* __exit removed because xts_init calls this */
xts_cleanup(void)
{
	while (dev_list)
		remove_head_dev();

	if (xts_task) {
		task_shutdown = 1;
		send_sig(SIGKILL, xts_task, 1);
		wait_for_completion(&task_sync);
	}
	if (devfs_dir) {
		devfs_unregister(devfs_dir);
		devfs_dir = NULL;
	}
	if (devfs_unregister_chrdev(xts_major, XTS_NAME) != 0)
		printk(KERN_ERR "%s: unable to release major %d.\n",
		       XTS_NAME, xts_major);
}

static int __init
probe(int index)
{
	static const unsigned long remap_size
	    = XPAR_TOUCHSCREEN_0_HIGHADDR - XPAR_TOUCHSCREEN_0_BASEADDR + 1;
	struct xts_dev *dev;
	XTouchscreen_Config *cfg;
	unsigned int irq;
	char name[20];

	switch (index) {
#if defined(XPAR_INTC_0_TOUCHSCREEN_0_VEC_ID)
	case 0:
		irq = 31 - XPAR_INTC_0_TOUCHSCREEN_0_VEC_ID;
		break;
#if defined(XPAR_INTC_0_TOUCHSCREEN_1_VEC_ID)
	case 1:
		irq = 31 - XPAR_INTC_0_TOUCHSCREEN_1_VEC_ID;
		break;
#if defined(XPAR_INTC_0_TOUCHSCREEN_2_VEC_ID)
	case 2:
		irq = 31 - XPAR_INTC_0_TOUCHSCREEN_2_VEC_ID;
		break;
#if defined(XPAR_INTC_0_TOUCHSCREEN_3_VEC_ID)
#error Edit this file to add more devices.
#endif				/* 3 */
#endif				/* 2 */
#endif				/* 1 */
#endif				/* 0 */
	default:
		return -ENODEV;
	}

	/* Find the config for our device. */
	cfg = XTouchscreen_GetConfig(index);
	if (!cfg)
		return -ENODEV;

	/* Allocate the dev and zero it out. */
	dev = (struct xts_dev *) kmalloc(sizeof (struct xts_dev), GFP_KERNEL);
	if (!dev) {
		printk(KERN_ERR "%s #%d: Could not allocate device.\n",
		       XTS_NAME, index);
		return -ENOMEM;
	}
	memset(dev, 0, sizeof (struct xts_dev));
	dev->index = index;

	/* Make it the head of dev_list. */
	spin_lock(&dev_lock);
	dev->next_dev = dev_list;
	dev_list = dev;
	spin_unlock(&dev_lock);

	/* Change the addresses to be virtual; save the old ones to restore. */
	dev->save_BaseAddress = cfg->BaseAddress;
	cfg->BaseAddress = (u32) ioremap(dev->save_BaseAddress, remap_size);
	dev->remapped = 1;

	/* Tell the Xilinx code to bring this touchscreen interface up. */
	if (XTouchscreen_Initialize(&dev->Touchscreen,
				    cfg->DeviceId) != XST_SUCCESS) {
		printk(KERN_ERR "%s #%d: Could not initialize device.\n",
		       XTS_NAME, dev->index);
		remove_head_dev();
		return -ENODEV;
	}

	XTouchscreen_SetHandler(&dev->Touchscreen, Handler, (void *) dev);

	/* We'll do the request_irq in open. */
	dev->irq = irq;

	init_MUTEX(&dev->sem);
	init_waitqueue_head(&dev->read_wait);
	sprintf(name, "xilinx%u", index);
	dev->devfs_handle = devfs_register(devfs_dir, name, DEVFS_FL_DEFAULT,
					   xts_major, index,
					   S_IFCHR | S_IRUGO | S_IWUGO, &fops,
					   dev);

	printk(KERN_INFO "%s #%d at 0x%08X mapped to 0x%08X, irq=%d\n",
	       XTS_NAME, dev->index,
	       dev->save_BaseAddress, cfg->BaseAddress, dev->irq);

	return 0;
}

static int __init
xts_init(void)
{
	int index = 0;
	int retval;

	if ((retval = devfs_register_chrdev(xts_major, XTS_NAME, &fops)) < 0) {
		printk(KERN_ERR "%s: unable to register major %d.\n", XTS_NAME,
		       xts_major);
		return retval;
	}
	if (xts_major == 0)
		xts_major = retval;

	devfs_dir = devfs_mk_dir(NULL, "touchscreen", NULL);

	/* Try to initialize all the touchscreens. */
	while (probe(index++) == 0) ;

	if (index > 1) {
		/* We found at least one. */
		init_completion(&task_sync);
		init_waitqueue_head(&irq_wait);
		if ((retval = kernel_thread(xts_thread, 0, 0)) >= 0) {
			wait_for_completion(&task_sync);
			return 0;
		}
	} else
		retval = -ENODEV;

	/* None found or error starting thread, clean up and get out of here. */
	xts_cleanup();
	return retval;
}

EXPORT_NO_SYMBOLS;

module_init(xts_init);
module_exit(xts_cleanup);
