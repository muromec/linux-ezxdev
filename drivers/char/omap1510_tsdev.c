/*
 * File: tsdev.c
 *
 * touchscreen input driver
 *
 * Created 2001, Copyright (C) 2001 RidgeRun, Inc.  All rights reserved.
 * Author: Greg Lonnon <glonnon@ridgerun.com>
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

#define MOUSEDEV_MINOR_BASE 	32
#define MOUSEDEV_MINORS		32
#define MOUSEDEV_MIX		31

#include <linux/poll.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/config.h>
#include <linux/smp_lock.h>
#include <linux/random.h>
#include <linux/slab.h>

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#  define DBG(fmt, args...) printk( "%s: " fmt, __FUNCTION__ , ## args)
#define ENTRY() DBG(":entry\n")
#else
#  define DBG(fmt, args...)
#define ENTRY()
#endif

#define RR_BUT_LEFT      (1<<0)

struct mouse_packet {
	int x;
	int y;
	int buttons;
};

struct mousedev {
	int exist;
	int open;
	int ready;
	wait_queue_head_t wait;
	struct input_handle handle;
	struct fasync_struct *fasync;
	struct mouse_packet packet;
	devfs_handle_t devfs;
	int bufsize;
	int x;
	int y;
	int but_left;
        int ts_x;
        int ts_y;
        int fb_x;
        int fb_y;
};

static void mousedev_packet(void);

static struct mousedev mouse;

static void mousedev_event(struct input_handle *handle, unsigned int type, 
                           unsigned int code, int value)
{
	struct mousedev *mouse =  handle->private;

	ENTRY();
	add_mouse_randomness((type << 4) ^ code ^ (code >> 4) ^ value);

	switch(type) {
	case EV_ABS:
		switch(code) {
		case ABS_X:
			mouse->x = value * mouse->fb_x/ mouse->ts_x;
			break;
		case ABS_Y:
			mouse->y = value * mouse->fb_y/mouse->ts_y;
			break;
			
		default:
			printk(__FUNCTION__ " wrong code for EV_ABS\n");
			break;
		}
		break;
	case EV_KEY:
		switch(code) {
		case BTN_LEFT:   mouse->but_left = value; break;
		default: printk(__FUNCTION__ " wrong code for EV_KEY\n");
		}
		break;

	default:
		printk(__FUNCTION__ " wrong type for mousedev %d\n",code);
	}
	mouse->ready = 1;
	kill_fasync(&mouse->fasync, SIGIO, POLL_IN);
	wake_up_interruptible(&(mouse->wait));
}

static int mousedev_fasync(int fd, struct file *file, int on)
{
	int retval;
	struct mousedev *mouse = file->private_data;

	ENTRY();
	retval = fasync_helper(fd, file, on, &mouse->fasync);
	return retval < 0 ? retval : 0;
}

static int mousedev_release(struct inode * inode, struct file * file)
{
	struct mousedev *mouse = file->private_data;
	ENTRY();
	lock_kernel();

	mousedev_fasync(-1, file, 0);

	if (!--mouse->open) {
		input_close_device(&mouse->handle);
	}
	unlock_kernel();

	return 0;
}

static int mousedev_open(struct inode * inode, struct file * file)
{
	file->private_data = &mouse;
	ENTRY();
	if (!mouse.open++) {
		struct input_handle *handle = &mouse.handle;
		input_open_device(handle);
	}
	return 0;
}

static void mousedev_packet(void)
{
	int old_buttons;

	ENTRY();
	old_buttons = mouse.packet.buttons;

        mouse.packet.buttons = 
		(mouse.but_left ? RR_BUT_LEFT : 0);

	if((mouse.x == mouse.packet.x) && (mouse.y == mouse.packet.y) && 
	   (old_buttons == mouse.packet.buttons))
		mouse.ready = 0;
	mouse.packet.x = mouse.x;
	mouse.packet.y = mouse.y;
        DBG("x %d y %d buttons %x\n",mouse.x,mouse.y,mouse.packet.buttons);
	mouse.bufsize = sizeof(mouse.packet);
}

static ssize_t mousedev_write(struct file * file, const char * buffer, 
                              size_t count, loff_t *ppos)
{
	ENTRY();
	printk(__FUNCTION__ "write not supported\n");
	return 0;
}

static ssize_t mousedev_read(struct file *file, char * buffer, size_t count, 
                             loff_t *ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	struct mousedev *mouse = file->private_data;
	int retval = 0;

	ENTRY();
	if (!mouse->ready && !mouse->bufsize) {

		add_wait_queue(&mouse->wait, &wait);
		current->state = TASK_INTERRUPTIBLE;

		while (!mouse->ready) {

			if (file->f_flags & O_NONBLOCK) {
				retval = -EAGAIN;
				break;
			}
			if (signal_pending(current)) {
				retval = -ERESTARTSYS;
				break;
			}

			schedule();
		}

		current->state = TASK_RUNNING;
		remove_wait_queue(&mouse->wait, &wait);
	}

	if (retval)
		return retval;

	mousedev_packet();
	if(count > mouse->bufsize)
		count = mouse->bufsize;

	if (copy_to_user(buffer, &mouse->packet, count))
		return -EFAULT;
	mouse->bufsize -= count;
	return count;	
}

/* No kernel lock - fine */
static unsigned int mousedev_poll(struct file *file, poll_table *wait)
{
	struct mousedev *mouse = file->private_data;

	poll_wait(file, &mouse->wait, wait);
	if (mouse->ready || mouse->bufsize)
		return POLLIN | POLLRDNORM;
	return 0;
}

static struct file_operations mousedev_fops = {
	owner:		THIS_MODULE,
	read:		mousedev_read,
	write:		mousedev_write,
	poll:		mousedev_poll,
	open:		mousedev_open,
	release:	mousedev_release,
	fasync:		mousedev_fasync,
};

static struct input_handle *mousedev_connect(struct input_handler *handler, 
                                             struct input_dev *dev)
{
	ENTRY();
	if (!test_bit(EV_KEY, dev->evbit) ||
	   (!test_bit(BTN_LEFT, dev->keybit) && !test_bit(BTN_TOUCH, dev->keybit)))
		return NULL;

	if ((!test_bit(EV_ABS, dev->evbit) || !test_bit(ABS_Y, dev->absbit)) &&
	    (!test_bit(EV_ABS, dev->evbit) || !test_bit(ABS_X, dev->absbit)))
		return NULL;

	DBG("mouse connected handler %p dev %s\n",handler,dev->name);
	mouse.handle.dev = dev;
	mouse.handle.handler = handler;
	mouse.handle.private = &mouse;
	mouse.devfs = input_register_minor("rrts%d", 0, MOUSEDEV_MINOR_BASE);
	DBG("handler %p\n",handler);
	input_open_device(&mouse.handle);
	DBG("exit");
	return &mouse.handle;
}

static void mousedev_disconnect(struct input_handle *handle)
{
	struct mousedev *mousedev = handle->private;

	ENTRY();
	mousedev->exist = 0;

	if (mousedev->open) {
		input_close_device(handle);
	} 
}
	
static struct input_handler mouse_handler = {
	event:		mousedev_event,
	connect:	mousedev_connect,
	disconnect:	mousedev_disconnect,
	fops:		&mousedev_fops,
	minor:		MOUSEDEV_MINOR_BASE,
};

static int mousedev_installed = 0;
static int __init mousedev_init(void)
{
	ENTRY();
	if(mousedev_installed++)
		return 0;
	memset(&mouse, 0, sizeof(struct mousedev));
        mouse.ts_x = mouse.ts_y = 1000;
        mouse.fb_x = 320;
        mouse.fb_y = 240;
	DBG("handler %p %d\n",mouse.handle,sizeof(struct mousedev));
	input_register_handler(&mouse_handler);
	init_waitqueue_head(&mouse.wait);
	input_register_minor("mice", MOUSEDEV_MIX, MOUSEDEV_MINOR_BASE);
	printk("mice: DSPLinux mouse device\n");

	return 0;
}

static void __exit mousedev_exit(void)
{
	ENTRY();
	if(!mousedev_installed--)
		input_unregister_handler(&mouse_handler);
}

module_init(mousedev_init);
module_exit(mousedev_exit);

MODULE_AUTHOR("Greg Lonnon <glonnon@rigderun.com>");
MODULE_DESCRIPTION("DSPLinux simulator mouse driver");
/*
 * ---------------------------------------------------------------------------
 * Local variables:
 * c-file-style: "linux"
 * End:
 */
