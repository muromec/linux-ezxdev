/*
 * Device driver for controlling LEDs.  See linux/led.h for info on
 * how to use this.
 *
 * This currently only supports mono and bicolor LEDs, but support for
 * fancier LEDs (scrolling text or numeric LEDs, for instance) could
 * easily be added.
 *
 * Corey Minyard <minyard@mvista.com>
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <linux/init.h>
#include <linux/led.h>
#include <linux/string.h>
#include <linux/spinlock.h>

#define VERSION "1.0"

/* A lock that is held when manipulating LED info. */
static spinlock_t led_lock = SPIN_LOCK_UNLOCKED;

/* A linked list of all the LEDs registered. */
static struct led_reg_info *leds;

/* Register a new LED with the driver. */
int register_led_info(struct led_reg_info *info)
{
	struct led_reg_info *led;
	int		    rv = 0;

	spin_lock(&led_lock);
	info->next = NULL;
	led = leds;
	if (led == NULL)
	{
		leds = info;
	}
	else
	{
		while (led->next != NULL)
		{
			if (strcmp(led->info->name, info->info->name) == 0)
			{
				/* Registering a duplicate. */
				rv = -1;
				goto out;
			}
			led = led->next;
		}
		led->next = info;
	}
	MOD_INC_USE_COUNT;

out:
	spin_unlock(&led_lock);

	return rv;
}

/* Unregister an LED from the driver. */
int unregister_led_info(struct led_reg_info *info)
{
	int                 rv = -1;
	struct led_reg_info *led;

	if (info == NULL)
	{
		return -1;
	}

	spin_lock(&led_lock);
	led = leds;
	if (led == info)
	{
	        MOD_DEC_USE_COUNT;
		leds = leds->next;
	}
	else
	{
		while (led->next != NULL)
		{
			if (led->next == info)
			{
				MOD_DEC_USE_COUNT;
				led->next = info->next;
				rv = 0;
				goto out;
			}
			led = led->next;
		}
	}
out:
	spin_unlock(&led_lock);
	return rv;
}

/* Return the number of LEDs registered. */
static int count_leds(void)
{
	struct led_reg_info *led;
	int                 count = 0;

	spin_lock(&led_lock);
	led = leds;
	while (led != NULL)
	{
		count = count + 1;
		led = led->next;
	}
	spin_unlock(&led_lock);

	return count;
}

/* Find an LED by its number.  Must be called with the led spin lock
   held. */
static struct led_reg_info *get_led_by_num(int num)
{
	struct led_reg_info *led;
	int                 count = num;

	led = leds;
	while (led != NULL)
	{
		if (count == 0)
		{
			led->info->led_num = num;
			return led;
		}
		count = count - 1;
		led = led->next;
	}

	return NULL;
}

/* Find an LED by its name.  Must be called with the led spin lock
   held. */
static struct led_reg_info *get_led_by_name(char *name)
{
	struct led_reg_info *led;
	int                 count = 0;

	led = leds;
	while (led != NULL)
	{
		if (strcmp(name, led->info->name) == 0)
		{
			led->info->led_num = count;
			return led;
		}
		count++;
		led = led->next;
	}

	return NULL;
}

/* The IOCTL handler for the LED driver. */
static int led_ioctl(struct inode  *inode,
		     struct file   *file,
		     unsigned int  cmd,
		     unsigned long arg)
{
	int i;
	struct led_reg_info *led;
	struct led_info     info;
	struct led_op       op;

	switch(cmd)
	{
	case LEDIOC_GETCOUNT:
		return count_leds();

	case LEDIOC_GETINFO_BY_NUM:
	case LEDIOC_GETINFO_BY_NAME:
		i = copy_from_user(&info, (void*)arg, sizeof(info));
		if (i)
		{
			return -EFAULT;
		}

		spin_lock(&led_lock);
		if (cmd == LEDIOC_GETINFO_BY_NUM)
			led = get_led_by_num(info.led_num);
		else
			led = get_led_by_name(info.name);
		if (led == NULL)
		{
			spin_unlock(&led_lock);
			return -EINVAL;
		}

		info.type = led->info->type;
		info.led_num = led->info->led_num;
		strcpy(info.name, led->info->name);
		info.info = led->info->info;
		i = copy_to_user((void *)arg, &info, sizeof(info));

		spin_unlock(&led_lock);

		if (i)
		{
			return -EFAULT;
		}
		return 0;

	case LEDIOC_OP:
		i = copy_from_user(&op, (void*)arg, sizeof(op));
		if (i)
		{
			return -EFAULT;
		}

		spin_lock(&led_lock);
		led = get_led_by_name(op.name);
		if (led == NULL)
		{
			spin_unlock(&led_lock);
			return -EINVAL;
		}

		i = led->handle_led_op(led, &op);
		if (!i)
		{
		    i = copy_to_user((void *)arg, &op, sizeof(op));
		    if (i)
		    {
			i = -EFAULT;
		    }
		}
		spin_unlock(&led_lock);

		return i;

	default:
		return -ENOIOCTLCMD;
	}
}

/* Not much to do for opening. */ 
static int led_open(struct inode *inode, struct file *file)
{
	switch(MINOR(inode->i_rdev))
	{
		case LED_MINOR:
			return 0;

		default:
			return -ENODEV;
	}
}

/* Closing is really easy. */ 
static int led_close(struct inode *inode, struct file *file)
{
	return 0;
}

 
static struct file_operations led_fops = {
	owner:		THIS_MODULE,
	llseek:		NULL,
	read:		NULL,
	write:		NULL,
	ioctl:		led_ioctl,
	open:		led_open,
	release:	led_close,
};

static struct miscdevice led_miscdev=
{
	LED_MINOR,
	"led",
	&led_fops
};


/* Remove the LED driver. */
static void __exit led_exit(void)
{
	misc_deregister(&led_miscdev);
}

/* Set up the LED device driver. */
static int __init led_init(void)
{
	int ret;

	printk("generic LED driver: v%s Corey Minyard (minyard@mvista.com)\n",
	       VERSION);

	leds = NULL;
	ret = misc_register(&led_miscdev);
	if (ret) {
		printk(KERN_ERR "led: can't misc_register on minor=%d\n",
		       LED_MINOR);
		return -1;
	}
	return 0;
}

module_init(led_init);
module_exit(led_exit);

EXPORT_SYMBOL(register_led_info);
EXPORT_SYMBOL(unregister_led_info);

MODULE_LICENSE("GPL");
