/*----------------------------------------------------------------------------
 *  mx21-keyp.c
 *  Motorola i.MX21 ADS on-board keypad driver
 *
 *  Author: MontaVista Software, Inc. <source@mvista.com>
 *  Copyright (C) 2005 MontaVista Software, Inc.
 *
 *  Heavily based on the keypad driver from Metroweks BSP
 *  Copyright © 2004 Metrowerks Corp.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *--------------------------------------------------------------------------*/
/*
 * This driver can be configured to use either
 * - event input interface (default)
 * - misc device interface (like in Metrowerks BSP)
 */

#include <linux/module.h>
#include <linux/config.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/input.h>	/* Keyboard mappings */
#include <linux/fs.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/arch/hardware.h>
#include <asm/arch/irqs.h>
#include <asm/arch/gpio.h>
#include <linux/pm.h>
#include <asm/arch/pll.h>

#if !defined (CONFIG_MX2ADS_KEYPAD_MISC) && !defined (CONFIG_MX2ADS_KEYPAD_INPUT)
#define CONFIG_MX2ADS_KEYPAD_INPUT 1
#endif

#define MODULE_NAME "keypad"
#define PKMOD "mx21-keyp: "

#define KEYPAD_MINOR 22		/* misc minor device number */
			 /* the misc major device number is 10 */
#define QUEUE_SIZE 16		/* size of the keycode queue plus one */
#define PRESSED_FLAG 0x8000

/* row and column masks
 * The hardware supports up to 8 row lines and 8 column lines.
 * Some of the upper row and column pins are shared with the
 * uart. This defines how many bits to use for 
 * keypad row and column lines */
#define NUM_ROW_LINES 6
#define NUM_COL_LINES 6
#define ROW_MASK (0xff >> (8 - NUM_ROW_LINES))
#define COL_MASK ((0xff00 >> (8 - NUM_COL_LINES)) & 0xff00)

/* key sample period */
#define SAMPLE_PERIOD ((2*HZ)/100)	/* in jiffies */

#define KPP_KPSR_KPP_EN		0x0400	/* kpp clock gating enable */
#define KPP_KRIE_BIT_MASK		0x0200	/* kpp release interrupt enable */
#define KPP_KPSR_KDIE		0x0100	/* kpp depress interrupt enable */
#define KPP_KPSR_KRSS		0x0008	/* kpp release synchronizer set */
#define KPP_KPSR_KDSC		0x0004	/* kpp depress synchronizer clear */
#define KPP_KPSR_KPKR		0x0002	/* kpp key release */
#define KPP_KPSR_KPKD		0x0001	/* kpp key depress */

/*--------------------------------------------------------------------------*/
typedef unsigned short keycode_t;

static DECLARE_WAIT_QUEUE_HEAD(keypad_wait_queue);	/* Used for blocking read */

/* --- keypad_lock --- 
 * protects the following data items and
 * the hardware registers */
static spinlock_t keypad_lock = SPIN_LOCK_UNLOCKED;

#ifdef CONFIG_MX2ADS_KEYPAD_MISC
static keycode_t queue[QUEUE_SIZE];
static int head = 0;
static int tail = 0;
static int open_count = 0;

static struct fasync_struct *keypad_async_queue = 0;
#endif

static int keypad_suspended;

static struct timer_list keypad_timer;
static unsigned char keys_debounce[8] =
    { ROW_MASK, ROW_MASK, ROW_MASK, ROW_MASK,
	ROW_MASK, ROW_MASK, ROW_MASK, ROW_MASK
};
static unsigned char keys_last[8] = { ROW_MASK, ROW_MASK, ROW_MASK, ROW_MASK,
	ROW_MASK, ROW_MASK, ROW_MASK, ROW_MASK
};

#ifdef CONFIG_MX2ADS_KEYPAD_INPUT
static struct input_dev keypad_input_dev;
#endif

static int __initdata keypad_region_init, keypad_gpio_init, keypad_irq_init;

#ifdef CONFIG_MX2ADS_KEYPAD_MISC
static int __initdata keypad_misc_init;
#endif

static keycode_t keymap[64] = {
	/* Low Word Key Mapping 0-1f */
	KEY_F1,			/* EXTRA 5 */
	KEY_F2,			/* EXTRA 4 */
	KEY_F3,			/* EXTRA 3 */
	KEY_F4,			/* EXTRA 2 */
	KEY_BACKSPACE,		/* BACK */
	KEY_END,		/* END */
	KEY_RESERVED,		/* Key not mapped on keypad */
	KEY_RESERVED,		/* Key not mapped on keypad */
	KEY_MACRO,		/* # */
	KEY_9,			/* 9 */
	KEY_6,			/* 6 */
	KEY_3,			/* 3 */
	KEY_RIGHT,		/* RIGHT */
	KEY_F5,			/* KEY 2 */
	KEY_RESERVED,		/* Key not mapped on keypad */
	KEY_RESERVED,		/* Key not mapped on keypad */
	KEY_0,			/* 0 */
	KEY_8,			/* 8 */
	KEY_5,			/* 5 */
	KEY_2,			/* 2 */
	KEY_F6,			/* ACTION */
	KEY_UP,			/* UP */
	KEY_RESERVED,		/* Key not mapped on keypad */
	KEY_RESERVED,		/* Key not mapped on keypad */
	KEY_KPASTERISK,		/* * */
	KEY_7,			/* 7 */
	KEY_4,			/* 4 */
	KEY_1,			/* 1 */
	KEY_LEFT,		/* LEFT */
	KEY_F7,			/* KEY 1 */
	KEY_RESERVED,		/* Key not mapped on keypad */
	KEY_RESERVED,		/* Key not mapped on keypad */
	KEY_F8,			/* RECORD */
	KEY_F9,			/* EXTRA 1 */
	KEY_F10,		/* APP 4 */
	KEY_F11,		/* APP 3 */
	KEY_HOME,		/* HOME */
	KEY_F12,		/* SEND */
	KEY_RESERVED,		/* Key not mapped on keypad */
	KEY_RESERVED,		/* Key not mapped on keypad */
	KEY_POWER,		/* POWER */
	KEY_VOLUMEDOWN,		/* VOL DOWN */
	KEY_VOLUMEUP,		/* VOL UP */
	KEY_DOWN,		/* Down */
	KEY_F16,		/* APP 2 */
	KEY_F17,		/* APP 1 */
	KEY_RESERVED,		/* Key not mapped on keypad */
	KEY_RESERVED		/* Key not mapped on keypad */
	    /* Remaining Keys not defined */
};

/*--------------------------------------------------------------------------*/

static void enable_keypad_interrupt(void);
static void disable_keypad_interrupt(void);

/*--------------------------------------------------------------------------*/
/* power management */

static struct pm_dev *keypad_pm;

static void
keypad_suspend(void)
{
	unsigned long flags;

	if (!keypad_suspended) {
		spin_lock_irqsave(&keypad_lock, flags);
		keypad_suspended = 1;
		wake_up_interruptible(&keypad_wait_queue);
		disable_irq(INT_KPP_TX);
		mx_module_clk_close(IPG_MODULE_KPP);
		spin_unlock_irqrestore(&keypad_lock, flags);
	}

}

static void
keypad_resume(void)
{
	unsigned long flags;

	if (keypad_suspended) {
		spin_lock_irqsave(&keypad_lock, flags);
		mx_module_clk_open(IPG_MODULE_KPP);
		enable_irq(INT_KPP_TX);
		mod_timer(&keypad_timer, jiffies + SAMPLE_PERIOD);
		keypad_suspended = 0;
		spin_unlock_irqrestore(&keypad_lock, flags);
	}

}

static int
keypad_pm_handler(struct pm_dev *dev, pm_request_t rqst, void *data)
{
	switch (rqst) {
	case PM_RESUME:
		keypad_resume();
		break;
	case PM_SUSPEND:
		keypad_suspend();
		break;
	default:
		break;
	}
	return 0;
}

#if 1				/*CEE LDM */
#include <linux/device.h>

extern void mx21_ldm_bus_register(struct device *device,
				  struct device_driver *driver);
extern void mx21_ldm_bus_unregister(struct device *device,
				    struct device_driver *driver);
static int
keypad_ldm_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		keypad_suspend();
		break;
	}
	return 0;
}

static int
keypad_ldm_resume(struct device *dev, u32 level)
{
	switch (level) {
	case RESUME_POWER_ON:
		keypad_resume();
		break;
	}
	return 0;
}

static struct device_driver keypad_driver_ldm = {
	.name = "mx21-keyp",
	.devclass = NULL,
	.probe = NULL,
	.suspend = keypad_ldm_suspend,
	.resume = keypad_ldm_resume,
	.scale = NULL,
	.remove = NULL,
};

static struct device keypad_device_ldm = {
	.name = "Keypad",
	.bus_id = "keypad",
	.driver = &keypad_driver_ldm,
	.power_state = DPM_POWER_ON,
};

#endif

#ifdef CONFIG_MX2ADS_KEYPAD_MISC

/*----------------------------------------------------------------------------
 * Get a keycode for the queue.
 *
 * Parameters:
 *    code_pointer   were to return the keycode
 *
 * Return: 
 *    0 if queue is empty
 *    1 on success
 *--------------------------------------------------------------------------*/
static int
get_keycode(keycode_t * code_pointer)
{
	unsigned long flags;

	spin_lock_irqsave(&keypad_lock, flags);
	if (head == tail) {
		spin_unlock_irqrestore(&keypad_lock, flags);
		return 0;
	}
	*code_pointer = queue[tail];
	tail++;
	if (tail >= QUEUE_SIZE) {
		tail = 0;
	}
	spin_unlock_irqrestore(&keypad_lock, flags);
	return 1;
}

/*----------------------------------------------------------------------------
 * Put a keycode into the queue.
 *
 * If the queue is full then replace the oldest entry
 * with the new one. Therefore, the queue will contain the
 * most resent key events.
 *
 * Parameters:
 *    keycode   keycode to be put in the queue
 *--------------------------------------------------------------------------*/
static void
put_keycode(keycode_t keycode)
{
	unsigned long flags;

	spin_lock_irqsave(&keypad_lock, flags);
	queue[head] = keycode;
	head++;
	if (head >= QUEUE_SIZE) {
		head = 0;
	}
	if (head == tail) {
		/* queue overflowed so
		 * toss the oldest entry
		 */
		tail++;
		if (tail >= QUEUE_SIZE) {
			tail = 0;
		}
	}
	spin_unlock_irqrestore(&keypad_lock, flags);
}

/*----------------------------------------------------------------------------
 * Support of the async IO
 *--------------------------------------------------------------------------*/
static int
keypad_fasync(int fd, struct file *filp, int mode)
{
	return fasync_helper(fd, filp, mode, &keypad_async_queue);
}

/*----------------------------------------------------------------------------
 * open keypad device
 *--------------------------------------------------------------------------*/
static int
keypad_open(struct inode *inode, struct file *filp)
{
	unsigned long flags;

	MOD_INC_USE_COUNT;

	spin_lock_irqsave(&keypad_lock, flags);
	if (open_count == 0) {
		/* toss any queue input */
		head = 0;
		tail = 0;
	}
	open_count++;
	spin_unlock_irqrestore(&keypad_lock, flags);

	return 0;
}

/*----------------------------------------------------------------------------
 * release keypad device
 *--------------------------------------------------------------------------*/
static int
keypad_release(struct inode *inode, struct file *filp)
{
	unsigned long flags;

	spin_lock_irqsave(&keypad_lock, flags);
	open_count--;
	spin_unlock_irqrestore(&keypad_lock, flags);

	/* remove this filp from the asynchronously notified filp's */
	keypad_fasync(-1, filp, 0);

	MOD_DEC_USE_COUNT;
	return 0;
}

/*----------------------------------------------------------------------------
 * read keypad device
 *--------------------------------------------------------------------------*/
static ssize_t
keypad_read(struct file *filp, char *buf, size_t count, loff_t * f_pos)
{
	int non_blocking;
	int ret_count;
	keycode_t code;

	if (count < sizeof (keycode_t)) {
		return -EINVAL;
	}

	ret_count = 0;
	non_blocking = filp->f_flags & O_NONBLOCK;

	while (count >= sizeof (keycode_t)) {
		if (get_keycode(&code)) {
			if (copy_to_user(buf, &code, sizeof (keycode_t))) {
				return -EFAULT;
			}
			buf += sizeof (keycode_t);
			ret_count += sizeof (keycode_t);
			count -= sizeof (keycode_t);
		} else {
			if (non_blocking) {
				if (ret_count == 0) {
					return -EAGAIN;
				}
				break;
			} else {
				interruptible_sleep_on(&keypad_wait_queue);
				if (signal_pending(current)) {
					return -ERESTARTSYS;
				}
			}
		}
	}
	return ret_count;
}

/*----------------------------------------------------------------------------
 * Support of the poll system call
 *--------------------------------------------------------------------------*/
static unsigned int
keypad_poll(struct file *filp, struct poll_table_struct *wait)
{
	poll_wait(filp, &keypad_wait_queue, wait);

	return (head == tail) ? 0 : (POLLIN | POLLRDNORM);
}

static struct file_operations keypad_fops = {
	open:keypad_open,
	release:keypad_release,
	read:keypad_read,
	poll:keypad_poll,
	fasync:keypad_fasync,
};

static struct miscdevice keypad_misc_dev = {
	minor:KEYPAD_MINOR,
	name:MODULE_NAME,
	fops:&keypad_fops,
};
#endif

/*----------------------------------------------------------------------------
 * Initialize the keypad controller hardware
 *--------------------------------------------------------------------------*/
static void
keypad_hardware_init(void)
{
	unsigned long flags;

	spin_lock_irqsave(&keypad_lock, flags);

	/* enable the keypad clock */
	mx_module_clk_open(IPG_MODULE_KPP);

	KPP_KPSR = KPP_KPSR_KPP_EN;

	/* init data register */
	KPP_KPDR |= COL_MASK;

	/* enable row and column lines */
	KPP_KPCR |= COL_MASK | ROW_MASK;

	/* set column pins to be outputs */
	KPP_KDDR |= COL_MASK;
	/* set row pins to be inputs  */
	KPP_KDDR &= ~ROW_MASK;

	/* clear out the key depress and key release synchronizers */
	KPP_KPSR |= KPP_KPSR_KDSC | KPP_KPSR_KRSS;

	enable_keypad_interrupt();

	spin_unlock_irqrestore(&keypad_lock, flags);
}

/*----------------------------------------------------------------------------
 * Enable keypad depress interrupt
 *--------------------------------------------------------------------------*/
static void
enable_keypad_interrupt(void)
{
	unsigned long flags;

	spin_lock_irqsave(&keypad_lock, flags);
	KPP_KPSR &= ~(KPP_KPSR_KDSC | KPP_KPSR_KRSS);
	KPP_KPDR &= ~COL_MASK;
	KPP_KPSR |= KPP_KPSR_KDIE;
	spin_unlock_irqrestore(&keypad_lock, flags);
}

/*----------------------------------------------------------------------------
 * Disable keypad depress interrupt
 *--------------------------------------------------------------------------*/
static void
disable_keypad_interrupt(void)
{
	unsigned long flags;

	spin_lock_irqsave(&keypad_lock, flags);
	KPP_KPSR |= KPP_KPSR_KDSC | KPP_KPSR_KRSS;
	KPP_KPSR &= ~(KPP_KPSR_KDIE | KPP_KPSR_KPKD | KPP_KPSR_KPKR);
	spin_unlock_irqrestore(&keypad_lock, flags);
}

/*----------------------------------------------------------------------------
 * Keypad interrupt service routine
 *
 * The key pad timer routine is scheduled to run and the
 * key pad interrupt is disabled.
 *--------------------------------------------------------------------------*/
static void
keypad_isr(int irq, void *dev_id, struct pt_regs *regs)
{
	mod_timer(&keypad_timer, jiffies + SAMPLE_PERIOD);
	disable_keypad_interrupt();
}

/*----------------------------------------------------------------------------
 * Keypad timer routine
 *
 * Once a key is pressed poll looking for additional keys being pressed
 * and for key releases. Put key press and key release events into the
 * keypad event queue. Do this until all of the keys are released.
 *--------------------------------------------------------------------------*/
static void
keypad_timer_routine(unsigned long data)
{
	unsigned char row;
	int col_sel;
	int row_sel;
	int diff;
	int new_event;
	int key_num;
	int done;

	/* scan the key matrix and put key events into the queue */
	new_event = 0;
	done = 1;
	for (col_sel = 0; col_sel < NUM_COL_LINES; col_sel++) {
		KPP_KPDR = (KPP_KPDR | COL_MASK) & ~(0x100 << col_sel);
		udelay(3);	/* give the lines a little time to settle */
		row = KPP_KPDR & ROW_MASK;

		/* Debounce the keys. They need to be low for two samples */
		if (row != keys_debounce[col_sel]) {
			keys_debounce[col_sel] = row;
			done = 0;
			continue;
		}

		if (row != ROW_MASK) {
			done = 0;
		}

		/* check for key state changes */
		diff = row ^ keys_last[col_sel];
		for (row_sel = 0; diff && (row_sel < NUM_ROW_LINES); row_sel++) {
			if (diff & 1) {
				key_num = col_sel * 8 + row_sel;
				if (row & (1 << row_sel)) {
#ifdef CONFIG_MX2ADS_KEYPAD_MISC
					put_keycode(keymap[key_num]);
#endif
#ifdef CONFIG_MX2ADS_KEYPAD_INPUT
					input_report_key(&keypad_input_dev,
							 keymap[key_num], 0);
#endif
				} else {
#ifdef CONFIG_MX2ADS_KEYPAD_MISC
					put_keycode(keymap[key_num] |
						    PRESSED_FLAG);
#endif
#ifdef CONFIG_MX2ADS_KEYPAD_INPUT
					input_report_key(&keypad_input_dev,
							 keymap[key_num], 1);
#endif
				}
				new_event = 1;
			}
			diff >>= 1;
		}
		keys_last[col_sel] = row;
	}
	if (new_event) {
		wake_up_interruptible(&keypad_wait_queue);
#ifdef CONFIG_MX2ADS_KEYPAD_MISC
		if (keypad_async_queue) {
			kill_fasync(&keypad_async_queue, SIGIO, POLL_IN);
		}
#endif
	}
	if (done) {
		enable_keypad_interrupt();
	} else {
		mod_timer(&keypad_timer, jiffies + SAMPLE_PERIOD);
	}
}

static void __init keypad_exit(void);

/*----------------------------------------------------------------------------
 * Initialization code for the keypad driver.
 *
 * This is run only once. 
 *--------------------------------------------------------------------------*/
static int __init
keypad_init(void)
{
	int tmp;

	if (!request_region(KPP_BASE, 0x1000, "mx21-keyp")) {
		printk(KERN_ERR PKMOD "keypad is already in use\n");
		keypad_exit();
		return -1;
	}
	keypad_region_init = 1;

	/* Set GPIO port-E pins 3,4,6,7 to alternate function.
	 * The UART2 conflicts with the keypad. */
	tmp = mx2_register_gpios(PORT_E, (1 << 3) |
				 (1 << 4) | (1 << 6) | (1 << 7), SECONDARY);
	if (tmp < 0) {
		printk(KERN_ERR PKMOD
		       "GPIO PORT E pins 3,4,6,7 already in use\n");
		keypad_exit();
		return tmp;
	}
	keypad_gpio_init = 1;

#ifdef CONFIG_MX2ADS_KEYPAD_MISC
	tmp = misc_register(&keypad_misc_dev);
	if (tmp != 0) {
		printk(KERN_ERR PKMOD "error registering misc device\n");
		keypad_exit();
		return tmp;
	}
	keypad_misc_init = 1;
#endif

	init_timer(&keypad_timer);
	keypad_timer.function = keypad_timer_routine;

	tmp =
	    request_irq(INT_KPP_TX, (void *) keypad_isr,
			SA_INTERRUPT | SA_SAMPLE_RANDOM, MODULE_NAME, NULL);
	if (tmp) {
		printk(KERN_ERR PKMOD "error requesting keypad irq %d\n",
		       INT_KPP_TX);
		keypad_exit();
		return tmp;
	}
	keypad_irq_init = 1;
#ifdef CONFIG_MX2ADS_KEYPAD_INPUT
	keypad_input_dev.name = "Motorola i.MX21 ADS Keypad";
	keypad_input_dev.idbus = BUS_ONCHIP;
	keypad_input_dev.idversion = 0x1;

	keypad_input_dev.evbit[LONG(EV_KEY)] |= BIT(EV_KEY);
	for (tmp = 0; tmp < (sizeof (keymap) / sizeof (keycode_t)); tmp++) {
		if (keymap[tmp] != KEY_RESERVED)
			keypad_input_dev.keybit[LONG(keymap[tmp])] |=
			    BIT(keymap[tmp]);
	}

	input_register_device(&keypad_input_dev);
#endif

	keypad_hardware_init();

	keypad_pm = pm_register(PM_SYS_DEV, PM_SYS_VGA, keypad_pm_handler);
#if 1				/*CEE LDM */
	mx21_ldm_bus_register(&keypad_device_ldm, &keypad_driver_ldm);
#endif

	return 0;
}

/*----------------------------------------------------------------------------
 * Exit code for the keypad driver.
 *
 *--------------------------------------------------------------------------*/
static void __init
keypad_exit(void)
{
	if (keypad_irq_init) {
		pm_unregister(keypad_pm);
#if 1				/*CEE LDM */
		mx21_ldm_bus_unregister(&keypad_device_ldm, &keypad_driver_ldm);
#endif

		disable_keypad_interrupt();
		free_irq(INT_KPP_TX, NULL);

		del_timer(&keypad_timer);

		mx_module_clk_close(IPG_MODULE_KPP);
#ifdef CONFIG_MX2ADS_KEYPAD_INPUT
		input_unregister_device(&keypad_input_dev);
#endif
	}
#ifdef CONFIG_MX2ADS_KEYPAD_MISC
	if (keypad_misc_init) {
		misc_deregister(&keypad_misc_dev);
	}
#endif
	if (keypad_gpio_init) {
		mx2_unregister_gpios(PORT_E, (1 << 3) |
				     (1 << 4) | (1 << 6) | (1 << 7));
	}
	if (keypad_region_init) {
		release_region(KPP_BASE, 0x1000);
	}
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Motorola i.MX21 Keypad Driver");

module_init(keypad_init);
module_exit(keypad_exit);
