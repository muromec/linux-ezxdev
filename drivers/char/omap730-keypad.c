/*
 * linux/drivers/char/omap730-keypad.c
 *
 * TI P2-SAMPLE/OMAP730 keypad driver
 *
 * Taken from p2sample-keypad.c by Jean Pihet
 * 
 * Author: MontaVista Software, Inc.
 *         <gdavis@mvista.com> or <source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * History:
 *
 * 20030529: George G. Davis <gdavis@mvista.com>
 *	Initial release
 *
 * TODO:
 * 1. Test with a variety of GUIs.
 * 2. Complete DPM/LDM stubs and test.
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/stddef.h>
#include <linux/timer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/slab.h>

#include <asm/hardware.h>
#include <asm/uaccess.h>
#include <asm/arch/irqs.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/irq.h>

#undef CONFIG_CEE
#define CONFIG_CEE

#undef	DEBUG
#ifdef	DEBUG
#define	DPRINTK(fmt, args...) 				\
	printk("%s:%d: " fmt,				\
	       __FUNCTION__ , __LINE__ , ## args);
#else
#define	DPRINTK(fmt, args...)
#endif

// Physical keypad definitions
#define NB_ROW		6
#define NB_COL		5


static struct input_dev keypad_dev;
static int omap730_keypad_pid;
static DECLARE_COMPLETION(keypadt);
static wait_queue_head_t keypadt_wait;
struct task_struct *omap730_keypad_task;


#define KBD_ENABLE_INT      0   
#define KBD_DISABLE_INT     1

#ifdef CONFIG_CEE	/* MVL-CEE */
#include <linux/device.h>

static int omap730_keypad_suspend(struct device *dev, u32 state, u32 level);
static int omap730_keypad_resume(struct device *dev, u32 level);
static int omap730_keypad_scale(struct bus_op_point *op, u32 level);

static struct device_driver omap730_keypad_driver_ldm = {
	.name		= "perseus2-keypad",
	.devclass	= NULL,
	.probe		= NULL,
	.suspend	= omap730_keypad_suspend,
	.resume		= omap730_keypad_resume,
	.scale		= omap730_keypad_scale,
	.remove		= NULL,
};

static struct device omap730_keypad_device_ldm = {
	.name		= "Perseus2/OMAP730 Keypad",
	.bus_id		= "keypad",
	.driver		= NULL,
	.power_state	= DPM_POWER_ON,
};

static void
omap730_keypad_ldm_driver_register(void)
{
	extern void mpu_public_driver_register(struct device_driver *driver);
	mpu_public_driver_register(&omap730_keypad_driver_ldm);
}

static void
omap730_keypad_ldm_device_register(void)
{
	extern void mpu_public_device_register(struct device *device);
	mpu_public_device_register(&omap730_keypad_device_ldm);
}

static void
omap730_keypad_ldm_driver_unregister(void)
{
	extern void mpu_public_driver_unregister(struct device_driver *driver);
	mpu_public_driver_unregister(&omap730_keypad_driver_ldm);
}

static void
omap730_keypad_ldm_device_unregister(void)
{
	extern void mpu_public_device_unregister(struct device *device);
	mpu_public_device_unregister(&omap730_keypad_device_ldm);
}

static int
omap730_keypad_scale(struct bus_op_point *op, u32 level)
{
	/* REVISIT */
	return 0;
}

static int
omap730_keypad_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		/* REVISIT */
		break;
	}

	return 0;
}

static int
omap730_keypad_resume(struct device *dev, u32 level)
{
	int ret = 0;

	switch (level) {
	case RESUME_POWER_ON:
		/* REVISIT */
		break;
	}

	return ret;
}
#endif	/* MVL-CEE */

static u32
omap730_keypad_read(void)
{
	// Just put the button presses in a u32. Bit are organized as follows:
	//  bit 0: C0,R0; bit 1: C0,R1; bit 2: C0, R2 ...
	//  bit 6: C1,R0; bit 7: C1,R1; bit 8: C1, R2 ...
	int i;
	u32 keys = 0;

	outw(0xff, MPUIO_KBC_REG);
	udelay(1);

	for (i = 0; i < NB_COL; i += 1) {
		outw(~((1 << i) & ((1 << NB_COL) - 1)), MPUIO_KBC_REG);
		udelay(1);
		keys |= ((~inw(MPUIO_KBR_LATCH)) & ((1 << NB_ROW) - 1)) << (i * NB_ROW);
		inw(OMAP730_32kHz_TIMER_BASE);	/* BTS_Errata.22 */
	}

	outw(0x0, MPUIO_KBC_REG);
	udelay(1);

	DPRINTK("keys: %0x\n", keys);

	return keys;
}

static void
omap730_keypad_scan(void)
{
	int timeout;
	u32 old = 0;
	u32 new = 0;

	do {
		new = omap730_keypad_read();
		if (old != new) {
			DPRINTK("old: %0x, new: %0x\n", old, new);
			input_event(&keypad_dev, EV_MSC, MSC_KEYPAD, new);
			old = new;
		}
		timeout = interruptible_sleep_on_timeout(&keypadt_wait, 10);
		if (timeout) {
			break;
		}
	} while (new);

	return;
}

static int
omap730_keypad_thread(void *null)
{
	struct task_struct *tsk = current;
	unsigned long flags;

	daemonize();
	reparent_to_init();
	strcpy(tsk->comm, "keypadt");
	tsk->tty = NULL;

	/* only want to receive SIGKILL */
	spin_lock_irq(&tsk->sigmask_lock);
	siginitsetinv(&tsk->blocked, sigmask(SIGKILL));
	recalc_sigpending(tsk);
	spin_unlock_irq(&tsk->sigmask_lock);

	omap730_keypad_task = tsk;

	complete(&keypadt);

	DPRINTK("init\n");

	do {
		/* Careful, a dead lock can occur here if an keypad
		 * interrupt occurs before we're ready and waiting on
		 * the keypadt_wait queue. So we disable interrupts
		 * while unmasking device interrupts prior to putting
		 * the thread to sleep.
		 */
		local_irq_save(flags);
		outw(KBD_ENABLE_INT, MPUIO_KBD_MASKIT);

		interruptible_sleep_on(&keypadt_wait);
		local_irq_restore(flags);
		if (signal_pending(tsk))
			break;
		omap730_keypad_scan();
	} while (!signal_pending(tsk));

	outw(KBD_DISABLE_INT, MPUIO_KBD_MASKIT);
	complete_and_exit(&keypadt, 0);
}

static void
omap730_keypad_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	outw(KBD_DISABLE_INT, MPUIO_KBD_MASKIT);   // Mask off irq

	DPRINTK("interrupt\n");

	/* Wake up thread to scan keypad. */
	wake_up(&keypadt_wait);
}

static int __init
omap730_keypad_init(void)
{
	printk(KERN_INFO "TI Perseus2/OMAP730 keypad driver.\n");

	/* 
	 * Sensible default setup.
	 */
	outw(KBD_DISABLE_INT, MPUIO_KBD_MASKIT);
	outw(0xff, MPUIO_GPIO_DEBOUNCING_REG);
	outw(0x0, MPUIO_KBC_REG);


	keypad_dev.name = "Perseus2/OMAP730 Keypad";
	keypad_dev.idbus = BUS_ONCHIP;
	keypad_dev.idvendor = 0x0451;	/* Texas Instruments, Inc. */
	keypad_dev.idproduct = 0x0730;	/* total fabrication here */
	keypad_dev.idversion = 0x0100;

	keypad_dev.evbit[LONG(EV_MSC)] |= BIT(EV_MSC);
	keypad_dev.mscbit[LONG(MSC_KEYPAD)] |= BIT(MSC_KEYPAD);

	input_register_device(&keypad_dev);

	init_waitqueue_head(&keypadt_wait);
	init_completion(&keypadt);
	omap730_keypad_pid =
	    kernel_thread(omap730_keypad_thread, NULL,
			  CLONE_FS | CLONE_FILES | CLONE_SIGHAND);
	if (omap730_keypad_pid <= 0) {
		return -1;
	}
	wait_for_completion(&keypadt);

	if (request_irq(INT_KEYBOARD, omap730_keypad_interrupt,
			SA_INTERRUPT, "keypad", 0) < 0) {
		/* Kill the thread */
		init_completion(&keypadt);
		send_sig(SIGKILL, omap730_keypad_task, 1);
		wake_up(&keypadt_wait);
		wait_for_completion(&keypadt);
		input_unregister_device(&keypad_dev);
		return -EINVAL;
	}

#ifdef CONFIG_CEE	/* MVL-CEE */
	omap730_keypad_ldm_driver_register();
	omap730_keypad_ldm_device_register();
#endif			/* MVL-CEE */

	return 0;
}

static void __exit
omap730_keypad_exit(void)
{
	outw(KBD_DISABLE_INT, MPUIO_KBD_MASKIT);

#ifdef CONFIG_CEE	/* MVL-CEE */
	omap730_keypad_ldm_device_unregister();
	omap730_keypad_ldm_driver_unregister();
#endif			/* MVL-CEE */

	/* Kill the thread */
	init_completion(&keypadt);
	send_sig(SIGKILL, omap730_keypad_task, 1);
	wake_up(&keypadt_wait);
	wait_for_completion(&keypadt);

	input_unregister_device(&keypad_dev);
	free_irq(INT_KEYBOARD, 0);

	return;
}

module_init(omap730_keypad_init);
module_exit(omap730_keypad_exit);

MODULE_AUTHOR("Dave Peverley <dpeverley@mpc-data.co.uk>");
MODULE_DESCRIPTION("TI Perseus2/OMAP730 Keypad driver");
MODULE_LICENSE("GPL");
