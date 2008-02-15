/*
 * linux/drivers/char/innovator-keypad.c
 *
 * TI Innovator/OMAP1510 keypad driver
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
 * 20031003: MontaVista Software, Inc. <source@mvista.com>
 *      Added OMAP1610 support
 *
 * 20030529: George G. Davis <gdavis@mvista.com>
 *	Initial release
 * 20031029: George G. Davis <gdavis@mvista.com>
 *	Convert to EV_MSC based device since EV_KEY was not appropriate.
 *
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
int debug = 0;
#define	DPRINTK(fmt, args...) \
	if (debug) {						\
		printk(KERN_DEBUG "%s:%d: " fmt,		\
		       __FUNCTION__ , __LINE__ , ## args);	\
	}
#else
#define	DPRINTK(fmt, args...)
#endif

static struct input_dev keypad_dev;
static int innovator_keypad_pid;
static DECLARE_COMPLETION(keypadt);
static wait_queue_head_t keypadt_wait;
struct task_struct *innovator_keypad_task;
static unsigned char keypad_state[8];

#define KEY(col, row, val) (((col) << 28) | ((row) << 24) | (val))
#ifdef CONFIG_OMAP_H2 
static int h2_keymap[] = {
        KEY(0, 0, KEY_LEFT),
        KEY(0, 1, KEY_RIGHT),
        KEY(0, 2, KEY_3),
        KEY(0, 3, KEY_F10),
        KEY(0, 4, KEY_F5),
        KEY(0, 5, KEY_9),
        KEY(1, 0, KEY_DOWN),
        KEY(1, 1, KEY_UP),
        KEY(1, 2, KEY_2),
        KEY(1, 3, KEY_F9),
        KEY(1, 4, KEY_F7),
        KEY(1, 5, KEY_0),
        KEY(2, 0, KEY_ENTER),
        KEY(2, 1, KEY_6),
        KEY(2, 2, KEY_1),
        KEY(2, 3, KEY_F2),
        KEY(2, 4, KEY_F6),
        KEY(2, 5, KEY_HOME),
        KEY(3, 0, KEY_8),
        KEY(3, 1, KEY_5),
        KEY(3, 2, KEY_F12),
        KEY(3, 3, KEY_F3),
        KEY(3, 4, KEY_F8),
        KEY(3, 5, KEY_END),
        KEY(4, 0, KEY_7),
        KEY(4, 1, KEY_4),
        KEY(4, 2, KEY_F11),
        KEY(4, 3, KEY_F1),
        KEY(4, 4, KEY_F4),
        KEY(4, 5, KEY_ESC),
        0
};

#define KDELAY 9
#else
static int innovator_keymap[] = {
        KEY(0, 0, KEY_F1),
        KEY(0, 3, KEY_DOWN),
        KEY(1, 1, KEY_F2),
        KEY(1, 2, KEY_RIGHT),
        KEY(2, 0, KEY_F3),
        KEY(2, 1, KEY_F4),
        KEY(2, 2, KEY_UP),
        KEY(3, 2, KEY_ENTER),
        KEY(3, 3, KEY_LEFT),
        0
};
#define KDELAY 2
#endif

static int *keymap;


#if	defined(CONFIG_CEE)	/* MVL-CEE */
#include <linux/device.h>

static int innovator_keypad_suspend(struct device *dev, u32 state, u32 level);
static int innovator_keypad_resume(struct device *dev, u32 level);
static int innovator_keypad_scale(struct bus_op_point *op, u32 level);

static struct device_driver innovator_keypad_driver_ldm = {
	.name		= "innovator-keypad",
	.devclass	= NULL,
	.probe		= NULL,
	.suspend	= innovator_keypad_suspend,
	.resume		= innovator_keypad_resume,
	.scale		= innovator_keypad_scale,
	.remove		= NULL,
};

static struct device innovator_keypad_device_ldm = {
	.name		= "Innovator/OMAP1510/1610 Keypad",
	.bus_id		= "keypad",
	.driver		= NULL,
	.power_state	= DPM_POWER_ON,
};

static void
innovator_keypad_ldm_driver_register(void)
{
	extern void mpu_public_driver_register(struct device_driver *driver);
	mpu_public_driver_register(&innovator_keypad_driver_ldm);
}

static void
innovator_keypad_ldm_device_register(void)
{
	extern void mpu_public_device_register(struct device *device);
	mpu_public_device_register(&innovator_keypad_device_ldm);
}

static void
innovator_keypad_ldm_driver_unregister(void)
{
	extern void mpu_public_driver_unregister(struct device_driver *driver);
	mpu_public_driver_unregister(&innovator_keypad_driver_ldm);
}

static void
innovator_keypad_ldm_device_unregister(void)
{
	extern void mpu_public_device_unregister(struct device *device);
	mpu_public_device_unregister(&innovator_keypad_device_ldm);
}

static int
innovator_keypad_scale(struct bus_op_point *op, u32 level)
{
	/* REVISIT */
	return 0;
}

static int
innovator_keypad_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		/* REVISIT */
		break;
	}

	return 0;
}

static int
innovator_keypad_resume(struct device *dev, u32 level)
{
	int ret = 0;

	switch (level) {
	case RESUME_POWER_ON:
		/* REVISIT */
		break;
	}

	return ret;
}
#endif				/* MVL-CEE */

#ifdef CONFIG_ARCH_OMAP1610
/* Needed only for bug workaround below.
   32 kHz timer no longer supported. */
#define OMAP1610_32kHz_TIMER_BASE (0xfffb9000)

#define OMAP1510_MPUIO_KBC_REG MPUIO_KBC_REG
#define OMAP1510_MPUIO_KBR_LATCH  MPUIO_KBR_LATCH_REG
#define OMAP1510_32kHz_TIMER_BASE OMAP1610_32kHz_TIMER_BASE
#define OMAP1510_MPUIO_KBD_MASKIT     MPUIO_KBD_MASKIT_REG
#define OMAP1510_GPIO_DEBOUNCING_REG  MPUIO_DEBOUNCING_REG
#endif

static void 
innovator_keypad_read(unsigned char *state)
{
	 int col = 0;

	 /* read the keypad status */
        outw(0xff,  OMAP1510_MPUIO_KBC_REG);
        for (col = 0; col < 8; col++) {
               	outw(~(1 << col) & 0xff,  OMAP1510_MPUIO_KBC_REG);
               	udelay(KDELAY);
               	state[col] = ~inw(OMAP1510_MPUIO_KBR_LATCH) & 0xff;
		inw(OMAP1510_32kHz_TIMER_BASE);	/* BTS_Errata.22 */
         }
         outw(0x00,  OMAP1510_MPUIO_KBC_REG);
         udelay(KDELAY);

	DPRINTK("col: %0x\n", col);

}

static inline int omap_kp_find_key(int col, int row)
{
        int i, key;

        key = KEY(col, row, 0);
        for (i = 0; keymap[i] != 0; i++)
            if ((keymap[i] & 0xff000000) == key)
                    return keymap[i] & 0x00ffffff;
        return -1;
}

static void
innovator_keypad_scan(void)
{
	 unsigned char new_state[8], changed, key_down = 0;
	 int col, row, key;

	innovator_keypad_read(new_state);
	/* check for changes and print those */
	for (col = 0; col < 8; col++) {
		changed = new_state[col] ^ keypad_state[col];
               	key_down |= new_state[col];
               	if (changed == 0)
                        continue;

                for (row = 0; row < 8; row++) {
                    	if (!(changed & (1 << row)))
                       		continue;
                    	key = omap_kp_find_key(col, row);
                    	if (key < 0) {
                               printk(KERN_WARNING "omap-keypad: Spurious key event %d-%d\n", col, row);
                             	continue;
			}
                     	printk(KERN_INFO "omap-keypad:  new key %d-%d pressed\n", col, row);
			input_event(&keypad_dev, EV_MSC, MSC_KEYPAD, key);
	        } /* for */
	} /* for */
	memcpy(keypad_state, new_state, sizeof(keypad_state));
	 
	if (key_down) {
		interruptible_sleep_on_timeout(&keypadt_wait, 10);
 	}else {
		 /* enable interrupts */
		outw(0x0, OMAP1510_MPUIO_KBD_MASKIT);
	}	 

	return;
}

static int
innovator_keypad_thread(void *null)
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

	innovator_keypad_task = tsk;

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
		DPRINTK("sleep\n");
		outw(0x0, OMAP1510_MPUIO_KBD_MASKIT);
		interruptible_sleep_on(&keypadt_wait);
		local_irq_restore(flags);
		if (signal_pending(tsk))
			break;
		DPRINTK("wake\n");
		innovator_keypad_scan();
	} while (!signal_pending(tsk));

	outw(0x1, OMAP1510_MPUIO_KBD_MASKIT);
	complete_and_exit(&keypadt, 0);
}

static void
innovator_keypad_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	outw(1, OMAP1510_MPUIO_KBD_MASKIT);

	DPRINTK("interrupt\n");

	/* Wake up thread to scan keypad. */
	wake_up(&keypadt_wait);
}

static int __init
innovator_keypad_init(void)
{
	int i = 0;
	printk(KERN_INFO "TI Innovator/OMAP1510/1610 keypad driver.\n");

#ifdef CONFIG_ARCH_OMAP1610
	/* Configure as KBR5, default
	   configuration for this ball is GPIO13 */
	writel((readl(FUNC_MUX_CTRL_6) & ~(0x7 << 12)) | (0x01 << 12), FUNC_MUX_CTRL_6);

	/*configure pullup for KBR 5 */
	outl(inl(PULL_DWN_CTRL_1) & ~0x04, PULL_DWN_CTRL_1); 
	outl(inl(PU_PD_SEL_1) | 0x04, PU_PD_SEL_1);
	outl(inl(PULL_DWN_CTRL_0) & ~0x7FF, PULL_DWN_CTRL_0);
	outl(inl(PU_PD_SEL_0)|0x7C0, PU_PD_SEL_0);
	outw(0x1, OMAP1510_MPUIO_KBD_MASKIT);
	outw(0xff, OMAP1510_GPIO_DEBOUNCING_REG);
	outw(0x0, OMAP1510_MPUIO_KBD_MASKIT);

	keymap = h2_keymap;
#else
	keymap = innovator_keymap;

#endif
	keypad_dev.name = "Innovator/OMAP1510 Keypad";
	keypad_dev.idbus = BUS_ONCHIP;
	keypad_dev.idvendor = 0x0451;	/* Texas Instruments, Inc. */
	keypad_dev.idproduct = 0x1510;	/* total fabrication here */
	keypad_dev.idversion = 0x0100;

	keypad_dev.evbit[LONG(EV_MSC)] |= BIT(EV_MSC);
	keypad_dev.mscbit[LONG(MSC_KEYPAD)] |= BIT(MSC_KEYPAD);
 	for (i = 0; keymap[i] != 0; i++)
                 set_bit(keymap[i] & 0x00ffffff, keypad_dev.keybit);

	input_register_device(&keypad_dev);
	init_waitqueue_head(&keypadt_wait);
	init_completion(&keypadt);
	innovator_keypad_pid =
	    kernel_thread(innovator_keypad_thread, NULL,
			  CLONE_FS | CLONE_FILES | CLONE_SIGHAND);
	if (innovator_keypad_pid <= 0) {
		return -1;
	}
	wait_for_completion(&keypadt);

	if (request_irq(INT_KEYBOARD, innovator_keypad_interrupt,
			SA_INTERRUPT, "keypad", 0) < 0) {
		/* Kill the thread */
		init_completion(&keypadt);
		send_sig(SIGKILL, innovator_keypad_task, 1);
		wake_up(&keypadt_wait);
		wait_for_completion(&keypadt);
		input_unregister_device(&keypad_dev);
		return -EINVAL;
	}

#if	defined(CONFIG_CEE)	/* MVL-CEE */
	innovator_keypad_ldm_driver_register();
	innovator_keypad_ldm_device_register();
#endif				/* MVL-CEE */
	outw(0x1, OMAP1510_MPUIO_KBD_MASKIT);
	innovator_keypad_read(keypad_state);
	outw(0x0, OMAP1510_MPUIO_KBD_MASKIT);

	return 0;
}

static void __exit
innovator_keypad_exit(void)
{
	outw(1, OMAP1510_MPUIO_KBD_MASKIT);

#if	defined(CONFIG_CEE)	/* MVL-CEE */
	innovator_keypad_ldm_device_unregister();
	innovator_keypad_ldm_driver_unregister();
#endif				/* MVL-CEE */

	/* Kill the thread */
	init_completion(&keypadt);
	send_sig(SIGKILL, innovator_keypad_task, 1);
	wake_up(&keypadt_wait);
	wait_for_completion(&keypadt);

	input_unregister_device(&keypad_dev);
	free_irq(INT_KEYBOARD, 0);
}

module_init(innovator_keypad_init);
module_exit(innovator_keypad_exit);

MODULE_AUTHOR("George G. Davis <gdavis@mvista.com>");
MODULE_DESCRIPTION("TI Innovator/OMAP1510/1610 Keypad driver");
MODULE_LICENSE("GPL");
