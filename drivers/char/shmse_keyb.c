/* 
 * linux/drivers/char/shmse_keyb.c
 *
 * SH-Mobile SolutionEngine MS7300CP/MS73180CP01 Keypad driver.
 * Copyright (C) 2003 Takashi Yoshii <yoshii-takashi@hitachi-ul.co.jp>
 * Modified to support MS73180CP. Kazuyoshi Ishiwatari.
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>

#include <asm/machvec.h>
#include <asm/delay.h>
#include <asm/io.h>

#include "scan_keyb.h"

#ifdef CONFIG_DPM
static void shmse_kbd_ldm_register(void);
static void shmse_kbd_ldm_unregister(void);
#endif

#if defined(CONFIG_CPU_SUBTYPE_SH73180)
#define KEY_BASE	0xb1400000
#else
#define KEY_BASE	0xa44b0000
#endif
#define KEYCR1   (KEY_BASE+0x0)
#define KEYCR2   (KEY_BASE+0x4)
#define KEYINDR  (KEY_BASE+0x8)
#define KEYOUTDR (KEY_BASE+0xc)

static const unsigned char shmse_scan_table[] = {
/*	 in0   in1   in2   in3   in4     x     x     x	*/
#if defined(CONFIG_CPU_SUBTYPE_SH73180)
	  35,   34,   33,   32,   31,    0,    0,    0, /* out0 */
	  30,   29,   28,   27,   26,    0,    0,    0, /* out1 */
	  25,   24,   23,   22,   21,    0,    0,    0, /* out2 */
	  20,   19,   18,   17,   16,    0,    0,    0, /* out3 */
	  15,   14,   13,   12,   11,    0,    0,    0, /* out4 */
	  10,    9,    8,    7,    6,    0,    0,    0, /* out5 */
#else
	0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00, 0x00, /* out0 */
	0x06, 0x07, 0x08, 0x09, 0x0a, 0x00, 0x00, 0x00, /* out1 */
	0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00, 0x00, 0x00, /* out2 */
	0x11, 0x12, 0x13, 0x14, 0x15, 0x00, 0x00, 0x00, /* out3 */
	0x16, 0x17, 0x18, 0x19, 0x1a, 0x00, 0x00, 0x00, /* out4 */
	0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x00, 0x00, 0x00, /* out5 */
#endif
};

static const unsigned short shmse_keyout[] = {
	0x0ffc, 0x0ff3, 0x0fcf, 0x0f3f, 0x0cff, 0x03ff
};
#define LEN ARRAY_SIZE(shmse_keyout)

static int shmse_scan_kbd(unsigned char *s)
{
	int i;

	for(i=0; i<LEN; i++) {
 		ctrl_outw(shmse_keyout[i], KEYOUTDR);
 		udelay(35);
		*s++=ctrl_inw(KEYINDR);
	}
	ctrl_outw(0x0000, KEYOUTDR);

	return 0;
}


void __init shmse_kbd_init_hw(void)
{
 	ctrl_outw(0x0000, KEYCR1);
 	ctrl_outw(0x0010, KEYCR2);
 	ctrl_outw(0x00, KEYOUTDR);
	scan_kbd_init();
 	register_scan_keyboard(shmse_scan_kbd, shmse_scan_table, LEN);
 	printk(KERN_INFO "shmse matrix scan keyboard registered\n");
#ifdef CONFIG_DPM
	shmse_kbd_ldm_register();
#endif
}

#ifdef CONFIG_PM

extern struct timer_list scan_timer;
#define SCANHZ	(HZ/20)
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/device.h>

static int
shmse_kbd_suspend(struct device * dev,  u32 state, u32 level )
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		del_timer(&scan_timer);
		break;
	}

	return 0;
}

static int
shmse_kbd_resume(struct device * dev, u32 level )
{
	switch (level) {
	case RESUME_POWER_ON:
		ctrl_outw(0x0000, KEYCR1);
		ctrl_outw(0x0010, KEYCR2);
		ctrl_outw(0x00, KEYOUTDR);
		mod_timer(&scan_timer,jiffies + SCANHZ);
		break;
	}

	return 0;
}

#endif /* CONFIG_PM */


#ifdef CONFIG_DPM

static struct device_driver shmse_kbd_driver_ldm = {
	name:      	"shmse_kbd",
	devclass:  	NULL,
	probe:     	NULL,
	suspend:   	shmse_kbd_suspend,
	resume:    	shmse_kbd_resume,
	scale:	  	NULL,
	remove:    	NULL,
};

static struct device shmse_kbd_device_ldm = {
	name:		"Keyboard",
	bus_id:		"kbd",
	driver: 	NULL,
	power_state:	DPM_POWER_ON,
};

static void shmse_kbd_ldm_register(void)
{
	extern void plb_driver_register(struct device_driver *driver);
	extern void plb_device_register(struct device *device);

	plb_driver_register(&shmse_kbd_driver_ldm);
	plb_device_register(&shmse_kbd_device_ldm);
}

static void shmse_kbd_ldm_unregister(void)
{
	extern void plb_driver_unregister(struct device_driver *driver);
	extern void plb_device_unregister(struct device *device);

	plb_driver_unregister(&shmse_kbd_driver_ldm);
	plb_device_unregister(&shmse_kbd_device_ldm);
}

#endif /* CONFIG_DPM */
