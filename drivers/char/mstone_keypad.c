/*
 * linux/driver/char/mstone_keypad.c
 * Keypad driver for Intel Mainstone development board
 *
 * Author:	Nicolas Pitre
 * Created:	may 5, 2004
 * Copyright:	(C) 2004 MontaVista Software Inc.
 *
 * Copyright (C) 2005 Motorola
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* 
 * Port to EzX platform by Motorola.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/input.h>

#include <asm/irq.h>
#include <asm/hardware.h>

#ifdef CONFIG_DPM /* MVL-CEE */
#include <linux/device.h>

static unsigned int saved_kpc;

static int
pxakpd_resume(struct device *dev, u32 level)
{
	switch (level) {
	case RESUME_POWER_ON:
		KPC = saved_kpc;
		/* enable clock to keypad */
		CKEN |= CKEN19_KEYPAD;
#if 0
                {
                        struct kp_event kpe;
                        unsigned char c;

                        kpe.flags |= (KP_DIRECT | KP_MATRIX);
                        c = get_scancode(&kpe);
                        if (c != NO_KEY) {
                                /* Insert it into key event list. */
                                kpq_put(&kpe);
                        } else {
                                /* We are not woke by key press. */
                        }
                }
#endif
		break;
	}
	
	return 0;
}

static int
pxakpd_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		/* disable clock to keypad */
		CKEN &= ~CKEN19_KEYPAD;
		
		/* disable direct and matrix interrupts */
		saved_kpc = KPC;
		KPC &= ~(KPC_DIE | KPC_MIE);
		break;
	}
	
	return 0;
}

static struct device_driver pxakpd_driver_ldm = {
	name:          "pxa-kpd",
	suspend:       pxakpd_suspend,
	resume:        pxakpd_resume,
};

static struct device pxakpd_device_ldm = {
	name:         "PXA Keypad",
	bus_id:       "pxakpd",
	power_state:  DPM_POWER_ON,
};

static void
pxakpd_ldm_register(void)
{
	extern void pxaopb_driver_register(struct device_driver *driver);
	extern void pxaopb_device_register(struct device *device);
	
	pxaopb_driver_register(&pxakpd_driver_ldm);
	pxaopb_device_register(&pxakpd_device_ldm);
}

static void
pxakpd_ldm_unregister(void)
{
	extern void pxaopb_driver_unregister(struct device_driver *driver);
	extern void pxaopb_device_unregister(struct device *device);
	
	pxaopb_device_unregister(&pxakpd_device_ldm);
	pxaopb_driver_unregister(&pxakpd_driver_ldm);
}

#endif


/*
 * The Mainstone II manual says:
 *
 * · The 75-button, 7x7 matrix keypad maps to the Bulverde matrix signals
 *   MKIN<7:0> and MKOUT<7:0>.
 * · The rotary scroll wheel maps to the Bulverde signals DKIN<1:0>. The
 *   push-button maps to a matrix key.
 *
 * Since a 7x7=49 matrix can't hold all those keys, many keys were
 * made to activate 2 matrix positions at the same time.
 * This is not the most reliable thing to do, but we're stuck with it.
 *
 * The problem is that those combined key presses don't activate
 * simultaneously, so for each matrix event we need to wait a
 * certain delay to account for this before reporting a real key event.
 *
 * But yet some of those combined keys are rather unstable and don't produce
 * a consistent state for the whole duration of the key down time.
 *
 * So it seems that the only way to have some sort of sane behavior is to
 * actually report any event only when all keys have been released. Because
 * of this we can't rely on and report any maintained state.
 *
 * "Please don't use such scheme for your own product."
 */

#define MULTISWITCH

/* First, the array of single switch keys */
static const unsigned short mstkp_keycodes[64] = {
  KEY_A,     KEY_G,    KEY_M,    KEY_S,     KEY_DOT,       KEY_HOME,      KEY_UP,    0,
  KEY_B,     KEY_H,    KEY_N,    KEY_T,     KEY_EQUAL,     KEY_LEFTSHIFT, KEY_DOWN,  0,
  KEY_C,     KEY_I,    KEY_O,    KEY_U,     KEY_Y,         KEY_SPACE,     KEY_LEFT,  0,
  KEY_D,     KEY_J,    KEY_P,    KEY_V,     KEY_Z,         KEY_SPACE,     KEY_RIGHT, 0,
  KEY_E,     KEY_K,    KEY_Q,    KEY_W,     KEY_SLASH,     KEY_POWER,     KEY_ENTER, 0,
  KEY_F,     KEY_L,    KEY_R,    KEY_X,     KEY_BACKSLASH, KEY_DELETE,    0,         0,
  KEY_PHONE, KEY_STOP, BTN_LEFT, BTN_RIGHT, BTN_MIDDLE,    0,	          0,         0,
  0,         0,        0,        0,         0,             0,             0,         0,
};

#ifdef MULTISWITCH

/* Next, multiswitch keys, indexed by their lowest coordinate */
static const unsigned short mstkp_multicodes[40] = {
  0, KEY_1,  KEY_4,  KEY_7,         KEY_KPASTERISK, KEY_TAB,        0, 0,
  0, KEY_F1, KEY_F3, KEY_MINUS,     KEY_APOSTROPHE, KEY_LEFTBRACE,  0, 0,   
  0, KEY_2,  KEY_5,  KEY_8,         KEY_0,          KEY_BACKSPACE,  0, 0,
  0, KEY_F2, KEY_F4, KEY_SEMICOLON, KEY_COMMA,      KEY_RIGHTBRACE, 0, 0,   
  0, KEY_3,  KEY_6,  KEY_9,         KEY_GRAVE,      KEY_KPENTER,    0, 0,
};

static unsigned long mst_keys_down[2];
static unsigned long mst_keys_cumul[2];

static void mst_handle_key(struct input_dev *dev, unsigned int key, int down)
{
	if (key >= 48 || key % 8 >= 6) {
		/* outside of the alphanum keypad -- send event right away */
		input_report_key( dev, mstkp_keycodes[key], down);
	} else 	if (down) {
		__set_bit(key, mst_keys_down);
		__set_bit(key, mst_keys_cumul);
	} else {
		__clear_bit(key, mst_keys_down);
		if ((mst_keys_down[0] | mst_keys_down[1]) == 0) {
			/* everything released at this point */
			int nbits = hweight32(mst_keys_cumul[0])
				  + hweight32(mst_keys_cumul[1]);
			if (nbits == 1) {
				/* this was a single switch key */
				input_report_key(dev, mstkp_keycodes[key], 1);
				input_report_key(dev, mstkp_keycodes[key], 0);
			} else if (nbits == 2) {
				int x, y;
				mst_keys_cumul[0] ^= ~0;
				mst_keys_cumul[1] ^= ~0;
				x = find_first_zero_bit(mst_keys_cumul, 64);
				y = find_next_zero_bit(mst_keys_cumul, 64, x+1);
				if (x < 38 && y - x == 7) {
					/* we have a valid multiswitch key */
					input_report_key(dev, mstkp_multicodes[x], 1);
					input_report_key(dev, mstkp_multicodes[x], 0);
				}
			}
			mst_keys_cumul[0] = 0;
			mst_keys_cumul[1] = 0;
		}
	}
}

#endif  /* MULTISWITCH */

static void pxakp_interrupt(int irq, void *ptr, struct pt_regs *regs) 
{
	struct input_dev *dev = ptr;
	unsigned long kpc = KPC;

	if (kpc & KPC_DI) {
		unsigned long kpdk = KPDK;
		if (!(kpdk & KPDK_DKP)) {
			/* better luck next time */
		} else if (kpc & KPC_REE0) {
			int rel;
			unsigned long kprec = KPREC;
			KPREC = 0x7f;
			if (kprec & KPREC_OF0) {
				rel = (kprec & 0xff) + 0x7f;
			} else if (kprec & KPREC_UF0) {
				rel = (kprec & 0xff) - 0x7f - 0xff;
			} else {
				rel = (kprec & 0xff) - 0x7f;
			}
			if (rel) {
				//printk("wheel0: %d\n", kpdk, rel);
				input_report_rel(dev, REL_WHEEL, rel);
			}
		}
	}

	if (kpc & KPC_MI) {
		static unsigned long prev_mkp[4] = {0, 0, 0, 0};
		unsigned long mkp[4] = {KPASMKP0, KPASMKP1, KPASMKP2, KPASMKP3};
		unsigned long stat, flip;
		unsigned int col, row;
		for (col = 0; col < 8; col += 2) {
			stat = mkp[col/2];
			if (stat & KPASMKPx_SO)
				continue;
			flip = stat ^ prev_mkp[col/2];
			prev_mkp[col/2] = stat;
			while (flip) {
				row = __ffs(flip);
				if (row >= 8) {
					stat >>= 16;
					flip >>= 16;
					col |= 1;
					continue;
				}
				//printk( "key %d %s\n", row*8 + col,
				//        (stat & (1 << row)) ? "down" : "up" );
#ifndef MULTISWITCH
				input_report_key( dev, 
						  mstkp_keycodes[row*8 + col],
						  stat & (1 << row) );
#else
				mst_handle_key( dev, row*8 + col, 
						stat & (1 << row) );
#endif
				flip &= ~(1 << row);
			}
			col &= ~1;
		}
	}
}

static int pxakp_open(struct input_dev *dev)
{
	if (dev->private++ == 0) {
		/* Set keypad control register */
		KPC = (KPC_ASACT | (7<<26) | (7<<23) | KPC_MS_ALL |
		       (2<<6) | KPC_REE0 | KPC_DK_DEB_SEL |
		       KPC_ME | KPC_MIE | KPC_DE | KPC_DIE);

		/* Set rotary count to mid-point value */
		KPREC = 0x7F;

		/* Enable unit clock */
		CKEN |= CKEN19_KEYPAD;
	}

	return 0;
}

static void pxakp_close(struct input_dev *dev)
{
	if (--dev->private == 0) {
		/* Disable clock unit */
		CKEN &= ~CKEN19_KEYPAD;

		/* Disable keypad */
		KPC = 0;
	}
}

static struct input_dev *pxakp_dev;

static int __init pxakp_init(void)
{
	int ret, i;

	pxakp_dev = kmalloc(sizeof(*pxakp_dev), GFP_KERNEL);
	if (!pxakp_dev)
		return -ENOMEM;
	memset(pxakp_dev, 0, sizeof(*pxakp_dev));

	ret = request_irq(IRQ_KEYPAD, pxakp_interrupt, 0, "keypad", pxakp_dev);
	if (ret < 0) {
		kfree(pxakp_dev);
		return ret;
	}

	/* Setup GPIOs */
	set_GPIO_mode( 93 | GPIO_ALT_FN_1_IN);	/* KP_DKIN0 */
	set_GPIO_mode( 94 | GPIO_ALT_FN_1_IN);	/* KP_DKIN1 */
	set_GPIO_mode( 95 | GPIO_ALT_FN_3_IN);	/* KP_MKIN6 */
	set_GPIO_mode( 96 | GPIO_ALT_FN_3_OUT);	/* KP_MKOUT6 */
	set_GPIO_mode( 97 | GPIO_ALT_FN_3_IN);	/* KP_MKIN3 */
	set_GPIO_mode( 98 | GPIO_ALT_FN_3_IN);	/* KP_MKIN4 (?) */
	set_GPIO_mode( 99 | GPIO_ALT_FN_3_IN);	/* KP_MKIN5 */
	set_GPIO_mode(100 | GPIO_ALT_FN_1_IN);	/* KP_MKIN0 */
	set_GPIO_mode(101 | GPIO_ALT_FN_1_IN);	/* KP_MKIN1 */
	set_GPIO_mode(102 | GPIO_ALT_FN_1_IN);	/* KP_MKIN2 */
	set_GPIO_mode(103 | GPIO_ALT_FN_2_OUT);	/* KP_MKOUT0 */
	set_GPIO_mode(104 | GPIO_ALT_FN_2_OUT);	/* KP_MKOUT1 */
	set_GPIO_mode(105 | GPIO_ALT_FN_2_OUT);	/* KP_MKOUT2 */
	set_GPIO_mode(106 | GPIO_ALT_FN_2_OUT);	/* KP_MKOUT3 */
	set_GPIO_mode(107 | GPIO_ALT_FN_2_OUT);	/* KP_MKOUT4 */
	set_GPIO_mode(108 | GPIO_ALT_FN_2_OUT);	/* KP_MKOUT5 */

	/* Set default debounce time */
	KPKDI = ((0 << 8) | (100 << 0));

	pxakp_dev->name = "keypad";
	pxakp_dev->idbus = BUS_ONCHIP;
	pxakp_dev->open = pxakp_open;
	pxakp_dev->close = pxakp_close;

	__set_bit(EV_REL, pxakp_dev->evbit);
	__set_bit(EV_KEY, pxakp_dev->evbit);
	__set_bit(REL_WHEEL, pxakp_dev->relbit);
	for (i = 0; i < ARRAY_SIZE(mstkp_keycodes); i++)
		__set_bit(mstkp_keycodes[i], pxakp_dev->keybit);
#ifdef MULTISWITCH
	for (i = 0; i < ARRAY_SIZE(mstkp_multicodes); i++)
		__set_bit(mstkp_multicodes[i], pxakp_dev->keybit);
#endif

	input_register_device(pxakp_dev);

#ifdef CONFIG_DPM
	pxakpd_ldm_register();
#endif /* CONFIG_DPM */ 

	return 0;
}

static void __exit pxakp_exit(void)
{
#ifdef CONFIG_DPM
	pxakpd_ldm_unregister();
#endif /* CONFIG_DPM */

	free_irq(IRQ_KEYPAD, pxakp_dev);
	input_unregister_device(pxakp_dev);
	kfree(pxakp_dev);
}

module_init(pxakp_init);
module_exit(pxakp_exit);

MODULE_DESCRIPTION("Intel Mainstone KEYPAD driver");
MODULE_LICENSE("GPL");
