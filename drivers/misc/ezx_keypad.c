/*
 * linux/drivers/misc/keypad.c
 * 
 * Copyright (C) 2004-2005 Motorola
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 */

/*
 * Add new product support of Barbados
 * Jay Jia
 * w20091@motorola.com
 *
 * 2004-11-25 Yin Kangkai e12051@motorola.com (currently maintainer)
 * fix some bugs and re-organize the file.
 *
 * Modified by Roy King(e5537c@motorola.com) 2005/10/12 
 * add kernel panic triggger
 *
 */

/*
 * This driver is for three devices: /dev/keypad, /dev/keypadB and
 * /dev/keypadI.
 * /dev/keypad would be used for reading the key event buffer. 
 * It can be opened by one process at a time.
 * /dev/keypadB would be used for ioctl(KEYPAD_IOC_GETBITMAP).
 * It can be opened by one process at a time.
 * /dev/keypadI would be used for ioctl(KEYPAD_IOC_INSERT_EVENT).
 * It can be opened any number of times simultaneously.
 *
 * The bulverde specification is ambiguous about when interrupts happen
 * for released keys.  We were told by Intel that we won't
 * get interrupts for released keys except in the case when multiple
 * keys were down and they've all been released.  So we implemented
 * a timer to poll for changes in key states.
 *
 * The E680 P2 hardware gives us interrupts when any key is released,
 * whether it was the only key down or not, and whether it's the last
 * of multiple keys to be released or not.  We don't know whether this
 * behavior will continue in future hardware releases, so the release
 * timer is still in the code, #ifdef'd with USE_RELEASE_TIMER.  On the
 * P2 hardware, the code works with or without USE_RELEASE_TIMER defined.
 * If the release interrupts are always going to happen, we can remove
 * the #ifdef'd code.
 *
 * With the P2 hardware, the power key bit in the KPDK register is always
 * on.  We don't know if this is correct behavior or not, but in any case
 * it certainly causes trouble to have that key autorepeating indefinitely,
 * or to be checking indefinitely for the key to be released (if we're doing
 * release polling).
 *
 * For now, any_keys_down() returns 0 if the power key is the only one down.
 * . At interrupt time, if the power key is the only one down we don't
 *   set the autorepeat timer or release timer.
 * . When the release timer goes off, if the power key is the only one
 *   still down we don't set the timer again.
 * . When the autorepeat timer goes off, if the power key is the only
 *   one down we don't generate any events.
 * In autorepeat_timer_went_off, if there are non-power keys down, while we're
 * looking through the whole bitmap of keys down we ignore the power key.
 *
 * E680/A780 is finished.
 * Jay Jia
 *
 */

#include <linux/tty.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/bitops.h>
#include <linux/param.h>
#include <linux/pm.h>
#include <linux/apm_bios.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/arch/irqs.h>
#include <asm/arch/ezx.h>
#include "keypad.h"

#define KEYPAD_DEBUG    /* debug option */
//#define KEYPAD_TRIGGER  /* trigger panic log */

/*
 * This is the number of microseconds we wait before checking to see if any
 * keys have been released.
 */
#define RDELAY  200

/* Product specific defines */
#ifdef CONFIG_KEYPAD_BARBADOS
    /* 6 columns */
    #define KEYPAD_COLUMNS  6
    /* 5 rows */
    #define KEYPAD_ROWS     5
    /* 97, 98, 100, 101, 102 */
    #define PKSR_MASK       0xEC000
    /* 97, 98, 100, 101, 102 */
    #define PKWR_MASK       0xEC000
    /* 103 104 105 106 107 108 */
    #define PGSR3_MASK      0x1F80

#elif CONFIG_KEYPAD_MARTINIQUE
    /* 6 columns */
    #define KEYPAD_COLUMNS  6
    /* 5 rows */
    #define KEYPAD_ROWS     5
    /* 97, 98, 100, 101, 102 */
    #define PKSR_MASK       0xEC000
    /* 97, 98, 100, 101, 102 */
    #define PKWR_MASK       0xEC000
    /* 103 104 105 106 107 108 */
    #define PGSR3_MASK      0x1F80

#elif CONFIG_KEYPAD_HAINAN
    /* 6 columns */
    #define KEYPAD_COLUMNS  6
    /* 5 rows */
    #define KEYPAD_ROWS     5
    /* 97, 98, 100, 101, 102 */
    #define PKSR_MASK       0xEC000
    /* 97, 98, 100, 101, 102 */
    #define PKWR_MASK       0xEC000
    /* 103 104 105 106 107 108 */
    #define PGSR3_MASK      0x1F80

#elif CONFIG_KEYPAD_SUMATRA
    /* 6 columns */
    #define KEYPAD_COLUMNS  6
    /* 5 rows */
    #define KEYPAD_ROWS     5
    /* 97, 98, 100, 101, 102 */
    #define PKSR_MASK       0xEC000
    /* 97, 98, 100, 101, 102 */
    #define PKWR_MASK       0xEC000
    /* 103 104 105 106 107 108 */
    #define PGSR3_MASK      0x1F80

#endif

#if defined(CONFIG_KEYPAD_SUMATRA)
    #define SWITCH_STATUS_E             SCREEN_LOCK_STATUS_E
    #define SWITCH_LOCKED               SCREEN_LOCK_LOCKED
    #define SWITCH_UNLOCKED             SCREEN_LOCK_UNLOCKED
    #define EVENT_DEV_SWITCH            EVENT_DEV_KEY
    #define KEYPAD_SWITCH               KEYPAD_SCREEN_LOCK
    #define KEYPAD_IOC_GET_SWITCH_STATUS  KEYPAD_IOC_GET_SCREEN_LOCK_STATUS
    #define KEYPAD_IOC_SET_SWITCH_STATUS  KEYPAD_IOC_SET_SCREEN_LOCK_STATUS
#else
    #define SWITCH_STATUS_E             FLIP_STATUS_E
    #define SWITCH_LOCKED               FLIP_CLOSE
    #define SWITCH_UNLOCKED             FLIP_OPEN
    #define EVENT_DEV_SWITCH            EVENT_DEV_FLIP
    #define KEYPAD_SWITCH               KEYPAD_FLIP
    #define KEYPAD_IOC_GET_SWITCH_STATUS  KEYPAD_IOC_GET_FLIP_STATUS
    #define KEYPAD_IOC_SET_SWITCH_STATUS  KEYPAD_IOC_SET_FLIP_STATUS
#endif

/* keypad structure */
struct keypad_s {
        unsigned short  keybuf[KEYBUF_SIZE];    /* circular buffer */
        int             keybuf_start;
        int             keybuf_end;
        spinlock_t      keypad_lock;
        
        int             reading_opens;
        unsigned short  recent_key_value;
        SWITCH_STATUS_E switch_status;
        
        /* saved bitmap of keys */
        unsigned long   oldkeybitmap[NUM_WORDS_IN_BITMAP];
        /* current bitmap of keys */
        unsigned long   keybitmap[NUM_WORDS_IN_BITMAP];

        unsigned long   kpas;                   /* saved value of KPAS */
        unsigned long   kpdk;                   /* saved value of KPDK */
        unsigned long   kpasmkp0;               /* saved value of KPASMKP0 */
        unsigned long   kpasmkp1;               /* saved value of KPASMKP1 */
        unsigned long   kpasmkp2;               /* saved value of KPASMKP2 */

#ifdef CONFIG_PM
        int             kpc_res;
        int             kpkdi_res;
#endif        

        int             do_autorepeat;
        long            jiffies_to_first_repeat;
        long            jiffies_to_next_repeat;

        struct timer_list autorepeat_timer;
        struct timer_list switch_timer;
};

static struct keypad_s ezx_keypad = {
        keybuf_start:           0,
        keybuf_end:             0,
        keypad_lock:            SPIN_LOCK_UNLOCKED,

        reading_opens:          0,
        recent_key_value:       0,
        switch_status:          SWITCH_UNLOCKED,

        kpas:                   0,
        kpdk:                   0,
        kpasmkp0:               0,
        kpasmkp1:               0,
        kpasmkp2:               0,

#ifdef CONFIG_PM
        kpc_res:                0,
        kpkdi_res:              0,
#endif

        do_autorepeat:          1,
        jiffies_to_first_repeat:        30,
        jiffies_to_next_repeat:         30,
};


#define KEYBUF_EMPTY() (ezx_keypad.keybuf_start == ezx_keypad.keybuf_end)
#define KEYBUF_FULL()  (((ezx_keypad.keybuf_end + 1) % KEYBUF_SIZE) == \
                        ezx_keypad.keybuf_start)
#define KEYBUF_INC(x)  ((x) = (((x) + 1) % KEYBUF_SIZE))

/* local function */
static int keypad_open(struct inode *, struct file *);
static int keypad_close(struct inode *, struct file *);
static int keypad_ioctl(struct inode *, struct file *,
                        unsigned int, unsigned long);
static unsigned int keypad_poll(struct file *, poll_table *);
static ssize_t keypad_read(struct file *, char *, size_t, loff_t *);

static void kp_interrupt(int, void *, struct pt_regs *);
static void autorepeat_timer_went_off(unsigned long);

static int add_to_keybuf(unsigned short);
static int add_events(unsigned long *, unsigned long *);
static unsigned short get_from_keybuf(void);
static int scan_to_bitmap
    (unsigned long, unsigned long, unsigned long, unsigned long);

static inline void set_bitmap_to_zero(unsigned long bitmap[]);
static inline void copy_bitmap(unsigned long dest[],
                               unsigned long source[]);
static inline int turn_on_bit(unsigned long bitmap[], int);
static inline int any_keys_down(unsigned long *);


/* global function */
int power_key_event(unsigned short);
void power_key_pressed(void);
void power_key_released(void);
int headset_key_handler(unsigned short);

/*
 * code[c*8+r] is the key code for the key that's down when there's a 1
 * in column c, row r of the scan registers.
 */
#if defined(CONFIG_KEYPAD_BARBADOS)
static int scanbit_to_keycode[] = {
        /* col 0 */
        KEYPAD_6, KEYPAD_8, KEYPAD_7, KEYPAD_2,
        KEYPAD_4, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 1 */
        KEYPAD_NAV_RIGHT, KEYPAD_NAV_DOWN, KEYPAD_9, KEYPAD_NAV_UP,
        KEYPAD_NAV_LEFT, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 2 */
        KEYPAD_SIDE_DOWN, KEYPAD_NONE, KEYPAD_VAVR, KEYPAD_SEND,
        KEYPAD_SIDE_UP, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 3 */
        KEYPAD_NAV_CENTER, KEYPAD_SOFT_LEFT, KEYPAD_SOFT_RIGHT, KEYPAD_0,
        KEYPAD_POUND, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 4 */
        KEYPAD_5, KEYPAD_STAR, KEYPAD_CLEAR, KEYPAD_1,
        KEYPAD_3, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 5 */
        KEYPAD_CARRIER, KEYPAD_NONE, KEYPAD_SIDE_SELECT, KEYPAD_PTT,
        KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
};

#elif CONFIG_KEYPAD_MARTINIQUE
static int scanbit_to_keycode[] = {
        /* col 0 */
        KEYPAD_6, KEYPAD_8, KEYPAD_7, KEYPAD_2,
        KEYPAD_4, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 1 */
        KEYPAD_NAV_RIGHT, KEYPAD_NAV_DOWN, KEYPAD_9, KEYPAD_NAV_UP,
        KEYPAD_NAV_LEFT, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 2 */
        KEYPAD_SIDE_DOWN, KEYPAD_NONE, KEYPAD_VAVR, KEYPAD_SEND,
        KEYPAD_SIDE_UP, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 3 */
        KEYPAD_NAV_CENTER, KEYPAD_SOFT_LEFT, KEYPAD_SOFT_RIGHT, KEYPAD_0,
        KEYPAD_POUND, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 4 */
        KEYPAD_5, KEYPAD_STAR, KEYPAD_CLEAR, KEYPAD_1,
        KEYPAD_3, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 5 */
        KEYPAD_CARRIER, KEYPAD_NONE, KEYPAD_SIDE_SELECT, KEYPAD_PTT,
        KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
};

#elif CONFIG_KEYPAD_HAINAN
static int scanbit_to_keycode[] = {
        /* col 0 */
        KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 1 */
        KEYPAD_NAV_RIGHT, KEYPAD_NAV_DOWN, KEYPAD_NAV_CENTER, KEYPAD_NAV_UP,
        KEYPAD_NAV_LEFT, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 2 */
        KEYPAD_SIDE_DOWN, KEYPAD_CAMERA, KEYPAD_VAVR, KEYPAD_SEND,
        KEYPAD_SIDE_UP, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 3 */
        KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 4 */
        KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 5 */
        KEYPAD_NONE, KEYPAD_NONE, KEYPAD_SIDE_SELECT, KEYPAD_NONE,
        KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
};

#elif CONFIG_KEYPAD_SUMATRA
static int scanbit_to_keycode[] = {
        /* col 0 */
        KEYPAD_6, KEYPAD_8, KEYPAD_7, KEYPAD_2,
        KEYPAD_4, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 1 */
        KEYPAD_NAV_RIGHT, KEYPAD_NAV_DOWN, KEYPAD_NAV_CENTER, KEYPAD_NAV_UP,
        KEYPAD_NAV_LEFT, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 2 */
        KEYPAD_9, KEYPAD_NONE, KEYPAD_VAVR, KEYPAD_SEND,
        KEYPAD_POUND, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 3 */
        KEYPAD_AUD_NEXT, KEYPAD_SIDE_UP, KEYPAD_SIDE_DOWN, KEYPAD_PLAY_PAUSE,
        KEYPAD_AUD_PRV, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 4 */
        KEYPAD_5, KEYPAD_STAR, KEYPAD_CLEAR, KEYPAD_1,
        KEYPAD_3, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
        /* col 5 */
        KEYPAD_SOFT_LEFT, KEYPAD_SOFT_RIGHT, KEYPAD_0, KEYPAD_iTUNE,
        KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE, KEYPAD_NONE,
};

#endif


#ifdef CONFIG_PM
static struct pm_dev *pm_dev;
#endif

static DECLARE_WAIT_QUEUE_HEAD(keypad_wait);

static void ezx_set_gpio(void)
{
#ifdef CONFIG_KEYPAD_BARBADOS
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW0 | GPIO_ALT_FN_1_IN); /* KP_MKIN<0> */
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW1 | GPIO_ALT_FN_1_IN); /* KP_MKIN<1> */
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW2 | GPIO_ALT_FN_1_IN); /* KP_MKIN<2> */
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW3 | GPIO_ALT_FN_3_IN); /* KP_MKIN<3> */
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW4 | GPIO_ALT_FN_3_IN); /* KP_MKIN<4> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL0 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<0> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL1 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<1> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL2 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<2> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL3 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<3> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL4 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<4> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL5 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<5> */
        set_GPIO_mode(GPIO_FLIP_PIN | GPIO_IN);                  /* flip */
        set_GPIO_IRQ_edge(GPIO_FLIP_PIN,
                          GPIO_FALLING_EDGE | GPIO_RISING_EDGE);

#elif CONFIG_KEYPAD_MARTINIQUE
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW0 | GPIO_ALT_FN_1_IN); /* KP_MKIN<0> */
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW1 | GPIO_ALT_FN_1_IN); /* KP_MKIN<1> */
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW2 | GPIO_ALT_FN_1_IN); /* KP_MKIN<2> */
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW3 | GPIO_ALT_FN_3_IN); /* KP_MKIN<3> */
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW4 | GPIO_ALT_FN_3_IN); /* KP_MKIN<4> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL0 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<0> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL1 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<1> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL2 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<2> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL3 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<3> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL4 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<4> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL5 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<5> */
        set_GPIO_mode(GPIO_FLIP_PIN | GPIO_IN);                  /* flip */
        set_GPIO_IRQ_edge(GPIO_FLIP_PIN,
                          GPIO_FALLING_EDGE | GPIO_RISING_EDGE);

#elif CONFIG_KEYPAD_HAINAN
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW0 | GPIO_ALT_FN_1_IN); /* KP_MKIN<0> */
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW1 | GPIO_ALT_FN_1_IN); /* KP_MKIN<1> */
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW2 | GPIO_ALT_FN_1_IN); /* KP_MKIN<2> */
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW3 | GPIO_ALT_FN_3_IN); /* KP_MKIN<3> */
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW4 | GPIO_ALT_FN_3_IN); /* KP_MKIN<4> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL0 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<0> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL1 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<1> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL2 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<2> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL3 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<3> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL4 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<4> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL5 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<5> */
        set_GPIO_mode(GPIO_FLIP_PIN | GPIO_IN);                  /* flip */
        set_GPIO_IRQ_edge(GPIO_FLIP_PIN,
                          GPIO_FALLING_EDGE | GPIO_RISING_EDGE);


#elif CONFIG_KEYPAD_SUMATRA
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW0 | GPIO_ALT_FN_1_IN); /* KP_MKIN<0> */
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW1 | GPIO_ALT_FN_1_IN); /* KP_MKIN<1> */
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW2 | GPIO_ALT_FN_1_IN); /* KP_MKIN<2> */
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW3 | GPIO_ALT_FN_3_IN); /* KP_MKIN<3> */
        set_GPIO_mode(GPIO_MATRIX_KEY_ROW4 | GPIO_ALT_FN_3_IN); /* KP_MKIN<4> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL0 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<0> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL1 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<1> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL2 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<2> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL3 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<3> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL4 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<4> */
        set_GPIO_mode(GPIO_MATRIX_KEY_COL5 | GPIO_ALT_FN_2_OUT); /* KP_MKOUT<5> */
        set_GPIO_mode(GPIO_FLIP_PIN | GPIO_IN);                  /* flip */
        set_GPIO_IRQ_edge(GPIO_FLIP_PIN,
                          GPIO_FALLING_EDGE | GPIO_RISING_EDGE);

#endif /* GPIO setup */
        return;
}


/*
 * set all words in the specified bitmap to 0
 */
static inline void set_bitmap_to_zero(unsigned long bitmap[])
{
        int i;
        
        for (i = 0; i < NUM_WORDS_IN_BITMAP; i++) {
                bitmap[i] = 0;
        }
        
        return;
}

/*
 * copy source bitmap to destination bitmap
 */
static inline void copy_bitmap(unsigned long dest[],
                               unsigned long source[])
{
        int i;
        
        for (i = 0; i < NUM_WORDS_IN_BITMAP; i++) {
                dest[i] = source[i];
        }

        return;
}

/*
 * Turn on the bit position in map for the specified key code.
 *
 * Return -1 on error, 0 success.
 */
static inline int turn_on_bit(unsigned long bitmap[], int code)
{
        int word, pos;

        if (code > KEYPAD_MAXCODE) {
                printk(KERN_WARNING "<%s>Undefined keycode: %d\n", __FUNCTION__, code);
                return -1;
        }

        /* 
         * bitmap: |63 62 ... ... 32|31 30 ... 1 0|
         * code  :                32 31    ... 1 0 
         * So, we will set the "code"th bit to 1.
         */

        word = code / 32;
        pos = code % 32;
        bitmap[word] |= (1 << pos);

        return 0;
}


/*
 * Return 1 if any bits are down in map[]
 */
static inline int any_keys_down(unsigned long *bitmap)
{
        int i;
        
        for (i = 0; i < NUM_WORDS_IN_BITMAP; i++) {
                if (*bitmap++) {
                        return 1;
                }
        }
        
        return 0;
}


/*
 * Add the specified event to the key buffer.
 * This should be called with keypad_lock locked.
 */
static int add_to_keybuf(unsigned short event)
{
        struct keypad_s *p = &ezx_keypad;

#ifdef KEYPAD_DEBUG
        printk(KERN_INFO "add keypad event = 0x%04x\n", event);
#endif
        p->recent_key_value = event;

#if CONFIG_APM
        if ((KEYDOWN | KEYPAD_SWITCH) == event) 
                /* Flip Close */
                apm_event_notify(APM_EVENT_DEVICE, EVENT_DEV_SWITCH, DEV_OFF);
        else if ((KEYUP | KEYPAD_SWITCH) == event)
                /* Flip Open */
                apm_event_notify(APM_EVENT_DEVICE, EVENT_DEV_SWITCH, DEV_ON);
        else
                /* other keys */
                apm_event_notify(APM_EVENT_DEVICE, EVENT_DEV_KEY, 0);

#if defined(CONFIG_ARCH_EZX_BARBADOS_POWEROFF_TRIGGER)
	if(event == 0x802c)
		queue_apm_event(KRNL_TEST_POWEROFF, NULL);
#endif
#endif
        
        if (KEYBUF_FULL()) {
                printk(KERN_WARNING "keypad buf full, add failed.\n");
                return -ENOMEM;
        }

        p->keybuf[p->keybuf_end] = event;
        KEYBUF_INC(p->keybuf_end);
        
        
        return 0;
}

/*
 * Decode the specified scan register values into the keybitmap.
 * The bitmap will contain a 1 for each key that's down.
 *
 * Return 0 on success, -1 on error.
 */
static int
scan_to_bitmap(unsigned long kpas, unsigned long kpasmkp0,
               unsigned long kpasmkp1, unsigned long kpasmkp2)
{
        struct keypad_s *p = &ezx_keypad;
        
        int row, col;
        int bitnum;
        int ret = 0;
        unsigned long scanmap;
                
        set_bitmap_to_zero(p->keybitmap);

        if ((kpas & KPAS_MUKP) == MUKP_NO_KEYS) {
                return ret;
        }
        
        /* One key is pressed */
        if ((kpas & KPAS_MUKP) == MUKP_ONE_KEY) {
                row = (kpas & KPAS_RP) >> 4;
                col = kpas & KPAS_CP;
#ifdef KEYPAD_DEBUG
                printk(KERN_INFO "single key pressed, row:%d, col:%d\n", row, col);
#endif

                ret = turn_on_bit(p->keybitmap, 
                                scanbit_to_keycode[col * 8 + row]);

                return ret;
        }

        /* reach here if multiple keys are pressed */
#ifdef KEYPAD_TRIGGER
        /*
         * Multiple keys pressed, trigger for panic log.
         *
         * KPASMKP1 = 00011001(binary)
         *                 mkc3               mkc2
         *  ------------------------------------------- 
         *  |so|reserved|  8bits  |reserved| 00011001 |
         *  -------------------------------------------
         *
         * for HaiNan, this means:
         *             SIDE_UP, SIDE_DOWN and SEND
         * for Bartinique, this means:
         *	       SIDE_UP, SIDE_DOWN and SEND
         * for Sumatra, this means:
         *             KEYPAD_9, KEYPAD_POUND and SEND
         */
        if (kpasmkp1 == 0x19) {
        	/* trigger panic log */
        	printk("PANIC LOG TRIGGERED.\n");
		BUG();
	}
#endif

        scanmap = (kpasmkp0 & 0x1f) | ((kpasmkp0 & 0x1f0000) >> 8) |
                ((kpasmkp1 & 0x1f) << 16) | ((kpasmkp1 & 0x1f0000) << 8);
        
        while ((bitnum = ffs(scanmap)) != 0) {
                /*
                 * ffs returns bit numbers starting with 1, so subtract 1 to
                 * inde into scanbit_to_keycode[]
                 */
                ret = turn_on_bit(p->keybitmap, scanbit_to_keycode[bitnum - 1]);
                if (ret < 0)
                        return ret;
                scanmap &= ~(1 << (bitnum - 1));
        }

        scanmap = (kpasmkp2 & 0x1f) | ((kpasmkp2 & 0x1f0000) >> 8);
        while ((bitnum = ffs(scanmap)) != 0) {
                ret = turn_on_bit(p->keybitmap, 
                                scanbit_to_keycode[32 + bitnum - 1]);
                if (ret < 0)
                        return ret;
                scanmap &= ~(1 << (bitnum - 1));
        }

        return 0;
}

/* 
 * Add events indicated by the difference between the last scan (oldbitmap)
 * and this scan (newbitmap) to the input buffer.
 *
 * Return nonzero right away if any of the events can't be added.
 * Return zero if all the events were added.
 *
 * This should be called with keypad_lock locked.
 */
static int add_events(unsigned long *oldbitmap, unsigned long *newbitmap)
{
        unsigned long onebitmap;
        unsigned long change;
        int bitnum, i, ret;

        for (i = 0; i < NUM_WORDS_IN_BITMAP; i++) {
                change = oldbitmap[i] ^ newbitmap[i];
                while ((bitnum = ffs(change)) != 0) {
                        onebitmap = 1 << (bitnum - 1);
                        if ((oldbitmap[i] & onebitmap) == 0) {
                                ret = add_to_keybuf
                                        (((bitnum - 1) + 32 * i) | KEYDOWN);
                                if (ret < 0)
                                        return ret;
                        } else {
                                ret = add_to_keybuf
                                        (((bitnum - 1) + 32 * i) | KEYUP);
                                if (ret < 0)
                                        return ret;
                        }

                        change &= ~onebitmap;
                }
        }

        return 0;
}

static unsigned short get_from_keybuf(void)
{
        unsigned short event;
        unsigned long flags;

        struct keypad_s *p = &ezx_keypad;
        
        spin_lock_irqsave(&p->keypad_lock, flags);
        event = p->keybuf[p->keybuf_start];
        KEYBUF_INC(p->keybuf_start);
        spin_unlock_irqrestore(&p->keypad_lock, flags);
        return event;
}

#ifdef CONFIG_PM
static int button_pm_callback(struct pm_dev *pm_dev, pm_request_t req,
                              void *data)
{
        struct keypad_s *p = &ezx_keypad;
        unsigned long pksr = PKSR;

        PKSR = 0xFFFFF;
        
        switch (req) {
        case PM_SUSPEND:
                p->kpc_res = KPC;
                p->kpkdi_res = KPKDI;
                set_bitmap_to_zero(p->oldkeybitmap);
                set_bitmap_to_zero(p->keybitmap);
                p->kpas = 0;
                p->kpdk = 0;
                p->kpasmkp0 = 0;
                p->kpasmkp1 = 0;
                p->kpasmkp2 = 0;
                break;
                
        case PM_RESUME:
                KPC = p->kpc_res;
                KPKDI = p->kpkdi_res;

                if (pksr & PKSR_MASK) {
                        apm_event_notify(APM_EVENT_DEVICE, EVENT_DEV_KEY, 0);
                }
                break;
        default:
                printk(KERN_WARNING "<%s> unknow req.\n", __FUNCTION__);
                break;
        }
        
        return 0;
}
#endif

/*
 * This is called when the autorepeat timer goes off.
 */
static void autorepeat_timer_went_off(unsigned long unused)
{
        int i, bitnum;
        unsigned long tmp;
        unsigned long flags;

        struct keypad_s *p = &ezx_keypad;
        
        spin_lock_irqsave(&p->keypad_lock, flags);
        if (!any_keys_down(p->keybitmap)) {
                spin_unlock_irqrestore(&p->keypad_lock, flags);
                return;
        }
        
        for (i = 0; i < NUM_WORDS_IN_BITMAP; i++) {
                tmp = p->keybitmap[i];
                while ((bitnum = ffs(tmp)) != 0) {
                        (void) add_to_keybuf(((bitnum - 1) + 32 * i) | KEYDOWN);
                        tmp &= ~(1 << (bitnum - 1));
                }
        }
        
        spin_unlock_irqrestore(&p->keypad_lock, flags);
        wake_up_interruptible(&keypad_wait);

        mod_timer(&p->autorepeat_timer, jiffies + p->jiffies_to_next_repeat);

        return;
}

/* handling the switch event */
static void inline switch_status_handler(SWITCH_STATUS_E status)
{
        struct keypad_s *p = &ezx_keypad;

        p->switch_status = status;

        switch (status) {
        case SWITCH_LOCKED:
                add_to_keybuf(KEYDOWN | KEYPAD_SWITCH);
#ifdef KEYPAD_DEBUG
                printk(KERN_INFO "<keypad> switch locked\n");
#endif
                break;
        case SWITCH_UNLOCKED:
                add_to_keybuf(KEYUP | KEYPAD_SWITCH);
#ifdef KEYPAD_DEBUG 
                printk(KERN_INFO "<keypad> switch unlocked\n");
#endif
                break;
        default:
                break;
        }

        return;
}

static void inline switch_status_check(void)
{
        unsigned long data;

        data = GPLR(GPIO_FLIP_PIN) & GPIO_bit(GPIO_FLIP_PIN);
        if (!data)      /* flip close: 0 */
                switch_status_handler(SWITCH_LOCKED);
        else
                switch_status_handler(SWITCH_UNLOCKED);

        return;
}

/* when switch timeout, call this */
static void switch_timer_handler(unsigned long unused)
{
        switch_status_check();
        wake_up_interruptible(&keypad_wait);
        return;
}


static void switch_irq_handler(int irq, void *ptr, struct pt_regs *regs)
{
        struct timer_list *t = &(ezx_keypad.switch_timer);
        
        del_timer(t);
        t->expires = jiffies + 10;
        add_timer(t);

        return;
}


static void kp_interrupt(int irq, void *ptr, struct pt_regs *regs)
{
        unsigned long flags;
        unsigned long kpc_val;
        int ret;

        struct keypad_s *p = &ezx_keypad;
        
#ifdef KEYPAD_DEBUG
        printk(KERN_INFO "keypad interrupt occurred\n");
#endif

        del_timer(&p->autorepeat_timer);
        spin_lock_irqsave(&p->keypad_lock, flags);

        /* ack interrupt */
        kpc_val = KPC;

        /* matrix interrupt */
        if (kpc_val & KPC_MI) {
                /*
                 * The Intel driver turned on KPC_AS here.  It doesn't seem
                 * that we would need to do that, because getting an interrupt
                 * means that a scan was just done.  For now, I've commented
                 * out the setting and clearing of this bit.
                 */
                /* KPC |= KPC_AS; */

                while (KPAS & KPAS_SO) {
                        /* Wait for the Scan On bit to go off before
                         * reading the scan registers.
                         */
                };

                p->kpas = KPAS;
                p->kpasmkp0 = KPASMKP0;
                p->kpasmkp1 = KPASMKP1;
                p->kpasmkp2 = KPASMKP2;

        }

        /* direct interrupt */
        if (kpc_val & KPC_DI) {
                p->kpdk = KPDK;
                /* 
                 * reading the register turns off the "key pressed since last
                 * read" bit if it was on, so we turn it off 
                 */
                p->kpdk &= ~KPDK_DKP;
        }

        copy_bitmap(p->oldkeybitmap, p->keybitmap);
        ret = scan_to_bitmap(p->kpas, p->kpasmkp0, p->kpasmkp1, p->kpasmkp2);
        if (ret < 0)
                goto out;

        (void) add_events(p->oldkeybitmap, p->keybitmap);

        /*
         * If any keys are down, set a timer to check for key release
         * and one for autorepeat if that's on.
         */
        if (any_keys_down(p->keybitmap) && p->do_autorepeat) {
                p->autorepeat_timer.expires =
                        (jiffies + p->jiffies_to_first_repeat);
                add_timer(&p->autorepeat_timer);
        }

        wake_up_interruptible(&keypad_wait);

out:
        spin_unlock_irqrestore(&p->keypad_lock, flags);
        return;
}



static int keypad_open(struct inode *inode, struct file *file)
{
        struct keypad_s *p = &ezx_keypad;
        
#ifdef KEYPAD_DEBUG
        printk(KERN_DEBUG "keypad_open\n");
#endif

        spin_lock(&p->keypad_lock);
        if (p->reading_opens > 0) {
                spin_unlock(&p->keypad_lock);
                return -EBUSY;
        }
        p->reading_opens++;
        spin_unlock(&p->keypad_lock);

        return 0;
}

static int keypad_close(struct inode *inode, struct file *file)
{
        struct keypad_s *p = &ezx_keypad;
        
#ifdef KEYPAD_DEBUG
        printk(KERN_DEBUG "keypad_close\n");
#endif
        spin_lock(&p->keypad_lock);
        p->reading_opens--;
        spin_unlock(&p->keypad_lock);

        return 0;
}


static ssize_t
keypad_read(struct file *file, char *buf, size_t count, loff_t * ptr)
{
        int i, ret;
        unsigned short event;

#ifdef KEYPAD_DEBUG
        printk(KERN_DEBUG " keypad_read\n");
#endif
        /* Can't seek (pread) on this device */
        if (ptr != &file->f_pos) {
                return -ESPIPE;
        }

        if (count == 0) {
                return 0;
        }

        if (KEYBUF_EMPTY()) {
                /* buffer is empty */
                /* if not blocking return */
                if (file->f_flags & O_NONBLOCK) {
                        return -EAGAIN;
                }
                /* blocking, so wait for input */
                ret =
                    wait_event_interruptible(keypad_wait, !KEYBUF_EMPTY());
                if (ret) {
                        return ret;
                }
        }

        i = 0;
        /* copy events until we have what the user asked for or we run out */
        while ((i + 1 < count) && !KEYBUF_EMPTY()) {
                event = get_from_keybuf();
                if ((ret = put_user(event, (unsigned short *) buf)) != 0) {
                        return ret;
                }
                buf += EVENTSIZE;
                i += EVENTSIZE;
        }
        return i;
}



static int
keypad_ioctl(struct inode *inode, struct file *file,
             unsigned int cmd, unsigned long arg)
{
        struct keypad_s *p = &ezx_keypad;
        unsigned short status;
        
        int interval;                   /* debounce interval */
        int imkp;                       /* Ignore Multiple Key Press bit */
        struct autorepeatinfo ar;       /* autorepeat information */

#ifdef KEYPAD_DEBUG
        printk("keypad_ioctl: 0x%x\n", cmd);
#endif

        switch (cmd) {
        case KEYPAD_IOC_INSERT_EVENT:
                printk(KERN_WARNING "INSERT EVENT not support.\n");
                return -EINVAL;
                break;
                
        case KEYPAD_IOC_GET_DEBOUNCE_INTERVAL:
                interval = KPKDI & KPKDI_BITS;
                return put_user(interval, (unsigned long *)arg);
                break;
                
        case KEYPAD_IOC_SET_DEBOUNCE_INTERVAL:
                interval = (unsigned short)arg;
                if (interval > KPKDI_BITS) {
                        return -EINVAL;
                }
                
                KPKDI &= ~KPKDI_BITS;
                KPKDI |= interval;
                break;
                
        case KEYPAD_IOC_GET_IMKP_SETTING:
                imkp = ((KPC & KPC_IMKP) == KPC_IMKP);
                return put_user(imkp, (unsigned char *)arg);
                break;
                
        case KEYPAD_IOC_SET_IMKP_SETTING:
                imkp = (unsigned char)arg;
                if (imkp) {
                        KPC |= KPC_IMKP;
                } else {
                        KPC &= ~KPC_IMKP;
                }
                break;
                
        case KEYPAD_IOC_SET_AUTOREPEAT:
                if (copy_from_user(&ar, (void *)arg,
                                   sizeof(struct autorepeatinfo)) != 0) {
                        return -EFAULT;
                }
                p->do_autorepeat = ar.r_repeat;
                /* times are specified in milliseconds; convert to jiffies */
                p->jiffies_to_first_repeat =
                    ar.r_time_to_first_repeat * HZ / 1000;
                p->jiffies_to_next_repeat =
                    ar.r_time_between_repeats * HZ / 1000;
                break;

        case KEYPAD_IOC_GET_AUTOREPEAT:
                ar.r_repeat = p->do_autorepeat;
                ar.r_time_to_first_repeat =
                        p->jiffies_to_first_repeat * 1000 / HZ;
                ar.r_time_between_repeats =
                        p->jiffies_to_next_repeat * 1000 / HZ;
                if (copy_to_user((void *)arg, &ar,
                                 sizeof(struct autorepeatinfo)) != 0) {
                        return -EFAULT;
                }
                break;
                
        case KEYPAD_IOC_GET_RECENT_KEY:  /*recent key value */
                return put_user(p->recent_key_value, (unsigned short *)arg);
                break;
                
        case KEYPAD_IOC_GET_SWITCH_STATUS: /* get switch status */
                return put_user(p->switch_status, (unsigned short *)arg);
                break;

        case KEYPAD_IOC_SET_SWITCH_STATUS: /* set switch status */
                status = (unsigned short)arg;
                if ((status != SWITCH_LOCKED) && (status != SWITCH_UNLOCKED))
                        return -EINVAL;
                switch_status_handler(status);
                break;
                
        default:
                return -ENOTTY;
        }
        
        return 0;
}

static unsigned int keypad_poll(struct file *file, poll_table * wait)
{
        poll_wait(file, &keypad_wait, wait);

        if (!KEYBUF_EMPTY()) {
                return (POLLIN | POLLRDNORM);
        }
        return 0;
}


/* for /dev/keypad */
static struct file_operations keypad_fops = {
      read:             keypad_read,
      llseek:           no_llseek,
      poll:             keypad_poll,
      ioctl:            keypad_ioctl,
      open:             keypad_open,
      release:          keypad_close,
};

static struct miscdevice keypad_misc_device = {
        KEYPAD_MINOR,
        KEYPAD_NAME,
        &keypad_fops,
};

static int
__init keypad_init(void)
{
        int err;
        struct keypad_s *p = &ezx_keypad;
        
#ifdef KEYPAD_DEBUG
        printk(KERN_DEBUG "keypad_init\n");
#endif

        set_bitmap_to_zero(p->oldkeybitmap);
        set_bitmap_to_zero(p->keybitmap);

        /* set up gpio */
        ezx_set_gpio();

        /* set keypad control register */
        KPC = (KPC_ASACT |      /* automatic scan on activity */
               KPC_ME    |      /* matrix  keypad enabled */
               ((KEYPAD_COLUMNS - 1) << 23) |
               ((KEYPAD_ROWS - 1) << 26)    |
               KPC_MS7_0);      /* scan all columns */

        CKEN |= CKEN19_KEYPAD;

        err = request_irq(IRQ_KEYPAD, kp_interrupt, 0, "Keypad", NULL);
        if (err) {
                printk(KERN_CRIT
                       "can't register IRQ%d for keypad, error %d\n",
                       IRQ_KEYPAD, err);
                CKEN &= ~CKEN19_KEYPAD;
                return -ENODEV;
        }

        if (request_irq
            (IRQ_GPIO(GPIO_FLIP_PIN), switch_irq_handler, SA_INTERRUPT, "flip", NULL)) {
                printk(KERN_WARNING "<keypad> switch irq is free.\n");
                return -EIO;
        }

        if (misc_register(&keypad_misc_device)) {
                printk(KERN_ERR "Couldn't register keypad driver\n");
                return -EIO;
        }

        init_timer(&p->switch_timer);
        p->switch_timer.function = switch_timer_handler;
        init_timer(&p->autorepeat_timer);
        p->autorepeat_timer.function = autorepeat_timer_went_off;
        
        /* get switch status */
        switch_status_check();

#ifdef CONFIG_PM
        pm_dev = pm_register(PM_SYS_DEV, 0, button_pm_callback);
        PKWR = PKWR_MASK;
        PGSR3 |= PGSR3_MASK;
#endif

        KPC |= (KPC_MIE);
        KPKDI = 0x20;
        
        return 0;
}

static void __exit keypad_exit(void)
{
        struct keypad_s *p = &ezx_keypad;
        
#ifdef KEYPAD_DEBUG
        printk(KERN_DEBUG " keypad_exit\n");
#endif
        misc_deregister(&keypad_misc_device);

#ifdef CONFIG_PM
        pm_unregister(pm_dev);
#endif

        CKEN &= ~CKEN19_KEYPAD;
        del_timer(&p->switch_timer);
        free_irq(IRQ_GPIO(GPIO_FLIP_PIN), NULL);

        return;
}

module_init(keypad_init);
module_exit(keypad_exit);

/* ----------- some global funtions ------------- */

/* handler for EMU headset key event */
int emu_headset_key_handler(unsigned short up_down)
{
        int ret = -1;
        if ((up_down != KEYUP) && (up_down != KEYDOWN))
                return ret;
        ret = add_to_keybuf(up_down | KEYPAD_EMU_HEADSET);
        wake_up_interruptible(&keypad_wait);
        return ret;
}

/* handler for headset key event */
int headset_key_handler(unsigned short up_down)
{
        int ret = -1;
        if ((up_down != KEYUP) && (up_down != KEYDOWN))
                return ret;
        ret = add_to_keybuf(up_down | KEYPAD_HEADSET);
        wake_up_interruptible(&keypad_wait);
        return ret;
}

/* specially handle for PWR_END key*/
int power_key_event(unsigned short up_down)
{
        int ret = -1;
        if ((up_down != KEYUP) && (up_down != KEYDOWN))
                return ret;
        ret = add_to_keybuf(up_down | KEYPAD_HANGUP);
        wake_up_interruptible(&keypad_wait);
        return ret;
}

void power_key_pressed(void)
{
        add_to_keybuf(KEYDOWN | KEYPAD_HANGUP);
        
        wake_up_interruptible(&keypad_wait);
        return;
}

void power_key_released(void)
{
        add_to_keybuf(KEYUP | KEYPAD_HANGUP);
        
        wake_up_interruptible(&keypad_wait);
        return;
}


