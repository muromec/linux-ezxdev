/*
 * Keylight module for EZX based platform.
 *
 * Copyright (c) 2005 Motorola
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
 * Maintainer: Yin Kangkai <kkyin@motorola.com>
 */
                                                                              
#include <linux/init.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/keylight.h>
#include <linux/power_ic.h>
#include <linux/lights_backlight.h>

#include <asm/uaccess.h>

#define KEYLIGHT_OPENED   0x1
static unsigned long keylight_status = 0;

spinlock_t keylight_lock = SPIN_LOCK_UNLOCKED;


static int keylight_open(struct inode *inode, struct file *file)
{
        /* Can be opened by one process */
        spin_lock(&keylight_lock);
        if (keylight_status & KEYLIGHT_OPENED) {
                spin_unlock(&keylight_lock);
                return -EBUSY;
        }

        keylight_status |= KEYLIGHT_OPENED;
        spin_unlock(&keylight_lock);
        return 0;
}

static int keylight_close(struct inode *inode, struct file *file)
{
        spin_lock(&keylight_lock);
        keylight_status &= ~KEYLIGHT_OPENED;
        spin_unlock(&keylight_lock);
        return 0;
}

static int keylight_ioctl(struct inode *inode, struct file *file,
                                  unsigned int cmd, unsigned long arg)
{
        LIGHTS_BACKLIGHT_SET_T set;
        uint8_t bri;

        switch (cmd) {
        default:
                printk("invalid cmd.\n");
                return -EINVAL;
        case KEYLIGHT_MAIN_ON:
                /* 
                 * since we do not save the previous brightness,
                 * we set to 255.
                 */
                bri = 255;

                /* and LIGHTS_BACKLIGHT_NAV ? */
                set.bl_select = LIGHTS_BACKLIGHT_KEYPAD;
                set.bl_brightness = bri;
                lights_backlightset(set.bl_select, set.bl_brightness);

                break;
        case KEYLIGHT_MAIN_OFF:
                bri = 0;

                set.bl_select = LIGHTS_BACKLIGHT_KEYPAD;
                set.bl_brightness = bri;
                lights_backlightset(set.bl_select, set.bl_brightness);

                break;
        case KEYLIGHT_MAIN_SETTING:
        case KEYLIGHT_SET_BRIGHTNESS:
                bri = (uint8_t) arg;

                /* and LIGHTS_BACKLIGHT_NAV ? */
                set.bl_select = LIGHTS_BACKLIGHT_KEYPAD;
                set.bl_brightness = bri;
                lights_backlightset(set.bl_select, set.bl_brightness);

                break;
        case KEYLIGHT_AUX_ON:
        case KEYLIGHT_AUX_OFF:
        case KEYLIGHT_AUX_SETTING:
                printk("obsolete cmd.\n");
                return -EINVAL;
        }
                                                                              
        return 0;
}

/* This structure defines the file operations for the backlight device */
static struct file_operations keylight_fops =
{
        .owner          = THIS_MODULE,
        .open           = keylight_open,
        .ioctl          = keylight_ioctl,
        .release        = keylight_close,
};

static struct miscdevice keylight_misc_device = {
        KEYLIGHT_MINOR,
        KEYLIGHT_DEV_NAME,
        &keylight_fops,
};

static int __init keylight_init(void)
{
        if (misc_register(&keylight_misc_device)) {
                printk(KERN_ERR "Couldn't register keylight misc driver.\n");
                return -EIO;
        }

        return 0;
}

static void __exit keylight_exit(void)
{
        misc_deregister(&keylight_misc_device);
        return;
}

module_init(keylight_init);
module_exit(keylight_exit);

