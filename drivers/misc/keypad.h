#ifndef __DRIVER_CHAR_KEYPAD_H
#define __DRIVER_CHAR_KEYPAD_H

/*
 * linux/drivers/char/keypad.h
 *
 * Copyright (C) 2003-2005 Motorola
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
 * Origin author: Jay Jia (w20091@motorola.com)
 * Current maitainer: Yin Kangkai (e12051@motorola.com)
 */

#include <asm/hardware.h>
#include <linux/keypad.h>

/**
 * /dev/keypad is the main keypad device.  Opening this device provides access
 * for read(), ioctl() and poll()/select().  Only one process at a time
 * can open this device.
 */
#define KEYPAD_NAME         "keypad"
#define KEYPAD_MINOR        0xf0     /* 240 */

/**
 * /dev/keypadB can be opened for ioctl() and poll()/select().
 * Only one process at a time can open this device.
 */
#define KEYPAD_BITMAP_NAME  "keypadB"
#define KEYPAD_BITMAP_MINOR 0xf1     /* 241 */

/**
 * /dev/keypadI can be opened for ioctl(INSERT_EVENT).
 * Any number of devices can open this simultaneously.
 */
#define KEYPAD_INSERT_NAME  "keypadI"
#define KEYPAD_INSERT_MINOR 0xf2     /* 242 */

/* 
 * KEYBUF_SIZE must be a power of 2 to avoid performance-destroying
 * integer divisions in the compiled code.
 * Each keypad event is a short, so a buffer n bytes long can hold
 * n/2 keypad events.
 */
#define KEYBUF_SIZE 256

#define KPC      __REG(0x41500000) /* control register */
#define KPDK     __REG(0x41500008) /* direct key register */
#define KPAS     __REG(0x41500020) /* automatic scan register */
#define KPASMKP0 __REG(0x41500028) /* auto scan multiple key press reg 0 */
#define KPASMKP1 __REG(0x41500030) /* auto scan multiple key press reg 1 */
#define KPKDI    __REG(0x41500048) /* debounce interval register */

#define KPC_RSRVD0 0x80000000 /* reserved */
#define KPC_AS     0x40000000 /* automatic scan */
#define KPC_ASACT  0x20000000 /* automatic scan on activity */
#define KPC_MKRN   0x1c000000 /* number of matrix keypad rows - 1 */
#define KPC_MKCN   0x03800000 /* number of matrix keypad columns - 1 */
#define KPC_MI     0x00400000 /* matrix interrupt, reset when read */
#define KPC_IMKP   0x00200000 /* ignore multiple key press */
#define KPC_MS7_0  0x001fe000 /* MS7-MS0 assert to scan columns */
#define KPC_MS7    0x00100000 /* MS7 assert to scan column */
#define KPC_MS6    0x00080000 /* MS6 assert to scan column */
#define KPC_MS5    0x00040000 /* MS5 assert to scan column */
#define KPC_MS4    0x00020000 /* MS4 assert to scan column */
#define KPC_MS3    0x00010000 /* MS3 assert to scan column */
#define KPC_MS2    0x00008000 /* MS2 assert to scan column */
#define KPC_MS1    0x00004000 /* MS1 assert to scan column */
#define KPC_MS0    0x00002000 /* MS0 assert to scan column */
#define KPC_ME     0x00001000 /* matrix keypad enable */
#define KPC_MIE    0x00000800 /* matrix interrupt enable */
#define KPC_RSRVD1 0x00000600 /* reserved */
#define KPC_DKN    0x000001c0 /* number of direct keys - 1 */
#define KPC_DI     0x00000020 /* direct interrupt bit, reset when read */
#define KPC_RSRVD2 0x00000010 /* reserved */
#define KPC_REE1   0x00000008 /* rotary encoder 1 enable */
#define KPC_REE0   0x00000004 /* rotary encoder 0 enable */
#define KPC_DE     0x00000002 /* direct keypad enable */
#define KPC_DIE    0x00000001 /* direct keypad interrupt enable */

#define KPC_7rows  0x18000000 /* 6 in KPC_MKRN */
#define KPC_4cols  0x01800000 /* 3 in KPC_MKCN */

#define KPKDI_BITS 0xff       /* bits in KPKDI register for debounce interval */

#define KPAS_MUKP    0x7c000000 /* KPAS bits 30-26 */
#define MUKP_NO_KEYS 0x00000000 /* value of MUKP if no keys are down */
#define MUKP_ONE_KEY 0x04000000 /* value of MUKP if one key is down */
#define KPAS_SO 0x80000000  /* scan on bit */
#define KPAS_RP 0x000000f0  /* row of single key    */
#define KPAS_CP 0x0000000f  /* column of single key */

/*
 * bits in Direct Key register
 */
#define KPDK_DKP     (0x1 << 31) /* on if direct key pressed since last read */
#define KPDK_DK7     (0x1 <<  7)
#define KPDK_DK6     (0x1 <<  6)
#define KPDK_DK5     (0x1 <<  5)
#define KPDK_DK4     (0x1 <<  4)
#define KPDK_DK3     (0x1 <<  3)
#define KPDK_DK2     (0x1 <<  2)
#define KPDK_DK1     (0x1 <<  1)
#define KPDK_DK0     (0x1 <<  0)

#endif /* __DRIVER_CHAR_KEYPAD_H */
