#ifndef __LINUX_KEYPAD_H
#define __LINUX_KEYPAD_H

/*
 * include/linux/keypad.h
 *
 * Copyright (C) 2004-2005 Motorola, Inc.
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

/* Revision History
 *
 * Author       Date            Number  Description of Change
 * ------       ----            -----   ---------------------
 * e12051       11/25/2004      xx      add TST CMD ioctl micros
 * e12051       01/20/2005      xx      re-define some key name
 */

#include <linux/ioctl.h>

#define KEYDOWN		0x8000
#define KEYUP		0x0000

typedef enum {
        FLIP_OPEN = 0,
        FLIP_CLOSE
}FLIP_STATUS_E;

typedef enum {
        SCREEN_LOCK_UNLOCKED = 0,
        SCREEN_LOCK_LOCKED
}SCREEN_LOCK_STATUS_E;

#define EVENTSIZE       sizeof(unsigned short)
#define KEY_IS_DOWN(x)  ((x) & KEYDOWN)
#define KEYCODE(x)      ((x) & (KEYDOWN - 1))

struct autorepeatinfo {
        int r_repeat;                /* 1 to do autorepeat, 0 to not do it */
        int r_time_to_first_repeat;  /* time to first repeat in milliseconds */
        int r_time_between_repeats;  /* time between repeats in milliseconds */
};

#define KEYPAD_IOCTL_BASE    'k'

/* 
 * return the bitmap of keys currently down.
 * The bitmap is an array of NUM_WORDS_IN_BITMAP unsigned long words.
 * the bitmap looks like this:
 * 
 * word:        1                          0
 *  ----------------------------------------------------
 * | 63 62 ........... 33 32 | 31 30 ............. 1 0 |
 *  ----------------------------------------------------
 * Bit n corresponds to key code n.
 */

#define KEYPAD_IOC_GETBITMAP      _IOR(KEYPAD_IOCTL_BASE,1,unsigned long *)

#define NUM_WORDS_IN_BITMAP             2

/**
 * put the specified event on the keypad's input queue and update the bitmap.
 * The exact event as specified will be returned by read() when it
 * reaches the front of the queue.  A key press event consists of
 * the key code or'd with KEYDOWN.  A key release event consists of
 * just the key code.
 */
#define KEYPAD_IOC_INSERT_EVENT   _IOW(KEYPAD_IOCTL_BASE,2,unsigned short)

/**
 * have the driver interpret the specified scan register values as
 * if it had gotten an interrupt.  The passed-in argument is a pointer
 * to three consecutive 32-bit words in this order: KPAS, KPASMKP0, KPASMKP1.
 */
#define KEYPAD_IOC_INSERTKEYSCAN  _IOW(KEYPAD_IOCTL_BASE,3,unsigned long *)

/* get the hardware debounce interval (in milliseconds) */
#define KEYPAD_IOC_GET_DEBOUNCE_INTERVAL \
                                  _IOR(KEYPAD_IOCTL_BASE,4,unsigned short *)

/* set the hardware debounce interval (in milliseconds) */
#define KEYPAD_IOC_SET_DEBOUNCE_INTERVAL  \
                                  _IOW(KEYPAD_IOCTL_BASE,4,unsigned short)

/* get "Ignore Multiple Key Press" bit; 1 means they are ignored */
#define KEYPAD_IOC_GET_IMKP_SETTING _IOR(KEYPAD_IOCTL_BASE,5,unsigned char *)

/* set "Ignore Multiple Key Press" bit; 1 means they will be ignored */
#define KEYPAD_IOC_SET_IMKP_SETTING _IOW(KEYPAD_IOCTL_BASE,5,unsigned char)

/* 
 * specify what to do about autorepeat on held keys
 * the 3rd argument is a pointer to a struct autorepeat
 */
#define KEYPAD_IOC_SET_AUTOREPEAT _IOW(KEYPAD_IOCTL_BASE,6,unsigned long *)

/* get autorepeat */
#define KEYPAD_IOC_GET_AUTOREPEAT _IOR(KEYPAD_IOCTL_BASE,6,unsigned long *)

/* get recent key pressed */
#define KEYPAD_IOC_GET_RECENT_KEY _IOR(KEYPAD_IOCTL_BASE,7,unsigned short *)

/* get/set flip status:
 * FLIP_OPEN or
 * FLIP_CLOSE
 */
#define KEYPAD_IOC_GET_FLIP_STATUS _IOR(KEYPAD_IOCTL_BASE,8,unsigned short *)
#define KEYPAD_IOC_SET_FLIP_STATUS _IOW(KEYPAD_IOCTL_BASE,8,unsigned short)

/* get/set screen_lock status:
 * SCREEN_LOCK_LOCKED or
 * SCREEN_LOCK_UNLOCKED
 */
#define KEYPAD_IOC_GET_SCREEN_LOCK_STATUS \
        _IOR(KEYPAD_IOCTL_BASE, 9, unsigned short *)
#define KEYPAD_IOC_SET_SCREEN_LOCK_STATUS \
        _IOW(KEYPAD_IOCTL_BASE, 10, unsigned short)

#define KEYPADI_TURN_ON_LED		1
#define KEYPADI_TURN_OFF_LED		0
/*
 * the constant KEY_k is both the code for key k in an event returned by
 * read() and the bit position for key k in the bitmap returned by
 * ioctl(KEYPAD_IOC_GETBITMAP).
 */
#define KEYPAD_NONE             255

#define KEYPAD_0                0
#define KEYPAD_1                1
#define KEYPAD_2                2
#define KEYPAD_3                3
#define KEYPAD_4                4
#define KEYPAD_5                5
#define KEYPAD_6                6
#define KEYPAD_7                7
#define KEYPAD_8                8
#define KEYPAD_9                9
#define KEYPAD_POUND            10
#define KEYPAD_STAR             11

#define KEYPAD_NAV_UP           12
#define KEYPAD_NAV_DOWN         13
#define KEYPAD_NAV_LEFT         14
#define KEYPAD_NAV_RIGHT        15
#define KEYPAD_NAV_CENTER       16

#define KEYPAD_MENU             17
#define KEYPAD_SOFT_LEFT        18
#define KEYPAD_SOFT_RIGHT       19
#define KEYPAD_OK               20
#define KEYPAD_CANCEL           21
#define KEYPAD_CLEAR            22
#define KEYPAD_CARRIER          23
#define KEYPAD_MESSAGING        24

#define KEYPAD_CAMERA           25

#define KEYPAD_ACTIVATE         26
#define KEYPAD_FLIP             27
#define KEYPAD_SEND             28
#define KEYPAD_POWER            29
#define KEYPAD_HANGUP           30
#define KEYPAD_HOME             31

#define KEYPAD_VAVR             32

#define KEYPAD_PTT              33
#define KEYPAD_JOG_UP           34
#define KEYPAD_JOG_MIDDLE       35
#define KEYPAD_JOG_DOWN         36
#define KEYPAD_SIDE_UP          37
#define KEYPAD_SIDE_DOWN        38
#define KEYPAD_SIDE_SELECT      39

#define KEYPAD_HEADSET          40
#define KEYPAD_EMU_HEADSET      41

#define KEYPAD_AUD_PRV		42
#define KEYPAD_AUD_NEXT         43
#define KEYPAD_PLAY_PAUSE       44
#define KEYPAD_iTUNE		45

#define KEYPAD_SCREEN_LOCK      46

#define KEYPAD_MAXCODE          46

/* These are some interfaces to other kernel components */
#ifdef __KERNEL__

/* handler for EMU headset key event */
int emu_headset_key_handler(unsigned short up_down);

/* handler for headset key event */
int headset_key_handler(unsigned short up_down);

/* specially handle for PWR_END key*/
int power_key_event(unsigned short up_down);

#endif /* __KERNEL__ */
#endif /* __LINUX_KEYPAD_H */
