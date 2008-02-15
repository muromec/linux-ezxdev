/*
*
* Keyboard scancode definitions for the iPAQ H3600
*
* Copyright 2002 Compaq Computer Corporation.
*
* Use consistent with the GNU GPL is permitted,
* provided that this copyright notice is
* preserved in its entirety in all copies and derived works.
*
* COMPAQ COMPUTER CORPORATION MAKES NO WARRANTIES, EXPRESSED OR IMPLIED,
* AS TO THE USEFULNESS OR CORRECTNESS OF THIS CODE OR ITS
* FITNESS FOR ANY PARTICULAR PURPOSE.
*
* This file serves as a common repository for keyboard scancodes
* on the iPAQ.  The kernel does a one-to-one mapping between scancodes and
* keycodes by default, so these are also keycodes.
*
* 
*

* Author: Andrew Christian
*/


#ifndef __H3600_KEYBOARD_H__
#define __H3600_KEYBOARD_H__

/* These are the default scancodes for the buttons on the iPAQ itself */
#define H3600_SCANCODE_RECORD   120
#define H3600_SCANCODE_CALENDAR 122
#define H3600_SCANCODE_CONTACTS 123
#define H3600_SCANCODE_Q        124
#define H3600_SCANCODE_START    125
#define H3600_SCANCODE_UP       103 /* keycode up */
#define H3600_SCANCODE_RIGHT    106 /* keycode right */
#define H3600_SCANCODE_LEFT     105 /* keycode left */
#define H3600_SCANCODE_DOWN     108 /* keycode down */
#define H3600_SCANCODE_ACTION   96  /* keycode keypad enter */ /* 28 is regular enter, 126 is rocker enter */
#define H3600_SCANCODE_SUSPEND  121  /* keycode powerdown */

#define H3600_SCANCODE_ENVELOPE H3600_SCANCODE_Q /* Same as "Q" - appears on models 3700 & 3800 */

/* 
   These keycodes are the defaults used by a standard PC keyboard 
   We include them here to make it easier to define plug-on keyboards,
   for example the Targus Stowaway and the Compaq Microkeyboard
*/

#define SKEY_ESC		1		
#define SKEY_1			2		
#define SKEY_2			3		
#define SKEY_3			4		
#define SKEY_4			5		
#define SKEY_5			6		
#define SKEY_6			7		
#define SKEY_7			8		
#define SKEY_8			9		
#define SKEY_9			10		
#define SKEY_0			11		
#define SKEY_MINUS		12		
#define SKEY_EQUAL		13		
#define SKEY_BACKSPACE		14		/* Technically delete */
#define SKEY_TAB		15		
#define SKEY_Q			16		
#define SKEY_W			17		
#define SKEY_E			18		
#define SKEY_R			19		
#define SKEY_T			20		
#define SKEY_Y			21		
#define SKEY_U			22		
#define SKEY_I			23		
#define SKEY_O			24		
#define SKEY_P			25		
#define SKEY_LEFTBRACE		26		/* Bracket-left */
#define SKEY_RIGHTBRACE		27		/* Bracket-right */
#define SKEY_ENTER		28		/* Return */
#define SKEY_LEFTCTRL		29		/* Control */
#define SKEY_A			30		
#define SKEY_S			31		
#define SKEY_D			32		
#define SKEY_F			33		
#define SKEY_G			34		
#define SKEY_H			35		
#define SKEY_J			36		
#define SKEY_K			37		
#define SKEY_L			38		
#define SKEY_SEMICOLON		39		
#define SKEY_APOSTROPHE		40		
#define SKEY_GRAVE		41		
#define SKEY_LEFTSHIFT		42		/* Shift */
#define SKEY_BACKSLASH		43		
#define SKEY_Z			44		
#define SKEY_X			45		
#define SKEY_C			46		
#define SKEY_V			47		
#define SKEY_B			48		
#define SKEY_N			49		
#define SKEY_M			50		
#define SKEY_COMMA		51		
#define SKEY_DOT		52		/* period */
#define SKEY_SLASH		53		
#define SKEY_RIGHTSHIFT		54		/* Shift */
#define SKEY_KPASTERISK		55		/* KP_Multiply */
#define SKEY_LEFTALT		56		/* Alt */
#define SKEY_SPACE		57		
#define SKEY_CAPSLOCK		58		
#define SKEY_F1			59		
#define SKEY_F2			60		
#define SKEY_F3			61		
#define SKEY_F4			62		
#define SKEY_F5			63		
#define SKEY_F6			64		
#define SKEY_F7			65		
#define SKEY_F8			66		
#define SKEY_F9			67		
#define SKEY_F10		68		
#define SKEY_NUMLOCK		69		
#define SKEY_SCROLLLOCK		70		
#define SKEY_KP7		71		
#define SKEY_KP8		72		
#define SKEY_KP9		73		
#define SKEY_KPMINUS		74		
#define SKEY_KP4		75		
#define SKEY_KP5		76		
#define SKEY_KP6		77		
#define SKEY_KPPLUS		78		/* KP_Add */
#define SKEY_KP1		79		
#define SKEY_KP2		80		
#define SKEY_KP3		81		
#define SKEY_KP0		82		
#define SKEY_KPDOT		83		/* KP_Period */
#define SKEY_LASTCONSOLE	84      /* Last Console -- 103rd */
#define SKEY_UNASSIGNED_1	85
#define SKEY_102ND		86	/* < > */	
#define SKEY_F11		87		
#define SKEY_F12		88		
#define SKEY_UNASSIGNED_2	89
#define SKEY_UNASSIGNED_3	90	/* -- unassigned -- */			
#define SKEY_UNASSIGNED_4	91	/* -- unassigned -- */			
#define SKEY_UNASSIGNED_5	92	/* -- unassigned -- */			
#define SKEY_UNASSIGNED_6	93	/* -- unassigned -- */			
#define SKEY_UNASSIGNED_7	94	/* -- unassigned -- */			
#define SKEY_UNASSIGNED_8	95	/* -- unassigned -- */			
#define SKEY_KPENTER		96		
#define SKEY_RIGHTCTRL		97		/* Control */
#define SKEY_KPSLASH		98		/* KP_Divide */
#define SKEY_SYSRQ		99	/* Control_backslash */
#define SKEY_RIGHTALT		100		/* AltGr */
#define SKEY_LINEFEED		101     /* Break */
#define SKEY_HOME		102	/* Find  */
#define SKEY_UP			103	
#define SKEY_PAGEUP		104	/* Prior */	
#define SKEY_LEFT		105		
#define SKEY_RIGHT		106		
#define SKEY_END		107	/* Select */
#define SKEY_DOWN		108	
#define SKEY_PAGEDOWN		109	/* Next */	
#define SKEY_INSERT		110		
#define SKEY_DELETE		111             /* Remove */
#define SKEY_MACRO		112		
#define SKEY_MUTE		113	/* F13 */
#define SKEY_VOLUMEDOWN		114	/* F14 */
#define SKEY_VOLUMEUP		115	/* Help */	
#define SKEY_POWER		116	/* Do   */	
#define SKEY_KPEQUAL		117	/* F17  */
#define SKEY_KPPLUSMINUS	118     /* KP_MinPlus */
#define SKEY_PAUSE		119		
#define SKEY_F21		120    	/* -- unassigned -- */	
#define SKEY_F22		121	/* -- unassigned -- */	
#define SKEY_F23		122	/* -- unassigned -- */			
#define SKEY_F24		123	/* -- unassigned -- */			
#define SKEY_KPCOMMA		124	/* -- unassigned -- */	
#define SKEY_LEFTMETA		125	/* -- unassigned -- */			
#define SKEY_RIGHTMETA		126	/* -- unassigned -- */			
#define SKEY_COMPOSE		127	/* -- unassigned -- */			

#define SKEY_FUNCTION           128     /* The function key does not generate a scancode */

#endif /* __H3600_KEYBOARD_H__ */
