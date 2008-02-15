/*
 * Compaq Microkeyboard serial driver for the iPAQ H3800
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
 * Author: Andrew Christian 
 *         <andyc@handhelds.org>
 *         13 February 2002
 *
 * ---------- CHANGES-----------------------
 * 2002-05-19 Marcus Wolschon <Suran@gmx.net>
 *  - added proper support for the Compaq foldable keyboard
 * 
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/devfs_fs_kernel.h>
#include <linux/kbd_ll.h>
#include <linux/init.h>
#include <linux/kbd_kern.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <linux/h3600_keyboard.h>
#include <linux/pm.h>

#include <asm/bitops.h>
#include <asm/irq.h>
#include <asm/hardware.h>

MODULE_AUTHOR("Andrew Christian <andyc@handhelds.org>");

#define MKBD_PRESS	      0x7f
#define MKBD_RELEASE	      0x80

#define MKBD_MAX_KEYCODES      128        /* 128 possible keycodes */
#define MKBD_BITMAP_SIZE       MKBD_MAX_KEYCODES / BITS_PER_LONG

enum mkbd_state {
	MKBD_READ,
	MKBD_CHKSUM
};

struct mkbd_data;

struct microkbd_dev {
	char *         full_name;
	char *         name;
	void         (*process_char)(struct mkbd_data *, unsigned char);
	u32            baud;
	u32            cflag;
	devfs_handle_t devfs;
};

struct mkbd_data {
	struct microkbd_dev *dev;
	unsigned long        key_down[MKBD_BITMAP_SIZE];
	int                  usage_count;
	enum mkbd_state      state;      /* Used by Compaq microkeyboard */
	unsigned char        last;       /* Last character received */
};

struct mkbd_statistics {
	u32   isr;          /* RX interrupts       */
	u32   rx;           /* Bytes received      */
	u32   frame;        /* Frame errors        */
	u32   overrun;      /* Overrun errors      */
	u32   parity;       /* Parity errors       */
	u32   bad_xor;
	u32   invalid;
	u32   already_down;
	u32   already_up;
	u32   valid;
	u32   forced_release;
};

static struct mkbd_statistics     g_statistics;

#define MKBD_DLEVEL 0
#define SDEBUG(x, format, args...)  if ( x < MKBD_DLEVEL ) printk(format, ## args)
#define SFDEBUG(x, format, args...) if ( x < MKBD_DLEVEL ) printk(__FUNCTION__ ": " format, ## args )

/***********************************************************************************/
/*   Track keyboard press/releases and handle all state maintenance                */
/***********************************************************************************/

static void h3600_microkbd_release_all_keys(unsigned long *bitfield)
{
	int i,j;

	SFDEBUG(1,"releasing all keys.\n"); 

	for ( i = 0 ; i < MKBD_MAX_KEYCODES ; i+=BITS_PER_LONG, bitfield++ ) {
		if ( *bitfield ) {  /* Should fix this to use the ffs() function */
			for (j=0 ; j<BITS_PER_LONG; j++)
				if (test_and_clear_bit(j,bitfield))
					handle_scancode(i+j, 0);
		}
	}
}


/***********************************************************************************
 *   iConcepts
 * 
 *  Notes:
 *   The shift, control, alt, and fn keys are the only keys that can be held
 *      down while pressing another key.  
 *   All other keys generate keycode for down and "0x85 0x85" for up.
 *
 *  It seems to want an ACK of some kind?
 *
 *              Down       Up
 *   Control    1          9e 9e
 *   Function   2          9d 9d
 *   Shift      3          9c 9c
 *   Alt        4          9b 9b
 *   OTHERS     keycode    85 85
 *   
 ***********************************************************************************/

static unsigned char iconcepts_to_scancode[128] = { 
        0,  SKEY_LEFTCTRL,  SKEY_RIGHTALT,  SKEY_LEFTSHIFT,  
        SKEY_LEFTALT,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        SKEY_1,  SKEY_9,  SKEY_G,  SKEY_O,  
        SKEY_3,  SKEY_A,  SKEY_I,  SKEY_Q,  
        SKEY_5,  SKEY_C,  SKEY_K,  SKEY_S,  
        SKEY_7,  SKEY_E,  SKEY_M,  SKEY_U,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        SKEY_X,  SKEY_SEMICOLON,  SKEY_ENTER,  SKEY_LEFT,  
        SKEY_Z,  SKEY_COMMA,  SKEY_ESC,  0,  
        SKEY_EQUAL,  SKEY_SLASH,  SKEY_DELETE,  0,  
        SKEY_LEFTBRACE,  SKEY_TAB,  SKEY_DOWN,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        SKEY_V,  SKEY_N,  SKEY_F,  SKEY_8,  
        SKEY_T,  SKEY_L,  SKEY_D,  SKEY_6,  
        SKEY_R,  SKEY_J,  SKEY_B,  SKEY_4,  
        SKEY_P,  SKEY_H,  SKEY_0,  SKEY_2,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  SKEY_DOT,  SKEY_MINUS,  
        0,  SKEY_SPACE,  SKEY_APOSTROPHE,  SKEY_Y,  
        SKEY_RIGHT,  SKEY_CAPSLOCK,  SKEY_BACKSLASH,  SKEY_W,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        SKEY_BACKSPACE,  SKEY_UP,  SKEY_GRAVE,  SKEY_RIGHTBRACE,   };


static void h3600_iconcepts_process_char( struct mkbd_data *mkbd, unsigned char data )
{
	unsigned char key       = data;
	unsigned int  key_down  = !(data & 0x80);

	SDEBUG(2,"Read 0x%02x (%ld)\n", data, jiffies );

	if ( data >= 0x9b && data <= 0x9e ) {
		key = 0x9f - data;
		printk("set key to 0x%02x\n", key);
	}
	else if ( data == 0x85 ) {
		key = mkbd->last;
		printk("last key to 0x%02x\n", key);
		mkbd->last = 0;
		if ( !key ) return;  
	}

	if ( !iconcepts_to_scancode[key]) {  // Valid key down?
		SDEBUG(3,"   invalid key 0x%02x\n", key );
		g_statistics.invalid++;
	}
	else if (key_down && test_bit(key,mkbd->key_down)) {
		SDEBUG(3,"   already down 0x%02x\n", key );
		g_statistics.already_down++;
	}
	else {
		SDEBUG(3,"Sending 0x%02x down=%d\n", key, key_down);
		g_statistics.valid++;
		handle_scancode(iconcepts_to_scancode[key], key_down);
		if ( key_down ) {
			set_bit(key,mkbd->key_down);
			if ( key > 4 )
				mkbd->last = key;
		}
		else
			clear_bit(key,mkbd->key_down);
	}
}

/***********************************************************************************/
/*   Snap N Type keyboard                                                          */
/***********************************************************************************/

static unsigned char snapntype_to_scancode[128] = { 
        0,  0,  0,  0,  
        0,  SKEY_LEFTCTRL,  0,  0,  
        SKEY_BACKSPACE,  0,  SKEY_ENTER,  SKEY_PAGEUP,  
        SKEY_PAGEDOWN,  0,  0,  SKEY_ESC,  
        SKEY_LEFTSHIFT,  0,  H3600_SCANCODE_CALENDAR,  H3600_SCANCODE_CONTACTS,  
        H3600_SCANCODE_ENVELOPE,  H3600_SCANCODE_START,  0,  0,  
        0,  0,  0,  SKEY_RIGHTALT,  
        0,  0,  0,  0,  
        SKEY_SPACE,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  SKEY_A,  SKEY_B,  SKEY_C,  
        SKEY_D,  SKEY_E,  SKEY_F,  SKEY_G,  
        SKEY_H,  SKEY_I,  SKEY_J,  SKEY_K,  
        SKEY_L,  SKEY_M,  SKEY_N,  SKEY_O,  
        SKEY_P,  SKEY_Q,  SKEY_R,  SKEY_S,  
        SKEY_T,  SKEY_U,  SKEY_V,  SKEY_W,  
        SKEY_X,  SKEY_Y,  SKEY_Z,  0,  
        0,  0,  0,  0
};

static void h3600_snapntype_process_char( struct mkbd_data *mkbd, unsigned char data )
{
	unsigned char key       =   data & 0x7f;
	unsigned int  key_down  = !(data & 0x80);

	SDEBUG(3,"Read 0x%02x\n", data );
	if ( !snapntype_to_scancode[key] ) {
		g_statistics.invalid++;
		SDEBUG(3,"   invalid key 0x%02x\n", key );
	} 
	else if ( !key_down && !test_bit(key,mkbd->key_down) ) {
		g_statistics.already_up++;
		SDEBUG(3,"  already up 0x%02x\n", key );
	}
	else {
		SDEBUG(3,"Sending 0x%02x down=%d\n", key, key_down);
		g_statistics.valid++;
		handle_scancode(snapntype_to_scancode[key],key_down);
		if ( key_down )
			set_bit(key,mkbd->key_down);
		else
			clear_bit(key,mkbd->key_down);
	}
}

/***********************************************************************************/
/*   Compaq Microkeyboard                                                          */
/***********************************************************************************/

static unsigned char compaq_to_scancode[128] = { 
        0,  0,  SKEY_RIGHTALT,  0,  
        H3600_SCANCODE_ENVELOPE,  H3600_SCANCODE_CALENDAR,  H3600_SCANCODE_CONTACTS,  0,  
        0,  0,  0,  0,  
        H3600_SCANCODE_START,  0,  0,  0,  
        0,  0,  SKEY_LEFTSHIFT,  0,  
        0,  SKEY_Q,  0,  0,  
        0,  SKEY_LEFTCTRL,  SKEY_Z,  SKEY_S,  
        SKEY_A,  SKEY_W,  0,  0,  
        0,  SKEY_C,  SKEY_X,  SKEY_D,  
        SKEY_E,  0,  0,  0,  
        SKEY_UP,  0,  SKEY_V,  SKEY_F,  
        SKEY_T,  SKEY_R,  0,  SKEY_RIGHT,  
        0,  SKEY_N,  SKEY_B,  SKEY_H,  
        SKEY_G,  SKEY_Y,  0,  0,  
        0,  0,  SKEY_M,  SKEY_J,  
        SKEY_U,  0,  0,  0,  
        0,  0,  SKEY_K,  SKEY_I,  
        SKEY_O,  0,  0,  0,  
        0,  0,  0,  SKEY_L,  
        0,  SKEY_P,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        SKEY_CAPSLOCK,  0,  SKEY_ENTER,  0,  
        SKEY_SPACE,  0,  SKEY_LEFT,  0,  
        SKEY_DOWN,  0,  0,  0,  
        0,  0,  SKEY_BACKSPACE,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  0,  0,  
        0,  0,  H3600_SCANCODE_CALENDAR,  H3600_SCANCODE_CONTACTS,  
        H3600_SCANCODE_ENVELOPE,  H3600_SCANCODE_START,  0,  0
};

static void h3600_compaq_process_char(struct mkbd_data *mkbd, unsigned char data )
{
	unsigned char key       =   data & 0x7f;
	unsigned int  key_down  = !(data & 0x80);

	switch (mkbd->state) {
	case MKBD_READ:
 		SDEBUG(3,"Read 0x%02x\n", data );
		if ( !compaq_to_scancode[key] ) {
			g_statistics.invalid++;
			SDEBUG(3,"   bit invalid 0x%02x\n", key );
		} 
		else if ( key_down && test_bit(key,mkbd->key_down)) {
			g_statistics.already_down++;
			SDEBUG(3,"   already down 0x%02x\n", key );
		}
		else if ( !key_down && !test_bit(key,mkbd->key_down)) {
			g_statistics.already_up++;
			SDEBUG(3,"   already up 0x%02x\n", key );
		}
		else {
			mkbd->state = MKBD_CHKSUM;
			mkbd->last  = data;
			return;
		}
		break;
	case MKBD_CHKSUM: {
		unsigned char last      =   mkbd->last & 0x7f;
		unsigned int  last_down = !(mkbd->last & 0x80);

 		SDEBUG(3,"Checking read 0x%02x with 0x%02x\n", mkbd->last, data );
		if ( (last ^ data) != 0xff ) {
			g_statistics.bad_xor++;
			SDEBUG(3,"   XOR doesn't match last=0x%02x data=0x%02x XOR=0x%02x\n", 
			       last, data, (last ^ data));
		}
		else if ( last == 0x75 ) {
			SDEBUG(3,"   Valid keyboard restart\n");
		}
		else {
			SDEBUG(3,"Sending 0x%02x down=%d\n", last, last_down);
			g_statistics.valid++;
			handle_scancode(compaq_to_scancode[last], last_down);
			if ( last_down )
				set_bit(last,mkbd->key_down);
			else
				clear_bit(last,mkbd->key_down);
			mkbd->state = MKBD_READ;
			return;
		}
		break;
	}
	}
	g_statistics.forced_release++;
	h3600_microkbd_release_all_keys(mkbd->key_down);
};

/***********************************************************************************/
/*   Compaq foldable keyboard                                                      */
/***********************************************************************************/

// MW: I am not shure if I mapped all the en_US-non-alphanumeric keys correctly as I have only laptop-keyboards in german around here
// MW: [fn]+[Q/W/E/R/T/Z/...]-combinations are not implemented yet (not working that is)


// transmitted code->scancode (without fn-being pressed)

static unsigned char foldable_to_scancode[128] = {                                                             // decimal transmitted code, just a helper for entering new keys, will be removed
        0,                        SKEY_LEFTCTRL/* disabled FOLDABLE_FN*/,              0,            0,                                   // 0-3
        H3600_SCANCODE_ENVELOPE,  H3600_SCANCODE_CALENDAR,  H3600_SCANCODE_CONTACTS,  SKEY_HOME /*actually the window-key*/,    //4-7
        0,                        0,                        0,                        0,                       // 8-11
        0,                        SKEY_TAB,                 SKEY_ESC,                 0,                       // 12-15
        0,                        SKEY_RIGHTALT,            SKEY_LEFTSHIFT,  0,                                // 16-19
        SKEY_RIGHTCTRL,           SKEY_Q,                   SKEY_1,  0,                                        // 20-23
        0,                        SKEY_LEFTCTRL,            SKEY_Z,  SKEY_S,                                   // 24-27
        SKEY_A,                   SKEY_W,                   SKEY_2,  0,                                        // 28-31
        0,                        SKEY_C,                   SKEY_X,  SKEY_D,                                   // 32-35
        SKEY_E,                   SKEY_4,                   SKEY_3,  0,                                        // 36-39
        SKEY_UP,                  0,                        SKEY_V,  SKEY_F,                                   // 40-43
        SKEY_T,                   SKEY_R,                   SKEY_5,                   SKEY_RIGHT,              // 44-47
        0,                        SKEY_N,                   SKEY_B,                   SKEY_H,                  // 48-51 
        SKEY_G,                   SKEY_Y,                   SKEY_6,                   0,                       // 52-55
        0,                        0,                        SKEY_M,                   SKEY_J,                  // 56-59
        SKEY_U,                   SKEY_7,                   SKEY_8,                   0,                       // 60-63
        0,                        SKEY_COMMA,               SKEY_K,                   SKEY_I,                  // 64-67
        SKEY_O,                   SKEY_0,                   SKEY_9,                   0,                       // 68-71
        0,                        SKEY_DOT,                 SKEY_SLASH,               SKEY_L,                  // 72-75
        SKEY_SEMICOLON,           SKEY_P,                   SKEY_MINUS,               0,                       // 76-79
        0,                        0,                        SKEY_GRAVE,               0,                       // 80-83
        SKEY_LEFTBRACE,           SKEY_EQUAL,               0,                        0,                       // 84-87  //MW: leftbrace acts funny
        SKEY_CAPSLOCK,            SKEY_RIGHTSHIFT,          SKEY_ENTER,               SKEY_RIGHTBRACE,         // 88-91 
        SKEY_SPACE,               SKEY_APOSTROPHE,          SKEY_LEFT,                0,                       // 92-95
        SKEY_DOWN,                0,                        0,                        0,                       // 96-99
        0,                        0,                        SKEY_DELETE,              0,                       // 100-103
        0,                        0,                        0,                        0,                       // 104-107
        0,                        0,                        0,                        0,                       // 108-111
        H3600_SCANCODE_START,     SKEY_BACKSPACE,           0,                        SKEY_RIGHTCTRL,          // 112-115
        SKEY_BACKSLASH,           0,                        0,                        0,                       // 116-119
        0,                        0,                        H3600_SCANCODE_CALENDAR,  H3600_SCANCODE_CONTACTS, // 120-123
        H3600_SCANCODE_ENVELOPE,  H3600_SCANCODE_START,     0,                        0                        // 124-127
};

static void h3600_foldable_process_char(struct mkbd_data *mkbd, unsigned char data )
{
	unsigned char key       =   data & 0x7f;
	unsigned int  key_down  = !(data & 0x80);

	switch (mkbd->state) {
	case MKBD_READ: 
 		SDEBUG(3,"Read 0x%02x\n", data );
		if ( !foldable_to_scancode[key] ) {
			g_statistics.invalid++;
			SDEBUG(3,"[v1.1.9]   bit invalid 0x%02x\n", key );
		} 
		else if ( key_down && test_bit(key,mkbd->key_down)) {
			g_statistics.already_down++;
			SDEBUG(3,"   already down key=0x%02x\n", key);
		}
		else if ( !key_down && !test_bit(key,mkbd->key_down)) {
			g_statistics.already_up++;
			SDEBUG(3,"   already up 0x%02x\n", key );
		}
		else {
			mkbd->state = MKBD_CHKSUM;
			mkbd->last  = data;
			return;
		}
		break;
	case MKBD_CHKSUM: {
		unsigned char last      =   mkbd->last & 0x7f;
		unsigned int  last_down = !(mkbd->last & 0x80);

 		SDEBUG(3,"Checking read 0x%02x with 0x%02x\n", mkbd->last, data );
		if ( (last ^ data) != 0xff ) {
			g_statistics.bad_xor++;
			SDEBUG(3,"   XOR doesn't match last=0x%02x data=0x%02x XOR=0x%02x\n", 
			       last, data, (last ^ data));
		}
		else if ( last == 0x75 ) {
			SDEBUG(3,"   Valid keyboard restart\n");
		}
		else {
			SDEBUG(3,"Sending 0x%02x down=%d\n", last, last_down);
			g_statistics.valid++;

			handle_scancode(foldable_to_scancode[last], last_down);
			if ( last_down )
				set_bit(last,mkbd->key_down);
			else
				clear_bit(last,mkbd->key_down);
			mkbd->state = MKBD_READ;
			return;
		}
		break;
	}
	}
	g_statistics.forced_release++;
	h3600_microkbd_release_all_keys(mkbd->key_down);
}


static struct microkbd_dev keyboards[] = {
	{ "HP/Compaq Micro Keyboard", "compaq",    
	  h3600_compaq_process_char, 4800, UTCR0_8BitData | UTCR0_1StpBit },
	{ "RipTide SnapNType", "snapntype", 
	  h3600_snapntype_process_char, 2400, UTCR0_8BitData | UTCR0_1StpBit },
	{ "iConcepts Portable Keyboard", "iconcepts",
	  h3600_iconcepts_process_char, 9600, UTCR0_8BitData | UTCR0_1StpBit },
	{ "HP/Compaq Foldable Keyboard", "foldable",  
	  h3600_foldable_process_char,  4800, UTCR0_8BitData | UTCR0_1StpBit },
};

#define NUM_KEYBOARDS (sizeof(keyboards)/sizeof(keyboards[0]))


/***********************************************************************************/
/*   Interrupt handlers                                                            */
/***********************************************************************************/

static void h3600_microkbd_rx_chars( struct mkbd_data *mkbd )
{
	unsigned int status, ch;
	int count = 0;

	while ( (status = Ser3UTSR1) & UTSR1_RNE ) {
		ch = Ser3UTDR;
		g_statistics.rx++;

		if ( status & UTSR1_PRE ) { /* Parity error */
			g_statistics.parity++;
		} else 	if ( status & UTSR1_FRE ) { /* Framing error */
			g_statistics.frame++;
		} else {
			if ( status & UTSR1_ROR )   /* Overrun error */
				g_statistics.overrun++;
			
			count++;
			mkbd->dev->process_char( mkbd, ch );
		}
	}
}

static void h3600_microkbd_data_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct mkbd_data *mkbd = (struct mkbd_data *) dev_id;
        unsigned int status;	/* UTSR0 */

	SFDEBUG(2,"data interrupt %p\n", mkbd);
	g_statistics.isr++;
        status = Ser3UTSR0;

	if ( status & (UTSR0_RID | UTSR0_RFS) ) {
		if ( status & UTSR0_RID )
			Ser3UTSR0 = UTSR0_RID; /* Clear the Receiver IDLE bit */
		h3600_microkbd_rx_chars( mkbd );
	}

	/* Clear break bits */
	if (status & (UTSR0_RBB | UTSR0_REB))
		Ser3UTSR0 = status & (UTSR0_RBB | UTSR0_REB);
}


/***********************************************************************************/
/*   Initialization and shutdown code                                              */
/***********************************************************************************/

#define SERIAL_BAUD_BASE 230400

static void h3600_microkbd_reset_comm( struct mkbd_data *mkbd )
{
	int brd = (SERIAL_BAUD_BASE / mkbd->dev->baud) - 1;

	SFDEBUG(1,"initializing serial port\n");

	/* Set up interrupts */
	Ser3UTCR3 = 0;                                 /* Clean up CR3                  */
	Ser3UTCR0 = mkbd->dev->cflag;

	Ser3UTCR1 = (brd & 0xf00) >> 8;
	Ser3UTCR2 = brd & 0xff;

	Ser3UTSR0 = 0xff;                              /* Clear SR0 */
        Ser3UTCR3 = UTCR3_RXE | UTCR3_RIE;             /* Enable receive interrupt */
}

static int h3600_microkbd_startup(struct mkbd_data *mkbd)
{
	unsigned long flags;
	int retval = 0;

	SFDEBUG(1,"\n");
	save_flags_cli(flags);

	/* Timer structure */
	retval = request_irq( IRQ_Ser3UART, 
			      h3600_microkbd_data_interrupt,
			      SA_SHIRQ | SA_INTERRUPT | SA_SAMPLE_RANDOM,
			      "Microkbd data", (void *)mkbd);
				      
	restore_flags(flags);
	return retval;
}

static void h3600_microkbd_shutdown(struct mkbd_data * mkbd)
{
	unsigned long	flags;
	
	SFDEBUG(1,"\n");

	save_flags_cli(flags);
	free_irq(IRQ_Ser3UART, (void *)mkbd);
	restore_flags(flags);
}


/***********************************************************************************/
/*   Basic driver code                                                             */
/***********************************************************************************/

static ssize_t h3600_microkbd_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	SFDEBUG(1,"\n");

	current->state = TASK_INTERRUPTIBLE;
	while (!signal_pending(current))
		schedule();
	current->state = TASK_RUNNING;
	return 0;
}

static struct mkbd_data           g_keyboard;

static int h3600_microkbd_open( struct inode * inode, struct file * filp)
{
	unsigned int minor = MINOR( inode->i_rdev );
	int retval;

	SFDEBUG(1,"\n");

	if ( minor >= NUM_KEYBOARDS ) {
		printk(KERN_ALERT __FUNCTION__ " bad minor=%d\n", minor);
		return -ENODEV;
	}

	if ( g_keyboard.usage_count == 0 ) {
		/* Set up the interrupts - this also checks if someone else
		   is using the serial line */
		if ( (retval = h3600_microkbd_startup(&g_keyboard)) != 0 )
			return retval;

		SFDEBUG(1,"Setting up new keyboard\n");
		memset(&g_keyboard, 0, sizeof(struct mkbd_data));
		g_keyboard.dev = &keyboards[minor];

		/* Turn on power to the RS232 chip */
		set_h3600_egpio(IPAQ_EGPIO_RS232_ON);
		h3600_microkbd_reset_comm( &g_keyboard );
	}
	else {
		printk(KERN_ALERT __FUNCTION__ " keyboard device already open\n");
		return -EBUSY;
	}

	filp->private_data = &g_keyboard;
	g_keyboard.usage_count++;
	MOD_INC_USE_COUNT;
	return 0;
}

static int h3600_microkbd_release(struct inode * inode, struct file * filp)
{
	struct mkbd_data *mkbd = (struct mkbd_data *) filp->private_data;

	if (1) SFDEBUG(1,"\n");

	if ( --mkbd->usage_count == 0 ) {
		SFDEBUG(1,"Closing down keyboard\n");
		h3600_microkbd_release_all_keys(mkbd->key_down);
		h3600_microkbd_shutdown(mkbd);
		clr_h3600_egpio(IPAQ_EGPIO_RS232_ON);
	}

	MOD_DEC_USE_COUNT;
	return 0;
}

struct file_operations microkbd_fops = {
	read:           h3600_microkbd_read,
	open:		h3600_microkbd_open,
	release:	h3600_microkbd_release,
};

/***********************************************************************************/
/*   Proc filesystem interface                                                     */
/***********************************************************************************/

static struct proc_dir_entry   *proc_dir;

#define PRINT_DATA(x,s) \
	p += sprintf (p, "%-20s : %d\n", s, g_statistics.x)

int h3600_microkbd_proc_stats_read(char *page, char **start, off_t off,
				   int count, int *eof, void *data)
{
	char *p = page;
	int len, i;

	PRINT_DATA(isr,        "Keyboard interrupts");
	PRINT_DATA(rx,         "Bytes received");
	PRINT_DATA(frame,      "Frame errors");
	PRINT_DATA(overrun,    "Overrun errors");
	PRINT_DATA(parity,     "Parity errors");
	PRINT_DATA(bad_xor,    "Bad XOR");
	PRINT_DATA(invalid,    "Invalid keycode");
	PRINT_DATA(already_down,  "Key already down");
	PRINT_DATA(already_up,    "Key already up");
	PRINT_DATA(valid,         "Valid keys");
	PRINT_DATA(forced_release, "Forced release");
	p += sprintf(p,"%-20s : %d\n", "Usage count", g_keyboard.usage_count);
	p += sprintf(p,"\nRegistered keyboards\n");
	for ( i = 0 ; i < NUM_KEYBOARDS ; i++ ) 
		p += sprintf(p, " %15s (%d) : %s\n", keyboards[i].name, i, keyboards[i].full_name);

	len = (p - page) - off;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}


/***********************************************************************************/
/*   Initialization code                                                           */
/***********************************************************************************/

/* 
   TODO:  We seem to survive suspend/resume - no doubt due to settings by the serial driver
   We might want to not rely on them.

   TODO:  Limit ourselves to a single reader at a time.
   
   TODO:  Have a PROC setting so you don't need a reader to keep alive....
          This is the "I want to be myself" mode of operation.
*/

static struct pm_dev *microkbd_pm;

#ifdef CONFIG_PM
static int h3600_microkbd_pm_callback(struct pm_dev *dev, pm_request_t rqst, void *data)
{
	struct mkbd_data *mkbd = (struct mkbd_data *) dev->data;
	SFDEBUG(0,"changing to state %d\n", rqst);
	if ( mkbd->usage_count > 0 ) {
		switch (rqst) {
		case PM_SUSPEND:
			h3600_microkbd_release_all_keys(mkbd->key_down);
			break;
		case PM_RESUME:
			h3600_microkbd_reset_comm( mkbd );
			break;
		}
	}
	return 0;
}
#endif

static int            g_microkbd_major;  /* Dynamic major number */
static devfs_handle_t devfs_dir;


#define H3600_MICROKBD_MODULE_NAME "microkbd"
#define H3600_MICROKBD_PROC_STATS  "microkbd"

int __init h3600_microkbd_init_module( void )
{
	int i;

        /* register our character device */
        SFDEBUG(0,"registering char device\n");

        g_microkbd_major = devfs_register_chrdev(0, H3600_MICROKBD_MODULE_NAME, &microkbd_fops);
        if (g_microkbd_major < 0) {
                printk(KERN_ALERT __FUNCTION__ ": can't get major number\n");
                return g_microkbd_major;
        }

	devfs_dir = devfs_mk_dir(NULL, "microkbd", NULL);
	if ( !devfs_dir ) return -EBUSY;

	for ( i = 0 ; i < NUM_KEYBOARDS ; i++ )
		keyboards[i].devfs = devfs_register( devfs_dir, keyboards[i].name, DEVFS_FL_DEFAULT, 
						     g_microkbd_major, i, S_IFCHR | S_IRUSR | S_IWUSR, &microkbd_fops, NULL );

	/* Register in /proc filesystem */
	create_proc_read_entry(H3600_MICROKBD_PROC_STATS, 0, proc_dir,
			       h3600_microkbd_proc_stats_read, NULL );

#ifdef CONFIG_PM
	microkbd_pm = pm_register(PM_SYS_DEV, PM_SYS_COM, h3600_microkbd_pm_callback);
	if ( microkbd_pm )
		microkbd_pm->data = &g_keyboard;
#endif
	return 0;
}

void __exit h3600_microkbd_cleanup_module( void )
{
	int i;

	pm_unregister(microkbd_pm);
	remove_proc_entry(H3600_MICROKBD_PROC_STATS, proc_dir);

	for ( i = 0 ; i < NUM_KEYBOARDS ; i++ )
		devfs_unregister( keyboards[i].devfs );

	devfs_unregister( devfs_dir );
	devfs_unregister_chrdev( g_microkbd_major, H3600_MICROKBD_MODULE_NAME );
}

module_init(h3600_microkbd_init_module);
module_exit(h3600_microkbd_cleanup_module);


