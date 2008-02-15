/*
 * Defines for the LED device driver.
 *
 * This driver uses ioctl() calls to get information about the LEDS
 * and to set their values.
 *
 * This currently only supports mono and bicolor LEDs, but support for
 * fancier LEDs (scrolling text or numeric LEDs, for instance) could
 * easily be added.
 *
 * Corey Minyard <minyard@mvista.com>
 *
 */

#ifndef _LINUX_LED_H
#define _LINUX_LED_H

#include <linux/ioctl.h>

#define	LED_IOCTL_BASE	'L'

/* Types of LEDs. */
#define LED_TYPE_MONOCOLOR	1
#define LED_TYPE_BICOLOR	2

/* Defines for LED colors. */
#define LED_COLOR_OFF		0
#define LED_COLOR_RED		1
#define LED_COLOR_GREEN		2
#define LED_COLOR_BLUE		3
#define LED_COLOR_YELLOW	4
#define LED_COLOR_AUXMODE	-1 /* Set it to the auxiliary mode. */

union led_info_u
{
	struct
	{
		int color;		/* Color of the LED. */
		int has_aux_mode;	/* If true, the LED has an auxiliary
					   mode (like a disk drive or
					   network LED) that it can be set
					   to work in. */
	} monocolor;

	struct
	{
		int color1;		/* First color of the LED. */
		int color2;		/* Second color of the LED. */
		int has_aux_mode;	/* If true, the LED has an auxiliary
					   mode (like a disk drive or
					   network LED) that it can be set
					   to work in. */
	} bicolor;
};

/* Information about an LED.  This is not the dynamic information, but
   the information about the capabilities of the LED. */
struct led_info
{
	int led_num;
	int type;
	char name[32]; /* The name of the LED, used to address it. */
	union led_info_u info;
};

/* Operation types that are valid. */
#define SET_LED		1	/* Set the value of the LED. */
#define GET_LED		2	/* Return the LED's current value. */

/* Operations defined for each different type of LED. */
union led_op_u
{
	struct
	{
		int color;
	} monocolor;

	struct
	{
		int color;
	} bicolor;
};

/* An operation to perform on the LED.  Set the name and the operation
   and any needed info and do an LEDIOC_OP ioctl. */
struct led_op
{
	int op;
	char name[32]; /* The name of the LED, used to address it. */
	union led_op_u op_info;
};



/* Return the number of LEDs that have registered with the driver. */
#define	LEDIOC_GETCOUNT		_IOR(LED_IOCTL_BASE, 0, void)

/* Get the info for a single LED, indexed by number or name. */
#define	LEDIOC_GETINFO_BY_NUM	_IOR(LED_IOCTL_BASE, 1, struct led_info)
#define	LEDIOC_GETINFO_BY_NAME	_IOR(LED_IOCTL_BASE, 2, struct led_info)

/* Perform an operation on single LED, indexed by name. */
#define	LEDIOC_OP		_IOR(LED_IOCTL_BASE, 3, struct led_op)



/* This is the structure that is registered with the LED driver by the
   subtending LED drivers.  One of these must be registered for each
   LED. */
struct led_reg_info
{
	struct led_info     *info; /* Pointer to the info structure for
				      this LED. */
	void                *data; /* Used by the specific LED driver for
				      whatever it likes. */

	/* The handler the driver supplies to handle operations. */
	int (*handle_led_op)(struct led_reg_info *info,
			     struct led_op       *op);

	struct led_reg_info *next;
};

/* Add an LED to the set of LEDs the driver controls. */
int register_led_info(struct led_reg_info *info);

/* Remove an LED from the set of LEDs the driver controls. */
int unregister_led_info(struct led_reg_info *info);

#endif  /* ifndef _LINUX_LED_H */
