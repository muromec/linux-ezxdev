
/*
 * LED support for the Force CPCI735 card.
 *
 * Corey Minyard <minyard@mvista.com>
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <asm/io.h>
#include <linux/led.h>
#include <linux/init.h>

/************************************************************************/
/* The following are defines for the CPCI735 FPGA1.  If something else
   ever uses that FPGA's registers, this will need to be split out. */

#define CPCI735_FPGA1_LED_REG		0x03
#define CPCI735_FPGA1_VERSION_REG	0x08

#define CPCI735_FPGA1_LED_OFF	0x00
#define CPCI735_FPGA1_LED_RED	0x01
#define CPCI735_FPGA1_LED_GREEN	0x02
#define CPCI735_FPGA1_LED_AUX	0x03

#define CPCI735_FPGA1_LED_MASK	0x03

static spinlock_t fpga1_lock = SPIN_LOCK_UNLOCKED;

static unsigned short fpga1_index_port = 0x100;
static unsigned short fpga1_data_port = 0x101;

static unsigned char cpci735_read_fpga1(int offset)
{
	unsigned char rv;

	spin_lock(&fpga1_lock);
	outb(offset, fpga1_index_port);
	rv = inb(fpga1_data_port);
	spin_unlock(&fpga1_lock);
	return rv;
}

static void cpci735_write_fpga1(int offset, unsigned char value)
{
	spin_lock(&fpga1_lock);
	outb(offset, fpga1_index_port);
	outb(value, fpga1_data_port);
	spin_unlock(&fpga1_lock);
}

/************************************************************************/


#define VERSION "1.0"

/* These are the control structures for all the LEDs. */
static struct led_info info1;
static struct led_reg_info led1;
static struct led_info info2;
static struct led_reg_info led2;
static struct led_info info3;
static struct led_reg_info led3;
static struct led_info info4;
static struct led_reg_info led4;

/* Handle an actual LED set or get operation for the card. */
int handle_led_op(struct led_reg_info *info,
		  struct led_op       *op)
{
	unsigned char val;
	unsigned char newval;
	int           shift = ((int) info->data) * 2;

	switch (op->op)
	{
	case SET_LED:
		val = cpci735_read_fpga1(CPCI735_FPGA1_LED_REG);
		switch (op->op_info.bicolor.color)
		{
		case LED_COLOR_OFF:
			newval = CPCI735_FPGA1_LED_OFF; break;
		case LED_COLOR_RED:
			newval = CPCI735_FPGA1_LED_RED; break;
		case LED_COLOR_GREEN:
			newval = CPCI735_FPGA1_LED_GREEN; break;
		case LED_COLOR_AUXMODE:
			if (shift == 0) {
				return -EINVAL;
			}
			newval = CPCI735_FPGA1_LED_AUX; 
			break;

		default:
			return -EINVAL;
		}
		val = val & ~(CPCI735_FPGA1_LED_MASK << shift);
		val = val | newval << shift;
		cpci735_write_fpga1(CPCI735_FPGA1_LED_REG, val);
		return 0;
		
	case GET_LED:
		val = cpci735_read_fpga1(CPCI735_FPGA1_LED_REG);
		val = (val >> shift) & CPCI735_FPGA1_LED_MASK;
		switch (val)
		{
		case CPCI735_FPGA1_LED_OFF:
			op->op_info.bicolor.color = LED_COLOR_OFF; break;
		case CPCI735_FPGA1_LED_RED:
			op->op_info.bicolor.color = LED_COLOR_RED; break;
		case CPCI735_FPGA1_LED_GREEN:
			op->op_info.bicolor.color = LED_COLOR_GREEN; break;
		case CPCI735_FPGA1_LED_AUX:
			op->op_info.bicolor.color = LED_COLOR_AUXMODE; break;
		}
		return 0;

	default:
		return -EINVAL;
	}
}

/**
 *	cpci735_exit:
 *
 *	Remove the LEDs from the LED driver
 */
 
static void __exit cpci735_exit(void)
{
	unregister_led_info(&led1);
	unregister_led_info(&led2);
	unregister_led_info(&led3);
	unregister_led_info(&led4);
}

/**
 * 	cpci735_init:
 *
 *	Register the LEDs with the LED driver.
 */
 
static int __init cpci735_init(void)
{
	unsigned char vera, verb;

	vera = cpci735_read_fpga1(CPCI735_FPGA1_VERSION_REG);
	verb = (vera >> 4) & 0xf;
	vera &= 0xf;
	printk("cpci735 LED driver: v%s Corey Minyard (minyard@mvista.com)\n",
	       VERSION);
	printk("  FPGA1 version %d.%d\n", vera, verb);

	strcpy(info1.name, "LED1");
	info1.type = LED_TYPE_BICOLOR;
	info1.info.bicolor.color1 = LED_COLOR_RED;
	info1.info.bicolor.color2 = LED_COLOR_GREEN;
	info1.info.bicolor.has_aux_mode = 0;
	led1.info = &info1;
	led1.data = (void *) 0;
	led1.handle_led_op = handle_led_op;

	strcpy(info2.name, "LED2/IDE");
	info2.type = LED_TYPE_BICOLOR;
	info2.info.bicolor.color1 = LED_COLOR_RED;
	info2.info.bicolor.color2 = LED_COLOR_GREEN;
	info2.info.bicolor.has_aux_mode = 1;
	led2.info = &info2;
	led2.data = (void *) 1;
	led2.handle_led_op = handle_led_op;

	strcpy(info3.name, "LED3/ETH0");
	info3.type = LED_TYPE_BICOLOR;
	info3.info.bicolor.color1 = LED_COLOR_RED;
	info3.info.bicolor.color2 = LED_COLOR_GREEN;
	info3.info.bicolor.has_aux_mode = 1;
	led3.info = &info3;
	led3.data = (void *) 2;
	led3.handle_led_op = handle_led_op;

	strcpy(info4.name, "LED4/ETH1");
	info4.type = LED_TYPE_BICOLOR;
	info4.info.bicolor.color1 = LED_COLOR_RED;
	info4.info.bicolor.color2 = LED_COLOR_GREEN;
	info4.info.bicolor.has_aux_mode = 1;
	led4.info = &info4;
	led4.data = (void *) 3;
	led4.handle_led_op = handle_led_op;

	register_led_info(&led1);
	register_led_info(&led2);
	register_led_info(&led3);
	register_led_info(&led4);

	return 0;
}

module_init(cpci735_init);
module_exit(cpci735_exit);

MODULE_LICENSE("GPL");
