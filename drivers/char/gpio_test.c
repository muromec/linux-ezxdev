/*
 * Copyright (C) 2004 Motorola Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/*
 *  linux/drivers/char/gpio_test.c
 *
 *  Support for the Motorola Ezx A780 Development Platform.
 *  
 *  Author:	Jay Jia
 *  Created:	April 25, 2004
 *  
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/wrapper.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/arch/ezx.h>


#define SUCCESS 0
#define DEVICE_NAME "gpio_test"
#define GPIO_TEST_IOCTL_CMD_OPS_MASK 0x000F
#define GPIO_TEST_IOCTL_CMD_VAL_MASK 0x0080
#define GPIO_TEST_IOCTL_CMD_NUM_MASK 0xFF00
#define GPIO_TEST_IOCTL_CMD_VAL_SHIFT 7
#define GPIO_TEST_IOCTL_CMD_NUM_SHIFT 8

static int Major=127;
//static unsigned char pcap_usb_ps = 0;
//static unsigned char pcap_vusb_en = 0;
//static unsigned char pcap_ov_test_on = 0;

static int gpio_test_open(struct inode *inode,struct file *file)
{
	MOD_INC_USE_COUNT;
      	return SUCCESS;
}

static int gpio_test_release(struct inode *inode,struct file *file)
	
{
    	MOD_DEC_USE_COUNT;
	return SUCCESS;
}


/*******************************************************************************
Function Name: gpio_test_ioctl

Parameter:
  The ioctl() accept a 2-byte command : cmd
  Use this param as follows:

  bit 0 - 3: operation type.
	0x01, gpio read
 	0x02, gpio write
	0x03, gpio config
	0x04, PCAP test
  bit 4 -6: 
	reserved. keep ZERO
  bit 7: 
	For gpio, write/config value. 
	For Over voltage test, 
		0, ON
		1, OFF
  bit 8 - 15:
	For gpio, it is gpio pin number
	For PCAP test,
		0x01, Over voltage test.
		others, reserved.
		
Return Value:
  For Over voltage test, 
    return 0 for success.
    return 1 for error.
*******************************************************************************/
static int gpio_test_ioctl(struct inode *inode,struct file *file,unsigned int cmd,unsigned long arg)
{
	int num, val, ops, ret;

	ret = 0;
	num = (cmd & GPIO_TEST_IOCTL_CMD_NUM_MASK) >> GPIO_TEST_IOCTL_CMD_NUM_SHIFT;
	val = cmd & GPIO_TEST_IOCTL_CMD_VAL_MASK;
	ops = cmd & GPIO_TEST_IOCTL_CMD_OPS_MASK;

	if(ops == 1)/*read gpio*/
	{
//		set_GPIO_mode(num);
//		GPDR(num) &=  ~GPIO_bit(num); 
		ret = GPLR(num) & GPIO_bit(num);
//        printk("in GPIO kernel read function, num = %d and ret = %d\n",num,ret);
		return (ret == 0) ? 0:1;		
	}

	if(ops == 2)/*write gpio*/
	{
//		set_GPIO_mode(num);
//		GPDR(num) |=  GPIO_bit(num);
		if( val )
			GPSR(num) = GPIO_bit(num);
		else
			GPCR(num) = GPIO_bit(num);	
		return 2;
	}	

	if(ops == 3)/*set gpio*/
	{
		set_GPIO_mode(num);
        if( val )
          GPDR(num) |=  GPIO_bit(num);
        else
          GPDR(num) &=  ~GPIO_bit(num);
		return 3;
	}
	return ret;
#if 0
	if(ops == 4)/* PCAP test */
	{
		printk("PCAP test, cmd = 0x%x.\n", cmd);
		val = val >> GPIO_TEST_IOCTL_CMD_VAL_SHIFT;
		switch(num)
		{
		case 1: /* Over Voltage test */
			if(val == 0) /* ON */
			{
				printk("Over Voltage test ON.\n");
				/* backup USB_PS and VUSB_EN */
				pcap_ov_test_on = 1;
				pcap_usb_ps = SSP_PCAP_get_bit_from_PCAP(SSP_PCAP_ADJ_BIT_BUSCTRL_USB_PS);
				pcap_vusb_en = SSP_PCAP_get_bit_from_PCAP(SSP_PCAP_ADJ_BIT_BUSCTRL_VUSB_EN);
				/* USB_PS = 0, PCAP USB powered by VUSB_IN */
				SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_BUSCTRL_USB_PS);
				/* VUSB_EN = 1, USB tranceiver enabled */
				SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_BUSCTRL_VUSB_EN);
				/* USB 1V INT mask */
				SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_MSR_USB1VM);
				/* USB 4V INT mask */
				SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_MSR_USB4VM);
				ret = 0;
			}
			else if(val == 1) /* OFF */
			{
				printk("Over Voltage test OFF.\n");
				if(pcap_ov_test_on != 1)
				{
					ret = 1; /* return error */
					break;
				}
				/* restore USB_PS and VUSB_EN */
				if(pcap_usb_ps)
				{
					SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_BUSCTRL_USB_PS);
				}
				else
				{
					SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_BUSCTRL_USB_PS);
				}
				if(pcap_vusb_en)
				{
					SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_BUSCTRL_VUSB_EN);
				}
				else
				{
					SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_BUSCTRL_VUSB_EN);
				}
				pcap_ov_test_on = 0;
				/* USB 1V INT unmask */
				SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_ISR_USB1VI); /* clear 1V INT status */
				SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_MSR_USB1VM);
				/* USB 4V INT unmask */
				SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_ISR_USB4VI); /* clear 4V INT status */
				SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_MSR_USB4VM);
				ret = 0;
			}
			else /* Unrecongized command */
			{
				ret = 1;
			}
			break;
		default:
			ret = 1;
		}	
		return ret;
	} /* end of PCAP test */		
#endif
}

static struct file_operations gpio_test_fops={
	ioctl:  gpio_test_ioctl,
	open:   gpio_test_open,
	release:gpio_test_release,
};

int gpio_test_init_module(void)
{
	register_chrdev(Major, DEVICE_NAME, &gpio_test_fops);
	return 0;
}

void gpio_test_cleanup_module(void)
{
	unregister_chrdev(Major,DEVICE_NAME);
}

module_init(gpio_test_init_module);
module_exit(gpio_test_cleanup_module);
MODULE_AUTHOR("Jay Jia");
MODULE_LICENSE("GPL");
