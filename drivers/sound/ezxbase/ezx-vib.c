/*
 * Copyright (C) 2002-2005 Motorola Inc.
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
 *
 *
 *  History:
 *  zhouqiong          Jun 20,2002             created
 *  Kin Wong           Nov 05,2003             Renamed ezx-vibrator.h to vibrator.h 
 *  Jin Lihong(w20076) Apr.13,2004,LIBdd96876  reorganise file header
 *  Lv Yunguang(a6511c) Mar.10,2005            Change the debug log for barbados 
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/pm.h>
#include <linux/fs.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>
#include <asm/dma.h>

#include "ezx-common.h"
#include "ezx-vib.h"


#ifdef CONFIG_PM
static struct pm_dev *pm_dev;
#endif

static int active=0;
static int voltage=0;

static int vibrator_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	int ret;
	long val;

	switch(cmd)
	{
	case VIBRATOR_ENABLE:
                AUDPRINTk1("enable vibrator \n");
		ret = get_user(val, (int *) arg);
		if (ret)
			return ret;
		switch(val)
		{
		    case 0:
                        AUDPRINTk1("vibrator level 0 \n");
			PCAP_V_VIB_level_set(VIBRATOR_VOLTAGE_LEVEL0);
			PCAP_V_VIB_level_set(VIBRATOR_VOLTAGE_LEVEL0);
			active = 1;
			voltage = 0;
			PCAP_vibrate_start_command();
			break;
		    case 1:
                        AUDPRINTk1("vibrator level 1 \n");
			PCAP_V_VIB_level_set(VIBRATOR_VOLTAGE_LEVEL1);
			PCAP_V_VIB_level_set(VIBRATOR_VOLTAGE_LEVEL1);
			active = 1;
			voltage = 1;
			PCAP_vibrate_start_command();
			break;
		    case 2:
                        AUDPRINTk1("vibrator level 2 \n");
			PCAP_V_VIB_level_set(VIBRATOR_VOLTAGE_LEVEL2);
			PCAP_V_VIB_level_set(VIBRATOR_VOLTAGE_LEVEL2);
			active = 1;
			voltage = 2;
			PCAP_vibrate_start_command();
			break;
		    case 3:
                        AUDPRINTk1("vibrator level 3 \n");
			PCAP_V_VIB_level_set(VIBRATOR_VOLTAGE_LEVEL3);
			PCAP_V_VIB_level_set(VIBRATOR_VOLTAGE_LEVEL3);
			active = 1;
			voltage = 3;
			PCAP_vibrate_start_command();
			break;
		    default:
                        AUDPRINTk1("vibrator level error \n");
			return -EINVAL;
		}
		return put_user(ret, (int *) arg);

	case VIBRATOR_DISABLE:
                AUDPRINTk1("disable vibrator \n");
		ret = 0;
		active = 0;
		PCAP_vibrate_stop_command();
		return put_user(ret, (int *) arg);
	default:
		return -EINVAL;
	}
	return 0;
}

#ifdef CONFIG_PM
static int vibrator_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
{
	switch(req)
	{
		case PM_SUSPEND:
			if(active)
			{
				PCAP_vibrate_stop_command();
			}
			break;
		case PM_RESUME:
			if(active)
			{
				switch(voltage)
				{
				    case 0:
					PCAP_V_VIB_level_set(VIBRATOR_VOLTAGE_LEVEL0);
					PCAP_V_VIB_level_set(VIBRATOR_VOLTAGE_LEVEL0);
					break;
				    case 1:
					PCAP_V_VIB_level_set(VIBRATOR_VOLTAGE_LEVEL1);
					PCAP_V_VIB_level_set(VIBRATOR_VOLTAGE_LEVEL1);
					break;
				    case 2:
					PCAP_V_VIB_level_set(VIBRATOR_VOLTAGE_LEVEL2);
					PCAP_V_VIB_level_set(VIBRATOR_VOLTAGE_LEVEL2);
					break;
				    case 3:
					PCAP_V_VIB_level_set(VIBRATOR_VOLTAGE_LEVEL3);
					PCAP_V_VIB_level_set(VIBRATOR_VOLTAGE_LEVEL3);
					break;
				    default:
					break;
				}
				PCAP_vibrate_start_command();
			}
			break;
	}
	return 0;
}
#endif

static int count=0;
static int vibrator_open(struct inode *inode, struct file *file)
{
	if(!count)
	{
		count ++;
		//ssp_pcap_init();
#ifdef CONFIG_PM
		pm_dev = pm_register(PM_SYS_DEV, 0, vibrator_pm_callback);
#endif
                AUDPRINTk1("open vibrator \n");
		return 0;
	}
	else
		return -EBUSY;
}

static int vibrator_release(struct inode *inode, struct file *file)
{
	count --;
	if(!count)
	{
#ifdef CONFIG_PM
		pm_unregister(pm_dev);
#endif
                AUDPRINTk1("open vibrator \n");
	}
	return 0;
}


static struct file_operations device_fops = {
    owner:		THIS_MODULE,
	open:		vibrator_open,
	release:	vibrator_release,
	ioctl:		vibrator_ioctl,
};

static int __init vibrator_init(void)
{
	int ret;
        AUDPRINTk1("enter vibrator init...\n");
	ret = register_chrdev(VIBRATOR_MAJOR,"vibrator", &device_fops);
	if(ret){
		printk("can't register vibrator device with kernel");
		}
        AUDPRINTk1("vibrator init ok\n");
	return 0;
}

static void __exit vibrator_exit(void)
{
	unregister_chrdev(VIBRATOR_MAJOR, "vibrator");
        AUDPRINTk1("vibrator exit ok\n");
	return;
}


module_init(vibrator_init);
module_exit(vibrator_exit);


