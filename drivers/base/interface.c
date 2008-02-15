/*
 * drivers/base/interface.c - common driverfs interface that's exported to 
 * 	the world for all devices.
 * Copyright (c) 2002 Patrick Mochel
 *		 2002 Open Source Development Lab
 */
/*
 * Copyright (C) 2005 Motorola Inc.
 *
 *  2005-May-11  update montavista patch, Zhuang Xiaofan
 *  2005-Aug-12  resolve kloc issue, Yin kangkai
 *
 */

#include <linux/device.h>
#if 0 /* linux-pm */
#include <linux/err.h>
#endif
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/slab.h>

DECLARE_MUTEX(runtime_pm_sem);

static ssize_t device_read_name(struct device * dev, char * buf, size_t count, loff_t off)
{
	return off ? 0 : sprintf(buf,"%s\n",dev->name);
}

static DEVICE_ATTR(name,S_IRUGO,device_read_name,NULL);

static ssize_t
device_read_power(struct device * dev, char * page, size_t count, loff_t off)
{
	return off ? 0 : sprintf(page,"%d\n",dev->power_state);
}

static ssize_t
device_write_power(struct device * dev, const char * buf, size_t count, loff_t off)
{
	char	str_command[20];
	char	str_level[20];
	int	num_args;
	u32	state;
	u32	int_level;
	int	error = 0;

	if (off)
		return 0;

	if (!dev->driver)
		goto done;

	num_args = sscanf(buf,"%10s %10s %u",str_command,str_level,&state);

	error = -EINVAL;

	if (!num_args)
		goto done;

	if (!strnicmp(str_command,"suspend",7)) {
		if (num_args != 3)
			goto done;
		if (!strnicmp(str_level,"notify",6))
			int_level = SUSPEND_NOTIFY;
		else if (!strnicmp(str_level,"save",4))
			int_level = SUSPEND_SAVE_STATE;
		else if (!strnicmp(str_level,"disable",7))
			int_level = SUSPEND_DISABLE;
		else if (!strnicmp(str_level,"powerdown",8))
			int_level = SUSPEND_POWER_DOWN;
		else
			goto done;

		if (dev->driver->suspend) {
			/* Change to DPM enum for driver_powerdown */
			int_level = DPM_POWER_OFF;

			down(&runtime_pm_sem);
			error = dev->power_state ? 0 : 
				driver_powerdown(dev->driver, dev, int_level);
			up(&runtime_pm_sem);
		}
		else
			error = 0;
	} else if (!strnicmp(str_command,"resume",6)) {
		int_level = DPM_POWER_ON;

		if (num_args != 2)
			goto done;

		if (!strnicmp(str_level,"poweron",7))
			int_level = RESUME_POWER_ON;
		else if (!strnicmp(str_level,"restore",7))
			int_level = RESUME_RESTORE_STATE;
		else if (!strnicmp(str_level,"enable",6))
			int_level = RESUME_ENABLE;
		else
			goto done;

		down(&runtime_pm_sem);
		error = dev->power_state ? 
			driver_powerup(dev->driver, dev, int_level) : 0;
		up(&runtime_pm_sem);
	}
 done:
	return error < 0 ? error : count;
}

static DEVICE_ATTR(power,S_IWUSR | S_IRUGO,
		   device_read_power,device_write_power);

static ssize_t 
device_read_constraints(struct device * dev, char * buf, size_t count, loff_t off)
{
	int i, cnt = 0;
	
	if (! off) {
		/* Buses do not have a driver, so check the driver
		   before checking for constraints in the driver */
		if(dev->constraints) {
			for (i = 0; i < DPM_PARAM_MAX; i++) {
				cnt += sprintf(buf + cnt,"%d %d %d\n",
					       dev->constraints->param[i].id,
					       dev->constraints->param[i].max,
					       dev->constraints->param[i].min);
			}
		}
	}

	return cnt;
}

static ssize_t
device_write_constraints(struct device * dev, const char * buf, size_t count, loff_t off)
{
	int num_args, id, max, min, constr_count;

	if(!dev) {
		/* Very odd to get here with no device */
		return -EINVAL;
	}

	if (!dev->constraints) {
		dev->constraints =
			kmalloc(sizeof(struct constraints), GFP_KERNEL);
		
		if (!dev->constraints)
			return -EINVAL;

		memset(dev->constraints, 0, sizeof(struct constraints));

		/* New constraints should start off asserted */
		dev->constraints->asserted = 1;
	}

	num_args = sscanf(buf,"%d %d %d",&id,&max, &min);

	/* Get current count, incremement, and check */
	constr_count = dev->constraints->count;

	if ((num_args != 3) || (constr_count >= DPM_PARAM_MAX) || 
	    (id >= DPM_PARAM_MAX)) {
		return -EINVAL;
	}
	
	/* Save the new count */
	dev->constraints->param[constr_count].id = id; 
	dev->constraints->param[constr_count].max = max; 
	dev->constraints->param[constr_count].min = min; 
	/* Now increment */
	dev->constraints->count++;

	return count;
}

static DEVICE_ATTR(constraints,S_IWUSR | S_IRUGO,
		   device_read_constraints,device_write_constraints);

struct attribute * dev_default_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_power.attr,
	&dev_attr_constraints.attr,
	NULL,
};
