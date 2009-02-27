/*
 * driver.c - centralized device driver management
 *
 * Copyright (C) 2004-2005 Motorola
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
 * Port to EzX platform by Motorola.
 */


#if 1 /* linux-pm */
#define DEBUG 1
#else
#define DEBUG
#endif

#include <linux/device.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/slab.h>
#include "base.h"

#define to_dev(node) container_of(node,struct device,driver_list)
#define to_drv(obj) container_of(obj,struct device_driver,kobj)

static void dpm_add_to_device_event_queue(char *str);

/**
 *	driver_create_file - create sysfs file for driver.
 *	@drv:	driver.
 *	@attr:	driver attribute descriptor.
 */

/* spinlock for protecting the event queue */
static spinlock_t dpm_controller_lock = SPIN_LOCK_UNLOCKED;
struct dpm_device_event_queue *dpm_event_queue = 0;

int driver_create_file(struct device_driver * drv, struct driver_attribute * attr)
{
	int error;
	if (get_driver(drv)) {
		error = sysfs_create_file(&drv->kobj,&attr->attr);
		put_driver(drv);
	} else
		error = -EINVAL;
	return error;
}


/**
 *	driver_remove_file - remove sysfs file for driver.
 *	@drv:	driver.
 *	@attr:	driver attribute descriptor.
 */

void driver_remove_file(struct device_driver * drv, struct driver_attribute * attr)
{
	if (get_driver(drv)) {
		sysfs_remove_file(&drv->kobj,&attr->attr);
		put_driver(drv);
	}
}


/**
 *	get_driver - increment driver reference count.
 *	@drv:	driver.
 */
struct device_driver * get_driver(struct device_driver * drv)
{
	return drv ? to_drv(kobject_get(&drv->kobj)) : NULL;
}


/**
 *	put_driver - decrement driver's refcount.
 *	@drv:	driver.
 */
void put_driver(struct device_driver * drv)
{
	kobject_put(&drv->kobj);
}


/**
 *	driver_register - register driver with bus
 *	@drv:	driver to register
 *
 *	We pass off most of the work to the bus_add_driver() call,
 *	since most of the things we have to do deal with the bus 
 *	structures.
 *
 *	The one interesting aspect is that we initialize @drv->unload_sem
 *	to a locked state here. It will be unlocked when the driver
 *	reference count reaches 0.
 */
int driver_register(struct device_driver * drv)
{
	INIT_LIST_HEAD(&drv->devices);
	init_MUTEX_LOCKED(&drv->unload_sem);
	return bus_add_driver(drv);
}


/**
 *	driver_unregister - remove driver from system.
 *	@drv:	driver.
 *
 *	Again, we pass off most of the work to the bus-level call.
 *
 *	Though, once that is done, we attempt to take @drv->unload_sem.
 *	This will block until the driver refcount reaches 0, and it is
 *	released. Only modular drivers will call this function, and we 
 *	have to guarantee that it won't complete, letting the driver 
 *	unload until all references are gone.
 */

void driver_unregister(struct device_driver * drv)
{
	bus_remove_driver(drv);
	down(&drv->unload_sem);
	up(&drv->unload_sem);
}

#if 1 /* linux-pm */
static int device_power_onoff(struct device *dev, u32 level)
{
	struct device_driver *drv = dev->driver;

	switch(level) {
	case DPM_POWER_ON:
		if (drv->resume)
			drv->resume(dev, RESUME_POWER_ON);
		break;
	case DPM_POWER_OFF:
	case DPM_SUSPEND_FOR_OP:
		if (drv->suspend)
			drv->suspend(dev, 0, SUSPEND_POWER_DOWN);
		break;
	}

	/*
	 * If the device has been previously explicitly powered off
	 * then leave that state intact in case we're suspending due to
	 * scaling to an incompatible operating point.
	 */

	if ((level != DPM_SUSPEND_FOR_OP) ||
	    (dev->power_state != DPM_POWER_OFF))
		dev->power_state = level;

	return 0;
}

/* Added by Susan for CSTN smartpanel suspend */
int device_power_suspend(struct device *dev, u32 level)
{
	struct device_driver *drv = dev->driver;

	switch(level) {
	case DPM_SUSPEND_FOR_OP:
		if (drv->suspend)
			drv->suspend(dev, 0, SUSPEND_DISABLE);
		break;
	}

	/*
	 * For other passing-in level, do nothing.
	 */

	return 0;
}

int driver_powerup(struct device_driver *drv, struct device *dev, u32 level)
{
	int valid = validate_constraints(drv->bus, dev->constraints);
#ifdef CONFIG_DPM
	char *event_str;
	int len;
#endif
	if (dev)
		device_power_onoff(dev, valid ? level : DPM_SUSPEND_FOR_OP);
	else {
		struct list_head * entry, * next;

		list_for_each_safe(entry,next,&drv->devices) {
			dev = container_of(entry,struct device,driver_list);
			device_power_onoff(dev, valid ? level : 
					   DPM_SUSPEND_FOR_OP);
		}
	}

#ifdef CONFIG_DPM 
	/* dpm_add_to_device_event_queue requires a malloc'd
	   string. It will be free'd by the caller to
	   dpm_get_from_device_event_queue */
	len = strlen(dev->bus->name) + 1 /* ":" */
	      + strlen(dev->name) + 1 /* ":" */
	      + 3; /* "on\0" */
	if (! (event_str = (char *)kmalloc(len, GFP_KERNEL)))
		return -ENOMEM;
	sprintf(event_str, "%s:%s:on", dev->bus->name, dev->name);
	dpm_add_to_device_event_queue(event_str);

	if (dev->constraints && (dev->constraints->asserted != valid)) {
		dev->constraints->asserted = valid;

		if (valid) {
			assert_constraints(drv->bus, dev->constraints);
		}
		else
			bus_reeval_constraints(drv->bus);
	} 
#endif
	return 0;
}

int driver_powerdown(struct device_driver *drv, struct device *dev, u32 level)
{
#ifdef CONFIG_DPM
	char *event_str;
	int len;
#endif

	if (dev)
		device_power_onoff(dev, level);
	else {
		struct list_head * entry, * next;

		list_for_each_safe(entry,next,&drv->devices) {
			dev = container_of(entry,struct device,driver_list);
			device_power_onoff(dev, level);
		}
	}

#ifdef CONFIG_DPM
	/* dpm_add_to_device_event_queue requires a malloc'd
	   string. It will be free'd by the caller to
	   dpm_get_from_device_event_queue */
	len = strlen(dev->bus->name) + 1 /* ":" */
	      + strlen(dev->name) + 1 /* ":" */
	      + 4; /* "off\0" */
	event_str = (char *)kmalloc(len, GFP_KERNEL);
	sprintf(event_str, "%s:%s:off", dev->bus->name, dev->name);
	dpm_add_to_device_event_queue(event_str);

	if (dev->constraints && dev->constraints->asserted) {
		deassert_constraints(drv->bus, dev->constraints);
	} 
#endif
	return 0;
}

/* Allocate memory for the DPM device event_queue */
int __init
dpm_event_queue_init(void)
{
	/* Allocate memory for the DPM event queue */
	dpm_event_queue = (struct dpm_device_event_queue *)
			  kmalloc(sizeof(*dpm_event_queue), GFP_KERNEL);

	if (dpm_event_queue == NULL) {
		printk(KERN_ERR "dpm_event_queue_init(): out of memory\n");
		return -ENOMEM;
	}

	memset(dpm_event_queue, 0, sizeof(*dpm_event_queue));
	dpm_event_queue->head = dpm_event_queue->tail = 0;

	init_waitqueue_head(&dpm_event_queue->proc_list);

	return 0;
}

/* dpm_add_to_device_event_queue requires that the string passed in be
   malloc'd so it may be freed by the caller to
   dpm_get_from_device_event_queue */
static void dpm_add_to_device_event_queue(char *str)
{
	int head, len;
	unsigned long flags;

	len = strlen(str);
	if (len <= 0) {
		return;
	}
	/* lock the queue */
	spin_lock_irqsave(&dpm_controller_lock, flags);

	/* Add to queue */
	head = dpm_event_queue->head;
	dpm_event_queue->str[head] = str;
	head = (head + 1) & (DPM_EVENT_QUEUE_SIZE - 1);
	if (head != dpm_event_queue->tail) {
		dpm_event_queue->head = head;
		wake_up_interruptible(&dpm_event_queue->proc_list);
	}

	/* Unlock */
	spin_unlock_irqrestore(&dpm_controller_lock, flags);
}

/* The caller to dpm_get_from_device_event_queue should free the
   string returned; it was malloc'd by the caller of
   dpm_add_to_device_event_queue */
char *dpm_get_from_device_event_queue(void)
{
	unsigned long flags;
	char *str;

	if (dpm_event_queue_empty()) {
		/* The queue is empty */
		return 0;
	}

	/* else */

	spin_lock_irqsave(&dpm_controller_lock, flags);

	str = dpm_event_queue->str[dpm_event_queue->tail];
	dpm_event_queue->tail = (dpm_event_queue->tail + 1) &
				(DPM_EVENT_QUEUE_SIZE - 1);

	spin_unlock_irqrestore(&dpm_controller_lock, flags);

	return str;
}

inline int dpm_event_queue_empty(void)
{
	return dpm_event_queue->head == dpm_event_queue->tail;
}

static int device_op_check(struct device * dev)
{
	if (dev->power_state == DPM_SUSPEND_FOR_OP) {
		device_power_onoff(dev, DPM_POWER_ON);

		if (dev->constraints && 
		    ! dev->constraints->asserted) {
			dev->constraints->asserted = 1;
			assert_constraints(dev->bus, dev->constraints);
		} 
	}

	return 0;
}

int driver_scale(struct device_driver * drv, void * data)
{
	int level = (u32) data;
	int valid = 0; //+++validate_constraints(drv->bus, drv->constraints);
	int error = 0;

	if (drv->scale)
		error = drv->scale(drv->bus->bus_op, level);

	if (! valid && (level == DPM_SCALE))
		driver_powerdown(drv, NULL, DPM_SUSPEND_FOR_OP);
	else {
		struct list_head * entry, * next;
		struct device *dev;

		list_for_each_safe(entry,next,&drv->devices) {
			dev = container_of(entry,struct device,driver_list);
			device_op_check(dev);
		}
	}

        return error;
}

#endif /* linux-pm */

__initcall(dpm_event_queue_init);

EXPORT_SYMBOL(driver_register);
EXPORT_SYMBOL(driver_unregister);
EXPORT_SYMBOL(get_driver);
EXPORT_SYMBOL(put_driver);

EXPORT_SYMBOL(driver_create_file);
EXPORT_SYMBOL(driver_remove_file);
