/*
 * device.h - generic, centralized driver model
 *
 * Copyright (c) 2001 Patrick Mochel <mochel@osdl.org>
 *
 * This is a relatively simple centralized driver model.
 * The data structures were mainly lifted directly from the PCI
 * driver model. These are thought to be the common fields that
 * are relevant to all device buses.
 *
 * All the devices are arranged in a tree. All devices should
 * have some sort of parent bus of whom they are children of.
 * Devices should not be direct children of the system root.
 *
 * Device drivers should not directly call the device_* routines
 * or access the contents of struct device directly. Instead,
 * abstract that from the drivers and write bus-specific wrappers
 * that do it for you.
 *
 * See Documentation/driver-model.txt for more information.
 */

#ifndef _DEVICE_H_
#define _DEVICE_H_

#if 1 /* linux-pm */
#include <linux/stringify.h>

/**
 * container_of - cast a member of a structure out to the containing structure
 *
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 *
 */
#define container_of(ptr, type, member) ({			\
        const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
        (type *)( (char *)__mptr - offsetof(type,member) );})

#endif /* linux-pm */

#include <linux/config.h>
#include <linux/ioport.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <asm/atomic.h>

#define DEVICE_NAME_SIZE	50
#define DEVICE_NAME_HALF	__stringify(20)	/* Less than half to accommodate slop */
#define DEVICE_ID_SIZE		32
#define BUS_ID_SIZE		16


enum {
	SUSPEND_NOTIFY,
	SUSPEND_SAVE_STATE,
	SUSPEND_DISABLE,
	SUSPEND_POWER_DOWN,
};

enum {
	RESUME_POWER_ON,
	RESUME_RESTORE_STATE,
	RESUME_ENABLE,
};

enum device_state {
	DEVICE_UNINITIALIZED	= 0,
	DEVICE_INITIALIZED	= 1,
	DEVICE_REGISTERED	= 2,
	DEVICE_GONE		= 3,
};

#if 1 /* linux-pm */
enum {
	DPM_POWER_ON,
	DPM_POWER_OFF,
	DPM_SUSPEND_FOR_OP,
	DPM_RESUME_FOR_OP,
};

enum {
	DPM_SCALE_NOTIFY,
	DPM_SCALE,
	DPM_SCALE_DISABLE,
};

enum {
	DPM_PARAM_BUS_FREQ,
	DPM_PARAM_EXT_CLOCK,
};

/* now Samsung24a0 has the maximum of dpm params FIXME */
#define DPM_PARAM_MAX 29

struct bus_op_param {
	int id;
	int val;
};

struct bus_op_point {
	int count;
	struct bus_op_param param[DPM_PARAM_MAX];
};

struct constraint_param {
	int id;
	int min;
	int max;
};

struct constraints {
	int asserted;
	int count;
	struct constraint_param param[DPM_PARAM_MAX];
};
#endif /* linux-pm */

struct device;
struct device_driver;
struct device_class;

struct bus_type {
	char			* name;

	struct subsystem	subsys;
	struct subsystem	drvsubsys;
	struct subsystem	devsubsys;
	struct list_head	devices;
	struct list_head	drivers;

#if 1 /* linux-pm */
	struct constraints	* constraints;
	struct bus_op_point	* bus_op;
#endif /* linux-pm */

	int		(*match)(struct device * dev, struct device_driver * drv);
	struct device * (*add)	(struct device * parent, char * bus_id);
	int		(*hotplug) (struct device *dev, char **envp, 
				    int num_envp, char *buffer, int buffer_size);
};


extern int bus_register(struct bus_type * bus);
extern void bus_unregister(struct bus_type * bus);

extern struct bus_type * get_bus(struct bus_type * bus);
extern void put_bus(struct bus_type * bus);


/* driverfs interface for exporting bus attributes */

struct bus_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct bus_type *, char * buf, size_t count, loff_t off);
	ssize_t (*store)(struct bus_type *, const char * buf, size_t count, loff_t off);
};

#define BUS_ATTR(_name,_mode,_show,_store)	\
struct bus_attribute bus_attr_##_name = { 		\
	.attr = {.name = __stringify(_name), .mode = _mode },	\
	.show	= _show,				\
	.store	= _store,				\
};

extern int bus_create_file(struct bus_type *, struct bus_attribute *);
extern void bus_remove_file(struct bus_type *, struct bus_attribute *);

struct device_driver {
	char			* name;
	struct bus_type		* bus;
	struct device_class	* devclass;

	struct semaphore	unload_sem;
	struct kobject		kobj;
	struct list_head	bus_list;
	struct list_head	class_list;
	struct list_head	devices;

	int	(*probe)	(struct device * dev);
	int 	(*remove)	(struct device * dev);
	void	(*shutdown)	(struct device * dev);
	int	(*suspend)	(struct device * dev, u32 state, u32 level);
	int	(*resume)	(struct device * dev, u32 level);
#if 1 /* linux-pm */
	int	(*scale)	(struct bus_op_point * op, u32 level);
#endif /* linux-pm */
};


extern int driver_register(struct device_driver * drv);
extern void driver_unregister(struct device_driver * drv);

extern struct device_driver * get_driver(struct device_driver * drv);
extern void put_driver(struct device_driver * drv);


/* driverfs interface for exporting driver attributes */

struct driver_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct device_driver *, char * buf, size_t count, loff_t off);
	ssize_t (*store)(struct device_driver *, const char * buf, size_t count, loff_t off);
};

#define DRIVER_ATTR(_name,_mode,_show,_store)	\
struct driver_attribute driver_attr_##_name = { 		\
	.attr = {.name = __stringify(_name), .mode = _mode },	\
	.show	= _show,				\
	.store	= _store,				\
};

extern int driver_create_file(struct device_driver *, struct driver_attribute *);
extern void driver_remove_file(struct device_driver *, struct driver_attribute *);


/*
 * device classes
 */
struct device_class {
	char			* name;
	u32			devnum;

	struct subsystem	subsys;
	struct subsystem	devsubsys;
	struct subsystem	drvsubsys;
	struct list_head	drivers;

	int	(*add_device)(struct device *);
	void	(*remove_device)(struct device *);
	int	(*hotplug)(struct device *dev, char **envp, 
			   int num_envp, char *buffer, int buffer_size);
};

extern int devclass_register(struct device_class *);
extern void devclass_unregister(struct device_class *);

extern struct device_class * get_devclass(struct device_class *);
extern void put_devclass(struct device_class *);


struct devclass_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct device_class *, char * buf, size_t count, loff_t off);
	ssize_t (*store)(struct device_class *, const char * buf, size_t count, loff_t off);
};

#define DEVCLASS_ATTR(_name,_str,_mode,_show,_store)	\
struct devclass_attribute devclass_attr_##_name = { 		\
	.attr = {.name	= _str,	.mode	= _mode },	\
	.show	= _show,				\
	.store	= _store,				\
};

extern int devclass_create_file(struct device_class *, struct devclass_attribute *);
extern void devclass_remove_file(struct device_class *, struct devclass_attribute *);


/*
 * device interfaces
 * These are the logical interfaces of device classes. 
 * These entities map directly to specific userspace interfaces, like 
 * device nodes.
 * Interfaces are registered with the device class they belong to. When
 * a device is registered with the class, each interface's add_device 
 * callback is called. It is up to the interface to decide whether or not
 * it supports the device.
 */

struct intf_data;

struct device_interface {
	char			* name;
	struct device_class	* devclass;

	struct kobject		kobj;
	struct list_head	devices;

	u32			devnum;

	int (*add_device)	(struct device *);
	int (*remove_device)	(struct intf_data *);
};

extern int interface_register(struct device_interface *);
extern void interface_unregister(struct device_interface *);


/*
 * intf_data - per-device data for an interface
 * Each interface typically has a per-device data structure 
 * that it allocates. It should embed one of these structures 
 * in that structure and call interface_add_data() to add it
 * to the device's list.
 * That will also enumerate the device within the interface
 * and create a driverfs symlink for it.
 */
struct intf_data {
	struct list_head	node;
	struct device_interface	* intf;
	struct device		* dev;
	u32			intf_num;
};

extern int interface_add_data(struct intf_data *);



struct device {
	struct list_head g_list;        /* node in depth-first order list */
	struct list_head node;		/* node in sibling list */
	struct list_head bus_list;	/* node in bus's list */
	struct list_head driver_list;
	struct list_head children;
	struct list_head intf_list;
	struct device 	* parent;

	struct kobject kobj;
	char	name[DEVICE_NAME_SIZE];	/* descriptive ascii string */
	char	bus_id[BUS_ID_SIZE];	/* position on parent bus */

	struct bus_type	* bus;		/* type of bus device is on */
	struct device_driver *driver;	/* which driver has allocated this
					   device */
	void		*driver_data;	/* data private to the driver */

	u32		class_num;	/* class-enumerated value */
	void		* class_data;	/* class-specific data */

	void		*platform_data;	/* Platform specific data (e.g. ACPI,
					   BIOS data relevant to device) */

	u32		power_state;  /* Current operating state. In
					   ACPI-speak, this is D0-D3, D0
					   being fully functional, and D3
					   being off. */

#if 1 /* linux-pm */
	struct constraints	*constraints;
#endif /* linux-pm */
	unsigned char *saved_state;	/* saved device state */

	void	(*release)(struct device * dev);
};

static inline struct device *
list_to_dev(struct list_head *node)
{
	return list_entry(node, struct device, node);
}

static inline struct device *
g_list_to_dev(struct list_head *g_list)
{
	return list_entry(g_list, struct device, g_list);
}

static inline void *
dev_get_drvdata (struct device *dev)
{
	return dev->driver_data;
}

static inline void
dev_set_drvdata (struct device *dev, void *data)
{
	dev->driver_data = data;
}

/*
 * High level routines for use by the bus drivers
 */
extern int device_register(struct device * dev);
extern void device_unregister(struct device * dev);
extern void device_initialize(struct device * dev);
extern int device_add(struct device * dev);
extern void device_del(struct device * dev);

/* driverfs interface for exporting device attributes */

struct device_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct device * dev, char * buf, size_t count, loff_t off);
	ssize_t (*store)(struct device * dev, const char * buf, size_t count, loff_t off);
};

#define DEVICE_ATTR(_name,_mode,_show,_store) \
struct device_attribute dev_attr_##_name = { 		\
	.attr = {.name = __stringify(_name), .mode = _mode },	\
	.show	= _show,				\
	.store	= _store,				\
};


extern int device_create_file(struct device *device, struct device_attribute * entry);
extern void device_remove_file(struct device * dev, struct device_attribute * attr);

/*
 * Platform "fixup" functions - allow the platform to have their say
 * about devices and actions that the general device layer doesn't
 * know about.
 */
/* Notify platform of device discovery */
extern int (*platform_notify)(struct device * dev);

extern int (*platform_notify_remove)(struct device * dev);


/**
 * get_device - atomically increment the reference count for the device.
 *
 */
extern struct device * get_device(struct device * dev);
extern void put_device(struct device * dev);

/* drivers/base/sys.c */

struct sys_root {
	u32		id;
	struct device 	dev;
	struct device	sysdev;
};

extern int sys_register_root(struct sys_root *);
extern void sys_unregister_root(struct sys_root *);


struct sys_device {
	char		* name;
	u32		id;
	struct sys_root	* root;
	struct device	dev;
};

extern int sys_device_register(struct sys_device *);
extern void sys_device_unregister(struct sys_device *);

extern struct bus_type system_bus_type;

/* drivers/base/platform.c */

struct platform_device {
	char		* name;
	u32		id;
	struct device	dev;
};

extern int platform_device_register(struct platform_device *);
extern void platform_device_unregister(struct platform_device *);

extern struct bus_type platform_bus_type;

/* drivers/base/power.c */
extern int device_suspend(u32 state, u32 level);
extern void device_resume(u32 level);
extern void device_shutdown(void);


/* drivers/base/firmware.c */
extern int firmware_register(struct subsystem *);
extern void firmware_unregister(struct subsystem *);

/* debugging and troubleshooting/diagnostic helpers. */
#ifdef DEBUG
#define dev_dbg(dev, format, arg...)		\
	printk (KERN_DEBUG "%s %s: " format ,	\
		(dev).driver->name , (dev).bus_id , ## arg)
#else
#define dev_dbg(dev, format, arg...) do {} while (0)
#endif

#define dev_err(dev, format, arg...)		\
	printk (KERN_ERR "%s %s: " format ,	\
		(dev).driver->name , (dev).bus_id , ## arg)
#define dev_info(dev, format, arg...)		\
	printk (KERN_INFO "%s %s: " format ,	\
		(dev).driver->name , (dev).bus_id , ## arg)
#define dev_warn(dev, format, arg...)		\
	printk (KERN_WARNING "%s %s: " format ,	\
		(dev).driver->name , (dev).bus_id , ## arg)

#if 1 /* linux-pm */
extern int bus_scale(struct bus_type * bus, struct bus_op_point * op);
extern void assert_constraints(struct bus_type *bus, 
			       struct constraints *dev_constraints);
extern void deassert_constraints(struct bus_type *bus, 
				 struct constraints *drv_constraints);
extern int validate_constraints(struct bus_type *bus, 
				struct constraints *constraints);
extern void bus_reeval_constraints(struct bus_type * bus);
extern int driver_powerup(struct device_driver *drv, struct device *dev,
			  u32 level);
extern int driver_powerdown(struct device_driver *drv, struct device *dev,
			    u32 level);
extern int device_powerup(struct device *dev);
extern int device_powerdown(struct device *dev);
extern int driver_scale(struct device_driver * drv, void * data);

#define DPM_EVENT_QUEUE_SIZE 64
#define DPM_EVENT_STRING_SIZE 256
struct dpm_device_event_queue {
	int head;
	int tail;
	
	wait_queue_head_t proc_list;

	char *str[DPM_EVENT_QUEUE_SIZE];
};
char *dpm_get_from_device_event_queue(void);
inline int dpm_event_queue_empty(void);

#endif /* linux-pm */

#endif /* _DEVICE_H_ */
