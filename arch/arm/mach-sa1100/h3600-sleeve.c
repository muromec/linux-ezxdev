/*
*
* Driver for iPAQ H3xxx Extension Packs
*
* Copyright 2000 Compaq Computer Corporation.
*
* Use consistent with the GNU GPL is permitted,
* provided that this copyright notice is
* preserved in its entirety in all copies and derived works.
*
* COMPAQ COMPUTER CORPORATION MAKES NO WARRANTIES, EXPRESSED OR IMPLIED,
* AS TO THE USEFULNESS OR CORRECTNESS OF THIS CODE OR ITS
* FITNESS FOR ANY PARTICULAR PURPOSE.
*
* Author: Jamey Hicks.
*
*/

#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <linux/pm.h>

#ifdef CONFIG_DRIVERFS_FS
#include <linux/device.h>
#endif

/* SA1100 serial defines */
#include <asm/arch/hardware.h>
#include <linux/serial_reg.h>
#include <asm/arch/irqs.h>
#include <asm/arch/h3600-sleeve.h>
#include <asm/arch/h3600_hal.h>

static void msleep(unsigned int msec)
{
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout( (msec * HZ) / 1000);
}

static struct sleeve_dev *sleeve_dev;


/*******************************************************************************/

#ifdef CONFIG_HOTPLUG

#ifndef FALSE
#define FALSE	(0)
#endif
#ifndef TRUE
#define TRUE	(!FALSE)
#endif

extern char hotplug_path[];

extern int call_usermodehelper(char *path, char **argv, char **envp);

static void run_sbin_hotplug(struct sleeve_dev *sdev, int insert)
{
	int i;
	char *argv[3], *envp[8];
	char id[64], sub_id[64], name[100];

	if (!hotplug_path[0])
		return;

	i = 0;
	argv[i++] = hotplug_path;
	argv[i++] = "sleeve";
	argv[i] = 0;

        printk(KERN_CRIT __FUNCTION__ ":%d * hotplug_path=%s\n", __LINE__, hotplug_path);
	i = 0;
	/* minimal command environment */
	envp[i++] = "HOME=/";
	envp[i++] = "PATH=/sbin:/bin:/usr/sbin:/usr/bin";
	
	/* other stuff we want to pass to /sbin/hotplug */
        sprintf(id,     "VENDOR_ID=%04x",   sdev->vendor);
        sprintf(sub_id, "DEVICE_ID=%04x",   sdev->device);
	if ( sdev->driver != NULL )
		sprintf(name,   "DEVICE_NAME=%s", sdev->driver->name);
	else 
		sprintf(name,   "DEVICE_NAME=Unknown");

	envp[i++] = id;
	envp[i++] = sub_id;
	envp[i++] = name;
	if (insert)
		envp[i++] = "ACTION=add";
	else
		envp[i++] = "ACTION=remove";
	envp[i] = 0;

	call_usermodehelper (argv [0], argv, envp);
}
#else
static void run_sbin_hotplug(struct sleeve_dev *sdev, int insert) { }
#endif /* CONFIG_HOTPLUG */

/*******************************************************************************/


static int h3600_sleeve_major = 0;
struct proc_dir_entry *proc_sleeve_dir = NULL;

static int h3600_sleeve_proc_read_eeprom(char *page, char **start, off_t off, int count, int *eof, void *data)
{
        char *p = page;
        int len = 0;
        int i;
        char mark = 0;
        int offset = off;
        int ret;

        h3600_spi_read(0, &mark, 1);
        h3600_spi_read(1, (char*)&len, 4);
#if 0
        len += 5; /* include the mark byte */
        if (mark != 0xaa || len == 0xFFFFFFFF)
                return 0;
#else
	len++;    /* Add one for the byte before? */
        if (len == -1)
                len = 256;
#endif

        if (len < (off+count)) {
                count = len-off;
                /* *eof = 1; */
        }
        for (i = 0; i < count; i+=8) {
                h3600_spi_read(offset, p, 8);
                p += 8;
                offset += 8;
        }
	*start = page + off;
        ret = count;

        return ret;
}

static int h3600_sleeve_proc_read_device(char *page, char **start, off_t off, int count, int *eof, void *data)
{
        char *p = page;
        int len = 0;

        if (sleeve_dev) {
                p += sprintf(p, "vendor=0x%04x\n", sleeve_dev->vendor);
                p += sprintf(p, "device=0x%04x\n", sleeve_dev->device);
                if (sleeve_dev->driver != NULL) {
                        p += sprintf(p, "driver=%s\n", sleeve_dev->driver->name);
                }
        }

        len = (p - page) - off;
	*start = page + off;
        return len;
}

static int h3600_sleeve_proc_read_features(char *page, char **start, off_t off, int count, int *eof, void *data)
{
        char *p = page;
        int len = 0;

        if (sleeve_dev && sleeve_dev->driver) {
		unsigned int f = sleeve_dev->driver->features;
		if ( f & SLEEVE_HAS_CF_MASK )
			p += sprintf(p, "CF slots=%d\n", SLEEVE_GET_CF(f));
		if ( f & SLEEVE_HAS_PCMCIA_MASK )
			p += sprintf(p, "PCMCIA slots=%d\n", SLEEVE_GET_PCMCIA(f));
		
		if ( f & SLEEVE_HAS_BLUETOOTH ) 
			p += sprintf(p, "Bluetooth\n");
		if ( f & SLEEVE_HAS_GPS ) 
			p += sprintf(p, "GPS\n");
		if ( f & SLEEVE_HAS_GPRS ) 
			p += sprintf(p, "GPRS\n");
		if ( f & SLEEVE_HAS_CAMERA ) 
			p += sprintf(p, "Camera\n");
		if ( f & SLEEVE_HAS_ACCELEROMETER ) 
			p += sprintf(p, "Accelerometer\n");
		if ( f & SLEEVE_HAS_FLASH ) 
			p += sprintf(p, "Flash\n");
		if ( f & SLEEVE_HAS_BARCODE_READER ) 
			p += sprintf(p, "Barcode Reader\n");
		if ( f & SLEEVE_HAS_802_11B ) 
			p += sprintf(p, "802.11b\n");
		if ( f & SLEEVE_HAS_EBAT ) 
			p += sprintf(p, "EBAT discharging\n");

		if ( f & SLEEVE_HAS_FIXED_BATTERY ) 
			p += sprintf(p, "Fixed battery\n");
		if ( f & SLEEVE_HAS_REMOVABLE_BATTERY ) 
			p += sprintf(p, "Removable battery\n");
	}

        len = (p - page) - off;
	*start = page + off;
        return len;
}

int h3600_sleeve_proc_attach_device(struct sleeve_dev *dev)
{
        dev->procent = NULL;
	return 0;
}

int h3600_sleeve_proc_detach_device(struct sleeve_dev *dev)
{
	struct proc_dir_entry *e;

	if ((e = dev->procent) != NULL) {
#if Broken 
		if (atomic_read(e->count) != 0)
			return -EBUSY;
#endif
		remove_proc_entry(e->name, proc_sleeve_dir);
		dev->procent = NULL;
	}
	return 0;
}

/***********************************************************************************/

#ifdef CONFIG_DRIVERFS_FS
static int sleeve_iobus_scan(struct iobus *bus)
{
        printk(__FUNCTION__ "\n");
        return 0;
}

static int sleeve_iobus_add_device(struct iobus *bus, char *devname)
{
        printk(__FUNCTION__ ": devname=%s\n", devname);
        return 0;
}

static struct device_driver sleeve_device_driver = {
        probe: NULL,
};

static struct device sleeve_device = {
        name: "sleeve device",
        driver: &sleeve_device_driver,
};

static struct iobus_driver sleeve_iobus_driver = {
        name: "sleeve bus",
        scan: sleeve_iobus_scan,
        add_device: sleeve_iobus_add_device,
};

struct iobus h3600_sleeve_iobus = {
        self: &sleeve_device,
	bus_id: "sleeve",
	name:	"H3600 Sleeve Bus",
        driver: &sleeve_iobus_driver,
};
EXPORT_SYMBOL(h3600_sleeve_iobus);
#endif

/*******************************************************************************/



static LIST_HEAD(sleeve_drivers);

const struct sleeve_device_id *
h3600_sleeve_match_device(const struct sleeve_device_id *ids, struct sleeve_dev *dev)
{
        while (ids->vendor) {
                if ((ids->vendor == SLEEVE_ANY_ID || ids->vendor == dev->vendor) &&
		    (ids->device == SLEEVE_ANY_ID || ids->device == dev->device))
			return ids;
                ids++;
        }
        return NULL;
}

static int h3600_sleeve_announce_device(struct sleeve_driver *drv, struct sleeve_dev *dev)
{
        const struct sleeve_device_id *id = NULL;

        if (drv->id_table) {
                id = h3600_sleeve_match_device(drv->id_table, dev);
                if (!id)
                        return 0;
        }

        dev->driver = drv;
        if (drv->probe(dev, id) >= 0) {

#ifdef CONFIG_DRIVERFS_FS
                strncpy(dev->dev.name, dev->driver->name, DEVICE_NAME_SIZE);
                strcpy(dev->dev.bus_id, "0");
                dev->dev.driver = drv->driver;
                dev->dev.driver_data = dev;
                device_register(&dev->dev);
#endif
		if (drv->features & SLEEVE_HAS_EBAT)
			h3600_set_ebat();

                run_sbin_hotplug(dev,TRUE);
                return 1;
        }

        return 0;
}

static void h3600_sleeve_insert(void) 
{
	struct list_head *item;

	set_h3600_egpio(IPAQ_EGPIO_OPT_NVRAM_ON);

        msleep(250);

	if ( sleeve_dev->vendor || sleeve_dev->device )
		printk(KERN_CRIT __FUNCTION__ ": *** Error - sleeve insert without sleeve remove ***\n");

        h3600_spi_read(6, (char*)&sleeve_dev->vendor, 2);
        h3600_spi_read(8, (char*)&sleeve_dev->device, 2);

        printk(KERN_CRIT __FUNCTION__ ": vendorid=%#x devid=%#x\n", sleeve_dev->vendor, sleeve_dev->device);
			
#ifdef CONFIG_PROC_FS
        h3600_sleeve_proc_attach_device(sleeve_dev);
#endif
#ifdef CONFIG_DRIVERFS_FS
        device_init_dev(&sleeve_dev->dev);
        sleeve_dev->dev.parent = &h3600_sleeve_iobus;
#endif

        for (item=sleeve_drivers.next; item != &sleeve_drivers; item=item->next) {
                struct sleeve_driver *drv = list_entry(item, struct sleeve_driver, node);
                if (h3600_sleeve_announce_device(drv, sleeve_dev)) {
			printk(__FUNCTION__ ": matched driver %s\n", drv->name);
			return;
		}
        }

	printk(KERN_CRIT __FUNCTION__ ": unrecognized sleeve\n");
}

static void h3600_sleeve_eject(void) 
{
	/* Notify user of device removal */
	run_sbin_hotplug(sleeve_dev, FALSE);

        printk(KERN_CRIT __FUNCTION__ "-9- sleeve_dev->driver=%p sleeve removed\n", sleeve_dev->driver);
        if (sleeve_dev->driver && sleeve_dev->driver->remove)
                sleeve_dev->driver->remove(sleeve_dev);

        memset(sleeve_dev, 0, sizeof(struct sleeve_dev));
        clr_h3600_egpio(IPAQ_EGPIO_OPT_NVRAM_ON);
	clr_h3600_egpio(IPAQ_EGPIO_OPT_ON);
}

static void h3600_sleeve_suspend(void)
{
        printk(KERN_CRIT __FUNCTION__ "-9- sleeve_dev->driver=%p sleeve suspended\n", sleeve_dev->driver);
        if (sleeve_dev->driver && sleeve_dev->driver->suspend)
                sleeve_dev->driver->suspend(sleeve_dev);
	clr_h3600_egpio(IPAQ_EGPIO_OPT_ON);
}

static void h3600_sleeve_resume(void)
{
        printk(KERN_CRIT __FUNCTION__ "-9- sleeve_dev->driver=%p sleeve resumed\n", sleeve_dev->driver);
        if (sleeve_dev->driver && sleeve_dev->driver->resume)
                sleeve_dev->driver->resume(sleeve_dev);
}

int h3600_current_sleeve( void )
{
	return H3600_SLEEVE_ID( sleeve_dev->vendor, sleeve_dev->device );
}

EXPORT_SYMBOL(h3600_current_sleeve);

/*******************************************************************************/

/**
 * h3600_sleeve_register_driver: Registers a driver for an iPAQ extension pack.
 * @drv: the sleeve driver
 */
int h3600_sleeve_register_driver(struct sleeve_driver *drv)
{
 	list_add_tail(&drv->node, &sleeve_drivers);

	if ( h3600_sleeve_announce_device( drv, sleeve_dev ) ) {
		printk(__FUNCTION__ ": registered a driver that matches the current sleeve %s\n", drv->name);
		return 1;
	}

	return 0;
}

/**
 * h3600_sleeve_unregister_driver: Unregisters a driver for an iPAQ extension pack.
 * @drv: the sleeve driver
 */
void
h3600_sleeve_unregister_driver(struct sleeve_driver *drv)
{
        list_del(&drv->node);
        if (sleeve_dev->driver == drv) {
                if (drv->remove)
                        drv->remove(sleeve_dev);
                sleeve_dev->driver = NULL;
        }
}

EXPORT_SYMBOL(h3600_sleeve_register_driver);
EXPORT_SYMBOL(h3600_sleeve_unregister_driver);

static int h3600_sleeve_proc_read_drivers(char *page, char **start, off_t off, int count, int *eof, void *data)
{
        char *p = page;
        int len = 0;
	struct list_head *item;

        for (item=sleeve_drivers.next; item != &sleeve_drivers; item=item->next) {
                struct sleeve_driver *drv = list_entry(item, struct sleeve_driver, node);
		p += sprintf(p, "%35s : ", drv->name);
		if ( drv->id_table ) {
			const struct sleeve_device_id *id = drv->id_table;
			while ( id->vendor ) {
				p += sprintf(p, "(0x%04x 0x%04x) ", id->vendor, id->device);
				id++;
			}
			p += sprintf(p,"\n");
		}
		else {
			p += sprintf(p,"ANY\n");
		}
        }

        len = (p - page) - off;
	*start = page + off;
        return len;
}

/*******************************************************************************/

static void h3600_sleeve_task_handler(void *x)
{
	int opt_det;
        /* debounce */
        msleep(100);
        h3600_get_option_detect(&opt_det);

        if (opt_det) {
                h3600_sleeve_insert();
        } else {
                h3600_sleeve_eject();
        }

}

static struct tq_struct h3600_sleeve_task = {
	routine: h3600_sleeve_task_handler
};

void h3600_sleeve_interrupt( int present )
{
	schedule_task(&h3600_sleeve_task);
}

/***********************************************************************************/

/*    Power management callbacks  */

#ifdef CONFIG_PM
static struct pm_dev *h3600_sleeve_pm_dev;
static int suspended = 0;

static int h3600_sleeve_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
{
	if (0) printk(__FUNCTION__ ": sleeve pm callback %d\n", req);

	switch (req) {
	case PM_SUSPEND: /* Enter D1-D3 */
		h3600_sleeve_suspend();
		suspended = 1;
                break;
	case PM_RESUME:  /* Enter D0 */
		if ( suspended ) {
			h3600_sleeve_resume();
			suspended = 0;
		}
		break;
        }
        return 0;
}
#endif /* CONFIG_PM */


/*******************************************************************************/

static struct ctl_table sleeve_table[] = 
{
	{1, "eject", NULL, 0, 0600, NULL, (proc_handler *)&h3600_sleeve_eject},
	{2, "insert", NULL, 0, 0600, NULL, (proc_handler *)&h3600_sleeve_insert},
	{0}
};
static struct ctl_table sleeve_dir_table[] = 
{
	{BUS_SLEEVE, "sleeve", NULL, 0, 0555, sleeve_table},
        {0}
};
static struct ctl_table bus_dir_table[] = 
{
	{CTL_BUS, "bus", NULL, 0, 0555, sleeve_dir_table},
        {0}
};
static struct ctl_table_header *sleeve_ctl_table_header = NULL;

static struct h3600_driver_ops hal_driver_ops = {
	option_detect : h3600_sleeve_interrupt,
};

int __init h3600_sleeve_init_module(void)
{
	int result;

        if (!machine_is_h3xxx())
		return -ENODEV;

        clr_h3600_egpio(IPAQ_EGPIO_OPT_NVRAM_ON);
	clr_h3600_egpio(IPAQ_EGPIO_OPT_ON);

        sleeve_dev = (struct sleeve_dev *)kmalloc(sizeof(struct sleeve_dev), GFP_KERNEL);
        if (sleeve_dev == NULL) {
		printk(__FUNCTION__ ": kmalloc failed sleeve_dev=%p\n", sleeve_dev);
		return -ENOMEM;
	}
        memset(sleeve_dev, 0, sizeof(struct sleeve_dev));

	result = h3600_hal_register_driver( &hal_driver_ops );

#ifdef CONFIG_PROC_FS
        proc_sleeve_dir = proc_mkdir("sleeve", proc_bus);
        if (proc_sleeve_dir) {
                create_proc_read_entry("device", 0, proc_sleeve_dir, h3600_sleeve_proc_read_device, 0);
                create_proc_read_entry("drivers", 0, proc_sleeve_dir, h3600_sleeve_proc_read_drivers, 0);
                create_proc_read_entry("eeprom", 0, proc_sleeve_dir, h3600_sleeve_proc_read_eeprom, 0);
                create_proc_read_entry("features", 0, proc_sleeve_dir, h3600_sleeve_proc_read_features, 0);
        }
#endif

#ifdef CONFIG_PM
	h3600_sleeve_pm_dev = pm_register(PM_UNKNOWN_DEV, PM_SYS_UNKNOWN, h3600_sleeve_pm_callback);
#endif

        sleeve_ctl_table_header = register_sysctl_table(bus_dir_table, 0);

#ifdef CONFIG_DRIVERFS_FS
        device_register(&sleeve_device);
        iobus_init(&h3600_sleeve_iobus);
        iobus_register(&h3600_sleeve_iobus);
#endif

        if (!result)
                schedule_task(&h3600_sleeve_task);

	if (!result)
                printk(KERN_CRIT "init_module successful init major= %d\n", h3600_sleeve_major);
	else
		printk(__FUNCTION__ ": error from irq %d\n", result);

	return result;
}

void h3600_sleeve_cleanup_module(void)
{
	flush_scheduled_tasks(); /* make sure all tasks have run */
	h3600_hal_unregister_driver( &hal_driver_ops );

#ifdef CONFIG_PM
        pm_unregister(h3600_sleeve_pm_dev);
#endif

#ifdef CONFIG_PROC_FS
        if (proc_sleeve_dir) {
		remove_proc_entry("features", proc_sleeve_dir);
                remove_proc_entry("device", proc_sleeve_dir);
                remove_proc_entry("drivers", proc_sleeve_dir);
                remove_proc_entry("eeprom", proc_sleeve_dir);
                remove_proc_entry("sleeve", proc_bus);
        }
#endif
        unregister_sysctl_table(sleeve_ctl_table_header);

#ifdef CONFIG_DRIVERFS_FS
        /* no unregister functions? */
#endif
}

module_init(h3600_sleeve_init_module);
module_exit(h3600_sleeve_cleanup_module);

