#include <asm/system.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/module.h>

static struct bus_type mx21_local_bus = {
        .name = "mx21-ldm",
};

static struct device mx21_local_bus_device = {
        .name = "MX21 Local Bus",
        .bus_id = "mx21_local_bus",
        .bus = &mx21_local_bus,
        .power_state = DPM_POWER_ON,
};

void mx21_ldm_bus_register(struct device *device,
                          struct device_driver *driver)
{
        driver->bus = &mx21_local_bus;
        (void) driver_register(driver);
        device->parent = &mx21_local_bus_device;
        device->bus = &mx21_local_bus;
        device_register(device);
}

void mx21_ldm_bus_unregister(struct device *device,
                          struct device_driver *driver)
{
        (void) driver_unregister(driver);
        device_unregister(device);
}

void mx21_ldm_device_register(struct device *device)
{
	device->parent = &mx21_local_bus_device;
	device->bus = &mx21_local_bus;
	device_register(device);
}

void mx21_ldm_device_unregister(struct device *device)
{
	device_unregister(device);
}

void mx21_ldm_driver_register(struct device_driver *driver)
{
	driver->bus = &mx21_local_bus;
	(void) driver_register(driver);
}

void mx21_ldm_driver_unregister(struct device_driver *driver)
{
	driver_unregister(driver);
}

void __init mx21_ldm_init(void)
{
        bus_register(&mx21_local_bus);
        device_register(&mx21_local_bus_device);
}

EXPORT_SYMBOL(mx21_ldm_bus_register);
EXPORT_SYMBOL(mx21_ldm_bus_unregister);
EXPORT_SYMBOL(mx21_ldm_driver_register);
EXPORT_SYMBOL(mx21_ldm_device_register);
EXPORT_SYMBOL(mx21_ldm_driver_unregister);
EXPORT_SYMBOL(mx21_ldm_device_unregister);
