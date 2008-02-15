#include <asm/system.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/module.h>

static int tipb_bus_match(struct device *dev, struct device_driver *drv);

struct bus_type mpu_private_bus = {
        name:	"mpu-private",
        match:	tipb_bus_match,
};

struct device mpu_private_device = {
	name:		"MPU Private TI Peripheral Bus Controller",
	bus_id:		"mpu-private",
	parent:		NULL,
	bus:		&mpu_private_bus,
	driver:		NULL,
	power_state:	DPM_POWER_ON,
};

struct bus_type mpu_public_bus = {
        name:	"mpu-public",
        match:	tipb_bus_match,
};

struct device mpu_public_device = {
	name:		"MPU Public TI Peripheral Bus Controller",
	bus_id:		"mpu-public",
	parent:		NULL,
	bus:		&mpu_public_bus,
	driver:		NULL,
	power_state:	DPM_POWER_ON,
};

struct bus_type dsp_public_bus = {
        name:   "dsp-public",
        match:  tipb_bus_match,
};

struct device dsp_public_device = {
        name:		"DSP Public TI Peripheral Bus Controller",
        bus_id:		"dsp-public",
        parent:		NULL,
        bus:		&dsp_public_bus,
        driver:		NULL,
        power_state:	DPM_POWER_ON,
};

static int
tipb_bus_match(struct device *dev, struct device_driver *drv)
{
	if (strcmp(dev->bus_id, "lcd") == 0)
		return (strcmp(drv->name, "omapfb") == 0);
	if (strncmp(dev->bus_id, "uart", 4) == 0)
		return (strcmp(drv->name, "serial") == 0);
        if (strcmp(dev->bus_id, "I2S_Audio") == 0) {
	        return (strcmp(drv->name, "omap1610-aic23"  ) == 0) ||
                       (strcmp(drv->name, "omap1610-tsc2101") == 0);
	}
        if (strcmp(dev->bus_id, "innovator-ts") == 0)
	        return (strcmp(drv->name, "innovator-touch_screen") == 0);
        if (strcmp(dev->bus_id, "I2C") == 0)
	        return (strcmp(drv->name, "i2c-adap-omap1610") == 0);
        if (strcmp(dev->bus_id, "eth") == 0)
	        return (strcmp(drv->name, "smc9194") == 0);
        if (strcmp(dev->bus_id, "IrDA") == 0)
	        return (strcmp(drv->name, "omap1610-IrDA") == 0);
	if (strcmp(dev->bus_id, "MMC/SD") == 0)
	        return (strcmp(drv->name, "omap-mmc") == 0);
	if (strcmp(dev->bus_id, "OHCI") == 0)
	        return (strcmp(drv->name, "omap-ohci") == 0);
	if (strcmp(dev->bus_id, "usbd") == 0)
	        return (strcmp(drv->name, "usb-device") == 0);
	if (strcmp(dev->bus_id, "Camera") == 0)
	        return (strcmp(drv->name, "omap-camera") == 0);

	return 0;
}

void mpu_private_register(struct device *device,
			  struct device_driver *driver)
{
	driver->bus = &mpu_private_bus;
	(void) driver_register(driver);
	device->parent = &mpu_private_device;
	device->bus = &mpu_private_bus;
	device_register(device);
}

void mpu_private_unregister(struct device *device,
			  struct device_driver *driver)
{
	(void) driver_unregister(driver);
	device_unregister(device);
}

void mpu_public_device_register(struct device *device)
{
	device->parent = &mpu_public_device;
	device->bus = &mpu_public_bus;
	device_register(device);
}

void mpu_public_device_unregister(struct device *device)
{
	device_unregister(device);
}

void mpu_public_driver_register(struct device_driver *driver)
{
	driver->bus = &mpu_public_bus;
	(void) driver_register(driver);
}

void mpu_public_driver_unregister(struct device_driver *driver)
{
	driver_unregister(driver);
}

void dsp_public_device_register(struct device *device)
{
       device->parent = &dsp_public_device;
       device->bus = &dsp_public_bus;
       device_register(device);
}

void dsp_public_device_unregister(struct device *device)
{
       device_unregister(device);
}

void dsp_public_driver_register(struct device_driver *driver)
{
       driver->bus = &dsp_public_bus;
       (void) driver_register(driver);
}

void dsp_public_driver_unregister(struct device_driver *driver)
{
       driver_unregister(driver);
}

void __init innovator_ldm_setup(void)
{
	bus_register(&mpu_private_bus);
	device_register(&mpu_private_device);
	bus_register(&mpu_public_bus);
	device_register(&mpu_public_device);
        bus_register(&dsp_public_bus);
        device_register(&dsp_public_device);
}

EXPORT_SYMBOL(dsp_public_driver_register);
EXPORT_SYMBOL(dsp_public_device_register);
EXPORT_SYMBOL(dsp_public_driver_unregister);
EXPORT_SYMBOL(dsp_public_device_unregister);
EXPORT_SYMBOL(mpu_public_driver_register);
EXPORT_SYMBOL(mpu_public_device_register);
EXPORT_SYMBOL(mpu_public_driver_unregister);
EXPORT_SYMBOL(mpu_public_device_unregister);
EXPORT_SYMBOL(mpu_private_register);
EXPORT_SYMBOL(mpu_private_unregister);
