/*
 * linux/arch/arm/mach-pxa/pxabus_ldm.c
 *
 * Author:     Echo Engineering
 * Created:    Oct 05, 2003
 * Copyright:  MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Copyright (C) 2005 - Motorola
 *
 * history:
 * Modified by w15879 for Motorola EZX platform.
 */

#include <asm/system.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/module.h>

/*
 * Tthis file came from ppc/platforms/ocp_ldm.c for the PPC "on chip
 * peripheral" "processor local bus"
 */

/* PXA System Bus */

//#define PXASYS_BUS_DEBUG 1
#if PXASYS_BUS_DEBUG
static unsigned int pxasys_bus_debug = 1;
#else
#define pxasys_bus_debug 0
#endif

static int pxasys_bus_match(struct device *dev, struct device_driver *drv);

static struct constraints pxasys_constraints = {
	count:		0,
};

static struct bus_op_point pxasys_op = {
	count:		0,
};

struct bus_type pxasys_bus = {
	name:		"pxasys",
	match:		pxasys_bus_match,
	constraints:	&pxasys_constraints,
	bus_op:		&pxasys_op,
};

struct device pxasys_device = {
	name:		"PXA System Bus Controller",
	bus_id:		"pxasys",
	parent:		NULL,
	bus:		&pxasys_bus,
	driver:		NULL,
	power_state:	DPM_POWER_ON,
};

static int
pxasys_bus_match(struct device *dev, struct device_driver *drv)
{
	if (pxasys_bus_debug)
	  printk("+++: pxasys_bus_match() for %s:%s\n",
		 dev->bus_id, drv->name);
  
	if (strcmp(dev->bus_id, "lcd") == 0)
          return (strcmp(drv->name, "pxafb") == 0);

	if (strcmp(dev->bus_id, "lcd1") == 0)
          return (strcmp(drv->name, "pxafb1") == 0);

	if (strcmp(dev->bus_id, "pcmcia") == 0)
          return (strcmp(drv->name, "pxapcmcia") == 0); 
	if (strcmp(dev->bus_id, "usb-ohci") == 0)
          return (strcmp(drv->name, "usb-ohci-wmmx") == 0); 

	if (pxasys_bus_debug)
	  printk("pxasys_bus_match(): Unhandled bus ID:driver name: %s:%s\n",
	       dev->bus_id, drv->name);
	return 0;
}

void pxasys_device_register(struct device *device)
{
	device->parent = &pxasys_device;
	device->bus = &pxasys_bus;
	device_register(device);
	return;
}

void pxasys_driver_register(struct device_driver *driver)
{
	driver->bus = &pxasys_bus;
	(void) driver_register(driver);
	return;
}

void pxasys_driver_unregister(struct device_driver *driver)
{
	driver_unregister(driver);  
	return;
}

void pxasys_device_unregister(struct device *device)
{
	device_unregister(device);
	return;
}

/*
 * PXA OPB Onboard Peripheral Bus
 */

static int pxaopb_bus_match(struct device *dev, struct device_driver *drv);

static struct constraints pxaopb_constraints = {
	count:		0,
};

static struct bus_op_point pxaopb_op = {
	count:		0,
};

struct bus_type pxaopb_bus = {
	name:		"pxaopb",
	match:		pxaopb_bus_match,
	constraints:	&pxaopb_constraints,
	bus_op:		&pxaopb_op,
};

struct device pxaopb_device = {
	name:		"PXA OPB Onboard Peripheral Bus Controller",
	bus_id:		"pxaopb",
	parent:		NULL,
	bus:		&pxaopb_bus,
	driver:		NULL,
	power_state:	DPM_POWER_ON,
};

static int
pxaopb_bus_match(struct device *dev, struct device_driver *drv)
{
	if (pxasys_bus_debug)
	  printk("+++: pxaopb_bus_match() for %s:%s\n",
		 dev->bus_id, drv->name);
  
	if (strcmp(dev->bus_id, "ac97") == 0)
          return (strcmp(drv->name, "ac97") == 0);       
	if (strcmp(dev->bus_id, "mmc") == 0)
          return (strcmp(drv->name, "mmc") == 0);       
	if (strcmp(dev->bus_id, "pxai2c") == 0)
          return (strcmp(drv->name, "pxa-i2c") == 0);
	if (strcmp(dev->bus_id, "pxairda") == 0)
          return (strcmp(drv->name, "pxaficp-irda") == 0);
	if (strcmp(dev->bus_id, "pxakpd") == 0)
          return (strcmp(drv->name, "pxa-kpd") == 0);       
	if (strcmp(dev->bus_id, "pxartc") == 0)
          return (strcmp(drv->name, "pxa-rtc") == 0);
	if (strncmp(dev->bus_id, "uart", 4) == 0)
          return (strcmp(drv->name, "serial") == 0);
	if (strcmp(dev->bus_id, "ucb1x00") == 0)
          return (strcmp(drv->name, "ucb1x00") == 0);	
	if (strcmp(dev->bus_id, "ucb1x00_ts") == 0)
          return (strcmp(drv->name, "ucb1x00_ts") == 0);	
	if (strcmp(dev->bus_id, "usbd") == 0)
          return (strcmp(drv->name, "usb-device") == 0);       

	if (pxasys_bus_debug)
	  printk("pxaopb_bus_match(): Unhandled bus ID:driver name: %s:%s\n",
	       dev->bus_id, drv->name);
	return 0;
}

void pxaopb_device_register(struct device *device)
{
	device->parent = &pxaopb_device;
	device->bus = &pxaopb_bus;
	device_register(device);
	return;
}

void pxaopb_driver_register(struct device_driver *driver)
{
	driver->bus = &pxaopb_bus;
	(void) driver_register(driver);
	return;
}

void pxaopb_device_unregister(struct device *device)
{
	device_unregister(device);  
	return;
}

void pxaopb_driver_unregister(struct device_driver *driver)
{
	driver_unregister(driver);  
	return;
}

/*
 * PXA EBC External Bus Controller
 */

static int pxaebc_bus_match(struct device *dev, struct device_driver *drv);

static struct constraints pxaebc_constraints = {
	count:		0,
};

static struct bus_op_point pxaebc_op = {
	count:		0,
};

struct bus_type pxaebc_bus = {
	name:	        "pxaebc",
	match:		pxaebc_bus_match,
	constraints:	&pxaebc_constraints,
	bus_op:		&pxaebc_op,
};

struct device pxaebc_device = {
	name:		"PXA EBC External Bus Controller",
	bus_id:		"pxaebc",
	parent:		NULL,
	bus:		&pxaebc_bus,
	driver:		NULL,
	power_state:	DPM_POWER_ON,
};

static int
pxaebc_bus_match(struct device *dev, struct device_driver *drv)
{
	if (pxasys_bus_debug)
	  printk("+++: pxaebc_bus_match() for %s:%s\n",
		 dev->bus_id, drv->name);
	
	if (strcmp(dev->bus_id, "net") == 0)
          return (strcmp(drv->name, "smc91x") == 0);
	if (strcmp(dev->bus_id, "video") == 0)
 	  return (strcmp(drv->name, "camera") == 0);

	if (pxasys_bus_debug)
	  printk("pxaebc_bus_match(): Unhandled bus ID:driver name: %s:%s\n",
	       dev->bus_id, drv->name);
	return 0;
}

void pxaebc_device_register(struct device *device)
{
	device->parent = &pxaebc_device;
	device->bus = &pxaebc_bus;
	device_register(device);
	return;
}

void pxaebc_driver_register(struct device_driver *driver)
{
	driver->bus = &pxaebc_bus;
	(void) driver_register(driver);
	return;
}

void pxaebc_device_unregister(struct device *device)
{
	device_unregister(device);  
	return;
}

void pxaebc_driver_unregister(struct device_driver *driver)
{
	driver_unregister(driver);  
	return;
}

void __init pxa_ldm_init(void)
{
	if (pxasys_bus_debug)
	  printk("+++: in pxa_ldm_init (aka ocp_ldm_init)\n");
	bus_register(&pxasys_bus);
	device_register(&pxasys_device);
	bus_register(&pxaopb_bus);
	device_register(&pxaopb_device);
	bus_register(&pxaebc_bus);
	device_register(&pxaebc_device);
	return;
}

EXPORT_SYMBOL(pxasys_bus);
EXPORT_SYMBOL(pxaopb_bus);
EXPORT_SYMBOL(pxaebc_bus);
EXPORT_SYMBOL(pxasys_device_register);
EXPORT_SYMBOL(pxasys_driver_register);
EXPORT_SYMBOL(pxasys_device_unregister);
EXPORT_SYMBOL(pxasys_driver_unregister);
EXPORT_SYMBOL(pxaopb_device_register);
EXPORT_SYMBOL(pxaopb_driver_register);
EXPORT_SYMBOL(pxaopb_device_unregister);
EXPORT_SYMBOL(pxaopb_driver_unregister);
EXPORT_SYMBOL(pxaebc_device_register);
EXPORT_SYMBOL(pxaebc_driver_register);
EXPORT_SYMBOL(pxaebc_device_unregister);
EXPORT_SYMBOL(pxaebc_driver_unregister);
