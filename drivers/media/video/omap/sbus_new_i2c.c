/*
 * File: sbus_new_i2c.c
 *
 * Description:
 *   OMAP1610 I2C link to Camera Serial Bus.
 *
 * Author: Monta Vista Software, Inc.,
 * based on sbus_old_i2c.c:
 * * Author: TI, Inc.
 * * Created 2002, Copyright (C) 2002 Texas Instruments  All rights reserved.
 * Copyright (C) 2003-2004 MontaVista Software, Inc.  All rights reserved.
 *
 * 5/5/2004:
 *   Converted to a true I2c client driver, instead of using the
 *   I2c device interface <stevel@mvista.com>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */
#include <linux/config.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/system.h>
#define MODULE_NAME "cami2c"
#include "common.h"
#include "camif.h"
#include <linux/i2c.h>

static struct camera_serial_bus *this;

static struct i2c_client omap_i2c_client;
static struct i2c_driver omap_i2c_driver;

static int 
omap_i2c_probe_adapter(struct i2c_adapter *adap)
{
	struct i2c_client *client = &omap_i2c_client;
	int err;

	if (client->adapter)
		return -EBUSY;	/* our client is already attached */

	strcpy(client->name, MODULE_NAME);
	client->id = omap_i2c_driver.id;
	client->flags = I2C_CLIENT_ALLOW_USE;
	client->driver = &omap_i2c_driver;
	client->adapter = adap;

	err = i2c_attach_client(client);
	if (err) {
		client->adapter = NULL;
		return err;
	}

	return 0;
}

static int 
omap_i2c_detach_client(struct i2c_client *client)
{
	int err;

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	err = i2c_detach_client(client);
	client->adapter = NULL;

	return err;
}

static int
i2c_configure(void)
{
	struct i2c_driver *driver = &omap_i2c_driver;
	struct i2c_client *client = &omap_i2c_client;
	int err;
	
	this = &camera_sbus_new_i2c;
	
	memset(driver, 0, sizeof(*driver));
	memset(client, 0, sizeof(*client));
	
	strcpy(driver->name, MODULE_NAME);
	driver->id = I2C_DRIVERID_EXP0;
	driver->flags = I2C_DF_NOTIFY;
	driver->attach_adapter = omap_i2c_probe_adapter;
	driver->detach_client = omap_i2c_detach_client;

	err = i2c_add_driver(driver);
	if (err) {
		err("Failed to register OMAP Camera I2C client.\n");
		return err;
	}
	
	if (!client->adapter)
		warn("Failed to detect OMAP camera.\n");
	
	return 0;
}

static void
i2c_close(void)
{
	struct i2c_client *client = &omap_i2c_client;
	if (client->adapter)
		i2c_del_driver(&omap_i2c_driver);
}

static int
i2c_set_devid(int id)
{
	this->dev_id = id;
	omap_i2c_client.addr = id >> 1;
	return 0;
}

static int
i2c_read(u8 subaddr, u8 * buf, int len)
{
	struct i2c_client *client = &omap_i2c_client;
	int i;

	if (!client->adapter)
		return -ENODEV;

	for (i = 0; i < len; i++) {
		i2c_master_send(client, &subaddr, 1);
		udelay(100);
		i2c_master_recv(client, &buf[i], 1);
		udelay(100);
		subaddr++;
	}

	return 0;
}

static int
i2c_write(u8 subaddr, u8 * buf, int len)
{
	struct i2c_client *client = &omap_i2c_client;
	int i;
	char tmpbuf[2];

	if (!client->adapter)
		return -ENODEV;

	for (i = 0; i < len; i++) {
		tmpbuf[0] = subaddr++;	/*register number */
		tmpbuf[1] = buf[i];	/*register data */
		i2c_master_send(client, &tmpbuf[0], 2);
		udelay(100);
	}

	return 0;
}

#define WRITE_VERIFY_RETRIES 5

static int
i2c_write_verify(u8 subaddr, u8 val)
{
	struct i2c_client *client = &omap_i2c_client;
	int count = 0;
	u8 readval;

	if (!client->adapter)
		return -ENODEV;

	do {
		i2c_write(subaddr, &val, 1);
		readval = ~val;
		i2c_read(subaddr, &readval, 1);
		if (readval != val)
			dbg(": failed to verify data, count %d, "
			    "subaddr %x, written %x, read %x\n",
			    count, subaddr, val, readval);
	} while (readval != val && count++ < WRITE_VERIFY_RETRIES);

	if (readval != val && count >= WRITE_VERIFY_RETRIES) {
		err("%d attempts to write %02x to reg %02x failed\n",
		    WRITE_VERIFY_RETRIES, val, subaddr);
		return -ENXIO;
	}

	return 0;
}

struct camera_serial_bus camera_sbus_new_i2c = {
	init:         i2c_configure,
	cleanup:      i2c_close,
	set_devid:    i2c_set_devid,
	read:         i2c_read,
	write:        i2c_write,
	write_verify: i2c_write_verify,
};
