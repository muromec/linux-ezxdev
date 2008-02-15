/*
 * drivers/media/video/mx2ads/ov9640_sabus.c
 *
 * Description:
 *   MX2ADS I2C link to ov9640 camera Serial Bus. Theoreticaly this driver can
 *   be merged with seria bus driver for misoc camera, but due HW bug in ov9640
 *   it is not necessary. This driver handles request only from ov9640. To
 *   serve other devices it should be advanced
 *
 * Author: Monta Vista Software, Inc., <source@mvista.com>
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/config.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/system.h>

#define MODULE_NAME "ov9640cami2c"

#include "common.h"
#include "camif.h"
#include <linux/i2c.h>

static struct camera_serial_bus *this;

static char __initstate_i2c;

static unsigned short __normal_i2c_addr[] = { (CAMERA_OV9640_DEV_ID >> 1), I2C_CLIENT_END};
static unsigned short __ignore_i2c_addr[] = { I2C_CLIENT_END };

static struct i2c_client_address_data __i2c_addr_data = {
	.normal_i2c = __normal_i2c_addr,
	.normal_i2c_range = __ignore_i2c_addr,
	.probe = __ignore_i2c_addr,
	.probe_range = __ignore_i2c_addr,
	.ignore = __ignore_i2c_addr,
	.ignore_range = __ignore_i2c_addr,
	.force = __ignore_i2c_addr
};

static int __i2c_probe(struct i2c_adapter *adap);
static int __i2c_detach(struct i2c_client *client);
static int __i2c_command(struct i2c_client *client, unsigned int cmd,
			   void *arg);

static struct i2c_client *__i2c_client;

static struct i2c_driver __i2c_driver = {
	.name = "OV9640 CAM",
	.id = I2C_DRIVERID_EXP0,	/* Fake Id */
	.flags = I2C_DF_NOTIFY,
	.attach_adapter = __i2c_probe,
	.detach_client = __i2c_detach,
	.command = __i2c_command
};

static int
__i2c_attach(struct i2c_adapter *adap, int addr, unsigned short flags,
	       int kind)
{
	struct i2c_client *c;

	c = (struct i2c_client *) kmalloc(sizeof (*c), GFP_KERNEL);

	if (!c)
		return -ENOMEM;

	strcpy(c->name, "OV9640 CAM");
	c->id = __i2c_driver.id;
	c->flags = 0;
	c->addr = addr;
	c->adapter = adap;
	c->driver = &__i2c_driver;
	c->data = NULL;

	__i2c_client = c;

	return i2c_attach_client(c);
}

static int
__i2c_probe(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &__i2c_addr_data, __i2c_attach);
}

static int
__i2c_detach(struct i2c_client *client)
{
	i2c_detach_client(client);
	kfree(__i2c_client);
	return 0;
}

/* No commands defined */
static int
__i2c_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	return 0;
}

static void
i2c_close(void)
{
	if (__initstate_i2c) {
		i2c_del_driver(&__i2c_driver);
		__initstate_i2c = 0;
	}
}

static int
i2c_configure(void)
{
	int tmp;
	__initstate_i2c = 0;
	this = &camera_mx2ads_ov9640_i2c;
        /*
 * Special MDS register (controls CNTL and LEDs)
 */
        uint16_t reg_val;

        CSI_REG_READ(EXP_IO, reg_val);

        /*
         * Make standby low (disable standby)
         */
        reg_val &= ~EXP_IO_CSI_CTL0;

        CSI_REG_WRITE(EXP_IO, reg_val);

        udelay(20);

        /*
         * Make reset high (reset sensor)
         */
        reg_val |= EXP_IO_CSI_CTL1;

        CSI_REG_WRITE(EXP_IO, reg_val);

        udelay(200);

        /*
         * Make reset low
         */
        reg_val &= ~EXP_IO_CSI_CTL1;

        CSI_REG_WRITE(EXP_IO, reg_val);

        mdelay(15);

	tmp = i2c_add_driver(&__i2c_driver);
	if (tmp < 0) {
		err("cannot initialize I2C\n");
		i2c_close();
		return tmp;
	}
	__initstate_i2c = 1;

	if (!__i2c_client) {
		i2c_close();
		return -ENODEV;
	}

	return 0;
}

static int
i2c_set_devid(int id)
{
	this->dev_id = id >> 1;
	return 0;
}

static int
i2c_read(u8 subaddr, u8 * buf, int len)
{
	struct i2c_msg msg;

	msg.addr = this->dev_id;
	msg.flags = I2C_M_WR | I2C_M_NOSTART;
	msg.len = 1;
	msg.buf = &subaddr;

	if (i2c_transfer(__i2c_client->adapter, &msg, 1) != 1) return -EIO;
        
	/*ov9640 doesn't like repeat start, need to send a separate message*/
	msg.addr = this->dev_id;
	msg.flags = I2C_M_RD | I2C_M_NOSTART;
	msg.len = len;
	msg.buf = buf;

	if (i2c_transfer(__i2c_client->adapter, &msg, 1) != 1) return -EIO;

        return 0;
}

static int
i2c_write(u8 subaddr, u8 * buf, int len)
{
	struct i2c_msg msg;
        u8 wbuf[2];
	wbuf[0] = subaddr;
	wbuf[1] = buf[0];

	msg.addr = this->dev_id;
	msg.flags = I2C_M_WR | I2C_M_NOSTART;
	msg.len = 2;
	msg.buf = wbuf;

	if (i2c_transfer(__i2c_client->adapter, &msg, 1) != 1) return -EIO;
	return 0;
}

#define WRITE_VERIFY_RETRIES 5

static int
i2c_write_verify(u8 subaddr, u8 val)
{
	int count = 0;
	u8 readval;

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

struct camera_serial_bus camera_mx2ads_ov9640_i2c = {
	init:         i2c_configure,
	cleanup:      i2c_close,
	set_devid:    i2c_set_devid,
	read:         i2c_read,
	write:        i2c_write,
	write_verify: i2c_write_verify,
};
