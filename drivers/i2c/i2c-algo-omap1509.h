/* ------------------------------------------------------------------------- */
/* i2c-algo-omap1509.h i2c driver algorithms for OMAP1509 adapters           */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 1995-97 Simon G. Vogl
                   1998-99 Hans Berglund

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */

/* With some changes from Kyösti Mälkki <kmalkki@cc.hut.fi> and even
   Frodo Looijaard <frodol@dds.nl> */

/* Modifications by MontaVista Software, 2001
   Changes made to support the ITE IIC peripheral */

/* Modifications by RidgeRun, Inc.
   Changes made to support the OMAP1509 controller */


#ifndef I2C_ALGO_OMAP_H
#define I2C_ALGO_OMAP_H 

#include <linux/i2c.h>

/* Example of a sequential read request:
	struct i2c_iic_msg s_msg; 

	s_msg.addr=device_address;
	s_msg.len=length;
	s_msg.buf=buffer;
	s_msg.waddr=word_address;
	ioctl(file, I2C_SREAD, &s_msg);
 */
#define I2C_SREAD	0x780	/* SREAD ioctl command */

struct i2c_omap1509_msg {
	__u16 addr;	/* device address */
	__u16 waddr;	/* word address */
	short len;	/* msg length */
	char *buf;	/* pointer to msg data */
};

struct i2c_algo_omap1509_data {
	void *data;		/* private data for low-level routines	*/
	void (*setomap1509) (void *data, int ctl, u16 val);
	u16  (*getomap1509) (void *data, int ctl);
	int  (*getown) (void *data);
	int  (*getclock) (void *data);
	void (*waitforpin) (void);     

	/* local settings */
	int udelay;
	int mdelay;
	int timeout;
};

int i2c_omap1509_add_bus(struct i2c_adapter *);
int i2c_omap1509_del_bus(struct i2c_adapter *);

#endif /* I2C_ALGO_OMAP_H */
