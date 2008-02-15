/*
 *  idc-adcm2650.c
 *
 *  ADCM 2650 Camera Module control via I2C bus driver.
 *
 *  Copyright (C) 2003, Intel Corporation
 *  Copyright (C) 2003, Montavista Software Inc.
 *
 *  Author: Intel Corporation Inc.
 *          MontaVista Software, Inc.
 *           source@mvista.com
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <asm/hardware.h>
#include <asm/types.h>

#include "i2c-adcm2650.h"

void i2c_adcm2650_inc_use (struct i2c_client *client);
void i2c_adcm2650_dec_use (struct i2c_client *client);
int i2c_adcm2650_attach_adapter(struct i2c_adapter *adapter);
extern int i2c_adapter_id(struct i2c_adapter *adap);
int i2c_adcm2650_detect_client(struct i2c_adapter *, int, unsigned short, int);
int i2c_adcm2650_cleanup(void);
int i2c_adcm2650_detach_client(struct i2c_client *client);

struct i2c_driver adcm2650_driver  = 
{
	name:				"adcm2650 driver",		/* name           */
	id:				I2C_DRIVERID_ADCM2650, 		/* id             */
	flags:				I2C_DF_NOTIFY,        		/* flags          */
	attach_adapter:			&i2c_adcm2650_attach_adapter,   /* attach_adapter */
	detach_client:			&i2c_adcm2650_detach_client,    /* detach_client  */
	command:			NULL,
	inc_use:			&i2c_adcm2650_inc_use, 
	dec_use:			&i2c_adcm2650_dec_use
};

extern  struct i2c_adapter *i2cdev_adaps[];
/* Unique ID allocation */
static int adcm2650_id = 0;
struct i2c_client *g_client = NULL;
static unsigned short normal_i2c[] = { 0x52,I2C_CLIENT_END };
static unsigned short normal_i2c_range[] = { I2C_CLIENT_END }; 
I2C_CLIENT_INSMOD;

/* 
 * This call returns a unique low identifier for each registered adapter,
 * or -1 if the adapter was not registered.
 */
void i2c_adcm2650_inc_use (struct i2c_client *client)
{
#ifdef MODULE
	MOD_INC_USE_COUNT;
#endif
}

void i2c_adcm2650_dec_use (struct i2c_client *client)
{
#ifdef MODULE
	MOD_DEC_USE_COUNT;
#endif
}

static int ChgBlockAddr(u8 block)
{

	struct adcm2650_data *p;
	int	res;
	p = g_client->data;
	
	down(&p->update_lock);
	/*	FIXME:	Shall we change the g_client->addr?	*/
	g_client->addr = PIPE_SLAVE_ADDR;	
	res = i2c_smbus_write_byte_data(g_client, BLOCK_SWITCH_CMD, block);
	p->blockaddr = block;
	up( &p->update_lock);
	return res;
}

int adcm2650_read(u16 addr, u16 *pvalue)
{
	int	res=0;
	struct 	adcm2650_data *p;
	u8	blockaddr = BLOCK(addr);
	u8	offset;

	if( g_client == NULL )	/*	No global client pointer?	*/
		return -1;

	p = g_client->data;

	//if( p->blockaddr != blockaddr )
	res = ChgBlockAddr( blockaddr);
	
	if( res !=0 )  {
		printk("Change block address failed. block = %2x \n", blockaddr);
		return -1;
	}
	offset = (addr << 1) & 0xff;
	res = i2c_smbus_read_word_data(g_client, offset);
	*pvalue = (u16)res;
	
	return res;
}	

int adcm2650_write(u16 addr, u16 value)
{
	int	res=0;
	struct adcm2650_data *p;
	u8	blockaddr = BLOCK(addr);
	u8 	offset;

	if( g_client == NULL )	/*	No global client pointer?	*/
		return -1;

	p = g_client->data;
	//if( p->blockaddr != blockaddr )
	res = ChgBlockAddr( blockaddr );
	
	if( res !=0 )  {
		printk("Change block address failed. block = %2x \n", blockaddr);
		return -1;
	}
	offset = (addr << 1) & 0xff;

	return i2c_smbus_write_word_data(g_client, offset, value );
}	


int i2c_adcm2650_read(struct i2c_client *client, u8 reg)
{
    return i2c_smbus_read_word_data(client,reg);
//  return i2c_smbus_read_byte_data(client,reg);
}

int i2c_adcm2650_write(struct i2c_client *client, u8 reg, u16 value)
{
    return i2c_smbus_write_word_data(client,reg,value);
}


int i2c_adcm2650_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap,&addr_data,i2c_adcm2650_detect_client);
}


int i2c_adcm2650_detect_client(struct i2c_adapter *adapter, int address, unsigned short flags, int kind)
{
    struct i2c_client *new_client;
    int err = 0;
	struct adcm2650_data *data;

    /* Check if the camera module has been already detected on other I2C bus */
    if (g_client != NULL)
    {
    	err = -ENXIO;
    	goto ERROR0;
    }

    /* Let's see whether this adapter can support what we need.
       Please substitute the things you need here!  */
	if ( !i2c_check_functionality(adapter,I2C_FUNC_SMBUS_WORD_DATA) ) {
		printk(KERN_INFO "Word op is not permited.\n");
		goto ERROR0;
	}

    /* OK. For now, we presume we have a valid client. We now create the
       client structure, even though we cannot fill it completely yet.
       But it allows us to access several i2c functions safely */
    
    /* Note that we reserve some space for adcm2650_data too. If you don't
       need it, remove it. We do it here to help to lessen memory
       fragmentation. */

    new_client=kmalloc(sizeof(struct i2c_client)+sizeof(struct adcm2650_data),
		  				GFP_KERNEL );

    if ( !new_client )  {
      err = -ENOMEM;
      goto ERROR0;
    }

	data = (struct adcm2650_data *) (new_client + 1);
	new_client->addr = address;	
	new_client->data = data;
    new_client->adapter = adapter;
    new_client->driver = &adcm2650_driver;
    new_client->flags = 0;

    g_client = new_client;

    /* Now, we do the remaining detection. If no `force' parameter is used. */

    /* First, the generic detection (if any), that is skipped if any force
       parameter was used. */

	if (kind <= 0) {
		/* The below is of course bogus */
		printk("I2C: Probe ADCM2650 chip..");
		if (i2c_adcm2650_read(new_client, REV) != 0x0600 ) {
			printk(KERN_WARNING "Failed.\n");		
			goto ERROR1;
		}		 
		else {
			if ( adcm2650_id == 0 )
				printk(" detected.\n");
		}
	}

	strcpy(new_client->name, "ADCM2650");
    new_client->id = adcm2650_id++; /* Automatically unique */
	data->valid = 0; /* Only if you use this field */
	init_MUTEX(&data->update_lock); /* Only if you use this field */

    /* Tell the i2c layer a new client has arrived */
    if ((err = i2c_attach_client(new_client)))
      goto ERROR3;

    /* This function can write default values to the client registers, if
       needed. */
	/*	adcm2650_init_client(new_client);	*/
    return 0;

    /* OK, this is not exactly good programming practice, usually. But it is
       very code-efficient in this case. */

ERROR3:
ERROR1:
      kfree(new_client);
      g_client = NULL;
ERROR0:
      return err;
}

int i2c_adcm2650_detach_client(struct i2c_client *client)
{
	int err;

    /* Try to detach the client from i2c space */
    if ((err = i2c_detach_client(client))) {
      printk("adcm2650.o: Client deregistration failed, client not detached.\n");
      return err;
    }

    kfree(client); /* Frees client data too, if allocated at the same time */
    g_client = NULL;
    return 0;
}

/* 	Keep track of how far we got in the initialization process. If several
	things have to initialized, and we fail halfway, only those things
	have to be cleaned up! */
static int __initdata adcm2650_initialized = 0;

int i2c_adcm2650_init(void)
{
	int res;

	printk("I2C: driver for device adcm2650.\n");
	if ( (res = i2c_add_driver(&adcm2650_driver)) ) {
		printk("adcm2650: Driver registration failed, module not inserted.\n");
		i2c_adcm2650_cleanup();
		return res;
	}
	adcm2650_initialized ++;
	return 0;
}

int i2c_adcm2650_cleanup(void)
{
	int res;

	if (adcm2650_initialized == 1) {
		if ((res = i2c_del_driver(&adcm2650_driver))) {
			printk("adcm2650: Driver registration failed, module not removed.\n");
			return res;
		}
		adcm2650_initialized --;
	}
	return 0;
}

EXPORT_SYMBOL(i2c_adcm2650_init);
EXPORT_SYMBOL(adcm2650_write);
EXPORT_SYMBOL(adcm2650_read);
EXPORT_SYMBOL(i2c_adcm2650_cleanup);
//module_init(i2c_adcm2650_init);
//module_exit(i2c_adcm2650_cleanup);
