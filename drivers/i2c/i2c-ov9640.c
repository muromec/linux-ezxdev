/*
 * Copyright (C) 2003 Motorola Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 *  History:
 *  created by w20158 for ezx platform
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <asm/hardware.h>
#include <asm/types.h>
#include <linux/delay.h>

#include "i2c-ov9640.h"

#define DEBUG 1
#define DPRINTK(fmt,args...)	do { if (DEBUG) printk("in function %s "fmt,__FUNCTION__,##args);} while(0)
extern int i2c_adapter_id(struct i2c_adapter *adap);

int  i2c_ov9640_cleanup(void);
void i2c_ov9640_inc_use (struct i2c_client *client);
void i2c_ov9640_dec_use (struct i2c_client *client);
int  i2c_ov9640_attach_adapter(struct i2c_adapter *adapter);
int  i2c_ov9640_detect_client(struct i2c_adapter *, int, unsigned short, int);
int  i2c_ov9640_detach_client(struct i2c_client *client);

struct i2c_driver ov9640_driver  = 
{
	name:			"ov9640 driver",	            /* name           */
	id:			I2C_DRIVERID_OV9640,         	    /* id             */
	flags:			I2C_DF_NOTIFY,        		    /* flags          */
	attach_adapter:		&i2c_ov9640_attach_adapter,       /* attach_adapter */
	detach_client:		&i2c_ov9640_detach_client,        /* detach_client  */
	command:		NULL,
	inc_use:		&i2c_ov9640_inc_use,
	dec_use:		&i2c_ov9640_dec_use
};

extern  struct i2c_adapter *i2cdev_adaps[];
/* Unique ID allocation */
static int ov9640_id = 0;
struct i2c_client *g_client = NULL;
static unsigned short normal_i2c[] = {OV9640_SLAVE_ADDR ,I2C_CLIENT_END };
static unsigned short normal_i2c_range[] = { I2C_CLIENT_END }; 
I2C_CLIENT_INSMOD;

/* 
 * This call returns a unique low identifier for each registered adapter,
 * or -1 if the adapter was not registered.
 */
void i2c_ov9640_inc_use (struct i2c_client *client)
{
	MOD_INC_USE_COUNT;
#ifdef MODULE
#endif
}

void i2c_ov9640_dec_use (struct i2c_client *client)
{
	MOD_DEC_USE_COUNT;
#ifdef MODULE
#endif
}

char ov9640_read(u8 addr, u8 *pvalue)
{
	int	res=0;
	char 	buf=0;
	struct i2c_msg msgs[2] = {
		{ 0, I2C_M_WR, 1, &addr },
		{ 0, I2C_M_RD, 1, &buf }};
	
	if( g_client == NULL )	
		return -1;
	i2c_ov9640_inc_use(g_client);
	msgs[0].addr=msgs[1].addr=g_client->addr;
	res=i2c_transfer(g_client->adapter,&msgs[0],1);
	if (res<=0) 
		goto out;
	res=i2c_transfer(g_client->adapter,&msgs[1],1);
	if (res<=0) 
		goto out;
	*pvalue = buf;
	i2c_ov9640_dec_use(g_client);
out:
	DPRINTK(KERN_INFO "In funtion %s addr:%x,value=%x\n", __FUNCTION__, addr,*pvalue);
	if (res<=0) DPRINTK("res = %d \n",res);
	return res;
}	

int ov9640_write(u8 addr, u8 value)
{
	int	res=0;
	if( g_client == NULL )
		return -1;
	/*
	char 	buf=0;
	struct i2c_msg msgs[2] = {
		{ 0, I2C_M_WR, 1, &addr },
		{ 0, I2C_M_WR, 1, &value }};
	msgs[0].addr=msgs[1].addr=g_client->addr;
	res=i2c_transfer(g_client->adapter,&msgs[0],1);
	if (res<=0) return res;
	res=i2c_transfer(g_client->adapter,&msgs[1],1);
	if (res<=0) return res;


	res=i2c_smbus_write_byte_data(g_client, addr, value );
	*/
	char 	buf[2]={addr,value};
	i2c_ov9640_inc_use(g_client);
	res = i2c_master_send(g_client, buf, 2);
	i2c_ov9640_dec_use(g_client);
	if (res >0) res =0;
	else res =-1;
	DPRINTK(KERN_INFO "In funtion %s addr:%x value:%xreturn %d \n", __FUNCTION__, addr,value,res);
	return res;
}	


int i2c_ov9640_read(struct i2c_client *client, u8 reg)
{
	unsigned char msgbuf=0;
	DPRINTK("in function %s\n",__FUNCTION__);
	i2c_master_recv(client,&msgbuf,1);
	return msgbuf;
	/*
	*/
//    return i2c_smbus_read_word_data(client,reg);
//  return i2c_smbus_read_byte_data(client,reg);
}

int i2c_ov9640_write(struct i2c_client *client, u8 reg, u16 value)
{
    return i2c_smbus_write_word_data(client,reg,value);
}


int i2c_ov9640_attach_adapter(struct i2c_adapter *adap)
{
	DPRINTK("In function %s.\n", __FUNCTION__);
	return i2c_probe(adap,&addr_data,i2c_ov9640_detect_client);
}


int i2c_ov9640_detect_client(struct i2c_adapter *adapter, int address, unsigned short flags, int kind)
{
    struct i2c_client *new_client;
    int err = 0;
    struct ov9640_data *data;
    
    /*check if */
    if(g_client != NULL) {
      err = -ENXIO;
      goto ERROR0;
    }
 

	DPRINTK(KERN_INFO "In funtion %s. address=0X%X\n", __FUNCTION__, address);
    /* Let's see whether this adapter can support what we need.
       Please substitute the things you need here!  */
	if ( !i2c_check_functionality(adapter,I2C_FUNC_SMBUS_WORD_DATA) ) {
		DPRINTK(KERN_INFO "Word op is not permited.\n");
		goto ERROR0;
	}

    /* OK. For now, we presume we have a valid client. We now create the
       client structure, even though we cannot fill it completely yet.
       But it allows us to access several i2c functions safely */
    
    /* Note that we reserve some space for ov9640_data too. If you don't
       need it, remove it. We do it here to help to lessen memory
       fragmentation. */

    new_client=kmalloc(sizeof(struct i2c_client)+sizeof(struct ov9640_data),
		  				GFP_KERNEL );

    if ( !new_client )  {
      err = -ENOMEM;
      goto ERROR0;
    }

	data = (struct ov9640_data *) (new_client + 1);

	new_client->addr = address;	
	new_client->data = data;
	new_client->adapter = adapter;
	new_client->driver = &ov9640_driver;
	new_client->flags = 0;

    g_client = new_client;

    /* Now, we do the remaining detection. If no `force' parameter is used. */

    /* First, the generic detection (if any), that is skipped if any force
       parameter was used. */

	if (kind <= 0) {
                 char res = -1;
		mdelay(2000);
		 ov9640_read(REV,&res);
		/* The below is of course bogus */
		DPRINTK("I2C: Probe ov9640 chip..addr=0x%x, REV=%d, res=0x%x\n", address, REV, res);
                /*ov9640 chip id is 0x9648
                 if(res != OV9640_CHIP_ID) {
			DPRINTK(KERN_WARNING "Failed.product id =%d \n",res);
			goto ERROR1;
		 }		 
		else {
                       DPRINTK("OV9640 chip id is 0X%04X\n", OV9640_CHIP_ID);
			if ( ov9640_id == 0 )
				DPRINTK(" detected.\n");
		}*/
	}

	strcpy(new_client->name, "ov9640");
    /* Automatically unique */
    new_client->id = ov9640_id++; 

    /* Only if you use this field */
	data->valid = 0; 

    /* Only if you use this field */
	init_MUTEX(&data->update_lock); 

    /* Tell the i2c layer a new client has arrived */
    if ((err = i2c_attach_client(new_client)))
      goto ERROR3;

    /* This function can write default values to the client registers, if
       needed. */
	/*	ov9640_init_client(new_client);	*/
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

int i2c_ov9640_detach_client(struct i2c_client *client)
{
	int err;

    /* Try to detach the client from i2c space */
    if ((err = i2c_detach_client(client))) {
      DPRINTK("ov9640.o: Client deregistration failed, client not detached.\n");
      return err;
    }

    kfree(client); /* Frees client data too, if allocated at the same time */
    g_client = NULL;
    return 0;
}

/* 	Keep track of how far we got in the initialization process. If several
	things have to initialized, and we fail halfway, only those things
	have to be cleaned up! */
static int ov9640_initialized = 0;

int i2c_ov9640_init(void)
{
	int res;

	if (ov9640_initialized) 
		return 0;
	DPRINTK("I2C: driver for device ov9640.\n");
	if ( (res = i2c_add_driver(&ov9640_driver)) ) {
		DPRINTK("ov9640: Driver registration failed, module not inserted.\n");
		i2c_ov9640_cleanup();
		return res;
	}
	ov9640_initialized ++;
        if(g_client != NULL)
   	   DPRINTK("I2C: driver for device %s registed!.\n", g_client->name);
        else 
           DPRINTK("I2C: driver for device unregisted!.\n");
	return 0;
}

int i2c_ov9640_cleanup(void)
{
	int res;

	if (ov9640_initialized == 1) {
		if ((res = i2c_del_driver(&ov9640_driver))) {
			DPRINTK("ov9640: Driver registration failed, module not removed.\n");
			return res;
		}
		ov9640_initialized --;
	}
	return 0;
}

EXPORT_SYMBOL(i2c_ov9640_init);
EXPORT_SYMBOL(ov9640_write);
EXPORT_SYMBOL(ov9640_read);
EXPORT_SYMBOL(i2c_ov9640_cleanup);
//module_init(i2c_ov9640_init);
//module_exit(i2c_ov9640_cleanup);
MODULE_LICENSE("GPL");

