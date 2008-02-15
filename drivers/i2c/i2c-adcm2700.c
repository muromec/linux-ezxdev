/*================================================================================
 *
 *   Copyright (C) 2003,2004  Motorola Inc.
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
==================================================================================*/
/*
  Module Name:  i2c-adcm2700.c

  General Description: Camera module adcm2700 I2C interface source file

==================================================================================
  Revision History:
                            Modification     Tracking
  Author                 Date          Number     Description of Changes
  ----------------   ------------    ----------   -------------------------
  Wangfei(w20239)     12/16/2003      LIBdd35749   Created
  wangfei(w20239)     02/26/2004      LIBdd81055   New chip id support
                                                                                                 
  Portability: Indicate if this module is portable to other compilers or 
  platforms. If not, indicate specific reasons why is it not portable.

==================================================================================
                                 INCLUDE FILES
================================================================================*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <asm/hardware.h>
#include <asm/types.h>
#include <linux/delay.h>
#include "i2c-pxa.h"
#include "i2c-adcm2700.h"

extern int  i2c_adcm2700_cleanup(void);
extern void i2c_adcm2700_inc_use (struct i2c_client *client);
extern void i2c_adcm2700_dec_use (struct i2c_client *client);
extern int  i2c_adcm2700_attach_adapter(struct i2c_adapter *adapter);
extern int  i2c_adcm2700_detect_client(struct i2c_adapter *, int, unsigned short, int);
extern int  i2c_adcm2700_detach_client(struct i2c_client *client);

struct i2c_driver adcm2700_driver  = 
{
	name:			"adcm2700 driver",	            /* name           */
	id:			I2C_DRIVERID_ADCM2700,         	    /* id             */
	flags:			I2C_DF_NOTIFY,        		    /* flags          */
	attach_adapter:		&i2c_adcm2700_attach_adapter,       /* attach_adapter */
	detach_client:		&i2c_adcm2700_detach_client,        /* detach_client  */
	command:		NULL,
	inc_use:		&i2c_adcm2700_inc_use,
	dec_use:		&i2c_adcm2700_dec_use
};

extern  struct i2c_adapter *i2cdev_adaps[];

/* Unique ID allocation */
static int adcm2700_id = 0;   
static struct i2c_client *g_client = NULL;
static unsigned short normal_i2c[] = {ADCM2700_I2C_ADDR, I2C_CLIENT_END};
static unsigned short normal_i2c_range[] = { I2C_CLIENT_END }; 


I2C_CLIENT_INSMOD;

/* 
 * This call returns a unique low identifier for each registered adapter,
 * or -1 if the adapter was not registered.
 */
void i2c_adcm2700_inc_use (struct i2c_client *client)
{
#ifdef MODULE
	MOD_INC_USE_COUNT;
#endif
}

void i2c_adcm2700_dec_use (struct i2c_client *client)
{
#ifdef MODULE
	MOD_DEC_USE_COUNT;
#endif
}

static int ChgBlockAddr(u8 block)
{
	struct adcm2700_data *p = g_client->data;
	int	res;
	
	if(p->blockaddr == block)
	{
 	   return 0;
	}
	
	down(&p->update_lock);
	res = i2c_smbus_write_byte_data(g_client, BLOCK_SWITCH_CMD, block);
	p->blockaddr = block;
	up( &p->update_lock);
	
	return res;
}

int adcm2700_read(u16 addr, u16 *pvalue)
{
	u8	blockaddr = BLOCK(addr);
	u8	offset    = OFFSET_R(addr);

	if(g_client == NULL)
	{
	   return -1;
	}

	if(ChgBlockAddr(blockaddr) != 0)  
	{
	    dbg_print("Change block address failed. block = %2x", blockaddr);
	    return -1;
	}
	
	return  (int)(*pvalue = (u16)i2c_smbus_read_word_data(g_client, offset));
}	
int adcm2700_write_byte(u16 addr, u8 value)
{
  	u8	blockaddr = BLOCK(addr);
	u8 	offset    = OFFSET(addr);;

	if(g_client == NULL)
	{
	   return -1;
	}
		
	if(ChgBlockAddr(blockaddr) != 0)  
	{
	   dbg_print("Change block address failed. block = %2x", blockaddr);
	   return -1;
	}
	
	return i2c_smbus_write_byte_data(g_client, offset, value);

}
int adcm2700_read_byte(u16 addr, u8 * pvalue)
{
  	u8	blockaddr = BLOCK(addr);
	u8	offset    = OFFSET_R(addr);

	if(g_client == NULL)
	{
	   return -1;
	}

	if(ChgBlockAddr(blockaddr) != 0)  
	{
	    dbg_print("Change block address failed. block = %2x", blockaddr);
	    return -1;
	}
	
	return  (int)(*pvalue = (u8)i2c_smbus_read_byte_data(g_client, offset));
 }
int adcm2700_write(u16 addr, u16 value)
{
	u8	blockaddr = BLOCK(addr);
	u8 	offset    = OFFSET(addr);;

	if(g_client == NULL)
	{
	   return -1;
	}
		
	if(ChgBlockAddr(blockaddr) != 0)  
	{
	   dbg_print("Change block address failed. block = %2x", blockaddr);
	   return -1;
	}
	
	return i2c_smbus_write_word_data(g_client, offset, value);
}	

int i2c_adcm2700_attach_adapter(struct i2c_adapter *adap)
{
        dbg_print("");
	return i2c_probe(adap,&addr_data,i2c_adcm2700_detect_client);
}


int i2c_adcm2700_detect_client(struct i2c_adapter *adapter, int address, unsigned short flags, int kind)
{
    struct i2c_client *new_client;
    int err = 0;
    
    if(g_client != NULL || address != ADCM2700_I2C_ADDR) 
    {
      return  -ENXIO;
    }
    /* Let's see whether this adapter can support what we need.
       Please substitute the things you need here!  */
    if(!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) 
    {
	dbg_print("Word op is not permited.\n");
	return -ENXIO;
    }

    /* OK. For now, we presume we have a valid client. We now create the
       client structure, even though we cannot fill it completely yet.
       But it allows us to access several i2c functions safely */
    
    /* Note that we reserve some space for adcm2700_data too. If you don't
       need it, remove it. We do it here to help to lessen memory
       fragmentation. */
    new_client=kmalloc(sizeof(struct i2c_client)+sizeof(struct adcm2700_data), GFP_KERNEL );

    if(new_client == NULL)  
    {
      return  -ENOMEM;
    }

    new_client->addr    = address;	
    new_client->data    = (struct adcm2700_data *) (new_client + 1);
    new_client->adapter = adapter;
    new_client->driver  = &adcm2700_driver;
    new_client->flags   = 0;

    g_client = new_client;
    
    /* Now, we do the remaining detection. If no `force' parameter is used. */
    /* First, the generic detection (if any), that is skipped if any force
       parameter was used. */
    if(kind <= 0) 
    {
         /* The below is of course bogus */
         int chipId = i2c_smbus_read_word_data(new_client, OFFSET_R(REV_ID));
         if(chipId != ADCM2700_CHIP_ID && chipId != ADCM2700_CHIP_ID_NEW) 
	     {
	      dbg_print(KERN_WARNING "Failed.\n");		
 	      goto ERROR1;
	     }		 
         else if(adcm2700_id == 0)
	     {
 	       dbg_print("Adcm2700 detected.CHIP ID is 0x%x\n", chipId);
         }
    }

    strcpy(new_client->name, "adcm2700");
    
    /* Automatically unique */
    new_client->id = adcm2700_id++; 

    /* Only if you use this field */
    ((struct adcm2700_data *)new_client->data)->valid = 0; 

    /* Only if you use this field */
    init_MUTEX(&(((struct adcm2700_data *)new_client->data)->update_lock)); 

    /* Tell the i2c layer a new client has arrived */
    if((err = i2c_attach_client(new_client)))
    {
       goto ERROR1;
    }

    /* This function can write default values to the client registers, if
       needed. */
    dbg_print("Success!!");
    
    return 0;

    /* OK, this is not exactly good programming practice, usually. But it is
       very code-efficient in this case. */

ERROR1:
      kfree(new_client);
      g_client = NULL;
      return err;
}

int i2c_adcm2700_detach_client(struct i2c_client *client)
{
    int err;

    /* Try to detach the client from i2c space */
    if((err = i2c_detach_client(client)))
    {
      dbg_print("adcm2700.o: Client deregistration failed, client not detached.\n");
      return err;
    }

    kfree(client); /* Frees client data too, if allocated at the same time */
    g_client = NULL;
    return 0;
}

/* 	Keep track of how far we got in the initialization process. If several
	things have to initialized, and we fail halfway, only those things
	have to be cleaned up! */
static int __initdata adcm2700_initialized = 0;

int i2c_adcm2700_init(void)
{
	dbg_print("I2C: driver for device adcm2700.");
	
	if(i2c_add_driver(&adcm2700_driver)) 
	{
	    dbg_print("adcm2700: Driver registration failed, module not inserted.");
 	    i2c_adcm2700_cleanup();
	    return 0;
	}
	
	adcm2700_initialized ++;
	
        if(g_client != NULL)
	{
	  dbg_print("I2C: driver for device %s registed!.", g_client->name);
	}
        else 
	{
           dbg_print("I2C: driver for device unregisted!.");
	}
	   
      	return 0;
}

int i2c_adcm2700_cleanup(void)
{
	int res;

	if(adcm2700_initialized == 1) 
	{
  	    if((res = i2c_del_driver(&adcm2700_driver))) 
	    {
		dbg_print("adcm2700: Driver registration failed, module not removed.");
		return res;
	    }
 	    adcm2700_initialized --;
	}
	
	return 0;
}

MODULE_LICENSE("GPL");
EXPORT_SYMBOL(i2c_adcm2700_init);
EXPORT_SYMBOL(adcm2700_write);
EXPORT_SYMBOL(adcm2700_write_byte);
EXPORT_SYMBOL(adcm2700_read_byte);
EXPORT_SYMBOL(adcm2700_read);
EXPORT_SYMBOL(i2c_adcm2700_cleanup);
//module_init(i2c_adcm2700_init);
//module_exit(i2c_adcm2700_cleanup);


