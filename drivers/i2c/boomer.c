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
 *  History:
 *  Jay Jia(w20091),   Motorola  Nov 25,2003,            Created
 *  Jin Lihong(w20076),Motorola  Jan 13,2004,LIBdd68327  Make the e680 louder speaker work.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/videodev.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <asm/semaphore.h>

#include <linux/sound.h>
#include <linux/soundcard.h>
#include <asm/uaccess.h>
#include <asm/irq.h>


#define TESTCODE
#undef  TESTCODE
#define I2C_MIXER 0x80

static int mixer_adapter_attach(struct i2c_adapter *adap);
static int mixer_detach(struct i2c_client *client);
static int mixer_client_register(struct i2c_client *client);
static int mixer_client_unregister(struct i2c_client *client);
extern struct i2c_client *fmradio_client;
/* ----------------------------------------------------------------------- */
static struct i2c_driver driver = {
	name:            "mixer driver",
    id:              I2C_MIXER,
	flags:           I2C_DF_DUMMY,
    attach_adapter:  mixer_adapter_attach,        
    detach_client:   mixer_detach,
};

static struct i2c_adapter mixer_adapter = {
        name:                   "Mixer adapter",
        id:                     I2C_MIXER,
        client_register:        mixer_client_register,
        client_unregister:      mixer_client_unregister,
};

static struct i2c_client client_template =
{
    name:   "(unset)",        
    adapter:&mixer_adapter,
};

struct i2c_client *mixer_client;
unsigned int mixer_minor;
	
int mixer_write(const char *buf, size_t count);
int mixer_open(void)
{
	MOD_INC_USE_COUNT;
	

#ifdef TESTCODE
	char data[4]={0x18,0x7d,0xbd,0xcd};
        char freq[5]={0x2e,0x56,0x41,0x11,0x40};
	unsigned char s[5]={0, 0, 0, 0, 0};
	
	mixer_write(data,4);
//	i2c_master_send(fmradio_client,freq,5);	
//	printk("fmradio test code\n");
//        mdelay(300);		
//	i2c_master_recv(fmradio_client, s, 5);
//	printk("s0=%02x   s1=%02x   s2=%02x    s3=%02x   s4=%02x \n",s[0],s[1],s[2],s[3],s[4]);
#endif


	return 0;
}

int mixer_release(void)
{
	MOD_DEC_USE_COUNT;
	return 0;
}

int mixer_write(const char *buf, size_t count)
{
	int ret;
    unsigned int flags;	

    mixer_open();
    local_irq_save(flags);
    enable_irq(IRQ_I2C);
	ret = i2c_master_send(mixer_client, buf, count);
    local_irq_restore(flags);
//	printk("i2c-%d writing %d bytes.\n", mixer_minor, ret);
    mixer_release();
	return ret;
}

static int mixer_client_register(struct i2c_client *client)
{
	
	return 0;
}

static int mixer_client_unregister(struct i2c_client *client)
{
	
	return 0;	
}
/* ----------------------------------------------------------------------- */

static int mixer_adapter_attach(struct i2c_adapter *adap)
{
	if(! (mixer_client = kmalloc(sizeof(struct i2c_client),GFP_KERNEL)))
		return -ENOMEM;
	memcpy(mixer_client,&client_template,sizeof(struct i2c_client));
	mixer_client->adapter = adap;
        
	mixer_client->addr = 0x7c;
	
	printk("adapter %s\n",adap->name);
	i2c_attach_client(mixer_client);
#ifdef TESTCODE
	mixer_open();
#endif
	return 0;
}

static int mixer_detach(struct i2c_client *client)
{	
	i2c_detach_client(mixer_client);
	return 0;
}
/* ----------------------------------------------------------------------- */

static int mixer_init_module(void)
{
	int res;
	
	res = i2c_add_driver(&driver);
	if( res < 0 )
	{
		printk("error in add i2c driver\n");
		return res;
	}
	return 0;
}

static void mixer_cleanup_module(void)
{	
	i2c_del_driver(&driver);
}

module_init(mixer_init_module);
module_exit(mixer_cleanup_module);
MODULE_AUTHOR("Jay Jia");
MODULE_LICENSE("GPL");
