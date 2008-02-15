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
 * History:
 *   Jay Jia(w20091),     Motorola , Feb,2004         Created
 */

#include <linux/tty.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/bitops.h>
#include <linux/param.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/pm.h>
#include <linux/apm_bios.h>
#include <asm/arch-pxa/irqs.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/videodev.h>
#include <linux/kdev_t.h>
#include <asm/semaphore.h>

#include <linux/sound.h>
#include <linux/soundcard.h>
#include <asm/uaccess.h>
#include <asm/irq.h>

#include "../sound/ezx-common.h"
#include "fmradio.h"

#define I2C_FMRADIO 0x81

static int fmradio_adapter_attach(struct i2c_adapter *adap);
static int fmradio_detach(struct i2c_client *client);
static int fmradio_client_register(struct i2c_client *client);
static int fmradio_client_unregister(struct i2c_client *client);
/* ----------------------------------------------------------------------- */
static struct i2c_driver driver = {
	name:            "fmradio driver",
    	id:              I2C_FMRADIO,
	flags:           I2C_DF_DUMMY,
    	attach_adapter:  fmradio_adapter_attach,        
    	detach_client:   fmradio_detach,
};

static struct i2c_adapter fmradio_adapter = {
        name:                   "fmradio adapter",
	id:                     I2C_FMRADIO,
        client_register:        fmradio_client_register,
        client_unregister:      fmradio_client_unregister,
};

static struct i2c_client client_template =
{
    	name:   "(unset)",        
    	adapter:&fmradio_adapter,
};

struct i2c_client *fmradio_client;

static int fmradio_open(struct inode *inode, struct file *file) 
{
	if( audioonflag & (DSP16_DEVICE|PHONE_DEVICE) ){
#ifdef EZX_OSS_DEBUG
		printk(EZXOSS_DEBUG "E680 open FM EBUSY because 0x%X device is using the sound hardware.\n",audioonflag );
#endif
		return -EBUSY;
	}


	poweron_mixer(FM_DEVICE);
	MOD_INC_USE_COUNT;
	
	return 0;
}

static int fmradio_close(struct inode * inode, struct file *file)
{
	unsigned int flags;

	MOD_DEC_USE_COUNT;
        //e680_boomer_path_tmp_mono_lineout (); //mute boomer
	SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_AUD_RX_AMPS_PGA_IN_SW);

        //write standby command
        unsigned char standby[5] = {0x2a, 0x48, 0xa1, 0xdf, 0x40};
        local_irq_save(flags);
        enable_irq(IRQ_I2C);
        i2c_master_send (fmradio_client, standby, 5);
        local_irq_restore(flags);
        shutdown_mixer(FM_DEVICE);

	return 0;
}


static int fmradio_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg) 
{
	return mixer_ioctl(inode, file, cmd, arg);
}

static ssize_t fmradio_read(struct file *file, char *buf, size_t count, loff_t *ptr)
{
	char *tmp;
	int ret;
    unsigned int flags;

	/* copy user space data to kernel space. */
	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;
    local_irq_save(flags);
    enable_irq(IRQ_I2C);
	ret = i2c_master_recv(fmradio_client,tmp,count);
    local_irq_restore(flags);
	if (ret >= 0)
		copy_to_user(buf,tmp,count);
	kfree(tmp);
	return ret;
}
static ssize_t fmradio_write(struct file *file, const char *buf, size_t count, loff_t *ptr)
{
	int ret;
	char *tmp;
// 	char data[4]={0x18,0x7d,0xbd,0xcd};
//        char freq[5]={0x2e,0x56,0x41,0x11,0x40};
//        unsigned char s[5]={0, 0, 0, 0, 0};

//        mixer_write(data,4);
//      	i2c_master_send(fmradio_client,freq,5);
//      printk("fmradio test code\n");
//        mdelay(300);
//      i2c_master_recv(fmradio_client, s, 5);
//      printk("s0=%02x   s1=%02x   s2=%02x    s3=%02x   s4=%02x \n",s[0],s[1],s[2],s[3],s[4]);

    unsigned int flags;      	
	/* copy user space data to kernel space. */
	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;
	if (copy_from_user(tmp,buf,count)) {
		kfree(tmp);
		return -EFAULT;
	}
    local_irq_save(flags);
    enable_irq(IRQ_I2C);
	ret = i2c_master_send(fmradio_client,tmp,count);
    local_irq_restore(flags);
	kfree(tmp);
	return ret;	
}

static int fmradio_client_register(struct i2c_client *client)
{
	
	return 0;
}

static int fmradio_client_unregister(struct i2c_client *client)
{
	
	return 0;	
}
/* ----------------------------------------------------------------------- */
static struct file_operations fmradio_fops = {
	read:           fmradio_read, 
        write:          fmradio_write,
    	ioctl:          fmradio_ioctl, 
    	open:           fmradio_open, 
    	release:        fmradio_close,
};

static struct miscdevice fmradio_misc_device = {
    	FMRADIO_MINOR,
    	FMRADIO_NAME,
    	&fmradio_fops,
};

/* ----------------------------------------------------------------------- */

static int fmradio_adapter_attach(struct i2c_adapter *adap)
{
	if(! (fmradio_client = kmalloc(sizeof(struct i2c_client),GFP_KERNEL)))
		return -ENOMEM;
	memcpy(fmradio_client,&client_template,sizeof(struct i2c_client));
	fmradio_client->adapter = adap;
        
	fmradio_client->addr = 0x60;
	
	printk("adapter %s\n",adap->name);
	i2c_attach_client(fmradio_client);

	return 0;
}

static int fmradio_detach(struct i2c_client *client)
{	
	i2c_detach_client(fmradio_client);
	return 0;
}
/* ----------------------------------------------------------------------- */

static int fmradio_init_module(void)
{
	int res;
	
	res = i2c_add_driver(&driver);
	if( res < 0 )
	{
		printk("error in add fmradio i2c driver\n");
		return res;
	}
	
	if (misc_register (&fmradio_misc_device))
    	{
       		printk(KERN_ERR "Couldn't register fmradio driver\n");
       		return -EIO;
    	}
	return 0;
}

static void fmradio_cleanup_module(void)
{	
	i2c_del_driver(&driver);
	misc_deregister(&fmradio_misc_device);
}

module_init(fmradio_init_module);
module_exit(fmradio_cleanup_module);
MODULE_AUTHOR("Jay Jia");
MODULE_LICENSE("GPL");
