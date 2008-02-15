/*
 * Copyright (C) 2003,2005 Motorola Inc.
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
 *  Nov 25, 2003     Created for Moto e680, Jay Jia
 *  May 20, 2005     Ported to platform and integrated into the whole audio system, Jin Lihong
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
#include <asm/dma.h>

#include <linux/sound.h>
#include <linux/soundcard.h>
#include <asm/uaccess.h>
#include <asm/irq.h>

#include "ezx-audio.h"
#include "ezx-common.h"
#include "fmradio.h"


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
    	name:     "(unset)",
    	adapter:  &fmradio_adapter
};

struct i2c_client *fmradio_client;

static int fmradio_open(struct inode *inode, struct file *file) 
{
        int res;

        res = i2c_add_driver(&driver);
        if( res < 0 ){
                printk(EZXOSS_DEBUG "error in add fmradio i2c driver\n");
                return res;
        }

	if( audioonflag & (~DSP_DEVICE) ){
		AUDPRINTk1(EZXOSS_DEBUG "open FM EBUSY because 0x%X device is using the sound hardware.\n",audioonflag );
	        i2c_del_driver(&driver);

		return -EBUSY;
	}

	poweron_mixer(FM_DEVICE);
	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, PGA_IN_SW_INDEX, PCAP_BIT_SET_VALUE, PGA_IN_SW_NUM_BITS );

        AUDPRINTk1(EZXOSS_DEBUG "codec_output_path=%d\n", codec_output_path);
	set_audio_output(codec_output_path);
        print_pcap_audio_reg_vals();

	MOD_INC_USE_COUNT;
	
	return 0;
}


static int fmradio_close(struct inode * inode, struct file *file)
{
	MOD_DEC_USE_COUNT;

	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, PGA_IN_SW_INDEX, PCAP_BIT_CLEAN_VALUE, PGA_IN_SW_NUM_BITS );

        /* write standby command
	unsigned int flags;
        unsigned char standby[5] = {0x2a, 0x48, 0xa1, 0xdf, 0x40};
        local_irq_save(flags);
        enable_irq(IRQ_I2C);
        i2c_master_send (fmradio_client, standby, 5);
        local_irq_restore(flags); */
        shutdown_mixer(FM_DEVICE);

        i2c_del_driver(&driver);
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

	AUDPRINTk1(EZXOSS_DEBUG "fmradio want to read %d bytes \n", count);

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

	AUDPRINTk1(EZXOSS_DEBUG "fmradio read i2c return value:%d \n", ret);
	return ret;
}


static ssize_t fmradio_write(struct file *file, const char *buf, size_t count, loff_t *ptr)
{
	int ret;
	char *tmp;
	unsigned int flags;  
    	
	AUDPRINTk1(EZXOSS_DEBUG "fmradio write %d bytes \n", count);

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

	AUDPRINTk1(EZXOSS_DEBUG "fmradio write i2c return value:%d \n", ret);
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
static struct file_operations fm_audio_fops = {
	read:           fmradio_read,
        write:          fmradio_write,
    	ioctl:          fmradio_ioctl,
    	open:           fmradio_open,
    	release:        fmradio_close
};


static audio_state_t fm_audio_state = {
	output_stream:		NULL,
	input_stream:		NULL,
	client_ioctl:		fmradio_ioctl,
	hw_init:		NULL,
	hw_shutdown:		NULL,
	sem:			__MUTEX_INITIALIZER(fm_audio_state.sem),
};


/* ----------------------------------------------------------------------- */
static int fmradio_adapter_attach(struct i2c_adapter *adap)
{
	if(! (fmradio_client = kmalloc(sizeof(struct i2c_client),GFP_KERNEL)))
		return -ENOMEM;
	memcpy(fmradio_client,&client_template,sizeof(struct i2c_client));
	fmradio_client->adapter = adap;
        
	fmradio_client->addr = FM_I2C_ADDR;
	
	printk("adapter %s\n",adap->name);
	i2c_attach_client(fmradio_client);

	return 0;
}


static int fmradio_detach(struct i2c_client *client)
{	
	i2c_detach_client(fmradio_client);
	if( fmradio_client )
		kfree( fmradio_client );

	return 0;
}


/* ----------------------------------------------------------------------- */
static int fmradio_init_module(void)
{
	fm_audio_state.dev_dsp = register_sound_fm(&fm_audio_fops, -1);
	AUDPRINTk1(EZXOSS_DEBUG "register fm device with kernel ok \n");

	return 0;
}


static void fmradio_cleanup_module(void)
{	
	unregister_sound_fm(fm_audio_state.dev_dsp);

	AUDPRINTk1(EZXOSS_DEBUG "unregister fm device \n");
}


module_init(fmradio_init_module);
module_exit(fmradio_cleanup_module);
MODULE_AUTHOR("Jay Jia");
MODULE_LICENSE("GPL");


