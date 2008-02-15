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
 *  Jay Jia(w20091),   Motorola  Nov 25,2003,            Created
 *  Jin Lihong(w20076),Motorola  Jan 13,2004,LIBdd68327  Make the e680 louder speaker work.
 *
 */

#include <linux/miscdevice.h>

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
#include <linux/pxa_camera.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <asm/semaphore.h>


#include <linux/sound.h>
#include <linux/soundcard.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/arch/hardware.h>
#include "i2c-adcm2700.h"

/* Major 10, Minor 244, /dev/camerai2c */
#define CAM_NAME        "cami2c"
#define CAM_MINOR       244

#define TESTCODE
#undef  TESTCODE
#define I2C_E680_CAMERA 0x86

#define I2C_CLIENT_NONE      0
#define I2C_CLIENT_ADCM2700  1
#define I2C_CLIENT_MT9V111   2
static int i2c_camera_client_type = I2C_CLIENT_NONE;
static unsigned long i2c_camera_chipid = 0;
#define REG_ADDRSELECT       (0x01)
#undef BLOCK(a)
#define BLOCK(a) ((u8)(((a)>>7)&0xFF))

static int e680_camera_adapter_attach(struct i2c_adapter *adap);
static int e680_camera_detach(struct i2c_client *client);
static int e680_camera_client_register(struct i2c_client *client);
static int e680_camera_client_unregister(struct i2c_client *client);

typedef struct 
{
  u16 addr;
  union {u16 word; u8 byte;}value;
}i2c_camera_reg;
/* ----------------------------------------------------------------------- */
static struct i2c_driver driver = {
	name:            "e680 camera driver",
	id:              I2C_E680_CAMERA,
	flags:           I2C_DF_DUMMY,
	attach_adapter:  e680_camera_adapter_attach,        
	detach_client:   e680_camera_detach,
};

static struct i2c_adapter e680_camera_adapter = {
        name:                   "e680 camera adapter",
        id:                     I2C_E680_CAMERA,
        client_register:        e680_camera_client_register,
        client_unregister:      e680_camera_client_unregister,
};

static struct i2c_client client_template =
{
    name:   "(unset)",        
    adapter:&e680_camera_adapter,
};

struct i2c_client *e680_camera_client;
unsigned int e680_camera_minor;

static int e680_camera_open(void)
{
	MOD_INC_USE_COUNT;
	
	return 0;
}

static int e680_camera_release(void)
{
	MOD_DEC_USE_COUNT;
	return 0;
}


int e680_camera_read(char *buf, size_t count)
{
    int ret;        
    unsigned int flags;
    
	e680_camera_open();	
    local_irq_save(flags);
    enable_irq(IRQ_I2C);
    ret = i2c_master_recv(e680_camera_client, buf, count);
    local_irq_restore(flags);
	e680_camera_release();
	return ret;
			
}
	
int e680_camera_write(const char *buf, size_t count)
{
	int ret;
    unsigned int flags;
    
	e680_camera_open();
    local_irq_save(flags);
 	enable_irq(IRQ_I2C);
	ret = i2c_master_send(e680_camera_client, buf, count);
    local_irq_restore(flags);
	e680_camera_release();
	return ret;
}

static int e680_camera_client_register(struct i2c_client *client)
{
	
	return 0;
}

static int e680_camera_client_unregister(struct i2c_client *client)
{
	
	return 0;	
}
/* ----------------------------------------------------------------------- */

static int e680_camera_adapter_attach(struct i2c_adapter *adap)
{
	if(! (e680_camera_client = kmalloc(sizeof(struct i2c_client),GFP_KERNEL)))
		return -ENOMEM;
        
	memcpy(e680_camera_client,&client_template,sizeof(struct i2c_client));
	e680_camera_client->adapter = adap;
        
    /*adcm2700 i2c client address*/    
	e680_camera_client->addr = 0x53;
	i2c_attach_client(e680_camera_client);
	return 0;
}	

static int e680_camera_detach(struct i2c_client *client)
{	
	i2c_detach_client(e680_camera_client);
	return 0;
}

/* ----------------------------------------------------------------------- */
static int cam_open(struct inode *inode, struct file *file)
{
        if(i2c_camera_client_type == I2C_CLIENT_NONE)
            return -EINVAL;

        MOD_INC_USE_COUNT;
        return 0;
}

static int i2c_camera_readw(unsigned short addr, unsigned short *pvalue);
static int i2c_camera_readb(unsigned short addr, unsigned char *pvalue);
static int i2c_camera_writew(unsigned short addr, unsigned short value);
static int i2c_camera_writeb(unsigned short addr, unsigned char value);

static int cam_close(struct inode * inode, struct file *file)
{
        MOD_DEC_USE_COUNT;
        return 0;
}

#define DETECT_BUFLEN 256
static int cam_ioctl_detectid (void * arg)
{
    int    buflen, idlen;
    char*  id;
    struct camera_i2c_detectid * param = arg;
    if(copy_from_user(&buflen, &(param->buflen), sizeof(buflen)))
    {
        return -EFAULT;
    }
    if(buflen > DETECT_BUFLEN)
    {
        return -ENOMEM;
    }
    id = kmalloc(DETECT_BUFLEN, GFP_KERNEL);
    if(id == NULL)
    {
        return -ENOMEM;
    }

    idlen = 0;
    switch(i2c_camera_client_type)
    {
        case I2C_CLIENT_MT9V111:
            idlen = snprintf(id, DETECT_BUFLEN-1, "%s %s %lx", 
                            "MICRON", "MT9V111", i2c_camera_chipid);
            break;
        case I2C_CLIENT_ADCM2700:
            idlen = snprintf(id, DETECT_BUFLEN-1, "%s %s %lx", 
                            "AGILENT", "ADCM2700", i2c_camera_chipid);
            break;
        default:
            break;
    }
    id[DETECT_BUFLEN-1] = 0;
    idlen = strlen(id)+1;
    if(buflen < idlen)
    {
        kfree(id);
        return -ENOMEM;
    }
    if(copy_to_user(param->data, id, idlen))
    {
        kfree(id);
        return -EFAULT;
    }
    kfree(id);
    return 0;
}

static int cam_ioctl_register_rw (unsigned int cmd, void * arg)
{
    int ret = -ENOTSUPP;
    struct camera_i2c_register reg;
    if(copy_from_user(&reg, arg, sizeof(reg)))
    {
        return -EFAULT;
    }
    switch(cmd)
    {
        case CAMERA_I2C_WRITEW:
            ret=i2c_camera_writew(reg.addr, reg.value.w);
            break;
        case CAMERA_I2C_WRITEB:
            ret=i2c_camera_writeb(reg.addr, reg.value.b);
            break;
        case CAMERA_I2C_READW:
            if((ret=i2c_camera_readw(reg.addr, &(reg.value.w)))>=0)
            {
                if(copy_to_user(arg, &reg, sizeof(reg)))
                    ret = -EFAULT;
            }
            break;
        case CAMERA_I2C_READB:
            if((ret=i2c_camera_readb(reg.addr, &(reg.value.b)))>=0)
            {
                if(copy_to_user(arg, &reg, sizeof(reg)))
                    ret = -EFAULT;
            }
            break;
        default:
            break;
    }
    return ret;
}

static int cam_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = -ENOTSUPP;
    switch (cmd)
    {
        case CAMERA_I2C_WRITEW:
        case CAMERA_I2C_WRITEB:
        case CAMERA_I2C_READW:
        case CAMERA_I2C_READB:
            ret = cam_ioctl_register_rw(cmd, (void *)arg);
            break;
        case CAMERA_I2C_DETECTID:
            ret = cam_ioctl_detectid((void *)arg);
            break;
        default:
            ret = -EINVAL;
            break;
    }
    return ret;
}

static struct file_operations cam_fops = {
        ioctl:          cam_ioctl,
        open:           cam_open,
        release:        cam_close,
};

static struct miscdevice cam_misc_device = {
        CAM_MINOR,
        CAM_NAME,
        &cam_fops,
};

/* ----------------------------------------------------------------------- */
static int e680_camera_init_module(void)
{
	int res;
	
	res = i2c_add_driver(&driver);
	if( res < 0 )
	{
		printk("error in add i2c driver\n");
		return res;
	}
    if (misc_register (&cam_misc_device))
    {
        printk(KERN_ERR "Couldn't register cam driver\n");
        return -EIO;
    }

	return 0;
}

static void e680_camera_cleanup_module(void)
{	
	i2c_del_driver(&driver);
    misc_deregister(&cam_misc_device);
}
/*-----------------------------------------------------------*/
static int ChgBlockAddr(unsigned char block)
{
    static unsigned char old_blk = 0xFF;
    int res;
    char tmp[2]={BLOCK_SWITCH_CMD, block};
    
    if(block == old_blk)
    {
	return 0;
    }
    
    res = e680_camera_write(tmp, 2);
    if(res < 0)
    {
        dbg_print("error code = %d", res);
	return  -1;
    }
    old_blk = block;
    
    return 0; 
}

int adcm2700_read(unsigned short addr, unsigned short *pvalue)
{
	unsigned char	blockaddr = BLOCK(addr);
	unsigned char	offset    = OFFSET_R(addr);
        int     ret;

	if((ret = ChgBlockAddr(blockaddr)) != 0)  
	{
	    dbg_print("Change block address failed. block = 0x%2x", blockaddr);
	    return -1;
	}
        if((ret = e680_camera_write(&offset, 1)) < 0)
        {
            dbg_print("i2c write error code =%d", ret);
            return -1;
        }
        
        ret = e680_camera_read((char *)pvalue, 2); 
        if(ret < 0)
        {
           dbg_print("i2c read error oce = %d", ret);
           return -1;
        } 
	
	return  *pvalue;
}	
int adcm2700_write_byte(unsigned short addr, unsigned char value)
{
	unsigned char	blockaddr = BLOCK(addr);
	unsigned char 	offset    = OFFSET(addr);;
    char    tmp[3]={offset, value};
    int     ret;
		
	if((ret = ChgBlockAddr(blockaddr)) != 0)  
	{
	   dbg_print("Change block address failed. block = 0x%2x", blockaddr);
	   return -1;
	}
	ret = e680_camera_write(tmp, 2);
        if(ret < 0)
        {
          dbg_print("i2c write error code = %d", ret);
          return -1;
        }
    return 0;
 
}
int adcm2700_read_byte(unsigned short addr, unsigned char * pvalue)
{
  	unsigned char	blockaddr = BLOCK(addr);
	unsigned char 	offset    = OFFSET_R(addr);;
    int     ret;
		
	if((ret = ChgBlockAddr(blockaddr)) != 0)  
	{
	   dbg_print("Change block address failed. block = 0x%2x", blockaddr);
	   return -1;
	}
    if((ret = e680_camera_write(&offset, 1)) < 0)
        {
            dbg_print("i2c write error code =%d", ret);
            return -1;
        }
        
        ret = e680_camera_read((char *)pvalue, 1); 
        if(ret < 0)
        {
           dbg_print("i2c read error oce = %d", ret);
           return -1;
        } 
    return 0;

}
int adcm2700_write(unsigned short addr, unsigned short value)
{
	unsigned char	blockaddr = BLOCK(addr);
	unsigned char 	offset    = OFFSET(addr);;
        char    tmp[3]={offset, (char)(value&0xFF), (char)(value>>8)};
        int     ret;
		
	if((ret = ChgBlockAddr(blockaddr)) != 0)  
	{
	   dbg_print("Change block address failed. block = 0x%2x", blockaddr);
	   return -1;
	}
	ret = e680_camera_write(tmp, 3);
        if(ret < 0)
        {
          dbg_print("i2c write error code = %d", ret);
          return -1;
        }
        return 0;
}

int i2c_adcm2700_cleanup(void)
{
  i2c_camera_client_type = I2C_CLIENT_NONE;
  return 0;
} 
int i2c_adcm2700_init(void)
{
    unsigned short chipid;
    unsigned short adcm_chipids[] = {ADCM2700_CHIP_ID, ADCM2700_CHIP_ID_NEW, 0x62};
    int i;
    e680_camera_client->addr = 0x53;
 
    if(adcm2700_read(0, &chipid) < 0)
    {
       return -1;
    }

    dbg_print("CHIP ID IS %x", chipid);
    
    for(i = 0; i < sizeof(adcm_chipids)/sizeof(adcm_chipids[0]); i++)
    {
       if(chipid == adcm_chipids[i])
          break;	       
    }
    
    if(i >= sizeof(adcm_chipids)/sizeof(adcm_chipids[0]))
    {
       return -1;
    } 

   i2c_camera_client_type = I2C_CLIENT_ADCM2700;
   i2c_camera_chipid = chipid;
    
   return 0;
}
//=======================================================================
static int  mt9v111_addr_select(unsigned char addrSpace)
{
  int    ret;
  static u8 curSpace = 0;
  char   buf[3] = {REG_ADDRSELECT, 0, addrSpace};

  if(curSpace != addrSpace)
  {
    if((ret = e680_camera_write(buf, 3)) < 0)
    {
       dbg_print("i2c write error code = %d", ret);
       return  ret;
    }
    curSpace = addrSpace;
  }
  return 0;
}

int mt9v111_read(unsigned char addrSpace, unsigned short addr)
{
    unsigned short value;
    unsigned char  reg_addr = (unsigned char)addr;
    
    if(addr != REG_ADDRSELECT)
    {
      if(mt9v111_addr_select(addrSpace) != 0)
      {
        dbg_print("Address space select  failed. addrSpace = 0x%2x", addrSpace);
        return -1;
      }
    }
    if(e680_camera_write(&reg_addr, 1) < 0)
    {
       return -1;
    }

	if(e680_camera_read((char *)&value, 2) < 0)
    {
        return -1;
    }
    
    value = (value << 8 | value >> 8);
    return value;
}	

int mt9v111_write(unsigned char addrSpace, unsigned short addr, unsigned short value)
{
    int  ret;
    char buf[3] = {(char)addr, (char)(value>>8), (char)value};
    
    if(addr != REG_ADDRSELECT)
    {
      if(mt9v111_addr_select(addrSpace) != 0)
      {
       dbg_print("Address space select  failed. addrSpace = 0x%2x", addrSpace);
       return -1;
      }
    }
    
    if((ret = e680_camera_write(buf, 3)) < 0)
    {
      dbg_print("write error code = %d", ret);
    }
    
	return ret;
}	
int i2c_mt9v111_cleanup(void)
{
  i2c_camera_client_type = I2C_CLIENT_NONE;
  return 0;
} 
int i2c_mt9v111_init(void)
{
    int chipid;
    e680_camera_client->addr = 0x5C;
 
    if((chipid = mt9v111_read(0x04, 0x36)) < 0)
    {
       return -1;
    }
    
    if((chipid & 0xFF00) != 0x8200)
    {
       return -1;
    } 

   i2c_camera_client_type = I2C_CLIENT_MT9V111;
   i2c_camera_chipid = chipid;
    
   return 0;

}

static int i2c_camera_readw(unsigned short addr, unsigned short *pvalue)
{
    int ret = -ENOTSUPP;
    switch(i2c_camera_client_type)
    {
        case I2C_CLIENT_MT9V111:
            ret = mt9v111_read((addr>>8), (addr&0xFF));
            if(ret > 0)
            {
               *pvalue = (u16)ret;
            }
            break;
        case I2C_CLIENT_ADCM2700:
            ret = adcm2700_read(addr, pvalue);
            break;
        default:
            break;
    }
    return ret;
}

static int i2c_camera_readb(unsigned short addr, unsigned char *pvalue)
{
    int ret = -ENOTSUPP;
    unsigned short value;
    switch(i2c_camera_client_type)
    {
        case I2C_CLIENT_ADCM2700:
            ret = adcm2700_read(addr, &value);
            *pvalue = (value>>8);
            break;
        default:
            break;
    }
    return ret;
}

static int i2c_camera_writew(unsigned short addr, unsigned short value)
{
    int ret = -ENOTSUPP;
    switch(i2c_camera_client_type)
    {
        case I2C_CLIENT_MT9V111:
            ret = mt9v111_write((addr>>8), (addr&0xFF), value);
            break;
        case I2C_CLIENT_ADCM2700:
            ret = adcm2700_write(addr, value);
            break;
        default:
            break;
    }
    return ret;
}

static int i2c_camera_writeb(unsigned short addr, unsigned char value)
{
    int ret = -ENOTSUPP;
    switch(i2c_camera_client_type)
    {
        case I2C_CLIENT_ADCM2700:
            ret = adcm2700_write_byte(addr, value);
            break;
        default:
            break;
    }
    return ret;
}


EXPORT_SYMBOL(i2c_adcm2700_init);
EXPORT_SYMBOL(adcm2700_write);
EXPORT_SYMBOL(adcm2700_write_byte);
EXPORT_SYMBOL(adcm2700_read_byte);
EXPORT_SYMBOL(adcm2700_read);
EXPORT_SYMBOL(i2c_adcm2700_cleanup);

EXPORT_SYMBOL(i2c_mt9v111_init);
EXPORT_SYMBOL(mt9v111_write);
EXPORT_SYMBOL(mt9v111_read);
EXPORT_SYMBOL(i2c_mt9v111_cleanup);

module_init(e680_camera_init_module);
module_exit(e680_camera_cleanup_module);
MODULE_AUTHOR("Jay Jia");
MODULE_LICENSE("GPL");
