/*
 * Copyright (C) 2005 Motorola Inc.
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
 */

/*
    03/25/2005, created by w20596, for EZX platform
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
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <asm/semaphore.h>

#include <linux/light_sensor.h>
#include "light_sensor_cam.h"
#include "camera.h"
#ifdef  CONFIG_CAMERA_MT9M111
#include "mt9m111.h"
#endif

static int initialized = 0;

//Light sensor functions
static int light_sensor_init(void)
{
    int ret;
    if(initialized)
        return -EINVAL;
    
    ret = camera_trylock();
    if(ret)
    {
        dbg_print("camera device is busy");
        return -EBUSY;
    }

    //disable CPU/CI clock dynamic change when open camera device:
    if((ret = cam_ipm_hook()))
    {
        err_print("cam_ipm_hook fail! return %d", ret);
        return ret;
    }

    ret = -ENOTSUPP;
#ifdef  CONFIG_CAMERA_MT9M111
    ret = light_sensor_mt9m111_init();
#endif
    if(ret<0)
    {
        //reenable CPU/CI clock dynamic change when close camera device:
        cam_ipm_unhook();

        err_print("error: initial fail");
        camera_unlock();
    }
    initialized = 1;

    return ret;
}

static int light_sensor_deinit(void)
{
    if(!initialized)
        return -EINVAL;

    int ret = -ENOTSUPP;
#ifdef  CONFIG_CAMERA_MT9M111
    ret = light_sensor_mt9m111_deinit();
#endif

    //reenable CPU/CI clock dynamic change when close camera device:
    cam_ipm_unhook();

    initialized = 0;
    camera_unlock();

    return ret;
}


static int light_sensor_ioctl_get_luma(void *arg)
{
    int ret = -ENOTSUPP;
    int start = 0;
    if(!initialized)
    {
        start = 1;
        ret = light_sensor_init();
        if(ret<0)
            return ret;
    }

    struct light_sensor_luminance luma;
#ifdef  CONFIG_CAMERA_MT9M111
    ret = light_sensor_mt9m111_get_luma(&(luma.integer), &(luma.decimal));
#endif
    if(start)
        light_sensor_deinit();
    
    if(ret<0)
        return ret;
    if(copy_to_user(arg, &luma, sizeof(luma)))
        ret = -EFAULT;
    return ret;
}


/* ----------------------------------------------------------------------- */
static int light_sensor_open(struct inode *inode, struct file *file)
{
    MOD_INC_USE_COUNT;

    return 0;
}

static int light_sensor_close(struct inode * inode, struct file *file)
{
    light_sensor_deinit();
            
    MOD_DEC_USE_COUNT;
    return 0;
}

static int light_sensor_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = -ENOTSUPP;
    switch (cmd)
    {
        case LIGHT_SENSOR_START:
            ret = light_sensor_init();
            break;
        case LIGHT_SENSOR_STOP:
            ret = light_sensor_deinit();
            break;
        case LIGHT_SENSOR_GET_LUMA:
            ret = light_sensor_ioctl_get_luma((void *)arg);
            break;
        default:
            ret = -EINVAL;
            break;
    }
    return ret;
}

static ssize_t light_sensor_read(struct file *file, char *buf, size_t count, loff_t *ptr)
{
    int ret;

    /* Can't seek (pread) on this device */
    if (ptr != &file->f_pos) 
    {
        return -ESPIPE;
    }

    if (count == 0)
    {
        return 0;
    }

    ret = light_sensor_init();
    if(ret<0)
        return ret;

    char strbuf[32];
    ret = -ENOTSUPP;
    struct light_sensor_luminance luma;
#ifdef  CONFIG_CAMERA_MT9M111
    ret = light_sensor_mt9m111_get_luma(&(luma.integer), &(luma.decimal));
#endif
    if(ret>=0) {
        sprintf(strbuf, "%d.%03d\n", luma.integer, luma.decimal);
    }
    else {
        sprintf(strbuf, "error:%d\n", ret);
    }

    int n = strlen(strbuf);
    ddbg_print("%d:%d  %s", count, n, strbuf);

    /* copy data */
    if(n<=count)
    {
        if(copy_to_user(buf, strbuf, n))
            return -EFAULT;
        ret = n;
    }
    else
        ret = -EAGAIN;

    light_sensor_deinit();
    return ret;
}

static struct file_operations light_sensor_fops = {
        ioctl:          light_sensor_ioctl,
        read:           light_sensor_read,
        open:           light_sensor_open,
        release:        light_sensor_close,
};

static struct miscdevice light_sensor_misc_device = {
        LIGHT_SENSOR_MINOR,
        LIGHT_SENSOR_NAME,
        &light_sensor_fops,
};

/* ----------------------------------------------------------------------- */
static int __init light_sensor_init_module(void)
{
    if (misc_register (&light_sensor_misc_device))
    {
        printk(KERN_ERR "Couldn't register light sensor driver\n");
        return -EIO;
    }

    return 0;
}

static void __exit light_sensor_exit_module(void)
{	
    misc_deregister(&light_sensor_misc_device);
}


module_init(light_sensor_init_module);
module_exit(light_sensor_exit_module);

MODULE_DESCRIPTION("Light sensor driver");
MODULE_LICENSE("GPL");

