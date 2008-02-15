/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/module/accy.c
 * 
 * Description - This file includes all of the interfaces for the accessory driver 
 *
 * Copyright (C) 2004 - 2006 Motorola, Inc. All Rights Reserved.
 */

/* 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  
 * 02111-1307, USA
 *
 * 2004-Dec-17 - Design Implementation of the accessory driver.
 *
 * 2006-Jan-17 - Finalized the implementation of the accessory driver.
 */

/*!
 * @file accy.c
 *
 * @ingroup poweric_accy
 *
 * @brief The implementation of the accessory driver
 *
 * This file includes all of the interfaces for the accesory driver including
 * initialization and tear-down functions as well as the kernel-level interface
 * for reporting accessory events and the user-mode device file access functions
 * (open(), close(), read(), etc.).
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/apm_bios.h>

#ifdef CONFIG_CPU_BULVERDE
#include <linux/ezxusbd.h>
#endif

#include <linux/power_ic.h>
#include <linux/moto_accy.h>
#include "emu.h"
#include "../core/os_independent.h"

/******************************************************************************
* Constants
******************************************************************************/

/*! Maximum number of times that /dev/accy can be opened concurrently */
#define MAX_OPENS 5

/******************************************************************************
* Local Variables
******************************************************************************/

/* Wait queue for poll() */
static DECLARE_WAIT_QUEUE_HEAD(accy_wait);

/*! Bitmap holding state of the accessories (bit set indicates connected) */
static MOTO_ACCY_MASK_T connected_accessories = 0;

/*! Lock to control access to the bitmask of connected accessories */
static spinlock_t connected_lock = SPIN_LOCK_UNLOCKED;

/*! Array of structures (one structure per opened file) holding event information */
static struct
{
    int open; /*!< flag indicating if slot is in use */
    spinlock_t lock; /*!< lock to control access to following data */
    MOTO_ACCY_MASK_T interesting_accessories; /*!< Bitmap showing which accessories are interesting */
    MOTO_ACCY_MASK_T changed_accessory; /*!< Bitmap indicate when accessory states change */
} changed_flags[MAX_OPENS];

/*! Lock to control changes to the open flags in the changed_flags array */
static spinlock_t changed_lock = SPIN_LOCK_UNLOCKED;

/******************************************************************************
* Local Functions
******************************************************************************/

/*!
 * @brief the open() handler for the accessory device node
 *
 * This function implements the open() system call on the accessory device node.
 * The function "allocates" a slot in the changed_flags array for the open device
 * and resets the contents of the changed_flags array slot.  If all of the slots
 * have been allocated, the function returns -EBUSY and the open() fails.
 *
 * @param        inode       inode pointer
 * @param        file        file pointer
 *
 * @return 0 if successfully opened, < 0 otherwise
 */

static int accy_open(struct inode *inode, struct file *file) 
{
    int i;

    /* Acquire the lock to ensure our exclusive access to the changed_flags array */
    spin_lock (&changed_lock);

    /* Find the first open slot in the array */
    for (i = 0; i < MAX_OPENS; i++)
    {
        if (changed_flags[i].open == 0)
        {
            /* Reset the variables */
            changed_flags[i].lock = SPIN_LOCK_UNLOCKED;

            ACCY_BITMASK_SET_ALL(changed_flags[i].interesting_accessories);
            ACCY_BITMASK_CLEAR_ALL(changed_flags[i].changed_accessory);

            /* Claim the slot for this file descriptor */
            changed_flags[i].open = 1;

            break;
        }
    }

    /* We are done, so release the lock */
    spin_unlock (&changed_lock);

    /* If we could not find a free slot, return busy */
    if (i == MAX_OPENS)
    {
        return -EBUSY;
    }

    /* Save "i" in the file to remember which slot we are using */
    file->private_data = (void *)i;

    return 0;
}

/*!
 * @brief the close() handler for the accessory device node
 *
 * This function implements the close() system call on the accessory device node.
 * This function releases the allocated slot in the changed_flags array.
 *
 * @param        inode       inode pointer
 * @param        file        file pointer
 *
 * @return 0
 */

static int accy_free(struct inode *inode, struct file *file)
{
    /* Acquire the lock to ensure our exclusive access to the changed_flags array */
    spin_lock (&changed_lock);

    /* Release our slot in the array */
    changed_flags[(int)file->private_data].open = 0;

    /* We are done, so release the lock */
    spin_unlock (&changed_lock);

    return 0;
}

/*!
 * @brief the ioctl() handler for the accessory device node
 *
 * This function implements the ioctl() system call for the accessory device node.
 *
 * @param        inode       inode pointer
 * @param        file        file pointer
 * @param        cmd         the ioctl() command
 * @param        arg         the ioctl() argument
 *
 * @return 0 if successful
 */

static int accy_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    int retval;
    unsigned long data;
    MOTO_ACCY_MASK_T mask;
    MOTO_ACCY_DEVICE_STATE_T device_state;
    
    switch (cmd)
    {
        case MOTO_ACCY_IOCTL_GET_INTERESTED_MASK:
            /* Retreive the bitmask */
            mask = changed_flags[(int)file->private_data].interesting_accessories;

            /* Copy the data to user space */
            retval = copy_to_user ((void *)arg, (void *)&mask, sizeof(mask));

            /* If the copy failed, return an error */
            if (retval != 0)
            {
                retval = -EFAULT;
            }

            break;

        case MOTO_ACCY_IOCTL_SET_INTERESTED_MASK:
            /* Copy the new mask from user space */
            retval = copy_from_user ((void *)&mask, (void *)arg, sizeof(mask));

            /* If the copy failed, return an error */
            if (retval != 0)
            {
                retval = -EFAULT;
            }

            /* Else, save the new bitmask */
            else
            {
                /* Lock out access while writing the interesting accessories */
                spin_lock(&(changed_flags[(int)file->private_data].lock));

                /* Write the new interesting accessories mask */
                changed_flags[(int)file->private_data].interesting_accessories = mask;

                /* Release the lock */
                spin_unlock(&(changed_flags[(int)file->private_data].lock));
            }

            break;

        case MOTO_ACCY_IOCTL_GET_DEVICE_STATE:
            /* Copy the device number from user space */
            retval = copy_from_user ((void *)&device_state, (void *)arg, sizeof(device_state));

            /* If the copy failed, return an error */
            if (retval != 0)
            {
                retval = -EFAULT;
            }

            /* Else, retrive the device state and try to pass it back to the caller */
            else
            {
                /* Acquire the lock to prevent changes to connected_accessories */
                spin_lock (&connected_lock);

                /* Set the state field to 1 if the accessory is connected */
                device_state.state = ACCY_BITMASK_ISSET(connected_accessories, device_state.device);

                /* Release the lock for connected_accessories */
                spin_unlock (&connected_lock);

                /* Copy the data back to user space */
                retval = copy_to_user ((void *)arg, (void *)&device_state, sizeof(device_state));

                /* If the copy failed, return an error */
                if (retval != 0)
                {
                    retval = -EFAULT;
                }
            }

            break;

        case MOTO_ACCY_IOCTL_SET_CHARGER_LOAD_LINE:
            /* Copy the load line setting from user space */
            retval = copy_from_user ((void *)&data, (void *)arg, sizeof(data));

            /* If the copy failed, return an error */
            if (retval != 0)
            {
                retval = -EFAULT;
            }

            /* Else, configure the hardware for the requested load line setting */
            else
            {
                /* Acquire the lock to prevent changes to connected_accessories */
                spin_lock (&connected_lock);

                /* Verify that the 3G fast charger is connected */
                if (ACCY_BITMASK_ISSET(connected_accessories, MOTO_ACCY_TYPE_CHARGER_FAST_3G))
                {
                    /* This charger is the only charger that supports an adjustable load line */

                    /* TBD */
                }

                /* Else, the appropriate charger type is not connected */
                else
                {
                    /* Return error: No such device */
                    retval = -ENODEV;
                }

                /* Release the lock for connected_accessories */
                spin_unlock (&connected_lock);
            }

            break;

        case MOTO_ACCY_IOCTL_SET_MUTE_STATE:
            /* Copy the mute setting from user space */
            retval = copy_from_user ((void *)&device_state, (void *)arg, sizeof(device_state));

            /* If the copy failed, return an error */
            if (retval != 0)
            {
                retval = -EFAULT;
            }

            /* Else, configure the hardware for the request mute setting */
            else
            {
                /* Acquire the lock to prevent changes to connected_accessories */
                spin_lock (&connected_lock);

                /* Verify that the requested accessory is connected */
                if (ACCY_BITMASK_ISSET(connected_accessories, device_state.device) == 0)
                {
                    /* Device is not connected, return error: No such device */
                    retval = -ENODEV;
                }

                /* Verify that the requested accessory can be muted */
                else if ((device_state.device != MOTO_ACCY_TYPE_CARKIT_MID) &&
                         (device_state.device != MOTO_ACCY_TYPE_CARKIT_FAST) &&
                         (device_state.device != MOTO_ACCY_TYPE_CARKIT_SMART) &&
                         (device_state.device != MOTO_ACCY_TYPE_HEADSET_EMU_MONO) &&
                         (device_state.device != MOTO_ACCY_TYPE_HEADSET_EMU_STEREO))
                {
                    /* Return error: Invalid argument */
                    retval = -EINVAL;
                }

                /* Device is connected and valid, configuration can occur */
                else
                {
                    moto_accy_set_accessory_power(device_state.device, device_state.state);
                }

                /* Release the lock for connected_accessories */
                spin_unlock (&connected_lock);
            }

            break;
            
        case MOTO_ACCY_IOCTL_GET_ALL_DEVICES:
            /* Just pass the bitmask of accessories back to user space. */
            mask = moto_accy_get_all_devices();
            
            /* Copy the data to user space */
            retval = copy_to_user ((void *)arg, (void *)&mask, sizeof(mask));
            break;

        default:
            retval = -ENOTTY;
            break;
    }

    return retval;
}

/*!
 * @brief the poll() handler for the accessory driver device node
 *
 * This function will add the accessory wait queue to the poll table to allow
 * for polling for accessory events to occur.  If an event is pending for the
 * device, the function returns the indication that data is available.
 *
 * @param        file        file pointer
 * @param        wait        poll table for this poll()
 *
 * @return 0 if no data to read, (POLLIN|POLLRDNORM) if data available
 */

static unsigned int accy_poll(struct file *file, poll_table *wait)
{
    unsigned int retval = 0;

    /* Add our wait queue to the poll table */
    poll_wait (file, &accy_wait, wait);

    /* Acquire lock to prevent changes to changed_accessory bitmask */
    spin_lock(&(changed_flags[(int)file->private_data].lock));

    /* If there are changes, indicate that there is data available to read */
    if (!ACCY_BITMASK_ISEMPTY(changed_flags[(int)file->private_data].changed_accessory))
    {
        retval = (POLLIN | POLLRDNORM);
    }

    /* Release the lock on changed_accessory */
    spin_unlock(&(changed_flags[(int)file->private_data].lock));

    return retval;
}

/*!
 * @brief the read() handler for the accessory driver device node
 *
 * This function implements the read() system call for the accessory driver device
 * node.  The function returns up to one accessory event per call as an unsigned
 * long integer (4 bytes).  The value passed back is an accessory number (from
 * the MOTO_ACCY_TYPE_T enumeration) and, in the case of an insertion event,
 * is OR'd with the MOTO_ACCY_CONNECTED_FLAG (the most significant bit).
 *
 * If no events are pending, the function will return immediately with a return
 * value of 0, indicating that no data was copied into the buffer.  If more than
 * one event is pending, the function will return only one event and an additional
 * call to read() will need to be made to read the other events.
 *
 * @param        file        file pointer
 * @param        buf         buffer pointer to copy the data to
 * @param        count       size of buffer
 * @param        ppos        read position (ignored)
 *
 * @return 0 if nothing changed, 4 if an event was stored in the buffer, < 0 on error
 */

static ssize_t accy_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
    ssize_t retval = 0;
    unsigned long accy;
    unsigned long i;

    /* We always return one unsigned long per read, so count must be large enough */
    if (count < sizeof(accy))
    {
        return -EINVAL;
    }

    /* Lock out changes to the flags while we read/update them */
    spin_lock(&connected_lock);
    spin_lock(&(changed_flags[(int)file->private_data].lock));

    /* Check to see if anything at all has changed */
    if (!ACCY_BITMASK_ISEMPTY(changed_flags[(int)file->private_data].changed_accessory))
    {
        /* Find out what it was that changed */
        accy = i = ACCY_BITMASK_FFS(changed_flags[(int)file->private_data].changed_accessory);

        /* If the accessory is connected, OR in the connected flag */
        if (ACCY_BITMASK_ISSET(connected_accessories, i))
        {
            accy |= MOTO_ACCY_CONNECTED_FLAG;
        }

        /* Copy the accessory information into the buffer */
        retval = copy_to_user ((unsigned long *)buf, (void *)&accy, sizeof(accy));

        /* Check to see if the copy was successful */
        if (retval == 0)
        {
            /* Return value should be the size of the copied data */
            retval = sizeof(accy);

            /* Clear the changed flag since the data has been reported to app */
            ACCY_BITMASK_CLEAR(changed_flags[(int)file->private_data].changed_accessory, i);
        }
    }

    /* Release the lock */
    spin_unlock(&(changed_flags[(int)file->private_data].lock));
    spin_unlock(&connected_lock);

    return retval;
}

/*! This structure defines the file operations for the accessory device */
static struct file_operations accy_fops =
{
    .owner =    THIS_MODULE,
    .ioctl =    accy_ioctl,
    .open =     accy_open,
    .release =  accy_free,
    .read =     accy_read,
    .poll =     accy_poll,
};

/******************************************************************************
* Global Functions
******************************************************************************/

/*!
 * @brief accessory module initialization function
 *
 * This function performs the initialization of the data structures for the
 * accessory driver.  Currently, this just involves registering our character
 * device.
 *
 * @return 0
 */

void __init moto_accy_init (void)
{
    int ret;

    /* Register our character device */
    ret = register_chrdev(MOTO_ACCY_DRIVER_MAJOR_NUM, MOTO_ACCY_DRIVER_DEV_NAME, &accy_fops);

    /* Display a message if the registration fails */
    if (ret < 0)
    {
        tracemsg(_k_d("unable to get a major (%d) for accessory driver: %d"),
            (int)MOTO_ACCY_DRIVER_MAJOR_NUM, ret);
    }
    
#ifdef CONFIG_ARCH_SCMA11   
    devfs_mk_cdev(MKDEV(MOTO_ACCY_DRIVER_MAJOR_NUM,0), S_IFCHR | S_IRUGO | S_IWUSR, "accy");
#endif
}

/*!
 * @brief accessory module cleanup function
 *
 * This function is called when the power IC driver is being destroyed (which
 * generally never happens).  Its purpose is to unregister the character special
 * device that was registered when the accessory module was initialized.
 */

void moto_accy_destroy (void)
{
    unregister_chrdev(MOTO_ACCY_DRIVER_MAJOR_NUM, MOTO_ACCY_DRIVER_DEV_NAME);
}

/*!
 * @brief interface function to pass notification of accessory insertion to accessory driver
 *
 * This function receives notifications of accessory insertions from other modules and
 * adds this information into a "queue" of accessory events maintained for each open
 * accessory device file.  If any processes are waiting for these indications, they are
 * woken up.
 *
 * @param accy type of inserted accessory
 */

void moto_accy_notify_insert (MOTO_ACCY_TYPE_T accy)
{
    int i;

#ifdef CONFIG_CPU_BULVERDE
    /* If USB cable is detected and in NFS mode, enable the pull-up */
    if (accy == MOTO_ACCY_TYPE_CABLE_USB && usbd_in_nfs_mode())
    {
        power_ic_periph_set_usb_pull_up(0);
        mdelay(1);
        power_ic_periph_set_usb_pull_up(1);
     }
#endif

    /* Only do something if the insert is for a device not already connected */
    if (ACCY_BITMASK_ISSET(connected_accessories, accy) == 0)
    {
#ifdef CONFIG_CPU_BULVERDE
        /* Notify power management */
        apm_event_notify(APM_EVENT_DEVICE, EVENT_DEV_ACCS, accy | MOTO_ACCY_CONNECTED_FLAG);
#endif

        /* Indicate that the accessory is now connected */
        ACCY_BITMASK_SET(connected_accessories, accy);
        
        /* Mark the change for all open files */
        for (i = 0; i < MAX_OPENS; i++)
        {
            if (changed_flags[i].open)
            {
                /* Lock out access while we make changes to the flag */
                spin_lock(&(changed_flags[i].lock));

                /* If the changed flag for the accessory was already set, clear it */
                if (ACCY_BITMASK_ISSET(changed_flags[i].changed_accessory, accy))
                {
                    ACCY_BITMASK_CLEAR(changed_flags[i].changed_accessory, accy);
                }

                /* Else, if the accessory event is interesting, set the changed flag */
                else if (ACCY_BITMASK_ISSET(changed_flags[i].interesting_accessories, accy))
                {
                    ACCY_BITMASK_SET(changed_flags[i].changed_accessory, accy);
                }

                /* Release the lock */
                spin_unlock(&(changed_flags[i].lock));
            }
        }

        /* Wake up any threads waiting for accessory events */
        wake_up_interruptible(&accy_wait);
    }
}

/*!
 * @brief interface function to pass notification of accessory removal to accessory driver
 *
 * This function receives notifications of accessory removals from other modules and
 * adds this information into a "queue" of accessory events maintained for each open
 * accessory device file.  If any processes are waiting for these indications, they are
 * woken up.
 *
 * @param accy type of removed accessory
 */

void moto_accy_notify_remove (MOTO_ACCY_TYPE_T accy)
{
    int i;

    /* Only do something if the removal is for a device that is connected */
    if (ACCY_BITMASK_ISSET(connected_accessories, accy))
    {
#ifdef CONFIG_CPU_BULVERDE
        /* Notify power management */
        apm_event_notify(APM_EVENT_DEVICE, EVENT_DEV_ACCS, accy);
#endif

        /* Indicate that the accessory is not connected */
        ACCY_BITMASK_CLEAR(connected_accessories, accy);

        /* Mark the change for all open files */
        for (i = 0; i < MAX_OPENS; i++)
        {
            if (changed_flags[i].open)
            {
                /* Lock out access while we make changes to the flag */
                spin_lock(&(changed_flags[i].lock));

                /* If the changed flag for the accessory was already set, clear it */
                if (ACCY_BITMASK_ISSET(changed_flags[i].changed_accessory, accy))
                {
                    ACCY_BITMASK_CLEAR(changed_flags[i].changed_accessory, accy);
                }

                /* Else, if the accessory event is interesting, set the changed flag */
                else if (ACCY_BITMASK_ISSET(changed_flags[i].interesting_accessories, accy))
                {
                    ACCY_BITMASK_SET(changed_flags[i].changed_accessory, accy);
                }

                /* Release the lock */
                spin_unlock(&(changed_flags[i].lock));
            }
        }

        /* Wake up any threads waiting for accessory events */
        wake_up_interruptible(&accy_wait);
    }
}


/*!
 * @brief Interface function to retrieve the status of all attached accessories.
 *
 * This function is intended to simplify the initial discovery of accessories attached
 * to the phone soon after powerup. Users of the driver should already be familiar with
 * the bitmask as they set a mask of interesting devices for attach/removal notification.
 * For each device in the mask, a bit set indicates that device is attached.
 *
 * @return Bitmask of attached accessories.
 */
MOTO_ACCY_MASK_T moto_accy_get_all_devices(void)
{
    return connected_accessories;
}

/*!
 * @brief Interface function to set the necessary signals based on the accessory type
 *
 *  This function calls the macro in emu_eoc.h to set the SW B+ for headset or the ID_PD for
 *  the car kit.
 *
 * @param device type - the accessory type
 * @param on/off - ENABLE/DISABLE the signal
 *
 */
void moto_accy_set_accessory_power(MOTO_ACCY_TYPE_T device, int on_off)
{
    switch(device)
    {
        case MOTO_ACCY_TYPE_CARKIT_MID:          
        case MOTO_ACCY_TYPE_CARKIT_FAST:         
        case MOTO_ACCY_TYPE_CARKIT_SMART:       
            EMU_SET_ID_PULL_DOWN(on_off);
            break;
            
        case MOTO_ACCY_TYPE_HEADSET_EMU_MONO:
        case MOTO_ACCY_TYPE_HEADSET_EMU_STEREO:
            EMU_SET_REVERSE_MODE(on_off);
            break;
        default:
            break;
    }
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
EXPORT_SYMBOL( moto_accy_init);
EXPORT_SYMBOL( moto_accy_destroy );
EXPORT_SYMBOL(moto_accy_notify_insert );
EXPORT_SYMBOL(moto_accy_notify_remove );
EXPORT_SYMBOL(moto_accy_get_all_devices);
EXPORT_SYMBOL(moto_accy_set_accessory_power);
#endif
