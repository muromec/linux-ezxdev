/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/core/spi_user.c
 *
 * Description - This file contains the ioctls necessary to use the SPI from user space.
 *
 * Copyright (C) 2005 - Motorola
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Motorola 2005-Sep-13 - File Creation
 *
 */
/*!
 * @defgroup spi_user User space interface to the SPI
 */
/*!
 * @file spi_user.c
 *
 * @ingroup spi_user
 *
 * @brief This file contains the ioctls necessary to use the SPI from user space.
 *
 *    Provides an interface to control the SPI from user space.  The following
 * ioctls are provided:
 *
 *    -# #MOTO_SPI_IOCTL_GET_SPI_PARAMS Get the SPI parameters for the file handle.
 *    -# #MOTO_SPI_IOCTL_SET_SPI_PARAMS Set the SPI parameters for the file handle.
 *    -# #MOTO_SPI_IOCTL_SET_FILLER Set the filler byte to be used for the file handle.
 *    -# #MOTO_SPI_IOCTL_TRANSCEIVE Send and receive data for the file handle.
 *    -# #MOTO_SPI_IOCTL_LOCK Lock or unlock the chip select for the file handle.
 */

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi_user.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <stdbool.h>
#include "spi_main.h"
#include "os_independent.h"

/******************************************************************************
* Constants
******************************************************************************/

/*!
 * @brief Initial size of scratch buffers used to hold data moving between
 *        kernel and user space
 *
 * The initial size of the kernel scratch buffers which user space data is
 * copied into.  These buffers will dynamically grow to bigger sizes if
 * necessary.  The buffers are allocated when the file is opened, so a balance
 * must be acheived between the memory used and the number of times the buffer
 * will be re-allocated.
 */
#define SCRATCH_BUFFER_SIZE 32

/*! @brief Maximum number of times that /dev/spi_user can be opened concurrently. */
#define MAX_OPENS 5

/******************************************************************************
* Local Variables
******************************************************************************/

typedef struct
{
    /*! @brief flag indicating if slot is in use. */
    int open;

    /*! @brief Transfer parameters passed to SPI driver. */
    SPI_DATA_PARAMETER_T params;
    
    /*! @brief Value of filler byte used to fill dummy data. */
    unsigned char filler_byte;
    
     /*!
      * @brief The number of bytes allocated for the scratch buffers.  The * 2
      * is done since a buffer is needed for Tx and for Rx.
      */
    unsigned int data_sz[(MOTO_SPI_MAX_VECTORS * 2)];
    /*!
     * @brief Scratch buffers used when Rx or Tx buffer is in user-space.
     */ 
    void *data[(MOTO_SPI_MAX_VECTORS * 2)];
} SPI_FDS_T;

/*! @brief Data for the open file descriptors. */
static SPI_FDS_T open_fds[MAX_OPENS];

/*! @brief Lock to control changes to the open flags in the open_fds array. */
static spinlock_t global_lock = SPIN_LOCK_UNLOCKED;

/******************************************************************************
* Local Functions
******************************************************************************/

/*!
 * @brief the open() handler for the SPI device node
 *
 * @param inode inode pointer
 * @param file  file pointer
 *
 * @return 0 if successfully opened.<BR>
 *         -EBUSY if all slots are in use.
 */

static int spi_open (struct inode *inode, struct file *file)
{
    unsigned int i;
    SPI_DATA_PARAMETER_T *params_p;

    /* Acquire the lock to ensure our exclusive access to the open_fds array */
    spin_lock (&global_lock);

    /* Find the first open slot in the array */
    for (i = 0; i < MAX_OPENS; i++)
    {
        if (open_fds[i].open == 0)
        {
            /* Claim the slot for this file descriptor */
            open_fds[i].open = 1;

            break;
        }
    }

    /* We are done, so release the lock */
    spin_unlock (&global_lock);

    /* If we could not find a free slot, return busy */
    if (i == MAX_OPENS)
    {
        return -EBUSY;
    }

    /* Save "i" in the file to remember which slot we are using */
    file->private_data = (void *)i;

    /* Allocate memory for scratch buffers */
    for (i = 0; i < (MOTO_SPI_MAX_VECTORS * 2); i++)
    {
        open_fds[(int)file->private_data].data_sz[i] = SCRATCH_BUFFER_SIZE;
        open_fds[(int)file->private_data].data[i] = kmalloc(SCRATCH_BUFFER_SIZE, GFP_KERNEL|__GFP_DMA);
        /* Start out with known scratch data. */
        memset(open_fds[(int)file->private_data].data[i], 0x00, SCRATCH_BUFFER_SIZE);
    }
    /* Set the filler character to a known value. */
    open_fds[(int)file->private_data].filler_byte = 0x00;
    
    /* Start out with an invalid chip select for error checking. */
    params_p = &open_fds[(int)file->private_data].params;
    params_p->cs = SPI_CS__END;
    
    /* Set up the parameters which user space cannot change. */
    params_p->nonblocking = false;
    params_p->xfer_complete_p = NULL;
    params_p->xfer_complete_param_p = NULL;
    return 0;
}

/*!
 * @brief the close() handler for the SPI device node
 *
 * This function implements the close() system call on the SPI device node.
 * This function releases the allocated slot in the open_fds array.
 *
 * @param inode inode pointer
 * @param file  file pointer
 *
 * @return 0
 */
 
static int spi_close (struct inode *inode, struct file *file)
{
    unsigned int i;
    
    /* If the chip select was locked, unlock it before closing */
    if (open_fds[(int)file->private_data].params.lock_cs == true)
    {
        /* Update the flag in the parameters */
        open_fds[(int)file->private_data].params.lock_cs = false;

        /* Send the request to unlock the chip select */
        spi_release_cs_lock(&(open_fds[(int)file->private_data].params)); 
    }

    /* Free the scratch memory buffers. */
    for (i = 0; i < (MOTO_SPI_MAX_VECTORS * 2); i++)
    {
        kfree(open_fds[(int)file->private_data].data[i]);
        open_fds[(int)file->private_data].data[i] = NULL;
    }
    
    /* Acquire the lock to ensure our exclusive access to the open_fds array */
    spin_lock (&global_lock);

    /* Release our slot in the array */
    open_fds[(int)file->private_data].open = 0;

    /* We are done, so release the lock */
    spin_unlock (&global_lock);

    return 0;
}

/*!
 * @brief the ioctl() handler for the SPI device node
 *
 * This function implements the ioctl() system call for the SPI device node.
 *
 * @param inode inode pointer
 * @param file  file pointer
 * @param cmd   the ioctl() command
 * @param arg   the ioctl() argument
 *
 * @return 0 if successful
 */

static int spi_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    static const SPI_CS_T usr_cs_to_kernel_cs[MOTO_SPI_CS__END] =
    {
        SPI_CS_TFLASH, /* MOTO_SPI_CS_TFLASH */
        SPI_CS_DUMMY   /* MOTO_SPI_CS_DUMMY */
    };
    static const MOTO_SPI_CS_T kernel_cs_to_usr_cs[SPI_CS__END+1] =
    {
        MOTO_SPI_CS__END,     /* SPI_CS_PCAP */
        MOTO_SPI_CS_TFLASH,   /* SPI_CS_TFLASH */
        MOTO_SPI_CS_DUMMY,    /* SPI_CS_DUMMY */
        MOTO_SPI_CS__END      /* SPI_CS__END */
    }; 
    int retval = 0;
    MOTO_SPI_TRANSCEIVE_T xcvdata;
    MOTO_SPI_PARAMETER_T spi_params;
    SPI_IOVEC_T vectors[MOTO_SPI_MAX_VECTORS];
    SPI_FDS_T *fds_p = &open_fds[(int)file->private_data];
    SPI_DATA_PARAMETER_T *params_p = &fds_p->params;
    int i;
    int j;
    

    switch (cmd)
    {
        case MOTO_SPI_IOCTL_GET_SPI_PARAMS:
            /* Copy the needed parameters from the kernel SPI parameters to the
               user space parameters. */
            spi_params.cs = kernel_cs_to_usr_cs[params_p->cs];
            spi_params.lock_cs = params_p->lock_cs;
            spi_params.clk_idle_state = params_p->clk_idle_state;
            spi_params.clk_phase = params_p->clk_phase;
            spi_params.speed = params_p->speed;
                
            /* Copy the settings to user space */
            retval = copy_to_user (
                (void *)arg,
                (void *)&spi_params,
                sizeof(spi_params));

            /* If the copy failed, return an error */
            if (retval != 0)
            {
                retval = -EFAULT; 
            }
            break;

        case MOTO_SPI_IOCTL_SET_SPI_PARAMS:
            /* Copy the new settings from user space */
            retval = copy_from_user (
                (void *)&spi_params,
                (void *)arg,
                sizeof(spi_params));

            /* If the copy failed, return an error */
            if (retval != 0)
            {
                return -EFAULT;
            }

            if (spi_params.cs >= MOTO_SPI_CS__END)
            {
                return -EINVAL;
            }
            
            /* Copy the user parameters to the kernel SPI parameters. */
            params_p->cs = usr_cs_to_kernel_cs[spi_params.cs];
            params_p->lock_cs = spi_params.lock_cs;
            params_p->clk_idle_state = spi_params.clk_idle_state;
            params_p->clk_phase = spi_params.clk_phase;
            params_p->speed = spi_params.speed;
            break;

        case MOTO_SPI_IOCTL_SET_FILLER:
            fds_p->filler_byte = (unsigned char)arg;
            break;

        case MOTO_SPI_IOCTL_TRANSCEIVE:
            /* If the parameters have not yet been initialized return an error. */
            if (params_p->cs == SPI_CS__END)
            {
                return -ENOTCONN;
            }
            /* Copy the transceive structure from user space */
            if (copy_from_user ((void *)&xcvdata, (void *)arg, sizeof(xcvdata)) != 0)
            {
                return -EFAULT;
            }

            /* Copy the data from the transceive structure into the SPI_IOVEC_T's */
            for (i = 0, j = 0; i < xcvdata.count; i++)
            {
                /* If the Tx data is already a kernel pointer, nothing to do */
                if ((xcvdata.vectors[i].tx_data_is_kernel_space) &&
                    (xcvdata.vectors[i].tx_data != NULL))
                {
                    vectors[i].tx_base = xcvdata.vectors[i].tx_data;
                }

                /* Else, Tx data must be a user-space pointer (or NULL) */
                else
                {
                    /* If the data is larger than the scratch buffer, increase the size
                       of the scratch buffer. */
                    if (xcvdata.vectors[i].size > fds_p->data_sz[j])
                    {
                        /* Set the size to 0 in case of an allocation error. */
                        fds_p->data_sz[j] = 0;
                        kfree(fds_p->data[j]);
                        fds_p->data[j] = kmalloc(xcvdata.vectors[i].size, GFP_KERNEL|__GFP_DMA);
                        if (fds_p->data[j] == NULL)
                        {
                            return -ENOMEM;
                        }
                        fds_p->data_sz[j] = xcvdata.vectors[i].size;
                    }

                    if (xcvdata.vectors[i].tx_data == NULL)
                    {
                        /* Fill the scratch buffer with the filler data */
                        memset(fds_p->data[j],
                            fds_p->filler_byte,
                            xcvdata.vectors[i].size);
                    }
                    else
                    {
                        /* Copy the data from user space into the next scratch buffer */
                        if (copy_from_user (fds_p->data[j], xcvdata.vectors[i].tx_data,
                                            xcvdata.vectors[i].size) != 0)
                        {
                            return -EFAULT;
                        }
                    }

                    /* Set the tx base address for this vector to the scratch buffer */
                    vectors[i].tx_base = fds_p->data[j];

                    /* Advance to the next scratch buffer */
                    j++;
                }

                /* If the Rx data is already a kernel pointer, nothing to do */
                if ((xcvdata.vectors[i].rx_data_is_kernel_space) &&
                    (xcvdata.vectors[i].rx_data != NULL))
                {
                    vectors[i].rx_base = xcvdata.vectors[i].rx_data;
                }

                /* Else, Rx data must be a user-space pointer (or NULL) */
                else
                {
                    /* If the data is larger than the scratch buffer, increase the size
                       of the scratch buffer. */
                    if (xcvdata.vectors[i].size > fds_p->data_sz[j])
                    {
                        /* Set the size to 0 in case of an allocation error. */
                        fds_p->data_sz[j] = 0;
                        kfree(fds_p->data[j]);
                        fds_p->data[j] = kmalloc(xcvdata.vectors[i].size,  GFP_KERNEL|__GFP_DMA);
                        if (fds_p->data[j] == NULL)
                        {
                            return -ENOMEM;
                        }
                        fds_p->data_sz[j] = xcvdata.vectors[i].size;
                    }

                    /* Set the rx base address for this vector to the scratch buffer */
                    vectors[i].rx_base = fds_p->data[j];

                    /* Advance to the next scratch buffer */
                    j++;
                }
                vectors[i].iov_len = xcvdata.vectors[i].size;
            }

            /* Perform the transfer(s).  This will block until the transfers are complete. */
            retval = spi_transceivev (
                params_p,
                vectors,
                xcvdata.count);

            /* If the transfer was successful, see if we need to copy any data */
            if (retval < 0)
            {
                return retval;
            }
            /* Copy any Rx data back into the user space buffers */
            for (i = 0, j = 0; i < xcvdata.count; i++)
            {
                /* Check to see if the Tx data would have taken a scratch buffer */
                if ((!xcvdata.vectors[i].tx_data_is_kernel_space) ||
                    (xcvdata.vectors[i].tx_data == NULL))
                {
                    /* Tx data was in scratch buffer, so advance to next buffer */
                    j++;
                }

                /* If the Rx data was not in kernel space, copy the data back */
                if ((!xcvdata.vectors[i].rx_data_is_kernel_space) ||
                    (xcvdata.vectors[i].rx_data == NULL))
                {
                    /* If Rx data pointer is non-NULL, need to copy data back to user */
                    if (xcvdata.vectors[i].rx_data != NULL)
                    {
                        if (copy_to_user (xcvdata.vectors[i].rx_data, fds_p->data[j],
                                          xcvdata.vectors[i].size) != 0)
                        {
                            return -EFAULT;
                        }
                    }   
                    /* Advance to the next scratch buffer */
                    j++;
                }
            }
            break;

        case MOTO_SPI_IOCTL_LOCK:
            /* If the parameters have not yet been initialized return an error. */
            if (params_p->cs == SPI_CS__END)
            {
                return -ENOTCONN;
            }
            /* If the argument is 0, the request is to unlock the chip select */
            if (arg == 0 && params_p->lock_cs == true)
            {
                /* Update the flag in the parameters */
                params_p->lock_cs = false;

                /* Send the request to unlock the chip select */
                spi_release_cs_lock(params_p);
            }
            else
            {
                /* Update the flag in the parameters */
                params_p->lock_cs = (arg != 0) ? true : false;
            }
            break;

        default:
            return -ENOTTY;
            break;
    }
    return 0;
}

/*! This structure defines the file operations for the SPI device */
static struct file_operations spi_fops =
{
    .owner =    THIS_MODULE, 
    .ioctl =    spi_ioctl,
    .open =     spi_open,
    .release =  spi_close
};

/******************************************************************************
* Global Functions
******************************************************************************/

/*!
 * @brief SPI module initialization function
 *
 * This function performs the initialization of the data structures for the
 * SPI driver.  Currently, this just involves registering our character
 * device.
 *
 * @return 0
 */

void __init moto_spi_init (void)
{
    int ret;

    /* Register our character device */
    ret = register_chrdev(MOTO_SPI_DRIVER_MAJOR_NUM, MOTO_SPI_DRIVER_DEV_NAME, &spi_fops);

    /* Display a message if the registration fails */
    if (ret < 0)
    {
        tracemsg(_k_d("unable to get a major (%d) for SPI driver: %d"),
            (int)MOTO_SPI_DRIVER_MAJOR_NUM, ret);
    }
}

/*!
 * @brief SPI module cleanup function
 *
 * This function is called when the power IC driver is being destroyed (which
 * generally never happens).  Its purpose is to unregister the character special
 * device that was registered when the SPI module was initialized.
 */

void __exit moto_spi_destroy (void)
{
    unsigned int i;
    unsigned int j;
    
    /* Free any memory which is allocated to scratch buffers. */
    for (i = 0; i < MAX_OPENS; i++)
    {
        if (open_fds[i].open != 0)
        {
            for (j = 0; j < (MOTO_SPI_MAX_VECTORS * 2); j++)
            {
                kfree(open_fds[i].data[j]);
                open_fds[i].data[j] = NULL;
            }
        }
    }
    unregister_chrdev(MOTO_SPI_DRIVER_MAJOR_NUM, MOTO_SPI_DRIVER_DEV_NAME);
}
