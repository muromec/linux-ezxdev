/*
 * Copyright 2004 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2004 - 2006 Motorola, Inc. All Rights Reserved.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 * Motorola 2004-Dec-06 - Design the  Low level interface
 *                        to EMU One Chip IC.
 *
 * Motorola 2006-Jan-10 - Finalize the design for the EMU one chip 
 *                        interface. 
 */

/*!
 * @file eoc_register.c
 *
 * @ingroup poweric_core
 *
 * @brief Low-level interface to EMU One Chip registers
 *
 * The communication to the EMU One Chip is done through the I2C bus connected
 * to the processor.  To use the I2C bus, the following needs to happen:
 *
 *    - Register the I2C driver for the EMU One Chip
 *    - Wait for an I2C adapter to be registered
 *    - Register the I2C client for the EMU One Chip.  The client is considered
 *      a combination of a driver and adapter.
 *
 * Once those things are finished, the driver can use the standard I2C functions
 * to send and receive data from the EMU One Chip.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/init.h>

#include "os_independent.h"
#include "eoc_register.h"

/******************************************************************************
* Constants
******************************************************************************/

/*! The size (in bytes) of the register address */
#define EOC_REG_ADDR_SIZE  1

/*! The size (in bytes) of the register value */
#define EOC_REG_DATA_SIZE  3

/*! Number of times reads/writes will be retried on failure */
#define MAX_RETRIES        3

/*! Number of different addresses from the various versions of the EOC */
#define EOC_I2C_ADDR_ARRAY_SIZE   2

/******************************************************************************
* Local Variables
******************************************************************************/

/*! The I2C client structure obtained for communication with the EMU One Chip */
static struct i2c_client *eoc_i2c_client = NULL;

/*! A pointer to a function to be called once the I2C communication to EMU One Chip is available */
static void (*eoc_reg_init)(void) = NULL;

/*! Array of the current valid I2C addresses the most resent is listed first */
static const unsigned int eoc_i2c_valid_addrs [EOC_I2C_ADDR_ARRAY_SIZE] = 
{ 
    0x17,     /* Address for version 2.0 and above  */
    0x7C      /* Address for versions prior too 2.0 */
};


/******************************************************************************
* Local Functions
******************************************************************************/

/*!
 * @brief Handles the removal of the I2C adapter being used for the EMU One Chip
 *
 * This function call is an indication that our I2C client is being disconnected
 * because the I2C adapter has been removed.  The only thing that can be done here
 * is to make sure that our client is destroyed properly by calling the
 * i2c_detach_client() function.
 *
 * @param client Pointer to the I2C client that is being disconnected
 *
 * @return 0 if successful
 */

static int client_detach (struct i2c_client *client)
{
    int retval;

    retval = i2c_detach_client(client);

    kfree(client);

    return retval;
}

/*!
 * @brief Handles the addition of an I2C adapter capable of supporting the EMU One Chip
 *
 * This function is called by the I2C driver when an adapter has been added to the
 * system that can support communications with the EMU One Chip.  The function will
 * attempt to register a I2C client strucure with the adapter so that communication
 * with the EMU One Chip can start.  If the client is successfully registered, the
 * EMU One Chip register initialization function will be called (through the function
 * pointer, #eoc_reg_init) to do any register writes required at power-up.
 *
 * In practice, we see this adapter attach function called immediately after the EMU
 * One Chip I2C driver is registered.  This happens because the adapter gets registered
 * before our driver.  If the adapter were to be registered after our driver, then
 * the adapater attach function would not be called until after the adapter was
 * registered.
 *
 * @param adap A pointer to the new I2C adapter 
 *
 * @return 0 if successful
 */

static int adapter_attach (struct i2c_adapter *adap)
{
    int retval;
    int entry = 0;
    int storage;

    /* Allocate memory for our client structure */
    eoc_i2c_client = kmalloc (sizeof (struct i2c_client), GFP_KERNEL);
    if (eoc_i2c_client == NULL)
    {
        return -ENOMEM;
    }

    /* Fill in the required fields */
    eoc_i2c_client->adapter = adap;
    eoc_i2c_client->addr = eoc_i2c_valid_addrs[0];
 
    /* Register our client starting with most recent valid EOC addr then trying older versions */
    retval = i2c_attach_client(eoc_i2c_client);
       
    if (retval != 0)
    {
        /* Request failed, free the memory that we allocated */
        kfree(eoc_i2c_client);
        eoc_i2c_client = NULL;
    }

    /* Else, we succesfully registered */
    else
    {
        
        /* The following loop is used to determine which address should be used to initialize 
	 * the EMU One Chip.  The addresses are checked starting with the most recent one and proceding to 
	 * the oldest valid address until a valid one is found.  In the case that no valid address is 
	 * found, which should not happen, the client will be detached.   
         */
	 
	do
	{
	    eoc_i2c_client->addr = eoc_i2c_valid_addrs[entry];
	    retval = eoc_reg_read (0, &storage);
	    entry++;
        }
        while  (retval != 0 && (entry < EOC_I2C_ADDR_ARRAY_SIZE));
	     
        if (retval != 0)
	{
	    retval = client_detach(eoc_i2c_client);
            eoc_i2c_client = NULL;     
            tracemsg(_k_d("eoc address check failed" ));
	}
        
	else
	{
	    tracemsg(_k_d("After eoc address check: i2c_addr: %d " ), eoc_i2c_valid_addrs[entry - 1]);
	
            if (eoc_reg_init != NULL)
            {     
	        /* Initialize the EOC registers now that the adapter is available */
                (*eoc_reg_init)();
	    }        
        }
    }
    return retval;
}

/*! Structure defining the EMU One Chip I2C driver */
static const struct i2c_driver driver = {
	name:            "EMU One Chip I2C driver",
	flags:           I2C_DF_NOTIFY,
	attach_adapter:  adapter_attach,        
	detach_client:   client_detach,
};

/******************************************************************************
* Global Functions
******************************************************************************/

/*!
 * @brief Initializes communication with the EMU One Chip
 *
 * This function performs any initialization required for the EMU One Chip.  It
 * starts the process mentioned above for initiating I2C communication with the EMU
 * One Chip by registering our I2C driver.  The rest of the process is handled
 * through the callback function adapter_attach().
 *
 * @param reg_init_fcn Pointer to a function to call when communications are available
 */

void __init eoc_initialize (void (*reg_init_fcn)(void))
{
    /* Save the register initialization function for later */
    eoc_reg_init = reg_init_fcn;

    /* Register our driver */
    i2c_add_driver((struct i2c_driver *)&driver);
}

/*!
 * @brief Reads an EMU One Chip register
 *
 * This function implements a read of a given EMU One Chip register.  The read
 * of a register is implemented in two steps:
 *
 *    - An I2C write transaction to write the register number to read to the EMU One Chip
 *    - An I2C read transaction to read the register's value from the EMU One Chip
 *
 * @param reg EMU One Chip register number
 * @param reg_value Pointer to which the read data should be stored
 *
 * @return 0 if successful
 */

int eoc_reg_read (int reg, unsigned int *reg_value)
{
    unsigned char reg_num = reg;
    unsigned char value[EOC_REG_DATA_SIZE];
    int retries = 0;
    int retval;

    struct i2c_msg msgs[2] =
    {
        { eoc_i2c_client->addr, I2C_M_WR, EOC_REG_ADDR_SIZE, &reg_num },
        { eoc_i2c_client->addr, I2C_M_RD, EOC_REG_DATA_SIZE, value }
    };

    /* Fail if we weren't able to initialize (yet) */
    if (eoc_i2c_client == NULL)
    {
        tracemsg(_k_d("eoc_reg_read: not initialized?"));
        return -EINVAL;
    }

    do
    {
        /* Perform the transfer */
        retval = i2c_transfer(eoc_i2c_client->adapter, msgs, 2);
        if (retval >= 0)
        {
            *reg_value  = (value[2] <<  0);
            *reg_value |= (value[1] <<  8);
            *reg_value |= (value[0] << 16);

            retval = 0;

            break;
        }
        else
        {
            tracemsg(_k_d("eoc_reg_read: i2c_transfer failed: %d"), retval);
            retries++;
        }
    } while (retries < MAX_RETRIES);

    return retval;
}

/*!
 * @brief Writes an EMU One Chip register
 *
 * This function implements a write to a given EMU One Chip register.  Unlike the read,
 * the write of a register can be done in a single transaction.  The transaction
 * includes both the register number and the new contents of the register.
 *
 * @param reg EMU One Chip register number
 * @param reg_value Register value to write
 *
 * @return 0 if successful
 */

int eoc_reg_write (int reg, unsigned int reg_value)
{
    unsigned char value[EOC_REG_ADDR_SIZE + EOC_REG_DATA_SIZE];
    int retries = 0;
    int retval;

    /* Fail if we weren't able to initialize (yet) */
    if (eoc_i2c_client == NULL)
    {
        tracemsg(_k_d("eoc_reg_write: not initialized?"));
        return -EINVAL;
    }

    /* Copy the data into a buffer into the correct format */
    value[0] = reg;
    value[1] = (reg_value >> 16) & 0xFF;
    value[2] = (reg_value >>  8) & 0xFF;
    value[3] = (reg_value >>  0) & 0xFF;

    do
    {
        /* Write the data to the EOC */
        retval = i2c_master_send (eoc_i2c_client, value, EOC_REG_ADDR_SIZE + EOC_REG_DATA_SIZE);
        if (retval < 0)
        {
            tracemsg(_k_d("eoc_reg_write: i2c_master_send failed: %d"), retval);
            retries++;
        }
        else
        {
            retval = 0;
            break;
        }
    } while (retries < MAX_RETRIES);

    return retval;
}
