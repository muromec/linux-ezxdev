/*
 * Copyright 2004 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2004-2005 Motorola, Inc. All Rights Reserved.
 */

/* 
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
 * Motorola 2004-Dec-06 - Design the Low Level interface to Funlight IC.
 */

/*!
 * @file fl_register.c
 * 
 * @ingroup poweric_core
 *
 * @brief Low-level interface to the Funlight registers
 *
 * Using the connection of the I2C bus to the processor, the Funlight Chip, LP3944 can be accessed.  
 * In order for this to occur, a couple things must first happen. 
 * 
 *    - The I2C driver must be registered for the Funlight Chip to use.  
 *    - We must then wait for the I2C adaptor to be registered. 
 *    - An I2C client has to be registered, which will contain the driver and the adapter.
 *
 * Once all of this is complete, the I2C functions can be used to read, write and transfer data from
 * the Funlight Chip.
 *
 * The Funlight Chip has 10 registers, three of which will never be used.  Register 1, 8, and 9 are
 * empty and non-functional registers.  Register 1 is read-only and has all the bits hard-wired to 
 * zero.  Registers 8 and 9 can be written and read, but the contents does not have any effect on the
 * operation of the Funlight Chip.
 * 
 * The registers that are used on the Funlight Chip are as follows, along with the LED controls:
 *
 * REGISTERS
 *    - Input 1 (Read Only) - LED0-7 input register
 *    - PSC0    (R/W)       - Frequency Prescaler 0...used to program the period of DIM0
 *    - PWM0    (R/W)       - PWM Register 0...determines the duty cycle of DIM0 
 *    - PSC1    (R/W)       - Frequency Prescaler 1...used to program the period of DIM1
 *    - PWM1    (R/W)       - PWM Register 1...determines the duty cycle of DIM1
 *    - LS0     (R/W)       - LED0-3 Selector
 *    - LS1     (R/W)       - LED4-7 Selector
 *
 * LED Control
 *    - LED0 (bits 0 and 1) controls the Red LED
 *    - LED1 (bits 2 and 3) controls the Green LED
 *    - LED2 (bits 4 and 5) controls the Blue LED
 *    - LED3 (bits 6 and 7) controls the CLI backlight
 *    - LED4 (bits 0 and 1) controls the Main Display backlight
 *         
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/init.h>

#include "os_independent.h"
#include "fl_register.h"

/******************************************************************************
* Constants
******************************************************************************/

/*! The size (in bytes) of the register address */
#define FL_REG_ADDR_SIZE 1

/*! The size (in bytes) of the register value */
#define FL_REG_DATA_SIZE 1

/*! The I2C client structure obtained for communication with the Funlight Chip */
static struct i2c_client *fl_i2c_client = NULL;

/*! A pointer to a function to be called when communication between the I2C and Funlight Chip is available*/
static void (*fl_reg_init)(void) = NULL;

/******************************************************************************
* Local function prototypes
******************************************************************************/
/* Function prototype to detach the client when finished with communication */
static int client_detach(struct i2c_client *client);

/******************************************************************************
* Local Functions
******************************************************************************/

 /*!
 * @brief Handles the addition of an I2C adapter capable of supporting the Funlight Chip
 *
 * This function is called by the I2C driver when an adapter has been added to the
 * system that can support communications with the Funlight Chip.  The function will
 * attempt to register a I2C client structure with the adapter so that communication
 * can start.  If the client is successfully registered, the Funlight initialization
 * function will be called to do any register writes required at power-up.
 *
 * @param    adap   A pointer to I2C adapter 
 *
 * @return   This function returns 0 if successful
 */
static int adapter_attach (struct i2c_adapter *adap)
{
    int retval;

    tracemsg(_k_d("in adapter_attach"));

    /* Allocate memory for client structure */
    fl_i2c_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);
    if(fl_i2c_client == NULL)
    {
        return -ENOMEM;
    }

    /* Fill in the required fields */
    fl_i2c_client->adapter = adap;
    fl_i2c_client->addr = 0x60;

    /* Register our client */
    retval = i2c_attach_client(fl_i2c_client);
    if(retval != 0)
    {
        /* Request failed, free the memory that we allocated */
        kfree(fl_i2c_client);
        fl_i2c_client = NULL;
    }
    /* else we are registered */
    else
    {
        tracemsg(_k_d("successfully attached our client"));

        /* Initialize Funlight registers now */
        if(fl_reg_init != NULL)
        {
            (*fl_reg_init)();
        }
    }

    return retval;
}

 /*!
 * @brief Handles the removal of the I2C adapter being used for the Funlight Chip
 *
 * This function call is an indication that our I2C client is being disconnected
 * because the I2C adapter has been removed.  Calling the i2c_detach_client() will
 * make sure that the client is destroyed properly.
 *
 * @param   client   Pointer to the I2C client that is being disconnected
 *
 * @return  This function returns 0 if successful
 */
static int client_detach(struct i2c_client *client)
{
    return i2c_detach_client(client);
}

/* Creation of the i2c_driver for Funlights */ 
static const struct i2c_driver driver = {
    name:           "FunLight I2C Driver",
    flags:          I2C_DF_NOTIFY,
    attach_adapter: adapter_attach,
    detach_client:  client_detach,
};


/******************************************************************************
* Global Functions
******************************************************************************/

/*!
 * @brief Initializes communication with the Funlight Chip
 *
 * This function performs any initialization required for the Funlight Chip.  It
 * starts the process mentioned above for initiating I2C communication with the Funlight
 * Chip by registering our I2C driver.  The rest of the process is handled
 * through the callback function adapter_attach().
 *
 * @param reg_init_fcn  Pointer to a function to call when communications are available
 */
void __init fl_initialize(void (*reg_init_fcn)(void))
{
    tracemsg(_k_d("in fl_initialize"));
    
    /* Save the register initialization function for later */
    fl_reg_init = reg_init_fcn;

    /* Register our driver */
    i2c_add_driver((struct i2c_driver *)&driver);
}

/*!
 * @brief Reads a specific Funlight Chip Register
 *
 * This function implements a read of a given Funlight register.  The read is done using i2c_transfer
 * which is a set of transactions.  The first transaction will be an I2C write of the register
 * number that is going to next be read.  The second transaction will be an I2C read which will read
 * the contents of the register previously written from the Funlight Chip.  
 *
 * Originally the read was going to be done using the I2C commands i2c_master_send() to write and then 
 * i2c_master_recv() to read, but this was found to be incorrect.  These two transactions both send STOP
 * bits, whereas the funlight timing needs a RESTART in between the write and read transactions.  The
 * i2c_transfer is what gives us this functionality.
 *
 * @param reg        Funlight register number
 * @param reg_value  Pointer to which the read data should be stored
 *
 * @return This function returns 0 if successful
 */
int fl_reg_read (int reg, unsigned int *reg_value)
{

    unsigned char reg_num = reg;
    unsigned char value[FL_REG_DATA_SIZE];
    int retval;
    
    struct i2c_msg msgs[2] = 
    {
	    { 0, I2C_M_WR, 1, &reg_num },
	    { 0, I2C_M_RD, 1, value    }
    };

    tracemsg(_k_d("in fl_reg_read"));

    /* Fail if we weren't able to initialize (yet) */
    if (fl_i2c_client == NULL)
    {
        tracemsg(_k_d("fl_reg_read: not initialized?"));
        return -EINVAL;
    }

    msgs[0].addr = msgs[1].addr = fl_i2c_client->addr;
    tracemsg(_k_d("Writing register number %2d to the funlight"), reg_num);

    retval = i2c_transfer(fl_i2c_client->adapter,msgs,2);
    if (retval >= 0) 
    {
       tracemsg(_k_d("fl_reg_read: i2c_transfer successful, reading register %2d with 0x%02x"),reg_num,
       *(msgs[1].buf));
       *reg_value = *(msgs[1].buf);
       retval = 0;
    }
    
    return retval;
}

 /*!
 * @brief Writes a value to a specified Funlight register
 *
 * This function implements a write to a specified Funlight register.  The write is accomplished
 * by sending the register number and the new contents to the i2c function i2c_master_send.
 *
 * @param reg        Funlight register number
 * @param reg_value  Register value to write
 *
 * @return This function returns 0 if successful
 */
int fl_reg_write(int reg, unsigned int reg_value)
{
    unsigned char value[FL_REG_ADDR_SIZE + FL_REG_DATA_SIZE];
    int retval;

    tracemsg(_k_d("in fl_reg_write"));

    /* Fail if we weren't able to initialize */
    if(fl_i2c_client == NULL)
    {
        tracemsg(_k_d("fl_reg_write: not initialized?"));
        return -EINVAL;
    }

    /*Copy the data into a buffer for correct format */
    value[0] = reg;
    value[1] = reg_value;

    /* Write the data to the FL */
    retval = i2c_master_send(fl_i2c_client, value, FL_REG_ADDR_SIZE + FL_REG_DATA_SIZE);

    if(retval < 0)
    {
        tracemsg(_k_d("fl_reg_write: i2c_master_send failed: %d"), retval);
    }
    else
    {
    retval = 0;
    }

    return retval;
}
