/*
 * File: sbus_old_i2c.c
 *
 * Description:
 *   OMAP1510 old I2c mode implementation of Camera Serial Bus.
 *
 * Author: TI, Inc.
 * Created 2002, Copyright (C) 2002 Texas Instruments  All rights reserved.
 * Copyright (C) 2003 MontaVista Software, Inc.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */
#include <linux/config.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/system.h>
#define MODULE_NAME "cami2c"
#include "common.h"
#include "camif.h"
#include "sbus_old_i2c.h"

static struct camera_serial_bus * this;

#define I2CIRQnum 36

static volatile unsigned int i2cStatus = 0;
static volatile unsigned int i2cIT = 0;
static volatile unsigned int i2cBusFree = 0;

static int i2c_int = 0;

enum I2C_Bus_Status { I2C_IDLE, I2C_BUSY };

#define BUS_SPEED  100  //kHz

static void i2c_handle_interrupt(int irq, void *client_data,
				 struct pt_regs *regs);

static int i2c_setFifoDepth(unsigned int depth)
{
	if (!(depth == 0 || depth > 16))
 		I2C_CONF_FIFO_REG_VAL = (depth - 1);
	return 0;
}

static int i2c_setBusSpeed(unsigned int bus)
{
	I2C_IRQ_MASK_DISABLE;
	I2C_CLOCK_DISABLE;
	
	I2C_CONF_CLK_REF_REG_VAL = 0x089;
	if (bus == 400)
		I2C_CONF_CLK_REG_VAL = 0xD0;
	else if (bus == 100)
		I2C_CONF_CLK_REG_VAL = 0xD2;
	else
		return 0;
	I2C_CLOCK_ENABLE;

	I2C_SOFT_RESET(1);
	I2C_SOFT_RESET(0);
	i2c_setFifoDepth(1);
	return 0;
}

static int i2c_configure(void)
{
	int status;
	ENTRY();

	this = &camera_sbus_old_i2c;

	/* make sure we're using the "old" 1509-compatible mode */
	outl(inl(MOD_CONF_CTRL_0) | (1<<16), MOD_CONF_CTRL_0);

	i2c_setBusSpeed(BUS_SPEED);

	if (!i2c_int) {
		status = request_irq(I2CIRQnum, &i2c_handle_interrupt,
				     SA_INTERRUPT, "camera i2c", NULL);

		if (status < 0) {
			err("Error; request for irq %d denied: %d.\n",
			    I2CIRQnum, status);
			return 1;
		} else {
			dbg("Request for irq %d granted.\n", I2CIRQnum);
			i2c_int = I2CIRQnum;
		}
	}
	
	i2cIT = 0;
	i2cBusFree = I2C_IDLE;
	i2cStatus = 0;
	I2C_IRQ_MASK_ENABLE;

	return 0;
}

// interrupt handler
static void i2c_handle_interrupt(int irq, void *client_data,
				 struct pt_regs *regs)
{
	unsigned char result;

	i2cIT++;
	// Read and clear status
	result = I2C_STATUS_ACTIVITY_REG_VAL;

	// hw bug
	if (!(result & 0x08) ) {
		// I2C Interrupt indicator not set, but here we are in the ISR.  Read status again.
		err("Error: I2C interrupt indicator not set\n");
		result = I2C_STATUS_ACTIVITY_REG_VAL;
	}

	i2cBusFree = I2C_IDLE;

	if (result & 0x02) {
		// Device Error
		err("Device Error\n");
	} else if (result & 0x01) {
		// Data Error
		err("Data Error\n");
	}
	i2cStatus = (unsigned int)(result & 0x3);
}

static int i2c_camera_set(unsigned char reg,
			  unsigned char *data,
			  unsigned int numBytes)
{
	unsigned char i;
	if (numBytes <= 16) {
 		I2C_SOFT_RESET(1);
 		I2C_SOFT_RESET(0);
		i2c_setFifoDepth(numBytes);
		// Fill the FIFO
		for(i=0;i<numBytes;i++)
			I2C_DATA_WRITE_REG_VAL = data[i];
	}
	I2C_WRITE;
	I2C_DEVICE(this->dev_id >> 1);
	I2C_ADDRESS(reg);
	i2cBusFree = I2C_BUSY;
	I2C_START;
	return 0;
}


static void i2c_waitBusFree(void)
{
	unsigned int timeout=0x4ffffff;
	while(--timeout)
		if (i2cBusFree == I2C_IDLE) {
			return;
		}
	err("Timeout waiting for I2C_IDLE\n");
	return;  // timeout
}


static int i2c_set_devid(int id)
{
	this->dev_id = id;
	return 0;
}

static int i2c_write_verify(unsigned char subaddr, unsigned char data)
{
	int retry=5;

	// There is a really obvious race condition that exists with
	// i2cStatus.  Since it is modified by the ISR, if this code
	// is ever changed to send multiple bytes of data or does
	// not wait for the bus to free, the error reporting mechanism
	// must also change.
	do {
		i2c_camera_set(subaddr, &data, sizeof(data));
		i2c_waitBusFree();
		if (retry && i2cStatus)
			dbg("RETRY: %d, i2cStatus == %d\n",retry,i2cStatus);
	} while (--retry && i2cStatus);
	return i2cStatus ? -EIO : 0;
}

static int i2c_write(u8 addr, u8* buf, int len)
{
	int ret = 0;
	while (len--) {
		if ((ret = i2c_write_verify(addr++, *buf++)))
			return ret;
	}

	return ret;
}

static int i2c_read(u8 addr, u8* buf, int len)
{
	return -ENOSYS;
}

static void i2c_close(void)
{
	I2C_IRQ_MASK_DISABLE;
	I2C_CLOCK_DISABLE;
	I2C_SOFT_RESET(1);
	if (i2c_int) {
		free_irq(i2c_int, NULL);
	}
	i2c_int = 0;
}


#ifdef I2C_DUMP_REGS
static int i2c_dump_regs()
{
	dbg("Timeout waiting for bus to free\n");
	dbg("I2C_CMD_REG             == 0x%02x\n", I2C_CMD_REG_VAL);
	dbg("I2C_CONF_FIFO_REG       == 0x%02x\n", I2C_CONF_FIFO_REG_VAL);
	dbg("I2C_CONF_CLK_REG        == 0x%02x\n", I2C_CONF_CLK_REG_VAL);
	dbg("I2C_CONF_CLK_REF_REG    == 0x%02x\n", I2C_CONF_CLK_REF_REG_VAL);
	dbg("I2C_STATUS_FIFO_REG     == 0x%02x\n", I2C_STATUS_FIFO_REG_VAL);
	dbg("I2C_STATUS_ACTIVITY_REG == 0x%02x\n", I2C_STATUS_ACTIVITY_REG_VAL);
	dbg("\nIRQ L1 Mask == 0x%04x\n", *((volatile int *)0xFFFECB04));
	dbg("IRQ L2 Mask == 0x%04x\n", *((volatile int *)0xFFFE0004));
	dbg("IRQ36_ILR   == 0x%04x\n", *((volatile int *)0xFFFE002C));
	return 0;
}
#endif


struct camera_serial_bus camera_sbus_old_i2c = {
	init:         i2c_configure,
	cleanup:      i2c_close,
	set_devid:    i2c_set_devid,
	read:         i2c_read,
	write:        i2c_write,
	write_verify: i2c_write_verify,
};
