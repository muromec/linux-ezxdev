/*
 * File: sbus_old_i2c.h
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
#ifndef _CAMERA_I2C_H_
#define _CAMERA_I2C_H_

#define I2C_BASE_ADDR		0xFFFB3800

#define            I2C_DEVICE_REG_OFFSET                  0x00
#define            I2C_ADDRESS_REG_OFFSET                 0x04
#define            I2C_DATA_WRITE_REG_OFFSET              0x08
#define            I2C_DATA_READ_REG_OFFSET               0x0C
#define            I2C_CMD_REG_OFFSET                     0x10
#define            I2C_CONF_FIFO_REG_OFFSET               0x14
#define            I2C_CONF_CLK_REG_OFFSET                0x18
#define            I2C_CONF_CLK_REF_REG_OFFSET            0x1C
#define            I2C_STATUS_FIFO_REG_OFFSET             0x20
#define            I2C_STATUS_ACTIVITY_REG_OFFSET         0x24


#define I2C_DEVICE_REG_VAL  (*(volatile unsigned char *)(I2C_BASE_ADDR+I2C_DEVICE_REG_OFFSET))
#define I2C_ADDRESS_REG_VAL  (*(volatile unsigned char *)(I2C_BASE_ADDR+I2C_ADDRESS_REG_OFFSET))
#define I2C_DATA_WRITE_REG_VAL  (*(volatile unsigned char *)(I2C_BASE_ADDR+I2C_DATA_WRITE_REG_OFFSET))
#define I2C_DATA_READ_REG_VAL  (*(volatile unsigned char *)(I2C_BASE_ADDR+I2C_DATA_READ_REG_OFFSET))
#define I2C_CMD_REG_VAL  (*(volatile unsigned char *)(I2C_BASE_ADDR+I2C_CMD_REG_OFFSET))
#define I2C_CONF_FIFO_REG_VAL  (*(volatile unsigned char *)(I2C_BASE_ADDR+I2C_CONF_FIFO_REG_OFFSET))
#define I2C_CONF_CLK_REG_VAL  (*(volatile unsigned char *)(I2C_BASE_ADDR+I2C_CONF_CLK_REG_OFFSET))
#define I2C_CONF_CLK_REF_REG_VAL  (*(volatile unsigned char *)(I2C_BASE_ADDR+I2C_CONF_CLK_REF_REG_OFFSET))
#define I2C_STATUS_FIFO_REG_VAL (*(volatile unsigned char *)(I2C_BASE_ADDR+I2C_STATUS_FIFO_REG_OFFSET))
#define I2C_STATUS_ACTIVITY_REG_VAL  (*(volatile unsigned char *)(I2C_BASE_ADDR+I2C_STATUS_ACTIVITY_REG_OFFSET))

#define I2C_DEVICE(x) (I2C_DEVICE_REG_VAL) = ((x) | (I2C_DEVICE_REG_VAL & 0x00))
#define I2C_ADDRESS(x) (I2C_ADDRESS_REG_VAL) = (x)


#define I2C_IRQ_MASK_ENABLE (I2C_CMD_REG_VAL |= 0x20)
#define I2C_IRQ_MASK_DISABLE (I2C_CMD_REG_VAL &= ~0x20)

#define I2C_FIFO_SIZE (I2C_CONF_FIFO_REG_VAL &= 0x0F)

#define I2C_SOFT_RESET(x) (I2C_CMD_REG_VAL = ((x & 0x01) | (I2C_CMD_REG_VAL & ~0x01)))

#define I2C_SIMPLE_READ (I2C_CMD_REG_VAL &= ~0x10)
#define I2C_COMBINED_READ (I2C_CMD_REG_VAL |= 0x10)

#define I2C_READ (I2C_CMD_REG_VAL |= 0x08)
#define I2C_WRITE (I2C_CMD_REG_VAL &= ~0x08)

#define I2C_START (I2C_CMD_REG_VAL |= 0x04)
#define I2C_FIFO_FULL (I2C_STATUS_FIFO_REG_VAL &= 0x0F)

#define I2C_CLOCK_ENABLE	(I2C_CMD_REG_VAL |=  0x02)
#define I2C_CLOCK_DISABLE	(I2C_CMD_REG_VAL &= ~0x02)

#define TX_WAIT_TIME 0x008F

#endif



