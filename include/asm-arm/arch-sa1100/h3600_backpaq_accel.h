/*
*
* Driver for the Compaq iPAQ Mercury Backpaq accelerometer
*
* Copyright 2001 Compaq Computer Corporation.
*
* Use consistent with the GNU GPL is permitted,
* provided that this copyright notice is
* preserved in its entirety in all copies and derived works.
*
* COMPAQ COMPUTER CORPORATION MAKES NO WARRANTIES, EXPRESSED OR IMPLIED,
* AS TO THE USEFULNESS OR CORRECTNESS OF THIS CODE OR ITS
* FITNESS FOR ANY PARTICULAR PURPOSE.
*                   
*       Acceleration directions are as follows:
*
*             < 0 >          <=== Camera
*     +Y <-  /-----\
*            |     |         <=== iPAQ
*            |     |
*            |     |
*            |     |
*            \-----/
* 
*             | +X
*             V
*
* Author: Andrew Christian
*         <andrew.christian@compaq.com>
 */

#ifndef _H3600_BACKPAQ_ACCEL_H
#define _H3600_BACKPAQ_ACCEL_H

#include <linux/ioctl.h>

/* 
   Return scaled 16 bit values in gravities:
   8 bits of integer, 8 bits of fraction

   This structure is returned for each read of the device
*/

typedef short          s16_scaled;    /* 8 bits of integer, 8 bits of fraction */
typedef unsigned short u16_fraction;    /* 16 bits of fraction */

/* Format of data returned from a read() request */
struct h3600_backpaq_accel_data {
 	s16_scaled x_acceleration;
	s16_scaled y_acceleration;
};

/* Raw timing data, returned from an ioctl() */
struct h3600_backpaq_accel_raw_data {
	unsigned short xt1;
	unsigned short xt2;
	unsigned short yt1;
	unsigned short yt2;
};

/* Acceleration parameters, set using the EEPROM interface */
struct h3600_backpaq_accel_params {
	u16_fraction x_offset;   /* 16-bit fraction (i.e., 0x8000 = 0.5) */
	u16_fraction x_scale;
	u16_fraction y_offset;
	u16_fraction y_scale;
};

/*
  Ioctl interface
 */

#define H3600_ACCEL_MAGIC    0xD0      /* Picked at random - see Documentation/ioctl-number.txt */

#define H3600_ACCEL_G_PARAMS   _IOR( H3600_ACCEL_MAGIC, 0, struct h3600_backpaq_accel_params)
#define H3600_ACCEL_G_RAW      _IOR( H3600_ACCEL_MAGIC, 2, struct h3600_backpaq_accel_raw_data)

#endif /*  _H3600_BACKPAQ_ACCEL_H */

