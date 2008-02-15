/*
 * Copyright 2004-2005 Motorola, Inc.
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
 */

/*!
 * @file light_sensor.h
 *
 * @brief Global symbol definitions for the light sensor interface.
 */

/* 
 * Created by Mu, C.L. (w20596@motorola.com)
 */

#ifndef _LINUX_LIGHT_SENSOR_H_
#define _LINUX_LIGHT_SENSOR_H_

/* ioctl request to enable light sensor device to start luminance measure */
#define LIGHT_SENSOR_START      1

/* ioctl request to diable light sensor device to stop luminance measure */
#define LIGHT_SENSOR_STOP       2

/* ioctl request to get luminance value from light sensor device */
#define LIGHT_SENSOR_GET_LUMA   3

/*
 * Define the struct for the user ioctl parameter to return the luminance value
 */
struct light_sensor_luminance {
    int integer;    // integer part of luminance lux
    int decimal;    // decimal part of luminance lux
};

#endif //_LINUX_LIGHT_SENSOR_H_

