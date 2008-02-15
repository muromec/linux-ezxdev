/******************************************************************************

    misoc0343.h
    driver for Micron MI-SOC-0343 Camera

    Author: MontaVista Software, Inc. <source@mvista.com>
    Copyright (c) 2004 MontaVista Software, Inc.

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

********************************************************************************/
#ifndef __MISOC_0343_CAM_H__
#define __MISOC_0343_CAM_H__

/*
 * Micron CMOS camera registers:
 */

#define MISOC0343_R1     0x01

/* Micron has 2 areas - Image core and Image flow processor */
#define MISOC0343_IFP    0x01   /* set R1 to choose Image Flow Processor (IFP) */
#define MISOC0343_IC     0x04   /* set R1 to choose Image Core (IC) */

/* IFP registers */
#define MISOC0343_IFP_RESET       0x07
#define MISOC0343_IFP_FORMAT      0x08
#define MISOC0343_IFP_SHWIDTH     0x37
#define MISOC0343_IFP_DECIMATION  0x46
#define MISOC0343_IFP_VERSION     0x49


/* IFP registers bit values */
#define MISOC0343_SHWIDTH_DEFAULT  0x0100 /* Default value for Shutter Width */

#define MISOC0343_FORMAT_565RGB    0x1000 /* Enable sensor output in RGB565 */
#define MISOC0343_FORMAT_FLCKDTC   0x0800 /* Enable auto flicker detection  */
#define MISOC0343_FORMAT_YUV422    0x0400 /* Enable auto sequence Cb1 Y1 CR1 Y2
                                           */
#define MISOC0343_FORMAT_LENSSHADE 0x0100 /* Enable lens shading correction */

#define MISOC0343_DECIMATION_VGA    0x0000 /* VGA (640 x 480) */
#define MISOC0343_DECIMATION_QVGA   0x0001 /* QVGA (320 x 240) */
#define MISOC0343_DECIMATION_CIF    0x0002 /* CIF (352 x 288) */
#define MISOC0343_DECIMATION_QCIF   0x0003 /* QCIF (176 x 144) */
#define MISOC0343_DECIMATION_QQVGA  0x0004 /* QQVGA (160 x 120) */

#define MISOC0343_R6            0x0006
#define MISOC0343_R33           0x0021

#define MISOC0343_IMAGE_VERSION     0x07D3 /* IFP Core version */

/* IC registers */
#define MISOC0343_IC_RESET    0x0D
#define MISOC0343_IC_RDMODE   0x20 /* Read Mode register */
#define MISOC0343_IC_VERSION  0x36 /* Image Core Version */


#define MISOC0343_RDMODE_UPSDOWN    0x8080 /* Read Mode: readout from bottom to top
					      (it's recommended to set readout starting 1 row later) */
#define MISOC0343_RDMODE_MIRRORED   0x4020 /* Read Mode: readout from left to right
					      (it's recommended to set readout starting 1 column later) */
#define MISOC0343_RDMODE_BOOSTRST   0x1000 /* Read Mode: Enabled boosted reset */
#define MISOC0343_RDMODE_ALLFRAMES  0x0001 /* Read Mode: output all frames (including bad) */

#define MISOC0343_CORE_VERSION      0xE342 /* Imager Core Version */

#endif /* __MISOC_0343_CAM_H__ */
