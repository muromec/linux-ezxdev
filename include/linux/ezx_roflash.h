/*
 * Copyright (C) 2002 Motorola Inc.
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
 *
 *
 *  linux/include/linux/ezx_roflash.h 
 *
 *  Created by Susan Gu    0ct, 22 2002
 *  
 *  For multiple cramfs partitions in ezx project
 *  At repsent, there are three cramfs partitions in ezx project, they are: 
 *  1.  root file system
 *  2.  Language package
 *  3.  setup package 
*/

#ifndef EZX_ROFLASH_FS_H
#define EZX_ROFLASH_FS_H

#include <asm/ioctl.h>

#define ROFLASH_MAJOR  62

#define ROFLASH_LINEAR  0x0010
#define ROFLASH_LINEAR_XIP     0x0011
#define ROFLASH_BLOCK   0x1000
#define ROFLASH_CHAR    0x1100
#define MAX_ROFLASH  8

typedef struct
{
	char name[16];  /* This length should be enough for DC stuff */
	unsigned long offset;
	unsigned long size;
	int (*roflash_read)(void *, unsigned long, size_t, size_t *, u_char *);
	/* Added by Susan for multiple Linear or block cramfs */
	void *priv_map;
	unsigned long phys_addr;
	unsigned short l_x_b;
}roflash_area;

extern roflash_area *roflash_get_dev(unsigned char minor);
extern roflash_area **roflash_table_pptr;

#endif
