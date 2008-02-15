/*
 * pram_fs_sb.h
 *
 * Definitions for the PRAM filesystem.
 *
 * Author: Steve Longerbeam <stevel@mvista.com, or source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc.
 * Copyright 2003 Sony Corporation
 * Copyright 2003 Matsushita Electric Industrial Co., Ltd.
 *
 * This software is being distributed under the terms of the GNU General Public
 * License version 2.  Some or all of the technology encompassed by this
 * software may be subject to one or more patents pending as of the date of
 * this notice.  No additional patent license will be required for GPL
 * implementations of the technology.  If you want to create a non-GPL
 * implementation of the technology encompassed by this software, please
 * contact legal@mvista.com for details including licensing terms and fees.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#ifndef _LINUX_PRAM_FS_SB
#define _LINUX_PRAM_FS_SB

/*
 * PRAM filesystem super-block data in memory
 */
struct pram_sb_info {
	/*
	 * base physical and virtual address of PRAMFS (which is also
	 * the pointer to the super block)
	 */
	unsigned long phys_addr;
	void * virt_addr;

	u32 maxsize;
	u32 features;
	u16 magic;
};

#endif	/* _LINUX_PRAM_FS_SB */
