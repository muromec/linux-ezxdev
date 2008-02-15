#ifndef __ASM_XTENSA_STATFS_H
#define __ASM_XTENSA_STATFS_H

/*
 * include/asm-xtensa/statfs.h
 *
 * Swiped from SH.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

#ifndef __KERNEL_STRICT_NAMES

#include <linux/types.h>

typedef __kernel_fsid_t	fsid_t;

#endif

struct statfs {
	long f_type;
	long f_bsize;
	long f_blocks;
	long f_bfree;
	long f_bavail;
	long f_files;
	long f_ffree;
	__kernel_fsid_t f_fsid;
	long f_namelen;
	long f_spare[6];
};

#endif /* __ASM_XTENSA_STATFS_H */
