/* 
 * include/linux/mnt_record.h
 *  
 * Copyright (c) 2005 Motorola Inc.
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
 * According the records of different partitions' mounting successful times 
 * it makes the decision that the flash partition should be checked or not.
 *
 * Design for nor-flash memory
 *
 * Author: Ru Yi <e5537c@motorola.com> 2005/10/11
 */

#ifndef __MNT_RECORD_LINUX_H__
#define __MNT_RECORD_LINUX_H__
#include <linux/version.h>
#include <linux/version.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>

/* 
 * To keep the module independent, we just use macro to define mtd and do not 
 * include the mtd_info to the structure mnt_record_sb though this looks like 
 * somewhat ugly.
 */
/* #define MNT_RECORD_MAJOR MTD_BLOCK_MAJOR */
#ifdef CONFIG_ARCH_EZX_HAINAN
#define MNT_RECORD_OFS 0x01880000
#else
#define MNT_RECORD_OFS 0x01C80000
#endif
#define MNT_RECORD_LEN 0x00020000
//struct mtd_info *mnt_record_mtd;
#define mnt_record_write(ofs, len, retlen, buf) (mnt_record_mtd->write(mnt_record_mtd, ofs + MNT_RECORD_OFS, len, retlen, buf))
#define mnt_record_read(ofs, len, retlen, buf) (mnt_record_mtd->read(mnt_record_mtd, ofs + MNT_RECORD_OFS, len, retlen, buf))

/* By default, we assume the max mount count for establishing mount record */
#define MAX_MNT_LENGTH (16*1024 - 2) 
#define MAX_MNT_COUNT (MAX_MNT_LENGTH*8)
static inline uint8_t mnt_bits_to_count(uint8_t x){
	uint8_t n = 0;
	x = ~(x) & 0xff;
	while(x) { x = x >> 1; ++n; }
	return n;
}

static inline uint8_t mnt_count_to_bits(uint8_t n){
	uint8_t x = 0xff;
	while(n){ x = x << 1; --n; }
	return x;
}

struct mnt_record_sb {
	spinlock_t mnt_record_lock;
	uint16_t parts;
	uint16_t partlen;
	uint32_t total_size;
	uint32_t used_size;
	struct mnt_record_info *info_list;
};

/* define mounted */
#define RC_NOMOUNT	0
#define RC_MOUNTING	1
#define RC_MOUNTED	2

struct mnt_record_info {
	struct mnt_record_info *next;
	kdev_t mnt_dev;
	uint32_t mounted; 
	uint32_t mnt_count;
	uint32_t max_mnt_count;
};

#define MNT_RC_MAGIC	4
#define MNT_MAX_PARTS	2
struct mnt_record_head {
	uint8_t magic[MNT_RC_MAGIC];
	uint8_t parts;
	uint8_t part_len[2];
	uint8_t reserved;
}__attribute__((packed));

#define MNT_RC_MAJOR	MTD_BLOCK_MAJOR 
#define MNT_MAX_MINOR	MAX_MTD_DEVICES
#define MNT_BANKWIDTH	2
struct mnt_record_part {
	uint8_t dev_major;
	uint8_t dev_minor;
	uint8_t mnt_record[0];
} __attribute__((packed));

extern int mnt_record_enter(struct mtd_info *mnt_mtd, kdev_t mnt_dev);
extern int mnt_record_leave(kdev_t mnt_dev);
#endif
