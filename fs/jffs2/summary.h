/*
 * JFFS2 -- Journalling Flash File System, Version 2.
 *
 * Copyright (C) 2004  Ferenc Havasi <havasi@inf.u-szeged.hu>,
 *                     Zoltan Sogor <weth@inf.u-szeged.hu>,
 *                     Patrik Kluba <pajko@halom.u-szeged.hu>,
 *                     University of Szeged, Hungary
 * Copyright (c) 2005 Motorola Inc.
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
 * Modified by Jiang Jun <a21079@motorola.com> 2005/04/13
 *
 * For licensing information, see the file 'LICENCE' in this directory.
 *
 * $Id$
 *
 */

#ifndef JFFS2_SUMMARY_H
#define JFFS2_SUMMARY_H

#include <linux/uio.h>
#include <linux/jffs2.h>

#define DIRTY_SPACE(x) do { typeof(x) _x = (x); \
		c->free_size -= _x; c->dirty_size += _x; \
		jeb->free_size -= _x ; jeb->dirty_size += _x; \
		}while(0)
#define USED_SPACE(x) do { typeof(x) _x = (x); \
		c->free_size -= _x; c->used_size += _x; \
		jeb->free_size -= _x ; jeb->used_size += _x; \
		}while(0)
#define WASTED_SPACE(x) do { typeof(x) _x = (x); \
		c->free_size -= _x; c->wasted_size += _x; \
		jeb->free_size -= _x ; jeb->wasted_size += _x; \
		}while(0)
#define UNCHECKED_SPACE(x) do { typeof(x) _x = (x); \
		c->free_size -= _x; c->unchecked_size += _x; \
		jeb->free_size -= _x ; jeb->unchecked_size += _x; \
		}while(0)

#define BLK_STATE_ALLFF		0
#define BLK_STATE_CLEAN		1
#define BLK_STATE_PARTDIRTY	2
#define BLK_STATE_CLEANMARKER	3
#define BLK_STATE_ALLDIRTY	4
#define BLK_STATE_BADBLOCK	5

#define JFFS2_SUMMARY_NOSUM_SIZE 0xffffffff
#define JFFS2_SUMMARY_INODE_SIZE (sizeof(struct jffs2_sum_inode_flash))
#define JFFS2_SUMMARY_DIRENT_SIZE(x) (sizeof(struct jffs2_sum_dirent_flash) + (x))

struct jffs2_sum_unknown_flash
{
	jint16_t nodetype;	/* node type	*/
};

struct jffs2_sum_inode_flash
{
	jint16_t nodetype;	/* node type	*/
	jint32_t inode;		/* inode number */
	jint32_t version;	/* inode version */
	jint32_t offset;	/* offset on jeb */
	jint32_t totlen; 	/* record length */
} __attribute__((packed));

struct jffs2_sum_dirent_flash
{
	jint16_t nodetype;	/* == JFFS_NODETYPE_DIRENT */
	jint32_t totlen;	/* record length */
	jint32_t offset;	/* ofset on jeb */
	jint32_t pino;		/* parent inode */
	jint32_t version;	/* dirent version */
	jint32_t ino; 		/* == zero for unlink */
	uint8_t nsize;		/* dirent name size */
	uint8_t type;		/* dirent type */
	uint8_t name[0];	/* dirent name */
} __attribute__((packed));

union jffs2_sum_flash{
	struct jffs2_sum_unknown_flash u;
	struct jffs2_sum_inode_flash i;
	struct jffs2_sum_dirent_flash d; 
};

/* list version of jffs2_sum_*flash for kernel and sumtool */
struct jffs2_sum_unknown_mem
{
	union jffs2_sum_mem *next;
	jint16_t nodetype;	/* node type	*/
	
};

struct jffs2_sum_inode_mem
{
	union jffs2_sum_mem *next;
	jint16_t nodetype;	/* node type	*/
	jint32_t inode;		/* inode number */
	jint32_t version;	/* inode version */
	jint32_t offset;	/* offset on jeb */
	jint32_t totlen; 	/* record length */
} __attribute__((packed));

struct jffs2_sum_dirent_mem
{
	union jffs2_sum_mem *next;
	jint16_t nodetype;	/* == JFFS_NODETYPE_DIRENT */
	jint32_t totlen;	/* record length */
	jint32_t offset;	/* ofset on jeb */
	jint32_t pino;		/* parent inode */
	jint32_t version;	/* dirent version */
	jint32_t ino; 		/* == zero for unlink */
	uint8_t nsize;		/* dirent name size */
	uint8_t type;		/* dirent type */
	uint8_t name[0];	/* dirent name */
} __attribute__((packed));

union jffs2_sum_mem 
{
	struct jffs2_sum_unknown_mem u;
	struct jffs2_sum_inode_mem i;
	struct jffs2_sum_dirent_mem d; 
};

struct jffs2_sum_info
{
	uint32_t sum_size;
	uint32_t sum_num;
	uint32_t sum_padded;
	union jffs2_sum_mem *sum_list;
};

struct jffs2_sum_marker
{
	jint32_t offset;
	jint32_t erase_size;
	jint32_t magic;
};

#define JFFS2_SUMMARY_FRAME_SIZE (sizeof(struct jffs2_summary_node) + sizeof(struct jffs2_sum_marker))
	
#if !(defined(SUM_TOOL) || defined(JFFS2DUMP))

int jffs2_sum_init(struct jffs2_sb_info *c);
void jffs2_sum_exit(struct jffs2_sb_info *c);

int jffs2_sum_care_sum_collected(struct jffs2_eraseblock *jeb);

void jffs2_sum_clean_collected(struct jffs2_eraseblock *jeb);
void jffs2_sum_clean_all_info(struct jffs2_sb_info *c); /* clean up all summary information in all jeb (umount) */

int jffs2_sum_add_padding_mem(struct jffs2_eraseblock *jeb, uint32_t size);
int jffs2_sum_add_inode_mem(struct jffs2_eraseblock *jeb, struct jffs2_raw_inode *ri, uint32_t ofs);	
int jffs2_sum_add_dirent_mem(struct jffs2_eraseblock *jeb, struct jffs2_raw_dirent *rd, uint32_t ofs);	
int jffs2_sum_add_kvec(struct jffs2_sb_info *c, const struct kvec *invecs, unsigned long count,  uint32_t to);	

int jffs2_sum_scan_sumnode(struct jffs2_sb_info *c, struct jffs2_eraseblock *jeb, uint32_t ofs, uint32_t *pseudo_random);
int jffs2_sum_write_sumnode(struct jffs2_sb_info *c);	

#endif

#endif /* JFFS2_SUMMARY_H */
