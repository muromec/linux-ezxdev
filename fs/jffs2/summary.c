/*
 * JFFS2 -- Journalling Flash File System, Version 2.
 *
 * Copyright (C) 2004  Ferenc Havasi <havasi@inf.u-szeged.hu>,
 *                     Zoltan Sogor <weth@inf.u-szeged.hu>,
 *                     Patrik Kluba <pajko@halom.u-szeged.hu>,
 *                     University of Szeged, Hungary
 * Copyright (c) 2005 Motorola Inc.
 *
 * Modified by Ru Yi <e5537c@motorola.com> 2005/10/11
 *     Mount record interface added on summary
 *
 * For licensing information, see the file 'LICENCE' in this directory.
 *
 * $Id$
 *
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/pagemap.h>
#include <linux/crc32.h>
#include <linux/compiler.h>
#include <linux/vmalloc.h>
#include "nodelist.h"

int jffs2_sum_init(struct jffs2_sb_info *c) 
{
        c->summary_buf = (jint32_t *) vmalloc(c->sector_size);
        if (!c->summary_buf) {
		printk(KERN_WARNING "JFFS2: can't allocate memory to dump summary information!\n");
                return 1;
        }
        return 0;
}

void jffs2_sum_exit(struct jffs2_sb_info *c) 
{
        if (c->summary_buf) {
                vfree(c->summary_buf);
                c->summary_buf = NULL;
        }
}

int jffs2_sum_care_sum_collected(struct jffs2_eraseblock *jeb)
{
	if (!jeb->sum_collected) {
		jeb->sum_collected = (struct jffs2_sum_info *) kmalloc(sizeof(struct jffs2_sum_info), GFP_KERNEL);
	
		if (!jeb->sum_collected)
			return -ENOMEM;
			
		jeb->sum_collected->sum_list = NULL;
		jeb->sum_collected->sum_num = 0;
		jeb->sum_collected->sum_size = 0; 
		jeb->sum_collected->sum_padded = 0; 
	}
        return 0;
}

static int jffs2_sum_add_mem(struct jffs2_eraseblock *jeb, union jffs2_sum_mem *item) 
{
	
	union jffs2_sum_mem *walk;
        int ret;
	
        if ((ret=jffs2_sum_care_sum_collected(jeb))) return ret;
	
	if (!jeb->sum_collected->sum_list) {
		jeb->sum_collected->sum_list = (union jffs2_sum_mem *) item;
	} 
	else {
		walk = jeb->sum_collected->sum_list;
		
		while (walk->u.next) {
			walk = walk->u.next;
		}
		walk->u.next = (union jffs2_sum_mem *) item;
	}
	switch (je16_to_cpu(item->u.nodetype)) {
    	    case JFFS2_NODETYPE_INODE:
		jeb->sum_collected->sum_size += JFFS2_SUMMARY_INODE_SIZE;
		jeb->sum_collected->sum_num++;
		break;
    	    case JFFS2_NODETYPE_DIRENT:
		jeb->sum_collected->sum_size += JFFS2_SUMMARY_DIRENT_SIZE(item->d.nsize);
		jeb->sum_collected->sum_num++;
		break;
	    default:
		printk(KERN_WARNING "__jffs2_add_sum_mem(): UNKNOWN node type %d\n", je16_to_cpu(item->u.nodetype));
		return 1;
	}
	return 0;
}

void jffs2_sum_clean_all_info(struct jffs2_sb_info *c)
{
	int i;
	
	for (i=0; i<c->nr_blocks; i++) {
		struct jffs2_eraseblock *jeb = &c->blocks[i];
		
		jffs2_sum_clean_collected(jeb);
		kfree(jeb->sum_collected);
		jeb->sum_collected = NULL;
	}
}
	
/* These 3 functions are called from scan.c to collect summary info for not closed jeb */

int jffs2_sum_add_padding_mem(struct jffs2_eraseblock *jeb, uint32_t size)
{
        int ret;
	
        if ((ret=jffs2_sum_care_sum_collected(jeb))) return ret;
        jeb->sum_collected->sum_padded += size;
        return 0;
}

int jffs2_sum_add_inode_mem(struct jffs2_eraseblock *jeb, struct jffs2_raw_inode *ri, uint32_t ofs) 
{
	
	struct jffs2_sum_inode_mem *temp = (struct jffs2_sum_inode_mem *) kmalloc(sizeof(struct jffs2_sum_inode_mem), GFP_KERNEL);
	
	if (!temp)
		return -ENOMEM;

        ofs -= jeb->offset;
	
	temp->nodetype = ri->nodetype;
	temp->inode = ri->ino;
	temp->version = ri->version;
	temp->offset = cpu_to_je32(ofs); 
	temp->totlen = ri->totlen;
	temp->next = NULL;
	
	return jffs2_sum_add_mem(jeb, (union jffs2_sum_mem *)temp);
}

int jffs2_sum_add_dirent_mem(struct jffs2_eraseblock *jeb, struct jffs2_raw_dirent *rd, uint32_t ofs) 
{
	
	struct jffs2_sum_dirent_mem *temp = (struct jffs2_sum_dirent_mem *) 
			kmalloc(sizeof(struct jffs2_sum_dirent_mem) + rd->nsize, GFP_KERNEL);
	
	if (!temp)
		return -ENOMEM;
	
        ofs -= jeb->offset;

	temp->nodetype = rd->nodetype;
	temp->totlen = rd->totlen;
	temp->offset = cpu_to_je32(ofs);
	temp->pino = rd->pino;
	temp->version = rd->version;
	temp->ino = rd->ino;
	temp->nsize = rd->nsize;
	temp->type = rd->type;
	temp->next = NULL;
	
	memcpy(temp->name, rd->name, rd->nsize);

	return jffs2_sum_add_mem(jeb, (union jffs2_sum_mem *)temp);
}

/* Cleanup every collected summary information */

void jffs2_sum_clean_collected(struct jffs2_eraseblock *jeb) 
{
	
	union jffs2_sum_mem *temp;
	
	if(jeb && jeb->sum_collected){
		
		while(jeb->sum_collected->sum_list){
			temp = jeb->sum_collected->sum_list;
			jeb->sum_collected->sum_list = jeb->sum_collected->sum_list->u.next;
			kfree(temp);
			jeb->sum_collected->sum_num--;
		}
		
		if(jeb->sum_collected->sum_num != 0){
			printk(KERN_WARNING "Ooops, something wrong happened! sum_num != 0, but sum_list = null ???");
			jeb->sum_collected->sum_num = 0;
		}
	}	
}

/* Called from wbuf.c to collect writed node info */

int jffs2_sum_add_kvec(struct jffs2_sb_info *c, const struct kvec *invecs, unsigned long count, uint32_t ofs)
{
	union jffs2_node_union *node;
	struct jffs2_eraseblock *jeb;	
        int ret;
	
	node = (union jffs2_node_union *) invecs[0].iov_base;
	jeb = &c->blocks[ofs / c->sector_size];
        ofs -= jeb->offset;
       
        if ((ret=jffs2_sum_care_sum_collected(jeb))) return ret;        
	
	switch(je16_to_cpu(node->u.nodetype)){
		case JFFS2_NODETYPE_INODE : {
			struct jffs2_sum_inode_mem *temp = (struct jffs2_sum_inode_mem *) 
				kmalloc(sizeof(struct jffs2_sum_inode_mem), GFP_KERNEL);
			
			if (!temp)
				return -ENOMEM;

			temp->nodetype = node->i.nodetype;
			temp->inode = node->i.ino;
			temp->version = node->i.version;
			temp->offset = cpu_to_je32(ofs); 
			temp->totlen = node->i.totlen;
			temp->next = NULL;
						
			return jffs2_sum_add_mem(jeb, (union jffs2_sum_mem *)temp);
			
			break;
		}
		
		case JFFS2_NODETYPE_DIRENT : {
			struct jffs2_sum_dirent_mem *temp = (struct jffs2_sum_dirent_mem *) 
				kmalloc(sizeof(struct jffs2_sum_dirent_mem) + node->d.nsize, GFP_KERNEL);
			
			if (!temp)
				return -ENOMEM;
			
			temp->nodetype = node->d.nodetype;
			temp->totlen = node->d.totlen;
			temp->offset = cpu_to_je32(ofs);
			temp->pino = node->d.pino;
			temp->version = node->d.version;
			temp->ino = node->d.ino;
			temp->nsize = node->d.nsize;
			temp->type = node->d.type;
			temp->next = NULL;
			
			switch (count) {
			
				case 1 : 
					memcpy(temp->name,node->d.name,node->d.nsize);
					break;
				
				case 2 : 
					memcpy(temp->name,invecs[1].iov_base,node->d.nsize);
					break;
				
				default :
					printk(KERN_WARNING "jffs2_sum_add_kvec(): bad count value \n");
					break;
			}
			
			return jffs2_sum_add_mem(jeb, (union jffs2_sum_mem *)temp);
			
			break;
		}
		
		case JFFS2_NODETYPE_PADDING : {
			D1(printk(KERN_DEBUG "jffs2_sum_add_kvec(): Node PADDING\n"));
                        jeb->sum_collected->sum_padded += je32_to_cpu(node->u.totlen);
			break;
		}
		
		case JFFS2_NODETYPE_CLEANMARKER : {
			D1(printk(KERN_DEBUG "jffs2_sum_add_kvec(): Node CLEANMARKER\n"));
			break;
		}
		
		case JFFS2_NODETYPE_SUMMARY : {
			D1(printk(KERN_DEBUG "jffs2_sum_add_kvec(): Node SUMMARY\n"));
			break;
		}
		
		default : {
			printk(KERN_WARNING "jffs2_sum_add_kvec(): Node not supported\n");
			BUG();
			break;
		}
	}
	
	return 0;
}

/* Process the summary information - called from jffs2_scan_eraseblock() */

int jffs2_sum_scan_sumnode(struct jffs2_sb_info *c, struct jffs2_eraseblock *jeb, uint32_t ofs, uint32_t *pseudo_random)
{
	
	struct jffs2_unknown_node crcnode;
	struct jffs2_raw_node_ref *raw;
	struct jffs2_raw_node_ref *cache_ref;
	struct jffs2_inode_cache *ic;
	struct jffs2_full_dirent *fd;
		
	int i, err;
	int bad_sum = 0;
	int sumsize;
	uint32_t ino;
	uint32_t crc;
	struct jffs2_summary_node *summary;
		
	sumsize = c->sector_size - ofs;
	ofs += jeb->offset;
	
	D1(printk(KERN_DEBUG "JFFS2: summary found for 0x%08x at 0x%08x (0x%x bytes)\n", jeb->offset, ofs, sumsize));
	
	summary = (struct jffs2_summary_node *) kmalloc(sumsize, GFP_KERNEL);
		
	if (!summary) {
			return -ENOMEM;
	}
	
	err = jffs2_fill_scan_buf(c, (unsigned char *)summary, ofs, sumsize);
	
	if (err) {
			kfree(summary);
			return err;
	}

	/* OK, now check for node validity and CRC */
	crcnode.magic = cpu_to_je16(JFFS2_MAGIC_BITMASK);
	crcnode.nodetype = cpu_to_je16(JFFS2_NODETYPE_SUMMARY);
	crcnode.totlen = summary->totlen;
	crc = crc32(0, &crcnode, sizeof(crcnode)-4);
	
	if (je32_to_cpu(summary->hdr_crc) != crc) {
			D1(printk(KERN_DEBUG "jffs2_scan_eraseblock(): Summary node header is corrupt (bad CRC or no summary at all)\n"));
			bad_sum = 1;
	}
	
	if ((!bad_sum) && (je32_to_cpu(summary->totlen) != sumsize)) {
			D1(printk(KERN_DEBUG "jffs2_scan_eraseblock(): Summary node is corrupt (wrong erasesize?)\n"));
			bad_sum = 1;
	}
	
	crc = crc32(0, summary, sizeof(struct jffs2_summary_node)-8);
		
	if ((!bad_sum) && (je32_to_cpu(summary->node_crc) != crc)) {
			D1(printk(KERN_DEBUG "jffs2_scan_eraseblock(): Summary node is corrupt (bad CRC)\n"));
			bad_sum = 1;
	}
	
	crc = crc32(0, summary->sum, sumsize - sizeof(struct jffs2_summary_node));

	if ((!bad_sum) && (je32_to_cpu(summary->sum_crc) != crc)) {
			D1(printk(KERN_DEBUG "jffs2_scan_eraseblock(): Summary node data is corrupt (bad CRC)\n"));
			bad_sum = 1;
	}
	
	if (!bad_sum) {
		
		struct jffs2_sum_unknown_flash *sp;
		sp = (struct jffs2_sum_unknown_flash *) summary->sum;

		if ( je32_to_cpu(summary->cln_mkr) ) {
			
			D1(printk(KERN_DEBUG "Summary : CLEANMARKER node \n"));
			
			if (je32_to_cpu(summary->cln_mkr) != c->cleanmarker_size) {
				D1(printk(KERN_DEBUG "CLEANMARKER node has totlen 0x%x != normal 0x%x\n", 
				   je32_to_cpu(summary->cln_mkr), c->cleanmarker_size));
				UNCHECKED_SPACE( PAD(je32_to_cpu(summary->cln_mkr)) );
			} 
			else if (jeb->first_node) {
				D1(printk(KERN_DEBUG "CLEANMARKER node not first node in block (0x%08x)\n", jeb->offset));
				UNCHECKED_SPACE( PAD(je32_to_cpu(summary->cln_mkr)));
			} 
			else {
				struct jffs2_raw_node_ref *marker_ref = jffs2_alloc_raw_node_ref();
					
				if (!marker_ref) {
					D1(printk(KERN_NOTICE "Failed to allocate node ref for clean marker\n"));
					kfree(summary);
					return -ENOMEM;
				}
				
				marker_ref->next_in_ino = NULL;
				marker_ref->next_phys = NULL;
				marker_ref->flash_offset = jeb->offset | REF_NORMAL;
				marker_ref->__totlen = je32_to_cpu(summary->cln_mkr);
				jeb->first_node = jeb->last_node = marker_ref;
			
				USED_SPACE( PAD(je32_to_cpu(summary->cln_mkr)) );
								
			}
		}

		if ( je32_to_cpu(summary->padded) ) {
                        DIRTY_SPACE(je32_to_cpu(summary->padded));
                }
		
		for(i = 0; i < je16_to_cpu(summary->sum_num); i++) {
			
			D1(printk(KERN_DEBUG "jffs2_scan_eraseblock(): Processing summary information %d\n", i));
			uint8_t *temp8ptr = NULL;
			
			switch (je16_to_cpu(sp->nodetype)) {
				
				case JFFS2_NODETYPE_INODE : {
					struct jffs2_sum_inode_flash *spi;
					spi = (struct jffs2_sum_inode_flash *) sp;
						
					ino = je32_to_cpu(spi->inode);
					D1(printk(KERN_DEBUG "jffs2_scan_eraseblock(): Inode at 0x%08x\n", jeb->offset + je32_to_cpu(spi->offset)));
					raw = jffs2_alloc_raw_node_ref();
					if (!raw) {
						printk(KERN_NOTICE "jffs2_scan_eraseblock(): allocation of node reference failed\n");
						kfree(summary);
						return -ENOMEM;
					}
		
					ic = jffs2_get_ino_cache(c, ino);
					if (!ic) {
						ic = jffs2_scan_make_ino_cache(c, ino);
						if (!ic) {
							printk(KERN_NOTICE "jffs2_scan_eraseblock(): scan_make_ino_cache failed\n");
							jffs2_free_raw_node_ref(raw);
							kfree(summary);
							return -ENOMEM;
						}
					#ifdef CONFIG_MOUNT_RECORD
						if (!c->jffs2_need_checking)
							ic->state = INO_STATE_CHECKEDABSENT;
					#endif
					#ifdef CONFIG_JFFS2_SUMMARY_NOCRCCHECK	
						ic->state = INO_STATE_CHECKEDABSENT;
					#endif
					}
						
				#ifdef CONFIG_MOUNT_RECORD
					if (!c->jffs2_need_checking)
						raw->flash_offset = (jeb->offset + je32_to_cpu(spi->offset)) | REF_NORMAL;
					else
				#endif
					raw->flash_offset = (jeb->offset + je32_to_cpu(spi->offset)) |
				#ifdef CONFIG_JFFS2_SUMMARY_NOCRCCHECK
					REF_NORMAL; 
				#else
					REF_UNCHECKED;
				#endif
					raw->__totlen = PAD(je32_to_cpu(spi->totlen));
					raw->next_phys = NULL;
					raw->next_in_ino = ic->nodes;
						
					ic->nodes = raw;
					if (!jeb->first_node)
							jeb->first_node = raw;
					if (jeb->last_node)
							jeb->last_node->next_phys = raw;
					jeb->last_node = raw;
						
					*pseudo_random += je32_to_cpu(spi->version);
					
				#ifdef CONFIG_MOUNT_RECORD
					if (!c->jffs2_need_checking)
						USED_SPACE(PAD(je32_to_cpu(spi->totlen)));
					else
				#endif
				#ifdef CONFIG_JFFS2_SUMMARY_NOCRCCHECK
					USED_SPACE(PAD(je32_to_cpu(spi->totlen)));
				#else
					UNCHECKED_SPACE(PAD(je32_to_cpu(spi->totlen)));
				#endif
					temp8ptr = (uint8_t *) sp;
					temp8ptr += JFFS2_SUMMARY_INODE_SIZE;
					sp = (struct jffs2_sum_unknown_flash *) temp8ptr;
					
					break;
				}
					
				case JFFS2_NODETYPE_DIRENT : {
					struct jffs2_sum_dirent_flash *spd;
					spd = (struct jffs2_sum_dirent_flash *) sp;
					
					fd = jffs2_alloc_full_dirent(spd->nsize+1);
					if (!fd) {
						kfree(summary);
						return -ENOMEM;
					}
					
					memcpy(&fd->name, spd->name, spd->nsize);
					fd->name[spd->nsize] = 0;
					
					raw = jffs2_alloc_raw_node_ref();
					if (!raw) {
						jffs2_free_full_dirent(fd);
						printk(KERN_NOTICE "jffs2_scan_dirent_node(): allocation of node reference failed\n");
						kfree(summary);
						return -ENOMEM;
					}
					
					ic = jffs2_get_ino_cache(c, je32_to_cpu(spd->pino));
					if (!ic) {
						ic = jffs2_scan_make_ino_cache(c, je32_to_cpu(spd->pino));
						if (!ic) {
							jffs2_free_full_dirent(fd);
							jffs2_free_raw_node_ref(raw);
							kfree(summary);
							return -ENOMEM;
						}
					#ifdef CONFIG_MOUNT_RECORD
						if (!c->jffs2_need_checking)
							ic->state = INO_STATE_CHECKEDABSENT;
					#endif
					#ifdef CONFIG_JFFS2_SUMMARY_NOCRCCHECK	
						ic->state = INO_STATE_CHECKEDABSENT;
					#endif
					}
					
					raw->__totlen = PAD(je32_to_cpu(spd->totlen));
					raw->flash_offset = (jeb->offset + je32_to_cpu(spd->offset)) | REF_PRISTINE;
					raw->next_phys = NULL;
					raw->next_in_ino = ic->nodes;
					ic->nodes = raw;
					if (!jeb->first_node)
						jeb->first_node = raw;
					if (jeb->last_node)
						jeb->last_node->next_phys = raw;
					jeb->last_node = raw;
				
					fd->raw = raw;
					fd->next = NULL;
					fd->version = je32_to_cpu(spd->version);
					fd->ino = je32_to_cpu(spd->ino);
					fd->nhash = full_name_hash(fd->name, spd->nsize);
					fd->type = spd->type;
					USED_SPACE(PAD(je32_to_cpu(spd->totlen)));
					jffs2_add_fd_to_list(c, fd, &ic->scan_dents);
					
					*pseudo_random += je32_to_cpu(spd->version);
					
					temp8ptr = (uint8_t *) sp;
					temp8ptr += JFFS2_SUMMARY_DIRENT_SIZE(spd->nsize); 
					sp = (struct jffs2_sum_unknown_flash *) temp8ptr;
					
					break;
				}
				
				default : {
					printk(KERN_WARNING "Kernel doesn't support this type of node !!! Exiting");
					return -EIO;
					break;			
				}
			}
		}
		
		kfree(summary);

		/* for ACCT_PARANOIA_CHECK */
		cache_ref = jffs2_alloc_raw_node_ref();
		
		if (!cache_ref) {
			printk(KERN_NOTICE "Failed to allocate node ref for cache\n");
			return -ENOMEM;
		}
		
		cache_ref->next_in_ino = NULL;
		cache_ref->next_phys = NULL;
		cache_ref->flash_offset = ofs | REF_NORMAL;
		cache_ref->__totlen = sumsize;
		
		if (!jeb->first_node)
			jeb->first_node = cache_ref;
		if (jeb->last_node)
			jeb->last_node->next_phys = cache_ref;
		jeb->last_node = cache_ref;
		
		USED_SPACE(sumsize);

		jeb->wasted_size += jeb->free_size;
		c->wasted_size += jeb->free_size;
		c->free_size -= jeb->free_size;
		jeb->free_size = 0;

		/* somebody check this and all of space accounting in summary support */

		if ((jeb->used_size + jeb->unchecked_size) == PAD(c->cleanmarker_size) && !jeb->dirty_size 
			&& (!jeb->first_node || !jeb->first_node->next_in_ino) ) { 
				return BLK_STATE_CLEANMARKER; 
			}		
		/* move blocks with max 4 byte dirty space to cleanlist */	
		else if (!ISDIRTY(c->sector_size - (jeb->used_size + jeb->unchecked_size))) {
			c->dirty_size -= jeb->dirty_size;
			c->wasted_size += jeb->dirty_size; 
			jeb->wasted_size += jeb->dirty_size;
			jeb->dirty_size = 0;
			return BLK_STATE_CLEAN;
		}
		else if (jeb->used_size || jeb->unchecked_size) { 
				return BLK_STATE_PARTDIRTY; 
		}
		else { 
				return BLK_STATE_ALLDIRTY; 
		}
	}
	
	return 0;
}

/* Write out summary information - called from jffs2_do_reserve_space */

int jffs2_sum_write_sumnode(struct jffs2_sb_info *c)
{
	struct jffs2_summary_node isum;
	struct jffs2_raw_node_ref *summary_ref;
	union jffs2_sum_mem *temp;
	jint32_t offset;
	jint32_t *wpage;
	uint8_t *tempptr;
	int datasize;
	int infosize;
	int padsize;
	size_t retlen;
	int ret;
	struct jffs2_eraseblock *jeb;
	struct kvec vecs[2];	
	jint32_t magic = cpu_to_je32(JFFS2_SUM_MAGIC);
		
	D2(printk(KERN_DEBUG "jffs2_sum_write_sumnode()\n"));
	
	jeb = c->nextblock;
	
	if (!jeb->sum_collected->sum_num || !jeb->sum_collected->sum_list) {
		printk(KERN_WARNING "JFFS2: jffs2_sum_write_sumnode(): empty summary info!!!\n");
		BUG(); 
	}
	
	datasize = jeb->sum_collected->sum_size + sizeof(struct jffs2_sum_marker);
	infosize = sizeof(struct jffs2_summary_node) + datasize;
	padsize = jeb->free_size - infosize;
	infosize += padsize; datasize += padsize;
	offset = cpu_to_je32(c->sector_size - jeb->free_size);
	
	if (padsize < 0) { // if jeb hasn't got enought free space for summary
		
		union jffs2_sum_mem *temp;	
		
		while(jeb->sum_collected->sum_list){ //cleanup sum_list
			temp = jeb->sum_collected->sum_list;
			jeb->sum_collected->sum_list = jeb->sum_collected->sum_list->u.next;
			kfree(temp);
			jeb->sum_collected->sum_num--;
		}
		
		jeb->sum_collected->sum_list = NULL;
		jeb->sum_collected->sum_num = 0;
		jeb->sum_collected->sum_size = JFFS2_SUMMARY_NOSUM_SIZE; // don't try to write out summary for this node
		
		printk(KERN_WARNING "JFFS2: not enough space for summary, padsize = %d\n",padsize);
                return 0;
	}
			
	memset(c->summary_buf, 0xff, datasize);
	memset(&isum, 0, sizeof(isum));
	
	isum.magic = cpu_to_je16(JFFS2_MAGIC_BITMASK);
	isum.nodetype = cpu_to_je16(JFFS2_NODETYPE_SUMMARY);
	isum.totlen = cpu_to_je32(infosize);
	isum.hdr_crc = cpu_to_je32(crc32(0, &isum, sizeof(struct jffs2_unknown_node) - 4));
	isum.padded = cpu_to_je32(jeb->sum_collected->sum_padded);
		
	if (c->cleanmarker_size) {
		isum.cln_mkr = cpu_to_je32(c->cleanmarker_size);	
	}
	else{
		isum.cln_mkr = cpu_to_je32(0);
	}
	
	isum.sum_num = cpu_to_je16(jeb->sum_collected->sum_num);
	wpage = c->summary_buf;
	
		
	while (jeb->sum_collected->sum_num) { // write sum_data 
		

		switch(je16_to_cpu(jeb->sum_collected->sum_list->u.nodetype)){
			
			case JFFS2_NODETYPE_INODE : {
				jint16_t *temp16ptr = (jint16_t *)wpage;
				
				*(temp16ptr++) = jeb->sum_collected->sum_list->i.nodetype;
				wpage = (jint32_t *) temp16ptr;
				
				*(wpage++) = jeb->sum_collected->sum_list->i.inode;
				*(wpage++) = jeb->sum_collected->sum_list->i.version;
				*(wpage++) = jeb->sum_collected->sum_list->i.offset;
				*(wpage++) = jeb->sum_collected->sum_list->i.totlen;
				break;
			}
			
			case JFFS2_NODETYPE_DIRENT : {
				jint16_t *temp16ptr = (jint16_t *) wpage;
				uint8_t *temp8ptr = NULL;
				
				*(temp16ptr++) = jeb->sum_collected->sum_list->d.nodetype;
				wpage = (jint32_t *) temp16ptr;
				
				*(wpage++) = jeb->sum_collected->sum_list->d.totlen;
				*(wpage++) = jeb->sum_collected->sum_list->d.offset;
				*(wpage++) = jeb->sum_collected->sum_list->d.pino;
				*(wpage++) = jeb->sum_collected->sum_list->d.version;
				*(wpage++) = jeb->sum_collected->sum_list->d.ino;
				
				temp8ptr = (uint8_t *) wpage;
				*(temp8ptr++) = jeb->sum_collected->sum_list->d.nsize;
				*(temp8ptr++) = jeb->sum_collected->sum_list->d.type;
								
				memcpy(temp8ptr,jeb->sum_collected->sum_list->d.name,jeb->sum_collected->sum_list->d.nsize);
				temp8ptr += jeb->sum_collected->sum_list->d.nsize;
				wpage = (jint32_t *) temp8ptr;
				
				break;
			}
			
			default : {
				printk(KERN_WARNING "Unknown node in summary information!!!\n");
				BUG();
			}
		}
		
		temp = jeb->sum_collected->sum_list;
		jeb->sum_collected->sum_list = jeb->sum_collected->sum_list->u.next;
		kfree(temp);
		
		jeb->sum_collected->sum_num--;
	}
	
	jeb->sum_collected->sum_size = 0;
	jeb->sum_collected->sum_padded = 0;
	
	tempptr = (uint8_t *) wpage;
	tempptr += padsize;
	wpage = (jint32_t *) tempptr;
	
	*(wpage++) = offset;
	*(wpage++) = cpu_to_je32(c->sector_size);
	*(wpage++) = magic;
	isum.sum_crc = cpu_to_je32(crc32(0, c->summary_buf, datasize));
	isum.node_crc = cpu_to_je32(crc32(0, &isum, sizeof(isum) - 8));
	
	vecs[0].iov_base = &isum;
	vecs[0].iov_len = sizeof(isum);
	vecs[1].iov_base = c->summary_buf;
	vecs[1].iov_len = datasize;
			
	D1(printk(KERN_DEBUG "JFFS2: writing out data to flash to pos : 0x%08x\n",jeb->offset + c->sector_size - jeb->free_size));
	
	spin_unlock(&c->erase_completion_lock);
	ret = jffs2_flash_writev(c, vecs, 2, jeb->offset + c->sector_size - jeb->free_size, &retlen, 0);
	spin_lock(&c->erase_completion_lock);

	
	if (ret || (retlen != infosize)) {
		printk(KERN_WARNING "JFFS2: write of %zd bytes at 0x%08x failed. returned %d, retlen %zd\n", 
		      infosize, jeb->offset + c->sector_size - jeb->free_size, ret, retlen);

		jeb->sum_collected->sum_size = JFFS2_SUMMARY_NOSUM_SIZE;
				
		WASTED_SPACE(infosize);
		
		return 0;
	}	
	
	/* for ACCT_PARANOIA_CHECK */
	summary_ref = jffs2_alloc_raw_node_ref();
	
	if (!summary_ref) {
		printk(KERN_NOTICE "Failed to allocate node ref for summary\n");
		return -ENOMEM;
	}
		
	summary_ref->next_in_ino = NULL;
	summary_ref->next_phys = NULL;
	summary_ref->flash_offset = (jeb->offset + c->sector_size - jeb->free_size) | REF_NORMAL;
	summary_ref->__totlen = infosize;
		
	if (!jeb->first_node)
		jeb->first_node = summary_ref;
	if (jeb->last_node)
		jeb->last_node->next_phys = summary_ref;
	jeb->last_node = summary_ref;
	
	USED_SPACE(infosize);
	
	return 0;	
}
