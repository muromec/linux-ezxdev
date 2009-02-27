/*
 * Common Flash Interface support:
 *   Intel Extended Vendor Command Set (ID 0x0001)
 *
 * (C) 2000 Red Hat. GPL'd
 * (C) 2003 Intel Corporation. GPL'd
 * Copyright (C) 2005 Motorola Inc.
 *
 * $Id: cfi_cmdset_0001_mp.c,v 1.1.4.2 2003/12/31 02:03:45 tpoynor Exp $
 *
 * 
 * 10/10/2000	Nicolas Pitre <nico@cam.org>
 * 	- completely revamped method functions so they are aware and
 * 	  independent of the flash geometry (buswidth, interleave, etc.)
 * 	- scalability vs code size is completely set at compile-time
 * 	  (see include/linux/mtd/cfi.h for selection)
 *	- optimized write buffer method
 * 02/05/2002	Christopher Hoover <ch@hpl.hp.com>/<ch@murgatroid.com>
 *	- reworked lock/unlock/erase support for var size flash
 * 08/15/2003  Yu Tang <yu.tang@intel.com>, Dan Post <daniel.j.post@intel.com>
 *      - added multi-partition support for Intel Strata Flash
 * 01/15/2005  Susan Gu <w15879@motorola.com>
 *      - Modified for EzX and pm support 
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <asm/io.h>
#include <asm/byteorder.h>

#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mtd/map.h>
#include <linux/mtd/cfi.h>
#include <linux/mtd/compatmac.h>
#include <linux/mtd/cfi_cmdset_0001_mp.h>


#ifdef CONFIG_ARCH_EZX


#ifdef CONFIG_ARCH_EZX_A780
#include <linux/pm.h>

#define L18_RW_REGION_START 0x01A00000  // The start offset for the R/W filesystem //
#ifdef CONFIG_PANIC_LOG
#define L18_RW_REGION_END   0x01FE0000  
#else
#define L18_RW_REGION_END   0x01F80000  // Susan -- 0x580000 --The end offset for the R/W filesystem   //
#endif	// CONFIG_PANIC_LOG

#define TIME_OUT   1000000L

/* This is for notification of flash status before we are asked entering into sleep mode -- Susan */
#ifdef CONFIG_PM
static struct pm_flash_s
{
	struct pm_dev *pm_dev;
	atomic_t pm_flash_count;
}pm_flash;
#endif

struct mtd_info *ezx_mymtd;
extern struct map_info bulverde_map;  //Susan
extern unsigned long bulverde_map_cacheable;  //Susan
int cfi_intelext_read_roflash(void *priv_map, unsigned long from, size_t len, size_t *retlen, u_char *buf);

#endif  //CONFIG_ARCH_EZX_A780

#ifdef CONFIG_ARCH_EZX_E680
#include <linux/pm.h>

#define L18_RW_REGION_START 0x01A00000  // The start offset for the R/W filesystem //
#ifdef CONFIG_PANIC_LOG
#define L18_RW_REGION_END   0x01FE0000  
#else
#define L18_RW_REGION_END   0x01F80000  // Susan -- 0x580000 --The end offset for the R/W filesystem   //
#endif // CONFIG_PANIC_LOG

#define TIME_OUT   1000000L

/* This is for notification of flash status before we are asked entering into sleep mode -- Susan */
#ifdef CONFIG_PM
static struct pm_flash_s
{
	struct pm_dev *pm_dev;
	atomic_t pm_flash_count;
}pm_flash;
#endif

struct mtd_info *ezx_mymtd;
extern struct map_info bulverde_map;  //Susan
extern unsigned long bulverde_map_cacheable;  //Susan
int cfi_intelext_read_roflash(void *priv_map, unsigned long from, size_t len, size_t *retlen, u_char *buf);

#endif  //CONFIG_ARCH_EZX_E680


#endif  //#ifdef CONFIG_ARCH_EZX


#if (defined(CONFIG_ARCH_EZX_A780) || defined(CONFIG_ARCH_EZX_E680))  // For A780/E680

/* debugging, turns off buffer write mode #define FORCE_WORD_WRITE */

static int cfi_intelext_read (struct mtd_info *, loff_t, size_t, size_t *, u_char *);
static int cfi_intelext_read_user_prot_reg (struct mtd_info *, loff_t, size_t, size_t *, u_char *);
static int cfi_intelext_read_fact_prot_reg (struct mtd_info *, loff_t, size_t, size_t *, u_char *);
static int cfi_intelext_write_words(struct mtd_info *, loff_t, size_t, size_t *, const u_char *);
static int cfi_intelext_write_buffers(struct mtd_info *, loff_t, size_t, size_t *, const u_char *);
static int cfi_intelext_erase_varsize(struct mtd_info *, struct erase_info *);
static void cfi_intelext_sync (struct mtd_info *);
static int cfi_intelext_lock(struct mtd_info *mtd, loff_t ofs, size_t len);
static int cfi_intelext_unlock(struct mtd_info *mtd, loff_t ofs, size_t len);
static int cfi_intelext_suspend (struct mtd_info *);
static void cfi_intelext_resume (struct mtd_info *);

static void cfi_intelext_destroy(struct mtd_info *);

struct mtd_info *cfi_cmdset_0001_mp(struct map_info *, int);

static struct mtd_info *cfi_intelext_setup (struct map_info *);

static struct mtd_chip_driver cfi_intelext_chipdrv = {
	probe: NULL, /* Not usable directly */
	destroy: cfi_intelext_destroy,
	name: "cfi_cmdset_0001_mp",
	module: THIS_MODULE
};

#undef DEBUG_LOCK_BITS
#undef DEBUG_CFI_FEATURES

#ifdef DEBUG_CFI_FEATURES
static void cfi_tell_features(struct cfi_pri_intelext *extp)
{
	int i;
	printk("  Feature/Command Support: %4.4X\n", extp->FeatureSupport);
	printk("     - Chip Erase:         %s\n", extp->FeatureSupport&1?"supported":"unsupported");
	printk("     - Suspend Erase:      %s\n", extp->FeatureSupport&2?"supported":"unsupported");
	printk("     - Suspend Program:    %s\n", extp->FeatureSupport&4?"supported":"unsupported");
	printk("     - Legacy Lock/Unlock: %s\n", extp->FeatureSupport&8?"supported":"unsupported");
	printk("     - Queued Erase:       %s\n", extp->FeatureSupport&16?"supported":"unsupported");
	printk("     - Instant block lock: %s\n", extp->FeatureSupport&32?"supported":"unsupported");
	printk("     - Protection Bits:    %s\n", extp->FeatureSupport&64?"supported":"unsupported");
	printk("     - Page-mode read:     %s\n", extp->FeatureSupport&128?"supported":"unsupported");
	printk("     - Synchronous read:   %s\n", extp->FeatureSupport&256?"supported":"unsupported");
	for (i=9; i<32; i++) {
		if (extp->FeatureSupport & (1<<i)) 
			printk("     - Unknown Bit %X:      supported\n", i);
	}
	
	printk("  Supported functions after Suspend: %2.2X\n", extp->SuspendCmdSupport);
	printk("     - Program after Erase Suspend: %s\n", extp->SuspendCmdSupport&1?"supported":"unsupported");
	for (i=1; i<8; i++) {
		if (extp->SuspendCmdSupport & (1<<i))
			printk("     - Unknown Bit %X:               supported\n", i);
	}
	
	printk("  Block Status Register Mask: %4.4X\n", extp->BlkStatusRegMask);
	printk("     - Lock Bit Active:      %s\n", extp->BlkStatusRegMask&1?"yes":"no");
	printk("     - Valid Bit Active:     %s\n", extp->BlkStatusRegMask&2?"yes":"no");
	for (i=2; i<16; i++) {
		if (extp->BlkStatusRegMask & (1<<i))
			printk("     - Unknown Bit %X Active: yes\n",i);
	}
	
	printk("  Vcc Logic Supply Optimum Program/Erase Voltage: %d.%d V\n", 
	       extp->VccOptimal >> 8, extp->VccOptimal & 0xf);
	if (extp->VppOptimal)
		printk("  Vpp Programming Supply Optimum Program/Erase Voltage: %d.%d V\n", 
		       extp->VppOptimal >> 8, extp->VppOptimal & 0xf);
}
#endif

/* This routine is made available to other mtd code via
 * inter_module_register.  It must only be accessed through
 * inter_module_get which will bump the use count of this module.  The
 * addresses passed back in cfi are valid as long as the use count of
 * this module is non-zero, i.e. between inter_module_get and
 * inter_module_put.  Keith Owens <kaos@ocs.com.au> 29 Oct 2000.
 */
struct mtd_info *cfi_cmdset_0001_mp(struct map_info *map, int primary)
{
	struct cfi_private *cfi = map->fldrv_priv;
	struct cfi_pri_intelext *extp;
	int i, ri, ofs_factor, qry_size, size;
	__u32 base; 
	__u16 adr;
	struct cfi_intelext_regioninfo *rinfo;
	struct cfi_intelext_blockinfo *binfo;
	struct flprivate *priv;
	struct flpartition *partition;
	unsigned char num_r;
	int ofs=0, j=0, num_p=0;

	for (i=0; i< cfi->numchips; i++) {
		cfi->chips[i].word_write_time = 128;
		cfi->chips[i].buffer_write_time = 128;
		cfi->chips[i].erase_time = 1024;

		if (cfi->cfi_mode != CFI_MODE_CFI) 
			continue;
		/* 
		 * It's a real CFI chip, not one for which the probe
		 * routine faked a CFI structure. So we read the feature
		 * table from it.
		 */
		base = cfi->chips[i].start;
		adr = primary?cfi->cfiq->P_ADR:cfi->cfiq->A_ADR;
		ofs_factor = cfi->interleave * cfi->device_type;
		qry_size = 0;

#ifdef CONFIG_MTD_DEBUG
		printk(" Intel/Sharp Extended Query Table at 0x%4.4X\n", adr);
#endif

		if (!adr)
			return NULL;

		/* Switch it into Query Mode */
		cfi_send_gen_cmd(0x98, 0x55, base, map, cfi, cfi->device_type, NULL);

		qry_size = CFI_QUERY_SIZE(map, base, adr);

		extp = kmalloc(qry_size, GFP_KERNEL);
		if (!extp) {
			printk(KERN_ERR "Failed to allocate memory\n");
			return NULL;
		}
		
		/* Read in the Extended Query Table */
		CFI_READ_QUERY(map, base, adr, extp, qry_size);
		
		if (extp->MajorVersion != '1' || 
		    (extp->MinorVersion < '0' || extp->MinorVersion > '3')) {
#ifdef CONFIG_MTD_DEBUG
			printk(KERN_WARNING "  Unknown IntelExt Extended Query "
			       "version %c.%c.\n",  extp->MajorVersion,
			       extp->MinorVersion);
#endif
			kfree(extp);
			return NULL;
		}


		/* Protection Register info */
		ofs += (extp->NumProtectionFields - 1) * (sizeof(__u32) + 6);

		/* Burst Read info */
		ofs += 6;

		/* Number of device hardware-parition regions within the device */
		num_r = *((unsigned char*)(extp+1) + ofs);
		ofs ++;

		/* Partition Region info */
		rinfo = (struct cfi_intelext_regioninfo*)((unsigned char*)(extp+1) + ofs);

		for (ri=0; ri<num_r; ri++) {

			rinfo->NumIdentPartitions=le16_to_cpu(rinfo->NumIdentPartitions);

			num_p += rinfo->NumIdentPartitions;

			for (j=0; j<rinfo->NumBlockTypes; j++) {
				binfo = rinfo->BlockTypes + j;
				
				binfo->NumIdentBlocks = le16_to_cpu(binfo->NumIdentBlocks);
				binfo->BlockSize = le16_to_cpu(binfo->BlockSize);
			}

			rinfo = (struct cfi_intelext_regioninfo*)(rinfo->BlockTypes + 
					rinfo->NumBlockTypes);

		}

		priv = (struct flprivate*)kmalloc(num_p * sizeof(struct flpartition) + 
				sizeof(struct flprivate), GFP_KERNEL);

		if (!priv) {
			printk(KERN_ERR "Failed to allocate memory\n");
			return NULL;
		}

		priv->num_writing = priv->num_erasing = 0;
		priv->curpos = 0;
		priv->num_partitions = num_p;
		priv->partitions = (struct flpartition*) (priv+1);
		priv->extp = extp;
		
		partition = priv->partitions;

		rinfo = (struct cfi_intelext_regioninfo*)((unsigned char*)(extp+1) + ofs);
		ofs = 0;
		for (ri=0; ri<num_r; ri++) {

			size = 0;
			for (j=0; j<rinfo->NumBlockTypes; j++) {
				binfo = rinfo->BlockTypes + j;

				size += (binfo->NumIdentBlocks + 1) * binfo->BlockSize * 
					256 * cfi->interleave;
			}

			for (j=0; j<rinfo->NumIdentPartitions; j++) {
				partition->offset = ofs;
				partition->size   = size;
				partition->info   = rinfo;
				partition->state  = FL_READY;
				partition->mutex  = &partition->_spinlock;
				spin_lock_init(&partition->_spinlock);
				init_waitqueue_head(&partition->wq);

				ofs += size;
				partition ++;
			}

			rinfo = (struct cfi_intelext_regioninfo*)(rinfo->BlockTypes + 
					rinfo->NumBlockTypes);
		}

		cfi->chips[i].priv = priv;

		/* Make sure it's in read mode */
		cfi_send_gen_cmd(0xff, 0x55, base, map, cfi, cfi->device_type, NULL);
		
		/* Do some byteswapping if necessary */
		extp->FeatureSupport = le32_to_cpu(extp->FeatureSupport);
		extp->BlkStatusRegMask = le16_to_cpu(extp->BlkStatusRegMask);
		extp->ProtRegAddr = le16_to_cpu(extp->ProtRegAddr);
			
#ifdef DEBUG_CFI_FEATURES
		/* Tell the user about it in lots of lovely detail */
		cfi_tell_features(extp);
#endif	

		/* Install our own private info structure */
		cfi->cmdset_priv = extp;	

	}

	map->fldrv = &cfi_intelext_chipdrv;
	MOD_INC_USE_COUNT;
	
	return cfi_intelext_setup(map);
}

static struct mtd_info *cfi_intelext_setup(struct map_info *map)
{
	struct cfi_private *cfi = map->fldrv_priv;
	struct mtd_info *mtd;
	unsigned long offset = 0;
	int i,j;
	unsigned long devsize = (1<<cfi->cfiq->DevSize) * cfi->interleave;

	mtd = kmalloc(sizeof(*mtd), GFP_KERNEL);

#ifdef CONFIG_MTD_DEBUG
	printk(KERN_DEBUG "number of CFI chips: %d\n", cfi->numchips);
#endif

	if (!mtd) {
		printk(KERN_ERR "Failed to allocate memory for MTD device\n");
		kfree(cfi->cmdset_priv);
		return NULL;
	}

	memset(mtd, 0, sizeof(*mtd));
	mtd->priv = map;
	mtd->type = MTD_NORFLASH;
	mtd->size = devsize * cfi->numchips;

	mtd->numeraseregions = cfi->cfiq->NumEraseRegions * cfi->numchips;
	mtd->eraseregions = kmalloc(sizeof(struct mtd_erase_region_info) 
			* mtd->numeraseregions, GFP_KERNEL);
	if (!mtd->eraseregions) { 
		printk(KERN_ERR "Failed to allocate memory for MTD erase region info\n");
		kfree(cfi->cmdset_priv);
		return NULL;
	}
	
	for (i=0; i<cfi->cfiq->NumEraseRegions; i++) {
		unsigned long ernum, ersize;
		ersize = ((cfi->cfiq->EraseRegionInfo[i] >> 8) & ~0xff) * cfi->interleave;
		ernum = (cfi->cfiq->EraseRegionInfo[i] & 0xffff) + 1;

		if (mtd->erasesize < ersize) {
			mtd->erasesize = ersize;
		}
		for (j=0; j<cfi->numchips; j++) {
			mtd->eraseregions[(j*cfi->cfiq->NumEraseRegions)+i].offset = (j*devsize)+offset;
			mtd->eraseregions[(j*cfi->cfiq->NumEraseRegions)+i].erasesize = ersize;
			mtd->eraseregions[(j*cfi->cfiq->NumEraseRegions)+i].numblocks = ernum;
		}
		offset += (ersize * ernum);
		}

		if (offset != devsize) {
			/* Argh */
			printk(KERN_WARNING "Sum of regions (%lx) != total size of set of interleaved chips (%lx)\n", offset, devsize);
			kfree(mtd->eraseregions);
			kfree(cfi->cmdset_priv);
			return NULL;
		}

		for (i=0; i<mtd->numeraseregions;i++){
			printk(KERN_DEBUG "%d: offset=0x%x,size=0x%x,blocks=%d\n",
			       i,mtd->eraseregions[i].offset,
			       mtd->eraseregions[i].erasesize,
			       mtd->eraseregions[i].numblocks);
		}

	/* Also select the correct geometry setup too */ 
		mtd->erase = cfi_intelext_erase_varsize;
	mtd->read = cfi_intelext_read;
#ifndef FORCE_WORD_WRITE
	if ( cfi->cfiq->BufWriteTimeoutTyp ) {
		printk("Using buffer write method\n" );
		mtd->write = cfi_intelext_write_buffers;
	} else {
#else
	{
#endif
		printk("Using word write method\n" );
		mtd->write = cfi_intelext_write_words;
	}
	mtd->read_user_prot_reg = cfi_intelext_read_user_prot_reg;
	mtd->read_fact_prot_reg = cfi_intelext_read_fact_prot_reg;
	mtd->sync = cfi_intelext_sync;
	mtd->lock = cfi_intelext_lock;
	mtd->unlock = cfi_intelext_unlock;
	mtd->suspend = cfi_intelext_suspend;
	mtd->resume = cfi_intelext_resume;
	mtd->flags = MTD_CAP_NORFLASH;
	map->fldrv = &cfi_intelext_chipdrv;
	MOD_INC_USE_COUNT;
	mtd->name = map->name;
	return mtd;
}

static inline int do_read_onechip(struct map_info *map, struct flchip *chip, loff_t adr, size_t len, u_char *buf)
{
	cfi_word status, status_OK[2], mask;
	unsigned long timeo;
	DECLARE_WAITQUEUE(wait, current);
	int suspended = 0;
	unsigned long cmd_addr;
	struct cfi_private *cfi = map->fldrv_priv;
	struct flpartition *partition;

//Susan for pm
#ifdef CONFIG_PM
	atomic_inc(&(pm_flash.pm_flash_count));
#endif

	partition = PARTITION_SEL(chip, adr);
	if (!partition)
	{
//Susan for pm
#ifdef CONFIG_PM
		atomic_dec(&(pm_flash.pm_flash_count));
#endif
		BUG();
	}

	adr += chip->start;

	/* Ensure cmd read/writes are aligned. */ 
	cmd_addr = adr & ~(CFIDEV_BUSWIDTH-1); 

	/* Let's determine this according to the interleave only once
	 * 
	 * 0x80 = No active program or erase operations
	 * 0x01 = Program or erase operation in other partition
	 */
	status_OK[0] = CMD(0x80);
	status_OK[1] = CMD(0x01);
	mask = CMD(0x81);

	timeo = jiffies + HZ;
 retry:
	spin_lock_bh(partition->mutex);

	/* Check that the chip's ready to talk to us.
	 * If it's in FL_ERASING state, suspend it and make it talk now.
	 */
	switch (partition->state) {
	case FL_ERASING:
		if (!(((struct cfi_pri_intelext *)cfi->cmdset_priv)->FeatureSupport & 2))
			goto sleep; /* We don't support erase suspend */
		
		cfi_write (map, CMD(0xb0), cmd_addr);
		/* If the flash has finished erasing, then 'erase suspend'
		 * appears to make some (28F320) flash devices switch to
		 * 'read' mode.  Make sure that we switch to 'read status'
		 * mode so we get the right data. --rmk
		 */
		cfi_write(map, CMD(0x70), cmd_addr);
		partition->oldstate = FL_ERASING;
		partition->state = FL_ERASE_SUSPENDING;

#ifdef CONFIG_MTD_DEBUG
		printk("Erase suspending at 0x%lx\n", cmd_addr);
#endif

		for (;;) {
			status = cfi_read(map, cmd_addr);
			if ((status & mask) == status_OK[0])
				break;
			
			if (time_after(jiffies, timeo)) {
				/* Urgh */
				cfi_write(map, CMD(0xd0), cmd_addr);
				/* make sure we're in 'read status' mode */
				cfi_write(map, CMD(0x70), cmd_addr);
				partition->state = FL_ERASING;
				spin_unlock_bh(partition->mutex);
				printk(KERN_ERR "Chip not ready after erase "
				       "suspended: status = 0x%llx\n", (__u64)status);
				//Susan for pm
#ifdef CONFIG_PM
				atomic_dec(&(pm_flash.pm_flash_count));
#endif
				return -EIO;
			}
			
			spin_unlock_bh(partition->mutex);
			cfi_udelay(1);
			spin_lock_bh(partition->mutex);
		}
		
		suspended = 1;
		cfi_write(map, CMD(0xff), cmd_addr);
		partition->state = FL_READY;
		break;
	
#if 0
	case FL_WRITING:
		/* Not quite yet */
#endif

	case FL_READY:
		break;

	case FL_CFI_QUERY:
	case FL_JEDEC_QUERY:
		cfi_write(map, CMD(0x70), cmd_addr);
		partition->state = FL_STATUS;

	case FL_STATUS:
		status = cfi_read(map, cmd_addr);
		if (((status & mask) == status_OK[0]) || ((status & mask) == status_OK[1])) {
			cfi_write(map, CMD(0xff), cmd_addr);
			partition->state = FL_READY;
			break;
		}
		
		/* Urgh. Chip not yet ready to talk to us. */
		if (time_after(jiffies, timeo)) {
			spin_unlock_bh(partition->mutex);
			printk(KERN_ERR "waiting for chip to be ready timed out in read. WSM status = %llx\n", (__u64)status);
#ifdef CONFIG_PM
				atomic_dec(&(pm_flash.pm_flash_count));
#endif
			return -EIO;
		}

		/* Latency issues. Drop the lock, wait a while and retry */
		spin_unlock_bh(partition->mutex);
		cfi_udelay(1);
		goto retry;

	default:
	sleep:
		/* Stick ourselves on a wait queue to be woken when
		   someone changes the status */
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue(&partition->wq, &wait);
		spin_unlock_bh(partition->mutex);
		schedule();
		remove_wait_queue(&partition->wq, &wait);
		timeo = jiffies + HZ;
		goto retry;
	}

	map->copy_from(map, buf, adr, len);

	if (suspended) {
		partition->state = partition->oldstate;
		/* What if one interleaved chip has finished and the 
		   other hasn't? The old code would leave the finished
		   one in READY mode. That's bad, and caused -EROFS 
		   errors to be returned from do_erase_oneblock because
		   that's the only bit it checked for at the time.
		   As the state machine appears to explicitly allow 
		   sending the 0x70 (Read Status) command to an erasing
		   chip and expecting it to be ignored, that's what we 
		   do. */
		/* Susan -- clear status register before resume from erase-suspending */
		cfi_write(map, CMD(0x50), cmd_addr);
			
		cfi_write(map, CMD(0xd0), cmd_addr);
		cfi_write(map, CMD(0x70), cmd_addr);		
	}

	wake_up(&partition->wq);
	spin_unlock_bh(partition->mutex);
	
#ifdef CONFIG_PM
	atomic_dec(&(pm_flash.pm_flash_count));
#endif

	return 0;
}

static int cfi_intelext_read (struct mtd_info *mtd, loff_t from, size_t len, size_t *retlen, u_char *buf)
{
	struct map_info *map = mtd->priv;
	struct cfi_private *cfi = map->fldrv_priv;
	unsigned long ofs;
	int chipnum;
	int ret = 0;

	/* ofs: offset within the first chip that the first read should start */
	chipnum = (from >> cfi->chipshift);
	ofs = from - (chipnum <<  cfi->chipshift);

	*retlen = 0;

	while (len) {
		unsigned long thislen;

		if (chipnum >= cfi->numchips)
			break;

		if ((len + ofs -1) >> cfi->chipshift)
			thislen = (1<<cfi->chipshift) - ofs;
		else
			thislen = len;

		ret = do_read_onechip(map, &cfi->chips[chipnum], ofs, thislen, buf);
		if (ret)
			break;

		*retlen += thislen;
		len -= thislen;
		buf += thislen;
		
		ofs = 0;
		chipnum++;
	}
	return ret;
}

static int cfi_intelext_read_prot_reg (struct mtd_info *mtd, loff_t from, size_t len, size_t *retlen, u_char *buf, int base_offst, int reg_sz)
{
	struct map_info *map = mtd->priv;
	struct cfi_private *cfi = map->fldrv_priv;
	struct cfi_pri_intelext *extp=cfi->cmdset_priv;
	int ofs_factor = cfi->interleave * cfi->device_type;
	int   count=len;
	struct flchip *chip;
	int chip_num,offst;
	unsigned long timeo;
	DECLARE_WAITQUEUE(wait, current);
	struct flpartition *partition;
	struct flprivate *priv;

	//Susan for pm
#ifdef CONFIG_PM
	atomic_inc(&(pm_flash.pm_flash_count));
#endif

	chip=0;
	/* Calculate which chip & protection register offset we need */
	chip_num=((unsigned int)from/reg_sz);
	offst=from-(reg_sz*chip_num)+base_offst;

	while(count){
		
		if(chip_num>=cfi->numchips)
			goto out;

		/* Make sure that the chip is in the right state */

		timeo = jiffies + HZ;
		chip=&cfi->chips[chip_num];

		priv = (struct flprivate*) chip->priv;
		partition = PARTITION_SEL(chip, 0);
		if (!partition)
		{
//Susan for pm
#ifdef CONFIG_PM
			atomic_dec(&(pm_flash.pm_flash_count));
#endif
			BUG();
		}
	retry:		
		spin_lock_bh(partition->mutex);
	
		switch (partition->state) {
		case FL_READY:
		case FL_STATUS:
		case FL_CFI_QUERY:
		case FL_JEDEC_QUERY:
			break;
		
		default:
				/* Stick ourselves on a wait queue to be woken when
				   someone changes the status */
			set_current_state(TASK_UNINTERRUPTIBLE);
			add_wait_queue(&partition->wq, &wait);
			spin_unlock_bh(partition->mutex);
			schedule();
			remove_wait_queue(&partition->wq, &wait);
			timeo = jiffies + HZ;
			goto retry;
		}

		/* Erasing/writing is not allowed in CFI_QUERY */
		spin_lock_bh(chip->mutex);
		if (priv->num_erasing || priv->num_writing) {
			set_current_state(TASK_UNINTERRUPTIBLE);
			add_wait_queue(&chip->wq, &wait);
			spin_unlock_bh(partition->mutex);
			spin_unlock_bh(chip->mutex);
			schedule();
			remove_wait_queue(&chip->wq, &wait);
			timeo = jiffies + HZ;
			goto retry;
		}

		priv->num_writing ++;
		spin_unlock_bh(chip->mutex);
			
		/* Now read the data required from this flash */
       
		cfi_send_gen_cmd(0x90, 0x55,chip->start, map, cfi, cfi->device_type, NULL);
		while(count && ((offst-base_offst)<reg_sz)){
			*buf=map->read8(map,(chip->start+(extp->ProtRegAddr*ofs_factor)+offst));
			buf++;
			offst++;
			count--;
		}
	       
		/* Don't stay in CFI_QUERY mode */
		cfi_write(map, CMD(0x70), chip->start);

		partition->state=FL_STATUS;
		wake_up(&partition->wq);
		spin_unlock_bh(partition->mutex);

		spin_lock_bh(chip->mutex);
		priv->num_writing --;
		if (!priv->num_writing)
			wake_up(&chip->wq);
		spin_unlock_bh(chip->mutex);

		/* Move on to the next chip */
		chip_num++;
		offst=base_offst;
	
	}
	
 out:	

#if 0
	wake_up(&chip->wq);
#endif

#ifdef CONFIG_PM
	atomic_dec(&(pm_flash.pm_flash_count));
#endif

	return len-count;
}
	
static int cfi_intelext_read_user_prot_reg (struct mtd_info *mtd, loff_t from, size_t len, size_t *retlen, u_char *buf)
{
	struct map_info *map = mtd->priv;
	struct cfi_private *cfi = map->fldrv_priv;
	struct cfi_pri_intelext *extp=cfi->cmdset_priv;
	int base_offst,reg_sz;
	
	/* Check that we actually have some protection registers */
	if(!(extp->FeatureSupport&64)){
		printk(KERN_WARNING "%s: This flash device has no protection data to read!\n",map->name);
		return 0;
	}

	base_offst=(1<<extp->FactProtRegSize);
	reg_sz=(1<<extp->UserProtRegSize);

	return cfi_intelext_read_prot_reg(mtd, from, len, retlen, buf, base_offst, reg_sz);
}

static int cfi_intelext_read_fact_prot_reg (struct mtd_info *mtd, loff_t from, size_t len, size_t *retlen, u_char *buf)
{
	struct map_info *map = mtd->priv;
	struct cfi_private *cfi = map->fldrv_priv;
	struct cfi_pri_intelext *extp=cfi->cmdset_priv;
	int base_offst,reg_sz;
	
	/* Check that we actually have some protection registers */
	if(!(extp->FeatureSupport&64)){
		printk(KERN_WARNING "%s: This flash device has no protection data to read!\n",map->name);
		return 0;
	}

	base_offst=0;
	reg_sz=(1<<extp->FactProtRegSize);

	return cfi_intelext_read_prot_reg(mtd, from, len, retlen, buf, base_offst, reg_sz);
}


static int do_write_oneword(struct map_info *map, struct flchip *chip, unsigned long adr, cfi_word datum)
{
	struct cfi_private *cfi = map->fldrv_priv;
	cfi_word status, status_OK[2], mask;
	unsigned long timeo;
	DECLARE_WAITQUEUE(wait, current);
	int z;
	struct flpartition *partition;
	struct flprivate *priv = (struct flprivate*) chip->priv;

//Susan for pm
#ifdef CONFIG_PM
	atomic_inc(&(pm_flash.pm_flash_count));
#endif

	partition = PARTITION_SEL(chip,adr);
	if (!partition) 
	{
//Susan for pm
#ifdef CONFIG_PM
		atomic_dec(&(pm_flash.pm_flash_count));
#endif
		BUG();
	}

	adr += chip->start;

	/* Let's determine this according to the interleave only once 
	 * 
	 * 0x80 = No active program or erase operations
	 * 0x01 = Program or erase operation in other partition
	 */
	status_OK[0] = CMD(0x80);
	status_OK[1] = CMD(0x01);
	mask = CMD(0x81);

	timeo = jiffies + HZ;
 retry:
	spin_lock_bh(partition->mutex);

	/* Check that the chip's ready to talk to us.
	 * Later, we can actually think about interrupting it
	 * if it's in FL_ERASING state.
	 * Not just yet, though.
	 */
	switch (partition->state) {
	case FL_READY:
		break;
		
	case FL_CFI_QUERY:
	case FL_JEDEC_QUERY:
		cfi_write(map, CMD(0x70), adr);
		partition->state = FL_STATUS;

	case FL_STATUS:
		status = cfi_read(map, adr);
		if ( ((status & mask) == status_OK[0]) || ((status & mask) == status_OK[1]))
			break;
		
		/* Urgh. Chip not yet ready to talk to us. */
		if (time_after(jiffies, timeo)) {
			spin_unlock_bh(partition->mutex);
			printk(KERN_ERR "waiting for chip to be ready timed out in read\n");
#ifdef CONFIG_PM
			atomic_dec(&(pm_flash.pm_flash_count));
#endif
			return -EIO;
		}

		/* Latency issues. Drop the lock, wait a while and retry */
		spin_unlock_bh(partition->mutex);
		cfi_udelay(1);
		goto retry;

	default:
		/* Stick ourselves on a wait queue to be woken when
		   someone changes the status */
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue(&partition->wq, &wait);
		spin_unlock_bh(partition->mutex);
		schedule();
		remove_wait_queue(&partition->wq, &wait);
		timeo = jiffies + HZ;
		goto retry;
	}

	/* check the other partitions are ready */
	spin_lock_bh(chip->mutex);

	if (priv->num_erasing || priv->num_writing) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue(&chip->wq, &wait);
		spin_unlock_bh(partition->mutex);
		spin_unlock_bh(chip->mutex);
		schedule();
		remove_wait_queue(&chip->wq, &wait);
		timeo = jiffies + HZ;
		goto retry;
	}

	priv->num_writing ++;
	spin_unlock_bh(chip->mutex);

	ENABLE_VPP(map);
	cfi_write(map, CMD(0x40), adr);
	cfi_write(map, datum, adr);
	partition->state = FL_WRITING;

#ifdef CONFIG_MTD_CACHED_READS
	/*Invalidate the cache lines possibly affect by this write -- Intel MTD patch*/
	map->invalidate_cache( bulverde_map_cacheable +(adr-chip->start), (unsigned long) map->buswidth );
#endif
	spin_unlock_bh(partition->mutex);
	cfi_udelay(chip->word_write_time);
	spin_lock_bh(partition->mutex);

	timeo = jiffies + (HZ/2);
	z = 0;
	for (;;) {
		if (partition->state != FL_WRITING) {
			/* Someone's suspended the write. Sleep */
			set_current_state(TASK_UNINTERRUPTIBLE);
			add_wait_queue(&partition->wq, &wait);
			spin_unlock_bh(partition->mutex);
			schedule();
			remove_wait_queue(&partition->wq, &wait);
			timeo = jiffies + (HZ / 2); /* FIXME */
			spin_lock_bh(partition->mutex);
			continue;
		}

		status = cfi_read(map, adr);
		if ((status & mask) == status_OK[0])
			break;
		
		/* OK Still waiting */
		if (time_after(jiffies, timeo)) {
			partition->state = FL_STATUS;
			DISABLE_VPP(map);
			spin_unlock_bh(partition->mutex);
			printk(KERN_ERR "waiting for chip to be ready timed out in word write\n");
#ifdef CONFIG_PM
			atomic_dec(&(pm_flash.pm_flash_count));
#endif
			return -EIO;
		}

		/* Latency issues. Drop the lock, wait a while and retry */
		spin_unlock_bh(partition->mutex);
		z++;
		cfi_udelay(1);
		spin_lock_bh(partition->mutex);
	}
	if (!z) {
		chip->word_write_time--;
		if (!chip->word_write_time)
			chip->word_write_time++;
	}
	if (z > 1) 
		chip->word_write_time++;

	/* Done and happy. */
	DISABLE_VPP(map);
	partition->state = FL_STATUS;

	/* wake up the processes */
	spin_lock_bh(chip->mutex);
	priv->num_writing --;
	if (!priv->num_writing) 
		wake_up(&chip->wq);
	spin_unlock_bh(chip->mutex);

	/* check for lock bit */
	if (status & CMD(0x02)) {
		/* clear status */
		cfi_write(map, CMD(0x50), adr);
		/* put back into read status register mode */
		cfi_write(map, CMD(0x70), adr);
		wake_up(&partition->wq);
		spin_unlock_bh(partition->mutex);
#ifdef CONFIG_PM
		atomic_dec(&(pm_flash.pm_flash_count));
#endif
		return -EROFS;
	}
	wake_up(&partition->wq);
	spin_unlock_bh(partition->mutex);
#ifdef CONFIG_PM
	atomic_dec(&(pm_flash.pm_flash_count));
#endif
	return 0;
}


static int cfi_intelext_write_words (struct mtd_info *mtd, loff_t to , size_t len, size_t *retlen, const u_char *buf)
{
	struct map_info *map = mtd->priv;
	struct cfi_private *cfi = map->fldrv_priv;
	int ret = 0;
	int chipnum;
	unsigned long ofs;

	*retlen = 0;
	if (!len)
		return 0;

	chipnum = to >> cfi->chipshift;
	ofs = to  - (chipnum << cfi->chipshift);

	/* If it's not bus-aligned, do the first byte write */
	if (ofs & (CFIDEV_BUSWIDTH-1)) {
		unsigned long bus_ofs = ofs & ~(CFIDEV_BUSWIDTH-1);
		int gap = ofs - bus_ofs;
		int i = 0, n = 0;
		u_char tmp_buf[8];
		cfi_word datum;

		while (gap--)
			tmp_buf[i++] = 0xff;
		while (len && i < CFIDEV_BUSWIDTH)
			tmp_buf[i++] = buf[n++], len--;
		while (i < CFIDEV_BUSWIDTH)
			tmp_buf[i++] = 0xff;

		if (cfi_buswidth_is_2()) {
			datum = *(__u16*)tmp_buf;
		} else if (cfi_buswidth_is_4()) {
			datum = *(__u32*)tmp_buf;
		} else if (cfi_buswidth_is_8()) {
			datum = *(__u64*)tmp_buf;
		} else {
			return -EINVAL;  /* should never happen, but be safe */
		}

		ret = do_write_oneword(map, &cfi->chips[chipnum],
					       bus_ofs, datum);
		if (ret) 
			return ret;
		
		ofs += n;
		buf += n;
		(*retlen) += n;

		if (ofs >> cfi->chipshift) {
			chipnum ++; 
			ofs = 0;
			if (chipnum == cfi->numchips)
				return 0;
		}
	}
	
	while(len >= CFIDEV_BUSWIDTH) {
		cfi_word datum;

		if (cfi_buswidth_is_1()) {
			datum = *(__u8*)buf;
		} else if (cfi_buswidth_is_2()) {
			datum = *(__u16*)buf;
		} else if (cfi_buswidth_is_4()) {
			datum = *(__u32*)buf;
		} else if (cfi_buswidth_is_8()) {
			datum = *(__u64*)buf;
		} else {
			return -EINVAL;
		}

		ret = do_write_oneword(map, &cfi->chips[chipnum],
				ofs, datum);
		if (ret)
			return ret;

		ofs += CFIDEV_BUSWIDTH;
		buf += CFIDEV_BUSWIDTH;
		(*retlen) += CFIDEV_BUSWIDTH;
		len -= CFIDEV_BUSWIDTH;

		if (ofs >> cfi->chipshift) {
			chipnum ++; 
			ofs = 0;
			if (chipnum == cfi->numchips)
				return 0;
		}
	}

	if (len & (CFIDEV_BUSWIDTH-1)) {
		int i = 0, n = 0;
		u_char tmp_buf[8];
		cfi_word datum;

		while (len--)
			tmp_buf[i++] = buf[n++];
		while (i < CFIDEV_BUSWIDTH)
			tmp_buf[i++] = 0xff;

		if (cfi_buswidth_is_2()) {
			datum = *(__u16*)tmp_buf;
		} else if (cfi_buswidth_is_4()) {
			datum = *(__u32*)tmp_buf;
		} else if (cfi_buswidth_is_8()) {
			datum = *(__u64*)tmp_buf;
		} else {
			return -EINVAL;  /* should never happen, but be safe */
		}

		ret = do_write_oneword(map, &cfi->chips[chipnum],
					       ofs, datum);
		if (ret) 
			return ret;
		
		(*retlen) += n;
	}

	return 0;
}


static inline int do_write_buffer(struct map_info *map, struct flchip *chip, 
				  unsigned long adr, const u_char *buf, int len)
{
	struct cfi_private *cfi = map->fldrv_priv;
	cfi_word status, status_OK[2], mask;
	unsigned long cmd_adr, timeo;
	DECLARE_WAITQUEUE(wait, current);
	int wbufsize, z;
	struct flpartition *partition;
	struct flprivate *priv = (struct flprivate*) chip->priv;

//Susan for pm
#ifdef CONFIG_PM
	atomic_inc(&(pm_flash.pm_flash_count));
#endif

	partition = PARTITION_SEL(chip,adr);

	wbufsize = CFIDEV_INTERLEAVE << cfi->cfiq->MaxBufWriteSize;
	adr += chip->start;
	cmd_adr = adr & ~(wbufsize-1);
	
	/* Let's determine this according to the interleave only once 
	 * 
	 * 0x80 = No active program or erase operations
	 * 0x01 = Program or erase operation in other partition
	 */
	status_OK[0] = CMD(0x80);
	status_OK[1] = CMD(0x01);
	mask = CMD(0x81);

	timeo = jiffies + HZ;
 retry:
	spin_lock_bh(partition->mutex);

	/* Check that the chip's ready to talk to us.
	 * Later, we can actually think about interrupting it
	 * if it's in FL_ERASING state.
	 * Not just yet, though.
	 */
	switch (partition->state) {
	case FL_READY:
	case FL_CFI_QUERY:
	case FL_JEDEC_QUERY:
		cfi_write(map, CMD(0x70), cmd_adr);
		partition->state = FL_STATUS;

	case FL_STATUS:
		status = cfi_read(map, cmd_adr);
		if (((status & mask) == status_OK[0]) || ((status & mask) == status_OK[1]))
			break;
		/* Urgh. Chip not yet ready to talk to us. */
		if (time_after(jiffies, timeo)) {
			spin_unlock_bh(partition->mutex);
			printk(KERN_ERR "waiting for chip to be ready timed out in buffer write\n");
#ifdef CONFIG_PM
			atomic_dec(&(pm_flash.pm_flash_count));
#endif
			return -EIO;
		}

		/* Latency issues. Drop the lock, wait a while and retry */
		spin_unlock_bh(partition->mutex);
		cfi_udelay(1);
		goto retry;

	default:
		/* Stick ourselves on a wait queue to be woken when
		   someone changes the status */
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue(&partition->wq, &wait);
		spin_unlock_bh(partition->mutex);
		schedule();
		remove_wait_queue(&partition->wq, &wait);
		timeo = jiffies + HZ;
		goto retry;
	}

	/* check the other partitions are ready */
	spin_lock_bh(chip->mutex);

	if (priv->num_erasing || priv->num_writing) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue(&chip->wq, &wait);
		spin_unlock_bh(partition->mutex);
		spin_unlock_bh(chip->mutex);
		schedule();
		remove_wait_queue(&chip->wq, &wait);
		timeo = jiffies + HZ;
		goto retry;
	}

	priv->num_writing ++;
	spin_unlock_bh(chip->mutex);

	/* We know we're now in FL_STATUS mode, and 'status' is current */
	/* ?.8 of the 28FxxxJ3A datasheet says "Any time SR.4 and/or SR.5 is set
	   [...], the device will not accept any more Write to Buffer commands". 
	   So we must check here and reset those bits if they're set. Otherwise
	   we're just pissing in the wind */
	if (status & CMD(0x30)) {
		printk(KERN_WARNING "SR.4 or SR.5 bits set in buffer write (status %x). Clearing.\n", status);
		cfi_write(map, CMD(0x50), cmd_adr);
		cfi_write(map, CMD(0x70), cmd_adr);
	}
	ENABLE_VPP(map);
	partition->state = FL_WRITING_TO_BUFFER;

	z = 0;
	for (;;) {
		cfi_write(map, CMD(0xe8), cmd_adr);

		status = cfi_read(map, cmd_adr);
		if ((status & mask) == status_OK[0])
			break;

//Susan -- to avoid two-cmd sequence be interrupted by one write cmd		spin_unlock_bh(partition->mutex);
//Susan -- to avoid two-cmd sequence be interrupted by one write cmd		cfi_udelay(1);
//Susan -- to avoid two-cmd sequence be interrupted by one write cmd		spin_lock_bh(partition->mutex);

		if (++z > 20) {
			/* Argh. Not ready for write to buffer */
			cfi_write(map, CMD(0x70), cmd_adr);
			partition->state = FL_STATUS;
			DISABLE_VPP(map);
			printk(KERN_ERR "Chip not ready for buffer write. Xstatus = %llx, status = %llx\n", (__u64)status, (__u64)cfi_read(map, cmd_adr));
			/* Odd. Clear status bits */
			cfi_write(map, CMD(0x50), cmd_adr);
			cfi_write(map, CMD(0x70), cmd_adr);
			spin_unlock_bh(partition->mutex);
#ifdef CONFIG_PM
			atomic_dec(&(pm_flash.pm_flash_count));
#endif
			return -EIO;
		}
	}

	/* Write length of data to come */
	cfi_write(map, CMD(len/CFIDEV_BUSWIDTH-1), cmd_adr );

	/* Write data */
	for (z = 0; z < len; z += CFIDEV_BUSWIDTH) {
		if (cfi_buswidth_is_1()) {
			map->write8 (map, *((__u8*)buf)++, adr+z);
		} else if (cfi_buswidth_is_2()) {
			map->write16 (map, *((__u16*)buf)++, adr+z);
		} else if (cfi_buswidth_is_4()) {
			map->write32 (map, *((__u32*)buf)++, adr+z);
		} else if (cfi_buswidth_is_8()) {
			map->write64 (map, *((__u64*)buf)++, adr+z);
		} else {
			DISABLE_VPP(map);
#ifdef CONFIG_PM
			atomic_dec(&(pm_flash.pm_flash_count));
#endif
			return -EINVAL;
		}
	}
	/* GO GO GO */
	cfi_write(map, CMD(0xd0), cmd_adr);
	partition->state = FL_WRITING;

#ifdef CONFIG_MTD_CACHED_READS
	/*Invalidate the cache lines possibly affect by this write*/
	map->invalidate_cache( bulverde_map_cacheable +(adr-chip->start), (unsigned long) len );
#endif
	spin_unlock_bh(partition->mutex);
	cfi_udelay(chip->buffer_write_time);
	spin_lock_bh(partition->mutex);

	timeo = jiffies + (HZ/2);
	z = 0;
	for (;;) {
		if (partition->state != FL_WRITING) {
			/* Someone's suspended the write. Sleep */
			set_current_state(TASK_UNINTERRUPTIBLE);
			add_wait_queue(&partition->wq, &wait);
			spin_unlock_bh(partition->mutex);
			schedule();
			remove_wait_queue(&partition->wq, &wait);
			timeo = jiffies + (HZ / 2); /* FIXME */
			spin_lock_bh(partition->mutex);
			continue;
		}

		status = cfi_read(map, cmd_adr);
		if ((status & mask) == status_OK[0])
			break;

		/* OK Still waiting */
		if (time_after(jiffies, timeo)) {
			partition->state = FL_STATUS;
			DISABLE_VPP(map);
			spin_unlock_bh(partition->mutex);
			printk(KERN_ERR "waiting for chip to be ready timed out in bufwrite\n");
#ifdef CONFIG_PM
			atomic_dec(&(pm_flash.pm_flash_count));
#endif
			return -EIO;
		}
		
		/* Latency issues. Drop the lock, wait a while and retry */
		spin_unlock_bh(partition->mutex);
		cfi_udelay(1);
		z++;
		spin_lock_bh(partition->mutex);
	}
	if (!z) {
		chip->buffer_write_time--;
		if (!chip->buffer_write_time)
			chip->buffer_write_time++;
	}
	if (z > 1) 
		chip->buffer_write_time++;

	/* Done and happy. */
	DISABLE_VPP(map);
	partition->state = FL_STATUS;

	/* wake up the processes */
	spin_lock_bh(chip->mutex);
	priv->num_writing --;
	if (!priv->num_writing) 
		wake_up(&chip->wq);
	spin_unlock_bh(chip->mutex);

	/* check for lock bit */
	if (status & CMD(0x02)) {
		/* clear status */
		cfi_write(map, CMD(0x50), cmd_adr);
		/* put back into read status register mode */
		cfi_write(map, CMD(0x70), adr);
		wake_up(&partition->wq);
		spin_unlock_bh(partition->mutex);
#ifdef CONFIG_PM
		atomic_dec(&(pm_flash.pm_flash_count));
#endif
		return -EROFS;
	}
	wake_up(&partition->wq);
	spin_unlock_bh(partition->mutex);
#ifdef CONFIG_PM
	atomic_dec(&(pm_flash.pm_flash_count));
#endif
	return 0;
}

static int cfi_intelext_write_buffers (struct mtd_info *mtd, loff_t to, 
				       size_t len, size_t *retlen, const u_char *buf)
{
	struct map_info *map = mtd->priv;
	struct cfi_private *cfi = map->fldrv_priv;
	int wbufsize = CFIDEV_INTERLEAVE << cfi->cfiq->MaxBufWriteSize;
	int ret = 0;
	int chipnum;
	unsigned long ofs;

	*retlen = 0;
	if (!len)
		return 0;

	chipnum = to >> cfi->chipshift;
	ofs = to  - (chipnum << cfi->chipshift);

	/* If it's not bus-aligned, do the first word write */
	if (ofs & (CFIDEV_BUSWIDTH-1)) {
		size_t local_len = (-ofs)&(CFIDEV_BUSWIDTH-1);
		if (local_len > len)
			local_len = len;
		ret = cfi_intelext_write_words(mtd, to, local_len,
					       retlen, buf);
		if (ret)
			return ret;
		ofs += local_len;
		buf += local_len;
		len -= local_len;

		if (ofs >> cfi->chipshift) {
			chipnum ++;
			ofs = 0;
			if (chipnum == cfi->numchips)
				return 0;
		}
	}

	/* Write buffer is worth it only if more than one word to write... */
	while(len > CFIDEV_BUSWIDTH) {
		/* We must not cross write block boundaries */
		int size = wbufsize - (ofs & (wbufsize-1));

		if (size > len)
			size = len & ~(CFIDEV_BUSWIDTH-1);
		ret = do_write_buffer(map, &cfi->chips[chipnum], 
				      ofs, buf, size);
		if (ret)
			return ret;

		ofs += size;
		buf += size;
		(*retlen) += size;
		len -= size;

		if (ofs >> cfi->chipshift) {
			chipnum ++; 
			ofs = 0;
			if (chipnum == cfi->numchips)
				return 0;
		}
	}

	/* ... and write the remaining bytes */
	if (len > 0) {
		size_t local_retlen;
		ret = cfi_intelext_write_words(mtd, ofs + (chipnum << cfi->chipshift),
					       len, &local_retlen, buf);
		if (ret)
			return ret;
		(*retlen) += local_retlen;
	}

	return 0;
}

typedef int (*varsize_frob_t)(struct map_info *map, struct flchip *chip,
			      unsigned long adr, void *thunk);

static int cfi_intelext_varsize_frob(struct mtd_info *mtd, varsize_frob_t frob,
				     loff_t ofs, size_t len, void *thunk)
{
	struct map_info *map = mtd->priv;
	struct cfi_private *cfi = map->fldrv_priv;
	unsigned long adr;
	int chipnum, ret = 0;
	int i, first;
	struct mtd_erase_region_info *regions = mtd->eraseregions;

	if (ofs > mtd->size)
		return -EINVAL;

	if ((len + ofs) > mtd->size)
		return -EINVAL;

	/* Check that both start and end of the requested erase are
	 * aligned with the erasesize at the appropriate addresses.
	 */

	i = 0;

	/* Skip all erase regions which are ended before the start of 
	   the requested erase. Actually, to save on the calculations,
	   we skip to the first erase region which starts after the
	   start of the requested erase, and then go back one.
	*/
	
	while (i < mtd->numeraseregions && ofs >= regions[i].offset)
	       i++;
	i--;

	/* OK, now i is pointing at the erase region in which this 
	   erase request starts. Check the start of the requested
	   erase range is aligned with the erase size which is in
	   effect here.
	*/

	if (ofs & (regions[i].erasesize-1))
		return -EINVAL;

	/* Remember the erase region we start on */
	first = i;

	/* Next, check that the end of the requested erase is aligned
	 * with the erase region at that address.
	 */

	while (i<mtd->numeraseregions && (ofs + len) >= regions[i].offset)
		i++;

	/* As before, drop back one to point at the region in which
	   the address actually falls
	*/
	i--;
	
	if ((ofs + len) & (regions[i].erasesize-1))
		return -EINVAL;

	chipnum = ofs >> cfi->chipshift;
	adr = ofs - (chipnum << cfi->chipshift);

	i=first;

	while(len) {
		ret = (*frob)(map, &cfi->chips[chipnum], adr, thunk);
		
		if (ret)
			return ret;

		adr += regions[i].erasesize;
		len -= regions[i].erasesize;

		if (adr % (1<< cfi->chipshift) == ((regions[i].offset + (regions[i].erasesize * regions[i].numblocks)) %( 1<< cfi->chipshift)))
			i++;

		if (adr >> cfi->chipshift) {
			adr = 0;
			chipnum++;
			
			if (chipnum >= cfi->numchips)
			break;
		}
	}

	return 0;
}


static int do_erase_oneblock(struct map_info *map, struct flchip *chip, unsigned long adr, void *thunk)
{
	struct cfi_private *cfi = map->fldrv_priv;
	cfi_word status, status_OK[2], mask;
	unsigned long timeo;
	int retries = 3;
	DECLARE_WAITQUEUE(wait, current);
	int ret = 0;
	struct flpartition *partition;
	struct flprivate *priv = (struct flprivate*) chip->priv;

//Susan for pm
#ifdef CONFIG_PM
	atomic_inc(&(pm_flash.pm_flash_count));
#endif

	partition = PARTITION_SEL(chip,adr);
	if (!partition)
	{
#ifdef CONFIG_PM
		atomic_dec(&(pm_flash.pm_flash_count));
#endif
		BUG();
	}

	adr += chip->start;

	/* Let's determine this according to the interleave only once 
	 * 
	 * 0x80 = No active program or erase operations
	 * 0x01 = Program or erase operation in other partition
	 */
	status_OK[0] = CMD(0x80);
	status_OK[1] = CMD(0x01);
	mask = CMD(0x81);

	timeo = jiffies + HZ;
retry:
	spin_lock_bh(partition->mutex);

	/* Check that the chip's ready to talk to us. */
	switch (partition->state) {
	case FL_CFI_QUERY:
	case FL_JEDEC_QUERY:
	case FL_READY:
		cfi_write(map, CMD(0x70), adr);
		partition->state = FL_STATUS;

	case FL_STATUS:
		status = cfi_read(map, adr);
		if (((status & mask) == status_OK[0]) || ((status & mask) == status_OK[1]))
			break;
		
		/* Urgh. Chip not yet ready to talk to us. */
		if (time_after(jiffies, timeo)) {
			spin_unlock_bh(partition->mutex);
			printk(KERN_ERR "waiting for chip to be ready timed out in erase\n");
#ifdef CONFIG_PM
			atomic_dec(&(pm_flash.pm_flash_count));
#endif
			return -EIO;
		}

		/* Latency issues. Drop the lock, wait a while and retry */
		spin_unlock_bh(partition->mutex);
		cfi_udelay(1);
		goto retry;

	default:
		/* Stick ourselves on a wait queue to be woken when
		   someone changes the status */
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue(&partition->wq, &wait);
		spin_unlock_bh(partition->mutex);
		schedule();
		remove_wait_queue(&partition->wq, &wait);
		timeo = jiffies + HZ;
		goto retry;
	}

	/* check the other partitions are ready */
	spin_lock_bh(chip->mutex);

	if (priv->num_erasing || priv->num_writing) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue(&chip->wq, &wait);
		spin_unlock_bh(partition->mutex);
		spin_unlock_bh(chip->mutex);
		schedule();
		remove_wait_queue(&chip->wq, &wait);
		timeo = jiffies + HZ;
		goto retry;
	}

	priv->num_erasing ++;
	spin_unlock_bh(chip->mutex);
	ENABLE_VPP(map);

	/* Clear the status register first */
	cfi_write(map, CMD(0x50), adr);

	/* Now erase */
	cfi_write(map, CMD(0x20), adr);
	cfi_write(map, CMD(0xD0), adr);
	partition->state = FL_ERASING;
	partition->oldstate = 0;

#ifdef CONFIG_MTD_CACHED_READS
	/*Invalidate the cache lines possibly affect by this erase -- Intel MTD patch*/
	map->invalidate_cache( bulverde_map_cacheable +(adr-chip->start),(unsigned long) thunk );
#endif
	spin_unlock_bh(partition->mutex);
	schedule_timeout(HZ);
	spin_lock_bh(partition->mutex);

	/* FIXME. Use a timer to check this, and return immediately. */
	/* Once the state machine's known to be working I'll do that */

	timeo = jiffies + (HZ*20);
	for (;;) {
		if (partition->state != FL_ERASING) {
			/* Someone's suspended the erase. Sleep */
			set_current_state(TASK_UNINTERRUPTIBLE);
			add_wait_queue(&partition->wq, &wait);
			spin_unlock_bh(partition->mutex);
			schedule();
			remove_wait_queue(&partition->wq, &wait);
			spin_lock_bh(partition->mutex);
			continue;
		}
		if (partition->oldstate) {
			/* This erase was suspended and resumed.
			   Adjust the timeout */
			timeo = jiffies + (HZ*20); /* FIXME */
			partition->oldstate = 0;
		}

		status = cfi_read(map, adr);
		if ((status & mask) == status_OK[0])
			break;
		
		/* OK Still waiting */
		if (time_after(jiffies, timeo)) {
			cfi_write(map, CMD(0x70), adr);
			partition->state = FL_STATUS;
			printk(KERN_ERR "waiting for erase at %08lx to complete timed out. Xstatus = %llx, status = %llx.\n",
			       adr, (__u64)status, (__u64)cfi_read(map, adr));
			/* Clear status bits */
			cfi_write(map, CMD(0x50), adr);
			cfi_write(map, CMD(0x70), adr);
			DISABLE_VPP(map);
			spin_unlock_bh(partition->mutex);
#ifdef CONFIG_PM
			atomic_dec(&(pm_flash.pm_flash_count));
#endif
			return -EIO;
		}
		
		/* Latency issues. Drop the lock, wait a while and retry */
		spin_unlock_bh(partition->mutex);
		cfi_udelay(1);
		spin_lock_bh(partition->mutex);
	}
	
	DISABLE_VPP(map);
	ret = 0;

	/* We've broken this before. It doesn't hurt to be safe */
	cfi_write(map, CMD(0x70), adr);
	partition->state = FL_STATUS;
	status = cfi_read(map, adr);

	/* wake up the processes */
	spin_lock_bh(chip->mutex);
	priv->num_erasing --;
	if (!priv->num_erasing) 
		wake_up(&chip->wq);
	spin_unlock_bh(chip->mutex);

	/* check for lock bit */
	if (status & CMD(0x3a)) {
		unsigned char chipstatus = status;
		if (status != CMD(status & 0xff)) {
			int i;
			for (i = 1; i<CFIDEV_INTERLEAVE; i++) {
				      chipstatus |= status >> (cfi->device_type * 8);
			}
			printk(KERN_WARNING "Status is not identical for all chips: 0x%llx. Merging to give 0x%02x\n", (__u64)status, chipstatus);
		}
		/* Reset the error bits */
		cfi_write(map, CMD(0x50), adr);
		cfi_write(map, CMD(0x70), adr);
		
		if ((chipstatus & 0x30) == 0x30) {
			printk(KERN_NOTICE "Chip reports improper command sequence: status 0x%llx\n", (__u64)status);
			ret = -EIO;
		} else if (chipstatus & 0x02) {
			/* Protection bit set */
			ret = -EROFS;
		} else if (chipstatus & 0x8) {
			/* Voltage */
			printk(KERN_WARNING "Chip reports voltage low on erase: status 0x%llx\n", (__u64)status);
			ret = -EIO;
		} else if (chipstatus & 0x20) {
			if (retries--) {
				printk(KERN_DEBUG "Chip erase failed at 0x%08lx: status 0x%llx. Retrying...\n", adr, (__u64)status);
				timeo = jiffies + HZ;
				partition->state = FL_STATUS;
				spin_unlock_bh(partition->mutex);
				goto retry;
			}
			printk(KERN_DEBUG "Chip erase failed at 0x%08lx: status 0x%llx\n", adr, (__u64)status);
			ret = -EIO;
		}
	}

	wake_up(&partition->wq);
	spin_unlock_bh(partition->mutex);
#ifdef CONFIG_PM
	atomic_dec(&(pm_flash.pm_flash_count));
#endif
	return ret;
}

int cfi_intelext_erase_varsize(struct mtd_info *mtd, struct erase_info *instr)
{
	unsigned long ofs, len;
	int ret;

	ofs = instr->addr;
	len = instr->len;

	ret = cfi_intelext_varsize_frob(mtd, do_erase_oneblock, ofs, len, 0);
	if (ret)
		return ret;

	instr->state = MTD_ERASE_DONE;
	if (instr->callback)
		instr->callback(instr);
	
	return 0;
}

static void cfi_intelext_sync (struct mtd_info *mtd)
{
	struct map_info *map = mtd->priv;
	struct cfi_private *cfi = map->fldrv_priv;
	int i,j;
	struct flchip *chip;
	struct flprivate *priv;
	struct flpartition *partition;
	int ret = 0;

//Susan for pm
#ifdef CONFIG_PM
	atomic_inc(&(pm_flash.pm_flash_count));
#endif

	DECLARE_WAITQUEUE(wait, current);

	for (i=0; !ret && i<cfi->numchips; i++) {
		chip = &cfi->chips[i];

		priv = (struct flprivate*) chip->priv;

		for (j=0; j<priv->num_partitions; j++) {
			partition = priv->partitions + j;
		retry:
			spin_lock_bh(partition->mutex);

			switch(partition->state) {
			case FL_READY:
			case FL_STATUS:
			case FL_CFI_QUERY:
			case FL_JEDEC_QUERY:
				partition->oldstate = partition->state;
				partition->state = FL_SYNCING;
				/* No need to wake_up() on this state change - 
				 * as the whole point is that nobody can do anything
				 * with the chip now anyway.
				 */
			case FL_SYNCING:
				spin_unlock_bh(partition->mutex);
				break;

			default:
				/* Not an idle state */
				add_wait_queue(&partition->wq, &wait);
			
				spin_unlock_bh(partition->mutex);
				schedule();
		        	remove_wait_queue(&partition->wq, &wait);
			
				goto retry;
			}
		}
	}

	/* Unlock the chips again */

	for (i--; i >=0; i--) {
		chip = &cfi->chips[i];
		priv = (struct flprivate*) chip->priv;

		for (j=0; j<priv->num_partitions; j++) {
			partition = priv->partitions + j;

			spin_lock_bh(partition->mutex);
		
			if (partition->state == FL_SYNCING) {
				partition->state = partition->oldstate;
				wake_up(&partition->wq);
			}
			spin_unlock_bh(partition->mutex);
		}
	}
#ifdef CONFIG_PM
		atomic_dec(&(pm_flash.pm_flash_count));
#endif
}

#ifdef DEBUG_LOCK_BITS
static int do_printlockstatus_oneblock(struct map_info *map, struct flchip *chip,
				       unsigned long adr, void *thunk)
{
	struct cfi_private *cfi = map->fldrv_priv;
	int ofs_factor = cfi->interleave * cfi->device_type;

	cfi_send_gen_cmd(0x90, 0x55, 0, map, cfi, cfi->device_type, NULL);
	printk(KERN_DEBUG "block status register for 0x%08lx is %x\n",
	       adr, cfi_read_query(map, adr+(2*ofs_factor)));
	cfi_send_gen_cmd(0xff, 0x55, 0, map, cfi, cfi->device_type, NULL);
	
	return 0;
}
#endif

#define DO_XXLOCK_ONEBLOCK_LOCK		((void *) 1)
#define DO_XXLOCK_ONEBLOCK_UNLOCK	((void *) 2)

static int do_xxlock_oneblock(struct map_info *map, struct flchip *chip,
			      unsigned long adr, void *thunk)
{
	struct cfi_private *cfi = map->fldrv_priv;
	cfi_word status, status_OK[2], mask;
	unsigned long timeo = jiffies + HZ;
	DECLARE_WAITQUEUE(wait, current);
	struct flpartition *partition;

//Susan for pm
#ifdef CONFIG_PM
	atomic_inc(&(pm_flash.pm_flash_count));
#endif
	
	partition = PARTITION_SEL(chip,adr);
	if (!partition)
	{
#ifdef CONFIG_PM
		atomic_dec(&(pm_flash.pm_flash_count));
#endif
		BUG();
	}

	adr += chip->start;

	/* Let's determine this according to the interleave only once 
	 * 
	 * 0x80 = No active program or erase operations
	 * 0x01 = Program or erase operation in other partition
	 */
	status_OK[0] = CMD(0x80);
	status_OK[1] = CMD(0x01);
	mask = CMD(0x81);

	timeo = jiffies + HZ;
retry:
	spin_lock_bh(partition->mutex);

	/* Check that the chip's ready to talk to us. */
	switch (partition->state) {
	case FL_CFI_QUERY:
	case FL_JEDEC_QUERY:
	case FL_READY:
		cfi_write(map, CMD(0x70), adr);
		partition->state = FL_STATUS;

	case FL_STATUS:
		status = cfi_read(map, adr);
		if (((status & mask) == status_OK[0]) ||
		    ((status & mask) == status_OK[1]))
			break;
		
		/* Urgh. Chip not yet ready to talk to us. */
		if (time_after(jiffies, timeo)) {
			spin_unlock_bh(partition->mutex);
			printk(KERN_ERR "%s(): waiting for chip to be ready"
			       " timed out\n", __FUNCTION__);
#ifdef CONFIG_PM
		atomic_dec(&(pm_flash.pm_flash_count));
#endif
			return -EIO;
		}

		/* Latency issues. Drop the lock, wait a while and retry */
		spin_unlock_bh(partition->mutex);
		cfi_udelay(1);
		goto retry;

	default:
		/* Stick ourselves on a wait queue to be woken when
		   someone changes the status */
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue(&partition->wq, &wait);
		spin_unlock_bh(partition->mutex);
		schedule();
		remove_wait_queue(&partition->wq, &wait);
		timeo = jiffies + HZ;
		goto retry;
	}

	ENABLE_VPP(map);
	cfi_write(map, CMD(0x60), adr);

	if (thunk == DO_XXLOCK_ONEBLOCK_LOCK) {
		cfi_write(map, CMD(0x01), adr);
		partition->state = FL_LOCKING;
	} 
	else if (thunk == DO_XXLOCK_ONEBLOCK_UNLOCK) {
		cfi_write(map, CMD(0xD0), adr);
		partition->state = FL_UNLOCKING;
	} 
	else
	{
#ifdef CONFIG_PM
		atomic_dec(&(pm_flash.pm_flash_count));
#endif
		BUG();
	}

	spin_unlock_bh(partition->mutex);
	schedule_timeout(HZ);
	spin_lock_bh(partition->mutex);

	/* FIXME. Use a timer to check this, and return immediately. */
	/* Once the state machine's known to be working I'll do that */

	timeo = jiffies + (HZ*20);
	for (;;) {

		status = cfi_read(map, adr);
		if (((status & mask) == status_OK[0]) || ((status & mask) == status_OK[1]))
			break;
		
		/* OK Still waiting */
		if (time_after(jiffies, timeo)) {
			cfi_write(map, CMD(0x70), adr);
			partition->state = FL_STATUS;
			printk(KERN_ERR "waiting for unlock to complete timed out. Xstatus = %llx, status = %llx.\n", (__u64)status, (__u64)cfi_read(map, adr));
			DISABLE_VPP(map);
			spin_unlock_bh(partition->mutex);
#ifdef CONFIG_PM
			atomic_dec(&(pm_flash.pm_flash_count));
#endif
			return -EIO;
		}
		
		/* Latency issues. Drop the lock, wait a while and retry */
		spin_unlock_bh(partition->mutex);
		cfi_udelay(1);
		spin_lock_bh(partition->mutex);
	}
	
	/* Done and happy. */
	partition->state = FL_STATUS;
	DISABLE_VPP(map);
	wake_up(&partition->wq);
	spin_unlock_bh(partition->mutex);
#ifdef CONFIG_PM
	atomic_dec(&(pm_flash.pm_flash_count));
#endif
	return 0;
}

static int cfi_intelext_lock(struct mtd_info *mtd, loff_t ofs, size_t len)
{
	int ret;

#ifdef DEBUG_LOCK_BITS
	printk(KERN_DEBUG __FUNCTION__ 
	       ": lock status before, ofs=0x%08llx, len=0x%08X\n",
	       ofs, len);
	cfi_intelext_varsize_frob(mtd, do_printlockstatus_oneblock,
				  ofs, len, 0);
#endif

	ret = cfi_intelext_varsize_frob(mtd, do_xxlock_oneblock, 
					ofs, len, DO_XXLOCK_ONEBLOCK_LOCK);
	
#ifdef DEBUG_LOCK_BITS
	printk(KERN_DEBUG __FUNCTION__
	       ": lock status after, ret=%d\n", ret);
	cfi_intelext_varsize_frob(mtd, do_printlockstatus_oneblock,
				  ofs, len, 0);
#endif

	return ret;
}

static int cfi_intelext_unlock(struct mtd_info *mtd, loff_t ofs, size_t len)
{
	int ret;

#ifdef DEBUG_LOCK_BITS
	printk(KERN_DEBUG __FUNCTION__ 
	       ": lock status before, ofs=0x%08llx, len=0x%08X\n",
	       ofs, len);
	cfi_intelext_varsize_frob(mtd, do_printlockstatus_oneblock,
				  ofs, len, 0);
#endif

	ret = cfi_intelext_varsize_frob(mtd, do_xxlock_oneblock,
					ofs, len, DO_XXLOCK_ONEBLOCK_UNLOCK);
	
#ifdef DEBUG_LOCK_BITS
	printk(KERN_DEBUG __FUNCTION__
	       ": lock status after, ret=%d\n", ret);
	cfi_intelext_varsize_frob(mtd, do_printlockstatus_oneblock, 
				  ofs, len, 0);
#endif
	
	return ret;
}

static int msc0_val, sxcnfg_val;
static int mode = 0;

static int cfi_intelext_suspend(struct mtd_info *mtd)
{
#if (0)
struct cfi_private *cfi = map->fldrv_priv;
	int i, j;
	struct flchip *chip;
	struct flprivate *priv;
	struct flpartition *partition;
	int ret = 0;
#endif  //A780/E680 case

	int flags;
	int status = 0;
	struct map_info *map = &bulverde_map;
	struct cfi_private *cfi = map->fldrv_priv;
	
	/* return flash status indicating sleep available or inavailable */
	status = atomic_read(&(pm_flash.pm_flash_count));
	printk(KERN_NOTICE "flash device refer-count(%d) when try to sleep\n",status);
	if (!status) 
	{
		printk("switch to async mode\n");

		local_irq_save(flags);

		/* zxf -- Disable MMU cache and buffer */
		__asm__ __volatile__("mrc	p15, 0, r0, c1, c0, 0\n\
			bic	r0, r0, #0x000c\n\
			b	1f			\n\
			.align	5			\n\
	1:		mcr	p15, 0, r0, c1, c0, 0":::"r0","memory");
				
		msc0_val = MSC0;
		sxcnfg_val = SXCNFG;

//This is for A760			MSC0 = 0x12bb12bb;
//This is for A760				cfi_write(map, CMD(0x60), 0x0001fffe);
//This is for A760				cfi_write(map, CMD(0x03), 0x0001fffe);
//This is for A760				cfi_write(map, CMD(0x60), 0x0101fffe);
//This is for A760				cfi_write(map, CMD(0x03), 0x0101fffe);
		MSC0 = 0x128b128b;
		cfi_write(map, CMD(0x60), 0x00017f9e);
		cfi_write(map, CMD(0x03), 0x00017f9e);

		SXCNFG = 0;
		MDREFR &= ~(MDREFR_K0DB2 | MDREFR_K0RUN | MDREFR_E0PIN);

//no necessary			cfi_write(map, CMD(0xff), 0x0);
//no necessary			cfi_write(map, CMD(0xff), 0x01000000);

		/* zxf -- Enable MMU cache and buffer */
		__asm__ __volatile__("mrc	p15, 0, r0, c1, c0, 0\n\
			orr	r0, r0, #0x000c\n\
			b	2f			\n\
			.align 5			\n\
	2:		mcr	p15, 0, r0, c1, c0, 0":::"r0","memory");			

		local_irq_restore(flags);
		mode = 1;
	}
	
	return status;

#if (0)        //Susan -- use cfi_intelext_pm_ezx suspend function
	for (i=0; !ret && i<cfi->numchips; i++) {
		chip = &cfi->chips[i];
		priv = (struct flprivate*) chip->priv;

		for (j=0; j<priv->num_partitions; j++) {
			partition = priv->partitions + j;
			spin_lock_bh(partition->mutex);

			switch(partition->state) {
			case FL_READY:
			case FL_STATUS:
			case FL_CFI_QUERY:
			case FL_JEDEC_QUERY:
				partition->oldstate = partition->state;
				partition->state = FL_PM_SUSPENDED;
				/* No need to wake_up() on this state change - 
				 * as the whole point is that nobody can do anything
				 * with the chip now anyway.
				 */
			case FL_PM_SUSPENDED:
				break;

			default:
				ret = -EAGAIN;
				break;
			}
			spin_unlock_bh(partition->mutex);
		}
	}

	if (!ret) goto done;

	/* Unlock the chips again */
	for (i--; i >=0; i--) {
		chip = &cfi->chips[i];
		priv = (struct flprivate*) chip->priv;
			
		for (j=0; j<priv->num_partitions; j++) {
			partition = priv->partitions + j;

			spin_lock_bh(partition->mutex);
			
			if (partition->state == FL_PM_SUSPENDED) {
				/* No need to force it into a known state here,
				   because we're returning failure, and it didn't
				   get power cycled */
				partition->state = partition->oldstate;
				wake_up(&partition->wq);
			}
			spin_unlock_bh(partition->mutex);
		}
	}
	
done:

	return ret;
#endif  //#if (0)
}

static void cfi_intelext_resume(struct mtd_info *mtd)
{
#if (0)
	struct map_info *map = mtd->priv;
	struct cfi_private *cfi = map->fldrv_priv;
	int i,j;
	struct flchip *chip;
	struct flprivate *priv;
	struct flpartition *partition;
#endif

#ifdef CONFIG_ARCH_MAINSTONE
//ss	extern void bulverde_mtd_unlock_all(void);
#endif

	int flags;
	int status = 0;
	struct map_info *map = &bulverde_map;
	struct cfi_private *cfi = map->fldrv_priv;

	if (mode) 
	{
		printk("switch to sync mode\n");

		local_irq_save(flags);

		/* zxf -- Disable MMU cache and buffer */
		__asm__ __volatile__("mrc	p15, 0, r0, c1, c0, 0\n\
			bic	r0, r0, #0x000c\n\
			b	1f			\n\
			.align	5			\n\
	1:		mcr	p15, 0, r0, c1, c0, 0":::"r0","memory");

		MSC0 = msc0_val;
		MDREFR |= (MDREFR_K0DB2 | MDREFR_K0RUN | MDREFR_E0PIN);
			
		cfi_write(map, CMD(0x60), 0x00004984);
		cfi_write(map, CMD(0x03), 0x00004984);

		SXCNFG = sxcnfg_val;
	
		printk(KERN_NOTICE "Unlock after reset -- begin(%ld)\n", jiffies);
		status = cfi_intelext_unlockdown_L18(ezx_mymtd);
		printk(KERN_NOTICE "Unlock after reset -- end(%ld)\n", jiffies);
	
		if (status)
			printk(KERN_NOTICE "Unlock flash fail after reset flash.\n");
				
		/* zxf -- Enable MMU cache and buffer */
		__asm__ __volatile__("mrc	p15, 0, r0, c1, c0, 0\n\
			orr	r0, r0, #0x000c\n\
			b	2f			\n\
			.align	5			\n\
	2:		mcr	p15, 0, r0, c1, c0, 0":::"r0","memory");
	
		local_irq_restore(flags);
		mode = 0;

#if (0)		  //Susan -- use cfi_intelext_pm_ezx resume function which contains lockdown/unlock
	for (i=0; i<cfi->numchips; i++) {
	
		chip = &cfi->chips[i];
		priv = (struct flprivate*) chip->priv;

		for (j=0; j<priv->num_partitions; j++) {
			partition = priv->partitions + j;

			spin_lock_bh(partition->mutex);
		
			/* Go to known state. Chip may have been power cycled */
			if (partition->state == FL_PM_SUSPENDED) {
				// cfi_write(map, CMD(0xFF), 0);  //Susan -- is it an issue??
				cfi_write(map, CMD(0xFF), chip->start + partition->offset );
				partition->state = FL_READY;
				wake_up(&partition->wq);
			}

			spin_unlock_bh(partition->mutex);
		}
	}
#endif

#ifdef CONFIG_ARCH_MAINSTONE
//ss	bulverde_mtd_unlock_all();
#endif
		
	return status;
}

static void cfi_intelext_destroy(struct mtd_info *mtd)
{
	struct map_info *map = mtd->priv;
	struct cfi_private *cfi = map->fldrv_priv;
	kfree(cfi->cmdset_priv);
	kfree(cfi);
}

/* Added by Susan for L18 UNLOCK/LOCKDOWN mechanism */
int cfi_intelext_unlockdown_L18(struct mtd_info *mymtd)
{
	struct map_info *map;
	unsigned short i, j;
	struct cfi_private *cfi;
	unsigned short npartitions = 0;
	struct flpartition *this_partition = NULL;
	struct flchip *this_chip = NULL;
	struct flprivate *this_flprivate = NULL;
	
	__u32 timeout = TIME_OUT;
	__u32 flash_status;
	__u32 OK_status = CMD(0x80);

//Susan for pm
#ifdef CONFIG_PM
	atomic_inc(&(pm_flash.pm_flash_count));
#endif

	map = mymtd->priv;
	cfi = (struct cfi_private *)(map->fldrv_priv);

	#ifdef CONFIG_L18_DEBUG
	printk("Enter into <cfi_intelext_unlockdown_L18>\n");
	#endif

	cfi_write(map, CMD(0x70), 0);
	flash_status = cfi_read(map, 0);
	cfi_write(map, CMD(0xff), 0);
	
	if ( (flash_status & CMD(0x80)) != OK_status )
	{
	#ifdef CONFIG_L18_DEBUG
	printk("cfi_intelext_unlockdown_L18: initialization status wrong\n");
	#endif
		cfi_write(map, CMD(0x50), 0);  /* Clear status register */
#ifdef CONFIG_PM
		atomic_dec(&(pm_flash.pm_flash_count));
#endif
		return -EINVAL;
	}

	for ( i = 0; i < mymtd->numeraseregions; i ++ )
	{
		int j;
		unsigned short ebnums;
		unsigned long the_ebsize = 0;
		unsigned long the_offset = 0;

		ebnums = mymtd->eraseregions[i].numblocks;
		the_ebsize = mymtd->eraseregions[i].erasesize;
		the_offset = mymtd->eraseregions[i].offset;
		
		for (j = 0; j < ebnums; j ++)
		{
			if ( (the_offset >= L18_RW_REGION_START) && (the_offset < L18_RW_REGION_END) )
			{
				/* unlock it */
				cfi_write(map, CMD(0x60), the_offset);
				cfi_write(map, CMD(0xD0), the_offset);

				while ( -- timeout )
				{
					cfi_write(map, CMD(0x70), the_offset);
					flash_status = cfi_read(map, the_offset);
					if ( (flash_status & CMD(0x80)) == OK_status )
						break;
				}

				cfi_write(map, CMD(0xff), the_offset);  /* Force flash returns to read-array mode */
				if (!timeout)
				{
					#ifdef CONFIG_L18_DEBUG
					printk("cfi_intelext_unlockdown_L18: unlock failed at %x\n", the_offset);
					#endif
					cfi_write(map, CMD(0x50), the_offset);  /* Clear status register before we leave here*/
#ifdef CONFIG_PM
					atomic_dec(&(pm_flash.pm_flash_count));
#endif
					return -EINVAL;
				}
				#ifdef CONFIG_L18_DEBUG
				printk("cfi_intelext_unlockdown_L18: unlock suceed at %x\n", the_offset);
				#endif
			}
			else
			{
				/* lock down it */
				cfi_write(map, CMD(0x60), the_offset);
				cfi_write(map, CMD(0x2F), the_offset);

				while ( -- timeout )
				{
					cfi_write(map, CMD(0x70), the_offset);
					flash_status = cfi_read(map, the_offset);
					if ( (flash_status & CMD(0x80)) == OK_status )
						break;
				}

				cfi_write(map, CMD(0xff), the_offset);  /* Force flash returns to read-array mode */
				if (!timeout)
				{
					#ifdef CONFIG_L18_DEBUG
					printk("cfi_intelext_unlockdown_L18: lockdown failed at %x\n", the_offset);
					#endif
					cfi_write(map, CMD(0x50), the_offset);  /* Clear status register before we leave here*/
#ifdef CONFIG_PM
					atomic_dec(&(pm_flash.pm_flash_count));
#endif
					return -EINVAL;
				}
				#ifdef CONFIG_L18_DEBUG
				printk("cfi_intelext_unlockdown_L18: lockdown succeed at %x\n", the_offset);
				#endif
			}
			the_offset += the_ebsize;
			timeout = TIME_OUT;
		}

		cfi_write(map, CMD(0x50), (the_offset - the_ebsize));  /* clear status register when we leave this region */
		cfi_write(map, CMD(0xFF), (the_offset - the_ebsize));  /* Make sure the related partition to be in read-array mode */
	}

	/* Resume each partition's state to be the same as before entering sleep mode */
	this_chip = cfi->chips;
	this_flprivate = (struct flprivate *)(this_chip->priv);
	npartitions = this_flprivate->num_partitions;

	for ( i = 0 ; i < npartitions ; i ++ )
	{
		this_partition = (struct flpartition *)(&(this_flprivate->partitions[i]));
//the old method		if (this_partition->state == FL_STATUS)
//the old method			cfi_write(map, CMD(0x70), (this_partition->offset));
		this_partition->state = FL_READY;
		// For other case except for FL_READY, there must be something error somehow, ignore it at this point //
	}
	
#ifdef CONFIG_PM
	atomic_dec(&(pm_flash.pm_flash_count));
#endif
	return 0;
}

int cfi_intelext_read_roflash(void *priv_map, unsigned long from, size_t len, size_t *retlen, u_char *buf)
{
	struct map_info *map = (struct map_info *)(priv_map);
	struct cfi_private *cfi = map->fldrv_priv;
	unsigned long ofs;
	int chipnum;
	int ret = 0;

	/* ofs: offset within the first chip that the first read should start */
	chipnum = (from >> cfi->chipshift);
	ofs = from - (chipnum <<  cfi->chipshift);

	*retlen = 0;

	while (len) {
		unsigned long thislen;

		if (chipnum >= cfi->numchips)
			break;

		if ((len + ofs -1) >> cfi->chipshift)
			thislen = (1<<cfi->chipshift) - ofs;
		else
			thislen = len;

		ret = do_read_onechip(map, &cfi->chips[chipnum], ofs, thislen, buf);
		if (ret)
			break;

		*retlen += thislen;
		len -= thislen;
		buf += thislen;
		
		ofs = 0;
		chipnum++;
	}
	return ret;
}


#ifdef CONFIG_PM

static int cfi_intelext_pm_ezx(struct pm_dev *dev, pm_request_t rqst, void *data)
{
	int flags;
	int status = 0;
	struct map_info *map = &bulverde_map;
	struct cfi_private *cfi = map->fldrv_priv;
	
	switch (rqst) 
	{
	case PM_SUSPEND: /* return flash status indicating sleep available or inavailable */
		status = atomic_read(&(pm_flash.pm_flash_count));
		printk(KERN_NOTICE "flash device refer-count(%d) when try to sleep\n",status);
		if (!status) {
			printk("switch to async mode\n");

			local_irq_save(flags);

			/* zxf -- Disable MMU cache and buffer */
			__asm__ __volatile__("mrc	p15, 0, r0, c1, c0, 0\n\
				bic	r0, r0, #0x000c\n\
				mcr	p15, 0, r0, c1, c0, 0":::"r0","memory");
				
			msc0_val = MSC0;
			sxcnfg_val = SXCNFG;

//This is for A760			MSC0 = 0x12bb12bb;
//This is for A760				cfi_write(map, CMD(0x60), 0x0001fffe);
//This is for A760				cfi_write(map, CMD(0x03), 0x0001fffe);
//This is for A760				cfi_write(map, CMD(0x60), 0x0101fffe);
//This is for A760				cfi_write(map, CMD(0x03), 0x0101fffe);
//			MSC0 = 0x128b128b;
			MSC0 = 0x7ff07ff8;
			cfi_write(map, CMD(0x60), 0x00017f9e);
			cfi_write(map, CMD(0x03), 0x00017f9e);

			SXCNFG = 0;
			MDREFR &= ~(MDREFR_K0DB2 | MDREFR_K0RUN | MDREFR_E0PIN);

//no necessary			cfi_write(map, CMD(0xff), 0x0);
//no necessary			cfi_write(map, CMD(0xff), 0x01000000);

			/* zxf -- Enable MMU cache and buffer */
			__asm__ __volatile__("mrc	p15, 0, r0, c1, c0, 0\n\
				orr	r0, r0, #0x000c\n\
				mcr	p15, 0, r0, c1, c0, 0":::"r0","memory");			


			local_irq_restore(flags);
			mode = 1;
		}
		return status;
		
		break;
	case PM_RESUME:  /* flash reset after sleep */
		if (mode) {
			printk("switch to sync mode\n");

			local_irq_save(flags);

			/* zxf -- Disable MMU cache and buffer */
			__asm__ __volatile__("mrc	p15, 0, r0, c1, c0, 0\n\
				bic	r0, r0, #0x000c\n\
				mcr	p15, 0, r0, c1, c0, 0":::"r0","memory");

			MSC0 = msc0_val;
			MDREFR |= (MDREFR_K0DB2 | MDREFR_K0RUN | MDREFR_E0PIN);
			
			cfi_write(map, CMD(0x60), 0x00004984);
			cfi_write(map, CMD(0x03), 0x00004984);

			SXCNFG = sxcnfg_val;
	
			printk(KERN_NOTICE "Unlock after reset -- begin(%ld)\n", jiffies);
			status = cfi_intelext_unlockdown_L18(ezx_mymtd);
			printk(KERN_NOTICE "Unlock after reset -- end(%ld)\n", jiffies);
	
			if (status)
				printk(KERN_NOTICE "Unlock flash fail after reset flash.\n");
				
			/* zxf -- Enable MMU cache and buffer */
			__asm__ __volatile__("mrc	p15, 0, r0, c1, c0, 0\n\
				orr	r0, r0, #0x000c\n\
				mcr	p15, 0, r0, c1, c0, 0":::"r0","memory");
	
			local_irq_restore(flags);
			mode = 0;
		}
			
		break;
	}
	return status;
}

#endif


static char im_name_1_mp[]="cfi_cmdset_0001_mp";

static int __init cfi_intelext_init(void)
{
	inter_module_register(im_name_1_mp, THIS_MODULE, &cfi_cmdset_0001_mp);

	/* Added by Susan for initialization of pm_flash device */
#ifdef CONFIG_PM
//ss -- replace cfi_intelext_suspend/resume with cfi_intelext_pm_ezx

//ss	pm_flash.pm_dev = pm_register(PM_SYS_DEV, 0, cfi_intelext_pm_ezx);
	atomic_set(&(pm_flash.pm_flash_count), 0);
#endif

	return 0;
}

static void __exit cfi_intelext_exit(void)
{
	inter_module_unregister(im_name_1_mp);
}

module_init(cfi_intelext_init);
module_exit(cfi_intelext_exit);

#endif //CONFIG_ARCH_EZX_A780 || CONFIG_ARCH_EZX_E680


MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Woodhouse <dwmw2@infradead.org> et al.");
MODULE_DESCRIPTION("MTD chip driver for Intel/Sharp flash chips");
