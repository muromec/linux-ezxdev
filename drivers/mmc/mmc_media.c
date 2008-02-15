/*
 * Block driver for media (i.e., flash cards)
 *
 * Copyright 2002 Hewlett-Packard Company
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 * HEWLETT-PACKARD COMPANY MAKES NO WARRANTIES, EXPRESSED OR IMPLIED,
 * AS TO THE USEFULNESS OR CORRECTNESS OF THIS CODE OR ITS
 * FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 * Many thanks to Alessandro Rubini and Jonathan Corbet!
 *
 * Author:  Andrew Christian
 *          28 May 2002
 */
 
/*
 * Copyright 2004-2005 Motorola, Inc. All Rights Reserved.
 * Revision History:
                    Modification    
 Changed by            Date             Description of Changes
----------------   ------------      -------------------------
Zhu Zhifu           05/27/2004         change for sync fs when close device
jiang lili          04/15/2005         change for ioctl
*/ 

#include <linux/config.h>
#include <linux/module.h>

#include <linux/sched.h>
#include <linux/kernel.h> /* printk() */
#include <linux/fs.h>     /* everything... */
#include <linux/errno.h>  /* error codes */
#include <linux/types.h>  /* size_t */
#include <linux/fcntl.h>  /* O_ACCMODE */
#include <linux/hdreg.h>  /* HDIO_GETGEO */
#include <linux/init.h>
#include <linux/devfs_fs_kernel.h>
#include <linux/completion.h>

#include <asm/system.h>
#include <asm/uaccess.h>

#include "mmc_core.h"

#define MAJOR_NR mmc_major /* force definitions on in blk.h */
static int mmc_major = 0xf3; /* must be declared before including blk.h */

//#ifdef CONFIG_ARCH_EZX_E680
/* add by w20598 for mmcsysif */
static int mmcsysif_major = 0xf4;
static int mmcsysif_usage = 0;
extern int mmc_slot_enable;
extern struct completion fatpanic_completion;
extern unsigned short panicdev;

#ifdef CONFIG_DEVFS_FS
devfs_handle_t mmcsysif_de;
#endif

/* add end */
//#endif

#define MMC_SHIFT           3             /* max 8 partitions per card */

#define DEVICE_NR(device)   (MINOR(device)>>MMC_SHIFT)
#define DEVICE_NAME         "mmc"         /* name for messaging */
#define DEVICE_INTR         mmc_intrptr   /* pointer to the bottom half */
#define DEVICE_NO_RANDOM                  /* no entropy to contribute */
#define DEVICE_REQUEST      mmc_media_request
#define DEVICE_OFF(d) /* do-nothing */

#include <linux/blk.h>
#include <linux/blkpg.h>

static int rahead     = 8;
static int maxsectors = 128;
static unsigned long bh_pages = 0;

MODULE_PARM(maxsectors,"i");
MODULE_PARM_DESC(maxsectors,"Maximum number of sectors for a single request");
MODULE_PARM(rahead,"i");
MODULE_PARM_DESC(rahead,"Default sector read ahead");

#define MMC_NDISK	(MMC_MAX_SLOTS << MMC_SHIFT)

/* 
   Don't bother messing with blksize_size....it gets changed by various filesystems.
   You're better off dealing with arbitrary blksize's
*/ 
static int              mmc_blk[MMC_NDISK];  /* Used for hardsect_size - should be 512 bytes */
static int              mmc_max[MMC_NDISK];  /* Used for max_sectors[] - limit size of individual request */

static int              mmc_sizes[MMC_NDISK];        /* Used in gendisk - gives whole size of partition */
static struct hd_struct mmc_partitions[MMC_NDISK];   /* Used in gendisk - gives particular partition information */

static char             mmc_gendisk_flags;
#ifdef CONFIG_DEVFS_FS
static devfs_handle_t   mmc_devfs_handle;
#endif

// There is one mmc_media_dev per inserted card
struct mmc_media_dev {
	int              usage;
	struct mmc_slot *slot;
	spinlock_t       lock;
	int              changed;
	long             nr_sects;   // In total number of sectors

	int              read_block_len;      // Valid read block length
	int              write_block_len;     // Valid write block length

	unsigned char	 lock_param[258];  // Lock card data structure 
	struct mmc_io_request io_request;
};

static struct mmc_media_dev   g_media_dev[MMC_MAX_SLOTS];
static struct mmc_io_request  g_io_request;
static int                    g_busy;

static struct gendisk mmc_gendisk = {
 //modify by zhuzf       major:	        0,               /* major number dynamically assigned */
        major:	        0xf3,
	major_name:	DEVICE_NAME,
	minor_shift:	MMC_SHIFT,	 /* shift to get device number */
	max_p:	        1 << MMC_SHIFT,	 /* Number of partiions */
	/* The remainder will be filled in dynamically */
};

/*************************************************************************/
/* TODO: O_EXCL, O_NDELAY, invalidate_buffers */
static int mmc_media_open( struct inode *inode, struct file *filp )
{
	struct mmc_media_dev *dev;
	int num = DEVICE_NR(inode->i_rdev);
	int ro = 0;

	DEBUG(1,": num=%d\n", num);

//#ifdef CONFIG_ARCH_EZX_E680
	/* add by w20598 */
	if (!mmc_slot_enable)
	    return -ENODEV;
	/* add end */
//#endif

	if ( num >= MMC_MAX_SLOTS)
		return -ENODEV;

	dev = &g_media_dev[num];
	if ( !dev->slot ) 
		return -ENODEV;
        
	spin_lock(&dev->lock);
	if (!dev->usage)
		check_disk_change(inode->i_rdev);
	dev->usage++;
	MOD_INC_USE_COUNT;

	if ( (dev->slot->dev->sdrive->is_wp && dev->slot->dev->sdrive->is_wp(dev->slot->id)) || 
		dev->slot->csd.perm_write_protect || 
		dev->slot->csd.tmp_write_protect) 
		ro = 1;
	set_device_ro(inode->i_rdev, ro);

	spin_unlock(&dev->lock);
	return 0;
}

static int mmc_media_release( struct inode *inode, struct file *filep )
{
	struct mmc_media_dev *dev = &g_media_dev[DEVICE_NR(inode->i_rdev)];

	DEBUG(1,": num=%d\n", DEVICE_NR(inode->i_rdev));

	spin_lock(&dev->lock);
	dev->usage--;
	/* Is this worth doing? */
	/* add by w20598. force to sync filesystem */
//	if (!dev->usage) {	
		fsync_dev(inode->i_rdev);
		destroy_buffers(inode->i_rdev);
//	}
	MOD_DEC_USE_COUNT;
	spin_unlock(&dev->lock);
	return 0;
}

static int mmc_media_revalidate(kdev_t i_rdev)
{
	int index, max_p, start, i;
	struct mmc_media_dev *dev;

	index = DEVICE_NR(i_rdev);
	DEBUG(2,": index=%d\n", index);

	max_p = mmc_gendisk.max_p;
	start = index << MMC_SHIFT;
	dev   = &g_media_dev[index];

	for ( i = max_p - 1 ; i >= 0 ; i-- ) {
		int item = start + i;
		invalidate_device(MKDEV(mmc_major,item),1);
		mmc_gendisk.part[item].start_sect = 0;
		mmc_gendisk.part[item].nr_sects   = 0;
		/* TODO: Fix the blocksize? */
	}

	register_disk(&mmc_gendisk, i_rdev, 1 << MMC_SHIFT, mmc_gendisk.fops, dev->nr_sects);
	return 0;
}

static void mmc_media_wait_done ( struct mmc_io_request *req )
{
 	complete(req->done_data);
}
 
static int mmc_media_ioctl (struct inode *inode, struct file *filp,
			    unsigned int cmd, unsigned long arg)
{
 	DECLARE_COMPLETION(completion);
	int num = DEVICE_NR(inode->i_rdev);
	int size;
	struct hd_geometry geo;
	struct mmc_media_dev *dev;
	struct mmc_slot *slot;
	unsigned long status;

	DEBUG(1," ioctl 0x%x 0x%lx\n", cmd, arg);

//#ifdef CONFIG_ARCH_EZX_E680	
	/* add by w20598 */
	if (!mmc_slot_enable)
	    return -EFAULT;
	/* add end */
//#endif

	dev	= &g_media_dev[num];
	slot	= g_media_dev[num].slot;

	switch(cmd) {
	case BLKGETSIZE:
		/* Return the device size, expressed in sectors */
		/* Not really necessary, but this is faster than walking the gendisk list */
		if (!access_ok(VERIFY_WRITE, arg, sizeof(long)))
			return -EFAULT;
		return put_user(mmc_partitions[MINOR(inode->i_rdev)].nr_sects, (long *)arg);

	case BLKRRPART: /* re-read partition table */
		if (!capable(CAP_SYS_ADMIN)) 
			return -EACCES;
		return mmc_media_revalidate(inode->i_rdev);

	case HDIO_GETGEO:
		if (!access_ok(VERIFY_WRITE, arg, sizeof(geo)))
			return -EFAULT;
		/* Grab the size from the 0 partition for this minor */
                geo.cylinders = mmc_partitions[MINOR(inode->i_rdev)].nr_sects;
                geo.heads     = 1;
                geo.sectors   = 1;
                geo.start     = mmc_partitions[MINOR(inode->i_rdev)].start_sect;
                if (copy_to_user((void *) arg, &geo, sizeof(geo)))
                        return -EFAULT;
                return 0;

       case IOCMMCSETCLKRATE:
		if (!access_ok(VERIFY_READ, arg, sizeof(int)))
			return -EFAULT;

		if (MMC_NO_ERROR != slot->dev->sdrive->set_clock(arg) )
			return -EINVAL;

		return 0;

        case IOCMMCGETCARDCID:
                if (!access_ok(VERIFY_WRITE, arg, sizeof(struct mmc_cid)))
                        return -EFAULT;
                if (copy_to_user ((void *) arg, &(slot->cid), sizeof(struct mmc_cid)))
                        return -EFAULT;
                return 0;

        case IOCMMCGETCARDCSD:
                if (!access_ok(VERIFY_WRITE, arg, sizeof(struct mmc_csd)))
                        return -EFAULT;
                if (copy_to_user ((void *) arg, &(slot->csd), sizeof(struct mmc_csd)))
                        return -EFAULT;
                return 0;

	case IOCMMCGETCARDSTATUS:
		if (!access_ok(VERIFY_WRITE, arg, sizeof(int)))
			return -EFAULT;

		status = 0;

		if (slot->dev->sdrive->is_wp && slot->dev->sdrive->is_wp(num))
			status |= 0x1;

		if (slot->flags & MMC_SLOT_FLAG_LOCKED)
			status |= 0x2;

		if (slot->flags & MMC_SLOT_FLAG_LOCK_FAILED) {
			status |= 0x4;
			slot->flags &= ~MMC_SLOT_FLAG_LOCK_FAILED;
		}

		if (!(slot->csd.ccc & 0x80)) /* dont support lock/ulock */
		        status |= 0x8;

		if (copy_to_user((void*) arg, & status, sizeof(unsigned long)))
			return -EFAULT;
		
		return 0;

	case IOCMMCLOCKCARD:
#if 0
/* e680 dont support lock the card when the phone power on */
		spin_lock(&dev->lock);
		if (dev->usage > 1) {
			spin_unlock(&dev->lock);
			return -EBUSY;
		}
		spin_unlock(&dev->lock);
#endif
 		/*  class 7 support */
		if ( !(slot->csd.ccc & 0x80) ) return -EINVAL;

		/* card lock data structure */
		if (!access_ok(VERIFY_READ, arg, 2)) return -EFAULT;
		if (copy_from_user(dev->lock_param, (void*)arg, 2)) 
			return -EFAULT;

		size = dev->lock_param[1] + 2;
		if (!access_ok(VERIFY_READ, arg, size)) return -EFAULT;
		if (copy_from_user(dev->lock_param, (void*)arg, size))
			return -EFAULT;

		/* issue i/o request for lock/unlock */
		dev->io_request.id 		= num;
		dev->io_request.cmd 		= MMC_IO_LOCK;
		dev->io_request.block_len  	= size;
		dev->io_request.buffer     	= dev->lock_param;
		dev->io_request.done_data	= &completion;
		dev->io_request.done		= mmc_media_wait_done;

		if (!mmc_handle_io_request(&dev->io_request))
			return -EBUSY;

		/* wait until done */
		wait_for_completion(&completion);

		return 0;

//#ifdef CONFIG_ARCH_EZX_E680
	case IOCMMCGETCARDTYPE:
	       	if (!access_ok(VERIFY_WRITE, arg, sizeof(long)))
			return -EFAULT;
				
		return put_user(g_media_dev[num].slot->sd,(long *)arg);
//#endif
		
	default:
		return blk_ioctl(inode->i_rdev, cmd, arg);
	}

	return -ENOTTY; /* should never get here */
}

static int mmc_media_check_change(kdev_t i_rdev) 
{
	int                   index, retval;
	struct mmc_media_dev *dev;
	unsigned long         flags;

	index = DEVICE_NR(i_rdev);
	DEBUG(2," device=%d\n", index);
	if (index >= MMC_MAX_SLOTS) 
		return 0;

	dev = &g_media_dev[index];

	spin_lock_irqsave(&dev->lock, flags);
	retval = (dev->changed ? 1 : 0);
	dev->changed = 0;
	spin_unlock_irqrestore(&dev->lock, flags);

	return retval;
}

static struct mmc_media_dev * mmc_media_locate_device(const struct request *req)
{
	int num = DEVICE_NR(req->rq_dev);
	if ( num >= MMC_MAX_SLOTS) {
		static int count = 0;
		if (count++ < 5) /* print the message at most five times */
			printk(KERN_WARNING "mmc: request for unknown device\n");
		return NULL;
	}
	return &g_media_dev[num];
}

static int mmc_media_transfer( struct mmc_media_dev *dev, const struct request *req )
{
	int minor = MINOR(req->rq_dev);
	unsigned long flags;
	int cmd;

	struct buffer_head *bh;
	int nr_sectors;
	unsigned char *buffer,*data;
	
	DEBUG(2,": minor=%d\n", minor);

	nr_sectors = 0;
	bh = req->bh;

	if (!bh_pages) {
	    buffer = req->buffer;
	    data = NULL;
	}
	else data = buffer = (u8*) bh_pages;

        while(bh) {
	    
	    nr_sectors += bh->b_size >> 9;
            if ( (req->cmd == WRITE) && data ) {
	        memcpy(data, bh->b_data, bh->b_size);
                data += bh->b_size;
            }

            if ( (!bh->b_reqnext) ||
		 (bh->b_reqnext->b_rsector != (bh->b_rsector + (bh->b_size >> 9))) ||
                 ((!data) && (bh->b_reqnext->b_page != bh->b_page)) )
                 break;

            bh = bh->b_reqnext;
        }	    

	if ( req->sector + nr_sectors > mmc_partitions[minor].nr_sects) {
		static int count = 0;
		if (count++ < 5)
			printk(KERN_WARNING "%s: request past end of partition\n", __FUNCTION__);
		return 0;
	}

	cmd = (req->cmd == READ) ? MMC_IO_READ : 
		((req->cmd == WRITE) ? MMC_IO_WRITE : MMC_IO_UNKNOWN);

	if (cmd == MMC_IO_UNKNOWN) {
		printk(KERN_WARNING "%s : unknown request->cmd = %d\n",__FUNCTION__, req->cmd);
		return 0;
	}
	
	spin_lock_irqsave(&dev->lock, flags);

	g_io_request.id         = DEVICE_NR(req->rq_dev);
	g_io_request.cmd        = cmd;
	g_io_request.sector     = mmc_partitions[minor].start_sect + req->sector;
	g_io_request.nr_sectors = nr_sectors;
	g_io_request.block_len  = mmc_blk[minor];
	g_io_request.buffer     = buffer;

	DEBUG(2,": id=%d cmd=%d sector=%ld nr_sectors=%ld block_len=%ld buf=%p\n",
	      g_io_request.id, g_io_request.cmd, g_io_request.sector, g_io_request.nr_sectors,
	      g_io_request.block_len, g_io_request.buffer );

	mmc_handle_io_request(&g_io_request);
	spin_unlock_irqrestore(&dev->lock, flags);
	return 1;
}

static void mmc_media_request( request_queue_t *q )
{
	struct mmc_media_dev *dev;

	if ( g_busy )
		return;

	while(1) {
		INIT_REQUEST;  /* returns when queue is empty */

//#ifdef CONFIG_ARCH_EZX_E680		
		/* add by w20598 to prevent from flushing data when card is removed */
		if (!mmc_slot_enable)
		{
		    if (READ == CURRENT->cmd)
		    {
		        DEBUG(3," card is removed.read cmd return error!\n");
		        end_request(0);
		    }
		    else
		    {
		        DEBUG(3," card is removed.write cmd return ok!\n");
		        end_request(1);
		    }
		    continue;
		}
		/* add end */
//#endif
		
		dev = mmc_media_locate_device(CURRENT);
		if ( !dev ) {
			end_request(0);
			continue;
		}

		DEBUG(2," (%p): cmd %i sec %li (nr. %li)\n", CURRENT,
		      CURRENT->cmd, CURRENT->sector, CURRENT->current_nr_sectors);

		if ( mmc_media_transfer(dev,CURRENT) ) {
			g_busy = 1;
			return;
		}
		end_request(0);  /* There was a problem with the request */
	}
}

void mmc_media_transfer_done( struct mmc_io_request *trans, int result )
{
	unsigned long flags;
	int nsect,nr_sectors;
	struct request *req = CURRENT;
	struct buffer_head *bh = req->bh, *next;
	unsigned char *data = (unsigned char *)bh_pages;
		
	DEBUG(3,": result=%d\n", result);
	spin_lock_irqsave(&io_request_lock, flags);

	nr_sectors = trans->nr_sectors;

	for (;;)
	{
	    if ((req->cmd == READ) && data)
	    {
		memcpy(bh->b_data,data,bh->b_size);
		data += bh->b_size;
            }

	    if (!(next = bh->b_reqnext)) break;
	    if (!(nr_sectors - (next->b_size >> 9))) break;

	    nsect = bh->b_size >> 9;
	    blk_finished_io(nsect);
	    bh->b_reqnext = NULL;
	    bh->b_end_io(bh,result);

	    req->bh = bh = next;
	    next = bh->b_reqnext;

	    req->hard_sector += nsect;
	    req->hard_nr_sectors -= nsect;
	    req->sector = req->hard_sector;
	    req->nr_sectors = req->hard_nr_sectors;

	    req->current_nr_sectors = bh->b_size >> 9;
	    req->hard_cur_sectors = req->current_nr_sectors;
	    if (req->nr_sectors < req->current_nr_sectors) {
		    req->nr_sectors = req->current_nr_sectors;
		    printk("end_request:buffer-list destroyed\n");
	    }

	    req->buffer = bh->b_data;

	    nr_sectors -= nsect;
	}

//#ifdef CONFIG_ARCH_EZX_E680
	/* add by w20598 */
	if (!mmc_slot_enable)
	    if (MMC_IO_READ == trans->cmd)
	        end_request(0);
	    else
	        end_request(1);
	else
	/* add end */ 
//#endif
	    end_request(result);
	g_busy = 0;
	if (!QUEUE_EMPTY)
		mmc_media_request(NULL);  // Start the next transfer
	spin_unlock_irqrestore(&io_request_lock, flags);
}


static struct block_device_operations mmc_bdops = {
	open:               mmc_media_open,
	release:            mmc_media_release,
	ioctl:              mmc_media_ioctl,
	check_media_change: mmc_media_check_change,
	revalidate:         mmc_media_revalidate
};

//#ifdef CONFIG_ARCH_EZX_E680
/*********************************************************************/
/* add by w20598 for hotplug support                                 */
/* if user unplugs the card, driver will prevent from flushing datas */
/* create a new inferface to set driver enable                       */
/*********************************************************************/

/*******************************************************************************
  * Function name:	mmcsysif_open
  * Arguments:		struct inode *
  *			struct file *
  * Return:		int : 0  for successful,
  *			      negative number signaling a error
  * Comments:	open operation
  *******************************************************************************/
static int mmcsysif_open(struct inode * inode, struct file * file)
{
         DEBUG(2,": mmcsysif_usage=%d\n", mmcsysif_usage);
	 
	 if (mmcsysif_usage > 2)
	     return -1;
	     
	 mmcsysif_usage++;
	 
         return 0;
}

/*******************************************************************************
  * Function name:	mmcsysif_release
  * Arguments:		struct inode *
  *			struct file *
  * Return:		int  : 0  for successful,
  *			       negative number signaling a error
  * Comments:	release operation
  *******************************************************************************/
static int mmcsysif_release(struct inode * inode, struct file * file)
{
        if (mmcsysif_usage > 2)
	    return -1;
	    
	mmcsysif_usage--;
	
	DEBUG(2,": mmcsysif_usage=%d\n", mmcsysif_usage);

        return 0;
}

/* ioctl for tcmd. ap_tcmd can run as ezx user */
static int mmcsysif_ioctl_ext(unsigned int cmd, unsigned long arg)
{
        int num = 0;
        struct mmc_media_dev *dev;
        struct mmc_slot *slot;
        unsigned long status;

	dev     = &g_media_dev[num];
	if (dev->slot == NULL)
		return -ENODEV;
	slot    = g_media_dev[num].slot;

	switch(cmd) {
	case IOCMMCGETSIZE:
	    if (!access_ok(VERIFY_WRITE, arg, sizeof(long)))
                return -EFAULT;
            return put_user(dev->nr_sects * dev->read_block_len, (long *)arg);
	    
	case IOCMMCGETCARDCID:
	    if (!access_ok(VERIFY_WRITE, arg, sizeof(struct mmc_cid)))
                return -EFAULT;
            if (copy_to_user ((void *) arg, &(slot->cid), sizeof(struct mmc_cid)))
                return -EFAULT;
            return 0;
		
	case IOCMMCGETCARDCSD:
	    if (!access_ok(VERIFY_WRITE, arg, sizeof(struct mmc_csd)))
                return -EFAULT;
            if (copy_to_user ((void *) arg, &(slot->csd), sizeof(struct mmc_csd)))
                return -EFAULT;
            return 0;
		
	case IOCMMCGETCARDSTATUS:
            if (!access_ok(VERIFY_WRITE, arg, sizeof(int)))
                return -EFAULT;
            status = 0;
            if (slot->dev->sdrive->is_wp && slot->dev->sdrive->is_wp(num))
                status |= 0x1;
            if (copy_to_user((void*) arg, & status, sizeof(unsigned long)))
                return -EFAULT;
            return 0;     
		
	case IOCMMCGETCARDTYPE:
	    if (!access_ok(VERIFY_WRITE, arg, sizeof(long)))
                        return -EFAULT;
            return put_user(g_media_dev[num].slot->sd,(long *)arg);
		
	default:
	      break;
	}

        return -ENOTTY; 

}

static int mmcsysif_ioctl(struct inode *inode, struct file *filp,
                 unsigned int cmd, unsigned long arg)
{
	DEBUG(2," ioctl 0x%x 0x%lx\n", cmd, arg);

	switch(cmd) {
	case IOCMMCSETENABLE:
		if (!access_ok(VERIFY_WRITE, arg, sizeof(long)))
			return -EFAULT;
                if (mmc_slot_enable)
		{
		    DEBUG(3, "slot%d is already enable.\n",arg);
		    return 0;
		}			
		DEBUG(3, "set slot%d enable\n",arg);
		mmc_slot_enable = 1;
		return 0;
	case IOCSETERROR:
		panicdev = (unsigned short)MKDEV(mmc_major,1);
		complete(&fatpanic_completion);
		return 0;
	case IOCMMCEXIST:
		if (!access_ok(VERIFY_WRITE, arg, sizeof(long)))
			return -EFAULT;
		
#ifdef CONFIG_ARCH_EZX_A780
		return put_user(( GPLR(GPIO_MMC_DATA3) & GPIO_bit(GPIO_MMC_DATA3)),(long *)arg);
#else
        return put_user(( GPLR(GPIO_MMC_DETECT) & GPIO_bit(GPIO_MMC_DETECT)),(long *)arg);
#endif


	case IOCMMCGETSIZE:
	case IOCMMCGETCARDCID:
	case IOCMMCGETCARDCSD:
	case IOCMMCGETCARDSTATUS:
	case IOCMMCGETCARDTYPE:
	     return mmcsysif_ioctl_ext(cmd,arg);
	default:
	      break;
	}

	return -ENOTTY; /* should never get here */

}

static ssize_t mmcsysif_read(struct file * filp, char * buf, size_t count, loff_t *ppos)
{
    panicdev = 0;
    init_completion(&fatpanic_completion);
    wait_for_completion(&fatpanic_completion);
    if (!copy_to_user(buf,kdevname((kdev_t) panicdev),5))
	return 0;

    return -1; /* error */	
}

static struct file_operations mmcsysif_bdops = {
       ioctl:      mmcsysif_ioctl,
       open:       mmcsysif_open,
       read:      mmcsysif_read,
       release:    mmcsysif_release
};
//#endif

/******************************************************************/
/* TODO:
   We have a race condition if two slots need to be revalidated at the same
   time.  Perhaps we should walk the list of devices and look for change
   flags?
*/

static void mmc_media_load_task_handler( void *nr )
{
	int slot_id = (int) nr;
	DEBUG(2," slot_id=%d\n", slot_id );
	mmc_media_revalidate(MKDEV(mmc_major,(slot_id<<MMC_SHIFT)));
}

static struct tq_struct mmc_media_load_task = {
	routine:  mmc_media_load_task_handler
};

static void mmc_media_load( struct mmc_slot *slot )
{
	unsigned long flags;
	struct mmc_media_dev *dev  = &g_media_dev[slot->id];
	int i;

	long nr_sects;
	int  write_block_len;
	int  read_block_len;

	spin_lock_irqsave(&dev->lock, flags);

	nr_sects        = (1 + slot->csd.c_size) * (1 << (slot->csd.c_size_mult + 2));
	write_block_len = 1 << slot->csd.write_bl_len;
	read_block_len  = 1 << slot->csd.read_bl_len;

	MOD_INC_USE_COUNT;
	DEBUG(1, " slot=%p nr_sect=%ld write_block_length=%d read_block_len=%d\n", 
	      slot, nr_sects, write_block_len, read_block_len );

	dev->slot            = slot;
	dev->nr_sects        = nr_sects;
	dev->read_block_len  = read_block_len;
	dev->write_block_len = write_block_len;
	dev->changed         = 1;
	mmc_gendisk.nr_real++;

	/* Fix up the block size to match read_block_len */
	/* TODO: can we really do this?  Right now we're affecting blksize_size and hardsect_size */
	for ( i = 0 ; i < (1 << MMC_SHIFT) ; i++ )
		mmc_blk[(slot->id << MMC_SHIFT) + i] = read_block_len;

	mmc_media_load_task.data = (void *) slot->id;
	schedule_task( &mmc_media_load_task );

	spin_unlock_irqrestore(&dev->lock, flags);
}

/* TODO: This is a problem area.  We've lost our card, so we'd like
   to flush all outstanding buffers and requests, remove the partitions from
   the file system, and generally shut everything down.
*/

static void mmc_media_unload( struct mmc_slot *slot )
{
	unsigned long flags;
	struct mmc_media_dev *dev = &g_media_dev[slot->id];

	spin_lock_irqsave(&dev->lock, flags);

//	for ( i = 0 ; i < MMC_SHIFT ; i++ )
//		fsync_dev(MKDEV(mmc_major,slot->id,i));

	MOD_DEC_USE_COUNT;
	DEBUG(1," slot=%p id=%d\n", slot, slot->id);

	dev->slot            = NULL;
	dev->nr_sects        = 0;
	dev->changed         = 1;
	mmc_gendisk.nr_real--;

	mmc_media_load_task.data = (void *) slot->id;
	schedule_task( &mmc_media_load_task );

	spin_unlock_irqrestore(&dev->lock, flags);
}

/* 
   Called once the device has a valid CSD structure
   In the future this should determine what type of card we have
   For the moment, everything is a memory card 
*/

static int mmc_media_probe( struct mmc_slot *slot )
{
	return 1;
}

static struct mmc_media_driver mmc_driver = {
	name:            "flash",
	load:            mmc_media_load,
	unload:          mmc_media_unload,
	probe:           mmc_media_probe,
	io_request_done: mmc_media_transfer_done,
};

/******************************************************************/

static int __init mmc_media_init( void )
{
	int i, result,order;
	DEBUG(0,"Init MMC media\n");
	
#ifdef CONFIG_DEVFS_FS
	mmc_devfs_handle = devfs_mk_dir(NULL, DEVICE_NAME, NULL);
	if (!mmc_devfs_handle) return -EBUSY;
	result = devfs_register_blkdev(mmc_major, DEVICE_NAME, &mmc_bdops);
#else
        result = register_blkdev(mmc_major, DEVICE_NAME, &mmc_bdops);	
#endif	

	if (result < 0) {
		printk(KERN_WARNING "Unable to get major %d for MMC media\n", mmc_major);
		return result;
	}

	if ( !mmc_major ) mmc_major = result;
	DEBUG(3, "register block device /dev/%s* layer with major %d\n",
	      DEVICE_NAME, mmc_major);

//#ifdef CONFIG_ARCH_EZX_E680
/* add by w20598 for mmcsysif */
#ifdef CONFIG_DEVFS_FS
	mmcsysif_de = devfs_register(NULL,"midicomm",DEVFS_FL_AUTO_DEVNUM,
								0, 0, S_IFCHR | S_IRUGO | S_IWUGO,
								&mmcsysif_bdops,NULL);

	if( NULL == mmcsysif_de)
		DEBUG(0,":devfs_register mmcsysif failed");
#else
	result = register_chrdev(mmcsysif_major, "mmcsysif", &mmcsysif_bdops);
	if (result < 0)
	{
		DEBUG(0," :mmcsysif can not get major %d",mmcsysif_major);
		return result;
	}
	if( 0 == mmcsysif_major)
	{
		mmcsysif_major = result;
		DEBUG(2," :mmcsysif:mmcsysif_major = %d",mmcsysif_major);
	}
	DEBUG(2,": register mmcsysif success. major = %d",mmcsysif_major);
#endif
/* add end */
//#endif
	/* Set up global block arrays */
	read_ahead[mmc_major]    = rahead;
	for(i=0 ; i < MMC_NDISK; i++)
		mmc_blk[i] = 512;
	hardsect_size[mmc_major] = mmc_blk;
	for(i=0; i < MMC_NDISK; i++)
		mmc_max[i] = maxsectors;
	max_sectors[mmc_major]   = mmc_max;

	/* Start with zero-sized partitions : we'll fix this later */
	memset(mmc_sizes, 0, sizeof(int) * MMC_NDISK);
	blk_size[mmc_major] = mmc_sizes;

	/* Fix up the gendisk structure */
	mmc_gendisk.part    = mmc_partitions;
	mmc_gendisk.sizes   = mmc_sizes;
	mmc_gendisk.nr_real = 0;
	
#ifdef CONFIG_DEVFS_FS
	mmc_gendisk.de_arr  = &mmc_devfs_handle;
#endif	

	mmc_gendisk.flags   = &mmc_gendisk_flags;
	mmc_gendisk.fops    = &mmc_bdops;

	/* Add ourselves to the global list */
	mmc_gendisk.major = mmc_major;
	add_gendisk(&mmc_gendisk);

	order = 0;
	while (maxsectors > (1<< (order + 2))) order++;

	bh_pages = __get_free_pages(GFP_KERNEL,order);
	
	blk_init_queue(BLK_DEFAULT_QUEUE(mmc_major), DEVICE_REQUEST);
	return mmc_register_media_driver(&mmc_driver);
}

static void mmc_media_cleanup( void )
{
	int i,order;
	DEBUG(0,"\n");

	flush_scheduled_tasks();
	unregister_blkdev(mmc_major, DEVICE_NAME);

	for ( i = 0 ; i < MMC_NDISK; i++ )
		fsync_dev(MKDEV(mmc_major,i));

	mmc_unregister_media_driver(&mmc_driver);

	blk_cleanup_queue(BLK_DEFAULT_QUEUE(mmc_major));

	blk_size[mmc_major]      = NULL;
	hardsect_size[mmc_major] = NULL;
	max_sectors[mmc_major]   = NULL;

	del_gendisk(&mmc_gendisk);

	order = 0;
	while (maxsectors > (1<< (order + 2))) order++;

	if (bh_pages) free_pages(bh_pages,order);

//#ifdef CONFIG_ARCH_EZX_E680	
/* add by w20598 */
#ifdef CONFIG_DEVFS_FS
	devfs_unregister(mmcsysif_de);
#else
	unregister_chrdev(mmcsysif_major,"mmcsysif");
#endif
        DEBUG(3,":mmcsysif quit");
/* add end */
//#endif

#ifdef CONFIG_DEVFS_FS
	devfs_unregister(mmc_devfs_handle);
#endif
	
}

struct mmc_media_module media_module = {
	init:    mmc_media_init,
	cleanup: mmc_media_cleanup
};
