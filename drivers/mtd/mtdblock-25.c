/* 
 * Direct MTD block device access
 *
 * $Id: mtdblock-25.c,v 1.1 2003/01/09 21:35:46 ahennessy Exp $
 *
 * 02-nov-2000	Nicolas Pitre		Added read-modify-write with cache
 */

#include <linux/config.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/buffer_head.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/compatmac.h>
#include <linux/buffer_head.h>
#include "mtdblock.h"

#define DEVICE_NR(device) (device)
#define LOCAL_END_REQUEST
#include <linux/blk.h>

#ifdef CONFIG_DEVFS_FS
#include <linux/devfs_fs_kernel.h>
static devfs_handle_t devfs_dir_handle = NULL;
static devfs_handle_t devfs_rw_handle[MAX_MTD_DEVICES];
#endif

static void mtd_notify_add(struct mtd_info* mtd);
static void mtd_notify_remove(struct mtd_info* mtd);
static struct mtd_notifier notifier = {
        mtd_notify_add,
        mtd_notify_remove,
        NULL
};

static struct mtdblk_dev *mtdblks[MAX_MTD_DEVICES];

static struct gendisk *mtddisk[MAX_MTD_DEVICES];

static spinlock_t mtdblks_lock;

static struct block_device_operations mtd_fops;

static int mtdblock_open(struct inode *inode, struct file *file)
{
	struct mtdblk_dev *mtdblk;
	struct mtd_info *mtd;
	int dev = minor(inode->i_rdev);
	struct gendisk *disk;

	DEBUG(MTD_DEBUG_LEVEL1,"mtdblock_open\n");

	if (dev >= MAX_MTD_DEVICES)
		return -EINVAL;

	mtd = get_mtd_device(NULL, dev);
	if (!mtd)
		return -ENODEV;
	if (MTD_ABSENT == mtd->type) {
		put_mtd_device(mtd);
		return -ENODEV;
	}
	
	spin_lock(&mtdblks_lock);

	/* If it's already open, no need to piss about. */
	if (mtdblks[dev]) {
		mtdblks[dev]->count++;
		spin_unlock(&mtdblks_lock);
		return 0;
	}
	
	/* OK, it's not open. Try to find it */

	/* First we have to drop the lock, because we have to
	   to things which might sleep.
	*/
	spin_unlock(&mtdblks_lock);

	mtdblk = kmalloc(sizeof(struct mtdblk_dev), GFP_KERNEL);
	disk = mtddisk[dev];
	if (!mtdblk || !disk)
		goto Enomem;
	memset(mtdblk, 0, sizeof(*mtdblk));
	mtdblk->count = 1;
	mtdblk->mtd = mtd;

	init_MUTEX (&mtdblk->cache_sem);
	mtdblk->cache_state = STATE_EMPTY;
	if ((mtdblk->mtd->flags & MTD_CAP_RAM) != MTD_CAP_RAM &&
	    mtdblk->mtd->erasesize) {
		mtdblk->cache_size = mtdblk->mtd->erasesize;
		mtdblk->cache_data = vmalloc(mtdblk->mtd->erasesize);
		if (!mtdblk->cache_data)
			goto Enomem;
	}

	/* OK, we've created a new one. Add it to the list. */

	spin_lock(&mtdblks_lock);

	if (mtdblks[dev]) {
		/* Another CPU made one at the same time as us. */
		mtdblks[dev]->count++;
		spin_unlock(&mtdblks_lock);
		put_mtd_device(mtdblk->mtd);
		vfree(mtdblk->cache_data);
		kfree(mtdblk);
		return 0;
	}

	mtdblks[dev] = mtdblk;
	set_device_ro(inode->i_bdev, !(mtdblk->mtd->flags & MTD_WRITEABLE));

	spin_unlock(&mtdblks_lock);

	DEBUG(MTD_DEBUG_LEVEL1, "ok\n");

	return 0;
Enomem:
	put_mtd_device(mtd);
	kfree(mtdblk);
	return -ENOMEM;
}

static release_t mtdblock_release(struct inode *inode, struct file *file)
{
	int dev;
	struct mtdblk_dev *mtdblk;
   	DEBUG(MTD_DEBUG_LEVEL1, "mtdblock_release\n");

	if (inode == NULL)
		release_return(-ENODEV);

	dev = minor(inode->i_rdev);
	mtdblk = mtdblks[dev];

	down(&mtdblk->cache_sem);
	write_cached_data(mtdblk);
	up(&mtdblk->cache_sem);

	spin_lock(&mtdblks_lock);
	if (!--mtdblk->count) {
		/* It was the last usage. Free the device */
		mtdblks[dev] = NULL;
		spin_unlock(&mtdblks_lock);
		if (mtdblk->mtd->sync)
			mtdblk->mtd->sync(mtdblk->mtd);
		put_mtd_device(mtdblk->mtd);
		vfree(mtdblk->cache_data);
		kfree(mtdblk);
	} else {
		spin_unlock(&mtdblks_lock);
	}

	DEBUG(MTD_DEBUG_LEVEL1, "ok\n");

	release_return(0);
}  


/* 
 * This is a special request_fn because it is executed in a process context 
 * to be able to sleep independently of the caller.  The queue_lock 
 * is held upon entry and exit.
 * The head of our request queue is considered active so there is no need 
 * to dequeue requests before we are done.
 */
static struct request_queue mtd_queue;
static void handle_mtdblock_request(void)
{
	struct mtdblk_dev *mtdblk;
	unsigned int res;

	while (!blk_queue_empty(&mtd_queue)) {
		struct request *req = elv_next_request(&mtd_queue);
		struct mtdblk_dev **p = req->rq_disk->private_data;
		spin_unlock_irq(mtd_queue.queue_lock);
		mtdblk = *p;
		res = 0;

		if (! (req->flags & REQ_CMD))
			goto end_req;

		if ((req->sector + req->current_nr_sectors) > (mtdblk->mtd->size >> 9))
			goto end_req;

		// Handle the request
		switch (rq_data_dir(req))
		{
			int err;

			case READ:
			down(&mtdblk->cache_sem);
			err = do_cached_read (mtdblk, req->sector << 9, 
					req->current_nr_sectors << 9,
					req->buffer);
			up(&mtdblk->cache_sem);
			if (!err)
				res = 1;
			break;

			case WRITE:
			// Read only device
			if ( !(mtdblk->mtd->flags & MTD_WRITEABLE) ) 
				break;

			// Do the write
			down(&mtdblk->cache_sem);
			err = do_cached_write (mtdblk, req->sector << 9,
					req->current_nr_sectors << 9, 
					req->buffer);
			up(&mtdblk->cache_sem);
			if (!err)
				res = 1;
			break;
		}

end_req:
		spin_lock_irq(mtd_queue.queue_lock);
		if (!end_that_request_first(req, res, req->hard_cur_sectors)) {
			blkdev_dequeue_request(req);
			end_that_request_last(req);
		}

	}
}

static volatile int leaving = 0;
static DECLARE_MUTEX_LOCKED(thread_sem);
static DECLARE_WAIT_QUEUE_HEAD(thr_wq);

int mtdblock_thread(void *dummy)
{
	struct task_struct *tsk = current;
	DECLARE_WAITQUEUE(wait, tsk);

	/* we might get involved when memory gets low, so use PF_MEMALLOC */
	tsk->flags |= PF_MEMALLOC;
	strcpy(tsk->comm, "mtdblockd");
	spin_lock_irq(&tsk->sig->siglock);
	sigfillset(&tsk->blocked);
	recalc_sigpending();
	spin_unlock_irq(&tsk->sig->siglock);
	daemonize();

	while (!leaving) {
		add_wait_queue(&thr_wq, &wait);
		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irq(mtd_queue.queue_lock);
		if (blk_queue_empty(&mtd_queue) || blk_queue_plugged(&mtd_queue)) {
			spin_unlock_irq(mtd_queue.queue_lock);
			schedule();
			remove_wait_queue(&thr_wq, &wait); 
		} else {
			remove_wait_queue(&thr_wq, &wait); 
			set_current_state(TASK_RUNNING);
			handle_mtdblock_request();
			spin_unlock_irq(mtd_queue.queue_lock);
		}
	}

	up(&thread_sem);
	return 0;
}

static void mtdblock_request(struct request_queue *q)
{
	/* Don't do anything, except wake the thread if necessary */
	wake_up(&thr_wq);
}


static int mtdblock_ioctl(struct inode * inode, struct file * file,
		      unsigned int cmd, unsigned long arg)
{
	struct mtdblk_dev *mtdblk;

	mtdblk = mtdblks[minor(inode->i_rdev)];

#ifdef PARANOIA
	if (!mtdblk)
		BUG();
#endif

	switch (cmd) {
	case BLKFLSBUF:
		fsync_bdev(inode->i_bdev);
		invalidate_bdev(inode->i_bdev, 0);
		down(&mtdblk->cache_sem);
		write_cached_data(mtdblk);
		up(&mtdblk->cache_sem);
		if (mtdblk->mtd->sync)
			mtdblk->mtd->sync(mtdblk->mtd);
		return 0;

	default:
		return -EINVAL;
	}
}

static struct block_device_operations mtd_fops = 
{
	.owner		= THIS_MODULE,
	.open		= mtdblock_open,
	.release	= mtdblock_release,
	.ioctl		= mtdblock_ioctl
};

/* Notification that a new device has been added. Create the devfs entry for
 * it. */

static void mtd_notify_add(struct mtd_info* mtd)
{
	struct gendisk *disk;
        char name[8];

        if (!mtd || mtd->type == MTD_ABSENT)
                return;

#ifdef CONFIG_DEVFS_FS
        sprintf(name, "%d", mtd->index);
        devfs_rw_handle[mtd->index] = devfs_register(devfs_dir_handle, name,
                        DEVFS_FL_DEFAULT, MTD_BLOCK_MAJOR, mtd->index,
                        S_IFBLK | S_IRUGO | S_IWUGO,
                        &mtd_fops, NULL);
#endif

	disk = alloc_disk(1);
	if (disk) {
		disk->major = MAJOR_NR;
		disk->first_minor = mtd->index;
		disk->fops = &mtd_fops;
		sprintf(disk->disk_name, "mtdblock%d", mtd->index);

		mtddisk[mtd->index] = disk;
		set_capacity(disk, mtd->size / 512);
		disk->private_data = &mtdblks[mtd->index];
		disk->queue = &mtd_queue;
		add_disk(disk);
	}
}

static void mtd_notify_remove(struct mtd_info* mtd)
{
        if (!mtd || mtd->type == MTD_ABSENT)
                return;

#ifdef CONFIG_DEVFS_FS
        devfs_unregister(devfs_rw_handle[mtd->index]);
#endif

        if (mtddisk[mtd->index]) {
		del_gendisk(mtddisk[mtd->index]);
        	put_disk(mtddisk[mtd->index]);
        	mtddisk[mtd->index] = NULL;
        }
}

static spinlock_t mtddev_lock = SPIN_LOCK_UNLOCKED;

int __init init_mtdblock(void)
{
	spin_lock_init(&mtdblks_lock);
	if (register_blkdev(MAJOR_NR,DEVICE_NAME,&mtd_fops)) {
		printk(KERN_NOTICE "Can't allocate major number %d for Memory Technology Devices.\n",
		       MTD_BLOCK_MAJOR);
		return -EAGAIN;
	}
#ifdef CONFIG_DEVFS_FS
	devfs_dir_handle = devfs_mk_dir(NULL, DEVICE_NAME, NULL);
#endif
	register_mtd_user(&notifier);
	
	init_waitqueue_head(&thr_wq);
	blk_init_queue(&mtd_queue, &mtdblock_request, &mtddev_lock);
	kernel_thread (mtdblock_thread, NULL, CLONE_FS|CLONE_FILES|CLONE_SIGHAND);
	return 0;
}

static void __exit cleanup_mtdblock(void)
{
	leaving = 1;
	wake_up(&thr_wq);
	down(&thread_sem);
	unregister_mtd_user(&notifier);
#ifdef CONFIG_DEVFS_FS
	devfs_unregister(devfs_dir_handle);
#endif
	unregister_blkdev(MAJOR_NR,DEVICE_NAME);
	blk_cleanup_queue(&mtd_queue);
}

