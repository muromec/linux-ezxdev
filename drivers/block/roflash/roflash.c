/*
 * Copyright (C) 2002, 2005 Motorola Inc.
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
 *  History:
 *  Gu susan           Api.11,2002,  create for ezx platform
 *  Gu susan           July.12,2005,  remove warning code
 *
 */

/*
 * Linux Flash read-only block device driver For Cramfs file system, support simultaneous linear + block device mounting.  -- Susan Gu  Mar, 15 2002
 * In order to resolve logo area in cachable flash mapping, add roflash_c_fops methods, that is stolen from mtdchar.c (dwmw2)
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/config.h>
#include <linux/kmod.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/vmalloc.h>

#include <asm/uaccess.h>
#include <linux/mtd/map.h>
#include <linux/ezx_roflash.h>

#ifdef	CONFIG_PROC_FS
#include <linux/proc_fs.h>	/* For /proc/roflash_info */
#endif	/* CONFIG_PROC_FS */

#define MAJOR_NR	ROFLASH_MAJOR
#define DEVICE_NAME "roflash"
#define DEVICE_REQUEST roflash_request
#define DEVICE_NR(device) (MINOR(device))
#define DEVICE_ON(device)
#define DEVICE_OFF(device)
#define DEVICE_NO_RANDOM

#define MAJOR_NR_C 101
#define DEVICE_NAME_C "roflash_c"
#include <linux/blk.h>
#include <linux/blkdev.h>


/* K3 flash lowlevel */
#define FLASH_WORD_ALIGN_MASK 0xFFFFFFFE
#define WIDTH_ADJUST	0x0001L		/* I.E. Converts 0x90 to 0x90 */
#define FlashCommandRead        (unsigned short)(0xFF * WIDTH_ADJUST)
#define FlashCommandStatus       (unsigned short)(0x70 * WIDTH_ADJUST)
#define FlashStatusReady          (unsigned short)(0x80 * WIDTH_ADJUST)

/* Variables used to track operational state of ROFLASH */
#define ROFLASH_STATUS_READY			0
#define ROFLASH_STATUS_NOT_READY    -1
#define ROFLASH_BLOCK_SIZE (4096)
static unsigned long flash_blocksizes[MAX_ROFLASH];
static unsigned long flash_sizes[MAX_ROFLASH];

#ifdef	CONFIG_PROC_FS
static struct proc_dir_entry *roflash_proc_entry;
static int roflash_read_procmem(char *, char **, off_t, int, int *, void *);
#endif	/* CONFIG_PROC_FS */


extern unsigned short roflash_partitions;
extern struct map_info bulverde_map;
extern unsigned long bulverde_map_cacheable;

#define __KERNEL_SYSCALLS__
#include <linux/unistd.h>
#include <asm/unaligned.h>


void fmemoryCopy (void * dest, void * src, unsigned long bytes)
{
	unsigned char *new_dest;
	unsigned char *new_src;
	new_dest = (unsigned char *)dest;
	new_src = (unsigned char *)src;
	/* while bytes to modify is not complete */
	while (bytes-- != 0)
	{
		/* update destination byte to equal src byte */
		*new_dest = *new_src;
		new_dest++;
		new_src++;
	}
}
void memorySet (void * buff, unsigned long bytes, unsigned char value)
{
	unsigned char * new_buff;
	new_buff = (unsigned char *)buff;
	/* while not at end of modification */
	while (bytes-- != 0)
	{
		/* set contents of buffer to value specified */
		*new_buff = value;

		/* increment to next byte */
		new_buff++;
	}
}

#if (0)
unsigned short IntelSCSRead(unsigned long CardAddress, unsigned long Length, unsigned char * Buffer)
{
	unsigned long volatile pointer;
	unsigned short * volatile pointer0ed;

#ifdef VFM_DEBUG2
	if ( roflash_status == ROFLASH_STATUS_READY )
	{
		printk("IntelSCSRead:  PositionPtr(pointer, CardAddress);\n");
		printk("IntelSCSRead: pointer=%d, CardAddress=%d\n", (DWORD)pointer, (DWORD)CardAddress);
	}
#endif
	pointer = (unsigned long)(DALHART_K3_BASE_ADDR + CardAddress);

	pointer0ed = (unsigned short *)((unsigned long)pointer & FLASH_WORD_ALIGN_MASK);
#ifdef CONFIG_K3_DEBUG
	printk("IntelSCSRead(): Reading %d bytes at 0x%lx\n",Length, pointer);
#endif

	*pointer0ed = FlashCommandRead;  /* Ensure it is in read mode */

	/* This is assumimg that farmemory copy performs at a byte at a time */
	fmemoryCopy(Buffer, pointer, Length);

	return(0);
}
#endif

void roflash_request(request_queue_t *q)
{
    unsigned short minor;
    unsigned long offset, len, retlen;
	unsigned short err = 0;
	unsigned char *tmpBufPtr;
	unsigned short nrSectors;
	
#ifdef ROFLASH_DEBUG_ERR
	    printk(KERN_ERR "Roflash_request(): Entered\n");
#endif

	/* Loop here forever */
	/* Not really, Linux decides when to return inside *_REQUEST macros */
    for(;;) 
    {
		INIT_REQUEST;

		/* Verify minor number is in range */
		minor = MINOR(CURRENT->rq_dev);
		if (minor >= roflash_partitions) 
		{
#ifdef ROFLASH_DEBUG_ERR
		    printk(KERN_ERR "roflash_request(): ERROR! Out of partition range (minor = %d)\n", minor );
#endif
			end_request(0);
		    continue;
		}

		/* Check if writting-request is demanded */
		if ( CURRENT->cmd == WRITE )
		{
			printk(KERN_ERR "roflash_request: write-accessing is not allowed!\n");
			end_request(0);
			break;
		}

		/* Verify sector is in range */
		offset = CURRENT->sector << 9;			// 512 bytes per sector
		len = CURRENT->current_nr_sectors << 9;	// 512 bytes per sector
		if ((offset + len) > (flash_sizes[minor] << BLOCK_SIZE_BITS)) 
		{
#ifdef ROFLASH_DEBUG_ERR
		    printk(KERN_ERR "roflash_request(): ERROR! Access beyond end of partition\n" );
#endif
		    end_request(0);
		    continue;
		}

#ifdef ROFLASH_DEBUG_ERR
		printk(KERN_ERR "roflash_request(): minor=%d, %d sectors at sector %ld\n", minor, CURRENT->current_nr_sectors, CURRENT->sector);
#endif

		/* Prepare to move data to/from the Linux buffer */	
		nrSectors = CURRENT->current_nr_sectors;
		tmpBufPtr = CURRENT->buffer;
//		blockTempBufPtr = roflashTempBuffer; -- this may cause race-condition in multiple cramfs

		/* Execute the request */
		switch( CURRENT->cmd ) 
		{
		    case READ:
		    {
				roflash_area *this_area = NULL;
#ifdef ROFLASH_DEBUG_ERR
		    	printk(KERN_ERR "roflash_request: READ\n");
#endif
//				memorySet(roflashTempBuffer, len, 0x00); -- don't use the internal buffer, may exist race-conditions.
				
				this_area = (roflash_area *)(roflash_table_pptr[minor]);
				err = (*this_area->roflash_read)((void *)(this_area->priv_map),(offset + this_area->offset), len, (size_t *)&retlen, (u_char *)tmpBufPtr);
				if (err == 0)
				{
#ifdef ROFLASH_DEBUG_ERR
					printk(KERN_ERR " < %x %x %x %x ...>\n",CURRENT->buffer[0],CURRENT->buffer[1],CURRENT->buffer[2],CURRENT->buffer[3]);
#endif
					end_request(1);
				}
				else
				{
#ifdef ROFLASH_DEBUG_ERR
					printk(KERN_ERR "roflash_request(): ERROR READ %d sectors at sector %d\n", CURRENT->current_nr_sectors, CURRENT->sector);
#endif
					end_request(0);
				}
			break;
		    }

		    default:
		    {
#ifdef ROFLASH_DEBUG_ERR
				printk(KERN_ERR "roflash_request(): WARNING! Unknown Request 0x%x\n",CURRENT->cmd);
#endif
				end_request(0);
			break;
		    }
		}
    }
} 

static int roflash_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
// What is the requirements? -- todo list
    return 0;
}


static int roflash_open(struct inode * inode, struct file * filp)
{
#ifdef ROFLASH_DEBUG_ERR
    printk(KERN_ERR "roflash_open() called, minor = %d\n", DEVICE_NR(inode->i_rdev));
#endif
    if (DEVICE_NR(inode->i_rdev) >= roflash_partitions)
	{
#ifdef ROFLASH_DEBUG_ERR
    printk(KERN_ERR "vfm_open() Failed!\n");
#endif
		return -ENXIO;
	}
    return 0;
}

static int roflash_release(struct inode * inode, struct file * filp)
{
#ifdef ROFLASH_DEBUG_ERR
    printk(KERN_ERR "roflash_release() called\n");
#endif
    return 0;
}


/*Susan 2.4.6 */
static struct block_device_operations roflash_fops = {
    open:	roflash_open,
    release:	roflash_release,
    ioctl:	roflash_ioctl,
};

static loff_t roflash_c_lseek (struct file *file, loff_t offset, int orig)
{
	roflash_area *dev_def = (roflash_area *)(file->private_data);
	
	switch (orig) {
	case 0:
		/* SEEK_SET */
		file->f_pos = offset;
		break;
	case 1:
		/* SEEK_CUR */
		file->f_pos += offset;
		break;
	case 2:
		/* SEEK_END */
		file->f_pos = (unsigned long)(dev_def->size + offset);
		break;
	default:
		return -EINVAL;
	}

	if (file->f_pos < 0)
		file->f_pos = 0;
	else if (file->f_pos >= dev_def->size)
		file->f_pos = dev_def->size - 1;

	return file->f_pos;
}

static int roflash_c_open(struct inode *inode, struct file *file)
{
	int minor = MINOR(inode->i_rdev);
	roflash_area *dev_def;

	if (minor >= MAX_ROFLASH)
		return -EACCES;

	dev_def = (roflash_area *)(roflash_table_pptr[minor]);

	/* Sanity check */
	if (dev_def->l_x_b != ROFLASH_CHAR)
		return -EACCES;

	/* You can't open the RO devices RW */
	if (file->f_mode & 2)
		return -EACCES;
	
	file->private_data = dev_def;
		
	return 0;
} /* mtd_open */

static int roflash_c_close(struct inode *inode, struct file *file)
{
	return 0;
} /* mtd_close */

/* FIXME: This _really_ needs to die. In 2.5, we should lock the
   userspace buffer down and use it directly with readv/writev.
*/
#define MAX_KMALLOC_SIZE 0x20000

static int roflash_c_read(struct file *file, char *buf, size_t count,loff_t *ppos)
{
	roflash_area *dev_def = (roflash_area *)file->private_data;
	size_t total_retlen=0;
	int len;
	char *kbuf;
	unsigned long virt_dev_start = 0;
	
	if (*ppos + count > dev_def->size)
		count = dev_def->size - *ppos;

	if (!count)
		return 0;
	
	/* FIXME: Use kiovec in 2.5 to lock down the user's buffers
	   and pass them directly to the MTD functions */
	while (count) 
	{
		if (count > MAX_KMALLOC_SIZE) 
			len = MAX_KMALLOC_SIZE;
		else
			len = count;

		kbuf=kmalloc(len,GFP_KERNEL);
		if (!kbuf)
			return -ENOMEM;
		
		//ret = (dev_def->roflash_read)( (void *)(dev_def->priv_map),(*ppos + dev_def->offset), len, &retlen, kbuf);
		virt_dev_start = (unsigned long)(dev_def->priv_map) + dev_def->offset;
		memcpy((void *)kbuf,(void *)(virt_dev_start + (unsigned long)(*ppos)),len);

		*ppos += len;
		if (copy_to_user(buf, kbuf, len)) 
		{
			kfree(kbuf);
			return -EFAULT;
		}
		else
			total_retlen += len;

		count -= len;
		buf += len;

		
		kfree(kbuf);
	}
	
	return total_retlen;
} /* mtd_read */


static struct file_operations roflash_c_fops = {
	owner:		THIS_MODULE,
	llseek:		roflash_c_lseek,     	/* lseek */
	read:		roflash_c_read,	/* read */
	open:		roflash_c_open,	/* open */
	release:	roflash_c_close,	/* release */
};

roflash_area *roflash_get_dev(unsigned char minor)
{
	roflash_area *ret = NULL;
	
	if (minor < MAX_ROFLASH)
		ret = (roflash_area *)(roflash_table_pptr[minor]);

	return ret;
}
	
int __init roflash_init(void)
{
        int		i, size = 0;

#ifdef ROFLASH_DEBUG_ERR
	printk(KERN_ERR "roflash_init(): Enter into this API.\n");
#endif
	
/* We should do initialization for flash chips, however, we don't need to do it twice after __init_lubbock has initialized K3 flash device */
	
#ifdef ROFLASH_DEBUG_ERR
		printk(KERN_ERR "roflash_init: Registering device major %d [%s]\n",ROFLASH_MAJOR,DEVICE_NAME);
#endif

		if (register_blkdev(ROFLASH_MAJOR, DEVICE_NAME, &roflash_fops)) 
		{
#ifdef ROFLASH_DEBUG_ERR
			printk(KERN_ERR "roflash_init(): Could not get major %d", ROFLASH_MAJOR);
#endif
			return -EIO;
		}
#ifdef ROFLASH_DEBUG_ERR
		printk(KERN_ERR "roflash_init(): register_blkdev success\n");
#endif

	/* We know this is invoked only once during kernel booting up, so there is not race-conditions */
		for (i = 0; i < roflash_partitions; i++) 
		{
			unsigned short lxb_flag = ((roflash_area *)(roflash_table_pptr[i]))->l_x_b;

			switch (lxb_flag)
			{
				case ROFLASH_LINEAR:
				case ROFLASH_LINEAR_XIP:
				{
					flash_blocksizes[i] = ROFLASH_BLOCK_SIZE;
					size = ((roflash_area *)(roflash_table_pptr[i]))->size;
					flash_sizes[i] = size >> BLOCK_SIZE_BITS ;
					((roflash_area *)(roflash_table_pptr[i]))->priv_map = (void *)(bulverde_map_cacheable);  // The virtual address of the cacheable mapping //
	#ifdef ROFLASH_DEBUG_ERR
					printk(KERN_ERR "roflash_init(): not-char roflash_table_pptr[%d]->size = %d\n",i,size);
	#endif
					break;
				}
				case ROFLASH_BLOCK:
				{
					if (bulverde_map.size == 0)
					{
						unregister_blkdev(ROFLASH_MAJOR, DEVICE_NAME);
						return -EACCES;
					}
					
					flash_blocksizes[i] = ROFLASH_BLOCK_SIZE;
					size = ((roflash_area *)(roflash_table_pptr[i]))->size;
					flash_sizes[i] = size >> BLOCK_SIZE_BITS ;
					((roflash_area *)(roflash_table_pptr[i]))->priv_map = (void *)(&bulverde_map);  //The pointer to struct map_info of the noncacheable mapping //
	#ifdef ROFLASH_DEBUG_ERR
					printk(KERN_ERR "roflash_init(): not-char roflash_table_pptr[%d]->size = %d\n",i,size);
	#endif
					break;
				}
				case ROFLASH_CHAR:
				{
					if (register_chrdev(MAJOR_NR_C, DEVICE_NAME_C, &roflash_c_fops))
					{
						printk(KERN_NOTICE "Can't allocate major number %d for Memory Technology Devices.\n",
						       MAJOR_NR_C);
						
						unregister_blkdev(ROFLASH_MAJOR, DEVICE_NAME);
						return -EAGAIN;
					}
					((roflash_area *)(roflash_table_pptr[i]))->priv_map = (void *)(bulverde_map_cacheable);  //The virtual address of the cacheable mapping //
	#ifdef ROFLASH_DEBUG_ERR
					printk(KERN_ERR "roflash_init(): char roflash_table_pptr[%d]->size = %d\n",i,size);
	#endif
					break;
				}
			}
		}
		
		blksize_size[ROFLASH_MAJOR] = (int *)flash_blocksizes;
		blk_size[ROFLASH_MAJOR] = (int *)flash_sizes;
#ifdef ROFLASH_DEBUG_ERR
		printk(KERN_ERR "roflash_init(): fill in blksize_size[] and blk_size[] success\n");
#endif
		blk_init_queue((request_queue_t *)(BLK_DEFAULT_QUEUE(ROFLASH_MAJOR)),(request_fn_proc *)(&roflash_request));

#ifdef ROFLASH_DEBUG_ERR
		printk(KERN_ERR "roflash_init(): ROFLASH Block device: blksize_size = %d, blk_size = %d\n",flash_blocksizes[0],flash_sizes[0]);
#endif

#ifdef	CONFIG_PROC_FS
		/* register procfs device */
		roflash_proc_entry = create_proc_entry("roflash_info", 0, 0);
		if (roflash_proc_entry)
		{
			roflash_proc_entry->read_proc = roflash_read_procmem;
		}

#endif	/* CONFIG_PROC_FS */
	
		for ( i = 0; i < roflash_partitions; i ++ )
		{
			//printk(KERN_NOTICE "ROFLASH Driver initialized and ready for use. Size: %d Offset: %d\n", ((roflash_area*)(roflash_table_pptr[i]))->size, ((roflash_area*)(roflash_table_pptr[i]))->offset);
		}

	return (0);	
}


#ifdef CONFIG_PROC_FS
static int roflash_read_procmem(char *buf, char **start, off_t offset, int len,
				int *eof, void *data)
{
	#define LIMIT (PAGE_SIZE-80)	/* don't print anymore after this size */

	int i;
	
	len=0;
	
#if (0)
	switch (roflash_status)
	{
		case ROFLASH_STATUS_NOT_READY:
			strcpy(roflash_status_str,"Not-ready");
			break;
		case ROFLASH_STATUS_READY:
			strcpy(roflash_status_str,"Ready");
			break;
		default:
			strcpy(roflash_status_str,"Unknown!");
			break;
	}
#endif

	len += sprintf(buf+len, "ROFLASH Driver status: %s\n\n","Ready");
	for ( i = 0; i < roflash_partitions; i ++ )
	{
		len += sprintf(buf+len, "ROFLASH area name is %s\n",((roflash_area*)(roflash_table_pptr[i]))->name);
		len += sprintf(buf+len, "ROFLASH area size = %ld bytes\n",((roflash_area*)(roflash_table_pptr[i]))->size);
		len += sprintf(buf+len, "ROFLASH area offset = %ld bytes\n",((roflash_area*)(roflash_table_pptr[i]))->offset);
		len += sprintf(buf+len, "ROFLASH area l_x_b(%x)\n",((roflash_area*)(roflash_table_pptr[i]))->l_x_b);
	}
	return len;
}
#endif	/* CONFIG_PROC_FS */

#ifdef MODULE
static void __exit roflash_cleanup(void)  //Susan//
{
    int i;
    int err_info = 0;

    /* For read-only flash device, we don't need to invoke fsync_dev */
 
    unregister_blkdev(ROFLASH_MAJOR, DEVICE_NAME);
	blk_cleanup_queue(BLK_DEFAULT_QUEUE(ROFLASH_MAJOR));
    blk_size[ROFLASH_MAJOR] = NULL;
    blksize_size[ROFLASH_MAJOR] = NULL;

    /* For roflash char device  -- sometimes, we don't have char roflash devices */
	for (i = 0; i < roflash_partitions; i++) 
	{
		unsigned short lxb_flag = ((roflash_area *)(roflash_table_pptr[i]))->l_x_b;

		if (lxb_flag == ROFLASH_CHAR)
		{
			unregister_chrdev(MAJOR_NR_C, DEVICE_NAME_C);
			break;
		}
	}

    remove_proc_entry("roflash_info",NULL);

    printk(KERN_ERR "remove roflash_info\n");

}

module_init(roflash_init);
module_exit(roflash_cleanup);

#endif
