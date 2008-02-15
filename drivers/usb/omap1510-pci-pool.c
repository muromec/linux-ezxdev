/******************************************************************************

	linux/drivers/usb/omap1510-pci-pool.c
	PCI Bus Emulation for USB HOST on OMAP1510/1610

	Author: MontaVista Software, Inc. <source@mvista.com>
	Copyright (c) 2003 MontaVista Software, Inc.

	This is a copy of drivers/pci/pci.c with everything except the pool 
	allocator deleted.  We need the pool allocator for the usb-ohci driver, and 
	some targets which use this driver do not have a PCI bus.
		PCI Bus Services, see include/linux/pci.h for further explanation.
		Copyright 1993 -- 1997 Drew Eckhardt, Frederic Potter,
		David Mosberger-Tang
		Copyright 1997 -- 2000 Martin Mares <mj@ucw.cz>

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; either version 2
	of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

*******************************************************************************/

#include <linux/config.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/spinlock.h>
#include <linux/pm.h>
#include <linux/kmod.h>		/* for hotplug_path */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/cache.h>

#include <asm/page.h>
#include <asm/dma.h>	/* isa_dma_bridge_buggy */

#undef DEBUG

#ifdef DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...)
#endif

/* Here is a hack to replace the pci_alloc_consistent() function with one that 
 * returns the USB local bus address for the DMA handle.  We are assuming 
 * that the local bus MMU has been programmed such that RAM is linearly 
 * mapped to the local bus address space beginning at 0x30000000.  If this 
 * MMU mapping is implemented through static TLB loading as is done above, then 
 * this only works for systems with a maximum of 32MB of RAM as that is the 
 * most we can statically map with our 32 local bus MMU TLBs.
 * It would be cleaner to do this by defining virt_to_bus() and 
 * bus_to_virt() so that they convert between kernel virtual addresses 
 * and USB local bus addresses, but there are other OMAP drivers that have been 
 * written under the assumption that virt_to_bus() and bus_to_virt() convert 
 * between kernel virtual addresses and physical addresses.
 */
#if !defined(CONFIG_ARCH_OMAP1610) && !defined(CONFIG_ARCH_OMAP730)
#define OMAP1510_LB_OFFSET (0x30000000UL)

#define phys_to_lb(x) \
	((x) ? (((unsigned long) (x)) - PHYS_OFFSET + OMAP1510_LB_OFFSET) : 0)
#define lb_to_phys(x) \
	((x) ? (((unsigned long) (x)) - OMAP1510_LB_OFFSET + PHYS_OFFSET) : 0)
#else
/*
 * NOTE: The local bus has been removed in 1610. OHCI controller has full
 *       access to the memory if OCPI is configured to allow access to it.
 */
#define phys_to_lb(x)	(x)
#define lb_to_phys(x)	(x)
#endif

void *omap_usb_pci_alloc_consistent(struct pci_dev *hwdev, 
				    size_t size, dma_addr_t *handle)
{
	void *ret;
	dma_addr_t lb_handle;
	
	ret = pci_alloc_consistent(hwdev, size, handle);

	lb_handle = phys_to_lb(*handle);
	DBG(__FUNCTION__ ": virt=0x%08lx phys=0x%08lx lb=0x%08lx\n",
		(unsigned long) ret, *handle, lb_handle);
	*handle = lb_handle;
	return ret;
}

void omap_usb_pci_free_consistent(struct pci_dev *hwdev, 
				  size_t size, void *vaddr,
				  dma_addr_t dma_handle)
{
	DBG(__FUNCTION__ ": virt=0x%08lx phys=0x%08lx lb=0x%08lx\n", 
		(unsigned long) vaddr, lb_to_phys(dma_handle), 
		dma_handle);
	pci_free_consistent(hwdev, size, vaddr, lb_to_phys(dma_handle));
	return;
}

dma_addr_t omap_usb_pci_map_single(struct pci_dev *hwdev, void *ptr, 
				   size_t size, int direction)
{
	dma_addr_t handle;
	
	handle = pci_map_single(hwdev, ptr, size, direction);

	DBG(__FUNCTION__ ": virt=0x%08lx phys=0x%08lx lb=0x%08lx\n",
		(unsigned long) ptr, handle, phys_to_lb(handle));
	return phys_to_lb(handle);
}

void omap_usb_pci_unmap_single(struct pci_dev *hwdev, dma_addr_t dma_addr, 
				size_t size, int direction)
{
	DBG(__FUNCTION__ ": phys=0x%08lx lb=0x%08lx\n", 
		lb_to_phys(dma_addr), dma_addr);
	pci_unmap_single(hwdev, lb_to_phys(dma_addr), size, direction);
	return;
}

/*
 * The following is a copy of drivers/pci/pci.c with everything except the pool 
 * allocator deleted.  We need the pool allocator for the usb-ohci driver, and 
 * since the OMAP doesn't have a PCI bus the file drivers/pci/pci.c doesn't 
 * get built on the OMAP.  Duplicating the pool allocator code here obviously 
 * isn't the ideal approach, but for now it avoids having to reorganize  
 * the generic PCI code.  The only modifications made to following code were to 
 * replace pci_alloc_consistent/pci_free_consistent with 
 * omap_usb_pci_alloc_consistent/omap_usb_pci_free_consistent.  --JAL
 */
 
/*
 * Pool allocator ... wraps the pci_alloc_consistent page allocator, so
 * small blocks are easily used by drivers for bus mastering controllers.
 * This should probably be sharing the guts of the slab allocator.
 */

struct pci_pool {	/* the pool */
	struct list_head	page_list;
	spinlock_t		lock;
	size_t			blocks_per_page;
	size_t			size;
	int			flags;
	struct pci_dev		*dev;
	size_t			allocation;
	char			name [32];
	wait_queue_head_t	waitq;
};

struct pci_page {	/* cacheable header for 'allocation' bytes */
	struct list_head	page_list;
	void			*vaddr;
	dma_addr_t		dma;
	unsigned long		bitmap [0];
};

#define	POOL_TIMEOUT_JIFFIES	((100 /* msec */ * HZ) / 1000)
#define	POOL_POISON_BYTE	0xa7

// #define CONFIG_PCIPOOL_DEBUG


/**
 * pci_pool_create - Creates a pool of pci consistent memory blocks, for dma.
 * @name: name of pool, for diagnostics
 * @pdev: pci device that will be doing the DMA
 * @size: size of the blocks in this pool.
 * @align: alignment requirement for blocks; must be a power of two
 * @allocation: returned blocks won't cross this boundary (or zero)
 * @flags: SLAB_* flags (not all are supported).
 *
 * Returns a pci allocation pool with the requested characteristics, or
 * null if one can't be created.  Given one of these pools, pci_pool_alloc()
 * may be used to allocate memory.  Such memory will all have "consistent"
 * DMA mappings, accessible by the device and its driver without using
 * cache flushing primitives.  The actual size of blocks allocated may be
 * larger than requested because of alignment.
 *
 * If allocation is nonzero, objects returned from pci_pool_alloc() won't
 * cross that size boundary.  This is useful for devices which have
 * addressing restrictions on individual DMA transfers, such as not crossing
 * boundaries of 4KBytes.
 */
struct pci_pool *
pci_pool_create (const char *name, struct pci_dev *pdev,
	size_t size, size_t align, size_t allocation, int flags)
{
	struct pci_pool		*retval;

	if (align == 0)
		align = 1;
	if (size == 0)
		return 0;
	else if (size < align)
		size = align;
	else if ((size % align) != 0) {
		size += align + 1;
		size &= ~(align - 1);
	}

	if (allocation == 0) {
		if (PAGE_SIZE < size)
			allocation = size;
		else
			allocation = PAGE_SIZE;
		// FIXME: round up for less fragmentation
	} else if (allocation < size)
		return 0;

	if (!(retval = kmalloc (sizeof *retval, flags)))
		return retval;

#ifdef	CONFIG_PCIPOOL_DEBUG
	flags |= SLAB_POISON;
#endif

	strncpy (retval->name, name, sizeof retval->name);
	retval->name [sizeof retval->name - 1] = 0;

	retval->dev = pdev;
	INIT_LIST_HEAD (&retval->page_list);
	spin_lock_init (&retval->lock);
	retval->size = size;
	retval->flags = flags;
	retval->allocation = allocation;
	retval->blocks_per_page = allocation / size;
	init_waitqueue_head (&retval->waitq);

#ifdef CONFIG_PCIPOOL_DEBUG
	printk (KERN_DEBUG "pcipool create %s/%s size %d, %d/page (%d alloc)\n",
		pdev ? pdev->slot_name : NULL, retval->name, size,
		retval->blocks_per_page, allocation);
#endif

	return retval;
}


static struct pci_page *
pool_alloc_page (struct pci_pool *pool, int mem_flags)
{
	struct pci_page	*page;
	int		mapsize;

	mapsize = pool->blocks_per_page;
	mapsize = (mapsize + BITS_PER_LONG - 1) / BITS_PER_LONG;
	mapsize *= sizeof (long);

	page = (struct pci_page *) kmalloc (mapsize + sizeof *page, mem_flags);
	if (!page)
		return 0;
	page->vaddr = omap_usb_pci_alloc_consistent (pool->dev,
						     pool->allocation,
						     &page->dma);
	if (page->vaddr) {
		memset (page->bitmap, 0xff, mapsize);	// bit set == free
		if (pool->flags & SLAB_POISON)
			memset (page->vaddr, POOL_POISON_BYTE, pool->allocation);
		list_add (&page->page_list, &pool->page_list);
	} else {
		kfree (page);
		page = 0;
	}
	return page;
}


static inline int
is_page_busy (int blocks, unsigned long *bitmap)
{
	while (blocks > 0) {
		if (*bitmap++ != ~0UL)
			return 1;
		blocks -= BITS_PER_LONG;
	}
	return 0;
}

static void
pool_free_page (struct pci_pool *pool, struct pci_page *page)
{
	dma_addr_t	dma = page->dma;

	if (pool->flags & SLAB_POISON)
		memset (page->vaddr, POOL_POISON_BYTE, pool->allocation);
	omap_usb_pci_free_consistent (pool->dev, pool->allocation, 
				      page->vaddr, dma);
	list_del (&page->page_list);
	kfree (page);
}


/**
 * pci_pool_destroy - destroys a pool of pci memory blocks.
 * @pool: pci pool that will be destroyed
 *
 * Caller guarantees that no more memory from the pool is in use,
 * and that nothing will try to use the pool after this call.
 */
void
pci_pool_destroy (struct pci_pool *pool)
{
	unsigned long		flags;

#ifdef CONFIG_PCIPOOL_DEBUG
	printk (KERN_DEBUG "pcipool destroy %s/%s\n",
		pool->dev ? pool->dev->slot_name : NULL,
		pool->name);
#endif

	spin_lock_irqsave (&pool->lock, flags);
	while (!list_empty (&pool->page_list)) {
		struct pci_page		*page;
		page = list_entry (pool->page_list.next,
				struct pci_page, page_list);
		if (is_page_busy (pool->blocks_per_page, page->bitmap)) {
			printk (KERN_ERR "pci_pool_destroy %s/%s, %p busy\n",
				pool->dev ? pool->dev->slot_name : NULL,
				pool->name, page->vaddr);
			/* leak the still-in-use consistent memory */
			list_del (&page->page_list);
			kfree (page);
		} else
			pool_free_page (pool, page);
	}
	spin_unlock_irqrestore (&pool->lock, flags);
	kfree (pool);
}


/**
 * pci_pool_alloc - get a block of consistent memory
 * @pool: pci pool that will produce the block
 * @mem_flags: SLAB_KERNEL or SLAB_ATOMIC
 * @handle: pointer to dma address of block
 *
 * This returns the kernel virtual address of a currently unused block,
 * and reports its dma address through the handle.
 * If such a memory block can't be allocated, null is returned.
 */
void *
pci_pool_alloc (struct pci_pool *pool, int mem_flags, dma_addr_t *handle)
{
	unsigned long		flags;
	struct list_head	*entry;
	struct pci_page		*page;
	int			map, block;
	size_t			offset;
	void			*retval;

restart:
	spin_lock_irqsave (&pool->lock, flags);
	list_for_each (entry, &pool->page_list) {
		int		i;
		page = list_entry (entry, struct pci_page, page_list);
		/* only cachable accesses here ... */
		for (map = 0, i = 0;
				i < pool->blocks_per_page;
				i += BITS_PER_LONG, map++) {
			if (page->bitmap [map] == 0)
				continue;
			block = ffz (~ page->bitmap [map]);
			if ((i + block) < pool->blocks_per_page) {
				clear_bit (block, &page->bitmap [map]);
				offset = (BITS_PER_LONG * map) + block;
				offset *= pool->size;
				goto ready;
			}
		}
	}
	if (!(page = pool_alloc_page (pool, mem_flags))) {
		if (mem_flags == SLAB_KERNEL) {
			DECLARE_WAITQUEUE (wait, current);

			current->state = TASK_INTERRUPTIBLE;
			add_wait_queue (&pool->waitq, &wait);
			spin_unlock_irqrestore (&pool->lock, flags);

			schedule_timeout (POOL_TIMEOUT_JIFFIES);

			current->state = TASK_RUNNING;
			remove_wait_queue (&pool->waitq, &wait);
			goto restart;
		}
		retval = 0;
		goto done;
	}

	clear_bit (0, &page->bitmap [0]);
	offset = 0;
ready:
	retval = offset + page->vaddr;
	*handle = offset + page->dma;
done:
	spin_unlock_irqrestore (&pool->lock, flags);
	return retval;
}


static struct pci_page *
pool_find_page (struct pci_pool *pool, dma_addr_t dma)
{
	unsigned long		flags;
	struct list_head	*entry;
	struct pci_page		*page;

	spin_lock_irqsave (&pool->lock, flags);
	list_for_each (entry, &pool->page_list) {
		page = list_entry (entry, struct pci_page, page_list);
		if (dma < page->dma)
			continue;
		if (dma < (page->dma + pool->allocation))
			goto done;
	}
	page = 0;
done:
	spin_unlock_irqrestore (&pool->lock, flags);
	return page;
}


/**
 * pci_pool_free - put block back into pci pool
 * @pool: the pci pool holding the block
 * @vaddr: virtual address of block
 * @dma: dma address of block
 *
 * Caller promises neither device nor driver will again touch this block
 * unless it is first re-allocated.
 */
void
pci_pool_free (struct pci_pool *pool, void *vaddr, dma_addr_t dma)
{
	struct pci_page		*page;
	unsigned long		flags;
	int			map, block;

	if ((page = pool_find_page (pool, dma)) == 0) {
		printk (KERN_ERR "pci_pool_free %s/%s, %p/%x (bad dma)\n",
			pool->dev ? pool->dev->slot_name : NULL,
			pool->name, vaddr, (int) (dma & 0xffffffff));
		return;
	}
#ifdef	CONFIG_PCIPOOL_DEBUG
	if (((dma - page->dma) + (void *)page->vaddr) != vaddr) {
		printk (KERN_ERR "pci_pool_free %s/%s, %p (bad vaddr)/%x\n",
			pool->dev ? pool->dev->slot_name : NULL,
			pool->name, vaddr, (int) (dma & 0xffffffff));
		return;
	}
#endif

	block = dma - page->dma;
	block /= pool->size;
	map = block / BITS_PER_LONG;
	block %= BITS_PER_LONG;

#ifdef	CONFIG_PCIPOOL_DEBUG
	if (page->bitmap [map] & (1UL << block)) {
		printk (KERN_ERR "pci_pool_free %s/%s, dma %x already free\n",
			pool->dev ? pool->dev->slot_name : NULL,
			pool->name, dma);
		return;
	}
#endif
	if (pool->flags & SLAB_POISON)
		memset (vaddr, POOL_POISON_BYTE, pool->size);

	spin_lock_irqsave (&pool->lock, flags);
	set_bit (block, &page->bitmap [map]);
	if (waitqueue_active (&pool->waitq))
		wake_up (&pool->waitq);
	/*
	 * Resist a temptation to do
	 *    if (!is_page_busy(bpp, page->bitmap)) pool_free_page(pool, page);
	 * it is not interrupt safe. Better have empty pages hang around.
	 */
	spin_unlock_irqrestore (&pool->lock, flags);
}

/* Pool allocator */

EXPORT_SYMBOL (pci_pool_create);
EXPORT_SYMBOL (pci_pool_destroy);
EXPORT_SYMBOL (pci_pool_alloc);
EXPORT_SYMBOL (pci_pool_free);
EXPORT_SYMBOL (omap_usb_pci_alloc_consistent);
EXPORT_SYMBOL (omap_usb_pci_free_consistent);
EXPORT_SYMBOL (omap_usb_pci_map_single);
EXPORT_SYMBOL (omap_usb_pci_unmap_single);

MODULE_LICENSE("GPL");
