/*
 *
 * BRIEF MODULE DESCRIPTION
 *	Defines for using the DMA channels of the RC32334.
 *
 * Copyright 2001,2002 THOMSON multimedia.
 * Author: Stephane Fillod
 *         	fillods@thmulti.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __RC32334_DMA_H
#define __RC32334_DMA_H


#include <linux/config.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <asm/scatterlist.h>
#include <linux/string.h>
#include <asm/io.h>			/* for virt_to_bus()  */
#include <linux/pci.h>		/* needed for sg_dma_len et al. */

#define RC32334_MAX_DMA_CHANNELS 4

/*
 * An image of one RC32334 dma channel registers
 */
typedef struct {
	u32 config;      // 00
	u32 base_desc;   // 04
	u32 curr_addr;   // 08
	u32 reserved;    // 0C
	u32 status;      // 10
	u32 src_addr;    // 14
	u32 dst_addr;    // 18
	u32 next_desc;   // 1C
} rc32334_dma_reg_t;

#define DMA_BASE(n) (((n) > 1) ? \
    DMA23_BASE + ((n)-2)*DMA_CHAN_OFFSET : DMA01_BASE + (n)*DMA_CHAN_OFFSET)
#define DMA_CONFREG(n)  (DMA_BASE(n) + 0x00)
#define DMA_BASEREG(n)  (DMA_BASE(n) + 0x04)
#define DMA_CURRREG(n)  (DMA_BASE(n) + 0x08)
#define DMA_STATREG(n)  (DMA_BASE(n) + 0x10)
#define DMA_SRCREG(n)   (DMA_BASE(n) + 0x14)
#define DMA_DSTREG(n)   (DMA_BASE(n) + 0x18)
#define DMA_NEXTREG(n)  (DMA_BASE(n) + 0x1c)

#define DMA_IRQ(n)  (GROUP7_IRQ_BASE+5*(n))


/*
 * DMA descriptor as used by RC32334 DMA
 */
typedef struct {
	u32 status;
	u32 src_addr;
	u32 dst_addr;
	u32 next;
} rc32334_dma_desc_t;

/* max size of a transaction, DMA cannot do full 64KB, such a shame! */
#define DMA_MAX_TRANSZ 65535

#define DMA_DUMMY_ADDR 0xffffffff

/* This defines the direction arg to the DMA mapping routines. */
#define DMA_BIDIRECTIONAL 0
#define DMA_TODEVICE      1
#define DMA_FROMDEVICE    2
#define DMA_NONE          3

/*
 * default(ie. 0) is SRC incr/DST incr, little-endian in and out.
 * these flags are exactly the same as the one on the DMA registers.
 * see rc32334_dma_setup_single() and rc32334_dma_setup_sg().
 */
#define DMA_SRC_CONST	0x04000000
#define DMA_SRC_DEC	0x02000000
#define DMA_DST_CONST	0x01000000
#define DMA_DST_DEC	0x00800000
#define DMA_FROM_ADDR	(DMA_SRC_CONST|DMA_SRC_DEC)
#define DMA_TO_ADDR	(DMA_DST_CONST|DMA_DST_DEC)

#define DMA_SRC_BE	0x00400000
#define DMA_DST_BE	0x00200000

#define DMA_DONE_INT	0x08000000

/*
 * not for use above HAL
 */
#define DMA_DMAOWNER	0x80000000
#define DMA_LASTDESC	0x10000000


/*
 * Configuration register
 * default (i.e. 0) is disabled, restart from first descriptor,
 * 	ignore dma_rdy_pin, and max burst size of 1 byte.
 */
#define DMA_ENABLE	0x80000000
#define DMA_RESTART	0x40000000
#define DMA_RDY		0x10000000
#define DMA_DONE	0x08000000
#define DMA_BURSTSZ16	0x04000000
#define DMA_BURSTSZ4	0x02000000
#define DMA_BURSTSZ2	0x01000000
#define DMA_BURSTSZ1	0x00000000


/*
 * Allocate and map kernel buffer using consistent mode DMA for a device.
 * hwdev is totally ignored, and has been kept just for to mimic 
 * rc32334_alloc_consistent.
 * Returns non-NULL cpu-view pointer to the buffer if successful and
 * sets *dma_addrp to the idt side dma address as well, else *dma_addrp
 * is undefined.
 */
extern void *rc32334_alloc_consistent(void *hwdev, size_t size,
				      dma_addr_t *dma_handle);

/*
 * Free and unmap a consistent DMA buffer.
 * cpu_addr is what was returned from rc32334_alloc_consistent,
 * size must be the same as what as passed into rc32334_alloc_consistent,
 * and likewise dma_addr must be the same as what *dma_addrp was set to.
 *
 * References to the memory and mappings associated with cpu_addr/dma_addr
 * past this call are illegal.
 */
extern void rc32334_free_consistent(void *hwdev, size_t size,
				    void *vaddr, dma_addr_t dma_handle);

/*
 * Map a single buffer of the indicated size for DMA in streaming mode.
 * The 32-bit bus address to use is returned.
 *
 * Once the device is given the dma address, the device owns this memory
 * until either rc32334_unmap_single or rc32334_dma_sync_single is performed.
 *
 * NOTE NOTE NOTE: ptr MUST be a logical address! it won't work on
 * mmapped address!
 */
extern inline dma_addr_t rc32334_map_single(void *hwdev, void *ptr,
					    size_t size, int direction)
{
	dma_cache_wback_inv((unsigned long)ptr, size);

	return virt_to_bus(ptr);
}

/*
 * Unmap a single streaming mode DMA translation.  The dma_addr and size
 * must match what was provided for in a previous rc32334_map_single call.  All
 * other usages are undefined.
 *
 * After this call, reads by the cpu to the buffer are guarenteed to see
 * whatever the device wrote there.
 */
extern inline void rc32334_unmap_single(void *hwdev, dma_addr_t dma_addr,
					size_t size, int direction)
{
	/* Nothing to do */
}

/*
 * rc32334_{map,unmap}_single_page maps a kernel page to a dma_addr_t. identical
 * to rc32334_map_single, but takes a struct page instead of a virtual address
 */
static inline dma_addr_t rc32334_map_page(void *hwdev, struct page *page,
					  unsigned long offset, size_t size,
					  int direction)
{
	unsigned long addr;

	addr = (unsigned long) page_address(page);
	addr += offset;
#ifdef CONFIG_NONCOHERENT_IO
	dma_cache_wback_inv(addr, size);
#endif

	return virt_to_bus((void *)addr);
}

static inline void rc32334_unmap_page(void *hwdev, dma_addr_t dma_address,
				      size_t size, int direction)
{
	/* Nothing to do */
}

/*
 * Map a set of buffers described by scatterlist in streaming
 * mode for DMA.  This is the scather-gather version of the
 * above rc32334_map_single interface.  Here the scatter gather list
 * elements are each tagged with the appropriate dma address
 * and length.  They are obtained via sg_dma_{address,length}(SG).
 *
 * NOTE: An implementation may be able to use a smaller number of
 *       DMA address/length pairs than there are SG table elements.
 *       (for example via virtual mapping capabilities)
 *       The routine returns the number of addr/length pairs actually
 *       used, at most nents.
 *
 * Device ownership issues as mentioned above for rc32334_map_single are
 * the same here.
 */
extern inline int rc32334_map_sg(void *hwdev, const struct scatterlist *sg,
				 int nents, int direction)
{
#ifndef CONFIG_COHERENT_IO
	int i;
#endif

#ifndef CONFIG_COHERENT_IO
	/* Make sure that gcc doesn't leave the empty loop body.  */
	for (i = 0; i < nents; i++, sg++)
		dma_cache_wback_inv((unsigned long)sg->address, sg->length);
#endif

	return nents;
}

/*
 * Unmap a set of streaming mode DMA translations.
 * Again, cpu read rules concerning calls here are the same as for
 * rc32334_unmap_single() above.
 */
extern inline void rc32334_unmap_sg(void *hwdev, const struct scatterlist *sg,
				    int nents, int direction)
{
	/* Nothing to do */
}

/*
 * Make physical memory consistent for a single
 * streaming mode DMA translation after a transfer.
 *
 * If you perform a rc32334_map_single() but wish to interrogate the
 * buffer using the cpu, yet do not wish to teardown the PCI dma
 * mapping, you must call this function before doing so.  At the
 * next point you give the PCI dma address back to the card, the
 * device again owns the buffer.
 */
extern inline void rc32334_dma_sync_single(void *hwdev,
					   dma_addr_t dma_handle,
					   size_t size, int direction)
{
	dma_cache_wback_inv((unsigned long)bus_to_virt(dma_handle), size);
}

/*
 * Make physical memory consistent for a set of streaming
 * mode DMA translations after a transfer.
 *
 * The same as rc32334_dma_sync_single but for a scatter-gather list,
 * same rules and usage.
 */
extern inline void rc32334_dma_sync_sg(void *hwdev,
				       const struct scatterlist *sg,
				       int nelems, int direction)
{
#ifndef CONFIG_COHERENT_IO
	int i;
#endif

	/* Make sure that gcc doesn't leave the empty loop body.  */
#ifndef CONFIG_COHERENT_IO
	for (i = 0; i < nelems; i++, sg++)
		dma_cache_wback_inv((unsigned long)sg->address, sg->length);
#endif
}


extern void rc32334_dma_setup_single(unsigned int dmanr,
				     dma_addr_t src, dma_addr_t dst,
				     size_t size, int status, 
				     void (*dma_done_cb)(void *), void *arg);
extern void rc32334_dma_initiate_single(unsigned int dmanr, unsigned int cfg);
extern void rc32334_dma_setup_sg(unsigned int dmanr,
				 rc32334_dma_desc_t *sgdescs,
				 const struct scatterlist *sg, 
				 dma_addr_t addr, int nents, int status,
				 void (*dma_done_cb)(void *), void *arg);
extern void rc32334_dma_setup_from_sg(unsigned int dmanr,
				      rc32334_dma_desc_t *sgdescs,
				      const struct scatterlist *sg, 
				      dma_addr_t addr, int nents, int status,
				      void (*dma_done_cb)(void *), void *arg);
extern void rc32334_dma_setup_to_sg(unsigned int dmanr,
				    rc32334_dma_desc_t *sgdescs,
				    const struct scatterlist *sg, 
				    dma_addr_t addr, int nents, int status,
				    void (*dma_done_cb)(void *), void *arg);
extern void rc32334_dma_initiate_sg(unsigned int dmanr, unsigned int cfg);
extern void rc32334_dma_disable(unsigned int dmanr);


/* These are in kernel/dma.c: */
/* reserve a DMA channel */
extern int request_dma(unsigned int dmanr, const char * device_id);
extern void free_dma(unsigned int dmanr);	/* release it again */

#endif /* __RC32334_DMA_H */
