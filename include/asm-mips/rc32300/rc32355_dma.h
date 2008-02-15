/*
 *
 * BRIEF MODULE DESCRIPTION
 *      DMA controller defines on IDT RC32355 (Banyan)
 *
 * Copyright 2002 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *         	stevel@mvista.com or source@mvista.com
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

#ifndef BANYAN_DMA_H
#define BANYAN_DMA_H

/*
 * An image of one RC32355 dma channel registers
 */
typedef struct {
	u32 dmac;
	u32 dmas;
	u32 dmasm;
	u32 dmadptr;
	u32 dmandptr;
} rc32355_dma_ch_t;

/*
 * An image of all RC32355 dma channel registers
 */
typedef struct {
	rc32355_dma_ch_t ch[16];
} rc32355_dma_regs_t;


#define rc32355_dma_regs ((rc32355_dma_regs_t*)KSEG1ADDR(RC32355_DMA_BASE))


/* DMAC register layout */

#define DMAC_RUN	0x1	/* Halts processing when cleared	*/
#define DMAC_DM		0x2	/* Done Mask, ignore DMA events		*/
#define DMAC_MODE_MASK	0xC	/* DMA operating mode			*/

#define DMAC_MODE_AUTO	0x0	/* DMA Auto Request Mode		*/
#define DMAC_MODE_BURST	0x4	/* DMA Burst Request Mode		*/
#define DMAC_MODE_TFER	0x8	/* DMA Transfer Request Mode		*/

/* DMAS and DMASM register layout */

#define DMAS_F		0x01	/* Finished */
#define DMAS_D		0x02	/* Done */
#define DMAS_C		0x04	/* Chain */
#define DMAS_E		0x08	/* Error */
#define DMAS_H		0x10	/* Halt */

/* Polling count for DMAS_H bit in DMAS register after halting DMA */
#define DMA_HALT_TIMEOUT 500

static inline int rc32355_halt_dma(rc32355_dma_ch_t* ch)
{
	int timeout=1;
	
	if (readl(&ch->dmac) & DMAC_RUN) {
		writel(0, &ch->dmac); 
		for (timeout = DMA_HALT_TIMEOUT; timeout > 0; timeout--) {
			if (readl(&ch->dmas) & DMAS_H) {
				writel(0, &ch->dmas);  
				break;
			}
		}
	}

	return timeout ? 0 : 1;
}

static inline void rc32355_start_dma(rc32355_dma_ch_t* ch, u32 dma_addr)
{
	writel(0, &ch->dmandptr); 
	writel(dma_addr, &ch->dmadptr);
}

static inline void rc32355_chain_dma(rc32355_dma_ch_t* ch, u32 dma_addr)
{
	writel(dma_addr, &ch->dmandptr);
}


/* The following can be used to describe DMA channels 0 to 15, and the	*/
/* sub device's needed to select them in the DMADESC_DS_MASK field	*/

#define DMA_CHAN_ATM01		0	     /* ATM interface 0,1 chan	*/

#define DMA_CHAN_ATM0IN		0	     /* ATM interface 0 input	*/
#define DMA_DEV_ATM0IN		0	     /* ATM interface 0 input	*/

#define DMA_CHAN_ATM1IN		0	     /* ATM interface 1 input	*/
#define DMA_DEV_ATM1IN		1	     /* ATM interface 1 input	*/

#define DMA_CHAN_ATM0OUT	0	     /* ATM interface 0 output	*/
#define DMA_DEV_ATM0OUT		2	     /* ATM interface 0 output	*/

#define DMA_CHAN_ATM1OUT	0	     /* ATM interface 1 output	*/
#define DMA_DEV_ATM1OUT		3	     /* ATM interface 1 output	*/

/* for entry in {0,1,2,3,4,5,6,7} - note 5,6,7 share with those below */
#define DMA_CHAN_ATMVCC(entry)	((entry)+1)  /* ATM VC cache entry 	*/
#define DMA_DEV_ATMVCC(entry)	0

#define DMA_CHAN_MEMTOMEM	6	     /* Memory to memory DMA 	*/
#define DMA_DEV_MEMTOMEM	1	     /* Memory to memory DMA 	*/

#define DMA_CHAN_ATMFMB0	7	     /* ATM Frame Mode Buffer 0	*/
#define DMA_DEV_ATMFMB0		1	     /* ATM Frame Mode Buffer 0	*/

#define DMA_CHAN_ATMFMB1	8	     /* ATM Frame Mode Buffer 1	*/
#define DMA_DEV_ATMFMB1		1	     /* ATM Frame Mode Buffer 1	*/

#define DMA_CHAN_ETHERIN	9	     /* Ethernet input		*/
#define DMA_DEV_ETHERIN		0	     /* Ethernet input		*/

#define DMA_CHAN_ETHEROUT	10	     /* Ethernet output		*/
#define DMA_DEV_ETHEROUT	0	     /* Ethernet output		*/

#define DMA_CHAN_TDMIN		11	     /* TDM Bus input		*/
#define DMA_DEV_TDMIN		0	     /* TDM Bus input		*/

#define DMA_CHAN_TDMOUT		12	     /* TDM Bus output		*/
#define DMA_DEV_TDMOUT		0	     /* TDM Bus output		*/

#define DMA_CHAN_USBIN		13	     /* USB input		*/
#define DMA_DEV_USBIN		0	     /* USB input		*/

#define DMA_CHAN_USBOUT		14	     /* USB output		*/
#define DMA_DEV_USBOUT		0	     /* USB output		*/

#define DMA_CHAN_EXTERN		15	     /* External DMA		*/
#define DMA_DEV_EXTERN		0	     /* External DMA		*/

/*
 * An RC32355 dma descriptor in system memory
 */
typedef struct {
	u32 cmdstat;	/* control and status */
	u32 curr_addr;	/* current address of data */
	u32 devcs;	/* peripheral-specific control and status */
	u32 link;	/* link to next descriptor */
} rc32355_dma_desc_t;

/* Values for the descriptor cmdstat word */

#define DMADESC_F		0x80000000u  /* Finished bit		*/
#define DMADESC_D		0x40000000u  /* Done bit		*/
#define DMADESC_T		0x20000000u  /* Terminated bit		*/
#define DMADESC_IOD		0x10000000u  /* Interrupt On Done	*/
#define DMADESC_IOF		0x08000000u  /* Interrupt On Finished	*/
#define DMADESC_COD		0x04000000u  /* Chain On Done		*/
#define DMADESC_COF		0x02000000u  /* Chain On Finished	*/

#define DMADESC_DEVCMD_MASK	0x01C00000u  /* Device Command mask	*/
#define DMADESC_DEVCMD_SHIFT	22	     /* Device Command shift	*/

#define DMADESC_DS_MASK		0x00300000u  /* Device Select mask	*/
#define DMADESC_DS_SHIFT	20	     /* Device Select shift	*/

#define DMADESC_COUNT_MASK	0x0003FFFFu  /* Byte Count mask		*/
#define DMADESC_COUNT_SHIFT	0	     /* Byte Count shift	*/

#define IS_DMA_FINISHED(X)   ( ( (X) & DMADESC_F ) >> 31)   /* F Bit    */
#define IS_DMA_DONE(X)       ( ( (X) & DMADESC_D ) >> 30)   /* D Bit    */
#define IS_DMA_TERMINATED(X) ( ( (X) & DMADESC_T ) >> 29)   /* T Bit    */
#define IS_DMA_USED(X) (((X) & (DMADESC_F | DMADESC_D | DMADESC_T)) != 0)

#define DMA_DEVCMD(devcmd) \
  (((devcmd) << DMADESC_DEVCMD_SHIFT) & DMADESC_DS_MASK)
#define DMA_DS(ds)         \
  (((ds) << DMADESC_DS_SHIFT) & DMADESC_DS_MASK)
#define DMA_COUNT(count)   \
  ((count) & DMADESC_COUNT_MASK)

#endif /* BANYAN_DMA_H */
