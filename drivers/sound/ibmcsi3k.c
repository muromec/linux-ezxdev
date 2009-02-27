/*
 *      ibmcsi3k.c : IBM PowerPC 405LP Codec Serial Interface (CSI) +
 *                      Si3000 voiceband codec driver
 *                      for the 405LP evaluation board
 *
 *	Based on various sound drivers in linux/drivers/sound, including but not limited to
 *	es1370.c and vwsnd.c, as well as an unfinished driver from IBM Austin Research Lab.
 *
 *      Copyright (C) 2002  IBM Corp.
 *
 *  Module command line parameters:
 *  	To be determined
 *
 *  Supported devices:
 *  	/dev/dsp    standard /dev/dsp device, hopefully OSS compatible
 *  	/dev/mixer  standard /dev/mixer device, hopefully OSS compatible
 *
 * TODOs:
 *
 * - Sampling rate is fixed at 11.025 KHz.
 * - Sample format is limited to 16 bit big endian.
 *	(this is a deviation since an OSS DSP device is supposed to support 8 bit as default.)
 * - Drain DAC/ADC
 * - Fragment handling
 * - Simultaneous capture and playback (TBD)
 * - MMAP support (TBD)
 * - Split CSI and Si3000 drivers (TBD)
 * - Module parameters (TBD)
 * - 128 bit per frame timing adjustment for all CPU speeds (TBD)
 * - Tune retry counts and jiffies
 * - Revisit inline functions
 *
 * - Write ibmcsi3k.txt in the Documentation directory
 * - Anything else?
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
/*****************************************************************************/

/*****************************************************************************/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/sound.h>
#include <linux/slab.h>
#include <linux/soundcard.h>
#include <linux/smp_lock.h>
#include <linux/wrapper.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <asm/delay.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <asm/hardirq.h>
#include <asm/time.h>

#include <platforms/ibm405lp.h>

#include "ibmcsi3k.h"

/*****************************************************************************/
/* Start #defines that might belong in ibm405lp.h */

#define CSI0_IO_BASE 0xEF600900 /* CSI I/O base address				*/

#define CSI0_ER	        (CSI0_IO_BASE + 0)	/* Enable register		*/
#define CSI0_CFG	(CSI0_IO_BASE + 4)	/* Configuration register	*/
#define CSI0_SR         (CSI0_IO_BASE + 8)	/* Status register		*/
#define CSI0_TBUF	(CSI0_IO_BASE + 0x0C)	/* Transmit buffer		*/
#define CSI0_RBUF   	(CSI0_IO_BASE + 0x10)	/* Receive buffer		*/
#define CSI0_ID		(CSI0_IO_BASE + 0x14)	/* ID				*/
#define CSI0_SCR	(CSI0_IO_BASE + 0x18)	/* Sleep control register	*/

#define CSI0_EXTENT 28			/* I/O address extent			*/
#define CSI_ID_405LP	0x1107			/* CSI core ID (halfword) */


#define CSI_ER_ECSI	0x00000001	/* Enable the CSI			*/
#define CSI_ER_RXEN	0x00000002	/* Receive enable			*/
#define CSI_ER_TXEN	0x00000004	/* Transmit enable			*/

#define CSI_ER_ESLOT(n) (0x80000000 >> (n)) /* Enable (n)th slot		*/

#define CSI_SR_TOD	0x00000001	/* Transmit overrun 			*/
#define CSI_SR_ROD	0x00000002	/* Receive overrun			*/
#define CSI_SR_CSIB	0x00008000	/* CSI Busy				*/

/*-----------------------------------------------------------------------------+
| DMA Channel Control register
| (Device Control Register bus register DCRN_DMA0_CR0 through DCRN_DMA0_CR3)
+-----------------------------------------------------------------------------*/

#define DCRN_DMA_CR_CE                 0x80000000   /* Channel Enable             */
#define DCRN_DMA_CR_CIE                0x40000000   /* Channel Interrupt Enable   */
#define DCRN_DMA_CR_TD                 0x20000000   /* Transfer Direction         */
#define DCRN_DMA_CR_PL                 0x10000000   /* Peripheral Location        */
#define DCRN_DMA_CR_PW_8               0x00000000   /* 8 bit peripheral width     */
#define DCRN_DMA_CR_PW_16              0x04000000   /* 16 bit peripheral width    */
#define DCRN_DMA_CR_PW_32              0x08000000   /* 32 bit peripheral width    */
#define DCRN_DMA_CR_PW_64              0x0c000000   /* 64 bit peripheral width    */
#define DCRN_DMA_CR_DAI                0x02000000   /* Destination Addr Increment */
#define DCRN_DMA_CR_SAI                0x01000000   /* Source Address Increment   */
#define DCRN_DMA_CR_BE                 0x00800000   /* Buffer Enable              */
#define DCRN_DMA_CR_TM_BUFFERED        0x00000000   /* Buffered Transfer mode     */
#define DCRN_DMA_CR_TM_SW_MEM_TO_MEM   0x00400000   /* Software started mem to mem*/
#define DCRN_DMA_CR_TM_HW_MEM_TO_MEM   0x00600000   /* Hardware paced mem to mem  */
#define DCRN_DMA_CR_PSC_0              0x00000000   /* 0 Peripheral Setup Cycles  */
#define DCRN_DMA_CR_PSC_1              0x00080000   /* 1 Peripheral Setup Cycles  */
#define DCRN_DMA_CR_PSC_2              0x00100000   /* 2 Peripheral Setup Cycles  */
#define DCRN_DMA_CR_PSC_3              0x00180000   /* 3 Peripheral Setup Cycles  */
#define DCRN_DMA_CR_PSC(n) (((n)&0x3)<<19)          /* n Peripheral setup cycles  */
#define DCRN_DMA_CR_PWC(n) (((n)&0x3f)<<13)         /* n peripheral wait cycles   */
#define DCRN_DMA_CR_PHC(n) (((n)&0x7)<<10)          /* n peripheral hold cycles   */
#define DCRN_DMA_CR_ETD                0x00000200   /* EOT/TC Pin Direction       */
#define DCRN_DMA_CR_TCE                0x00000100   /* Terminal Count Enable      */
#define DCRN_DMA_CR_CP_MASK            0x000000C0   /* Channel Priority           */
#define DCRN_DMA_CR_CP(n) (((n)&0x3)<<6)
#define DCRN_DMA_CR_PF                 0x00000030   /* Memory read prefetch trans */
#define DCRN_DMA_CR_PF_1               0x00000000   /*   Prefetch 1 dword         */
#define DCRN_DMA_CR_PF_2               0x00000010   /*   Prefetch 2 dword         */
#define DCRN_DMA_CR_PF_4               0x00000020   /*   Prefetch 4 dword         */
#define DCRN_DMA_CR_PCE                0x00000008   /* Parity check enable        */
#define DCRN_DMA_CR_DEC                0x00000004   /* Address decrement          */

/*-----------------------------------------------------------------------------+
| DMA Status Register
| (Device Control Register bus register DCRN_DMA0_SR)
+-----------------------------------------------------------------------------*/
/* (n) = DMA channel number, 0-3							*/

#define DCRN_DMA_SR_CS(n)	(0x80000000 >>(n))  /* Terminal count status		*/
#define DCRN_DMA_SR_TS(n)       (0x08000000 >>(n))  /* End Of Transfer status		*/
#define DCRN_DMA_SR_RI(n)	(0x00800000 >>(n))  /* Error status    			*/
#define DCRN_DMA_SR_IR(n)	(0x00080000 >>(n))  /* Internal DMA request pending	*/
#define DCRN_DMA_SR_ER(n)	(0x00008000 >>(n))  /* External DMA request pending	*/
#define DCRN_DMA_SR_CB(n)	(0x00000800 >>(n))  /* Channel Busy        		*/
#define DCRN_DMA_SR_SG(n)	(0x00000080 >>(n))  /* Scatter/gather status		*/

/* Status register bits for the (n)th channel (write to clear) */
#define DCRN_DMA_SR_ALL(n) (DCRN_DMA_SR_CS(n) | \
			DCRN_DMA_SR_TS(n) | \
			DCRN_DMA_SR_RI(n) | \
			DCRN_DMA_SR_IR(n) | \
			DCRN_DMA_SR_ER(n) | \
			DCRN_DMA_SR_CB(n) | \
			DCRN_DMA_SR_SG(n) )

/* DCRN_DMA0_SGC Scatter/Gather Command Register bits */
#define DCRN_DMA_SGC_SSG0 0x80000000
#define DCRN_DMA_SGC_SSG1 0x40000000
#define DCRN_DMA_SGC_SSG2 0x20000000
#define DCRN_DMA_SGC_SSG3 0x10000000

#define DCRN_DMA_SGC_EM0  0x00008000
#define DCRN_DMA_SGC_EM1  0x00004000
#define DCRN_DMA_SGC_EM2  0x00002000
#define DCRN_DMA_SGC_EM3  0x00001000


struct dma_sgdt { /* Must be word aligned */
	unsigned int ccw; /* Channel Control Word */
	void *srcP;	/* Source address (physical) */
	void *destP;	/* Destination address (physical) */
	unsigned int ctrl; /* MSB = link bit, lower halfword = count */
			/* Other 3 bits unused */
	void *nextP;	/* Next scatter/gather descriptor list physical address */
	/* ------------------------------------- Private use ---------------*/
	struct dma_sgdt *prevV;	/* Prev scatter/gather descriptor list virtual address */
	struct dma_sgdt *nextV; /* Next */
	unsigned int dummy;	/* Reserved (for 16 byte alignment) */
};



/* End ibm405lp.h candidates */


/*****************************************************************************/
/* Driver specific defines 						     */
/*****************************************************************************/

/* The DMA channels for the CSI are hardcoded in the 405LP chip, so we hardcode them.	*/
/* If a future chip adopts programmable channel assignment, I expect access to DMA    	*/
/* channels would be handled by a separate driver.					*/

#define IBMCSI_TXDMA 	0		/* Transmit from CSI to Si3000 : channel 0 	*/
#define IBMCSI_RXDMA	1		/* Receive from Si3000 to CSI  : channel 1	*/

#define IBMCSI_TXDMA_IRQ	5
#define IBMCSI_RXDMA_IRQ	6

#define IBMCSI_DMA_SR	DCRN_DMASR

/* Transmit (playback) DMA registers	*/
#define IBMCSI_TXDMA_CR	DCRN_DMACR0
#define IBMCSI_TXDMA_DA	DCRN_DMADA0
#define IBMCSI_TXDMA_SA	DCRN_DMASA0
#define IBMCSI_TXDMA_CT	DCRN_DMACT0

/* Receive (capture) DMA registers	*/
#define IBMCSI_RXDMA_CR	DCRN_DMACR1
#define IBMCSI_RXDMA_DA	DCRN_DMADA1
#define IBMCSI_RXDMA_SA	DCRN_DMASA1
#define IBMCSI_RXDMA_CT	DCRN_DMACT1


#define IBMCSI_TXDMA_CONFIG (		    	/* Channel disabled			*/ \
			DCRN_DMA_CR_CIE |    	/* Channel interrupt enabled 		*/ \
					    	/* Memory to peripheral			*/ \
      			DCRN_DMA_CR_PL	|   	/* Peripheral on OPB			*/ \
			DCRN_DMA_CR_PW_32 |  	/* 32 bit wide peripheral		*/ \
						/* Dest address not incremented		*/ \
			DCRN_DMA_CR_SAI	| 	/* Source address incremented		*/ \
						/* Peripheral transfer mode 		*/ \
						/* Peripheral setup cycle 0 		*/ \
			DCRN_DMA_CR_PWC(2) |	/* Peripheral wait cycle 3 		*/ \
						/* Peripheral hold cycle 0		*/ \
                        DCRN_DMA_CR_ETD |	/* EOTn = TC				*/ \
                        DCRN_DMA_CR_TCE )	/* Terminal count enable		*/

#define IBMCSI_TXDMA_GO ( IBMCSI_TXDMA_CONFIG | DCRN_DMA_CR_CE )	/* For int	*/
#define IBMCSI_TXDMA_GO_NOI ( IBMCSI_TXDMA_GO & ~DCRN_DMA_CR_CIE ) 	/* For polling	*/

#define IBMCSI_RXDMA_CONFIG (			/* Channel disabled			*/ \
			DCRN_DMA_CR_CIE |    	/* Channel interrupt enabled 		*/ \
			DCRN_DMA_CR_TD  |   	/* Peripheral to memory 		*/ \
			DCRN_DMA_CR_PL	|   	/* Peripheral on OPB			*/ \
			DCRN_DMA_CR_PW_32 |  	/* 32 bit wide peripheral		*/ \
			DCRN_DMA_CR_DAI |   	/* Dest address incremented		*/ \
			                   	/* Source address not incremented	*/ \
					    	/* Peripheral transfer mode 		*/ \
					    	/* Peripheral setup cycle 0 		*/ \
			DCRN_DMA_CR_PWC(2) |	/* Peripheral wait cycle 3 		*/ \
				            	/* Peripheral hold cycle 0		*/ \
                        DCRN_DMA_CR_ETD |   	/* EOTn = TC				*/ \
                        DCRN_DMA_CR_TCE ) 	/* Terminal count enable		*/

#define IBMCSI_RXDMA_GO (IBMCSI_RXDMA_CONFIG | DCRN_DMA_CR_CE )
#define IBMCSI_RXDMA_GO_NOI (IBMCSI_RXDMA_GO & ~DCRN_DMA_CR_CIE )

#define IBMCSI_DEFAULT_SAMPLING_RATE 11025

/* Scatter/Gather related */
#define USE_SG 

#define DAC_TIMER_PERIOD (HZ/50)
#define ADC_TIMER_PERIOD (HZ/50)

#define TX_SG DCRN_ASG0
#define RX_SG DCRN_ASG1

#define TX_SG_ENABLE  	DCRN_DMA_SGC_SSG0
#define TX_SG_MASK 	DCRN_DMA_SGC_EM0

#define RX_SG_ENABLE  	DCRN_DMA_SGC_SSG1
#define RX_SG_MASK 	DCRN_DMA_SGC_EM1

/*****************************************************************************/

#undef OSS_DOCUMENTED_MIXER_SEMANTICS
#define DBG(x) {}
/*#define DBG(x) {x}*/

#define IBMCSI3K_MAGIC 0xB31BCB /* Copied from the Austin Research version */
/* TODO: verify this value is legitimate */

#define DMABUF_DEFAULTORDER (16-PAGE_SHIFT) /* TODO: check out this value */
#define DMABUF_MINORDER 1			/* TODO: ditto */

#define IBMCSI_WRITE(reg, val) (__raw_writel(val, reg))
#define IBMCSI_READ(reg)	(__raw_readl(reg))

/* Enable the following if using consistent_alloc() for DMA buffer */
/* #define USECONSISTENTALLOC */

/* Enable the following to debug FSYNC timing. This will toggle touch panel sense lines. */
/* #define TIMINGDEBUGPROBE 	*/

#define VALIDATE_STATE(s)                         \
({                                                \
	if (!(s) || (s)->magic != IBMCSI3K_MAGIC) { \
		printk(KERN_ERR "ibmcsi3k: invalid signature (magic) in private data\n");  \
		return -ENXIO;                    \
	}                                         \
})


/*****************************************************************************/
/* Static variables, globals and structs 				     */
/*****************************************************************************/

static const unsigned sample_shift[] = { 0, 1, 1, 2 };
static LIST_HEAD(devs);

/* -------------------------------------------------------------------- */
/* Private data structure for the devices supported by this driver 	*/
/* -------------------------------------------------------------------- */

struct ibmcsi3k_state {
	unsigned int magic;	/* Magic signature value for sanity check */

   	unsigned int state;	/* Driver state (DAC/ADC running, Halt, etc.)	*/


	/* DSP device variables */
   	struct list_head devs; 	/* For multi-device support; not used for the 405LP */
	int dev_dsp;          	/* Return value from register_sound_dsp	*/
   				/* We are a DSP device because we do not support mu-law encoding */
	spinlock_t lock;

	struct semaphore dsp_sem;
	struct semaphore open_sem;

	mode_t open_mode;
	wait_queue_head_t open_wait;

	/* Mixer device variables */
	/* Generic */
	int dev_mixer;          /* Return value from register_sound_mixer */
	struct semaphore mix_sem;


	unsigned int requested_sample_rate; /* TODO: variable sampling rate to be implemented */
	unsigned int actual_sample_rate;

   	unsigned int modcnt;

	/* We hardcode the hardware resource assignments in the defines:	 	*/
    	/* DMA channels, DMA IRQs, CSI address, CSI IRQ.				*/
    	/* If this driver is ported to a CPU that supports dynamic assignment  		*/
	/* these would go here.								*/

   	/* Buffers */
   	unsigned char *write_line;	/* Pointer to a small buffer for register writes */
	dma_addr_t dma_write_line;
	unsigned char *read_line;	/* Pointer to a small buffer for register reads */
	dma_addr_t dma_read_line;

	unsigned int timeout; 		/* Timeout flag for Si3000 register access 	*/

	struct dmabuf {	/* Control blocks for audio playback (dac) and capture (adc) 	*/
		/* The important ones... */
	      	void *rawbuf;		/* DMA buffer logical address 			*/
		dma_addr_t dmaaddr;	/* DMA buffer physical address 			*/
		unsigned hwptr, swptr;	/* Offsets from rawbuf for data. HWPTR = DMAC, SWPTR = driver */
		int count;

		wait_queue_head_t wait;

		/* And the rest, inherited from sample drivers... */
		unsigned fragsize;
		unsigned dmasize;
		unsigned fragsamples;

	      	unsigned buforder;
		unsigned numfrag;
		unsigned fragshift;

		/* OSS stuff */
		unsigned mapped:1;
		unsigned ready:1;
		unsigned endcleared:1;
		unsigned enabled:1;
		unsigned ossfragshift;
		int ossmaxfrags;
		unsigned subdivision;

      		unsigned total_bytes;
		unsigned error;                	/* Over/underrun */
#if defined(USE_SG)
		unsigned sg_count;
#endif

	} dma_dac, dma_adc;

   	unsigned int sync_interval ;	/* FSYNC synchronization interval */

#if defined(USE_SG)
        struct timer_list dac_timer;
	struct timer_list adc_timer;
	struct dma_sgdt *dac_free_sgdt_q;
	struct dma_sgdt *adc_free_sgdt_q;
	struct dma_sgdt *dac_active_sgdt_q;
	struct dma_sgdt *adc_active_sgdt_q;

	struct dma_sgdt *dac_sgdt_lastV; /* Anchors */
	struct dma_sgdt *adc_sgdt_lastV;

	struct dma_sgdt *adc_prev_sgdt;
#endif

};

/* Driver state flags */
#define IBMCSI_DEFAULT_STATE 	0x00000000
#define IBMCSI_DAC_RUNNING 	0x00010000
#define IBMCSI_ADC_RUNNING	0x00020000
#define IBMCSI_HALT		0x00040000


/*****************************************************************************/
/* Function Prototypes							     */
/*									     */
/* Quote: "Warning: function declaration is not prototype" (gcc 2.95.3)      */
/*****************************************************************************/


/*****************************************************************************/
/* Driver function prototypes */
/*****************************************************************************/

static int __init ibmcsi3k_setup(char *str);	/* Kernel driver setup */
static int __init init_ibmcsi3k(void);			/* Driver initialization */
static void __exit cleanup_ibmcsi3k(void); /* Driver exit cleanup */

/*****************************************************************************/
/* DSP driver function prototypes					*/
/*****************************************************************************/

/* Top level */
static int ibmcsi3k_open(struct inode *inode, struct file *file);
static ssize_t ibmcsi3k_read(struct file *file, char *buffer, size_t count, loff_t *ppos);
static ssize_t ibmcsi3k_write(struct file *file, const char *buffer, size_t count, loff_t *ppos);
static unsigned int ibmcsi3k_poll(struct file *file, struct poll_table_struct *wait);
static int ibmcsi3k_release(struct inode *inode, struct file *file);
static int ibmcsi3k_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

/* Interrupt handlers */
static void ibmcsi3k_dac_interrupt(int irq, void *dev_id, struct pt_regs *regs);
static void ibmcsi3k_adc_interrupt(int irq, void *dev_id, struct pt_regs *regs);
static void ibmcsi_adc_timer(unsigned long param);
static void ibmcsi_dac_timer(unsigned long param);

/* Utility routines */
static unsigned long copy_reformat_to_user(char *pDest, const char *pSrc, size_t count);
static unsigned long copy_reformat_from_user(char *pDest, const char *pSrc, size_t count);

static void start_adc(struct ibmcsi3k_state *s);
static void start_dac(struct ibmcsi3k_state *s);
static int drain_dac(struct ibmcsi3k_state *s, int nonblock);
static inline void stop_adc(struct ibmcsi3k_state *s);
static inline void stop_dac(struct ibmcsi3k_state *s);

static inline void dealloc_dmabuf(struct ibmcsi3k_state *s, struct dmabuf *buf);
static int prog_dmabuf(struct ibmcsi3k_state *s, struct dmabuf *buf, unsigned rate, unsigned fmt);
static inline int prog_dmabuf_adc(struct ibmcsi3k_state *s);
static inline int prog_dmabuf_dac(struct ibmcsi3k_state *s);

static inline unsigned get_hwptr(struct ibmcsi3k_state *s, struct dmabuf *buf, unsigned channel);
static void ibmcsi3k_update_ptr(struct ibmcsi3k_state *s);

static inline void clear_advance(void *buf, unsigned bsize, unsigned bptr, unsigned len, unsigned char c);

static inline unsigned ld2(unsigned int x);

/* Scatter/Gather descriptor table maintenance routines */

static void init_sgdt_q(struct dma_sgdt *queue, int count);
static struct dma_sgdt *get_sgdt(struct dma_sgdt **queueaddress);
static void free_sgdt(struct dma_sgdt **queueaddress, struct dma_sgdt *dt);
static unsigned int check_sgdt_range(struct ibmcsi3k_state *s, struct dma_sgdt* dt, int count);

/*****************************************************************************/
/* Mixer driver function prototypes					*/
/*****************************************************************************/

/* Top level */
static int ibmcsi3k_open_mixdev(struct inode *inode, struct file *file);
static int ibmcsi3k_ioctl_mixdev(struct inode *ioctl, struct file *file, unsigned int cmd, unsigned long arg);
static int ibmcsi3k_release_mixdev(struct inode *inode, struct file *file);

static int mixer_read_ioctl(struct ibmcsi3k_state *s, unsigned int nr, caddr_t arg);
static int mixer_write_ioctl(struct ibmcsi3k_state *s, unsigned int nr, caddr_t arg);

/* Mixer read routines */
static int ibmcsi3k_get_volume(struct ibmcsi3k_state *s) ;
static int ibmcsi3k_get_line(struct ibmcsi3k_state *s) ;
static int ibmcsi3k_get_mic(struct ibmcsi3k_state *s) ;
static int ibmcsi3k_get_speaker(struct ibmcsi3k_state *s) ;
static int ibmcsi3k_get_txpga(struct ibmcsi3k_state *s) ;
static int ibmcsi3k_get_rxpga(struct ibmcsi3k_state *s) ;
static int ibmcsi3k_get_recsrc(struct ibmcsi3k_state *s) ;
static int ibmcsi3k_get_outsrc(struct ibmcsi3k_state *s) ;

/* Mixer write routines */
static int ibmcsi3k_set_volume(struct ibmcsi3k_state *s, int val);
static int ibmcsi3k_set_line(struct ibmcsi3k_state *s, int val) ;
static int ibmcsi3k_set_mic(struct ibmcsi3k_state *s, int val) ;
static int ibmcsi3k_set_speaker(struct ibmcsi3k_state *s, int val) ;
static int ibmcsi3k_set_txpga(struct ibmcsi3k_state *s, int val) ;
static int ibmcsi3k_set_rxpga(struct ibmcsi3k_state *s, int val) ;
static int ibmcsi3k_set_recsrc(struct ibmcsi3k_state *s, int val) ;
static int ibmcsi3k_set_outsrc(struct ibmcsi3k_state *s, int val) ;

/* Si3000 control register access routines */
static unsigned char si3000_read_reg( struct ibmcsi3k_state *s, int reg );
static unsigned char si3000_read_reg2( struct ibmcsi3k_state *s, int reg, int retrycount);

static void si3000_write_reg2( struct ibmcsi3k_state *s, int reg, unsigned char val, int retrycount) ;
static void si3000_write_reg( struct ibmcsi3k_state *s, int reg, unsigned char val) ;

static unsigned char si3000_transfer( struct ibmcsi3k_state *s,
	int reg, int read , unsigned char val , unsigned int sync_value);


static int si3000_set_sampling_rate(struct ibmcsi3k_state *s, int val);
static int si3000_get_sampling_rate(struct ibmcsi3k_state *s);

static void ibmcsi3k_synchronize(struct ibmcsi3k_state *s);

/*****************************************************************************/

/* maximum number of devices; only used for command line params */
#define NR_DEVICE 1

static int micbias[NR_DEVICE] = { 0, };

static unsigned int devindex = 0;

MODULE_PARM(micbias, "1-" __MODULE_STRING(NR_DEVICE) "i");
MODULE_PARM_DESC(micbias, "sets the +5V bias for an electret microphone");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Benjamin Li, Bishop Brock, Ken Inoue");
MODULE_DESCRIPTION("IBM PPC 405LP CSI / SI Si3000 Audio Driver");


/*****************************************************************************/
/*****************************************************************************/
/* 			Code Section					     */
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/* Module Initialization						     */ 
/*****************************************************************************/

#ifndef MODULE

/* Currently no module parameters are supported. The following is left as template. */

static int __init ibmcsi3k_setup(char *str)
{
	static unsigned __initdata nr_dev = 0;

	if (nr_dev >= NR_DEVICE)
		return 0;

	(void)
	(
   	 get_option(&str,&micbias [nr_dev])
	);

	nr_dev++;
	return 1;
}

__setup("ibmcsi3k=", ibmcsi3k_setup);

#endif /* MODULE */

/*****************************************************************************/
/* 		Initialization tables					     */
/*****************************************************************************/

static /*const*/ struct file_operations ibmcsi3k_audio_fops = {
	owner:		THIS_MODULE,
	llseek:		no_llseek,
	read:		ibmcsi3k_read,
	write:		ibmcsi3k_write,
	poll:		ibmcsi3k_poll,
	ioctl:		ibmcsi3k_ioctl,
	mmap:		NULL,
	open:		ibmcsi3k_open,
	release:	ibmcsi3k_release,
};

static /*const*/ struct file_operations ibmcsi3k_mixer_fops = {
	owner:		THIS_MODULE,
	llseek:		no_llseek,
	ioctl:		ibmcsi3k_ioctl_mixdev,
	open:		ibmcsi3k_open_mixdev,
	release:	ibmcsi3k_release_mixdev,
};


typedef int (*PIF) (struct ibmcsi3k_state *, int);

static struct inittable {
	PIF proc;
	int val;
} inittable[] __initdata = {
	{ ibmcsi3k_set_recsrc, SOUND_MASK_LINE|SOUND_MASK_MIC } ,
	{ ibmcsi3k_set_volume, 	100 },
	{ ibmcsi3k_set_line,	100 },
	{ ibmcsi3k_set_speaker, 100 },
	{ ibmcsi3k_set_mic,	100 },
	{ ibmcsi3k_set_rxpga,	100 },
	{ ibmcsi3k_set_txpga,	 75 }
};

/*****************************************************************************/
/*	Driver Initialization 						     */
/*****************************************************************************/
static int __init init_ibmcsi3k(void)
{
	struct ibmcsi3k_state *s;
	mm_segment_t fs;
	int i, ret;
#if 0
	unsigned long end_time;
#endif

	static unsigned long si3000_lines_virt = 0;
	static dma_addr_t si3000_lines = (dma_addr_t)NULL;

	printk(KERN_INFO "ibmcsi3k: version v0.01 compiled at " __TIME__ " " __DATE__ "\n");

#ifdef CONFIG_PM
	/* ---- Power up the CSI core during initialization ---- */
	ppc4xx_cpm_fr(CPM_CSI, 0);
#endif

	/* ---- Allocate line buffers ---- */
	if (!(si3000_lines_virt = (unsigned long)consistent_alloc(GFP_ATOMIC | GFP_DMA, PAGE_SIZE, &(si3000_lines) ))) {
		printk(KERN_ERR "ibmcsi3k: out of memory\n");
		return -ENOMEM;
	}
	mem_map_reserve(virt_to_page(__va(si3000_lines)));

	/* ---- Allocate and initialize private data ---- */
        if (!(s = kmalloc(sizeof(struct ibmcsi3k_state), GFP_KERNEL))) {
		printk(KERN_WARNING "ibmcsi3k: out of memory\n");
		return -ENOMEM;
	}
	memset(s, 0, sizeof(struct ibmcsi3k_state));

	init_waitqueue_head(&s->dma_adc.wait);
	init_waitqueue_head(&s->dma_dac.wait);
	init_waitqueue_head(&s->open_wait);
	init_MUTEX(&s->open_sem);
	spin_lock_init(&s->lock);
	s->magic = IBMCSI3K_MAGIC;

	s->write_line = (void *)si3000_lines_virt;
	s->dma_write_line = si3000_lines;
	s->read_line = (void *)si3000_lines_virt + 128;
	s->dma_read_line = si3000_lines + 128;

	if (!request_region(CSI0_IO_BASE, CSI0_EXTENT, "ibmcsi3k")) {
		printk(KERN_ERR "ibmcsi3k: I/O ports %x-%x in use\n", CSI0_IO_BASE, CSI0_EXTENT-1);
		ret = -EBUSY;
		goto err_region;
	}

#ifdef TIMINGDEBUGPROBE	/* Debug probe */
	request_region(0xef600a00, 16, "ibmcsi3k");
#endif

	if ( ( IBMCSI_READ(CSI0_ID) >> 16 ) != CSI_ID_405LP) {
		printk(KERN_WARNING "ibmcsi3k: Unexpected CSI ID %x\n", IBMCSI_READ(CSI0_ID));
      	goto err_irq1;
	}
	else {
		printk(KERN_INFO "ibmcsi3k: Found chip \n");
	}

	/* Interrupts - currently held indefinitely since no external device will be using them */
	/* on the 405LP evaluation board. */ 
 	/* Mixer and DSP use the same interrupts. */

   	if ((ret=request_irq(IBMCSI_TXDMA_IRQ, ibmcsi3k_dac_interrupt, SA_SHIRQ, "ibmcsi3k",s))) {
		printk(KERN_ERR "ibmcsi3k: IRQ %d in use\n", IBMCSI_TXDMA_IRQ);
		goto err_irq1;
	}
   	if ((ret=request_irq(IBMCSI_RXDMA_IRQ, ibmcsi3k_adc_interrupt, SA_SHIRQ, "ibmcsi3k",s))) {
		printk(KERN_ERR "ibmcsi3k: IRQ %d in use\n",IBMCSI_RXDMA_IRQ);
		goto err_irq2;
	}

#if 0 /* TODO: CSI error interrupts not implemented */
   	if ((ret=request_irq(21, ibmcsi3k_csi_interrupt, SA_SHIRQ, "ibmcsi3k",s))) {
		printk(KERN_ERR "ibmcsi3k: irq 21 in use\n");
		goto err_irq;
	}
#endif

	/* register devices */
	if ((s->dev_dsp = register_sound_dsp(&ibmcsi3k_audio_fops, -1)) < 0) {
		ret = s->dev_dsp;
		goto err_dev1;
	}
	if ((s->dev_mixer = register_sound_mixer(&ibmcsi3k_mixer_fops, -1)) < 0) {
		ret = s->dev_mixer;
		goto err_dev2;
	}

       	fs = get_fs();
	set_fs(KERNEL_DS);

	/* Initialize the chips:  DMA controller, CSI and codec.				*/

     	/* Set the Si3000 PLL to 11.025 KHz - the values depend on master clock */
	/* TODO: sampling rate is hardcoded for now				*/

	if (!si3000_set_sampling_rate(s, IBMCSI_DEFAULT_SAMPLING_RATE)) {
		printk(KERN_ERR "ibmcsi3k: cannot set default sampling rate\n");
		goto err_dev3;
	}
	
	/* Initialize mixer settings (see inittable[] above for procedures and values.		*/
	for (i=0; i<sizeof(inittable)/sizeof(inittable[0]); i++) {
		inittable[i].proc(s, inittable[i].val);
	}

	/* Power on the line drivers (TODO: power management...)				*/
	/* Turn on speaker and line drivers as well as mic bias. Handset driver turned off.	*/
	/* (Mic bias bit is reverse polarity - 0 = ON) 						*/
	si3000_write_reg( s, SI3000_CR1, SI3000_CR1_SPD | SI3000_CR1_LPD );
	/* Note: 										*/
	/* We need a way to toggle the Si3000 RESET line if we want to use the power-down mode, */
	/* since a reset is the only way to wake up the Si3000 once SI3000_CR1_CPD is toggled.	*/
	/* Turning off the drivers yield minimal power saving, so we will leave them on.	*/ 
 
	set_fs(fs);

	/* Add to driver list */
	list_add_tail(&s->devs, &devs);
	/* Increment device index */
	if (devindex < NR_DEVICE-1)
		devindex++;

#ifdef CONFIG_PM
	/* ---- Power down the CSI core until an open() request ---- */
	/*ppc4xx_cpm_fr(CPM_CSI, 1); Bug?*/
#endif

	return 0;

 err_dev3:		/* Error exit initializing chips 	*/
	unregister_sound_mixer(s->dev_mixer);
 err_dev2:		/* Error exit registering mixer device	*/
	unregister_sound_dsp(s->dev_dsp);
 err_dev1:
	free_irq(5, s);
 err_irq2:
       	free_irq(6, s);
#if 0	/* TODO */
	free_irq(21,s);
#endif
 err_irq1:
	release_region(CSI0_IO_BASE, CSI0_EXTENT);
 err_region:
	kfree(s);
#ifdef CONFIG_PM
	/*ppc4xx_cpm_fr(CPM_CSI, 1); Bug?*/
#endif
	return ret;

}

/*****************************************************************************/
/*	Driver Cleanup							     */
/*****************************************************************************/
static void __exit cleanup_ibmcsi3k(void)
{
	printk(KERN_INFO "ibmcsi3k: unloading\n");
}

module_init(init_ibmcsi3k);
module_exit(cleanup_ibmcsi3k);


/*****************************************************************************/
/*****************************************************************************/
/*	Part I : DSP device /dev/dsp					     */
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/*	/dev/dsp Open							     */
/*****************************************************************************/
static int ibmcsi3k_open(struct inode *inode, struct file *file)
{
	int minor = MINOR(inode->i_rdev);
	DECLARE_WAITQUEUE(wait, current);
	unsigned long flags;
	struct list_head *list;
	struct ibmcsi3k_state *s;

	for (list = devs.next; ; list = list->next) {
		if (list == &devs)
			return -ENODEV;
		s = list_entry(list, struct ibmcsi3k_state, devs);
		if (!((s->dev_dsp ^ minor) & ~0xf))
			break;
	}
       	VALIDATE_STATE(s);
	file->private_data = s;
	/* wait for device to become free */
	down(&s->open_sem);
	while (s->open_mode & (file->f_mode & (FMODE_READ | FMODE_WRITE))) {
		if (file->f_flags & O_NONBLOCK) {
			up(&s->open_sem);
			return -EBUSY;
		}
		add_wait_queue(&s->open_wait, &wait);
		__set_current_state(TASK_INTERRUPTIBLE);
		up(&s->open_sem);
		schedule();
		remove_wait_queue(&s->open_wait, &wait);
		set_current_state(TASK_RUNNING);
		if (signal_pending(current))
			return -ERESTARTSYS;
		down(&s->open_sem);
	}
	s->open_mode |= (file->f_mode & (FMODE_READ | FMODE_WRITE));
	spin_lock_irqsave(&s->lock, flags);

#ifdef CONFIG_PM
	/* ppc4xx_cpm_fr(CPM_CSI, 0); Bug? Power up the CSI core */
#endif

	/* TODO: sampling rate hardcoded */
#ifdef USE_SG /* Use Scatter/Gather DMA */
	{
	struct dma_sgdt *dacp, *adcp;

#define SGDT_COUNT ((PAGE_SIZE/2)/(sizeof(struct dma_sgdt)))
	DBG(printk("ibmcsi: %d SGDTs\n", (int)SGDT_COUNT));

	s->dac_free_sgdt_q = (struct dma_sgdt *)(s->write_line);
	s->adc_free_sgdt_q = (struct dma_sgdt *)(s->write_line) + SGDT_COUNT;

 	init_sgdt_q(s->dac_free_sgdt_q, SGDT_COUNT);
 	init_sgdt_q(s->adc_free_sgdt_q, SGDT_COUNT);

	/* Prepare default (anchor) descriptor for DAC */
	dacp = s->dac_active_sgdt_q = get_sgdt(&(s->dac_free_sgdt_q));

  	dacp->ccw = (unsigned int) (IBMCSI_TXDMA_GO_NOI) ;

	dacp->nextP = (void *) virt_to_phys(dacp); /* Loop in on itself */

	dacp->srcP = (void *) virt_to_phys(&(dacp->dummy)); /* Point to dummy data */
	dacp->destP = (void *)CSI0_TBUF;
	dacp->dummy = 0;	/* Dummy neutral data (FIXME: currently assumes signed 16 format) */
	dacp->ctrl = 0x80000001; /* Link, count = 1 */

	s->dac_sgdt_lastV = dacp;

	adcp = s->adc_active_sgdt_q = get_sgdt(&(s->adc_free_sgdt_q));
	adcp->ccw = (unsigned int) (IBMCSI_RXDMA_GO_NOI) ;
	adcp->nextP = (void *) virt_to_phys(adcp);
	adcp->ctrl = 0x80000001;

	adcp->srcP = (void *)CSI0_RBUF;
	adcp->destP = (void *)virt_to_phys(&(adcp->dummy));

	s->adc_sgdt_lastV = adcp;

	DBG(printk("ibmcsi: s = %8.8x free_sgdt_q = %8.8x at %8.8x lastV = %8.8x at %8.8x\n", s, s->dac_free_sgdt_q ,&s->dac_free_sgdt_q,s->dac_sgdt_lastV,&s->dac_sgdt_lastV));

	} /* TODO: split the above into read and write */
#endif /* USE_SG */


	if (file->f_mode & FMODE_READ) {
		s->dma_adc.ossfragshift = s->dma_adc.ossmaxfrags = s->dma_adc.subdivision = 0;
		s->dma_adc.enabled = 1;
		/* TODO: data format hardcoded */

		IBMCSI_WRITE(CSI0_ER, 0);	/* Disable CSI */
		IBMCSI_WRITE(CSI0_SR, CSI_SR_TOD | CSI_SR_ROD); 	/* Clear CSI errors */
		mtdcr(IBMCSI_TXDMA_CR, 0);	/* Disable DMA Tx */
		mtdcr(IBMCSI_RXDMA_CR, 0); 	/* Disable DMA Rx */
		mtdcr(IBMCSI_DMA_SR, DCRN_DMA_SR_ALL(IBMCSI_TXDMA) | DCRN_DMA_SR_ALL(IBMCSI_RXDMA) ) ;
				 		/* Clear DMA status bits */
	}

   	if (file->f_mode & FMODE_WRITE) {
		s->dma_dac.ossfragshift = s->dma_dac.ossmaxfrags = s->dma_dac.subdivision = 0;
		s->dma_dac.enabled = 1;

      /* TODO: data format hardcoded */

		s->dma_dac.hwptr = 0;
		s->dma_dac.swptr = 0;
		s->dma_dac.count = 0;
#ifdef USE_SG
		s->dma_dac.sg_count = 0;
		init_timer(&s->dac_timer);
		s->dac_timer.function = ibmcsi_dac_timer;
		s->dac_timer.data = (unsigned long)s;
		s->dac_timer.expires = jiffies + DAC_TIMER_PERIOD;
		add_timer(&s->dac_timer);
#endif
	}

   	spin_unlock_irqrestore(&s->lock, flags);
	up(&s->open_sem);
	init_MUTEX(&s->dsp_sem);
	return 0;
}

/*****************************************************************************/
/* 	/dev/dsp Read							     */
/*****************************************************************************/
static ssize_t ibmcsi3k_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	struct ibmcsi3k_state *s = (struct ibmcsi3k_state *)file->private_data;
	DECLARE_WAITQUEUE(wait, current);
	ssize_t ret = 0;
	unsigned long flags;
	unsigned swptr;
	int cnt;

	VALIDATE_STATE(s);
	if (ppos != &file->f_pos)
		return -ESPIPE;
	if (s->dma_adc.mapped)
		return -ENXIO;
	if (!access_ok(VERIFY_WRITE, buffer, count))
		return -EFAULT;
	down(&s->dsp_sem);
	if (!s->dma_adc.ready && (ret = prog_dmabuf_adc(s)))
		goto out;

	add_wait_queue(&s->dma_adc.wait, &wait);
	while (count > 0) {
		/* Spinlock on */
		spin_lock_irqsave(&s->lock, flags);

		swptr = s->dma_adc.swptr;
		if (s->dma_adc.hwptr >= s->dma_adc.swptr) {
			cnt = s->dma_adc.hwptr - swptr;
		}
		else { /* HWPTR wrapped */
			cnt = s->dma_adc.dmasize - swptr;
		}

		if (s->dma_adc.count < cnt)
			cnt = s->dma_adc.count;
		if (cnt <= 0)
			__set_current_state(TASK_INTERRUPTIBLE);

		spin_unlock_irqrestore(&s->lock, flags);
		/* Spinlock off */

		if (cnt > count*2 ) /* cnt is raw (4 bytes per sample), count is cooked (2 bytes per sample) */
			cnt = count * 2;

		if (cnt <= 0) {
			if (s->dma_adc.enabled)
				start_adc(s);
			if (file->f_flags & O_NONBLOCK) {
				if (!ret)
					ret = -EAGAIN;
				goto out;
			}
			up(&s->dsp_sem);
			schedule();
			if (signal_pending(current)) {
				if (!ret)
					ret = -ERESTARTSYS;
				goto out;
			}
			down(&s->dsp_sem);
			if (s->dma_adc.mapped)
			{
				ret = -ENXIO;
				goto out;
			}
			continue;
		}

	      	if (copy_reformat_to_user(buffer, s->dma_adc.rawbuf + swptr, cnt/4)) {
			if (!ret)
				ret = -EFAULT;
			goto out;
		}
		swptr = (swptr + cnt) % s->dma_adc.dmasize;
		spin_lock_irqsave(&s->lock, flags);
		s->dma_adc.swptr = swptr;
		s->dma_adc.count -= cnt;
		spin_unlock_irqrestore(&s->lock, flags);
		count -= cnt/2;
		buffer += cnt/2;
		ret += cnt/2;
		if (s->dma_adc.enabled)
			start_adc(s);
	}
out:
	up(&s->dsp_sem);
        remove_wait_queue(&s->dma_adc.wait, &wait);
	set_current_state(TASK_RUNNING);
	return ret;
}


/*****************************************************************************/
/*	/dev/dsp Write							     */
/*****************************************************************************/
static ssize_t ibmcsi3k_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	struct ibmcsi3k_state *s = (struct ibmcsi3k_state *)file->private_data;
	DECLARE_WAITQUEUE(wait, current);
	ssize_t ret = 0;
	unsigned long flags;
	unsigned swptr;
	int cnt;

	VALIDATE_STATE(s);
	if (ppos != &file->f_pos)
		return -ESPIPE;
	if (s->dma_dac.mapped)
		return -ENXIO;

	if (!access_ok(VERIFY_READ, buffer, count)) {
		return -EFAULT;
	}
	down(&s->dsp_sem);
	if (!s->dma_dac.ready && (ret = prog_dmabuf_dac(s))) {
		printk("DEBUG: ibmcsi3k_write dac ready %d prog_dmabuf retcode %d \n", s->dma_dac.ready, ret);
		goto out;
	}
	ret = 0;
        add_wait_queue(&s->dma_dac.wait, &wait);
	while (count > 0) {
		spin_lock_irqsave(&s->lock, flags);
		if (s->dma_dac.count < 0) {
			s->dma_dac.count = 0;
			s->dma_dac.swptr = s->dma_dac.hwptr;
		}
		swptr = s->dma_dac.swptr;
		cnt = s->dma_dac.dmasize-swptr;				/* cnt = available buffer */
		if (s->dma_dac.count + cnt > s->dma_dac.dmasize) 	/* Truncate to fit in dma buffer */
			cnt = s->dma_dac.dmasize - s->dma_dac.count;

		if (cnt <= 0)
			__set_current_state(TASK_INTERRUPTIBLE);
		spin_unlock_irqrestore(&s->lock, flags);

      /* TODO: format assumed 16 bit for now */
		if (cnt > (count * 2))
      	 	cnt = count * 2;

		if (cnt <= 0) {
			if (s->dma_dac.enabled)
				start_dac(s);
			if (file->f_flags & O_NONBLOCK) {
				if (!ret)
					ret = -EAGAIN;
				goto out;
			}
			up(&s->dsp_sem);
			schedule();
			if (signal_pending(current)) {
				if (!ret)
					ret = -ERESTARTSYS;
				goto out;
			}
			down(&s->dsp_sem);
			if (s->dma_dac.mapped)
			{
			ret = -ENXIO;
			goto out;
			}
			continue;
		}

		if (copy_reformat_from_user(s->dma_dac.rawbuf + swptr, buffer, cnt/4)) { /* cnt/4 = # of samples */
			printk("DEBUG: copy_reformat_from_user failed\n");
			if (!ret)
				ret = -EFAULT;
			goto out;
		}

#ifndef USECONSISTENTALLOC
		dma_cache_wback_inv((unsigned long)(s->dma_dac.rawbuf + swptr), cnt);
#endif

		swptr = (swptr + cnt) % s->dma_dac.dmasize; /* Wrap */
		spin_lock_irqsave(&s->lock, flags);
		s->dma_dac.swptr = swptr;
		s->dma_dac.count += cnt;
		s->dma_dac.endcleared = 0;		/* Remember to zero clear one fragment at the end */

#ifdef USE_SG /* Scatter/Gather mode */
		s->dma_dac.sg_count += cnt;
#endif

		spin_unlock_irqrestore(&s->lock, flags);
/* TODO: broken if odd byte written. Also, format is assumed 16 bit */
		count -= cnt / 2;
		buffer += cnt / 2;
		ret += cnt / 2;

          if (s->dma_dac.enabled)
			start_dac(s);

	}
out:
	up(&s->dsp_sem);
        remove_wait_queue(&s->dma_dac.wait, &wait);
	set_current_state(TASK_RUNNING);
	return ret;
}

/*****************************************************************************/
/*	/dev/dsp Poll							     */
/*****************************************************************************/
static unsigned int ibmcsi3k_poll(struct file *file, struct poll_table_struct *wait)
{
	struct ibmcsi3k_state *s = (struct ibmcsi3k_state *)file->private_data;
	unsigned long flags;
	unsigned int mask = 0;

	VALIDATE_STATE(s);
	if (file->f_mode & FMODE_WRITE) {
		if (!s->dma_dac.ready && prog_dmabuf_dac(s))
			return 0;
		poll_wait(file, &s->dma_dac.wait, wait);
	}
	if (file->f_mode & FMODE_READ) {
		if (!s->dma_adc.ready && prog_dmabuf_adc(s))
			return 0;
		poll_wait(file, &s->dma_adc.wait, wait);
	}
	spin_lock_irqsave(&s->lock, flags);
	ibmcsi3k_update_ptr(s);
	if (file->f_mode & FMODE_READ) {
		if (s->dma_adc.count >= (signed)s->dma_adc.fragsize)
			mask |= POLLIN | POLLRDNORM;
	}
	if (file->f_mode & FMODE_WRITE) {
		if (s->dma_dac.mapped) {
			if (s->dma_dac.count >= (signed)s->dma_dac.fragsize)
				mask |= POLLOUT | POLLWRNORM;
		}
		else {
			if ((signed)s->dma_dac.dmasize >= s->dma_dac.count + (signed)s->dma_dac.fragsize)
				mask |= POLLOUT | POLLWRNORM;
		}
	}
	spin_unlock_irqrestore(&s->lock, flags);
	return mask;
}

/*****************************************************************************/
/*	/dev/dsp IOCTL 							     */
/*****************************************************************************/
static int ibmcsi3k_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ibmcsi3k_state *s = (struct ibmcsi3k_state *)file->private_data;
	unsigned long flags;
        audio_buf_info abinfo;
        count_info cinfo;
	int count;
	int val, mapped, ret;

	VALIDATE_STATE(s);
        mapped = ((file->f_mode & FMODE_WRITE) && s->dma_dac.mapped) ||
		((file->f_mode & FMODE_READ) && s->dma_adc.mapped);
	switch (cmd) {
	case OSS_GETVERSION:
		return put_user(SOUND_VERSION, (int *)arg);

	case SNDCTL_DSP_SYNC:
		if (file->f_mode & FMODE_WRITE)
			return drain_dac(s, 0/*file->f_flags & O_NONBLOCK*/);
		return 0;

	case SNDCTL_DSP_SETDUPLEX:
		return 0;

	case SNDCTL_DSP_GETCAPS:
		return put_user(DSP_CAP_DUPLEX | DSP_CAP_REALTIME | DSP_CAP_TRIGGER | DSP_CAP_MMAP, (int *)arg);

        case SNDCTL_DSP_RESET:
		if (file->f_mode & FMODE_WRITE) {
			stop_dac(s);
			synchronize_irq();
			s->dma_dac.swptr = s->dma_dac.hwptr = s->dma_dac.count = s->dma_dac.total_bytes = 0;
		}
		if (file->f_mode & FMODE_READ) {
			stop_adc(s);
			synchronize_irq();
			s->dma_adc.swptr = s->dma_adc.hwptr = s->dma_adc.count = s->dma_adc.total_bytes = 0;
		}
		return 0;

        case SNDCTL_DSP_SPEED:
                if (get_user(val, (int *)arg))
			return -EFAULT;
      		if (val >= 0) {
			/* Check if we are open */
			if (s->state & (~file->f_mode) & (FMODE_READ|FMODE_WRITE))
				return -EINVAL;
			/* Si3000 supports sampling rates 4KHz thru 12KHz. */
        	  	if (val < 4000) val = 4000;
			if (val > 12000) val = 120000;

	         	stop_adc(s);
			stop_dac(s);
			s->dma_adc.ready = s->dma_dac.ready = 0;
			spin_lock_irqsave(&s->lock, flags);
         		/* TODO: Reprogram Si3000 PLL here, if you dare!! */
			spin_unlock_irqrestore(&s->lock, flags);
		}

		return put_user(IBMCSI_DEFAULT_SAMPLING_RATE, (int *)arg);	/* Hardcoded for now */

        case SNDCTL_DSP_STEREO:
        	return 0;

        case SNDCTL_DSP_CHANNELS:
		if (get_user(val, (int *)arg))
			return -EFAULT;
		return put_user(1, (int *)arg);

	case SNDCTL_DSP_GETFMTS: /* Returns a mask */
	  return put_user(AFMT_S16_BE, (int *)arg);

	case SNDCTL_DSP_SETFMT: /* Selects ONE fmt*/
		if (get_user(val, (int *)arg))
			return -EFAULT;
		return put_user(AFMT_S16_BE, (int *)arg);
	case SNDCTL_DSP_POST:
                return 0;

        case SNDCTL_DSP_GETTRIGGER:
		val = 0;
		if ((file->f_mode) & FMODE_READ && (s->state & IBMCSI_ADC_RUNNING))
			val |= PCM_ENABLE_INPUT;
		if ((file->f_mode & FMODE_WRITE) && (s->state & IBMCSI_DAC_RUNNING))
			val |= PCM_ENABLE_OUTPUT;
		return put_user(val, (int *)arg);

	case SNDCTL_DSP_SETTRIGGER:
		if (get_user(val, (int *)arg))
			return -EFAULT;
		if (file->f_mode & FMODE_READ) {
			if (val & PCM_ENABLE_INPUT) {
				if (!s->dma_adc.ready && (ret = prog_dmabuf_adc(s)))
					return ret;
				s->dma_adc.enabled = 1;
				start_adc(s);
			}
			else {
				s->dma_adc.enabled = 0;
				stop_adc(s);
			}
		}
		if (file->f_mode & FMODE_WRITE) {
			if (val & PCM_ENABLE_OUTPUT) {
				if (!s->dma_dac.ready && (ret = prog_dmabuf_dac(s)))
					return ret;
				s->dma_dac.enabled = 1;
				start_dac(s);
			}
			else {
				s->dma_dac.enabled = 0;
				stop_dac(s);
			}
		}
		return 0;

	case SNDCTL_DSP_GETOSPACE:
		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		if (!s->dma_dac.ready && (val = prog_dmabuf_dac(s)) != 0)
			return val;
		spin_lock_irqsave(&s->lock, flags);
		ibmcsi3k_update_ptr(s);
		abinfo.fragsize = s->dma_dac.fragsize;
		count = s->dma_dac.count;
		if (count < 0)
			count = 0;
                abinfo.bytes = s->dma_dac.dmasize - count;
                abinfo.fragstotal = s->dma_dac.numfrag;
                abinfo.fragments = abinfo.bytes >> s->dma_dac.fragshift;
		spin_unlock_irqrestore(&s->lock, flags);
		return copy_to_user((void *)arg, &abinfo, sizeof(abinfo)) ? -EFAULT : 0;

	case SNDCTL_DSP_GETISPACE:
		if (!(file->f_mode & FMODE_READ))
			return -EINVAL;
		if (!s->dma_adc.ready && (val = prog_dmabuf_adc(s)) != 0)
			return val;
		spin_lock_irqsave(&s->lock, flags);
		ibmcsi3k_update_ptr(s);
		abinfo.fragsize = s->dma_adc.fragsize;
		count = s->dma_adc.count;
		if (count < 0)
			count = 0;
                abinfo.bytes = count;
                abinfo.fragstotal = s->dma_adc.numfrag;
                abinfo.fragments = abinfo.bytes >> s->dma_adc.fragshift;
		spin_unlock_irqrestore(&s->lock, flags);
		return copy_to_user((void *)arg, &abinfo, sizeof(abinfo)) ? -EFAULT : 0;

        case SNDCTL_DSP_NONBLOCK:
                file->f_flags |= O_NONBLOCK;
                return 0;

        case SNDCTL_DSP_GETODELAY:
		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		if (!s->dma_dac.ready && (val = prog_dmabuf_dac(s)) != 0)
			return val;
		spin_lock_irqsave(&s->lock, flags);
		ibmcsi3k_update_ptr(s);
                count = s->dma_dac.count;
		spin_unlock_irqrestore(&s->lock, flags);
		if (count < 0)
			count = 0;
		return put_user(count, (int *)arg);

        case SNDCTL_DSP_GETIPTR:
		if (!(file->f_mode & FMODE_READ))
			return -EINVAL;
		if (!s->dma_adc.ready && (val = prog_dmabuf_adc(s)) != 0)
			return val;
		spin_lock_irqsave(&s->lock, flags);
		ibmcsi3k_update_ptr(s);
                cinfo.bytes = s->dma_adc.total_bytes;
		count = s->dma_adc.count;
		if (count < 0)
			count = 0;
                cinfo.blocks = count >> s->dma_adc.fragshift;
                cinfo.ptr = s->dma_adc.hwptr;
		if (s->dma_adc.mapped)
			s->dma_adc.count &= s->dma_adc.fragsize-1;
		spin_unlock_irqrestore(&s->lock, flags);
                return copy_to_user((void *)arg, &cinfo, sizeof(cinfo));

        case SNDCTL_DSP_GETOPTR:
		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		if (!s->dma_dac.ready && (val = prog_dmabuf_dac(s)) != 0)
			return val;
		spin_lock_irqsave(&s->lock, flags);
		ibmcsi3k_update_ptr(s);
                cinfo.bytes = s->dma_dac.total_bytes;
		count = s->dma_dac.count;
		if (count < 0)
			count = 0;
                cinfo.blocks = count >> s->dma_dac.fragshift;
                cinfo.ptr = s->dma_dac.hwptr;
		if (s->dma_dac.mapped)
			s->dma_dac.count &= s->dma_dac.fragsize-1;
		spin_unlock_irqrestore(&s->lock, flags);
                return copy_to_user((void *)arg, &cinfo, sizeof(cinfo));

        case SNDCTL_DSP_GETBLKSIZE:
		if (file->f_mode & FMODE_WRITE) {
			if ((val = prog_dmabuf_dac(s)))
				return val;
			return put_user(s->dma_dac.fragsize, (int *)arg);
		}
		if ((val = prog_dmabuf_adc(s)))
			return val;
		return put_user(s->dma_adc.fragsize, (int *)arg);

        case SNDCTL_DSP_SETFRAGMENT:
                if (get_user(val, (int *)arg))
			return -EFAULT;
		if (file->f_mode & FMODE_READ) {
			s->dma_adc.ossfragshift = val & 0xffff;
			s->dma_adc.ossmaxfrags = (val >> 16) & 0xffff;
			if (s->dma_adc.ossfragshift < 4)
				s->dma_adc.ossfragshift = 4;
			if (s->dma_adc.ossfragshift > 15)
				s->dma_adc.ossfragshift = 15;
			if (s->dma_adc.ossmaxfrags < 4)
				s->dma_adc.ossmaxfrags = 4;
		}
		if (file->f_mode & FMODE_WRITE) {
			s->dma_dac.ossfragshift = val & 0xffff;
			s->dma_dac.ossmaxfrags = (val >> 16) & 0xffff;
			if (s->dma_dac.ossfragshift < 4)
				s->dma_dac.ossfragshift = 4;
			if (s->dma_dac.ossfragshift > 15)
				s->dma_dac.ossfragshift = 15;
			if (s->dma_dac.ossmaxfrags < 4)
				s->dma_dac.ossmaxfrags = 4;
		}
		return 0;

        case SNDCTL_DSP_SUBDIVIDE:
		if ((file->f_mode & FMODE_READ && s->dma_adc.subdivision) ||
		    (file->f_mode & FMODE_WRITE && s->dma_dac.subdivision))
			return -EINVAL;
                if (get_user(val, (int *)arg))
			return -EFAULT;
		if (val != 1 && val != 2 && val != 4)
			return -EINVAL;
		if (file->f_mode & FMODE_READ)
			s->dma_adc.subdivision = val;
		if (file->f_mode & FMODE_WRITE)
			s->dma_dac.subdivision = val;
		return 0;

        case SOUND_PCM_READ_RATE:
		return put_user(IBMCSI_DEFAULT_SAMPLING_RATE, (int *)arg);

        case SOUND_PCM_READ_CHANNELS:
		return put_user(2, (int *)arg);

        case SOUND_PCM_READ_BITS:
		return put_user(16,(int *)arg);

        case SOUND_PCM_WRITE_FILTER:
        case SNDCTL_DSP_SETSYNCRO:
        case SOUND_PCM_READ_FILTER:
                return -EINVAL;

	}
	return -EINVAL;
}

/*****************************************************************************/
/* 	/dev/dsp MMAP (Not supported)					     */
/*****************************************************************************/
#if 0 /* No mmap support */
static int ibmcsi3k_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct ibmcsi3k_state *s = (struct ibmcsi3k_state *)file->private_data;
	struct dmabuf *db;
	int ret = 0;
	unsigned long size;

	VALIDATE_STATE(s);
   	printk("DEBUG: ibmcsi3k_mmap\n");
	lock_kernel();
	down(&s->dsp_sem);
	if (vma->vm_flags & VM_WRITE) {
		if ((ret = prog_dmabuf_dac(s)) != 0) {
			goto out;
		}
		db = &s->dma_dac;
	} else if (vma->vm_flags & VM_READ) {
		if ((ret = prog_dmabuf_adc(s)) != 0) {
			goto out;
		}
		db = &s->dma_adc;
	} else  {
		ret = -EINVAL;
		goto out;
	}
	if (vma->vm_pgoff != 0) {
		ret = -EINVAL;
		goto out;
	}
	size = vma->vm_end - vma->vm_start;
	if (size > (PAGE_SIZE << db->buforder)) {
		ret = -EINVAL;
		goto out;
	}
	if (remap_page_range(vma->vm_start, virt_to_phys(db->rawbuf), size, vma->vm_page_prot)) {
		ret = -EAGAIN;
		goto out;
	}
	db->mapped = 1;
out:
	up(&s->dsp_sem);
	unlock_kernel();
	return ret;
}
#endif


/*****************************************************************************/
/*	/dev/dsp Release						     */
/*****************************************************************************/
static int ibmcsi3k_release(struct inode *inode, struct file *file)
{
	struct ibmcsi3k_state *s = (struct ibmcsi3k_state *)file->private_data;

	VALIDATE_STATE(s);
	lock_kernel();
	if (file->f_mode & FMODE_WRITE)
		drain_dac(s, file->f_flags & O_NONBLOCK);
	down(&s->open_sem);
	if (file->f_mode & FMODE_WRITE) {
		stop_dac(s);
#ifdef USE_SG
		del_timer_sync(&s->dac_timer);
#endif
		synchronize_irq();
		dealloc_dmabuf(s, &s->dma_dac);
	}
	if (file->f_mode & FMODE_READ) {
		stop_adc(s);
#ifdef USE_SG /* ADC to be implemented */
        /*       	del_timer_sync(&s->adc_timer); */
#endif
		dealloc_dmabuf(s, &s->dma_adc);
	}
	s->open_mode &= ~(file->f_mode & (FMODE_READ|FMODE_WRITE));
#ifdef CONFIG_PM
	/* if (!s->open_mode)
	   ppc4xx_cpm_fr(CPM_CSI, 1); Bug? */
#endif
	wake_up(&s->open_wait);
	up(&s->open_sem);
	unlock_kernel();
	return 0;
}



/*****************************************************************************/
/*****************************************************************************/
/*	/dev/dsp suppport routines 					     */
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/* 	Start DAC (Digital to Analog Conversion - Audio Playback)	     */
/*****************************************************************************/
static void start_dac(struct ibmcsi3k_state *s)
{
#ifdef USE_SG /* Scatter/Gather mode DMA */
	unsigned long flags;
	unsigned fragremain;
   	struct dma_sgdt *dt;
	unsigned int count;

	DBG(printk("start_dac_sg state %8.8x count %d ready %d sgcnt %d hwptr %d\n",
		s->state, s->dma_dac.count, s->dma_dac.ready, s->dma_dac.sg_count, s->dma_dac.hwptr));

	spin_lock_irqsave(&s->lock, flags); /* Spinlock held from here to end */

	if (s->dma_dac.sg_count) { /* There is data that needs to be posted */

		if (s->dma_dac.hwptr >= s->dma_dac.dmasize) {/* Sanity check */
			s->dma_dac.hwptr = 0;
		}

		count = s->dma_dac.sg_count;
		if (s->dma_dac.hwptr + count > s->dma_dac.dmasize) {
			count = s->dma_dac.dmasize - s->dma_dac.hwptr;
		}

		if ( count > 65536*4 ) count = 65536*4 ;  /* 4 bytes per sample, DMA count limited to 64K */

		if ( count > 4 ) { /* Got enough data to do DMA */

			dt = get_sgdt(&(s->dac_free_sgdt_q)); /* Get descriptor */

   			if (dt) {	/* Descriptor available */
				DBG(printk("new dt %8.8x ",dt));

		 		/* TODO: Break data into fragments. Easiest way for now! */

				dt->ccw = (unsigned int) (IBMCSI_TXDMA_GO_NOI) ;
				dt->ctrl = 0x80000000 | ((count/4)&0xffff);
				dt->srcP = (void *) virt_to_phys(s->dma_dac.rawbuf + s->dma_dac.hwptr);
				dt->destP = (void *) CSI0_TBUF;
				dt->nextP = (struct dma_sgdt *) virt_to_phys(s->dac_sgdt_lastV);

				asm volatile ("sync");

				DBG(printk("TX SG %8.8x, last %8.8x\n", mfdcr(TX_SG),s->dac_sgdt_lastV));
				DBG(printk("TX SA %8.8x, count %d \n",
						mfdcr(IBMCSI_TXDMA_SA), mfdcr(IBMCSI_TXDMA_CT)));
				DBG(printk("last->phys %8.8x\n",s->dac_sgdt_lastV->srcP));

				if ( mfdcr(DCRN_ASGC) & TX_SG_ENABLE ) {
					unsigned int current_sa = mfdcr(IBMCSI_TXDMA_SA);
					if ( (struct dmag_sgdt *)current_sa == s->dac_sgdt_lastV->srcP ) {
						s->dac_active_sgdt_q = dt;
						stop_dac(s);
						udelay(100);
					}
					else if ( mfdcr(IBMCSI_TXDMA_CT) <=2 ) {
						int timeout=0;
						/* No time to safely reprogram this pass 	*/
						/* Wait until this pass completes. 		*/
						while (current_sa == mfdcr(IBMCSI_TXDMA_SA)) {
							timeout++;
							if (timeout > 1000000) {
								printk("ibmcsi: DMA timeout!!\n");
								break;
							}
						};

						if ((struct dma_sgdt *)mfdcr(IBMCSI_TXDMA_SA) ==
                                                        s->dac_sgdt_lastV->srcP) {
							stop_dac(s);
							udelay(100);
						}
					}
				}
				else {
					s->dac_active_sgdt_q = dt; /* This gets programmed only if not active */
				}

				/* Insert new dt between last anchor and previous data */

				dt->nextV = s->dac_sgdt_lastV;
				dt->prevV = s->dac_sgdt_lastV->prevV;

				if (dt->prevV) {
					dt->prevV->nextV = dt;
					dt->prevV->nextP = (void *)virt_to_phys(dt);
				}

				s->dac_sgdt_lastV->prevV = dt;

				if ( ( mfdcr(DCRN_ASGC) & TX_SG_ENABLE ) ) {
					if ( mfdcr(TX_SG) == virt_to_phys(s->dac_sgdt_lastV) ) {
						/* Next descriptor is the bottom anchor */

						mtdcr(TX_SG, virt_to_phys(dt));
						/* Replace the next descriptor address with myself */
						/* Not sure if this works */
					}
				}

				asm volatile ("sync");

				/* Update count and pointer */
				s->dma_dac.sg_count -= count;
				s->dma_dac.hwptr += count;
				if (s->dma_dac.hwptr >= s->dma_dac.dmasize) {
					s->dma_dac.hwptr = 0;
				}

			} /* End if descriptor available */
			else {
				DBG(printk("No dt!\n"));
			}

		      	/* If DMA is not already running, kick it. */

			if (!(mfdcr(DCRN_ASGC) & TX_SG_ENABLE ))  {
				DBG(printk("kick\n"));

	        		s->state |= IBMCSI_DAC_RUNNING;

				/* Clear Terminal Count etc. */
				mtdcr(IBMCSI_DMA_SR, DCRN_DMA_SR_ALL(IBMCSI_TXDMA));

				/* Clear CSI overrun / underrun */
				IBMCSI_WRITE(CSI0_SR, CSI_SR_ROD | CSI_SR_TOD);

		        	/* Write address of the first scatter/gather descriptor table */
				mtdcr(TX_SG, virt_to_phys(s->dac_active_sgdt_q));

				/* Enable scatter/gather */
				mtdcr(DCRN_ASGC, mfdcr(DCRN_ASGC) | TX_SG_ENABLE | TX_SG_MASK) ;

				/* Set up CSI config */
				IBMCSI_WRITE(CSI0_CFG, 0);

#if 0 /* DMATIMINGPROBE */
			__raw_writel(0x54a54000, 0xef600a00); /* Turn on touch panel X2 line */
#endif
				asm volatile ("sync"); 	/* Sync */

				/* Start CSI, enable slot 0 and Tx. */
				IBMCSI_WRITE(CSI0_ER, CSI_ER_ESLOT(0) |	CSI_ER_TXEN | CSI_ER_ECSI );

				udelay(100);
				DBG(printk("CSI SA %8.8x, CT %8.8x",mfdcr(IBMCSI_TXDMA_SA), mfdcr(IBMCSI_TXDMA_CT)));
			}
			else {
/*				DBG(printk("no kick \n")); */
			}

		} /* End count not zero */
	}

	spin_unlock_irqrestore(&s->lock, flags);

#else /* Normal mode */

        unsigned long flags;
	unsigned fragremain;

	spin_lock_irqsave(&s->lock, flags);

	if (!(s->state & IBMCSI_DAC_RUNNING) && (s->dma_dac.mapped || s->dma_dac.count > 0)
	    && s->dma_dac.ready) {
        	s->state |= IBMCSI_DAC_RUNNING;

		fragremain = s->dma_dac.count;

		if (fragremain > s->dma_dac.fragsize) fragremain = s->dma_dac.fragsize;

		/* Disable CSI */
		IBMCSI_WRITE(CSI0_ER, 0);
		IBMCSI_WRITE(CSI0_CFG, 0);

	      	/* Disable Tx DMA channel first before modifying other bits in CR */
		mtdcr(IBMCSI_TXDMA_CR, 0);

		/* Clear Terminal Count etc. */
		mtdcr(IBMCSI_DMA_SR, DCRN_DMA_SR_ALL(IBMCSI_TXDMA));

		/* Clear CSI overrun / underrun */
		IBMCSI_WRITE(CSI0_SR, CSI_SR_ROD | CSI_SR_TOD);

      		/* Prepare Tx DMA channel */
		mtdcr(IBMCSI_TXDMA_CR, IBMCSI_TXDMA_CONFIG);

	      	/* Set Tx DMA channel source address */
		mtdcr(IBMCSI_TXDMA_SA, (unsigned)(s->dma_dac.dmaaddr) + (unsigned)(s->dma_dac.hwptr) );

		/* Set Tx DMA channel count	    */
		mtdcr(IBMCSI_TXDMA_CT, fragremain / 2);

		/* Set Tx DMA channel destination address */
		mtdcr(IBMCSI_TXDMA_DA, CSI0_TBUF);

		/* Reconfigure and enable Tx DMA channel */
		mtdcr(IBMCSI_TXDMA_CR, IBMCSI_TXDMA_GO);

		asm volatile ("sync"); 	/* Sync */

		/* Start CSI, enable slot 0 and Tx. */
		IBMCSI_WRITE(CSI0_ER, CSI_ER_ESLOT(0) |	CSI_ER_TXEN | CSI_ER_ECSI );
	}

	spin_unlock_irqrestore(&s->lock, flags);
#endif /* End normal mode DMA */
}

/*****************************************************************************/
/* 	Start ADC (Analog to Digital Conversion - Audio Capture)	     */
/*****************************************************************************/
static void start_adc(struct ibmcsi3k_state *s)
{
	unsigned long flags;
	unsigned fragremain;

	spin_lock_irqsave(&s->lock, flags);

	if (!(s->state & IBMCSI_ADC_RUNNING) &&
   		(s->dma_adc.mapped ||
         	 s->dma_adc.count < (signed) (s->dma_adc.dmasize - 4*s->dma_adc.fragsize)) 
		/* Multiply by 4 since we do 32 bit transfer per count */
	    	&& s->dma_adc.ready) {
        	s->state |= IBMCSI_ADC_RUNNING;

		fragremain = (s->dma_adc.dmasize - s->dma_adc.count)/4;

		if (fragremain > s->dma_adc.fragsize) fragremain = s->dma_adc.fragsize;

		/* Disable CSI, then disable Rx DMA channel (must be in this order.) */
		IBMCSI_WRITE(CSI0_ER, 0);
		mtdcr(IBMCSI_RXDMA_CR, 0);

		/* Clear Rx channel status flags (Terminal Count etc.) if any */
		mtdcr(IBMCSI_DMA_SR, DCRN_DMA_SR_ALL(IBMCSI_RXDMA));

		/* Clear CSI overrun / underrun errors if any */
		IBMCSI_WRITE(CSI0_SR, CSI_SR_ROD | CSI_SR_TOD);


		/* Reconfigure Rx DMA channel	*/
	      	mtdcr(IBMCSI_RXDMA_CR, IBMCSI_RXDMA_CONFIG);

      		/* Set Rx DMA channel DESTINATION address (to memory) */
		mtdcr(IBMCSI_RXDMA_DA, (unsigned)(s->dma_adc.dmaaddr) + (unsigned)(s->dma_adc.hwptr) );

		/* Set Rx DMA channel transfer count	    */
		mtdcr(IBMCSI_RXDMA_CT, fragremain);

		/* Set Rx DMA channel SOURCE address (from CSI Rx buffer) */
		mtdcr(IBMCSI_RXDMA_SA, CSI0_RBUF);

		/* Enable Rx DMA channel */
		mtdcr(IBMCSI_RXDMA_CR, IBMCSI_RXDMA_GO);

		asm volatile ("sync"); 	/* Sync all mtdcr instructions and MMIO accesses */

		/* Configure CSI for 256 bits per frame	*/
	      	IBMCSI_WRITE(CSI0_CFG, 0);

		/* Start CSI */
		IBMCSI_WRITE(CSI0_ER, CSI_ER_ESLOT(0) |	CSI_ER_RXEN | CSI_ER_ECSI );
	}

	spin_unlock_irqrestore(&s->lock, flags);
}

/*****************************************************************************/
/*	Stop DAC							     */
/*****************************************************************************/
static inline void stop_dac(struct ibmcsi3k_state *s)
{
	unsigned long flags;

	spin_lock_irqsave(&s->lock, flags);

	IBMCSI_WRITE(CSI0_ER,0);	/* Stop CSI */
	IBMCSI_WRITE(CSI0_TBUF,0); 	/* Zero out tx buffer to prevent popping sound */
#ifdef USE_SG /* Scatter/Gather */
	mtdcr(DCRN_ASGC,(mfdcr(DCRN_ASGC) | TX_SG_MASK) & ~TX_SG_ENABLE);
#endif
        mtdcr(IBMCSI_TXDMA_CR,
		mfdcr(IBMCSI_TXDMA_CR) & ~DCRN_DMA_CR_CE);	/* Stop Tx DMA channel */
	s->state &= ~IBMCSI_DAC_RUNNING;

   	spin_unlock_irqrestore(&s->lock, flags);

}

/*****************************************************************************/
/*	Stop ADC							     */
/*****************************************************************************/
static inline void stop_adc(struct ibmcsi3k_state *s) {

	unsigned long flags;
	spin_lock_irqsave(&s->lock, flags);

	/* TODO: currently assumes only playback or capture at a time, no duplex. */
	IBMCSI_WRITE(CSI0_ER,0);		/* Stop CSI0 */
	mtdcr(IBMCSI_RXDMA_CR, mfdcr(IBMCSI_RXDMA_CR) & ~DCRN_DMA_CR_CE);
	s->state &= ~IBMCSI_ADC_RUNNING;

	spin_unlock_irqrestore(&s->lock, flags);
}


/*****************************************************************************/
/* 	Allocate / initialize DMA buffer and related variables		     */
/*****************************************************************************/
static int prog_dmabuf(struct ibmcsi3k_state *s, struct dmabuf *db, unsigned rate, unsigned fmt)
{
	int order;
	unsigned bytepersec;
	unsigned bufs;
	struct page *page, *pend;


	db->hwptr = db->swptr = db->total_bytes = db->count = db->error = db->endcleared = 0;

	if (!db->rawbuf) { /* DMA buffer not allocated yet, go get it. */ 
		db->ready = db->mapped = 0;
		for (order = DMABUF_DEFAULTORDER; order >= DMABUF_MINORDER; order--) {

#ifdef USECONSISTENTALLOC
			if ((db->rawbuf = consistent_alloc(GFP_ATOMIC | GFP_DMA, order, &(db->dmaaddr)))) {
				db->hwptr = 0;
				break;
			}
#else
			if ((db->rawbuf = (void *)__get_free_pages(GFP_KERNEL | GFP_DMA, order))) {
				db->dmaaddr = (dma_addr_t)virt_to_phys(db->rawbuf);
				break;
			}
#endif
		}

		if (!db->rawbuf)
			return -ENOMEM;

 		DBG(printk("buforder: %8.8x\n", order));

		db->buforder = order;
		/* now mark the pages as reserved; otherwise remap_page_range doesn't do what we want */
		pend = virt_to_page(__va(db->dmaaddr) + (PAGE_SIZE << db->buforder) - 1);
		for (page = virt_to_page(__va(db->dmaaddr)); page <= pend; page++) {
			mem_map_reserve(page);
		}

#if defined(USE_SG)
		{ /* Program the count and address into all descriptors in the ring */
			struct dma_sgdt *pWork = s->adc_free_sgdt_q; /* Use all the queue for now */
			unsigned dma_count = ((long)PAGE_SIZE << db->buforder) / 4;

			s->adc_active_sgdt_q = pWork;

			while (pWork) { /* Sanity check */
				pWork->ccw = (unsigned int) (IBMCSI_RXDMA_GO_NOI) ;
				pWork->srcP = (void *)CSI0_RBUF;
				pWork->destP = (void *)db->dmaaddr;
				pWork->ctrl = dma_count | 0x80000000;

				if (pWork->nextV == NULL) {
					pWork->nextV = s->adc_active_sgdt_q; /* Create a ring */
				}
				pWork->nextP = (void *) virt_to_phys(pWork->nextV);

				pWork = pWork->nextV;
				if (pWork == s->adc_active_sgdt_q) break;
			}
			if (pWork == NULL) printk("ibmcsi3k: INTERNAL ERROR! Queue corrupted.\n");
		}
#endif

        }

	bytepersec = IBMCSI_DEFAULT_SAMPLING_RATE * 4; /* TODO: Hardcoded for 11.025 KHz */

	bufs = PAGE_SIZE << db->buforder;


	/* TODO: decypher the following code block (inherited from es1370.c) */
	if (db->ossfragshift) {
		if ((1000 << db->ossfragshift) < bytepersec)
			db->fragshift = ld2(bytepersec/1000);
		else
			db->fragshift = db->ossfragshift;
	}
	else {
		db->fragshift = ld2(bytepersec/100/(db->subdivision ? db->subdivision : 1));
		if (db->fragshift < 3)
			db->fragshift = 3;
	}
	db->numfrag = bufs >> db->fragshift;
	while (db->numfrag < 4 && db->fragshift > 3) {
		db->fragshift--;
		db->numfrag = bufs >> db->fragshift;
	}
	db->fragsize = 1 << db->fragshift;
	if (db->ossmaxfrags >= 4 && db->ossmaxfrags < db->numfrag)
		db->numfrag = db->ossmaxfrags;
	db->fragsamples = db->fragsize >> sample_shift[fmt];
	db->dmasize = db->numfrag << db->fragshift;
	/* End TODO */

	/* TODO: Sample format hardcoded - assumes 0 is neutral sample. */
	memset(db->rawbuf, 0, db->dmasize);

	db->enabled = 1;
	db->ready = 1;
	return 0;
}

/*****************************************************************************/
/*	Prepare DMA buffer for ADC					     */
/*****************************************************************************/
static inline int prog_dmabuf_adc(struct ibmcsi3k_state *s)
{
	stop_adc(s);

	/* TODO: Sampling rate hardcoded at 11.025 KHz. */
	return prog_dmabuf(s,&s->dma_adc,IBMCSI_DEFAULT_SAMPLING_RATE,0);
}

/*****************************************************************************/
/* 	Prepare DMA buffer for DAC					     */
/*****************************************************************************/
static inline int prog_dmabuf_dac(struct ibmcsi3k_state *s)
{
	stop_dac(s);

	/* TODO: Sampling rate hardcoded at 11.025 KHz. */
	return prog_dmabuf(s, &s->dma_dac, IBMCSI_DEFAULT_SAMPLING_RATE, 0);
}

/*****************************************************************************/
/* 	Deallocate DMA buffer						     */
/*****************************************************************************/
static inline void dealloc_dmabuf(struct ibmcsi3k_state *s, struct dmabuf *db)
{
	struct page *page, *pend;

	if (db->rawbuf) {
		/* undo marking the pages as reserved */
		pend = virt_to_page(__va(db->dmaaddr) + (PAGE_SIZE << db->buforder) - 1);
		for (page = virt_to_page(__va(db->dmaaddr)); page <= pend; page++)
			mem_map_unreserve(page);

#ifdef USECONSISTENTALLOC
		consistent_free((void *)db->rawbuf, db->buforder);
#else
		free_pages((unsigned long)db->rawbuf, db->buforder);
#endif
	}
	db->rawbuf = NULL;
	db->mapped = db->ready = 0;
}



/*****************************************************************************/
/*	Procedure get_hwptr: Update DMA controller address offset hwptr	     */ 
/*		(returns # of bytes transferred)			     */
/*****************************************************************************/
static inline unsigned get_hwptr(struct ibmcsi3k_state *s, struct dmabuf *db, unsigned channel)
{
	unsigned hwptr, diff;
	switch (channel) {
    		case IBMCSI_TXDMA:	/* Tx DMA channel */
			hwptr = (unsigned)mfdcr(IBMCSI_TXDMA_SA) - (unsigned)(db->dmaaddr);
			/* Get source offset for next transfer on Tx DMA channel */
              		break;
               	case IBMCSI_RXDMA:	/* Rx DMA channel */
			hwptr = (unsigned)mfdcr(IBMCSI_RXDMA_DA) - (unsigned)(db->dmaaddr);
                       	break;
               	default:/* not supported */
			return 0;
	}

	/* Calculate number of actual data byte transfers */
	diff = hwptr - db->hwptr; 
	/* The pointer is void * into physical memory; diff here is in bytes */

	if (hwptr >=  db-> dmasize) /* Wrap */
		hwptr = 0;

	if (hwptr < 0) { /* Out of area */
		hwptr = 0;
		diff = 0; /* Fix up */
	}
	db->hwptr = hwptr; /* Update next transfer start offset */
	return diff;
}

/*****************************************************************************/
/* 	Procedure clear_advance:					     */
/* 	Fill the tail end of buffer with neutral sample data 	             */
/*****************************************************************************/
static inline void clear_advance(void *buf, unsigned bsize, unsigned bptr, unsigned len, unsigned char c)
{
	if (bptr + len > bsize) {
		unsigned x = bsize - bptr;
		memset(((char *)buf) + bptr, c, x);
		bptr = 0;
		len -= x;
	}
	memset(((char *)buf) + bptr, c, len);
}

/*****************************************************************************/
/* 	Update pointers for the next DMA operation.			     */
/*	* Must be called with spinlock held.  				     */
/*****************************************************************************/
static void ibmcsi3k_update_ptr(struct ibmcsi3k_state *s)
{
	int diff;

	/* update ADC pointer */
	if (s->state & IBMCSI_ADC_RUNNING) {
		diff = get_hwptr(s, &s->dma_adc, IBMCSI_RXDMA); /* ADC channel (Rx DMA) */
		s->dma_adc.total_bytes += diff;
		s->dma_adc.count += diff;
		if (s->dma_adc.count >= (signed)s->dma_adc.fragsize) {
			wake_up(&s->dma_adc.wait);
		}
		else {
			DBG(printk("!@ %8.8x %8.8x\n", s->dma_adc.count, s->dma_adc.fragsize));
		}

		if (!s->dma_adc.mapped) {
			if (s->dma_adc.count > (signed)(s->dma_adc.dmasize - ((3 * s->dma_adc.fragsize) >> 1))) {
				s->state &= ~IBMCSI_ADC_RUNNING;
				s->dma_adc.error++;
			}
			else {
				s->state &= ~IBMCSI_ADC_RUNNING;
				start_adc(s);
			}
		}
		/* TODO: Give some thoughts about mmap support */
	}

	/* update DAC pointer */
	if (s->state & IBMCSI_DAC_RUNNING) {

		diff = get_hwptr(s, &s->dma_dac, IBMCSI_TXDMA);	 /* DAC channel (Tx DMA) */

		s->dma_dac.total_bytes += diff;
		if (s->dma_dac.mapped) { /* mapped = 0 if prepared */
			s->dma_dac.count += diff;
			if (s->dma_dac.count >= (signed)s->dma_dac.fragsize)
				wake_up(&s->dma_dac.wait);
		}
		else {
			s->dma_dac.count -= diff;
			if (s->dma_dac.count <= 0) {
				mtdcr(IBMCSI_TXDMA_CR, 0);	/* Disable Tx DMA channel */
                                s->state &= ~IBMCSI_DAC_RUNNING;
				s->dma_dac.error++;
			}
			else if (s->dma_dac.count <= (signed)s->dma_dac.fragsize && !s->dma_dac.endcleared) {
				clear_advance(s->dma_dac.rawbuf, s->dma_dac.dmasize, s->dma_dac.swptr,
					      s->dma_dac.fragsize,  0 );
				s->dma_dac.endcleared = 1;
			}

			s->state &= ~IBMCSI_DAC_RUNNING; /* Let start_dac() do it */
			if (s->dma_dac.count + (signed)s->dma_dac.fragsize <= (signed)s->dma_dac.dmasize) {
				wake_up(&s->dma_dac.wait);
			}
			else start_dac(s);

		}
	}
}


/*****************************************************************************/
/* 	DAC (Playback) Timer Interrupt Handler				     */
/*****************************************************************************/
static void ibmcsi_dac_timer(unsigned long param) {

#ifdef USE_SG
	struct ibmcsi3k_state *s = (struct ibmcsi3k_state *)param;
	struct dma_sgdt *pCurrent_dt, *pNext_dt, *pWork;
	struct dma_sgdt *pAnchor;
	int count = 0;

        pAnchor = s->dac_sgdt_lastV;

	if ( ( mfdcr(DCRN_ASGC) & TX_SG_ENABLE ) &&  /* DAC running */
	     ( mfdcr(IBMCSI_TXDMA_CT) <= 4 ) && /* No time to do anything  	*/
	/* 4 samples = a little less than 100us at 44KHz 			*/
		( (void *)mfdcr( IBMCSI_TXDMA_SA ) != (pAnchor->srcP) ) ) {
		udelay(100); /* Let the 4 samples run its course */
	}

	if ( mfdcr(DCRN_ASGC) & TX_SG_ENABLE )  { /* DAC running */

		if (pAnchor->prevV) { /* Data queued */

			pNext_dt = (struct dma_sgdt *) phys_to_virt(mfdcr(TX_SG)); /* Next S/G descriptor table */

			if (check_sgdt_range(s, pNext_dt, SGDT_COUNT)) {
				pCurrent_dt = pNext_dt->prevV;

				if (pCurrent_dt) {
					pWork = pCurrent_dt;
					if (pWork->prevV) {
						struct dma_sgdt *pTemp = pWork;
						pWork = pWork->prevV;
						pTemp->prevV = NULL; /* Dequeue previous data */

						while (pWork) {
							count += ((unsigned int)(pWork->ctrl)) & (unsigned int) 0xffff;
							pTemp = pWork;
							pWork = pWork->prevV; /* Fetch next uplink pointer */
							free_sgdt( &(s->dac_free_sgdt_q), pTemp );
						}
					}
					s->dac_active_sgdt_q = pCurrent_dt;
				}

			}

			s->dma_dac.total_bytes += count * 4;
			s->dma_dac.count -= count * 4;
			if (s->dma_dac.count <= 0) {

			/*	stop_dac(s);	*//* ??? */

				s->dma_dac.error++;
			}
			if (count) {
				wake_up(&s->dma_dac.wait);
			}
		}
	}

	mod_timer(&s->dac_timer, jiffies + DAC_TIMER_PERIOD);
#endif

}

/*****************************************************************************/
/* 	DAC (Playback) Interrupt Handler				     */
/*****************************************************************************/
static void ibmcsi3k_dac_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
#ifdef USE_SG
	printk("ibmcsi: spurious dac int\n");
	return;
#else
        struct ibmcsi3k_state *s = (struct ibmcsi3k_state *)dev_id;
	/* We assume no interrupt sharing. Eval board has no other peripherals on the audio IRQs */

	spin_lock(&s->lock);

	/* Stop CSI first, then clear DMA TX interrupt. */
	IBMCSI_WRITE(CSI0_ER, 0);	 	/* Disable CSI */

		DBG(printk("DEBUG: DMA0_SR %8x\n", mfdcr(IBMCSI_DMA_SR)));

	mtdcr(IBMCSI_TXDMA_CR, 0);		/* Disable Tx DMA channel */
	mtdcr(IBMCSI_DMA_SR, DCRN_DMA_SR_ALL(IBMCSI_TXDMA)); 	/* Clear Terminal Count etc. */
	IBMCSI_WRITE(CSI0_SR, CSI_SR_TOD); 	/* Clear Tx overrun for good measure */

		DBG(IBMCSI_WRITE(CSI0_SCR, 0x02)); 	/* Debug breakpoint */
	if ( !(s->state & IBMCSI_HALT) ) { /* Update pointers later if in HALT state. */
		ibmcsi3k_update_ptr(s);
	}
	spin_unlock(&s->lock);
#endif
}


/*****************************************************************************/
/* 	ADC (Capture) Interrupt Handler					     */
/*****************************************************************************/
static void ibmcsi3k_adc_interrupt(int irq, void *dev_id, struct pt_regs *regs) {
	struct ibmcsi3k_state *s = (struct ibmcsi3k_state *)dev_id;

	spin_lock(&s->lock);

	/* Stop CSI, then clear DMA Rx interrupt. */
	IBMCSI_WRITE(CSI0_ER, 0);
	mtdcr(IBMCSI_RXDMA_CR, 0);
	mtdcr(IBMCSI_DMA_SR, DCRN_DMA_SR_ALL(IBMCSI_RXDMA));	/* Clear Terminal Count etc. */
	IBMCSI_WRITE(CSI0_SR, CSI_SR_ROD);	/* Clear Rx overrun for good measure */

	ibmcsi3k_update_ptr(s);
	spin_unlock(&s->lock);
}
/* ibmcsi_adc_timer to be added */

/*****************************************************************************/
/* 	Drain DAC (flush playback buffer)				     */
/*****************************************************************************/
static int drain_dac(struct ibmcsi3k_state *s, int nonblock)
{
#if 0 /* TODO! : Drain_dac not implemented yet. Timeout value depends on sampling rate. */
	DECLARE_WAITQUEUE(wait, current);

	unsigned long flags;
	int count, tmo;

	if (s->dma_dac.mapped || !s->dma_dac.ready)
		return 0;
        add_wait_queue(&s->dma_dac.wait, &wait);
        for (;;) {
		__set_current_state(TASK_INTERRUPTIBLE);
                spin_lock_irqsave(&s->lock, flags);
		count = s->dma_dac.count;
                spin_unlock_irqrestore(&s->lock, flags);
		if (count <= 0)
			break;
		if (signal_pending(current))
                        break;
                if (nonblock) {
                        remove_wait_queue(&s->dma_dac.wait, &wait);
                        set_current_state(TASK_RUNNING);
                        return -EBUSY;
                }
		tmo = 3 * HZ * (count + s->dma_dac.fragsize) / 2
			/ DAC2_DIVTOSR((s->ctrl & CTRL_PCLKDIV) >> CTRL_SH_PCLKDIV);
		tmo >>= sample_shift[(s->sctrl & SCTRL_P2FMT) >> SCTRL_SH_P2FMT];
		if (!schedule_timeout(tmo + 1))
			DBG(printk(KERN_DEBUG "ibmcsi3k: DMA timed out??\n");)
        }
        remove_wait_queue(&s->dma_dac.wait, &wait);
        set_current_state(TASK_RUNNING);
        if (signal_pending(current))
                return -ERESTARTSYS;
        return 0;
#else
		return 0;
#endif
}

/*****************************************************************************/
/* 	Reformat and copy device format data to user format data	     */
/*		(for capture)						     */
/*****************************************************************************/
static unsigned long copy_reformat_to_user(char *pDest, const char *pSrc, size_t count) {
	unsigned long ret;
	unsigned int i;
	char *pWrp = (char *)pSrc ;		/* Point at the first sample */
	char *pRdp = (char *)pSrc ;

	/* Copy 'count' SAMPLES of 16 bit data from pSrc to pDest */
	/* Contract in place on source */
	for (i=0; i<count; i++) {
		*( (unsigned int *) pWrp )  = *( (unsigned int *) pRdp );
		pWrp += 2;
		pRdp += 4;
	}
	if ( (ret = copy_to_user(pDest, pSrc, count*2)) ) {
		printk("DEBUG: copy_to_user failed, from %p to %p for %8x \n", pSrc, pDest, count);
		return ret;
	}
	return 0;
}

/*****************************************************************************/
/*	Reformat and copy user format data to device format data	     */
/*		(for playback)						     */
/*****************************************************************************/
static unsigned long copy_reformat_from_user(char *pDest, const char *pSrc, size_t count) {
	unsigned long ret;
	unsigned int i;
	char *pWrp = pDest + (count-1) * 4;		/* Point to the last sample */
	char *pRdp = pDest + (count-1) * 2;

	/* Copy 'count' samples of 16 bit data from pSrc to pDest */
	if ( (ret = copy_from_user(pDest, pSrc, count*2) ) ) {
		printk("DEBUG: copy_from_user failed, from %p to %p for %8x \n", pSrc, pDest, count);
		return ret;
	}
	/* Expand in place */
	for (i=0; i<count; i++) {
		unsigned int temp;
		temp = *((unsigned int *)pRdp);
		temp &= 0xfffe0000; /* Must turn off the LSB of the upper 16 bit data */
		*((unsigned int *)pWrp) = temp;
		pWrp -= 4;
		pRdp -= 2;
	}
	return 0;
}

/*****************************************************************************/
/* 	Base 2 logarithm (approx.)					     */
/*****************************************************************************/
static inline unsigned ld2(unsigned int x)
{
	unsigned r = 0;

	if (x >= 0x10000) {
		x >>= 16;
		r += 16;
	}
	if (x >= 0x100) {
		x >>= 8;
		r += 8;
	}
	if (x >= 0x10) {
		x >>= 4;
		r += 4;
	}
	if (x >= 4) {
		x >>= 2;
		r += 2;
	}
	if (x >= 2)
		r++;
	return r;
}

#if defined(USE_SG)
/*****************************************************************************/
/* 	Scatter/Gather descriptor queue handlers			     */
/*****************************************************************************/
/* Spinlocks TBD (rely on global spinlocks for now)  			     */

/* Initialize logical (virtual address) pool of descriptor tables */
static void init_sgdt_q(struct dma_sgdt *queue, int count) {
	int i;
	struct dma_sgdt *work = queue;

	work->ctrl = 0;
	work->prevV = NULL;		/* Terminate backward link 	*/
	for (i=1;i<count;i++) {
		work[i].ctrl = 0;
		work[i].prevV = &(work[i-1]);
		work[i-1].nextV = &(work[i]);
	}
	work[count-1].nextV = NULL; 	/* Terminate forward link 	*/

	/* Clean house so we get an exception if any field is uninitialized */
	for (i=0;i<count;i++) {
		work[i].ccw = 0;
		work[i].srcP = NULL;
		work[i].destP = NULL;
		work[i].ctrl = 0;
	}
}

/* Get a descriptor table from free pool */
static struct dma_sgdt *get_sgdt(struct dma_sgdt **queueaddress) {
	struct dma_sgdt *work = *queueaddress;

	DBG(printk(">"));
	if (*queueaddress == NULL) return NULL;

	work->prevV = NULL; /* Unlink backward */
	*queueaddress = work->nextV; /* Update queue pointer */

	if (*queueaddress)
	  (*queueaddress)->prevV = NULL;

	work->nextV = NULL; /* Unlink forward */

	DBG(printk("get_sgdt q %8.8x ret %8.8x\n",*queueaddress, work));
	return work;
}

/* Return a descriptor table to free pool */
static void free_sgdt(struct dma_sgdt **queueaddress, struct dma_sgdt *dt) {
	struct dma_sgdt *work = *queueaddress;
	DBG(printk("free_sgdt %8.8x queue %8.8x ", dt, work));
	DBG(printk("<"));

	*queueaddress = dt;	/* Update queue read pointer */
	dt->prevV = NULL; /* Unlink backward */
	dt->nextV = work; /* Link forward */
	if (work) work->prevV = dt; /* Link backward if non-null */

	DBG(printk("now %8.8x\n",*queueaddress));
}

/* Check the range of descriptor (stubbed out for now) */
static unsigned int check_sgdt_range(struct ibmcsi3k_state *s, struct dma_sgdt *dt, int count) {
	return 1;
}
#endif

/********************************************************************************/
/********************************************************************************/
/********************************************************************************/
/*		 Part II : Mixer device	/dev/mixer				*/
/********************************************************************************/
/********************************************************************************/
/********************************************************************************/


/* ---------------------------------------------------------------------------- */
/* The Si3000 codec supports the following channels: 				*/
/* 	- Master Volume (Line out; std. 600 ohm impedance)			*/
/*	- Speaker 	(Speaker; 32 ohm impedance)				*/
/* 	- Line 		(Line in)						*/
/*	- Mic		(Mic in)						*/
/* 	- Phone In/Out  (Handset; not connected on IBM 405LP Evaluation Board)	*/
/* 										*/
/* ---------------------------------------------------------------------------- */


/*****************************************************************************/
/*	/dev/mixer Open							     */
/*****************************************************************************/
static int ibmcsi3k_open_mixdev(struct inode *inode, struct file *file)
{
	int minor = MINOR(inode->i_rdev);
	struct list_head *list;
	struct ibmcsi3k_state *s;

	for (list = devs.next; ; list = list->next) {
		if (list == &devs)
			return -ENODEV;
		s = list_entry(list, struct ibmcsi3k_state, devs);
		if (s->dev_mixer == minor)
			break;
	}
       	VALIDATE_STATE(s);
	file->private_data = s;
	init_MUTEX(&s->mix_sem);
	return 0;
}

/*****************************************************************************/
/*	/dev/mixer IOCTL						     */
/*****************************************************************************/
/* This is the ioctl entry to the mixer driver. */
static int ibmcsi3k_ioctl_mixdev(struct inode *ioctl,
			      struct file *file,
			      unsigned int cmd,
			      unsigned long arg)
{
	struct ibmcsi3k_state *s = (struct ibmcsi3k_state *) file->private_data;
	const unsigned int nr = _IOC_NR(cmd);
	int retval;

	down(&s->mix_sem);
	{
	        if (cmd == SOUND_MIXER_INFO) {
			mixer_info info;
			strncpy(info.id, "IBMCSI3K", sizeof(info.id));
			strncpy(info.name, "IBM 405LP CSI3K", sizeof(info.name));
			info.modify_counter = s->modcnt;
			if (copy_to_user((void *)arg, &info, sizeof(info)))
				return -EFAULT;
			return 0;
		}
		if (cmd == SOUND_OLD_MIXER_INFO) {
			_old_mixer_info info;
			strncpy(info.id, "IBMCSI3K", sizeof(info.id));
			strncpy(info.name, "IBM 405LP CSI3K", sizeof(info.name));
			if (copy_to_user((void *)arg, &info, sizeof(info)))
				return -EFAULT;
			return 0;
		}
		if (cmd == OSS_GETVERSION)
			return put_user(SOUND_VERSION, (int *)arg);

		if (_IOC_TYPE(cmd) != 'M' || _SIOC_SIZE(cmd) != sizeof(int))
			return -EINVAL;

		if (_SIOC_DIR(cmd) == _SIOC_READ)
			retval = mixer_read_ioctl(s, nr, (caddr_t) arg);
		else if ( (_SIOC_DIR(cmd) == _SIOC_WRITE) ||
			  (_SIOC_DIR(cmd) == (_SIOC_READ | _SIOC_WRITE) ) )
			retval = mixer_write_ioctl(s, nr, (caddr_t) arg);
		else
			retval = -EINVAL;
	}
	up(&s->mix_sem);
	return retval;
}

/*****************************************************************************/
/*	/dev/mixer Release						     */
/*****************************************************************************/
static int ibmcsi3k_release_mixdev(struct inode *inode, struct file *file)
{
	struct ibmcsi3k_state *s = (struct ibmcsi3k_state *)file->private_data;

	VALIDATE_STATE(s);
	return 0;
}


/*****************************************************************************/
/*	/dev/mixer Read IOCTLs						     */
/*****************************************************************************/
static int mixer_read_ioctl(struct ibmcsi3k_state *s, unsigned int nr, caddr_t arg)
{
	int val = -1;

	switch (nr) {
	case SOUND_MIXER_CAPS:
		val = 0; /* Can support mixing mic and line in */
		break;

	case SOUND_MIXER_DEVMASK:
		val = (	SOUND_MASK_VOLUME | SOUND_MASK_LINE 	| 	/* Line out / Line in */
		       	SOUND_MASK_MIC	| SOUND_MASK_SPEAKER	| 	/* Mic in / Speaker out */

#if 0 /* No support for handset hardware */
			SOUND_MASK_PHONEIN | SOUND_MASK_PHONEOUT |
#endif

			SOUND_MASK_OGAIN | SOUND_MASK_IGAIN ); 		/* Tx/Rx PGA */
		break;

	case SOUND_MIXER_STEREODEVS:
		val = 0 ; /* No stereo devices */
		break;

	case SOUND_MIXER_OUTMASK:
		val = 0 ; /* No routing of inputs to outputs */
		break;

	case SOUND_MIXER_RECMASK: /* Recording devices */
		val = (SOUND_MASK_LINE | SOUND_MASK_MIC );
		break;

	case SOUND_MIXER_VOLUME: /* Line out volume */
		val = ibmcsi3k_get_volume(s);
		break;

	case SOUND_MIXER_LINE: 	/* Line in volume */
		val = ibmcsi3k_get_line(s);
		break;

	case SOUND_MIXER_MIC:	/* Mic in volume */
		val = ibmcsi3k_get_mic(s);
		break;

	case SOUND_MIXER_SPEAKER: /* Speaker out volume */
		val = ibmcsi3k_get_speaker(s);
		break;

	case SOUND_MIXER_OGAIN:	/* Output Pgmable Gain/Attenuation */
		val = ibmcsi3k_get_txpga(s);
		break;

	case SOUND_MIXER_IGAIN: /* Input Pgmable Gain/Attenuation */
		val = ibmcsi3k_get_rxpga(s);
		break;

	case SOUND_MIXER_OUTSRC: /* Unmuted output sources */
		val = ibmcsi3k_get_outsrc(s);
		break;

	case SOUND_MIXER_RECSRC: /* Unmuted recording sources */
		val = ibmcsi3k_get_recsrc(s);
		break;
	default:
		return -EINVAL;
	}
	return put_user(val, (int *) arg);
}

/*****************************************************************************/
/*	/dev/mixer Write IOCTLs						     */
/*****************************************************************************/
static int mixer_write_ioctl(struct ibmcsi3k_state *s, unsigned int nr, caddr_t arg)
{
	int val;
	int err;

	err = get_user(val, (int *) arg);

	if (err)
		return -EFAULT;
	switch (nr) {
	case SOUND_MIXER_VOLUME:
		val = ibmcsi3k_set_volume(s, val>>8);
		break;

	case SOUND_MIXER_LINE:
		val = ibmcsi3k_set_line(s, val>>8);
		break;

	case SOUND_MIXER_MIC:
		val = ibmcsi3k_set_mic(s, val>>8);
		break;

	case SOUND_MIXER_SPEAKER:
		val = ibmcsi3k_set_speaker(s, val>>8);
		break;

	case SOUND_MIXER_IGAIN:
		val = ibmcsi3k_set_rxpga(s, val>>8);
		break;

	case SOUND_MIXER_OGAIN:
		val = ibmcsi3k_set_txpga(s, val>>8);
		break;

	case SOUND_MIXER_RECSRC:
		if (s->state & (IBMCSI_DAC_RUNNING | IBMCSI_ADC_RUNNING) )
			return -EBUSY;	/* can't change recsrc while running */
		val = ibmcsi3k_set_recsrc(s, val);
		break;

	case SOUND_MIXER_OUTSRC:
		val = ibmcsi3k_set_outsrc(s, val);
		break;

	default:
		return -EINVAL;
	}
	if (val < 0)
		return val;
	return put_user(val, (int *) arg);
}


/*****************************************************************************/
/*	Mixer IOCTL routines						     */
/*****************************************************************************/

	/************************/
	/************************/
	/* 	Get routines 	*/
	/************************/
	/************************/

/*****************************************************************************/
/* 	Read Line Out volume						     */
/*****************************************************************************/
static int ibmcsi3k_get_volume(struct ibmcsi3k_state *s) {
	int val ;

	unsigned char attenuation =
			( si3000_read_reg( s, SI3000_AA /* 9 */ )
				& SI3000_AA_LOT /* 0x0C */ )
				>> SI3000_AA_LOT_OFFSET /* 2 */ ;
	/* Register 9 bit 3:2  11= -18dB, 10=-12 dB, 01=-6 dB, 00= 0 dB */

	unsigned char active =
			( si3000_read_reg( s, SI3000_ADC_VC /* 6 */ )
				& SI3000_ADC_VC_LOM /* 0x02 */ )
				>> SI3000_ADC_VC_LOM_OFFSET  /* 1 */ ;
	/* Register 6 bit 1    0 = Line Out Mute, 1 = Active */

	int volume[4] = { 100, 75, 50, 25 };	/* Map to 0-100 */

	if (active) val = volume[attenuation];
	else val = 0;

	return val;
}

/*****************************************************************************/
/* 	Read Line In volume (VOLUME)					     */
/*****************************************************************************/
static int ibmcsi3k_get_line(struct ibmcsi3k_state *s) {
	int val;

	unsigned char gain, mute, temp;
	int volume[4] = { 0, 33, 66, 100 } ;

	temp = si3000_read_reg( s, SI3000_RXGC1 /* 5 */ );
	/* Register 5 	bit 7:6 11=20dB, 10=10dB, 01=0dB (00 reserved) 	*/
	/*		bit 5 1 = Line In Mute (reverse polarity vs out */

	gain = ( temp & SI3000_RXGC1_LIG /* 0xC0 */ ) >> SI3000_RXGC1_LIG_OFFSET /* 6 */ ;
	mute = temp & SI3000_RXGC1_LIM /* 0x20 */ ;

	if (mute) val = 0;
	else val = volume[gain];

	return val;
}

/*****************************************************************************/
/*	Read Mic In volume (MIC)					     */
/*****************************************************************************/
static int ibmcsi3k_get_mic(struct ibmcsi3k_state *s) {
	int val ;
	int mic_volume[4] = { 25, 50, 75, 100 } ;

	unsigned char gain, mute, temp;
	temp = si3000_read_reg( s, SI3000_RXGC1 /* 5 */  );
	/* Register 5 	bit 4:3 11=30dB, 10=20dB, 01=10dB, 00 = 0dB	*/
	/*		bit 2 0 = Mic In Mute (reverse polarity vs out) */

	gain = ( temp & SI3000_RXGC1_MIG /* 0x18 */ )
		>> SI3000_RXGC1_MIG_OFFSET /* 3 */;
	mute = ( temp & SI3000_RXGC1_MIM /* 0x04 */ );

	if (mute) val = 0;
	else val = mic_volume[gain];

	return val;
}

/*****************************************************************************/
/*	Read Speaker volume (AltPCM / PCM2)				     */
/*****************************************************************************/
static int ibmcsi3k_get_speaker(struct ibmcsi3k_state *s) {
	int val ;
	int speaker_volume[4] = { 100, 75, 50, 25 };

	unsigned char attenuation =
		( si3000_read_reg( s, SI3000_AA /* 9 */ )
		& SI3000_AA_SOT /* 0x03 */ );
		/* Register 9 bit 1:0 Speaker Attenuation 		*/
		/* 		11=-18dB 10=-12 dB 01=-6dB 00=0dB 	*/
	unsigned char active =
		( si3000_read_reg( s, SI3000_DAC_VC /* 7 */ )
		& ( SI3000_DAC_VC_SLM | SI3000_DAC_VC_SRM) /* 0x03 */ );
		/* Register 7 bit 1 0 = Speaker Left Mute, bit 0 0 = Speaker Right Mute */

	if (active) val = speaker_volume[attenuation];
	else val = 0;

	return val;
}

/*****************************************************************************/
/*	Read Output Programmable Gain / Attenuation (OGAIN)		     */
/*****************************************************************************/
static int ibmcsi3k_get_txpga(struct ibmcsi3k_state *s) {
	int val;
	unsigned char gain = ( si3000_read_reg( s, SI3000_DAC_VC /* 7 */ )
		 & SI3000_DAC_VC_TXG /* 0x7C */ )
		>> SI3000_DAC_VC_TXG_OFFSET /* 2 */ ;

	val = ( (unsigned int) gain * 100) / 31;

	return val;
}

/*****************************************************************************/
/*	Read Input Programmable Gain / Attenuation (IGAIN)		     */
/*****************************************************************************/
static int ibmcsi3k_get_rxpga(struct ibmcsi3k_state *s) {
	int val;
	unsigned char gain = ( si3000_read_reg( s, SI3000_ADC_VC /* 6 */ )
			& SI3000_ADC_VC_RXG /* 0x7C */ )
			>> SI3000_ADC_VC_RXG_OFFSET /* 2 */ ;

	val = ((unsigned int) gain * 100 ) / 31; /* 31 = binary 111 11 (5 bits), remap to 0-100 */

	return val;
}

/*****************************************************************************/
/*	Read Recording Source						     */
/*****************************************************************************/
static int ibmcsi3k_get_recsrc(struct ibmcsi3k_state *s) {
	int val = SOUND_MASK_LINE | SOUND_MASK_MIC;
	unsigned char temp = si3000_read_reg( s, SI3000_RXGC1 /* 5 */ );

	if ( temp & SI3000_RXGC1_LIM /* 0x20 */ ) val &= ~SOUND_MASK_LINE;
	if ( temp & SI3000_RXGC1_MIM /* 0x04 */ ) val &= ~SOUND_MASK_MIC;

	return val;
}

/*****************************************************************************/
/*	Read Output Source						     */
/*****************************************************************************/
static int ibmcsi3k_get_outsrc(struct ibmcsi3k_state *s) {
	int val = 0;
	unsigned char temp = si3000_read_reg( s, SI3000_ADC_VC /* 6 */ );
	if ( temp & SI3000_ADC_VC_LOM /* 0x02 */ )	/* Line out active (1) */
		val |= SOUND_MASK_VOLUME;

	temp = si3000_read_reg( s, SI3000_DAC_VC /* 7 */ );
	if ( temp & (SI3000_DAC_VC_SLM | SI3000_DAC_VC_SRM) ) /* Speaker left or right active (1) */
		val |= SOUND_MASK_SPEAKER;

	return val;
}

/*****************************************************************************/

	/************************/
	/************************/
	/* 	Set routines 	*/
	/************************/
	/************************/

/*****************************************************************************/
/*	Set Line Out volume (VOLUME)					     */
/*****************************************************************************/
static int ibmcsi3k_set_volume(struct ibmcsi3k_state *s, int val) {

	if (val < 0) val = 0;
	if (val > 100) val = 100;

	if ( val == 0) { /* Mute */
		si3000_write_reg( s, SI3000_ADC_VC /* 6 */ ,
			si3000_read_reg( s, SI3000_ADC_VC )
				& ~SI3000_ADC_VC_LOM /* 0xFD */ ) ; /* Bit 1 off */
	}
	else {
		unsigned char temp;
		si3000_write_reg( s, SI3000_ADC_VC /* 6 */ ,
			si3000_read_reg( s, SI3000_ADC_VC )
				| SI3000_ADC_VC_LOM /* 0x02 */ ) ; /* Unmute */

		temp =  ( (  100 - val ) / 25 ) << SI3000_AA_LOT_OFFSET /* 2 */ ;
		si3000_write_reg( s, SI3000_AA /* 9 */,
			( si3000_read_reg( s, SI3000_AA )
			& ~SI3000_AA_LOT /* 0xF3 */ ) | temp ) ;
	}

	return ibmcsi3k_get_volume(s);
}

/*****************************************************************************/
/*	Set Line In volume (LINE)					     */
/*****************************************************************************/
static int ibmcsi3k_set_line(struct ibmcsi3k_state *s, int val) {

	if (val < 0) val = 0;
	if (val > 100) val = 100;

	if ( val == 0 ) {
		si3000_write_reg( s, SI3000_RXGC1 /* 5 */,
			si3000_read_reg( s, SI3000_RXGC1 )
			| SI3000_RXGC1_LIM /* 0x20 */ ) ; /* Bit 5 on */
	}
	else {
		unsigned char temp;

		val = ( (val + 31) / 33 );

		if (val == 0) temp = 1; /* Patch ;-) */
		else temp = (unsigned char) val;

		temp = temp << SI3000_RXGC1_LIG_OFFSET /* 6 */ ;
		si3000_write_reg( s, SI3000_RXGC1 /* 5 */,
			( si3000_read_reg( s, SI3000_RXGC1 )
				& ~(SI3000_RXGC1_LIG | SI3000_RXGC1_LIM) /* 0x1F */ )
				| temp ) ;
			 /* Write bits 7:6, also bit 5 off (unmute) */
	}

	return ibmcsi3k_get_line(s);
}

/*****************************************************************************/
/*	Set Mic In volume (MIC)						     */
/*****************************************************************************/
static int ibmcsi3k_set_mic(struct ibmcsi3k_state *s, int val) {

	if (val < 0) val = 0;
	if (val > 100) val = 100;

	if (val == 0) { /* Mute mic in */
		si3000_write_reg( s, SI3000_RXGC1 /* 5 */,
			si3000_read_reg( s, SI3000_RXGC1 ) | SI3000_RXGC1_MIM /* 0x04 */ ) ; /* Bit 2 on */
	}
	else {
		unsigned char temp;

		val = (( ( val + 24) / 25 ) - 1);
		temp = (( (unsigned char) val ) & 0x03 ) << SI3000_RXGC1_MIG_OFFSET /* 3 */ ;
		si3000_write_reg( s, SI3000_RXGC1 /* 5 */,
			( si3000_read_reg( s, SI3000_RXGC1 )
			& ~( SI3000_RXGC1_MIG | SI3000_RXGC1_MIM) /* ~(0x18 | 0x04) = 0xE3 */ )
			| temp ) ;

	}

	return ibmcsi3k_get_mic(s);
}

/*****************************************************************************/
/* 	Set Speaker volume (AltPCM / PCM2)				     */
/*****************************************************************************/
static int ibmcsi3k_set_speaker(struct ibmcsi3k_state *s, int val) {

	if (val < 0) val = 0;
	if (val > 100) val = 100;

	if (val == 0) {	/* Mute speakers */
		si3000_write_reg( s, SI3000_DAC_VC /* 7 */,
			si3000_read_reg( s, SI3000_DAC_VC )
			& ~(SI3000_DAC_VC_SLM | SI3000_DAC_VC_SRM) /* 0xFC */ );
	}
	else { /* Set analog attenuation */
		unsigned char temp;
		si3000_write_reg( s, SI3000_DAC_VC /* 7 */,
			si3000_read_reg( s, SI3000_DAC_VC )
			| (SI3000_DAC_VC_SLM | SI3000_DAC_VC_SRM) /* 3 */ ); /* Unmute speakers */

		val = (100 - val)/25;
		temp = ( si3000_read_reg( s, SI3000_AA /* 9 */ )
			& ~SI3000_AA_SOT /* 0xFC */ )
			| (unsigned char) (val & SI3000_AA_SOT /* 0x03 */) ;
		si3000_write_reg( s, SI3000_AA, temp );
	}

	return ibmcsi3k_get_speaker(s);
}

/*****************************************************************************/
/*	Set Output Programmable Gain / Attenuation volume (OGAIN)	     */
/*****************************************************************************/
static int ibmcsi3k_set_txpga(struct ibmcsi3k_state *s, int val) {

	if (val < 0) val = 0;
	if (val > 100) val = 100;

	val = ( ( val * 31 ) / 100 ) << SI3000_DAC_VC_TXG_OFFSET /* 2 */;

	si3000_write_reg( s, SI3000_DAC_VC /* 7 */,
			( si3000_read_reg( s, SI3000_DAC_VC )
			& ~SI3000_DAC_VC_TXG /*0x83*/ )
			| val );

	return ibmcsi3k_get_txpga(s);
}

/*****************************************************************************/
/*	Set Input Programmable Gain / Attenuation volume (IGAIN)	     */
/*****************************************************************************/
static int ibmcsi3k_set_rxpga(struct ibmcsi3k_state *s, int val) {

	if (val < 0) val = 0;
	if (val > 100) val = 100;

	val = ( (val * 31) / 100 ) << SI3000_ADC_VC_RXG_OFFSET /* 2 */;

	si3000_write_reg( s, SI3000_ADC_VC /* 6 */,
			( si3000_read_reg( s, SI3000_ADC_VC )
			& ~SI3000_ADC_VC_RXG /* 0x83 */ )
			| val );

	return ibmcsi3k_get_rxpga(s);
}

/*****************************************************************************/
/* 	Set Recording Source 						     */
/*****************************************************************************/
static int ibmcsi3k_set_recsrc(struct ibmcsi3k_state *s, int val) {

	if (val & SOUND_MASK_MIC) { /* Mic in active */
		si3000_write_reg( s, SI3000_RXGC1 /* 5 */,
			si3000_read_reg( s, SI3000_RXGC1 )
			& ~SI3000_RXGC1_MIM /* 0xFB */ ) ; /* Turn off Mic In Mute bit 	*/
	}
	else { /* Mic in mute */
		si3000_write_reg( s, SI3000_RXGC1,
			si3000_read_reg( s, SI3000_RXGC1 )
			| SI3000_RXGC1_MIM /* 0x04 */ ) ; /* Turn on Mic In Mute bit 	*/
	}

	if (val & SOUND_MASK_LINE) { /* Line in active */
		si3000_write_reg( s, SI3000_RXGC1,
			si3000_read_reg( s, SI3000_RXGC1 )
			& ~SI3000_RXGC1_LIM /* 0xDF */ ) ; /* Turn off Line In Mute bit */
	}
	else { /* Line in mute */
		si3000_write_reg( s, SI3000_RXGC1,
			si3000_read_reg( s, SI3000_RXGC1 )
			| SI3000_RXGC1_LIM /* 0x20 */ ) ; /* Turn on Line In Mute bit 	*/
	}

	return ibmcsi3k_get_recsrc(s);
}

/*****************************************************************************/
/*	Set Output Source						     */
/*****************************************************************************/
static int ibmcsi3k_set_outsrc(struct ibmcsi3k_state *s, int val) {

	if (val & SOUND_MASK_VOLUME) { /* Line out active */
		si3000_write_reg( s, SI3000_ADC_VC /* 6 */,
			si3000_read_reg( s, SI3000_ADC_VC )
			| SI3000_ADC_VC_LOM /* 0x02 */ ) ; /* Turn on Line Out Mute bit */
	}
	else { /* Line out mute */
		si3000_write_reg( s, SI3000_ADC_VC,
			si3000_read_reg( s, SI3000_ADC_VC )
			& ~SI3000_ADC_VC_LOM /* 0xFD */ ) ;
	}

	if (val & SOUND_MASK_SPEAKER) { /* Speaker L/R active */
		si3000_write_reg( s, SI3000_DAC_VC /* 7 */,
			si3000_read_reg( s, SI3000_DAC_VC )
			| ( SI3000_DAC_VC_SLM | SI3000_DAC_VC_SRM ) /* 0x03 */ ) ;
	}
	else {	/* Speaker L/R mute */
		si3000_write_reg( s, SI3000_DAC_VC,
			si3000_read_reg( s, SI3000_DAC_VC )
			& ~( SI3000_DAC_VC_SLM | SI3000_DAC_VC_SRM ) /* 0xFC */ ) ;

	}

	return ibmcsi3k_get_outsrc(s);
}


/*****************************************************************************/


	/************************/
	/************************/
	/* Low level routines   */
	/************************/
	/************************/

/*****************************************************************************/
/*	Read Si3000 control register 					     */
/*	(handles pause/restart during playback/capture)			     */
/*****************************************************************************/
static unsigned char si3000_read_reg( struct ibmcsi3k_state *s, int reg ) {
   	unsigned int csi_er, csi_cfg, dma_cr, dma_ct, dma_sa, dma_da, val;
	unsigned long timeout, flag;

	if (s->state & IBMCSI_DAC_RUNNING) {
#if defined(USE_SG)
                return 0;       /* Tricky to halt/restart Scatter/Gather.       */
                                /* Stubbed out for now.                         */
#else
   		s->state |= IBMCSI_HALT;

		/* Save CSI status 	*/
		csi_er  = IBMCSI_READ(CSI0_ER);
		csi_cfg = IBMCSI_READ(CSI0_CFG);

		/* Halt CSI first	*/
		IBMCSI_WRITE(CSI0_ER, 0);

		timeout = jiffies + HZ/25; /* TODO (NTH):  fine tune */
		while (jiffies < timeout) { /* Wait for busy flag to go down. 	*/
			if ( !(IBMCSI_READ(CSI0_SR) & CSI_SR_CSIB) ) {
				break;
			}
		}

		asm volatile("sync");

		/* Save DMA setup 	*/
		dma_cr = mfdcr(IBMCSI_TXDMA_CR);
		dma_ct = mfdcr(IBMCSI_TXDMA_CT);
		dma_sa = mfdcr(IBMCSI_TXDMA_SA);
		dma_da = mfdcr(IBMCSI_TXDMA_DA);

		asm volatile("sync"); /* Paranoia: Make sure mtdcr 's and MMIO writes have all completed */

		spin_unlock_irqrestore(&s->lock, flag);
		/* Pending DMA interrupt (if any) will kick in at this point.	*/

		if ( !(IBMCSI_READ(CSI0_SR) & CSI_SR_CSIB) ) {
			mtdcr(IBMCSI_TXDMA_CR, 0);
			asm volatile("sync");

         		val = si3000_read_reg2(s, reg, 3);
			/* Fewer retries to minimize artifact / noise */
		}

		/* Restore CSI / Tx DMA setup */
		s->state &= ~IBMCSI_HALT;
		if (dma_ct == 0) {
			/* Transfer had completed, go fetch next block. */
			ibmcsi3k_update_ptr(s);
		}
		else {
			mtdcr(IBMCSI_DMA_SR, DCRN_DMA_SR_ALL(IBMCSI_TXDMA));

			mtdcr(IBMCSI_TXDMA_CT, dma_ct);
			mtdcr(IBMCSI_TXDMA_SA, dma_sa);
			mtdcr(IBMCSI_TXDMA_DA, dma_da);
			mtdcr(IBMCSI_TXDMA_CR, dma_cr);

			IBMCSI_WRITE(CSI0_CFG, csi_cfg);

			asm volatile ("sync");

			IBMCSI_WRITE(CSI0_ER, csi_er);
		}

#endif
	   }

	else if (s->state & IBMCSI_ADC_RUNNING) {
		s->state |= IBMCSI_HALT;

		/* Save CSI status 	*/
		csi_er  = IBMCSI_READ(CSI0_ER);
		csi_cfg = IBMCSI_READ(CSI0_CFG);

		/* Halt CSI first	*/
		IBMCSI_WRITE(CSI0_ER, 0);

		timeout = jiffies + HZ/25; /* TODO (NTH):  fine tune */
		while (jiffies < timeout) { /* Wait for busy flag to go down. 	*/
			if ( !(IBMCSI_READ(CSI0_SR) & CSI_SR_CSIB) ) {
				break;
			}
		}

		asm volatile("sync");

		/* Save DMA setup 	*/
		dma_cr = mfdcr(IBMCSI_RXDMA_CR);
		dma_ct = mfdcr(IBMCSI_RXDMA_CT);
		dma_sa = mfdcr(IBMCSI_RXDMA_SA);
		dma_da = mfdcr(IBMCSI_RXDMA_DA);

		asm volatile("sync"); /* Paranoia: Make sure mtdcr 's and MMIO writes have all completed */

		spin_unlock_irqrestore(&s->lock, flag);
		/* Pending DMA interrupt (if any) will kick in at this point.	*/

		if ( !(IBMCSI_READ(CSI0_SR) & CSI_SR_CSIB) ) {
			mtdcr(IBMCSI_RXDMA_CR, 0);
			asm volatile("sync");

         		val = si3000_read_reg2(s, reg, 3);
			/* Fewer retries to minimize artifact / noise */
		}

		/* Restore CSI / Rx DMA setup */
		s->state &= ~IBMCSI_HALT;
		if (dma_ct == 0) {
			/* Transfer had completed, go fetch next block. */
			ibmcsi3k_update_ptr(s);
		}
		else {	/* Restart previous transfer */
			mtdcr(IBMCSI_DMA_SR, DCRN_DMA_SR_ALL(IBMCSI_RXDMA));

			mtdcr(IBMCSI_RXDMA_CT, dma_ct);
			mtdcr(IBMCSI_RXDMA_SA, dma_sa);
			mtdcr(IBMCSI_RXDMA_DA, dma_da);
			mtdcr(IBMCSI_RXDMA_CR, dma_cr);

			IBMCSI_WRITE(CSI0_CFG, csi_cfg);

			asm volatile ("sync");

			IBMCSI_WRITE(CSI0_ER, csi_er);
		}

	}

	else {
		val = si3000_read_reg2( s, reg, 20 );
		/* Can afford a lot of retries until we get it right, if currently not active */
	}

   	return val;
}

/* TODO: Reduce redundancy. */

/*****************************************************************************/
/*	Read Si3000 control register - 2nd level 			     */
/*		(handles actual transfer, verification and retry) 	     */
/*****************************************************************************/
static unsigned char si3000_read_reg2( struct ibmcsi3k_state *s, int reg, int retrycount) {
		unsigned char result1, result2, result3;
		int i;
		for (i=0; i<retrycount;i++) {
			result1 = si3000_transfer( s, reg, 1 , 0, s->sync_interval);
			if (s->timeout)
				continue;
			result2 = si3000_transfer( s, reg, 1 , 0, s->sync_interval);
			if (s->timeout)
				continue;
			result3 = si3000_transfer( s, reg, 1 , 0, s->sync_interval);
			if (s->timeout)
				continue;

			if ( (result1 == result2) && (result1 == result3) ) {
#if 0 /* Debug messages */
				if (i) {
					printk("OK after %d mismatch(es), result %x\n",i,result1);
				}
				else {
					printk("OK first time, result %x\n", result1);
				}
#endif
				return result1;
			}
		}
		/* Bad! */
		printk("ibmcsi3k: Si3000 I/O error!\n");
    		return 0;

}

/*****************************************************************************/
/*	Write Si3000 control register 					     */
/*	(handles pause/restart during playback/capture)			     */
/*****************************************************************************/

/* Note: write_reg2 uses read_reg to verify, so read must be working first.  */

static void si3000_write_reg( struct ibmcsi3k_state *s, int reg, unsigned char val) {
	unsigned int csi_er, csi_cfg, dma_cr, dma_ct, dma_sa, dma_da;
	unsigned long timeout,flag;

	spin_lock_irqsave(&s->lock, flag);
	if (s->state & IBMCSI_DAC_RUNNING) {
#if defined(USE_SG)
                return;
#else
		s->state |= IBMCSI_HALT;

		/* Save CSI status 	*/
		csi_er  = IBMCSI_READ(CSI0_ER);
		csi_cfg = IBMCSI_READ(CSI0_CFG);

		/* Halt CSI first	*/
		IBMCSI_WRITE(CSI0_ER, 0);

		timeout = jiffies + HZ/25; /* TODO (NTH):  fine tune */
		while (jiffies < timeout) { /* Wait for busy flag to go down. 	*/
			if ( !(IBMCSI_READ(CSI0_SR) & CSI_SR_CSIB) ) {
				break;
			}
		}

		asm volatile("sync");

		/* Save DMA setup 	*/
		dma_cr = mfdcr(IBMCSI_TXDMA_CR);
		dma_ct = mfdcr(IBMCSI_TXDMA_CT);
		dma_sa = mfdcr(IBMCSI_TXDMA_SA);
		dma_da = mfdcr(IBMCSI_TXDMA_DA);

		asm volatile("sync");

		spin_unlock_irqrestore(&s->lock, flag);
		/* Pending DMA interrupt (if any) will kick in at this point.	*/

		if ( !(IBMCSI_READ(CSI0_SR) & CSI_SR_CSIB) ) {
			mtdcr(IBMCSI_TXDMA_CR, 0);
			asm volatile("sync");

			/* Write register 		*/
         		si3000_write_reg2(s,reg,val,5);
		}

		/* Restore CSI / Tx DMA setup */
		s->state &= ~IBMCSI_HALT;
		if (dma_ct == 0) {
			/* Transfer had completed, go fetch next block. */
			ibmcsi3k_update_ptr(s);
		}
		else {
			mtdcr(IBMCSI_DMA_SR, DCRN_DMA_SR_ALL(IBMCSI_TXDMA));

			mtdcr(IBMCSI_TXDMA_CT, dma_ct);
			mtdcr(IBMCSI_TXDMA_SA, dma_sa);
			mtdcr(IBMCSI_TXDMA_DA, dma_da);
			mtdcr(IBMCSI_TXDMA_CR, dma_cr);

			IBMCSI_WRITE(CSI0_CFG, csi_cfg);

			asm volatile ("sync");

			IBMCSI_WRITE(CSI0_ER, csi_er);
		}
#endif
	} /* TODO: See if we can support concurrent record/playback */
	else if (s->state & IBMCSI_ADC_RUNNING) {
		s->state |= IBMCSI_HALT;

		/* Save CSI status 	*/
		csi_er  = IBMCSI_READ(CSI0_ER);
		csi_cfg = IBMCSI_READ(CSI0_CFG);

		/* Halt CSI first	*/
		IBMCSI_WRITE(CSI0_ER, 0);

		timeout = jiffies + HZ/25; /* TODO (NTH):  fine tune */
		while (jiffies < timeout) { /* Wait for busy flag to go down. 	*/
			if ( !(IBMCSI_READ(CSI0_SR) & CSI_SR_CSIB) ) {
				break;
			}
		}

		asm volatile("sync");

		/* Save DMA setup 	*/
		dma_cr = mfdcr(IBMCSI_RXDMA_CR);
		dma_ct = mfdcr(IBMCSI_RXDMA_CT);
		dma_sa = mfdcr(IBMCSI_RXDMA_SA);
		dma_da = mfdcr(IBMCSI_RXDMA_DA);

		asm volatile("sync");

		spin_unlock_irqrestore(&s->lock, flag);
		/* Pending DMA interrupt (if any) will kick in at this point.	*/

		if ( !(IBMCSI_READ(CSI0_SR) & CSI_SR_CSIB) ) {
			mtdcr(IBMCSI_RXDMA_CR, 0);
			asm volatile("sync");

			/* Write register 		*/
         		si3000_write_reg2(s,reg,val,5);
		}

		/* Restore CSI / Rx DMA setup */
		s->state &= ~IBMCSI_HALT;
		if (dma_ct == 0) {
			/* Transfer had completed, go fetch next block. */
			ibmcsi3k_update_ptr(s);
		}
		else {
			mtdcr(IBMCSI_DMA_SR, DCRN_DMA_SR_ALL(IBMCSI_RXDMA));

			mtdcr(IBMCSI_RXDMA_CT, dma_ct);
			mtdcr(IBMCSI_RXDMA_SA, dma_sa);
			mtdcr(IBMCSI_RXDMA_DA, dma_da);
			mtdcr(IBMCSI_RXDMA_CR, dma_cr);

			IBMCSI_WRITE(CSI0_CFG, csi_cfg);

			asm volatile ("sync");

			IBMCSI_WRITE(CSI0_ER, csi_er);
		}

	}
	else {
		spin_unlock_irqrestore(&s->lock, flag);
		si3000_write_reg2(s,reg,val,20);
	}

}


/*****************************************************************************/
/*	Write Si3000 control register - 2nd level 			     */
/*		(handles actual transfer, verification and retry) 	     */
/*****************************************************************************/
static void si3000_write_reg2( struct ibmcsi3k_state *s, int reg, unsigned char val, int retrycount) {
	int i;
	for (i=0; i<retrycount; i++) {
		si3000_transfer( s, reg, 0 , val , s->sync_interval ); 	/* Write it */
		if (val == si3000_transfer(s, reg, 1, 0, s->sync_interval ) ) /* Verify */
			return;
	}
      	DBG(printk("ibmcsi3k: Error: cannot write/verify Si3000 register %d\n", reg));
}

/*****************************************************************************/
/*	Set the Si3000 sampling rate	 				     */
/* 									     */
/*****************************************************************************/
static int si3000_set_sampling_rate(struct ibmcsi3k_state *s, int val) {
	int current_sampling_rate, i, j, retcode;
	unsigned int sync_value; 
	
	/* TODO: currently limited to default sampling rate */
	if (val != IBMCSI_DEFAULT_SAMPLING_RATE) return 0; 

	for (j=0; j<5; j++) { /* Arbitrary retry count */

		/* We need to know the current sampling rate in order to access the Si3000   */
		/* control registers, due to timing requirements of the CSI. 		     */
		/* So we can't simply read the control registers during init. 		     */

		for (i=0; i< 5; i++) { /* Another arbitrary retry count */ 
	 		if ((current_sampling_rate = si3000_get_sampling_rate(s))) break;
		}
		if (current_sampling_rate == 0) {
			printk(KERN_ERR "ibmcsi3k: cannot determine sampling rate \n");
			return 0; 
		}

		sync_value = ( (256 + 160 - 64) * 1000000 / ( current_sampling_rate * 256 ) );
		/* We need to hit the 160th clock tick in the next frame.		*/
		/* We sync at 64th clock (see si3000_synchronize), and there are 	*/
		/* 256 clocks per frame. We convert this to microseconds and pass it to */
		/* data transfer handler. 						*/
	
		printk("ibmcsi3k: current sampling rate %d Hz, sync delay %d us \n", 
					current_sampling_rate, sync_value);

		/* The PLL divider must be written first, and the multiplier written back to back. 	*/
		/* The routines are self-correcting, so any residual errors from previous boot will be	*/
		/* (supposedly) cleared by the first call. 						*/

		/* TODO: calculate multiplier / divider from given sampling rate. 	*/
		/* Current values are for 11.025 KHz.					*/
		si3000_transfer(s, SI3000_PLL1D, 0, 47, sync_value);
		si3000_transfer(s, SI3000_PLL1M, 0, 244, sync_value);

		udelay(1000);	/* Ugly but the CSI may not function correctly while    */
				/* si3000 PLL is re-locking and FSYNC is changing.	*/
				/* PLL lock time is guaranteed to be < 1 ms. 		*/

		/* Read the sampling rate again to recalculate sync delay value.	*/
		for (i=0; i<5; i++) {
			if ( (current_sampling_rate = si3000_get_sampling_rate(s))) break;
		}
		if (current_sampling_rate == 0) {
			printk(KERN_ERR "ibmcsi3k: cannot determine sampling rate \n");
			return 0; 
		}
		sync_value = ( (256 + 160 - 64) * 1000000 / ( current_sampling_rate * 256 ) );
		printk("ibmcsi3k: updated sampling rate %d Hz, sync delay %d us\n", 
				current_sampling_rate, sync_value);

		s->sync_interval = sync_value; 
		if ( ( si3000_read_reg(s, SI3000_PLL1D) == 47 ) &&
			( si3000_read_reg(s, SI3000_PLL1M) == 244 ) ) {
			printk("ibmcsi3k: sampling rate set successfully.\n"); 
			retcode = 1; 
			break;
		}
		else {
			retcode = 0; 
		}
	}

	return retcode; 
}


/*****************************************************************************/
/* 	Read the actual sampling rate of the Si3000			     */
/*									     */
/* 	We do two back to back dummy DMA transfers which end at a fixed      */
/*	offset from frame sync signal FSYNC, and measure the time it takes   */
/* 	for the second transfer to complete. 				     */
/*									     */
/*	This is necessary because the Si3000 register access requires	     */
/*	precise timing alignment w.r.t FSYNC and serial clock ticks. 	     */
/*									     */
/*****************************************************************************/
static int si3000_get_sampling_rate(struct ibmcsi3k_state *s) {
	unsigned long tbl_before, tbl_after, usecs;
   	int val = 0;

	ibmcsi3k_synchronize(s); 	/* First sync			*/
	tbl_before = get_tbl();		/* Read time base register low  */
	ibmcsi3k_synchronize(s);	/* Second sync 			*/
	tbl_after = get_tbl();		/* Read tbl again 		*/

	usecs = mulhwu(tb_to_us, tbl_after - tbl_before); /* Get usecs/frame */
	/* This conversion to usec is not very accurate...		*/  

	if (usecs) {
		val = 1000000/usecs;
	}
	return val;
}


/*****************************************************************************/
/*	Transfer data to and from Si3000 (polling mode)			     */
/*****************************************************************************/
static unsigned char si3000_transfer( struct ibmcsi3k_state *s,
	int reg, int read , unsigned char val , unsigned int sync_value) {

	unsigned long end_time;
	int timeout = 0;
	unsigned char command = reg;

	if (read) command |= 0x20;

	/* Prepare line buffer */
	(s->write_line)[0]  = 0x00;	/* Dummy value */
	(s->write_line)[1]  = 0x01;	/* Secondary frame request bit */
	(s->write_line)[2]  = 0x00;	/* Dummy value - ignored */
	(s->write_line)[3]  = 0x00;

	/* Register number (lower 5 bits) and bit 6 = read (1) / write (0) */
	(s->write_line)[4]  = command;
	/* Register value for write */
	(s->write_line)[5]  = val;
	(s->write_line)[6]  = 0x00;
	(s->write_line)[7]  = 0x00;

	(s->write_line)[8]  = 0x00;	/* Ditto above */
	(s->write_line)[9]  = 0x01;
	(s->write_line)[10] = 0x00;
	(s->write_line)[11] = 0x00;

	(s->write_line)[12] = command;
	(s->write_line)[13] = val;
	(s->write_line)[14] = 0x00;
	(s->write_line)[15] = 0x00;

	(s->write_line)[16] = 0x00;
	(s->write_line)[17] = 0x01;
	(s->write_line)[18] = 0x00;
	(s->write_line)[19] = 0x00;

	(s->write_line)[20] = command;
	(s->write_line)[21] = val;
	(s->write_line)[22] = 0x00;
	(s->write_line)[23] = 0x00;


	(s->write_line)[24] = 0x00;
	(s->write_line)[25] = 0x00;
	(s->write_line)[26] = 0x00;
	(s->write_line)[27] = 0x00;

	(s->write_line)[28] = 0x00;
	(s->write_line)[29] = 0x00;
	(s->write_line)[30] = 0x00;
	(s->write_line)[31] = 0x00;

	/* The above code is intentionally redundant. 			*/
	/* Initialization is done byte by byte in the event that   	*/
	/* this routine is ported to drive a little-endian codec chip.	*/

	ibmcsi3k_synchronize(s);

	/* Disable CSI */
	IBMCSI_WRITE(CSI0_ER, 0);

	/* Clear CSI overrun / underrun */
	IBMCSI_WRITE(CSI0_SR, CSI_SR_ROD | CSI_SR_TOD);

	/* Disable channels */
	mtdcr(IBMCSI_TXDMA_CR, 0);
	mtdcr(IBMCSI_RXDMA_CR, 0);

	/* Clear all Tx DMA channel/1 status register flags - Terminal Count etc. */
	mtdcr(IBMCSI_DMA_SR, DCRN_DMA_SR_ALL(IBMCSI_TXDMA) | DCRN_DMA_SR_ALL(IBMCSI_RXDMA));

	/* Clear any pending internal DMA request that may be stuck due to premature termination etc. */
   	/* 	(Writing a '1' to the status register will not clear such a state.)	*/
	if (mfdcr(IBMCSI_DMA_SR) & (DCRN_DMA_SR_IR(IBMCSI_TXDMA) | DCRN_DMA_SR_IR(IBMCSI_RXDMA)) ) {
		mtdcr(IBMCSI_TXDMA_SA, (unsigned) s->dma_write_line + 28);
		mtdcr(IBMCSI_TXDMA_DA, CSI0_TBUF);
		mtdcr(IBMCSI_RXDMA_SA, CSI0_RBUF);
		mtdcr(IBMCSI_RXDMA_DA, (unsigned) s->dma_read_line + 28);
		mtdcr(IBMCSI_TXDMA_CT, 2);
		mtdcr(IBMCSI_RXDMA_CT, 2);
		mtdcr(IBMCSI_TXDMA_CR, IBMCSI_TXDMA_GO_NOI ); /* No int */
		mtdcr(IBMCSI_RXDMA_CR, IBMCSI_RXDMA_GO_NOI );	/* No int */

		udelay(2); /* Paranoia */

		mtdcr(IBMCSI_TXDMA_CR, 0);
		mtdcr(IBMCSI_RXDMA_CR, 0);
	}

	/* Set Tx DMA channel source address */
	mtdcr(IBMCSI_TXDMA_SA, (unsigned)(s->dma_write_line));

	/* Set Rx DMA channel SOURCE address */
	mtdcr(IBMCSI_RXDMA_SA, CSI0_RBUF);

	/* Set Tx DMA channel count */
	mtdcr(IBMCSI_TXDMA_CT, 2);

	/* Set Rx DMA channel count */
	mtdcr(IBMCSI_RXDMA_CT, 2);

	/* Set Tx DMA channel destination address */
	mtdcr(IBMCSI_TXDMA_DA, CSI0_TBUF);

	/* Set Rx DMA channel DESTINATION address */
	mtdcr(IBMCSI_RXDMA_DA, (unsigned)(s->dma_read_line));

	/* Enable DMA0 with interrupt disable */
	mtdcr(IBMCSI_TXDMA_CR, IBMCSI_TXDMA_GO_NOI );

	/* Enable DMA1 with interrupt disable */
	mtdcr(IBMCSI_RXDMA_CR, IBMCSI_RXDMA_GO_NOI );

	asm volatile ("sync"); 	/* Sync */

	/* Synchronize CSI enable to FSYNC */

#if 1
	/* udelay( (2560/28) + sync_value + 5 ); */ /* Works if OpenBIOS has set it up */ 
	udelay(sync_value); 

#ifdef TIMINGDEBUGPROBE
	__raw_writel(0x30600000, 0xef600a00);
#endif

	IBMCSI_WRITE(CSI0_CFG,	0x00200000);	/* 128 bit frame size */
	IBMCSI_WRITE(CSI0_ER, 	CSI_ER_ESLOT(0) |
			CSI_ER_RXEN | CSI_ER_TXEN | CSI_ER_ECSI );
	/* This must be done between 160th and 224th serial clock ticks */

#else	/* 256 bit frame size mode */
	udelay(150);
	IBMCSI_WRITE(CSI0_CFG, 0);
   	IBMCSI_WRITE(CSI0_ER, CSI_ER_ESLOT(0) | CSI_ER_ESLOT(8) |
			CSI_ER_RXEN | CSI_ER_TXEN | CSI_ER_ECSI );
	/* This method is unsupported, but is known to work after one frame */
#endif

	end_time = jiffies + HZ/50;

	while ( !(mfdcr(IBMCSI_DMA_SR) & DCRN_DMA_SR_CS(IBMCSI_RXDMA)) ) {
		if (jiffies > end_time) {
			timeout = 1;
			break;
		}
	} /* Wait until Rx DMA channel Terminal Count is posted or timeout */

	/* Stop CSI */
	IBMCSI_WRITE(CSI0_ER, 0);

	IBMCSI_WRITE(CSI0_TBUF, 0); 	/* Reset tx buffer data to prevent noise */

	/* Stop Tx/Rx DMA */
	mtdcr(IBMCSI_TXDMA_CR, 0);
	mtdcr(IBMCSI_RXDMA_CR, 0);

	/* Clear Tx/Rx DMA status */
	mtdcr(IBMCSI_DMA_SR, DCRN_DMA_SR_ALL(IBMCSI_TXDMA) | DCRN_DMA_SR_ALL(IBMCSI_RXDMA));

	if (!timeout) {
		s->timeout = 0;
		return ((s->read_line)[5]);
	}
     	else {
		s->timeout = 1; 	/* Error return 			*/
		return 0;
     	}

}


/*****************************************************************************/
/* 	Synchronize code execution to Si3000 FSYNC signal 		     */
/*		         						     */
/*	This is necessary to precisely synchronize the timing of CSI enable  */
/*	for the 128 bits per frame mode, in which the CSI must be enabled    */
/*	between the 160th and 224th serial ticks since FSYNC. 		     */
/*									     */
/*	This routine does a dummy Rx DMA of 2 samples and polls for DMA	     */
/*	completion, which should happen immediately following the 64th tick. */
/* 									     */
/*****************************************************************************/
static void ibmcsi3k_synchronize(struct ibmcsi3k_state *s) {
   	unsigned int timeout;

	IBMCSI_WRITE(CSI0_ER, 0);
	IBMCSI_WRITE(CSI0_CFG, 0);

	/* Clear CSI overrun / underrun */
	IBMCSI_WRITE(CSI0_SR, CSI_SR_TOD | CSI_SR_ROD );

	/* Disable channels */
	mtdcr(IBMCSI_TXDMA_CR, 0);
	mtdcr(IBMCSI_RXDMA_CR, 0);

	/* Clear Terminal Count etc. */
	mtdcr(IBMCSI_DMA_SR, 	DCRN_DMA_SR_ALL(IBMCSI_TXDMA) |
				DCRN_DMA_SR_ALL(IBMCSI_RXDMA) );

	mtdcr(IBMCSI_RXDMA_SA, CSI0_RBUF);
	mtdcr(IBMCSI_RXDMA_DA, (unsigned) s->dma_read_line + 64);

	mtdcr(IBMCSI_RXDMA_CT, 2);		/* Do 2 reads for good measure */

	mtdcr(IBMCSI_RXDMA_CR, IBMCSI_RXDMA_GO_NOI ); /* No interrupts */

	IBMCSI_WRITE(CSI0_CFG, 0); 		/* 256 bpf mode */

	IBMCSI_WRITE(CSI0_ER, 	CSI_ER_ESLOT(0) | CSI_ER_ESLOT(1) |
				CSI_ER_ESLOT(2) | CSI_ER_ESLOT(3) |
				CSI_ER_RXEN | CSI_ER_ECSI );

	timeout = jiffies + HZ/50;
	while( !( mfdcr(IBMCSI_DMA_SR) & DCRN_DMA_SR_CS(IBMCSI_RXDMA) ) ) {
   		if (jiffies > timeout) {
			printk("ibmcsi3k: Error: DMA timeout\n");
         		break;
	        }
   	} /* Read until DMA Channel 1 terminal count is posted or timeout.   */

#ifdef TIMINGDEBUGPROBE	/* Timing probe for debug - toggles TPC VDDX charge */
	__raw_writel(0xB2E54000, 0xef600a00);
#endif

	IBMCSI_WRITE(CSI0_ER, 0);	/* Disable CSI */

	mtdcr(IBMCSI_RXDMA_CR, 0);	/* Disable Rx DMA channel */

}
