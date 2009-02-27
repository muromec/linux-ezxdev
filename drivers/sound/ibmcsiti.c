/*****************************************************************************/
/*
 *      ibmcsiti.c : IBM PPC 405LP Codec Serial Interface (CSI) +
 *                      Texas Instruments TLV320AIC23 stereo audio codec
 *                      for the Arctic-II reference board
 *
 *	Based on the ibmcsiti driver for IBM PPC 405LP CSI + TLV320AIC23 codec
 *
 *      Copyright (C) 2002 Ken Inoue and David Gibson, IBM Corporation
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
 *
 *  Supported devices:
 *  	/dev/dsp    standard /dev/dsp device, hopefully OSS compatible
 *  	/dev/mixer  standard /dev/mixer device, hopefully OSS compatible
 *
 * TODOs:
 * - Integration testing with ViaVoice embedded edition
 * - Sampling rate is fixed at 44.1 KHz.
 * - Sample format is limited to 16 bit big endian.
 *	(this is a deviation since an OSS DSP device is supposed to support 8 bit as default.)
 * - Drain DAC/ADC
 * - Fragment handling
 * - MMAP support
 * - Split CSI and codec drivers
 * - Module parameters
 * - Tune retry counts and jiffies
 * - Revisit inline functions
 * - Write ibmcsi.txt in the Documentation directory
 * - Anything else?
 */

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
#include <linux/i2c.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <asm/delay.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <asm/hardirq.h>
#include <asm/time.h>
#include <asm/ocp.h>

#include <platforms/ibm405lp.h>

#include "tlv320aic23.h"

/*****************************************************************************/
/* Start #defines that might belong in ibm405lp.h */

#define CSI0_IO_BASE 0xEF600900 /* CSI I/O base address */

#define CSI0_ER	        (CSI0_IO_BASE + 0)	/* Enable register */
#define CSI0_CFG	(CSI0_IO_BASE + 4)	/* Configuration register */
#define CSI0_SR         (CSI0_IO_BASE + 8)	/* Status register */
#define CSI0_TBUF	(CSI0_IO_BASE + 0x0C)	/* Transmit buffer */
#define CSI0_RBUF   	(CSI0_IO_BASE + 0x10)	/* Receive buffer */
#define CSI0_ID		(CSI0_IO_BASE + 0x14)	/* ID */
#define CSI0_SCR	(CSI0_IO_BASE + 0x18)	/* Sleep control register */

#define CSI0_EXTENT 28			/* I/O address extent */
#define CSI_ID_405LP	0x1107		/* CSI core ID (halfword) */


#define CSI_ER_ECSI	0x00000001	/* Enable the CSI */
#define CSI_ER_RXEN	0x00000002	/* Receive enable */
#define CSI_ER_TXEN	0x00000004	/* Transmit enable */

#define CSI_ER_ESLOT(n) (0x80000000 >> (n)) /* Enable (n)th slot */

#define CSI_SR_TOD	0x00000001	/* Transmit overrun */
#define CSI_SR_ROD	0x00000002	/* Receive overrun */
#define CSI_SR_CSIB	0x00008000	/* CSI Busy */

/************************************************************************/
/* DMA Channel Control register						*/
/* (DCRN_DMA0_CR0 through DCRN_DMA0_CR3)				*/
/************************************************************************/

#define DCRN_DMA_CR_CE                 0x80000000   /* Channel Enable */
#define DCRN_DMA_CR_CIE                0x40000000   /* Channel Interrupt Enable */
#define DCRN_DMA_CR_TD                 0x20000000   /* Transfer Direction */
#define DCRN_DMA_CR_PL                 0x10000000   /* Peripheral Location */
#define DCRN_DMA_CR_PW_8               0x00000000   /* 8 bit peripheral width */
#define DCRN_DMA_CR_PW_16              0x04000000   /* 16 bit peripheral width */
#define DCRN_DMA_CR_PW_32              0x08000000   /* 32 bit peripheral width */
#define DCRN_DMA_CR_PW_64              0x0c000000   /* 64 bit peripheral width */
#define DCRN_DMA_CR_DAI                0x02000000   /* Destination Addr Increment */
#define DCRN_DMA_CR_SAI                0x01000000   /* Source Address Increment */
#define DCRN_DMA_CR_BE                 0x00800000   /* Buffer Enable */
#define DCRN_DMA_CR_TM_BUFFERED        0x00000000   /* Buffered Transfer mode */
#define DCRN_DMA_CR_TM_SW_MEM_TO_MEM   0x00400000   /* Software started mem to mem */
#define DCRN_DMA_CR_TM_HW_MEM_TO_MEM   0x00600000   /* Hardware paced mem to mem */
#define DCRN_DMA_CR_PSC_0              0x00000000   /* 0 Peripheral Setup Cycles */
#define DCRN_DMA_CR_PSC_1              0x00080000   /* 1 Peripheral Setup Cycles */
#define DCRN_DMA_CR_PSC_2              0x00100000   /* 2 Peripheral Setup Cycles */
#define DCRN_DMA_CR_PSC_3              0x00180000   /* 3 Peripheral Setup Cycles */
#define DCRN_DMA_CR_PSC(n) (((n)&0x3)<<19)          /* n Peripheral setup cycles */
#define DCRN_DMA_CR_PWC(n) (((n)&0x3f)<<13)         /* n peripheral wait cycles */
#define DCRN_DMA_CR_PHC(n) (((n)&0x7)<<10)          /* n peripheral hold cycles   */
#define DCRN_DMA_CR_ETD                0x00000200   /* EOT/TC Pin Direction */
#define DCRN_DMA_CR_TCE                0x00000100   /* Terminal Count Enable */
#define DCRN_DMA_CR_CP_MASK            0x000000C0   /* Channel Priority */
#define DCRN_DMA_CR_CP(n) (((n)&0x3)<<6)
#define DCRN_DMA_CR_PF                 0x00000030   /* Memory read prefetch trans */
#define DCRN_DMA_CR_PF_1               0x00000000   /*   Prefetch 1 dword */
#define DCRN_DMA_CR_PF_2               0x00000010   /*   Prefetch 2 dword */
#define DCRN_DMA_CR_PF_4               0x00000020   /*   Prefetch 4 dword */
#define DCRN_DMA_CR_PCE                0x00000008   /* Parity check enable */
#define DCRN_DMA_CR_DEC                0x00000004   /* Address decrement */

/************************************************************************/
/* DMA Status Register							*/
/* (Device Control Register bus register DCRN_DMASR)			*/
/************************************************************************/
/* (n) = DMA channel number, 0-3 */

#define DCRN_DMA_SR_CS(n)	(0x80000000 >>(n))  /* Terminal count status */
#define DCRN_DMA_SR_TS(n)       (0x08000000 >>(n))  /* End Of Transfer status */
#define DCRN_DMA_SR_RI(n)	(0x00800000 >>(n))  /* Error status */
#define DCRN_DMA_SR_IR(n)	(0x00080000 >>(n))  /* Internal DMA request pending */
#define DCRN_DMA_SR_ER(n)	(0x00008000 >>(n))  /* External DMA request pending */
#define DCRN_DMA_SR_CB(n)	(0x00000800 >>(n))  /* Channel Busy */
#define DCRN_DMA_SR_SG(n)	(0x00000080 >>(n))  /* Scatter/gather status */

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
	u32 ccw; /* Channel Control Word */
	u32 srcP;	/* Source address (physical) */
	u32 destP;	/* Destination address (physical) */
	u32 ctrl; /* MSB = link bit, lower halfword = count */
		/* Other 3 bits unused */ 
	u32 nextP;	/* Next scatter/gather descriptor list physical address */
	/* ------------------------------------- Private use ---------------*/
	struct dma_sgdt *prevV;	/* Prev scatter/gather descriptor list virtual address */
	struct dma_sgdt *nextV; /* Next */
	unsigned int dummy;	/* Reserved (for 16 byte alignment) */ 
}; 

/* End ibm405lp.h candidates */


/*****************************************************************************/
/* Driver specific defines 						     */
/*****************************************************************************/

/* The DMA channels for the CSI are hardcoded in the 405LP chip, so we
 * hardcode them.  If a future chip adopts programmable channel
 * assignment, I expect access to DMA channels would be handled by a
 * separate driver.
 */

#define IBMCSI_TXDMA 	0	/* Transmit from CSI to codec : channel 0 */
#define IBMCSI_RXDMA	1	/* Receive from codec to CSI  : channel 1 */

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


#define IBMCSI_TXDMA_CONFIG (	    	/* Channel disabled */ \
		DCRN_DMA_CR_CIE |    	/* Channel interrupt enabled */ \
				    	/* Memory to peripheral */ \
      		DCRN_DMA_CR_PL	|   	/* Peripheral on OPB */ \
		DCRN_DMA_CR_PW_32 |  	/* 32 bit wide peripheral */ \
					/* Dest address not incremented */ \
		DCRN_DMA_CR_SAI	| 	/* Source address incremented */ \
					/* Peripheral transfer mode */ \
					/* Peripheral setup cycle 0 */ \
		DCRN_DMA_CR_PWC(2) |	/* Peripheral wait cycle 3 */ \
					/* Peripheral hold cycle 0 */ \
		DCRN_DMA_CR_ETD |	/* EOTn = TC */ \
		DCRN_DMA_CR_TCE )	/* Terminal count enable */

#define IBMCSI_TXDMA_GO (IBMCSI_TXDMA_CONFIG | DCRN_DMA_CR_CE)	 /* For int */
#define IBMCSI_TXDMA_GO_NOI (IBMCSI_TXDMA_GO & ~DCRN_DMA_CR_CIE) /* For polling	*/

#define IBMCSI_RXDMA_CONFIG (		/* Channel disabled */ \
		DCRN_DMA_CR_CIE |    	/* Channel interrupt enabled */ \
		DCRN_DMA_CR_TD  |   	/* Peripheral to memory */ \
		DCRN_DMA_CR_PL	|   	/* Peripheral on OPB */ \
		DCRN_DMA_CR_PW_32 |  	/* 32 bit wide peripheral */ \
		DCRN_DMA_CR_DAI |   	/* Dest address incremented */ \
		                   	/* Source address not incremented */ \
				    	/* Peripheral transfer mode */ \
				    	/* Peripheral setup cycle 0 */ \
		DCRN_DMA_CR_PWC(2) |	/* Peripheral wait cycle 3 */ \
			            	/* Peripheral hold cycle 0 */ \
		DCRN_DMA_CR_ETD |   	/* EOTn = TC */ \
		DCRN_DMA_CR_TCE ) 	/* Terminal count enable */

#define IBMCSI_RXDMA_GO (IBMCSI_RXDMA_CONFIG | DCRN_DMA_CR_CE)
#define IBMCSI_RXDMA_GO_NOI (IBMCSI_RXDMA_GO & ~DCRN_DMA_CR_CIE)

#define IBMCSI_DEFAULT_SAMPLING_RATE 44100

#define IBMCSI_TI_CFG 0x00400010		 /* 64 bits per frame mode */

#define DAC_TIMER_PERIOD (HZ/50)
#define ADC_TIMER_PERIOD (HZ/50)

#define TX_SG DCRN_ASG0
#define RX_SG DCRN_ASG1

#define TX_SG_ENABLE  	DCRN_DMA_SGC_SSG0
#define TX_SG_MASK 	DCRN_DMA_SGC_EM0

#define RX_SG_ENABLE  	DCRN_DMA_SGC_SSG1
#define RX_SG_MASK 	DCRN_DMA_SGC_EM1


/*****************************************************************************/

#undef OSS_DOCUMENTED_MIXER_SEMANTICS /* FIXME: does this have any effect? */

#define DBG(x) {}
/*#define DBG(x) {x}*/

#define IBMCSI_MAGIC 0xB31BCB /* Copied from the Austin Research version */
/* TODO: verify this value is legitimate */

#define DMABUF_DEFAULTORDER (16-PAGE_SHIFT) /* FIXME: check out this value */
#define DMABUF_MINORDER 1			/* FIXME: ditto */

#define IBMCSI_WRITE(reg, val) (__raw_writel(val, reg))
#define IBMCSI_READ(reg)	(__raw_readl(reg))

#define VALIDATE_STATE(s)				\
({							\
	if (!(s) || (s)->magic != IBMCSI_MAGIC) {	\
		printk(KERN_ERR "ibmcsi: invalid signature (magic) in private data\n");\
		return -ENXIO;				\
	}						\
})


/*****************************************************************************/
/* Static variables, globals and structs 				     */
/*****************************************************************************/

static const unsigned sample_shift[] = {0, 1, 1, 2};
static LIST_HEAD(devs);

/*
 * Private data structure for the devices supported by this driver
 */
struct ibmcsiti_state {
	unsigned int magic; /* Magic signature value for sanity check */

   	unsigned int state; /* Driver state (DAC/ADC running, Halt, etc.) */

	struct i2c_client *i2c;

 	/* For multi-device support; not used for the 405LP */
   	struct list_head devs;

	/* DSP device variables */
	int dev_dsp; /* unit number of our registered DSP device */
	int outstereo; /* are we in stereo output mode? */

	spinlock_t lock;

	struct semaphore dsp_sem;
	struct semaphore open_sem;

	mode_t open_mode;
	wait_queue_head_t open_wait;

	/* Mixer device variables */
	int dev_mixer; /* unit number of our registered mixer device */
	struct semaphore mix_sem;

   	/* Buffers */
   	unsigned char *write_line;
	dma_addr_t dma_write_line;
	unsigned char *read_line;
	dma_addr_t dma_read_line;

	/* Control blocks for audio playback (dac) and capture (adc) */
	struct dmabuf {
		/* The important ones... */
	      	void *rawbuf;		/* DMA buffer logical address */
		dma_addr_t dmaaddr;	/* DMA buffer physical address */
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
		unsigned error; 		/* Over/underrun */
		unsigned sg_count;
	} dma_dac, dma_adc;

	/* FIXME: should move the following to dma_dac/dma_adc (trivial) */
	struct timer_list dac_timer;
	struct timer_list adc_timer;
	struct dma_sgdt *dac_free_sgdt_q;
	struct dma_sgdt *adc_free_sgdt_q;
	struct dma_sgdt *dac_active_sgdt_q;
	struct dma_sgdt *adc_active_sgdt_q;

	struct dma_sgdt *dac_sgdt_lastV; /* Anchors */
	struct dma_sgdt *adc_sgdt_lastV;

	struct dma_sgdt *adc_hw_prev_sgdt; 
	struct dma_sgdt *adc_sw_prev_sgdt;

	/* Local copy of TI codec settings (the registers being write only) */
	u16 codec_reg[TLV320_REG_EXTENT]; /* the registers are
					   * actually 9-bits each */
};

/* Driver state flags */
#define IBMCSI_DAC_RUNNING 	0x00010000
#define IBMCSI_ADC_RUNNING	0x00020000
#define IBMCSI_HALT		0x00040000

/************************************************************************/
/* Misc function prototypes						*/
/************************************************************************/

static int __init init_ibmcsiti(void);		/* Driver initialization */
static void __exit cleanup_ibmcsiti(void); 	/* Driver exit cleanup */

static int ibmcsi_i2c_attach_adapter(struct i2c_adapter *);
static int ibmcsi_i2c_detach_client(struct i2c_client *);
static int ibmcsi_i2c_detect_client(struct i2c_adapter *, int address,
				    unsigned short flags, int kind);
static void ibmcsi_i2c_inc_use(struct i2c_client *);
static void ibmcsi_i2c_dec_use(struct i2c_client *);

/************************************************************************/
/* DSP driver function prototypes					*/
/************************************************************************/

/* Top level */
static int ibmcsiti_dsp_open(struct inode *inode, struct file *file);
static ssize_t ibmcsiti_dsp_read(struct file *file, char *buffer,
				 size_t count, loff_t *ppos);
static ssize_t ibmcsiti_dsp_write(struct file *file, const char *buffer,
				  size_t count, loff_t *ppos);
static unsigned int ibmcsiti_dsp_poll(struct file *file,
				      struct poll_table_struct *wait);
static int ibmcsiti_dsp_release(struct inode *inode, struct file *file);
static int ibmcsiti_dsp_ioctl(struct inode *inode, struct file *file,
			      unsigned int cmd, unsigned long arg);

/* Interrupt handlers */
static void ibmcsiti_dac_interrupt(int irq, void *dev_id,
				   struct pt_regs *regs);
static void ibmcsiti_adc_interrupt(int irq, void *dev_id,
				   struct pt_regs *regs);
static void ibmcsi_adc_timer(unsigned long param);
static void ibmcsi_dac_timer(unsigned long param);

/* Utility routines */
static unsigned long copy_samples_to_user(char *dest, const char *src,
					  unsigned nsamples, int stereo);
static unsigned long copy_samples_from_user(char *dest, const char *src,
					    unsigned nsamples, int stereo);

static void start_adc(struct ibmcsiti_state *s);
static void start_dac(struct ibmcsiti_state *s);
static int drain_dac(struct ibmcsiti_state *s, int nonblock);
static inline void stop_adc(struct ibmcsiti_state *s);
static inline void stop_dac(struct ibmcsiti_state *s);

static int ibmcsi_stop_csi_sync(void);

static inline void dealloc_dmabuf(struct ibmcsiti_state *s,
				  struct dmabuf *buf);
static int prog_dmabuf(struct ibmcsiti_state *s, struct dmabuf *buf,
		       unsigned rate, unsigned fmt,unsigned adc_init);
static inline int prog_dmabuf_adc(struct ibmcsiti_state *s);
static inline int prog_dmabuf_dac(struct ibmcsiti_state *s);

static inline unsigned get_hwptr(struct ibmcsiti_state *s, struct dmabuf *buf,
				 unsigned channel);
static void ibmcsiti_update_ptr(struct ibmcsiti_state *s);

static inline void clear_advance(void *buf, unsigned bsize, unsigned bptr,
				 unsigned len, unsigned char c);

static inline unsigned ld2(unsigned int x);

/* Scatter/Gather descriptor table maintenance routines */

static void init_sgdt_q(struct dma_sgdt *queue, int count);
static struct dma_sgdt *get_sgdt(struct dma_sgdt **queueaddress);
static void free_sgdt(struct dma_sgdt **queueaddress, struct dma_sgdt *dt);
static unsigned int check_sgdt_range(struct ibmcsiti_state *s,
				     struct dma_sgdt* dt, int count);

/************************************************************************/
/* Mixer driver function prototypes					*/
/************************************************************************/

/* Top level */
static int ibmcsiti_mixer_open(struct inode *inode, struct file *file);
static int ibmcsiti_mixer_ioctl(struct inode *ioctl, struct file *file,
				 unsigned int cmd, unsigned long arg);
static int ibmcsiti_mixer_release(struct inode *inode, struct file *file);

static int mixer_read_ioctl(struct ibmcsiti_state *s, unsigned int nr,
			    caddr_t arg);
static int mixer_write_ioctl(struct ibmcsiti_state *s, unsigned int nr,
			     caddr_t arg);

/* Mixer read routines */
static int ibmcsiti_get_volume(struct ibmcsiti_state *s) ;
static int ibmcsiti_get_line(struct ibmcsiti_state *s) ;
static int ibmcsiti_get_mic(struct ibmcsiti_state *s) ;
static int ibmcsiti_get_recsrc(struct ibmcsiti_state *s) ;
static int ibmcsiti_get_outsrc(struct ibmcsiti_state *s) ;

/* Mixer write routines */
static int ibmcsiti_set_volume(struct ibmcsiti_state *s, int val);
static int ibmcsiti_set_line(struct ibmcsiti_state *s, int val) ;
static int ibmcsiti_set_mic(struct ibmcsiti_state *s, int val) ;
static int ibmcsiti_set_recsrc(struct ibmcsiti_state *s, int val) ;

/************************************************************************/
/* Helper functions							*/
/************************************************************************/

/* TLV320AIC23 control register access routines */
static u16 tlv320_read_reg(struct ibmcsiti_state *s, int reg);
static void tlv320_write_reg(struct ibmcsiti_state *s, int reg,
			       u16 val);

/************************************************************************/

MODULE_AUTHOR("David Gibson");
MODULE_DESCRIPTION("IBM PPC 405LP CSI / TI TLV320AIC23 Audio Driver");

#if 1  /* linux-pm */
#include <linux/device.h>

static int ibmcsiti_suspend(struct device * dev, u32 state, u32 level);
static int ibmcsiti_resume(struct device * dev, u32 level);
static int ibmcsiti_scale(struct bus_op_point * op, u32 level);

static struct device_driver ibmcsiti_driver_ldm = {
       name:      	"IBM_CSI_TI_audio",
       devclass:  	NULL,
       probe:     	NULL,
       suspend:   	ibmcsiti_suspend,
       resume:    	ibmcsiti_resume,
       scale:	  	ibmcsiti_scale,
       remove:    	NULL,
};

static struct device ibmcsiti_device_ldm = {
       name:		"Audio Codec ",
       bus_id:		"ibmcsiti",
       driver: 		NULL,
       power_state:	DPM_POWER_OFF,
};

static void ibmcsiti_ldm_register(void)
{
	extern void opb_driver_register(struct device_driver *driver);
	extern void opb_device_register(struct device *device);

	opb_driver_register(&ibmcsiti_driver_ldm);
	opb_device_register(&ibmcsiti_device_ldm);
}

static void ibmcsiti_ldm_unregister(void)
{
	extern void opb_driver_unregister(struct device_driver *driver);
	extern void opb_device_unregister(struct device *device);

	opb_driver_unregister(&ibmcsiti_driver_ldm);
	opb_device_unregister(&ibmcsiti_device_ldm);
}

static int ibmcsiti_scale(struct bus_op_point * op, u32 level)
{
	/* linux-pm-TODO */

	return 0;
}

/* Note: Only scatter/gather mode DMA is supported. */
 
static u32 ibm_csi_hardware_registers[3];
static void *ibm_csi_base;
enum { CSI0_ER_reg, CSI0_CFG_reg, CSI0_SCR_reg};

static struct dma_sgdt *ibm_dma_Tx_descriptor;
static struct dma_sgdt *ibm_dma_Rx_descriptor;

#define CSI_ER_OFFSET 	(CSI0_ER 	- CSI0_IO_BASE) 
#define CSI_CFG_OFFSET	(CSI0_CFG 	- CSI0_IO_BASE)
#define CSI_SR_OFFSET	(CSI0_SR 	- CSI0_IO_BASE)
#define CSI_SCR_OFFSET	(CSI0_SCR 	- CSI0_IO_BASE)

static void ibmcsi_save_state(struct ibmcsiti_state *s)
{
	/* We will do this 99% based on the hardware state. */
	/* "In Hardware We Trust" */

	u32 tx_cr, rx_cr; 

	/* Make sure the core is powered up. */
	ppc4xx_cpm_fr(CPM_CSI, 0);
 
	ibm_csi_base = ioremap((long) CSI0_IO_BASE, 0x1B);
	if ( ibm_csi_base == NULL ) {
    		printk(KERN_ERR "dpm_save_ibmcsi_state: ioremap failed\n");
		return;
	}

	/* Just save / restore CSI if not enabled. */ 
	ibm_csi_hardware_registers[CSI0_ER_reg]	= __raw_readl(ibm_csi_base + CSI_ER_OFFSET);
	ibm_csi_hardware_registers[CSI0_CFG_reg] = __raw_readl(ibm_csi_base + CSI_CFG_OFFSET);
	ibm_csi_hardware_registers[CSI0_SCR_reg] = __raw_readl(ibm_csi_base + CSI_SCR_OFFSET);

   	if ( (__raw_readl(ibm_csi_base + CSI_ER_OFFSET) & CSI_ER_ECSI) == 0) {
		return; 	
	}   		
	    	
    	/* OK, either DAC or ADC is in progress. The fun begins. */
 
    	/* Stop CSI first, then DMAC. */ 
	__raw_writel( __raw_readl(ibm_csi_base + CSI_ER_OFFSET) & ~CSI_ER_ECSI,
		ibm_csi_base + CSI_ER_OFFSET);
	iobarrier_rw();   
    	/* Wait until the current codec frame ends. 		*/
	udelay(1000); /* 1ms should be enough for all frame rates. */
 
	/* We will be restarting only after a POR, so we don't  *
	 * have to be too neat and tidy about leaving the chip  *
	 * in good state like we do elsewhere in stop_csi_sync. *
	 * In fact, we don't really want to be too picky about	*
	 * the state of hardware here, because people may wish 	*
	 * to use suspend / resume to clear some (as yet 	*
	 * unforseen) error condition.          		*/
		
	
	/* In theory disabling CSI should stop any DMA transfer	*/ 
	/* in progress. We explicitly stop DMA for good measure */ 
	mtdcr(DCRN_ASGC, 
		( mfdcr(DCRN_ASGC) & ~(TX_SG_ENABLE | RX_SG_ENABLE) ) 
				   |  (TX_SG_MASK | RX_SG_MASK) );
	tx_cr = mfdcr(IBMCSI_TXDMA_CR);
	rx_cr = mfdcr(IBMCSI_RXDMA_CR);

	mtdcr(IBMCSI_TXDMA_CR, tx_cr & ~DCRN_DMA_CR_CE);
	mtdcr(IBMCSI_RXDMA_CR, rx_cr & ~DCRN_DMA_CR_CE); 
	
	udelay(1000); 
		
	if ( (__raw_readl(ibm_csi_base + CSI_ER_OFFSET) & CSI_ER_TXEN) ) {
    	/* Saving the state for DAC (Playback) */
		/* See if we are at the bottom of the queue.			*
		 * We can skip the following if we are spinning at the anchor	* 
		 * (underrun) 							*/
		struct dma_sgdt *pCurrent = phys_to_virt(mfdcr(TX_SG)); 
		
		if (pCurrent == NULL) goto suspend_Rx; 
	
		if ( (mfdcr(IBMCSI_TXDMA_SA) == pCurrent->srcP) && 
		     (pCurrent->srcP == virt_to_phys(&(pCurrent->dummy))) ) {
		/* The current descriptor is in effect, and the source is dummy */
			printk("ibmcsiti: Dummy anchor descriptor found\n");  
			ibm_dma_Tx_descriptor = pCurrent; 
			goto suspend_Rx; 
		}
		printk("ibmcsiti: Normal descriptor found\n"); 

    		/* Get hold of a new scatter/gather descriptor.  */

		/* WIBNI the DMA core kept a copy of the CURRENT descriptor	*/  
		ibm_dma_Tx_descriptor = get_sgdt(&s->dac_free_sgdt_q);
		if (ibm_dma_Tx_descriptor == NULL) {
    			printk(KERN_ERR "dpm_save_ibmcsi_state: get Tx descriptor failed\n");
			ibm_dma_Tx_descriptor = phys_to_virt(mfdcr(TX_SG)); 
			goto suspend_Rx;
		}

    		/* Populate the new scatter/gather descriptor for restart on resume. */
		ibm_dma_Tx_descriptor->ccw = tx_cr;
		ibm_dma_Tx_descriptor->srcP = mfdcr(IBMCSI_TXDMA_SA);
		ibm_dma_Tx_descriptor->destP = mfdcr(IBMCSI_TXDMA_DA);
		ibm_dma_Tx_descriptor->ctrl = mfdcr(IBMCSI_TXDMA_CT) | 0x80000000; /* Link bit */
		ibm_dma_Tx_descriptor->nextP = mfdcr(TX_SG); 
		ibm_dma_Tx_descriptor->prevV = 0; 
		ibm_dma_Tx_descriptor->nextV = phys_to_virt(mfdcr(TX_SG)); 	

		/* Link it to the "current" descriptor. */ 
		((struct dma_sgdt *)ibm_dma_Tx_descriptor->nextV)->prevV = 
			ibm_dma_Tx_descriptor;
 
		/* Fingers crossed */ 
	}

suspend_Rx: 
	if ( (__raw_readl(ibm_csi_base + CSI_ER_OFFSET) & CSI_ER_RXEN) ) { 
    	/* Saving the state for ADC (Capture) */
		struct dma_sgdt *pWork; 
		/* We simply backtrack one descriptor and populate it. */ 
		pWork = (struct dma_sgdt *)phys_to_virt(mfdcr(RX_SG));
		if (pWork->prevV) {
			ibm_dma_Rx_descriptor = pWork->prevV; 
			ibm_dma_Rx_descriptor->ccw = rx_cr;
			ibm_dma_Rx_descriptor->srcP = mfdcr(IBMCSI_RXDMA_SA);
			ibm_dma_Rx_descriptor->destP = mfdcr(IBMCSI_RXDMA_DA);
			ibm_dma_Rx_descriptor->ctrl = mfdcr(IBMCSI_RXDMA_CT) | 0x80000000; 
			ibm_dma_Rx_descriptor->nextP = mfdcr(RX_SG);
			ibm_dma_Rx_descriptor->prevV = 0; 
			ibm_dma_Rx_descriptor->nextV = phys_to_virt(mfdcr(RX_SG)); 
		}
		else {
    			printk(KERN_ERR "dpm_save_ibmcsi_state: Rx descriptor chain corupted\n");
			ibm_dma_Rx_descriptor = 0; 
		}
	} 
}

static void ibmcsi_restore_state(struct ibmcsiti_state *s)
{


	/* Reinitialize the codec. I2C is a prerequisite. */
 
	/* Set up TI codec via direct I2C manipulation. */
	/* The control registers are write-only, so we have to write back 
	 * the shadow registers in a certain sequence. */ 
	{
		int i; 
 		int i2c_error = 0;
		u16 i2c_data[] = {	
			TLV320_REG_WRITE(TLV320_RR,  0x0000), /* 0x1e00: Reset chip */
			TLV320_REG_WRITE(TLV320_DIA, 0x0000), /* 0x1200: Deactivate digital interface */
			TLV320_REG_WRITE(TLV320_DAI, 0x0053), /* 0x0e53: Master, MSB on 2nd BCLK, DSP format */
			TLV320_REG_WRITE(TLV320_PDC, 0x0100), /* 0x0d00: Power up everything */
			TLV320_REG_WRITE(TLV320_DIA, 0x0001), /* 0x1201: Activate digital interface */
			TLV320_REG_WRITE(TLV320_LLI, 0x0017), /* 0x0017: Left Line-In volume 0dB */
			TLV320_REG_WRITE(TLV320_RLI, 0x0017), /* 0x0217: Right Line-In volume 0dB */
			TLV320_REG_WRITE(TLV320_LCH, 0x00F9), /* 0x04f9: Left headphone volume 0dB */
			TLV320_REG_WRITE(TLV320_RCH, 0x00F9), /* 0x06f9: Right headphone volume 0dB */
			TLV320_REG_WRITE(TLV320_DAP, 0x0004)  /* 0x0a04: DAC enable */
		}; 

		printk("ibmcsiti: Restoring audio chip \n");
		for (i=0; i < (sizeof(i2c_data) / sizeof(i2c_data[0])); i++) {
			if (i2c_master_send(s->i2c, (u8 *)(i2c_data+i), 2) != 2) {
				printk(KERN_ERR "ibmcsiti: I2C write error, "
				       "cannot restore audio chip.\n"); 
				i2c_error = 1; 
				break; 
			}
 			udelay(1000); 
			/* Undocumented, but some commands
			 * apparently require some delay. */
		}
		ibmcsiti_set_volume(s, ibmcsiti_get_volume(s)); 
		ibmcsiti_set_line(s, ibmcsiti_get_line(s)); 
		ibmcsiti_set_mic(s, ibmcsiti_get_mic(s)); 
		ibmcsiti_set_recsrc(s, ibmcsiti_get_recsrc(s)); 
	
		printk("ibmcsiti: Audio setting restored, proceeding with CSI/DMAC\n"); 

	}
	/* End I2C code */

	if ( ibm_csi_base == NULL ) return;
 
    	/* Restart Tx DMA channel if DAC was running */ 
	if ( ( ibm_csi_hardware_registers[CSI0_ER_reg] & CSI_ER_TXEN) ) {
		mtdcr(TX_SG, virt_to_phys(ibm_dma_Tx_descriptor));
		mtdcr(DCRN_ASGC, mfdcr(DCRN_ASGC) | TX_SG_ENABLE | TX_SG_MASK );
	}

    	/* Ditto for Rx / ADC */ 
	if ( ( ibm_csi_hardware_registers[CSI0_ER_reg] & CSI_ER_RXEN) ) {
		mtdcr(RX_SG, virt_to_phys(ibm_dma_Rx_descriptor)); 
		mtdcr(DCRN_ASGC, mfdcr(DCRN_ASGC) | RX_SG_ENABLE | RX_SG_MASK );
	}

	/* Restart CSI (note the order in which the registers are restored) */ 
	__raw_writel(ibm_csi_hardware_registers[CSI0_SCR_reg], ibm_csi_base + CSI_SCR_OFFSET);
	__raw_writel(ibm_csi_hardware_registers[CSI0_CFG_reg], ibm_csi_base + CSI_CFG_OFFSET);

   	__raw_writel(ibm_csi_hardware_registers[CSI0_ER_reg], ibm_csi_base + CSI_ER_OFFSET);

	printk("ibmcsiti: Restore done, exiting\n");

  	iounmap(ibm_csi_base);
	/* ..... phew ..... */ 
}
   

static int ibmcsiti_suspend(struct device * dev, u32 state, u32 level)
{

	switch(level)
	{ 
	case SUSPEND_POWER_DOWN:

		/* Perhaps no need to tweak CPM, but need access to private data */
		/* The dev struct is useless when retrieving driver private data */ 
		/* We do it the hard way */
	{ 
		struct ibmcsiti_state *s;
		struct list_head *pdevs = &devs;
 
		while (pdevs->next != &devs) {
			s = list_entry(pdevs->next, struct ibmcsiti_state, devs);
			/* Note the last 'devs' is the member in ibmcsiti_state */ 
			ibmcsi_save_state(s); 	
			pdevs = pdevs->next; 	
		}
	
	}
	ppc4xx_cpm_fr(CPM_CSI, 1); /* Power down the core */
	break;
	}
  
	return 0;
}

static int ibmcsiti_resume(struct device * dev, u32 level)
{

	switch(level)
	{
	case RESUME_POWER_ON:

		/*
		 * Turn off sleep mode.
		 */

		ppc4xx_cpm_fr(CPM_CSI, 0); /* Power up the core */
		{ 
			struct ibmcsiti_state *s;
			struct list_head *pdevs = &devs;
 
			while (pdevs->next != &devs) {
				s = list_entry(pdevs->next, struct ibmcsiti_state, devs);
				ibmcsi_restore_state(s); 	
				pdevs = pdevs->next; 	
			} /* Strictly speaking we may need to reverse traverse dev list */ 
	
		}
		break;
	}

	return 0;
}
#endif /* linux-pm */

/*****************************************************************************/
/* 		Initialization tables					     */
/*****************************************************************************/

static /*const*/ struct file_operations ibmcsiti_audio_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= ibmcsiti_dsp_read,
	.write		= ibmcsiti_dsp_write,
	.poll		= ibmcsiti_dsp_poll,
	.ioctl		= ibmcsiti_dsp_ioctl,
	.mmap		= NULL,
	.open		= ibmcsiti_dsp_open,
	.release	= ibmcsiti_dsp_release,
};

static /*const*/ struct file_operations ibmcsiti_mixer_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.ioctl		= ibmcsiti_mixer_ioctl,
	.open		= ibmcsiti_mixer_open,
	.release	= ibmcsiti_mixer_release,
};

static struct i2c_driver ibmcsiti_i2c_driver = {
	.name		= "IBMCSI+TLV320AIC23 codec",
	.id		= 0xf314,
	.flags		= I2C_DF_NOTIFY,
	.attach_adapter	= ibmcsi_i2c_attach_adapter,
	.detach_client	= ibmcsi_i2c_detach_client,
	.command	= NULL,
	.inc_use	= ibmcsi_i2c_inc_use,
	.dec_use	= ibmcsi_i2c_dec_use,
};

typedef int (*PIF) (struct ibmcsiti_state *, int);

static struct inittable {
	PIF proc;
	int val;
} inittable[] __initdata = {
	{ibmcsiti_set_recsrc,	SOUND_MASK_MIC},
	{ibmcsiti_set_volume,	100 * 35/47},
	{ibmcsiti_set_line,	100},
	{ibmcsiti_set_mic,	100},
};

/*****************************************************************************/
/* Driver initialization and cleanup					     */
/*****************************************************************************/
static int __init init_ibmcsiti(void)
{
	int ret;

	/* We just register the I2C driver here.  The initialization
	 * of the device proper happens when the I2C layer calls us
	 * back. */
	printk(KERN_INFO "ibmcsiti: compiled at " __TIME__ " " __DATE__ "\n");

	ret = i2c_add_driver(&ibmcsiti_i2c_driver);
	if (ret)
		printk(KERN_ERR "ibmcsiti: i2c_add_driver() failed\n");

	return ret;
}

static int ibmcsiti_i2c_init(struct i2c_client *client)
{
	struct ibmcsiti_state *s = client->data;
	int ret;
	int i;
	unsigned long end_time;
	void *codec_lines_virt;
	dma_addr_t codec_lines;

	printk("ibmcsiti_i2c_init():  addr=0x%x\n", client->addr);

	/* Power up the CSI core during initialization */
	ppc4xx_cpm_fr(CPM_CSI, 0);

	/* Allocate line buffers */
	codec_lines_virt = consistent_alloc(GFP_DMA, PAGE_SIZE, &codec_lines);
	if (! codec_lines_virt) {
		printk(KERN_ERR "ibmcsiti: out of memory\n");
		/* FIXME we should turn the CSI off again if init fails */
		ret = -ENOMEM;
		goto err_linebuffer;
	}

	if (! request_mem_region(CSI0_IO_BASE, CSI0_EXTENT, "ibmcsiti")) {
		printk(KERN_ERR "ibmcsiti: I/O ports %x-%x in use\n",
		       CSI0_IO_BASE, CSI0_EXTENT-1);
		return -EBUSY;
	}

	/* Check CSI core presence */ 	
	if ( (IBMCSI_READ(CSI0_ID) >> 16) != CSI_ID_405LP) {
		printk(KERN_WARNING "ibmcsiti: Unexpected CSI ID %x\n",
		       IBMCSI_READ(CSI0_ID));
		ret = -ENODEV;
		goto err_irq1;
	}

	memset(s, 0, sizeof(struct ibmcsiti_state));
	s->i2c = client;
	s->magic = IBMCSI_MAGIC;

	init_waitqueue_head(&s->dma_adc.wait);
	init_waitqueue_head(&s->dma_dac.wait);
	init_waitqueue_head(&s->open_wait);
	init_MUTEX(&s->open_sem);
	spin_lock_init(&s->lock);

	s->write_line = codec_lines_virt;
	s->dma_write_line = codec_lines;
	s->read_line = (void *)((char *)codec_lines_virt + 128);
	s->dma_read_line = codec_lines + 128;

	/* Interrupts - currently held indefinitely since no one else
	 * will be using them.  Mixer and DSP use the same
	 * interrupts. */

   	ret = request_irq(IBMCSI_TXDMA_IRQ, ibmcsiti_dac_interrupt,
			  SA_SHIRQ, "ibmcsiti", s);
   	if (ret != 0) {
		printk(KERN_ERR "ibmcsiti: IRQ %d in use\n", IBMCSI_TXDMA_IRQ);
		goto err_irq1;
	}
   	ret = request_irq(IBMCSI_RXDMA_IRQ, ibmcsiti_adc_interrupt,
			  SA_SHIRQ, "ibmcsiti", s);
   	if (ret != 0) {
		printk(KERN_ERR "ibmcsiti: IRQ %d in use\n",IBMCSI_RXDMA_IRQ);
		goto err_irq2;
	}

#if 0 /* FIXME: CSI error interrupts not implemented */
	ret=request_irq(21, ibmcsiti_csi_interrupt, SA_SHIRQ, "ibmcsiti",s);
   	if (ret) {
		printk(KERN_ERR "ibmcsiti: irq 21 in use\n");
		goto err_irq;
	}
#endif

	/* register devices */
	ret = s->dev_dsp = register_sound_dsp(&ibmcsiti_audio_fops, -1);
	if (ret < 0) {
		printk(KERN_ERR "ibmcsiti: Could not register DSP.\n");
		goto err_dev1;
	}

	ret = s->dev_mixer = register_sound_mixer(&ibmcsiti_mixer_fops, -1);
	if (ret < 0) {
		printk(KERN_ERR "ibmcsiti: Could not register mixer.\n");
		goto err_dev2;
	}

	/* Initialize the cores / chip:  DMA controller, CSI and codec.*/
	/* Turn on the DMA controller 				       */
#if 0				/* Unnecessary - Bishop */
	end_time = jiffies + HZ/10;
	mtdcr(DCRN_CPC0_FR, mfdcr(DCRN_CPC0_FR) & (~CPM_DMA));

	while (! (mfdcr(DCRN_CPC0_SR) & CPM_DMA) ) { /* DMAC asleep? */
		mfdcr(IBMCSI_DMA_SR); /* Dummy read to wake it up */
		mtdcr(DCRN_CPC0_FR, mfdcr(DCRN_CPC0_FR) & (~CPM_DMA)); /* Turn on DMA Controller */
		asm("isync");
		if (jiffies > end_time) {
			printk(KERN_ERR "ibmcsiti: cannot turn on DMA controller \n");
			goto err_dev3;
		}
	}

	/* Caveat : If you don't do the above and the DMA controller
	 happens to be turned off, the controller will still respond
	 to DCR reads/writes and even change state to DRQ Pending /
	 Channel Active when you try to kick off DMA, but will
	 adamantly refuse to do any real work. This can happen if the
	 kernel is configured without PPC405 DMA support, or if a
	 power management driver / daemon tried to save power.  We
	 therefore power up the DMA controller before doing anything
	 with it and codec.  */
#endif

	IBMCSI_WRITE(CSI0_ER,0);	/* Stop CSI */

	/* Set up TI codec via direct I2C manipulation. */
	/* The control registers are write-only, so we have to init
	 * them to a known state. */  
	{
 		int i2c_error = 0; 
		u16 i2c_data[] = {	
			TLV320_REG_WRITE(TLV320_RR,  0x0000), /* 0x1e00: Reset chip */
			TLV320_REG_WRITE(TLV320_DIA, 0x0000), /* 0x1200: Deactivate digital interface */
			TLV320_REG_WRITE(TLV320_DAI, 0x0053), /* 0x0e53: Master, MSB on 2nd BCLK, DSP format */
			TLV320_REG_WRITE(TLV320_PDC, 0x0100), /* 0x0d00: Power up everything */
			TLV320_REG_WRITE(TLV320_DIA, 0x0001), /* 0x1201: Activate digital interface */
			TLV320_REG_WRITE(TLV320_LLI, 0x0017), /* 0x0017: Left Line-In volume 0dB */
			TLV320_REG_WRITE(TLV320_RLI, 0x0017), /* 0x0217: Right Line-In volume 0dB */
			TLV320_REG_WRITE(TLV320_LCH, 0x00F9), /* 0x04f9: Left headphone volume 0dB */
			TLV320_REG_WRITE(TLV320_RCH, 0x00F9), /* 0x06f9: Right headphone volume 0dB */
			TLV320_REG_WRITE(TLV320_DAP, 0x0004)  /* 0x0a04: DAC enable */
		}; 

		for (i=0; i < (sizeof(i2c_data) / sizeof(i2c_data[0])); i++) {
			if (i2c_master_send(client, (u8 *)(i2c_data+i), 2) != 2) {
				printk(KERN_ERR "ibmcsiti: I2C write error, "
				       "cannot set up audio chip.\n"); 
				i2c_error = 1; 
				break; 
			} 
			udelay(1000); 
			/* Undocumented, but some commands
			 * apparently require some delay. */
		}

	}
	/* End I2C code */

	/* Initialize shadow registers */
	{
		u16 codec_reg_init[TLV320_REG_EXTENT] = {0x0117,
							 0x0117,
							 0x01f9,
							 0x01f9,
							 0x0010,
							 0x0004,
							 0x0100,
							 0x0053,
							 0x0020,
							 0x0001,};

		for (i=0; i<TLV320_REG_EXTENT; i++)
			s->codec_reg[i] = codec_reg_init[i];
	}

	/* Initialize mixer settings (see inittable[] above for
	 * procedures and values.*/
	for (i=0; i < (sizeof(inittable)/sizeof(inittable[0])); i++)
		inittable[i].proc(s, inittable[i].val);

	/* Add to driver list */
	list_add_tail(&s->devs, &devs);
	
#if 1 /* linux-pm */
        ibmcsiti_ldm_register();
#endif /* linux-pm */

	return 0;

 err_dev3:		/* Error exit initializing chips 	*/
	unregister_sound_mixer(s->dev_mixer);
 err_dev2:		/* Error exit registering mixer device	*/
	unregister_sound_dsp(s->dev_dsp);
 err_dev1:
	free_irq(IBMCSI_TXDMA_IRQ, s);
 err_irq2:
       	free_irq(IBMCSI_RXDMA_IRQ, s);
#if 0	/* FIXME */
	free_irq(21,s);
#endif
 err_irq1:
	release_mem_region(CSI0_IO_BASE, CSI0_EXTENT);
 err_linebuffer:
	consistent_free(codec_lines_virt);
	ppc4xx_cpm_fr(CPM_CSI, 1); /* Turn the core off again */

	return ret;
}

static void __exit cleanup_ibmcsiti(void)
{
	struct ibmcsiti_state *s;
	static void *codec_lines_virt;
	
	while (devs.next != &devs) {
		s = list_entry(devs.next, struct ibmcsiti_state, devs);
		list_del(devs.next);
		unregister_sound_mixer(s->dev_mixer);
		unregister_sound_dsp(s->dev_dsp);
		free_irq(IBMCSI_TXDMA_IRQ, s);
		free_irq(IBMCSI_RXDMA_IRQ, s);
		release_mem_region(CSI0_IO_BASE, CSI0_EXTENT);
		codec_lines_virt = s->write_line;
		kfree(s);
		consistent_free(codec_lines_virt);
	}

#if 1 /* linux-pm */
        ibmcsiti_ldm_unregister();
#endif /* linux-pm */
}

module_init(init_ibmcsiti);
module_exit(cleanup_ibmcsiti);

/*****************************************************************************/
/*	I2C Callbacks 							     */
/*****************************************************************************/

/* Blech.  It's not as bad as PCMCIA, but boy does the i2c
 * infrastructure suck... */

static unsigned short normal_i2c[] = {0x1a, I2C_CLIENT_END,};
static unsigned short dummy_i2c_addrlist[] = {I2C_CLIENT_END,};

static struct i2c_client_address_data ibmcsi_i2c_address_data = {
	.normal_i2c		= normal_i2c,
	.normal_i2c_range	= dummy_i2c_addrlist,
	.probe			= dummy_i2c_addrlist,
	.probe_range		= dummy_i2c_addrlist,
	.ignore			= dummy_i2c_addrlist,
	.ignore_range		= dummy_i2c_addrlist,
	.force			= dummy_i2c_addrlist,
};

static int ibmcsi_i2c_id; /* = 0 */

static int ibmcsi_i2c_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &ibmcsi_i2c_address_data,
			 &ibmcsi_i2c_detect_client);
}

static int ibmcsi_i2c_detach_client(struct i2c_client *client)
{
	printk(KERN_WARNING "ibmcsiti: ibmcsi_i2c_detach_client() called. "
	       "Don't know how to cope :-(\n");
	return 0;
}

static void ibmcsi_i2c_inc_use(struct i2c_client *client)
{
	MOD_INC_USE_COUNT;
}

static void ibmcsi_i2c_dec_use(struct i2c_client *client)
{
	MOD_DEC_USE_COUNT;
}

static int ibmcsi_i2c_detect_client(struct i2c_adapter *adap, int addr, 
				    unsigned short flags, int kind)
{
	int err = 0;
	struct i2c_client *client;
	struct ibmcsiti_state *data;
	
	printk("ibmcsi_i2c_detect_client(): addr=%x\n", addr);

	if (! i2c_check_functionality(adap, I2C_FUNC_I2C)) {
		printk(KERN_ERR "ibmcsiti: I2C bus lacks functionality\n");
		return -ENODEV;
	}

	/* FIXME: Check the I2C adapter is the one we expect? */

	/* This device is write-only, and hence undetectable.  We just
	 * assume we've been called with the right address... (did I
	 * mention the i2c infrastructure was crap?) */	
	
	client = kmalloc(sizeof(struct i2c_client)
			 + sizeof(struct ibmcsiti_state), GFP_KERNEL);

	if (! client)
		return -ENOMEM;

	/* This is tricky, but it will set the data to the right value. */
	client->data = client + 1;
	data = (struct ibmcsiti_state *)client->data;
	
	client->addr = addr;
	client->adapter = adap;
	client->driver = &ibmcsiti_i2c_driver;
	client->flags = 0;
	strcpy(client->name, "Arctic-2 codec");
	client->id = ibmcsi_i2c_id++; /* FIXME: Racy? */
	
	/* Tell the i2c layer a new client has arrived */
	err = i2c_attach_client(client);
	if (err) {
		printk(KERN_ERR "ibmcsiti: Could not attach I2C client\n");
		goto fail;
	}

	/* Now set up the device */
	err = ibmcsiti_i2c_init(client);
	if (err) {
		i2c_detach_client(client); /* FIXME: ugly, do better */
		goto fail;
	}

	return 0;

 fail:	
	if (client)
		kfree(client);

	return err;
}

/*****************************************************************************/
/*	Part I : DSP device /dev/dsp					     */
/*****************************************************************************/


static int ibmcsiti_dsp_open(struct inode *inode, struct file *file)
{
	int minor = MINOR(inode->i_rdev);
	DECLARE_WAITQUEUE(wait, current);
	unsigned long flags;
	struct list_head *list;
	struct ibmcsiti_state *s;

	for (list = devs.next; ; list = list->next) {
		if (list == &devs)
			return -ENODEV;
		s = list_entry(list, struct ibmcsiti_state, devs);
		if (!((s->dev_dsp ^ minor) & ~0xf))
			break;
	}

       	VALIDATE_STATE(s);

	file->private_data = s;

	/* wait for device to become free */
	down(&s->open_sem);
	while (s->open_mode & file->f_mode) {
		if (file->f_flags & O_NONBLOCK) {
			up(&s->open_sem);
			return -EBUSY;
		}
		add_wait_queue(&s->open_wait, &wait);
		set_current_state(TASK_INTERRUPTIBLE);
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

	/* FIXME: sampling rate hardcoded */

	{
	struct dma_sgdt *dacp, *adcp;

#define SGDT_COUNT ((PAGE_SIZE/2)/(sizeof(struct dma_sgdt)))

	if (file->f_mode & FMODE_WRITE) {
		s->dac_free_sgdt_q = (struct dma_sgdt *)(s->write_line);
	 	init_sgdt_q(s->dac_free_sgdt_q, SGDT_COUNT);
		/* Prepare default (anchor) descriptor for DAC */
		dacp = s->dac_active_sgdt_q = get_sgdt(&(s->dac_free_sgdt_q));

  		dacp->ccw = (unsigned int)IBMCSI_TXDMA_GO_NOI;

		dacp->nextP = virt_to_phys(dacp); /* Loop in on itself */

		dacp->srcP = virt_to_phys(&(dacp->dummy)); /* Point to dummy data */
		dacp->destP = CSI0_TBUF;
		dacp->dummy = 0;	/* Dummy neutral data (FIXME: currently assumes signed 16 format) */
		dacp->ctrl = 0x80000001; /* Link, count = 1 */
		s->dac_sgdt_lastV = dacp;
		printk("ibmcsi: s = %p dac free_sgdt_q = %p at %p lastV = %p at %p\n",
		       s, s->dac_free_sgdt_q, &s->dac_free_sgdt_q, s->dac_sgdt_lastV, 
		       &s->dac_sgdt_lastV); 
	}

   	if (file->f_mode & FMODE_READ) {
		s->adc_free_sgdt_q = (struct dma_sgdt *)(s->write_line) + SGDT_COUNT;
	 	init_sgdt_q(s->adc_free_sgdt_q, SGDT_COUNT);

#if 0
		/* Prepare default (anchor) descriptor for ADC */
		adcp = s->adc_active_sgdt_q = get_sgdt(&(s->adc_free_sgdt_q));
		adcp->ccw = (unsigned int)IBMCSI_RXDMA_GO_NOI;
		adcp->nextP = virt_to_phys(adcp);
		adcp->ctrl = 0x80000001;

		adcp->srcP = CSI0_RBUF;
		adcp->destP = virt_to_phys(&(adcp->dummy));

		s->adc_sgdt_lastV = adcp;
#endif
		printk("ibmcsi: s = %p adc free_sgdt_q = %p at %p lastV = %p at %p\n",
		       s, s->adc_free_sgdt_q, &s->adc_free_sgdt_q, s->adc_sgdt_lastV,
		       &s->adc_sgdt_lastV); 
	}


	} 

	if (file->f_mode & FMODE_READ) {
		s->dma_adc.ossfragshift = s->dma_adc.ossmaxfrags = s->dma_adc.subdivision = 0;
		s->dma_adc.enabled = 1;
		/* FIXME: data format hardcoded */

		if ( !(IBMCSI_READ(CSI0_ER) & 1) ) { /* CSI currently not in use */ 
			IBMCSI_WRITE(CSI0_ER, 0); /* Clear enable reg, disable CSI */
			IBMCSI_WRITE(CSI0_SR, CSI_SR_TOD | CSI_SR_ROD); /* Clear CSI errors */
			mtdcr(IBMCSI_RXDMA_CR, 0); 	/* Disable DMA Rx */
			mtdcr(IBMCSI_DMA_SR, DCRN_DMA_SR_ALL(IBMCSI_RXDMA) | 
					     DCRN_DMA_SR_ALL(IBMCSI_TXDMA) ) ;
				 		/* Clear DMA status bits */
		} /* If currently active, assume it's working OK */ 

		init_timer(&s->adc_timer);
		s->adc_timer.function = ibmcsi_adc_timer;
		s->adc_timer.data = (unsigned long)s;
		s->adc_timer.expires = jiffies + ADC_TIMER_PERIOD;
		add_timer(&s->adc_timer);
	}

   	if (file->f_mode & FMODE_WRITE) {
		s->dma_dac.ossfragshift = s->dma_dac.ossmaxfrags
			= s->dma_dac.subdivision = 0;
		s->dma_dac.enabled = 1;

		/* FIXME: data format hardcoded */

		s->dma_dac.hwptr = 0;
		s->dma_dac.swptr = 0;
		s->dma_dac.count = 0;
		s->dma_dac.sg_count = 0;

		if ( !(IBMCSI_READ(CSI0_ER) & 1) ) { /* CSI currently not in use */ 
			IBMCSI_WRITE(CSI0_ER, 0); /* Clear enable reg, disable CSI */
			IBMCSI_WRITE(CSI0_SR, CSI_SR_TOD | CSI_SR_ROD);	/* Clear CSI errors */
			mtdcr(IBMCSI_TXDMA_CR, 0);	/* Disable DMA Tx */
			mtdcr(IBMCSI_DMA_SR, DCRN_DMA_SR_ALL(IBMCSI_RXDMA) | 
					     DCRN_DMA_SR_ALL(IBMCSI_TXDMA) ) ;
				 		/* Clear DMA status bits */
		}

		init_timer(&s->dac_timer);
		s->dac_timer.function = ibmcsi_dac_timer;
		s->dac_timer.data = (unsigned long)s;
		s->dac_timer.expires = jiffies + DAC_TIMER_PERIOD;
		add_timer(&s->dac_timer);

		DBG(printk("ibmcsi: jiffies %d HZ %d timer %d\n", jiffies, HZ,
			   s->dac_timer.expires));
	}

   	spin_unlock_irqrestore(&s->lock, flags);
	up(&s->open_sem);
 	init_MUTEX(&s->dsp_sem);

	return 0;
}

static ssize_t ibmcsiti_dsp_read(struct file *file, char *buffer,
				 size_t count, loff_t *ppos)
{
	struct ibmcsiti_state *s = (struct ibmcsiti_state *)file->private_data;
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
		spin_lock_irqsave(&s->lock, flags);

		swptr  = s->dma_adc.swptr;
		/* Record curent sgdt for this snapshot */ 

#if 0
		if (s->dma_adc.hwptr >= s->dma_adc.swptr)
			cnt = s->dma_adc.hwptr - swptr;
		else /* HWPTR wrapped */
			cnt = s->dma_adc.dmasize - swptr;

		if (s->dma_adc.count < cnt)
			cnt = s->dma_adc.count;

		if (cnt <= 0)
			set_current_state(TASK_INTERRUPTIBLE);

		if (cnt > count*2) /* cnt is raw (4 bytes per sample),
				    * count is cooked (2 bytes per
				    * sample) */
			cnt = count * 2;

#else /* Change to handle multiple interrupts before a successful read */ 

		cnt = s->dma_adc.count; /* Use the count accumulated
					 * by interrupt handler */

		/* cnt is raw (4 bytes per sample), count is cooked (2
		 * bytes per sample) */
		if (cnt > count*2)
			cnt = count * 2;
		
		if (cnt + swptr > s->dma_adc.dmasize)
			cnt = s->dma_adc.dmasize - swptr; 

		if (cnt > 0) { 
			s->dma_adc.swptr = (swptr + cnt) % s->dma_adc.dmasize;
			DBG(printk("%8.8x %8.8x\n",s->dma_adc.swptr, s->dma_adc.hwptr));
			if (s->dma_adc.swptr <= s->dma_adc.hwptr)  
				s->adc_sw_prev_sgdt = s->adc_hw_prev_sgdt; 
			s->dma_adc.count -= cnt;
		} else {
			set_current_state(TASK_INTERRUPTIBLE);
		}

#endif 
		spin_unlock_irqrestore(&s->lock, flags);

		if (cnt <= 0) { /* No data yet */

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

		if (copy_samples_to_user(buffer, s->dma_adc.rawbuf + swptr,
					 cnt/4, 0)) {
			ret = -EFAULT;
			goto out;
		}

#if 0 /* Change to handle multiple interrupts before read */ 
		swptr = (swptr + cnt) % s->dma_adc.dmasize;
		spin_lock_irqsave(&s->lock, flags);
		s->dma_adc.swptr = swptr;
		s->dma_adc.count -= cnt;
		spin_unlock_irqrestore(&s->lock, flags);
#endif

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


static ssize_t ibmcsiti_dsp_write(struct file *file, const char *buffer,
				  size_t inbytes, loff_t *ppos)
{
	struct ibmcsiti_state *s = file->private_data;
	DECLARE_WAITQUEUE(wait, current);
	ssize_t ret = 0;
	unsigned long flags;
	unsigned swptr;
	size_t bytespersample, insamples, outsamples;
	long bufspace;

	VALIDATE_STATE(s);

	if (ppos != &file->f_pos)
		return -ESPIPE;
	if (s->dma_dac.mapped)
		return -ENXIO;
	if (! access_ok(VERIFY_READ, buffer, inbytes))
		return -EFAULT;

	down(&s->dsp_sem);

	bytespersample = s->outstereo ? 4 : 2;

	insamples = inbytes / bytespersample;

	if (!s->dma_dac.ready && (ret = prog_dmabuf_dac(s))) {
		printk("DEBUG: ibmcsiti_dsp_write dac ready %d prog_dmabuf retcode %d \n", s->dma_dac.ready, ret);
		goto out;
	}

	ret = 0;
        add_wait_queue(&s->dma_dac.wait, &wait);
	while (insamples > 0) {
		spin_lock_irqsave(&s->lock, flags);
		if (s->dma_dac.count < 0) {
			s->dma_dac.count = 0;
			s->dma_dac.swptr = s->dma_dac.hwptr;
		}
		swptr = s->dma_dac.swptr;
		bufspace = s->dma_dac.dmasize-swptr;

		/* Truncate to fit in dma buffer */
		if (s->dma_dac.count + bufspace > s->dma_dac.dmasize)
			bufspace = s->dma_dac.dmasize - s->dma_dac.count;

		if ((bufspace <= 0) || (s->dac_free_sgdt_q == NULL))
			set_current_state(TASK_INTERRUPTIBLE);
		spin_unlock_irqrestore(&s->lock, flags);
		
		if ((bufspace <= 0) || (s->dac_free_sgdt_q == NULL)) {
			/* No room... */
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
			if (s->dma_dac.mapped) {
				ret = -ENXIO;
				goto out;
			}
			continue;
		}

		/* Always 4 bytes/sample (stereo) in the DMA buffer */
		outsamples = min_t(size_t, insamples, bufspace/4);

		if (copy_samples_from_user(s->dma_dac.rawbuf + swptr, buffer,
					   outsamples, s->outstereo)) {
			ret = -EFAULT;
			goto out;
		}

		/* FIXME: replace with consistent_sync()? */
		dma_cache_wback_inv((unsigned long)(s->dma_dac.rawbuf + swptr),
				    outsamples*4);

		swptr = (swptr + outsamples*4) % s->dma_dac.dmasize; /* Wrap */
		spin_lock_irqsave(&s->lock, flags);
		s->dma_dac.swptr = swptr;
		s->dma_dac.count += outsamples * 4;
		s->dma_dac.endcleared = 0; /* Remember to zero clear
					    * one fragment at the
					    * end */

		s->dma_dac.sg_count += outsamples*4;

		spin_unlock_irqrestore(&s->lock, flags);
		/* FIXME: broken if odd byte written. Also, format is
		 * assumed 16 bit */
		insamples -= outsamples;
		buffer += outsamples * bytespersample;
		ret += outsamples * bytespersample;

		if (s->dma_dac.enabled) {
			DBG(printk("bottom start_dac\n"));
			DBG(printk("dma_dac.count %d, dma_dac.sg_count %d\n",
				   s->dma_dac.count, s->dma_dac.sg_count)); 
			start_dac(s);
			DBG(printk("dma_dac.count %d, dma_dac.sg_count %d\n",
				   s->dma_dac.count, s->dma_dac.sg_count));
		}
		
	}
out:
	up(&s->dsp_sem);
        remove_wait_queue(&s->dma_dac.wait, &wait);
	set_current_state(TASK_RUNNING);

	DBG(printk("swptr %8.8x hwptr %8.8x sg_count %d\n", s->dma_dac.swptr,
		   s->dma_dac.hwptr, s->dma_dac.sg_count);)

	return ret;
}

static unsigned int ibmcsiti_dsp_poll(struct file *file,
				      struct poll_table_struct *wait)
{
	struct ibmcsiti_state *s = file->private_data;
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
	ibmcsiti_update_ptr(s);
	if (file->f_mode & FMODE_READ) {
		if (s->dma_adc.count >= (signed)s->dma_adc.fragsize)
			mask |= POLLIN | POLLRDNORM;
	}
	if (file->f_mode & FMODE_WRITE) {
		if (s->dma_dac.mapped) {
			if (s->dma_dac.count >= (signed)s->dma_dac.fragsize)
				mask |= POLLOUT | POLLWRNORM;
		} else {
			if ((signed)s->dma_dac.dmasize >= s->dma_dac.count + (signed)s->dma_dac.fragsize)
				mask |= POLLOUT | POLLWRNORM;
		}
	}
	spin_unlock_irqrestore(&s->lock, flags);
	return mask;
}

static int ibmcsiti_dsp_ioctl(struct inode *inode, struct file *file,
			      unsigned int cmd, unsigned long arg)
{
	struct ibmcsiti_state *s = (struct ibmcsiti_state *)file->private_data;
	unsigned long flags;
        audio_buf_info abinfo;
        count_info cinfo;
	int count;
	int val, ret;

	VALIDATE_STATE(s);

	switch (cmd) {
	case OSS_GETVERSION:
		return put_user(SOUND_VERSION, (int *)arg);

	case SNDCTL_DSP_SYNC:
		printk(KERN_DEBUG "SNDCTL_DSP_SYNC\n");
		if (file->f_mode & FMODE_WRITE)
			return drain_dac(s, 0/*file->f_flags & O_NONBLOCK*/);
		return 0;

	case SNDCTL_DSP_SETDUPLEX:
		printk(KERN_DEBUG "SNDCTL_DSP_SETDUPLEX\n");
		return 0;

	case SNDCTL_DSP_GETCAPS:
		printk(KERN_DEBUG "SNDCTL_DSP_GETCAPS\n");
		return put_user(DSP_CAP_DUPLEX | DSP_CAP_REALTIME | DSP_CAP_TRIGGER | DSP_CAP_MMAP, (int *)arg);

        case SNDCTL_DSP_RESET:
		printk(KERN_DEBUG "SNDCTL_DSP_RESET\n");
		if (file->f_mode & FMODE_WRITE) {
			stop_dac(s);
			synchronize_irq();
			s->dma_dac.swptr = s->dma_dac.hwptr = s->dma_dac.count
				= s->dma_dac.total_bytes = 0;
		}
		if (file->f_mode & FMODE_READ) {
			stop_adc(s);
			synchronize_irq();
			s->dma_adc.swptr = s->dma_adc.hwptr = s->dma_adc.count
				= s->dma_adc.total_bytes = 0;
		}
		return 0;

        case SNDCTL_DSP_SPEED:
                if (get_user(val, (int *)arg))
			return -EFAULT;

		printk(KERN_DEBUG "SNDCTL_DSP_SPEED  arg=%d\n", val);

		/* FIXME: Hardcoded for now */
		return put_user(IBMCSI_DEFAULT_SAMPLING_RATE, (int *)arg); 

        case SNDCTL_DSP_STEREO:
                if (get_user(val, (int *)arg))
			return -EFAULT;

		printk(KERN_DEBUG "SNDCTL_DSP_STEREO arg=%d\n", val);

		if (file->f_mode & FMODE_WRITE) {
			down(&s->dsp_sem);
			if (val)
				s->outstereo = 1;
			else
				s->outstereo = 0;
			
			if (put_user(s->outstereo, (int *)arg) != 0)
				ret = -EFAULT;
			up(&s->dsp_sem);
		} else {
			if (put_user(0, (int *)arg) != 0)
				ret = -EFAULT;
		}

        	return ret;

        case SNDCTL_DSP_CHANNELS:
		if (get_user(val, (int *)arg))
			return -EFAULT;

		printk(KERN_DEBUG "SNDCTL_DSP_CHANNELS arg=%d\n", val);

		if (file->f_mode & FMODE_WRITE) {
			if (val >= 2) {
				s->outstereo = 1;
				ret = put_user(2, (int *)arg);
			} else {
				s->outstereo = 0;
				ret = put_user(1, (int *)arg);
			}
		} else {
			ret = put_user(1, (int *)arg);
		}

		if (ret)
			return -EFAULT;

		return 0;

	case SNDCTL_DSP_GETFMTS: /* Returns a mask */
		printk(KERN_DEBUG "SNDCTL_DSP_GETFMTS\n");
		return put_user(AFMT_S16_BE, (int *)arg);

	case SNDCTL_DSP_SETFMT: /* Selects ONE fmt*/
		if (get_user(val, (int *)arg))
			return -EFAULT;
		printk(KERN_DEBUG "SNDCTL_DSP_SETFMT arg=%d\n", val);
		return put_user(AFMT_S16_BE, (int *)arg);

	case SNDCTL_DSP_POST:
                return 0;

        case SNDCTL_DSP_GETTRIGGER:
		printk(KERN_DEBUG "SNDCTL_DSP_GETTRIGGER\n");
		val = 0;
		if ((file->f_mode) & FMODE_READ && (s->state & IBMCSI_ADC_RUNNING))
			val |= PCM_ENABLE_INPUT;
		if ((file->f_mode & FMODE_WRITE) && (s->state & IBMCSI_DAC_RUNNING))
			val |= PCM_ENABLE_OUTPUT;
		return put_user(val, (int *)arg);

	case SNDCTL_DSP_SETTRIGGER:
		if (get_user(val, (int *)arg))
			return -EFAULT;
		printk(KERN_DEBUG "SNDCTL_DSP_SETTRIGGER arg=%d\n", val);
		if (file->f_mode & FMODE_READ) {
			if (val & PCM_ENABLE_INPUT) {
				if (!s->dma_adc.ready && (ret = prog_dmabuf_adc(s)))
					return ret;
				s->dma_adc.enabled = 1;
				start_adc(s);
			} else {
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
			} else {
				s->dma_dac.enabled = 0;
				stop_dac(s);
			}
		}
		return 0;

	case SNDCTL_DSP_GETOSPACE:
		printk(KERN_DEBUG "SNDCTL_DSP_GETOSPACE\n");
		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		if (!s->dma_dac.ready && (val = prog_dmabuf_dac(s)) != 0)
			return val;
		spin_lock_irqsave(&s->lock, flags);
		ibmcsiti_update_ptr(s);
		abinfo.fragsize = s->dma_dac.fragsize;
		count = s->dma_dac.count;
		if (count < 0)
			count = 0;
                abinfo.bytes = s->dma_dac.dmasize - count;
                abinfo.fragstotal = s->dma_dac.numfrag;
                abinfo.fragments = abinfo.bytes >> s->dma_dac.fragshift;
		spin_unlock_irqrestore(&s->lock, flags);
		return copy_to_user((void *)arg, &abinfo, sizeof(abinfo))
			? -EFAULT : 0;

	case SNDCTL_DSP_GETISPACE:
		printk(KERN_DEBUG "SNDCTL_DSP_GETISPACE\n");
		if (!(file->f_mode & FMODE_READ))
			return -EINVAL;
		if (!s->dma_adc.ready && (val = prog_dmabuf_adc(s)) != 0)
			return val;
		spin_lock_irqsave(&s->lock, flags);
		ibmcsiti_update_ptr(s);
		abinfo.fragsize = s->dma_adc.fragsize;
		count = s->dma_adc.count;
		if (count < 0)
			count = 0;
                abinfo.bytes = count;
                abinfo.fragstotal = s->dma_adc.numfrag;
                abinfo.fragments = abinfo.bytes >> s->dma_adc.fragshift;
		spin_unlock_irqrestore(&s->lock, flags);
		return copy_to_user((void *)arg, &abinfo, sizeof(abinfo))
			? -EFAULT : 0;

        case SNDCTL_DSP_NONBLOCK:
		printk(KERN_DEBUG "SNDCTL_DSP_NONBLOCK\n");
                file->f_flags |= O_NONBLOCK;
                return 0;

        case SNDCTL_DSP_GETODELAY:
		printk(KERN_DEBUG "SNDCTL_DSP_GETODELAY\n");
		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		if (!s->dma_dac.ready && (val = prog_dmabuf_dac(s)) != 0)
			return val;
		spin_lock_irqsave(&s->lock, flags);
		ibmcsiti_update_ptr(s);
                count = s->dma_dac.count;
		spin_unlock_irqrestore(&s->lock, flags);
		if (count < 0)
			count = 0;
		return put_user(count, (int *)arg);

        case SNDCTL_DSP_GETIPTR:
		printk(KERN_DEBUG "SNDCTL_DSP_GETIPTR\n");
		if (!(file->f_mode & FMODE_READ))
			return -EINVAL;
		if (!s->dma_adc.ready && (val = prog_dmabuf_adc(s)) != 0)
			return val;
		spin_lock_irqsave(&s->lock, flags);
		ibmcsiti_update_ptr(s);
                cinfo.bytes = s->dma_adc.total_bytes;
		count = s->dma_adc.count;
		if (count < 0)
			count = 0;
                cinfo.blocks = count >> s->dma_adc.fragshift;
                cinfo.ptr = s->dma_adc.hwptr;
		if (s->dma_adc.mapped)
			s->dma_adc.count &= s->dma_adc.fragsize-1;
		spin_unlock_irqrestore(&s->lock, flags);
                return copy_to_user((void *)arg, &cinfo, sizeof(cinfo))
			? -EFAULT : 0;

        case SNDCTL_DSP_GETOPTR:
		printk(KERN_DEBUG "SNDCTL_DSP_GETOPTR\n");
		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		if (!s->dma_dac.ready && (val = prog_dmabuf_dac(s)) != 0)
			return val;
		spin_lock_irqsave(&s->lock, flags);
		ibmcsiti_update_ptr(s);
                cinfo.bytes = s->dma_dac.total_bytes;
		count = s->dma_dac.count;
		if (count < 0)
			count = 0;
                cinfo.blocks = count >> s->dma_dac.fragshift;
                cinfo.ptr = s->dma_dac.hwptr;
		if (s->dma_dac.mapped)
			s->dma_dac.count &= s->dma_dac.fragsize-1;
		spin_unlock_irqrestore(&s->lock, flags);
                return copy_to_user((void *)arg, &cinfo, sizeof(cinfo))
			? -EFAULT : 0;

        case SNDCTL_DSP_GETBLKSIZE:
		printk(KERN_DEBUG "SNDCTL_DSP_GETBLKSIZE\n");
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
		printk(KERN_DEBUG "SNDCTL_DSP_SETFRAGMENT  arg=%d\n", val);
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
		printk(KERN_DEBUG "SNDCTL_DSP_SUBDIVIDE\n");
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
		printk(KERN_DEBUG "SNDCTL_PCM_READ_RATE\n");
		return put_user(IBMCSI_DEFAULT_SAMPLING_RATE, (int *)arg);

        case SOUND_PCM_READ_CHANNELS:
		printk(KERN_DEBUG "SNDCTL_PCM_READ_CHANNELS\n");
		return put_user(2, (int *)arg);

        case SOUND_PCM_READ_BITS:
		printk(KERN_DEBUG "SNDCTL_PCM_READ_BITS\n");
		return put_user(16,(int *)arg);

        case SOUND_PCM_WRITE_FILTER:
        case SNDCTL_DSP_SETSYNCRO:
        case SOUND_PCM_READ_FILTER:
		printk(KERN_DEBUG "SNDCTL_xxx\n");
                return -EOPNOTSUPP;

	}
	return -EOPNOTSUPP;
}

static int ibmcsiti_dsp_release(struct inode *inode, struct file *file)
{
	struct ibmcsiti_state *s = (struct ibmcsiti_state *)file->private_data;

	VALIDATE_STATE(s);

	lock_kernel(); /* FIXME: do we need this? */

	if (file->f_mode & FMODE_WRITE)
		drain_dac(s, file->f_flags & O_NONBLOCK);

	down(&s->open_sem);
	if (file->f_mode & FMODE_WRITE) {
		stop_dac(s);
		del_timer_sync(&s->dac_timer); 
		synchronize_irq();
		dealloc_dmabuf(s, &s->dma_dac);
	}
	if (file->f_mode & FMODE_READ) {
		stop_adc(s);
		del_timer_sync(&s->adc_timer); 
		dealloc_dmabuf(s, &s->dma_adc);
	}
	s->open_mode &= (~file->f_mode) & (FMODE_READ|FMODE_WRITE);
	wake_up(&s->open_wait);

	up(&s->open_sem);
	unlock_kernel();

	return 0;
}



/*****************************************************************************/
/*	/dev/dsp suppport routines 					     */
/*****************************************************************************/

/*****************************************************************************/
/* 	Start DAC (Digital to Analog Conversion - Audio Playback)	     */
/*****************************************************************************/

/* 	Design Notes: 							      
 *
 * This routine is designed so it can be called safely as many times
 * as desired whenever it is necessary to ensure that the DAC process
 * is running, conditions permitting.
 *
 * In scatter/gather mode, we grab a descriptor (if available) and
 * enqueue it at the bottom of active descriptor chain. The
 * descriptors are configured to form a FIFO queue, not a loop /
 * ping-pong, to avoid replaying the same chunk of data. (*)
 *
 *   * Once a chunk of data has been processed, we need to remove it
 * from the queue/loop. We have to do such housekeeping on timer
 * interrupt; we cannot use DMA EOT interrupt, because the PPC 405 DMA
 * core traditionally halts scatter / gather DMA whenever it raises an
 * interrupt, until the interrupt is cleared. Since timer interrupts
 * are the lowest priority under Linux, they can be masked out under
 * stress conditions. Therefore, if we use a looping list / ping-pong
 * descriptors, we may not be able to remove an already processed
 * chunk of data in time, causing it to be played back again and again
 * as the DMAC loops through the list.
 *
 * When an underrun has occurred, the DMAC will be reading dummy
 * neutral data off the last (anchor) descriptor, which loops on
 * itself. In this case, we disable the CSI and halt the DMAC
 * temporarily, then enqueue the new descriptor and restart the
 * scatter/gather process afterwards.
 *
 * Whether we have reached the anchor descriptor, is determined by
 * reading the DMA source address and comparing it with the anchor's
 * dummy data address.  Since the desriptor address field in the DMAC
 * points to the NEXT descriptor to be used, we cannot use it to
 * determine whether we are at the anchor (underrun) or just before
 * anchor (normal case after the first pass).
 *
 * To avoid possible race conditions at the descriptor boundary
 * (i.e. DMAC going off to the next descriptor after we have read the
 * DMAC values), if the current descriptor has only a couple samples
 * to go, we wait in a loop until things stabilize.
 */
static void start_dac(struct ibmcsiti_state *s)
{
	unsigned long flags;
   	struct dma_sgdt *dt;
	unsigned int count; 

	DBG(printk("start_dac_sg state %8.8x count %d ready %d sgcnt %d hwptr %d\n", 
		   s->state, s->dma_dac.count, s->dma_dac.ready,
		   s->dma_dac.sg_count, s->dma_dac.hwptr));

	/* Spinlock held from here to end */
	spin_lock_irqsave(&s->lock, flags);

	if (s->dma_dac.sg_count) { /* There is data that needs to be posted */

		if (s->dma_dac.hwptr >= s->dma_dac.dmasize) {/* Sanity check */
			s->dma_dac.hwptr = 0;
		}

		count = s->dma_dac.sg_count; 
		if (s->dma_dac.hwptr + count > s->dma_dac.dmasize) {
			count = s->dma_dac.dmasize - s->dma_dac.hwptr; 
		}

		/* 4 bytes per sample, DMA count limited to 64K */ 
		if (count > 65536*4)
			count = 65536*4 ;

		if (count > 4) { /* Got enough data to do DMA */
			/* Get descriptor */
 			dt = get_sgdt(&(s->dac_free_sgdt_q));

   			if (dt) { /* Descriptor available */
				DBG(printk("new dt %8.8x ",dt)); 

		 		/* FIXME: Break data into
				 * fragments. Easiest way for now! */

				dt->ccw = (unsigned int) (IBMCSI_TXDMA_GO_NOI);
				dt->ctrl = 0x80000000 | ((count/4)&0xffff);
				dt->srcP = virt_to_phys(s->dma_dac.rawbuf
							+ s->dma_dac.hwptr);
				dt->destP = CSI0_TBUF;
				dt->nextP = virt_to_phys(s->dac_sgdt_lastV);

				asm volatile ("sync");

				DBG(printk("TX SG %8.8x, last %8.8x\n",
					   mfdcr(TX_SG),s->dac_sgdt_lastV));
				DBG(printk("TX SA %8.8x, count %d \n",
					   mfdcr(IBMCSI_TXDMA_SA),
					   mfdcr(IBMCSI_TXDMA_CT)));
				DBG(printk("last->phys %8.8lx\n",
					   s->dac_sgdt_lastV->srcP));

				if (mfdcr(DCRN_ASGC) & TX_SG_ENABLE) {
					unsigned int current_sa = mfdcr(IBMCSI_TXDMA_SA);
					if (current_sa == s->dac_sgdt_lastV->srcP) {
						s->dac_active_sgdt_q = dt;
						stop_dac(s);
						udelay(100);
					} else if (mfdcr(IBMCSI_TXDMA_CT) <= 2) {
						int timeout = 0;
						/* No time to safely
						 * reprogram this
						 * pass. Wait until
						 * this pass
						 * completes. */
						while (current_sa == mfdcr(IBMCSI_TXDMA_SA)) {
							timeout++;
							if (timeout > 1000000) {
								printk("ibmcsi: DMA timeout!!\n");
								break;
							}
						};

						if (mfdcr(IBMCSI_TXDMA_SA) == s->dac_sgdt_lastV->srcP) {
							stop_dac(s);
							udelay(100);
						}
					}
				} else {
					/* This gets programmed only if not active */
					s->dac_active_sgdt_q = dt;
				}

				/* Insert new dt between last anchor
				 * and previous data */

				dt->nextV = s->dac_sgdt_lastV;
				dt->prevV = s->dac_sgdt_lastV->prevV;

				if (dt->prevV) {
					dt->prevV->nextV = dt;
					dt->prevV->nextP = virt_to_phys(dt);
				}

				s->dac_sgdt_lastV->prevV = dt;

				if (mfdcr(DCRN_ASGC) & TX_SG_ENABLE) {
					if (mfdcr(TX_SG) == virt_to_phys(s->dac_sgdt_lastV) ) {
						/* Next descriptor is
						 * the bottom
						 * anchor */

						mtdcr(TX_SG, virt_to_phys(dt));
						/* Replace the next
						 * descriptor address
						 * with myself.  Not
						 * sure if this
						 * works */
					}
				}

				asm volatile ("sync");

				/* Update count and pointer */
				s->dma_dac.sg_count -= count;
				s->dma_dac.hwptr += count;
				if (s->dma_dac.hwptr >= s->dma_dac.dmasize)
					s->dma_dac.hwptr = 0;
			} else { /* End if descriptor available */
				printk("0dt\n");
			}

		      	/* If DMA is not already running, kick it. */
			if (! (mfdcr(DCRN_ASGC) & TX_SG_ENABLE))  {
				unsigned int current_csi_er 
					= IBMCSI_READ(CSI0_ER); 
	        		s->state |= IBMCSI_DAC_RUNNING;

				ibmcsi_stop_csi_sync(); /* FIXME: add timeout */  
				IBMCSI_WRITE(CSI0_ER,
					     IBMCSI_READ(CSI0_ER)
					     & ~CSI_ER_TXEN);

				/* Clear Terminal Count etc. */
				mtdcr(IBMCSI_DMA_SR, 
				      DCRN_DMA_SR_ALL(IBMCSI_TXDMA));

				/* Clear CSI  underrun */
				IBMCSI_WRITE(CSI0_SR, CSI_SR_TOD);

		        	/* Write address of the first
				 * scatter/gather descriptor table */
				mtdcr(TX_SG, 
				      virt_to_phys(s->dac_active_sgdt_q));

				/* Enable scatter/gather */
				mtdcr(DCRN_ASGC, mfdcr(DCRN_ASGC)
				      | TX_SG_ENABLE | TX_SG_MASK);

				/* Set up CSI config */ 
				IBMCSI_WRITE(CSI0_CFG, IBMCSI_TI_CFG);

				asm volatile ("sync");

				/* Start CSI, enable slot 0 and Tx. */
				IBMCSI_WRITE(CSI0_ER, 
					     current_csi_er | CSI_ER_ESLOT(0)
					     | CSI_ER_ESLOT(1)
					     | CSI_ER_TXEN | CSI_ER_ECSI);

				udelay(100);
				DBG(printk("CSI SA %8.8x, CT %8.8x",
					   mfdcr(IBMCSI_TXDMA_SA),
					   mfdcr(IBMCSI_TXDMA_CT)));
				DBG(printk("%2.2x %8.8x\n",
					   IBMCSI_READ(CSI0_SR),
					   mfdcr(DCRN_DMASR)));  
			}
		} /* End count not zero */
	}

	spin_unlock_irqrestore(&s->lock, flags);
}

/************************************************************************/
/* Start ADC (Analog to Digital Conversion - Audio Capture)		*/
/************************************************************************/

/*	Design Notes:
 *
 * Like its companion start_dac(), this routine is designed to be
 * called as many times as desired whenever it is necessary to ensure
 * that the ADC process is running, conditions permitting.
 *
 * Unlike DAC, ADC uses a ring descriptor scheme because we can simply
 * let the DMAC overwrite stale data when an overrun occurs. All the
 * descriptors in the ADC chain point to the same buffer (the entire
 * DMA buffer). In theory it might be enough to use just one
 * descriptor that loops on itself, but in this implementation we will
 * use redundant multiple descriptors, so that we can detect an
 * overrun condition trivially, by comparing the current descriptor
 * address with the previous one's. (This is done in the timer int
 * handler.)  So long as timer interrupts are not masked out for the
 * duration of the entire descriptor loop (in which case we are most
 * likely in much deeeper trouble than an overrun anyway), we should
 * be able to detect an overrun reliably in this manner.
 *
 * If we want to be truly paranoid, and/or have faith in kernel
 * timekeeping functions and incoming audio data rate, we could also
 * use timebase and fewer descriptors to do the same; you would still
 * have to worry about things like * timebase accuracy (e.g. tb_to_us
 * doesn't work with slow timebase clock).
 */

static void start_adc(struct ibmcsiti_state *s)
{
	unsigned long flags;
	
	spin_lock_irqsave(&s->lock, flags);

	if (! (mfdcr(DCRN_ASGC) & RX_SG_ENABLE)) { /* In hardware we trust */
		unsigned int current_csi_er = IBMCSI_READ(CSI0_ER); 

		s->state |= IBMCSI_ADC_RUNNING;

		ibmcsi_stop_csi_sync(); /* FIXME: add timeout */  
		IBMCSI_WRITE(CSI0_ER, IBMCSI_READ(CSI0_ER) & ~CSI_ER_RXEN );  
		
		/* Clear Terminal Count etc. for Rx channel */
		mtdcr(IBMCSI_DMA_SR, DCRN_DMA_SR_ALL(IBMCSI_RXDMA));

		/* Clear the CSI status flag for Rx */
		IBMCSI_WRITE(CSI0_SR, CSI_SR_ROD);

		/* Write the first scatter/gather descriptor address */
		mtdcr(RX_SG, virt_to_phys(s->adc_active_sgdt_q));

		s->adc_hw_prev_sgdt = s->adc_active_sgdt_q;  
		s->adc_sw_prev_sgdt = s->adc_active_sgdt_q;  
		s->dma_adc.hwptr = 0; /* We will never halt ADC so we can init here */
		s->dma_adc.swptr = 0; /* Otherwise we will have to
				       * figure out how to restart. */ 
		
		/* Enable scatter/gather DMA */
		mtdcr(DCRN_ASGC, mfdcr(DCRN_ASGC) | RX_SG_ENABLE | RX_SG_MASK);
		
		/* Set up CSI config */
		IBMCSI_WRITE(CSI0_CFG, IBMCSI_TI_CFG);

		asm volatile("sync");

		/* Start CSI, enable slot 0 and Rx */
		IBMCSI_WRITE(CSI0_ER, current_csi_er
			     | CSI_ER_ESLOT(0) | CSI_ER_ESLOT(1) 
			     | CSI_ER_RXEN | CSI_ER_ECSI );
		udelay(100);
	}	

	spin_unlock_irqrestore(&s->lock, flags);
}

static inline void stop_dac(struct ibmcsiti_state *s)
{
	unsigned long flags;

	spin_lock_irqsave(&s->lock, flags);

	/* Stop CSI synchronously - let transfer complete */
	ibmcsi_stop_csi_sync();
	IBMCSI_WRITE(CSI0_ER, IBMCSI_READ(CSI0_ER) & ~CSI_ER_TXEN);
	if (IBMCSI_READ(CSI0_ER) & CSI_ER_RXEN) {
		IBMCSI_WRITE(CSI0_ER, IBMCSI_READ(CSI0_ER) & ~CSI_ER_RXEN);
	}

	mtdcr(DCRN_ASGC,(mfdcr(DCRN_ASGC) | TX_SG_MASK) & ~TX_SG_ENABLE);

	/* Stop Tx DMA channel */
	mtdcr(IBMCSI_TXDMA_CR,
	      mfdcr(IBMCSI_TXDMA_CR) & ~DCRN_DMA_CR_CE);

	s->state &= ~IBMCSI_DAC_RUNNING;

	if (s->state & IBMCSI_ADC_RUNNING) { /* ADC was running */
		IBMCSI_READ(CSI0_RBUF); /* Dummy read */ 
		IBMCSI_WRITE(CSI0_SR, CSI_SR_ROD | CSI_SR_TOD); 
		IBMCSI_WRITE(CSI0_ER,
			     CSI_ER_ESLOT(0) | CSI_ER_ESLOT(1)
			     | CSI_ER_RXEN | CSI_ER_ECSI );
		udelay(100);
		DBG(printk("%2.2x %8.8x\n", IBMCSI_READ(CSI0_SR),
			   mfdcr(DCRN_DMASR))); 
	}

   	spin_unlock_irqrestore(&s->lock, flags);
}

static inline void stop_adc(struct ibmcsiti_state *s)
{
	unsigned long flags;

	spin_lock_irqsave(&s->lock, flags);

	/* Stop CSI synchronously - let transfer complete */
	ibmcsi_stop_csi_sync();
	IBMCSI_WRITE(CSI0_ER, IBMCSI_READ(CSI0_ER) & ~CSI_ER_RXEN);

	mtdcr(DCRN_ASGC,(mfdcr(DCRN_ASGC) | RX_SG_MASK) & ~RX_SG_ENABLE);

	mtdcr(IBMCSI_RXDMA_CR, mfdcr(IBMCSI_RXDMA_CR) & ~DCRN_DMA_CR_CE);
	s->state &= ~IBMCSI_ADC_RUNNING;

	if (s->state & IBMCSI_DAC_RUNNING) { /* DAC is still running */
		IBMCSI_WRITE(CSI0_ER, IBMCSI_READ(CSI0_ER) | CSI_ER_ECSI);
	}

	spin_unlock_irqrestore(&s->lock, flags);
}

/*
 * Globally stop the CSI core synchronously
 */
static int ibmcsi_stop_csi_sync()
{
	int retcode = 0; 
	unsigned int tbl_timeout;
	unsigned long count = 2000; /* 2ms */ 
	unsigned int current_csi_er = IBMCSI_READ(CSI0_ER);  

	DBG(printk("stop_csi: %8.8x\n", current_csi_er)); 

	/* Turn off CSI */ 	
	IBMCSI_WRITE(CSI0_ER, 0);
	eieio();  

	/* timebase 1 tick = 1000000 / (HZ * tb_ticks_per_jiffy) usec */
	/* (count) usec = count * HZ * tb_ticks_per_jiffy / 1000000 ticks */ 
	
	tbl_timeout = ( (count*tb_ticks_per_jiffy*HZ) + 999999 ) / 1000000;
	DBG(printk("tbl delta %d\n", tbl_timeout));

	tbl_timeout += get_tbl();

	while(IBMCSI_READ(CSI0_SR) & CSI_SR_CSIB) {
   		if (get_tbl() > tbl_timeout) {
			printk("ibmcsiti: Error: DMA timeout\n");
			retcode = -1;
         		break;
	        }
   	} /* Wait till CSI Busy turns off or timeout.   */
	udelay(1);
	
	IBMCSI_WRITE(CSI0_ER, current_csi_er & ~CSI_ER_ECSI);
	/* Globally disable CSI, preserve whichever channel was in use */

	return retcode;
}

/*****************************************************************************/
/* 	Allocate / initialize DMA buffer and related variables		     */
/*****************************************************************************/
static int prog_dmabuf(struct ibmcsiti_state *s, struct dmabuf *db,
		       unsigned rate, unsigned fmt, unsigned adc_init)
{
	int order;
	unsigned bytepersec;
	unsigned bufs;

	db->hwptr = db->swptr = db->total_bytes = db->count
		= db->error = db->endcleared = 0;

	if (! db->rawbuf) { /* DMA buffer not allocated yet, go get it. */
		db->ready = db->mapped = 0;
		for (order = DMABUF_DEFAULTORDER;
		     order >= DMABUF_MINORDER; order--) {
			db->rawbuf = (void *)__get_free_pages(GFP_KERNEL | GFP_DMA,
							      order);
			if (db->rawbuf) {
				db->dmaaddr = (dma_addr_t)virt_to_phys(db->rawbuf);
				/* db->hwptr = db->dmaaddr; */
				break;
			}
		}

		if (!db->rawbuf)
			return -ENOMEM;

 		DBG(printk("buforder: %8.8x\n", order));

		db->buforder = order;

		if (adc_init) {
			/* Program the count and address into all
			 * descriptors in the ring */
			/* Use all the queue for now */
			struct dma_sgdt *pWork = s->adc_free_sgdt_q;
			unsigned dma_count =
				((long)PAGE_SIZE << db->buforder) / 4;
		
			s->adc_active_sgdt_q = pWork;
	
			while (pWork) { /* Sanity check */
				pWork->ccw = (unsigned int)(IBMCSI_RXDMA_GO_NOI);
				pWork->srcP = CSI0_RBUF;
				pWork->destP = db->dmaaddr;
				pWork->ctrl = dma_count | 0x80000000;
 
				if (pWork->nextV == NULL) {
					/* End of chain reached */
					/* Create a double ring */
					pWork->nextV = s->adc_active_sgdt_q;
					s->adc_active_sgdt_q->prevV = pWork;
				}
				pWork->nextP = virt_to_phys(pWork->nextV);

				pWork = pWork->nextV;
				if (pWork == s->adc_active_sgdt_q)
					break;
			}
		}

	}

	/* FIXME: Hardcoded for 11.025 KHz */
	bytepersec = IBMCSI_DEFAULT_SAMPLING_RATE * 4;

	bufs = PAGE_SIZE << db->buforder;


	/* FIXME: decypher the following code block (from es1370.c) */
	if (db->ossfragshift) {
		if ((1000 << db->ossfragshift) < bytepersec)
			db->fragshift = ld2(bytepersec/1000);
		else
			db->fragshift = db->ossfragshift;
	} else {
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

   	DBG(printk("ibmcsi: fragsize was %d\n", db->fragsize));

	if (db->ossmaxfrags >= 4 && db->ossmaxfrags < db->numfrag)
		db->numfrag = db->ossmaxfrags;
	db->fragsamples = db->fragsize >> sample_shift[fmt];
	db->dmasize = db->numfrag << db->fragshift;
	/* End FIXME */

	if (adc_init)
		db->dmasize  = ((long)PAGE_SIZE << db->buforder);

	/* FIXME: Sample format hardcoded - assumes 0 is neutral sample. */
	memset(db->rawbuf, 0, db->dmasize);

	db->enabled = 1;
	db->ready = 1;
	return 0;
}

/*****************************************************************************/
/*	Prepare DMA buffer for ADC					     */
/*****************************************************************************/
static inline int prog_dmabuf_adc(struct ibmcsiti_state *s)
{
	stop_adc(s);

	/* FIXME: Sampling rate hardcoded at 11.025 KHz. */
	return prog_dmabuf(s, &s->dma_adc, IBMCSI_DEFAULT_SAMPLING_RATE, 0, 1);
}

/*****************************************************************************/
/* 	Prepare DMA buffer for DAC					     */
/*****************************************************************************/
static inline int prog_dmabuf_dac(struct ibmcsiti_state *s)
{
	stop_dac(s);

	/* FIXME: Sampling rate hardcoded at 44.1 KHz. */
	return prog_dmabuf(s, &s->dma_dac, IBMCSI_DEFAULT_SAMPLING_RATE, 0, 0);
}

static inline void dealloc_dmabuf(struct ibmcsiti_state *s, struct dmabuf *db)
{
	if (db->rawbuf)
		free_pages((unsigned long)db->rawbuf, db->buforder);

	db->rawbuf = NULL;
	db->mapped = db->ready = 0;
}



/*****************************************************************************/
/*	Procedure get_hwptr: Update DMA controller address offset hwptr	     */
/*		(returns # of bytes transferred)			     */
/*****************************************************************************/
static inline unsigned get_hwptr(struct ibmcsiti_state *s, struct dmabuf *db,
				 unsigned channel)
{
	unsigned hwptr, diff;
	switch (channel) {
    		case IBMCSI_TXDMA:	/* Tx DMA channel */
			hwptr = (unsigned)mfdcr(IBMCSI_TXDMA_SA)
				- (unsigned)(db->dmaaddr);
			/* Get source offset for next transfer on Tx
			 * DMA channel */
              		break;
               	case IBMCSI_RXDMA:	/* Rx DMA channel */
			hwptr = (unsigned)mfdcr(IBMCSI_RXDMA_DA)
				- (unsigned)(db->dmaaddr);
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
static inline void clear_advance(void *buf, unsigned bsize, unsigned bptr,
				 unsigned len, unsigned char c)
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
/*	* Must be called with spinlock held (FIXME: which one -dwg)	     */
/*****************************************************************************/
static void ibmcsiti_update_ptr(struct ibmcsiti_state *s)
{
	int diff;

	/* update ADC pointer */
	if (s->state & IBMCSI_ADC_RUNNING) {
		/* ADC channel (Rx DMA) */
		diff = get_hwptr(s, &s->dma_adc, IBMCSI_RXDMA);
		s->dma_adc.total_bytes += diff;
		s->dma_adc.count += diff;
		if (s->dma_adc.count >= (signed)s->dma_adc.fragsize) {
			wake_up(&s->dma_adc.wait);
		} else {
			DBG(printk("!@ %8.8x %8.8x\n", s->dma_adc.count,
				   s->dma_adc.fragsize));
		}

		if (!s->dma_adc.mapped) {
			if (s->dma_adc.count > (signed)(s->dma_adc.dmasize - ((3 * s->dma_adc.fragsize) >> 1))) {
				s->state &= ~IBMCSI_ADC_RUNNING;
				s->dma_adc.error++;
			} else {
				s->state &= ~IBMCSI_ADC_RUNNING;
				start_adc(s);
			}
		}
		/* FIXME: Give some thoughts about mmap support */
	}

	/* update DAC pointer */
	if (s->state & IBMCSI_DAC_RUNNING) {
		/* DAC channel (Tx DMA) */
		diff = get_hwptr(s, &s->dma_dac, IBMCSI_TXDMA);

		s->dma_dac.total_bytes += diff;
		if (s->dma_dac.mapped) { /* mapped = 0 if prepared */
			printk(KERN_ERR "DEBUG: ERR: mapped \n");
			s->dma_dac.count += diff;
			if (s->dma_dac.count >= (signed)s->dma_dac.fragsize)
				wake_up(&s->dma_dac.wait);
		} else {
			s->dma_dac.count -= diff;
			if (s->dma_dac.count <= 0) {
				/* Disable Tx DMA channel */
				mtdcr(IBMCSI_TXDMA_CR, 0);
                                s->state &= ~IBMCSI_DAC_RUNNING;
				s->dma_dac.error++;
			} else if ((s->dma_dac.count <= 
				    (signed)s->dma_dac.fragsize)
				   && !s->dma_dac.endcleared) {
				clear_advance(s->dma_dac.rawbuf,
					      s->dma_dac.dmasize,
					      s->dma_dac.swptr,
					      s->dma_dac.fragsize, 0);
				s->dma_dac.endcleared = 1;
			}

			s->state &= ~IBMCSI_DAC_RUNNING; /* Let start_dac() do it */

			if (s->dma_dac.count + (signed)s->dma_dac.fragsize
			    <= (signed)s->dma_dac.dmasize) {
				wake_up(&s->dma_dac.wait);
			} else
				start_dac(s);

		}
	}
}


/*****************************************************************************/
/* 	DAC (Playback) Interrupt Handler				     */
/*****************************************************************************/
static void ibmcsi_dac_timer(unsigned long param)
{
	struct ibmcsiti_state *s = (struct ibmcsiti_state *)param;
	struct dma_sgdt *pCurrent_dt, *pNext_dt, *pWork; 
	struct dma_sgdt *pAnchor = s->dac_sgdt_lastV;
	int count = 0;

	if ( (mfdcr(DCRN_ASGC) & TX_SG_ENABLE)	/* DAC running */
	     && (mfdcr(IBMCSI_TXDMA_CT) <= 4)	/* No time to do anything */
	     /* 4 samples = a little less than 100us at 44KHz */
	     && (mfdcr(IBMCSI_TXDMA_SA) != (pAnchor->srcP)) ) {
		udelay(100); /* Let the 4 samples run its course */
	}

	if (mfdcr(DCRN_ASGC) & TX_SG_ENABLE)  { /* DAC running */
		DBG(printk("t act %8.8x last %8.8x ", s->dac_active_sgdt_q,
			   s->dac_sgdt_lastV));
		DBG(printk("t"));

		if (pAnchor->prevV) { /* Data queued */
			/* Next S/G descriptor table */
			pNext_dt = (struct dma_sgdt *) phys_to_virt(mfdcr(TX_SG));

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
							DBG(printk("count %d free %8.8x ", count, pTemp));
							free_sgdt( &(s->dac_free_sgdt_q), pTemp);
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
				DBG(printk (" w"));
				wake_up(&s->dma_dac.wait);
			}
		}
	}

	mod_timer(&s->dac_timer, jiffies + DAC_TIMER_PERIOD);
}

static void ibmcsiti_dac_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	printk("ibmcsi: spurious dac int\n");
	return;
}


/*****************************************************************************/
/* 	ADC (Capture) Interrupt Handlers				     */
/*****************************************************************************/
static void ibmcsi_adc_timer(unsigned long param)
{
	struct ibmcsiti_state *s = (struct ibmcsiti_state *)param;
	unsigned long pWork;
	struct dma_sgdt *pWorkV; 
	unsigned long hwptr, diff = 0; 	

	if ( (mfdcr(DCRN_ASGC) & RX_SG_ENABLE) /* ADC running */
	     && (mfdcr(IBMCSI_RXDMA_CT) <= 4 )) { 
		/* Too close to descriptor changeover point */
		/* 4 samples = a little less than 100us at 44KHz */
		udelay(100); /* Let the 4 samples run its course */
	}
	
	if (mfdcr(DCRN_ASGC) & RX_SG_ENABLE) {
		pWork = mfdcr(RX_SG);
		hwptr = mfdcr(IBMCSI_RXDMA_DA);
		eieio();

		while ( (pWork != mfdcr(RX_SG))
			|| (hwptr != mfdcr(IBMCSI_RXDMA_DA)) ) {
			pWork = mfdcr(RX_SG);
			hwptr = mfdcr(IBMCSI_RXDMA_DA);
			eieio();
		} /* HACK: Loop until things stabilize */
	
		pWorkV = (struct dma_sgdt *) phys_to_virt(pWork);
		hwptr = hwptr - (unsigned)s->dma_adc.dmaaddr;
 
		if (pWorkV == NULL) {
			printk("ibmcsiti: INTERNAL ERROR! Bad sgdt@\n");
			stop_adc(s);
		} else {
			pWorkV = pWorkV->prevV;
			if (pWorkV == NULL) {
				printk(KERN_ERR "ibmcsiti: INTERNAL ERROR! "
				       "sgdt ring corrupted\n");
				stop_adc(s); 
			} else {/* No errors. pWork now has the
				 * current descriptor address */
				/* The following typically gets done
				 * in update_ptr(), but since the
				 * implementation for scatter/gather
				 * is different enough we'll do it
				 * here.  */ 

				DBG(printk("%p\n", pWorkV));

/*(Read timing changed to get a stable reading..	hwptr = (unsigned)mfdcr(IBMCSI_RXDMA_DA) - (unsigned)s->dma_adc.dmaaddr; */
				
				if (pWorkV == s->adc_sw_prev_sgdt) {
					/* Same descriptor as before? */
					/* Normally, software read
					 * pointer should be less than
					 * current DMA dest
					 * address. If not, we have an
					 * overrun big time - DMAC has
					 * run through all scatter /
					 * gather descriptors and came
					 * back full circle... (With
					 * the current design this
					 * should never happen, unless
					 * you reduce the number of
					 * sgdt's.)  */
					
					/* Normal case */
					if (s->dma_adc.swptr <= hwptr) {
						/* Must be '<=' not
						 * '<'. '=' can happen
						 * if interrupt
						 * happens when no
						 * data has yet been
						 * DMA'ed.  */
						if (hwptr >= s->dma_adc.hwptr) 
							diff = hwptr - s->dma_adc.hwptr;
						else {
							/* Big time overrun. */
							diff = 0;
							/* We are
							 * using the
							 * same
							 * descriptor
							 * as previous
							 * task time
							 * read; hwptr
							 * will only
							 * advance,
							 * unless it
							 * ran thru
							 * all the
							 * descriptor
							 * chain. */ 
						}
					} else {
						/* Overrun case */ 
						/* We now have buffer
						 * full of data
						 * starting at the
						 * current DMA dest
						 * address (which is
						 * the next one to be
						 * used by DMAC.  */ 
						printk("!0 %08lx %8.8x\n",
						       hwptr,
						       s->dma_adc.swptr);
						DBG(printk("0 %x %x\n",
							   IBMCSI_READ(CSI0_SR),
							   mfdcr(DCRN_DMASR)));
						diff = s->dma_adc.dmasize;  
						s->dma_adc.swptr = hwptr; 
					}
				} else if (pWorkV == s->adc_sw_prev_sgdt->nextV) {
					/* Next descriptor from previous */
					/* This time, software read
					 * pointer should be MORE than
					 * curent DMA dest address,
					 * since we wrapped around. If
					 * not, we are overrun - DMAC
					 * trampled data between
					 * software read pointer and
					 * current DMA address.  */

					if (s->dma_adc.swptr >= hwptr) {
						/* Normal case */
					 	/* Previous hwptr can be anywhere */
						if (s->dma_adc.hwptr >= hwptr) { /*Wrap*/
							diff = s->dma_adc.dmasize - 
								(s->dma_adc.hwptr - hwptr); 
						} else { /* Simple advance */
							DBG(printk("!1\n")); /* This is also normal */ 
							DBG(printk("1 %2.2x %8.8x\n",
								   IBMCSI_READ(CSI0_SR),
								   mfdcr(DCRN_DMASR)));
							diff = hwptr - s->dma_adc.hwptr;
						}
					} else { /* Overrun case */
						printk("!2\n"); 
						DBG(printk("2 %8.8x %8.8x %8.8x %8.8x\n",
							   IBMCSI_READ(CSI0_SR),
							   mfdcr(DCRN_DMASR),
							   IBMCSI_READ(CSI0_ER),
							   mfdcr(IBMCSI_RXDMA_DA) )); 
						diff = s->dma_adc.dmasize;  
#if 1
						s->dma_adc.swptr = hwptr; 
#else
						if (hwptr + 4 < s->dma_adc.dmasize) 
							s->dma_adc.swptr = hwptr+4; 
						else
							s->dma_adc.swptr = 0; 
#endif
					}
				} else { /* None of the above. We are overrun, big time. */
					printk("!3\n");
					DBG(printk("3 %2.2x %8.8x\n",
						   IBMCSI_READ(CSI0_SR), 
						   mfdcr(DCRN_DMASR))); 

					diff = s->dma_adc.dmasize;
					s->dma_adc.swptr = hwptr; 
					
				}	

				s->dma_adc.hwptr = hwptr; 
				s->dma_adc.total_bytes += diff;
				s->dma_adc.count += diff;
				s->adc_hw_prev_sgdt = pWorkV;

				if (s->dma_adc.count > s->dma_adc.dmasize) /* Overrun */
					s->dma_adc.count = s->dma_adc.dmasize;

				if (s->dma_adc.count >= (signed)s->dma_adc.fragsize) {
					wake_up(&s->dma_adc.wait);
				}

				mod_timer(&s->adc_timer, jiffies + ADC_TIMER_PERIOD);
			}
		}
	} else {
		mod_timer(&s->adc_timer, jiffies + ADC_TIMER_PERIOD); 
	}
	/* Any errors, we stop the ADC and refrain from restarting the
	 * timer. */ 
}

/************************************************************************/
/* ADC DMA interrupt (straightforward non-scatter/gather DMA)		*/ 
/************************************************************************/
static void ibmcsiti_adc_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	printk("ibmcsi: spurious adc int\n"); 
	return; 
}

/*****************************************************************************/
/* 	Drain DAC (flush playback buffer)				     */
/*****************************************************************************/
static int drain_dac(struct ibmcsiti_state *s, int nonblock)
{
#if 0 /* FIXME: Drain_dac not implemented yet. Timeout value depends
       * on sampling rate. */
	DECLARE_WAITQUEUE(wait, current);

	unsigned long flags;
	int count, tmo;

	if (s->dma_dac.mapped || !s->dma_dac.ready)
		return 0;

        add_wait_queue(&s->dma_dac.wait, &wait);
        for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);
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
			DBG(printk(KERN_DEBUG "ibmcsiti: DMA timed out??\n");)
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

/* Reformat and copy device format data to user format data (for
 * capture)
 */
static unsigned long copy_samples_to_user(char *dest, const char *src,
					  unsigned nsamples, int stereo)
{
	u16 *from = (u16 *)src;
	u16 *to = (u16 *)dest;
	int i;

	if (stereo) {
		/* 16-bit stereo => 4 bytes per sample */
		if (copy_to_user(dest, src, nsamples*4) != 0)
			return -EFAULT;
	} else {
		/* convert stereo to mono as we copy */
		for (i = 0; i < nsamples; i++) {
			if (put_user(*from, to) != 0)
				return -EFAULT;
			
			from += 2; /* throw away the right channel info */
			to++;
		}
	}

	return 0;
}

/* Reformat and copy user format data to device format data (for
 * playback)
 */
static unsigned long copy_samples_from_user(char *dest, const char *src,
					    unsigned nsamples, int stereo)
{
	u16 *from = (u16 *)src;
	u16 *to = (u16 *)dest;
	u16 temp;
	int i;

	if (stereo) {
		/* 16-bit stereo => 4 bytes per sample */
		if (copy_from_user(dest, src, nsamples*4) != 0)
			return -EFAULT;
	} else {
		/* convert mono to stereo as we copy */
		for (i=0; i < nsamples; i++) {
			if (get_user(temp, from) != 0)
				return -EFAULT;
			
			*(to++) = temp;
			*(to++) = 0; /* Pad right-channel with zero data */
			from++;
		}
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

/*****************************************************************************/
/* 	Scatter/Gather descriptor queue handlers			     */
/*****************************************************************************/
/* FIXME: spinlocks TBD (rely on global spinlocks for now) */

/* Initialize logical (virtual address) pool of descriptor tables */
static void init_sgdt_q(struct dma_sgdt *queue, int count)
{
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
	for (i = 0 ;i < count; i++) {
		work[i].ccw = 0;
		work[i].srcP = 0;
		work[i].destP = 0;
		work[i].ctrl = 0;
	}
}

/* Get a descriptor table from free pool */
static struct dma_sgdt *get_sgdt(struct dma_sgdt **queueaddress)
{
	struct dma_sgdt *work = *queueaddress;

	DBG(printk(">"));
	if (*queueaddress == NULL)
		return NULL; 
	
	work->prevV = NULL; /* Unlink backward */ 
	*queueaddress = work->nextV; /* Update queue pointer */ 

	if (*queueaddress) 
		(*queueaddress)->prevV = NULL;
	
	work->nextV = NULL; /* Unlink forward */ 

	DBG(printk("get_sgdt q %8.8x ret %8.8x\n", *queueaddress, work));
	return work; 
}

/* Return a descriptor table to free pool */ 
static void free_sgdt(struct dma_sgdt **queueaddress, struct dma_sgdt *dt)
{
	struct dma_sgdt *work = *queueaddress;
	DBG(printk("free_sgdt %8.8x queue %8.8x ", dt, work));
	DBG(printk("<")); 

	*queueaddress = dt;	/* Update queue read pointer */
	dt->prevV = NULL; /* Unlink backward */
	dt->nextV = work; /* Link forward */
	if (work) work->prevV = dt; /* Link backward if non-null */
		
	DBG(printk("now %8.8x\n",*queueaddress)); 
}

/* Check the range of descriptor */
static unsigned int check_sgdt_range(struct ibmcsiti_state *s,
				     struct dma_sgdt *dt, int count) 
{
#if 1
	return 1; 
#else
	if ( (struct dma_sgdt *)s->write_line <= dt ) && 
	     (dt <= (struct dma_sgdt *)s->write_line + count) {
		return 1;
	}		 
	else return 0; 
#endif
}

/************************************************************************/
/* Part II : Mixer device /dev/mixer					*/
/************************************************************************/

/* The TLV320AIC23 codec supports the following channels:*/
/* 	- Master Volume (Line out; 1.0V RMS)			*/
/*	- Alt PCM 	(Speaker; 16-32 ohm impedance mapped to Alt PCM, per AC'97)*/
/* 	- Line 		(Line in)					*/
/*	- Mic		(Mic in)					*/
/* Although the chip supports individual left/right channel volume control,*/
/* both channels will always be set to the same value. */


static int ibmcsiti_mixer_open(struct inode *inode, struct file *file)
{
	int minor = MINOR(inode->i_rdev);
	struct list_head *list;
	struct ibmcsiti_state *s;

	for (list = devs.next; ; list = list->next) {
		if (list == &devs)
			return -ENODEV;
		s = list_entry(list, struct ibmcsiti_state, devs);
		if (s->dev_mixer == minor)
			break;
	}
       	VALIDATE_STATE(s);
	file->private_data = s;
	init_MUTEX(&s->mix_sem);
	return 0;
}

static int ibmcsiti_mixer_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int ibmcsiti_mixer_ioctl(struct inode *ioctl, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	struct ibmcsiti_state *s = file->private_data;
	const unsigned int nr = _IOC_NR(cmd);
	int ret = 0;

	switch (cmd) {
	case SOUND_MIXER_INFO: {
		struct mixer_info info;

		strncpy(info.id, "IBMCSITI", sizeof(info.id));
		strncpy(info.name, "IBM 405LP CSI+TLV320", sizeof(info.name));
		info.modify_counter = 0;
		if (copy_to_user((void *)arg, &info, sizeof(info)))
			ret = -EFAULT;
	}
		break;

	case SOUND_OLD_MIXER_INFO: {
		struct _old_mixer_info info;

		strncpy(info.id, "IBMCSITI", sizeof(info.id));
		strncpy(info.name, "IBM 405LP CSI+TLV320", sizeof(info.name));
		if (copy_to_user((void *)arg, &info, sizeof(info)))
			ret = -EFAULT;
	}
		break;

	case OSS_GETVERSION:
		ret = put_user(SOUND_VERSION, (int *)arg);
		break;

	default:
		if (_IOC_TYPE(cmd) != 'M' || _SIOC_SIZE(cmd) != sizeof(int)) {
			ret = -EINVAL;
			break;
		}

		down(&s->mix_sem);
		
		if (_SIOC_DIR(cmd) == _SIOC_READ)
			ret = mixer_read_ioctl(s, nr, (caddr_t) arg);
		else if ( (_SIOC_DIR(cmd) == _SIOC_WRITE) ||
			  (_SIOC_DIR(cmd) == (_SIOC_READ | _SIOC_WRITE) ) )
			ret = mixer_write_ioctl(s, nr, (caddr_t) arg);
		else
			ret = -EINVAL;
		
		up(&s->mix_sem);
	}

	return ret;
}

static int mixer_read_ioctl(struct ibmcsiti_state *s, unsigned int nr,
			    caddr_t arg)
{
	int val = -1;

	switch (nr) {
	case SOUND_MIXER_CAPS:
		/* Cannot support mixing mic and line in */
		val = SOUND_CAP_EXCL_INPUT; 
		break;

	case SOUND_MIXER_DEVMASK:
		val = (SOUND_MASK_VOLUME | SOUND_MASK_LINE /* Line in/out */
		       | SOUND_MASK_MIC );
		break;

	case SOUND_MIXER_STEREODEVS:
		val = SOUND_MASK_VOLUME;
		break;

	case SOUND_MIXER_OUTMASK:
		val = 0 ; /* No routing of inputs to outputs */
		break;

	case SOUND_MIXER_RECMASK: /* Recording devices */
		val = (SOUND_MASK_LINE | SOUND_MASK_MIC);
		break;

	case SOUND_MIXER_VOLUME: /* Line out volume */
		val = ibmcsiti_get_volume(s);
		break;

	case SOUND_MIXER_LINE: 	/* Line in volume */
		val = ibmcsiti_get_line(s);
		break;

	case SOUND_MIXER_MIC:	/* Mic in volume */
		val = ibmcsiti_get_mic(s);
		break;

	case SOUND_MIXER_OUTSRC: /* Unmuted output sources */
		val = ibmcsiti_get_outsrc(s);
		break;

	case SOUND_MIXER_RECSRC: /* Unmuted recording sources */
		val = ibmcsiti_get_recsrc(s);
		break;
	default:
		return -EINVAL;
	}
	return put_user(val, (int *) arg);
}

static int mixer_write_ioctl(struct ibmcsiti_state *s, unsigned int nr,
			     caddr_t arg)
{
	int val;

	if (get_user(val, (int *) arg) != 0)
		return -EFAULT;

	switch (nr) {
	case SOUND_MIXER_VOLUME:
		val = ibmcsiti_set_volume(s, val);
		break;

	case SOUND_MIXER_LINE:
		val = ibmcsiti_set_line(s, val>>8);
		break;

	case SOUND_MIXER_MIC:
		val = ibmcsiti_set_mic(s, val>>8);
		break;

	case SOUND_MIXER_RECSRC:
		val = ibmcsiti_set_recsrc(s, val);
		break;

	case SOUND_MIXER_OUTSRC:
		/* FIXME: we only have one output source, but we
		 * should probably let it be enabled and disabled
		 * through this */
		val = ibmcsiti_get_outsrc(s);
		break;

	default:
		return -EINVAL;
	}
	if (val < 0)
		return val;
	return put_user(val, (int *) arg);
}


/*
 * Individual mixer ioctl() routines
 */

/*Read Headphone/line out volume */
static int ibmcsiti_get_volume(struct ibmcsiti_state *s)
{
	int val;
	u16 lv = tlv320_read_reg(s, TLV320_LCH) & TLV320_LCH_LHV;
	u16 rv = tlv320_read_reg(s, TLV320_RCH) & TLV320_RCH_RHV;
	u16 mute = tlv320_read_reg(s, TLV320_DAP) & TLV320_DAP_DACM;

	if (mute) {
		val = 0;
	} else {
		/* The volume field is 7-bits, but values 48 and below
		 * are all treated as muted */
		lv = 100 * (lv - 48) / 79;
		rv = 100 * (rv - 48) / 79;
		val = lv | (rv << 8);
	}

	return val;
}


/*****************************************************************************/
/* 	Read Line In volume (VOLUME)					     */
/*****************************************************************************/
static int ibmcsiti_get_line(struct ibmcsiti_state *s)
{
	int val;
	u16 temp = tlv320_read_reg(s, TLV320_LLI);
	/* Left channel input mute bit 7 (1=mute)			*/
	/* Left channel input volume bits 4:0, 10111 = 0dB. 0-31      	*/
	/* 00000 = -34.5 dB : low enough to be considered 0...          */

	if (temp & TLV320_LLI_LIM)
		val = 0;
	else
		val = 100 * (temp & TLV320_LLI_LIV) / TLV320_LLI_LIV;

	return val;
}

/*****************************************************************************/
/*	Read Mic In volume (MIC)					     */
/*****************************************************************************/
static int ibmcsiti_get_mic(struct ibmcsiti_state *s)
{
	int val ;
	u16 temp = tlv320_read_reg(s, TLV320_AAP);

	if (temp & TLV320_AAP_MICM)
		val = 0;	/* Mute */
	else if (temp & TLV320_AAP_MICB)
		val = 100; /* 20dB boost */
	else
		val = 50; /* 0dB boost */	

	return val;
}

/*****************************************************************************/
/*	Read Recording Source						     */
/*****************************************************************************/
static int ibmcsiti_get_recsrc(struct ibmcsiti_state *s)
{
	int val;  
	u16 aap = tlv320_read_reg(s, TLV320_AAP);

	if (aap & TLV320_AAP_INSEL) {
		if (aap & TLV320_AAP_MICM)
			val = 0;
		else
			val = SOUND_MASK_MIC;
	} else {
		val = SOUND_MASK_LINE;
	}
	return val;
}

/* Read Output Source */
static int ibmcsiti_get_outsrc(struct ibmcsiti_state *s)
{
	
	int val = 0;
		
 	if (! ibmcsiti_get_volume(s))
		val |= SOUND_MASK_VOLUME;

	return val; 
}

/* Set master (line out / headphone) volume (VOLUME) */
static int ibmcsiti_set_volume(struct ibmcsiti_state *s, int val)
{
	u16 lv = val & 0xff;
	u16 rv = (val >> 8) & 0xff;
	u16 dap = tlv320_read_reg(s, TLV320_DAP);

	if ( (lv == 0) && (rv == 0) ) {
		/* Mute... */
		tlv320_write_reg(s, TLV320_DAP, dap | TLV320_DAP_DACM);
	} else {
		/* Clamp: */
		if (lv > 100)
			lv = 100;
		if (rv > 100)
			rv = 100;

		/* Scale: headphone volumes go from 127 (+6dB) to 48
		 * (-73dB == mute) everything less is also mute */
		lv = (79 * lv / 100) + 48;
		rv = (79 * rv / 100) + 48;

		/* Update: */
		tlv320_write_reg(s, TLV320_LCH, lv);
		tlv320_write_reg(s, TLV320_RCH, rv);
		tlv320_write_reg(s, TLV320_DAP, dap & ~TLV320_DAP_DACM);
	}

	return ibmcsiti_get_volume(s);
}

/*****************************************************************************/
/*	Set Line In volume (LINE)					     */
/*****************************************************************************/
static int ibmcsiti_set_line(struct ibmcsiti_state *s, int val) {
	u16 reg = tlv320_read_reg(s, TLV320_LLI) | TLV320_LLI_LRS;

	if (val <= 0) {
		reg |= TLV320_LLI_LIM; /* Mute */ 
	} else {
		if (val > 100)
			val = 100; /* clamp volume */

		/* unmute, clear volume */
		reg &= ~(TLV320_LLI_LIM | TLV320_LLI_LIV);
		reg |= val * TLV320_LLI_LIV / 100; /* fill in volume */
	}
	tlv320_write_reg(s, TLV320_LLI, reg);
	tlv320_write_reg(s, TLV320_RLI, reg); /* For good measure */

	return ibmcsiti_get_line(s);
}

/*****************************************************************************/
/*	Set Mic In volume (MIC)						     */
/*****************************************************************************/
static int ibmcsiti_set_mic(struct ibmcsiti_state *s, int val)
{
	u16 reg = tlv320_read_reg(s, TLV320_AAP);

	if (val <= 0) { /* Mute */
		reg |= TLV320_AAP_MICM;
	} else {
		reg &= ~TLV320_AAP_MICM; /* Unmute */

		if (val <= 50) /* 0dB boost */
			reg &= ~TLV320_AAP_MICB;
		else
			reg |= TLV320_AAP_MICB;
	}

	tlv320_write_reg(s, TLV320_AAP, reg);

	return ibmcsiti_get_mic(s);
}

/*****************************************************************************/
/* 	Set Recording Source 						     */
/*****************************************************************************/
static int ibmcsiti_set_recsrc(struct ibmcsiti_state *s, int val)
{
	if (val & SOUND_MASK_MIC) {
		/* Select Mic and unmute it */
		tlv320_write_reg(s, TLV320_AAP,
				   (tlv320_read_reg(s, TLV320_AAP)
				    | TLV320_AAP_INSEL) & ~TLV320_AAP_MICM);

	}
	if (val & SOUND_MASK_LINE) {
		/* Select Line In */
		tlv320_write_reg(s, TLV320_AAP,
				   (tlv320_read_reg(s, TLV320_AAP)
				    & ~TLV320_AAP_INSEL) );
	}
	if (! (val & (SOUND_MASK_LINE | SOUND_MASK_MIC)) ) {
		/* Select Mic and mute it */ 
		tlv320_write_reg(s, TLV320_AAP,
				   tlv320_read_reg(s, TLV320_AAP)
				   | TLV320_AAP_INSEL | TLV320_AAP_MICM);
	}

	return ibmcsiti_get_recsrc(s);
}

/************************************************************************/
/* Low level codec routines						*/
/************************************************************************/

static u16 tlv320_read_reg( struct ibmcsiti_state *s, int reg)
{
	if (reg >= TLV320_REG_EXTENT)
		return 0xffff; /* as good as any bad value */
	return s->codec_reg[reg];
}

static void tlv320_write_reg(struct ibmcsiti_state *s, int reg, u16 val)
{
	u16 temp = TLV320_REG_WRITE(reg, val);
	int i;
	int err;
	
	if (reg >= TLV320_REG_EXTENT)
		BUG();

	for (i=0; i<5; i++) {
		err = i2c_master_send(s->i2c, (u8 *)&temp, sizeof(temp));
		if (err == sizeof(temp)) {
			s->codec_reg[reg] = val; 	
			udelay(100);
			return;
		}
		printk("ibmcsiti: I2C write retry!\n");
		udelay(100);
	}

	printk("ibmcsiti: I2C write error.\n");
}

