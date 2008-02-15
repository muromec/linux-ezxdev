/*------------------------------------------------------------------------
 . smc91x.c
 . This is a driver for SMSC's 91C9x/91C1xx single-chip Ethernet devices.
 .
 . Copyright (C) 1996 by Erik Stahlman
 . Copyright (C) 2001 Standard Microsystems Corporation
 .	Developed by Simple Network Magic Corporation
 . Copyright (C) 2003 Monta Vista Software, Inc.
 .	Unified SMC91x driver by Nicolas Pitre
 .
 . This program is free software; you can redistribute it and/or modify
 . it under the terms of the GNU General Public License as published by
 . the Free Software Foundation; either version 2 of the License, or
 . (at your option) any later version.
 .
 . This program is distributed in the hope that it will be useful,
 . but WITHOUT ANY WARRANTY; without even the implied warranty of
 . MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 . GNU General Public License for more details.
 .
 . You should have received a copy of the GNU General Public License
 . along with this program; if not, write to the Free Software
 . Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 .
 . Arguments:
 . 	io	= for the base address
 .	irq	= for the IRQ
 .	nowait	= 0 for normal wait states, 1 eliminates additional wait states
 .
 . original author:
 . 	Erik Stahlman <erik@vt.edu>
 .
 . hardware multicast code:
 .    Peter Cammaert <pc@denkart.be>
 .
 . contributors:
 . 	Daris A Nevil <dnevil@snmc.com>
 .      Nicolas Pitre <nico@cam.org>
 .
 . History:
 .   08/20/00  Arnaldo Melo       fix kfree(skb) in smc_hardware_send_packet
 .   12/15/00  Christian Jullien  fix "Warning: kfree_skb on hard IRQ"
 .   03/16/01  Daris A Nevil      modified smc9194.c for use with LAN91C111
 .   08/22/01  Scott Anderson     merge changes from smc9194 to smc91111
 .   08/21/01  Pramod B Bhardwaj  added support for RevB of LAN91C111
 .   12/20/01  Jeff Sutherland    initial port to Xscale PXA with DMA support
 .   04/07/03  Nicolas Pitre      unified SMC91x driver, killed irq races,
 .                                more bus abstraction, big cleanup, etc.
 ----------------------------------------------------------------------------*/

static const char version[] =
	"smc91x.c: v1.0, mar 07 2003 by Nicolas Pitre <nico@cam.org>\n";

/* Debugging level */
#ifndef SMC_DEBUG
#define SMC_DEBUG		0
#endif


#include <linux/config.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/ioport.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#ifdef CONFIG_PM
#include <linux/pm.h>
#endif
#ifdef CONFIG_DPM
#include <linux/device.h>
#endif

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/irq.h>

#include "smc91x.h"


#ifdef CONFIG_ISA
/*
 . the LAN91C111 can be at any of the following port addresses.  To change,
 . for a slightly different card, you can add it to the array.  Keep in
 . mind that the array must end in zero.
*/
static unsigned int smc_portlist[] __initdata = {
	0x200, 0x220, 0x240, 0x260, 0x280, 0x2A0, 0x2C0, 0x2E0,
	0x300, 0x320, 0x340, 0x360, 0x380, 0x3A0, 0x3C0, 0x3E0, 0
};
#endif  /* CONFIG_ISA */

#ifndef SMC_IOADDR
# define SMC_IOADDR		-1
#endif
static int io = SMC_IOADDR;

#ifndef SMC_IRQ
# define SMC_IRQ		-1
#endif
static int irq = SMC_IRQ;

#ifndef SMC_NOWAIT
# define SMC_NOWAIT		0
#endif
static int nowait = SMC_NOWAIT;

MODULE_PARM(io, "i");
MODULE_PARM(irq, "i");
MODULE_PARM(nowait, "i");
MODULE_PARM_DESC(io, "I/O base address");
MODULE_PARM_DESC(irq, "IRQ number");
MODULE_PARM_DESC(nowait, "set to 1 for no wait state");


/*------------------------------------------------------------------------
 .
 . The internal workings of the driver.  If you are changing anything
 . here with the SMC stuff, you should have the datasheet and know
 . what you are doing.
 .
 -------------------------------------------------------------------------*/
#define CARDNAME "LAN91x"

// Use power-down feature of the chip
#define POWER_DOWN		1

/*
 . Wait time for memory to be free.  This probably shouldn't be
 . tuned that much, as waiting for this means nothing else happens
 . in the system
*/
#define MEMORY_WAIT_TIME	16

/*
 . This selects whether TX packets are sent one by one to the SMC91x internal
 . memory ans throttled until transmission completes.  This may prevent
 . RX overruns a litle by keeping much of the memory free for RX packets
 . but to the expense of reduced TX throughput and increased IRQ overhead.
 . Note this is not a cure for a too slow data bus or too high IRQ latency.
 */
#define THROTTLE_TX_PKTS	0


typedef enum {
        SMC_PXA_DMA_STATE_IDLE = 0,
        SMC_PXA_DMA_STATE_RX = 1,
        SMC_PXA_DMA_STATE_TX = 2,
        SMC_PXA_DMA_STATE_UNSURE = 3,
} smc_pxa_dma_state_t;

/* store this information for the driver.. */
struct smc_local {

	// If I have to wait until memory is available to send
	// a packet, I will store the skbuff here, until I get the
	// desired memory.  Then, I'll send it out and free it.
	struct sk_buff *saved_skb;

 	// these are things that the kernel wants me to keep, so users
	// can find out semi-useless statistics of how well the card is
	// performing
	struct net_device_stats stats;

	// version/revision of the SMC91x chip
	int version;

	// Set to true during the auto-negotiation sequence
	int autoneg_active;

	// Address of our PHY port
	int	phyaddr;

	// Type of PHY
	int	phytype;

	// Last contents of PHY Register 18
	int	lastPhy18;

	// Contains the current active transmission mode
	int	tcr_cur_mode;

	// Contains the current active receive mode
	int	rcr_cur_mode;

	// Contains the current active receive/phy mode
	int	rpc_cur_mode;
	int	ctl_autoneg;
	int	ctl_rfduplx;
	int	ctl_rspeed;

#ifdef CONFIG_PM
	struct	pm_dev* pm;
#endif

#ifdef SMC_USE_PXA_DMA
	int		dma;

        /* SMC interrupts raised during  */
        int             saved_mask;

        /* Represents the current DMA operation performed */
        volatile smc_pxa_dma_state_t dma_state;

	struct sk_buff	*rx_skb;
	dma_addr_t	rx_dma_buf;
	int		rx_dma_len;
	struct sk_buff	*tx_skb;
	dma_addr_t	tx_dma_buf;
	int		tx_dma_len;
	struct sk_buff	*wait_dma_skb;
#endif

};

static struct net_device *global_dev = NULL;  /* needs to be fixed */

#ifdef CONFIG_DPM

static int smc91x_suspend(struct device * dev, u32 state, u32 level);
static int smc91x_resume(struct device * dev, u32 level);
static int smc91x_scale(struct bus_op_point * op, u32 level);

static void smc_tx(struct net_device *dev);


static struct device_driver smc91x_driver_ldm = {
	name:      	"smc91x",
	devclass:  	NULL,
	probe:     	NULL,
	suspend:   	smc91x_suspend,
	resume:    	smc91x_resume,
	scale:	  	smc91x_scale,
	remove:    	NULL,
};

static struct device smc91x_device_ldm = {
	name:		"SMC91X Ethernet Controller",
	bus_id:		"net",
	driver: 		NULL,
	power_state:	DPM_POWER_ON,
};


static void smc91x_ldm_register(struct net_device *netdev)
{
#ifdef CONFIG_ARCH_MAINSTONE
	extern void pxaebc_driver_register(struct device_driver *driver);
	extern void pxaebc_device_register(struct device *device);
  
	pxaebc_driver_register(&smc91x_driver_ldm);
	pxaebc_device_register(&smc91x_device_ldm);
#endif
#ifdef CONFIG_MACH_OMAP_PERSEUS2
	extern void mpu_public_driver_register(struct device_driver *driver);
	extern void mpu_public_device_register(struct device *device);

	smc91x_device_ldm.platform_data = (void *) netdev;
	mpu_public_driver_register(&smc91x_driver_ldm);
	mpu_public_device_register(&smc91x_device_ldm);
#endif
#ifdef CONFIG_ROOT_NFS
   {
	   extern struct device *nfs_root_dev;
	   nfs_root_dev = &smc91x_device_ldm;
   }
#endif
}

static void smc91x_ldm_unregister(struct net_device *netdev)
{
#ifdef CONFIG_ARCH_MAINSTONE
	extern void pxaebc_driver_unregister(struct device_driver *driver);
	extern void pxaebc_device_unregister(struct device *device);

	pxaebc_driver_unregister(&smc91x_driver_ldm);
	pxaebc_device_unregister(&smc91x_device_ldm);
#endif
#ifdef CONFIG_MACH_OMAP_PERSEUS2
	extern void mpu_public_driver_unregister(struct device_driver *driver);
	extern void mpu_public_device_unregister(struct device *device);

	mpu_public_driver_unregister(&smc91x_driver_ldm);
	mpu_public_device_unregister(&smc91x_device_ldm);
	smc91x_device_ldm.platform_data = NULL;
#endif
}
#endif /* CONFIG_DPM */

#ifdef SMC_USE_PXA_DMA
static void smc_pxa_dma_irq(int dma, void *dev_ptr, struct pt_regs *regs);
#endif

#if SMC_DEBUG > 2
#define PRINTK3(args...)  printk(args)
#else
#define PRINTK3(args...)  do { } while(0)
#endif

#if SMC_DEBUG > 1
#define PRINTK2(args...)  printk(args)
#else
#define PRINTK2(args...)  do { } while(0)
#endif

#if SMC_DEBUG > 0
#define PRINTK1(args...)  printk(args)
#define PRINTK0(args...)   printk(args)
#define PRINTK(args...)   printk(args)
#else
#define PRINTK1(args...)  do { } while(0)
#define PRINTK0(args...)  do { } while(0)
#define PRINTK(args...)   printk(KERN_DEBUG args)
#endif

#if SMC_DEBUG > 3
static void PRINT_PKT(u_char *buf, int length)
{
	int i;

        for (i = 0; i < length; i++)
                printk("%02x%s", *buf++,
                       ((i + 1) & 0xf) ? ((i & 1) ? " " : "") : "\n");
	printk("\n");
}
#else
#define PRINT_PKT(x...)  do { } while(0)
#endif


/* this enables an interrupt in the interrupt mask register */
#define SMC_ENABLE_INT(irq) do {						\
	unsigned long flags;						\
	unsigned char mask;						\
	local_irq_save(flags);						\
	mask = SMC_GET_INT_MASK();					\
	mask |= (irq);							\
	SMC_SET_INT_MASK(mask);						\
        PRINTK2("  SMC_ENABLE_INT(0x%x) at %s:%d\n", (irq), __FILE__, __LINE__); \
	local_irq_restore(flags);					\
} while (0)

/* this disables an interrupt from the interrupt mask register */
#define SMC_DISABLE_INT(irq) do {						\
	unsigned long flags;						\
	unsigned char mask;						\
	local_irq_save(flags);						\
	mask = SMC_GET_INT_MASK();					\
	mask &= ~(irq);							\
	SMC_SET_INT_MASK(mask);						\
        PRINTK2("  SMC_DISABLE_INT(0x%x) at %s:%d\n", (irq), __FILE__, __LINE__);\
	local_irq_restore(flags);					\
} while (0)

/* wait while MMU is busy */
#define SMC_WAIT_MMU_BUSY() do {					\
	if (unlikely(SMC_GET_MMU_CMD() & MC_BUSY)) {                    \
		unsigned long timeout = jiffies + 2;			\
		while (SMC_GET_MMU_CMD() & MC_BUSY) {			\
			if (time_after(jiffies, timeout)) {		\
				printk("%s: timeout %s line %d\n",	\
					dev->name, __FILE__, __LINE__);	\
				break;					\
			}						\
		}							\
	}								\
} while (0)

#if SMC_DEBUG > 0

#define SMC_CHECK_POINTER(p) ((((uint32_t)p) & 0xf0000000) != 0xc0000000)

#define SMC_CHECK_SKB(skb)                                              \
        (SMC_CHECK_POINTER(skb) || SMC_CHECK_POINTER(skb->data) ||      \
         SMC_CHECK_POINTER(skb->tail) || SMC_CHECK_POINTER(skb->end))

#define SMC_TEST_SKB(test_skb)                                          \
        do {                                                            \
                if (SMC_CHECK_SKB(test_skb)) {                          \
                        PRINTK2("!!!%s:%d: Invalid skb:"                \
                                " skb=0x%p, skb->data=0x%p\n",          \
                                __FILE__, __LINE__,                     \
                                test_skb, test_skb->data);              \
                        BUG();                                          \
                }                                                       \
        } while (0)

#else
#define SMC_TEST_SKB(test_skb) do { } while(0)
#endif


#ifdef SMC_USE_PXA_DMA

/*
 * DMA stuff for XScale PXA2xx.
 */

#define SMC_PXA_DMA_THRESHOLD    0


/* Wait untile DMA transfer finishes */
static inline void
smc_pxa_dma_wait_busy(struct net_device *dev)
{
	struct smc_local *lp = (struct smc_local *)dev->priv;

        if (lp->dma == -1)
                return;

	while (DCSR(lp->dma) & DCSR_RUN);
}

/* Stop DMA transfer */
static void
smc_pxa_dma_stop(struct net_device * dev)
{
        u32 dcsr; 
	struct smc_local *lp = (struct smc_local *)dev->priv;

        if (lp->dma == -1)
                return;

        dcsr = DCSR(lp->dma);
        dcsr &= ~(DCSR_RUN|DCSR_STOPIRQEN);
        DCSR(lp->dma) = dcsr;
        
        while (!(DCSR(lp->dma) & DCSR_STOPSTATE));
}

/* Reset any active DMA operation */
static void
smc_pxa_dma_reset(struct net_device * dev)
{
	struct smc_local *lp = (struct smc_local *)dev->priv;

	if (lp->dma_state == SMC_PXA_DMA_STATE_RX) {
		pci_unmap_single(NULL, lp->rx_dma_buf, lp->rx_dma_len, 
				 PCI_DMA_FROMDEVICE);
		SMC_TEST_SKB(lp->rx_skb);
		dev_kfree_skb(lp->rx_skb);
		lp->stats.rx_dropped++;
	} else if (lp->dma_state == SMC_PXA_DMA_STATE_TX) {
		pci_unmap_single(NULL, lp->tx_dma_buf, lp->tx_dma_len, 
				 PCI_DMA_TODEVICE);
		SMC_TEST_SKB(lp->tx_skb);
		dev_kfree_skb(lp->tx_skb);
		lp->stats.tx_dropped++;
	}

	lp->dma_state = SMC_PXA_DMA_STATE_IDLE;
	lp->saved_mask = 0;
}

/* RX packets. */
static inline void
smc_pxa_dma_insl(u_long ioaddr, int reg, struct net_device *dev, 
		 u_char *buf, int len)
{
	struct smc_local       *lp = (struct smc_local *)dev->priv;

        lp->dma_state = SMC_PXA_DMA_STATE_RX;

	/* fallback if no DMA available */
	if ((lp->dma == -1) || ((len << 2) < SMC_PXA_DMA_THRESHOLD)) {
		insl(ioaddr + reg, buf, len);
        	smc_pxa_dma_irq(lp->dma, dev, NULL);
		return;
	}

	/* 64 bit alignment is required for memory to memory DMA */
	if ((long)buf & 4) {
		*((u32 *)buf)++ = SMC_inl(ioaddr, reg);
		len--;
	}

	len *= 4;
	lp->rx_dma_buf = pci_map_single(NULL, buf, len, PCI_DMA_FROMDEVICE);
	lp->rx_dma_len = len;

	DCSR(lp->dma) = DCSR_NODESC;
	DTADR(lp->dma) = lp->rx_dma_buf;
	DSADR(lp->dma) = SMC_IOADDR + reg;
	DCMD(lp->dma) = (DCMD_INCTRGADDR | DCMD_BURST32 | DCMD_WIDTH4 | len);

#ifdef SMC_USE_PXA_DMA_IRQ
	/* Start DMA transfer */
	DCSR(lp->dma) = DCSR_NODESC | DCSR_RUN | DCSR_STOPIRQEN;
#else
	/* Start DMA transfer */
	DCSR(lp->dma) = DCSR_NODESC | DCSR_RUN;
        /* Wait until the DMA transfer finishes */
        smc_pxa_dma_wait_busy(dev);
	lp->dma_state = SMC_PXA_DMA_STATE_IDLE;
#endif
}

/* TX packets. */
static inline void
smc_pxa_dma_outsl(u_long ioaddr, int reg, struct net_device *dev, 
		 u_char *buf, int len)
{
	struct smc_local *lp = (struct smc_local *)dev->priv;

        lp->dma_state = SMC_PXA_DMA_STATE_TX;

	/* fallback if no DMA available */
	if ((lp->dma == -1) || ((len << 2) < SMC_PXA_DMA_THRESHOLD)) {
		outsl(ioaddr + reg, buf, len);
        	smc_pxa_dma_irq(lp->dma, dev, NULL);
		return;
	}

	/* 64 bit alignment is required for memory to memory DMA */
	if ((long)buf & 4) {
		SMC_outl(*((u32 *)buf)++, ioaddr, reg);
		len--;
	}

	len *= 4;
	lp->tx_dma_buf = pci_map_single(NULL, buf, len, PCI_DMA_TODEVICE);
	lp->tx_dma_len = len;

	DCSR(lp->dma) = DCSR_NODESC;
	DSADR(lp->dma) = lp->tx_dma_buf;
	DTADR(lp->dma) = SMC_IOADDR + reg;
	DCMD(lp->dma) = (DCMD_INCSRCADDR | DCMD_BURST32 | DCMD_WIDTH4 | len);

#ifdef SMC_USE_PXA_DMA_IRQ
	/* Start DMA transfer */
	DCSR(lp->dma) = DCSR_NODESC | DCSR_RUN | DCSR_STOPIRQEN;
#else
	/* Start DMA transfer */
	DCSR(lp->dma) = DCSR_NODESC | DCSR_RUN;
        /* Wait until the DMA transfer finishes */
        smc_pxa_dma_wait_busy(dev);

	/* This may run outside of IRQ context, so unmask any active 
	   SMC interrupts raised during DMA transfer */
	{
		long flags;
		local_irq_save(flags);
		lp->dma_state = SMC_PXA_DMA_STATE_IDLE;
		SMC_ENABLE_INT(lp->saved_mask);
		lp->saved_mask = 0;
		local_irq_restore(flags);
	}
#endif
}

#else /* SMC_USE_PXA_DMA */

#define smc_pxa_dma_wait_busy(dev) do {} while (0)
#define smc_pxa_dma_stop(dev) do {} while (0)
#define smc_pxa_dma_reset(dev) do {} while (0)

#endif /* SMC_USE_PXA_DMA */



/* this does a soft reset on the device */
static void
smc_reset(struct net_device *dev)
{
	unsigned long ioaddr = dev->base_addr;

	PRINTK2("%s: %s\n", dev->name, __FUNCTION__);

	/* This resets the registers mostly to defaults, but doesn't
	   affect EEPROM.  That seems unnecessary */
	SMC_SELECT_BANK( 0 );
	SMC_SET_RCR( RCR_SOFTRST );

	/* Setup the Configuration Register */
	/* This is necessary because the CONFIG_REG is not affected */
	/* by a soft reset */
	SMC_SELECT_BANK( 1 );
	SMC_SET_CONFIG( CONFIG_DEFAULT );

	/* Setup for fast accesses if requested */
	/* If the card/system can't handle it then there will */
	/* be no recovery except for a hard reset or power cycle */
	if (nowait)
		SMC_SET_CONFIG( SMC_GET_CONFIG() | CONFIG_NO_WAIT );

#ifdef POWER_DOWN
	/* Release from possible power-down state */
	/* Configuration register is not affected by Soft Reset */
	SMC_SELECT_BANK( 1 );
	SMC_SET_CONFIG( SMC_GET_CONFIG() | CONFIG_EPH_POWER_EN );
#endif

	/* this should pause enough for the chip to be happy */
	udelay(1);

	/* Disable transmit and receive functionality */
	SMC_SELECT_BANK( 0 );
	SMC_SET_RCR( RCR_CLEAR );
	SMC_SET_TCR( TCR_CLEAR );

	/* set the control register to automatically
	   release successfully transmitted packets, to make the best
	   use out of our limited memory */
	SMC_SELECT_BANK( 1 );
#if ! THROTTLE_TX_PKTS
	SMC_SET_CTL( SMC_GET_CTL() | CTL_AUTO_RELEASE );
#else
	SMC_SET_CTL( SMC_GET_CTL() & ~CTL_AUTO_RELEASE );
#endif

	/* Disable all interrupts */
	SMC_SELECT_BANK( 2 );
	SMC_SET_INT_MASK( 0 );

	/* Reset the MMU */
	SMC_SET_MMU_CMD( MC_RESET );
	SMC_WAIT_MMU_BUSY();
}

/* Enable Interrupts, Receive, and Transmit */
static void
smc_enable(struct net_device *dev)
{
	unsigned long ioaddr = dev->base_addr;
	struct smc_local *lp = (struct smc_local *)dev->priv;
	int mask;

	PRINTK2("%s: %s\n", dev->name, __FUNCTION__);

	/* see the header file for options in TCR/RCR DEFAULT*/
	SMC_SELECT_BANK( 0 );
	SMC_SET_TCR( lp->tcr_cur_mode );
	SMC_SET_RCR( lp->rcr_cur_mode );

	/* now, enable interrupts */
	mask = IM_EPH_INT | IM_RX_OVRN_INT | IM_RCV_INT;
	if (lp->version >= 0x70)
		mask |= IM_MDINT;

	SMC_SELECT_BANK( 2 );
	SMC_SET_INT_MASK( mask );
}

/* this puts the device in an inactive state */
static void
smc_shutdown(unsigned long ioaddr)
{
	PRINTK2("%s: %s\n", CARDNAME, __FUNCTION__);

	/* no more interrupts for me */
	SMC_SELECT_BANK( 2 );
	SMC_SET_INT_MASK( 0 );

	/* and tell the card to stay away from that nasty outside world */
	SMC_SELECT_BANK( 0 );
	SMC_SET_RCR( RCR_CLEAR );
	SMC_SET_TCR( TCR_CLEAR );

#ifdef POWER_DOWN
	/* finally, shut the chip down */
	SMC_SELECT_BANK( 1 );
	SMC_SET_CONFIG( SMC_GET_CONFIG() & ~CONFIG_EPH_POWER_EN );
#endif
}

/* 
 * Pass received packet up. It should be called after the packet is
 * transferred (with or without DMA).
 */
static inline void
smc_rcv_finish(struct net_device *dev, struct sk_buff *skb)
{
	struct smc_local *lp = (struct smc_local *)dev->priv;
	unsigned long ioaddr = dev->base_addr;

	SMC_WAIT_MMU_BUSY();
	SMC_SET_MMU_CMD( MC_RELEASE );

        PRINTK2("%s: smc_rcv_finish(): skb=0x%p, skb->data=0x%p, skb->len=%d\n",
                dev->name, skb, skb->data, skb->len);
	PRINT_PKT(skb->data, skb->len - 2);

	dev->last_rx = jiffies;
	skb->dev = dev;
	skb->protocol = eth_type_trans(skb, dev);

	lp->stats.rx_packets++;
	lp->stats.rx_bytes += skb->len;

	netif_rx(skb);
}

/*
 * Tell everyone that packet is sent.
 */
static inline void
smc_send_finish(struct net_device *dev, struct sk_buff *skb)
{
	struct smc_local *lp = (struct smc_local *)dev->priv;
	unsigned long ioaddr = dev->base_addr;

#ifdef SMC_USE_PXA_DMA
	/* push leftover halfword if necessary */
	if (skb->len >= 2 && ((skb->len ^ (long)skb->data) & 2))
		SMC_outw( *(u16 *)(&skb->data[(skb->len&~1)-2]),
			  ioaddr, DATA_REG );
#endif

	/* Send final ctl word with the last byte if there is one */
	SMC_outw(((skb->len & 1) ? (0x2000 | skb->data[skb->len-1]) : 0), 
		 ioaddr, DATA_REG );

	/* and let the chipset deal with it */
	SMC_SET_MMU_CMD( MC_ENQUEUE );
	SMC_ACK_INT( IM_TX_EMPTY_INT );

	lp->stats.tx_packets++;
	lp->stats.tx_bytes += skb->len;

	dev->trans_start = jiffies;
	dev_kfree_skb_any(skb);
}

/* This is the procedure to handle the receipt of a packet. */
static void 
smc_rcv(struct net_device *dev)
{
	struct smc_local *lp = (struct smc_local *)dev->priv;
	unsigned long ioaddr = dev->base_addr;
	unsigned int packet_number, status, packet_len;

	PRINTK3("%s: %s\n", dev->name, __FUNCTION__);

	packet_number = SMC_GET_RXFIFO();
	if (unlikely(packet_number & RXFIFO_REMPTY)) {
		PRINTK2("%s: smc_rcv with nothing on FIFO.\n", dev->name);
		return;
	}

	/* read from start of packet */
	SMC_SET_PTR( PTR_READ | PTR_RCV | PTR_AUTOINC );

	/* First two words are status and packet length */
	SMC_GET_PKT_HDR(status, packet_len);
	packet_len &= 0x07ff;  /* mask off top bits */
	PRINTK2("%s: RX PNR 0x%x STATUS 0x%04x LENGTH 0x%04x (%d)\n",
		dev->name, packet_number, status,
		packet_len, packet_len);

	if (unlikely(status & RS_ERRORS)) {
		lp->stats.rx_errors++;
		if (status & RS_ALGNERR)
			lp->stats.rx_frame_errors++;
		if (status & (RS_TOOSHORT | RS_TOOLONG))
			lp->stats.rx_length_errors++;
		if (status & RS_BADCRC)
			lp->stats.rx_crc_errors++;
		free_mem:
		SMC_WAIT_MMU_BUSY();
		SMC_SET_MMU_CMD( MC_RELEASE );
	} else {
		struct sk_buff *skb;
		unsigned char *data;
		unsigned int data_len;

		/* set multicast stats */
		if (status & RS_MULTICAST)
			lp->stats.multicast++;

		/*
		 * Actual payload is packet_len - 4 (or 3 if odd byte).
		 * We want skb_reserve(2) and the final ctrl word
		 * (2 bytes, possibly containing the payload odd byte).
		 * Ence packet_len - 4 + 2 + 2.
		 */
		skb = dev_alloc_skb(packet_len);
		if (unlikely(skb == NULL)) {
			printk(KERN_NOTICE "%s: Low memory, packet dropped.\n",
				dev->name);
			lp->stats.rx_dropped++;
			goto free_mem;
		}

		/* Align IP header to 32 bits */
		skb_reserve(skb, 2);

		/* BUG: the LAN91C111 rev A never sets this bit. Force it. */
		if (lp->version == 0x90)
			status |= RS_ODDFRAME;

		/*
		 * If odd length: packet_len - 3,
		 * otherwise packet_len - 4.
		 */
		data_len = packet_len - ((status & RS_ODDFRAME) ? 3 : 4);
		data = skb_put(skb, data_len);

#ifdef SMC_USE_PXA_DMA
		lp->rx_skb = skb;
#endif

		SMC_PULL_DATA(data, packet_len - 2);

#ifndef SMC_USE_PXA_DMA_IRQ
		smc_rcv_finish(dev, skb);
#endif
	}
}

/*
 * This is called to actually send a packet to the chip.
 * Returns non-zero when successful.
 */
static int
smc_hardware_send_packet(struct net_device *dev)
{
	struct smc_local *lp = (struct smc_local *)dev->priv;
	unsigned long ioaddr = dev->base_addr;
	struct sk_buff *skb = lp->saved_skb;
	unsigned int packet_no, len;
	unsigned char *buf;

	PRINTK3("%s: %s\n", dev->name, __FUNCTION__);

	lp->saved_skb = NULL;
	packet_no = SMC_GET_AR();
	if (unlikely(packet_no & AR_FAILED)) {
		printk("%s: Memory allocation failed.\n", dev->name);
		lp->stats.tx_errors++;
		lp->stats.tx_fifo_errors++;
		dev_kfree_skb_any(skb);
		return -ENOMEM;
	}

	/* point to the beginning of the packet */
	SMC_SET_PN( packet_no );
	SMC_SET_PTR( PTR_AUTOINC );

	buf = skb->data;
	len = skb->len;
	PRINTK2("%s: TX PNR 0x%x LENGTH 0x%04x (%d) BUF 0x%p\n",
		dev->name, packet_no, len, len, buf);
	PRINT_PKT(buf, len);

	/*
	 * Send the packet length ( +6 for status words, length, and ctl.
	 * The card will pad to 64 bytes with zeroes if packet is too small.
	 */
	SMC_PUT_PKT_HDR(0, len + 6);

#ifdef SMC_USE_PXA_DMA
	lp->tx_skb = skb;
	/* Hack to prevent SMC_PUSH_DATA() from pushing the last halfword.
	   We will push it manually in smc_send_finish() after DMA is done. */
	if (len >= 2 && ((len ^ (long)buf) & 2)) len -= 2;
#endif

	/* send the actual data */
	SMC_PUSH_DATA(buf, len & ~1);

#ifndef SMC_USE_PXA_DMA_IRQ
	smc_send_finish(dev, skb);
	return 1;
#else
	return 0;
#endif
}

static int
smc_tx_alloc_buf( struct sk_buff * skb, struct net_device * dev)
{
	struct smc_local *lp = (struct smc_local *)dev->priv;
	unsigned long ioaddr = dev->base_addr;
	unsigned int numPages;
	unsigned int poll_count;

	/*
	** The MMU wants the number of pages to be the number of 256 bytes
	** 'pages', minus 1 ( since a packet can't ever have 0 pages :) )
	**
	** The 91C111 ignores the size bits, but the code is left intact
	** for backwards and future compatibility.
	**
	** Pkt size for allocating is data length +6 (for additional status
	** words, length and ctl!)
	**
	** If odd size then last byte is included in ctl word.
	*/
	numPages = ((skb->len & ~1) + (6 - 1)) >> 8;
	if (unlikely(numPages > 7)) {
		printk("%s: Far too big packet error.\n", dev->name);
		lp->stats.tx_errors++;
		lp->stats.tx_dropped++;
		lp->saved_skb = NULL;
		dev_kfree_skb_any(skb);
		return -EINVAL;
	}

	/* now, try to allocate the memory */
	SMC_SET_MMU_CMD( MC_ALLOC | numPages );
        
	/*
	 * Poll the chip for a short amount of time in case the
	 * allocation succeeds quickly.
	 */
	for (poll_count = MEMORY_WAIT_TIME; poll_count > 0; poll_count--) {
		if (SMC_GET_INT() & IM_ALLOC_INT) {
			SMC_ACK_INT( IM_ALLOC_INT );
  			break;
		}
        };

   	if (poll_count) {
		/* Send current packet immediately.. */
		int ret = smc_hardware_send_packet(dev);
		if (ret >= 0 && THROTTLE_TX_PKTS)
			netif_stop_queue(dev);
		return ret;
   	} else {
		/* oh well, wait until the chip finds memory later */
		PRINTK2("%s: TX memory allocation deferred.\n", dev->name);
		netif_stop_queue(dev);
		return -EBUSY;
	}
}

/*
 . Since I am not sure if I will have enough room in the chip's ram
 . to store the packet, I call this routine which either sends it
 . now, or set the card to generates an interrupt when ready
 . for the packet.
 */
static int
smc_hard_start_xmit( struct sk_buff * skb, struct net_device * dev )
{
	struct smc_local *lp = (struct smc_local *)dev->priv;
	unsigned long ioaddr = dev->base_addr;

	PRINTK3("%s: %s\n", dev->name, __FUNCTION__);

        BUG_ON(lp->saved_skb != NULL);
	lp->saved_skb = skb;

#ifdef SMC_USE_PXA_DMA_IRQ
	{
		unsigned long flags;
		local_irq_save(flags);
		if (lp->dma_state != SMC_PXA_DMA_STATE_IDLE) {
			netif_stop_queue(dev);
			lp->wait_dma_skb = skb;
			local_irq_restore(flags);
			return 0;
		}
		/* prevent RX interrupt from messing with our TX DMA */
		SMC_SET_INT_MASK( SMC_GET_INT_MASK() & ~IM_RCV_INT );
		local_irq_restore(flags);
	}
#endif

	switch (smc_tx_alloc_buf(skb, dev)) {
	case 1:
		SMC_ENABLE_INT(IM_TX_INT | IM_TX_EMPTY_INT);
		break;
        case -EBUSY:
		SMC_ENABLE_INT(IM_ALLOC_INT);
		break;
	}

#ifdef SMC_USE_PXA_DMA_IRQ
	/* it's safe to turn this back on again at this point */
	SMC_ENABLE_INT(IM_RCV_INT);
#endif

	return 0;
}

/*
 . This handles a TX interrupt, which is only called when an error
 . relating to a packet is sent or CTL_AUTO_RELEASE is not set.
*/
static void
smc_tx(struct net_device *dev)
{
	unsigned long ioaddr = dev->base_addr;
	struct smc_local *lp = (struct smc_local *)dev->priv;
	unsigned int saved_packet, packet_no, tx_status, pkt_len;

	PRINTK3("%s: %s\n", dev->name, __FUNCTION__);

	/* If the TX FIFO is empty then nothing to do */
	packet_no = SMC_GET_TXFIFO();
	if (unlikely(packet_no & TXFIFO_TEMPTY)) {
		PRINTK2("%s: smc_tx with nothing on FIFO.\n", dev->name);
		return;
	}

	/* select packet to read from */
	saved_packet = SMC_GET_PN();
	SMC_SET_PN( packet_no );

	/* read the first word (status word) from this packet */
	SMC_SET_PTR( PTR_AUTOINC | PTR_READ );
	SMC_GET_PKT_HDR(tx_status, pkt_len);
	PRINTK2("%s: TX STATUS 0x%04x PNR 0x%02x\n",
		dev->name, tx_status, packet_no);

	if (!(tx_status & TS_SUCCESS))
		lp->stats.tx_errors++;
	if (tx_status & TS_LOSTCAR)
		lp->stats.tx_carrier_errors++;
	if (tx_status & TS_LATCOL) {
		printk( KERN_DEBUG
			"%s: Late collision occurred on last xmit.\n",
			dev->name);
		lp->stats.tx_window_errors++;
	}

	/* kill the packet */
	SMC_WAIT_MMU_BUSY();
	SMC_SET_MMU_CMD( MC_FREEPKT );

	/* Don't restore Packet Number Reg until busy bit is cleared */
	SMC_WAIT_MMU_BUSY();
	SMC_SET_PN( saved_packet );

	/* re-enable transmit */
	SMC_SELECT_BANK( 0 );
	SMC_SET_TCR( lp->tcr_cur_mode );
	SMC_SELECT_BANK( 2 );
}


//---PHY CONTROL AND CONFIGURATION-----------------------------------------

/*------------------------------------------------------------
 . Debugging function for viewing MII Management serial bitstream
 .-------------------------------------------------------------*/
#if SMC_DEBUG > 3
static void
PRINT_MII_STREAM(u_char *bits, int size)
{
	int i;

	printk("BIT#:");
	for (i = 0; i < size; ++i)
		printk("%d", i%10);

	printk("\nMDOE:");
	for (i = 0; i < size; ++i) {
		if (bits[i] & MII_MDOE)
			printk("1");
		else
			printk("0");
	}

	printk("\nMDO :");
	for (i = 0; i < size; ++i) {
		if (bits[i] & MII_MDO)
			printk("1");
		else
			printk("0");
	}

	printk("\nMDI :");
	for (i = 0; i < size; ++i) {
		if (bits[i] & MII_MDI)
			printk("1");
		else
			printk("0");
	}

	printk("\n");
}
#else
#define PRINT_MII_STREAM(x...)
#endif

/*------------------------------------------------------------
 . Reads a register from the MII Management serial interface
 .-------------------------------------------------------------*/
static int
smc_read_phy_register(unsigned long ioaddr, int phyaddr, int phyreg)
{
	int i, mask, mii_reg;
	u_char bits[64];
	int input_idx, phydata;
	int clk_idx = 0;

	// 32 consecutive ones on MDO to establish sync
	for (i = 0; i < 32; ++i)
		bits[clk_idx++] = MII_MDOE | MII_MDO;

	// Start code <01>
	bits[clk_idx++] = MII_MDOE;
	bits[clk_idx++] = MII_MDOE | MII_MDO;

	// Read command <10>
	bits[clk_idx++] = MII_MDOE | MII_MDO;
	bits[clk_idx++] = MII_MDOE;

	// Output the PHY address, msb first
	mask = 0x10;
	for (i = 0; i < 5; ++i) {
		if (phyaddr & mask)
			bits[clk_idx++] = MII_MDOE | MII_MDO;
		else
			bits[clk_idx++] = MII_MDOE;

		// Shift to next lowest bit
		mask >>= 1;
	}

	// Output the phy register number, msb first
	mask = 0x10;
	for (i = 0; i < 5; ++i) {
		if (phyreg & mask)
			bits[clk_idx++] = MII_MDOE | MII_MDO;
		else
			bits[clk_idx++] = MII_MDOE;

		// Shift to next lowest bit
		mask >>= 1;
	}

	// Tristate and turnaround (2 bit times)
	bits[clk_idx++] = 0;
	//bits[clk_idx++] = 0;

	// Input starts at this bit time
	input_idx = clk_idx;

	// Will input 16 bits
	for (i = 0; i < 16; ++i)
		bits[clk_idx++] = 0;

	// Final clock bit
	bits[clk_idx++] = 0;

	// Select bank 3
	SMC_SELECT_BANK( 3 );

	// Get the current MII register value
	mii_reg = SMC_GET_MII();

	// Turn off all MII Interface bits
	mii_reg &= ~(MII_MDOE|MII_MCLK|MII_MDI|MII_MDO);

	// Clock all 64 cycles
	for (i = 0; i < sizeof bits; ++i) {
		// Clock Low - output data
		SMC_SET_MII( mii_reg | bits[i] );
		udelay(50);

		// Clock Hi - input data
		SMC_SET_MII( mii_reg | bits[i] | MII_MCLK );
		udelay(50);
		bits[i] |= SMC_GET_MII() & MII_MDI;
	}

	// Return to idle state
	// Set clock to low, data to low, and output tristated
	SMC_SET_MII( mii_reg );
	udelay(50);

	// Restore original bank select
	SMC_SELECT_BANK( 2 );

	// Recover input data
	phydata = 0;
	for (i = 0; i < 16; ++i) {
		phydata <<= 1;

		if (bits[input_idx++] & MII_MDI)
			phydata |= 0x0001;
	}

	PRINTK3("%s: phyaddr=0x%x, phyreg=0x%x, phydata=0x%x\n",
		__FUNCTION__, phyaddr, phyreg, phydata);
	PRINT_MII_STREAM(bits, sizeof(bits));

	return phydata;
}

/*------------------------------------------------------------
 . Writes a register to the MII Management serial interface
 .-------------------------------------------------------------*/
static void
smc_write_phy_register( unsigned long ioaddr, int phyaddr,
			int phyreg, int phydata )
{
	int i, mask, mii_reg;
	u_char bits[65];
	int clk_idx = 0;

	// 32 consecutive ones on MDO to establish sync
	for (i = 0; i < 32; ++i)
		bits[clk_idx++] = MII_MDOE | MII_MDO;

	// Start code <01>
	bits[clk_idx++] = MII_MDOE;
	bits[clk_idx++] = MII_MDOE | MII_MDO;

	// Write command <01>
	bits[clk_idx++] = MII_MDOE;
	bits[clk_idx++] = MII_MDOE | MII_MDO;

	// Output the PHY address, msb first
	mask = 0x10;
	for (i = 0; i < 5; ++i) {
		if (phyaddr & mask)
			bits[clk_idx++] = MII_MDOE | MII_MDO;
		else
			bits[clk_idx++] = MII_MDOE;

		// Shift to next lowest bit
		mask >>= 1;
	}

	// Output the phy register number, msb first
	mask = 0x10;
	for (i = 0; i < 5; ++i) {
		if (phyreg & mask)
			bits[clk_idx++] = MII_MDOE | MII_MDO;
		else
			bits[clk_idx++] = MII_MDOE;

		// Shift to next lowest bit
		mask >>= 1;
	}

	// Tristate and turnaround (2 bit times)
	bits[clk_idx++] = 0;
	bits[clk_idx++] = 0;

	// Write out 16 bits of data, msb first
	mask = 0x8000;
	for (i = 0; i < 16; ++i) {
		if (phydata & mask)
			bits[clk_idx++] = MII_MDOE | MII_MDO;
		else
			bits[clk_idx++] = MII_MDOE;

		// Shift to next lowest bit
		mask >>= 1;
	}

	// Final clock bit (tristate)
	bits[clk_idx++] = 0;

	// Select bank 3
	SMC_SELECT_BANK( 3 );

	// Get the current MII register value
	mii_reg = SMC_GET_MII();

	// Turn off all MII Interface bits
	mii_reg &= ~(MII_MDOE|MII_MCLK|MII_MDI|MII_MDO);

	// Clock all cycles
	for (i = 0; i < sizeof bits; ++i) {
		// Clock Low - output data
		SMC_SET_MII( mii_reg | bits[i] );
		udelay(50);

		// Clock Hi - input data
		SMC_SET_MII( mii_reg | bits[i] | MII_MCLK );
		udelay(50);
		bits[i] |= SMC_GET_MII() & MII_MDI;
	}

	// Return to idle state
	// Set clock to low, data to low, and output tristated
	SMC_SET_MII( mii_reg );
	udelay(50);

	// Restore original bank select
	SMC_SELECT_BANK( 2 );

	PRINTK3("%s: phyaddr=0x%x, phyreg=0x%x, phydata=0x%x\n",
		__FUNCTION__, phyaddr, phyreg, phydata);
	PRINT_MII_STREAM(bits, sizeof(bits));
}


/*------------------------------------------------------------
 . Finds and reports the PHY address
 .-------------------------------------------------------------*/
static int smc_detect_phy(struct net_device* dev)
{
	struct smc_local *lp = (struct smc_local *)dev->priv;
	unsigned long ioaddr = dev->base_addr;
	int phy_id1, phy_id2;
	int phyaddr;
	int found = 0;

	PRINTK2("%s: %s\n", dev->name, __FUNCTION__);

	// Scan all 32 PHY addresses if necessary
	for (phyaddr = 0; phyaddr < 32; ++phyaddr) {
		// Read the PHY identifiers
		phy_id1 = smc_read_phy_register(ioaddr, phyaddr, PHY_ID1_REG);
		phy_id2 = smc_read_phy_register(ioaddr, phyaddr, PHY_ID2_REG);

		PRINTK3("%s: phy_id1=0x%x, phy_id2=0x%x\n",
			dev->name, phy_id1, phy_id2);

		// Make sure it is a valid identifier
		if ((phy_id2 > 0x0000) && (phy_id2 < 0xffff) &&
		    (phy_id1 > 0x0000) && (phy_id1 < 0xffff)) {
			if ((phy_id1 != 0x8000) && (phy_id2 != 0x8000)) {
				// Save the PHY's address
				lp->phyaddr = phyaddr;
				found = 1;
				break;
			}
		}
	}

	if (!found) {
		PRINTK("%s: No PHY found\n", dev->name);
		return(0);
	}

	// Set the PHY type
	if ( (phy_id1 == 0x0016) && ((phy_id2 & 0xFFF0) == 0xF840 ) ) {
		lp->phytype = PHY_LAN83C183;
		PRINTK("%s: PHY=LAN83C183 (LAN91C111 Internal)\n", dev->name);
	}

	if ( (phy_id1 == 0x0282) && ((phy_id2 & 0xFFF0) == 0x1C50) ) {
		lp->phytype = PHY_LAN83C180;
		PRINTK("%s: PHY=LAN83C180\n", dev->name);
	}

	return 1;
}

/*------------------------------------------------------------
 . Waits the specified number of milliseconds - kernel friendly
 .-------------------------------------------------------------*/
static void
smc_wait_ms(unsigned int ms)
{
	if (!in_interrupt()) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(1 + ms * HZ / 1000);
	} else {
		/* if this happens it must be fixed */
		printk( KERN_WARNING "%s: busy wait while in interrupt!\n",
			__FUNCTION__);
		mdelay(ms);
	}
}

/*------------------------------------------------------------
 . Sets the PHY to a configuration as determined by the user
 .-------------------------------------------------------------*/
static int
smc_phy_fixed(struct net_device *dev)
{
	unsigned long ioaddr = dev->base_addr;
	struct smc_local *lp = (struct smc_local *)dev->priv;
	int phyaddr = lp->phyaddr;
	int my_fixed_caps, cfg1;

	PRINTK3("%s: %s\n", dev->name, __FUNCTION__);

	// Enter Link Disable state
	cfg1 = smc_read_phy_register(ioaddr, phyaddr, PHY_CFG1_REG);
	cfg1 |= PHY_CFG1_LNKDIS;
	smc_write_phy_register(ioaddr, phyaddr, PHY_CFG1_REG, cfg1);

	// Set our fixed capabilities
	// Disable auto-negotiation
	my_fixed_caps = 0;

	if (lp->ctl_rfduplx)
		my_fixed_caps |= PHY_CNTL_DPLX;

	if (lp->ctl_rspeed == 100)
		my_fixed_caps |= PHY_CNTL_SPEED;

	// Write our capabilities to the phy control register
	smc_write_phy_register(ioaddr, phyaddr, PHY_CNTL_REG, my_fixed_caps);

	// Re-Configure the Receive/Phy Control register
	SMC_SET_RPC( lp->rpc_cur_mode );

	// Success
	return(1);
}

/*------------------------------------------------------------
 . Configures the specified PHY through the MII management interface
 . using Autonegotiation.
 . Calls smc_phy_fixed() if the user has requested a certain config.
 .-------------------------------------------------------------*/
static void
smc_phy_configure(struct net_device* dev)
{
	unsigned long ioaddr = dev->base_addr;
	struct smc_local *lp = (struct smc_local *)dev->priv;
	int timeout;
	int phyaddr;
	int my_phy_caps; // My PHY capabilities
	int my_ad_caps; // My Advertised capabilities
	int status;

	PRINTK3("%s:smc_program_phy()\n", dev->name);

	// Set the blocking flag
	lp->autoneg_active = 1;

	// Find the address and type of our phy
	if (!smc_detect_phy(dev))
		goto smc_phy_configure_exit;

	// Get the detected phy address
	phyaddr = lp->phyaddr;

	// Reset the PHY, setting all other bits to zero
	smc_write_phy_register(ioaddr, phyaddr, PHY_CNTL_REG, PHY_CNTL_RST);

	// Wait for the reset to complete, or time out
	timeout = 6; // Wait up to 3 seconds
	while (timeout--) {
		if (!(smc_read_phy_register(ioaddr, phyaddr, PHY_CNTL_REG)
		    & PHY_CNTL_RST))
			// reset complete
			break;
		smc_wait_ms(500); // wait 500 millisecs
		if (signal_pending(current)) { // Exit anyway if signaled
			PRINTK("%s: PHY reset interrupted by signal\n",
				dev->name);
			timeout = 0;
			break;
		}
	}

	if (timeout < 1) {
		printk("%s: PHY reset timed out\n", dev->name);
		goto smc_phy_configure_exit;
	}

	// Read PHY Register 18, Status Output
	lp->lastPhy18 = smc_read_phy_register(ioaddr, phyaddr, PHY_INT_REG);

	// Enable PHY Interrupts (for register 18)
	// Interrupts listed here are disabled
	smc_write_phy_register(ioaddr, phyaddr, PHY_MASK_REG,
		PHY_INT_LOSSSYNC | PHY_INT_CWRD | PHY_INT_SSD |
		PHY_INT_ESD | PHY_INT_RPOL | PHY_INT_JAB |
		PHY_INT_SPDDET | PHY_INT_DPLXDET);

	/* Configure the Receive/Phy Control register */
	SMC_SELECT_BANK( 0 );
	SMC_SET_RPC( lp->rpc_cur_mode );
	SMC_SELECT_BANK( 2 );

	// Copy our capabilities from PHY_STAT_REG to PHY_AD_REG
	my_phy_caps = smc_read_phy_register(ioaddr, phyaddr, PHY_STAT_REG);

	// If the user requested no auto neg, then go set his request
	if (!(lp->ctl_autoneg)) {
		smc_phy_fixed(dev);
		goto smc_phy_configure_exit;
	}

	if( !( my_phy_caps & PHY_STAT_CAP_ANEG))
	{
		printk(KERN_INFO "Auto negotiation NOT supported\n");
		smc_phy_fixed(dev);
		goto smc_phy_configure_exit;
	}

	my_ad_caps  = PHY_AD_CSMA; // I am CSMA capable

	if (my_phy_caps & PHY_STAT_CAP_T4)
		my_ad_caps |= PHY_AD_T4;

	if (my_phy_caps & PHY_STAT_CAP_TXF)
		my_ad_caps |= PHY_AD_TX_FDX;

	if (my_phy_caps & PHY_STAT_CAP_TXH)
		my_ad_caps |= PHY_AD_TX_HDX;

	if (my_phy_caps & PHY_STAT_CAP_TF)
		my_ad_caps |= PHY_AD_10_FDX;

	if (my_phy_caps & PHY_STAT_CAP_TH)
		my_ad_caps |= PHY_AD_10_HDX;

	// Disable capabilities not selected by our user
	if (lp->ctl_rspeed != 100)
		my_ad_caps &= ~(PHY_AD_T4|PHY_AD_TX_FDX|PHY_AD_TX_HDX);

	if (!lp->ctl_rfduplx)
		my_ad_caps &= ~(PHY_AD_TX_FDX|PHY_AD_10_FDX);

	// Update our Auto-Neg Advertisement Register
	smc_write_phy_register(ioaddr, phyaddr, PHY_AD_REG, my_ad_caps);

	// Read the register back.  Without this, it appears that when
	// auto-negotiation is restarted, sometimes it isn't ready and
	// the link does not come up.
	status = smc_read_phy_register(ioaddr, phyaddr, PHY_AD_REG);

	PRINTK2("%s: phy caps=%x\n", dev->name, my_phy_caps);
	PRINTK2("%s: phy advertised caps=%x\n", dev->name, my_ad_caps);

	// Restart auto-negotiation process in order to advertise my caps
	smc_write_phy_register( ioaddr, phyaddr, PHY_CNTL_REG,
		PHY_CNTL_ANEG_EN | PHY_CNTL_ANEG_RST );

	// Wait for the auto-negotiation to complete.  This may take from
	// 2 to 3 seconds.
	// Wait for the reset to complete, or time out
	timeout = 20; // Wait up to 10 seconds
	while (timeout--) {
		status = smc_read_phy_register(ioaddr, phyaddr, PHY_RMT_REG);
		if (status & PHY_AD_ACK)
			// auto-negotiate complete
			break;

		smc_wait_ms(500); // wait 500 millisecs
		if (signal_pending(current)) { // Exit anyway if signaled
			printk(KERN_DEBUG
				"%s: PHY auto-negotiate interrupted by signal\n",
				dev->name);
			timeout = 0;
			break;
		}
	}
	status = smc_read_phy_register(ioaddr, phyaddr, PHY_STAT_REG);

	if (timeout < 1) {
		PRINTK("%s: PHY auto-negotiate timed out\n", dev->name);
	}

	// Fail if we detected an auto-negotiate remote fault
	if (status & PHY_STAT_REM_FLT) {
		PRINTK("%s: PHY remote fault detected\n", dev->name);
	}

	// Wait for link. Once the link is up, phy18 should be up to date
        timeout = 200;
        do {
                udelay(100);
                status = smc_read_phy_register(ioaddr, phyaddr, PHY_STAT_REG);
        } while ( ((status & PHY_STAT_LINK)==0) && --timeout);

        if (status & PHY_STAT_LINK) {                                      
                PRINTK("%s: Ethernet Link Detected\n", dev->name); 
        }       

	// The smc_phy_interrupt() routine will be called to update lastPhy18

	// Set our sysctl parameters to match auto-negotiation results
	if ( lp->lastPhy18 & PHY_INT_SPDDET ) {
		PRINTK("%s: PHY 100BaseT\n", dev->name);
		lp->rpc_cur_mode |= RPC_SPEED;
	} else {
		PRINTK("%s: PHY 10BaseT\n", dev->name);
		lp->rpc_cur_mode &= ~RPC_SPEED;
	}

	if ( lp->lastPhy18 & PHY_INT_DPLXDET ) {
		PRINTK("%s: PHY Full Duplex\n", dev->name);
		lp->rpc_cur_mode |= RPC_DPLX;
		lp->tcr_cur_mode |= TCR_SWFDUP;
	} else {
		PRINTK("%s: PHY Half Duplex\n", dev->name);
		lp->rpc_cur_mode &= ~RPC_DPLX;
		lp->tcr_cur_mode &= ~TCR_SWFDUP;
	}

        SMC_SELECT_BANK( 0 );

	// Re-Configure the Receive/Phy Control register and TCR
	SMC_SET_RPC( lp->rpc_cur_mode );
	SMC_SET_TCR( lp->tcr_cur_mode );

	SMC_SELECT_BANK( 2 );
	
smc_phy_configure_exit:
	// Exit auto-negotiation
	lp->autoneg_active = 0;
}

/*************************************************************************
 . smc_phy_interrupt
 .
 . Purpose:  Handle interrupts relating to PHY register 18. This is
 .  called from the "hard" interrupt handler.
 .
 ************************************************************************/
static void
smc_phy_interrupt(struct net_device* dev)
{
	unsigned long ioaddr = dev->base_addr;
	struct smc_local *lp = (struct smc_local *)dev->priv;
	int phyaddr = lp->phyaddr;
	int phy18;

	PRINTK2("%s: %s\n", dev->name, __FUNCTION__);

	for(;;) {
		// Read PHY Register 18, Status Output
		phy18 = smc_read_phy_register(ioaddr, phyaddr, PHY_INT_REG);

		// Exit if not more changes
		if (phy18 == lp->lastPhy18)
			break;

#if SMC_DEBUG > 1
		PRINTK2("%s:     phy18=0x%04x\n", dev->name, phy18);
		PRINTK2("%s: lastPhy18=0x%04x\n", dev->name, lp->lastPhy18);

		// Handle events
		if ((phy18 & PHY_INT_LNKFAIL) !=
				(lp->lastPhy18 & PHY_INT_LNKFAIL))
			PRINTK2("%s: PHY Link Fail=%x\n", dev->name,
					phy18 & PHY_INT_LNKFAIL);

		if ((phy18 & PHY_INT_LOSSSYNC) !=
				(lp->lastPhy18 & PHY_INT_LOSSSYNC))
			PRINTK2("%s: PHY LOSS SYNC=%x\n", dev->name,
					phy18 & PHY_INT_LOSSSYNC);

		if ((phy18 & PHY_INT_CWRD) != (lp->lastPhy18 & PHY_INT_CWRD))
			PRINTK2("%s: PHY INVALID 4B5B code=%x\n", dev->name,
					phy18 & PHY_INT_CWRD);

		if ((phy18 & PHY_INT_SSD) != (lp->lastPhy18 & PHY_INT_SSD))
			PRINTK2("%s: PHY No Start Of Stream=%x\n", dev->name,
					phy18 & PHY_INT_SSD);

		if ((phy18 & PHY_INT_ESD) != (lp->lastPhy18 & PHY_INT_ESD))

			PRINTK2("%s: PHY No End Of Stream=%x\n", dev->name,
					phy18 & PHY_INT_ESD);

		if ((phy18 & PHY_INT_RPOL) != (lp->lastPhy18 & PHY_INT_RPOL))
			PRINTK2("%s: PHY Reverse Polarity Detected=%x\n",
					dev->name, phy18 & PHY_INT_RPOL);

		if ((phy18 & PHY_INT_JAB) != (lp->lastPhy18 & PHY_INT_JAB))
			PRINTK2("%s: PHY Jabber Detected=%x\n", dev->name,
					phy18 & PHY_INT_JAB);

		if ((phy18 & PHY_INT_SPDDET) !=
				(lp->lastPhy18 & PHY_INT_SPDDET))
			PRINTK2("%s: PHY Speed Detect=%x\n", dev->name,
					phy18 & PHY_INT_SPDDET);

		if ((phy18 & PHY_INT_DPLXDET) !=
				(lp->lastPhy18 & PHY_INT_DPLXDET))
			PRINTK2("%s: PHY Duplex Detect=%x\n", dev->name,
					phy18 & PHY_INT_DPLXDET);
#endif
		// Update the last phy 18 variable
		lp->lastPhy18 = phy18;
	}
}

//--- END PHY CONTROL AND CONFIGURATION-------------------------------------


/*
 * This is the main routine of the driver, to handle the device when
 * it needs some attention.
 */
static void
smc_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct net_device *dev = dev_id;
	unsigned long ioaddr = dev->base_addr;
	struct smc_local *lp = (struct smc_local *)dev->priv;
	int status, mask, timeout, card_stats;
	int saved_bank, saved_pointer;

	PRINTK3("%s: %s\n", dev->name, __FUNCTION__);

#ifdef SMC_USE_PXA_DMA
	{
		long flags;
		local_irq_save(flags);
		if (lp->dma_state != SMC_PXA_DMA_STATE_IDLE) {
			if (SMC_CURRENT_BANK() != 0x3302) {
				PRINTK( "DMA is active while the register's "
					"bank is wrong\n");
				BUG();
			}

			/* Let's disable all SMC IRQs while DMA is active */
			lp->saved_mask |= SMC_GET_INT_MASK();
			SMC_SET_INT_MASK( 0 );
			local_irq_restore(flags);
			return;
		}
		local_irq_restore(flags);
	}
#endif

	saved_bank = SMC_CURRENT_BANK();
	SMC_SELECT_BANK(2);
	saved_pointer = SMC_GET_PTR();
	mask = SMC_GET_INT_MASK();
	SMC_SET_INT_MASK( 0 );

#ifdef SMC_USE_PXA_DMA_IRQ
	/* this is for when smc_pxa_dma_irq() calls us directly */
	if (lp->saved_mask) {
		mask |= lp->saved_mask;
		lp->saved_mask = 0;
	}
#endif

	/* set a timeout value, so I don't stay here forever */
	timeout = 8;

	do {
		status = SMC_GET_INT();

		PRINTK2("%s: IRQ 0x%02x MASK 0x%02x MEM 0x%04x FIFO 0x%04x\n",
			dev->name, status, mask,
			({ int meminfo; SMC_SELECT_BANK(0);
			   meminfo = SMC_GET_MIR();
			   SMC_SELECT_BANK(2); meminfo; }),
			SMC_GET_FIFO());

		status &= mask;
		if (!status)
			break;

		if (status & IM_RCV_INT) {
			PRINTK3("%s: RX irq\n", dev->name);
			smc_rcv(dev);
		} else if (status & IM_TX_INT) {
			PRINTK3("%s: TX int\n", dev->name);
			smc_tx(dev);
			SMC_ACK_INT( IM_TX_INT );
#if THROTTLE_TX_PKTS
			netif_wake_queue(dev);
#endif
		} else if (status & IM_ALLOC_INT) {
			PRINTK3("%s: Allocation irq\n", dev->name);
			if (smc_hardware_send_packet(dev) >= 0)
				mask |= (IM_TX_INT | IM_TX_EMPTY_INT);
			mask &= ~IM_ALLOC_INT;
#if ! THROTTLE_TX_PKTS
			netif_wake_queue(dev);
#endif
		} else if (status & IM_TX_EMPTY_INT) {
			PRINTK3("%s: TX empty\n", dev->name);
			mask &= ~IM_TX_EMPTY_INT;

			/* update stats */
			SMC_SELECT_BANK( 0 );
			card_stats = SMC_GET_COUNTER();
			SMC_SELECT_BANK( 2 );

			/* single collisions */
			lp->stats.collisions += card_stats & 0xF;
			card_stats >>= 4;

			/* multiple collisions */
			lp->stats.collisions += card_stats & 0xF;
		} else if (status & IM_RX_OVRN_INT) {
			PRINTK1( "%s: RX overrun\n", dev->name);
			SMC_ACK_INT( IM_RX_OVRN_INT );
			lp->stats.rx_errors++;
			lp->stats.rx_fifo_errors++;
		} else if (status & IM_EPH_INT) {
			PRINTK("%s: UNSUPPORTED: EPH INTERRUPT\n", dev->name);
		} else if (status & IM_MDINT) {
			SMC_ACK_INT( IM_MDINT );
			smc_phy_interrupt(dev);
		} else if (status & IM_ERCV_INT ) {
			SMC_ACK_INT( IM_ERCV_INT );
			PRINTK("%s: UNSUPPORTED: ERCV INTERRUPT \n", dev->name);
		}

#ifdef SMC_USE_PXA_DMA_IRQ
		{
			long flags;
			local_irq_save(flags);
			if (lp->dma_state != SMC_PXA_DMA_STATE_IDLE) {
				/* if DMA got started we must get out of here */
				lp->saved_mask |= mask;
				local_irq_restore(flags);
				return;
			} else {
				/* incase DMA got started and already done */
				mask |= SMC_GET_INT_MASK();
				SMC_SET_INT_MASK( 0 );
			}
			local_irq_restore(flags);
		}
#endif
	} while (--timeout);

	/* restore register states */
	SMC_SET_INT_MASK( mask );
	SMC_SET_PTR( saved_pointer );
	SMC_SELECT_BANK( saved_bank );

	PRINTK3("%s: Interrupt done\n", dev->name);
}

/* Our watchdog timed out. Called by the networking layer */
static void
smc_timeout(struct net_device *dev)
{
	struct smc_local *lp 	= (struct smc_local *)dev->priv;

	PRINTK2("%s: %s\n", dev->name, __FUNCTION__);

        smc_pxa_dma_stop(dev);
	smc_reset(dev);
        smc_pxa_dma_reset(dev);
	smc_enable(dev);

#if 0
	/* Reconfiguring the PHY doesn't seem like a bad idea here, but
	 * it introduced a problem.  Now that this is a timeout routine,
	 * we are getting called from within an interrupt context.
	 * smc_phy_configure() calls smc_wait_ms() which calls
	 * schedule_timeout() which calls schedule().  When schedule()
	 * is called from an interrupt context, it prints out
	 * "Scheduling in interrupt" and then calls BUG().  This is
	 * obviously not desirable.  This was worked around by removing
	 * the call to smc_phy_configure() here because it didn't seem
	 * absolutely necessary.  Ultimately, if smc_wait_ms() is
	 * supposed to be usable from an interrupt context (which it
	 * looks like it thinks it should handle), it should be fixed.
	 */
	/* Reconfigure the PHY */
	smc_phy_configure(dev);
#endif

	/* clear anything saved */
	if (lp->saved_skb != NULL) {
                printk("%s:%d: %s(): call dev_kfree_skb()\n",
                       __FILE__, __LINE__, __FUNCTION__);
		dev_kfree_skb (lp->saved_skb);
		lp->saved_skb = NULL;
		lp->stats.tx_errors++;
		lp->stats.tx_aborted_errors++;
	}
	dev->trans_start = jiffies;

        PRINTK2("%s:%d: %s: netif_wake_queue()\n",
               __FILE__, __LINE__, __FUNCTION__);
	netif_wake_queue(dev);
}

/*
 * Finds the CRC32 of a set of bytes.
 * (from Peter Cammaert's code)
 */
static int
crc32(char *s, int length)
{
	/* indices */
	int perByte;
	int perBit;
	/* crc polynomial for Ethernet */
	const unsigned long poly = 0xedb88320;
	/* crc value - preinitialized to all 1's */
	unsigned long crc_value = 0xffffffff;

	for ( perByte = 0; perByte < length; perByte ++ ) {
		unsigned char	c;

		c = *(s++);
		for ( perBit = 0; perBit < 8; perBit++ ) {
			crc_value = (crc_value>>1)^
				(((crc_value^c)&0x01)?poly:0);
			c >>= 1;
		}
	}
	return	crc_value;
}

/*
 .    This sets the internal hardware table to filter out unwanted multicast
 .    packets before they take up memory.
 .
 .    The SMC chip uses a hash table where the high 6 bits of the CRC of
 .    address are the offset into the table.  If that bit is 1, then the
 .    multicast packet is accepted.  Otherwise, it's dropped silently.
 .
 .    To use the 6 bits as an offset into the table, the high 3 bits are the
 .    number of the 8 bit register, while the low 3 bits are the bit within
 .    that register.
 .
 .    This routine is based very heavily on the one provided by Peter Cammaert.
*/
static void
smc_setmulticast(unsigned long ioaddr, int count, struct dev_mc_list *addrs)
{
	int			i;
	unsigned char		multicast_table[ 8 ];
	struct dev_mc_list	*cur_addr;

	/* table for flipping the order of 3 bits */
	static unsigned char invert3[] = { 0, 4, 2, 6, 1, 5, 3, 7 };

	/* start with a table of all zeros: reject all */
	memset( multicast_table, 0, sizeof( multicast_table ) );

	cur_addr = addrs;
	for ( i = 0; i < count ; i ++, cur_addr = cur_addr->next  ) {
		int position;

		/* do we have a pointer here? */
		if ( !cur_addr )
			break;
		/* make sure this is a multicast address - shouldn't this
		   be a given if we have it here ? */
		if ( !( *cur_addr->dmi_addr & 1 ) )
			continue;

		/* only use the low order bits */
		position = crc32( cur_addr->dmi_addr, 6 ) & 0x3f;

		/* do some messy swapping to put the bit in the right spot */
		multicast_table[invert3[position&7]] |=
					(1<<invert3[(position>>3)&7]);

	}
	/* now, the table can be loaded into the chipset */
	SMC_SELECT_BANK( 3 );
	SMC_SET_MCAST( multicast_table );
	SMC_SELECT_BANK( 2 );
}

/*
 . This routine will, depending on the values passed to it,
 . either make it accept multicast packets, go into
 . promiscuous mode ( for TCPDUMP and cousins ) or accept
 . a select set of multicast packets
*/
static void smc_set_multicast_list(struct net_device *dev)
{
        unsigned long flags;
	struct smc_local *lp = (struct smc_local *)dev->priv;
	unsigned long ioaddr = dev->base_addr;

	PRINTK2("%s: %s\n", dev->name, __FUNCTION__);

        local_irq_save(flags);

#ifdef SMC_USE_PXA_DMA_IRQ
        while (lp->dma_state != SMC_PXA_DMA_STATE_IDLE) {
		local_irq_restore(flags);
		local_irq_save(flags);
	}
#endif
        
	SMC_SELECT_BANK(0);
	if ( dev->flags & IFF_PROMISC ) {
		PRINTK2("%s: RCR_PRMS\n", dev->name);
		lp->rcr_cur_mode |= RCR_PRMS;
		SMC_SET_RCR( lp->rcr_cur_mode );
	}

/* BUG?  I never disable promiscuous mode if multicasting was turned on.
   Now, I turn off promiscuous mode, but I don't do anything to multicasting
   when promiscuous mode is turned on.
*/

	/* Here, I am setting this to accept all multicast packets.
	   I don't need to zero the multicast table, because the flag is
	   checked before the table is
	*/
	else if (dev->flags & IFF_ALLMULTI) {
		lp->rcr_cur_mode |= RCR_ALMUL;
		SMC_SET_RCR( lp->rcr_cur_mode );
		PRINTK2("%s: RCR_ALMUL\n", dev->name);
	}

	/* We just get all multicast packets even if we only want them
	 . from one source.  This will be changed at some future
	 . point. */
	else if (dev->mc_count ) {
		/* support hardware multicasting */

		/* be sure I get rid of flags I might have set */
		lp->rcr_cur_mode &= ~(RCR_PRMS | RCR_ALMUL);
		SMC_SET_RCR( lp->rcr_cur_mode );
		/* NOTE: this has to set the bank, so make sure it is the
		   last thing called.  The bank is set to zero at the top */
		smc_setmulticast( ioaddr, dev->mc_count, dev->mc_list );
	} else {
		PRINTK2("%s: ~(RCR_PRMS|RCR_ALMUL)\n", dev->name);
		lp->rcr_cur_mode &= ~(RCR_PRMS | RCR_ALMUL);
		SMC_SET_RCR( lp->rcr_cur_mode );

		/*
		  since I'm disabling all multicast entirely, I need to
		  clear the multicast list
		*/
		SMC_SELECT_BANK( 3 );
		SMC_CLEAR_MCAST();
	}
        
        SMC_SELECT_BANK( 2 );
        
        local_irq_restore(flags);
}


/*
 * Open and Initialize the board
 *
 * Set up everything, reset the card, etc ..
 */
static int
smc_open(struct net_device *dev)
{
	struct smc_local *lp = (struct smc_local *)dev->priv;
	unsigned long ioaddr = dev->base_addr;
#ifdef SMC_USE_PXA_DMA
	int dma = lp->dma; 
#endif

        PRINTK2("%s: %s()\n", dev->name, __FUNCTION__);

#ifdef SMC_USE_PXA_DMA
        /* Wait until current DMA operation will finish */
        if (lp->dma_state != SMC_PXA_DMA_STATE_IDLE) {
                return -EBUSY;
        }
#endif

	/* clear out all the junk that was put here before... */
	memset(dev->priv, 0, sizeof(struct smc_local));
#ifdef SMC_USE_PXA_DMA
	lp->dma = dma;
        lp->dma_state = SMC_PXA_DMA_STATE_IDLE;
#endif

	// Setup the default Register Modes
	lp->tcr_cur_mode = TCR_DEFAULT;
	lp->rcr_cur_mode = RCR_DEFAULT;
	lp->rpc_cur_mode = RPC_DEFAULT;

	/* Set default parameters */
#ifdef CONFIG_ARCH_RAMSES
	lp->ctl_autoneg = 0;
	lp->ctl_rfduplx = 0;
	lp->ctl_rspeed = 10;
#else
	lp->ctl_autoneg = 1;
	lp->ctl_rfduplx = 1;
	lp->ctl_rspeed = 100;
#endif

	SMC_SELECT_BANK(3);
	lp->version = SMC_GET_REV() & 0xff;

	/* reset the hardware */
	smc_reset(dev);
	smc_enable(dev);

	SMC_SELECT_BANK( 1 );
	SMC_SET_MAC_ADDR(dev->dev_addr);
	SMC_SELECT_BANK( 2 );

	/* Configure the PHY */
	if (lp->version >= 0x70)
		smc_phy_configure(dev);

        PRINTK2("%s:%d: %s: netif_start_queue()\n",
               __FILE__, __LINE__, __FUNCTION__);
	netif_start_queue(dev);
	return 0;
}

/*----------------------------------------------------
 . smc_close
 .
 . this makes the board clean up everything that it can
 . and not talk to the outside world.   Caused by
 . an 'ifconfig ethX down'
 .
 -----------------------------------------------------*/
static int
smc_close(struct net_device *dev)
{
	PRINTK2("%s: %s\n", dev->name, __FUNCTION__);

        PRINTK2("%s:%d: %s: netif_stop_queue()\n",
               __FILE__, __LINE__, __FUNCTION__);
	netif_stop_queue(dev);

	/* clear everything */
	smc_pxa_dma_stop(dev);
	smc_shutdown(dev->base_addr);
	smc_pxa_dma_reset(dev);

	return 0;
}

/*------------------------------------------------------------
 . Get the current statistics.
 . This may be called with the card open or closed.
 .-------------------------------------------------------------*/
static struct net_device_stats *
smc_query_statistics(struct net_device *dev)
{
	struct smc_local *lp = (struct smc_local *)dev->priv;

	PRINTK2("%s: %s\n", dev->name, __FUNCTION__);

	return &lp->stats;
}

/*----------------------------------------------------------------------
 . smc_findirq
 .
 . This routine has a simple purpose -- make the SMC chip generate an
 . interrupt, so an auto-detect routine can detect it, and find the IRQ,
 ------------------------------------------------------------------------
*/
int __init
smc_findirq( unsigned long ioaddr )
{
	int timeout = 20;
	unsigned long cookie;

	PRINTK2("%s: %s\n", CARDNAME, __FUNCTION__);

	cookie = probe_irq_on();

	/*
	 * What I try to do here is trigger an ALLOC_INT. This is done
	 * by allocating a small chunk of memory, which will give an interrupt
	 * when done.
	 */

	/* enable ALLOCation interrupts ONLY */
	SMC_SELECT_BANK(2);
	SMC_SET_INT_MASK( IM_ALLOC_INT );

	/*
 	 . Allocate 512 bytes of memory.  Note that the chip was just
	 . reset so all the memory is available
	*/
	SMC_SET_MMU_CMD( MC_ALLOC | 1 );

	/*
	 . Wait until positive that the interrupt has been generated
	*/
	do {
		int int_status;
		udelay(10);
		int_status = SMC_GET_INT();
		if (int_status & IM_ALLOC_INT)
			break;		/* got the interrupt */
	} while (--timeout);

	/* there is really nothing that I can do here if timeout fails,
	   as autoirq_report will return a 0 anyway, which is what I
	   want in this case.   Plus, the clean up is needed in both
	   cases.  */

	/* and disable all interrupts again */
	SMC_SET_INT_MASK( 0 );

	/* and return what I found */
	return probe_irq_off(cookie);
}

/*----------------------------------------------------------------------
 . Function: smc_probe( unsigned long ioaddr )
 .
 . Purpose:
 .	Tests to see if a given ioaddr points to an SMC91x chip.
 .	Returns a 0 on success
 .
 . Algorithm:
 .	(1) see if the high byte of BANK_SELECT is 0x33
 . 	(2) compare the ioaddr with the base register's address
 .	(3) see if I recognize the chip ID in the appropriate register
 .
 .---------------------------------------------------------------------
 */
/*---------------------------------------------------------------
 . Here I do typical initialization tasks.
 .
 . o  Initialize the structure if needed
 . o  print out my vanity message if not done so already
 . o  print out what type of hardware is detected
 . o  print out the ethernet address
 . o  find the IRQ
 . o  set up my private data
 . o  configure the dev structure with my subroutines
 . o  actually GRAB the irq.
 . o  GRAB the region
 .-----------------------------------------------------------------
*/
static int __init
smc_probe(struct net_device *dev, unsigned long ioaddr)
{
	struct smc_local *lp = (struct smc_local *)dev->priv;
	static int version_printed = 0;
	int i, retval;
	unsigned int val, revision_register;
	const char *version_string;

	PRINTK2("%s: %s\n", CARDNAME, __FUNCTION__);

	/* Grab the region so that no one else tries to probe our ioports. */
	if (!request_region(ioaddr, SMC_IO_EXTENT, dev->name))
		return -EBUSY;

	/* First, see if the high byte is 0x33 */
	val = SMC_CURRENT_BANK();
	PRINTK2("%s: bank signature probe returned 0x%04x\n", CARDNAME, val);
	if ( (val & 0xFF00) != 0x3300 ) {
		if ( (val & 0xFF) == 0x33 ) {
			printk( KERN_WARNING
				"%s: Detected possible byte-swapped interface"
				" at IOADDR 0x%lx\n", CARDNAME, ioaddr);
		}
		retval = -ENODEV;
		goto err_out;
	}

	/* The above MIGHT indicate a device, but I need to write to further
 	 	test this.  */
	SMC_SELECT_BANK(0);
	val = SMC_CURRENT_BANK();
	if ( (val & 0xFF00 ) != 0x3300 ) {
		retval = -ENODEV;
		goto err_out;
	}

	/* well, we've already written once, so hopefully another time won't
 	   hurt.  This time, I need to switch the bank register to bank 1,
	   so I can access the base address register */
	SMC_SELECT_BANK(1);
	val = SMC_GET_BASE();
	val = ((val & 0x1F00) >> 3) << SMC_IO_SHIFT;
	if ( (ioaddr & ((PAGE_SIZE-1)<<SMC_IO_SHIFT)) != val ) {
		printk( "%s: IOADDR %lx doesn't match configuration (%x).\n",
			CARDNAME, ioaddr, val );
	}

	/*  check if the revision register is something that I recognize.
	    These might need to be added to later, as future revisions
	    could be added.  */
	SMC_SELECT_BANK(3);
	revision_register = SMC_GET_REV();
	PRINTK2("%s: revision = 0x%04x\n", CARDNAME, revision_register);
	version_string = chip_ids[ (revision_register >> 4) & 0xF];
	if (!version_string || (revision_register & 0xff00) != 0x3300) {
		/* I don't recognize this chip, so... */
		printk( "%s: IO 0x%lx: Unrecognized revision register 0x%04x"
			", Contact author.\n", CARDNAME,
			ioaddr, revision_register);

		retval = -ENODEV;
		goto err_out;
	}

	/* At this point I'll assume that the chip is an SMC91x. */
	if (version_printed++ == 0)
		printk("%s", version);

	/* set the private data to zero by default */
	memset(lp, 0, sizeof(struct smc_local));

	/* fill in some of the fields */
	dev->base_addr = ioaddr;
	lp->version = revision_register & 0xff;

	/* Get the MAC address */
	SMC_SELECT_BANK( 1 );
	SMC_GET_MAC_ADDR(dev->dev_addr);
	SMC_SELECT_BANK( 2 );
	
	/* now, reset the chip, and put it into a known state */
	smc_reset( dev );

	/*
	 . If dev->irq is 0, then the device has to be banged on to see
	 . what the IRQ is.
 	 .
	 . This banging doesn't always detect the IRQ, for unknown reasons.
	 . a workaround is to reset the chip and try again.
	 .
	 . Interestingly, the DOS packet driver *SETS* the IRQ on the card to
	 . be what is requested on the command line.   I don't do that, mostly
	 . because the card that I have uses a non-standard method of accessing
	 . the IRQs, and because this _should_ work in most configurations.
	 .
	 . Specifying an IRQ is done with the assumption that the user knows
	 . what (s)he is doing.  No checking is done!!!!
 	 .
	*/
	if ( dev->irq < 1 ) {
		int	trials;

		trials = 3;
		while ( trials-- ) {
			dev->irq = smc_findirq( ioaddr );
			if ( dev->irq )
				break;
			/* kick the card and try again */
			smc_reset( dev );
		}
	}
	if (dev->irq == 0 ) {
		printk("%s: Couldn't autodetect your IRQ. Use irq=xx.\n",
			dev->name);
		retval = -ENODEV;
		goto err_out;
	}
	dev->irq = irq_cannonicalize(dev->irq);

	/* now, print out the card info, in a short format.. */
	printk( "%s: %s (rev %d) at %#lx IRQ %d%s%s\n",
		dev->name, version_string, revision_register & 0x0f,
		ioaddr, dev->irq, nowait ? " [nowait]" : "",
		THROTTLE_TX_PKTS ? " [throttle_tx]" : "" );

	/* Print the Ethernet address */
	printk("%s: Ethernet addr: ", dev->name);
	for (i = 0; i < 5; i++)
		printk("%2.2x:", dev->dev_addr[i] );
	printk("%2.2x\n", dev->dev_addr[5] );

	/* Fill in the fields of the device structure with ethernet values. */
	ether_setup(dev);

	/* Grab the IRQ */
      	retval = request_irq(dev->irq, &smc_interrupt, 0, dev->name, dev);
      	if (retval) {
  	  	goto err_out;
      	}

	dev->open		        = smc_open;
	dev->stop		        = smc_close;
	dev->hard_start_xmit    	= smc_hard_start_xmit;
	dev->tx_timeout		    	= smc_timeout;
	dev->watchdog_timeo		= HZ/10;
	dev->get_stats			= smc_query_statistics;
	dev->set_multicast_list 	= smc_set_multicast_list;

	return 0;

err_out:
	release_region(ioaddr, SMC_IO_EXTENT);
	return retval;
}

/*-------------------------------------------------------------------------
 |
 | smc_init( void )
 |   Input parameters:
 |	dev->base_addr == 0, try to find all possible locations
 |	dev->base_addr > 0x1ff, this is the address to check
 |	dev->base_addr == <anything else>, return failure code
 |
 |   Output:
 |	0 --> there is a device
 |	anything else, error
 |
 ---------------------------------------------------------------------------
*/

#ifdef CONFIG_DPM
static int
smc91x_scale(struct bus_op_point * op, u32 level)
{
	PRINTK1("+++: in smc91x_scale()\n");
	/* linux-pm-TODO */
	return 0;
}

static int
smc91x_suspend(struct device * dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		PRINTK1("%s: %s suspending\n", dev->name, __FUNCTION__);
		netif_stop_queue(global_dev);
		netif_carrier_off(global_dev);
		smc_pxa_dma_stop(global_dev);
		smc_shutdown(global_dev->base_addr);
		smc_pxa_dma_reset(global_dev);
		break;
	}
	return 0;
}

static int
smc91x_resume(struct device * dev, u32 level)
{
	unsigned long ioaddr = global_dev->base_addr;
	struct smc_local *lp = (struct smc_local *)global_dev->priv;
	
	switch (level) {
	case RESUME_POWER_ON:
		smc_reset(global_dev);
		smc_enable(global_dev);
		SMC_SELECT_BANK( 1 );
		SMC_SET_MAC_ADDR(global_dev->dev_addr);
		SMC_SELECT_BANK( 2 );
		if (lp->version >= 0x70)
	          smc_phy_configure(global_dev);
		PRINTK1("%s: %s resumed\n", dev->name, __FUNCTION__);
		netif_carrier_on(global_dev);
		netif_wake_queue(global_dev);
		break;
	}
			
	return 0;
}
#endif /* CONFIG_DPM */

#ifdef CONFIG_PM
static int smc_pm_callback(struct pm_dev *dev, pm_request_t rqst, void *data)
{
	unsigned long ioaddr = global_dev->base_addr;
	struct smc_local *lp = (struct smc_local *)global_dev->priv;

	switch (rqst) {
	case PM_SUSPEND:
		smc_pxa_dma_stop(global_dev);
		smc_shutdown(global_dev->base_addr);
		smc_pxa_dma_reset(global_dev);
		break;
	case PM_RESUME:
		smc_reset(global_dev);
		smc_enable(global_dev);
		SMC_SELECT_BANK( 1 );
	        SMC_SET_MAC_ADDR(global_dev->dev_addr);
		SMC_SELECT_BANK( 2 );
	        if (lp->version >= 0x70)
        	        smc_phy_configure(global_dev);
		break;
	}
	return 0;
}

#endif


#ifdef SMC_USE_PXA_DMA
static void
smc_pxa_dma_irq(int dma, void *dev_ptr, struct pt_regs *regs)
{
#ifdef SMC_USE_PXA_DMA_IRQ
	struct net_device *dev = dev_ptr;
	struct smc_local  *lp = (struct smc_local *)dev->priv;
	unsigned long ioaddr = dev->base_addr;
        unsigned long flags;

	PRINTK2("%s: %s\n", dev->name, __FUNCTION__);

	if (unlikely(SMC_CURRENT_BANK() != 0x3302)) {
		PRINTK( "DMA interrupt while the register's bank is wrong\n");
		BUG();
	}

	DCSR(dma) &= ~DCSR_STOPIRQEN;

	switch (lp->dma_state) {
	case SMC_PXA_DMA_STATE_RX:
		/* Packet received. */
		pci_unmap_single(NULL, lp->rx_dma_buf, lp->rx_dma_len, 
				 PCI_DMA_FROMDEVICE);
                SMC_TEST_SKB(lp->rx_skb);
		smc_rcv_finish(dev, lp->rx_skb);
		break;

	case SMC_PXA_DMA_STATE_TX:
		/* Packet transmitted. */
		pci_unmap_single(NULL, lp->tx_dma_buf, lp->tx_dma_len, 
				 PCI_DMA_TODEVICE);
                SMC_TEST_SKB(lp->tx_skb);
		smc_send_finish(dev, lp->tx_skb);
		local_irq_save(flags);
		lp->saved_mask |= (IM_TX_INT | IM_TX_EMPTY_INT);
		local_irq_restore(flags);
		break;

	default:
		PRINTK("%s: DMA interrupt with no DMA transfer.\n", dev->name);
		return;
        }

	lp->dma_state = SMC_PXA_DMA_STATE_UNSURE;

	/* Receive packets if there are any. */
	if (SMC_GET_INT() & IM_RCV_INT) {
		smc_rcv(dev);
	} else if (lp->wait_dma_skb != NULL) {
		netif_wake_queue(dev);
		if (smc_tx_alloc_buf(lp->wait_dma_skb, dev) == -EBUSY) {
			local_irq_save(flags);
			lp->saved_mask |= IM_ALLOC_INT;
			local_irq_restore(flags);
		}
		lp->wait_dma_skb = NULL;
	}

	if (lp->dma_state == SMC_PXA_DMA_STATE_UNSURE) {
#if 0
		lp->dma_state = SMC_PXA_DMA_STATE_IDLE;
		smc_interrupt(dev->irq, dev, regs);
#else
		/* FIXME: calling smc_interrupt() directly mysteriously
		   crashes the kernel for unknown reasons. */
		local_irq_save(flags);
		lp->dma_state = SMC_PXA_DMA_STATE_IDLE;
		SMC_SET_INT_MASK( SMC_GET_INT_MASK() | lp->saved_mask );
		lp->saved_mask = 0;
		local_irq_restore(flags);
#endif
	}
#endif
}
#endif

static int __init
smc_init(void)
{
	int ret;

	PRINTK2("%s: %s\n", CARDNAME, __FUNCTION__);

#ifdef MODULE
	if (io == -1)
		printk( KERN_WARNING 
			"%s: You shouldn't use auto-probing with insmod!\n",
			CARDNAME );
#endif

	if (global_dev) {
		printk("%s: already initialized.\n", CARDNAME);
		return -EBUSY;
	}

	global_dev = init_etherdev(0, sizeof(struct smc_local));
	if (!global_dev) {
		printk("%s: could not allocate device.\n", CARDNAME);
		return -ENOMEM;
	}
	SET_MODULE_OWNER(global_dev);

	/* copy the parameters from insmod into the device structure */
	if (io != -1)
		global_dev->base_addr	= io;
	if (irq != -1)
		global_dev->irq	= irq;

#ifdef CONFIG_ISA
	/*  try a specific location */
	if (global_dev->base_addr > 0x1ff)
		ret = smc_probe(global_dev, global_dev->base_addr);
	else if (global_dev->base_addr != 0)
		ret = -ENXIO;
	else {
		int i;

		/* check every ethernet address */
		for (i = 0; smc_portlist[i]; i++) {
			ret = smc_probe(global_dev, smc_portlist[i]);
			if (ret == 0)
				break;
		}
	}
#elif defined(CONFIG_ARCH_LUBBOCK)
	{
		int ioaddr = LUBBOCK_ETH_VIRT + (0x300 << 2);
		volatile int *attaddr = (int *)(LUBBOCK_ETH_VIRT + 0x100000);
		unsigned long flags;

		/* first reset, then enable the device. Sequence is critical */
		local_irq_save(flags);
		attaddr[ECOR] |= ECOR_RESET;
		udelay(100);
		attaddr[ECOR] &= ~ECOR_RESET;
		attaddr[ECOR] |= ECOR_ENABLE;

		/* force 16-bit mode */
		attaddr[ECSR] &= ~ECSR_IOIS8;
		mdelay(1);
		local_irq_restore(flags);

		global_dev->irq = LUBBOCK_ETH_IRQ;
		ret = smc_probe(global_dev, ioaddr);
	}
#elif defined(CONFIG_ARCH_PXA_IDP)
	{
		int ioaddr = IDP_ETH_BASE + 0x300;
		global_dev->irq = SMC_IRQ;
		ret = smc_probe(global_dev, ioaddr);
	}
#elif defined(CONFIG_ARCH_RAMSES)
	{
		int ioaddr = RAMSES_ETH_BASE + 0x300;
		global_dev->irq = SMC_IRQ;
		ret = smc_probe(global_dev, ioaddr);
	}
#elif defined(CONFIG_MACH_OMAP_PERSEUS2)
	{
		int ioaddr = PERSEUS2_FPGA_ETHR_BASE + 0x300;
		global_dev->irq = SMC_IRQ;
		ret = smc_probe(global_dev, ioaddr);
	}
#else
	if (global_dev->base_addr == -1) {
		printk(KERN_WARNING"%s: SMC91X_BASE_ADDR not set!\n", CARDNAME);
		ret = -ENXIO;
	} else {
		void *ioaddr = ioremap(global_dev->base_addr, SMC_IO_EXTENT);
		ret = smc_probe(global_dev, (unsigned long)ioaddr);
		if (ret != 0)
			iounmap(ioaddr);
	}
#endif

#ifdef SMC_USE_PXA_DMA
	if (ret == 0) {
		struct smc_local *lp = (struct smc_local *)global_dev->priv;
		int dma;
		
		dma = pxa_request_dma(global_dev->name, DMA_PRIO_HIGH,
				      smc_pxa_dma_irq, global_dev);
		if (dma >= 0) {
			lp->dma = dma;
			PRINTK0("%s: using DMA channel %d for RX\n", 
			       global_dev->name, dma);
		} else {
			printk(KERN_WARNING "%s: no DMA channel "
			       "is available\n", global_dev->name);
			ret = -ENOMEM;
		}
	}
#endif

#ifdef CONFIG_PM
	if (ret == 0) {
		struct smc_local *lp = (struct smc_local *)global_dev->priv;
		lp->pm = pm_register(PM_SYS_UNKNOWN, 0x73393178, smc_pm_callback);
	}
#endif
#ifdef CONFIG_DPM
	if (ret == 0) {
		/* register this device/driver with ldm */
		smc91x_ldm_register(global_dev);
	}
#endif /* CONFIG_DPM */

	if (ret != 0) {
		printk("%s: not found.\n", CARDNAME);
		kfree(global_dev->priv);
		unregister_netdev(global_dev);
		kfree(global_dev);
	}

	return ret;
}

static void __exit
smc_cleanup(void)
{
	unregister_netdev(global_dev);
#ifdef CONFIG_PM
	{
		struct smc_local *lp = (struct smc_local *)global_dev->priv;
		pm_unregister(lp->pm);
	}
#endif
#ifdef CONFIG_DPM
	/* unregister device/driver from ldm */
	smc91x_ldm_unregister(global_dev);
#endif
	free_irq(global_dev->irq, global_dev);
	release_region(global_dev->base_addr, SMC_IO_EXTENT);

#ifndef CONFIG_ISA
	iounmap((void *)global_dev->base_addr);
#endif

	kfree(global_dev);
	global_dev = NULL;
}

module_init(smc_init);
module_exit(smc_cleanup);

