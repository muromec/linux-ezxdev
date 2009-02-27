/*
 * linux/drivers/net/irda/pxaficp_ir.c
 *
 * Ported from sa1100_ir.c by Russell King
 *
 * Changes copyright (C) 2003 MontaVista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Infra-red driver (SIR/FIR) for the PXA250 embedded microprocessor
 *
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/slab.h>
#include <linux/rtnetlink.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/pm.h>

#include <net/irda/irda.h>
#include <net/irda/irmod.h>
#include <net/irda/wrapper.h>
#include <net/irda/irda_device.h>

#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/arch/dma.h> 
#include <asm/arch/mainstone.h>

//#define PXAFICP_IR_DEBUG 1
#if PXAFICP_IR_DEBUG
static unsigned int pxaficp_ir_debug = 1;
#else
#define pxaficp_ir_debug 0
#endif

/* If you use DBPXA250, you can unremark following definition to enable LED for debug purpose */
/* #define DEBUG_LED_ON */


#define IrSR_RXPL_NEG_IS_ZERO (1<<4)
#define IrSR_RXPL_POS_IS_ZERO 0x0
#define IrSR_TXPL_NEG_IS_ZERO (1<<3)
#define IrSR_TXPL_POS_IS_ZERO 0x0
#define IrSR_XMODE_PULSE_1_6  (1<<2)
#define IrSR_XMODE_PULSE_3_16 0x0
#define IrSR_RCVEIR_IR_MODE   (1<<1)
#define IrSR_RCVEIR_UART_MODE 0x0
#define IrSR_XMITIR_IR_MODE   (1<<0)
#define IrSR_XMITIR_UART_MODE 0x0

#define IrSR_IR_RECEIVE_ON (\
                IrSR_RXPL_NEG_IS_ZERO | \
                IrSR_TXPL_POS_IS_ZERO | \
                IrSR_XMODE_PULSE_3_16 | \
                IrSR_RCVEIR_IR_MODE   | \
                IrSR_XMITIR_UART_MODE)

#define IrSR_IR_TRANSMIT_ON (\
                IrSR_RXPL_NEG_IS_ZERO | \
                IrSR_TXPL_POS_IS_ZERO | \
                IrSR_XMODE_PULSE_3_16 | \
                IrSR_RCVEIR_UART_MODE | \
                IrSR_XMITIR_IR_MODE)

/*
 * Our netdevice.  There is only ever one of these.
 */
static struct net_device *netdev = NULL;
struct pxa_irda {
	unsigned char		open;
	int			speed;
	int			newspeed;
	int			mtt;

	unsigned char		*dma_rx_buff;
	unsigned char		*dma_tx_buff;
	dma_addr_t		dma_rx_buff_phy;
	dma_addr_t		dma_tx_buff_phy;
	unsigned int		dma_tx_buff_len;
	int			txdma;
	int			rxdma;

	struct net_device_stats	stats;
	struct irlap_cb		*irlap;
	struct qos_info		qos;

	iobuff_t		tx_buff;
	iobuff_t		rx_buff;

#ifdef CONFIG_PM
	struct pm_dev		*pmdev;
#endif
};

static int pxa_irda_startup(struct pxa_irda *si);
static void pxa_irda_shutdown(struct pxa_irda *si);

#ifdef CONFIG_DPM
#include <linux/device.h>

static int pxairda_suspend(struct device *dev, u32 state, u32 level);
static int pxairda_resume(struct device *dev, u32 level);

static struct device_driver pxairda_driver_ldm = {
	name:          "pxa-irda",
	devclass:      NULL,
	probe:         NULL,
	suspend:       pxairda_suspend,
	resume:        pxairda_resume,
	scale:         NULL,
	remove:        NULL,
};

static struct device pxairda_device_ldm = {
	name:         "PXA IRDA",
	bus_id:       "pxaficp-irda",
	driver:       NULL,
	power_state:  DPM_POWER_ON,
};

static void
pxairda_ldm_register(void)
{
	extern void pxaopb_driver_register(struct device_driver *driver);
	extern void pxaopb_device_register(struct device *device);
	
	pxaopb_driver_register(&pxairda_driver_ldm);
	pxaopb_device_register(&pxairda_device_ldm);
}

static void
pxairda_ldm_unregister(void)
{
	extern void pxaopb_driver_unregister(struct device_driver *driver);
	extern void pxaopb_device_unregister(struct device *device);
  
	pxaopb_device_unregister(&pxairda_device_ldm);
	pxaopb_driver_unregister(&pxairda_driver_ldm);
}

static int
pxairda_resume(struct device *dev, u32 level)
{
	struct pxa_irda *si = netdev->priv;

	switch (level) {
	case RESUME_POWER_ON:
		if (si && si->open) {
			/*
			 * If we missed a speed change, initialise at
			 * the new speed directly.  It is debatable
			 * whether this is actually required, but in
			 * the interests of continuing from where we
			 * left off it is desireable.  The converse
			 * argument is that we should re-negotiate at
			 * 9600 baud again.
			 */
			if (si->newspeed) {
				si->speed = si->newspeed;
				si->newspeed = 0;
			}
			/* add here: restart IR transceiver on the board*/
#ifdef CONFIG_ARCH_MAINSTONE
			/* this seems to be redundant with what is
			   done in pxa_irda_startup(), specifically,
			   MST_MSCWR1 &=
			   ~MST_MISC_WR_IRDA_TRANS_MODE */
			MST_MSCWR1 &= ~MST_MSCWR1_IRDA_MASK;
#endif
			pxa_irda_startup(si);
			enable_irq(IRQ_STUART);
			enable_irq(IRQ_ICP);
			/*
			 * This automatically wakes up the queue
			 */
			netif_device_attach(netdev);
		}
		break;
	}
	
	return 0;
}

static int
pxairda_suspend(struct device *dev, u32 state, u32 level)
{
	struct pxa_irda *si = netdev->priv;

	switch (level) {
	case SUSPEND_POWER_DOWN:

		if (si && si->open) {
			/*
			 * Stop the transmit queue
			 */
			netif_device_detach(netdev);
			disable_irq(IRQ_STUART);
			disable_irq(IRQ_ICP);
			pxa_irda_shutdown(si);
			/* add here: shutdown IR transceiver on the board */
#ifdef CONFIG_ARCH_MAINSTONE
			/* okay to turn this off here; will be re-enabled when
			   the driver is resumed; disable all power bits, then
			   turn on shutdown bit */
			MST_MSCWR1 &= ~MST_MSCWR1_IRDA_MASK;
			MST_MSCWR1 |= MST_MSCWR1_IRDA_OFF;
#endif
		}
		break;
	}
	
	return 0;
}
#endif /* CONFIG_DPM */

#define IS_FIR(si)		((si)->speed >= 4000000)
#define IRDA_FRAME_SIZE_LIMIT	2047

#define DEBUG_LED_RX	D21
#define DEBUG_LED_TX	D22

#define BOARD_IRDA_SIR	0
#define BOARD_IRDA_FIR	1

#ifdef CONFIG_ARCH_MAINSTONE
#define MST_MISC_WR_IRDA_MODE (1 << 4)
#define MST_MISC_WR_IRDA_TRANS_MODE (3 << 4)
inline static void board_irda_select(int irda_mode)
{
	if (irda_mode == BOARD_IRDA_SIR) {
		/* select SIR on DBPXA250 */
		MST_MSCWR1 &= ~MST_MISC_WR_IRDA_MODE;
	} else {
		/* select FIR on DBPXA250 */
		MST_MSCWR1 |= MST_MISC_WR_IRDA_MODE;
	}
}
#else
#error "Board is not supported by this driver"
#endif

inline static void pxa_irda_fir_dma_rx_start(struct pxa_irda *si)
{
	consistent_sync(si->dma_rx_buff, IRDA_FRAME_SIZE_LIMIT, PCI_DMA_FROMDEVICE);
	si->dma_rx_buff_phy = virt_to_bus(si->dma_rx_buff);
	DCSR(si->rxdma)  = DCSR_NODESC;
	DSADR(si->rxdma) = __PREG(ICDR);
	DTADR(si->rxdma) = si->dma_rx_buff_phy;
	DCMD(si->rxdma) = DCMD_INCTRGADDR | DCMD_FLOWSRC |  DCMD_WIDTH1 | DCMD_BURST32 | IRDA_FRAME_SIZE_LIMIT;
	DCSR(si->rxdma) |= DCSR_RUN;
}

inline static void pxa_irda_fir_dma_tx_start(struct pxa_irda *si)
{
	consistent_sync(si->dma_tx_buff, IRDA_FRAME_SIZE_LIMIT, PCI_DMA_TODEVICE);
	si->dma_tx_buff_phy = virt_to_bus(si->dma_tx_buff);
	DCSR(si->txdma)  = DCSR_NODESC;
	DSADR(si->txdma) = si->dma_tx_buff_phy;
	DTADR(si->txdma) = __PREG(ICDR);
	DCMD(si->txdma) = DCMD_INCSRCADDR | DCMD_FLOWTRG |  DCMD_ENDIRQEN | DCMD_WIDTH1 | DCMD_BURST32 | si->dma_tx_buff_len;
	DCSR(si->txdma) |= DCSR_RUN;
}



/*
 * Set the IrDA communications speed.
 */
static int pxa_irda_set_speed(struct pxa_irda *si, int speed)
{
	unsigned long flags;
	int divisor, ret = -EINVAL;
	
#ifdef DEBUG_LED_ON
	{
		if (speed == 9600) {
			MST_LEDDAT1 = 0x9600;
		} else if (speed == 19200) {
			MST_LEDDAT1 = 0x19200;
		} else if (speed == 38400) {
			MST_LEDDAT1 = 0x38400;
		} else if (speed == 57600) {
			MST_LEDDAT1 = 0x57600;
		} else if (speed == 115200) {
			MST_LEDDAT1 = 0x115200;
		} else if (speed == 4000000) {
			MST_LEDDAT1 = 0x4000000;
		}
	}
#endif

	switch (speed) {
	case 9600:	case 19200:	case 38400:
	case 57600:	case 115200:

		/* refer to PXA250/210 Developer's Manual 10-7 */
		/*  BaudRate = 14.7456 MHz / (16*Divisor) */
		divisor = 14745600 / (16 * speed) ;

		local_irq_save(flags);

		if (IS_FIR(si)) {
			/* stop RX DMA */
			DCSR(si->rxdma) &= ~DCSR_RUN;
			/* disable FICP */
			ICCR0 &= ~ICCR0_ITR;
			CKEN &= ~CKEN13_FICP;

			board_irda_select(BOARD_IRDA_SIR);
			/* configure GPIO46/47 */
			set_GPIO_mode(GPIO46_STRXD_MD);
			set_GPIO_mode(GPIO47_STTXD_MD);
			/* enable the STUART clock */
			CKEN |= CKEN5_STUART;
		}

		/* disable STUART first */
		STIER &= ~IER_UUE;

		/* access DLL & DLH */
		STLCR |= LCR_DLAB;
		STDLL = divisor;
		STDLH = 0;
		/* restore to access THR, RBR & IER */
		STLCR &= ~LCR_DLAB;
		si->speed = speed;
		STISR = IrSR_IR_RECEIVE_ON | IrSR_XMODE_PULSE_1_6;
		STIER = IER_UUE | IER_RLSE | IER_RAVIE | IER_RTIOE;

		local_irq_restore(flags);
		ret = 0;
		break;

	case 4000000:
		local_irq_save(flags);

		/* disable STUART */
		STIER = 0;
		STISR = 0;
		CKEN &= ~CKEN5_STUART;

		/* disable FICP first */
		ICCR0 &= ~ICCR0_ITR;

		board_irda_select(BOARD_IRDA_FIR);
		/* configure GPIO46/47 */
		set_GPIO_mode(GPIO46_ICPRXD_MD);
		set_GPIO_mode(GPIO47_ICPTXD_MD);
		/* enable the FICP clock */
		CKEN |= CKEN13_FICP;
		si->speed = speed;
		pxa_irda_fir_dma_rx_start(si);
		ICCR0 = ICCR0_ITR | ICCR0_RXE;

		local_irq_restore(flags);
		ret = 0;
		break;

	default:
		break;
	}

	return ret;
}



/* SIR interrupt service routine. */
static void pxa_irda_sir_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	struct net_device *dev = dev_id;
	struct pxa_irda *si = dev->priv;
	int iir,lsr,data;

	iir = STIIR;

	switch  (iir & 0x0F) {
	case 0x06:
	  	/* Receiver Line Status */
	  	lsr = STLSR;
		while (lsr & LSR_FIFOE) {
			data = STRBR;
			if (lsr & (LSR_OE | LSR_PE | LSR_FE | LSR_BI)) {
				printk(KERN_DEBUG "pxa_ir: sir receiving error\n");
				si->stats.rx_errors++;
				if (lsr & LSR_FE)
					si->stats.rx_frame_errors++;
				if (lsr & LSR_OE)
					si->stats.rx_fifo_errors++;
			} else {
				async_unwrap_char(dev, &si->stats, &si->rx_buff, data);
			}
			lsr = STLSR;
		}
		break;

	case 0x04:
	  	/* Received Data Available */
		/* forth through */

	case 0x0C:
	  	/* Character Timeout Indication	*/
	  	do  {
	            async_unwrap_char(dev, &si->stats, &si->rx_buff, STRBR);
	  	} while (STLSR & LSR_DR);
	  	dev->last_rx = jiffies;
	  	break;

	case 0x02:
	    	/* Transmit FIFO Data Request */
	    	while ((si->tx_buff.len) && (STLSR & LSR_TDRQ)) {
	    		STTHR = *si->tx_buff.data++;
			si->tx_buff.len -= 1;
	    	}

		if (si->tx_buff.len == 0) {
			si->stats.tx_packets++;
			si->stats.tx_bytes += si->tx_buff.data -
					      si->tx_buff.head;

                        /*
		 	* We need to ensure that the transmitter has
		 	* finished.
		 	*/
			while ((STLSR & LSR_TEMT) == 0) {
				rmb();
			}
			/*
		 	* Ok, we've finished transmitting.  Now enable
		 	* the receiver.  Sometimes we get a receive IRQ
		 	* immediately after a transmit...
		 	*/
#ifdef DEBUG_LED_ON
			DISCRETE_LED_ON(DEBUG_LED_RX);
			DISCRETE_LED_OFF(DEBUG_LED_TX);
#endif
			if (si->newspeed) {
				pxa_irda_set_speed(si, si->newspeed);
				si->newspeed = 0;
			} else {
				/* enable IR Receiver, disable IR Transmitter */
				STISR = IrSR_IR_RECEIVE_ON | IrSR_XMODE_PULSE_1_6;
				/* enable STUART and receive interrupts */
				STIER = IER_UUE | IER_RLSE | IER_RAVIE | IER_RTIOE;
			}
			/* I'm hungry! */
			netif_wake_queue(dev);
		}

		break;
	case 0:
		/* Modem Status */
		break;

	default:
		break;
	}
}

/* FIR Receive DMA interrupt handler */
static void pxa_irda_fir_dma_rx_irq(int channel, void *data, struct pt_regs *regs)
{
	int dcsr = DCSR(channel);
	DCSR(channel) &= ~DCSR_RUN;
	DCSR(channel) |= DCSR_STARTINTR|DCSR_ENDINTR|DCSR_BUSERR;
	if (dcsr & DCSR_BUSERR)  {
		printk(KERN_DEBUG "pxa_ir: fir rx dma bus error.\n");
	}
}

/* FIR Transmit DMA interrupt handler */
static void pxa_irda_fir_dma_tx_irq(int channel, void *data, struct pt_regs *regs)
{
	struct pxa_irda* si = data;
	int dcsr;

	dcsr = DCSR(channel);
	DCSR(channel) &= ~DCSR_RUN;

	if (dcsr & DCSR_ENDINTR)  {
		DCSR(channel) |= DCSR_ENDINTR;
		si->stats.tx_packets++;
		si->stats.tx_bytes += si->dma_tx_buff_len;
		while (ICSR1 & ICSR1_TBY)  {
			rmb();
		}
#ifdef DEBUG_LED_ON
		DISCRETE_LED_ON(DEBUG_LED_RX);
		DISCRETE_LED_OFF(DEBUG_LED_TX);
#endif
		/* minimal turn-around time delay */
		if (si->mtt) udelay(si->mtt);

		if (si->newspeed) {
			pxa_irda_set_speed(si, si->newspeed);
			si->newspeed = 0;
		} else {
			pxa_irda_fir_dma_rx_start(si);
			ICCR0 = ICCR0_ITR | ICCR0_RXE;
		}
		netif_wake_queue(netdev);
	}
}

/* EIF(Error in FIFO/End in Frame) handler for FIR */
static void pxa_irda_fir_irq_eif(struct pxa_irda *si, struct net_device *dev)
{
	unsigned int len, stat, data;

	/* Get the current data position. */
	len = DTADR(si->rxdma) - si->dma_rx_buff_phy;

	do {
		/* Read Status, and then Data. 	 */
		stat = ICSR1;
		rmb();
		data = ICDR;

		if (stat & (ICSR1_CRE | ICSR1_ROR)) {
			si->stats.rx_errors++;
			if (stat & ICSR1_CRE) {
				printk(KERN_DEBUG "pxa_ir: fir receive CRC error\n");
				si->stats.rx_crc_errors++;
			}
			if (stat & ICSR1_ROR) {
				printk(KERN_DEBUG "pxa_ir: fir receive overrun\n");
				si->stats.rx_frame_errors++;
			}
		} else	{
			si->dma_rx_buff[len++] = data;
		}
		/* If we hit the end of frame, there's no point in continuing. */
		if (stat & ICSR1_EOF) {
			break;
		}
	} while (ICSR0 & ICSR0_EIF);

	if (stat & ICSR1_EOF) {
		/* end of frame. */
		{
			struct sk_buff *skb;

			skb = alloc_skb(len+1,GFP_ATOMIC);
			if (!skb)  {
				printk(KERN_ERR "pxa_ir: fir out of memory for receive skb\n");
				si->stats.rx_dropped++;
				return;
			}

			/* Align IP header to 20 bytes  */
			skb_reserve(skb, 1);
			memcpy(skb->data,si->dma_rx_buff,len);
			skb_put(skb,len);
			/* Feed it to IrLAP  */
			skb->dev = dev;
			skb->mac.raw  = skb->data;
			skb->protocol = htons(ETH_P_IRDA);
			netif_rx(skb);

			si->stats.rx_packets++;
			si->stats.rx_bytes += len;

		}
		dev->last_rx = jiffies;
	}
}

/* FIR interrupt handler */
static void pxa_irda_fir_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	struct net_device *dev = dev_id;
	struct pxa_irda *si = dev->priv;
	int icsr0;

	/* stop RX DMA */
	DCSR(si->rxdma) &= ~DCSR_RUN;
	icsr0 = ICSR0;
	if (icsr0 & (ICSR0_FRE | ICSR0_RAB)) {
		if (icsr0 & ICSR0_FRE) {
		        printk(KERN_DEBUG "pxa_ir: fir receive frame error\n"); 
			si->stats.rx_frame_errors++;

		} else {
			printk(KERN_DEBUG "pxa_ir: fir receive abort\n");
			si->stats.rx_errors++;
		}
		ICSR0 |= ICSR0_FRE | ICSR0_RAB;
	}
	if (icsr0 & ICSR0_EIF) {
		/* An error in FIFO occues, or there is a end of frame */
		pxa_irda_fir_irq_eif(si, dev);
	}
	pxa_irda_fir_dma_rx_start(si);
}

/* hard_xmit interface of irda device */
static int pxa_irda_hard_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct pxa_irda *si = dev->priv;
	int speed = irda_get_next_speed(skb);

	/*
	 * Does this packet contain a request to change the interface
	 * speed?  If so, remember it until we complete the transmission
	 * of this frame.
	 */
	if (speed != si->speed && speed != -1)
		si->newspeed = speed;

	/*
	 * If this is an empty frame, we can bypass a lot.
	 */
	if (skb->len == 0) {
		if (si->newspeed) {
			si->newspeed = 0;
			pxa_irda_set_speed(si, speed);
		}
		dev_kfree_skb(skb);
		return 0;
	}


	if (!IS_FIR(si)) {

		netif_stop_queue(dev);

		si->tx_buff.data = si->tx_buff.head;
		si->tx_buff.len  = async_wrap_skb(skb, si->tx_buff.data, si->tx_buff.truesize);

#ifdef DEBUG_LED_ON
		DISCRETE_LED_OFF(DEBUG_LED_RX);
		DISCRETE_LED_ON(DEBUG_LED_TX);
#endif

		/* disable IR Receiver, enable IR Transmitter */
		STISR = IrSR_IR_TRANSMIT_ON | IrSR_XMODE_PULSE_1_6;

		/* It is said when GPIO47 switch from normal mode to SIR mode,
		   we must force GPIO low to avoid false start bit.
		   However it seems also ok when skip it */
		/* GPCR1 |= 0x00008000; */

		/* enable STUART and transmit interrupts */
		STIER = IER_UUE | IER_TIE;

		/* reset Transmitter FIFO to fire the interrupt */
		/* STFCR |= FCR_RESETTF; */
		/* but it doesn't alway work. it seems only when FIFO drops,
		 * the interrupt will occur. so we copy the codes
		 * in the interrupt handler here
		 */
		disable_irq(IRQ_STUART);
		while ((si->tx_buff.len) && (!(STLSR & LSR_TEMT) == 0)) {
			STTHR = *si->tx_buff.data++;
			si->tx_buff.len -= 1;
		};
		enable_irq(IRQ_STUART);

		if (si->tx_buff.len == 0) {
			si->stats.tx_packets++;
			si->stats.tx_bytes += si->tx_buff.data -
					      si->tx_buff.head;
                        /*
		 	* We need to ensure that the transmitter has
		 	* finished.
		 	*/
			while ((STLSR & LSR_TEMT) == 0) {
				rmb();
			}
			/*
		 	* Ok, we've finished transmitting.  Now enable
		 	* the receiver.  Sometimes we get a receive IRQ
		 	* immediately after a transmit...
		 	*/
#ifdef DEBUG_LED_ON
			DISCRETE_LED_ON(DEBUG_LED_RX);
			DISCRETE_LED_OFF(DEBUG_LED_TX);
#endif
			if (si->newspeed) {
				pxa_irda_set_speed(si, si->newspeed);
				si->newspeed = 0;
			} else {
				/* enable IR Receiver, disable IR Transmitter */
				STISR = IrSR_IR_RECEIVE_ON | IrSR_XMODE_PULSE_1_6;
				/* enable STUART and receive interrupts */
				STIER = IER_UUE | IER_RLSE | IER_RAVIE | IER_RTIOE;

			}
                        /* I'm hungry! */
			netif_wake_queue(dev);
		}
		dev_kfree_skb(skb);
	} else {
		si->mtt = irda_get_mtt(skb);

		netif_stop_queue(dev);
#ifdef DEBUG_LED_ON
		DISCRETE_LED_OFF(DEBUG_LED_RX);
		DISCRETE_LED_ON(DEBUG_LED_TX);
#endif
		si->dma_tx_buff_len = skb->len;
		memcpy(si->dma_tx_buff, skb->data, skb->len);

		if (si->mtt) udelay(si->mtt);
		pxa_irda_fir_dma_tx_start(si);
		ICCR0 = ICCR0_ITR | ICCR0_TXE;
		dev_kfree_skb(skb);
	}
	dev->trans_start = jiffies;
	return 0;
}

static int pxa_irda_ioctl(struct net_device *dev, struct ifreq *ifreq, int cmd)
{
	struct if_irda_req *rq = (struct if_irda_req *)ifreq;
	struct pxa_irda *si = dev->priv;
	int ret = -EOPNOTSUPP;

	switch (cmd) {
	case SIOCSBANDWIDTH:
		if (capable(CAP_NET_ADMIN)) {
			/*
			 * We are unable to set the speed if the
			 * device is not running.
			 */
			if (si->open) {
				ret = pxa_irda_set_speed(si,
						rq->ifr_baudrate);
			} else {
				printk(KERN_INFO "pxa_ir: SIOCSBANDWIDTH: !netif_running\n");
				ret = 0;
			}
		}
		break;

	case SIOCSMEDIABUSY:
		ret = -EPERM;
		if (capable(CAP_NET_ADMIN)) {
			irda_device_set_media_busy(dev, TRUE);
			ret = 0;
		}
		break;

	case SIOCGRECEIVING:
		rq->ifr_receiving = IS_FIR(si) ? 0
					: si->rx_buff.state != OUTSIDE_FRAME;
		break;

	default:
		break;
	}
	return ret;
}

static struct net_device_stats *pxa_irda_stats(struct net_device *dev)
{
	struct pxa_irda *si = dev->priv;
	return &si->stats;
}

static int pxa_irda_startup(struct pxa_irda *si)
{
	int ret;

	/* enable STUART interrupt to the processor */
	STMCR = MCR_OUT2;
	/* configure SIR frame format: StartBit - Data 7 ... Data 0 - Stop Bit */
	STLCR = LCR_WLS0 | LCR_WLS1;
	/* enable FIFO, we use FIFO to improve performance */
	STFCR = FCR_TRFIFOE | FCR_ITL_32;

	/* configure FICP ICCR2 */
	ICCR2 = ICCR2_TXP | ICCR2_TRIG_32;
	/* configure DMAC */
	DRCMR17 = si->rxdma | DRCMR_MAPVLD;
	DRCMR18 = si->txdma | DRCMR_MAPVLD;

	/* we start from SIR 9600 baudrate */
	/* disable FICP */
	ICCR0 &= ~ICCR0_ITR;
	CKEN &= ~CKEN13_FICP;

	board_irda_select(BOARD_IRDA_SIR);

	MST_MSCWR1 &= ~MST_MISC_WR_IRDA_TRANS_MODE;

	/* configure GPIO46/47 */
	set_GPIO_mode(GPIO46_STRXD_MD);
	set_GPIO_mode(GPIO47_STTXD_MD);
	/* enable the STUART clock */
	CKEN |= CKEN5_STUART;
	ret = pxa_irda_set_speed(si, si->speed = 9600);

	if (ret)
		return ret;

#ifdef DEBUG_LED_ON
	DISCRETE_LED_ON(DEBUG_LED_RX);
	DISCRETE_LED_OFF(DEBUG_LED_TX);
#endif
	printk(KERN_INFO "pxa_ir: irda startup\n");
	return 0;
}

static void pxa_irda_shutdown(struct pxa_irda *si)
{
	/* disable the STUART clock */
	CKEN &= ~CKEN5_STUART;
	/* disable STUART and interrupt */
	STIER = 0;
	/* disable STUART SIR mode */
	STISR = 0;

	/* disable the FICP clock */
	CKEN &= ~CKEN13_FICP;
	/* disable FICP */
	ICCR0 = 0;

#ifdef DEBUG_LED_ON
	DISCRETE_LED_OFF(DEBUG_LED_RX);
	DISCRETE_LED_OFF(DEBUG_LED_TX);
#endif
	printk(KERN_INFO "pxa_ir: irda shutdown\n");

}

static int pxa_irda_start(struct net_device *dev)
{
	struct pxa_irda *si = dev->priv;
	int err;

	MOD_INC_USE_COUNT;

	si->speed = 9600;

	err = request_irq(IRQ_STUART, pxa_irda_sir_irq, 0, dev->name, dev);
	if (err)
		goto err_irq1;

	err = request_irq(IRQ_ICP, pxa_irda_fir_irq, 0, dev->name, dev);
	if (err)
		goto err_irq2;

	/*
	 * The interrupt must remain disabled for now.
	 */
	disable_irq(IRQ_STUART);
	disable_irq(IRQ_ICP);

	si->rxdma = pxa_request_dma("FICP_RX",DMA_PRIO_LOW, pxa_irda_fir_dma_rx_irq, si);
	if (si->rxdma < 0)
		goto err_rx_dma;

	si->txdma = pxa_request_dma("FICP_TX",DMA_PRIO_LOW, pxa_irda_fir_dma_tx_irq, si);
	if (si->txdma < 0)
		goto err_tx_dma;

	if (!(si->dma_rx_buff = kmalloc(IRDA_FRAME_SIZE_LIMIT, 
					GFP_KERNEL | GFP_DMA))) {
		goto err_dma_rx_buff;
	}
	if (!(si->dma_tx_buff = kmalloc(IRDA_FRAME_SIZE_LIMIT, 
					GFP_KERNEL | GFP_DMA))) {
		goto err_dma_tx_buff;
	}

	consistent_sync(si->dma_rx_buff, IRDA_FRAME_SIZE_LIMIT, 
			PCI_DMA_FROMDEVICE);
	si->dma_rx_buff_phy = virt_to_bus(si->dma_rx_buff);

	consistent_sync(si->dma_tx_buff, IRDA_FRAME_SIZE_LIMIT, 
			PCI_DMA_TODEVICE);
	si->dma_tx_buff_phy = virt_to_bus(si->dma_tx_buff);


	/*
	 * Setup the serial port for the specified speed.
	 */
	err = pxa_irda_startup(si);
	if (err)
		goto err_startup;

	/*
	 * Open a new IrLAP layer instance.
	 */
	si->irlap = irlap_open(dev, &si->qos, "pxa");
	err = -ENOMEM;
	if (!si->irlap)
		goto err_irlap;

	/*
	 * Now enable the interrupt and start the queue
	 */
	si->open = 1;
	enable_irq(IRQ_STUART);
	enable_irq(IRQ_ICP);
	netif_start_queue(dev);

	printk(KERN_INFO "pxa_ir: irda driver opened\n");

	return 0;

err_irlap:
	si->open = 0;
	pxa_irda_shutdown(si);
err_startup:
	kfree(si->dma_tx_buff);
err_dma_tx_buff:
	kfree(si->dma_rx_buff);
err_dma_rx_buff:
	pxa_free_dma(si->txdma);
err_tx_dma:
	pxa_free_dma(si->rxdma);
err_rx_dma:
	free_irq(IRQ_ICP, dev);
err_irq2:
	free_irq(IRQ_STUART, dev);
err_irq1:
	MOD_DEC_USE_COUNT;
	return err;
}

static int pxa_irda_stop(struct net_device *dev)
{
	struct pxa_irda *si = dev->priv;

	disable_irq(IRQ_STUART);
	disable_irq(IRQ_ICP);
	pxa_irda_shutdown(si);

	/* Stop IrLAP */
	if (si->irlap) {
		irlap_close(si->irlap);
		si->irlap = NULL;
	}

	netif_stop_queue(dev);
	si->open = 0;

	free_irq(IRQ_STUART, dev);
	free_irq(IRQ_ICP, dev);

	pxa_free_dma(si->rxdma);
	pxa_free_dma(si->txdma);

	if (si->dma_rx_buff) kfree(si->dma_rx_buff);
	if (si->dma_tx_buff) kfree(si->dma_tx_buff);
	MOD_DEC_USE_COUNT;

	printk(KERN_INFO "pxa_ir: irda driver closed\n");
	return 0;
}


#ifdef CONFIG_PM
/*
 * Suspend the IrDA interface.
 */
static int pxa_irda_suspend(struct net_device *dev, int state)
{
	struct pxa_irda *si = dev->priv;
	printk(KERN_INFO "pxa_ir: irda suspend\n");
	if (si && si->open) {
		/*
		 * Stop the transmit queue
		 */
		netif_device_detach(dev);
		disable_irq(IRQ_STUART);
		disable_irq(IRQ_ICP);
		pxa_irda_shutdown(si);
		/* add here: shutdown IR transceiver on the board */
	}
	return 0;
}

/*
 * Resume the IrDA interface.
 */
static int pxa_irda_resume(struct net_device *dev)
{
	struct pxa_irda *si = dev->priv;

	printk(KERN_INFO "pxa_ir: irda resume\n");
	if (si && si->open) {
		/*
		 * If we missed a speed change, initialise at the new speed
		 * directly.  It is debatable whether this is actually
		 * required, but in the interests of continuing from where
		 * we left off it is desireable.  The converse argument is
		 * that we should re-negotiate at 9600 baud again.
		 */
		if (si->newspeed) {
			si->speed = si->newspeed;
			si->newspeed = 0;
		}
		/* add here: restart IR transceiver on the board*/
		pxa_irda_startup(si);
		enable_irq(IRQ_STUART);
		enable_irq(IRQ_ICP);
		/*
		 * This automatically wakes up the queue
		 */
		netif_device_attach(dev);
	}
	return 0;
}

static int pxa_irda_pmproc(struct pm_dev *dev, pm_request_t rqst, void *data)
{
	int ret;

	if (!dev->data)
		return -EINVAL;

	switch (rqst) {
	case PM_SUSPEND:
		ret = pxa_irda_suspend((struct net_device *)dev->data,
					  (int)data);
		break;

	case PM_RESUME:
		ret = pxa_irda_resume((struct net_device *)dev->data);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
#endif

static int pxa_irda_init_iobuf(iobuff_t *io, int size)
{
	io->head = kmalloc(size, GFP_KERNEL | GFP_DMA);
	if (io->head != NULL) {
		io->truesize = size;
		io->in_frame = FALSE;
		io->state    = OUTSIDE_FRAME;
		io->data     = io->head;
	}
	return io->head ? 0 : -ENOMEM;
}

static int pxa_irda_net_init(struct net_device *dev)
{
	struct pxa_irda *si = dev->priv;
	unsigned int baudrate_mask;
	int err = -ENOMEM;

	si = kmalloc(sizeof(struct pxa_irda), GFP_KERNEL);
	if (!si)
		goto out;

	memset(si, 0, sizeof(*si));

	/*
	 * Initialise the SIR buffers
	 */
	err = pxa_irda_init_iobuf(&si->rx_buff, 14384);
	if (err)
		goto out;
	err = pxa_irda_init_iobuf(&si->tx_buff, 4000);
	if (err)
		goto out_free_rx;

	dev->priv = si;
	dev->hard_start_xmit	= pxa_irda_hard_xmit;
	dev->open		= pxa_irda_start;
	dev->stop		= pxa_irda_stop;
	dev->do_ioctl		= pxa_irda_ioctl;
	dev->get_stats		= pxa_irda_stats;

	irda_device_setup(dev);
	irda_init_max_qos_capabilies(&si->qos);

	baudrate_mask = IR_9600|IR_19200|IR_38400|IR_57600|IR_115200;
	baudrate_mask |= IR_4000000 << 8;

	si->qos.baud_rate.bits &= baudrate_mask;
	si->qos.min_turn_time.bits = 7;  /* 1ms or more */

	irda_qos_bits_to_value(&si->qos);

#ifdef CONFIG_PM
	/*
	 * Power-Management is optional.
	 */
	si->pmdev = pm_register(PM_SYS_DEV, PM_SYS_IRDA, pxa_irda_pmproc);
	if (si->pmdev) {
		si->pmdev->data = dev;
		printk(KERN_DEBUG "pxa_ir: pm registered\n");
	} else {
		goto out_free_tx;
	}
#endif
	printk(KERN_INFO "pxa_ir: irda driver inited\n");

	return 0;
#ifdef CONFIG_PM
	/* ifdef'd to prevent compiler warning */
out_free_tx:
	kfree(si->tx_buff.head);
#endif
out_free_rx:
	kfree(si->rx_buff.head);
out:
	kfree(si);

	return err;
}

/*
 * Remove all traces of this driver module from the kernel, so we can't be
 * called.  Note that the device has already been stopped, so we don't have
 * to worry about interrupts or dma.
 */
static void pxa_irda_net_uninit(struct net_device *dev)
{
	struct pxa_irda *si = dev->priv;

	dev->hard_start_xmit	= NULL;
	dev->open		= NULL;
	dev->stop		= NULL;
	dev->do_ioctl		= NULL;
	dev->get_stats		= NULL;
	dev->priv		= NULL;

#ifdef CONFIG_PM
	pm_unregister(si->pmdev);
	printk(KERN_DEBUG "pxa_ir: pm unregistered\n");
#endif
	kfree(si->tx_buff.head);
	kfree(si->rx_buff.head);
	kfree(si);
	printk(KERN_INFO "pxa_ir: irda driver uninited\n");
}

#ifdef MODULE
static
#endif
int __init pxa_irda_init(void)
{
	struct net_device *dev;
	int err;

#ifdef DEBUG_LED_ON
	DISCRETE_LED_OFF(DEBUG_LED_RX);
	DISCRETE_LED_OFF(DEBUG_LED_TX);
#endif
 	

	err = request_mem_region(__PREG(STUART), 0x24, "IrDA") ? 0 : -EBUSY;
	if (err)
		goto err_mem_1;

	err = request_mem_region(__PREG(FICP), 0x1c, "IrDA") ? 0 : -EBUSY;
	if (err)
		goto err_mem_2;

	rtnl_lock();
	dev = dev_alloc("irda%d", &err);
	if (dev) {
		dev->init   = pxa_irda_net_init;
		dev->uninit = pxa_irda_net_uninit;

		err = register_netdevice(dev);
		if (err)
			kfree(dev);
		else
			netdev = dev;

#ifdef CONFIG_DPM
		if (netdev) {
			pxairda_ldm_register();
		}
#endif /* CONFIG_DPM */ 
	}
	rtnl_unlock();

	if (err) {
		release_mem_region(__PREG(FICP), 0x1c);
err_mem_2:
		release_mem_region(__PREG(STUART), 0x24);
	}
err_mem_1:
	return err;
}

static void __exit pxa_irda_exit(void)
{
	struct net_device *dev = netdev;

	netdev = NULL;
	if (dev) {
#ifdef CONFIG_DPM
		pxairda_ldm_unregister();
#endif /* CONFIG_DPM */
		rtnl_lock();
		unregister_netdevice(dev);
		rtnl_unlock();
	}

	release_mem_region(__PREG(STUART), 0x24);
	release_mem_region(__PREG(FICP), 0x1c);

#ifdef DEBUG_LED_ON
	DISCRETE_LED_OFF(DEBUG_LED_RX);
	DISCRETE_LED_OFF(DEBUG_LED_TX);
#endif

	/*
	 * We now know that the netdevice is no longer in use, and all
	 * references to our driver have been removed.  The only structure
	 * which may still be present is the netdevice, which will get
	 * cleaned up by net/core/dev.c
	 */
}

//#ifdef MODULE
module_init(pxa_irda_init);
module_exit(pxa_irda_exit);
//#endif

MODULE_AUTHOR("Hao Wu");
MODULE_DESCRIPTION("PXA250 SIR/FIR driver");
MODULE_LICENSE("GPL");
EXPORT_NO_SYMBOLS;
