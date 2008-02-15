/*
 *  linux/drivers/net/irda/mx21_ir.c
 *
 *  Copyright (C) 2000-2001 Jacky Zhao<r53709@motorola.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Infra-red driver for the DragonBall DBMX21
 *
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/slab.h>
#include <linux/rtnetlink.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/pm.h>

#include <net/irda/irda.h>
#include <net/irda/irmod.h>
#include <net/irda/wrapper.h>
#include <net/irda/irda_device.h>
#include "mx21_ir.h"

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <asm/arch/dma.h>
#include <asm/arch/gpio.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/arch/pll.h>

#define INT_SIR  INT_UART3

#define SIR_MODE 0x1
#define MIR_MODE 0x2
#define FIR_MODE 0x4

#define ENABLE	1
#define DISABLE 0

#define SIR_MAX_RXLEN		2047
#define UART_3 2
#define GPIOE 4

#define enable_sir_tx		ucr2_3_write(UCR2_3_TXEN,1)
#define enable_sir_rx		ucr2_3_write(UCR2_3_RXEN,1)
#define disable_sir_tx		ucr2_3_write(UCR2_3_TXEN,0)
#define disable_sir_rx		ucr2_3_write(UCR2_3_RXEN,0)

#define IRDA_TRANSMIT		1
#define IRDA_RECEIVE		2

#define SIR_Tx_FIFO_DEPTH	16
#define SIR_Rx_FIFO_DEPTH	1

//#define DBMX21_IRDA_DEBUG
#ifdef DBMX21_IRDA_DEBUG
#define DUMP_MSG(fmt, args...) pr_debug("%s: " fmt, __FUNCTION__ , ## args)
#else
#define DUMP_MSG(fmt, args...)
#endif

#ifndef GPIO_IRDA_POWER
#define GPIO_IRDA_POWER		(0)
#endif

#if 1 /*CEE LDM*/
#include <linux/device.h>

extern void mx21_ldm_bus_register(struct device *device,
                          struct device_driver *driver);
extern void mx21_ldm_bus_unregister(struct device *device,
                          struct device_driver *driver);
static int
__ldm_suspend(struct device *dev, u32 state, u32 level);

static int
__ldm_resume(struct device *dev, u32 level);

static struct device_driver __driver_ldm = {
	.name = "mx21_ir",
	.devclass = NULL,
	.probe = NULL,
	.suspend = __ldm_suspend,
	.resume = __ldm_resume,
	.scale = NULL,
	.remove = NULL,
};

static struct device __device_ldm = {
	.name = "IRDA",
	.bus_id = "irda",
	.driver = &__driver_ldm,
	.power_state = DPM_POWER_ON,
};

#endif


static int power_level = 3;

/*
 * Our netdevice.  There is only ever one of these.
 */
static struct net_device *netdev;

struct mx21_irda {
	void *irda_ctrl_pins;

	int sir_rx_interrupt;
	int sir_tx_interrupt;
	unsigned char direction;

	unsigned char power;
	unsigned char open;
	unsigned char mode;

	int speed;
	int newspeed;

	struct sk_buff *txskb;
	struct sk_buff *rxskb;
	dma_addr_t txbuf_dma;
	dma_addr_t rxbuf_dma;
	int txdma_channel;
	int rxdma_channel;

	struct net_device_stats stats;
	struct irlap_cb *irlap;
	struct pm_dev *pmdev;
	struct qos_info qos;

	iobuff_t tx_buff;
	iobuff_t rx_buff;
};

static struct mx21_irda *p_mx21_irda = 0;

#define IS_SIR(mi)		( (mi)->speed < 576000 )
#define IS_MIR(mi)		( (mi)->speed < 4000000 && (mi)->speed >= 576000 )
#define IS_FIR(mi)		( (mi)->speed >= 4000000 )

static void dbmx21_irda_setting(void);
static void dbmx21_gpio_set_sir(void);
static void dbmx21_transceiver_mode(struct mx21_irda *mi, U32 mode,
				    U32 irda_enable);
static void dbmx21_gpio_config(int mode);
static void dbmx21_sir_tx_interrupt(int enable);
static void dbmx21_sir_rx_interrupt(int enable);
static void dbmx21_sir_interrupt(int enable);

/*
 * Set the IrDA communications speed.
 */
static int
dbmx21_irda_set_speed(struct mx21_irda *mi, int speed)
{
	unsigned long flags, fcr, bir, rfdiv, perclk1_freq;
	int ret = -EINVAL;

	perclk1_freq = mx_module_get_clk(PERCLK1);

	fcr = UART_UFCR(UART_3);
    	
	switch (fcr & UFCR_RFDIV_MASK)
	{
		case UFCR_RFDIV_7:
			rfdiv = 7;
			break;
		case UFCR_RFDIV_6:
			rfdiv = 6;
			break;
		case UFCR_RFDIV_5:
			rfdiv = 5;
			break;
		case UFCR_RFDIV_4:
			rfdiv = 4;
			break;
		case UFCR_RFDIV_3:
			rfdiv = 3;
			break;
		case UFCR_RFDIV_2:
			rfdiv = 2;
			break;
		case UFCR_RFDIV_1:
			rfdiv = 1;
			break;
		default:
			printk(KERN_WARNING "Unknown Reference Freq Divider\n");
			rfdiv = 2;
	}

	/*
	 * i.MX21 Reference Manual defines equation for BRM setting
	 * 
	 *        16 * desired_baud_rate * RFDIV[2:0] * DENUM 
	 *  NUM = -------------------------------------------,
	 *                 PERCLK1
	 *
	 * where UBIR = NUM - 1 , DENUM - 1
	 *
	 *  Let's set DENUM to 10000
	 */
	 
	switch (speed) {
	case 9600:
	case 19200:
	case 38400:
	case 57600:
	case 115200:
		bir = (16 * speed * rfdiv) / (perclk1_freq / 10000) - 1;

		local_irq_save(flags);

		/* Disable Tx and Rx */
		disable_sir_tx;
		disable_sir_rx;
		dbmx21_sir_interrupt(DISABLE);

		/* Make sure GPIO ports is served as SIR. */
		dbmx21_transceiver_mode(mi, SIR_MODE, ENABLE);

		/*
		 * Software Reset.
		 *      1. Reset transmitter and receiver state machines
		 *      2. Clear status register and FIFOs
		 */
		ucr2_3_write(UCR2_3_SRST, 0);

		ubir_3_write(UBIR_3_INC, bir);
		ubmr_3_write(UBMR_3_MOD, 10000 - 1);

		mi->speed = speed;

		/* Before enable sir rx, clear the rx interrupt flags. */
		usr2_3_write(USR2_3_ORE, 1);	        /* rx overrun */
		usr1_3_write(USR1_3_FRAMERR, 1);	/* frame error */
		usr1_3_write(USR1_3_PARITYERR, 1);	/*parity error */

		dbmx21_sir_rx_interrupt(ENABLE);
		mi->direction = IRDA_TRANSMIT;
		enable_sir_rx;

		local_irq_restore(flags);
		ret = 0;
		break;

	default:
		break;
	}

	return ret;
}

/*
 * Control the power state of the IrDA transmitter.
 * State:
 *  0 - off
 *  1 - short range, lowest power
 *  2 - medium range, medium power
 *  3 - maximum range, high power
 *
 */
static inline int
dbmx21_irda_set_power(struct mx21_irda *si, unsigned int state)
{
	int ret;

	ret = 0;
	/*
	 * Operations on Power Setting.
	 * To Be Done.
	 */
	si->power = state;

	return ret;
}

/*
 * It is used to init the IRDA hardware function as SIR and start to work.
 */

static int
dbmx21_irda_startup(struct mx21_irda *mi)
{
	int ret;

	/* Configure to sir mode */
	dbmx21_transceiver_mode(mi, SIR_MODE, ENABLE);
	/*
	 * Need setting of SIR to enable SIR modulation to work around.
	 * Ensure that the port is disabled.
	 */

	DUMP_MSG("\n");
	ret = dbmx21_irda_set_speed(mi, mi->speed = 9600);
	if (ret)
		return ret;

	return 0;
}

static void
dbmx21_irda_shutdown(struct mx21_irda *mi)
{
	/* Disable Tx and Rx of the SIR */
	ucr2_3_write(UCR2_3_TXEN, 0);
	ucr2_3_write(UCR2_3_RXEN, 0);
}

#if 1 /*CEE LDM*/
/*
 * Suspend the IrDA interface.
 */
static int
dbmx21_irda_suspend(struct net_device *dev, int state)
{
	struct mx21_irda *mi = dev->priv;

	if (mi && mi->open) {
		/*
		 * Stop the transmit queue
		 */
		netif_device_detach(dev);
		disable_irq(INT_SIR);
		dbmx21_irda_shutdown(mi);
		dbmx21_irda_set_power(mi, 0);
	}

	return 0;
}

/*
 * Resume the IrDA interface.
 */
static int
dbmx21_irda_resume(struct net_device *dev)
{
	struct mx21_irda *mi = dev->priv;

	if (mi && mi->open) {
		/*
		 * If we missed a speed change, initialise at the new speed
		 * directly.  It is debatable whether this is actually
		 * required, but in the interests of continuing from where
		 * we left off it is desireable.  The converse argument is
		 * that we should re-negotiate at 9600 baud again.
		 */
		if (mi->newspeed) {
			mi->speed = mi->newspeed;
			mi->newspeed = 0;
		}

		dbmx21_irda_startup(mi);
		dbmx21_irda_set_power(mi, mi->power);
		enable_irq(INT_SIR);

		/*
		 * This automatically wakes up the queue
		 */
		netif_device_attach(dev);
	}

	return 0;
}
#endif
#ifdef CONFIG_PM
static int
dbmx21_irda_pmproc(struct pm_dev *dev, pm_request_t rqst, void *data)
{
	int ret;

	if (!dev->data)
		return -EINVAL;

	switch (rqst) {
	case PM_SUSPEND:
		ret = dbmx21_irda_suspend((struct net_device *) dev->data, 0);
		mx_module_clk_close(IPG_MODULE_UART3);
		break;

	case PM_RESUME:
		mx_module_clk_open(IPG_MODULE_UART3);
		ret = dbmx21_irda_resume((struct net_device *) dev->data);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
#endif

#if 1 /*CEE LDM*/
static int
__ldm_suspend(struct device *dev, u32 state, u32 level)
{
	int ret = 0;
	if (!dev->platform_data)
		return -EINVAL;

	switch (level) {
	case SUSPEND_POWER_DOWN:
		ret = dbmx21_irda_suspend((struct net_device *) dev->platform_data, 0);
		mx_module_clk_close(IPG_MODULE_UART3);
		break;
	}
	return ret;
}

static int
__ldm_resume(struct device *dev, u32 level)
{
	int ret = 0;
	if (!dev->platform_data)
		return -EINVAL;

	switch (level) {
	case RESUME_POWER_ON:
		mx_module_clk_open(IPG_MODULE_UART3);
		ret = dbmx21_irda_resume((struct net_device *) dev->platform_data);
		break;
	}
	return ret;
}
#endif


/*
 * SIR interrupt service routines.
 */
static void
dbmx21_sir_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	struct net_device *dev = dev_id;
	struct mx21_irda *mi = dev->priv;

	unsigned long parity_error, frame_error, overrun_error;
	unsigned long tx_fifo_empty;

	unsigned long error;

	unsigned long data, status;

	if (ucr2_3_read(UCR2_3_RXEN) && ucr2_3_read(UCR2_3_TXEN)) {
	}
	if (ucr2_3_read(UCR2_3_RXEN)) {
		parity_error = usr1_3_read(USR1_3_PARITYERR);
		frame_error = usr1_3_read(USR1_3_FRAMERR);
		overrun_error = usr2_3_read(USR2_3_ORE);

		/*
		 * Handling Receive.
		 *
		 * It is beleivable that the error occurs rarily. So the following
		 * code shows the optimization for error handling.
		 */
		error = parity_error | frame_error | overrun_error;
		while (error) {
			DUMP_MSG("rx error.\n");
			data = UART_URXD(UART_3);
			rmb();
			status = data & 0xD400;

			if (status & URXD_ERR) {	/* check the error bit  */
				mi->stats.rx_errors++;
				if (status & URXD_OVRRUN) {	/* overrun */
					mi->stats.rx_fifo_errors++;
					usr2_3_write(USR2_3_ORE, 1);
					printk(KERN_ERR "Rx overrun.\n");
					return;
				}
				if (status & URXD_PRERR) {	/* parity */
					usr1_3_write(USR1_3_PARITYERR, 1);
					printk(KERN_ERR "Rx parity error.\n");
					return;
				}
				if (status & URXD_FRMERR) {	/* frame error */
					mi->stats.rx_frame_errors++;
					usr1_3_write(USR1_3_FRAMERR, 1);
					printk(KERN_ERR "Rx frame error.\n");
					return;
				}

			} else {
				/* it is a correct data. */
				data &= 0xff;
				async_unwrap_char(dev, &mi->stats, &mi->rx_buff,
						  data);
			}
		}

		while (usr2_3_read(USR2_3_RDR)) {
			data = UART_URXD(UART_3);

			rmb();
			if (data & URXD_CHARRDY) {	/* Rx char ready */
				status = data & 0xf400;
				if (status & URXD_ERR) {	/* check the error bit */
					mi->stats.rx_errors++;
					if (status & URXD_OVRRUN) {	/* overrun */
						mi->stats.rx_fifo_errors++;
						usr2_3_write(USR2_3_ORE, 1);
						printk(KERN_ERR
						       "Rx overrun.\n");
					}
					if (status & URXD_FRMERR) {	/* frame error */
						mi->stats.rx_frame_errors++;
						usr1_3_write(USR1_3_FRAMERR, 1);
						printk(KERN_ERR
						       "Rx frame error.\n");
					}
					if (status & URXD_PRERR) {	/* parity */
						usr1_3_write(USR1_3_PARITYERR,
							     1);
						printk(KERN_ERR
						       "Rx parity error.\n");
					}
					/* Other: it is the Break char.
					 * Do nothing for it. throw out the data.
					 */
				} else {
					/* it is a correct data. */
					data &= 0xff;
					async_unwrap_char(dev, &mi->stats,
							  &mi->rx_buff, data);

					dev->last_rx = jiffies;
				}
			}
			rmb();
		}
	}

	/*
	 * Handle the interrupt from the transmitter
	 */
	if (ucr2_3_read(UCR2_3_TXEN)) {
		tx_fifo_empty = usr2_3_read(USR2_3_TXFE);
		if (tx_fifo_empty && mi->tx_buff.len) {
			DUMP_MSG("sir tx interrupt.\n");
			do {
				UART_UTXD(UART_3) = *mi->tx_buff.data++;
				mi->tx_buff.len -= 1;
				rmb();
			} while (usr1_3_read(USR1_3_TRDY) && mi->tx_buff.len);

			if (mi->tx_buff.len == 0) {
				mi->stats.tx_packets++;
				mi->stats.tx_bytes += mi->tx_buff.data -
				    mi->tx_buff.head;

				/*
				 * We need to wait for a while to make sure
				 * the transmitter has finished.
				 */
				do
					rmb();
				while (!usr2_3_read(USR2_3_TXDC));
				DUMP_MSG("sir tx interrupt@3.\n");

				/*
				 * Ok, we've finished transmitting.  Now enable
				 * the receiver.  Sometimes we get a receive IRQ
				 * immediately after a transmit...
				 *
				 * Before start Rx, we should disable the Tx interrupt
				 * because Tx FIFO empty will lead to another interrupt.
				 */
				dbmx21_sir_tx_interrupt(DISABLE);
				disable_sir_tx;

				/* Clear Rx status. */
				usr1_3_write(USR1_3_PARITYERR, 1);
				usr1_3_write(USR1_3_FRAMERR, 1);
				usr2_3_write(USR2_3_ORE, 1);

				/* Enable Rx interrupt */
				dbmx21_sir_rx_interrupt(ENABLE);

				/* Enable Tx and Rx. */
				disable_sir_tx;
				enable_sir_rx;

				if (mi->newspeed) {
					DUMP_MSG("\n");
					dbmx21_irda_set_speed(mi, mi->newspeed);
					mi->newspeed = 0;
				}
				netif_wake_queue(dev);
			}
		}

		if (mi->tx_buff.len == 0) {
			DUMP_MSG("disable sir tx interrupt.\n");
			dbmx21_sir_tx_interrupt(DISABLE);
		}

	}

}

static int
dbmx21_irda_hard_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct mx21_irda *mi = dev->priv;
	int speed = irda_get_next_speed(skb);

	DUMP_MSG(" function begin.\n");

	/*
	 * Does this packet contain a request to change the interface
	 * speed?  If so, remember it until we complete the transmission
	 * of this frame.
	 */
	if (speed != mi->speed && speed != -1)
		mi->newspeed = speed;

	/*
	 * If this is an empty frame, we can bypass a lot.
	 */
	if (skb->len == 0) {
		if (mi->newspeed) {
			mi->newspeed = 0;
			DUMP_MSG("\n");
			dbmx21_irda_set_speed(mi, speed);
		}
		dev_kfree_skb(skb);
		return 0;
	}

	if (IS_SIR(mi)) {
		DUMP_MSG("sir mode.\n");
		netif_stop_queue(dev);

		mi->tx_buff.data = mi->tx_buff.head;
		mi->tx_buff.len = async_wrap_skb(skb, mi->tx_buff.data,
						 mi->tx_buff.truesize);

		/*
		 * Set the transmit interrupt enable.  This will fire
		 * off an interrupt immediately.  Note that we disable
		 * the receiver so we won't get spurious characteres
		 * received.
		 */
		disable_sir_rx;
		dbmx21_sir_interrupt(DISABLE);
		mi->direction = IRDA_TRANSMIT;
		enable_sir_tx;
		dbmx21_sir_interrupt(ENABLE);

		dev->trans_start = jiffies;
		dev_kfree_skb(skb);
	} else {
	}

	DUMP_MSG(" function end.\n");

	return 0;
}

static int
dbmx21_irda_ioctl(struct net_device *dev, struct ifreq *ifreq, int cmd)
{
	struct if_irda_req *rq = (struct if_irda_req *) ifreq;
	struct mx21_irda *mi = dev->priv;
	int ret = -EOPNOTSUPP;

	DUMP_MSG(" function begin.\n");

	switch (cmd) {
	case SIOCSBANDWIDTH:
		DUMP_MSG("@1\n");
		if (capable(CAP_NET_ADMIN)) {
			DUMP_MSG("@2\n");
			/*
			 * We are unable to set the speed if the
			 * device is not running.
			 */
			if (mi->open) {
				ret =
				    dbmx21_irda_set_speed(mi, rq->ifr_baudrate);
			} else {
				printk
				    ("dbmx21_irda_ioctl: SIOCSBANDWIDTH: !netif_running\n");
				ret = 0;
			}
		}
		break;

	case SIOCSMEDIABUSY:
		DUMP_MSG("@3\n");
		ret = -EPERM;
		if (capable(CAP_NET_ADMIN)) {
			DUMP_MSG("@4\n");
			irda_device_set_media_busy(dev, TRUE);
			ret = 0;
		}
		break;

	case SIOCGRECEIVING:
		DUMP_MSG("@5\n");
		rq->ifr_receiving =
		    IS_SIR(mi) ? mi->rx_buff.state != OUTSIDE_FRAME : 0;
		DUMP_MSG("rq->ifr_receiving is %d",rq->ifr_receiving);
		break;

	default:
		DUMP_MSG("@6\n");
		break;
	}

	DUMP_MSG(" function end.\n");
	return ret;
}

static struct net_device_stats *
dbmx21_irda_stats(struct net_device *dev)
{
	struct mx21_irda *mi = dev->priv;
	return &mi->stats;
}

static int
dbmx21_irda_open(struct net_device *dev)
{
	struct mx21_irda *mi = dev->priv;
	int err;

	DUMP_MSG(" function begin.\n");

	MOD_INC_USE_COUNT;

	mi->speed = 9600;

	err = request_irq(INT_SIR, dbmx21_sir_irq_handler, 0, dev->name, dev);
	if (err)
		goto err_irq_sir;

	/*
	 * The interrupt must remain disabled for now.
	 */
	disable_irq(INT_SIR);

	/*
	 * Setup the serial port for the specified speed.
	 */
	err = dbmx21_irda_startup(mi);
	if (err)
		goto err_startup;

	/*
	 * Open a new IrLAP layer instance.
	 */
	mi->irlap = irlap_open(dev, &mi->qos, "dbmx21");
	err = -ENOMEM;
	if (!mi->irlap)
		goto err_irlap;

	/*
	 * Now enable the interrupt and start the queue
	 */
	mi->open = 1;
	dbmx21_irda_set_power(mi, power_level);	/* low power mode */

	/* Before enable sir tx, clear the tx interrupt flags. */
	/* Do nothing. */
#ifdef IOU
	enable_sir_tx;
#endif
	/* Before enable sir rx, clear the rx interrupt flags. */
	usr2_3_write(USR2_3_ORE, 1);            /* rx overrun */
	usr1_3_write(USR1_3_FRAMERR, 1);	/* frame error */
	usr1_3_write(USR1_3_PARITYERR, 1);	/* parity error */
	enable_sir_rx;

#ifdef IOU
	dbmx21_sir_interrupt(ENABLE);
#endif

	enable_irq(INT_SIR);
	netif_start_queue(dev);

	DUMP_MSG(" function end.\n");
	return 0;

      err_irlap:
	mi->open = 0;
	dbmx21_irda_shutdown(mi);
      err_startup:
	free_irq(INT_SIR, dev);
      err_irq_sir:
	MOD_DEC_USE_COUNT;
	return err;
}

static int
dbmx21_irda_stop(struct net_device *dev)
{
	struct mx21_irda *mi = dev->priv;

	DUMP_MSG(" function begin.\n");
	disable_irq(INT_SIR);
	dbmx21_irda_shutdown(mi);

	/*
	 * If we have been doing DMA receive, make sure we
	 * tidy that up cleanly.
	 */
	if (mi->rxskb) {
		dev_kfree_skb(mi->rxskb);
		mi->rxskb = NULL;
	}

	/* Stop IrLAP */
	if (mi->irlap) {
		irlap_close(mi->irlap);
		mi->irlap = NULL;
	}

	netif_stop_queue(dev);
	mi->open = 0;

	/*
	 * Free resources
	 */
	free_irq(INT_SIR, dev);

	dbmx21_irda_set_power(mi, 0);

	MOD_DEC_USE_COUNT;

	DUMP_MSG(" function end.\n");
	return 0;
}

static int
dbmx21_irda_init_iobuf(iobuff_t * io, int size)
{
	io->head = kmalloc(size, GFP_KERNEL | GFP_DMA);
	if (io->head != NULL) {
		io->truesize = size;
		io->in_frame = FALSE;
		io->state = OUTSIDE_FRAME;
		io->data = io->head;
	}
	return io->head ? 0 : -ENOMEM;
}

static int
dbmx21_irda_net_init(struct net_device *dev)
{
	struct mx21_irda *mi = dev->priv;
	unsigned int baudrate_mask;
	int err = -ENOMEM;

	DUMP_MSG("begin.\n");
	dbmx21_irda_setting();
	p_mx21_irda = mi = kmalloc(sizeof (struct mx21_irda), GFP_KERNEL);
	if (!mi)
		goto out;

	memset(mi, 0, sizeof (*mi));

	/*
	 * Initialise the SIR buffers
	 */
	err = dbmx21_irda_init_iobuf(&mi->rx_buff, 14384);
	if (err)
		goto out;
	err = dbmx21_irda_init_iobuf(&mi->tx_buff, 4000);
	if (err)
		goto out_free_rx;

	/*
	 * Iomap the fir control pins which is controlled by accessing
	 * the physical address 0xCC80_0000;
	 */
	mi->irda_ctrl_pins = ioremap(0xCC800000, 0x1000);

	dev->priv = mi;
	dev->hard_start_xmit = dbmx21_irda_hard_xmit;
	dev->open = dbmx21_irda_open;
	dev->stop = dbmx21_irda_stop;
	dev->do_ioctl = dbmx21_irda_ioctl;
	dev->get_stats = dbmx21_irda_stats;

	irda_device_setup(dev);
	irda_init_max_qos_capabilies(&mi->qos);

	/*
	 * IR_* Macro from net/irda/qos.h to define the capability of baudrate of IRDA
	 */

	/* SIR */
	baudrate_mask = IR_2400 | IR_9600 | IR_19200 | IR_38400 | IR_57600 | IR_115200;

	mi->qos.baud_rate.bits &= baudrate_mask;
	mi->qos.min_turn_time.bits = 7;

	irda_qos_bits_to_value(&mi->qos);

	/*
	 * Initially enable SIR.
	 */
	disable_sir_tx;
	disable_sir_rx;
	dbmx21_transceiver_mode(mi, SIR_MODE, ENABLE);

	/*
	 * Keep disableing transceive.
	 */
	disable_sir_tx;
	disable_sir_rx;

#ifdef CONFIG_PM
	/* Registration of Power-Management */
	mi->pmdev = pm_register(PM_SYS_DEV, PM_SYS_IRDA, dbmx21_irda_pmproc);
	if (mi->pmdev)
		mi->pmdev->data = dev;
#endif
#if 1 /*CEE LDM*/
	mx21_ldm_bus_register(&__device_ldm, &__driver_ldm);
	__device_ldm.platform_data = dev;
#endif
	DUMP_MSG("no error at net_init\n");
	return 0;

	kfree(mi->tx_buff.head);
      out_free_rx:
	kfree(mi->rx_buff.head);
      out:
	kfree(mi);

	return err;
}

/*
 * Remove all traces of this driver module from the kernel, so we can't be
 * called.  Note that the device has already been stopped, so we don't have
 * to worry about interrupts or dma.
 */
static void
dbmx21_irda_net_uninit(struct net_device *dev)
{
	struct mx21_irda *mi = dev->priv;

	DUMP_MSG("begin.\n");
	dev->hard_start_xmit = NULL;
	dev->open = NULL;
	dev->stop = NULL;
	dev->do_ioctl = NULL;
	dev->get_stats = NULL;
	dev->priv = NULL;

#ifdef CONFIG_PM
	pm_unregister(mi->pmdev);
#endif
#if 1 /*CEE LDM*/
	mx21_ldm_bus_unregister(&__device_ldm, &__driver_ldm);
#endif

	kfree(mi->tx_buff.head);
	kfree(mi->rx_buff.head);
	kfree(mi);
	mx_module_clk_close(IPG_MODULE_UART3);
	DUMP_MSG("End.\n");
}

/*
 * Interrupt En/Disable functions
 */
static void
dbmx21_sir_rx_interrupt(int enable)
{
	int bit = enable ? ENABLE : DISABLE;

	/* Enable Parity error interrupt. */
	/* Enable Frame error interrupt. */
	ucr3_3_write(UCR3_3_PARERREN, bit);
	ucr3_3_write(UCR3_3_FRAERREN, bit);

	/* Enable Receive Overrun and Data Ready interrupts. */
	ucr4_3_write(UCR4_3_OREN, bit);
	ucr4_3_write(UCR4_3_DREN, bit);
}

static void
dbmx21_sir_tx_interrupt(int enable)
{
	int bit = enable ? ENABLE : DISABLE;

	/* Enable Transmit Complete interrupt. */
	ucr4_3_write(UCR4_3_TCEN, bit);
}

static void
dbmx21_sir_interrupt(int enable)
{
	int bit = enable ? ENABLE : DISABLE;
	dbmx21_sir_tx_interrupt(bit);
	dbmx21_sir_rx_interrupt(bit);
}

/*
 * Clock setting
 */
static void
dbmx21_irda_setting(void)
{
	unsigned long bir, fcr, rfdiv, perclk1_freq;

	perclk1_freq = mx_module_get_clk(PERCLK1);

	/*
	 * SIR clock & registers setting
	 */

	/* Enable the clock gating to the uart3 module */
	mx_module_clk_open(IPG_MODULE_UART3);

	/* software reset */
	UART_UCR2(UART_3) = 0x61E6;
	UART_UCR1(UART_3) = 0;
	UART_UCR3(UART_3) = 0x4;
	UART_UCR4(UART_3) = 0x8000;

	/* Enable SIR, Disable DMA transfer in SIR. */
	UART_UCR1(UART_3) = 0x0081;

	/* 8 bit tx and rx,1 stop bit, disable parity */
	UART_UCR2(UART_3) = 0x6026;
	/* Then disable Tx,Rx. */
	disable_sir_tx;
	disable_sir_rx;

	/* INVT=0 */
	UART_UCR3(UART_3) = 0x4;
	/* INVR=1,IRSC=1 */
	UART_UCR4(UART_3) = 0x822A;

	/* INVT=1, INVR=0 */
	ucr3_3_write(UCR3_3_INVT, 1);
	ucr4_3_write(UCR4_3_INVR, 0);

	/* 
	 * Set the PERCLK1 divider and threshold of FIFOs 
	 * to trigger interrupt
	 */

	fcr = (SIR_Tx_FIFO_DEPTH << UART_TXTL_BIT) | UART_RFDIV_2 |
		(SIR_Rx_FIFO_DEPTH << UART_RXTL_BIT);
	
	UART_UFCR(UART_3) = fcr;
    	
	switch (fcr & UFCR_RFDIV_MASK)
	{
		case UFCR_RFDIV_7:
			rfdiv = 7;
			break;
		case UFCR_RFDIV_6:
			rfdiv = 6;
			break;
		case UFCR_RFDIV_5:
			rfdiv = 5;
			break;
		case UFCR_RFDIV_4:
			rfdiv = 4;
			break;
		case UFCR_RFDIV_3:
			rfdiv = 3;
			break;
		case UFCR_RFDIV_2:
			rfdiv = 2;
			break;
		case UFCR_RFDIV_1:
			rfdiv = 1;
			break;
		default:
			printk(KERN_WARNING "Unknown Reference Freq Divider\n");
			rfdiv = 2;
	}

	/* clear loopback bit */
	UART_UTS(UART_3) = 0x0000;

	/* 
	 * Configure BRM with default speed 9600bps 
	 * 
	 * i.MX21 Refernce Manual defines equation for BRM setting
	 * 
	 *        16 * desired_baud_rate * RFDIV[2:0] * DENUM 
	 *  NUM  = ------------------------------------------,
	 *                 PERCLK1
	 * , where UBIR = NUM - 1 , DENUM - 1
	 * 
	 *  Let's set DENUM to 10000, RFDIV to 2, baud rate is 9600
	 */

	bir = (16 * 9600 * rfdiv * 10000) / perclk1_freq - 1;

	UART_ONEMS(UART_3) = perclk1_freq / (rfdiv * 1000);
	UART_UBIR(UART_3) = bir;
	UART_UBMR(UART_3) = 10000 - 1;

	/*
	 * Interrupt configuration
	 */
	disable_irq(INT_SIR);
	dbmx21_sir_interrupt(DISABLE);

	/* By now, Tx and Rx are still disabled. */
}

/*
 * GPIO pins' configuration to SIR mode
 *
 * GPIO: PE8 -- UART3_TXD
 *		 PE9 -- UART3_RXD
 *
 * Multiplex SIR with FIR:
 *		 SIR -- Primay Function
 *
 * Default of GPIO: PE8 & PE9
 */
static void
dbmx21_gpio_set_sir(void)
{
	int retval;

	retval = mx2_register_gpios(PORT_E, (1 << 8) |
				    (1 << 9) |
				    (1 << 10) |
				    (1 << 11),
				    PRIMARY);
}

static void
dbmx21_transceiver_mode(struct mx21_irda *mi, U32 mode, U32 irda_enable)
{
	/* Control pins:
	 * D10 -- IRDA_EN_B
	 * D11 -- IRDA_FIR_SEL
	 * D12 -- IRDA_MD0
	 * D13 -- IRDA_MD1
	 */

	/* Set the IRDA_EN_B at bit 10 */
	if (irda_enable == ENABLE)
		*(u16 *) mi->irda_ctrl_pins |= 0x400;
	else
		*(u16 *) mi->irda_ctrl_pins &= ~0x400;

	/* Transceiver Enable at Full Dist Power with Mode1=0 and Mode0=0 */
	*(u16 *) mi->irda_ctrl_pins |= 0x3000;

	/* Set the Mode of the Transceiver */
	if (mode & (SIR_MODE)) {
		dbmx21_gpio_set_sir();
		*(u16 *) mi->irda_ctrl_pins &= ~0x800;
	}
	return;
}

static void
dbmx21_gpio_config(int mode)
{
	if (mode == SIR_MODE)
		dbmx21_gpio_set_sir();
}

int __init
dbmx21_irda_init(void)
{
	struct net_device *dev;
	int err;

	/*
	 * Limit power level a sensible range.
	 */
	if (power_level < 1)
		power_level = 1;
	if (power_level > 3)
		power_level = 3;

	err = 0;
	dev = 0;

	dbmx21_gpio_config(SIR_MODE);

	rtnl_lock();
	dev = dev_alloc("irda%d", &err);
	if (dev) {
		dev->init = dbmx21_irda_net_init;
		dev->uninit = dbmx21_irda_net_uninit;

		err = register_netdevice(dev);

		if (err)
			kfree(dev);
		else
			netdev = dev;
	}
	rtnl_unlock();

	DUMP_MSG("End with error =%d.\n", err);
	return err;
}

void __exit
dbmx21_irda_exit(void)
{
	struct net_device *dev = netdev;

	netdev = NULL;
	if (dev) {
		rtnl_lock();
		unregister_netdevice(dev);
		rtnl_unlock();
	}

}

module_init(dbmx21_irda_init);
module_exit(dbmx21_irda_exit);

MODULE_AUTHOR("Jacky Zhao <r53709@motorola.com>");
MODULE_DESCRIPTION("Motorola DBMX21 IrDA driver");
MODULE_LICENSE("GPL");
