/*
 * BRIEF MODULE DESCRIPTION
 *
 *	Infra-red driver for the OMAP730 Platform
 *          (SIR/MIR/FIR modes)
 *          (based on omap-sir.c)
 *
 * Copyright 2003 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *	   source@mvista.com
 *
 * Modified for the OMAP730 / Perseus2 board
 * Copyright 2004 MPC-Data Limited
 * Author: Dave Peverley
 *	   dpeverley at mpc-data.co.uk
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED	  ``AS	IS'' AND   ANY	EXPRESS OR IMPLIED
 *  WARRANTIES,	  INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO	EVENT  SHALL   THE AUTHOR  BE	 LIABLE FOR ANY	  DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED	  TO, PROCUREMENT OF  SUBSTITUTE GOODS	OR SERVICES; LOSS OF
 *  USE, DATA,	OR PROFITS; OR	BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN	 CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
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
#include <linux/ioport.h>

#include <net/irda/irda.h>
#include <net/irda/irmod.h>
#include <net/irda/wrapper.h>
#include <net/irda/irda_device.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/dma.h>

#undef CONFIG_CEE
#define CONFIG_CEE

/*
 * Our netdevice.  There is only ever one of these.
 */
 
static struct net_device *netdev; /* Our net device */
static int rx_state = 0;          /* RX state for IOCTL */
struct omap730_irda {
	
	unsigned char		open;

        int			speed;  /* Current IrDA speed */
	int			newspeed;

	struct net_device_stats	stats;
	struct irlap_cb		*irlap;
	struct qos_info		qos;

        dma_regs_t              *rx_dma_regs;  /* RX DMA registers */
        dma_regs_t              *tx_dma_regs;  /* TX DMA registers */

        dma_addr_t              rx_buf_dma_phys; /* Physical adress of RX DMA buffer */
        dma_addr_t              tx_buf_dma_phys; /* Physical adress of TX DMA buffer */

        void *                  rx_buf_dma_virt; /* Virtual adress of RX DMA buffer */
        void *                  tx_buf_dma_virt; /* Virtual adress of TX DMA buffer */

};

/* Linux Driver Model Structures */

#ifdef CONFIG_CEE  /* MVL-CEE  */
#include <linux/device.h>

static int omap730_irda_suspend(struct device * dev, u32 state, u32 level);
static int omap730_irda_resume(struct device * dev, u32 level);

static struct device_driver irda_driver_ldm = {
       name:      "omap730-IrDA",
       devclass:  NULL,
       probe:     NULL,
       suspend:   omap730_irda_suspend,
       resume:    omap730_irda_resume,
       remove:    NULL,
};

static struct device irda_device_ldm = {
       name: "OMAP730 IrDA",
       bus_id: "IrDA",
       driver: NULL,
       power_state: DPM_POWER_ON,
};

static void irda_ldm_driver_register(void)
{
   extern void mpu_public_driver_register(struct device_driver *driver);

   mpu_public_driver_register(&irda_driver_ldm);
}

static void irda_ldm_device_register(void)
{
   extern void mpu_public_device_register(struct device *device);

   mpu_public_device_register(&irda_device_ldm);
}

static void irda_ldm_driver_unregister(void)
{
   extern void mpu_public_driver_unregister(struct device_driver *driver);

   mpu_public_driver_unregister(&irda_driver_ldm);
}

static void irda_ldm_device_unregister(void)
{
   extern void mpu_public_device_unregister(struct device *device);

   mpu_public_device_unregister(&irda_device_ldm);
}
#endif /* MVL-CEE */

/*
 * If you want to disable debug information
 * please uncomment line bellow
 */


/* #define OMAP730_IR_DEBUG_ENABLE */

#ifdef OMAP730_IR_DEBUG_ENABLE
#define __ECHO_IN printk(KERN_ERR "%s: enter\n",__FUNCTION__);
#define __ECHO_OUT printk(KERN_ERR "%s: exit\n",__FUNCTION__);
#define DBG(args...) printk(KERN_ERR __FUNCTION__"():"args);
#else
#define __ECHO_IN
#define __ECHO_OUT
#define DBG(args...)
#endif

#define DBG_IRQ(args...) printk(KERN_ERR __FUNCTION__"():"args);

#if defined(OMAP730_IR_HARDWARE_DEBUG_ENABLE)


#define HDBG_DELAY 200
void hard_debug1(u16 i)
{
	for(;i;i--)
	{
#if defined(CONFIG_OMAP_H2)
	        outw(0x2000,GPIO_CLEAR_DATAOUT_REG);
		udelay(HDBG_DELAY);
	        outw(0x2000,GPIO_SET_DATAOUT_REG);
		udelay(HDBG_DELAY);
#elif defined(CONFIG_MACH_OMAP_PERSEUS2)
		/* The H2 manipulates GPIO_13 - this doesnt seem to 
		 * be connected :-/
		 */
#endif
	}
}

void hard_debug2(u16 i)
{
	for(;i;i--)
	{
#if defined(CONFIG_OMAP_H2)
	        outw(0x8000,GPIO_CLEAR_DATAOUT_REG);
		udelay(HDBG_DELAY);
	        outw(0x8000,GPIO_SET_DATAOUT_REG);
		udelay(HDBG_DELAY);
#elif defined(CONFIG_MACH_OMAP_PERSEUS2)
		/* The H2 manipulates GPIO_15 - this doesnt seem to 
		 * be connected :-/
		 */
#endif
	}
}
#define HDBG1(i) hard_debug1(i)
#define HDBG2(i) hard_debug2(i)
#else
#define HDBG1(i)
#define HDBG2(i)
#endif
/*  #undef DBG_IRQ */
/*  #define DBG_IRQ(args...)  */

/* forward declarations */

static int omap730_irda_set_speed(struct net_device *dev,int speed);

static void omap730_irda_start_rx_dma(struct omap730_irda *si)
{
   dma_regs_t *regs;

   regs = si->rx_dma_regs; 

         /* Configure DMA */
              
   regs->csdp = (1 << 3) | (1 << 2); 
   regs->ccr |= (1 << 10) | (1 << 14);
   regs->cicr = (1 << 3) | (1 << 1) | 1 ;
   regs->cssa_l = UART3_RHR & 0xffff;
   regs->cssa_u = UART3_RHR >> 16;
   regs->cdsa_l =  si->rx_buf_dma_phys & 0xffff;
   regs->cdsa_u =  si->rx_buf_dma_phys >> 16;
   regs->cen = 4096;          /* we can get 4096 bytes only  */
   regs->cfn = 1;  /* Maximum frames in RX buffer */
   regs->ccr2 = 0;
   regs->lch_ctrl = 2;
 
	/* Start DMA */

   regs->ccr |= (1 << 7); 

}

static void omap730_start_tx_dma(struct omap730_irda *si, int size)
{
   dma_regs_t *regs;

   __ECHO_IN;

   regs = si->tx_dma_regs; 

         /* Configure DMA */
              
   regs->csdp = (1 << 10) | (1 << 9); 
   regs->ccr |= (1 << 10) | (1 << 12);
   regs->cicr = (1 << 3) | (1 << 1) | 1 ;
   regs->cdsa_l = UART3_THR & 0xffff;
   regs->cdsa_u = UART3_THR >> 16;
   regs->cssa_l =  si->tx_buf_dma_phys & 0xffff;
   regs->cssa_u =  si->tx_buf_dma_phys >> 16;
   regs->cen = size;  /* Block length for DMA transfer */
   regs->cfn = 1 ;
   regs->ccr2 = 0;  
   regs->lch_ctrl = 2;

	/* Start DMA */

   //   printk("Trace 1 \n");
   HDBG1(1);

   regs->ccr |= (1 << 7); 

   HDBG1(1);
   //  printk("Trace 2 \n");


   __ECHO_OUT;
}

/* DMA RX callback - normally, we should not go here, 
   it calls only if something is going wrong
 */

static void omap730_irda_rx_dma_callback(void *data)
{
   struct net_device *dev = data;
   struct omap730_irda *si = dev->priv;
   
   printk("RX Transfer error or very big frame \n");

   /* Clear interrupts */

   inb(UART3_IIR);

   si->stats.rx_frame_errors++;

   inb(UART3_RESUME);

   /* Re-init RX DMA */

   omap730_irda_start_rx_dma(si);  
}


/* DMA TX callback - calling when frame transfer has been finished */

static void omap730_irda_tx_dma_callback(void *data)
{
  struct net_device *dev = data;
  struct omap730_irda *si = dev->priv;
  dma_regs_t *regs = si->tx_dma_regs;

   __ECHO_IN;   

   /*Stop DMA controller */

   regs->ccr &= ~(1 << 7);

   __ECHO_OUT;
}

/*
 * Set the IrDA communications speed.
 * Interrupt have to be disabled here.
 */

static int omap730_irda_startup(struct net_device *dev)
{

   __ECHO_IN;
         
#if defined(CONFIG_OMAP_H2)

	/* Set CONF_MOD_UART3_IRDA_MODE_R - Depreciated in 1610! */
	outl(inl(MOD_CONF_CTRL_0) | (1 << 15), MOD_CONF_CTRL_0);

	/* Set CONF_MOD_UART3_CLK_MODE_R to active request. */
	outl(inl(MOD_CONF_CTRL_0) | (1 << 31), MOD_CONF_CTRL_0);

	/* MUX Y15 to 0x7 -> GPIO17 (or ! UART3.RTS) */
	outl(inl(FUNC_MUX_CTRL_A) | 7, FUNC_MUX_CTRL_A);

	/* ???? H2 doesn't use GPIO_33 (Reg2, bit 1) so whats this about? */
	outw(inw(GPIO2_DIRECTION_REG)&(~2),GPIO2_DIRECTION_REG);
	outw(2,GPIO2_CLEAR_DATAOUT_REG);
#elif defined (CONFIG_MACH_OMAP_PERSEUS2)
	/* Set SD_MODE bit - force FIR_SEL low */ 
	outb(UART3_ACREG_SD_MOD, UART3_ACREG);
#endif
   
	outb(MDR1_MODE_SELECT_DISABLE, UART3_MDR1);         /* Put UART3 in reset mode*/

	/* Clear DLH and DLL */

	outb(UART3_LCR_DIV_EN, UART3_LCR);

	outb(0, UART3_DLL);
	outb(0, UART3_DLH);

	/* LCR must be set to 0xBF for access to EFR. */
	outb(0xBF, UART3_LCR);

	/* Enable access to UART3_TLR and UART3_TCR registers 
	 * (When EFR[4] == 1 && MCR[6] == 1 
	 */
	outb((1 << 4), UART3_EFR);

	/* Set LCR back again */
	outb(UART3_LCR_DIV_EN, UART3_LCR);

	outb((1 << 6), UART3_MCR);

	outb(0, UART3_SCR);

	/* Set Rx trigger to 1 and Tx trigger to 1 */ 

	outb(0, UART3_TLR);

	/* Set LCR to 8 bits and 1 stop bit */

	outb(UART3_LCR_CHAR_LENGTH_8_BITS, UART3_LCR);

	/* Clear RX and TX FIFO, 
	   and enable FIFO */
	/* Use DMA Req for transfers */    

	outb(UART3_FCR_RX_FIFO_CLEAR | UART3_FCR_TX_FIFO_CLEAR |
		UART3_FCR_DMA_MODE | UART3_FCR_FIFO_EN | 
		(UART3_FCR_FIFO_SPACES_8 << UART3_FCR_RX_FIFO_TRIG_POS) |
		(UART3_FCR_FIFO_SPACES_8 << UART3_FCR_TX_FIFO_TRIG_POS),
		UART3_FCR);

	outb(0, UART3_MCR);

	outb(UART3_SCR_TX_TRIG_GRANU1 | UART3_SCR_RX_TRIG_GRANU1, UART3_SCR);

	/* Enable UART3 SIR Mode, 
	   (Frame-length method to end frames) */

	outb(MDR1_MODE_SELECT_DISABLE, UART3_MDR1); 


	/* Set Status FIFO trig to 1 */

	outb(0, UART3_MDR2);

	/* Enables RXIR input */
	/* and disable TX underrun */
	/* SEND_SIP pulse */

	outb( inb(UART3_ACREG) | UART3_ACREG_DIS_TX_UNDERRUN , UART3_ACREG);

	/* Enable EOF Interrupt only */

	outb(UART3_IER_TX_STATUS_IT | UART3_IER_EOF_IT,  UART3_IER);

	/* Set Maximum Received Frame size to 2048 bytes*/

	outb(0x00, UART3_RXFLL);
	outb(0x08, UART3_RXFLH);

	inb(UART3_RESUME);

	__ECHO_OUT;

	return 0;
	
}

static int omap730_irda_shutdown(struct omap730_irda *si)
{
   
	/* Disable all UART3 Interrupts */

	outb(0, UART3_IER);

	/* Disable UART3 and disable baud rate generator */

	outb(MDR1_MODE_SELECT_DISABLE, UART3_MDR1); /* Put UART3 in reset mode*/

	/* set SD_MODE pin to high and Disable RX IR */ 
	outb(UART3_ACREG_DIS_IR_RX, UART3_ACREG);       

	/* Clear DLH and DLL */

	outb(UART3_LCR_DIV_EN, UART3_LCR);
	outb(0, UART3_DLL);
	outb(0, UART3_DLH);  

	return 0;

}

#ifdef CONFIG_CEE  /* MVL-CEE */

/*
 * Suspend the IrDA interface.
 */

static int omap730_irda_suspend(struct device *dev, u32 state, u32 level)
{
	struct omap730_irda *si = netdev->priv;

	/*
	 * Stop the transmit queue
	 */

        switch(level)
        {
           case SUSPEND_POWER_DOWN: 

       /* Turn off power */
       
	        netif_device_detach(netdev);
	        disable_irq(netdev->irq);
	        omap730_irda_shutdown(si);
        }

	return 0;
}

/*
 * Resume the IrDA interface.
 */

static int omap730_irda_resume(struct device *dev, u32 level)
{
	struct omap730_irda *si = netdev->priv;

        switch(level)
	{
          case RESUME_POWER_ON:
     
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
                else
		{
		   si->speed = 9600;
		}    

	        omap730_irda_startup(netdev);
		omap730_irda_set_speed(netdev, si->speed);
                netif_device_attach(netdev);
                enable_irq(netdev->irq);

                break;
	}


	return 0;
}
#endif	/* MVL-CEE */

static void omap730_irda_irq(int irq, void *dev_id, struct pt_regs *hw_regs)
{
	struct net_device *dev = dev_id;
	struct omap730_irda *si =  dev->priv;
	dma_regs_t *regs = si->rx_dma_regs;
	struct sk_buff *skb;

	u8 status;

	__ECHO_IN;

	/* Clear EOF interrupt */

	status = inb(UART3_IIR);
	if(status & UART3_IIR_TX_STATUS_IT)
	{
		u8 mdr2 = inb(UART3_MDR2);
		HDBG1(2);
		if (mdr2 & UART3_MDR2_IRTX_UNDERRUN) 
			printk(KERN_ERR "IRDA Buffer underrun error");

		si->stats.tx_packets++;  

		if (si->newspeed)
		{
			omap730_irda_set_speed(dev, si->newspeed);
			si->newspeed = 0;
		}

		netif_wake_queue(dev);

		if(!(status & UART3_IIR_EOF_IT)) return;
	}
	/* Stop DMA and if there are no errors,
	   send frame to upper layer */

	regs->ccr &= ~(1 << 7);

	status = inb(UART3_SFLSR); /* Take a frame status */

	if(status != 0)               /* Bad frame? */
	{
		/* If we do not read the status register again the status FIFO 
		 * read pointer will be referencing invalid data on the 
		 * subsequent interrupt, which will cause the irda module to 
		 * stop receiving.
		 */
		inb(UART3_SFLSR);
		si->stats.rx_frame_errors++;
		inb(UART3_RESUME);
	}
	else
	{        
		/* We got a frame! */
		skb = alloc_skb(4096, GFP_ATOMIC);
		if (!skb) {
			printk(KERN_ERR "omap_sir: out of memory for RX SKB\n");
			return;
		}
		/*
		 * Align any IP headers that may be contained
		 * within the frame.
		 */

		skb_reserve(skb, 1);           

		if (si->speed != 4000000)
		{
			memcpy(skb_put(skb,  regs->cdac - regs->cdsa_l - 2), 
				si->rx_buf_dma_virt,
				regs->cdac - regs->cdsa_l - 2); /* Copy DMA buffer to skb */
		}
		else
		{
			memcpy(skb_put(skb,  regs->cdac - regs->cdsa_l - 4), 
				si->rx_buf_dma_virt,
				regs->cdac - regs->cdsa_l - 4); /* Copy DMA buffer to skb */
		}
		skb->dev = dev;
		skb->mac.raw = skb->data;
		skb->protocol = htons(ETH_P_IRDA);
		si->stats.rx_packets++;
		si->stats.rx_bytes += skb->len;
		netif_rx(skb);            /* Send data to upper level */
	}

	/* Re-init RX DMA */

	omap730_irda_start_rx_dma(si);  

	dev->last_rx = jiffies; 

	__ECHO_OUT;
}


static int omap730_irda_hard_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct omap730_irda *si = dev->priv;
	int speed = irda_get_next_speed(skb);
        int mtt = irda_get_mtt(skb);
        int xbofs = irda_get_next_xbofs(skb);

        __ECHO_IN;

	/*
	 * Does this packet contain a request to change the interface
	 * speed?  If so, remember it until we complete the transmission
	 * of this frame.
	 */
	if (speed != si->speed && speed != -1)
		si->newspeed = speed;

        if(xbofs)
        {
           /* Set number of addtional BOFS */
	  outb(xbofs+1, UART3_EBLR);  
        }

	/*
	 * If this is an empty frame, we can bypass a lot.
	 */
	if (skb->len == 0) {
		if (si->newspeed) {
			si->newspeed = 0;
			omap730_irda_set_speed(dev, speed);
		}
		dev_kfree_skb(skb);
		return 0;
	}

       netif_stop_queue(dev);

       /* Copy skb data to DMA buffer */

       memcpy(si->tx_buf_dma_virt, skb->data, skb->len);
   
       si->stats.tx_bytes += skb->len;

       /* Set frame length */

       outb((skb->len & 0xff), UART3_TXFLL);
       outb((skb->len >> 8), UART3_TXFLH);

       if(mtt > 1000)
         mdelay(mtt / 1000);
       else
         udelay(mtt);

       /* Start TX DMA transfer*/

       omap730_start_tx_dma(si, skb->len);

       /* We can free skb now because it's already in DMA buffer */

       dev_kfree_skb(skb);

       dev->trans_start = jiffies;

       __ECHO_OUT;

       return 0;
}

static int
omap730_irda_ioctl(struct net_device *dev, struct ifreq *ifreq, int cmd)
{
	struct if_irda_req *rq = (struct if_irda_req *)ifreq;
	struct omap730_irda *si = dev->priv;
	int ret = -EOPNOTSUPP;

	__ECHO_IN;
	
	switch (cmd) {
	case SIOCSBANDWIDTH:
		if (capable(CAP_NET_ADMIN)) {
			/*
			 * We are unable to set the speed if the
			 * device is not running.
			 */
			if (si->open) {
				ret = omap730_irda_set_speed(dev,
						rq->ifr_baudrate);
			} else {
				printk("omap_irda_ioctl: SIOCSBANDWIDTH: !netif_running\n");
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
	  rq->ifr_receiving = rx_state;
		break;

	default:
		break;
	}

	__ECHO_OUT;

	return ret;
}

static struct net_device_stats *omap730_irda_stats(struct net_device *dev)
{
	struct omap730_irda *si = dev->priv;
	return &si->stats;
}

static int omap730_irda_start(struct net_device *dev)
{
	struct omap730_irda *si = dev->priv;
	int err;
	unsigned int flags;

	MOD_INC_USE_COUNT;

	__ECHO_IN;
	si->speed = 9600;

	save_flags(flags);
	cli();
		
	err = request_irq(dev->irq, omap730_irda_irq, 0, dev->name, dev);
	if (err)
		goto err_irq;


	/*
	 * The interrupt must remain disabled for now.
	 */
	
	disable_irq(dev->irq);

	restore_flags(flags);

	/*  Request DMA channels for IrDA hardware*/

	if(omap_request_dma(eUART2Rx, "IrDA Rx DMA", 
		(dma_callback_t)omap730_irda_rx_dma_callback, 
		dev, &(si->rx_dma_regs)))
	{
		printk(KERN_ERR "Failed to request IrDA Rx DMA \n");
		goto err_irq;
	}

	if(omap_request_dma(eUART2Tx, "IrDA Tx DMA", 
		(dma_callback_t)omap730_irda_tx_dma_callback, 
		dev, &(si->tx_dma_regs)))
	{
		printk(KERN_ERR "Failed to request IrDA Tx DMA \n");
		goto err_irq;
	}

	/* Allocate TX and RX buffers for DMA channels */

	si->rx_buf_dma_virt = consistent_alloc(GFP_KERNEL | GFP_DMA | GFP_ATOMIC,
						4096, &(si->rx_buf_dma_phys));

	si->tx_buf_dma_virt = consistent_alloc(GFP_KERNEL | GFP_DMA | GFP_ATOMIC,
						4096, &(si->tx_buf_dma_phys));

	/*
	 * Setup the serial port for the specified config.
	 */

	err = omap730_irda_startup(dev);
	if (err)
		goto err_startup;

	omap730_irda_set_speed(dev,si->speed = 9600);

	/*
	 * Open a new IrLAP layer instance.
	 */

	si->irlap = irlap_open(dev, &si->qos, "omap_sir");
	err = -ENOMEM;
	if (!si->irlap)
		goto err_irlap;

	/*
	 * Now enable the interrupt and start the queue
	 */
	si->open = 1;

	/* Start RX DMA */

	omap730_irda_start_rx_dma(si);  

	enable_irq(dev->irq);
	netif_start_queue(dev);

	__ECHO_OUT;

	return 0;

err_irlap:
	si->open = 0;
	omap730_irda_shutdown(si);
err_startup:
err_irq:
	free_irq(dev->irq, dev);
	MOD_DEC_USE_COUNT;
	return err;
}

static int omap730_irda_stop(struct net_device *dev)
{
	struct omap730_irda *si = dev->priv;

	__ECHO_IN;	

	disable_irq(dev->irq);

	netif_stop_queue(dev);

	omap_free_dma(si->rx_dma_regs);
	omap_free_dma(si->tx_dma_regs);

	consistent_free(si->rx_buf_dma_virt, 4096, si->rx_buf_dma_phys);
	consistent_free(si->tx_buf_dma_virt, 4096, si->tx_buf_dma_phys);

	omap730_irda_shutdown(si); 

	/* Stop IrLAP */
	if (si->irlap) {
		irlap_close(si->irlap);
		si->irlap = NULL;
	}

	si->open = 0;

	/*
	 * Free resources
	 */

	free_irq(dev->irq, dev);

	MOD_DEC_USE_COUNT;

	__ECHO_OUT;
 
	return 0;
}

static int omap730_irda_set_speed(struct net_device *dev, int speed)
{
	struct omap730_irda *si = dev->priv;
	int divisor;

	__ECHO_IN;

	local_irq_disable();   

	DBG("OMAP IRDA set speed: %d \n",speed);

	/* Set IrDA speed */
	if(speed <= 115200)
	{
		/* SIR mode */

#ifdef CONFIG_OMAP_H2
		outw(2,GPIO2_CLEAR_DATAOUT_REG);
#elif defined (CONFIG_MACH_OMAP_PERSEUS2)
		/* Set SD_MODE bit - force FIR_SEL low */ 
		outb((inb(UART3_ACREG) | UART3_ACREG_SD_MOD), UART3_ACREG);
#endif

		outb(MDR1_MODE_SELECT_SIR, UART3_MDR1);    /* Set SIR mode */

		outb(1, UART3_EBLR);

		divisor = 48000000 / (16 * speed); /* Base clock 48 MHz */

		HDBG2(1);
		outb(UART3_LCR_DIV_EN, UART3_LCR);      

		outb((divisor & 0xFF), UART3_DLL);

		outb((divisor >> 8 ), UART3_DLH);  

		outb(UART3_LCR_CHAR_LENGTH_8_BITS, UART3_LCR);  

		outb(0, UART3_MCR);

		HDBG2(1);

	}
	else if(speed <= 1152000)
	{
		/* MIR mode */

		/* Set MIR mode with SIP after each frame*/
		outb(MDR1_MODE_SELECT_MIR | UART3_MDR1_SIP_MODE, UART3_MDR1); 

		outb(2, UART3_EBLR);

		divisor = 48000000 / (41 * speed); /* Base clock 48 MHz */

		outb(UART3_LCR_DIV_EN, UART3_LCR); 

		outb((divisor & 0xFF), UART3_DLL);

		outb((divisor >> 8 ), UART3_DLH);    

		outb(UART3_LCR_CHAR_LENGTH_8_BITS, UART3_LCR); 

#ifdef CONFIG_OMAP_H2
		/* switch transmitter mode */
		outw(2,GPIO2_SET_DATAOUT_REG); 
#elif defined (CONFIG_MACH_OMAP_PERSEUS2)

		/* Clear SD_MODE bit - force FIR_SEL high */ 
		outb((inb(UART3_ACREG) & ~(UART3_ACREG_SD_MOD)), UART3_ACREG);
#endif
	}
	else
	{
		/* FIR mode */

		/* Set FIR mode with SIP after each frame*/
		outb(MDR1_MODE_SELECT_FIR | UART3_MDR1_SIP_MODE, UART3_MDR1); 

#ifdef CONFIG_OMAP_H2
		outw(2,GPIO2_SET_DATAOUT_REG); //switch transmitter mode
#elif defined (CONFIG_MACH_OMAP_PERSEUS2)
		/* Clear SD_MODE bit - force FIR_SEL high */ 
		outb((inb(UART3_ACREG) & ~(UART3_ACREG_SD_MOD)), UART3_ACREG);
#endif
	}

	si->speed = speed;

	local_irq_enable();

	__ECHO_OUT;

	return 0;
}


static int omap730_irda_net_init(struct net_device *dev)
{
	struct omap730_irda *si = dev->priv;
	unsigned int baudrate_mask;
	int err = -ENOMEM;

	si = kmalloc(sizeof(struct omap730_irda), GFP_KERNEL);
	if (!si)
		goto out;

	memset(si, 0, sizeof(*si));

	/*
	 * Initialize structures
	 */

	dev->priv = si;
	dev->hard_start_xmit	= omap730_irda_hard_xmit;
	dev->open		= omap730_irda_start;
	dev->stop		= omap730_irda_stop;
	dev->do_ioctl		= omap730_irda_ioctl;
	dev->get_stats		= omap730_irda_stats;

	irda_device_setup(dev);
	irda_init_max_qos_capabilies(&si->qos);

	/*
	 *  OMAP730  supports SIR, MIR, FIR modes,
	 *  but actualy supported modes depend on hardware implementation.
	 *  OMAP730 Innovator supports only SIR and 
	 *  OMAP730 H2 supports both SIR and FIR
	 */

	baudrate_mask = IR_9600 | IR_19200 | IR_38400 | IR_57600 | IR_115200;
	
#ifdef CONFIG_MACH_OMAP_PERSEUS2
        baudrate_mask |= (IR_4000000 << 8);
#endif	
  	si->qos.baud_rate.bits &= baudrate_mask;
	si->qos.min_turn_time.bits = 7;

	irda_qos_bits_to_value(&si->qos);

	return 0;
out:
	kfree(si);

	return err;
}

/*
 * Remove all traces of this driver module from the kernel, so we can't be
 * called.  Note that the device has already been stopped, so we don't have
 * to worry about interrupts or dma.
 */

static void omap730_irda_net_uninit(struct net_device *dev)
{
	struct omap730_irda *si = dev->priv;

	dev->hard_start_xmit	= NULL;
	dev->open		= NULL;
	dev->stop		= NULL;
	dev->do_ioctl		= NULL;
	dev->get_stats		= NULL;
	dev->priv		= NULL;

	kfree(si);
}

#ifdef MODULE
static
#endif
int __init omap730_irda_init(void)
{
	struct net_device *dev;
	int err;

	/* OMAP730 IR  net device register stage */

        rtnl_lock();
	dev = dev_alloc("irda%d", &err);
	if (dev) {
	        dev->irq    = INT_UART2;
		dev->init   = omap730_irda_net_init;
		dev->uninit = omap730_irda_net_uninit;

		err = register_netdevice(dev);

		if (err)
			kfree(dev);
		else
		{
			netdev = dev;
#ifdef CONFIG_CEE  /* MVL-CEE */
                        irda_ldm_device_register();
                        irda_ldm_driver_register();
#endif /* MVL-CEE */                        
                } 
	}

	rtnl_unlock();
	
	return err;
}

static void __exit omap730_irda_exit(void)
{
	struct net_device *dev = netdev;

#ifdef CONFIG_CEE  /* MVL-CEE */
        irda_ldm_device_unregister();
        irda_ldm_driver_unregister();
#endif /* MVL-CEE */ 

	netdev = NULL;
	if (dev) {
		rtnl_lock();
		unregister_netdevice(dev);
		rtnl_unlock();
	}

	/*
	 * We now know that the netdevice is no longer in use, and all
	 * references to our driver have been removed.  The only structure
	 * which may still be present is the netdevice, which will get
	 * cleaned up by net/core/dev.c
	 */
}

#ifdef MODULE
module_init(omap730_irda_init);
module_exit(omap730_irda_exit);
#endif

MODULE_AUTHOR("MontaVista");
MODULE_DESCRIPTION("OMAP730 IR");
MODULE_LICENSE("GPL");
EXPORT_NO_SYMBOLS;
