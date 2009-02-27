/*
 *  linux/drivers/net/irda/pxa_ir.c
 *
 *  Author:
 *  Alexey Lugovskoy RTSoft. 
 *	lugovskoy@rtsoft.msk.ru
 *
 *  Dmitrij Frasenyak RTSoft. 
 *      sed@mipt.sw.ru
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Infra-red SIR and FIR driver for the PXA 210/250 embedded microprocessors
 *  Based on linux/drivers/net/irda/sa1100_ir.c
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
#include <linux/ioport.h>
#include <linux/delay.h>

#include <linux/pm.h>

#include <net/irda/irda.h>
#include <net/irda/irmod.h>
#include <net/irda/wrapper.h>
#include <net/irda/irda_device.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/arch/lubbock.h>


static int rx_count = 0;
static int tx_count = 0;

/*
 * Our netdevice.  There is only ever one of these.
 */
 
static struct net_device *netdev;

struct pxa250_irda {
	
	unsigned char		open;

	int			speed;
	int			newspeed;

	struct sk_buff		*txskb;
	struct sk_buff		*rxskb;
	

	/* => FIR */
	unsigned int		fir_irq;
	int			txdma_ch;
	int			rxdma_ch;
	dma_addr_t		txbuf_dma;
	dma_addr_t		rxbuf_dma;
	void* 			txbuf_dma_virt;
	void* 			rxbuf_dma_virt;
	/* <= FIR*/
	struct net_device_stats	stats;
	struct irlap_cb		*irlap;
	struct pm_dev		*pmdev;
	struct qos_info		qos;

	/* => SIR */
	iobuff_t		tx_buff;
	iobuff_t		rx_buff;
	/* <= SIR */
};

#define IS_FIR(si)		((si)->speed >= 4000000)

#define HPSIR_MAX_RXLEN		2050
#define HPSIR_MAX_TXLEN		2050
#define TXBUFF_MAX_SIZE		HPSIR_MAX_TXLEN
#define SET_SIR_MODE            STISR = STISR_RCVEIR | STISR_XMITIR | STISR_XMODE

/*
 * If you want to disable debug information
 * please uncomment line bellow
 */

#define PXA_FIR_DUMP_ENABLE
#undef PXA_FIR_DUMP_ENABLE     


#define PXA_FIR_DEBUG_ENABLE
#undef PXA_FIR_DEBUG_ENABLE              

#define PXA_FIR_IRQ_DEBUG_ENABLE
#undef PXA_FIR_IRQ_DEBUG_ENABLE               

#ifdef PXA_FIR_DEBUG_ENABLE
#define __ECHO_IN printk(KERN_ERR "%s: enter\n",__FUNCTION__);
#define __ECHO_OUT printk(KERN_ERR "%s: exit\n",__FUNCTION__);
#define DBG(args...) printk(KERN_ERR __FUNCTION__"():"args);
#else
#define __ECHO_IN
#define __ECHO_OUT
#define DBG(args...)
#endif

#ifdef PXA_FIR_IRQ_DEBUG_ENABLE
#define DBG_IRQ(args...) printk(KERN_ERR __FUNCTION__"():"args);
#else
#define DBG_IRQ(args...)
#endif


static int pxa250_irda_set_speed(struct net_device *dev,int speed);
static void pxa250_start_rx_dma(struct net_device *dev);



/**************************************************************************
 *			Misc FIR/SIR functions				  *
 **************************************************************************/
/*
 * Allocate the receive buffer, unless it is already allocated.
 */

static int pxa250_irda_rx_alloc(struct pxa250_irda *si)
{
   __ECHO_IN;
   
   if (si->rxskb)
      return 0;

   si->rxskb = alloc_skb(HPSIR_MAX_RXLEN + 1, GFP_ATOMIC);

   if (!si->rxskb) {
      printk(KERN_ERR "pxa250_ir: out of memory for RX SKB\n");
      return -ENOMEM;
   }

   /*
    * Align any IP headers that may be contained
    * within the frame.
    */
   skb_reserve(si->rxskb, 1);

   __ECHO_OUT;

   return 0;
}



/**************************************************************************
 *			FIR						  *
 **************************************************************************/




static inline void pxa250_dma_stop(int ch)
{
   __ECHO_IN;

   DCSR(ch) &= ~DCSR_RUN;

   __ECHO_OUT;
   
}


static void pxa250_ficp_rx_start(void)
{
   ICCR0 = 0;
   ICCR2 =  1 << 2 | 0 << 3 ; 
   ICCR0 = ICCR0_ITR ;
   ICCR0 |= ICCR0_RIE |  ICCR0_RXE ; 
}

/*
 * Change Alternative Function encoding
 * Enable ICP unit
 * Disabe STUART unit
 * Enable IRQ unit clock;
 * Configure direction of GPIO used by ICP
 */


static void pxa250_do_fir_GPIO_config(void)
{
   /*
    * Modify GPIO 46 and 47 Alternate Function 
    */

   __ECHO_IN;

   /*Switch AF*/
   set_GPIO_mode (GPIO46_ICPRXD_MD);
   set_GPIO_mode (GPIO47_ICPTXD_MD);

   if (machine_is_lubbock())
      LUB_MISC_WR |= 1 << 4;

   /*init clock*/
   CKEN |= CKEN13_FICP;

   __ECHO_OUT;
}

/*
 * Low level hardware configuration and startup.
 */

static int pxa250_fir_irda_startup(struct pxa250_irda *si)
{

	__ECHO_IN;

	/*
	 * Disable STUART
	 */

	STIER &= ~IER_UUE;

	/*Disable STUART FIFO */
	STFCR = 0;

	/*
	 * Do low level configuration for HW AF and clock
	 */
	pxa250_do_fir_GPIO_config();

	__ECHO_OUT;
	return 0;
}


/*
 * Aieeeeee .. we should never get here :(
 */
static void pxa250_irda_rxdma_irq(int ch,void *id, struct pt_regs *regs)
{
   struct net_device *dev=id;
   struct pxa250_irda *si=dev->priv;
   u_int dcsr;


   __ECHO_IN;

   /* 
    * Make sure that irq is our.
    */

   if ( ch != si->rxdma_ch )
      /*just*/ return;

   /*
    * Check status 
    */
   dcsr = DCSR(ch);

   DBG("DCSR=%x\n",dcsr);

   if (dcsr &  DCSR_STOPSTATE )
   {
      DBG_IRQ("Chanel %d in stop state\n",ch);
   }

   if (dcsr &  DCSR_BUSERR )
   {
      /*
       * BUS Error we must restart reception
       */
      
      DBG("PXA IrDA: bus error interrupt on channel %d\n", ch);
      DCSR(ch) |= DCSR_BUSERR;
   }

   if (dcsr &  DCSR_ENDINTR )
   {
      DBG("PXA IrDA: Normal end of dma channel %d - packet to big\n", ch);
      DCSR(ch) |= DCSR_ENDINTR;
   }

   /* no mater what restart rx*/
   pxa250_start_rx_dma(dev);
   
   return ;
   
}


static void pxa250_irda_txdma_irq(int ch, void *id , struct pt_regs *regs)
{
   struct net_device *dev=id;
   struct pxa250_irda *si=dev->priv;
   struct sk_buff *skb = si->txskb;
   u_int dcsr;


   __ECHO_IN;
   DBG_IRQ("transmit\n"); 
   
     
   /* 
    * Make sure that irq is our.
    */

   if ( ch != si->txdma_ch )
      /*just*/ return;


   /*
    * Check status 
    */
   dcsr = DCSR(ch);

   DBG("DCSR=%x",dcsr);

   if (dcsr &  DCSR_STOPSTATE )
   {
      DBG("Chanel %d in stop state\n",ch);
   }

   if (dcsr &  DCSR_BUSERR )
   {
      DBG("PXA IrDA: bus error interrupt on channel %d\n", ch);
      DCSR(ch) |= DCSR_BUSERR;
      si->txskb = NULL;
   }

   if (dcsr &  DCSR_ENDINTR )
   {
      DBG("PXA IrDA: Normal end of dma channel %d\n", ch);
      DCSR(ch) |= DCSR_ENDINTR;
      si->txskb = NULL;
   }

   /*
    * Account and free the packet.
    */
   if (skb)
   {
      si->stats.tx_packets ++;
      si->stats.tx_bytes += skb->len;
      dev_kfree_skb_irq(skb);
   }

	/*Disable transceiver and enable receiver*/

	if (si->newspeed) {
	   pxa250_irda_set_speed(dev, si->newspeed);
	   si->newspeed = 0;
	}

	while (ICSR1 & ICSR1_TBY)
	   udelay(1);
	
     	ICCR0 &= ~ICCR0_TXE;

	
	enable_irq(si->fir_irq);

  	ICCR0 |= ICCR0_RXE; 

	/*
	 * Make sure that the TX queue is available for sending
	 * (for retries).  TX has priority over RX at all times.
	 */
	netif_wake_queue(dev);
	
	__ECHO_OUT;
}


static void pxa250_start_rx_dma(struct net_device *dev)
{
   struct pxa250_irda *si = dev->priv;
   int ch=si->rxdma_ch;

   if (!si->rxskb) {
      DBG("rx buffer went missing\n");
/*        return; */
   }

   DCSR(ch)=0;
   DCSR(ch)=DCSR_NODESC;
   DSADR(ch) = __PREG(ICDR);
   DTADR(ch) = si->rxbuf_dma; /* phisical address */;

   /* We should never do END_IRQ.  !!!*/
   DCMD(ch) = DCMD_ENDIRQEN| DCMD_INCTRGADDR | DCMD_FLOWSRC | DCMD_BURST8 | DCMD_WIDTH1 | HPSIR_MAX_RXLEN;

   /*
    * All right information will be available as soon as we set RXE flag
    */
   
   DCSR(ch) = DCSR_ENDINTR | DCSR_BUSERR;
   DCSR(ch) = DCSR_RUN | DCSR_NODESC ;

}




static int pxa250_get_rx_len(struct pxa250_irda *si)
{
   /*
    * DMA have to be stoped here
    */

   if ( ! (DCSR(si->rxdma_ch) & DCSR_STOPSTATE) )
      printk("warning dma have to be stoped befor counting len\n");
   
   return ( HPSIR_MAX_RXLEN - ( DCMD(si->rxdma_ch) & DCMD_LENGTH ) );
   
}

static void pxa250_irda_fir_error(struct net_device *dev)
{
   struct pxa250_irda *si = dev->priv;
   struct sk_buff *skb = si->rxskb;
   int len;
   int stat,data;

   __ECHO_IN;
   
      if (!skb)
      {
	 printk("pxa250 fir_error: SKB is NULL!\n");
	 return;
      }

      /*
       * Get the current data position.
       */

      len=pxa250_get_rx_len(si);
      DBG("RXLEN=%d\n",len);
      memcpy(skb->data, si->rxbuf_dma_virt, len);

      do {
	 /*
	  * Read Status, and then Data.
	  */
	   stat = ICSR1;
	   rmb();
	   data = ICDR;
	   if (stat & (ICSR1_CRE | ICSR1_ROR)) {
	      si->stats.rx_errors++;
	      if (stat & ICSR1_CRE)
		 si->stats.rx_crc_errors++;
	      if (stat & ICSR1_ROR)
		 si->stats.rx_frame_errors++;
	   } else
	      skb->data[len++] = data;

		/*
		 * If we hit the end of frame, there's
		 * no point in continuing.
		 */
	   if (stat & ICSR1_EOF)
	      break;
	} while (ICSR0 & ICSR0_EIF);

	if (stat & ICSR1_EOF) {
	   si->rxskb = NULL;

	   skb_put(skb, len);
	   skb->dev = dev;
	   skb->mac.raw = skb->data;
	   skb->protocol = htons(ETH_P_IRDA);
	   si->stats.rx_packets++;
	   si->stats.rx_bytes += len;

	   /*
	    * Before we pass the buffer up, allocate a new one.
	    */

	   si->rxskb = alloc_skb(HPSIR_MAX_RXLEN + 1, GFP_ATOMIC);

	   if (!si->rxskb) {
	      printk(KERN_ERR "pxa250_ir: out of memory for RX SKB\n");
	      return;
	   }

	   /*
	    * Align any IP headers that may be contained
	    * within the frame.
	    */
	   skb_reserve(si->rxskb, 1);

	   netif_rx(skb);
	}
}

/*
 * FIR format interrupt service routine.  We only have to
 * handle RX events; transmit events go via the TX DMA irq handler.
 *
 * No matter what, we disable RX, process, and then restart RX.
 */

static void pxa250_irda_fir_irq(int irq, void *dev_id, struct pt_regs *regs)
{
   struct net_device *dev = dev_id;
   struct pxa250_irda *si = dev->priv;
   int status;
   
   /*
    * Stop RX
    */

   __ECHO_IN;

   pxa250_dma_stop(si->rxdma_ch);

   
   /*
    * Framing error - we throw away the packet completely.
    * Clearing RXE flushes the error conditions and data
    * from the fifo.
    */
   status=ICSR0;
   
   if (status & (ICSR0_FRE | ICSR0_RAB)) {
      DBG_IRQ("Framing error or RAB\n"); 
      
      si->stats.rx_errors++;

      if (ICSR0 & ICSR0_FRE)
	 si->stats.rx_frame_errors++;

      /* Clear RX fifo
       * DMA will be cleared when we restart RX
       * Should we check RNE after that? 
       */

      ICCR0 &= ~ICCR0_RXE;
      
      /*
       * Clear selected status bits now, so we
       * don't miss them next time around.
       */
      ICSR0 = status & (ICSR0_FRE | ICSR0_RAB);
   }

   
   /*
    * Deal with any receive errors.  The any of the lowest
    * 8 bytes in the FIFO may contain an error.  We must read
    * them one by one.  The "error" could even be the end of
    * packet!
    */
   if (ICSR0 & ICSR0_EIF)
      pxa250_irda_fir_error(dev);

   /*
    * No matter what happens, we must restart reception.
    */

   ICCR0 = 0;
   pxa250_start_rx_dma(dev);
   pxa250_ficp_rx_start();
   __ECHO_OUT;
}





/**************************************************************************
 *			SIR 						  *
 **************************************************************************/
/*
 * HP-SIR format interrupt service routines.
 */
static void pxa250_sir_transmit(struct net_device *dev)
{
   struct pxa250_irda *si = dev->priv;
   
   if (si->tx_buff.len) 
   {
	/* Disable receiver and  enable transmiter*/

      
      
		STISR &= ~STISR_RCVEIR; 
//	        STISR |= STISR_XMITIR;

				
		
                disable_irq(dev->irq);
		
		do 
		{

		   if (STLSR & LSR_TDRQ)
		   {
		      STTHR = *si->tx_buff.data++;
		      si->tx_buff.len -= 1;
 
		      tx_count++;
		   }
		   
				         
		} while (si->tx_buff.len);

				
		if (si->tx_buff.len == 0) 
		{
		   
		   
			si->stats.tx_packets++;
			si->stats.tx_bytes += si->tx_buff.data -
					      si->tx_buff.head;

			/*
			 * We need to ensure that the transmitter has
			 * finished.
			 */
			
		        do
			{
			   udelay(1);
			   
            		}				
			while ( ! (STLSR & LSR_TEMT) );
                       
						
			/*

			 * Ok, we've finished transmitting.  Now enable
			 * the receiver.  Sometimes we get a receive IRQ
			 * immediately after a transmit...
			 */

			if (si->newspeed)
			{
				pxa250_irda_set_speed(dev, si->newspeed);
				si->newspeed = 0;
			}

			/* I'm hungry! */
			netif_wake_queue(dev);
		}
		
		enable_irq (dev->irq);
                STIER = (IER_RAVIE | IER_UUE | IER_RTIOE);

		STISR |= STISR_RCVEIR;
//		STISR &= ~STISR_XMITIR;
   }
}

static void pxa250_irda_hpsir_irq(struct net_device *dev)
{
	struct pxa250_irda *si = dev->priv;

	/*
	 * Deal with any receive errors first.  The bytes in error may be
	 * the only bytes in the receive FIFO, so we do this first.
	 */
  	__ECHO_IN; 
	
	while (STLSR & LSR_FIFOE)
	{
		int stat, data;

		stat = STLSR; 
		data = STRBR;
               
		
                if (stat & (LSR_FE | LSR_OE | LSR_PE)) 
		
		{
		        si->stats.rx_errors++;
			if (stat & LSR_FE) 
				si->stats.rx_frame_errors++;
			if (stat & LSR_OE) 
				si->stats.rx_fifo_errors++;
			
		} else
		{
		   rx_count++;
		   async_unwrap_char(dev, &si->stats, &si->rx_buff, data);
                }
		
	}

	/*
	 * We must clear certain bits.
	 */
	 
	if (STLSR & (LSR_DR)) 
	{
		/*
		 * Fifo contains at least 1 character.
		 */
		do
		{
		   int data;
		   
		   data = STRBR;
		   
		   async_unwrap_char(dev, &si->stats, &si->rx_buff,
					  data); /* was Ser2UTDR); Clo */
		   rx_count++;
		   
		} while (STLSR & LSR_DR); 
		
		dev->last_rx = jiffies;
	}

  	__ECHO_OUT; 
}

static void pxa250_sir_irda_shutdown(struct pxa250_irda *si)
{

   STIER = 0;
   STFCR = 0;
   STISR = 0;
   CKEN &= ~CKEN5_STUART; 
}


/************************************************************************************/

/*Low level init/uninstall function PM control and IrDA protocol stack registration */

/*
 * Set the IrDA communications speed.
 * Interrupt have to be disabled here.
 */

static int pxa250_irda_startup(struct net_device *dev)
{
   

   __ECHO_IN;

   /*
    * Ensure that the ports for this device are setup correctly.
    */


   set_GPIO_mode (GPIO46_STRXD_MD);
   set_GPIO_mode (GPIO47_STTXD_MD);

   STMCR = MCR_OUT2;
   STLCR = LCR_WLS1 | LCR_WLS0;

   SET_SIR_MODE;
   CKEN |= CKEN5_STUART;
   /* enable irq from stuart */
   ICMR |= ( 1 << 20 );
	
   /*reset FIFO*/
		
/*	STFCR = FCR_TRFIFOE |  FCR_RESETTF | FCR_RESETRF;// | FCR_ITL_16;

	STIER = IER_UUE | IER_RAVIE | IER_RTOIE;
*/	
   __ECHO_OUT;

   return 0;
	
}


#ifdef CONFIG_PM
/*
 * Suspend the IrDA interface.
 */

static int pxa250_irda_shutdown(struct pxa250_irda *si)
{

   pxa250_sir_irda_shutdown(si);
   return 0;
   
}


static int pxa250_irda_suspend(struct net_device *dev, int state)
{
	struct pxa250_irda *si = dev->priv;

	if (si && si->open) {
	   /*
	    * Stop the transmit queue
	    */
	   if (IS_FIR(si))
	      return -1;

	   netif_stop_queue(dev);
	   disable_irq(dev->irq);
	   disable_irq(si->fir_irq);
	   pxa250_sir_irda_shutdown(si);
	}

	return 0;
}

/*
 * Resume the IrDA interface.
 */

static int pxa250_irda_resume(struct net_device *dev)
{
	struct pxa250_irda *si = dev->priv;

	__ECHO_IN;
	
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

		pxa250_irda_startup(dev);
		enable_irq(dev->irq);

		/*
		 * This automatically wakes up the queue
		 */
		netif_wake_queue(dev);
		pxa250_irda_set_speed(dev,si->speed = 9600);
		
	}

	__ECHO_OUT;
	return 0;
}

static int pxa250_irda_pmproc(struct pm_dev *dev, pm_request_t rqst, void *data)
{
	int ret;

	
	if (!dev->data)
		return -EINVAL;


	switch (rqst) {
	case PM_SUSPEND:
		ret = pxa250_irda_suspend((struct net_device *)dev->data,
					  (int)data);
		break;

	case PM_RESUME:
		ret = pxa250_irda_resume((struct net_device *)dev->data);
		break;

	default:

	   ret = -EINVAL;
		break;
	}

	return ret;
}
#endif




static void pxa250_irda_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	struct net_device *dev = dev_id;
	
	pxa250_irda_hpsir_irq(dev);
	
}


static int pxa250_irda_hard_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct pxa250_irda *si = dev->priv;
	int speed = irda_get_next_speed(skb);
	int mtt;
	
  	__ECHO_IN; 

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
			pxa250_irda_set_speed(dev, speed);
		}
		dev_kfree_skb(skb);
		return 0;
	}


  	DBG("stop queue\n"); 
	netif_stop_queue(dev);

	if(!IS_FIR(si))
	{
	   
	   si->tx_buff.data = si->tx_buff.head;
	   si->tx_buff.len  = async_wrap_skb(skb, si->tx_buff.data,
						  si->tx_buff.truesize);

        
	   pxa250_sir_transmit(dev);

	
	
	   dev_kfree_skb(skb);

	   dev->trans_start = jiffies;

	   return 0;
	}
	else /* FIR */
	{
	   DBG("Enter FIR transmit\n");
	   /*
	    * We must not be transmitting...
	    */
	   if (si->txskb)
	      BUG();

      	   disable_irq(si->fir_irq); 
	   
	   netif_stop_queue(dev);
	   DBG("queue stoped\n");
	   si->txskb = skb;

	   /* we could not just map so we'll need some triks */
	   /* skb->data may be not DMA capable -Sed- */


	   if (skb->len > TXBUFF_MAX_SIZE)
	   {
	      printk (KERN_ERR "skb data too large\n");
	      printk (KERN_ERR "len=%d",skb->len);
	      BUG();
	   }
		

	   DBG("gonna copy %d bytes to txbuf\n",skb->len);

	   memcpy (si->txbuf_dma_virt, skb->data , skb->len);
	   
	   /* Actual sending ;must not be receiving !!! */
	   /* Write data and source address */

	   DBG("ICSR1 & RNE =%d\n",(ICSR1 & ICSR1_RNE) ? 1 : 0 );

	   /*Disable receiver and enable transifer */
  	   ICCR0 &= ~ICCR0_RXE;      
	   
	   if (ICSR1 & ICSR1_TBY)
	      BUG();

    	   ICCR0 |= ICCR0_TXE;  
		
	   DBG("FICP status %x\n",ICSR0);

	   if (0){
	      int i;
		   
	      DBG("sending packet\n");
	      for (i=0;i<skb->len;i++)
		 (i % 64) ? printk ("%2x ",skb->data[i]) : printk ("%2x \n",skb->data[i]) ;
	      DBG(" done\n");
   
	   }
	   /*
	    * If we have a mean turn-around time, impose the specified
	    * specified delay.  We could shorten this by timing from
	    * the point we received the packet.
	    */
	   
	   mtt = irda_get_mtt(skb); 
	   if(mtt)    
	      udelay(mtt);    
	   
	   DCSR(si->txdma_ch)=0;
	   DCSR(si->txdma_ch)=DCSR_NODESC;
	   DSADR(si->txdma_ch) = si->txbuf_dma; /* phisic address */
	   DTADR(si->txdma_ch) = __PREG(ICDR);
		
	   DCMD(si->txdma_ch) = DCMD_ENDIRQEN| DCMD_INCSRCADDR | DCMD_FLOWTRG | DCMD_BURST8 | DCMD_WIDTH1 | skb->len;

	   DCSR(si->txdma_ch) = DCSR_ENDINTR | DCSR_BUSERR;
	   DCSR(si->txdma_ch) = DCSR_RUN | DCSR_NODESC ;

	   DBG("FICP status %x\n",ICSR0);

	   return 0;
	}
	
}

static int
pxa250_irda_ioctl(struct net_device *dev, struct ifreq *ifreq, int cmd)
{
	struct if_irda_req *rq = (struct if_irda_req *)ifreq;
	struct pxa250_irda *si = dev->priv;
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
				ret = pxa250_irda_set_speed(dev,
						rq->ifr_baudrate);
			} else {
				printk("pxa250_irda_ioctl: SIOCSBANDWIDTH: !netif_running\n");
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

	__ECHO_OUT;

	return ret;
}

static struct net_device_stats *pxa250_irda_stats(struct net_device *dev)
{
	struct pxa250_irda *si = dev->priv;
	return &si->stats;
}

static int pxa250_irda_start(struct net_device *dev)
{
	struct pxa250_irda *si = dev->priv;
	int err;
	unsigned int flags;
	

	MOD_INC_USE_COUNT;

	__ECHO_IN;
	si->speed = 9600;

	local_irq_save(flags);
	
	err = request_irq(si->fir_irq, pxa250_irda_fir_irq, 0,  dev->name, dev);
	if (err)
		goto err_fir_irq;

	err = request_irq(dev->irq, pxa250_irda_irq, 0, dev->name, dev);
	if (err)
		goto err_irq;

	/*
	 * The interrupt must remain disabled for now.
	 */
	
	disable_irq(dev->irq);
  	disable_irq(si->fir_irq);

	local_irq_restore(flags);


	/* Allocate DMA channel for receiver (not used) */
	err = pxa_request_dma("IrDA receive", DMA_PRIO_LOW, pxa250_irda_rxdma_irq, dev);
	if (err < 0 )
	   goto err_rx_dma;
	si->rxdma_ch=err;

	DRCMRRXICDR = DRCMR_MAPVLD | si->rxdma_ch;
	

	/* Allocate DMA channel for transmit */
	err = pxa_request_dma("IrDA transmit", DMA_PRIO_LOW, pxa250_irda_txdma_irq , dev);
	if (err < 0 )
	   goto err_tx_dma;

	si->txdma_ch=err;

	/*
	 * Make sure that ICP will be able 
	 * to assert the transmit dma request bit
	 * through the peripherals request bus (PREQ)
	 */
	
	DRCMRTXICDR = DRCMR_MAPVLD | si->txdma_ch;

	DBG("rx(not used) channel=%d tx channel=%d\n",si->rxdma_ch,si->txdma_ch);
	
	/* allocate consistent buffers for dma access
	 * buffers have to be aligned and situated in dma capable memory region;
	 */
	si->rxbuf_dma_virt = consistent_alloc(GFP_KERNEL | GFP_DMA ,HPSIR_MAX_RXLEN , &si->rxbuf_dma);
	if (! si->rxbuf_dma_virt )
		goto err_rxbuf_dma;

	si->txbuf_dma_virt = consistent_alloc(GFP_KERNEL | GFP_DMA, HPSIR_MAX_TXLEN,  &si->txbuf_dma); 
	if (! si->txbuf_dma_virt )
		goto err_txbuf_dma;

	/* Alocate skb for receiver */
	err=pxa250_irda_rx_alloc(si);
	if (err)
	   goto err_rx_alloc;
	
	/*
	 * Setup the serial port for the specified config.
	 */
	err = pxa250_irda_startup(dev);
	if (err)
		goto err_startup;

	pxa250_irda_set_speed(dev,si->speed = 9600);


	/*
	 * Open a new IrLAP layer instance.
	 */
	si->irlap = irlap_open(dev, &si->qos, "pxa250");
	err = -ENOMEM;
	if (!si->irlap)
		goto err_irlap;

	/*
	 * Now enable the interrupt and start the queue
	 */
	si->open = 1;
	enable_irq(dev->irq);
	netif_start_queue(dev);
	return 0;

err_irlap:
	si->open = 0;
	pxa250_sir_irda_shutdown(si);
err_startup:
	dev_kfree_skb(si->rxskb);
err_rx_alloc:	
	consistent_free (si->txbuf_dma_virt,HPSIR_MAX_TXLEN,si->txbuf_dma);
err_txbuf_dma:
	consistent_free (si->rxbuf_dma_virt,HPSIR_MAX_RXLEN,si->rxbuf_dma);
err_rxbuf_dma:
	pxa_free_dma(si->txdma_ch);
err_tx_dma:
	pxa_free_dma(si->rxdma_ch);
err_rx_dma:
	free_irq(dev->irq, dev);
err_irq:
	free_irq(si->fir_irq, dev);
err_fir_irq:	
	MOD_DEC_USE_COUNT;
	return err;
}

static int pxa250_irda_stop(struct net_device *dev)
{
	struct pxa250_irda *si = dev->priv;
	
	printk(KERN_ERR "Irda stop... RX = %d TX = %d\n",rx_count,tx_count);

	disable_irq(dev->irq);
  	disable_irq(si->fir_irq); 
/*  	pxa250_irda_shutdown(si); */

	/*
	 * If we have been doing DMA receive, make sure we
	 * tidy that up cleanly.
	 */
	if (si->rxskb) {
	        dev_kfree_skb(si->rxskb);
		si->rxskb = NULL;
	}

	/* Stop IrLAP */
	if (si->irlap) {
		irlap_close(si->irlap);
		si->irlap = NULL;
	}

	consistent_free (si->txbuf_dma_virt,HPSIR_MAX_TXLEN,si->txbuf_dma);
	consistent_free (si->rxbuf_dma_virt,HPSIR_MAX_RXLEN,si->rxbuf_dma);
	pxa_free_dma(si->txdma_ch);
	pxa_free_dma(si->rxdma_ch);

	netif_stop_queue(dev);
	si->open = 0;

	/*
	 * Free resources
	 */
	free_irq(dev->irq, dev);
	free_irq(si->fir_irq, dev);


	MOD_DEC_USE_COUNT;

	return 0;
}

static int pxa250_irda_init_iobuf(iobuff_t *io, int size)
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




static int pxa250_stop_fir(struct net_device *dev)
{
   struct pxa250_irda *si = dev->priv;
   unsigned int flag;

   save_flags(flag);
   cli();
   
   pxa250_dma_stop(si->txdma_ch);
   pxa250_dma_stop(si->rxdma_ch);

   if (si->txskb)
      dev_kfree_skb_irq(si->txskb);

   ICCR0 &= ~(ICCR0_RXE | ICCR0_TXE );
   disable_irq(si->fir_irq);
   CKEN &= ~CKEN13_FICP;

   restore_flags(flag);

   return 0;
}



static int pxa250_irda_set_speed(struct net_device *dev, int speed)
{
   struct pxa250_irda *si = dev->priv;
   int brd, ret = -EINVAL;
   static int last_fir_speed=0;

   __ECHO_IN;
   


   switch (speed) {
      case 9600:	case 19200:	case 38400:
      case 57600:	case 115200:
	   
	 /* Baud rate fixed - Clo */

	 /*
	  * FIXME
	  */
	 if (last_fir_speed) 
	 {

	    pxa250_stop_fir(dev);
	    set_GPIO_mode (GPIO46_STRXD_MD);
	    set_GPIO_mode (GPIO47_STTXD_MD);
   
	    enable_irq(dev->irq);
	    netif_wake_queue(dev);
	    last_fir_speed=0;
	 }
	 

	 LUB_MISC_WR &= ~(1 << 4);

	 brd = 14745600 / (16 * speed); 

	 STLCR |= LCR_DLAB;

	 STDLH = brd >> 8; /* Clo: set Divisor Latch High */
	 STDLL = brd & 0xFF; /* Clo: set Devisor Latch Low */
		
	 STLCR &= ~LCR_DLAB; /* Clo: clear DLAB bit */ 

	 STMCR = MCR_OUT2;

	 CKEN |= CKEN5_STUART;

	 ICMR |= ( 1 << 20 );
		
	 STLCR = LCR_WLS1 | LCR_WLS0;

	 SET_SIR_MODE;
		
	 STFCR = FCR_TRFIFOE |  FCR_RESETTF | FCR_RESETRF | FCR_ITL_1 ;// | FCR_ITL_16;

	 STIER = IER_UUE | IER_RAVIE | IER_RTIOE;

	 si->speed = speed;

	 ret = 0;
	 break;

      case 4000000:

	 if (last_fir_speed)
	    goto speed_out;
	 
	 disable_irq(dev->irq);

	 pxa250_sir_irda_shutdown(si);
	 pxa250_fir_irda_startup(si);
	 pxa250_irda_rx_alloc(si);
	 ICCR0=0;
	 pxa250_start_rx_dma(dev);
	 pxa250_ficp_rx_start();
	 
	 enable_irq(si->fir_irq);
	 DBG("enable FIR \n");
	 si->speed = speed;

	 netif_wake_queue(dev);
	 last_fir_speed=1;
speed_out:
	 
	 ret=0;
	 
	 break;

      default:
	 break;
   }
   __ECHO_OUT;

   return ret;
}


static int pxa250_irda_net_init(struct net_device *dev)
{
	struct pxa250_irda *si = dev->priv;
	unsigned int baudrate_mask;
	int err = -ENOMEM;

	si = kmalloc(sizeof(struct pxa250_irda), GFP_KERNEL);
	if (!si)
		goto out;

	memset(si, 0, sizeof(*si));

	/*
	 * Initialise the HP-SIR buffers
	 */

	err = pxa250_irda_init_iobuf(&si->rx_buff, 14384);
	if (err)
		goto out;
	err = pxa250_irda_init_iobuf(&si->tx_buff, 4000);
	if (err)
		goto out_free_rx;

	si->fir_irq		= IRQ_ICP;
	dev->priv = si;
	dev->hard_start_xmit	= pxa250_irda_hard_xmit;
	dev->open		= pxa250_irda_start;
	dev->stop		= pxa250_irda_stop;
	dev->do_ioctl		= pxa250_irda_ioctl;
	dev->get_stats		= pxa250_irda_stats;

	irda_device_setup(dev);
	irda_init_max_qos_capabilies(&si->qos);

	/*
	 * We support original IRDA up to 115k2. (we don't currently
	 * support 4Mbps).  Min Turn Time set to 1ms or greater.
	 */
	baudrate_mask = IR_9600|IR_19200|IR_38400|IR_57600|IR_115200;
  	baudrate_mask |= IR_4000000 << 8; 
	si->qos.baud_rate.bits &= baudrate_mask;
	si->qos.min_turn_time.bits = 7;

	irda_qos_bits_to_value(&si->qos);

#ifdef CONFIG_PM
	/*
	 * Power-Management is optional.
	 */
	si->pmdev = pm_register(PM_SYS_DEV, PM_SYS_IRDA, pxa250_irda_pmproc);
	if (si->pmdev)
		si->pmdev->data = dev;
#endif

	return 0;

	kfree(si->tx_buff.head);
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
static void pxa250_irda_net_uninit(struct net_device *dev)
{
	struct pxa250_irda *si = dev->priv;

	dev->hard_start_xmit	= NULL;
	dev->open		= NULL;
	dev->stop		= NULL;
	dev->do_ioctl		= NULL;
	dev->get_stats		= NULL;
	dev->priv		= NULL;

	pm_unregister(si->pmdev);

	kfree(si->tx_buff.head);
	kfree(si->rx_buff.head);
	kfree(si);
}

static int __init pxa250_irda_init(void)
{
	struct net_device *dev;
	int err;

	/* STUART */
	err = request_mem_region(__PREG(STRBR), 0x24, "IrDA") ? 0 : -EBUSY;
	if (err)
		goto err_mem_1;

	/* FIR */
	err = request_mem_region(__PREG(ICCR0), 0x1c, "IrDA") ? 0 : -EBUSY;
	if (err)
		goto err_mem_2;


	rtnl_lock();
	dev = dev_alloc("irda%d", &err);
	if (dev) {
		dev->irq    = IRQ_STUART;
		dev->init   = pxa250_irda_net_init;
		dev->uninit = pxa250_irda_net_uninit;

		err = register_netdevice(dev);

		if (err)
			kfree(dev);
		else
			netdev = dev;
	}
	rtnl_unlock();

	if (err) {
		release_mem_region(__PREG(ICCR0), 0x1c);
err_mem_2:
		release_mem_region(__PREG(STRBR), 0x24);
	}
err_mem_1:
	return err;
}

static void __exit pxa250_irda_exit(void)
{
	struct net_device *dev = netdev;

	netdev = NULL;
	if (dev) {
		rtnl_lock();
		unregister_netdevice(dev);
		rtnl_unlock();
	}

        release_mem_region(__PREG(ICCR0), 0x1c);

	release_mem_region(__PREG(STRBR), 0x24);

	/*
	 * We now know that the netdevice is no longer in use, and all
	 * references to our driver have been removed.  The only structure
	 * which may still be present is the netdevice, which will get
	 * cleaned up by net/core/dev.c
	 */
}

module_init(pxa250_irda_init);
module_exit(pxa250_irda_exit);

MODULE_AUTHOR("Alexey Lugovskoy Frasenyak Dmitrij");
MODULE_DESCRIPTION("PXA250 SIR/FIR");
MODULE_LICENSE("GPL");
EXPORT_NO_SYMBOLS;
