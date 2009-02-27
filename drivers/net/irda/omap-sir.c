/*
 * BRIEF MODULE DESCRIPTION
 *
 *	Infra-red SIR driver for the OMAP Innovator Platform
 *
 *
 * Copyright 2002 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *	   source@mvista.com
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

/*
 * Our netdevice.  There is only ever one of these.
 */
 
static struct net_device *netdev; /* Our net device */
static int rx_state = 0;          /* Rec state for IOCTL */
struct omap_irda {
	
	unsigned char		open;

	int			speed;
	int			newspeed;

	struct sk_buff		*txskb;
	struct sk_buff		*rxskb;
	
	struct net_device_stats	stats;
	struct irlap_cb		*irlap;
	struct qos_info		qos;

};

/* Linux Driver Model Structures */

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CEE */
#include <linux/device.h>

static int omap_irda_suspend(struct device * dev, u32 state, u32 level);
static int omap_irda_resume(struct device * dev, u32 level);

static struct device_driver irda_driver_ldm = {
       name:      "omap1510-IrDA",
       devclass:  NULL,
       probe:     NULL,
       suspend:   omap_irda_suspend,
       resume:    omap_irda_resume,
       remove:    NULL,
};

static struct device irda_device_ldm = {
       name: "OMAP1510 IrDA",
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
 * Allocate and map the receive buffer, unless it is already allocated.
 */

/*
 * If you want to disable debug information
 * please uncomment line bellow
 */


//#define OMAP_SIR_DEBUG_ENABLE
/*  #undef OMAP_SIR_DEBUG_ENABLE       */

#ifdef OMAP_SIR_DEBUG_ENABLE
#define __ECHO_IN printk(KERN_ERR "%s: enter\n",__FUNCTION__);
#define __ECHO_OUT printk(KERN_ERR "%s: exit\n",__FUNCTION__);
#define DBG(args...) printk(KERN_ERR __FUNCTION__"():"args);
#else
#define __ECHO_IN
#define __ECHO_OUT
#define DBG(args...)
#endif

#define DBG_IRQ(args...) printk(KERN_ERR __FUNCTION__"():"args);
/*  #undef DBG_IRQ */
/*  #define DBG_IRQ(args...)  */

/* hate warnings */
static int omap_irda_set_speed(struct net_device *dev,int speed);


/**************************************************************************
 *			Misc SIR functions				  *
 **************************************************************************/
static int omap_irda_rx_alloc(struct omap_irda *si)
{
   __ECHO_IN;
   
   //si->rxskb = alloc_skb(HPSIR_MAX_RXLEN + 1, GFP_ATOMIC);

   si->rxskb = alloc_skb(4096, GFP_ATOMIC);
   if (!si->rxskb) {
      printk(KERN_ERR "omap_sir: out of memory for RX SKB\n");
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
 *			SIR 						  *
 **************************************************************************/
/*
 * SIR interrupts and services routines.
 */

static void omap_irda_tx_irq(struct net_device *dev)
{

    int tx_delay;                     /* Depends on current baudrate */
    struct omap_irda *si = dev->priv;
    struct sk_buff *skb = si->txskb;    

    if (skb->len) 
    {
       while(!(inb(UART3_SSR) & 1) && (skb->len))  
       {
	  if (skb->len == 1) 
             outb(inb(UART3_ACREG) | 1, UART3_ACREG);

          outb(*skb->data++, UART3_THR);

          skb->len -= 1;
          si->stats.tx_bytes +=1;
        }
    } 
				         
    if (skb->len == 0) 
    {
       
      
    /*
     * We need to ensure that the transmitter has
     * finished.
     */

       while(!(inb(UART3_LSR) & (1 << 7)))
       {
         udelay(1);
       }

       tx_delay = (50 * 1000000)/si->speed; /* delay in us */    

       while(tx_delay)
       {
          udelay(1);
          tx_delay--;
       }

    /* Transmission finished */

       si->stats.tx_packets++;
		
       if (si->newspeed)
       {
          omap_irda_set_speed(dev, si->newspeed);
          si->newspeed = 0;
       }
    /* Now - RX IR should be enabled */
 
       dev_kfree_skb(si->txskb);

       netif_wake_queue(dev);

       outb(inb(UART3_ACREG) & ~(1 << 5), UART3_ACREG); 

       inb(UART3_RESUME);

       outb(0x81, UART3_IER);
    }

}

static void omap_irda_rx_irq(struct net_device *dev)
{
	struct omap_irda *si = dev->priv;
        struct sk_buff *skb = si->rxskb;
        u8 status,data;

  	__ECHO_IN; 
        
	/*
	 * Fifo contains at least 1 character.
	 */

        status = inb(UART3_LSR);
        rx_state = 1;

        while(!(status & 1))
	{

	   data = inb(UART3_RHR);

           *skb_put(skb,1) = data;
      	
           status = inb(UART3_LSR);

           if(status & ((1 << 4) | (1 << 3) | (1 << 2))) /* Recieve errors? */
	   {
	     si->stats.rx_errors++;
	     inb(UART3_RESUME);
	   }
         }  

       dev->last_rx = jiffies;
	
  	__ECHO_OUT; 
}

static void omap_irda_eof_irq(struct net_device *dev)
{
   struct omap_irda *si = dev->priv;
   struct sk_buff *skb = si->rxskb;
   u8 status;
   
   status = inb(UART3_SFLSR);
         
   if(status != 0)               /* Bad frame? */
   {
      si->stats.rx_frame_errors++;
      dev_kfree_skb(si->rxskb);        /* Discard all data and free buffer*/ 
      si->rxskb = NULL;
      omap_irda_rx_alloc(si);          /* Allocate new buffer */
      skb = si->rxskb;
     // printk(KERN_ERR "Err in Frame: %x\n", status);
      inb(UART3_RESUME);
    }
    else
    {
	si->rxskb = NULL;          
        skb->dev = dev;
        skb->mac.raw = skb->data;
        skb->protocol = htons(ETH_P_IRDA);
        si->stats.rx_packets++;
        si->stats.rx_bytes += skb->len;
        omap_irda_rx_alloc(si);   /* Allocate new buffer */
        skb_trim(skb, skb->len - 2);
        netif_rx(skb);            /* Send data to upper level */
        skb = si->rxskb;
        inb(UART3_RESUME);
    }     
   rx_state = 0;
}


static void omap_sir_irda_shutdown(struct omap_irda *si)
{

/* Disable all UART3 Interrupts */

  outb(0, UART3_IER);
  
  /* Disable UART3 and disable baud rate generator */

  outb(0x07, UART3_MDR1);         /* Put UART3 in reset mode*/

  outb((1 << 5), UART3_ACREG);       /* set SD_MODE pin to high and Disable RX IR */ 

  /* Clear DLH and DLL */

  outb(1 << 7, UART3_LCR);
  outb(0, UART3_DLL);
  outb(0, UART3_DLH);  

  /* Clear RX/TX FIFO and disable FIFO*/

  outb((1 << 1) | (1 << 2), UART3_FCR); 

}

/************************************************************************************/

/*Low level init/uninstall function PM control and IrDA protocol stack registration */

/************************************************************************************/
/*
 * Set the IrDA communications speed.
 * Interrupt have to be disabled here.
 */

static int omap_irda_startup(struct net_device *dev)
{

   __ECHO_IN;

   
   /* Mux UART3 signal out */

   /* For Compatibility Mode */

   outl(inl(FUNC_MUX_CTRL_0) |  ((1 << 26) | (1 << 9)), FUNC_MUX_CTRL_0);
   outl(inl(FUNC_MUX_CTRL_0) &  ~(1 << 5), FUNC_MUX_CTRL_0);

   /* For Native Mode */

   outl(inl(0xFFFE101C) | 1, 0xFFFE101C);
   outl(inl(0xFFFE101C) & ~((1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5)), 0xFFFE101C);
   outl(inl(0xFFFE102C) | 1, 0xFFFE102C);
   
   outb(0x07, UART3_MDR1);         /* Put UART3 in reset mode*/

   /* Clear DLH and DLL */

   outb(1 << 7, UART3_LCR);

   outb(0, UART3_DLL);
   outb(0, UART3_DLH);

   /* Set EFR[4] to 1 */

   outb(0xBF, UART3_LCR);

   outb(1 << 4, UART3_EFR);


   /* Set LCR[7] = 1 to set MCR[6] = 1 */

   outb(1 << 7, UART3_LCR);

   outb(1 << 6, UART3_MCR);
   
   /* Set SCR = 0 */

   outb(0x00, UART3_SCR);


   /* Set RX triger to 1  and TX trigger to 16 */

   outb((1 << 1), UART3_TLR);


   /* Clear LCR[7] */

   outb(0x03, UART3_LCR);


   /* Clear RX and TX FIFO, and enable FIFO */   

   outb(0x07 | (1 << 6), UART3_FCR);


   /* Clear MCR[6] */

   outb(0, UART3_MCR);


   /* Clear EFR[4] */

   outb(0xBF, UART3_LCR);

   outb(0x00, UART3_EFR);


   /* Set SCR[6,7] for RX and TX FIFO granularity of 1 */

   outb((1 << 7) | (1 << 6), UART3_SCR);

   
   /* Set LCR to 8 bits and 1 stop bit */

   outb(0x03, UART3_LCR);

 
   /* Enable UART3 SIR Mode */
 
   outb(0x81, UART3_MDR1); 


   /* Set Status FIFO trig to 1 */

   outb(0, UART3_MDR2);

   /* Clear Status Fifo */

   while(!(inb(UART3_LSR) & (1 << 1)))
   {
     inb(UART3_SFLSR);
   }

   /* Enables RXIR input and set SIR pulse-width to 1.6 uS */
   /* and disable TX underrun */

   outb((1 << 7) | (1 << 6) | (1 << 4), UART3_ACREG);


   /* Enable RX Interrupt */

   outb(0x81, UART3_IER);


   /* Set Maximum Received Frame size */

   outb(0xFF, UART3_RXFLL);
   outb(0x0f, UART3_RXFLH);

   inb(UART3_RESUME);

   __ECHO_OUT;

   return 0;
	
}

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CEE */

/*
 * Suspend the IrDA interface.
 */

static int omap_irda_suspend(struct device *dev, u32 state, u32 level)
{
	struct omap_irda *si = netdev->priv;
        unsigned int flags;

	__ECHO_IN;

        switch(level)
        {
           case SUSPEND_POWER_DOWN: 

       /* Turn off power */
	        save_flags(flags);  
                cli();
              
        	netif_stop_queue(netdev);
	
                disable_irq(netdev->irq);

	        omap_sir_irda_shutdown(si);

                restore_flags(flags);
        }

	__ECHO_OUT;

	return 0;
}

/*
 * Resume the IrDA interface.
 */

static int omap_irda_resume(struct device *dev, u32 level)
{
	struct omap_irda *si = netdev->priv;
        unsigned int flags;

        __ECHO_IN;

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
	        save_flags(flags); 
	        cli(); 

                omap_irda_startup(netdev);

        	if (si->newspeed) {
	           si->speed = si->newspeed;
	           si->newspeed = 0;
	        }
                else
		{
		   si->speed = 9600;
		}    

                omap_irda_set_speed(netdev, si->speed);

                enable_irq(netdev->irq);	  

                netif_wake_queue(netdev);
                 
                restore_flags(flags);

	}

        __ECHO_OUT;

	return 0;
}
#endif

static void omap_irda_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	struct net_device *dev = dev_id;
        int int_ind  = inb(UART3_IIR);

	
	//printk(KERN_INFO "IIR = %x", int_ind);

        if (int_ind & 2)
	{
          omap_irda_tx_irq(dev); /*TX Handler */
	}
        else if (int_ind & 1)
          omap_irda_rx_irq(dev); /*RX Handler */
        else if (int_ind & 0x80)
          omap_irda_eof_irq(dev);
        
}


static int omap_irda_hard_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct omap_irda *si = dev->priv;
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
			omap_irda_set_speed(dev, speed);
		}
		dev_kfree_skb(skb);
		return 0;
	}

/*  	DBG("stop queue\n"); */

       netif_stop_queue(dev);

/*     DBG("sir xmit \n"); */

       si->txskb = skb;

       /* Disable RX IR */

       outb(inb(UART3_ACREG) | (1 << 5), UART3_ACREG);
     
       outb(0x02, UART3_IER);

       dev->trans_start = jiffies;

       return 0;
}

static int
omap_irda_ioctl(struct net_device *dev, struct ifreq *ifreq, int cmd)
{
	struct if_irda_req *rq = (struct if_irda_req *)ifreq;
	struct omap_irda *si = dev->priv;
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
				ret = omap_irda_set_speed(dev,
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

static struct net_device_stats *omap_irda_stats(struct net_device *dev)
{
	struct omap_irda *si = dev->priv;
	return &si->stats;
}

static int omap_irda_start(struct net_device *dev)
{
	struct omap_irda *si = dev->priv;
	int err;
	unsigned int flags;
	

	MOD_INC_USE_COUNT;

	__ECHO_IN;
	si->speed = 9600;

	save_flags(flags);
	cli();
		
	err = request_irq(dev->irq, omap_irda_irq, 0, dev->name, dev);
	if (err)
		goto err_irq;

	/*
	 * The interrupt must remain disabled for now.
	 */
	
	disable_irq(dev->irq);

	restore_flags(flags);

	/* Alocate skb for receiver */

	err=omap_irda_rx_alloc(si);
	if (err)
	   goto err_rx_alloc;
	
	/*
	 * Setup the serial port for the specified config.
	 */

	err = omap_irda_startup(dev);
	if (err)
		goto err_startup;

	omap_irda_set_speed(dev,si->speed = 9600);

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
	enable_irq(dev->irq);
	netif_start_queue(dev);
	return 0;

err_irlap:
	si->open = 0;
	omap_sir_irda_shutdown(si);
err_startup:
	dev_kfree_skb(si->rxskb);
err_rx_alloc:	
err_irq:
	free_irq(dev->irq, dev);
	MOD_DEC_USE_COUNT;
	return err;
}

static int omap_irda_stop(struct net_device *dev)
{
	struct omap_irda *si = dev->priv;
	
	disable_irq(dev->irq);

  	omap_sir_irda_shutdown(si); 

	
	if (si->rxskb) {
	   dev_kfree_skb(si->rxskb);
	   si->rxskb = NULL;
	}

	/* Stop IrLAP */
	if (si->irlap) {
		irlap_close(si->irlap);
		si->irlap = NULL;
	}


	netif_stop_queue(dev);
	si->open = 0;

	/*
	 * Free resources
	 */
	free_irq(dev->irq, dev);

	MOD_DEC_USE_COUNT;

	return 0;
}

static int omap_irda_set_speed(struct net_device *dev, int speed)
{
   struct omap_irda *si = dev->priv;
   int divisor;
   u8  div16;

   __ECHO_IN;
   
   //   printk("OMAP IRDA set speed: %d \n",speed);

   /* Set LCR[7] to access DLH, DLL and DIV16 */

   outb(1 << 7, UART3_LCR);

   /* Set IRDA SIR speed */
    
   divisor = 12000000 / (16 * speed);
      
   if (speed == 115200)
     divisor = 1;

   outb((divisor & 0xFF), UART3_DLL);
    
   outb((divisor >> 8 ), UART3_DLH);    
   
   /* Set DIV_1.6 Value */
   
   switch (speed)
   {
      case 9600:   div16 = 0xD7;
	           break;
      case 19200:  div16 = 0x62; 
	           break;
      case 38400:  div16 = 0x27;
	           break;
      case 57600:  div16 = 0x14;
	           break;
      case 115200:
      default:     div16 = 0x01;
	           break; 
 } 
          
   outb(0, UART3_MDR2);   

   outb(div16, UART3_DIV16);  
  
   outb(0x03, UART3_LCR);

   outb((12 * speed) / 115200, UART3_EBLR); /* Set number of addition BOFS */

   

   if (speed == 115200) 
     outb(0x01, UART3_OSC_12M_SEL);
   else
     outb(0x00, UART3_OSC_12M_SEL);

   si->speed = speed;



   __ECHO_OUT;

   return 0;
}


static int omap_irda_net_init(struct net_device *dev)
{
	struct omap_irda *si = dev->priv;
	unsigned int baudrate_mask;
	int err = -ENOMEM;

	si = kmalloc(sizeof(struct omap_irda), GFP_KERNEL);
	if (!si)
		goto out;

	memset(si, 0, sizeof(*si));

	/*
	 * Initialise the HP-SIR buffers
	 */
	dev->priv = si;
	dev->hard_start_xmit	= omap_irda_hard_xmit;
	dev->open		= omap_irda_start;
	dev->stop		= omap_irda_stop;
	dev->do_ioctl		= omap_irda_ioctl;
	dev->get_stats		= omap_irda_stats;

	irda_device_setup(dev);
	irda_init_max_qos_capabilies(&si->qos);

	/*
	 * We support original IRDA up to 115k2. 
	 */

	baudrate_mask = IR_9600|IR_19200|IR_38400|IR_57600|IR_115200;
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
static void omap_irda_net_uninit(struct net_device *dev)
{
	struct omap_irda *si = dev->priv;

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
int __init omap_irda_init(void)
{
	struct net_device *dev;
	int err;

	/* OMAP SIR  net device register stage */
	
        rtnl_lock();
	dev = dev_alloc("irda%d", &err);
	if (dev) {
	        dev->irq    = INT_UART3;
		dev->init   = omap_irda_net_init;
		dev->uninit = omap_irda_net_uninit;

		err = register_netdevice(dev);

		if (err)
			kfree(dev);
		else
		{
			netdev = dev;
#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CCE */
                        irda_ldm_device_register();
                        irda_ldm_driver_register();
#endif /* MVL-CCE */                        
                } 
	}

	rtnl_unlock();
	
	return err;
}

static void __exit omap_irda_exit(void)
{
	struct net_device *dev = netdev;

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CCE */
        irda_ldm_device_unregister();
        irda_ldm_driver_unregister();
#endif /* MVL-CCE */ 

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
module_init(omap_irda_init);
module_exit(omap_irda_exit);
#endif

MODULE_AUTHOR("Alexey Lugovskoy");
MODULE_DESCRIPTION("OMAP Innovator SIR");
MODULE_LICENSE("GPL");
EXPORT_NO_SYMBOLS;
