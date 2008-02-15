/* @(#)27 1.6 linux405/drivers/net/npnet.c, linux, np 2/7/02 15:06:29 */

/*************************************************************************
 * (C) Copyright IBM Corp. 2001
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License (GPL) version 2.0
 * or (at your option) any later version.
 * 
 **************************************************************************/

#include <linux/module.h>
#include <linux/slab.h>        /* kmalloc() */
#include <linux/netdevice.h>   /* struct device, and other headers */
#include <linux/etherdevice.h> /* eth_type_trans */
#include <linux/ip.h>          /* struct iphdr */
#include <linux/init.h>
#ifndef CONFIG_RAINIER
#include <linux/pci.h>
#include <platforms/powerk2.h>
#endif /* CONFIG_RAINIER */

#include "npnet.h"

#include <asm/checksum.h>
#include <asm/io.h>

#undef NPNET_DEBUG

#ifdef NPNET_DEBUG
#define DBG(x...) printk(x)
#else /* NPNET_DEBUG */
#define DBG(x...)
#endif /* NPNET_DEBUG */

MODULE_AUTHOR("John F. Davis");
MODULE_LICENSE("GPL");
/* This code was modeled after Alessandro Rubini's  */
/* Linux Device Driver book. */

/////////////////////////////////////////////////////
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
// #DEFINES
/////////////////////////////////////////////////////

#define NP_NET_VERSION		"1.0"
#define NP_PAGE			0x1004		/*4100 bytes */
#define MB_405			0x00000018
#define H2P_MSG_ADDR_ALT	0x00000050
#define H2P_MSG_ADDR_PRI	0x00000060
#define NP_INT_ENB		0x00000008
#define P2H_MSG			1		/*enable P2H message interrupt*/
#define ENV_405			(ENV_750 + NP_PAGE)
#define ENV_750			0x00000000
#define MAC_405			"\0\0\4\0\5\0"
#define MAC_750			"\0\0\7\5\0\0"

#ifdef CONFIG_RAINIER
#define MB_BASE_ADDR		0x78010000
#define ENV_BASE_ADDR		0x03F00000
#define MYENVELOPE		ENV_405
#define HISENVELOPE		ENV_750
#define IRQ			4
#define MY_MAC			MAC_405
#define HIS_MAC			MAC_750
#else
#define MB_BASE_ADDR		rainier_mb_base_addr
#define ENV_BASE_ADDR		rainier_env_base_addr
#define MYENVELOPE		ENV_750
#define HISENVELOPE		ENV_405
#define IRQ			11
#define MY_MAC			MAC_750
#define HIS_MAC			MAC_405
#endif

#define TX_TIMEOUT		(1*HZ)            /* In jiffies */
#define TX_COUNT		100

/////////////////////////////////////////////////////
// Types
/////////////////////////////////////////////////////

struct shared_buffer {
	volatile int iBusy;	/* If 1, its busy.  */
	volatile int iSize;	/* If 1, its valid. */
	u8 pData[2048];		/* Was 4096 */
};

/*
 * This structure is private to each device. It is used to pass
 * packets in and out, so there is place for a packet
 */

struct npnet_priv {
    struct net_device_stats stats;
    int status;
    int tx_packetlen;
    u8 *tx_packetdata;
    struct sk_buff *skb;
    spinlock_t lock;
};

/////////////////////////////////////////////////////
// Prototypes
/////////////////////////////////////////////////////
static int npnet_init(struct net_device *dev);
static void npnet_int_handler(int irq, void *dev_id, struct pt_regs *regs);
static int npnet_header(struct sk_buff *skb, struct net_device *dev,
                 unsigned short type, void *daddr, void *saddr, 
                 unsigned len);

/////////////////////////////////////////////////////
// Globals
/////////////////////////////////////////////////////
static volatile void *pvMailboxPage;
static volatile void *pvEnvelopePage;

/*
 * The devices
 */
static struct net_device npnet_devs= {
    init: npnet_init  /* init, nothing more */
};

/////////////////////////////////////////////////////
// Notes
/////////////////////////////////////////////////////

void DumpSkbStruct(struct sk_buff *skb) {
	int i;
	printk("Dump sk_buff \n");
	printk("skb addy %p\n",skb);

	printk( KERN_ERR "skb: len is %i\n",skb->len);
	printk( KERN_ERR "skb : data:\n");
	for (i=0 ; i<skb->len; i++) {
		printk(" %02x",skb->data[i]&0xff);
	}
	printk("\n");
	printk("net_device pointer addy %p\n",skb->dev);
	printk("    Dump sk_buff END\n");
}

void DumpDevStruct(struct net_device *dev) {

	int i;

	printk("Dump net_device\n");
	printk("dev addy %p\n",dev);

	printk("priv pointer addy = %p\n",dev->priv);

	printk("dev_addr: (len = %d)\n",dev->addr_len);
	for (i=0 ; i<dev->addr_len; i++) {
		printk(" %02x",dev->dev_addr[i]);
	}
	printk("\n");

	printk("    Dump net_device END\n");
}

/*********************************************************************
 *
 * The Prototypes.
 *
 *
 *
 ********************************************************************/

/*
 * Receive a packet: retrieve, encapsulate and pass over to upper levels
 */

void npnet_rx(struct net_device *dev, int len, unsigned char *buf)
{
    struct sk_buff *skb;
    struct npnet_priv *privp = (struct npnet_priv *)dev->priv;

    /*
     * The packet has been retrieved from the transmission
     * medium. Build an skb around it, so upper layers can handle it
     */
    skb = dev_alloc_skb(len+2);
    if (!skb) {
        printk("snull rx: low on mem - packet dropped\n");
        privp->stats.rx_dropped++;
        return;
    }
    skb_reserve(skb, 2); /* align IP on 16B boundary */  
    memcpy(skb_put(skb, len), buf, len);

    /* Write metadata, and then pass to the receive level */
    skb->dev = dev;
    skb->protocol = eth_type_trans(skb, dev); /* this strips off the mac. */

    skb->ip_summed = CHECKSUM_UNNECESSARY; /* don't check it */
    skb->pkt_type = PACKET_HOST;

    netif_rx(skb);
    return;
}

/*********************************************************************
 *
 * The Interrupt Handler.
 *
 *
 ********************************************************************/
void npnet_int_handler(int irq, void *dev_id, struct pt_regs *regs)
{
    volatile unsigned *pvBuffer;
    volatile int iSize;
#ifndef CONFIG_RAINIER
    volatile int iStatus;
#endif
    volatile struct shared_buffer *pSharedBuffer;
    struct net_device *dev = (struct net_device *)dev_id;

    /* Lock the device */
    /*privptr = (struct npnet_priv *)(dev->priv); */
    /*nospin spin_lock(&privptr->lock); */
    
#ifndef CONFIG_RAINIER
    /* this is a hack to see if the int status has been cleared. */
    pvBuffer = pvMailboxPage + 0; /* Read the status register. */
    iStatus = *pvBuffer;
    if ((iStatus & P2H_MSG) == 0) {
        /* We have already service this interrupt. */
        /* The hardware just hasn't cleared it yet. */
        DBG(KERN_ERR "The hw hasn't cleared the interrupt yet.\n");
        goto out;
    }
#endif

    /* So, we have a valid interrupt which we haven't cleared yet.
     * Maybe we need to do this last!!!!!
     * This read will __eventually__ reset the msg busy bit in the status
     * register. 
     */

#ifdef CONFIG_RAINIER
    pvBuffer = pvMailboxPage + H2P_MSG_ADDR_PRI;
#else
    pvBuffer = pvMailboxPage + MB_405;
#endif
    iSize = *pvBuffer;

    /* Determine if this is a 750 interupt when it initializes. */
    if (iSize == 0) {
	/* This looks like a interupt generated when the 750 is initializing.*/
	DBG("This is just a 750 initializing interrupt..\n");
	goto out;
    }
    /* Verify that we have a good buffer. */
    pSharedBuffer = pvEnvelopePage + HISENVELOPE;
    if (iSize != pSharedBuffer->iSize) {
	/* Size Mismatch. */
	printk("int handler size mismatch iS= %d pS= %d\n", iSize,pSharedBuffer->iSize);
	goto out;
    } 
    /* Save it to the device. */
    npnet_rx(dev, iSize, (unsigned char*)pSharedBuffer->pData);

    /* We have the data free the buffer.. */
    pSharedBuffer->iBusy = 0;

 out:
    /* Unlock the device and we are done */
    /*nospin    spin_unlock(&privptr->lock); */
    return;
}

/*********************************************************************
 * 
 * Open
 * This is called when you do:
 * # ifconfig np0 local0
 *   
 ********************************************************************/
int npnet_open(struct net_device *dev)
{

    MOD_INC_USE_COUNT;
    
    /* 
     * Assign the hardware address of the board: use "\0SNULx", where
     * x is 0 or 1. The first byte is '\0' to avoid being a multicast
     * address (the first byte of multicast addrs is odd).
     */

    memcpy(dev->dev_addr, MY_MAC, ETH_ALEN);  /* ETH_ALEN = 6; */
    netif_start_queue(dev);

    return 0;
}

/*********************************************************************
 * 
 * Release
 * This is called when you do:
 * # ifconfig np0 down
 *   
 ********************************************************************/
int npnet_release(struct net_device *dev)
{

    /* release ports, irq and such -- like fops->close */

    /* Stop the queue when closed. */
    netif_stop_queue(dev); /* can't transmit any more */

    MOD_DEC_USE_COUNT;

    return 0;
}

/*********************************************************************
 * 
 * Transmit a packet (low level interface)
 * The routine is called after the tx routine.
 *  
 *   
 ********************************************************************/

static struct timer_list timer;

void npnet_hw_tx(unsigned long idev)
{
    /*
     * This function deals with hw details. 
     * This function implements the npnet behaviour,
     * while all other procedures are rather device-independent
     */

    struct iphdr *ih;
    struct net_device *dev = (struct net_device*) idev;
    struct npnet_priv *priv = (struct npnet_priv *)dev->priv;
    volatile struct shared_buffer *pSharedBuffer;
    volatile unsigned *pvBuffer;
    static int iLoop = 0;
    struct timer_list *ptimer = &timer;
    char * buf;
    int len;

    len = priv->tx_packetlen;
    buf = priv->tx_packetdata;
    /*
     * Ethhdr is 14 bytes, but the kernel arranges for iphdr
     * to be aligned (i.e., ethhdr is unaligned)
     */
    ih = (struct iphdr *)(buf+sizeof(struct ethhdr));
    ih->check = 0;         /* and rebuild the checksum (ip needs it) */
    ih->check = ip_fast_csum((unsigned char *)ih,ih->ihl);

    /* Hardware says we can do a write.  Now, we need to check to  */
    /*  make sure we don't use a buffer that's in use. */
    pSharedBuffer = pvEnvelopePage + MYENVELOPE;
    if (pSharedBuffer->iBusy) {
        if (++iLoop < TX_COUNT){
	    ptimer->expires = jiffies;
	    ptimer->data = (unsigned long) dev;
	    ptimer->function = npnet_hw_tx;
	    init_timer (ptimer);
	    add_timer(ptimer);
	    /* printk("We did not restart the queue in tx.\n"); */
	    return ;
	}
        else{
	    DBG("We looped more than TX_COUNT times\n");
	    iLoop = 0;
	    return;
	}
    }
    iLoop = 0;
    pSharedBuffer->iBusy = 1;  /* Take the buffer so its no longer free. */
    /* Place in data in D6 buffer. */
    memcpy((char *)pSharedBuffer->pData,buf,len);
    /* Complete header in D6 buffer. */
    pSharedBuffer->iSize = len;

    /* Place in mailbox. */
#ifdef CONFIG_RAINIER
    pvBuffer = pvMailboxPage + MB_405;
#else
    pvBuffer = pvMailboxPage + H2P_MSG_ADDR_PRI;
#endif
    *pvBuffer = len;

    dev_kfree_skb(priv->skb);        
    priv->skb = NULL;
    priv->stats.tx_packets++;
    priv->stats.tx_bytes += len;
    netif_wake_queue(dev);
    return;
}

/*********************************************************************
 * 
 * Transmit a packet 
 *   
 ********************************************************************/
int npnet_xmit(struct sk_buff *skb, struct net_device *dev)
{
    int len;
    struct npnet_priv *priv = (struct npnet_priv *)dev->priv;

    /* Lock the device */
    /*nospin spin_lock(&priv->lock); */

    /* Stop the queue */
    netif_stop_queue(dev);        

    len = skb->len < ETH_ZLEN ? ETH_ZLEN : skb->len;
    dev->trans_start = jiffies; /* save the timestamp */

    /* Remember the skb, so we can free it at interrupt time */
    priv->skb = skb;
    priv->tx_packetdata= skb->data;
    priv->tx_packetlen = len;
    npnet_hw_tx((int)dev);

    /* Unlock the device and we are done */
    /*nospin spin_unlock(&priv->lock); */
    return 0;
}

/*********************************************************************
 * 
 * Return statistics to the caller
 *  
 ********************************************************************/
struct net_device_stats *npnet_stats(struct net_device *dev)
{
    struct npnet_priv *priv = (struct npnet_priv *)dev->priv;
    return &priv->stats;
}

/*********************************************************************
 *
 * This function is called to fill up an eth header, since arp is not
 * available on the interface
 *  
 * This doesn't seem to be called.
 *  
 ********************************************************************/
int npnet_rebuild_header(struct sk_buff *skb)
{
    struct ethhdr *eth = (struct ethhdr *) skb->data;
    struct net_device *dev = skb->dev;

    memcpy(eth->h_source, dev->dev_addr, dev->addr_len);
    memcpy(eth->h_dest, dev->dev_addr, dev->addr_len);
    eth->h_dest[ETH_ALEN-1]   ^= 0x01;   /* dest is us xor 1 */
    return 0;
}

/*********************************************************************
 * 
 * I think I can do without this function.  I just keep it around
 * so that I can see the call.
 *   
 ********************************************************************/
int npnet_header(struct sk_buff *skb, struct net_device *dev,
                 unsigned short type, void *daddr, void *saddr, 
                 unsigned len)
{

	struct ethhdr *eth = (struct ethhdr *) skb_push(skb,ETH_HLEN);

	eth->h_proto = htons(type);

	memcpy(eth->h_source,MY_MAC,6);
	memcpy(eth->h_dest,HIS_MAC,6);
	return (dev->hard_header_len);
}

/*********************************************************************
 *   
 ********************************************************************/
void npnet_tx_timeout(struct net_device *dev) {

    struct npnet_priv *priv = (struct npnet_priv *)dev->priv;

    DBG(KERN_NOTICE "***npnet_tx_timeout ***\n");

    /* Lock the device */
    /*nospin spin_lock(&priv->lock); */

    priv->stats.tx_errors++;

    /* Unlock the device and we are done */
    /*nospin spin_unlock(&priv->lock); */

    /* Wake the queue */
    netif_wake_queue(dev);
}

/*********************************************************************
 * 
 * The init function (sometimes called probe).
 * It is invoked by register_netdev()
 *  
 *   
 ********************************************************************/
int npnet_init(struct net_device *dev)
{

    /*
     * Make the usual checks: check_region(), probe irq, ...  -ENODEV
     * should be returned if no device found.  No resource should be
     * grabbed: this is done on open(). 
     */

	/* 
	 * Then, assign other fields in dev, using ether_setup() and some
	 * hand assignments
	 */
	ether_setup(dev); /* assign some of the fields */

	dev->open            = npnet_open;
	dev->stop            = npnet_release;
	dev->hard_start_xmit = npnet_xmit;
	dev->hard_header     = npnet_header;
	dev->rebuild_header  = npnet_rebuild_header;
	dev->tx_timeout = npnet_tx_timeout;
	dev->watchdog_timeo = TX_TIMEOUT;     
	dev->get_stats       = npnet_stats;
	dev->flags           |= IFF_NOARP;
	dev->hard_header_cache = NULL;      /* Disable caching */
	/*
	 * Then, allocate the priv field. This encloses the statistics
	 * and a few private fields.
	 */
	dev->priv = kmalloc(sizeof(struct npnet_priv), GFP_KERNEL);
	if (dev->priv == NULL) {
		printk("kmalloc in init failed.\n");
		return -ENOMEM;
	}
	memset(dev->priv, 0, sizeof(struct npnet_priv));

	/*nospin spin_lock_init(& ((struct npnet_priv *) dev->priv)->lock); */

	/* Register the interrupt */
	request_irq(IRQ,npnet_int_handler,SA_INTERRUPT,"np_net_int",dev);
	return 0;
}

/*********************************************************************
 * 
 * This is called when you do:
 * # insmod npnet
 *  
 *   
 ********************************************************************/
static int __init npnet_init_module(void) {

    int result, device_present = 0;
    volatile unsigned *pvMailboxInt;
    volatile int dontcare;
    volatile struct shared_buffer *pSharedBuffer;
#ifndef CONFIG_RAINIER
    unsigned int rainier_mb_base_addr = 0;
    unsigned int rainier_env_base_addr = 0;
    struct pci_dev *dev = NULL;
    unsigned int rainier_board_present[2];

    while ((dev =
           pci_find_device(PCI_VENDOR_ID_IBM, PCI_DEVICE_ID_IBM_NP4GS3, dev))) {
        /* Identify and Count NP4GS3 carrier cards */
        if (dev->bus->number == 1 && ( PCI_SLOT(dev->devfn)==NP1)) {
	    DBG(KERN_INFO "Found NP4GS3 in slot 1\n");
	    rainier_board_present[0] = 1;
        } else if (dev->bus->number == 1 && ( PCI_SLOT(dev->devfn)==NP2)) {
	    DBG(KERN_INFO "Found NP4GS3 in slot 2\n");
	    rainier_board_present[1] = 1;
        }
    }

    if (!rainier_board_present[0] && !rainier_board_present[1]) {
      printk(KERN_ERR "NPNet found no NP4GS3 carrier cards\n");
      return -ENODEV;
    } else if ( !rainier_board_present[0] && rainier_board_present[1] ) {
      /* We have a board only in slot 2, so configure npnet to talk to it */
      rainier_mb_base_addr = 0xA8010000;
      rainier_env_base_addr = 0xA3F00000;
    } else {
      /* We either have a single board in slot 1, or two boards installed,
         so configure npnet to talk to the board in slot 1 */
      rainier_mb_base_addr = 0xB8010000;
      rainier_env_base_addr = 0xB3F00000;
    }
#endif

    strcpy(npnet_devs.name, "np0");
    printk(KERN_INFO "NP Net memory driver v" NP_NET_VERSION "\n");

    /* Remap the memory for the MB and Envelope pages. */
    pvMailboxPage =  ioremap(MB_BASE_ADDR,0x256); 
    if (pvMailboxPage == NULL) {
		printk(KERN_ERR "Can not ioremap() mailbox\n");
    }
    pvEnvelopePage = ioremap(ENV_BASE_ADDR,(NP_PAGE*2)); 
    if (pvEnvelopePage == NULL) {
		printk(KERN_ERR "Can not ioremap() envelope\n");
    }
    DBG("npnet: pvMailboxPage = %p\n", pvMailboxPage);
    DBG("npnet: pvEnvelopePage = %p\n",  pvEnvelopePage);

    /* Clear the incoming mailbox so we don't get */
    /* and interrupt as soon as we register the handler. */
    dontcare = *(volatile unsigned *) (pvMailboxPage + H2P_MSG_ADDR_PRI);

    result = register_netdev(&npnet_devs);
    if ( result ) { 
	printk("npnet: error %i registering device \"%s\"\n",
			   result, npnet_devs.name);
    } else {
	device_present++;
    }

    /* Clear the shared area (D6).  Do this first so that when
     * we clear the outgoing mailbox, the other end will be interrupted
     * but we will have a clear envelope area. (Busy flag cleared also.)
     */
    pSharedBuffer = pvEnvelopePage + MYENVELOPE;
    memset((void*)pSharedBuffer,0,sizeof(struct shared_buffer));
#if 0 /* BMS only if necessary */
    pSharedBuffer = pvEnvelopePage+HISENVELOPE;/*BMS only if necessary*/ 
    pSharedBuffer->iBusy = 0;			 /*BMS only if necessary*/
#endif /* 0 */

    /* Clear the outgoing mailbox. This will generate a interrupt to far end.*/
#ifdef CONFIG_RAINIER
    *(volatile unsigned *)(pvMailboxPage + MB_405) = 0;
#else
    *(volatile unsigned*)(pvMailboxPage+H2P_MSG_ADDR_PRI) = 0x489;
#endif

    /* Enable interrupts  to K2 in hardware. */
    pvMailboxInt = pvMailboxPage + NP_INT_ENB;
    dontcare = *pvMailboxInt;
    *pvMailboxInt = dontcare | P2H_MSG;

#ifndef NPNET_DEBUG
    EXPORT_NO_SYMBOLS;
#endif
    return device_present ? 0 : -ENODEV;
}

/*********************************************************************
 * 
 * This is called when you do:
 * # rmmod npnet
 *  
 *   
 ********************************************************************/
static void __exit npnet_cleanup(void)
{
    free_irq(IRQ,&npnet_devs);
    kfree(npnet_devs.priv);
    unregister_netdev(&npnet_devs);
    return;
}

module_init(npnet_init_module);
module_exit(npnet_cleanup);
