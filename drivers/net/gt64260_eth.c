/* Driver for EVB64260 ethernet ports
   Copyright (C)2000, 2001 Rabeeh Khoury, Marvell */

/*
 * drivers/net/gt64260_eth.c
 * 
 * Ethernet driver for the Marvell/Galileo GT64260 (Discovery) chip,
 *
 * Author: Rabeeh Khoury from Marvell
 * Heavily Modified by: Mark A. Greer <mgreer@mvista.com> and 
 *                      Troy Benjegerdes <tbenjegerdes@mvista.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
/*
 * This is an early version of an ethernet driver for the gt64260.
 * If you make any improvements to this driver please forward them to:
 * source@mvista.com
 *
 * In reality, a rewrite may be more appropriate.
 */

#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/sched.h>
#include <linux/ptrace.h>
#include <linux/fcntl.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/ip.h>
#include <linux/mii.h>

#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/types.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/ppcboot.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/skbuff.h>
#include <linux/ctype.h>

#ifdef CONFIG_NET_FASTROUTE
#include <linux/if_arp.h>
#include <net/ip.h>
#endif

#include <asm/gt64260.h>
#include "gt64260_eth.h"

extern bd_t ppcboot_bd;
extern int ppcboot_bd_valid;

#define BIT(n) (1<<(n))

static unsigned char GT64260_ETH_irq[3] = { 32, 33, 34 };

static const char *version = "gt64260_eth.c: v0.09";

static unsigned char gt64260_mac_addrs[3][12];	/* Will be init'd to 0's */
static unsigned char null_mac[6];		/* Will be init'd to 0's */

/* set port config value */

static inline void
set_port_config(u32 value, unsigned int port)
{
	gt_write(GT64260_ENET_E0PCR + ETHERNET_PORTS_DIFFERENCE_OFFSETS * port,
			value);
}

/* get port config value */

inline u32
get_port_config(unsigned int port)
{
	return gt_read(GT64260_ENET_E0PCR + ETHERNET_PORTS_DIFFERENCE_OFFSETS * port);
}

/*
 * Set Promiscuous mode to normal mode.
 * in order to enable Filtering.     XXX this should probably go away
 */
static inline void
enableFiltering(u32 port)
{
	ETHERNET_PCR portControlReg;

	portControlReg = get_port_config(port);
	portControlReg &= ~(1 << PROMISCUOUS_MODE);
	set_port_config(portControlReg, port);
}

/*
 * This function will set the Promiscuous mode to Promiscuous mode.
 * in order to disable Filtering.    XXX this should probably go away
 */
static inline void
disableFiltering(u32 port)
{
	ETHERNET_PCR portControlReg;

	portControlReg = get_port_config(port);
	portControlReg |= (1 << PROMISCUOUS_MODE);
	set_port_config(portControlReg, port);
}




/*
 * ----------------------------------------------------------------------------
 * Allocate the specified number of pages and mark them uncached.
 * ASSUME: kmalloc() supplies page-aligned memory
 *         mapped to contiguous physical pages
 */
u32
uncachedPages(u32 pages)
{
	pte_t *pte;
	u32 addr;
	u32 firstPage;

	firstPage = addr = (u32) kmalloc((pages * PAGE_SIZE), GFP_KERNEL);
	if (!addr || (addr & ~PAGE_MASK)) {
		panic("uncachedPages: can't get page-aligned memory.\n");
	}
	while (pages--) {
		pte = va_to_pte(addr);
		pte_val(*pte) |= (_PAGE_NO_CACHE | _PAGE_GUARDED);
		flush_tlb_page(init_mm.mmap, addr);
		invalidate_dcache_range(addr, addr + PAGE_SIZE);
		addr += PAGE_SIZE;
	}
	mb();
	return (firstPage);
}

#ifdef PHY_LXT97x
static int
dump_intel_phy_state(struct net_device *dev)
{
	unsigned int mii_11 = 0;
	gt_eth_priv *private = (gt_eth_priv *) dev->priv;

#if 1
	etherReadMIIReg(private->port, 0x11, &mii_11);
	printk(" mii:%s:%s:%s:%s %s:%s\n",
	       mii_11 & (1 << 14) ? "100" : " 10",
	       mii_11 & (1 << 10) ? " Link" : "nLink",
	       mii_11 & (1 << 9) ? "FD" : "HD",
	       mii_11 & (1 << 4) ? " FC" : "nFC",
	       mii_11 & (1 << 7) ? "ANc" : "ANnc",
	       mii_11 & (1 << 8) ? "AN" : "Manual");
#else
	int i;
	for (i = 0; i < 0x15; i++) {
		etherReadMIIReg(private->port, i, &mii_11);
		printk("    mii %2d 0x%4x", i, mii_11);
		if ((i % 4) == 3)
			printk("\n");
	}
	printk("\n");
#endif
	return 0;
}
#endif

/* Sometimes it seems the phy and the GT don't always agree. 
 * Make sure they do, or restart autoneg. */
static void
check_phy_state(struct net_device *dev)
{
	gt_eth_priv *private = (gt_eth_priv *) dev->priv;
	struct mii_if_info *mii = &private->mii_if;
	int bmsr = mii->mdio_read(mii->dev, mii->phy_id, MII_BMSR);

	if (mii_link_ok(&private->mii_if) && (bmsr & BMSR_ANEGCOMPLETE)) {
		int advert = mii->mdio_read(dev, mii->phy_id, MII_ADVERTISE);
		int lpa = mii->mdio_read(dev, mii->phy_id, MII_LPA);
		int nego = mii_nway_result(advert & lpa);
		int psr, wanted;

		switch (nego) {
		case LPA_100FULL:
			wanted = 0x3;
			break;
		case LPA_100HALF:
			wanted = 0x1;
			break;
		case LPA_10FULL:
			wanted = 0x2;
			break;
		case LPA_10HALF:
			wanted = 0x0;
			break;
		default:
			printk("%s: MII negotiated strange settings %d\n",
			       dev->name, nego);
			break;
		}
		psr =
		    gt_read(GT64260_ENET_E0PSR +
			    (ETH_ADDR_GAP * private->port));

		if ((psr & 0x3) != wanted) {
			printk
			    ("%s: MII said %x, GT said %x, restarting autoneg\n",
			     dev->name, wanted, psr);
			mii_nway_restart(mii);
		}
	}
}

static void
dump_link_state(struct net_device *dev)
{
	gt_eth_priv *private = (gt_eth_priv *) dev->priv;
	unsigned int psr;

	psr = gt_read(GT64260_ENET_E0PSR + (ETH_ADDR_GAP * private->port));
	printk("%s: link state:\n"
	       "  GT:%s:%s:%s:%s\n",
	       dev->name,
	       psr & 1 ? "100" : " 10",
	       psr & 8 ? " Link" : "nLink",
	       psr & 2 ? "FD" : "HD", psr & 4 ? " FC" : "nFC");

#ifdef PHY_LXT97x
	dump_intel_phy_state(dev);
#endif
}

/*
 * ----------------------------------------------------------------------------
 *  Create an addressTable entry from MAC address info
 *  found in the specifed net_device struct
 *
 *  Input : pointer to ethernet interface network device structure
 *  Output : N/A
 */
void
gt64260_eth_update_mac_address(struct net_device *dev)
{
	u32 macH;
	u32 macL;
	u8 *byte;

	byte = dev->dev_addr;
	macH = byte[0];
	macH = (macH << 8) | byte[1];
	macL = byte[2];
	macL = (macL << 8) | byte[3];
	macL = (macL << 8) | byte[4];
	macL = (macL << 8) | byte[5];

	/*
	 * addAddressTableEntry() will flush Dcache and sync
	 */
	addAddressTableEntry(((gt_eth_priv *) (dev->priv))->port, macH, macL, 1,
			     0);
}

/*
 * ----------------------------------------------------------------------------
 * Set the MAC address for the specified interface
 * to the specified value, forsaking all others.
 *
 * No special hardware thing should be done because  XXX_MIKE - old commentary?
 * interface is always put in promiscuous mode.      XXX_MIKE - old commentary?
 *
 * Input : pointer to ethernet interface network device structure and
 *         a pointer to the designated entry to be added to the cache.
 * Output : zero upon success, negative upon failure
 */
s32
gt64260_eth_set_mac_address(struct net_device *dev, void *addr)
{
	u32 i;
	struct sockaddr *sock;

	sock = (struct sockaddr *) addr;
	for (i = 0; i < 6; i++) {
		dev->dev_addr[i] = sock->sa_data[i];
	}

	addressTableClear(((gt_eth_priv *) (dev->priv))->port);	/* Does flush */
	gt64260_eth_update_mac_address(dev);
	return (0);
}

/*
 * ----------------------------------------------------------------------------
 *  Update the statistics structure in the private data structure
 *
 *  Input : pointer to ethernet interface network device structure
 *  Output : N/A
 */
void
gt64260_eth_update_stat(struct net_device *dev)
{
    gt_eth_priv             *private;
    struct net_device_stats *stat;
    u32                     base;
    static int              first_time[3] = {1, 1, 1};

    private =   dev->priv;
    stat    = &(private->stat);
    base    =   GT64260_ENET_0_MIB_CTRS + (private->port * ETH_ADDR_GAP);

    if (first_time[private->port]) {
           stat->rx_bytes = 0;
           stat->tx_bytes = 0;
           stat->rx_packets = 0;
           stat->tx_packets = 0;
           stat->rx_errors = 0;
           stat->rx_dropped = 0;
           stat->multicast = 0;
           stat->collisions = 0;
           stat->rx_length_errors = 0;
           stat->rx_length_errors = 0;
           stat->rx_crc_errors = 0;

           /* Clear chips MIB counters */
           gt_read( base + 0x00 );
           gt_read( base + 0x04 );
           gt_read( base + 0x08 );
           gt_read( base + 0x0c );
           gt_read( base + 0x50 );
           gt_read( base + 0x20 );
           gt_read( base + 0x1c );
           gt_read( base + 0x30 );
           gt_read( base + 0x60 );
           gt_read( base + 0x24 );
           gt_read( base + 0x20 );

           first_time[private->port] = 0;
    }

    stat->rx_bytes         += gt_read( base + 0x00 );
    stat->tx_bytes         += gt_read( base + 0x04 );
    stat->rx_packets       += gt_read( base + 0x08 );
    stat->tx_packets       += gt_read( base + 0x0c );
    stat->rx_errors        += gt_read( base + 0x50 );

    /*
     * Rx dropped is for received packet with CRC error
     */
    stat->rx_dropped       += gt_read( base + 0x20 );
    stat->multicast        += gt_read( base + 0x1c );
    stat->collisions       += gt_read( base + 0x30 );

    /*
     * detailed rx errors
     */
    stat->rx_length_errors += gt_read( base + 0x60 );
    stat->rx_length_errors += gt_read( base + 0x24 );
    stat->rx_crc_errors    += gt_read( base + 0x20 );

    /*
     * detailed tx errors - XXX_MIKE - INCOMPLETE IMPLEMENTATION ?
     */
}

/*
 * ----------------------------------------------------------------------------
 * Returns a pointer to the interface statistics.
 *
 * Input : dev - a pointer to the required interface
 *
 * Output : a pointer to the interface's statistics
 */
struct net_device_stats *
gt64260_eth_get_stats(struct net_device *dev)
{
	gt_eth_priv *private;

	gt64260_eth_update_stat(dev);
	private = dev->priv;
	return (&(private->stat));
}

/*
 * ----------------------------------------------------------------------------
 *  change rx mode
 *
 *  Input : pointer to ethernet interface network device structure
 *  Output : N/A
 */
void
gt64260_eth_set_rx_mode(struct net_device *dev)
{
	gt_eth_priv *private;

	private = dev->priv;
	if (dev->flags & IFF_PROMISC) {
		disableFiltering(private->port);
	} else {
		enableFiltering(private->port);
	}
}

#ifdef CONFIG_NET_FASTROUTE	/* XXX_MIKE - ??? */
/*
 * ----------------------------------------------------------------------------
 * Used to authenticate to the kernel that a fast path entry can be
 * added to device's routing table cache
 *
 * Input : pointer to ethernet interface network device structure and
 *         a pointer to the designated entry to be added to the cache.
 * Output : zero upon success, negative upon failure
 */
static s32
gt64260_eth_accept_fastpath(struct net_device *dev, struct dst_entry *dst)
{
	struct net_device *odev = dst->dev;

	if ((dst->ops->protocol != __constant_htons(ETH_P_IP))
	    || (odev->type != ARPHRD_ETHER)
	    || !odev->accept_fastpath) {
		return (-1);
	}
	return (0);
}
#endif				/* #ifdef CONFIG_NET_FASTROUTE */

static int
mdio_read(struct net_device *dev, int phy_id, int location)
{
	unsigned int val;

	etherReadMIIReg(phy_id & 0x1f, location & 0x1f, &val);

	return (val & 0xffff);
}

static void
mdio_write(struct net_device *dev, int phy_id, int location, int value)
{
	etherWriteMIIReg(phy_id & 0x1f, location & 0x1f, value);
}

static int
netdev_ethtool_ioctl(struct net_device *dev, void *useraddr)
{
	gt_eth_priv *private = (gt_eth_priv *) dev->priv;
	u32 ethcmd;

	if (copy_from_user(&ethcmd, useraddr, sizeof (ethcmd)))
		return -EFAULT;

	switch (ethcmd) {

		/* Get driver info */
	case ETHTOOL_GDRVINFO:{
			struct ethtool_drvinfo info = { ETHTOOL_GDRVINFO };
			strncpy(info.driver, "gt64260",
				sizeof (info.driver) - 1);
			strncpy(info.version, version,
				sizeof (info.version) - 1);
			if (copy_to_user(useraddr, &info, sizeof (info)))
				return -EFAULT;
			return 0;
		}
		/* get settings */
	case ETHTOOL_GSET:{
			struct ethtool_cmd ecmd = { ETHTOOL_GSET };
			spin_lock_irq(&private->lock);
			mii_ethtool_gset(&private->mii_if, &ecmd);
			spin_unlock_irq(&private->lock);
			if (copy_to_user(useraddr, &ecmd, sizeof (ecmd)))
				return -EFAULT;
			return 0;
		}
		/* set settings */
	case ETHTOOL_SSET:{
			int r;
			struct ethtool_cmd ecmd;
			if (copy_from_user(&ecmd, useraddr, sizeof (ecmd)))
				return -EFAULT;
			spin_lock_irq(&private->lock);
			r = mii_ethtool_sset(&private->mii_if, &ecmd);
			spin_unlock_irq(&private->lock);
			return r;
		}
		/* restart autonegotiation */
	case ETHTOOL_NWAY_RST:{
			return mii_nway_restart(&private->mii_if);
		}
		/* get link status */
	case ETHTOOL_GLINK:{
			struct ethtool_value edata = { ETHTOOL_GLINK };
			edata.data = mii_link_ok(&private->mii_if);
			if (copy_to_user(useraddr, &edata, sizeof (edata)))
				return -EFAULT;
			return 0;
		}
		/* get message-level */
	case ETHTOOL_GMSGLVL:{
			struct ethtool_value edata = { ETHTOOL_GMSGLVL };
			edata.data = 0;	/* XXX */
			if (copy_to_user(useraddr, &edata, sizeof (edata)))
				return -EFAULT;
			return 0;
		}
		/* set message-level */
	case ETHTOOL_SMSGLVL:{
			struct ethtool_value edata;
			if (copy_from_user(&edata, useraddr, sizeof (edata)))
				return -EFAULT;
/* debug = edata.data; *//* XXX */
			return 0;
		}
	}
	return -EOPNOTSUPP;
}

static int
gt64260_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct mii_ioctl_data *data = (struct mii_ioctl_data *) &rq->ifr_data;
	int phy = dev->base_addr & 0x1f;
	int retval;

	switch (cmd) {
	case SIOCETHTOOL:
		retval = netdev_ethtool_ioctl(dev, (void *) rq->ifr_data);
		break;

	case SIOCGMIIPHY:	/* Get address of MII PHY in use. */
	case SIOCDEVPRIVATE:	/* for binary compat, remove in 2.5 */
		data->phy_id = phy;
		/* Fall through */

	case SIOCGMIIREG:	/* Read MII PHY register. */
	case SIOCDEVPRIVATE + 1:	/* for binary compat, remove in 2.5 */
		data->val_out =
		    mdio_read(dev, data->phy_id & 0x1f, data->reg_num & 0x1f);
		retval = 0;
		break;

	case SIOCSMIIREG:	/* Write MII PHY register. */
	case SIOCDEVPRIVATE + 2:	/* for binary compat, remove in 2.5 */
		if (!capable(CAP_NET_ADMIN)) {
			retval = -EPERM;
		} else {
			mdio_write(dev, data->phy_id & 0x1f,
				   data->reg_num & 0x1f, data->val_in);
			retval = 0;
		}
		break;

	default:
		retval = -EOPNOTSUPP;
		break;
	}
	return retval;
}

/*
 * ----------------------------------------------------------------------------
 * Initializes the ethernet interface's private structure.
 * Statistics, descriptors, etc...
 *
 * Input : pointer to network device structure to be filled
 * Output : N/A
 */
void
gt64260_eth_init_priv(struct net_device *dev)
{
	gt_eth_priv *private;
	u32 queue;

	private = (gt_eth_priv *) kmalloc(sizeof (*private), GFP_KERNEL);
	if (!private) {
		panic("gt64260_eth_init_priv : kmalloc1 failed\n");
	}
	dev->priv = (void *) private;
	memset(private, 0, sizeof (*(private)));

	/*
	 * The GT64260 Enet engine accesses its queues of DMA
	 * descriptors in uncached mode, so we dedicate a page
	 * for each queue and mark each such page uncached.
	 */
	for (queue = 0; queue < NUM_TX_QUEUES; queue++) {
		private->TXqueue[queue] = (gt_dma_desc *) uncachedPages(1);
	}

	for (queue = 0; queue < NUM_RX_QUEUES; queue++) {
		private->RXqueue[queue] = (gt_dma_desc *) uncachedPages(1);
	}

	private->mii_if.dev = dev;
	private->mii_if.phy_id = dev->base_addr;
	private->mii_if.mdio_read = mdio_read;
	private->mii_if.mdio_write = mdio_write;
	private->mii_if.advertising =
	    mdio_read(dev, dev->base_addr, MII_ADVERTISE);

	spin_lock_init(&private->lock);	/* XXX_MIKE - broken for SMP */
}

/*
 * ----------------------------------------------------------------------------
 * Currently a no-op.  According to previous commentary, the hardware
 * "cannot [!] be stuck because it is a built-in hardware - if we reach
 * here the ethernet port might be [physically] disconnected..."
 */
void
gt64260_eth_tx_timeout(struct net_device *dev)
{
}

/*
 * ----------------------------------------------------------------------------
 *  First function called after registering the network device.
 *  It's purpose is to initialize the device as an ethernet device,
 *  fill the structure that was given in registration with pointers
 *  to functions, and setting the MAC address of the interface
 *
 *  Input : pointer to network device structure to be filled
 *  Output : -ENONMEM if failed, 0 if success
 */
void
gt64260_eth_str2mac(char *str, unsigned char *mac)
{
	int i;

	for (i = 0; i < 12; i += 2) {
		mac[i / 2] = ((isdigit(str[i]) ?
			       str[i] - '0' :
			       (toupper(str[i]) - 'A' + 10)) << 4) |
		    (isdigit(str[i + 1]) ?
		     str[i + 1] - '0' : (toupper(str[i + 1]) - 'A' + 10));
	}

	return;
}

s32
gt64260_eth_init(struct net_device * dev)
{
	static u32 gt64260_eth2_initialized = 0;
	static u32 gt64260_eth1_initialized = 0;
	static u32 gt64260_eth0_initialized = 0;
	gt_eth_priv *private;

	ether_setup(dev);	/* auto assign some of the fields by kernel */
	dev = init_etherdev(dev, sizeof (gt_eth_priv));
	if (!dev) {
		panic("gt64260_eth_init : init_etherdev failed\n");
	}
	gt64260_eth_init_priv(dev);
	private = (gt_eth_priv *) (dev->priv);

	dev->open = gt64260_eth_open;
	dev->stop = gt64260_eth_stop;
	dev->set_config = NULL;	/* no runtime config support for now */
	dev->hard_start_xmit = gt64260_eth_start_xmit;
	dev->do_ioctl = gt64260_ioctl;
	dev->get_stats = gt64260_eth_get_stats;
	dev->set_mac_address = gt64260_eth_set_mac_address;
	dev->set_multicast_list = gt64260_eth_set_rx_mode;
	dev->tx_timeout = gt64260_eth_tx_timeout;	/* Currently no-op */
	dev->watchdog_timeo = 2 * HZ;
	dev->flags &= ~IFF_RUNNING;

#ifdef CONFIG_NET_FASTROUTE
	dev->accept_fastpath = gt64260_eth_accept_fastpath;
#endif				/* #ifdef CONFIG_NET_FASTROUTE */

#if 0
#ifdef CONFIG_PPCBOOT
	memcpy(dev->dev_addr, boardinfo.bi_enetaddr, 6);
#else				/* #ifdef CONFIG_PPCBOOT */
#if 0
	memcpy(dev->dev_addr, eeprom_param.eth0_mac, 6);
#else
	dev->dev_addr[0] = 0x00;
	dev->dev_addr[1] = 0xa0;
	dev->dev_addr[2] = 0xf7;
	dev->dev_addr[3] = 0x33;
	dev->dev_addr[4] = 0x34;
	dev->dev_addr[5] = 0x36;
#endif
#endif				/* #else #ifdef CONFIG_PPCBOOT */
#endif

	if (dev->base_addr == 0 || dev->base_addr == 2) {
		/* FIXME: find who else is modifying this * (other than ev64260_pci.c)
		 * Maybe MPSC? - NTL */
#ifdef TWO_ETHERNET_MII_PORTS
		if (dev->base_addr == 0) {
			/* connect port 0 to MII */
			gt_set_bits(GT64260_MPP_SERIAL_PORTS_MULTIPLEX,
				    (1 << 0));
			gt_clr_bits(GT64260_MPP_SERIAL_PORTS_MULTIPLEX,
				    (1 << 1));
			/* port 1 is RMII/MII */
			/* nothing */
		}
#endif
#ifdef THREE_ETHERNET_RMII_PORTS
		/* connect port 0+2 to RMII */
		gt_clr_bits(GT64260_MPP_SERIAL_PORTS_MULTIPLEX, (1 << 0));
		gt_set_bits(GT64260_MPP_SERIAL_PORTS_MULTIPLEX, (1 << 1));
		/* port 1 is RMII/MII */
		/* nothing */
#endif
	}

	switch (dev->base_addr) {
	case 0:
		if (!gt64260_eth0_initialized) {
			if (memcmp(gt64260_mac_addrs[0],null_mac,6) != 0) {
				gt64260_eth_str2mac(gt64260_mac_addrs[0],
							dev->dev_addr);
			}
#ifndef	CONFIG_USE_PPCBOOT
			else {
				return -ENODEV;
			}
#else
			else if (ppcboot_bd_valid &&
			    *((unsigned long *) ppcboot_bd.bi_enetaddr) != 0) {
				memcpy(dev->dev_addr, ppcboot_bd.bi_enetaddr,
				       6);
			}
			else {
				printk("%s: Couldn't assign MAC address\n",
				       dev->name);
				return (-ENODEV);
			}
#endif

			private->port = 0;
			gt_write(GT64260_ENET_E0SDCR, (2 << 12) | (1 << 9) | (0xf << 2));	// 0000.203c

			/* enable */
			gt_set_bits(GT64260_ENET_E0PCR, (1 << 7));
			/* no promisc */
			gt_clr_bits(GT64260_ENET_E0PCR, (1 << 0));

			/*
			 * Receive packets in 1536 bit max length and enable DSCP
			 */
			gt_write(GT64260_ENET_E0PCXR,
				 PORT_CONTROL_EXTEND_VALUE);

			/*
			 * Initialize address table for hash mode 0 with 1/2K size
			 */
			initAddressTable(private->port, 0, 1, 0);
			gt64260_eth0_initialized = 1;
		}
		break;
	case 1:
		if (!gt64260_eth1_initialized) {
			if (memcmp(gt64260_mac_addrs[1],null_mac,6) != 0) {
				gt64260_eth_str2mac(gt64260_mac_addrs[1],
							dev->dev_addr);
			}
#ifndef	CONFIG_USE_PPCBOOT
			else {
				return -ENODEV;
			}
#else
			else if (ppcboot_bd_valid &&
			    *((unsigned long *) ppcboot_bd.bi_enet1addr) != 0) {
				memcpy(dev->dev_addr, ppcboot_bd.bi_enet1addr,
				       6);
			}
			else {
				printk("%s: Couldn't assign MAC address\n",
				       dev->name);
				return (-ENODEV);
			}
#endif
			private->port = 1;
			gt_write(GT64260_ENET_E1SDCR, 0x0000203c);

			/* enable */
			gt_set_bits(GT64260_ENET_E1PCR, (1 << 7));
			/* no promisc */
			gt_clr_bits(GT64260_ENET_E1PCR, (1 << 0));

			gt_write(GT64260_ENET_E1PCXR,
				 PORT_CONTROL_EXTEND_VALUE);

			/*
			 * Initialize address table for hash mode 0 with 1/2K size
			 */
			initAddressTable(private->port, 0, 1, 0);
			gt64260_eth1_initialized = 1;
		}
		break;
	case 2:
		if (!gt64260_eth2_initialized) {
			if (memcmp(gt64260_mac_addrs[2],null_mac,6) != 0) {
				gt64260_eth_str2mac(gt64260_mac_addrs[2],
							dev->dev_addr);
			}
#ifndef	CONFIG_USE_PPCBOOT
			else {
				return -ENODEV;
			}
#else
			else if (ppcboot_bd_valid &&
			    *((unsigned long *) ppcboot_bd.bi_enet2addr) != 0) {
				memcpy(dev->dev_addr, ppcboot_bd.bi_enet2addr,
				       6);
			}
			else {
				printk("%s: Couldn't assign MAC address\n",
				       dev->name);
				return (-ENODEV);
			}
#endif
			private->port = 2;
			gt_write(GT64260_ENET_E2SDCR, 0x0000203c);

			/* enable */
			gt_set_bits(GT64260_ENET_E2PCR, (1 << 7));
			/* no promisc */
			gt_clr_bits(GT64260_ENET_E2PCR, (1 << 0));

			gt_write(GT64260_ENET_E2PCXR,
				 PORT_CONTROL_EXTEND_VALUE);

			/*
			 * Initialize address table for hash mode 0 with 1/2K size
			 */
			initAddressTable(private->port, 0, 1, 0);
			gt64260_eth2_initialized = 1;
		}
		break;
	default:
		return (-ENODEV);	/* Trouble if we haven't returned by this point... */
	}
	/*
	 * Read MIB counters on the GT in order to reset them,
	 * then zero all the stats fields in memory
	 */
	gt64260_eth_update_stat(dev);

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 *  This function is called when opening the network device. The function
 *  should initialize all the hardware, initialize cyclic Rx/Tx
 *  descriptors chain and buffers and allocate an IRQ to the network
 *  device.
 *
 *  Input : a pointer to the network device structure
 *
 *  Output : zero if success, nonzero if fails.
 */
s32
gt64260_eth_open(struct net_device * dev)
{
	gt_dma_desc *desc;
	gt_eth_priv *priv;
	s32 retval;
	struct sk_buff *sk;
	u32 count;
	u32 gap;
	u32 port;
	u32 port_status;
	u32 queue;

	priv = dev->priv;

	/*
	 * Initialize the lists of Tx/Rx descriptors (as circular chains,
	 * each in its own uncached page) using the physical addresses in
	 * the "next" pointers that the Enet DMA engine expects.  The Rx
	 * descriptors also get an sk_buff pre-allocated for them and their
	 * "data" pointers set to point to the corresponding sk_buff buffer.
	 */
	for (queue = 0; queue < NUM_TX_QUEUES; queue++) {
		priv->TXskbIndex[queue] = priv->TXindex[queue] = 0;
		desc = priv->TXqueue[queue];
		memset((void *) desc, 0, PAGE_SIZE);	/* The whole list. */
		for (count = 0; count < Q_INDEX_LIMIT; count++, desc++) {
			desc->next = virt_to_phys((void *) (desc + 1));
			priv->TXskbuff[queue][count] = 0;
		}
		--desc;		/* Link last back to first. */
		desc->next = virt_to_phys((void *) (priv->TXqueue[queue]));
		flush_dcache_addr_size((u32) (priv->TXqueue[queue]), PAGE_SIZE);
	}

	for (queue = 0; queue < NUM_RX_QUEUES; queue++) {
		priv->RXindex[queue] = 0;
		desc = priv->RXqueue[queue];
		memset((void *) desc, 0, PAGE_SIZE);	/* The whole list. */
		for (count = 0; count < Q_INDEX_LIMIT; count++, desc++) {
			desc->next = virt_to_phys((void *) (desc + 1));
			desc->count.rx.bufferBytes = MAX_BUFF_SIZE;	/* XXX_MIKE: really? */
			desc->command_status = GT_ENET_DESC_OWNERSHIP
			    | GT_ENET_DESC_INT_ENABLE;

			sk = dev_alloc_skb(MAX_BUFF_SIZE);
			desc->data = (void *) virt_to_phys((void *) (sk->data));
			priv->RXskbuff[queue][count] = sk;
			invalidate_dcache_range((u32) (sk->data),
						(u32) (sk->data) +
						MAX_BUFF_SIZE);
		}
		--desc;		/* Link last back to first. */
		desc->next = virt_to_phys((void *) (priv->RXqueue[queue]));
		flush_dcache_addr_size((u32) (priv->RXqueue[queue]), PAGE_SIZE);
	}

	/*
	 * Update Hash Table with the dedicated MAC address
	 * XXX_MIKE - why "update" rather than "set" ?
	 */
	gt64260_eth_update_mac_address(dev);

	/*
	 * Initialize DMA descriptor-pointer registers
	 */
	port = priv->port;
	gap = ETH_ADDR_GAP * port;

	gt_write(GT64260_ENET_E0CTDP0 + gap,
		 virt_to_phys((void *) priv->TXqueue[0]));
	gt_write(GT64260_ENET_E0CTDP1 + gap,
		 virt_to_phys((void *) priv->TXqueue[1]));

	gt_write(GT64260_ENET_E0FRDP0 + gap,
		 virt_to_phys((void *) priv->RXqueue[0]));
	gt_write(GT64260_ENET_E0CRDP0 + gap,
		 virt_to_phys((void *) priv->RXqueue[0]));
	gt_write(GT64260_ENET_E0FRDP1 + gap,
		 virt_to_phys((void *) priv->RXqueue[1]));
	gt_write(GT64260_ENET_E0CRDP1 + gap,
		 virt_to_phys((void *) priv->RXqueue[1]));
	gt_write(GT64260_ENET_E0FRDP2 + gap,
		 virt_to_phys((void *) priv->RXqueue[2]));
	gt_write(GT64260_ENET_E0CRDP2 + gap,
		 virt_to_phys((void *) priv->RXqueue[2]));
	gt_write(GT64260_ENET_E0FRDP3 + gap,
		 virt_to_phys((void *) priv->RXqueue[3]));
	gt_write(GT64260_ENET_E0CRDP3 + gap,
		 virt_to_phys((void *) priv->RXqueue[3]));

	/*
	 * Set IP TOS Rx priority queueing
	 * These registers not #defined in header file.  Manual description:
	 *   IP Differentiated Services CodePoint to Priority[01] {low,high}
	 */
	gt_write(0x2460 + gap, 0xffff0000);
	gt_write(0x2464 + gap, 0xffff0000);
	gt_write(0x2468 + gap, 0x00000000);
	gt_write(0x246c + gap, 0xffffffff);

	/*
	 * Allocate IRQ
	 */
#if 0				/* XXXX */
	for (EVB64260_ETH_irq[port] = 8;
	     EVB64260_ETH_irq[port] < 32; EVB64260_ETH_irq[port]++) {
#endif
		retval = request_irq(GT64260_ETH_irq[port] + gt64260_irq_base,
				     gt64260_eth_int_handler,
				     (SA_INTERRUPT | SA_SAMPLE_RANDOM),
				     "GT64260_Eth", dev);
		if (!retval) {
			dev->irq = GT64260_ETH_irq[port] + gt64260_irq_base;
			printk
			    ("gt64260_eth_open : Assigned IRQ %d to gt64260_eth%d\n",
			     dev->irq, port);
#if 0				/* XXXX */
			break;
#endif
		}
#if 0				/* XXXX */
	}
#endif
	if (retval) {		/* XXX_MIKE - flawed logic... */
		printk("Can not assign IRQ number to GT64260_eth%d\n", port);
		GT64260_ETH_irq[port] = 0;
		return (-1);
	}

	/*
	 * clear all interrupts
	 */
	gt_write(GT64260_ENET_E0ICR + gap, 0x000000000);

	/*
	 * enable relevant interrupts on GT
	 */
	gt_write(GT64260_ENET_E0IMR + gap, 0xb0ff010d);

	/*
	 * Enable interrupts in high cause register
	 */
	gt_set_bits(GT64260_IC_CPU_INTR_MASK_HI, BIT(0) << port);

	/*
	 * Check Link status on Ethernet0
	 */
	port_status = gt_read(GT64260_ENET_E0PSR + gap);
	if (!(port_status & 0x8)) {
		netif_stop_queue(dev);
		dev->flags &= ~IFF_RUNNING;
	} else {
		netif_start_queue(dev);
		dev->flags |= IFF_RUNNING;
	}

	check_phy_state(dev);
	dump_link_state(dev);
	/*
	 * start RX  (BIT(7) == EnableRXdma)
	 */
	gt_write(GT64260_ENET_E0SDCMR + gap, BIT(7));
	spin_lock_init(&dev->xmit_lock);
	MOD_INC_USE_COUNT;

	return (0);
}

/*
 * ----------------------------------------------------------------------------
 * This function is used when closing the network device.  It should update
 * the hardware, release all memory that holds buffers and descriptors and
 * release the IRQ.
 * Input : a pointer to the device structure
 * Output : zero if success, nonzero if fails
 */
int
gt64260_eth_stop(struct net_device *dev)
{
	gt_eth_priv *priv;
	u32 queue;
	u32 count;

	priv = dev->priv;

	/*
	 * stop RX and mask interrupts
	 */
	gt_clr_bits(GT64260_IC_CPU_INTR_MASK_HI, (BIT(0) << priv->port));
	gt_write(GT64260_ENET_E0SDCMR
		 + (ETH_ADDR_GAP * priv->port), 0x00008000);
	netif_stop_queue(dev);

	/*
	 * Free RX pre allocated SKB's
	 */
	for (queue = 0; queue < NUM_RX_QUEUES; queue++) {
		for (count = 0; count < Q_INDEX_LIMIT; count++) {
			if (priv->RXskbuff[queue][count]) {
				dev_kfree_skb(priv->RXskbuff[queue][count]);
				priv->RXskbuff[queue][count] = 0;
			}
		}
	}

#if 0				/* XXXX */
	GT64260_ETH_irq[priv->port] = 0;
#endif
	free_irq(dev->irq, dev);
	MOD_DEC_USE_COUNT;

	return (0);
}

/*
 * ----------------------------------------------------------------------------
 * This function queues a packet in the Tx descriptor for required
 * port.  It checks the IPTOS_PREC_FLASHOVERRIDE bit of the ToS
 * field in the IP header and decides where to queue the packet,
 * in the high priority queue or in the low priority queue.
 *
 * Input : skb - a pointer to socket buffer
 *         dev - a pointer to the required port
 *
 * Output : zero upon success, negative number upon failure
 *         (-EBUSY when the interface is busy)
 */
int
gt64260_eth_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	gt_eth_priv *priv;
	gt_dma_desc *tx;
	struct iphdr *iph;
	u32 queue;
	u32 TXindex;

	iph = skb->nh.iph;
	priv = dev->priv;
	spin_lock_irq(&(priv->lock));

	/*
	 * Paranoid check - this shouldn't happen
	 */
	if (skb == NULL) {
		priv->stat.tx_dropped++;
		return (1);
	}

	/*
	 * Queue packet to either queue 0 (high priority) or 1 (low
	 * priority) depending on the MSB of the TOS in the IP header
	 */
	queue = !(IPTOS_PREC(iph->tos) & IPTOS_PREC_FLASHOVERRIDE);
	dev->trans_start = jiffies;	/* timestamp */
	TXindex = priv->TXindex[queue];
	if (priv->TXskbuff[queue][TXindex]) {
		panic("Error on gt64260_eth device driver");
	}

	priv->TXskbuff[queue][TXindex] = skb;
	tx = &(priv->TXqueue[queue][TXindex]);
	invalidate_dcache_range((u32) tx, (u32) tx);
	mb();
	tx->data = (void *) virt_to_phys((void *) skb->data);
	tx->count.tx.bytes = skb->len;

	/*
	 * Flush/sync now before transferring ownership.
	 */
	flush_dcache_addr_size((u32) skb->data, skb->len);

	/*
	 * Don't generate interrupts upon routed packets,
	 * only when reached threshold
	 */
#ifdef CONFIG_NET_FASTROUTE
	if ((skb->pkt_type == PACKET_FASTROUTE)
	    && !(TXindex % GT64260_INT_THRE)) {
		priv->TXqueue[queue][TXindex].command_status =
		    BIT(31) | BIT(22) | (7 << 16);
	} else {
#endif				/* #ifdef CONFIG_NET_FASTROUTE */
		/*
		 * XXX_MIKE - document these magic numbers and logic
		 */
		wmb();
		if (1 /*skb->nf.iph->saddr */ ) {
			tx->command_status =
			    (1 << 31) | (1 << 23) | (1 << 22) | (7 << 16);
		} else {
			if (TXindex % GT64260_INT_THRE) {
				tx->command_status =
				    (1 << 31) | (1 << 22) | (7 << 16);
			} else {
				tx->command_status =
				    (1 << 31) | (1 << 23) | (1 << 22) | (7 <<
									 16);
			}
		}
#ifdef CONFIG_NET_FASTROUTE
	}
#endif				/* #ifdef CONFIG_NET_FASTROUTE */

	/*
	 * Officially transfer ownership of descriptor to GT
	 */
	flush_dcache(tx);
	mb();

	TXindex++;
	if (TXindex == Q_INDEX_LIMIT) {
		TXindex = 0;
	}

	/*
	 * If next descriptor is GT owned then the tx queue is full
	 * XXX_MIKE - possession of sk_buff is unambiguous sign?
	 */
	if (priv->TXskbuff[queue][TXindex]) {
		//    printk ("Stopping queue on port %d\n",priv->port);
		netif_stop_queue(dev);
	}
	priv->TXindex[queue] = TXindex;

	/*
	 * Start Tx LOW dma
	 */
	gt_write(GT64260_ENET_E0SDCMR
		 + (ETH_ADDR_GAP * priv->port), queue ? (1 << 23) : (1 << 24));

	spin_unlock_irq(&priv->lock);

	return (0);		/* success */
}

/*
 * ----------------------------------------------------------------------------
 * This function is forward packets that are received from the port's
 * queues toward kernel core or FastRoute them to another interface.
 *
 * Input : dev - a pointer to the required interface
 *
 * Output : number of served packets
 */
u32
gt64260_eth_receive_queue(struct net_device * dev, s32 queue, u32 max)
{
	gt_eth_priv *priv;
	gt_dma_desc *rx;
	struct sk_buff *skb;
	u32 RXindex;
	u32 served;
#ifdef CONFIG_NET_FASTROUTE
	struct ethhdr *eth;
	struct iphdr *iph;
	struct net_device *odev;
	struct rtable *rt;
	u32 CPU_ID = smp_processor_id();
	u32 fast_routed = 0;
	u32 h;
#endif				/* #ifdef CONFIG_NET_FASTROUTE */

	priv = dev->priv;
	RXindex = priv->RXindex[queue];	/* Where we left off... */
	served = 0;
	while (max) {
#ifdef CONFIG_NET_FASTROUTE
		fast_routed = 0;
#endif				/* #ifdef CONFIG_NET_FASTROUTE */
		rx = &(priv->RXqueue[queue][RXindex]);
		invalidate_dcache(rx);
		mb();
		if (rx->command_status & GT_ENET_DESC_OWNERSHIP) {
			break;
		}
		max--;

		/*
		 * If received packet has errors, keep the socket buffer and change
		 * descriptor to GT owner ship and continue analyzing next descriptor.
		 */
		if (rx->command_status & GT_ENET_DESC_ERROR_SUMMARY) {
			rx->command_status = GT_ENET_DESC_OWNERSHIP
			    | GT_ENET_DESC_INT_ENABLE;
			flush_dcache(rx);
			mb();
			RXindex++;
			if (RXindex == Q_INDEX_LIMIT) {
				RXindex = 0;
			}
			continue;
		}
		served++;
		skb = priv->RXskbuff[queue][RXindex];
		invalidate_dcache(rx);
		mb();
		if (skb) {
			if (skb->len) {
				printk
				    ("gt64260_eth_receive_queue: nonzero existing SKB\n");
				dev_kfree_skb(skb);
				skb = dev_alloc_skb(MAX_BUFF_SIZE);
			}

			/*
			 * XXX_MIKE - document these magic numbers
			 */
			skb_put(skb, (rx->count.rx.bytesReceived - 4));
			invalidate_dcache(rx);
			mb();

#ifdef CONFIG_NET_FASTROUTE
			/*
			 * Fast Route
			 */
			eth = (struct ethhdr *) skb->data;
			if (eth->h_proto == __constant_htons(ETH_P_IP)) {
				iph = (struct iphdr *) (skb->data + ETH_HLEN);
				h = (*(u8 *) & iph->daddr
				     ^ *(u8 *) & iph->
				     saddr) & NETDEV_FASTROUTE_HMASK;
				rt = (struct rtable *) (dev->fastpath[h]);
				if (rt
				    && ((u16 *) & iph->daddr)[0] ==
				    ((u16 *) & rt->key.dst)[0]
				    && ((u16 *) & iph->daddr)[1] ==
				    ((u16 *) & rt->key.dst)[1]
				    && ((u16 *) & iph->saddr)[0] ==
				    ((u16 *) & rt->key.src)[0]
				    && ((u16 *) & iph->saddr)[1] ==
				    ((u16 *) & rt->key.src)[1]
				    && !rt->u.dst.obsolete) {
					odev = rt->u.dst.dev;
					netdev_rx_stat[CPU_ID].fastroute_hit++;

					if (*(u8 *) iph == 0x45 && !(eth->h_dest[0] & 0x80)	/* This should be 0x80 */
					    &&neigh_is_valid(rt->u.dst.
							     neighbour)
					    && iph->ttl > 1) {	/* Fast Route Path */
						fast_routed = 1;
						if ((!netif_queue_stopped(odev))
						    &&
						    (!spin_is_locked
						     (&odev->xmit_lock))) {
							skb->pkt_type =
							    PACKET_FASTROUTE;
							skb->protocol =
							    __constant_htons
							    (ETH_P_IP);

							ip_decrease_ttl(iph);

#if 0
							/*
							 * UNNECESSARY - a CRC is performed by hardware
							 */
							skb->ip_summed =
							    CHECKSUM_NONE;
							ip_statistics.
							    IpInReceives++;
							ip_statistics.
							    IpForwDatagrams++;
#endif				/* #if 0 */
							memcpy(eth->h_source,
							       odev->dev_addr,
							       6);
							memcpy(eth->h_dest,
							       rt->u.dst.
							       neighbour->ha,
							       6);
							skb->dev = odev;
							if (odev->
							    hard_start_xmit(skb,
									    odev)
							    != 0) {
								panic
								    ("Shit, what went wrong ??!!");
							}
							netdev_rx_stat[CPU_ID].
							    fastroute_success++;
						} else {	/* Semi Fast Route Path */
							skb->pkt_type =
							    PACKET_FASTROUTE;
							skb->nh.raw =
							    skb->data +
							    ETH_HLEN;
							skb->protocol =
							    __constant_htons
							    (ETH_P_IP);
							netdev_rx_stat[CPU_ID].
							    fastroute_defer++;
							netif_rx(skb);
						}
					}
				}
			}

			if (fast_routed == 0)
#endif				/* #ifdef CONFIG_NET_FASTROUTE */
			{

				skb->protocol = eth_type_trans(skb, dev);
				skb->pkt_type = PACKET_HOST;
				skb->ip_summed = CHECKSUM_NONE;	//UNNECESSARY; /* CRC performed by hardware */
				skb->dev = dev;
				netif_rx(skb);
			}
		}

		skb = dev_alloc_skb(MAX_BUFF_SIZE);
		priv->RXskbuff[queue][RXindex] = skb;
		if (!skb) {
			/*
			 * No memory, quit meantime, and will try to
			 * recover next received packet or using watchdog
			 */
			printk("Ethernet ev64260 port %d - no mem\n",
			       priv->port);
			break;	/* XXX_MIKE - descriptor not updated? */
		} else {
			invalidate_dcache_range((u32) (skb->data),
						(u32) (skb->data) +
						MAX_BUFF_SIZE);
		}
		skb->dev = dev;
		rx->data = (void *) virt_to_phys((void *) skb->data);

		/*
		 * Officially transfer ownership of descriptor to GT
		 */
		rx->command_status = 0x80800000;	/* GT owner bit */
		flush_dcache(rx);
		mb();

		RXindex++;
		if (RXindex == Q_INDEX_LIMIT) {
			RXindex = 0;
		}
	}

	priv->RXindex[queue] = RXindex;

	return (served);
}

/*
 * ----------------------------------------------------------------------------
 * Input : dev - a pointer to the required interface
 *
 * Output : N/A
 */
u32
gt64260_eth_free_tx_queue(struct net_device * dev, s32 queue)
{
	gt_eth_priv *priv;
	gt_dma_desc *tx;
	struct sk_buff *sk;
	u32 freed_skbs;
	u32 TXskbIndex;

	priv = dev->priv;
	spin_lock(&(priv->lock));
	freed_skbs = 0;
	TXskbIndex = priv->TXskbIndex[queue];
	while (1) {
		sk = priv->TXskbuff[queue][TXskbIndex];
		if (!sk) {
			break;
		}
		tx = &(priv->TXqueue[queue][TXskbIndex]);	/* No write to tx here */
		invalidate_dcache(tx);
		mb();

		if (tx->command_status & 0x80000000) {
			break;
		}
		if (tx->command_status & 0x40) {
			priv->stat.tx_fifo_errors++;
		}
		dev_kfree_skb_irq(sk);
		priv->TXskbuff[queue][TXskbIndex] = 0;
		TXskbIndex++;
		if (TXskbIndex == Q_INDEX_LIMIT) {
			TXskbIndex = 0;
		}
		freed_skbs++;
	}
	priv->TXskbIndex[queue] = TXskbIndex;
	spin_unlock(&(priv->lock));

	return (freed_skbs);
}

/*
 * ----------------------------------------------------------------------------
 */
void
gt64260_eth_int_handler(s32 irq, void *dev_id, struct pt_regs *regs)
{
	gt_eth_priv *priv;
	struct net_device *dev;
	u32 eth_int_cause;
	u32 gap;

	dev = (struct net_device *) dev_id;
	priv = dev->priv;
	gap = (ETH_ADDR_GAP * priv->port);
	eth_int_cause = gt_read(GT64260_ENET_E0ICR + gap);

	if (eth_int_cause & 0xcc) {	/* XXX_MIKE - document this */
		if (eth_int_cause & 0x88) {	/* Free queue 0, which is TxBufferHigh */
			gt64260_eth_free_tx_queue(dev, 0);
		}
		if (eth_int_cause & 0x44) {	/* Free queue 1, which is TxBufferLow */
			gt64260_eth_free_tx_queue(dev, 1);
		}
		if (netif_queue_stopped(dev)
		    && (priv->TXskbuff[0][priv->TXindex[0]] == 0)
		    && (priv->TXskbuff[1][priv->TXindex[1]] == 0)
		    && (dev->flags & IFF_RUNNING)) {
			netif_wake_queue(dev);
		}
		eth_int_cause &= ~0xcc;
		gt_write(GT64260_ENET_E0ICR + gap, ~(u32) 0xcc);
	}

	if (eth_int_cause & 0x101) {	/*RxBuffer, RxResource Error */
		/*
		 * Serve 4 Queues starting from the one with highest priority
		 */
		if (eth_int_cause & 0x00880100) {
			/*
			 * Rx Return Buffer / Resource Error Priority queue 3
			 */
			gt64260_eth_receive_queue(dev, 3, 50);
		}
		if (eth_int_cause & 0x00440100) {
			/*
			 * Rx Return Buffer / Resource Error Priority queue 2
			 */
			gt64260_eth_receive_queue(dev, 2, 25);
		}
		if (eth_int_cause & 0x00220100) {
			/*
			 * Rx Priority Return Buffer / Resource Error queue 1
			 */
			gt64260_eth_receive_queue(dev, 1, 13);
		}
		if (eth_int_cause & 0x00110100) {
			/*
			 * Rx Priority Return Buffer / Resource Error queue 0
			 */
			gt64260_eth_receive_queue(dev, 0, 12);
		}

		/*
		 * start/continue ethernet 0 RX
		 */
		gt_write(GT64260_ENET_E0SDCMR + gap, (1 << 7));
		eth_int_cause &= ~0x00ff0101;
		gt_write(GT64260_ENET_E0ICR + gap, ~0x00ff0101);
	}

	if (eth_int_cause & 0x10000000) {	/* MII PHY status changed */
		u32 port_status;

		/*
		 * Check Link status on Ethernet0
		 */
		port_status = gt_read(GT64260_ENET_E0PSR + gap);
		if (!(port_status & 0x8)) {
			if (netif_carrier_ok(dev)) {
				printk("%s: changed link status to DOWN\n",
				       dev->name);
				netif_carrier_off(dev);
			}
		} else {
			if (!netif_carrier_ok(dev)) {
				printk("%s: changed link status to UP\n",
				       dev->name);
				netif_carrier_on(dev);
			}

			/*
			 * start/continue ethernet 0 TX
			 */
			gt_write(GT64260_ENET_E0SDCMR
				 + (0x4000 * priv->port),
				 ((1 << 23) | (1 << 24)));
		}
		eth_int_cause &= ~0x10000000;
		gt_write(GT64260_ENET_E0ICR + gap, ~0x10000000);
	}

	if (eth_int_cause & 0x20000000) {	/* MII SMI complete */
		eth_int_cause &= ~0x20000000;
		gt_write(GT64260_ENET_E0ICR + gap, ~0x20000000);
	}

	if (eth_int_cause & ~(1 << 31)) {
		printk("%s: unhandled int %08x\n", dev->name, eth_int_cause);
		gt_write(GT64260_ENET_E0ICR + gap, 0);
	}

	return;
}

typedef struct addressTableEntryStruct {
	u32 hi;
	u32 lo;
} addrTblEntry;

static u32 addressTableHashMode[MAX_NUMBER_OF_ETHERNET_PORTS];
static u32 addressTableHashSize[MAX_NUMBER_OF_ETHERNET_PORTS];
static addrTblEntry *addressTableBase[MAX_NUMBER_OF_ETHERNET_PORTS];

static u32 hashLength[MAX_NUMBER_OF_ETHERNET_PORTS] = {
	(0x8000),		/* XXX_MIKE - are these correct? *//* 32K entries */
	(0x8000 / 16),		/*  2K entries */
};

/*
 * clear the address table
 * port - ETHERNET port number.
 */
static void
addressTableClear(u32 port)
{
	memset((void *) addressTableBase[port],
	       0, (hashLength[addressTableHashSize[port]] * MAC_ENTRY_SIZE));

	flush_dcache_addr_size((u32) (addressTableBase[port]),
			       (hashLength[addressTableHashSize[port]]
				* MAC_ENTRY_SIZE));
}

/*
 * ----------------------------------------------------------------------------
 * This function will initialize the address table and will enableFiltering.
 * Inputs
 * hashMode         - hash mode 0 or hash mode 1.
 * hashSizeSelector - indicates number of hash table entries (0=0x8000,1=0x800)
 * hashDefaultMode  - 0 = discard addresses not found in the address table,
 *                    1 = pass addresses not found in the address table.
 * port - ETHERNET port number.
 * Outputs
 * address table is allocated and initialized.
 * Always returns TRUE
 */

static int
initAddressTable(u32 port,
		 u32 hashMode, u32 hashSizeSelector, u32 hashDefaultMode)
{
	u32 addr;
	u32 bytes;
	ETHERNET_PCR portControlReg;

	addressTableHashMode[port] = hashMode;
	addressTableHashSize[port] = hashSizeSelector;

	/*
	 * Allocate memory for the address table, which must reside
	 * on an 8-byte boundary.
	 */
	bytes = MAC_ENTRY_SIZE * hashLength[hashSizeSelector];
	if (bytes & ~PAGE_MASK) {
		panic("initAddressTable: computed size isn't page-multiple.\n");
	}
	addr = uncachedPages(bytes >> PAGE_SHIFT);
	memset((void *) addr, 0, bytes);
	flush_dcache_addr_size(addr, bytes);
	gt_write(GT64260_ENET_E0HTPR
		 + (ETHERNET_PORTS_DIFFERENCE_OFFSETS * port),
		 virt_to_phys((void *) addr));

	invalidate_dcache_range(addr, addr + bytes);
	mb();
	addressTableBase[port] = (addrTblEntry *) addr;

	/*
	 * set hash {size,mode} and HDM in the PCR
	 */
	portControlReg = get_port_config(port);
	portControlReg &= ~((1 << HASH_DEFAULT_MODE) | (1 << HASH_MODE)
			    | (1 << HASH_SIZE));

	portControlReg |= ((hashDefaultMode << HASH_DEFAULT_MODE)
			   | (hashMode << HASH_MODE)
			   | (hashSizeSelector << HASH_SIZE));

	set_port_config(portControlReg, port);
	enableFiltering(port);

	return (TRUE);
}

/*
 * ----------------------------------------------------------------------------
 * This function will calculate the hash function of the address.
 * depends on the hash mode and hash size.
 * Inputs
 * macH             - the 2 most significant bytes of the MAC address.
 * macL             - the 4 least significant bytes of the MAC address.
 * hashMode         - hash mode 0 or hash mode 1.
 * hashSizeSelector - indicates number of hash table entries (0=0x8000,1=0x800)
 * Outputs
 * return the calculated entry.
 */
static u32
hashTableFunction(u32 macH, u32 macL, u32 HashSize, u32 hash_mode)
{
	u32 hashResult;
	u32 addrH;
	u32 addrL;
	u32 addr0;
	u32 addr1;
	u32 addr2;
	u32 addr3;
	u32 addrHSwapped;
	u32 addrLSwapped;

	addrH = NIBBLE_SWAPPING_16_BIT(macH);
	addrL = NIBBLE_SWAPPING_32_BIT(macL);

	addrHSwapped = FLIP_4_BITS(addrH & 0xf)
	    + ((FLIP_4_BITS((addrH >> 4) & 0xf)) << 4)
	    + ((FLIP_4_BITS((addrH >> 8) & 0xf)) << 8)
	    + ((FLIP_4_BITS((addrH >> 12) & 0xf)) << 12);

	addrLSwapped = FLIP_4_BITS(addrL & 0xf)
	    + ((FLIP_4_BITS((addrL >> 4) & 0xf)) << 4)
	    + ((FLIP_4_BITS((addrL >> 8) & 0xf)) << 8)
	    + ((FLIP_4_BITS((addrL >> 12) & 0xf)) << 12)
	    + ((FLIP_4_BITS((addrL >> 16) & 0xf)) << 16)
	    + ((FLIP_4_BITS((addrL >> 20) & 0xf)) << 20)
	    + ((FLIP_4_BITS((addrL >> 24) & 0xf)) << 24)
	    + ((FLIP_4_BITS((addrL >> 28) & 0xf)) << 28);

	addrH = addrHSwapped;
	addrL = addrLSwapped;

	if (hash_mode == 0) {
		addr0 = (addrL >> 2) & 0x03f;
		addr1 = (addrL & 0x003) | ((addrL >> 8) & 0x7f) << 2;
		addr2 = (addrL >> 15) & 0x1ff;
		addr3 = ((addrL >> 24) & 0x0ff) | ((addrH & 1) << 8);
	} else {
		addr0 = FLIP_6_BITS(addrL & 0x03f);
		addr1 = FLIP_9_BITS(((addrL >> 6) & 0x1ff));
		addr2 = FLIP_9_BITS((addrL >> 15) & 0x1ff);
		addr3 =
		    FLIP_9_BITS((((addrL >> 24) & 0x0ff) |
				 ((addrH & 0x1) << 8)));
	}

	hashResult = (addr0 << 9) | (addr1 ^ addr2 ^ addr3);

	if (HashSize == _8K_TABLE) {
		hashResult = hashResult & 0xffff;
	} else {
		hashResult = hashResult & 0x07ff;
	}

	return (hashResult);
}

/*
 * ----------------------------------------------------------------------------
 * This function will add an entry to the address table.
 * depends on the hash mode and hash size that was initialized.
 * Inputs
 * port - ETHERNET port number.
 * macH - the 2 most significant bytes of the MAC address.
 * macL - the 4 least significant bytes of the MAC address.
 * skip - if 1, skip this address.
 * rd   - the RD field in the address table.
 * Outputs
 * address table entry is added.
 * TRUE if success.
 * FALSE if table full
 */
static int
addAddressTableEntry(u32 port, u32 macH, u32 macL, u32 rd, u32 skip)
{
	addrTblEntry *entry;
	u32 newHi;
	u32 newLo;
	u32 i;

	newLo = (((macH >> 4) & 0xf) << 15)
	    | (((macH >> 0) & 0xf) << 11)
	    | (((macH >> 12) & 0xf) << 7)
	    | (((macH >> 8) & 0xf) << 3)
	    | (((macL >> 20) & 0x1) << 31)
	    | (((macL >> 16) & 0xf) << 27)
	    | (((macL >> 28) & 0xf) << 23)
	    | (((macL >> 24) & 0xf) << 19)
	    | (skip << SKIP_BIT) | (rd << 2) | VALID;

	newHi = (((macL >> 4) & 0xf) << 15)
	    | (((macL >> 0) & 0xf) << 11)
	    | (((macL >> 12) & 0xf) << 7)
	    | (((macL >> 8) & 0xf) << 3)
	    | (((macL >> 21) & 0x7) << 0);

	/*
	 * Pick the appropriate table, start scanning for free/reusable
	 * entries at the index obtained by hashing the specified MAC address
	 */
	entry = addressTableBase[port];
	entry += hashTableFunction(macH, macL, addressTableHashSize[port],
				   addressTableHashMode[port]);
	for (i = 0; i < HOP_NUMBER; i++, entry++) {
		if (!(entry->lo & VALID) /*|| (entry->lo & SKIP) */ ) {
			break;
		} else {	/* if same address put in same position */
			if (((entry->lo & 0xfffffff8) == (newLo & 0xfffffff8))
			    && (entry->hi == newHi)) {
				break;
			}
		}
	}

	if (i == HOP_NUMBER) {
		printk("addGT64260addressTableEntry: table section is full\n");
		return (FALSE);
	}

	/*
	 * Update the selected entry
	 */
	entry->hi = newHi;
	entry->lo = newLo;
	flush_dcache_addr_size((u32) entry, MAC_ENTRY_SIZE);
	return (TRUE);
}

/* port control extend register funcions */

/*****************************************************************************
*
* int etherReadMIIReg (unsigned int portNumber , unsigned int MIIReg,
* unsigned int* value)
*
* Description
* This function will access the MII registers and will read the value of
* the MII register , and will retrieve the value in the pointer.
* Inputs
* portNumber - one of the 2 possiable Ethernet ports (0-1).
* MIIReg - the MII register offset.
* Outputs
* value - pointer to unsigned int which will receive the value.
* Returns Value
* TRUE if success.
* FALSE if fail to make the assignment.
* Error types (and exceptions if exist)
*/

static int
etherReadMIIReg(unsigned int portNumber, unsigned int MIIReg,
		unsigned int *value)
{
	SMI_REG smiReg;
	unsigned int phyAddr;
	unsigned int timeOut = 1000;
	int i;
	
	phyAddr = PHY_ADD0 + portNumber;

	/* first check that it is not busy */
	smiReg = gt_read(GT64260_ENET_ESMIR);
	while (smiReg & SMI_BUSY) {
		if (timeOut-- < 1) {
			printk("TimeOut Passed Phy is busy\n");
			return FALSE;
		}
		for (i = 0; i < 1000; i++) ;
		smiReg = gt_read(GT64260_ENET_ESMIR);
	}
	/* not busy */

	gt_write(GT64260_ENET_ESMIR,
		 (SMI_OP_CODE_BIT_READ << 26) | (MIIReg << 21) | (phyAddr <<
								  16));

	timeOut = 1000;		/* initialize the time out var again */

	for (i = 0; i < 1000; i++) ;
	smiReg = gt_read(GT64260_ENET_ESMIR);
	while (!(smiReg & READ_VALID)) {
		for (i = 0; i < 1000; i++) ;
		smiReg = gt_read(GT64260_ENET_ESMIR);
		if (timeOut-- < 1) {
			printk("TimeOut Passed Read is not valid\n");
			return FALSE;
		}
	}

	*value = (unsigned int) (smiReg & 0xffff);

	return TRUE;
}

/*****************************************************************************
* 
* int etherWriteMIIReg (unsigned int portNumber , unsigned int MIIReg,
* unsigned int value)
* 
* Description
* This function will access the MII registers and will write the value
* to the MII register.
* Inputs
* portNumber - one of the 2 possiable Ethernet ports (0-1).
* MIIReg - the MII register offset.
* value -the value that will be written.
* Outputs
* Returns Value
* TRUE if success.
* FALSE if fail to make the assignment.
* Error types (and exceptions if exist)
*/

static int
etherWriteMIIReg(unsigned int portNumber, unsigned int MIIReg,
		 unsigned int value)
{
	SMI_REG smiReg;
	unsigned int phyAddr;
	unsigned int timeOut = 10;	/* in 100MS units */
	int i;

	/* first check that it is not busy */
	(unsigned int) smiReg = gt_read(GT64260_ENET_ESMIR);
	if (smiReg & SMI_BUSY) {
		for (i = 0; i < 10000000; i++) ;
		do {
			(unsigned int) smiReg = gt_read(GT64260_ENET_ESMIR);
			if (timeOut-- < 1) {
				printk("TimeOut Passed Phy is busy\n");
				return FALSE;
			}
		} while (smiReg & SMI_BUSY);
	}
	/* not busy */

	phyAddr = PHY_ADD0 + portNumber;

	smiReg = 0;		/* make sure no garbage value in reserved bits */
	smiReg = smiReg | (phyAddr << 16) | (SMI_OP_CODE_BIT_WRITE << 26) |
	    (MIIReg << 21) | (value & 0xffff);

	gt_write(GT64260_ENET_ESMIR, *((unsigned int *) &smiReg));

	return (TRUE);
}

static int __init
gt_mac0_setup(char *mac_addr)
{
	strncpy(gt64260_mac_addrs[0], mac_addr, 12);
	return 0;
}

static int __init
gt_mac1_setup(char *mac_addr)
{
	strncpy(gt64260_mac_addrs[1], mac_addr, 12);
	return 0;
}

static int __init
gt_mac2_setup(char *mac_addr)
{
	strncpy(gt64260_mac_addrs[2], mac_addr, 12);
	return 0;
}

__setup("gt_mac0=", gt_mac0_setup);
__setup("gt_mac1=", gt_mac1_setup);
__setup("gt_mac2=", gt_mac2_setup);


struct net_device gt64260_eth_devs[] = {
      {init:gt64260_eth_init,},
      {init:gt64260_eth_init,},
      {init:gt64260_eth_init,},
};

static int __init
gt64260_eth_module_init(void)
{
	int cards = 0;

	gt64260_eth_devs[0].base_addr = 0;
	if (register_netdev(&gt64260_eth_devs[0]) == 0) {
		cards++;
	}
	gt64260_eth_devs[1].base_addr = 1;
	if (register_netdev(&gt64260_eth_devs[1]) == 0) {
		cards++;
	}
	gt64260_eth_devs[2].base_addr = 2;
	if (register_netdev(&gt64260_eth_devs[2]) == 0) {
		cards++;
	}

	return cards > 0 ? 0 : -ENODEV;
}

static void __exit
gt64260_eth_module_exit(void)
{

}

module_init(gt64260_eth_module_init);
module_exit(gt64260_eth_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rabeeh Khoury");
MODULE_DESCRIPTION("Ethernet driver for Marvell/Galileo GT64260");
