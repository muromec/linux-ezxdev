/*
 * ibm_ocp_enet.c
 *
 * Ethernet driver for the built in ethernet on the IBM 4xx PowerPC
 * processors.
 * 
 * Added support for multiple PHY's and use of MII for PHY control
 * configurable and bug fixes.
 *
 * Based on  the Fast Ethernet Controller (FEC) driver for
 * Motorola MPC8xx and other contributions, see driver for contributers.
 *
 *      Armin Kuster akuster@mvista.com
 *      Sept, 2001
 *
 *      Original driver
 * 	Author: Johnnie Peters
 *	jpeters@mvista.com
 *
 * Copyright 2000 MontaVista Softare Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR   IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * TODO: re-link
 *       re-start
 *       check for memory allocation errors in: ppc405_enet_open(),
 *         init_ppc405_enet()
 *       Make it module - ak
 *       emac_num when ifconfig down/up changes
 *	 get rid of the need to have hard coded mal selection for the emac channels
 *
 *	version 2.1 (01/09/28) Armin kuster
 *		moved mii_* func's to ppc405_phy.c
 *		moved structs to ppc405_enet.h
 *
 *	version 2.2 (01/10/05) Armin kuster
 *		use dev->base_addr for EMAC regs addr
 *		removed "first_time" in int handles
 *
 *	Version 2.3 (10/10/15) Armin
 *		added NULL checks for dev_alloc_skb
 *		added mii_queue_schedule for processing mii queues
 *
 *	Version 2.4 (01/11/01) Armin
 *		cleaned up NULL logic
 *
 *	Version 2.5 (01/11/16) - frank Rowand
 *		fixed low memory routines in
 *		rx interrupt handlers
 *
 *	Version 2.6 (01/11/20) Armin Kuster
 *		Added back code to fix race
 *		condition in rx/tx interrupt
 *		routines (first_time)
 *	Vesrion 2.7 (01/12/26) Armin Kuster
 *		Added inital zmii bidge support
 *		Fixed shift for _write from 18 to 16
 *		migrating to OCP support
 *	Version: 2.8 (02/05/02) Armin
 *		Name change
 *		Added Multi-mal support via set_mal* / get_mal* & friends see
 *		ibm_ocp_mal.c
 *	Version: 2.9 (02/12/02) Armin, Amit, David 
 *		Added multi-irq support for multi-emacs - Armin
 *		Fix prototype & init in debug code - David M.
 *		EMAC_IER set default value and
 *		changed tx_aborted_errors from TEO to TE1 - Amit C.
 *
 *	Version: 3.0 (03/01/02) Andrew May
 *		Added checks in open/close
 *		Changed check on softreset
 *
 *	Version: 3.1 (03/25/02) Andrew May / armin
 *		added a skb_res module parm
 *		added rx_clean and rx_fill functions
 *		MBASE is defined in the /platforms/core.c - armin
 *
 *	Version: 3.2 (03/30/02) Armin
 *		Moved zmii code to ibm_ocp_zmii.c
 *
 *	Version: 3.3 (04/04/02) Matt Porter
 *		Message cleanup
 *		Cleanup debug warnings
 *		MALDBR is only on 40x
 *
 *	Version: 3.4 (04/05/02) Armin
 *		Multi- MAL support fixes
 *
 * 	Version: 3.5 (04/09/02) Matt Porter
 * 		Fix build problem
 * 		440 fixes for ZMII/RMII and watermarks
 *
 * 	Version: 3.5 (04/11/02) - Armin/David Gibson/ Todd Poynor
 * 		moved crc funtions to opc_crc32.c
 * 		Multicast bug fixes by David Gibson
 * 		cleaned up init by having ocp_register do more - ak
 * 		Added zmii_port_speed - T.P.
 * 	Version: 3.6 (04/17/02) - Armin
 * 		moved debug funtions to ibm_ocp_debug.c
 * 		moved emac inits to open
 * 		moved *virt_addr alloc to init
 * 		moved *virt_add free to exit
 * 		moved ocp_unregister to close
 * 	        removed 440 zmii_init, its auto now
 * 	        changed reset loops to a larger loop count & smaller delay
 *
 * 	Version: 3.7 (04/18/02) -Armin/ Todd
 * 		added shared irq flag, removed commented out code
 *
 * 	Version: 3.8 (04/24/02) Armin / Todd P. / Naoki Hamada<nao@tom-yam.or.jp>
 * 	        crc32.h includes and multicast cleanup by Naoki
 * 	        add 1/2 dup & 10MB flags - armin
 * 	        Change check for whether to reset existing MAL channels 
 * 	        (prior to reconfiguring) to test if any channels are 
 * 	        enabled, rather than testing if it is not EMAC0.  This 
 * 	        allows initial boot off EMAC > 0 and later ifconfig 
 * 	        eth0.  Similar change for the check as to whether the 
 * 	        MAL is to be reset and reconfigured, in case EMAC0 is not 
 * 	        the first device opened. - Todd  
 *
 * 	        Move init of fep->sequence_done to after fep init'ed  -Todd.
 * 	        added "already_opened" to _open so that it does not setup 
 * 	        the mal a second time - Todd
 * 	        Moved externs to ibm_ocp_emac.h
 *
 * 	 Version: 3.9 (05/01/02) - Armin
 * 	 	changed vaddr to paddr
 *	
 *	 Version: 4.0 (05/03/02) - Armin
 *		Added support for emac 3 & 4 in the mal area
 *		added ocp_free_dev when ocp_reg fails
 *		replaced EMAC_NUMS w/ emac_max = ocp_get_max()
 *	 Version: 4.1 (05/15/02) John Tyner
 *	          moved ocp_alloc_dev from open to init	
 *	 Version: 4.2 (05/25/02) Armin
 *	         name change *_driver *_dev
 *
 *	 Version: 4.3 06/05/02 - Armin
 *	 moved several pieces of mal code out to *_mal.c
 *	 added a probe to init. fixed ability to boot from any emac
 *	 fixed some mal setting of CARR & CASR reg.
 *	 added new headers to reflect splitting of define out.
 *
 *	 Version: 4.5 06/05/02 - Armin
 *	 removed need for emac_tx_all, now using reg for info
 *	 and using >> to get channel position
 *
 *	Vesrion: 4.6 07/05/02 - Armin/ David Gibson
 *      removed need for irq_resource
 *	removed need for loop in request & free irq
 *	changed interrupt handles to static
 *	 Version 4.7 07/25/02 - Armin
 *	    added CPM enable/diable in init/exit
 *
 *	Version: 4.7 08/22/02 - Eugene Surovegin
 *	1) Typo in request_irq 
 *	2) Bug in receive packet processing.
 *         I moved call ppc405_rx_fill() to ppc405_rx_clean() (and made in conditional).
 *         The reason for this is that if ppc405_rx_clean() is called but doesn't process
 *         any BD, ppc405_rx_fill() gets called anyway with i == fep->rx_slot. It will 
 *         corrupt the current BD (fep->rx_slot) and NUM_RX_BUFF packets will be lost 
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/pci.h>

#include <asm/processor.h>	/* Processor type for cache alignment. */
#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <asm/irq.h>
#include <asm/ocp.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/crc32.h>

#include "ocp_zmii.h"
#include "ibm_ocp_enet.h"
#include "ibm_ocp_mal.h"

/* Forward declarations of some structures to support different PHYs */

static int ppc405_enet_open(struct net_device *);
static int ppc405_enet_start_xmit(struct sk_buff *, struct net_device *);
static struct net_device_stats *ppc405_enet_stats(struct net_device *);
static int ppc405_enet_close(struct net_device *);
static void ppc405_enet_set_multicast_list(struct net_device *);

static void ppc405_eth_wakeup(int, void *, struct pt_regs *);
static void ppc405_eth_serr(int, void *, struct pt_regs *);
static void ppc405_eth_txeob(int, void *, struct pt_regs *);
static void ppc405_eth_rxeob(int, void *, struct pt_regs *);
static void ppc405_eth_txde(int, void *, struct pt_regs *);
static void ppc405_eth_rxde(int, void *, struct pt_regs *);
static void ppc405_eth_mac(int, void *, struct pt_regs *);
static void ppc405_rx_fill(struct net_device *, int);
static void ppc405_rx_clean(struct net_device *, int);

int ocp_enet_mdio_read(struct net_device *dev, int reg, uint * value);
int ocp_enet_mdio_write(struct net_device *dev, int reg);
int ocp_enet_ioctl(struct net_device *, struct ifreq *rq, int cmd);

static struct net_device *emac_dev[EMAC_NUMS];

dma_addr_t rx_phys_addr = 0;
dma_addr_t tx_phys_addr = 0;
static mal_desc_t *rx_virt_addr = 0;
static mal_desc_t *tx_virt_addr = 0;
mii_list_t mii_cmds[NMII];

int emac_max;

static int skb_res = SKB_RES;
MODULE_PARM(skb_res, "i");
MODULE_PARM_DESC(skb_res, "Amount of data to reserve on skb buffs\n"
		 "The 405 handles a misaligned IP header fine but\n"
		 "this can help if you are routing to a tunnel or a\n"
		 "device that needs aligned data");

static void
init_rings(struct net_device *dev)
{
	struct ocp_enet_private *ep = dev->priv;
	int loop;

	ep->tx_desc = (mal_desc_t *) ((char *) tx_virt_addr +
				      (ep->emac_num * PAGE_SIZE));
	ep->rx_desc = (mal_desc_t *) ((char *) rx_virt_addr +
				      (ep->emac_num * PAGE_SIZE));

	/* Fill in the transmit descriptor ring. */
	for (loop = 0; loop < NUM_TX_BUFF; loop++) {
		ep->tx_skb[loop] = (struct sk_buff *) NULL;
		ep->tx_desc[loop].ctrl = 0;
		ep->tx_desc[loop].data_len = 0;
		ep->tx_desc[loop].data_ptr = NULL;
	}
	ep->tx_desc[loop - 1].ctrl |= MAL_TX_CTRL_WRAP;

	/* Format the receive descriptor ring. */
	ep->rx_slot = 0;
	ppc405_rx_fill(dev, 0);
	if (ep->rx_slot != 0) {
		printk(KERN_ERR
		       "%s: Not enough mem for RxChain durning Open?\n",
		       dev->name);
		/*We couldn't fill the ring at startup?
		 *We could clean up and fail to open but right now we will try to
		 *carry on. It may be a sign of a bad NUM_RX_BUFF value
		 */
	}

	ep->tx_cnt = 0;
	ep->tx_slot = 0;
	ep->ack_slot = 0;
}

static int
ppc405_enet_open(struct net_device *dev)
{
	unsigned long mode_reg;
	struct ocp_enet_private *fep = dev->priv;
	volatile emac_t *emacp = fep->emacp;
	unsigned long emac_ier;

	if (!fep->link) {
		printk(KERN_NOTICE "%s: Cannot open interface without phy\n",
		       dev->name);
		return -ENODEV;
	}
	disable_mal_chan(fep);
	set_mal_chan_addr(fep);

	/* set the high address */
	out_be32(&emacp->em0iahr, (dev->dev_addr[0] << 8) | dev->dev_addr[1]);

	/* set the low address */
	out_be32(&emacp->em0ialr,
		 (dev->dev_addr[2] << 24) | (dev->dev_addr[3] << 16)
		 | (dev->dev_addr[4] << 8) | dev->dev_addr[5]);

	mii_do_cmd(dev, fep->phy->ack_int);
	mii_do_cmd(dev, fep->phy->config);
	mii_queue_schedule(dev);
	while (!fep->sequence_done)
		schedule();

	mii_display_status(dev);

	/* set receive fifo to 4k and tx fifo to 2k */
	mode_reg = EMAC_M1_RFS_4K | EMAC_M1_TX_FIFO_2K | EMAC_M1_APP |
	    EMAC_M1_TR0_MULTI;

	/* set speed (default is 10Mb) */
	if (fep->phy_speed == _100BASET) {
		mode_reg = mode_reg | EMAC_M1_MF_100MBPS;
		zmii_port_speed(100, dev);
	} else {
		mode_reg = mode_reg & ~EMAC_M1_MF_100MBPS;	/* 10 MBPS */
		zmii_port_speed(10, dev);
	}

	if (fep->phy_duplex == FULL)
		mode_reg = mode_reg | EMAC_M1_FDE | EMAC_M1_EIFC | EMAC_M1_IST;
	else
		mode_reg = mode_reg & ~(EMAC_M1_FDE | EMAC_M1_EIFC | EMAC_M1_ILE);	/* half duplex */

	out_be32(&emacp->em0mr1, mode_reg);

	/* enable broadcast and individual address */
	out_be32(&emacp->em0rmr, EMAC_RMR_IAE | EMAC_RMR_BAE);

	/* set transmit request threshold register */
	out_be32(&emacp->em0trtr, EMAC_TRTR_256);

	/* set receive low/high water mark register */
#ifdef CONFIG_440
	/* 440GP has a 64 byte burst length */
	out_be32(&emacp->em0rwmr, 0x80009000);
	out_be32(&emacp->em0tmr1, 0xf8640000);
#else
	/* 405s have a 16 byte burst length */
	out_be32(&emacp->em0rwmr, 0x0f002000);
#endif				/* CONFIG_440 */

	/* set frame gap */
	out_be32(&emacp->em0ipgvr, CONFIG_IBM_OCP_ENET_GAP);

	emac_ier = EMAC_ISR_PP | EMAC_ISR_BP | EMAC_ISR_RP |
	    EMAC_ISR_SE | EMAC_ISR_PTLE | EMAC_ISR_ALE |
	    EMAC_ISR_BFCS | EMAC_ISR_ORE | EMAC_ISR_IRE;

	out_be32(&emacp->em0iser, emac_ier);

	request_irq(ocp_get_irq(EMAC,fep->emac_num), ppc405_eth_mac, 0, "OCP EMAC MAC", dev);

	if (!(get_mal_dcrn(fep, DCRN_MALTXCASR))) {
 		request_irq(BL_MAC_WOL,ppc405_eth_wakeup,0,"OCP EMAC Wakeup",dev);
		request_irq(BL_MAL_SERR,ppc405_eth_serr,0,"OCP EMAC MAL SERR",dev);
		request_irq(BL_MAL_TXDE,ppc405_eth_txde,0,"OCP EMAC TX DE ",dev);
		request_irq(BL_MAL_RXDE,ppc405_eth_rxde,0,"OCP EMAC RX DE",dev);
		request_irq(BL_MAL_TXEOB,ppc405_eth_txeob,0,"OCP EMAC TX EOB",dev);
		request_irq(BL_MAL_RXEOB,ppc405_eth_rxeob,0,"OCP EMAC RX EOB",dev);
	}

	/* init buffer descriptors rings */
	init_rings(dev);

	/* enable all MAL transmit and receive channels */
	enable_mal_chan(fep);

	/* set transmit and receive enable */
	out_be32(&emacp->em0mr0, EMAC_M0_TXE | EMAC_M0_RXE);
	netif_start_queue(dev);
	printk(KERN_NOTICE "%s: IBM EMAC: open completed\n\n", dev->name);

	return 0;
}

static int
ppc405_enet_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	unsigned short ctrl;
	unsigned long flags;
	struct ocp_enet_private *fep = dev->priv;
	volatile emac_t *emacp = fep->emacp;

	save_flags(flags);
	cli();
	if (netif_queue_stopped(dev) || (fep->tx_cnt == NUM_TX_BUFF)) {

		fep->stats.tx_dropped++;
		restore_flags(flags);
		return -EBUSY;
	}

	if (++fep->tx_cnt == NUM_TX_BUFF)
		netif_stop_queue(dev);

	/* Store the skb buffer for later ack by the transmit end of buffer
	 * interrupt.
	 */
	fep->tx_skb[fep->tx_slot] = skb;
	consistent_sync((void *) skb->data, skb->len, PCI_DMA_TODEVICE);

	ctrl = EMAC_TX_CTRL_DFLT;
	if ((NUM_TX_BUFF - 1) == fep->tx_slot)
		ctrl |= MAL_TX_CTRL_WRAP;

	fep->tx_desc[fep->tx_slot].data_ptr = (char *) virt_to_phys(skb->data);
	fep->tx_desc[fep->tx_slot].data_len = (short) skb->len;
	fep->tx_desc[fep->tx_slot].ctrl = ctrl;

	/* Send the packet out. */
	out_be32(&emacp->em0tmr0, EMAC_TXM0_GNP0);

	if (++fep->tx_slot == NUM_TX_BUFF)
		fep->tx_slot = 0;

	fep->stats.tx_packets++;
	fep->stats.tx_bytes += skb->len;

	restore_flags(flags);

	return 0;
}

static int
ppc405_enet_close(struct net_device *dev)
{
	struct ocp_enet_private *fep = dev->priv;
	volatile emac_t *emacp = fep->emacp;

	disable_mal_chan(fep);

	out_be32(&emacp->em0mr0, EMAC_M0_SRST);
	udelay(10);

	if (emacp->em0mr0 & EMAC_M0_SRST) {
		/*not sure what to do here hopefully it clears before another open */
		printk(KERN_ERR "%s: Phy SoftReset didn't clear, no link?\n",
		       dev->name);
	}

	/* Free the irq's */
	free_irq(ocp_get_irq(EMAC,fep->emac_num), dev);

	if (!(get_mal_dcrn(fep, DCRN_MALTXCASR))) {
 		free_irq(BL_MAC_WOL,dev);
		free_irq(BL_MAL_SERR,dev);
		free_irq(BL_MAL_TXDE,dev);
		
		free_irq(BL_MAL_RXDE,dev);

		free_irq(BL_MAL_TXEOB,dev);
		free_irq(BL_MAL_RXEOB,dev);
	}


	free_phy(dev);
	return 0;
}

static void
ppc405_enet_set_multicast_list(struct net_device *dev)
{
	struct ocp_enet_private *ep = dev->priv;
	volatile emac_t *emacp = ep->emacp;

	if (dev->flags & IFF_PROMISC) {

		/* If promiscuous mode is set then we do not need anything else */
		out_be32(&emacp->em0rmr, EMAC_RMR_PME);

	} else if (dev->flags & IFF_ALLMULTI || 32 < dev->mc_count) {

		/* Must be setting up to use multicast.  Now check for promiscuous
		 * multicast
		 */
		out_be32(&emacp->em0rmr,
			 EMAC_RMR_IAE | EMAC_RMR_BAE | EMAC_RMR_PMME);
	} else if (dev->flags & IFF_MULTICAST && 0 < dev->mc_count) {

		unsigned short em0gaht[4] = { 0, 0, 0, 0 };
		struct dev_mc_list *dmi;

		/* Need to hash on the multicast address. */
		for (dmi = dev->mc_list; dmi; dmi = dmi->next) {
			unsigned long mc_crc;
			unsigned int bit_number;

			mc_crc = ether_crc(6, (char *) dmi->dmi_addr);
			bit_number = 63 - (mc_crc >> 26);	/* MSB: 0 LSB: 63 */
			em0gaht[bit_number >> 4] |=
			    0x8000 >> (bit_number & 0x0f);
		}
		emacp->em0gaht1 = em0gaht[0];
		emacp->em0gaht2 = em0gaht[1];
		emacp->em0gaht3 = em0gaht[2];
		emacp->em0gaht4 = em0gaht[3];

		/* Turn on multicast addressing */
		out_be32(&emacp->em0rmr,
			 EMAC_RMR_IAE | EMAC_RMR_BAE | EMAC_RMR_MAE);

	} else {
		/* If multicast mode is not set then we are 
		 * turning it off at this point 
		 */
		out_be32(&emacp->em0rmr, EMAC_RMR_IAE | EMAC_RMR_BAE);

	}

	return;
}

static struct net_device_stats *
ppc405_enet_stats(struct net_device *dev)
{
	struct ocp_enet_private *fep = dev->priv;
	return &fep->stats;
}

static void
ocp_remove_one(struct ocp_dev *pdev)
{
	struct net_device *dev = ocp_get_drvdata(pdev);
	printk("removing net dev \n");
	unregister_netdev(dev);

}

static int
ocp_enet_probe(int curr_emac)
{
	int ret = 1;
	int i;
	bd_t *bd;
	struct net_device *dev;
	struct ocp_enet_private *ep;
	struct ocp_dev *emac_driver;

	dev = init_etherdev(NULL, sizeof (struct ocp_enet_private));
	if (dev == NULL) {
		printk(KERN_ERR
		       "ibm_ocp_enet: Could not allocate ethernet device.\n");
		return -1;
	}
	ep = dev->priv;
	ep->emac_num = curr_emac;
	ep->mal = DCRN_MAL_BASE;
	ep->sequence_done = 0;

	emac_driver = &ep->ocpdev;
	emac_driver->type = EMAC;
	ocp_register(emac_driver);
	ocp_set_drvdata(emac_driver, dev);

	ep->emacp = (volatile emac_t *) __ioremap
	    (emac_driver->paddr, sizeof (emac_t), _PAGE_NO_CACHE);
	init_zmii(ZMII_AUTO, dev);
	find_phy(dev);
	if (!ep->phy) {
		ocp_remove_one(emac_driver);
		return -1;
	}
	ep->link = 1;
	ep->txchan = 0x80000000  >> curr_emac*2 ;
	ep->rxchan = 0x80000000 >> curr_emac;
	dev->irq = ocp_get_irq(EMAC,curr_emac);
	/* read the MAC Address */
	bd = (bd_t *) __res;
	for (i = 0; i < 6; i++)
		dev->dev_addr[i] = bd->BD_EMAC_ADDR(curr_emac, i);	/* Marco to disques array */

	/* Fill in the driver function table */
	dev->open = &ppc405_enet_open;
	dev->hard_start_xmit = &ppc405_enet_start_xmit;
	dev->stop = &ppc405_enet_close;
	dev->get_stats = &ppc405_enet_stats;
	dev->set_multicast_list = &ppc405_enet_set_multicast_list;
	dev->do_ioctl = &ocp_enet_ioctl;
	emac_dev[curr_emac] = dev;
	printk(KERN_NOTICE "Reset ethernet interfaces\n");

	/* Reset the EMAC */
	out_be32(&ep->emacp->em0mr0, EMAC_M0_SRST);
	udelay(10);

	if (in_be32(&ep->emacp->em0mr0) & EMAC_M0_SRST) {
		printk(KERN_NOTICE "%s: Cannot open interface without Link\n",
		       dev->name);
		return -1;
	}
	config_mal(ep);

	return (ret);

}
static int __init
init_ppc405_enet(void)
{
	int i, curr_emac;

	emac_max = ocp_get_max(EMAC);

	/* Allocate memory for the transmit descriptors and save its physical
	 * address.  EMACs share the upper 13 bits of address lines, so
	 * allocate one buffer big enough for everybody.
	 * FIXME: How to ensure aligned on 32768-byte boundary?
	 */

	tx_virt_addr = (mal_desc_t *)
	    consistent_alloc(GFP_KERNEL, PAGE_SIZE * emac_max, &tx_phys_addr);

	rx_virt_addr = (mal_desc_t *)
	    consistent_alloc(GFP_KERNEL, PAGE_SIZE * emac_max, &rx_phys_addr);
	for (curr_emac = 0; curr_emac < emac_max; curr_emac++) {
		if(ocp_get_pm(EMAC, curr_emac)){
			mtdcr(DCRN_CPMFR,
			      mfdcr(DCRN_CPMFR) & ~IBM_CPM_EMAC(ocp_get_pm(EMAC, curr_emac)));
		}
		ocp_enet_probe(curr_emac);
	}

	for (i = 0; i < NMII - 1; i++)
		mii_cmds[i].mii_next = &mii_cmds[i + 1];
	mii_free = mii_cmds;

	return 0;
}

/*
 * int ocp_enet_mdio_read()
 *
 * Description:
 *   This routine reads from a specified register on a PHY over the MII
 *   Management Interface.
 *
 * Input(s):
 *   phy    - The address of the PHY to read from. May be 0 through 31.
 *   reg    - The PHY register to read. May be 0 through 31.
 *   *value - Storage for the value to read from the PHY register.
 *
 * Output(s):
 *   *value - The value read from the PHY register.
 *
 * Returns:
 *   0 if OK, otherwise -1 on error.
 *
 */
int
ocp_enet_mdio_read(struct net_device *dev, int reg, uint * value)
{
	register int i;
	uint32_t stacr;
	struct ocp_enet_private *fep = dev->priv;
	volatile emac_t *emacp = fep->emacp;

	/* Wait for data transfer complete bit */
	enable_zmii_port(dev);
	for (i = 0; i < OCP_RESET_DELAY; ++i) {
		if (emacp->em0stacr & EMAC_STACR_OC)
			break;
		udelay(MDIO_DELAY);	/* changed to 2 with new scheme -armin */
	}
	if ((emacp->em0stacr & EMAC_STACR_OC) == 0) {
		printk("OCP Reset timeout #1!\n");
		return -1;
	}

	/* Clear the speed bits and make a read request to the PHY */

	stacr = reg | ((fep->phy_addr & 0x1F) << 5);

	out_be32(&emacp->em0stacr, stacr);
	stacr = in_be32(&emacp->em0stacr);
	/* Wait for data transfer complete bit */
	for (i = 0; i < OCP_RESET_DELAY; ++i) {
		if ((stacr = in_be32(&emacp->em0stacr)) & EMAC_STACR_OC)
			break;
		udelay(MDIO_DELAY);
	}
	if ((stacr & EMAC_STACR_OC) == 0) {
		printk("OCP Reset timeout #2!\n");
		return -1;
	}

	/* Check for a read error */
	if (stacr & EMAC_STACR_PHYE) {
		return -1;
	}
	*value = (stacr >> 16);
	return 0;
}

/*
 * int ocp_enet_mdio_write()
 *
 * Description:
 *   This routine reads from a specified register on a PHY over the MII
 *   Management Interface.
 *
 * Input(s):
 *   phy    - The address of the PHY to read from. May be 0 through 31.
 *   reg    - The PHY register to read. May be 0 through 31.
 *
 * Output(s):
 *   value - The value writing to the PHY register.
 *
 * Returns:
 *   0 if OK, otherwise -1 on error.
 *
 */
int
ocp_enet_mdio_write(struct net_device *dev, int reg)
{
	register int i = 0;
	uint32_t stacr;
	struct ocp_enet_private *fep = dev->priv;
	volatile emac_t *emacp = fep->emacp;

	enable_zmii_port(dev);
	/* Wait for data transfer complete bit */
	for (i = 0; i < OCP_RESET_DELAY; ++i) {
		if (emacp->em0stacr & EMAC_STACR_OC)
			break;
		udelay(MDIO_DELAY);	/* changed to 2 with new scheme -armin */
	}
	if ((emacp->em0stacr & EMAC_STACR_OC) == 0) {
		printk("OCP Reset timeout!\n");
		return -1;
	}

	/* Clear the speed bits and make a read request to the PHY */

	stacr = reg | ((fep->phy_addr & 0x1F) << 5);
	out_be32(&emacp->em0stacr, stacr);

	/* Wait for data transfer complete bit */
	for (i = 0; i < OCP_RESET_DELAY; ++i) {
		if ((stacr = emacp->em0stacr) & EMAC_STACR_OC)
			break;
		udelay(MDIO_DELAY);
	}
	if ((emacp->em0stacr & EMAC_STACR_OC) == 0) {
		printk("OCP Reset timeout!\n");
		return -1;
	}

	/* Check for a read error */
	if ((stacr & EMAC_STACR_PHYE) != 0) {
		return -1;
	}

	return 0;
}

/*
 * int ocp_enet_ioctl()
 *
 * Description:
 *   This routine performs the specified I/O control command on the
 *   specified net device.
 *
 * Input(s):
 *   *dev - Pointer to the device structure for this driver.
 *   *rq  - Pointer to data to be written and/or storage for return data.
 *    cmd - I/O control command to perform.
 *
 *
 * Output(s):
 *   *rq  - If OK, pointer to return data for a read command.
 *
 * Returns:
 *   0 if OK, otherwise an error number on error.
 *
 */
int
ocp_enet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct ocp_enet_private *fep = dev->priv;
	uint *data = (uint *) & rq->ifr_data;

	switch (cmd) {

	case SIOCDEVPRIVATE:
		data[0] = fep->phy_addr;
	 /*FALLTHRU*/ case SIOCDEVPRIVATE + 1:
		if (ocp_enet_mdio_read(dev, mk_mii_read(data[1]), &data[3]) < 0)
			return -EIO;

		return 0;

	case SIOCDEVPRIVATE + 2:
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;

		if (ocp_enet_mdio_write(dev, mk_mii_write(data[1], data[2])) <
		    0)
			return -EIO;

		return 0;

	default:
		return -EOPNOTSUPP;
	}
}

static void
ppc405_eth_wakeup(int irq, void *dev_instance, struct pt_regs *regs)
{
	/* On Linux the 405 ethernet will always be active if configured
	 * in.  This interrupt should never occur.
	 */
	printk(KERN_INFO "interrupt ppc405_eth_wakeup\n");
}

static void
ppc405_eth_serr(int irq, void *dev_instance, struct pt_regs *regs)
{
	struct net_device *dev = dev_instance;
	struct ocp_enet_private *fep = dev->priv;
	unsigned long mal_error;

	/*
	 * This SERR applies to one of the devices on the MAL, here we charge
	 * it against the first EMAC registered for the MAL.
	 */

	mal_error = get_mal_dcrn(fep, DCRN_MALESR);

	if (mal_error & MALESR_EVB) {

		if (mal_error & MALESR_CIDRX)
			fep->stats.rx_errors++;
		else
			fep->stats.tx_errors++;

		ppc405_serr_dump_0(dev);
	} else {
		/*
		 * Don't know whether it is transmit or receive error, blame
		 * both.
		 */
		fep->stats.rx_errors++;
		fep->stats.tx_errors++;
	}

	ppc405_serr_dump_1(dev);
	/* Clear the error status register */
	set_mal_dcrn(fep, DCRN_MALESR, mal_error);
}

static void
ppc405_eth_txeob_dev(struct net_device *dev)
{
	struct ocp_enet_private *fep = dev->priv;

	while (fep->tx_cnt &&
	       !(fep->tx_desc[fep->ack_slot].ctrl & MAL_TX_CTRL_READY)) {

		/* Tell the system the transmit completed. */
		dev_kfree_skb_irq(fep->tx_skb[fep->ack_slot]);

		if (fep->tx_desc[fep->ack_slot].ctrl &
		    (EMAC_TX_ST_EC | EMAC_TX_ST_MC | EMAC_TX_ST_SC))
			fep->stats.collisions++;

		fep->tx_skb[fep->ack_slot] = (struct sk_buff *) NULL;
		if (++fep->ack_slot == NUM_TX_BUFF)
			fep->ack_slot = 0;

		fep->tx_cnt--;

		netif_wake_queue(dev);
	}

	return;
}

static void
ppc405_eth_txeob(int irq, void *dev_instance, struct pt_regs *regs)
{
	int i, count, isr;
	struct ocp_enet_private *fep = ((struct net_device*)dev_instance)->priv;


	for (count = 0; count < 2; ++count) {
		isr = get_mal_dcrn(fep, DCRN_MALTXEOBISR);
		if (isr == 0)
			break;
		set_mal_dcrn(fep, DCRN_MALTXEOBISR, isr);

		for (i = 0; i < emac_max; i++) {
			if (isr & 0x80000000 >> i*2) {
				ppc405_eth_txeob_dev(emac_dev[i]);
			}
		}
	}
}

/*
  Fill/Re-fill the rx chain with valid ctrl/ptrs.
  This function will fill from rx_slot up to the parm end.
  So to completely fill the chain pre-set rx_slot to 0 and
  pass in an end of 0.
 */
static void
ppc405_rx_fill(struct net_device *dev, int end)
{
	int i;
	struct ocp_enet_private *fep = dev->priv;
	unsigned char *ptr;

	i = fep->rx_slot;
	do {
		if (fep->rx_skb[i] != NULL) {
			/*We will trust the skb is still in a good state */
			ptr = (char *) virt_to_phys(fep->rx_skb[i]->data);
		} else {

			fep->rx_skb[i] =
			    dev_alloc_skb(DESC_RX_BUF_SIZE + skb_res);

			if (fep->rx_skb[i] == NULL) {
				/* Keep rx_slot here, the next time clean/fill is called
				 * we will try again before the MAL wraps back here
				 * If the MAL tries to use this descriptor with
				 * the EMPTY bit off it will cause the
				 * rxde interrupt.  That is where we will
				 * try again to allocate an sk_buff.
				 */
				break;

			}

			if (skb_res)
				skb_reserve(fep->rx_skb[i], skb_res);

			consistent_sync((void *) fep->rx_skb[i]->
					data, DESC_RX_BUF_SIZE,
					PCI_DMA_BIDIRECTIONAL);
			ptr = (char *) virt_to_phys(fep->rx_skb[i]->data);
		}
		fep->rx_desc[i].ctrl = MAL_RX_CTRL_EMPTY | MAL_RX_CTRL_INTR |	/*could be smarter about this to avoid ints at high loads */
		    (i == (NUM_RX_BUFF - 1) ? MAL_RX_CTRL_WRAP : 0);

		fep->rx_desc[i].data_ptr = ptr;
		/*
		   * 440GP uses the previously reserved bits in the
		   * data_len to encode the upper 4-bits of the buffer
		   * physical address (ERPN). Initialize these.
		 */
		fep->rx_desc[i].data_len = 0;
	} while ((i = (i + 1) % NUM_RX_BUFF) != end);

	fep->rx_slot = i;
}

static void 
ppc405_rx_clean(struct net_device *dev, int call_rx_fill)
{
	int i;
	int error, frame_length;
	struct ocp_enet_private *fep = dev->priv;
	unsigned short ctrl;
	int slots_walked = 0;

	i = fep->rx_slot;

	do {
		if (fep->rx_skb[i] == NULL)
			goto skip;	/*we have already handled the packet but haved failed to alloc */
		/* 
		   since rx_desc is in uncached mem we don't keep reading it directly 
		   we pull out a local copy of ctrl and do the checks on the copy.
		 */
		ctrl = fep->rx_desc[i].ctrl;
		if (ctrl & MAL_RX_CTRL_EMPTY)
			break;	/*we don't have any more ready packets */

		if (ctrl & EMAC_BAD_RX_PACKET) {

			fep->stats.rx_errors++;
			fep->stats.rx_dropped++;

			if (ctrl & EMAC_RX_ST_OE)
				fep->stats.rx_fifo_errors++;
			if (ctrl & EMAC_RX_ST_AE)
				fep->stats.rx_frame_errors++;
			if (ctrl & EMAC_RX_ST_BFCS)
				fep->stats.rx_crc_errors++;
			if (ctrl & (EMAC_RX_ST_RP | EMAC_RX_ST_PTL |
				    EMAC_RX_ST_ORE | EMAC_RX_ST_IRE))
				fep->stats.rx_length_errors++;
		} else {

			/* Send the skb up the chain. */
			frame_length = fep->rx_desc[i].data_len - 4;

			skb_put(fep->rx_skb[i], frame_length);
			fep->rx_skb[i]->dev = dev;
			fep->rx_skb[i]->protocol =
			    eth_type_trans(fep->rx_skb[i], dev);

			error = netif_rx(fep->rx_skb[i]);
			if ((error == NET_RX_DROP) || (error == NET_RX_BAD)) {
				fep->stats.rx_dropped++;
			} else {
				fep->stats.rx_packets++;
				fep->stats.rx_bytes += frame_length;
			}
			fep->rx_skb[i] = NULL;
		}
	      skip:
		++slots_walked;

	} while ((i = (i + 1) % NUM_RX_BUFF) != fep->rx_slot);

	if (slots_walked && call_rx_fill)
           ppc405_rx_fill(dev, i);
}

static void
ppc405_eth_rxeob(int irq, void *dev_instance, struct pt_regs *regs)
{
	int i, isr;
	struct ocp_enet_private *fep = ((struct net_device*)dev_instance)->priv;

	/*
	 * Protect against ppc405_eth_rxde() modifying data structures
	 * this function is using.
	 */

	disable_irq(BL_MAL_RXDE);

	isr = get_mal_dcrn(fep, DCRN_MALRXEOBISR);
	set_mal_dcrn(fep, DCRN_MALRXEOBISR, isr);

	/* This determines which emac is sending the interrupt */
	for (i = 0; i < emac_max; i++) {
	 if (isr & 0x80000000 >> i)
                        ppc405_rx_clean(emac_dev[i], 1);
	}

	enable_irq(BL_MAL_RXDE);
	return;
}

/*
 * This interrupt should never occurr, we don't program
 * the MAL for contiunous mode.
 */
static void
ppc405_eth_txde(int irq, void *dev_instance, struct pt_regs *regs)
{
	struct net_device *dev = dev_instance;
	struct ocp_enet_private *fep = dev->priv;

	printk(KERN_WARNING "%s: transmit descriptor error\n",
	       "PPC 405 MAL0 eth");

	ppc405_eth_emac_dump(dev);
	ppc405_eth_mal_dump(dev);

	/* Reenable the transmit channel */
	set_mal_dcrn(fep, DCRN_MALTXCASR, get_mal_dcrn(fep, DCRN_MALTXCASR));
	return;
}

/*
 * This interrupt should be very rare at best.  This occurs when
 * the hardware has a problem with the receive descriptors.  The manual
 * states that it occurs when the hardware cannot the receive descriptor
 * empty bit is not set.  The recovery mechanism will be to
 * traverse through the descriptors, handle any that are marked to be
 * handled and reinitialize each along the way.  At that point the driver
 * will be restarted.
 */
static void
ppc405_eth_rxde(int irq, void *dev_instance, struct pt_regs *regs)
{
	int i, isr;
 	struct net_device *dev = dev_instance;
       	struct ocp_enet_private *fep = dev->priv;

	/*
	 * This really is needed.  This case encountered in stress testing.
	 */
	if (get_mal_dcrn(fep, DCRN_MALRXDEIR) == 0)
		return;

	printk("%s: receive descriptor error\n", "PPC 405 MAL0 eth");

	ppc405_eth_emac_dump(dev);
	ppc405_eth_mal_dump(dev);
	ppc405_eth_desc_dump(dev);

	/*
	 * Protect against ppc405_eth_rxeob modifying these same
	 * structures.  If an rxde interrupt occurs the hardware will
	 * have disabled that EMAC, but since there may be multiple
	 * EMACs on the same MAL another rxeob interrupt could occur
	 * for another EMAC prior to ppc405_eth_rxde() re-enabling
	 * EMACs below.
	 */

	disable_irq(BL_MAL_RXEOB);
	for (i = 0; i < emac_max; i++) {
		isr = get_mal_dcrn(fep, DCRN_MALRXEOBISR);
		if (isr & 0x80000000 >> i) {
			dev = emac_dev[i];
			fep = dev->priv;
			/* For now, charge the error against all emacs */
			fep->stats.rx_errors++;

			/* so do we have any good packets still? */
			ppc405_rx_clean(dev,0);

			/* When the interface is restarted it resets processing to the
			 *  first descriptor in the table.
			 */

			fep->rx_slot = 0;
			ppc405_rx_fill(dev, 0);

			set_mal_dcrn(fep, DCRN_MALRXEOBISR, 0x80000000 >> i);
		}
	}

	/* Clear the interrupt */
	set_mal_dcrn(fep, DCRN_MALRXDEIR, get_mal_dcrn(fep, DCRN_MALRXDEIR));

	enable_irq(BL_MAL_RXEOB);

	/* Reenable the receive channels */
	set_mal_dcrn(fep, DCRN_MALRXCASR, get_mal_dcrn(fep, DCRN_MALRXCASR));
}

static void
ppc405_eth_mac(int irq, void *dev_instance, struct pt_regs *regs)
{
	unsigned long tmp_em0isr;
	struct ocp_enet_private *fep = ((struct net_device*)dev_instance)->priv;
	volatile emac_t *emacp = fep->emacp;

	/* EMAC interrupt */
	tmp_em0isr = in_be32(&emacp->em0isr);
	if (tmp_em0isr & (EMAC_ISR_TE0 | EMAC_ISR_TE1)) {
		/* This error is a hard transmit error - could retransmit */
		fep->stats.tx_errors++;

		/* Reenable the transmit channel */
		set_mal_dcrn(fep, DCRN_MALTXCASR, fep->txchan);

	} else {
		fep->stats.rx_errors++;
	}

/*	if (tmp_em0isr & EMAC_ISR_OVR ) fep->stats.ZZZ++;		*/
/*	if (tmp_em0isr & EMAC_ISR_PP ) fep->stats.ZZZ++;		*/
/*	if (tmp_em0isr & EMAC_ISR_BP ) fep->stats.ZZZ++;		*/
	if (tmp_em0isr & EMAC_ISR_RP)
		fep->stats.rx_length_errors++;
/*	if (tmp_em0isr & EMAC_ISR_SE ) fep->stats.ZZZ++;		*/
	if (tmp_em0isr & EMAC_ISR_ALE)
		fep->stats.rx_frame_errors++;
	if (tmp_em0isr & EMAC_ISR_BFCS)
		fep->stats.rx_crc_errors++;
	if (tmp_em0isr & EMAC_ISR_PTLE)
		fep->stats.rx_length_errors++;
	if (tmp_em0isr & EMAC_ISR_ORE)
		fep->stats.rx_length_errors++;
/*	if (tmp_em0isr & EMAC_ISR_IRE ) fep->stats.ZZZ++;		*/
/*	if (tmp_em0isr & EMAC_ISR_DBDM) fep->stats.ZZZ++;		*/
/*	if (tmp_em0isr & EMAC_ISR_DB0 ) fep->stats.ZZZ++;		*/
/*	if (tmp_em0isr & EMAC_ISR_SE0 ) fep->stats.ZZZ++;		*/
	if (tmp_em0isr & EMAC_ISR_TE0)
		fep->stats.tx_aborted_errors++;
/*	if (tmp_em0isr & EMAC_ISR_DB1 ) fep->stats.ZZZ++;		*/
/*	if (tmp_em0isr & EMAC_ISR_SE1 ) fep->stats.ZZZ++;		*/
/*	if (tmp_em0isr & EMAC_ISR_TE1) fep->stats.ZZZ++;		*/
/*	if (tmp_em0isr & EMAC_ISR_MOS ) fep->stats.ZZZ++;		*/
/*	if (tmp_em0isr & EMAC_ISR_MOF ) fep->stats.ZZZ++;		*/

	ppc405_bl_mac_eth_dump(dev, tmp_em0isr);

	out_be32(&emacp->em0isr, tmp_em0isr);
}

static void __exit
exit_ppc405_enet(void)
{
	int i;
	struct ocp_dev *emac_dev;

	for (i = 0; i < ocp_get_max(EMAC); i++) {
		emac_dev = ocp_get_dev(EMAC, i);
		if(ocp_get_pm(EMAC, i)) {
			mtdcr(DCRN_CPMFR, mfdcr(DCRN_CPMFR)
					| IBM_CPM_EMAC(ocp_get_pm(EMAC, i)));
		}
		ocp_unregister(emac_dev);
	}

	/*
	 * Unmap the non cached memory space.
	 */
	consistent_free((void *) tx_virt_addr);
	consistent_free((void *) rx_virt_addr);

}

module_init(init_ppc405_enet);
module_exit(exit_ppc405_enet);
