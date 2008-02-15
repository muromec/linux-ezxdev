/*
 * ibm_ocp_enet.h
 *
 * Ethernet driver for the built in ethernet on the IBM 405 PowerPC
 * processor.
 *
 *      Armin Kuster akuster@mvista.com
 *      Sept, 2001
 *
 *      Orignial driver
 *         Johnnie Peters
 *         jpeters@mvista.com
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
 *  Version: 1.0: Name change - armin
 *  Version: 1.0: Added extern prototypes for new mal func. - David M.
 *  		: removed #ifdef for irqs.i - Armin
 *  		: added irq_resourse to private data struct. 
 *
 *  Version: 1.1: Added new min/max value for phy & removed unused phy_id_done - *			- andrew May
 *  Version: 1.2: added all emac extern protos here - Armin
 *
 *  Version: 1.3: Using CONFIG_IBM_OCP_ZMII instead of ZMII_NUMS > 0
 *
 *  Version: 1.3: Name change *_driver to *_dev
 *  Version: 1.4: removed irq_resource & BL_* defines
 *
 */

#ifndef _IBM_OCP_ENET_H_
#define _IBM_OCP_ENET_H_

#include <linux/netdevice.h>
#include <asm/ocp.h>
#include <asm/mmu.h>		/* For phys_addr_t */
#include "ibm_ocp_emac.h"
#include "ibm_ocp_phy.h"

#ifndef CONFIG_IBM_OCP_ENET_TX_BUFF
#define NUM_TX_BUFF		6
#define NUM_RX_BUFF		64
#else
#define NUM_TX_BUFF		CONFIG_IBM_OCP_ENET_TX_BUFF
#define NUM_RX_BUFF		CONFIG_IBM_OCP_ENET_RX_BUFF
#endif

/* This does 16 byte alignment, exactly what we need.
 * The packet length includes FCS, but we don't want to
 * include that when passing upstream as it messes up
 * bridging applications.
 */
#ifndef CONFIG_IBM_OCP_ENET_SKB_RES
#define SKB_RES 2
#else
#define SKB_RES CONFIG_IBM_OCP_ENET_SKB_RES
#endif

#define MAX_NUM_BUF_DESC	255
#define DECS_TX_BUF_SIZE	2048
#define DESC_RX_BUF_SIZE	2048	/* max 4096-16 */
#define DESC_BUF_SIZE		2048
#define DESC_BUF_SIZE_REG	(DESC_RX_BUF_SIZE / 16)


#define LAST_EMAC		EMAC_NUMS -1

#define MIN_PHY_ADDR            0x00
#define MAX_PHY_ADDR            0x1f
/* Transmitter timeout. */
#define TX_TIMEOUT		(2*HZ)
#define OCP_RESET_DELAY		50
#define MDIO_DELAY		2
#define NMII				20

/* Power managment shift registers */
#define IBM_CPM_EMMII	0	/* Shift value for MII */
#define IBM_CPM_EMRX	1	/* Shift value for recv */
#define IBM_CPM_EMTX	2	/* Shift value for MAC */
#define IBM_CPM_EMAC(x)	(((x)>>IBM_CPM_EMMII) | ((x)>>IBM_CPM_EMRX) | ((x)>>IBM_CPM_EMTX))

#ifdef CONFIG_IBM_OCP_ENET_ERROR_MSG
void ppc405_serr_dump_0(struct net_device *dev);
void ppc405_serr_dump_1(struct net_device *dev);
void ppc405_bl_mac_eth_dump(struct net_device *dev, int em0isr);
void  ppc405_phy_dump(struct net_device *);
void ppc405_eth_desc_dump(struct net_device *);
void ppc405_eth_emac_dump(struct net_device *);
void ppc405_eth_mal_dump(struct net_device *);
#else
#define ppc405_serr_dump_0(dev) do { } while (0)
#define ppc405_serr_dump_1(dev) do { } while (0)
#define ppc405_bl_mac_eth_dump(dev,x) do { } while (0)
#define ppc405_phy_dump(dev) do { } while (0)
#define ppc405_eth_desc_dump(dev) do { } while (0)
#define ppc405_eth_emac_dump(dev) do { } while (0)
#define ppc405_eth_mal_dump(dev) do { } while (0)
#endif

#define mk_mii_read(REG)		((EMAC_STACR_READ| (REG & 0x1f)) & \
						~EMAC_STACR_CLK_100MHZ)
#define mk_mii_write(REG,VAL)		(((EMAC_STACR_WRITE | (REG & 0x1f)) & \
						~EMAC_STACR_CLK_100MHZ) | \
						((VAL & 0xffff) << 16))

/* MAL Buffer Descriptor structure */
typedef struct {
	volatile unsigned short ctrl;	/* MAL / Commac status control bits */
	volatile short data_len;	/* Max length is 4K-1 (12 bits)     */
	unsigned char *data_ptr;	/* pointer to actual data buffer    */
} mal_desc_t;



struct ocp_enet_private {
	uint emac_num;
	uint txchan;
	uint rxchan;
	struct sk_buff *tx_skb[NUM_TX_BUFF];
	struct sk_buff *rx_skb[NUM_RX_BUFF];
	mal_desc_t *tx_desc;
	mal_desc_t *rx_desc;
	mal_desc_t *rx_dirty;
	struct net_device_stats stats;
	int tx_cnt;
	int rx_slot;
	int dirty_rx;
	int tx_slot;
	int ack_slot;
	uint phy_id;
	uint phy_id_done;
	uint phy_status;
	uint phy_speed;
	uint phy_duplex;
	phy_info_t *phy;
	struct tq_struct phy_task;
	uint sequence_done;
	uint phy_addr;
	int link;
	int old_link;
	int full_duplex;
	zmii_t *zmii_base;
	int zmii_mode;
	int mal;
	volatile emac_t *emacp;
	struct ocp_dev ocpdev;
};


#endif				/* _IBM_OCP_ENET_H_ */
