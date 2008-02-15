/*
 *
 * BRIEF MODULE DESCRIPTION
 *      Ethernet registers on IDT RC32355 (Banyan)
 *
 * Copyright 2002 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *         	stevel@mvista.com or source@mvista.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
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
 */

#ifndef BANYAN_ETHER_H
#define BANYAN_ETHER_H

#include <asm/rc32300/rc32355_dma.h>

/*
 * A partial image of the RC32355 ethernet registers
 */
typedef struct {
	u32 ethintfc;
	u32 ethfifott;
	u32 etharc;
	u32 ethhash0;
	u32 ethhash1;
	u32 ethfifost;
	u32 ethfifos;
	u32 ethodeops;
	u32 ethis;
	u32 ethos;
	u32 ethmcp;
	u32 _u1;
	u32 ethid;
	u32 _u2;
	u32 _u3;
	u32 _u4;
	u32 ethod;
	u32 _u5;
	u32 _u6;
	u32 _u7;
	u32 ethodeop;
	u32 _u8[43]; 
	u32 ethsal0;
	u32 ethsah0;
	u32 ethsal1;
	u32 ethsah1;
	u32 ethsal2;
	u32 ethsah2;
	u32 ethsal3;
	u32 ethsah3;
	u32 ethrbc;
	u32 ethrpc;
	u32 ethrupc;
	u32 ethrfc;
	u32 ethtbc;
	u32 ethgpf;
	u32 _u9[50];
	u32 ethmac1;
	u32 ethmac2;
	u32 ethipgt;
	u32 ethipgr;
	u32 ethclrt;
	u32 ethmaxf;
	u32 _u10;
	u32 ethmtest;
	u32 miimcfg;
	u32 miimcmd;
	u32 miimaddr;
	u32 miimwtd;
	u32 miimrdd;
	u32 miimind;
	u32 _u11;
	u32 _u12;
	u32 ethcfsa0;
	u32 ethcfsa1;
	u32 ethcfsa2;
} rc32355_eth_regs_t;
 
#define rc32355_eth_regs ((rc32355_eth_regs_t*)KSEG1ADDR(RC32355_ETH_BASE))

#define ETH_INTFC   (RC32355_ETH_BASE + 0x000) /* INTerFace Control  */
#define ETH_FIFOTT  (RC32355_ETH_BASE + 0x004) /* FIFO Transmit Threshold  */
#define ETH_ARC     (RC32355_ETH_BASE + 0x008) /* Address Recognition Ctrl  */
#define ETH_HASH0   (RC32355_ETH_BASE + 0x00C) /* 32 multicast Hash bits */
#define ETH_HASH1   (RC32355_ETH_BASE + 0x010) /* another 32 Hash bits */
#define ETH_FIFOST  (RC32355_ETH_BASE + 0x014) /* FIFO Status Threshold */
#define ETH_FIFOS   (RC32355_ETH_BASE + 0x018) /* FIFO Status Register */
#define ETH_ODEOPS  (RC32355_ETH_BASE + 0x01C) /* Out Data End-Of-Pkt Size */
#define ETH_IS      (RC32355_ETH_BASE + 0x020) /* Input Status */
#define ETH_OS      (RC32355_ETH_BASE + 0x024) /* Output Status  */
#define ETH_MCP     (RC32355_ETH_BASE + 0x028) /* Managemt Clock Prescaler */
#define ETH_ID      (RC32355_ETH_BASE + 0x030) /* Input Data register */
#define ETH_OD      (RC32355_ETH_BASE + 0x040) /* Output Data register */
#define ETH_ODEOP   (RC32355_ETH_BASE + 0x050) /* OD End-Of-Packet Size */

/* for n in { 0, 1, 2, 3 } */
#define ETH_SAL(n)  (RC32355_ETH_BASE + 0x100 + (n * 8)) /* Stn Address 2-5 */
#define ETH_SAH(n)  (RC32355_ETH_BASE + 0x104 + (n * 8)) /* Stn Address 0-1 */

#define ETH_RBC     (RC32355_ETH_BASE + 0x120) /* Receive Byte Count */
#define ETH_RPC     (RC32355_ETH_BASE + 0x124) /* Receive Packet Count */
#define ETH_RUPC    (RC32355_ETH_BASE + 0x128) /* Rx Undersized Pkt count */
#define ETH_RFC     (RC32355_ETH_BASE + 0x12C) /* Receive Fragment Count */
#define ETH_TBC     (RC32355_ETH_BASE + 0x130) /* Transmit Byte Count */
#define ETH_GPF     (RC32355_ETH_BASE + 0x134) /* Generate Pause Frame */
#define ETH_MAC1    (RC32355_ETH_BASE + 0x200) /* Medium Access Control 1 */
#define ETH_MAC2    (RC32355_ETH_BASE + 0x204) /* Medium Access Control 2 */
#define ETH_IPGT    (RC32355_ETH_BASE + 0x208) /* Back-to-back InterPkt Gap */
#define ETH_IPGR    (RC32355_ETH_BASE + 0x20C) /* Non " InterPkt Gap */
#define ETH_CLRT    (RC32355_ETH_BASE + 0x210) /* Collis'n Window and Retry */
#define ETH_MAXF    (RC32355_ETH_BASE + 0x214) /* Maximum Frame Length */
#define ETH_MTEST   (RC32355_ETH_BASE + 0x21C) /* MAC Test */

#define ETHMIIM_CFG (RC32355_ETH_BASE + 0x220) /* MII Mgmt Configuration */
#define ETHMIIM_CMD (RC32355_ETH_BASE + 0x224) /* MII Mgmt Command  */
#define ETHMIIM_ADDR (RC32355_ETH_BASE + 0x228) /* MII Mgmt Address */
#define ETHMIIM_WTD (RC32355_ETH_BASE + 0x22C) /* MII Mgmt Write Data */
#define ETHMIIM_RDD (RC32355_ETH_BASE + 0x230) /* MII Mgmt Read Data */
#define ETHMIIM_IND (RC32355_ETH_BASE + 0x234) /* MII Mgmt Indicators */

/* for n in { 0, 1, 2 } */
#define ETH_CFSA(n) (RC32355_ETH_BASE + 0x240 + ((n) * 4))  /* Station Addr */


/*
 * Register Interpretations follow
 */

/******************************************************************************
 * ETHINTFC register
 *****************************************************************************/

#define ETHERINTFC_EN            (1<<0)
#define ETHERINTFC_ITS           (1<<1)
#define ETHERINTFC_RES           (1<<2)
#define ETHERINTFC_RIP           (1<<2)
#define ETHERINTFC_JAM           (1<<3)

/******************************************************************************
 * ETHFIFOTT register
 *****************************************************************************/

#define ETHERFIFOTT_TTH(v)      (((v)&0x3f)<<0)

/******************************************************************************
 * ETHARC register
 *****************************************************************************/

#define ETHERARC_PRO             (1<<0)
#define ETHERARC_AM              (1<<1)
#define ETHERARC_AFM             (1<<2)
#define ETHERARC_AB              (1<<3)

/******************************************************************************
 * ETHHASH registers
 *****************************************************************************/

#define ETHERHASH0(v)            (((v)&0xffff)<<0)
#define ETHERHASH1(v)            (((v)&0xffff)<<0)

/******************************************************************************
 * ETHSA registers
 *****************************************************************************/

#define ETHERSAL0(v)             (((v)&0xffff)<<0)
#define ETHERSAL1(v)             (((v)&0xffff)<<0)
#define ETHERSAL2(v)             (((v)&0xffff)<<0)
#define ETHERSAL3(v)             (((v)&0xffff)<<0)
#define ETHERSAH0(v)             (((v)&0xff)<<0)
#define ETHERSAH1(v)             (((v)&0xff)<<0)
#define ETHERSAH2(v)             (((v)&0xff)<<0)
#define ETHERSAH3(v)             (((v)&0xff)<<0)

/******************************************************************************
 * ETHFIFOST register
 *****************************************************************************/

#define ETHERFIFOST_IRTH(v)      (((v)&0x3f)<<0)
#define ETHERFIFOST_ORTH(v)      (((v)&0x3f)<<16)

/******************************************************************************
 * ETHFIFOS register
 *****************************************************************************/

#define ETHERFIFOS_IR            (1<<0)
#define ETHERFIFOS_OR            (1<<1)  
#define ETHERFIFOS_OVR           (1<<2)  
#define ETHERFIFOS_UND           (1<<3)  

/******************************************************************************
 * DATA registers
 *****************************************************************************/

#define ETHERID(v)               (((v)&0xffff)<<0)
#define ETHEROD(v)               (((v)&0xffff)<<0)

/******************************************************************************
 * ETHODEOPS register
 *****************************************************************************/

#define ETHERODEOPS_SIZE(v)      (((v)&0x3)<<0)

/******************************************************************************
 * ETHODEOP register
 *****************************************************************************/

#define ETHERODEOP(v)            (((v)&0xffff)<<0)

/******************************************************************************
 * ETHIS register
 *****************************************************************************/

#define ETHERIS_EOP              (1<<0)  
#define ETHERIS_ROK              (1<<2)  
#define ETHERIS_FM               (1<<3)  
#define ETHERIS_MP               (1<<4)  
#define ETHERIS_BP               (1<<5)  
#define ETHERIS_VLT              (1<<6)  
#define ETHERIS_CF               (1<<7)  
#define ETHERIS_OVR              (1<<8)  
#define ETHERIS_CRC              (1<<9)  
#define ETHERIS_CV               (1<<10)  
#define ETHERIS_DB               (1<<11)  
#define ETHERIS_LE               (1<<12)  
#define ETHERIS_LOR              (1<<13)  
#define ETHERIS_SIZE(v)          (((v)&0x3)<<14)
#define ETHERIS_LENGTH(v)        (((v)&0xff)<<16)

/******************************************************************************
 * ETHOS register
 *****************************************************************************/

#define ETHEROS_T                (1<<0)  
#define ETHEROS_TOK              (1<<6)  
#define ETHEROS_MP               (1<<7)  
#define ETHEROS_BP               (1<<8)  
#define ETHEROS_UND              (1<<9)  
#define ETHEROS_OF               (1<<10)  
#define ETHEROS_ED               (1<<11)  
#define ETHEROS_EC               (1<<12)  
#define ETHEROS_LC               (1<<13)  
#define ETHEROS_TD               (1<<14)  
#define ETHEROS_CRC              (1<<15)  
#define ETHEROS_LE               (1<<16)  
#define ETHEROS_CC(v)            (((v)&0xf)<<17)
#define ETHEROS_PFD              (1<<21)  

/******************************************************************************
 * Statistics registers
 *****************************************************************************/

#define ETHERRBC(v)              (((v)&0xffff)<<0)
#define ETHERRPC(v)              (((v)&0xffff)<<0)
#define ETHERRUPC(v)             (((v)&0xffff)<<0)
#define ETHERRFC(v)              (((v)&0xffff)<<0)
#define ETHERTBC(v)              (((v)&0xffff)<<0)

/******************************************************************************
 * ETHGPF register
 *****************************************************************************/

#define ETHERGPF_PTV(v)          (((v)&0xff)<<0)

/******************************************************************************
 * MAC registers
 *****************************************************************************/
//ETHMAC1
#define ETHERMAC1_RE             (1<<0)
#define ETHERMAC1_PAF            (1<<1)
#define ETHERMAC1_RFC            (1<<2)
#define ETHERMAC1_TFC            (1<<3)
#define ETHERMAC1_LB             (1<<4)
#define ETHERMAC1_MR             (1<<15)

//ETHMAC2
#define ETHERMAC2_FD             (1<<0)
#define ETHERMAC2_FLC            (1<<1)
#define ETHERMAC2_HFE            (1<<2)
#define ETHERMAC2_DC             (1<<3)
#define ETHERMAC2_CEN            (1<<4)
#define ETHERMAC2_PE             (1<<5)
#define ETHERMAC2_VPE            (1<<6)
#define ETHERMAC2_APE            (1<<7)
#define ETHERMAC2_PPE            (1<<8)
#define ETHERMAC2_LPE            (1<<9)
#define ETHERMAC2_NB             (1<<12)
#define ETHERMAC2_BP             (1<<13)
#define ETHERMAC2_ED             (1<<14)

//ETHIPGT
#define ETHERIPGT(v)             (((v)&0x3f)<<0)

//ETHIPGR
#define ETHERIPGR_IPGR1(v)       (((v)&0x3f)<<0)
#define ETHERIPGR_IPGR2(v)       (((v)&0x3f)<<8)

//ETHCLRT
#define ETHERCLRT_MAXRET(v)      (((v)&0x3f)<<0)
#define ETHERCLRT_COLWIN(v)      (((v)&0x3f)<<8)

//ETHMAXF
#define ETHERMAXF(v)             (((v)&0x3f)<<0)

//ETHMTEST
#define ETHERMTEST_TB            (1<<2)

//ETHMCP
#define ETHERMCP_DIV(v)          (((v)&0xff)<<0)

//MIIMCFG
#define ETHERMIIMCFG_CS(v)          (((v)&0x3)<<2)
#define ETHERMIIMCFG_R              (1<<15)

//MIIMCMD
#define ETHERMIIMCMD_RD             (1<<0)
#define ETHERMIIMCMD_SCN            (1<<1)

//MIIMADDR
#define ETHERMIIMADDR_REGADDR(v)    (((v)&0x1f)<<0)
#define ETHERMIIMADDR_PHYADDR(v)    (((v)&0x1f)<<8)

//MIIMWTD
#define ETHERMIIMWTD(v)             (((v)&0xff)<<0)

//MIIMRDD
#define ETHERMIIMRDD(v)             (((v)&0xff)<<0)

//MIIMIND
#define ETHERMIIMIND_BSY            (1<<0)
#define ETHERMIIMIND_SCN            (1<<1)
#define ETHERMIIMIND_NV             (1<<2)

//DMA DEVCS IN
#define ETHERDMA_IN_LENGTH(v)	(((v)&0xffff)<<16)
#define ETHERDMA_IN_CES		(1<<14)
#define ETHERDMA_IN_LOR		(1<<13)
#define ETHERDMA_IN_LE		(1<<12)
#define ETHERDMA_IN_DB		(1<<11)
#define ETHERDMA_IN_CV		(1<<10)
#define ETHERDMA_IN_CRC		(1<<9)
#define ETHERDMA_IN_OVR		(1<<8)
#define ETHERDMA_IN_CF		(1<<7)
#define ETHERDMA_IN_VLT		(1<<6)
#define ETHERDMA_IN_BP		(1<<5)
#define ETHERDMA_IN_MP		(1<<4)
#define ETHERDMA_IN_FM		(1<<3)
#define ETHERDMA_IN_ROK		(1<<2)
#define ETHERDMA_IN_LD		(1<<1)
#define ETHERDMA_IN_FD		(1<<0)

//DMA DEVCS OUT
#define ETHERDMA_OUT_CC(v)	(((v)&0xf)<<17)
#define ETHERDMA_OUT_CNT         0x001e0000
#define ETHERDMA_OUT_SHFT       17
#define ETHERDMA_OUT_LE		(1<<16)

#define ETHERDMA_OUT_CRC	(1<<15)
#define ETHERDMA_OUT_TD		(1<<14)
#define ETHERDMA_OUT_LC		(1<<13)
#define ETHERDMA_OUT_EC		(1<<12)
#define ETHERDMA_OUT_ED		(1<<11)
#define ETHERDMA_OUT_OF		(1<<10)
#define ETHERDMA_OUT_UND	(1<<9)
#define ETHERDMA_OUT_BP		(1<<8)
#define ETHERDMA_OUT_MP		(1<<7)
#define ETHERDMA_OUT_TOK	(1<<6)
#define ETHERDMA_OUT_HEN	(1<<5)
#define ETHERDMA_OUT_CEN	(1<<4)
#define ETHERDMA_OUT_PEN	(1<<3)
#define ETHERDMA_OUT_OEN	(1<<2)
#define ETHERDMA_OUT_LD		(1<<1)
#define ETHERDMA_OUT_FD		(1<<0)

#define RCV_ERRS \
  (ETHERDMA_IN_OVR | ETHERDMA_IN_CRC | ETHERDMA_IN_CV | ETHERDMA_IN_LE)
#define TX_ERRS  \
  (ETHERDMA_OUT_LC | ETHERDMA_OUT_EC | ETHERDMA_OUT_ED | \
   ETHERDMA_OUT_OF | ETHERDMA_OUT_UND)

#define IS_RCV_ROK(X)        (((X) & (1<<2)) >> 2)       /* Receive Okay     */
#define IS_RCV_FM(X)         (((X) & (1<<3)) >> 3)       /* Is Filter Match  */
#define IS_RCV_MP(X)         (((X) & (1<<4)) >> 4)       /* Is it MP         */
#define IS_RCV_BP(X)         (((X) & (1<<5)) >> 5)       /* Is it BP         */
#define IS_RCV_VLT(X)        (((X) & (1<<6)) >> 6)       /* VLAN Tag Detect  */
#define IS_RCV_CF(X)         (((X) & (1<<7)) >> 7)       /* Control Frame    */
#define IS_RCV_OVR_ERR(X)    (((X) & (1<<8)) >> 8)       /* Receive Overflow */
#define IS_RCV_CRC_ERR(X)    (((X) & (1<<9)) >> 9)       /* CRC Error        */
#define IS_RCV_CV_ERR(X)     (((X) & (1<<10))>>10)       /* Code Violation   */
#define IS_RCV_DB_ERR(X)     (((X) & (1<<11))>>11)       /* Dribble Bits     */
#define IS_RCV_LE_ERR(X)     (((X) & (1<<12))>>12)       /* Length error     */
#define IS_RCV_LOR_ERR(X)    (((X) & (1<<13))>>13)       /* Length Out of
                                                            Range            */
#define IS_RCV_CES_ERR(X)    (((X) & (1<<14))>>14)       /* Preamble error   */
#define RCVPKT_LENGTH(X)     (((X) & 0xFFFF0000)>>16)    /* Length of the
                                                            received packet  */

#define IS_TX_TOK(X)         (((X) & (1<<6) ) >> 6 )     /* Transmit Okay    */
#define IS_TX_MP(X)          (((X) & (1<<7) ) >> 7 )     /* Multicast        */

#define IS_TX_BP(X)          (((X) & (1<<8) ) >> 8 )     /* Broadcast        */
#define IS_TX_UND_ERR(X)     (((X) & (1<<9) ) >> 9 )     /* Transmit FIFO
                                                            Underflow        */
#define IS_TX_OF_ERR(X)      (((X) & (1<<10)) >>10 )     /* Oversized frame  */
#define IS_TX_ED_ERR(X)      (((X) & (1<<11)) >>11 )     /* Excessive
							    deferral        */
#define IS_TX_EC_ERR(X)      (((X) & (1<<12)) >>12 )     /* Excessive
							    collisions      */
#define IS_TX_LC_ERR(X)      (((X) & (1<<13)) >>13 )     /* Late Collision   */
#define IS_TX_TD_ERR(X)      (((X) & (1<<14)) >>14 )     /* Transmit deferred*/
#define IS_TX_CRC_ERR(X)     (((X) & (1<<15)) >>15 )     /* CRC Error        */
#define IS_TX_LE_ERR(X)      (((X) & (1<<16)) >>16 )     /* Length Error     */

#define TX_COLLISION_COUNT(X) (((X) & 0x001E0000u)>>17)  /* Collision Count  */

#endif /* BANYAN_ETHER_H */

