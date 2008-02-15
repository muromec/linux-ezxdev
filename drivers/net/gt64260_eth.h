/* Driver for EVB64260 ethernet ports
   Copyright (C)2000, 2001 Rabeeh Khoury, Marvell */
/*
 * drivers/net/gt64260_eth.h
 * 
 * Author: Rabeeh Khoury from Marvell
 * Modified by: Mark A. Greer <mgreer@mvista.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef GT64260_ETH_H
#define GT64260_ETH_H

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <asm/gt64260.h>

#define DESCRIPTOR_SIZE            sizeof(gt_dma_desc)
#define Q_INDEX_LIMIT              (PAGE_SIZE / DESCRIPTOR_SIZE)

#define NUM_TX_QUEUES              2
#define NUM_RX_QUEUES              4
#define GT64260_INT_THRE          5
#define GT_ENET_DESC_OWNERSHIP     (1<<31)
#define GT_ENET_DESC_INT_ENABLE    (1<<23)
#define GT_ENET_DESC_ERROR_SUMMARY (1<<15)
#define MAX_BUFF_SIZE              1536

#define ETH_ADDR_GAP ( GT64260_ENET_E1PCR - GT64260_ENET_E0PCR )

const char gt64260_eth0_name[] = "GEth0";
const char gt64260_eth1_name[] = "GEth1";

s32 gt64260_eth_init(struct net_device *);
s32 gt64260_eth_open(struct net_device *);
s32 gt64260_eth_start_xmit(struct sk_buff *, struct net_device *);
s32 gt64260_eth_stop(struct net_device *);
void gt64260_eth_int_handler(s32, void *, struct pt_regs *);
s32 gt64260_eth_set_mac_address(struct net_device *, void *);
struct net_device_stats *gt64260_eth_get_stats(struct net_device *);

typedef volatile struct gt_dma_desc_struct {
	volatile union {
		volatile struct {
			u16 bytes;
			u16 reserved;
		} tx;
		volatile struct {
			u16 bufferBytes;
			u16 bytesReceived;
		} rx;
	} count;

	volatile u32 command_status;
	volatile u32 next;	/* Physical address, only read by the DMA engine. */
	volatile void *data;	/* Physical address, only read by the DMA engine. */

	/*
	 * Force sizeof(gt64260enetDMAdescriptor) == sizeof(cacheline) 
	 * Not yet sure exactly why this is necessary but the GT64260-B
	 * part apparently has (yet another) bug that shows itself without
	 * this padding.  The symptom is that all Enet comms simply stop.
	 */
	u32 cachelineSizePadding[4];
} gt_dma_desc;

typedef struct gt_eth_priv_struct {
	gt_dma_desc *TXqueue[NUM_TX_QUEUES];
	gt_dma_desc *RXqueue[NUM_RX_QUEUES];
	struct sk_buff *TXskbuff[NUM_TX_QUEUES][Q_INDEX_LIMIT];
	struct sk_buff *RXskbuff[NUM_RX_QUEUES][Q_INDEX_LIMIT];
	u32 TXindex[NUM_TX_QUEUES];
	u32 RXindex[NUM_RX_QUEUES];
	u32 TXskbIndex[NUM_TX_QUEUES];
	u32 irq;
	u8 port;
	struct net_device_stats stat;
	spinlock_t lock;
	struct mii_if_info mii_if;

} gt_eth_priv;

/*
 * ----------------------------------------------------------------------------
 * addressTable.h - this file has all the declarations of the address table
 */

#define _8K_TABLE                           0
#define ADDRESS_TABLE_ALIGNMENT             8
#define HASH_DEFAULT_MODE                   14
#define HASH_MODE                           13
#define HASH_SIZE                           12
#define HOP_NUMBER                          12
#define MAC_ADDRESS_STRING_SIZE             12
#define MAC_ENTRY_SIZE                      sizeof(addrTblEntry)
#define MAX_NUMBER_OF_ADDRESSES_TO_STORE    1000
#define PROMISCUOUS_MODE                    0
#define SKIP                                1<<1
#define SKIP_BIT                            1
#define VALID                               1

/*
 * ----------------------------------------------------------------------------
 * XXX_MIKE - potential sign-extension bugs lurk here...
 */
#define NIBBLE_SWAPPING_32_BIT(X) ( (((X) & 0xf0f0f0f0) >> 4) \
                                  | (((X) & 0x0f0f0f0f) << 4) )

#define NIBBLE_SWAPPING_16_BIT(X) ( (((X) & 0x0000f0f0) >> 4) \
                                  | (((X) & 0x00000f0f) << 4) )

#define FLIP_4_BITS(X)  ( (((X) & 0x01) << 3) | (((X) & 0x002) << 1) \
                        | (((X) & 0x04) >> 1) | (((X) & 0x008) >> 3) )

#define FLIP_6_BITS(X)  ( (((X) & 0x01) << 5) | (((X) & 0x020) >> 5) \
                        | (((X) & 0x02) << 3) | (((X) & 0x010) >> 3) \
                        | (((X) & 0x04) << 1) | (((X) & 0x008) >> 1) )

#define FLIP_9_BITS(X)  ( (((X) & 0x01) << 8) | (((X) & 0x100) >> 8) \
                        | (((X) & 0x02) << 6) | (((X) & 0x080) >> 6) \
                        | (((X) & 0x04) << 4) | (((X) & 0x040) >> 4) \
         | ((X) & 0x10) | (((X) & 0x08) << 2) | (((X) & 0x020) >> 2) )

/*
 * V: value we're operating on
 * O: offset of rightmost bit in field
 * W: width of field to shift
 * S: distance to shift left
 */
#define MASK( fieldWidth )                            ((1 << (fieldWidth)) - 1)
#define leftShiftedBitfield( V,O,W,S)        (((V) & (MASK(W) << (O)))  << (S))
#define rightShiftedBitfield(V,O,W,S)  (((u32)((V) & (MASK(W) << (O)))) >> (S))

/* cache flushing helpers */

#define flush_dcache_addr_size( A, N )     \
    flush_dcache_range( ((u32)A), ((u32)(A)+(N)) )

#define flush_dcache(x) flush_dcache_range( ((u32)x) , ((u32)x) + sizeof(*(x)))

#define invalidate_dcache(x) invalidate_dcache_range( ((u32)x) , ((u32)x) + sizeof(*(x)))

static u32 uncachedPages(u32 pages);

static u32 hashTableFunction(u32 macH, u32 macL, u32 HashSize, u32 hash_mode);

static void addressTableClear(u32 port);

static int
 initAddressTable(u32 port, u32 hashMode, u32 hashSize, u32 hashDefaultMode);

static int
 addAddressTableEntry(u32 port, u32 macH, u32 macL, u32 rd, u32 skip);

/* this file has all the ethernet low level definitions */


#define ETHERNET_PORTS_DIFFERENCE_OFFSETS       0x400

#define ETHERNET0_HH_PRIORITY                   11
#define ETHERNET0_LL_PRIORITY                   2
#define ETHERNET1_HH_PRIORITY                   12

/* this macros are used to enable access to SMI_REG */
#define SMI_OP_CODE_BIT_READ                    1
#define SMI_OP_CODE_BIT_WRITE                   0
#define SMI_BUSY                                1<<28
#define READ_VALID                              1<<27

#ifdef CONFIG_ZUMA_V2
#define PHY_LXT97x
#define PHY_ADD0                                0
#define PHY_ADD1  				1
#elif CONFIG_MOT_MVP || CONFIG_HXEB100
#define PHY_LXT97x
#define PHY_ADD0                                4
#define PHY_ADD1                                5
#else
#define PHY_ADD0                                4
#define PHY_ADD1                                5
#define PHY_ADD2                                6
#endif

/* this macros are used to enable access to ETHERNET_PCXR */
#define OVERRIDE_RX_PRIORITY                    1<<8
#define MIB_CLEAR_MODE                          1<<16

/* this macros are used to enable access to ETHERNET_SDCMR */
#define START_TX_HIGH       1<<23
#define START_TX_LOW        1<<24
#define ENABLE_RX_DMA       1<<7
#define ABORT_RECEIVE       1<<15
#define STOP_TX_HIGH        1<<16
#define STOP_TX_LOW         1<<17
#define ABORT_TRANSMIT      1<<31

/* this macros are used to enable access to ETHERNET_SDCR */
#define ETHERNET_SDMA_BURST_SIZE          3

typedef unsigned int ETHERNET_PCR;
typedef unsigned int ETHERNET_PCXR;
typedef unsigned int ETHERNET_PCMR;
typedef unsigned int ETHERNET_PSR;
typedef unsigned int ETHERNET_SDCMR;
typedef unsigned int ETHERNET_SDCR;

typedef unsigned int PHY_ADD_REG;
typedef unsigned int SMI_REG;

typedef struct mibCounters {
	unsigned int byteReceived;
	unsigned int byteSent;
	unsigned int framesReceived;
	unsigned int framesSent;
	unsigned int totalByteReceived;
	unsigned int totalFramesReceived;
	unsigned int broadcastFramesReceived;
	unsigned int multicastFramesReceived;
	unsigned int cRCError;
	unsigned int oversizeFrames;
	unsigned int fragments;
	unsigned int jabber;
	unsigned int collision;
	unsigned int lateCollision;
	unsigned int frames64;
	unsigned int frames65_127;
	unsigned int frames128_255;
	unsigned int frames256_511;
	unsigned int frames512_1023;
	unsigned int frames1024_MaxSize;
	unsigned int macRxError;
	unsigned int droppedFrames;
	unsigned int outMulticastFrames;
	unsigned int outBroadcastFrames;
	unsigned int undersizeFrames;
} STRUCT_MIB_COUNTERS;

static int etherReadMIIReg(unsigned int portNum, unsigned int miiReg,
			   unsigned int *value);

static int etherWriteMIIReg(unsigned int portNum, unsigned int miiReg,
			    unsigned int value);

#ifdef PHY_LXT97x
#undef THREE_ETHERNET_RMII_PORTS
#define TWO_ETHERNET_MII_PORTS
#else
#define THREE_ETHERNET_RMII_PORTS
#undef TWO_ETHERNET_MII_PORTS
#endif

#define ETHERNET_PORT2                      2
#define ETHERNET_PORT1                      1
#define ETHERNET_PORT0                      0

#define MAX_NUMBER_OF_MPSC_PORTS                3
#define MAX_NUMBER_OF_ETHERNET_PORTS            3

#ifdef THREE_ETHERNET_RMII_PORTS
/********/
/* RMII */
/********/

#define NUMBER_OF_ETHERNET_PORTS                3
#define NUMBER_OF_MPSC_PORTS                    2
#define MRR_REG_VALUE                           0x7ffe38

/* connect MPSC0 + 3 ports of RMII */
#define SERIAL_PORT_MULTIPLEX_REGISTER_VALUE    0x1102
/* GALILEO value */
// 0000 0000 0001 0001  20 - RMII
//                      16 - clear MIB counters
// 1000 1000 0000 0000  15:14 - 2048 (10)
//#define PORT_CONTROL_EXTEND_VALUE             0x00118000

/* PPCBoot value */
// 0000 0000 0000 0001  20 - RMII
// 0100 1101 0000 0000  15:14 - 1536 (01)
//                      11 - dont force link pass
//                      10 - disable FC AN
//                       8 - prio override
//#define PORT_CONTROL_EXTEND_VALUE             0x00104d00

/* Montavista value */
// 0000 0000 0011 0000  21 - DSCP
//                      20 - RMII
// 0100 1100 0010 0010  15:14 - 1536 (01)
//                      11 - dont force link pass
//                      10 - disable fc AN
//                      5:3 - 8pkt high, 1 low (100)
//                      1 - bpdu trap
#define PORT_CONTROL_EXTEND_VALUE               0x00304c20

#define ETHERNET_DOWNLOADING_PORT               ETHERNET_PORT2

#else				/* if !THREE_ETHERNET_RMII_PORTS */

#ifdef TWO_ETHERNET_MII_PORTS
/*******/
/* MII */
/*******/

#define NUMBER_OF_ETHERNET_PORTS                2
#define NUMBER_OF_MPSC_PORTS                    2
#define MRR_REG_VALUE                           0x7ffe38
/* connect MPSC0 + 2 ports of MII */
#define SERIAL_PORT_MULTIPLEX_REGISTER_VALUE    0x1101
/* GALILEO value */
// 0000 0000 0000 0001  16 - clear MIB counters
// 1000 1000 0000 0000  15:14 - 2048 (10)
//#define PORT_CONTROL_EXTEND_VALUE             0x00018000

/* PPCBoot (ZUMA) value */
// 0000 0000 0000 0000
// 0100 1101 0000 0000  15:14 - 1536 (01)
//                      11 - dont force link pass
//                      10 - disable FC AN
//                       8 - prio override
//#define PORT_CONTROL_EXTEND_VALUE             0x00004d00

/* Montavista (ZUMA) value */
// 0000 0000 0010 0000  21 - DSCP
// 0100 1100 0010 0010  15:14 - 1536 (01)
//                      11 - dont force link pass
//                      10 - disable fc AN
//                      5:3 - 8pkt high, 1 low (100)
//                      1 - bpdu trap
#ifndef CONFIG_MOT_MVP
#define PORT_CONTROL_EXTEND_VALUE               0x00204c20
#else
/* Hack, don't allow 10mbit for now */
// 0000 0000 0010 1100  21 - DSCP
// 0100 1100 0010 0010  18 - Speed (1-> 100mbit)
//                      19 - SpeedEn (1-> disable speed AN)
//                      15:14 - 1536 (01)
//                      11 - dont force link pass
//                      10 - disable fc AN
//                      5:3 - 8pkt high, 1 low (100)
//                      1 - bpdu trap
// 
#define PORT_CONTROL_EXTEND_VALUE		0x002c4c20
#endif

#define ETHERNET_DOWNLOADING_PORT           ETHERNET_PORT1

#endif				/* endif TWO_ETHERNET_MII_PORTS */
#endif				/* endif !THREE_ETHERNET_RMII_PORTS */

#define LL_QUEUE_PRIORITY                       1
#define L_QUEUE_PRIORITY                        2
#define H_QUEUE_PRIORITY                        3
#define HH_QUEUE_PRIORITY                       4

#define NUMBER_OF_MIB_COUNTERS		25

#define	TIME_OUT			10	/* 1/6 SEC */


#endif				/* #ifndef GT64260_ETH_H */
