/*
 *  linux/include/asm-arm/arch-mx2ads/dma.h
 *
 *  Copyright (C) 1995-2000 Russell King
 *  Copyright (C) 2003 MontaVista Software Inc <source@mvista.com>,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * MX2-specific DMA handling
 */

#ifndef __MX2_DMA_INCLUDE__
#define __MX2_DMA_INCLUDE__

#define MAX_DMA_ADDRESS		0xffffffff

#define MAX_DMA_CHANNELS 16

/* DMA IRQ status */
#define DMA_DONE		0x1000
#define DMA_BURST_TIMEOUT 	0x1
#define DMA_REQUEST_TIMEOUT 	0x2
#define DMA_TRANSFER_ERROR	0x4
#define DMA_BUFFER_OVERFLOW	0x8

typedef void (*dma_callback_t) (dmach_t channel, void *dev_id, int status);

#define MAX_DMA_2D_REG_SETS 2
typedef unsigned char mx2dma2dset_t;

extern int mx2_request_dma_irq(dmach_t channel, dma_callback_t callback,
			       void *dev_id);
extern void mx2_free_dma_irq(dmach_t channel, void *dev_id);

extern int mx2_request_2d_dma(mx2dma2dset_t set);
extern void mx2_free_2d_dma(mx2dma2dset_t set);

/* DMA Control Register Bits */
#define DMA_CTL_CEN 0x1
#define DMA_CTL_FRC 0x2
#define DMA_CTL_RPT 0x4
#define DMA_CTL_REN 0x8

#define DMA_CTL_MSEL 0x100
#define DMA_CTL_MDIR 0x200
#define DMA_CTL_ACRPT 0x4000

#define DMA_CTL_GET_SSIZ(x) (((x)>>4)&0x3)
#define DMA_CTL_GET_DSIZ(x) (((x)>>6)&0x3)
#define DMA_CTL_GET_SMOD(x)  (((x)>>10)&0x3)
#define DMA_CTL_GET_DMOD(x)  (((x)>>12)&0x3)

#define DMA_CTL_SET_SSIZ(x,value) (x) = ((x) & ~(0x3 << 4)) | ((value) & 0x3) << 4
#define DMA_CTL_SET_DSIZ(x,value) (x) = ((x) & ~(0x3 << 6)) | ((value) & 0x3) << 6
#define DMA_CTL_SET_SMOD(x,value) (x) = ((x) & ~(0x3 << 10)) | ((value) & 0x3) << 10
#define DMA_CTL_SET_DMOD(x,value) (x) = ((x) & ~(0x3 << 12)) | ((value) & 0x3) << 12

/* Request Timeout Register defines */
#define DMA_RTOR_EN (1<<15)
#define DMA_RTOR_CLK (1<<14)
#define DMA_RTOR_PSC (1<<13)
#define DMA_RTOR_MASK 0x1fff

/* Burst Timeout Register defines */
#define DMA_DBTOCR_EN (1<<15)
#define DMA_DBTOCR_MASK 0x7fff

/* configures dma burst timeout, common for all registers */
#define mx2_conf_dma_burst_timeout(en,x) DMA_DBTOCR = ((x) & DMA_DBTOCR_MASK) | ((en) ? DMA_DBTOCR_EN : 0)

/* DMA CHANNEL CONFIGURATION DEFINES: */

/*forces a dma burst*/
#define mx2_force_dma_burst(chan) DMA_CCR(chan) |= DMA_CTL_FRC

/*returns status of dma burst initiated by mx2_force_dma_burst(): 1 burst not started, 0 - burst started.*/
#define mx2_force_dma_burst_status(chan) ((DMA_CCR(chan) & DMA_CTL_FRC) ? 1 : 0)

/* configures 2d dma.
   set - 2d register set number to be used with the channel. must be requested with mx2_request_2d_dma()
 */
#define mx2_conf_2d_dma(chan,set,w,x,y) \
do { \
	DMA_CCR(chan) = \
	    (DMA_CCR(chan) & ~DMA_CTL_MSEL) | ((set) ? DMA_CTL_MSEL : 0); \
	DMA_WSR((set) & 1) = (w); \
	DMA_XSR((set) & 1) = (x); \
	DMA_YSR((set) & 1) = (y); \
} while (0)

/* the two functions below configure dma source and destination.*/

/* use one of these for size parameter: */
#define DMA_MEM_SIZE_8	0x1
#define DMA_MEM_SIZE_16	0x2
#define DMA_MEM_SIZE_32	0x0

/* use one of these for type parameter: */
#define DMA_TYPE_LINEAR	0x0
#define DMA_TYPE_2D		0x01
#define DMA_TYPE_FIFO	0x2
#define DMA_TYPE_EBE	0x3	/*end-of-burst enable fifo */

/* address must be physical */
#define mx2_conf_dma_source(chan,type,size,addr) \
do { \
	DMA_SAR(chan) = (addr); \
	DMA_CTL_SET_SMOD(DMA_CCR(chan), (type)); \
	DMA_CTL_SET_SSIZ(DMA_CCR(chan), (size)); \
} while (0)

#define mx2_conf_dma_destination(chan,type,size,addr) \
do { \
	DMA_DAR(chan) = (addr); \
	DMA_CTL_SET_DMOD(DMA_CCR(chan), (type)); \
	DMA_CTL_SET_DSIZ(DMA_CCR(chan), (size)); \
} while (0)

/*these functions are useful for double-buffering (or DMA chaining in MX2 ref manual terms)*/
#define mx2_set_dma_source(chan,addr,cnt) \
do { \
	DMA_SAR(chan) = (addr); \
	DMA_CNTR(chan) = (cnt); \
} while (0)

#define mx2_set_dma_destination(chan,addr,cnt) \
do { \
	DMA_DAR(chan) = (addr); \
	DMA_CNTR(chan) = (cnt); \
} while (0)

/* configures a channel to use memory increment or decrement during transfers.
   mdir: 0 - memory increment, 1 - memory decrement */

/* use one of these for mdir parameter: */
#define DMA_MEMDIR_INCR 0
#define DMA_MEMDIR_DECR 1

#define mx2_conf_dma_memdir(chan,mdir) DMA_CCR(chan) = (DMA_CCR(chan) & ~DMA_CTL_MDIR) | ((mdir) ? DMA_CTL_MDIR : 0)

/* configure dma request or disable it.
   en: 1 - enable dma request, 0 - disable
   src: request source
   toutEn: 1 - request timeout enable, 0 - disable
   toutClk: timeout clock selections, 0 - HCLK, 1 - 32.768 KHz
   toutX256: timeout clock prescaler selection, 0 - no prescaling, 1 - divide clock by 256
   tout: timeout, max - DMA_RTOR_MASK
 */
/* use one of these for en parameter: */
#define DMA_REQUEST_EN 1
#define DMA_REQUEST_DIS 0
/* use one of these for toutEn parameter: */
#define DMA_REQ_TIMEOUT_EN 1
#define DMA_REQ_TIMEOUT_DIS 0
/* use one of these for toutClk parameter: */
#define DMA_REQ_TOUT_CLK_IS_HCLK 0
#define DMA_REQ_TOUT_CLK_32_8_KHZ 1
/* use one of these for toutX256 parameter: */
#define DMA_REQ_TOUT_CLK_DIV_BY_1 0
#define DMA_REQ_TOUT_CLK_DIV_BY_256 1

#define mx2_conf_dma_request(chan,en,src,toutEn,toutClk,toutX256,tout) \
do { \
	if (en) { \
		DMA_CCR(chan) |= DMA_CTL_REN; \
		DMA_RSSR(chan) = (src); \
		DMA_RTOR(chan) = ((toutEn) ? DMA_RTOR_EN : 0) \
		    | ((toutClk) ? DMA_RTOR_CLK : 0) | ((toutX256) ? \
							DMA_RTOR_PSC : 0) \
		    | ((tout) & DMA_RTOR_MASK); \
	} \
	else DMA_CCR(chan) &= ~DMA_CTL_REN; \
} while (0)

/* dma request source list */

#define DMA_REQ_CSI_RX     31
#define DMA_REQ_CSI_STAT   30
#define DMA_REQ_BMI_RX     29
#define DMA_REQ_BMI_TX     28
#define DMA_REQ_UART1_TX   27
#define DMA_REQ_UART1_RX   26
#define DMA_REQ_UART2_TX   25
#define DMA_REQ_UART2_RX   24
#define DMA_REQ_UART3_TX   23
#define DMA_REQ_UART3_RX   22
#define DMA_REQ_UART4_TX   21
#define DMA_REQ_UART4_RX   20
#define DMA_REQ_CSPI1_TX   19
#define DMA_REQ_CSPI1_RX   18
#define DMA_REQ_CSPI2_TX   17
#define DMA_REQ_CSPI2_RX   16
#define DMA_REQ_SSI1_TX1   15
#define DMA_REQ_SSI1_RX1   14
#define DMA_REQ_SSI1_TX0   13
#define DMA_REQ_SSI1_RX0   12
#define DMA_REQ_SSI2_TX1   11
#define DMA_REQ_SSI2_RX1   10
#define DMA_REQ_SSI2_TX0   9
#define DMA_REQ_SSI2_RX0   8
#define DMA_REQ_SDHC1      7
#define DMA_REQ_SDHC2      6
#define DMA_FIRI_TX        5
#define DMA_FIRI_RX        4
#define DMA_EX             3

/* configure dma burst length.
 * if dma request is disabled with mx2_conf_dma_request(),
 *  also set inter-burst delay
 * len: dma burst length in bytes
 * delay: dma burst delay in system clock ticks
 */

#define mx2_conf_dma_burst(chan,len,delay) \
do { \
	DMA_BLR(chan) = (len); \
	if (!(DMA_CCR(chan) & DMA_CTL_REN)) \
		DMA_BUCR(chan) = (delay); \
} while (0)

/* enables repeat, also with or without autoclear repeat */
#define DMA_RPT_EN 1
#define DMA_RPT_DIS 0
#define DMA_AUTO_CLR_RPT_EN 1
#define DMA_AUTO_CLR_RPT_DIS 0
#define mx2_conf_dma_rpt(chan,rptEn,rptAutoClr) DMA_CCR(chan) = (DMA_CCR(chan) & ~(DMA_CTL_RPT | DMA_CTL_ACRPT)) \
| ((rptEn) ? DMA_CTL_RPT : 0) | ((rptAutoClr) ? DMA_CTL_ACRPT : 0)

#define mx2_dma_count(chan) DMA_CCNR(chan)
#define mx2_dma_disable(chan) DMA_CCR(chan) &= ~DMA_CTL_CEN
#define mx2_dma_enable(chan) \
do { \
	DMA_DISR = 1 << (chan); \
	DMA_DBTOSR = 1 << (chan); \
	DMA_DRTOSR = 1 << (chan); \
	DMA_DSESR = 1 << (chan); \
	DMA_DBOSR = 1 << (chan); \
	DMA_CCR(chan) |= DMA_CTL_CEN; \
} while (0)

#endif
