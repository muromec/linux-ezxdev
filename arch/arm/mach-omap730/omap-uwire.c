/*
 * arch/arm/mach-omap730/omap-uwire.c
 *
 * uWire interface for OMAP730/1610
 *
 * Note : On the OMAP730 (or P2 at least...) the muxed pin on the CPU that
 * is SDI is also used for KBR5 which is used instead - effectively this 
 * means that the P2's uWire implementation is TX only....
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 */

#include <linux/module.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/hardware.h>

void omap_uwire_cs0_configure_mode (u8 edge_rd, u8 edge_wr, u8 lvl, u8 frq, u8 chk)
{
	edge_rd &= 1;
	edge_wr &= 1;
	lvl     &= 1;
	frq     &= CS0_FRQ_MSK;
	chk     &= 1;

	outw((inw(UWIRE_SR1) & 0xFFC0) | 
	     (chk << CS0_CHK_POS) | 
	     (frq << CS0_FRQ_POS) | 
	     (lvl << CS0CS_LVL_POS) | 
	     (edge_wr << CS0_EDGE_WR_POS) | 
	     edge_rd << CS0_EDGE_RD_POS, 
	     UWIRE_SR1);
}

void omap_uwire_cs1_configure_mode (u8 edge_rd, u8 edge_wr, u8 lvl, u8 frq, u8 chk)
{
	edge_rd &= 1;
	edge_wr &= 1;
	lvl     &= 1;
	frq     &= CS1_FRQ_MSK;
	chk     &= 1;

	outw((inw(UWIRE_SR1) & 0xF03F) | 
	     (chk << CS1_CHK_POS) | 
	     (frq << CS1_FRQ_POS) | 
	     (lvl << CS1CS_LVL_POS) | 
	     (edge_wr << CS1_EDGE_WR_POS) | 
	     edge_rd << CS1_EDGE_RD_POS, 
	     UWIRE_SR1);
}

void omap_uwire_cs2_configure_mode (u8 edge_rd, u8 edge_wr, u8 lvl, u8 frq, u8 chk)
{
	edge_rd &= 1;
	edge_wr &= 1;
	lvl     &= 1;
	frq     &= CS2_FRQ_MSK;
	chk     &= 1;

	outw((inw(UWIRE_SR2) & 0xFFC0) | 
	     (chk << CS2_CHK_POS) | 
	     (frq << CS2_FRQ_POS) | 
	     (lvl << CS2CS_LVL_POS) | 
	     (edge_wr << CS2_EDGE_WR_POS) | 
	     edge_rd << CS2_EDGE_RD_POS, 
	     UWIRE_SR2);
}

void omap_uwire_cs3_configure_mode (u8 edge_rd, u8 edge_wr, u8 lvl, u8 frq, u8 chk)
{

	edge_rd &= 1;
	edge_wr &= 1;
	lvl     &= 1;
	frq     &= CS3_FRQ_MSK;
	chk     &= 1;

	outw((inw(UWIRE_SR2) & 0xF03F) | 
	     (chk << CS3_CHK_POS) | 
	     (frq << CS3_FRQ_POS) | 
	     (lvl << CS3CS_LVL_POS) | 
	     (edge_wr << CS3_EDGE_WR_POS) | 
	     edge_rd << CS3_EDGE_RD_POS, 
	     UWIRE_SR2);
}

u16 omap_uwire_data_transfer(u8 cs, u16 data, u8 trans_size, u8 rec_size)
{
	u8 leavecs = cs & 0x80;

	/* Make sure parameters are sane. */
	trans_size &= UWIRE_NB_BITS_WR_MSK;
	rec_size   &= UWIRE_NB_BITS_RD_MSK;
	cs         &= UWIRE_CSR_INDEX_MSK;

	/* Enable clock, set internal clock rate to F/2 */
	outw((1 << SR3_CLK_EN_POS), UWIRE_SR3);

	/* 
	 * NB_BITS_RD:0 ; NB_BITS_WR:0 ; INDEX: cs ; CS_CMD:1 ; START:0 
	 */
	outw((cs << UWIRE_CSR_INDEX_POS) | (1 << UWIRE_CS_CMD_POS), UWIRE_CSR); 

	outw(data, UWIRE_TDR); /* Place data in TX register */

	udelay(10);

	while(inw(UWIRE_CSR) & (1 << UWIRE_CSRB_POS))
	{ 
		udelay(1);  /* Wait for the CSRB bit to be reset */ 
	};

	/* Start uWire read/write */

	outw((rec_size << UWIRE_NB_BITS_RD_POS) | 
	     (trans_size << UWIRE_NB_BITS_WR_POS) | 
	     (1 << UWIRE_CS_CMD_POS) | 
	     (cs << UWIRE_CSR_INDEX_POS) | 
	     (1 << UWIRE_START_POS), 
	     UWIRE_CSR); 

	udelay(10);

	while((rec_size?!(inw(UWIRE_CSR) & (1 << UWIRE_RDRB_POS)):0) || (inw(UWIRE_CSR) & (1 << UWIRE_CSRB_POS)))
	{
		udelay(1); /* Wait for both transfers to be completed */
	}

	if(!leavecs) {
		*((volatile u16 *)UWIRE_CSR) &= ~(UWIRE_CSR_INDEX_MSK << UWIRE_CSR_INDEX_POS); 
		*((volatile u16 *)UWIRE_CSR) |= cs << UWIRE_CSR_INDEX_POS; 
	}

	udelay(10);

	/* Disable clock, set internal clock rate to F/2 */
	if(!leavecs) 
		outw(0, UWIRE_SR3);

	return(inw(UWIRE_RDR));
}

EXPORT_SYMBOL(omap_uwire_data_transfer);
EXPORT_SYMBOL(omap_uwire_cs0_configure_mode);
EXPORT_SYMBOL(omap_uwire_cs1_configure_mode);
EXPORT_SYMBOL(omap_uwire_cs2_configure_mode);
EXPORT_SYMBOL(omap_uwire_cs3_configure_mode);
