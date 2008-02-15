/*
 * BRIEF MODULE DESCRIPTION
 *
 *	uWire interface driver for the OMAP1610 Platform
 *
 * Copyright 2003 MontaVista Software Inc.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/delay.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/io.h>

/* General OMAP1610 uWire functions */

void omap1610_uwire_cs0_configure_mode (u8 edge_rd, u8 edge_wr, u8 lvl, u8 frq, u8 chk)
{
  outw((inw(UWIRE_SR1) & 0xFFC0) | (chk << 5) | (frq << 3) | (lvl << 2) | (edge_wr << 1) | edge_rd, 
       UWIRE_SR1);

}

void omap1610_uwire_cs1_configure_mode (u8 edge_rd, u8 edge_wr, u8 lvl, u8 frq, u8 chk)
{

  outw((inw(UWIRE_SR1) & 0xF03F) | (chk << 11) | (frq << 9) | (lvl << 8) | (edge_wr << 7) | 
       (edge_rd << 6), UWIRE_SR1);

}

void omap1610_uwire_cs2_configure_mode (u8 edge_rd, u8 edge_wr, u8 lvl, u8 frq, u8 chk)
{

    outw((inw(UWIRE_SR2) & 0xFFC0) | (chk << 5) | (frq << 3) | (lvl << 2) | (edge_wr << 1) | edge_rd, 
       UWIRE_SR2);

}

void omap1610_uwire_cs3_configure_mode (u8 edge_rd, u8 edge_wr, u8 lvl, u8 frq, u8 chk)
{
  outw((inw(UWIRE_SR2) & 0xF03F) | (chk << 11) | (frq << 9) | (lvl << 8) | (edge_wr << 7) | 
       (edge_rd << 6), UWIRE_SR2);

}

/* uWire data trasfer function:  cs - chip select, data - data to be trasmited */
/*                               trans_size - number of bits to be transmited */
/*                               rec_size - number of bits to be recieved */

u16 omap1610_uwire_data_transfer(u8 cs, u16 data, u8 trans_size, u8 rec_size)
{
	u8 leavecs = cs&0x80;

	cs = cs&3;
	outw(1, UWIRE_SR3);

	outw((cs << 10) | (1 << 12), UWIRE_CSR); /* NB_BITS_RD:0 ; NB_BITS_WR:0 ; INDEX: cs ; */
	/*CS_CMD:1 ; START:0 */
	outw(data, UWIRE_TDR); /* Place data in TX register */

	udelay(10);

	while(inw(UWIRE_CSR) & (1 << 14))
		udelay(1);  /* Wait for the CSRB bit to be reset */ 
	
	/* Start uWire read/write */
	outw((rec_size << 0) | (trans_size << 5) | (1 << 12) | (cs << 10) | (1 << 13), UWIRE_CSR); 

	udelay(10);

	while((rec_size?!(inw(UWIRE_CSR) & (1 << 15)):0) || (inw(UWIRE_CSR) & (1 << 14)))
		udelay(1); /* Wait for both transfers to be completed */
 
	if(!leavecs)
		outw((cs << 10), UWIRE_CSR); 

	udelay(10);
	if(!leavecs)
		outw(0, UWIRE_SR3);
	return inw(UWIRE_RDR);
}

EXPORT_SYMBOL(omap1610_uwire_cs0_configure_mode);
EXPORT_SYMBOL(omap1610_uwire_cs1_configure_mode);
EXPORT_SYMBOL(omap1610_uwire_cs2_configure_mode);
EXPORT_SYMBOL(omap1610_uwire_cs3_configure_mode);
EXPORT_SYMBOL(omap1610_uwire_data_transfer);
