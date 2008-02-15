/*
 * ibm_ocp_mal.c
 *
 *      Armin Kuster akuster@mvista.com
 *      Juen, 2002
 *
 * Copyright 2002 MontaVista Softare Inc.
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
 *
 * Version 1.0 (06/06/02) by Armin kuster
 *
 * Version 1.1 06/11/02 - armin
 * fixed mal reset values
 * moved ISR to enable_mal
 * fixed channel pointer to match mal channel
 * 
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/netdevice.h>

#include <asm/io.h>

#include "ibm_ocp_enet.h"
#include "ibm_ocp_mal.h"

extern dma_addr_t rx_phys_addr;
extern dma_addr_t tx_phys_addr;

void
config_mal(struct ocp_enet_private *fep)
{

	set_mal_dcrn(fep, DCRN_MALRXCARR, 0xFFFFFFFF);
	set_mal_dcrn(fep, DCRN_MALTXCARR, 0xFFFFFFFF);

	set_mal_dcrn(fep, DCRN_MALCR, MALCR_MMSR);	/* 384 */
	/* FIXME: Add delay */

	/* Set the MAL configuration register */
	set_mal_dcrn(fep, DCRN_MALCR,
		     MALCR_PLBB | MALCR_OPBBL | MALCR_LEA |
		     MALCR_PLBLT_DEFAULT);

}

void
disable_mal_chan(struct ocp_enet_private *fep)
{
	set_mal_dcrn(fep, DCRN_MALRXCARR, fep->rxchan);
	set_mal_dcrn(fep, DCRN_MALTXCARR, fep->txchan);

}

void
enable_mal_chan(struct ocp_enet_private *fep)
{
	set_mal_dcrn(fep, DCRN_MALRXCASR,
		     get_mal_dcrn(fep, DCRN_MALRXCASR) | fep->rxchan);
	set_mal_dcrn(fep, DCRN_MALTXCASR,
		     get_mal_dcrn(fep, DCRN_MALTXCASR) | fep->txchan);
	set_mal_dcrn(fep, DCRN_MALIER, MALIER_DE |
		     MALIER_NE | MALIER_TE | MALIER_OPBE | MALIER_PLBE);


}

void
set_mal_chan_addr(struct ocp_enet_private *fep)
{
#ifdef CONFIG_NP405H
	/* setup MAL tx and rx channel pointers */
	if (fep->emac_num == 3) {
		set_mal_dcrn(fep, DCRN_MALTXCTP6R,
			     tx_phys_addr + (fep->emac_num * PAGE_SIZE));
		set_mal_dcrn(fep, DCRN_MALRXCTP3R,
			     rx_phys_addr + (fep->emac_num * PAGE_SIZE));
		set_mal_dcrn(fep, DCRN_MALRCBS3, DESC_BUF_SIZE_REG);

	} else if (fep->emac_num == 2) {
		set_mal_dcrn(fep, DCRN_MALTXCTP4R,
			     tx_phys_addr + (fep->emac_num * PAGE_SIZE));
		set_mal_dcrn(fep, DCRN_MALRXCTP2R,
			     rx_phys_addr + (fep->emac_num * PAGE_SIZE));
		set_mal_dcrn(fep, DCRN_MALRCBS2, DESC_BUF_SIZE_REG);

	} else
#endif				/* CONFIG_440 */
	if (fep->emac_num == 1) {
		set_mal_dcrn(fep, DCRN_MALTXCTP2R,
			     tx_phys_addr + (fep->emac_num * PAGE_SIZE));
		set_mal_dcrn(fep, DCRN_MALRXCTP1R,
			     rx_phys_addr + (fep->emac_num * PAGE_SIZE));
		set_mal_dcrn(fep, DCRN_MALRCBS1, DESC_BUF_SIZE_REG);

	} else if (fep->emac_num == 0) {
		set_mal_dcrn(fep, DCRN_MALTXCTP0R, tx_phys_addr);
		set_mal_dcrn(fep, DCRN_MALRXCTP0R, rx_phys_addr);
		set_mal_dcrn(fep, DCRN_MALRCBS0, DESC_BUF_SIZE_REG);
	}
}
