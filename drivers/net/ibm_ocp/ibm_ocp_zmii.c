/*
 * ibm_ocp_zmii.c
 *
 *      Armin Kuster akuster@mvista.com
 *      Sept, 2001
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
 * 	V .1: 03/30/02 - Armin
 * 	   Initial release - moved zmii func from *_enet.c
 * 	V .2: 04/15/02 - Todd Poynor
 * 	   Added zmii_port_speed and some arrays for speed
 * 	V .3 04/16/02 - Armin
 * 		using ocp_devs struct more and added EMAC_DEV macro
 * 		fixed zmii_mode and enables auto_zmii
 * 	V .4 04/18/02 - Armin
 * 		added mii support
 * 		changed curr_zmii divisor to #define 
 * 	v .5 04/19/02 - Armin
 * 	 	removed need for emac_num as parameter
 * 	 	created zmii_enable array and remove the other *enable one
 * 	V .6 05/06/02 - Armin
 * 	  converted to core_ocp[];
 * 	  removed *_ADDR[]
 * 	  moved EMACS_PER_ZMII to ocp.h
 *
 * 	V .7 05/24/02 - Armin
 * 	name change of ocp_get_dev
 *	name change *_driver to *_dev
 *
 *	V .8 06/13/02 - Armin
 *	removed unneeded enable_zmii+port in init
 */

#include <linux/config.h>
#include <linux/netdevice.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/ocp.h>
#include "ocp_zmii.h"
#include "ibm_ocp_enet.h"

static unsigned int zmii_enable[][4] = {
	{ZMII_SMII0, ZMII_RMII0, ZMII_MII0,
	 ~(ZMII_MDI1 | ZMII_MDI2 | ZMII_MDI3)},
	{ZMII_SMII1, ZMII_RMII1, ZMII_MII1,
	 ~(ZMII_MDI0 | ZMII_MDI2 | ZMII_MDI3)},
	{ZMII_SMII2, ZMII_RMII2, ZMII_MII2,
	 ~(ZMII_MDI0 | ZMII_MDI1 | ZMII_MDI3)},
	{ZMII_SMII3, ZMII_RMII3, ZMII_MII3, ~(ZMII_MDI0 | ZMII_MDI1 | ZMII_MDI2)}
};
static unsigned int mdi_enable[] =
    { ZMII_MDI0, ZMII_MDI1, ZMII_MDI2, ZMII_MDI3 };

static unsigned int zmii_speed = 0x0;
static unsigned int zmii_speed100[] = { ZMII_MII0_100MB, ZMII_MII1_100MB };

void
enable_zmii_port(struct net_device *dev)
{
	struct ocp_enet_private *fep = dev->priv;
	zmii_t *zmiip = fep->zmii_base;
	unsigned int mask;

	mask = in_be32(&zmiip->fer);

	mask &= zmii_enable[fep->emac_num][MDI];	/* turn all non enable MDI's off */
	mask |= zmii_enable[fep->emac_num][fep->zmii_mode]
	    | mdi_enable[fep->emac_num];
	out_be32(&zmiip->fer, mask);

#ifdef EMAC_DEBUG
	printk("EMAC# %d zmiip 0x%x  = 0x%x\n", fep->emac_num, zmiip,
	       zmiip->fer);
#endif
}

void
zmii_port_speed(int speed, struct net_device *dev)
{
	struct ocp_enet_private *fep = dev->priv;
	zmii_t *zmiip = fep->zmii_base;

	if (speed == 100)
		zmii_speed |= zmii_speed100[fep->emac_num];

	out_be32(&zmiip->ssr, zmii_speed);
	return;
}

int
init_zmii(int mode, struct net_device *dev)
{
	struct ocp_enet_private *fep = dev->priv;
	struct zmii_regs *zmiip;
	struct ocp_dev *zmii_dev;
	char *mode_name[] = { "SMII", "RMII", "MII" };
	int curr_zmii;

	curr_zmii = (fep->emac_num / EMACS_PER_ZMII);
	zmii_dev = ocp_get_dev(ZMII, curr_zmii);
	if (zmii_dev == NULL) {
		if (!(zmii_dev = ocp_alloc_dev(0)))
			return -ENOMEM;
		zmii_dev->type = ZMII;
		if ((curr_zmii = ocp_register(zmii_dev)) == -ENXIO) {
			ocp_free_dev(zmii_dev);
			return -ENXIO;
		}
	}

	zmiip = (struct zmii_regs *)
	    __ioremap(zmii_dev->paddr, sizeof (*zmiip), _PAGE_NO_CACHE);
	fep->zmii_base = zmiip;
	fep->zmii_mode = mode;
	if (mode == ZMII_AUTO) {
		if (zmiip->fer & (ZMII_MII0 | ZMII_MII1 | 
				  ZMII_MII2 | ZMII_MII3))
			fep->zmii_mode = MII;
		if (zmiip->fer & (ZMII_RMII0 | ZMII_RMII1 |
				  ZMII_RMII2 | ZMII_RMII3))
			fep->zmii_mode = RMII;
		if (zmiip->fer & (ZMII_SMII0 | ZMII_SMII1 |
				  ZMII_SMII2 | ZMII_SMII3))
			fep->zmii_mode = SMII;

		/* Failsafe: ZMII_AUTO is invalid index into the arrays,
		   so force SMII if all else fails. */

		if (fep->zmii_mode == ZMII_AUTO)
			fep->zmii_mode = SMII;
	}

	printk(KERN_NOTICE "Zmii bridge in %s mode\n",
			mode_name[fep->zmii_mode]);
	return (fep->zmii_mode);
}
