/*
 * linux/drivers/pcmcia/pxa/mainstone.c
 *
 * Created:	Aug 28, 2003
 * Copyright:	MontaVista Software Inc.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Originally based upon lubbock.c
 *
 * Mainstone PCMCIA specific routines.
 *
 */

#include <linux/kernel.h>
#include <linux/sched.h>

#include <pcmcia/ss.h>

#include <asm/delay.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/arch/pcmcia.h>

/* structure for storing the init parameter for mainstone_pcmcia_init() */
static struct pcmcia_init mainstone_init; 

static void
mainstone_handler(int irq, void *dev, struct pt_regs *regs)
{
	/* Workaround of STSCHG interrupt lock problem */

	/* Mask S0_STSCHG interrupt to avoid interrupt lock */
	if ((MST_PCMCIA0 & MST_PCMCIA_nSTSCHG_BVD1) == 0)
		disable_irq(MAINSTONE_S0_STSCHG_IRQ);

	/* Mask S1_STSCHG interrupt to avoid interrupt lock */
	if ((MST_PCMCIA1 & MST_PCMCIA_nSTSCHG_BVD1) == 0)
		disable_irq(MAINSTONE_S1_STSCHG_IRQ);

	(*mainstone_init.handler)(irq, dev, regs);
}

static int
mainstone_pcmcia_init(struct pcmcia_init *init) {

	int return_val=0;
	
	/* turn of all power from PCMCIA sockets, RESET is off */
  	MST_PCMCIA0 = 0;
	MST_PCMCIA1 = 0;

	/* Workaround of STSCHG interrupt lock problem */

	mainstone_init.handler = init->handler;

	return_val += request_irq(MAINSTONE_S0_CD_IRQ, &mainstone_handler,
				  SA_INTERRUPT,	"Mainstone PCMCIA (0) CD",
				  NULL);
	return_val += request_irq(MAINSTONE_S1_CD_IRQ, &mainstone_handler,
				  SA_INTERRUPT, "Mainstone PCMCIA (1) CD", NULL);
	return_val += request_irq(MAINSTONE_S0_STSCHG_IRQ, &mainstone_handler,
				  SA_INTERRUPT, "Mainstone PCMCIA (0) STSCHG",
				  NULL);
	return_val += request_irq(MAINSTONE_S1_STSCHG_IRQ, &mainstone_handler,
				  SA_INTERRUPT, "Mainstone PCMCIA (1) STSCHG", NULL);

	return (return_val < 0) ? -1 : 2;
}

static int
mainstone_pcmcia_shutdown(void) {

	free_irq(MAINSTONE_S0_CD_IRQ, NULL);
	free_irq(MAINSTONE_S1_CD_IRQ, NULL);
	free_irq(MAINSTONE_S0_STSCHG_IRQ, NULL);
	free_irq(MAINSTONE_S1_STSCHG_IRQ, NULL);

	return 0;
}

static int
mainstone_pcmcia_socket_state(struct pcmcia_state_array *state_array) {

	unsigned long status0, status1;
	int return_val = 1;

	if (state_array->size < 2)
  		return -1;

	memset(state_array->state, 0, 
	       (state_array->size)*sizeof(struct pcmcia_state));

	/* socket 0 status */

	status0 = MST_PCMCIA0;

	state_array->state[0].detect =
		((status0 & MST_PCMCIA_nCD) == 0) ? 1 : 0;

	state_array->state[0].ready =
		((status0 & MST_PCMCIA_nIRQ) == 0) ? 0 : 1;

	state_array->state[0].bvd1 =
		((status0 & MST_PCMCIA_nSTSCHG_BVD1) == 0) ? 0 : 1;

	state_array->state[0].bvd2 =
		((status0 & MST_PCMCIA_nSPKR_BVD2) == 0) ? 0 : 1;

	/* there is no wrprot flag in status register */
	state_array->state[0].wrprot = 0;

	state_array->state[0].vs_3v =
		((status0 & MST_PCMCIA_nVS1) == 0) ? 1 : 0;

	state_array->state[0].vs_Xv =
		((status0 & MST_PCMCIA_nVS2) == 0) ? 1 : 0;

	/* socket 1 status */

	status1 = MST_PCMCIA1;

	state_array->state[1].detect =
		((status1 & MST_PCMCIA_nCD) == 0) ? 1 : 0;

	state_array->state[1].ready =
		((status1 & MST_PCMCIA_nIRQ) == 0) ? 0 : 1;

	state_array->state[1].bvd1 =
		((status1 & MST_PCMCIA_nSTSCHG_BVD1) == 0) ? 0 : 1;

	state_array->state[1].bvd2 =
		((status1 & MST_PCMCIA_nSPKR_BVD2) == 0) ? 0 : 1;

	/* there is no wrprot flag in status register */
	state_array->state[1].wrprot = 0;

	state_array->state[1].vs_3v =
		((status1 & MST_PCMCIA_nVS1) == 0) ? 1 : 0;

	state_array->state[1].vs_Xv =
		((status1 & MST_PCMCIA_nVS2) == 0) ? 1 : 0;

	/* Unmask S0_STSCHG interrupt */
	if ((MST_PCMCIA0 & MST_PCMCIA_nSTSCHG_BVD1) != 0)
		enable_irq(MAINSTONE_S0_STSCHG_IRQ);

	/* Unmask S1_STSCHG interrupt */
	if ((MST_PCMCIA1 & MST_PCMCIA_nSTSCHG_BVD1) != 0)
		enable_irq(MAINSTONE_S1_STSCHG_IRQ);

	return return_val;
}

static int
mainstone_pcmcia_get_irq_info(struct pcmcia_irq_info *info) {

	switch(info->sock) {
		case 0:
			info->irq = MAINSTONE_S0_IRQ;
			break;

		case 1:
			info->irq = MAINSTONE_S1_IRQ;
			break;

		default:
			return -1;
	}

	return 0;
}

static int
mainstone_pcmcia_configure_socket(unsigned int sock, socket_state_t *state)
{
	unsigned long flags, power, status;

	int ret=1;


	local_irq_save(flags);

again:
	power = 0;
	switch (state->Vcc) {
		case 0:
			power |= MST_PCMCIA_PWR_VCC_0;
			break;

		case 33:
			power |= MST_PCMCIA_PWR_VCC_33;
			break;

		case 50:
			power |= MST_PCMCIA_PWR_VCC_50;
			break;

		default:
			printk(KERN_ERR "%s(): unrecognized Vcc %u\n",
			       __FUNCTION__, state->Vcc);
			ret = -1;
			break;
	}

	switch (state->Vpp) {
		case 0:
			power |= MST_PCMCIA_PWR_VPP_0;
			break;

		case 120:
			power |= MST_PCMCIA_PWR_VPP_120;
			break;

		default:
			if(state->Vpp == state->Vcc)
				power |= MST_PCMCIA_PWR_VPP_VCC;
			else {
				printk(KERN_ERR
				       "%s(): unrecognized Vpp %u\n",
				       __FUNCTION__, state->Vpp);
				ret = -1;
				break;
			}
	}

	switch (sock) {
	case 0:
		MST_PCMCIA0 = power |
			((state->flags & SS_RESET) ? MST_PCMCIA_RESET : 0);
		break;

	case 1:
		MST_PCMCIA1 = power |
			((state->flags & SS_RESET) ? MST_PCMCIA_RESET : 0);
		break;
	default:
		ret = -1;
	}
		
	if (ret > 0) {
		ret = 0;
		/* 
		 * HACK ALERT: 
		 * We can't sense the voltage properly on Mainstone before
		 * actually applying some power to the socket (catch 22).
		 * Resense the socket Voltage Sense pins after applying socket power.
		 */

		if (sock == 0)
			status = MST_PCMCIA0 & (MST_PCMCIA_nVS1 | MST_PCMCIA_nVS2);
		else
			status = MST_PCMCIA1 & (MST_PCMCIA_nVS1 | MST_PCMCIA_nVS2);

		if (((status & (MST_PCMCIA_nVS1 | MST_PCMCIA_nVS2)) == 0) &&
		    (state->Vcc == 33)) {
			/* Switch to 5V,  Configure socket 0  with 5V voltage */
			if (sock == 0)
				MST_PCMCIA0 &= ~MST_PCMCIA_PWR_MASK;
			else
				MST_PCMCIA1 &= ~MST_PCMCIA_PWR_MASK;

		}
		state->Vcc = 50;
		state->Vpp = 50;
		goto again;
	}

	local_irq_restore(flags);

	return ret;
}

static int
mainstone_pcmcia_mask_irqs(unsigned int sock, unsigned int mask)
{
	if (mask) {
		/* disable irqs */
		disable_irq(MAINSTONE_S0_CD_IRQ);
		disable_irq(MAINSTONE_S1_CD_IRQ);
		disable_irq(MAINSTONE_S0_STSCHG_IRQ);
		disable_irq(MAINSTONE_S1_STSCHG_IRQ);
	}
	else {
		/* enable irqs */
		enable_irq(MAINSTONE_S0_CD_IRQ);
		enable_irq(MAINSTONE_S1_CD_IRQ);
		enable_irq(MAINSTONE_S0_STSCHG_IRQ);
		enable_irq(MAINSTONE_S1_STSCHG_IRQ);
	}
	return 0;
}

struct pcmcia_low_level mainstone_pcmcia_ops = { 
	mainstone_pcmcia_init,
	mainstone_pcmcia_shutdown,
	mainstone_pcmcia_socket_state,
	mainstone_pcmcia_get_irq_info,
	mainstone_pcmcia_configure_socket,
	mainstone_pcmcia_mask_irqs
};
