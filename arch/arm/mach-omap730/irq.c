/*
 *  linux/arch/arm/mach-omap730/irq.c
 *
 * BRIEF MODULE DESCRIPTION
 *   OMAP irq interface
 * 
 * Copyright (C) 2004 MontaVista Software, Inc.
 *  <source@mvista.com>
 *
 * Copyright (C) 2001 RidgeRun, Inc.
 * Author: RidgeRun, Inc.
 *         Greg Lonnon (glonnon@ridgerun.com) or info@ridgerun.com
 * 
 * Modifications for the OMAP730 :
 * Copyright (c) 2004 MPC-Data Limited (http://www.mpc-data.co.uk)
 * Dave Peverley <dpeverley@mpc-data.co.uk>
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
#include <linux/config.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/arch/irq.h>
#include <asm/io.h>

#undef  DEBUG
#ifdef  DEBUG
#define DPRINTK(fmt, args...)                           \
        printk("%s:%d: " fmt,                           \
               __FUNCTION__ , __LINE__ , ## args);
#else
#define DPRINTK(fmt, args...)
#endif

static void mask_irq(unsigned int irq);
static void unmask_irq(unsigned int irq);
static void ack_irq(unsigned int irq);

static u32 omap_ih2_base[] = {
  OMAP_IH2_0_BASE, 
  OMAP_IH2_1_BASE
};

/* Prototypes for the irqaction structs */
static void omap730_GPIO_irq_demux(int irq, void *dev_id, struct pt_regs *regs);
static void omap730_mpuio_irq_demux(int irq, void *dev_id, struct pt_regs *regs);

/*
 * irqaction structs - these *could* all be the same for thge GPIOs, but 
 * I wanted to use individual ones to give a little more detail to people 
 * looking at /proc/interrupts unaware of the irq architecture.
 */

static struct irqaction omap730_gpio_irq_0 = {
	.name		= "OMAP730 GPIO IRQ demux (GPIO 0 - 31)",
	.handler	= omap730_GPIO_irq_demux,
	.flags		= SA_INTERRUPT
};

static struct irqaction omap730_gpio_irq_1 = {
	.name		= "OMAP730 GPIO IRQ demux (GPIO 32 - 63)",
	.handler	= omap730_GPIO_irq_demux,
	.flags		= SA_INTERRUPT
};

static struct irqaction omap730_gpio_irq_2 = {
	.name		= "OMAP730 GPIO IRQ demux (GPIO 64 - 95)",
	.handler	= omap730_GPIO_irq_demux,
	.flags		= SA_INTERRUPT
};

static struct irqaction omap730_gpio_irq_3 = {
	.name		= "OMAP730 GPIO IRQ demux (GPIO 96 - 127)",
	.handler	= omap730_GPIO_irq_demux,
	.flags		= SA_INTERRUPT
};

static struct irqaction omap730_gpio_irq_4 = {
	.name		= "OMAP730 GPIO IRQ demux (GPIO 128 - 159)",
	.handler	= omap730_GPIO_irq_demux,
	.flags		= SA_INTERRUPT
};

static struct irqaction omap730_gpio_irq_5 = {
	.name		= "OMAP730 GPIO IRQ demux (GPIO 160 - 191)",
	.handler	= omap730_GPIO_irq_demux,
	.flags		= SA_INTERRUPT
};

static struct irqaction omap730_mpuio_irq = {
	.name		= "OMAP730 MPUIO IRQ demux",
	.handler	= omap730_mpuio_irq_demux,
	.flags		= SA_INTERRUPT
};

/*
 * GPIO irq routines
 *      
 * The GPIO multiplexes the interrupts from all of its 192 pins into 
 * six irq lines. 
 *
 * We treat GPIO like another level of interrupt handlers.
 */

/* find_gpio_bank_from_irq_number()
 *
 * Simple utility - given a GPIO IRQ number (IH_GPIO_BASE -> INT_GPIO_LAST), 
 * we find out which bank of GPIOs it's associated with.
 */ 
static inline int
find_gpio_bank_from_irq_number(int irq_num)
{
	int bank = 0;
  

	irq_num -= INT_GPIO_0;

	if (irq_num < (OMAP730_GPIO_NB * 1)) {
		bank = 0;
	} else if (irq_num < (OMAP730_GPIO_NB * 2)) {
		bank = 1;
	} else if (irq_num < (OMAP730_GPIO_NB * 3)) {
		bank = 2;
	} else if (irq_num < (OMAP730_GPIO_NB * 4)) {
		bank = 3;
	} else if (irq_num < (OMAP730_GPIO_NB * 5)) {
		bank = 4;
	} else if (irq_num < (OMAP730_GPIO_NB * 6)) {
		bank = 5;
	} else {
		DPRINTK("Asked to find bank from Illegal irq number %d \n", 
			irq_num);
		BUG();
	}

	return bank;
}

/* find_gpio_bank_from_irq_line()
 *
 * Simple utility - given an IRQ number (IH1/2 line number), we find out 
 * which bank of GPIOs it's associated with.
 */ 
static inline int
find_gpio_bank_from_irq_line(int irq_line)
{
	int bank = 0;
	
	switch(irq_line) {
		
	case INT_GPIO1:
		bank = 0;
		break;
	case INT_GPIO2:
		bank = 1;
		break;
	case INT_GPIO3:
		bank = 2;
		break;
	case INT_GPIO4:
		bank = 3;
		break;
	case INT_GPIO5:
		bank = 4;
		break;
	case INT_GPIO6:
		bank = 5;
		break;
	default:
		DPRINTK("Asked to find bank from Illegal irq line %d \n", 
		       irq_line);
		BUG();  // Oops....
	}

	return bank;
}

/* gpio_bit_in_bank()
 *
 * Takes an IRQ line number, and works out the bit position within its
 * associated bank.
 */ 
static inline u32
gpio_bit_in_bank(int irq_line)
{
	return (irq_line - INT_GPIO_0 - (find_gpio_bank_from_irq_number(irq_line) * OMAP730_GPIO_NB));
}

/* This routine returns a bit mask where a 1 indicates the  
 * corresponding unmasked GPIO interrupt is requesting service. 
 * A GPIO interrupt is deemed to be requesting service if it 
 * is unmasked and either of the following is true:
 * 
 * 	The corresponding bit is set in the GPIO interrupt 
 * 	status register, indicating that the GPIO interrupt 
 * 	controller has latched an edge on the interrupt input.
 *
 * or
 * 
 * 	The level of the GPIO input indicates that a level 
 * 	interrupt request is asserted.  GPIO inputs 
 * 	configured to latch interrupts on falling edges are 
 * 	considered to be asserting a level interrupt request 
 * 	when the input is low.  GPIO inputs configured to latch 
 * 	interrupts on rising edges are considered to be asserting 
 * 	a level interrupt request when the input is high.
 */
static inline u32
get_gpio_unmasked_irqs(int bank)
{
	u32 gpio_unmasked_ints = ~*((volatile __u32 *)GPIO_INT_MASK_REGn(bank));
	u32 gpio_inputs        = *((volatile __u32 *)GPIO_DATA_INPUT_REGn(bank));
	u32 gpio_control       = *((volatile __u32 *)GPIO_INT_CONTROL_REGn(bank));
	u32 gpio_status        = *((volatile __u32 *)GPIO_INT_STATUS_REGn(bank));
	u32 pending;


	DPRINTK("bank = %d\n", bank);

	pending = /* gpio_unmasked_ints & */ gpio_status;   // Masked in hardware
	pending |= gpio_unmasked_ints &	
		((gpio_control & gpio_inputs) | ~(gpio_control | gpio_inputs));

	DPRINTK("bank = %d unmasked = 0x%08x, inputs = 0x%08x, gpio_status = 0x%08x, pending = 0x%08x\n", 
		bank, (int)gpio_unmasked_ints, (int)gpio_inputs, 
		(int)gpio_status, (int)pending);

	return pending;
}

static void
gpio_mask_irq(unsigned int irq)
{
	u32 mask;
	int bank;


	DPRINTK("irq = %d\n", irq);

	bank = find_gpio_bank_from_irq_number(irq);

	mask = 1 << gpio_bit_in_bank(irq);

	/* Set the bit to disable the interrupt */
	*((volatile __u32 *)GPIO_INT_MASK_REGn(bank)) |= mask;
}

static void
gpio_ack_irq(unsigned int irq)
{
	u32 mask;
	int bank;


	DPRINTK("irq = %d\n", irq);

	bank = find_gpio_bank_from_irq_number(irq);

	mask =  1 << gpio_bit_in_bank(irq);
	/* Set the bit to clear the status  */
	*((volatile __u32 *)GPIO_INT_STATUS_REGn(bank)) = mask;
}

/* 
 * TODO : This function needs fixing to reenable code thats there.
 */

static void
gpio_unmask_irq(unsigned int irq)
{
	u32 mask;
	int bank;


	DPRINTK("irq = %d\n", irq);

	bank = find_gpio_bank_from_irq_number(irq);

	mask = 1 << gpio_bit_in_bank(irq);

	/* unmask the specific GPIO interrupt */
	*((volatile __u32 *)GPIO_INT_MASK_REGn(bank)) &= ~mask;

#if 0
	/* Now check to see if the GPIO interrupt we just unmasked is requesting 
	 * service.  In this case we will write to the Interrupt Set Register 
	 * to force an OMAP level 1 IRQ14 (GPIO cascade) interrupt.  This is 
	 * harmless if an IRQ14 interrupt is already pending, but if not it 
	 * will keep us from missing a GPIO interrupt request.
	 * Note that we can only force an interrupt if the interrupt is 
	 * configured as edge sensitive, so the level 1 IRQ14 interrupt MUST be 
	 * configured as edge sensitive.
	 * Additionally, it turns out that the ISR register can't force an 
	 * interrupt while a level interrupt request is asserted to the 
	 * corresponding IRQ.  This means that we must clear any latched GPIO 
	 * interrupt requests in the GPIO interrupt controller before forcing 
	 * the GPIO cascade interrupt.  Any latched GPIO interrupts that are 
	 * cleared will still be detected when the GPIO demux ISR executes as 
	 * long as the GPIO interrupt request input is still asserted.
	 */
	if (get_gpio_unmasked_irqs(bank) & mask) {
		u16 gpio_int_mask = *((volatile __u32 *)GPIO_INT_MASK_REGn(bank));

		*((volatile __u32 *)GPIO_INT_MASK_REGn(bank)) = 0xFFFFFFFF;
		*((volatile __u32 *)GPIO_INT_STATUS_REGn(bank)) = 0xFFFFFFFF;
		*((volatile __u32 *)(OMAP_IH1_BASE + IRQ_ISR)) = 1 << INT_GPIO1;

		DPRINTK("unmask1\n");

		*((volatile __u32 *)GPIO_INT_MASK_REGn(bank)) = gpio_int_mask;
		DPRINTK("unmask2\n");
	}
#endif
}

static void
gpio_mask_ack_irq(unsigned int irq)
{
	gpio_mask_irq(irq);
	gpio_ack_irq(irq);
}

/* omap730_GPIO_irq_demux()
 * 
 * Its worth noting that this gets called by any of the six IH lines for
 * the MPU GPIO lines. This means we must remember we're looking at a
 * window of 32 gpios within the full set of 192...
 * 
 * TODO : This function needs fixing to reenable code thats there.
 */
static void
omap730_GPIO_irq_demux(int irq, void *dev_id, struct pt_regs *regs)
{
	u32 pending;
	int gpio_irq;
	int current_bank; 
	int i;


	/* Find out which bank of GPIOs we're looking at. */
	current_bank = find_gpio_bank_from_irq_line(irq);

	/* Since the level 1 GPIO interrupt cascades are configured as 
	 * edge-sensitive, we need to unmask whichever here in order to avoid 
	 * missing any additional GPIO interrupts that might occur after the 
	 * last time we check for pending GPIO interrupts here.
	 *
	 * We are relying on the fact that this interrupt handler was installed 
	 * with the SA_INTERRUPT flag so that interrupts are disabled at the 
	 * CPU while it is executing.
	 */
#if 0
	unmask_irq(irq);

	for (;;) {
#endif
		pending = get_gpio_unmasked_irqs(current_bank);
#if 0
		if (!pending)
			break;
#endif

		for (i = 0; (i < OMAP730_GPIO_NB) && pending; i++, pending >>= 1) {
			if (pending & 1)
			{
				gpio_irq = INT_GPIO_0 + (OMAP730_GPIO_NB * current_bank) + i;

				DPRINTK("do_irq: %d\n", gpio_irq);

				do_IRQ(gpio_irq, regs);
			}
		}
#if 0
	}
#endif
}

static void
gpio_init_irq(void)
{
	int i;
	unsigned int bank;


	/* Set up default GPIO register states. */
	for(bank = 0; bank < OMAP730_GPIO_BANKS; bank++) {
	  /* Mask off all GPIO interrupts. */
	  *((volatile __u32 *)GPIO_INT_MASK_REGn(bank)) = 0xFFFFFFFF;

	  /* clear any pending GPIO interrupts */
	  *((volatile __u32 *)GPIO_INT_STATUS_REGn(bank)) = 0xFFFFFFFF;

	  /* configure all GPIO interrupts to trigger on falling edge */
	  *((volatile __u32 *)GPIO_INT_CONTROL_REGn(bank)) = 0x00000000;
	}

	/* Turn in clock */
	*((volatile __u16 *)(CAM_CLK_CTRL)) |= SYSTEM_CLK_EN;

	/* 
	 * Set up the GPIO entries in the irq descriptor array 
	 */

	for (i = INT_GPIO_0; i <= INT_GPIO_LAST; i++) {
		irq_desc[i].valid    = 1;
		irq_desc[i].probe_ok = 0;
		irq_desc[i].mask_ack = gpio_mask_ack_irq;
		irq_desc[i].mask     = gpio_mask_irq;
		irq_desc[i].unmask   = gpio_unmask_irq;
	}

	setup_arm_irq(INT_GPIO1, &omap730_gpio_irq_0);
	setup_arm_irq(INT_GPIO2, &omap730_gpio_irq_1);
	setup_arm_irq(INT_GPIO3, &omap730_gpio_irq_2);
	setup_arm_irq(INT_GPIO4, &omap730_gpio_irq_3);
	setup_arm_irq(INT_GPIO5, &omap730_gpio_irq_4);
	setup_arm_irq(INT_GPIO6, &omap730_gpio_irq_5);
}

/* This routine returns a bit mask where a 1 indicates the  
 * corresponding unmasked MPUIO interrupt is requesting service. 
 * A MPUIO interrupt is deemed to be requesting service if it 
 * is unmasked and either of the following is true:
 * 
 * 	The corresponding bit is set in the MPUIO interrupt 
 * 	status register, indicating that the MPUIO interrupt 
 * 	controller has latched an edge on the interrupt input.
 *
 * or
 * 
 * 	The level of the MPUIO input indicates that a level 
 * 	interrupt request is asserted.  MPUIO inputs 
 * 	configured to latch interrupts on falling edges are 
 * 	considered to be asserting a level interrupt request 
 * 	when the input is low.  MPUIO inputs configured to latch 
 * 	interrupts on rising edges are considered to be asserting 
 * 	a level interrupt request when the input is high.
 */

static inline u16
get_mpuio_unmasked_irqs(void)
{
	u16 gpio_unmasked_ints = ~*((volatile u16 *)MPUIO_GPIO_MASKIT);
	u16 gpio_inputs        = *((volatile u16 *)MPUIO_INPUT_LATCH);
	u16 gpio_control       = *((volatile u16 *)MPUIO_GPIO_INT_EDGE_REG);
	u16 gpio_status        = *((volatile u16 *)MPUIO_GPIO_INT);

	u16 pending = /* gpio_unmasked_ints &*/ gpio_status; // Masked in hardware


	pending |= gpio_unmasked_ints &
		((gpio_control & gpio_inputs) | ~(gpio_control | gpio_inputs));

	return pending;
}

static void
mpuio_mask_irq(unsigned int irq)
{
	u16 mask = 1 << (irq - INT_MPUIO_0);

	/* Set the bit to disable the interrupt */
	*((volatile u16 *)MPUIO_GPIO_MASKIT) |= mask;
}

static void
mpuio_unmask_irq(unsigned int irq)
{
	u16 mask = 1 << (irq - INT_MPUIO_0);


	/* unmask the specific GPIO interrupt */
	*((volatile u16 *)MPUIO_GPIO_MASKIT) &= ~mask;

#if 0
	/* TODO : Should this be implemented / fixed? 
	 *        MV must have commented it out for a reason>
	 */

	/* Now check to see if the MPUIO interrupt we just unmasked is requesting 
	 * service.  In this case we will write to the Interrupt Set Register 
	 * to force an OMAP level 1 IRQ14 (GPIO cascade) interrupt.  This is 
	 * harmless if an IRQ14 interrupt is already pending, but if not it 
	 * will keep us from missing a GPIO interrupt request.
	 * Note that we can only force an interrupt if the interrupt is 
	 * configured as edge sensitive, so the level 1 IRQ14 interrupt MUST be 
	 * configured as edge sensitive.
	 * Additionally, it turns out that the ISR register can't force an 
	 * interrupt while a level interrupt request is asserted to the 
	 * corresponding IRQ.  This means that we must clear any latched GPIO 
	 * interrupt requests in the GPIO interrupt controller before forcing 
	 * the GPIO cascade interrupt.  Any latched GPIO interrupts that are 
	 * cleared will still be detected when the GPIO demux ISR executes as 
	 * long as the GPIO interrupt request input is still asserted.
	 */
	if (get_gpio_unmasked_irqs() & mask) {
		u16 gpio_int_mask = inw(GPIO_INT_MASK1_REG);

		outw(0xffff, GPIO_INT_MASK1_REG);
		outw(0xffff, GPIO_INT_STATUS1_REG);

		outl((1 << INT_GPIO1), OMAP_IH1_BASE + IRQ_ISR);

		outw(gpio_int_mask, GPIO_INT_MASK1_REG);
	}
#endif
}

static void
omap730_mpuio_irq_demux(int irq, void *dev_id, struct pt_regs *regs)
{
	u16 pending;
	int mpuio_irq;

	/* Since the level 1 GPIO interrupt cascade (IRQ14) is configured as 
	 * edge-sensitive, we need to unmask it here in order to avoid missing 
	 * any additional GPIO interrupts that might occur after the last time 
	 * we check for pending GPIO interrupts here.
	 * We are relying on the fact that this interrupt handler was installed 
	 * with the SA_INTERRUPT flag so that interrupts are disabled at the 
	 * CPU while it is executing.
	 */

	pending = get_mpuio_unmasked_irqs();

	for (mpuio_irq = INT_MPUIO_0;
		(mpuio_irq <= INT_MPUIO_LAST) && pending;
		mpuio_irq++, pending >>= 1)
	{
		if (pending & 1)
			do_IRQ(mpuio_irq, regs);
	}
}

static void
mpuio_init_irq(void)
{
	int i;
	volatile u16 pending;


	/* Mask all MPUIO interrupts */
	*((volatile u16 *)MPUIO_GPIO_MASKIT) = 0xffff;

	/* Clear any pending MPUIO interrupts */
	pending = *((volatile u16 *)MPUIO_GPIO_INT);

	/* Configure all MPUIO interrupts to trigger on falling edge */
	*((volatile u16 *)MPUIO_GPIO_INT_EDGE_REG) = 0x0000;

	/* Note, the numbers go up to 12 from 0, but there are only 10 
	 * MPUIO irqs in the irq_desc table.
	 */
	for (i = INT_MPUIO_0; i <= INT_MPUIO_LAST; i++) {
		irq_desc[i].valid = 1;
		irq_desc[i].probe_ok = 0;
		irq_desc[i].mask_ack = mpuio_mask_irq;
		irq_desc[i].mask = mpuio_mask_irq;
		irq_desc[i].unmask = mpuio_unmask_irq;
	}

	setup_arm_irq(INT_MPUIO, &omap730_mpuio_irq);
}

/*
 *      Standard irq routines
 */

static inline void
write_ih(int level, int part, int reg, u32 value)
{
        /* If not a banked register, don't offset */
        if ((reg == IRQ_SIR_IRQ) || (reg == IRQ_SIR_FIQ) ||
            (reg == IRQ_CONTROL_REG) || (reg == IRQ_OCP_CFG)) {
                *((volatile u32 *)((level ? OMAP_IH2_0_BASE : OMAP_IH1_BASE) + (reg))) = value;
        } else {
                /* IRQ_ITRX, IRQ_MIRX, IRQ_ISRX, (IRQ_ILR0 + x) */
                *((volatile u32 *)((level ? omap_ih2_base[part]: OMAP_IH1_BASE) + (reg))) = value;
        }
}
                                                                                                                             
inline u32
read_ih(int level, int part, int reg)
{
        /* If not a banked register, don't offset */
        if ((reg == IRQ_SIR_IRQ) || (reg == IRQ_SIR_FIQ) ||
            (reg == IRQ_CONTROL_REG) || (reg == IRQ_OCP_CFG)) {
                return *((volatile u32 *)((level ? OMAP_IH2_0_BASE : OMAP_IH1_BASE) + (reg)));
        } else {
                /* IRQ_ITRX, IRQ_MIRX, IRQ_ISRX, (IRQ_ILR0 + x) */
                return *((volatile u32 *)((level ? omap_ih2_base[part] : OMAP_IH1_BASE) + (reg)));
        }
}

static inline int
get_level(int irq)
{
	return (((irq) < IH2_BASE) ? 0 : 1);
}

static inline int
get_part(int irq)
{
	return (((irq) - IH2_BASE) >> 5);
}

static inline int
get_irq_num(int irq)
{
	return (((irq) < IH2_BASE) ? irq : ((irq - IH2_BASE) & 0x1F));
}

static void
mask_irq(unsigned int irq)
{
	int level = get_level(irq);
	int part = get_part(irq);
	int irq_num = get_irq_num(irq);
	u32 mask = read_ih(level, part, IRQ_MIR) | (1 << irq_num);

	write_ih(level, part, IRQ_MIR, mask);	// mask the interrupt
}

static void
ack_irq(unsigned int irq)
{
	int level = get_level(irq);
	int part = get_part(irq);

	do {
		write_ih(level, part, IRQ_CONTROL_REG, 0x1);
		/* REVISIT: So says the TRM:
		 *	if (level) write_ih(0, ITR, 0);
		 */
	} while (level--);
}

static void
unmask_irq(unsigned int irq)
{
	int level = get_level(irq);
	int part = get_part(irq);
	int irq_num = get_irq_num(irq);
	u32 mask = read_ih(level, part, IRQ_MIR) & ~(1 << irq_num);
	write_ih(level, part, IRQ_MIR, mask);
}

static void
mask_ack_irq(unsigned int irq)
{
	mask_irq(irq);
	ack_irq(irq);
}

static void
irq_priority(int irq, int fiq, int priority, int trigger)
{
	unsigned long reg_value, reg_addr;

	int level = get_level(irq);
	int part = get_part(irq);
	int irq_num = get_irq_num(irq);


        // FIQ only available on level 0 interrupts
        fiq = fiq & 0x1;
	reg_value = (fiq) | ((priority & 0x1f) << 2) |
	    ((trigger & 0x1) << 1);
	reg_addr = (IRQ_ILR0 + irq_num * 0x4);
	write_ih(level, part, reg_addr, reg_value);
}

void
omap730_init_irq(void)
{
	int i;
	uint trigger;
	uint def_trigger[] = { 0xb3f8e22f, 0xfdb9c1f2, 0x800040f3 };

	write_ih(0, 0, IRQ_MIR, ~0x0);	// mask all interrupts
	write_ih(0, 0, IRQ_ITR, 0x0);	// clear all interrupts
	write_ih(1, 0, IRQ_MIR, ~0x0);	// mask all interrupts
	write_ih(1, 1, IRQ_MIR, ~0x0);	// mask all interrupts
	write_ih(1, 0, IRQ_ITR, 0x0);	// clear all interrupts
	write_ih(1, 1, IRQ_ITR, 0x0);	// clear all interrupts
	write_ih(1, 0, IRQ_CONTROL_REG, 3); /* clear any pending interrupt */
	write_ih(0, 0, IRQ_CONTROL_REG, 3); /* clear any pending interrupt */

	for (i = 0; i <= INT_IH2_LAST; i++) {
		irq_desc[i].valid = 1;
		irq_desc[i].probe_ok = 0;
		irq_desc[i].mask_ack = mask_ack_irq;
		irq_desc[i].mask = mask_irq;
		irq_desc[i].unmask = unmask_irq;

		trigger = (def_trigger[i>>5] >> get_irq_num(i)) & 0x1;
		irq_priority(i, 0, 0, trigger);
	}
	gpio_init_irq();
	mpuio_init_irq();
	unmask_irq(INT_IH2_IRQ);
//!!!!!	unmask_irq(INT_IH2_FIQ);
}

/* 
 * Set MPUIO and GPIO irq sensitivity to Front, Rear, High or Low levels.
 *
 * TODO : Front and rear are currently unimplemented, but whats actually
 *        being used for high/low are really rising/falling edge anyway?!
 */

int omap730_tune_irq(unsigned int irq, u32 flags)
{
	int bank;

	
	if((flags != IFL_LOWLEVEL) && (flags != IFL_HIGHLEVEL))
		return -2;

	if((irq >= INT_MPUIO_0) && (irq <= INT_MPUIO_LAST)) {
		u16 edge_reg;

		/* MPUIO Interrupt - Only first five are valid though?! */
		edge_reg = *((volatile u16 *)MPUIO_GPIO_INT_EDGE_REG);
		edge_reg &= ~(1 << (irq - INT_MPUIO_0));
		if (flags == IFL_HIGHLEVEL) 
			/* Actually, rising edge */
			edge_reg |= 1 << (irq - INT_MPUIO_0);
		else
			/* Actually, falling edge */
			edge_reg &= ~(1 << (irq - INT_MPUIO_0));       
		*((volatile u16 *)MPUIO_GPIO_INT_EDGE_REG) = edge_reg;
	} else if ((irq >= INT_GPIO_0) && (irq <= INT_GPIO_LAST)) {
		u32 edge_reg;

		/* GPIO Interrupt */
		bank = find_gpio_bank_from_irq_number(irq);

		edge_reg = *((volatile u32 *)GPIO_INT_CONTROL_REG + (bank * OMAP730_GPIO_SIZE));
		edge_reg &= ~(1 << (irq - INT_GPIO_0 - (bank * OMAP730_GPIO_NB)));

		if (flags == IFL_HIGHLEVEL) 
			/* Actually, rising edge */
			edge_reg |= 1 << (irq - INT_GPIO_0 - (bank * OMAP730_GPIO_NB));
		else
			/* Actually, falling edge */
			edge_reg &= ~(0 << (irq - INT_GPIO_0 - (bank * OMAP730_GPIO_NB)));       
		*((volatile u32 *)GPIO_INT_CONTROL_REG) = edge_reg;
	} else {
		/* Neither MPUIO or GPIO :-( */
		// BUG() maybe?  
	}
	return 0;
}

EXPORT_SYMBOL(omap730_tune_irq);
