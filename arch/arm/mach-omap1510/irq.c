/*
 *  linux/arch/arm/mach-omap/irq.c
 *
 * BRIEF MODULE DESCRIPTION
 *   OMAP irq interface
 *
 * Copyright (C) 2001 RidgeRun, Inc.
 * Author: RidgeRun, Inc.
 *         Greg Lonnon (glonnon@ridgerun.com) or info@ridgerun.com
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

static void mask_irq(unsigned int irq);
static void unmask_irq(unsigned int irq);
static void ack_irq(unsigned int irq);


/*
 *      GPIO irq routines
 */

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
static inline u16
get_gpio_unmasked_irqs(void)
{
	u16 gpio_unmasked_ints = ~inw(GPIO_INT_MASK_REG);
	u16 gpio_inputs = inw(GPIO_DATA_INPUT_REG);
	u16 gpio_control = inw(GPIO_INT_CONTROL_REG);
	u16 gpio_status = inw(GPIO_INT_STATUS_REG);
	u16 pending;

	pending = gpio_unmasked_ints & gpio_status;

	pending |= gpio_unmasked_ints &
		((gpio_control & gpio_inputs) | ~(gpio_control | gpio_inputs));

	return pending;
}

static void
gpio_mask_irq(unsigned int irq)
{
	u16 mask = 1 << (irq - IH_GPIO_BASE);

	/* Set the bit to disable the interrupt */
	outw((inw(GPIO_INT_MASK_REG) | mask), GPIO_INT_MASK_REG);
}

static void
gpio_ack_irq(unsigned int irq)
{
	u16 mask = 1 << (irq - IH_GPIO_BASE);

	/* Set the bit to clear the status  */
	outw(mask, GPIO_INT_STATUS_REG);
}

static void
gpio_unmask_irq(unsigned int irq)
{
	u16 mask = 1 << (irq - IH_GPIO_BASE);

	/* unmask the specific GPIO interrupt */
	outw((inw(GPIO_INT_MASK_REG) & ~mask), GPIO_INT_MASK_REG);

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
	if (get_gpio_unmasked_irqs() & mask) {
		u16 gpio_int_mask = inw(GPIO_INT_MASK_REG);

		outw(0xffff, GPIO_INT_MASK_REG);
		outw(0xffff, GPIO_INT_STATUS_REG);

		outl((1 << INT_GPIO), OMAP_IH1_BASE + IRQ_ISR);

		outw(gpio_int_mask, GPIO_INT_MASK_REG);
	}
}

static void
gpio_mask_ack_irq(unsigned int irq)
{
	gpio_mask_irq(irq);
	gpio_ack_irq(irq);
}

static void
omap1510_GPIO_irq_demux(int irq, void *dev_id, struct pt_regs *regs)
{
	u16 pending;
	int gpio_irq;

	/* Since the level 1 GPIO interrupt cascade (IRQ14) is configured as 
	 * edge-sensitive, we need to unmask it here in order to avoid missing 
	 * any additional GPIO interrupts that might occur after the last time 
	 * we check for pending GPIO interrupts here.
	 * We are relying on the fact that this interrupt handler was installed 
	 * with the SA_INTERRUPT flag so that interrupts are disabled at the 
	 * CPU while it is executing.
	 */
	unmask_irq(irq);

	for (;;) {
		pending = get_gpio_unmasked_irqs();

		if (!pending)
			break;

		for (gpio_irq = IH_GPIO_BASE;
			(gpio_irq < (IH_GPIO_BASE + 16)) && pending;
			gpio_irq++, pending >>= 1) {
			if (pending & 1)
				do_IRQ(gpio_irq, regs);
		}
	}
}

static struct irqaction omap1510_gpio_irq = {
	.name		= "OMAP1510 GPIO IRQ demux",
	.handler	= omap1510_GPIO_irq_demux,
	.flags		= SA_INTERRUPT
};

static void
gpio_init_irq(void)
{
	int i;

	/* mask all GPIO interrupts */
	outw(0xffff, GPIO_INT_MASK_REG);
	/* clear any pending GPIO interrupts */
	outw(0xffff, GPIO_INT_STATUS_REG);
	/* configure all GPIO interrupts to trigger on falling edge */
	outw(0x0000, GPIO_INT_CONTROL_REG);

	for (i = IH_GPIO_BASE; i < (IH_GPIO_BASE + 16); i++) {
		irq_desc[i].valid = 1;
		irq_desc[i].probe_ok = 0;
		irq_desc[i].mask_ack = gpio_mask_ack_irq;
		irq_desc[i].mask = gpio_mask_irq;
		irq_desc[i].unmask = gpio_unmask_irq;
	}

	setup_arm_irq(INT_GPIO, &omap1510_gpio_irq);
}


/*
 *      Standard irq routines
 */

static inline void
write_ih(int level, int reg, u32 value)
{
	outl(value,
	     (IO_ADDRESS((level ? OMAP_IH2_BASE : OMAP_IH1_BASE) + (reg))));
}

inline u32
read_ih(int level, int reg)
{
	return inl((IO_ADDRESS((level ? OMAP_IH2_BASE : OMAP_IH1_BASE)
		+ (reg))));
}

static inline int
get_level(int irq)
{
	return (((irq) < IH2_BASE) ? 0 : 1);
}

static inline int
get_irq_num(int irq)
{
	return (((irq) < IH2_BASE) ? irq : irq - IH2_BASE);
}

static void
mask_irq(unsigned int irq)
{
	int level = get_level(irq);
	int irq_num = get_irq_num(irq);
	u32 mask = read_ih(level, IRQ_MIR) | (1 << irq_num);

	write_ih(level, IRQ_MIR, mask);	// mask the interrupt
}

static void
ack_irq(unsigned int irq)
{
	int level = get_level(irq);

	do {
		write_ih(level, IRQ_CONTROL_REG, 0x1);
		/* REVISIT: So says the TRM:
		 *	if (level) write_ih(0, ITR, 0);
		 */
	} while (level--);
}

static void
unmask_irq(unsigned int irq)
{
	int level = get_level(irq);
	int irq_num = get_irq_num(irq);
	u32 mask = read_ih(level, IRQ_MIR) & ~(1 << irq_num);
	write_ih(level, IRQ_MIR, mask);
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
	int level, irq_num;
	unsigned long reg_value, reg_addr;

	level = get_level(irq);
	irq_num = get_irq_num(irq);
        // FIQ only available on level 0 interrupts
        fiq = level ? 0 : (fiq & 0x1);
	reg_value = (fiq) | ((priority & 0x1f) << 2) |
	    ((trigger & 0x1) << 1);
	reg_addr = (IRQ_ILR0 + irq_num * 0x4);
	write_ih(level, reg_addr, reg_value);
}

void
omap1510_init_irq(void)
{
	int i;
	uint trigger;

	write_ih(0, IRQ_MIR, ~0x0);	// mask all interrupts
	write_ih(0, IRQ_ITR, 0x0);	// clear all interrupts
	write_ih(1, IRQ_MIR, ~0x0);	// mask all interrupts
	write_ih(1, IRQ_ITR, 0x0);	// clear all interrupts
	write_ih(0, IRQ_CONTROL_REG, 3); /* clear any pending interrupt */
	write_ih(1, IRQ_CONTROL_REG, 3); /* clear any pending interrupt */

	for (i = 0; i < IH_GPIO_BASE; i++) {
		irq_desc[i].valid = 1;
		irq_desc[i].probe_ok = 0;
		irq_desc[i].mask_ack = mask_ack_irq;
		irq_desc[i].mask = mask_irq;
		irq_desc[i].unmask = unmask_irq;

		trigger = ((i < 32) ? (0xb3febfff >> i) :
			   (0xffbfffed >> (i - 32))) & 1;

		irq_priority(i, 0, 0, trigger);
	}

	gpio_init_irq();
	unmask_irq(INT_IH2_IRQ);
}
