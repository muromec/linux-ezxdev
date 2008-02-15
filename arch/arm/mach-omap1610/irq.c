/*
 * linux/arch/arm/mach-omap/irq.c
 *
 * OMAP irq interface
 * 
 * Copyright (C) 2004 MontaVista Software, Inc.
 *  <source@mvista.com>
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

static u32 omap_ih2_base[] = {OMAP_IH2_0_BASE, OMAP_IH2_1_BASE, OMAP_IH2_2_BASE, OMAP_IH2_3_BASE};

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
	u16 gpio_unmasked_ints = inw(GPIO_IRQENABLE1_REG);
	u16 gpio_inputs = inw(GPIO_DATAIN_REG);
	u32 gpio_edge = inw(GPIO_EDGE_CTRL1_REG)|(((u32)inw(GPIO_EDGE_CTRL2_REG))<<16);
	
	u16 gpio_status = inw(GPIO_IRQSTATUS1_REG);
	u16 pending;

	u16 gpio_falling_mask=0, gpio_rising_mask=0;
	while(gpio_edge)
	{
		gpio_falling_mask=(gpio_falling_mask>>1)|(((gpio_edge&0x3)==0x1)?0x8000:0);
		gpio_rising_mask=(gpio_rising_mask>>1)|(((gpio_edge&0x3)==0x2)?0x8000:0);
		gpio_edge>>=2;
	}
	pending = gpio_unmasked_ints & gpio_status;
	pending |= gpio_unmasked_ints &
		((gpio_rising_mask & gpio_inputs) | (gpio_falling_mask & ~gpio_inputs));

	pr_debug("----pending: 0x%04x, 0x%04x, 0x%04x, 0x%04x, 0x%04x, 0x%04x\n",
		 (int)gpio_unmasked_ints, (int)gpio_inputs, (int)gpio_status, 
		 (int)gpio_falling_mask, (int)gpio_rising_mask, (int)pending);
	return pending;
}

static void
gpio_mask_irq(unsigned int irq)
{
	u16 mask = 1 << (irq - IH_GPIO_BASE);

	pr_debug("---mask: %d",irq);
	/* Set the bit to disable the interrupt */
	outw( mask, GPIO_CLEAR_IRQENABLE1_REG);
}

static void
gpio_ack_irq(unsigned int irq)
{
	u16 mask = 1 << (irq - IH_GPIO_BASE);

	pr_debug("---ask: %d",irq);
	/* Set the bit to clear the status  */
	outw(mask, GPIO_IRQSTATUS1_REG);
}

static void
gpio_unmask_irq(unsigned int irq)
{
	u16 mask = 1 << (irq - IH_GPIO_BASE);

	pr_debug("---unmask: %d",irq);
	/* unmask the specific GPIO interrupt */
	outw( mask, GPIO_SET_IRQENABLE1_REG);

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
		outl(0x00000100, omap_ih2_base[3] + IRQ_ISR);
		pr_debug("---unmask1");
		pr_debug("---unmask2");
	}
}

static void
gpio_mask_ack_irq(unsigned int irq)
{
	pr_debug("---mask-ask");
	gpio_mask_irq(irq);
	gpio_ack_irq(irq);
}

static void
omap1610_gpio_irq_demux(int irq, void *dev_id, struct pt_regs *regs)
{
	u16 pending;
	int gpio_irq;

	pr_debug("---demux: %d",irq);
	/* Since the level 1 GPIO interrupt cascade (IRQ14) is configured as 
	 * edge-sensitive, we need to unmask it here in order to avoid missing 
	 * any additional GPIO interrupts that might occur after the last time 
	 * we check for pending GPIO interrupts here.
	 * We are relying on the fact that this interrupt handler was installed 
	 * with the SA_INTERRUPT flag so that interrupts are disabled at the 
	 * CPU while it is executing.
	 */

	for (;;) {
		pending = get_gpio_unmasked_irqs();
		if (!pending)
			break;

		for (gpio_irq = IH_GPIO_BASE;
			(gpio_irq < (IH_GPIO_BASE + 16)) && pending;
			gpio_irq++, pending >>= 1) {
			if (pending & 1)
			{
				pr_debug("---do_irq: %d",gpio_irq);
				do_IRQ(gpio_irq, regs);
			}
		}
	}
}

static struct irqaction omap1610_gpio_irq = {
	.name		= "OMAP1610 GPIO IRQ demux",
	.handler	= omap1610_gpio_irq_demux,
	.flags		= SA_INTERRUPT
};

static void
gpio_init_irq(void)
{
	int i;

	/* mask all GPIO interrupts */
	outw(0xffff, GPIO_CLEAR_IRQENABLE1_REG);
	/* turn in clock */
	outw(inw(ULPD_CAM_CLK_CTRL) | 0x4, ULPD_CAM_CLK_CTRL);
	/* clear any pending GPIO interrupts */
	outw(0xffff, GPIO_IRQSTATUS1_REG);
	/* configure all GPIO interrupts to trigger on falling edge */
#ifdef CONFIG_OMAP_H2	
	outw(0x5555, GPIO_EDGE_CTRL1_REG);
#else
	outw(0x5556, GPIO_EDGE_CTRL1_REG);
#endif	
	outw(0x5555, GPIO_EDGE_CTRL2_REG);

	for (i = IH_GPIO_BASE; i < (IH_GPIO_BASE + 16); i++) {
		irq_desc[i].valid = 1;
		irq_desc[i].probe_ok = 0;
		irq_desc[i].mask_ack = gpio_mask_ack_irq;
		irq_desc[i].mask = gpio_mask_irq;
		irq_desc[i].unmask = gpio_unmask_irq;
	}

	setup_arm_irq(INT_GPIO1, &omap1610_gpio_irq);
	setup_arm_irq(IH2_LAST-23, &omap1610_gpio_irq);
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
	u16 gpio_unmasked_ints = ~inw(MPUIO_MASKIT_REG);
	u16 gpio_inputs = inw(MPUIO_INPUT_LATCH_REG);
	u16 gpio_control = inw(MPUIO_INT_EDGE_REG);
	u16 gpio_status = inw(MPUIO_INT_REG);

	u16 pending = /* gpio_unmasked_ints &*/ gpio_status; //Masked in hardware

	pending |= gpio_unmasked_ints &
		((gpio_control & gpio_inputs) | ~(gpio_control | gpio_inputs));

	return pending;
}

static void
mpuio_mask_irq(unsigned int irq)
{
	u16 mask = 1 << (irq - IH_MPUIO_BASE);

	/* Set the bit to disable the interrupt */
	outw((inw(MPUIO_MASKIT_REG) | mask), MPUIO_MASKIT_REG);
}

static void
mpuio_unmask_irq(unsigned int irq)
{
	u16 mask = 1 << (irq - IH_MPUIO_BASE);

	/* unmask the specific GPIO interrupt */
	outw((inw(MPUIO_MASKIT_REG) & ~mask), MPUIO_MASKIT_REG);
}

static void
omap1610_mpuio_irq_demux(int irq, void *dev_id, struct pt_regs *regs)
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
	for (mpuio_irq = IH_MPUIO_BASE;
	     (mpuio_irq < (IH_MPUIO_BASE + 16)) && pending;
	     mpuio_irq++, pending >>= 1)
	{
		if (pending & 1)
			do_IRQ(mpuio_irq, regs);
	}
}

static struct irqaction omap1610_mpuio_irq = {
	.name		= "OMAP1610 MPUIO IRQ demux",
	.handler	= omap1610_mpuio_irq_demux,
	.flags		= SA_INTERRUPT
};

static void
mpuio_init_irq(void)
{
	int i;

	/* mask all MPUIO interrupts */
	outw(0xffff, MPUIO_MASKIT_REG);
	/* clear any pending MPUIO interrupts */
	inw(MPUIO_INT_REG);
	/* configure all MPUIO interrupts to trigger on falling edge */
	outw(0x0000, MPUIO_INT_EDGE_REG);

	for (i = IH_MPUIO_BASE; i <= INT_MPUIO_LAST; i++) {
		irq_desc[i].valid = 1;
		irq_desc[i].probe_ok = 0;
		irq_desc[i].mask_ack = mpuio_mask_irq;
		irq_desc[i].mask = mpuio_mask_irq;
		irq_desc[i].unmask = mpuio_unmask_irq;
	}
	setup_arm_irq(INT_MPUIO, &omap1610_mpuio_irq);
}

/*
 *      Standard irq routines
 */

static inline void
write_ih(int level, int part, int reg, u32 value)
{
	outl(value,
	     (IO_ADDRESS((level ? omap_ih2_base[part]: OMAP_IH1_BASE) + (reg))));
}

inline u32
read_ih(int level, int part, int reg)
{
	return inl((IO_ADDRESS((level ? omap_ih2_base[part] : OMAP_IH1_BASE)
		+ (reg))));
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
	do {
		write_ih(level, 0, IRQ_CONTROL_REG, 0x1);
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
omap1610_init_irq(void)
{
	int i;
	static uint trigger[5]={0x23ffdf3f,0xfdb7fffd,0x801ff7ff,0xdfffffff,0xfffffeff};

	write_ih(0, 0, IRQ_MIR, ~0x0);	// mask all interrupts
	write_ih(0, 0, IRQ_ITR, 0x0);	// clear all interrupts
	for(i=0;i<4;i++)
	{
		write_ih(1, i, IRQ_MIR, ~0x0);	// mask all interrupts
		write_ih(1, i, IRQ_ITR, 0x0);	// clear all interrupts
	}
	write_ih(1, 0, IRQ_CONTROL_REG, 3); /* clear any pending interrupt */
	write_ih(0, 0, IRQ_CONTROL_REG, 3); /* clear any pending interrupt */

	for (i = 0; i <= IH2_LAST; i++) {
		irq_desc[i].valid = 1;
		irq_desc[i].probe_ok = 0;
		irq_desc[i].mask_ack = mask_ack_irq;
		irq_desc[i].mask = mask_irq;
		irq_desc[i].unmask = unmask_irq;

		irq_priority(i, 0, 0, (trigger[i>>5] >> (i & 0x1F)) & 0x1);
	}
	gpio_init_irq();
	mpuio_init_irq();
	unmask_irq(INT_IH2_IRQ);
}

/* to be fully implemented - set irq sensitivity to front, rear or levels
 */
int omap1610_tune_irq(unsigned int irq, u32 flags)
{
	if(irq < IH_MPUIO_BASE || irq > INT_MPUIO_LAST)
		return -1;
	if(flags != IFL_LOWLEVEL && flags != IFL_HIGHLEVEL)
		return -2;
	outw((inw(MPUIO_INT_EDGE_REG)&~(1<<(irq-IH_MPUIO_BASE))|
	      (((flags==IFL_HIGHLEVEL)?1:0)<<(irq-IH_MPUIO_BASE))), MPUIO_INT_EDGE_REG);
	return 0;
}

EXPORT_SYMBOL(omap1610_tune_irq);
