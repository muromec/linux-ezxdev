/*
 *  linux/include/asm-arm/arch-omap710/irq.h
 *
 * BRIEF MODULE DESCRIPTION
 *   TI925 irq interface
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

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#  define DBG(fmt, args...) printk( "%s: " fmt, __FUNCTION__ , ## args)
#  define ENTRY() DBG(":entry\n")
#else
#  define DBG(fmt, args...)
#  define ENTRY()
#endif

/*
 *      FPGA irq routines
 */
/* The FPGA multiplexes the interrupts from several peripherals into one
   interrupt line that goes to GPIO pin 72. We treat the FPGA like another
   level of interrupt handling.
   (gmcnutt)
 */
#ifdef PARANOIA
#define FPGA_CHECK_IRQ(irq) \
	if ((irq < IH_FPGA_BASE) || (irq > (IH_FPGA_BASE + 16))) \
		BUG();
#else
#define FPGA_CHECK_IRQ(irq)
#endif

void
fpga_mask_irq(unsigned int irq)
{
	FPGA_CHECK_IRQ(irq);

	irq -= IH_FPGA_BASE;

	if (irq < 8)
		outb((inb(FPGA_IMR_LO) & ~(1 << irq)), FPGA_IMR_LO);
	else
		outb((inb(FPGA_IMR_HI) & ~(1 << (irq - 8))), FPGA_IMR_HI);
}

void
fpga_ack_irq(unsigned int irq)
{
	// Subtle. The GPIO level is edge-triggered, but the FPGA bits are
	// level-sensitive. Consider:
	// 1. ethernet chip asserts its irq to the fpga
	// 2. fpga asserts its irq to the gpio, which latches it
	// 3. ethernet irq handler runs
	// 4. ethernet chip deasserts its irq to the fpga
	// 5. fpga deasserts to gpio (but it's still latched)
	// 6. do_IRQ finds gpio irq still asserted, so calls gpio_fixup_irq
	// 7. which calls fpga_fixup_irq
	// 8. which finds nothing asserted so it returns
	// 9. just before gpio_fixup_irq() acks the latched irq (which would
	//    clear it), the ethernet chip asserts its irq line
	// 10. gpio_fixup_irq acks the fpga bit and clears it, and so we miss
	//    the new ethernet interrupt
	//
	// To eliminate this race we must eliminate the time delay between
	// fpga_fixup_irq and gpio_fixup_irq. The way to do that is to ack
	// the GPIO pin *here*, after we've correctly identified the interrupt,
	// and before we've called its handler (see do_IRQ() for full context).
	//
	// However, if more than one irq into the FPGA is asserted then we
	// cannot clear the GPIO irq pin or we will lose the one we haven't
	// started handling yet. In that case we need to leave the GPIO irq
	// asserted and clear it the next time through when we process the
	// last one.
	//

	int stat, count = 0;

	// Get the unmasked irqs asserted to the fpga.
	stat = (inb(FPGA_ISR_HI) & inb(FPGA_IMR_HI)) << 8 |
	    (inb(FPGA_ISR_LO) & inb(FPGA_IMR_LO));

	// Check if more than one is set. If so then do *not* ack the irq
	// asserted by the fpga to the gpio.
	while (stat) {
		if (stat & 1) {
			if (++count > 1) {
				printk("Multiple FPGA ints");
				return;
			}
		}
		stat = stat >> 1;
	}

	// If we got here then it's safe to ack at the gpio level.
	outw((inw(GPIO3_INT_STATUS_REG) | (1 << (INT_FPGA - IH_GPIO_BASE))),
	     GPIO3_INT_STATUS_REG);

}

void
fpga_unmask_irq(unsigned int irq)
{
	FPGA_CHECK_IRQ(irq);

	irq -= IH_FPGA_BASE;

	if (irq < 8)
		outb((inb(FPGA_IMR_LO) | (1 << irq)), FPGA_IMR_LO);
	else
		outb((inb(FPGA_IMR_HI) | (1 << (irq - 8))), FPGA_IMR_HI);
}

void
fpga_mask_ack_irq(unsigned int irq)
{
	FPGA_CHECK_IRQ(irq);

	fpga_mask_irq(irq);
	fpga_ack_irq(irq);
}

void
fpga_init_irq(void)
{
	int i;

	outb(0, FPGA_IMR_LO);
	outb(0, FPGA_IMR_HI);

	/* I allow probing of the FPGA interrupts for the ethernet driver. This
	   means one less custom modification to the stock smc9194 driver. Probing
	   means that we'll briefly unmask probable irqs and try to force a device
	   to cause an interrupt. Spurious interrupts are automatically masked
	   back out, so the danger should be pretty low... (gjm)
	 */
	for (i = IH_FPGA_BASE; i < (IH_FPGA_BASE + 16); i++) {
		irq_desc[i].valid = 1;
		irq_desc[i].probe_ok = 1;
		irq_desc[i].mask_ack = fpga_mask_ack_irq;
		irq_desc[i].mask = fpga_mask_irq;
		irq_desc[i].unmask = fpga_unmask_irq;
	}

}
int
fpga_fixup_irq(int irq)
{
	int stat;

	stat = (inb(FPGA_ISR_HI) & inb(FPGA_IMR_HI)) << 8 |
	    (inb(FPGA_ISR_LO) & inb(FPGA_IMR_LO));
	if (!stat)
		return irq;

	irq = IH_FPGA_BASE;

	while (!(stat & 1)) {
		stat = stat >> 1;
		irq++;
	}

	return irq;

}

/*
 *      GPIO irq routines
 */
/* The GPIO multiplexes the interrupts from all of its pins into one irq. We
   treat GPIO like another level of interrupt handlers.
   (gmcnutt)
 */
#ifdef PARANOIA
#define GPIO_CHECK_IRQ(irq) \
	if ((irq < IH_GPIO_BASE) || (irq > (IH_GPIO_BASE + 16))) \
		BUG();
#else
#define GPIO_CHECK_IRQ(irq)
#endif
void
gpio_mask_irq(unsigned int irq)
{
	GPIO_CHECK_IRQ(irq);

	/* Hack -- don't mask out FPGA */
	if (irq == INT_FPGA)
		return;

	irq -= IH_GPIO_BASE;
	/* Set the bit to disable the interrupt */
	outw((inw(GPIO3_INT_MASK_REG) | (1 << irq)), GPIO3_INT_MASK_REG);
}

void
gpio_ack_irq(unsigned int irq)
{
	GPIO_CHECK_IRQ(irq);
	/* Set the bit to clear the status  */
	outw((inw(GPIO3_INT_STATUS_REG) | (1 << irq)), GPIO3_INT_STATUS_REG);
}

void
gpio_unmask_irq(unsigned int irq)
{
	GPIO_CHECK_IRQ(irq);
	irq -= IH_GPIO_BASE;
	/* Clear the bit to enable the interrupt */
	outw((inw(GPIO3_INT_MASK_REG) & ~(1 << irq)), GPIO3_INT_MASK_REG);	// ok!
}

void
gpio_mask_ack_irq(unsigned int irq)
{
	GPIO_CHECK_IRQ(irq);
	gpio_mask_irq(irq);
	gpio_ack_irq(irq);
}

void
gpio_init_irq(void)
{
	int i;

	outw(0xffff, GPIO3_INT_MASK_REG);
	outw(0xffff, GPIO3_INT_STATUS_REG);

	for (i = IH_GPIO_BASE; i < (IH_GPIO_BASE + 16); i++) {
		irq_desc[i].valid = 1;
		irq_desc[i].probe_ok = 0;
		irq_desc[i].mask_ack = gpio_mask_ack_irq;
		irq_desc[i].mask = gpio_mask_irq;
		irq_desc[i].unmask = gpio_unmask_irq;
	}

//	fpga_init_irq();

	/* The FPGA interrupt line comes in on pin 72 (-64 base for group gpio3,
	 * so bit is 8). Claim this pin for the ARM interrupt. */

	/* Configure it as an input pin. */
//	outw((inw(GPIO3_DIR_CONTROL_REG) | (1 << 8)), GPIO3_DIR_CONTROL_REG);

	/* Configure it to fire on high-to-low */
//	outw((inw(GPIO3_INT_CONTROL_REG) & ~(1 << 8)), GPIO3_INT_CONTROL_REG);

	/* Always leave the FPGA pin interrupt enabled. Interrupts coming in
	   from the FPGA peripherals should be masked out in the FPGA
	   registers, not here. */
//	gpio_unmask_irq(INT_FPGA);

}
int
gpio_fixup_irq(int irq)
{
	int stat;

	/* Invert the mask because 0 = enabled */
	stat = inw(GPIO3_INT_STATUS_REG) & ~(inw(GPIO3_INT_MASK_REG));

	if (!stat)
		return irq;

	irq = IH_GPIO_BASE;

	while (!(stat & 1)) {
		stat = stat >> 1;
		irq++;
	}

	if (irq == INT_FPGA)
		irq = fpga_fixup_irq(irq);

	return irq;
}

/*
 *      Standard irq routines
 */
#ifdef PARANOIA
#define CHECK_IRQ(irq) \
	if ((irq < 0) || (irq > (IH2_BASE + 32))) \
		BUG();
#else
#define CHECK_IRQ(irq)
#endif

inline void
write_ih(int level, int reg, int value)
{
//	printk("write addr: %8.8x value: %x\n", (IO_ADDRESS((level ? OMAP710_IH2_BASE : OMAP710_IH1_BASE) +   (reg))), value);
	outl(value,
	     (IO_ADDRESS((level ? OMAP710_IH2_BASE : OMAP710_IH1_BASE) +
			 (reg))));
}

inline unsigned long
read_ih(int level, int reg)
{
	unsigned long value =
	    inl((IO_ADDRESS
		 ((level ? OMAP710_IH2_BASE : OMAP710_IH1_BASE) + (reg))));
	//	printk("read  addr: %8.8x value %x\n",
	//	IO_ADDRESS((level ? OMAP710_IH2_BASE : OMAP710_IH1_BASE) +
	//	(reg)), value);
	return value;
}

inline int
get_level(int irq)
{
	return (((irq) < IH2_BASE) ? 0 : 1);
}

inline int
get_irq_num(int irq)
{
	return (((irq) < IH2_BASE) ? irq : irq - IH2_BASE);
}

void
mask_irq(unsigned int irq)
{
	unsigned long mask;
	int level = get_level(irq);
	int irq_num = get_irq_num(irq);

	CHECK_IRQ(irq);

	/* Hack -- don't mask out GPIO */
	if (irq == INT_MPUIO3)
		return;

	if (irq != TIMER_IRQ)
		ENTRY();
	if (irq != TIMER_IRQ)
		DBG("irq: %d\n", irq);

	mask = read_ih(level, IRQ_MIR);
	mask = mask | (1 << irq_num);

	if (irq != TIMER_IRQ)
		DBG(" mask: %8.8x\n", (int) mask);

	write_ih(level, IRQ_MIR, mask);	// mask the interrupt
}

void
ack_irq(unsigned int irq)
{
	int level = get_level(irq);

	CHECK_IRQ(irq);

	if (irq != TIMER_IRQ)
		ENTRY();

	/* __DO NOT__ try to ack level-sensitive interrupts by clearing their
	   bits in the ITR or you will drop timer interrupts.
	   (gjm)
	 */

	do {
		write_ih(level, IRQ_CONTROL_REG, 0x3);
	} while (level--);

}

void
unmask_irq(unsigned int irq)
{
	unsigned long mask;
	int level = get_level(irq);
	int irq_num = get_irq_num(irq);

	CHECK_IRQ(irq);

	if (irq != TIMER_IRQ)
		ENTRY();
	if (irq != TIMER_IRQ)
		DBG("irq %d\n", irq);

	mask = read_ih(level, IRQ_MIR);
	if (irq != TIMER_IRQ)
		DBG("before mask: %8.8x\n", (int) mask);
	mask &= ~((1 << irq_num));
	if (irq != TIMER_IRQ)
		DBG("after mask : %8.8x\n", (int) mask);
	write_ih(level, IRQ_MIR, mask);
}

void
mask_ack_irq(unsigned int irq)
{

	CHECK_IRQ(irq);

	if (irq != TIMER_IRQ)
		ENTRY();
	mask_irq(irq);
	ack_irq(irq);
}

#if 0				/* dead code? */
void
ack_level_irq(int irq)
{
	if (irq != TIMER_IRQ)
		ENTRY();
	mask_ack_irq(irq);
	unmask_irq(irq);
}
#endif
void
irq_priority(int irq, int fiq, int priority, int trigger)
{
	int level, irq_num;
	unsigned long reg_value, reg_addr;

	CHECK_IRQ(irq);

	level = get_level(irq);
	irq_num = get_irq_num(irq);
	reg_value = ((fiq & 0x1)) | ((priority & 0xf) << 3) |
	    ((trigger & 0x1) << 1);
	reg_addr = (IRQ_ILR0 + irq_num * 0x4);
	write_ih(level, reg_addr, reg_value);
}

void
irq_init_irq(void)
{
	int i;
	uint trigger;
	write_ih(0, IRQ_MIR, ~0x0);	// mask all interrupts
	write_ih(0, IRQ_ITR, 0x0);	// clear all interrupts
	write_ih(1, IRQ_MIR, ~0x0);	// mask all interrupts
	write_ih(1, IRQ_ITR, 0x0);	// clear all interrupts
	for (i = 0; i < IH_GPIO_BASE; i++) {
		irq_desc[i].valid = 1;
		irq_desc[i].probe_ok = 0;
		irq_desc[i].mask_ack = mask_ack_irq;
		irq_desc[i].mask = mask_irq;
		irq_desc[i].unmask = unmask_irq;

		DBG("initing irq %d\n", i);

		trigger = ((i < 32) ? (0xb3fffe4f >> i) : (0x7e80fc61 >> (i - 32))) & 1;

		irq_priority(i, 0, 0, trigger);
	}

	gpio_init_irq();

	unmask_irq(INT_MPUIO3);
//	unmask_irq(INT_IH2_FIQ);
	unmask_irq(INT_IH2_IRQ);

#if 1				/* skranz Mar 2002 */
	// Update, as it turns out there was so a not yet understood
	// side effect with letting the ide driver probe for the IRQ
	// associated with the CF card. It was preventing the ether
	// chipset interrupt from working correctly. For now we will
	// use logic in include/asm-arm/arch-omap710/ide.h to explicitly
	// assign the CF interrupt number to the ide driver so that it
	// will not try and probe for it.
#else
	// When the jumper is on JP36 pins1-2, it enables the CompactFlash slot
	// (as opposed to the SD/MMC slot) which means we will need the ide
	// driver to control it. That driver will expect to probe for the
	// interrupt associated with the device, therefore we will set it
	// probe-able so that when the ide init routines use the system's
	// probe_irq_on() and probe_irq_off() facilities it will be able
	// to deduce the irq num and hence supply a request_irq() call
	// appropriately. The ide driver is not hardcoded to know the
	// interrupt associated with CompactFlash activity -- it discovers
	// it at init time.
	irq_desc[INT_CFIREQ].probe_ok = 1;
	irq_desc[INT_CFCD].probe_ok = 1;
#endif
}

int
fixup_irq(unsigned int irq)
{
	int level = 0;

level1_int:
	irq = read_ih(level, IRQ_SIR_FIQ);

	if (!irq)
		irq = read_ih(level, IRQ_SIR_IRQ);

	if ((irq != 22) && (level == 1))
		DBG(" level %d irq %d\n", level, irq + (level * IH2_BASE));
	if (((irq == 0) || (irq == 1)) && (level == 0)) {
		level = 1;
		goto level1_int;
	}

	irq += (level ? IH2_BASE : 0);

	if (irq == INT_MPUIO3)
		irq = gpio_fixup_irq(irq);

	return irq;
}

/*---------------------------------------------------------------------------
 * Local variables:
 * c-file-style: "linux"
 * End:
 */
