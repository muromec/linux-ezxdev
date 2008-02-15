/*
 *  irq.c
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

#define WRITE_IC(r,v)       outl(v,((IO_ADDRESS(TI925_IC_BASE)+(r))))
#define READ_IC(r)          inl((IO_ADDRESS(TI925_IC_BASE)+(r)))

#define WRITE_HIO_IC(r,v)   outw(v,IO_ADDRESS(TI925_HIO_IC)+(r))
#define READ_HIO_IC(r)      inw(IO_ADDRESS(TI925_HIO_IC)+(r))

#undef DEBUG
#if defined(DEBUG)
#define DBG(x) printk x
#else
#define DBG(x)
#endif

#define TIMER_IRQ 9

extern void IRQ_interrupt(void);
extern void timer_IRQ_interrupt(void);
extern void fast_IRQ_interrupt(void);
extern void bad_IRQ_interrupt(void);
extern void probe_IRQ_interrupt(void);
void irq_priority(int irq, int fiq, int priority);

void
mask_irq(unsigned int irq)
{
	unsigned long mask;

	if (irq < MAX_INT_BASE) {
		mask = READ_IC(IRQ_MIR);
		mask = mask | (1 << irq);

		DBG((__FUNCTION__ " mask: %8.8x\n", (int) mask));

		WRITE_IC(IRQ_MIR, mask);	// mask the interrupt
	} else {
		// for this level we don't mask the top level interrupt
		irq -= MAX_INT_BASE;
		mask = READ_HIO_IC(IRQ_HIO_IMR);
		mask |= (1 << irq);

		DBG((__FUNCTION__ " hio irq %d mask: %8.8x\n", irq,
		     (int) mask));

		WRITE_HIO_IC(IRQ_HIO_IMR, mask);
	}
}

void
ack_irq(unsigned int irq)
{
	// NOTE: must be called with either the pending irq masked (if level) or 
	// a trigger irq
	unsigned long mask;

	mask = READ_IC(IRQ_ITR);
	if (irq != 9)
		DBG((__FUNCTION__ " mask: %8.8x\n", (int) mask));

	WRITE_IC(IRQ_CONTROL_REG, 0x3);
}

void
unmask_irq(unsigned int irq)
{
	unsigned long mask;

	if (irq >= MAX_INT_BASE) {
		int irq_hio = irq - MAX_INT_BASE;
		irq = INT_EXT_INT2;
		mask = READ_HIO_IC(IRQ_HIO_IMR);
		mask |= (1 << irq_hio);
		DBG((__FUNCTION__ " hio irq %d mask: %8.8x\n", irq_hio,
		     (int) mask));

		WRITE_HIO_IC(IRQ_HIO_IMR, mask);
	}

	mask = READ_IC(IRQ_MIR) & ~(1 << irq);

	DBG((__FUNCTION__ " mask: %8.8x\n", (int) mask));
	if (irq != TIMER_IRQ)
		DBG((__FUNCTION__ " mask: %8.8x\n", (int) mask));

	WRITE_IC(IRQ_MIR, 0);	//mask);
	ack_irq(irq);
}

void
mask_ack_irq(unsigned int irq)
{
	DBG((__FUNCTION__ "\n"));
	mask_irq(irq);
	ack_irq(irq);
}

void
ack_level_irq(int irq)
{
	mask_ack_irq(irq);
	unmask_irq(irq);
}

static __inline__ unsigned long
get_enabled_irqs(void)
{
	return ~READ_IC(IRQ_ITR);	// not the mask register
}

void
irq_init_irq(void)
{
	int i;
	DBG((__FUNCTION__ "\n"));
	for (i = 0; i < NR_IRQS; i++) {
		irq_desc[i].valid = 1;
		irq_desc[i].probe_ok = 1;
		irq_desc[i].mask_ack = mask_ack_irq;
		irq_desc[i].mask = mask_irq;
		irq_desc[i].unmask = unmask_irq;

		DBG(("initing irq %d\n", i));

		irq_priority(i, 0, 0);	// clear out the priorites
	}

	WRITE_IC(IRQ_MIR, ~0x0);	// mask all interrupts
	WRITE_IC(IRQ_ITR, 0x0);	// clear all interrupts
	WRITE_HIO_IC(IRQ_HIO_IMR, 0x0);
}

int
fixup_irq(unsigned int irq)
{
	DBG((__FUNCTION__ "\n"));

	// prioritize the irqs
	irq = 0;

	irq = READ_IC(IRQ_SIR_FIQ);

	if (!irq)
		irq = READ_IC(IRQ_SIR_IRQ);

	if (irq) {
		unsigned int mask;
		int i;

		DBG((__FUNCTION__ " irq %d\n", irq));
		if (irq != INT_EXT_INT2)
			return irq;

		// fix the interrupt on the board
		mask = READ_HIO_IC(IRQ_HIO_IMR);

		for (i = 0; i <= MAX_INT_HIO_BASE; i++)
			if (mask & (1 << i))
				break;

		if (i <= MAX_INT_HIO_BASE) {
			irq = i + MAX_INT_BASE;
			return irq;
		}
	}
	printk(__FUNCTION__ " spur IRQ\n");
	return NR_IRQS;		// spurrious IRQ
}

void
irq_priority(int irq, int fiq, int priority)
{
	unsigned long reg_value = ((fiq & 0x1)) | ((priority & 0xf) << 3);
	unsigned long reg_addr = (IRQ_ILR0 + irq * 0x8);

	if (irq < MAX_INT_BASE)
		WRITE_IC(reg_addr, reg_value);
}
