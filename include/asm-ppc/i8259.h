/*
 * BK Id: SCCS/s.i8259.h 1.17 10/16/02 10:15:45 paulus
 */

#ifndef _PPC_KERNEL_i8259_H
#define _PPC_KERNEL_i8259_H

#include <linux/irq.h>

extern struct hw_interrupt_type i8259_pic;

void i8259_init(unsigned long int_ack);
int i8259_irq(struct pt_regs *regs);

#endif /* _PPC_KERNEL_i8259_H */
