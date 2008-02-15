/*
 * BK Id: SCCS/s.hw_irq.h 1.17 12/19/02 16:32:51 paulus
 */
/*
 * Copyright (C) 1999 Cort Dougan <cort@cs.nmt.edu>
 */
#ifdef __KERNEL__
#ifndef _PPC_HW_IRQ_H
#define _PPC_HW_IRQ_H

extern unsigned long timer_interrupt_intercept;
extern unsigned long do_IRQ_intercept;
extern int timer_interrupt(struct pt_regs *);
extern void ppc_irq_dispatch_handler(struct pt_regs *regs, int irq);

#define mfmsr()		({unsigned int rval; \
			asm volatile("mfmsr %0" : "=r" (rval)); rval;})
#define mtmsr(v)	asm volatile("mtmsr %0" : : "r" (v))

#define irqs_disabled()	((mfmsr() & MSR_EE) == 0)

#ifdef CONFIG_ILATENCY

extern void intr_cli(const char*, unsigned);
extern void intr_sti(const char *, unsigned, int);
extern void intr_restore_flags(const char *, unsigned, unsigned);

#define __cli()                intr_cli(__BASE_FILE__,__LINE__);
#define __sti()                intr_sti(__BASE_FILE__,__LINE__,0);
#define __restore_flags(flags) intr_restore_flags(__BASE_FILE__,__LINE__, flags);

extern void __intr_sti(void);
extern void __intr_cli(void);
extern void __intr_restore_flags(unsigned long);

#else 
extern void __sti(void);
extern void __cli(void);
extern void __restore_flags(unsigned long);
#endif

extern void __save_flags_ptr(unsigned long *);
extern unsigned long __sti_end, __cli_end, __restore_flags_end, __save_flags_ptr_end;

#define __save_flags(flags) __save_flags_ptr((unsigned long *)&flags)
#define __save_and_cli(flags) ({__save_flags(flags);__cli();})
#define __save_and_sti(flags) ({__save_flags(flags);__sti();})

extern void do_lost_interrupts(unsigned long);

#define mask_irq(irq) ({if (irq_desc[irq].handler && irq_desc[irq].handler->disable) irq_desc[irq].handler->disable(irq);})
#define unmask_irq(irq) ({if (irq_desc[irq].handler && irq_desc[irq].handler->enable) irq_desc[irq].handler->enable(irq);})
#define ack_irq(irq) ({if (irq_desc[irq].handler && irq_desc[irq].handler->ack) irq_desc[irq].handler->ack(irq);})

/* Should we handle this via lost interrupts and IPIs or should we don't care like
 * we do now ? --BenH.
 */
struct hw_interrupt_type;
static inline void hw_resend_irq(struct hw_interrupt_type *h, unsigned int i) {}


#endif /* _PPC_HW_IRQ_H */
#endif /* __KERNEL__ */
