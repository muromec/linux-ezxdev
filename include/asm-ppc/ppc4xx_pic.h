/*
 * BK Id: SCCS/s.ppc4xx_pic.h 1.14 08/13/02 22:03:43 paulus
 */
/*
 *
 *    Copyright (c) 1999 Grant Erickson <grant@lcse.umn.edu>
 *
 *    Module name: ppc4xx_pic.h
 *
 *    Description:
 *      Interrupt controller driver for PowerPC 4xx-based processors.
 *
 *      July 17, 2002 - Armin
 *      added sense and polarity ability to 4xx
 */

#ifndef	__PPC4XX_PIC_H__
#define	__PPC4XX_PIC_H__

#include <linux/config.h>
#include <linux/irq.h>

/* External Global Variables */
/*
 * For the IBM4xxPIC_InitSenses table, we include both the sense
 * and polarity in one number and mask out the value we want
 * later on. -- Tom
 */
#define IBM4xx_IRQ_SNS_MASK		0x1
#define IBM4xx_IRQ_SNS_LVL		0x0
#define IBM4xx_IRQ_SNS_EDG		0x1

#define IBM4xx_IRQ_POL_MASK		0x2
#define IBM4xx_IRQ_POL_POS		0x2
#define IBM4xx_IRQ_POL_NEG		0x0

extern struct hw_interrupt_type *ppc4xx_pic;
extern unsigned int ibm4xxPIC_NumInitSenses;
extern unsigned char *ibm4xxPIC_InitSenses;

/* Function Prototypes */

extern void ppc4xx_pic_init(void);
extern int ppc4xx_pic_get_irq(struct pt_regs *regs);

#endif				/* __PPC4XX_PIC_H__ */
