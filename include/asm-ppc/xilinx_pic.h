/*
 * xilinx_pic.h
 *
 * Definitions for interrupt controller driver for Xilinx Virtex-II Pro.
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2002 (c) MontaVista, Software, Inc.  This file is licensed under the terms
 * of the GNU General Public License version 2.1.  This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */

/*
 * This file was derived from ppc4xx_pic.h which is
 *    Copyright (c) 1999 Grant Erickson <grant@lcse.umn.edu>
 */

#ifndef __XILINX_PIC_H__
#define __XILINX_PIC_H__

#include <linux/config.h>
#include <linux/irq.h>

/* External Global Variables */

extern struct hw_interrupt_type *ppc4xx_pic;

/* Function Prototypes */

extern void ppc4xx_pic_init(void);
extern int xilinx_pic_get_irq(struct pt_regs *regs);

#endif				/* __XILINX_PIC_H__ */
