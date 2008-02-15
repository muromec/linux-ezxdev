/*
 * BK Id: SCCS/s.mpc8xx.h 1.20 11/28/01 10:22:04 paulus
 */

/* This is the single file included by all MPC8xx build options.
 * Since there are many different boards and no standard configuration,
 * we have a unique include file for each.  Rather than change every
 * file that has to include MPC8xx configuration, they all include
 * this one and the configuration switching is done here.
 */
#ifdef __KERNEL__
#ifndef __CONFIG_8xx_DEFS
#define __CONFIG_8xx_DEFS

#include <linux/config.h>

#ifdef CONFIG_8xx

#ifdef CONFIG_MBX
#include <platforms/mbx.h>
#endif

#ifdef CONFIG_FADS
#include <platforms/fads.h>
#endif

#ifdef CONFIG_RPXLITE
#include <platforms/rpxlite.h>
#endif

#ifdef CONFIG_BSEIP
#include <platforms/bseip.h>
#endif

#ifdef CONFIG_RPXCLASSIC
#include <platforms/rpxclassic.h>
#endif

#if defined(CONFIG_TQM8xxL)
#include <platforms/tqm8xx.h>
#endif

#if defined(CONFIG_SPD823TS)
#include <platforms/spd8xx.h>
#endif

#if defined(CONFIG_IVMS8) || defined(CONFIG_IVML24)
#include <platforms/ivms8.h>
#endif

#if defined(CONFIG_HERMES_PRO)
#include <platforms/hermes.h>
#endif

#if defined(CONFIG_IP860)
#include <platforms/ip860.h>
#endif

#if defined(CONFIG_LWMON)
#include <platforms/lwmon.h>
#endif

#if defined(CONFIG_PCU_E)
#include <platforms/pcu_e.h>
#endif

#if defined(CONFIG_CCM)
#include <platforms/ccm.h>
#endif

#if defined(CONFIG_LANTEC)
#include <platforms/lantec.h>
#endif

/* The PCI_ISA_IO_ADDR, PCI_ISA_MEM_ADDR, and PCI_DRAM_OFFSET macros
 * must be defined in the board-specific header file for targets that
 * need them.  Default values are defined here for targets that don't need
 * them.
  */
#if defined(CONFIG_PCI)
#if !defined(_IO_BASE)
#define	_IO_BASE PCI_ISA_IO_ADDR
#endif
#if !defined(_ISA_MEM_BASE)
#define	_ISA_MEM_BASE PCI_ISA_MEM_ADDR
#endif
#if !defined(PCI_DRAM_OFFSET)
#define PCI_DRAM_OFFSET 0
#endif
#else	/* if defined(CONFIG_PCI) */
#if !defined(_IO_BASE)
#define _IO_BASE        0
#endif
#if !defined(_ISA_MEM_BASE)
#define _ISA_MEM_BASE   0
#endif
#if !defined(PCI_DRAM_OFFSET)
#define PCI_DRAM_OFFSET 0
#endif
#endif	/* if defined(CONFIG_PCI) */

#ifndef __ASSEMBLY__
/* The "residual" data board information structure the boot loader
 * hands to us.
 */
extern unsigned char __res[];

#define request_8xxirq request_irq

#endif /* !__ASSEMBLY__ */
#endif /* CONFIG_8xx */
#endif /* __CONFIG_8xx_DEFS */
#endif /* __KERNEL__ */
