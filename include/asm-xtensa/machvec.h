/*
 * (Virtual) Mach-Vectors for Xtensa
 *
 * Copyright (C) 2001 Tensilica, Inc.
 * 
 * Author:  Christian Zankel <zankel@tensilica.com> <chris@zankel.net>
 *
 */

#ifndef __ASM_XTENSA_MACHVEC_H
#define __ASM_XTENSA_MACHVEC_H

#include <linux/config.h>
#include <linux/types.h>
#include <linux/pci.h>

#include <asm/platform/machvec.h>
#include <asm/bootparam.h>

/*
 * The Xtensa port provides two methods to call platform specific functions:
 *  - direct linkage, and
 *  - using virtual vectors.
 *
 * To use virtual vectors the variable XTENSA_USE_MACHVEC must be 
 * defined at compile time and the platform must setup the vector table with
 *
 *    XTENSA_MACHVEC_INIT(<name>);
 *
 * (see incluse/asm-xtensa/platform-template/config.h as an example)
 *
 * [ To add a new function follow the instructions in the []- brackets. ]
 *
 * Note:
 *  Software vectors are usually used to provide a generic kernel for 
 *  multiple targets. 
 */

extern void platform_init(bp_tag_t*);

/* ---------------------------------------------------------------------------*/


/* Typedef's for all mach-vectors 
 *
 * [ Add another typedef. Format should be xtvec_<function>_t      ]
 * [   typedef <returns> xtvec_<function>_t [(<parameter list>)    ]
 */

typedef void xtvec_setup_t (char **);
typedef u32 xtvec_calibrate_counter_t (void);
typedef void xtvec_init_irq_t (void);
typedef u8 xtvec_pci_swizzle_t (struct pci_dev *, u8 *);
typedef int xtvec_pci_map_irq_t (struct pci_dev*, u8, u8);
typedef void xtvec_pci_fixup_t (void);

/* ---------------------------------------------------------------------------*/

/* Define all platform_<function> functions to either use the  
 * vector table or to call the functions provided by the platform.
 *
 * [ Add the new platform_<...> instruction here        ]
 * [  #define platform_<function>_t XT_FUNC(<function>) ]
 */

/* Use the vector table entry of the xtvec_<function> directly. */

#ifdef XTENSA_USE_MACHVEC
# define XT_FUNC(func) xtvec.##func
#else
# define XT_FUNC(func) xtvec_##func
#endif

/* Generic functions */
#define platform_setup XT_FUNC(setup)
#define platform_calibrate_counter XT_FUNC(calibrate_counter)
#define platform_init_irq XT_FUNC(init_irq)

/* PCI related functions */
#define platform_pci_swizzle XT_FUNC(pci_swizzle)
#define platform_pci_map_irq XT_FUNC(pci_map_irq)
#define platform_pci_fixup   XT_FUNC(pci_fixup)

/* ---------------------------------------------------------------------------*/

/* This is the mach-vector table.
 *
 * [ Add new function to the structure          ]
 * [  xtvec_<function>_t *<function>;           ]
 *
 */

#ifdef XTENSA_USE_MACHVEC

struct xt_machvec {
	/* generic functions */
	const char *name;
	xtvec_setup_t *setup;
	xtvec_calibrate_counter_t *calibrate_counter;
	xtvec_init_irq_t *init_irq;

	/* pci related functions */
	xtvec_pci_swizzle_t *pci_swizzle;
	xtvec_pci_map_irq_t *pci_map_irq;
	xtvec_pci_fixup     *pci_fixup;
};

#endif /* XTENSA_USE_MACHVEC */

/* ---------------------------------------------------------------------------*/

/* Macro to initialize a mach-vector table
 *
 * [ Simply add the new function (must match the structure above)    ]
 */

#ifdef XTENSA_USE_MACHVEC
# define XTENSA_MACHVEC_INIT(name)			\
struct xtvec 		 				\
__attribute__ ((unused, __section__ (".xtvec"))) =	\
{							\
	#name,						\
	xtvec_setup,					\
	xtvec_calibrate_counter,			\
	xtvec_init_irq,					\
	xtvec_pci_swizzle,				\
	xtvec_pci_map_irq,				\
	xtvec_pci_fixup,				\
} xtvec_##name;

#else
# define XTENSA_MACHVEC_INIT(name)	;
#endif /* XTENSA_USE_MACHVEC */

/* ---------------------------------------------------------------------------*/

/* Provide a default functions if it is not implemented by the target.
 *
 * [ Add another ifndef, define, endif construct here.                 ]
 * [  xtvec_noop is function that simply returns,                      ]
 * [  #ifndef xtvec_<function>                                         ]
 * [  # define xtvec_<function> (<typedef>*) xtvec_noop                ]
 * [  #endif                                                           ]
 */

#ifdef XTENSA_USE_MACHVEC
extern void xtvec_noop(void);
#else
static inline void xtvec_noop(void) { return; };
#endif

#ifndef xtvec_setup
# define xtvec_setup		((xtvec_setup_t *) xtvec_noop)
#endif
#ifndef xtvec_calibrate_counter
# define xtvec_calibrate_counter ((xtvec_calibrate_counter_t *) xtvec_noop) 
#endif
#ifndef xtvec_init_irq
# define xtvec_init_irq		((xtvec_init_irq_t *) xtvec_noop)
#endif
#ifndef xtvec_pci_swizzle
# define xtvec_pci_swizzle	((xtvec_pci_swizzle_t *) xtvec_noop)
#endif
#ifndef xtvec_pci_map_irq
# define xtvec_pci_map_irq	((xtvec_pci_map_t *) xtvec_noop)
#endif
#ifndef xtvec_pci_fixup
# define xtvec_pci_fixup	((xtvec_pci_fixup_t *) xtvec_noop)
#endif

/*
#ifndef xtvec_<name>
# define xtvec_<name>		((xtvec_<name>_t *) xtvec_noop)
#endif
*/

/* ---------------------------------------------------------------------------*/

#endif /* __ASM_XTENSA_MACHVEC_H */

