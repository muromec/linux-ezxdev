/* 
 * drivers/pcmcia/mx21.h
 *
 * Motorola MX21 PCMCIA/CF Controller definitions
 *
 * Author: MontaVista Software, Inc. <source@mvista.com>.
 *
 * This file is based on mx21.h from Motorola Dragonball MX2 ADS BSP
 * Copyright 2002, 2003 Motorola, Inc. All Rights Reserved.
 * Copyright (C) 2000 John G Dorsey <john+@cs.cmu.edu>
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#ifndef _MX21_H
#define _MX21_H

#include <linux/config.h>

#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/ss.h>
#include <pcmcia/bulkmem.h>
#include <pcmcia/cistpl.h>

#include "cs_internal.h"
#include "mx21_generic.h"

#include <asm/arch/hardware.h>

#ifdef CONFIG_MX2TO1
#define MX21_PCMCIA_MAX_WINDOW	7
#else
#define MX21_PCMCIA_MAX_WINDOW	5
#endif

typedef struct _reg_layout {
	unsigned long shift;
	unsigned long mask;
} reg_layout_t;

/* PIPR: PCMCIA Input Pins Register (Read-only)
 *
 * PIPR layout (common part) is:  
 *  
 *   PWRON RDY BVD2/SPER_IN BVDC1/STSCHG_IN CD<1:0> WP VS<1:0>
 * 
 */

/* Index of elements of PIPR layout*/
#define PIPR_POWERON		6
#define PIPR_RDY		5
#define PIPR_BVD2		4
#define PIPR_SPER_IN		4
#define PIPR_BVD1		3
#define PIPR_STSCHG_IN		3
#define PIPR_CD			2
#define PIPR_WP			1
#define PIPR_VS			0

#define PIPR_MAX_INDEX	7
static reg_layout_t pipr_layout[PIPR_MAX_INDEX] = {
	{0, 0x3},
	{2, 0x4},
	{3, 0x18},
	{5, 0x20},
	{6, 0x40},
	{7, 0x80},
	{8, 0x100}
};

/* PSCR: PCMCIA Status Change Register
 *
 * PSCR layout (common part) is:  
 * [11:0] 
 *   POW RDYR RDYF RDYH RDYL BVDC2 BVDC1 CDC2 CDC1 WPC VSC2 VSC1
 * 
 */

/* Index of elements of PSCR layout*/
#define PSCR_POWC		11
#define PSCR_RDYR		10
#define PSCR_RDYF		9
#define PSCR_RDYH		8
#define PSCR_RDYL		7
#define PSCR_BVDC2		6
#define PSCR_BVDC1		5
#define PSCR_CDC2		4
#define PSCR_CDC1		3
#define PSCR_WPC		2
#define PSCR_VSC2		1
#define PSCR_VSC1		0

#define PSCR_MAX_INDEX	12
static reg_layout_t pscr_layout[PSCR_MAX_INDEX] = {
	{0, 0x1},
	{1, 0x2},
	{2, 0x4},
	{3, 0x8},
	{4, 0x10},
	{5, 0x20},
	{6, 0x40},
	{7, 0x80},
	{8, 0x100},
	{9, 0x200},
	{10, 0x400},
	{11, 0x800}
};

/* PER:  PCMCIA Enable Register
 *
 * PER layout (common part) is:  
 * [12:0] 
 *   ERRINTEN PWRONEN RDYRE RDYFE RDYHE RDYLE BVDE2 BVDE1 CDE2 CDE1 WPE VSE2 VSE1
 * 
 */

/* Index of elements of PER layout*/
#define PER_ERRINTEN		12
#define PER_PWRONEN		11
#define PER_RDYRE		10
#define PER_RDYFE		9
#define PER_RDYHE		8
#define PER_RDYLE		7
#define PER_BVDE2		6
#define PER_BVDE1		5
#define PER_CDE2		4
#define PER_CDE1		3
#define PER_WPE			2
#define PER_VSE2		1
#define PER_VSE1		0

#define PER_MAX_INDEX	13
static reg_layout_t per_layout[PER_MAX_INDEX] = {
	{0, 0x1},
	{1, 0x2},
	{2, 0x4},
	{3, 0x8},
	{4, 0x10},
	{5, 0x20},
	{6, 0x40},
	{7, 0x80},
	{8, 0x100},
	{9, 0x200},
	{10, 0x400},
	{11, 0x800},
	{12, 0x1000}
};

/* PBR: PCMCIA Base Register
 *
 * PBR layout is:  
 *
 *   PBA<31:0> for TO-1
 *
 */
#define PBR_PBA		0
#define PBR_MAX_INDEX	1
static reg_layout_t pbr_layout[PBR_MAX_INDEX] = {
	{0, 0xffffffff}
};

/* POR: PCMCIA Option Register
 *
 * POR layout is:  
 *
 *   PV WPEN WP PRS<1:0> PPS PSL<6:0> PSST<5:0> PSHT<5:0> BSIZE<4:0>
 *
 */

/* Index of elements of POR layout*/
#define POR_PV			8
#define POR_WPEN		7
#define POR_WP			6
#define POR_PRS			5
#define POR_PPS			4
#define POR_PSL			3
#define POR_PSST		2
#define POR_PSHT		1
#define POR_BSIZE		0

#define POR_MAX_INDEX	9
static reg_layout_t por_layout[POR_MAX_INDEX] = {
	{0, 0x1f},
	{5, 0x7e0},
	{11, 0x1f800},
	{17, 0xfe0000},
	{24, 0x1000000},
	{25, 0x6000000},
	{27, 0x8000000},
	{28, 0x10000000},
	{29, 0x20000000}
};

/* POFR: PCMCIA Offset Register
 *
 * POFR layout is:  
 *
 *   POFA<25:0>
 *
 */
#define POFR_POFA	0
#define POFR_MAX_INDEX	1
static reg_layout_t pofr_layout[POFR_MAX_INDEX] = {
	{0, 0x03ffffff}
};

/* PGCR: PCMCIA General Control Register
 *
 * PGCR layout is:  
 *
 *   ATASEL ATAMODE LPMEN SPKREN POE RESET
 *
 */

/* Index of elements of PGCR layout*/
#ifdef CONFIG_MX2TO1 /* These bits were removed in TO-2 */
#define PGCR_ATASEL		5
#define PGCR_ATAMODE		4
#endif
#define PGCR_LPMEN		3
#define PGCR_SPKREN		2
#define PGCR_POE		1
#define PGCR_RESET		0

#ifdef CONFIG_MX2TO1 /* ATA bits were removed in TO-2 */
#define PGCR_MAX_INDEX	6
#else
#define PGCR_MAX_INDEX	4
#endif

static reg_layout_t pgcr_layout[PGCR_MAX_INDEX] = {
	{0, 0x1},
	{1, 0x2},
	{2, 0x4},
	{3, 0x8},
#ifdef CONFIG_MX2TO1
	{4, 0x10},
	{5, 0x20}
#endif
};

 /* PGSR: PCMCIA General Status Register
  *
  * PGSR layout is:  
  *
  *   NWINE LPE SE CDE WPE
  *
  */

/* Index of elements of PGSR layout*/
#define PGSR_NWINE	4
#define PGSR_LPE	3
#define PGSR_SE		2
#define PGSR_CDE	1
#define PGSR_WPE	0

#define PGSR_MAX_INDEX	5
static reg_layout_t pgsr_layout[PGSR_MAX_INDEX] = {
	{0, 0x1},
	{1, 0x2},
	{2, 0X4},
	{3, 0X8},
	{4, 0X10}
};

#define reg_element_set(reg, shift, mask, value) \
	( (reg) = ((reg)&(~(mask))) | (((value)<<(shift))&(mask)) )

#define reg_element_get(reg, shift, mask) \
	( ((reg)&(mask))>>(shift) )

#define pipr_read(element) reg_element_get(PCMCIA_PIPR, pipr_layout[element].shift, \
	pipr_layout[element].mask)

#define pipr_write(element,value) reg_element_set(PCMCIA_PIPR, pipr_layout[element].shift, \
	pipr_layout[element].mask,value)

#define pscr_read(element) reg_element_get(PCMCIA_PSCR, pscr_layout[element].shift, \
	pscr_layout[element].mask)

#define pscr_write(element,value) reg_element_set(PCMCIA_PSCR, pscr_layout[element].shift, \
	pscr_layout[element].mask,value)

#define per_read(element) reg_element_get(PCMCIA_PER, per_layout[element].shift, \
	per_layout[element].mask)

#define per_write(element,value) reg_element_set(PCMCIA_PER, per_layout[element].shift, \
	per_layout[element].mask,value)

#define pgcr_read(element) reg_element_get(PCMCIA_PGCR, pgcr_layout[element].shift, \
	pgcr_layout[element].mask)

#define pgcr_write(element,value) reg_element_set(PCMCIA_PGCR, pgcr_layout[element].shift, \
	pgcr_layout[element].mask,value)

#define pgsr_read(element) reg_element_get(PCMCIA_PGSR, pgsr_layout[element].shift, \
	pgsr_layout[element].mask)

#define pgsr_write(element,value) reg_element_set(PCMCIA_PGSR, pgsr_layout[element].shift, \
	pgsr_layout[element].mask,value)

#define pbr_read(window,element) reg_element_get((PCMCIA_PBR(window), pbr_layout[element].shift, \
	pbr_layout[element].mask)

#define pbr_write(window,element,value) reg_element_set(PCMCIA_PBR(window), pbr_layout[element].shift, \
	pbr_layout[element].mask,value)

#define por_read(window,element) reg_element_get(PCMCIA_POR(window), por_layout[element].shift, \
	por_layout[element].mask)

#define por_write(window,element,value) reg_element_set(PCMCIA_POR(window), por_layout[element].shift, \
	por_layout[element].mask,value)

#define pofr_read(window,element) reg_element_get(PCMCIA_POFR(window), pofr_layout[element].shift, \
	pofr_layout[element].mask)

#define pofr_write(window,element,value) reg_element_set(PCMCIA_POFR(window), pofr_layout[element].shift, \
	pofr_layout[element].mask,value)

/* i.MX21 PCMCIA Memory and I/O timing
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 * The PC Card Standard, Release 7, section 4.13.4, says that twIORD
 * has a minimum value of 165ns. Section 4.13.5 says that twIOWR has
 * a minimum value of 165ns, as well. Section 4.7.2 (describing
 * common and attribute memory write timing) says that twWE has a
 * minimum value of 150ns for a 250ns cycle time (for 5V operation;
 * see section 4.7.4), or 300ns for a 600ns cycle time (for 3.3V
 * operation, also section 4.7.4). Section 4.7.3 says that taOE
 * has a maximum value of 150ns for a 300ns cycle time (for 5V
 * operation), or 300ns for a 600ns cycle time (for 3.3V operation).
 *
 * When configuring memory maps, Card Services appears to adopt the policy
 * that a memory access time of "0" means "use the default." The default
 * PCMCIA I/O command width time is 165ns. The default PCMCIA 5V attribute
 * and memory command width time is 150ns; the PCMCIA 3.3V attribute and
 * memory command width time is 300ns.
 */

/* I/O Timings */
#define MX21_PCMCIA_IO_ACCESS      		165
#define MX21_PCMCIA_IO_ADDR_SETUP_TIME 		70
#define MX21_PCMCIA_IO_ADDR_HOLD_TIME 		20

/* Common and Attribute Memory timings */
#define MX21_PCMCIA_5V_MEM_ACCESS  		150
#define MX21_PCMCIA_5V_MEM_ADDR_SETUP_TIME 	30
#define MX21_PCMCIA_5V_MEM_ADDR_HOLD_TIME 	20
#define MX21_PCMCIA_3V_MEM_ACCESS		300
#define MX21_PCMCIA_3V_MEM_ADDR_SETUP_TIME	100
#define MX21_PCMCIA_3V_MEM_ADDR_HOLD_TIME	35

/* The socket driver actually works nicely in interrupt-driven form,
 * so the (relatively infrequent) polling is "just to be sure."
 */
#define MX21_PCMCIA_POLL_PERIOD    (2*HZ)

struct mx21_pcmcia_socket {
	/*
	 * Core PCMCIA state
	 */
	socket_state_t cs_state;
	pccard_io_map io_map[MAX_IO_WIN];
	pccard_mem_map mem_map[MAX_WIN];
	void (*handler) (void *, unsigned int);
	void *handler_info;

	struct pcmcia_state k_state;
	ioaddr_t phys_attr, phys_mem;
	void *virt_io;
	unsigned short speed_io, speed_attr, speed_mem;

	/*
	 * Info from low level handler
	 */
	unsigned int irq;
};

/*
 * Declaration for all implementation specific low_level operations.
 */
extern struct pcmcia_low_level mx21ads_pcmcia_ops;

#endif
