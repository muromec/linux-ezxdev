/* 
 * drivers/pcmcia/mx21_generic.h
 *
 * Motorola MX21 PCMCIA/CF Controller definitions
 *
 * Author: MontaVista Software, Inc. <source@mvista.com>.
 *
 * This file is based on mx_generic.h from Motorola Dragonball MX2 ADS BSP
 * Copyright 2002, 2003 Motorola, Inc. All Rights Reserved.
 * Copyright (C) 2000 John G Dorsey <john+@cs.cmu.edu>
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#ifndef _MX21_GENERIC_H
#define _MX21_GENERIC_H

#define MX21_PCMCIA_MAX_SOCK	(1)

struct pcmcia_init {
	void (*handler) (int irq, void *dev, struct pt_regs * regs);
};

struct pcmcia_state {
	unsigned poweron:1, ready:1, bvd2:1, bvd1:1, detect:1, wrprot:1,
	    vs_3v:1, vs_Xv:1;
};

struct pcmcia_state_array {
	unsigned int size;
	struct pcmcia_state *state;
};

struct pcmcia_configure {
	unsigned sock:8, vcc:8, vpp:8, output:1, speaker:1, reset:1, irq:1;
};

struct pcmcia_irq_info {
	unsigned int sock;
	unsigned int irq;
};

struct pcmcia_low_level {
	int (*init) (struct pcmcia_init *);
	int (*shutdown) (void);
	int (*socket_state) (struct pcmcia_state_array *);
	int (*get_irq_info) (struct pcmcia_irq_info *);
	int (*configure_socket) (const struct pcmcia_configure *);

	/*
	 * Enable card status IRQs on (re-)initialisation.  This can
	 * be called at initialisation, power management event, or
	 * pcmcia event.
	 */
	int (*socket_init) (int sock);

	/*
	 * Disable card status IRQs and PCMCIA bus on suspend.
	 */
	int (*socket_suspend) (int sock);
};

extern struct pcmcia_low_level *pcmcia_low_level;

#endif				/* _MX21_GENERIC_H */
