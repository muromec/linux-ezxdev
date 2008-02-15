/* 
 * drivers/pcmcia/mx21_generic.c
 *
 * Device driver for the PCMCIA control functionality of 
 * Motorola MX21 microprocessors.
 *
 * Author: MontaVista Software, Inc. <source@mvista.com>.
 *
 * This file is based on mx_generic.c from Motorola Dragonball MX2 ADS BSP
 * Copyright 2002, 2003 Motorola, Inc. All Rights Reserved.
 * Copyright (C) 2000 John G Dorsey <john+@cs.cmu.edu>
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/config.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/tqueue.h>
#include <linux/timer.h>
#include <linux/mm.h>
#include <linux/notifier.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/cpufreq.h>

#include <pcmcia/version.h>
#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/ss.h>
#include <pcmcia/bus_ops.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach-types.h>

#include <asm/arch/pll.h>

#include "mx21_generic.h"
#include "mx21.h"

#ifdef PCMCIA_DEBUG
static int pc_debug = PCMCIA_DEBUG;
MODULE_PARM(pc_debug, "i");
#endif

/* This structure maintains housekeeping state for each socket, such
 * as the last known values of the card detect pins, or the Card Services
 * callback value associated with the socket:
 */
static int mx21_pcmcia_socket_count;
static struct mx21_pcmcia_socket mx21_pcmcia_socket[MX21_PCMCIA_MAX_SOCK];

#define PCMCIA_SOCKET(x)	(mx21_pcmcia_socket + (x))

/* Returned by the low-level PCMCIA interface: */
static struct pcmcia_low_level *pcmcia_low_level;

static struct timer_list poll_timer;
static struct tq_struct mx21_pcmcia_task;

/* Forward declarations */
static int
 mx21_pcmcia_set_io_map(unsigned int sock, struct pccard_io_map *map);

/*
 * mx21_pcmcia_state_to_config
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *
 * Convert PCMCIA socket state to our socket configure structure.
 */
static struct pcmcia_configure
mx21_pcmcia_state_to_config(unsigned int sock, socket_state_t * state)
{
	struct pcmcia_configure conf;

	conf.sock = sock;
	conf.vcc = state->Vcc;
	conf.vpp = state->Vpp;
	conf.output = state->flags & SS_OUTPUT_ENA ? 1 : 0;
	conf.speaker = state->flags & SS_SPKR_ENA ? 1 : 0;
	conf.reset = state->flags & SS_RESET ? 1 : 0;
	conf.irq = state->io_irq != 0;

	return conf;
}

/* mx21_pcmcia_init()
 * ^^^^^^^^^^^^^^^^^^^^
 *
 * (Re-)Initialise the socket, turning on status interrupts
 * and PCMCIA bus.  This must wait for power to stabilise
 * so that the card status signals report correctly.
 *
 * Returns: 0
 */
static int
mx21_pcmcia_init(unsigned int sock)
{
	struct mx21_pcmcia_socket *skt = PCMCIA_SOCKET(sock);
	struct pcmcia_configure conf;

	DEBUG(2, "%s(): initializing socket %d\n", __FUNCTION__, sock);

	skt->cs_state = dead_socket;

	conf = mx21_pcmcia_state_to_config(sock, &dead_socket);

	pcmcia_low_level->configure_socket(&conf);

	return pcmcia_low_level->socket_init(sock);
}

/*
 * mx21_pcmcia_suspend()
 * ^^^^^^^^^^^^^^^^^^^^^^^
 *
 * Remove power on the socket, disable IRQs from the card.
 * Turn off status interrupts, and disable the PCMCIA bus.
 *
 * Returns: 0
 */
static int
mx21_pcmcia_suspend(unsigned int sock)
{
	struct mx21_pcmcia_socket *skt = PCMCIA_SOCKET(sock);
	struct pcmcia_configure conf;
	int ret;

	DEBUG(2, "%s(): suspending socket %d\n", __FUNCTION__, sock);

	conf = mx21_pcmcia_state_to_config(sock, &dead_socket);

	ret = pcmcia_low_level->configure_socket(&conf);

	if (ret == 0) {
		skt->cs_state = dead_socket;
		ret = pcmcia_low_level->socket_suspend(sock);
	}

	return ret;
}

/* mx21_pcmcia_events()
 * ^^^^^^^^^^^^^^^^^^^^^^
 * Helper routine to generate a Card Services event mask based on
 * state information obtained from the kernel low-level PCMCIA layer
 * in a recent (and previous) sampling. Updates `prev_state'.
 *
 * Returns: an event mask for the given socket state.
 */
static inline unsigned int
mx21_pcmcia_events(struct pcmcia_state *state,
		   struct pcmcia_state *prev_state,
		   unsigned int mask, unsigned int flags)
{
	unsigned int events = 0;

	if (state->detect != prev_state->detect) {
		DEBUG(3, "%s(): card detect value %d\n", __FUNCTION__,
		      state->detect);

		events |= SS_DETECT;
	}

	if (state->ready != prev_state->ready) {
		DEBUG(3, "%s(): card ready value %d\n", __FUNCTION__,
		      state->ready);

		events |= flags & SS_IOCARD ? 0 : SS_READY;
	}

	if (state->bvd1 != prev_state->bvd1) {
		DEBUG(3, "%s(): card BVD1 value %d\n", __FUNCTION__,
		      state->bvd1);

		events |= flags & SS_IOCARD ? SS_STSCHG : SS_BATDEAD;
	}

	if (state->bvd2 != prev_state->bvd2) {
		DEBUG(3, "%s(): card BVD2 value %d\n", __FUNCTION__,
		      state->bvd2);

		events |= flags & SS_IOCARD ? 0 : SS_BATWARN;
	}

	*prev_state = *state;

	events &= mask;

	DEBUG(2, "events: %s%s%s%s%s%s\n",
	      events == 0 ? "<NONE>" : "",
	      events & SS_DETECT ? "DETECT " : "",
	      events & SS_READY ? "READY " : "",
	      events & SS_BATDEAD ? "BATDEAD " : "",
	      events & SS_BATWARN ? "BATWARN " : "",
	      events & SS_STSCHG ? "STSCHG " : "");

	return events;
}				/* mx21_pcmcia_events() */

/* mx21_pcmcia_task_handler()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 * Processes serviceable socket events using the "eventd" thread context.
 *
 * Event processing (specifically, the invocation of the Card Services event
 * callback) occurs in this thread rather than in the actual interrupt
 * handler due to the use of scheduling operations in the PCMCIA core.
 */
static void
mx21_pcmcia_task_handler(void *data)
{
	struct pcmcia_state state[MX21_PCMCIA_MAX_SOCK];
	struct pcmcia_state_array state_array;
	unsigned int all_events;

	DEBUG(4, "%s(): entering PCMCIA monitoring thread\n", __FUNCTION__);

	state_array.size = mx21_pcmcia_socket_count;
	state_array.state = state;

	DEBUG(4, "%s(): PSCR=0x%x\n", __FUNCTION__, PCMCIA_PSCR);
	DEBUG(4, "%s(): PGSR=0x%x\n", __FUNCTION__, PCMCIA_PGSR);

	do {
		unsigned int events;
		int ret, i;

		memset(state, 0, sizeof (state));

		DEBUG(4, "%s(): interrogating low-level PCMCIA service\n",
		      __FUNCTION__);

		ret = pcmcia_low_level->socket_state(&state_array);
		if (ret < 0) {
			printk(KERN_ERR
			       "mx21_pcmcia: unable to read socket status\n");
			break;
		}

		all_events = 0;

		for (i = 0; i < state_array.size; i++, all_events |= events) {
			struct mx21_pcmcia_socket *skt = PCMCIA_SOCKET(i);

			events = mx21_pcmcia_events(&state[i], &skt->k_state,
						    skt->cs_state.csc_mask,
						    skt->cs_state.flags);

			if (events && mx21_pcmcia_socket[i].handler != NULL)
				skt->handler(skt->handler_info, events);
		}
	} while (all_events);

	DEBUG(4, "%s(): exiting\n", __FUNCTION__);

}				/* mx21_pcmcia_task_handler() */

static struct tq_struct mx21_pcmcia_task = {
      routine:mx21_pcmcia_task_handler
};

/* mx21_pcmcia_poll_event()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^
 * Let's poll for events in addition to IRQs since IRQ only is unreliable...
 */
static void
mx21_pcmcia_poll_event(unsigned long dummy)
{
	DEBUG(4, "%s(): polling for events\n", __FUNCTION__);
	poll_timer.function = mx21_pcmcia_poll_event;
	poll_timer.expires = jiffies + MX21_PCMCIA_POLL_PERIOD;
	add_timer(&poll_timer);
	schedule_task(&mx21_pcmcia_task);
}

/* mx21_pcmcia_interrupt()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^
 * Service routine for socket driver interrupts (requested by the
 * low-level PCMCIA init() operation via mx21_pcmcia_thread()).
 * The actual interrupt-servicing work is performed by
 * mx21_pcmcia_thread(), largely because the Card Services event-
 * handling code performs scheduling operations which cannot be
 * executed from within an interrupt context.
 */
static void
mx21_pcmcia_interrupt(int irq, void *dev, struct pt_regs *regs)
{
	int i;

	DEBUG(5, "%s(): servicing IRQ %d\n", __FUNCTION__, irq);

	i = PCMCIA_PSCR;

	DEBUG(5, "%s(): PSCR=0x%x\n", __FUNCTION__, i);

	PCMCIA_PSCR = i;

	i = PCMCIA_PGSR;

	DEBUG(5, "%s(): PGSR=0x%x\n", __FUNCTION__, i);

	PCMCIA_PGSR = i;

	schedule_task(&mx21_pcmcia_task);
}

/* mx21_pcmcia_register_callback()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 * Implements the register_callback() operation for the in-kernel
 * PCMCIA service (formerly SS_RegisterCallback in Card Services). If 
 * the function pointer `handler' is not NULL, remember the callback 
 * location in the state for `sock', and increment the usage counter 
 * for the driver module. (The callback is invoked from the interrupt
 * service routine, mx21_pcmcia_interrupt(), to notify Card Services
 * of interesting events.) Otherwise, clear the callback pointer in the
 * socket state and decrement the module usage count.
 *
 * Returns: 0
 */
static int
mx21_pcmcia_register_callback(unsigned int sock,
			      void (*handler) (void *, unsigned int),
			      void *info)
{
	struct mx21_pcmcia_socket *skt = PCMCIA_SOCKET(sock);

	if (handler == NULL) {
		skt->handler = NULL;
		MOD_DEC_USE_COUNT;
	} else {
		MOD_INC_USE_COUNT;
		skt->handler_info = info;
		skt->handler = handler;
	}

	return 0;
}

/* mx21_pcmcia_inquire_socket()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 * Implements the inquire_socket() operation for the in-kernel PCMCIA
 * service (formerly SS_InquireSocket in Card Services). Of note is
 * the setting of the SS_CAP_PAGE_REGS bit in the `features' field of
 * `cap' to "trick" Card Services into tolerating large "I/O memory" 
 * addresses. Also set is SS_CAP_STATIC_MAP, which disables the memory
 * resource database check. (Mapped memory is set up within the socket
 * driver itself.)
 *
 * In conjunction with the STATIC_MAP capability is a new field,
 * `io_offset', recommended by David Hinds. Rather than go through
 * the SetIOMap interface (which is not quite suited for communicating
 * window locations up from the socket driver), we just pass up
 * an offset which is applied to client-requested base I/O addresses
 * in alloc_io_space().
 *
 * SS_CAP_PAGE_REGS: used by setup_cis_mem() in cistpl.c to set the
 *   force_low argument to validate_mem() in rsrc_mgr.c -- since in
 *   general, the mapped * addresses of the PCMCIA memory regions
 *   will not be within 0xffff, setting force_low would be
 *   undesirable.
 *
 * SS_CAP_STATIC_MAP: don't bother with the (user-configured) memory
 *   resource database; we instead pass up physical address ranges
 *   and allow other parts of Card Services to deal with remapping.
 *
 * SS_CAP_PCCARD: we can deal with 16-bit PCMCIA & CF cards, but
 *   not 32-bit CardBus devices.
 *
 * Return value is irrelevant; the pcmcia subsystem ignores it.
 */
static int
mx21_pcmcia_inquire_socket(unsigned int sock, socket_cap_t * cap)
{
	struct mx21_pcmcia_socket *skt = PCMCIA_SOCKET(sock);
	int ret = -1;
	pccard_io_map io = { 0, 0, 0, 0, 1 };

	DEBUG(2, "%s() for sock %d\n", __FUNCTION__, sock);

	if (sock < mx21_pcmcia_socket_count) {
		cap->features = SS_CAP_PAGE_REGS |	/* SS_CAP_MEM_ALIGN | */
		    SS_CAP_STATIC_MAP | SS_CAP_PCCARD;
		cap->irq_mask = 0;
		cap->map_size = PCMCIA_IOMEM_PARTITION_SIZE;
		cap->pci_irq = skt->irq;
		cap->io_offset = (unsigned long) skt->virt_io;

		ret = 0;
	}

	mx21_pcmcia_set_io_map(sock, &io);

	return ret;
}

/* mx21_pcmcia_get_status()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^
 * Implements the get_status() operation for the in-kernel PCMCIA
 * service (formerly SS_GetStatus in Card Services). Essentially just
 * fills in bits in `status' according to internal driver state or
 * the value of the voltage detect chipselect register.
 *
 * As a debugging note, during card startup, the PCMCIA core issues
 * three set_socket() commands in a row the first with RESET deasserted,
 * the second with RESET asserted, and the last with RESET deasserted
 * again. Following the third set_socket(), a get_status() command will
 * be issued. The kernel is looking for the SS_READY flag (see
 * setup_socket(), reset_socket(), and unreset_socket() in cs.c).
 *
 * Returns: 0
 */
static int
mx21_pcmcia_get_status(unsigned int sock, unsigned int *status)
{
	struct mx21_pcmcia_socket *skt = PCMCIA_SOCKET(sock);
	struct pcmcia_state state[MX21_PCMCIA_MAX_SOCK];
	struct pcmcia_state_array state_array;
	unsigned int stat;

	DEBUG(2, "%s() for sock %d\n", __FUNCTION__, sock);

	state_array.size = mx21_pcmcia_socket_count;
	state_array.state = state;

	memset(state, 0, sizeof (state));

	if ((pcmcia_low_level->socket_state(&state_array)) < 0) {
		printk(KERN_ERR "mx21_pcmcia: unable to get socket status\n");
		return -1;
	}

	skt->k_state = state[sock];

	stat = state[sock].detect ? SS_DETECT : 0;
	stat |= state[sock].ready ? SS_READY : 0;
	stat |= state[sock].vs_3v ? SS_3VCARD : 0;
	stat |= state[sock].vs_Xv ? SS_XVCARD : 0;

	/* The power status of individual sockets is not available
	 * explicitly from the hardware, so we just remember the state
	 * and regurgitate it upon request:
	 */
	stat |= skt->cs_state.Vcc ? SS_POWERON : 0;

	if (skt->cs_state.flags & SS_IOCARD)
		stat |= state[sock].bvd1 ? SS_STSCHG : 0;
	else {
		if (state[sock].bvd1 == 0)
			stat |= SS_BATDEAD;
		else if (state[sock].bvd2 == 0)
			stat |= SS_BATWARN;
	}

	DEBUG(3, "    status: %s%s%s%s%s%s%s%s\n",
	      stat & SS_DETECT ? "DETECT " : "",
	      stat & SS_READY ? "READY " : "",
	      stat & SS_BATDEAD ? "BATDEAD " : "",
	      stat & SS_BATWARN ? "BATWARN " : "",
	      stat & SS_POWERON ? "POWERON " : "",
	      stat & SS_STSCHG ? "STSCHG " : "",
	      stat & SS_3VCARD ? "3VCARD " : "",
	      stat & SS_XVCARD ? "XVCARD " : "");

	*status = stat;

	return 0;
}				/* mx21_pcmcia_get_status() */

/* mx21_pcmcia_get_socket()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^
 * Implements the get_socket() operation for the in-kernel PCMCIA
 * service (formerly SS_GetSocket in Card Services). Not a very 
 * exciting routine.
 *
 * Returns: 0
 */
static int
mx21_pcmcia_get_socket(unsigned int sock, socket_state_t * state)
{
	struct mx21_pcmcia_socket *skt = PCMCIA_SOCKET(sock);

	DEBUG(2, "%s() for sock %d\n", __FUNCTION__, sock);

	*state = skt->cs_state;
	DEBUG(3, "\nGetSocket:vcc=%d,vpp=%d,io_req=%d,\n", state->Vcc,
	      state->Vpp, state->io_irq);
	return 0;
}

/* mx21_pcmcia_set_socket()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^
 * Implements the set_socket() operation for the in-kernel PCMCIA
 * service (formerly SS_SetSocket in Card Services). We more or
 * less punt all of this work and let the kernel handle the details
 * of power configuration, reset, &c. We also record the value of
 * `state' in order to regurgitate it to the PCMCIA core later.
 *
 * Returns: 0
 */
static int
mx21_pcmcia_set_socket(unsigned int sock, socket_state_t * state)
{
	struct mx21_pcmcia_socket *skt = PCMCIA_SOCKET(sock);
	struct pcmcia_configure conf;

	DEBUG(2, "%s(): for sock %d\n", __FUNCTION__, sock);

	DEBUG(3, "   mask:  %s%s%s%s%s%s\n",
	      (state->csc_mask == 0) ? "<NONE>" : "",
	      (state->csc_mask & SS_DETECT) ? "DETECT " : "",
	      (state->csc_mask & SS_READY) ? "READY " : "",
	      (state->csc_mask & SS_BATDEAD) ? "BATDEAD " : "",
	      (state->csc_mask & SS_BATWARN) ? "BATWARN " : "",
	      (state->csc_mask & SS_STSCHG) ? "STSCHG " : "");
	DEBUG(3, "    flags: %s%s%s%s%s%s\n",
	      (state->flags == 0) ? "<NONE>" : "",
	      (state->flags & SS_PWR_AUTO) ? "PWR_AUTO " : "",
	      (state->flags & SS_IOCARD) ? "IOCARD " : "",
	      (state->flags & SS_RESET) ? "RESET " : "",
	      (state->flags & SS_SPKR_ENA) ? "SPKR_ENA " : "",
	      (state->flags & SS_OUTPUT_ENA) ? "OUTPUT_ENA " : "");
	DEBUG(3, "Vcc %d  Vpp %d  irq %d\n",
	      state->Vcc, state->Vpp, state->io_irq);
	DEBUG(3, "SetSocket:vcc=%d,vpp=%d,io_req=%d,\n", state->Vcc,
	      state->Vpp, state->io_irq);
	conf = mx21_pcmcia_state_to_config(sock, state);

	if (pcmcia_low_level->configure_socket(&conf) < 0) {
		printk(KERN_ERR "mx21_pcmcia: unable to configure socket %d\n",
		       sock);
		return -1;
	}

	skt->cs_state = *state;

	return 0;
}				/* mx21_pcmcia_set_socket() */

/* mx21_pcmcia_get_io_map()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^
 * Implements the get_io_map() operation for the in-kernel PCMCIA
 * service (formerly SS_GetIOMap in Card Services). Just returns an
 * I/O map descriptor which was assigned earlier by a set_io_map().
 *
 * Returns: 0 on success, -1 if the map index was out of range
 */
static int
mx21_pcmcia_get_io_map(unsigned int sock, struct pccard_io_map *map)
{
	struct mx21_pcmcia_socket *skt = PCMCIA_SOCKET(sock);
	int ret = -1;

	DEBUG(2, "%s() for sock %d\n", __FUNCTION__, sock);

	if (map->map < MAX_IO_WIN) {
		*map = skt->io_map[map->map];
		ret = 0;
	}

	return ret;
}

/* mx21_pcmcia_set_io_map()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^
 * Implements the set_io_map() operation for the in-kernel PCMCIA
 * service (formerly SS_SetIOMap in Card Services). We configure
 * the map speed as requested, but override the address ranges
 * supplied by Card Services.
 *
 * Returns: 0 on success, -1 on error
 */
static int
mx21_pcmcia_set_io_map(unsigned int sock, struct pccard_io_map *map)
{
	struct mx21_pcmcia_socket *skt = PCMCIA_SOCKET(sock);
	unsigned int hclk_ns, hclk_cycles;
	
	DEBUG(2, "%s(): for sock %d\n", __FUNCTION__, sock);

	DEBUG(3, "     flags: %s%s%s%s%s%s%s%s\n",
	      (map->flags == 0) ? "<NONE>" : "",
	      (map->flags & MAP_ACTIVE) ? "ACTIVE " : "",
	      (map->flags & MAP_16BIT) ? "16BIT " : "",
	      (map->flags & MAP_AUTOSZ) ? "AUTOSZ " : "",
	      (map->flags & MAP_0WS) ? "0WS " : "",
	      (map->flags & MAP_WRPROT) ? "WRPROT " : "",
	      (map->flags & MAP_USE_WAIT) ? "USE_WAIT " : "",
	      (map->flags & MAP_PREFETCH) ? "PREFETCH " : "");

	if (map->map >= MAX_IO_WIN) {
		printk(KERN_ERR "%s(): map (%d) out of range\n", __FUNCTION__,
		       map->map);
		return -1;
	}

	if (map->flags & MAP_ACTIVE) {
		unsigned int speed = map->speed;

		if (speed == 0)
			speed = MX21_PCMCIA_IO_ACCESS;

		skt->speed_io = speed;
	}

	if (map->stop == 1)
		map->stop = PCMCIA_IOMEM_PARTITION_SIZE - 1;

	skt->io_map[map->map] = *map;

	por_write(2, POR_PV, 0);

	pbr_write(2, PBR_PBA, _PCMCIAIO(0) & 0x7ff);

#define IO_PORT 3
	por_write(2, POR_PRS, IO_PORT);

	por_write(2, POR_BSIZE, 0xf);	/* 1Kb */
	
	/* 
	 * i.MX21 Reference Manual says:
	 *  - PCMCIA Strobe Length (PSL) determines the number of the cycles 
	 * the strobe is asserted during a PCMCIA access for this window and, 
	 * thus,it is the main parameter for determining cycle length. The cycle
	 * may be lengthened by asserting #WAIT.
	 * Note: To sample #WAIT signal the PSL should be calculated as the 
	 * maximum valid time on WAIT plus two.
	 *
	 * I believe that PSL corresponds to taOE/twWE PC Card Standard's 
	 * timing symbols 
	 *
	 * - PCMCIA Strobe Setup Time (PSST) specifies when #IOWR and #WE are
	 * asserted during a PCMCIA write access or when #IORD or #OE are 
	 * asserted during a PCMCIA read access handled by PCMCIA/CF controller.
	 *
	 * I believe (comparing with Figure 36-4 and 36-5 of Reference Manual) 
	 * that PSST corresponds to tsuA (Address Setup Time) of PC Card 
	 * Standard's timing symbols 
	 * 
	 * - PCMCIA Strobe Hold Time (PSHT) specifies when #IOWR or #WE are
	 * negated during a PCMCIA write access or when #IORD or #OE are 
	 * negated during a PCMCIA read access handled by PCMCIA/CF controller.
	 *
	 * I believe (comparing with Figure 36-4 and 36-5 of Reference Manual) 
	 * that PSHT corresponds to thA (Address Hold Time) of PC Card 
	 * Standard's timing symbols 
	 * 
	 */

	hclk_ns = 1000 / (mx_module_get_clk(HCLK) / 1000000); /* in nanosec */
	hclk_cycles = (skt->speed_io / hclk_ns + 2);
	
	if (hclk_cycles > 128) {
		printk(KERN_ERR "mx21_pcmcia_set_io_map: Unable to set "
	               "requested I/O speed (PSL) %d\n", hclk_cycles);
		return -1;
	}
	
	por_write(2, POR_PSL, hclk_cycles % 128);

	hclk_cycles = MX21_PCMCIA_IO_ADDR_SETUP_TIME / hclk_ns;

	if (hclk_cycles > 63) {
		printk(KERN_ERR "mx21_pcmcia_set_io_map: Unable to set "
	               "requested I/O speed (PSST) %d\n", hclk_cycles);
		return -1;
	}
	
	por_write(2, POR_PSST, (hclk_cycles > 1) ? hclk_cycles : 2);

	hclk_cycles = MX21_PCMCIA_IO_ADDR_HOLD_TIME / hclk_ns;

	if (hclk_cycles > 63) {
		printk(KERN_ERR "mx21_pcmcia_set_io_map: Unable to set "
	               "requested I/O speed (PSHT) %d\n", hclk_cycles);
		return -1;
	}

	por_write(2, POR_PSHT, hclk_cycles);

	por_write(2, POR_PPS, 0);
	pofr_write(2, POFR_POFA, 0);
	por_write(2, POR_PV, 1);

	DEBUG(4, "mx21_pcmcia_set_io_map: index=%d,pbr=0x%x,por=0x%x,pofr=0x%x.\n ", 2,
	      PCMCIA_PBR(2), PCMCIA_POR(2), PCMCIA_POFR(2));

	return 0;
}

/* mx21_pcmcia_get_mem_map()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^
 * Implements the get_mem_map() operation for the in-kernel PCMCIA
 * service (formerly SS_GetMemMap in Card Services). Just returns a
 *  memory map descriptor which was assigned earlier by a
 *  set_mem_map() request.
 *
 * Returns: 0 on success, -1 if the map index was out of range
 */
static int
mx21_pcmcia_get_mem_map(unsigned int sock, struct pccard_mem_map *map)
{
	struct mx21_pcmcia_socket *skt = PCMCIA_SOCKET(sock);
	int ret = -1;

	DEBUG(2, "%s() for sock %d\n", __FUNCTION__, sock);

	if (map->map < MAX_WIN) {
		*map = skt->mem_map[map->map];
		ret = 0;
	}

	return ret;
}

/* mx21_pcmcia_set_mem_map()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^
 * Implements the set_mem_map() operation for the in-kernel PCMCIA
 * service (formerly SS_SetMemMap in Card Services). We configure
 * the map speed as requested, but override the address ranges
 * supplied by Card Services.
 *
 * Returns: 0 on success, -1 on error
 */
static int
mx21_pcmcia_set_mem_map(unsigned int sock, struct pccard_mem_map *map)
{
	struct mx21_pcmcia_socket *skt = PCMCIA_SOCKET(sock);
	unsigned long start;
	int index;
	unsigned int hclk_ns, hclk_cycles;
	unsigned long bsize = ~0;	/* Initialize to non-valid value */

	DEBUG(3, "mx21_pcmcia_set_mem_map: entered socket#=%d\n", sock);

	DEBUG(3, "%s():	flags: %s%s%s%s%s%s%s%s\n", __FUNCTION__,
	      (map->flags == 0) ? "<NONE>" : "",
	      (map->flags & MAP_ACTIVE) ? "ACTIVE " : "",
	      (map->flags & MAP_16BIT) ? "16BIT " : "",
	      (map->flags & MAP_AUTOSZ) ? "AUTOSZ " : "",
	      (map->flags & MAP_0WS) ? "0WS " : "",
	      (map->flags & MAP_WRPROT) ? "WRPROT " : "",
	      (map->flags & MAP_ATTRIB) ? "ATTRIB " : "",
	      (map->flags & MAP_USE_WAIT) ? "USE_WAIT " : "");

	if (map->map >= MAX_WIN) {
		printk(KERN_ERR "\n%s(): map (%d) out of range\n", __FUNCTION__,
		       map->map);
		return -1;
	}

	if (map->flags & MAP_ACTIVE) {
		unsigned int speed = map->speed;
		/*
		 * When clients issue RequestMap, the access speed is not always
		 * properly configured.  Choose some sensible defaults.
		 */
		if (speed == 0) {
			if (skt->cs_state.Vcc == 33)
				speed = MX21_PCMCIA_3V_MEM_ACCESS;
			else
				speed = MX21_PCMCIA_5V_MEM_ACCESS;
		}

		if (map->flags & MAP_ATTRIB) {
			skt->speed_attr = speed;
		} else {
			skt->speed_mem = speed;
		}
	}

	start = (map->flags & MAP_ATTRIB) ? skt->phys_attr : skt->phys_mem;
	index = (map->flags & MAP_ATTRIB) ? 1 : 0;

	if (map->sys_stop == 0)
		map->sys_stop = PCMCIA_IOMEM_PARTITION_SIZE - 1;

	map->sys_stop -= map->sys_start;
	map->sys_stop += start;
	map->sys_start = start;

	skt->mem_map[map->map] = *map;

	DEBUG(3, "mx21_pcmcia_set_mem_map: skt->phys_attr=0x%x skt->phys_mem=0x%x\n",
	       skt->phys_attr, skt->phys_mem);

	DEBUG(3, "mx21_pcmcia_set_mem_map: index=%d start 0x%08x map->sys_start=0x%x map->sys_stop0x%x\n",
	       index, start, map->sys_start, map->sys_stop);

	/* 
	 * Assumption: Window 0 is for Common Memory, 
	 * Window 1 is for Attr. Memory.
	 */

	por_write(index, POR_PV, 0);
	pbr_write(index, PBR_PBA, start);

	por_write(index, POR_WPEN, 0);
	por_write(index, POR_WP, 0);

#define COMMON_MEMORY 0
#define ATTRIBUTE_MEMORY 2
	por_write(index, POR_PRS,
		  (index == 0) ? COMMON_MEMORY : ATTRIBUTE_MEMORY);

	switch (map->sys_stop - start + 1) {
	case 1:
		bsize = 0;
		break;		/* 1 byte */
	case 2:
		bsize = 1;
		break;
	case 4:
		bsize = 3;
		break;
	case 8:
		bsize = 2;
		break;
	case 0x10:
		bsize = 0x6;
		break;		/* 16 bytes */
	case 0x20:
		bsize = 0x7;
		break;
	case 0x40:
		bsize = 0x5;
		break;
	case 0x80:
		bsize = 0x4;
		break;
	case 0x100:
		bsize = 0xc;
		break;		/* 256 bytes */
	case 0x200:
		bsize = 0xd;
		break;
	case 0x400:
		bsize = 0xf;
		break;
	case 0x800:
		bsize = 0xe;
		break;
#ifdef CONFIG_MX2TO1
	case 0x1000:
		bsize = 0xa;
		break;		/* 4Kb */
	case 0x2000:
		bsize = 0xb;
		break;
	case 0x4000:
		bsize = 0x9;
		break;
	case 0x8000:
		bsize = 0x8;
		break;
	case 0x10000:
		bsize = 0x18;
		break;		/* 64Kb */
	case 0x20000:
		bsize = 0x19;
		break;
	case 0x40000:
		bsize = 0x1b;
		break;
	case 0x80000:
		bsize = 0x1a;
		break;
	case 0x100000:
		bsize = 0x1e;
		break;		/* 1Mb */
	case 0x200000:
		bsize = 0x1f;
		break;
	case 0x400000:
		bsize = 0x1d;
		break;
	case 0x800000:
		bsize = 0x1c;
		break;
	case 0x1000000:
		bsize = 0x14;
		break;		/* 16Mb */
	case 0x2000000:
		bsize = 0x15;
		break;
	case 0x4000000:
		bsize = 0x17;
		break;
#endif
	default:
		printk("bsize error.\n");
		break;
	};
	por_write(index, POR_BSIZE, bsize);

	/* 
	 * i.MX21 Reference Manual says:
	 *  - PCMCIA Strobe Length (PSL) determines the number of the cycles 
	 * the strobe is asserted during a PCMCIA access for this window and, 
	 * thus,it is the main parameter for determining cycle length. The cycle
	 * may be lengthened by asserting #WAIT.
	 * Note: To sample #WAIT signal the PSL should be calculated as the 
	 * maximum valid time on WAIT plus two.
	 *
	 * I believe that PSL corresponds to taOE/twWE PC Card Standard's 
	 * timing symbols 
	 *
	 * - PCMCIA Strobe Setup Time (PSST) specifies when #IOWR and #WE are
	 * asserted during a PCMCIA write access or when #IORD or #OE are 
	 * asserted during a PCMCIA read access handled by PCMCIA/CF controller.
	 *
	 * I believe (comparing with Figure 36-4 and 36-5 of Reference Manual) 
	 * that PSST corresponds to tsuA (Address Setup Time) of PC Card 
	 * Standard's timing symbols 
	 * 
	 * - PCMCIA Strobe Hold Time (PSHT) specifies when #IOWR or #WE are
	 * negated during a PCMCIA write access or when #IORD or #OE are 
	 * negated during a PCMCIA read access handled by PCMCIA/CF controller.
	 *
	 * I believe (comparing with Figure 36-4 and 36-5 of Reference Manual) 
	 * that PSHT corresponds to thA (Address Hold Time) of PC Card 
	 * Standard's timing symbols 
	 * 
	 */

	hclk_ns = 1000 / (mx_module_get_clk(HCLK) / 1000000); /* in nanosec */

	if (index == 0) {
		/* Common Memory */

		hclk_cycles = (skt->speed_mem / hclk_ns + 2);
	
		if (hclk_cycles > 128) {
			printk(KERN_ERR "mx21_pcmcia_set_mem_map: Unable to set "
		                "requested Common Memory speed (PSL) %d\n", 
				hclk_cycles);
		return -1;
		}
	
		por_write(index, POR_PSL, hclk_cycles % 128);

		if (skt->cs_state.Vcc == 33)
			hclk_cycles = MX21_PCMCIA_3V_MEM_ADDR_SETUP_TIME / 
					hclk_ns;
		else
			hclk_cycles = MX21_PCMCIA_5V_MEM_ADDR_SETUP_TIME / 
					hclk_ns;
		
		if (hclk_cycles > 63) {
			printk(KERN_ERR "mx21_pcmcia_set_mem_map: Unable to set "
	        	       "requested Common Memory speed (PSST) %d\n", 
				hclk_cycles);
			return -1;
		}
	
		por_write(index, POR_PSST, 
				(hclk_cycles > 1) ? hclk_cycles : 2);

		if (skt->cs_state.Vcc == 33)
			hclk_cycles = MX21_PCMCIA_3V_MEM_ADDR_HOLD_TIME / 
					hclk_ns;
		else
			hclk_cycles = MX21_PCMCIA_5V_MEM_ADDR_HOLD_TIME / 
					hclk_ns;

		if (hclk_cycles > 63) {
			printk(KERN_ERR "mx21_pcmcia_set_mem_map: Unable to set "
	               		"requested Common Memory speed (PSHT) %d\n", 
				hclk_cycles);
			return -1;
		}

		por_write(index, POR_PSHT, hclk_cycles);

		por_write(index, POR_PPS, 0);

	} else {
		/* Attribute Memory */

		hclk_cycles = (skt->speed_attr / hclk_ns + 2);
	
		if (hclk_cycles > 128) {
			printk(KERN_ERR "mx21_pcmcia_set_mem_map: Unable to set "
		                "requested Attr Memory speed (PSL) %d\n", 
				hclk_cycles);
		return -1;
		}
	
		por_write(index, POR_PSL, hclk_cycles % 128);

		if (skt->cs_state.Vcc == 33)
			hclk_cycles = MX21_PCMCIA_3V_MEM_ADDR_SETUP_TIME / 
					hclk_ns;
		else
			hclk_cycles = MX21_PCMCIA_5V_MEM_ADDR_SETUP_TIME / 
					hclk_ns;
		
		if (hclk_cycles > 63) {
			printk(KERN_ERR "mx21_pcmcia_set_mem_map: Unable to set "
	        	       "requested Attr Memory speed (PSST) %d\n", 
				hclk_cycles);
			return -1;
		}
	
		por_write(index, POR_PSST, 
				(hclk_cycles > 1) ? hclk_cycles : 2);

		if (skt->cs_state.Vcc == 33)
			hclk_cycles = MX21_PCMCIA_3V_MEM_ADDR_HOLD_TIME / 
					hclk_ns;
		else
			hclk_cycles = MX21_PCMCIA_5V_MEM_ADDR_HOLD_TIME / 
					hclk_ns;

		if (hclk_cycles > 63) {
			printk(KERN_ERR "mx21_pcmcia_set_mem_map: Unable to set "
	               		"requested Attr Memory speed (PSHT) %d\n", 
				hclk_cycles);
			return -1;
		}

		por_write(index, POR_PSHT, hclk_cycles);

		por_write(index, POR_PPS, 0);

	}

	pofr_write(index, POFR_POFA, 0);
	por_write(index, POR_PV, 1);
	DEBUG(4, "%s() for sock %d\n", __FUNCTION__, sock);

	DEBUG(4, "index=%d,pbr=0x%x,por=0x%x,pofr=0x%x.\n ", index,
	      PCMCIA_PBR(index), PCMCIA_POR(index), PCMCIA_POFR(index));

	return 0;
}

#if defined(CONFIG_PROC_FS)

/* mx21_pcmcia_proc_status()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^
 * Implements the /proc/bus/pccard/??/status file.
 *
 * Returns: the number of characters added to the buffer
 */
static int
mx21_pcmcia_proc_status(char *buf, char **start, off_t pos,
			int count, int *eof, void *data)
{
	unsigned int sock = (unsigned int) data;
	struct mx21_pcmcia_socket *skt = PCMCIA_SOCKET(sock);
	char *p = buf;

	p += sprintf(p, "k_state  : %s%s%s%s%s%s%s\n",
		     skt->k_state.detect ? "detect " : "",
		     skt->k_state.ready ? "ready " : "",
		     skt->k_state.bvd1 ? "bvd1 " : "",
		     skt->k_state.bvd2 ? "bvd2 " : "",
		     skt->k_state.wrprot ? "wrprot " : "",
		     skt->k_state.vs_3v ? "vs_3v " : "",
		     skt->k_state.vs_Xv ? "vs_Xv " : "");

	p += sprintf(p, "status   : %s%s%s%s%s%s%s%s%s\n",
		     skt->k_state.detect ? "SS_DETECT " : "",
		     skt->k_state.ready ? "SS_READY " : "",
		     skt->cs_state.Vcc ? "SS_POWERON " : "",
		     skt->cs_state.flags & SS_IOCARD ? "SS_IOCARD " : "",
		     (skt->cs_state.flags & SS_IOCARD &&
		      skt->k_state.bvd1) ? "SS_STSCHG " : "",
		     ((skt->cs_state.flags & SS_IOCARD) == 0 &&
		      (skt->k_state.bvd1 == 0)) ? "SS_BATDEAD " : "",
		     ((skt->cs_state.flags & SS_IOCARD) == 0 &&
		      (skt->k_state.bvd2 == 0)) ? "SS_BATWARN " : "",
		     skt->k_state.vs_3v ? "SS_3VCARD " : "",
		     skt->k_state.vs_Xv ? "SS_XVCARD " : "");

	p += sprintf(p, "mask     : %s%s%s%s%s\n",
		     skt->cs_state.csc_mask & SS_DETECT ? "SS_DETECT " : "",
		     skt->cs_state.csc_mask & SS_READY ? "SS_READY " : "",
		     skt->cs_state.csc_mask & SS_BATDEAD ? "SS_BATDEAD " : "",
		     skt->cs_state.csc_mask & SS_BATWARN ? "SS_BATWARN " : "",
		     skt->cs_state.csc_mask & SS_STSCHG ? "SS_STSCHG " : "");

	p += sprintf(p, "cs_flags : %s%s%s%s%s\n",
		     skt->cs_state.flags & SS_PWR_AUTO ? "SS_PWR_AUTO " : "",
		     skt->cs_state.flags & SS_IOCARD ? "SS_IOCARD " : "",
		     skt->cs_state.flags & SS_RESET ? "SS_RESET " : "",
		     skt->cs_state.flags & SS_SPKR_ENA ? "SS_SPKR_ENA " : "",
		     skt->cs_state.
		     flags & SS_OUTPUT_ENA ? "SS_OUTPUT_ENA " : "");

	p += sprintf(p, "Vcc      : %d\n", skt->cs_state.Vcc);
	p += sprintf(p, "Vpp      : %d\n", skt->cs_state.Vpp);
	p += sprintf(p, "IRQ      : %d\n", skt->cs_state.io_irq);

	return p - buf;
}

/* mx21_pcmcia_proc_setup()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^
 * Implements the proc_setup() operation for the in-kernel PCMCIA
 * service (formerly SS_ProcSetup in Card Services).
 *
 * Returns: 0 on success, -1 on error
 */
static void
mx21_pcmcia_proc_setup(unsigned int sock, struct proc_dir_entry *base)
{
	struct proc_dir_entry *entry;

	DEBUG(4, "%s() for sock %d\n", __FUNCTION__, sock);

	if ((entry = create_proc_entry("status", 0, base)) == NULL) {
		printk(KERN_ERR "unable to install \"status\" procfs entry\n");
		return;
	}

	entry->read_proc = mx21_pcmcia_proc_status;
	entry->data = (void *) sock;
}
#endif				/* CONFIG_PROC_FS */

static struct pccard_operations mx21_pcmcia_operations = {
	.init = mx21_pcmcia_init,
	.suspend = mx21_pcmcia_suspend,
	.register_callback = mx21_pcmcia_register_callback,
	.inquire_socket = mx21_pcmcia_inquire_socket,
	.get_status = mx21_pcmcia_get_status,
	.get_socket = mx21_pcmcia_get_socket,
	.set_socket = mx21_pcmcia_set_socket,
	.get_io_map = mx21_pcmcia_get_io_map,
	.set_io_map = mx21_pcmcia_set_io_map,
	.get_mem_map = mx21_pcmcia_get_mem_map,
	.set_mem_map = mx21_pcmcia_set_mem_map,
#ifdef CONFIG_PROC_FS
	.proc_setup = mx21_pcmcia_proc_setup
#endif
};

static int __init
mx21_pcmcia_machine_init(void)
{

#ifdef CONFIG_ARCH_MX2ADS
	if (machine_is_mx2ads())
		pcmcia_low_level = &mx21ads_pcmcia_ops;
#endif

	if (!pcmcia_low_level) {
		printk(KERN_ERR
		       "This hardware is not supported by the MX21 Card Service driver\n");
		return -ENODEV;
	}

	return 0;
}

/* mx21_pcmcia_driver_init()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *
 * This routine performs a basic sanity check to ensure that this
 * kernel has been built with the appropriate board-specific low-level
 * PCMCIA support, performs low-level PCMCIA initialization, registers
 * this socket driver with Card Services, and then spawns the daemon
 * thread which is the real workhorse of the socket driver.
 *
 * Returns: 0 on success, -1 on error
 */
static int __init
mx21_pcmcia_driver_init(void)
{
	servinfo_t info;
	struct pcmcia_init pcmcia_init;
	struct pcmcia_state state[MX21_PCMCIA_MAX_SOCK];
	struct pcmcia_state_array state_array;
	unsigned int i;
	int ret;

	CardServices(GetCardServicesInfo, &info);

	if (info.Revision != CS_RELEASE_CODE) {
		printk(KERN_ERR "Card Services release codes do not match\n");
		return -EINVAL;
	}

	ret = mx21_pcmcia_machine_init();

	if (ret)
		return ret;

	pcmcia_init.handler = &mx21_pcmcia_interrupt;

	ret = pcmcia_low_level->init(&pcmcia_init);
	if (ret < 0) {
		printk(KERN_ERR
		       "Unable to initialize kernel PCMCIA service (%d).\n",
		       ret);
		return ret == -1 ? -EIO : ret;
	}

	mx21_pcmcia_socket_count = ret;
	state_array.size = mx21_pcmcia_socket_count;
	state_array.state = state;

	memset(state, 0, sizeof (state));

	if (pcmcia_low_level->socket_state(&state_array) < 0) {
		pcmcia_low_level->shutdown();
		printk(KERN_ERR "Unable to get PCMCIA status from kernel.\n");
		return -EIO;
	}

	for (i = 0; i < mx21_pcmcia_socket_count; i++) {
		struct mx21_pcmcia_socket *skt = PCMCIA_SOCKET(i);
		struct pcmcia_irq_info irq_info;

		if (!request_mem_region
		    (_PCMCIA(i), PCMCIA_IOMEM_SPACE, "pcmcia")) {
			ret = -EBUSY;
			goto out_err;
		}

		irq_info.sock = i;
		irq_info.irq = -1;
		ret = pcmcia_low_level->get_irq_info(&irq_info);
		if (ret < 0)
			printk(KERN_ERR
			       "Unable to get IRQ for socket %d (%d)\n", i,
			       ret);

		skt->irq = irq_info.irq;
		skt->k_state = state[i];
		skt->speed_io = MX21_PCMCIA_IO_ACCESS;
		skt->speed_attr = MX21_PCMCIA_3V_MEM_ACCESS;
		skt->speed_mem = MX21_PCMCIA_3V_MEM_ACCESS;
		skt->phys_attr = _PCMCIAAttr(i);
		skt->phys_mem = _PCMCIAMem(i);
		skt->virt_io = ioremap(_PCMCIAIO(i), PCMCIA_IOMEM_PARTITION_SIZE);

		DEBUG(3,"mx21_pcmcia_driver_init:Socket#%d "
		    "phys_attr=%p phys_mem=%p virt_io=%p\n", i,
		    skt->phys_attr, skt->phys_mem, skt->virt_io);

		if (skt->virt_io == NULL) {
			ret = -ENOMEM;
			goto out_err;
		}

	}

	/* Only advertise as many sockets as we can detect */
	ret = register_ss_entry(mx21_pcmcia_socket_count,
				&mx21_pcmcia_operations);
	if (ret < 0) {
		printk(KERN_ERR "Unable to register sockets\n");
		goto out_err;
	}

	/*
	 * Start the event poll timer.  It will reschedule by itself afterwards.
	 */
	mx21_pcmcia_poll_event(0);

	return 0;

      out_err:
	for (i = 0; i < mx21_pcmcia_socket_count; i++) {
		iounmap(mx21_pcmcia_socket[i].virt_io);
		release_mem_region(_PCMCIA(i), PCMCIA_IOMEM_SPACE);
	}

	pcmcia_low_level->shutdown();

	return ret;
}				/* mx21_pcmcia_driver_init() */

/* mx21_pcmcia_driver_shutdown()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 * Invokes the low-level kernel service to free IRQs associated with this
 * socket controller and reset GPIO edge detection.
 */
static void __exit
mx21_pcmcia_driver_shutdown(void)
{
	int i;

	del_timer_sync(&poll_timer);

	unregister_ss_entry(&mx21_pcmcia_operations);

	for (i = 0; i < mx21_pcmcia_socket_count; i++) {
		iounmap(mx21_pcmcia_socket[i].virt_io);
		release_mem_region(_PCMCIA(i), PCMCIA_IOMEM_SPACE);
	}

	pcmcia_low_level->shutdown();

	flush_scheduled_tasks();
}

MODULE_DESCRIPTION
    ("Linux PCMCIA Card Services: Motorola MX21 Socket Controller");
MODULE_LICENSE("Dual MPL/GPL");

module_init(mx21_pcmcia_driver_init);
module_exit(mx21_pcmcia_driver_shutdown);
