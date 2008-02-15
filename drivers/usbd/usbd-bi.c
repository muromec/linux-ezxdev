/*
 * usbd/usbd-bi.c - USB Bus Interface Driver
 *
 *      Copyright (c) 2004 Belcarra
 *
 * Adapted from earlier work:
 *      Copyright (c) 2002, 2003 Belcarra
 *      Copyright (c) 2000, 2001, 2002 Lineo
 *      Copyright (c) 2001 Hewlett Packard
 *
 * By: 
 *      Stuart Lynne <sl@belcara.com>, 
 *      Tom Rushworth <tbr@belcara.com>, 
 *      Bruce Balden <balden@belcara.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/*
 * Copyright (C) 2004 - Motorola
 *
 * 2004-Dec-06 - Modified for EZXBASE By Zhao Liang <w20146@motorola.com>
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include <linux/init.h>
#include <linux/list.h>

#include <linux/proc_fs.h>

#include <linux/smp_lock.h>
#include <linux/ctype.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/interrupt.h>


//#include <linux/netdevice.h>
#include <linux/proc_fs.h>
#include <asm/hardware.h>
#include <asm/hardware.h>
#include <asm/uaccess.h>


#include "usbd-chap9.h"
#include "usbd-mem.h"
#include "usbd.h"
#include "usbd-bus.h"
#include "usbd-func.h"
#include "trace.h"
#include "usbd-bi.h"
#include "usbd-admin.h"

int trace_init (void);
void trace_exit (void);



/* Module Parameters ************************************************************************* */



#ifdef MODULE

// override serial number
static char *serial_number_str;
MODULE_PARM (serial_number_str, "s");
MODULE_PARM_DESC (serial_number_str, "Serial Number");
#else 
static char *serial_number_str;
#endif

int bi_debug;

struct usb_bus_instance *usbd_bus;

/* globals */

int have_cable_irq;

#ifdef CONFIG_ARCH_EZX
struct timer_list reenum_timer;
void reenum_timeout(unsigned long data);
#endif

/* Bus Interface Callback Functions ********************************************************** */


/* bi_endpoint_halted - check if endpoint halted
 *
 * Used by the USB Device Core to check endpoint halt status.
 */
int bi_endpoint_halted (struct usb_bus_instance *bus, int endpoint_index)
{
#if 1
        // XXX TODO
        struct usb_endpoint_instance *endpoint;
	endpoint = bus->endpoint_array + (endpoint_index & (~USB_ENDPOINT_DIR_MASK));
        return endpoint->status;
#else
        return 0;
#endif
}


/* * bi_device_feature - handle set/clear feature requests
 *
 * Used by the USB Device Core to check endpoint halt status.
 */
int bi_device_feature (struct usb_bus_instance *bus, int endpoint_index, int flag)
{
#if 1
        // XXX TODO
        struct usb_endpoint_instance *endpoint;
        
	endpoint = bus->endpoint_array + (endpoint_index & (~USB_ENDPOINT_DIR_MASK));

        if (flag && !endpoint->status) {
                udc_stall_ep (endpoint->physical_endpoint);
                endpoint->status = 1;
        }
        else if (!flag && endpoint->status){
                udc_reset_ep (endpoint->physical_endpoint);
                endpoint->status = 0;
        }
#endif
        return 0;
}

/* bi_disable_endpoints - disable udc and all endpoints
 */
static void bi_disable_endpoints (struct usb_bus_instance *bus)
{
        int i;
        RETURN_IF (!bus || !bus->endpoint_array);
        for (i = 1; i < udc_max_endpoints (); i++) {
                struct usb_endpoint_instance *endpoint;
                CONTINUE_IF(!(endpoint = (bus->endpoint_array + i)));
                usbd_flush_endpoint(endpoint);
        }
}


/* bi_device_event_irq - handle generic bus event
 *
 * Called by usb core layer to inform bus of an event.
 */
int bi_device_event_irq (struct usb_bus_instance *bus, usb_device_event_t event, int data)
{
        int epn;
        int endpointsRequested = bus->function_instance->endpointsRequested;
        struct usb_endpoint_map *endpoint_map_array = bus->function_instance->endpoint_map_array;

        TRACE_MSG32("EVENT %x", event);
        switch (event) {
        case DEVICE_UNKNOWN:
                break;
        case DEVICE_INIT:
                TRACE_MSG("EVENT INIT");
                break;
        case DEVICE_CREATE:	
                TRACE_MSG("EVENT CREATE");
                bi_disable_endpoints (bus);
                udc_enable ();
                udc_suspended_interrupts ();
                break;

        case DEVICE_HUB_CONFIGURED:
                TRACE_MSG("EVENT HUB_CONFIGURED");
                udc_connect ();
                break;

        case DEVICE_RESET:
                TRACE_MSG("EVENT RESET");
                udc_set_address (0);
                udc_reset_ep (0);
                udc_suspended_interrupts ();
                bi_disable_endpoints (bus);
                break;

        case DEVICE_ADDRESS_ASSIGNED:
                TRACE_MSG("EVENT ADDRESSED");
                udc_set_address (data);
                udc_all_interrupts ();
                break;

        case DEVICE_CONFIGURED:
                TRACE_MSG("EVENT CONFIGURED");
                // iterate across the physical endpoint instance array to enable the endpoints
                for (epn = 1; epn < bus->endpoints; epn++) 
                        udc_setup_ep (epn, bus->endpoint_array + epn);
                return 0;

        case DEVICE_DE_CONFIGURED:
                TRACE_MSG("EVENT DE-CONFIGURED");
                break;

        case DEVICE_SET_INTERFACE:
                TRACE_MSG("EVENT SET INTERFACE");
                break;

        case DEVICE_SET_FEATURE:
                TRACE_MSG("EVENT SET FEATURE");
                break;

        case DEVICE_CLEAR_FEATURE:
                TRACE_MSG("EVENT CLEAR FEATURE");
                break;

        case DEVICE_BUS_INACTIVE:
                TRACE_MSG("EVENT INACTIVE");
                udc_suspended_interrupts (); // disable suspend interrupt
                if (!udc_attached ()) 
                        usbd_bus_event_irq (bus, DEVICE_RESET, 0);
                break;

        case DEVICE_BUS_ACTIVITY:
                TRACE_MSG("EVENT ACTIVITY");
                udc_all_interrupts (); // enable suspend interrupt
                break;

        case DEVICE_POWER_INTERRUPTION:
                TRACE_MSG("POWER INTERRUPTION");
                break;

        case DEVICE_HUB_RESET:
                TRACE_MSG("HUB RESET");
                break;

        case DEVICE_DESTROY:
                TRACE_MSG("DEVICE DESTROY");
                udc_disconnect ();
                bi_disable_endpoints (bus);
                udc_disable_interrupts ();
                udc_disable ();
                break;

        case DEVICE_CLOSE:
                break;
        }
        return 0;
}


/* bi_start_endpoint_in
 */
int bi_start_endpoint_in(struct usb_bus_instance *bus, struct usb_endpoint_instance *endpoint)
{
        unsigned long flags;

        //printk(KERN_INFO"%s:  bus: %p status: %d\n", __FUNCTION__, bus, bus->status);
        
        RETURN_ZERO_IF(!endpoint);
        udc_interrupts++;

        TRACE_MSG32("BI START ENDPOINT IN:  tx_urb: %x", (int)endpoint->tx_urb);
        local_irq_save (flags);
        // call udc_start_endpoint_in IFF we didn't previously have a tx urb 
        if (!endpoint->tx_urb && bi_tx_next_irq(endpoint)) {
                TRACE_MSG16("BI START ENDPOINT IN:  bEndpointAddress: %x actual_length: %d", 
                                (int)endpoint->bEndpointAddress, endpoint->tx_urb->actual_length);
                udc_start_endpoint_in(endpoint);
        }
        local_irq_restore (flags);
        //printk(KERN_INFO"%s: finis\n", __FUNCTION__);
        return 0;
}

/* bi_start_endpoint_out
 */
int bi_start_endpoint_out(struct usb_bus_instance *bus, struct usb_endpoint_instance *endpoint)
{
        unsigned long flags;

        //printk(KERN_INFO"%s:  bus: %p status: %d\n", __FUNCTION__, bus, bus->status);
        
        RETURN_ZERO_IF(!endpoint);
        udc_interrupts++;
        TRACE_MSG32("BI START ENDPOINT OUT:  rcv_urb: %x", (int)endpoint->rcv_urb);
        local_irq_save (flags);
        // call udc_start_endpoint_OUT IFF we didn't previously have a rcv urb 
        if (!endpoint->rcv_urb && bi_rcv_next_irq(endpoint)) {
                TRACE_MSG16("BI START ENDPOINT OUT:  bEndpointAddress: %x request_length: %d", 
                                (int)endpoint->bEndpointAddress, endpoint->rcv_urb->request_length);
                udc_start_endpoint_out(endpoint);
        }
        local_irq_restore (flags);
        //printk(KERN_INFO"%s: finis\n", __FUNCTION__);
        return 0;
}


/* bi_cancel_urb_irq - cancel sending an urb
 *
 * Used by the USB Device Core to cancel an urb.
 */
int bi_cancel_urb_irq (struct urb *urb)
{
        RETURN_EINVAL_IF (!urb);
        switch(urb->endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) {
        case USB_DIR_IN:
                //printk(KERN_INFO"%s: IN urb: %p\n", __FUNCTION__, urb);
                // is this the active urb?
                if (urb->endpoint->tx_urb == urb) {
                        urb->endpoint->tx_urb = NULL;
                        udc_cancel_in_irq(urb);
                }
                usbd_urb_sent_finished_irq (urb, SEND_FINISHED_CANCELLED);
                break;

        case USB_DIR_OUT:
                //printk(KERN_INFO"%s: OUT urb: %p\n", __FUNCTION__, urb);
                // is this the active urb?
                if (urb->endpoint->rcv_urb == urb) {
                        urb->endpoint->rcv_urb = NULL;
                        udc_cancel_out_irq(urb);
                }
                TRACE_MSG("CANCEL RECV URB");
                usbd_urb_recv_finished_irq (urb, RECV_CANCELLED);
                break;
        }
        return 0;
}

struct urb * bi_rcv_complete_irq (struct usb_endpoint_instance *endpoint, int len, int urb_bad)
{
        return _bi_rcv_complete_irq (endpoint, len, urb_bad);
}
struct urb * bi_tx_complete_irq (struct usb_endpoint_instance *endpoint, int restart)
{
        return _bi_tx_complete_irq (endpoint, restart);
}

/* Module Init - the device init and exit routines ******************************************* */

/* bi_udc_init_irq - initialize USB Device Controller
 * 
 * Get ready to use the USB Device Controller.
 *
 * Register an interrupt handler and IO region. Return non-zero for error.
 */
int bi_udc_init_irq (void)
{
	// request device IRQ, request device IO and probe for device
	THROW_IF (udc_request_udc_irq (), irq_err);
	THROW_IF (udc_request_io (), io_err); 
	THROW_IF (udc_init (), init_err);

        CATCH(irq_err) {
                printk(KERN_ERR"%s: could not request USB IRQ\n", __FUNCTION__);
                CATCH(io_err) {
                        CATCH(init_err) {
                                printk(KERN_ERR"%s: could not request USB IO space\n", __FUNCTION__);
                                udc_release_io ();
                        }
                        udc_release_udc_irq ();
                }
                printk(KERN_ERR"%s: error\n", __FUNCTION__);
                return -EINVAL;
        }
	return 0;
}


/* bi_udc_exit - Stop using the USB Device Controller
 *
 * Stop using the USB Device Controller.
 *
 * Shutdown and free dma channels, de-register the interrupt handler.
 */
void bi_udc_exit (struct usb_bus_instance *bus)
{
        udc_disable_ep (0);
        bi_disable_endpoints (bus);
	udc_disconnect ();
	udc_disable ();
	udc_release_io ();
	udc_release_udc_irq ();
}


/* Module Init - the module init and exit routines ************************************* */

/* module */
/* Bus Interface Admin operations ************************************************************ */

int bus_disable(struct usb_bus_instance *bus, char *arg);
int bus_enable(struct usb_bus_instance *bus, char *arg);

int bi_pm_off(struct usb_bus_instance *bus, char *arg)
{
        //printk(KERN_INFO"%s: \n", __FUNCTION__);
#ifdef CONFIG_ARCH_EZX
	// clear data to avoid schedule_timeout in pm off process - Aug 16, 2004 by w20146
	usbd_bus->device_bh.data = NULL;
#endif
        RETURN_EINVAL_IF(in_interrupt());
        return bus_disable(bus, arg);
}

int bi_pm_on(struct usb_bus_instance *bus, char *arg)
{
        //printk(KERN_INFO"%s:\n", __FUNCTION__);
        RETURN_EINVAL_IF(in_interrupt());
    //    return (udc_attached()) ? bus_enable(bus, arg) : 0;
        return bus_enable(bus, arg);  //added by Jordan to support USBC PM 2004/04/27
}

int bi_fix_serial_number_str(struct usb_bus_instance *bus)
{
        char *sp, *dp;
        for (sp = dp = usbd_bus->serial_number_str; sp && sp[0]; sp++) {
                CONTINUE_IF (!isxdigit(sp[0]));
                *dp++ = toupper(*sp);
        }
        if (dp) 
                *dp = '\0';
        return 0;
}

int bi_serial_number(struct usb_bus_instance *bus, char *arg)
{
        char *sp, *dp;
        RETURN_EINVAL_IF(in_interrupt());
        if (usbd_bus->serial_number_str) 
                lkfree(usbd_bus->serial_number_str);
        usbd_bus->serial_number_str = lstrdup(arg);
        bi_fix_serial_number_str(bus);
        return 0;
}


/* Bus Interface Operations ****************************************************************** */

int bi_attached(struct usb_bus_instance *bus)
{
        return (udc_attached());
}

int bi_connected(struct usb_bus_instance *bus)
{
        return (udc_connected());
}

int bi_disconnect(struct usb_bus_instance *bus, char *arg)
{
        RETURN_EINVAL_IF(in_interrupt());
        udc_disconnect();
        return 0;
}

int bi_connect(struct usb_bus_instance *bus, char *arg)
{
        RETURN_EINVAL_IF(in_interrupt());
        udc_connect();
        return 0;
}

/* Bus Interface Admin operations ************************************************************ */
/*
 * These are the operations used by the upper layers.
 *
 * N.B. None of these are optional.
 */
struct usb_bus_operations bi_ops = {
        bus_enable: bus_enable,
        bus_disable: bus_disable,
        bus_pm_off: bi_pm_off,
        bus_pm_on: bi_pm_on,
        bus_serial_number: bi_serial_number,
        start_endpoint_in: bi_start_endpoint_in,
        start_endpoint_out: bi_start_endpoint_out,
        cancel_urb_irq: bi_cancel_urb_irq,
        endpoint_halted: bi_endpoint_halted,
        device_feature: bi_device_feature,
        device_event: bi_device_event_irq,
        bus_disconnect: bi_disconnect,
        bus_connect: bi_connect,
        bus_attached: bi_attached,
        bus_connected: bi_connected,
        request_endpoints: udc_request_endpoints,
        set_endpoints: udc_set_endpoints,
};


struct usb_bus_driver bi_driver = {
	bops: &bi_ops,
};

/* Bus Interface Cable Events **************************************************************** */

void udc_cable_event_irq(void)
{
        (udc_attached() ?  usbd_enable_irq : usbd_disable_irq) (NULL);
}

void udc_cable_event(void)
{
        unsigned long flags;
        local_irq_save (flags);
        udc_cable_event_irq();
        local_irq_restore (flags);
}


/* Bus Interface ***************************************************************************** */

/* Prevent overlapp of bi administrative functions mainly:
 *      bus_enable
 *      bus_disable
 *      bi_modinit
 *      bi_modexit
 */ 
DECLARE_MUTEX(usbd_bi_sem);     

int bus_disable_sem(struct usb_bus_instance *bus, char *arg)
{
	struct bi_data *data;
        unsigned long flags;

        RETURN_ZERO_IF (usbd_bus_state_enabled != bus->bus_state);
        MOD_DEC_USE_COUNT;

        udc_disconnect ();
        udc_disable ();

        local_irq_save (flags);
        if (bus->device_state != STATE_ATTACHED) {
                usbd_bus_event_irq (bus, DEVICE_RESET, 0);
                usbd_bus_event_irq (bus, DEVICE_POWER_INTERRUPTION, 0);
                usbd_bus_event_irq (bus, DEVICE_HUB_RESET, 0);
        }
        usbd_bus_event_irq (bus, DEVICE_DESTROY, 0);
        bi_disable_endpoints(bus);
        bi_udc_exit(bus);
        local_irq_restore (flags);

        usbd_disable_function (bus);
        bus->bus_state = usbd_bus_state_disabled;
        return 0;
}

int bus_disable(struct usb_bus_instance *bus, char *arg)
{
        printk (KERN_INFO "%s: %s bus_state: %d\n", __FUNCTION__, arg ? arg : "(NULL)", bus->bus_state);
        RETURN_EINVAL_IF(in_interrupt());
        down(&usbd_bi_sem);
        bus_disable_sem(bus, arg);
        up(&usbd_bi_sem);
        printk(KERN_INFO"%s: finis\n", __FUNCTION__);
        return 0;
}

int bus_enable(struct usb_bus_instance *bus, char *arg)
{
        struct usb_endpoint_instance *endpoint;
        int rc = -EINVAL;
        unsigned long flags;

        //printk (KERN_INFO "%s: %s bus_state: %d\n", __FUNCTION__, arg ? arg : "(NULL)", bus->bus_state);
        RETURN_EINVAL_IF(in_interrupt());

        down(&usbd_bi_sem);
        bus_disable_sem(bus, arg);
        MOD_INC_USE_COUNT;

        local_irq_save (flags);
        do {
                // check if we can see the UDC and register, then enable the function
                BREAK_IF(bi_udc_init_irq ());
                BREAK_IF(usbd_enable_function_irq (bus, arg));
                rc = 0;

                // setup endpoint zero
                endpoint = bus->endpoint_array + 0;
                endpoint->bEndpointAddress = 0;

                //endpoint->tx_attributes = 0;
                endpoint->wMaxPacketSize = udc_ep0_packetsize ();
//                endpoint->rcv_transferSize = 255;       // XXX should this be higher
                endpoint->rcv_transferSize = 4096;       // Modified by Jordan to support long  transfer
                endpoint->wMaxPacketSize = udc_ep0_packetsize ();
                udc_setup_ep (0, endpoint);

                // hopefully device enumeration will finish this process
                udc_startup_events ();
                trace_reinit();
        } while(0);
        local_irq_restore (flags);
        up(&usbd_bi_sem);
        
        //printk(KERN_INFO"%s: finis\n", __FUNCTION__);
        if (rc) {
                printk (KERN_INFO "%s: failed\n", __FUNCTION__);
                bi_udc_exit (NULL);
        }
        return rc;
}


#ifdef CONFIG_ARCH_EZX
unsigned int saved_bMaxPower;
int powermode_flag;

void restore_powermode(void)
{
	struct usb_function_driver *function_driver;
	struct usb_configuration_instance *cfg_inst;
	struct usb_configuration_descriptor *cfg_desc;
	int cfg = 0;

	printk (KERN_INFO "%s: saved_bMaxPower %d powermode_flag %d\n", __FUNCTION__, saved_bMaxPower, powermode_flag);
	if(usbd_bus && powermode_flag)	//saved_bMaxPower)
	{
		function_driver = usbd_bus->function_instance->function_driver;
		cfg = usbd_bus->ConfigurationValue ? (usbd_bus->ConfigurationValue - 1) : 0;
		printk (KERN_INFO "%s: cfg %d\n", __FUNCTION__, cfg);
		cfg_inst = &function_driver->configuration_instance_array[cfg];
		cfg_desc = cfg_inst->configuration_descriptor;
		cfg_desc->bMaxPower = saved_bMaxPower;
		powermode_flag = 0;
	}
	return ;
}

// only change the current configurations's power mode
// So when switching, it will use the function driver's own power setting
static void change_powermode(void)
{
	struct usb_function_driver *function_driver;
	struct usb_configuration_instance *cfg_inst;
	struct usb_configuration_descriptor *cfg_desc;
	int cfg = 0;

//LIN	if(usbd_bus)
        if(usbd_bus && powermode_flag)  //LIN fix bug libgg91450
	{
		function_driver = usbd_bus->function_instance->function_driver;
		cfg = usbd_bus->ConfigurationValue ? (usbd_bus->ConfigurationValue - 1) : 0;
		printk (KERN_INFO "%s: cfg %d\n", __FUNCTION__, cfg);
		cfg_inst = &function_driver->configuration_instance_array[cfg];
		cfg_desc = cfg_inst->configuration_descriptor;
		saved_bMaxPower = cfg_desc->bMaxPower;
		cfg_desc->bMaxPower = POWER_CURRENT_100MA_SETTING;
		powermode_flag = 1;
		printk (KERN_INFO "%s: saved_bMaxPower %d\n", __FUNCTION__, saved_bMaxPower);
	}
	return ;
}

extern int udccontrol_context_safe(int control_flag);

void reenum_timeout(unsigned long data)
{
	change_powermode();

	// queue post-reenum event to thread context
	udccontrol_context_safe(1);
	return ;
}
#endif	// CONFIG_ARCH_EZX

/* Module Init - the module init and exit routines ************************************* */
static int bi_modinit (void)
{
        extern const char *usbd_bi_module_info(void);
	struct bi_data *data = NULL;
        static int first = 0;

        printk (KERN_INFO "%s: %s serial: \"%s\"\n", __FUNCTION__, usbd_bi_module_info(), 
                        serial_number_str && strlen(serial_number_str) ? serial_number_str : "");

#ifdef CONFIG_ARCH_EZX
	init_timer(&reenum_timer);
	reenum_timer.function = reenum_timeout;
	reenum_timer.data = 0;
	saved_bMaxPower = 0;
	powermode_flag = 0;
#endif
	
        down(&usbd_bi_sem);

        THROW_IF(trace_init(), error);
        THROW_IF(usbd_bus, error);

        // Set the UDC defaults
        udc_disable_interrupts();
        udc_disconnect();

	bi_driver.name = udc_name ();
	bi_driver.max_endpoints = udc_max_endpoints ();
	bi_driver.maxpacketsize = udc_ep0_packetsize ();

	// register this bus interface driver and create the device driver instance
	THROW_IF(!(usbd_bus = usbd_register_bus (&bi_driver)), error);

	THROW_IF (!(data = ckmalloc (sizeof (struct bi_data), GFP_KERNEL)), error);
	memset (data, 0, sizeof (struct bi_data));
	usbd_bus->privdata = data;

	// see if we can scrounge up something to set a sort of unique device address
	if (udc_serial_init ()) {
                if (serial_number_str && strlen(serial_number_str)) 
                        bi_serial_number(usbd_bus, serial_number_str);
        }
        else 
                bi_fix_serial_number_str(usbd_bus);
        
        have_cable_irq = !udc_request_cable_irq ();
        up(&usbd_bi_sem);
        
#if defined(MODULE) || !defined(CONFIG_USBD_BI_DELAY_ENABLE)
        // if we are connected OR if we don't have a cable irq fake an attach event
        
        // XXX this is not quite correct, if not attached then an enable
        // will be required later if attached on a system that does not have
        // cable event.
        //
        //if (!have_cable_irq || udc_attached()) {
	// Even if we have a cable irq, we need to check the current status,
	// because the cable may have been attached before we installed the handler.

        printk(KERN_INFO"%s: checking USB cable\n", __FUNCTION__);
        if (udc_attached()) {
                printk(KERN_INFO"%s: cable attached\n", __FUNCTION__);
                udc_cable_event();
	}
#endif

	return 0;

        CATCH(error) {
                printk(KERN_ERR"%s: error loading module\n", __FUNCTION__);
                if (data)
                        lkfree(data);
                if (usbd_bus) 
                        usbd_deregister_bus (usbd_bus);
                usbd_bus = NULL;
                trace_exit();
                up(&usbd_bi_sem);
                printk(KERN_INFO"%s: UP\n", __FUNCTION__);
                return -EINVAL;
        }
}

#ifdef MODULE
/* bi_modexit - This is *only* used for drivers compiled and used as a module.
 */
static void bi_modexit (void)
{
        down(&usbd_bi_sem);
        RETURN_IF(!usbd_bus);

	if (have_cable_irq) 
                udc_release_cable_irq ();

        bus_disable_sem(usbd_bus, NULL);
        
        if ((usbd_bus->privdata)) 
                lkfree (usbd_bus->privdata);

        usbd_bus->privdata = NULL;

        if (usbd_bus->serial_number_str) 
                lkfree (usbd_bus->serial_number_str);
        
        usbd_deregister_bus (usbd_bus);
        usbd_bus = NULL;
        trace_exit();
}

module_exit (bi_modexit);
#endif

module_init (bi_modinit);

