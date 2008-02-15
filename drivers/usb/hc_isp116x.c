/*-------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------*
 * isp1161 USB HCD for Linux Version 0.8 (9/3/2001)
 * 
 * Roman Weissgaerber weissg@vienna.at (C) 2001
 *
 *-------------------------------------------------------------------------*
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *-------------------------------------------------------------------------*/

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/smp_lock.h>
#include <linux/list.h>
#include <linux/ioport.h>
#include <asm/io.h>

#include <linux/usb.h>

#include "hc_isp116x.h"

static int hc_verbose = 1;
static int hc_error_verbose = 0;
static int urb_debug = 0;

static int irq = 25;
static unsigned long hcport = 0xf9080000;


MODULE_PARM(hc_verbose,"i");
MODULE_PARM_DESC(hc_verbose,"verbose startup messages, default is 1 (yes)");
MODULE_PARM(hc_error_verbose,"i");
MODULE_PARM_DESC(hc_error_verbose,"verbose error/warning messages, default is 0 (no)");
MODULE_PARM(urb_debug,"i");
MODULE_PARM_DESC(urb_debug,"debug urb messages, default is 0 (no)");

MODULE_PARM(irq,"i");
MODULE_PARM_DESC(irq,"IRQ 3, 10 (default)");
MODULE_PARM(hcport,"i");
MODULE_PARM_DESC(hcport,"ISP116x PORT 0x290");

/*-------------------------------------------------------------------------*
 * Following from hc_simple.c Version 0.8 (9/3/2001)
 * Roman Weissgaerber weissg@vienna.at (C) 2001
 *-------------------------------------------------------------------------*/

/* main lock for urb access */
static spinlock_t usb_urb_lock = SPIN_LOCK_UNLOCKED; 

/*-------------------------------------------------------------------------*/ 

#ifdef CONFIG_ARCTIC2  /* MVL-CEE */
#include <linux/device.h>

static int hc_isp_suspend(struct device * dev, u32 state, u32 level);
static int hc_isp_resume(struct device * dev, u32 level);
static int hc_isp_scale(struct bus_op_point * op, u32 level);

static struct device_driver hc_isp_driver_ldm = {
       name:      	"USB_PH_ISP1161",
       devclass:  	NULL,
       probe:     	NULL,
       suspend:   	hc_isp_suspend,
       resume:    	hc_isp_resume,
       scale:	  	hc_isp_scale,
       remove:    	NULL,
};

static struct device hc_isp_device_ldm = {
       name:		"Philips ISP1161 USB ",
       bus_id:		"hc_isp",
       driver: 		NULL,
       power_state:	DPM_POWER_OFF,
};

static void hc_isp_ldm_register(void)
{
	extern void ebc_driver_register(struct device_driver *driver);
	extern void ebc_device_register(struct device *device);

	ebc_driver_register(&hc_isp_driver_ldm);
	ebc_device_register(&hc_isp_device_ldm);
}

static void hc_isp_ldm_unregister(void)
{
	extern void ebc_driver_unregister(struct device_driver *driver);
	extern void ebc_device_unregister(struct device *device);

	ebc_driver_unregister(&hc_isp_driver_ldm);
	ebc_device_unregister(&hc_isp_device_ldm);
}

static int hc_isp_scale(struct bus_op_point * op, u32 level)
{
	/* linux-pm-TODO */

	return 0;
}

/*
 * TODO: get rid of pm_hci.
 */

static hci_t *pm_hci = NULL;
static int hc_reset (hci_t * hci);
static int hc_start (hci_t * hci);
static int hc_alloc_trans_buffer (hci_t * hci);

static void hc_isp116x_intregdump(hci_t * hci)
{
       printk("HcInterruptStatus=%x HcInterruptEnable=%x\n", READ_REG32 (hci, HcInterruptStatus), READ_REG32 (hci, HcInterruptEnable));
       printk("HcuPInterrupt=%x HcuPInterruptEnable=%x\n", READ_REG16 (hci, HcuPInterrupt), READ_REG16 (hci, HcuPInterruptEnable));
       printk("HcHardwareConfiguration=%x\n", READ_REG16 (hci,HcHardwareConfiguration)); 
}

static int hc_isp_suspend(struct device * dev, u32 state, u32 level)
{

  switch(level)
  { 
     case SUSPEND_POWER_DOWN:

       WRITE_REG16 (pm_hci, 0, HcDMAConfiguration);
       WRITE_REG16 (pm_hci, 0, HcuPInterruptEnable);
       WRITE_REG16(pm_hci, READ_REG16(pm_hci, HcHardwareConfiguration) & 
		   ~InterruptPinEnable, HcHardwareConfiguration);
       hc_release_hci(pm_hci);
       break;
  }
  
  return 0;
}

static int hc_isp_resume(struct device * dev, u32 level)
{

  switch(level)
  {
     case RESUME_POWER_ON:

		/*
		 * Turn off sleep mode.
		 */

       hc_found_hci(hcport, irq, 0);  

       break;
  }

  return 0;
}
#endif /* MVL-CEE */

/*-------------------------------------------------------------------------*/ 
/* URB HCD API function layer
 * * * */

/*-------------------------------------------------------------------------*/ 
/* set actual length to 0 and set status 
 * queue URB
 * */
 
static inline int hcs_urb_queue (hci_t * hci, struct urb * urb)
{
	int i;
	
	if (usb_pipeisoc (urb->pipe)) {
		for (i = 0; i < urb->number_of_packets; i++) {
  			urb->iso_frame_desc[i].actual_length = 0;
  			urb->iso_frame_desc[i].status = -EXDEV;
  		}
	}	

	urb->status = USB_ST_URB_PENDING;
	urb->actual_length = 0;
	urb->error_count = 0;

	qu_queue_urb (hci, urb);

	return 0;
}

/*-------------------------------------------------------------------------*/
/* return path (complete - callback) of URB API
 * also handles resubmition of intr URBs 
 * */

static int hcs_return_urb (hci_t * hci, struct urb * urb, int resub_ok)
{	
	struct usb_device * dev = urb->dev;
	int resubmit = 0;

	if (urb_debug)
		urb_print (urb, "RET", usb_pipeout (urb->pipe));
	
  	resubmit = urb->interval && resub_ok;
 	
	urb->dev = urb->hcpriv = NULL;

	if (urb->complete) 
		urb->complete (urb); /* call complete */
	
	if (resubmit) { /* requeue the URB */
		urb->dev = dev;		
		hcs_urb_queue (hci, urb);
	}

	return 0;
}


/*-------------------------------------------------------------------------*/
/* got a transfer request 
 * */
 
static int hci_submit_urb (struct urb * urb)
{
	hci_t * hci;
	unsigned int pipe = urb->pipe;
	unsigned long flags;
	int ret;

	if (!urb->dev || !urb->dev->bus || urb->hcpriv) 
		return -EINVAL;

	if (usb_endpoint_halted (urb->dev, 
				usb_pipeendpoint (pipe), usb_pipeout (pipe))) 
		return -EPIPE;
	
	hci = (hci_t *) urb->dev->bus->hcpriv;
	
	/* a request to the virtual root hub */
	if (usb_pipedevice (pipe) == hci->rh.devnum) {
		return rh_submit_urb (urb);	
	}

	if (urb_debug)
		urb_print (urb, "SUB", usb_pipein (pipe));

	/* queue the URB to its endpoint-queue */
	 
	spin_lock_irqsave (&usb_urb_lock, flags);	
	ret = hcs_urb_queue (hci, urb);
	spin_unlock_irqrestore (&usb_urb_lock, flags);

	return ret;

}

/*-------------------------------------------------------------------------*/
/* unlink URB: mark the URB to unlink  
 * */
 
static int hci_unlink_urb (struct urb * urb)
{
	unsigned long flags;
	hci_t * hci;
	DECLARE_WAITQUEUE (wait, current);
	void * comp = NULL;
	
	if (!urb) /* just to be sure */ 
		return -EINVAL;
		
	if (!urb->dev || !urb->dev->bus)
		return -ENODEV;

	hci = (hci_t *) urb->dev->bus->hcpriv; 

	/* a request to the virtual root hub */
	if (usb_pipedevice (urb->pipe) == hci->rh.devnum) {
		return rh_unlink_urb (urb); 
	}

	if (urb_debug)
		urb_print (urb, "UNLINK", 1);
		
	spin_lock_irqsave (&usb_urb_lock, flags);

	/* HACK: It seems the HID driver sometimes tries to unlink
	 * URBs which have never been submitted - and hence whose
	 * urb_list field has never been initialized.  This causes
	 * crashes on unplug */
	if ( (urb->urb_list.prev &&  urb->urb_list.next)
	     && !list_empty (&urb->urb_list)) { /* URB active? */
	
		if (urb->transfer_flags & (USB_ASYNC_UNLINK | USB_TIMEOUT_KILLED)) { 
			/* asynchron with callback */
		
			list_del (&urb->urb_list); /* relink the urb to the del list */
			list_add (&urb->urb_list, &hci->del_list);
			comp = urb->complete;
			urb->complete = NULL;
			spin_unlock_irqrestore (&usb_urb_lock, flags);

		} else { /* synchron without callback */
			
			add_wait_queue (&hci->waitq, &wait);	  

			set_current_state (TASK_UNINTERRUPTIBLE);

			list_del (&urb->urb_list); /* relink the urb to the del list */
			list_add (&urb->urb_list, &hci->del_list);
			spin_unlock_irqrestore (&usb_urb_lock, flags);
			
			schedule_timeout(HZ/50);
			
			if (!list_empty (&urb->urb_list))
				list_del (&urb->urb_list);
			urb->complete = comp;
			urb->hcpriv = NULL;
			remove_wait_queue (&hci->waitq, &wait); 
		}
	} else { /* hcd does not own URB but we keep the driver happy anyway */
		spin_unlock_irqrestore (&usb_urb_lock, flags);
		
		if (urb->complete && (urb->transfer_flags & USB_ASYNC_UNLINK)) {	
			urb->status = -ENOENT;
			urb->actual_length = 0;		
			urb->complete (urb); 
			urb->status = 0;	
		} else {
			urb->status = -ENOENT;
		}	
	}	

	return 0;
}

/*-------------------------------------------------------------------------*/
/* allocate private data space for a usb device 
 * */

static int hci_alloc_dev (struct usb_device * usb_dev)
{
	struct hci_device * dev;
	int i;
	
	dev = kmalloc (sizeof (*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
		
	memset (dev, 0, sizeof (*dev));

	for (i = 0; i < 32; i++) {
		INIT_LIST_HEAD (&(dev->urb_queue [i]));
		dev->pipe_head [i] = NULL;
	}
		
	usb_dev->hcpriv = dev;
	
	if (hc_verbose)
		printk ("USB HC dev alloc %d bytes\n", sizeof (*dev));
		
	return 0;
}

/*-------------------------------------------------------------------------*/
/* free private data space of usb device 
 * */
  
static int hci_free_dev (struct usb_device * usb_dev)
{
	if (hc_verbose)
		printk ("USB HC dev free\n");
		
	if (usb_dev->hcpriv)
		kfree (usb_dev->hcpriv);

	usb_dev->hcpriv = NULL;

	return 0;
}

/*-------------------------------------------------------------------------*/
/* tell us the current USB frame number 
 * */

static int hci_get_current_frame_number (struct usb_device *usb_dev) 
{
	hci_t * hci = usb_dev->bus->hcpriv;
	int frame_no;
	
	frame_no =  GET_FRAME_NUMBER (hci);
	return frame_no;
}

/*-------------------------------------------------------------------------*/
/* make a list of all io-functions 
 * */
 
static struct usb_operations hci_device_operations = {
	hci_alloc_dev,
	hci_free_dev,
	hci_get_current_frame_number,
	hci_submit_urb,
	hci_unlink_urb
};


/*-------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------*/
/* URB queueing:
 * 
 * For each type of transfer (INTR, BULK, ISO, CTRL) there is a list of 
 * active URBs.
 * (hci->intr_list, hci->bulk_list, hci->iso_list, hci->ctrl_list)
 * For every endpoint the head URB of the queued URBs is linked to one of 
 * those lists.
 * 
 * The rest of the queued URBs of an endpoint are linked into a 
 * private URB list for each endpoint. (hci_dev->urb_queue [endpoint_io])
 * hci_dev->pipe_head [endpoint_io] .. points to the head URB which is 
 * in one of the active URB lists.
 * 
 * The index of an endpoint consists of its number and its direction.
 * 
 * The state of an intr and iso URB is 0. 
 * For ctrl URBs the states are US_CTRL_SETUP, US_CTRL_DATA, US_CTRL_ACK
 * Bulk URBs states are US_BULK and US_BULK0 (with 0-len packet)
 * 
 * * * */
 
static inline int qu_pipeindex (__u32 pipe) 
{
	return (usb_pipeendpoint (pipe) << 1) | (usb_pipecontrol (pipe) ? 
				0 : usb_pipeout (pipe));
}

static inline void qu_seturbstate (struct urb * urb, int state) 
{
	urb->pipe &= ~0x3;
	urb->pipe |= state & 0x3;
}

static inline int qu_urbstate (struct urb * urb) 
{
	return urb->pipe & 0x3;
}

/*-------------------------------------------------------------------------*/	

static inline void qu_queue_active_urb (hci_t * hci, struct urb * urb) 
{
	int urb_state = 0;

	switch (usb_pipetype (urb->pipe)) {
		case PIPE_CONTROL:
			list_add (&urb->urb_list, &hci->ctrl_list);
			urb_state = US_CTRL_SETUP;
			break;
		
		case PIPE_BULK:
			list_add (&urb->urb_list, &hci->bulk_list);
			if (urb->transfer_flags & USB_ZERO_PACKET)
				urb_state = US_BULK0;
			break;
		
		case PIPE_INTERRUPT:
			urb->start_frame = (hci->frame_no + urb->interval) & 0x7ff;
			list_add (&urb->urb_list, &hci->intr_list);
			break;
		
		case PIPE_ISOCHRONOUS:
			list_add (&urb->urb_list, &hci->iso_list);
			break;
	}
	qu_seturbstate (urb, urb_state);
}

static int qu_queue_urb (hci_t * hci, struct urb * urb)
{
	int index = qu_pipeindex (urb->pipe);
	struct hci_device * hci_dev = usb_to_hci (urb->dev);

	if (hci_dev->pipe_head [index]) {
		__list_add (&urb->urb_list, 
				hci_dev->urb_queue [index].prev, &(hci_dev->urb_queue [index]));
	 } else {
	 	hci_dev->pipe_head [index] = urb;
		qu_queue_active_urb (hci, urb);
	}
	return 0;
}

/*-------------------------------------------------------------------------*/
/* return path (after transfer)
 * remove URB from queue and return it to the api layer
 * */

static void qu_return_urb (hci_t * hci, struct urb * urb, int resub_ok) 
{	
	struct hci_device * hci_dev = usb_to_hci (urb->dev);
	int index = qu_pipeindex (urb->pipe);
	
	list_del (&urb->urb_list);
	INIT_LIST_HEAD (&urb->urb_list);
		
	if (!list_empty (&(hci_dev->urb_queue [index]))) {
		urb = list_entry (hci_dev->urb_queue [index].next, struct urb, urb_list);
		list_del (&urb->urb_list);
		INIT_LIST_HEAD (&urb->urb_list);
		hci_dev->pipe_head [index] = urb;
		qu_queue_active_urb (hci, urb);
	} else {
		hci_dev->pipe_head [index] = NULL;
	}
	
	hcs_return_urb (hci, urb, resub_ok);	
}

/*-------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------*/
/* SCHEDULE operations
 * 
 * * * */


static int sh_scan_urb_list (hci_t * hci, struct list_head * list_lh) 
{
	struct list_head * lh;
	struct urb * urb;

	list_for_each (lh, list_lh) {
		urb = list_entry (lh, struct urb, urb_list);
		if (!usb_pipeint (urb->pipe) ||
			(((hci->frame_no - urb->start_frame) & 0x7ff) >= urb->interval)) {
				if (!sh_add_packet (hci, urb))
					return 0;
				else
					urb->start_frame = hci->frame_no;
		}
	}
	return 1;
}

/*-------------------------------------------------------------------------*/
/* scan lists of active URBs and construct a schedule for a frame
 * */

static int sh_schedule_trans (hci_t * hci)
{
	int units_left;
	hci->trans = 0;
		
	units_left = sh_scan_urb_list (hci, &hci->intr_list);

	if (units_left) {  /* add CTRL transfers */
		units_left = sh_scan_urb_list (hci, &hci->ctrl_list);
	}
 	
	if (units_left) {  /* add BULK transfers */
		sh_scan_urb_list (hci, &hci->bulk_list);
	}

	return 0;
}

/*-------------------------------------------------------------------------*/
/* add some parts of the active URB to the schedule
 * */

static int sh_add_packet (hci_t * hci, struct urb * urb)
{
	__u8 * data = NULL;
	int len = 0;
	int toggle = 0;
	int maxps = usb_maxpacket (urb->dev, urb->pipe, usb_pipeout (urb->pipe));
	int endpoint = usb_pipeendpoint (urb->pipe);
	int address = usb_pipedevice (urb->pipe);
	int slow = usb_pipeslow (urb->pipe);
	int out = usb_pipeout (urb->pipe);
	int pid = 0;
	int ret;
	int i;
	int iso = 0;

 	dbg("transfer_pa: addr: %d, ep:%d, out:%x in:%x\n", 
	    address, endpoint, urb->dev->toggle[1], urb->dev->toggle[0]);

	if (maxps == 0) maxps = 8;

	/* calculate len, toggle bit and add the transaction */
	switch (usb_pipetype (urb->pipe)) {
		case PIPE_ISOCHRONOUS:
			iso = 1;
			i = hci->frame_no - urb->start_frame;
			data = urb->transfer_buffer + urb->iso_frame_desc[i].offset;
			len = urb->iso_frame_desc[i].length;
			break;
			
		case PIPE_BULK: /* BULK and BULK0 */
		case PIPE_INTERRUPT:
			pid = out ? PID_OUT : PID_IN;	
			len = urb->transfer_buffer_length - urb->actual_length;
			data = urb->transfer_buffer + urb->actual_length;
			toggle = usb_gettoggle (urb->dev, endpoint, out);
			break;

		case PIPE_CONTROL:
			switch (qu_urbstate (urb)) {
				case US_CTRL_SETUP:	
					len = 8;
					pid = PID_SETUP;
					data = urb->setup_packet;
					toggle = 0;
					break;

				case US_CTRL_DATA:
					if (urb->transfer_buffer_length != 0) {
						pid = out ? PID_OUT : PID_IN;	
						len = urb->transfer_buffer_length - urb->actual_length;
						data = urb->transfer_buffer + urb->actual_length;
						toggle = (urb->actual_length & maxps) ? 0 : 1;
						break;
					} else
						/* correct state and fall through */
						qu_seturbstate (urb, US_CTRL_ACK);
						
				case US_CTRL_ACK:
					len = 0;
					/* reply in opposite direction */
					pid = !out ? PID_OUT : PID_IN; 
					toggle = 1;
					break;
			}
	}

	ret = hc_add_trans (hci, len, data, toggle, maxps, slow, 
						endpoint, address, pid, iso);
	if (ret) {
		hci->urb_array [hci->trans] = urb;
		hci->trans ++;
		if (usb_pipetype (urb->pipe) == PIPE_INTERRUPT)
			 urb->start_frame = hci->frame_no;
	}

	return ret;
}


/*-------------------------------------------------------------------------*/
/* parse the done_list
 * */

static int sh_done_list (hci_t * hci) 
{
	int actbytes;
	int active = 0;
	void * data = NULL; 
	void * data_urb = NULL;
	int cc;
	int maxps;
	int toggle;
	struct urb * urb;
	int urb_state;
	int ret = 1; /* -1 parse abbort, 1 parse ok, 0 last element */
	int trans = 0;

	for (trans = 0; trans < hci->trans; trans++) {

		urb = hci->urb_array [trans];

		if (urb->dev == 0) {
			printk("hc_isp1116x: urb on done list has no device.\n");
			continue;
		}

		urb_state = qu_urbstate (urb);
		toggle = usb_gettoggle (urb->dev, usb_pipeendpoint (urb->pipe), 
					usb_pipeout (urb->pipe));
	
		ret = hc_parse_trans (hci, &actbytes, &data, &cc, &toggle);
		maxps = usb_maxpacket (urb->dev, urb->pipe, usb_pipeout (urb->pipe));

		if (maxps == 0) maxps = 8;

		if (cc != TD_NOTACCESSED) {
			active = (urb_state != US_CTRL_SETUP) && 
					(actbytes && !(actbytes & (maxps - 1)) &&
					(urb->transfer_buffer_length != urb->actual_length + actbytes));

			data_urb = urb->transfer_buffer + urb->actual_length;
			if (!(urb->transfer_flags & USB_DISABLE_SPD) && cc == TD_DATAUNDERRUN)
								cc = 0;
			if (cc) { /* last packet has an error */
				if (hc_error_verbose)
					printk("hc USB %d error %d in frame %d ep %d out %d addr %d\n", 
						urb->error_count, cc, hci->frame_no, 
							usb_pipeendpoint (urb->pipe),
							usb_pipeout (urb->pipe), usb_pipedevice (urb->pipe));

				if (++urb->error_count > 3 || cc == TD_CC_STALL) {
					active = 0;
					urb_state = 0; /* return */
				} else { 
			//		actbytes = actbytes & ~(maxps - 1);
					actbytes = 0;
					active = 1;
				}
			} else if (urb_state != US_CTRL_SETUP) { /* no error */
				if (!usb_pipeout (urb->pipe) && actbytes && data_urb != data)
					memcpy (data_urb, data, actbytes);
				urb->actual_length += actbytes;
				usb_settoggle (urb->dev, usb_pipeendpoint (urb->pipe), 
					usb_pipeout (urb->pipe), 
					!toggle);
//					(!actbytes || !((actbytes - 1) & maxps) ? !toggle : toggle));
			}
			
			if (!active) {
				if (!(urb_state--)) {
					urb->status = cc_to_error [cc];
					qu_return_urb (hci, urb, 1);
				} else {
					qu_seturbstate (urb, urb_state);
				}
			}
		}
	}	

	return ret;
}

/*-------------------------------------------------------------------------*/
/* scan the delete list (qu_return_urb removes URB from list) 
 * */

static void sh_del_list (hci_t * hci) 
{
	struct list_head * lh = &hci->del_list;
	struct urb * urb;

	if (!list_empty (lh)) {
		do {
			lh = lh->next;
			urb = list_entry (lh, struct urb, urb_list);
			qu_return_urb (hci, urb, 0);
		} while (!list_empty (lh));
		
		wake_up (&hci->waitq);
	}
	return;
}
	


/*-------------------------------------------------------------------------
 * ISP116x virtual root hub
 * From hc_isp116x_rh.c by Roman Weissgaerber weissg@vienna.at
 *-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*
 * Virtual Root Hub 
 *-------------------------------------------------------------------------*/
 
/* Device descriptor */
static __u8 root_hub_dev_des[] =
{
	0x12,       /*  __u8  bLength; */
	0x01,       /*  __u8  bDescriptorType; Device */
	0x10,	    /*  __u16 bcdUSB; v1.1 */
	0x01,
	0x09,	    /*  __u8  bDeviceClass; HUB_CLASSCODE */
	0x00,	    /*  __u8  bDeviceSubClass; */
	0x00,       /*  __u8  bDeviceProtocol; */
	0x08,       /*  __u8  bMaxPacketSize0; 8 Bytes */
	0x00,       /*  __u16 idVendor; */
	0x00,
	0x00,       /*  __u16 idProduct; */
 	0x00,
	0x00,       /*  __u16 bcdDevice; */
 	0x00,
	0x00,       /*  __u8  iManufacturer; */
	0x02,       /*  __u8  iProduct; */
	0x01,       /*  __u8  iSerialNumber; */
	0x01        /*  __u8  bNumConfigurations; */
};


/* Configuration descriptor */
static __u8 root_hub_config_des[] =
{
	0x09,       /*  __u8  bLength; */
	0x02,       /*  __u8  bDescriptorType; Configuration */
	0x19,       /*  __u16 wTotalLength; */
	0x00,
	0x01,       /*  __u8  bNumInterfaces; */
	0x01,       /*  __u8  bConfigurationValue; */
	0x00,       /*  __u8  iConfiguration; */
	0x40,       /*  __u8  bmAttributes; 
                 Bit 7: Bus-powered, 6: Self-powered, 5 Remote-wakwup, 4..0: resvd */
	0x00,       /*  __u8  MaxPower; */
      
	/* interface */	  
	0x09,       /*  __u8  if_bLength; */
	0x04,       /*  __u8  if_bDescriptorType; Interface */
	0x00,       /*  __u8  if_bInterfaceNumber; */
	0x00,       /*  __u8  if_bAlternateSetting; */
	0x01,       /*  __u8  if_bNumEndpoints; */
	0x09,       /*  __u8  if_bInterfaceClass; HUB_CLASSCODE */
	0x00,       /*  __u8  if_bInterfaceSubClass; */
	0x00,       /*  __u8  if_bInterfaceProtocol; */
	0x00,       /*  __u8  if_iInterface; */
     
	/* endpoint */
	0x07,       /*  __u8  ep_bLength; */
	0x05,       /*  __u8  ep_bDescriptorType; Endpoint */
	0x81,       /*  __u8  ep_bEndpointAddress; IN Endpoint 1 */
 	0x03,       /*  __u8  ep_bmAttributes; Interrupt */
 	0x02,       /*  __u16 ep_wMaxPacketSize; ((MAX_ROOT_PORTS + 1) / 8 */
 	0x00,
	0xff        /*  __u8  ep_bInterval; 255 ms */
};

/* Hub class-specific descriptor is constructed dynamically */


/*-------------------------------------------------------------------------*/

/* prepare Interrupt pipe data; HUB INTERRUPT ENDPOINT */ 
 
static int rh_send_irq (hci_t * hci, void * rh_data, int rh_len)
{
	int num_ports;
	int i;
	int ret;
	int len;
	u8 data[8];

	num_ports = READ_REG32 (hci, HcRhDescriptorA) & RH_A_NDP; 
	
	data[0] = (READ_REG32 (hci, HcRhStatus) & (RH_HS_LPSC | RH_HS_OCIC))
		? 1 : 0;
	ret = data[0];

	for (i = 0; i < num_ports; i++) {
		u32 reg = READ_REG32(hci, HcRhPortStatus + i);

		data[(i + 1) / 8] |= 
			((reg &
			 (RH_PS_CSC | RH_PS_PESC | RH_PS_PSSC | RH_PS_OCIC | RH_PS_PRSC))
			    ? 1 : 0) << ((i + 1) % 8);
		ret += data[(i + 1) / 8] ;
	}
	len = i/8 + 1;
  
	if (ret > 0) { 
		memcpy (rh_data, data, min(len, min_t(int,rh_len, sizeof(data))));
		return len;
	}
	return 0;
}

/*-------------------------------------------------------------------------*/

/* Virtual Root Hub INTs are polled by this timer every "interval" ms */
 
static void rh_int_timer_do (unsigned long ptr)
{
	int len; 

	struct urb * urb = (struct urb *) ptr;
	hci_t * hci = urb->dev->bus->hcpriv;

#if 0 /* cleanup? */
	if (hci->disabled)
//		return;

	/* ignore timers firing during PM suspend, etc */
	if ((hci->hc_control & OHCI_CTRL_HCFS) != OHCI_USB_OPER)
		goto out;
#endif

	if(hci->rh.send) { 
		len = rh_send_irq (hci, urb->transfer_buffer, urb->transfer_buffer_length);
		if (len > 0) {
			urb->actual_length = len;
			if (urb_debug == 2)
				urb_print (urb, "RET-t(rh)", usb_pipeout (urb->pipe));

			if (urb->complete)
				urb->complete (urb);
		}
	}
// out:
	rh_init_int_timer (urb);
}

/*-------------------------------------------------------------------------*/

/* Root Hub INTs are polled by this timer */

static int rh_init_int_timer (struct urb * urb) 
{
	hci_t * hci = urb->dev->bus->hcpriv;

	hci->rh.interval = urb->interval;
	init_timer (&hci->rh.rh_int_timer);
	hci->rh.rh_int_timer.function = rh_int_timer_do;
	hci->rh.rh_int_timer.data = (unsigned long) urb;
	hci->rh.rh_int_timer.expires = 
			jiffies + (HZ * (urb->interval < 30? 30: urb->interval)) / 1000;
	add_timer (&hci->rh.rh_int_timer);
	
	return 0;
}

/*-------------------------------------------------------------------------*/

#define OK(x) 			len = (x); break
#define WR_RH_STAT(x) 		WRITE_REG32 (hci, (x), HcRhStatus)
#define WR_RH_PORTSTAT(x) 	WRITE_REG32 (hci, (x), HcRhPortStatus + wIndex - 1)
#define RD_RH_STAT		READ_REG32 (hci, HcRhStatus)
#define RD_RH_PORTSTAT		READ_REG32 (hci, HcRhPortStatus + wIndex - 1)

/* request to virtual root hub */

static int rh_submit_urb (struct urb * urb)
{
	struct usb_device * usb_dev = urb->dev;
	hci_t * hci = usb_dev->bus->hcpriv;
	unsigned int pipe = urb->pipe;
	struct usb_ctrlrequest * cmd = (struct usb_ctrlrequest *) urb->setup_packet;
	void * data = urb->transfer_buffer;
	int leni = urb->transfer_buffer_length;
	int len = 0;
	int status = TD_CC_NOERROR;
	
	__u32 datab[4];
	__u8  * data_buf = (__u8 *) datab;
	
 	__u16 bmRType_bReq;
	__u16 wValue; 
	__u16 wIndex;
	__u16 wLength;

	if (usb_pipeint(pipe)) {
		hci->rh.urb =  urb;
		hci->rh.send = 1;
		hci->rh.interval = urb->interval;
		rh_init_int_timer(urb);
		urb->status = cc_to_error [TD_CC_NOERROR];
		
		return 0;
	}

	bmRType_bReq  = cmd->bRequestType | (cmd->bRequest << 8);
	wValue        = le16_to_cpu (cmd->wValue);
	wIndex        = le16_to_cpu (cmd->wIndex);
	wLength       = le16_to_cpu (cmd->wLength);

	dbg ("rh_submit_urb, req = %d(%x) len=%d", bmRType_bReq,
		bmRType_bReq, wLength);

	switch (bmRType_bReq) {
	/* Request Destination:
	   without flags: Device, 
	   RH_INTERFACE: interface, 
	   RH_ENDPOINT: endpoint,
	   RH_CLASS means HUB here, 
	   RH_OTHER | RH_CLASS  almost ever means HUB_PORT here 
	*/
  
	case RH_GET_STATUS: 				 		
		*(__u16 *) data_buf = cpu_to_le16 (1); OK (2);
	case RH_GET_STATUS | RH_INTERFACE: 	 		
		*(__u16 *) data_buf = cpu_to_le16 (0); OK (2);
	case RH_GET_STATUS | RH_ENDPOINT:	 		
		*(__u16 *) data_buf = cpu_to_le16 (0); OK (2);   
	case RH_GET_STATUS | RH_CLASS: 				
		*(__u32 *) data_buf = cpu_to_le32 (
			RD_RH_STAT & ~(RH_HS_CRWE | RH_HS_DRWE));
		OK (4);
	case RH_GET_STATUS | RH_OTHER | RH_CLASS: 	
		*(__u32 *) data_buf = cpu_to_le32 (RD_RH_PORTSTAT); OK (4);
		
	case RH_CLEAR_FEATURE | RH_ENDPOINT:  
		switch (wValue) {
		case (RH_ENDPOINT_STALL): OK (0);
		}
		break;
		
	case RH_CLEAR_FEATURE | RH_CLASS:
		switch (wValue) {
		case RH_C_HUB_LOCAL_POWER:
			OK(0);
		case (RH_C_HUB_OVER_CURRENT): 
			WR_RH_STAT(RH_HS_OCIC); OK (0);
		}
		break;
		
	case RH_CLEAR_FEATURE | RH_OTHER | RH_CLASS:
		switch (wValue) {
		case (RH_PORT_ENABLE): 			
			WR_RH_PORTSTAT (RH_PS_CCS ); OK (0);
		case (RH_PORT_SUSPEND):			
			WR_RH_PORTSTAT (RH_PS_POCI); OK (0);
		case (RH_PORT_POWER):			
			WR_RH_PORTSTAT (RH_PS_LSDA); OK (0);
		case (RH_C_PORT_CONNECTION):	
			WR_RH_PORTSTAT (RH_PS_CSC ); OK (0);
		case (RH_C_PORT_ENABLE):		
			WR_RH_PORTSTAT (RH_PS_PESC); OK (0);
		case (RH_C_PORT_SUSPEND):		
			WR_RH_PORTSTAT (RH_PS_PSSC); OK (0);
		case (RH_C_PORT_OVER_CURRENT):	
			WR_RH_PORTSTAT (RH_PS_OCIC); OK (0);
		case (RH_C_PORT_RESET):			
			WR_RH_PORTSTAT (RH_PS_PRSC); OK (0); 
		}
		break;
		
	case RH_SET_FEATURE | RH_OTHER | RH_CLASS:
		switch (wValue) {
		case (RH_PORT_SUSPEND):			
			WR_RH_PORTSTAT (RH_PS_PSS ); OK (0); 
		case (RH_PORT_RESET): /* BUG IN HUB CODE *********/
			if (RD_RH_PORTSTAT & RH_PS_CCS)
				WR_RH_PORTSTAT (RH_PS_PRS);
			OK (0);
		case (RH_PORT_POWER):			
			WR_RH_PORTSTAT (RH_PS_PPS ); OK (0); 
		case (RH_PORT_ENABLE): /* BUG IN HUB CODE *********/
			if (RD_RH_PORTSTAT & RH_PS_CCS)
				WR_RH_PORTSTAT (RH_PS_PES );
			OK (0);
		}
		break;
		
	case RH_SET_ADDRESS: hci->rh.devnum = wValue; OK(0);
		
	case RH_GET_DESCRIPTOR:
		switch ((wValue & 0xff00) >> 8) {
		case (0x01): /* device descriptor */
			len = min (leni, min_t(int, sizeof (root_hub_dev_des), wLength));
			data_buf = root_hub_dev_des; OK(len);
		case (0x02): /* configuration descriptor */
			len = min (leni, min_t(int, sizeof (root_hub_config_des), wLength));
			data_buf = root_hub_config_des; OK(len);
		case (0x03): /* string descriptors */
			len = usb_root_hub_string (wValue & 0xff,
						   (int)(long) 0, "ISP116x",
						   data, wLength);
			if (len > 0) {
				data_buf = data;
				OK (min (leni, len));
			}
			// else fallthrough
		default: 
			status = TD_CC_STALL;
		}
		break;
		
	case RH_GET_DESCRIPTOR | RH_CLASS:
	{
		__u32 temp = READ_REG32 (hci, HcRhDescriptorA);
		
		data_buf [0] = 9;		// min length;
		data_buf [1] = 0x29;
		data_buf [2] = temp & RH_A_NDP;
		data_buf [3] = 0;
		if (temp & RH_A_PSM) 	/* per-port power switching? */
			data_buf [3] |= 0x1;
		if (temp & RH_A_NOCP)	/* no overcurrent reporting? */
			data_buf [3] |= 0x10;
		else if (temp & RH_A_OCPM)	/* per-port overcurrent reporting? */
			data_buf [3] |= 0x8;
		
		datab [1] = 0;
		data_buf [5] = (temp & RH_A_POTPGT) >> 24;
		temp = READ_REG32 (hci, HcRhDescriptorB);
		data_buf [7] = temp & RH_B_DR;
		if (data_buf [2] < 7) {
			data_buf [8] = 0xff;
		} else {
				data_buf [0] += 2;
				data_buf [8] = (temp & RH_B_DR) >> 8;
				data_buf [10] = data_buf [9] = 0xff;
		}
		
		len = min (leni, min_t(int, data_buf [0], wLength));
		OK (len);
	}
	
	case RH_GET_CONFIGURATION: 	*(__u8 *) data_buf = 0x01; OK (1);
		
	case RH_SET_CONFIGURATION: 	WR_RH_STAT (0x10000); OK (0);
		
	default: 
		dbg ("unsupported root hub command");
		status = TD_CC_STALL;
	}
	
	len = min(len, leni);
	if (data != data_buf)
	    memcpy (data, data_buf, len);
  	urb->actual_length = len;
	urb->status = cc_to_error [status];
	
	// hci_return_urb (hci, urb, NULL); 
	urb->hcpriv = NULL;
	urb->dev = NULL;

	if (urb->complete)
	    	urb->complete (urb);
	
	return 0;
}

/*-------------------------------------------------------------------------*/

static int rh_unlink_urb (struct urb * urb)
{
	hci_t * hci = urb->dev->bus->hcpriv;
 
	if (hci->rh.urb == urb) {
		hci->rh.send = 0;
		del_timer (&hci->rh.rh_int_timer);
		hci->rh.urb = NULL;

		urb->hcpriv = NULL;
		urb->dev = NULL;
		if (urb->transfer_flags & USB_ASYNC_UNLINK) {
			urb->status = -ECONNRESET;
			if (urb->complete)
				urb->complete (urb);
		} else
			urb->status = -ENOENT;
	}
	return 0;
}

/*-------------------------------------------------------------------------*/
/* connect the virtual root hub */
 
static int rh_connect_rh (hci_t * hci) {
 
        struct usb_device  * usb_dev;
 
        hci->rh.devnum = 0;
        usb_dev = usb_alloc_dev (NULL, hci->bus);
        if (!usb_dev)
            return -ENOMEM;
 
        hci->bus->root_hub = usb_dev;
        usb_connect (usb_dev);
        if (usb_new_device (usb_dev) != 0) {
                usb_free_dev (usb_dev);
                return -ENODEV;
        }
 
        return 0;
} 




	
/*
 * ---------------------------------------------------------------------------
 * ISP116x driver
 */


static inline void REG_DELAY(void)
{
	/* The sepcs of the ISP1611 controller require a 300ns delay
	 * between writing the command register and writing another.
	 * The following is overkill, but will do until I come up with
	 * a better way. */
	udelay(1);
}

static inline u32 READ_REG32 (hci_t * hci, int regindex)
{
	hcipriv_t * hp = &hci->hp; 
	u32 val16, val;
	writew (regindex, hp->hcvirtual + 2);
	REG_DELAY();
	val16 = readw (hp->hcvirtual);
	val = val16;
	val16 = readw (hp->hcvirtual);
	val |= val16 << 16;

	return val;
}

static inline u16 READ_REG16 (hci_t * hci, int regindex) 
{
	hcipriv_t * hp = &hci->hp; 
	writew (regindex, hp->hcvirtual + 2);
	REG_DELAY();
	return readw (hp->hcvirtual);
}

static inline void READ_REGn16 (hci_t * hci, int regindex, int length, __u8 * buffer)
{
	hcipriv_t * hp = &hci->hp; 
	int i;
	unsigned int val = 0;
	writew (regindex, hp->hcvirtual + 2);
	REG_DELAY();
	for (i = 0; i < length; i += 2) {
		val = readw (hp->hcvirtual);
		buffer [i] = val;
		buffer [i+1] = val >> 8; 
	}
	if (length & 1) {
		val = readw (hp->hcvirtual);
		buffer [length - 1] = val;
	}
}

static inline void WRITE_REG32 (hci_t * hci, u32 value, int regindex) 
{
	hcipriv_t * hp = &hci->hp; 

	writew (regindex | 0x80, hp->hcvirtual + 2);
	REG_DELAY();
	writew (value, hp->hcvirtual);
	writew (value >> 16, hp->hcvirtual);
}

static inline void WRITE_REG16 (hci_t * hci, u16 value, int regindex) 
{
	hcipriv_t * hp = &hci->hp; 

	writew (regindex | 0x80, hp->hcvirtual + 2);
	REG_DELAY();
	writew (value, hp->hcvirtual);
}

static inline void WRITE_REGn16 (hci_t * hci, int regindex, int length, __u8 * buffer) 
{
	hcipriv_t * hp = &hci->hp; 
	int i;
	writew (regindex | 0x80, hp->hcvirtual + 2);
	REG_DELAY();

	for (i = 0; i < length; i+=2) {
		writew (buffer [i] + (buffer [i+1] << 8), hp->hcvirtual);
	}
	if (length & 1) {
		writew (buffer [length - 1], hp->hcvirtual);
	}
}

/*-------------------------------------------------------------------------*/
/* tl functions */

static inline void hc_mark_last_trans (hci_t * hci) 
{
	hcipriv_t * hp = &hci->hp; 
	__u8 * ptd = hp->tl;
	
	if (hp->tlp > 0)
		*(ptd + hp->tl_last) |= (1 << 3);
}

static inline int hc_add_trans (hci_t * hci, int len, void * data,
		int toggle, int maxps, int slow, int endpoint, int address, int pid, int format)
{
	hcipriv_t * hp = &hci->hp; 
	int last = 0;
	__u8 * ptd = hp->tl;

	/* just send 4 packets of each kind at a frame,
	 * other URBs also want some bandwitdh, and a NACK is cheaper
	 * with less packets */
	if (len > maxps * 1)
			len = maxps * 1; 
	
	if (hp->units_left < len + 8)
		return 0;
	else
		hp->units_left -= len + 8;

	
	ptd += hp->tlp;
	hp->tl_last = hp->tlp + 3;
	ptd [0] = 0;
	ptd [1] = (toggle << 2) | (1 << 3) | (0xf << 4);
	ptd [2] = maxps;
	ptd [3] = ((maxps >> 8) & 0x3) | (slow << 2) | (last << 3) | (endpoint << 4);
	ptd [4] = len;
	ptd [5] = ((len >> 8) & 0x3) | (pid << 2);
	ptd [6] = address | (format << 7);
    ptd [7] = 0;
    
	memcpy (ptd + 8, data, len);
	hp->tlp += ((len + 8 + 3) & ~0x3);

	return 1;
}

static inline int hc_parse_trans (hci_t * hci, int * actbytes, void ** data, 
			int * cc, int * toggle)
{
	hcipriv_t * hp = &hci->hp; 
	int last = 0;
	int totbytes;
	__u8 *ptd = hp->tl + hp->tlp;


	*cc = (ptd [1] >> 4) & 0xf;
	last = (ptd [3] >> 3) & 0x1;
	*actbytes = ((ptd [1] & 0x3) << 8) | ptd [0];
	totbytes = ((ptd [5] & 0x3) << 8) | ptd [4];
	*data = ptd + 8;

	*toggle = !(ptd [1] >> 2 & 1);
	hp->tlp += ((totbytes + 8 + 3) & ~0x3);  
	
	return !last;
}


/*-------------------------------------------------------------------------*/

/*
static void hc_start_list (hci_t * hci, ed_t * ed)
{
	hcipriv_t * hp = hci->hcipriv;
	int units_left = hp->atl_buffer_len;
	
	switch (usb_pipetype (ed->pipe)) {
	case PIPE_CONTROL:
	case PIPE_BULK:
	case PIPE_INTERRUPT:
		if (hp->atl_len == 0) {
			hp->tlp = 0;
			sh_schedule_trans (hci, TR_INTR | TR_CTRL | TR_BULK, &units_left);
			mark_last_trans (hci);
			hp->atl_len = hp->tlp;
			if (hp->atl_len > 0) {
				WRITE_REG16 (hci, hp->atl_len, HcTransferCounter);
				WRITE_REGn16 (hci, HcATLBufferPort, hp->atl_len, hp->tl);
			}
		}
	}
}	
*/

/* an interrupt happens */
static void hc_interrupt (int irq, void * __hci, struct pt_regs * r)
{
	hci_t * hci = __hci;
	hcipriv_t * hp = &hci->hp; 
	int ints_uP, ints = 0, bstat = 0;

	if ((ints_uP = (READ_REG16 (hci, HcuPInterrupt) & READ_REG16 (hci, HcuPInterruptEnable))) == 0) {
		return;
	}
	WRITE_REG16 (hci, ints_uP, HcuPInterrupt);
	if ((ints_uP & OPR_Reg) &&
 		(ints = (READ_REG32 (hci, HcInterruptStatus)))) {

		if (ints & OHCI_INTR_SO) {
			dbg("USB Schedule overrun");
			WRITE_REG32 (hci, OHCI_INTR_SO, HcInterruptEnable);
		}

		if (ints & OHCI_INTR_SF) {
			WRITE_REG32 (hci, OHCI_INTR_SF, HcInterruptEnable);
		}
		WRITE_REG32 (hci, ints, HcInterruptStatus);
		WRITE_REG32 (hci, OHCI_INTR_MIE, HcInterruptEnable);
	}

	bstat = READ_REG16 (hci, HcBufferStatus);
	if (ints_uP & SOFITLInt) {	
	}

	hci->frame_no = GET_FRAME_NUMBER (hci);

#ifdef DEBUG
	if (hci->frame_no == 0) 
		printk("*** INTERRUPT frame %u intuP %x ints %x bstat %x \n", hci->frame_no, ints_uP, ints, bstat);
#endif

	if (ints_uP & ATLInt) {
		dbg("int2 %x %x bstat %x\n", ints_uP, ints, bstat);

		if ((bstat & ATLBufferFull) && (bstat & ATLBufferDone)) {
			if (hp->atl_len > 0) {
				READ_REGn16 (hci, HcATLBufferPort, hp->atl_len, hp->tl);
				hp->tlp = 0;
				sh_done_list (hci);
			}
			hp->tlp = 0;
			hci->trans = 0;
		}
	}
	sh_del_list (hci);
	if (hci->trans == 0 && !(bstat & ATLBufferFull)) {
		hp->units_left = hp->atl_buffer_len;
		hp->tlp = 0;
		sh_schedule_trans (hci);
		hc_mark_last_trans (hci);
		hp->atl_len = hp->tlp;
		if (hp->atl_len > 0) {
			WRITE_REG16 (hci, hp->atl_len, HcTransferCounter);
			WRITE_REGn16 (hci, HcATLBufferPort, hp->atl_len, hp->tl);
		}
		
	}
}



/*-------------------------------------------------------------------------*
 * HC functions
 *-------------------------------------------------------------------------*/

/* reset the HC and BUS */

static int hc_reset (hci_t * hci)
{
	int timeout = 30;
	 	
	/* Disable HC interrupts */
	WRITE_REG32 (hci, OHCI_INTR_MIE, HcInterruptDisable);

	dbg ("USB HC reset_hc usb-: ctrl = 0x%x ;",
		READ_REG32 (hci, HcControl));

  	/* Reset USB (needed by some controllers) */
	WRITE_REG32 (hci, 0, HcControl);
  
	WRITE_REG16 (hci, 0xf6, HcSoftwareReset);
    	
	/* HC Reset requires max 10 us delay */
	WRITE_REG32 (hci, OHCI_HCR, HcCommandStatus);
	while ((READ_REG32 (hci, HcCommandStatus) & OHCI_HCR) != 0) {
		if (--timeout == 0) {
			err ("USB HC reset timed out!");
			return -1;
		}	
		udelay (1);
	}	 
	return 0;
}

/*-------------------------------------------------------------------------*/

/* Start an host controller, set the BUS operational
 * enable interrupts 
 * connect the virtual root hub */

static int hc_alloc_trans_buffer (hci_t * hci)
{
	hcipriv_t * hp = &hci->hp; 
	int maxlen;

	hp->itl0_len = 0;
	hp->itl1_len = 0;
	hp->atl_len = 0;

	hp->itl_buffer_len = 1024;
	hp->atl_buffer_len = 4096 - 2 * hp->itl_buffer_len; /* 2048 */

	WRITE_REG16 (hci, hp->itl_buffer_len, HcITLBufferLength);
	WRITE_REG16 (hci, hp->atl_buffer_len, HcATLBufferLength); 
	WRITE_REG16 (hci, 
		InterruptPinEnable |
		InterruptPinTrigger |
		InterruptOutputPolarity | 
		DataBusWidth16 | 
		AnalogOCEnable, 
		HcHardwareConfiguration);
	WRITE_REG16 (hci, 0, HcDMAConfiguration);

	maxlen = (hp->itl_buffer_len > hp->atl_buffer_len) ? hp->itl_buffer_len : hp->atl_buffer_len;

	if (!hp->tl)
		hp->tl = kmalloc (maxlen, GFP_KERNEL);
	
	if (!hp->tl)
		return -ENOMEM;

	memset (hp->tl, 0, maxlen);
	return 0;
}
	
static int hc_start (hci_t * hci)
{
	hcipriv_t * hp = &hci->hp; 
  	__u32 mask;
  	unsigned int fminterval;

  	fminterval = 0x2edf;
	fminterval |= ((((fminterval - 210) * 6) / 7) << 16); 
	WRITE_REG32 (hci, fminterval, HcFmInterval);	
	WRITE_REG32 (hci, 0x628, HcLSThreshold);


 	/* start controller operations */
 	hp->hc_control = OHCI_USB_OPER;
 	WRITE_REG32 (hci, hp->hc_control, HcControl);
 
	/* Choose the interrupts we care about now, others later on demand */
	mask = OHCI_INTR_MIE | 
	OHCI_INTR_SO | 
	OHCI_INTR_SF;

	WRITE_REG32 (hci, mask, HcInterruptEnable);
	WRITE_REG32 (hci, mask, HcInterruptStatus);

	mask = SOFITLInt | ATLInt | OPR_Reg;
	WRITE_REG16 (hci, mask, HcuPInterrupt); 
	WRITE_REG16 (hci, mask, HcuPInterruptEnable);


#ifdef	OHCI_USE_NPS
	WRITE_REG32 ((READ_REG32 (hci, HcRhDescriptorA) | RH_A_NPS) & ~RH_A_PSM,
		HcRhDescriptorA);
	WRITE_REG32 (hci, RH_HS_LPSC, HcRhStatus);
#endif	/* OHCI_USE_NPS */

	// POTPGT delay is bits 24-31, in 2 ms units.
	mdelay ((READ_REG32 (hci, HcRhDescriptorA) >> 23) & 0x1fe);
 
	rh_connect_rh (hci);
	
	return 0;
}


/*-------------------------------------------------------------------------*/

/* allocate HCI */

static hci_t * __devinit hc_alloc_hci (void)
{
	hci_t * hci;
	hcipriv_t * hp; 
	
	struct usb_bus * bus;

	hci = (hci_t *) kmalloc (sizeof (hci_t), GFP_KERNEL);
	if (!hci)
		return NULL;

	memset (hci, 0, sizeof (hci_t));

	hp = &hci->hp;
	
	hp->irq = -1;
	hp->hcport = 0;
	hp->hcvirtual = 0;
 
	INIT_LIST_HEAD (&hci->hci_hcd_list);
	list_add (&hci->hci_hcd_list, &hci_hcd_list);
	init_waitqueue_head (&hci->waitq);

	INIT_LIST_HEAD (&hci->ctrl_list);
	INIT_LIST_HEAD (&hci->bulk_list);
	INIT_LIST_HEAD (&hci->iso_list);
	INIT_LIST_HEAD (&hci->intr_list);
	INIT_LIST_HEAD (&hci->del_list);

	bus = usb_alloc_bus (&hci_device_operations);
	if (!bus) {
		kfree (hci);
		return NULL;
	}

	hci->bus = bus;
	bus->bus_name = "isp116x";
	bus->hcpriv = (void *) hci;

	return hci;
} 


/*-------------------------------------------------------------------------*/

/* De-allocate all resources.. */

static void hc_release_hci (hci_t * hci)
{	
	hcipriv_t * hp = &hci->hp; 

	/* disconnect all devices */    
	if (hci->bus->root_hub)
		usb_disconnect (&hci->bus->root_hub);

	if (hp->hcport > 0) {
		WRITE_REG16 (hci, 0, HcHardwareConfiguration);
		WRITE_REG16 (hci, 0, HcDMAConfiguration);
		WRITE_REG16 (hci, 0, HcuPInterruptEnable);
	}

	hc_reset (hci);
	
	if (hp->tl)
		kfree (hp->tl);

	if (hp->hcvirtual) {
		iounmap(hp->hcvirtual);
		hp->hcvirtual = 0;
	}
		
	if (hp->hcport > 0) {
		release_mem_region (hp->hcport, 4);
		hp->hcport = 0;
	}

	if (hp->irq >= 0) {
		free_irq (hp->irq, hci);
		hp->irq = -1;
	}

	usb_deregister_bus (hci->bus);
	usb_free_bus (hci->bus);

	list_del (&hci->hci_hcd_list);
	INIT_LIST_HEAD (&hci->hci_hcd_list);
       
	kfree (hci);
}

/*-------------------------------------------------------------------------*/

/* Increment the module usage count, start the control thread and
 * return success. */

 
static int __devinit hc_found_hci (int addr, int irq, int dma)
{
	hci_t * hci;
	hcipriv_t * hp; 

	printk("hc_found_hci: addr=%x\n", addr);

	hci = hc_alloc_hci ();
	if (!hci) {
		return -ENOMEM;
	}

	hp = &hci->hp;
	
	if (!request_mem_region (addr, 4, "ISP116x USB HOST")) {
		err ("request address %d-%d failed", addr, addr+4);
		hc_release_hci (hci);
		return -EBUSY;	
	}

	hp->hcport = addr;

	hp->hcvirtual = ioremap(addr, 4);
	if (! hp->hcvirtual) {
		err("ioremap() failed");
		hc_release_hci(hci);
		return -EBUSY;
	}

	if ((READ_REG16(hci, HcChipID) & 0xff00) != 0x6100) {
		hc_release_hci (hci);
		return -ENODEV;
	}
	
	if (hc_reset (hci) < 0) {
		hc_release_hci (hci);
		return -ENODEV;
	}
	
	if (hc_alloc_trans_buffer (hci)) {
		hc_release_hci (hci);
		return -ENOMEM;
	}

	printk(KERN_INFO __FILE__ ": USB ISP116x at %x IRQ %d Rev. %x ChipID: %x\n",
		addr, irq, READ_REG32(hci, HcRevision), READ_REG16(hci, HcChipID));

	/* FIXME this is a second HC reset; why?? */
	WRITE_REG32 (hci, hp->hc_control = OHCI_USB_RESET, HcControl);
	wait_ms (1000);

	usb_register_bus (hci->bus);
	
	if (request_irq (irq, hc_interrupt, 0,
			"ISP116x", hci) != 0) {
		err ("request interrupt %d failed", irq);
		hc_release_hci (hci);
		return -EBUSY;
	}
	hp->irq = irq;

	if (hc_start (hci) < 0) {
		err ("can't start usb-%x", addr);
		hc_release_hci (hci);
		return -EBUSY;
	}

	pm_hci = hci;

	return 0;
}


/*-------------------------------------------------------------------------*/

static int __init hci_hcd_init (void) 
{
	int ret;

	ret = hc_found_hci(hcport, irq, 0);   

#ifdef CONFIG_ARCTIC2 /* MVL-CEE */
        hc_isp_ldm_register();
#endif                /* MVL-CEE */

	return ret;
}

/*-------------------------------------------------------------------------*/

static void __exit hci_hcd_cleanup (void) 
{
	struct list_head *  hci_l;
	hci_t * hci;
	for (hci_l = hci_hcd_list.next; hci_l != &hci_hcd_list;) {
		hci = list_entry (hci_l, hci_t, hci_hcd_list);
		hci_l = hci_l->next;
		hc_release_hci(hci);
	}	

#ifdef CONFIG_ARCTIC2 /* MVL-CEE */
       hc_isp_ldm_unregister();
#endif                /* MVL-CEE */

}

module_init (hci_hcd_init);
module_exit (hci_hcd_cleanup);

MODULE_AUTHOR ("Roman Weissgaerber <weissg@vienna.at>");
MODULE_DESCRIPTION ("USB ISP116x Host Controller Driver");
