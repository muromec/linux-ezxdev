/******************************************************************************

	usb-mx2hcd.c
	driver for Motorola Dragonball MX2 on-chip USB Host (part of USB OTG)
	Reference manual used: MX2: IMX21RM/D Rev. 0.5, 07/25/2003

	Copyright (c) 2004 MontaVista Software, Inc. <source@mvista.com>
	-----------------------------------------------------------------------
	virtual root hub code is almost unchanged from usb-ohci.c "ver. 5.3":
	"URB OHCI HCD (Host Controller Driver) for USB."

	"(C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>"
	"(C) Copyright 2000-2001 David Brownell <dbrownell@users.sourceforge.net>"

	"[ Initialisation is based on Linus'  ]"
	"[ uhci code and gregs ohci fragments ]"
	"[ (C) Copyright 1999 Linus Torvalds  ]"
	"[ (C) Copyright 1999 Gregory P. Smith]"
	-----------------------------------------------------------------------
	URB scheduling is based on hc_simple.c:
	"simple generic USB HCD frontend Version 0.9.5 (10/28/2001)"
	"for embedded HCs (SL811HS)"
	-----------------------------------------------------------------------

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; either version 2
	of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

********************************************************************************/

#include "usb-mx2hcd.h"

/***************************************************************************
 * URB queueing:
 *
 * For each type of transfer (INTR, BULK, ISO, CTRL) there is a list of
 * active URBs.
 * (hci->intr_list, hci->bulk_list, hci->iso_list, hci->ctrl_list)
 * For every endpoint the head URB of the queued URBs is linked to one of
 * those lists.
 *
 * The rest of the queued URBs of an endpoint are linked into a
 * private URB list for each endpoint. (hci_dev->ed [endpoint_io].urb_queue)
 * hci_dev->ed [endpoint_io].pipe_head .. points to the head URB which is
 * in one of the active URB lists.
 *
 * The index of an endpoint consists of its number and its direction.
 *
 * The state of an intr and iso URB is 0.
 * For ctrl URBs the states are US_CTRL_SETUP, US_CTRL_DATA, US_CTRL_ACK
 * Bulk URBs states are US_BULK and US_BULK0 (with 0-len packet)
 *
 **************************************************************************/

/***************************************************************************
 * URB timeout handler
 **************************************************************************/
static void
qu_urb_timeout(unsigned long lurb)
{
  struct urb *urb = (struct urb *) lurb;
  dbg("urb timeout");
  urb->transfer_flags |= USB_TIMEOUT_KILLED;
  hci_unlink_urb(urb);
}

/***************************************************************************
 * Get an index for local ED#-based URB queue
 **************************************************************************/
static inline int
qu_pipeindex(__u32 pipe)
{
  return (usb_pipeendpoint(pipe) << 1) | (usb_pipecontrol(pipe) ? 0 :
					  usb_pipeout(pipe));
}

/***************************************************************************
 * Save local control or bulk URB state
 * in unused bits of the URB pipe descriptor
 **************************************************************************/
static inline void
qu_seturbstate(struct urb *urb, int state)
{
  urb->pipe &= ~0x1f;
  urb->pipe |= state & 0x1f;
}

/***************************************************************************
 * Get local control or bulk URB state
 **************************************************************************/
static inline int
qu_urbstate(struct urb *urb)
{
  return urb->pipe & 0x1f;
}

/***************************************************************************
 * This function adds the urb to the appropriate active urb list and set
 * the urb state.
 *
 * There are four active lists: isochoronous list, interrupt list,
 * control list, and bulk list.
 **************************************************************************/
static inline void
qu_queue_active_urb(hci_t * hci, struct urb *urb, epd_t * ed)
{
  int urb_state = 0;
  if (++hci->active_urbs == 1)
    hc_start_int();
  switch (usb_pipetype(urb->pipe)) {
  case PIPE_CONTROL:
    list_add(&urb->urb_list, &hci->ctrl_list);
    urb_state = US_CTRL_SETUP;
    break;

  case PIPE_BULK:
    list_add(&urb->urb_list, &hci->bulk_list);
    urb_state = US_BULK;
    break;

  case PIPE_INTERRUPT:
    list_add(&urb->urb_list, &hci->intr_list);
    break;

  case PIPE_ISOCHRONOUS:
    list_add(&urb->urb_list, &hci->iso_list);
    break;
  }

#ifdef HC_URB_TIMEOUT
  if (urb->timeout) {
    ed->timeout.data = (unsigned long) urb;
    ed->timeout.expires = urb->timeout + jiffies;
    ed->timeout.function = qu_urb_timeout;
    add_timer(&ed->timeout);
  }
#endif

  qu_seturbstate(urb, urb_state);
}

/***************************************************************************
 * If the urb was active, add the next urb waiting from the proper local ED
 *   list to the proper local active list and return 0
 * If the urb was not active, return -1
 **************************************************************************/
static int
qu_next_urb(hci_t * hci, struct urb *urb)
{
  struct hci_device *hci_dev = usb_to_hci(urb->dev);
  epd_t *ed = &hci_dev->ed[qu_pipeindex(urb->pipe)];

  if (ed->pipe_head == urb) {	/*the urb was active */

    if (!list_empty(&ed->urb_queue)) {
      urb = list_entry(ed->urb_queue.next, struct urb, urb_list);
      list_del(&urb->urb_list);
      INIT_LIST_HEAD(&urb->urb_list);
      ed->pipe_head = urb;
      qu_queue_active_urb(hci, urb, ed);
    } else {
      ed->pipe_head = NULL;
    }
    return 0;
  }
  return -1;
}

/***************************************************************************
 * Initialize the urb status and length, then queue the urb.
 **************************************************************************/
static inline void
hcs_urb_queue(hci_t * hci, struct urb *urb)
{
  int i;
  struct hci_device *hci_dev = usb_to_hci(urb->dev);
  epd_t *ed = &hci_dev->ed[qu_pipeindex(urb->pipe)];

  if (usb_pipeisoc(urb->pipe)) {
    for (i = 0; i < urb->number_of_packets; i++) {
      urb->iso_frame_desc[i].actual_length = 0;
      urb->iso_frame_desc[i].status = -EXDEV;
    }
  }

  urb->status = -EINPROGRESS;
  urb->actual_length = 0;
  urb->error_count = 0;
  if (ed->pipe_head) {
    __list_add(&urb->urb_list, ed->urb_queue.prev, &(ed->urb_queue));
  } else {
    ed->pipe_head = urb;
    qu_queue_active_urb(hci, urb, ed);
  }
}

/***************************************************************************
 * A USB core handler. Submits either a virtual root-hub or regular URB
 **************************************************************************/
static int
hci_submit_urb(struct urb *urb)
{
  hci_t *hci;
  unsigned int pipe = urb->pipe;
  unsigned long flags;
  urb_priv_t *urb_priv;

  if (!urb->dev || !urb->dev->bus || urb->hcpriv)
    return -EINVAL;

#if 0				/*do we need it here or not? */
  if (usb_endpoint_halted(urb->dev, usb_pipeendpoint(pipe), usb_pipeout(pipe))) {
    return -EPIPE;
  }
#endif

  hci = (hci_t *) urb->dev->bus->hcpriv;

  /* a request to the virtual root hub */

  if (usb_pipedevice(pipe) == hci->rh.devnum) {
#ifdef DEBUG
    urb_print(urb, "SUB-RH", usb_pipein(pipe));
#endif
    usb_inc_dev_use(urb->dev);
    return rh_submit_urb(urb);
  }

  /* allocate the private part of the URB */
  urb_priv =
      kmalloc(sizeof (urb_priv_t), in_interrupt()? GFP_ATOMIC : GFP_KERNEL);
  if (!urb_priv) {
    return -ENOMEM;
  }
  memset(urb_priv, 0, sizeof (urb_priv_t));
  urb_priv->etd_num = -1;
  urb_priv->etd_num1 = -1;

  urb->hcpriv = urb_priv;
  init_completion(&urb_priv->done);

  usb_inc_dev_use(urb->dev);
#ifdef DEBUG
  urb_print(urb, "SUB", !usb_pipecontrol(urb->pipe));
#endif

  /* queue the URB to its endpoint-queue */
  mx2hcd_irq_save_disable(flags);
  hcs_urb_queue(hci, urb);
  mx2hcd_irq_restore(flags);

  return 0;

}

/***************************************************************************
 * A USB Core handler. Also used locally. Unlinks an URB
 * or sets an unlink timer.
 **************************************************************************/
static int
hci_unlink_urb(struct urb *urb)
{
  unsigned long flags;
  hci_t *hci;
  urb_priv_t *urb_priv;

  mx2hcd_irq_save_disable(flags);

  if (!urb) {			/* just to be sure */
    mx2hcd_irq_restore(flags);
    dbg("cannot unlink - no urb");
    return -EINVAL;
  }

  if (!urb->dev || !urb->dev->bus) {
    mx2hcd_irq_restore(flags);
    dbg("cannot unlink - no device");
    return -ENODEV;
  }

  hci = (hci_t *) urb->dev->bus->hcpriv;
  urb_priv = urb->hcpriv;

  /* a request to the virtual root hub */
  if (usb_pipedevice(urb->pipe) == hci->rh.devnum) {
    int ret = rh_unlink_urb(urb);
    mx2hcd_irq_restore(flags);
    return ret;

  }

  if (!urb_priv) {
    mx2hcd_irq_restore(flags);
    dbg("cannot unlink - urb is unlinked already or was not submitted");
    return -EINVAL;		/*urb was not submitted */
  }

  if ((urb_priv->unlinked) && !(urb->transfer_flags & USB_ASYNC_UNLINK)) {
    /* urb is being sync unlinked already */
    complete(&urb_priv->done);
    mx2hcd_irq_restore(flags);
    dbg("urb is being sync unlinked already");
    return -EINPROGRESS;
  }

  if (!((urb_priv->unlinked)
	|| (urb->transfer_flags & USB_TIMEOUT_KILLED) || (hci->disabled))) {

    /*an URB can be unlinked instantaneously. However, usbserial, for example,
       unlinks URBs without waiting for their completion, thus loosing the last byte
       of output data. Lets give each URB one last chance
       to process data before unlinking them. */

    if (urb->transfer_flags & USB_ASYNC_UNLINK) {
      urb->status = -ECONNRESET;
    } else {
      if (in_interrupt()) {
	mx2hcd_irq_restore(flags);
	return -EWOULDBLOCK;
      }

      urb->status = -ENOENT;
    }
    /*wait for urb completion */
    urb_priv->unlinked = 1;

    urb_priv->timeout.data = (unsigned long) urb;
    urb_priv->timeout.expires = MX2HCD_UNLINK_URB_TIMEOUT + jiffies;
    urb_priv->timeout.function = qu_urb_timeout;
    add_timer(&urb_priv->timeout);

    if (urb->transfer_flags & USB_ASYNC_UNLINK) {
      mx2hcd_irq_restore(flags);
      dbg("async unlink exit");
      return -EINPROGRESS;
    } else {
      mx2hcd_irq_restore(flags);
      dbg("sync unlink wait");
      wait_for_completion(&urb_priv->done);
      mx2hcd_irq_save_disable(flags);
    }

  }
  dbg("actual unlink: unlinked ?: %c, timeout ?: %c, disabled ?: %c",
      urb_priv->unlinked ? 'y' : 'n',
      (urb->transfer_flags & USB_TIMEOUT_KILLED) ? 'y' : 'n',
      hci->disabled ? 'y' : 'n');

  del_timer(&urb_priv->timeout);

#ifdef HC_URB_TIMEOUT
  if (urb->timeout)
    del_timer(&ed->timeout);
#endif

#ifdef DEBUG
  urb_print(urb, "UNLINK", 0);
#endif

  /*terminate all on-going URB transactions */
  hc_abort_transfer(hci, urb_priv->etd_num);
  hc_abort_transfer(hci, urb_priv->etd_num1);

  if (!list_empty(&urb->urb_list)) {
    /*urb is still in one of local queues */
    list_del(&urb->urb_list);
    INIT_LIST_HEAD(&urb->urb_list);
  }

  /*if this urb was in one of active queues,
     put the next one from the proper ep queue */
  if (!qu_next_urb(hci, urb)) {
    /*this urb was in one of active queues. Not any more */
    if (!--hci->active_urbs)
      hc_stop_int();
  }

  kfree(urb_priv);
  urb->hcpriv = NULL;

  usb_dec_dev_use(urb->dev);
  urb->dev = NULL;

  if (urb->transfer_flags & (USB_ASYNC_UNLINK | USB_TIMEOUT_KILLED)) {
    urb->transfer_flags &= ~USB_TIMEOUT_KILLED;	/*in case this field is reused */
    urb->status = -ECONNRESET;
  } else {
    urb->status = -ENOENT;
  }

  if (urb->complete)
    urb->complete(urb);

  mx2hcd_irq_restore(flags);

  return 0;
}

/***************************************************************************
 * A USB Core handler. Allocate private data space for the usb device and
 * initialize local endpoint queues.
 **************************************************************************/
static int
hci_alloc_dev(struct usb_device *usb_dev)
{
  struct hci_device *dev;
  int i;

  dev = kmalloc(sizeof (*dev), in_interrupt()? GFP_ATOMIC : GFP_KERNEL);
  if (!dev)
    return -ENOMEM;

  memset(dev, 0, sizeof (*dev));

  for (i = 0; i < 32; i++) {
    INIT_LIST_HEAD(&(dev->ed[i].urb_queue));
    dev->ed[i].pipe_head = NULL;
  }

  usb_dev->hcpriv = dev;
  dbg("alloc dev");

  return 0;
}

/***************************************************************************
 * A USB Core handler. Remove private data space for the usb device.
 * Try to scan and unlink all URBs for the device (just in case)
 **************************************************************************/
static int
hci_free_dev(struct usb_device *usb_dev)
{
  struct hci_device *dev = usb_dev->hcpriv;
  struct urb *urb;
  int i;
  unsigned long flags;

  dbg("free dev");
  /*remove all pending transactions for this device,
  in case upper level fails to do so */
  if (dev) {
    for (i = 0; i < 32; i++) {
      mx2hcd_irq_save_disable(flags);
      while (!list_empty(&(dev->ed[i].urb_queue))) {
	urb = list_entry(dev->ed[i].urb_queue.next, struct urb, urb_list);
	list_del(&urb->urb_list);
	dbg("unlink form free dev");
	qu_urb_timeout((unsigned long) urb);	/*force immediate urb unlink */
      }

      list_del(&(dev->ed[i].urb_queue));
      urb = dev->ed[i].pipe_head;
      if (urb) {		/*this active urb is not complete */
	dbg("unlink active urb form free dev");
	qu_urb_timeout((unsigned long) urb);	/*force immediate urb unlink */
      }
      mx2hcd_irq_restore(flags);
    }
  }

  if (usb_dev->hcpriv)
    kfree(usb_dev->hcpriv);

  usb_dev->hcpriv = NULL;
  return 0;
}

/***************************************************************************
 * A USB Core handler. Get current frame number
 **************************************************************************/
static int
hci_get_current_frame_number(struct usb_device *usb_dev)
{
  return OTG_HOST_FRM_NUM & 0xffff;
}

/***************************************************************************
 * Links to USB core handlers are set here
 **************************************************************************/
static struct usb_operations hci_device_operations = {
  allocate:hci_alloc_dev,
  deallocate:hci_free_dev,
  get_frame_number:hci_get_current_frame_number,
  submit_urb:hci_submit_urb,
  unlink_urb:hci_unlink_urb,
};

/***************************************************************************
 * Try to start an URB stransaction
 **************************************************************************/
static int
sh_add_packet(hci_t * hci, struct urb *urb)
{
#ifdef DEBUG
  urb_print(urb, "ADD-PCT", !usb_pipecontrol(urb->pipe));
#endif
  if (usb_pipeisoc(urb->pipe)) {
    return hc_add_iso_trans(hci, urb);
  } else
    return hc_add_noniso_trans(hci, urb, qu_urbstate(urb));

}

/***************************************************************************
 * Scan through the active URB list and try to start transactions until the
 * first failure. Remove all succesfully started URBs from the list.
 **************************************************************************/
static int
sh_scan_urb_list(hci_t * hci, struct list_head *list_lh)
{
  struct urb *urb;

  while (!list_empty(list_lh)) {
    urb = list_entry(list_lh->next, struct urb, urb_list);
    if (urb == NULL)
      return 0;

    if (sh_add_packet(hci, urb) < 0) {
      return -1;
    } else {
      list_del(&urb->urb_list);
      INIT_LIST_HEAD(&urb->urb_list);
    }

  }
  return 0;
}

/***************************************************************************
 * Scan through all of the the active URB lists and try to start
 * transactions until the first failure.
 * Remove all succesfully started URBs from the list.
 **************************************************************************/
static void
sh_schedule_trans(hci_t * hci)
{
  int res = 0;

  if (!list_empty(&hci->intr_list)) {
    res = sh_scan_urb_list(hci, &hci->intr_list);
  }

  if (!list_empty(&hci->iso_list) && res >= 0) {
    res = sh_scan_urb_list(hci, &hci->iso_list);
  }

  if (!list_empty(&hci->ctrl_list) && res >= 0) {
    res = sh_scan_urb_list(hci, &hci->ctrl_list);
  }

  if (!list_empty(&hci->bulk_list) && res >= 0) {
    sh_scan_urb_list(hci, &hci->bulk_list);
  }
}

/***************************************************************************
 * Part of the Host Controller interrupt handler.
 * Scan through all ETD done states. Submit another URB transaction if
 * necessary, or report URB completion.
 **************************************************************************/
static void
sh_done_list(hci_t * hci)
{
  int actbytes = 0;
  int etd_done = 1;
  int cc;
  struct urb *urb;
  int urb_state = 0;
  int trans;
  u32 iso_index = 0;
  urb_priv_t *urb_priv;

  for (trans = 0; trans < MX2HCD_MAX_ETD; trans++) {

    urb = hc_parse_trans(hci, trans, &actbytes, &cc);
    if (!urb)
      continue;			/*either not ready or not initiated */

    urb->error_count = 0;
    urb_priv = urb->hcpriv;

    if (!(urb->transfer_flags & USB_DISABLE_SPD)
	&& (cc == TD_DATAUNDERRUN))
      cc = TD_CC_NOERROR;

    if (cc == TD_CC_STALL) {
      usb_endpoint_halt(urb->dev,
			usb_pipeendpoint(urb->pipe), usb_pipeout(urb->pipe));
    }

    etd_done = (cc_to_error[cc] != USB_ST_NOERROR);	/*stop if error */
    switch (usb_pipetype(urb->pipe)) {

    case PIPE_ISOCHRONOUS:
      {
	struct urb *urbt;
	int i;
	u32 framenum;
	u32 iso_index1;

	urb->actual_length += actbytes;

	if (urb_priv->unlinked)
	  etd_done = 1;

	if (!etd_done) {

	  if (trans == urb_priv->etd_num)
	    iso_index = urb_priv->iso_index;
	  else
	    iso_index = urb_priv->iso_index1;

	  framenum = OTG_HOST_FRM_NUM & 0xffff;
	  iso_index1 = urb->start_frame + iso_index;
	  if ((((iso_index1 + 1) & 0xffff) != framenum)
	      && ((iso_index1 & 0xffff) != framenum)) {
	    /*something delayed us, restore the pace.
             Sometimes we may be called here on the same frame */

	    if (framenum < urb->start_frame)
	      framenum += 0x10000;
	    iso_index1 = framenum - 1 - urb->start_frame;
	    dbg("iso delayed %d frames", iso_index1 - iso_index);
	    iso_index = iso_index1;

	  }

	  if ((iso_index + 1) >= urb->number_of_packets) {
	    /*processing the last packet */
	    for (urbt = urb->next; urbt && (urbt != urb); urbt = urbt->next) ;
	    if (urbt) {
	      /*need to requeue the ISO urb. Sending the second packet */

	      urb->status = cc_to_error[cc];
	      if (urb->complete) {
		urb->complete(urb);
	      }
#ifdef DEBUG
	      urb_print(urb, "RET", 0);
#endif

	      urb->actual_length = 0;
	      urb->status = USB_ST_URB_PENDING;
	      urb->start_frame = (urb->start_frame + urb->number_of_packets)
		  & 0xffff;

	      for (i = 0; i < urb->number_of_packets; i++) {
		urb->iso_frame_desc[i].actual_length = 0;
		urb->iso_frame_desc[i].status = -EXDEV;
	      }

	      if ((urb->number_of_packets > 1)
		  && ((urb_priv->iso_index == 0)
		      || (urb_priv->iso_index1 == 0)))
		hc_next_iso_trans(hci, urb, 1, trans);
	      else
		hc_next_iso_trans(hci, urb, 0, trans);

	    } else {
	      etd_done = 1;
	    }
	  } else if ((iso_index + 2) >= urb->number_of_packets) {
	    /*processing the next to last packet */
	    for (urbt = urb->next; urbt && (urbt != urb); urbt = urbt->next) ;
	    if (urbt) {
	      /*Need to requeue ISO. Send the first packet */
	      /*iso_index will be converted to 0 inside */
	      hc_next_iso_trans(hci, urb, iso_index + 2, trans);
	    } else {
	      /*clear up unused resources */
	      hc_abort_transfer(hci, trans);
	      /*let the next urb waiting start being processed ahead of time */
	      qu_next_urb(hci, urb);
	    }
	  } else {
	    /*processing a regular packet */
	    hc_next_iso_trans(hci, urb, iso_index + 2, trans);
	  }
	}
	if (etd_done) {
	  hc_abort_transfer(hci, urb_priv->etd_num);
	  hc_abort_transfer(hci, urb_priv->etd_num1);
	}

	break;
      }

    case PIPE_CONTROL:
      urb_state = qu_urbstate(urb);

      if (urb_state == US_CTRL_SETUP) {
	if (urb->transfer_buffer_length > 0)
	  urb_state = US_CTRL_DATA;
	else
	  urb_state = US_CTRL_ACK;
      } else if (urb_state == US_CTRL_DATA) {
	urb->actual_length += actbytes;
	urb_state = US_CTRL_ACK;

      } else
	etd_done = 1;		/*US_CTRL_ACK */

      if (!etd_done) {
	qu_seturbstate(urb, urb_state);
	if (hc_add_noniso_trans(hci, urb, urb_state) < 0)
	  /*requeue in case of res. shortage */
	  list_add(&urb->urb_list, &hci->ctrl_list);
      }

      break;
    case PIPE_BULK:
      urb->actual_length += actbytes;

      if ((qu_urbstate(urb) == US_BULK)
	  && (urb->transfer_flags & USB_ZERO_PACKET)
	  && urb->transfer_buffer_length > 0
	  && ((urb->transfer_buffer_length %
	    usb_maxpacket(urb->dev, urb->pipe, usb_pipeout(urb->pipe))) == 0)) {
	dbg("processing bulk 0-packet");
	/*need a 0-packet */
	qu_seturbstate(urb, US_BULK0);

	/*requeue in case of res. shortage */
	if (hc_add_noniso_trans(hci, urb, US_BULK0) < 0)
	  list_add(&urb->urb_list, &hci->bulk_list);

      } else {
	etd_done = 1;
      }
      break;
    case PIPE_INTERRUPT:
      urb->actual_length += actbytes;
      if (!etd_done) {
	if (urb_priv->unlinked) {
	  complete(&urb_priv->done);
	} else {
	  urb->status = cc_to_error[cc];
#ifdef DEBUG
	  urb_print(urb, "RET", 0);
#endif
	  if (urb->complete) {
	    urb->complete(urb);	/* call complete */
	  }

	  urb->actual_length = 0;
	  urb->status = -EINPROGRESS;

	  /*requeue in case of res. shortage */
	  if (hc_add_noniso_trans(hci, urb, 0) < 0)
	    list_add(&urb->urb_list, &hci->intr_list);
	}
      }
      break;
    }

    if (etd_done) {

      if (urb_priv->unlinked) {
	complete(&urb_priv->done);
      } else {

#ifdef HC_URB_TIMEOUT
	if (urb->timeout)
	  del_timer(&ed->timeout);
#endif

	if (!--hci->active_urbs)
	  hc_stop_int();
	qu_next_urb(hci, urb);

	urb->status = cc_to_error[cc];
#ifdef DEBUG
	urb_print(urb, "RET", !usb_pipecontrol(urb->pipe));
#endif
	kfree(urb_priv);
	urb->hcpriv = NULL;

	usb_dec_dev_use(urb->dev);
	urb->dev = NULL;

	if (urb->complete) {
	  urb->complete(urb);	/* call complete */
	}

      }

    }

  }

}

/*-------------------------------------------------------------------------*
 * Virtual Root Hub 
 *-------------------------------------------------------------------------*/

/* Device descriptor */
static __u8 root_hub_dev_des[] = {
  0x12,				/*  __u8  bLength; */
  0x01,				/*  __u8  bDescriptorType; Device */
  0x10,				/*  __u16 bcdUSB; v1.1 */
  0x01,
  0x09,				/*  __u8  bDeviceClass; HUB_CLASSCODE */
  0x00,				/*  __u8  bDeviceSubClass; */
  0x00,				/*  __u8  bDeviceProtocol; */
  0x08,				/*  __u8  bMaxPacketSize0; 8 Bytes */
  0x00,				/*  __u16 idVendor; */
  0x00,
  0x00,				/*  __u16 idProduct; */
  0x00,
  0x00,				/*  __u16 bcdDevice; */
  0x00,
  0x00,				/*  __u8  iManufacturer; */
  0x02,				/*  __u8  iProduct; */
  0x01,				/*  __u8  iSerialNumber; */
  0x01				/*  __u8  bNumConfigurations; */
};

/* Configuration descriptor */
static __u8 root_hub_config_des[] = {
  0x09,				/*  __u8  bLength; */
  0x02,				/*  __u8  bDescriptorType; Configuration */
  0x19,				/*  __u16 wTotalLength; */
  0x00,
  0x01,				/*  __u8  bNumInterfaces; */
  0x01,				/*  __u8  bConfigurationValue; */
  0x00,				/*  __u8  iConfiguration; */
  0x40,				/*  __u8  bmAttributes; 
				   Bit 7: Bus-powered, 6: Self-powered, 5 Remote-wakwup, 4..0: resvd */
  0x00,				/*  __u8  MaxPower; */

  /* interface */
  0x09,				/*  __u8  if_bLength; */
  0x04,				/*  __u8  if_bDescriptorType; Interface */
  0x00,				/*  __u8  if_bInterfaceNumber; */
  0x00,				/*  __u8  if_bAlternateSetting; */
  0x01,				/*  __u8  if_bNumEndpoints; */
  0x09,				/*  __u8  if_bInterfaceClass; HUB_CLASSCODE */
  0x00,				/*  __u8  if_bInterfaceSubClass; */
  0x00,				/*  __u8  if_bInterfaceProtocol; */
  0x00,				/*  __u8  if_iInterface; */

  /* endpoint */
  0x07,				/*  __u8  ep_bLength; */
  0x05,				/*  __u8  ep_bDescriptorType; Endpoint */
  0x81,				/*  __u8  ep_bEndpointAddress; IN Endpoint 1 */
  0x03,				/*  __u8  ep_bmAttributes; Interrupt */
  0x02,				/*  __u16 ep_wMaxPacketSize; ((MAX_ROOT_PORTS + 1) / 8 */
  0x00,
  0xff				/*  __u8  ep_bInterval; 255 ms */
};

/* Hub class-specific descriptor is constructed dynamically */

/*-------------------------------------------------------------------------*/

/* prepare Interrupt pipe data; HUB INTERRUPT ENDPOINT */

static int
rh_send_irq(hci_t * hci, void *rh_data, int rh_len)
{
  int num_ports;
  int i;
  int ret;
  int len;

  __u8 data[8];

  num_ports = OTG_HOST_HUB_DESCA & RH_A_NDP;
  if (num_ports > MAX_ROOT_PORTS) {
    err("bogus NDP=%d", num_ports);
    /* retry later; "should not happen" */
    return 0;
  }
  *(__u8 *) data = (OTG_HOST_HUB_STAT & (RH_HS_LPSC | RH_HS_OCIC))
      ? 1 : 0;
  ret = *(__u8 *) data;

  for (i = 0; i < num_ports; i++) {
    *(__u8 *) (data + (i + 1) / 8) |=
	((OTG_HOST_PORT_STAT(i) &
	  (RH_PS_CSC | RH_PS_PESC | RH_PS_PSSC | RH_PS_OCIC | RH_PS_PRSC))
	 ? 1 : 0) << ((i + 1) % 8);
    ret += *(__u8 *) (data + (i + 1) / 8);
  }
  len = i / 8 + 1;

  if (ret > 0) {
#ifdef DEBUG
    printk("OTG_HOST_HUB_STAT %x\n", (int)OTG_HOST_HUB_STAT);
    printk("OTG_HOST_PORT1_STAT %x\n", (int)OTG_HOST_PORT1_STAT);
    printk("OTG_HOST_PORT2_STAT %x\n", (int)OTG_HOST_PORT2_STAT);
    printk("OTG_HOST_PORT3_STAT %x\n", (int)OTG_HOST_PORT3_STAT);
#endif
    memcpy(rh_data, data,
	   min_t(unsigned int, len,
		 min_t(unsigned int, rh_len, sizeof (data))));
    return len;
  }
  return 0;
}

/*-------------------------------------------------------------------------*/

/* Virtual Root Hub INTs are polled by this timer every "interval" ms */

static void
rh_int_timer_do(unsigned long ptr)
{
  int len;

  struct urb *urb = (struct urb *) ptr;
  hci_t *hci = urb->dev->bus->hcpriv;

  /* ignore timers firing during PM suspend, etc */
  if ((OTG_HOST_CTRL & HCI_CTRL_HCFS) != HCI_USB_OPER)
    goto out;

  if (hci->rh.send) {
    len = rh_send_irq(hci, urb->transfer_buffer, urb->transfer_buffer_length);
    if (len > 0) {
      urb->actual_length = len;
#ifdef DEBUG
      urb_print(urb, "RET-t(rh)", usb_pipeout(urb->pipe));
#endif
      if (urb->complete)
	urb->complete(urb);
    }
  }
out:
  rh_init_int_timer(urb);
}

/*-------------------------------------------------------------------------*/

/* Root Hub INTs are polled by this timer */

static int
rh_init_int_timer(struct urb *urb)
{
  hci_t *hci = urb->dev->bus->hcpriv;

  hci->rh.interval = urb->interval;
  init_timer(&hci->rh.rh_int_timer);
  hci->rh.rh_int_timer.function = rh_int_timer_do;
  hci->rh.rh_int_timer.data = (unsigned long) urb;
  hci->rh.rh_int_timer.expires =
      jiffies + (HZ * (urb->interval < 30 ? 30 : urb->interval)) / 1000;
  add_timer(&hci->rh.rh_int_timer);

  return 0;
}

/*-------------------------------------------------------------------------*/

#define OK(x) 			len = (x); break
#define WR_RH_STAT(x) 		OTG_HOST_HUB_STAT=(x)
#define WR_RH_PORTSTAT(x) 	OTG_HOST_PORT_STAT(wIndex-1)=(x)
#define RD_RH_STAT		OTG_HOST_HUB_STAT
#define RD_RH_PORTSTAT		OTG_HOST_PORT_STAT(wIndex-1)

/* request to virtual root hub */

static int
rh_submit_urb(struct urb *urb)
{
  struct usb_device *usb_dev = urb->dev;
  hci_t *hci = usb_dev->bus->hcpriv;
  unsigned int pipe = urb->pipe;
  struct usb_ctrlrequest *cmd = (struct usb_ctrlrequest *) urb->setup_packet;
  void *data = urb->transfer_buffer;
  int leni = urb->transfer_buffer_length;
  int len = 0;
  int status = TD_CC_NOERROR;

  __u32 datab[4];
  __u8 *data_buf = (__u8 *) datab;

  __u16 bmRType_bReq;
  __u16 wValue;
  __u16 wIndex;
  __u16 wLength;

  if (usb_pipeint(pipe)) {
    hci->rh.urb = urb;
    hci->rh.send = 1;
    hci->rh.interval = urb->interval;
    rh_init_int_timer(urb);
    urb->status = cc_to_error[TD_CC_NOERROR];

    return 0;
  }

  bmRType_bReq = cmd->bRequestType | (cmd->bRequest << 8);
  wValue = le16_to_cpu(cmd->wValue);
  wIndex = le16_to_cpu(cmd->wIndex);
  wLength = le16_to_cpu(cmd->wLength);

  switch (bmRType_bReq) {
    /* Request Destination:
       without flags: Device, 
       RH_INTERFACE: interface, 
       RH_ENDPOINT: endpoint,
       RH_CLASS means HUB here, 
       RH_OTHER | RH_CLASS  almost ever means HUB_PORT here 
     */

  case RH_GET_STATUS:
    *(__u16 *) data_buf = cpu_to_le16(1);
    OK(2);
  case RH_GET_STATUS | RH_INTERFACE:
    *(__u16 *) data_buf = cpu_to_le16(0);
    OK(2);
  case RH_GET_STATUS | RH_ENDPOINT:
    *(__u16 *) data_buf = cpu_to_le16(0);
    OK(2);
  case RH_GET_STATUS | RH_CLASS:
    *(__u32 *) data_buf = cpu_to_le32(RD_RH_STAT & ~(RH_HS_CRWE | RH_HS_DRWE));
    OK(4);
  case RH_GET_STATUS | RH_OTHER | RH_CLASS:
    *(__u32 *) data_buf = cpu_to_le32(RD_RH_PORTSTAT);
    OK(4);

  case RH_CLEAR_FEATURE | RH_ENDPOINT:
    switch (wValue) {
    case (RH_ENDPOINT_STALL):
      OK(0);
    }
    break;

  case RH_CLEAR_FEATURE | RH_CLASS:
    switch (wValue) {
    case RH_C_HUB_LOCAL_POWER:
      OK(0);
    case (RH_C_HUB_OVER_CURRENT):
      WR_RH_STAT(RH_HS_OCIC);
      OK(0);
    }
    break;

  case RH_CLEAR_FEATURE | RH_OTHER | RH_CLASS:
    switch (wValue) {
    case (RH_PORT_ENABLE):
      WR_RH_PORTSTAT(RH_PS_CCS);
      OK(0);
    case (RH_PORT_SUSPEND):
      WR_RH_PORTSTAT(RH_PS_POCI);
      OK(0);
    case (RH_PORT_POWER):
      WR_RH_PORTSTAT(RH_PS_LSDA);
      OK(0);
    case (RH_C_PORT_CONNECTION):
      WR_RH_PORTSTAT(RH_PS_CSC);
      OK(0);
    case (RH_C_PORT_ENABLE):
      WR_RH_PORTSTAT(RH_PS_PESC);
      OK(0);
    case (RH_C_PORT_SUSPEND):
      WR_RH_PORTSTAT(RH_PS_PSSC);
      OK(0);
    case (RH_C_PORT_OVER_CURRENT):
      WR_RH_PORTSTAT(RH_PS_OCIC);
      OK(0);
    case (RH_C_PORT_RESET):
      WR_RH_PORTSTAT(RH_PS_PRSC);
      OK(0);
    }
    break;

  case RH_SET_FEATURE | RH_OTHER | RH_CLASS:
    switch (wValue) {
    case (RH_PORT_SUSPEND):
      WR_RH_PORTSTAT(RH_PS_PSS);
      OK(0);
    case (RH_PORT_RESET):	/* BUG IN HUP CODE ******** */
      if (RD_RH_PORTSTAT & RH_PS_CCS)
	WR_RH_PORTSTAT(RH_PS_PRS);
      OK(0);
    case (RH_PORT_POWER):
      WR_RH_PORTSTAT(RH_PS_PPS);
      OK(0);
    case (RH_PORT_ENABLE):	/* BUG IN HUP CODE ******** */
      if (RD_RH_PORTSTAT & RH_PS_CCS)
	WR_RH_PORTSTAT(RH_PS_PES);
      OK(0);
    }
    break;

  case RH_SET_ADDRESS:
    hci->rh.devnum = wValue;
    OK(0);

  case RH_GET_DESCRIPTOR:
    switch ((wValue & 0xff00) >> 8) {
    case (0x01):		/* device descriptor */
      len = min_t(unsigned int,
		  leni,
		  min_t(unsigned int, sizeof (root_hub_dev_des), wLength));
      data_buf = root_hub_dev_des;
      OK(len);
    case (0x02):		/* configuration descriptor */
      len = min_t(unsigned int,
		  leni,
		  min_t(unsigned int, sizeof (root_hub_config_des), wLength));
      data_buf = root_hub_config_des;
      OK(len);
    case (0x03):		/* string descriptors */
      len = usb_root_hub_string(wValue & 0xff,
				OTG_HOST_BASE, "MX2HCI", data, wLength);
      if (len > 0) {
	data_buf = data;
	OK(min_t(int, leni, len));
      }
      /* else fallthrough */
    default:
      status = TD_CC_STALL;
    }
    break;

  case RH_GET_DESCRIPTOR | RH_CLASS:
    {
      __u32 temp = OTG_HOST_HUB_DESCA;

      data_buf[0] = 9;		/* min length; */
      data_buf[1] = 0x29;
      data_buf[2] = temp & RH_A_NDP;
      data_buf[3] = 0;
      if (temp & RH_A_PSM)	/* per-port power switching? */
	data_buf[3] |= 0x1;
      if (temp & RH_A_NOCP)	/* no overcurrent reporting? */
	data_buf[3] |= 0x10;
      else if (temp & RH_A_OCPM)	/* per-port overcurrent reporting? */
	data_buf[3] |= 0x8;

      datab[1] = 0;
      data_buf[5] = (temp & RH_A_POTPGT) >> 24;
      temp = OTG_HOST_HUB_DESCB;
      data_buf[7] = temp & RH_B_DR;
      if (data_buf[2] < 7) {
	data_buf[8] = 0xff;
      } else {
	data_buf[0] += 2;
	data_buf[8] = (temp & RH_B_DR) >> 8;
	data_buf[10] = data_buf[9] = 0xff;
      }

      len = min_t(unsigned int, leni,
		  min_t(unsigned int, data_buf[0], wLength));
      OK(len);
    }

  case RH_GET_CONFIGURATION:
    *(__u8 *) data_buf = 0x01;
    OK(1);

  case RH_SET_CONFIGURATION:
    WR_RH_STAT(0x10000);
    OK(0);

  default:
    dbg("unsupported root hub command");
    status = TD_CC_STALL;
  }

  len = min_t(int, len, leni);
  if (data != data_buf)
    memcpy(data, data_buf, len);
  urb->actual_length = len;
  urb->status = cc_to_error[status];

#ifdef DEBUG
  urb_print(urb, "RET(rh)", usb_pipeout(urb->pipe));
#endif

  urb->hcpriv = NULL;
  usb_dec_dev_use(usb_dev);
  urb->dev = NULL;
  if (urb->complete)
    urb->complete(urb);
  return 0;
}

/*-------------------------------------------------------------------------*/

static int
rh_unlink_urb(struct urb *urb)
{
  hci_t *hci = urb->dev->bus->hcpriv;

  if (hci->rh.urb == urb) {
    hci->rh.send = 0;
    del_timer(&hci->rh.rh_int_timer);
    hci->rh.urb = NULL;

    urb->hcpriv = NULL;
    usb_dec_dev_use(urb->dev);
    urb->dev = NULL;
    if (urb->transfer_flags & USB_ASYNC_UNLINK) {
      urb->status = -ECONNRESET;
    } else
      urb->status = -ENOENT;
    if (urb->complete)
      urb->complete(urb);
  }
  return 0;
}

/*-------------------------------------------------------------------------*
 * End of Virtual Root Hub code
 *-------------------------------------------------------------------------*/

/***************************************************************************
 * Setup a bulk, control or interrupt ETD. Configure DMA if necessary.
 * Trigger the processing of the ETD by the host controller. Return error
 * and not do anything in case there are not enough resources.
 **************************************************************************/
static int
hc_add_noniso_trans(hci_t * hci, struct urb *urb, int state)
{
  unsigned int pipe = urb->pipe;
  urb_priv_t *urb_priv = urb->hcpriv;
  int etd_num;
  int buf_addr;
  u32 etd_addr;
  u16 buf_size;
  u32 etd_mask;
  u8 *src_addr;
  u32 count;
  u8 dir;
  u8 bufround;
  u8 datatoggle;

  u16 maxpacket;

  u8 interval = 0;
  u8 relpolpos = 0;

  if (!(maxpacket = usb_maxpacket(urb->dev, pipe, usb_pipeout(pipe))))
    maxpacket = 8;

  if (usb_pipecontrol(pipe) && (state != US_CTRL_DATA)) {
    if (state == US_CTRL_SETUP) {
      dir = MX2HCD_DIR_SETUP;
      src_addr = (u8 *) urb->setup_packet;
      bufround = 0;
      count = 8;
      datatoggle = MX2HCD_TGL_DATA0;
    } else {
      dir = usb_pipeout(pipe) ? MX2HCD_DIR_IN : MX2HCD_DIR_OUT;
      src_addr = (u8 *) urb->transfer_buffer;
      bufround = 0;
      count = 0;
      datatoggle = MX2HCD_TGL_DATA1;
    }

  } else {
    dir = usb_pipeout(pipe) ? MX2HCD_DIR_OUT : MX2HCD_DIR_IN;
    bufround = (dir == MX2HCD_DIR_IN) ? 1 : 0;
    src_addr = (u8 *) urb->transfer_buffer;
    if (usb_pipebulk(pipe) && (state == US_BULK0))
      count = 0;
    else
      count = urb->transfer_buffer_length;
    if (usb_pipecontrol(pipe)) {
      datatoggle = MX2HCD_TGL_DATA1;
    } else {
      datatoggle = (usb_gettoggle(urb->dev,
				  usb_pipeendpoint(urb->
						   pipe),
				  usb_pipeout(urb->
					      pipe))) ?
	  MX2HCD_TGL_DATA1 : MX2HCD_TGL_DATA0;

    }
  }

  if (count > maxpacket)
    buf_size = maxpacket * 2;
  else
    buf_size = maxpacket;

  etd_num = mx2hcd_get_free_ETD_num(hci);
  if (etd_num < 0) {
    dbg("cannot get a free ETD for non-iso");
    return etd_num;
  }

/* allocate x and y buffer space at once*/
  buf_addr = mx2otg_alloc_DATA_mem(buf_size);
  if (buf_addr < 0) {
    dbg("not enough DATA memory for non-iso");
    return buf_addr;
  }

  etd_mask = 1 << etd_num;

  hci->etd_buf[etd_num] = 0;

#ifdef MX2HCD_USE_DIRECT_DMA
  if ((u32) src_addr & 0x3) {
#endif
    hci->etd_buf[etd_num] = kmalloc(count, GFP_ATOMIC | GFP_DMA);
    if (!hci->etd_buf[etd_num]) {
      dbg("not enough memory for non-iso DMA buffer, %d bytes needed", count);
      mx2otg_free_DATA_mem(buf_addr);
      return -ENOMEM;
    }
    if (dir != MX2HCD_DIR_IN) {
      memcpy(hci->etd_buf[etd_num], src_addr, count);
      consistent_sync(hci->etd_buf[etd_num], count, PCI_DMA_TODEVICE);
    } else {
      consistent_sync(hci->etd_buf[etd_num], count, PCI_DMA_FROMDEVICE);
    }

    src_addr = hci->etd_buf[etd_num];
#ifdef MX2HCD_USE_DIRECT_DMA
  } else {
    if (dir != MX2HCD_DIR_IN) {
      consistent_sync(src_addr, count, PCI_DMA_TODEVICE);
    } else {
      consistent_sync(src_addr, count, PCI_DMA_FROMDEVICE);
    }
  }
#endif

  if (usb_pipeint(pipe)) {
    interval = urb->interval;
    relpolpos = (OTG_HOST_FRM_NUM + 1) & 0xff;
  }

  hci->etd_urb[etd_num] = urb;
  urb_priv->etd_num = etd_num;



  etd_addr = OTG_ETD_BASE + (etd_num << 4);   /*set up word 0*/

  /*Normal USB mode is "do not stop on NAK" (bit 30 ==0)*/
  __REG32(etd_addr) = (u32) usb_pipedevice(pipe) |
      ((u32) usb_pipeendpoint(pipe) << 7) |
      ((u32) dir << 11) |
      ((u32) usb_pipeslow(pipe) << 13) |
      ((u32) fmt_urb_to_etd[usb_pipetype(pipe)] << 14) |
      ((u32) maxpacket << 16);

  etd_addr += 4;	/*set up word 1*/
  __REG32(etd_addr) =
      (((u32) buf_addr + (u32) maxpacket) << 16) | (u32) buf_addr;

  etd_addr += 4;	/*set up word 2*/

  /*delay interrupt option is disabled (bit 19-21 == 0)*/
  __REG32(etd_addr) = (u32) interval |
    ((u32) relpolpos << 8) |
    ((u32) dir << 16) |
    ((u32) bufround << 18) |
    ((u32) datatoggle << 22) |
    ((u32) 0xf << 28);	/*reset completion code */

  etd_addr += 4;	/*set up word 3*/

  /*X or Y buffer size is always == 1 maxpacket */
  __REG32(etd_addr) = ((u32) (maxpacket - 1) << 21) | (u32) count;

  /*clear etd done status */
  mx2otg_clear_toggling_bit(OTG_HOST_ETD_STAT, etd_mask);

  /*enable etd done interrupt */
  OTG_HOST_ETD_DONE |= etd_mask;

  mx2otg_clear_toggling_bit(OTG_HOST_XFILL_STAT, etd_mask);
  mx2otg_clear_toggling_bit(OTG_HOST_YFILL_STAT, etd_mask);

  if (count == 0) {

    if (dir != MX2HCD_DIR_IN) {
      /*need to set it even if count == 0 */
      mx2otg_set_toggling_bit(OTG_HOST_XFILL_STAT, etd_mask);
      mx2otg_set_toggling_bit(OTG_HOST_YFILL_STAT, etd_mask);
    }

  } else {

    mx2otg_set_toggling_bit(OTG_DMA_ETD_CH_CLR, etd_mask);

    mx2otg_clear_toggling_bit(OTG_HOST_XINT_STAT, etd_mask);
    mx2otg_clear_toggling_bit(OTG_HOST_YINT_STAT, etd_mask);

    OTG_DMA_ETD_MSA(etd_num) = virt_to_phys(src_addr);
    mx2otg_set_toggling_bit(OTG_DMA_ETD_EN, etd_mask);

  }

  hci->etd_len[etd_num] = count;

#ifdef DEBUG
  etd_dump(etd_num, "ADD");
#endif

#ifdef CONFIG_MX2TO1
  mx2otg_set_toggling_bit(OTG_HOST_ETD_EN, etd_mask);
#else
  OTG_HOST_ETD_EN = etd_mask;
#endif
  return 0;
}

/***************************************************************************
 * Terminate an on-going ETD processing, clear up all resources associated
 * with the ETD, and clear all the associated status flags
 **************************************************************************/
static void
hc_abort_transfer(hci_t * hci, int etd_num)
{
  struct urb *urb;
  urb_priv_t *urb_priv;
  u32 etd_addr;
  u16 xbufaddr;
  u32 etd_mask = 1 << etd_num;
  urb = hci->etd_urb[etd_num];
  urb_priv = urb->hcpriv;
  if (etd_num < 0)
    return;

#ifdef DEBUG
  etd_dump(etd_num, "UNL");
#endif
#ifdef CONFIG_MX2TO1
  mx2otg_clear_toggling_bit(OTG_HOST_ETD_EN, etd_mask);
#else
  OTG_HOST_ETD_EN_CLR = etd_mask;
#endif
  mx2otg_clear_toggling_bit(OTG_HOST_ETD_STAT, etd_mask);
  mx2otg_clear_toggling_bit(OTG_DMA_ETD_EN, etd_mask);

  mx2otg_set_toggling_bit(OTG_DMA_ETD_CH_CLR, etd_mask);

  mx2otg_clear_toggling_bit(OTG_HOST_XINT_STAT, etd_mask);
  mx2otg_clear_toggling_bit(OTG_HOST_YINT_STAT, etd_mask);

  mx2otg_clear_toggling_bit(OTG_HOST_XFILL_STAT, etd_mask);
  mx2otg_clear_toggling_bit(OTG_HOST_YFILL_STAT, etd_mask);

  etd_addr = OTG_ETD_BASE + (etd_num << 4) + 4;
  xbufaddr = __REG32(etd_addr) & 0xffff;
  mx2otg_free_DATA_mem(xbufaddr);

  __REG32(etd_addr - 4) = 0;
  __REG32(etd_addr) = 0;
  __REG32(etd_addr + 4) = 0;
  __REG32(etd_addr + 8) = 0;

  if (etd_num == urb_priv->etd_num) {
    urb_priv->etd_num = -1;
  } else {
    urb_priv->etd_num1 = -1;
  }

  hci->etd_urb[etd_num] = 0;

  if (hci->etd_buf[etd_num]) {
    kfree(hci->etd_buf[etd_num]);
  }

}

/***************************************************************************
 * Setup (an) isochronous ETD(s). Configure DMA if necessary.
 * Trigger the processing of the ETD by the host controller. Return error
 * and not do anything in case there are not enough resources.
 *
 * For isochronous transactions, 1 or 2 ETDs
 * and DATA memory for 1 or 2 packets are reserved and these
 * resources are freed only upon end of processing of all packets
 * in the ISO URB by calling hc_abort_transfer() for each ETD.
 *
 * Calls hc_next_iso_trans() to complete ETD setup and trigger ETD processing
 * by the host controller.
 **************************************************************************/
static void
hc_next_iso_trans(hci_t * hci, struct urb *urb, int iso_index, int etd_num)
{
  u8 *virt_buf_addr;
  u8 *src_addr;
  u16 pktlen;
  u32 etd_mask;
  u16 startfrm;
  urb_priv_t *urb_priv = urb->hcpriv;
  u32 buf_addr;
  u32 etd_addr;

  if (etd_num == urb_priv->etd_num)
    urb_priv->iso_index = iso_index;
  else
    urb_priv->iso_index1 = iso_index;

  etd_mask = 1 << etd_num;

  startfrm = (urb->start_frame + iso_index) & 0xffff;

  /*take care of repeating ISO URBs */
  if (iso_index >= urb->number_of_packets)
    iso_index -= urb->number_of_packets;

  pktlen = urb->iso_frame_desc[iso_index].length;

  /*get buffer address from word 1*/
  etd_addr = OTG_ETD_BASE + (etd_num << 4) + 4;
  buf_addr = __REG32(etd_addr);


  etd_addr += 4;	  /*set up word 2*/

  /*delay interrupt option is disabled (bit 19-21 == 0)*/
  /*always sending 1 packet (bit 24 == 0)*/
  __REG32(etd_addr) = (u32) startfrm |
      ((u32) 0xf << 28); /*reset completion code */ ;

  etd_addr += 4;	/*set up word 3*/
  __REG32(etd_addr) = ((u32) 0xf << 12) | (u32) pktlen;

  /*no need to use dma for ISO transfers*/

  /*clear etd done status */
  mx2otg_clear_toggling_bit(OTG_HOST_ETD_STAT, etd_mask);

  /*enable etd done interrupt */
  OTG_HOST_ETD_DONE |= etd_mask;

  mx2otg_clear_toggling_bit(OTG_HOST_XFILL_STAT, etd_mask);
  mx2otg_clear_toggling_bit(OTG_HOST_YFILL_STAT, etd_mask);

  src_addr = (u8 *) urb->transfer_buffer +
      urb->iso_frame_desc[iso_index].offset;
#ifdef MX2HCD_USE_DIRECT_ISO_DMA
  if (!((u32) src_addr & 0x3) && (pktlen > 0)) {
    consistent_sync(src_addr, pktlen,
		    usb_pipeout(urb->
				pipe) ? PCI_DMA_TODEVICE : PCI_DMA_FROMDEVICE);

    mx2otg_set_toggling_bit(OTG_DMA_ETD_CH_CLR, etd_mask);

    mx2otg_clear_toggling_bit(OTG_HOST_XINT_STAT, etd_mask);
    mx2otg_clear_toggling_bit(OTG_HOST_YINT_STAT, etd_mask);

    OTG_DMA_ETD_MSA(etd_num) = virt_to_phys(src_addr);
    mx2otg_set_toggling_bit(OTG_DMA_ETD_EN, etd_mask);
  } else {
#endif

    virt_buf_addr = (u8 *) phys_to_virt(OTG_DATA_BASE + buf_addr);
    if (usb_pipeout(urb->pipe)) {

      memcpy(virt_buf_addr, src_addr, pktlen);
      consistent_sync(virt_buf_addr, pktlen, PCI_DMA_TODEVICE);

      mx2otg_set_toggling_bit(OTG_HOST_XFILL_STAT, etd_mask);
    } else {
      consistent_sync(virt_buf_addr, pktlen, PCI_DMA_FROMDEVICE);
    }
#ifdef MX2HCD_USE_DIRECT_ISO_DMA
  }
#endif

#ifdef DEBUG
  etd_dump(etd_num, "ADDISO");
#endif

#ifdef CONFIG_MX2TO1
  mx2otg_set_toggling_bit(OTG_HOST_ETD_EN, etd_mask);
#else
  OTG_HOST_ETD_EN = etd_mask;
#endif
}

/***************************************************************************
 * Complete iso transaction setup and trigger processing of the iso ETD(s).
 **************************************************************************/
static inline void
set_iso_etd_word_0(int etd_num, struct urb *urb)
{
  unsigned int pipe = urb->pipe;
  __REG32(OTG_ETD_BASE + (etd_num << 4)) = (u32) usb_pipedevice(pipe) |
    ((u32) usb_pipeendpoint(pipe) << 7) |
    ((u32) (usb_pipeout(pipe) ? MX2HCD_DIR_OUT : MX2HCD_DIR_IN) << 11) |
    ((u32) usb_pipeslow(pipe) << 13) |
    ((u32) 0x1 << 14) |	/*set isochronous format */
    ((u32) usb_maxpacket(urb->dev, pipe, usb_pipeout(pipe)) << 16);
    /*Normal USB mode is "do not stop on NAK" (bit 30 == 0) */
}

static inline int
hc_add_iso_trans(hci_t * hci, struct urb *urb)
{
  int etd_num, etd_num1;
  int buf_addr, buf_addr1;
  urb_priv_t *urb_priv = urb->hcpriv;
  unsigned int pipe = urb->pipe;

  u16 maxpacket;

  if ((urb_priv->etd_num != -1) || (urb_priv->etd_num1 != -1))
    return 0;

  maxpacket = usb_maxpacket(urb->dev, pipe, usb_pipeout(pipe));

  etd_num = mx2hcd_get_free_ETD_num(hci);
  if (etd_num < 0) {
    dbg("cannot get a free ETD for iso");
    return etd_num;
  }

  hci->etd_urb[etd_num] = urb;	/*reserve this ETD */

  if (urb->number_of_packets > 1) {

    etd_num1 = mx2hcd_get_free_ETD_num(hci);
    if (etd_num1 < 0) {
      dbg("cannot get a free ETD for iso1");
      hci->etd_urb[etd_num] = 0;	/*free the reserved ETD */
      return etd_num1;
    }
  } else {
    etd_num1 = -1;
  }

  /* allocate x and y buffer space at once */
  buf_addr = mx2otg_alloc_DATA_mem(maxpacket);
  if (buf_addr < 0) {
    dbg("not enough DATA memory for iso");
    hci->etd_urb[etd_num] = 0;	/* free the reserved ETD */
    return buf_addr;
  }

  if (urb->number_of_packets > 1) {
    buf_addr1 = mx2otg_alloc_DATA_mem(maxpacket);
    if (buf_addr1 < 0) {
      dbg("not enough DATA memory for iso1");
      hci->etd_urb[etd_num] = 0;	/* free the reserved ETD */
      mx2otg_free_DATA_mem(buf_addr);
      return buf_addr1;
    }
  } else
    buf_addr1 = 0; /* this assignment is to remove a compiler warning */

  hci->etd_urb[etd_num1] = urb;	/* reserve the other ETD */
  hci->etd_buf[etd_num] = 0;
  hci->etd_buf[etd_num1] = 0;
  urb_priv->etd_num = etd_num;
  urb_priv->etd_num1 = etd_num1;

  if (urb->transfer_flags & USB_ISO_ASAP) {
    urb->start_frame = (OTG_HOST_FRM_NUM + 1) & 0xffff;
  }

  /*configure the first ETD*/

  set_iso_etd_word_0(etd_num, urb);	/*set up word 0*/

  /*set up word 1*/
  __REG32(OTG_ETD_BASE + (etd_num << 4) + 4) = (u32) buf_addr;

  hc_next_iso_trans(hci, urb, 0, etd_num);

  if (urb->number_of_packets > 1) {

    /*configure the second ETD*/

    set_iso_etd_word_0(etd_num1, urb);	    /*set up word 0*/

    /*set up word 1*/
    __REG32(OTG_ETD_BASE + (etd_num1 << 4) + 4) = (u32) buf_addr1;
    hc_next_iso_trans(hci, urb, 1, etd_num1);
  }

  return 0;
}

/***************************************************************************
 * If the ETD is done, parse all status data. Copy received data with
 * memcpy if necessary. Report number of bytes actually transferred and
 * completion code
 **************************************************************************/
static inline struct urb *
hc_parse_trans(hci_t * hci, int etd_num, int *actbytes, int *cc)
{
  u32 etd_mask = 1 << etd_num;
  urb_priv_t *urb_priv;
  unsigned int pipe;
  struct urb *urb;
  u32 etd_addr;
  u16 xbufaddr;

  int compl_code;
  u32 bytes_xfrd;
  int dir;

  if (!(OTG_HOST_ETD_STAT & etd_mask))
    return 0;
  urb = hci->etd_urb[etd_num];
  if (!urb)
    return 0;

  urb_priv = urb->hcpriv;
  pipe = urb->pipe;

  etd_addr = OTG_ETD_BASE + (etd_num << 4);
  dir = (__REG32(etd_addr) >> 11) & 0x3;

#ifdef DEBUG
  etd_dump(etd_num, "RCV");
#endif

#ifdef CONFIG_MX2TO1
  mx2otg_clear_toggling_bit(OTG_HOST_ETD_EN, etd_mask);
#else
  OTG_HOST_ETD_EN_CLR = etd_mask;
#endif

  mx2otg_clear_toggling_bit(OTG_HOST_ETD_STAT, etd_mask);
  mx2otg_clear_toggling_bit(OTG_DMA_ETD_EN, etd_mask);

  if (usb_pipeisoc(pipe)) {
    int iso_index;
    u8 *src_addr;

    if (etd_num == urb_priv->etd_num)
      iso_index = urb_priv->iso_index;
    else
      iso_index = urb_priv->iso_index1;

    etd_addr += 4;		/*move to word 1 */
    xbufaddr = __REG32(etd_addr) & 0xffff;

    etd_addr += 4;		/*move to word 2 */

    /*etd completion code */
    /*compl_code = (__REG32(etd_addr) >> 28) & 0xf; */

    etd_addr += 4;		/*move to word 3 */

    /*packet completion code */
    compl_code = (__REG32(etd_addr) >> 12) & 0xf;

    /*indicates the number of bytes not transferred */
    bytes_xfrd = __REG32(etd_addr) & 0x3ff;

    /*or may be not, if receiving data. May be it is just not being updated.
    Convert to the number of bytes transferred */
    if (usb_pipeout(pipe))
      bytes_xfrd = urb->iso_frame_desc[iso_index].length - bytes_xfrd;

    if (usb_pipein(pipe)) {
      src_addr = (u8 *) urb->transfer_buffer +
	  urb->iso_frame_desc[iso_index].offset;

#ifdef MX2HCD_USE_DIRECT_ISO_DMA
      if ((u32) src_addr & 0x3) {
#endif
	memcpy(src_addr,
	       (u8 *) phys_to_virt(OTG_DATA_BASE + xbufaddr), bytes_xfrd);
#ifdef MX2HCD_USE_DIRECT_ISO_DMA
      }
#endif
      mx2otg_clear_toggling_bit(OTG_HOST_XFILL_STAT, etd_mask);
    }

    urb->iso_frame_desc[iso_index].actual_length = bytes_xfrd;
    urb->iso_frame_desc[iso_index].status = cc_to_error[compl_code];

    /* freeing of ETD resources is done outside this function when all packets are processed */

  } else {			/* BULK, INT, CONTROL DATA */

    /*save toggle carry */
    usb_settoggle(urb->dev, usb_pipeendpoint(urb->pipe),
      usb_pipeout(urb->pipe), (__REG32(etd_addr) >> 28) & 0x1);
    etd_addr += 4;		/*move to word 1 */
    xbufaddr = __REG32(etd_addr) & 0xffff;

    etd_addr += 4;		/*move to word 2 */

    /*etd completion code */
    compl_code = (__REG32(etd_addr) >> 28) & 0xf;

    etd_addr += 4;		/*move to word 3 */

    /*indicates the number of bytes left to be transferred */
    bytes_xfrd = __REG32(etd_addr) & 0x1fffff;

    /*convert to the number of bytes transfered */
    bytes_xfrd = hci->etd_len[etd_num] - bytes_xfrd;

    if (dir == MX2HCD_DIR_IN) {
      /*if data input */

      mx2otg_clear_toggling_bit(OTG_HOST_XFILL_STAT, etd_mask);
      mx2otg_clear_toggling_bit(OTG_HOST_YFILL_STAT, etd_mask);

      if (hci->etd_buf[etd_num]) {
	memcpy((char *) urb->transfer_buffer,
	       hci->etd_buf[etd_num], bytes_xfrd);
      }

    }

    if (hci->etd_buf[etd_num]) {
      kfree(hci->etd_buf[etd_num]);
    }

    mx2otg_free_DATA_mem(xbufaddr);
    hci->etd_urb[etd_num] = 0;
    urb_priv->etd_num = -1;
  }
  *cc = compl_code;
  *actbytes = bytes_xfrd;

  return urb;
}

/***************************************************************************
 * enable host controller interrupts
 **************************************************************************/
static inline void
hc_start_int(void)
{
  OTG_HOST_SINT_STEN = HCI_INTR_UE | HCI_INTR_SO | HCI_INTR_WDH | HCI_INTR_SF;
}

/***************************************************************************
 * disable host controller interrupts
 **************************************************************************/
static inline void
hc_stop_int(void)
{
  OTG_HOST_SINT_STEN = 0;
}

/***************************************************************************
 * host controller interrupt handler
 **************************************************************************/
static void
hc_interrupt(int irq, void *__hci, struct pt_regs *r)
{
  hci_t *hci = __hci;
  int ints;

  ints = OTG_HOST_SINT_STAT;

  if (ints & HCI_INTR_UE) {
    err("MX2 HCI Unrecoverable Error");
  }
  if (ints & HCI_INTR_SO) {
    err("USB Schedule overrun");
  }

  if (ints & HCI_INTR_WDH) {
    /* USB Done interrupt occurred */
    sh_done_list(hci);
  }

  if (ints & HCI_INTR_SF) {
    /* Look for new transactions scheduled every 1 ms */
    sh_schedule_trans(hci);
  }

  OTG_HOST_SINT_STAT = ints;
}

/*-------------------------------------------------------------------------*/

void
mx2otg_host_dma_error(void)
{
  u32 err_stat;
  err_stat = OTG_DMA_ETD_ERR;
  err("DMA error occurred, status: %x", err_stat);

  /*just disable the etds */
#ifdef CONFIG_MX2TO1
  mx2otg_clear_toggling_bit(OTG_HOST_ETD_EN, (OTG_HOST_ETD_EN & err_stat));
#else
  OTG_HOST_ETD_EN_CLR = err_stat;
#endif

  OTG_DMA_ETD_ERR = err_stat;
}

int
mx2otg_host_enable(void)
{
  struct usb_device *usb_dev;
  hci_t *hci = mx2hci_struct;

  /*##############CONFIGURING USB HOST UPPER LEVEL################# */
  hci->disabled = 0;

  hci->bus = usb_alloc_bus(&hci_device_operations);
  if (!hci->bus) {
    err("cannot allocate usb bus");
    return -ENOMEM;
  }

  hci->bus->bus_name = "usb-mx2hci";
  hci->bus->hcpriv = (void *) hci;
  usb_register_bus(hci->bus);

  OTG_CORE_CINT_STEN |= OTG_CORE_HCINTEN;	/*enable host interrupt */

  /* start controller operations */
  OTG_HOST_CTRL = HCI_USB_OPER;

  /* connect the virtual root hub */
  hci->rh.devnum = 0;
  usb_dev = usb_alloc_dev(NULL, hci->bus);
  if (!usb_dev) {
    err("cannot allocate root hub");
    return -ENOMEM;
  }
  hci->bus->root_hub = usb_dev;

  usb_connect(usb_dev);
  if (usb_new_device(usb_dev) != 0) {
    err("cannot connect root hub");
    return -ENODEV;
  }
  return 0;
}

void
mx2otg_host_disable(void)
{
  hci_t *hci = mx2hci_struct;

  hci->disabled = 1;

  /* disconnect all devices */
  if (hci->bus->root_hub) {
    usb_disconnect(&hci->bus->root_hub);
  }

  if (hci->bus) {
    if (hci->bus->busnum != -1)
      usb_deregister_bus(hci->bus);
    usb_free_bus(hci->bus);
  }

  OTG_CORE_CINT_STEN &= ~OTG_CORE_HCINTEN;	/*disable host interrupt */

  OTG_HOST_CTRL = 0;		/* put the host controller in the reset state */
}

/* module init*/
static int __init
mx2ads_usbhost_init(void)
{

  hci_t *hci;

  long tmp;

  dbg(MODULE_NAME " ver. 1.0 built on %s %s", __TIME__, __DATE__);

  hci = (hci_t *) kmalloc(sizeof (hci_t), GFP_KERNEL);
  if (!hci) {
    mx2ads_usbhost_exit();
    err("cannot get memory for local structures");
    return -ENOMEM;
  }

  memset(hci, 0, sizeof (hci_t));
  INIT_LIST_HEAD(&hci->ctrl_list);
  INIT_LIST_HEAD(&hci->bulk_list);
  INIT_LIST_HEAD(&hci->iso_list);
  INIT_LIST_HEAD(&hci->intr_list);
  init_waitqueue_head(&hci->waitq);

  mx2hci_struct = hci;

  OTG_CORE_CINT_STEN &= ~OTG_CORE_HCINTEN;	/*disable host interrupt */

  tmp = request_irq(INT_USBHOST, hc_interrupt, 0, MODULE_NAME, hci);
  if (tmp) {
    mx2ads_usbhost_exit();
    err("failed to request irq");
    return -EFAULT;
  }
  mx2hci_irq_inited = 1;

  if ((tmp = mx2otg_register_host(&mx2hcd_otg_descriptor)) < 0) {
    mx2ads_usbhost_exit();
    err("cannot register host with OTG");
    return tmp;
  }
  mx2hci_otg_inited = 1;

  return 0;
}

/* module exit*/
static void __init
mx2ads_usbhost_exit(void)
{
  hci_t *hci = mx2hci_struct;

  if (mx2hci_otg_inited) {
    mx2otg_unregister_host(&mx2hcd_otg_descriptor);
  }

  if (mx2hci_irq_inited) {
    OTG_CORE_CINT_STEN &= ~OTG_CORE_HCINTEN;	/*disable host interrupt */
    free_irq(INT_USBHOST, hci);
  }

  if (mx2hci_struct)
    kfree(mx2hci_struct);

}

module_init(mx2ads_usbhost_init);
module_exit(mx2ads_usbhost_exit);

MODULE_AUTHOR("MontaVista Software Inc");
MODULE_DESCRIPTION("Motorola i.MX21 USB Host");
MODULE_LICENSE("GPL");
