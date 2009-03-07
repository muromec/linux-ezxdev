/*
 * drivers/usb/ipcusb.c 
 *
 *  Description of the file
 *  This file implements usb host function driver based on Intel's Bulverde USB host Controller. 
 *
 *  Copyright (C) 2003-2005 - Motorola
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Author: 
 *  2003-Mar-11 ---  Created file     [q16136@motorola.com]
 *  2004-Feb-20 --- power management support [q16136@motorola.com]	
 *  2004-Apr-20	--- port suspend/resume support [q16136@motorola.com]
 *  2005-Aug-01 ---  redesign host resume mechanism  and bp_rdy reentry protection mechanism[w20535@motorola.com]	
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/circ_buf.h>
#include <linux/usb.h>
#include <linux/apm_bios.h>

#include "ipcusb.h"

#define DRIVER_VERSION "1.0"
#define DRIVER_AUTHOR "Levis Xu <Levis@Motorola.com>"
#define DRIVER_DESC "IPCoUSB Driver"

/*global values defined*/
static struct usb_driver 		usb_ipc_driver;
static struct timer_list 		ipcusb_timer;
static struct timer_list 		suspend_timer;
static struct usb_flip_buffer ipcusb_flip_buf;
extern int dsplog_active;

static struct tty_driver		ipcusb_tty_driver;	/* the coresponding tty driver, we just use write and chars in buff here*/
struct tty_driver *usb_for_mux_driver = NULL;
struct usb_flip_buffer *usb_for_mux_flip_buf = NULL;
void (*usb_mux_dispatcher)(struct tty_struct *tty) = NULL;
void (*usb_mux_sender)(void) = NULL;
void (*ipcusb_ap_to_bp)(unsigned char*, int) = NULL;
void (*ipcusb_bp_to_ap)(unsigned char*, int) = NULL;
EXPORT_SYMBOL(usb_for_mux_driver);	
EXPORT_SYMBOL(usb_for_mux_flip_buf);	
EXPORT_SYMBOL(usb_mux_dispatcher);	
EXPORT_SYMBOL(usb_mux_sender);
EXPORT_SYMBOL(ipcusb_ap_to_bp);
EXPORT_SYMBOL(ipcusb_bp_to_ap);
static int sumbit_times = 0; 	
static int callback_times = 0; 
//static unsigned long last_jiff = 0;
extern void wmmx_ohci_resume_call(void);
extern void set_BLE(void);
extern void clear_BLE(void);
/*end global values defined*/

int usb_host_resumed = 1;
extern int usbh_finished_resume;

/*this fragment code to prevent bp_rdy_int reentry*/
static  int bprdy_interrupt_handling = 0;
int in_bprdy_interrupt(void)
{
	return bprdy_interrupt_handling;
}
void set_in_bprdy_interrupt(void)
{
	bprdy_interrupt_handling = 1;
	bvd_dbg("bprdy handling set\n");
}
void clear_in_bprdy_interrupt(void)
{
	bprdy_interrupt_handling = 0;
	bvd_dbg("bprdy handling cleared\n");
}

static int ipc_traffic = 0;
int host_port_suspended = 0;
int ipc_is_active(void)
{
	if(host_port_suspended)
		ipc_traffic = 0;
	else 
		ipc_traffic = 1;

	return ipc_traffic;
}
void set_ipc_is_active(void)
{
	host_port_suspended = 0;
}
EXPORT_SYMBOL(ipc_is_active);

MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");

/* USB device context */
typedef struct {
  struct list_head list;
  int size;
  char* body; 
} buf_list_t;

struct ipc_usb_data
{
	__u8 write_finished_flag;
	struct usb_device	*ipc_dev;/* USB device handle */	
	struct urb   readurb_mux, writeurb_mux, writeurb_dsplog;/* urbs */
	__u8	write_flag, ipc_flag, suspend_flag;
	char *obuf, *ibuf;
	struct circ_buf	xmit;				/* write cric bufffer */
  	struct list_head 	in_buf_list;
	char bulk_in_ep_mux, bulk_out_ep_mux, bulk_in_ep_dsplog;		/* Endpoint assignments */
	unsigned int 		ifnum;				/* Interface number of the USB device */
	struct tq_struct tqueue;	/*task queue for write urbs*/
	struct tq_struct tqueue_bp;	/*task queue for bp ready IN token*/

	spinlock_t lock;
}; 
struct ipc_usb_data *bvd_ipc;

#ifdef BVD_DEBUG
static void bvd_dbg_hex(__u8 *buf, int len)
{
	static unsigned char tbuf[ DBG_MAX_BUF_SIZE ];

	int i;
	int c;

	if( len <= 0 ) {
		return;
	}  

	c = 0;  
	for( i=0; (i < len) && (c < (DBG_MAX_BUF_SIZE - 3)); i++){
		sprintf(&tbuf[c], "%02x ",buf[i]);
		c += 3;
	}
	tbuf[c] = 0;

	printk("%s: %s\n", __FUNCTION__, tbuf);
}
#endif
static int unlink_urbs(struct urb *urb)
{	
	unsigned long	flags;
	int 	retval;

	spin_lock_irqsave(&bvd_ipc->lock, flags);
	
	retval = usb_unlink_urb (urb);
	if (retval != -EINPROGRESS && retval != 0)
	{
		printk("unlink urb err, %d", retval);
	}
	
	spin_unlock_irqrestore(&bvd_ipc->lock, flags);
	return retval;
}
static void append_to_inbuf_list(struct urb *urb)
{
	buf_list_t *inbuf;
	int count = urb->actual_length;
	
	inbuf = (buf_list_t*)kmalloc( sizeof(buf_list_t), GFP_KERNEL );
	if ( !inbuf ) {
		printk( "append_to_inbuf_list: (%d) out of memory!\n", sizeof(buf_list_t) );
		return;
	}
	
	inbuf->size = count;
	inbuf->body = (char*)kmalloc( sizeof(char)*count, GFP_KERNEL );
	if ( !(inbuf->body) ) 
	{
		kfree(inbuf);
		printk( "append_to_inbuf_list: (%d) out of memory!\n", sizeof(char)*count );
		return;
	}
	memcpy(inbuf->body, (unsigned char*)urb->transfer_buffer, count);
	list_add_tail( &inbuf->list, &bvd_ipc->in_buf_list);
}

static void ipcusb_timeout(unsigned long data)
{
	struct usb_flip_buffer *flip_buf = &ipcusb_flip_buf;	
	struct urb *urb;
	
	bvd_dbg("ipcusb_timeout***");

 	urb = (struct urb *)data;
	if(flip_buf->count == 0)
	{
		while(!(list_empty(&bvd_ipc->in_buf_list)))
		{
			int count;
			buf_list_t *inbuf;
			struct list_head* ptr=NULL;			
			
			ptr = bvd_ipc->in_buf_list.next;
			inbuf = list_entry (ptr, buf_list_t, list);			
			count = inbuf->size;
			if ((flip_buf->count + count) <= USB_FLIPBUF_SIZE) 
			{
				memcpy(flip_buf->char_buf_ptr, inbuf->body, count);	
				flip_buf->count += count;	
				flip_buf->char_buf_ptr += count;  					

				list_del (ptr);
				kfree (inbuf->body);
				inbuf->body = NULL;
				kfree (inbuf);				
			} else{
				bvd_dbg("ipcusb_timeout: bvd_ipc->in_buf_list empty!");
				break;
			}
		}

		if (usb_mux_dispatcher)
		{
			usb_mux_dispatcher(0);	/**call Liu changhui's func.**/	
		}
		
		if(list_empty(&bvd_ipc->in_buf_list))
		{
			urb->actual_length = 0;
			urb->dev = bvd_ipc->ipc_dev;
			if (usb_submit_urb(urb))
			{
				bvd_dbg("ipcusb_timeout: failed resubmitting read urb");
			}
			bvd_dbg("ipcusb_timeout: resubmited read urb");
		} else{				
			ipcusb_timer.data = (unsigned long)urb;
			mod_timer(&ipcusb_timer, jiffies+(10*HZ/1000));
		}
	} else{
		/***for test flow control***/
		if (usb_mux_dispatcher)
		{
			usb_mux_dispatcher(0);	/**call Liu changhui's func.**/	
		}
		/***end test***/

		mod_timer(&ipcusb_timer, jiffies+(10*HZ/1000));
		bvd_dbg("ipcusb_timeout: add timer again!");
	}	
}

static void usb_ipc_read_bulk(struct urb *urb)
{	
	buf_list_t *inbuf;
	int count = urb->actual_length;
	struct usb_flip_buffer *flip_buf = &ipcusb_flip_buf;	
	
 	//printk("usb_ipc_read_bulk: begining!\n");
	if (urb->status)
	{
		bvd_dbg("nonzero read bulk status received: %d\n", urb->status);
	}
	
 	//printk("usb_ipc_read_bulk: urb->actual_length=%d\n", urb->actual_length);

#ifdef BVD_DEBUG 	
	bvd_dbg_hex((unsigned char*)urb->transfer_buffer, urb->actual_length);
#endif

	if( count>0 && ((*ipcusb_bp_to_ap) != NULL) )
	{
		(*ipcusb_bp_to_ap)(urb->transfer_buffer, urb->actual_length);
	}
 
 	if(!(list_empty(&bvd_ipc->in_buf_list)))	
	{
		int need_mux = 0;

 		bvd_dbg("usb_ipc_read_bulk: some urbs in_buf_list");	
		if(count > 0)
		{
			bvd_ipc->suspend_flag = 1;
			append_to_inbuf_list(urb);	/**append the current received urb**/	
#if 0		
			if(jiffies - last_jiff > ICL_EVENT_INTERVAL) 
			{
				last_jiff = jiffies;
				apm_event_notify(APM_EVENT_DEVICE, EVENT_DEV_ICL, 0);
			}	
#endif
		}
		
		while(!(list_empty(&bvd_ipc->in_buf_list)))
		{
			struct list_head* ptr=NULL;	
			ptr = bvd_ipc->in_buf_list.next;
			inbuf = list_entry (ptr, buf_list_t, list);			
			count = inbuf->size;
			if ((flip_buf->count + count) <= USB_FLIPBUF_SIZE) 
			{	
				need_mux = 1;
				memcpy(flip_buf->char_buf_ptr, inbuf->body, count);	
				flip_buf->count += count;	
				flip_buf->char_buf_ptr += count;  				

				list_del (ptr);
				kfree (inbuf->body);
				inbuf->body = NULL;
				kfree (inbuf);				
			} else{
				bvd_dbg("usb_ipc_read_bulk: bvd_ipc->in_buf_list empty!");
				break;
			}			
		}	

		if (usb_mux_dispatcher && need_mux) 
		{
			usb_mux_dispatcher(0);	/**call Liu changhui's func.**/					
		}

		if(list_empty(&bvd_ipc->in_buf_list))
		{
			urb->actual_length = 0;
			urb->dev = bvd_ipc->ipc_dev;
			if (usb_submit_urb(urb))
			{
				bvd_dbg("usb_ipc_read_bulk: failed resubmitting read urb");
			}
			bvd_dbg("usb_ipc_read_bulk: resubmited read urb");
		} else{				
			ipcusb_timer.data = (unsigned long)urb;
			mod_timer(&ipcusb_timer, jiffies+(10*HZ/1000));
		}
	}else if ((count > 0) && ((flip_buf->count + count) <= USB_FLIPBUF_SIZE)) {
 		bvd_dbg("usb_ipc_read_bulk: no urbs in_buf_list");	
		bvd_ipc->suspend_flag = 1;
		memcpy(flip_buf->char_buf_ptr, (unsigned char*)urb->transfer_buffer, count);	
		flip_buf->count += count;	
		flip_buf->char_buf_ptr += count;    

		if (usb_mux_dispatcher)
		{
			usb_mux_dispatcher(0);/**call Liu changhui's func.**/
		}

		urb->actual_length = 0;
		urb->dev = bvd_ipc->ipc_dev;
		if (usb_submit_urb(urb))
		{
			bvd_dbg("failed resubmitting read urb");
		}
#if 0
		if(jiffies - last_jiff > ICL_EVENT_INTERVAL) 
		{
			last_jiff = jiffies;
			apm_event_notify(APM_EVENT_DEVICE, EVENT_DEV_ICL, 0);
		}
#endif	
		bvd_dbg("usb_ipc_read_bulk: resubmited read urb");
	}else if(count > 0) {
			bvd_ipc->suspend_flag = 1;
			append_to_inbuf_list(urb);
			ipcusb_timer.data = (unsigned long)urb;
			mod_timer(&ipcusb_timer, jiffies+(10*HZ/1000));
#if 0			
			if(jiffies - last_jiff > ICL_EVENT_INTERVAL) 
			{
				last_jiff = jiffies;
				apm_event_notify(APM_EVENT_DEVICE, EVENT_DEV_ICL, 0);
			}
#endif	
	}
		
	bvd_dbg("usb_ipc_read_bulk: completed!!!");		
}

static void usb_ipc_write_bulk(struct urb *urb)
{
	callback_times++;
	bvd_ipc->write_finished_flag = 1;
	
	bvd_dbg("usb_ipc_write_bulk: begining!");
	//printk("%s: write_finished_flag=%d\n", __FUNCTION__, bvd_ipc->write_finished_flag);
	
	if (urb->status)
	{
		printk("nonzero write bulk status received: %d\n", urb->status);		
	}

	if (usb_mux_sender)
	{
		usb_mux_sender();		/**call Liu changhui's func**/
	}

	//printk("usb_ipc_write_bulk: mark ipcusb_softint!\n");
	queue_task(&bvd_ipc->tqueue, &tq_immediate);
	mark_bh(IMMEDIATE_BH);

	bvd_dbg("usb_ipc_write_bulk: finished!");
}

static void suspend_timeout(unsigned long data)
{
#ifdef CONFIG_NO_USB_SUSPEND_RESUME
	return;
#else

#ifdef CONFIG_DSPLOG_USB
	/*
		 This will prevent the case that suspend port when doing dsplog. 
		 this is a temporary solution for the case of dsplog and ipc function simultaneously.
	 */
	if(dsplog_active)   
		return;
#endif
	if(!usb_host_resumed)
	{
		printk("\n!!!suspend timer has been started. however host not resumed\n");
		return;
	}

	if( !(UHCRHPS3 & 0x2))
        {
                printk("\n suspend_timeout:port has been disabled.\n");
                return;
        }

	if((bvd_ipc->suspend_flag == 1))
	{
		bvd_ipc->suspend_flag = 0;
		mod_timer(&suspend_timer, jiffies+(IPC_USB_SUSPEND_INTERVAL*HZ/1000));
		bvd_dbg("suspend_timeout: add the suspend timer again");	
	}else
	{
		unlink_urbs(&bvd_ipc->readurb_mux);
		clear_BLE();
		mdelay(1);
		UHCRHPS3 = 0x4;
		bvd_dbg("suspend_timeout: send SUSPEND cmd! UHCRHPS3=0x%x\n", UHCRHPS3);
		while(!(UHCRHPS3 & 0x4)) mdelay(1);
//		ipc_traffic = 0;
		host_port_suspended = 1;
		printk("ipc-suspend\n");
		if(in_bprdy_interrupt())
			clear_in_bprdy_interrupt();
	}
#endif
}

static void ipcusb_xmit_data(void)
{	
	int c, count = IPC_URB_SIZE;
	int result = 0;
	int buf_flag = 0;
	int buf_num = 0;

	if( !(UHCRHPS3 & 0x2))
       	{
                printk("\n ipcusb_xmit_data:port has been disabled.\n");
                return;
        }		
	//printk("%s: sumbit_times=%d, callback_times=%d\n", __FUNCTION__, sumbit_times, callback_times);
	if(bvd_ipc->write_finished_flag == 0)
	{
		//printk("%s: write_finished_flag=%d, return!\n", __FUNCTION__, bvd_ipc->write_finished_flag); 
		return;
	}
 
	while (1) 
	{
		c = CIRC_CNT_TO_END(bvd_ipc->xmit.head, bvd_ipc->xmit.tail, IPC_USB_XMIT_SIZE);
		if (count < c)
		  c = count;
		if (c <= 0) {		  
		  break;
		}
		memcpy(bvd_ipc->obuf+buf_num, bvd_ipc->xmit.buf + bvd_ipc->xmit.tail, c);
		buf_flag = 1;	
		bvd_ipc->xmit.tail = ((bvd_ipc->xmit.tail + c) & (IPC_USB_XMIT_SIZE-1));
		count -= c;
		buf_num += c;
	}    
#ifdef CONFIG_NO_USB_SUSPEND_RESUME
		
#else	 
	if(buf_num == 0)
	{
		bvd_dbg("ipcusb_xmit_data: buf_num=%d, add suspend_timer", buf_num);
		bvd_ipc->suspend_flag = 0;
		mod_timer(&suspend_timer, jiffies+(IPC_USB_SUSPEND_INTERVAL*HZ/1000));
	}
#endif
	bvd_dbg("ipcusb_xmit_data: buf_num=%d", buf_num);
	bvd_dbg("ipcusb_xmit_data: bvd_ipc->obuf: ");

#ifdef BVD_DEBUG	
	bvd_dbg_hex((bvd_ipc->obuf)-buf_num, buf_num);
#endif

	if( buf_flag )
	{
		
		bvd_ipc->writeurb_mux.transfer_buffer_length = buf_num;
		bvd_dbg("ipcusb_xmit_data: copy data to write urb finished! ");
#ifdef CONFIG_NO_USB_SUSPEND_RESUME 

#else		
		if((UHCRHPS3 & 0x4) == 0x4)
		{
			static int ret;
			int time = 0;	

#ifdef CONFIG_ARCH_EZXBASE
			/**if BP sleep, wake up BP first**/
			set_GPIO_mode(GPIO_IN | GPIO_BP_RDY);
			if(!GPIO_is_high(GPIO_BP_RDY))
			{
				if(GPIO_is_high(GPIO_AP_RDY ))
				{
					GPCR(GPIO_AP_RDY ) = GPIO_bit(GPIO_AP_RDY );
					udelay(WAKE_UP_BP_UDELAY);
					GPSR(GPIO_AP_RDY ) = GPIO_bit(GPIO_AP_RDY );
				}else {
					GPSR(GPIO_AP_RDY ) = GPIO_bit(GPIO_AP_RDY );
					udelay(WAKE_UP_BP_UDELAY);
					GPCR(GPIO_AP_RDY ) = GPIO_bit(GPIO_AP_RDY );
				}
				time = jiffies;
				while(!GPIO_is_high(GPIO_BP_RDY) && (jiffies < (time+HZ)))					;

				if(!GPIO_is_high(GPIO_BP_RDY))
				{
					printk("%s: Wakeup BP timeout! BP is still in sleep state!\n", __FUNCTION__);		
				}				
			}
#else 
			/**if BP sleep, wake up BP first**/
			set_GPIO_mode(GPIO_IN | 41);
			if(GPIO_is_high(41))
			{
				if(GPIO_is_high(GPIO_MCU_INT_SW))
					GPCR(GPIO_MCU_INT_SW) = GPIO_bit(GPIO_MCU_INT_SW);
				else{
					GPSR(GPIO_MCU_INT_SW) = GPIO_bit(GPIO_MCU_INT_SW);
				}
				time = jiffies;
				while(GPIO_is_high(41) && (jiffies < (time+HZ)))					;
				if(GPIO_is_high(41))
				{
					printk("%s: Wakeup BP timeout! BP state is %d\n", __FUNCTION__, GPIO_is_high(41));		
				}
				if(GPIO_is_high(GPIO_MCU_INT_SW))
					GPCR(GPIO_MCU_INT_SW) = GPIO_bit(GPIO_MCU_INT_SW);
				else{
					GPSR(GPIO_MCU_INT_SW) = GPIO_bit(GPIO_MCU_INT_SW);
				}
			}
#endif
			/** Resume BP**/ 
//			ipc_traffic = 1;
	
			UHCRHPS3 = 0x8;
			while(!(UHCRHPS3 & (1<<18))) mdelay(1);
			UHCRHPS3 = 1<<18; // Write 1 to clear
			bvd_dbg("ipc-resume[0x%x]\n",UHCRHPS3);
			bvd_dbg("ipcusb_xmit_data: Send RESUME signal! UHCRHPS3=0x%x\n", UHCRHPS3);
			host_port_suspended = 0;
			mdelay(3);
			set_BLE();
			/*send IN token*/
			bvd_ipc->readurb_mux.actual_length = 0;
			bvd_ipc->readurb_mux.dev = bvd_ipc->ipc_dev;
			if((ret = usb_submit_urb(&bvd_ipc->readurb_mux)) < 0)
			{
				warn("\nerr:[out]submit_readurb failed [%d]\n", ret);
			}
			bvd_dbg("\n[out]IN token sent-----\n");
		}
#endif
		sumbit_times++;
		bvd_ipc->write_finished_flag = 0;
		bvd_dbg("%s: clear write_finished_flag:%d\n", __FUNCTION__, bvd_ipc->write_finished_flag);
		bvd_ipc->writeurb_mux.dev = bvd_ipc->ipc_dev;
		if((result = usb_submit_urb(&bvd_ipc->writeurb_mux)) < 0)
		{
			warn("\n[out]submit_urb_ret[%d]\n", result);
		}
		
		bvd_dbg("\n[out]usb_submit_urb[%d]\n", result);

	}		
}

static void ipcusb_softint(void *private)
{
	if(!usb_host_resumed)
	{
		resume_suspend_dbg("\nintoken_bf: resume\n");
		wmmx_ohci_resume_call();
		usb_host_resumed = 1;
	}
	ipcusb_xmit_data();
}

extern void get_halted_bit(void);
static void ipcusb_softint_send_readurb(void *private)
{
	
	if(!usb_host_resumed)
	{
		resume_suspend_dbg("\n[bf_readurb]host resume\n");
		wmmx_ohci_resume_call();
		usb_host_resumed = 1;
	}

	if( !(UHCRHPS3 & 0x2))
	{
		printk("ipcusb_softint_send_readurb:port has been disabled.UHCRHPS3[%x]\n", UHCRHPS3);
		
		if(in_bprdy_interrupt())
			clear_in_bprdy_interrupt();
		return;
	}
	if((UHCRHPS3 & 0x4) == 0x4)
	{
		UHCRHPS3 = 0x8;
		while(!(UHCRHPS3 & (1<<18))) {
			mdelay(1);
		}
		UHCRHPS3 = 1<<18; // Write 1 to clear 
		host_port_suspended = 0;
		bvd_dbg("ipc-resume[0x%x]\n", UHCRHPS3);
		mdelay(3);
		set_BLE();
	}
	if (bvd_ipc->ipc_flag == IPC_USB_PROBE_READY)
	{ 
		get_halted_bit();

		/*send a IN token*/
		bvd_ipc->readurb_mux.dev = bvd_ipc->ipc_dev;
		if(usb_submit_urb(&bvd_ipc->readurb_mux))
		{
			bvd_dbg("err:[bf_readurb]urb submit failed!\n");
		}
		bvd_dbg("[bf_readurb] IN token sent----\n");
		bvd_ipc->suspend_flag = 0;
		bvd_dbg("ipcusb_softint_send_readurb: add suspend_timer");
		mod_timer(&suspend_timer, jiffies+(IPC_USB_SUSPEND_INTERVAL*HZ/1000));		
	}
	if(in_bprdy_interrupt()){
		clear_in_bprdy_interrupt();
		bvd_dbg(" bprdy_interrupt_handling cleared.\n");
	}
}

static int usb_ipc_write (struct tty_struct * tty, int from_user, const unsigned char *buf, int count)
{	
	int c, ret = 0;
        unsigned long flags;
		
	bvd_dbg("usb_ipc_write: count=%d, buf: ", count);

#ifdef BVD_DEBUG
	bvd_dbg_hex(buf, count);
#endif

	if(count <= 0)
	{
		return 0;
	}	

	if(*ipcusb_ap_to_bp != NULL)
		(*ipcusb_ap_to_bp)(buf, count);

	bvd_ipc->suspend_flag = 1;
	
	if ((bvd_ipc->ipc_flag == IPC_USB_PROBE_READY) && (bvd_ipc->xmit.head == bvd_ipc->xmit.tail))
	{
		bvd_dbg("usb_ipc_write: set write_flag");
		bvd_ipc->write_flag = IPC_USB_WRITE_XMIT;
	}
		
        save_flags(flags);cli();
	while (1) 
	{
		c = CIRC_SPACE_TO_END(bvd_ipc->xmit.head, bvd_ipc->xmit.tail, IPC_USB_XMIT_SIZE);
		if (count < c)		
			c = count;		
		if (c <= 0) {
			break;
		}
		memcpy(bvd_ipc->xmit.buf + bvd_ipc->xmit.head, buf, c);
		bvd_ipc->xmit.head = ((bvd_ipc->xmit.head + c) & (IPC_USB_XMIT_SIZE-1));
		buf += c;
		count -= c;
		ret += c;
	}	
        restore_flags(flags);
	bvd_dbg("usb_ipc_write: ret=%d, bvd_ipc->xmit.buf: ", ret);
	
#ifdef BVD_DEBUG
	bvd_dbg_hex(bvd_ipc->xmit.buf, ret);
#endif

	if (bvd_ipc->write_flag == IPC_USB_WRITE_XMIT)
	{
		bvd_ipc->write_flag = IPC_USB_WRITE_INIT;
		bvd_dbg("usb_ipc_write: mark ipcusb_softint");

		queue_task(&bvd_ipc->tqueue, &tq_immediate);
		mark_bh(IMMEDIATE_BH);
	}
	
	bvd_dbg("usb_ipc_write: ret=%d\n", ret);

	return ret;
}

static int usb_ipc_chars_in_buffer(struct tty_struct *tty)
{		
	return CIRC_CNT(bvd_ipc->xmit.head, bvd_ipc->xmit.tail, IPC_USB_XMIT_SIZE);
}

void usb_send_readurb( void )
{
        bvd_dbg("\nusb_send_readurb:UHCRHPS3[0x%x]\n", UHCRHPS3);
	
	queue_task(&bvd_ipc->tqueue_bp, &tq_immediate);
	mark_bh(IMMEDIATE_BH);
}

static void *usb_ipc_probe(struct usb_device *usbdev, unsigned int ifnum,
			   const struct usb_device_id *id)
{	
	struct usb_config_descriptor *ipccfg;
	struct usb_interface_descriptor *interface;
	struct usb_endpoint_descriptor *endpoint;		
	int ep_cnt, readsize, writesize;
	char have_bulk_in_mux, have_bulk_out_mux;

	bvd_dbg("(q16136)usb_ipc_probe: vendor id 0x%x, device id 0x%x\n", usbdev->descriptor.idVendor, usbdev->descriptor.idProduct);

	if ((usbdev->descriptor.idVendor != MOTO_IPC_VID) || (usbdev->descriptor.idProduct != MOTO_IPC_PID))
	{	
 		return NULL;
	}
	// a2590c : dsplog interface is not supported by this driver
	if (ifnum == 2 ) {   // dsplog interface number is 2
		return NULL;
	}

	bvd_dbg("usb_ipc_probe: USB dev address:%p", usbdev);
	bvd_dbg("usb_ipc_probe: ifnum:%u", ifnum);
	
	ipccfg = usbdev->actconfig;
	bvd_dbg("usb_ipc_prob: config%d", ipccfg->bConfigurationValue);
	bvd_dbg("usb_ipc_prob: bNumInterfaces = %d", ipccfg->bNumInterfaces);

/*
 * After this point we can be a little noisy about what we are trying to configure, hehe.
 */
	if (usbdev->descriptor.bNumConfigurations != 1) 
	{
		info("usb_ipc_probe: Only one device configuration is supported.");
		return NULL;
	}
	
	interface = usbdev->config[0].interface[0].altsetting;
	endpoint = interface[0].endpoint;
/*
 * Start checking for two bulk endpoints or ... FIXME: This is a future enhancement...
 */	
	bvd_dbg("usb_ipc_probe: Number of Endpoints:%d", (int) interface->bNumEndpoints);	

	if (interface->bNumEndpoints != 2) 
	{
		info("usb_ipc_probe: Only two endpoints supported.");
		return NULL;
	}

	ep_cnt = have_bulk_in_mux = have_bulk_out_mux = 0;	

	bvd_dbg("usb_ipc_probe: endpoint[0] is:%x", (&endpoint[0])->bEndpointAddress);
	bvd_dbg("usb_ipc_probe: endpoint[1] is:%x ", (&endpoint[1])->bEndpointAddress);
	
	while (ep_cnt < interface->bNumEndpoints) 
	{

		if (!have_bulk_in_mux && IS_EP_BULK_IN(endpoint[ep_cnt])) 
		{			
			bvd_dbg("usb_ipc_probe: bEndpointAddress(IN) is:%x ", (&endpoint[ep_cnt])->bEndpointAddress);
			have_bulk_in_mux = (&endpoint[ep_cnt])->bEndpointAddress;
			readsize = (&endpoint[ep_cnt])->wMaxPacketSize;	
			bvd_dbg("usb_ipc_probe: readsize=%d", readsize);			
			ep_cnt++;
			continue;
		}

		if (!have_bulk_out_mux && IS_EP_BULK_OUT(endpoint[ep_cnt])) 
		{			
			bvd_dbg("usb_ipc_probe: bEndpointAddress(OUT) is:%x ", (&endpoint[ep_cnt])->bEndpointAddress);
			have_bulk_out_mux = (&endpoint[ep_cnt])->bEndpointAddress;			
			writesize = (&endpoint[ep_cnt])->wMaxPacketSize;
			bvd_dbg("usb_ipc_probe: writesize=%d", writesize);			
			ep_cnt++;
			continue;
		}
		
		info("usb_ipc_probe: Undetected endpoint ^_^ ");
		return NULL;	/* Shouldn't ever get here unless we have something weird */
	}

/*
 * Perform a quick check to make sure that everything worked as it should have.
 */

	switch(interface->bNumEndpoints) 
	{
		case 2:
			if (!have_bulk_in_mux || !have_bulk_out_mux) 
			{
				info("usb_ipc_probe: Two bulk endpoints required.");
				return NULL;
			}
			break;	
		default:
			info("usb_ipc_probe: Endpoint determination failed ^_^ ");
			return NULL;
	}	

/* Ok, now initialize all the relevant values */
	if (!(bvd_ipc->obuf = (char *)kmalloc(IPC_URB_SIZE, GFP_KERNEL))) 
	{
		err("usb_ipc_probe: Not enough memory for the output buffer.");
		kfree(bvd_ipc);		
		return NULL;
	}
	bvd_dbg("usb_ipc_probe: obuf address:%p", bvd_ipc->obuf);

	if (!(bvd_ipc->ibuf = (char *)kmalloc(IPC_URB_SIZE, GFP_KERNEL))) 
	{
		err("usb_ipc_probe: Not enough memory for the input buffer.");
		kfree(bvd_ipc->obuf);
		kfree(bvd_ipc);		
		return NULL;
	}
	bvd_dbg("usb_ipc_probe: ibuf address:%p", bvd_ipc->ibuf);

	bvd_ipc->ipc_flag = IPC_USB_PROBE_READY;	
	bvd_ipc->write_finished_flag = 1;
	bvd_ipc->suspend_flag = 1;
	bvd_ipc->bulk_in_ep_mux= have_bulk_in_mux;
	bvd_ipc->bulk_out_ep_mux= have_bulk_out_mux;		
	bvd_ipc->ipc_dev = usbdev;	
	INIT_LIST_HEAD (&bvd_ipc->in_buf_list);
	bvd_ipc->tqueue.routine = ipcusb_softint;
	bvd_ipc->tqueue.data = bvd_ipc;	
	bvd_ipc->tqueue_bp.routine = ipcusb_softint_send_readurb;
	bvd_ipc->tqueue_bp.data = bvd_ipc;	
	
	/*Build a write urb*/
	FILL_BULK_URB(&bvd_ipc->writeurb_mux, usbdev, usb_sndbulkpipe(bvd_ipc->ipc_dev, bvd_ipc->bulk_out_ep_mux),bvd_ipc->obuf, IPC_URB_SIZE, usb_ipc_write_bulk, bvd_ipc);
	bvd_ipc->writeurb_mux.transfer_flags |= USB_ASYNC_UNLINK;

	/*Build a read urb and send a IN token first time*/
	FILL_BULK_URB(&bvd_ipc->readurb_mux, usbdev, usb_rcvbulkpipe(usbdev, bvd_ipc->bulk_in_ep_mux), bvd_ipc->ibuf, IPC_URB_SIZE, usb_ipc_read_bulk, bvd_ipc);

	bvd_ipc->readurb_mux.transfer_flags |= USB_ASYNC_UNLINK;	
	
	usb_driver_claim_interface(&usb_ipc_driver, &ipccfg->interface[0], bvd_ipc);
	usb_driver_claim_interface(&usb_ipc_driver, &ipccfg->interface[1], bvd_ipc);
	
        // a2590c: dsplog is not supported by this driver
	//	usb_driver_claim_interface(&usb_ipc_driver, &ipccfg->interface[2], bvd_ipc);
	/*send a IN token first time*/
	bvd_ipc->readurb_mux.dev = bvd_ipc->ipc_dev;
	if(usb_submit_urb(&bvd_ipc->readurb_mux))
	{
		printk("usb_ipc_prob: usb_submit_urb(read mux bulk) failed!\n");
	}
	bvd_dbg("usb_ipc_prob: Send a IN token successfully!");
	
	if(bvd_ipc->xmit.head != bvd_ipc->xmit.tail)
	{
		bvd_dbg("usb_ipc_probe: mark ipcusb_softint!\n");
		queue_task(&bvd_ipc->tqueue, &tq_immediate);
		mark_bh(IMMEDIATE_BH);
	}

	printk("usb_ipc_probe: completed probe!");
	return bvd_ipc;
}

static void usb_ipc_disconnect(struct usb_device *usbdev, void *ptr)
{
	struct ipc_usb_data *bvd_ipc_disconnect = (struct ipc_usb_data *) ptr;
		
	printk("usb_ipc_disconnect:*** \n");
#ifdef CHK_IPC
	printk("oscr = 0x%x\n", OSCR);
	panic(" in usb ipc");
#endif
	if((UHCRHPS3 & 0x4) == 0)
	{	
		usb_unlink_urb(&bvd_ipc_disconnect->readurb_mux);
	}
	usb_unlink_urb(&bvd_ipc_disconnect->writeurb_mux);
        
	bvd_ipc_disconnect->ipc_flag = IPC_USB_PROBE_NOT_READY;
	kfree(bvd_ipc_disconnect->ibuf);
	kfree(bvd_ipc_disconnect->obuf);
	
	usb_driver_release_interface(&usb_ipc_driver, &bvd_ipc_disconnect->ipc_dev->actconfig->interface[0]);
        usb_driver_release_interface(&usb_ipc_driver, &bvd_ipc_disconnect->ipc_dev->actconfig->interface[1]);
        
	//a2590c: dsplog interface is not supported by this driver
	//usb_driver_release_interface(&usb_ipc_driver, &bvd_ipc_disconnect->ipc_dev->actconfig->interface[2]);

	bvd_ipc_disconnect->ipc_dev = NULL;

//	printk("usb_ipc_disconnect completed!\n");
}

static struct usb_device_id usb_ipc_id_table [] = {
	{ USB_DEVICE(MOTO_IPC_VID, MOTO_IPC_PID) },
	{ }						/* Terminating entry */
};

static struct usb_driver usb_ipc_driver = {
	name:		"usb ipc",
	probe:		usb_ipc_probe,
	disconnect:	usb_ipc_disconnect,
	id_table:	usb_ipc_id_table,
	id_table:	NULL,
};

static int __init usb_ipc_init(void)
{
	int result;

	bvd_dbg ("init usb_ipc");

	/* register driver at the USB subsystem */
	result = usb_register(&usb_ipc_driver);

	if (result < 0) {
		err ("usb ipc driver could not be registered");
		return -1;
	}

	/*init the related mux interface*/
	if (!(bvd_ipc = kmalloc (sizeof (struct ipc_usb_data), GFP_KERNEL))) {
		err("usb_ipc_init: Out of memory.");		
		return -1;
	}
	memset (bvd_ipc, 0, sizeof(struct ipc_usb_data));
	bvd_dbg ("usb_ipc_init: Address of bvd_ipc:%p", bvd_ipc);

	if (!(bvd_ipc->xmit.buf = (char *)kmalloc(IPC_USB_XMIT_SIZE, GFP_KERNEL))) 
	{
		err("usb_ipc_init: Not enough memory for the input buffer.");
		kfree(bvd_ipc);		
		return -1;
	}
	bvd_dbg("usb_ipc_init: bvd_ipc->xmit.buf address:%p", bvd_ipc->xmit.buf);
	bvd_ipc->ipc_dev = NULL;
	bvd_ipc->xmit.head = bvd_ipc->xmit.tail = 0;
	bvd_ipc->write_flag = IPC_USB_WRITE_INIT;
	memset(&ipcusb_tty_driver, 0, sizeof(struct tty_driver));
	ipcusb_tty_driver.write = usb_ipc_write;
	ipcusb_tty_driver.chars_in_buffer = usb_ipc_chars_in_buffer;
	memset(&ipcusb_flip_buf, 0, sizeof(struct usb_flip_buffer));
	ipcusb_flip_buf.char_buf_ptr = ipcusb_flip_buf.char_buf;
	usb_for_mux_driver = &ipcusb_tty_driver;
	usb_for_mux_flip_buf = &ipcusb_flip_buf;
	
	/*init timers for ipcusb read process and usb suspend*/
	init_timer(&ipcusb_timer);
	ipcusb_timer.function = ipcusb_timeout;
	init_timer(&suspend_timer);
	suspend_timer.function = suspend_timeout;

	usb_host_resumed = 1;
	host_port_suspended = 0;
	bprdy_interrupt_handling = 0;
  	return 0;
}

static void __exit usb_ipc_exit(void)
{
	bvd_dbg ("cleanup bvd_ipc");

	kfree(bvd_ipc->xmit.buf);
	kfree(bvd_ipc);
		
	usb_deregister(&usb_ipc_driver);
	info("USB Host(Bulverde) IPC driver deregistered.");
}

module_init(usb_ipc_init);
module_exit(usb_ipc_exit);
EXPORT_SYMBOL(usb_send_readurb);





