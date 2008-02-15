/*
 * ptf_fd/ptfproto.c
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 * Copyright (c) 2001 Hewlett Packard
 *
 * By: 
 *      Stuart Lynne <sl@lineo.com>, 
 *      Tom Rushworth <tbr@lineo.com>, 
 *      Bruce Balden <balden@lineo.com>
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
 *
 * Copyright (C) 2003-2004 - Motorola
 *
 *----------Motorola-ezx --history---
 *26-02-2003    created by a17400 try to finish char device driver
 *26-08-2003	   first build pass, no read write function
 *26-02-2004       second pass, read and recv function added.
 *
 *
 */


#include <linux/config.h>
#include <linux/module.h>
#ifndef MODULE
#undef GET_USE_COUNT
#define GET_USE_COUNT(foo) 1
#endif

#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/fs.h> //add by a17400
//#include <linux/tty_driver.h>
//#include <linux/tty_flip.h>
//#include <linux/tty.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>
#include <asm/segment.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <asm/irq.h>
#include <asm/system.h>


#include "ptfproto.h"
#include "ptf_buf.h"

#include <usbd-mem.h>		/* for ckmalloc */

/*  a17400 define MINOR and MAJOR */
/*  MAJOR is for character device;MINOR stands each device */


#define        PTF_CHAR_MAJOR         189	//a17400 random...???
#define        PTF_CHAR_MINOR_1        1
#define        PTF_CHAR_MINOR_2       2
#define        PTF_CHAR_MINOR_3       3
#define        PTF_CHAR_MINOR_4       4

#define	     PTF_DEV_MAX_TX_SIZE	64	//what is good???
#define        PTF_MAX_BUF_SIZE		2048 //the bottom usbd_setup_ep() fill 16*64 receive, we just twice it

#define        MIN(a,b) ((a>b) ? b : a)
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
static int ptf_usb_func_ready=0; /* a17400:1, current PTF USB interface is present, 0, no PTF usb interface,set/clear it by usb func driver*/
int ptf_cable_connected=0;	/* a17400: 1, pluged usb cable, 0, unplug usb cable */
int ptf_func_created=0;		/* a17400: to indicate bottom available(=1),poll() needs check dev pointer,see ptf_poll()*/
DECLARE_WAIT_QUEUE_HEAD(ptf_write_queue); /* a17400: add wait queue for ptf/aplog write */
/* a17400, for buttom, tell char driver that PTF interface ready */
void enable_ptf_func_driver(void){
  ptf_usb_func_ready=1;

}
/* a17400, for buttom, tell char driver that PTF interface gone */
void disable_ptf_func_driver(void){
  ptf_usb_func_ready=0;

}
/* a17400, for upper or outside, PTF interface current status
ret 0, not ready
ret 1 ready
 */
int check_ptf_func_driver(void){

  return(ptf_usb_func_ready);

}

//test print func -- comment out inside when release
void ptf_dbg_print(char * info){
  //  printk("%s",info); /*remove in release*/
}

/* defination for each device a17400*/

#define PTF_RECV_BUF_SIZE	 255
struct ptfproto_dev{
	wait_queue_head_t inq;	//receive wait queue, for polling etc.
	int dev_num;		// ptf device number (index in ptfproto_device_array)
	int opencnt;		// number of opens

 // 	char buffer[PTF_RECV_BUF_SIZE];
//  	char * pRecv_pos;     //indicates the current pos in buffer for receiving functions
// 	char * pRead_pos;     //indicates the current pos in buffer for read() of char device
 //	int overlay;	//flag indicates recv pointer will exceed read pointer
 	struct ptf_buf_queue recv_queue;	//we use a queue instead loopy buffer
	int (*xmit_data) (int, unsigned char *, int);	// callback to send data */
 	int tx_size;		// maximum transmit size 
  struct fasync_struct *async_queue; /* asynchronous signal struct, we use it to inform app config changed */
  rwlock_t rwlock;	// lock changing this structure/
};


static struct ptfproto_dev **ptfproto_device_array;	// pointer to active interaces

static rwlock_t ptfproto_rwlock = RW_LOCK_UNLOCKED;	// lock for changing global structures

/* Debug switches ****************************************************************************** */


/* interface functions for function driver  ****************************************************************************** */

/*ptfproto_recv() --called by ptf_recv_urb() in function layer, put data in device's buffer
dev_num: device index in ptfproto_dev_array
buf: buffer pointer
length: data length

return 0 ok

*/
int ptfproto_recv(int dev_num, unsigned char * buf, size_t length)
{

	int result;
	struct ptfproto_dev * dev;
	if (!ptfproto_device_array){
	  ptf_dbg_print("ptfproto_recv(): null==device array, block\n");
	  while(1);
	}
	dev=ptfproto_device_array[dev_num];

	if(NULL==dev){
		printk(KERN_WARNING "null priv data in filp\n");
		return -ENODEV;
	}
#if 0
	//zero length
	if (!length)
		return 0;

	if(NULL==dev)
	{
		printk(KERN_WARNING "null device pointer error\n");
		return -EINVAL;
	}

	if(0==dev->opencnt)
	{
		printk(KERN_WARNING "device not opened\n");
		return -EINVAL;
	}

	//put data to buffer
	//this is simplest and worst solution, just fill data to buffer looply
	for (i=0;i<length;i++)
	{
		*(dev->pRecv_pos)=buf[i];

		if (dev->pRecv_pos==dev->pRead_pos)// if true, indicates overlay happens, at least current position is unread.
			dev->overlay=1;

		
		if (PTF_RECV_BUF_SIZE -1 == dev->pRecv_pos-dev->buffer) 	//up to last char, roll back
			dev->pRecv_pos=dev->buffer; 
		else
			dev->pRecv_pos++;
	}

	return 0;
#endif
	// in ptf_buf_put_data(), create element process will copy data...
//	return ptf_buf_put_data(buf, length, &(dev->recv_queue)); 
	result= ptf_buf_put_data(buf, length, &(dev->recv_queue)); 
	//	printk("ptfproto_recv(): result=%d\n",result);
	if(!result)	//success add data
	  
	  wake_up_interruptible(&(dev->inq));//is it ok for NONBLOCK???
	//ptf_dbg_print("ptfproto_recv(): after wakeup...\n");
	return result;

	
	
}


/*tool static function *******************************************************************************/

//check device number is in the max allowed scope
static int IsValidDevNum(int dev_num)
{
  if((dev_num>=PTF_MAX_DEV_NUMBER)||(dev_num<0))
		return FALSE;
	else
		return TRUE;

}

//initialize a device after allocate it, initial it's member
//dev: pointer to device
//num: device number 
//xmit_data: send function
//tx_size :max tx size
//return 0 - ok

static int  ptfproto_init_dev(struct ptfproto_dev * dev, int num, int (* xmit_data) (int, unsigned char *, int), int tx_size)
{
	if(!dev){
		printk(KERN_WARNING "null device pointer error\n");
		return -EINVAL;
	}

	if (!(IsValidDevNum(num)))
		return -EINVAL;
	//memset(dev,0,sizeof(ptfproto_dev);
	
	init_waitqueue_head(&(dev->inq));

	dev->dev_num=num;
	dev->opencnt=0;
//	dev->pRead_pos=dev->buffer;	//init read position pointer
//	dev->pRecv_pos=dev->buffer;	//init receive position pointer
	dev->tx_size=tx_size;//is it well??? should in ptfproto_create()?
//	dev->overlay=0;
	dev->xmit_data=xmit_data;
	//device->rwlock=RW_LOCK_UNLOCKED need???
	//rwlock_init (&device->rwlock); //need ???
	dev->async_queue=NULL;
//	memset(dev->buffer, 0, sizeof(char)*PTF_RECV_BUF_SIZE);
	ptf_buf_init(&(dev->recv_queue),PTF_MAX_BUF_SIZE);
	
	return 0;
}


/* PTF character device driver Support Functions ***************************************************** */

/*a17400: fill out function_driver ops*/
static struct file_operations ptf_fops ={

	llseek:	NULL,
	read:	ptf_read,
	write:	ptf_write,
	ioctl:	NULL,
	open:	ptf_open,
	release: ptf_release,
	fasync: ptf_fasync,
	readdir: NULL,
	poll : ptf_poll,
};
/* async IO signal function, to inform app some events */
int ptf_fasync(int fd, struct file *filp, int mode){
  struct ptfproto_dev *dev = filp->private_data;
  printk (KERN_WARNING "%s: fd=%x,filp=%x mode=%d\n",__FUNCTION__,fd,filp,mode);
  if(!dev){
      printk (KERN_WARNING "%s: dev=NULL!\n",__FUNCTION__);
      return -ENODEV;
    }
  else{
    int rc=-1;
    printk (KERN_WARNING "%s: Before fasync helper\n",__FUNCTION__);
    printk (KERN_WARNING "%s: fd=%d,filp=%x mode=%d &dev->async_queue=%x\n",__FUNCTION__,fd,filp,mode,&dev->async_queue);
    rc=fasync_helper(fd, filp, mode, &dev->async_queue);
    printk (KERN_WARNING "%s: After fasync helper\n",__FUNCTION__);    
    return rc;
  }

}

/* an interface to usb-func driver, so it can inform sig to proto layer */
void ptfproto_send_fasync_to_all_devices(int sig){
        int i=0;
        for(i=0;i<PTF_MAX_DEV_NUMBER;i++){
	  struct ptfproto_dev * pDev=0;
	  
	  pDev=ptfproto_device_array[i];
	  printk (KERN_WARNING "%s: pDev=0x%x,i=%d\n",__FUNCTION__,pDev,i);
	  if(pDev){
	    if(pDev->async_queue){
	      printk (KERN_WARNING "%s: pDev->async_queue=%x,&pDev->async_queue=%x\n",__FUNCTION__,pDev->async_queue,&pDev->async_queue);
	      //    kill_fasync(&pDev->async_queue, sig, POLL_IN); /* what for band??? */
	      kill_fasync(&pDev->async_queue, sig, POLL_IN);
	    }
		    
	  }

	}


}


/*ptf_open() -to open a device 
*inode: from kernel
*file* file pointer, unused
*
*allocate device in device array if first open, else do nothing.
*open many times a same device is nothing
*inode->i_rdev indicates which device should be open
*
*device must be created by ptfproto_create() before calling ptf_open()
*/

int ptf_open (struct inode *inode, struct file *filp)
{
	int dev_num=MINOR(inode->i_rdev)&0xf;	//get minor to know which device to open
/* 	if(!ptf_cable_connected){ */
/* 	  // printk(KERN_WARNING "\n%s: Failed due to USB cable unavailable,cable_flag=%d\n",__FUNCTION__,ptf_cable_connected); */
/* 	  return -ENODEV; */
/* 	} */
	
	if(!ptfproto_device_array[dev_num])	
	{
		printk(KERN_WARNING "device not created\n");
		return -ENODEV;
	}
        if(!ptf_cable_connected)
	{
		printk(KERN_WARNING "%s: USB cable not ready,or config unfinished!!!\n",__FUNCTION__);
		return -ENODEV;
	}
	
        if(!check_ptf_func_driver())
	{
		printk(KERN_WARNING "%s: PTF/APLOG USB interface is not ready!!!\n",__FUNCTION__);
		return -ENODEV;
	}
	if (0==ptfproto_device_array[dev_num]->opencnt)	//first open
		{
		  ptf_dbg_print("ptf_open(): first open()\n");
			filp->private_data=ptfproto_device_array[dev_num];
			filp->f_op=&ptf_fops;
			//printk("ptf_open(): filp->f_op=%p\n",filp->f_op);
		}

	ptfproto_device_array[dev_num]->opencnt++;	//now useless, reset to 0 in ptf_release(), judge first open by array pointer

	MOD_INC_USE_COUNT;
	return 0;
	
	
}

/*ptf_release()-- close or release an opened device
*inode: from kernel
*file* file pointer, unused
*
*free device space
*inode->i_rdev indicates which device should be open
*/
int ptf_release (struct inode * inode, struct file *filp)
{
	int dev_num=MINOR(inode->i_rdev)&0xf;
	unsigned long flags=0;
	ptf_fasync(-1,filp, 0); /* degister helper for fasync */
	printk (KERN_WARNING "%s: Before set open cnt\n",__FUNCTION__);
	if(ptfproto_device_array[dev_num]){ /* proto_device may  be destoryed in usbd event handler!!!*/
	  ptfproto_device_array[dev_num]->opencnt=0;	//just reset opencnt, we free device in ptfproto_destory()
	  /* a17400: since keep all private and proto structs, we need clean recv buffer here */
	  save_flags_cli(flags); /* disable interrupt to access recv queue */
	  ptf_buf_queue_release(&(ptfproto_device_array[dev_num]->recv_queue));
	  restore_flags(flags);
	  printk (KERN_WARNING "%s: in set open cnt\n",__FUNCTION__);
	}
	else 
	  printk (KERN_WARNING "%s: ptfproto_device_array[%d]=NULL\n",__FUNCTION__,dev_num);
	MOD_DEC_USE_COUNT;


	//more to do???
	return 0;

}
/*read function --- put data from a device's buffer to user, maintain buffer.
filp: file pointer from kernel, 
buf : returned buffer //we used unsigned char, driver needs char
count : 
f_pos: unused

return 
	   cnt<count:???
	   cnt==count: read ok
*/
int ptf_read(struct file *filp, char *buf, size_t count,loff_t *f_pos) 
{
//	int read_num=0;
//	int i;
//	int recv_cnt=0;
//	char * cp_data=NULL;

	struct ptfproto_dev * dev;
	//	ptf_dbg_print("ptf_read(): begining\n");
	dev=filp->private_data;

	if(NULL==dev)
	{
		printk(KERN_WARNING "null priv data in filp\n");
		return -ENODEV;
	}

	if(ptf_buf_is_empty(&(dev->recv_queue))){
	  //if(filp->f_flags& O_NONBLOCK)
		  //return -EAGAIN;
	  //interruptible_sleep_on(&(dev->inq));
		  return 0;
		}

	return ptf_buf_get_data(buf, count, &(dev->recv_queue), 1);
//	if(!cp_data=kmalloc(sizeof(char)*count,GFP_KERNEL)){
//				printk(KERN_WARNING "can not allocate temp buffer in kernel for read\n");
//			return -ENOMEM;
//		}
//	copy_to_user(buf, cp_data,recv_cnt);
//	kfree(cp_data);
//	return recv_cnt;
#if 0
	//fixed me ??? this read solution is bad and easy to lose data
	for (i=0;i<count;i++)
	{
		if (dev->pRead_pos==dev->pRecv_pos)// overlay or nodata
		{
			if(!dev->overlay)	//overlay==0, no data
				return 0;	//we reach end of valid buffer
			else				//overlay happened
			{
				//copy_to_user(buf+i, dev->pRead_pos, 1);///??? use it?

				dev->overlay=0;
			}
				
				
			}
		//set 1 char
		copy_to_user(buf+i, dev->pRead_pos, 1);

		
		/*change read position */
		if(PTF_RECV_BUF_SIZE-1<dev->pRead_pos-dev->buffer)	//reach last char in buffer, roll back
			dev->pRead_pos=dev->buffer;
		else
			dev->pRead_pos++;
		//reach file end, return 0 at once
		if (dev->pRead_pos==dev->pRecv_pos)	//reach file end
		{
			dev->overlay=0;	//indicates no data more
			return 0;

		}
		

	}
	
	return i;

#endif
}

int ptf_write(struct file *filp, const char *buf, size_t count,loff_t *f_pos) 
{

        struct ptfproto_dev *dev;
        int cnt = count;
        int size = 0;
        int res;
        const unsigned char *currpos = buf;
        unsigned char *buffer1;
        unsigned char *buffer;
        if(!ptf_cable_connected)
                interruptible_sleep_on(&ptf_write_queue); /*a17400: sleep if cable unavailable */

        if ((dev = filp->private_data) == NULL) {
                printk(KERN_WARNING "null device in private data in filp\n");
                return -EINVAL;
        }

        if(!count)
                return count;

        /* we use usbd.c's ckmalloc to keep malloc counter matching, in XXX_xmit_data(),we just pass pointer and 0 len */
        if(count%64){	/* normal case  */
                if ((buffer = ckmalloc (sizeof(unsigned char)*count, GFP_KERNEL)) == NULL) {
                        printk(KERN_WARNING "can not allocate buffer in kernel for write\n");
                        return -ENOMEM;
                }


                // copy data
                copy_from_user ((void *) buffer, buf, count);



                res=dev->xmit_data (dev->dev_num, buffer, count);
                if (res){
                        printk("error,ptf_write(): dev->xmit_data()return : %d size =%d retcnt=%d\n",res,size,cnt);
                        return 0;	/* return 0 for error */
                }


                // Everything went out.
                //	printk("ptf_write(): return %d\n", count);
                return count;
        }
        else{			/* 64*n case, chop last 64 byte to 32+32 */
                int chop=32;		/* to change chop limit if necessary */
                int chop_size=count-chop;

                // printk("ptf_write(): in 64*n branch...\n");

                if ((buffer = ckmalloc (sizeof(unsigned char)*chop_size, GFP_KERNEL)) == NULL) {
                        printk(KERN_WARNING "ptf_write(): can not allocate buffer in kernel for write...,64 chopping branch\n");
                        return -ENOMEM;
                }
                //memset (buffer, '\0', size);


                // copy data
                copy_from_user ((void *) buffer, buf, chop_size);


                //	currpos += size;


                res=dev->xmit_data (dev->dev_num, buffer, chop_size);
                if (res){
                        printk("error,ptf_write(): dev->xmit_data()return : %d size =%d retcnt=%d, 64 chopping branch\n",res,size,cnt);
                        return 0;	/* return 0 for error */
                }

                /* send last piece */
                if ((buffer1 = ckmalloc (sizeof(unsigned char)*chop, GFP_KERNEL)) == NULL) {
                        printk(KERN_WARNING "ptf_write(): can not allocate buffer in kernel for write...,64 last chop branch\n");
                        return -ENOMEM;
                }
                // copy data
                copy_from_user ((void *)buffer1, buf+chop_size, chop);

                res=dev->xmit_data (dev->dev_num, buffer1, chop);
                if (res){
                        printk("error,ptf_write(): dev->xmit_data()return : %d size =%d retcnt=%d, 64 last chop branch\n",
                                        res,size,cnt);
                        return 0;	/* return 0 for error */
                }
                return count;

        }
}

//ptf_poll() --for polling


unsigned int ptf_poll (struct file * filp, struct poll_table_struct * wait){

        struct ptfproto_dev * dev;

        int mask=0;
	unsigned long flags;	/* a17400: we need secure competition conditions */
	save_flags_cli(flags);

        dev=filp->private_data;

	if((!ptf_func_created) || (!dev)){
	  restore_flags(flags);
	  return POLLERR;
	}
        poll_wait(filp,&(dev->inq), wait);

        if(ptf_buf_is_empty(&(dev->recv_queue))){
	  restore_flags(flags);
	  return mask;
	}
        mask|=POLLIN|POLLRDNORM;	//data recived, readable
	restore_flags(flags);
        return mask;
}


/*********ptf interface functions for function driver***********/
/*ptfproto_create()--called by ptf_fd_event() to create a ptf device
arguments:
name :no used
xmit_data: function to send
tx_size: max tx size

return: created device's index in device array, <0 error

int serproto_create (char *name,
int (*xmit_data) (int, unsigned char *, int),
int tx_size,
int max_queue_entries, int max_queue_bytes, int blocked, int trailer)
 */
int ptfproto_create(char *name,int (*xmit_data) (int, unsigned char *, int), int tx_size)
{
        struct ptfproto_dev * dev=NULL;
        int i=0;
        for(i=0;i<PTF_MAX_DEV_NUMBER;i++)
                if(!ptfproto_device_array[i])
                        break;

        if (PTF_MAX_DEV_NUMBER<=i)
        {
                printk(KERN_WARNING "no free device\n");
                return -ENODEV;

        }

        dev=kmalloc(sizeof (struct ptfproto_dev), GFP_KERNEL);

        if(!dev)
        {
                printk(KERN_WARNING "kmalloc fail to allocate device\n");
                return -EINVAL;	//what is better???
        }

        //initialize a device after allocate space
        ptfproto_init_dev(dev, i, xmit_data, tx_size);

        ptfproto_device_array[i]=dev;

        dev->dev_num=i; //save index number in device array to device element.

        return i;

}

int ptfproto_destory(int dev_num)
{
        //more to do ???

        if(!IsValidDevNum(dev_num))
        {
                printk(KERN_WARNING "invalid device number\n");
                return -EINVAL;	//what is better???
        }

        if(!ptfproto_device_array[dev_num])
        {
                printk(KERN_WARNING "try to destory an invaild device\n");
                return -EINVAL;	//what is better???
        }

        ptf_buf_queue_release(&(ptfproto_device_array[dev_num]->recv_queue));

        kfree(ptfproto_device_array[dev_num]);	//need null pointer check???

        ptfproto_device_array[dev_num]=NULL;

        return 0; //a17400 ??? which is better???

}

/*******module functions***************/
/*module init
 *name: device name, useless
 *max_dev_number: the max device number
 ??? we suppose return 0 is an error
return: <0 error, >=0 ok
 */
int ptfproto_modinit (char * name,int max_dev_number)
{
        int result;
        /*allocate memory for device array*/
        if (!(ptfproto_device_array = kmalloc (sizeof (struct ptfproto_dev *) * max_dev_number, GFP_KERNEL))) {
                printk (KERN_WARNING "kmalloc failed to allocate device array\n");
                return -EINVAL;
        }

	/*for each element, or device, we should allocate mem space in ptf_open() and dellocate them in ptf_release(), ptfproto_modinit()*/
	memset (ptfproto_device_array, 0, sizeof(struct ptfproto_dev *) * max_dev_number);
	
	/*register character device driver*/
	if((result=register_chrdev(PTF_CHAR_MAJOR, "ptf", &ptf_fops))<0){

		printk(KERN_WARNING "fail to get major driver number\n");
		kfree (ptfproto_device_array);	
		
	}

	return result;
}

/*module exit
 *
 */

void ptfproto_modexit (void)
{
	int i;
	/*free all memory space in device array*/
	/*??? if change to fifo, the free procedure will be changed...*/
	for (i=0;i<PTF_MAX_DEV_NUMBER;i++)
		if (ptfproto_device_array[i])
//			kfree(ptfproto_device_array[i]);
			ptfproto_destory(i);	//??? destry a not initialed device will cause error...
		kfree(ptfproto_device_array);

	/*unregister device driver*/
	unregister_chrdev(PTF_CHAR_MAJOR, "ptf");	
}



