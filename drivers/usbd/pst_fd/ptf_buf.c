/*
 * ptf_fd/ptf_buf.c
 *
 * Copyright (c) 2003 Motorola
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
 *
 * By: a17400 Motorola
 *
 *Description:
 *This file inplements a buffer management for ptf driver to store data that has been received but not been read yet.

The Main Functions:

It allows dynamic size of the buffer.

It allows caller read any length data within current buffer

It is a queue structure buffer.A piece of data(treated as a element) will be added into the queue after header and be 
pust out before end of queue everytime. For each read action, if data user wants is larger than first valid data element, 
the queue will continue reading from second buffer and recycle memory space of first element until reach the queue or 
up to user requesting length. User just can see a "continuous" buffer with single direction. In fact the queue and its each
element may scatter in discontinuous space in memory.

The reading always start from the cur_pos pointer in the first element after queue head.


 *
 *---------File History:---------------
 *04 MAR 2003 		created by a17400
 *19 MAR 2003		add maximum size for each buffer, and will delete elelent when buffer is full
 */


#include <linux/string.h>
#include <asm/uaccess.h> //copy_to_user() 
#include "ptf_buf.h"
//ptf_buf_init()- initialize buf, link head and end each other
//it should be the first call before use buf
//queue: which queue to operate
//max_size: queue's max data size.


void ptf_buf_init(struct ptf_buf_queue* queue,size_t max_size){
	if (NULL==queue){
		printk (KERN_WARNING "invalid parameter in ptf_buf_init()\n");
		return;
	}
	queue->queue_head.next=&(queue->queue_end);
	queue->queue_head.prev=NULL;
	queue->queue_end.prev=&(queue->queue_head);
	queue->queue_end.next=NULL;
	queue->max_size=max_size;
	queue->cur_len=0;
}



// ptf_buf_element_create() --create and initialize a ptf_buf_element, with a piece of data(copy it)
//data: data pointer for a piece of data, it will be freed by ptf_buf_element_destory()
//len : length of data
//return the pointer to new element, NULL if error
//we leave list_node untouched until when it will be added in a queue;

static struct ptf_buf_element *  ptf_buf_element_create(unsigned char * data, size_t len){
	struct ptf_buf_element * element=NULL;
	unsigned char * cp_data=NULL;
	if (NULL==data){
	 	  printk(KERN_WARNING "element create: empty data pointer \n"); 
			return NULL;
	}
	//	printk(KERN_WARNING "element_create():before alloc element atomic\n");	
	element=kmalloc(sizeof (struct ptf_buf_element), GFP_ATOMIC);
	if (!element)
		printk(KERN_WARNING "fail to create ptf buffer element\n");
	
	else{
	  //printk(KERN_WARNING "element_create():before alloc cp_data atomic, len=%d\n",len);
	  
		cp_data=kmalloc(sizeof(unsigned char)*len,GFP_ATOMIC);
		if (!cp_data){
		  printk(KERN_WARNING "cp_data:fail to create ptf buffer element-char buffer\n");
		  kfree(element);
	/* 	  printk(KERN_WARNING "fail to create ptf buffer element-char buffer\n"); */
			return NULL;
		}
		memcpy(cp_data,data,len);
		element->data=cp_data;
		element->cur_pos=element->data;	//init cur_pos to head
		element->length=len;
		}
	
	return element;
}

// ptf_buf_element_destory() --destory a ptf_buf_element. recycle all mem space
//element : element to destory
//destroyed element should be dequeue before call this function.
static void  ptf_buf_element_destory(struct  ptf_buf_element * element){
	if (!element){
		printk(KERN_WARNING "try to destory null ptf buffer element\n");
		return;
		}
	if(element->data)
		kfree(element->data);	//how to tell data in stack, from char [] ???
	kfree(element);
}
//ptf_buf_enqueue() insert an element into a queue's head (in fact, the second element in queue
//queue: which queue is inserted by new element
//element : new element to queue
static void ptf_buf_enqueue (struct ptf_buf_queue * queue, struct ptf_buf_element * element){
	if ((!element)||(!queue))
		{
		printk(KERN_WARNING "invalid queue or element when enqueue\n");
		return;
		}

	//add new element just after queue's head
	list_add(&(element->list_node),&(queue->queue_head));	
	queue->cur_len+=element->length;
}

//ptf_buf_is_empty() --judge a queue is empty
//queue: which queue to operation
//return : empty return 1, no empty return 0
 int ptf_buf_is_empty(struct ptf_buf_queue * queue){
	if(queue->queue_end.prev==&(queue->queue_head))
		return 1;
	else 
		return 0;
}


//ptf_buf_dequeue() --pop last element out list before queue end but not destory it
//queue: which queue for operation
//return: pointer to poped out element, NULL if queue is empty or error
//need check queue is empty.

static struct ptf_buf_element * ptf_buf_dequeue(struct ptf_buf_queue * queue){
	struct ptf_buf_element * element=NULL;
	if(ptf_buf_is_empty(queue))	//queue empty, just head and end two nodes
		return element;

	//get last valid data emlement before end of queue
	element=list_entry(queue->queue_end.prev,struct ptf_buf_element, list_node);

	//del this element in list

	list_del(&(element->list_node));
	queue->cur_len-=element->length;
	return element;	//return element for recycle	
}

//ptf_get_last_data_element() return last valid data element, normally just before queue's end.
//queue: which queue to operation
//return: no valid data, NULL; pointer to valid data before end of queue
static struct ptf_buf_element * ptf_buf_get_last_data_element(struct ptf_buf_queue * queue){
	if(ptf_buf_is_empty(queue))
		return NULL;
	else
		return list_entry(queue->queue_end.prev,struct ptf_buf_element, list_node);
}


//ptf_buf_get_data() return data to caller with size caller provides.
//data : output buffer 
//len : how many bytes caller request
//queue : which queue to operation
//user_flag: 1 if data is in user space, 0 if data in kernel space.
//return : count how manu bytes read in fact.
//note : always start receive at cur_pos of last data element before end of queue.
//if all data in current element is used, free the whole elelment and dequeue it.




int ptf_buf_get_data(unsigned char * data, size_t len, struct ptf_buf_queue * queue, int user_flag){
 	struct ptf_buf_element * cur_element;	//current processed element
 	struct ptf_buf_element * tmp_element;
	unsigned char * data_pos=data;	//dest buffer current pos
	size_t cnt=len;	//count for loop
	size_t cur_left;	//left unread length in current element
	size_t read_cnt=0;	//return value, indicates how many bytes are read

	while(cnt>0){
		//queue is empty
	  if(ptf_buf_is_empty(queue))
			break;
		
		cur_element=ptf_buf_get_last_data_element(queue);
		
		cur_left=cur_element->length-(cur_element->cur_pos-cur_element->data);
		
		//current element has enough data...
		if (cnt<cur_left){
			if(user_flag)
				copy_to_user(data_pos,cur_element->cur_pos,cnt);
			else
				memcpy(data_pos,cur_element->cur_pos,cnt);
			
			cur_element->cur_pos+=cnt;	//update current read position in current element

			read_cnt+=cnt;

			//return read_cnt;
			//all data finished, exit loop, or in fact there is cnt-=cnt;
			break;
		}

		//current element has not enough data or just have same length data...
		else{
			if(user_flag)
				copy_to_user(data_pos,cur_element->cur_pos,cur_left);
			else
				memcpy(data_pos,cur_element->cur_pos,cur_left);

			data_pos+=cur_left;	//move dest buf position

			cnt-=cur_left;

			read_cnt+=cur_left;
			//free whole element
			tmp_element=ptf_buf_dequeue(queue);
			//sanity check, normally current element is the last data element in queue
			//??? it is late, last element has been dequeued if wrong
			if(tmp_element!=cur_element){
				printk(KERN_WARNING "dequeue element != current data element, logic error\n");
				//return read_cnt;
				break;
				}
			ptf_buf_element_destory(tmp_element);
	
			}
		
	}
		return read_cnt;	

 	}

 //ptf_buf_put_data() --put a piece data to queue, in fact create a new element and insert it in list
 //data: pointer to data
 //len :data's length should not be 0
 //queue: queue to operation
 //return: success, 0; error <0
 int ptf_buf_put_data(unsigned char * data, size_t len, struct ptf_buf_queue * queue){
 	struct ptf_buf_element * element;
	if(!queue){
		printk(KERN_WARNING "invalid queue or element when enqueue\n");
		return -EINVAL;
		}
	if(len>queue->max_size){
		printk(KERN_WARNING "received package is too large\n");
		return -EINVAL;
		}


	if(!len){		//ptf_read() will not be waked if block  mode
		printk(KERN_WARNING "error to put zero len data in queue\n");
		return -EINVAL;
		}
		
	//delete oldest in queue when no enough room in queue;
	//judge > instead of >=, zero length impossible

	if(queue->cur_len+len>queue->max_size)
		printk(KERN_WARNING "Warning: PTF device  buffer overflowed\n");	//test team's request
	
	while(queue->cur_len+len>queue->max_size){
		
		 struct ptf_buf_element * del_element;

		if(!(del_element=ptf_buf_dequeue(queue)))	{
			
			//it should not mean queue is empty but an error, becasue queue's cur_size is 0 when last element is 
			//deleted and we have made sure package len less than queue's max size
			printk(KERN_WARNING "Error: null element in queue\n");
			return -EINVAL;
			}

			//break; //queue empty and we have make sure package len less than max size, just go on
		else{
			
		//	queue->cur_len-=del_element->length; add this in dequeue 
			ptf_buf_element_destory(del_element);
			
			}
		}
	
	
	element=ptf_buf_element_create(data, len);
	
	if(!element)
		return -EINVAL;
	ptf_buf_enqueue( queue, element);
	//queue->cur_len+=len;add this in enqueue 
	return 0;
 	}

//ptf_buf_queue_release() -release all data elements of a queue, until just left queue head and end
//queue: queue to operation

 void ptf_buf_queue_release(struct ptf_buf_queue * queue ){
	struct ptf_buf_element * element;
	
	while (!ptf_buf_is_empty(queue)){
		element=ptf_buf_dequeue(queue);

		if(!element){
		printk(KERN_WARNING "invalid element when release queue\n");
		return;
		}
		
		ptf_buf_element_destory(element);

		}

 	}
