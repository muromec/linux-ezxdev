/*
	linux/net/ipv4/lids_check_scan.c  
	
	Copy right by Huagang Xie(xie@gnuchina.org) for LIDS Project

	200.5.17	add some pointer checkers.
	2000.5.6 	fixed some condition contest.
	2000.4.3	initially released 
*/

#include <linux/lids.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/config.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/random.h>
#include <linux/init.h>
#include <linux/ipsec.h>

#include <net/icmp.h>
#include <net/tcp.h>
#include <net/ipv6.h>

#include <asm/segment.h>

#include <linux/inet.h>
#include <linux/stddef.h>
#include <linux/timex.h>



#define LIDS_WARNING	10
#define LIDS_SCAN_TIMEOUT 3*HZ
#define LIDS_SCAN_TIME 10*HZ

struct lids_scan 
{
	struct lids_scan *next,*pre;
	__u32 addr;
	unsigned long counter;
	unsigned long lower_counter;
	unsigned long create_time;
	spinlock_t   lock;
	struct timer_list timer;
}lids_scan_head;

void free_scan(struct lids_scan *p)
{
	LIDS_DBG("free %d.%d.%d.%d\n",NIPQUAD(p->addr));
	del_timer(&(p->timer));

	spin_lock(&(p->lock));
	spin_lock(&(p->pre->lock));
	p->pre->next = p->next;
	spin_unlock(&(p->pre->lock));
	if(p->next) {
		spin_lock(&(p->next->lock));
		p->next->pre = p->pre;
		spin_unlock(&(p->next->lock));
	}
	spin_unlock(&(p->lock));

	kfree(p);
	LIDS_DBG("out\n");
}
/*
 *
 */ 
static void lids_proceed_scan(unsigned long __data)
{
	struct lids_scan *p=(struct lids_scan *)__data;
	unsigned long current_time = jiffies;
	unsigned long counter;
	unsigned long lower_counter;
	__u32 addr;


	if(p) {
		if(p->counter > LIDS_WARNING ) { 
			addr=p->addr;
			counter=p->counter;
			lower_counter=p->lower_counter;
		
			lids_security_alert("Port scan detected: %d.%d.%d.%d scanned %ld closed ports including %ld ports < 1024)", NIPQUAD(addr),counter, lower_counter);
			free_scan(p);
		}
		else {
			if(current_time - p->create_time > LIDS_SCAN_TIME)
				free_scan(p);
			else 
				mod_timer(&(p->timer),  LIDS_SCAN_TIMEOUT+current_time);
		}
	}
	LIDS_DBG("exit\n");
}
/*
 *
 */ 
struct lids_scan * lids_find_scan(__u32 addr)
{
	struct lids_scan *p ;
	
	LIDS_DBG("checking %d.%d.%d.%d\n",NIPQUAD(addr));

	for(p=lids_scan_head.next;p!=NULL;p=p->next) {
	LIDS_DBG("searching exist: %d.%d.%d.%d\n",NIPQUAD(p->addr));
		if(p->addr == addr) {
			LIDS_DBG("exit from return not NULL\n");
			return p;
		}
	}	
	
	LIDS_DBG("exit from return NULL\n");
	return NULL;
}
/*
 *	lids_check_scan ...
 */ 
int lids_check_scan(__u32 addr,__u16 port) 
{
	struct lids_scan *p,*p1;
	unsigned long current_time=jiffies;

	LIDS_DBG("%d.%d.%d.%d : %d\n",NIPQUAD(addr),port);
	if((p = lids_find_scan(addr)) == NULL) {

		p1 = &lids_scan_head;
		p = (struct lids_scan*)kmalloc(sizeof(struct lids_scan),GFP_ATOMIC);
		if(p == NULL ) {
			LIDS_DBG("kmalloc error, return -1\n");
			return -1;
		}
		spin_lock_init(&(p->lock));
		
		while((p1->next)!=NULL)p1=p1->next;
	
		spin_lock(&(p1->lock));
		p1->next = p;
		spin_unlock(&(p1->lock));

		spin_lock(&(p->lock));
		p->pre = p1;
		spin_unlock(&(p->lock));

		p->next = NULL;
		p->addr = addr;
		p->counter = 0;	
		p->lower_counter = 0;
		p->create_time = current_time;

		init_timer(&(p->timer));
		p->timer.expires = LIDS_SCAN_TIMEOUT + current_time;
		p->timer.data = (unsigned long) p;
		p->timer.function = &lids_proceed_scan; 
		add_timer(&(p->timer));
	}
	spin_lock(&(p->lock));
	(p->counter)++;
	if(port < 1024)
		(p->lower_counter)++;
	spin_unlock(&(p->lock));
	LIDS_DBG("exit return 0\n");
	return 0;
}
void lids_port_scanner_detector_init(void)
{
	static int initialized=0;
	if(!initialized)spin_lock_init(&(lids_scan_head.lock));
	initialized=1;
}
