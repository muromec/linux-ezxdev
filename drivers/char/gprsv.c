/*
 * gprsv.c
 *
 * GPRS Interface for Linux.
 *      This module implemented a tty line discipline and a virtual network 
 *      device. The macro NR_LDISCS defined in tty.h should be changed to 17, and in 
 *      termios.h and a new line discipline is defined(#define N_GPRS 16).
 *
 * Copyright (C) 2002, 2005 - Motorola
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
 * Change History:
 *     December 2002, created.
 *     March 2005, add support for modem bridge.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/config.h>
#include <linux/tty.h>
#include <linux/kmod.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/devfs_fs_kernel.h>
#include <linux/netdevice.h>
#include <linux/poll.h>
#include <linux/filter.h>
#include <linux/rtnetlink.h>
#include <linux/if_arp.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/spinlock.h>
#include <linux/smp_lock.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>

#include "modem_bridge.h"
#ifdef ENABLE_DNS_BROADCAST
#include <net/ip.h>
#include <net/udp.h>
#include <net/checksum.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#endif

#include "if_gprsv.h"
/*
 * #if CONFIG_MODVERSIONS==1
 * #define MODVERSIONS
 * #include <linux/modversions>
 * #endif
 */
#define GPRS_HDRLEN 0
#define GPRS_MTU    1500
#define GPRS_MAXTX  10  /* the maxium buffered skb packets */

/* Bit numbers in xmit_flags */
#define XMIT_BUSY       2
#define RECV_MID        0

#define CTL_FLAG        0x7e
#define CTL_ESC         0x7d

#define OBUFSIZE        255

#define PUT_BYTE(buf, c)    do {            \
    if ( c == CTL_FLAG || c == CTL_ESC ) {  \
        *buf++ = CTL_ESC;                   \
    }                                       \
    *buf++ = c;                             \
} while (0)

struct gprs_struct {
    struct list_head list;

    struct net_device *dev; /*network device instance it attached*/
    struct tty_struct *tty; /*tty device instance it attached*/
    
    struct sk_buff_head xq; /*transmission queue*/

    struct sk_buff *xmit_pending; /* a packet ready to go out */
    int tpkt_pos;
    unsigned char *optr;
    unsigned char *olim;
    unsigned char obuf[OBUFSIZE];
    
    struct sk_buff_head rq; /*receive queue*/
    unsigned char * inp;
    /*short iplen;*/
    unsigned long recv_flags;
    unsigned char lastin;

    unsigned long xmit_flags;
    spinlock_t  xl; /* transmission lock to protect output queue */
    spinlock_t  rl; /* transmission lock to protect input buffer */

    struct net_device_stats stats;  /* statistics */
    unsigned long last_xmit;
    unsigned long last_recv;

    int index; /* interface unit */
    
    int bridge_index;
    int peer_index;
};

static LIST_HEAD(all_gprsUnits);

static int gprs_push(struct gprs_struct *gprsv);

#ifdef LOGPACKET
static int
print64byte(int debugPoint, char * devname, struct sk_buff *skb)
{
    unsigned char left[52];
    unsigned char right[20];
    unsigned char *buf;
    int i, len, length;
    int leftindex;

    if( skb == NULL ) return -1;
    buf = skb->data;

    if( debugPoint == 0 )
        printk(KERN_INFO "\nSend IP packet through %s at %lu\n", devname, jiffies);
    else
        printk(KERN_INFO "\nRecv IP packet through %s at %lu\n", devname, jiffies);

    /* length = (skb->len > 64) ? 64 : skb->len; */
    length = skb->len;

    len = leftindex = 0;
    for(i = 0; i < length; i++)
    {
        if(len == 16)
        {
            left[leftindex] = right[len] = '\0';
            printk(KERN_INFO "%s   %s\n", left, right);
            leftindex = len = 0;
        }
        else if(len == 8)
        {
            left[leftindex++] = ' ';
        }

        left[leftindex++] = (buf[i]/16 > 9) ? ('A'+buf[i]/16-10) : ('0'+buf[i]/16);
        left[leftindex++] = (buf[i]%16 > 9) ? ('A'+buf[i]%16-10) : ('0'+buf[i]%16);
        left[leftindex++] = ' ';

        right[len++] = (32 <= buf[i] && buf[i] <= 126)? buf[i] : '.';
    }

    if(len > 0)
    {
        for(i = len; i < 16; i++)
        {
            if( i == 8 )
            {
                left[leftindex++] = ' ';
            }
            left[leftindex++] = ' ';
            left[leftindex++] = ' ';
            left[leftindex++] = ' ';
        }
        left[leftindex] = right[len] = '\0';
        printk(KERN_INFO "%s   %s\n", left, right);
    }

    printk(KERN_INFO "\n");

    return 0;
}
#endif

static int
gprs_async_encode(struct gprs_struct *gprsv)
{
    int i, count, c;
    unsigned char *buf, *buflim;
    unsigned char *data;

    buf = gprsv->obuf;
    gprsv->olim = buf;
    gprsv->optr = buf;
    i = gprsv->tpkt_pos;
    data = gprsv->xmit_pending->data;
    count = gprsv->xmit_pending->len;

    if( i == 0 ) 
        *buf++ = CTL_FLAG;

    /*
     * Once we put in the last byte, we need to put in the closing flag,
     * so make sure there is at least 1*2+1 = 3 bytes of free space in 
     * the output buffer.
     */
    buflim = gprsv->obuf + OBUFSIZE - 2;
    while (i < count && buf < buflim) {
        c = data[i++];
        PUT_BYTE(buf, c);
    }

    gprsv->tpkt_pos = i;
        
    if (i < count) {
        /*
         * Remember where we are up to in this packet.
         */
        gprsv->olim = buf;
        return 0;
    }

    /*
     * We have finished the packet.  Add the Ending flag.
     */
    *buf++ = CTL_FLAG;
    gprsv->olim = buf;

    return 1;
}

/*
 * Called to do any work queued up on the transmit side
 * that can now be done.
 */
static void
gprs_xmit_process(struct gprs_struct *gprsv)
{
    struct tty_struct * tty = gprsv->tty;
    struct sk_buff *skb;

    if (test_and_set_bit(XMIT_BUSY, &gprsv->xmit_flags))
        return;

    spin_lock_bh(&gprsv->xl);

    gprs_push(gprsv);
    
    while (gprsv->xmit_pending == 0
           && (skb = skb_dequeue(&gprsv->xq)) != 0)
    {
        gprsv->xmit_pending = skb;
        gprsv->tpkt_pos = 0;
        gprs_async_encode(gprsv);
        gprs_push(gprsv);
    }

    /* 
     * If there's no work left to do, tell the core net
     * or bridge peer that we can accept some more. 
     */
    if ( gprsv->xmit_pending == 0 && skb_peek(&gprsv->xq) == 0 )
    {
        clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);

        if( gprsv->bridge_index != -1 && gprsv->peer_index != -1 )
        {
            modem_bridge_wakeup_peer(gprsv->peer_index);
        }
        else if( gprsv->dev != 0)
        {
            netif_wake_queue(gprsv->dev);
        }
    }
    else
    {
        set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
    }

    spin_unlock_bh(&gprsv->xl);
    
    clear_bit(XMIT_BUSY, &gprsv->xmit_flags);   
}

/*
 * bridge interfaces
 */
static int
gprs_bridge_start_xmit(void * pdata, int len, void * priv)
{
    struct gprs_struct *gprsv = (struct gprs_struct *)priv;
    struct sk_buff * skb;

    (void)len;

    if( !pdata ) return -1;

    skb = (struct sk_buff *)pdata;

    if( !gprsv ) return -1;

    if( skb_queue_len(&gprsv->xq) >= GPRS_MAXTX )
    {
        if( modem_bridge_stop_peer( gprsv->peer_index ) == -1 )
        {
            /* peer do not support flow control */
            kfree_skb( skb );
            return -1;
        }
    }
    skb_queue_tail(&gprsv->xq, skb);

    gprs_xmit_process(gprsv);
    return 0;
}

/*
 * peer use this function to turn on our flow control
 */
static int 
gprs_bridge_stop_recv(void * priv)
{
    struct gprs_struct * gprsv = (struct gprs_struct *)priv;
    struct tty_struct * tty;

    if( !gprsv ) return -1;

    tty = gprsv->tty;

    if (!test_and_set_bit(TTY_THROTTLED, &tty->flags)
        && tty->driver.throttle )
    {
        tty->driver.throttle(tty);
    }
    return 0;
}

/*
 * to turn off the flow control
 */
static int
gprs_bridge_wakeup_recv(void * priv)
{
    struct gprs_struct * gprsv = (struct gprs_struct *)priv;
    struct tty_struct * tty;

    if( !gprsv ) return -1;

    tty = gprsv->tty;

    if (test_and_clear_bit(TTY_THROTTLED, &tty->flags)
        && tty->driver.unthrottle)
    {
        tty->driver.unthrottle(tty);
    }

    return 0;
}

static struct modem_bridge_ops gprs_bridge_ops = {
    gprs_bridge_start_xmit,
    gprs_bridge_stop_recv,
    gprs_bridge_wakeup_recv
};

/*
 * Network interface unit routines.
 */
static int
gprs_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct gprs_struct *gprsv = (struct gprs_struct *)dev->priv;

#ifdef LOGPACKET
    print64byte(0, dev->name, skb);
#endif

    netif_stop_queue(dev);
    skb_queue_tail(&gprsv->xq, skb);
    gprs_xmit_process(gprsv);
    return 0;
}

/*just for /proc file system to record the statistic information: 
   sprintf_stats() in dev.c will call it*/
static struct net_device_stats *
gprs_net_stats(struct net_device *dev)
{
    struct gprs_struct *gprsv = (struct gprs_struct *) dev->priv;

    if(gprsv)
        return &gprsv->stats;
    else
        return 0;
}

static int
gprs_net_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
    struct gprs_struct *gprsv = (struct gprs_struct *) dev->priv;
    struct gprsv_stats stats;
    struct gprsv_idle idle;
    
    /*
     * !!!Remember
     * user space program should set ifreq.ifR_ifru.ifru_data pointer to 
     * the address of ifgprsvstatsreq.stats or ifgprsvidlesreq.idle before 
     * calling ioctl, so kernel(here) can copy data to ifgprsvstatsreq.stats,
     * or ifgprsvidlesreq.idle otherwise I will fly.
     */
    void *addr = (void *) ifr->ifr_ifru.ifru_data; 
    
    int err = -EFAULT;

    if(!gprsv) 
    {
        printk(KERN_ERR "GPRSV: in gprs_net_ioctl gprsv is null\n");
        return -EFAULT;
    }

    switch (cmd) 
    {
    case SIOCGGPRSVSTATS:
        /*get GPRS interface status*/
        stats.gprsv_ibytes = gprsv->stats.rx_bytes;
        stats.gprsv_obytes = gprsv->stats.tx_bytes;
        if (copy_to_user(addr, &stats, sizeof(stats)))
        {
            printk(KERN_ERR "GPRSV: copy in/out bytes error\n");
            break;
        }
        err = 0;
        break;

    case SIOCGGPRSIDLETIME:
        /*get GPRS idle time*/
        /*
         * the linger time is kept in user space process, 
         * used to decide if the link should be shut down
	 * when phone got into sleep, jiffies will not change
	 * use xtime instead to deal with this issue
         */
        idle.xmit_idle = xtime.tv_sec - gprsv->last_xmit;
        idle.recv_idle = xtime.tv_sec - gprsv->last_recv;
        if (copy_to_user(addr, &idle, sizeof(idle)))
        {
            printk(KERN_ERR "GPRSV: get gprs idle time error\n");
            break;
        }
        err = 0;
        break;

    default:
        printk(KERN_ERR "GPRSV: unknown ioctl command\n");
        err = -EINVAL;
    }

    return err;
}

static int
gprs_net_init(struct net_device *dev)
{
    dev->hard_header_len = GPRS_HDRLEN;
    dev->mtu             = GPRS_MTU;
    dev->hard_start_xmit = gprs_start_xmit;
    dev->get_stats       = gprs_net_stats;
    dev->do_ioctl        = gprs_net_ioctl;
    dev->addr_len        = 0;
    dev->tx_queue_len    = GPRS_MAXTX;
    dev->type            = ARPHRD_PPP;
    dev->flags           = IFF_POINTOPOINT | IFF_NOARP | IFF_MULTICAST;
    return 0;
}

/*
 * Push as much data as possible out to the mux-tty. This function perhaps can
 * be reentried by mux's bottom half. 
 */
static int gprs_push(struct gprs_struct *gprsv)
{
    int avail, sent = 0;
    struct tty_struct * tty = gprsv->tty;

    if( gprsv->xmit_pending == 0 ) return 0;

    while( gprsv->olim > gprsv->optr )
    {
        avail = gprsv->olim - gprsv->optr;
        set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
        sent = tty->driver.write(tty, 0, gprsv->optr, avail);
        if( sent > 0 )
        {
            /*record the last transmission timestamp*/
            gprsv->last_xmit = xtime.tv_sec;
            gprsv->stats.tx_bytes += sent;

            gprsv->optr += sent;
            if( gprsv->optr >= gprsv->olim )
            {
                if( gprsv->tpkt_pos >= gprsv->xmit_pending->len )
                {
                    kfree_skb(gprsv->xmit_pending);
                    gprsv->xmit_pending = 0;
                    return 0;
                }
                else
                {
                    gprs_async_encode( gprsv );
                }
            }
            continue;
        }
        else if( sent < 0 )
        {
            kfree_skb(gprsv->xmit_pending);
            gprsv->xmit_pending = 0;
            return -1;
        }
        else
            return 0;
    }
    return 0;
}

static spinlock_t all_gprs_lock = SPIN_LOCK_UNLOCKED;
static int gprs_create_interface(struct gprs_struct * gprsv, int unit, int *retp)
{
    struct gprs_struct * pTemp;
    struct net_device *dev;
    struct list_head *list;
    int last_unit = -1;
    int ret = -ENODEV;

    spin_lock( &all_gprs_lock );

    /* create a new GPRSV unit */
    if( gprsv == 0 ) goto out;

    ret = -EEXIST;
    if( gprsv->index != -1 ) goto out;

    list = &all_gprsUnits;

    while( (list = list->next) != &all_gprsUnits )
    {
        pTemp = list_entry(list, struct gprs_struct, list);
        if( (unit < 0 && pTemp->index > last_unit + 1)
            || (unit >= 0 && unit < pTemp->index) )
        {
            break;
        }
        if( unit == pTemp->index ) goto out;
        last_unit = pTemp->index;
    }
    if( unit < 0 ) unit = last_unit + 1;

    ret = -ENOMEM; 
    dev = kmalloc(sizeof(struct net_device), GFP_KERNEL);
    if(dev == 0)
    {
        printk(KERN_ERR "GPRSV: can not get memory for network device\n");
        goto out;
    }

    skb_queue_head_init(&gprsv->xq);
    skb_queue_head_init(&gprsv->rq);

    gprsv->last_xmit = gprsv->last_recv = xtime.tv_sec;

    memset(dev, 0, sizeof(struct net_device));
    dev->init = gprs_net_init;
    snprintf(dev->name, sizeof(dev->name)-1, "gprsv%d", unit);
    dev->features |= NETIF_F_DYNALLOC;

    gprsv->dev = dev;
    dev->priv = gprsv;

    rtnl_lock();
    ret = register_netdevice(dev);
    rtnl_unlock();
    if (ret != 0)
    {
        kfree(dev);
        goto out;
    }

    netif_start_queue(dev);

    gprsv->index = unit;
    
    list_add(&gprsv->list, list->prev);

out:
    spin_unlock( &all_gprs_lock );
    *retp = ret;
    if(ret != 0) return 1;
    return 0;
}

/*
 * TTY line discpline: GPRS line discpline, the macro NR_LDISCS defined in 
 * tty.h should be changed to 17, and in termios.h, a new line discipline 
 * macro should be defined: #define N_GPRS 16
 */

/*
 * Routines implementing the gprs line discipline.
 */

/*
 * Called when a tty is put into GPRS line discipline.
 */
static int
gprs_ldisc_open(struct tty_struct *tty)
{
    struct gprs_struct *gprsv;
    int err;

    MOD_INC_USE_COUNT;

    err = -ENOMEM;
    gprsv = kmalloc(sizeof(struct gprs_struct), GFP_KERNEL);
    if(gprsv == 0)
    {
        printk(KERN_ERR "GPRSV: can not get memory for gprsv\n");
        goto errout;
    }
    memset(gprsv, 0, sizeof(struct gprs_struct));

    tty->disc_data = gprsv;

    gprsv->tty = tty;

    gprsv->index = -1;
    gprsv->bridge_index = -1;
    gprsv->peer_index = -1;

    /*
     * we first link GPRSV and TTY here, we will create GPRS interface 
     * or bridge interface when ioctl() is called. 
     */

    return 0;

errout:
    MOD_DEC_USE_COUNT;
    return err;
}

/*
 * Called when the tty is put into another line discipline
 * or it hangs up.
 * We assume that while we are in this routine, the tty layer
 * won't call any of the other line discipline entries for the
 * same tty.
 */
static void
gprs_ldisc_close(struct tty_struct *tty)
{
    struct gprs_struct *gprsv;
    struct net_device *dev;

    MOD_DEC_USE_COUNT;

    gprsv = (struct gprs_struct *)tty->disc_data ;
    tty->disc_data = 0;

    if (gprsv) 
    {
        if( gprsv->dev )
        {
            spin_lock( &all_gprs_lock );
            list_del( &gprsv->list );
            spin_unlock( &all_gprs_lock );

            dev = (struct net_device *)gprsv->dev;
            rtnl_lock();
            netif_stop_queue(dev);
            dev_close(dev);
            unregister_netdevice(dev);
            rtnl_unlock();
        }

        if( gprsv->bridge_index != -1 )
        {
            if( gprsv->peer_index != -1 )
            {
                modem_bridge_unbind_device(gprsv->bridge_index, gprsv->peer_index);
                gprsv->peer_index = -1;
            }
            modem_bridge_unregister_device( gprsv->bridge_index );
            gprsv->bridge_index = -1;
        }

        skb_queue_purge(&gprsv->xq);
        skb_queue_purge(&gprsv->rq);
        if( gprsv->xmit_pending )
        {
            kfree_skb(gprsv->xmit_pending);
        }
        kfree(gprsv);
    }
}

/*
 * Read does nothing - no data is ever available this way(tty file).
 * gprs data only via network.
 */
static ssize_t
gprs_ldisc_read(struct tty_struct *tty, struct file *file,
          unsigned char *buf, size_t count)
{
    /*
     * to remove kloc warning
     */ 
    (void)tty; (void)file; (void)buf; (void)count;

    return -EAGAIN;
}

/*
 * Write does nothing - no data is ever available this way(tty file).
 * gprs data only via network.
 */
static ssize_t
gprs_ldisc_write(struct tty_struct *tty, struct file *file,
           const unsigned char *buf, size_t count)
{
    /*
     * to remove kloc warning
     */ 
    (void)tty; (void)file; (void)buf; (void)count;

    return -EAGAIN;
}

static int
gprs_ldisc_ioctl(struct tty_struct *tty, struct file *file,
           unsigned int cmd, unsigned long arg)
{
    struct gprs_struct *gprsv = tty->disc_data;

    int unit = 0, err = -EFAULT;

    /*
     * to remove kloc warning
     */
    (void)file;

    switch(cmd)
    {
    case GPRSVIOCNEWBRDEVICE:
        if( gprsv->index != -1 || gprsv->bridge_index != -1 )
        {
            /* already create network interface or bridge device for this port */
            err = -EEXIST;
            break;
        }

        gprsv->bridge_index = modem_bridge_register_device(gprsv, &gprs_bridge_ops);
        if( gprsv->bridge_index >= 0 )
        {
            err = -EFAULT;
            if ( put_user(gprsv->bridge_index, (int*)arg) ) break;
            err = 0;

            skb_queue_head_init(&gprsv->xq);
            skb_queue_head_init(&gprsv->rq);
            break;
        }

        gprsv->bridge_index = -1;
        err = -ENOMEM;
        break;
        
    case GPRSVIOCBINDPEER:
        /* 
         * should first create bridge device, 
         * then to bind to peer 
         */
        if( gprsv->bridge_index == -1 ) break;
        
        if( get_user(unit, (int*)arg) ) break;

        /*
         * we only check if unit >= 0, more check will 
         * be made in bridge module
         */
        if( unit < 0 ) break;

        err = -ENOENT;
        if( modem_bridge_bind_device(gprsv->bridge_index, unit) ) break;

        gprsv->peer_index = unit;
        err = 0;
        break;
                
    case GPRSVIOCNEWUNIT:
        if( gprsv->bridge_index != -1 )
        {
            err = -EEXIST;
            break;
        }
        
        if ( get_user(unit, (int*)arg) ) break;

        if ( gprs_create_interface(gprsv, unit, &err) ) break;

        err = -EFAULT;
        if ( put_user(gprsv->index, (int*)arg) ) break;
        err = 0;
        break;

    default:
        err = -ENOIOCTLCMD;
    }

    return err;
}

/* No kernel lock - fine */
static unsigned int
gprs_ldisc_poll(struct tty_struct *tty, struct file *file, poll_table *wait)
{
    /*
     * to remove kloc warning
     */ 
    (void)tty; (void)file; (void)wait;

    return 0;
}

static int
gprs_ldisc_room(struct tty_struct *tty)
{ 
    /* if throttled, don't receive from lower layer */
    if (test_bit(TTY_THROTTLED, &tty->flags))
    {
        return 0;
    }

    return 65535;
}

static void
gprs_ldisc_receive(struct tty_struct *tty, const unsigned char *buf,
          char *flags, int count)
{
    struct gprs_struct *gprsv = (struct gprs_struct *)tty->disc_data;
    struct sk_buff *skb = 0;
    int f,b = 0,i;
    unsigned char * tbuf = 0;
    int canup = 0;
    int whole = 1;

    if(gprsv == 0) return;
    
    if(gprsv->dev == 0 && (gprsv->bridge_index == -1 || gprsv->peer_index == -1))
    {
        /* not in network device mode & not in bridge device mode */
        return;
    }

    if(!(count>0)) return;

    spin_lock_bh(&gprsv->rl);

    f = 0;
    if (flags != 0) 
    {
        /* check the flags to see if any char had an error */
        for (i = 0; i < count; ++i)
        {
            if ((f = flags[i]) != 0) break;
        }
    }
    if (f != 0)  goto errret;
    
    if( test_bit(RECV_MID,&gprsv->recv_flags) )
    {
        /*
         * got unfinished packet at previous time
         */
        skb = skb_peek_tail(&gprsv->rq);
        tbuf = skb->tail;

        /*
         * b is offset to put new data
         */
        b = skb->len;
        whole = 0;
    }

    for(i=0; i<count; i++)
    {
        if( skb == 0 )
        {
            if( buf[i] != CTL_FLAG && gprsv->lastin != CTL_FLAG )
            {
                continue;
            }

            skb = dev_alloc_skb(GPRS_MTU);
            if(!skb) goto errret;

            skb_queue_tail(&gprsv->rq,skb);

            tbuf = skb->data;

            b = 0;
            whole = 0;
            if( buf[i] == CTL_FLAG ) continue;
        }

        if( gprsv->lastin == CTL_ESC )
        {
            gprsv->lastin = 0;
            if( (b < GPRS_MTU) && (tbuf != 0) )
            {
                tbuf[b++] = buf[i];
            }
        }
        else
        {
            if( buf[i] == CTL_ESC )
            {
                gprsv->lastin = CTL_ESC;
                continue;
            }
            else if( buf[i] == CTL_FLAG )
            {
                gprsv->lastin = CTL_FLAG;
                if( b > 0 )
                {
                    /*finished the whole IP packet*/
                    whole = 1;
                    skb->len = 0;
                    gprsv->stats.rx_bytes += b;
                    skb_put(skb,b);
                    canup = 1;
                    b = 0;
                    skb = 0;
                    if( i<count-1 )
                    {
                        skb = dev_alloc_skb(GPRS_MTU);    
                        if(!skb) goto errret;

                        /*First in at head,last in at tail*/
                        skb_queue_tail(&gprsv->rq,skb);
                        tbuf = skb->data;
                        whole = 0;
                    }
                }
            }
            else
            {
                if( (b < GPRS_MTU) && (tbuf != 0) )
                    tbuf[b++] = buf[i];
            }
        }
    }/*end of for ...*/

    if( !whole )
    {
        set_bit(RECV_MID,&gprsv->recv_flags);
        if( skb != NULL ) skb->len = b;
    }
    else
        clear_bit(RECV_MID,&gprsv->recv_flags);

    if(canup)
    {
        /*now, we can up at least one ip packet to network layer*/
        skb = skb_dequeue(&gprsv->rq);

        while(skb)
        {
            if (gprsv->dev && !((gprsv->dev->flags & IFF_UP) == 0))
            {
#ifdef LOGPACKET
                print64byte(1, gprsv->dev->name, skb);
#endif
                skb->dev = gprsv->dev;
                skb->protocol = htons(ETH_P_IP);
                skb->mac.raw = skb->data;
                netif_rx(skb);
    
                /*record the last reception time stamp*/
                gprsv->last_recv = xtime.tv_sec;
            }
            else if( (gprsv->bridge_index != -1) && (gprsv->peer_index != -1) )
            {
                skb->protocol = htons(ETH_P_IP);
                skb->mac.raw = skb->data;            
                modem_bridge_send2peer( gprsv->peer_index, skb, 0 );
            }
            else
            {
                kfree_skb(skb);
            }
                
            /*the last skb is not an entire ip packet when RECV_MID is set.*/
            if( (!whole && skb_queue_len(&gprsv->rq)==1)||skb_queue_empty(&gprsv->rq) )
                break;
            skb = skb_dequeue(&gprsv->rq);
        }
    }

    spin_unlock_bh(&gprsv->rl);

    /*
     * operation needed only when work as a network device
     */
    if( gprsv->bridge_index == -1 )
    {
        if (test_and_clear_bit(TTY_THROTTLED, &tty->flags)
            && tty->driver.unthrottle)
        {
            tty->driver.unthrottle(tty);
        }
    }
    return;

errret:
    printk(KERN_ERR "GPRSV: data error, i can not accept any data!\n");
    spin_unlock_bh(&gprsv->rl);
}

static void
gprs_ldisc_wakeup(struct tty_struct *tty)
{
    struct gprs_struct *gprsv = (struct gprs_struct *)tty->disc_data;

    clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);

    if (gprsv == 0) return;

    gprs_xmit_process(gprsv);
}

static struct tty_ldisc gprs_ldisc = {
    magic:  TTY_LDISC_MAGIC,
    name:   "gprs",
    /*routines called from above tty file interfaces*/
    open:   gprs_ldisc_open,
    close:  gprs_ldisc_close,
    read:   gprs_ldisc_read,
    write:  gprs_ldisc_write,
    ioctl:  gprs_ldisc_ioctl,
    poll:   gprs_ldisc_poll,
    /*routines called by below driver(mux)*/
    receive_room: gprs_ldisc_room,
    receive_buf: gprs_ldisc_receive,
    write_wakeup: gprs_ldisc_wakeup,
};

#ifndef N_GPRS
#define N_GPRS 16
#endif

#ifdef ENABLE_DNS_BROADCAST
static unsigned int 
dns_local_fn(unsigned int hooknum,
    struct sk_buff **pskb,
    const struct net_device *in,
    const struct net_device *out,
    int (*okfn)(struct sk_buff *))
{
    struct net_device * gprs0_device;
    struct net_device * gprs1_device;
    struct sk_buff *skb;
    struct sk_buff *new_skb;
    struct iphdr  *ip;
    struct udphdr *uh;

    /*printk(KERN_INFO "drop into gprsv dns filter\n");*/
    /* escape kloc warning */
    (void)( hooknum );
    (void)( in );
    (void)( out );
    (void)( okfn );

    MOD_INC_USE_COUNT;

    skb = *pskb;

    if( skb->len < sizeof(struct iphdr)
        || skb->nh.iph->ihl * 4 < sizeof(struct iphdr))
    {
        goto __accept;
    }

    ip = skb->nh.iph;

    if( ip->daddr == htonl(0x7F000001L) || ip->protocol != IPPROTO_UDP )
    {
        /*
         * only process udp packets sent to somebody else
         */
        goto __accept;
    }

    uh = (struct udphdr *)((char *)ip + ip->ihl * 4);

    if( uh->dest != htons(53) )
    {
        goto __accept;
    }

    /* process packet sent to DNS well known ports */
    gprs1_device = dev_get_by_name("gprsv1");
    if( !gprs1_device )
    {
        /*
         * not multiple pdp context
         */
        goto __accept;
    }

    gprs0_device = dev_get_by_name("gprsv0");
    if( !gprs0_device )
    {
        dev_put( gprs1_device );
        /*
         * not multiple pdp context
         */
        goto __accept;
    }

    {
        struct rt_key key = { dst:ip->daddr,
            src:0,
            oif:gprs0_device->ifindex,
            tos:RT_TOS(skb->nh.iph->tos),
#ifdef CONFIG_IP_ROUTE_FWMARK
            fwmark:skb->nfmark
#endif
        };
        struct rtable *rt;

        if( ip_route_output_key(&rt, &key) != 0 )
        {
            printk(KERN_INFO "can not find route to gprsv0\n");
            goto __out;
        }

        if( rt->rt_src == ip->saddr )
        {
            ip_rt_put(rt);
            key.oif = gprs1_device->ifindex;
            if( ip_route_output_key(&rt, &key) != 0 )
            {
                printk(KERN_INFO "can not find route to gprsv1\n");
                goto __out;
            }
            if( rt->rt_src == ip->saddr )
            {
                printk(KERN_INFO "failed to find different route for dns packet\n");
                ip_rt_put(rt);
                goto __out;
            }
        }

        /* rt_src != ip->saddr */
        new_skb = skb_copy(skb, GFP_ATOMIC);
        if(new_skb == 0) goto __out;
        ip = new_skb->nh.iph;
        uh = (struct udphdr *)((char *)ip + ip->ihl * 4);
        /* copy one to this interface */
        ip->saddr = rt->rt_src;
        uh->check = 0;
        ip->check = 0;
        ip->check = ip_fast_csum((void *)ip, ip->ihl);
        dst_release(new_skb->dst);
        new_skb->dst = &rt->u.dst;
        new_skb->dst->output(new_skb);
    }

__out:
    dev_put( gprs0_device );
    dev_put( gprs1_device );

__accept:
    MOD_DEC_USE_COUNT;
    return NF_ACCEPT;
}

static struct nf_hook_ops nf_dns_ops
  = { { NULL, NULL }, dns_local_fn, PF_INET, NF_IP_LOCAL_OUT, NF_IP_PRI_FILTER };
#endif

/* Module Functions*/

/*
 * Module init function
 */
static int __init gprs_init(void)
{
    int err;

#ifdef ENABLE_DNS_BROADCAST
    err = nf_register_hook(&nf_dns_ops);
    if( err != 0 )
    {
        printk(KERN_ERR "GPRSV: error %d registering dns filter.\n",err);
        return err;
    }
#endif

    err = tty_register_ldisc(N_GPRS, &gprs_ldisc);
    if (err != 0)
    {
        printk(KERN_ERR "GPRSV: error %d registering line disc.\n",err);

#ifdef ENABLE_DNS_BROADCAST
        nf_unregister_hook(&nf_dns_ops);
#endif

        return err;
    }
    return 0;
}

/*
 * Module clean up function
 */
static void __exit gprs_cleanup(void)
{
#ifdef ENABLE_DNS_BROADCAST
    nf_unregister_hook(&nf_dns_ops);
#endif

    if (tty_register_ldisc(N_GPRS, NULL) != 0)
        printk(KERN_ERR "GPRSV:failed to unregister GPRS line discipline\n");
}

module_init(gprs_init);
module_exit(gprs_cleanup);

MODULE_LICENSE("GPL");
