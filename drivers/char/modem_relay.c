/*   
 * modem_relay.c 
 *
 * Copyright (c) 2005 Motorola  
 *
 * Modem relay is a line discipline for CSD/FAX modem data
 * transfer in kernel module. Two instance of this module makes up a bridge. 
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
 *   Initial version created Jan 2005 
 */
 
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
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
/*
 * #if CONFIG_MODVERSIONS==1
 * #define MODVERSIONS
 * #include <linux/modversions>
 * #endif
 */

/* Bit numbers in xmit_flags */
#define XMIT_BUSY       2
#define RECV_MID        0

#define THRESHOLD      (1024 * 2)
#define BUFSIZE        (1024 * 4)

#define MAX_ESC_CHAR_COUNT 3

#define RELAYVIOCNEWBRDEVICE _IOWR('t', 165, int)   /*create RELAY bridge device*/
#define RELAYVIOCBINDPEER    _IOWR('t', 166, int)   /*bind with another bridge device*/
#define MODEMESCAPESTATUS    _IOWR('t', 167, int)   /*query about escape +++ status*/
#define MODEMESCAPEMONITOR   _IOWR('t', 168, int)   /*enable monitor escape +++ */

struct modem_relay_struct {
    struct tty_struct *tty; /*tty device instance it attached*/
    
    unsigned char buf[BUFSIZE];
    unsigned char *optr;
    unsigned char *iptr;
    int length;
    
    unsigned long recv_flags;
    unsigned long xmit_flags;

    spinlock_t  xrl; /* transmission lock to protect input/output queue */

    int bridge_index;
    int peer_index;

    /* for modem +++ escape */
    unsigned long	last_recv_for_esc;
    u16             esc_char_count;
    u16             monitor_esc;
    int             got_esc;

    /* for flow control */
    u16    bFlowControl; 
};

//static LIST_HEAD(all_ModemRelayUnits);

static int
modem_relay_write2tty(struct tty_struct * tty, char * pout, int len)
{
    int total = 0, sent;
    while( len > 0 )
    {
        sent = tty->driver.write(tty, 0, pout, len);
        if ( sent > 0 )
        {
            total += sent;
            len -= sent;
            pout += sent;
        }
        else
            break;
    }
    return total;
}

/*
 * Called to do any work queued up on the transmit side
 * that can now be done.
 */
static void
modem_relay_xmit_process(struct modem_relay_struct * prelay)
{
    struct tty_struct * tty = prelay->tty;
    int length, tail, sent;

    if (test_and_set_bit(XMIT_BUSY, &prelay->xmit_flags))
        return;

    sent = 0;
    for(;;)
    {
        spin_lock_bh(&prelay->xrl);
        prelay->length -= sent;
        if( prelay->length < 0 )
            prelay->length = 0;
        length = prelay->length;
        spin_unlock_bh(&prelay->xrl);

        if( length <= 0 )
            break;

        tail = prelay->buf + BUFSIZE - prelay->optr;
        if( tail > length )
        {
            sent = modem_relay_write2tty( tty, prelay->optr, length);
        }
        else
        {
            sent = modem_relay_write2tty( tty, prelay->optr, tail);
        }

        if( sent >= tail )
        {
            prelay->optr = prelay->buf;
        }
        else if( sent > 0 )
        {
            prelay->optr += sent;
        }
        else
            break;

    }

    spin_lock_bh(&prelay->xrl);
    length = prelay->length;
    spin_unlock_bh(&prelay->xrl);

    /* 
     * If there's no work left to do, tell the core net
     * or bridge peer that we can accept some more. 
     */
    if( length <= 0 )
    {
        clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);

        if( prelay->bridge_index != -1 && prelay->peer_index != -1 )
        {
            modem_bridge_wakeup_peer(prelay->peer_index);
        }
    }
    else
    {
        set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
    }

    clear_bit(XMIT_BUSY, &prelay->xmit_flags);   
}

/*
 * bridge interfaces
 */
static int
modem_relay_start_xmit(void * pdatain, int len, void * priv)
{
    struct modem_relay_struct * prelay = (struct modem_relay_struct *)priv;
    int length, tail, sent;
    char *pdata = (char *)pdatain;

    if( (!prelay) || (!pdata) || (len <= 0) ) return -1;   

    spin_lock_bh(&prelay->xrl);
    length = prelay->length;
    spin_unlock_bh(&prelay->xrl);

    if( length <= 0 )
    { 
        set_bit(XMIT_BUSY, &prelay->xmit_flags);
        /* no data in buffer */
        sent = modem_relay_write2tty( prelay->tty, pdata, len );
        clear_bit(XMIT_BUSY, &prelay->xmit_flags);   

        if( sent >= len )
        {
            /* all data has been write out */
            return 0;
        }
        else if( sent > 0 )
        {
            pdata += sent;
            len -= sent;
        }
    }

    /* 
     * need buffer data, either because there is data 
     * in the buffer, all because we did not write out all data 
     */
    if( length + len > BUFSIZE )
    {
        /* we can not hold so many data, discard... */
        len = BUFSIZE - length;
    }

    tail = prelay->buf + BUFSIZE - prelay->iptr;
    if( tail >= len )
    {
        memcpy(prelay->iptr, pdata, len);
        prelay->iptr += len;
    }
    else
    {
        memcpy(prelay->iptr, pdata, tail);
        memcpy(prelay->buf, pdata+tail, len - tail);
        prelay->iptr = prelay->buf + len - tail;
    }

    /* reset the length value */
    spin_lock_bh(&prelay->xrl);
    prelay->length = prelay->length + len;
    if( prelay->length >= THRESHOLD )
    {
        /* begin to flow control peer */
        modem_bridge_stop_peer(prelay->peer_index);
    }
    spin_unlock_bh(&prelay->xrl);

    modem_relay_xmit_process(prelay);
    return 0;
}

/*
 * peer use this function to turn on our flow control
 */
static int 
modem_relay_stop_recv(void * priv)
{
    struct modem_relay_struct * prelay = (struct modem_relay_struct *)priv;
    struct tty_struct * tty;

    if( !prelay ) return -1;
    
    prelay->bFlowControl = 1;
    tty = prelay->tty;

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
modem_relay_wakeup_recv(void * priv)
{
    struct modem_relay_struct * prelay = (struct modem_relay_struct *)priv;
    struct tty_struct * tty;

    if( !prelay ) return -1;

    prelay->bFlowControl = 0;
    tty = prelay->tty;

    if (test_and_clear_bit(TTY_THROTTLED, &tty->flags)
        && tty->driver.unthrottle)
    {
        tty->driver.unthrottle(tty);
    }

    return 0;
}

static struct modem_bridge_ops modem_relay_bridge_ops = {
    modem_relay_start_xmit,
    modem_relay_stop_recv,
    modem_relay_wakeup_recv
};

/*
 * TTY line discpline: RELAY line discpline
 */

/*
 * Routines implementing the RELAY line discipline.
 */

/*
 * Called when a tty is put into RELAY line discipline.
 */
static int
modem_relay_ldisc_open(struct tty_struct *tty)
{
    struct modem_relay_struct * prelay;
    int err;

    MOD_INC_USE_COUNT;

    err = -ENOMEM;
    prelay = kmalloc(sizeof(struct modem_relay_struct), GFP_KERNEL);
    if(prelay == 0)
    {
        printk(KERN_ERR "Modem relay: can not get memory for modem relay\n");
        goto errout;
    }
    memset(prelay, 0, sizeof(struct modem_relay_struct));

    tty->disc_data = prelay;

    prelay->tty = tty;

    prelay->bridge_index = -1;
    prelay->peer_index = -1;

    prelay->optr = prelay->iptr = prelay->buf;
    spin_lock_init(prelay->xrl);

    /*
     * we first link Modem relay and TTY here, we will create 
     * bridge interface when ioctl() is called. 
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
modem_relay_ldisc_close(struct tty_struct *tty)
{
    struct modem_relay_struct * prelay;

    MOD_DEC_USE_COUNT;

    prelay = (struct modem_relay_struct *)tty->disc_data ;
    tty->disc_data = 0;

    if (prelay) 
    {
        if( prelay->bridge_index != -1 )
        {
            if( prelay->peer_index != -1 )
            {
                modem_bridge_unbind_device(prelay->bridge_index, prelay->peer_index);
                prelay->peer_index = -1;
            }
            modem_bridge_unregister_device( prelay->bridge_index );
            prelay->bridge_index = -1;
        }
        kfree(prelay);
    }
}

/*
 * Read does nothing - no data is ever available this way(tty file).
 * RELAY data only via bridge.
 */
static ssize_t
modem_relay_ldisc_read(struct tty_struct *tty, struct file *file,
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
 * RELAY data only via bridge.
 */
static ssize_t
modem_relay_ldisc_write(struct tty_struct *tty, struct file *file,
           const unsigned char *buf, size_t count)
{
    /*
     * to remove kloc warning
     */ 
    (void)tty; (void)file; (void)buf; (void)count;

    return -EAGAIN;
}

static int
modem_relay_ldisc_ioctl(struct tty_struct *tty, struct file *file,
           unsigned int cmd, unsigned long arg)
{
    struct modem_relay_struct *prelay = tty->disc_data;

    int unit = 0, err = -EFAULT;

    /*
     * to remove kloc warning
     */
    (void)file;

    switch(cmd)
    {
    case RELAYVIOCNEWBRDEVICE:
        if( prelay->bridge_index != -1 )
        {
            /* already create network interface or bridge device for this port */
            err = -EEXIST;
            break;
        }

        prelay->bridge_index = modem_bridge_register_device(prelay, &modem_relay_bridge_ops);
        if( prelay->bridge_index >= 0 )
        {
            err = -EFAULT;
            if ( put_user(prelay->bridge_index, (int*)arg) ) break;
            err = 0;
            break;
        }

        prelay->bridge_index = -1;
        err = -ENOMEM;
        break;
        
    case RELAYVIOCBINDPEER:
        /* 
         * should first create bridge device, 
         * then to bind to peer 
         */
        if( prelay->bridge_index == -1 ) break;
        
        if( get_user(unit, (int*)arg) ) break;

        /*
         * we only check if unit >= 0, more check will 
         * be made in bridge module
         */
        if( unit < 0 ) break;

        err = -ENOENT;
        if( modem_bridge_bind_device(prelay->bridge_index, unit) ) break;

        prelay->peer_index = unit;
        err = 0;
        break;

    /* for modem +++ escape */
    case MODEMESCAPESTATUS:
		if (put_user(prelay->got_esc, (int *) arg))
			break;
		err = 0;
        break;

    /* for modem +++ escape */
    case MODEMESCAPEMONITOR:
		prelay->monitor_esc = 1;
		err = 0;
        break;
                
    default:
        err = -ENOIOCTLCMD;
    }

    return err;
}

/* No kernel lock - fine */
static unsigned int
modem_relay_ldisc_poll(struct tty_struct *tty, struct file *file, poll_table *wait)
{
    /*
     * to remove kloc warning
     */ 
    (void)tty; (void)file; (void)wait;

    return 0;
}

static int
modem_relay_ldisc_room(struct tty_struct *tty)
{
    struct modem_relay_struct *prelay = tty->disc_data;

    if(prelay == 0)
    {
        return 0;
    }   

    /* if flow controled, don't receive from lower layer */
    if (prelay->bFlowControl)
    {
        return 0;
    }

    return (BUFSIZE - THRESHOLD);
}

/*
 * Return number of characters buffered to be delivered to user
 */
static ssize_t 
modem_relay_ldisc_chars_in_buffer(struct tty_struct *tty)
{
    struct modem_relay_struct *prelay = tty->disc_data;

    if(prelay == 0)
    {
        return 65535;
    }   

    /* if flow controled, don't receive from lower layer */
    if (prelay->bFlowControl)
    {
        return 65535;
    }

    return 0;
}

static void
modem_relay_ldisc_receive(struct tty_struct *tty, const unsigned char *buf,
          char *flags, int count)
{
    struct modem_relay_struct * prelay = (struct modem_relay_struct *)tty->disc_data;
    int f, i;

    /* for modem +++ escape */
	int esc_char_recv = 0;
	int k;

    if(prelay == 0) return;
    
    if( prelay->bridge_index == -1 || prelay->peer_index == -1 )
    {
        /* not bind yet, discard the data */
        return;
    }

    if (count <= 0) {
        return;
    }

    /* for modem +++ escape */
    if ( !(prelay->monitor_esc) ) {
        goto no_esc;
    }
    if ( count <= (MAX_ESC_CHAR_COUNT - prelay->esc_char_count) ) {
        if ( (prelay->esc_char_count > 0) 
            || ( (prelay->esc_char_count == 0) && ((jiffies - prelay->last_recv_for_esc) >= 10) ) ) {

            for (k = 0; k < count; k++) {
                if ( (flags != 0) && (flags[k] != 0) )
                    break;
                if (buf[k] != '+')
                    break;
                prelay->esc_char_count++;
            }
            if (k == count) {
                esc_char_recv = 1;
                if (prelay->esc_char_count == MAX_ESC_CHAR_COUNT) {
                    /* OK, we got escape +++ */
                    prelay->got_esc = 1;
                    printk("\nModem relay: got escape +++!\n");
                    if( kill_sl(tty->session, SIGHUP, 1) != 0 ) {
                        printk("\nModem relay: fail to send SIGHUP to modem engine!\n");
                    }
                    esc_char_recv = 0; /* prepare for next escape +++ */
                }
            }
        }
    }
    if (!esc_char_recv) {
        prelay->esc_char_count = 0;
        prelay->last_recv_for_esc = jiffies;
    }

    /* Work-around for BP can not handler escape +++ correctly */
    if(prelay->got_esc) 
    {
        printk("RELAY work-around for BP escape bug\n"); 
        return;
    }
    /*~END of work-around*/

no_esc:

    f = 0;
    if (flags != 0) 
    {
        /* check the flags to see if any char had an error */
        for (i = 0; i < count; ++i)
        {
            if ((f = flags[i]) != 0) break;
        }
    }
    if (f != 0)  return;

    modem_bridge_send2peer( prelay->peer_index, (void*)buf, count );
}

static void
modem_relay_ldisc_wakeup(struct tty_struct *tty)
{
    struct modem_relay_struct * prelay = (struct modem_relay_struct *)tty->disc_data;

    clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);

    if (prelay == 0) return;

    modem_relay_xmit_process(prelay);
}

static struct tty_ldisc modem_relay_ldisc = {
    magic:  TTY_LDISC_MAGIC,
    name:   "modem relay",
    /*routines called from above tty file interfaces*/
    open:   modem_relay_ldisc_open,
    close:  modem_relay_ldisc_close,
	chars_in_buffer: modem_relay_ldisc_chars_in_buffer,
    read:   modem_relay_ldisc_read,
    write:  modem_relay_ldisc_write,
    ioctl:  modem_relay_ldisc_ioctl,
    poll:   modem_relay_ldisc_poll,
    /*routines called by below driver(mux)*/
    receive_room: modem_relay_ldisc_room,
    receive_buf: modem_relay_ldisc_receive,
    write_wakeup: modem_relay_ldisc_wakeup,
};

#ifndef N_MODEM_RELAY
#define N_MODEM_RELAY 18
#endif

/* Module Functions*/

/*
 * Module init function
 */
static int __init modem_relay_init(void)
{
    int err;

    err = tty_register_ldisc(N_MODEM_RELAY, &modem_relay_ldisc);
    if (err != 0)
        printk(KERN_ERR "Modem relay: error %d registering line disc.\n",err);
    return err;
}

/*
 * Module clean up function
*/
static void __exit modem_relay_cleanup(void)
{
    if (tty_register_ldisc(N_MODEM_RELAY, NULL) != 0)
        printk(KERN_ERR "Modem relay:failed to unregister modem relay line discipline\n");
}

module_init(modem_relay_init);
module_exit(modem_relay_cleanup);

MODULE_LICENSE("GPL");
