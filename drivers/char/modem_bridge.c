/*   
 * modem_bridge.c 
 *
 * Copyright (c) 2005 Motorola  
 *      
 * Modem bridge works as a link layer relay system. With the help 
 * of this module, two link layer devices could work together to send and receive data 
 * between each other. 
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
#include <linux/kmod.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include "modem_bridge.h"

struct modem_bridge{
    int avail;         /* none zero if slot not occupied */
    void * priv;       /* private pointer only has meaning to registered device */
    int peer_index;   /* to whom I am working with */
    struct modem_bridge_ops * ops; /* my specific functions */
} modem_bridge_array[MAX_MODEM_BRIDGE_NUMBER];

static DECLARE_MUTEX(modem_bridge_sem);

/*
 * Turn flow control off, so that peer can send us more data
 */
int modem_bridge_wakeup_peer(int peer_index)
{
    /* first make sure index in range */
    if( 0 > peer_index || peer_index >= MAX_MODEM_BRIDGE_NUMBER )
    {
        return -1;
    }

    if( modem_bridge_array[peer_index].avail ) 
    {
        return -1;
    }

    if( modem_bridge_array[peer_index].ops->wakeup )
    {
        return modem_bridge_array[peer_index].ops->wakeup(modem_bridge_array[peer_index].priv);
    }
    else
    {
        return -1;
    }
}

/*
 * Turn flow control on, peer should not send us data 
 * until flow control is turned off again
 */
int modem_bridge_stop_peer(int peer_index)
{
    /* first make sure index in range */
    if( 0 > peer_index || peer_index >= MAX_MODEM_BRIDGE_NUMBER )
    {
        return -1;
    }

    /*
     * then make sure peer actually existed
     */
    if( modem_bridge_array[peer_index].avail )
    {
        return -1;
    }

    if( modem_bridge_array[peer_index].ops->stop )
    {
        return modem_bridge_array[peer_index].ops->stop(modem_bridge_array[peer_index].priv);
    }
    else
    {
        return -1;
    }
}

/*
 * call this function to send skb to peer, 
 * pdata could be a skb buffer(in this case, len is zero),
 * or a pointer to start of data, with len defines the length
 */
int modem_bridge_send2peer( int peer_index, void* pdata, int len )
{
    /* first make sure index in range */
    if( 0 > peer_index || peer_index >= MAX_MODEM_BRIDGE_NUMBER )
    {
        goto __errout;
    }

    /*
     * make sure peer actually existed
     */
    if( modem_bridge_array[peer_index].avail ) 
    {
        goto __errout;
    }

    if( modem_bridge_array[peer_index].ops->send )
    {
        return modem_bridge_array[peer_index].ops->send(pdata, len, modem_bridge_array[peer_index].priv);
    }
    /* else fall through */

__errout:
    if ( !len )
        /* FIXME: pdata maybe not skb */
        kfree_skb( (struct sk_buff*)pdata );
    return -1;
}

/*
 * bind two devices together to make a bridge,
 * call this function from any of the two devices
 */
int modem_bridge_bind_device( int index1, int index2 )
{
    int ret = -1;

    /* first make sure index in range */
    if( 0 > index1 || index1 >= MAX_MODEM_BRIDGE_NUMBER ||
        0 > index2 || index2 >= MAX_MODEM_BRIDGE_NUMBER )
    {
        return ret;
    }

    /* do not bind to self */
    if( index1 == index2 )
    {
        return ret;
    }

    down(&modem_bridge_sem);

    if( !modem_bridge_array[index1].avail && !modem_bridge_array[index2].avail )
    {
        if( modem_bridge_array[index1].peer_index == index2 
            && modem_bridge_array[index2].peer_index == index1 )
        {
            /* we are already bind together */
            ret = 0;
        }
        else if( modem_bridge_array[index1].peer_index == -1 
            && modem_bridge_array[index2].peer_index == -1 )
        {
            modem_bridge_array[index1].peer_index = index2;
            modem_bridge_array[index2].peer_index = index1;
            ret = 0;
        }
    }

    up(&modem_bridge_sem);

    return ret;
}

/*
 * unbind the two devices
 */
int modem_bridge_unbind_device(int index1, int index2)
{
    int ret = -1;

    /* first make sure index in range */
    if( 0 > index1 || index1 >= MAX_MODEM_BRIDGE_NUMBER ||
        0 > index2 || index2 >= MAX_MODEM_BRIDGE_NUMBER || index1 == index2 )
    {
        return ret;
    }
    if( modem_bridge_array[index1].avail && modem_bridge_array[index2].avail )
    {
        return ret;
    }

    down(&modem_bridge_sem);

    if( !modem_bridge_array[index1].avail && !modem_bridge_array[index2].avail )
    {
        if( modem_bridge_array[index1].peer_index == index2 &&
            modem_bridge_array[index2].peer_index == index1 )
        {
            /*
             * we are bind at some early time
             */
            modem_bridge_array[index1].peer_index = -1;
            modem_bridge_array[index2].peer_index = -1;
            ret = 0;
        }
        else if( modem_bridge_array[index1].peer_index == -1 &&
                 modem_bridge_array[index2].peer_index == -1 )
        {
            /* We are already not bound */
            ret = 0;
        }
    }
    else if( (modem_bridge_array[index1].peer_index == -1 && modem_bridge_array[index2].avail) 
            || (modem_bridge_array[index1].avail && modem_bridge_array[index2].peer_index == -1) )
    {
        /* We are already not bound and peer unregistered */
        ret = 0;
    }
    
    up(&modem_bridge_sem);
    return ret;
}

/*
 * register a device, create a unique index for it
 */
int modem_bridge_register_device(void * priv, struct modem_bridge_ops * ops)
{
    int i;

    /*
     * both priv & ops should contain meaningful data
     */
    if( !priv ) return -1;
    if( !ops ) return -1;
    
    down(&modem_bridge_sem);
    for (i = 0; i < MAX_MODEM_BRIDGE_NUMBER; i++)
    {
        if( modem_bridge_array[i].avail )
        {
            modem_bridge_array[i].avail = 0;
            modem_bridge_array[i].peer_index = -1;
            modem_bridge_array[i].priv = priv;
            modem_bridge_array[i].ops = ops;

            MOD_INC_USE_COUNT;
            goto __out;
        }
    }
    i = -1;
    
__out:
    up(&modem_bridge_sem);
    return i;
}

/*
 * unregister me from modem bridge
 */
int modem_bridge_unregister_device( int index )
{
    int ret = -1;

    if( 0 > index || index >= MAX_MODEM_BRIDGE_NUMBER )
    {
        return ret;
    }

    down(&modem_bridge_sem);
    
    if( !modem_bridge_array[index].avail )
    {
        if( modem_bridge_array[index].peer_index != -1 )
        {
            goto __out;
        }
        MOD_DEC_USE_COUNT;
        modem_bridge_array[index].avail = 1;

        ret = 0;
    }

__out:
    up(&modem_bridge_sem);
    return ret;
}

/*
 * initialize modem bridge
 */
static int __init modem_bridge_init(void)
{
    int i;
    memset(modem_bridge_array, 0, sizeof(struct modem_bridge) * MAX_MODEM_BRIDGE_NUMBER);
    for ( i = 0; i < MAX_MODEM_BRIDGE_NUMBER; i++ )
    {
        modem_bridge_array[i].avail = 1;
    }

    return 0;
}

static void __exit modem_bridge_cleanup(void)
{
}

EXPORT_SYMBOL(modem_bridge_wakeup_peer);
EXPORT_SYMBOL(modem_bridge_stop_peer);
EXPORT_SYMBOL(modem_bridge_send2peer);
EXPORT_SYMBOL(modem_bridge_bind_device);
EXPORT_SYMBOL(modem_bridge_unbind_device);
EXPORT_SYMBOL(modem_bridge_register_device);
EXPORT_SYMBOL(modem_bridge_unregister_device);

module_init(modem_bridge_init);
module_exit(modem_bridge_cleanup);

MODULE_LICENSE("GPL");
