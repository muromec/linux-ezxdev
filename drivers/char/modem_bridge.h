/*   
 * modem_bridge.h 
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

#ifndef __MODEM_BRIDGE__
#define __MODEM_BRIDGE__

#include <linux/skbuff.h>

#define MAX_MODEM_BRIDGE_NUMBER 16

struct modem_bridge_ops {
    int (*send)(void* pdata, int len, void *priv);
    int (*stop)(void* priv);
    int (*wakeup)(void* priv);
};

int modem_bridge_wakeup_peer(int);
int modem_bridge_stop_peer(int);
int modem_bridge_send2peer(int, void*, int);
int modem_bridge_bind_device(int, int);
int modem_bridge_unbind_device(int, int);
int modem_bridge_register_device(void*, struct modem_bridge_ops*);
int modem_bridge_unregister_device(int);

#endif
