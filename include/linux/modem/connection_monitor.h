/*
 * include/linux/modem/connection_monitor.h
 *
 * Copyright (C) 2005 Motorola
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 */

/*
 * Originally written by Liu, Changhui  2005-01-21 (w20041@motorola.com)
 */
#ifndef _CONNECTION_MONITOR_H
#define _CONNECTION_MONITOR_H


typedef enum {
    CONNECTION_USB = 0,
    CONNECTION_BT,
    CONNECTION_UART,
    CONNECTION_IRDA
} CONNECTION_TYPE;

typedef enum {
    CONNECTION_BROKEN = 0,
    CONNECTION_OK
} CONNECTION_STATUS;

typedef struct {
    CONNECTION_STATUS usb;
    CONNECTION_STATUS bt;
    CONNECTION_STATUS uart;
    CONNECTION_STATUS irda;
} connection_status;

/* Called in Kernel space */
extern int modem_connection_status_inform_kernel(CONNECTION_TYPE connection, CONNECTION_STATUS status); 

/* Called in User space */
extern int modem_connection_status_inform_userspace(CONNECTION_TYPE connection, CONNECTION_STATUS status); 


#define MDMCONNMONITORSETOWNER    _IO('m', 163)   
#define MDMCONNMONITORCONNSTATUS    _IOW('m', 164, int)  
#define MDMCONNMONITORBTFILENAME    _IOR('m', 165, int)  
#endif /* _CONNECTION_MONITOR_H */

