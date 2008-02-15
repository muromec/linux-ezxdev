/*
 * drivers/modem/connection_monitor.c
 *
 * Description of the file  
 *
 * Copyright (C) 2005 Motorola Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 * Revision History:
 *                   Modification    
 Changed by            Date             Description of Changes
----------------   ------------       -------------------------
Liu Chang Hui       2005/02/24         add for ezx
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <linux/modem/connection_monitor.h>

static struct task_struct *connection_monitor_owner = NULL;

DECLARE_WAIT_QUEUE_HEAD(connection_monitor_wait);

static connection_status modem_connection_status;
static char bt_filename[100];
static spinlock_t access_lock;
static int new_status = 0;

/* Called in Kernel space */
int modem_connection_status_inform_kernel(CONNECTION_TYPE connection, CONNECTION_STATUS status)
{
    unsigned long flags;
    int ret = 0;

    spin_lock_irqsave(&access_lock, flags);

    switch( connection )
    {
        case CONNECTION_USB:
            if( modem_connection_status.usb != status )
            {
                modem_connection_status.usb = status;
                new_status = 1;
                printk("\nModem Connection Monitor: USB connection change to %s\n", 
                    ( status ? "OK" : "Broken"));
            }
            break;

        case CONNECTION_BT:
            if( modem_connection_status.bt != status )
            {
                modem_connection_status.bt = status;
                new_status = 1;
                printk("\nModem Connection Monitor: BT connection change to %s\n", 
                    ( status ? "OK" : "Broken"));
            }
            break;

        case CONNECTION_UART:
            if( modem_connection_status.uart != status )
            {
                modem_connection_status.uart = status;
                new_status = 1;
                printk("\nModem Connection Monitor: UART connection change to %s\n", 
                    ( status ? "OK" : "Broken"));
            }
            break;

        case CONNECTION_IRDA:
            if( modem_connection_status.irda != status )
            {
                modem_connection_status.irda = status;
                new_status = 1;
                printk("\nModem Connection Monitor: IRDA connection change to %s\n", 
                    ( status ? "OK" : "Broken"));
            }
            break;

        default:
            ret = -1;
            break;
    }

    if( new_status )
        wake_up_interruptible(&connection_monitor_wait);
    
    spin_unlock_irqrestore(&access_lock, flags);

    return ret;
}

static int connection_monitor_open(struct inode * inode, struct file * file)
{
    (void)inode;(void)file;/* For Kloc warning */
    
    MOD_INC_USE_COUNT;
    return 0;
}

static int connection_monitor_release(struct inode * inode, struct file * file)
{
    (void)inode;(void)file;/* For Kloc warning */

    if( (connection_monitor_owner != NULL) && (connection_monitor_owner == current) )
    {
        connection_monitor_owner = NULL;       
    }
    MOD_DEC_USE_COUNT;
    return 0;
}

static ssize_t connection_monitor_read(struct file * file, char * buf,
             size_t count, loff_t *ppos)
{
    unsigned long flags;
    int ret;
    int newStatus;
    char bufLocal[ sizeof(modem_connection_status) ];
    
    (void)file;(void)ppos;/* For Kloc warning */

    if( (connection_monitor_owner == NULL) || (current != connection_monitor_owner) )
        return -EAGAIN;

    if( count < sizeof(modem_connection_status) )
        return -EINVAL;

    spin_lock_irqsave(&access_lock, flags);
    newStatus = new_status;
    new_status = 0;
    memcpy(bufLocal, &modem_connection_status, sizeof(modem_connection_status));
    spin_unlock_irqrestore(&access_lock, flags);

    if( newStatus )
    {
        if( copy_to_user(buf, bufLocal, sizeof(modem_connection_status)) )
        {
            spin_lock_irqsave(&access_lock, flags);
            new_status = 1;
            spin_unlock_irqrestore(&access_lock, flags);
            ret = -EFAULT;
        }
        else
        {
            ret = sizeof(modem_connection_status);
        }
    }
    else
    {
        ret = -EAGAIN;
    }

    return ret;
}

static ssize_t connection_monitor_write(struct file * file, const char * buf, 
             size_t count, loff_t *ppos)
{
    unsigned long flags;
    char cmd_buf[256];
    char *pBegin = NULL;
    char *pEnd = NULL;
    CONNECTION_STATUS btStatus;

    (void)file;(void)ppos;/* For Kloc warning */

    if( (!buf) || (count <= 0) ) {
        return -EINVAL;
    }
    if( count > (sizeof(cmd_buf) - 1) ) {
        count = (sizeof(cmd_buf) - 1);
    }
    if( copy_from_user(cmd_buf, buf, count) ) {
        return -EFAULT;
    }
    cmd_buf[count] = '\0';
    printk("\nModem Connection Monitor: input cmd string %s\n", cmd_buf);

    if( strncmp(cmd_buf, "BT_OK", 5) == 0 ) {
        if( (pBegin = strchr(cmd_buf, '/')) == NULL) {
            return -EINVAL;
        }
        if( (pEnd = strchr(pBegin, '\n')) != NULL) {
            *pEnd = '\0';
        }
        if( strlen(pBegin) > (sizeof(bt_filename) - 1) ) {
            return -EINVAL;
        }
        spin_lock_irqsave(&access_lock, flags);
        strcpy(bt_filename, pBegin);
        spin_unlock_irqrestore(&access_lock, flags);
        btStatus = CONNECTION_OK;        
    } else if( strncmp(cmd_buf, "BT_BROKEN", 9) == 0 ) {
        spin_lock_irqsave(&access_lock, flags);
        bt_filename[0] = '\0';
        spin_unlock_irqrestore(&access_lock, flags);
        btStatus = CONNECTION_BROKEN;
    } else {
        return -EINVAL;
    }

    modem_connection_status_inform_kernel(CONNECTION_BT, btStatus);    
    return count;
}

static unsigned int connection_monitor_poll(struct file *file, poll_table * wait)
{
    unsigned long flags;
    int newStatus;
    
    if( (connection_monitor_owner == NULL) || (current != connection_monitor_owner) )
        return 0;
    poll_wait(file, &connection_monitor_wait, wait);
    spin_lock_irqsave(&access_lock, flags);
    newStatus = new_status;
    spin_unlock_irqrestore(&access_lock, flags);
    if( newStatus )
        return POLLIN | POLLRDNORM;
    return 0;
}

static int connection_monitor_ioctl(struct inode * inode, struct file * file,
            unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    int connStatus;
    int connection;
    int status;
    unsigned long flags;
    char btFileName[ sizeof(bt_filename) ];
    
    (void)inode;(void)file;

    switch( cmd )
    {
        case MDMCONNMONITORSETOWNER:
            if( connection_monitor_owner != NULL )
            {
                ret = -EFAULT;
            }
            else
            {
                connection_monitor_owner = current;
            }
            break;

        case MDMCONNMONITORCONNSTATUS:
            if( get_user(connStatus, (int *)arg) )
            {
		        ret = -EFAULT;
		        break;
            }

            connection = connStatus >> 16;
            status = connStatus & 0x0000FFFF;

            if( (connection < CONNECTION_USB) || (connection > CONNECTION_IRDA) )
            {
                ret = -EINVAL;
                break;                
            }
            if( (status != CONNECTION_BROKEN) && (status != CONNECTION_OK) )
            {
                ret = -EINVAL;
                break;
            }
            
            modem_connection_status_inform_kernel((CONNECTION_TYPE)connection, (CONNECTION_STATUS)status);
            break;

        case MDMCONNMONITORBTFILENAME:
            spin_lock_irqsave(&access_lock, flags);
            strcpy(btFileName, bt_filename);
            spin_unlock_irqrestore(&access_lock, flags);
            if( copy_to_user((void *)arg, (const void *)btFileName, strlen(btFileName)+1) )
            {
                ret = -EFAULT;
            }            
            break;

        default:
            ret = -ENOIOCTLCMD;
            break;
    }

    return ret;
}

static struct proc_dir_entry *connection_monitor_proc_file = NULL; 
static struct file_operations proc_connection_monitor_operations = {
    read:       connection_monitor_read,
	write:		connection_monitor_write,
    poll:       connection_monitor_poll,
    ioctl:		connection_monitor_ioctl,
    open:       connection_monitor_open,
    release:    connection_monitor_release,
};


static int __init connection_monitor_init(void)
{
    connection_monitor_owner = NULL;
    init_waitqueue_head(&connection_monitor_wait);
    spin_lock_init(&access_lock);

    modem_connection_status.usb = CONNECTION_BROKEN;
    modem_connection_status.bt = CONNECTION_BROKEN;
    modem_connection_status.uart = CONNECTION_BROKEN;
    modem_connection_status.irda = CONNECTION_BROKEN;
    new_status = 0;
    bt_filename[0] = '\0';

    connection_monitor_proc_file = create_proc_entry("modemconnections", S_IRUGO | S_IWUGO, NULL);
    if( connection_monitor_proc_file )
    {
        connection_monitor_proc_file->proc_fops = &proc_connection_monitor_operations;
    }
    else
    {
        printk("\nModem Connection Monitor: Fail to create_proc_entry\n");
    }

    return 0;
}

static void __exit connection_monitor_exit(void)
{
    if( connection_monitor_proc_file )
    {
        remove_proc_entry("modemconnections", connection_monitor_proc_file);
    }
}


module_init(connection_monitor_init);
module_exit(connection_monitor_exit);
