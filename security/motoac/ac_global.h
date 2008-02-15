#ifndef AC_GLOBAL_H
#define AC_GLOBAL_H
/*================================================================================
                                                                               
                      Header Name: ac_global.h

General Description: define the common type, common functions
 
==================================================================================
 
   Copyright (C) 2005 - Motorola
 
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 2 as
   published by the Free Software Foundation.
 
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
 
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
  
Revision History:
                            Modification     Tracking
Author(core ID)                 Date          Number      Description of Changes
------------------------    ------------    ----------   -----------------------------------
Zhu zhifu (w20598)           01/20/2005     LIBff54376   Init version   

==================================================================================*/
/*===============================================================================
                                 INCLUDE FILES
================================================================================*/

/*================================================================================
                          STRUCTURES AND OTHER TYPEDEFS
================================================================================*/
/* define the basic type */
#if 0
typedef _u32 ac_process_id
typedef _u32 ac_domain_id
typedef _u32 ac_object_id
typedef _u32 ac_object_class
typedef _u32 ac_privilege_bits
#endif

#include "linux/ac_class.h"

/* defines safe operations for kernel */ 
#ifdef __KERNEL__
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <asm/semaphore.h>

typedef unsigned long security_class_t;
typedef unsigned long security_id_t;
typedef unsigned long access_vector_t;

/*
 * Use this for memory allocation flags (e.g. for kmalloc()) when the 
 * interrupt state is unknown.
 */
#define SAFE_ALLOC (in_interrupt () ? GFP_ATOMIC : GFP_KERNEL)

/*
 * Use these for semaphores when the interrupt state is unknown.
 */

static inline int safe_down(struct semaphore *sem)
{
    if (in_interrupt()) {
        if (down_trylock(sem))
	    return -EAGAIN;
    } else {
	down(sem);
    }
    return 0;
}

static inline void safe_up(struct semaphore *sem)
{
    up(sem);
}
#endif /* __KERNEL__ */


#ifndef __KERNEL__

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>
#include <errno.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <byteswap.h>
#include <endian.h>

#define NIPQUAD(addr) \
	((unsigned char *)&addr)[0], \
	((unsigned char *)&addr)[1], \
	((unsigned char *)&addr)[2], \
	((unsigned char *)&addr)[3]

#if __BYTE_ORDER == __LITTLE_ENDIAN
#define cpu_to_le32(x) (x)
#define le32_to_cpu(x) (x)
#define cpu_to_le64(x) (x)
#define le64_to_cpu(x) (x)
#else
#define cpu_to_le32(x) bswap_32(x)
#define le32_to_cpu(x) bswap_32(x)
#define cpu_to_le64(x) bswap_64(x)
#define le64_to_cpu(x) bswap_64(x)
#endif

#define wmb() 

#define malloc_sleep(size) malloc(size)

#else

#include <linux/kernel.h>	/* printk */
#include <linux/slab.h>		/* kmalloc, kfree */
#include <linux/vmalloc.h>
#include <linux/fs.h>		/* read, write, open */
#include <linux/file.h>
#include <linux/string.h>	/* strcpy, strncpy, strcmp */
#include <linux/in.h>		/* IPPROTO_* */
#include <linux/ctype.h>
#include <asm/system.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#include "ac_plug.h"

#define malloc(size) kmalloc(size, GFP_ATOMIC)
#define malloc_sleep(size) kmalloc(size, GFP_KERNEL)
#define free(ptr) kfree(ptr)

typedef struct file FILE;

static inline FILE *fopen(char *path, char *type)
{
    struct nameidata nd;
    struct file *file;
    int err = 0;

    if (strcmp(type, "r"))
        panic("MOTO AC:fopen");

    if (path_init(path, LOOKUP_FOLLOW | LOOKUP_POSITIVE, &nd)) 
        err = path_walk(path, &nd);
    if (err)
        return NULL;

    if (!ac_policy_init)
	inode_security_set_sid(nd.dentry->d_inode,AC_DEFAULT_POLICY);
			
    if (!S_ISREG(nd.dentry->d_inode->i_mode)) 
        goto bad;

    file = dentry_open(nd.dentry, nd.mnt, O_RDONLY);
    if (IS_ERR(file))
        return NULL;
    else 
        return file;

bad:
    path_release(&nd);
    return NULL;
}


static inline int fclose(FILE * stream)
{
	fput(stream);
	return 0;
}

static inline ssize_t fread(void *buf, size_t size, size_t nitems, FILE * fp)
{
    unsigned long bytes = nitems * size;
    mm_segment_t old_fs;
    ssize_t rc;

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    rc = (fp)->f_op->read((fp), (buf), bytes, &(fp)->f_pos);
    set_fs(old_fs);
    if (rc > 0)
        return (rc / size);
    
    return 0;
}

#if CONFIG_SECURITY_AC_DEVELOP
#define printf printk
#else
#define printf(...)
#endif

#define exit(error_code) panic("Moto AC: exiting (%d)",error_code)

#endif

#endif  /*AC_TYPES_H*/

