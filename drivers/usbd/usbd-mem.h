/*
 * usbd/usbd-mem.h 
 *
 *      Copyright (c) 2004 Belcarra
 *
 * Adapted from earlier work:
 *      Copyright (c) 2002, 2003 Belcarra
 *      Copyright (c) 2000, 2001, 2002 Lineo
 *      Copyright (c) 2001 Hewlett Packard
 *
 * By: 
 *      Stuart Lynne <sl@belcarra.com>, 
 *      Tom Rushworth <tbr@belcarra.com>, 
 *      Bruce Balden <balden@belcarra.com>
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
 */


#ifndef MODULE
#undef __init
#define __init
#undef __exit
#define __exit
#undef THIS_MODULE
#define THIS_MODULE 0
#endif

#define USBD_MODULE_INFO(info) static const char __usbd_module_info[] = info " " USBD_BUILD " %D% %@%";


#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif
#define CATCH(x) while(0) x:
#define THROW(x) goto x
#define THROW_IF(e, x) if (e)  goto x; 
#define BREAK_IF(x) if (x)  break; 
#define CONTINUE_IF(x) if (x)  continue; 
#define RETURN_IF(x) if (x)  return; 
#define RETURN_ZERO_IF(x) if (x)  return 0; 
#define RETURN_EINVAL_IF(x) if (x)  return -EINVAL; 
#define RETURN_EFAULT_IF(x) if (x)  return -EFAULT; 
#define RETURN_ENOMEM_IF(x) if (x)  return -ENOMEM; 
#define RETURN_EBUSY_IF(x) if (x)  return -EBUSY; 
#define RETURN_NULL_IF(x) if (x)  return NULL; 
#define unless(x) if(!(x))
#define UNLESS(x) if(!(x))
#define RETURN_UNLESS(x) UNLESS (x)  return; 
#define CONTINUE_UNLESS(x) UNLESS (x)  continue; 


#ifndef likely
#define likely(x) x
#endif

#ifndef unlikely 
#define unlikely(x) x
#endif

#define ckmalloc(n,f) _ckmalloc(__FUNCTION__, __LINE__, n, f)
#define lstrdup(str) _lstrdup(__FUNCTION__, __LINE__, str)
#define lkfree(p) _lkfree(__FUNCTION__, __LINE__, p)

#define MALLOC_TEST
#undef MALLOC_DEBUG

#ifdef MALLOC_TEST
extern int usbd_mallocs;
#endif


static __inline__ void *_ckmalloc (char *func, int line, int n, int f)
{
	void *p;
	if ((p = kmalloc (n, f)) == NULL) {
		return NULL;
	}
	memset (p, 0, n);
#ifdef MALLOC_TEST
        ++usbd_mallocs;
#endif
#ifdef MALLOC_DEBUG
        printk(KERN_INFO"%s: %p %s %d %d\n", __FUNCTION__, p, func, line, usbd_mallocs);
#endif
	return p;
}

static __inline__ char *_lstrdup (char *func, int line, char *str)
{
	int n;
	char *s;
	if (str && (n = strlen (str) + 1) && (s = kmalloc (n, GFP_ATOMIC))) {
#ifdef MALLOC_TEST
                ++usbd_mallocs;
#endif
#ifdef MALLOC_DEBUG
                printk(KERN_INFO"%s: %p %s %d %d\n", __FUNCTION__, s, func, line, usbd_mallocs);
#endif
		return strcpy (s, str);
	}
	return NULL;
}

static __inline__ void _lkfree (char *func, int line, void *p)
{
	if (p) {
#ifdef MALLOC_TEST
                --usbd_mallocs;
#endif
#ifdef MALLOC_DEBUG
                printk(KERN_INFO"%s: %p %s %d %d\n", __FUNCTION__, p, func, line, usbd_mallocs);
#endif
		kfree (p);
#ifdef MALLOC_TEST
                if (usbd_mallocs < 0) {
                        printk(KERN_INFO"%s: %p %s %d %d usbd_mallocs less zero!\n", __FUNCTION__, p, func, line, usbd_mallocs);
                }
#endif
	}
#ifdef MALLOC_DEBUG
        else {
                printk(KERN_INFO"%s: %s %d NULL\n", __FUNCTION__, func, line);
        }
#endif
}



