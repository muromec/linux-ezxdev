/*
 * npnet.h -- definitions for the network module
 *
 *********/


/*
 * Macros to help debugging
 */

#undef PDEBUG             /* undef it, just in case */
#ifdef NPNET_DEBUG
#  ifdef __KERNEL__
     /* This one if debugging is on, and kernel space */
#    define PDEBUG(fmt, args...) printk( KERN_DEBUG "npnet: " fmt, ## args)
#  else
     /* This one for user space */
#    define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif

#undef PDEBUGG
#define PDEBUGG(fmt, args...) /* nothing: it's a placeholder */


/* These are the flags in the statusword */
#define NPNET_RX_INTR 0x0001
#define NPNET_TX_INTR 0x0002

/* Default timeout period */
#define NPNET_TIMEOUT 5   /* In jiffies */

extern struct net_device npnet_devs;




