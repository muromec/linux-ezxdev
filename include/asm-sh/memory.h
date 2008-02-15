#ifndef __ASM_SH_MEMORY_H
#define __ASM_SH_MEMORY_H

#include <linux/config.h>


#ifdef  CONFIG_DISCONTIGMEM 
#error DISCONTIGMEM is not supported yet
#else
#define page_to_nid(x) (0)
#endif

#endif
