/* $Id: cache.h,v 1.3 2000/07/04 06:24:49 gniibe Exp $
 *
 * include/asm-sh/cache.h
 *
 * Copyright 1999 (C) Niibe Yutaka
 */
#ifndef __ASM_SH_CACHE_H
#define __ASM_SH_CACHE_H

/* bytes per L1 cache line */
#if defined(__sh3__)
#define        L1_CACHE_BYTES  16
#elif defined(__SH4__)
#define        L1_CACHE_BYTES  32
#endif

struct cache_info {
        unsigned int ways;
        unsigned int sets;
        unsigned int linesz;

        unsigned int way_shift;
        unsigned int entry_shift;
        unsigned int entry_mask;

        unsigned long flags;
};



#endif /* __ASM_SH_CACHE_H */
