#ifndef __ARM_MMU_H
#define __ARM_MMU_H

#include <asm-generic/mmu.h>
/*
 * The ARM doesn't have a mmu context
 */
typedef struct { } mm_context_t;

#endif
