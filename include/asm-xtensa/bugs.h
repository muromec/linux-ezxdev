#ifndef __ASM_XTENSA_BUGS_H
#define __ASM_XTENSA_BUGS_H

/*
 * This is included by init/main.c to check for architecture-dependent bugs.
 *
 * Needs:
 *	void check_bugs(void);
 */

/*
 * I don't know of any Xtensa Processor bugs yet.
 */

#include <asm/processor.h>

static void __init check_bugs(void)
{
}
#endif /* __ASM_XTENSA_BUGS_H */
