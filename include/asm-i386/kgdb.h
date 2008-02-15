#ifndef __KGDB
#define __KGDB

/*
 * This file should not include ANY others.  This makes it usable
 * most anywhere without the fear of include order or inclusion.
 * Make it so!
 */

extern void breakpoint(void);
#define INIT_KGDB_INTS kgdb_enable_ints()

#ifndef BREAKPOINT   
#define BREAKPOINT   asm("   int $3");
#endif
/*
 * GDB debug stub (or any debug stub) can point the 'linux_debug_hook'
 * pointer to its routine and it will be entered as the first thing
 * when a trap occurs.
 *
 * Return values are, at present, undefined.
 *
 * The debug hook routine does not necessarily return to its caller.
 * It has the register image and thus may choose to resume execution
 * anywhere it pleases. 
 */
struct pt_regs;

typedef int	gdb_debug_hook(int trapno,
			       int signo,
			       int err_code,
			       struct pt_regs *regs) ;
extern gdb_debug_hook	*linux_debug_hook ;
extern int in_kgdb(struct pt_regs *regs);
#ifdef CONFIG_KGDB_TS
void kgdb_tstamp(int line, char * source, int data0, int data1);
/*
 * This is the time stamp function.  The macro adds the source info and
 * does a cast on the data to allow most any 32-bit value.
 */

#define kgdb_ts(data0,data1) kgdb_tstamp(__LINE__,__FILE__,(int)data0,(int)data1)
#else
#define kgdb_ts(data0,data1)
#endif
#endif /* __KGDB */

