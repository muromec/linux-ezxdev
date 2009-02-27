/*
 * include/asm-arm/kgdb.h
 *
 * ARM KGDB support 
 *
 * Author: Deepak Saxena <dsaxena@mvista.com>
 *
 * Copyright (C) 2002 MontaVista Software Inc.
 *
 */

#ifndef __ASM_KGDB_H__
#define __ASM_KGDB_H__

#include <linux/config.h>
//#include <linux/ptrace.h>

#ifdef CONFIG_KGDB
/*
 * GDB assumes that we're a user process being debugged, so
 * it will send us an SWI command to write into memory as the
 * debug trap. This doesn't work for kernel debugging as
 * in the kernel we're already in SVC mode, and the SWI instruction
 * would cause us to loose the kernel LR, which is a bad thing.
 * (see ARM ARM for reasoning)
 *
 * By doing this as an undefined instruction trap, we force a mode
 * switch from SVC which allows us to save full kernel state.
 * If we see GDB writing 4 bytes == GDB_BREAKINST to a memory
 * location, we just replace it with KGDB_BREAKINST.
 *
 * We also define a KGDB_COMPILED_BREAK which can be used to compile
 * in breakpoints. This is important for things like sysrq-G and for
 * the initial breakpoint from trap_init().
 *
 * Note to ARM HW designers: Add real trap support like SH && PPC to
 * make our lives much much simpler.
 */
#define GDB_BREAKINST                   0xef9f0001
#define KGDB_BREAKINST                  0xe7ffdefe
#define KGDB_COMPILED_BREAK             0xe7ffdeff


#if	!defined(__ASSEMBLY__)
/*
 * Generic kgdb entry point
 *
 * This is called by various trap handlers when all else fails
 * and a kernel excpetion has occured.
 */
void do_kgdb(struct pt_regs *, unsigned char);


/*
 * Determine if KGDB is running or not. This is needed for
 * things like the console driver which need to know whether
 * or not to stuff serial output in a GDB packet.
 */
int kgdb_active(void);

/* 
 * kgdb_handle_bus_error() is used to recover from bus errors
 * which may occur during kgdb memory read/write operations.
 * Before performing a memory read/write operation, we set
 * the kgdb_fault_expected flag, call setjmp to save the
 * current machine context, and then perform the read/write
 * operation. If an error occurs, kgdb_handle_bus_error is
 * invoked from do_page_fault and we call longjmp to restore
 * the machine state and return to kgdb at the setjmp return
 * address. Both setjmp and longjmp return a flag which is
 * tested upon return to determine if all is well. Upon the
 * initial call to setjmp, the machine context is saved and
 * a return value of zero indicates all is well. Next, we
 * perform the read/write operation. Now, if an error occurs,
 * longjmp will return to the setjmp == 0 test case. But
 * this time, non-zero status is returned. In which case,
 * we return an error status message to the gdb host. Else,
 * if the read/write operation succeeds, we return the usual
 * status message to the gdb host.
 */
extern void kgdb_handle_bus_error(void);                                                                                              
extern int kgdb_setjmp(int *machine_context);
extern int kgdb_longjmp(int *machine_context, int flag);
extern int kgdb_fault_expected;
 
extern int kgdb_enter;
extern int kgdb_ttyS;
extern int kgdb_baud;

/*
 * breakpoint() is used by the kernel at initialization to force
 * a sync with the GDB on the host side.
 */
extern void breakpoint(void);

/* It would be nice if the following used the KGDB_COMPILED_BREAK
 * define from above. But for now, it's hardwired...
 */
#define BREAKPOINT()	asm (".word	0xe7ffdeff")

extern void kgdb_get_packet(unsigned char *, int);
extern void kgdb_put_packet(const unsigned char *);

extern int kgdb_io_init(void);

#ifdef CONFIG_KGDB_SERIAL

extern unsigned char kgdb_serial_getchar(void);
extern void kgdb_serial_putchar(unsigned char);

extern void kgdb_serial_init(void);

#endif

#ifdef CONFIG_KGDB_UDP

extern unsigned kgdb_net_rx(unsigned char *);
extern unsigned kgdb_net_tx(unsigned char *);

#endif

#ifdef CONFIG_KGDB_CONSOLE
/* Console */
void kgdb_console_init(void);
#endif

#else // NO KGDB

#define	kgdb_active()	0

#define	breakpoint()

#endif 

#endif // !__ASSEMBLY__
#endif // __ASM_KGDB_H__
