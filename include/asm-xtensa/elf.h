#ifndef __ASM_XTENSA_ELF_H
#define __ASM_XTENSA_ELF_H

/*
 * include/asm-xtensa/elf.h
 *
 * ELF register definitions
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 *	Authors:	Kevin Chea
 *			Marc Gauthier
 *			Joe Taylor <joe@tensilica.com, joetylr@yahoo.com>
 */

#include <asm/ptrace.h>
#include <asm/user.h>
#include <asm/page.h>
#include <xtensa/config/core.h>

/* Xtensa processor ELF architecture-magic number */
#define EM_XTENSA	94
#define EM_XTENSA_OLD	0xABC7

/* ELF register definitions */

#define ELF_NGREG	(sizeof (struct pt_regs) / sizeof(elf_greg_t))

/* TOTAL_CP_SIZE is a byte count.  Round up ELF_NFPREG to the next
 * 16-byte boundary.  Coprocessors are usually defined in TIE, and TIE
 * load/stores can require 16-byte alignment. However, we do not need
 * to ensure 16-byte alignment for each coprocessor's space here
 * because we do not directly load/store coprocessor state to these
 * data structures (see 'struct task_struct' definition in processor.h
 * for that).  We use memcpy() to copy information over to here. */

#define TOTAL_CP_SIZE  (XCHAL_CP0_SA_SIZE + XCHAL_CP1_SA_SIZE + \
			XCHAL_CP2_SA_SIZE + XCHAL_CP3_SA_SIZE + \
			XCHAL_CP4_SA_SIZE + XCHAL_CP5_SA_SIZE + \
			XCHAL_CP6_SA_SIZE + XCHAL_CP7_SA_SIZE)

#define ELF_NFPREG	(((TOTAL_CP_SIZE + 15) & ~15) / sizeof(elf_fpreg_t))

typedef unsigned long elf_greg_t;
typedef elf_greg_t elf_gregset_t[ELF_NGREG];

typedef double elf_fpreg_t;
typedef elf_fpreg_t elf_fpregset_t[ELF_NFPREG] __attribute__ ((aligned(16)));

/*
 * This is used to ensure we don't load something for the wrong architecture.
 */
#define elf_check_arch(x) ( ( (x)->e_machine == EM_XTENSA )  || \
			    ( (x)->e_machine == EM_XTENSA_OLD ) )

/*
 * These are used to set parameters in the core dumps.
 */
#define ELF_CLASS	ELFCLASS32
#if XCHAL_HAVE_LE
#define ELF_DATA	ELFDATA2LSB
#elif XCHAL_HAVE_BE
#define ELF_DATA	ELFDATA2MSB
#else
#error endianess not defined
#endif
#define ELF_ARCH	EM_XTENSA

#define USE_ELF_CORE_DUMP
#define ELF_EXEC_PAGESIZE	PAGE_SIZE

/* This is the location that an ET_DYN program is loaded if exec'ed.  Typical
   use of this is to invoke "./ld.so someprog" to test out a new version of
   the loader.  We need to make sure that it is out of the way of the program
   that it will "exec", and that there is sufficient room for the brk.  */

#define ELF_ET_DYN_BASE         (2 * TASK_SIZE / 3)


#define ELF_CORE_COPY_REGS(_dest,_regs)				\
	memcpy((char *) &_dest, (char *) _regs,			\
	       sizeof(struct pt_regs));

/* This yields a mask that user programs can use to figure out what
   instruction set this CPU supports.  This could be done in user space,
   but it's not easy, and we've already done it here.  */

#define ELF_HWCAP	(0)

/* This yields a string that ld.so will use to load implementation
   specific libraries for optimization.  This is more specific in
   intent than poking at uname or /proc/cpuinfo.

   For the moment, we have only optimizations for the Intel generations,
   but that could change... */

#define ELF_PLATFORM  (NULL)

/* The Xtensa processor ABI says that when the program starts, a2
   contains a pointer to a function which might be registered using
   `atexit'.  This provides a mean for the dynamic linker to call
   DT_FINI functions for shared libraries that have been loaded before
   the code runs.

   A value of 0 tells we have no such handler. 

   We might as well make sure everything else is cleared too (except
   for the stack pointer in a1), just to make things more
   deterministic.  Also, clearing a0 terminates debugger backtraces.
 */

#define ELF_PLAT_INIT(_r) \
  do { _r->aregs[0]=0; /*_r->aregs[1]=0;*/ _r->aregs[2]=0;  _r->aregs[3]=0;  \
       _r->aregs[4]=0;  _r->aregs[5]=0;    _r->aregs[6]=0;  _r->aregs[7]=0;  \
       _r->aregs[8]=0;  _r->aregs[9]=0;    _r->aregs[10]=0; _r->aregs[11]=0; \
       _r->aregs[12]=0; _r->aregs[13]=0;   _r->aregs[14]=0; _r->aregs[15]=0; \
  } while (0)

#ifdef __KERNEL__
#define SET_PERSONALITY(ex, ibcs2) set_personality(PER_LINUX_32BIT)
#endif

#endif /* __ASM_XTENSA_ELF_H */
