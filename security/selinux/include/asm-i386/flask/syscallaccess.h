/* -*- linux-c -*- */

#ifndef _ASM_FLASK_SYSCALLACCESS_H_
#define _ASM_FLASK_SYSCALLACCESS_H_

extern void *sys_call_table[];

#define SELINUX_SYSCALL_GET(x) sys_call_table[x]
#define SELINUX_SYSCALL_SET(x, y) sys_call_table[x] = y

#endif
