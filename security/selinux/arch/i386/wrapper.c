#include <linux/kernel.h>
#include <linux/ptrace.h>

extern long sys_security_selinux_worker(unsigned int magic,
					unsigned int call, 
					unsigned long* args, 
					struct pt_regs* regp);

asmlinkage long sys_security_selinux(struct pt_regs regs) 
{
	unsigned int magic = regs.ebx;
	unsigned int call = regs.ecx;
	unsigned long *args = (unsigned long *)regs.edx;
	return sys_security_selinux_worker(magic, call, args, &regs);
}
	
