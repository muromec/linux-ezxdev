/*================================================================================

Module Name:  ac_syscalls.c

General Description: implement all necessary system calls in Access Control.

==================================================================================
 
   Copyright (C) 2005 - Motorola
 
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 2 as
   published by the Free Software Foundation.
 
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
 
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

Revision History:
                            Modification     Tracking
Author (core ID)                Date          Number      Description of Changes
------------------------    ------------    ----------   -------------------------
Wang Yang (w20619)           02/03/2005     LIBff54376   Init version

==================================================================================*/
/*================================================================================
                                 INCLUDE FILES
================================================================================*/
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/security.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <asm/current.h>
#include "ac_policy.h"
#include "ac_plug.h"

int sys_ac_getdomaintype(int pid, int *domainno)
{
    struct task_struct *task;
    struct task_security_struct *tsec;
    int dno;
  
    task = find_task_by_pid(pid);
    if(!task)
         return -ENOENT;

    tsec = task->security;
    if(!tsec)
        return -ENOENT;
  
    dno = ac_proc_to_dom(tsec->pno);

    if (copy_to_user(domainno,&dno,sizeof(int)))
        return -EFAULT;

    return 0;
}

int sys_ac_getprocessid(int pid,int *procno)
{
    struct task_struct *task;
    struct task_security_struct *tsec;

    task = find_task_by_pid(pid);
    if (!task)
	return -ENOENT;

    tsec = task->security;
    if (!tsec)
	return -ENOENT;

    if (copy_to_user(procno,&tsec->pno,sizeof(int)))
	return -EFAULT;

    return 0;
}

int sys_ac_setdomaintype(int dno)
{
    struct task_security_struct *tsec;
    int ret;
    int proc_no;
   
    if (dno == 0)
        return -EINVAL;

    if ((dno != 0) && ((ret = ac_dom_exist(dno)) == 0))
	return -ENOENT;
    
    tsec = current->security;

    ret = ac_task_has_perm(tsec->pno,AC_CLASS_SECURITY,AC_SECURITY_ACCESS);
    if (ret < 0)
       return ret;
	    
    if ((tsec->in_sid != 0) && (dno != 0))
	return -EINVAL;

    proc_no = ac_dom_default_proc(dno);
    if (proc_no <= 0)
        return proc_no;
	
    tsec->in_sid = proc_no;

    return 0;
}

static spinlock_t ac_value_lock = SPIN_LOCK_UNLOCKED;

int sys_ac_enable(int flag)
{
    struct task_security_struct *tsec;
    int ret;
    
    tsec = current->security;

    if (tsec)
    {
        ret = ac_task_has_perm(tsec->pno,AC_CLASS_SECURITY,AC_SECURITY_ACCESS);
        if (ret < 0)
           return ret;
    
        spin_lock(&ac_value_lock);
        if (flag == 0)
           ac_policy_enable = 0;
        else
           ac_policy_enable = 1;
        spin_unlock(&ac_value_lock);

        return 0;    
    }
    else
    {
        return -1;
    }
}

long sys_ac_execv(const char *path, char **argv,char **envp,int domaintype,struct pt_regs *regp)
{
    int err;
    char *filename;
    struct task_security_struct *tsec;
    int proc_no;

    if (path == NULL)
        return -1;

    tsec = current->security;
    err = ac_task_has_perm(tsec->pno,AC_CLASS_SECURITY,AC_SECURITY_ACCESS);
    if (err < 0)
	return err;

    proc_no = ac_dom_default_proc(domaintype);
    if (proc_no <= 0)
	return proc_no;
    
    filename = getname(path);
    err = PTR_ERR(filename);
    if (IS_ERR(filename))
	return err;
   
    tsec->in_sid = proc_no;
    err = do_execve(filename,argv,envp,regp);
    tsec->in_sid = 0;
    if (err == 0)
	current->ptrace &= ~PT_DTRACE;

    putname(filename);

    return err;    
}

long sys_ac_getressid(const char *path,int *sid)
{
	int err;
	char *tmp;
	struct nameidata nd;
	struct inode_security_struct *isec;

    if (path == NULL)
        return -EINVAL;

	tmp = getname(path);
	err = PTR_ERR(tmp);
	if (IS_ERR(tmp))
	    return err;

	if (path_init(tmp,LOOKUP_FOLLOW|LOOKUP_POSITIVE,&nd))
	    err = path_walk(tmp,&nd);
	if (err)
	{
	    putname(tmp);
	    return -ENOENT;
	}

	putname(tmp);
	isec = nd.dentry->d_inode->i_security;
	if (!isec)
	    return -EFAULT;		    
    
	if (copy_to_user(sid,&isec->rno,sizeof(int)))
	    return -EFAULT;

	return 0;
	    
}

int sys_task_to_dom(struct task_struct *p)
{
  struct task_security_struct *tsec;
  int dno;

  tsec = p->security;
  if(!tsec)
  {
      dno = -EFAULT;
  }
  else
  {
      dno = ac_proc_to_dom(tsec->pno);
  }
  return dno;
}

int sys_task_to_proc(struct task_struct *p)
{
  struct task_security_struct *tsec;
  int pno;

  tsec = p->security;
  if(!tsec)
  {
      pno = -EFAULT;
  }
  else
  {
      pno = tsec->pno;
  }
  return pno;
}

#define AL(x) ((x) * sizeof(unsigned long))
static unsigned char nargs[AC_SYSCALL_NUM] = 
{
    AL(0),   /* NULL */
    AL(2),   /* get domain type */
    AL(2),   /* get process number */
    AL(1),   /* set domain type */
    AL(4),   /* security execv */
    AL(2),   /* get resource SID */
    AL(1)    /* disable/enable ac function */
};

long sys_security_ac_worker(unsigned int magic, unsigned int call,unsigned long *args,struct pt_regs *regp)
{
    unsigned long a[4];
    unsigned long a0,a1;
    int err = -EINVAL;
	    
    a0 = a1 = 0;
    if (magic != AC_POLICY_MAGIC)
        return -ENOSYS;

    if (call < 1 || call >= AC_SYSCALL_NUM)
	return -EINVAL;

    if (nargs[call] > sizeof(a))
	return -EFAULT;
   
    if (copy_from_user(a,args,nargs[call]))
	return -EFAULT;

    a0 = a[0];

    if (nargs[call] > AL(1))
        a1 = a[1];

    switch (call)
    {
	case AC_SYSCALL_GETDOMAIN:
	     err = sys_ac_getdomaintype(a0,(int *)a1);
	     break;
	case AC_SYSCALL_GETPROCNO:
	     err = sys_ac_getprocessid(a0,(int *)a1); 
	     break;
	case AC_SYSCALL_SETDOMAINTYPE:
	     err = sys_ac_setdomaintype(a0);
	     break;
	case AC_SYSCALL_EXECV:
	     err = sys_ac_execv((char *)a0,(char **)a1,(char **)a[2],a[3],regp);
	     break;
	case AC_SYSCALL_GETRESSID:
	     err = sys_ac_getressid((char *)a0,(int *)a1);
         break;
        case AC_SYSCALL_ENABLE:
             err = sys_ac_enable(a0);
	     break;
    default:
         err = -EINVAL;
	     break;
    }
	    
    return err;
}
#undef AL

asmlinkage long sys_security_ac(struct pt_regs regs)
{
    unsigned int magic = regs.ARM_r0;
    unsigned int call = regs.ARM_r1;
    unsigned long *args = (unsigned long *)regs.ARM_r2;
    
    return sys_security_ac_worker(magic,call,args,&regs);   
}
		
EXPORT_SYMBOL(sys_task_to_dom);
EXPORT_SYMBOL(sys_task_to_proc);
EXPORT_SYMBOL(sys_security_ac);
