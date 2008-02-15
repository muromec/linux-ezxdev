/*
 * Copyright (C) 2005 Motorola Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/*
 * created by a17400, for EZX platform
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
/*-------------------------------------------------------------------------*/
char start_reason[20];
u32 power_up_reason=0;    //a17400: store power up reason
//translate u32 pow up reason to char for proc
static char * hex_template="0123456789abcdef";
static void trans_u32_to_char(u32 data, char *desc){
  int i=0;
  desc[8]=0;   //termimate null char
  for (i=7;i>=0;i--){
    desc[i]=hex_template[(data & 0xf)];
    data>>=4;
  }
}

u32 read_powerup_reason_value(void){

	return power_up_reason;
}

static int get_start_reason(char *buffer, char **buf_locate, off_t offset, int len)
{
  trans_u32_to_char(power_up_reason,start_reason);  
	len = sprintf(buffer, start_reason);
	return len;
}

static struct proc_dir_entry *proc_start_reason;
extern struct proc_dir_entry proc_root;
#if 0
{
        0,
        12,"powerup_info",
        S_IFREG | S_IRUGO,
        1, 0, 0,
        0,
        NULL,
        &get_start_reason,
};
#endif
/*--------------------------------------------------------------------------*/
int __init start_reason_init_module(void)
{

	proc_start_reason = &proc_root;
	proc_start_reason->owner = THIS_MODULE;
	create_proc_info_entry("powerup_info", 0, proc_start_reason, get_start_reason);
//	proc_register(&proc_root, &proc_start_reason); 
	printk("proc register for powerup_info\n");
	return 0;
}
void __exit start_reason_cleanup_module(void)
{
	if (proc_start_reason) {
                remove_proc_entry("powerup_info", proc_start_reason);
                proc_start_reason = NULL;
        }	
//	proc_unregister(&proc_root, proc_start_reason.low_ino);	
}
module_init(start_reason_init_module);
module_exit(start_reason_cleanup_module);
MODULE_AUTHOR("Jay Jia");
