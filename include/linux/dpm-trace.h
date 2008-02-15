/*
 * include/linux/dpm.h  DPM policy management
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (C) 2002, International Business Machines Corporation
 * All Rights Reserved
 *
 * Robert Paulsen
 * IBM Linux Technology Center
 * rpaulsen@us.ibm.com
 * August, 2002
 *
 */

#ifndef __DPM_TRACE_H_
#define __DPM_TRACE_H_

#include <linux/config.h>

#ifdef CONFIG_DPM_TRACE

#define DPM_TRACE_SET_OPT_ASYNC  0x00000001
#define DPM_TRACE_SET_OPT_SYNC   0x00000002
#define DPM_TRACE_RESYNC         0x00000004
#define DPM_TRACE_UNLOCK         0x00000008
#define DPM_TRACE_SET_OS         0x00000010
#define DPM_TRACE_SET_POLICY     0x00000020
#define DPM_TRACE_START          0x00000040
#define DPM_TRACE_STOP           0x00000080
#define DPM_TRACE_SET_TASK_STATE 0x00000100
#define DPM_TRACE_CONSTRAINT_ASSERTED 0x00000200

#define DPM_TRACE_ALL            0x000002ff

void dpm_trace(unsigned event, ...);
void dpm_trace_start(unsigned events);
void dpm_trace_stop(void);
void dpm_trace_reset(void);

int
read_proc_dpm_trace(char *page, char **start, off_t offset, 
		    int count, int *eof, void *data);
int 
write_proc_dpm_trace(struct file *file, const char *buffer,
		     unsigned long count, void *data);

#else

#define dpm_trace(args...) do {} while (0)

#endif /* CONFIG_DPM_TRACE */

#endif /*__DPM_TRACE_H_*/
