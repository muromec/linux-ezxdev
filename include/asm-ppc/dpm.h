/*
 * include/asm/dpm.h        Platform-dependent DPM defines for PPC
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

#ifndef __ASM_DPM_H__
#define __ASM_DPM_H__

#define __NR_sys_dpm 35

#ifdef CONFIG_405LP
#include <platforms/ibm405lp_dpm.h>
#endif

#endif /* __ASM_DPM_H__ */
