/*
 * include/asm/cpu-regs.h
 *
 * Choose cpu regs acording to .config file.
 *
 * Author: Dmitrij Frasenyak <sed@ru.mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __INCLUDE_ASM_CPU_REGS_H__
#define __INCLUDE_ASM_CPU_REGS_H__

#include <linux/config.h>

#if defined (CONFIG_CPU_SUBTYPE_SH7300)
#include <asm/sh7300-regs.h>
#endif

#if defined (CONFIG_CPU_SUBTYPE_SH73180)
#include <asm/sh73180-regs.h>
#endif

#endif
