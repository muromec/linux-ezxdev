/*
 * include/asm/dpm.h        
 *
 *  Platform-dependent DPM defines for SH
 *
 * SH-specific DPM support
 *
 * Author: Dmitrij Frasenyak <sed@ru.mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __ASM_DPM_H__
#define __ASM_DPM_H__

#ifdef CONFIG_CPU_SUBTYPE_SH7300
#include <asm/sh_mobile_dpm.h>
#elif defined(CONFIG_CPU_SUBTYPE_SH73180)
#include <asm/sh_mobile3_dpm.h>
#endif

#define __NR_sys_dpm 239

static inline void
dpm_relock_setup(struct dpm_md_opt *md_relock, struct dpm_md_opt *md_new)
{
}

#endif /* __ASM_DPM_H__ */
