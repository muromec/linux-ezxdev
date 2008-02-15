/*
 * include/asm-sh/sh_mobile3_pm.h
 *
 * Power Management header file for SH-Mobile3(SH73180CP01).
 *
 * Author: Hirsohi DOYU <Hiroshi_DOYU@montavista.co.jp>
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

/*
 * List of global SH-Mobile3 registers to preserve. 
 * More ones like CP and general purpose register values are preserved
 * with the stack pointer in sleep.S.
 */
#ifndef __ASM_SH_MOBILE3_PM_H__
#define __ASM_SH_MOBILE3_PM_H__

#include <asm/time.h>
#include <asm/cpu-regs.h>
#include <asm/irq.h>

#define SHM3_FLASH_ETHBOOT      0xb8000000
#define SHM3_KERN_ENTRY_STEXT   0xa0001000

#define SHM3_SRAM_API_IPL	0xa0000000
#define SHM3_SRAM_API_SUSPEND	0xa0000400
#define SHM3_SRAM_API_RESUME	0xa0000500
#define SHM3_SRAM_REGS_SAVE     (SHM3_KERN_ENTRY_STEXT - 4)

#define SHM3_SW5_ADDR           0xb1000000

#define MMUCR_RESUME_INIT       0x00000005  /* TI AT */
#define CCR_RESUME_INIT         0x0000090d  /* ICI ICE OCI CB OCE */

/* Stand-by status is stored in "sh73180_pm_standby_mode" */
#define USTBY                   0x10  /* SW5-5 */
#define RSTBY                   0x20  /* SW5-6 */
#define MLRST                   0x40  /* This bit will be set after manual reset */
#define SWSTBY                  0x80  /* SW5-8 */


#ifndef __ASSEMBLER__

extern unsigned short shm3_stby_mode;

struct timer_saved_regs {
	__u32 tcor;
	__u32 tcnt;
	__u16  tcr; 
};

struct tmu_saved_regs {
	__u8  tstr;
	struct timer_saved_regs t[3];
};

static inline void 
shm3_tmu_save(struct tmu_saved_regs* r)
{
	r->tstr = ctrl_inb(TMU_TSTR);
	
	/* TMU0 */
	disable_irq(TMU0_IRQ);
	r->t[0].tcor = ctrl_inl(TMU0_TCOR);
	r->t[0].tcnt = ctrl_inl(TMU0_TCNT);
	r->t[0].tcr  = ctrl_inw(TMU0_TCR);

	/* TMU1 */
	if (r->tstr & 0x02) {
		disable_irq(TMU1_IRQ);
		r->t[1].tcor = ctrl_inl(TMU1_TCOR);
		r->t[1].tcnt = ctrl_inl(TMU1_TCNT);
		r->t[1].tcr  = ctrl_inw(TMU1_TCR);
	}
	/* TMU2 */
	if (r->tstr & 0x04) {
		disable_irq(TMU2_IRQ);
		r->t[2].tcor = ctrl_inl(TMU2_TCOR);
		r->t[2].tcnt = ctrl_inl(TMU2_TCNT);
		r->t[2].tcr  = ctrl_inw(TMU2_TCR);
	}
	

	return;
}

static inline void 
shm3_tmu_restore(struct tmu_saved_regs* r)
{
	/* TMU0 */
	ctrl_outl(r->t[0].tcor, TMU0_TCOR);
	ctrl_outl(r->t[0].tcor, TMU0_TCNT);
	ctrl_outw(r->t[0].tcr , TMU0_TCR);
	enable_irq(TMU0_IRQ);
	
	/* TMU1 */
	if (r->tstr & 0x02) {
		ctrl_outl(r->t[1].tcor, TMU1_TCOR);
		ctrl_outl(r->t[1].tcor, TMU1_TCNT);
		ctrl_outw(r->t[1].tcr , TMU1_TCR);
		enable_irq(TMU1_IRQ);
	}
	/* TMU2 */
	if (r->tstr & 0x04) {
		ctrl_outl(r->t[2].tcor, TMU2_TCOR);
		ctrl_outl(r->t[2].tcor, TMU2_TCNT);
		ctrl_outw(r->t[2].tcr , TMU2_TCR);
		enable_irq(TMU2_IRQ);
	}
	
	/* Timer start */
	ctrl_outb(r->tstr, TMU_TSTR);
	
	return;
}

extern void shm3_cpu_ipl(void);
extern void shm3_cpu_suspend(void);
extern void shm3_cpu_resume(void);
extern void nmi_restore_dma(void);

#endif	/* ASSEMBLER */
#endif /* __ASM_SH_MOBILE3_PM_H__ */

