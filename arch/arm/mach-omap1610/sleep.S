/*
 * Low-level PXA250/210 sleep/wakeUp support
 *
 * Initial SA1110 code:
 * Copyright (c) 2001 Cliff Brake <cbrake@accelent.com>
 *
 * Adapted for PXA by Nicolas Pitre:
 * Copyright (c) 2002 Monta Vista Software, Inc.
 *
 * Modified for the OMAP1610 by Alexandr Shkurenkov:
 * Copyright (c) 2003 Monta Vista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */

#include <linux/config.h>
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/hardware.h>
#include <asm/arch/pm.h>

		.text

/*
 * Forces CPU into sleep state
 *
 * omap1610_idle_loop_suspend()
 *
 * cpu_omap_suspend()'s resume point.
 *
 * Note: This code get's copied to SRAM at boot.  When the omap
 *	wakes up it continues execution at the point it went to sleep.
 *	It will just start executing here, so we'll restore stuff from the
 *	stack, reset the ARM_IDLECT1 and ARM_IDLECT2 and return 
 */

ENTRY(omap1610_idle_loop_suspend)

	stmfd	sp!, {r0 - r12, lr}		@ save registers on stack

	mov	r4, #ARM_REG_BASE & 0xff000000
	orr	r4, r4, #ARM_REG_BASE & 0x00ff0000
	orr	r4, r4, #ARM_REG_BASE & 0x0000ff00

	@ turn off clock domains
	@ get arm_idlect2 into r1
	ldrh	r1, [r4, #ARM_ASM_IDLECT2 & 0xff]
	mov	r5, #IDLE_CLOCK_DOMAINS & 0xff
 	orr	r5,r5, #IDLE_CLOCK_DOMAINS & 0xff00
	strh	r5, [r4, #ARM_ASM_IDLECT2 & 0xff]

	@ request ARM idle
	@ get arm_idlect1 into r2
	ldrh	r2, [r4, #ARM_ASM_IDLECT1 & 0xff]
	orr	r3, r2, #IDLE_LOOP_REQUEST
	strh	r3, [r4, #ARM_ASM_IDLECT1 & 0xff]

	mov	r5, #IDLE_WAIT_CYCLES
loop:	subs	r5, r5, #1
	bne	loop
/*
 * Let's wait for the next clock tick to wake us up.
 */
	mcr	p15, 0, r0, c7, c0, 4		@ wait for interrupt
/*
 * cpu_omap_suspend()'s resume point.
 *
 * Note: This code get's copied to SRAM at boot.  When the omap
 *	wakes up it continues execution at the point it went to sleep.
 *	It will just start executing here, so we'll restore stuff from the
 *	stack, reset the ARM_IDLECT1 and ARM_IDLECT2 and return to
 *	the address passed in arg0.
 */

	@ restore ARM_IDLECT1 and ARM_IDLECT2 and return
	@ r2 has arm_idlect1 and r1 still has arm_idlect2
	strh	r1, [r4, #ARM_ASM_IDLECT2 & 0xff]
	strh	r2, [r4, #ARM_ASM_IDLECT1 & 0xff]

	ldmfd   sp!, {r0 - r12, pc}     @ restore regs and return

ENTRY(omap1610_idle_loop_suspend_sz)
	.word	. - omap1610_idle_loop_suspend

ENTRY(omap1610_cpu_suspend)

	stmfd	sp!, {r0 - r12, lr}		@ save registers on stack

	@ prepare to put SDRAM into self-refresh manually
	mov	r4, #TCMIF_BASE & 0xff000000
	orr	r4, r4, #TCMIF_BASE & 0x00ff0000
	orr	r4, r4, #TCMIF_BASE & 0x0000ff00

	ldr	r5, [r4, #EMIFF_ASM_SDRAM_CONFIG & 0xff]
	orr	r5, r5, #SELF_REFRESH_MODE & 0xff000000
	orr	r5, r5, #SELF_REFRESH_MODE & 0x000000ff
	str	r5, [r4, #EMIFF_ASM_SDRAM_CONFIG & 0xff]

	@ prepare to put EMIFS to Sleep
	ldr	r5, [r4, #EMIFS_ASM_CONFIG_REG & 0xff]
	orr	r5, r5, #IDLE_EMIFS_REQUEST
	str	r5, [r4, #EMIFS_ASM_CONFIG_REG & 0xff]

	@ turn off clock domains
	mov	r4, #ARM_REG_BASE & 0xff000000
	orr	r4, r4, #ARM_REG_BASE & 0x00ff0000
	orr	r4, r4, #ARM_REG_BASE & 0x0000ff00
	ldrh	r1, [r4, #ARM_ASM_IDLECT2 & 0xff]
	mov	r5, #IDLE_CLOCK_DOMAINS & 0xff
 	orr	r5,r5, #IDLE_CLOCK_DOMAINS & 0xff00
	strh	r5, [r4, #ARM_ASM_IDLECT2 & 0xff]
	
	@ work around errata of OMAP1610/5912. Enable (!) peripheral
	@ clock to let the chip go into deep sleep
	ldrh	r5, [r4, #ARM_ASM_IDLECT2 & 0xff]
 	orr	r5,r5, #EN_PERCK_BIT & 0xff
	strh	r5, [r4, #ARM_ASM_IDLECT2 & 0xff]

	@ request ARM idle
	ldrh	r2, [r4, #ARM_ASM_IDLECT1 & 0xff]
	mov	r3, #DEEP_SLEEP_REQUEST & 0xff
	orr	r3, r3, #DEEP_SLEEP_REQUEST & 0xff00
	strh	r3, [r4, #ARM_ASM_IDLECT1 & 0xff]
	
/*
 * The next UART interrupt will awaken it.
 */
	mov	r5, #IDLE_WAIT_CYCLES
loop2:	subs	r5, r5, #1
	bne	loop2

	mcr	p15, 0, r0, c7, c0, 4		@ wait for interrupt

	strh	r1, [r4, #ARM_ASM_IDLECT2 & 0xff]
	strh	r2, [r4, #ARM_ASM_IDLECT1 & 0xff]

	ldmfd   sp!, {r0 - r12, pc}     @ restore regs and return

ENTRY(omap1610_cpu_suspend_sz)
	.word	. - omap1610_cpu_suspend
