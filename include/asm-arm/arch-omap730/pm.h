/*
 * include/asm/arch-omap730/pm.h
 *
 * Author: MontaVista Software, Inc.
 *         support@mvista.com
 *
 * Copyright 2002 MontaVista Software Inc.
 *
 * Modifications for the OMAP730 cpu :
 * Author: MPC-Data Limited
 *         Dave Peverley <dpeverley@mpc-data.co.uk>
 *
 * Copyright (c) 2004, MPC-Data Limited
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * List of global OMAP registers to preserve.  All registers are 16 bits
 * and must be accessed with 16 read/writes.
 * More ones like CP and general purpose register values are preserved
 * with the stack pointer in sleep.S.
 */
#ifndef __ASM_ARCH_OMAP730_PM_H
#define __ASM_ARCH_OMAP730_PM_H  

#define ARM_REG_BASE               (0xfffece00)
#define ARM_ASM_IDLECT1            (ARM_REG_BASE + 0x4)    
#define ARM_ASM_IDLECT2            (ARM_REG_BASE + 0x8)    
#define ARM_ASM_RSTCT1             (ARM_REG_BASE + 0x10)    
#define ARM_ASM_RSTCT2             (ARM_REG_BASE + 0x14)    
#define ARM_ASM_SYSST              (ARM_REG_BASE + 0x18)
/*
 * Traffic Controller Memory Interface Registers
 */
#define TCMIF_BASE                 0xfffecc00
#define EMIFS_ASM_CONFIG_REG       (TCMIF_BASE + 0x0c)
#define EMIFF_ASM_SDRAM_CONFIG     (TCMIF_BASE + 0x20)
#define IRQ_MIR1                   (volatile unsigned int *)(OMAP_IH1_BASE + IRQ_MIR)
#define IRQ_MIR2_0                 (volatile unsigned int *)(OMAP_IH2_0_BASE + IRQ_MIR)
#define IRQ_MIR2_1                 (volatile unsigned int *)(OMAP_IH2_1_BASE + IRQ_MIR)

#define IDLE_WAIT_CYCLES           0x000000ff
#define DEEP_SLEEP_REQUEST         0x16C7      // (Value for ARM_IDLECT1)
#define IDLE_LOOP_REQUEST          0x0C00      // (Value for ARM_IDLECT1)                  [[ 0x0400 on 1610 ]]
#define SELF_REFRESH_MODE          0x0C000001
#define IDLE_EMIFS_REQUEST         0x000C
#define IDLE_CLOCK_DOMAINS         0x09C7      // (Value for ARM_IDLECT2 - Set EN_XORPCK)  [[ 0x09c7 on 1610 ]]
#define MODEM_32K_EN               0x0001
#define PER_EN                     0x0001      // (Value for ARM_RSTCT2)


#define IRQ_IH2_IRQ                (1 << 1)    // Mask for IH1 MIR
#define IRQ_UART1                  (1 << 14)   // Mask for IH2 MIR
#define IRQ_UART2                  (1 << 15)   // Mask for IH2 MIR


#ifndef __ASSEMBLER__
extern void omap730_pm_idle(void);
extern void omap_pm_suspend(void);
extern int omap730_cpu_suspend(void);
extern int omap730_idle_loop_suspend(void);
extern struct async_struct *omap_pm_sercons;
extern unsigned int serial_in(struct async_struct *, int);
extern unsigned int serial_out(struct async_struct *, int, int);

#define OMAP730_SRAM_IDLE_SUSPEND  0xd0000400
#define OMAP730_SRAM_API_SUSPEND   0xd0000600
#define CPU_SUSPEND_SIZE           200
#define ARM_REG_BASE               (0xfffece00)
#define ARM_ASM_IDLECT1            (ARM_REG_BASE + 0x4)    
#define ARM_ASM_IDLECT2            (ARM_REG_BASE + 0x8)    
#define ARM_ASM_RSTCT1             (ARM_REG_BASE + 0x10)    
#define ARM_ASM_RSTCT2             (ARM_REG_BASE + 0x14)    
#define ARM_ASM_SYSST              (ARM_REG_BASE + 0x18)

#define ULPD_REG_BASE              (0xfffe0800)
#define ULPD_IT_STATUS_REG         (volatile __u16 *)(ULPD_REG_BASE + 0x14)
#define ULPD_CLOCK_CTRL_REG        (volatile __u16 *)(ULPD_REG_BASE + 0x30)
#define ULPD_SOFT_REQ_REG          (volatile __u16 *)(ULPD_REG_BASE + 0x34)
#define ULPD_STATUS_REQ_REG        (volatile __u16 *)(ULPD_REG_BASE + 0x40)
#define ULPD_POWER_CTRL_REG        (volatile __u16 *)(ULPD_REG_BASE + 0x50)
#define ULPD_SLEEP_STATUS          (volatile __u16 *)(ULPD_REG_BASE + 0x58)
#define ULPD_STATUS_REQ_REG2       (volatile __u16 *)(ULPD_REG_BASE + 0x54)



#define TCMIF_BASE                 0xfffecc00
#define PM_EMIFS_CONFIG_REG        (volatile unsigned int *)(TCMIF_BASE + 0x0c)
#define PM_EMIFF_SDRAM_CONFIG      (volatile unsigned int *)(TCMIF_BASE + 0x20)

#define DSP_IDLE_DELAY             10

#define SUFFICIENT_DSP_RESET_TIME  1000
#define ENABLE_XORCLK              0x2                  // in ARM_IDLECT2
#define DSP_RESET                  0x2000               // in ARM_RSTCT1


#define ARM_SAVE(x) arm_sleep_save[ARM_SLEEP_SAVE_##x] = *((volatile unsigned short *)x)
#define ARM_RESTORE(x) *((volatile unsigned short *)x) = (unsigned short)arm_sleep_save[ARM_SLEEP_SAVE_##x] 
#define ARM_SHOW(x) arm_sleep_save[ARM_SLEEP_SAVE_##x]

#define ULPD_SAVE(x) ulpd_sleep_save[ULPD_SLEEP_SAVE_##x] = *((volatile unsigned short *)x)
#define ULPD_RESTORE(x) *((volatile unsigned short *)x) = (unsigned short)ulpd_sleep_save[ULPD_SLEEP_SAVE_##x] 
#define ULPD_SHOW(x) ulpd_sleep_save[ULPD_SLEEP_SAVE_##x]

#define MPUI_SAVE(x) mpui_sleep_save[MPUI_SLEEP_SAVE_##x] = *((volatile unsigned int *)x)
#define MPUI_RESTORE(x) *((volatile unsigned int *)x) = (unsigned int)mpui_sleep_save[MPUI_SLEEP_SAVE_##x] 
#define MPUI_SHOW(x) (unsigned int)mpui_sleep_save[MPUI_SLEEP_SAVE_##x]

enum arm_save_state {
        ARM_SLEEP_SAVE_START = 0,
	/* 16 bit registers */
        ARM_SLEEP_SAVE_ARM_CKCTL, 
        ARM_SLEEP_SAVE_ARM_IDLECT1, 
        ARM_SLEEP_SAVE_ARM_IDLECT2, 
        ARM_SLEEP_SAVE_ARM_EWUPCT, 
        ARM_SLEEP_SAVE_ARM_RSTCT1, 
        ARM_SLEEP_SAVE_ARM_RSTCT2,
        ARM_SLEEP_SAVE_ARM_SYSST, 
        ARM_SLEEP_SAVE_ARM_IDLECT3, 
        ARM_SLEEP_SAVE_SIZE
};

enum ulpd_save_state {
        ULDP_SLEEP_SAVE_START = 0,
	/* 16 bit registers */
        ULPD_SLEEP_SAVE_ULPD_IT_STATUS_REG,
        ULPD_SLEEP_SAVE_ULPD_CLOCK_CTRL_REG,
        ULPD_SLEEP_SAVE_ULPD_SOFT_REQ_REG, 
        ULPD_SLEEP_SAVE_ULPD_STATUS_REQ_REG, 
        ULPD_SLEEP_SAVE_ULPD_POWER_CTRL_REG,
        ULPD_SLEEP_SAVE_ULPD_STATUS_REQ_REG2, 
        ULPD_SLEEP_SAVE_SIZE
};

enum mpui_save_state {
        MPUI_SLEEP_SAVE_START = 0,
	/* 32 bit registers */
        MPUI_SLEEP_SAVE_PM_EMIFF_SDRAM_CONFIG,   // 32 bit 
        MPUI_SLEEP_SAVE_PM_EMIFS_CONFIG_REG,     // 32 bit
        MPUI_SLEEP_SAVE_IRQ_MIR1,                // 32 bit
        MPUI_SLEEP_SAVE_IRQ_MIR2_0,              // 32 bit
        MPUI_SLEEP_SAVE_IRQ_MIR2_1,              // 32 bit
        MPUI_SLEEP_SAVE_SIZE
};


#endif        /* ASSEMBLER */
#endif 
