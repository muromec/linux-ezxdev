/****************************************************************************

  registers.c: Register monitor of OMAP710

  Initially based upon arch/arm/mach-sa1100/registers.c
  Date: Fri, 2 Feb 2001 10:03:10 -0500
  From: Sukjae Cho <sjcho@east.isi.edu>

The code has a long table but simple. It makes proc file of each OMAP710
I/O register under /proc/cpu/registers directory. Each register can be
read/written via cat/echo respectively.

This module does not implement full file i/o support, and is made for just
debugging and testing purpose.

****************************************************************************/

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>	/* because we are a module */
#include <linux/init.h>		/* for the __init macros */
#include <linux/proc_fs.h>	/* all the /proc functions */
#include <linux/ioport.h>
#include <asm/uaccess.h>	/* to copy to/from userspace */
#include <asm/arch/hardware.h>

#define MODULE_NAME "regmon"
#define CPU_DIRNAME "cpu"
#define REG_DIRNAME "registers"

#define	io_p2v(x)	(x)

static ssize_t proc_read_reg(struct file *file, char *buf,
			     size_t nbytes, loff_t * ppos);
static ssize_t proc_write_reg(struct file *file, const char *buffer,
			      size_t count, loff_t * ppos);

static struct file_operations proc_reg_operations = {
	read:proc_read_reg,
	write:proc_write_reg
};

typedef struct omap710_reg_entry {
	u32 phyaddr;
	char *name;
	char *description;
	unsigned short low_ino;
} omap710_reg_entry_t;

static omap710_reg_entry_t omap710_regs[] = {
/* ARMINTH Registers 0xfffecb00: */
	{ 0xfffecb00, "ARMINTH_ITR", "ARMINTH Interrupt" },
	{ 0xfffecb04, "ARMINTH_MIR", "ARMINTH Mask interrupt" },
	{ 0xfffecb10, "ARMINTH_SIR_IRQ_CODE", "ARMINTH IRQ interrupt encoded source" },
	{ 0xfffecb14, "ARMINTH_SIR_FIQ_CODE", "ARMINTH FIQ interrupt encoded source" },
	{ 0xfffecb18, "ARMINTH_CONTROL", "ARMINTH interrupt control" },
	{ 0xfffecb1c, "ARMINTH_ILRX", "ARMINTH interrupt priority Level" },
	{ 0xfffecb9c, "ARMINTH_ISR", "ARMINTH interrupt set" },
/* MPU TIPB Private Registers 0xfffed300: */
	{ 0xfffed300, "pTIPB_RHEA_CNTL", "TIPB (private) control" },
	{ 0xfffed304, "pTIPB_RHEA_BUS_ALLOC", "TIPB (private) allocation" },
	{ 0xfffed308, "pTIPB_ARM_RHEA_CNTL", "TIPB (private) MPU TIPB (private) control" },
	{ 0xfffed30c, "pTIPB_ENHANCED_RHEA_CNTL", "TIPB (private) Enhanced TIPB (private) control" },
	{ 0xfffed310, "pTIPB_ADDRESS_DEBUG", "TIPB (private) Debug address" },
	{ 0xfffed314, "pTIPB_DATA_DEBUG_LOW", "TIPB (private) Debug data LSB" },
	{ 0xfffed318, "pTIPB_DATA_DEBUG_HIGH", "TIPB (private) Debug data MSB" },
	{ 0xfffed31c, "pTIPB_DEBUG_CNTR_SIG", "TIPB (private) Debug control signals" },
/* MPU TIPB Public Registers 0xfffed300: */
	{ 0xfffeca00, "gTIPB_RHEA_CNTL", "TIPB (public) control" },
	{ 0xfffeca04, "gTIPB_RHEA_BUS_ALLOC", "TIPB (public) allocation" },
	{ 0xfffeca08, "gTIPB_ARM_RHEA_CNTL", "TIPB (public) MPU TIPB (public) control" },
	{ 0xfffeca0c, "gTIPB_ENHANCED_RHEA_CNTL", "TIPB (public) Enhanced TIPB (public) control" },
	{ 0xfffeca10, "gTIPB_ADDRESS_DEBUG", "TIPB (public) Debug address" },
	{ 0xfffeca14, "gTIPB_DATA_DEBUG_LOW", "TIPB (public) Debug data LSB" },
	{ 0xfffeca18, "gTIPB_DATA_DEBUG_HIGH", "TIPB (public) Debug data MSB" },
	{ 0xfffeca1c, "gTIPB_DEBUG_CNTR_SIG", "TIPB (public) Debug control signals" },
/* MPU Timer 1 Registers (0xfffec500)  */
	{ 0xfffec500, "MPU_T1_CNTL_TIMER", "MPU T1 Control timer" },
	{ 0xfffec504, "MPU_T1_LOAD_TIM", "MPU T1 Load timer" },
	{ 0xfffec508, "MPU_T1_READ_TIM", "MPU T1 Read timer" },
/* MPU Timer 2 Registers (0xfffec600)  */
	{ 0xfffec600, "MPU_T2_CNTL_TIMER", "MPU T2 Control timer" },
	{ 0xfffec604, "MPU_T2_LOAD_TIM", "MPU T2 Load timer" },
	{ 0xfffec608, "MPU_T2_READ_TIM", "MPU T2 Read timer" },
/* MPU Timer 3 Registers (0xfffec700)  */
	{ 0xfffec700, "MPU_T3_CNTL_TIMER", "MPU T3 Control timer" },
	{ 0xfffec704, "MPU_T3_LOAD_TIM", "MPU T3 Load timer" },
	{ 0xfffec708, "MPU_T3_READ_TIM", "MPU T3 Read timer" },
/* MPU Gigacell UART Registers (0xfffce800)  */
	/* Support for these registers intentionally excluded. */
/* MPU Watchdog Registers (0xfffec800)  */
	{ 0xfffec800, "MPU_WD_CNTL_TIMER", "MPU Watchdog Control timer" },
	{ 0xfffec804, "MPU_WD_LOAD_TIMER", "MPU Watchdog Load timer" },
	{ 0xfffec804, "MPU_WD_READ_TIMER", "MPU Watchdog Read timer" },
	{ 0xfffec808, "MPU_WD_TIMER_MODE", "MPU Watchdog Timer mode" },
/* CompactFlash Controller Registers (0xfffbe000)  */
	{ 0xfffbe000, "TIPB_CF_STS_CFG", "TIPB CF status & configuration" },
	{ 0xfffbe004, "TIPB_CF_CONTROL", "TIPB CF control" },
/* Clock Reset Registers (0xfffece00)  */
	{ 0xfffece00, "ARM_CKCTL", "MPU clock control" },
	{ 0xfffece04, "ARM_IDLECT1", "MPU idle control 1" },
	{ 0xfffece08, "ARM_IDLECT2", "MPU idle control 2" },
	{ 0xfffece0c, "ARM_EWUPCT", "MPU external power control" },
	{ 0xfffece10, "ARM_RSTCT1", "MPU reset control 1" },
	{ 0xfffece14, "ARM_RSTCT2", "MPU reset control 2" },
	{ 0xfffece18, "ARM_SYSST", "MPU system status" },
	{ 0xfffece1c, "ARM_CKOUT1", "MPU clock out 1" },
	{ 0xfffece20, "ARM_CKOUT2", "MPU clock out 2" },
	{ 0xfffece24, "CLKM2_CKTL", "UART clock control" },
	{ 0xfffece28, "CLKM2_IDLECT1", "Clock domain idle mode entry control 1" },
	{ 0xfffece2c, "CLKM2_IDLECT2", "Clock domain idle mode entry control 2" },
/* DPLL Registers (0xfffecf00)  */
	{ 0xfffecf00, "DPLL1_CTL_REG", "DPLL1 control" },
	{ 0xfffed100, "DPLL3_CTL_REG", "DPLL3 control" },
/* Enhanced Audio Controller Registers (0xfffbb000)  */
	/* TODO: Needs 16-bit register read/write support:
	{ 0xfffbb000, "EAC_CPCFR1", "EAC Codec port configuration 1" },
	{ 0xfffbb002, "EAC_CPCFR2", "EAC Codec port configuration 2" },
	{ 0xfffbb004, "EAC_CPCFR3", "EAC Codec port interface configuration 3" },
	{ 0xfffbb006, "EAC_CPCFR4", "EAC Codec port interface configuration 4" },
	{ 0xfffbb008, "EAC_CPTCTL", "EAC Codec port interface control and status" },
	{ 0xfffbb00a, "EAC_CPTTADR", "EAC Codec port interface address" },
	{ 0xfffbb00c, "EAC_CPTDATL", "EAC Codec port interface data LSB" },
	{ 0xfffbb00e, "EAC_CPTDATH", "EAC Codec port interface data MSB" },
	{ 0xfffbb010, "EAC_CPTVSLL", "EAC Codec port interface valid time slots LSB" },
	{ 0xfffbb012, "EAC_CPTVSLH", "EAC Codec port interface valid time slots MSB" },
	{ 0xfffbb020, "EAC_MPCTR", "EAC Modem port control" },
	{ 0xfffbb022, "EAC_MPMCCFR", "EAC Modem port main channel configuration" },
	{ 0xfffbb024, "EAC_MPACCFR", "EAC Modem port auxiliary channel configuration" },
	{ 0xfffbb026, "EAC_MPADLTR", "EAC Modem port auxiliary data LSB transmit" },
	{ 0xfffbb028, "EAC_MPADMTR", "EAC Modem port auxiliary data MSB transmit" },
	{ 0xfffbb02a, "EAC_MPADLRR", "EAC Modem port auxiliary data LSB receive" },
	{ 0xfffbb02c, "EAC_MPADMRR", "EAC Modem port auxiliary data MSB receive" },
	{ 0xfffbb030, "EAC_BPCTR", "EAC Bluetooth port control" },
	{ 0xfffbb032, "EAC_BPMCCFR", "EAC Bluetooth port main channel configuration" },
	{ 0xfffbb034, "EAC_BPACCFR", "EAC Bluetooth port auxiliary channel configuration" },
	{ 0xfffbb036, "EAC_BPADLTR", "EAC Bluetooth port auxiliary data LSB transmit" },
	{ 0xfffbb038, "EAC_BPADMTR", "EAC Bluetooth port auxiliary data MSB transmit" },
	{ 0xfffbb03a, "EAC_BPADLRR", "EAC Bluetooth port auxiliary data LSB receive" },
	{ 0xfffbb03c, "EAC_BPADMRR", "EAC Bluetooth port auxiliary data MSB receive" },
	{ 0xfffbb040, "EAC_AMSCFR", "EAC Audio mixer switches configuration" },
	{ 0xfffbb042, "EAC_AMVCTR", "EAC Audio master volume control" },
	{ 0xfffbb044, "EAC_AM1VCTR", "EAC Audio mixer 1 volume control" },
	{ 0xfffbb046, "EAC_AM2VCTR", "EAC Audio mixer 2 volume control" },
	{ 0xfffbb048, "EAC_AM3VCTR", "EAC Audio mixer 3 volume control" },
	{ 0xfffbb04a, "EAC_ASTCTR", "EAC Audio side tone control" },
	{ 0xfffbb04c, "EAC_APD1LCR", "EAC Audio peak detector 1 left channel" },
	*/
/* High-Speed Access Bus Registers (0xfffec300)  */
	{ 0xfffec300, "HSAB_CLOCK_DIVISION", "HSAB Clock division" },
	{ 0xfffec308, "HSAB_ACCESS_TYPE", "HSAB Access type" },
/* Intersystem Communication Registers (0xfffbb800)  */
	/* TODO: Needs 16-bit register read/write support:
	{ 0xfffbb800, "ICR_M_ICR", "ICR MPU-S" },
	{ 0xfffbb802, "ICR_G_ICR", "ICR GSM-S" },
	{ 0xfffbb804, "ICR_M_CTL", "ICR MPU-S control" },
	{ 0xfffbb806, "ICR_G_CTL", "ICR GSM-S control" },
	{ 0xfffbb80a, "ICR_PM_BA", "ICR Program memory base address" },
	{ 0xfffbb80c, "ICR_DM_BA", "ICR Data memory base address" },
	{ 0xfffbb80e, "ICR_RM_BA", "ICR Random memory base address" },
	*/
/* Level 2 Interrupt Handler Registers (0xfffe0000)  */
	{ 0xfffe0004, "MPU_MIR", "MPU Mask interrupt" },
	{ 0xfffe0000, "MPU_ITR", "MPU Interrupt input" },
	{ 0xfffe0010, "MPU_SIR_IRQ_CODE", "MPU Binary-coded source IRQ" },
	{ 0xfffe0014, "MPU_SIR_FIQ_CODE", "MPU Binary-coded source FIQ" },
	{ 0xfffe0018, "MPU_CONTROL", "MPU Control register" },
	{ 0xfffe001c, "MPU_ILR_IRQ0_IL0", "MPU IRQ0 interrupt level 0" },
	{ 0xfffe0020, "MPU_ILR_IRQ0_IL1", "MPU IRQ0 interrupt level 1" },
	{ 0xfffe0024, "MPU_ILR_IRQ0_IL2", "MPU IRQ0 interrupt level 2" },
	{ 0xfffe0028, "MPU_ILR_IRQ0_IL3", "MPU IRQ0 interrupt level 3" },
	{ 0xfffe002c, "MPU_ILR_IRQ0_IL4", "MPU IRQ0 interrupt level 4" },
	{ 0xfffe0030, "MPU_ILR_IRQ0_IL5", "MPU IRQ0 interrupt level 5" },
	{ 0xfffe0034, "MPU_ILR_IRQ0_IL6", "MPU IRQ0 interrupt level 6" },
	{ 0xfffe0038, "MPU_ILR_IRQ0_IL7", "MPU IRQ0 interrupt level 7" },
	{ 0xfffe003c, "MPU_ILR_IRQ0_IL8", "MPU IRQ0 interrupt level 8" },
	{ 0xfffe0040, "MPU_ILR_IRQ0_IL9", "MPU IRQ0 interrupt level 9" },
	{ 0xfffe0044, "MPU_ILR_IRQ0_IL10", "MPU IRQ0 interrupt level 10" },
	{ 0xfffe0048, "MPU_ILR_IRQ0_IL11", "MPU IRQ0 interrupt level 11" },
	{ 0xfffe004c, "MPU_ILR_IRQ0_IL12", "MPU IRQ0 interrupt level 12" },
	{ 0xfffe0050, "MPU_ILR_IRQ0_IL13", "MPU IRQ0 interrupt level 13" },
	{ 0xfffe0054, "MPU_ILR_IRQ0_IL14", "MPU IRQ0 interrupt level 14" },
	{ 0xfffe0058, "MPU_ILR_IRQ0_IL15", "MPU IRQ0 interrupt level 15" },
	{ 0xfffe005c, "MPU_ILR_IRQ0_IL16", "MPU IRQ0 interrupt level 16" },
	{ 0xfffe0060, "MPU_ILR_IRQ0_IL17", "MPU IRQ0 interrupt level 17" },
	{ 0xfffe0064, "MPU_ILR_IRQ0_IL18", "MPU IRQ0 interrupt level 18" },
	{ 0xfffe0068, "MPU_ILR_IRQ0_IL19", "MPU IRQ0 interrupt level 19" },
	{ 0xfffe006c, "MPU_ILR_IRQ0_IL20", "MPU IRQ0 interrupt level 20" },
	{ 0xfffe0070, "MPU_ILR_IRQ0_IL21", "MPU IRQ0 interrupt level 21" },
	{ 0xfffe0074, "MPU_ILR_IRQ0_IL22", "MPU IRQ0 interrupt level 22" },
	{ 0xfffe0078, "MPU_ILR_IRQ0_IL23", "MPU IRQ0 interrupt level 23" },
	{ 0xfffe007c, "MPU_ILR_IRQ0_IL24", "MPU IRQ0 interrupt level 24" },
	{ 0xfffe0080, "MPU_ILR_IRQ0_IL25", "MPU IRQ0 interrupt level 25" },
	{ 0xfffe0084, "MPU_ILR_IRQ0_IL26", "MPU IRQ0 interrupt level 26" },
	{ 0xfffe0088, "MPU_ILR_IRQ0_IL27", "MPU IRQ0 interrupt level 27" },
	{ 0xfffe008c, "MPU_ILR_IRQ0_IL28", "MPU IRQ0 interrupt level 28" },
	{ 0xfffe0090, "MPU_ILR_IRQ0_IL29", "MPU IRQ0 interrupt level 29" },
	{ 0xfffe0094, "MPU_ILR_IRQ0_IL30", "MPU IRQ0 interrupt level 30" },
	{ 0xfffe0098, "MPU_ILR_IRQ0_IL31", "MPU IRQ0 interrupt level 31" },
	{ 0xfffe009c, "MPU_ISR", "MPU Interrupt set" },
/* LCD Controller Registers (0xfffec000)  */
	{ 0xfffec000, "MPU_LCDCONTROL", "MPU LCD control" },
	{ 0xfffec004, "MPU_LCDTIMING0", "MPU LCD timing 0" },
	{ 0xfffec008, "MPU_LCDTIMING1", "MPU LCD timing 1" },
	{ 0xfffec00c, "MPU_LCDTIMING2", "MPU LCD timing 2" },
	{ 0xfffec010, "MPU_LCDSTATUS", "MPU LCD status" },
	{ 0xfffec014, "MPU_LCDSUBPANEL", "MPU LCD subpanel" },
/* LED Pulse Generator Registers (0xfffba800)  */
	/* TODO: Needs 8-bit register read/write support:
	{ 0xfffba800, "TIPB_LPG_LCR", "TIPB LPG control" },
	{ 0xfffba801, "TIPB_LPG_PMR", "TIPB Power management" },
	*/
/* McBSP Registers (0xfffb1000)  */
	/* TODO: Needs 16-bit register read/write support:
	{ 0xfffb1000, "TI925T_MCBSP_DRR2_REG", "TI925T McBSP data receive 2" },
	{ 0xfffb1002, "TI925T_MCBSP_DRR1_REG", "TI925T McBSP data receive 1" },
	{ 0xfffb1004, "TI925T_MCBSP_DXR2_REG", "TI925T McBSP data transmit 2" },
	{ 0xfffb1006, "TI925T_MCBSP_DXR1_REG", "TI925T McBSP data transmit 1" },
	{ 0xfffb1008, "TI925T_MCBSP_SPCR2_REG", "TI925T McBSP serial port control 2" },
	{ 0xfffb100a, "TI925T_MCBSP_SPCR1_REG", "TI925T McBSP serial port control 1" },
	{ 0xfffb100c, "TI925T_MCBSP_RCR2_REG", "TI925T McBSP receive control 2" },
	{ 0xfffb100e, "TI925T_MCBSP_RCR1_REG", "TI925T McBSP receive control 1" },
	{ 0xfffb1010, "TI925T_MCBSP_XCR2_REG", "TI925T McBSP transmit control 2" },
	{ 0xfffb1012, "TI925T_MCBSP_XCR1_REG", "TI925T McBSP transmit control 1" },
	{ 0xfffb1014, "TI925T_MCBSP_SRGR2_REG", "TI925T McBSP sample rate generator 2" },
	{ 0xfffb1016, "TI925T_MCBSP_SRGR1_REG", "TI925T McBSP sample rate generator 1" },
	{ 0xfffb1018, "TI925T_MCBSP_MCR2_REG", "TI925T McBSP multichannel 2" },
	{ 0xfffb101a, "TI925T_MCBSP_MCR1_REG", "TI925T McBSP multichannel 1" },
	{ 0xfffb101c, "TI925T_MCBSP_RCERA_REG", "TI925T McBSP receive channel enable partition A" },
	{ 0xfffb101e, "TI925T_MCBSP_RCERB_REG", "TI925T McBSP receive channel enable partition B" },
	{ 0xfffb1020, "TI925T_MCBSP_XCERA_REG", "TI925T McBSP transmit channel enable partition A" },
	{ 0xfffb1022, "TI925T_MCBSP_XCERB_REG", "TI925T McBSP transmit channel enable partition B" },
	{ 0xfffb1024, "TI925T_MCBSP_PCR_REG", "TI925T McBSP pin control" },
	*/
/* Multimedia Card Adapter Registers (0xfffb7800)  */
	/* TODO:
	{ 0xfffb7800,	"MMC_CMD_REG",	"MMC command" },
	{ 0xfffb7802,	"MMC_ARG_REG_1",	"MMC argument 1" },
	{ 0xfffb7804,	"MMC_ARG_REG_2",	"MMC argument 2" },
	{ 0xfffb7806,	"FREQ_RATIO_REG",	"Frequency ratio" },
	{ 0xfffb7808,	"ADP_ST_REG",	"Status" },
	{ 0xfffb780a,	"MASK_IRQ_REG",	"Interrupt mask" },
	{ 0xfffb780c,	"RELEASE_REG",	"Release" },
	{ 0xfffb7810,	"BUF_DT_REG",	"Buffer data" },
	{ 0xfffb7812,	"BUF_CONF_REG",	"Buffer configuration" },
	{ 0xfffb7814,	"BUF_AF_REG",	"Buffer almost full level" },
	{ 0xfffb7816,	"BUF_AE_REG",	"Buffer almost empty level" },
	{ 0xfffb7820,	"RESP_REG1",	"Response 1" },
	{ 0xfffb7822,	"RESP_REG2",	"Response 2" },
	{ 0xfffb7824,	"RESP_REG3",	"Response 3" },
	{ 0xfffb7826,	"RESP_REG4",	"Response 4" },
	{ 0xfffb7828,	"RESP_REG5",	"Response 5" },
	{ 0xfffb782a,	"RESP_REG6",	"Response 6" },
	{ 0xfffb782c,	"RESP_REG7",	"Response 7" },
	{ 0xfffb782e,	"RESP_REG8",	"Response 8" },
	*/
/* MPU IO32_1 Registers (0xfffbc800)  */
	/* TODO:
	{ 0xfffbc800, "DATA_INPUT_REG",	"Data input" },
	{ 0xfffbc804, "DATA_OUTPUT_REG",	"Data output" },
	{ 0xfffbc808, "DIR_CTRL",	"Direction control" },
	{ 0xfffbc80c, "INTERRUPT_CTRL",	"Interrupt control" },
	{ 0xfffbc810, "INTERRUPT_MASK",	"Interrupt mask" },
	{ 0xfffbc814, "INTERRUPT_STATUS",	"Interrupt status" },
	*/
/* MPU IO32_2 Registers (0xfffbd000)  */
	/* TODO:
	{ 0xfffbd000, "DATA_INPUT_REG",	"Data input" },
	{ 0xfffbd004, "DATA_OUTPUT_REG",	"Data output" },
	{ 0xfffbd008, "DIR_CTRL",	"Direction control" },
	{ 0xfffbd00c, "INTERRUPT_CTRL",	"Interrupt control" },
	{ 0xfffbd010, "INTERRUPT_MASK",	"Interrupt mask" },
	{ 0xfffbd014, "INTERRUPT_STATUS",	"Interrupt status" },
	*/
/* MPU IO32_3 Registers (0xfffbd800)  */
	/* TODO
	{ 0xfffbd800, "DATA_INPUT_REG",	"Data input" },
	{ 0xfffbd804, "DATA_OUTPUT_REG",	"Data output" },
	{ 0xfffbd808, "DIR_CTRL",	"Direction control" },
	{ 0xfffbd80c, "INTERRUPT_CTRL",	"Interrupt control" },
	{ 0xfffbd810, "INTERRUPT_MASK",	"Interrupt mask" },
	{ 0xfffbd814, "INTERRUPT_STATUS",	"Interrupt status" },
	*/
/* MPU-S GPIO Registers (0xfffce000)  */
	/* TODO
	{ 0xfffce000, "DATA_INPUT_REG",	"Data input" },
	{ 0xfffce004, "DATA_OUTPUT_REG",	"Data output" },
	{ 0xfffce008, "DIR_CTRL",	"Direction control" },
	{ 0xfffce00c, "INTERRUPT_CTRL",	"Interrupt control" },
	{ 0xfffce010, "INTERRUPT_MASK",	"Interrupt mask" },
	{ 0xfffce014, "INTERRUPT_STATUS",	"Interrupt status" },
	*/
/* OMAP710 Configuration Registers (0xfffe1000)  */
	/* TODO
	{ 0xfffe1000, "PERSEUS_DEV_ID0",	"Device identification 0" },
	{ 0xfffe1002, "PERSEUS_DEV_ID1",	"Device identification 1" },
	{ 0xfffe1004, "GSM_EXT_LEAD_CONF",	"GSM external configuration" },
	{ 0xfffe1006, "GSM_EXT_ARM_CONF",	"GSM external MPU configuration" },
	{ 0xfffe1008, "GSM_ASIC_CONF",	"GSM ASIC configuration" },
	{ 0xfffe100a, "GSM_IO_CONF",	"GSM input/output configuration" },
	{ 0xfffe100c, "PERSEUS_OSC32K_GSM_CONF",	"32-kHz oscillator GSM configuration" },
	{ 0xfffe100e, "GSM_MCU_TRACE_SW",	"GSM MCU trace software" },
	{ 0xfffe1010, "PERSEUS_DIE_ID0",	"Die identification 0" },
	{ 0xfffe1012, "PERSEUS_DIE_ID1",	"Die identification 1" },
	{ 0xfffe1014, "PERSEUS_DIE_ID2",	"Die identification 2" },
	{ 0xfffe1016, "PERSEUS_DIE_ID3",	"Die Identification 3" },
	{ 0xfffe101c, "PERSEUS_MODE1",	"Mode 1 configuration" },
	{ 0xfffe101a, "PERSEUS_MODE2",	"Mode 2 configuration" },
	{ 0xfffe101e, "PERSEUS_IO_CONF0",	"Shared I/O configuration 0" },
	{ 0xfffe1020, "PERSEUS_IO_CONF1",	"Shared I/O configuration 1" },
	{ 0xfffe1022, "PERSEUS_IO_CONF2",	"Shared I/O configuration 2" },
	{ 0xfffe1024, "PERSEUS_IO_CONF3",	"Shared I/O configuration 3" },
	{ 0xfffe1026, "PERSEUS_IO_CONF4",	"Shared I/O configuration 4" },
	{ 0xfffe1028, "PERSEUS_IO_CONF5",	"Shared I/O configuration 5" },
	{ 0xfffe102a, "PERSEUS_IO_CONF6",	"Shared I/O configuration 6" },
	{ 0xfffe102c, "PERSEUS_IO_CONF7",	"Shared I/O configuration 7" },
	{ 0xfffe102e, "PERSEUS_IO_CONF8",	"Shared I/O configuration 8" },
	{ 0xfffe1030, "PERSEUS_IO_CONF9",	"Shared I/O configuration 9" },
	{ 0xfffe1032, "PERSEUS_AUDIO_CONF",	"Audio configuration" },
	{ 0xfffe1034, "TEST_ARM925MM_DPLL",	"DPLL test" },
	{ 0xfffe1036, "PERSEUS_OSC32K_MPU",	" 32-kHz oscillator MPU configuration" },
	{ 0xfffe1038, "FLASH_PROTECT",	"Flash protect" },
	*/
/* Real-Time Clock Registers (0xfffb4800)  */
	/* Support for these registers intentionally excluded. */
/* SPI_100KHZ Registers (0xfffbe800)  */
	/* TODO
	{ 0xfffbe800, "VSPI_SET1",	"VSPI configuration 1" },
	{ 0xfffbe802, "VSPI_SET2",	"VSPI configuration 2" },
	{ 0xfffbe804, "VSPI_CTRL",	"VSPI control" },
	{ 0xfffbe806, "VSPI_STATUS",	"VSPI status" },
	{ 0xfffbe808, "VSPI_TX_LSB",	"VSPI transmit data storage low" },
	{ 0xfffbe80a, "VSPI_TX_MSB",	"VSPI transmit data storage high" },
	{ 0xfffbe80c, "VSPI_RX_LSB",	"VSPI receive data storage low" },
	{ 0xfffbe80e, "VSPI_RX_MSB",	"VSPI receive data storage high" },
	*/
/* System DMA Registers (0xfffed800)  */
	/* TODO
	{ 0xfffed800, "DMA_CSDP_CHX",	"Channel source destination parameters" },
	{ 0xfffed802, "DMA_CCR_CHX",	"Channel control" },
	{ 0xfffed804, "DMA_CICR_CHX",	"Channel interrupt control" },
	{ 0xfffed806, "DMA_CSR_CHX",	"Channel status" },
	{ 0xfffed808, "DMA_CSSA_L_CHX",	"Channel source start address lower bits" },
	{ 0xfffed80a, "DMA_CSSA_U_CHX",	"Channel source start address upper bits" },
	{ 0xfffed80c, "DMA_CDSA_L_CHX",	"Channel destination start address lower bits" },
	{ 0xfffed80e, "DMA_CDSA_U_CHX",	"Channel destination start address upper bits" },
	{ 0xfffed810, "DMA_CEN_CHX",	"Channel element number" },
	{ 0xfffed812, "DMA_CFN_CHX",	"Channel frame number" },
	{ 0xfffed814, "DMA_CFI_CHX",	"Channel frame index" },
	{ 0xfffed816, "DMA_CEI_CHX",	"Channel element index" },
	{ 0xfffed818, "DMA_CPC_CHX",	"(DMA_CPC_CHX)" },
	{ 0xfffedb00, "DMA_LCD_CTRL",	"LCD control 0x300
	{ 0xfffedb02, "DMA_LCD_TOP_F1_L",	"LCD top address for frame buffer 1 lower bits 0x302
	{ 0xfffedb04, "DMA_LCD_TOP_F1_U",	"LCD top address for frame buffer 1 upper bits 0x304
	{ 0xfffedb06, "DMA_LCD_BOT_F1_L",	"LCD bottom address for frame buffer 1 lower bits 0x306
	{ 0xfffedb08, "DMA_LCD_BOT_F1_U",	"LCD bottom address for frame buffer 1 upper bits 0x308
	{ 0xfffedb0a, "DMA_LCD_TOP_F2_L",	"LCD top address for frame buffer 2 lower bits 0x30A
	{ 0xfffedb0c, "DMA_LCD_TOP_F2_U",	"LCD top address for frame buffer 2 upper bits 0x30C
	{ 0xfffedb0e, "DMA_LCD_BOT_F2_L",	"LCD bottom address for frame buffer 2 lower bits 0x30E
	{ 0xfffedb10, "DMA_LCD_BOT_F2_U",	"LCD bottom address for frame buffer 2 upper bits 0x310
	{ 0xfffedc00, "DMA_GCR",	"Global control 0x400
	*/
/* Traffic Controller Registers (0xfffecc00)  */
	{ 0xfffecc00, "TC_IMIF_PRIO", "TC IMIF priority" },
	{ 0xfffecc04, "TC_EMIFS_PRIO", "TC EMIF slow priority" },
	{ 0xfffecc08, "TC_EMIFF_PRIO", "TC EMIF fast priority" },
	{ 0xfffecc0c, "TC_EMIFS_GLB_CONFIG", "TC EMIF slow global configuration" },
	{ 0xfffecc10, "TC_EMIFS_CS0_CONFIG", "TC EMIF slow chip-select 0" },
	{ 0xfffecc14, "TC_EMIFS_CS1_CONFIG", "TC EMIF slow chip-select 1" },
	{ 0xfffecc18, "TC_EMIFS_CS2_CONFIG", "TC EMIF slow chip-select 2" },
	{ 0xfffecc1c, "TC_EMIFS_CS3_CONFIG", "TC EMIF slow chip-select 3" },
	{ 0xfffecc20, "TC_EMIFF_SDRAM_CONFIG", "TC EMIF fast SDRAM configuration" },
	{ 0xfffecc28, "TC_EMIFF_MRS", "TC EMIF fast SDRAM MRS 0x24 TIMEOUT2 Time-out 2" },
	{ 0xfffecc2c, "TC_TIMEOUT3", "TC Time-out 3" },
	{ 0xfffecc34, "TC_ENDIANISM", "TC DSP endianism configuration" },
	{ 0xfffecc3c, "TC_EMIFF_SDRAM_CONFIG2", "TC EMIF fast configuration 2" },
	{ 0xfffecc40, "TC_CONFIG_DYNAMIC_WAIT_STATE", "TC Waiting flash ready" },
/* Traffic Controller Interface Registers (0xfffea800 (GSM))  */
	/* TODO
	{ 0xfffea800, "TCIF_CTL",	"TCIF general control" },
	{ 0xfffea802, "PGM_CACHE_CTL",	"Program cache control" },
	{ 0xfffea804, "DATA_CACHE_CTL",	"Data cache control" },
	{ 0xfffea806, "RAND_BUFFER_CTL",	"Random buffer control" },
	{ 0xfffea810, "IT_DESCRIPTION",	"Interrupt description" },
	{ 0xfffea812, "IT_ADDRESS_L",	"Interrupt address LSB" },
	{ 0xfffea814, "IT_ADDRESS_H",	"Interrupt address MSB" },
	{ 0xfffea820, "COUNT_MNGT",	"Counter management" },
	{ 0xfffea830, "GSM_PGM_COUNT_L",	"GSM program counter LSB" },
	{ 0xfffea832, "GSM_PGM_COUNT_H",	"GSM program counter MSB" },
	{ 0xfffea834, "GSM_DATA_COUNT_L",	"GSM data counter LSB" },
	{ 0xfffea836, "GSM_DATA_COUNT_H",	"GSM data counter MSB" },
	{ 0xfffea838, "GSM_RAND_COUNT_L",	"GSM random counter LSB" },
	{ 0xfffea83a, "GSM_RAND_COUNT_H",	"GSM random counter MSB" },
	{ 0xfffea840, "MPU_PGM_COUNT_L",	"MPU program counter LSB" },
	{ 0xfffea842, "MPU_PGM_COUNT_H",	"MPU program counter MSB" },
	{ 0xfffea844, "MPU_DATA_COUNT_L",	"MPU data counter LSB" },
	{ 0xfffea846, "MPU_DATA_COUNT_H",	"MPU data counter MSB" },
	{ 0xfffea848, "MPU_RAND_COUNT_L",	"MPU random counter LSB" },
	{ 0xfffea84a, "MPU_RAND_COUNT_H",	"MPU random counter MSB" },
	*/
/* Timer_32K Registers (0xfffb9000)  */
	/* TODO
	{ 0xfffb9000, "TICK_VALUE_REG",	"Tick value" },
	{ 0xfffb9002, "TICK_COUNTER_REG",	"Tick counter" },
	{ 0xfffb9004, "TIMER_CTRL_REG",	"Timer control" },
	*/
/* UART IrDA Registers (0xfffb0800)  */
	/* Support for these registers intentionally excluded. */
/* UART Modem Registers (0xfffb0000)  */
	/* Support for these registers intentionally excluded. */
/* ULPD Registers (0xfffe0800  */
	/* TODO
	{ 0xfffe0800, "COUNTER_32_LSB_REG",	"Lower value of 32-kHz clocks" },
	{ 0xfffe0804, "COUNTER_32_MSB_REG",	"Upper value of 32-kHz clocks" },
	{ 0xfffe0808, "COUNTER_HI_FREQ_LSB_REG",	"Lower value of frequency clock" },
	{ 0xfffe080c, "COUNTER_HI_FREQ_MSB_REG",	"Upper value of frequency clock" },
	{ 0xfffe0810, "GAUGING_CTRL_REG",	"Gauging control" },
	{ 0xfffe0814, "GAUGING_STATUS_REG",	"Gauging status" },
	{ 0xfffe0824, "SETUP_ANALOG_CELL3",	" Number of 32-kHz periods to enable cell 3 register" },
	{ 0xfffe0830, "CLOCK_CTRL_REG",	"Clock output management" },
	{ 0xfffe0834, "SOFT_REQ_REG",	"Software clock request management" },
	{ 0xfffe0838, "COUNTER_32_FIQ_REG",	"Delay before modem shutdown" },
	{ 0xfffe083c, "DPLL_CTRL_REG",	"DPLL included in ULPD management" },
	{ 0xfffe0840, "STATUS_REQ_REG",	"Hardware reset status" },
	*/
/* USB Registers (0xfffb4000)  */
	/* TODO
	{ 0xfffb4000, "TXDAT",	"Transmit FIFO data" },
	{ 0xfffb4004, "TXSTATFLG",	"Transmit status" },
	{ 0xfffb4008, "TXCON1",	"Transmit control 1" },
	{ 0xfffb400c, "TXCON2",	"Transmit control 2" },
	{ 0xfffb4010, "RXDAT",	"Receive FIFO data" },
	{ 0xfffb4014, "RXSTATFLG",	"Receive status" },
	{ 0xfffb4018, "RXCON1",	"Receive control 1" },
	{ 0xfffb401c, "RXCON2",	"Receive control 2" },
	{ 0xfffb4020, "RXFSTAT",	"Receive FIFO status" },
	{ 0xfffb4024, "SYSCON1",	"System configuration 1" },
	{ 0xfffb4028, "SYSCON2",	"System configuration 2" },
	{ 0xfffb402c, "DEVSTAT",	"Device status" },
	{ 0xfffb4030, "SOF",	"Start of frame" },
	{ 0xfffb4040, "GENIE",	"General-purpose interrupt enable" },
	{ 0xfffb4044, "SBIE1",	"Non-ISO endpoint interrupt enable 1" },
	{ 0xfffb4048, "SBIE2",	"Non-ISO endpoint interrupt enable 2" },
	{ 0xfffb404c, "SOFIE",	"Start of frame interrupt enable" },
	{ 0xfffb4050, "GENI",	"General-purpose interrupt" },
	{ 0xfffb4054, "SBI1",	"Non-ISO endpoint interrupt 1" },
	{ 0xfffb4058, "SBI2",	"Non-ISO endpoint interrupt 2" },
	{ 0xfffb405c, "SOFI",	"Start of frame interrupt" },
	{ 0xfffb4080, "TXDCH0",	"Transmit FIFO data register DMA channel 0" },
	{ 0xfffb4084, "TXDCH1",	"Transmit FIFO data register DMA channel 1" },
	{ 0xfffb4088, "TXDMA1",	"Transmit DMA control 1" },
	{ 0xfffb408c, "TXDMA2",	"Transmit DMA control 2" },
	{ 0xfffb4090, "TXDMA3",	"Transmit DMA control 3" },
	{ 0xfffb40a0, "RXDCH0",	"Receive FIFO data register DMA channel 0" },
	{ 0xfffb40a4, "RXDCH1",	"Receive FIFO data register DMA channel 1" },
	{ 0xfffb40a8, "RXDMA1",	"Receive DMA control 1" },
	{ 0xfffb40ac, "RXDMA2",	"Receive DMA control 2" },
	{ 0xfffb40b0, "RXDMA3",	"Receive DMA control 3" },
	*/
/* Voice Serial Port Interface Registers (0xfffbc000)  */
	/* TODO
	{ 0xfffbc000, "VSPI_SET1",	"VSPI configuration 1" },
	{ 0xfffbc002, "VSPI_SET2",	"VSPI configuration 2" },
	{ 0xfffbc004, "VSPI_CTRL",	"VSPI control" },
	{ 0xfffbc006, "VSPI_STATUS",	"VSPI status" },
	{ 0xfffbc008, "VSPI_TX_LSB",	"VSPI transmit data storage low" },
	{ 0xfffbc00a, "VSPI_TX_MSB",	"VSPI transmit data storage high" },
	{ 0xfffbc00c, "VSPI_RX_LSB",	"VSPI receive data storage low" },
	{ 0xfffbc00e, "VSPI_RX_MSB",	"VSPI receive data storage high" },
	*/
};

#define NUM_OF_OMAP710_REG_ENTRY	(sizeof(omap710_regs)/sizeof(omap710_reg_entry_t))

static int
proc_read_reg(struct file *file, char *buf, size_t nbytes, loff_t * ppos)
{
	int i_ino = (file->f_dentry->d_inode)->i_ino;
	char outputbuf[15];
	int count;
	int i;
	omap710_reg_entry_t *current_reg = NULL;
	if (*ppos > 0)		/* Assume reading completed in previous read */
		return 0;
	for (i = 0; i < NUM_OF_OMAP710_REG_ENTRY; i++) {
		if (omap710_regs[i].low_ino == i_ino) {
			current_reg = &omap710_regs[i];
			break;
		}
	}
	if (current_reg == NULL)
		return -EINVAL;

	count = sprintf(outputbuf, "0x%08X\n",
			*((volatile unsigned int *)
			  io_p2v(current_reg->phyaddr)));
	*ppos += count;
	if (count > nbytes)	/* Assume output can be read at one time */
		return -EINVAL;
	if (copy_to_user(buf, outputbuf, count))
		return -EFAULT;
	return count;
}

static ssize_t
proc_write_reg(struct file *file, const char *buffer,
	       size_t count, loff_t * ppos)
{
	int i_ino = (file->f_dentry->d_inode)->i_ino;
	omap710_reg_entry_t *current_reg = NULL;
	int i;
	unsigned long newRegValue;
	char *endp;

	for (i = 0; i < NUM_OF_OMAP710_REG_ENTRY; i++) {
		if (omap710_regs[i].low_ino == i_ino) {
			current_reg = &omap710_regs[i];
			break;
		}
	}
	if (current_reg == NULL)
		return -EINVAL;

	newRegValue = simple_strtoul(buffer, &endp, 0);
	*((volatile unsigned int *) io_p2v(current_reg->phyaddr)) = newRegValue;
	return (count + endp - buffer);
}

static struct proc_dir_entry *regdir;
static struct proc_dir_entry *cpudir;

static int __init
init_reg_monitor(void)
{
	struct proc_dir_entry *entry;
	int i;

	cpudir = proc_mkdir(CPU_DIRNAME, &proc_root);
	if (cpudir == NULL) {
		printk(KERN_ERR MODULE_NAME ": can't create /proc/" CPU_DIRNAME
		       "\n");
		return (-ENOMEM);
	}

	regdir = proc_mkdir(REG_DIRNAME, cpudir);
	if (regdir == NULL) {
		printk(KERN_ERR MODULE_NAME ": can't create /proc/" CPU_DIRNAME
		       "/" REG_DIRNAME "\n");
		return (-ENOMEM);
	}

	for (i = 0; i < NUM_OF_OMAP710_REG_ENTRY; i++) {
		entry = create_proc_entry(omap710_regs[i].name,
					  S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH,
					  regdir);
		if (entry) {
			omap710_regs[i].low_ino = entry->low_ino;
			entry->proc_fops = &proc_reg_operations;
		} else {
			printk(KERN_ERR MODULE_NAME
			       ": can't create /proc/" REG_DIRNAME
			       "/%s\n", omap710_regs[i].name);
			return (-ENOMEM);
		}
	}
	return (0);
}

static void __exit
cleanup_reg_monitor(void)
{
	int i;
	for (i = 0; i < NUM_OF_OMAP710_REG_ENTRY; i++)
		remove_proc_entry(omap710_regs[i].name, regdir);
	remove_proc_entry(REG_DIRNAME, cpudir);
	remove_proc_entry(CPU_DIRNAME, &proc_root);
}

module_init(init_reg_monitor);
module_exit(cleanup_reg_monitor);

MODULE_AUTHOR("George Davis (gdavis@mvista.com)");
MODULE_DESCRIPTION("OMAP710 Register monitor");

EXPORT_NO_SYMBOLS;
