/******************************************************************************

	drivers/mtd/nand/mx2_nand.c
	driver for i.MX21 on-chip NAND Flash controller.
	tested with NAND Flash devices that come with MX21ADS board:
	SAMSUNG K9F1208Q0A (8bit i/o)
	SAMSUNG K9F5616Q0C (16Bit i/o)

	Copyright (c) 2004 MontaVista Software, Inc. <source@mvista.com>

	Based on mx2 nand support code from Motorola's MX21 BSP:

	Copyright (C) 2003 Motorola Inc. Ltd

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; either version 2
	of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
	

********************************************************************************/

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <asm/arch/gpio.h>
#include <asm/arch/pll.h>
#include <asm/arch/hardware.h>

static void mx2nand_setup_clock(void);

#if 1 /*CEE LDM*/

static char mx2nand_ldm_inited;

#include <linux/device.h>

extern void mx21_ldm_bus_register(struct device *device,
                          struct device_driver *driver);
extern void mx21_ldm_bus_unregister(struct device *device,
                          struct device_driver *driver);

static int mx2nand_freq_scale(struct bus_op_point * op, u32 level)
{
	mx2nand_setup_clock();
	return 0;
}

static struct device_driver __driver_ldm = {
	.name = "mx2_nand",
	.scale = mx2nand_freq_scale,
};

static struct device __device_ldm = {
	.name = "NAND FLASH",
	.bus_id = "mx2_nand",
	.driver = &__driver_ldm,
	.power_state = DPM_POWER_ON,
};

#endif

#define NAND_FLASH_CONFIG2_INT 0x8000
#define NAND_FLASH_CONFIG2_FCMD 0x1
#define NAND_FLASH_CONFIG2_FADD 0x2
#define NAND_FLASH_CONFIG2_FDI  0x4
#define NAND_FLASH_CONFIG2_PAGE_OUT  0x8
#define NAND_FLASH_CONFIG2_ID_OUT  0x10
#define NAND_FLASH_CONFIG2_STATUS_OUT  0x20

/*critical section*/
#define MX2NAND_DECLARE_CS_FLAGS(name) u32 name

#ifdef  CONFIG_MX2_16_BIT_NAND

#define MX2NAND_ENTER_CS(x) do { \
	save_flags(x); \
	cli(); \
	SYS_FMCR |= (1 << 4); \
} while (0)

#define MX2NAND_EXIT_CS(x) do { \
	SYS_FMCR &= ~(1 << 4); \
	restore_flags(x); \
} while (0)

#else

#define MX2NAND_ENTER_CS(x) do { \
	save_flags(x); \
	cli(); \
} while (0)

#define MX2NAND_EXIT_CS(x) do { \
	restore_flags(x); \
} while (0)

#endif

#ifdef CONFIG_MTD_CMDLINE_PARTS
const char *mx2nand_probes[] = { "cmdlinepart", NULL };
#endif

static struct mtd_info *mx2nand_mtd = NULL;

static u8 mx2nand_region_init;
static u8 mx2nand_gpio_init;

static void __init mx2nand_cleanup(void);

#define NAND_TIMEOUT HZ*5

struct mx2nand_priv {
	u_char mode;
	u_char cmd;
	u32 offset;
	u32 max_offset;
};

static uint8_t mx2nand_bi_pattern[] = { 0 };	/*bad block indicator pattern - I saw only 0's */

#ifdef  CONFIG_MX2_16_BIT_NAND
static struct nand_oobinfo mx2nand_oob = {
	.useecc = MTD_NANDECC_AUTOPLACE,
	.eccbytes = 0,		/* 0 instead of 3 to prevent writing attempts to ECC area */
	.eccpos = {6, 7, 8},	/*no practical value - SW ECC write is disabled, handled by HW */
	.oobfree = {{0, 6}, {12, 4}},	/*accessible bytes in the spare area */
};
static struct nand_bbt_descr mx2nand_bbt_descr = {
	.options = 0,
	.offs = 11,
	.len = 1,
	.pattern = mx2nand_bi_pattern,
};
#else
static struct nand_oobinfo mx2nand_oob = {
	.useecc = MTD_NANDECC_AUTOPLACE,
	.eccbytes = 0,		/* 0 instead of 3 to prevent writing attempts to ECC area */
	.eccpos = {6, 7, 8},	/*no practical value - SW ECC write is disabled, handled by HW */
	.oobfree = {{0, 5}, {11, 5}},	/*accessible bytes in the spare area */
};
static struct nand_bbt_descr mx2nand_bbt_descr = {
	.options = 0,
	.offs = 5,
	.len = 1,
	.pattern = mx2nand_bi_pattern,
};
#endif

static int
mx2nand_scan_bbt(struct mtd_info *mtd)
{
	nand_scan_bbt(mtd, &mx2nand_bbt_descr);
	return 0;
}

static int
mx2nand_correct_data(struct mtd_info *mtd, u_char * dat,
		     u_char * read_ecc, u_char * calc_ecc)
{
/*ecc is handled entirely by the MX2 NFC*/
/*report ECC Uncorrectable error*/
	if (NFC_ECC_STAT_RES & 0xA)
		return -1;
	else
		return 0;
}

static void
mx2nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
/*ecc is handled entirely by the MX2 NFC*/
}

static int
mx2nand_calculate_ecc(struct mtd_info *mtd, const u_char * dat,
		      u_char * ecc_code)
{
/*ecc is handled entirely by the MX2 NFC*/
	return 0;
}

static void
mx2nand_wait(void)
{
	unsigned long timeout;
	timeout = jiffies + NAND_TIMEOUT;
	while (!(NFC_NF_CONFIG2 & NAND_FLASH_CONFIG2_INT)) {
		if (time_after(jiffies, timeout)) {
			printk(KERN_ERR
			       "MX2_NAND: timeout waiting for Nand command complete\n");
			return;
		}
	}
}

static void
mx2nand_request_data(int cmd)
{
	MX2NAND_DECLARE_CS_FLAGS(flags);

	MX2NAND_ENTER_CS(flags);
	switch (cmd) {
	case NAND_CMD_READID:
		NFC_NF_CONFIG2 = NAND_FLASH_CONFIG2_ID_OUT;
		break;
	case NAND_CMD_STATUS:
		NFC_NF_CONFIG2 = NAND_FLASH_CONFIG2_STATUS_OUT;
		break;
	default:
		NFC_NF_CONFIG2 = NAND_FLASH_CONFIG2_PAGE_OUT;
	}
	mx2nand_wait();
	MX2NAND_EXIT_CS(flags);
}

static u_char
mx2nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip_priv = mtd->priv;
	struct mx2nand_priv *mx2_priv = chip_priv->priv;
	char b;
	u16 buf;
	u32 offset = mx2_priv->offset;
	int cmd = mx2_priv->cmd;

	if (offset == 0) {
		mx2nand_request_data(cmd);
	}

	if ((offset >= 512) || (cmd == NAND_CMD_READOOB))
		buf = *((u16 *) NFC_SAB_BASE(3) + (offset >> 1));
	else
		buf = *((u16 *) NFC_MAB_BASE(3) + (offset >> 1));

	b = ((u8 *) & buf)[offset & 0x1];

#ifdef CONFIG_MX2_16_BIT_NAND
	if (cmd == NAND_CMD_READID)
		offset++;	/*skip 1 byte */
#endif
	offset++;

	if (offset >= mx2_priv->max_offset)
		offset = 0;

	mx2_priv->offset = offset;

	return b;
}

static u16
mx2nand_read_word(struct mtd_info *mtd)
{
	u8 buf[2];

	buf[0] = mx2nand_read_byte(mtd);
	buf[1] = mx2nand_read_byte(mtd);

	return *(u16 *) buf;
}

static void
mx2nand_send_command(struct mtd_info *mtd, u8 cmd)
{
	struct nand_chip *chip_priv = mtd->priv;
	struct mx2nand_priv *mx2_priv = chip_priv->priv;
	MX2NAND_DECLARE_CS_FLAGS(flags);

	mx2_priv->cmd = cmd;	/*remember command for further
				   internal handling */

	if (cmd == NAND_CMD_PAGEPROG) {
		MX2NAND_ENTER_CS(flags);
		NFC_NF_CONFIG2 = NAND_FLASH_CONFIG2_FDI;
		mx2nand_wait();
		MX2NAND_EXIT_CS(flags);
	}

	/*send command */
	NFC_NAND_FLASH_CMD = cmd;
	MX2NAND_ENTER_CS(flags);
	NFC_NF_CONFIG2 = NAND_FLASH_CONFIG2_FCMD;
	if (cmd != NAND_CMD_RESET)
		mx2nand_wait();
	MX2NAND_EXIT_CS(flags);

	/*extra command processing for read/write-prepare commands */
	switch (cmd) {
	case NAND_CMD_READ0:
		NFC_NF_CONFIG1 = 0x8 | 0x2;	/*enable ECC, Spare + Main Area */
		mx2_priv->offset = 0;
		mx2_priv->max_offset = 528;
		break;
	case NAND_CMD_READOOB:
		NFC_NF_CONFIG1 = 0xC | 0x2;	/*enable ECC, Spare Area Only */
		mx2_priv->offset = 0;
		mx2_priv->max_offset = 16;
		break;
	case NAND_CMD_READID:
		NFC_NF_CONFIG1 = 0x8 | 0x2;	/*enable ECC, Spare + Main Area */
		mx2_priv->offset = 0;
#ifdef CONFIG_MX2_16_BIT_NAND
		mx2_priv->max_offset = 12;
#else
		mx2_priv->max_offset = 6;
#endif
		break;
	case NAND_CMD_STATUS:
		NFC_NF_CONFIG1 = 0x8 | 0x2;	/*enable ECC, Spare + Main Area */
		mx2_priv->offset = 0;
		mx2_priv->max_offset = 1;
		break;
	}
}

static void
mx2nand_send_address(u8 addr)
{
	MX2NAND_DECLARE_CS_FLAGS(flags);
	NFC_NAND_FLASH_ADDR = addr;
	MX2NAND_ENTER_CS(flags);
	NFC_NF_CONFIG2 = NAND_FLASH_CONFIG2_FADD;
	mx2nand_wait();
	MX2NAND_EXIT_CS(flags);
}

static void
mx2nand_write_byte(struct mtd_info *mtd, u_char byte)
{
	struct nand_chip *chip_priv = mtd->priv;
	struct mx2nand_priv *mx2_priv = chip_priv->priv;
	u32 offset;
	u16 buf;
	int cmd;

	switch (mx2_priv->mode) {
	case NAND_CTL_SETCLE:
		mx2nand_send_command(mtd, byte);
		break;
	case NAND_CTL_SETALE:
		mx2nand_send_address(byte);
		break;
	default:
		offset = mx2_priv->offset;
		cmd = mx2_priv->cmd;

		if ((offset >= 512) || (cmd == NAND_CMD_READOOB))
			buf = *((u16 *) NFC_SAB_BASE(3) + (offset >> 1));
		else
			buf = *((u16 *) NFC_MAB_BASE(3) + (offset >> 1));

		((u8 *) & buf)[offset & 0x1] = byte;

		if ((offset >= 512) || (cmd == NAND_CMD_READOOB))
			*((u16 *) NFC_SAB_BASE(3) + (offset >> 1)) = buf;
		else
			*((u16 *) NFC_MAB_BASE(3) + (offset >> 1)) = buf;

		offset++;
		if (offset >= mx2_priv->max_offset)
			offset = 0;
		mx2_priv->offset = offset;
	}
}

static void
mx2nand_write_word(struct mtd_info *mtd, u16 word)
{
	u8 *buf = (u8 *) & word;

	mx2nand_write_byte(mtd, buf[0]);
	mx2nand_write_byte(mtd, buf[1]);

}

static void
mx2nand_read_buf(struct mtd_info *mtd, u_char * buf, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		buf[i] = mx2nand_read_byte(mtd);
	}
}

static void
mx2nand_write_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		mx2nand_write_byte(mtd, buf[i]);
	}
}

static int
mx2nand_verify_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	int i;
	char b;
	for (i = 0; i < len; i++) {
		b = mx2nand_read_byte(mtd);
		if (buf[i] != b) {
			return -EFAULT;
		}
	}
	return 0;
}

static void
mx2nand_hwcontrol(struct mtd_info *mtd, int cmd)
{
	struct nand_chip *chip_priv = mtd->priv;
	struct mx2nand_priv *mx2_priv = chip_priv->priv;

	switch (cmd) {
	case NAND_CTL_SETNCE:	/* Enable NFC Clock */
		mx_module_clk_open(IPG_MODULE_NFC);
		mx2_priv->mode = 0;
		break;
	case NAND_CTL_CLRNCE:	/* Disable NFC Clock */
		mx_module_clk_close(IPG_MODULE_NFC);
		mx2_priv->mode = 0;
		break;
	case NAND_CTL_SETCLE:	/* Send Command mode start */
		mx2_priv->mode = cmd;
		NFC_CONFIGURATION = 0x0002;
		NFC_ULOCK_START_BLK = 0;
		NFC_ULOCK_END_BLK = 0xffff;	/*unlock all */
		NFC_NF_WR_PROT = 0x0004;
		NFC_RAM_BUF_ADDR = 0x3;
		break;
	case NAND_CTL_CLRCLE:	/* Send Command mode end */
		mx2_priv->mode = 0;
		break;
	case NAND_CTL_SETALE:	/* Send address mode start */
		mx2_priv->mode = cmd;
		break;
	case NAND_CTL_CLRALE:	/* Send address mode end */
		mx2_priv->mode = 0;
		break;
	default:
		mx2_priv->mode = 0;
	}
}

const static struct mtd_partition mx2nand_partition_info[] = {

      {name:"ipl",
	      offset:0,
      size:0x04000},

      {name:"spl",
	      offset:MTDPART_OFS_APPEND,
      size:0x100000},

      {name:"kernel",
	      offset:MTDPART_OFS_APPEND,
      size:0x200000},

      {name:"RootDisk",
	      offset:MTDPART_OFS_APPEND,
      size:MTDPART_SIZ_FULL}

};

#define MX2_NAND_PARTITIONS_NUM 4

/*NAND pinout:
NFCE_B   T15  PF1
NFCLE    U15  PF3
NFALE    U14  PF4
NFWE_B   T13  PF6
NFRE_B   W13  PF5
NFWP_B   L12  PF2
NFRB     M12  PF0
NFIO0..7  W12, U13, L11, T12, U12, T11, V13, M11  PF7..PF14
A13..A15, A21..A25  F3, F1, G3, G1, H3, L2, L1, M2

Note: PF0..PF14 port mux mode is PRIMARY
*/

#define MX2_NAND_GPIO_PORTF_MASK 0x7fff

#define MX2_NFCCLK 25000000 /*max Nand Flash Controller clock freq in Hz*/

static void
mx2nand_setup_clock(void)
{
u32 tmpclock;
u32 maxclock;

	/*NFC clock may not be slower that HCLK ?*/
	maxclock = mx_module_get_clk(HCLK);
	if (maxclock > MX2_NFCCLK)
		maxclock = MX2_NFCCLK;
	
	/*there is no lower boundary limit. Upper boundary is checked in mx2nand_validate_clock*/
	tmpclock = mx_module_get_clk(FCLK);

	tmpclock = tmpclock/maxclock + ((tmpclock%maxclock > ((u32)maxclock >> 1)) ? 1 : 0);

	if (tmpclock) {
		tmpclock--; /*slow clock OK*/
	}
	
#ifdef CONFIG_MX2TO1
	CRM_PCDR = (CRM_PCDR & ~PCDR_NFCDIV) | (tmpclock << PCDR_NFCDIV_SHIFT);	/*clock control */
#else
	CRM_PCDR0 = (CRM_PCDR0 & ~PCDR0_NFCDIV) | (tmpclock << PCDR0_NFCDIV_SHIFT);	/*clock control */
#endif


}

static inline int __init
mx2nand_validate_clock(void)
{
u32 tmpclock;

/* although  NFC DIVISOR gets clock form FCLK, MPLL clock is selected as a base for calculations,
  assuming the worst-case (and typical) scenario of PRESC Divisor = 0.
  It is hard to impelement suspend/resume for this driver
  - the only way to stop MTD from accessing it would be to destroy partitions, what may be
  confusing to the user
  On the other hand, even with the limitation of the above assumption, the maximal MPLL frequency
  allowed is 512 MHz, which is enough to cover the known 350 and 266 MHz options*/

	tmpclock = mx_module_get_clk(MPLL);

	tmpclock = tmpclock/MX2_NFCCLK + ((tmpclock%MX2_NFCCLK > ((u32)MX2_NFCCLK >> 1)) ? 1 : 0);

	if (tmpclock) {
		tmpclock--; /*slow clock OK*/
	}


#ifdef CONFIG_MX2TO1
	if (tmpclock > ((u32)PCDR_NFCDIV >> PCDR_NFCDIV_SHIFT)) {
		printk(KERN_ERR "MX2 NAND: FCLK too fast, cannot operate\n");
		return -1; /*FCLK is too fast*/
	}
#else
	if (tmpclock > ((u32)PCDR0_NFCDIV >> PCDR0_NFCDIV_SHIFT)) {
		printk(KERN_ERR "MX2 NAND: FCLK too fast, cannot operate\n");
		return -1; /*FCLK is too fast*/
	}
#endif

	return 0;
}


/**
 * Module initialization routine
 */
int __init
mx2nand_init(void)
{
	struct nand_chip *this;
	int mtd_parts_nb = 0;
	int tmp;

#ifdef CONFIG_MX2_16_BIT_NAND
	printk(KERN_INFO "MX2 NAND: 16-bit i/o mode\n");
#else
	printk(KERN_INFO "MX2 NAND: 8-bit i/o mode\n");
#endif

	if (!(request_region(NFC_BASE, NFC_IO_SIZE, "mx2nand"))) {
		printk(KERN_ERR "MX2 NAND: NFC region is already in use \n");
		mx2nand_cleanup();
		return -1;
	}
	mx2nand_region_init = 1;

	SYS_FMCR &= ~(1 << 4);

	if (mx2nand_validate_clock() < 0) {
		mx2nand_cleanup();
		return -EFAULT;
	}

	mx2nand_setup_clock();

	tmp = mx2_register_gpios(PORT_F, MX2_NAND_GPIO_PORTF_MASK, PRIMARY);
	if (tmp < 0) {
		printk(KERN_ERR
		       "MX2 NAND: Unable to request GPIO pins: PORT F, mask 0x%x\n",
		       MX2_NAND_GPIO_PORTF_MASK);
		mx2nand_cleanup();
		return tmp;
	}
	mx2nand_gpio_init = 1;

	/* Allocate memory for MTD device structure and private data */
	mx2nand_mtd =
	    kmalloc(sizeof (struct mtd_info) + sizeof (struct nand_chip) +
		    sizeof (struct mx2nand_priv), GFP_KERNEL);
	if (!mx2nand_mtd) {
		printk(KERN_ERR
		       "MX2 NAND: Not enough memory for a MTD device structure\n");
		mx2nand_cleanup();
		return -ENOMEM;
	}

	/* Initialize structures */
	memset((char *) mx2nand_mtd, 0,
	       sizeof (struct mtd_info) + sizeof (struct nand_chip) +
	       sizeof (struct mx2nand_priv));

	/* Get pointer to private data */
	this =
	    (struct nand_chip *) ((char *) mx2nand_mtd +
				  sizeof (struct mtd_info));

	/* Link the private data with the MTD structure */
	mx2nand_mtd->priv = this;

	this->hwcontrol = mx2nand_hwcontrol;
	this->read_byte = mx2nand_read_byte;
	this->write_byte = mx2nand_write_byte;	/*used to send out a command or address */
	this->read_word = mx2nand_read_word;	/*used once at initialization 16-bit */
	this->write_word = mx2nand_write_word;	/*never used */
	this->read_buf = mx2nand_read_buf;
	this->write_buf = mx2nand_write_buf;
	this->verify_buf = mx2nand_verify_buf;

	/* ECC handling */
	this->autooob = &mx2nand_oob;
	this->enable_hwecc = mx2nand_enable_hwecc;
	this->calculate_ecc = mx2nand_calculate_ecc;
	this->eccmode = NAND_ECC_HW3_512;
	this->correct_data = mx2nand_correct_data;
	/* end of ECC handling */

	/* BBT handling */
	this->scan_bbt = mx2nand_scan_bbt;

#ifdef CONFIG_MX2_16_BIT_NAND
	this->options |= NAND_BUSWIDTH_16;
#endif
	this->priv =
	    (struct mx2nand_priv *) ((char *) mx2nand_mtd +
				     sizeof (struct mtd_info) +
				     sizeof (struct nand_chip));

	/* Scan to find existence of the device */
	tmp = nand_scan(mx2nand_mtd, 1);
	if (tmp) {
		printk(KERN_ERR "MX2 NAND: device not found\n");
		mx2nand_cleanup();
		return -ENXIO;
	}

	if ((mx2nand_mtd->oobsize != 16) || (mx2nand_mtd->oobblock != 512)) {
		printk(KERN_ERR
		       "MX2 NAND: Unsupported NAND Flash configuration\n");
		mx2nand_cleanup();
		return -EFAULT;
	}

	/* Register the partitions */
#ifdef CONFIG_MTD_PARTITIONS
#ifdef CONFIG_MTD_CMDLINE_PARTS
	{
		struct mtd_partition *mtd_parts;
		mx2nand_mtd->name = "mx2-nand";
		mtd_parts_nb =
		    parse_mtd_partitions(mx2nand_mtd, mx2nand_probes,
					 &mtd_parts, 0);
		if (mtd_parts_nb > 0)
			tmp =
			    add_mtd_partitions(mx2nand_mtd, mtd_parts,
					       mtd_parts_nb);
	}
#endif				/* end of CONFIG_MTD_CMDLINE_PARTS */
	if (mtd_parts_nb <= 0) {
		tmp = add_mtd_partitions(mx2nand_mtd, mx2nand_partition_info,
					 MX2_NAND_PARTITIONS_NUM);
	}
	if (tmp < 0) {
		printk(KERN_ERR "MX2 NAND: Error registering MTD partitions\n");
		mx2nand_cleanup();
		return tmp;
	}
#else				/* if !CONFIG_MTD_PARTITIONS */
	tmp = add_mtd_device(mx2nand_mtd);
	if (tmp < 0) {
		printk(KERN_ERR "MX2 NAND: Error registering MTD device\n");
		mx2nand_cleanup();
		return tmp;
	}
#endif				/* end of CONFIG_MTD_PARTITIONS */

#if 1 /*CEE LDM*/
	mx21_ldm_bus_register(&__device_ldm, &__driver_ldm);
	mx2nand_ldm_inited = 1;
#endif

	/* Return happy */
	return 0;
}

/**
 * Module clean up routine
 */
static void __init
mx2nand_cleanup(void)
{

#if 1 /*CEE LDM*/
	if (mx2nand_ldm_inited)
		mx21_ldm_bus_unregister(&__device_ldm, &__driver_ldm);
#endif

#ifdef CONFIG_MTD_PARTITIONS
	/* Clean up partitions */
	del_mtd_partitions(mx2nand_mtd);
#else
	/* Unregister the device */
	del_mtd_device(mx2nand_mtd);
#endif

	/* Free the MTD device structure */
	kfree(mx2nand_mtd);

	if (mx2nand_gpio_init) {
		mx2_unregister_gpios(PORT_F, MX2_NAND_GPIO_PORTF_MASK);
	}

	if (mx2nand_region_init) {
		release_region(NFC_BASE, NFC_IO_SIZE);
	}

}

module_init(mx2nand_init);
module_exit(mx2nand_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("MontaVista Software, Inc");
MODULE_DESCRIPTION("MTD Driver for MX2 Nand Flash Controller");
