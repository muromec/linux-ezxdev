/*
 * IDE driver for IBM On-chip IDE contollers 
 *    Copyright 2001 - 2002 MontaVista Software Inc.
 *    Dan Malek.
 *
 *    I snagged bits and pieces from a variety of drives, primarily
 *    ide-pmac.c.....thanks to previous authors!
 *
 *    Version 1.2 (01/30/12) Armin
 *    Converted to ocp
 *    merger up to new ide-timing.h
 *
 *    Version 2.0 (05/02/15) - armin
 *    converted to new core_ocp and only supports one interface for now.
 *
 *    Version 2.1 (05/25/02) - armin
 *      name change from *_driver to *_dev
 *    Version 2.2 06/13/02 - Armin
 *      changed irq_resource array to just irq
 *
 */

#include <linux/types.h>
#include <linux/hdreg.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/pci.h>
#include <asm/ocp.h>
#include <asm/io.h>
#include <asm/scatterlist.h>

#include "ide-timing.h"
#define OCPVR	"2.2"

/* The structure of the PRD entry.  The address must be word aligned,
 * and the count must be an even number of bytes.
 */
typedef struct {
	unsigned int prd_physptr;
	unsigned int prd_count;	/* Count only in lower 16 bits */
} prd_entry_t;
#define PRD_EOT		(uint)0x80000000	/* Set in prd_count */

/* The number of PRDs required in a single transfer from the upper IDE
 * functions.  I believe the maximum number is 128, but most seem to
 * code to 256.  It's probably best to keep this under one page......
 */
#define NUM_PRD	256

#define MK_TIMING(AS, DIOP, DIOY, DH) \
	((FIT((AS),    0, 15) << 27) | \
	 (FIT((DIOP),  0, 63) << 20) | \
	 (FIT((DIOY),  0, 63) << 13) | \
	 (FIT((DH),    0,  7) << 9))

#define UTIMING_SETHLD	(EZ(20 /*tACK*/, SYS_CLOCK_NS) - 1 /*fixed cycles*/)
#define UTIMING_ENV	(EZ(20 /*tENV*/, SYS_CLOCK_NS) - 1 /*fixed cycles*/)
#define UTIMING_SS	(EZ(50 /*tSS */, SYS_CLOCK_NS) - 3 /*fixed cycles*/)
#define MK_UTIMING(CYC, RP) \
	((FIT(UTIMING_SETHLD, 0, 15) << 27) | \
	 (FIT(UTIMING_ENV,    0, 15) << 22) | \
	 (FIT((CYC),          0, 15) << 17) | \
	 (FIT((RP),           0, 63) << 10) | \
	 (FIT(UTIMING_SS,     0, 15) << 5)  | \
	 1 /* Turn on Ultra DMA */)

/* Define the period of the STB clock used to generate the
 * IDE bus timing.  The clock is actually 63 MHz, but it
 * get rounded in a favorable direction.
 */
#define IDE_SYS_FREQ	63	/* MHz */
#define SYS_CLOCK_NS	(1000 / IDE_SYS_FREQ)

struct whold_timing {
	short mode;
	short whold;
};

static struct whold_timing whold_timing[] = {

	{XFER_UDMA_5, 0},
	{XFER_UDMA_4, 0},
	{XFER_UDMA_3, 0},

	{XFER_UDMA_2, 0},
	{XFER_UDMA_1, 0},
	{XFER_UDMA_0, 0},

	{XFER_UDMA_SLOW, 0},

	{XFER_MW_DMA_2, 0},
	{XFER_MW_DMA_1, 0},
	{XFER_MW_DMA_0, 0},

	{XFER_SW_DMA_2, 0},
	{XFER_SW_DMA_1, 0},
	{XFER_SW_DMA_0, 10},

	{XFER_PIO_5, 10},
	{XFER_PIO_4, 10},
	{XFER_PIO_3, 15},

	{XFER_PIO_2, 20},
	{XFER_PIO_1, 30},
	{XFER_PIO_0, 50},

	{XFER_PIO_SLOW,},

	{-1}
};

/* The interface doesn't have register/PIO timing for each device,
 * but rather "fast" and "slow" timing.  We have to determeine
 * which is the "fast" device based upon their capability.
 */
static int pio_mode[2];

/* Pointer to the IDE controller registers.
*/
static volatile ide_t *idp;

/* Virtual and physical address of the PRD page.
*/
static prd_entry_t *prd_table;
static dma_addr_t prd_phys;

/* Some prototypes we need.
*/
extern char *ide_dmafunc_verbose(ide_dma_action_t dmafunc);

int
nonpci_ide_default_irq(ide_ioreg_t base)
{
	return IDE0_IRQ;
}

/* this iis barrowed from ide_timing_find_mode so we can find the proper 
 * whold parameter 
 */

static short
whold_timing_find_mode(short speed)
{
	struct whold_timing *t;

	for (t = whold_timing; t->mode != speed; t++)
		if (t->mode < 0)
			return 0;
	return t->whold;
}

/* The STB04 has a fixed number of cycles that get added in
 * regardless.  Adjust an ide_timing struct to accommodate that.
 */
static
    void
stb04xxx_ide_adjust_timing(struct ide_timing *t)
{
	t->setup -= 2;
	t->act8b -= 1;
	t->rec8b -= 1;
	t->active -= 1;
	t->recover -= 1;
}

static int
stb04xxx_ide_set_drive(ide_drive_t * drive, unsigned char speed)
{
	ide_drive_t *peer;
	struct ide_timing d, p, merge, *fast;
	int fast_device;
	unsigned int ctl;
	volatile unsigned int *dtiming;

	if (speed != XFER_PIO_SLOW && speed != drive->current_speed)
		if (ide_config_drive_speed(drive, speed))
			printk(KERN_WARNING
			       "ide%d: Drive %d didn't accept speed setting. Oh, well.\n",
			       drive->dn >> 1, drive->dn & 1);

	ide_timing_compute(drive, speed, &d, SYS_CLOCK_NS, SYS_CLOCK_NS);
	stb04xxx_ide_adjust_timing(&d);

	/* This should be set somewhere else, but it isn't.....
	 */
	drive->dn = ((drive->select.all & 0x10) != 0);
	peer = HWIF(drive)->drives + (~drive->dn & 1);

	if (peer->present) {
		ide_timing_compute(peer, peer->current_speed, &p,
				   SYS_CLOCK_NS, SYS_CLOCK_NS);
		stb04xxx_ide_adjust_timing(&p);
		ide_timing_merge(&p, &d, &merge,
				 IDE_TIMING_8BIT | IDE_TIMING_SETUP);
	} else {
		merge = d;
	}

	if (!drive->init_speed)
		drive->init_speed = speed;
	drive->current_speed = speed;

	/* Now determine which drive is faster, and set up the
	 * interface timing.  It would sure be nice if they would
	 * have just had the timing registers for each device......
	 */
	if (drive->dn & 1)
		pio_mode[1] = (int) speed;
	else
		pio_mode[0] = (int) speed;

	if (pio_mode[0] > pio_mode[1])
		fast_device = 0;
	else
		fast_device = 1;

	/* Now determine which of the drives
	 * the first call we only know one device, and on subsequent
	 * calls the user may manually change drive parameters.
	 * Make timing[0] the fast device and timing[1] the slow.
	 */
	if (fast_device == (drive->dn & 1))
		fast = &d;
	else
		fast = &p;

	/* Now we know which device is the fast one and which is
	 * the slow one.  The merged timing goes into the "regular"
	 * timing registers and represents the slower of both times.
	 */

	idp->si_c0rt = MK_TIMING(merge.setup, merge.act8b,
				 merge.rec8b,
				 whold_timing_find_mode(merge.mode));

	idp->si_c0fpt = MK_TIMING(fast->setup, fast->act8b,
				  fast->rec8b,
				  whold_timing_find_mode(fast->mode));

	/* Tell the interface which drive is the fast one.
	 */
	ctl = idp->si_c0c;	/* Chan 0 Control */
	ctl &= ~0x10000000;
	ctl |= fast_device << 28;
	idp->si_c0c = ctl;

	/* Set up DMA timing.
	 */
	if ((speed & XFER_MODE) != XFER_PIO) {
		/* NOTE: si_c0d0m and si_c0d0u are two different names
		 * for the same register.  Whether it is used for
		 * Multi-word DMA timings or Ultra DMA timings is
		 * determined by the LSB written into it.  This is also
		 * true for si_c0d1m and si_c0d1u.  */
		if (drive->dn & 1)
			dtiming = &(idp->si_c0d1m);
		else
			dtiming = &(idp->si_c0d0m);

		if ((speed & XFER_MODE) == XFER_UDMA) {
			static const int tRP[] = {
				EZ(160, SYS_CLOCK_NS) - 2 /*fixed cycles */ ,
				EZ(125, SYS_CLOCK_NS) - 2 /*fixed cycles */ ,
				EZ(100, SYS_CLOCK_NS) - 2 /*fixed cycles */ ,
				EZ(100, SYS_CLOCK_NS) - 2 /*fixed cycles */ ,
				EZ(100, SYS_CLOCK_NS) - 2 /*fixed cycles */ ,
				EZ(85, SYS_CLOCK_NS) - 2	/*fixed cycles */
			};
			static const int NUMtRP =
			    (sizeof (tRP) / sizeof (tRP[0]));
			*dtiming =
			    MK_UTIMING(d.udma,
				       tRP[FIT(speed & 0xf, 0, NUMtRP - 1)]);
		} else {
			/* Multi-word DMA.  Note that d.recover/2 is an
			 * approximation of MAX(tH, MAX(tJ, tN)) */
			*dtiming = MK_TIMING(d.setup, d.active,
					     d.recover, d.recover / 2);
		}
		drive->using_dma = 1;
	}

	return 0;
}

static void
stb04xxx_ide_tuneproc(ide_drive_t * drive, unsigned char pio)
{

	if (pio == 255)
		pio = ide_find_best_mode(drive, XFER_PIO | XFER_EPIO);
	else
		pio = XFER_PIO_0 + MIN(pio, 5);

	stb04xxx_ide_set_drive(drive, pio);
}

#ifdef CONFIG_BLK_DEV_IDEDMA

/* DMA stuff mostly stolen from PMac....thanks Ben :-)!
*/

static int
stb04xxx_ide_build_dmatable(ide_drive_t * drive, int wr)
{
	prd_entry_t *table;
	int count = 0;
	struct request *rq = HWGROUP(drive)->rq;
	struct buffer_head *bh = rq->bh;
	unsigned int size, addr;

	table = prd_table;

	do {
		/*
		 * Determine addr and size of next buffer area.  We assume that
		 * individual virtual buffers are always composed linearly in
		 * physical memory.  For example, we assume that any 8kB buffer
		 * is always composed of two adjacent physical 4kB pages rather
		 * than two possibly non-adjacent physical 4kB pages.
		 * We also have to ensure cache coherency here.  If writing,
		 * flush the data cache to memory.  Logically, if reading
		 * we should do it after the DMA is complete, but it is
		 * more convenient to do it here.  If someone is messing
		 * with a buffer space after it is handed to us, they
		 * shouldn't be surprised by corrupted data, anyway :-).
		 */
		if (bh == NULL) {	/* paging requests have (rq->bh == NULL) */
			addr = virt_to_bus(rq->buffer);
			size = rq->nr_sectors << 9;
			if (wr)
				consistent_sync(rq->buffer, size,
						PCI_DMA_TODEVICE);
			else
				consistent_sync(rq->buffer, size,
						PCI_DMA_FROMDEVICE);
		} else {
			/* group sequential buffers into one large buffer */
			addr = virt_to_bus(bh->b_data);
			size = bh->b_size;
			if (wr)
				consistent_sync(bh->b_data, size,
						PCI_DMA_TODEVICE);
			else
				consistent_sync(bh->b_data, size,
						PCI_DMA_FROMDEVICE);
			while ((bh = bh->b_reqnext) != NULL) {
				if ((addr + size) != virt_to_bus(bh->b_data))
					break;
				size += bh->b_size;
				if (wr)
					consistent_sync(bh->b_data,
							bh->b_size,
							PCI_DMA_TODEVICE);
				else
					consistent_sync(bh->b_data,
							bh->b_size,
							PCI_DMA_FROMDEVICE);
			}
		}

		/*
		 * Fill in the next PRD entry.
		 * Note that one PRD entry can transfer
		 * at most 65535 bytes.
		 */
		while (size) {
			unsigned int tc = (size < 0xfe00) ? size : 0xfe00;

			if (++count >= NUM_PRD) {
				printk(KERN_WARNING "%s: DMA table too small\n",
				       drive->name);
				return 0;	/* revert to PIO for this request */
			}
			table->prd_physptr = (addr & 0xfffffffe);
			if (table->prd_physptr & 0xF) {
				printk(KERN_WARNING
				       "%s: DMA buffer not 16 byte aligned.\n",
				       drive->name);
				return 0;	/* revert to PIO for this request */
			}
			table->prd_count = (tc & 0xfffe);
			addr += tc;
			size -= tc;
			++table;
		}
	} while (bh != NULL);

	/* Add the EOT to the last table entry.
	 */
	if (count) {
		table--;
		table->prd_count |= PRD_EOT;
	} else {
		printk(KERN_DEBUG "%s: empty DMA table?\n", drive->name);
	}

	return 1;
}

/*
 * dma_intr() is the handler for disk read/write DMA interrupts
 * This is taken directly from ide-dma.c, which we can't use because
 * it requires PCI support.
 */
ide_startstop_t
ide_dma_intr(ide_drive_t * drive)
{
	int i;
	byte stat, dma_stat;

	dma_stat = HWIF(drive)->dmaproc(ide_dma_end, drive);
	stat = GET_STAT();	/* get drive status */
	if (OK_STAT(stat, DRIVE_READY, drive->bad_wstat | DRQ_STAT)) {
		if (!dma_stat) {
			struct request *rq = HWGROUP(drive)->rq;
			rq = HWGROUP(drive)->rq;
			for (i = rq->nr_sectors; i > 0;) {
				i -= rq->current_nr_sectors;
				ide_end_request(1, HWGROUP(drive));
			}
			return ide_stopped;
		}
		printk("%s: dma_intr: bad DMA status (dma_stat=%x)\n",
		       drive->name, dma_stat);
	}
	return ide_error(drive, "dma_intr", stat);
}

/* ....and another one....
*/
int
report_drive_dmaing(ide_drive_t * drive)
{
	struct hd_driveid *id = drive->id;

	if ((id->field_valid & 4) && (eighty_ninty_three(drive)) &&
	    (id->dma_ultra & (id->dma_ultra >> 11) & 7)) {
		if ((id->dma_ultra >> 13) & 1) {
			printk(", UDMA(100)");	/* UDMA BIOS-enabled! */
		} else if ((id->dma_ultra >> 12) & 1) {
			printk(", UDMA(66)");	/* UDMA BIOS-enabled! */
		} else {
			printk(", UDMA(44)");	/* UDMA BIOS-enabled! */
		}
	} else if ((id->field_valid & 4) &&
		   (id->dma_ultra & (id->dma_ultra >> 8) & 7)) {
		if ((id->dma_ultra >> 10) & 1) {
			printk(", UDMA(33)");	/* UDMA BIOS-enabled! */
		} else if ((id->dma_ultra >> 9) & 1) {
			printk(", UDMA(25)");	/* UDMA BIOS-enabled! */
		} else {
			printk(", UDMA(16)");	/* UDMA BIOS-enabled! */
		}
	} else if (id->field_valid & 4) {
		printk(", (U)DMA");	/* Can be BIOS-enabled! */
	} else {
		printk(", DMA");
	}
	return 1;
}

static int
stb04xxx_ide_check_dma(ide_drive_t * drive)
{
#if 0				/* Code using this is ifdef'ed out */
	struct hd_driveid *id = drive->id;
#endif
	int enable = 1;
	int speed;

	drive->using_dma = 0;

	if (drive->media == ide_floppy)
		enable = 0;

	/* Check timing here, we may be able to include XFER_UDMA_66
	 * and XFER_UDMA_100.  This basically tells the 'best_mode'
	 * function to also consider UDMA3 to UDMA5 device timing.
	 */
	if (enable) {
		/* Section 1.6.2.6 "IDE Controller, ATA/ATAPI-5" in the STB04xxx
		 * Datasheet says the following modes are supported:
		 *   PIO modes 0 to 4
		 *   Multiword DMA modes 0 to 2
		 *   UltraDMA modes 0 to 4
		 */
		int map = XFER_PIO | XFER_EPIO | XFER_MWDMA | XFER_UDMA;
		/* XFER_EPIO includes both PIO modes 4 and 5.  Mode 5 is not
		 * valid for the STB04, so mask it out of consideration just
		 * in case some drive sets it...
		 */
		id->eide_pio_modes &= ~4;

		/* Allow UDMA_66 only if an 80 conductor cable is connected. */
		if (eighty_ninty_three(drive))
			map |= XFER_UDMA_66;

		speed = ide_find_best_mode(drive, map);
		stb04xxx_ide_set_drive(drive, speed);

		if (HWIF(drive)->autodma &&
		    (((speed & XFER_MODE) == XFER_PIO) ||
		     ((speed & XFER_MODE) == XFER_EPIO))) {
			drive->using_dma = 0;
		}
	}

	return 0;
}

static int
stb04xxx_ide_dmaproc(ide_dma_action_t func, ide_drive_t * drive)
{
	unsigned int dstat;

	switch (func) {
	case ide_dma_off:
		printk(KERN_INFO "%s: DMA disabled\n", drive->name);
	case ide_dma_off_quietly:
		drive->using_dma = 0;
		break;
	case ide_dma_on:
	case ide_dma_check:
		stb04xxx_ide_check_dma(drive);
		break;
	case ide_dma_read:
	case ide_dma_write:
		if (!stb04xxx_ide_build_dmatable(drive, func == ide_dma_write))
			return 1;
		drive->waiting_for_dma = 1;
		if (drive->media != ide_disk)
			return 0;
		ide_set_handler(drive, &ide_dma_intr, WAIT_CMD, NULL);
		OUT_BYTE(func == ide_dma_write ? WIN_WRITEDMA : WIN_READDMA,
			 IDE_COMMAND_REG);
	case ide_dma_begin:
		idp->si_c0tb = (unsigned int) prd_phys;
		idp->si_c0s0 = 0xdc800000;	/* Clear all status */
		idp->si_c0ie = 0x90000000;	/* Enable all intr */
		idp->si_c0dcm = 0;
		idp->si_c0dcm =
		    (func == ide_dma_write ? 0x09000000 : 0x01000000);
		break;
	case ide_dma_end:	/* return 1 on error, zero otherwise */
		drive->waiting_for_dma = 0;
		dstat = idp->si_c0s1;
		idp->si_c0s0 = 0xdc800000;	/* Clear all status */
		/* verify good dma status */
		return (dstat & 0x80000000);
	case ide_dma_test_irq:	/* return 1 if IRQ issued, 0 otherwise */
		if (idp->si_c0s0 & 0x10000000)
			return 1;
		break;

#if 0
		/* Let's implement tose just in case someone wants them */
	case ide_dma_bad_drive:
	case ide_dma_good_drive:
		return check_drive_lists(drive, (func == ide_dma_good_drive));
#else
	case ide_dma_bad_drive:
	case ide_dma_good_drive:
#endif
	case ide_dma_verbose:
		return report_drive_dmaing(drive);
	case ide_dma_retune:
	case ide_dma_lostirq:
	case ide_dma_timeout:
		printk(KERN_WARNING
		       "ide_stb04_dmaproc: chipset supported %s func only: %d\n",
		       ide_dmafunc_verbose(func), func);
		return 1;
	default:
		printk(KERN_WARNING
		       "ide_stb04_dmaproc: unsupported %s func: %d\n",
		       ide_dmafunc_verbose(func), func);
		return 1;
	}
	return 0;
}
#endif				/* CONFIG_BLK_DEV_IDEDMA */

void
nonpci_ide_init_hwif_ports(hw_regs_t * hw,
			   ide_ioreg_t data_port, ide_ioreg_t ctrl_port,
			   int *irq)
{
	ide_ioreg_t *p;
	unsigned char *ip;
	unsigned int uicdcr;
	int i;
	struct ocp_dev *ide_dev;
	int curr_ide;

	curr_ide = 0;

	p = hw->io_ports;
	*p = 0;
	if (irq)
		*irq = 0;

	if (data_port != 0)
		return;
	printk("IBM IDE ocp driver version %s\n", OCPVR);
	if (!(ide_dev = ocp_alloc_dev(0)))
		return;
	
	ide_dev->type = IDE;
	if ((curr_ide = ocp_register(ide_dev)) == -ENXIO) {
		ocp_free_dev(ide_dev);
		return;
	} else {

		if ((idp = (ide_t *) ioremap(ide_dev->paddr,
					     IDE0_SIZE)) == NULL) {
			printk(KERN_WARNING "ide: failed ioremap\n");
			return;
		}

		/* Enable the interface.
		 */
		idp->si_control = 0x80000000;
		idp->si_c0s0 = 0xdc800000;	/* Clear all status */
#if 0
		idp->si_c0sr = 0xf0000000;	/* Mandated val to Slew Rate Control */
#endif
		idp->si_intenable = 0x80000000;
		/* Per the STB04 data sheet:
		 *  1)  tTO = ((8*RDYT) + 1) * SYS_CLK
		 * and:
		 *  2)  tTO >= 1250 + (2 * SYS_CLK) - t2
		 * Solving the first equation for RDYT:
		 *             (tTO/SYS_CLK) - 1
		 *  3)  RDYT = -----------------
		 *                     8
		 * Substituting equation 2) for tTO in equation 3:
		 *             ((1250 + (2 * SYS_CLK) - t2)/SYS_CLK) - 1
		 *  3)  RDYT = -----------------------------------------
		 *                                8
		 * It's just the timeout so having it too long isn't too
		 * significant, so we'll just assume t2 is zero.  All this math
		 * is handled by the compiler and RDYT ends up being 11 assuming
		 * that SYS_CLOCK_NS is 15.
		 */
		idp->si_c0timo = (EZ(EZ(1250 + 2 * SYS_CLOCK_NS, SYS_CLOCK_NS) - 1, 8)) << 23;	/* Chan 0 timeout */

		/* Stuff some slow default PIO timing.
		 */
		idp->si_c0rt = MK_TIMING(6, 19, 15, 2);
		idp->si_c0fpt = MK_TIMING(6, 19, 15, 2);

		ip = (unsigned char *) (&(idp->si_c0d));	/* Chan 0 data */

		for (i = 0; i <= IDE_STATUS_OFFSET; i++) {
			*p++ = (int) (ip++);
		}
		hw->io_ports[IDE_CONTROL_OFFSET] = (int) (&(idp->si_c0adc));

		if (irq)
			*irq = ide_dev->irq;

		pio_mode[0] = pio_mode[1] = -1;

		/* We should probably have UIC functions to set external
		 * interrupt level/edge.
		 */
		uicdcr = mfdcr(DCRN_UIC_PR(UIC0));
		uicdcr &= ~(0x80000000 >> IDE0_IRQ);
		mtdcr(DCRN_UIC_PR(UIC0), uicdcr);
		mtdcr(DCRN_UIC_TR(UIC0), 0x80000000 >> IDE0_IRQ);

		/* Grab a page for the PRD Table.
		 */
		prd_table = (prd_entry_t *) consistent_alloc(GFP_KERNEL,
							     NUM_PRD *
							     sizeof
							     (prd_entry_t),
							     &prd_phys);

		ide_hwifs[data_port].tuneproc = &stb04xxx_ide_tuneproc;
		ide_hwifs[data_port].speedproc = &stb04xxx_ide_set_drive;
		/* Figure out if an 80 conductor cable is connected or not. */
		ide_hwifs[data_port].udma_four =
		    (idp->si_c0s1 & 0x20000000) != 0;
		ide_hwifs[data_port].drives[0].autotune = 1;
		ide_hwifs[data_port].drives[1].autotune = 1;

#ifdef CONFIG_BLK_DEV_IDEDMA
		ide_hwifs[data_port].autodma = 1;
		ide_hwifs[data_port].dmaproc = &stb04xxx_ide_dmaproc;
#endif

		ide_hwifs[data_port].noprobe = 0;

		memcpy(ide_hwifs[data_port].io_ports, hw->io_ports,
		       sizeof (hw->io_ports));
		ide_hwifs[data_port].irq = ide_dev->irq;
	}
}
