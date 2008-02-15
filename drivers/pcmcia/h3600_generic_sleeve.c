/*
 * drivers/pcmcia/h3600_generic
 *
 * PCMCIA implementation routines for H3600 iPAQ standards sleeves
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/i2c.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/arch/h3600-sleeve.h>
#include <asm/arch/linkup-l1110.h>

#include <linux/serial.h>
#include <pcmcia/cs_types.h>
#include <linux/sysctl.h>
#include "sa1100_generic.h"

extern int sa1100_h3600_common_pcmcia_init( struct pcmcia_init *init );
extern int sa1100_h3600_common_pcmcia_shutdown( void );
extern int sa1100_h3600_common_pcmcia_socket_state( struct pcmcia_state_array *state_array );
extern int sa1100_h3600_common_pcmcia_get_irq_info( struct pcmcia_irq_info *info );
extern int sa1100_h3600_common_pcmcia_socket_init(int sock);
extern int sa1100_h3600_common_pcmcia_socket_suspend(int sock);
extern void sa1100_h3600_pcmcia_change_sleeves(struct pcmcia_low_level *ops);
extern void sa1100_h3600_pcmcia_remove_sleeve( void );

static struct linkup_l1110 *dual_pcmcia_sleeve[2]; 

static int timing_increment_ns = 0;
static unsigned int verbose = 0;

/***************** Initialization *****************/


static int h3600_single_sleeve_pcmcia_socket_state( struct pcmcia_state_array *state_array )
{
	int sock, result;

	result = sa1100_h3600_common_pcmcia_socket_state( state_array );
	if ( result < 0 )
		return result;

	for (sock = 0; sock < 2; sock++) { 
		/* no bvd or vs bits on single pcmcia sleeve or CF sleeve */
		state_array->state[sock].bvd1=1;
		state_array->state[sock].bvd2=1;
		state_array->state[sock].wrprot=0; /* Not available on H3600. */
		state_array->state[sock].vs_3v=0;
		state_array->state[sock].vs_Xv=0;
	}
	
	return 1;
}

static int h3600_single_sleeve_pcmcia_configure_socket( const struct pcmcia_configure *configure )
{
	unsigned long flags;
        int sock = configure->sock;

        if(sock>1) 
		return -1;


	if (0) printk(__FUNCTION__ ": socket=%d vcc=%d vpp=%d reset=%d\n", 
                      sock, configure->vcc, configure->vpp, configure->reset);

	save_flags_cli(flags);
	switch (configure->vcc) {
	case 0:
		break;

	case 50:
	case 33:
		break;

	default:
		printk(KERN_ERR "%s(): unrecognized Vcc %u\n", __FUNCTION__,
		       configure->vcc);
		restore_flags(flags);
		return -1;
	}

	assign_h3600_egpio( IPAQ_EGPIO_CARD_RESET, configure->reset );
	restore_flags(flags);
	return 0;
}

/* This function implements the BS value calculation for setting the MECR
 * using integer arithmetic:
 */
static inline unsigned int sa1100_pcmcia_mecr_bs(unsigned int pcmcia_cycle_ns,
						 unsigned int cpu_clock_khz)
{
	unsigned int t = ((pcmcia_cycle_ns * cpu_clock_khz) / 6) - 1000000;
	return (t / 1000000) + (((t % 1000000) == 0) ? 0 : 1);
}

static int h3600_pcmcia_mecr_timing(unsigned int sock, unsigned int cpu_speed,
		unsigned int cmd_time )
{
        unsigned int timing = sa1100_pcmcia_mecr_bs( cmd_time + timing_increment_ns, cpu_speed );
        if (verbose)
                printk(__FUNCTION__ ": sock=%d cpu_speed=%d cmd_time=%d timing=%x\n",
		       sock, cpu_speed, cmd_time + timing_increment_ns, timing);
        return timing;
}

/*******************************************************/

struct pcmcia_low_level h3600_single_sleeve_ops = {
	init:              sa1100_h3600_common_pcmcia_init,
	shutdown:          sa1100_h3600_common_pcmcia_shutdown,
	socket_state:      h3600_single_sleeve_pcmcia_socket_state,
	get_irq_info:      sa1100_h3600_common_pcmcia_get_irq_info,
	configure_socket:  h3600_single_sleeve_pcmcia_configure_socket,
        socket_init:       sa1100_h3600_common_pcmcia_socket_init,
        socket_suspend:    sa1100_h3600_common_pcmcia_socket_suspend,
        socket_get_timing: h3600_pcmcia_mecr_timing,
};

/*************************************************************************************/
/*     
       Driverfs support 
 */
/*************************************************************************************/

#ifdef CONFIG_DRIVERFS_FS
static struct iobus_driver h3600_generic_pcmcia_iobus_driver = {
        name: "pcmcia driver",
};

static struct iobus h3600_generic_pcmcia_iobus = {
        parent: &h3600_sleeve_iobus,
        name: "pcmcia iobus",
        bus_id: "pcmcia",
        driver: &h3600_generic_pcmcia_iobus_driver,
};
#endif


/*************************************************************************************/
/*     
       Compact Flash sleeve 
 */
/*************************************************************************************/

static int __devinit cf_probe_sleeve(struct sleeve_dev *sleeve_dev, const struct sleeve_device_id *ent)
{
        if (0) printk(__FUNCTION__ ": %s\n", sleeve_dev->driver->name);
        sa1100_h3600_pcmcia_change_sleeves(&h3600_single_sleeve_ops);
        return 0;
}

static void __devexit cf_remove_sleeve(struct sleeve_dev *sleeve_dev)
{
        if (0) printk(__FUNCTION__ ": %s\n", sleeve_dev->driver->name);
        sa1100_h3600_pcmcia_remove_sleeve();
}

#ifdef CONFIG_DRIVERFS_FS
static struct device_driver cf_device_driver = {
        probe: cf_probe_sleeve,
        remove: cf_remove_sleeve,
};
#endif

static struct sleeve_device_id cf_tbl[] __devinitdata = {
        { COMPAQ_VENDOR_ID, SINGLE_COMPACTFLASH_SLEEVE },
        { 0, }
};

static struct sleeve_driver cf_driver = {
        name:     "Compaq Compact Flash Sleeve",
        id_table: cf_tbl,
	features: SLEEVE_HAS_CF(1),
        probe:    cf_probe_sleeve,
        remove:   cf_remove_sleeve,
#ifdef CONFIG_DRIVERFS_FS
        driver:   &cf_device_driver, 
#endif
};

static struct sleeve_device_id cf_plus_tbl[] __devinitdata = {
        { COMPAQ_VENDOR_ID, SINGLE_CF_PLUS_SLEEVE },
        { 0, }
};

static struct sleeve_driver cf_plus_driver = {
        name:     "Compaq Compact Flash Plus Sleeve",
        id_table: cf_plus_tbl,
	features: (SLEEVE_HAS_CF(1) | SLEEVE_HAS_REMOVABLE_BATTERY | SLEEVE_HAS_EBAT),
        probe:    cf_probe_sleeve,
        remove:   cf_remove_sleeve,
#ifdef CONFIG_DRIVERFS_FS
        driver:   &cf_device_driver, 
#endif
};

static struct sleeve_device_id gprs_tbl[] __devinitdata = {
        { COMPAQ_VENDOR_ID, GPRS_EXPANSION_PACK },
        { 0, }
};

static struct sleeve_driver gprs_driver = {
        name:     "Compaq GSM/GPRS Sleeve",
        id_table: gprs_tbl,
	features: (SLEEVE_HAS_FIXED_BATTERY | SLEEVE_HAS_GPRS),
        probe:    cf_probe_sleeve,
        remove:   cf_remove_sleeve,
#ifdef CONFIG_DRIVERFS_FS
        driver:   &cf_device_driver, 
#endif
};


/*************************************************************************************/
/*     
       Single slot PCMCIA sleeve 
 */
/*************************************************************************************/

static int __devinit pcmcia_probe_sleeve(struct sleeve_dev *sleeve_dev, 
					 const struct sleeve_device_id *ent)
{
        if (0) printk(__FUNCTION__ ": %s\n", sleeve_dev->driver->name);
        sa1100_h3600_pcmcia_change_sleeves(&h3600_single_sleeve_ops);
//        pcmcia_sleeve_attach_flash(pcmcia_set_vpp, 0x02000000);
        return 0;
}

static void __devexit pcmcia_remove_sleeve(struct sleeve_dev *sleeve_dev)
{
        if (0) printk(__FUNCTION__ ": %s\n", sleeve_dev->driver->name);
//        pcmcia_sleeve_detach_flash();
	sa1100_h3600_pcmcia_remove_sleeve();
}

static struct sleeve_device_id pcmcia_tbl[] __devinitdata = {
        { COMPAQ_VENDOR_ID, SINGLE_PCMCIA_SLEEVE },
        { 0xFFFF, 0xFFFF },
        { 0, }
};

#ifdef CONFIG_DRIVERFS_FS
static struct device_driver pcmcia_device_driver = {
        probe: pcmcia_probe_sleeve,
        remove: pcmcia_remove_sleeve,
};
#endif

static struct sleeve_driver pcmcia_driver = {
        name:     "Compaq PC Card Sleeve",
        id_table: pcmcia_tbl,
	features: (SLEEVE_HAS_PCMCIA(1) | SLEEVE_HAS_FIXED_BATTERY),
        probe:    pcmcia_probe_sleeve,
        remove:   pcmcia_remove_sleeve,
#ifdef CONFIG_DRIVERFS_FS
        driver:   &pcmcia_device_driver,
#endif
};



/*************************************************************************************/
/*     
       Compaq Bluetooth/CF Sleeve (operationally equivalent to dual CF sleeve)
 */
/*************************************************************************************/

static struct sleeve_device_id bluetooth_cf_tbl[] __devinitdata = {
        { COMPAQ_VENDOR_ID, BLUETOOTH_EXPANSION_PACK },
        { 0, }
};

static struct sleeve_driver bluetooth_cf_driver = {
        name:     "Compaq Bluetooth/CF Sleeve",
        id_table: bluetooth_cf_tbl,
	features: (SLEEVE_HAS_CF(1) | SLEEVE_HAS_BLUETOOTH),
        probe:    pcmcia_probe_sleeve,
        remove:   pcmcia_remove_sleeve,
#ifdef CONFIG_DRIVERFS_FS
        driver:   &pcmcia_device_driver,
#endif
};

/*************************************************************************************/
/*     
       Nexian Camera/CF Sleeve
 */
/*************************************************************************************/

static struct sleeve_device_id nexian_camera_cf_tbl[] __devinitdata = {
        { NEXIAN_VENDOR, NEXIAN_CAMERA_CF_SLEEVE },
        { 0, }
};

static struct sleeve_driver nexian_camera_cf_driver = {
        name:     "Nexian Nexicam Camera/CF Sleeve",
        id_table: nexian_camera_cf_tbl,
	features: (SLEEVE_HAS_CF(1) | SLEEVE_HAS_CAMERA),
        probe:    pcmcia_probe_sleeve,
        remove:   pcmcia_remove_sleeve,
#ifdef CONFIG_DRIVERFS_FS
        driver:   &pcmcia_device_driver,
#endif
};

/*************************************************************************************/
/*     
       Navman GPS/CF Sleeve
 */
/*************************************************************************************/

/* adapted from drivers/char/pcmcia/serial_cs.c (where it is a static procedure) */
static int setup_serial(ioaddr_t port, int irq)
{
    struct serial_struct serial;
    int line;
    
    memset(&serial, 0, sizeof(serial));
    serial.port = port;
    serial.irq = irq;
    serial.flags = ASYNC_SKIP_TEST | ASYNC_SHARE_IRQ;
    line = register_serial(&serial);
    if (line < 0) {
	printk(KERN_NOTICE " " __FILE__ ": register_serial() at 0x%04lx,"
	       " irq %d failed\n", (u_long)serial.port, serial.irq);
    }
    return line;
}

static int serial_port = 0;
static int serial_irq = IRQ_GPIO_H3600_OPT_IRQ;
MODULE_PARM(serial_port, "i");
MODULE_PARM(serial_irq, "i");
static int serial_line = -1;
static void *serial_ioaddr = 0;
static int __devinit navman_gps_cf_probe_sleeve(struct sleeve_dev *sleeve_dev, 
                                                const struct sleeve_device_id *ent)
{
        serial_ioaddr = ioremap(0x30000000 + serial_port, PAGE_SIZE);
        if (1) printk(__FUNCTION__ ": %s serial_port=%x serial_ioaddr=%p\n", 
                      sleeve_dev->driver->name, serial_port, serial_ioaddr);
        sa1100_h3600_pcmcia_change_sleeves(&h3600_single_sleeve_ops);

        GPDR &= ~GPIO_H3600_OPT_IRQ;    /* GPIO line as input */
        set_GPIO_IRQ_edge( GPIO_H3600_OPT_IRQ, GPIO_RISING_EDGE );  /* Rising edge */

        serial_line = setup_serial((ioaddr_t) serial_ioaddr,  serial_irq);
        return 0;
}

static void __devexit navman_gps_cf_remove_sleeve(struct sleeve_dev *sleeve_dev)
{
        if (1) printk(__FUNCTION__ ": %s\n", sleeve_dev->driver->name);
        if (serial_line > 0)
                unregister_serial(serial_line);
        if (serial_ioaddr) 
                iounmap(serial_ioaddr);
	sa1100_h3600_pcmcia_remove_sleeve();
}

static struct sleeve_device_id navman_gps_cf_tbl[] __devinitdata = {
        { TALON_VENDOR, NAVMAN_GPS_SLEEVE },
	{ TALON_VENDOR, NAVMAN_GPS_SLEEVE_ALT },
        { 0, }
};

static struct sleeve_driver navman_gps_cf_driver = {
        name:     "Navman GPS/CF Sleeve",
        id_table: navman_gps_cf_tbl,
	features: (SLEEVE_HAS_CF(1) | SLEEVE_HAS_GPS),
        probe:    navman_gps_cf_probe_sleeve,
        remove:   navman_gps_cf_remove_sleeve,
#ifdef CONFIG_DRIVERFS_FS
        driver:   &pcmcia_device_driver,
#endif
};

/*************************************************************************************/
/*     
       Symbol wlan barcode scanner sleeve 
 */
/*************************************************************************************/

static struct sleeve_device_id symbol_wlan_scanner_tbl[] __devinitdata = {
        { SYMBOL_VENDOR, SYMBOL_WLAN_SCANNER_SLEEVE },
        { 0, }
};

#ifdef CONFIG_DRIVERFS_FS
static struct device_driver symbol_wlan_scanner_device_driver = {
        probe: pcmcia_probe_sleeve,
        remove: pcmcia_remove_sleeve,
};
#endif

static struct sleeve_driver symbol_wlan_scanner_driver = {
        name:     "Symbol WLAN Scanner Sleeve",
        id_table: symbol_wlan_scanner_tbl,
	features: (SLEEVE_HAS_BARCODE_READER | SLEEVE_HAS_802_11B | SLEEVE_HAS_FIXED_BATTERY),
        probe:    pcmcia_probe_sleeve,
        remove:   pcmcia_remove_sleeve,
#ifdef CONFIG_DRIVERFS_FS
        driver:   &symbol_wlan_scanner_device_driver,
#endif
};

/*************************************************************************************/
/*     
       Compaq Dual Sleeve
 */
/*************************************************************************************/


static int h3600_dual_sleeve_pcmcia_init( struct pcmcia_init *init )
{
	dual_pcmcia_sleeve[0] = (struct linkup_l1110 *)__ioremap(0x1a000000, PAGE_SIZE, 0);
	dual_pcmcia_sleeve[1] = (struct linkup_l1110 *)__ioremap(0x19000000, PAGE_SIZE, 0);

	writel(LINKUP_PRC_S2|LINKUP_PRC_S1, &dual_pcmcia_sleeve[0]->prc);
	writel(LINKUP_PRC_S2|LINKUP_PRC_S1|LINKUP_PRC_SSP, &dual_pcmcia_sleeve[1]->prc);

	return sa1100_h3600_common_pcmcia_init( init );
}

static int h3600_dual_sleeve_pcmcia_shutdown( void )
{
	__iounmap(dual_pcmcia_sleeve[0]);
	__iounmap(dual_pcmcia_sleeve[1]);

	return sa1100_h3600_common_pcmcia_shutdown();
}

static int h3600_dual_sleeve_pcmcia_socket_state( struct pcmcia_state_array *state_array )
{
	int sock, result;

	result = sa1100_h3600_common_pcmcia_socket_state( state_array );
	if ( result < 0 )
		return result;

	for (sock = 0; sock < 2; sock++) { 
		short prs = readl(&dual_pcmcia_sleeve[sock]->prc);
		state_array->state[sock].bvd1 = prs & LINKUP_PRS_BVD1;
		state_array->state[sock].bvd2 = prs & LINKUP_PRS_BVD2;
		state_array->state[sock].wrprot = 0;
		if ((prs & LINKUP_PRS_VS1) == 0)
			state_array->state[sock].vs_3v = 1;
		if ((prs & LINKUP_PRS_VS2) == 0)
			state_array->state[sock].vs_Xv = 1;
	}

	return 1;
}

static int h3600_dual_sleeve_pcmcia_configure_socket( const struct pcmcia_configure *configure )
{
	unsigned long flags;
	unsigned int  prc;
        int sock = configure->sock;

        if(sock>1) 
		return -1;

	if (0) printk(__FUNCTION__ ": socket=%d vcc=%d vpp=%d reset=%d\n", 
                      sock, configure->vcc, configure->vpp, configure->reset);

	prc = (LINKUP_PRC_APOE | LINKUP_PRC_SOE | LINKUP_PRC_S1 | LINKUP_PRC_S2
	       | (sock * LINKUP_PRC_SSP));

	save_flags_cli(flags);
	/* Linkup Systems L1110 with TI TPS2205 PCMCIA Power Switch */
	/* S1 is VCC5#, S2 is VCC3# */ 
	/* S3 is VPP_VCC, S4 is VPP_PGM */
	/* PWR_ON is wired to #SHDN */
	switch (configure->vcc) {
	case 0:
		break;
	case 50:
		prc &= ~LINKUP_PRC_S1;
		break;
	case 33:
		prc &= ~LINKUP_PRC_S2;
		break;
	default:
		printk(KERN_ERR "%s(): unrecognized Vcc %u\n", __FUNCTION__,
		       configure->vcc);
		restore_flags(flags);
		return -1;
	}
	if (configure->vpp == 12) {
		prc |= LINKUP_PRC_S4;
	} else if (configure->vpp == configure->vcc) {
		prc |= LINKUP_PRC_S3;
	}

	if (configure->reset)
		prc |= LINKUP_PRC_RESET;

	writel(prc, &dual_pcmcia_sleeve[sock]->prc);

	restore_flags(flags);
	return 0;
}

struct pcmcia_low_level h3600_dual_sleeve_ops = {
	init:              h3600_dual_sleeve_pcmcia_init,
	shutdown:          h3600_dual_sleeve_pcmcia_shutdown,
	socket_state:      h3600_dual_sleeve_pcmcia_socket_state,
	get_irq_info:      sa1100_h3600_common_pcmcia_get_irq_info,
	configure_socket:  h3600_dual_sleeve_pcmcia_configure_socket,
        socket_init:       sa1100_h3600_common_pcmcia_socket_init,
        socket_suspend:    sa1100_h3600_common_pcmcia_socket_suspend,
        socket_get_timing: h3600_pcmcia_mecr_timing,
};

static int __devinit dual_pcmcia_probe_sleeve(struct sleeve_dev *sleeve_dev, 
					      const struct sleeve_device_id *ent)
{
        if (0) printk(__FUNCTION__ ": %s\n", sleeve_dev->driver->name);
        sa1100_h3600_pcmcia_change_sleeves(&h3600_dual_sleeve_ops);
        return 0;
}

static void __devexit dual_pcmcia_remove_sleeve(struct sleeve_dev *sleeve_dev)
{
        if (0) printk(__FUNCTION__ ": %s\n", sleeve_dev->driver->name);
	sa1100_h3600_pcmcia_remove_sleeve();
}

static struct sleeve_device_id dual_pcmcia_tbl[] __devinitdata = {
        { COMPAQ_VENDOR_ID, DUAL_PCMCIA_SLEEVE },
        { 0, }
};

#ifdef CONFIG_DRIVERFS_FS
static struct device_driver dual_pcmcia_device_driver = {
        probe: dual_pcmcia_probe_sleeve,
        remove: dual_pcmcia_remove_sleeve,
};
#endif

static struct sleeve_driver dual_pcmcia_driver = {
        name:     "Compaq Dual PC Card Sleeve",
        id_table: dual_pcmcia_tbl,
	features: (SLEEVE_HAS_PCMCIA(2) | SLEEVE_HAS_FIXED_BATTERY),
        probe:    dual_pcmcia_probe_sleeve,
        remove:   dual_pcmcia_remove_sleeve,
#ifdef CONFIG_DRIVERFS_FS
        driver:   &dual_pcmcia_device_driver,
#endif
};



static struct sleeve_device_id pcmcia_plus_tbl[] __devinitdata = {
        { COMPAQ_VENDOR_ID, SINGLE_PCMCIA_PLUS_SLEEVE },
        { 0, }
};

static struct sleeve_driver pcmcia_plus_driver = {
        name:     "Compaq PC Card Plus Sleeve",
        id_table: pcmcia_plus_tbl,
	features: (SLEEVE_HAS_PCMCIA(1) | SLEEVE_HAS_REMOVABLE_BATTERY | SLEEVE_HAS_EBAT),
        probe:    dual_pcmcia_probe_sleeve,
        remove:   dual_pcmcia_remove_sleeve,
#ifdef CONFIG_DRIVERFS_FS
        driver:   &dual_pcmcia_device_driver,
#endif
};



/*************************************************************************************/
/*     
       Nexian Dual CF Sleeve
 */
/*************************************************************************************/

static struct sleeve_device_id nexian_dual_cf_tbl[] __devinitdata = {
        { NEXIAN_VENDOR, NEXIAN_DUAL_CF_SLEEVE },
        { 0, }
};

static struct sleeve_driver nexian_dual_cf_driver = {
        name:     "Nexian Dual CF Sleeve",
        id_table: nexian_dual_cf_tbl,
	features: (SLEEVE_HAS_CF(2) | SLEEVE_HAS_REMOVABLE_BATTERY),
        probe:    dual_pcmcia_probe_sleeve,
        remove:   dual_pcmcia_remove_sleeve,
#ifdef CONFIG_DRIVERFS_FS
        driver:   &pcmcia_device_driver,
#endif
};


/*************************************************************************************/

static struct ctl_table pcmcia_ops_table[] = 
{
	{1, "timing_increment_ns", &timing_increment_ns, sizeof(int), 0644, NULL, &proc_dointvec },
	{2, "verbose",             &verbose,             sizeof(int), 0644, NULL, &proc_dointvec },
	{0}
};

static struct ctl_table pcmcia_table[] = 
{
	{BUS_PCMCIA, "pcmcia", NULL, 0, 0555, pcmcia_ops_table},
	{0}
};
static struct ctl_table bus_dir_table[] = 
{
	{CTL_BUS, "bus", NULL, 0, 0555, pcmcia_table},
        {0}
};
static struct ctl_table_header *pcmcia_sleeve_ctl_table_header = NULL;

/*************************************************************************************/

int __init h3600_generic_pcmcia_init_module(void)
{
	if (0) printk(__FUNCTION__ ": registering sleeve drivers\n");
        h3600_sleeve_register_driver(&cf_driver);
        h3600_sleeve_register_driver(&pcmcia_driver);
        h3600_sleeve_register_driver(&bluetooth_cf_driver);
        h3600_sleeve_register_driver(&nexian_dual_cf_driver);
        h3600_sleeve_register_driver(&nexian_camera_cf_driver);
        h3600_sleeve_register_driver(&navman_gps_cf_driver);
        h3600_sleeve_register_driver(&symbol_wlan_scanner_driver);
        h3600_sleeve_register_driver(&dual_pcmcia_driver);
        h3600_sleeve_register_driver(&pcmcia_plus_driver);
        h3600_sleeve_register_driver(&cf_plus_driver);
        h3600_sleeve_register_driver(&gprs_driver);
        pcmcia_sleeve_ctl_table_header = register_sysctl_table(bus_dir_table, 0);
	return 0;
}

void __exit h3600_generic_pcmcia_exit_module(void)
{
	if (0) printk(__FUNCTION__ ": unregistering sleeve drivers\n");
        h3600_sleeve_unregister_driver(&cf_driver);
        h3600_sleeve_unregister_driver(&pcmcia_driver);
        h3600_sleeve_unregister_driver(&bluetooth_cf_driver);
        h3600_sleeve_unregister_driver(&nexian_dual_cf_driver);
        h3600_sleeve_unregister_driver(&nexian_camera_cf_driver);
        h3600_sleeve_unregister_driver(&navman_gps_cf_driver);
        h3600_sleeve_unregister_driver(&symbol_wlan_scanner_driver);
        h3600_sleeve_unregister_driver(&dual_pcmcia_driver);
        h3600_sleeve_unregister_driver(&pcmcia_plus_driver);
        h3600_sleeve_unregister_driver(&cf_plus_driver);
        h3600_sleeve_unregister_driver(&gprs_driver);
        unregister_sysctl_table(pcmcia_sleeve_ctl_table_header);
} 

module_init(h3600_generic_pcmcia_init_module);
module_exit(h3600_generic_pcmcia_exit_module);
