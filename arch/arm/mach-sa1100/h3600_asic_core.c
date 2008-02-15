/*
* Driver interface to the ASIC Complasion chip on the iPAQ H3800
*
* Copyright 2001 Compaq Computer Corporation.
*
* Use consistent with the GNU GPL is permitted,
* provided that this copyright notice is
* preserved in its entirety in all copies and derived works.
*
* COMPAQ COMPUTER CORPORATION MAKES NO WARRANTIES, EXPRESSED OR IMPLIED,
* AS TO THE USEFULNESS OR CORRECTNESS OF THIS CODE OR ITS
* FITNESS FOR ANY PARTICULAR PURPOSE.
*
* Author:  Andrew Christian
*          <Andrew.Christian@compaq.com>
*          October 2001
*/

#include <linux/module.h>
#include <linux/version.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/mtd/mtd.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/serial.h>  /* For bluetooth */

#include <asm/irq.h>
#include <asm/uaccess.h>   /* for copy to/from user space */
#include <asm/arch/hardware.h>
#include <asm/arch/h3600_hal.h>
#include <asm/arch/h3600_asic.h>
#include <asm/arch/serial_h3800.h>

#include "h3600_asic_io.h"
#include "h3600_asic_mmc.h"
#include "h3600_asic_core.h"

#define H3600_ASIC_PROC_DIR     "asic"
#define H3600_ASIC_PROC_STATS   "stats"
#define REG_DIRNAME "registers"

/* Parameters */

int battery_sample_interval = 2000;  /* Two seconds  */
int sleep_battery_interval  = 5;     /* Five seconds */
int owm_sample_delay        = 200;   /* 20 milliseconds? */
int owm_reset_delay         = 50;    /* 50 milliseconds */

MODULE_PARM(battery_sample_interval,"i");
MODULE_PARM_DESC(battery_sample_interval,"Time between battery samples (milliseconds)");
MODULE_PARM(sleep_battery_interval,"i");
MODULE_PARM_DESC(sleep_battery_interval,"Time betwen battery samples while asleep (seconds)");
MODULE_PARM(owm_reset_delay,"i");
MODULE_PARM_DESC(owm_reset_delay,"Pause time after OWM reset (milliseconds)");
MODULE_PARM(owm_sample_delay,"i");
MODULE_PARM_DESC(owm_sample_delay,"Pause time while waiting for OWM response (milliseconds)");

MODULE_AUTHOR("Andrew Christian");
MODULE_DESCRIPTION("Hardware abstraction layer for the iPAQ H3800");

/* Statistics */
struct asic_statistics g_h3600_asic_statistics;

/***********************************************************************************
 *      Shared resources                                                           
 *                                                                                 
 *      EX1 on Clock      24.576 MHz crystal (OWM,ADC,PCM,SPI,PWM,UART,SD,Audio)       
 *      EX2               33.8688 MHz crystal (SD Controller and Audio)
 ***********************************************************************************/

struct asic_shared {
	spinlock_t        lock;
	int               clock_ex1;    /* 24.765 MHz crystal */
	int               clock_ex2;    /* 33.8688 MHz crystal */
} g_shared;

void h3600_asic_shared_add( unsigned long *s, enum ASIC_SHARED v )
{
	unsigned long flags;
	if (0) printk(__FUNCTION__ " %lx %d\n",*s,v);

	if ( (*s & v) == v )    /* Already set */
		return;

	spin_lock_irqsave( &g_shared.lock, flags );

	if ( v & ASIC_SHARED_CLOCK_EX1 && !(*s & ASIC_SHARED_CLOCK_EX1)) {
		if ( !g_shared.clock_ex1++ ) {
			if (0) printk(__FUNCTION__ ": enabling EX1\n");
			H3800_ASIC2_CLOCK_Enable |= ASIC2_CLOCK_EX1;
		}
	}

	if ( v & ASIC_SHARED_CLOCK_EX2 && !(*s & ASIC_SHARED_CLOCK_EX2)) {
		if ( !g_shared.clock_ex2++ ) {
			if (0) printk(__FUNCTION__ ": enabling EX2\n");
			H3800_ASIC2_CLOCK_Enable |= ASIC2_CLOCK_EX2;
		}
	}

	spin_unlock_irqrestore( &g_shared.lock, flags );
	*s |= v;
}

void h3600_asic_shared_release( unsigned long *s, enum ASIC_SHARED v )
{
	unsigned long flags;
	if (0) printk(__FUNCTION__ " %lx %d\n",*s,v);

	spin_lock_irqsave( &g_shared.lock, flags );

	if ( v & ASIC_SHARED_CLOCK_EX1 && !(~*s & ASIC_SHARED_CLOCK_EX1)) {
		if ( !--g_shared.clock_ex1 ) {
			if (0) printk(__FUNCTION__ ": disabling EX1\n");
			H3800_ASIC2_CLOCK_Enable &= ~ASIC2_CLOCK_EX1;
		}
	}

	if ( v & ASIC_SHARED_CLOCK_EX2 && !(~*s & ASIC_SHARED_CLOCK_EX2)) {
		if ( !--g_shared.clock_ex2 ) {
			if (0) printk(__FUNCTION__ ": disabling EX2\n");
			H3800_ASIC2_CLOCK_Enable &= ~ASIC2_CLOCK_EX2;
		}
	}

	spin_unlock_irqrestore( &g_shared.lock, flags );
	*s &= ~v;
}


static int __init h3600_asic_shared_init( void )
{
	DEBUG_INIT();
	spin_lock_init(&g_shared.lock);
	H3800_ASIC2_CLOCK_Enable &= ~(ASIC2_CLOCK_EX1 | ASIC2_CLOCK_EX2);
	return 0;
}

static void __exit h3600_asic_shared_cleanup( void )
{
	DEBUG_INIT();

	if ( H3800_ASIC2_CLOCK_Enable & (ASIC2_CLOCK_EX1 | ASIC2_CLOCK_EX2) ) {
		printk(__FUNCTION__ ": Error - EX still enabled\n");
		H3800_ASIC2_CLOCK_Enable &= ~(ASIC2_CLOCK_EX1 | ASIC2_CLOCK_EX2);
	}
}


/***********************************************************************************
 *      Generic IRQ handling
 ***********************************************************************************/

static int __init h3600_asic_init_isr( void )
{
	int result;
	DEBUG_INIT();

	/* Request IRQs */
        set_GPIO_IRQ_edge( GPIO_H3600_NPOWER_BUTTON, GPIO_BOTH_EDGES );
	result = request_irq(IRQ_GPIO_H3600_NPOWER_BUTTON, 
			     h3600_asic_power_isr, 
			     SA_INTERRUPT | SA_SAMPLE_RANDOM,
			     "h3600_suspend", NULL);

	if ( result ) {
		printk(KERN_CRIT __FUNCTION__ ": unable to grab power button IRQ\n");
		return result;
	}

	set_GPIO_IRQ_edge( GPIO_H3800_AC_IN, GPIO_BOTH_EDGES );
	result = request_irq(IRQ_GPIO_H3800_AC_IN,
			     h3600_asic_ac_in_isr,
			     SA_INTERRUPT | SA_SAMPLE_RANDOM,
			     "h3800_ac_in", NULL );
	if ( result ) {
		printk(KERN_CRIT __FUNCTION__ ": unable to grab AC in IRQ\n");
		free_irq(IRQ_GPIO_H3600_NPOWER_BUTTON, NULL );
		return result;
	}

	set_GPIO_IRQ_edge( GPIO_H3800_NOPT_IND, GPIO_BOTH_EDGES );
	result = request_irq(IRQ_GPIO_H3800_NOPT_IND,
			     h3600_asic_sleeve_isr,
			     SA_INTERRUPT | SA_SAMPLE_RANDOM,
			     "h3800_sleeve", NULL );
	if ( result ) {
		printk(KERN_CRIT __FUNCTION__ ": unable to grab sleeve insertion IRQ\n");
		free_irq(IRQ_GPIO_H3600_NPOWER_BUTTON, NULL );
		free_irq(IRQ_GPIO_H3800_AC_IN, NULL );
		return result;
	}

	return 0;
}

static void __exit h3600_asic_release_isr( void )
{
	DEBUG_INIT();

//	H3800_ASIC2_KPIINTSTAT  = 0;    
//	H3800_ASIC2_GPIINTSTAT  = 0;

	free_irq(IRQ_GPIO_H3600_NPOWER_BUTTON, NULL);
	free_irq(IRQ_GPIO_H3800_AC_IN, NULL);
	free_irq(IRQ_GPIO_H3800_NOPT_IND, NULL);
}


/***********************************************************************************/
/*      Standard entry points for HAL requests                                     */
/***********************************************************************************/

static int h3600_asic_version( struct h3600_ts_version *result )
{
	return -EINVAL;
}

static int h3600_asic_eeprom_read( unsigned short address, unsigned char *data, unsigned short len )
{
	if (0) printk(__FUNCTION__ ": address=%d len=%d\n", address, len);
        return -EINVAL;
}

static int h3600_asic_eeprom_write( unsigned short address, unsigned char *data, unsigned short len )
{
        return -EINVAL;
}

static int h3600_asic_thermal_sensor( unsigned short *result )
{
	return -EINVAL;
}

static int h3600_asic_notify_led( unsigned char mode, unsigned char duration, 
			    unsigned char ontime, unsigned char offtime )
{
	return -EINVAL;
}

static int h3600_asic_spi_write(unsigned short address, unsigned char *data, unsigned short len)
{
	return -EINVAL;
}

static int h3600_asic_read_light_sensor( unsigned char *result )
{
	int value = h3600_asic_adc_read_channel( ASIC2_ADMUX_0_LIGHTSENSOR );
	if ( value >= 0 ) {
		*result = value >> 2;
		return 0;
	}
		
	return -EIO;
}

static int h3600_asic_option_pack( int *result )
{
        int opt_ndet = (GPLR & GPIO_H3800_NOPT_IND);

	if (0) printk(__FUNCTION__ ": opt_ndet=%d\n", opt_ndet);
	*result = opt_ndet ? 0 : 1;
	return 0;
}

/***********************************************************************************
 *   Proc filesystem interface
 ***********************************************************************************/

static int h3600_asic_reset_handler(ctl_table *ctl, int write, struct file * filp,
			     void *buffer, size_t *lenp);

static struct ctl_table h3600_asic_table[] =
{
	{ 1, "owm_reset_delay", &owm_reset_delay, sizeof(int),
	  0666, NULL, &proc_dointvec },
	{ 2, "owm_sample_delay", &owm_sample_delay, sizeof(int),
	  0666, NULL, &proc_dointvec },
	{ 3, "battery_sample_interval", &battery_sample_interval, sizeof(int),
	  0666, NULL, &proc_dointvec },
	{ 4, "sleep_battery_interval", &sleep_battery_interval, sizeof(int),
	  0666, NULL, &proc_dointvec },
	{ 5, "reset", NULL, 0, 0600, NULL, (proc_handler *) &h3600_asic_reset_handler },
	{0}
};

static struct ctl_table h3600_asic_dir_table[] =
{
	{1, "asic", NULL, 0, 0555, h3600_asic_table},
	{0}
};

static struct ctl_table_header *h3600_asic_sysctl_header = NULL;

static struct proc_dir_entry   *asic_proc_dir;
static struct proc_dir_entry   *reg_proc_dir;
static struct proc_dir_entry   *adc_proc_dir;

#define PRINT_DATA(x,s) \
	p += sprintf (p, "%-28s : %d\n", s, g_h3600_asic_statistics.x)

static int h3600_asic_proc_stats_read(char *page, char **start, off_t off,
				      int count, int *eof, void *data)
{
	char *p = page;
	int len;
	int i;

	PRINT_DATA(spi_bytes,   "SPI bytes sent/received");
	PRINT_DATA(spi_wip,     "SPI write-in-process delays");
	PRINT_DATA(spi_timeout, "SPI timeouts");
	PRINT_DATA(owm_timeout, "OWM timeouts");
	PRINT_DATA(owm_reset,   "OWM reset");
	PRINT_DATA(owm_written, "OWM written");
	PRINT_DATA(owm_read,    "OWM read");
	p += sprintf(p,         "OWM ISR received   Valid Invalid Post\n");
	for ( i = 0 ; i < 5 ; i++ )
		p += sprintf(p, "     %10s : %4d   %4d   %4d\n", 
			     owm_state_names[i],
			     g_h3600_asic_statistics.owm_valid_isr[i],
			     g_h3600_asic_statistics.owm_invalid_isr[i],
			     g_h3600_asic_statistics.owm_post_isr[i]);

	p += sprintf(p, "%-28s : %d\n", "EX1 usage", g_shared.clock_ex1 );
	p += sprintf(p, "%-28s : %d\n", "EX2 usage", g_shared.clock_ex2 );

	PRINT_DATA(mmc_insert,  "MMC insert events");
	PRINT_DATA(mmc_eject,   "MMC eject event");
	PRINT_DATA(mmc_reset,   "MMC reset commands");
	PRINT_DATA(mmc_command, "MMC commands issued");
	PRINT_DATA(mmc_read,    "MMC blocks read");
	PRINT_DATA(mmc_written, "MMC blocks written");
	PRINT_DATA(mmc_timeout, "MMC timeout interrupts");
	PRINT_DATA(mmc_error ,  "MMC response errors");

	len = (p - page) - off;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}

static int h3600_asic_proc_adc_read(char *page, char **start, off_t off,
				    int count, int *eof, void *data )
{
	int result;
	char *p = page;

	MOD_INC_USE_COUNT;
	result = h3600_asic_adc_read_channel((int) data);
	if ( result < 0 )
		p += sprintf(p, "Error code %d\n", result );
	else
		p += sprintf(p, "0x%04x\n",(unsigned int) result );

	*eof = 1;
	MOD_DEC_USE_COUNT;
	return (p - page);
}

static int h3600_asic_proc_battery_read(char *page, char **start, off_t off,
					int count, int *eof, void *data)
{
	char *p = page;
	int result;
	unsigned char buf[64];
	int i;

	MOD_INC_USE_COUNT;    // Necessary because we can sleep

	result = h3600_asic_owm_read_bytes( NULL, 0, buf, 64 );
	if ( result < 0 || !buf[1] ) {
		p += sprintf(p, "Unable to read battery (%d)\n", result);
	}
	else {
		int v;
		unsigned char chem, percent, flag;
		for ( i = 0 ; i < 64 ; i++ ) {
			p += sprintf(p, " %02x", buf[i]);
			if ( ((i+1) % 8 ) == 0 )
				*p++ = '\n';
		}
		p += sprintf(p, "Protection          : 0x%02x\n", buf[0] );
		p += sprintf(p, "Status              : 0x%02x\n", buf[1] );
		p += sprintf(p, "EEPROM              : 0x%02x\n", buf[7] );
		p += sprintf(p, "Special feature     : 0x%02x\n", buf[8] );
		p += sprintf(p, "Voltage             : 0x%02x%02x", buf[12], buf[13] );
		v = ((char) buf[12]) << 8 | buf[13];
		v = (v >> 5) * 5000 / 1024;
		p += sprintf(p, " (%d mV)\n", v);
		p += sprintf(p, "Current             : 0x%02x%02x\n", buf[14], buf[15] );
		p += sprintf(p, "Accumulated current : 0x%02x%02x\n", buf[16], buf[17] );
		p += sprintf(p, "Temperature         : 0x%02x%02x\n", buf[24], buf[25] );
		p += sprintf(p, "Cell chemistry      : 0x%02x\n", buf[48] >> 6 );
		p += sprintf(p, "Manufacturer        : 0x%02x\n", (buf[48] & 0x3c) >> 2 );
		p += sprintf(p, "Rated cell capacity : %d\n", buf[51] * 10 );
		p += sprintf(p, "Current offset      : 0x%02x\n", buf[51] );
		p += sprintf(p, "Sense resistor      : 0x%02x\n", buf[52] );

		if ( !h3600_asic_spi_read_pcmcia_battery( &chem, &percent, &flag ) ) {
			p += sprintf(p, "Option jacket\n");
			p += sprintf(p, "          chemistry : 0x%02x\n", chem );
			p += sprintf(p, "         percentage : 0x%02x (%d)\n", percent, percent );
			p += sprintf(p, "               flag : 0x%02x\n", flag );
		}
	}

	*eof = 1;
	MOD_DEC_USE_COUNT;
	return (p - page);
}

/* Coded lifted from "registers.c" */

static ssize_t proc_read_reg(struct file * file, char * buf,
		size_t nbytes, loff_t *ppos);
static ssize_t proc_write_reg(struct file * file, const char * buffer,
		size_t count, loff_t *ppos);

static struct file_operations proc_reg_operations = {
	read:	proc_read_reg,
	write:	proc_write_reg
};

typedef struct asic_reg_entry {
	u16   bytes;
	u16   phyaddr;
	char* name;
	char* description;
	unsigned short low_ino;
} asic_reg_entry_t;

static asic_reg_entry_t asic_regs[] =
{
/*	{ virt_addr,   name,     description } */
	{ 4, 0x0000, "GPIODIR",    "GPIO Input/Output direction register" },
	{ 4, 0x0004, "GPIINTTYPE", "GPI Interrupt Type (Edge/Level)"},
	{ 4, 0x0008, "GPIINTESEL", "GPI Interrupt active edge select"},
	{ 4, 0x000c, "GPIINTALSEL","GPI Interrupt active level select"},
	{ 4, 0x0010, "GPIINTFLAG", "GPI Interrupt active flag" },
	{ 4, 0x0014, "GPIOPIOD",   "GPIO Port input/output data" },
	{ 4, 0x0018, "GPOBFSTAT",  "GPO output data in batt_fault" },
	{ 4, 0x001c, "GPIINTSTAT", "GPI Interrupt status" },
	{ 4, 0x003c, "GPIOALT",    "GPIO ALTernate function" },

	{ 4, 0x0200, "KPIODIR",    "KPIO Input/output direction"},
	{ 4, 0x0204, "KPIINTTYP",  "KPI interrupt type (edge/level)" },
	{ 4, 0x0208, "KPIINTESEL", "KPI Interrupt active edge select" },
	{ 4, 0x020c, "KPIINTALSEL","KPI Interrupt active level select" },
	{ 4, 0x0210, "KPIINTFLAG", "KPI Interrupt active flag" },
	{ 4, 0x0214, "KPIOPIOD",   "KPIO Port input/output data" },
	{ 4, 0x0218, "KPOBFSTAT",  "KPO Ouput data in batt_fault status" },
	{ 4, 0x021c, "KPIINTSTAT", "KPI Interrupt status" },
	{ 4, 0x023c, "KALT",       "KIU alternate function" },

	{ 4, 0x0400, "SPICR",      "SPI control register" },
	{ 4, 0x0404, "SPIDR",      "SPI data register" },
	{ 4, 0x0408, "SPIDCS",     "SPI chip select disabled register" },

	{ 4, 0x0600, "PWM0_TBS",   "PWM Time base set register" },
	{ 4, 0x0604, "PWM0_PTS",   "PWM Period time set register" },
	{ 4, 0x0608, "PWM0_DTS",   "PWM Duty time set register" },

	{ 4, 0x0700, "PWM1_TBS",   "PWM Time base set register" },
	{ 4, 0x0704, "PWM1_PTS",   "PWM Period time set register" },
	{ 4, 0x0708, "PWM1_DTS",   "PWM Duty time set register" },

	{ 4, 0x0800, "LED0_TBS",   "LED time base set register" },
	{ 4, 0x0804, "LED0_PTS",   "LED period time set register" },
	{ 4, 0x0808, "LED0_DTS",   "LED duty time set register" },
	{ 4, 0x080c, "LED0_ASTC",  "LED auto stop counter register" },

	{ 4, 0x0880, "LED1_TBS",   "LED time base set register" },
	{ 4, 0x0884, "LED1_PTS",   "LED period time set register" },
	{ 4, 0x0888, "LED1_DTS",   "LED duty time set register" },
	{ 4, 0x088c, "LED1_ASTC",  "LED auto stop counter register" },

	{ 4, 0x0900, "LED2_TBS",   "LED time base set register" },
	{ 4, 0x0904, "LED2_PTS",   "LED period time set register" },
	{ 4, 0x0908, "LED2_DTS",   "LED duty time set register" },
	{ 4, 0x090c, "LED2_ASTC",  "LED auto stop counter register" },

	{ 4, 0x0a00, "UART0_BUF",  "Receive/transmit buffer"},
	{ 4, 0x0a04, "UART0_IER",  "Interrupt enable" },
	{ 4, 0x0a08, "UART0_IIR",  "Interrupt identify" },
	{ 4, 0x0a08, "UART0_FCR",  "Fifo control" },
	{ 4, 0x0a0c, "UART0_LCR",  "Line control" },
	{ 4, 0x0a10, "UART0_MCR",  "Modem control" },
	{ 4, 0x0a14, "UART0_LSR",  "Line status" },
	{ 4, 0x0a18, "UART0_MSR",  "Modem status" },
	{ 4, 0x0a1c, "UART0_SCR",  "Scratch pad" },

	{ 4, 0x0c00, "UART1_BUF",  "Receive/transmit buffer"},
	{ 4, 0x0c04, "UART1_IER",  "Interrupt enable" },
	{ 4, 0x0c08, "UART1_IIR",  "Interrupt identify" },
	{ 4, 0x0c08, "UART1_FCR",  "Fifo control" },
	{ 4, 0x0c0c, "UART1_LCR",  "Line control" },
	{ 4, 0x0c10, "UART1_MCR",  "Modem control" },
	{ 4, 0x0c14, "UART1_LSR",  "Line status" },
	{ 4, 0x0c18, "UART1_MSR",  "Modem status" },
	{ 4, 0x0c1c, "UART1_SCR",  "Scratch pad" },

	{ 4, 0x0e00, "TIMER_0",    "Timer counter 0 register" },
	{ 4, 0x0e04, "TIMER_1",    "Timer counter 1 register" },
	{ 4, 0x0e08, "TIMER_2",    "Timer counter 2 register" },
	{ 4, 0x0e0a, "TIMER_CNTL", "Timer control register (write only)" },
	{ 4, 0x0e10, "TIMER_CMD",  "Timer command register" },

	{ 4, 0x1000, "CDEX",       "Crystal source, control clock" },

	{ 4, 0x1200, "ADMUX",      "ADC multiplixer select register" },
	{ 4, 0x1204, "ADCSR",      "ADC control and status register" },
	{ 4, 0x1208, "ADCDR",      "ADC data register" },
	
	{ 4, 0x1600, "INTMASK",    "Interrupt mask control & cold boot flag" },
	{ 4, 0x1604, "INTCPS",     "Interrupt timer clock pre-scale" },
	{ 4, 0x1608, "INTTBS",     "Interrupt timer set" },

	{ 4, 0x1800, "OWM_CMD",    "OWM command register" },
	{ 4, 0x1804, "OWM_DATA",   "OWM transmit/receive buffer" },
	{ 4, 0x1808, "OWM_INT",    "OWM interrupt register" },
	{ 4, 0x180c, "OWM_INTEN",  "OWM interrupt enable register" },
	{ 4, 0x1810, "OWM_CLKDIV", "OWM clock divisor register" },

	{ 4, 0x1a00, "SSETR",      "Size of flash memory setting register" },

	{ 2, 0x1c00, "MMC_STR_STP_CLK",      "MMC Start/Stop Clock" },
	{ 2, 0x1c04, "MMC_STATUS",           "MMC Status" },
	{ 2, 0x1c08, "MMC_CLK_RATE",         "MMC Clock rate" },
	{ 2, 0x1c0c, "MMC_REVISION",         "MMC revision" },
	{ 2, 0x1c10, "MMC_SPI_REG",          "MMC SPI register" },
	{ 2, 0x1c14, "MMC_CMD_DATA_CONT",    "MMC command data control"},
	{ 2, 0x1c18, "MMC_RESPONSE_TO",      "MMC response timeout" }, 
	{ 2, 0x1c1c, "MMC_READ_TO",          "MMC read timeout" }, 
	{ 2, 0x1c20, "MMC_BLK_LEN",          "MMC block length" },
	{ 2, 0x1c24, "MMC_NOB",              "MMC number of blocks" },
	{ 2, 0x1c34, "MMC_INT_MASK",         "MMC interrupt mask" },
	{ 2, 0x1c38, "MMC_CMD",              "MMC command number" },
	{ 2, 0x1c3c, "MMC_ARGUMENT_H",       "MMC argument high word" },
	{ 2, 0x1c40, "MMC_ARGUMENT_L",       "MMC argument low word" },
	{ 2, 0x1c44, "MMC_RES_FIFO",         "MMC response fifo" },
	{ 2, 0x1c4c, "MMC_DATA_BUF",         "MMC data buffer" },
	{ 2, 0x1c50, "MMC_BUF_PART_FULL",    "MMC buffer part full" },

	{ 2, 0x1e60, "GPIO1_MASK",           "GPIO1 mask" },
	{ 2, 0x1e64, "GPIO1_DIR",            "GPIO1 direction" },
	{ 2, 0x1e68, "GPIO1_OUT",            "GPIO1 output level" },
	{ 2, 0x1e6c, "GPIO1_LEVELTRI",       "GPIO1 level trigger (0=edge trigger)" },
	{ 2, 0x1e70, "GPIO1_RISING",         "GPIO1 rising edge trigger" },
	{ 2, 0x1e74, "GPIO1_LEVEL",          "GPIO1 high level trigger" },
	{ 2, 0x1e78, "GPIO1_LEVEL_STATUS",   "GPIO1 level trigger sttaus" },
	{ 2, 0x1e7c, "GPIO1_EDGE_STATUS",    "GPIO1 edge trigger status" },
	{ 2, 0x1e80, "GPIO1_STATE",          "GPIO1 state (read only)" },
	{ 2, 0x1e84, "GPIO1_RESET",          "GPIO1 reset" },
	{ 2, 0x1e88, "GPIO1_SLEEP_MASK",     "GPIO1 sleep mask trigger" },
	{ 2, 0x1e8c, "GPIO1_SLEEP_DIR",      "GPIO1 sleep direction" },
	{ 2, 0x1e90, "GPIO1_SLEEP_OUT",      "GPIO1 sleep level" },
	{ 2, 0x1e94, "GPIO1_STATUS",         "GPIO1 status (read only)" },
	{ 2, 0x1e98, "GPIO1_BATT_FAULT_DIR", "GPIO1 battery fault direction" },
	{ 2, 0x1e9c, "GPIO1_BATT_FAULT_OUT", "GPIO1 battery fault level" },

	{ 4, 0x1f00, "FLASHWP",    "Flash write protect" },
};

#define NUM_OF_ASIC_REG_ENTRY	(sizeof(asic_regs)/sizeof(asic_reg_entry_t))

static int proc_read_reg(struct file * file, char * buf,
		size_t nbytes, loff_t *ppos)
{
	int i_ino = (file->f_dentry->d_inode)->i_ino;
	char outputbuf[15];
	int count;
	int i;
	asic_reg_entry_t* current_reg=NULL;
	if (*ppos>0) /* Assume reading completed in previous read*/
		return 0;
	for (i=0;i<NUM_OF_ASIC_REG_ENTRY;i++) {
		if (asic_regs[i].low_ino==i_ino) {
			current_reg = &asic_regs[i];
			break;
		}
	}
	if (current_reg==NULL)
		return -EINVAL;

	switch (current_reg->bytes) {
	case 1:
		count = sprintf(outputbuf, "0x%02X\n", *((volatile u8 *)(_H3800_ASIC2_Base + current_reg->phyaddr)));
		break;
	case 2:
		count = sprintf(outputbuf, "0x%04X\n", *((volatile u16 *)(_H3800_ASIC2_Base + current_reg->phyaddr)));
		break;
	case 4:
	default:
		count = sprintf(outputbuf, "0x%08X\n", *((volatile u32 *)(_H3800_ASIC2_Base + current_reg->phyaddr)));
		break;
	}
	*ppos+=count;
	if (count>nbytes)  /* Assume output can be read at one time */
		return -EINVAL;
	if (copy_to_user(buf, outputbuf, count))
		return -EFAULT;
	return count;
}

static ssize_t proc_write_reg(struct file * file, const char * buffer,
		size_t count, loff_t *ppos)
{
	int i_ino = (file->f_dentry->d_inode)->i_ino;
	asic_reg_entry_t* current_reg=NULL;
	int i;
	unsigned long newRegValue;
	char *endp;

	for (i=0;i<NUM_OF_ASIC_REG_ENTRY;i++) {
		if (asic_regs[i].low_ino==i_ino) {
			current_reg = &asic_regs[i];
			break;
		}
	}
	if (current_reg==NULL)
		return -EINVAL;

	newRegValue = simple_strtoul(buffer,&endp,0);
	switch (current_reg->phyaddr) {
	case 1:
		*((volatile u8 *)(_H3800_ASIC2_Base + current_reg->phyaddr))=newRegValue;
		break;
	case 2:
		*((volatile u16 *)(_H3800_ASIC2_Base + current_reg->phyaddr))=newRegValue;
		break;
	case 4:
	default:
		*((volatile u32 *)(_H3800_ASIC2_Base + current_reg->phyaddr))=newRegValue;
		break;
	}
	return (count+endp-buffer);
}

static int h3600_asic_proc_wakeup_read(char *page, char **start, off_t off,
				       int count, int *eof, void *data );

static int __init h3600_asic_register_procfs( void )
{
	int i;

	asic_proc_dir = proc_mkdir(H3600_ASIC_PROC_DIR, NULL);
	if ( !asic_proc_dir ) {
		printk(KERN_ALERT __FUNCTION__ 
		       ": unable to create proc entry " H3600_ASIC_PROC_DIR "\n");
		return -ENOMEM;
	}

	create_proc_read_entry(H3600_ASIC_PROC_STATS, 0, asic_proc_dir, 
			       h3600_asic_proc_stats_read, NULL );

	create_proc_read_entry("battery", 0, asic_proc_dir, 
			       h3600_asic_proc_battery_read, NULL );

	create_proc_read_entry("wakeup", 0, asic_proc_dir, 
			       h3600_asic_proc_wakeup_read, NULL );

	adc_proc_dir = proc_mkdir("adc", asic_proc_dir);
	if (adc_proc_dir == NULL) {
		printk(KERN_ERR __FUNCTION__ ": can't create /proc/adc\n");
		return(-ENOMEM);
	}
	for (i=0;i<5;i++) {
		unsigned char name[10];
		sprintf(name,"%d",i);
		create_proc_read_entry(name, S_IWUSR |S_IRUSR | S_IRGRP | S_IROTH, 
					       adc_proc_dir, h3600_asic_proc_adc_read, (void *) i );
	}

	reg_proc_dir = proc_mkdir(REG_DIRNAME, asic_proc_dir);
	if (reg_proc_dir == NULL) {
		printk(KERN_ERR __FUNCTION__ ": can't create /proc/" REG_DIRNAME "\n");
		return(-ENOMEM);
	}

	for(i=0;i<NUM_OF_ASIC_REG_ENTRY;i++) {
		struct proc_dir_entry *entry = create_proc_entry(asic_regs[i].name,
								 S_IWUSR |S_IRUSR | S_IRGRP | S_IROTH,
								 reg_proc_dir);
		
		if ( !entry ) {
			printk( KERN_ERR __FUNCTION__ ": can't create /proc/" REG_DIRNAME
				"/%s\n", asic_regs[i].name);
			return(-ENOMEM);
		}

		asic_regs[i].low_ino = entry->low_ino;
		entry->proc_fops = &proc_reg_operations;
	}

	h3600_asic_sysctl_header = register_sysctl_table(h3600_asic_dir_table, 0 );
	return 0;
}

static void __exit h3600_asic_unregister_procfs( void )
{
	int i;
	if ( asic_proc_dir ) {
		unregister_sysctl_table(h3600_asic_sysctl_header);
		for(i=0;i<NUM_OF_ASIC_REG_ENTRY;i++)
			remove_proc_entry(asic_regs[i].name,reg_proc_dir);
		remove_proc_entry(REG_DIRNAME, asic_proc_dir);
		for(i=0;i<5;i++) {
			char name[10];
			sprintf(name,"%d",i);
			remove_proc_entry(name,adc_proc_dir);
		}
		remove_proc_entry("adc",asic_proc_dir);
		remove_proc_entry("battery", asic_proc_dir);
		remove_proc_entry("wakeup", asic_proc_dir);
		remove_proc_entry(H3600_ASIC_PROC_STATS, asic_proc_dir);
		remove_proc_entry(H3600_ASIC_PROC_DIR, NULL );
		asic_proc_dir = NULL;
	}
}

/***********************************************************************************
 *   Utility routines
 ***********************************************************************************/

struct asic_system_handler { 
	char *name;
	int  (*init)( void );
	void (*cleanup)( void );
	int  (*suspend)( void );
	void (*resume)( void );
};

/* 
   We initialize and resume from top to bottom,
   cleanup and suspend from bottom to top 
*/
   
const struct asic_system_handler asic_system_handlers[] = {
	{ 
		name:    "shared",
		init:    h3600_asic_shared_init,      
		cleanup: h3600_asic_shared_cleanup 
	},{ 
		name:   "adc",
		init:    h3600_asic_adc_init,
		cleanup: h3600_asic_adc_cleanup,
		suspend: h3600_asic_adc_suspend,
		resume:  h3600_asic_adc_resume,
	},{ 
		name:    "key",
		init:    h3600_asic_key_init,
		cleanup: h3600_asic_key_cleanup,
		suspend: h3600_asic_key_suspend,
		resume:  h3600_asic_key_resume 
	},{ 
		name:    "spi",
		init:    h3600_asic_spi_init,
		cleanup: h3600_asic_spi_cleanup,
		suspend: h3600_asic_spi_suspend,
		resume:  h3600_asic_spi_resume 
	},{ 
		name:    "backlight",
		init:    h3600_asic_backlight_init,   
		cleanup: h3600_asic_backlight_cleanup 
	},{ 
		name:    "touchscreen",
		init:    h3600_asic_touchscreen_init, 
		cleanup: h3600_asic_touchscreen_cleanup,
		suspend: h3600_asic_touchscreen_suspend, 
		resume:  h3600_asic_touchscreen_resume 
	},{ 
		name:    "owm",
		init:    h3600_asic_owm_init, 
		cleanup: h3600_asic_owm_cleanup,
		suspend: h3600_asic_owm_suspend, 
		resume:  h3600_asic_owm_resume 
	},{ 
		name:    "battery",
		init:    h3600_asic_battery_init, 
		cleanup: h3600_asic_battery_cleanup,
		suspend: h3600_asic_battery_suspend, 
		resume:  h3600_asic_battery_resume 
	},{ 
		name:    "audio",
		init:    h3600_asic_audio_init, 
		cleanup: h3600_asic_audio_cleanup,
		suspend: h3600_asic_audio_suspend, 
		resume:  h3600_asic_audio_resume  
	},{ 
		name:    "asset",
		init:    h3600_asic_asset_init,   
		cleanup: h3600_asic_asset_cleanup 
	},{
		name:    "bluetooth",
		init:    h3600_asic_bluetooth_init,
		cleanup: h3600_asic_bluetooth_cleanup,
		suspend: h3600_asic_bluetooth_suspend,
		resume:  h3600_asic_bluetooth_resume
	},{
		name:    "mmc",
		init:    h3600_asic_mmc_init,
		cleanup: h3600_asic_mmc_cleanup,
		suspend: h3600_asic_mmc_suspend,
		resume:  h3600_asic_mmc_resume
	}
};

#define SYS_HANDLER_SIZE   (sizeof(asic_system_handlers)/sizeof(struct asic_system_handler))

static int h3600_asic_suspend_handlers( void )
{
	int i;
	int result;

	for ( i = SYS_HANDLER_SIZE - 1 ; i >= 0 ; i-- ) {
		if ( asic_system_handlers[i].suspend ) {
			if ((result = asic_system_handlers[i].suspend()) != 0 ) {
				while ( ++i < SYS_HANDLER_SIZE )
					if ( asic_system_handlers[i].resume )
						asic_system_handlers[i].resume();
				return result;
			}
		}
	}
	return 0;
}

static void h3600_asic_resume_handlers( void )
{
	int i;

	for ( i = 0 ; i < SYS_HANDLER_SIZE ; i++ )
		if ( asic_system_handlers[i].resume )
			asic_system_handlers[i].resume();
}

static int __init h3600_asic_init_handlers( void )
{
	int i;
	int result;

	for ( i = 0 ; i < SYS_HANDLER_SIZE ; i++ ) {
		if ( asic_system_handlers[i].init ) {
			if ( (result = asic_system_handlers[i].init()) != 0 ) {
				while ( --i >= 0 )
					if ( asic_system_handlers[i].cleanup )
						asic_system_handlers[i].cleanup();
				return result;
			}
		}
	}
	return 0;
}

static void __exit h3600_asic_cleanup_handlers( void )
{
	int i;

	for ( i = SYS_HANDLER_SIZE - 1 ; i >= 0 ; i-- )
		if ( asic_system_handlers[i].cleanup )
			asic_system_handlers[i].cleanup();
}

/***********************************************************************************
 *   Power management
 *
 *   On sleep, if we return anything other than "0", we will cancel sleeping.
 *
 *   On resume, if we return anything other than "0", we will put the iPAQ
 *     back to sleep immediately.
 ***********************************************************************************/

enum {
	REASON_BUTTON      = 0x01,
	REASON_RTC_BATTERY = 0x02,
	REASON_RTC_OTHER   = 0x04,
	REASON_AC_IN       = 0x08,
	REASON_AC_OUT      = 0x10
};

struct wakeup_reason {
	int reason;
	int rcnr;
	int saved_rtar;
};

#define MAX_REASONS  24

struct sleep_state {
	struct wakeup_reason reason_list[MAX_REASONS];
	int count;
	int ac_power;
	u32 saved_rtar;
	enum charging_state state;
};

static struct sleep_state g_sleep;

static void h3600_asic_set_for_sleep( struct sleep_state *s )
{
	u32 rcnr = RCNR;

        s->ac_power   = (GPLR & GPIO_H3800_AC_IN) ? 0 : 1;
	s->saved_rtar = 0;

	if ( s->ac_power ) {
		H3800_ASIC1_GPIO_OUT     |= GPIO1_CH_TIMER;
		if ( s->state == CHARGING_STATE_NO_AC || s->state == CHARGING_STATE_INIT )
			s->state = CHARGING_STATE_ACTIVE;
	}
	else {
		H3800_ASIC1_GPIO_OUT     &= ~GPIO1_CH_TIMER;
		H3800_ASIC2_CLOCK_Enable &= ~ASIC2_CLOCK_ADC;
		if ( s->state == CHARGING_STATE_ACTIVE || s->state == CHARGING_STATE_FULL )
			s->state = CHARGING_STATE_NO_AC;
	}

	h3600_asic_battery_set_led( s->state );

	if ( (RTSR & RTSR_ALE) && (RTAR > rcnr)) {
		s->saved_rtar = RTAR;   // Save the old interrupt time
		if ( RTAR <= rcnr + sleep_battery_interval + 2)
			return;   // The RTC interrupt occurs very soon; don't set our own
	}

	/* Set up a battery RTC alarm and run the ADC converter */
	if ( s->ac_power ) {
		RTAR  = rcnr + sleep_battery_interval;
		RTSR |= RTSR_ALE;

		H3800_ASIC2_CLOCK_Enable |= ASIC2_CLOCK_ADC;
		H3800_ASIC2_ADMUX         = ASIC2_ADMUX_1_IMIN | ASIC2_ADMUX_CLKEN;
		H3800_ASIC2_ADCSR         = ASIC2_ADCSR_ADPS(4) | ASIC2_ADCSR_ENABLE;
		H3800_ASIC2_ADCSR        |= ASIC2_ADCSR_START;
	}
}

static int h3600_asic_wakeup_reason( struct sleep_state *s )
{
	struct wakeup_reason *r;
	int reason   = 0;
        int ac_power = (GPLR & GPIO_H3800_AC_IN) ? 0 : 1;

	if ( ac_power != s->ac_power ) 
		reason |= ( ac_power ? REASON_AC_IN : REASON_AC_OUT);

	if ( ipaq_model_ops.icpr & IC_RTCAlrm ) {
		if ( s->saved_rtar == 0 || s->saved_rtar > RCNR )
			reason |= REASON_RTC_BATTERY;
		else
			reason |= REASON_RTC_OTHER;
	}

	if ( ipaq_model_ops.icpr & IC_GPIO0 )
		reason |= REASON_BUTTON;

	r = &(s->reason_list[ s->count++ % MAX_REASONS ]);
	r->reason = reason;
	r->rcnr   = RCNR;
	r->saved_rtar = s->saved_rtar;

	return reason;
}


static int h3600_asic_check_wakeup( struct sleep_state *s )
{
	int reason = h3600_asic_wakeup_reason(s);
	u32 rtsr = RTSR;

	/* 
	   Sort out the RTC alarms.  If it was a user-set alarm, we don't want
	   to touch anything.  If it was a battery alarm, we need to acknowledge it.
	   No matter what the wake up reason, we shut off the battery RTC alarm
	   and restore any pending user-set alarms.
	*/

	if ( reason & REASON_RTC_OTHER )  // Better not touch the alarm structures
		return 0;

	if ( reason & REASON_RTC_BATTERY ) {    // Acknowledge the interrupt
		RTSR = 0;
		RTSR = RTSR_AL; 
	}

	if ( s->saved_rtar > RCNR ) {    // Re-enable the user-set RTC alarm
		RTAR = s->saved_rtar;
		rtsr |= RTSR_ALE;
	}
	else                           // Otherwise disable RTC alarms
		rtsr &= ~RTSR_ALE;

	RTSR = rtsr & (RTSR_ALE|RTSR_HZE);
		
	/* 
	   The RTC alarms have been sorted out.  Decide how to handle
	   the wakeup
	*/
	if ( reason & REASON_BUTTON || !reason )
		return 0;     // Wake up the iPAQ for either a button press or an unexplained event

	if ( reason & REASON_AC_IN )
		s->state = CHARGING_STATE_ACTIVE;

	if ( reason & REASON_RTC_BATTERY )
		s->state = ( H3800_ASIC2_ADCDR < 100 ) ? CHARGING_STATE_FULL : CHARGING_STATE_ACTIVE;

	if ( reason & REASON_AC_OUT )   // Note: it's possible to AC_OUT && RTC_BATTERY at same time
		s->state = CHARGING_STATE_NO_AC;

	h3600_asic_set_for_sleep(s);
	return 1;    // Don't wake up
}

static int h3600_asic_proc_wakeup_read(char *page, char **start, off_t off,
				       int count, int *eof, void *data )
{
	char *p = page;
	int i;
	struct sleep_state *s = &g_sleep;

	for ( i = ( s->count - MAX_REASONS < 0 ? 0 : s->count - MAX_REASONS ) ; i < s->count ; i++ ) {
		struct wakeup_reason *r = &s->reason_list[ i % MAX_REASONS ];
		p += sprintf(p, "[%03d] RCNR=%08x SAVED_RTAR=%08x  %02x ", i, r->rcnr, r->saved_rtar, r->reason);
		if ( r->reason & REASON_BUTTON ) 
			p += sprintf(p, "BUTTON ");
		if ( r->reason & REASON_RTC_BATTERY )
			p += sprintf(p, "RTC_BAT ");
		if ( r->reason & REASON_RTC_OTHER )
			p += sprintf(p, "RTC_OTHER ");
		if ( r->reason & REASON_AC_IN )
			p += sprintf(p, "AC_IN ");
		if ( r->reason & REASON_AC_OUT )
			p += sprintf(p, "AC_OUT ");
		p += sprintf(p, "\n");
	}

	*eof = 1;
	return (p - page);
}



static int h3600_asic_pm_callback(pm_request_t req)
{
	int result = 0;
        if (0) printk(__FUNCTION__ ": req=%d\n", req );

	switch (req) {
	case PM_SUSPEND:
		H3800_ASIC2_INTR_MaskAndFlag &= ~ASIC2_INTMASK_GLOBAL;
		result = h3600_asic_suspend_handlers();
		H3800_ASIC2_KPIINTSTAT  = 0;    
		H3800_ASIC2_GPIINTSTAT  = 0;
		g_sleep.count = 0;
		g_sleep.state = h3600_asic_battery_get_state();
		h3600_asic_set_for_sleep( &g_sleep );
		break;

	case PM_RESUME:
		MSC2 = (MSC2 & 0x0000ffff) | 0xE4510000;  /* Set MSC2 correctly */
		result = h3600_asic_check_wakeup( &g_sleep );
		if ( !result ) {
			/* These probably aren't necessary */
			H3800_ASIC2_KPIINTSTAT     =  0;     /* Disable all interrupts */
			H3800_ASIC2_GPIINTSTAT     =  0;

			H3800_ASIC2_KPIINTCLR      =  0xffff;     /* Clear all KPIO interrupts */
			H3800_ASIC2_GPIINTCLR      =  0xffff;     /* Clear all GPIO interrupts */

			H3800_ASIC2_CLOCK_Enable       |= ASIC2_CLOCK_EX0;   /* 32 kHZ crystal on */
			H3800_ASIC2_INTR_ClockPrescale |= ASIC2_INTCPS_SET;
			H3800_ASIC2_INTR_ClockPrescale  = ASIC2_INTCPS_CPS(0x0e) | ASIC2_INTCPS_SET;
			H3800_ASIC2_INTR_TimerSet       = 1;

			h3600_asic_resume_handlers();
			H3800_ASIC2_INTR_MaskAndFlag  |= ASIC2_INTMASK_GLOBAL; /* Turn on ASIC interrupts */
		}
		break;
	}
	return result;
}

static int h3600_asic_reset_handler(ctl_table *ctl, int write, struct file * filp,
				    void *buffer, size_t *lenp)
{
	MOD_INC_USE_COUNT;
	h3600_asic_pm_callback( PM_SUSPEND );
	h3600_asic_pm_callback( PM_RESUME );
	MOD_DEC_USE_COUNT;

	return 0;
}


/***********************************************************************************
 *   Initialization code
 ***********************************************************************************/

static struct h3600_hal_ops h3800_asic_ops = {
	get_version         : h3600_asic_version,
	eeprom_read         : h3600_asic_eeprom_read,
	eeprom_write        : h3600_asic_eeprom_write,
	get_thermal_sensor  : h3600_asic_thermal_sensor,
	set_notify_led      : h3600_asic_notify_led,
	read_light_sensor   : h3600_asic_read_light_sensor,
	get_battery         : h3600_asic_battery_read,
	spi_read            : h3600_asic_spi_read,
	spi_write           : h3600_asic_spi_write,
	get_option_detect   : h3600_asic_option_pack,
	audio_clock         : h3600_asic_audio_clock,
	audio_power         : h3600_asic_audio_power,
	audio_mute          : h3600_asic_audio_mute,
	backlight_control   : h3600_asic_backlight_control,
	asset_read          : h3600_asic_asset_read,
	set_ebat            : h3600_asic_spi_set_ebat,
        owner               : THIS_MODULE,
};

static void __exit h3600_asic_cleanup( void )
{
	h3600_unregister_pm_callback( h3600_asic_pm_callback );
	h3600_asic_unregister_procfs();
	h3600_hal_unregister_interface( &h3800_asic_ops );
	h3600_asic_cleanup_handlers();
	h3600_asic_release_isr();
}

int __init h3600_asic_init( void )
{
	int result;

	if ( !machine_is_h3800() ) {
		printk(__FUNCTION__ ": unknown iPAQ model %s\n", h3600_generic_name() );
		return -ENODEV;
	}

	/* We need to set nCS5 without changing nCS4 */
	/* Properly, this should be set in the bootldr */
	MSC2 = (MSC2 & 0x0000ffff) | 0xE4510000;

	result = h3600_asic_init_isr();  
	if ( result ) return result;

	result = h3600_asic_init_handlers();
	if ( result ) goto init_fail;

	result = h3600_hal_register_interface( &h3800_asic_ops );
	if ( result ) goto init_fail;

	result = h3600_asic_register_procfs();
	if ( result ) goto init_fail;

	h3600_register_pm_callback( h3600_asic_pm_callback );

	H3800_ASIC2_INTR_MaskAndFlag  |= ASIC2_INTMASK_GLOBAL; /* Turn on ASIC interrupts */
	return 0;

init_fail:
	printk(__FUNCTION__ ": FAILURE!  Exiting...\n");
	h3600_asic_cleanup();
	return result;
}

void __exit h3600_asic_exit( void )
{
	h3600_asic_cleanup();
	H3800_ASIC2_INTR_MaskAndFlag &= ~ASIC2_INTMASK_GLOBAL;
}

module_init(h3600_asic_init)
module_exit(h3600_asic_exit)
