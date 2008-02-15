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
#include <linux/proc_fs.h>

#include <asm/uaccess.h>   /* for copy to/from user space */
#include <asm/arch/hardware.h>
#include <asm/arch/irqs.h>
#include <asm/arch/h3600_hal.h>
#include <asm/arch/h3600_asic.h>

#define H3600_ASIC_PROC_DIR     "asic"
#define H3600_ASIC_PROC_STATS   "stats"
#define REG_DIRNAME "registers"

// Define this to see all suspend/resume init/cleanup messages
#define DEBUG_INIT()  \
        if (1) printk(__FUNCTION__ "\n")

struct asic_statistics {
	u32  isr;         /* Interrupts received */
	u32  key_isr;
	u32  pen_isr;     /* Pen interrupts */
	u32  adc_isr;     /* ADC interrupts */
	u32  spi_bytes;   /* Bytes handled by SPI */
	u32  spi_isr;     /* Interrupts received */
	u32  spi_timeout; /* Timeouts in transmission */
	u32  spi_wip;     /* Write in process */
	u32  kpio_int;    /* Interrupt enable shadow */
} g_statistics;

/***********************************************************************************
 *  Gunk
 * 
 *  The ASIC does not reliably read from H3800_ASIC2_KPIO_InterruptEnable
 *  Since we toggle the bits on and off repeatedly, we create a shadow register
 *  and only permit writing to the ASIC.
 ***********************************************************************************/

static void set_kpio_int( u32 x ) {
	g_statistics.kpio_int |= x;
	H3800_ASIC2_KPIINTSTAT = g_statistics.kpio_int;
}

static void clear_kpio_int( u32 x ) {
	g_statistics.kpio_int &= ~x;
	H3800_ASIC2_KPIINTSTAT = g_statistics.kpio_int;
}


/***********************************************************************************
 *      Shared resources                                                           
 *                                                                                 
 *      EX1 on Clock      24.576 MHz crystal (ADC,PCM,SPI,PWM,UART,SD,Audio)       
 ***********************************************************************************/

struct asic_shared {
	spinlock_t        lock;
	int               clock_ex1;    /* 24.765 MHz crystal */
	int               clock_ex2;    /* 33.8688 MHz crystal */
} g_shared;

enum ASIC_SHARED {
	ASIC_SHARED_CLOCK_EX1 = 1,  /* Bit fields */
	ASIC_SHARED_CLOCK_EX2 = 2
};

static void h3600_asic_shared_add( unsigned long *s, enum ASIC_SHARED v )
{
	unsigned long flags;
	if (0) printk(__FUNCTION__ " %lx %d\n",*s,v);

	if ( (*s & v) == v )    /* Already set */
		return;

	spin_lock_irqsave( &g_shared.lock, flags );

	if ( v & ASIC_SHARED_CLOCK_EX1 && !(*s & ASIC_SHARED_CLOCK_EX1)) {
		if ( !g_shared.clock_ex1++ ) {
			printk(__FUNCTION__ ": enabling EX1\n");
			H3800_ASIC2_CLOCK_Enable |= ASIC2_CLOCK_EX1;
		}
	}

	if ( v & ASIC_SHARED_CLOCK_EX2 && !(*s & ASIC_SHARED_CLOCK_EX2)) {
		if ( !g_shared.clock_ex2++ ) {
			printk(__FUNCTION__ ": enabling EX2\n");
			H3800_ASIC2_CLOCK_Enable |= ASIC2_CLOCK_EX2;
		}
	}

	spin_unlock_irqrestore( &g_shared.lock, flags );
	*s |= v;
}

static void h3600_asic_shared_release( unsigned long *s, enum ASIC_SHARED v )
{
	unsigned long flags;
	if (0) printk(__FUNCTION__ " %lx %d\n",*s,v);

	spin_lock_irqsave( &g_shared.lock, flags );

	if ( v & ASIC_SHARED_CLOCK_EX1 && !(~*s & ASIC_SHARED_CLOCK_EX1)) {
		if ( !--g_shared.clock_ex1 ) {
			printk(__FUNCTION__ ": disabling EX1\n");
			H3800_ASIC2_CLOCK_Enable &= ~ASIC2_CLOCK_EX1;
		}
	}

	if ( v & ASIC_SHARED_CLOCK_EX2 && !(~*s & ASIC_SHARED_CLOCK_EX2)) {
		if ( !--g_shared.clock_ex2 ) {
			printk(__FUNCTION__ ": disabling EX2\n");
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
 *      Keyboard IRQs
 *
 *   Resources used:     KPIO interface on ASIC2
 *                       GPIO for Power button on SA1110
 ***********************************************************************************/

#define MAKEKEY(index, down)  ((down) ? (index) : ((index) | 0x80))

static void h3600_asic_power_isr(int irq, void *dev_id, struct pt_regs *regs)
{
        int down = (GPLR & GPIO_H3600_NPOWER_BUTTON) ? 0 : 1;
        if (dev_id != h3600_asic_power_isr)
                return;

	h3600_hal_keypress( MAKEKEY( 11, down ) );
}

struct asic_to_button { 
	u32 mask;
	u32 button;
	u8  down;
};

static struct asic_to_button asic_to_button[] = {
	{ 0, 0 },      /* Null */
	{ KPIO_RECORD_BTN_N,   KPIO_RECORD_BTN_N },  // 1:  Record button
	{ KPIO_KEY_LEFT_N,     KPIO_KEY_LEFT_N },    // 2:  Calendar
	{ KPIO_KEY_RIGHT_N,    KPIO_KEY_RIGHT_N },   // 3:  Contacts (looks like Outlook)
	{ KPIO_KEY_AP1_N,      KPIO_KEY_AP1_N },     // 4:  Envelope (Q on older iPAQs)
	{ KPIO_KEY_AP2_N,      KPIO_KEY_AP2_N },     // 5:  Start (looks like swoopy arrow)
	{ KPIO_ALT_KEY_ALL,    KPIO_KEY_5W1_N | KPIO_KEY_5W4_N | KPIO_KEY_5W5_N }, // 6:  Up
	{ KPIO_KEY_AP4_N,      KPIO_KEY_AP4_N },     // 7:  Right
	{ KPIO_KEY_AP3_N,      KPIO_KEY_AP3_N },     // 8:  Left
	{ KPIO_ALT_KEY_ALL,    KPIO_KEY_5W2_N | KPIO_KEY_5W3_N | KPIO_KEY_5W5_N }, // 9:  Down
	{ KPIO_ALT_KEY_ALL,    KPIO_KEY_5W5_N },     // 10: Action
};

static void h3600_asic_key_irq( int irq )
{
	int i;
	u32 keys = H3800_ASIC2_KPIOPIOD;

	if (0) printk(__FUNCTION__ ": %x\n", keys);
	g_statistics.key_isr++;
	
	clear_kpio_int( KPIO_KEY_ALL );

	for ( i = 1 ; i < (sizeof(asic_to_button)/sizeof(struct asic_to_button)) ; i++ ) {
		int down = ((~keys & asic_to_button[i].mask) == asic_to_button[i].button) ? 1 : 0;
		if ( down != asic_to_button[i].down ) {
			h3600_hal_keypress(MAKEKEY(i,down));
			asic_to_button[i].down = down;
		}
	}

	H3800_ASIC2_KPIINTALSEL  = (KPIO_KEY_ALL & ~keys);
	set_kpio_int( KPIO_KEY_ALL );
}

static void h3600_asic_key_setup( void ) 
{
	H3800_ASIC2_KPIODIR        =  KPIO_KEY_ALL;  /* Inputs    */
	H3800_ASIC2_KPIOALT        =  KPIO_ALT_KEY_ALL;
	H3800_ASIC2_KPIINTTYPE    &= ~KPIO_KEY_ALL;  /* Level */
	H3800_ASIC2_KPIINTALSEL    =  (KPIO_KEY_ALL & ~H3800_ASIC2_KPIOPIOD);

	set_kpio_int( KPIO_KEY_ALL );
}

static int h3600_asic_key_suspend( void )
{
	DEBUG_INIT();
	clear_kpio_int( KPIO_KEY_ALL );
	return 0;
}

static void h3600_asic_key_resume( void )
{
	DEBUG_INIT();
	h3600_asic_key_setup();
}

static int __init h3600_asic_key_init( void )
{
	DEBUG_INIT();
	h3600_asic_key_setup();
	return 0;
}

static void __exit h3600_asic_key_cleanup( void )
{
	DEBUG_INIT();
	clear_kpio_int( KPIO_KEY_ALL );
}

/***********************************************************************************
 *      SPI interface
 *
 *   Resources used:     SPI interface on ASIC2
 *                       SPI Clock on CLOCK (CX5)
 *                       SPI KPIO interrupt line
 *   Shared resource:    EX1 (24.576 MHz crystal) on CLOCK
 ***********************************************************************************/

static struct h3600_asic_spidev {
	struct semaphore  lock;
	wait_queue_head_t waitq;
} g_spidev;

static void h3600_asic_spi_irq( int irq )
{
	if (0) printk(__FUNCTION__ "\n");

	g_statistics.spi_isr++;
	wake_up_interruptible( &g_spidev.waitq );
	H3800_ASIC2_SPI_Control &= ~SPI_CONTROL_SPIE;   /* Clear the interrupt */
}

/* 
 * This routine both transmits and receives data from the SPI bus.
 * It returns the byte read in the result code, or a negative number
 * if there was an error.
 */

static int h3600_asic_spi_process_byte( unsigned char data )
{
	wait_queue_t  wait;
	signed long   timeout;
	int           result = 0;

	if (0) printk(__FUNCTION__ ": sending=%x ", data);
	g_statistics.spi_bytes++;

	H3800_ASIC2_SPI_Control |= SPI_CONTROL_SPIE;     /* Enable interrupts */
	H3800_ASIC2_SPI_Data = data;                     /* Send data */
	H3800_ASIC2_SPI_Control |= SPI_CONTROL_SPE;      /* Start the transfer */

	/* We're basically using interruptible_sleep_on_timeout */
	/* and waiting for the transfer to finish */
	init_waitqueue_entry(&wait,current);
	add_wait_queue(&g_spidev.waitq, &wait);
	timeout = 100 * HZ / 1000;    /* 100 milliseconds (empirically derived) */

	while ( timeout > 0 ) {
		set_current_state( TASK_INTERRUPTIBLE );
		if ( !(H3800_ASIC2_SPI_Control & SPI_CONTROL_SPE )) {
			result = H3800_ASIC2_SPI_Data;
			break;
		}
		if ( signal_pending(current) ) {
			result = -ERESTARTSYS;
			break;
		}
		timeout = schedule_timeout( timeout );
		if ( timeout <= 0 ) {
			result = -ETIMEDOUT;       /* is this right? */
			g_statistics.spi_timeout++;
		}
	}
	current->state = TASK_RUNNING;
	remove_wait_queue(&g_spidev.waitq, &wait);

	H3800_ASIC2_SPI_Control &= ~SPI_CONTROL_SPIE;    /* Disable interrupts (may be timeout) */
	if (0) printk(" result=%d\n", result);
	return result;
}

/* 
   Read from a Microchip 25LC040 EEPROM tied to CS1
   This is the standard EEPROM part on all option packs
*/

#define SPI_25LC040_RDSR        0x05     /* Read status register */
#define SPI_25LC040_RDSR_WIP  (1<<0)     /* Write-in-process (1=true) */
#define SPI_25LC040_RDSR_WEL  (1<<1)     /* Write enable latch */
#define SPI_25LC040_RDSR_BP0  (1<<2)     /* Block protection bit */
#define SPI_25LC040_RDSR_BP1  (1<<3)     /* Block protection bit */

#define SPI_25LC040_READ_HIGH(addr)   (0x03 |((addr & 0x100)>>5))  /* Read command (put A8 in bit 3) */
#define SPI_25LC040_READ_LOW(addr)    (addr&0xff)                  /* Low 8 bits */

/* Wait until the EEPROM is finished writing */
static int h3600_asic_spi_eeprom_ready( void )
{
	int result;
	int i;

	if (0) printk(__FUNCTION__ "\n");

	H3800_ASIC2_SPI_ChipSelectDisabled = 0; /* Disable all chip selects */

	for ( i = 0 ; i < 20 ; i++ ) {
		if ( (result = h3600_asic_spi_process_byte( SPI_25LC040_RDSR )) < 0 )
			return result;

		if ( (result = h3600_asic_spi_process_byte( 0 )) < 0 )
			return result;

		/* Really should wait a bit before giving up */
		if (!(result & SPI_25LC040_RDSR_WIP) )
			return 0;

		g_statistics.spi_wip++;
	}
	return -ETIMEDOUT;
}

static int h3600_asic_spi_eeprom_read( unsigned short address, unsigned char *data, unsigned short len )
{
	int result = 0;
	int i;

	if (0) printk(__FUNCTION__ ": address=%x, len=%d, data=%p\n", address,len,data);

	H3800_ASIC2_SPI_ChipSelectDisabled = 0; /* Disable all chip selects */

	if ( (result = h3600_asic_spi_process_byte( SPI_25LC040_READ_HIGH(address) )) < 0 )
		return result;

	if ( (result = h3600_asic_spi_process_byte( SPI_25LC040_READ_LOW(address) )) < 0 )
		return result;

	for ( i = 0 ; i < len ; i++ ) {
		if ( (result = h3600_asic_spi_process_byte( 0 )) < 0 )
			return result;
		data[i] = result;
	}

	return 0;
}

static int h3600_asic_spi_read(unsigned short address, unsigned char *data, unsigned short len)
{
	int result = 0;
	unsigned long shared = 0;

	if (0) printk(__FUNCTION__ ": address=%x, len=%d, data=%p\n", address,len,data);

	if ( down_interruptible(&g_spidev.lock) )
		return -ERESTARTSYS;

	h3600_asic_shared_add( &shared, ASIC_SHARED_CLOCK_EX1 );

	H3800_ASIC2_SPI_Control = SPI_CONTROL_SPR(3) | SPI_CONTROL_SEL_CS0;
	H3800_ASIC2_CLOCK_Enable |= ASIC2_CLOCK_SPI;

	if ( (result = h3600_asic_spi_eeprom_ready()) < 0 )
		goto read_optionpaq_exit;

	if ( (result = h3600_asic_spi_eeprom_read( address, data, len )) < 0 )
		goto read_optionpaq_exit;

	result = 0;    /* Good return code */

read_optionpaq_exit:
	H3800_ASIC2_SPI_ChipSelectDisabled = 0;
	h3600_asic_shared_release( &shared, ASIC_SHARED_CLOCK_EX1 );

	up(&g_spidev.lock);
	return result;
}

static int h3600_asic_spi_suspend( void )
{
	DEBUG_INIT();
	down(&g_spidev.lock);   /* Grab the lock, no interruptions */
	clear_kpio_int( KPIO_SPI_INT );
	return 0;
}

static void h3600_asic_spi_resume( void )
{
	DEBUG_INIT();
	set_kpio_int( KPIO_SPI_INT );
	up(&g_spidev.lock);
}

static int __init h3600_asic_spi_init( void )
{
	DEBUG_INIT();
	init_waitqueue_head( &g_spidev.waitq );
	init_MUTEX( &g_spidev.lock );
	set_kpio_int( KPIO_SPI_INT );
	return 0;
}

static void __exit h3600_asic_spi_cleanup( void )
{
	DEBUG_INIT();
	clear_kpio_int( KPIO_SPI_INT );
}

/***********************************************************************************
 *   Backlight
 *
 *   Resources used:     PWM_0
 *                       PWM Clock enable on CLOCK (CX7)
 *                       GPIO pin on ASIC1 (for frontlight power)
 ***********************************************************************************/

static unsigned long backlight_shared;

static int h3600_asic_backlight_control( enum flite_pwr power, unsigned char level )
{
	printk(__FUNCTION__ " power=%d level=%d\n", power, level);
	switch (power) {
	case FLITE_PWR_OFF:
		H3800_ASIC1_GPIO_Out       &= ~GPIO_H3800_ASIC1_FL_PWR_ON;
		H3800_ASIC2_PWM_0_TimeBase &= ~PWM_TIMEBASE_ENABLE;
		H3800_ASIC2_CLOCK_Enable   &= ~ASIC2_CLOCK_PWM;
		h3600_asic_shared_release( &backlight_shared, ASIC_SHARED_CLOCK_EX1 );
		break;
	case FLITE_PWR_ON:
		h3600_asic_shared_add( &backlight_shared, ASIC_SHARED_CLOCK_EX1 );
		H3800_ASIC2_CLOCK_Enable    |= ASIC2_CLOCK_PWM;
		if ( level < 21 ) level = 21;
		if ( level > 64 ) level = 64;
		H3800_ASIC2_PWM_0_DutyTime = level;
		H3800_ASIC2_PWM_0_TimeBase |= PWM_TIMEBASE_ENABLE;
		H3800_ASIC1_GPIO_Out       |= GPIO_H3800_ASIC1_FL_PWR_ON;
		break;
	}
	return 0;
}

/* 
   The backlight should automatically be handled through suspend/resume
   by the framebuffer, so no special handling is necessary
 */

static int h3600_asic_backlight_init( void )
{
	DEBUG_INIT();
	H3800_ASIC2_PWM_0_TimeBase   = PWM_TIMEBASE_VALUE(8);
	H3800_ASIC2_PWM_0_PeriodTime = 0x40;
	return 0;
}

static void h3600_asic_backlight_cleanup( void )
{
	DEBUG_INIT();
	H3800_ASIC1_GPIO_Out       &= ~GPIO_H3800_ASIC1_FL_PWR_ON;
	H3800_ASIC2_PWM_0_TimeBase &= ~PWM_TIMEBASE_ENABLE;
	H3800_ASIC2_CLOCK_Enable   &= ~ASIC2_CLOCK_PWM;
	h3600_asic_shared_release( &backlight_shared, ASIC_SHARED_CLOCK_EX1 );
}

/***********************************************************************************
 *      Touchscreen
 *
 *   Resources used:     ADC 3 & 4
 *                       Clock: ADC (CX4)
 *                       ADC Interrupt on KPIO
 *                       Pen interrupt on GPIO2
 *
 *   Shared resources:   Clock: 24.576MHz crystal (EX1)
 ***********************************************************************************/

#define TSMASK  (H3800_ASIC2_GPIOPIOD & ~(GPIO2_IN_Y1_N | GPIO2_IN_X0 | GPIO2_IN_Y0 | GPIO2_IN_X1_N))

struct ts_sample {
	unsigned int   mask;
	unsigned short mux;
	char *         name;
};

/* TODO:  This is ugly, but good for debugging.
   In the future, we'd prefer to use one of the ASIC2 timers
   to toggle ADC sampling, rather than queueing a timer.
   We also should look at settling times for the screen.
*/

const struct ts_sample g_samples[] = {
	{ GPIO2_IN_X1_N | GPIO2_IN_Y0, ASIC2_ADMUX_3_TP_X0, "X" },    /* Measure X */
	{ GPIO2_IN_X1_N | GPIO2_IN_Y0, ASIC2_ADMUX_3_TP_X0, "X" },    /* Measure X */
	{ GPIO2_IN_X1_N | GPIO2_IN_Y0, ASIC2_ADMUX_3_TP_X0, "X" },    /* Measure X */
	{ GPIO2_IN_X1_N | GPIO2_IN_Y0, ASIC2_ADMUX_3_TP_X0, "X" },    /* Measure X */
	{ GPIO2_IN_X1_N | GPIO2_IN_Y0, ASIC2_ADMUX_4_TP_Y1, "XY" },  
	{ GPIO2_IN_Y1_N | GPIO2_IN_X0, ASIC2_ADMUX_4_TP_Y1, "Y" },    /* Measure Y */
	{ GPIO2_IN_Y1_N | GPIO2_IN_X0, ASIC2_ADMUX_4_TP_Y1, "Y" },    /* Measure Y */
	{ GPIO2_IN_Y1_N | GPIO2_IN_X0, ASIC2_ADMUX_4_TP_Y1, "Y" },    /* Measure Y */
	{ GPIO2_IN_Y1_N | GPIO2_IN_X0, ASIC2_ADMUX_4_TP_Y1, "Y" },    /* Measure Y */
	{ GPIO2_IN_Y1_N | GPIO2_IN_X0, ASIC2_ADMUX_3_TP_X0, "YX" },    /* Measure Y */
	{ GPIO2_IN_Y1_N | GPIO2_IN_Y0, ASIC2_ADMUX_3_TP_X0, "CX" },
	{ GPIO2_IN_Y1_N | GPIO2_IN_Y0, ASIC2_ADMUX_4_TP_Y1, "CY" },

	{ GPIO2_IN_Y1_N | GPIO2_IN_X0 | GPIO2_IN_X1_N, 0,   "End" },      /* Go to PEN_IRQ mode */
};

#define TS_SAMPLE_COUNT (sizeof(g_samples)/sizeof(struct ts_sample))

struct touchscreen_data {
	unsigned long          shared;   // Shared resources
	int                    samples[TS_SAMPLE_COUNT];
	int                    index;
	struct timer_list      timer;
} g_touch;

static int setup_sample( const struct ts_sample *s )
{
	H3800_ASIC2_GPIOPIOD = TSMASK | s->mask;

	if ( s->mux ) {
//		H3800_ASIC2_ADCSR     = ASIC2_ADCSR_ADPS(8) | ASIC2_ADCSR_FREE_RUN |
//			                            ASIC2_ADCSR_INT_ENABLE | ASIC2_ADCSR_ENABLE;
		H3800_ASIC2_ADMUX     = s->mux | ASIC2_ADMUX_CLKEN;
		H3800_ASIC2_ADCSR     = ASIC2_ADCSR_ADPS(4) | ASIC2_ADCSR_INT_ENABLE | ASIC2_ADCSR_ENABLE;
		set_kpio_int( KPIO_ADC_INT );
		H3800_ASIC2_ADCSR    |= ASIC2_ADCSR_START;
	}
	else {
		H3800_ASIC2_ADCSR &= ~ASIC2_ADCSR_INT_ENABLE; 
	}
	
	return s->mux;
}

static void h3600_asic_timer_callback( unsigned long nr )
{
//	int i;
//	for ( i = 0 ; i < TS_SAMPLE_COUNT ; i++ )
//		printk(" %s:0x%04X", g_samples[i].name, g_touch.samples[i]);

	/* Check to see if we're still down */
	if ( H3800_ASIC2_GPIOPIOD & GPIO2_PEN_IRQ ) {
		h3600_asic_shared_release( &g_touch.shared, ASIC_SHARED_CLOCK_EX1 );
		h3600_hal_touchpanel(0,0,0);
		// printk(" **** Up\n");
		H3800_ASIC2_GPIINTSTAT |= GPIO2_PEN_IRQ;
	} else {
		// printk(" **** Down\n");
		h3600_hal_touchpanel( g_touch.samples[3], g_touch.samples[8], 1 );
		g_touch.index = 0;
		setup_sample(&g_samples[0]);
	}
}

#define ASIC_ADC_DELAY  10  /* Delay 10 milliseconds */

static void h3600_asic_adc_irq( int irq )
{
	int data = H3800_ASIC2_ADCDR;
	//printk(__FUNCTION__ " ADC Control Status 0x%08X\n", H3800_ASIC2_ADCSR );
	g_statistics.adc_isr++;
	
	H3800_ASIC2_ADCSR &= ~ASIC2_ADCSR_INT_ENABLE;   /* Disable the interrupt */
	clear_kpio_int( KPIO_ADC_INT );
	H3800_ASIC2_KPIINTCLR = KPIO_ADC_INT;           /* Clear the interrupt */

	g_touch.samples[g_touch.index++] = data;
	if ( !setup_sample(&g_samples[g_touch.index])) 
		mod_timer(&g_touch.timer, jiffies + (ASIC_ADC_DELAY * HZ) / 1000);
}

static void h3600_asic_pen_irq( int irq )
{
//	printk(__FUNCTION__" 0x%08X\n", H3800_ASIC2_ADMUX );
	g_statistics.pen_isr++;
	H3800_ASIC2_GPIINTSTAT &= ~GPIO2_PEN_IRQ;  /* Mask pen interrupts */

	h3600_asic_shared_add( &g_touch.shared, ASIC_SHARED_CLOCK_EX1 );
	g_touch.index = 0;
	setup_sample( &g_samples[0] );
}


static void h3600_asic_touchscreen_down( void )
{
	/* Turn off all interrupt sources */
	del_timer(&g_touch.timer);
	H3800_ASIC2_GPIINTSTAT &= ~GPIO2_PEN_IRQ;
	clear_kpio_int( KPIO_ADC_INT );

	H3800_ASIC2_GPIOPIOD    = TSMASK | GPIO2_IN_Y1_N | GPIO2_IN_X1_N; /* Normal off state */
	h3600_asic_shared_release( &g_touch.shared, ASIC_SHARED_CLOCK_EX1 );
}

static void h3600_asic_touchscreen_up( void )
{
	H3800_ASIC2_GPIOPIOD    = TSMASK | GPIO2_IN_Y1_N | GPIO2_IN_X0 | GPIO2_IN_X1_N;
	H3800_ASIC2_GPIINTSTAT |= GPIO2_PEN_IRQ;
}

static int h3600_asic_touchscreen_suspend( void )
{
	DEBUG_INIT();
	h3600_asic_touchscreen_down();
	return 0;
}

static void h3600_asic_touchscreen_resume( void )
{
	DEBUG_INIT();
	h3600_asic_touchscreen_up();
}

static int h3600_asic_touchscreen_init( void )
{
	DEBUG_INIT();

	init_timer(&g_touch.timer);
	g_touch.timer.function = h3600_asic_timer_callback;
	g_touch.timer.data     = (unsigned long) NULL;

	H3800_ASIC2_CLOCK_Enable         |= ASIC2_CLOCK_ADC;
	h3600_asic_touchscreen_up();
	return 0;
}

static void h3600_asic_touchscreen_cleanup( void )
{
	DEBUG_INIT();

	h3600_asic_touchscreen_down();  /* Shut everything down */
	H3800_ASIC2_CLOCK_Enable         &= ~ASIC2_CLOCK_ADC;
}

/***********************************************************************************
 *      Generic IRQ handling
 ***********************************************************************************/

#define MAX_ASIC_ISR_LOOPS    20

struct asic_irq_handler {
	u32    mask;
	void (*handler)( int irq );
};

const struct asic_irq_handler kpio_irq_handlers[] = {
	{ KPIO_KEY_ALL,     h3600_asic_key_irq },
	{ KPIO_SPI_INT,     h3600_asic_spi_irq },
	{ KPIO_ADC_INT,     h3600_asic_adc_irq },
	{ 0, NULL }
};

const struct asic_irq_handler gpio2_irq_handlers[] = {
	{ GPIO2_PEN_IRQ,    h3600_asic_pen_irq },
	{ 0, NULL }
};

static void h3600_asic_isr(int irq, void *dev_id, struct pt_regs *regs)
{
	int i = 0;

        if (dev_id != h3600_asic_isr)
                return;

	if (0) printk(__FUNCTION__ ": interrupt received\n");

	g_statistics.isr++;

	for ( i = 0 ; i < MAX_ASIC_ISR_LOOPS && (GPLR & GPIO_H3800_ASIC) ; i++ ) {
		u32 irq;
		const struct asic_irq_handler *h;
		
		/* KPIO */
		irq = H3800_ASIC2_KPIINTFLAG;
		if (0) printk(__FUNCTION__" KPIO 0x%08X\n", irq );
		for ( h = kpio_irq_handlers; h->handler != NULL ; h++ )
			if ( irq & h->mask )
				h->handler( irq );

		/* GPIO2 */
		irq = H3800_ASIC2_GPIINTFLAG;
		if (0) printk(__FUNCTION__" GPIO 0x%08X\n", irq );
		for ( h = gpio2_irq_handlers; h->handler != NULL ; h++ )
			if ( irq & h->mask )
				h->handler( irq );
	}

	if ( i >= MAX_ASIC_ISR_LOOPS )
		printk(__FUNCTION__ ": interrupt processing overrun\n");
}

static int __init h3600_asic_init_irq( void )
{
	int result;
	DEBUG_INIT();

	H3800_ASIC2_KPIINTSTAT     =  0;     /* Disable all interrupts */
	H3800_ASIC2_GPIINTSTAT     =  0;

	H3800_ASIC2_KPIINTCLR      =  0;     /* Clear all KPIO interrupts */
	H3800_ASIC2_GPIINTCLR      =  0;     /* Clear all GPIO interrupts */

	H3800_ASIC2_CLOCK_Enable       |= ASIC2_CLOCK_EX0;   /* 32 kHZ crystal on */
	H3800_ASIC2_INTR_ClockPrescale |= ASIC2_INTCPS_SET;
	H3800_ASIC2_INTR_ClockPrescale  = ASIC2_INTCPS_CPS(0x0e) | ASIC2_INTCPS_SET;
	H3800_ASIC2_INTR_TimerSet       = 1;

	/* Request IRQs */
        set_GPIO_IRQ_edge( GPIO_H3600_NPOWER_BUTTON, GPIO_BOTH_EDGES );
	result = request_irq(IRQ_GPIO_H3600_NPOWER_BUTTON, 
			     h3600_asic_power_isr, 
			     SA_SHIRQ | SA_INTERRUPT | SA_SAMPLE_RANDOM,
			     "h3600_suspend", h3600_asic_power_isr);

	if ( result ) {
		printk(KERN_CRIT __FUNCTION__ ": unable to grab power button IRQ\n");
		return result;
	}

	set_GPIO_IRQ_edge( GPIO_H3800_ASIC, GPIO_RISING_EDGE );
	result = request_irq(IRQ_GPIO_H3800_ASIC,
			     h3600_asic_isr,
			     SA_SHIRQ | SA_INTERRUPT | SA_SAMPLE_RANDOM,
			     "h3600_asic", h3600_asic_isr );
	if ( result ) {
		printk(KERN_CRIT __FUNCTION__ ": unable to grab ASIC IRQ\n");
		free_irq(IRQ_GPIO_H3600_NPOWER_BUTTON, h3600_asic_power_isr);
		return result;
	}

	return 0;
}

static void __exit h3600_asic_release_irq( void )
{
	DEBUG_INIT();

	H3800_ASIC2_KPIINTSTAT  = 0;    
	H3800_ASIC2_GPIINTSTAT  = 0;

	free_irq(IRQ_GPIO_H3600_NPOWER_BUTTON, h3600_asic_power_isr);
	free_irq(IRQ_GPIO_H3800_ASIC, h3600_asic_isr);
}


/***********************************************************************************/
/*      Standard entry points for HAL requests                                     */
/***********************************************************************************/

static int h3600_asic_version( struct h3600_ts_version *result )
{
	return -EIO;
}

static int h3600_asic_eeprom_read( unsigned short address, unsigned char *data, unsigned short len )
{
	if (0) printk(__FUNCTION__ ": address=%d len=%d\n", address, len);
        return -EIO;
}

static int h3600_asic_eeprom_write( unsigned short address, unsigned char *data, unsigned short len )
{
        return -EIO;
}

static int h3600_asic_thermal_sensor( unsigned short *result )
{
	return -EIO;
}

static int h3600_asic_notify_led( unsigned char mode, unsigned char duration, 
			    unsigned char ontime, unsigned char offtime )
{
	return -EIO;
}

static int h3600_asic_battery( struct h3600_battery *result )
{
	return -EIO;
}

static int h3600_asic_spi_write(unsigned short address, unsigned char *data, unsigned short len)
{
	return -EIO;
}

static int h3600_asic_read_light_sensor( unsigned char *result )
{
	return -EIO;
}

/***********************************************************************************
 *   Proc filesystem interface
 ***********************************************************************************/

static struct proc_dir_entry   *asic_proc_dir;
static struct proc_dir_entry   *reg_proc_dir;

#define PRINT_DATA(x,s) \
	p += sprintf (p, "%-28s : %d\n", s, g_statistics.x)
#define PRINT_DATAX(x,s) \
	p += sprintf (p, "%-28s : 0x%08X\n", s, g_statistics.x)

static int h3600_asic_proc_stats_read(char *page, char **start, off_t off,
				      int count, int *eof, void *data)
{
	char *p = page;
	int len;

	PRINT_DATA(isr,         "Interrupts");
	PRINT_DATA(key_isr,     "Keyboard interrupts");
	PRINT_DATA(pen_isr,     "Pen interrupts");
	PRINT_DATA(adc_isr,     "ADC interrupts");
	PRINT_DATA(spi_isr,     "SPI interrupts");
	PRINT_DATA(spi_bytes,   "SPI bytes sent/received");
	PRINT_DATA(spi_wip,     "SPI write-in-process delays");
	PRINT_DATA(spi_timeout, "SPI timeouts");
	PRINT_DATAX(kpio_int,   "KPIO Interrupt flag");
	p += sprintf(p, "%-28s : %d\n", "EX1 usage", g_shared.clock_ex1 );
	p += sprintf(p, "%-28s : %d\n", "EX2 usage", g_shared.clock_ex2 );

	len = (p - page) - off;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
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
	u32   phyaddr;
	char* name;
	char* description;
	unsigned short low_ino;
} asic_reg_entry_t;

static asic_reg_entry_t asic_regs[] =
{
/*	{ virt_addr,   name,     description } */
	{ 0x0000, "GPIODIR",    "GPIO Input/Output direction register" },
	{ 0x0004, "GPIINTTYPE", "GPI Interrupt Type (Edge/Level)"},
	{ 0x0008, "GPIINTESEL", "GPI Interrupt active edge select"},
	{ 0x000c, "GPIINTALSEL","GPI Interrupt active level select"},
	{ 0x0010, "GPIINTFLAG", "GPI Interrupt active flag" },
	{ 0x0014, "GPIOPIOD",   "GPIO Port input/output data" },
	{ 0x0018, "GPOBFSTAT",  "GPO output data in batt_fault" },
	{ 0x001c, "GPIINTSTAT", "GPI Interrupt status" },
	{ 0x003c, "GPIOALT",    "GPIO ALTernate function" },

	{ 0x0200, "KPIODIR",    "KPIO Input/output direction"},
	{ 0x0204, "KPIINTTYP",  "KPI interrupt type (edge/level)" },
	{ 0x0208, "KPIINTESEL", "KPI Interrupt active edge select" },
	{ 0x020c, "KPIINTALSEL","KPI Interrupt active level select" },
	{ 0x0210, "KPIINTFLAG", "KPI Interrupt active flag" },
	{ 0x0214, "KPIOPIOD",   "KPIO Port input/output data" },
	{ 0x0218, "KPOBFSTAT",  "KPO Ouput data in batt_fault status" },
	{ 0x021c, "KPIINTSTAT", "KPI Interrupt status" },
	{ 0x023c, "KALT",       "KIU alternate function" },

	{ 0x0400, "SPICR",      "SPI control register" },
	{ 0x0404, "SPIDR",      "SPI data register" },
	{ 0x0408, "SPIDCS",     "SPI chip select disabled register" },

	{ 0x0600, "PWM0_TBS",   "PWM Time base set register" },
	{ 0x0604, "PWM0_PTS",   "PWM Period time set register" },
	{ 0x0608, "PWM0_DTS",   "PWM Duty time set register" },

	{ 0x0700, "PWM1_TBS",   "PWM Time base set register" },
	{ 0x0704, "PWM1_PTS",   "PWM Period time set register" },
	{ 0x0708, "PWM1_DTS",   "PWM Duty time set register" },

	{ 0x0800, "LED0_TBS",   "LED time base set register" },
	{ 0x0804, "LED0_PTS",   "LED period time set register" },
	{ 0x0808, "LED0_DTS",   "LED duty time set register" },
	{ 0x080c, "LED0_ASTC",  "LED auto stop counter register" },

	{ 0x0880, "LED1_TBS",   "LED time base set register" },
	{ 0x0884, "LED1_PTS",   "LED period time set register" },
	{ 0x0888, "LED1_DTS",   "LED duty time set register" },
	{ 0x088c, "LED1_ASTC",  "LED auto stop counter register" },

	{ 0x0900, "LED2_TBS",   "LED time base set register" },
	{ 0x0904, "LED2_PTS",   "LED period time set register" },
	{ 0x0908, "LED2_DTS",   "LED duty time set register" },
	{ 0x090c, "LED2_ASTC",  "LED auto stop counter register" },

	{ 0x0a00, "U0_BUF",     "Receive/transmit buffer"},

	{ 0x0c00, "U1_BUF",     "Receive/transmit buffer"},

	{ 0x0e00, "TIMER_0",    "Timer counter 0 register" },
	{ 0x0e04, "TIMER_1",    "Timer counter 1 register" },
	{ 0x0e08, "TIMER_2",    "Timer counter 2 register" },
	{ 0x0e0a, "TIMER_CNTL", "Timer control register (write only)" },
	{ 0x0e10, "TIMER_CMD",  "Timer command register" },

	{ 0x1000, "CDEX",       "Crystal source, control clock" },

	{ 0x1200, "ADMUX",      "ADC multiplixer select register" },
	{ 0x1204, "ADCSR",      "ADC control and status register" },
	{ 0x1208, "ADCDR",      "ADC data register" },
	
	{ 0x1600, "INTMASK",    "Interrupt mask control & cold boot flag" },
	{ 0x1604, "INTCPS",     "Interrupt timer clock pre-scale" },
	{ 0x1608, "INTTBS",     "Interrupt timer set" },

	{ 0x1800, "OWM_CMD",    "OWM command register" },
	{ 0x1804, "OWM_DATA",   "OWM transmit/receive buffer" },
	{ 0x1808, "OWM_INT",    "OWM interrupt register" },
	{ 0x180c, "OWM_INTEN",  "OWM interrupt enable register" },
	{ 0x1810, "OWM_CLKDIV", "OWM clock divisor register" },

	{ 0x1a00, "SSETR",      "Size of flash memory setting register" },

	{ 0x1f00, "FLASHWP",    "Flash write protect" },
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

	count = sprintf(outputbuf, "0x%08X\n", *((volatile u32 *)(_H3800_ASIC2_Base + current_reg->phyaddr)));
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
	*((volatile u32 *)(_H3800_ASIC2_Base + current_reg->phyaddr))=newRegValue;
	return (count+endp-buffer);
}

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
	return 0;
}

static void __exit h3600_asic_unregister_procfs( void )
{
	int i;
	if ( asic_proc_dir ) {
		for(i=0;i<NUM_OF_ASIC_REG_ENTRY;i++)
			remove_proc_entry(asic_regs[i].name,reg_proc_dir);
		remove_proc_entry(REG_DIRNAME, asic_proc_dir);

		remove_proc_entry(H3600_ASIC_PROC_STATS, asic_proc_dir);
		remove_proc_entry(H3600_ASIC_PROC_DIR, NULL );
		asic_proc_dir = NULL;
	}
}

/***********************************************************************************
 *   Utility routines
 ***********************************************************************************/

struct asic_system_handler { 
	int  (*init)( void );
	void (*cleanup)( void );
	int  (*suspend)( void );
	void (*resume)( void );
};

const struct asic_system_handler asic_system_handlers[] = {   /* Order matters! */
	{ 
		init:    h3600_asic_shared_init,      
		cleanup: h3600_asic_shared_cleanup 
	},{ 
		init:    h3600_asic_key_init,
		cleanup: h3600_asic_key_cleanup,
		suspend: h3600_asic_key_suspend,
		resume:  h3600_asic_key_resume 
	},{ 
		init:    h3600_asic_spi_init,
		cleanup: h3600_asic_spi_cleanup,
		suspend: h3600_asic_spi_suspend,
		resume:  h3600_asic_spi_resume 
	},{ 
		init:    h3600_asic_backlight_init,   
		cleanup: h3600_asic_backlight_cleanup 
	},{ 
		init:    h3600_asic_touchscreen_init, 
		cleanup: h3600_asic_touchscreen_cleanup,
		suspend: h3600_asic_touchscreen_suspend, 
		resume:  h3600_asic_touchscreen_resume 
	}
};

#define SYS_HANDLER_SIZE   (sizeof(asic_system_handlers)/sizeof(struct asic_system_handler))

static int h3600_asic_suspend_handlers( void )
{
	int i;
	int result;

	for ( i = 0 ; i < SYS_HANDLER_SIZE ; i++ ) {
		if ( asic_system_handlers[i].suspend ) {
			if ( (result = asic_system_handlers[i].suspend()) != 0 ) {
				while ( --i >= 0 )
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

	for ( i = SYS_HANDLER_SIZE - 1 ; i >= 0 ; i-- )
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
 ***********************************************************************************/

static int h3600_asic_pm_callback(pm_request_t req)
{
	int result = 0;
        if (1) printk(__FUNCTION__ ": req=%d\n", req );

	switch (req) {
	case PM_SUSPEND:
		result = h3600_asic_suspend_handlers();
		break;
	case PM_RESUME:
		h3600_asic_resume_handlers();
		break;
	}
	return result;
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
	get_battery         : h3600_asic_battery,
	spi_read            : h3600_asic_spi_read,
	spi_write           : h3600_asic_spi_write,
	backlight_control   : h3600_asic_backlight_control,
        owner               : THIS_MODULE,
};

static void __exit h3600_asic_cleanup( void )
{
	h3600_unregister_pm_callback( h3600_asic_pm_callback );
	h3600_asic_unregister_procfs();
	h3600_hal_unregister_interface( &h3800_asic_ops );
	h3600_asic_cleanup_handlers();
	h3600_asic_release_irq();
}

int __init h3600_asic_init( void )
{
	int result;

	if ( !machine_is_h3800() ) {
		printk(__FUNCTION__ ": unknown iPAQ model %s\n", h3600_generic_name() );
		return -ENODEV;
	}

	/* TODO: This is ugly...and conflicts with h3600_backpaq */
	/* Properly, this should be set in the bootldr */
	MSC2 = 0xE451FFFC;

	result = h3600_asic_init_irq();  
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
