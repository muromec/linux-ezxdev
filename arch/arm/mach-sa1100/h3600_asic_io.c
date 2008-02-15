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
*
* Restrutured June 2002
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
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/serial.h>  /* For bluetooth */

#include <asm/irq.h>
#include <asm/uaccess.h>   /* for copy to/from user space */
#include <asm/arch/hardware.h>
#include <asm/arch/h3600_hal.h>
#include <asm/arch/h3600_asic.h>
#include <asm/arch/serial_h3800.h>

#include "h3600_asic_io.h"
#include "h3600_asic_core.h"

/***********************************************************************************
 *      Sleeve ISR
 *
 *   Resources used:     KPIO interface on ASIC2
 *                       GPIO for Power button on SA1110
 ***********************************************************************************/

void h3600_asic_sleeve_isr(int irq, void *dev_id, struct pt_regs *regs)
{
        int present = (GPLR & GPIO_H3800_NOPT_IND) ? 0 : 1;
	h3600_hal_option_detect( present );
        GEDR = GPIO_H3800_NOPT_IND;    /* Clear the interrupt */
}


/***********************************************************************************
 *      Keyboard ISRs
 *
 *   Resources used:     KPIO interface on ASIC2
 *                       GPIO for Power button on SA1110
 ***********************************************************************************/

#define MAKEKEY(index, down)  ((down) ? (index) : ((index) | 0x80))

void h3600_asic_power_isr(int irq, void *dev_id, struct pt_regs *regs)
{
        int down = (GPLR & GPIO_H3600_NPOWER_BUTTON) ? 0 : 1;
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

static void h3600_asic_key_isr( int irq, void *dev_id, struct pt_regs *regs )
{
	int i;
	u32 keys = H3800_ASIC2_KPIOPIOD;

	if (0) printk(__FUNCTION__ ": %x\n", keys);
	
	for ( i = 1 ; i < (sizeof(asic_to_button)/sizeof(struct asic_to_button)) ; i++ ) {
		int down = ((~keys & asic_to_button[i].mask) == asic_to_button[i].button) ? 1 : 0;
		if ( down != asic_to_button[i].down ) {
			h3600_hal_keypress(MAKEKEY(i,down));
			asic_to_button[i].down = down;
		}
	}

	H3800_ASIC2_KPIINTALSEL  = (KPIO_KEY_ALL & ~keys);
}

static void h3600_asic_key_setup( void ) 
{
	H3800_ASIC2_KPIODIR        =  KPIO_KEY_ALL;  /* Inputs    */
	H3800_ASIC2_KPIOALT        =  KPIO_ALT_KEY_ALL;
	H3800_ASIC2_KPIINTTYPE    &= ~KPIO_KEY_ALL;  /* Level */
	H3800_ASIC2_KPIINTALSEL    =  (KPIO_KEY_ALL & ~H3800_ASIC2_KPIOPIOD);
}

int h3600_asic_key_suspend( void )
{
	DEBUG_INIT();
	disable_irq( IRQ_H3800_KEY );
	return 0;
}

void h3600_asic_key_resume( void )
{
	DEBUG_INIT();
	h3600_asic_key_setup();
	enable_irq( IRQ_H3800_KEY );
}

int __init h3600_asic_key_init( void )
{
	int result;

	DEBUG_INIT();
	h3600_asic_key_setup();

	result = request_irq(IRQ_H3800_KEY, h3600_asic_key_isr, 
			     SA_INTERRUPT | SA_SAMPLE_RANDOM,
			     "h3800_keyboard", NULL );

	if ( result )
		printk(KERN_CRIT __FUNCTION__ ": unable to grab keyboard virtual IRQ\n");
	return result;
}

void __exit h3600_asic_key_cleanup( void )
{
	DEBUG_INIT();
	free_irq( IRQ_H3800_KEY, NULL );
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

static void h3600_asic_spi_isr( int irq, void *dev_id, struct pt_regs *regs )
{
	if (0) printk(__FUNCTION__ "\n");

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

	if (0) printk(__FUNCTION__ ": sending=0x%02x ", data);
	g_h3600_asic_statistics.spi_bytes++;

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
			g_h3600_asic_statistics.spi_timeout++;
		}
	}
	set_current_state( TASK_RUNNING );
	remove_wait_queue(&g_spidev.waitq, &wait);

	H3800_ASIC2_SPI_Control &= ~SPI_CONTROL_SPIE;    /* Disable interrupts (may be timeout) */
	if (0) printk(" result=0x%02x\n", result);
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

		g_h3600_asic_statistics.spi_wip++;
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

int h3600_asic_spi_read(unsigned short address, unsigned char *data, unsigned short len)
{
	int result;
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

/* 
   Read from the PCMCIA option jacket microcontroller
*/

#define SPI_PCMCIA_HEADER          0xA1     /* STX */
#define	SPI_PCMCIA_ID              0x10
#define SPI_PCMCIA_ID_RESULT       0x13     /* 3 bytes : x.xx */
#define SPI_PCMCIA_BATTERY         0x20
#define SPI_PCMCIA_BATTERY_RESULT  0x24     /* 4 bytes : chem percent flag voltage */
#define SPI_PCMCIA_EBAT_ON         0x30     /* No result */

#define SPI_WRITE(_x) do { int _result = h3600_asic_spi_process_byte(_x); \
                           if (_result < 0) return _result; } while (0)
#define SPI_READ(_x)  do { _x = h3600_asic_spi_process_byte(0); \
                           if (_x < 0) return _x; } while (0)

static int h3600_asic_spi_pcmcia_read( unsigned char cmd, unsigned char reply, unsigned char *data )
{
	unsigned char checksum;
	int result;
	int i;

	if (0) printk(__FUNCTION__ ": cmd=%d reply=%d\n", cmd, reply);
	/* Send message */
	SPI_WRITE( SPI_PCMCIA_HEADER );
	SPI_WRITE( cmd );
	SPI_WRITE( cmd );  /* The checksum */

	if ( !data )
		return 0;
	
	/* Pause for a jiffie to give the micro time to respond */
	set_current_state( TASK_INTERRUPTIBLE );
	schedule_timeout(1);
	set_current_state( TASK_RUNNING );

	/* Read message here */
	SPI_READ( result );
	if ( result != SPI_PCMCIA_HEADER )
		return -EIO;

	SPI_READ( result );
	if ( result != reply )
		return -EIO;
	checksum = result;

	for ( i = 0 ; i < ( reply & 0x0f ) ; i++ ) {
		SPI_READ( result );
		data[i] = result;
		checksum += result;
	}
	
	SPI_READ( result );
	if ( checksum != result )
		return -EIO;

	return 0;
}

int h3600_asic_spi_read_pcmcia_battery( unsigned char *chem, unsigned char *percent, unsigned char *flag )
{
	int result;
	unsigned long shared = 0;
	unsigned char data[4];

	if ( down_interruptible(&g_spidev.lock) )
		return -ERESTARTSYS;

	h3600_asic_shared_add( &shared, ASIC_SHARED_CLOCK_EX1 );

	H3800_ASIC2_SPI_Control = SPI_CONTROL_SPR(2) | SPI_CONTROL_SEL_CS1;
	H3800_ASIC2_CLOCK_Enable |= ASIC2_CLOCK_SPI;

	if ( (result = h3600_asic_spi_pcmcia_read( SPI_PCMCIA_BATTERY, SPI_PCMCIA_BATTERY_RESULT, data )) < 0 )
		goto read_optionpaq_exit;

	result = 0;    /* Good return code */
	*chem    = data[0];
	*percent = data[1];
	*flag    = data[2];

read_optionpaq_exit:
	H3800_ASIC2_SPI_ChipSelectDisabled = 0;
	h3600_asic_shared_release( &shared, ASIC_SHARED_CLOCK_EX1 );

	up(&g_spidev.lock);
	return result;
}

int h3600_asic_spi_set_ebat( void )
{
	int result;
	unsigned long shared = 0;

	if (1) printk(__FUNCTION__ "\n");

	if ( down_interruptible(&g_spidev.lock) )
		return -ERESTARTSYS;

	h3600_asic_shared_add( &shared, ASIC_SHARED_CLOCK_EX1 );

	H3800_ASIC2_SPI_Control = SPI_CONTROL_SPR(2) | SPI_CONTROL_SEL_CS1;
	H3800_ASIC2_CLOCK_Enable |= ASIC2_CLOCK_SPI;

	result = h3600_asic_spi_pcmcia_read( SPI_PCMCIA_EBAT_ON, 0, NULL );

	H3800_ASIC2_SPI_ChipSelectDisabled = 0;
	h3600_asic_shared_release( &shared, ASIC_SHARED_CLOCK_EX1 );

	up(&g_spidev.lock);
	return result;
}

int h3600_asic_spi_suspend( void )
{
	DEBUG_INIT();
	down(&g_spidev.lock);   // Grab the lock, no interruptions
	disable_irq( IRQ_H3800_SPI );
	return 0;
}

void h3600_asic_spi_resume( void )
{
	DEBUG_INIT();
	enable_irq( IRQ_H3800_SPI );
	up(&g_spidev.lock);
}

int __init h3600_asic_spi_init( void )
{
	int result;
	DEBUG_INIT();
	init_waitqueue_head( &g_spidev.waitq );
	init_MUTEX( &g_spidev.lock );

	result = request_irq(IRQ_H3800_SPI, h3600_asic_spi_isr, 
			     SA_INTERRUPT | SA_SAMPLE_RANDOM,
			     "h3800_spi", NULL );
	if ( result )
		printk(KERN_CRIT __FUNCTION__ ": unable to grab SPI virtual IRQ\n");
	return result;
}

void __exit h3600_asic_spi_cleanup( void )
{
	DEBUG_INIT();
	down(&g_spidev.lock);   // Grab the lock, no interruptions
	free_irq( IRQ_H3800_SPI, NULL );
}


/***********************************************************************************
 *   Backlight
 *
 *   Resources used:     PWM_0
 *                       PWM Clock enable on CLOCK (CX7)
 *                       GPIO pin on ASIC1 (for frontlight power)
 ***********************************************************************************/

static unsigned long backlight_shared;

int h3600_asic_backlight_control( enum flite_pwr power, unsigned char level )
{
	if (0) printk(__FUNCTION__ " power=%d level=%d\n", power, level);
	switch (power) {
	case FLITE_PWR_OFF:
		H3800_ASIC1_GPIO_OUT       &= ~GPIO1_FL_PWR_ON;
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
		H3800_ASIC1_GPIO_OUT       |= GPIO1_FL_PWR_ON;
		break;
	}
	return 0;
}

/* 
   The backlight should automatically be handled through suspend/resume
   by the framebuffer, so no special handling is necessary
 */

int h3600_asic_backlight_init( void )
{
	DEBUG_INIT();
	H3800_ASIC2_PWM_0_TimeBase   = PWM_TIMEBASE_VALUE(8);
	H3800_ASIC2_PWM_0_PeriodTime = 0x40;
	return 0;
}

void h3600_asic_backlight_cleanup( void )
{
	DEBUG_INIT();
	H3800_ASIC1_GPIO_OUT       &= ~GPIO1_FL_PWR_ON;
	H3800_ASIC2_PWM_0_TimeBase &= ~PWM_TIMEBASE_ENABLE;
	H3800_ASIC2_CLOCK_Enable   &= ~ASIC2_CLOCK_PWM;
	h3600_asic_shared_release( &backlight_shared, ASIC_SHARED_CLOCK_EX1 );
}


/***********************************************************************************
 *      ADC - Shared resource
 *
 *   Resources used:     ADC 3 & 4
 *                       Clock: ADC (CX4)
 *                       ADC Interrupt on KPIO
 *
 *   Shared resources:   Clock: 24.576MHz crystal (EX1)
 *
 *   The ADC is multiplexed between the touchscreen, battery charger, and light sensor
 ***********************************************************************************/

enum adc_state {
	ADC_STATE_IDLE,
	ADC_STATE_TOUCHSCREEN,  // Servicing the touchscreen
	ADC_STATE_USER          // Servicing a user-level request
};

struct adc_data {
	enum adc_state     state;
	struct semaphore   lock;      // Mutex for access from user-level
	wait_queue_head_t  waitq;     // Waitq for user-level access (waits for interrupt service)
	int                last;      // Return value for user-level acces
	int                user_mux;  // Requested mux for the next user read
	int                ts_mux;    // Requested mux for the next touchscreen read
	int              (*ts_callback)(int);  // Touchscreen callback
	unsigned long      shared;    // Shared resources
} g_adcdev;

static void h3600_asic_start_adc_sample( int mux )
{
	H3800_ASIC2_ADMUX     = mux | ASIC2_ADMUX_CLKEN;
	H3800_ASIC2_ADCSR     = ASIC2_ADCSR_ADPS(4) | ASIC2_ADCSR_INT_ENABLE | ASIC2_ADCSR_ENABLE;
	enable_irq( IRQ_H3800_ADC );
	H3800_ASIC2_ADCSR    |= ASIC2_ADCSR_START;
}

static void h3600_asic_adc_select_next_sample( struct adc_data *adc )
{
	if ( adc->ts_mux > 0 ) {
		adc->state = ADC_STATE_TOUCHSCREEN;
		h3600_asic_start_adc_sample( adc->ts_mux );
	}
	else if ( adc->user_mux >= 0 ) {
		adc->state = ADC_STATE_USER;
		h3600_asic_start_adc_sample( adc->user_mux );
	} else {
		adc->state = ADC_STATE_IDLE;
	}
}

static void h3600_asic_adc_isr( int irq, void *dev_id, struct pt_regs *regs )
{
	struct adc_data *adc = &g_adcdev;
	int data = H3800_ASIC2_ADCDR;

	H3800_ASIC2_ADCSR &= ~ASIC2_ADCSR_INT_ENABLE;   /* Disable the interrupt */
	disable_irq( IRQ_H3800_ADC );

	switch ( adc->state ) {
	case ADC_STATE_IDLE:
		printk(__FUNCTION__ ": Error --- called when no outstanding requests!\n");
		break;
	case ADC_STATE_TOUCHSCREEN:
		if ( adc->ts_callback )
			adc->ts_mux = adc->ts_callback( data );
		else {
			printk(__FUNCTION__ ": Error --- no touchscreen callback\n");
			adc->ts_mux = 0;
		}
		break;
	case ADC_STATE_USER:
		if ( adc->user_mux >= 0 ) {
			adc->last     = data;
			adc->user_mux = -1;
			wake_up_interruptible( &adc->waitq );
		}
		break;
	}
	
	h3600_asic_adc_select_next_sample( adc );
}

/* Call this from touchscreen code (running in interrupt context) */
static void h3600_asic_adc_start_touchscreen( int (*callback)(int), int mux )
{
	struct adc_data *adc = &g_adcdev;

	adc->ts_mux = mux;
	adc->ts_callback = callback;
	if ( adc->state == ADC_STATE_IDLE )
		h3600_asic_adc_select_next_sample(adc);
}

/* Call this from user-mode programs */
int h3600_asic_adc_read_channel( int mux )
{
	struct adc_data *adc = &g_adcdev;
	int              result;
	unsigned long    flags;

	if ( down_interruptible(&adc->lock) )
		return -ERESTARTSYS;

	// Kick start if we aren't currently running
	save_flags_cli( flags );
	adc->user_mux = mux;
	if ( adc->state == ADC_STATE_IDLE ) 
		h3600_asic_adc_select_next_sample( adc );
	restore_flags( flags );

	result = wait_event_interruptible( adc->waitq, adc->user_mux < 0 );

	adc->user_mux = -1;  // May not be -1 if we received a signal
	if ( result >= 0 ) 
		result = adc->last;
	
	up(&adc->lock);
	return result;
}

static void h3600_asic_adc_up( struct adc_data *adc )
{
	h3600_asic_shared_add( &adc->shared, ASIC_SHARED_CLOCK_EX1 );
	H3800_ASIC2_CLOCK_Enable         |= ASIC2_CLOCK_ADC;
}

static void h3600_asic_adc_down( struct adc_data *adc )
{
	H3800_ASIC2_CLOCK_Enable         &= ~ASIC2_CLOCK_ADC;
	h3600_asic_shared_release( &adc->shared, ASIC_SHARED_CLOCK_EX1 );

	// Clear any current touchscreen requests
	adc->ts_mux = 0;
	adc->state  = ADC_STATE_IDLE;
}

int h3600_asic_adc_suspend( void )
{
	DEBUG_INIT();
	down(&g_adcdev.lock);  // No interruptions
	h3600_asic_adc_down( &g_adcdev );
	disable_irq( IRQ_H3800_ADC );
	return 0;
}

void h3600_asic_adc_resume( void )
{
	DEBUG_INIT();
	enable_irq( IRQ_H3800_ADC );
	h3600_asic_adc_up( &g_adcdev );
	up(&g_adcdev.lock);
}

int h3600_asic_adc_init( void )
{
	int result;

	DEBUG_INIT();
	init_MUTEX(&g_adcdev.lock);
	init_waitqueue_head( &g_adcdev.waitq );
	h3600_asic_adc_up( &g_adcdev );

	result = request_irq(IRQ_H3800_ADC, h3600_asic_adc_isr, 
			     SA_INTERRUPT | SA_SAMPLE_RANDOM,
			     "h3800_adc", NULL );

	if ( result )
		printk(KERN_CRIT __FUNCTION__ ": unable to grab ADC IRQ %d error=%d\n", 
		       IRQ_H3800_ADC, result);

	return result;
}

void h3600_asic_adc_cleanup( void )
{
	DEBUG_INIT();
	h3600_asic_adc_down( &g_adcdev );
	free_irq( IRQ_H3800_ADC, NULL );
}


/***********************************************************************************
 *      Touchscreen
 *
 *   Resources           ADC stuff
 *                       Pen interrupt on GPIO2
 *
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

#define ASIC_ADC_DELAY  10  /* Delay 10 milliseconds */

/* Called by ADC ISR routine */
/* Return the number of the next mux to read, or 0 to stop */

static int h3600_asic_touchscreen_record( int data )
{
	struct touchscreen_data *touch = &g_touch;
	const struct ts_sample  *s;

	touch->samples[ touch->index++ ] = data;
	s = &g_samples[ touch->index ];

	H3800_ASIC2_GPIOPIOD = TSMASK | s->mask;    // Set the output pins
	if ( !s->mux )
		mod_timer(&touch->timer, jiffies + (ASIC_ADC_DELAY * HZ) / 1000);

	return s->mux;
}

static void h3600_asic_touchscreen_start_record( struct touchscreen_data *touch )
{
	touch->index = 0;
	H3800_ASIC2_GPIOPIOD = TSMASK | g_samples[0].mask;
	h3600_asic_adc_start_touchscreen( h3600_asic_touchscreen_record, g_samples[0].mux );
}

/* Invoked after a complete series of ADC samples has been taken  */
static void h3600_asic_timer_callback( unsigned long nr )
{
	// The last ADC sample sets us up for "pen" mode
	if ( H3800_ASIC2_GPIOPIOD & GPIO2_PEN_IRQ ) {
		h3600_hal_touchpanel(0,0,0);
		enable_irq( IRQ_H3800_PEN );
	} else {
		unsigned long flags;
		h3600_hal_touchpanel( g_touch.samples[3], g_touch.samples[8], 1 );
		save_flags_cli(flags);
		h3600_asic_touchscreen_start_record( &g_touch );
		restore_flags(flags);
	}
}

static void h3600_asic_pen_isr( int irq, void *dev_id, struct pt_regs *regs )
{
	disable_irq( IRQ_H3800_PEN );
	h3600_asic_touchscreen_start_record( &g_touch );
}

int h3600_asic_touchscreen_suspend( void )
{
	DEBUG_INIT();
	disable_irq( IRQ_H3800_PEN );
	if (del_timer_sync(&g_touch.timer))
		h3600_hal_touchpanel(0,0,0);
	H3800_ASIC2_GPIOPIOD    = TSMASK | GPIO2_IN_Y1_N | GPIO2_IN_X1_N; /* Normal off state */
	return 0;
}

void h3600_asic_touchscreen_resume( void )
{
	DEBUG_INIT();

	H3800_ASIC2_GPIOPIOD    = TSMASK | GPIO2_IN_Y1_N | GPIO2_IN_X0 | GPIO2_IN_X1_N;
	enable_irq( IRQ_H3800_PEN );
}

int h3600_asic_touchscreen_init( void )
{
	int result;
	DEBUG_INIT();

	init_timer(&g_touch.timer);
	g_touch.timer.function = h3600_asic_timer_callback;
	g_touch.timer.data     = (unsigned long) NULL;

	H3800_ASIC2_GPIOPIOD    = TSMASK | GPIO2_IN_Y1_N | GPIO2_IN_X0 | GPIO2_IN_X1_N;

	result = request_irq(IRQ_H3800_PEN, h3600_asic_pen_isr, 
			     SA_INTERRUPT | SA_SAMPLE_RANDOM,
			     "h3800_touchscreen", NULL );

	if ( result )
		printk(KERN_CRIT __FUNCTION__ ": unable to grab touchscreen virtual IRQ\n");

	return result;
}

void h3600_asic_touchscreen_cleanup( void )
{
	DEBUG_INIT();

	free_irq( IRQ_H3800_PEN, NULL );
	if (del_timer_sync(&g_touch.timer))
		h3600_hal_touchpanel(0,0,0);

	H3800_ASIC2_GPIOPIOD    = TSMASK | GPIO2_IN_Y1_N | GPIO2_IN_X1_N; /* Normal off state */
}

/***********************************************************************************
 *   One wire interface for talking with batteries
 *
 *   Resources used:     OWM interface on ASIC2
 *                       OWM Clock on CLOCK (CX6)
 *                       OWM KPIO interrupt line
 *   Shared resource:    EX1 (24.576 MHz crystal) on CLOCK
 ***********************************************************************************/

unsigned char *owm_state_names[] = {
	"Idle", "Reset", "Write", "Read", "Done"
};

static struct h3600_asic_owmdev {
	struct semaphore  lock;
	wait_queue_head_t waitq;
	enum owm_state    state;
	unsigned long     shared;
	unsigned char     last;
} g_owmdev;

struct owm_net_address {
	char address[8];
};

static int  h3600_asic_owm_sample( struct h3600_asic_owmdev *dev )
{
	int value = H3800_ASIC2_OWM_Interrupt;

	if ( (value & OWM_INT_PD) && dev->state == OWM_STATE_RESET ) {
		wake_up_interruptible(&dev->waitq);
		return 0;
	}
	
	if ( (value & OWM_INT_TBE) && (value & OWM_INT_RBF) && dev->state == OWM_STATE_WRITE ) {
		dev->last = H3800_ASIC2_OWM_Data;
		wake_up_interruptible(&dev->waitq);
		return 0;
	}

	if ( (value & OWM_INT_RBF) ) {
		dev->last = H3800_ASIC2_OWM_Data;
		if ( dev->state == OWM_STATE_READ ) {
			wake_up_interruptible(&dev->waitq);
			return 0;
		}
	}
	return 1;
}

static void h3600_asic_owm_isr( int irq, void *dev_id, struct pt_regs *regs ) 
{
	if ( h3600_asic_owm_sample(&g_owmdev) )
		g_h3600_asic_statistics.owm_invalid_isr[g_owmdev.state]++;
	else {
		g_h3600_asic_statistics.owm_valid_isr[g_owmdev.state]++;
		g_owmdev.state = OWM_STATE_DONE;
	}
}

static int one_wire_wait_for_interrupt( int msec )
{
	wait_queue_t  wait;
	signed long   timeout;
	int           result = 0;

	/* We're basically using interruptible_sleep_on_timeout */
	/* and waiting for the transfer to finish */
	init_waitqueue_entry(&wait,current);
	add_wait_queue(&g_owmdev.waitq, &wait);
	timeout = msec * HZ / 1000;

	while ( timeout > 0 ) {
		set_current_state( TASK_INTERRUPTIBLE );
		if ( g_owmdev.state == OWM_STATE_DONE )
			break;
		if ( signal_pending(current) ) {
			result = -ERESTARTSYS;
			break;
		}
		timeout = schedule_timeout( timeout );
		if ( timeout <= 0 ) {
			if ( h3600_asic_owm_sample(&g_owmdev) ) {
				result = -ETIMEDOUT;       /* is this right? */
				g_h3600_asic_statistics.owm_timeout++;
			}
			else {
				g_h3600_asic_statistics.owm_post_isr[g_owmdev.state]++;
			}
		}
	}
	set_current_state( TASK_RUNNING );
	remove_wait_queue(&g_owmdev.waitq, &wait);

	g_owmdev.state = OWM_STATE_IDLE;
	return result;
}

static int one_wire_reset( void )
{
	int result;

	if (0) printk(__FUNCTION__ "\n");

	g_h3600_asic_statistics.owm_reset++;
	g_owmdev.state = OWM_STATE_RESET;
	result = H3800_ASIC2_OWM_Interrupt;    /* Dummy read */

	H3800_ASIC2_OWM_Command |= OWM_CMD_ONE_WIRE_RESET;
	result = one_wire_wait_for_interrupt( owm_sample_delay );

	if ( result ) {
		printk(__FUNCTION__ " OWM reset failed %d (0x%04x)\n", result, H3800_ASIC2_OWM_Interrupt);
		return result;
	}
		
	/* No battery? */
	if ( H3800_ASIC2_OWM_Interrupt & OWM_INT_PDR ) { /* 0 indicates success */
		printk(__FUNCTION__ " OWM reset failed: no battery\n");
		return -ERESTARTSYS;
	}

	udelay(owm_reset_delay);
	return 0;
}

static int one_wire_write( unsigned char data )
{
	int result;

	if (0) printk(__FUNCTION__ ": 0x%02x\n", data);
	g_h3600_asic_statistics.owm_written++;
	g_owmdev.state = OWM_STATE_WRITE;

	result = H3800_ASIC2_OWM_Interrupt;    /* Dummy read */

	H3800_ASIC2_OWM_Data = data;
	result = one_wire_wait_for_interrupt( owm_sample_delay );

	if ( result ) 
		printk("OWM write failed %d (0x%04x)\n", 
		       result, H3800_ASIC2_OWM_Interrupt);

	return result;
}

/* To read, we must clock in data and then read the output buffer */
static int one_wire_read( void )
{
	int result;

	if (0) printk(__FUNCTION__ "\n");
	g_h3600_asic_statistics.owm_read++;
	g_owmdev.state = OWM_STATE_READ;

	result = H3800_ASIC2_OWM_Interrupt;    /* Dummy read */

	H3800_ASIC2_OWM_Data = 0xff;
	result = one_wire_wait_for_interrupt( owm_sample_delay );

	if ( result ) {
		printk("OWM read failed %d (0x%04x)\n", 
		       result, H3800_ASIC2_OWM_Interrupt);
		return result;
	}

	return g_owmdev.last;
}

/* Higher-level routines */

int h3600_asic_owm_read_bytes( struct owm_net_address *net, unsigned char address, 
			       unsigned char *data, unsigned short len )
{
	int result = 0;
	int i;

	if ( down_interruptible(&g_owmdev.lock) )
		return -ERESTARTSYS;

	if ( (result = one_wire_reset()) != 0 ) goto owm_read_bytes_fail;

	if ( net ) {
		if ((result = one_wire_write(0x55)) != 0) goto owm_read_bytes_fail;
		for ( i = 0 ; i < 8 ; i++ )
			if ((result = one_wire_write(net->address[i])) != 0 ) goto owm_read_bytes_fail;
	}
	else {
		if ((result = one_wire_write(0xcc)) != 0) goto owm_read_bytes_fail;
	}
	/* Set address */
	if ( (result = one_wire_write(0x69)) < 0 ) goto owm_read_bytes_fail;
	if ( (result = one_wire_write(address)) < 0 ) goto owm_read_bytes_fail;

	/* Read data */
	for ( i = 0 ; i < len ; i++ ) {
		if ((result = one_wire_read()) < 0) goto owm_read_bytes_fail;
		data[i] = result;
	}

	result = 0;

owm_read_bytes_fail:
	up(&g_owmdev.lock);
	return result;
}

/*
static int h3600_asic_owm_write_bytes( struct owm_net_address *net, unsigned char address, 
				      unsigned char *data, unsigned short len )
{
	int result = 0;
	int i;

	if ( down_interruptible(&g_owmdev.lock) )
		return -ERESTARTSYS;

	if ( (result = one_wire_reset())     != 0 ) goto owm_write_bytes_fail;

	if ( net ) {
		if ((result = one_wire_write(0x55)) != 0) goto owm_write_bytes_fail;
		for ( i = 0 ; i < 8 ; i++ )
			if ((result = one_wire_write(net->address[i])) != 0 ) goto owm_write_bytes_fail;
	}
	else {
		if ((result = one_wire_write(0xcc)) != 0) goto owm_write_bytes_fail;
	}
	// Set address 
	if ( (result = one_wire_write(0x6c)) < 0 ) goto owm_write_bytes_fail;
	if ( (result = one_wire_write(address)) < 0 ) goto owm_write_bytes_fail;

	// Read data 
	for ( i = 0 ; i < len ; i++ )
		if ((result = one_wire_write(data[i])) < 0) goto owm_write_bytes_fail;

	result = 0;

owm_write_bytes_fail:
	up(&g_owmdev.lock);
	return result;
}
*/

static void h3600_asic_owm_up( struct h3600_asic_owmdev *owm )
{
	owm->state = OWM_STATE_IDLE;

	h3600_asic_shared_add( &owm->shared, ASIC_SHARED_CLOCK_EX1 );
	H3800_ASIC2_CLOCK_Enable         |= ASIC2_CLOCK_OWM;
	H3800_ASIC2_OWM_ClockDivisor      = 2;
	H3800_ASIC2_OWM_InterruptEnable   = OWM_INTEN_ERBF | OWM_INTEN_IAS | OWM_INTEN_EPD;
	H3800_ASIC2_INTR_MaskAndFlag     |= ASIC2_INTMASK_OWM; /* Turn on ASIC interrupts */
}

static void h3600_asic_owm_down( struct h3600_asic_owmdev *owm )
{
	wake_up_interruptible(&owm->waitq);

	H3800_ASIC2_INTR_MaskAndFlag     &= ~ASIC2_INTMASK_OWM; /* Turn off ASIC interrupts */
	H3800_ASIC2_OWM_InterruptEnable   = 0;
	H3800_ASIC2_CLOCK_Enable         &= ~ASIC2_CLOCK_OWM;
	h3600_asic_shared_release( &owm->shared, ASIC_SHARED_CLOCK_EX1 );
}

int h3600_asic_owm_suspend( void )
{
	DEBUG_INIT();
	down(&g_owmdev.lock);
	h3600_asic_owm_down( &g_owmdev );
	disable_irq( IRQ_H3800_OWM );
	return 0;
}

void h3600_asic_owm_resume( void )
{
	DEBUG_INIT();
	enable_irq( IRQ_H3800_OWM );
	h3600_asic_owm_up( &g_owmdev );
	up(&g_owmdev.lock);
}

int h3600_asic_owm_init( void )
{
	int result;
	DEBUG_INIT();

	init_waitqueue_head( &g_owmdev.waitq );
	init_MUTEX(&g_owmdev.lock);

	h3600_asic_owm_up( &g_owmdev );

	result = request_irq(IRQ_H3800_OWM, h3600_asic_owm_isr, 
			     SA_INTERRUPT | SA_SAMPLE_RANDOM,
			     "h3800_owm", NULL );

	if ( result )
		printk(KERN_CRIT __FUNCTION__ ": unable to grab OWM virtual IRQ\n");
	return result;
}

void h3600_asic_owm_cleanup( void )
{
	DEBUG_INIT();
	h3600_asic_owm_down( &g_owmdev );
	free_irq( IRQ_H3800_OWM, NULL );
}


/***********************************************************************************
 *   LED control
 ***********************************************************************************/

enum led_color {
	BLUE_LED,
	GREEN_LED,
	YELLOW_LED
};

static void h3600_asic_set_led( enum led_color color, int tbs, int pts, int dts )
{
	H3800_ASIC2_LED_TimeBase(color) = 0;
	if ( tbs ) {
		H3800_ASIC2_LED_PeriodTime(color) = pts;
		H3800_ASIC2_LED_DutyTime(color)   = dts;
		H3800_ASIC2_LED_TimeBase(color)   = tbs;
	}
}

#define h3600_asic_led_on(color)  \
        h3600_asic_set_led(color, LEDTBS_BLINK | LEDTBS_AUTOSTOP | LEDTBS_ALWAYS, 4, 4 )

#define h3600_asic_led_off(color)  \
        h3600_asic_set_led(color, 0, 0, 0)

#define h3600_asic_led_blink(color,rate,pts,dts)  \
        h3600_asic_set_led(color, LEDTBS_BLINK | LEDTBS_AUTOSTOP | LEDTBS_ALWAYS | rate, pts, dts )


/***********************************************************************************
 *   Battery control
 * 
 *   Interface for talking with batteries and controlling the battery charger.
 *
 *   Resources used:     OWM interface
 *                       AC GPIO line
 *                       ADC converter to read battery charger status
 ***********************************************************************************/

struct h3600_asic_batdev {
	struct timer_list    timer;
	enum charging_state  state;
} g_batdev;

int h3600_asic_battery_get_state( void )
{
	return g_batdev.state;
}

void h3600_asic_battery_set_led( enum charging_state state )
{
	struct h3600_asic_batdev *dev = &g_batdev;

	if ( state != dev->state ) {
		switch (state) {
		case CHARGING_STATE_INIT: /* Deliberate fall through */
		case CHARGING_STATE_NO_AC:
			H3800_ASIC2_LED_2_TimeBase = 0;
			break;
		case CHARGING_STATE_ACTIVE:
			H3800_ASIC2_LED_2_TimeBase   = 0;
			H3800_ASIC2_LED_2_PeriodTime = 4;
			H3800_ASIC2_LED_2_DutyTime   = 2;
			H3800_ASIC2_LED_2_TimeBase   = LEDTBS_BLINK | LEDTBS_AUTOSTOP | LEDTBS_ALWAYS;
			break;
		case CHARGING_STATE_FULL:
			H3800_ASIC2_LED_2_TimeBase   = 0;
			H3800_ASIC2_LED_2_PeriodTime = 4;
			H3800_ASIC2_LED_2_DutyTime   = 4;
			H3800_ASIC2_LED_2_TimeBase   = LEDTBS_BLINK | LEDTBS_AUTOSTOP | LEDTBS_ALWAYS;
			break;
		}
		dev->state = state;
	}
}

static int voltage_to_percent( int voltage, int *cal )
{
	int i;

	if ( voltage > cal[0] )
		return 100;

	for ( i = 2 ; cal[i] > 0 ; i+=2 ) 
		if ( voltage > cal[i] ) {
			int p1 = cal[i+1];
			int v1 = cal[i];
			int p2 = cal[i-1];
			int v2 = cal[i-2];
			return (p1 * (v2 - voltage) + p2 * (voltage - v1)) / (v2 - v1);
		}

	return 0;
}

static int lipolymer_calibration[] = {
	881, 100, // 100%
	830,  90, //  90%
	816,  80, //  80%
	805,  70, //  70%
	797,  60, //  60%
	789,  50, //  50%
	783,  40, //  40%
	777,  30, //  30%
	770,  20, //  20%
	764,  10, //  10%
	750,   0, //   0% 
	  0
};

/* 
   The OWM interface call doesn't always work ... sometimes the battery
   just doesn't respond.  We return an error code, but the APM circuitry
   doesn't use error codes (go figure!).  So we also return a structure filled
   out with values equal to the last time we called.  The only value that is
   affected will be the battery voltage level.
*/

int h3600_asic_battery_read( struct h3600_battery *query )
{
	static int voltage   = 783;         /* DUMMY VALUE - good for first initialization */
	static int chemistry = H3600_BATT_CHEM_UNKNOWN;

	int result;
	unsigned char buf[64];
	int percentage;
        int ac_power = (GPLR & GPIO_H3800_AC_IN) ? 0 : 1;

	result = h3600_asic_owm_read_bytes( NULL, 0, buf, 64 );

	if ( !result && buf[1] ) {
		chemistry = buf[48] >> 6;
		voltage   = (((char) buf[12]) << 8 | buf[13]) >> 5;   /* In units of 4.88 mV per */
	}

	query->ac_status            = (ac_power ? H3600_AC_STATUS_AC_ONLINE : H3600_AC_STATUS_AC_OFFLINE );
	query->battery_count        = 1;
	query->battery[0].chemistry = ( chemistry == 0 ? H3600_BATT_CHEM_LIPOLY : H3600_BATT_CHEM_UNKNOWN );
	query->battery[0].voltage   = voltage;

	query->battery[0].status    = H3600_BATT_STATUS_UNKNOWN;
	switch ( g_batdev.state ) {
	case CHARGING_STATE_INIT:
	case CHARGING_STATE_NO_AC:
		if ( voltage > 782 )  /* About 3.82 volts */
			query->battery[0].status = H3600_BATT_STATUS_HIGH;
		else if ( voltage > 764 ) /* About 3.73 voltags */
			query->battery[0].status = H3600_BATT_STATUS_LOW;
		else
			query->battery[0].status = H3600_BATT_STATUS_CRITICAL;
		break;
	case CHARGING_STATE_ACTIVE:
		query->battery[0].status = H3600_BATT_STATUS_CHARGING;
		voltage -= 12;   /* Adjust for charging effect on battery percentage */
		break;
	case CHARGING_STATE_FULL:
		query->battery[0].status = H3600_BATT_STATUS_FULL;
		break;
	}

	percentage = voltage_to_percent( voltage, lipolymer_calibration );
	query->battery[0].percentage = percentage;
	query->battery[0].life       = 300 * percentage / 100;
	
	/* If a sleeve has been inserted, give it a try */
	if ( !(GPLR & GPIO_H3800_NOPT_IND ) && (H3800_ASIC2_GPIOPIOD & GPIO2_OPT_ON)) {
		unsigned char chem, percent, flag;
		if ( !h3600_asic_spi_read_pcmcia_battery( &chem, &percent, &flag ) ) {
			query->battery_count = 2;
			query->battery[1].chemistry  = chem;
			query->battery[1].voltage    = 0;
			query->battery[1].status     = flag;
			query->battery[1].percentage = percent;
			query->battery[1].life       = 0;
		}
	}

	return result;
}

static void h3600_asic_battery_probe_task_handler( void *nr )
{
	int result = h3600_asic_adc_read_channel( ASIC2_ADMUX_1_IMIN );

	if ( result < 0 ) {
		printk(__FUNCTION__ " error reading battery channel\n");
	}
	else if ( result < 100 ) {
		h3600_asic_battery_set_led( CHARGING_STATE_FULL );
	}
	else {
		h3600_asic_battery_set_led( CHARGING_STATE_ACTIVE );
	}
}

static struct tq_struct battery_probe_task = { routine: h3600_asic_battery_probe_task_handler };

static void h3600_asic_battery_timer_callback( unsigned long nr )
{
	struct h3600_asic_batdev *bat = (struct h3600_asic_batdev *) nr;
	schedule_task(&battery_probe_task);
	mod_timer(&bat->timer, jiffies + battery_sample_interval);
}

static void h3600_asic_battery_up( struct h3600_asic_batdev *bat )
{
        int ac_power = (GPLR & GPIO_H3800_AC_IN) ? 0 : 1;
	
	if ( ac_power ) {
		h3600_asic_battery_set_led( CHARGING_STATE_ACTIVE );
		H3800_ASIC1_GPIO_OUT |= GPIO1_CH_TIMER;
		mod_timer(&bat->timer, jiffies + battery_sample_interval);
	}
	else {
		h3600_asic_battery_set_led( CHARGING_STATE_NO_AC );
		H3800_ASIC1_GPIO_OUT &= ~GPIO1_CH_TIMER;
		del_timer_sync(&bat->timer);
	}
}

static void h3600_asic_battery_down( struct h3600_asic_batdev *bat )
{
	h3600_asic_battery_set_led( CHARGING_STATE_NO_AC );
	H3800_ASIC1_GPIO_OUT &= ~GPIO1_CH_TIMER;
	del_timer_sync(&bat->timer);
        flush_scheduled_tasks();
}

void h3600_asic_ac_in_isr(int irq, void *dev_id, struct pt_regs *regs)
{
	h3600_asic_battery_up( &g_batdev );
}

int h3600_asic_battery_suspend( void )
{
	DEBUG_INIT();
	del_timer_sync(&g_batdev.timer);
        flush_scheduled_tasks();
	return 0;
}

void h3600_asic_battery_resume( void )
{
	DEBUG_INIT();
	h3600_asic_battery_up( &g_batdev );
}

int h3600_asic_battery_init( void )
{
	DEBUG_INIT();

	init_timer(&g_batdev.timer);
	g_batdev.timer.function = h3600_asic_battery_timer_callback;
	g_batdev.timer.data     = (unsigned long) &g_batdev;
	g_batdev.state          = CHARGING_STATE_INIT;

	h3600_asic_battery_up( &g_batdev );
	return 0;
}

void h3600_asic_battery_cleanup( void )
{
	DEBUG_INIT();

	h3600_asic_battery_down( &g_batdev );
}

/***********************************************************************************
 *   Audio handlers
 ***********************************************************************************/

#define SET_ASIC2_CLOCK(x) \
	H3800_ASIC2_CLOCK_Enable = (H3800_ASIC2_CLOCK_Enable & ~ASIC2_CLOCK_AUDIO_MASK) | (x)

int h3600_asic_audio_clock( long samplerate )
{
	static unsigned long shared;

	if ( !samplerate ) {
		h3600_asic_shared_release( &shared, ASIC_SHARED_CLOCK_EX1 );
		h3600_asic_shared_release( &shared, ASIC_SHARED_CLOCK_EX2 );
		return 0;
	}

	/* Set the external clock generator */
	switch (samplerate) {
	case 24000:
	case 32000:
	case 48000:
		/* 12.288 MHz - needs 24.576 MHz crystal */
		h3600_asic_shared_add( &shared, ASIC_SHARED_CLOCK_EX1 );
		SET_ASIC2_CLOCK(ASIC2_CLOCK_AUDIO_2);
		h3600_asic_shared_release( &shared, ASIC_SHARED_CLOCK_EX2 );
		break;
	case 22050:
	case 29400:
	case 44100:
		/* 11.2896 MHz - needs 33.869 MHz crystal */
		h3600_asic_shared_add( &shared, ASIC_SHARED_CLOCK_EX2 );
		SET_ASIC2_CLOCK(ASIC2_CLOCK_AUDIO_4);
		h3600_asic_shared_release( &shared, ASIC_SHARED_CLOCK_EX1 );
		break;
	case 8000:
	case 10666:
	case 16000:
		/* 4.096 MHz  - needs 24.576 MHz crystal */
		h3600_asic_shared_add( &shared, ASIC_SHARED_CLOCK_EX1 );
		SET_ASIC2_CLOCK(ASIC2_CLOCK_AUDIO_1);
		h3600_asic_shared_release( &shared, ASIC_SHARED_CLOCK_EX2 );
		break;
	case 10985:
	case 14647:
	case 21970:
		/* 5.6245 MHz  - needs 33.869 MHz crystal */
		h3600_asic_shared_add( &shared, ASIC_SHARED_CLOCK_EX2 );
		SET_ASIC2_CLOCK(ASIC2_CLOCK_AUDIO_3);
		h3600_asic_shared_release( &shared, ASIC_SHARED_CLOCK_EX1 );
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int h3600_asic_audio_power( long samplerate )
{
	int retval;

	H3800_ASIC1_GPIO_OUT &= ~GPIO1_AUD_PWR_ON;
	retval = h3600_asic_audio_clock(samplerate);

	if ( samplerate > 0 )
		H3800_ASIC1_GPIO_OUT |= GPIO1_AUD_PWR_ON;

	return retval;
}

static void h3600_asic_audio_fix_jack( int mute )
{
	int earphone = H3800_ASIC2_GPIOPIOD & GPIO2_EAR_IN_N ? 0 : 1;

	if ( earphone )
		H3800_ASIC2_GPIINTESEL |= GPIO2_EAR_IN_N; /* Rising */
	else
		H3800_ASIC2_GPIINTESEL &= ~GPIO2_EAR_IN_N; /* Falling */

	if ( mute ) 
		H3800_ASIC1_GPIO_OUT = (H3800_ASIC1_GPIO_OUT | GPIO1_EAR_ON_N) & ~GPIO1_SPK_ON;
	else {
		if ( earphone )
			H3800_ASIC1_GPIO_OUT &= ~(GPIO1_SPK_ON | GPIO1_EAR_ON_N);
		else
			H3800_ASIC1_GPIO_OUT |= GPIO1_SPK_ON | GPIO1_EAR_ON_N;
	}
}

/* Called from HAL or the debounce timer (from an interrupt)*/
int h3600_asic_audio_mute( int mute )
{
	unsigned long flags;
	local_irq_save(flags);
	h3600_asic_audio_fix_jack(mute);
	local_irq_restore(flags);

	return 0;
}

static void audio_timer_callback( unsigned long nr )
{
	h3600_asic_audio_mute( !(H3800_ASIC1_GPIO_OUT & GPIO1_AUD_PWR_ON) );
}

static struct timer_list g_audio_timer = { function: audio_timer_callback };

static void h3600_asic_ear_in_isr( int irq, void *dev_id, struct pt_regs *regs )
{
	mod_timer( &g_audio_timer, jiffies + (2 * HZ) / 1000 );
}

int h3600_asic_audio_suspend( void )
{
	DEBUG_INIT();
	del_timer_sync(&g_audio_timer);
	disable_irq( IRQ_H3800_EAR_IN );
	return 0;
}

void h3600_asic_audio_resume( void )
{
	DEBUG_INIT();
	H3800_ASIC2_GPIINTTYPE |= GPIO2_EAR_IN_N;   /* Set edge-type interrupt */
	h3600_asic_audio_fix_jack(1);
	enable_irq( IRQ_H3800_EAR_IN );
}

int h3600_asic_audio_init( void )
{
	int result;

	DEBUG_INIT();
	init_timer(&g_audio_timer);
	H3800_ASIC2_GPIINTTYPE |= GPIO2_EAR_IN_N;   /* Set edge-type interrupt */
	h3600_asic_audio_fix_jack(1);

	result = request_irq(IRQ_H3800_EAR_IN, h3600_asic_ear_in_isr, 
			     SA_INTERRUPT | SA_SAMPLE_RANDOM,
			     "h3800_ear_in", NULL );

	if ( result )
		printk(KERN_CRIT __FUNCTION__ ": unable to grab EAR_IN virtual IRQ\n");
	return result;
}

void h3600_asic_audio_cleanup( void )
{
	DEBUG_INIT();
	del_timer_sync(&g_audio_timer);
	free_irq( IRQ_H3800_EAR_IN, NULL );
}


/***********************************************************************************
 *   Assets
 ***********************************************************************************/

struct mtd_info *g_asset_mtd;

/*
#define DUMP_BUF_SIZE   16
#define MYPRINTABLE(x) \
    ( ((x)<='z' && (x)>='a') || ((x)<='Z' && (x)>='A') || isdigit(x))

static void dump_mem( struct mtd_info *mtd, loff_t from, size_t count )
{
	size_t retlen;
	u_char buf[DUMP_BUF_SIZE];
	u_char pbuf[DUMP_BUF_SIZE + 1];

	int    ret;
	size_t len;
	int    i;

	printk(__FUNCTION__ ": dumping 0x%08X, len %dd\n", from, count);
	while ( count ) {
		len = count;
		if ( len > DUMP_BUF_SIZE )
			len = DUMP_BUF_SIZE;

		ret = mtd->read( mtd, from, len, &retlen, buf );
		if (!ret) {
			for ( i = 0 ; i < retlen ; i++ ) {
				printk(" %02x", buf[i]);
				pbuf[i] = ( MYPRINTABLE(buf[i]) ? buf[i] : '.' );
				from++;
				count--;
				if ( (from % DUMP_BUF_SIZE) == 0 ) {
					pbuf[i+1] = '\0';
					printk(" %s\n", pbuf);
				}
			}
		}
		else {
			printk(__FUNCTION__ ": error %d reading at %d, count=%d", ret,from,count);
			return;
		}
	}

}
*/

static int copytchar( struct mtd_info *mtd, unsigned char *dest, loff_t from, size_t count )
{
	size_t retlen;

	if (0) printk(__FUNCTION__ ": %p %lld %u\n", dest, from, count );

	while ( count ) {
		int ret = mtd->read( mtd, from, count, &retlen, dest );
		if (!ret) {
			count -= retlen;
			dest += retlen;
			from += retlen;
		}
		else {
			printk(__FUNCTION__ ": error %d reading at %lld, count=%d\n", ret, from, count );
			return ret;
		}
	}
	return 0;
}

static int copyword( struct mtd_info *mtd, unsigned short *dest, loff_t from )
{
	unsigned char buf[2];
	int ret = copytchar( mtd, buf, from, 2 );
	if ( ret ) 
		return ret;
	*dest = (((unsigned short) buf[1]) << 8 | (unsigned short) buf[0]);
	return 0;
}

#define COPYTCHAR(_p,_offset,_len) \
        ( g_asset_mtd ? \
        copytchar( g_asset_mtd, _p, _offset + 0x30000, _len ) \
        : 0)

#define COPYWORD(_w,_offset) \
        ( g_asset_mtd ? \
        copyword( g_asset_mtd, &_w, _offset + 0x30000 ) \
          : 0)

int h3600_asic_asset_read( struct h3600_asset *asset )
{
	int retval = -1;

	if (0) printk(__FUNCTION__ "\n");

	switch (asset->type) {
	case ASSET_HM_VERSION:
		retval = COPYTCHAR( asset->a.tchar, 0, 10 );
		break;
	case ASSET_SERIAL_NUMBER:
		retval = COPYTCHAR( asset->a.tchar, 10, 40 );
		break;
	case ASSET_MODULE_ID:
		retval = COPYTCHAR( asset->a.tchar, 152, 20 );
		break;
	case ASSET_PRODUCT_REVISION:
		retval = COPYTCHAR( asset->a.tchar, 182, 10 );
		break;
	case ASSET_PRODUCT_ID:
		retval = COPYWORD( asset->a.vshort, 110 );
		break;
	case ASSET_FRAME_RATE:
		retval = COPYWORD( asset->a.vshort, 966 );
		break;
	case ASSET_PAGE_MODE:    
		retval = COPYWORD( asset->a.vshort, 976 );
		break;
	case ASSET_COUNTRY_ID:
		retval = COPYWORD( asset->a.vshort, 980 );
		break;
	case ASSET_IS_COLOR_DISPLAY:
		retval = COPYWORD( asset->a.vshort, 292 );
		break;
	case ASSET_ROM_SIZE:
		retval = COPYWORD( asset->a.vshort, 1514 );
		break;
	case ASSET_RAM_SIZE:
		asset->a.vshort = 0;
		retval = 0;
		break;
	case ASSET_HORIZONTAL_PIXELS:
		retval = COPYWORD( asset->a.vshort, 276 );
		break;
	case ASSET_VERTICAL_PIXELS:
		retval = COPYWORD( asset->a.vshort, 278 );
		break;
	}

	return retval;
}


int h3600_asic_asset_init( void )
{
	int i;
	struct mtd_info *mtd;

	for ( i = 0 ; i < MAX_MTD_DEVICES ; i++ ) {
		if ( (mtd = get_mtd_device( NULL, i )) != NULL ) {
			if ( !strcmp(mtd->name, "asset" )) {
				g_asset_mtd = mtd;
				return 0;
			}
			else {
				put_mtd_device(mtd);
			}
		}
	}
	return 0;
}

void h3600_asic_asset_cleanup( void )
{
	if ( g_asset_mtd )
		put_mtd_device(g_asset_mtd);
}


/***********************************************************************************
 *   Bluetooth
 *
 *   Resources used:    
 *   Shared resources:  
 *
 ***********************************************************************************/

struct h3600_asic_bluetooth {
	int line;       // Serial port line
	unsigned long shared;
};

static struct h3600_asic_bluetooth g_bluedev;

static void h3600_asic_bluetooth_up( struct h3600_asic_bluetooth *dev )
{
	H3800_ASIC2_INTR_MaskAndFlag &= ~ASIC2_INTMASK_UART_0;

	H3800_ASIC2_UART_0_RSR = 1;   // Reset the UART
	H3800_ASIC2_UART_0_RSR = 0;

	H3800_ASIC2_CLOCK_Enable = 
		(H3800_ASIC2_CLOCK_Enable & ~ASIC2_CLOCK_UART_0_MASK) | ASIC2_CLOCK_SLOW_UART_0;
	h3600_asic_shared_add( &dev->shared, ASIC_SHARED_CLOCK_EX1 );

	// H3600.c has already taken care of sending the 3.6864 MHz clock to the UART (TUCR) 
	// but we're paranoid...
	GPDR |= GPIO_H3800_CLK_OUT;
	GAFR |= GPIO_H3800_CLK_OUT;

	TUCR = TUCR_3_6864MHzA;

	// Set up the TXD & RXD of the UART to normal operation
	H3800_ASIC2_GPIODIR &= ~GPIO2_UART0_TXD_ENABLE;
	H3800_ASIC2_GPIOPIOD |= GPIO2_UART0_TXD_ENABLE;

	// More black magic
	H3800_ASIC2_KPIODIR  &= ~KPIO_UART_MAGIC;
	H3800_ASIC2_KPIOPIOD |=  KPIO_UART_MAGIC;
}

static void h3600_asic_bluetooth_down( struct h3600_asic_bluetooth *dev )
{
	H3800_ASIC2_INTR_MaskAndFlag   |= ASIC2_INTMASK_UART_0;
	h3600_asic_shared_release( &dev->shared, ASIC_SHARED_CLOCK_EX1 );
}

int h3600_asic_bluetooth_suspend( void )
{
	h3600_asic_bluetooth_down( &g_bluedev );
	return 0;
}

void h3600_asic_bluetooth_resume( void )
{
	h3600_asic_bluetooth_up( &g_bluedev );
}

static void serial_fn_pm( struct uart_port *port, u_int state, u_int oldstate )
{
	if (0) printk(__FUNCTION__ ": %p %d %d\n", port, state, oldstate);
}

static int serial_fn_open( struct uart_port *port, struct uart_info *info )
{
	if (0) printk(__FUNCTION__ ": %p %p\n", port, info);
	MOD_INC_USE_COUNT;

	H3800_ASIC1_GPIO_OUT |= GPIO1_BT_PWR_ON;
	h3600_asic_led_blink(BLUE_LED,1,7,1);
	return 0;
}

static void serial_fn_close( struct uart_port *port, struct uart_info *info )
{
	if (0) printk(__FUNCTION__ ": %p %p\n", port, info);
	h3600_asic_led_off(BLUE_LED);
	H3800_ASIC1_GPIO_OUT &= ~GPIO1_BT_PWR_ON;

	MOD_DEC_USE_COUNT;
}

static struct serial_h3800_fns serial_fns = {
	pm    : serial_fn_pm,
	open  : serial_fn_open,
	close : serial_fn_close
};

int h3600_asic_bluetooth_init( void )
{
	h3600_asic_bluetooth_up( &g_bluedev );

	register_serial_h3800_fns(&serial_fns);
	g_bluedev.line = register_serial_h3800(0);
	if ( g_bluedev.line < 0 ) {
		printk(__FUNCTION__ ": register serial failed\n");
		return g_bluedev.line;
	} 

	return 0;
}

void h3600_asic_bluetooth_cleanup( void )
{
	printk(__FUNCTION__ " %d\n", g_bluedev.line);
	unregister_serial_h3800( g_bluedev.line );
	register_serial_h3800_fns(NULL);
	h3600_asic_bluetooth_down( &g_bluedev );
}
