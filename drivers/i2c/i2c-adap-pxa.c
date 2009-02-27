/*
 *  i2c_adap_pxa.c
 *
 *  I2C adapter for the PXA I2C bus access.
 *
 *  Copyright (C) 2002 Intrinsyc Software Inc.
 *
 *  Copyright (C) 2003 Montavista Software Inc.
 *
 *  Author: Intrinsyc Software Inc.
 *          MontaVista Software, Inc.
 *           source@mvista.com
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  History:
 *    Apr 2002: Initial version [CS]
 *    Jun 2002: Properly seperated algo/adap [FB]
 *    Sep 2003: Bulverde Power-I2C support added
 */
/*
 * Copyright (C) 2003 Motorola Inc.
 *
 *  History:
 *  Jay Jia(w20091),    Nov 25,2003 ,	Modified for EZX platform
 */


#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/arch/irqs.h>              /* for IRQ_I2C, IRQ_PWRI2C */
#include <linux/pm.h>
#include "i2c-pxa.h"

/* Physical base address of Standart I2C controller */
#define I2C_STANDART_BASE       0x40301680

#ifdef CONFIG_POWER_I2C

/* Physical base address of Power I2C controller */
#define I2C_POWER_BASE		0x40F00180

/* I2C Bus Monitor Register - IBMR/PIBMR */
#define _IBMR(adap)	__REG(((adap)->base) + 0x00)
/* I2C Data Buffer Register - IDBR/PIDBR */
#define _IDBR(adap)	__REG(((adap)->base) + 0x08)
/* I2C Control Register - ICR/PICR */
#define _ICR(adap)	__REG(((adap)->base) + 0x10)
/* I2C Status Register - ISR/PISR */
#define _ISR(adap)	__REG(((adap)->base) + 0x18)
/* I2C Slave Address Register - ISAR/PISAR */
#define _ISAR(adap)	__REG(((adap)->base) + 0x20)

#else

#define _IBMR(adap)	IBMR
#define _IDBR(adap)	IDBR
#define _ICR(adap)	ICR
#define _ISR(adap)	ISR
#define _ISAR(adap)	ISAR

#endif

/*
 * Set this to zero to remove all the debug statements via dead code elimination.
 */
//#define DEBUG 1

#if DEBUG
static unsigned int i2c_debug = DEBUG;
#else
#define i2c_debug	0
#endif

static struct i2c_algo_pxa_data i2c_pxa_data;
//static struct i2c_algo_pxa_data i2c_pxa_pwr_data;

#ifdef CONFIG_PM
//static struct pm_dev *pm_dev;
#endif

#ifdef CONFIG_DPM
#include <linux/device.h>

static int pxai2c_suspend(struct device *dev, u32 state, u32 level);
static int pxai2c_resume(struct device *dev, u32 level);
static int pxai2c_scale(struct bus_op_point *op, u32 level);

static struct device_driver pxai2c_driver_ldm = {
	name:          "pxa-i2c",
	devclass:      NULL,
	probe:         NULL,
	suspend:       pxai2c_suspend,
	resume:        pxai2c_resume,
	scale:         pxai2c_scale,
	remove:        NULL,
	constraints:   NULL,
};

static struct device pxai2c_device_ldm = {
	name:         "PXA I2C",
	bus_id:       "pxai2c",
	driver:       NULL,
	power_state:  DPM_POWER_ON,
};

static void
pxai2c_ldm_register(void)
{
	extern void pxaopb_driver_register(struct device_driver *driver);
	extern void pxaopb_device_register(struct device *device);
	
	pxaopb_driver_register(&pxai2c_driver_ldm);
	pxaopb_device_register(&pxai2c_device_ldm);
}

static void
pxai2c_ldm_unregister(void)
{
	extern void pxaopb_driver_unregister(struct device_driver *driver);
	extern void pxaopb_device_unregister(struct device *device);
	
	pxaopb_device_unregister(&pxai2c_device_ldm);
	pxaopb_driver_unregister(&pxai2c_driver_ldm);
}

static int
pxai2c_resume(struct device *dev, u32 level)
{
	if (i2c_debug)
          printk("+++: in pxai2c_resume()\n");

	switch (level) {
	case RESUME_POWER_ON:
		/* enable clocks */
		CKEN |= CKEN14_I2C;
#ifdef CONFIG_POWER_I2C
		CKEN |= CKEN15_PWRI2C;
#endif
		
		/* enable PI2C Controller */
		PCFR |= PCFR_PI2CEN;
		
		/* enable irqs */
		enable_irq(i2c_pxa_data.irq);
#ifdef CONFIG_POWER_I2C
		enable_irq(i2c_pxa_pwr_data.irq);
#endif
		break;
	}
	
	return 0;
}

static int
pxai2c_suspend(struct device *dev, u32 state, u32 level)
{
	if (i2c_debug)
	  printk("+++: in pxai2c_suspend()\n");

	switch (level) {
	case SUSPEND_POWER_DOWN:
		/* disable IRQs */
		if (i2c_pxa_data.irq) 
	          disable_irq(i2c_pxa_data.irq);
#ifdef CONFIG_POWER_I2C
		if (i2c_pxa_pwr_data.irq)
	          disable_irq(i2c_pxa_pwr_data.irq);
#endif
		
		/* disable PI2C Controller */
		PCFR &= ~PCFR_PI2CEN;
		
		/* disable clocks */
		CKEN &= ~CKEN14_I2C;
#ifdef CONFIG_POWER_I2C
		CKEN &= ~CKEN15_PWRI2C;
#endif	
		break;
	}
	
	return 0;
}

static int
pxai2c_scale(struct bus_op_point *op, u32 level)
{
	printk("+++: in pxai2c_scale()\n");
	return 0;
}
#endif /* CONFIG_DPM */

/* place a byte in the transmit register */
static void i2c_pxa_write_byte(struct i2c_algo_pxa_data *adap, u8 value) 
{
        _IDBR(adap) = value;
}

/* read byte in the receive register */
static u8 i2c_pxa_read_byte(struct i2c_algo_pxa_data *adap) 
{
        return (u8) (0xff & _IDBR(adap));
}

static void i2c_pxa_start(struct i2c_algo_pxa_data *adap)
{
	_ICR(adap) |= ICR_START;
	_ICR(adap) &= ~(ICR_STOP | ICR_ALDIE | ICR_ACKNAK);

	adap->bus_error=0;	/* clear any bus_error from previous txfers */
	adap->tx_finished=0;    /* clear rx and tx interrupts */
	adap->rx_finished=0;    /* from previous txfers */
}

static void i2c_pxa_repeat_start(struct i2c_algo_pxa_data *adap)
{
	_ICR(adap) |= ICR_START;
	_ICR(adap) &= ~(ICR_STOP | ICR_ALDIE);

	adap->bus_error=0;	/* clear any bus_error from previous txfers */
	adap->tx_finished=0;    /* clear rx and tx interrupts */
	adap->rx_finished=0;    /* from previous txfers */
}

static void i2c_pxa_stop(struct i2c_algo_pxa_data *adap)
{
	_ICR(adap) |= ICR_STOP;
	_ICR(adap) &= ~(ICR_START);
}

static void i2c_pxa_midbyte(struct i2c_algo_pxa_data *adap)
{
	_ICR(adap) &= ~(ICR_START | ICR_STOP);
}

static void i2c_pxa_abort(struct i2c_algo_pxa_data *adap)
{
	_ICR(adap) |= ICR_MA;
}

static int i2c_pxa_wait_bus_not_busy(struct i2c_algo_pxa_data *adap)
{
        int timeout = DEF_TIMEOUT;

        while (timeout-- && (_ISR(adap) & ISR_IBB)) {
                udelay(100); /* wait for 100 us */
        }

        return (timeout<=0);
}

static void i2c_pxa_wait_for_ite(struct i2c_algo_pxa_data *adap)
{
//	unsigned long flags;

	if (adap->irq > 0) {
//		save_flags_cli(flags);
		if (adap->i2c_pending == 0) {
                    int timeout , count ;
		    timeout =	interruptible_sleep_on_timeout(&adap->i2c_wait,
                                           /* I2C_SLEEP_TIMEOUT*/8);
                    if ( ( 8 == timeout ) && ( adap->i2c_pending == 0 ) ) 
                    {
                        for ( count = 0 ; count < 1000 ; count ++ )
                        {
                            if ( adap->i2c_pending == 0 ) 
                                udelay( 10 ) ;
                            else
                                break ;
                        }
                    }
		}
		adap->i2c_pending = 0;
//		restore_flags(flags);
	} else {
		udelay(100);
	}
}

static int i2c_pxa_wait_for_int(struct i2c_algo_pxa_data *adap, int wait_type)
{
        int timeout = DEF_TIMEOUT;

#ifdef DEBUG
	if (adap->bus_error)
		printk(KERN_INFO"i2c_pxa_wait_for_int: Bus error on enter\n");
	if (adap->rx_finished)
		printk(KERN_INFO"i2c_pxa_wait_for_int: Receive interrupt on enter\n");
	if (adap->tx_finished)
		printk(KERN_INFO"i2c_pxa_wait_for_int: Transmit interrupt on enter\n");
#endif

        if (wait_type == I2C_RECEIVE){         /* wait on receive */

                while (timeout-- && !(adap->rx_finished)){
                        i2c_pxa_wait_for_ite(adap);
                }
#ifdef DEBUG
                if (timeout<0){
                        if (adap->tx_finished)
                                printk("Error: i2c-algo-pxa.o: received a tx"
                                        " interrupt while waiting on a rx in wait_for_int");
                }
#endif
        } else {                  /* wait on transmit */
        
                while (timeout-- && !(adap->tx_finished)){
                        i2c_pxa_wait_for_ite(adap);
                }
#ifdef DEBUG
                if (timeout<0){
                        if (adap->rx_finished)
                                printk("Error: i2c-algo-pxa.o: received a rx"
                                        " interrupt while waiting on a tx in wait_for_int");
                }
#endif
        }       

        udelay(ACK_DELAY);      /* this is needed for the bus error */

        adap->tx_finished=0;
        adap->rx_finished=0;

        if (adap->bus_error){
                adap->bus_error=0;
                if( i2c_debug > 2)printk("wait_for_int: error - no ack.\n");
                return BUS_ERROR;
        }

        if (timeout < 0)
                return(-1);
        else
                return(0);
}

static void i2c_pxa_transfer(struct i2c_algo_pxa_data *adap,
			     int lastbyte, int receive, int midbyte)
{
	if( lastbyte)
	{
		if( receive==I2C_RECEIVE) _ICR(adap) |= ICR_ACKNAK; 
		i2c_pxa_stop(adap);
	}
	else if( midbyte)
	{
		i2c_pxa_midbyte(adap);
	}
	_ICR(adap) |= ICR_TB;
}

static void i2c_pxa_reset(struct i2c_algo_pxa_data *adap)
{
        /* set the global I2C clocks on */
        CKEN |= CKEN14_I2C;

	set_GPIO_mode(117 | GPIO_ALT_FN_1_IN);
	set_GPIO_mode(118 | GPIO_ALT_FN_1_IN);

#ifdef CONFIG_POWER_I2C
        /* set the global PWR_I2C clock on */
        CKEN |= CKEN15_PWRI2C;

	/* Enable PI2C controller PWR_SCL and PWR_SDA */
	PCFR |= PCFR_PI2CEN;

	/* Setup GPIO3 and GPIO4 to ALT_FN1_IN (PWR_SCL and PWR_SDA) */
	set_GPIO_mode(3 | GPIO_ALT_FN_1_IN);
	set_GPIO_mode(4 | GPIO_ALT_FN_1_IN);

#endif

        /* disable unit */
	_ICR(adap) &= ~ICR_IUE;

        /* reset the unit */
	_ICR(adap) |= ICR_UR;
        udelay(100);
	_ICR(adap) |= ICR_FM;
	
        /* disable unit */
	_ICR(adap) &= ~ICR_IUE;

	/* XXX: I2C_PXA_SLAVE_ADDR == I2C_PXA_PWR_SLAVE_ADDR ??? */
        /* set our slave address */
	_ISAR(adap) = I2C_PXA_SLAVE_ADDR;

        /* set control register values */
	_ICR(adap) = I2C_ICR_INIT;

        /* set clear interrupt bits */
	_ISR(adap) = I2C_ISR_INIT;

        /* enable unit */
	_ICR(adap) |= ICR_IUE;
        udelay(100);

}

static void i2c_pxa_handler(int this_irq, void *dev_id, struct pt_regs *regs)
{
        int status;

	struct i2c_algo_pxa_data *adap = &i2c_pxa_data;
	
        status = (_ISR(adap));

        if (status & ISR_BED){
                (_ISR(adap)) |= ISR_BED;
                adap->bus_error = ISR_BED;
        }
        if (status & ISR_ITE){
                (_ISR(adap)) |= ISR_ITE;
                adap->tx_finished=ISR_ITE;
        }
        if (status & ISR_IRF){
                (_ISR(adap)) |= ISR_IRF;
                adap->rx_finished=ISR_IRF;
        }
        adap->i2c_pending = 1;
        wake_up_interruptible(&adap->i2c_wait);
}

#ifdef CONFIG_POWER_I2C
static void i2c_pxa_pwr_handler(int this_irq, void *dev_id,
				struct pt_regs *regs)
{
        int status;

	struct i2c_algo_pxa_data *adap = &i2c_pxa_pwr_data;
	
        status = (_ISR(adap));

        if (status & ISR_BED){
                (_ISR(adap)) |= ISR_BED;
                adap->bus_error = ISR_BED;
        }
        if (status & ISR_ITE){
                (_ISR(adap)) |= ISR_ITE;
                adap->tx_finished=ISR_ITE;
        }
        if (status & ISR_IRF){
                (_ISR(adap)) |= ISR_IRF;
                adap->rx_finished=ISR_IRF;
        }
        adap->i2c_pending = 1;
        wake_up_interruptible(&adap->i2c_wait);
}
#endif

static int i2c_pxa_resource_init(void)
{
	struct i2c_algo_pxa_data *adap = &i2c_pxa_data;

        init_waitqueue_head(&adap->i2c_wait);

        if (request_irq(IRQ_I2C, &i2c_pxa_handler, SA_INTERRUPT, "I2C_PXA", 0) < 0) {
                adap->irq = 0;
                if (i2c_debug)
			printk(KERN_INFO "I2C: Failed to register I2C irq %i\n", IRQ_I2C);
                return -ENODEV;
        }else{

                adap->irq = IRQ_I2C;
                enable_irq(adap->irq);
        }
        return 0;
}

#ifdef CONFIG_POWER_I2C
static int i2c_pxa_pwr_resource_init(void)
{
	struct i2c_algo_pxa_data *adap = &i2c_pxa_pwr_data;

        init_waitqueue_head(&adap->i2c_wait);

        if (request_irq(IRQ_PWRI2C, &i2c_pxa_pwr_handler, SA_INTERRUPT, "PWRI2C_PXA", 0) < 0) {
                adap->irq = 0;
                if (i2c_debug)
			printk(KERN_INFO "I2C: Failed to register I2C irq %i\n", IRQ_I2C);
                return -ENODEV;
        }else{

                adap->irq = IRQ_PWRI2C;
                enable_irq(adap->irq);
        }
        return 0;
}
#endif

static void i2c_pxa_resource_release(struct i2c_algo_pxa_data *adap)
{
        if( adap->irq > 0)
        {
                disable_irq(adap->irq);
                free_irq(adap->irq, 0);
                adap->irq = 0;
        }
}

static void i2c_pxa_inc_use(struct i2c_adapter *adap)
{
#ifdef MODULE
        MOD_INC_USE_COUNT;
#endif
}

static void i2c_pxa_dec_use(struct i2c_adapter *adap)
{
#ifdef MODULE
        MOD_DEC_USE_COUNT;
#endif
}

static int i2c_pxa_client_register(struct i2c_client *client)
{
        return 0;
}

static int i2c_pxa_client_unregister(struct i2c_client *client)
{
        return 0;
}

/* Standart I2C controller info structure */
static struct i2c_algo_pxa_data i2c_pxa_data = {
        write_byte:		i2c_pxa_write_byte,
        read_byte:		i2c_pxa_read_byte,

        start:			i2c_pxa_start,
        repeat_start:		i2c_pxa_repeat_start,
        stop:			i2c_pxa_stop,
        abort:			i2c_pxa_abort,

        wait_bus_not_busy:	i2c_pxa_wait_bus_not_busy,
        wait_for_interrupt:	i2c_pxa_wait_for_int,
        transfer:		i2c_pxa_transfer,
        reset:			i2c_pxa_reset,
	
	base:			I2C_STANDART_BASE,
	
	irq:			0,
	i2c_pending:		0,
	bus_error:		0,
	tx_finished:		0,
	rx_finished:		0,

	udelay:			10,
	timeout:		DEF_TIMEOUT,
};

#ifdef CONFIG_POWER_I2C
/* Power I2C controller info structure */
static struct i2c_algo_pxa_data i2c_pxa_pwr_data = {
        write_byte:		i2c_pxa_write_byte,
        read_byte:		i2c_pxa_read_byte,

        start:			i2c_pxa_start,
        repeat_start:		i2c_pxa_repeat_start,
        stop:			i2c_pxa_stop,
        abort:			i2c_pxa_abort,

        wait_bus_not_busy:	i2c_pxa_wait_bus_not_busy,
        wait_for_interrupt:	i2c_pxa_wait_for_int,
        transfer:		i2c_pxa_transfer,
        reset:			i2c_pxa_reset,
	
	base:			I2C_POWER_BASE,
	
	irq:			0,
	i2c_pending:		0,
	bus_error:		0,
	tx_finished:		0,
	rx_finished:		0,

	udelay:			10,
	timeout:		DEF_TIMEOUT,
};
#endif

static struct i2c_adapter i2c_pxa_ops = {
        name:                   "PXA-I2C-Adapter",
        id:                     I2C_ALGO_PXA,
        algo_data:              &i2c_pxa_data,
        inc_use:                i2c_pxa_inc_use,
        dec_use:                i2c_pxa_dec_use,
        client_register:        i2c_pxa_client_register,
        client_unregister:      i2c_pxa_client_unregister,
        retries:                2,
};

#ifdef CONFIG_POWER_I2C
static struct i2c_adapter i2c_pxa_pwr_ops = {
        name:                   "PXA-Power-I2C-Adapter",
        id:                     I2C_ALGO_PXA,
        algo_data:              &i2c_pxa_pwr_data,
        inc_use:                i2c_pxa_inc_use,
        dec_use:                i2c_pxa_dec_use,
        client_register:        i2c_pxa_client_register,
        client_unregister:      i2c_pxa_client_unregister,
        retries:                2,
};
#endif

extern int i2c_pxa_add_bus(struct i2c_adapter *);
extern int i2c_pxa_del_bus(struct i2c_adapter *);

#ifdef CONFIG_PM
void ezx_i2c_pm_suspend(void)
{
	/* disable IRQs */
        if (i2c_pxa_data.irq)
                disable_irq(i2c_pxa_data.irq);
        /* disable clocks */
        CKEN &= ~CKEN14_I2C;
}
void ezx_i2c_pm_resume(void)
{
        /* enable clocks */
        CKEN |= CKEN14_I2C;
        set_GPIO_mode(117 | GPIO_ALT_FN_1_IN);
        set_GPIO_mode(118 | GPIO_ALT_FN_1_IN);
        /* enable irqs */
        enable_irq(i2c_pxa_data.irq);
        /* disable unit */
        _ICR(adap) &= ~ICR_IUE;
        /* reset the unit */
        _ICR(adap) |= ICR_UR;
        udelay(100);
        _ICR(adap) |= ICR_FM;
        /* disable unit */
        _ICR(adap) &= ~ICR_IUE;
        /* XXX: I2C_PXA_SLAVE_ADDR == I2C_PXA_PWR_SLAVE_ADDR ??? */
        /* set our slave address */
        _ISAR(adap) = I2C_PXA_SLAVE_ADDR;
        /* set control register values */
        _ICR(adap) = I2C_ICR_INIT;
        /* set clear interrupt bits */
        _ISR(adap) = I2C_ISR_INIT;
        /* enable unit */
        _ICR(adap) |= ICR_IUE;
        udelay(100);					
}
#if 0
static int ezx_i2c_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
{
	switch(req)
	{
	case PM_SUSPEND:
		/* disable IRQs */
		if (i2c_pxa_data.irq) 
		          disable_irq(i2c_pxa_data.irq);
		/* disable PI2C Controller */
		PCFR &= ~PCFR_PI2CEN;
		
		/* disable clocks */
		CKEN &= ~CKEN14_I2C;

#ifdef CONFIG_POWER_I2C
		if (i2c_pxa_pwr_data.irq)
		          disable_irq(i2c_pxa_pwr_data.irq);
		/* disable PI2C Controller */
		PCFR &= ~PCFR_PI2CEN;
		CKEN &= ~CKEN15_PWRI2C;
#endif		
		break;
	case PM_RESUME:
        /* enable clocks */
        CKEN |= CKEN14_I2C;

        set_GPIO_mode(117 | GPIO_ALT_FN_1_IN);
        set_GPIO_mode(118 | GPIO_ALT_FN_1_IN);
        /* enable irqs */
        enable_irq(i2c_pxa_data.irq);
#ifdef CONFIG_POWER_I2C
      	/* set the global PWR_I2C clock on */
        CKEN |= CKEN15_PWRI2C;

		/* Enable PI2C controller PWR_SCL and PWR_SDA */
		PCFR |= PCFR_PI2CEN;
        set_GPIO_mode(3 | GPIO_ALT_FN_1_IN);
        set_GPIO_mode(4 | GPIO_ALT_FN_1_IN);
		enable_irq(i2c_pxa_pwr_data.irq);
#endif	
	
 	    /* disable unit */
		_ICR(adap) &= ~ICR_IUE;

	    /* reset the unit */
		_ICR(adap) |= ICR_UR;
	    udelay(100);
		_ICR(adap) |= ICR_FM;
        /* disable unit */
		_ICR(adap) &= ~ICR_IUE;

		/* XXX: I2C_PXA_SLAVE_ADDR == I2C_PXA_PWR_SLAVE_ADDR ??? */
	    /* set our slave address */
		_ISAR(adap) = I2C_PXA_SLAVE_ADDR;

	    /* set control register values */
		_ICR(adap) = I2C_ICR_INIT;

    	/* set clear interrupt bits */
		_ISR(adap) = I2C_ISR_INIT;

        /* enable unit */
		_ICR(adap) |= ICR_IUE;
        udelay(100);	
		break;
	}
	return 0;
}
#endif
#endif

static int __init i2c_adap_pxa_init(void)
{
        if( i2c_pxa_resource_init() == 0) {

                if (i2c_pxa_add_bus(&i2c_pxa_ops) < 0) {
                        i2c_pxa_resource_release(&i2c_pxa_data);
                        printk(KERN_INFO "I2C: Failed to add bus\n");
                        return -ENODEV;
                }
        } else {
                printk(KERN_INFO "I2C: Failed to add bus\n");
                return -ENODEV;
        }

        printk(KERN_INFO "I2C: Successfully added bus\n");

#ifdef CONFIG_POWER_I2C
        if( i2c_pxa_pwr_resource_init() == 0) {

                if (i2c_pxa_add_bus(&i2c_pxa_pwr_ops) < 0) {
			i2c_pxa_del_bus( &i2c_pxa_ops);
                        i2c_pxa_resource_release(&i2c_pxa_data);
		        printk(KERN_INFO "I2C: Successfully removed bus\n");

                        i2c_pxa_resource_release(&i2c_pxa_pwr_data);
                        printk(KERN_INFO "PWRI2C: Failed to add bus\n");
                        return -ENODEV;
                }
        } else {
		i2c_pxa_del_bus( &i2c_pxa_ops);
		i2c_pxa_resource_release(&i2c_pxa_data);
	        printk(KERN_INFO "I2C: Successfully removed bus\n");
                printk(KERN_INFO "PWRI2C: Failed to add bus\n");
                return -ENODEV;
        }

        printk(KERN_INFO "PWRI2C: Successfully added bus\n");
#endif
		
#ifdef CONFIG_DPM
	pxai2c_ldm_register();
#endif /* CONFIG_DPM */ 
#ifdef CONFIG_PM
//    	pm_dev = pm_register(PM_SYS_DEV, 0, ezx_i2c_pm_callback);
#endif   
	printk("call i2c_pxa_reset \n");
	i2c_pxa_reset(&i2c_pxa_data);
        return 0;
}

static void i2c_adap_pxa_exit(void)
{
#ifdef CONFIG_DPM
	pxai2c_ldm_unregister();
#endif /* CONFIG_DPM */ 
#ifdef CONFIG_PM
//	    pm_unregister(pm_dev);
#endif
        i2c_pxa_del_bus(&i2c_pxa_ops);
        i2c_pxa_resource_release(&i2c_pxa_data);
        printk(KERN_INFO "I2C: Successfully removed bus\n");

#ifdef CONFIG_POWER_I2C
        i2c_pxa_del_bus(&i2c_pxa_pwr_ops);
        i2c_pxa_resource_release(&i2c_pxa_pwr_data);
        printk(KERN_INFO "PWRI2C: Successfully removed bus\n");
#endif

}

MODULE_AUTHOR("Intrinsyc Software Inc., MontaVista Software Inc.");
MODULE_LICENSE("GPL");

module_init(i2c_adap_pxa_init);
module_exit(i2c_adap_pxa_exit);
