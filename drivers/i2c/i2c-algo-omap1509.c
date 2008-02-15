/*
   -------------------------------------------------------------------------
   i2c-algo-omap1509.c i2c driver algorithms for OMAP support
   
   Steve Johnson, RidgeRun, Inc.

   Copyright 2002 RidgeRun, Inc.

   ---------------------------------------------------------------------------
   This file was highly leveraged from i2c-algo-ite.c, which was created
   by Hai-Pao Fan:

   Hai-Pao Fan, MontaVista Software, Inc.
   hpfan@mvista.com or source@mvista.com

   Copyright 2000 MontaVista Software Inc.

   ---------------------------------------------------------------------------
   This file was highly leveraged from i2c-algo-pcf.c, which was created
   by Simon G. Vogl and Hans Berglund:


     Copyright (C) 1995-1997 Simon G. Vogl
                   1998-2000 Hans Berglund

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.		     */
/* ------------------------------------------------------------------------- */

/* With some changes from Kyösti Mälkki <kmalkki@cc.hut.fi> and 
   Frodo Looijaard <frodol@dds.nl> ,and also from Martin Bailey
   <mbailey@littlefeet-inc.com> */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include "i2c-algo-omap1509.h"
#include "i2c-omap1509.h"

/* ----- global defines ----------------------------------------------- */
#define DEB(x) if (i2c_debug>=1) x
#define DEB2(x) if (i2c_debug>=2) x
#define DEB3(x) if (i2c_debug>=3) x /* print several statistical values*/
#define DEBPROTO(x) if (i2c_debug>=9) x;
 	/* debug the protocol by showing transferred bits */
#define DEF_TIMEOUT 16


/* ----- global variables ---------------------------------------------	*/

#ifdef SLO_IO
	int jif;
#endif

/* module parameters:
 */
static int i2c_debug;
static int omap1509_test;	/* see if the line-setting functions work	*/
static int omap1509_scan;	/* have a look at what's hanging 'round		*/

/* --- setting states on the bus with the right timing: ---------------	*/

#define get_clock(adap) adap->getclock(adap->data)
#define omap1509_outw(adap, reg, val) adap->setomap1509(adap->data, reg, val)
#define omap1509_inw(adap, reg) adap->getomap1509(adap->data, reg)


/* --- other auxiliary functions --------------------------------------	*/
static inline int try_address(struct i2c_algo_omap1509_data *adap,
                              unsigned int addr, int retries);


static void omap1509_start(struct i2c_algo_omap1509_data *adap)
{
	omap1509_outw(adap, I2C_CMD_REG, omap1509_inw(adap, I2C_CMD_REG) | I2C_START_WRITE);
}

static void omap1509_start_clock(struct i2c_algo_omap1509_data *adap)
{
	omap1509_outw(adap, I2C_CMD_REG, omap1509_inw(adap, I2C_CMD_REG) | I2C_EN_CLK);
}

static void omap1509_stop(struct i2c_algo_omap1509_data *adap)
{
	omap1509_outw(adap, I2C_CMD_REG, omap1509_inw(adap, I2C_CMD_REG) & ~I2C_START_WRITE);
}

static void omap1509_reset(struct i2c_algo_omap1509_data *adap)
{
	omap1509_outw(adap, I2C_CMD_REG, omap1509_inw(adap, I2C_CMD_REG) | I2C_SOFT_RESET);
        udelay(100);
	omap1509_outw(adap, I2C_CMD_REG, omap1509_inw(adap, I2C_CMD_REG) & ~I2C_SOFT_RESET);
}

/*
  Waiting on Bus Busy
*/
static int wait_for_bb(struct i2c_algo_omap1509_data *adap)
{
	int timeout = DEF_TIMEOUT;
	short status;

	status = omap1509_inw(adap, I2C_STATUS_ACTIVITY_REG);
#ifndef STUB_I2C
	while (timeout-- && (status & I2C_NOT_IDLE)) {
		udelay(1000); /* How much is this? */
		status = omap1509_inw(adap, I2C_STATUS_ACTIVITY_REG);
	}
#endif
	if (timeout<=0) {
		printk(KERN_ERR "Timeout, host is busy\n");
		omap1509_reset(adap);
	}
	return(timeout<=0);
}

/*
 * Puts this process to sleep for a period equal to timeout 
 */
static inline void omap1509_sleep(unsigned long timeout)
{
	schedule_timeout( timeout * HZ);
}

/* After we issue a transaction on the OMAP1509 bus, this function
 * is called.  It puts this process to sleep until we get an interrupt from
 * the controller telling us that the transaction we requested in complete.
 * pin = Pending Interrupt Not
 */
static int wait_for_pin(struct i2c_algo_omap1509_data *adap, u16 *status) 
{

	int timeout = DEF_TIMEOUT * 3;
	
	*status = omap1509_inw(adap, I2C_STATUS_ACTIVITY_REG);
//        DEB2(printk(__FUNCTION__ ": status 0x%x, status-and %x\n", *status, (*status & I2C_INTERRUPT)));
#ifndef STUB_I2C
	while (timeout-- && 
//               ((*status & I2C_ERROR) == 0) &&
//               ((*status & I2C_INTERRUPT) != I2C_INTERRUPT) &&
               ((*status & I2C_NOT_IDLE) == I2C_NOT_IDLE)) {
                adap->waitforpin();
                *status = omap1509_inw(adap, I2C_STATUS_ACTIVITY_REG);
//                DEB2(printk(__FUNCTION__ ": status 0x%x, status-and %x\n", *status, (*status & I2C_INTERRUPT)));
	}
#endif
	if (timeout <= 0) {
                DEB2(printk(__FUNCTION__ ": status 0x%x\n", *status));
		return(-1);
        }
	else
		return(0);
}

/*
  FIFO Empty
*/
static int wait_for_fe(struct i2c_algo_omap1509_data *adap, u16 *status)
{
	int timeout = DEF_TIMEOUT;

	*status = omap1509_inw(adap, I2C_STATUS_FIFO_REG);
#ifndef STUB_I2C 
	while (timeout-- && !(*status & I2C_FIFO_EMPTY)) {
		udelay(1000);
		omap1509_inw(adap, I2C_STATUS_FIFO_REG);
	}
#endif
	if (timeout <= 0) 
		return(-1);
	else
		return(0);
}

static int omap1509_init (struct i2c_algo_omap1509_data *adap)
{
        u16 command_register;
        u16 clock;

        // Initialize OMAP1509 I2C hardware       

        // Disable i2c clock
        command_register = omap1509_inw(adap, I2C_CMD_REG);
        command_register &= ~I2C_EN_CLK;
        omap1509_outw(adap, I2C_CMD_REG, command_register);

        // Set clocks for standard 100kHz transmission mode
        // set div2 = 9 ext_frq/(div1/div2 + 1)
        clock = get_clock(adap);
        omap1509_outw(adap, I2C_CONF_CLK_REF_REG, (clock & 0xff));
        // set spike filter 2 ( = 3 master clocks), div1 2 ( = 4 )
        omap1509_outw(adap, I2C_CONF_CLK_REG, ((clock & 0xff00) >> 8));

        // Enable i2c clock & interrupts
        command_register |= (I2C_EN_CLK | I2C_IRQ_EN);
        omap1509_outw(adap, I2C_CMD_REG, command_register);

        // Reset FIFO
        command_register |= I2C_SOFT_RESET;
        omap1509_outw(adap, I2C_CMD_REG, command_register);
        command_register &= ~I2C_SOFT_RESET;
        omap1509_outw(adap, I2C_CMD_REG, command_register);

        // Set FIFO to 0 ( = 1 byte )
        omap1509_outw(adap, I2C_CONF_FIFO_REG, 0xf0);

	DEB2(printk("omap1509_init: Initialized OMAP1509 I2C Status 0x%x\n",
                    omap1509_inw(adap, I2C_STATUS_ACTIVITY_REG)));
	return 0;
}


/*
 * Sanity check for the adapter hardware - check the reaction of
 * the bus lines only if it seems to be idle.
 */
static int test_bus(struct i2c_algo_omap1509_data *adap, char *name) 
{
#if 0
	int scl,sda;
	sda=getsda(adap);
	if (adap->getscl==NULL) {
		printk("test_bus: Warning: Adapter can't read from clock line - skipping test.\n");
		return 0;		
	}
	scl=getscl(adap);
	printk("test_bus: Adapter: %s scl: %d  sda: %d -- testing...\n",
               name,getscl(adap),getsda(adap));
	if (!scl || !sda ) {
		printk("test_bus: %s seems to be busy.\n",adap->name);
		goto bailout;
	}
	sdalo(adap);
	printk("test_bus:1 scl: %d  sda: %d \n",getscl(adap),
	       getsda(adap));
	if ( 0 != getsda(adap) ) {
		printk("test_bus: %s SDA stuck high!\n",name);
		sdahi(adap);
		goto bailout;
	}
	if ( 0 == getscl(adap) ) {
		printk("test_bus: %s SCL unexpected low while pulling SDA low!\n",
                       name);
		goto bailout;
	}		
	sdahi(adap);
	printk("test_bus:2 scl: %d  sda: %d \n",getscl(adap),
	       getsda(adap));
	if ( 0 == getsda(adap) ) {
		printk("test_bus: %s SDA stuck low!\n",name);
		sdahi(adap);
		goto bailout;
	}
	if ( 0 == getscl(adap) ) {
		printk("test_bus: %s SCL unexpected low while SDA high!\n",
		       adap->name);
                goto bailout;
	}
	scllo(adap);
	printk("test_bus:3 scl: %d  sda: %d \n",getscl(adap),
	       getsda(adap));
	if ( 0 != getscl(adap) ) {

		sclhi(adap);
		goto bailout;
	}
	if ( 0 == getsda(adap) ) {
		printk("test_bus: %s SDA unexpected low while pulling SCL low!\n",
                       name);
		goto bailout;
	}
	sclhi(adap);
	printk("test_bus:4 scl: %d  sda: %d \n",getscl(adap),
	       getsda(adap));
	if ( 0 == getscl(adap) ) {
		printk("test_bus: %s SCL stuck low!\n",name);
		sclhi(adap);
		goto bailout;
	}
	if ( 0 == getsda(adap) ) {
		printk("test_bus: %s SDA unexpected low while SCL high!\n",
                       name);
		goto bailout;
	}
	printk("test_bus: %s passed test.\n",name);
	return 0;
 bailout:
	sdahi(adap);
	sclhi(adap);
	return -ENODEV;
#endif
	return (0);
}

/* ----- Utility functions
 */


/* Verify the device we want to talk to on the OMAP1509 bus really exists. */
static inline int try_address(struct i2c_algo_omap1509_data *adap,
                              unsigned int addr, int retries)
{
	int i = 0, ret = -1;
	short status;

	for (i=0;i<retries;i++) {
		omap1509_start(adap);
		if (wait_for_pin(adap, &status) == 0) {
			if ((status & I2C_ERROR_DEVICE) == 0) { 
//				omap1509_stop(adap);
                                omap1509_reset(adap);   // flush FIFO
				ret=1;
				break;	/* success! */
			}
		}
                DEB2(printk(__FUNCTION__ ": status 0x%x\n", status));
		omap1509_stop(adap);
		udelay(adap->udelay);
	}

	DEB2(if (i) printk("try_address: needed %d retries for 0x%x\n",i,
	                   addr));
	return ret;
}


static int omap1509_sendbytes(struct i2c_adapter *i2c_adap,
                              const char *buf, int count)
{
	struct i2c_algo_omap1509_data *adap = i2c_adap->algo_data;
	int wrcount=0, timeout;
	short status;
	int loops, remainder, i, j;
   
	omap1509_outw(adap, I2C_ADDRESS_REG, (unsigned short)buf[wrcount++]);
	count--;
	if (count == 0)
		return -EIO;

	loops =  count / I2C_FIFO_MAX;		/* 16-byte FIFO */
	remainder = count % I2C_FIFO_MAX;

	if(loops) {
		for(i=0; i<loops; i++) {

			omap1509_outw(adap, I2C_CONF_FIFO_REG, I2C_FIFO_MASK);
			for(j=0; j<I2C_FIFO_MAX; j++) {
				omap1509_outw(adap, I2C_DATA_WRITE_REG, buf[wrcount++]); 
			}

                        if (!(omap1509_inw(adap, I2C_STATUS_FIFO_REG) & I2C_FIFO_FULL)) {
                                DEB(printk(__FUNCTION__ ": FIFO not full when it should be\n"));
                        }
 
                        /* Issue WRITE command */
                        omap1509_outw(adap, I2C_CMD_REG, omap1509_inw(adap, I2C_CMD_REG) | I2C_START_WRITE);

			/* Wait for transmission to complete */
			timeout = wait_for_pin(adap, &status);
			if(timeout) {
				omap1509_stop(adap);
				printk(__FUNCTION__ ": %s write timeout.\n", i2c_adap->name);
				return -EREMOTEIO; /* got a better one ?? */
                        }
			if (status & I2C_ERROR) {
				omap1509_stop(adap);
				printk(__FUNCTION__ ": %s write error - no ack.\n", i2c_adap->name);
				return -EREMOTEIO; /* got a better one ?? */
			}
		}
	}
	if(remainder) {
		omap1509_outw(adap, I2C_CONF_FIFO_REG, (remainder - 1));  // 0 based register
		for(i=0; i < remainder; i++) {
			omap1509_outw(adap, I2C_DATA_WRITE_REG, buf[wrcount++]);
		}

                if (!(omap1509_inw(adap, I2C_STATUS_FIFO_REG) & I2C_FIFO_FULL)) {
                        DEB(printk(__FUNCTION__ ": FIFO not full when it should be\n"));
                }
 
                /* Issue WRITE command */
                omap1509_outw(adap, I2C_CMD_REG, omap1509_inw(adap, I2C_CMD_REG) | I2C_START_WRITE);

		timeout = wait_for_pin(adap, &status);
		if(timeout) {
			omap1509_stop(adap);
			printk(__FUNCTION__ ": %s write timeout.\n", i2c_adap->name);
			return -EREMOTEIO; /* got a better one ?? */
		}
#ifndef STUB_I2C
		if (status & I2C_ERROR) { 
			omap1509_stop(adap);
			printk(__FUNCTION__ ": %s write error - no ack.\n", i2c_adap->name);
			return -EREMOTEIO; /* got a better one ?? */
		}
#endif
	}
	omap1509_stop(adap);
	return wrcount;
}


static int omap1509_readbytes(struct i2c_adapter *i2c_adap, char *buf, int count)
{
	int rdcount=0, i, timeout;
	short status;
	struct i2c_algo_omap1509_data *adap = i2c_adap->algo_data;
	int loops, remainder, j, bytes_read;
		
	loops = count / I2C_FIFO_MAX;				/* 16-byte FIFO */
	remainder = count % I2C_FIFO_MAX;

	if(loops) {
		for(i=0; i<loops; i++) {

			omap1509_outw(adap, I2C_CONF_FIFO_REG, I2C_FIFO_MASK);

                        /* Issue READ command */
                        omap1509_outw(adap, I2C_CMD_REG, omap1509_inw(adap, I2C_CMD_REG) | I2C_START_READ);

			timeout = wait_for_pin(adap, &status);
			if(timeout) {
				omap1509_stop(adap);
				printk(__FUNCTION__ ":  %s read timeout.\n", i2c_adap->name);
				return (-1);
			}
#ifndef STUB_I2C
			if (status & I2C_ERROR) {
				omap1509_stop(adap);
				printk(__FUNCTION__ ": %s read error - no ack.\n", i2c_adap->name);
				return (-1);
			}
#endif

			timeout = wait_for_fe(adap, &status);
			if(timeout) {
				omap1509_stop(adap);
				printk(__FUNCTION__ ":  %s FIFO is empty\n", i2c_adap->name);
				return (-1); 
			}
                        bytes_read = ((status >> I2C_READ_CPT_SHIFT) & 0xf);
                        if (bytes_read != I2C_FIFO_MAX) {
                                printk(__FUNCTION__ ": returned %d bytes instead of 16\n", bytes_read);
                        }
			for(j=0; j < bytes_read; j++) {
				buf[rdcount++] = (omap1509_inw(adap, I2C_READ) & 0xf);
			}
		}
	}


	if(remainder) {
                omap1509_outw(adap, I2C_CONF_FIFO_REG, remainder);

                /* Issue READ command */
                omap1509_outw(adap, I2C_CMD_REG, omap1509_inw(adap, I2C_CMD_REG) | I2C_START_READ);

		timeout = wait_for_pin(adap, &status);
		if(timeout) {
			omap1509_stop(adap);
			printk(__FUNCTION__ ":  %s read timeout.\n", i2c_adap->name);
			return (-1);
		}
#ifndef STUB_I2C
		if (status & I2C_ERROR) {
			omap1509_stop(adap);
			printk(__FUNCTION__ ": %s read error - no ack.\n", i2c_adap->name);
			return (-1);
		}
#endif
		timeout = wait_for_fe(adap, &status);
		if(timeout) {
			omap1509_stop(adap);
			printk(__FUNCTION__ ":  %s FIFO is empty\n", i2c_adap->name);
			return (-1);
		}         

                bytes_read = ((status >> I2C_READ_CPT_SHIFT) & 0xf);
                if (bytes_read != I2C_FIFO_MAX) {
                        printk(__FUNCTION__ ": returned %d bytes instead of 16\n", bytes_read);
                }
		for(i=0; i < bytes_read; i++) {
                        buf[rdcount++] = (omap1509_inw(adap, I2C_READ) & 0xf);
                }

	}

	omap1509_stop(adap);
	return rdcount;
}


/* This function implements combined transactions.  Combined
 * transactions consist of combinations of reading and writing blocks of data.
 * Each transfer (i.e. a read or a write) is separated by a repeated start
 * condition.
 */
#if 0
static int omap1509_combined_transaction(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[], int num) 
{
        int i;
        struct i2c_msg *pmsg;
        int ret;

        DEB2(printk("Beginning combined transaction\n"));

        for(i=0; i<(num-1); i++) {
                pmsg = &msgs[i];
                if(pmsg->flags & I2C_M_RD) {
                        DEB2(printk("  This one is a read\n"));
                        ret = omap1509_readbytes(i2c_adap, pmsg->buf, pmsg->len, OMAP1509_COMBINED_XFER);
                }
                else if(!(pmsg->flags & I2C_M_RD)) {
                        DEB2(printk("This one is a write\n"));
                        ret = omap1509_sendbytes(i2c_adap, pmsg->buf, pmsg->len, OMAP1509_COMBINED_XFER);
                }
        }
        /* Last read or write segment needs to be terminated with a stop */
        pmsg = &msgs[i];

        if(pmsg->flags & I2C_M_RD) {
                DEB2(printk("Doing the last read\n"));
                ret = omap1509_readbytes(i2c_adap, pmsg->buf, pmsg->len, OMAP1509_SINGLE_XFER);
        }
        else if(!(pmsg->flags & I2C_M_RD)) {
                DEB2(printk("Doing the last write\n"));
                ret = omap1509_sendbytes(i2c_adap, pmsg->buf, pmsg->len, OMAP1509_SINGLE_XFER);
        }

        return ret;
}
#endif


/* Whenever we initiate a transaction, the first byte clocked
 * onto the bus after the start condition is the address (7 bit) of the
 * device we want to talk to.  This function manipulates the address specified
 * so that it makes sense to the hardware when written to the OMAP1509 peripheral.
 *
 * Note: 10 bit addresses are not supported in this driver, although they are
 * supported by the hardware.  This functionality needs to be implemented.
 */
static inline int omap1509_doAddress(struct i2c_algo_omap1509_data *adap,
                                     struct i2c_msg *msg, int retries) 
{
	unsigned short flags = msg->flags;
	unsigned int addr;
	int ret;

/* Ten bit addresses not supported right now */
	if ( (flags & I2C_M_TEN)  ) { 
#if 0
		addr = 0xf0 | (( msg->addr >> 7) & 0x03);
		DEB2(printk("addr0: %d\n",addr));
		ret = try_address(adap, addr, retries);
		if (ret!=1) {
			printk(__FUNCTION__ ": died at extended address code.\n");
			return -EREMOTEIO;
		}
		omap1509_outw(adap,msg->addr & 0x7f);
		if (ret != 1) {
			printk(__FUNCTION__ ": died at 2nd address code.\n");
			return -EREMOTEIO;
		}
		if ( flags & I2C_M_RD ) {
			i2c_repstart(adap);
			addr |= 0x01;
			ret = try_address(adap, addr, retries);
			if (ret!=1) {
				printk(__FUNCTION__ ": died at extended address code.\n");
				return -EREMOTEIO;
			}
		}
#else
                return -EREMOTEIO;
#endif
	} else {

		addr = ( msg->addr << 1 );

                /*
                  We have the R/W bit of I2C_CMD_REG to set the RD bit,
                  so don't use the standard bit-banging code.
                */
#if 0
		if (flags & I2C_M_RD )
			addr |= 1;
		if (flags & I2C_M_REV_DIR_ADDR )
			addr ^= 1;
#endif

		if ((omap1509_inw(adap, I2C_DEVICE_REG) & 0x7f) != addr) {
			omap1509_outw(adap, I2C_DEVICE_REG, addr);
			ret = try_address(adap, addr, retries);
			if (ret!=1) {
				DEB(printk(__FUNCTION__ ": died at address code.\n"));
				return -EREMOTEIO;
			}
		}

        }

	return 0;
}


/* Description: Prepares the controller for a transaction (clearing status
 * registers, data buffers, etc), and then calls either omap1509_readbytes or
 * omap1509_sendbytes to do the actual transaction.
 *
 * still to be done: Before we issue a transaction, we should
 * verify that the bus is not busy or in some unknown state.
 */
static int omap1509_xfer(struct i2c_adapter *i2c_adap,
                         struct i2c_msg msgs[], 
                         int num)
{
	struct i2c_algo_omap1509_data *adap = i2c_adap->algo_data;
	struct i2c_msg *pmsg;
	int i = 0;
	int ret, timeout;
       
	pmsg = &msgs[i];

        // length 0 generally means a probe, so don't fail.
	if(!pmsg->len) {
		DEB2(printk(__FUNCTION__ ": read/write length is 0\n");)
//                        return -EIO;
	}

        omap1509_start_clock(adap);

	/* Wait for any pending transfers to complete */
	timeout = wait_for_bb(adap);
	if (timeout) {
		DEB2(printk(__FUNCTION__ ": Timeout waiting for host not busy\n");)
                        return -EIO;
	}

	/* Flush FIFO */
	omap1509_reset(adap);

	/* Load address */
	ret = omap1509_doAddress(adap, pmsg, i2c_adap->retries);
	if (ret) {
		return -EIO;
        }
        
#if 0
	/* Combined transaction (read and write) */
	if(num > 1) {
                DEB2(printk(__FUNCTION__ ": Call combined transaction\n"));
                ret = omap1509_combined_transaction(i2c_adap, msgs, num);
        }
#endif
        
	DEB3(printk(__FUNCTION__ ": Msg %d, addr=0x%x, flags=0x%x, len=%d\n",
                    i, msgs[i].addr, msgs[i].flags, msgs[i].len));
        
        if (pmsg->len) {
                if(pmsg->flags & I2C_M_RD) 		/* Read */
                        ret = omap1509_readbytes(i2c_adap, pmsg->buf, pmsg->len);
                else {													/* Write */ 
//                udelay(1000);
                        ret = omap1509_sendbytes(i2c_adap, pmsg->buf, pmsg->len);
                }
        }
        
	if (ret != pmsg->len) {
		DEB2(printk(__FUNCTION__ ": error or fail on read/write %d bytes.\n",ret)); 
        }
	else {
		DEB3(printk(__FUNCTION__ ": read/write %d bytes.\n",ret));
        }
        
        omap1509_stop(adap);
	return ret;
}


/* Implements device specific ioctls.  Higher level ioctls can
 * be found in i2c-core.c and are typical of any i2c controller (specifying
 * slave address, timeouts, etc).  These ioctls take advantage of any hardware
 * features built into the controller for which this algorithm-adapter set
 * was written.  These ioctls allow you to take control of the data and clock
 * lines and set the either high or low,
 * similar to a GPIO pin.
 */
static int algo_control(struct i2c_adapter *adapter, 
                        unsigned int cmd, unsigned long arg)
{

        struct i2c_algo_omap1509_data *adap = adapter->algo_data;
        struct i2c_omap1509_msg s_msg;
        char *buf;
	int ret;

        if (cmd == I2C_SREAD) {
		if(copy_from_user(&s_msg, (struct i2c_omap1509_msg *)arg, 
                                  sizeof(struct i2c_omap1509_msg))) 
			return -EFAULT;
		buf = kmalloc(s_msg.len, GFP_KERNEL);
		if (buf== NULL)
			return -ENOMEM;

		/* Flush FIFO */
                omap1509_reset(adap);

		/* Load address */
		omap1509_outw(adap, I2C_DEVICE_REG, s_msg.addr<<1);
		omap1509_outw(adap, I2C_ADDRESS_REG, s_msg.waddr & 0xff);

		ret = omap1509_readbytes(adapter, buf, s_msg.len);
		if (ret>=0) {
			if(copy_to_user( s_msg.buf, buf, s_msg.len) ) 
				ret = -EFAULT;
		}
		kfree(buf);
	}
	return 0;
}


#if 0            // use default instead
static u32 omap1509_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C;
}
#endif

/* -----exported algorithm data: -------------------------------------	*/

static struct i2c_algorithm omap1509_algo = {
	"OMAP1509 I2C algorithm",
	I2C_ALGO_EXP,
	omap1509_xfer,		/* master_xfer	*/
	NULL,				/* smbus_xfer	*/
	NULL,				/* slave_xmit		*/
	NULL,				/* slave_recv		*/
	algo_control,			/* ioctl		*/
//	omap1509_func,			/* functionality	*/
};


/* 
 * registering functions to load algorithms at runtime 
 */
int i2c_omap1509_add_bus(struct i2c_adapter *adap)
{
	int i;
	short status;
	struct i2c_algo_omap1509_data *omap1509_adap = adap->algo_data;

	if (omap1509_test) {
		int ret = test_bus(omap1509_adap, adap->name);
		if (ret<0)
			return -ENODEV;
	}

	DEB2(printk("i2c-algo-omap1509: hw routines for %s registered.\n",
	            adap->name));

	/* register new adapter to i2c module... */

	adap->id |= omap1509_algo.id;
	adap->algo = &omap1509_algo;

	adap->timeout = 100;	/* default values, should	*/
	adap->retries = 3;	/* be replaced by defines	*/
	adap->flags = 0;

#ifdef MODULE
	MOD_INC_USE_COUNT;
#endif

	i2c_add_adapter(adap);
	omap1509_init(omap1509_adap);

	/* scan bus */
	/* By default scanning the bus is turned off. */
	if (omap1509_scan) {
		printk(KERN_INFO " i2c-algo-omap1509: scanning bus %s.\n",
		       adap->name);
		for (i = 0x00; i < 0x7f; i+=2) {
                        omap1509_reset(omap1509_adap);
                        while (omap1509_inw(omap1509_adap, I2C_STATUS_ACTIVITY_REG) & 0xf) {
                                omap1509_reset(omap1509_adap);
                        }
			omap1509_outw(omap1509_adap, I2C_DEVICE_REG, i);
			omap1509_start(omap1509_adap);
			if ( (wait_for_pin(omap1509_adap, &status) == 0) && 
                             ((status & I2C_ERROR_DEVICE) == 0) ) { 
				printk("\n(%02x)\n",i>>1); 
			} else {
				printk("."); 
			}
//			udelay(omap1509_adap->udelay);
//                        printk("status 0x%x, i 0x%x\n", status, i);
		}
	}
	return 0;
}


int i2c_omap1509_del_bus(struct i2c_adapter *adap)
{
	int res;
	if ((res = i2c_del_adapter(adap)) < 0)
		return res;
	DEB2(printk("i2c-algo-omap1509: adapter unregistered: %s\n",adap->name));

#ifdef MODULE
	MOD_DEC_USE_COUNT;
#endif
	return 0;
}


int __init i2c_algo_omap1509_init (void)
{
	printk(KERN_INFO "OMAP1509 (i2c) algorithm module\n");
	return 0;
}


void i2c_algo_omap1509_exit(void)
{
	return;
}


EXPORT_SYMBOL(i2c_omap1509_add_bus);
EXPORT_SYMBOL(i2c_omap1509_del_bus);

/* The MODULE_* macros resolve to nothing if MODULES is not defined
 * when this file is compiled.
 */
MODULE_AUTHOR("RidgeRun");
MODULE_DESCRIPTION("OMAP1509 I2C algorithm");
MODULE_LICENSE("GPL");

MODULE_PARM(omap1509_test, "i");
MODULE_PARM(omap1509_scan, "i");
MODULE_PARM(i2c_debug,"i");

MODULE_PARM_DESC(omap1509_test, "Test if the I2C bus is available");
MODULE_PARM_DESC(omap1509_scan, "Scan for active chips on the bus");
MODULE_PARM_DESC(i2c_debug,
        "debug level - 0 off; 1 normal; 2,3 more verbose; 9 omap1509-protocol");


/* This function resolves to init_module (the function invoked when a module
 * is loaded via insmod) when this file is compiled with MODULES defined.
 * Otherwise (i.e. if you want this driver statically linked to the kernel),
 * a pointer to this function is stored in a table and called
 * during the intialization of the kernel (in do_basic_setup in /init/main.c) 
 *
 * All this functionality is complements of the macros defined in linux/init.h
 */
module_init(i2c_algo_omap1509_init);


/* If MODULES is defined when this file is compiled, then this function will
 * resolved to cleanup_module.
 */
module_exit(i2c_algo_omap1509_exit);
