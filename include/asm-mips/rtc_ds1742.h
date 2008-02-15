/*
 * linux/include/asm-mip/rtc_ds1742.h
 *
 * driver for Dallas DS1742 RTC
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * Copyright 2001-2002 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _LINUX_INCLUDE_LINUX_RTC_DS1742_H_
#define _LINUX_INCLUDE_LINUX_RTC_DS1742_H_


#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/rtc.h>
#include <linux/time.h>
#include <asm/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/system.h>

#ifndef BCD_TO_BIN
#define BCD_TO_BIN(val) (((val)&15) + ((val)>>4)*10)
#endif

#ifndef BIN_TO_BCD
#define BIN_TO_BCD(val) ((((val)/10)<<4) + (val)%10)
#endif

/* register offsets */
#define RTC_DS1742_YEAR     0x7ff   /* 00 - 99 tm.tm_year%100 */ 
#define RTC_DS1742_MONTH    0x7fe   /* 01 - 12 tm.tm_mon      */ 
#define RTC_DS1742_DATE     0x7fd   /* 01 - 31 tm.tm_mday     */ 
#define RTC_DS1742_DAY      0x7fc   /* 01 - 07 tm.tm_wday     */ 
#define RTC_DS1742_HOUR     0x7fb   /* 00 - 23 tm.tm_hour     */ 
#define RTC_DS1742_MINUTE   0x7fa   /* 00 - 59 tm.tm_min      */ 
#define RTC_DS1742_SECOND   0x7f9   /* 00 - 59 tm.tm_sec      */ 
#define RTC_DS1742_CENTURY  0x7f8   /* 00 - 39 tm.tm_year/100 */ 


/* register field masks */
#define RTC_DS1742_MASK_YEAR        0xff
#define RTC_DS1742_MASK_MONTH       0x1f
#define RTC_DS1742_MASK_DATE        0x3f
#define RTC_DS1742_MASK_DAY         0x07
#define RTC_DS1742_MASK_DAY_FT      0x40    
#define RTC_DS1742_MASK_DAY_BF      0x80    
#define RTC_DS1742_MASK_HOUR        0x7f
#define RTC_DS1742_MASK_MINUTE      0x7f    
#define RTC_DS1742_MASK_SECOND      0x7f    
#define RTC_DS1742_MASK_SECOND_OSC  0x80  /* oscolator enable */
#define RTC_DS1742_MASK_CENTURY     0x3f    
#define RTC_DS1742_MASK_CENTURY_RD  0x40  /* read rtc lock */
#define RTC_DS1742_MASK_CENTURY_WR  0x80  /* write rtc lock */


extern unsigned long rtc_ds1742_base;


#define RTC_DS1742_RD(reg)      (*(volatile u8*)(rtc_ds1742_base+(reg)))
#define RTC_DS1742_WR(reg,val) ((*(volatile u8*)(rtc_ds1742_base+(reg)))=(val))

#define RTC_DS1742_CLR(reg,msk) ( RTC_DS1742_WR( (reg), RTC_DS1742_RD( (reg) ) & ~(msk) ) )
#define RTC_DS1742_SET(reg,msk) ( RTC_DS1742_WR( (reg), RTC_DS1742_RD( (reg) ) |  (msk) ) )

unsigned long
rtc_ds1742_get_time( void );


int
rtc_ds1742_set_time( unsigned long t );


void __init
rtc_ds1742_init( unsigned long base );


extern inline u8
rtc_ds1742_atomic_rdrw_lock_beg( u8 mask )
{
  u8 orig;

  orig = RTC_DS1742_RD( RTC_DS1742_CENTURY );
  RTC_DS1742_WR( RTC_DS1742_CENTURY, orig|mask );

  return( orig );
}
#define rtc_ds1742_atomic_rd_lock_beg() rtc_ds1742_atomic_rdrw_lock_beg(RTC_DS1742_MASK_CENTURY_RD)
#define rtc_ds1742_atomic_wr_lock_beg() rtc_ds1742_atomic_rdrw_lock_beg(RTC_DS1742_MASK_CENTURY_WR)


extern inline void
rtc_ds1742_atomic_rdwr_lock_end( u8 orig )
{
  RTC_DS1742_WR( RTC_DS1742_CENTURY, orig );

  return;
}
#define rtc_ds1742_atomic_rd_lock_end(orig) rtc_ds1742_atomic_rdwr_lock_end(orig)
#define rtc_ds1742_atomic_wr_lock_end(orig) rtc_ds1742_atomic_rdwr_lock_end(orig)


#define rtc_ds1742_rd_century() (u8)BCD_TO_BIN( ( RTC_DS1742_RD( RTC_DS1742_CENTURY ) & RTC_DS1742_MASK_CENTURY  ) )
#define rtc_ds1742_rd_year()    (u8)BCD_TO_BIN( ( RTC_DS1742_RD( RTC_DS1742_YEAR    ) & RTC_DS1742_MASK_YEAR     ) )
#define rtc_ds1742_rd_month()   (u8)BCD_TO_BIN( ( RTC_DS1742_RD( RTC_DS1742_MONTH   ) & RTC_DS1742_MASK_MONTH    ) )
#define rtc_ds1742_rd_date()    (u8)BCD_TO_BIN( ( RTC_DS1742_RD( RTC_DS1742_DATE    ) & RTC_DS1742_MASK_DATE     ) )
#define rtc_ds1742_rd_day()     (u8)BCD_TO_BIN( ( RTC_DS1742_RD( RTC_DS1742_DAY     ) & RTC_DS1742_MASK_DAY      ) )
#define rtc_ds1742_rd_hour()    (u8)BCD_TO_BIN( ( RTC_DS1742_RD( RTC_DS1742_HOUR    ) & RTC_DS1742_MASK_HOUR     ) )
#define rtc_ds1742_rd_minute()  (u8)BCD_TO_BIN( ( RTC_DS1742_RD( RTC_DS1742_MINUTE  ) & RTC_DS1742_MASK_MINUTE   ) )
#define rtc_ds1742_rd_second()  (u8)BCD_TO_BIN( ( RTC_DS1742_RD( RTC_DS1742_SECOND  ) & RTC_DS1742_MASK_SECOND   ) )


#define rtc_ds1742_wr_century(val) RTC_DS1742_WR( RTC_DS1742_CENTURY, BIN_TO_BCD( (val) ) & RTC_DS1742_MASK_CENTURY )
#define rtc_ds1742_wr_year(val)    RTC_DS1742_WR( RTC_DS1742_YEAR,    BIN_TO_BCD( (val) ) & RTC_DS1742_MASK_YEAR    )
#define rtc_ds1742_wr_month(val)   RTC_DS1742_WR( RTC_DS1742_MONTH,   BIN_TO_BCD( (val) ) & RTC_DS1742_MASK_MONTH   )
#define rtc_ds1742_wr_date(val)    RTC_DS1742_WR( RTC_DS1742_DATE,    BIN_TO_BCD( (val) ) & RTC_DS1742_MASK_DATE    )
#define rtc_ds1742_wr_day(val)     RTC_DS1742_WR( RTC_DS1742_DAY,     BIN_TO_BCD( (val) ) & RTC_DS1742_MASK_DAY     )
#define rtc_ds1742_wr_hour(val)    RTC_DS1742_WR( RTC_DS1742_HOUR,    BIN_TO_BCD( (val) ) & RTC_DS1742_MASK_HOUR    )
#define rtc_ds1742_wr_minute(val)  RTC_DS1742_WR( RTC_DS1742_MINUTE,  BIN_TO_BCD( (val) ) & RTC_DS1742_MASK_MINUTE  )
#define rtc_ds1742_wr_second(val)  RTC_DS1742_WR( RTC_DS1742_SECOND,  BIN_TO_BCD( (val) ) & RTC_DS1742_MASK_SECOND  )


/* wait for seconds to change */
extern inline void 
rtc_ds1742_wait( void )
{
  u8 val;

  val = RTC_DS1742_RD( RTC_DS1742_SECOND );
  while ( val == RTC_DS1742_RD( RTC_DS1742_SECOND ) );

  return;
}

#endif
