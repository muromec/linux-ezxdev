/*
 * Copyright (C) 2002-2005 Motorola Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* REVISION HISTORY 
	Date          Author			Reference
	========      ==========		==========================
	2002-07-02    weiqiang lin		create
        2004-02-09    weiqiang lin      update PM for Bulverde OS libdd74345.
        2004-04-13    weiqiang lin      update accessory usb attach and detach
        2004-04-27    weiqiang lin      send the USB accessory detach/attach information to PM
        2004-05-11    weiqiang lin      after sleep TC_MM_EN will be high else will be low
        2004-06-24    Cheng Xuefeng     remove the SSP_PCAP_configure_USB_UART_transeiver()
        2004-08-19    weiqiang lin      after sleep set SW1 output 1.3V and low power mode and when headset button press/release, send an event to PM 
        2004-08-31    weiqiang lin      remove the headset button press/release event to PM.
        2004-11-19    weiqiang lin      add some code to support PRIMARY SPI configure.
	2004-12-15    Jordan Wang	BARBADOS HANDSHAKE UPDATE 
        2005-05-12    weiqiang lin      remove warning code.
*/

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/init.h>
//#include <linux/apm_bios.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>

//#define  CONFIG_SSP_PCAP_SUPPORT_PRIMARY_SPI 

#ifndef TRUE
#define TRUE       1
#endif

#ifndef FALSE
#define FALSE      0
#endif

#include "ssp_pcap.h"
//#define  SSP_PCAP_OPERATE_WITH_SPI                                  

static u32 ssp_pcap_operate_pin_with_inverter;                
static u32 ssp_pcap_rxd_pin_with_inverter;                
//#define  SSP_PCAP_OPERATE_DEBUG_INFORNMATION                        

#ifdef _DEBUG_BOARD
#define  SSP_PCAP_DEBUG_BOARD                                       
#endif

/* 0 -- unlock   1- lock */
static u32 ssp_pcap_screenlock_status;
static struct timer_list ssp_pcap_screenlock_timer;

static U32 ssp_pcap_status = 0;
static u32 ssp_pcap_registerValue[SSP_PCAP_REGISTER_NUMBER] = {0};

SSP_PCAP_BIT_STATUS SSP_PCAP_get_audio_in_status(void);
SSP_PCAP_STATUS SSP_PCAP_write_data_to_PCAP(SSP_PCAP_SECONDARY_PROCESSOR_REGISTER ssp_pcap_register,u32 ssp_pcap_register_value);
SSP_PCAP_STATUS SSP_PCAP_read_data_from_PCAP(SSP_PCAP_SECONDARY_PROCESSOR_REGISTER ssp_pcap_register,u32* p_ssp_pcap_register_value);

SSP_PCAP_STATUS SSP_PCAP_bit_set(SSP_PCAP_SECONDARY_PROCESSOR_REGISTER_BIT_TYPE ssp_pcap_bit ) ;
SSP_PCAP_STATUS SSP_PCAP_bit_clean(SSP_PCAP_SECONDARY_PROCESSOR_REGISTER_BIT_TYPE ssp_pcap_bit ) ;
SSP_PCAP_BIT_STATUS SSP_PCAP_get_bit_from_buffer(SSP_PCAP_SECONDARY_PROCESSOR_REGISTER_BIT_TYPE ssp_pcap_bit ) ;
SSP_PCAP_BIT_STATUS SSP_PCAP_get_bit_from_PCAP(SSP_PCAP_SECONDARY_PROCESSOR_REGISTER_BIT_TYPE ssp_pcap_bit ) ;
u32 SSP_PCAP_get_register_value_from_buffer(SSP_PCAP_SECONDARY_PROCESSOR_REGISTER ssp_pcap_register ) ;

#ifdef SSP_PCAP_OPERATE_WITH_SPI
static wait_queue_head_t ssp_pcap_send_wait;
static u32 ssp_pcap_readValue;
static void ssp_interrupt_routine(int irq, void *dev_id, struct pt_regs *regs );
void ssp_put_intoData(u32 pcapValue);
#endif

static spinlock_t pcapoperation_lock = SPIN_LOCK_UNLOCKED;

void ssp_pcap_init(void);
void ssp_pcap_release(void);
void ssp_pcap_clear_int(void);
void ssp_pcap_intoSleepCallBack(void);
void ssp_pcap_wakeUpCallBack(void);

extern int usb_gpio_init(void);
extern int stop_usb(void);

extern void ezx_ts_dataReadok_interrupt(int irq,void* dev_id, struct pt_regs *regs);        
extern void ezx_ts_touch_interrupt(int irq,void* dev_id, struct pt_regs *regs);    
extern void headjack_change_interrupt_routine(int ch,void *dev_id,struct pt_regs *regs);
extern void mic_change_interrupt_routine(int ch,void *dev_id,struct pt_regs *regs);
extern u8 pxafb_ezx_getLCD_status(void);

extern int cable_hotplug_attach(ACCESSORY_DEVICE_STATUS status);

/*---------------------------------------------------------------------------
  DESCRIPTION: log system stored in buffer. it can output via FFUART. also you can change a little to other output mode.

   INPUTS: none.
      
       
   OUTPUTS: none.
      
       
   IMPORTANT NOTES: the macro PCAP_LOG_FFUART_OUT is a switch.
                    it is auto-output to FFUART every PCAP_LOG_OUTPUT_TIMER seconds
       
     
---------------------------------------------------------------------------*/
/* record log system by linwq */
//#define PCAP_LOG_FFUART_OUT

#ifdef  PCAP_LOG_FFUART_OUT
static struct timer_list pcap_outlog_timer;

#define PCAP_LOG_OUTPUT_TIMER   60
#define PCAP_LOG_RECORD_LENGTH  50000

unsigned char pcap_log_data[PCAP_LOG_RECORD_LENGTH];

u8* pcap_log_now_address = pcap_log_data;
#define PCAP_LOG_START_ADDRESS     pcap_log_data
#define PCAP_LOG_END_ADDRESS       pcap_log_data + PCAP_LOG_RECORD_LENGTH
u32 pcap_log_len = 0;

void pcap_log_add_pure_data(u8* pData,u32 len)
{
    u32 i;
    if ((pData ==NULL)|| len == 0)
        return;
    for(i = 0;i<len;i++)
    {
        *pcap_log_now_address ++ = pData[i];
        if( PCAP_LOG_END_ADDRESS == pcap_log_now_address )
            pcap_log_now_address = PCAP_LOG_START_ADDRESS; 
        pcap_log_len++;
        if(pcap_log_len >= PCAP_LOG_RECORD_LENGTH) 
            pcap_log_len = PCAP_LOG_RECORD_LENGTH;      
    }
    return;
}

void pcap_log_add_data(u8* pData,u32 len)
{
    u32 i;
    u8 temp[20];
    if ((pData ==NULL)|| len == 0)
        return;
    sprintf(temp,"\n <%d MS>",(int)(OSCR/3686));
    for(i = 0;i<(u32)strlen(temp);i++)
    {
        *pcap_log_now_address ++ = temp[i];
        if( PCAP_LOG_END_ADDRESS == pcap_log_now_address )
            pcap_log_now_address = PCAP_LOG_START_ADDRESS; 
        pcap_log_len++;
        if(pcap_log_len >= PCAP_LOG_RECORD_LENGTH) 
            pcap_log_len = PCAP_LOG_RECORD_LENGTH;      
    }

    for(i = 0;i<len;i++)
    {
        *pcap_log_now_address ++ = pData[i];
        if( PCAP_LOG_END_ADDRESS == pcap_log_now_address )
            pcap_log_now_address = PCAP_LOG_START_ADDRESS; 
        pcap_log_len++;
        if(pcap_log_len >= PCAP_LOG_RECORD_LENGTH) 
            pcap_log_len = PCAP_LOG_RECORD_LENGTH;      
    }
    return;
}

static void pcap_log_output_from_ffuart(void)
{
    u32 i; 
    u32 ssp_ffuart_dll,ssp_ffuart_dlh;
    u8 ssp_ffuart_cken = 1,ssp_ffuart_en = 1;
    if(0 == pcap_log_len)
        return;

    /* if cable in NOT UART cable, should return */
    printk("\n *********log out ************* \n <%d jiffies>log:",(int)jiffies);
    __cli();
    if(!(CKEN&CKEN6_FFUART))
    {
        ssp_ffuart_cken = 0;
        CKEN |= CKEN6_FFUART;
        pcap_log_add_data("FFUART CLK not ENABLE!",strlen("FFUART CLK not ENABLE!"));
    }
    
    FFLCR = (FFLCR&0xff)|0x80;
    ssp_ffuart_dll = FFDLL&0xff;
    ssp_ffuart_dlh = FFDLH&0xff;
    FFLCR = FFLCR&0x7f;

    if((0x08 !=ssp_ffuart_dll)||(0x00 != ssp_ffuart_dlh))        
    {
        FFLCR = 0x83;
        FFDLL = 0x08;
        FFDLH = 0;
        FFLCR = 0x03;
    }

    if(!(FFIER&0x00000040))
    {
        ssp_ffuart_en = 0;
        FFIER |= 0x00000040;
        pcap_log_add_data("FFUART model not ENABLE!",strlen("FFUART model not ENABLE!"));
    }

    for(i=0;i<pcap_log_len;i++)
    {
        while((FFLSR &0x40) == 0);
        FFTHR = pcap_log_data[i];
    }
    if(!ssp_ffuart_cken)
    {
        CKEN &= ~CKEN6_FFUART;
    }


    if((0x08 !=ssp_ffuart_dll)||(0x00 != ssp_ffuart_dlh))        
    {
        FFLCR = 0x83;
        FFDLL = ssp_ffuart_dll;
        FFDLH = ssp_ffuart_dlh;
        FFLCR = 0x03;
    }
    
    if(!ssp_ffuart_en)
    {
        CKEN &= ~CKEN6_FFUART;
    }
    __sti();

    printk("\n ********* log end ************ \n"); 
} 

static void pcap_log_output_from_stuart(void)
{
    u32 i; 
    u32 ssp_stuart_dll,ssp_stuart_dlh;
    u8 ssp_stuart_cken = 1,ssp_stuart_en = 1;
    if(0 == pcap_log_len)
        return;

    /* if cable in NOT UART cable, should return */
    printk("\n *********log out ************* \n <%d jiffies>log:",(int)jiffies);
    __cli();
    if(!(CKEN&CKEN5_STUART))
    {
        ssp_stuart_cken = 0;
        CKEN |= CKEN5_STUART;
        pcap_log_add_data("STUART CLK not ENABLE!",strlen("STUART CLK not ENABLE!"));
    }
    
    STLCR = (STLCR&0xff)|0x80;
    ssp_stuart_dll = STDLL&0xff;
    ssp_stuart_dlh = STDLH&0xff;
    STLCR &= 0x7f;

    if((0x08 !=ssp_stuart_dll)||(0x00 != ssp_stuart_dlh))        
    {
        STLCR = 0x83;
        STDLL = 0x08;
        STDLH = 0;
        STLCR = 0x03;
    }

    if(!(STIER&0x00000040))
    {
        ssp_stuart_en = 0;
        STIER |= 0x00000040;
        pcap_log_add_data("STUART model not ENABLE!",strlen("STUART model not ENABLE!"));
    }ACCESSORY_DEVICE_STATUS

    for(i=0;i<pcap_log_len;i++)
    {
        while((STLSR &0x40) == 0);
        STTHR = pcap_log_data[i];
    }
    if(!ssp_stuart_cken)
    {
        CKEN &= ~CKEN5_STUART;
    }


    if((0x08 !=ssp_stuart_dll)||(0x00 != ssp_stuart_dlh))        
    {
        STLCR = 0x83;
        STDLL = ssp_stuart_dll;
        STDLH = ssp_stuart_dlh;
        STLCR = 0x03;
    }
    
    if(!ssp_stuart_en)
    {
        CKEN &= ~CKEN5_STUART;
    }
    __sti();

    printk("\n ********* log end ************ \n"); 
} 


static void pcap_output_log_handler( u32 data)
{ 
     pcap_log_output_from_stuart();

     pcap_outlog_timer.function = pcap_output_log_handler;
     pcap_outlog_timer.expires = (jiffies + PCAP_LOG_OUTPUT_TIMER * HZ);
     add_timer(&pcap_outlog_timer);
}

static void pcap_output_log_init(void)
{
     init_timer(&pcap_outlog_timer);	    	
     pcap_outlog_timer.function = pcap_output_log_handler;
     pcap_outlog_timer.expires = (jiffies + 2*PCAP_LOG_OUTPUT_TIMER * HZ);
     add_timer(&pcap_outlog_timer);
}
/* record log end  */
#else
void pcap_log_add_pure_data(u8* pData,u32 len)
{
    return;
}

void pcap_log_add_data(u8* pData,u32 len)
{
    return;
}
#endif

/*---------------------------------------------------------------------------
  DESCRIPTION: GPIO init for operating PCAP via GPIO mode.

   INPUTS: none.
      
       
   OUTPUTS: none.
      
       
   IMPORTANT NOTES: auto-recognized pcap's CE signal. init PCAP's register.
       
     
---------------------------------------------------------------------------*/
#ifndef SSP_PCAP_OPERATE_WITH_SPI 

static u32 test_pcap_cs_rxd(void)
{
    u32 ret = FALSE;
    u32 temp,myIMR;
    /* write a data 0x0003ffff to IMR */
    SSP_PCAP_read_data_from_PCAP(SSP_PCAP_ADJ_MSR_REGISTER, &myIMR);
    
    SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_MSR_REGISTER, 0x0003ffff);
    SSP_PCAP_read_data_from_PCAP(SSP_PCAP_ADJ_MSR_REGISTER, &temp);
    if( 0x0003ffff == temp)
    {
        SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_MSR_REGISTER, myIMR);        
        ret = TRUE;
    } 
    return ret;
}

#if defined(CONFIG_ARCH_EZXBASE)
void ssp_pcap_gpioInit(void)
#else
static void ssp_pcap_gpioInit(void)
#endif
{
    u32 i;
    u32 tempValue;
    u32 ret;
    /* stop SPI port work mode  disable SPI clock */ 
    SSCR0  =  0x00000000; 
    CKEN  &= ~CKEN23_SSP1;

    /* the four PIN as GPIO mode */
    set_GPIO_mode(GPIO_SPI_CLK|GPIO_OUT);
    set_GPIO_mode(GPIO_SPI_CE|GPIO_OUT);
    set_GPIO_mode(GPIO_SPI_MOSI|GPIO_OUT);
    set_GPIO_mode(GPIO_SPI_MISO|GPIO_IN);

    /* test pcap's CE with inverter??? */ 
    ssp_pcap_operate_pin_with_inverter = 0;
    ssp_pcap_rxd_pin_with_inverter = 0;

    /* deassert SEC_CE  */
    GPCR(GPIO_SPI_CE) = GPIO_bit(GPIO_SPI_CE);
    for(i=0;i<500;i++);
    ret = test_pcap_cs_rxd();
    
    if (FALSE == ret)
    {
        ssp_pcap_operate_pin_with_inverter = 0;
        ssp_pcap_rxd_pin_with_inverter = 1;
        /* deassert SEC_CE  */
        GPCR(GPIO_SPI_CE) = GPIO_bit(GPIO_SPI_CE);
        for(i=0;i<500;i++);
        ret = test_pcap_cs_rxd();       
    }

    if (FALSE == ret)
    {
        ssp_pcap_operate_pin_with_inverter = 1;
        ssp_pcap_rxd_pin_with_inverter = 0;
        /* deassert SEC_CE  */
        GPSR(GPIO_SPI_CE) = GPIO_bit(GPIO_SPI_CE);
        for(i=0;i<500;i++);
        ret = test_pcap_cs_rxd();       
    }


    if (FALSE == ret)
    {
        ssp_pcap_operate_pin_with_inverter = 1;
        ssp_pcap_rxd_pin_with_inverter = 1;
        /* deassert SEC_CE  */
        GPSR(GPIO_SPI_CE) = GPIO_bit(GPIO_SPI_CE);
        for(i=0;i<500;i++);
        ret = test_pcap_cs_rxd();       
    }

    if (FALSE == ret )
    {
        printk("\n Bulverde can not communicate with PCAP!!!!");
        return;
    }

    SSP_PCAP_write_data_to_PCAP(1, 0x0013ffff);

    for(i=0; i<32 ;i++)
    {
        if(SSP_PCAP_SUCCESS == SSP_PCAP_read_data_from_PCAP(i,&tempValue))
        {
            ssp_pcap_registerValue[i] = tempValue;  
        }
        else
        {
            ssp_pcap_registerValue[i] = 0; 
        }
    }
    return;    
}


static void ssp_pcap_gpioClkControl(u32 bitValue)
{
    if(bitValue)
    {
        GPSR(GPIO_SPI_CLK) = GPIO_bit(GPIO_SPI_CLK);
    }
    else
    {
        GPCR(GPIO_SPI_CLK) = GPIO_bit(GPIO_SPI_CLK);
    }
}

static void ssp_pcap_gpioFrmControl(u32 bitValue)
{
    if( ssp_pcap_operate_pin_with_inverter )
    {
        if(bitValue)
        {
            GPCR(GPIO_SPI_CE) = GPIO_bit(GPIO_SPI_CE);
        }
        else
        {
            GPSR(GPIO_SPI_CE) = GPIO_bit(GPIO_SPI_CE);
        }
    }
    else
    {
        if(bitValue)
        {
            GPSR(GPIO_SPI_CE) = GPIO_bit(GPIO_SPI_CE);
        }
        else
        {
            GPCR(GPIO_SPI_CE) = GPIO_bit(GPIO_SPI_CE);
        }
    }
}

static void ssp_pcap_gpioTxdControl(u32 bitValue)
{
    if(bitValue)
    {
        GPSR(GPIO_SPI_MOSI) = GPIO_bit(GPIO_SPI_MOSI);
    }
    else
    {
        GPCR(GPIO_SPI_MOSI) = GPIO_bit(GPIO_SPI_MOSI);
    }
}

static u32 ssp_pcap_gpioRxdGetStatus(void)
{
    if(ssp_pcap_rxd_pin_with_inverter)
    {
        if( GPLR(GPIO_SPI_MISO)&GPIO_bit(GPIO_SPI_MISO) )
        {
            return SSP_PCAP_BIT_ZERO;
        }
        else
        {
            return SSP_PCAP_BIT_ONE;
        }
    }
    else
    {
        if( GPLR(GPIO_SPI_MISO)&GPIO_bit(GPIO_SPI_MISO) )
        {
            return SSP_PCAP_BIT_ONE;
        }
        else
        {
            return SSP_PCAP_BIT_ZERO;
        }
    }
}

static void ssp_pcap_gpioWrite(u32 pcapValue)
{
    u32 tempValue;
    u32 loopValue = 0x80000000;
    int i;
    /*  prepare for starting the frame  */
    ssp_pcap_gpioClkControl(SSP_PCAP_BIT_ZERO);
    ssp_pcap_gpioFrmControl(SSP_PCAP_BIT_ZERO);
    ssp_pcap_gpioTxdControl(SSP_PCAP_BIT_ZERO);

    /*   start the data transfering frame  */
    ssp_pcap_gpioFrmControl(SSP_PCAP_BIT_ONE);
    for(i=0;i<32;i++)
    {
        tempValue = pcapValue&loopValue;
        /* setup data bit to TXD line    */
        ssp_pcap_gpioTxdControl(tempValue);
        /*  create clock   */
        ssp_pcap_gpioClkControl(SSP_PCAP_BIT_ONE);
        loopValue = loopValue/2;
        ssp_pcap_gpioClkControl(SSP_PCAP_BIT_ZERO);
    }
    /*   end the frame */
    ssp_pcap_gpioFrmControl(SSP_PCAP_BIT_ZERO);
#ifdef SSP_PCAP_OPERATE_DEBUG_INFORNMATION
    printk( " \n Write END  \n " );
#endif
    return;
}


/*---------------------------------------------------------------------------
  DESCRIPTION:

   INPUTS:
   OUTPUTS:


   IMPORTANT NOTES:


---------------------------------------------------------------------------*/

static void ssp_pcap_gpioRead( u32 registerNumber,u32* pPcapValue)
{
    int i;
    u32 tempValue;
    u32 loopValue = 0x80000000;
    
    /*  prepare for starting the frame  */
    ssp_pcap_gpioClkControl(SSP_PCAP_BIT_ZERO);
    ssp_pcap_gpioFrmControl(SSP_PCAP_BIT_ZERO);
    ssp_pcap_gpioTxdControl(SSP_PCAP_BIT_ZERO);
    
    /* start the data transfering frame   */
    ssp_pcap_gpioFrmControl(SSP_PCAP_BIT_ONE);
    /* indictaor it's a read data command */
    ssp_pcap_gpioTxdControl(SSP_PCAP_BIT_ZERO);
    ssp_pcap_gpioClkControl(SSP_PCAP_BIT_ONE);
    loopValue = loopValue/2;
    ssp_pcap_gpioClkControl(SSP_PCAP_BIT_ZERO);

    for(i=0; i<5;i++)
    {
        /*  here maybe can be optimized */
        tempValue = registerNumber&(loopValue/0x04000000);
        ssp_pcap_gpioTxdControl(tempValue);
        ssp_pcap_gpioClkControl(SSP_PCAP_BIT_ONE);
        loopValue = loopValue/2;
        ssp_pcap_gpioClkControl(SSP_PCAP_BIT_ZERO);
    }
    
    /*  the dead bit  */
    ssp_pcap_gpioClkControl(SSP_PCAP_BIT_ONE);
    loopValue = loopValue/2;
    ssp_pcap_gpioClkControl(SSP_PCAP_BIT_ZERO);

    tempValue = 0;
    for(i=0;i<25;i++)
    {
        tempValue |= loopValue*ssp_pcap_gpioRxdGetStatus();
        ssp_pcap_gpioClkControl(SSP_PCAP_BIT_ONE);
        loopValue = loopValue/2;
        ssp_pcap_gpioClkControl(SSP_PCAP_BIT_ZERO);
    }
    /*   end the data frame   */
    ssp_pcap_gpioFrmControl(SSP_PCAP_BIT_ZERO);
    if(pPcapValue)
    {
        *pPcapValue = tempValue;
    }

#ifdef SSP_PCAP_OPERATE_DEBUG_INFORNMATION
    printk("  \n  Read Data end \n");
#endif
    return ;
}

#endif
/*---------------------------------------------------------------------------
  DESCRIPTION:

   INPUTS:

       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
  DESCRIPTION:
      
   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/
#if 0
static int pcap_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
{
    switch(req)
    {
        case PM_SUSPEND:
             ssp_pcap_intoSleepCallBack();
             break;
        case PM_RESUME:
             ssp_pcap_wakeUpCallBack();
             break;
    }
    return 0; 
}

#endif

#if defined(CONFIG_ARCH_EZXBASE)
int ssp_pcap_init_is_ok = 0;
#else
static int ssp_pcap_init_is_ok = 0;
#endif
void ssp_pcap_init(void)
{
    if(!ssp_pcap_init_is_ok)
    {
        ssp_pcap_gpioInit();
	ssp_pcap_init_is_ok = 1;
    }
#ifdef CONFIG_MOT_POWER_IC

#else
    set_GPIO_mode(GPIO_PCAP_SEC_INT | GPIO_IN);
    set_GPIO_IRQ_edge(GPIO_PCAP_SEC_INT,GPIO_RISING_EDGE);

    request_irq(IRQ_GPIO1,ssp_pcap_interrupt_routine,0,"PCAP request irq ",NULL);
    init_timer(&ssp_tsi_timer);	
    ssp_tsi_timer.function = ssp_tsi_keeper;

    init_timer(&ssp_usb_accessory_debounce_timer);
    ssp_usb_accessory_debounce_timer.function = ssp_usb_accessory_debounce_handler;

    ssp_pcap_screenlock_status = 0 ;
    init_timer(&ssp_pcap_screenlock_timer);
    ssp_pcap_screenlock_timer.function = ssp_pcap_screenlock_lockhandler;

#ifdef CONFIG_SSP_PCAP_SUPPORT_PRIMARY_SPI
    /* add code to support secondary SPI control SIM voltage */
    SSP_PCAP_bit_clean( SSP_PCAP_PRI_BIT_INT_SEL_ONOFFS);
    SSP_PCAP_bit_clean( SSP_PCAP_PRI_BIT_INT_SEL_MOBPORTS);
    SSP_PCAP_bit_clean( SSP_PCAP_PRI_BIT_INT_SEL_MB2S);
    SSP_PCAP_bit_clean( SSP_PCAP_PRI_BIT_INT_SEL_A1S);
    SSP_PCAP_bit_clean( SSP_PCAP_PRI_BIT_INT_SEL_STS);
    SSP_PCAP_bit_clean( SSP_PCAP_PRI_BIT_INT_SEL_ONOFFS);

    SSP_PCAP_bit_set( SSP_PCAP_PRI_BIT_AUX_VREG_MASK_VSIM_SEL );
    SSP_PCAP_bit_set( SSP_PCAP_PRI_BIT_AUX_VREG_VSIM_0 );
    SSP_PCAP_bit_clean( SSP_PCAP_PRI_BIT_AUX_VREG_VSIM_EN );

    SSP_PCAP_bit_clean( SSP_PCAP_PRI_BIT_MSR_ONOFFM);
    
    SSP_PCAP_bit_clean( SSP_PCAP_PRI_BIT_MSR_STM);
    SSP_PCAP_bit_clean( SSP_PCAP_PRI_BIT_MSR_A1M);

    u32 i;

    SSP_PCAP_read_data_from_PCAP( SSP_PCAP_PRI_AUX_VREG_REGISTER,&i);
     
    printk("\n AUX_VREG =0x%x!!! ",i);
    
#endif

    /* since in GPIO init, we have gotten the register's value.  */  
    SSP_PCAP_bit_set( SSP_PCAP_ADJ_BIT_ISR_USB4VI);
    SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_MSR_USB4VM);
    SSP_PCAP_bit_set( SSP_PCAP_ADJ_BIT_ISR_USB1VI);
    SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_MSR_USB1VM);
    SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_AUD_RX_AMPS_A1CTRL);
    SSP_PCAP_V_VIB_level_set(PCAP_VIBRATOR_VOLTAGE_LEVEL3);
    SSP_PCAP_V_VIB_level_set(PCAP_VIBRATOR_VOLTAGE_LEVEL3);

    /* set SW1 sleep to keep SW1 1.3v in sync mode */
    SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_LOWPWR_CTRL_SW1_MODE10  );
    SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_LOWPWR_CTRL_SW1_MODE11  );
    /*  SW1 active in sync mode */
    SSP_PCAP_bit_set( SSP_PCAP_ADJ_BIT_LOWPWR_CTRL_SW1_MODE00 );
    SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_LOWPWR_CTRL_SW1_MODE01);
    /*  at SW1 -core voltage to 1.30V   */
    SSP_PCAP_bit_set( SSP_PCAP_ADJ_BIT_LOWPWR_CTRL_SW10_DVS);
    SSP_PCAP_bit_set( SSP_PCAP_ADJ_BIT_LOWPWR_CTRL_SW11_DVS);
    SSP_PCAP_bit_set( SSP_PCAP_ADJ_BIT_LOWPWR_CTRL_SW12_DVS);
    SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_LOWPWR_CTRL_SW13_DVS);

    /* when STANDY2 PIN ACTIVE (high) set V3-- sram V8 -- pll off  */     
    SSP_PCAP_bit_set( SSP_PCAP_ADJ_BIT_VREG2_V3_STBY);
    SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_VREG2_V3_LOWPWR);

    SSP_PCAP_bit_set( SSP_PCAP_ADJ_BIT_VREG2_V8_STBY);
    SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_VREG2_V8_LOWPWR);

    /* when STANDY2 PIN ACTIVE (high) set V4-- lcd only for e680 V6 --- camera for e680 */     
    SSP_PCAP_bit_set( SSP_PCAP_ADJ_BIT_VREG2_V4_STBY);
    SSP_PCAP_bit_set( SSP_PCAP_ADJ_BIT_VREG2_V4_LOWPWR);

    SSP_PCAP_bit_set( SSP_PCAP_ADJ_BIT_VREG2_V6_STBY);
    SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_VREG2_V6_LOWPWR);
    
    /* set VAUX2 to voltage 2.775V and low power mode when AP sleep */
    SSP_PCAP_bit_set( SSP_PCAP_ADJ_BIT_AUX_VREG_VAUX2_1);
    SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_AUX_VREG_VAUX2_0);
    SSP_PCAP_bit_set( SSP_PCAP_ADJ_BIT_LOWPWR_CTRL_VAUX2_STBY);
    SSP_PCAP_bit_set( SSP_PCAP_ADJ_BIT_LOWPWR_CTRL_VAUX2_LOWPWR);
    SSP_PCAP_bit_set( SSP_PCAP_ADJ_BIT_AUX_VREG_VAUX2_EN );
    
#endif
    return;
}

/*---------------------------------------------------------------------------
  DESCRIPTION:
      
   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/
void pcap_switch_off_usb(void)
{
    if(!ssp_pcap_init_is_ok)
    {
        ssp_pcap_gpioInit();
	ssp_pcap_init_is_ok = 1;
    }
    /*  only for PST tool recognize the USB cable  */
    if(SSP_PCAP_BIT_ONE == SSP_PCAP_get_bit_from_PCAP(SSP_PCAP_ADJ_BIT_PSTAT_USBDET_4V ))
    {
        SSP_PCAP_get_bit_from_PCAP(SSP_PCAP_ADJ_BIT_BUSCTRL_USB_PU);
        SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_BUSCTRL_USB_PU);  
    }
    return;
}

void pcap_switch_on_usb(void)
{
    SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_BUSCTRL_USB_PU); 
}

/*---------------------------------------------------------------------------
  DESCRIPTION:
      
   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/
void ssp_pcap_release(void)
{
/* the PM callback will move to apm.c */
//    pm_unregister(pcap_pm_dev);
}

/*---------------------------------------------------------------------------
  DESCRIPTION:
      
   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/
void ssp_pcap_open(SSP_PCAP_INIT_DRIVER_TYPE portType)
{
#ifdef CONFIG_MOT_POWER_IC

#else
    switch(portType)
    {
        case SSP_PCAP_TS_OPEN:
#ifdef SSP_PCAP_OPERATE_DEBUG_INFORNMATION
            printk(" \n SSP TS open!!! ");
#endif
            SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_ADC1_TS_REFENB );
            SSP_PCAP_bit_set( SSP_PCAP_ADJ_BIT_ISR_ADCDONE2I);
            SSP_PCAP_bit_set( SSP_PCAP_ADJ_BIT_ISR_TSI);
            SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_MSR_TSM);
            SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_MSR_ADCDONE2M);
            SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_ADC2_ADINC1);
            SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_ADC2_ADINC2);
            SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_ADC1_ATO0);
            SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_ADC1_ATO1);
            SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_ADC1_ATO2);
            SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_ADC1_ATO3);
            SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_ADC1_ATOX);
            SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_ADC1_MTR1);
            SSP_PCAP_bit_clean( SSP_PCAP_ADJ_BIT_ADC1_MTR2);
            SSP_PCAP_TSI_mode_set(PCAP_TS_STANDBY_MODE);
            /* send the usb accessory infomation to PM */
            if((ACCESSORY_TYPE) sspUsbAccessoryInfo.type == ACCESSORY_DEVICE_USB_PORT)
            {
                if( (ACCESSORY_DEVICE_STATUS )sspUsbAccessoryInfo.status == ACCESSORY_DEVICE_STATUS_ATTACHED )
                {
                    apm_event_notify(APM_EVENT_DEVICE, EVENT_DEV_ACCS, DEV_ATTACH);
                }
                else
                {
                    apm_event_notify(APM_EVENT_DEVICE, EVENT_DEV_ACCS, DEV_DETACH);
                }
            }            
            break;
        case SSP_PCAP_AUDIO_OPEN:
            break;
        default:
            break;
    }
#ifdef SSP_PCAP_OPERATE_DEBUG_INFORNMATION
    for(i=0;i<32;i++)
    {
        SSP_PCAP_read_data_from_PCAP(i, &tempValue);
        printk("\n PCAP register %d = 0x%x!",i,tempValue);
    }
#endif
#endif
    return;
}

/*---------------------------------------------------------------------------
  DESCRIPTION:
      
   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/
void ssp_pcap_close(void)
{
    SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_MSR_REGISTER,PCAP_MASK_ALL_INTERRUPT);
    return;
}

/*---------------------------------------------------------------------------
  DESCRIPTION:
      
   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/
void ssp_pcap_intoSleepCallBack(void)
{
    /* set TS_REF to low power mode */
#ifdef CONFIG_MOT_POWER_IC

#else
    SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_ADC1_TS_REF_LOWPWR);
    SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_MSR_MB2M);
#endif
    /* set SPI_CLK as input PIN */;
    GPDR(GPIO_SPI_CLK)  &= ~GPIO_bit(GPIO_SPI_CLK);
    /* set SPI_MOSI as input PIN */
    GPDR(GPIO_SPI_MOSI) &= ~GPIO_bit(GPIO_SPI_MOSI);
    /* set PCAP's CE pin to dessert signal  SPI_CE must be in range of 0~31 PIN */
    if(ssp_pcap_operate_pin_with_inverter)
    {
        PGSR0  |= GPIO_bit(GPIO_SPI_CE);
    }
    else
    {
        PGSR0  &= ~GPIO_bit(GPIO_SPI_CE);
    }
}
/*---------------------------------------------------------------------------
  DESCRIPTION:
      
   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/
void ssp_pcap_wakeUpCallBack(void)
{
    /* set SPI_CLK to output PIN */;
    GPDR(GPIO_SPI_CLK)  |= GPIO_bit(GPIO_SPI_CLK) ;
    /* set SPI_MOSI to output PIN */
    GPDR(GPIO_SPI_MOSI) |= GPIO_bit(GPIO_SPI_MOSI);
#ifdef CONFIG_MOT_POWER_IC

#else
    SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_ADC1_TS_REF_LOWPWR);
    SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_MSR_MB2M);
#endif
}   

/*---------------------------------------------------------------------------
  DESCRIPTION:
      
   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/
SSP_PCAP_STATUS SSP_PCAP_configure_USB_UART_transeiver(SSP_PCAP_PORT_TYPE portType)
{
    SSP_PCAP_STATUS ret = SSP_PCAP_SUCCESS;
    switch(portType)
    {
        case SSP_PCAP_SERIAL_PORT:
            /* according to HW requirement, do not clear VUSB_EN  2004-05-11 */
            //ret = SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_BUSCTRL_VUSB_EN);
            if(SSP_PCAP_SUCCESS == ret)
            {
                ret = SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_BUSCTRL_RS232ENB);
            }
            break;
        case SSP_PCAP_LOW_USB_PORT:
            ret = SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_BUSCTRL_RS232ENB);
            if(SSP_PCAP_SUCCESS == ret)
            {
                ret = SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_BUSCTRL_FSENB);
            }
            if(SSP_PCAP_SUCCESS == ret)
            {
                ret = SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_BUSCTRL_VUSB_EN);
            }
            break;
        case SSP_PCAP_HIGH_USB_PORT:
            ret = SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_BUSCTRL_RS232ENB);
            if(SSP_PCAP_SUCCESS == ret)
            {
                ret = SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_BUSCTRL_FSENB);
            }
            if(SSP_PCAP_SUCCESS == ret)
            {
                ret = SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_BUSCTRL_VUSB_EN);
            }
            break;
        default:
            ret = SSP_PCAP_NOT_RUN ;
            break;
    }
    return ret;
}
/*---------------------------------------------------------------------------
  DESCRIPTION:
      
   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/

SSP_PCAP_STATUS SSP_PCAP_TSI_mode_set(TOUCH_SCREEN_DETECT_TYPE mode_Type )
{
    u32 tempValue;
    tempValue = ssp_pcap_registerValue[SSP_PCAP_ADJ_ADC1_REGISTER]&(~SSP_PCAP_TOUCH_PANEL_POSITION_DETECT_MODE_MASK);
    tempValue = tempValue|mode_Type;
    return SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_ADC1_REGISTER,tempValue);
}

/*---------------------------------------------------------------------------
  DESCRIPTION:
      
   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/
SSP_PCAP_STATUS SSP_PCAP_TSI_start_XY_read(void)
{
    SSP_PCAP_STATUS ret = SSP_PCAP_SUCCESS;
    u32 tempValue;
    tempValue = ssp_pcap_registerValue[SSP_PCAP_ADJ_ADC1_REGISTER]&SSP_PCAP_ADC_START_VALUE_SET_MASK;
    tempValue |= SSP_PCAP_ADC_START_VALUE;
    SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_ADC1_REGISTER,tempValue);
    SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_ADC2_ASC);
    return ret;    
}    

/*---------------------------------------------------------------------------
  DESCRIPTION:
      
   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES: y reads TSX1's ADC value in ADD2 register, x reads TSY1's ADC value in ADD1 register.  
          
---------------------------------------------------------------------------*/
SSP_PCAP_STATUS SSP_PCAP_TSI_get_XY_value(P_U16 p_x,P_U16 p_y)
{
    u32 tempValue;
    SSP_PCAP_STATUS ret = SSP_PCAP_read_data_from_PCAP(SSP_PCAP_ADJ_ADC2_REGISTER,&tempValue);
    if( SSP_PCAP_SUCCESS != ret)
        return ret;
    if(ssp_pcap_screenlock_status)
    {
        *p_x = 0;
        *p_y = 0;
        SSP_PCAP_TSI_mode_set(PCAP_TS_STANDBY_MODE); 
        mod_timer(&ssp_pcap_screenlock_timer,jiffies + SSP_PCAP_TS_KEEPER_TIMER);                   
        return  ret;
    }
    if(tempValue&0x00400000)
        return SSP_PCAP_ERROR_VALUE;
    if(p_x)
    { 
        *p_x = (U16)(tempValue&SSP_PCAP_ADD1_VALUE_MASK);
    }
    if(p_y)
    {
        *p_y = (U16)((tempValue&SSP_PCAP_ADD2_VALUE_MASK)>>SSP_PCAP_ADD2_VALUE_SHIFT);
    }
    SSP_PCAP_TSI_mode_set(PCAP_TS_STANDBY_MODE);            
    return ret;
}

/*---------------------------------------------------------------------------
  DESCRIPTION:
      
   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/

SSP_PCAP_STATUS SSP_PCAP_CDC_CLK_set(PHONE_CDC_CLOCK_TYPE clkType)
{
    u32 tempValue;
    tempValue = ssp_pcap_registerValue[SSP_PCAP_ADJ_AUD_CODEC_REGISTER]&(~SSP_PCAP_PHONE_CDC_CLOCK_MASK);
    tempValue = tempValue|clkType;
    return SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_AUD_CODEC_REGISTER,tempValue);
}

/*---------------------------------------------------------------------------
  DESCRIPTION:

   INPUTS:


   OUTPUTS:


   IMPORTANT NOTES:


---------------------------------------------------------------------------*/
SSP_PCAP_STATUS SSP_PCAP_V_VIB_level_set(VibratorVoltageLevel_TYPE VIBLevelType)
{
    u32 tempValue;
    tempValue = ssp_pcap_registerValue[SSP_PCAP_ADJ_AUX_VREG_REGISTER]&(~SSP_PCAP_VIBRATOR_VOLTAGE_LEVEL_MASK);
    tempValue = tempValue|VIBLevelType;
    return SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_AUX_VREG_REGISTER,tempValue);
}

/*---------------------------------------------------------------------------
  DESCRIPTION:

   INPUTS:


   OUTPUTS:


   IMPORTANT NOTES:


---------------------------------------------------------------------------*/
SSP_PCAP_STATUS SSP_PCAP_CDC_SR_set(ST_SAMPLE_RATE_TYPE srType)
{
    u32 tempValue;
    tempValue = ssp_pcap_registerValue[SSP_PCAP_ADJ_ST_DAC_REGISTER]&(~SSP_PCAP_STEREO_SAMPLE_RATE_MASK);
    tempValue = tempValue|srType;
    return SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_ST_DAC_REGISTER,tempValue);
}

/*---------------------------------------------------------------------------
  DESCRIPTION:

   INPUTS:


   OUTPUTS:


   IMPORTANT NOTES:


---------------------------------------------------------------------------*/
SSP_PCAP_STATUS SSP_PCAP_BCLK_set(ST_BCLK_TIME_SLOT_TYPE bclkType)
{
    u32 tempValue;
    tempValue = ssp_pcap_registerValue[SSP_PCAP_ADJ_ST_DAC_REGISTER]&(~SSP_PCAP_STEREO_BCLK_TIME_SLOT_MASK);
    tempValue = tempValue|bclkType;
    return SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_ST_DAC_REGISTER,tempValue);
}

/*---------------------------------------------------------------------------
  DESCRIPTION:

   INPUTS:


   OUTPUTS:


   IMPORTANT NOTES:


---------------------------------------------------------------------------*/
SSP_PCAP_STATUS SSP_PCAP_STCLK_set(ST_CLK_TYPE stClkType)
{
    u32 tempValue;
    tempValue = ssp_pcap_registerValue[SSP_PCAP_ADJ_ST_DAC_REGISTER]&(~SSP_PCAP_STEREO_CLOCK_MASK);
    tempValue = tempValue|stClkType;
    return SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_ST_DAC_REGISTER,tempValue);
}

/*---------------------------------------------------------------------------
  DESCRIPTION:

   INPUTS:


   OUTPUTS:


   IMPORTANT NOTES:


---------------------------------------------------------------------------*/
SSP_PCAP_STATUS SSP_PCAP_DIG_AUD_FS_set(DIG_AUD_MODE_TYPE fsType)
{
    u32 tempValue;
    tempValue = ssp_pcap_registerValue[SSP_PCAP_ADJ_ST_DAC_REGISTER]&(~SSP_PCAP_DIGITAL_AUDIO_MODE_MASK);
    tempValue = tempValue|fsType;
    return SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_ST_DAC_REGISTER,tempValue);
}

/*---------------------------------------------------------------------------
  DESCRIPTION:

   INPUTS:


   OUTPUTS:


   IMPORTANT NOTES:


---------------------------------------------------------------------------*/
SSP_PCAP_STATUS SSP_PCAP_AUDIG_set(u32 audioInGain)
{
    u32 tempValue;
    if (audioInGain > PCAP_AUDIO_IN_GAIN_MAX_VALUE)
        return SSP_PCAP_ERROR_VALUE;
    tempValue = ssp_pcap_registerValue[SSP_PCAP_ADJ_TX_AUD_AMPS_REGISTER]&(~SSP_PCAP_AUDIO_IN_GAIN_MASK);
    
    /* shoulb be better like this 	tempValue |= audioInGain <<SSP_PCAP_AUDIO_IN_GAIN_SHIFT; */
    tempValue = tempValue|audioInGain;
    return SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_TX_AUD_AMPS_REGISTER,tempValue);
}

/*---------------------------------------------------------------------------
  DESCRIPTION:

   INPUTS:


   OUTPUTS:


   IMPORTANT NOTES:


---------------------------------------------------------------------------*/
SSP_PCAP_STATUS SSP_PCAP_MONO_set(MONO_TYPE monoType)
{
    u32 tempValue;
    tempValue = ssp_pcap_registerValue[SSP_PCAP_ADJ_AUD_RX_AMPS_REGISTER]&(~SSP_PCAP_MONO_PGA_MASK);
    tempValue = tempValue|monoType;
    return SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_AUD_RX_AMPS_REGISTER,tempValue);
}

/*---------------------------------------------------------------------------
  DESCRIPTION:

   INPUTS:


   OUTPUTS:


   IMPORTANT NOTES:


---------------------------------------------------------------------------*/
SSP_PCAP_STATUS SSP_PCAP_AUDOG_set(u32 audioOutGain)
{
    u32 tempValue;
    if (audioOutGain > PCAP_AUDIO_OUT_GAIN_MAX_VALUE)
        return SSP_PCAP_ERROR_VALUE;
    tempValue = ssp_pcap_registerValue[SSP_PCAP_ADJ_AUD_RX_AMPS_REGISTER]&(~SSP_PCAP_AUDIO_OUT_GAIN_MASK);
    tempValue |= audioOutGain <<SSP_PCAP_AUDIO_OUT_GAIN_SHIFT;
    return SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_AUD_RX_AMPS_REGISTER,tempValue);
}

/*---------------------------------------------------------------------------
  DESCRIPTION:
      
   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/
SSP_PCAP_STATUS SSP_PCAP_write_data_to_PCAP(SSP_PCAP_SECONDARY_PROCESSOR_REGISTER sspPcapRegister,u32 registerValue)
{
    u32 pcapTempValue ;
    unsigned long flags;
    /* prevent the process schedule out and mask the PCAP's interrupt handler */
    //ICMR &= ~(1<<(GPIO1_RST+PXA_IRQ_SKIP));
    save_flags(flags);
    __cli();
    spin_lock(&pcapoperation_lock);

    if(!test_and_set_bit(0,&ssp_pcap_status)) 
    {
        switch(sspPcapRegister)
        {
#ifdef CONFIG_SSP_PCAP_SUPPORT_PRIMARY_SPI
            case SSP_PCAP_PRI_INT_SEL_REGISTER:
            case SSP_PCAP_PRI_SWCTRL_REGISTER:
            case SSP_PCAP_PRI_VREG1_REGISTER:
            case SSP_PCAP_PRI_RTC_TOD_REGISTER:
            case SSP_PCAP_PRI_RTC_TODA_REGISTER:
            case SSP_PCAP_PRI_RTC_DAY_REGISTER:
            case SSP_PCAP_PRI_RTC_DAYA_REGISTER:
            case SSP_PCAP_PRI_MTRTMR_REGISTER:
            case SSP_PCAP_PRI_PWR_CTRL_REGISTER:
            case SSP_PCAP_PRI_AUX_VREG_MASK_REGISTER:
            case SSP_PCAP_PRI_VENDOR_REV_REGISTER:
            case SSP_PCAP_PRI_PERIPH_MASK_REGISTER:
#endif
            case SSP_PCAP_ADJ_ISR_REGISTER:
            case SSP_PCAP_ADJ_MSR_REGISTER:
            case SSP_PCAP_ADJ_PSTAT_REGISTER:
            case SSP_PCAP_ADJ_VREG2_REGISTER:
            case SSP_PCAP_ADJ_AUX_VREG_REGISTER:
            case SSP_PCAP_ADJ_BATT_DAC_REGISTER:
            case SSP_PCAP_ADJ_ADC1_REGISTER:
            case SSP_PCAP_ADJ_ADC2_REGISTER:
            case SSP_PCAP_ADJ_AUD_CODEC_REGISTER:
            case SSP_PCAP_ADJ_AUD_RX_AMPS_REGISTER:
            case SSP_PCAP_ADJ_ST_DAC_REGISTER:
            case SSP_PCAP_ADJ_BUSCTRL_REGISTER:
            case SSP_PCAP_ADJ_PERIPH_REGISTER:
            case SSP_PCAP_ADJ_LOWPWR_CTRL_REGISTER:
            case SSP_PCAP_ADJ_TX_AUD_AMPS_REGISTER:
            case SSP_PCAP_ADJ_GP_REG_REGISTER:
                ssp_pcap_registerValue[sspPcapRegister] = registerValue&SSP_PCAP_REGISTER_VALUE_MASK;
                pcapTempValue = SSP_PCAP_REGISTER_WRITE_OP_BIT|(sspPcapRegister<<SSP_PCAP_REGISTER_ADDRESS_SHIFT )|(ssp_pcap_registerValue[sspPcapRegister]);

                ssp_pcap_gpioWrite(pcapTempValue);
                clear_bit(0,&ssp_pcap_status);

                /* Now it's OK */
                spin_unlock(&pcapoperation_lock);
                restore_flags(flags);
                //ICMR |= (1<<(GPIO1_RST+PXA_IRQ_SKIP));
                return SSP_PCAP_SUCCESS;
                /* maybe here should NOT be a break */
                break;
            default:
                clear_bit(0,&ssp_pcap_status);

                /* Now it's OK */
                spin_unlock(&pcapoperation_lock);
                restore_flags(flags);
                //ICMR |= (1<<(GPIO1_RST+PXA_IRQ_SKIP));
                return SSP_PCAP_ERROR_REGISTER;
                /* maybe here should NOT be a break */
                break;
        }
    }
    else
    {

         /* Now it's OK */
         spin_unlock(&pcapoperation_lock);
         restore_flags(flags);
         //ICMR |= (1<<(GPIO1_RST+PXA_IRQ_SKIP));
         return SSP_PCAP_NOT_RUN;
    }    
}

/*---------------------------------------------------------------------------
  DESCRIPTION:
      
   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/
SSP_PCAP_STATUS SSP_PCAP_read_data_from_PCAP(SSP_PCAP_SECONDARY_PROCESSOR_REGISTER sspPcapRegister,u32* pSspPcapRegisterValue)
{
    unsigned long flags;
    /* prevent the process schedule out and mask the PCAP's interrupt handler */
    //ICMR &= ~(1<<(GPIO1_RST+PXA_IRQ_SKIP));
    save_flags(flags);
    __cli();
    spin_lock(&pcapoperation_lock);
      
    if(!test_and_set_bit(0,&ssp_pcap_status)) 
    {
        switch(sspPcapRegister)
        {
#ifdef CONFIG_SSP_PCAP_SUPPORT_PRIMARY_SPI
            case SSP_PCAP_PRI_INT_SEL_REGISTER:
            case SSP_PCAP_PRI_SWCTRL_REGISTER:
            case SSP_PCAP_PRI_VREG1_REGISTER:
            case SSP_PCAP_PRI_RTC_TOD_REGISTER:
            case SSP_PCAP_PRI_RTC_TODA_REGISTER:
            case SSP_PCAP_PRI_RTC_DAY_REGISTER:
            case SSP_PCAP_PRI_RTC_DAYA_REGISTER:
            case SSP_PCAP_PRI_MTRTMR_REGISTER:
            case SSP_PCAP_PRI_PWR_CTRL_REGISTER:
            case SSP_PCAP_PRI_AUX_VREG_MASK_REGISTER:
            case SSP_PCAP_PRI_VENDOR_REV_REGISTER:
            case SSP_PCAP_PRI_PERIPH_MASK_REGISTER:

            case SSP_PCAP_PRI_TEST1_REGISTER:
            case SSP_PCAP_PRI_TEST2_REGISTER:
            case SSP_PCAP_PRI_VENDOR_TEST1_REGISTER:
            case SSP_PCAP_PRI_VENDOR_TEST2_REGISTER:
#endif
            case SSP_PCAP_ADJ_ISR_REGISTER:
            case SSP_PCAP_ADJ_MSR_REGISTER:
            case SSP_PCAP_ADJ_PSTAT_REGISTER:
            case SSP_PCAP_ADJ_VREG2_REGISTER:
            case SSP_PCAP_ADJ_AUX_VREG_REGISTER:
            case SSP_PCAP_ADJ_BATT_DAC_REGISTER:
            case SSP_PCAP_ADJ_ADC1_REGISTER:
            case SSP_PCAP_ADJ_ADC2_REGISTER:
            case SSP_PCAP_ADJ_AUD_CODEC_REGISTER:
            case SSP_PCAP_ADJ_AUD_RX_AMPS_REGISTER:
            case SSP_PCAP_ADJ_ST_DAC_REGISTER:
            case SSP_PCAP_ADJ_BUSCTRL_REGISTER:
            case SSP_PCAP_ADJ_PERIPH_REGISTER:
            case SSP_PCAP_ADJ_LOWPWR_CTRL_REGISTER:
            case SSP_PCAP_ADJ_TX_AUD_AMPS_REGISTER:
            case SSP_PCAP_ADJ_GP_REG_REGISTER:
                ssp_pcap_gpioRead(sspPcapRegister,pSspPcapRegisterValue);
                ssp_pcap_registerValue[sspPcapRegister] = *pSspPcapRegisterValue&SSP_PCAP_REGISTER_VALUE_MASK;
                clear_bit(0,&ssp_pcap_status);

                /* Now it's OK */
                spin_unlock(&pcapoperation_lock);
                restore_flags(flags);
                //ICMR |= (1<<(GPIO1_RST+PXA_IRQ_SKIP));
                return SSP_PCAP_SUCCESS;
                /* maybe here should NOT be a break */
                break;
            default:
                clear_bit(0,&ssp_pcap_status);
                
                /* Now it's OK */
                spin_unlock(&pcapoperation_lock);
                restore_flags(flags);
                //ICMR |= (1<<(GPIO1_RST+PXA_IRQ_SKIP));
                return SSP_PCAP_ERROR_REGISTER;
                /* maybe here should NOT be a break */
                break;
        }
    }
    else
    {
        /* Now it's OK */
        spin_unlock(&pcapoperation_lock);
        restore_flags(flags);
        //ICMR |= (1<<(GPIO1_RST+PXA_IRQ_SKIP));
        return SSP_PCAP_NOT_RUN;
    }    
}

/*---------------------------------------------------------------------------
  DESCRIPTION:

   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/
SSP_PCAP_STATUS SSP_PCAP_bit_set(SSP_PCAP_SECONDARY_PROCESSOR_REGISTER_BIT_TYPE sspPcapBit ) 
{
    u32 sspPcapRegisterBitValue;
    SSP_PCAP_STATUS ret;
    u8 sspPcapRegister = (sspPcapBit&SSP_PCAP_REGISTER_ADDRESS_MASK)>>SSP_PCAP_REGISTER_ADDRESS_SHIFT;
    
    switch(sspPcapRegister)
    {
	case SSP_PCAP_ADJ_ISR_REGISTER:
            ssp_pcap_registerValue[sspPcapRegister] = 0;

#ifdef CONFIG_SSP_PCAP_SUPPORT_PRIMARY_SPI
        case SSP_PCAP_PRI_INT_SEL_REGISTER:
        case SSP_PCAP_PRI_SWCTRL_REGISTER:
        case SSP_PCAP_PRI_VREG1_REGISTER:
        case SSP_PCAP_PRI_RTC_TOD_REGISTER:
        case SSP_PCAP_PRI_RTC_TODA_REGISTER:
        case SSP_PCAP_PRI_RTC_DAY_REGISTER:
        case SSP_PCAP_PRI_RTC_DAYA_REGISTER:
        case SSP_PCAP_PRI_MTRTMR_REGISTER:
        case SSP_PCAP_PRI_PWR_CTRL_REGISTER:
        case SSP_PCAP_PRI_AUX_VREG_MASK_REGISTER:
        case SSP_PCAP_PRI_VENDOR_REV_REGISTER:
        case SSP_PCAP_PRI_PERIPH_MASK_REGISTER:
#endif
        case SSP_PCAP_ADJ_MSR_REGISTER:
	case SSP_PCAP_ADJ_PSTAT_REGISTER:
	case SSP_PCAP_ADJ_VREG2_REGISTER:
	case SSP_PCAP_ADJ_AUX_VREG_REGISTER:
	case SSP_PCAP_ADJ_BATT_DAC_REGISTER:
	case SSP_PCAP_ADJ_ADC1_REGISTER:
	case SSP_PCAP_ADJ_ADC2_REGISTER:
	case SSP_PCAP_ADJ_AUD_CODEC_REGISTER:
	case SSP_PCAP_ADJ_AUD_RX_AMPS_REGISTER:
	case SSP_PCAP_ADJ_ST_DAC_REGISTER:
	case SSP_PCAP_ADJ_BUSCTRL_REGISTER:
	case SSP_PCAP_ADJ_PERIPH_REGISTER:
	case SSP_PCAP_ADJ_LOWPWR_CTRL_REGISTER:
	case SSP_PCAP_ADJ_TX_AUD_AMPS_REGISTER:
        case SSP_PCAP_ADJ_GP_REG_REGISTER:

            sspPcapRegisterBitValue = (sspPcapBit&SSP_PCAP_REGISTER_VALUE_MASK);
            ssp_pcap_registerValue[sspPcapRegister] |= sspPcapRegisterBitValue;
            /* should care the return value */
            ret =SSP_PCAP_write_data_to_PCAP(sspPcapRegister,ssp_pcap_registerValue[sspPcapRegister]);
            return SSP_PCAP_SUCCESS;
            break;
	default:
	    return SSP_PCAP_ERROR_REGISTER;
	    break;
    }
    return SSP_PCAP_SUCCESS;
}

/*---------------------------------------------------------------------------
  DESCRIPTION:

   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/
SSP_PCAP_STATUS SSP_PCAP_bit_clean(SSP_PCAP_SECONDARY_PROCESSOR_REGISTER_BIT_TYPE sspPcapBit )
{
    u32 sspPcapRegisterBitValue;
    SSP_PCAP_STATUS ret;
    u8 sspPcapRegister = (sspPcapBit&SSP_PCAP_REGISTER_ADDRESS_MASK)>>SSP_PCAP_REGISTER_ADDRESS_SHIFT;
    
    switch(sspPcapRegister)
    {
	case SSP_PCAP_ADJ_ISR_REGISTER:
            ssp_pcap_registerValue[sspPcapRegister] = 0;
#ifdef CONFIG_SSP_PCAP_SUPPORT_PRIMARY_SPI
        case SSP_PCAP_PRI_INT_SEL_REGISTER:
        case SSP_PCAP_PRI_SWCTRL_REGISTER:
        case SSP_PCAP_PRI_VREG1_REGISTER:
        case SSP_PCAP_PRI_RTC_TOD_REGISTER:
        case SSP_PCAP_PRI_RTC_TODA_REGISTER:
        case SSP_PCAP_PRI_RTC_DAY_REGISTER:
        case SSP_PCAP_PRI_RTC_DAYA_REGISTER:
        case SSP_PCAP_PRI_MTRTMR_REGISTER:
        case SSP_PCAP_PRI_PWR_CTRL_REGISTER:
        case SSP_PCAP_PRI_AUX_VREG_MASK_REGISTER:
        case SSP_PCAP_PRI_VENDOR_REV_REGISTER:
        case SSP_PCAP_PRI_PERIPH_MASK_REGISTER:
#endif
        case SSP_PCAP_ADJ_MSR_REGISTER:
	case SSP_PCAP_ADJ_PSTAT_REGISTER:
	case SSP_PCAP_ADJ_VREG2_REGISTER:
	case SSP_PCAP_ADJ_AUX_VREG_REGISTER:
	case SSP_PCAP_ADJ_BATT_DAC_REGISTER:
	case SSP_PCAP_ADJ_ADC1_REGISTER:
	case SSP_PCAP_ADJ_ADC2_REGISTER:
	case SSP_PCAP_ADJ_AUD_CODEC_REGISTER:
	case SSP_PCAP_ADJ_AUD_RX_AMPS_REGISTER:
	case SSP_PCAP_ADJ_ST_DAC_REGISTER:
	case SSP_PCAP_ADJ_BUSCTRL_REGISTER:
	case SSP_PCAP_ADJ_PERIPH_REGISTER:
	case SSP_PCAP_ADJ_LOWPWR_CTRL_REGISTER:
	case SSP_PCAP_ADJ_TX_AUD_AMPS_REGISTER:
        case SSP_PCAP_ADJ_GP_REG_REGISTER:
	    sspPcapRegisterBitValue = (sspPcapBit&SSP_PCAP_REGISTER_VALUE_MASK);
	    ssp_pcap_registerValue[sspPcapRegister] &= ~sspPcapRegisterBitValue;
	    /* should care the return value */
	    ret =SSP_PCAP_write_data_to_PCAP(sspPcapRegister,ssp_pcap_registerValue[sspPcapRegister]);
	    return SSP_PCAP_SUCCESS;
	    break;
	default:
	    return SSP_PCAP_ERROR_REGISTER;
	    break;
    }
    return SSP_PCAP_SUCCESS;
}

/*---------------------------------------------------------------------------
  DESCRIPTION:
      
   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:

     
---------------------------------------------------------------------------*/
SSP_PCAP_BIT_STATUS SSP_PCAP_get_bit_from_buffer(SSP_PCAP_SECONDARY_PROCESSOR_REGISTER_BIT_TYPE sspPcapBit )
{
    u32 sspPcapRegisterBitValue;
    u8 sspPcapRegister = (sspPcapBit&SSP_PCAP_REGISTER_ADDRESS_MASK)>>SSP_PCAP_REGISTER_ADDRESS_SHIFT;
    switch(sspPcapRegister)
    {
#ifdef CONFIG_SSP_PCAP_SUPPORT_PRIMARY_SPI
        case SSP_PCAP_PRI_INT_SEL_REGISTER:
        case SSP_PCAP_PRI_SWCTRL_REGISTER:
        case SSP_PCAP_PRI_VREG1_REGISTER:
        case SSP_PCAP_PRI_RTC_TOD_REGISTER:
        case SSP_PCAP_PRI_RTC_TODA_REGISTER:
        case SSP_PCAP_PRI_RTC_DAY_REGISTER:
        case SSP_PCAP_PRI_RTC_DAYA_REGISTER:
        case SSP_PCAP_PRI_MTRTMR_REGISTER:
        case SSP_PCAP_PRI_PWR_CTRL_REGISTER:
        case SSP_PCAP_PRI_AUX_VREG_MASK_REGISTER:
        case SSP_PCAP_PRI_VENDOR_REV_REGISTER:
        case SSP_PCAP_PRI_PERIPH_MASK_REGISTER:

        case SSP_PCAP_PRI_TEST1_REGISTER:
        case SSP_PCAP_PRI_TEST2_REGISTER:
        case SSP_PCAP_PRI_VENDOR_TEST1_REGISTER:
        case SSP_PCAP_PRI_VENDOR_TEST2_REGISTER:
#endif
        case SSP_PCAP_ADJ_ISR_REGISTER:
	case SSP_PCAP_ADJ_MSR_REGISTER:
	case SSP_PCAP_ADJ_PSTAT_REGISTER:
	case SSP_PCAP_ADJ_VREG2_REGISTER:
	case SSP_PCAP_ADJ_AUX_VREG_REGISTER:
	case SSP_PCAP_ADJ_BATT_DAC_REGISTER:
	case SSP_PCAP_ADJ_ADC1_REGISTER:
	case SSP_PCAP_ADJ_ADC2_REGISTER:
	case SSP_PCAP_ADJ_AUD_CODEC_REGISTER:
	case SSP_PCAP_ADJ_AUD_RX_AMPS_REGISTER:
	case SSP_PCAP_ADJ_ST_DAC_REGISTER:
	case SSP_PCAP_ADJ_BUSCTRL_REGISTER:
	case SSP_PCAP_ADJ_PERIPH_REGISTER:
	case SSP_PCAP_ADJ_LOWPWR_CTRL_REGISTER:
	case SSP_PCAP_ADJ_TX_AUD_AMPS_REGISTER:
        case SSP_PCAP_ADJ_GP_REG_REGISTER:
            sspPcapRegisterBitValue = (sspPcapBit&SSP_PCAP_REGISTER_VALUE_MASK);
            sspPcapRegisterBitValue &= ssp_pcap_registerValue[sspPcapRegister];
            if(sspPcapRegisterBitValue)
            {
                return SSP_PCAP_BIT_ONE;
            }
            else
            {
                return SSP_PCAP_BIT_ZERO;
            }    
            break;
	default:
            return SSP_PCAP_BIT_ERROR;
            break;
    }
    return SSP_PCAP_BIT_ERROR;
}

/*---------------------------------------------------------------------------
  DESCRIPTION:
      
   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/
SSP_PCAP_BIT_STATUS SSP_PCAP_get_bit_from_PCAP(SSP_PCAP_SECONDARY_PROCESSOR_REGISTER_BIT_TYPE sspPcapBit )
{
    u32 sspPcapTempRegisterValue;
    u32 sspPcapRegisterBitValue;
    SSP_PCAP_STATUS ret;
    u8 sspPcapRegister = (sspPcapBit&SSP_PCAP_REGISTER_ADDRESS_MASK)>>SSP_PCAP_REGISTER_ADDRESS_SHIFT;
    
    switch(sspPcapRegister)
    {
#ifdef CONFIG_SSP_PCAP_SUPPORT_PRIMARY_SPI
        case SSP_PCAP_PRI_INT_SEL_REGISTER:
        case SSP_PCAP_PRI_SWCTRL_REGISTER:
        case SSP_PCAP_PRI_VREG1_REGISTER:
        case SSP_PCAP_PRI_RTC_TOD_REGISTER:
        case SSP_PCAP_PRI_RTC_TODA_REGISTER:
        case SSP_PCAP_PRI_RTC_DAY_REGISTER:
        case SSP_PCAP_PRI_RTC_DAYA_REGISTER:
        case SSP_PCAP_PRI_MTRTMR_REGISTER:
        case SSP_PCAP_PRI_PWR_CTRL_REGISTER:
        case SSP_PCAP_PRI_AUX_VREG_MASK_REGISTER:
        case SSP_PCAP_PRI_VENDOR_REV_REGISTER:
        case SSP_PCAP_PRI_PERIPH_MASK_REGISTER:

        case SSP_PCAP_PRI_TEST1_REGISTER:
        case SSP_PCAP_PRI_TEST2_REGISTER:
        case SSP_PCAP_PRI_VENDOR_TEST1_REGISTER:
        case SSP_PCAP_PRI_VENDOR_TEST2_REGISTER:
#endif
	case SSP_PCAP_ADJ_ISR_REGISTER:
	case SSP_PCAP_ADJ_MSR_REGISTER:
	case SSP_PCAP_ADJ_PSTAT_REGISTER:
	case SSP_PCAP_ADJ_VREG2_REGISTER:
	case SSP_PCAP_ADJ_AUX_VREG_REGISTER:
	case SSP_PCAP_ADJ_BATT_DAC_REGISTER:
	case SSP_PCAP_ADJ_ADC1_REGISTER:
	case SSP_PCAP_ADJ_ADC2_REGISTER:
	case SSP_PCAP_ADJ_AUD_CODEC_REGISTER:
	case SSP_PCAP_ADJ_AUD_RX_AMPS_REGISTER:
	case SSP_PCAP_ADJ_ST_DAC_REGISTER:
	case SSP_PCAP_ADJ_BUSCTRL_REGISTER:
	case SSP_PCAP_ADJ_PERIPH_REGISTER:
	case SSP_PCAP_ADJ_LOWPWR_CTRL_REGISTER:
	case SSP_PCAP_ADJ_TX_AUD_AMPS_REGISTER:
        case SSP_PCAP_ADJ_GP_REG_REGISTER:
            sspPcapRegisterBitValue = (sspPcapBit&SSP_PCAP_REGISTER_VALUE_MASK);
		/* should care the return value */
            ret = SSP_PCAP_read_data_from_PCAP(sspPcapRegister,&sspPcapTempRegisterValue);
            sspPcapRegisterBitValue &= sspPcapTempRegisterValue;
            if(sspPcapRegisterBitValue)
            {
                return SSP_PCAP_BIT_ONE;
            }
            else
            {
                return SSP_PCAP_BIT_ZERO;
            }
            break;
	default:
            return SSP_PCAP_BIT_ERROR;
            break;
    }
    return SSP_PCAP_BIT_ERROR;
}
/*---------------------------------------------------------------------------
  DESCRIPTION:
      
   INPUTS:
      
       
   OUTPUTS:
      
       
   IMPORTANT NOTES:
       
     
---------------------------------------------------------------------------*/
u32 SSP_PCAP_get_register_value_from_buffer(SSP_PCAP_SECONDARY_PROCESSOR_REGISTER sspPcapRegister )
{
    switch(sspPcapRegister)
    {
#ifdef CONFIG_SSP_PCAP_SUPPORT_PRIMARY_SPI
        case SSP_PCAP_PRI_INT_SEL_REGISTER:
        case SSP_PCAP_PRI_SWCTRL_REGISTER:
        case SSP_PCAP_PRI_VREG1_REGISTER:
        case SSP_PCAP_PRI_RTC_TOD_REGISTER:
        case SSP_PCAP_PRI_RTC_TODA_REGISTER:
        case SSP_PCAP_PRI_RTC_DAY_REGISTER:
        case SSP_PCAP_PRI_RTC_DAYA_REGISTER:
        case SSP_PCAP_PRI_MTRTMR_REGISTER:
        case SSP_PCAP_PRI_PWR_CTRL_REGISTER:
        case SSP_PCAP_PRI_AUX_VREG_MASK_REGISTER:
        case SSP_PCAP_PRI_VENDOR_REV_REGISTER:
        case SSP_PCAP_PRI_PERIPH_MASK_REGISTER:

        case SSP_PCAP_PRI_TEST1_REGISTER:
        case SSP_PCAP_PRI_TEST2_REGISTER:
        case SSP_PCAP_PRI_VENDOR_TEST1_REGISTER:
        case SSP_PCAP_PRI_VENDOR_TEST2_REGISTER:
#endif
	case SSP_PCAP_ADJ_ISR_REGISTER:
	case SSP_PCAP_ADJ_MSR_REGISTER:
	case SSP_PCAP_ADJ_PSTAT_REGISTER:
	case SSP_PCAP_ADJ_VREG2_REGISTER:
	case SSP_PCAP_ADJ_AUX_VREG_REGISTER:
	case SSP_PCAP_ADJ_BATT_DAC_REGISTER:
	case SSP_PCAP_ADJ_ADC1_REGISTER:
	case SSP_PCAP_ADJ_ADC2_REGISTER:
	case SSP_PCAP_ADJ_AUD_CODEC_REGISTER:
	case SSP_PCAP_ADJ_AUD_RX_AMPS_REGISTER:
	case SSP_PCAP_ADJ_ST_DAC_REGISTER:
	case SSP_PCAP_ADJ_BUSCTRL_REGISTER:
	case SSP_PCAP_ADJ_PERIPH_REGISTER:
	case SSP_PCAP_ADJ_LOWPWR_CTRL_REGISTER:
	case SSP_PCAP_ADJ_TX_AUD_AMPS_REGISTER:
        case SSP_PCAP_ADJ_GP_REG_REGISTER:
            return ssp_pcap_registerValue[sspPcapRegister];
            break;
	default:
	    return SSP_PCAP_ERROR_REGISTER;
	    break;
    }
}

#ifdef __cplusplus
}
#endif
