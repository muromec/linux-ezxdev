/*
 * Copyright (C) 2005 Motorola Inc.
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
 *
 *
 *  History:
 *  Lv YunGuang , Motorola            Jun 31,2005     port from E680's nssp.c
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/sound.h>
#include <linux/soundcard.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>
#include <asm/dma.h>

#include "ezx-audio.h"
#include "ezx-common.h"


static DECLARE_MUTEX(ezx_BTVR_nssp_mutex);

static int ezx_BTVR_init(void);
static void ezx_BTVR_shutdown(void);


/*initialize hardware, nssp controller and pcap register*/
static int ezx_BTVR_init(void)
{
	unsigned long flags;
	volatile unsigned long audiostatus;
	unsigned long timeout;
	unsigned long ssp_pcap_register_val;

	down(&ezx_BTVR_nssp_mutex); 
	audioonflag |= BT_DEVICE;

        AUDPRINTk1("setup nssp controller register for BTVR \n");
	local_irq_save(flags);

	CKEN |= CKEN3_NSSP;	                                /* need enable cken3 */

	/* setup nssp port */
	NSSCR0 = NSSCR0_FRF_PSP | NSSCR0_DSS_16bit | NSSCR0_MOD_NET | NSSCR0_FRDC_4  ; //for tcmd
        /* NSSCR1 setting: 
         *      NSSCR0_FRF_PSP   --- data format PSP mode
         *      NSSCR0_DSS_16bit --- Data size 16bit  
         *      NSSCR0_MOD_NET --- network mode
         *      NSSCR0_FRDC_4 --- Frame Rate, 4 time slot per frame 
        */
	NSSCR1 = NSSCR1_TTE |NSSCR1_TTELP | NSSCR1_EBCEI | NSSCR1_SCLKDIR | NSSCR1_SFRMDIR | NSSCR1_TFTH_4 | NSSCR1_RFTH_14;
        /* NSSCR1 setting:
         *       NSSCR1_TTE     --- TXD Tristate enable
         *       NSSCR1_TTELP   --- TXD Tristate enable on last Phase
         *       NSSCR1_EBCEI   --- bit count error interrupt enable
         *       NSSCR1_SCLKDIR --- bit_clock slave mode
         *       NSSCR1_SFRMDIR --- Frame_clock slave mode
         *       NSSCR1_TFTH_4  --- Transmit FIFO threshold 
         *       NSSCR1_RFTH_14 --- Receive threshold 
        */
        NSSPSP = NSSPSP_SFRMWDTH_1 | NSSPSP_SFRMP_HIGH | NSSPSP_SCMODE | NSSPSP_STRTDLY | NSSPSP_DMYSTRT_1 ;
        /* NSSPSP setting:
         *       NSSPSP_SFRMWDTH_1 --- serial frame width is 1 SCLK 
	 *       NSSPSP_SFRMP_HIGH --- NSSPSFRM is active high
         *       NSSPSP_SCMODE     --- Serial Bit-Rate Clock Mode(0b10): 
         *                          Data driven-rising; Data Sample-Falling; Idle State-High
        */
      
        NSSTSA = NSSTSA_TTSA_1; /* TX time slot 0 active   */
        NSSRSA = NSSRSA_RTSA_1; /* RX time slot 0 active   */

	set_GPIO_mode(GPIO_BITCLK_IN_NSSP_MD);                  /* BitCLK */
        set_GPIO_mode(GPIO_SYNC_IN_NSSP_MD);                    /* FSYNC */
        set_GPIO_mode(GPIO_SDATA_OUT_NSSP_MD);                  /* TXD ssp2 ALF out 2 */
        set_GPIO_mode(GPIO_SDATA_IN_NSSP_MD);                   /* RXD ssp2 ALF in 1*/

	NSSCR1 |= NSSCR1_TSRE | NSSCR1_RSRE;	                /* enable dma request */
	NSSCR0 |= NSSCR0_SSE;	                                /* enable nssp controller */
        
	local_irq_restore(flags);

	/** set PCAP ***/
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, V2_EN_2_INDEX, PCAP_BIT_SET_VALUE, V2_EN_2_NUM_BITS );
	ssp_pcap_register_val = CDC_CLK_IN_13M0;
	power_ic_set_reg_value( PCAP_AUD_CODEC, PCAP_AUD_CODEC_INDEX, PCAP_BIT_CLEAN_VALUE, PCAP_AUD_CODEC_NUM_BITS );
        power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_CLK_INDEX, CDC_CLK_IN_13M0, CDC_CLK_NUM_BITS );

	pcap_use_ap_13m_clock();

	power_ic_set_reg_value( PCAP_AUD_CODEC, FS_8K_16K_INDEX, PCAP_BIT_CLEAN_VALUE, FS_8K_16K_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, DIG_AUD_IN_INDEX, PCAP_BIT_CLEAN_VALUE, DIG_AUD_IN_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, AUDIHPF_INDEX, PCAP_BIT_SET_VALUE, AUDIHPF_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, AUDOHPF_INDEX, PCAP_BIT_SET_VALUE, AUDOHPF_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, CLK_INV_INDEX, PCAP_BIT_CLEAN_VALUE, CLK_INV_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, FS_INV_INDEX, PCAP_BIT_CLEAN_VALUE, FS_INV_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, DF_RESET_INDEX, PCAP_BIT_SET_VALUE, DF_RESET_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, ADITH_INDEX, PCAP_BIT_CLEAN_VALUE, ADITH_NUM_BITS );

	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, CD_BYP_INDEX, PCAP_BIT_CLEAN_VALUE, CD_BYP_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_EN_INDEX, PCAP_BIT_CLEAN_VALUE, CDC_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_CLK_EN_INDEX, PCAP_BIT_SET_VALUE, CDC_CLK_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, SMB_INDEX, PCAP_BIT_CLEAN_VALUE, SMB_NUM_BITS );
	mdelay(1);

        AUDPRINTk1("NSSCR0 = 0x%lx\n", NSSCR0 ); 
        AUDPRINTk1("NSSCR1 = 0x%lx\n", NSSCR1 );
        AUDPRINTk1("NSSPSP = 0x%lx\n", NSSPSP );        
        print_pcap_audio_reg_vals(); /* when open debug, printk the PCAP registers values  */
	
	timeout = 0;
	/* check if nssp is ready for slave operation	*/
	while(((audiostatus = NSSSR) & NSSSR_CSS) !=0){
		if((timeout++) > 10000000)
			goto err;
	}
        AUDPRINTk1(" complete all hardware init for BTVR \n");
	up(&ezx_BTVR_nssp_mutex);

	return 0;

err:
	up(&ezx_BTVR_nssp_mutex);
	printk("audio panic: nssp for BTVR don't ready for slave operation!!! ");
	return -ENODEV;	

}


static void ezx_BTVR_shutdown(void)
{

	down(&ezx_BTVR_nssp_mutex); 
	
	audioonflag &= ~BT_DEVICE;
	
        AUDPRINTk1("close nssp port for BTVR \n");
	NSSCR0 = 0;
	NSSCR1 = 0;
	NSSPSP = 0;
	CKEN &= ~CKEN3_NSSP;    /* SSP2 need control the CKEN3  */ 

	set_GPIO_mode(GPIO_NSSP_SCLK2 | GPIO_IN);    /* BitCLK */
        set_GPIO_mode(GPIO_NSSP_SFRM2 | GPIO_IN);    /* FSYNC */
        set_GPIO_mode(GPIO_NSSP_TXD2  | GPIO_IN);    /* TXD ssp2 ALF in 1 */
        set_GPIO_mode(GPIO_NSSP_RXD2  | GPIO_IN);   /*  RXD ssp2 ALF in 1 */

	power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_CLK_EN_INDEX, PCAP_BIT_CLEAN_VALUE, CDC_CLK_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_EN_INDEX, PCAP_BIT_CLEAN_VALUE, CDC_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, SMB_INDEX, PCAP_BIT_SET_VALUE, SMB_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, CD_TS_INDEX, PCAP_BIT_SET_VALUE, CD_TS_NUM_BITS );	

	if( STEREODEVOPENED )
		pcap_use_ap_13m_clock();
	else
		pcap_use_bp_13m_clock();

        up(&ezx_BTVR_nssp_mutex);
}


static int ezx_BTVR_ioctl(struct inode *inode, struct file *file,
                      unsigned int cmd, unsigned long arg)
{
                                                                                                                             
        switch(cmd) {
                case EZX_BTVR_NSSP_TX_OFF:
                	down(&ezx_BTVR_nssp_mutex);
			AUDPRINTk1(EZXOSS_DEBUG "set BTVR in factory test mode, Deactive NSSP TX time slot 0 \n");
               		NSSCR0 &= ~ NSSCR0_SSE;
			NSSTSA = NSSTSA_TTSA_0; /* deactive TX time slot 0 */
                	NSSCR0 |=  NSSCR0_SSE;
			up(&ezx_BTVR_nssp_mutex);
			break;
                                                                                                                             
	        case EZX_BTVR_NSSP_TX_ON:
			down(&ezx_BTVR_nssp_mutex);
                        AUDPRINTk1(EZXOSS_DEBUG "set BTVR in factory test mode, Active NSSP TX time slot 0 \n");
                        NSSCR0 &= ~ NSSCR0_SSE;
                        NSSTSA = NSSTSA_TTSA_1; /* active TX time slot 0 */
                        NSSCR0 |=  NSSCR0_SSE;
                        up(&ezx_BTVR_nssp_mutex);
			break;
       		default:
                	return mixer_ioctl(inode, file, cmd, arg);
        }

        return 0;
}



/*
 * Audio stuff for BTVR
 */

static audio_stream_t ezx_BTVR_nssp_audio_out = {
	name:			"ezx_BTVR_nssp_audio_out",
	dcmd:			DCMD_TXNSSDR,
	drcmr:			&DRCMRTXNSSDR,  /* nssp dma map register */
	dev_addr:		__PREG(NSSDR),
};

static audio_stream_t ezx_BTVR_nssp_audio_in = {
	name:			"ezx_BTVR_nssp_audio_in",
	dcmd:			DCMD_RXNSSDR,
	drcmr:			&DRCMRRXNSSDR,  /* nssp dma map register */
	dev_addr:		__PREG(NSSDR),
};

static audio_state_t ezx_BTVR_nssp_audio_state = {
	output_stream:		&ezx_BTVR_nssp_audio_out,
	input_stream:		&ezx_BTVR_nssp_audio_in,
	client_ioctl:		ezx_BTVR_ioctl, 
	hw_init:		ezx_BTVR_init,
	hw_shutdown:		ezx_BTVR_shutdown,
	sem:			__MUTEX_INITIALIZER(ezx_BTVR_nssp_audio_state.sem),
};


static int ezx_BTVR_nssp_audio_open(struct inode *inode, struct file *file)
{
        AUDPRINTk1("ezx_BTVR_nssp_audio_open \n");
	if( audioonflag & (CALL_DEVICE|DSP16_DEVICE|BT_DEVICE) ){
		AUDPRINTk1(EZXOSS_DEBUG "open btvr driver EBUSY because 0x%X device is using the sound hardware.\n",audioonflag );

		return -EBUSY;
	}

	return cotulla_audio_attach(inode, file, &ezx_BTVR_nssp_audio_state);
}

/*
 * Missing fields of this structure will be patched with the call to cotulla_audio_attach().
 */
static struct file_operations ezx_BTVR_fops = {
	open:		ezx_BTVR_nssp_audio_open,
	owner:		THIS_MODULE
};


static int __init ezx_BTVR_nssp_init(void)
{
	ezx_BTVR_nssp_audio_state.dev_dsp = register_sound_audio(&ezx_BTVR_fops, -1);

	set_GPIO_mode(GPIO_NSSP_SCLK2 | GPIO_IN);		/* BitCLK */
	set_GPIO_mode(GPIO_NSSP_SFRM2 | GPIO_IN);		/* FSYNC */
	set_GPIO_mode(GPIO_NSSP_TXD2  | GPIO_IN);              /*  TXD ssp2 ALF in 1 */
	set_GPIO_mode(GPIO_NSSP_RXD2  | GPIO_IN);		/* RXD ssp2 ALF in 1*/

        AUDPRINTk1("ezx_BTVR_nssp_init ok\n");
	return 0;
}


static void __exit ezx_BTVR_nssp_exit(void)
{
	unregister_sound_audio(ezx_BTVR_nssp_audio_state.dev_dsp);
        AUDPRINTk1("ezx_BTVR_nssp exit ok\n");
}


module_init(ezx_BTVR_nssp_init);
module_exit(ezx_BTVR_nssp_exit);


