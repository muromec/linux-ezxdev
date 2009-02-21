/*
 * Copyright (C) 2002-2004 Motorola Inc.
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
 *  zhouqiong          Aug 02,2002             created
 *  zhouqiong          Mar 03,2003             (1) don't close headset interrupt;
 *                                             (2) move bluetooth to phone device ioctl
 *  LiYong             Sep 23,2003             Port from EZX
 *  Jin Lihong(w20076) Mar.15,2004,LIBdd86574  mixer bug fix
 *  Jin Lihong(w20076) Mar.25,2004,LIBdd90336  play music noise tmp solution
 *
 */
 
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/poll.h>
#include <linux/pm.h>
#include <linux/sound.h>
#include <linux/soundcard.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>
#include <asm/dma.h>

#include "ezx-audio.h"
#include "ezx-common.h"

static int phone_init(void);
static void phone_shutdown(void);
static DECLARE_MUTEX(phone_mutex);
static int start_va(void);
static void stop_va(void);

static int phone_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
#ifdef EZX_OSS_DEBUG
	unsigned long ssp_pcap_register_val;
#endif

	switch(cmd) {
		case BLUETOOTH_AUDIO_ON:

		AUDPRINTk1(EZXOSS_DEBUG "bluetooth audio on\n");

		power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_EN_INDEX, PCAP_BIT_CLEAN_VALUE, CDC_EN_NUM_BITS );
#ifdef EZX_OSS_DEBUG
		power_ic_read_reg(PCAP_AUD_CODEC, &ssp_pcap_register_val);
#endif
		AUDPRINTk1(EZXOSS_DEBUG "AUD_CODEC=0x%x\n", ssp_pcap_register_val);

		power_ic_set_reg_value( PCAP_RX_AUD_AMPS, PGA_R_EN_INDEX, PCAP_BIT_CLEAN_VALUE, PGA_R_EN_NUM_BITS );
              (*mixer_close_output_path[codec_output_path])();
              (*mixer_close_input_path[codec_input_path])();
		break;
		
	case BLUETOOTH_AUDIO_OFF:
		AUDPRINTk1(EZXOSS_DEBUG "bluetooth audio off\n");

		power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_EN_INDEX, PCAP_BIT_SET_VALUE, CDC_EN_NUM_BITS );
#ifdef EZX_OSS_DEBUG
		power_ic_read_reg(PCAP_AUD_CODEC, &ssp_pcap_register_val);
#endif
		AUDPRINTk1(EZXOSS_DEBUG "AUD_CODEC=0x%x\n", ssp_pcap_register_val);

		power_ic_set_reg_value( PCAP_RX_AUD_AMPS, PGA_R_EN_INDEX, PCAP_BIT_SET_VALUE, PGA_R_EN_NUM_BITS );
		set_audio_output(codec_output_path);
		set_audio_input(codec_input_path);
		break;

	case EZX_START_VA:
		return start_va();
	case EZX_STOP_VA:
		stop_va();
		break;
		
	default:
		return mixer_ioctl(inode, file, cmd, arg);
	}
	return 0;
}


static int start_va(void)
{
	unsigned long flags;
	volatile unsigned long audiostatus;
	unsigned long timeout;

	AUDPRINTk1(EZXOSS_DEBUG "start va \n");
	down(&phone_mutex);

	local_irq_save(flags);
	CKEN |= CKEN3_NSSP;	                                /* need enable cken3 */

	NSSCR0 = NSSCR0_FRF_PSP | NSSCR0_DSS_16bit | NSSCR0_MOD_NET | NSSCR0_FRDC_4;
	NSSCR1 = NSSCR1_TTE | NSSCR1_TTELP | NSSCR1_EBCEI | NSSCR1_SCLKDIR | NSSCR1_SFRMDIR | NSSCR1_TFTH_4 | NSSCR1_RFTH_12;
	NSSPSP = NSSPSP_SFRMWDTH_1 | NSSPSP_SFRMP_HIGH | NSSPSP_SCMODE | NSSPSP_DMYSTRT_1;
	NSSTSA = NSSTSA_TTSA_0;
	NSSRSA = NSSRSA_RTSA_9;

	set_GPIO_mode(GPIO_BITCLK_IN_NSSP_MD);                  /* BitCLK */
	set_GPIO_mode(GPIO_SYNC_IN_NSSP_MD);                    /* FS */
	set_GPIO_mode(GPIO_SDATA_OUT_NSSP_MD);		        /* TXD ssp2 ALF out 2 */
	set_GPIO_mode(GPIO_SDATA_IN_NSSP_MD);	                /* RXD ssp2 ALF in 1*/
	set_GPIO_mode(GPIO_VA_NSSP_SWAP_MD);

	NSSCR1 |= NSSCR1_RSRE;	              			/* enable dma request */
	NSSCR0 |= NSSCR0_SSE;	                                /* enable nssp controller */
	local_irq_restore(flags);

	AUDPRINTk1(EZXOSS_DEBUG "NSSCR0 = 0x%lx\n", NSSCR0);
	AUDPRINTk1(EZXOSS_DEBUG "NSSCR1 = 0x%lx\n", NSSCR1);
	AUDPRINTk1(EZXOSS_DEBUG "NSSPSP = 0x%lx\n", NSSPSP);
	AUDPRINTk1(EZXOSS_DEBUG "NSSTSA = 0x%lx\n", NSSTSA);
	AUDPRINTk1(EZXOSS_DEBUG "NSSRSA = 0x%lx\n", NSSRSA);

	timeout = 0;
	/* check if ssp is ready for slave operation	*/
	while(((audiostatus = NSSSR) & NSSSR_CSS) !=0){
		if((timeout++) > 10000000)
			goto err;
	}
	AUDPRINTk1(EZXOSS_DEBUG "complete all hardware init \n");
	up(&phone_mutex);
	return 0;

err:
	up(&phone_mutex);
	AUDPRINTk1(EZXOSS_DEBUG "audio panic: ssp don't ready for slave operation!!! ");
	return -ENODEV;	
}


static void stop_va(void)
{
	down(&phone_mutex);
	AUDPRINTk1(EZXOSS_DEBUG "stop va\n");

	NSSCR0 = 0;
	NSSCR1 = 0;
	NSSPSP = 0;
	NSSTSA = 0;
	NSSRSA = 0;
	CKEN &= ~CKEN3_NSSP;    /* SSP2 need control the CKEN3  */ 
	set_GPIO_mode(GPIO_NSSP_SCLK2 | GPIO_IN);                /* BitCLK */
        set_GPIO_mode(GPIO_NSSP_SFRM2 | GPIO_IN);                /* FS */
        set_GPIO_mode(GPIO_NSSP_TXD2  | GPIO_IN);
        set_GPIO_mode(GPIO_NSSP_RXD2  | GPIO_IN);

	up(&phone_mutex);
}


static int phone_init(void)
{
	AUDPRINTk1(EZXOSS_DEBUG "open phone device, init pcap register\n");

	down(&phone_mutex);

	set_pcap_telephone_codec();
	poweron_mixer(CALL_DEVICE);
	set_audio_output(codec_output_path);
	set_audio_input(codec_input_path);
	set_input_gain_hw_reg();

	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, CDC_SW_INDEX, PCAP_BIT_SET_VALUE, CDC_SW_NUM_BITS );
	pcap_use_bp_13m_clock();

	print_pcap_audio_reg_vals();
	up(&phone_mutex);

	return 0;
}


static void phone_shutdown(void)
{
	down(&phone_mutex);

	shutdown_mixer(CALL_DEVICE);
	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, CDC_SW_INDEX, PCAP_BIT_CLEAN_VALUE, CDC_SW_NUM_BITS );

	power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_CLK_EN_INDEX, PCAP_BIT_CLEAN_VALUE, CDC_CLK_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_EN_INDEX, PCAP_BIT_CLEAN_VALUE, CDC_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, SMB_INDEX, PCAP_BIT_SET_VALUE, SMB_NUM_BITS );

	/* set fsync, tx, bitclk are tri-stated */
	power_ic_set_reg_value( PCAP_AUD_CODEC, CD_TS_INDEX, PCAP_BIT_SET_VALUE, CD_TS_NUM_BITS );

	up(&phone_mutex);

	if( audioonflag == 0 )
		pcap_use_bp_13m_clock();
	else
		pcap_use_ap_13m_clock();

	AUDPRINTk1(EZXOSS_DEBUG "close phone device.\n");	
}


static audio_stream_t phone_audio_in = {
	name:			"ezx va",
	dcmd:			DCMD_RXNSSDR,
	drcmr:			&DRCMRRXNSSDR,  /* nssp dma map register */
	dev_addr:		__PREG(NSSDR),
};


static audio_state_t phone_audio_state = {
	output_stream:		NULL,
	input_stream:		&phone_audio_in,
	client_ioctl:		phone_ioctl,
	hw_init:		phone_init,
	hw_shutdown:		phone_shutdown,
	sem:			__MUTEX_INITIALIZER(phone_audio_state.sem),
};


static int phone_open(struct inode *inode, struct file *file)
{
	AUDPRINTk1(EZXOSS_DEBUG "phone_open\n");
	
	return cotulla_audio_attach(inode, file, &phone_audio_state);
}


/*
 * Missing fields of this structure will be patched with the call
 * to cotulla_audio_attach().
 */
static struct file_operations phone_audio_fops = {
	open:		phone_open,
	owner:		THIS_MODULE
};


static int __init phone_register(void)
{
	phone_audio_state.dev_dsp = register_sound_phone(&phone_audio_fops, -1);

	set_GPIO_mode(GPIO_NSSP_SCLK2 | GPIO_IN);		/* BitCLK */
	set_GPIO_mode(GPIO_NSSP_SFRM2 | GPIO_IN);		/* FS */
	set_GPIO_mode(GPIO_NSSP_TXD2  | GPIO_IN);
	set_GPIO_mode(GPIO_NSSP_RXD2  | GPIO_IN);		/* RXD ssp2 ALF in 1*/

	AUDPRINTk1(EZXOSS_DEBUG "register phone device with kernel ok \n");

	return 0;
}


static void __exit phone_unregister(void)
{
	unregister_sound_phone(phone_audio_state.dev_dsp);

	AUDPRINTk1(EZXOSS_DEBUG "unregister phone device \n");
}


module_init(phone_register);
module_exit(phone_unregister);

