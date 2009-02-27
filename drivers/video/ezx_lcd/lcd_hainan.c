/*
 * linux/drivers/video/ezx_lcd/hainan.c
 * Intel Bulverde/PXA250/210 LCD Controller operations for Moto HAINAN product
 * 
 * Copyright (C) 2004-2005 - Motorola
 * 
 * Copyright 2003 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *	   source@mvista.com
 * Copyright (C) 2003 Intel Corporation (yu.tang@intel.com)
 * Copyright (C) 1999 Eric A. Thomas
 * Based on acornfb.c Copyright (C) Russell King.
 *
 * Please direct your questions and comments on this driver to the following
 * email address:
 *
 *	linux-arm-kernel@lists.arm.linux.org.uk
 *
 * Code Status:
 *
 * 2001/08/03: <cbrake@accelent.com>
 *      - Ported from SA1100 to PXA250
 *      - Added Overlay 1 & Overlay2 & Hardware Cursor support
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED	  ``AS	IS'' AND   ANY	EXPRESS OR IMPLIED
 *  WARRANTIES,	  INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO	EVENT  SHALL   THE AUTHOR  BE	 LIABLE FOR ANY	  DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED	  TO, PROCUREMENT OF  SUBSTITUTE GOODS	OR SERVICES; LOSS OF
 *  USE, DATA,	OR PROFITS; OR	BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN	 CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * 2004/06/28  Susan
 *      - Support Hainan product based on new LCD structure
 * 2005/12/12  Wang limei
 *      - Remove FL related operation from BK turning on/off to reduce time;
 *	 - Change mdelay to schedule_timeout() to improvement system performance;
 *	 - Update GPIO setting up method to make it easy to read;
*/

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
#include <linux/device.h>

#include <linux/lights_funlights.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>

#include "lcd_hainan.h"
#include "pxafb.h"


extern struct pxafb_info *pxafb_main;
extern struct pxafb_info *pxafb_smart;
extern struct global_state pxafb_global_state;

extern struct device pxafb_main_device_ldm;

void pxafb_bklight_turn_on(void *pxafbinfo)
{
	struct pxafb_info *fbi = (struct pxafb_info *)pxafbinfo;
	
	if (fbi == pxafb_main)
	{
	    //Adjust brightness -- configure PWM0 register 
	    CKEN |= CKEN0_PWM0;
	    PWM_CTRL0 = BKLIGHT_PRESCALE;
	    PWM_PERVAL0 = BKLIGHT_PERIOD;
	    // gbklightDutyCycle should got from stup file system 
	    PWM_PWDUTY0   = pxafb_global_state.bklight_main_dutycycle;

	    // configure GPIO for PWM0 
	    
	    set_GPIO_mode(GPIO16_PWM0_MD);

	    pxafb_global_state.main_state |= 0x1; //0b01
	    pxafb_global_state.smart_state &= 0x6; //0b110
	    
	    DPRINTK("\npxafb_bklight_turn_on.\n");
	    DPRINTK("\nCKEN: %x", CKEN);
	    DPRINTK("\nPWM_CTRL0: %d", PWM_CTRL0);
	    DPRINTK("\nPWM_PERVAL0: %d", PWM_PERVAL0);
	    DPRINTK("\nPWM_PWDUTY0: %d", PWM_PWDUTY0);
	}
}

void pxafb_ezx_Backlight_turn_on(void *pxafbinfo)
{
	struct pxafb_info *fbi = (struct pxafb_info *)pxafbinfo;

	down(&pxafb_global_state.g_sem);
	
	if (fbi == pxafb_main)
	{
		/*Check if Main panel has not been enabled, if so, don't turn on its backlight */
		if ( !(pxafb_global_state.main_state & PXAFB_PANEL_ON) )
		{
			up(&pxafb_global_state.g_sem);
			return;
		}

	    //Adjust brightness -- configure PWM0 register 
	    CKEN |= CKEN0_PWM0;
	    PWM_CTRL0 = BKLIGHT_PRESCALE;
	    PWM_PERVAL0 = BKLIGHT_PERIOD;
	    // gbklightDutyCycle should got from stup file system 
	    PWM_PWDUTY0   = pxafb_global_state.bklight_main_dutycycle;

	    // configure GPIO for PWM0 
	    
	    set_GPIO_mode(GPIO16_PWM0_MD);

	    pxafb_global_state.main_state |= 0x1; //0b01
	    pxafb_global_state.smart_state &= 0x6; //0b110
	    
	    DPRINTK("\npxafb_ezx_Backlight_turn_on.\n");
	    DPRINTK("\nCKEN: %x", CKEN);
	    DPRINTK("\nPWM_CTRL0: %d", PWM_CTRL0);
	    DPRINTK("\nPWM_PERVAL0: %d", PWM_PERVAL0);
	    DPRINTK("\nPWM_PWDUTY0: %d", PWM_PWDUTY0);
	}
	up(&pxafb_global_state.g_sem);
}


void pxafb_ezx_Backlight_turn_off(void *pxafbinfo)
{
	struct pxafb_info *fbi = (struct pxafb_info *)pxafbinfo;
	
    DPRINTK("\npxafb_ezx_Backlight_turn_off.\n");

    down(&pxafb_global_state.g_sem);

	/* Don't need to check if the requested backlight is for pxafb_current */
	/* It is possible when main is enabled, cli backlight is still turned on, after a while, cli's backlight will be turned off */
	if (fbi == pxafb_main)
	{
	    
	    PWM_PWDUTY0 = 0;
    	    set_GPIO_mode(GPIO16_PWM0|GPIO_OUT);
	    clr_GPIO(GPIO16_PWM0);	//PWM0 is GPIO16, make PWM0 output low level to save power when turn off bklight
	    CKEN &= ~CKEN0_PWM0;
	    PWM_PWDUTY0	 = 0;
	    //gbklightDutyCycle = MIN_DUTYCYCLE;

	    pxafb_global_state.main_state &= 0x06;//0b110
	    DPRINTK("\n MAIN panel: CKEN: %x", CKEN);
	    DPRINTK("\nPWM_PWDUTY0: %x", PWM_PWDUTY0);
	}
	
	up(&pxafb_global_state.g_sem);
}

void pxafb_setup_gpio(void *pxafbinfo)
{
	unsigned int lccr0;
	struct pxafb_info *fbi = (struct pxafb_info *)pxafbinfo;

	/*
	 * setup is based on type of panel supported
	 */
	if (fbi == pxafb_main)
	{
		DPRINTK("pxafb_setup_gpio(%d),pxafb_main\n",current->pid);

		lccr0 = fbi->reg_lccr0;

		/* 4 bit interface */
		if ((lccr0 & LCCR0_CMS) && (lccr0 & LCCR0_SDS) && !(lccr0 & LCCR0_DPD))
		{
			//bits 58-61, LDD0 -LDD3
			set_GPIO_mode(GPIO58_LDD_0_MD);
			set_GPIO_mode(GPIO59_LDD_1_MD);
			set_GPIO_mode(GPIO60_LDD_2_MD);
			set_GPIO_mode(GPIO61_LDD_3_MD);			
			// bits 74-77,LCD control signals
			set_GPIO_mode(GPIO74_LCD_FCLK_MD);	
			set_GPIO_mode(GPIO75_LCD_LCLK_MD);	
			set_GPIO_mode(GPIO76_LCD_PCLK_MD);	
			set_GPIO_mode(GPIO77_LCD_ACBIAS_MD);
		}
	    	/* 8 bit interface */
	   	else if (((lccr0 & LCCR0_CMS) && ((lccr0 & LCCR0_SDS) || (lccr0 & LCCR0_DPD))) || (!(lccr0 & LCCR0_CMS) && !(lccr0 & LCCR0_PAS) && !(lccr0 & LCCR0_SDS)))
	    	{
			//bits 58-65, LDD0-LDD7
			set_GPIO_mode(GPIO58_LDD_0_MD);
			set_GPIO_mode(GPIO59_LDD_1_MD);
			set_GPIO_mode(GPIO60_LDD_2_MD);
			set_GPIO_mode(GPIO61_LDD_3_MD);
			set_GPIO_mode(GPIO62_LDD_4_MD);
			set_GPIO_mode(GPIO63_LDD_5_MD);
			set_GPIO_mode(GPIO64_LDD_6_MD);
			set_GPIO_mode(GPIO65_LDD_7_MD);
			//bits 74-77,LCD control signals
			set_GPIO_mode(GPIO74_LCD_FCLK_MD);	
			set_GPIO_mode(GPIO75_LCD_LCLK_MD);	
			set_GPIO_mode(GPIO76_LCD_PCLK_MD);	
			set_GPIO_mode(GPIO77_LCD_ACBIAS_MD);	
		}
		/* 16bpp, 18bpp, 19bpp, 24bpp and 25bpp interface */
		else if (!(lccr0 & LCCR0_CMS) && ((lccr0 & LCCR0_SDS) || (lccr0 & LCCR0_PAS)))
		{
			switch (fbi->max_bpp) 
			{
			case 16:
				// bits 58-73, LDD0-LDD15
				set_GPIO_mode(GPIO58_LDD_0_MD);
				set_GPIO_mode(GPIO59_LDD_1_MD);
				set_GPIO_mode(GPIO60_LDD_2_MD);
				set_GPIO_mode(GPIO61_LDD_3_MD);
				set_GPIO_mode(GPIO62_LDD_4_MD);
				set_GPIO_mode(GPIO63_LDD_5_MD);
				set_GPIO_mode(GPIO64_LDD_6_MD);
				set_GPIO_mode(GPIO65_LDD_7_MD);
				set_GPIO_mode(GPIO66_LDD_8_MD);
				set_GPIO_mode(GPIO67_LDD_9_MD);
				set_GPIO_mode(GPIO68_LDD_10_MD);
				set_GPIO_mode(GPIO69_LDD_11_MD);
				set_GPIO_mode(GPIO70_LDD_12_MD);
				set_GPIO_mode(GPIO71_LDD_13_MD);
				set_GPIO_mode(GPIO72_LDD_14_MD);		
				set_GPIO_mode(GPIO73_LDD_15_MD);
				//bits 74-77 , LCD control signal
				set_GPIO_mode(GPIO74_LCD_FCLK_MD);	
				set_GPIO_mode(GPIO75_LCD_LCLK_MD);	
				set_GPIO_mode(GPIO76_LCD_PCLK_MD);	
				set_GPIO_mode(GPIO77_LCD_ACBIAS_MD);
				break;
			case 18:
			case 19:
			case 24:
			case 25:
				DPRINTK("Set 18-bit interface");
				// bits 58-73 for LDD0-LDD15, bits 86,87 for LDD16,LDD17
				set_GPIO_mode(GPIO58_LDD_0_MD);
				set_GPIO_mode(GPIO59_LDD_1_MD);
				set_GPIO_mode(GPIO60_LDD_2_MD);
				set_GPIO_mode(GPIO61_LDD_3_MD);
				set_GPIO_mode(GPIO62_LDD_4_MD);
				set_GPIO_mode(GPIO63_LDD_5_MD);
				set_GPIO_mode(GPIO64_LDD_6_MD);
				set_GPIO_mode(GPIO65_LDD_7_MD);
				set_GPIO_mode(GPIO66_LDD_8_MD);
				set_GPIO_mode(GPIO67_LDD_9_MD);
				set_GPIO_mode(GPIO68_LDD_10_MD);
				set_GPIO_mode(GPIO69_LDD_11_MD);
				set_GPIO_mode(GPIO70_LDD_12_MD);
				set_GPIO_mode(GPIO71_LDD_13_MD);
				set_GPIO_mode(GPIO72_LDD_14_MD);		
				set_GPIO_mode(GPIO73_LDD_15_MD);
				set_GPIO_mode(GPIO86_LDD_16_MD);		
				set_GPIO_mode(GPIO87_LDD_17_MD);
				//bits 74-77 for LCD control signals
				set_GPIO_mode(GPIO74_LCD_FCLK_MD);	
				set_GPIO_mode(GPIO75_LCD_LCLK_MD);	
				set_GPIO_mode(GPIO76_LCD_PCLK_MD);	
				set_GPIO_mode(GPIO77_LCD_ACBIAS_MD);	
				break;
			default:
				break;
			}
	    	}
	    	else
			printk(KERN_ERR "pxafb_setup_gpio: unable to determine bits per pixel\n");
			
		/* Read & save LCD ID: ID1 = GPIO<18> and ID0 = GPIO<80> */
		/* GPDR set to input at reset */
		fbi->lcd_id = (GPLR0 & 0x00040000) >> 17  | (GPLR2 & 0x00010000) >> 16;

		/* Setup GPIO<19> as general GPIO OUT HIGH */
		set_GPIO_mode(GPIO19_L_CS|GPIO_OUT);	
		set_GPIO(GPIO19_L_CS);			
		mdelay(1);
	}
}

void pxafb_enable_controller(void *pxafbinfo)
{
	struct pxafb_info *fbi = (struct pxafb_info *)pxafbinfo;
	unsigned long expire;
	DPRINTK("Enabling LCD controller");

	/* power_state is initialized in this structure to
	   DPM_POWER_OFF; make sure to initialize it to ON when the
	   controller is enabled.

	   Must use the global since this function has no other access
	   to pxafb_device_ldm.
	*/
	if (fbi == pxafb_main) 
	{
		DPRINTK("pxafb_enable_controller(%d):pxafb_main\n",current->pid);

		pxafb_main_device_ldm.power_state = DPM_POWER_ON;
	#ifdef CONFIG_CPU_BULVERDE
		/* workaround for insight 41187 */
		OVL1C2 = 0;
		OVL1C1 = 0;
		OVL2C2 = 0;
		OVL2C1 = 0;
		CCR = 0;

		LCCR4 |= (1<<31) | (1<<25) | (4<<17);
		LCCR3 = fbi->reg_lccr3;
		LCCR2 = fbi->reg_lccr2;
		LCCR1 = fbi->reg_lccr1;
		LCCR0 = fbi->reg_lccr0 & ~LCCR0_ENB;

		LCCR0 |= LCCR0_ENB;
		LCCR0 |= LCCR0_DIS;

		LCCR3 = fbi->reg_lccr3;
		LCCR2 = fbi->reg_lccr2;
		LCCR1 = fbi->reg_lccr1;
		LCCR0 = fbi->reg_lccr0 & ~LCCR0_ENB;

		LCCR0 |= LCCR0_ENB;
		LCCR0 &= ~LCCR0_ENB;

		FDADR0 = fbi->fdadr0;
		FDADR1 = fbi->fdadr1;
		

		if (PXAFB_MAIN_VALID_FB == PXAFB_MAIN_OVL1)  //overlay1fb is significant fb of _main_ panel
		{
			LCCR4 = (LCCR4 & (~(0x3<<15))) | (0x2<<15);/* PAL_FOR = "10" */		
			/* PDFOR = "11" */
			LCCR3 = (LCCR3 & (~(0x3<<30))) | (0x3<<30);
		}

		LCCR0 |= LCCR0_ENB;

	#else
		/* Sequence from 11.7.10 */
		LCCR3 = fbi->reg_lccr3;
		LCCR2 = fbi->reg_lccr2;
		LCCR1 = fbi->reg_lccr1;
		LCCR0 = fbi->reg_lccr0 & ~LCCR0_ENB;

		/* FIXME we used to have LCD power control here */

		FDADR0 = fbi->fdadr0;
		FDADR1 = fbi->fdadr1;
		LCCR0 |= LCCR0_ENB;
	#endif

		
		set_GPIO(GPIO78_LCD_SD);		/* Turn on LCD_SD(GPIO<78>) */
		clr_GPIO(GPIO78_LCD_SD);		/* LCD is enabled by setting LCD_SD low*/
		set_GPIO_out(GPIO78_LCD_SD);	

		clr_GPIO(GPIO79_LCD_CM);		/* Set LCD_CM(GPIO<79>) to be normal mode */ 
		set_GPIO_out(GPIO79_LCD_CM);	

		/*Waiting for display on, according to the spec 72r89803y01 spec requires 166ms, based on my testing, it should be at least 200ms*/
		set_current_state(TASK_UNINTERRUPTIBLE);
		expire = schedule_timeout(TFT_PANEL_TURNON_DELAY_IN_JIFFIES);

		//remove it -- bLCDOn = 1;

		//DPRINTK("FDADR0 = 0x%08x\n", (unsigned int)FDADR0);
		//DPRINTK("FDADR1 = 0x%08x\n", (unsigned int)FDADR1);
		//DPRINTK("LCCR0 = 0x%08x\n", (unsigned int)LCCR0);
		//DPRINTK("LCCR1 = 0x%08x\n", (unsigned int)LCCR1);
		//DPRINTK("LCCR2 = 0x%08x\n", (unsigned int)LCCR2);
		//DPRINTK("LCCR3 = 0x%08x\n", (unsigned int)LCCR3);
		//DPRINTK("LCCR4 = 0x%08x", (unsigned int)LCCR4);
	}

}

void pxafb_disable_controller(void *pxafbinfo)
{
	struct pxafb_info *fbi = (struct pxafb_info *)pxafbinfo;
	unsigned long expire;
	DPRINTK("Disabling LCD controller");

	/* Think about CLI/MAIN panel case, take semaphore in <set_ctrlr_state> before panel switchings -- Susan */
	if (fbi == pxafb_main)
	{
		DPRINTK("pxafb_disable_controller(%d):pxafb_main\n",current->pid);

		DPRINTK("pxafb_disable_controller: FBR0 (0x%x)\n", FBR0);
		while(FBR0 & 0x1)  //waiting for the completion of concurrent FBR branch cmd//
		;

		LCSR0 = 0xffffffff;	/* Clear LCD Status Register */
		
		/* needed to turn off LCD screen */
		DPRINTK("pxafb_disable_controller(%d):disable GPIOs for LCD_72R89341N\n",current->pid);

		clr_GPIO(GPIO78_LCD_SD);		/* LCD_SD(GPIO<78>), rising edge for turn off */
		set_GPIO(GPIO78_LCD_SD);	
		set_GPIO_out(GPIO78_LCD_SD);	
		PGSR2 |= 0x00004000;  /* set sleep state of GPIO<78> */

		set_current_state(TASK_UNINTERRUPTIBLE);
		expire = schedule_timeout(TFT_PANEL_TURNOFF_DELAY_IN_JIFFIES);

		set_GPIO(GPIO79_LCD_CM);	  /* Susan: LCD_CM(GPIO<79>), high for partial mode */

		LCCR0 &= ~LCCR0_LDM;	/* Enable LCD Disable Done Interrupt */
	//	LCCR0 &= ~LCCR0_ENB;	/* Disable LCD Controller */
		LCCR0 |= LCCR0_DIS;   //Normal disable LCD //

		set_current_state(TASK_UNINTERRUPTIBLE);
		expire = schedule_timeout(TO_JIFFIES(18));//18msec
		pxafb_main_device_ldm.power_state = DPM_POWER_OFF;
	}

}

