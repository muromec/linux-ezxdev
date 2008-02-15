/*
 * Copyright (C) 2003 Motorola Inc.
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
 *                           Modification     Tracking
 *  Author (core ID)                Date          Number     Description of Changes
 *  -------------------------   ------------    ----------   ----------------------------
 *  Liu weijie (A19553)         11/01/2003       LIBdd43835   Init create 
 *  Wang Jamshid(a5036c)        04/29/2004       LIBee01180   fix bug,register pm call back
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <linux/init.h>
#include <linux/led_pxa_e680.h>
#include <linux/pm.h>
#include "../misc/ssp_pcap.h"

#ifdef  CONFIG_E680_P4A
#define IND_CNTL_R_BUL 79
#define IND_CNTL_G_BUL 18
#else
#define IND_CNTL_R_BUL 46
#define IND_CNTL_G_BUL 47
#endif

#define SSP_PCAP_LED_MASK  0x000fffe0
#define SSP_PCAP_LED_SHIFT 5

#define VERSION "1.0"

//#define DEBUG_LED_DRIVER        

#ifdef DEBUG_LED_DRIVER
#define PRINTK(s...)  printk(s)
#else
#define PRINTK(s...)
#endif
typedef struct
{
  unsigned char ind_GPIO_red;   /*Indicator Red control GPIO 79: 0 active, 1 disactive*/
  unsigned char ind_GPIO_green;  /*Indicator Green control GPIO 18: 0 active, 1 disactive*/
  unsigned char pcap_LEDR_en;   /*pcap LEDR_EN bit value: 1 =Red LED(&Green) 
				  sink circuit enabled*/
  unsigned char pcap_LEDG_en;   /*pcap LEDG_EN bit value:1 =Green(->Blue)LED 
				  sink circuit enabled*/
  unsigned char pcap_LEDR_CTRL; /* 4bits Sets the timing for the red(&Green) LED
				  sink circuit*/
  unsigned char pcap_LEDG_CTRL; /* 4bits Sets the timing for the GREEN (->Blue) LED 
				   sink circuit*/
  unsigned char pcap_LEDR_I;   /* 2 bits 00 3mA,01 4mA, 10 5mA, 11 9mA */
                               /* sets the pulsed current level for LEDR*/
  unsigned char pcap_LEDG_I;  
  unsigned char pcap_SKIP_on;  /*1=The ON timing sequence defined by LEDx_CTRL
				   is executed on every other cycle*/
}PCAP2_LED_REGISTER_VALUE;

const PCAP2_LED_REGISTER_VALUE led_register_value [LED_BOTTOM_STATE +1 ]=
{  
   {0x1,0x1, 0x0,0x0, 0x0,0x0, 0x0,0x0,0x0}, /*LED_OFF*/
	      
   /*Always On*/
   {0x0,0x1, 0x1,0x0, 0xc,0x0, 0x1,0x0,0x0}, /*LED_RED_ALWAYS_ON,skip off*/
   {0x1,0x0, 0x1,0x0, 0xc,0x0, 0x1,0x0,0x0}, /*LED_GREEN_ALWAYS_ON*/
   {0x1,0x1, 0x0,0x1, 0x0,0xc, 0x0,0x0,0x0}, /*LED_BLUE_ALWAYS_ON*/
   {0x0,0x1, 0x1,0x1, 0xc,0xc, 0x1,0x0,0x0}, /*LED_LIGHT_RED_ALWAYS_ON*/
   {0x0,0x0, 0x1,0x0, 0xc,0x0, 0x1,0x0,0x0}, /*LED_ORANGE_ALWAYS_ON*/
   {0x1,0x0, 0x1,0x1, 0xc,0xc, 0x1,0x0,0x0}, /*LED_LIGHT_BLUE_ALWAYS_ON*/
   {0x0,0x0, 0x1,0x1, 0xc,0xc, 0x1,0x0,0x0}, /*LED_WHITE_ALWAYS_ON*/

   /* RED 5 mA current Skip OFF*/
   {0x0,0x1, 0x1,0x0, 0x1,0x0, 0x1,0x0,0x0}, /*LED_RED_01_ON_19_OFF,skip off*/
   {0x0,0x1, 0x1,0x0, 0x2,0x0, 0x1,0x0,0x0}, /*LED_RED_02_ON_18_OFF*/
   {0x0,0x1, 0x1,0x0, 0x3,0x0, 0x1,0x0,0x0}, /*LED_RED_05_ON_15_OFF*/
   {0x0,0x1, 0x1,0x0, 0x4,0x0, 0x1,0x0,0x0}, /*LED_RED_025_ON_075_OFF*/
   {0x0,0x1, 0x1,0x0, 0x5,0x0, 0x1,0x0,0x0}, /*LED_RED_025_ON_175_OFF*/
   {0x0,0x1, 0x1,0x0, 0x6,0x0, 0x1,0x0,0x0}, /*LED_RED_005_ON_195_OFF*/
   {0x0,0x1, 0x1,0x0, 0x7,0x0, 0x1,0x0,0x0}, /*LED_RED_05_ON_05_OFF*/
   {0x0,0x1, 0x1,0x0, 0x8,0x0, 0x1,0x0,0x0}, /*LED_RED_05_OFF_05_ON*/
   {0x0,0x1, 0x1,0x0, 0x9,0x0, 0x1,0x0,0x0}, /*LED_RED_0125_ON_05_OFF_0075_ON_15_OFF*/
   {0x0,0x1, 0x1,0x0, 0xa,0x0, 0x1,0x0,0x0}, /*LED_RED_0125_ON_2075_OFF*/
   {0x0,0x1, 0x1,0x0, 0xb,0x0, 0x1,0x0,0x0}, /*LED_RED_0625_OFF_0075_ON_15_OFF*/

   /* RED Skip ON*/
   {0x0,0x1, 0x1,0x0, 0x1,0x0, 0x1,0x0,0x1}, /*LED_RED_01_ON_19_OFF,skip On*/
   {0x0,0x1, 0x1,0x0, 0x2,0x0, 0x1,0x0,0x1}, /*LED_RED_02_ON_18_OFF*/
   {0x0,0x1, 0x1,0x0, 0x3,0x0, 0x1,0x0,0x1}, /*LED_RED_05_ON_15_OFF*/
   {0x0,0x1, 0x1,0x0, 0x4,0x0, 0x1,0x0,0x1}, /*LED_RED_025_ON_075_OFF*/
   {0x0,0x1, 0x1,0x0, 0x5,0x0, 0x1,0x0,0x1}, /*LED_RED_025_ON_175_OFF*/
   {0x0,0x1, 0x1,0x0, 0x6,0x0, 0x1,0x0,0x1}, /*LED_RED_005_ON_195_OFF*/
   {0x0,0x1, 0x1,0x0, 0x7,0x0, 0x1,0x0,0x1}, /*LED_RED_05_ON_05_OFF*/
   {0x0,0x1, 0x1,0x0, 0x8,0x0, 0x1,0x0,0x1}, /*LED_RED_05_OFF_05_ON*/
   {0x0,0x1, 0x1,0x0, 0x9,0x0, 0x1,0x0,0x1}, /*LED_RED_0125_ON_05_OFF_0075_ON_15_OFF*/
   {0x0,0x1, 0x1,0x0, 0xa,0x0, 0x1,0x0,0x1}, /*LED_RED_0125_ON_2075_OFF*/
   {0x0,0x1, 0x1,0x0, 0xb,0x0, 0x1,0x0,0x1}, /*LED_RED_0625_OFF_0075_ON_15_OFF*/

   /*GREEN Skip OFF*/
   {0x1,0x0, 0x1,0x0, 0x1,0x0, 0x1,0x0,0x0}, /*LED_GREEN_01_ON_19_OFF,skip off*/
   {0x1,0x0, 0x1,0x0, 0x2,0x0, 0x1,0x0,0x0}, /*LED_GREEN_02_ON_18_OFF*/
   {0x1,0x0, 0x1,0x0, 0x3,0x0, 0x1,0x0,0x0}, /*LED_GREEN_05_ON_15_OFF*/
   {0x1,0x0, 0x1,0x0, 0x4,0x0, 0x1,0x0,0x0}, /*LED_GREEN_025_ON_075_OFF*/
   {0x1,0x0, 0x1,0x0, 0x5,0x0, 0x1,0x0,0x0}, /*LED_GREEN_025_ON_175_OFF*/
   {0x1,0x0, 0x1,0x0, 0x6,0x0, 0x1,0x0,0x0}, /*LED_GREEN_005_ON_195_OFF*/
   {0x1,0x0, 0x1,0x0, 0x7,0x0, 0x1,0x0,0x0}, /*LED_GREEN_05_ON_05_OFF*/
   {0x1,0x0, 0x1,0x0, 0x8,0x0, 0x1,0x0,0x0}, /*LED_GREEN_05_OFF_05_ON*/
   {0x1,0x0, 0x1,0x0, 0x9,0x0, 0x1,0x0,0x0}, /*LED_GREEN_0125_ON_05_OFF_0075_ON_15_OFF*/
   {0x1,0x0, 0x1,0x0, 0xa,0x0, 0x1,0x0,0x0}, /*LED_GREEN_0125_ON_2075_OFF*/
   {0x1,0x0, 0x1,0x0, 0xb,0x0, 0x1,0x0,0x0}, /*LED_GREEN_0625_OFF_0075_ON_15_OFF*/

   /*GREEN Skip ON*/
   {0x1,0x0, 0x1,0x0, 0x1,0x0, 0x1,0x0,0x1}, /*LED_GREEN_01_ON_19_OFF,skip On*/
   {0x1,0x0, 0x1,0x0, 0x2,0x0, 0x1,0x0,0x1}, /*LED_GREEN_02_ON_18_OFF*/
   {0x1,0x0, 0x1,0x0, 0x3,0x0, 0x1,0x0,0x1}, /*LED_GREEN_05_ON_15_OFF*/
   {0x1,0x0, 0x1,0x0, 0x4,0x0, 0x1,0x0,0x1}, /*LED_GREEN_025_ON_075_OFF*/
   {0x1,0x0, 0x1,0x0, 0x5,0x0, 0x1,0x0,0x1}, /*LED_GREEN_025_ON_175_OFF*/
   {0x1,0x0, 0x1,0x0, 0x6,0x0, 0x1,0x0,0x1}, /*LED_GREEN_005_ON_195_OFF*/
   {0x1,0x0, 0x1,0x0, 0x7,0x0, 0x1,0x0,0x1}, /*LED_GREEN_05_ON_05_OFF*/
   {0x1,0x0, 0x1,0x0, 0x8,0x0, 0x1,0x0,0x1}, /*LED_GREEN_05_OFF_05_ON*/
   {0x1,0x0, 0x1,0x0, 0x9,0x0, 0x1,0x0,0x1}, /*LED_GREEN_0125_ON_05_OFF_0075_ON_15_OFF*/
   {0x1,0x0, 0x1,0x0, 0xa,0x0, 0x1,0x0,0x1}, /*LED_GREEN_0125_ON_2075_OFF*/
   {0x1,0x0, 0x1,0x0, 0xb,0x0, 0x1,0x0,0x1}, /*LED_GREEN_0625_OFF_0075_ON_15_OFF*/

   /*BLUE 5 mA Current Skip OFF*/
   {0x1,0x1, 0x0,0x1, 0x0,0x1, 0x0,0x0,0x0}, /*LED_BLUE_01_ON_19_OFF,skip off*/
   {0x1,0x1, 0x0,0x1, 0x0,0x2, 0x0,0x0,0x0}, /*LED_BLUE_02_ON_18_OFF*/
   {0x1,0x1, 0x0,0x1, 0x0,0x3, 0x0,0x0,0x0}, /*LED_BLUE_05_ON_15_OFF*/
   {0x1,0x1, 0x0,0x1, 0x0,0x4, 0x0,0x0,0x0}, /*LED_BLUE_025_ON_075_OFF*/
   {0x1,0x1, 0x0,0x1, 0x0,0x5, 0x0,0x0,0x0}, /*LED_BLUE_025_ON_175_OFF*/
   {0x1,0x1, 0x0,0x1, 0x0,0x6, 0x0,0x0,0x0}, /*LED_BLUE_005_ON_195_OFF*/
   {0x1,0x1, 0x0,0x1, 0x0,0x7, 0x0,0x0,0x0}, /*LED_BLUE_05_ON_05_OFF*/
   {0x1,0x1, 0x0,0x1, 0x0,0x8, 0x0,0x0,0x0}, /*LED_BLUE_05_OFF_05_ON*/
   {0x1,0x1, 0x0,0x1, 0x0,0x9, 0x0,0x0,0x0}, /*LED_BLUE_0125_ON_05_OFF_0075_ON_15_OFF*/
   {0x1,0x1, 0x0,0x1, 0x0,0xa, 0x0,0x0,0x0}, /*LED_BLUE_0125_ON_2075_OFF*/
   {0x1,0x1, 0x0,0x1, 0x0,0xb, 0x0,0x0,0x0}, /*LED_BLUE_0625_OFF_0075_ON_15_OFF*/

   /*BLUE 5 mA Current Skip On*/
   {0x1,0x1, 0x0,0x1, 0x0,0x1, 0x0,0x0,0x1}, /*LED_BLUE_01_ON_19_OFF,skip On*/
   {0x1,0x1, 0x0,0x1, 0x0,0x2, 0x0,0x0,0x1}, /*LED_BLUE_02_ON_18_OFF*/
   {0x1,0x1, 0x0,0x1, 0x0,0x3, 0x0,0x0,0x1}, /*LED_BLUE_05_ON_15_OFF*/
   {0x1,0x1, 0x0,0x1, 0x0,0x4, 0x0,0x0,0x1}, /*LED_BLUE_025_ON_075_OFF*/
   {0x1,0x1, 0x0,0x1, 0x0,0x5, 0x0,0x0,0x1}, /*LED_BLUE_025_ON_175_OFF*/
   {0x1,0x1, 0x0,0x1, 0x0,0x6, 0x0,0x0,0x1}, /*LED_BLUE_005_ON_195_OFF*/
   {0x1,0x1, 0x0,0x1, 0x0,0x7, 0x0,0x0,0x1}, /*LED_BLUE_05_ON_05_OFF*/
   {0x1,0x1, 0x0,0x1, 0x0,0x8, 0x0,0x0,0x1}, /*LED_BLUE_05_OFF_05_ON*/
   {0x1,0x1, 0x0,0x1, 0x0,0x9, 0x0,0x0,0x1}, /*LED_BLUE_0125_ON_05_OFF_0075_ON_15_OFF*/
   {0x1,0x1, 0x0,0x1, 0x0,0xa, 0x0,0x0,0x1}, /*LED_BLUE_0125_ON_2075_OFF*/
   {0x1,0x1, 0x0,0x1, 0x0,0xb, 0x0,0x0,0x1}, /*LED_BLUE_0625_OFF_0075_ON_15_OFF*/

   /*ORANGE 5 mA Current Skip OFF*/
   {0x0,0x0, 0x1,0x0, 0x1,0x0, 0x1,0x0,0x0}, /*LED_ORANGE_01_ON_19_OFF,skip off*/
   {0x0,0x0, 0x1,0x0, 0x2,0x0, 0x1,0x0,0x0}, /*LED_ORANGE_02_ON_18_OFF*/
   {0x0,0x0, 0x1,0x0, 0x3,0x0, 0x1,0x0,0x0}, /*LED_ORANGE_05_ON_15_OFF*/
   {0x0,0x0, 0x1,0x0, 0x4,0x0, 0x1,0x0,0x0}, /*LED_ORANGE_025_ON_075_OFF*/
   {0x0,0x0, 0x1,0x0, 0x5,0x0, 0x1,0x0,0x0}, /*LED_ORANGE_025_ON_175_OFF*/
   {0x0,0x0, 0x1,0x0, 0x6,0x0, 0x1,0x0,0x0}, /*LED_ORANGE_005_ON_195_OFF*/
   {0x0,0x0, 0x1,0x0, 0x7,0x0, 0x1,0x0,0x0}, /*LED_ORANGE_05_ON_05_OFF*/
   {0x0,0x0, 0x1,0x0, 0x8,0x0, 0x1,0x0,0x0}, /*LED_ORANGE_05_OFF_05_ON*/
   {0x0,0x0, 0x1,0x0, 0x9,0x0, 0x1,0x0,0x0}, /*LED_ORANGE_0125_ON_05_OFF_0075_ON_15_OFF*/
   {0x0,0x0, 0x1,0x0, 0xa,0x0, 0x1,0x0,0x0}, /*LED_ORANGE_0125_ON_2075_OFF*/
   {0x0,0x0, 0x1,0x0, 0xb,0x0, 0x1,0x0,0x0}, /*LED_ORANGE_0625_OFF_0075_ON_15_OFF*/

   /*ORANGE 5 mA current Skip On*/
   {0x0,0x0, 0x1,0x0, 0x1,0x0, 0x1,0x0,0x1}, /*LED_ORANGE_01_ON_19_OFF,skip On*/
   {0x0,0x0, 0x1,0x0, 0x2,0x0, 0x1,0x0,0x1}, /*LED_ORANGE_02_ON_18_OFF*/
   {0x0,0x0, 0x1,0x0, 0x3,0x0, 0x1,0x0,0x1}, /*LED_ORANGE_05_ON_15_OFF*/
   {0x0,0x0, 0x1,0x0, 0x4,0x0, 0x1,0x0,0x1}, /*LED_ORANGE_025_ON_075_OFF*/
   {0x0,0x0, 0x1,0x0, 0x5,0x0, 0x1,0x0,0x1}, /*LED_ORANGE_025_ON_175_OFF*/
   {0x0,0x0, 0x1,0x0, 0x6,0x0, 0x1,0x0,0x1}, /*LED_ORANGE_005_ON_195_OFF*/
   {0x0,0x0, 0x1,0x0, 0x7,0x0, 0x1,0x0,0x1}, /*LED_ORANGE_05_ON_05_OFF*/
   {0x0,0x0, 0x1,0x0, 0x8,0x0, 0x1,0x0,0x1}, /*LED_ORANGE_05_OFF_05_ON*/
   {0x0,0x0, 0x1,0x0, 0x9,0x0, 0x1,0x0,0x1}, /*LED_ORANGE_0125_ON_05_OFF_0075_ON_15_OFF*/
   {0x0,0x0, 0x1,0x0, 0xa,0x0, 0x1,0x0,0x1}, /*LED_ORANGE_0125_ON_2075_OFF*/
   {0x0,0x0, 0x1,0x0, 0xb,0x0, 0x1,0x0,0x1}, /*LED_ORANGE_0625_OFF_0075_ON_15_OFF*/

   /*LED_RED_0125_ON_2075_OFF_BLUE_0625_OFF_0075_ON_15_OFF*/
   {0x0,0x1, 0x1,0x1, 0xa,0xb, 0x1,0x0,0x0},
   /*LED_GREEN_0125_ON_2075_OFF_BLUE_0625_OFF_0075_ON_15_OFF*/
   {0x1,0x0, 0x1,0x1, 0xa,0xb, 0x1,0x0,0x0},
   /*LED_ORANGE_0125_ON_2075_OFF_BLUE_0625_OFF_0075_ON_15_OFF*/
   {0x0,0x0, 0x1,0x1, 0xa,0xb, 0x1,0x0,0x0},
   
   /*LED_RED_0125_ON_2075_OFF_BLUE_0625_OFF_0075_ON_15_OFF_SKIP*/
   {0x0,0x1, 0x1,0x1, 0xa,0xb, 0x1,0x0,0x1},
   /*LED_GREEN_0125_ON_2075_OFF_BLUE_0625_OFF_0075_ON_15_OFF_SKIP*/
   {0x1,0x0, 0x1,0x1, 0xa,0xb, 0x1,0x0,0x1},
   /*LED_ORANGE_0125_ON_2075_OFF_BLUE_0625_OFF_0075_ON_15_OFF_SKIP*/
   {0x0,0x0, 0x1,0x1, 0xa,0xb, 0x1,0x0,0x1},

   {0,    0,  0,  0,  0, 0,   0,  0,  0} 
};

#ifdef DEBUG_LED_DRIVER
char LED_Name[LED_BOTTOM_STATE +1][100]=
{
	"LED_OFF",

	"LED_RED_ALWAYS_ON",
	"LED_GREEN_ALWAYS_ON",
	"LED_BLUE_ALWAYS_ON",
	"LED_LIGHT_RED_ALWAYS_ON",
	"LED_ORANGE_ALWAYS_ON",
	"LED_LIGHT_BLUE_ALWAYS_ON",
	"LED_WHITE_ALWAYS_ON",

	"LED_RED_01_ON_19_OFF",
	"LED_RED_02_ON_18_OFF",
	"LED_RED_05_ON_15_OFF",
	"LED_RED_025_ON_075_OFF",
	"LED_RED_025_ON_175_OFF",
	"LED_RED_005_ON_195_OFF",
	"LED_RED_05_ON_05_OFF",
	"LED_RED_05_OFF_05_ON",
	"LED_RED_0125_ON_05_OFF_0075_ON_15_OFF",
	"LED_RED_0125_ON_2075_OFF",
	"LED_RED_0625_OFF_0075_ON_15_OFF",

	"LED_RED_01_ON_19_OFF_SKIP",
	"LED_RED_02_ON_18_OFF_SKIP",
	"LED_RED_05_ON_15_OFF_SKIP",
	"LED_RED_025_ON_075_OFF_SKIP",
	"LED_RED_025_ON_175_OFF_SKIP",
	"LED_RED_005_ON_195_OFF_SKIP",
	"LED_RED_05_ON_05_OFF_SKIP",
	"LED_RED_05_OFF_05_ON_SKIP",
	"LED_RED_0125_ON_05_OFF_0075_ON_15_OFF_SKIP",
	"LED_RED_0125_ON_2075_OFF_SKIP",
	"LED_RED_0625_OFF_0075_ON_15_OFF_SKIP",

	"LED_GREEN_01_ON_19_OFF",
	"LED_GREEN_02_ON_18_OFF",
	"LED_GREEN_05_ON_15_OFF",
	"LED_GREEN_025_ON_075_OFF",
	"LED_GREEN_025_ON_175_OFF",
	"LED_GREEN_005_ON_195_OFF",
	"LED_GREEN_05_ON_05_OFF",
	"LED_GREEN_05_OFF_05_ON",
	"LED_GREEN_0125_ON_05_OFF_0075_ON_15_OFF",
	"LED_GREEN_0125_ON_2075_OFF",
	"LED_GREEN_0625_OFF_0075_ON_15_OFF",

	"LED_GREEN_01_ON_19_OFF_SKIP",
	"LED_GREEN_02_ON_18_OFF_SKIP",
	"LED_GREEN_05_ON_15_OFF_SKIP",
	"LED_GREEN_025_ON_075_OFF_SKIP",
	"LED_GREEN_025_ON_175_OFF_SKIP",
	"LED_GREEN_005_ON_195_OFF_SKIP",
	"LED_GREEN_05_ON_05_OFF_SKIP",
	"LED_GREEN_05_OFF_05_ON_SKIP",
	"LED_GREEN_0125_ON_05_OFF_0075_ON_15_OFF_SKIP",
	"LED_GREEN_0125_ON_2075_OFF_SKIP",
	"LED_GREEN_0625_OFF_0075_ON_15_OFF_SKIP",

	"LED_BLUE_01_ON_19_OFF",
	"LED_BLUE_02_ON_18_OFF",
	"LED_BLUE_05_ON_15_OFF",
	"LED_BLUE_025_ON_075_OFF",
	"LED_BLUE_025_ON_175_OFF",
	"LED_BLUE_005_ON_195_OFF",
	"LED_BLUE_05_ON_05_OFF",
	"LED_BLUE_05_OFF_05_ON",
	"LED_BLUE_0125_ON_05_OFF_0075_ON_15_OFF",
	"LED_BLUE_0125_ON_2075_OFF",
	"LED_BLUE_0625_OFF_0075_ON_15_OFF",

	"LED_BLUE_01_ON_19_OFF_SKIP",
	"LED_BLUE_02_ON_18_OFF_SKIP",
	"LED_BLUE_05_ON_15_OFF_SKIP",
	"LED_BLUE_025_ON_075_OFF_SKIP",
	"LED_BLUE_025_ON_175_OFF_SKIP",
	"LED_BLUE_005_ON_195_OFF_SKIP",
	"LED_BLUE_05_ON_05_OFF_SKIP",
	"LED_BLUE_05_OFF_05_ON_SKIP",
	"LED_BLUE_0125_ON_05_OFF_0075_ON_15_OFF_SKIP",
	"LED_BLUE_0125_ON_2075_OFF_SKIP",
	"LED_BLUE_0625_OFF_0075_ON_15_OFF_SKIP",

	"LED_ORANGE_01_ON_19_OFF",
	"LED_ORANGE_02_ON_18_OFF",
	"LED_ORANGE_05_ON_15_OFF",
	"LED_ORANGE_025_ON_075_OFF",
	"LED_ORANGE_025_ON_175_OFF",
	"LED_ORANGE_005_ON_195_OFF",
	"LED_ORANGE_05_ON_05_OFF",
	"LED_ORANGE_05_OFF_05_ON",
	"LED_ORANGE_0125_ON_05_OFF_0075_ON_15_OFF",
	"LED_ORANGE_0125_ON_2075_OFF",
	"LED_ORANGE_0625_OFF_0075_ON_15_OFF",

	"LED_ORANGE_01_ON_19_OFF_SKIP",
	"LED_ORANGE_02_ON_18_OFF_SKIP",
	"LED_ORANGE_05_ON_15_OFF_SKIP",
	"LED_ORANGE_025_ON_075_OFF_SKIP",
	"LED_ORANGE_025_ON_175_OFF_SKIP",
	"LED_ORANGE_005_ON_195_OFF_SKIP",
	"LED_ORANGE_05_ON_05_OFF_SKIP",
	"LED_ORANGE_05_OFF_05_ON_SKIP",
	"LED_ORANGE_0125_ON_05_OFF_0075_ON_15_OFF_SKIP",
	"LED_ORANGE_0125_ON_2075_OFF_SKIP",
	"LED_ORANGE_0625_OFF_0075_ON_15_OFF_SKIP",

	"LED_RED_0125_ON_2075_OFF_BLUE_0625_OFF_0075_ON_15_OFF",
	"LED_GREEN_0125_ON_2075_OFF_BLUE_0625_OFF_0075_ON_15_OFF",
	"LED_ORANGE_0125_ON_2075_OFF_BLUE_0625_OFF_0075_ON_15_OFF",


	"LED_RED_0125_ON_2075_OFF_BLUE_0625_OFF_0075_ON_15_OFF_SKIP",
	"LED_GREEN_0125_ON_2075_OFF_BLUE_0625_OFF_0075_ON_15_OFF_SKIP",
	"LED_ORANGE_0125_ON_2075_OFF_BLUE_0625_OFF_0075_ON_15_OFF_SKIP",
	""
}; /* LED Name*/
#endif
/* These are the control structures for the LEDs. */
static struct led_info info1;
static struct led_reg_info led1;
static int old_state ;
static unsigned int old_tempValue;
static struct pm_dev *led_pxa_pm_dev;
/*---------------------------------------------------------------------------
DESCRIPTION: Set LED state.
INPUTS: int state LED light mode.
OUTPUTS: int 0, succesee else return error no.
IMPORTANT NOTES: 
To prevent RED to GREEN phase timing issues, the RED and Green LED bit codes should be written
simultaneously. Additionally, changing the timing of one or both drivers should be accomplished by first writing a
zero to each bit location, followed by a write to both locations with the desired bits.

---------------------------------------------------------------------------*/
int PXA_E680_LED_set(int state)
{
    unsigned char ledr_en,ledg_en,ledr_ctrl,ledg_ctrl,ledr_i,ledg_i,skip;
   unsigned int tempValue, value =0;
   
   if ( state >= LED_BOTTOM_STATE)
   {   
       PRINTK("Driver:LED The State is invalid.\n"); 
       return 1;
   }
   else if ( old_state == state)
   {  
       PRINTK("Driver:LED Same to previous State %s.\n",LED_Name[state]);
       return 0; /*Donn't need change the LED state.*/
   }
   /*First Disable LED.*/
   if(SSP_PCAP_read_data_from_PCAP(SSP_PCAP_ADJ_PERIPH_REGISTER,&tempValue)!=
        SSP_PCAP_SUCCESS)
   {   
       PRINTK("Driver:LED PCAP Read Failed.\n");
       return 1;
   }
   tempValue &= (~SSP_PCAP_LED_MASK);
   if(SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_PERIPH_REGISTER,tempValue)!=SSP_PCAP_SUCCESS)
   {
       PRINTK("Driver:LED PCAP Write Failed (Clear Data).\n");
       return 1;
   }

   /*Set GPIO as general I/O and as output*/
   set_GPIO_mode(IND_CNTL_R_BUL | GPIO_OUT);
   set_GPIO_mode(IND_CNTL_G_BUL | GPIO_OUT);
 
   if (led_register_value[state].ind_GPIO_red && led_register_value[state].ind_GPIO_green)
   {
       /*Disable Red & Green signal*/
       set_GPIO(IND_CNTL_R_BUL); /*IND_CNTL_R_BUL Low active*/
       PGSR(IND_CNTL_R_BUL) = PGSR(IND_CNTL_R_BUL) | GPIO_bit(IND_CNTL_R_BUL);
       
       clr_GPIO(IND_CNTL_G_BUL); /*IND_CNTL_G_BUL High active*/
       PGSR(IND_CNTL_G_BUL) = PGSR(IND_CNTL_G_BUL) & (~GPIO_bit(IND_CNTL_G_BUL));
       
       PRINTK("GPIO Green Disable, Red Disable!\n");      
   }else if ( (!led_register_value[state].ind_GPIO_red) &&
              led_register_value[state].ind_GPIO_green)
   {
       /*Green Disable, Red Enable*/
       clr_GPIO(IND_CNTL_R_BUL);
       PGSR(IND_CNTL_R_BUL) = PGSR(IND_CNTL_R_BUL) & (~GPIO_bit(IND_CNTL_R_BUL));
       
       clr_GPIO(IND_CNTL_G_BUL);
       PGSR(IND_CNTL_G_BUL) = PGSR(IND_CNTL_G_BUL) & (~GPIO_bit(IND_CNTL_G_BUL));

       PRINTK("GPIO Green Disable, Red Enable!\n");       
   }else if ( led_register_value[state].ind_GPIO_red && 
	      !led_register_value[state].ind_GPIO_green)
   {
       /*Red Disable, Green Enable*/
       set_GPIO(IND_CNTL_R_BUL);
       PGSR(IND_CNTL_R_BUL) = PGSR(IND_CNTL_R_BUL) | GPIO_bit(IND_CNTL_R_BUL);
       
       set_GPIO(IND_CNTL_G_BUL);
       PGSR(IND_CNTL_G_BUL) = PGSR(IND_CNTL_G_BUL) | GPIO_bit(IND_CNTL_G_BUL);
       PRINTK("GPIO Red Disable, Green Enable");
   }else 
   {
      /*Red & Green enable*/
      clr_GPIO(IND_CNTL_R_BUL);
      PGSR(IND_CNTL_R_BUL) = PGSR(IND_CNTL_R_BUL) & (~GPIO_bit(IND_CNTL_R_BUL));
       
      set_GPIO(IND_CNTL_G_BUL);
      PGSR(IND_CNTL_G_BUL) = PGSR(IND_CNTL_G_BUL) | GPIO_bit(IND_CNTL_G_BUL);
      PRINTK("GPIO Red & Green enable!\n");     
   }
   PRINTK("Driver:--IND_CNTL_G_BUL:%x IND_CNTL_R_BUL:%x\n",GPIO_is_high(IND_CNTL_G_BUL),
		                                         GPIO_is_high(IND_CNTL_R_BUL));
   /* Write PCAP Peripheral Control Register*/  
   ledr_en   = led_register_value[state].pcap_LEDR_en & 0x1;
   ledg_en   = led_register_value[state].pcap_LEDG_en & 0x1;
   ledr_ctrl = led_register_value[state].pcap_LEDR_CTRL & 0xf;
   ledg_ctrl = led_register_value[state].pcap_LEDG_CTRL & 0xf;
   ledr_i    = led_register_value[state].pcap_LEDR_I & 0x3; 
   ledg_i    = led_register_value[state].pcap_LEDG_I & 0x3;
   skip      = led_register_value[state].pcap_SKIP_on & 0x1;
   
   value = ( ledr_en | (ledg_en <<1) | (ledr_ctrl <<2) | (ledg_ctrl <<6) | 
             (ledr_i << 10) | (ledg_i <<12) | (skip <<14) ) & 0x7fff;
   tempValue |= (value <<SSP_PCAP_LED_SHIFT);
   if(SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_PERIPH_REGISTER,tempValue)==SSP_PCAP_SUCCESS)
   {
      PRINTK("Driver:LED write Value to PCAP:0x%x :0x%x\n",tempValue,value);
      PRINTK("Driver:LED State To: %s\n",LED_Name[state]);
      old_tempValue = tempValue;
      old_state = state;
      return 0;
   }else 
   {   
      PRINTK("Driver:LED: Changed to State %s Failed.\n",LED_Name[state]); 
      return 1;
   }
}


/*---------------------------------------------------------------------------
DESCRIPTION: reptore PGSR.
INPUTS: old_state.
OUTPUTS: int 0, succesee else return error no.
IMPORTANT NOTES: 

---------------------------------------------------------------------------*/
int PXA_E680_LED_retore_PGSR(struct pm_dev *dev, pm_request_t rqst, void *data)
{
    unsigned int tempValue;
    

    switch (rqst)
    {
        case PM_RESUME:
	{
            /*First Disable LED.*/
            if(SSP_PCAP_read_data_from_PCAP(SSP_PCAP_ADJ_PERIPH_REGISTER,&tempValue)!=
               SSP_PCAP_SUCCESS)
            {   
                PRINTK("Driver:LED PCAP Read Failed.\n");
                return 1;
            }
            tempValue &= (~SSP_PCAP_LED_MASK);
            if(SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_PERIPH_REGISTER,tempValue)!=SSP_PCAP_SUCCESS)
            {
                PRINTK("Driver:LED PCAP Write Failed (Clear Data).\n");
                return 1;
            }
    
            if (led_register_value[old_state].ind_GPIO_red && led_register_value[old_state].ind_GPIO_green)
            {
                /*Disable Red & Green signal*/
                set_GPIO(IND_CNTL_R_BUL); /*IND_CNTL_R_BUL Low active*/
                PGSR(IND_CNTL_R_BUL) = PGSR(IND_CNTL_R_BUL) | GPIO_bit(IND_CNTL_R_BUL);
       
                clr_GPIO(IND_CNTL_G_BUL); /*IND_CNTL_G_BUL High active*/
                PGSR(IND_CNTL_G_BUL) = PGSR(IND_CNTL_G_BUL) & (~GPIO_bit(IND_CNTL_G_BUL));
       
                PRINTK("RESTORE____GPIO Green Disable, Red Disable!\n");      
            }else if ( (!led_register_value[old_state].ind_GPIO_red) &&
                       led_register_value[old_state].ind_GPIO_green)
            {
               /*Green Disable, Red Enable*/
               clr_GPIO(IND_CNTL_R_BUL);
               PGSR(IND_CNTL_R_BUL) = PGSR(IND_CNTL_R_BUL) & (~GPIO_bit(IND_CNTL_R_BUL));
       
               clr_GPIO(IND_CNTL_G_BUL);
               PGSR(IND_CNTL_G_BUL) = PGSR(IND_CNTL_G_BUL) & (~GPIO_bit(IND_CNTL_G_BUL));

               PRINTK("RESTORE____GPIO Green Disable, Red Enable!\n");       
            }else if ( led_register_value[old_state].ind_GPIO_red && 
	              !led_register_value[old_state].ind_GPIO_green)
            {
               /*Red Disable, Green Enable*/
              set_GPIO(IND_CNTL_R_BUL);
               PGSR(IND_CNTL_R_BUL) = PGSR(IND_CNTL_R_BUL) | GPIO_bit(IND_CNTL_R_BUL);
       
               set_GPIO(IND_CNTL_G_BUL);
               PGSR(IND_CNTL_G_BUL) = PGSR(IND_CNTL_G_BUL) | GPIO_bit(IND_CNTL_G_BUL);
               PRINTK("RESTORE____GPIO Red Disable, Green Enable");
           }else 
           {
              /*Red & Green enable*/
              clr_GPIO(IND_CNTL_R_BUL);
              PGSR(IND_CNTL_R_BUL) = PGSR(IND_CNTL_R_BUL) & (~GPIO_bit(IND_CNTL_R_BUL));
       
              set_GPIO(IND_CNTL_G_BUL);
              PGSR(IND_CNTL_G_BUL) = PGSR(IND_CNTL_G_BUL) | GPIO_bit(IND_CNTL_G_BUL);
              PRINTK("RESTORE____GPIO Red & Green enable!\n");     
           }
           PRINTK("Driver:--IND_CNTL_G_BUL:%x IND_CNTL_R_BUL:%x\n",GPIO_is_high(IND_CNTL_G_BUL),
		                                         GPIO_is_high(IND_CNTL_R_BUL));

           if(SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_PERIPH_REGISTER,old_tempValue)==SSP_PCAP_SUCCESS)
           {
               PRINTK("Driver:LED write Value to PCAP:0x%x\n",old_tempValue);
               PRINTK("Driver:LED State To: %s\n",LED_Name[old_state]);
               return 0;
           }else 
           {   
               PRINTK("Driver:LED: Changed to State %s Failed.\n",LED_Name[old_state]); 
               return 1;
           }

	   break;
	}//end of case RESUME
    }//end of switch	
    return 0;
}//end of function

	  
/* Handle an actual LED set*/
int handle_led_op(struct led_reg_info *info,
		  struct led_op       *op)
{
    switch (op->op)
    {
        case SET_LED:
	   if ( PXA_E680_LED_set(op->op_info.bicolor.color) == 0)
	       return 0;
	   else 
	       return -EINVAL;
        default:
	   PRINTK("LED:Error CMD.\n");
           return -EINVAL;
    }
}

/**
 *	PXA_E680_LED_exit:
 *
 *	Remove the LEDs from the LED driver
 */
 
static void __exit PXA_E680_LED_exit(void)
{
    pm_unregister(led_pxa_pm_dev);	
    unregister_led_info(&led1);
}

/**
 *      PXA_E680_LED_init
 *
 *	Register the LEDs with the LED driver.
 */
 
static int __init PXA_E680_LED_init(void)
{
   info1.type = LED_TYPE_BICOLOR;
   strcpy(info1.name, LED_E680_NAME);	
   led1.info = &info1;
   led1.data = (void *) 0;
   led1.handle_led_op = handle_led_op;
   register_led_info(&led1);
  
   old_state = LED_OFF;
   old_tempValue = 0;
   led_pxa_pm_dev = pm_register(PM_UNKNOWN_DEV, PM_SYS_UNKNOWN, PXA_E680_LED_retore_PGSR);
 
   /*Set GPIO as general IO and as Output*/
   set_GPIO_mode(IND_CNTL_R_BUL | GPIO_OUT);
   set_GPIO_mode(IND_CNTL_G_BUL | GPIO_OUT);
   
   set_GPIO(IND_CNTL_R_BUL);
   PGSR(IND_CNTL_R_BUL) = PGSR(IND_CNTL_R_BUL) | GPIO_bit(IND_CNTL_R_BUL);

   clr_GPIO(IND_CNTL_G_BUL);
   PGSR(IND_CNTL_G_BUL) = PGSR(IND_CNTL_G_BUL) & (~GPIO_bit(IND_CNTL_G_BUL));
   PRINTK("PCAP2 LED driver: v%s E680\n", VERSION);
   return 0;
}

module_init(PXA_E680_LED_init);
module_exit(PXA_E680_LED_exit);
MODULE_LICENSE("GPL");
