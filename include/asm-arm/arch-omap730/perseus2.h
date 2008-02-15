/*
 *  linux/include/asm-arm/arch-omap730/perseus2.h
 *
 *  Copyright (c) 2004, MPC-Data Limited (http://www.mpc-data.co.uk)
 *    Dave Peverley <dpeverley at mpc-data.co.uk>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __INCLUDED_PERSEUS2_H_
#define __INCLUDED_PERSEUS2_H_

#include <asm/arch/hardware.h>


/* ---------------------------------------------------------------------------
 *  Perseus2 Board Definitions
 * ---------------------------------------------------------------------------
 */


/* 
 * Board specific IRQ lines.
 */

#define INT_P2_MMC_CD              INT_GPIO_8

#define INT_P2_ETHR                INT_MPU_EXT_NIRQ


/* 
 * GPIO used for various board functions & safe defaults. Note :GPIO_144 is 
 * used on the board for test purposes apparently so we don't need to 
 * support it.
 */

#define GPIO_SETUP(gpio_name)                                  \
            GPIO_SET_DIR(gpio_name##_NUM, gpio_name##_DIR);    \
            GPIO_SET_STATE(gpio_name##_NUM, gpio_name##_DEF);   

#define GPIO_BACKLIGHT_ENABLE_NUM  134
#define GPIO_BACKLIGHT_ENABLE_DIR  GPIO_DIR_OUTPUT
#define GPIO_BACKLIGHT_ENABLE_DEF  0

#define GPIO_MMC_WP_NUM            67 
#define GPIO_MMC_WP_DIR            GPIO_DIR_OUTPUT
#define GPIO_MMC_WP_DEF            0

#define GPIO_MMC_CD_NUM            8
#define GPIO_MMC_CD_DIR            GPIO_DIR_INPUT
#define GPIO_MMC_CD_DEF            0                /* Input so don't care */

#define GPIO_LCD_USB_BT_RESET_NUM  19
#define GPIO_LCD_USB_BT_RESET_DIR  GPIO_DIR_OUTPUT
#define GPIO_LCD_USB_BT_RESET_DEF  0

#define GPIO_NAND_RnB_NUM          74
#define GPIO_NAND_RnB_DIR          GPIO_DIR_INPUT
#define GPIO_NAND_RnB_DEF          0                /* Input so don't care */

#define GPIO_OTG_XCVR_INT_NUM      145
#define GPIO_OTG_XCVR_INT_DIR      GPIO_DIR_INPUT
#define GPIO_OTG_XCVR_INT_DEF      0                /* Input so don't care */

#define GPIO_HEADSET_DETECT_NUM    36
#define GPIO_HEADSET_DETECT_DIR    GPIO_DIR_INPUT
#define GPIO_HEADSET_DETECT_DEF    0                /* Input so don't care */

#define GPIO_CHARGE_ENABLE_NUM     73
#define GPIO_CHARGE_ENABLE_DIR     GPIO_DIR_OUTPUT
#define GPIO_CHARGE_ENABLE_DEF     0

#define GPIO_USB_CHARGE_CURR_NUM   35
#define GPIO_USB_CHARGE_CURR_DIR   GPIO_DIR_OUTPUT
#define GPIO_USB_CHARGE_CURR_DEF   0

#define GPIO_CHARGE_STATE_LED_NUM  72
#define GPIO_CHARGE_STATE_LED_DIR  GPIO_DIR_OUTPUT
#define GPIO_CHARGE_STATE_LED_DEF  0

#define GPIO_ENABLE_MMC_VDD_NUM    144
#define GPIO_ENABLE_MMC_VDD_DIR    GPIO_DIR_OUTPUT
#define GPIO_ENABLE_MMC_VDD_DEF    0


/* Backlight control */

typedef enum {
  BACKLIGHT_OFF = 0,
  BACKLIGHT_ON 
} p2_backlight_state_t;

static inline void set_p2_backlight_state(p2_backlight_state_t state)
{
	if (state == BACKLIGHT_OFF)
		GPIO_SET_STATE(GPIO_BACKLIGHT_ENABLE_NUM, 0);
	else
		GPIO_SET_STATE(GPIO_BACKLIGHT_ENABLE_NUM, 1);		
}
                                                                                                                             
static inline p2_backlight_state_t get_p2_backlight_state(void)
{
        if (GPIO_GET_OUTPUT_STATE(GPIO_BACKLIGHT_ENABLE_NUM) == 0)
		return BACKLIGHT_OFF;
	else
		return BACKLIGHT_ON;
}

/* MMC WP control */

typedef enum {
  MMC_WP_DISABLED = 0,
  MMC_WP_ENABLED
} p2_mmc_wp_state_t;

static inline void set_p2_mmc_wp_state(p2_mmc_wp_state_t state)
{
	if (state == MMC_WP_DISABLED)
		GPIO_SET_STATE(GPIO_MMC_WP_NUM, 0);
	else
		GPIO_SET_STATE(GPIO_MMC_WP_NUM, 1);
}
 
static inline p2_mmc_wp_state_t get_p2_mmc_wp_state(void)
{
	if (GPIO_GET_OUTPUT_STATE(GPIO_MMC_WP_NUM) == 0)
		return MMC_WP_DISABLED;
	else
		return MMC_WP_ENABLED;
}

/* MMC CD control */

static inline int get_p2_mmc_cd_state(void)
{
	return GPIO_GET_INPUT_STATE(GPIO_MMC_CD_NUM);
}

/* LCD/USB/BT Reset */

typedef enum {
  LCD_USB_BT_RESET_HOLD = 0,
  LCD_USB_BT_RESET_RELEASE
} p2_lcd_reset_state_t;

static inline void set_p2_reset_lcd_usb_bt(p2_lcd_reset_state_t state)
{
	if (state == LCD_USB_BT_RESET_HOLD)
		GPIO_SET_STATE(GPIO_LCD_USB_BT_RESET_NUM, 0);
	else 
		GPIO_SET_STATE(GPIO_LCD_USB_BT_RESET_NUM, 1);
}

/* Headset Detect */

typedef enum {
  HEADSET_UNCONNECTED = 0,
  HEADSET_CONNECTED
} p2_headset_state_t;

static inline p2_headset_state_t get_p2_headset_state(void)
{
	if (GPIO_GET_INPUT_STATE(GPIO_HEADSET_DETECT_NUM) == 0)
		return HEADSET_UNCONNECTED;
	else
		return HEADSET_CONNECTED;
}

/* Battery charger enable */

typedef enum {
  CHARGE_ENABLE = 0,
  CHARGE_DISABLE
} p2_charger_state_t;

static inline void set_p2_charge_enable(p2_charger_state_t state)
{
	if (state == CHARGE_ENABLE)
		GPIO_SET_STATE(GPIO_CHARGE_ENABLE_NUM, 0);
	else 
		GPIO_SET_STATE(GPIO_CHARGE_ENABLE_NUM, 1);
}

/* USB Charge Current Set */

typedef enum {
  USB_CHARGE_100MA = 0,
  USB_CHARGE_500MA
} p2_usb_charge_curr_t;

static inline void set_p2_usb_charge_curr(p2_usb_charge_curr_t state)
{
	if (state == USB_CHARGE_100MA)
		GPIO_SET_STATE(GPIO_USB_CHARGE_CURR_NUM, 0);
	else 
		GPIO_SET_STATE(GPIO_USB_CHARGE_CURR_NUM, 1);
}

/* Charge state LED */

typedef enum {
  CHARGE_STATE_LED_OFF = 0,
  CHARGE_STATE_LED_ON
} p2_charge_led_state_t;

static inline void set_p2_charge_state_led(p2_charge_led_state_t state)
{
	if (state == CHARGE_STATE_LED_OFF)
		GPIO_SET_STATE(GPIO_CHARGE_STATE_LED_NUM, 0);
	else 
		GPIO_SET_STATE(GPIO_CHARGE_STATE_LED_NUM, 1);
}

/* MMC VDD */

typedef enum {
  V_SDMMC_DISABLED = 0,
  V_SDMMC_ENABLED 
} p2_v_sdmmc_state_t;

static inline void set_p2_v_sdmmc_state(p2_v_sdmmc_state_t state)
{
	if (state == V_SDMMC_DISABLED)
		GPIO_SET_STATE(GPIO_ENABLE_MMC_VDD_NUM, 0);
	else
		GPIO_SET_STATE(GPIO_ENABLE_MMC_VDD_NUM, 1);
}
                                                                                                                             
static inline p2_v_sdmmc_state_t get_p2_v_sdmmc_state(void)
{
        if (GPIO_GET_OUTPUT_STATE(GPIO_ENABLE_MMC_VDD_NUM) == 0)
		return V_SDMMC_DISABLED;
	else
		return V_SDMMC_ENABLED;
}

/* NAND RnB (Ready/Busy) input */

typedef enum {
  NAND_BUSY = 0,
  NAND_READY
} p2_nand_rnb_state_t;

static inline p2_nand_rnb_state_t get_p2_nand_rnb_state(void)
{
	if (GPIO_GET_INPUT_STATE(GPIO_NAND_RnB_NUM) == 0)
		return NAND_BUSY;
	else
		return NAND_READY;
}



/* ---------------------------------------------------------------------------
 *  Perseus2 Debug Board Definitions
 * ---------------------------------------------------------------------------
 */

/* 
 * IO Map entry for the FPGA registers and the ETHR registers 
 */

#define PERSEUS2_FPGA_BASE               0xE8000000                 // VA
#define PERSEUS2_FPGA_SIZE               SZ_4K                      // SIZE
#define PERSEUS2_FPGA_START              0x04000000                 // PA   

/* Offsets for register on the FPGA */

#define PERSEUS2_FPGA_ETHR_BASE          PERSEUS2_FPGA_BASE
#define PERSEUS2_FPGA_FPGA_REV           (PERSEUS2_FPGA_BASE + 0x10) /* FPGA Revision */ 
#define PERSEUS2_FPGA_BOARD_REV          (PERSEUS2_FPGA_BASE + 0x12) /* Board Revision */
#define PERSEUS2_FPGA_GPIO               (PERSEUS2_FPGA_BASE + 0x14) /* GPIO outputs */
#define PERSEUS2_FPGA_LEDS               (PERSEUS2_FPGA_BASE + 0x16) /* LEDs outputs */
#define PERSEUS2_FPGA_MISC_INPUTS        (PERSEUS2_FPGA_BASE + 0x18) /* Misc inputs */
#define PERSEUS2_FPGA_LAN_STATUS         (PERSEUS2_FPGA_BASE + 0x1A) /* LAN Status line */
#define PERSEUS2_FPGA_LAN_RESET          (PERSEUS2_FPGA_BASE + 0x1C) /* LAN Reset line */



#endif  /* !  __INCLUDED_PERSEUS2_H_ */
