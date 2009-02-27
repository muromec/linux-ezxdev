/*
 * linux/drivers/video/ezx_lcd/pxafb.c
 * Intel Bulverde/PXA250/210 LCD Controller Frame Buffer Driver
 * 
 * Copyright (C) 2002-2005 - Motorola
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
 * 2005/12/26  w20535@motorola.com
	-  Implemented Errata 62 and ipm related tuning.
 * 2004/06/28  Susan
 *      - add double buffering for LCD controller
 * 2002/06/30: lilij
 *      - Ported to EZX LCD
 *
 * 2003/05/19: zxf
 *	- Add timer to turn on backlight until lcd controller is ready
 * 2003/10/17: sdavis3
 * 	- Added 18bpp/19bpp packed support
 * 2004/06/28  Susan
 *      - add double buffering for LCD controller
 * 2004/11/25  Susan
 *      - Restruct LCD driver to be compliant with Intel 24to18 bpp conversion and Intel CLI smart panel patches
 * 2005/12/12  Wang limei
 *      - Add CLI paritial mode display feature;
 *	 - Add workaround for LCD display issue caused by destroied FB, which is due to intel security function; 	
 *	 - Support setting backlight dutycycle range,add brightness and dutycycle convertion routines; 
 *	 - Overlay2 related update: add interface to support blending constant setting;
 *						    keep cpu freq at 208/312M  when ovl2 is enabled to support camera;
 *	 - Support Logo is showed by MBM instead of kernel, it is configurable;
 * 2005/12/12  Wang Jordan
 *	 - IPM releated code is polished after 312M freq point is added;
*/

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/init.h>
#include <linux/notifier.h>
#include <linux/cpufreq.h>

#include <linux/power_ic.h>
#include <linux/apm_bios.h>
#include <linux/device.h>

#include <linux/lights_funlights.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>

#include <video/fbcon.h>
#include <video/fbcon-mfb.h>
#include <video/fbcon-cfb4.h>
#include <video/fbcon-cfb8.h>
#include <video/fbcon-cfb16.h>
#include <video/fbcon-cfb24.h>
#include <video/lcdctrl.h> /* brightness, contrast, etc. control */

/*
 * debugging?
 */
#define CONFIG_CURSOR_CHANNEL 0 //  cursor is not used
/*
 * Complain if VAR is out of range.
 */

//#define DEBUG_VAR 1
//#define DEBUG_CLI 1

#undef ASSABET_PAL_VIDEO

#include "pxafb.h"
#ifdef CONFIG_ARCH_EZX_BARBADOS
#include "lcd_barbados.h"
#endif
#ifdef CONFIG_ARCH_EZX_HAINAN
#include "lcd_hainan.h"
#endif
#ifdef CONFIG_ARCH_EZX_SUMATRA
#include "lcd_sumatra.h"
#endif
#ifdef CONFIG_ARCH_EZX_MARTINIQUE
#include "lcd_barbados.h"
#endif



void (*pxafb_blank_helper)(int blank);
EXPORT_SYMBOL(pxafb_blank_helper);

//extern int handshake_pass();

#define BKLIGHTON  1
#define BKLIGHTOFF 0

#define BK_TIMEOUT	(200*HZ/1000)
#define MDREFR_DRI                      (0xFFF)

u8 pxafb_ezx_BackLight_Brightness_Intensity(unsigned long dutyCycle, struct fb_info *info);

/* Adjust brightness by change PWM_DUTY0 according to the input parameter dutyCycle.*/

/* Not local to be accessible from __exit fn, and LDM funcs */
//static struct pxafb_info *pxafbi = 0;
struct pxafb_info *pxafbi = 0;

/* For CLI panel porting -- Susan*/
struct pxafb_info *pxafb_main = NULL;
struct pxafb_info *pxafb_smart = NULL;
struct global_state pxafb_global_state;

#define BVD_SECURE_LCD_WORKAROUND
#ifdef BVD_SECURE_LCD_WORKAROUND
#define BVD_DESTROIED_START_ADDR   		io_p2v(0x5c030000)	/*start addr of ISRAM bank 3*/
#define BVD_SECURE_DESTROIED_LCD_SIZE 	(0xa000)	/*size =40k; end addr = 0x5c03,a000; start addr =0x5c03,0000*/
unsigned char  basefb_backup[BVD_SECURE_DESTROIED_LCD_SIZE]; /*used to save the last 64k base fb destroied by secure code when resuming */
#endif

#ifdef CONFIG_LOGO_SHOWED_BY_MBM
/*  This variable is used to decide if the kernel need to initialize LCD related HW.
*    The init process covers: GPIO setup, enable LCD module clock,  init LCD controller REG,
*    reset backlight enable pin and turnon backlight. During powering on, kernel don't need to do it,
*    because it has been done by MBM, otherwise, showing seamless logo can't be attained.
*/
u32 powering_up = 1;
#endif


static void set_ctrlr_state(struct pxafb_info *fbi, u_int state);
static inline int get_pcd(unsigned int pixclock);

#ifdef CONFIG_CPU_BULVERDE

#ifdef FCS_EOF
wait_queue_head_t  lcd_eof_fcs_wait_q;
volatile int fcs_in_eof = 0;
struct clk_regs fcs_regs;
static int sys_bus_change = 0;

#ifdef WAIT_MORE_EOF
#define FRAME_TIME	57215		/* one frame time measured: 57215 OSCR ticks */
#define DELAY_MAX	1000		/* the max delay that not cause underrun: 1000 OSCR */
#define DELAY_MIN	-1000
#endif

#endif

#ifdef CONFIG_IPM_DEEPIDLE
extern void deepidle_disable(void);
extern void deepidle_enable(void);
#endif
extern int ovl2_ipm_hook(void);
extern void ovl2_ipm_unhook(void);


/* extern function prototype declarations */
int device_powersuspend(struct device *dev);

#ifdef CONFIG_PXAFB_CLI
/* static function prototype declarations */
static void pxafb_smart_load_cmd(struct pxafb_info *fbi, unsigned short *cmd_buf, int cmd_num);
static int pxafb_smart_send_cmd(struct pxafb_info *fbi);
#endif

/* Framebuffer in SRAM support */
static unsigned long sram_start, sram_size = 0UL;

/* Overlays and hw cursor support prototypes */
static int overlay2fb_get_var(struct fb_var_screeninfo *var, int con, struct fb_info *info);
static int overlay2fb_pure_set_var(struct fb_var_screeninfo *var, int con, struct fb_info *info);
static int overlay2fb_pan_display(struct fb_var_screeninfo *var, int con, struct fb_info *info);

static int overlay1fb_enable(struct fb_info *info);
static int overlay2fb_enable(struct fb_info *info);
static int overlay1fb_disable(struct fb_info *info);
static int overlay2fb_disable(struct fb_info *info);
static void overlay1fb_blank(int blank, struct fb_info *info);
static void overlay2fb_blank(int blank, struct fb_info *info);

#if CONFIG_CURSOR_CHANNEL
static int cursorfb_enable(struct fb_info *info);
static int cursorfb_disable(struct fb_info *info);
static void cursorfb_blank(int blank, struct fb_info *info);
#endif

#define CLEAR_LCD_INTR(reg, intr) do {  \
	reg = (intr);			\
}while(0)

#define WAIT_FOR_LCD_INTR(reg,intr,timeout) ({	\
	int __done =0;				\
	int __t = timeout;			\
	while (__t) {				\
		__done = (reg) & (intr);	\
		if (__done) break;		\
		mdelay(1);			\
		__t--;				\
	}					\
	__done;					\
})

#define DISABLE_OVERLAYS(fbi) do { 			\
	if (fbi->overlay1fb->state == C_ENABLE) {	\
		overlay1fb_disable((struct fb_info*)fbi->overlay1fb); \
	}						\
	if (fbi->overlay2fb->state == C_ENABLE) {	\
		overlay2fb_disable((struct fb_info*)fbi->overlay2fb); \
	}						\
	if (fbi->cursorfb->state == C_ENABLE) {		\
		cursorfb_disable((struct fb_info*)fbi->cursorfb); \
	}						\
} while(0)

#define ENABLE_OVERLAYS(fbi) do {				\
	if (fbi->overlay1fb->state == C_DISABLE) {		\
		overlay1fb_enable((struct fb_info*)fbi->overlay1fb); \
	}							\
	if (fbi->overlay2fb->state == C_DISABLE) { 		\
		overlay2fb_enable((struct fb_info*)fbi->overlay2fb); \
	}							\
	if (fbi->cursorfb->state == C_DISABLE) {			\
		cursorfb_enable((struct fb_info*)fbi->cursorfb); \
	}							\
} while(0)


#endif /* CONFIG_CPU_BULVERDE */

/*
 * IMHO this looks wrong.  In 8BPP, length should be 8.
 */
static struct pxafb_rgb rgb_8 = {
	red:	{ offset: 0,  length: 4, },
	green:	{ offset: 0,  length: 4, },
	blue:	{ offset: 0,  length: 4, },
	transp:	{ offset: 0,  length: 0, },
};

static struct pxafb_rgb def_rgb_16 = {
	red:	{ offset: 11, length: 5, },
	green:	{ offset: 5,  length: 6, },
	blue:	{ offset: 0,  length: 5, },
	transp:	{ offset: 0,  length: 0, },
};

#ifdef CONFIG_CPU_BULVERDE
/* 16bpp, format 4 */
static struct pxafb_rgb def_rgbt_16 = {
	red:	{ offset: 10, length: 5, },
	green:	{ offset: 5,  length: 5, },
	blue:	{ offset: 0,  length: 5, },
	transp:	{ offset: 15,  length: 1, },
};
 
static struct pxafb_rgb def_rgbt_18 = {
	red:	{ offset: 12, length: 6, },
	green:	{ offset: 6,  length: 6, },
	blue:	{ offset: 0,  length: 6, },
	transp:	{ offset: 0,  length: 0, },
};

/* 18bpp packed */
static struct pxafb_rgb def_rgb_18 = {
	red:    { offset: 12, length: 6, },
	green:  { offset: 6,  length: 6, },
	blue:   { offset: 0,  length: 6, },
	transp: { offset: 0,  length: 0, },
};

/* 19bpp packed */
static struct pxafb_rgb def_rgbt_19 = {
	red:    { offset: 12, length: 6, },
	green:  { offset: 6,  length: 6, },
	blue:   { offset: 0,  length: 6, },
	transp: { offset: 18, length: 1, },
};

static struct pxafb_rgb  def_rgbt_24 = {
	red:	{ offset: 16, length: 8, },
	green:	{ offset: 8,  length: 8, },
	blue:	{ offset: 0,  length: 8, },
	transp:	{ offset: 0,  length: 0, },
};

static struct pxafb_rgb def_rgbt_25 = {
 	red:	{ offset: 16, length: 8, },
 	green:	{ offset: 8,  length: 8, },
 	blue:	{ offset: 0,  length: 8, },
	transp:	{ offset: 24, length: 1, },
};

#endif

static struct pxafb_mach_info pxa_fb_main_info __initdata = {
	pixclock:	LCD_PIXCLOCK,	/* clock period in ps */
	bpp:		LCD_BPP,  //This is the tricky 2bpp for baseframe LCD_BPP,
	xres:		LCD_XRES,
	yres:		LCD_YRES,
	hsync_len:	LCD_HORIZONTAL_SYNC_PULSE_WIDTH,
	vsync_len:	LCD_VERTICAL_SYNC_PULSE_WIDTH,
	left_margin:	LCD_BEGIN_OF_LINE_WAIT_COUNT,
	upper_margin:	LCD_BEGIN_FRAME_WAIT_COUNT,
	right_margin:	LCD_END_OF_LINE_WAIT_COUNT,
	lower_margin:	LCD_END_OF_FRAME_WAIT_COUNT,
	sync:		LCD_SYNC,
	lccr0:		LCD_LCCR0,
	lccr3:		LCD_LCCR3,
	lccr4:          LCD_LCCR4,
};

#ifdef CONFIG_PXAFB_CLI
//Susan -- disable it -- static struct pxafb_info * __init pxa_smartfb_init_fbinfo(struct pxafb_mach_info *, void (*mach_init)(struct pxafb_info*));
//Susan -- disable it -- static int pxa_smartfb_map_video_memory(struct pxafb_info *fbi);
//Susan -- disable it -- static void pxa_smartfb_set_ctrlr_state(struct pxafb_info *fbi, u_int state);
//Susan -- disable it -- static int pxa_smartfb_screenupdate_thread(void *arg);
//Susan -- disable it --  static struct pxafb_info *smartfbi;
//Susan -- disable it -- static DECLARE_WAIT_QUEUE_HEAD(screen_update_wait_q);
//Susan -- disable it -- static DECLARE_MUTEX(cmd_sem); /* Make each command execution automic */
static struct pxafb_mach_info pxa_fb_smart_info __initdata = {
	//Susan -- modename:       "MOT_CLI",
	pixclock:       0,
	bpp:            SMART_LCD_BPP,
	xres:           SMART_LCD_XRES,
	yres:           SMART_LCD_YRES,
	left_margin:    SMART_LCD_BEGIN_OF_LINE_WAIT_COUNT,
	right_margin:   SMART_LCD_END_OF_LINE_WAIT_COUNT,
	lccr0:          SMART_LCD_LCCR0,
	lccr1:          SMART_LCD_LCCR1,
	lccr2:          SMART_LCD_LCCR2,
	lccr3:          SMART_LCD_LCCR3,
	lccr4:          SMART_LCD_LCCR4,
	lccr5:          SMART_LCD_LCCR5,
};

#endif

static struct pxafb_mach_info * __init
pxafb_get_machine_info(struct pxafb_info *fbi)
{
	if (fbi->fb.modename[0]< PXAFB_CLI_BASE)
		return &pxa_fb_main_info;
#ifdef CONFIG_PXAFB_CLI
	if (fbi->fb.modename[0] >= PXAFB_CLI_BASE)
		return &pxa_fb_smart_info;
#endif
	return NULL; 
}

static void pxafb_blank(int blank, struct fb_info *info);
static int pxafb_suspend(struct device * dev, u32 state, u32 level);
static int pxafb_resume(struct device * dev, u32 level);
static int pxafb_scale(struct bus_op_point * op, u32 level);

#ifdef CONFIG_ARCH_MAINSTONE
#include <asm/arch/bulverde_dpm.h>

static struct constraints pxafb_constraints = {
	/* one constraint */
	count:  1,
	/* constrain the LCD clock to be > 0 */
	param: {{DPM_MD_PLL_LCD, 1, 104000},},
};  
#endif /* CONFIG_ARCH_MAINSTONE */
	  
struct device_driver pxafb_main_driver_ldm = {
	name:      	"pxafb",//"pxafb_main",
	devclass:  	NULL,
	probe:     	NULL,
	suspend:   	pxafb_suspend,
	resume:    	pxafb_resume,
	scale:	  	pxafb_scale,
	remove:    	NULL,
#ifdef CONFIG_ARCH_MAINSTONE
	constraints:	&pxafb_constraints,
#endif
};

struct device pxafb_main_device_ldm = {
	name:		"MAIN",
	bus_id:		"lcd",
	driver: 	NULL,
	power_state:	DPM_POWER_OFF,
	driver_data:  PXA_MAIN_PANEL,
};

#ifdef CONFIG_PXAFB_CLI
struct device_driver pxafb_smart_driver_ldm = {
	name:      	"pxafb1",//"pxafb_smart",
	devclass:  	NULL,
	probe:     	NULL,
	suspend:   	pxafb_suspend,
	resume:    	pxafb_resume,
	scale:	  	pxafb_scale,
	remove:    	NULL,
#ifdef CONFIG_ARCH_MAINSTONE
	constraints:	&pxafb_constraints,
#endif
};

struct device pxafb_smart_device_ldm = {
	name:		"CLI",
	bus_id:		"lcd1",
	driver: 	NULL,
	power_state:	DPM_POWER_OFF,
	driver_data:   (void *)PXA_SMART_PANEL,
};

#endif
static void pxafb_ldm_register(void)
{
	extern void pxasys_driver_register(struct device_driver *driver);
	extern void pxasys_device_register(struct device *device);
	
	pxasys_driver_register(&pxafb_main_driver_ldm);
	pxasys_device_register(&pxafb_main_device_ldm);

#ifdef CONFIG_PXAFB_CLI
	pxasys_driver_register(&pxafb_smart_driver_ldm);
	pxasys_device_register(&pxafb_smart_device_ldm);

#endif
}

static void pxafb_ldm_unregister(void)
{
	extern void pxasys_driver_unregister(struct device_driver *driver);
	extern void pxasys_device_unregister(struct device *device);
	
	pxasys_driver_unregister(&pxafb_main_driver_ldm);
	pxasys_device_unregister(&pxafb_main_device_ldm);

#ifdef CONFIG_PXAFB_CLI
	pxasys_driver_unregister(&pxafb_smart_driver_ldm);
	pxasys_device_unregister(&pxafb_smart_device_ldm);
#endif
}

void bulverde_change_freq() 
{
	unsigned long tmp, tmp_cccr;

#ifdef FCS_WITH_MEM
	if (sys_bus_change > 0) {
        	MDREFR = (fcs_regs.mdrefr & (~MDREFR_DRI)) | 0x2;
		tmp = MDREFR;
	}
#endif
       	CCCR = fcs_regs.cccr;
        tmp_cccr = CCCR;
	tmp = MDREFR;
                                                                                                                             
       	asm("mcr\tp14, 0, %0, c6, c0, 0" ::"r"(fcs_regs.clkcfg));
	MDREFR = tmp & (~MDREFR_RESERVED);
        tmp = MDREFR;
                                                                                                                             
       	do {
        	asm("mrc\tp14, 0, %0, c6, c0, 0" :"=r"(tmp));
       	} while ((tmp & 0xf) != fcs_regs.clkcfg);

#ifdef FCS_WITH_MEM
	/*   Errata 62 detail info please refer to 28007107 spec update July 2005(3).pdf
         *
         *  1) Disable interrupts.
         *  2) Leave the KxDB2 bits at their current state, but set MDREFR[DRI] to 0xFFF.
         *  3) Read MDREFR one time.
         *  4) Wait 1.6167 us. This can be accomplished by reading the OSCR0 register until its
         *     3.25MHz counter has incremented 7 times.
         *  5) Clear K1DB2 and/or K2DB2 bits, and set MDREFR[DRI] to the proper value at the
         *     same time.
         *  6) Read MDREFR one time.
         *  7) Enable interrupts.
         */
	if (sys_bus_change < 0) {
       		MDREFR |= (0xfff & ~MDREFR_RESERVED);
               	tmp=MDREFR;

		tmp = OSCR;
               	while (OSCR - tmp <= 7);
      	} 
         /*this branch ensure below requirement from Intel Spec.
          *if the clock frequency is changed, this register must be rewritten, even if the
          *value has not changed. This causes a refresh and resets the refresh
          *counter to the refresh interval.
          */
                                                                                                                             
	MDREFR = fcs_regs.mdrefr;
       	tmp=MDREFR;
#endif
	pxafb_scale(NULL, 0);
}

#ifdef FCS_EOF
int change_pclk(struct clk_regs *regs, int speedup)
{
        DECLARE_WAITQUEUE(wait, current);

	fcs_regs = *regs;
	sys_bus_change = speedup;	
	if (! (pxafb_global_state.main_state & PXAFB_PANEL_ON)) {
		unsigned long flags;
#ifdef CHK_IPC
		if (ipc_is_active())
			return -1;
#endif
	        save_flags_cli(flags);

		bulverde_change_freq();

	 	restore_flags(flags);

        } else {
#ifdef CONFIG_IPM_DEEPIDLE
	        deepidle_disable();
#endif
	        current->state = TASK_UNINTERRUPTIBLE;
       		add_wait_queue(&lcd_eof_fcs_wait_q, &wait);
	        fcs_in_eof = 1;

	        schedule();

	        remove_wait_queue(&lcd_eof_fcs_wait_q, &wait);
	        current->state = TASK_RUNNING;

#ifdef CONFIG_IPM_DEEPIDLE
	        deepidle_enable();
#endif
#ifdef CHK_IPC
		if (fcs_in_eof == -1)
			return -1;
#endif
	}
        return 0;
}
#else

int change_pclk(struct clk_regs *regs)
{
	struct pxafb_info *fbi = pxafbi;
	unsigned long flags;

        save_flags_cli(flags);

	fcs_regs = *regs;
	/* I don't neet to take pxafb_global_state.g_sem, because we are in disabling interrupt */
	if (pxafb_global_state.main_state & PXAFB_PANEL_ON) {
        	unsigned long start;

	        LCSR0 |= LCSR0_EOF;
		start = OSCR;
	        while((LCSR0 & LCSR0_EOF) == 0 ) {
			if (OSCR - start >3250000)
				panic("wait eof\n");
		}
	}

        CCCR = fcs_regs.cccr;
        asm("mcr\tp14, 0, %0, c6, c0, 0" ::"r"(fcs_regs.clkcfg));
	pxafb_scale(NULL, 0);
	printk("FCS, oscr=0x%x\n", OSCR);
	restore_flags(flags);
	return 0;
}

#endif

static int pxafb_scale(struct bus_op_point * op, u32 level)
{
	/* Bulverde is not using the passed-in arguments */

	struct pxafb_info *fbi = pxafb_main;
	u_int pcd;
	u_int lccr3;

	/* NOTE: the cpufreq notifier function in this file
	   dis/enables the LCD controller around updating these
	   values. How important is that for Bulverde? */
	if (!fbi)
		return -EAGAIN;  //Two panels are all disabled //
		
	pcd = get_pcd(fbi->fb.var.pixclock);
        lccr3 = fbi->reg_lccr3;
	fbi->reg_lccr3 = (fbi->reg_lccr3 & ~0xff) | LCCR3_PixClkDiv(pcd);

	if (pxafb_global_state.main_state & PXAFB_PANEL_ON)
        {
        	if (lccr3 != fbi->reg_lccr3)
	                LCCR3 = fbi->reg_lccr3;
        }

	return 0;

}


#ifdef BVD_SECURE_LCD_WORKAROUND
/* Since last 64k is used by secure function when resuming from sleep, so part of the LCD FB in ISRAM is destroied */
static void pxafb_save_basefb(void)
{
	u_char  *start_addr = (u_char *)BVD_DESTROIED_START_ADDR;		
	memcpy(basefb_backup,start_addr,BVD_SECURE_DESTROIED_LCD_SIZE);

}
static void pxafb_restore_basefb(void)
{
	u_char  *start_addr =(u_char *)BVD_DESTROIED_START_ADDR ;
	memcpy(start_addr,basefb_backup, BVD_SECURE_DESTROIED_LCD_SIZE);
}
static int pxafb_fb_callback(struct pm_dev *dev, pm_request_t rqst, void *data)
{      
	
	switch (rqst) {
       	 case PM_SUSPEND:
                /* save base fb */
		 pxafb_save_basefb();
                break;
        case PM_RESUME:
                /* restore base fb */
		 pxafb_restore_basefb();
                break;
        }
                                                                                                                             
        return 0;
}

#endif

u8 pxafb_ezx_getBacklight_status(struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	
	if (fbi == pxafb_main)
		return (pxafb_global_state.main_state & 0x1);
#ifdef CONFIG_PXAFB_CLI
	if (fbi == pxafb_smart)
		return (pxafb_global_state.smart_state & 0x1);
#endif
	return -EFAULT;
}
static unsigned long pxafb_convert_brightness_to_dutycycle(unsigned long brightness)
{
	unsigned long delta_dutycycle= pxafb_global_state.bkduty_range.max - pxafb_global_state.bkduty_range.min ;
	return (((delta_dutycycle* brightness )/(MAX_BRIGHTNESS - MIN_BRIGHTNESS)) + pxafb_global_state.bkduty_range.min);

}
static unsigned long pxafb_convert_dutycycle_to_brightness(unsigned long dutyCycle)
{
	unsigned long delta_dutycycle  = pxafb_global_state.bkduty_range.max - pxafb_global_state.bkduty_range.min;
	return  (((dutyCycle -pxafb_global_state.bkduty_range.min )* ( MAX_BRIGHTNESS - MIN_BRIGHTNESS))/delta_dutycycle);

}
unsigned long pxafb_ezx_getBacklight_brightness(struct fb_info *info)
{
        unsigned long brightness = 0;
        struct pxafb_info *fbi = (struct pxafb_info *)info;

        if (fbi == pxafb_main)
        {
        	
	        brightness = PWM_PWDUTY0;
		DPRINTK("MAIN:pxafb_ezx_getBacklight_brightness: dutycycle is %d \n", brightness);			
		brightness = pxafb_convert_dutycycle_to_brightness(brightness);	
	        DPRINTK("MAIN:pxafb_ezx_getBacklight_brightness: brightness is %d \n", brightness);
        }
#ifdef CONFIG_PXAFB_CLI
        if (fbi == pxafb_smart)
        {
       	        brightness = PWM_PWDUTY0;
 	 	DPRINTK("CLI:pxafb_ezx_getBacklight_brightness: dutycycle is %d \n", brightness);					
		brightness = pxafb_convert_dutycycle_to_brightness(brightness);	
	      	DPRINTK("CLI:pxafb_ezx_getBacklight_brightness: brightness is %d \n", brightness);
        }
#endif
	return (brightness);
}

u8 pxafb_ezx_BackLight_Brightness_Intensity(unsigned long brightness, struct fb_info *info)
{
  // setup file system should call this function whn powr on to set the dfault duty cycle
    struct pxafb_info *fbi = (struct pxafb_info *)info;
    unsigned long dutyCycle;
	
    if (fbi == pxafb_main)
  	{
  		down(&pxafb_global_state.g_sem);
		
	       /*Check if Main panel has been enabledlight, if not ,do nothing*/
  	       if (!(pxafb_global_state.main_state & PXAFB_PANEL_ON))
  	       {
  	       		up(&pxafb_global_state.g_sem);	 
	       		 return -1;
	       }
	   	dutyCycle = pxafb_convert_brightness_to_dutycycle(brightness);
	        PWM_PWDUTY0 = dutyCycle;
	        pxafb_global_state.bklight_main_dutycycle = dutyCycle;
		DPRINTK("MAIN:pxafb_ezx_BackLight_Brightness_Intensity: brightness %x \n", brightness);			
		DPRINTK("MAIN:pxafb_ezx_BackLight_Brightness_Intensity: PWMDUTY0 %x \n", PWM_PWDUTY0);

	        up(&pxafb_global_state.g_sem);
	        return 1;

    }
#ifdef CONFIG_PXAFB_CLI
    if (fbi == pxafb_smart)
    {
    		down(&pxafb_global_state.g_sem);
			
    	       /* Check if there is display on smart panel, if not, don't adjust it's brightness */
   	       if ( !(pxafb_global_state.smart_state & PXAFB_DISP_ON) )
	       {
      	 	     up(&pxafb_global_state.g_sem);	
	   	     return -1;
	       }
		dutyCycle = pxafb_convert_brightness_to_dutycycle(brightness);
	        PWM_PWDUTY0 = dutyCycle;
	        pxafb_global_state.bklight_cli_dutycycle = dutyCycle;
			
		DPRINTK("CLI:pxafb_ezx_BackLight_Brightness_Intensity: brightness %x \n", brightness);			
		DPRINTK("CLI:pxafb_ezx_BackLight_Brightness_Intensity: PWMDUTY0 %x \n", PWM_PWDUTY0);

	        up(&pxafb_global_state.g_sem);
	        return 1;
	
    }
#endif    
    return 0; 
}
u8 pxafb_ezx_getLCD_status(void)
{
	return (u8)(pxafb_main_device_ldm.power_state);
}
EXPORT_SYMBOL(pxafb_ezx_getLCD_status);

#if (0)
EXPORT_SYMBOL(pxafb_ezx_getBacklight_status);
EXPORT_SYMBOL(pxafb_ezx_getBacklight_dutycycle);
EXPORT_SYMBOL(pxafb_ezx_Backlight_turn_on);
EXPORT_SYMBOL(pxafb_ezx_Backlight_turn_off);
EXPORT_SYMBOL(pxafb_ezx_BackLight_Brightness_Intensity);
#endif

static int pxafb_suspend(struct device * dev, u32 state, u32 level)
{
	switch (level) 
	{
	case SUSPEND_POWER_DOWN:
		DPRINTK("pxafb_suspend(%d):SUSPEND_POWER_DOWN, dev->driver_data(%d)\n",current->pid,(unsigned long)dev->driver_data);
		if ( (pxa_panel)(dev->driver_data) == PXA_MAIN_PANEL)
		{
			DPRINTK("pxafb_suspend(%d): disable pxafb_main\n",current->pid);
			set_ctrlr_state(pxafb_main, C_DISABLE);
		}
#ifdef CONFIG_PXAFB_CLI
		if ( (pxa_panel)(dev->driver_data) == PXA_SMART_PANEL)
		{
			DPRINTK("pxafb_suspend(%d): disable pxafb_smart\n",current->pid);
			set_ctrlr_state(pxafb_smart, C_DISABLE);
		}
#endif
		break;
#ifdef CONFIG_PXAFB_CLI
	case SUSPEND_DISABLE:
		DPRINTK("pxafb_suspend(%d):SUSPEND_DISABLE, dev->driver_data(%d)\n",current->pid,(unsigned long)dev->driver_data);
		if ( (pxa_panel)(dev->driver_data) == PXA_SMART_PANEL)
		{
			DPRINTK("pxafb_suspend(%d): disable pxafb_smart\n",current->pid);
			set_ctrlr_state(pxafb_smart, C_SUSPEND);
		}
		else
		{
			/*
			* For MAIN panel pxafb_main, do nothing with SUSPEND_DISABLE
			*/
			DPRINTK("pxafb_suspend: main panel doesn't have C_SUSPEND state\n");
			return -EAGAIN;
		}
		
		break;
#endif
	}

	return 0;
}

static int pxafb_resume(struct device * dev, u32 level)
{
	struct pxafb_info *this_pxafb = NULL;

	if ((unsigned long)dev->driver_data == (unsigned long)PXA_MAIN_PANEL)
		this_pxafb = pxafb_main;
#ifdef CONFIG_PXAFB_CLI
	if ((unsigned long)dev->driver_data == (unsigned long)PXA_SMART_PANEL)
		this_pxafb = pxafb_smart;
#endif
	switch (level) {
	case RESUME_POWER_ON:
		set_ctrlr_state(this_pxafb, C_ENABLE);
		break;
	}
	
	return 0;
}

static int pxafb_activate_var(struct fb_var_screeninfo *var, struct pxafb_info *);

static inline void pxafb_schedule_task(struct pxafb_info *fbi, u_int state)
{
	unsigned long flags;

	local_irq_save(flags);
	/*
	 * We need to handle two requests being made at the same time.
	 * There are two important cases:
	 *  1. When we are changing VT (C_REENABLE) while unblanking (C_ENABLE)
	 *     We must perform the unblanking, which will do our REENABLE for us.
	 *  2. When we are blanking, but immediately unblank before we have
	 *     blanked.  We do the "REENABLE" thing here as well, just to be sure.
	 */
	if (fbi->task_state == C_ENABLE && state == C_REENABLE)
		state = (u_int) -1;
	if (fbi->task_state == C_DISABLE && state == C_ENABLE)
		state = C_REENABLE;

	if (state != (u_int)-1) {
		fbi->task_state = state;
		schedule_task(&fbi->task);
	}
	local_irq_restore(flags);
}

/*
 * Get the VAR structure pointer for the specified console
 */
static inline struct fb_var_screeninfo *get_con_var(struct fb_info *info, int con)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	return (con == fbi->currcon || con == -1) ? &fbi->current_fb->var : &fb_display[con].var;
}

/*
 * Get the DISPLAY structure pointer for the specified console
 */
static inline struct display *get_con_display(struct fb_info *info, int con)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	return (con < 0) ? fbi->fb.disp : &fb_display[con];
}

/*
 * Get the CMAP pointer for the specified console
 */
static inline struct fb_cmap *get_con_cmap(struct fb_info *info, int con)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	return (con == fbi->currcon || con == -1) ? &fbi->current_fb->cmap : &fb_display[con].cmap;
}

static inline u_int
chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int
pxafb_setpalettereg(u_int regno, u_int red, u_int green, u_int blue,
		       u_int trans, struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	u_int val, ret = 1;

	if (regno < fbi->palette_size) {
		val = ((red >> 0) & 0xf800);
		val |= ((green >> 5) & 0x07e0);
		val |= ((blue >> 11) & 0x001f);

		fbi->palette_cpu[regno] = val;
		ret = 0;
	}
	return ret;
}

static int
pxafb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
		   u_int trans, struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	u_int val;
	int ret = 1;

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no mater what visual we are using.
	 */
	if (fbi->current_fb->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
					7471 * blue) >> 16;

	switch (fbi->current_fb->disp->visual) {
	case FB_VISUAL_TRUECOLOR:
	case FB_VISUAL_DIRECTCOLOR:
		/*
		 * 12 or 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno <= 16) {
			u16 *pal = fbi->current_fb->pseudo_palette;

			val  = chan_to_field(red, &fbi->current_fb->var.red);
			val |= chan_to_field(green, &fbi->current_fb->var.green);
			val |= chan_to_field(blue, &fbi->current_fb->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_PSEUDOCOLOR:
		ret = pxafb_setpalettereg(regno, red, green, blue, trans, info);
		break;
	}

	return ret;
}

/*
 *  pxafb_decode_var():
 *    Get the video params out of 'var'. If a value doesn't fit, round it up,
 *    if it's too big, return -EINVAL.
 *
 *    Suggestion: Round up in the following order: bits_per_pixel, xres,
 *    yres, xres_virtual, yres_virtual, xoffset, yoffset, grayscale,
 *    bitfields, horizontal timing, vertical timing.
 */
static int pxafb_validate_var(struct fb_var_screeninfo *var,
				 struct pxafb_info *fbi)
{
	int ret = -EINVAL;

	if (var->xres < MIN_XRES)
		var->xres = MIN_XRES;
	if (var->yres < MIN_YRES)
		var->yres = MIN_YRES;
	if (var->xres > fbi->max_xres)
		var->xres = fbi->max_xres;
	if (var->yres > fbi->max_yres)
		var->yres = fbi->max_yres;
	var->xres_virtual =
	    var->xres_virtual < var->xres ? var->xres : var->xres_virtual;
	var->yres_virtual =
	    var->yres_virtual < var->yres ? var->yres : var->yres_virtual;
	    
	if (var->xoffset + var->xres > var->xres_virtual)
		var->xoffset = var->xres_virtual - var->xres;
	if (var->yoffset + var->yres > var->yres_virtual)  //for 16/18bpp case, yoffset(324), yres_virtual(644). for 24bpp case, yoffset(320),yres_virtual(640).
        var->yoffset = var->yres_virtual - var->yres;//so no conflict with double bufferings//
                
	DPRINTK("var->bits_per_pixel=%d", var->bits_per_pixel);
	switch (var->bits_per_pixel) {
#ifdef FBCON_HAS_CFB4
	case 4:  ret = 0; break;
#endif
#ifdef FBCON_HAS_CFB8
	case 8:  ret = 0; break;
#endif
#ifdef FBCON_HAS_CFB16
	case 12:
		/* make sure we are in passive mode */
		if (!(fbi->reg_lccr0 & LCCR0_PAS))
			ret = 0;
		break;

	case 16:
		/* 
		 * 16 bits works apparemtly fine in passive mode for those,
		 * so don't complain
		 */
		if (machine_is_lubbock() ||
		    machine_is_mainstone() ||
		    machine_is_pxa_cerf()) {
			ret = 0;
		} else
			/* make sure we are in active mode */
			if ((fbi->reg_lccr0 & LCCR0_PAS))
				ret = 0;
		break;
#endif
#ifdef CONFIG_CPU_BULVERDE
	case 18:
	case 19:
	case 24:
	case 25:
		DPRINTK("pxafb_validate_var: case 18, 19, 24, 25.\n");		
		ret = 0;
		break;
	case 2:
		ret = 0;
		break;		
#endif
	default:
		break;
	}

	return ret;
}

static inline void pxafb_set_truecolor(u_int is_true_color)
{
	DPRINTK("true_color = %d", is_true_color);
}

static void
pxafb_hw_set_var(struct fb_var_screeninfo *var, struct pxafb_info *fbi)
{

	fb_set_cmap(&fbi->current_fb->cmap, 1, pxafb_setcolreg, &fbi->fb);

	/* Set board control register to handle new color depth */
	pxafb_set_truecolor(var->bits_per_pixel >= 16);

	pxafb_activate_var(var, fbi);

}

/*
 * pxafb_set_var():
 *	Set the user defined part of the display for the specified console
 */
static int
pxafb_set_var(struct fb_var_screeninfo *var, int con, struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	struct fb_var_screeninfo *dvar = get_con_var(&fbi->fb, con);
	struct display *display = get_con_display(&fbi->fb, con);
	int err, chgvar = 0, rgbidx;

	DPRINTK("pxafb_set_var");
#ifdef DEBUG_CLI
	printk(KERN_NOTICE "pxafb_set_var(%d): panel is (%d)\n", current->pid, info->modename[0]);
#endif

	down(&pxafb_global_state.g_sem);
	/*
	 * Decode var contents into a par structure, adjusting any
	 * out of range values.
	 */
	err = pxafb_validate_var(var, fbi);
	if (err)
	{
		up(&pxafb_global_state.g_sem);
		return err;
	}

	if (var->activate & FB_ACTIVATE_TEST)
	{
		up(&pxafb_global_state.g_sem);
		return 0;
	}
	if ((var->activate & FB_ACTIVATE_MASK) != FB_ACTIVATE_NOW)  //For base/overlay1/overlay2/cursor, var.activate = FB_ACTIVATE_NOW //
	{
		up(&pxafb_global_state.g_sem);
		return -EINVAL;
	}
	
	if (dvar->xres != var->xres)
		chgvar = 1;
	if (dvar->yres != var->yres)
		chgvar = 1;
	if (dvar->xres_virtual != var->xres_virtual)
		chgvar = 1;
	if (dvar->yres_virtual != var->yres_virtual)
		chgvar = 1;
	if (dvar->bits_per_pixel != var->bits_per_pixel)
		chgvar = 1;
	if (con < 0)
		chgvar = 0;

	switch (var->bits_per_pixel) {
#ifdef FBCON_HAS_CFB4
	case 4:
		if (fbi->cmap_static)
			display->visual	= FB_VISUAL_STATIC_PSEUDOCOLOR;
		else
			display->visual	= FB_VISUAL_PSEUDOCOLOR;
		display->line_length	= var->xres / 2;
		display->dispsw		= &fbcon_cfb4;
		rgbidx			= RGB_8;
		break;
#endif
#ifdef FBCON_HAS_CFB8
	case 8:
		if (fbi->cmap_static)
			display->visual	= FB_VISUAL_STATIC_PSEUDOCOLOR;
		else
			display->visual	= FB_VISUAL_PSEUDOCOLOR;
		display->line_length	= var->xres;
		display->dispsw		= &fbcon_cfb8;
		rgbidx			= RGB_8;
		break;
#endif
#ifdef FBCON_HAS_CFB16
	case 12:
	case 16:
		display->visual		= FB_VISUAL_TRUECOLOR;
		display->line_length	= var->xres * 2;
		display->dispsw		= &fbcon_cfb16;
		display->dispsw_data	= fbi->current_fb->pseudo_palette;
		rgbidx			= RGB_16;
		break;
#endif
#ifdef CONFIG_CPU_BULVERDE
	case 18:
		display->visual		= FB_VISUAL_TRUECOLOR;
		display->line_length	= var->xres * 3;
		display->dispsw = &fbcon_cfb24;
		display->dispsw_data	= fbi->current_fb->pseudo_palette;
		rgbidx                  = RGB_18;
		break;
	case 19:
		display->visual		= FB_VISUAL_TRUECOLOR;
		display->line_length	= var->xres * 3;
		display->dispsw = &fbcon_cfb24;
		display->dispsw_data	= fbi->current_fb->pseudo_palette;
		rgbidx                  = RGB_19;
		break;		
	case 24:
	case 25:
		display->visual		= FB_VISUAL_TRUECOLOR;
		display->line_length	= var->xres * 4;
		display->dispsw = &fbcon_cfb24;
		display->dispsw_data	= fbi->current_fb->pseudo_palette;
		rgbidx                  = RGB_24;
		break;
#endif /* CONFIG_CPU_BULVERDE */
	default:
		rgbidx = 0;
		display->dispsw = &fbcon_dummy;
		break;
	}

	display->screen_base	= fbi->first_fb.screen_cpu;  //Susan: need to be accurate //
	display->next_line	= display->line_length;
	display->type		= fbi->current_fb->fix.type;
	display->type_aux	= fbi->current_fb->fix.type_aux;
	display->ypanstep	= fbi->current_fb->fix.ypanstep;
	display->ywrapstep	= fbi->current_fb->fix.ywrapstep;
	display->can_soft_blank	= 1;
	display->inverse	= 0;

	*dvar			= *var;
	dvar->activate		&= ~FB_ACTIVATE_ALL;

	/*
	 * Copy the RGB parameters for this display
	 * from the machine specific parameters.
	 */
	dvar->red		= fbi->rgb[rgbidx]->red;
	dvar->green		= fbi->rgb[rgbidx]->green;
	dvar->blue		= fbi->rgb[rgbidx]->blue;
	dvar->transp		= fbi->rgb[rgbidx]->transp;

	DPRINTK("RGBT length = %d:%d:%d:%d",
		dvar->red.length, dvar->green.length, dvar->blue.length,
		dvar->transp.length);

	DPRINTK("RGBT offset = %d:%d:%d:%d",
		dvar->red.offset, dvar->green.offset, dvar->blue.offset,
		dvar->transp.offset);

	/*
	 * Update the old var.  The fbcon drivers still use this.
	 * Once they are using fbi->fb.var, this can be dropped.
	 */
	display->var = *dvar;

	/*
	 * If we are setting all the virtual consoles, also set the
	 * defaults used to create new consoles.
	 */
	if (var->activate & FB_ACTIVATE_ALL)
		fbi->current_fb->disp->var = *dvar;

	/*
	 * If the console has changed and the console has defined
	 * a changevar function, call that function.
	 */
	if (chgvar && info && fbi->fb.changevar)
		fbi->fb.changevar(con);  //should be fbi->fb.changevar but not fbi->current_fb->changevar, because pxafb_info.fb_info is registered as independant device.

	/* If the current console is selected, activate the new var. */
	if (con != fbi->currcon)
		return 0;

	pxafb_hw_set_var(dvar, fbi);

	//if ((fbi == pxafb_smart) && ( list_empty(&(fbi->smart_refresh_task.list)) ))  //For the first time invoking of pxafb_set_var()
	//{
		//register pxafb_smart_refresh_task in tq_context queue -- Susan //
	//	schedule_task(&fbi->smart_refresh_task);
	//}

	up(&pxafb_global_state.g_sem);

	return 0;
}

static int
__do_set_cmap(struct fb_cmap *cmap, int kspc, int con,
	      struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	struct fb_cmap *dcmap = get_con_cmap(info, con);
	int err = 0;

	if (con == -1)
		con = fbi->currcon;

	/* no colormap allocated? (we always have "this" colour map allocated) */
	if (con >= 0)
		err = fb_alloc_cmap(&fb_display[con].cmap, fbi->palette_size, 0);

	if (!err && con == fbi->currcon)
		err = fb_set_cmap(cmap, kspc, pxafb_setcolreg, info);

	if (!err)
		fb_copy_cmap(cmap, dcmap, kspc ? 0 : 1);

	return err;
}

static int
pxafb_set_cmap(struct fb_cmap *cmap, int kspc, int con,
		  struct fb_info *info)
{
	struct display *disp = get_con_display(info, con);

	return (disp->visual == FB_VISUAL_TRUECOLOR) ? -EINVAL :
		__do_set_cmap(cmap, kspc, con, info);
}

static int
pxafb_get_fix(struct fb_fix_screeninfo *fix, int con, struct fb_info *info)
{
	struct display *display = get_con_display(info, con);

	*fix = ((struct pxafb_info *)info)->current_fb->fix;

	fix->line_length = display->line_length;
	fix->visual	 = display->visual;
	return 0;
}

static int
pxafb_get_var(struct fb_var_screeninfo *var, int con, struct fb_info *info)
{
	down(&pxafb_global_state.g_sem);
	
	*var = *get_con_var(info, con);
	
#ifdef CONFIG_FB_PXA_19BPP
	var->bits_per_pixel = 18;  //Susan for cheating QT //
#endif
	up(&pxafb_global_state.g_sem);
	
	return 0;
}

static int
pxafb_get_cmap(struct fb_cmap *cmap, int kspc, int con, struct fb_info *info)
{
	struct fb_cmap *dcmap = get_con_cmap(info, con);
	fb_copy_cmap(dcmap, cmap, kspc ? 0 : 2);
	return 0;
}


static int
pxafb_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	       unsigned long arg, int con, struct fb_info *info)
{
	u8 val;
	int ret;
	unsigned long val_bkbrigness;

	switch (cmd) {
	case FBIOSETBKLIGHT:
	    DPRINTK("pxafb_ioctl:FBIOSETBKLIGHT,arg(%d)\n",arg);
  	    switch (arg){
	    case BKLIGHT_OFF:
		 pxafb_ezx_Backlight_turn_off(info);
		 return 0;
            case BKLIGHT_ON:
                 pxafb_ezx_Backlight_turn_on(info);
		 return 0;	
		}
	case FBIOGETBKLIGHT:
	    DPRINTK("pxafb_ioctl: This is FBIOGETBKLIGHT.\n");
	    val = pxafb_ezx_getBacklight_status(info);
	    put_user(val, (u8*)arg);
	    return 0;
	case FBIOSETBRIGHTNESS:
	    DPRINTK("pxafb_ioctl: This is FBIOSETBRIGHTNESS.\n");
	    if ( (arg < MIN_BRIGHTNESS) || (arg > MAX_BRIGHTNESS) )
            {
            	return -EFAULT;
            }    
 	    if (pxafb_ezx_BackLight_Brightness_Intensity((unsigned long)arg, info))
	    {
		    return 0;	
	    }    
	    else return -EFAULT;   
	case FBIOGETBRIGHTNESS:
	    DPRINTK("pxafb_ioctl: This is FBIOGETBRIGHTNESS.\n");
	    val_bkbrigness = pxafb_ezx_getBacklight_brightness(info);
	    copy_to_user((unsigned long *)arg, &val_bkbrigness, sizeof(unsigned long));
	    return 0;
	/* Add interface for app to set brightness range at app's request*/
  	case FBIOSET_BRIGHTNESSRANGE:
	    DPRINTK("pxafb_ioctl: FBIOSET_BRIGHTNESSRANGE.\n");
	    ret = copy_from_user(&pxafb_global_state.bkduty_range, (struct bk_visible_dutycycle_range *)arg, sizeof(struct bk_visible_dutycycle_range))? -EFAULT:0;
	    if (ret)
	    {
			return ret;
	    }
		
	    if ( (pxafb_global_state.bkduty_range.min < 0 )||(pxafb_global_state.bkduty_range.min >pxafb_global_state.bkduty_range.max))
            {
            	 	return -EFAULT;
            }

	    DPRINTK("pxafb_ioctl: visible dutycycle range[%d,%d] \n",pxafb_global_state.bkduty_range.min,pxafb_global_state.bkduty_range.max );    
	    DPRINTK("pxafb_ioctl: visible dutycycle range[%d,%d] \n",pxafb_global_state.bkduty_range.min,pxafb_global_state.bkduty_range.max );
	    return 0;	
	case FBIOCKMAINVALIDFB:
		{
			if ( (info->modename[0] < PXAFB_CLI_BASE) && (info->modename[0] == PXAFB_MAIN_VALID_FB) )
				return 1;
			else
				return 0;
		}
#ifdef CONFIG_PXAFB_CLI
	case FBIOSMARTUPDATE:
		{
			int ret = 0;
			down(&pxafb_global_state.g_sem);
			
			/*Check if the smart panel is active */
			if ( (pxafb_global_state.active_panel != PXA_SMART_PANEL) || ((pxafb_global_state.smart_state & 0x6) == 0x0) )
			{
				up(&pxafb_global_state.g_sem);			
				return -EIO;
			}	

			ret = pxafb_smart_update(pxafb_smart);
			up(&pxafb_global_state.g_sem);

			if (ret)
				return -EIO;
			else
				return 0;			
		}
	case FBIOENTERCLIPWRSAVE:	/* Added  to support CLI CSTN partial mode */
		{
			/* Will down(&pxafb_global_state.g_sem) in set_ctrlr_state*/
			DPRINTK("pxafb_ioctl(FBIOENTER_CLIPWRSAVE): \n");
			set_ctrlr_state(pxafb_smart, C_ENTER_PWRSAVE);

			return 0;			
	      }
	case FBIOEXITCLIPWRSAVE:	/* Added  to support CLI CSTN partial mode */
		{
			int ret = 0;
			down(&pxafb_global_state.g_sem);
			
			/*Check if the smart panel is active */
			if ( (pxafb_global_state.active_panel != PXA_SMART_PANEL) || ((pxafb_global_state.smart_state & 0x6) == 0x0) )
			{
				up(&pxafb_global_state.g_sem);				
				return -EIO;
			}
			/* Check if CLI in partial mode */
			if ( pxafb_global_state.smart_state & PXAFB_PARTIAL_DISP_ON ) 
			{
				DPRINTK("PXAFB_IOCTL: Before exiting partial mode\n");
				ret = pxafb_smart_exit_partial_mode(pxafb_smart);/*Exit from partial mode*/
				if (!ret)
				{
					pxafb_global_state.smart_state &= ~PXAFB_PARTIAL_DISP_ON; //clear partial mode state bit
				}
				
			}
			up(&pxafb_global_state.g_sem);
			return ret;			
	      }

#endif
	case FBIOCHECK2BFS:
		{
			struct pxafb_info *pxafb_this = (struct pxafb_info *)info;
			
			down(&pxafb_global_state.g_sem);
			
#if (0)
			printk(KERN_NOTICE "pxafb_ioctl(FBIOCHECK2BFS):\n");
			printk(KERN_NOTICE "pxafb_ioctl(FBIOCHECK2BFS):fix.id[0](%d),info->modename[0](%d),pxafb_global_state.pxafb_current(0x%x),pxafb_this(0x%x),current_fb(0x%x),info(0x%x)\n",pxafb_this->current_fb->fix.id[0],pxafb_this->current_fb->modename[0],pxafb_global_state.pxafb_current,pxafb_this,pxafb_this->current_fb,info);
#endif

			if ( pxafb_this->double_buffers == 1)
			{
					unsigned long *p_screen_2_dma = (unsigned long *)arg;
					
					*p_screen_2_dma = pxafb_this->second_fb.screen_dma;

					up(&pxafb_global_state.g_sem);
					return 1;  //two framebuffers are never contiguous in SRAM or SDRAM //
			}

			up(&pxafb_global_state.g_sem);
			return 0;  //Currently double buffer is not supported for CLI panel //
		}
#if (0)
	case FBIOGETWPSTATE:
		{
			if (wpaper_enabled == 1)
			{
				unsigned long *addr = (unsigned long *)arg;
				(*addr) = pxafbi->second_fb.screen_dma;
				return 1;
			}
			else
				return 0;
		}
#endif

#if CONFIG_FBBASE_DOUBLE_BUFFERS
	case FBIOENABLE2BFS:
		{
			struct pxafb_info *fbi = (struct pxafb_info *)(info);
			unsigned long bytes_per_panel;
			unsigned long secondfb_len;
			struct list_head *tmp;
			struct db_entry_s *db_entry;
			struct db_entry_s *tmp_entry;
			unsigned short bytes_per_pixel;

			down(&pxafb_global_state.g_sem);
			
			switch(fbi->max_bpp)  //Note that pxafb_info.max_bpp may be not equal to var.bits_per_pixel //
			{
				case  16:
					bytes_per_pixel = 2;
					break;
				case 18:
				case 19:
					bytes_per_pixel = 3;
					break;
				case 24:
				case 25:
					bytes_per_pixel = 4;
					break;
				default:
					bytes_per_pixel = 1;
					break;
			}
			

DPRINTK("ENABLE2BFS--BEGIN:task(%d) enters into enable2bfs ioctl\n", current->pid);
DPRINTK("ENABLE2BFS--BEGIN:task(%d-%s) enters into enable2bfs ioctl\n", current->pid,current->comm);
			//Don't need to take semaphore //
			if (fbi->double_buffers == 0)  //The second framebuffer is not allocated yet //
			{
				bytes_per_panel = fbi->max_xres * fbi->max_yres * bytes_per_pixel;
				secondfb_len = PAGE_ALIGN(bytes_per_panel + PAGE_SIZE);  //only be available for 16bpp//
				fbi->second_fb.map_cpu = __consistent_alloc(GFP_KERNEL, secondfb_len, &(fbi->second_fb.map_dma), PTE_BUFFERABLE);
				if (!(fbi->second_fb.map_cpu))
				{

DPRINTK("ENABLE2BFS:allocate second fb fail,fbi->double_buffers = 0\n");

					fbi->double_buffers = 0;
					
					up(&pxafb_global_state.g_sem);
					return -ENOMEM;
				}

DPRINTK("consistent_alloc succeed, secondfb_len(%d), fbi->double_buffers(%d)\n", secondfb_len, fbi->double_buffers);

				memset((void *)fbi->second_fb.map_cpu, 0, secondfb_len);
				
				fbi->second_fb.screen_cpu = fbi->second_fb.map_cpu + PAGE_SIZE;
				fbi->second_fb.screen_dma = fbi->second_fb.map_dma + PAGE_SIZE;

				fbi->second_fb.dmadesc_fb_cpu = (struct pxafb_dma_descriptor *)(fbi->second_fb.screen_cpu -1*16);
				fbi->second_fb.dmadesc_fb_dma = fbi->second_fb.screen_dma - 1*16;

				fbi->second_fb.dmadesc_fb_cpu->fdadr = fbi->second_fb.dmadesc_fb_dma;
				fbi->second_fb.dmadesc_fb_cpu->fsadr = fbi->second_fb.screen_dma;
				fbi->second_fb.dmadesc_fb_cpu->fidr = 0;
				fbi->second_fb.dmadesc_fb_cpu->ldcmd = bytes_per_panel;

				fbi->current_fb->fix.smem_len = PAGE_ALIGN(bytes_per_panel) * 2;
				// 320*2 + 4 is the number calculated by Peter, for indicating the second fb//
				fbi->current_fb->var.yres_virtual = fbi->fb.var.yres *2 + ((bytes_per_pixel & 0x3)?4:0);  //both 16bpp and 18bpp, the delta lines are always 4 lines //
				fbi->current_fb->var.yoffset = 0;
				
				fbi->double_buffers = 1;

DPRINTK("ENABLE-2-BFS:fbi->double_buffers(%d),CURRENT(%s)\n",fbi->double_buffers,current->comm);
			}

			list_for_each(tmp,&(fbi->db_task_list))
			{
				tmp_entry = list_entry(tmp,struct db_entry_s,task_list);
		    	if (tmp_entry->pid == current->pid)
		    		goto exists;
			}
			
			db_entry = kmalloc(sizeof(struct db_entry_s),GFP_KERNEL);
			if (!db_entry)
			{
				up(&pxafb_global_state.g_sem);
				return -ENOSPC;
			}

			//JUST FOR TESTING		db_entry->task = current; //just for testing
			db_entry->pid = current->pid;
		    list_add(&(db_entry->task_list), &(fbi->db_task_list));

DPRINTK("Enable dbuffers:EMPTY_LIST(%d),&(fbi->db_task_list)(0x%x),(fbi->db_task_list.prev)(0x%x),(fbi->db_task_list.next)(0x%x),db_entry->task(%d),&(db_entry->task_list)(0x%x)\n",list_empty(&(fbi->db_task_list)),&(fbi->db_task_list),(fbi->db_task_list.prev),(fbi->db_task_list.next),db_entry->pid,&(db_entry->task_list));

exists:
			up(&pxafb_global_state.g_sem);
			return 0;
		}
	case FBIODISABLE2BFS:
		{
			struct pxafb_info *fbi = (struct pxafb_info *)info;
			unsigned int flags;
			struct db_entry_s *tmp_entry;
			struct list_head *tmp;
			unsigned short bytes_per_pixel;

			down(&pxafb_global_state.g_sem);
			
			switch(fbi->max_bpp)
			{
				case 16:
					bytes_per_pixel = 2;
					break;
				case 18:
				case 19:
					bytes_per_pixel = 3;
					break;
				case 24:
				case 25:
					bytes_per_pixel = 4;
					break;
				default:
					bytes_per_pixel = 1;
					break;
			}
			
			if ( list_empty(&(fbi->db_task_list)) ) //Nobody is using LCD double buffers//
			{
DPRINTK("DISABLE2BFS: list empty,fbi->double_buffers = 0;\n");
				fbi->double_buffers = 0;
				
				up(&pxafb_global_state.g_sem);
				return 0;
			}
			else
			{

DPRINTK("DISABLE2BFS:list is not empty\n");
				
				tmp = fbi->db_task_list.next;
				while ( tmp != &(fbi->db_task_list) )
				{
					tmp_entry = list_entry(tmp,struct db_entry_s,task_list);
					tmp = tmp->next;

DPRINTK("DISABLE2BFS:current(%s-%d),task(%d),&task_list(0x%x)\n",current->comm,current->pid,tmp_entry->pid,&(tmp_entry->task_list));

			    	if (tmp_entry->pid == current->pid)
					{
DPRINTK("DISABLE2BFS:tmp_entry->task(%d) will be removed\n",tmp_entry->pid);
			    		list_del(&tmp_entry->task_list);
						kfree(tmp_entry);

						/* If db_task_list contains only one entry, switch to #1 framebuffer */
						if ( (tmp != &(fbi->db_task_list)) && (tmp->next == tmp->prev) )
						{
							DPRINTK("FBIODISABLE2BFS: current(%s-%d),resume 1# framebuffer\n", current->comm,current->pid);
							if (fbi->current_fb == &fbi->fb)  //base is the exported framebuffer //
							{
								/* switch to the first framebuffer */
								if (fbi->fb.var.yoffset >= fbi->max_yres)  // The second fb is used for LCD panel, 16,18bpp(>), 24bpp(>=) //
								{
									local_irq_save(flags);

									fbi->dmadesc_fbhigh_cpu = fbi->first_fb.dmadesc_fb_cpu;
									fbi->dmadesc_fbhigh_cpu->fdadr = fbi->first_fb.dmadesc_fb_dma;
									fbi->dmadesc_fbhigh_dma = fbi->first_fb.dmadesc_fb_dma;
									fbi->fdadr0 = fbi->dmadesc_fbhigh_dma;
			
									FBR0 = fbi->fdadr0 | 0x01; //don't generate interrupt bit LCSR0[BS0] //
							
									local_irq_restore(flags);
							
									mdelay(18);  //PCD is 9, frame freq = 55 Frames/Second //
								}

								fbi->fb.var.yoffset = 0;							
							}
							if (fbi->current_fb != &fbi->fb) //base framebuffer is not the exported framebuffer //
							{
								/* switch to the first framebuffer */
								if (fbi->current_fb->var.yoffset >= fbi->max_yres)  // The second fb is used for LCD panel //
								{
									local_irq_save(flags);
									
									fbi->fdadr1 = fbi->first_fb.dmadesc_fb_dma;
									FBR1= fbi->fdadr1 | 0x01; //assuming double buffer for overlay1, don't generate interrupt bit LCSR0[BS0] //
							
									local_irq_restore(flags);
							
									mdelay(18);  //PCD is 9, frame freq = 55 Frames/Second //
								}

								fbi->current_fb->var.yoffset = 0;							
							}
						}
					}//current task related db_entry_s is removed
				}

				if ( list_empty(&(fbi->db_task_list)) )
				{
DPRINTK("DISABLE2BFS:After delete, list is empty\n");
DPRINTK("DISABLE2BFS:releaes #2 framebuffer\n");

					/* release second framebuffer -- 152KB + PAGE_SIZE */
					consistent_free((void *)fbi->second_fb.map_cpu, fbi->map_size, fbi->second_fb.map_dma);

					fbi->second_fb.map_cpu = (u_char *)NULL;
					fbi->second_fb.map_dma = (dma_addr_t)NULL;
					fbi->second_fb.dmadesc_fb_cpu = (struct pxafb_dma_descriptor *)NULL;
					fbi->second_fb.dmadesc_fb_dma = (dma_addr_t)NULL;
					fbi->second_fb.screen_cpu = (u_char *)NULL;
					fbi->second_fb.screen_dma = (dma_addr_t)NULL;

DPRINTK("DISABLE2BFS:AFTER releaes #2 framebuffer, fbi->double_buffers = 0\n");

 					fbi->double_buffers = 0;
 					
					fbi->current_fb->fix.smem_len = fbi->max_xres * fbi->max_yres * bytes_per_pixel;
					fbi->current_fb->var.yres_virtual = fbi->fb.var.yres;
					fbi->current_fb->var.yoffset = 0;
 				}
			}
			up(&pxafb_global_state.g_sem);
			return 0;
		}
#endif  //#if CONFIG_FBBASE_DOUBLE_BUFFERS
	case FBIOENABLEOVL2:
	{
	    DPRINTK("PXAFB_IOCTL: FBIOENABLEOVL2:\n");
	    down(&pxafb_global_state.g_sem);
	    val = overlay2fb_enable( (struct fb_info *)(((struct pxafb_info *)info)->overlay2fb) );
	    put_user(val, (u8*)arg);

	    pxafb_global_state.main_ovl2_status = (unsigned char)PXAFB_OVL2_ON;
	    DPRINTK("pxafb_ioctl: FBIOENABLEOVL2 -- main_ovl2_status = PXAFB_OVL2_ON\n");
	    ret = ovl2_ipm_hook(); /*Hook ovl2 to PM for keeping HIGH cpu freq when ovl2 is enabled*/
	    up(&pxafb_global_state.g_sem);
	    DPRINTK("After FBIOENABLEOVL2 -- LCCR0(0x%x)LCCR1(0x%x)LCCR2(0x%x)LCCR3(0x%x)LCCR4(0x%x)LCCR5(0x%x)OVL2C1(0x%x)OVL2C2(0x%x)\n", LCCR0,LCCR1,LCCR2,LCCR3,LCCR4,LCCR5,OVL2C1,OVL2C2);
	    return 0;
	}
	case FBIODISABLEOVL2:
	{
	    DPRINTK("FBIODISABLEOVL2:\n");
	    down(&pxafb_global_state.g_sem);
	    val = overlay2fb_disable( (struct fb_info *)(((struct pxafb_info *)info)->overlay2fb) );
	    put_user(val, (u8*)arg);

	    pxafb_global_state.main_ovl2_status = (unsigned char)PXAFB_OVL2_OFF;
	    DPRINTK("pxafb_ioctl: FBIODISABLEOVL2 -- main_ovl2_status = PXAFB_OVL2_OFF(%d)\n", pxafb_global_state.main_ovl2_status);
	    ovl2_ipm_unhook(); /*Unhook ovl2 from PM, restore cpu freq scaling*/
	    up(&pxafb_global_state.g_sem);
	    return 0;
	}
	case FBIOGET_OVL2_MAPINFO:
	{
		struct pxafb_info *pxafb_this = (struct pxafb_info *)info;
		struct fb_ovl2_mapinfo *ovl2_map_info = (struct fb_ovl2_mapinfo *)arg;
		unsigned long tmp_len;
		
		DPRINTK("pxafb_ioctl:  FBIOGET_OVL2INFO\n");
		
		down(&pxafb_global_state.g_sem);
		
		if (pxafb_this->double_buffers == 1)
			ovl2_map_info->ovl2_offset = pxafb_this->fb.fix.smem_len;
		else
			ovl2_map_info->ovl2_offset = PAGE_ALIGN(pxafb_this->fb.fix.smem_len) * 2;
		
		ovl2_map_info->ovl2_size = pxafb_this->overlay2fb->fb.fix.smem_len;
		ovl2_map_info->ovl2_priv = (unsigned long)pxafb_this->overlay2fb->screen_dma;
		ovl2_map_info->ovl2_framelen = pxafb_this->overlay2fb->fb.fix.smem_len/2;
		tmp_len = (pxafb_this->overlay2fb->fb.fix.line_length * pxafb_this->overlay2fb->fb.var.yres);
		ovl2_map_info->aylen = (tmp_len + 0xf)&(~0xf);
		ovl2_map_info->acblen = (tmp_len/2 + 0xf)&(~0xf);
		ovl2_map_info->acrlen = (tmp_len/2 + 0xf)&(~0xf);

		DPRINTK("FBIOGET_OVL2_MAPINFO:ovl2_offset(0x%x)\n", ovl2_map_info->ovl2_offset);
		DPRINTK("FBIOGET_OVL2_MAPINFO:ovl2_size(0x%x)\n", ovl2_map_info->ovl2_size);
		DPRINTK("FBIOGET_OVL2_MAPINFO:ovl2_framelen(0x%x)\n", ovl2_map_info->ovl2_framelen);
		DPRINTK("FBIOGET_OVL2_MAPINFO:aylen(0x%x)\n", ovl2_map_info->aylen);
		DPRINTK("FBIOGET_OVL2_MAPINFO:acblen(0x%x)\n", ovl2_map_info->acblen);
		DPRINTK("FBIOGET_OVL2_MAPINFO:acrlen(0x%x)\n", ovl2_map_info->acrlen);
		DPRINTK("FBIOGET_OVL2_MAPINFO:ovl2_priv(0x%x)\n", ovl2_map_info->ovl2_priv);
		
		up(&pxafb_global_state.g_sem);
		DPRINTK("pxafb_ioctl:  return 0 for FBIOGET_OVL2INFO\n");
		return 0;
	}
	case FBIOGET_OVL2FIX:
	{
		struct pxafb_info *pxafb_this = (struct pxafb_info *)info;
		DPRINTK("pxafb_ioctl: FBIOGET_OVL2FIX\n");

		return copy_to_user( (void *)arg, &(pxafb_this->overlay2fb->fb.fix), sizeof(struct fb_fix_screeninfo) )? -EFAULT:0;
	}
	case FBIOGET_OVL2VAR:
	{
		struct pxafb_info *pxafb_this = (struct pxafb_info *)info;
		struct fb_var_screeninfo ovl2_var;
		int ret = 0;
		
		DPRINTK("pxafb_ioctl: FBIOGET_OVL2VAR\n");

		down(&pxafb_global_state.g_sem);

		overlay2fb_get_var(&ovl2_var, 0, (struct fb_info *)(pxafb_this->overlay2fb));
		ret = copy_to_user( (void *)arg, &(pxafb_this->overlay2fb->fb.var), sizeof(struct fb_var_screeninfo) )? -EFAULT:0;
		
		up(&pxafb_global_state.g_sem);
		
		return ret;
	}
	case FBIOPUT_OVL2VAR:
	{
		struct pxafb_info *pxafb_this = (struct pxafb_info *)info;
		struct fb_var_screeninfo ovl2_var;
		int ret;
		
		DPRINTK("pxafb_ioctl: FBIOPUT_OVL2VAR\n");

		down(&pxafb_global_state.g_sem);

		ret = copy_from_user(&ovl2_var, (struct fb_var_screeninfo *)arg, sizeof(struct fb_var_screeninfo))? -EFAULT:0;
		if (ret)
		{
			up(&pxafb_global_state.g_sem);
			return ret;
		}
		ret = overlay2fb_pure_set_var(&ovl2_var, 0, (struct fb_info *)(pxafb_this->overlay2fb));
		
		up(&pxafb_global_state.g_sem);
		
		return ret;
	}
	case FBIOPAN_DISPLAY_OVL2:
	{
		struct pxafb_info *pxafb_this = (struct pxafb_info *)info;
		struct fb_var_screeninfo ovl2_var;
		int ret;

		ret = copy_from_user(&ovl2_var, (struct fb_var_screeninfo *)arg, sizeof(struct fb_var_screeninfo))? -EFAULT:0;
		if (ret)
			return ret;

		down(&pxafb_global_state.g_sem);
		val = overlay2fb_pan_display(&ovl2_var, 0, (struct fb_info *)(pxafb_this->overlay2fb));
		up(&pxafb_global_state.g_sem);
		
		return val;
	}
	case FBIOADJUST_TRANS:	/* Added  to support adjusting k1,k2,k3 */
	{
		unsigned short trans_value;
		int ret;

		ret = copy_from_user(&trans_value, (unsigned short *)arg, sizeof(unsigned short))? -EFAULT:0;
		if (ret)
			return ret;
		
		trans_value &= 0x01FF; //k3k2k1[8:0]

		down(&pxafb_global_state.g_sem);
		
		if ( !(pxafb_global_state.main_state & PXAFB_PANEL_ON)){ 
			  pxafb_global_state.trans_value = trans_value; /* It will be set when main panel is enable */
		}
		else{																		
			  pxafb_global_state.trans_value = trans_value ;
  		  	  LCCR4 = (LCCR4 & (~(0x01FF))) | trans_value ;  /* Set to new k3k2k1 value*/
		}

		up(&pxafb_global_state.g_sem);

		return 0;
	}
#if (0)
	case FBIOENABLEWP:
		wpaper_enabled = 1;
		break;
	case FBIODISABLEWP:
		wpaper_enabled = 0;
		break;
#endif
	}

	return -EINVAL;
}

#if CONFIG_FBBASE_DOUBLE_BUFFERS
int pxafb_2ndfb_init(struct pxafb_info *fbi)
{
	unsigned long bytes_per_panel;
	unsigned long secondfb_len;
	struct db_entry_s *db_entry;
	unsigned short bytes_per_pixel;

	if (fbi->double_buffers)
		return 0;
	
	switch(fbi->max_bpp)
	{
		case 16:
			bytes_per_pixel = 2;
			break;
		case 18:
		case 19:
			bytes_per_pixel = 3;
			break;
		case 24:
		case 25:
			bytes_per_pixel = 4;
			break;
		default:
			bytes_per_pixel = 1;
			break;
	}

	bytes_per_panel = fbi->max_xres * fbi->max_yres * bytes_per_pixel;
	secondfb_len = PAGE_ALIGN(bytes_per_panel + PAGE_SIZE);  //only be available for 16bpp//
	fbi->second_fb.map_cpu = __consistent_alloc(GFP_KERNEL, secondfb_len, &(fbi->second_fb.map_dma), PTE_BUFFERABLE);
	if (!(fbi->second_fb.map_cpu))
	{
DPRINTK("ENABLE2BFS:allocate second fb fail,fbi->double_buffers = 0\n");

		fbi->double_buffers = 0;
		return -ENOMEM;
	}

DPRINTK("consistent_alloc succeed, secondfb_len(%d), fbi->double_buffers(%d)\n", secondfb_len, fbi->double_buffers);

	memset((void *)fbi->second_fb.map_cpu, 0, secondfb_len);
				
	fbi->second_fb.screen_cpu = fbi->second_fb.map_cpu + PAGE_SIZE;
	fbi->second_fb.screen_dma = fbi->second_fb.map_dma + PAGE_SIZE;

	fbi->second_fb.dmadesc_fb_cpu = (struct pxafb_dma_descriptor *)(fbi->second_fb.screen_cpu -1*16);
	fbi->second_fb.dmadesc_fb_dma = fbi->second_fb.screen_dma - 1*16;

	fbi->second_fb.dmadesc_fb_cpu->fdadr = fbi->second_fb.dmadesc_fb_dma;
	fbi->second_fb.dmadesc_fb_cpu->fsadr = fbi->second_fb.screen_dma;
	fbi->second_fb.dmadesc_fb_cpu->fidr = 0;
	fbi->second_fb.dmadesc_fb_cpu->ldcmd = bytes_per_panel;

	db_entry = kmalloc(sizeof(struct db_entry_s),GFP_KERNEL);
	if (!db_entry)
		return -ENOSPC;

	//JUST FOR TESTING	db_entry->task = current; //just for testing
	db_entry->pid = current->pid;
	list_add(&(db_entry->task_list), &(fbi->db_task_list));

	//fbi->fb.fix.smem_len = PAGE_ALIGN(bytes_per_panel) * 2;
	//fbi->fb.var.yres_virtual = fbi->fb.var.yres *2 + 4;  // 320*2 + 4 is the number calculated by Peter, for indicating the second fb//
				
	fbi->double_buffers = 1;
	return 0;
}

int pxafb_pan_display(struct fb_var_screeninfo *var, int con, struct fb_info *info)
{
	unsigned long flags;
	
	struct pxafb_info *fbi = (struct pxafb_info *)(info);

	down(&pxafb_global_state.g_sem);
	
	/* Checking if LCD controller is turned off */
	if ( (fbi->state == C_DISABLE) || (fbi->state == C_DISABLE_CLKCHANGE) )
	{
		DPRINTK("LCD PANEL (%d) is off, just return -EAGAIN\n", info->modename[0]);
		up(&pxafb_global_state.g_sem);		
		return -EAGAIN;
	}
	
	/* Try to enters into framebuffer switching routine */
	
	DECLARE_WAITQUEUE(wait, current);

	if (var->yoffset == 0)  //switch to the first framebuffer
	{
		local_irq_save(flags);

		if (fbi->current_fb == &fbi->fb)  //base frame is exported to user space //
		{
			fbi->fb.var.yoffset = 0;  //mark that the first frame buffer is dedicated to LCD panel //
		//fbi->dmadesc_fbhigh_cpu = fbi->first_fb.dmadesc_fb_cpu;
		//fbi->dmadesc_fbhigh_cpu->fdadr = fbi->first_fb.dmadesc_fb_dma;
		//fbi->dmadesc_fbhigh_dma = fbi->first_fb.dmadesc_fb_dma;
			fbi->fdadr0 = fbi->first_fb.dmadesc_fb_dma;

			//br_duration = OSCR;
			FBR0 = fbi->fdadr0 | 0x3; //don't set LCSR0[BS0]  //BRA should be set, BRINT is disabled //
		}
		else  // if (fbi->current_fb != &fbi->fb) -- base frame buffer is _NOT_ exported to user space
		{
			fbi->current_fb->var.yoffset = 0;
		
		//fbi->dmadesc_fbhigh_cpu = fbi->first_fb.dmadesc_fb_cpu;
		//fbi->dmadesc_fbhigh_cpu->fdadr = fbi->first_fb.dmadesc_fb_dma;
		//fbi->dmadesc_fbhigh_dma = fbi->first_fb.dmadesc_fb_dma;
			fbi->fdadr1 = fbi->first_fb.dmadesc_fb_dma;

			//br_duration = OSCR;
			FBR1 = fbi->fdadr1 | 0x3; //Assuming overlay1fb is exported to user space. don't set LCSR0[BS0]  //BRA should be set, BRINT is disabled //
		}
		
		set_current_state(TASK_INTERRUPTIBLE);
		add_wait_queue(&fbi->ctrlr_wait, &wait);

		local_irq_restore(flags);
		
		/* method 1 for performance testing */
		//	mdelay(18);  //PCD is 9, so FCLK is 55Hz //
		
		/* method 2 for performance testing */
		schedule();
		remove_wait_queue(&fbi->ctrlr_wait, &wait);


		
#if (0)
DPRINTK("PAN_DISPLAY: switch to #1 framebuffer\n");
DPRINTK("fbi->screen_cpu = 0x%x\n", fbi->screen_cpu);
DPRINTK("fbi->screen_dma = 0x%x\n", fbi->screen_dma);
DPRINTK("fbi->dmadesc_fbhigh_cpu = 0x%x\n", fbi->dmadesc_fbhigh_cpu);
DPRINTK("fbi->dmadesc_fbhigh_dma = 0x%x\n", fbi->dmadesc_fbhigh_dma);
DPRINTK("fbi->fdadr0 = 0x%x\n", fbi->fdadr0);
DPRINTK("FBR0 = 0x%x\n", FBR0);
DPRINTK("FDADR0 = 0x%x\n", FDADR0);
#endif
		if (fbi->current_fb == &fbi->fb)  //base frame is exported to user space //
		{
		    DPRINTK("fbi->first_fb.screen_dma = 0x%x\n", fbi->first_fb.screen_dma);
		    DPRINTK("fbi->first_fb.dmadesc_fb_dma = 0x%x\n", fbi->first_fb.dmadesc_fb_dma);
		    DPRINTK("fbi->fdadr0 = 0x%x\n", fbi->fdadr0);
		    DPRINTK("FBR0 = 0x%x\n", FBR0);
		    DPRINTK("FDADR0 = 0x%x\n", FDADR0);
		}
		else  //if (fbi->current_fb != &fbi->fb) -- assuming overlay1 framebuffer is exported to user space //
		{
		    DPRINTK("fbi->overlay1fb->dma1->fsadr = 0x%x\n", fbi->first_fb.screen_dma);
		    DPRINTK("fbi->overlay1fb->dma1->fdadr0= 0x%x\n", fbi->first_fb.dmadesc_fb_dma);
		    DPRINTK("fbi->fdadr1 = 0x%x\n", fbi->fdadr1);
		    DPRINTK("FBR1 = 0x%x\n", FBR1);
		    DPRINTK("FDADR1 = 0x%x\n", FDADR1);
		}
			
	}

	if (var->yoffset >= fbi->max_yres)  //switch to the second framebuffer
	{
		unsigned short bytes_per_pixel;
		switch(var->bits_per_pixel)  //Note that pxafb_info.max_bpp may be not equal to var.bits_per_pixel //
		{
			case  16:
				bytes_per_pixel = 2;
				break;
			case 18:
			case 19:
				bytes_per_pixel = 3;
				break;
			case 24:
			case 25:
				bytes_per_pixel = 4;
				break;
			default:
				bytes_per_pixel = 1;
				break;
		}
	
		//DPRINTK("LCD panel shifts to the second framebuffer\n");
		local_irq_save(flags);

		if (fbi->current_fb == &fbi->fb)  //base framebuffer is exported to user space //
		{
			fbi->fb.var.yoffset = fbi->max_yres + ( (bytes_per_pixel & 0x3)? 4: 0);  //mark that the first frame buffer is dedicated to LCD panel //
		//fbi->dmadesc_fbhigh_cpu = fbi->first_fb.dmadesc_fb_cpu;
		//fbi->dmadesc_fbhigh_cpu->fdadr = fbi->first_fb.dmadesc_fb_dma;
		//fbi->dmadesc_fbhigh_dma = fbi->first_fb.dmadesc_fb_dma;
			fbi->fdadr0 = fbi->second_fb.dmadesc_fb_dma;

			//br_duration = OSCR;
			FBR0 = fbi->fdadr0 | 0x3; //don't set LCSR0[BS0]  //BRA should be set, BRINT is disabled //
		}
		else  // if (fbi->current_fb != &fbi->fb -- base framebuffer is NOT exported to user space, assuming it is overlay1fb //
		{
			fbi->current_fb->var.yoffset = fbi->max_yres + ( (bytes_per_pixel & 0x3)? 4: 0);
		
		//fbi->dmadesc_fbhigh_cpu = fbi->first_fb.dmadesc_fb_cpu;
		//fbi->dmadesc_fbhigh_cpu->fdadr = fbi->first_fb.dmadesc_fb_dma;
		//fbi->dmadesc_fbhigh_dma = fbi->first_fb.dmadesc_fb_dma;
			fbi->fdadr1 = fbi->second_fb.dmadesc_fb_dma;

			//br_duration = OSCR;
			FBR1 = fbi->fdadr1 | 0x3; //don't set LCSR0[BS0]  //BRA should be set, BRINT is disabled //
		}

		set_current_state(TASK_INTERRUPTIBLE);
		add_wait_queue(&fbi->ctrlr_wait, &wait);

		local_irq_restore(flags);
		
		/* The first method for performance comparing */
		//	mdelay(18);
		
		/* The second method */
		schedule();
		remove_wait_queue(&fbi->ctrlr_wait, &wait);


#if (0)
DPRINTK("switch to #2 framebuffer\n");
DPRINTK("fbi->dmadesc_fbhigh_cpu = 0x%x\n", fbi->dmadesc_fbhigh_cpu);
DPRINTK("fbi->dmadesc_fbhigh_dma = 0x%x\n", fbi->dmadesc_fbhigh_dma);
DPRINTK("fbi->fdadr0 = 0x%x\n", fbi->fdadr0);
DPRINTK("FBR0 = 0x%x\n", FBR0);
DPRINTK("FDADR0 = 0x%x\n", FDADR0);
#endif
		if (fbi->current_fb == &fbi->fb)  //base framebuffer is exported to user space //
		{
		    DPRINTK("fbi->second_fb.screen_dma = 0x%x\n", fbi->second_fb.screen_dma);
		    DPRINTK("fbi->second_fb.dmadesc_fb_dma = 0x%x\n", fbi->second_fb.dmadesc_fb_dma);
		    DPRINTK("fbi->fdadr0 = 0x%x\n", fbi->fdadr0);
		    DPRINTK("FBR0 = 0x%x\n", FBR0);
		    DPRINTK("FDADR0 = 0x%x\n", FDADR0);
		}
		else  //if (fbi->current_fb != &fbi->fb) -- Assuming overlay1fb is exported to user space //
		{
		    DPRINTK("fbi->second_fb.screen_dma = 0x%x\n", fbi->second_fb.screen_dma);
		    DPRINTK("fbi->second_fb.dmadesc_fb_dma = 0x%x\n", fbi->second_fb.dmadesc_fb_dma);
		    DPRINTK("fbi->fdadr1 = 0x%x\n", fbi->fdadr1);
		    DPRINTK("FBR1 = 0x%x\n", FBR1);
		    DPRINTK("FDADR1 = 0x%x\n", FDADR1);
		}
	}

	up(&pxafb_global_state.g_sem);

	return 0;
}

int pxafb_release(struct fb_info *info, int user)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	unsigned int flags = 0;
	struct db_entry_s *tmp_entry;
	struct list_head *tmp;
	unsigned short bytes_per_pixel;

	down(&pxafb_global_state.g_sem);
	
	switch(fbi->max_bpp)
	{
		case 16:
			bytes_per_pixel = 2;
			break;
		case 18:
		case 19:
			bytes_per_pixel = 3;
			break;
		case 24:
		case 25:
			bytes_per_pixel = 4;
			break;
		default:
			bytes_per_pixel = 1;
			break;
	}	

DPRINTK("PXAFB_RELEASE: current(0x%x--%s), EMPTY list(%d)\n", current, current->comm,list_empty(&(fbi->db_task_list)) );

	if ( list_empty(&(fbi->db_task_list)) ) //Nobody is using LCD double buffers//
	{
		up(&pxafb_global_state.g_sem);
		return 0;
	}
	else
	{
		tmp = fbi->db_task_list.next;
		while ( tmp != &(fbi->db_task_list) )
		{
			tmp_entry = list_entry(tmp,struct db_entry_s,task_list);
			tmp = tmp->next;

DPRINTK("DISABLE2BFS:current(%s-%d),task(%d),task_list(0x%x)\n",current->comm,current->pid,tmp_entry->pid,&(tmp_entry->task_list));

			if (tmp_entry->pid == current->pid)
			{
DPRINTK("DISABLE2BFS:tmp_entry->task(%d) will be removed\n",tmp_entry->pid);
			    list_del(&tmp_entry->task_list);
				kfree(tmp_entry);

				/* If db_task_list contains only one entry, switch to #1 framebuffer */
				if ( (tmp != &(fbi->db_task_list)) && (tmp->next == tmp->prev) )
				{
					DPRINTK("PXAFB_RELEASE: current(%s-%d),resume 1# framebuffer\n", current->comm,current->pid);

					/* switch to the first framebuffer */
					if (fbi->current_fb->var.yoffset >= fbi->max_yres)  // The second fb is used for LCD panel //
					{
						if (fbi->current_fb == &fbi->fb) //base frame buffer is exported to user space //
						{
							local_irq_save(flags);

							fbi->fb.var.yoffset = 0;
							fbi->dmadesc_fbhigh_cpu = fbi->first_fb.dmadesc_fb_cpu;
							fbi->dmadesc_fbhigh_cpu->fdadr = fbi->first_fb.dmadesc_fb_dma;
							fbi->dmadesc_fbhigh_dma = fbi->first_fb.dmadesc_fb_dma;
							fbi->fdadr0 = fbi->dmadesc_fbhigh_dma;
				
							FBR0 = fbi->fdadr0 | 0x01; //don't generate interrupt bit LCSR0[BS0] //
							
							local_irq_restore(flags);
							
							mdelay(18);  //PCD is 9, frame freq = 55 Frames/Second //
						}
						else  // if (fbi->current_fb != &fbi->fb) -- Assuming overlay1fb is exported to user space //
						{
							local_irq_save(flags);
							
							fbi->current_fb->var.yoffset = 0;
							fbi->fdadr1 = fbi->first_fb.dmadesc_fb_dma;
					
							FBR1 = fbi->fdadr1 | 0x01; //don't generate interrupt bit LCSR1[BS1] //
							
							local_irq_restore(flags);
							
							mdelay(18);  //PCD is 9, frame freq = 55 Frames/Second //
						}							
					}
				}							
			}
		}

		if ( list_empty(&(fbi->db_task_list)) )
		{
			/* release second framebuffer -- 152KB + PAGE_SIZE */
			consistent_free((void *)fbi->second_fb.map_cpu, fbi->map_size, fbi->second_fb.map_dma);

			fbi->second_fb.map_cpu = (u_char *)NULL;
			fbi->second_fb.map_dma = (dma_addr_t)NULL;
			fbi->second_fb.dmadesc_fb_cpu = (struct pxafb_dma_descriptor *)NULL;
			fbi->second_fb.dmadesc_fb_dma = (dma_addr_t)NULL;
			fbi->second_fb.screen_cpu = (u_char *)NULL;
			fbi->second_fb.screen_dma = (dma_addr_t)NULL;

DPRINTK("PXAFB_RELEASE:double_buffers = 0\n");

			fbi->double_buffers = 0;
			fbi->current_fb->fix.smem_len = fbi->max_xres * fbi->max_yres * bytes_per_pixel;
			fbi->current_fb->var.yres_virtual = fbi->max_yres;
			fbi->current_fb->var.yoffset = 0;
		}
		up(&pxafb_global_state.g_sem);
	}
	return 0;
}

void pxafb_db_proc(unsigned long *fb_data)
{
	fb_data[0] = ((pxafb_main->current_fb == &pxafb_main->fb)? 0x1:0x0);
	fb_data[1] = (unsigned long)(pxafb_main->current_fb->fix.smem_len);
	fb_data[2] = (unsigned long)(pxafb_main->current_fb->var.yres_virtual);
	fb_data[3] = (unsigned long)(pxafb_main->current_fb->var.yoffset);
	fb_data[4] = (unsigned long)(pxafb_main->first_fb.dmadesc_fb_dma);
	fb_data[5] = (unsigned long)(pxafb_main->first_fb.screen_dma);
	fb_data[6] = (unsigned long)(pxafb_main->second_fb.dmadesc_fb_dma);
	fb_data[7] = (unsigned long)(pxafb_main->second_fb.screen_dma);
#ifdef CONFIG_PXAFB_CLI
	fb_data[8] = ((pxafb_smart->current_fb == &pxafb_smart->fb)? 0x1:0x0);
	fb_data[9] = (unsigned long)(pxafb_smart->current_fb->fix.smem_len);
	fb_data[10] = ( unsigned long)(pxafb_smart->current_fb->var.yres_virtual);
	fb_data[11] = (unsigned long)(pxafb_smart->current_fb->var.yoffset);
	fb_data[12] = (unsigned long)(pxafb_smart->first_fb.dmadesc_fb_dma);
	fb_data[13] = (unsigned long)(pxafb_smart->first_fb.screen_dma);
	fb_data[14] = (unsigned long)(pxafb_smart->second_fb.dmadesc_fb_dma);
	fb_data[15] = (unsigned long)(pxafb_smart->second_fb.screen_dma);
#endif
}
#endif

static struct fb_ops pxafb_ops = {
	owner:		THIS_MODULE,
	fb_get_fix:	pxafb_get_fix,
	fb_get_var:	pxafb_get_var,
	fb_set_var:	pxafb_set_var,
	fb_get_cmap:	pxafb_get_cmap,
	fb_set_cmap:	pxafb_set_cmap,
#if CONFIG_FBBASE_DOUBLE_BUFFERS
	fb_release:  pxafb_release,
	fb_pan_display:  pxafb_pan_display,
#endif
	fb_ioctl:   pxafb_ioctl,
};

/*
 *  pxafb_switch():       
 *	Change to the specified console.  Palette and video mode
 *      are changed to the console's stored parameters.
 *
 *	Uh oh, this can be called from a tasklet (IRQ)
 */
#if (0)
static int pxafb_switch(int con, struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	struct display *disp;
	struct fb_cmap *cmap;

	DPRINTK("con=%d info->modename=%d", con, fbi->current_fb->modename[0]);

	if (con == fbi->currcon)
		return 0;

	if (fbi->currcon >= 0) {
		disp = fb_display + fbi->currcon;

		/*
		 * Save the old colormap and video mode.
		 */
		disp->var = fbi->current_fb->var;

		if (disp->cmap.len)
			fb_copy_cmap(&fbi->current_fb->cmap, &disp->cmap, 0);
	}

	fbi->currcon = con;
	disp = fb_display + con;

	/*
	 * Make sure that our colourmap contains 256 entries.
	 */
	fb_alloc_cmap(&fbi->current_fb->cmap, 256, 0);

	if (disp->cmap.len)
		cmap = &disp->cmap;
	else
		cmap = fb_default_cmap(1 << disp->var.bits_per_pixel);

	fb_copy_cmap(cmap, &fbi->current_fb->cmap, 0);

	fbi->current_fb->var = disp->var;
	fbi->current_fb->var.activate = FB_ACTIVATE_NOW;

	pxafb_set_var(&fbi->current_fb->var, con, info);
	return 0;
}
#endif

/*
 * Formal definition of the VESA spec:
 *  On
 *  	This refers to the state of the display when it is in full operation
 *  Stand-By
 *  	This defines an optional operating state of minimal power reduction with
 *  	the shortest recovery time
 *  Suspend
 *  	This refers to a level of power management in which substantial power
 *  	reduction is achieved by the display.  The display can have a longer 
 *  	recovery time from this state than from the Stand-by state
 *  Off
 *  	This indicates that the display is consuming the lowest level of power
 *  	and is non-operational. Recovery from this state may optionally require
 *  	the user to manually power on the monitor
 *
 *  Now, the fbdev driver adds an additional state, (blank), where they
 *  turn off the video (maybe by colormap tricks), but don't mess with the
 *  video itself: think of it semantically between on and Stand-By.
 *
 *  So here's what we should do in our fbdev blank routine:
 *
 *  	VESA_NO_BLANKING (mode 0)	Video on,  front/back light on
 *  	VESA_VSYNC_SUSPEND (mode 1)  	Video on,  front/back light off
 *  	VESA_HSYNC_SUSPEND (mode 2)  	Video on,  front/back light off
 *  	VESA_POWERDOWN (mode 3)		Video off, front/back light off
 *
 *  This will match the matrox implementation.
 */
/*
 * pxafb_blank():
 *	Blank the display by setting all palette values to zero.  Note, the 
 * 	12 and 16 bpp modes don't really use the palette, so this will not
 *      blank the display in all modes.  
 */
static void pxafb_blank(int blank, struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	int i;

	DPRINTK("pxafb_blank(%s): blank(%d),info->modename[0](%d),fbi->current_fb->modename[0](%d)\n",current->comm, blank,info->modename[0],fbi->current_fb->modename[0]);

	if (info->modename[0] < PXAFB_CLI_BASE)  //This is pxafb_main->fb //
	{
		switch (blank) {
		case VESA_POWERDOWN:
		case VESA_VSYNC_SUSPEND:
		case VESA_HSYNC_SUSPEND:
		  /* blank the screen */
		  	if (fbi->fb.disp->visual == FB_VISUAL_PSEUDOCOLOR || fbi->fb.disp->visual == FB_VISUAL_STATIC_PSEUDOCOLOR)
		     	for (i = 0; i < fbi->palette_size; i++)
			       pxafb_setpalettereg(i, 0, 0, 0, 0, info);

	#if 1
		  /* is this the proper time to call pxafb_blank_helper() */
		  	if (pxafb_blank_helper)
		    pxafb_blank_helper(blank);
  		  	
			/* if DPM is in use, need to register the fact that the driver disabled the console */
		     
			/* Disable LCD controller for pxafb_main panel */
			DPRINTK("pxafb_blank:device_powerdown(&pxafb_main_device_ldm)\n");
			if (pxafb_main_device_ldm.driver)
				DPRINTK("pxafb_blank:pxafb_main_device_ldm.driver(0x%x)\n",pxafb_main_device_ldm.driver);
			else
				DPRINTK("pxafb_blank:pxafb_main_device_ldm.driver(NULL)\n");

		  	device_powerdown(&pxafb_main_device_ldm);		  	
	#else
		  pxafb_schedule_task(fbi, C_DISABLE);
		  if (pxafb_blank_helper)
	            pxafb_blank_helper(blank);
	#endif
		  	break;

		case VESA_NO_BLANKING:
			if (pxafb_blank_helper)
			    pxafb_blank_helper(blank);
			if (fbi->fb.disp->visual == FB_VISUAL_PSEUDOCOLOR ||fbi->fb.disp->visual == FB_VISUAL_STATIC_PSEUDOCOLOR)
			    fb_set_cmap(&fbi->current_fb->cmap, 1, pxafb_setcolreg, info);

	#if 1 /* MVL-CEE */
			/* power up pxafb_main panel */
			device_powerup(&pxafb_main_device_ldm);
	#else
		  pxafb_schedule_task(fbi, C_ENABLE);
	#endif
		  break;
		}
	}
#ifdef CONFIG_PXAFB_CLI
	else if (info->modename[0] >= PXAFB_CLI_BASE)
	{
		switch (blank) 
		{
		case VESA_POWERDOWN:
			/* blank the screen */
			if (fbi->fb.disp->visual == FB_VISUAL_PSEUDOCOLOR ||fbi->fb.disp->visual == FB_VISUAL_STATIC_PSEUDOCOLOR)
				for (i = 0; i < fbi->palette_size; i++)
					pxafb_setpalettereg(i, 0, 0, 0, 0, info);
	#if 1
			  /* is this the proper time to call pxafb_blank_helper() */
			if (pxafb_blank_helper)
			    pxafb_blank_helper(blank);
		  
			/* if DPM is in use, need to register the fact that the driver disabled the console */
			/* Disable LCD controller from pxafb_smart CLI STN panel */
			DPRINTK("pxafb_blank:device_powerdown(&pxafb_smart_device_ldm)\n");
			if (pxafb_smart_device_ldm.driver)
				DPRINTK("pxafb_blank:pxafb_smart_device_ldm.driver(0x%x)\n",pxafb_smart_device_ldm.driver);
			else
				DPRINTK("pxafb_blank:pxafb_smart_device_ldm.driver(NULL)\n");

			device_powerdown(&pxafb_smart_device_ldm);		  	
	#else
		  pxafb_schedule_task(fbi, C_DISABLE);
		  if (pxafb_blank_helper)
	            pxafb_blank_helper(blank);
	#endif
			break;
		case VESA_VSYNC_SUSPEND:
		case VESA_HSYNC_SUSPEND:
			/* Disable LCD controller from pxafb_smart CLI STN panel */
			DPRINTK("pxafb_blank:device_suspend(&pxafb_smart_device_ldm)\n");
			device_powersuspend(&pxafb_smart_device_ldm);

			break;
		case VESA_NO_BLANKING:
			if (pxafb_blank_helper)
				pxafb_blank_helper(blank);
			if (fbi->fb.disp->visual == FB_VISUAL_PSEUDOCOLOR ||fbi->fb.disp->visual == FB_VISUAL_STATIC_PSEUDOCOLOR)
				fb_set_cmap(&fbi->current_fb->cmap, 1, pxafb_setcolreg, info);
	#if 1
			/* Power up pxafb_smart CSTN panel */
			device_powerup(&pxafb_smart_device_ldm);
	#else
		  pxafb_schedule_task(fbi, C_ENABLE);
	#endif
		  break;
		}
	}
#endif

}

static int pxafb_updatevar(int con, struct fb_info *info)
{
	DPRINTK("entered");
	return 0;
}

#if defined(CONFIG_CPU_BULVERDE)
static inline int get_lcd_clock_10khz( void )
{
        unsigned int L;
        unsigned int A;
        unsigned int a = 0;

        /* NOTE: calculating the clock from L might be easier than
           maintaining this array. For example:

           int k=1;
           if (L > 16) k = 4;
           else if (L > 7) k = 2;
           return (L * 1300)/k);
        */

        static int lclk_10khz[2][32] = {
                {0, 0, 2600, 3900, 5200, 6500, 7800, 9100, 5200, 5850,
                 6500, 7150, 7800, 8450, 9100, 9750, 5200, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0, 9425, 9750, 10075},
                {0, 0, 2600, 3900, 5200, 6500, 7800, 9100, 5200, 5850,
                 6500, 7150, 7800, 8450, 9100, 9750, 10400, 5525, 5850,
                 6225, 6500, 6825, 7150, 7475, 7800, 8125, 8450, 8775,
                 9100, 9425, 9750, 10075}
        };

        L = (CCCR&0x1f);
        A = CCCR&0x02000000;
        if (A)  a = 1;
        return  lclk_10khz[a][L];

}
#endif

/*
 * Calculate the PCD value from the clock rate (in picoseconds).
 * We take account of the PPCR clock setting.
 */
static inline int get_pcd(unsigned int pixclock)
{
	unsigned int pcd;

	if (pixclock) {
#if defined(CONFIG_CPU_BULVERDE)
		DPRINTK("pixclock = %d\n", pixclock );		
		pcd = (get_lcd_clock_10khz() * pixclock);
		pcd /= 100000000;
		if (pxafb_main->reg_lccr4 & 0x80000000)
			pcd = pcd - 1;	 
		else
			pcd = pcd/2 - 1;
			
		if ( pcd <= 0 ) pcd = 1;
		DPRINTK("lcd clock = %d\n", get_lcd_clock_10khz() );
		DPRINTK("pcd = %d\n", pcd );
#else
		pcd = get_memclk_frequency_10khz() * pixclock;
		pcd /= 100000000;
		pcd += 1;	/* make up for integer math truncations */
#endif
	} else {
		//printk(KERN_WARNING "Please convert me to use the PCD calculations\n");
		pcd = 0;
	}
	return pcd;
}

static inline int pxafb_bpp_bits(u_char bpp)
{
	switch (bpp) {
	case 2:
		return 0x1;
	case 4:
		return 0x2;
	case 8:
		return 0x3;
	case 16:
		return 0x4;
#ifdef CONFIG_CPU_BULVERDE
	case 18:
		return 0x6;  /* packed pixel format */
	case 19:
		return 0x8;  /* packed pixel format */
	case 24:
		return 0x9;
	case 25:
		return 0xa;
#endif
	default:
		printk(KERN_ERR "PXA FB: unsupported bpp mode %u\n", bpp);
		return 0x4;
	}
}

/*
 * pxafb_activate_var():
 *	Configures LCD Controller based on entries in var parameter.  Settings are      
 *      only written to the controller if changes were made.  
 */
static int pxafb_activate_var(struct fb_var_screeninfo *var, struct pxafb_info *fbi)
{
	struct pxafb_lcd_reg new_regs;
#ifdef CONFIG_CPU_BULVERDE	
	u_int pcd = get_pcd(var->pixclock);
#endif
	u_int bytes_per_panel;
	u_long flags = 0;
	u_int bytes_palette = 0;


	DPRINTK("Configuring PXA LCD");

	DPRINTK("var: xres=%d hslen=%d lm=%d rm=%d",
		var->xres, var->hsync_len,
		var->left_margin, var->right_margin);
	DPRINTK("var: yres=%d vslen=%d um=%d bm=%d",
		var->yres, var->vsync_len,
		var->upper_margin, var->lower_margin);

#ifdef DEBUG_VAR
	if (var->xres < 16        || var->xres > 1024)
		printk(KERN_ERR "%s: invalid xres %d\n",
			fbi->current_fb->fix.id, var->xres);
	if (var->hsync_len < 1    || var->hsync_len > 64)
		printk(KERN_ERR "%s: invalid hsync_len %d\n",
			fbi->current_fb->fix.id, var->hsync_len);
	if (var->left_margin < 1  || var->left_margin > 255)
		printk(KERN_ERR "%s: invalid left_margin %d\n",
			fbi->current_fb->fix.id, var->left_margin);
	if (var->right_margin < 1 || var->right_margin > 255)
		printk(KERN_ERR "%s: invalid right_margin %d\n",
			fbi->current_fb->fix.id, var->right_margin);
	if (var->yres < 1         || var->yres > 1024)
		printk(KERN_ERR "%s: invalid yres %d\n",
			fbi->current_fb->fix.id, var->yres);
	if (var->vsync_len < 1    || var->vsync_len > 64)
		printk(KERN_ERR "%s: invalid vsync_len %d\n",
			fbi->current_fb->fix.id, var->vsync_len);
	if (var->upper_margin < 0 || var->upper_margin > 255)
		printk(KERN_ERR "%s: invalid upper_margin %d\n",
			fbi->current_fb->fix.id, var->upper_margin);
	if (var->lower_margin < 0 || var->lower_margin > 255)
		printk(KERN_ERR "%s: invalid lower_margin %d\n",
			fbi->current_fb->fix.id, var->lower_margin);
#endif /* DEBUG_VAR */

	if (fbi == pxafb_main)
	{
#if defined (CONFIG_CPU_BULVERDE) 
	new_regs.lccr0 = fbi->reg_lccr0;
	new_regs.lccr1 = LCCR1_BegLnDel(var->left_margin) | LCCR1_EndLnDel(var->right_margin)
		| LCCR1_HorSnchWdth(var->hsync_len) | LCCR1_DisWdth(var->xres);
	new_regs.lccr2 = LCCR2_VrtSnchWdth(var->vsync_len) | LCCR2_BegFrmDel(var->upper_margin)
		| LCCR2_EndFrmDel(var->lower_margin) | LCCR2_DisHght(var->yres);
	new_regs.lccr3 = fbi->reg_lccr3 | pcd | LCCR3_Bpp(pxafb_bpp_bits(var->bits_per_pixel));
	new_regs.lccr4 = fbi->reg_lccr4;
	new_regs.lccr5 = 0x0;

	/* This is the Intel tricky setting for 24bpp->18bpp HW convertion */
	if ( (fbi->current_fb->modename[0] < PXAFB_CLI_BASE) && (PXAFB_MAIN_VALID_FB == PXAFB_MAIN_OVL1) )
	{   
		/* LCCR3[29,26:24] = "0000"
		LCCR3[PDFOR] = "11"
		LCCR4[PAL_FOR]="10"
		*/
		new_regs.lccr3 &= ~(0x27<<24); //LCCR3[PDFOR]
		new_regs.lccr3 |= LCCR3_PDFOR_3 | LCCR3_Bpp(pxafb_bpp_bits(2));
	}

#elif defined (CONFIG_PXA_CERF_PDA)
	new_regs.lccr0 = fbi->reg_lccr0;
	new_regs.lccr1 =
		LCCR1_DisWdth(var->xres) +
		LCCR1_HorSnchWdth(var->hsync_len) +
		LCCR1_BegLnDel(var->left_margin) +
		LCCR1_EndLnDel(var->right_margin);
		
	new_regs.lccr2 =
		LCCR2_DisHght(var->yres) +
		LCCR2_VrtSnchWdth(var->vsync_len) +
		LCCR2_BegFrmDel(var->upper_margin) +
		LCCR2_EndFrmDel(var->lower_margin);

	new_regs.lccr3 = fbi->reg_lccr3
		|
		(var->sync & FB_SYNC_HOR_HIGH_ACT ? LCCR3_HorSnchH : LCCR3_HorSnchL) |
		(var->sync & FB_SYNC_VERT_HIGH_ACT ? LCCR3_VrtSnchH : LCCR3_VrtSnchL);
#elif defined (CONFIG_FB_PXA_QVGA)
	new_regs.lccr0 = fbi->reg_lccr0;
	new_regs.lccr1 = LCCR1_BegLnDel(var->left_margin) 
		       | LCCR1_EndLnDel(var->right_margin) 
		       | LCCR1_HorSnchWdth(var->hsync_len) 
		       | LCCR1_DisWdth(var->xres);
	new_regs.lccr2 = LCCR2_VrtSnchWdth(var->vsync_len) 
		       | LCCR2_BegFrmDel(var->upper_margin)
		       | LCCR2_EndFrmDel(var->lower_margin) 
		       | LCCR2_DisHght(var->yres);
	new_regs.lccr3 = fbi->reg_lccr3 
		       | pcd 
		       | LCCR3_Bpp(pxafb_bpp_bits(var->bits_per_pixel));
#else
	/* FIXME using hardcoded values for now */
	new_regs.lccr0 = fbi->reg_lccr0;
        /*	|
		LCCR0_LEN | LCCR0_LDM | LCCR0_BAM |
		LCCR0_ERM | LCCR0_LtlEnd | LCCR0_DMADel(0); */

	new_regs.lccr1 = 0x3030A7F;
        /*	LCCR1_DisWdth(var->xres) +
		LCCR1_HorSnchWdth(var->hsync_len) +
		LCCR1_BegLnDel(var->left_margin) +
		LCCR1_EndLnDel(var->right_margin); */

	new_regs.lccr2 = 0x4EF;
	/*      | LCCR2_DisHght(var->yres) +
		LCCR2_VrtSnchWdth(var->vsync_len) +
		LCCR2_BegFrmDel(var->upper_margin) +
		LCCR2_EndFrmDel(var->lower_margin); */

	new_regs.lccr3 = fbi->reg_lccr3;
	/*      |
		(var->sync & FB_SYNC_HOR_HIGH_ACT ? LCCR3_HorSnchH : LCCR3_HorSnchL) |
		(var->sync & FB_SYNC_VERT_HIGH_ACT ? LCCR3_VrtSnchH : LCCR3_VrtSnchL) |
		LCCR3_ACBsCntOff; */
#endif
	}

	/* if (pcd)
	           new_regs.lccr3 |= LCCR3_PixClkDiv(pcd); */

	DPRINTK("LCCR0:0x%x LCCR1:0x%x LCCR2:0x%x LCCR3:0x%x LCCR4 0x%x LCCR5 0x%x\n", new_regs.lccr0, new_regs.lccr1, new_regs.lccr2, new_regs.lccr3,new_regs.lccr4,new_regs.lccr5);

	/* We don't support dynamically changing pixel depth format(bpp format) -- Susan */
	if (pxafb_global_state.init_phase)
	{
		/* During initialization, we should set base frame dma channel but not other channels --Susan */
		/* Update shadow copy atomically */
		local_irq_save(flags);

		/* Setting up DMA descriptors */
		fbi->dmadesc_fblow_cpu = (struct pxafb_dma_descriptor *)((unsigned int)fbi->palette_cpu - 3*16);
		fbi->dmadesc_fbhigh_cpu = (struct pxafb_dma_descriptor *)((unsigned int)fbi->palette_cpu - 2*16);
		fbi->dmadesc_palette_cpu = (struct pxafb_dma_descriptor *)((unsigned int)fbi->palette_cpu - 1*16);

		fbi->dmadesc_fblow_dma = fbi->palette_dma - 3*16;
		fbi->dmadesc_fbhigh_dma = fbi->palette_dma - 2*16;
		fbi->dmadesc_palette_dma = fbi->palette_dma - 1*16;

		if (var->bits_per_pixel < 16) 
		{
		        u_int pixels_per_line = 0;
		        u_int cnt = 0;
		        u32 * palette_data = (u32 *)fbi->palette_cpu;

				/* Each line in the memory must start at a 32-bit word boundary */
		        pixels_per_line = (var->xres + ( 32 / var->bits_per_pixel - 1)) & (~(32 / var->bits_per_pixel -1));
				bytes_per_panel = pixels_per_line * var->bits_per_pixel * var->yres / 8 ;
		        fbi->palette_size = 1 << var->bits_per_pixel;
	                /* FIXME: bytes_palette depends on LCCR4[PAL_FOR], here we use 25bits */
				bytes_palette = fbi->palette_size * sizeof(u32);
		        /* Set the transparent bit */
		        for (cnt = 0; cnt < fbi->palette_size; cnt++)
					*palette_data++ = 1 << 24;
		}
		else if (var->bits_per_pixel == 16)  
			bytes_per_panel = var->xres * var->yres * var->bits_per_pixel / 8;
		else if (var->bits_per_pixel > 19)
			bytes_per_panel = var->xres * var->yres * 4;
		else
			/* 18bpp and 19bpp, packed pixel format */
			bytes_per_panel = var->xres * var->yres * 3;

		/* We don't support dual-panel scan (a potential bug exists in below section) -- Susan */
		#if (0)
		if ( fbi->reg_lccr0 & LCCR0_SDS )
			bytes_per_panel >>= 1;
		
		/* Populate descriptors */
		fbi->dmadesc_fblow_cpu->fdadr = fbi->dmadesc_fblow_dma;
		fbi->dmadesc_fblow_cpu->fsadr = fbi->screen_dma + bytes_per_panel;  //here
		fbi->dmadesc_fblow_cpu->fidr  = 0;
		fbi->dmadesc_fblow_cpu->ldcmd = bytes_per_panel;

		fbi->fdadr1 = fbi->dmadesc_fblow_dma; /* only used in dual-panel mode */
		#endif
		
		fbi->dmadesc_fbhigh_cpu->fsadr = fbi->screen_dma;
		fbi->dmadesc_fbhigh_cpu->fidr = 0;
		fbi->dmadesc_fbhigh_cpu->ldcmd = bytes_per_panel;
		/* Enable EOFINT bit */
		fbi->dmadesc_fbhigh_cpu->ldcmd |= LDCMD_EOFINT;

		fbi->dmadesc_palette_cpu->fsadr = fbi->palette_dma;
		fbi->dmadesc_palette_cpu->fidr  = 0;
		fbi->dmadesc_palette_cpu->ldcmd = bytes_palette | LDCMD_PAL;

		if( var->bits_per_pixel < 12)
		{
			/* Assume any mode with <12 bpp is palette driven */
			fbi->dmadesc_palette_cpu->fdadr = fbi->dmadesc_fbhigh_dma;
			fbi->dmadesc_fbhigh_cpu->fdadr = fbi->dmadesc_palette_dma;
			fbi->fdadr0 = fbi->dmadesc_palette_dma; /* flips back and forth between pal and fbhigh */
		}
		else
		{
			/* Palette shouldn't be loaded in true-color mode ! */
			fbi->dmadesc_fbhigh_cpu->fdadr = fbi->dmadesc_fbhigh_dma;
			fbi->fdadr0 = fbi->dmadesc_fbhigh_dma; /* no pal just fbhigh */
		}

		DPRINTK("fbi->dmadesc_fblow_cpu = 0x%x\n", fbi->dmadesc_fblow_cpu);
		DPRINTK("fbi->dmadesc_fblow_dma = 0x%x\n", fbi->dmadesc_fblow_dma);
		DPRINTK("fbi->dmadesc_fblow_cpu->fdadr = 0x%x\n", fbi->dmadesc_fblow_cpu->fdadr);
		DPRINTK("fbi->dmadesc_fblow_cpu->fsadr = 0x%x\n", fbi->dmadesc_fblow_cpu->fsadr);
		DPRINTK("fbi->dmadesc_fblow_cpu->ldcmd = 0x%x\n", fbi->dmadesc_fblow_cpu->ldcmd);
		
		DPRINTK("fbi->dmadesc_fbhigh_cpu = 0x%x\n", fbi->dmadesc_fbhigh_cpu);
		DPRINTK("fbi->dmadesc_fbhigh_dma = 0x%x\n", fbi->dmadesc_fbhigh_dma);
		DPRINTK("fbi->dmadesc_fbhigh_cpu->fdadr = 0x%x\n", fbi->dmadesc_fbhigh_cpu->fdadr);
		DPRINTK("fbi->dmadesc_fbhigh_cpu->fsadr = 0x%x\n", fbi->dmadesc_fbhigh_cpu->fsadr);
		DPRINTK("fbi->dmadesc_fbhigh_cpu->ldcmd = 0x%x\n", fbi->dmadesc_fbhigh_cpu->ldcmd);
		
		DPRINTK("fbi->dmadesc_palette_cpu = 0x%x\n", fbi->dmadesc_palette_cpu);
		DPRINTK("fbi->dmadesc_palette_dma = 0x%x\n", fbi->dmadesc_palette_dma);
		DPRINTK("fbi->dmadesc_palette_cpu->fdadr = 0x%x\n", fbi->dmadesc_palette_cpu->fdadr);
		DPRINTK("fbi->dmadesc_palette_cpu->fsadr = 0x%x\n", fbi->dmadesc_palette_cpu->fsadr);
		DPRINTK("fbi->dmadesc_palette_cpu->ldcmd = 0x%x\n", fbi->dmadesc_palette_cpu->ldcmd);
		
		DPRINTK("fbi->fdadr0 = 0x%x\n", fbi->fdadr0);
		
	} //pxafb_global_state.init_phase != 0 -- we are in INIT phase //
	#if CONFIG_FBBASE_DOUBLE_BUFFERS
	if ((fbi == pxafb_main) && (PXAFB_MAIN_VALID_FB == PXAFB_MAIN_BASE) && (pxafb_main->double_buffers == 1) )
	{
		DPRINTK("fbi->second_fb.dmadesc_fb_cpu = 0x%x\n", fbi->second_fb.dmadesc_fb_cpu);
		DPRINTK("fbi->second_fb.dmadesc_fb_dma = 0x%x\n", fbi->second_fb.dmadesc_fb_dma);
		DPRINTK("fbi->second_fb.dmadesc_fb_cpu->fdadr = 0x%x\n", fbi->second_fb.dmadesc_fb_cpu->fdadr);
		DPRINTK("fbi->second_fb.dmadesc_fb_cpu->fsadr = 0x%x\n", fbi->second_fb.dmadesc_fb_cpu->fsadr);
		DPRINTK("fbi->second_fb.dmadesc_fb_cpu->ldcmd = 0x%x\n", fbi->second_fb.dmadesc_fb_cpu->ldcmd);
	}
	#endif

	if (fbi == pxafb_main)
	{
		fbi->reg_lccr0 = new_regs.lccr0;
		fbi->reg_lccr1 = new_regs.lccr1;
		fbi->reg_lccr2 = new_regs.lccr2;
		fbi->reg_lccr3 = new_regs.lccr3;
	}
	/*
	* We don't need to care about pxafb_smart.reg_lccr* settings, they should be initialized in pxafb_init_fbinfo()
	*/
	local_irq_restore(flags);

	/*
	 * Only update the registers if the controller is enabled
	 * and something has changed.
	 */
	//Susan -- Fix me: I just disable the fbi->task's registration in tq_context //
	if (0) //if ( (pxafb_global_state.init_phase) && (fbi == pxafb_main) )//if (0) -- don't enable/disable/enable will result in red-color pictures
	{
		if ((LCCR0 != fbi->reg_lccr0)       || (LCCR1 != fbi->reg_lccr1) ||
		    (LCCR2 != fbi->reg_lccr2)       || (LCCR3 != fbi->reg_lccr3) ||
		    (FDADR0 != fbi->fdadr0) || (FDADR1 != fbi->fdadr1))
			pxafb_schedule_task(fbi, C_REENABLE);
	}
	
	return 0;
}

/*
 * NOTE!  The following functions are purely helpers for set_ctrlr_state.
 * Do not call them directly; set_ctrlr_state does the correct serialisation
 * to ensure that things happen in the right way 100% of time time.
 *	-- rmk
 */

/*
 * FIXME: move LCD power stuff into pxafb_power_up_lcd()
 * Also, I'm expecting that the backlight stuff should
 * be handled differently.
 */
/*
static void pxafb_backlight_on(struct pxafb_info *fbi)
{
	DPRINTK("backlight on");

#ifdef CONFIG_ARCH_PXA_IDP
	if(machine_is_pxa_idp()) {	
		FB_BACKLIGHT_ON();
	}
#elif defined(CONFIG_CPU_BULVERDE)
	if (machine_is_mainstone()) {
		CKEN |= CKEN0_PWM0;
		PWM_CTRL0 = 0;
		PWM_PWDUTY0 = 0x3FF;
		PWM_PERVAL0 = 0x3FF;
	}
#endif
}
*/

/*
 * FIXME: move LCD power stuf into pxafb_power_down_lcd()
 * Also, I'm expecting that the backlight stuff should
 * be handled differently.
 */
/*
static void pxafb_backlight_off(struct pxafb_info *fbi)
{
	DPRINTK("backlight off");

#ifdef CONFIG_ARCH_PXA_IDP
	if(machine_is_pxa_idp()) {
		FB_BACKLIGHT_OFF();
	}
#elif defined(CONFIG_CPU_BULVERDE)
	if (machine_is_mainstone()) {
		PWM_CTRL0 = 0;
		PWM_PWDUTY0 = 0x0;
		PWM_PERVAL0 = 0x3FF;
		CKEN &= ~CKEN0_PWM0;
	}
#endif
	
}
*/

static void pxafb_power_up_lcd(struct pxafb_info *fbi)
{
	DPRINTK("LCD power on");
	CKEN |= CKEN16_LCD;

	if(machine_is_pxa_cerf()) {
		lcdctrl_enable();
	}

#if CONFIG_ARCH_PXA_IDP
	/* set GPIOs, etc */
	if(machine_is_pxa_idp()) {
		// FIXME need to add proper delays
		FB_PWR_ON();
		FB_VLCD_ON();	// FIXME this should be after scanning starts
	}
#endif

if (0)
{
#ifdef CONFIG_CPU_BULVERDE  //seperate backlight from <set_ctrlr_state> //
	if (fbi == pxafb_main)
	{
		DPRINTK("pxafb_power_up_lcd(%d): pxafb_main,enable CKEN16_LCD and CKEN0_PWM0\n",current->pid);
		CKEN |= CKEN0_PWM0;
	}
	if (fbi == pxafb_smart)
	{
		DPRINTK("pxafb_power_up_lcd(%d): pxafb_smart,enable CKEN16_LCD and CKEN0_PWM0???\n",current->pid);
		CKEN |= CKEN0_PWM0;
	}
#endif
}
}

static void pxafb_power_down_lcd(struct pxafb_info *fbi)
{
	DPRINTK("LCD power off");
	DPRINTK("pxafb_power_down_lcd(%d): for either main panel or cli panel\n",current->pid);
	DPRINTK("pxafb_power_down_lcd(%d): CKEN(0x%x)\n",current->pid, CKEN);

	CKEN &= ~CKEN16_LCD;  //Just disable LCD clock, we have only one lcd controller //

	if(machine_is_pxa_cerf()) {
		lcdctrl_disable();
	}

	/* set GPIOs, etc */
#if CONFIG_ARCH_PXA_IDP
	if(machine_is_pxa_idp()) {
		// FIXME need to add proper delays
		FB_PWR_OFF();
		FB_VLCD_OFF();	// FIXME this should be before scanning stops
	}
#endif
//#ifdef CONFIG_CPU_BULVERDE  //seperate backlight from <set_ctrlr_state> //
//	CKEN &= ~CKEN0_PWM0;
//#endif
}

#if (0)  //The below three functions are moved into product.c (barbados.c)
static void pxafb_setup_gpio(struct pxafb_info *fbi)
{
	unsigned int lccr0;

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
			// bits 58-61
			GPDR1 |= (0xf << 26);
			GAFR1_U = (GAFR1_U & ~(0xff << 20)) | (0xaa << 20);

			// bits 74-77
			GPDR2 |= (0xf << 10);
	        GAFR2_L = (GAFR2_L & ~(0xff << 20)) | (0xaa << 20);
		}

	    /* 8 bit interface */
	    else if (((lccr0 & LCCR0_CMS) && ((lccr0 & LCCR0_SDS) || (lccr0 & LCCR0_DPD))) || (!(lccr0 & LCCR0_CMS) && !(lccr0 & LCCR0_PAS) && !(lccr0 & LCCR0_SDS)))
	    {
			// bits 58-65
			GPDR1 |= (0x3f << 26);
	        GPDR2 |= (0x3);

	        GAFR1_U = (GAFR1_U & ~(0xfff << 20)) | (0xaaa << 20);
	        GAFR2_L = (GAFR2_L & ~0xf) | (0xa);

	        // bits 74-77
	        GPDR2 |= (0xf << 10);
	        GAFR2_L = (GAFR2_L & ~(0xff << 20)) | (0xaa << 20);
	    }

	    /* 16bpp, 18bpp, 19bpp, 24bpp and 25bpp interface */
	    else if (!(lccr0 & LCCR0_CMS) && ((lccr0 & LCCR0_SDS) || (lccr0 & LCCR0_PAS)))
	    {
			switch (fbi->max_bpp) 
			{
			case 16:
				// bits 58-77
				GPDR1 |= (0x3f << 26);
				GPDR2 |= 0x00003fff;
				
				GAFR1_U = (GAFR1_U & ~(0xfff << 20)) | (0xaaa << 20);
				GAFR2_L = (GAFR2_L & 0xf0000000) | 0x0aaaaaaa;
				break;
	#ifdef CONFIG_CPU_BULVERDE
			case 18:
			case 19:
			case 24:
			case 25:
				DPRINTK("Set 18-bit interface");
				// bits 58-77 and 86, 87
				GPDR1 |= (0x3f << 26);
				GPDR2 |= 0x00c03fff;
				
				GAFR1_U = (GAFR1_U & ~(0xfff << 20)) | (0xaaa << 20);
				GAFR2_L = (GAFR2_L & 0xf0000000) | 0x0aaaaaaa;
				GAFR2_U = (GAFR2_U & 0xffff0fff) | 0xa000;
				break;
	#endif
			default:
				break;
			}
	    }
		else
			printk(KERN_ERR "pxafb_setup_gpio: unable to determine bits per pixel\n");
			
	#ifdef LCD_72R89341N 
		/* Read & save LCD ID: ID1 = GPIO<18> and ID0 = GPIO<80> */
		/* GPDR set to input at reset */
	    fbi->lcd_id = (GPLR0 & 0x00040000) >> 17  | (GPLR2 & 0x00010000) >> 16;

	#ifdef BARBADOS_P3
		/* Setup GPIO<19> as general GPIO OUT HIGH */
		GAFR0_U &= ~(0x3 <<6);
		GPSR0 = 0x80000;
		GPDR0 |= 0x80000;
		mdelay(1);
	#else
		/* Set LCD_MUX to be high for main TFT panel */
		GPSR1 = 0x00020000;   //GPSR(49) = GPIO_bit(49);	/* GPIO<49> */
		GPDR1 |= 0x00020000; //GPDR(49) = GPIO_bit(49);

	#endif
	
	#endif
	}

	if (fbi == pxafb_smart)
	{		
		DPRINTK("pxafb_setup_gpio(%d):pxafb_smart\n",current->pid);
		/* We use 8 bit interface */
		/* LDD0 - LDD7 */	
		// bits 58-65 
        GPDR1 |= (0x3f << 26);
        GPDR2 |= (0x3);

        GAFR1_U = (GAFR1_U & ~(0xfff << 20)) | (0xaaa << 20);
        GAFR2_L = (GAFR2_L & ~0xf) | (0xa);

        // bits 74-76,77
        GPDR2 |= (0xf << 10);
        GAFR2_L = (GAFR2_L & ~(0xff << 20)) | (0xaa << 20);
        //GAFR2_L = (GAFR2_L & ~(0x3f << 20)) | (0x2a << 20);
        
 		/* We don't use L_VSYNC here. */
        
#ifdef LCD_SMART_72R89405Y01
		#ifdef BARBADOS_P3
       	/* L_CS: gpio19, L_CS for barbados P3 -- Susan*/
        GPDR0 |= (0x1 << 19);
        GAFR0_U = (GAFR0_U & ~(0x3 << 6)) | (0x2 << 6);
        #else
		/* Switch L_PCLK_WR to CS CLI panel */
		GPCR1 = 0x20000;  //GPCR(49) = GPIO_bit(49);
		GPDR1 |= 0x20000; //GPDR(49) = GPIO_bit(49); 
		#endif
#endif

	}
}

static void pxafb_enable_controller(struct pxafb_info *fbi)
{
	static unsigned int init_phase = 0;
	
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

		//mdelay(1); /* new delay time from xushiwei.  LCD timing delay*/
		
	#ifdef LCD_72R89341N

		GPSR2 = 0x00004000;	/* Turn on LCD_SD(GPIO<78>) */
		mdelay(1);		/* LCD timing delay*/
		GPCR2 = 0x00004000; 	/* LCD is enabled by setting LCD_SD low*/
		GPDR2 |= 0x00004000;

		GPCR2 = 0x00008000;  	/* Set LCD_CM(GPIO<79>) to be normal mode */ 
		GPDR2 |= 0x00008000;

		mdelay(200);  //72R89341N spec requires 166ms, based on my testing, it should be at least 200ms//

	#else
		/* This code needed to enable the LCD screen */
		GPSR3 = 0x00100000; /* enable GPIO<116> (LCD_OFF) */
		mdelay(10);
		GPCR3 = 0x00100000; /* new lCD is enabled by low level*/
		GPDR3 |= 0x00100000;
	#endif
		bLCDOn = 1;

		//DPRINTK("FDADR0 = 0x%08x\n", (unsigned int)FDADR0);
		//DPRINTK("FDADR1 = 0x%08x\n", (unsigned int)FDADR1);
		//DPRINTK("LCCR0 = 0x%08x\n", (unsigned int)LCCR0);
		//DPRINTK("LCCR1 = 0x%08x\n", (unsigned int)LCCR1);
		//DPRINTK("LCCR2 = 0x%08x\n", (unsigned int)LCCR2);
		//DPRINTK("LCCR3 = 0x%08x\n", (unsigned int)LCCR3);
		//DPRINTK("LCCR4 = 0x%08x", (unsigned int)LCCR4);
	}

	if (fbi == pxafb_smart)
	{
		DPRINTK("pxafb_enable_controller(%d):pxafb_smart\n",current->pid);
		pxafb_smart_device_ldm.power_state = DPM_POWER_ON;

		LCCR5 = fbi->reg_lccr5;
		LCCR4 = fbi->reg_lccr4;
		LCCR3 = fbi->reg_lccr3;
		LCCR2 = fbi->reg_lccr2;
		LCCR1 = fbi->reg_lccr1;
		LCCR0 = fbi->reg_lccr0 & ~LCCR0_ENB;
				
		/* Stop processor when it executed "wait for sync" command */
		CMDCR = 0x1;
		/* TODO:  need to setup corresponding GPIO pins -- Susan */
		/* TODO:  need to enable LCD controller */
		DPRINTK("LCCR0 = 0x%08x", (unsigned int)LCCR0);
		DPRINTK("LCCR1 = 0x%08x", (unsigned int)LCCR1);
		DPRINTK("LCCR2 = 0x%08x", (unsigned int)LCCR2);
		DPRINTK("LCCR3 = 0x%08x", (unsigned int)LCCR3);
		DPRINTK("LCCR4 = 0x%08x", (unsigned int)LCCR4);
		DPRINTK("LCCR5 = 0x%08x", (unsigned int)LCCR5);
		
#ifdef LCD_SMART_72R89405Y01
		if (!init_phase ++)
		{
			unsigned long i = 0;

			DPRINTK("Default state: GPIO<19> D: GPDR0(0x%x)\n", GPDR0);
			DPRINTK("Default state: GPIO<19> L: GPLR0(0x%x)\n", GPLR0);
			DPRINTK("Default state: GPIO<49> D: GPDR1(0x%x)\n", GPDR1);
		    DPRINTK("Default state: GPIO<49> L: GPLR1(0x%x)\n", GPLR1);
			DPRINTK("Default state: GPIO<76> D: GPDR2(0x%x)\n", GPDR2);
		    DPRINTK("Default state: GPIO<76> F: GAFR2_L(0x%x)\n", GAFR2_L);

			#ifdef BARBADOS_P3
	            set_GPIO_mode(49|GPIO_OUT);
				/* Pulse CLI active low hardware reset signal */
				GPSR(49) = GPIO_bit(49);	/* set GPIO<19> high  */
	 	        mdelay(1);
				GPCR(49) = GPIO_bit(49); 	/* CLI is reset by setting low */
				mdelay(20);		/* assert for ~20msec */
			    DPRINTK("Check GPIO<49> direction: GPDR1(0x%x)\n", GPDR1);
			    DPRINTK("Check GPIO<49> level: GPLR1(0x%x)\n", GPLR1);
				GPSR(49) = GPIO_bit(49);	/* set GPIO<19> high  */

				//mdelay(30);
			#else
	            set_GPIO_mode(19|GPIO_OUT);
				/* Pulse CLI active low hardware reset signal */
				GPSR(19) = GPIO_bit(19);	/* set GPIO<19> high  */
	 	        mdelay(1);
				GPCR(19) = GPIO_bit(19); 	/* CLI is reset by setting low */
				mdelay(20);		/* assert for ~20msec */
			    DPRINTK("Check GPIO<19> direction: GPDR0(0x%x)\n", GPDR0);
			    DPRINTK("Check GPIO<19> levels: GPLR0(0x%x)\n", GPLR0);
				GPSR(19) = GPIO_bit(19);	/* set GPIO<19> high  */

				//mdelay(30);
			#endif
			
			/* Initialize the LCD panel. Need to do initialize only once. */	
			pxafb_smart_send_init_cmd(pxafb_smart);	
		}
		else
		{
			/* Initialize the LCD panel. Need to do initialize only once. */	
			pxafb_smart_display_on(pxafb_smart);
		}

		if (1 == init_phase ++)
		{
		    DPRINTK("pxafb_smart_load_init_image\n");
			pxafb_smart_load_init_image(pxafb_smart);
		}
		
		/*Make sure CLI will not be reset by sleep/deep sleep mode */
		PGSR(49) |= GPIO_bit(49);
#endif
	}
}

static void pxafb_disable_controller(struct pxafb_info *fbi)
{
#if 0
	/* Unnecessary if nothing is put on the wait_queue */
	DECLARE_WAITQUEUE(wait, current);
#endif

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
	#ifdef LCD_72R89341N
		DPRINTK("pxafb_disable_controller(%d):disable GPIOs for LCD_72R89341N\n",current->pid);

		GPCR2 = 0x00004000;//GPCR(78) = GPIO_bit(78);   /* LCD_SD(GPIO<78>), rising edge for turn off */
		mdelay(1);
		GPSR2 = 0x00004000;//GPSR(78) = GPIO_bit(78); 
		GPDR2 |= 0x00004000; //GPDR(78) = GPIO_bit(78);
		PGSR2 |= 0x00004000;  /* set sleep state of GPIO<78> */
#ifdef CONFIG_ARCH_EZX_MARTINIQUE
		mdelay(67);  // According to Toshiba TFT spec //
#else
		mdelay(33);  // According to TFT spec //
#endif

		GPSR2 = 0x00008000;  //GPSR(79) = GPIO_bit(79); /* Susan: LCD_CM(GPIO<79>), high for partial mode */

		#if (0) //Based on my testing, this will cause CLI display disappear //
		
		#ifdef BARBADOS_P3
		/*Do nothing */
		#else
		GPCR1 = 0x00020000; //GPCR(49) = GPIO_bit(49);  /* Susan:switch LCD_MUX to CLI panel */
		GPDR1 |= 0x00020000; //GPDR(49) = GPIO_bit(49);
		#endif
		
		#endif
	#else
		GPSR(116) = GPIO_bit(116); //GPSR3 = 0x00100000;/* new LCD is disabled by high level*/
		PGSR3 |= 0x00100000;  // set sleep state of GPIO<116> -- LCD_OFF //
		mdelay(41);
	#endif	

		LCCR0 &= ~LCCR0_LDM;	/* Enable LCD Disable Done Interrupt */
	//	LCCR0 &= ~LCCR0_ENB;	/* Disable LCD Controller */
		LCCR0 |= LCCR0_DIS;   //Normal disable LCD //
		mdelay(18);
	
		pxafb_main_device_ldm.power_state = DPM_POWER_OFF;
	}

	if (fbi == pxafb_smart)
	{
		DPRINTK("pxafb_disable_controller(%d):pxafb_smart\n",current->pid);
		pxafb_smart_device_ldm.power_state = DPM_POWER_OFF;
		
#ifdef LCD_SMART_72R89405Y01
		/* Firstly, we need to display off CLI panel, then remove its CS */
		pxafb_smart_display_off(pxafb_smart);

		#ifdef BARBADOS_P3
		/* Enable GPIO<19> as general GPIO OUT HIGH */
		GAFR0_U &= ~(0x3 << 6);
		GPSR0 = 0x80000;
		GPDR0 |= 0x80000;
		mdelay(1);
		#else
		/* Set LCD_MUX to be HIGH to switch to main panel*/
		GPSR1 = 0x00020000;  //GPSR(49) = GPIO_bit(49); 	/* GPIO<49> */
		GPDR1 |= 0x00020000;//GPDR(49) = GPIO_bit(49); 
		#endif
#endif

		LCSR0 = 0xffffffff;	/* Clear LCD Status Register */
		LCCR0 &= ~LCCR0_LDM;
		LCCR0 |= LCCR0_DIS;	/* Disable LCD Controller */
		mdelay(2);
		//Since CLI panel is passive, we don't need to wait the completion of current frame -- Susan//
		
		/* make sure GPIO<49> will not be pulled down during CLI is disabled */
		PGSR(49) |= GPIO_bit(49); //PGSR1 &= ~(1<<17);
	}
}

static void pxafb_suspend_controller(struct pxafb_info *fbi)
{
#if 0
	/* Unnecessary if nothing is put on the wait_queue */
	DECLARE_WAITQUEUE(wait, current);
#endif

	DPRINTK("Disabling LCD controller");

	/* Think about CLI/MAIN panel case, take semaphore in <set_ctrlr_state> before panel switchings -- Susan */
	if (fbi == pxafb_main)
	{
		/* Do nothing, just return */
		return;
	}

	if (fbi == pxafb_smart)
	{
		DPRINTK("pxafb_suspend_controller(%d):pxafb_smart\n",current->pid);
		pxafb_smart_device_ldm.power_state = DPM_SUSPEND_FOR_OP;
		
#ifdef LCD_SMART_72R89405Y01

		#ifdef BARBADOS_P3
		/* Enable GPIO<19> as general GPIO OUT HIGH */
		GAFR0_U &= ~(0x3 << 6);
		GPSR0 = 0x80000;
		GPDR0 |= 0x80000;
		mdelay(1);
		#else
		/* Set LCD_MUX to be HIGH to switch to main panel*/
		GPSR1 = 0x00020000; //GPSR(49) = GPIO_bit(49) ; 	/* GPIO<49> */
		GPDR1 |= 0x00020000;//GPDR(49) = GPIO_bit(49); 
		#endif
#endif

		LCSR0 = 0xffffffff;	/* Clear LCD Status Register */
		LCCR0 &= ~LCCR0_LDM;
		LCCR0 |= LCCR0_DIS;	/* Disable LCD Controller */
		mdelay(2);
		//Since CLI panel is passive, we don't need to wait until the completion of current frame -- Susan//
		/* Make sure CLI will not be reset during sleep/deep sleep mode */
		PGSR(49) |= GPIO_bit(49);
	}
}
#endif

#if (0)  //pxafb_smart_load_init_image is moved to product.c (barbados.c)
static void pxafb_smart_load_init_image(struct pxafb_info *fbi)
{ //Susan for testing CLI display transfer byte-by-byte //
	unsigned long i, j, count, smart_offset, main_offset, this_one;
	unsigned char r, g, b;
	unsigned short *data;
	unsigned short cmd;
	unsigned char *source;
	int x, fblen, ret;
	unsigned int cmd_size;
			
	DPRINTK("pxafb_smart_load_init_image(%d), begin show picture in SMART panel\n", current->pid);

	/* This is for unpacked 18bpp */
	//for (i = 0; i < pxafb_smart->max_yres; i ++)
	//{
	//	for (j = 0; j < pxafb_smart->max_xres; j ++)
	//	{
	//		smart_offset = i * pxafb_smart->max_xres * 4 + j * 4;
	//		main_offset = i * pxafb_main->max_xres * 3 + j * 3;
	//		pxafb_smart->screen_cpu[smart_offset] = pxafb_main->screen_cpu[main_offset];
	//		pxafb_smart->screen_cpu[smart_offset + 1] = pxafb_main->screen_cpu[main_offset + 1];
	//		pxafb_smart->screen_cpu[smart_offset + 2] = pxafb_main->screen_cpu[main_offset + 2];
	//		pxafb_smart->screen_cpu[smart_offset + 3] = 0x0;
	//	}
	//}

	/* For packed 18bpp */
	if (1)  //For frame transfer //
	{
	for (i = 0; i < pxafb_smart->max_yres; i ++)
	{
		smart_offset = i * pxafb_smart->max_xres * 3;  //Only for packed 18bpp //
		main_offset = i * pxafb_main->max_xres * 3;
		memcpy(&pxafb_smart->screen_cpu[smart_offset], &pxafb_main->screen_cpu[main_offset], pxafb_smart->max_xres * 3);
	}

	count = pxafb_smart->max_xres * pxafb_smart->max_yres;
	for ( i = 0; i < count; i ++ )  //For each pixel //
	{
		this_one = i % pxafb_smart->max_xres;

		if ( (this_one >= 0) && (this_one <= pxafb_smart->max_xres/3) )
		{
			pxafb_smart->screen_cpu[i * 3] = 0x00;
			pxafb_smart->screen_cpu[i*3 + 1] = 0xF0;
			pxafb_smart->screen_cpu[i*3 + 2] = 0x03;
		}

		if ( (this_one > pxafb_smart->max_xres/3) && (this_one <= pxafb_smart->max_xres/3*2) )
		{
			pxafb_smart->screen_cpu[i * 3] = 0xC0;
			pxafb_smart->screen_cpu[i*3 + 1] = 0x0F;
			pxafb_smart->screen_cpu[i*3 + 2] = 0x00;
		}

		if ( (this_one > pxafb_smart->max_xres/3*2) && (this_one < pxafb_smart->max_xres) )
		{
			pxafb_smart->screen_cpu[i * 3] = 0x3F;
			pxafb_smart->screen_cpu[i*3 + 1] = 0x00;
			pxafb_smart->screen_cpu[i*3 + 2] = 0x00;
		}
	}

	pxafb_smart_send_frame_data(pxafb_smart);
	
	}
	
	if (0)  //For byte-to-byte transfer //
	{
	for (i = 0; i < pxafb_smart->max_yres; i ++)
	{
		smart_offset = i * pxafb_smart->max_xres * 3;  //Only for packed 18bpp //
		main_offset = i * pxafb_main->max_xres * 3;
		memcpy(&pxafb_smart->screen_cpu[smart_offset], &pxafb_main->screen_cpu[main_offset], pxafb_smart->max_xres * 3);
	}
	
	count = pxafb_smart->max_xres * pxafb_smart->max_yres;

	for ( i = 0; i < count; i ++ )
	{
		b = pxafb_smart->screen_cpu[3*i];
		g = pxafb_smart->screen_cpu[3*i+1];
		r = pxafb_smart->screen_cpu[3*i+2];
		pxafb_smart->screen_cpu[3*i] = b<<2;
		pxafb_smart->screen_cpu[3*i+1] = (g<<4)|((b>>6)<<2);
		pxafb_smart->screen_cpu[3*i+2] = (r<<6) | ((g>>4)<<2);
	}

	for ( i = 0; i < count; i ++ )  //For each pixel //
	{
		this_one = i % pxafb_smart->max_xres;

		if ( (this_one >= 0) && (this_one <= pxafb_smart->max_xres/3) )
		{
			pxafb_smart->screen_cpu[i * 3] = 0x0;
			pxafb_smart->screen_cpu[i*3 + 1] = 0x0;
			pxafb_smart->screen_cpu[i*3 + 2] = 0xFC;
		}

		if ( (this_one > pxafb_smart->max_xres/3) && (this_one <= pxafb_smart->max_xres/3*2) )
		{
			pxafb_smart->screen_cpu[i * 3] = 0x0;
			pxafb_smart->screen_cpu[i*3 + 1] = 0xFC;
			pxafb_smart->screen_cpu[i*3 + 2] = 0x0;
		}

		if ( (this_one > pxafb_smart->max_xres/3*2) && (this_one < pxafb_smart->max_xres) )
		{
			pxafb_smart->screen_cpu[i * 3] = 0xFC;
			pxafb_smart->screen_cpu[i*3 + 1] = 0x0;
			pxafb_smart->screen_cpu[i*3 + 2] = 0x0;
		}
	}
	pxafb_smart_send_display_data(pxafb_smart);
	}  //if (0) case

	#if (0)		
	r = pxafb_smart->max_xres/3;
	g = r * 2;
	b = pxafb_smart->max_xres;

	for ( i = 0; i < (pxafb_smart->max_xres * pxafb_smart->max_yres); i ++ )  //For each pixel //
	{
		this_one = i % pxafb_smart->max_xres;

		if ( (this_one >= 0) && (this_one <= r) )
		{
			pxafb_smart->screen_cpu[i * 3] = 0x0;
			pxafb_smart->screen_cpu[i*3 + 1] = 0x0;
			pxafb_smart->screen_cpu[i*3 + 2] = 0xFC;
		}

		if ( (this_one > r) && (this_one <= g) )
		{
			pxafb_smart->screen_cpu[i * 3] = 0x0;
			pxafb_smart->screen_cpu[i*3 + 1] = 0xFC;
			pxafb_smart->screen_cpu[i*3 + 2] = 0x0;
		}

		if ( (this_one > g) && (this_one < b) )
		{
			pxafb_smart->screen_cpu[i * 3] = 0xFC;
			pxafb_smart->screen_cpu[i*3 + 1] = 0x0;
			pxafb_smart->screen_cpu[i*3 + 2] = 0x0;
		}
	}
	DPRINTK("pxafb_init(%d), finish filling the framebuffer of the SMART panel\n", current->pid);
			
	DPRINTK("pxafb_init(%d), before pxafb_smart_send_frame_data\n", current->pid);
	pxafb_smart_send_frame_data(pxafb_smart);
	#endif
}
#endif  //move pxafb_smart_load_init_image into product.c (barbados.c)
#ifdef CONFIG_PXAFB_CLI
void pxafb_smart_send_init_cmd(struct pxafb_info *fbi)
{
	unsigned short wakeup_cmd[2] __attribute__ ((aligned (16))) = {
	/* This software init routine found in 6.3.4 of 72R88154Y01 (CLI LCD Spec) */
	SLEEPOUT, 			/* Sleep OUT */
	IOSON	 			/* Internal Oscillator ON */
	};

	#if (0)  //old glass routing //
	/* According to spec 72R89405Y01 */
	unsigned short smart_lcd_config[40] __attribute__ ((aligned (16))) = {
	0x0220,0x030B, 			/* 0x0220,0x030B, Power Control Set */
	0x02FB,0x0303, 			/* LCD Bias Ratio Set */
	0x02F7,0x0328,0x038E,0x0305, 	/* PWM/FRC Scroll */
	0x0282,0x0300, 			/* Temperature Gradient Set */
	0x02BB,0x0301, 			/* 0x02BB,0x0301,COM Scan Direction */
	0x02BC,0x0300,0x0300,0x0307, 	/* Data Scan Direction, 18BPP, 8-BIT interface*/
	0x02F2,0x0305,0x0346, 		/* 0x02F2,0x0305,0x0346, Frame Freq and N-line */
	0x0281,0x0314,0x0305, 		/* 0x0281,0x0314,0x0305, Electronic Contrast Control */
	0x02CA,0x0300,0x031A,0x0300, 	/* Display Control */
	0x0244,0x0300,				/* Set first display com, set start line=COM0 */
	0x0275,0x0300,0x036B, 		/* Page Address Set */
	/* FIXME: What is the exact value ? */
	//0x02F2,0x030d,0x0306, 		/* Frame Freq and N-line */
	0x0215,0x0300,0x037F, 		/* Column Address Set */
	0x02A7, 			/* Inverse Display */
	0x02F1,0x03F0,0x0300,0x0300, 	/* Set COM sequence */
	0x02AF 				/* Display ON */
	};
	#endif

	#if (1)  //new glass routing //
	/* According to spec 72R89405Y01 */
	unsigned short smart_lcd_config[40] __attribute__ ((aligned (16))) = {
	POWERCONTROLSET_0,POWERCONTROLSET_1, 			/* 0x0220,0x030B, Power Control Set */
	BIASRATIOSET_0,BIASRATIOSET_1, 			/* LCD Bias Ratio Set */
	PWM_FRC_SCROLL_0,PWM_FRC_SCROLL_1,PWM_FRC_SCROLL_2,PWM_FRC_SCROLL_3, 	/* PWM/FRC Scroll */
	TEMPE_GRADIENT_0,TEMPE_GRADIENT_1, 			/* Temperature Gradient Set */
	COM_SCAN_DIRECTION_0,COM_SCAN_DIRECTION_1, 			/* 0x02BB,0x0301,COM Scan Direction */
	DATA_SCAN_DIRECTION_0,DATA_SCAN_DIRECTION_1,DATA_SCAN_DIRECTION_2,DATA_SCAN_DIRECTION_3, 	/* Data Scan Direction, 18BPP, 8-BIT interface*/
	FRAME_FREQ_NLINE_0,FRAME_FREQ_NLINE_1,FRAME_FREQ_NLINE_2, 		/* 0x02F2,0x0305,0x0346, Frame Freq and N-line */
	CONTRAST_CONTROL_0,CONTRAST_CONTROL_1,CONTRAST_CONTROL_2, 		/* 0x0281,0x0314,0x0305, Electronic Contrast Control */
	DISPLAY_CONTROL_0,DISPLAY_CONTROL_1,DISPLAY_CONTROL_2,DISPLAY_CONTROL_3, 	/* Display Control */
	FIRST_DISPLAY_CMD,START_LINE,				/* Set first display com, set start line=COM0 */
	PAGE_ADDR_SET_0,PAGE_ADDR_SET_1,PAGE_ADDR_SET_2, 		/* Page Address Set */
	/* FIXME: What is the exact value ? */
	//0x02F2,0x030d,0x0306, 		/* Frame Freq and N-line */
	COLUMN_ADDRESS_SET_0,COLUMN_ADDRESS_SET_1,COLUMN_ADDRESS_SET_2, 		/* Column Address Set */
	INVERSE_DISPLAY, 			/* Inverse Display */
	SET_COM_SEQ_0,SET_COM_SEQ_1,SET_COM_SEQ_2,SET_COM_SEQ_3, 	/* Set COM sequence */
	DISPLAY_ON 				/* Display ON */
	};
	#endif
	
	unsigned short display_on_cmd[1] __attribute__ ((aligned (16))) = {
		DISPLAY_ON 				/* Display ON */
	};

	//down(&pxafb_global_state.g_sem);

	DPRINTK("\n");	
	DPRINTK("Begin send <wakeup_cmd> to the smart panel");	
	DPRINTK("\n");

	pxafb_smart_load_cmd(fbi, wakeup_cmd, 2);
	pxafb_smart_send_cmd(fbi);
	
	DPRINTK("Finish send <wakeup_cmd> to the smart panel");	
	mdelay(2);//mdelay(200);
	
	DPRINTK("\n");	
	DPRINTK("Begin send init cmd <smart_lcd_config> to the smart panel");	
	DPRINTK("\n");
	
	/* Try to only initialize CLI panel first, then wakeup/display on it -- Susan */
	pxafb_smart_load_cmd(fbi, smart_lcd_config, 40);
	pxafb_smart_send_cmd(fbi);

	DPRINTK("Finish send <smart_lcd_config> to the smart panel");	
	mdelay(2);//mdelay(30);

	DPRINTK("\n");	
	DPRINTK("Begin send <display_on_cmd> to the smart panel");	
	DPRINTK("\n");
	
	pxafb_smart_load_cmd(fbi, display_on_cmd, 1);
	pxafb_smart_send_cmd(fbi);

	DPRINTK("Finish send <display_on_cmd> to the smart panel");	
	mdelay(2);//mdelay(10);

	//up(&pxafb_global_state.g_sem);
}

/*
*  pxafb_smart_display_off: send display_off cmd, oscillator off cmd, sleep in cmd
*  Assumption: LCD controller has been switched to pxafb_smart CSTN panel
*/
void pxafb_smart_display_off(struct pxafb_info *fbi)
{
	unsigned short display_off_cmd[4] __attribute__ ((aligned (16))) = {
		/* this sequence found in 5.4.9 of 51R88450M28 (Solomon spec) */	
		/* adjust the seq according to new SW setting from CLI EE*/	
		DISPLAY_OFF,	/* Display OFF */	
		SLEEP_IN,	/* Sleep IN */		
		IOSOFF,	/* Internal Oscillator OFF */
		0x0800	/* Pad with noop */
	};
	DPRINTK("Begin send lcd sleep cmd");

	FDADR0 = fbi->fdadr0;
	
	pxafb_smart_load_cmd(fbi, display_off_cmd, 4);
	pxafb_smart_send_cmd(fbi);
	
	FDADR0 = 0;
}

/*
*  pxafb_smart_display_on: send sleep out cmd | oscillator on cmd | display on cmd
*  Assumption: LCD controller has been switched to pxafb_smart CSTN panel
*/
void pxafb_smart_display_on(struct pxafb_info *fbi)
{
	unsigned long expire;
	
	/* According to sw setting from CLI EE 2005-09-16*/
	unsigned short wake_up_cmd[2] __attribute__ ((aligned (16))) = {
		IOSON, 				/* Internal Oscillator ON */
		SLEEP_OUT 			/* Sleep OUT */	
	};

	unsigned short display_on_cmd[1] __attribute__ ((aligned (16))) = {
		DISPLAY_ON 				/* Display ON */
	};

	FDADR0 = fbi->fdadr0;
	
	/* Check if CLI panel is suspend or disabled, if suspend, just display_on_cmd, if disabled, wake_up_cmd is needed */
	/* pxafb_global_state.g_sem has been taken in <set_ctrlr_state> -- Susan */
	if ( !(pxafb_global_state.smart_state & PXAFB_DISP_ON) )  //someone has display_off CLI panel //
	{
		DPRINTK("Begin send <wake_up_cmd> cmd");
		pxafb_smart_load_cmd(fbi, wake_up_cmd, 2);
		pxafb_smart_send_cmd(fbi);

		/*Waiting 100ms to charge up VOUT */
		set_current_state(TASK_UNINTERRUPTIBLE);
		expire = schedule_timeout(CSTN_CHARGE_UP_VOUT_IN_JIFFIES);

		DPRINTK("Finish send <wake_up_cmd> cmd");
	}
	//Do we need to wait for 20ms ? -- Susan

	/*disp on */
	DPRINTK(" pxafb_smart_display_on: Begin send <display_on_cmd> cmd \n");
	pxafb_smart_load_cmd(fbi, display_on_cmd, 1);
	pxafb_smart_send_cmd(fbi);
	DPRINTK(" pxafb_smart_display_on: Finish send <display_on_cmd> cmd \n");
	//mdelay(30);  //Assuming framedata is transfered to CLI panel completely //
	
	FDADR0 = 0; //disable DMA
	
}
int pxafb_smart_send_display_data(struct pxafb_info *fbi)
{
	unsigned short *data;
	unsigned short cmd;
	unsigned char *source;
	int x, fblen, cmd_size,ret;
	
	/* set up data write command */
	data = (unsigned short *)(pxafb_smart->cmd_buf_cpu);
	*data++ = LCD_CMD_COMMAND_WRITE | LCD_CMD_A0_COMMAND | 0x5C;
	
	source = pxafb_smart->screen_cpu;
	fblen = pxafb_smart->max_xres * pxafb_smart->max_yres;
	cmd = LCD_CMD_DATA_WRITE | LCD_CMD_A0_DATA;

	if (pxafb_smart->fb.var.bits_per_pixel == 16) {
		for (x=0; x < fblen; x++)
		{
			*data++ = (cmd | *(source+1));
			*data++ = (cmd | *(source));
			source = source + 2;	
		}
	} else {

	/* Assumes framebuffer is RGB666 */
	/* Need to change the byte order. Red is written to higher address byte */
	for (x=0; x < fblen; x++) // For the CLI panel, 18bpp pixel format is fixed -- Susan //
	{
		*data++ = (cmd | *(source));
		*data++ = (cmd | *(source + 1));
		*data++ = (cmd |*(source + 2));	
		source = source + 3;	
	}
	}
	/* At the end, write the "Interrupt Processor" command */
	/* Also, write the "Wait for Vsync" command, 
	 * although no such input, just stops me */
	*data++ = LCD_CMD_INT_PROC | LCD_CMD_A0_COMMAND; 
	*data++ = LCD_CMD_WAIT_FOR_VSYNC | LCD_CMD_A0_COMMAND;
	
	/* Need alignment here */
	if (pxafb_smart->fb.var.bits_per_pixel == 16) 
		cmd_size = (fblen * 2 + 3) * sizeof(unsigned short);
	else
		cmd_size = (fblen * 3 + 3) * sizeof(unsigned short);
	pxafb_smart->dmadesc_cmd_cpu->ldcmd = (cmd_size + (4 - (cmd_size) % 4));
	pxafb_smart->dmadesc_cmd_cpu->fdadr = pxafb_smart->dmadesc_cmd_dma;
	pxafb_smart->dmadesc_cmd_cpu->fsadr = pxafb_smart->cmd_buf_dma;

	DPRINTK("pxafb_smart_send_display_data(%d): Before send display data into CLI panel\n", current->pid);
	
	ret = pxafb_smart_send_cmd(pxafb_smart);
	
	DPRINTK("pxafb_smart_send_display_data(%d): After send display data into CLI panel\n", current->pid);
	
	return ret;
}

int pxafb_smart_send_frame_data(struct pxafb_info *fbi)
{
	int ret;

	/* Note: For RGB 6:6:6, the data is sent as 
	 * RRRRRR00, GGGGGG00, BBBBBB00 */
	unsigned short frame_write_cmd[2] __attribute__ ((aligned (16))) = 
	{
		LCD_CMD_COMMAND_WRITE | LCD_CMD_A0_COMMAND | 0x5C, /* Enter Data Write Mode */
		LCD_CMD_FRAME_DATA_WRITE | LCD_CMD_A0_DATA /* LCD_CMD_A0_DATA is important */
	};
	
	//ss:  down_interruptible(&pxafbi->ctrlr_sem);
	/* The DMA setting looks strange, but it works.. */
		
	FDADR0 = fbi->fdadr0;
	DPRINTK("pxafb_smart_send_frame_data(%d), FDADR0(0x%x)\n", current->pid, FDADR0);
	
	//FBR0 = fbi->fdadr0 | 0x1;
	if (0)  //Right now, no overlay is enabled for CLI smart panel //
	{
		if (fbi->overlay1fb && fbi->overlay1fb->state == C_ENABLE) {
			FDADR1 = fbi->overlay1fb->dma1->fdadr;
			OVL1C1 |= OVL1C1_O1EN;
			//FBR1 = smartfbi->overlay1fb->dma1->fdadr | 0x1;
		}
		if (fbi->overlay2fb && fbi->overlay2fb->state == C_ENABLE) {
			FDADR2 = fbi->overlay2fb->dma2->fdadr;
			//FBR2 = smartfbi->overlay2fb->dma2->fdadr | 0x1;
			if (fbi->overlay2fb->format != 0) {
				FDADR3 = fbi->overlay2fb->dma3->fdadr;
				FDADR4 = fbi->overlay2fb->dma4->fdadr;
				//FBR3 = smartfbi->overlay2fb->dma3->fdadr | 0x1;
				//FBR4 = smartfbi->overlay2fb->dma4->fdadr | 0x1;
			}
			OVL2C1 |= OVL2C1_O2EN;
		}

		/* FIXME: When to set palette? */
		if(fbi->cursorfb && fbi->cursorfb->state == C_ENABLE) {
			FDADR5 = fbi->cursorfb->dma5_frame->fdadr | 0x1;
		}
	}	
	
	pxafb_smart_load_cmd(fbi, frame_write_cmd, 2);
	ret = pxafb_smart_send_cmd(fbi);

	//ss:  down_interruptible(&pxafbi->ctrlr_sem);
	FDADR0 = 0;
	DPRINTK("pxafb_smart_send_frame_data(%d),FDADR0(0x%x)\n", current->pid, FDADR0);
	//FBR0 = 0x1;
	
	if (0)  //right now, overlay is not supported on CLI panel
	{
		if (fbi->overlay1fb && fbi->overlay1fb->state == C_ENABLE) {
			OVL1C1 &= ~OVL1C1_O1EN;
			FDADR1 = 0;
			//FBR1 = 0x1;
		}
		if (fbi->overlay2fb && fbi->overlay2fb->state == C_ENABLE) {
			OVL2C1 &= ~OVL2C1_O2EN;
			FDADR2 = 0;
			//FBR2 = 0x1;
			if (fbi->overlay2fb->format != 0) {
				FDADR3 = 0;
				FDADR4 = 0;
				//FBR3 = 0x1;
				//FBR4 = 0x1;
			}
		}
		if(fbi->cursorfb && fbi->cursorfb->state == C_ENABLE) {
			FDADR5 = 0;
		}
	}
	//ss:  up(&pxafbi->ctrlr_sem);
	
	return ret;
}

int pxafb_smart_send_partial_cmd(struct pxafb_info *fbi,unsigned short * cmd_ptr, int cmd_len)
{
	int ret;

	FDADR0 = fbi->fdadr0;
	pxafb_smart_load_cmd(fbi, cmd_ptr, cmd_len);
	ret = pxafb_smart_send_cmd(fbi);
	DPRINTK(" pxafb_smart_send_partial_cmd: EXIT\n");	
	FDADR0 = 0;
	return ret;
}
	

#endif


/*
 *  pxafb_handle_irq: Handle 'LCD DONE' interrupts.
 */
static void pxafb_handle_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned int lcsr0 = LCSR0;
	unsigned int lcsr1 = LCSR1;

#ifdef WAIT_MORE_EOF
	static unsigned int cnt, oscr, oscr_s, delta, flag, fcs_cnt;

	if (lcsr0 & LCSR0_EOF) {
		if (fcs_in_eof == 0) {
			if (flag == 1) {
				cnt = 0;
				oscr_s = OSCR;
				flag = 2;
			} else if (flag == 2) {
				cnt ++;
				oscr = OSCR;
				if (oscr - oscr_s < (cnt * FRAME_TIME)) {
					cnt = 0;
					oscr_s = OSCR;
				}
			}
		} else if (fcs_in_eof == 1) {
			if (flag == 2) {
				cnt ++;
				delta = OSCR - (oscr_s + (cnt * FRAME_TIME));
			}
		}
	}
#endif
	
#if CONFIG_FBBASE_DOUBLE_BUFFERS

DPRINTK("PXAFB_HANDLE_IRQ: LCSR0(0x%x)\n",LCSR0);

	if (lcsr0 & LCSR0_BS)
	{
#if (0)
		/* This interrupt status bit has been set after load the branched DMA descriptor */
		br_duration = OSCR - br_duration;
		DPRINTK("BR_DURATION is %d OSCR counts\n", br_duration);
#endif		
		DPRINTK("PXAFB_HANDLE_IRQ: LCSR0_BS is set,wakeup pandisplay\n");

		wake_up(&(pxafb_global_state.pxafb_current->ctrlr_wait));
	}
	if (lcsr1 & LCSR1_BS1)
	{
		wake_up((&pxafb_global_state.pxafb_current->ctrlr_wait));
	}
#endif


#ifdef CONFIG_OVL2_CHANNEL
	if (lcsr1 & LCSR1_BS2)  //Y channel has a BS interrupt //
	{
		if (pxafb_global_state.pxafb_current == pxafb_main)
			wake_up(&(pxafb_global_state.pxafb_current->overlay2fb->bs_wait));
	}
#endif


#ifdef FCS_EOF

        if (lcsr0 & LCSR0_EOF && (fcs_in_eof == 1))  {
#ifdef CHK_IPC
		if (ipc_is_active())
			fcs_in_eof = -1;
		else 
#endif	
		{
			unsigned long flags;
#ifdef WAIT_MORE_EOF
			if ((((int)delta > DELAY_MAX) || ((int)delta < DELAY_MIN)) && (fcs_cnt < 3)) {
				LCSR0 = lcsr0;
				LCSR1 = lcsr1; 
				fcs_cnt ++;
				return;
			}
#endif

		        save_flags_cli(flags);
                	lcsr0 |= LCSR0_EOF;

			bulverde_change_freq();
#ifdef WAIT_MORE_EOF
			fcs_cnt = 0;
			flag = 1;
#endif
        	        fcs_in_eof = 0;
			restore_flags(flags);
		}
                wake_up(&lcd_eof_fcs_wait_q);
        }
#endif

	//if (lcsr0 & LCSR0_LDD) 
	//{
	//	LCCR0 |= LCCR0_LDM;
	//	wake_up((&pxafb_global_state.pxafb_current->ctrlr_wait));
	//}
	LCSR0 = lcsr0;
	LCSR1 = lcsr1; //clear all interrupts //
}

void delay_bklight(void)
{
	//if (bklight_timer.function)
	//	mod_timer(&bklight_timer, jiffies + bk_timeout);
}

/*
 * This function must be called from task context only, since it will
 * sleep when disabling the LCD controller, or if we get two contending
 * processes trying to alter state.
 */
static void set_ctrlr_state(struct pxafb_info *fbi, u_int state)
{
	u_int old_state;

	DPRINTK("set_ctrlr_state(%d): pxafb_main(0x%x),pxafb_smart(0x%x),fbi(0x%x), state(%d)\n",current->pid,pxafb_main,pxafb_smart,fbi,state);
	//down(&fbi->ctrlr_sem);
	DPRINTK("set_ctrlr_state(%d):down(&pxafb_global_state.g_sem)\n",current->pid);
	down(&pxafb_global_state.g_sem);

	old_state = fbi->state;

	switch (state) {
	case C_DISABLE_CLKCHANGE:
		/*
		 * Disable controller for clock change.  If the
		 * controller is already disabled, then do nothing.
		 */
		DPRINTK("set_ctrlr_state:old_state(%d),C_DISABLE_CLKCHANGE\n",old_state);

		if (fbi == pxafb_main)
		{
			if (old_state != C_DISABLE) 
			{
				if (PXAFB_MAIN_VALID_FB != PXAFB_MAIN_BASE)
				{
					//DISABLE_OVERLAYS(fbi);
					overlay1fb_disable(&(pxafb_main->overlay1fb->fb));
				}
	    			DPRINTK("set_ctrlr_state(C_DISABLE_CLKCHANGE): main_ovl2_status(%d)\n", pxafb_global_state.main_ovl2_status);
				if (pxafb_global_state.main_ovl2_status == PXAFB_OVL2_ON)
					overlay2fb_disable(&(pxafb_main->overlay2fb->fb));
				
				pxafb_disable_controller(fbi);
					
				fbi->state = state;
				DPRINTK("set_ctrlr_state(C_DISABLE_CLKCHANGE): MAIN fbi->state = state(%d)\n", state);
				
				pxafb_global_state.main_state &= 0x01;
				pxafb_global_state.active_panel = PXA_NONE_PANEL;
				pxafb_global_state.pxafb_current = NULL;
			}
		}
#ifdef CONFIG_PXAFB_CLI
		if (fbi == pxafb_smart)
		{
			/* For C_SUSPEND or C_DISABLE, we do nothing for pxafb_smart CSTN panel */
			if (old_state == C_ENABLE)
			{
				pxafb_disable_controller(fbi);
				
				fbi->state = state;
				DPRINTK("set_ctrlr_state(C_DISABLE_CLKCHANGE): CLI fbi->state = state(%d)\n", state);
				
				pxafb_global_state.smart_state &= 0x01; //0b001
				pxafb_global_state.active_panel = PXA_NONE_PANEL;
				pxafb_global_state.pxafb_current = NULL;
			}			
		}
#endif
		break;

	case C_DISABLE:
		/*
		 * Disable controller
		 */
		DPRINTK("set_ctrlr_state:old_state(%d),C_DISABLE\n",old_state);

		if (fbi == pxafb_main)
		{
			if (old_state != C_DISABLE) 
			{
				if (PXAFB_MAIN_VALID_FB != PXAFB_MAIN_BASE)
				{
					//DISABLE_OVERLAYS(fbi);
					overlay1fb_disable(&(pxafb_main->overlay1fb->fb));
				}
	    			DPRINTK("set_ctrlr_state(C_DISABLE): main_ovl2_status(%d)\n", pxafb_global_state.main_ovl2_status);
				if (pxafb_global_state.main_ovl2_status == PXAFB_OVL2_ON)
					overlay2fb_disable(&(pxafb_main->overlay2fb->fb));
				
				if (old_state != C_DISABLE_CLKCHANGE)
					pxafb_disable_controller(fbi);
				
				pxafb_power_down_lcd(fbi);

				fbi->state = state;
				DPRINTK("set_ctrlr_state(C_DISABLE): MAIN fbi->state = state(%d)\n", state);
				
				pxafb_global_state.main_state &= 0x01;
				pxafb_global_state.active_panel = PXA_NONE_PANEL;
				pxafb_global_state.pxafb_current = NULL;
			}
		}
#ifdef CONFIG_PXAFB_CLI
		if (fbi == pxafb_smart)
		{
			if (old_state != C_DISABLE)
			{
				/* CLI panel display off, LCD controller power off. */
				pxafb_disable_controller(fbi);
				pxafb_power_down_lcd(fbi);
				
				fbi->state = state;
				DPRINTK("set_ctrlr_state(C_DISABLE): CLI fbi->state = state(%d)\n", state);

				pxafb_global_state.smart_state &= 0x01;
				pxafb_global_state.active_panel = PXA_NONE_PANEL;
				pxafb_global_state.pxafb_current = NULL;
			}
		}
#endif
		break;

	case C_ENABLE_CLKCHANGE:
		/*
		 * Enable the controller after clock change.  Only
		 * do this if we were disabled for the clock change.
		 */
		DPRINTK("set_ctrlr_state:old_state(%d),C_ENABLE_CLKCHANGE\n",old_state);
		if (fbi == pxafb_main)
		{
			/* global status checking to avoid race condition */
			if (pxafb_global_state.smart_state & PXAFB_PANEL_ON)  //SMART panel is active
			{
				up(&pxafb_global_state.g_sem);
				return;  //-EBUSY
			}
			
			if (old_state == C_DISABLE_CLKCHANGE) 
			{
				fbi->state = C_ENABLE;
				DPRINTK("set_ctrlr_state(C_ENABLE_CLKCHANGE): MAIN fbi->state = C_ENABLE\n");
				
				pxafb_enable_controller(fbi);
				
				if (PXAFB_MAIN_VALID_FB != PXAFB_MAIN_BASE)
				{
					//ENABLE_OVERLAYS(fbi);
					overlay1fb_enable(&(pxafb_main->overlay1fb->fb));
				}
	    			DPRINTK("set_ctrlr_state(C_ENABLE_CLKCHANGE): main_ovl2_status(%d)\n", pxafb_global_state.main_ovl2_status);
				if (pxafb_global_state.main_ovl2_status == PXAFB_OVL2_ON)
					overlay2fb_enable(&(pxafb_main->overlay2fb->fb));
					
				pxafb_global_state.main_state |= 0x06; //0b110
				pxafb_global_state.active_panel = PXA_MAIN_PANEL;
				pxafb_global_state.pxafb_current = pxafb_main;
			}
		}
#ifdef CONFIG_PXAFB_CLI
		if (fbi == pxafb_smart)
		{
			/* global status checking to avoid race condition */
			if (pxafb_global_state.main_state & PXAFB_PANEL_ON) //MAIN panel is active
			{
				up(&pxafb_global_state.g_sem);
				return;  // -EBUSY;
			}
			if (old_state == C_DISABLE_CLKCHANGE) 
			{
				fbi->state = C_ENABLE;
				DPRINTK("set_ctrlr_state(C_ENABLE_CLKCHANGE): CLI fbi->state = C_ENABLE\n");

				pxafb_setup_gpio(fbi);
				pxafb_power_up_lcd(fbi);
				pxafb_enable_controller(fbi);
				
				pxafb_global_state.smart_state |= 0x06; //0b110
				pxafb_global_state.active_panel = PXA_SMART_PANEL;
				pxafb_global_state.pxafb_current = pxafb_smart;
			}
		}
#endif
		break;

	case C_REENABLE:
		/*
		 * Re-enable the controller only if it was already
		 * enabled.  This is so we reprogram the control
		 * registers.  Don't need to power off and remove clock? -- Susan
		 */
		DPRINTK("set_ctrlr_state:old_state(%d),C_REENABLE\n",old_state);

		if (fbi == pxafb_main)
		{
			DPRINTK("set_ctrlr_state(C_REENABLE): MAIN fbi->state (%d)\n", old_state);
			if (old_state == C_ENABLE) 
			{		
				if (PXAFB_MAIN_VALID_FB != PXAFB_MAIN_BASE)
				{
					//DISABLE_OVERLAYS(fbi);
					overlay1fb_disable(&(pxafb_main->overlay1fb->fb));
				}
	    			DPRINTK("set_ctrlr_state(C_REENABLE): main_ovl2_status(%d)\n", pxafb_global_state.main_ovl2_status);
				if (pxafb_global_state.main_ovl2_status == PXAFB_OVL2_ON)
					overlay2fb_disable(&(pxafb_main->overlay2fb->fb));
				
				
				pxafb_disable_controller(fbi);
				pxafb_setup_gpio(fbi);
				pxafb_enable_controller(fbi);
				
				
				if (PXAFB_MAIN_VALID_FB != PXAFB_MAIN_BASE)
				{
					//ENABLE_OVERLAYS(fbi);
					overlay1fb_enable(&(pxafb_main->overlay1fb->fb));
				}
	    			DPRINTK("set_ctrlr_state(C_REENABLE): main_ovl2_status(%d)\n", pxafb_global_state.main_ovl2_status);
				if (pxafb_global_state.main_ovl2_status == PXAFB_OVL2_ON)
					overlay2fb_enable(&(pxafb_main->overlay2fb->fb));
			}
		}
#ifdef CONFIG_PXAFB_CLI
		if (fbi == pxafb_smart)
		{
			DPRINTK("set_ctrlr_state(C_REENABLE): CLI fbi->state (%d)\n", old_state);
			if (old_state == C_ENABLE) 
			{		
				pxafb_disable_controller(fbi);
				pxafb_setup_gpio(fbi);
				pxafb_enable_controller(fbi);
			}
		}
#endif
		break;

	case C_ENABLE:
		/*
		 * Power up the LCD screen, enable controller, and
		 * turn on the backlight.
		 */
		DPRINTK("set_ctrlr_state:old_state(%d),C_ENABLE\n",old_state);

		if (fbi == pxafb_main)
		{
			/* Global status checking to avoid race condition */
			if (pxafb_global_state.smart_state & PXAFB_PANEL_ON) //SMART panel is active //
			{
				up(&pxafb_global_state.g_sem);
				return;  // -EBUSY;
			}
			if (old_state != C_ENABLE) 
			{
				fbi->state = C_ENABLE;
				DPRINTK("set_ctrlr_state(C_ENABLE): MAIN fbi->state = C_ENABLE\n");

				pxafb_setup_gpio(fbi);					
			#ifdef CONFIG_LOGO_SHOWED_BY_MBM
				/*
				*During powering up , the kernel can't do the following HW related init, 
				*otherwise, logo showing seamlessly can't be attained.
				*/
				if(!powering_up){
					pxafb_power_up_lcd(fbi);
					pxafb_enable_controller(fbi);
				}
			#else
				pxafb_power_up_lcd(fbi);
				pxafb_enable_controller(fbi);
			#endif
			
				if (PXAFB_MAIN_VALID_FB != PXAFB_MAIN_BASE)
				{
					DPRINTK("case C_ENABLE:ENABLE_OVERLAYS(fbi);\n");
					//ENABLE_OVERLAYS(fbi);
					overlay1fb_enable(&(pxafb_main->overlay1fb->fb));
				}
	    			DPRINTK("set_ctrlr_state(C_ENABLE): main_ovl2_status(%d)\n", pxafb_global_state.main_ovl2_status);
				if (pxafb_global_state.main_ovl2_status == PXAFB_OVL2_ON)
					overlay2fb_enable(&(pxafb_main->overlay2fb->fb));

	    			DPRINTK("After FBIOENABLEOVL2 -- LCCR0(0x%x)LCCR1(0x%x)LCCR2(0x%x)LCCR3(0x%x)LCCR4(0x%x)LCCR5(0x%x)OVL2C1(0x%x)OVL2C2(0x%x)\n", LCCR0,LCCR1,LCCR2,LCCR3,LCCR4,LCCR5,OVL2C1,OVL2C2);

				//if (handshake_pass())
					//mod_timer(&(fbi->bklight_timer), jiffies + bk_timeout);
				
				pxafb_global_state.active_panel = PXA_MAIN_PANEL;
				pxafb_global_state.main_state |= 0x06; //0b110
				pxafb_global_state.pxafb_current = pxafb_main;

			}
		}
#ifdef CONFIG_PXAFB_CLI
		if (fbi == pxafb_smart)
		{
			/* G	lobal status checking to avoid race condition */
			if (pxafb_global_state.main_state & PXAFB_PANEL_ON) //MAIN panel is active //
			{
				up(&pxafb_global_state.g_sem);
				return;  // -EBUSY;
			}
			if (old_state != C_ENABLE) 
			{
				fbi->state = C_ENABLE;
				DPRINTK("set_ctrlr_state(C_ENABLE): CLI fbi->state = C_ENABLE\n");

				pxafb_setup_gpio(fbi);
				pxafb_power_up_lcd(fbi);
				pxafb_enable_controller(fbi);
				
				//if (handshake_pass())
				//	mod_timer(&(fbi->bklight_timer), jiffies + bk_timeout);

				pxafb_global_state.active_panel = PXA_SMART_PANEL;
				pxafb_global_state.smart_state |= 0x06; //0b110
				pxafb_global_state.pxafb_current = pxafb_smart;	
				
			}
		}
#endif
		break;
#ifdef CONFIG_PXAFB_CLI
	case C_SUSPEND:
	
		/*
		 * Suspend CLI
		 */
		DPRINTK("set_ctrlr_state:old_state(%d),C_SUSPEND\n",old_state);

		if (fbi == pxafb_smart)
		{
			if (old_state == C_ENABLE)
			{
				/* LCD Controller disabled, CLI suspend mode,LCD power off */
				pxafb_suspend_controller(fbi);
				pxafb_power_down_lcd(fbi);
				
				fbi->state = state;
				DPRINTK("set_ctrlr_state(C_SUSPEND): CLI fbi->state = state(%d)\n", state);

				pxafb_global_state.smart_state &= 0x05; //0b101
				pxafb_global_state.active_panel = PXA_NONE_PANEL;
				pxafb_global_state.pxafb_current = NULL;
			}
		}
		
		/*
		* For pxafb_main panel, do nothing with C_SUSPEND
		*/
		break;

	case C_ENTER_PWRSAVE: 
		/*
		 * CLI:  enter partial mode
		 */
		DPRINTK("set_ctrlr_state:old_state(%d),C_ENTER_PWRSAVE\n",old_state);
		
		/*Check if the smart panel is active */
		if ( (pxafb_global_state.active_panel != PXA_SMART_PANEL) || ((pxafb_global_state.smart_state & 0x6) == 0x0) )
		{
			up(&pxafb_global_state.g_sem);
			return -EIO;
		}	

		if (fbi == pxafb_smart)
		{
			if (old_state == C_ENABLE)
			{
				/* LCD Controller disable, enter partial mode, LCD power off. */
				pxafb_powersave_controller(fbi);
				pxafb_power_down_lcd(fbi);
				
				fbi->state = state;
				DPRINTK("set_ctrlr_state(C_ENTER_PWRSAVE): CLI fbi->state = state(%d)\n", state);

				pxafb_global_state.smart_state &= 0x05; //panel off, other remains
				pxafb_global_state.smart_state |= PXAFB_PARTIAL_DISP_ON;//set partial mode state bit
				pxafb_global_state.active_panel = PXA_NONE_PANEL;
				pxafb_global_state.pxafb_current = NULL;

				DPRINTK ("pxafb_global_state.smart_state = %x \n ",pxafb_global_state.smart_state);
		
			}
		}
		
		/*
		* For pxafb_main panel, do nothing with C_ENTER_PWRSAVE
		*/
		break;
			
#endif
	}
	//up(&fbi->ctrlr_sem);
	up(&pxafb_global_state.g_sem);  //Susan

	DPRINTK("set_ctrlr_state(%d),return successfully\n",current->pid);
}

/*
 * Our LCD controller task (which is called when we blank or unblank)
 * via keventd.
 */
static void pxafb_task(void *dummy)
{
	struct pxafb_info *fbi = dummy;
	u_int state = xchg(&fbi->task_state, -1);

	set_ctrlr_state(fbi, state);
}

#ifdef CONFIG_PXAFB_CLI
/*These functions are only for pxafb_smart.smart_refresh_task */
static void pxafb_smart_load_cmd(struct pxafb_info *fbi, unsigned short *cmd_buf, int cmd_num)
{
	unsigned short * cmd_ptr = (unsigned short *) fbi->cmd_buf_cpu;
	unsigned int cnt = 0;
	unsigned int cmd_size_byte = 0, dma_size_byte = 0;

	DPRINTK("pxafb_smart_load_cmd, fbi(0x%x), cmd_num(%d)\n", fbi, cmd_num);
	
	cmd_size_byte = sizeof(unsigned short) * (cmd_num + 2);
	dma_size_byte = cmd_size_byte;
	if (cmd_size_byte % 4)
		dma_size_byte = cmd_size_byte + (4 - cmd_size_byte % 4);
	
	while(cnt < cmd_num)
		*cmd_ptr++ = cmd_buf[cnt++];
	/* Insert the "Interrupt Processor" and"Wait for Vsync" command */
	*cmd_ptr++ = LCD_CMD_INT_PROC | LCD_CMD_A0_COMMAND;
	*cmd_ptr++ = LCD_CMD_WAIT_FOR_VSYNC | LCD_CMD_A0_COMMAND;
	
	fbi->dmadesc_cmd_cpu->ldcmd = dma_size_byte;
	fbi->dmadesc_cmd_cpu->fdadr = fbi->dmadesc_cmd_dma;	

	DPRINTK("pxafb_smart_load_cmd, exit\n");
	
}

static int pxafb_smart_send_cmd(struct pxafb_info *fbi)
{
	/* Cannot change LCD state while sending cmd */
	//Susan -- down_interruptible(&pxafbi->ctrlr_sem);
	
	DPRINTK("Enter pxafb_smart_send_cmd\n");
	if (fbi->state != C_ENABLE) {	
		printk(KERN_WARNING"CLI not enabled. Cannot send command\n");
		return -1;
	}
	
	DPRINTK("pxafb_smart_send_cmd: FDADR0(0x%x)\n", FDADR0);

	/* Need to reset the dma control registers */
	PRSR |= PRSR_ST_OK | PRSR_CON_NT;
	FDADR6 = fbi->fdadr6;

#if (0)
	DPRINTK("before enable \n");	
	DPRINTK("PRSR: 0x%08x", (unsigned int)PRSR);
	DPRINTK("LCSR0: 0x%08x", (unsigned int)LCSR0);
	DPRINTK("LCSR1: 0x%08x", (unsigned int)LCSR1);
	DPRINTK("FDADR0: 0x%08x", (unsigned int)FDADR0);
	DPRINTK("FDADR1: 0x%08x", (unsigned int)FDADR1);
	DPRINTK("FDADR6: 0x%08x", (unsigned int)FDADR6);
	DPRINTK("FIDR6: 0x%08x", (unsigned int)FIDR6);
	DPRINTK("FSADR6: 0x%08x", (unsigned int)FSADR6);
	DPRINTK("LDCMD6: 0x%08x", (unsigned int)LDCMD6);
	DPRINTK("FBR6: 0x%08x", (unsigned int)FBR6);
    DPRINTK("fbi->fdadr0: 0x%08x", fbi->fdadr0);
    DPRINTK("fbi->fdadr6: 0x%08x", fbi->fdadr6);
    DPRINTK("fbi->dmadesc_cmd_cpu->fdadr: 0x%08x", fbi->dmadesc_cmd_cpu->fdadr);
    DPRINTK("fbi->dmadesc_cmd_cpu->fsadr: 0x%08x", fbi->dmadesc_cmd_cpu->fsadr);
    DPRINTK("fbi->dmadesc_cmd_cpu->fidr: 0x%08x", fbi->dmadesc_cmd_cpu->fidr);
    DPRINTK("fbi->dmadesc_cmd_cpu->ldcmd: 0x%08x\n", fbi->dmadesc_cmd_cpu->ldcmd);
#endif
	
	LCCR0 |= LCCR0_ENB;

	/* Wait until all command send to FIFO, executed, and date output finished. */
	/* cnt is for Debug only. It is used as a timer */

	//unsigned long long cnt = 0;
	while ((LCSR0 & LCSR0_CMD_INT) == 0)

	DPRINTK("pxafb_smart_send_cmd, pass (LCSR0 & LCSR0_CMD_INT),GPLR2(0x%x)\n", GPDR2);
	
	if (0)  //if ( FDADR0 == fbi->fdadr0)  //if (0) //
	{
		while ( (LCSR0 & LCSR0_EOF) == 0 );  //If during frame data write, wait until EOF0 happens -- Susan//
		LCSR0 |= LCSR0_EOF;
	}
	
#if (0)
	DPRINTK("After LCSR0_CMD_INT set ");	
	//DPRINTK("cnt: %llu", cnt);	
	DPRINTK("PRSR: 0x%08x", (unsigned int)PRSR);
	DPRINTK("LCSR0: 0x%08x", (unsigned int)LCSR0);
	DPRINTK("LCSR1: 0x%08x", (unsigned int)LCSR1);
	DPRINTK("FDADR0: 0x%08x", (unsigned int)FDADR0);
	DPRINTK("FDADR1: 0x%08x", (unsigned int)FDADR1);
	DPRINTK("FDADR6: 0x%08x", (unsigned int)FDADR6);
	DPRINTK("FIDR6: 0x%08x", (unsigned int)FIDR6);
	DPRINTK("FSADR6: 0x%08x", (unsigned int)FSADR6);
	DPRINTK("LDCMD6: 0x%08x", (unsigned int)LDCMD6);
	DPRINTK("FBR6: 0x%08x", (unsigned int)FBR6);
        DPRINTK("fbi->fdadr6: 0x%08x", fbi->fdadr6);
        DPRINTK("fbi->dmadesc_cmd_cpu->fdadr: 0x%08x", fbi->dmadesc_cmd_cpu->fdadr);
        DPRINTK("fbi->dmadesc_cmd_cpu->fsadr: 0x%08x", fbi->dmadesc_cmd_cpu->fsadr);
        DPRINTK("fbi->dmadesc_cmd_cpu->fidr: 0x%08x", fbi->dmadesc_cmd_cpu->fidr);
        DPRINTK("fbi->dmadesc_cmd_cpu->ldcmd: 0x%08x", fbi->dmadesc_cmd_cpu->ldcmd);
#endif
	
	/* Quick disable */
	PRSR &= ~(PRSR_ST_OK | PRSR_CON_NT);
	LCSR0 |= LCSR0_CMD_INT; /* Clear CMD_INT */
	
	LCCR0 &= ~(LCCR0_ENB);  /*Quick disable */
	
	FDADR6 = 0;	/* Disable DMA 6*/
	//Susan -- up(&pxafbi->ctrlr_sem);
	return 0;
}
#endif

#if (0) //move pxafb_smart_refresh to product.c (barbados.c) //
static int pxafb_smart_refresh(void *dummy)
{
	struct pxafb_info *fbi = (struct pxafb_info *)dummy;
	int ret = 0;

	DPRINTK("pxafb_smart_refresh(%d, time --%d)\n", current->pid, jiffies);

	/* Check if LCD controller has been removed from pxafb_smart CSTN panel */
	if (pxafb_global_state.smart_state & PXAFB_PANEL_ON)
	{
		if (down_trylock(&pxafb_global_state.g_sem))
		{
			DPRINTK("pxafb_smart_refresh(%d), down_trylock fail\n", current->pid);
			return -EAGAIN;
		}	
		ret = pxafb_smart_send_frame_data(pxafb_smart);
//		ret = pxafb_smart_send_display_data(fbi);

		schedule_task(&pxafb_smart->smart_refresh_task);  //schedule_task in context_switch kernel thread //
		
 		up(&pxafb_global_state.g_sem);
	}
	
	DPRINTK("pxafb_smart_refresh(%d), return.\n", current->pid);
	
	return ret;
}
#endif

#ifdef CONFIG_CPU_FREQ
/*
 * CPU clock speed change handler.  We need to adjust the LCD timing
 * parameters when the CPU clock is adjusted by the power management
 * subsystem.
 */
static int
pxafb_clkchg_notifier(struct notifier_block *nb, unsigned long val,
			 void *data)
{
	struct pxafb_info *fbi = TO_INF(nb, clockchg);
	u_int pcd;

	switch (val) {
	case CPUFREQ_MINMAX:
		/* todo: fill in min/max values */
		break;

	case CPUFREQ_PRECHANGE:
		set_ctrlr_state(fbi, C_DISABLE_CLKCHANGE);
		break;

	case CPUFREQ_POSTCHANGE:
		pcd = get_pcd(fbi->fb.var.pixclock);
		fbi->reg_lccr3 = (fbi->reg_lccr3 & ~0xff) | LCCR3_PixClkDiv(pcd);
		set_ctrlr_state(fbi, C_ENABLE_CLKCHANGE);
#ifdef CONFIG_CPU_BULVERDE
		set_ctrlr_state(fbi, C_REENABLE);
#endif
		break;
	}
	return 0;
}
#endif
#if (0) 
#ifdef CONFIG_PM
/*
 * Power management hook.  Note that we won't be called from IRQ context,
 * unlike the blank functions above, so we may sleep.
 */
static int
pxafb_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
{
	struct pxafb_info *fbi = pm_dev->data;

	DPRINTK("pm_callback: %d", req);

	if (req == PM_SUSPEND || req == PM_RESUME) {
		int state = (int)data;

		if (state == 0) {
			/* Enter D0. */
			set_ctrlr_state(fbi, C_ENABLE);
#ifdef CONFIG_CPU_BULVERDE
			set_ctrlr_state(fbi, C_REENABLE);
#endif
		} else {
			/* Enter D1-D3.  Disable the LCD controller.  */
			set_ctrlr_state(fbi, C_DISABLE);
		}
	}
	DPRINTK("done");
	return 0;
}
#endif
#endif
/*
 * pxafb_map_video_memory():
 *      Allocates the DRAM memory for the frame buffer.  This buffer is  
 *	remapped into a non-cached, non-buffered, memory region to  
 *      allow palette and pixel writes to occur without flushing the 
 *      cache.  Once this area is remapped, all virtual memory
 *      access to the video memory should occur at the new region.
 */
static int __init pxafb_map_video_memory(struct pxafb_info *fbi)
{
	u_long palette_mem_size;

	/*
	 * We reserve one page for the palette, plus the size
	 * of the framebuffer.
	 *
	 * layout of stuff in memory
	 *
	 *                fblow descriptor
	 *                fbhigh descriptor
	 *                palette descriptor
	 *                palette
	 *   page boundary->
	 *                frame buffer
	 */

	fbi->map_size = PAGE_ALIGN(fbi->fb.fix.smem_len + PAGE_SIZE);
	
#ifdef CONFIG_CPU_BULVERDE

	if (fbi == pxafb_main)  //This is for main panel //
	{
		fbi->total_sram_usage = 0;	
		if (fbi->map_size < 0x40000) 
		{
			DPRINTK("try to put fb in SRAM...\n");
			if (sram_access_obtain(&sram_start, &sram_size) < 0) 
			{
				printk(KERN_WARNING "pxafb: can't access SRAM for fb\n");
				fbi->map_cpu = __consistent_alloc
					(GFP_KERNEL, fbi->map_size,
					 &fbi->map_dma, PTE_BUFFERABLE);
			}
			else
			{
				DPRINTK("SRAM fb ok\n");
				fbi->map_cpu = (u_char *)sram_start;
				fbi->map_dma = (dma_addr_t)SRAM_MEM_PHYS;
				fbi->total_sram_usage += fbi->map_size;
			}
		}
		else
			fbi->map_cpu = __consistent_alloc(GFP_KERNEL, fbi->map_size, &fbi->map_dma, PTE_BUFFERABLE);
	}

#ifdef CONFIG_PXAFB_CLI
	if (fbi == pxafb_smart)  //This is for the CLI panel //
	{
		fbi->total_sram_usage = 0;	
		if ( (fbi->map_size + pxafb_main->total_sram_usage) < 0x40000) 
		{
			DPRINTK("try to put fb in SRAM...\n");
			if (sram_access_obtain(&sram_start, &sram_size) < 0) 
			{
				printk(KERN_WARNING "pxafb: can't access SRAM for fb\n");
				fbi->map_cpu = __consistent_alloc
					(GFP_KERNEL, fbi->map_size,
					 &fbi->map_dma, PTE_BUFFERABLE);
			}
			else 
			{
				DPRINTK("SRAM fb ok\n");
				fbi->map_cpu = (u_char *)sram_start + pxafb_main->total_sram_usage;
				fbi->map_dma = (dma_addr_t)(SRAM_MEM_PHYS + pxafb_main->total_sram_usage);
				fbi->total_sram_usage += fbi->map_size;
			}
		}
		else
			fbi->map_cpu = __consistent_alloc(GFP_KERNEL, fbi->map_size, &fbi->map_dma, PTE_BUFFERABLE);
	}
#endif //#ifdef CONFIG_PXAFB_CLI

#endif

	if (fbi->map_cpu) 
	{
		#ifndef CONFIG_LOGO_SHOWED_BY_MBM
		/*
		* If the logo is showed by MBM, it has been filled with logo,
		* so don't clear it here. Otherwise black screen will be showed.
		*/
		memset((void *)fbi->map_cpu, 0, fbi->map_size);
		#endif
		
		fbi->screen_cpu = fbi->map_cpu + PAGE_SIZE;
		fbi->screen_dma = fbi->map_dma + PAGE_SIZE;
		fbi->fb.fix.smem_start = fbi->screen_dma;

		fbi->palette_size = 1 << 8;  //This is the maximum number of palette entries --Susan //
		//palette_mem_size = fbi->palette_size * sizeof(u16);
		/* FIXME: May need two palettes, one for virtual console, one for display */
		palette_mem_size = fbi->palette_size * sizeof(u32);  //This is the maximum palette entries case -- Susan //

		DPRINTK("palette_mem_size = 0x%08lx", (u_long) palette_mem_size);

		fbi->palette_cpu = (u16 *)(fbi->map_cpu + PAGE_SIZE - palette_mem_size);
		fbi->palette_dma = fbi->map_dma + PAGE_SIZE - palette_mem_size;

		fbi->first_fb.map_cpu = fbi->map_cpu;
		fbi->first_fb.map_dma = fbi->map_dma;
		fbi->first_fb.screen_cpu = fbi->screen_cpu;
		fbi->first_fb.screen_dma = fbi->screen_dma;
		fbi->first_fb.dmadesc_fb_cpu = (struct pxafb_dma_descriptor *)((unsigned int)fbi->palette_cpu - 2*16);
		fbi->first_fb.dmadesc_fb_dma = fbi->palette_dma - 2*16;

		fbi->second_fb.map_cpu = NULL;
	}
#ifdef CONFIG_PXAFB_CLI
	/* The following part is related to smart panel's cmd buffer channel setup */
	if (fbi == pxafb_smart)  //This is the smart panel //
	{
		unsigned short *cmd_ptr;
		unsigned int cmd_cnt;
		
		/* Smart LCD specific command DMA setting. 
		 * It is put before the other three DMA descriptors */	
		fbi->dmadesc_cmd_cpu = (struct pxafb_dma_descriptor *)((unsigned int)fbi->palette_cpu - 4*16);
		fbi->dmadesc_cmd_dma = fbi->palette_dma - 4*16;

		fbi->cmd_buf_maxsize = sizeof(unsigned short) * CMD_BUF_MAX_NUM;
		
		fbi->cmd_buf_cpu = __consistent_alloc(GFP_KERNEL, fbi->cmd_buf_maxsize, &fbi->cmd_buf_dma, 0);
	    if (!fbi->cmd_buf_cpu)
	    {
			consistent_free((void *)fbi->map_cpu, fbi->map_size, fbi->map_dma);
	        return -ENOMEM;
	    }

		memset(fbi->cmd_buf_cpu, 0, fbi->cmd_buf_maxsize);
		
		/* Fill the command buffer with NOP */
		cmd_ptr = (unsigned short *)fbi->cmd_buf_cpu;
		cmd_cnt = fbi->cmd_buf_maxsize / sizeof(unsigned short);
		
		while (cmd_cnt-- > 0) 
		{
			*cmd_ptr++ = LCD_CMD_NOP;
		}
        
	    fbi->dmadesc_cmd_cpu->fsadr = fbi->cmd_buf_dma;
	    fbi->dmadesc_cmd_cpu->fidr = 0;
	    fbi->fdadr6 = fbi->dmadesc_cmd_dma;
	}
#endif

	return fbi->map_cpu ? 0 : -ENOMEM;
}

/* Fake monspecs to fill in fbinfo structure */
static struct fb_monspecs monspecs __initdata = {
	30000, 70000, 50, 65, 0	/* Generic */
};

static int dummy_updatevar(int con, struct fb_info *info)
{
	DPRINTK("entered");
	return 0;
}

#ifdef CONFIG_CPU_BULVERDE
/* 
 * LCD enhancement : Overlay 1 
 *
 * Features:
 * - support 16bpp (No palatte)
 */
static int overlay1fb_open(struct fb_info *info, int user)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	int ret = 0;

	down(&fbi->mutex);

	if (fbi->refcount) 
		ret = -EACCES;
	else
		fbi->refcount ++;

	up(&fbi->mutex);

	/* Initialize the variables in overlay1 framebuffer. */
	fbi->fb.var.xres = fbi->fb.var.yres = 0;
	fbi->fb.var.bits_per_pixel = 0;

	return ret;
}

static int overlay1fb_release(struct fb_info *info, int user)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	down(&fbi->mutex);

	if (fbi->refcount)
		fbi->refcount --;

	up(&fbi->mutex);
	/* disable overlay when released */
	overlay1fb_blank(1, info);

	/* Memory not needed */
	consistent_free((void *)fbi->map_cpu, fbi->map_size, fbi->map_dma);
	/* FIXME: fbi->map_cpu is not NULL after free */
	fbi->map_cpu = 0;
	return 0;
}

/*
 * Overlay 1: 16 bpp, 24 bpp (no palette)
 */
static int 
overlay1fb_map_video_memory(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;

	DPRINTK("overlay1fb_map_video_memory(%d):info->modename(%d),fbi->map_cpu(0x%x)\n",current->pid,info->modename[0],fbi->map_cpu);

	if (fbi->map_cpu)
		consistent_free((void*)fbi->map_cpu, fbi->map_size, fbi->map_dma);

	fbi->map_size = PAGE_ALIGN(fbi->fb.fix.smem_len + PAGE_SIZE);
	DPRINTK("overlay1fb_map_video_memory(%d):fbi->map_size(%d)\n",current->pid, (unsigned int)(fbi->map_size));
	
#ifdef CONFIG_PXAFB_CLI
	if ( (fbi->map_size + pxafb_main->total_sram_usage + pxafb_smart->total_sram_usage) < 0x40000) 
#else
	if ( (fbi->map_size + pxafb_main->total_sram_usage) < 0x40000) 
#endif
	{
		DPRINTK("try to put overlay1fb in SRAM...\n");
		if (sram_access_obtain(&sram_start, &sram_size) < 0) 
		{
				printk(KERN_WARNING "pxafb: can't access SRAM for fb\n");
				fbi->map_cpu = (unsigned long )__consistent_alloc(GFP_KERNEL,(size_t)fbi->map_size,(dma_addr_t *)(&(fbi->map_dma)),PTE_BUFFERABLE);
		}
		else
		{
				DPRINTK("SRAM fb ok\n");
#ifdef CONFIG_PXAFB_CLI
				fbi->map_cpu = (unsigned long )(sram_start + (unsigned long)(pxafb_main->total_sram_usage) + (unsigned long)(pxafb_smart->total_sram_usage));
				fbi->map_dma = (dma_addr_t)(SRAM_MEM_PHYS + pxafb_main->total_sram_usage + pxafb_smart->total_sram_usage);
#else
				fbi->map_cpu = (unsigned long)(sram_start + (unsigned long)(pxafb_main->total_sram_usage));
				fbi->map_dma = (dma_addr_t)(SRAM_MEM_PHYS + pxafb_main->total_sram_usage);
#endif
				pxafb_main->total_sram_usage += fbi->map_size;  //only MAIN panel cares about OVERLAY1//
		}
	}
	else
		fbi->map_cpu = (unsigned long )__consistent_alloc(GFP_KERNEL, (size_t)(fbi->map_size), (dma_addr_t *)(&(fbi->map_dma)), PTE_BUFFERABLE);
	DPRINTK("overlay1fb_map_video_memory(%d):fbi->map_cpu(0x%x)\n",current->pid,(unsigned int)(fbi->map_cpu));
	
	if (!fbi->map_cpu)
	{
		/* Right now We only think about main panel with overlay1 framebuffer being exported to user space.  -- Susan */
		if ( fbi->fb.modename[0] == PXAFB_MAIN_VALID_FB)
		{
			pxafb_main->first_fb.map_cpu = (u_char *)NULL;
			pxafb_main->first_fb.map_dma = (dma_addr_t)NULL;
			pxafb_main->first_fb.screen_cpu = (u_char *)NULL;
			pxafb_main->first_fb.screen_dma = (dma_addr_t)NULL;

			pxafb_main->second_fb.map_cpu = (u_char *)NULL;
		}
		return -ENOMEM;
	}

	memset((void *)fbi->map_cpu, 0, fbi->map_size);
	fbi->screen_cpu = (unsigned long )((unsigned long)(fbi->map_cpu) + (unsigned long)PAGE_SIZE);
	fbi->screen_dma = (dma_addr_t)(fbi->map_dma + PAGE_SIZE);
	
	fbi->fb.fix.smem_start = (unsigned long)(fbi->screen_dma);

	/* setup dma descriptor */
	fbi->dma1 = (struct pxafb_dma_descriptor*)
		(fbi->screen_cpu - sizeof(struct pxafb_dma_descriptor));

	fbi->dma1->fdadr = (fbi->screen_dma - sizeof(struct pxafb_dma_descriptor));
	fbi->dma1->fsadr = fbi->screen_dma;
	fbi->dma1->fidr  = 0;
	fbi->dma1->ldcmd = fbi->fb.fix.smem_len | 0x200000;  //Enable LDCMD1[EOFINT1]

	/* Right now We only think about main panel with double buffers support.  This can be modified easily for CLI panel double buffer supports.  -- Susan */
	if ( fbi->fb.modename[0] == PXAFB_MAIN_VALID_FB)
	{
		pxafb_main->first_fb.map_cpu = (u_char *)(fbi->map_cpu);
		pxafb_main->first_fb.map_dma = fbi->map_dma;
		pxafb_main->first_fb.screen_cpu = (u_char *)(fbi->screen_cpu);
		pxafb_main->first_fb.screen_dma = fbi->screen_dma;
		pxafb_main->first_fb.dmadesc_fb_dma = fbi->screen_dma - sizeof(struct pxafb_dma_descriptor);
		pxafb_main->first_fb.dmadesc_fb_cpu = fbi->dma1;

		pxafb_main->fdadr1 = pxafb_main->first_fb.dmadesc_fb_dma;

		pxafb_main->second_fb.map_cpu = (u_char *)(NULL);

		fbi->fb.disp->screen_base = pxafb_main->first_fb.screen_cpu;
	}
	
	DPRINTK("overlay1fb_map_video_memory(%d):fbi->map_cpu(0x%x)\n",current->pid,fbi->map_cpu);
	DPRINTK("overlay1fb_map_video_memory(%d):fbi->map_dma(0x%x)\n",current->pid,fbi->map_dma);
	DPRINTK("overlay1fb_map_video_memory(%d):fbi->dma1->fdadr(0x%x)\n",current->pid,fbi->dma1->fdadr);
	DPRINTK("overlay1fb_map_video_memory(%d):fbi->dma1->fsadr(0x%x)\n",current->pid,fbi->dma1->fsadr);
	DPRINTK("overlay1fb_map_video_memory(%d):fbi->dma1->fidr(0x%x)\n",current->pid,fbi->dma1->fidr);
	DPRINTK("overlay1fb_map_video_memory(%d):fbi->dma1->ldcmd(0x%x)\n",current->pid,fbi->dma1->ldcmd);
	DPRINTK("overlay1fb_map_video_memory(%d):pxafb_main->first_fb.map_cpu(0x%x)\n",current->pid,fbi->map_cpu);
	DPRINTK("overlay1fb_map_video_memory(%d):pxafb_main->first_fb.map_dma(0x%x)\n",current->pid,fbi->map_dma);
	DPRINTK("overlay1fb_map_video_memory(%d):pxafb_main->first_fb.screen_cpu(0x%x)\n",current->pid,fbi->screen_cpu);
	DPRINTK("overlay1fb_map_video_memory(%d):pxafb_main->first_fb.screen_dma(0x%x)\n",current->pid,fbi->screen_dma);
	DPRINTK("overlay1fb_map_video_memory(%d):pxafb_main->first_fb.dmadesc_fb_dma(0x%x)\n",current->pid,(fbi->screen_dma - sizeof(struct pxafb_dma_descriptor)) );

	DPRINTK("overlay1fb_map_video_memory(%d):pxafb_main->fdadr1(0x%x)\n",current->pid,pxafb_main->fdadr1);

	return 0;
}

static int overlay1fb_enable(struct fb_info *info) 
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	unsigned long bpp1;

	if (fbi->basefb->state != C_ENABLE) return 0;
	
	if (!fbi->map_cpu) return -EINVAL;

	DPRINTK("overlay1fb_enable called: %d", fbi->state);
	
	switch(fbi->fb.var.bits_per_pixel){
	case 16:
		bpp1 = 0x4;
		break;
	case 18:
		bpp1 = 0x6;
		break;
	case 19:
		bpp1 = 0x8;
		break;
	case 24:
		bpp1 = 0x9;
		break;
	case 25:
		bpp1 = 0xa;
		break;
	default:
		printk(KERN_ERR "pxafb: Overlay1 unsupported bpp: %d\n", 	fbi->fb.var.bits_per_pixel);
		return -EINVAL;
	}

	/* disable branch/start/end of frame interrupt */
//ori	LCCR5 |= (LCCR5_IUM1 | LCCR5_BSM1 | LCCR5_EOFM1 | LCCR5_SOFM1);
	LCCR5 |= (LCCR5_IUM1 | LCCR5_BSM1 | LCCR5_SOFM1);

	/* enable overlay 1 window */
	OVL1C2 = (fbi->ypos << 10) | fbi->xpos;
	OVL1C1 = OVL1C1_O1EN | (bpp1 << 20) | ((fbi->fb.var.yres-1)<<10) | (fbi->fb.var.xres-1);

	if (fbi->state == C_DISABLE)
 		FDADR1 = (fbi->dma1->fdadr);
	else
 		FBR1 = fbi->dma1->fdadr | 0x1;

	fbi->state = C_ENABLE;

	DPRINTK("overlay1fb_enable:LCCR0(0x%x),LCCR5(0x%x),OVL1C1(0x%x),OVL1C2(0x%x),FDADR1(0x%x)\n",LCCR0,LCCR5,OVL1C1,OVL1C2,FDADR1);

	return 0;
}

static int overlay1fb_disable(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*)info;
	int done;

	if (fbi->basefb->state != C_ENABLE) return 0;  //This means overlay1fb has been disabled //

	if (fbi->state == C_DISABLE) return 0;

	DPRINTK("overlay1fb_disable called: %d", fbi->state);

	fbi->state = C_DISABLE;
 
 	/* clear O1EN */
	OVL1C1 &= ~OVL1C1_O1EN;

	CLEAR_LCD_INTR(LCSR1, LCSR1_BS1);
	FBR1 = 0x3;
	done = WAIT_FOR_LCD_INTR(LCSR1, LCSR1_BS1, 100);

	if (!done) 
	{
		DPRINTK(KERN_INFO "%s:(%s) timeout\n", __FUNCTION__,current->comm);
		return -1;
	}
	DPRINTK("overlay1fb_disable:OVL1C1(0x%x)\n",OVL1C1);
	return 0;
}


static void overlay1fb_blank(int blank, struct fb_info *info)
{
	//down(&fbi->basefb->ctrlr_sem); -- we are already in the flow of set_ctrlr_state
	switch(blank) 
	{
	case 0:
		overlay1fb_enable(info);
		break;
	case 1:
		/* If cannot shutdown cleanly, just ignore */
		overlay1fb_disable(info);
		break;
	default:
		/* reserved */
		break;
	}
	//up(&fbi->basefb->ctrlr_sem);
}


 
static int
overlay1fb_get_fix(struct fb_fix_screeninfo *fix, int con, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;

	*fix = fbi->fb.fix;

	return 0;
}

static int
overlay1fb_get_var(struct fb_var_screeninfo *var, int con, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	*var = fbi->fb.var;

	var->nonstd = (fbi->ypos << 10) | fbi->xpos;

	return 0;
}

static int 
overlay1fb_validate_var( struct fb_var_screeninfo *var, struct overlayfb_info *fbi)
{
	int xpos, ypos;

	/* must in base frame */
	xpos = (var->nonstd & 0x3ff);
	ypos = ((var->nonstd>>10) & 0x3ff);

	if ( (xpos + var->xres) > fbi->basefb->max_xres )
		return -EINVAL;

	if ( (ypos + var->yres) > fbi->basefb->max_yres ) 
		return -EINVAL;

	return 0;
}

/*This API is a pure set_var function without map_video_memory functionality  -- Susan*/
/* It should be directly invoked  by pxafb_set_var in the case that overlay1 framebuffer should be mmaped to user space */
static int
overlay1fb_pure_set_var(struct fb_var_screeninfo *var, int con, struct fb_info *info)
{
	struct overlayfb_info *fbi=(struct overlayfb_info*)info;
	struct fb_var_screeninfo *dvar = &fbi->fb.var;
	int nbytes=0, err=0, pixels_per_line=0;

	/* validate parameters*/
	err = overlay1fb_validate_var(var, fbi);
	if (err) 
		return err;

	switch (var->bits_per_pixel) {
	case 16:
		if ( var->xres & 0x1 ) {
			printk("xres should be a multiple of 2 pixels!\n");
			return -EINVAL;
		}
		break;
	case 19:
		if ( var->xres & 0x7 ) {
			printk("xres should be a multiple of 8 pixels!\n");
			return -EINVAL;
		}
		break;
	default:
		break;
	}

	if (fbi->state == C_DISABLE) 
		goto out1;
	if ( (var->xres == dvar->xres)  && 
		(var->yres == dvar->yres) && 
		(var->bits_per_pixel == dvar->bits_per_pixel) ) 
		goto out2;

out1:
	/* update var_screeninfo fields*/
	*dvar = *var;

	switch(var->bits_per_pixel) {
	case 16:
		/* 2 pixels per line */ 
		pixels_per_line = (fbi->fb.var.xres + 0x1) & (~0x1);
		nbytes = 2;

		dvar->red    = def_rgbt_16.red;
		dvar->green  = def_rgbt_16.green;
		dvar->blue   = def_rgbt_16.blue;
		dvar->transp = def_rgbt_16.transp;
		if (PXAFB_MAIN_VALID_FB == PXAFB_MAIN_OVL1)
		{
			fbi->fb.disp->visual = FB_VISUAL_TRUECOLOR;
			fbi->fb.disp->line_length = var->xres * 2;
			fbi->fb.disp->dispsw		= &fbcon_cfb16;
			fbi->fb.disp->dispsw_data	= fbi->fb.pseudo_palette;
		}
		break;
	case 18:
		pixels_per_line = (fbi->fb.var.xres + 0x3 ) & (~0x3);
		nbytes = 3;
 
		dvar->red    = def_rgbt_18.red;
		dvar->green  = def_rgbt_18.green;
		dvar->blue   = def_rgbt_18.blue;
		dvar->transp = def_rgbt_18.transp;

		if (PXAFB_MAIN_VALID_FB == PXAFB_MAIN_OVL1)
		{
			fbi->fb.disp->visual = FB_VISUAL_TRUECOLOR;
			fbi->fb.disp->line_length = var->xres * 3;
			fbi->fb.disp->dispsw = &fbcon_dummy;
			fbi->fb.disp->dispsw_data = fbi->fb.pseudo_palette;
		}
		break;
	case 19:
		pixels_per_line = (fbi->fb.var.xres + 0x3 ) & (~0x3);
		nbytes = 3;

		dvar->red    = def_rgbt_19.red;
		dvar->green  = def_rgbt_19.green;
		dvar->blue   = def_rgbt_19.blue;
		dvar->transp = def_rgbt_19.transp;
		if (PXAFB_MAIN_VALID_FB == PXAFB_MAIN_OVL1)
		{
			fbi->fb.disp->visual		= FB_VISUAL_TRUECOLOR;
			fbi->fb.disp->line_length	= var->xres * 3;
			fbi->fb.disp->dispsw = &fbcon_dummy;
			fbi->fb.disp->dispsw_data	= fbi->fb.pseudo_palette;
		}
		break;
	case 24:
		pixels_per_line = fbi->fb.var.xres;
		nbytes = 4;

		dvar->red    = def_rgbt_24.red;
		dvar->green  = def_rgbt_24.green;
		dvar->blue   = def_rgbt_24.blue;
		dvar->transp = def_rgbt_24.transp;

		if (PXAFB_MAIN_VALID_FB == PXAFB_MAIN_OVL1)
		{
			fbi->fb.disp->visual		= FB_VISUAL_TRUECOLOR;
			fbi->fb.disp->line_length	= var->xres * 4;
			fbi->fb.disp->dispsw = &fbcon_dummy;
			fbi->fb.disp->dispsw_data	= fbi->fb.pseudo_palette;
		}

		break;
	case 25:
		pixels_per_line = fbi->fb.var.xres;
		nbytes = 4;
		dvar->red    = def_rgbt_25.red;
		dvar->green  = def_rgbt_25.green;
		dvar->blue   = def_rgbt_25.blue;
		dvar->transp = def_rgbt_25.transp;

		if (PXAFB_MAIN_VALID_FB == PXAFB_MAIN_OVL1)
		{
			fbi->fb.disp->visual		= FB_VISUAL_TRUECOLOR;
			fbi->fb.disp->line_length	= var->xres * 4;
			fbi->fb.disp->dispsw = &fbcon_dummy;
			fbi->fb.disp->dispsw_data	= fbi->fb.pseudo_palette;
		}

		break;
	}
		
	if (PXAFB_MAIN_VALID_FB == PXAFB_MAIN_OVL1)
	{
		fbi->fb.disp->screen_base	= (char *)(fbi->screen_cpu);
		fbi->fb.disp->next_line	= fbi->fb.disp->line_length;
		fbi->fb.disp->type		= fbi->fb.fix.type;
		fbi->fb.disp->type_aux	= fbi->fb.fix.type_aux;
		fbi->fb.disp->ypanstep	= fbi->fb.fix.ypanstep;
		fbi->fb.disp->ywrapstep	= fbi->fb.fix.ywrapstep;
		fbi->fb.disp->can_soft_blank	= 1;
		fbi->fb.disp->inverse	= 0;
		/*
		 * Update the old var.  The fbcon drivers still use this.
		 * Once they are using fbi->fb.var, this can be dropped.
		 */
		fbi->fb.disp->var = *dvar;
	}

	/* update fix_screeninfo fields */
	fbi->fb.fix.line_length = nbytes * pixels_per_line;
	fbi->fb.fix.smem_len = fbi->fb.fix.line_length * fbi->fb.var.yres;
	
	DPRINTK("overlay1fb_pure_set_var:fbi->fb.fix.line_length(%d),fbi->fb.fix.smem_len(%d)\n",fbi->fb.fix.line_length,fbi->fb.fix.smem_len);
		
out2:
	fbi->xpos = var->nonstd & 0x3ff;
	fbi->ypos = (var->nonstd>>10) & 0x3ff;
	return 0;
}

static int
overlay1fb_set_var(struct fb_var_screeninfo *var, int con, struct fb_info *info)
{
	struct overlayfb_info *fbi=(struct overlayfb_info*)info;
	struct fb_var_screeninfo *dvar = &fbi->fb.var;
	int nbytes=0, err=0, pixels_per_line=0;

	/* validate parameters*/
	err = overlay1fb_validate_var(var, fbi);
	if (err) 
		return err;

	switch (var->bits_per_pixel) {
	case 16:
		if ( var->xres & 0x1 ) {
			printk("xres should be a multiple of 2 pixels!\n");
			return -EINVAL;
		}
		break;
	case 19:
		if ( var->xres & 0x7 ) {
			printk("xres should be a multiple of 8 pixels!\n");
			return -EINVAL;
		}
		break;
	default:
		break;
	}

	if (fbi->state == C_DISABLE) 
		goto out1;
	if ( (var->xres == dvar->xres)  && 
		(var->yres == dvar->yres) && 
		(var->bits_per_pixel == dvar->bits_per_pixel) ) 
		goto out2;

out1:
	/* update var_screeninfo fields*/
	*dvar = *var;

	switch(var->bits_per_pixel) {
	case 16:
		/* 2 pixels per line */ 
		pixels_per_line = (fbi->fb.var.xres + 0x1) & (~0x1);
		nbytes = 2;

		dvar->red    = def_rgbt_16.red;
		dvar->green  = def_rgbt_16.green;
		dvar->blue   = def_rgbt_16.blue;
		dvar->transp = def_rgbt_16.transp;
		break;
	case 18:
		pixels_per_line = (fbi->fb.var.xres + 0x3 ) & (~0x3);
		nbytes = 3;
 
		dvar->red    = def_rgbt_18.red;
		dvar->green  = def_rgbt_18.green;
		dvar->blue   = def_rgbt_18.blue;
		dvar->transp = def_rgbt_18.transp;

		break;
	case 19:
		pixels_per_line = (fbi->fb.var.xres + 0x3 ) & (~0x3);
		nbytes = 3;

		dvar->red    = def_rgbt_19.red;
		dvar->green  = def_rgbt_19.green;
		dvar->blue   = def_rgbt_19.blue;
		dvar->transp = def_rgbt_19.transp;
		break;
	case 24:
		pixels_per_line = fbi->fb.var.xres;
		nbytes = 4;

		dvar->red    = def_rgbt_24.red;
		dvar->green  = def_rgbt_24.green;
		dvar->blue   = def_rgbt_24.blue;
		dvar->transp = def_rgbt_24.transp;

		break;
	case 25:
		pixels_per_line = fbi->fb.var.xres;
		nbytes = 4;
		dvar->red    = def_rgbt_25.red;
		dvar->green  = def_rgbt_25.green;
		dvar->blue   = def_rgbt_25.blue;
		dvar->transp = def_rgbt_25.transp;

		break;
	}
		
	/* update fix_screeninfo fields */
	fbi->fb.fix.line_length = nbytes * pixels_per_line;
	fbi->fb.fix.smem_len = fbi->fb.fix.line_length * fbi->fb.var.yres;
	
	DPRINTK("overlay1fb_pure_set_var:fbi->fb.fix.line_length(%d),fbi->fb.fix.smem_len(%d)\n",fbi->fb.fix.line_length,fbi->fb.fix.smem_len);
		
out2:
	fbi->xpos = var->nonstd & 0x3ff;
	fbi->ypos = (var->nonstd>>10) & 0x3ff;

	overlay1fb_enable((struct fb_info*)fbi);

	return 0;
}

/* we don't have cmap */
static int
overlay1fb_get_cmap(struct fb_cmap *cmap, int kspc, int con, struct fb_info *info)
{
	return 0;
}

static int
overlay1fb_set_cmap(struct fb_cmap *cmap, int kspc, int con,
		  struct fb_info *info)
{
	return -EINVAL;
}

static int overlay1fb_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg, int con, struct fb_info *info)
{
	switch (cmd) 
	{
	case FBIOCKMAINVALIDFB:
		{
			if ( (info->modename[0] < PXAFB_CLI_BASE) && (info->modename[0] == PXAFB_MAIN_VALID_FB) )
				return 1;
			else
				return 0;
		}
	}

	return -EINVAL;
}

static struct fb_ops overlay1fb_ops = {
	owner:		THIS_MODULE,
	fb_open:	overlay1fb_open,
	fb_release:	overlay1fb_release,
	fb_get_fix:	overlay1fb_get_fix,
	fb_get_var:	overlay1fb_get_var,
	fb_set_var:	overlay1fb_set_var,
	fb_get_cmap:	overlay1fb_get_cmap,
	fb_set_cmap:	overlay1fb_set_cmap,
	fb_ioctl:	overlay1fb_ioctl,
};

/* 
 * LCD enhancement : Overlay 2 
 *
 * Features:
 * - support planar YCbCr420/YCbCr422/YCbCr444;
 */ 
static int overlay2fb_open(struct fb_info *info, int user)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	int ret = 0;

	down(&fbi->mutex);

	if (fbi->refcount) 
		ret = -EACCES;
	else
		fbi->refcount ++;

	up(&fbi->mutex);
	fbi->fb.var.xres = fbi->fb.var.yres = 0;

	return ret;
}

static int overlay2fb_release(struct fb_info *info, int user)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;

	down(&fbi->mutex);

	if (fbi->refcount)
		fbi->refcount --;

	up(&fbi->mutex);

	/* disable overlay when released */
	overlay2fb_blank(1, info);
		
	consistent_free((void *)fbi->map_cpu, fbi->map_size, fbi->map_dma);
	fbi->map_cpu = 0;
	return 0;
}

/*
 * Overlay 2 - support planar  YCbCr420/YCbCr422/YCbCr444, RGB mode
 */
static int 
overlay2fb_map_YUV_memory( struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	unsigned int ylen, cblen, crlen, aylen, acblen, acrlen;
	unsigned int yoff, cboff, croff;
	unsigned int xres,yres; 
	unsigned int nbytes;
	
	unsigned long framelen;


	ylen = cblen = crlen = aylen = acblen = acrlen = 0;
	yoff = cboff = croff = 0;


	if (fbi->map_cpu) 
		consistent_free((void*)fbi->map_cpu, fbi->map_size, fbi->map_dma);

	yres = fbi->fb.var.yres;

	switch(fbi->format) {
	case 0x4: /* YCbCr 4:2:0 planar */
		/* 16 pixels per line */
		xres = (fbi->fb.var.xres + 0xf) & (~0xf);
		fbi->fb.fix.line_length = xres;

		nbytes = xres * yres;
		ylen = nbytes;
		cblen = crlen = (nbytes/4);

		break;
	case 0x3: /* YCbCr 4:2:2 planar */
		/* 8 pixles per line */
		xres = (fbi->fb.var.xres + 0x7) & (~0x7);
		fbi->fb.fix.line_length = xres;

		nbytes = xres * yres;
		ylen  = nbytes;
		cblen = crlen = (nbytes/2);

		break;
	case 0x2: /* YCbCr 4:4:4 planar */
		/* 4 pixels per line */
		xres = (fbi->fb.var.xres + 0x3) & (~0x3);
		fbi->fb.fix.line_length = xres;

		nbytes = xres * yres;
		ylen  = cblen = crlen = nbytes;
		break;
	}

	/* 16-bytes alignment for DMA */
	aylen  = (ylen + 0xf) & (~0xf);
	acblen = (cblen + 0xf) & (~0xf);
	acrlen = (crlen + 0xf) & (~0xf);

	//fbi->fb.fix.smem_len = aylen + acblen + acrlen;
	framelen = aylen + acblen + acrlen;
	fbi->fb.fix.smem_len = framelen * 2;

	/* alloc memory */

	fbi->map_size = PAGE_ALIGN(fbi->fb.fix.smem_len + PAGE_SIZE);
	fbi->map_cpu = (unsigned long)__consistent_alloc(GFP_KERNEL, fbi->map_size,
							 &fbi->map_dma, PTE_BUFFERABLE);
	
	DPRINTK("overlay2fb_map_video_memory: NEW Buffer: map_cpu=0x%x, map_size=0x%x\n", fbi->map_cpu, fbi->map_size);
	
	if (!fbi->map_cpu) return -ENOMEM;

	memset((void *)fbi->map_cpu, 0, fbi->map_size);
	fbi->screen_cpu = fbi->map_cpu + PAGE_SIZE;
	fbi->screen_dma = fbi->map_dma + PAGE_SIZE;
	
	fbi->fb.fix.smem_start = fbi->screen_dma;

	/* setup dma for Planar format */
	fbi->dma2 = (struct pxafb_dma_descriptor*)(fbi->screen_cpu - sizeof(struct pxafb_dma_descriptor));
	fbi->dma3 = fbi->dma2 - 1;
	fbi->dma4 = fbi->dma3 - 1;
	fbi->dma2_1 = fbi->dma4 - 1;
	fbi->dma3_1 = fbi->dma2_1 - 1;
	fbi->dma4_1 = fbi->dma3_1 - 1;

	/* offset */
	yoff = 0;
	cboff = yoff + aylen;
	croff = cboff + acblen;

	/* Y vector */
	fbi->dma2->fdadr = (fbi->screen_dma - sizeof(struct pxafb_dma_descriptor));
	fbi->dma2->fsadr = fbi->screen_dma + yoff;
	fbi->dma2->fidr  = 0;
	fbi->dma2->ldcmd = ylen;	
	fbi->dma2_1->fdadr = (fbi->screen_dma - 4*sizeof(struct pxafb_dma_descriptor));
        fbi->dma2_1->fsadr = fbi->screen_dma + framelen + yoff;
        fbi->dma2_1->fidr  = 0;
        fbi->dma2_1->ldcmd = ylen;


	/* Cb vector */
	fbi->dma3->fdadr = (fbi->dma2->fdadr - sizeof(struct pxafb_dma_descriptor));
	fbi->dma3->fsadr = (fbi->screen_dma + cboff);
	fbi->dma3->fidr  = 0;
	fbi->dma3->ldcmd = cblen;
	fbi->dma3_1->fdadr = (fbi->dma2->fdadr - 4*sizeof(struct pxafb_dma_descriptor));
        fbi->dma3_1->fsadr = fbi->screen_dma + framelen + cboff;
        fbi->dma3_1->fidr  = 0;
        fbi->dma3_1->ldcmd = cblen;

	/* Cr vector */
	fbi->dma4->fdadr = (fbi->dma3->fdadr - sizeof(struct pxafb_dma_descriptor));
	fbi->dma4->fsadr = (fbi->screen_dma + croff);
	fbi->dma4->fidr  = 0;
	fbi->dma4->ldcmd = crlen;
	fbi->dma4_1->fdadr = (fbi->dma3->fdadr - 4*sizeof(struct pxafb_dma_descriptor));
        fbi->dma4_1->fsadr = fbi->screen_dma + framelen + croff;
        fbi->dma4_1->fidr  = 0;
        fbi->dma4_1->ldcmd = crlen;

	DPRINTK("overlay2fb_map_video_memory results:");
        DPRINTK("fbi->dma2->fdadr = 0x%x", fbi->dma2->fdadr);
	DPRINTK("fbi->dma2->fsadr = 0x%x", fbi->dma2->fsadr);
        DPRINTK("fbi->dma2->fidr  = 0x%x", fbi->dma2->fidr);
        DPRINTK("fbi->dma2->ldcmd = 0x%x", fbi->dma2->ldcmd);
        DPRINTK("fbi->dma2_1->fdadr = 0x%x", fbi->dma2_1->fdadr);
        DPRINTK("fbi->dma2_1->fsadr = 0x%x", fbi->dma2_1->fsadr);
        DPRINTK("fbi->dma2_1->fidr  = 0x%x", fbi->dma2_1->fidr);
        DPRINTK("fbi->dma2_1->ldcmd = 0x%x", fbi->dma2_1->ldcmd);
        DPRINTK("fbi->dma3->fdadr = 0x%x", fbi->dma3->fdadr);
        DPRINTK("fbi->dma3->fsadr = 0x%x", fbi->dma3->fsadr);
        DPRINTK("fbi->dma3->fidr  = 0x%x", fbi->dma3->fidr);
        DPRINTK("fbi->dma3->ldcmd = 0x%x", fbi->dma3->ldcmd);
        DPRINTK("fbi->dma3_1->fdadr = 0x%x", fbi->dma3_1->fdadr);
        DPRINTK("fbi->dma3_1->fsadr = 0x%x", fbi->dma3_1->fsadr);
        DPRINTK("fbi->dma3_1->fidr  = 0x%x", fbi->dma3_1->fidr);
        DPRINTK("fbi->dma3_1->ldcmd = 0x%x", fbi->dma3_1->ldcmd);
        DPRINTK("fbi->dma4->fdadr = 0x%x", fbi->dma4->fdadr);
        DPRINTK("fbi->dma4->fsadr = 0x%x", fbi->dma4->fsadr);
        DPRINTK("fbi->dma4->fidr  = 0x%x", fbi->dma4->fidr);
        DPRINTK("fbi->dma4->ldcmd = 0x%x", fbi->dma4->ldcmd);
        DPRINTK("fbi->dma4_1->fdadr = 0x%x", fbi->dma4_1->fdadr);
        DPRINTK("fbi->dma4_1->fsadr = 0x%x", fbi->dma4_1->fsadr);
        DPRINTK("fbi->dma4_1->fidr  = 0x%x", fbi->dma4_1->fidr);
        DPRINTK("fbi->dma4_1->ldcmd = 0x%x", fbi->dma4_1->ldcmd);

	
	/* adjust for user */
	fbi->fb.var.red.length   = ylen;
	fbi->fb.var.red.offset   = yoff;
	fbi->fb.var.green.length = cblen;
	fbi->fb.var.green.offset = cboff;
	fbi->fb.var.blue.length  = crlen;
	fbi->fb.var.blue.offset  = croff;

	return 0;
};

static int 
overlay2fb_map_RGB_memory( struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	struct fb_var_screeninfo *var = &fbi->fb.var;
	int pixels_per_line=0 , nbytes=0;

	if (fbi->map_cpu) 
		consistent_free((void*)fbi->map_cpu, fbi->map_size, fbi->map_dma);

	switch(var->bits_per_pixel) {
	case 16:
		/* 2 pixels per line */ 
		pixels_per_line = (fbi->fb.var.xres + 0x1) & (~0x1);
		nbytes = 2;

		var->red    = def_rgbt_16.red;
		var->green  = def_rgbt_16.green;
		var->blue   = def_rgbt_16.blue;
		var->transp = def_rgbt_16.transp;
		break;

	case 18:
		/* 8 pixels per line */
		pixels_per_line = (fbi->fb.var.xres + 0x7 ) & (~0x7);
		nbytes = 3;

		var->red    = def_rgbt_18.red;
		var->green  = def_rgbt_18.green;
		var->blue   = def_rgbt_18.blue;
		var->transp = def_rgbt_18.transp;

		break;
	case 19:
		/* 8 pixels per line */
		pixels_per_line = (fbi->fb.var.xres + 0x7 ) & (~0x7);
		nbytes = 3;

		var->red    = def_rgbt_19.red;
		var->green  = def_rgbt_19.green;
		var->blue   = def_rgbt_19.blue;
		var->transp = def_rgbt_19.transp;

		break;
	case 24:
		pixels_per_line = fbi->fb.var.xres;
		nbytes = 4;

		var->red    = def_rgbt_24.red;
		var->green  = def_rgbt_24.green;
		var->blue   = def_rgbt_24.blue;
		var->transp = def_rgbt_24.transp;
		break;
	case 25:
		pixels_per_line = fbi->fb.var.xres;
		nbytes = 4;

		var->red    = def_rgbt_25.red;
		var->green  = def_rgbt_25.green;
		var->blue   = def_rgbt_25.blue;
		var->transp = def_rgbt_25.transp;

		break;
	}

	fbi->fb.fix.line_length = nbytes * pixels_per_line ;
	fbi->fb.fix.smem_len = fbi->fb.fix.line_length * fbi->fb.var.yres ;

	fbi->map_size = PAGE_ALIGN(fbi->fb.fix.smem_len + PAGE_SIZE);
	fbi->map_cpu = (unsigned long)__consistent_alloc(GFP_KERNEL, fbi->map_size,
					       &fbi->map_dma, PTE_BUFFERABLE );
	
	if (!fbi->map_cpu) return -ENOMEM;

	fbi->screen_cpu = fbi->map_cpu + PAGE_SIZE;
	fbi->screen_dma = fbi->map_dma + PAGE_SIZE;

	fbi->fb.fix.smem_start = fbi->screen_dma;

	/* setup dma descriptor */
	fbi->dma2 = (struct pxafb_dma_descriptor*)
		(fbi->screen_cpu - sizeof(struct pxafb_dma_descriptor));

	fbi->dma2->fdadr = (fbi->screen_dma - sizeof(struct pxafb_dma_descriptor));
	fbi->dma2->fsadr = fbi->screen_dma;
	fbi->dma2->fidr  = 0;
	fbi->dma2->ldcmd = fbi->fb.fix.smem_len;

	return 0;
}

/* FIXME: workaround for sighting #49219, #56573 */
static int 
overlay2fb_YUV420_fixup(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*)info;
	struct pxafb_dma_descriptor *dma; 
	u32 map_dma, map_cpu;
	int done, ret=0;

	map_cpu = (u32)__consistent_alloc(GFP_KERNEL, PAGE_SIZE, &map_dma, PTE_BUFFERABLE);
	if (!map_cpu) return -1;

	dma = (struct pxafb_dma_descriptor*)((map_cpu + PAGE_SIZE) - sizeof(struct pxafb_dma_descriptor));
	dma->fdadr = map_dma + PAGE_SIZE - sizeof(struct pxafb_dma_descriptor);
	dma->fsadr = map_dma;
	dma->fidr  = 0;
	dma->ldcmd = LDCMD_EOFINT | 128;

	/* step 2.a - enable overlay 2 with RGB mode
         *
         * - (xpos,ypos) = (0,0); 
	 * - 64 pixels, 16bpp 
	 */
	LCCR5 |= (LCCR5_IUM4 | LCCR5_IUM3 | LCCR5_IUM2 | 
		  LCCR5_BSM4 | LCCR5_BSM3 | LCCR5_BSM2 |
		  LCCR5_EOFM4 | LCCR5_EOFM3 | LCCR5_EOFM2 |
		  LCCR5_SOFM4 | LCCR5_SOFM3 | LCCR5_SOFM2);
	OVL2C2 = 0;
	OVL2C1 = OVL2C1_O2EN | (63);

	CLEAR_LCD_INTR(LCSR1, LCSR1_EOF2);
	if (fbi->state == C_DISABLE) 
		FDADR2 = dma->fdadr;
	else
		FBR2 = dma->fdadr | 0x1;

	/* step 2.b -  run at least 1 frame with Overlay 2 */
	done = WAIT_FOR_LCD_INTR(LCSR1, LCSR1_EOF2, 100);
	if (!done) goto err;

	/* step 2.c - disable overlay 2 */
	OVL2C1 &= ~OVL2C1_O2EN;

	CLEAR_LCD_INTR(LCSR1, LCSR1_BS2);
	FBR2 = 0x3;

	done = WAIT_FOR_LCD_INTR(LCSR1, LCSR1_BS2, 100);
	if (!done) goto err;

	/* step 2.d - Wait for base EOF interrupts */
	CLEAR_LCD_INTR(LCSR0, LCSR0_EOF);
	done = WAIT_FOR_LCD_INTR(LCSR0, LCSR0_EOF, 100);

	goto out;
err:
	ret = -1;
out:
	/* free buffer allocated */
	consistent_free((void*)map_cpu, PAGE_SIZE, map_dma);

	return ret;
}

static int
overlay2fb_enable(struct fb_info *info) 
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	unsigned long bpp2 = 0x4;
	unsigned int xres, yres;

	if (fbi->basefb->state != C_ENABLE) return 0;
	if (!fbi->map_cpu) return -EINVAL;

	DPRINTK("overlay2fb_enable called: %d", fbi->state);
	
	switch(fbi->fb.var.bits_per_pixel) {
	case 16:
		bpp2 = 0x4;
		break;
	case 18:
		bpp2 = 0x6;
		break;
	case 19:
		bpp2 = 0x8;
		break;
	case 24:
		bpp2 = 0x9;
		break;
	case 25:
		bpp2 = 0xa;
		break;
	default:
		break;
	}

	/* disable branch/start/end of frame interrupt */
	LCCR5 = 0;
	LCCR5 |= (LCCR5_IUM4 | LCCR5_IUM3 | LCCR5_IUM2 | 
		  LCCR5_BSM4 | LCCR5_BSM3 | LCCR5_EOFM4 | LCCR5_EOFM3 | LCCR5_EOFM2 |
		  LCCR5_SOFM4 | LCCR5_SOFM3 | LCCR5_SOFM2);

	LCCR4 = (LCCR4 & (~(0x01FF))) | pxafb_global_state.trans_value ;  

	printk(KERN_NOTICE "OVERLAY2FB_ENABLE: LCCR5 settings\n");
	printk(KERN_NOTICE "overlay2fb_enable:1 -- LCSR0(0x%x) LCSR1(0x%x)\n", LCSR0,LCSR1);

	xres = fbi->fb.var.xres;
	yres = fbi->fb.var.yres;

	OVL2C2 = (fbi->format << 20) | (fbi->ypos << 10) | fbi->xpos;
	OVL2C1 = OVL2C1_O2EN | (bpp2 << 20) | ((yres-1)<<10) | (xres-1);
	printk(KERN_NOTICE "OVERLAY2FB_ENABLE: OVL2C1,OVL2C2 settings\n");
	printk(KERN_NOTICE "overlay2fb_enable:2 -- LCSR0(0x%x) LCSR1(0x%x)\n", LCSR0,LCSR1);

	if (fbi->format == 0) {
		/* setup RGB DMA */
                if (fbi->state == C_DISABLE)
                        FDADR2 = fbi->dma2->fdadr;
                else
                        FBR2 = fbi->dma2->fdadr | 0x1;
	} else {
		/* FIXME */
		if (fbi->format == 4) {
			/* Wait util fifo emtpy */
			CLEAR_LCD_INTR(LCSR1, LCSR1_IU2);
			WAIT_FOR_LCD_INTR(LCSR1, LCSR1_IU2, 100);
		}

		/* setup YUV DMA */
                if (fbi->state == C_DISABLE) {
                        FDADR2 = fbi->dma2->fdadr;
                        FDADR3 = fbi->dma3->fdadr;
                        FDADR4 = fbi->dma4->fdadr;
			printk(KERN_NOTICE "OVERLAY2FB_ENABLE: branch C_DISABLE-- FDADR2 FDADR3 FDADR4 settings\n");

                } else {
                        FBR2 = fbi->dma2->fdadr | 0x1;
                        FBR3 = fbi->dma3->fdadr | 0x1;
                        FBR4 = fbi->dma4->fdadr | 0x1;
			printk(KERN_NOTICE "OVERLAY2FB_ENABLE: branch C_ENABLE-- FBR2 FBR3 FBR4 settings\n");
              }
	}
	printk(KERN_NOTICE "overlay2fb_enable:3 -- LCSR0(0x%x) LCSR1(0x%x)\n", LCSR0,LCSR1);

	fbi->state = C_ENABLE;
	
	return 0;
}

static int
overlay2fb_disable(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*)info;
	int done;

	if (fbi->basefb->state != C_ENABLE) return 0;
	if (fbi->state == C_DISABLE) return 0;
	
	DPRINTK("overlay2fb_disable called: %d", fbi->state);

	fbi->state = C_DISABLE;
		
	OVL2C1 &= ~OVL2C1_O2EN;
	
	CLEAR_LCD_INTR(LCSR1, LCSR1_BS2);

	if (fbi->format == 0) 
		FBR2 = 0x3;
	else {
		FBR2 = 0x3;
		FBR3 = 0x3;
		FBR4 = 0x3;
	}

	done = WAIT_FOR_LCD_INTR(LCSR1, LCSR1_BS2, 100);

	OVL2C1 = 0;
	OVL2C2 = 0;

	LCCR4 |= 0x01FF;  //K3=1 K2=1 K1=1 //
	LCCR5 |= LCCR5_BSM2;
	
	if (!done) {
		DPRINTK(KERN_INFO "%s: timeout\n", __FUNCTION__);
		return -1;
	}
	return 0;
}

static void overlay2fb_blank(int blank, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;

	down(&fbi->basefb->ctrlr_sem);
	switch(blank) 
	{
	case 0:
		if (fbi->state == C_DISABLE)
			overlay2fb_enable(info);
		break;
	case 1:
		if (fbi->state == C_ENABLE)
			overlay2fb_disable(info);
		break;
	default:
		/* reserved */
		break;
	}
	up(&fbi->basefb->ctrlr_sem);
}

static int
overlay2fb_get_fix(struct fb_fix_screeninfo *fix, int con, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;

	*fix = fbi->fb.fix;

	return 0;
}

static int
overlay2fb_get_var(struct fb_var_screeninfo *var, int con, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;

	*var = fbi->fb.var;
	var->nonstd = (fbi->format<<20) | (fbi->ypos<<10) | fbi->xpos;

	return 0;
}

static int 
overlay2fb_validate_var( struct fb_var_screeninfo *var, struct overlayfb_info *fbi)
{
	int xpos, ypos, xres, yres;
	int format;

	xres=yres=0;

	xpos = (var->nonstd & 0x3ff);
	ypos = (var->nonstd >> 10) & 0x3ff;
	format = (var->nonstd >>20) & 0x7;

	/* Palnar YCbCr444, YCbCr422, YCbCr420 */
	if ( (format != 0x4) && (format != 0x3) && (format != 0x2) && (format != 0x0)) 
		return -EINVAL;

	/* dummy pixels */
	switch(format) {
	case 0x0: /* RGB */
		xres = var->xres;
		break;
	case 0x2: /* 444 */
		xres = (var->xres + 0x3) & ~(0x3);
		break;
	case 0x3: /* 422 */
		xres = (var->xres + 0x7) & ~(0x7);
		break;
	case 0x4: /* 420 */
		xres = (var->xres + 0xf) & ~(0xf);
		break;
	}
	yres = var->yres;

	if ( (xpos + xres) > fbi->basefb->max_xres ) 
		return -EINVAL;

	if ( (ypos + yres) > fbi->basefb->max_yres ) 
		return -EINVAL;

	return 0;

}

/*
 * overlay2fb_pure_set_var()
 *
 * var.nonstd is used as YCbCr format.
 * var.red/green/blue is used as (Y/Cb/Cr) vector 
 */

static int
overlay2fb_pure_set_var(struct fb_var_screeninfo *var, int con, struct fb_info *info)
{
	struct overlayfb_info *fbi=(struct overlayfb_info*)info;
	struct fb_var_screeninfo *dvar = &fbi->fb.var;
	int format, err;

	/* validate parameters*/
	err = overlay2fb_validate_var(var, fbi);
	if (err) return err;

	//format = (var->nonstd>>20) & 0x7;
	format = fbi->format;  //we have configured it in overlay2fb_init_fbinfo //

	if (!fbi->map_cpu)
		goto out1;

	/* resolution */
	if ( (var->xres == dvar->xres) && (var->yres == dvar->yres) && 
		(format == fbi->format) &&
		((fbi->format != 0) || 
		 ((fbi->format == 0) && (var->bits_per_pixel == dvar->bits_per_pixel)))
	   )
		goto out2;

out1:
	DPRINTK("overlay2fb_set_var: Remap memory");
	/* FIXME */
	
	/* update var_screeninfo fields*/
	*dvar = *var;

	//fbi->format = format; -- susan

out2:
	fbi->xpos = var->nonstd & 0x3ff;
	fbi->ypos = (var->nonstd>>10) & 0x3ff;
	
	return 0;
}


/*
 * overlay2fb_set_var()
 *
 * var.nonstd is used as YCbCr format.
 * var.red/green/blue is used as (Y/Cb/Cr) vector 
 */

static int
overlay2fb_set_var(struct fb_var_screeninfo *var, int con, struct fb_info *info)
{
	struct overlayfb_info *fbi=(struct overlayfb_info*)info;
	struct fb_var_screeninfo *dvar = &fbi->fb.var;
	int format, err;

	/* validate parameters*/
	err = overlay2fb_validate_var(var, fbi);
	if (err) return err;

	format = (var->nonstd>>20) & 0x7;

	if (!fbi->map_cpu)
		goto out1;

	/* resolution */
	if ( (var->xres == dvar->xres) && (var->yres == dvar->yres) && 
		(format == fbi->format) &&
		((fbi->format != 0) || 
		 ((fbi->format == 0) && (var->bits_per_pixel == dvar->bits_per_pixel)))
	   )
		goto out2;

out1:
	DPRINTK("overlay2fb_set_var: Remap memory");
	/* FIXME */
	if ( (format == 0x4) &&
	     ((fbi->state == C_DISABLE) || (fbi->format != format)) )
		overlay2fb_YUV420_fixup(info);

	/* update var_screeninfo fields*/
	*dvar = *var;

	fbi->format = format;

	if (fbi->format == 0) 
		err= overlay2fb_map_RGB_memory(info);
	else 
		err= overlay2fb_map_YUV_memory(info);
	if (err) return err;
out2:
	fbi->xpos = var->nonstd & 0x3ff;
	fbi->ypos = (var->nonstd>>10) & 0x3ff;

	down(&fbi->basefb->ctrlr_sem);
	overlay2fb_enable(info);
	up(&fbi->basefb->ctrlr_sem);
	return 0;
}

/* we don't have cmap */
static int
overlay2fb_get_cmap(struct fb_cmap *cmap, int kspc, int con, struct fb_info *info)
{
	return 0;
}

static int
overlay2fb_set_cmap(struct fb_cmap *cmap, int kspc, int con,
		  struct fb_info *info)
{
	return -EINVAL;
}
static int overlay2fb_pan_display(struct fb_var_screeninfo *var, int con, struct fb_info *info)
{
	DECLARE_WAITQUEUE(wait, current);
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
        unsigned long fbr2, fbr3, fbr4;
	int flags;

	DPRINTK("overlay2fb_pan_display: enter");

	if ( (fbi->basefb->state != C_ENABLE) || (fbi->state != C_ENABLE) || (pxafb_global_state.main_ovl2_status != PXAFB_OVL2_ON) )
	{
		return -EINVAL;
	}
	/* for now only allow double buffering case */
	/* Firstly remove LCCR5_BSM2 (LCCR5_BSM3, LCCR5_BSM4) */

	if (var->yoffset==0) {
		DPRINTK("Setting Overlay2 to buffer 0");
	        fbr2 = fbi->dma2->fdadr | 0x3;
        	fbr3 = fbi->dma3->fdadr | 0x1;
	        fbr4 = fbi->dma4->fdadr | 0x1;
	} else if (var->yoffset==fbi->fb.var.yres ) {
		DPRINTK("Setting Overlay2 to buffer 1,fbi->dma2_1(0x%x) fbi->dma3_1(0x%x) fbi->dma4_1(0x%x)\n", fbi->dma2_1,fbi->dma3_1,fbi->dma4_1);
                fbr2 = fbi->dma2_1->fdadr | 0x3;
                fbr3 = fbi->dma3_1->fdadr | 0x1;
                fbr4 = fbi->dma4_1->fdadr | 0x1;
        } else {
                DPRINTK("overlay2fb_pan_display error: Can only pan to 0 or yres (if double buffering is enabled)");
                return -EINVAL;
        }
	
	DPRINTK("overlay2fb_pan_display: before add_wait_queue,LCCR0(0x%x) FBR2(0x%x),FBR3(0x%x),FBR4(0x%x)\n",LCCR0,FBR2,FBR3,FBR4);

	/* set up for sleeping */
        add_wait_queue(&(fbi->bs_wait), &wait);
	
	local_irq_save(flags);
	FBR2 = fbr2;
	FBR3 = fbr3;
	FBR4 = fbr4;
        set_current_state(TASK_INTERRUPTIBLE);
	local_irq_restore(flags);
        
	/* go to sleep until current frame is done */
        schedule();
        set_current_state(TASK_RUNNING);

        remove_wait_queue(&fbi->bs_wait, &wait);

	DPRINTK("overlay2fb_pan_display: After remove_wait_queue\n");
	DPRINTK("fbi->dma2->fdadr =   0x%x", fbi->dma2->fdadr);
        DPRINTK("fbi->dma2_1->fdadr = 0x%x", fbi->dma2_1->fdadr);
	DPRINTK("FBR2 = 0x%x", FBR2);

	DPRINTK("fbi->dma3->fdadr =   0x%x", fbi->dma3->fdadr);
        DPRINTK("fbi->dma3_1->fdadr = 0x%x", fbi->dma3_1->fdadr);
	DPRINTK("FBR3 = 0x%x", FBR3);

	DPRINTK("fbi->dma4->fdadr =   0x%x", fbi->dma4->fdadr);
        DPRINTK("fbi->dma4_1->fdadr = 0x%x", fbi->dma4_1->fdadr);
	DPRINTK("FBR4 = 0x%x", FBR4);

	return 0;
}


static struct fb_ops overlay2fb_ops = {
	owner:		THIS_MODULE,
	fb_open:	overlay2fb_open,
	fb_release:	overlay2fb_release,
	fb_get_fix:	overlay2fb_get_fix,
	fb_get_var:	overlay2fb_get_var,
	fb_set_var:	overlay2fb_set_var,
	fb_get_cmap:	overlay2fb_get_cmap,
	fb_set_cmap:	overlay2fb_set_cmap,
	fb_pan_display: overlay2fb_pan_display,
};

/*
 * Hardware cursor support
 */
/* Bulverde Cursor Modes */
#if CONFIG_CURSOR_CHANNEL 
struct cursor_mode{
	int xres;
	int yres;
	int bpp;
};

static struct cursor_mode cursor_modes[]={
	{ 32,  32, 2},
	{ 32,  32, 2},
	{ 32,  32, 2},
	{ 64,  64, 2},
	{ 64,  64, 2},
	{ 64,  64, 2},
	{128, 128, 1},
	{128, 128, 1}
};

static int cursorfb_enable(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;

	if (fbi->state == C_ENABLE) return 0;
	if (!fbi->map_cpu) return -EINVAL;

	CCR &= ~(1<<31);

	/* set palette format 
	 *
	 * FIXME: if only cursor uses palette
	 */
	LCCR4 = (LCCR4 & (~(0x3<<15))) | (0x1<<15);

	/* disable branch/start/end of frame interrupt */
	LCCR5 |= (LCCR5_IUM5 | LCCR5_BSM5 | LCCR5_EOFM5 | LCCR5_SOFM5);

	/* load palette and frame data */
	if(fbi->state == C_DISABLE) {
		FDADR5 = fbi->dma5_pal->fdadr;
		udelay(1);
		FDADR5 = fbi->dma5_frame->fdadr;
		udelay(1);
	}
	else {
		FBR5 = fbi->dma5_pal->fdadr | 0x1;
		udelay(1);
		FBR5 = fbi->dma5_frame->fdadr | 0x1;
		udelay(1);
	}

	CCR = (1<<31) | (fbi->ypos << 15) | (fbi->xpos << 5) | (fbi->format);

	fbi->state = C_ENABLE;

	return 0;
}

static int cursorfb_disable(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*)info;

	fbi->state = C_DISABLE;

	CCR &= ~(1<<31);
	
	return 0;

}
static int
cursorfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
		       u_int trans, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info *)info;
	u_int val, ret = 1;
	u_int *pal=(u_int*) fbi->palette_cpu;

	/* 25bit with Transparcy for 16bpp format */
	if (regno < fbi->palette_size) {
		val = ((trans << 24)  & 0x1000000);
		val |= ((red << 16)  & 0x0ff0000);
		val |= ((green << 8 ) & 0x000ff00);
		val |= ((blue << 0) & 0x00000ff);

		pal[regno] = val;
		ret = 0;
	}
	return ret;
}

static void cursorfb_blank(int blank, struct fb_info *info)
{
	switch(blank) 
	{
	case 0:
		cursorfb_enable(info);
		break;
	case 1:
		cursorfb_disable(info);
		break;
	default:
		/* reserved */
		break;
	}
	return;
}

static int
cursorfb_get_fix(struct fb_fix_screeninfo *fix, int con, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;

	*fix = fbi->fb.fix;

	return 0;
}

static int
cursorfb_get_var(struct fb_var_screeninfo *var, int con, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;

	*var = fbi->fb.var;
	var->nonstd = (fbi->ypos<<15) | (fbi->xpos<<5) | fbi->format;

	return 0;
}

static int
cursorfb_set_var(struct fb_var_screeninfo *var, int con, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	struct cursor_mode *cursor;
	int mode, xpos, ypos;
	int changed,err;

	mode = var->nonstd & 0x7;
	xpos = (var->nonstd>>5) & 0x3ff;
	ypos = (var->nonstd>>15) & 0x3ff;

	changed = 0;
	if (mode != fbi->format) {
		cursor = cursor_modes + mode;

		/* update "var" info */
		fbi->fb.var.xres = cursor->xres;
		fbi->fb.var.yres = cursor->yres;
		fbi->fb.var.bits_per_pixel = cursor->bpp;

		/* alloc video memory 
		 *
		 * 4k is engouh for 128x128x1 cursor, 
		 * - 2k for cursor pixels, 
		 * - 2k for palette data, plus 2 dma descriptor
		 */
		if (!fbi->map_cpu) {
			fbi->map_size = PAGE_SIZE;
			fbi->map_cpu = (unsigned long)__consistent_alloc(GFP_KERNEL, fbi->map_size, 
									 &fbi->map_dma, PTE_BUFFERABLE);
			if (!fbi->map_cpu) return -ENOMEM;
			memset((void *)fbi->map_cpu, 0, fbi->map_size);
		}

		cursor = cursor_modes + mode;

		/* update overlay & fix "info" */
		fbi->screen_cpu  = fbi->map_cpu;
		fbi->palette_cpu = fbi->map_cpu + (PAGE_SIZE/2);
		fbi->screen_dma  = fbi->map_dma;
		fbi->palette_dma = fbi->map_dma + (PAGE_SIZE/2);

		fbi->format = mode; 
		fbi->palette_size = (1<<cursor->bpp) ;
		fbi->fb.fix.smem_start = fbi->screen_dma;
		fbi->fb.fix.smem_len = cursor->xres * cursor->yres * cursor->bpp / 8;
		fbi->fb.fix.line_length = cursor->xres * cursor->bpp / 8 ;

		fbi->dma5_pal     = (struct pxafb_dma_descriptor*)(fbi->map_cpu + PAGE_SIZE - 16 );
		fbi->dma5_pal->fdadr = (fbi->map_dma + PAGE_SIZE - 16);
		fbi->dma5_pal->fsadr = fbi->palette_dma;
		fbi->dma5_pal->fidr  = 0;
		fbi->dma5_pal->ldcmd = (fbi->palette_size<<2) | LDCMD_PAL;

		fbi->dma5_frame   = (struct pxafb_dma_descriptor*)(fbi->map_cpu + PAGE_SIZE - 32 );
		fbi->dma5_frame->fdadr = (fbi->map_dma + PAGE_SIZE - 32);
		fbi->dma5_frame->fsadr = fbi->screen_dma;
		fbi->dma5_frame->fidr  = 0;
		fbi->dma5_frame->ldcmd = fbi->fb.fix.smem_len;

		/* alloc & set default cmap */
		err = fb_alloc_cmap(&fbi->fb.cmap, fbi->palette_size, 0);
		if (err) return err;
		err = fb_set_cmap(&fbi->fb.cmap, 1, cursorfb_setcolreg, info);
		if (err) return err;

		changed = 1;
	}

	/* update overlay info */
	if( (xpos != fbi->xpos) || (ypos != fbi->ypos) ) {
		fbi->xpos = xpos;
		fbi->ypos = ypos;
		changed = 1;
	}

	if (changed) cursorfb_enable(info);
	set_ctrlr_state(fbi->basefb, C_REENABLE);

	return 0;
}

static int
cursorfb_get_cmap(struct fb_cmap *cmap, int kspc, int con, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;

	fb_copy_cmap(&fbi->fb.cmap, cmap, kspc ? 0 : 1);

	return 0;
}

static int
cursorfb_set_cmap(struct fb_cmap *cmap, int kspc, int con,
		  struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	int err;

	err = fb_alloc_cmap(&fbi->fb.cmap, fbi->palette_size, 0);

	if (!err) 
		err = fb_set_cmap(cmap, kspc, cursorfb_setcolreg, info);

	if (!err)
		fb_copy_cmap(cmap, &fbi->fb.cmap, kspc ? 0 : 1);

	return err;
}

static struct fb_ops cursorfb_ops = {
	owner:		THIS_MODULE,
	fb_get_fix:	cursorfb_get_fix,
	fb_get_var:	cursorfb_get_var,
	fb_set_var:	cursorfb_set_var,
	fb_get_cmap:	cursorfb_get_cmap,
	fb_set_cmap:	cursorfb_set_cmap,
};
#endif //
static struct overlayfb_info * __init overlay1fb_init_fbinfo(void)
{
	struct overlayfb_info *fbi;

	fbi = kmalloc(sizeof(struct overlayfb_info) + sizeof(u16) * 16, GFP_KERNEL);
	if (!fbi)
		return NULL;

	memset(fbi, 0, sizeof(struct overlayfb_info) );

	fbi->refcount = 0;
	init_MUTEX(&fbi->mutex);

	//strcpy(fbi->fb.fix.id, "overlay1");
	fbi->fb.fix.id[0] = PXAFB_MAIN_OVL1;

	fbi->fb.fix.type	= FB_TYPE_PACKED_PIXELS;
	fbi->fb.fix.type_aux	= 0;
	fbi->fb.fix.xpanstep	= 0;
	fbi->fb.fix.ypanstep	= 0;
	fbi->fb.fix.ywrapstep	= 0;
	fbi->fb.fix.accel	= FB_ACCEL_NONE;

	fbi->fb.var.nonstd	= 0;
	fbi->fb.var.activate	= FB_ACTIVATE_NOW;
	fbi->fb.var.height	= -1;
	fbi->fb.var.width	= -1;
	fbi->fb.var.accel_flags	= 0;
	fbi->fb.var.vmode	= FB_VMODE_NONINTERLACED;

	//strcpy(fbi->fb.modename, "overlay1");
	fbi->fb.modename[0] = PXAFB_MAIN_OVL1;
	strcpy(fbi->fb.fontname, "Acorn8x8");
	fbi->fb.fbops		= &overlay1fb_ops;
	fbi->fb.changevar	= NULL;
	fbi->fb.switch_con	= NULL;
	fbi->fb.updatevar	= dummy_updatevar;
	fbi->fb.blank		= overlay1fb_blank;
	fbi->fb.flags		= FBINFO_FLAG_DEFAULT;
	fbi->fb.node		= -1;
	fbi->fb.monspecs	= monspecs;
	//fbi->fb.disp		= NULL;
	if (fbi->fb.modename[0] == PXAFB_MAIN_VALID_FB)
	{
		fbi->fb.disp = pxafb_main->fb.disp;
		fbi->fb.pseudo_palette	= (void *)(fbi->fb.disp + 1);  //actually overlay1fb->fb.pseudo_palette points to pxafb_main->fb.pseudo_palette //
	}
	else
	{
		fbi->fb.disp = NULL;
		fbi->fb.pseudo_palette	= NULL;
	}

	fbi->xpos   	= 0;
	fbi->ypos   	= 0;
	fbi->format 	= -1;
	//fbi->enabled 	= 0;
	fbi->state 	= C_DISABLE;

	fbi->basefb = pxafb_main;  //Susan

	return fbi;
}

static struct overlayfb_info * __init overlay2fb_init_fbinfo(void)
{
	struct overlayfb_info *fbi;

	fbi = kmalloc(sizeof(struct overlayfb_info) + sizeof(u16) * 16, GFP_KERNEL);
	if (!fbi)
		return NULL;

	memset(fbi, 0, sizeof(struct overlayfb_info) );

	fbi->refcount = 0;
	init_MUTEX(&fbi->mutex);

	strcpy(fbi->fb.fix.id, "overlay2");

	fbi->fb.fix.type	= FB_TYPE_PACKED_PIXELS;
	fbi->fb.fix.type_aux	= 0;
	fbi->fb.fix.xpanstep	= 0;
	fbi->fb.fix.ypanstep	= 0;
	fbi->fb.fix.ywrapstep	= 0;
	fbi->fb.fix.accel	= FB_ACCEL_NONE;

	fbi->fb.var.nonstd	= 0;
	fbi->fb.var.activate	= FB_ACTIVATE_NOW;
	fbi->fb.var.height	= -1;
	fbi->fb.var.width	= -1;
	fbi->fb.var.accel_flags	= 0;
	fbi->fb.var.vmode	= FB_VMODE_NONINTERLACED;

	strcpy(fbi->fb.modename, "overlay2");
	strcpy(fbi->fb.fontname, "null");

	fbi->fb.fbops		= &overlay2fb_ops;
	fbi->fb.changevar	= NULL;
	fbi->fb.switch_con	= NULL;
	fbi->fb.updatevar	= dummy_updatevar;
	fbi->fb.blank		= overlay2fb_blank;
	fbi->fb.flags		= FBINFO_FLAG_DEFAULT;
	fbi->fb.node		= -1;
	fbi->fb.monspecs	= monspecs;
	fbi->fb.disp		= NULL;
	fbi->fb.pseudo_palette	= NULL;

	fbi->xpos   	= 0;
	fbi->ypos   	= 0;
	fbi->format 	= 3;
	fbi->state 	= C_DISABLE;

	init_waitqueue_head(&fbi->bs_wait); //Susan

	fbi->basefb = pxafb_main;
	pxafb_main->overlay2fb = fbi;

	return fbi;
}

#if CONFIG_CURSOR_CHANNEL 
static struct overlayfb_info * __init cursorfb_init_fbinfo(void)
{
	struct overlayfb_info *fbi;

	fbi = kmalloc(sizeof(struct overlayfb_info) + sizeof(u16) * 16, GFP_KERNEL);
	if (!fbi)
		return NULL;

	memset(fbi, 0, sizeof(struct overlayfb_info) );

	fbi->refcount = 0;
	init_MUTEX(&fbi->mutex);

	strcpy(fbi->fb.fix.id, "curosr");

	fbi->fb.fix.type	= FB_TYPE_PACKED_PIXELS;
	fbi->fb.fix.type_aux	= 0;
	fbi->fb.fix.xpanstep	= 0;
	fbi->fb.fix.ypanstep	= 0;
	fbi->fb.fix.ywrapstep	= 0;
	fbi->fb.fix.accel	= FB_ACCEL_NONE;

	fbi->fb.var.nonstd	= 0;
	fbi->fb.var.activate	= FB_ACTIVATE_NOW;
	fbi->fb.var.height	= -1;
	fbi->fb.var.width	= -1;
	fbi->fb.var.accel_flags	= 0;
	fbi->fb.var.vmode	= FB_VMODE_NONINTERLACED;

	strcpy(fbi->fb.modename, "cursor");
	strcpy(fbi->fb.fontname, "null");

	fbi->fb.fbops		= &cursorfb_ops;
	fbi->fb.changevar	= NULL;
	fbi->fb.switch_con	= NULL;
	fbi->fb.updatevar	= dummy_updatevar;
	fbi->fb.blank		= cursorfb_blank;
	fbi->fb.flags		= FBINFO_FLAG_DEFAULT;
	fbi->fb.node		= -1;
	fbi->fb.monspecs	= monspecs;
	fbi->fb.disp		= NULL;
	fbi->fb.pseudo_palette	= NULL;

	fbi->xpos   	= 0;
	fbi->ypos   	= 0;
	fbi->format 	= -1;
	fbi->state 	= C_DISABLE;

	fbi->basefb = pxafb_main;  //Susan

	return fbi;
}
#endif
#endif /* CONFIG_CPU_BULVERDE */

void pxafb_init_global(void)
{
	DPRINTK("pxafb_init_global: begin\n");
	
	init_MUTEX(&pxafb_global_state.g_sem);

	pxafb_global_state.init_phase = 1;
	pxafb_global_state.active_panel = PXA_NONE_PANEL;
	pxafb_global_state.main_state = 0x0; //0b00
	pxafb_global_state.smart_state = 0x0; //0b00
	pxafb_global_state.pxafb_current = NULL;
	pxafb_global_state.bklight_main_dutycycle = DEFAULT_DUTYCYCLE;
	pxafb_global_state.bklight_cli_dutycycle = DEFAULT_DUTYCYCLE;
	pxafb_global_state.main_ovl2_status = PXAFB_OVL2_OFF;  //at the very being, overlay2 is not enabled -- Susan//
	pxafb_global_state.trans_value = 0x00DB;  /* Default k1=k2=k3=011, namely multipilation constant = 4/8  */
	pxafb_global_state.bkduty_range.min = MIN_DUTYCYCLE; /* Set visible backlight dutycycle range to default value*/
	pxafb_global_state.bkduty_range.max = MAX_DUTYCYCLE;


	DPRINTK("pxafb_init_global: end\n");
}
static struct pxafb_info * __init pxafb_init_fbinfo(pxa_panel  this_panel )
{
	struct pxafb_mach_info *inf;
	struct pxafb_info *fbi;

	DPRINTK("pxafb_init_fbinfo: begin for panel(%d)\n",this_panel);
	
	fbi = kmalloc(sizeof(struct pxafb_info) + sizeof(struct display) +
		      sizeof(u16) * 16, GFP_KERNEL);
	if (!fbi)
		return NULL;

	memset(fbi, 0, sizeof(struct pxafb_info) + sizeof(struct display));

	fbi->currcon		= -1;
	fbi->current_fb = &fbi->fb;

	INIT_LIST_HEAD(&(fbi->db_task_list));
	memset(&fbi->first_fb, 0x0, sizeof(struct fb_proto));
	memset(&fbi->second_fb, 0x0, sizeof(struct fb_proto));

	fbi->double_buffers = 0;

	//strcpy(fbi->fb.fix.id, PXA_NAME);  --Susan
	if (this_panel == PXA_MAIN_PANEL)
	{
		DPRINTK("pxafb_init_fbinfo: PXA_MAIN_PANEL,this_panel(%d)\n",this_panel);
		fbi->fb.fix.id[0] = PXAFB_MAIN_BASE;
	}
	else
	{
		DPRINTK("pxafb_init_fbinfo: NOT PXA_MAIN_PANEL,this_panel(%d)\n",this_panel);
		fbi->fb.fix.id[0] = PXAFB_CLI_BASE;
	}
	
	fbi->fb.fix.type	= FB_TYPE_PACKED_PIXELS;
	fbi->fb.fix.type_aux	= 0;
	fbi->fb.fix.xpanstep	= 0;
	fbi->fb.fix.ypanstep	= 0;
	fbi->fb.fix.ywrapstep	= 0;
	fbi->fb.fix.accel	= FB_ACCEL_NONE;

	fbi->fb.var.nonstd	= 0;
	fbi->fb.var.activate	= FB_ACTIVATE_NOW;
	fbi->fb.var.height	= -1;
	fbi->fb.var.width	= -1;
	fbi->fb.var.accel_flags	= 0;
	fbi->fb.var.vmode	= FB_VMODE_NONINTERLACED;

	/* For support 24to18bpp */
	//strcpy(fbi->fb.modename, PXA_NAME);
	if (this_panel == PXA_MAIN_PANEL)
	{
	printk(KERN_NOTICE "pxafb_init_fbinfo: PXA_MAIN_PANEL,this_panel(%d)\n",this_panel);
		fbi->fb.modename[0] = PXAFB_MAIN_BASE;
	}
	else
	{
	printk(KERN_NOTICE "pxafb_init_fbinfo: NOT PXA_MAIN_PANEL,this_panel(%d)\n",this_panel);
		fbi->fb.modename[0] = PXAFB_CLI_BASE;
	}
	
	strcpy(fbi->fb.fontname, "Acorn8x8");

	fbi->fb.fbops		= &pxafb_ops;
	fbi->fb.changevar	= NULL;
	fbi->fb.switch_con	= NULL;  //For the sake of overlay1 channel is served as display console -- pxafb_switch;
	fbi->fb.updatevar	= pxafb_updatevar;
	fbi->fb.blank		= pxafb_blank;
	fbi->fb.flags		= FBINFO_FLAG_DEFAULT;
	fbi->fb.node		= -1;
	fbi->fb.monspecs	= monspecs;
	fbi->fb.disp		= (struct display *)(fbi + 1);
	fbi->fb.pseudo_palette	= (void *)(fbi->fb.disp + 1);

	fbi->rgb[RGB_8]		= &rgb_8;
	fbi->rgb[RGB_16]	= &def_rgb_16;
	fbi->rgb[RGB_18]	= &def_rgb_18;
	fbi->rgb[RGB_19]	= &def_rgbt_19;
	fbi->rgb[RGB_24]	= &def_rgbt_24;

	inf = pxafb_get_machine_info(fbi);
	/* Note that the machine info keeps to be integrity with 24bpp configuration, the Intel sticky thing will be done in pxafb_init_fbinfo -- Susan*/

	fbi->max_xres			= inf->xres;
	fbi->fb.var.xres		= inf->xres;
	fbi->fb.var.xres_virtual	= inf->xres;
	fbi->max_yres			= inf->yres;
	fbi->fb.var.yres		= inf->yres;
	fbi->fb.var.yres_virtual	= inf->yres; //sensitive for double buffers//

	if (this_panel == PXA_MAIN_PANEL)
	{
#ifdef CONFIG_FB_PXA_24BPP	
		fbi->max_bpp	= 25; //For 24bpp,overlay1 channel is tricky to be the exported framebuffer channel
#else
		fbi->max_bpp 	= inf->bpp;  //18bpp and 16bpp are normal cases//
#endif
	}
	else
		fbi->max_bpp = inf->bpp;	//For CLI smart panel, no overlay1 tricky framebuffer //

	fbi->fb.var.bits_per_pixel	= inf->bpp;  //For 24bpp tricky case, main panel base fb.var.bits_per_pixel is 2bpp
	
	fbi->fb.var.pixclock		= inf->pixclock;
	fbi->fb.var.hsync_len		= inf->hsync_len;
	fbi->fb.var.left_margin		= inf->left_margin;
	fbi->fb.var.right_margin	= inf->right_margin;
	fbi->fb.var.vsync_len		= inf->vsync_len;
	fbi->fb.var.upper_margin	= inf->upper_margin;
	fbi->fb.var.lower_margin	= inf->lower_margin;
	fbi->fb.var.sync		= inf->sync;
	fbi->fb.var.grayscale		= inf->cmap_greyscale;
	fbi->cmap_inverse		= inf->cmap_inverse;
	fbi->cmap_static		= inf->cmap_static;
	fbi->reg_lccr0              = inf->lccr0;
	fbi->reg_lccr1              = inf->lccr1;
	fbi->reg_lccr2              = inf->lccr2;
	fbi->reg_lccr3              = inf->lccr3;
	fbi->reg_lccr4              = inf->lccr4;
	fbi->reg_lccr5              = inf->lccr5;
	
	fbi->state			= C_DISABLE;
	fbi->task_state			= (u_char)-1;

//	if (fbi->max_bpp <= 16) -- take into account of tricky case: fbi->max_bpp != fbi->fb.var.bits_per_pixel //
	if (fbi->fb.var.bits_per_pixel <= 16)
		fbi->fb.fix.smem_len = fbi->max_xres * fbi->max_yres * fbi->fb.var.bits_per_pixel / 8; //sensitive for double buffers//
	else if (fbi->fb.var.bits_per_pixel > 19)
		fbi->fb.fix.smem_len = fbi->max_xres * fbi->max_yres * 4;
	else
		/* packed format */
		fbi->fb.fix.smem_len = fbi->max_xres * fbi->max_yres * 3;

	init_timer(&fbi->bklight_timer);
//	fbi->bklight_timer.function = pxafb_ezx_Backlight_turn_on;

	init_waitqueue_head(&fbi->ctrlr_wait);
	INIT_TQUEUE(&fbi->task, pxafb_task, fbi);
	init_MUTEX(&fbi->ctrlr_sem);

#ifdef CONFIG_PXAFB_CLI
	if (this_panel == PXA_SMART_PANEL)
	{
		//init_timer(&fbi->smart_refresh_timer);
		//fbi->smart_refresh_timer.function = pxafb_smart_refresh_schedule;

		INIT_TQUEUE(&fbi->smart_refresh_task, pxafb_smart_refresh, fbi);
	}
	else
#endif
	{
		//init_timer(&fbi->smart_refresh_timer);
		fbi->smart_refresh_timer.function = NULL;
		INIT_TQUEUE(&fbi->smart_refresh_task, NULL, fbi);
	}

	return fbi;
}

/*Changed for adding CLI smart panel support */
#if (0)  //#ifdef CONFIG_CPU_BULVERDE
static int __init pxafb_init_overlay(struct pxafb_info *fbi)
{
	int ret;
	struct overlayfb_info *overlay1fb, *overlay2fb, *cursorfb;

	ret = -1;
	overlay1fb = overlay2fb = cursorfb = NULL;

	/* Overlay 1 windows */
	overlay1fb = overlay1fb_init_fbinfo();

	if(!overlay1fb) {
		ret = -ENOMEM;
		printk(KERN_ERR "PXA FB: overlay1fb_init_fbinfo failed\n");
		goto failed;
	}

	ret = register_framebuffer(&overlay1fb->fb);
	if (ret<0) goto failed;

	/* Overlay 2 window */
	overlay2fb = overlay2fb_init_fbinfo();

	if(!overlay2fb) {
		ret = -ENOMEM;
		printk(KERN_ERR "PXA FB: overlay2fb_init_fbinfo failed\n");
		goto failed;
	}

	ret = register_framebuffer(&overlay2fb->fb);
	if (ret<0) goto failed;

	/* Hardware cursor window */
	cursorfb = cursorfb_init_fbinfo();

	if(!cursorfb) {
		ret = -ENOMEM;
		printk(KERN_ERR "PXA FB: cursorfb_init_fbinfo failed\n");
		goto failed;
	}

	ret = register_framebuffer(&cursorfb->fb);
	if (ret<0) goto failed;

	/* set refernce to Overlays  */
	fbi->overlay1fb = overlay1fb;
	fbi->overlay2fb = overlay2fb;
	fbi->cursorfb = cursorfb;

	/* set refernce to BaseFrame */
	overlay1fb->basefb = fbi;
	overlay2fb->basefb = fbi;
	cursorfb->basefb = fbi;
	
	return 0;
failed:
	if (overlay1fb) 
		kfree(overlay1fb);
	if (overlay2fb) 
		kfree(overlay2fb);
	if (cursorfb)
		kfree(cursorfb);
	return ret;
}

static void pxafb_deinit_overlay(struct pxafb_info *fbi)
{
	/* Unregister overlays and cursor framebuffers and free it's memory */
	unregister_framebuffer(&fbi->cursorfb->fb);
	consistent_free((void *)fbi->cursorfb->map_cpu, fbi->cursorfb->map_size, fbi->cursorfb->map_dma);
	kfree(fbi->cursorfb);

	unregister_framebuffer(&fbi->overlay1fb->fb);
	consistent_free((void *)fbi->overlay1fb->map_cpu, fbi->overlay1fb->map_size, fbi->overlay1fb->map_dma);
	kfree(fbi->overlay1fb);

	unregister_framebuffer(&fbi->overlay2fb->fb);
	consistent_free((void *)fbi->overlay2fb->map_cpu, fbi->overlay2fb->map_size, fbi->overlay2fb->map_dma);
	kfree(fbi->overlay2fb);
}
#endif
int __init pxafb_init(void)
{
	int ret;
#ifdef CONFIG_CPU_BULVERDE
	struct overlayfb_info *overlay1fb, *overlay2fb, *cursorfb;

	overlay1fb = overlay2fb = cursorfb = NULL;
#endif
	DPRINTK("pxafb_init:begin\n");
	
	pxafb_init_global();

	pxafb_main = pxafb_init_fbinfo(PXA_MAIN_PANEL);
/* For support of CLI panel -- Susan */
#ifdef CONFIG_PXAFB_CLI
	pxafb_smart = pxafb_init_fbinfo(PXA_SMART_PANEL);
#endif

	/* For supporting of CLI panel --Susan */
	if (PXA_DEFAULT_PANEL == PXA_SMART_PANEL)
		pxafb_global_state.pxafb_current = pxafb_smart;
	else
		pxafb_global_state.pxafb_current = pxafb_main;

	DPRINTK("pxafb_init: pxafb_global_state.pxafb_current(0x%x), pxafb_main(0x%x), pxafb_smart(0x%x)\n",pxafb_global_state.pxafb_current,pxafb_main,pxafb_smart);
	
	ret = -ENOMEM;
#ifdef CONFIG_PXAFB_CLI
	if ((!pxafb_main) || (!pxafb_smart))
		goto failed;
#else
	if (!pxafb_main)
		goto failed;
#endif

	if(machine_is_pxa_cerf()) {
		// brightness&contrast is handled via lcdctrl.
		lcdctrl_init();
	}

	/* Initialize video memory */
	ret = pxafb_map_video_memory(pxafb_main);
	if (ret)
		goto failed;
	
#if  (CONFIG_FBBASE_DOUBLE_BUFFERS && ENABLE_BASE_DB_AT_INIT)
	if (PXAFB_MAIN_VALID_FB == PXAFB_MAIN_BASE)
	{
		pxafb_ioctl(0,0,FBIOENABLE2BFS,0,0,&(pxafb_main->fb));  //currently we only support double buffering for main LCD panel//	
	}
#endif

#ifdef CONFIG_PXAFB_CLI
	ret = pxafb_map_video_memory(pxafb_smart);
	if (ret)
		goto failed;
#endif

	ret = request_irq(IRQ_LCD, pxafb_handle_irq, SA_INTERRUPT, "LCD", pxafb_global_state.pxafb_current);  //no matter main or cli, register LCD controller irq //
	if (ret) {
		printk(KERN_ERR "pxafb: failed in request_irq: %d\n", ret);
		goto failed;
	}

	pxafb_set_var(&pxafb_main->fb.var, -1, &pxafb_main->fb);

#ifdef CONFIG_PXAFB_CLI
	pxafb_set_var(&pxafb_smart->fb.var, -1, &pxafb_smart->fb);
#endif

	ret = register_framebuffer(&pxafb_main->fb);
	if (ret < 0)
		goto failed;
#ifdef CONFIG_PXAFB_CLI
	ret = register_framebuffer(&pxafb_smart->fb);
	if (ret < 0)
		goto failed;
#endif

#ifdef BVD_SECURE_LCD_WORKAROUND
	pm_register(PM_SYS_DEV, PM_SYS_VGA, pxafb_fb_callback);
#endif

#if (0)
#ifdef CONFIG_PM
	/*
	 * Note that the console registers this as well, but we want to
	 * power down the display prior to sleeping.
	 */
	pxafbi->pm = pm_register(PM_SYS_DEV, PM_SYS_VGA, pxafb_pm_callback);
	if (pxafbi->pm)
		pxafbi->pm->data = pxafbi;
#endif
#endif

#if 1 /* MVL-CEE */
	pxafb_ldm_register();
#endif
#ifdef FCS_EOF
        init_waitqueue_head(&lcd_eof_fcs_wait_q);
#endif

#ifdef CONFIG_CPU_FREQ
	pxafb_global_state.pxafb_current->clockchg.notifier_call = pxafb_clkchg_notifier;
	cpufreq_register_notifier(&(pxafb_global_state.pxafb_current->clockchg));
#endif

	/* Overlay 1 windows */
	overlay1fb = overlay1fb_init_fbinfo();

	if(!overlay1fb) {
		ret = -ENOMEM;
		printk(KERN_ERR "PXA FB: overlay1fb_init_fbinfo failed\n");
		goto failed;
	}

	/* set refernce to Overlays  */
	pxafb_main->overlay1fb = overlay1fb;

	/* set refernce to BaseFrame */
	overlay1fb->basefb = pxafb_main;

	if (PXAFB_MAIN_VALID_FB == PXAFB_MAIN_OVL1)
	{
	/*
	 *  Set up Overlay 1 max screen size
	 *  25 BPP RGB
	 *  Positioned at (0,0)
	 */		
		struct fb_var_screeninfo var = pxafb_main->fb.var; //base fb_info.var //

		pxafb_main->current_fb = &overlay1fb->fb;
		
		var.bits_per_pixel = 25;  //For 24bpp tricky case, main panel base fb.var.bits_per_pixel is 2bpp
		
		/* color mode | ypos | xpos */
	    	var.nonstd = (0 << 20) | (0 << 10) | 0;
    		ret = overlay1fb_pure_set_var(&var, -1, &overlay1fb->fb);
		if (ret)
		{
			DPRINTK( "Overlay 1 Preallocation failed. ret = %d", ret );
			goto failed;
		}
		else
		{
			DPRINTK( "Overlay 1 Preallocated");
			DPRINTK( "var.xres = %d", overlay1fb->fb.var.xres);
			DPRINTK( "var.yres = %d", overlay1fb->fb.var.yres);
	    		DPRINTK( "var.bits_per_pixel = %d", overlay1fb->fb.var.bits_per_pixel);
	    		DPRINTK( "var.nonstd = %#x", overlay1fb->fb.var.nonstd);
		}

		ret= overlay1fb_map_video_memory((struct fb_info*)&(overlay1fb->fb));
		if (ret) 
			goto failed;

		overlay1fb->state = C_ENABLE;
		
#if (0)
#if CONFIG_FBBASE_DOUBLE_BUFFERS
		pxafb_ioctl(0,0,FBIOENABLE2BFS,0,0,&(pxafb_main->fb));  //currently we only support double buffering for main LCD panel//
		
#endif
#endif
	}	

	ret = register_framebuffer(&overlay1fb->fb);
	if (ret<0) goto failed;
	
#ifdef CONFIG_OVL2_CHANNEL
	/* Overlay 2 window */
	DPRINTK("pxafb_init: overlay2fb_init_fbinfo\n");
	overlay2fb = overlay2fb_init_fbinfo();
	if(!overlay2fb) {
		ret = -ENOMEM;
		printk(KERN_ERR "PXA FB: overlay2fb_init_fbinfo failed\n");
		goto failed;
	}
	/* Because OVL2 is not an independant fb device (/dev/fbi), don't need to register_framebuffer -- Susan */
	// ret = register_framebuffer(&overlay2fb->fb);
	// if (ret<0) goto failed;

	DPRINTK("pxafb_init: overlay2fb_pure_set_var");
	ret = overlay2fb_pure_set_var(&(pxafb_main->fb.var), -1, (struct fb_info *)overlay2fb);
	if (ret)
	{
		DPRINTK("overlay2fb_pure_set_var returns error(%d)\n", ret);
		goto failed;
	}

	DPRINTK("pxafb_init: overlay2fb_map_YUV_memory(for two framebuffers)");
	/* FIXME */
	if (overlay2fb->format == 0x4)
		overlay2fb_YUV420_fixup((struct fb_info *)overlay2fb);

	if (overlay2fb->format == 0) 
		ret = overlay2fb_map_RGB_memory((struct fb_info *)overlay2fb);
	else  
		ret = overlay2fb_map_YUV_memory((struct fb_info *)overlay2fb);
	if (ret)
	{
		DPRINTK("overlay2fb_map_YUV_memory returns error(%d)\n", ret);
		goto failed;
	}

#if (CONFIG_CURSOR_CHANNEL)
	/* Hardware cursor window */
	cursorfb = cursorfb_init_fbinfo();

	if(!cursorfb) {
		ret = -ENOMEM;
		printk(KERN_ERR "PXA FB: cursorfb_init_fbinfo failed\n");
		goto failed;
	}

	ret = register_framebuffer(&cursorfb->fb);
	if (ret<0) goto failed;

	/* set refernce to Overlays  */
	pxafb_main->overlay1fb = overlay1fb;
	pxafb_main->overlay2fb = overlay2fb;
	pxafb_main->cursorfb = cursorfb;

	/* set refernce to BaseFrame */
	overlay1fb->basefb = pxafb_main;
	overlay2fb->basefb = pxafb_main;
	cursorfb->basefb = pxafb_main;
#endif

#endif  //End of CONFIG_CPU_BULVERDE

	/*
	 * Ok, now enable the LCD controller
	 */
#ifndef CONFIG_LOGO_SHOWED_BY_MBM
	LCDBSCR = LCD_BUFFER_STRENGTH;  //Because LCDBSCNTR[0] is ignored by special Intel function --Susan
	DPRINTK("pxafb_init: LCDBSCR(0x%x)\n", LCDBSCR);
#endif

#ifndef CONFIG_LOGO_SHOWED_BY_MBM
	/*The kernel can't reset backlight enable pin if it has 
	*  been done by MBM, otherwise, blanking will occur.
	*/
	lights_fl_update(LIGHTS_FL_APP_CTL_DEFAULT,1,LIGHTS_FL_REGION_DISPLAY_BL,LIGHTS_FL_COLOR_BLACK);
#endif	
	#if (0)
	/* Disable CLI backlight, Enable MAIN backlight */
	bklight_value = 0x40;
	power_ic_write_reg(POWER_IC_REG_FL_LS0, &bklight_value);
	bklight_value = 0x01;
	power_ic_write_reg(POWER_IC_REG_FL_LS1, &bklight_value);
	#endif
	
#ifndef CONFIG_LOGO_SHOWED_BY_MBM
	/* The kernel can't do the one time CLI initialization here if logo is showed by MBM,
	*   because it will cause blank in display and affect seamless logo showing. 
	*   It will be performed when flip is turned off first time.
	*/
#ifdef CONFIG_PXAFB_CLI
	/* Depending on some condition, choose to enable first LCD or second LCD */
	/* Initialize the LCD panel. Need to do initialize only once. */	
	set_ctrlr_state(pxafb_smart, C_ENABLE);

	DPRINTK("secondary_display=on");
	set_ctrlr_state(pxafb_smart, C_DISABLE);

#endif
#endif
	
#ifdef CONFIG_LOGO_SHOWED_BY_MBM
	/*Setting this flag here will make set_ctrlr_state(pxafb_main, C_ENABLE)
	*  skip LCD HW initialization. 
	*/
	powering_up = 1;

	//DPRINTK("Try to enable main panel, pxafb_global_state.pxafb_current(0x%x)\n", pxafb_global_state.pxafb_current);
	set_ctrlr_state(pxafb_main, C_ENABLE);

	powering_up = 0;
#else	
	/* Enable Main LCD */
	DPRINTK("Try to enable main panel, pxafb_global_state.pxafb_current(0x%x)\n", pxafb_global_state.pxafb_current);
	set_ctrlr_state(pxafb_main, C_ENABLE);
#endif
	#if (0)
	pxafb_main->bklight_timer.data = (unsigned long)pxafb_main;
	pxafb_main->bklight_timer.function = pxafb_bklight_turn_on;
	init_timer(&pxafb_main->bklight_timer);
	mod_timer(&pxafb_main->bklight_timer, jiffies + bk_timeout);
	#endif
#ifndef CONFIG_LOGO_SHOWED_BY_MBM	
	/*Don't need to turn it on, it has aleady been done if MBM shows logo,if do, will affect display*/
	pxafb_bklight_turn_on(pxafb_main);
#endif

	//mdelay(10000);
	
	pxafb_global_state.init_phase = 0;  //Finish Initialization successfully //
		
	return 0;

failed:
	if (pxafb_main)
		kfree(pxafb_main);
#ifdef CONFIG_PXAFB_CLI
	if (pxafb_smart)
		kfree(pxafb_smart);
#endif
	/* ToDo:  For double buffering, need to free the second fb   -- Susan */
	/* ToDo:  For PXAFB_MAIN_VALID_FB == PXAFB_MAIN_OVL1, need to free overlay1fb buffer #0 and #1 -- Susan */
#ifdef CONFIG_CPU_BULVERDE
	if (overlay1fb) 
		kfree(overlay1fb);
	if (overlay2fb) 
		kfree(overlay2fb);
#if (0)  //we don't have cursor settings //
	if (cursorfb)
		kfree(cursorfb);
#endif

#endif

	pxafb_global_state.init_phase = -1;  //Fail to initialize LCD driver //
	
	DPRINTK("pxafb_init: end\n");
	return ret;
}

static void __exit pxafb_exit(void)
{
	/* Disable LCD controller */
	if (pxafb_global_state.pxafb_current)
		set_ctrlr_state(pxafb_global_state.pxafb_current, C_DISABLE);

#if CONFIG_FBBASE_DOUBLE_BUFFERS
	pxafb_ioctl(0,0,FBIODISABLE2BFS,0,0,(struct fb_info *)pxafb_main);  //release second framebuffer space//
#endif

#ifdef CONFIG_CPU_BULVERDE
	/* Unregister overlays and cursor framebuffers and free it's memory */
#if (0)
	unregister_framebuffer(&pxafb_main->cursorfb->fb);
	consistent_free((void *)pxafb_main->cursorfb->map_cpu, pxafb_main->cursorfb->map_size, pxafb_main->cursorfb->map_dma);
	kfree(pxafb_main->cursorfb);

	unregister_framebuffer(&pxafb_main->overlay2fb->fb);
	consistent_free((void *)pxafb_main->overlay2fb->map_cpu, pxafb_main->overlay2fb->map_size, pxafb_main->overlay2fb->map_dma);
	kfree(pxafb_main->overlay2fb);
#endif	
	unregister_framebuffer(&pxafb_main->overlay1fb->fb);
	consistent_free((void *)pxafb_main->overlay1fb->map_cpu, pxafb_main->overlay1fb->map_size, pxafb_main->overlay1fb->map_dma);
	kfree(pxafb_main->overlay1fb);
#endif
#ifdef CONFIG_CPU_FREQ
	cpufreq_unregister_notifier(&pxafb_main->clockchg);
#endif
	/* Free LCD IRQ */
	if (PXA_DEFAULT_PANEL == PXA_MAIN_PANEL)
		free_irq(IRQ_LCD, pxafb_main);
#ifdef CONFIG_PXAFB_CLI
	if (PXA_DEFAULT_PANEL == PXA_SMART_PANEL)
		free_irq(IRQ_LCD, pxafb_smart);
#endif	
	/* Unregister main framebuffer and free it's memory */
	unregister_framebuffer(&pxafb_main->fb);

#ifdef CONFIG_CPU_BULVERDE
	if (sram_size) {
		DPRINTK("SRAM release...\n");
		sram_access_release(&sram_start, &sram_size);
	}
	else
#endif
	consistent_free(pxafb_main->map_cpu, pxafb_main->map_size, pxafb_main->map_dma);
	kfree(pxafb_main);
	pxafb_main = 0;

#ifdef CONFIG_PXAFB_CLI
	//ss:  pxafb_deinit_overlay(smartfbi);
	unregister_framebuffer(&pxafb_smart->fb);
	consistent_free(pxafb_smart->map_cpu, pxafb_smart->map_size, pxafb_smart->map_dma);
	kfree(pxafb_smart);
	pxafb_smart = 0;
#endif

#if 1 /* MVL-CEE */
	pxafb_ldm_unregister();
#endif

}

#ifdef MODULE
module_init(pxafb_init);
module_exit(pxafb_exit);
#endif

MODULE_AUTHOR("source@mvista.com, yu.tang@intel.com");
MODULE_DESCRIPTION("Framebuffer driver for Bulverde/PXA");
MODULE_LICENSE("GPL");


