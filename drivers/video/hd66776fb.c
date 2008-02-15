/*
 * drivers/video/hd66776fb.c
 *
 * SH-73180 FrameBuffer driver for Super-H
 *
 * Author: Kazuyoshi Ishiwatari
 * Based on skeletonfb.c by Geert Uytterhoeven.
 *
 * 2003 (c) Kazuyoshi Ishiwatari. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/pm.h>
#include <linux/device.h>

#include <asm/machvec.h>
#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <asm/io.h>

#if defined(CONFIG_CPU_SUBTYPE_SH73180)
#include <asm/irq-sh73180.h>
#endif

#include <linux/fb.h>

#include <video/fbcon.h>
#include <video/fbcon-cfb16.h>


#define XRES 240
#define YRES 320

#if defined(CONFIG_CPU_SUBTYPE_SH73180)
#define UPDATE_FREQ 8
#else
#define UPDATE_FREQ 4
#endif

#if defined(CONFIG_CPU_SUBTYPE_SH73180)

#define HD66776_QVGA_BUFFER      0xadc00000	
#define HD66776_QVGA_BUFFER_ORG  0xadc30000
#define HD66776_QCIF_BUFFER      0xadc60000
#define HD66776_QVGA_SIZE        152600      


#define MSTPCR0 0xA4150030
#define MSTPCR2 0xA4150038
#define SBACR 0xFE80000C
#define SBOCR 0xFE800010

#define PACR            0xA4050100
#define PBCR            0xA4050102
#define PCCR            0xA4050104
#define PDCR            0xA4050106
#define PECR            0xA4050108
#define PFCR            0xA405010A
#define PGCR            0xA405010C
#define PHCR            0xA405010E
#define PJCR            0xA4050110
#define PKCR            0xA4050112
#define PLCR            0xA4050114
#define SCPCR           0xA4050116
#define PMCR            0xA4050118
#define PNCR            0xA405011A
#define PQCR            0xA405011C
#define PRCR            0xA405011E
#define PTCR            0xA405014C
#define PUCR            0xA405014E
#define PVCR            0xA4050150
#define PSELA           0xA4050140
#define PSELB           0xA4050142
#define PSELC           0xA4050144
#define PSELD           0xA4050156
#define PSELE           0xA4050158
#define DRVCR           0xA405014A

#define LCDC_REG_BASE	0xFE940000	/* 73180 LCDC */

#define LCDC_LDMT1R     (LCDC_REG_BASE+0x0418)
#define LCDC_LDMT2R     (LCDC_REG_BASE+0x041c)
#define LCDC_LDMT3R     (LCDC_REG_BASE+0x0420)
#define LCDC_LDDWD0R    (LCDC_REG_BASE+0x0480)
#define LCDC_LDDWD1R    (LCDC_REG_BASE+0x0484)
#define LCDC_LDDWD2R    (LCDC_REG_BASE+0x0488)
#define LCDC_LDDWD3R    (LCDC_REG_BASE+0x048c)
#define LCDC_LDDWD4R    (LCDC_REG_BASE+0x0490)
#define LCDC_LDDWD5R    (LCDC_REG_BASE+0x0494)
#define LCDC_LDDWD6R    (LCDC_REG_BASE+0x0498)
#define LCDC_LDDWD7R    (LCDC_REG_BASE+0x049c)
#define LCDC_LDDWD8R    (LCDC_REG_BASE+0x04a0)
#define LCDC_LDDWD9R    (LCDC_REG_BASE+0x04a4)
#define LCDC_LDDWDAR    (LCDC_REG_BASE+0x04a8)
#define LCDC_LDDWDBR    (LCDC_REG_BASE+0x04ac)
#define LCDC_LDDWDCR    (LCDC_REG_BASE+0x04b0)
#define LCDC_LDDRDR     (LCDC_REG_BASE+0x04c0)
#define LCDC_LDDWAR     (LCDC_REG_BASE+0x0600)
#define LCDC_LDDRAR     (LCDC_REG_BASE+0x0604)
#define LCDC_LDCNT1R    (LCDC_REG_BASE+0x046c)
#define LCDC_LDCNT2R    (LCDC_REG_BASE+0x0470)
#define LCDC_LDDFR      (LCDC_REG_BASE+0x0424)
#define LCDC_LDSM1R     (LCDC_REG_BASE+0x0428)
#define LCDC_LDSM2R     (LCDC_REG_BASE+0x042C)
#define LCDC_LDSAR      (LCDC_REG_BASE+0x0430)
#define LCDC_LDLAOR     (LCDC_REG_BASE+0x0434)
#define LCDC_LDWBFR     (LCDC_REG_BASE+0x0438)
#define LCDC_LDWBCNTR   (LCDC_REG_BASE+0x043c)
#define LCDC_LDWBAR     (LCDC_REG_BASE+0x0440)
#define LCDC_LDDCKR     (LCDC_REG_BASE+0x0410)
#define LCDC_LDDCKSTPR  (LCDC_REG_BASE+0x0414)
#define LCDC_LDDCKPAT1R (LCDC_REG_BASE+0x0400)
#define LCDC_LDDCKPAT2R (LCDC_REG_BASE+0x0404)
#define LCDC_LDDCKPAT3R (LCDC_REG_BASE+0x0408)
#define LCDC_LDDCKPAT4R (LCDC_REG_BASE+0x040c)
#define LCDC_LDHCNR  	(LCDC_REG_BASE+0x0448)
#define LCDC_LDHSYNR 	(LCDC_REG_BASE+0x044c)
#define LCDC_LDVLNR  	(LCDC_REG_BASE+0x0450)
#define LCDC_LDVSYNR 	(LCDC_REG_BASE+0x0454)
#define LCDC_LDHPDR  	(LCDC_REG_BASE+0x0458)
#define LCDC_LDVPDR  	(LCDC_REG_BASE+0x045c)
#define LCDC_LDDPMR  	(LCDC_REG_BASE+0x0468)
#define LCDC_LDVLNR  	(LCDC_REG_BASE+0x0450)
#define LCDC_LDVSYNR 	(LCDC_REG_BASE+0x0454)
#define LCDC_LDINTR 	(LCDC_REG_BASE+0x0460)
#endif

struct hd66776fb_info {
	struct fb_info_gen gen;
	unsigned long base, size;
	unsigned long dat, idx;
};

struct hd66776fb_par {
	/* nothing, so far */
};

static struct hd66776fb_info fb_info;
static struct hd66776fb_par current_par;
static int current_par_valid = 0;
static struct display disp;
static struct fb_var_screeninfo default_var;

static void hd66776fb_set_par(const void *fb_par, struct fb_info_gen *info);
static int hd66776fb_encode_var(struct fb_var_screeninfo *var, const void *par,
			    struct fb_info_gen *info);

static void hd66776fb_ldm_register(void);
static void hd66776fb_ldm_unregister(void);

#if defined(CONFIG_CPU_SUBTYPE_SH73180)
void hd66776_write(char *base_addr, unsigned long p, size_t count);
void hd66776_vmode(unsigned long arg);
void hd66776fb_wait(unsigned long n);
#endif

static u16 fbcon_cmap[256];

#if defined(CONFIG_CPU_SUBTYPE_SH73180)

static u16 *vfbuffer;
static u16 *vfbuffer_org;
u16 *qcifbuffer;
int hd66776fb_qcifmode; 
#else
static u16 vfbuffer[XRES*YRES] __attribute__ ((aligned (4096)));
#endif

struct timer_list hd66776fb_timer;

#if defined(CONFIG_CPU_SUBTYPE_SH73180)
/*
 * wait queue for processes wanting to open
 */
static wait_queue_head_t lcdc_reg;
static wait_queue_head_t lcdc_ve;

static int use_sleep; /* should we use sleep or polling */

static void lcdc_int_hdr(int irq, void *dev_id, struct pt_regs *regs)
{
	if( (ctrl_inl(LCDC_LDINTR) & 0x00000004) == 0x00000004 )
	{
		ctrl_outl( ctrl_inl(LCDC_LDINTR) & 0xfffffbfb, LCDC_LDINTR);
		wake_up_interruptible(&lcdc_reg); /* wake up */
	}
	else if( (ctrl_inl(LCDC_LDINTR) & 0x00000001) == 0x00000001 )
	{
		ctrl_outl( ctrl_inl(LCDC_LDINTR) & 0xfffffefe, LCDC_LDINTR);
		wake_up_interruptible(&lcdc_ve); /* wake up */
	}
}
#endif


static inline unsigned short
hd66776_inw(int a)
{
#if defined(CONFIG_CPU_SUBTYPE_SH73180)
	volatile unsigned int dummy;

	a = ((a & 0x0000ff00) << 2) + ((a & 0x000000ff) << 1);
	ctrl_outl(((volatile unsigned long) a + 0x10000000), LCDC_LDDWD0R);
	ctrl_outl(0, LCDC_LDDWD1R);

	while (1) {
		dummy = (ctrl_inl(LCDC_LDMT2R) & 0x10000000);
		if (!(dummy == 0x10000000)) {
			break;
		}
		udelay(10);
	}
	ctrl_outl(0x00000000, LCDC_LDDWAR);

	ctrl_outl(0x01000000, LCDC_LDDRDR);
	while (1) {
		dummy = (ctrl_inl(LCDC_LDMT2R) & 0x10000000);
		if (!(dummy == 0x10000000)) {
			break;
		}
		udelay(10);
	}
	dummy = ctrl_inl(LCDC_LDDRAR);

	return (ctrl_inl(LCDC_LDDRDR) & 0xffff);
#else
	ctrl_outw(a, fb_info.idx);
	return ctrl_inw(fb_info.dat);
#endif

}

static inline void
hd66776_outw(volatile unsigned int d, volatile int a)
{
#if defined(CONFIG_CPU_SUBTYPE_SH73180)
	volatile unsigned int dummy;
	volatile unsigned int val;
	dummy = ctrl_inl(LCDC_LDMT2R);
	if(dummy & 0x20000000) {
		printk("LCDC:SYS I/F command error. \n");
		ctrl_outl(ctrl_inl(LCDC_LDMT2R) & ~0x20000000,LCDC_LDMT2R);
	}
	ctrl_outl( ctrl_inl(LCDC_LDINTR) | 0x00000100, LCDC_LDINTR);
	if(use_sleep){
		interruptible_sleep_on(&lcdc_ve);
	}else
	{
		while( (ctrl_inl(LCDC_LDINTR) & 0x00000001) != 0x00000001 ){;}
		ctrl_outl( ctrl_inl(LCDC_LDINTR) & 0xfffffefe, LCDC_LDINTR);
	}

	val = ((a & 0x0000ff00)<<2) + ((a & 0x000000ff)<<1);
	ctrl_outl(((volatile unsigned long)val + 0x10000000), LCDC_LDDWD0R);    /* Write */

	val = ((d & 0x0000ff00)<<2) + ((d & 0x000000ff)<<1);
	ctrl_outl(((volatile unsigned long)val + 0x11000000), LCDC_LDDWD1R);    /* Data */

	ctrl_outl( 0x0, LCDC_LDDWD2R);  /* Data */

	ctrl_outl( ctrl_inl(LCDC_LDINTR) | 0x00000400, LCDC_LDINTR);
	if(use_sleep){
		interruptible_sleep_on(&lcdc_reg);
	}else
	{
		while( (ctrl_inl(LCDC_LDINTR) & 0x00000004) != 0x00000004 ){;}
		ctrl_outl( ctrl_inl(LCDC_LDINTR) & 0xfffffbfb, LCDC_LDINTR);
	}

	ctrl_outl(0x00000000, LCDC_LDDWAR);
#else
	ctrl_outw(a, fb_info.idx);
	ctrl_outw(d, fb_info.dat);
#endif
}

static inline void
hd66776_outw_p(volatile unsigned int d, volatile int a)
{
#if defined(CONFIG_CPU_SUBTYPE_SH73180)
	volatile unsigned int val;

	ctrl_outl( ctrl_inl(LCDC_LDINTR) | 0x00000100, LCDC_LDINTR);
	if(use_sleep){
		interruptible_sleep_on(&lcdc_ve);
	}else
	{
		while( (ctrl_inl(LCDC_LDINTR) & 0x00000001) != 0x00000001 ){;}
		ctrl_outl( ctrl_inl(LCDC_LDINTR) & 0xfffffefe, LCDC_LDINTR);
	}

	val =  ((a & 0x0000ff00)<<2) + ((a & 0x000000ff)<<1);
	ctrl_outl(((unsigned long)val + 0x10000000), LCDC_LDDWD0R);
	ctrl_outl( 0x0, LCDC_LDDWD1R);  /* Data */
                                                                                                                                                              
	ctrl_outl( ctrl_inl(LCDC_LDINTR) | 0x00000400, LCDC_LDINTR);
	if(use_sleep){
		interruptible_sleep_on(&lcdc_reg);
	}else
	{
		while( (ctrl_inl(LCDC_LDINTR) & 0x00000004) != 0x00000004 ){;}
		ctrl_outl( ctrl_inl(LCDC_LDINTR) & 0xfffffbfb, LCDC_LDINTR);
	}

	ctrl_outl(0x00000000, LCDC_LDDWAR);

	hd66776_outw((d | 0x0100), a);
	udelay(100);
#else
	int timeout=100;
	ctrl_outw(a, fb_info.idx);
	while(timeout-- && (ctrl_inw(fb_info.dat)&0x100))
			udelay(10);
	ctrl_outw(a, fb_info.idx);
	ctrl_outw(d|0x0100, fb_info.dat);
	udelay(100);
#endif
}

static inline void
hd66776_power_on(void)
{
	hd66776_outw(0x0001, 0x0000);
	mdelay(100);
	hd66776_outw(0x0000, 0x0100);
	hd66776_outw(0x0000, 0x0101);
	hd66776_outw(0x0000, 0x0102);
	hd66776_outw(0x0000, 0x0103);
	hd66776_outw(0x0014, 0x0104);
	hd66776_outw(0x0d1f, 0x0105);
	hd66776_outw_p(0x0000, 0x000a);
	hd66776_outw_p(0x0001, 0x000a);
	hd66776_outw_p(0x0002, 0x000a);
	hd66776_outw(0x19ec, 0x0101);
	hd66776_outw_p(0x0000, 0x000a);
	mdelay(40);
	hd66776_outw(0x2d1f, 0x0105);
	hd66776_outw_p(0x0000, 0x000a);
}

static inline void
hd66776_init_screen(int xres, int yres)
{
	hd66776_outw(0x0127, 0x0001);
	hd66776_outw(0x0000, 0x0002);
	hd66776_outw(0x0330, 0x0003);
	hd66776_outw(0x0000, 0x0007);
	hd66776_outw(0x0808, 0x0008);

	hd66776_outw(0x1006, 0x0010);
	hd66776_outw(0x1a06, 0x0011);
	hd66776_outw(0x0606, 0x0012);
	hd66776_outw(0x0000, 0x0013);
	hd66776_outw(0x0203, 0x0014);
	hd66776_outw(0x0000, 0x0015);
	
	hd66776_outw(0x0000, 0x0201);
	hd66776_outw(0x0000, 0x0202);
	hd66776_outw(0x0000, 0x0203);
	hd66776_outw(0x0000, 0x0204);

#if defined(CONFIG_CPU_SUBTYPE_SH73180)
	hd66776_outw(0x0001, 0x000c);
	hd66776_outw(0x0012, 0x000d);
#else
	hd66776_outw(0x0112, 0x000d);
	hd66776_outw(0x0000, 0x000c);
#endif

	hd66776_outw(0x0000, 0x0400);
	hd66776_outw(0x0000, 0x0401);
	hd66776_outw(0x0000, 0x0402);
	hd66776_outw(0x013f, 0x0403);
	hd66776_outw(0x013f, 0x0404);
	hd66776_outw(0x013f, 0x0405);

	hd66776_outw(0,   0x0406);
	hd66776_outw(xres-1, 0x0407);
	hd66776_outw(0,   0x0408);
	hd66776_outw(yres-1, 0x0409);
}

static inline void
hd66776_set_gamma(void)
{
	hd66776_outw(0x0100, 0x0300);
	hd66776_outw(0x0707, 0x0301);
	hd66776_outw(0x0102, 0x0302);
	hd66776_outw(0x0206, 0x0303);
	hd66776_outw(0x0302, 0x0304);
	hd66776_outw(0x0506, 0x0305);
	hd66776_outw(0x0000, 0x0306);
	hd66776_outw(0x0706, 0x0307);
	hd66776_outw(0x0206, 0x0308);
	hd66776_outw(0x0203, 0x0309);
}

static void hd66776_display_on(void)
{
	hd66776_outw(0x0001, 0x0007);
	mdelay(100);
	hd66776_outw(0x0022, 0x0007);
	hd66776_outw_p(0x0000, 0x000a);
#ifdef  MSDP220P01
	hd66776_outw(0x0033, 0x0007);
#else   /* MSDP220P02 */
	hd66776_outw(0x0037, 0x0007);
#endif
}

static void hd66776_display_off(void)
{
	hd66776_outw(0x001a, 0x0007);
	mdelay(100);
	hd66776_outw(0x0012, 0x0007);
	mdelay(100);
	hd66776_outw(0x0000, 0x0007);
}

static void hd66776_power_off(void)
{

	hd66776_outw(0x01ec, 0x0101);
	hd66776_outw(0x01e0, 0x0101);
	hd66776_outw(0x0d1f, 0x0105);
	hd66776_outw(0x0004, 0x0104);
}

static void hd66776_standby(void)
{
	hd66776_outw(0x0001, 0x0100);
}
static void hd66776_standby_cancel(void)
{
	hd66776_outw(0x0000, 0x0100);
}

static void hd66776_init(int xres, int yres)
{
	hd66776_power_on();
	hd66776_init_screen(xres, yres);
	hd66776_set_gamma();
	hd66776_display_on();
}

static void hd66776fb_update(unsigned long d);

static inline void
hd66776fb_restart_timer(struct timer_list *t, unsigned long d)
{
	init_timer(t);
	t->expires = jiffies + HZ/UPDATE_FREQ;
	t->data = d;
	t->function = hd66776fb_update;
	add_timer(t);
}

static void
hd66776fb_update(unsigned long d)
{
#if defined(CONFIG_CPU_SUBTYPE_SH73180)
        if(d >= (XRES*YRES) )
                d=0;
	disable_irq(LCDC_IRQ);

        ctrl_outl( ctrl_inl(LCDC_LDINTR) | 0x00001000, LCDC_LDINTR);          /* INTR? */

        ctrl_outl( 0x001d0000, LCDC_LDHPDR);    /* Set Data Format */
        ctrl_outl( 0x013f0000, LCDC_LDVPDR);    /* Set Data Format */
	if (hd66776fb_qcifmode == 0) 
		ctrl_outl(((unsigned int)(vfbuffer_org)&0x0fffffff) , LCDC_LDSAR);
	else
		ctrl_outl(((unsigned int)(HD66776_QVGA_BUFFER)&0x0fffffff) , LCDC_LDSAR);
        ctrl_outl((0x000001E0), LCDC_LDLAOR);
	ctrl_outl( (ctrl_inl(LCDC_LDSM2R) | 0x00000001), LCDC_LDSM2R);          /* ONE SHOT */

        while( (ctrl_inl(LCDC_LDINTR) & 0x00000010 ) != 0x00000010 );
        ctrl_outl( ctrl_inl(LCDC_LDINTR) & 0xffffefef, LCDC_LDINTR);

	enable_irq(LCDC_IRQ);
#else
	int cnt=XRES*YRES;
	if(d >= sizeof(vfbuffer)/2)
		d=0;
	hd66776_outw(d&0xff, 0x0200);
	hd66776_outw(d>>8,   0x0201);
	ctrl_outw(0x0202, fb_info.idx);
	while(cnt--)
		ctrl_outw(vfbuffer[d++], fb_info.dat);
#endif
	hd66776fb_restart_timer(&hd66776fb_timer, d);
}

static void hd66776fb_get_par(void *par, struct fb_info_gen *info);

static void
hd66776fb_detect(void)
{
#if defined(CONFIG_CPU_SUBTYPE_SH73180)
	fb_info.base = (unsigned long) vfbuffer_org;
	fb_info.size = XRES * YRES * 2;
#else
    fb_info.base = (unsigned long)vfbuffer;
    fb_info.size = sizeof(vfbuffer);
#endif
    fb_info.dat = 0xb8000000;
    fb_info.idx = 0xb8000004;

    hd66776_init(XRES, YRES);

    hd66776fb_get_par(&current_par, (struct fb_info_gen*)&fb_info);
    hd66776fb_encode_var(&default_var, &current_par, NULL);

    hd66776fb_restart_timer(&hd66776fb_timer, 0);
}

static int
hd66776fb_encode_fix(struct fb_fix_screeninfo *fix, const void *par,
				struct fb_info_gen *info)
{
    memset(fix, 0, sizeof(struct fb_fix_screeninfo));

    strcpy(fix->id, "hd66776fb");
    fix->smem_start = fb_info.base;
    fix->smem_len = fb_info.size;
    fix->type = FB_TYPE_PACKED_PIXELS;
    fix->type_aux = 0;
    fix->visual = FB_VISUAL_TRUECOLOR;
    fix->xpanstep = 0;
    fix->ypanstep = 0;
    fix->ywrapstep = 0;
    fix->line_length = XRES*2;

    return 0;
}

static int
hd66776fb_decode_var(const struct fb_var_screeninfo *var, void *par,
				struct fb_info_gen *info)
{
	return 0;
}

static int
hd66776fb_encode_var(struct fb_var_screeninfo *var, const void *par,
			     struct fb_info_gen *info)
{
    memset(var, 0, sizeof(*var));

    var->xres = XRES;
    var->yres = YRES;
    var->xres_virtual = var->xres;
    var->yres_virtual = var->yres;
    var->xoffset = 0;
    var->yoffset = 0;
    var->bits_per_pixel = 16;
    var->grayscale = 0;
    var->transp.offset = 0;
    var->transp.length = 0;
    var->transp.msb_right = 0;
    var->nonstd = 0;
    var->activate = 0;
    var->height = -1;
    var->width = -1;
    var->vmode = FB_VMODE_NONINTERLACED;
    var->pixclock = 0;
    var->sync = 0;
    var->left_margin = 0;
    var->right_margin = 0;
    var->upper_margin = 0;
    var->lower_margin = 0;
    var->hsync_len = 0;
    var->vsync_len = 0;

    var->red.offset = 11;
    var->red.length = 5;
    var->green.offset = 5;
    var->green.length = 6;
    var->blue.offset = 0;
    var->blue.length = 5;
    var->transp.offset = 0;
    var->transp.length = 0;

    var->red.msb_right = 0;
    var->green.msb_right = 0;
    var->blue.msb_right = 0;
    var->transp.msb_right = 0;

    return 0;
}

static void
hd66776fb_get_par(void *par, struct fb_info_gen *info)
{
    if (current_par_valid)
	*(struct hd66776fb_par *)par = current_par;
    else
	/* fill par with initial value */;
}

static void
hd66776fb_set_par(const void *par, struct fb_info_gen *info)
{
    current_par = *(struct hd66776fb_par*)par;
    current_par_valid = 1;
    /* set hardware */
}

static int
hd66776fb_getcolreg(unsigned regno, unsigned *red, unsigned *green,
		    unsigned *blue, unsigned *transp, struct fb_info *info)
{
    if (regno > 255)
	return 1;	

    *red = *green = *blue = regno << 8;
    *transp = 0;

    return 0;
}


static int hd66776fb_setcolreg(unsigned regno, unsigned red, unsigned green,
			   unsigned blue, unsigned transp,
			   struct fb_info *info)
{
    if (regno > 255)
	return 1;

    fbcon_cmap[regno] =
	((red   & 0xf800)      ) |
	((green & 0xfc00) >>  5) |
	((blue  & 0xf800) >> 11);

    return 0;
}


static int hd66776fb_pan_display(const struct fb_var_screeninfo *var,
			     struct fb_info_gen *info)
{
    return 0;
}


static int hd66776fb_blank(int blank_mode, struct fb_info_gen *info)
{
    return 0;
}

static void hd66776fb_set_disp(const void *par, struct display *disp,
			    struct fb_info_gen *info)
{
    disp->screen_base = (void *)fb_info.base;
    disp->scrollmode = SCROLL_YREDRAW;
    disp->dispsw = &fbcon_cfb16;
    disp->dispsw_data = fbcon_cmap;
}

struct fbgen_hwswitch hd66776fb_switch = {
    hd66776fb_detect,
    hd66776fb_encode_fix,
    hd66776fb_decode_var,
    hd66776fb_encode_var,
    hd66776fb_get_par,
    hd66776fb_set_par,
    hd66776fb_getcolreg,
    hd66776fb_setcolreg,
    hd66776fb_pan_display,
    hd66776fb_blank,
    hd66776fb_set_disp
};


static struct fb_ops hd66776fb_ops = {
    owner:		THIS_MODULE,
    fb_get_fix:		fbgen_get_fix,
    fb_get_var:		fbgen_get_var,
    fb_set_var:		fbgen_set_var,
    fb_get_cmap:	fbgen_get_cmap,
    fb_set_cmap:	fbgen_set_cmap,
    fb_pan_display:	fbgen_pan_display,
};

static void
hd66776fb_init_sib(void)
{
	ctrl_outl(0x00000001, SBACR);
	ctrl_outl(0x00000001, SBOCR);
}

static void
hd66776fb_init_pfc(void)
{
	ctrl_outw((ctrl_inw(PECR) & ~0x0300), PECR);	/* PTE         */
	ctrl_outw((ctrl_inw(PHCR) & ~0xffff), PHCR);	/* PTH7-0      */
	ctrl_outw((ctrl_inw(PJCR) & ~0x000f), PJCR);	/* PTJ1-0      */
	ctrl_outw((ctrl_inw(PLCR) & ~0xffff), PLCR);	/* PTL7-0      */
	ctrl_outw((ctrl_inw(PMCR) & ~0xffff), PMCR);	/* PTM7-0      */
	ctrl_outw((ctrl_inw(PRCR) & ~0xf000), PRCR);	/* PTR7-6      */
	ctrl_outw((ctrl_inw(PSELA) | 0x0080), PSELA);	/* bit7        */
	ctrl_outw((ctrl_inw(PSELB) | 0x8000), PSELB);	/* bit15       */
	ctrl_outw((ctrl_inw(DRVCR) | 0x0400), DRVCR);	/* bit10       */
}

static void
hd66776fb_init_hw(void)
{
	ctrl_outl((ctrl_inl(LCDC_LDCNT2R) & 0xfffffffe), LCDC_LDCNT2R);	/* DON=0 */
	while ((ctrl_inl(LCDC_LDDPMR) & 0x00000003) != 0x00000000) ;

	ctrl_outl(0x14011104, LCDC_LDMT1R);	/* SYS Interface/RGB18 */
	ctrl_outl(0x01000F09, LCDC_LDMT2R);	/* LCDCS(MAIN)/Write-cycle=200ns/Write-low=100ns  */
	ctrl_outl(0x000E3418, LCDC_LDMT3R);	/* Read-lach=200ns/read-cycl=800ns/Read-low=350ns */

	ctrl_outl(0x01000101, LCDC_LDSM1R);	/* Use FINT */

	ctrl_outl((ctrl_inl(LCDC_LDDCKSTPR) | 0x01), LCDC_LDDCKSTPR);	/* Stop Dot-clock */

	ctrl_outl((ctrl_inl(LCDC_LDDCKR) | 0x00010000), LCDC_LDDCKR);	/* Set Dot-clock pat */
	ctrl_outl(0x018c6318, LCDC_LDDCKPAT1R);
	ctrl_outl(0xc6318c63, LCDC_LDDCKPAT2R);
	ctrl_outl(0x0, LCDC_LDDCKPAT3R);
	ctrl_outl(0x0, LCDC_LDDCKPAT4R);
	ctrl_outl((ctrl_inl(LCDC_LDDCKR) | 0x0001013c), LCDC_LDDCKR);

	ctrl_outl((ctrl_inl(LCDC_LDDCKR) & 0xffffecff), LCDC_LDDCKR);	/* Non 1_1 Clock-rate */

	ctrl_outl((ctrl_inl(LCDC_LDDCKR) & 0xfffeffff), LCDC_LDDCKR);
	ctrl_outl((ctrl_inl(LCDC_LDDCKSTPR) & 0xfffffffe), LCDC_LDDCKSTPR);	/* Start Dot-clock  */

	ctrl_outl(0x00000013, LCDC_LDDFR);

	ctrl_outl(0x001d0027, LCDC_LDHCNR);
	ctrl_outl(0x00000022, LCDC_LDHSYNR);
	ctrl_outl(0x013f0149, LCDC_LDVLNR);
	ctrl_outl(0x00000146, LCDC_LDVSYNR);

	ctrl_outl(0x001d0000, LCDC_LDHPDR);
	ctrl_outl(0x013f0000, LCDC_LDVPDR);

	ctrl_outl(((unsigned int)(HD66776_QVGA_BUFFER)&0x0fffffff) , LCDC_LDSAR);
	ctrl_outl( (ctrl_inl(LCDC_LDCNT2R) | 0x00000001), LCDC_LDCNT2R);
	while( (ctrl_inl(LCDC_LDDPMR) & 0x00000003 ) != 0x00000003 );
}

int __init
hd66776fb_init(void)
{
#if defined(CONFIG_CPU_SUBTYPE_SH73180)
	vfbuffer = (unsigned short *) HD66776_QVGA_BUFFER;
	qcifbuffer = (unsigned short *) HD66776_QCIF_BUFFER;
        vfbuffer_org    =  (unsigned short *)HD66776_QVGA_BUFFER_ORG ;
	hd66776fb_qcifmode = 0; 

	memset( (unsigned int *)HD66776_QVGA_BUFFER ,0,HD66776_QVGA_SIZE);

	ctrl_outl(ctrl_inl(MSTPCR2) & 0xfffffffe, MSTPCR2);	/* LCDC-ON */

	hd66776fb_init_sib();

	hd66776fb_init_pfc();

	hd66776fb_init_hw();

	ctrl_outb(ctrl_inb(INTC_IMR4) | 0x01,INTC_IMCR4);

	request_irq(LCDC_IRQ, lcdc_int_hdr,SA_INTERRUPT, "lcdc-mint", NULL);
	init_waitqueue_head(&lcdc_reg);
	init_waitqueue_head(&lcdc_ve);
	use_sleep = 1;
#endif

    fb_info.gen.parsize = sizeof(struct hd66776fb_par);
    fb_info.gen.fbhw = &hd66776fb_switch;
    fb_info.gen.fbhw->detect();
    strcpy(fb_info.gen.info.modename, "hd66776fb");
    fb_info.gen.info.changevar = NULL;
    fb_info.gen.info.node = -1;
    fb_info.gen.info.fbops = &hd66776fb_ops;
    fb_info.gen.info.disp = &disp;
    fb_info.gen.info.switch_con = &fbgen_switch;
    fb_info.gen.info.updatevar = &fbgen_update_var;
    fb_info.gen.info.blank = &fbgen_blank;
    fb_info.gen.info.flags = FBINFO_FLAG_DEFAULT;

    fbgen_get_var(&disp.var, -1, &fb_info.gen.info);
    fbgen_do_set_var(&disp.var, 1, &fb_info.gen);
    fbgen_set_disp(-1, &fb_info.gen);
    fbgen_install_cmap(0, &fb_info.gen);

    if(register_framebuffer(&fb_info.gen.info)<0)
	return -EINVAL;

#if defined(CONFIG_CPU_SUBTYPE_SH73180)
	hd66776_outw(0, 0x0406);
	hd66776_outw(240 - 1, 0x0407);
	hd66776_outw(0, 0x0408);
	hd66776_outw(320 - 1, 0x0409);
	hd66776_outw(0, 0x0200);
	hd66776_outw(0, 0x0201);

	ctrl_outl( ctrl_inl(LCDC_LDINTR) | 0x00000100, LCDC_LDINTR);
	interruptible_sleep_on(&lcdc_ve);

	ctrl_outl(((unsigned long) 0x0804 + 0x10000000), LCDC_LDDWD0R);
	ctrl_outl(0x0, LCDC_LDDWD1R);

	ctrl_outl( ctrl_inl(LCDC_LDINTR) | 0x00000400, LCDC_LDINTR);
	interruptible_sleep_on(&lcdc_reg);
	ctrl_outl(0x00000000, LCDC_LDDWAR);

	ctrl_outl( ctrl_inl(LCDC_LDINTR) | 0x00000100, LCDC_LDINTR);
	interruptible_sleep_on(&lcdc_ve);

#endif

    printk(KERN_INFO "fb%d: %s frame buffer device\n",
	   GET_FB_IDX(fb_info.gen.info.node), fb_info.gen.info.modename);

    hd66776fb_ldm_register();
    return 0;
}


void hd66776fb_cleanup(void)
{
    unregister_framebuffer(&fb_info.gen.info);
    hd66776fb_ldm_unregister();
}

static int
hd66776fb_suspend(struct device * dev,  u32 state, u32 level )
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
#if defined(CONFIG_CPU_SUBTYPE_SH73180)
		use_sleep = 0;  /* fscaler called from interrupt, so we can't use interruptible_sleep_on() */
		disable_irq(LCDC_IRQ);
		del_timer(&hd66776fb_timer);
#endif
		hd66776_display_off();
		hd66776_power_off();
		hd66776_standby();
		break;
	}
	return(0);

}
static int
hd66776fb_resume(struct device * dev, u32 level )
{
	switch (level) {
	case RESUME_POWER_ON:

		ctrl_outl(ctrl_inl(MSTPCR2) & 0xfffffffe, MSTPCR2);	/* LCDC-ON */
		hd66776fb_init_sib();
		hd66776fb_init_pfc();
		hd66776fb_init_hw();
		ctrl_outb(ctrl_inb(INTC_IMR4) | 0x01,INTC_IMCR4);

		hd66776_outw(0, 0x0406);
		hd66776_outw(240 - 1, 0x0407);
		hd66776_outw(0, 0x0408);
		hd66776_outw(320 - 1, 0x0409);
		hd66776_outw(0, 0x0200);
		hd66776_outw(0, 0x0201);

		hd66776_standby_cancel();
		hd66776_power_on();
		hd66776_init_screen(XRES, YRES);
		hd66776_display_on();

#if defined(CONFIG_CPU_SUBTYPE_SH73180)
		ctrl_outl( ctrl_inl(LCDC_LDINTR) | 0x00000100, LCDC_LDINTR);
		while( (ctrl_inl(LCDC_LDINTR) & 0x00000001) != 0x00000001 ){;}
		ctrl_outl( ctrl_inl(LCDC_LDINTR) & 0xfffffefe, LCDC_LDINTR);

		ctrl_outl(((unsigned long) 0x0804 + 0x10000000), LCDC_LDDWD0R);
		ctrl_outl(0x0, LCDC_LDDWD1R);

		ctrl_outl( ctrl_inl(LCDC_LDINTR) | 0x00000400, LCDC_LDINTR);
		while( (ctrl_inl(LCDC_LDINTR) & 0x00000004) != 0x00000004 ){;}
		ctrl_outl( ctrl_inl(LCDC_LDINTR) & 0xfffffbfb, LCDC_LDINTR);
		ctrl_outl(0x00000000, LCDC_LDDWAR);

		ctrl_outl( ctrl_inl(LCDC_LDINTR) | 0x00000100, LCDC_LDINTR);
		while( (ctrl_inl(LCDC_LDINTR) & 0x00000001) != 0x00000001 ){;}
		ctrl_outl( ctrl_inl(LCDC_LDINTR) & 0xfffffefe, LCDC_LDINTR);

		hd66776fb_restart_timer(&hd66776fb_timer, 0);
		enable_irq(LCDC_IRQ);
		use_sleep = 1;
#endif
		break;
	}
	return(0);
}

static struct device_driver hd66776fb_driver_ldm = {
	name:      	"hd66776fb",
	devclass:  	NULL,
	probe:     	NULL,
	suspend:   	hd66776fb_suspend,
	resume:    	hd66776fb_resume,
	scale:	  	NULL,
	remove:    	NULL,
};

static struct device hd66776fb_device_ldm = {
	name:		"SUPERH FB LCD Controller",
	bus_id:		"lcd",
	driver: 	NULL,
	power_state:	DPM_POWER_ON,
};

static void hd66776fb_ldm_register(void)
{
	extern void plb_driver_register(struct device_driver *driver);
	extern void plb_device_register(struct device *device);

	plb_driver_register(&hd66776fb_driver_ldm);
	plb_device_register(&hd66776fb_device_ldm);
}

static void hd66776fb_ldm_unregister(void)
{
	extern void plb_driver_unregister(struct device_driver *driver);
	extern void plb_device_unregister(struct device *device);

	plb_driver_unregister(&hd66776fb_driver_ldm);
	plb_device_unregister(&hd66776fb_device_ldm);
}

#if defined(CONFIG_CPU_SUBTYPE_SH73180)
void
hd66776fb_wait(unsigned long n)
{
	volatile int i, j;
	for (j = 0; j < n; j++) {
		for (i = 0; i < 20; i++) ;
	}
	return;
}
#endif

MODULE_LICENSE("GPL");
#ifdef MODULE
module_init(hd66776fb_init);
#endif
module_exit(hd66776fb_cleanup);

EXPORT_SYMBOL(hd66776fb_suspend);
EXPORT_SYMBOL(hd66776fb_resume);


