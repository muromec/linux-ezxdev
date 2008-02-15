/*
 * drivers/video/mx21ads/mx21tvout.c
 *
 * FS453/454 TVOUT device driver
 *
 * Author: Ruslan Sushko <rsushko@ru.mvista.com, or source@mvista.com>
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/sysctl.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/delay.h>

#include <asm/arch/gpio.h>
#include <asm/arch/pll.h>
#include <asm/hardware.h>
#include <asm/hardware/fs453.h>

#include "dbmx21fb.h"

#include "mx21tvout.h"


#include "common.h"

#define MODULE_NAME "mx21tvout"

static int extclk_used = 0;

static char __initstate_i2c;

static unsigned short __normal_i2c_addr[] = { FS453_I2C_DEV_ID, I2C_CLIENT_END};
static unsigned short __ignore_i2c_addr[] = { I2C_CLIENT_END };

static struct i2c_client_address_data __i2c_addr_data = {
	.normal_i2c = __normal_i2c_addr,
	.normal_i2c_range = __ignore_i2c_addr,
	.probe = __ignore_i2c_addr,
	.probe_range = __ignore_i2c_addr,
	.ignore = __ignore_i2c_addr,
	.ignore_range = __ignore_i2c_addr,
	.force = __ignore_i2c_addr
};

static int __i2c_probe(struct i2c_adapter *adap);
static int __i2c_detach(struct i2c_client *client);
static int __i2c_command(struct i2c_client *client, unsigned int cmd,
			   void *arg);

static struct i2c_client *__i2c_client;

static struct i2c_driver __i2c_driver = {
	.name = "TVOUT",
	.id = I2C_DRIVERID_EXP0,	/* Fake Id */
	.flags = I2C_DF_NOTIFY,
	.attach_adapter = __i2c_probe,
	.detach_client = __i2c_detach,
	.command = __i2c_command
};

static int
__i2c_attach(struct i2c_adapter *adap, int addr, unsigned short flags,
	       int kind)
{
	struct i2c_client *c;

	c = (struct i2c_client *) kmalloc(sizeof (*c), GFP_KERNEL);

	if (!c)
		return -ENOMEM;

	strcpy(c->name, "TVOUT");
	c->id = __i2c_driver.id;
	c->flags = 0;
	c->addr = addr;
	c->adapter = adap;
	c->driver = &__i2c_driver;
	c->data = NULL;

	__i2c_client = c;

	return i2c_attach_client(c);
}

static int
__i2c_probe(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &__i2c_addr_data, __i2c_attach);
}

static int
__i2c_detach(struct i2c_client *client)
{
	i2c_detach_client(client);
	kfree(__i2c_client);
	return 0;
}

/* No commands defined */
static int
__i2c_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	return 0;
}



static void fs453_hardreset(void);

/************************** I2C primitives ********************************/

#define fs453_write_reg(_reg_, _val_)                   \
    SIO_write(FS453_I2C_##_reg_, (unsigned int)(_val_), \
            FS453_I2C_##_reg_##_SIZE)

#define fs453_read_reg(_reg_)                    \
    _SIO_read(FS453_I2C_##_reg_, FS453_I2C_##_reg_##_SIZE)

/*
 *      FS453 Register Write
 */
static int
SIO_write(unsigned char reg, unsigned int buf, int len)
{
        struct i2c_msg msg;
        uint8_t wbuf[5];

        wbuf[0] = reg;
        *((unsigned int *)(wbuf + 1)) = buf;

        msg.addr = FS453_I2C_DEV_ID;
        msg.flags = I2C_M_WR;
        msg.len = len > 4 ? 5 : len + 1;
        msg.buf = (char *) wbuf;

        if (i2c_transfer(__i2c_client->adapter, &msg, 1) != 1) {
                err("mx2_i2c_xfer failed\n");
                return -EIO;
        }
        return 0;
}

/*
 *      FS453 Register Read
 */
static int
SIO_read(unsigned char reg, unsigned int * buf, unsigned int len)
{
        struct i2c_msg msg[2];  /* Msg for write and read cycles */
        uint8_t subaddr = reg;
        *buf = 0;

        msg[0].addr = FS453_I2C_DEV_ID;
        msg[0].flags = I2C_M_WR;
        msg[0].len = 1;
        msg[0].buf = (char *) &subaddr;

        msg[1].addr = FS453_I2C_DEV_ID;
        msg[1].flags = I2C_M_RD;
        msg[1].len = len > 4 ? 4 :len;
        msg[1].buf = (char *) buf;

        if (i2c_transfer(__i2c_client->adapter, msg, 2) != 2) {
                err("mx2_i2c_xfer failed\n");
                return -EIO;
        }
        return 0;
}

/* Temporarry read wrapper*/
static inline unsigned int
_SIO_read(unsigned char reg, unsigned int len)
{
    unsigned int buf = 0;
    if (SIO_read(reg, &buf, len)) {
        return 0;
    }
    return buf;
}


/********************* FS453 TVOUT device control ***********************/

/* TV modes parameters should be in accordnace with tvmode_t enumeration */
tvmodeparam_t tvmodeparam[TVOUT_MODE_MAX] = {
    {625, 576, 864, 750, 50}, /* PAL 625/50 */
    {525, 484, 857, 700, 60}  /* NTSC */ /*The frame frequency for NTSC3.58 is
                                           59.94 */
};


#define MX2_FRAME_HSIZE_DELTA (4) /* FIXME: The total size of frame should be
                                     greater then active horizontal size
                                     because at least 4 clock blank zones will
                                     be produced by LCDC hardware */

#define MX2_FRAME_HSIZE_MAX_BLANK (263) /* FIXME: It seems it can be up to 323
                                           (According to LCDC_LVR register
                                           description) */

#define MX2_FRAME_VSIZE_DELTA (MX2_FRAME_HSIZE_DELTA)
#define MX2_FRAME_VSIZE_MAX_BLANK (MX2_FRAME_HSIZE_MAX_BLANK)

static int
calculate_frame_param(uint32_t xactres, 
        uint32_t yactres, 
        tvmode_t mode, 
        unsigned long lsclk,
        tvframe_t *frame
        )
{
    frame->mode = mode;
    frame->actlines = yactres;
    frame->actpixels = xactres;
    frame->totlines = frame->actlines * tvmodeparam[mode].totlines / 
        tvmodeparam[mode].actlines;

    frame->totpixels = lsclk / frame->totlines / tvmodeparam[mode].fps;

    if (frame->mode == TVOUT_MODE_NTSC358) {
        frame->totpixels +=1; /*FIXME: This hack is used to avoid floating
                                point operations in module. It used only with
                                current resolution in NTSC mode because we has
                                no ability to get correct frame size to achive
                                59.94 fps. Using other resolutions except
                                current (640x435) will cause changing or
                                removing this code. The best way to achive good
                                result is using floatin point operations with
                                rounding */
    }
    if (frame->totlines + MX2_FRAME_HSIZE_DELTA < frame->actlines) {
        err("Too small frame\n");
        return -1; /* Too small frame */
    }

    if (frame->totlines - frame->actlines > MX2_FRAME_VSIZE_MAX_BLANK) {
        err("Too big frame\n");
        return -1; /* Too big frame, The vertical blank zones can't be
                      reproduced  */
    }

    if (frame->totpixels + MX2_FRAME_HSIZE_DELTA < frame->actpixels) {
        err("Too slow clock rate\n");
        return -1; /* Not enough lsck rate to move frame to TVOUT */
    }
    
    if (frame->totpixels - frame->actpixels > MX2_FRAME_HSIZE_MAX_BLANK) {
        err("Too fast clock rate\n");
        return -1; /* The lsckl freq is too high to move frame to TVOUT, The
                      LCDC  hardware can't insert such blank zones */
    }
    return 0;
}

/*
 *	FS453 Soft Reset
 */
static void
fs453_softreset(void)
{
	uint32_t reg;

        reg = fs453_read_reg(CR);
        reg |= FS453_I2C_CR_SRESET;
        fs453_write_reg(CR, reg);
        mdelay(1);
        reg &= ~FS453_I2C_CR_SRESET;
        fs453_write_reg(CR, reg);
}

/*
 *	Sync System clock of FS453 to MX21
 */
static void 
fs453_bridgesync(void)
{
        uint32_t reg;

        /* Reset FS453 bridge */
        reg = fs453_read_reg(MISC);
        reg |= FS453_I2C_MISC_BRDG_RST;
        fs453_write_reg(MISC, reg);
        mdelay(1);
        reg &= ~FS453_I2C_MISC_BRDG_RST;
        fs453_write_reg(MISC, reg);

        /* Clear the CACQ and FIFO flags */
        reg = fs453_read_reg(CR);
        reg |= FS453_I2C_CR_CACQ_CLR | FS453_I2C_CR_FIFO_CLR;
        fs453_write_reg(CR, reg);
        mdelay(1);
        reg &= ~(FS453_I2C_CR_CACQ_CLR | FS453_I2C_CR_FIFO_CLR);
        fs453_write_reg(CR, reg);
        return;
}

static void
fs453_init(void)
{
    unsigned int reg;
    fs453_softreset();
    
    reg = fs453_read_reg(CR);
    reg &= ~FS453_I2C_CR_NCOEN; /* Disable NCO */
    fs453_write_reg(CR, reg);

    fs453_write_reg(PWR_MGNT, 
            FS453_SET_REG_F(PWR_MGNT_CLK_SOFF, CLK_SOFF_HDTV)); /* Turn off
                                                                   HDTV
                                                                   clock */
    fs453_write_reg(VID_CNTL0,
            FS453_I2C_VID_CNTL0_HSYNC_INV   |   /* Invert Hsync */
            FS453_I2C_VID_CNTL0_VSYNC_INV   |   /* Invert Vsync */
            FS453_I2C_VID_CNTL0_FIELD_MS        /* Field Master */
            );
}


static unsigned int
fs453_set_pll_clk(unsigned long long pllfreq, /* FS453 pll freq */
        unsigned long long extclk,            /* FS453 externla clk */
        unsigned long long intclk             /* FS453 internal clk */
        )
{
    unsigned int reg;
    unsigned long long freq;
    unsigned int ncon, ncod, plln, pllg, pllm, pllep, pllip;

    
    ncon = ncod = 1; /* Disable NCO block */
    fs453_write_reg(NCON, FS453_SET_REG_F(NCON_NCON, ncon));
    fs453_write_reg(NCOD, FS453_SET_REG_F(NCOD_NCOD, ncod));
    freq = FS453_XTAL_IN; /* NCO block disabled so output frequency will not
                             changed */

    /* Devide the XTAL_IN clock by 27 to get 1MHz output */
    plln = 27;
    fs453_write_reg(PLLN, FS453_SET_REG_F(PLLN_PLLN, plln - 1));
    freq /= plln;

    /* Now we canget any frequency which is multiple of 1 MHz
FIXME: The way to get working frequency should be reiplemented to have ability
       receive any frequency */
    
    /* Set PLL ouput clock to desired frequency.  Currenlty multiple of 1MHz
     * frequency supported only  */

    pllg = 4; /*FIXME: Selection from table should be mplemted */
    pllm = pllfreq / (unsigned long long)freq;

    reg = FS453_SET_REG_F(PLLMPC_PLLG, pllg) |
          FS453_SET_REG_F(PLLMPC_PLLM, pllm - 17);
    fs453_write_reg(PLLMPC, reg);

    pllep = pllfreq/extclk;
    pllip = pllfreq/intclk;
    fs453_write_reg(PLLPD, 
            FS453_SET_REG_F(PLLPD_IP, pllip - 1) |
            FS453_SET_REG_F(PLLPD_EP, pllep - 1) 
            );
    

    reg = fs453_read_reg(CR);    
    reg |= FS453_I2C_CR_NCOEN;
    fs453_write_reg(CR, reg);       /* Latch the NCO and PLL values */
    reg &= ~FS453_I2C_CR_NCOEN;
    reg  |=FS453_I2C_CR_GCC_CK_LVL; /* Set low level voltage */
    fs453_write_reg(CR, reg);

    return 0;
}

static int
fs453_switch_out_mode(tvframe_t *frm)
{
    unsigned int reg;
    unsigned int scaling_reg = 0;
    reg = 
        FS453_SET_REG_F(QPR_QK_INIT, QK_INIT_CODE)  | /* Init code:1001b */
        FS453_SET_REG_F(QPR_QK_MODE, MODE_800x600)  | /* XXX ?*/
        FS453_SET_REG_F(QPR_QK_OM, OM_COMP_SVID)    | /* Composit/S-video */
        FS453_SET_REG_F(QPR_QK_OS, OS_SDTV)         | /* S-video */
        FS453_SET_REG_F(QPR_QK_UIM, UIM_NATIONAL)   ; /*Universal input mode */

    switch (frm->mode) {
        case TVOUT_MODE_PAL625:
            reg |= FS453_SET_REG_F(QPR_QK_PN, PN_PAL);
            break;
        case TVOUT_MODE_NTSC358:
            reg |= FS453_SET_REG_F(QPR_QK_PN, PN_NTSC);
            break;
        default:
            return -1; /*Unknown mode */
    };
    fs453_write_reg(QPR, reg);
    
    fs453_write_reg(IHO, FS453_SET_REG_F(IHO_IHO, 2));
    fs453_write_reg(IVO, FS453_SET_REG_F(IVO_IVO, 2));

    fs453_write_reg(IHW, FS453_SET_REG_F(IHW_IHW, frm->actpixels & ~1u));
    if (frm->totlines > tvmodeparam[frm->mode].totlines) {
        /*downscaling */
        scaling_reg = FS353_FIFO_LAT_DOWNSCALING_VAL; /* Set fifo latency
                                                       * according to FIFO_LAT
                                                       * register description
                                                       */
        reg = tvmodeparam[frm->mode].totlines * 0x10000 / frm->totlines;
    } else {
        /* upscaling */
        scaling_reg = FS353_FIFO_LAT_UPSCALING_VAL;
        reg = tvmodeparam[frm->mode].totlines * 0x10000 / 
            frm->totlines - 0x10000; 
    }

    fs453_write_reg(VSC, FS453_SET_REG_F(VSC_VSC, reg));
    
    fs453_write_reg(FIFO_LAT, scaling_reg);

    if (frm->actpixels > tvmodeparam[frm->mode].actpixels) {
        uint32_t hdsc = 
            tvmodeparam[frm->mode].actpixels * 256 / frm->actpixels;
        
        reg = FS453_SET_REG_F(HSC_HDSC, hdsc) | 
            FS453_SET_REG_F(HSC_HUSC, 0);

    } else {
        uint32_t husc = 
            tvmodeparam[frm->mode].actpixels * 256 /  frm->actpixels - 256;

        reg = FS453_SET_REG_F(HSC_HDSC, 0) | 
            FS453_SET_REG_F(HSC_HUSC, husc);
    }

    fs453_write_reg(HSC, reg);

    fs453_write_reg(BYPASS, 
            FS453_I2C_BYPASS_CAC_BYPASS | FS453_I2C_BYPASS_HDS_BYPASS);

    reg = FS453_SET_REG_F(MISC_UIM_MOD, UIM_MOD_NATIONAL) | 
        FS453_I2C_MISCUV_SWAP;
    fs453_write_reg(MISC, reg);

    return 0;
}

static int
fs453_set_outport(tvout_port_t outport)
{
/*  DAC assignment (for TVOUT 2.0)
 *
 *  MODE A  B	 C   D
 *      S0 S1 S2  S3
 *  0    Y  C  GND CVBS
 *  2    G  R  B   GND
 *  1    G  R  B   CVBS
 */
        switch (outport) {
            case TVOUT_PORT_COMPOSIT:
                /* Connect Signal 3 (CVBS) to DAC D (Linked to composite port)*/
                fs453_write_reg(DAC_CNTL, 
                        FS453_SET_REG_F(DAC_CNTL_DAC_DMUX, DAC_DMUX_3_D));
                break;
            default:
                return -1;
        }
	
    return 0;
}

/********************* MX21 LCDC interfaces  ***********************/
static uint32_t MPCTL0_BAK;         /* MPCTL0 backup register   */
static uint32_t PCDR1_PERDIV3_BAK;  /* PERDIV3 placeholder      */

/*
 *	PLL init for FS453
 *
 *	Input  : 19MHz external osc
 *	Output : 266MHz FCLK
 */
void 
mx2_pll_ext_clk(void)
{
        unsigned int reg;
        CRM_CSCR |= CSCR_MCU_SEL;	/* MPLL select external clock source  */
        MPCTL0_BAK = CRM_MPCTL0;

        CRM_MPCTL0 = (7 << MPCTL0_MFI_SHIFT) & MPCTL0_MFI ;/* MPLL gen 266M
                                                              output with 19MHz
                                                              input */

        CRM_CSCR |= CSCR_MPLL_RESTART;	//MPLL restart
        while(1)	//poll for PLL lock
        {
        	if(CRM_MPCTL1 & MPCTL1_LF)
        		break;
        }

        reg = CRM_PCDR1;
        PCDR1_PERDIV3_BAK = reg & PCDR1_PERDIV3; /* Store old value */
        reg &= ~PCDR1_PERDIV3;
        reg |= ((0) << PCDR1_PERDIV3_SHIFT) & PCDR1_PERDIV3; //Devide by 1
        CRM_PCDR1 = reg;
        extclk_used = 1;
	return;
}


void 
mx2_pll_int_clk(void)
{
        unsigned int reg;
        if (!extclk_used) {
            return;
        }

        CRM_CSCR &= ~CSCR_MCU_SEL;	/* MPLL select external clock source  */
        
        CRM_MPCTL0= MPCTL0_BAK;

        CRM_CSCR |= CSCR_MPLL_RESTART;	//MPLL restart
        while(1)	        /* poll for PLL lock */
        {
        	if(CRM_MPCTL1 & MPCTL1_LF)
        		break;
        }

        reg = CRM_PCDR1;
        reg &= ~PCDR1_PERDIV3;
        reg |= PCDR1_PERDIV3_BAK;  /* Restore old value */
        CRM_PCDR1 = reg;

        extclk_used = 0;
	return;
}

int
mx2_lcdc_init(tvframe_t *frm, unsigned int frame_buf_addr)
{
        uint32_t x, y;
        uint32_t reg, wait1, wait2, width, blank, clkdiv, tmp;
        
	LCDC_LSSAR = frame_buf_addr;
        
        y = frm->actlines;
        x = frm->actpixels >> 4;
	LCDC_LSR = (x << 20) | y;

        LCDC_LVPWR = frm->actpixels/2; /* Size of LCD line in 32-bits words (16
                                          bits for each pixel) */
        
        clkdiv = FS453PLL_FREQ/LSCLK_FREQ - 1;
	LCDC_LPCR = 0xFA380080 | (clkdiv & 0x3F);
        
        width = 3;
        blank = frm->totpixels - frm->actpixels - width - 5; /* 5 is sum of
                                                                minimal value
                                                                for hwait1 +
                                                                hwait2 + width:
                                                                1-- min width +
                                                                1-- min wait1 +
                                                                3-- min wait2
                                                              */

        /* Move horizontal blank zone to the right*/
        wait2 = wait1 = blank / 2;
        wait1 += blank % 2;

        tmp =  255 - wait1;
        if (tmp < wait2) {
                wait2 -= tmp;
                wait1 += tmp;
        } else {
                wait1 += wait2;
                wait2 = 0;
        }
        
        reg =   ((width & 0x3F) << 26)  |
                ((wait1 & 0xFF) << 8)   |
                ((wait2 & 0xFF))        ;
        LCDC_LHCR = reg;

        /* Move horizontal blank zone to the top */
        width = 1;
        blank = frm->totlines - frm->actlines - width - 1; /* VWAIT2 should be
                                                              at least 1 clock
                                                              minimum */
        wait2 = wait1 = blank / 2;
        wait2 += blank % 2 + 1;

        reg =   ((width & 0x3F) << 26)  |
                ((wait1 & 0xFF) << 8)   |
                ((wait2 & 0xFF))        ;
        LCDC_LVCR = reg;

	LCDC_LSCR = 0x00090300;
	LCDC_LPCCR = 0x00A9008A;
	LCDC_LRMCR = 0x00000000; /* Disable self-refresh*/

        /* LCD DMA init*/
	LCDC_LDCR = 0x0003000F;	/* dynamic burst length */
        return 0;
}


/*
 *	Switch FS453 to selected mode
 */
static int
fs453_switch_mode(int xres, int yres, tvmode_t mode, tvframe_t *frm)
{

        fs453_init();

        if (calculate_frame_param(xres, yres, mode, LSCLK_FREQ, frm)) {
            err("Error to calculate frame paramters\n");
            return -1;
        }
        fs453_switch_out_mode(frm);

        fs453_set_pll_clk(FS453PLL_FREQ, FS453_MPLLIN_FREQ, LSCLK_FREQ);

        fs453_set_outport(TVOUT_PORT_COMPOSIT);

	return 0;
}

/*
 *	FS453 Hard Reset
 */
static void 
fs453_hardreset(void)
{
        mx2_gpio_set_bit(PORT_A, FS453_GPIO_RS, 0); 
        mdelay(10);
        mx2_gpio_set_bit(PORT_A, FS453_GPIO_RS, 1); 
}
    
    
static int
fs453_enable_device(void)
{
        return mx2_gpio_set_bit(PORT_A, FS453_GPIO_RS, 1); /* set reset line
                                                                 to high */
}

static int
fs453_enable_osc(void)
{
        return mx2_gpio_set_bit(PORT_A, FS453_GPIO_D0, 1); /* assert line*/
}

static int
fs453_disable_osc(void)
{
        return mx2_gpio_set_bit(PORT_A, FS453_GPIO_D0, 0); /* deassert line*/
}

static int
fs453_detect(void)
{
        if ( fs453_read_reg(ID) != FS453_I2C_ID_ID) {
                info("FS453 TVoutdevice has not been detected.\n");
                return -ENODEV;
        }
        info("TVout FS453 detected\n");

        return 0;
}

void
mx2_tvout_exit(void)
{
	if (__initstate_i2c) {
		i2c_del_driver(&__i2c_driver);
	__initstate_i2c = 0;
	}

        mx2_pll_int_clk();
        mx2_unregister_gpios(PORT_A,
                (FS453_GPIO_RS_MASK)      |
                (FS453_GPIO_D0_MASK)      |
                (FS453_GPIO_LSCLK_MASK)   |
                (FS453_GPIO_LD_MASK)      |
                (FS453_GPIO_VSYNC_MASK)   |
                (FS453_GPIO_HSYNC_MASK)   |
                (FS453_GPIO_ACDOE_MASK));
        
        fs453_disable_osc();
	if (__i2c_client) {
		fs453_write_reg(MISC_45, FS453_I2C_MISC_45_CLRBAR); /* Enable color
                                                               bar*/
	}
}


int
mx2_tvout_init(int xres, int yres, tvmode_t mode, unsigned int frame_buf_addr)
{
        tvframe_t frm;
	int tmp;

        if ((CLK_MPLL/1000000) != (FS453PLL_FREQ/1000000)) {
            err("The tvout cannot be used with MPLL frequency(%u)\n"
                "Current driver is implemted only for MPLL frequency %llu\n"
                "FS453 clocks should be recalculated to use this driver with"
                "current MPLL clock\n",
                CLK_MPLL, FS453PLL_FREQ);
            return -EINVAL;
        }

        mx2_register_gpios(PORT_A,
                (FS453_GPIO_RS_MASK) |      /* A25 Reset signal */
                (FS453_GPIO_D0_MASK),       /* A24 Clk Enable*/
                GPIO | OUTPUT | OCR_DATA | NOINTERRUPT | INIT_DATA_0
                );
        mx2_register_gpios(PORT_A, 
                (FS453_GPIO_LSCLK_MASK)   |
                (FS453_GPIO_LD_MASK)      |
                (FS453_GPIO_VSYNC_MASK)   |
                (FS453_GPIO_HSYNC_MASK)   |
                (FS453_GPIO_ACDOE_MASK), 
                PRIMARY);
        fs453_enable_device();
        fs453_enable_osc();

	__initstate_i2c = 0;

	tmp = i2c_add_driver(&__i2c_driver);
	if (tmp < 0) {
		err("cannot initialize I2C\n");
		mx2_tvout_exit();
		return tmp;
	}
	__initstate_i2c = 1;

	if (!__i2c_client) {
		err("wrong I2C address or device not present\n");
		mx2_tvout_exit();
		return -ENODEV;
	}

        if (fs453_detect()) {
                mx2_tvout_exit();
                return -1;
        }

	fs453_hardreset();  /* Do hard reset */
        
        if (fs453_switch_mode( xres, yres, mode, &frm)) {/* Switch FS453 to
                                                            selected mode */
                mx2_tvout_exit();
                return -1;
        };
         
	mx2_pll_ext_clk();  /* Switch to external clock */

        mx2_lcdc_init(&frm, frame_buf_addr); /* Perform LCDC initialization */

	fs453_bridgesync();


    return 0;
}

EXPORT_SYMBOL(mx2_tvout_init);
EXPORT_SYMBOL(mx2_tvout_exit);

