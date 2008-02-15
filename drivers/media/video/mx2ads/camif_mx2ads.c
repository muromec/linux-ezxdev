/* drivers/media/video/mx2ads/camif_mx2ads.c
 *
 * Implementation of Camera Interface for MX2ADS platform.
 * this driver based on media/video/omap/camif_innovator.c
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 *
 * Author: MontaVista Software, Inc.
 *              source@mvista.com
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/videodev.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/dma.h>
#include <asm/hardware.h>
#include <asm/arch/gpio.h>
#include <asm/arch/pll.h>
#include "prpscale.h"

#define MODULE_NAME "camif"

#include "common.h"
#include "camif.h"

static struct camera_interface * this;

static canif_chan_stat_t chan_stat[CAMIF_CHANNELS_NUM];

static unsigned long camera_clk = 20000000; /* default camera clock*/

static spinlock_t camif_lock;

static int camera_module_present = 0;
static int mem_region_reserved = 0;

static int camif_close(int chan);

extern const struct image_size mx2ads_image_size[]; 

/***************************************************************************/
/*                       EMMA functionality                                */
/***************************************************************************/


static inline void
emma_disable_interrupts(int chan)
{
        if (chan == 0) {
                PRP_INTRCNTL &= ~(PRP2_ICR_CH1 | PRP_INTRCNTL_CH1ERR);
        } else {
                PRP_INTRCNTL &= ~(PRP2_ICR_CH2 | PRP_INTRCNTL_CH2ERR);
        }
}

static inline void
emma_enable_interrupts(int chan)
{
        if (chan == 0) {
                PRP_INTRCNTL |= (PRP2_ICR_CH1 | PRP_INTRCNTL_CH1ERR);
        } else {
                PRP_INTRCNTL |= (PRP2_ICR_CH2 | PRP_INTRCNTL_CH2ERR);
        }
}


static void 
emma_prp_isr(int irq, void *client_data, struct pt_regs *regs)
{
	unsigned long flags;
        unsigned int isr_reg = PRP_INTRSTATUS;
        int i;
        canif_chan_stat_t *cstat;
        unsigned int chan_masks[CAMIF_CHANNELS_NUM] = {
                (PRP_INTRSTAT_CH1BUF2 | PRP_INTRSTAT_CH1BUF1),
                (PRP_INTRSTAT_CH2BUF2 | PRP_INTRSTAT_CH2BUF1)
        };
        unsigned int chan_en = 0;
	spin_lock_irqsave(&camif_lock, flags);
        PRP_INTRSTATUS = isr_reg;

	if (isr_reg & (chan_masks[0] | chan_masks[1])) {
                for (i = 0, cstat = &(chan_stat[0]); 
                                i< CAMIF_CHANNELS_NUM; i++, cstat++) {
                        if (!(isr_reg & chan_masks[i])) {
                                continue;
                        }
                        /* It seems hw bug: Channel bit does not cleared after
                         * interrupt recieving */
	                PRP_CNTL &= ~(1<<i); /* Clear the channel */
                        /* callback to V4L2 layer */
                        cstat->capture_callback(cstat->callback_data);
                        if (cstat->snapshot_active) {
                                cstat->snapshot_active = 0;
                        }
                        if (cstat->abort) {
                                cstat->abort = 0;
                                cstat->streaming_active = 0;
		                wake_up_interruptible (&cstat->abortqueue);
                                emma_disable_interrupts(i);
                        }
                        
                }
	} else {
                err("eMMa Interrupt error %x\n", PRP_INTRSTATUS);
                /* TODO Reset should be done after eMMa error */
	}

        /* Engage all channels in same time */
        for (i = 0, cstat = &(chan_stat[0]); 
                        i< CAMIF_CHANNELS_NUM; i++, cstat++) {
                if (cstat->streaming_active || cstat->snapshot_active) {
                        chan_en |= 1 << i;
                }
        }
        PRP_CNTL |= chan_en;
	spin_unlock_irqrestore(&camif_lock, flags);
}


typedef struct _coeff_t {
        unsigned int    coeff[2];
        unsigned int    cntl;
}coeff_t[2][2];

static void
prp_set_scaler(int ch, int dir, scale_t *scale)
{
	int	i;
	unsigned int	coeff[2];
	unsigned int	valid;

        volatile coeff_t *prpregs = (void*)&(PRP2_RSZ1_HCOEF1);

	for (coeff[0] = coeff[1] = valid = 0, i = 19; i >= 0; i--)
	{
		int	j;

		j = i > 9 ? 1 : 0;
		coeff[j] = (coeff[j] << BC_COEF) | 
			(scale->tbl[i] & (SZ_COEF - 1));
		if (i == 5 || i == 15)
		{
			coeff[j] <<= 1;
		}
		valid = (valid << 1) | (scale->tbl[i] >> BC_COEF);
	}
	valid |= (scale->len << 24) | ((2 - scale->algo) << 31);
	for (i = 0; i < 2; i++)
	{
		(*prpregs)[ch][dir].coeff[i] = coeff[i];
	}
	(*prpregs)[ch][dir].cntl = valid;
}

static int
emma_prp_scale(unsigned int channel, int camwidth, int camheight, 
                unsigned short *prpwidth, unsigned short *prpheight)
{
        int ret;
	scale_t		scale[2];
        unsigned short res_width, res_height;
        
	memset(scale, 0, sizeof(scale));
	scale[0].algo = 0;
	scale[1].algo = 0;

	ret = prp_scale(&(scale[0]), 0, 0,
		camwidth, 
                prpwidth, 
                &res_width);
	if (ret)
	{
                err("Width rescaling error\n");
		return ret;
	}
        *prpwidth = res_width;
	ret = prp_scale(&(scale[1]), 0 , 1, 
		camheight, 
                prpheight, 
                &res_height);
	if (ret)
	{
                err("Height rescaling error\n");
		return ret;
	}
        *prpheight = res_height;


        prp_set_scaler(channel, 0, &(scale[0]));
        prp_set_scaler(channel, 1, &(scale[1]));
        return 0;
}

#define CSC_RGB_TO_YUV          (0)
#define CSC_YUV_TO_RGB          (1)

#define EQUATION_A1             (0)
#define EQUATION_A0             (1)
#define EQUATION_B1             (2)
#define EQUATION_B0             (3)

/* Lasr coeficient define Video range */
static const unsigned short
csc_convmatrix[2][4][10] = {
    {
        /* RGB to YUV conversion */
        {0x4D, 0x4B, 0x3A, 0x57, 0x55, 0x40, 0x40, 0x6B, 0x29, 0x01 },/* A1 */
        {0x42, 0x41, 0x32, 0x4C, 0x4A, 0x38, 0x38, 0x5E, 0x24, 0x00 },/* A0 */
        {0x36, 0x5C, 0x25, 0x3B, 0x63, 0x40, 0x40, 0x74, 0x18, 0x01 },/* B1 */
        {0x2F, 0x4F, 0x20, 0x34, 0x57, 0x38, 0x38, 0x66, 0x15, 0x00 },/* B0 */
    },
    {
         /* YUV to RGB */
         {0x80, 0xb4, 0x2c, 0x5b, 0x0e4, 0x00, 0x00, 0x00, 0x00, 0x01},/* A1 */
         {0x95, 0xcc, 0x32, 0x68, 0x104, 0x00, 0x00, 0x00, 0x00, 0x00},/* A0 */
         {0x80, 0xca, 0x18, 0x3c, 0x0ec, 0x00, 0x00, 0x00, 0x00, 0x01},/* B1 */
         {0x95, 0xe5, 0x1b, 0x44, 0x1e0, 0x00, 0x00, 0x00, 0x00, 0x00},/* B0 */
    }
};

/* Return pointer to appropriative coeficient set */
/* FIXME: Currently only YUV with full range (0-255) is used (A1 equation ). To
 * used YCrCb special color type or ioctl should be implemented */
const unsigned short * get_csc_coefs(void)
{
        enum pixel_format camfmt = this->camera->pixfmt;
        int conversion = 0;
        int equation = EQUATION_A1;

        switch (camfmt) {
                case YUV:
                        conversion = CSC_YUV_TO_RGB;
                        break;
                case RGB565:
                        conversion = CSC_RGB_TO_YUV;
                        break;
                default:
                        return NULL;
        }
        return csc_convmatrix[conversion][equation];
}

static int
emma_set_videoformat(void)
{
        unsigned short const *csc_coefs;

        csc_coefs = get_csc_coefs();
        if (!csc_coefs) {
                return -EINVAL;
        }

        PRP2_CSC_COEF0 = (csc_coefs[0] << 21) |
                (csc_coefs[1] << 11) | csc_coefs[2];
        PRP2_CSC_COEF1 = (csc_coefs[3] << 21) |
                (csc_coefs[4] << 11) | csc_coefs[5];
        PRP2_CSC_COEF2 = (csc_coefs[6] << 21) |
                (csc_coefs[7] << 11) | csc_coefs[8] |
                (csc_coefs[9] << 31);

        return 0;
}

static int
emma_set_format(unsigned int chan, struct v4l2_pix_format *fmt)
{
        unsigned int camwidth = 
                mx2ads_image_size[this->camera->imgfmt].width;
        unsigned int camheight=
                mx2ads_image_size[this->camera->imgfmt].height;

        unsigned short prpwidth = fmt->width;
        unsigned short prpheight = fmt->height;
        
        if (emma_prp_scale(chan, camwidth, camheight, &prpwidth, &prpheight)) {
                return -EINVAL;
        }

        if (chan == 0) {
                PRP_CH1_OUT_IMAGE_SIZE          = 
                        ((prpwidth    & 0x7FF)  << 16)        |
                        ((prpheight   & 0x7FF))               ;
                PRP_CH1_DEST_FRAME_CNTL = prpwidth * fmt->depth/8;
        } else {
                PRP_CH2_OUT_IMAGE_SIZE = 
                        ((prpwidth    & 0x7FF)  << 16)        |
                        ((prpheight   & 0x7FF))               ;
        }

        chan_stat[chan].width = prpwidth;
        chan_stat[chan].height = prpheight;
        chan_stat[chan].depth = fmt->depth;

        /* set videoformat */
        if (emma_set_videoformat()) {
                err("eMMa does not support current cumera format\n");
                return -EINVAL;
        }
        return 0;
}

/**************************************************************************
 * MX21 CSI module specific functions
 **************************************************************************/
/* Dma channel for data tranfer between CSI and memory buffer */
/* Pointer to the memory area for image capturing via DMA and size of memory area */

static int 
camif_set_format(unsigned int chan, struct v4l2_pix_format* fmt)
{
        unsigned int prppixfmtreg;
        unsigned int prp_cntl = 0, prp_cntl_mask = 0xFFFFFFFF, reg;
        
        int ret;
        
	unsigned long flags;
        canif_chan_stat_t *cstat = &(chan_stat[chan]);
        
        if (cstat->snapshot_active || cstat->streaming_active) {
                return -EINVAL;
        }
        
	spin_lock_irqsave(&camif_lock, flags);

        if (chan_stat[1-chan].opened) {
                
                unsigned int camwidth = 
                        mx2ads_image_size[this->camera->imgfmt].width;
                unsigned int camheight=
                        mx2ads_image_size[this->camera->imgfmt].height;

                if ((fmt->width > camwidth) || (fmt->height > camheight)) {
                        err("Dynamic change resolution currently is not "
                             "supported\n Please open channel with greatest "
                             "resolution first\n");
                        spin_unlock_irqrestore(&camif_lock, flags);
                        return -EINVAL;
                }
        } else { /* No active channel */
                if(this->camera) {
                        struct v4l2_pix_format camfmt = *fmt;
                        camfmt.pixelformat = V4L2_PIX_FMT_RGB565;
                        /* Trying to change format on camera */
                        /*FIXME: It seems the prp cannot accept image in 176x144
                         * format so trying to workarond this: Setup 320x240
                         * resolution and use prp scaling to get requested
                         * resolution */
                        unsigned int res = fmt->width * fmt->height;
                        if ((res > (160 * 120)) && (res <= (176 * 144))) {
                                camfmt.width = 320;
                                camfmt.height = 240;
                        }
                        if (this->camera->set_format(&camfmt)) {
                                /* FIXME: In this case we can try to set pixel
                                 * format of same kind, but which is uspported
                                 * by camera (e.g. if RGB 555 is not supported
                                 * but RGB565 is supported try to change to
                                 * RGB565) which is supported. After the eMMa
                                 * hardware will be initialized, the conversion
                                 * will be done by PRP module */

                                 /* Currently the feature described above does
                                  * not supported */
                                err("Unable to set request format\n");
                        	spin_unlock_irqrestore(&camif_lock, flags);
                                return -EINVAL;
                        }
                        
                        prp_cntl_mask &= ~(PRP2_CNTL_IN_RGB32);

                        prppixfmtreg = 0x2CA00565;
                        PRP_SOURCE_FRAME_FORMAT_CNTL    = prppixfmtreg;
                        prp_cntl |= PRP2_CNTL_IN_RGB;
        

                        PRP_SOURCE_FRAME_SIZE = 
                             ((camfmt.width & 0x7FF) << 16) | 
                             (camfmt.height & 0x7FF);
                }
        }
       
        
        if (chan == 0) {
                prp_cntl_mask &= ~(PRP2_CNTL_CH1_YUV422);
                switch (fmt->pixelformat) {
                        case V4L2_PIX_FMT_YUYV:
                                prppixfmtreg = 0x22000888;
                                prp_cntl |= PRP2_CNTL_CH1_YUV422;
                                break;
                        case V4L2_PIX_FMT_UYVY:
                                prppixfmtreg = 0x30800888;
                                prp_cntl |= PRP2_CNTL_CH1_YUV422; /* FIXME */
                                break;
                        case V4L2_PIX_FMT_RGB565:
                                prppixfmtreg = 0x2CA00565;
                                prp_cntl |= PRP2_CNTL_CH1_RGB16;
                                break;
                        case V4L2_PIX_FMT_RGB555:
                        case V4L2_PIX_FMT_RGB565X:
                        case V4L2_PIX_FMT_BGR24:
                        case V4L2_PIX_FMT_RGB555X:
                                err("This type of pixel format is "
                                    "not supported by EMMA PRP currently\n");
                        default:
                        	spin_unlock_irqrestore(&camif_lock, flags);
                                return -EINVAL;
                }
                PRP_DEST_FRAME_FORMAT_CNTL      = prppixfmtreg;
        } else {
                prp_cntl_mask &= ~(PRP2_CNTL_CH2_YUV422 | PRP2_CNTL_CH2_YUV444);
                switch (fmt->pixelformat) {
                        case V4L2_PIX_FMT_YUV420:
                        case V4L2_PIX_FMT_YVU420:
                                prp_cntl |= (PRP2_CNTL_CH2_YUV420);
                                break;
                        case V4L2_PIX_FMT_YUV422P:
                        case V4L2_PIX_FMT_YVU422P:
                                prp_cntl |= (PRP2_CNTL_CH2_YUV422);
                                break;
                                /* FIXME: Additionaly the eMMa hw can output in
                                 * YUV 4:4:4 but current v4l2 specification has
                                 * not definition fro this type */
                        default:
                                err("This type of pixel format is "
                                    "not supported by EMMA PRP currently\n");
                        	spin_unlock_irqrestore(&camif_lock, flags);
                                return -EINVAL;
                }
        }
        chan_stat[chan].pixfmt = fmt->pixelformat;
        reg = PRP_CNTL;
        PRP_CNTL &= prp_cntl_mask;
        PRP_CNTL |= prp_cntl;
        ret = emma_set_format(chan, fmt);
        
	spin_unlock_irqrestore(&camif_lock, flags);
        return ret;
}

static inline void 
convert_to_BGR24(u16* s, u8* d, int to_user)
{
	unsigned int r,g,b;
	u8 bgr[3];
	
	r = ((*s >> 11) & 0x1f) << 3;
	g = ((*s >>  5) & 0x3f) << 2;
	b = ((*s >>  0) & 0x1f) << 3;
	bgr[2] = b;
	bgr[1] = g;
	bgr[0] = r;
	
	if (to_user)
		copy_to_user(d, bgr, sizeof(bgr));
	else
		memcpy(d, bgr, sizeof(bgr));
}


static int 
camif_convert_image(int chan, u8* src, void* dest, int to_user, int dest_stride,
				struct v4l2_pix_format* fmt)
{
	int x, y;
	int line_gap;
        
        canif_chan_stat_t *cstat = &(chan_stat[chan]);
	int width = cstat->width;
	int height = cstat->height;

	int src_BPP = (cstat->depth+7) >> 3;
	int dest_BPP = (fmt->depth+7) >> 3;
	
	line_gap = dest_stride - (width * dest_BPP);
	
	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB555X:
                if (line_gap == 0) {
			if (to_user)
				copy_to_user(dest, src, width*height*src_BPP);
			else
				memcpy(dest, src, width*height*src_BPP);
		} else {
			for (y = 0; y < height; y++) {
				if (to_user)
					copy_to_user(dest, src, width*src_BPP);
				else
					memcpy(dest, src, width*src_BPP);
				src += width*src_BPP;
				dest += width*dest_BPP + line_gap;
                                
			}
		}
		break;
        case V4L2_PIX_FMT_YUV422P:
        case V4L2_PIX_FMT_YVU422P:	
        case V4L2_PIX_FMT_YUV420:
        case V4L2_PIX_FMT_YVU420:
                /* TODO: all planar formats above has differen location for
                 * different color planes with different sizes e.g. YUV 4:2:0:
                 * Y[width * height] U[width/2 * height/2] V[width/2*height/2].
                 * So the method to copy packed formats can not be used here.
                 * Additionaly we should support copy each own color plane to
                 * each own location request by application.
                 * Current implemtation suppose U color plane located
                 * immediately after Y color plane, and V locatead immedtely
                 * after U.
                 */
                if (to_user)
                        copy_to_user(dest, src, width*height*fmt->depth/8);
                else
                        memcpy(dest, src, width*height*fmt->depth/8);
                break;
	case V4L2_PIX_FMT_BGR24:
		/* Convert camera data from RGB565 to BGR24. */
		for (y = 0; y < height; y++) {
			for (x = 0 ; x < width ; x++) {
				convert_to_BGR24((u16*)src,
						 (u8*)dest, to_user);
				src += src_BPP;
				dest += dest_BPP;
			}
			dest += line_gap;
		}
		break;
	default:
		err("unsupported conversion request.\n");
		return -ENXIO;
	}

	return 0;
}


static inline void 
camif_start(void)
{
        uint32_t reg_val;

	CSI_REG_READ(CSICR1, reg_val);      /* Clear register */

        reg_val |= CSICR1_MCLKEN;           /* Master clock enable */
	CSI_REG_WRITE(CSICR1, reg_val);
}


static inline 
void camif_stop(void)
{
        uint32_t reg_val;

	CSI_REG_READ(CSICR1, reg_val);      /* Clear register */

        reg_val &= ~CSICR1_MCLKEN;           /* Master clock disable */
	CSI_REG_WRITE(CSICR1, reg_val);
}

static int
prepare_emma_out_buf(unsigned int chan, u8*buf)
{
        canif_chan_stat_t *cstat = &(chan_stat[chan]);
        if (chan == 0) {
                PRP_DEST_RGB1_PTR = virt_to_phys((void*)buf);
                
        } else { /* chan == 1 */
                unsigned int Ysize, Usize, Ybuf, Ubuf, Vbuf;

                Ysize = cstat->width * cstat->height;

                switch (chan_stat[chan].pixfmt) {
                        case V4L2_PIX_FMT_YUV420:
                        case V4L2_PIX_FMT_YVU420:
                                Usize = Ysize /4;
                                break;

                        case V4L2_PIX_FMT_YUV422P:
                        case V4L2_PIX_FMT_YVU422P:
                                Usize = Ysize /2;
                                break;
                        default:
                                err("Bad format recived\n");
                                return -EINVAL;
                }

                Ybuf = virt_to_phys((void*)buf);
                Ubuf = Ybuf + Ysize;
                Vbuf = Ubuf + Usize;
                
                PRP_DEST_Y_PTR                  = Ybuf;
                PRP_DEST_CB_PTR                 = Ubuf;
                PRP_DEST_CR_PTR                 = Vbuf;
                
        }

        return 0;
}


static int 
camif_snapshot(unsigned int chan, u8* buf, int size)
{
	unsigned long flags;
        int ret;
        canif_chan_stat_t *cstat = &(chan_stat[chan]);
        canif_chan_stat_t *ostat = &(chan_stat[1 - chan]); /* opposite 
                                                              channel */
                
        /*Sanity check */
        if ((chan >= CAMIF_CHANNELS_NUM) || (!buf)) {
                return -EINVAL;
        }

	spin_lock_irqsave(&camif_lock, flags);
        
	if (!this->camera) {
		spin_unlock_irqrestore(&camif_lock, flags);
		return -ENODEV;
	}
	if (cstat->snapshot_active) {
		dbg("already active!\n");
		spin_unlock_irqrestore(&camif_lock, flags);
		return 0;
	}
	if (cstat->streaming_active) {
		dbg("streaming is active!\n");
		spin_unlock_irqrestore(&camif_lock, flags);
		return -EINVAL;
	}

        if ((ret = prepare_emma_out_buf(chan, buf))) {
                spin_unlock_irqrestore(&camif_lock, flags);
                return ret;
        }
        
	cstat->snapshot_active = 1;
        
        emma_enable_interrupts(chan);

        /* If no background stream processing -- start the channel, otherwise
         * streaming will be start on next eMMa interrupt */
        if (!(ostat->streaming_active || ostat->snapshot_active)) {
                if (chan == 0) {
                        PRP_CNTL |= PRP2_CNTL_CH1EN;
                } else {
                        PRP_CNTL |= PRP2_CNTL_CH2EN;
                }
        }

	spin_unlock_irqrestore(&camif_lock, flags);

        return 0;
}


/* TODO: The best way to implemete streaming is use emma loop mode.*/
static int 
camif_start_streaming(unsigned int chan, u8* buf, int size)
{
	unsigned long flags;
        int ret;
        canif_chan_stat_t *cstat = &(chan_stat[chan]);
        canif_chan_stat_t *ostat = &(chan_stat[1 - chan]); /* opposite 
                                                              channel */

	spin_lock_irqsave(&camif_lock, flags);

	if (!this->camera) {
		spin_unlock_irqrestore(&camif_lock, flags);
		return -ENODEV;
	}
	if (cstat->streaming_active) {
		dbg("already active!\n");
		spin_unlock_irqrestore(&camif_lock, flags);
		return 0;
	}
	if (cstat->snapshot_active) {
		dbg("snapshot is active!\n");
		spin_unlock_irqrestore(&camif_lock, flags);
		return -EINVAL;
	}

        if ((ret = prepare_emma_out_buf(chan, buf))) {
                spin_unlock_irqrestore(&camif_lock, flags);
                return ret;
        }

	cstat->streaming_active = 1;

        emma_enable_interrupts(chan);

        /* If no background stream processing -- start the channel, otherwise
         * streaming will be start on next eMMa interrupt */
        if (!(ostat->streaming_active || ostat->snapshot_active)) {
                if (chan == 0) {
                         PRP_CNTL |= PRP2_CNTL_CH1EN;
                } else {
                         PRP_CNTL |= PRP2_CNTL_CH2EN;
                }
        }

	spin_unlock_irqrestore(&camif_lock, flags);
	return 0;
}

static int 
camif_abort(unsigned int chan)
{
	long my_timeout = HZ / 5;
        
        canif_chan_stat_t *cstat = &(chan_stat[chan]);

        if ((cstat->streaming_active || cstat->snapshot_active)) {
                cstat->abort = 1;

                /* Try sfly stop channe in interrupt handler */
                my_timeout = interruptible_sleep_on_timeout (
                                &cstat->abortqueue, my_timeout
                                );

                emma_disable_interrupts(chan);
                chan_stat[chan].snapshot_active =
                         chan_stat[chan].streaming_active = 0;


        }
        return 0;
}


static int camif_set_fp(int fp)
{
	int ret = 0;
        unsigned int exclk_mhz = camera_clk ;

	if (!this->camera)
		return -ENODEV;

	if (fp == this->camera->get_frame_period())
		return fp;
	
	/* first test this frame rate with camera */
	ret = this->camera->set_frame_period(fp, exclk_mhz, 1);

	if (ret >= 0) {
		/* frame rate passed tests, apply it.*/
		ret = this->camera->set_frame_period(fp,
						     exclk_mhz,
						     0);
	}
	return ret;
}

#define MOD(a,b) (((a) > (b))?((a)-(b)):((b)-(a)))

static unsigned int
camera_set_csi_clock(unsigned long cam_clock, unsigned int *hclk, unsigned int *mclk)
{
        unsigned long sysclk;
        unsigned int div1, div2;
        unsigned long mod, minmod;
        unsigned long ret_freq;
        sysclk = CLK_MPLL;
        *mclk = 32;
        *hclk = 64;
        minmod = mod = (*mclk) * (*hclk);
        /* This is very rough estimation , but it should works */
        for (div2 = 1; div2 <= 64; div2++ ) {
            for (div1 = 2; div1 <= 32; div1 +=2) {
                 mod = MOD(div1 * div2, sysclk/cam_clock);
                 if (mod < minmod) {
                     minmod = mod;
                     *mclk = div1;
                     *hclk = div2;
                     if (!mod)
                         break;
                 }
             }
        }
        ret_freq = CLK_MPLL / (*hclk) / (*mclk);
        *hclk = *hclk - 1;
        *mclk = (*mclk - 1) >> 1;
        return ret_freq;
}

static int
camif_open(int channel)
{
        int ret;
        unsigned int hclk, mclk;
        unsigned int reg_val;
        int i, initilized = 0;

        if (channel >= CAMIF_CHANNELS_NUM)
                return -EINVAL;
        
        if (!this->camera)
                return 0;

        for (i = 0; i < CAMIF_CHANNELS_NUM; i++) {
                if (chan_stat[i].opened)
                        initilized = 1;
        }
        
        chan_stat[channel].opened = 1;
        init_waitqueue_head(&chan_stat[channel].abortqueue);

        if (initilized)
                return 0;

#ifdef CONFIG_MX2TO1
	mx_module_clk_open(HCLK_MODULE_CSI);
#else
        camera_clk = camera_set_csi_clock(camera_clk, &hclk, &mclk);
        dbg("Set camera Master clock to %ld Hz\n", camera_clk);
	CRM_PCDR1 = (CRM_PCDR1 & ~PCDR1_PERDIV4) | 
            ((hclk << PCDR1_PERDIV4_SHIFT) & PCDR1_PERDIV4);
        
	mx_module_clk_open(HCLK_MODULE_CSI);
	mx_module_clk_open(IPG_MODULE_PERCLK4);
        mx_module_clk_open(HCLK_MODULE_EMMA);
        mx_module_clk_open(IPG_MODULE_EMMA);
        
	CSI_REG_READ(CSICR1, reg_val); /* Clear register */
        reg_val &= ~(CSICR1_MCLKDIV);
        reg_val |= (mclk << CSICR1_MCLKDIV_SHIFT) & CSICR1_MCLKDIV;
	CSI_REG_WRITE(CSICR1, reg_val); /* Clear register */
#endif
        /* Reset eMMa */
        PRP_CNTL = PRP2_CNTL_RST;
        mdelay(5);
        
        if (PRP_CNTL & PRP2_CNTL_RST)
        {
                err("prp: reset timeout\n");
                return  -EIO;
        }
        
        PRP_CNTL        = PRP2_CNTL_IN_RGB16 | PRP2_CNTL_CSI | PRP2_CNTL_UNCHAIN;
        PRP_INTRSTATUS  = 0x0;
       
        /* change irq to named constatnt*/
        if (request_irq(INT_EMMAPRP, emma_prp_isr, SA_INTERRUPT, 
                                "eMMA prp", NULL)) { 

                err("eMMa interrupt requesting error\n");
                camif_close(channel);
		return -1;
	}

        if ((ret = this->camera->open())) {
                err("Camera open error\n");
                camif_close(channel);
                return ret;
        }
        camif_start();

	return 0;
}

/*
 * Deallocate CSI-related resources
 */
static int
camif_close(int channel)
{
        int i;
        if (channel >= CAMIF_CHANNELS_NUM)
                return -EINVAL;
        
        camif_abort(channel);

        chan_stat[channel].opened = 0;

        for (i = 0; i < CAMIF_CHANNELS_NUM; i++) {
                if (chan_stat[i].opened) {
                        /* Other channel opened  we can't stop now */
                        return 0;
                }
        }

	free_irq(INT_EMMAPRP, NULL);
        
        camif_stop();

#ifdef CONFIG_MX2TO1
	mx_module_clk_close(HCLK_MODULE_CSI);
#else
	/* disable CSI PerClock 4 and HCLK */
	mx_module_clk_close(HCLK_MODULE_CSI);
	mx_module_clk_close(IPG_MODULE_PERCLK4);
        mx_module_clk_close(HCLK_MODULE_EMMA);
        mx_module_clk_close(IPG_MODULE_EMMA);
#endif
	return 0;
}

static int
camif_inithw(void)
{
	uint32_t reg_val = 0;
        unsigned int hclk = 0, mclk = 0;
        
	/*
         * Before access to CSI registers, CSI clock control should be
         * configured
	 */
#ifdef CONFIG_MX2TO1
	mx_module_clk_open(HCLK_MODULE_CSI);
        mclk = CSICR1_MCLKDIV_4 >> CSICR1_MCLKDIV_SHIFT;
#else
	/* adjust CSI PerClock 4 */
	/* enable CSI PerClock 4 and HCLK */
        camera_set_csi_clock(camera_clk, &hclk, &mclk);
	CRM_PCDR1 = (CRM_PCDR1 & ~PCDR1_PERDIV4) | 
            ((hclk << PCDR1_PERDIV4_SHIFT) & PCDR1_PERDIV4);
        mx_module_clk_open(HCLK_MODULE_CSI);
	mx_module_clk_open(IPG_MODULE_PERCLK4);
        mx_module_clk_open(HCLK_MODULE_EMMA);
        mx_module_clk_open(IPG_MODULE_EMMA);
#endif
        reg_val = 0;
        reg_val |= (mclk << CSICR1_MCLKDIV_SHIFT) & CSICR1_MCLKDIV;
        reg_val |= CSICR1_SOF_POL_RISE;
        reg_val |= CSICR1_SOF_INTEN;
        reg_val |= CSICR1_REDGE;
        reg_val |= CSICR1_GCLK_MODE;
        reg_val |= CSICR1_HSYNC_POL_HIGH;
        reg_val |= CSICR1_FCC_SCLR ;
        reg_val |= CSICR1_RXFF_LEVEL_16;
        reg_val |= CSICR1_SWAP16_EN;
        reg_val |= CSICR1_PRP_IFEN;

	CSI_REG_WRITE(CSICR1, reg_val);
        return 0;
}

static int
camif_ioctl(unsigned int cmd, void *arg)
{
        return -ENOIOCTLCMD;
}

/*
 * Cannel specific initialization
 */

static int
camif_chan_init(unsigned int chan, void (*callback)(void *), void* data)
{
        if (chan >= CAMIF_CHANNELS_NUM) 
                return -EINVAL;

        chan_stat[chan].capture_callback = callback;
        chan_stat[chan].callback_data = data;

        return 0;
}

#define CSI_REG_REQUESTED       (1)
#define PRP_REG_REQUESTED       (2)
/*
 * Initialise CSI for first use (set gpios, reserve memory)
 */
static int
camif_init(void)
{
        this = &camif_mx2ads;

        memset(&chan_stat, 0, sizeof(chan_stat));
	/*
	 * reserve virtual addresses for CSI registers physical address
	 */
	if (!request_mem_region(CSI_BASE, CSI_IO_SIZE, "MX21.CSI registers")) {
		err("MX2 Camera: CSI memory region is already in use\n");
		dbg("Address=0x%08x, size=0x%x\n", CSI_BASE, CSI_IO_SIZE);
		return -1;
	}
	mem_region_reserved |= CSI_REG_REQUESTED;
        
        if (!request_mem_region(EMMA_PRP_BASE, EMMA_PRP_IO_SIZE, 
                                "MX21.EMMA PRP registers")) {
                err("MX2 Camera: MX21.EMMA PRP memory region is already in "
                                "use\n");
                dbg("Address=0x%08x, size=0x%x\n", EMMA_PRP_BASE,
                                EMMA_PRP_IO_SIZE);
		return -1;
	}
	mem_region_reserved |= PRP_REG_REQUESTED;

	/*
	 * Enable CSI GPIOs
	 */
	if (mx2_register_gpios(PORT_B, CSI_GPIO_MASK, PRIMARY)) {
		    err("MX2 Camera: CSI GPIO pins are already in use\n");
		return -1;
	}

        if (camif_inithw()) {
            return -1;
        }


	camera_module_present = 1;

        
	return 0;
}


static void
camif_cleanup(void) 
{
	info("Unloading camera interface \n");
	if (this->camera)
		this->camera->cleanup();
	if (this->sbus) {
		this->sbus->cleanup();
	}
        
        if (mem_region_reserved & CSI_REG_REQUESTED) {
		release_mem_region(CSI_BASE, CSI_IO_SIZE);
        }

        if (mem_region_reserved & PRP_REG_REQUESTED) {
		release_mem_region(EMMA_PRP_BASE, EMMA_PRP_IO_SIZE);
        }
	mx2_unregister_gpios(PORT_B, CSI_GPIO_MASK);

#ifdef CONFIG_MX2TO1
	mx_module_clk_close(HCLK_MODULE_CSI);
#else
	/* disable CSI PerClock 4 and HCLK */
	mx_module_clk_close(HCLK_MODULE_CSI);
	mx_module_clk_close(IPG_MODULE_PERCLK4);
#endif
}

typedef struct cam_dev_tag {
        char *cam_name;
        struct camera* camera_if;
        struct camera_serial_bus * sbus;
        unsigned long camera_clk;
} cam_dev_t;


cam_dev_t camera_devs[] = {
#ifdef CONFIG_VIDEO_MX2ADS_OV9640
        {
                .cam_name =     "OV9640",
                .camera_if =    &camera_ov9640,
                .sbus =         &camera_mx2ads_ov9640_i2c,
                .camera_clk =   CAMERA_OV9640_WORK_FREQ,
        },
#endif /* CONFIG_VIDEO_MX2ADS_OV9640 */
#ifdef CONFIG_VIDEO_MX2ADS_MISOC0343
        {
                .cam_name =     "MI-SOC-0343",
                .camera_if =    &camera_misoc0343,
                .sbus =         &camera_mx2ads_misoc0343_i2c,
                .camera_clk =   CAMERA_MISOC0343_WORK_FREQ,
        },
#endif /* CONFIG_VIDEO_MX2ADS_MISOC0343 */
        {
                .cam_name =     NULL,
                .camera_if =    NULL,
                .sbus =         NULL,
                .camera_clk =   0,
        }
};

/****************************
 * Routine:  Description:
 ***************************/
static struct camera * 
camif_camera_detect(void)
{
        cam_dev_t *devices = camera_devs;
	
	ENTRY();

	this->camera = NULL;

	if (!camera_module_present)
		return NULL;
	
	camif_start();

        while (devices->camera_if) {
                int ret;
                char *name = devices->cam_name;
	        struct camera * cam = devices->camera_if;
                
                cam->camif = this;
                
                camera_clk = devices->camera_clk;
	        this->sbus = devices->sbus;

                devices++;

                info("Trying to detect %s camera\n", name);
                
                if ((ret = this->sbus->init())) {
                        info("%s camera not detected\n", name);
                        if (ret == -ENODEV)
                                continue;
                        else
                                break;
                }

                if (cam->detect() == 0) {
                        info("%s camera detected\n", name);
                        this->camera = cam;
                        this->camera->init();
                        break;
                }
                
                info("%s camera not detected\n", name);
                this->sbus->cleanup();
        }

        if (!this->camera) {
                info("No camera devices detected\n");
                /*TODO: the camif has ability ro read stream indirect from
                 * memeoty, So we can use this V4l2 module even if no camera
                 * modules present. In this cae the write primitive should be
                 * defined. 
                 */
        }
                
	camif_stop();
	return this->camera;
}


static void*
camif_alloc_image_buffer(int size)
{
	return (void*)__get_dma_pages(GFP_KERNEL, get_order(size));
}

static void
camif_free_image_buffer(void* buffer, int size)
{
	free_pages((unsigned long)buffer, get_order(size));
}

struct camera_interface camif_mx2ads = {
	.camera_detect          = camif_camera_detect,
	.alloc_image_buffer     = camif_alloc_image_buffer,
	.free_image_buffer      = camif_free_image_buffer,
	.init                   = camif_init,
        .init_chan              = camif_chan_init,
	.cleanup                = camif_cleanup,
	.open                   = camif_open,
	.close                  = camif_close,
	.snapshot               = camif_snapshot,
	.start_streaming        = camif_start_streaming,
	.abort                  = camif_abort,
	.set_frame_period       = camif_set_fp,
        .ioctl                  = camif_ioctl,
        .set_format             = camif_set_format,
        .convert_image          = camif_convert_image,
};

