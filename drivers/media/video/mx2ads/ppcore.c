/* drivers/media/video/mx2ads/ppcore.c
 *
 * MX21 PP module
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 *
 * Author: MontaVista Software, Inc.
 *              source@mvista.com
 *
 * This file is based on scale.c, pphw.c from Motorola Dragonball MX2 ADS BSP
 * Copyright 2002, 2003, 2004  Motorola, Inc. All Rights Reserved.
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/devfs_fs_kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>

#include <asm/hardware.h>
#include <asm/arch/pll.h>
#include <asm/arch/mx2-v4l2.h>

#include "common.h"
#include "pp.h"
#include "ppscale.h"

#define MODULE_NAME "eMMa PostProcessor"


static void *frame_buf = NULL;
static int frame_size = 0;

static int major;
static devfs_handle_t devfs_handle = NULL;
static int irq_requested = 0;
static spinlock_t pp_lock;
static int mem_region_reserved = 0;
static pp_dev_t pp_dev;
static pp_dev_t *this = NULL;
static wait_queue_head_t frame_complete;
static int frame_in_process = 0;
static char pp_busy;

static int pp_start(void);
static int
set_frame_parameters(struct v4l2_pix_format *in_pix,
                struct v4l2_pix_format *out_pix);
static void
set_in_pointers(unsigned int phys_addr);


static const unsigned short csc_tbl[][6]  = {
        {0x80, 0xb4, 0x2c, 0x5b, 0x0e4, 0},
        {0x95, 0xcc, 0x32, 0x68, 0x104, 1},
        {0x80, 0xca, 0x18, 0x3c, 0x0ec, 0},
        {0x95, 0xe5, 0x1b, 0x44, 0x10e, 1},
};

/*
 *
 *      P O W E R   M A N A G E M E N T
 *
 */
#if 1   /* power management */

static char pp_suspended;

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

static void
mx2ads_pp_suspend(void)
{
	if (pp_suspended) return;
	pp_suspended = 1;
	if (!pp_busy) return;

	disable_irq(INT_EMMAPP);

	if (!check_mem_region(EMMA_PRP_BASE, EMMA_PRP_IO_SIZE)) {
                /* If no PRP modules exists in same time stop clocks */
		mx_module_clk_close(HCLK_MODULE_EMMA);
		mx_module_clk_close(IPG_MODULE_EMMA);
        }

}

static void
mx2ads_pp_resume(void)
{
	if (!pp_suspended) return;
	pp_suspended = 0;
	if (!pp_busy) return;

	mx_module_clk_open(HCLK_MODULE_EMMA);
	mx_module_clk_open(IPG_MODULE_EMMA);
	EMMA_PP_CNTL |= PP_CNTL_SWRST;
	mdelay(5);

	if (this->Yptr) {
		set_in_pointers(this->Yptr);
	}
	set_frame_parameters(&(this->in_format.fmt.pix),
                                        &(this->out_format.fmt.pix));

	if (frame_in_process) {
		pp_start();
	}
	enable_irq(INT_EMMAPP);
}

#ifdef CONFIG_PM
static struct pm_dev *mx2ads_pp_pmdev;

static int
mx2ads_pp_pm_callback(struct pm_dev *pmdev, pm_request_t rqst, void *data)
{
	switch (rqst) {
	case PM_SUSPEND:
		mx2ads_pp_suspend();
		break;
	case PM_RESUME:
		mx2ads_pp_resume();
		break;
	}
	return 0;
}
#endif /* CONFIG_PM*/

#include <linux/device.h>
#include <linux/dpm.h>

#ifdef CONFIG_DPM
static struct constraints mx2ads_pp_constraints = {
	.count = 1,
	.param = {{DPM_MD_HCLK, 0, 0}},	/*to be initialized at module init time */
	.asserted = 1,
};
#endif


extern void mx21_ldm_bus_register(struct device *device,
                          struct device_driver *driver);
extern void mx21_ldm_bus_unregister(struct device *device,
                          struct device_driver *driver);

static int
mx2ads_pp_dpm_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		mx2ads_pp_suspend();
		break;
	}
	return 0;
}

static int
mx2ads_pp_dpm_resume(struct device *dev, u32 level)
{
	switch (level) {
	case RESUME_POWER_ON:
		mx2ads_pp_resume();
		break;
	}
	return 0;
}

static struct device_driver mx2ads_pp_driver_ldm = {
	.name       = "mx2ads-pp",
	.suspend    = mx2ads_pp_dpm_suspend,
	.resume     = mx2ads_pp_dpm_resume,
};

static struct device mx2ads_pp_device_ldm = {
	.name           = "EMMA Post Processing",
	.bus_id         = "mx2ads-pp",
	.driver         = &mx2ads_pp_driver_ldm,
	.power_state    = DPM_POWER_ON,
#ifdef CONFIG_DPM
	.constraints = &mx2ads_pp_constraints,
#endif
};

#endif   /* power management */


static void
set_in_pointers(unsigned int phys_addr)
{
        unsigned int Yptr, Uptr, Vptr;
        struct v4l2_pix_format  *in_fmt = &(this->in_format.fmt.pix); 
        unsigned int reg;
        
        this->in_stride = in_fmt->width;
        reg = EMMA_PP_SFRM_WIDTH & 0x00FF0000; /* Store quantizer parameter */
        reg |= (this->in_stride);
        EMMA_PP_SFRM_WIDTH = reg;

        Yptr = phys_addr;
        Uptr = Yptr + in_fmt->height * this->in_stride;
        Vptr = Uptr + in_fmt->height/2 * this->in_stride/2;
        
        EMMA_PP_SY_PTR =        Yptr; 
        EMMA_PP_SCB_PTR =       Uptr;
        EMMA_PP_SCR_PTR =       Vptr;
}

static int
set_frame_parameters(struct v4l2_pix_format *in_pix, 
                struct v4l2_pix_format *out_pix)
{
        int inw, inh, outw, outh;

        /* Sanity check */
        if ((!in_pix) || (!out_pix)) {
                return -EINVAL;
        }
       
        inw  = in_pix->width    ? in_pix->width  : out_pix->width ;
        inh  = in_pix->height   ? in_pix->height : out_pix->height;
        outw = out_pix->width   ? out_pix->width : in_pix->width  ;
        outh = out_pix->height  ? out_pix->height: in_pix->height ;

        if ((!inw) || (!inh)) {
                /* Not configured yet */
                return -EINVAL;
        }

        if (pp_rescale(inw, inh, &outw, &outh)) {
                return -EINVAL;
        }

        EMMA_PP_PROC_PARA = 
                (inw << 16)     |
                (inh)           ;

        
        if (!this->out_stride) {
                EMMA_PP_DDIS_WIDTH      = outw * out_pix->depth/8;
        } else {
                EMMA_PP_DDIS_WIDTH      = this->out_stride * out_pix->depth/8;
        }

        EMMA_PP_DIMAGE_SIZE=
                (outw << 16)    |
                (outh)          ;
        return 0;
}


static void
pp_isr(int irq_no, void *dev_id, struct pt_regs *regs)
{
	unsigned long flags;
	spin_lock_irqsave(&pp_lock, flags);
        unsigned int stat = EMMA_PP_INTRSTATUS;

        EMMA_PP_INTRSTATUS = stat;
        
        if (!(stat & PP_INTRSTATUS_FRAME_INTR)) {
                err("Error ocured druing frame processing:%x\n", stat);
        }
        
        EMMA_PP_INTRCTRL &= ~(PP_INTRCNTL_FRAMECOMPINTREN | 
                        PP_INTRCNTL_ERRINTR_EN);

        /* Restore user bufer pointer in case if write was used */
        if (this->Yptr) {
                set_in_pointers(this->Yptr);
        }

        wake_up(&frame_complete);
        frame_in_process = 0;

	spin_unlock_irqrestore(&pp_lock, flags);
        return;
}

static int
pp_start(void)
{
        unsigned int cntl = 0;
        int csc_conv = 0;
        
        EMMA_PP_DRGB_PTR        = (unsigned int)this->outptr;

        switch (this->out_format.fmt.pix.pixelformat) {
                case V4L2_PIX_FMT_RGB565:
                        csc_conv = 1;
                        cntl |= PP_CNTL_CSC_OUT_RGB565;
                        EMMA_PP_DPIX_FMT = 0x2CA00565;
                        break;
                case V4L2_PIX_FMT_YUYV:
                        EMMA_PP_DPIX_FMT = 0x62000888;
                        csc_conv = 0;
                        /*XXX Add adtitinal YUV data types from book */
                        break;
                default:
                        err("Bad pixel format\n");
                        return -EINVAL;
        }
        
        if (csc_conv) {
                cntl |= PP_CNTL_CSCEN;
                cntl |= PP_CNTL_CSC_TABLE_SEL_B1; /* XXX */
                
                /* Loading CSC table */
		EMMA_PP_CSC_COEF_123 = 
                        (csc_tbl[0][0] << 24)    | 
			(csc_tbl[0][1] << 16)    | 
			(csc_tbl[0][2] << 8)     | 
			 csc_tbl[0][3]           ;
		EMMA_PP_CSC_COEF_4 = 
                        (csc_tbl[0][5] << 9) | 
			 csc_tbl[0][4];
        }
        
        EMMA_PP_CNTL = cntl;
        EMMA_PP_INTRCTRL = PP_INTRCNTL_FRAMECOMPINTREN | PP_INTRCNTL_ERRINTR_EN;

        frame_in_process = 1;
        EMMA_PP_CNTL |= PP_CNTL_EN;
        return 0;
}

static unsigned int
pp_poll(struct file *file, poll_table *table)
{
	if (pp_suspended)
		return -EPERM;

        if (!frame_in_process)
                return (POLLIN | POLLRDNORM);
        
	poll_wait (file, &frame_complete, table);
	return 0;
}

static int	
pp_open(struct inode *inode, struct file *file)
{
	if (pp_suspended)
		return -EPERM;
	if (pp_busy)
		return -EBUSY;
	pp_busy = 1;
        mx_module_clk_open(HCLK_MODULE_EMMA);
        mx_module_clk_open(IPG_MODULE_EMMA);
        EMMA_PP_CNTL |= PP_CNTL_SWRST;
        mdelay(5);
        if (EMMA_PP_CNTL & PP_CNTL_SWRST) {
                err("PP Reset error\n");
                return -EIO;
        }
        MOD_INC_USE_COUNT;

	enable_irq(INT_EMMAPP);
        return 0;
}


static int
pp_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
                unsigned long arg)
{
	if (pp_suspended)
		return -EPERM;
        if (frame_in_process)
                return -EAGAIN;

        switch (cmd) {
                /* Set ouput format*/
	        case VIDIOC_S_FMT: 
                {
                        struct v4l2_format *fmt = (void*)arg;
                        struct v4l2_pix_format *pfmt = &(fmt->fmt.pix);
                        if (!fmt) {
                                return -EINVAL;
                        }
                        switch (pfmt->pixelformat) {
                                case V4L2_PIX_FMT_YUYV:
                                case V4L2_PIX_FMT_RGB565:
                                        /*TODO additioanl format can be support
                                         * (RGB555, RGB 888)*/
                                        pfmt->depth = 16;
                                        break;
                                default:
                        /*XXX Add adtitinal YUVdata types from book */
                                        return -EINVAL;
                        }
                        this->out_format = *fmt;
                        return set_frame_parameters(&(this->in_format.fmt.pix), 
                                        &(this->out_format.fmt.pix));
                }
                /* Get ouput format */
	        case VIDIOC_G_FMT:
                {
                        struct v4l2_format *fmt = (void*)arg;
                        if (!fmt) {
                                return -EINVAL;
                        }
                        *fmt = this->out_format;
                        return 0;
                /* Set input format*/
                }
	        case VIDIOC_S_IN_FMT:
                {
                        struct v4l2_format *fmt = (void*)arg;
                        struct v4l2_pix_format *pfmt = &(fmt->fmt.pix);
                        int oldfrmsize, frmsize;
                        if (!fmt) {
                                return -EINVAL;
                        }
                        if (pfmt->pixelformat != V4L2_PIX_FMT_YUV420) {
                                return -EINVAL;
                        }
                        frmsize = pfmt->width * pfmt->height * 2;
                        oldfrmsize = this->in_format.fmt.pix.width * 
                                     this->in_format.fmt.pix.height * 2;
                        if (frmsize > oldfrmsize) {
                                if (frame_buf) {
                                        free_pages( (unsigned long)frame_buf, 
                                                        get_order(oldfrmsize));

                                }
                                frame_buf = (void*)__get_dma_pages(GFP_KERNEL, 
                                                get_order(frmsize));
                                frame_size = frmsize;
                                if (!frame_buf) {
                                        return -ENOMEM;
                                }
                        }
                        this->in_format = *fmt;
                        return set_frame_parameters(&(this->in_format.fmt.pix), 
                                        &(this->out_format.fmt.pix));
                }
                /* Get input format */
	        case VIDIOC_G_IN_FMT:
                {
                        struct v4l2_format *fmt = (void*)arg;
                        if (!fmt) {
                                return -EINVAL;
                        }
                        *fmt = this->in_format;
                        if (this->Yptr) {
                                set_in_pointers(this->Yptr);
                        }
                        return 0;
                }
                case VIDIOC_S_IN_PTR:
                {
                        if (!arg)
                                return -EINVAL;
                        this->Yptr = (unsigned int)arg;
                        set_in_pointers(this->Yptr);
                        return 0;
                }
                case VIDIOC_S_OUT_PTR:
                {
                        if (!arg)
                                return -EINVAL;
                        this->outptr = (void *)arg;
                        return 0;
                }
                case VIDIOC_START:
                {
                        return pp_start();
                }
                case VIDIOC_POLL:
                {
                        return frame_in_process;
                }
                case VIDIOC_S_OUT_STRIDE:
                {
                        if (!((int)arg)) {
                                return -EINVAL;
                        }
                        this->out_stride = (int)arg;
                        return 0;
                }
                default:
                {
                        return -ENOIOCTLCMD;
                }
        }
        return -ENOIOCTLCMD;
}

static ssize_t
pp_write (struct file *file, const char *buf, size_t count, loff_t *ppos)
{
        int ret;
	if (pp_suspended)
		return -EPERM;

        if (frame_in_process)
                return -EAGAIN;
                
        copy_from_user(frame_buf, buf, count);
        
        set_in_pointers(virt_to_phys(frame_buf));
        
	if ((ret = pp_start())) {
                return ret;
        }
        return count;
}

static int
pp_fasync(int fd, struct file *filp, int mode)
{
        return 0;
}

static int
pp_close(struct inode *inode, struct file *file)
{
	disable_irq(INT_EMMAPP);

        if (!check_mem_region(EMMA_PRP_BASE, EMMA_PRP_IO_SIZE)) {
                /* If no PRP modules exists in same time stop clocks */
                mx_module_clk_close(HCLK_MODULE_EMMA);
                mx_module_clk_close(IPG_MODULE_EMMA);
        }

        MOD_DEC_USE_COUNT;
	pp_busy = 0;
        return 0;
}

static struct 
file_operations	pp_fops = 
{
	open:		pp_open,
        write:          pp_write,
	ioctl:		pp_ioctl,
	release: 	pp_close,
	fasync:		pp_fasync,
        poll:           pp_poll,
};


static void __init
pp_exit(void)
{
        if (irq_requested) {
                disable_irq(INT_EMMAPP);
	        free_irq(INT_EMMAPP, NULL);
                irq_requested = 0;
#if 1   /* power management */
		mx21_ldm_bus_unregister( &mx2ads_pp_device_ldm,
        	        &mx2ads_pp_driver_ldm);

#ifdef CONFIG_PM
		if (mx2ads_pp_pmdev)
			pm_unregister(mx2ads_pp_pmdev);
#endif
#endif /* power management */
        }

        if (frame_buf) {
                free_pages( (unsigned long)frame_buf, 
                                get_order(frame_size));

        }

   	if(major > 0) {
		if(devfs_unregister_chrdev(major, "emma_pp") < 0) {
			err("failed to unregister from devfs\n");
			return;
		}
	}
	if(devfs_handle != NULL) {
		devfs_unregister(devfs_handle);
	} else {
                err("failed to unregister from devfs, devfs_handle = 0x%p\n", 
                                devfs_handle);
		return;
	}

        if (mem_region_reserved) {
		release_mem_region(EMMA_PP_BASE, EMMA_PP_IO_SIZE);
        }

}

#define PP_MIN_HCLK  33000

static int __init
pp_init(void)
{
#if 1 /*CEE LDM*/
	if (PP_MIN_HCLK > mx_module_get_clk(HCLK) / 1000) {
		err("cannot initialize - HCLK too slow\n");
		return -EPERM;
	}
#endif

        this = &pp_dev; 
        memset(&pp_dev, 0, sizeof(pp_dev));
        
 	major = devfs_register_chrdev(0, "emma_pp", &pp_fops);
 	if ( major < 0 ) {
		err("unable to register character device\n");
                pp_exit();
		return -ENODEV;
	}
	devfs_handle = devfs_register(NULL, "emma_pp", DEVFS_FL_DEFAULT,
				      major, 0,
				      S_IFCHR | S_IRUSR | S_IWUSR,
				      &pp_fops, NULL);
	if(devfs_handle == NULL) {
		err("unable to register devfs driver\n");
                pp_exit();
		return -ENODEV;
	}
        if (!request_mem_region(EMMA_PP_BASE, EMMA_PP_IO_SIZE, 
                                "MX21.EMMA PP registers")) {
                err("EMMA PP memory region is already in use\n");
                dbg("Address=0x%08x, size=0x%x\n", EMMA_PP_BASE,
                                EMMA_PP_IO_SIZE);
                pp_exit();
		return -1;
	}

        mem_region_reserved = 1;

	if (request_irq(INT_EMMAPP, pp_isr, SA_INTERRUPT, "eMMa PP", NULL)) {
                err("Error requesting IRQ\n");
                pp_exit();
		return -ENODEV;
        }
        irq_requested = 1;

	init_waitqueue_head (&frame_complete);

#if 1   /* power management */

#ifdef CONFIG_DPM
	mx2ads_pp_constraints.param[0].min = PP_MIN_HCLK;	/*in KHz */
	mx2ads_pp_constraints.param[0].max = mx_module_get_clk(MPLL) / 1000;	/* unlimited */
#endif

        mx21_ldm_bus_register(&mx2ads_pp_device_ldm,
                &mx2ads_pp_driver_ldm);

#ifdef CONFIG_PM
	mx2ads_pp_pmdev = pm_register(PM_UNKNOWN_DEV, PM_SYS_UNKNOWN,
					 mx2ads_pp_pm_callback);
	if (!mx2ads_pp_pmdev)
		warn("failed to initialize static power management\n");
#endif /* CONFIG_PM */
#endif /* power management */

        return 0;
}

module_init(pp_init);
module_exit(pp_exit);
        
MODULE_LICENSE ("GPL");

