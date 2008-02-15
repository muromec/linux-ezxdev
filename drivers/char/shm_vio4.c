/*
 * drivers/char/shm_vio4.c
 *
 * SH73180 VIO4 dirver for Super-H
 *
 * Auther: Kenichi KITAZAWA
 *
 * 2004(c) Kenichi KITAZAWA. This file licensed under
 * the terms of the GNU General Public License version 2. THis program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <asm/io.h>
#include <asm/irq-sh73180.h>
#include <asm/cpu-regs.h>
#include <asm/hitachi_shmse.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#ifdef CONFIG_PM
#include <asm/sh_mobile3_pm.h>
#endif
#include <linux/devfs_fs_kernel.h>
#include "shm_vio4.h"

#ifdef DEBUG
#define DPRINTK(fmt,args...) printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt,argc...)
#endif

#ifdef DEBUG
static int beu_int_hdr_cnt;
static int ceu_int_hdr_cnt;
static int veu_int_hdr_cnt;
#endif
#ifdef CONFIG_DPM
static void vio4_ldm_register(void);
static void vio4_ldm_unregister(void);
#endif

/*
 * Device Major Number
 */
static int vio4_major = VIO4_DEV_MAJOR;

#define Y_BUFF		((img_buff.viomem_base + img_buff.y_buff_os) & 0x1fffffff)
#define C_BUFF		((img_buff.viomem_base + img_buff.c_buff_os) & 0x1fffffff)
#define RGB_BUFF	((img_buff.viomem_base + img_buff.rgb_buff_os) & 0x1fffffff)
#define RGB_C_BUFF	((img_buff.viomem_base + img_buff.rgb_c_buff_os) & 0x1fffffff)
#define DISP_BUFF	((img_buff.viomem_base + img_buff.disp_buff_os) & 0x1fffffff)
#define OSD_BUFF	((img_buff.viomem_base + img_buff.osd_buff_os) & 0x1fffffff)

/*
 * filter tap
 */
const long fem_vft[] = { 0x00200121, 0x00400565 };
const long fem_hft[] = { 0x00200121, 0x00400565 };

/*
 * wait queue for processes wanting to open
 */
static wait_queue_head_t c_wq;
static wait_queue_head_t v_wq;
static wait_queue_head_t f_wq;
static wait_queue_head_t b_wq;

int veu_event = 0;		/* veu(yuv -> rgb) event flag */
int feu_event = 0;
int beu_event = 0;

const struct vio4_img_buff default_img_buff = {
	0xade00000,
	0x00040000,
	2,
	0x00000000,		/* y_buff_os */
	0x00080000,		/* c_buff_os */
	0x00100000,		/* rgb_buff_os */
	0x00000000,		/* rgb_c_buff_os */
	0x00180000,		/* disp_buff_os */
	0x00000000,
	0x00040000 * 2 * 5,
};

const struct vio4_capture default_cam = {
	176,			/* hsize:img_width */
	144,			/* vsize:img_height */
	((640 - 176) / 2),	/* hoffset:w_offset */
	((480 - 144) / 2),	/* voffset:v_offset */
	0,			/* VD Hight Active */
	0,			/* HD Hight Active */
	0,			/* image capture */
	CAPRESIZE_1_0,		/* down sample filter */
	CAPRESIZE_1_0,
	176,			/* clip image size */
	144,
	0,
	0,                      /* state:stopped */
};

const struct vio4_video default_video = {
	176,			/* width */
	144,			/* height */
	176,			/* clip width */
	144,			/* clip height */
	VEURESIZE_VER_1_0,	/* vertical */
	VEURESIZE_HOR_1_0,	/* horizon */
	16,			/* bit per pixel */
	0,			/* rgb pad */
	0,			/* chds */
	1,			/* chrr */
	6,			/* wpkf */
	0,			/* rpkf */
	0,			/* dith */
	2,			/* tery */
	0,                      /* state:stopped */
};

struct vio4_capture config_cam;
struct vio4_video config_video;
struct vio4_img_buff img_buff;

/*********************************************************************
 * CEU interrupt handler
 *********************************************************************/
static void
ceu_int_hdr(int irq, void *dev_id, struct pt_regs *regs)
{
	long fact;

	fact = ctrl_inl(VIO_CETCR);	/* read interrupt factor */

	DPRINTK("VIO4 ceu interrupt : %08x\n", (unsigned int) fact);

	if (fact & VIO_CETCR_NVD) {
		ctrl_outl(fact & ~VIO_CETCR_NVD, VIO_CETCR);
		DPRINTK("VIO_CETCR_NVD\n");
	}
	if (fact & VIO_CETCR_NHD) {
		ctrl_outl(fact & ~VIO_CETCR_NHD, VIO_CETCR);
		DPRINTK("VIO_CETCR_NHD\n");
	}
	if (fact & VIO_CETCR_VBP) {
		ctrl_outl(fact & ~VIO_CETCR_VBP, VIO_CETCR);
		DPRINTK("VIO_CETCR_VBP\n");
	}
	if (fact & VIO_CETCR_IGVS) {
		ctrl_outl(fact & ~VIO_CETCR_IGVS, VIO_CETCR);
		DPRINTK("VIO_CETCR_IGVS\n");
	}
	if (fact & VIO_CETCR_IGHS) {
		ctrl_outl(fact & ~VIO_CETCR_IGHS, VIO_CETCR);
		DPRINTK("VIO_CETCR_IGHS\n");
	}
	if (fact & VIO_CETCR_CDTOR) {
		ctrl_outl(fact & ~VIO_CETCR_CDTOR, VIO_CETCR);
		DPRINTK("VIO_CETCR_CDTOR\n");
	}
	if (fact & VIO_CETCR_VD) {
		ctrl_outl(fact & ~VIO_CETCR_VD, VIO_CETCR);
	}
	if (fact & VIO_CETCR_HD) {
		ctrl_outl(fact & ~VIO_CETCR_HD, VIO_CETCR);
	}
	if (fact & VIO_CETCR_IGRW) {
		ctrl_outl(fact & ~VIO_CETCR_IGRW, VIO_CETCR);
		DPRINTK("VIO_CETCR_IGRW\n");
	}
	if (fact & VIO_CETCR_CPE) {
		ctrl_outl(fact & ~VIO_CETCR_CPE, VIO_CETCR);
		wake_up_interruptible(&c_wq);	/* wake up */
		DPRINTK("VIO_CETCR_CPE\n");
	}

}

/*********************************************************************
 * VEU interrupt handler
 *********************************************************************/
#ifdef DDEBUG
static unsigned long veu_int_hdr_cnt;
#endif
static void
veu_int_hdr(int irq, void *dev_id, struct pt_regs *regs)
{
	long fact;

	fact = ctrl_inl(VIO_VEVTR);	/* read interrupt factor */

	DPRINTK("VIO4 veu interrupt : %08x\n", (unsigned int) fact);

	if (fact & VIO_VEVTR_FEEND) {
		ctrl_outl(fact & ~VIO_VEVTR_FEEND, VIO_VEVTR);
		ctrl_outl(0x00000000, VIO_FESTR);
		feu_event = 1;	/* event flag set */
		wake_up(&f_wq);	/* wake up */
	}
	if (fact & VIO_VEVTR_VEEND) {
		ctrl_outl(fact & ~VIO_VEVTR_VEEND, VIO_VEVTR);
		ctrl_outl(0x00000000, VIO_VESTR);
#ifdef DDEBUG
		veu_int_hdr_cnt++;
#endif
		veu_event = 1;	/* event flag set */
		wake_up(&v_wq);	/* wake up */
	}

}

/*********************************************************************
 * BEU interrupt handler
 *********************************************************************/
static void
beu_int_hdr(int irq, void *dev_id, struct pt_regs *regs)
{
	long fact;

	fact = ctrl_inl(VIO_BEVTR);	/* read interrupt factor */

	DPRINTK("VIO4 interrupt : %08x\n", (unsigned int) fact);
	if (fact & VIO_BEVTR_INTREQ) {
		ctrl_outl(fact & ~VIO_BEVTR_INTREQ, VIO_BEVTR);
		DPRINTK("VIO_BEVTR_INTREQ\n");
	}
	if (fact & VIO_BEVTR_BEVIO) {
		ctrl_outl(fact & ~VIO_BEVTR_BEVIO, VIO_BEVTR);
		DPRINTK("VIO_BEVTR_BEVIO\n");
	}
	if (fact & VIO_BEVTR_BEEND) {
		ctrl_outl(fact & ~VIO_BEVTR_BEEND, VIO_BEVTR);
		DPRINTK("VIO_BEVTR_BEEND\n");
	}
	ctrl_outl(0x00000000, VIO_BESTR);
	beu_event = 1;		/* event flag set */
	wake_up(&b_wq);		/* wake up */
#ifdef DEBUG
	beu_int_hdr_cnt++;
#endif

}

/*********************************************************************
 * VIO control function
 *********************************************************************/
static void
init_hw_vio4(void)
{
	/* VIO clock start */
	ctrl_outl(ctrl_inl(MSTPCR2) & ~(MSTP202|MSTP204) , MSTPCR2);   

	ctrl_outb(ctrl_inb(INTC_IMR1) | 0x70, INTC_IMCR1);

	/* configration Port PIN */
	/*
	   PACR:VIO Data port
	   PECR:VIO HD/VD/CKO
	   PFCR:VIO STEM/STEX
	   PJCR:VIO CLK
	 */
	ctrl_outw(0x0000, PORT_PACR);
	ctrl_outw(ctrl_inw(PORT_PECR) & 0x03ff, PORT_PECR);
	ctrl_outw(ctrl_inw(PORT_PFCR) & 0x0fff, PORT_PFCR);
	ctrl_outw(ctrl_inw(PORT_PJCR) & 0xfcff, PORT_PJCR);

	/* initialize PFC register */
	ctrl_outw(ctrl_inw(PORT_HIZCRA) & 0xcfff, PORT_HIZCRA);
	ctrl_outw(ctrl_inw(PORT_DRVCR) & 0x000c, PORT_DRVCR);

	ctrl_outb(ctrl_inb(STBCR6) | 0xf0, STBCR6);
	
	/* initialize VIO register */
	ctrl_outl(0x00000000, VIO_VEVTR);
}

/*
  capture function
*/
static void
config_capture_addr(int i)
{
	/* Y,Cr,Cb Data Address */
	ctrl_outl(Y_BUFF + i * img_buff.vmem_size, VIO_CDAYR);
	ctrl_outl(C_BUFF + i * img_buff.vmem_size, VIO_CDACR);
}

/*
  Configure Capture Register
*/
static void
config_capture(void)
{
	unsigned long tmpl;

	/*
	   CAPCR
	   - FDRP:0
	   - MTCM:10
	   - CTNCP:0
	   - LPF:0
	   - PIXDL:0
	   - CDS:1 (YCbCr=4:2:2)
	 */
	tmpl = config_cam.capcr_cds & 0x00000001;

	ctrl_outl(tmpl, VIO_CAPCR);

	/*
	   capture filter control(resize)       
	 */
	tmpl = config_cam.vds_coef;
	tmpl <<= 16;
	tmpl |= config_cam.hds_coef;
	ctrl_outl(tmpl, VIO_CFLCR);

	/*
	   CMCYR: camera interface cycle
	   no detect illegal VD and HD.
	 */
	ctrl_outl(0x00000000, VIO_CMCYR);

	/* capture output iamge width */
	ctrl_outl(((config_cam.clip_width) & 0x0fff), VIO_CDWDR);

	/* capture filter clip */
	tmpl = config_cam.clip_height & 0x0ffc;
	tmpl <<= 16;
	tmpl |= config_cam.clip_width & 0x0ffc;
	ctrl_outl(tmpl, VIO_CFSZR);

	/* output address incriment */
	ctrl_outl(0x00000000, VIO_CFXAR);

	/* data swap */
	ctrl_outl(0x00000003, VIO_CDSWR);	/* word swap and byte swap */

	/* camera interface control */
	tmpl = 0x00000000;
	tmpl |= (config_cam.img_capture & 0x1) << 4;
	tmpl |= (config_cam.vdpol & 0x1) << 1;
	tmpl |= (config_cam.hdpol & 0x1);
	ctrl_outl(tmpl, VIO_CAMCR);

	/*
	   wait 10 cycle camera clock
	   Hsync 1 cycle
	 */
	ctrl_outl(0x00000000, VIO_CETCR);
	while ((ctrl_inl(VIO_CETCR) & VIO_CETCR_HD) == 0) ;
	ctrl_outl(0x00000000, VIO_CETCR);
	while ((ctrl_inl(VIO_CETCR) & VIO_CETCR_HD) == 0) ;

	/*
	   camera interface
	   horizontal offset and vertical offset
	 */
	tmpl = 0;
	tmpl = config_cam.v_offset & 0x0fff;
	tmpl <<= 16;
	tmpl |= (config_cam.h_offset << 1) & 0x0fff;	/* pixcel count -> cycle */
	ctrl_outl(tmpl, VIO_CAMOR);

	/*
	   camera interface
	   capture image width and height
	 */
	tmpl = config_cam.img_height & 0x0ffc;
	tmpl <<= 16;
	tmpl |= (config_cam.img_width << 1) & 0x01ffc;
	ctrl_outl(tmpl, VIO_CAPWR);

	config_capture_addr(0);
}

/*
  Capture start
*/
static void
start_capture(void)
{
	ctrl_outl(0x03170011, VIO_CEIER);	/* Interrupt Enable without HD */
	ctrl_outl(0x00000001, VIO_CAPSR);	/* Capture Start */

	config_cam.running = 1;

	DPRINTK("VIO Capture start reqest\n");
}

/*
  Capture stop;
*/
static void
stop_capture(void)
{
	long fact;

	fact = ctrl_inl(VIO_CETCR);	/* read interrupt factor */
	ctrl_outl(~fact, VIO_CETCR);	/* clear interrupt event flag */
	ctrl_outl(0x00000000, VIO_CEIER);	/* Interrupt All Disable */
	ctrl_outl(0x00000000, VIO_CAPSR);	/* Capture Stop */

	config_cam.running = 0;

	DPRINTK("VIO Capture stop reqest\n");
}

/*
  transfer YUV -> RGB function
*/
/*
  Video engine start
*/
static void
start_veu(void)
{
	/* interrupt enable bit set */
	ctrl_outl(0x00000001, VIO_VEIER);

	while ((ctrl_inl(0xa4920510) & 0x03000000) == 0x03000000) {
		udelay(1);
	}

	DPRINTK("VIO Video Engine Start Request\n");

	/* video engine run */
	ctrl_outl(0x00000001, VIO_VESTR);

	config_video.running = 1;

	DPRINTK("VIO_VESTR = %08x VIO_VEIER = %08x VIO_VSTAR = %08x\n",
		ctrl_inl(VIO_VESTR), ctrl_inl(VIO_VEIER), ctrl_inl(VIO_VSTAR));

}

/*
  Video engine stop
*/
static void
stop_veu(void)
{
	ctrl_outl(0x00000000, VIO_VESTR);
	config_video.running = 0;
}

static void
config_rgb_addr(int i)
{
	/* input YUV image address */
	ctrl_outl(Y_BUFF + i * img_buff.vmem_size, VIO_VSAYR);
	ctrl_outl(C_BUFF + i * img_buff.vmem_size, VIO_VSACR);

	/* output RGB image address */
	ctrl_outl(RGB_BUFF + i * img_buff.vmem_size, VIO_VDAYR);
	ctrl_outl(RGB_C_BUFF + i * img_buff.vmem_size, VIO_VDACR);
}

/*
  Configure Video Register
*/
static void
config_yuv2rgb(void)
{
	unsigned long tmpl;
	DPRINTK("VIO4 YUV -> RGB\n");

	/* interrupt clear */
	ctrl_outl(0x00000000, VIO_VEIER);
	ctrl_outl(0x00000000, VIO_VESTR);

	/* config address fixed */
	ctrl_outl(0x00000000, VIO_VAFXR);

	/* input image memory width size on byte */
	if ((config_video.tery == 0x00000001)
	    || (config_video.tery == 0x00000003)) {
		ctrl_outl((config_video.img_width & 0x0000ffff) << 1,
			  VIO_VESWR);
	} else {
		ctrl_outl((config_video.img_width & 0x0000ffff), VIO_VESWR);
	}

	/* image size */
	tmpl = config_video.img_height << 16;
	tmpl |= config_video.img_width;
	ctrl_outl((tmpl & 0x0ffc0ffc), VIO_VESSR);

	/* filter clip size */
	tmpl = (config_video.clip_height) << 16;
	tmpl |= (config_video.clip_width);
	ctrl_outl((tmpl & 0x0ffc0ffc), VIO_VRFSR);

	/* resize filter control */
	tmpl = (config_video.v_resize) << 16;
	tmpl |= config_video.h_resize;
	ctrl_outl((tmpl & 0xfff8fff8), VIO_VRFCR);

	/* output image memory width size on byte */
	if ((config_video.tery == 0x00000001)
	    || (config_video.tery == 0x00000002)) {
		ctrl_outl((config_video.clip_width & 0x0000ffff) << 1,
			  VIO_VEDWR);
	} else {
		ctrl_outl((config_video.clip_width & 0x0000ffff), VIO_VEDWR);
	}

	/*
	   PAD: 0
	   CHDS: 0/ YCrCb422 output
	   WPKF: b'00110
	   CHRR: 0/ YCrCb420 input
	   RPKF: 0 / RGB input data pack
	   DITH: 0
	   TE,RY:b'10 / YCrCb -> RGB
	 */
	tmpl = (config_video.rgb_pad & 0x0000000f) << 24;	/* rgb pad */
	tmpl |= (config_video.chds & 0x00000001) << 22;	/* CHDS */
	tmpl |= (config_video.wpkf & 0x0000001f) << 16;	/* WPKF0-4 */
	tmpl |= (config_video.chrr & 0x00000001) << 14;	/* CHRR */
	tmpl |= (config_video.rpkf & 0x00000007) << 8;	/* RPKF0-2 */
	tmpl |= (config_video.dith & 0x00000001) << 4;	/* DITH */
	tmpl |= (config_video.tery & 0x00000003);	/* TE,RY */
	ctrl_outl(tmpl, VIO_VTRCR);

	/* data swap , vem word-swap-out and byte-swap-in word-swap-in */
	tmpl = ctrl_inl(VIO_VSWPR);
	tmpl &= ~0x00000033;
	if (config_video.tery == 2) {
		ctrl_outl(tmpl | 0x00000013, VIO_VSWPR);
	} else {
		ctrl_outl(tmpl | 0x00000011, VIO_VSWPR);
	}

	config_rgb_addr(0);
}

/*
  Resize and Rotate function
*/
/***************************************************************************
 * name    : open_vio4
 * argument: struct inode *inode : address of inode structerr
 *           struct file  *filp  : address of file structer
 * return  : only zero
 * function: 
 * date    : 2003.11.14
 **************************************************************************/
#define VIO_PRI (8 << 8)
static int
open_vio4(struct inode *inode, struct file *filp)
{
	DPRINTK("VIO4 open\n");

#ifdef DEBUG
	ceu_int_hdr_cnt = 0;
	veu_int_hdr_cnt = 0;
	beu_int_hdr_cnt = 0;
#endif
	/*
	 * registration IRQ
	 */

	request_irq(CEU_IRQ, ceu_int_hdr, SA_INTERRUPT, "vio4-ceu", NULL);
	request_irq(VEU_IRQ, veu_int_hdr, SA_INTERRUPT, "vio4-veu", NULL);
	request_irq(BEU_IRQ, beu_int_hdr, SA_INTERRUPT, "vio4-beu", NULL);

	ctrl_outb(ctrl_inb(INTC_IMR1) | 0x70, INTC_IMCR1);

	DPRINTK("VIO4 :IMCR1 %02x\n", ctrl_inb(INTC_IMCR1));

	init_waitqueue_head(&c_wq);
	init_waitqueue_head(&v_wq);
	init_waitqueue_head(&f_wq);
	init_waitqueue_head(&b_wq);

	/* set default parameter */
	config_cam = default_cam;
	config_video = default_video;
	img_buff = default_img_buff;

	config_capture();	/* configration CEU register */
	config_yuv2rgb();	/* configration VEU register */

	return 0;
}

/***************************************************************************
 * name    : close_vio4
 * argument: struct inode *inode : address of inode structer
 *           struct file  *filp  : address of file structer
 * return  : only zero
 * function: 
 * date    : 2003.11.14
 **************************************************************************/

static int
close_vio4(struct inode *inode, struct file *filp)
{

	stop_capture();
	stop_veu();

	config_cam = default_cam;
	config_video = default_video;

	ctrl_outw(ctrl_inw(VIO_IPR_ADDR) & ~(VIO_PRI), VIO_IPR_ADDR);
	ctrl_outb(ctrl_inb(INTC_IMCR1) | 0x70, INTC_IMR1);

	free_irq(CEU_IRQ, NULL);
	free_irq(VEU_IRQ, NULL);
	free_irq(BEU_IRQ, NULL);

	DPRINTK("VIO4 close\n");

	return 0;
}

/***************************************************************************
 * name    : ioctl_vio4
 * argument: 
 * return  : 
 * function: 
 * date    : 
 **************************************************************************/
static int
ioctl_vio4(struct inode *inode, struct file *filp,
	   unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	/* command check */
	if (_IOC_TYPE(cmd) != VIO4_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > VIO4_IOC_MAXNR)
		return -ENOTTY;

	/* switching command */
	switch (cmd) {
	case VIO4_IOCTCAPTURE:
		config_capture_addr(arg);
		start_capture();
		interruptible_sleep_on(&c_wq);
		break;
	case VIO4_IOCTYUV2RGB:
		veu_event = 0;
		config_rgb_addr(arg);
		DPRINTK("VIO4_IOCTYUV2RGB : buffer number %d\n", (int) arg);
		start_veu();
		wait_event_interruptible(v_wq, veu_event == 1);
		break;
	case VIO4_IOCSCAPCFG:
		if (copy_from_user
		    ((void *) &config_cam, (void *) arg,
		     sizeof (struct vio4_capture))) {
			ret = -EFAULT;
			goto ioctl_vio4_out;
		}
		config_capture();
		break;
	case VIO4_IOCSVEUCFG:
		if (copy_from_user
		    ((void *) &config_video, (void *) arg,
		     sizeof (struct vio4_video))) {
			ret = -EFAULT;
			goto ioctl_vio4_out;
		}
		DPRINTK("VIO4_IOCSVEUCFG \n");
		config_yuv2rgb();
		break;
	case VIO4_IOCSIMGBUF:
		if (copy_from_user
		    ((void *) &img_buff, (void *) arg,
		     sizeof (struct vio4_img_buff))) {
			ret = -EFAULT;
			goto ioctl_vio4_out;
		}
		config_capture();
		config_yuv2rgb();
		break;
	case VIO4_IOCGCAPCFG:
		if (copy_to_user
		    ((void *) arg, (void *) &config_cam,
		     sizeof (struct vio4_capture))) {
			ret = -EFAULT;
			goto ioctl_vio4_out;
		}
		break;
	case VIO4_IOCGVEUCFG:
		if (copy_to_user
		    ((void *) arg, (void *) &config_video,
		     sizeof (struct vio4_video))) {
			ret = -EFAULT;
			goto ioctl_vio4_out;
		}
		break;
	default:
		return -ENOTTY;
		break;
	}
      ioctl_vio4_out:
	return ret;
}

/*
  mmap
*/
static int
mmap_vio4(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long offset = (unsigned long) img_buff.viomem_base;

	pgprot_val(vma->vm_page_prot) &= ~_PAGE_CACHABLE;	/* Non_cache Mode */

	vma->vm_flags |= (VM_RESERVED | VM_IO);

	if (remap_page_range(vma->vm_start, offset,
			     (vma->vm_end - vma->vm_start),
			     vma->vm_page_prot)) {
		DPRINTK("vio4:mmap failed\n");
		return -EAGAIN;
	}
	return 0;
}

/*
 * file operations sturct
 */
static struct file_operations vio4_fops = {
      owner:THIS_MODULE,
      open:open_vio4,
      release:close_vio4,
      ioctl:ioctl_vio4,
      mmap:mmap_vio4,
};
static int
read_proc_vio4(char *buf, char **start, off_t offset,
	       int count, int *eof, void *data)
{
	int len = 0;
	len += sprintf(buf, "interrupt register\n");
	len += sprintf(buf + len, "IMCR1:%02x\n", ctrl_inb(INTC_IMCR1));
	len += sprintf(buf + len, "IMR1 :%02x\n", ctrl_inb(INTC_IMR1));
	len += sprintf(buf + len, "IPRE :%04x\n\n", ctrl_inw(INTC_IPRE));

	len += sprintf(buf + len, "vio4 register :\n");
	len += sprintf(buf + len, "CAPSR :%08x\n", ctrl_inl(VIO_CAPSR));
	len += sprintf(buf + len, "CAPCR :%08x\n", ctrl_inl(VIO_CAPCR));
	len += sprintf(buf + len, "CAMCR :%08x\n", ctrl_inl(VIO_CAMCR));
	len += sprintf(buf + len, "CMCYR :%08x\n", ctrl_inl(VIO_CMCYR));
	len += sprintf(buf + len, "CAMOR :%08x\n", ctrl_inl(VIO_CAMOR));
	len += sprintf(buf + len, "CAPWR :%08x\n", ctrl_inl(VIO_CAPWR));
	len += sprintf(buf + len, "CSTCR :%08x\n", ctrl_inl(VIO_CSTCR));
	len += sprintf(buf + len, "CSECR :%08x\n", ctrl_inl(VIO_CSECR));
	len += sprintf(buf + len, "CFLCR :%08x\n", ctrl_inl(VIO_CFLCR));
	len += sprintf(buf + len, "CFSZR :%08x\n", ctrl_inl(VIO_CFSZR));
	len += sprintf(buf + len, "CDWDR :%08x\n", ctrl_inl(VIO_CDWDR));
	len += sprintf(buf + len, "CDAYR :%08x\n", ctrl_inl(VIO_CDAYR));
	len += sprintf(buf + len, "CDACR :%08x\n", ctrl_inl(VIO_CDACR));
	len += sprintf(buf + len, "CFXAR :%08x\n", ctrl_inl(VIO_CFXAR));
	len += sprintf(buf + len, "CDSWR :%08x\n", ctrl_inl(VIO_CDSWR));
	len += sprintf(buf + len, "CEIER :%08x\n", ctrl_inl(VIO_CEIER));
	len += sprintf(buf + len, "CETCR :%08x\n", ctrl_inl(VIO_CETCR));
	len += sprintf(buf + len, "CETSR :%08x\n", ctrl_inl(VIO_CETSR));
	len += sprintf(buf + len, "CSTSR :%08x\n", ctrl_inl(VIO_CSTSR));
	len += sprintf(buf + len, "CSRTR :%08x\n", ctrl_inl(VIO_CSRTR));

	len += sprintf(buf + len, "VESTR :%08x\n", ctrl_inl(VIO_VESTR));
	len += sprintf(buf + len, "VESWR :%08x\n", ctrl_inl(VIO_VESWR));
	len += sprintf(buf + len, "VESSR :%08x\n", ctrl_inl(VIO_VESSR));
	len += sprintf(buf + len, "VSAYR :%08x\n", ctrl_inl(VIO_VSAYR));
	len += sprintf(buf + len, "VSACR :%08x\n", ctrl_inl(VIO_VSACR));
	len += sprintf(buf + len, "VEDWR :%08x\n", ctrl_inl(VIO_VEDWR));
	len += sprintf(buf + len, "VDAYR :%08x\n", ctrl_inl(VIO_VDAYR));
	len += sprintf(buf + len, "VDACR :%08x\n", ctrl_inl(VIO_VDACR));
	len += sprintf(buf + len, "VTRCR :%08x\n", ctrl_inl(VIO_VTRCR));
	len += sprintf(buf + len, "VRFCR :%08x\n", ctrl_inl(VIO_VRFCR));
	len += sprintf(buf + len, "VRFSR :%08x\n", ctrl_inl(VIO_VRFSR));

	len += sprintf(buf + len, "VAFXR :%08x\n", ctrl_inl(VIO_VAFXR));
	len += sprintf(buf + len, "VSWPR :%08x\n", ctrl_inl(VIO_VSWPR));
	len += sprintf(buf + len, "VEIER :%08x\n", ctrl_inl(VIO_VEIER));
	len += sprintf(buf + len, "VEVTR :%08x\n", ctrl_inl(VIO_VEVTR));
	len += sprintf(buf + len, "VSTAR :%08x\n", ctrl_inl(VIO_VSTAR));
	len += sprintf(buf + len, "VSWRR :%08x\n", ctrl_inl(VIO_VSWRR));

	len += sprintf(buf + len, "MACSR :%08x\n", ctrl_inl(0xa4920510));

	len += sprintf(buf + len, "FESTR :%08x\n", ctrl_inl(VIO_FESTR));
	len += sprintf(buf + len, "FESWR :%08x\n", ctrl_inl(VIO_FESWR));
	len += sprintf(buf + len, "FESSR :%08x\n", ctrl_inl(VIO_FESSR));
	len += sprintf(buf + len, "FSAYR :%08x\n", ctrl_inl(VIO_FSAYR));
	len += sprintf(buf + len, "FSACR :%08x\n", ctrl_inl(VIO_FSACR));
	len += sprintf(buf + len, "FEDWR :%08x\n", ctrl_inl(VIO_FEDWR));
	len += sprintf(buf + len, "FDAYR :%08x\n", ctrl_inl(VIO_FDAYR));
	len += sprintf(buf + len, "FDACR :%08x\n", ctrl_inl(VIO_FDACR));
	len += sprintf(buf + len, "FEFCR :%08x\n", ctrl_inl(VIO_FEFCR));
	len += sprintf(buf + len, "FEMCR :%08x\n", ctrl_inl(VIO_FEMCR));
	len += sprintf(buf + len, "FVTCR :%08x\n", ctrl_inl(VIO_FVTCR));
	len += sprintf(buf + len, "FHTCR :%08x\n", ctrl_inl(VIO_FHTCR));

	len += sprintf(buf + len, "BESTR  :%08x\n", ctrl_inl(VIO_BESTR));
	len += sprintf(buf + len, "BSMWR1 :%08x\n", ctrl_inl(VIO_BSMWR1));
	len += sprintf(buf + len, "BSSZR1 :%08x\n", ctrl_inl(VIO_BSSZR1));
	len += sprintf(buf + len, "BSAYR1 :%08x\n", ctrl_inl(VIO_BSAYR1));
	len += sprintf(buf + len, "BSACR1 :%08x\n", ctrl_inl(VIO_BSACR1));
	len += sprintf(buf + len, "BSIFR1 :%08x\n", ctrl_inl(VIO_BSIFR1));
	len += sprintf(buf + len, "BSMWR2 :%08x\n", ctrl_inl(VIO_BSMWR2));
	len += sprintf(buf + len, "BSSZR2 :%08x\n", ctrl_inl(VIO_BSSZR2));
	len += sprintf(buf + len, "BSAYR2 :%08x\n", ctrl_inl(VIO_BSAYR2));
	len += sprintf(buf + len, "BSACR2 :%08x\n", ctrl_inl(VIO_BSACR2));
	len += sprintf(buf + len, "BSIFR2 :%08x\n", ctrl_inl(VIO_BSIFR2));
	len += sprintf(buf + len, "BSMWR3 :%08x\n", ctrl_inl(VIO_BSMWR3));
	len += sprintf(buf + len, "BSSZR3 :%08x\n", ctrl_inl(VIO_BSSZR3));
	len += sprintf(buf + len, "BSAYR3 :%08x\n", ctrl_inl(VIO_BSAYR3));
	len += sprintf(buf + len, "BSACR3 :%08x\n", ctrl_inl(VIO_BSACR3));
	len += sprintf(buf + len, "BSIFR3 :%08x\n", ctrl_inl(VIO_BSIFR3));
	len += sprintf(buf + len, "BTPSR  :%08x\n", ctrl_inl(VIO_BTPSR));
	len += sprintf(buf + len, "BBLCR0 :%08x\n", ctrl_inl(VIO_BBLCR0));
	len += sprintf(buf + len, "BBLCR1 :%08x\n", ctrl_inl(VIO_BBLCR1));
	len += sprintf(buf + len, "BLOCR1 :%08x\n", ctrl_inl(VIO_BLOCR1));
	len += sprintf(buf + len, "BLOCR2 :%08x\n", ctrl_inl(VIO_BLOCR2));
	len += sprintf(buf + len, "BLOCR3 :%08x\n", ctrl_inl(VIO_BLOCR3));
	len += sprintf(buf + len, "BPKFR  :%08x\n", ctrl_inl(VIO_BPKFR));
	len += sprintf(buf + len, "BAPCR1 :%08x\n", ctrl_inl(VIO_BAPCR1));
	len += sprintf(buf + len, "BAPCR2 :%08x\n", ctrl_inl(VIO_BAPCR2));
	len += sprintf(buf + len, "BDMWR  :%08x\n", ctrl_inl(VIO_BDMWR));
	len += sprintf(buf + len, "BDAYR  :%08x\n", ctrl_inl(VIO_BDAYR));
	len += sprintf(buf + len, "BDACR  :%08x\n", ctrl_inl(VIO_BDACR));
	len += sprintf(buf + len, "BAFXR  :%08x\n", ctrl_inl(VIO_BAFXR));
	len += sprintf(buf + len, "BSWPR  :%08x\n", ctrl_inl(VIO_BSWPR));
	len += sprintf(buf + len, "BEIER  :%08x\n", ctrl_inl(VIO_BEIER));
	len += sprintf(buf + len, "BEVTR  :%08x\n", ctrl_inl(VIO_BEVTR));
	len += sprintf(buf + len, "BSTAR  :%08x\n", ctrl_inl(VIO_BSTAR));
	len += sprintf(buf + len, "BBRSTR :%08x\n", ctrl_inl(VIO_BBRSTR));

	*eof = 1;
	return len;
}

static devfs_handle_t devfs_handle;

/***************************************************************************
 * name    : init_vio4
 * argument: void
 * return  : fail in not get device major number
 * function: regist device name and getting device major number.
 * date    : 2003.11.14
 **************************************************************************/
static int __init
init_vio4(void)
{
	int dev_major;

	/*
	 * module regist
	 */
	dev_major = register_chrdev(VIO4_DEV_MAJOR, VIO4_DEV_NAME, &vio4_fops);
	if (dev_major < 0) {
		printk(KERN_WARNING "vio: can't get major %d\n", dev_major);
		return dev_major;
	}
	if (vio4_major == 0) {
		vio4_major = dev_major;	/* dynamic assignment major number */
	}

	devfs_handle = devfs_register(NULL, VIO4_DEV_NAME, DEVFS_FL_AUTO_DEVNUM,
				      VIO4_DEV_MAJOR, 0,
				      S_IFCHR | S_IRUSR | S_IWUSR,
				      &vio4_fops, NULL);
	/* register initilaization */
	init_hw_vio4();

	/* /proc file system entry */
	create_proc_read_entry("vio4", 0, NULL, read_proc_vio4, NULL);

	DPRINTK("VIO4 Load vio4_major %d\n", vio4_major);
	init_waitqueue_head(&c_wq);
	init_waitqueue_head(&v_wq);
	init_waitqueue_head(&f_wq);
	init_waitqueue_head(&b_wq);

#ifdef CONFIG_DPM
	vio4_ldm_register();
#endif
	return 0;
}

/***************************************************************************
 * name    : vio4_clearnup
 * argument: void
 * return  : void
 * function: unregister device
 * date    : 2003.11.14
 **************************************************************************/
static void __exit
cleanup_vio4(void)
{
#ifdef CONFIG_DPM
	vio4_ldm_unregister();
#endif
	/* remove /proc file system */
	remove_proc_entry("vio4", NULL);
	devfs_unregister(devfs_handle);
	/* un-regist */
	unregister_chrdev(vio4_major, VIO4_DEV_NAME);

	DPRINTK("VIO4 Cleanup\n");

	return;
}

#ifdef CONFIG_PM
static int
vio4_suspend(struct device * dev,  u32 state, u32 level )
{
        switch (level) {
        case SUSPEND_POWER_DOWN:
#if 0  /* should identify which stanby mode */
		if (shm3_stby_mode & (SWSTBY))
			goto skip_point;
#endif		
		if (config_cam.running) {
			stop_capture();
			/* FIXME: For appropriate flag */
			config_cam.running = 1; 
		}
		
		if (config_video.running) {
			stop_veu();
			/* FIXME: For appropriate flag */
			config_video.running = 1;
		}

	skip_point:
		disable_irq(CEU_IRQ);
		disable_irq(VEU_IRQ);
		disable_irq(BEU_IRQ);

		ctrl_outw(ctrl_inw(VIO_IPR_ADDR) & ~(VIO_PRI), VIO_IPR_ADDR);
		ctrl_outb(ctrl_inb(INTC_IMCR1) | 0x70, INTC_IMR1);
		
		/* VIO clock stop */
                ctrl_outl(ctrl_inl(MSTPCR2) | MSTP202|MSTP204, MSTPCR2);   
		
                break;
        }
        return 0;
}

static void
do_resume(void *x)
{
	enable_irq(CEU_IRQ);
	enable_irq(VEU_IRQ);
	enable_irq(BEU_IRQ);
#if 0  /* should identify which stanby mode */
	if ((shm3_stby_mode & (SWSTBY)) && !(shm3_stby_mode & (MLRST)))
		goto skip_point;
#endif
	init_hw_vio4();
	
	if (!config_cam.running)
		goto skip_point;
	
	config_capture();
	config_yuv2rgb();
	
	if (!config_video.running)	
		goto skip_point;
	
	config_capture_addr(0);
	start_capture();
	interruptible_sleep_on(&c_wq);
	
	config_rgb_addr(0);
	start_veu();
	wait_event_interruptible(v_wq, veu_event == 1);

 skip_point:
	if (waitqueue_active(&c_wq))
		wake_up_interruptible(&c_wq);
	
	if (waitqueue_active(&v_wq)) {
		veu_event = 1;
		wake_up_interruptible(&v_wq);
	}
	
	if (waitqueue_active(&b_wq)) {
		beu_event = 1;        
		wake_up_interruptible(&b_wq);
	}
	
	if (waitqueue_active(&f_wq)) {
		feu_event = 1;
		wake_up_interruptible(&f_wq);
	}

	return;
}

static struct tq_struct resume_task = { routine: do_resume };

static int
vio4_resume(struct device * dev, u32 level )
{
        switch (level) {
        case RESUME_POWER_ON:
		/*
		 * Resume is getting called in an interrupt context on
		 * SH, and resume requires waiting on queues etc. to power
		 * up the camera. So we can't resume here. So we have to
		 * use a kernel thread for resume requests (PITA).
		 */
		resume_task.data = NULL;
		schedule_task(&resume_task);

		break;
        }
        return 0;
}
#endif

#ifdef CONFIG_DPM

static struct device_driver vio4_driver_ldm = {
        name:           "vio4",
        devclass:       NULL,
        probe:          NULL,
        suspend:        vio4_suspend,
        resume:         vio4_resume,
        scale:          NULL,
        remove:         NULL,
};

static struct device vio4_device_ldm = {
        name:           "SUPERH Camera",
        bus_id:         "vio4",
        driver:         NULL,
        power_state:    DPM_POWER_ON,
};

static void vio4_ldm_register(void)
{
        extern void plb_driver_register(struct device_driver *driver);
        extern void plb_device_register(struct device *device);

        plb_driver_register(&vio4_driver_ldm);
        plb_device_register(&vio4_device_ldm);
}

static void vio4_ldm_unregister(void)
{
        extern void plb_driver_unregister(struct device_driver *driver);
        extern void plb_device_unregister(struct device *device);

        plb_driver_unregister(&vio4_driver_ldm);
        plb_device_unregister(&vio4_device_ldm);
}

#endif
MODULE_LICENSE("GPL");

module_init(init_vio4);
module_exit(cleanup_vio4);

