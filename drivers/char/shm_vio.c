/*
 * drivers/char/shm_vio.c
 *
 * SH-7300 VIO3 driver for Super-H
 *
 * Author: Takashi SHUDO
 *
 * 2003 (c) Takashi SHUDO. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifdef CONFIG_SH_7751_SOLUTION_ENGINE
#include <asm/hitachi_7751se.h>
#elif defined(CONFIG_SH_MOBILE_SOLUTION_ENGINE)
#include <asm/hitachi_shmse.h>
#else
#include <asm/hitachi_se.h>
#endif


#define YC_LINEAR		/* When Y of a frame buffer and CrCb are connected
				   and arranged */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/pm.h>
#include <linux/device.h>
#include <asm/io.h>
#include <asm/irq-sh7300.h>
#include <linux/devfs_fs_kernel.h>
#include "shm_vio.h"

#define DEBUG
#ifdef DEBUG
#define DPRINTK(fmt, args...) printk(KERN_DEBUG fmt, ## args)
#else
#define DPRINTK(fmt, args...)
#endif


#define VIO_INTLVL	10	/* Interruption priority level */

/*
 * wait queue for processes wanting to open
 */
static DECLARE_WAIT_QUEUE_HEAD(cap_wq);
static DECLARE_WAIT_QUEUE_HEAD(vip_wq);
static DECLARE_RWSEM(vio_sem);

static void
init_vio3(void)
{
	/* PFC setting */
	ctrl_outw(ctrl_inw(PSELA) | 0x8000, PSELA);
	ctrl_outw(0x0000, PACR);
	ctrl_outw(ctrl_inw(PECR) & 0x003F, PECR);
	ctrl_outw(ctrl_inw(PMCR) & 0x3FFF, PMCR);
	ctrl_outw(ctrl_inw(HIZCRA) & 0xBFFF, HIZCRA);
	ctrl_outw(ctrl_inw(DRVCR) | 0x0001, DRVCR);

	/* Initialize all registers */
	ctrl_outl(0x00000000, VIO3_CAPSR);
	ctrl_outl(0x00000000, VIO3_CAPCR);
	ctrl_outl(0x00000000, VIO3_CAMCR);
	ctrl_outl(0x00000000, VIO3_CAMOR);
	ctrl_outl(0x00000000, VIO3_CAPWR);
	ctrl_outl(0x00000000, VIO3_CSTCR);
	ctrl_outl(0x00000000, VIO3_CFLCR);
	ctrl_outl(0x00000000, VIO3_CFSZR);
	ctrl_outl(0x00000000, VIO3_CDWDR);
	ctrl_outl(0x00000000, VIO3_CDAYR);
	ctrl_outl(0x00000000, VIO3_CDACR);
	ctrl_outl(0x00000000, VIO3_CDMCR);
	ctrl_outl(0x00000000, VIO3_CDBCR);
	ctrl_outl(0x00000000, VIO3_CROTR);
	ctrl_outl(0x00000000, VIO3_VIPSR);
	ctrl_outl(0x00000000, VIO3_VVSWR);
	ctrl_outl(0x00000000, VIO3_VVSSR);
	ctrl_outl(0x00000000, VIO3_VDWDR);
	ctrl_outl(0x00000000, VIO3_VSARR);
	ctrl_outl(0x00000000, VIO3_VSAGR);
	ctrl_outl(0x00000000, VIO3_VSABR);
	ctrl_outl(0x00000000, VIO3_VDAYR);
	ctrl_outl(0x00000000, VIO3_VDACR);
	ctrl_outl(0x00000000, VIO3_VTRCR);
	ctrl_outl(0x00000000, VIO3_VFLCR);
	ctrl_outl(0x00000000, VIO3_VFSZR);
	ctrl_outl(0x00000000, VIO3_VLUTR);
	ctrl_outl(0x00000000, VIO3_VROTR);
	ctrl_outl(0x00000000, VIO3_VOSCR);
	ctrl_outl(0x00000000, VIO3_VOSWR);
	ctrl_outl(0x00000000, VIO3_VODSR);
	ctrl_outl(0x00000000, VIO3_VODOR);
	ctrl_outl(0x00000000, VIO3_VOSAR);
	ctrl_outl(0x00000000, VIO3_VOAYR);
	ctrl_outl(0x00000000, VIO3_VOACR);
	ctrl_outl(0x00000000, VIO3_VHCCR);
	ctrl_outl(0x00000000, VIO3_VHDSR);
	ctrl_outl(0x00000000, VIO3_VHDOR);
	ctrl_outl(0x00000000, VIO3_VHAYR);
	ctrl_outl(0x00000000, VIO3_VHACR);
	ctrl_outl(0x00000000, VIO3_FXADR);
	ctrl_outl(0x00000000, VIO3_DSWPR);
	ctrl_outl(0x00000000, VIO3_EVTCR);
	ctrl_outl(0x00000000, VIO3_EVTSR);
	ctrl_outl(0x00000000, VIO3_STATR);
	ctrl_outl(0x00000000, VIO3_SRSTR);
	vio_ldm_register();
}

static void
set_camera(void)
{
	/*
	   4:2:0
	   Cb0,Y0,Cr0,Y1
	   VD:High Active
	   HD:High Active
	 */
	ctrl_outl(0x0000004, VIO3_CAMCR);	/* 4:2:0 */

	/* offset */
	ctrl_outl(((M_HEIGHT << 16) + (M_WIDTH << 1)) & 0x0fff0fff, VIO3_CAMOR);

	/* size */
	ctrl_outl(((M_HEIGHT << 16) + (M_WIDTH << 1)) & 0x0ffc0ff8, VIO3_CAPWR);

	/* filter */
	ctrl_outl((RESIZERATIO_1_1 << 16) + RESIZERATIO_1_1, VIO3_CFLCR);

	/* haba */
	ctrl_outl((M_WIDTH & 0x0fff), VIO3_CDWDR);

	/* clip */
	ctrl_outl(((M_HEIGHT << 16) + M_WIDTH) & 0x07fc07fc, VIO3_CFSZR);

	/* address inc */
	ctrl_outl(0x00000000, VIO3_FXADR);

	/* data swap */
	ctrl_outl(0x00000013, VIO3_DSWPR);
}

static void
start_capture(unsigned long fn)
{
	DPRINTK("VIO3 Capture\n");

	down_write(&vio_sem);

	ctrl_outl(0x00060d20, VIO3_VTRCR);	/* 4:2:0 */

	/* Y,Cr,Cb Data Address */
#ifdef YC_REVERSE
	ctrl_outl(0x00000003, VIO3_CROTR);	/* 180-degree rotation */
	ctrl_outl(Y_BUFF + (VMEM_SIZE * fn) + M_WIDTH * M_HEIGHT - 4,
		  VIO3_CDAYR);
#ifdef YC_LINEAR
	ctrl_outl(Y_BUFF + Y_SIZE + (VMEM_SIZE * fn) +
		  (M_WIDTH * M_HEIGHT) / 2 - 4, VIO3_CDACR);
#else
	ctrl_outl(C_BUFF + (VMEM_SIZE * fn) + (M_WIDTH * M_HEIGHT) / 2 - 4,
		  VIO3_CDACR);
#endif
#else
	ctrl_outl(0x00000000, VIO3_CROTR);
	ctrl_outl(Y_BUFF + (VMEM_SIZE * fn), VIO3_CDAYR);
#ifdef YC_LINEAR
	ctrl_outl(Y_BUFF + Y_SIZE + (VMEM_SIZE * fn), VIO3_CDACR);
#else
	ctrl_outl(C_BUFF + (VMEM_SIZE * fn), VIO3_CDACR);
#endif
#endif

	/* data swap */
	ctrl_outl(0x00000053, VIO3_DSWPR);

	ctrl_outl((ctrl_inl(VIO3_VIPSR) & ~0x00000010), VIO3_VIPSR);	/* VIP */
	ctrl_outl(0x000000fe, VIO3_CAPCR);	/* YCrCb only, widthout HD Int Enable */

	up_write(&vio_sem);
}

static void
start_yuv2rgb(unsigned short yuv_fn, unsigned short rgb_fn, unsigned char rot)
{
	DPRINTK("VIO3 YUV(%d) -> RGB(%d)\n", yuv_fn, rgb_fn);

	down_write(&vio_sem);

	/* data swap */
	ctrl_outl(0x00000013, VIO3_DSWPR);

	ctrl_outl(((M_HEIGHT << 16) + M_WIDTH) & 0x07fc07fc, VIO3_VFSZR);

	ctrl_outl(rot & 0x03, VIO3_VROTR);

	/* YUV input address */
	ctrl_outl(Y_BUFF + (VMEM_SIZE * yuv_fn), VIO3_VSAGR);
#ifdef YC_LINEAR
	ctrl_outl(Y_BUFF + Y_SIZE + (VMEM_SIZE * yuv_fn), VIO3_VSABR);
#else
	ctrl_outl(C_BUFF + (VMEM_SIZE * yuv_fn), VIO3_VSABR);
#endif

	/* RGB output address */
	switch (rot) {
	case 0x01:
		ctrl_outl(RGB_BUFF + (M_WIDTH * 2) - 4 + (VMEM_SIZE * rgb_fn),
			  VIO3_VDAYR);
		break;

	default:
		ctrl_outl(RGB_BUFF + (VMEM_SIZE * rgb_fn), VIO3_VDAYR);
		break;
	}

	ctrl_outl((ctrl_inl(VIO3_VIPSR) | 0x00000010), VIO3_VIPSR);

	ctrl_outl(0x000000fe, VIO3_CAPCR);	/* RGB,YCrCb, widthout HD Int Enable */

	/* CDREP=1(4:2:0),TE=1,RY=0 */
	ctrl_outl(0x00060d21, VIO3_VTRCR);

	/* Conversion start */
	ctrl_outl((ctrl_inl(VIO3_VIPSR) | 0x00000001), VIO3_VIPSR);

	ctrl_outl(ctrl_inl(VIO3_EVTCR) & ~0x08000f01, VIO3_EVTCR);
	/* VD,HD <- 0 */

	up_write(&vio_sem);
}

void
vio_int_hdr(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned long fact;

	/* Clear interrupt factor */
	fact = ctrl_inl(VIO3_EVTCR);

	/****************************************
	 * FPC()
	 ****************************************/
	if (fact & 0x08000000) {
		ctrl_outl(~fact, VIO3_EVTCR);

		DPRINTK("VIO3 FPC int\n");
	}

	/****************************************
	 * NVD( VD nothing interrupt )
	 ****************************************/
	if (fact & 0x04000000) {
		ctrl_outl(~fact, VIO3_EVTCR);

		DPRINTK("VIO3 NVD int\n");
	}

	/****************************************
	 * NHD( HD nothing interrupt )
	 ****************************************/
	if (fact & 0x02000000) {
		ctrl_outl(~fact, VIO3_EVTCR);

		DPRINTK("VIO3 NHD int\n");
	}

	/****************************************
	 * CBOF( CIA buffer overflow interrupt )
	 ****************************************/
	if (fact & 0x01000000) {
		ctrl_outl(~fact, VIO3_EVTCR);

		DPRINTK("VIO3 CBOF int\n");
	}

	/****************************************
	 * RW( RAM access error interrupt )
	 ****************************************/
	if (fact & 0x00020000) {
		ctrl_outl(~fact, VIO3_EVTCR);

		DPRINTK("VIO3 RW int\n");
	}

	/****************************************
	 * VBOF( VIP buffer overflow interrupt )
	 ****************************************/
	if (fact & 0x00010000) {
		ctrl_outl(~fact, VIO3_EVTCR);

		DPRINTK("VIO3 VBOF int\n");
	}

	/****************************************
	 * CPE( Capture end interrupt )
	 ****************************************/
	if (fact & 0x00000400) {
		ctrl_outl(~fact, VIO3_EVTCR);

		wake_up_interruptible(&cap_wq);

		DPRINTK("VIO3 Cap END\n");
	}

	/****************************************
	 * RVS(  )
	 ****************************************/
	if (fact & 0x00000800) {
		if (waitqueue_active(&cap_wq) && !(ctrl_inl(VIO3_STATR) & CIAON)) {
			DPRINTK("VIO3 Capture start\n");
			ctrl_outl(0x00000001, VIO3_CAPSR);	/* Start */
		}
		ctrl_outl(~fact, VIO3_EVTCR);
		
		DPRINTK("VIO3 RVS int\n");
	}

	/****************************************
	 * VD( VD interrupt )
	 ****************************************/
	if (fact & 0x00000200) {
		ctrl_outl(~fact, VIO3_EVTCR);

		DPRINTK("VIO3 VD int\n");
	}

	/****************************************
	 * HD( HD interrupt )
	 ****************************************/
	if (fact & 0x00000100) {
		ctrl_outl(~fact, VIO3_EVTCR);
	}

	/****************************************
	 * CVE( VIP end interrupt )
	 ****************************************/
	if (fact & 0x00000001) {

		DPRINTK("VIO3 VIP END int\n");

		ctrl_outl(~fact, VIO3_EVTCR);
		wake_up_interruptible(&vip_wq);
	}
}

static void
open_vio(void)
{
	/* CIA setup */
	ctrl_outl(M_WIDTH, VIO3_VVSWR);
	ctrl_outl(((M_HEIGHT << 16) + M_WIDTH) & 0x07fc07fc, VIO3_VVSSR);

	/* VIP setup */
	ctrl_outl(((M_HEIGHT << 16) + M_WIDTH) & 0x07fc07fc, VIO3_VFSZR);
	ctrl_outl(((RESIZERATIO_1_1 << 16) + RESIZERATIO_1_1), VIO3_VFLCR);
	ctrl_outl(M_WIDTH * 2, VIO3_VDWDR);

	ctrl_outw(ctrl_inw(IPRE) | (VIO_INTLVL << 8), IPRE);	/* Set priority */
	request_irq(VIO_IRQ, vio_int_hdr, SA_INTERRUPT, "vio", NULL);

	ctrl_outb(0x10, IMCR1);	/* interrupt mask clear */
}

static void
release_vio(void)
{
	ctrl_outw(ctrl_inw(IPRE) & ~(VIO_INTLVL << 8), IPRE);	/* Reset priority */
	ctrl_outb(0x10, IMR1);	/* interrupt mask */

	free_irq(VIO_IRQ, NULL);
}

int eof = 0;

/*
  open
*/
static int
vio3_open(struct inode *inode, struct file *filp)
{
	DPRINTK(KERN_INFO "VIO3 Open\n");

	eof = 0;

	open_vio();

	set_camera();

	return 0;
}

/*
  release
*/
static int
vio3_release(struct inode *inode, struct file *filp)
{
	DPRINTK(KERN_INFO "VIO3 Release\n");

	release_vio();

	return 0;
}

/*
  read
*/
static int
vio3_read(struct file *filp, char *buf, size_t count, loff_t * ppos)
{
	int err;
#ifdef DEBUG
	volatile unsigned short *p = (volatile unsigned short *) PA_LED;
#endif
	DPRINTK(KERN_INFO "VIO3 Read\n");
#if 0
	if (eof != 0) {
		return 0;
	}
#endif

	start_capture(0x00000000);
	interruptible_sleep_on(&cap_wq);
	
	start_yuv2rgb(0, 0, 0x01);
	interruptible_sleep_on(&vip_wq);

#ifdef DEBUG
	*p = 0x0000;		/* All LED off */
#endif
	err = copy_to_user(buf, "ok", 3);

	if (err != 0) {
		return -EFAULT;
	}
	eof = 1;

	return 3;
}

/*
  ioctl
*/
static int
vio3_ioctl(struct inode *inode, struct file *filp,
	   unsigned int cmd, unsigned long arg)
{
	DPRINTK(KERN_INFO "VIO3 ioctl\n");

	switch (cmd) {
	case VIOCTL_CAPTURE:
		DPRINTK("VIOCTL_CAPTURE\n");
		start_capture(arg);
		interruptible_sleep_on(&cap_wq);
		break;
		
	case VIOCTL_YUV2RGB:
		DPRINTK("VIOCTL_YUV2RGB\n");
		start_yuv2rgb(arg >> 16, arg & 0xffff, 0);
		interruptible_sleep_on(&vip_wq);
		break;

	default:
		DPRINTK("vio: unknown ioctl 0x%x\n", cmd);
		return -EINVAL;
	}

	return 0;
}

/*
  mmap
*/
static int
vio3_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long offset = (unsigned long) VIOMEM_BASE;

	if (offset >= __pa(VIOMEM_BASE) || (filp->f_flags & O_SYNC)) {
		vma->vm_flags |= VM_IO;
	}
	vma->vm_flags |= VM_RESERVED;

	if (remap_page_range(vma->vm_start, offset, vma->vm_end - vma->vm_start,
			     vma->vm_page_prot)) {
		return -EAGAIN;
	}

	return 0;
}

static struct file_operations vio3_fops = {
	owner:        THIS_MODULE,
	read:         vio3_read,
	ioctl:        vio3_ioctl,
	mmap:         vio3_mmap,
	open:         vio3_open,
	release:      vio3_release,
};

static devfs_handle_t devfs_handle;
static int vio3_major = 0;

static int __init
init_vio(void)
{
	int result;

	result = devfs_register_chrdev(vio3_major, "vio", &vio3_fops);
	if (result < 0) {
		printk(KERN_WARNING "vio: can't get major %d\n", vio3_major);
		return result;
	}
	if (vio3_major == 0) {
		vio3_major = result;
	}
	devfs_handle = devfs_register(NULL, "vio", DEVFS_FL_AUTO_DEVNUM,
				      vio3_major, 0,
				      S_IFCHR | S_IRUSR | S_IWUSR,
				      &vio3_fops, NULL);
	init_vio3();

	DPRINTK(KERN_INFO "VIO3 Load\n");

	return 0;
}

static void __exit
cleanup_vio(void)
{
	devfs_unregister(devfs_handle);
	unregister_chrdev(vio3_major, "vio");
	vio_ldm_unregister();
	DPRINTK(KERN_INFO "VIO3 Cleanup\n");
}

static int
vio_suspend(struct device * dev,  u32 state, u32 level )
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		ctrl_outb(ctrl_inb(STBCR3) | 0x10, STBCR3);   /* VIO clock stop */
		break;
	}
	return(0);
}

static int
vio_resume(struct device * dev, u32 level )
{
	switch (level) {
	case RESUME_POWER_ON:
		ctrl_outb(ctrl_inb(STBCR3) & ~0x10, STBCR3);   /* VIO clock start */
		if (waitqueue_active(&cap_wq))
			wake_up_interruptible(&cap_wq);
		if (waitqueue_active(&vip_wq))
			wake_up_interruptible(&vip_wq);
		break;
	}
	return(0);
}

static struct device_driver vio_driver_ldm = {
	name:      	"vio",
	devclass:  	NULL,
	probe:     	NULL,
	suspend:   	vio_suspend,
	resume:    	vio_resume,
	scale:	  	NULL,
	remove:    	NULL,
};

static struct device vio_device_ldm = {
	name:		"SUPERH Camera",
	bus_id:		"vio",
	driver: 	NULL,
	power_state:	DPM_POWER_ON,
};

static void vio_ldm_register(void)
{
#ifdef CONFIG_SUPERH
	extern void plb_driver_register(struct device_driver *driver);
	extern void plb_device_register(struct device *device);

	plb_driver_register(&vio_driver_ldm);
	plb_device_register(&vio_device_ldm);
#endif
}

static void vio_ldm_unregister(void)
{
#ifdef CONFIG_SUPERH
	extern void plb_driver_unregister(struct device_driver *driver);
	extern void plb_device_unregister(struct device *device);

	plb_driver_unregister(&vio_driver_ldm);
	plb_device_unregister(&vio_device_ldm);
#endif
}

MODULE_LICENSE("GPL");
module_init(init_vio);
module_exit(cleanup_vio);

EXPORT_SYMBOL(vio_suspend);
EXPORT_SYMBOL(vio_resume);
