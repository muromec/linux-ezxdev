/*
 * ocp.h
 *
 *
 * 	Current Maintainer
 *      Armin Kuster akuster@pacbell.net
 *      Jan, 2002
 *
 *
 *
 * This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR   IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
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
 *
 *	Version 1.0 (02/01/26) - A. Kuster
 *	Initial version	 -
 *
 *	Version 1.1 (04/25/02)
 *		added active to ocp_dev for use when the device 
 *		needs to be unregistered temporarily and the data lives
 *		on in an structure such as net_device.
 *
 *	Version  1.2 (05/15/02) - Armin
 *	added DMA_TO* defines from pci.h
 *
 *	Version 1.3 (05/24/02) - Armin
 *	name change for API's to <class>_<action>_<object>
 *	added type_info strust
 *
 */

#ifdef __KERNEL__
#ifndef __OCP_H__
#define __OCP_H__

#include <linux/list.h>
#include <linux/config.h>
#include <linux/devfs_fs_kernel.h>

#include <asm/mmu.h>		/* For phys_addr_t */

#if defined(CONFIG_IBM_OCP)
#include <platforms/ibm_ocp.h>
#endif

#define OCP_MAX_IRQS	7
#define MAX_EMACS	4
#define OCP_IRQ_NA	-1	/* used when ocp device does not have an irq */
#define OCP_IRQ_MUL	-2	/* used for ocp devices with multiply irqs */
#define OCP_NULL_TYPE	-1	/* used to mark end of list */
#define OCP_CPM_NA	0	/* No Clock or Power Management avaliable */
#define EMACS_PER_ZMII	4

extern struct list_head ocp_list;

enum ocp_type {
	PCI = 0,
	GPT,			/* General purpose timers */
	UART,
	OPB,
	IIC,
	GPIO,
	EMAC,
	ZMII,
	IDE,
	USB,
	SCI,			/* Smart card */
	AUDIO,
	SSP,			/* sync serial port */
	SCP,			/* serial controller port */
	SCC,			/* serial contoller */
	VIDEO,
	DMA,
	UIC,
	RTC,
	LED
};

struct type_info {
	char name[16];
	char desc[50];
};

struct ocp_def {
	enum ocp_type type;
	phys_addr_t paddr;
	int irq;
	unsigned long cpm;
};

/* Struct for single ocp device managment */
struct ocp_dev {
	struct list_head ocp_list;
	char name[16];
	u16 num;
	enum ocp_type type;	/* OCP device type */
	phys_addr_t paddr;
	void *vaddr;
	u32 flags;
	int irq;
	void *ocpdev;		/* ocp device struct  pointer */
#if defined(CONFIG_PM)
	u32 current_state;	/* Current operating state. In ACPI-speak,
				   this is D0-D3, D0 being fully functional,
				   and D3 being off. */

	int (*save_state) (u32 state);	/* Save Device Context */
	int (*suspend) (u32 state);	/* Device suspended */
	int (*resume) (u32 state);	/* Device woken up */
	int (*enable_wake) (u32 state, int enable);	/* Enable wake event */
#endif
#if defined(CONFIG_OCP_PROC)
	struct proc_dir_entry *procent;	/* device entry in /proc/bus/ocp */
#endif
};
#define ocp_dev_g(n) list_entry(n, struct ocp_dev, ocp_list)

/* Similar to the helpers above, these manipulate per-ocp_dev
 * driver-specific data.  Currently stored as ocp_dev::ocpdev,
 * a void pointer, but it is not present on older kernels.
 */
static inline void *
ocp_get_drvdata(struct ocp_dev *pdev)
{
	return pdev->ocpdev;
}

static inline void
ocp_set_drvdata(struct ocp_dev *pdev, void *data)
{
	pdev->ocpdev = data;
}

extern int ocp_register(struct ocp_dev *drv);
extern void ocp_unregister(struct ocp_dev *drv);
extern struct ocp_dev *ocp_alloc_dev(int size);
extern void ocp_free_dev(void *dev);
extern struct ocp_dev *ocp_get_dev(int type, int index);
extern int ocp_proc_attach_device(struct ocp_dev *dev);
extern int ocp_proc_detach_device(struct ocp_dev *dev);
extern unsigned long ocp_get_paddr(int type, int dev_num);
extern int ocp_get_max(int type);
extern int ocp_get_irq(int type, int dev_num);
extern int ocp_get_pm(int type, int dev_num);

#endif				/* __OCP_H__ */
#endif				/* __KERNEL__ */
