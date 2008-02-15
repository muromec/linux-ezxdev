/*
 * linux/include/asm-arm/arch-pxa/keypad.h
 * Intel Mainstone keypad driver definitions
 *
 * Copyright (C) 2003, Intel Corporation (yu.tang@intel.com)
 * Copyright 2003 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *	   source@mvista.com
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
 */
/*
 * 2005 Motorola implement Intel sighting patch by w15879, for EZX platform
 */

#ifndef _ASM_ARCH_KEYPAD_H
#define _ASM_ARCH_KEYPAD_H

#include <linux/types.h>

#define KPIOGET_INTERVAL	_IOR('k', 0, __u16)
#define KPIOGET_IGNMULTI	_IOR('k', 1, __u16)
#define KPIOSET_INTERVAL	_IOW('k', 0, __u16)
#define KPIOSET_IGNMULTI	_IOW('k', 1, __u16)
#define KPIOSET_DKINTERVAL 	_IOW('k', 2, __u16)
#define KPIOSET_MKINTERVAL 	_IOW('k', 3, __u16)

#ifdef __KERNEL__

#define KP_DIRECT 	0x1
#define KP_MATRIX	0x2

/* Event got from keypad */
struct kp_event {
	unsigned int jiffies;
	unsigned int flags;
	unsigned int direct; 
	unsigned int rotary;
	unsigned int matrix[5]; 
};

/* Event queue length */
#define MAX_KPES 32

/* Event queue and syncronization stuff */
struct kpe_queue {
	unsigned short head, tail, len;
	struct kp_event	kpes[MAX_KPES];
	spinlock_t spinlock;
	wait_queue_head_t waitq;
};

extern int kp_init(void);
extern int kp_wait(struct kp_event *, int);
extern unsigned int kp_poll(struct file *, poll_table *);

/* Turn debug off */
#undef DEBUG

/*
 *  Debug macros 
 */
#if DEBUG
#  define DPRINTK(fmt, args...)	printk("KP[%s:%d]: " fmt "\n", __FILE__, __LINE__ , ## args)
#else
#  define DPRINTK(fmt, args...)
#endif

#endif /* __KERNEL__ */

#endif /* _ASM_ARCH_KEYPAD_H_ */
