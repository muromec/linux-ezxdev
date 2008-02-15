/*
 * File: fb.h
 *
 * This file is the common header file for the arm9 framebuffers.
 *
 * Copyright (C) 2001 RidgeRun, Inc.  
 * Author: Alex McMains <aam@ridgerun.com> 2001/05/10
 *         Greg Lonnon <glonnon@ridgerun.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS  PROVIDED  ``AS  IS''  AND   ANY  EXPRESS  OR IMPLIED
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
 * Please report all bugs and problems to the author or <support@dsplinux.net>
 *
 * key: RRGPLCR (do not remove)
 *
 */

#ifndef _fb_h_
#define _fb_h_

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/wrapper.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/page.h>

#undef FB_DEBUG 
#ifdef FB_DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...)
#endif

struct tifb_par 
{
        unsigned char *p_screen_base;
        unsigned char *v_screen_base;
        unsigned char *p_palette_base;
        unsigned char *v_palette_base;
        unsigned long screen_size;
        unsigned int  palette_size;
        unsigned int  currcon;
        unsigned int  bpp;
};

static void lcd_irq_handler(int irq, void *dummy, struct pt_regs *fp);

// LCD inlines
inline void LCD_WRITE(unsigned long v, unsigned long a) 
{ 
        outl((v),(IO_ADDRESS(a))); 
}

inline unsigned long LCD_READ(unsigned long a) 
{ 
        return inl(IO_ADDRESS(a)); 
}

#endif /* _fb_h_ */

/*
 * ---------------------------------------------------------------------------
 * Local variables:
 * c-file-style: "linux"
 * End:
 */

