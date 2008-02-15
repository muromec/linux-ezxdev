/*
 * debug.c
 *
 * Routines used to debug various problems with the eventing mechanism.
 *
 * Author: MontaVista Software, Inc.
 *         jpeters@mvista.com
 *         source@mvista.com
 *
 * Copyright 2000-2002 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
 *  AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
 *  THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/config.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <linux/wait.h>
#include <linux/event.h>
#include <asm/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "event_core.h"

void
DumpClassList(void)
{
	int Hash;
	EvGroupInfo_t *Loc;
	EvClassInfo_t *HashBase;
	EvDestBase_t *EventBase;
	EvDest_t *Dest;

	printk("  Dump Class List:\n");
	for (Loc = EvGroupHead; Loc != NULL; Loc = Loc->EgiNext) {
		 for (Hash = 0; Hash < Loc->EgiHashSize; Hash++) {
			 HashBase = &Loc->EgiClassHash[Hash];

			while (HashBase->EciNext) {
				printk("    %p Hash %d Class %d Name \"%s\"\n", HashBase, Hash,
					HashBase->EciClass, HashBase->EciName);

				EventBase = HashBase->EciEventQ;
				while (EventBase->EdbNext != NULL) {
					printk("        Event %d count %d\n", EventBase->EdbEvent, EventBase->EdbUseCount);

					Dest = &EventBase->EdbDestQ;
					while (Dest->EdNext != NULL) {
						printk("            User Id %d Priority %d Callback %p UserInfo %p KernelInfo %p\n",
				       			Dest->EdID, Dest->EdPri, Dest->EdCB,
				       			Dest->EdUinfo, Dest->EdKinfo);
						Dest = Dest->EdNext;
					}

					EventBase = EventBase->EdbNext;
				}

				HashBase = HashBase->EciNext;
			}
		}
	}

	return;
}

void
DumpUsersList(void)
{
	EvKernelInfo_t *TmpUser = EvUsersHead;

	while (TmpUser) {
		printk("Dumping users list\n");
		printk("    User %d is \"%s\"\n", TmpUser->EkiID, TmpUser->EkiName);
		TmpUser = TmpUser->EkiNext;
	}
}

