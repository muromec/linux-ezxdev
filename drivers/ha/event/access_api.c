/*
 * access_api.c
 *
 * Interface routines for the eventing mechasism made available to
 * other kernel code.
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

#define MODVERSIONS

/*
 * TBD in this file.  These functions need to echo the changes to
 * all other event group member.
 */


int
EvSetGroupAccessCode(EvGroupID_t groupID,
		     EvAccess_t oldCode, EvAccess_t newCode)
{
	EvGroupInfo_t *EGroup;
	unsigned long Flags;

	read_lock_irqsave(&EvGroupLock, Flags);
	if ((EGroup = EvGetGroupBase(groupID)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	if (EvCheckAccessCode(EGroup->EgiAccessCode, oldCode)) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_ACCESS;
	}

	EGroup->EgiAccessCode = newCode;
	write_unlock_irqrestore(&EvGroupLock, Flags);
	return EV_NOERR;
}

int
EvSetClassAccessCode(EvGroupID_t groupID, EvClassID_t classID,
		     EvAccess_t oldCode, EvAccess_t newCode)
{
	EvGroupInfo_t *EGroup;
	EvClassInfo_t *HashBase;
	EvClassInfo_t *ClassBase;
	EvDestBase_t *EventBase;
	unsigned long Flags;

	read_lock_irqsave(&EvGroupLock, Flags);
	if ((EGroup = EvGetGroupBase(groupID)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	HashBase = EvGetHashBase(EGroup, classID);

	spin_lock(&HashBase->EciLock);
	read_unlock(&EvGroupLock);

	if ((EventBase = EvFindEventBase(classID,HashBase,&ClassBase))==NULL) {
		spin_unlock_irqrestore(&HashBase->EciLock, Flags);
		return -EV_ERROR_CLASS_EXISTS;
	}

	if (EvCheckAccessCode(ClassBase->EciAccessCode, oldCode)) {
		spin_unlock_irqrestore(&HashBase->EciLock, Flags);
		return -EV_ERROR_CLASS_ACCESS;
	}

	ClassBase->EciAccessCode = newCode;
	spin_unlock_irqrestore(&HashBase->EciLock, Flags);
	return EV_NOERR;
}

int
EvSetLocalClassAccessCode(EvClassID_t classID,
		          EvAccess_t oldCode, EvAccess_t newCode)
{
	return EvSetClassAccessCode(EG_LOCAL, classID, oldCode, newCode);
}

int
EvCheckAccessCode(EvAccess_t code1, EvAccess_t code2)
{       
	if ((code1 != code2) && !capable(CAP_SYS_ADMIN)) {
		return -EPERM;
	}

        return 0;
}
