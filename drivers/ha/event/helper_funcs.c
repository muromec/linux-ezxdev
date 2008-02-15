/*
 * helper_funcs.c
 *
 * Interal functions used by various elements of the event broker.
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

EvGroupInfo_t *
EvGetGroupBase(EvGroupID_t groupID)
{
	EvGroupInfo_t *EGroup = EvGroupHead;

	while (EGroup) {
		if (EGroup->EgiID == groupID) {
			return EGroup;
		}
		EGroup = EGroup->EgiNext;
	}

	return NULL;
}

int
EvLockAndGetGroup(EvGroupID_t groupID, int accessCheck,
		  EvAccess_t accessCode, unsigned long *cliFlags,
		  int mode, EvGroupInfo_t **eGroup)
{
	/* Get the group search lock. */
	read_lock_irqsave(&EvGroupLock, *cliFlags);

	/* Has the event group groupID been defined? */
	if ((*eGroup = EvGetGroupBase(groupID)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, *cliFlags);
		return -EV_ERROR_GROUP_EXIST;
	}

	/* Check access before actually trying to use the group. */
	if (accessCheck) {
		if (EvCheckAccessCode((*eGroup)->EgiAccessCode, accessCode)) {
			read_unlock_irqrestore(&EvGroupLock, *cliFlags);
			return -EV_ERROR_GROUP_ACCESS;
		}
	}

	/* Switch to locking the group. */
	if (mode == EV_LOCK_GROUP_WRITE) {
		write_lock(&(*eGroup)->EgiLock);
	} else {
		read_lock(&(*eGroup)->EgiLock);
	}

	read_unlock(&EvGroupLock);

	return EV_NOERR;
}

/*
 * Internal helper function.
 */
int
EvCheckEventClass(EvGroupInfo_t *eGroup, char *className, EvClassID_t *class)
{
	int Hash;
	EvClassInfo_t *HashBase;

	for (Hash = 0; Hash < eGroup->EgiHashSize; Hash++) {
		HashBase = &eGroup->EgiClassHash[Hash];

		while (HashBase->EciNext) {
			if (!strcmp(HashBase->EciName, className)) {
				*class = HashBase->EciClass;
				return EV_NOERR;
			}
			HashBase = HashBase->EciNext;
		}
	}

	return -EV_ERROR_CLASS_EXISTS;
}

EvClassInfo_t *
EvGetHashBase(EvGroupInfo_t *eGroup, EvClassID_t classID)
{
	int Hash = classID % eGroup->EgiHashSize;
	
	return &eGroup->EgiClassHash[Hash];
}

EvAccess_t
EvGetClassAccess(EvGroupInfo_t *eGroup, EvClassID_t classID)
{
	EvClassInfo_t *HashBase = EvGetHashBase(eGroup, classID);

	while (HashBase->EciNext) {
		if (HashBase->EciClass == classID) {
			return HashBase->EciAccessCode;
		}
		HashBase = HashBase->EciNext;
	}

	return (EvAccess_t)0;
}

/*
 * This is a helper function for internal event broker functionality and
 * is not callable from externel kernel components.  It seaches the users
 * list for the specified ID and returns a pointer to the user control
 * structure.
 */
EvKernelInfo_t *
EvCheckUser(EvUserID_t id)
{
	EvKernelInfo_t *TmpUser;

	TmpUser = EvUsersHead;

	while (TmpUser) {
		if (TmpUser->EkiID == id) {
			return TmpUser;
		}
		TmpUser = TmpUser->EkiNext;
	}

	return NULL;
}

EvPendRem_t *
EvGetPendEntry(EvGroupInfo_t *eGroup, int request,
	       int info0, int info1, int info2, int info3,
	       int dataLen, char *dataBuf)
{
	EvPendRem_t *Pend;

	Pend = eGroup->EgiPendRemoteList;

	while (Pend) {
		if ((Pend->PrRequest == request) &&
		    (Pend->PrInfo[0] == info0) &&
		    (Pend->PrInfo[1] == info1) &&
		    (Pend->PrInfo[2] == info2) &&
		    (Pend->PrInfo[3] == info3)) {
			if ((Pend->PrDataLen != 0) &&
			    (dataBuf != NULL) &&
			    (Pend->PrDataLen != 0) &&
			    (Pend->PrData != NULL) &&
			    (dataLen == Pend->PrDataLen) &&
			    !memcmp(Pend->PrData, dataBuf, dataLen)) {
				Pend->PrUseCount++;
				return Pend;
			}
		}

		Pend = Pend->PrNext;
	}

	if (eGroup->EgiPendRemoteList == NULL) {
		if ((Pend = kmalloc(sizeof(EvPendRem_t), GFP_ATOMIC)) == NULL) {
			return NULL;
		}
		eGroup->EgiPendRemoteList = Pend;
	} else {
		Pend = eGroup->EgiPendRemoteList;
		while(Pend->PrNext) {
			Pend = Pend->PrNext;
		}

		if ((Pend->PrNext =
		     kmalloc(sizeof(EvPendRem_t), GFP_ATOMIC)) == NULL) {
			return NULL;
		}

		Pend = Pend->PrNext;
	}

	Pend->PrUseCount = 1;

	Pend->PrRequest = request;
	Pend->PrInfo[0] = info0;
	Pend->PrInfo[1] = info1;
	Pend->PrInfo[2] = info2;
	Pend->PrInfo[3] = info3;

	Pend->PrDataLen = dataLen;
	Pend->PrData = dataBuf;

	init_waitqueue_head(&Pend->PrWaitQ);
	Pend->PrNext = NULL;

	return Pend;
}

EvPendRem_t *
EvFindPendEntry(EvGroupInfo_t *eGroup, int request,
		int info0, int info1, int info2, int info3,
		int dataLen, char *dataBuf)
{
	EvPendRem_t *Pend = eGroup->EgiPendRemoteList;

	if (Pend == NULL) {
		return NULL;
	}

	while (Pend) {
	       if ((Pend->PrRequest == request) &&
	           (Pend->PrInfo[0] == info0) &&
	           (Pend->PrInfo[1] == info1) &&
	           (Pend->PrInfo[2] == info2) &&
	           (Pend->PrInfo[3] == info3)) {
			if (Pend->PrDataLen == 0) {
				return Pend;
			}

			if ((dataBuf != NULL) &&
			    (Pend->PrDataLen != 0) &&
			    (Pend->PrData != NULL) &&
			    (dataLen == Pend->PrDataLen) &&
			    !memcmp(Pend->PrData, dataBuf, dataLen)) {
					return Pend;
			}
		}

		Pend = Pend->PrNext;
	}

	return NULL;
}

void
EvDelPendEntry(EvGroupInfo_t *eGroup, EvPendRem_t *pend)
{
	EvPendRem_t *DelPend;

	if (--pend->PrUseCount != 0)
		return;

	if (pend == eGroup->EgiPendRemoteList) {
		eGroup->EgiPendRemoteList = eGroup->EgiPendRemoteList->PrNext; 
		kfree(pend);
		return;
	}

	DelPend = eGroup->EgiPendRemoteList;

	while (DelPend && (DelPend->PrNext != pend)) {
		DelPend = DelPend->PrNext;
	}

	DelPend->PrNext = pend->PrNext;
	kfree(pend);

	return;
}

/*
 * This is a helper function for EventRegisterEventClass() and is not
 * usable from external kernel components.  It creates the base information
 * for a particular event class.
 */
int
EvFillInHashInfo(EvClassInfo_t *HashBase, EvAccess_t accessCode,
		 char *className, unsigned short *classID)
{
	HashBase->EciClass = *classID;
	strncpy(HashBase->EciName, className, 16);

	if ((HashBase->EciEventQ =
			kmalloc(sizeof(EvDestBase_t), GFP_ATOMIC)) == NULL) {
		return -EV_ERROR_MEM_ALLOC;
	}

	HashBase->EciEventQ->EdbNext = NULL;

	if ((HashBase->EciNext =
			kmalloc(sizeof(EvClassInfo_t), GFP_ATOMIC)) == NULL) {
		kfree(HashBase->EciEventQ);
		HashBase->EciEventQ = NULL;
		return -EV_ERROR_MEM_ALLOC;
	}

	HashBase->EciNext->EciEventQ = NULL;
	HashBase->EciNext->EciNext = NULL;
	HashBase->EciUseCount = 0;
	HashBase->EciAccessCode = accessCode;
	HashBase->EciLock = SPIN_LOCK_UNLOCKED;

	return EV_NOERR;
}

