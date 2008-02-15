/*
 * user_api.c
 *
 * User process interface routines for the eventing mechanism.
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
#include <linux/time.h>
#include <asm/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "event_core.h"

/*
 * Called from user level ioctl entry mechanism.  EvUserSubscribeEvent()
 * subscribes a user process to receive an event.
 *
 * If the userID has not been registered then -ENOENT is returned.  If
 * the event group does not exist then -EEXIST is returned.  If the event
 * class has not been registered then -EINVAL.  If the user has already
 * registered to receive the event then -EBUSY is returned.
 *
 * Otherwise the user is registered to receive this type of event and
 * a zero is returned to indicate success.
 */
int
EvUserSubscribeEvent(EvUserInfo_t* userInfo, EvReq_t *eventInfo)
{
	EvDest_t *EventDestP = NULL;
	EvDest_t *TmpDestP = NULL;
	EvDest_t *LastEventDestP = NULL;
	EvDestBase_t *EventBase;
	EvGroupInfo_t *EGroup;
	EvClassInfo_t *HashBase;
	EvClassInfo_t *ClassBase;
	unsigned long Flags;
 
	/* Check the user ID for a valid user. */
	read_lock_irqsave(&EvUsersLock, Flags);
	if (!(userInfo->EuiID) || EvCheckUser(userInfo->EuiID) == NULL) {
		read_unlock_irqrestore(&EvUsersLock, Flags);
		return -EV_ERROR_USER_EXISTS;
	}

	read_lock(&EvGroupLock);
	read_unlock(&EvUsersLock);

	/* Find the event group information. */
	if ((EGroup = EvGetGroupBase(eventInfo->ErGroupID)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	HashBase = EvGetHashBase(EGroup, eventInfo->ErClassID);

	/* Find the top level entry for this class of events. */
	if ((EventBase = EvFindEventBase(eventInfo->ErClassID, HashBase,
					 &ClassBase)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_CLASS_EXISTS;
	}

	spin_lock(&ClassBase->EciLock);
	read_unlock(&EvGroupLock);

	/* Check permissions. */
	if (EvCheckAccessCode(ClassBase->EciAccessCode,
			      eventInfo->ErAccessCode)) {
		spin_unlock_irqrestore(&ClassBase->EciLock, Flags);
		return -EV_ERROR_CLASS_ACCESS;
	}

	/* search until the event is found in this catagory or until the
	 * last blank element on the list if found.
	 */
	while (EventBase->EdbNext != NULL) {
		if (EventBase->EdbEvent == eventInfo->ErEventID) {
			EventDestP = &EventBase->EdbDestQ;
			break;
		}
		EventBase = EventBase->EdbNext;
	}

	/* If not destination pointer has been identified for a chain
	 * search then this event type has not yet been registered by anybody
	 * so fill in the last empty list element and create a new empty one
	 * to inidcate end of list.
	 */
	if (EventDestP == NULL) {
		EventBase->EdbEvent = eventInfo->ErEventID;
		EventBase->EdbUseCount = 0;

		/* Create the next empty element to indicate end of list. */
		EventBase->EdbNext = kmalloc(sizeof(EvDestBase_t), GFP_KERNEL);
		EventBase->EdbNext->EdbNext = NULL;

		EventDestP = &EventBase->EdbDestQ;
		EventDestP->EdNext = kmalloc(sizeof(EvDest_t), GFP_KERNEL);
		EventDestP->EdNext->EdNext = NULL;

		goto EvFillInRequestPacket;
	}

	/* Now search to see if this file descriptor already has registered
	 * for this event type.
	 */
	while (EventDestP->EdNext != NULL) {
		if (EventDestP->EdID == userInfo->EuiID) {
			spin_unlock_irqrestore(&ClassBase->EciLock, Flags);
			return -EV_ERROR_CLASS_BUSY;
		}
		LastEventDestP = EventDestP;
		EventDestP = EventDestP->EdNext;
	}

	/* Now record the destination and create a new empty element to 
	 * indicate end of list.
	 */

	/* Most registrations go at the end of the list. */
	if ((LastEventDestP != NULL) && (LastEventDestP->EdPri >= eventInfo->ErPri)) {
		EventDestP->EdNext = kmalloc(sizeof(EvDest_t), GFP_KERNEL);
		EventDestP->EdNext->EdNext = NULL;
		goto EvFillInRequestPacket;
	}

	/* Check the priority against the top element */
	EventDestP = &EventBase->EdbDestQ;
	if (eventInfo->ErPri <=  EventDestP->EdPri) {
		/* Priority of event places it somewhere in the middle */
		while (EventDestP->EdNext->EdPri >= eventInfo->ErPri) {
			EventDestP = EventDestP->EdNext;
		}
	}

	TmpDestP = kmalloc(sizeof(EvDest_t), GFP_KERNEL);
	TmpDestP->EdPri = EventDestP->EdPri;
	TmpDestP->EdUinfo = EventDestP->EdUinfo;
	TmpDestP->EdCB = EventDestP->EdCB;
	TmpDestP->EdID = EventDestP->EdID;
	TmpDestP->EdKinfo = EventDestP->EdKinfo;
	TmpDestP->EdNext = EventDestP->EdNext;
	EventDestP->EdNext = TmpDestP;

EvFillInRequestPacket:
	EventBase->EdbUseCount++;
	EventDestP->EdPri = eventInfo->ErPri;
	EventDestP->EdUinfo = userInfo;
	EventDestP->EdCB = NULL;
	EventDestP->EdID = userInfo->EuiID;
	EventDestP->EdKinfo = NULL;

	ClassBase->EciUseCount++;
	spin_unlock_irqrestore(&ClassBase->EciLock, Flags);
	return EV_NOERR;
}

/*
 * Called from user level ioctl entry mechanism.  EvUserUnSubscribeEvent()
 * removes a user process from the list to receive the event.
 *
 * If the userID has not been registered then -ENOENT is returned.  If
 * the event group does not exist then -EEXIST is returned.  If the event
 * class has not been registered then -EINVAL.  If the user has not
 * registered to receive the event then -ESRCH is returned.
 *
 * Otherwise the user is registered to receive this type of event and
 * a zero is returned to indicate success.
 */
int
EvUserUnSubscribeEvent(EvUserInfo_t* userInfo, EvReq_t *eventInfo)
{
	EvDest_t *EventDestP = NULL;
	EvDest_t *FreeDest = NULL;
	EvDestBase_t *EventBase;
	EvDestBase_t *LastEventBase = NULL;
	EvGroupInfo_t *EGroup;
	EvClassInfo_t *HashBase;
	EvClassInfo_t *ClassBase;
	unsigned long Flags;

	/* Check the user ID. */
	read_lock_irqsave(&EvUsersLock, Flags);
	if (!(userInfo->EuiID) || EvCheckUser(userInfo->EuiID) == NULL) {
		read_unlock_irqrestore(&EvUsersLock, Flags);
		return -EV_ERROR_USER_EXISTS;
	}

	read_lock(&EvGroupLock);
	read_unlock(&EvUsersLock);

	/* Find the event group information. */
	if ((EGroup = EvGetGroupBase(eventInfo->ErGroupID)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	HashBase = EvGetHashBase(EGroup, eventInfo->ErClassID);

	/* Find the top level entry for this class of events. */
	if ((EventBase = EvFindEventBase(eventInfo->ErClassID,
					 HashBase, &ClassBase)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_CLASS_EXISTS;
	}

	spin_lock(&ClassBase->EciLock);
	read_unlock(&EvGroupLock);

	/* search until the event is found in this catagory or until the
	 * last blank element on the list if found.
	 */
	while (EventBase->EdbNext != NULL) {
		if (EventBase->EdbEvent == eventInfo->ErEventID) {
			EventDestP = &EventBase->EdbDestQ;
			break;
		}

		LastEventBase = EventBase;
		EventBase = EventBase->EdbNext;
	}

	/* If event type not found then the user process was obviously not
	 * registered for this event.
	 */
	if (EventDestP == NULL) {
		spin_unlock_irqrestore(&ClassBase->EciLock, Flags);
		return -EV_ERROR_CLASS_NO_SUB;
	}

	/* Check the top element first */
	if (EventDestP->EdID == userInfo->EuiID) {
		EventDestP->EdUinfo = EventDestP->EdNext->EdUinfo;
		EventDestP->EdCB = EventDestP->EdNext->EdCB;
		EventDestP->EdID = EventDestP->EdNext->EdID;
		EventDestP->EdKinfo = EventDestP->EdNext->EdKinfo;
		FreeDest = EventDestP->EdNext;
		EventDestP->EdNext = EventDestP->EdNext->EdNext;

		goto EvUnRegFreeBase;
	}
		
	/* Allways search one ahead to help with single link list removal. */
	while (EventDestP->EdNext->EdNext != NULL) {
		if (EventDestP->EdNext->EdID == userInfo->EuiID) {
			FreeDest = EventDestP->EdNext;
			EventDestP->EdNext = EventDestP->EdNext->EdNext;

			goto EvUnRegFreeBase;
		}
		EventDestP = EventDestP->EdNext;
	}

	/* Entry not found in list. */
	spin_unlock_irqrestore(&ClassBase->EciLock, Flags);
	return -EV_ERROR_CLASS_NO_SUB;

EvUnRegFreeBase:
	EventBase->EdbUseCount--;

	if (EventBase->EdbUseCount == 0) {
		/* Nobody is registered to receive this event. */
		if (LastEventBase == NULL) {
			/* Free the top element */
			EventBase->EdbEvent = EventBase->EdbNext->EdbEvent;
			EventBase->EdbUseCount = EventBase->EdbNext->EdbUseCount;
			EventBase->EdbDestQ = EventBase->EdbNext->EdbDestQ;
			LastEventBase = EventBase->EdbNext;
			EventBase->EdbNext = LastEventBase->EdbNext;
			kfree(LastEventBase);
		} else {
			LastEventBase->EdbNext = EventBase->EdbNext;
			kfree(EventBase);
		}
	}

	ClassBase->EciUseCount--;
	kfree(FreeDest);

	spin_unlock_irqrestore(&ClassBase->EciLock, Flags);
	return EV_NOERR;
}

int
EvUserSubscribeGroupEvents(EvUserInfo_t *userInfo, EvReq_t *eventInfo)
{
	EvGroupInfo_t *EGroup;
	unsigned long Flags;

	read_lock_irqsave(&EvUsersLock, Flags);
	if (!(userInfo->EuiID) || EvCheckUser(userInfo->EuiID) == NULL) {
		read_unlock_irqrestore(&EvUsersLock, Flags);
		return -EV_ERROR_USER_EXISTS;
	}

	read_lock(&EvGroupLock);
	read_unlock(&EvUsersLock);

	if ((EGroup = EvGetGroupBase(eventInfo->ErGroupID)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	/* Check the access code for the group. */
	if (EvCheckAccessCode(EGroup->EgiAccessCode, eventInfo->ErAccessCode)) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_ACCESS;
	}

	/* Check that there are no current control processes. */
	if (EGroup->EgiGroupDest.EdID != 0) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_BUSY;
	}

	EGroup->EgiGroupDest.EdPri = 0;		/* Not used. */
	EGroup->EgiGroupDest.EdID =  userInfo->EuiID;
	EGroup->EgiGroupDest.EdUinfo = userInfo;
	EGroup->EgiGroupDest.EdCB = NULL;
	EGroup->EgiGroupDest.EdKinfo = NULL;

	EGroup->EgiUseCount++;

	read_unlock_irqrestore(&EvGroupLock, Flags);
	return EV_NOERR;
}
