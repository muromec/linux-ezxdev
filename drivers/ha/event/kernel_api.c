/*
 * kernel_api.c
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

int EvCheckEventClass(EvGroupInfo_t *, char *, unsigned short *);

EvKernelInfo_t *EvUsersHead = NULL;
unsigned int KernelNextId = 0;   /* Zero is the local event control id. */
rwlock_t EvUsersLock = RW_LOCK_UNLOCKED;
unsigned int EvSysQLimit = 0;

/*
 * This function identifies an event user to the event broker.  It is
 * passed to most other calls to the system to identify the sender or
 * one of the recipients of an event.  A User is identified by an up
 * to 16 character string.  This string must be unique on the
 * system it is called on (intra system) but may be the same on each
 * system participating in a intersystem event group set.
 *
 * The userName argument is passed in and the system returns the 
 * unique assinged user identification in the userID parameter.
 *
 * If the user defined by the userName string is already registered 
 * -EEXIST will be returned as an error.  If the kernel could not
 * allocate data space the -ENOSPC will be returned.
 *
 * In successful completion a zero is returned with the userID
 * field containing the user identification number.
 *
 * This is sort of equivelent to the user level open call.
 */
int
EvRegisterUser(char *userName, EvUserID_t *userID)
{
	EvKernelInfo_t *TmpUser;
	unsigned long Flags;

	write_lock_irqsave(&EvUsersLock, Flags);

	/* Check to see if this user ID is already know. */
	TmpUser = EvUsersHead;
	while (TmpUser) {
		if (!strncmp(TmpUser->EkiName, userName, 16)) {
			write_unlock_irqrestore(&EvUsersLock, Flags);
			return -EV_ERROR_USER_EXISTS;
		}
		TmpUser = TmpUser->EkiNext;
	}

	/* Allocate a new info structure to handle this user ID. */
	if (EvUsersHead == NULL) {
		if ((EvUsersHead = kmalloc(sizeof(EvKernelInfo_t),
							GFP_KERNEL)) == NULL) {
			/* System is in immenent danger of dying. */
			write_unlock_irqrestore(&EvUsersLock, Flags);
			return -EV_ERROR_MEM_ALLOC;
		}
		TmpUser = EvUsersHead;
	} else {
		TmpUser = EvUsersHead;

		while (TmpUser->EkiNext) {
			TmpUser = TmpUser->EkiNext;
		}

		if ((TmpUser->EkiNext = kmalloc(sizeof(EvKernelInfo_t),
							GFP_KERNEL)) == NULL) {
			/* System is in immenent danger of dying. */
			write_unlock_irqrestore(&EvUsersLock, Flags);
			return -EV_ERROR_MEM_ALLOC;
		}
		TmpUser = TmpUser->EkiNext;
	}

	/* Fill in the user data. */
	TmpUser->EkiID = KernelNextId++;
	strncpy(TmpUser->EkiName, userName, 16);
	init_waitqueue_head(&TmpUser->EkiWaitQ);
	TmpUser->EkiHead = NULL;
	TmpUser->EkiTail = NULL;
	TmpUser->EkiNext = NULL;
	TmpUser->EkiLock = SPIN_LOCK_UNLOCKED;
	TmpUser->EkiQCount = 0;
	TmpUser->EkiQLimit = 0;

	write_unlock_irqrestore(&EvUsersLock, Flags);

	/* Return the allocated user ID. */
	*userID = TmpUser->EkiID;
	return EV_NOERR;
}

/*
 * The EvUnRegisterUser() removes a users identification from the
 * event broker.  If there are any pending events or event registerations
 * for that user they are deleted.
 *
 * If the ID passed in is not from a current registeration -ESRCH is
 * returned.  Otherwise zero is returned to indicate success.
 */
int
EvUnRegisterUser(EvUserID_t userID)
{
	EvKernelInfo_t *TmpUser;
	EvKernelInfo_t *FreeUser;
	unsigned long Flags;

	write_lock_irqsave(&EvUsersLock, Flags);

	/* Find the info data structure controlling this user. */
	if (EvUsersHead == NULL) {
		write_unlock_irqrestore(&EvUsersLock, Flags);
		return -EV_ERROR_USER_EXISTS;
	}

	if (EvUsersHead->EkiID == userID) {
		FreeUser = EvUsersHead;
		EvUsersHead = EvUsersHead->EkiNext;

		spin_lock(&FreeUser->EkiLock);
		write_unlock(&EvUsersLock);
		/* Delete pending events for this user. */
		EvUnSubscribeAllEvents(userID);
		spin_unlock_irqrestore(&FreeUser->EkiLock, Flags);

		kfree(FreeUser);

		return EV_NOERR;
	}

	TmpUser = EvUsersHead;

	while(TmpUser->EkiNext) {
		if (TmpUser->EkiNext->EkiID == userID) {
			FreeUser = TmpUser->EkiNext;
			TmpUser->EkiNext = FreeUser->EkiNext;

			spin_lock(&FreeUser->EkiLock);
			write_unlock(&EvUsersLock);
			/* Delete pending events for this user. */
			EvUnSubscribeAllEvents(userID);
			spin_unlock_irqrestore(&FreeUser->EkiLock, Flags);

			kfree(FreeUser);

			return EV_NOERR;
		}
		TmpUser = TmpUser->EkiNext;
	}

	/* Invalid user ID so return error. */
	write_unlock_irqrestore(&EvUsersLock, Flags);
	return -EV_ERROR_USER_EXISTS;
}

/*
 * Set maximum number of queued events for a userID.
 */
int
EvSetQLimit(EvUserID_t userID, unsigned int newQLimit)
{
	EvKernelInfo_t *TmpUser;
	unsigned long Flags;

	write_lock_irqsave(&EvUsersLock, Flags);

	TmpUser = EvUsersHead;

	while (TmpUser) {
		if (TmpUser->EkiID == userID) {
			TmpUser->EkiQLimit = newQLimit;
			write_unlock_irqrestore(&EvUsersLock, Flags);
			return EV_NOERR;
		}
		TmpUser = TmpUser->EkiNext;
	}

	write_unlock_irqrestore(&EvUsersLock, Flags);
	return -EV_ERROR_USER_EXISTS;
}

/*
 * Set system wide value for maximum number of queued events for a userID.
 */
int
EvSetSysQLimit(unsigned int newQLimit)
{
	EvSysQLimit = newQLimit;

	return EV_NOERR;
}

/*
 * EvGetUserID() returns the user ID associated with the name string
 * passed into the register user function.
 *
 * If the group ID identifies a group of events that are local or this
 * system is the master controler or the member ID is this system  then it
 * searches the local systems users for the specified name.  Otherwise
 * a request for the information is sent to the remote system.
 *
 * If groupID does not specify a known group then -EEXIST is returned.  IF
 * userName is not found in the correct list the -ESRCH is returned.
 * Otherwise a 0 is returned and userID contains the associated unique
 * user ID.
 */
int
EvGetUserID(EvGroupID_t groupID, EvGroupID_t memberID,
	     char *userName, EvUserID_t *userID)
{
	EvKernelInfo_t *TmpUser;
	EvGroupInfo_t *EGroup;
	unsigned long Flags;
	int RetVal;

	/* Get the group info pointer and return with it locked. */
	if ((RetVal = EvLockAndGetGroup(groupID, EV_ACCESS_IGNORE,
					EAC_NONE, &Flags,
					EV_LOCK_GROUP_WRITE, &EGroup))) {
		return RetVal;
	}

	/* If the memberID passed in does not match this system then send
	 * a message to the master to retieve and return it.
	 */
	if ((EGroup->EgiType != EG_LOCAL)&&(EGroup->EgiMemberID != memberID)) {
		RetVal = EvSendGetUserID(groupID, memberID, userName, userID);
		write_unlock_irqrestore(&EGroup->EgiLock, Flags);
		return RetVal;
	}

	/* Get the user ID from this system. */
	read_lock(&EvUsersLock);
	write_unlock(&EGroup->EgiLock);

	TmpUser = EvUsersHead;

	while (TmpUser) {
		if (!strcmp(TmpUser->EkiName, userName)) {
			*userID = TmpUser->EkiID;
			read_unlock_irqrestore(&EvUsersLock, Flags);
			return EV_NOERR;
		}
		TmpUser = TmpUser->EkiNext;
	}

	read_unlock_irqrestore(&EvUsersLock, Flags);
	return -EV_ERROR_USER_EXISTS;
}

int
EvGetLocalUserID(char *userName, EvUserID_t *userID)
{
	return EvGetUserID(EG_LOCAL, EG_LOCAL, userName, userID);
}

/*
 * EventGetUserName() returns the user name string associated with the
 * Ruser ID returned from the register user function.
 *
 * If the group ID identifies a group of events that are local or this
 * system is the master controler or the member ID is this system  then it
 * searches the local systems users for the specified ID.  Otherwise
 * a request for the information is sent to the remote system.
 *
 * If groupID does not specify a known group then -EEXIST is returned.  IF
 * userName is not found in the correct list the -ESRCH is returned.
 * Otherwise a 0 is returned and userID contains the associated unique
 * user ID.
 */
int
EvGetUserName(EvGroupID_t groupID, EvGroupID_t memberID,
		 EvUserID_t userID, char *userName)
{
	EvGroupInfo_t *EGroup;
	EvKernelInfo_t *TmpUser;
	unsigned long Flags;
	int RetVal;

	/* Get the group info pointer and return with it locked. */
	if ((RetVal = EvLockAndGetGroup(groupID, EV_ACCESS_IGNORE,
					EAC_NONE, &Flags,
					EV_LOCK_GROUP_WRITE, &EGroup))) {
		return RetVal;
	}

	if ((EGroup->EgiType == EG_MEMBER)&&(EGroup->EgiMemberID != memberID)) {
		RetVal = EvSendGetUserName(EGroup, memberID, userID, userName, &Flags);
		write_unlock_irqrestore(&EGroup->EgiLock, Flags);
		return RetVal;
	}

	read_lock(&EvUsersLock);
	write_unlock(&EGroup->EgiLock);

	TmpUser = EvUsersHead;
	while (TmpUser) {
		if (TmpUser->EkiID == userID) {
			strncpy(userName, TmpUser->EkiName, 16);
			read_unlock_irqrestore(&EvUsersLock, Flags);
			return 0;
		}
		TmpUser = TmpUser->EkiNext;
	}

	read_unlock_irqrestore(&EvUsersLock, Flags);
	return -EV_ERROR_USER_EXISTS;
}

int
EvGetLocalUserName(EvUserID_t userID, char *userName)
{
	return EvGetUserName(EG_LOCAL, EG_LOCAL, userID, userName);
}

/*
 * The EvRegisterEventClass() function registers the className string as the
 * identifier of an event class and returns a unique token to identify that
 * class.  The class is created as a part of an event group.
 *
 * If the event group is local then the event class is created as a part
 * of the local events and can be used only on the registering system.  If
 * this system is the master for an event group then a local identity is
 * created on this system and notification is sent to remote systems to
 * inidcate this event has been created.  If this event group is a member
 * on this system then the master is told to register this event class and
 * return the class ID and then a local copy of the registeration is made.
 *
 * If the event group defined by groupID does not exist then -EEXIST is
 * returned and no class is registered.  If the event class defined by
 * className already exists -EBUSY is returned and not class is registered.
 * returned.  If the kernel could not allocate enough memory for the
 * controling data structures -ENOSPC is returned and not class is
 * registered.  Otherwise the class identified by className is registered,
 * a unique ID is allocated and placed in the class parameter and a
 * zero is returned to indicate success.
 */
int
EvRegisterEventClass(EvGroupID_t groupID, char *className,
		     EvAccess_t groupAccessCode, EvAccess_t classAccessCode,
		     EvClassID_t *classID)
{
	EvGroupInfo_t *EGroup;
	EvClassInfo_t *HashBase;
	unsigned long Flags;
	int RetVal;

	/* Get the group info pointer and return with it locked. */
	if ((RetVal = EvLockAndGetGroup(groupID, EV_ACCESS_CHECK,
					groupAccessCode, &Flags,
					EV_LOCK_GROUP_WRITE, &EGroup))) {
		return RetVal;
	}

	/* Check to see if the class already exists */
	if (EvCheckEventClass(EGroup, className, classID) > -1) {
		write_unlock_irqrestore(&EGroup->EgiLock, Flags);
		return -EV_ERROR_CLASS_EXISTS;
	}

	/* If the event class is not a local one then send a register to
	 * the remote master.  The remote master returns the class ID.
	 */
	if ((EGroup->EgiType == EG_MEMBER) && EGroup->EgiNextClass) {
		if ((RetVal = EvSendRegEventClass(EGroup, className,
						  groupAccessCode,
						  classAccessCode,
						  classID, &Flags)) < 0) {
			write_unlock_irqrestore(&EGroup->EgiLock, Flags);
			return RetVal;
		}
	} else {
		/* Class is locally assigned */
		*classID = EGroup->EgiNextClass++;

		/* If this is an event master then send class create event. */
		if (EGroup->EgiType == EG_MASTER) {
			if ((RetVal = EvSendRegClassToMembers(EGroup, className,
					        groupAccessCode,
						classAccessCode, *classID))) {
			write_unlock_irqrestore(&EGroup->EgiLock, Flags);
			return RetVal;
			}
		}
	}

	HashBase = EvGetHashBase(EGroup, *classID);

	while (HashBase->EciNext != NULL) {
		HashBase = HashBase->EciNext;
	}

	RetVal=EvFillInHashInfo(HashBase, classAccessCode, className, classID);

	EGroup->EgiUseCount++;

	write_unlock_irqrestore(&EGroup->EgiLock, Flags);
	return RetVal;
}

int
EvRegisterLocalEventClass(char *className, EvAccess_t classAccessCode,
		          EvClassID_t *classID)
{
	return EvRegisterEventClass(EG_LOCAL, className, EAC_NONE,
		     		    classAccessCode, classID);
}

int
EvInternalUnRegisterEventClass(EvGroupInfo_t *eGroup, EvClassID_t classID,
			    EvAccess_t accessCode)
{
	EvClassInfo_t *HashBase;
	EvClassInfo_t* HashDel;

	HashBase = EvGetHashBase(eGroup, classID);

	/* Does this hash have any classes assigned? */
	if (HashBase->EciNext == NULL) {
		return -EV_ERROR_CLASS_EXISTS;
	}

	/* Is the top element his class? */
	if (HashBase->EciClass == classID) {
		/* If subscribers use count is not zero then return error. */
		if (HashBase->EciUseCount != 0) {
			return -EV_ERROR_CLASS_BUSY;
		}

		if (EvCheckAccessCode(HashBase->EciAccessCode, accessCode)) {
			return -EV_ERROR_CLASS_ACCESS;
		}

		HashDel = HashBase->EciNext;

		HashBase->EciClass = HashDel->EciClass;
		strncpy(HashBase->EciName, HashDel->EciName, 16);
		HashBase->EciEventQ = HashDel->EciEventQ;
		HashBase->EciNext = HashDel->EciNext;

		goto EvDelHashInfo;
	}

	HashDel = HashBase->EciNext;

	while(HashDel->EciNext != NULL) {
		if (HashDel->EciClass == classID) {
			break;
		}
		HashBase = HashDel;
		HashDel = HashDel->EciNext;
	}

	if (HashDel->EciNext == NULL) {
		return -EV_ERROR_CLASS_EXISTS;
	}

	if (HashDel->EciUseCount != 0) {
		return -EV_ERROR_CLASS_BUSY;
	}

	if (EvCheckAccessCode(HashDel->EciAccessCode, accessCode)) {
		return -EV_ERROR_CLASS_ACCESS;
	}

	HashBase->EciNext = HashDel->EciNext;
	
EvDelHashInfo:
	/* Now reaquire the Group lock decrement the use count and send 
	 * remote packets.
	 */
	eGroup->EgiUseCount--;

	/* If this is a master send remove packets to all members. */
	if (eGroup->EgiType == EG_MASTER) {
		EvSendUnRegClassToMembers(eGroup, classID, accessCode);
	}

	if (HashDel->EciEventQ) {
		kfree(HashDel->EciEventQ);
	}

	kfree(HashDel);

	return EV_NOERR;
}

/*
 * EvUnRegisterEventClass() removes an event class from the list
 * of known event classes.  If this is a local event group then
 * it is removed only locally.  If this is a member of an event
 * group then a notices is sent to the master to remove the event
 * class.  If the master indicates it is OK then the class is removed
 * locally.
 *
 * If this is an event master and every thing is OK locally then
 * the event class is removes locally and the members are informed
 * to remove it also.
 */
int
EvUnRegisterEventClass(EvGroupID_t groupID, EvClassID_t classID,
		       EvAccess_t accessCode)
{
	EvGroupInfo_t *EGroup;
	unsigned long Flags;
	int RetVal;

	/* Get the group info pointer and return with it locked. */
	if ((RetVal = EvLockAndGetGroup(groupID, EV_ACCESS_IGNORE,
					EAC_NONE, &Flags,
					EV_LOCK_GROUP_WRITE, &EGroup))) {
		return RetVal;
	}

	/* If this is a Member send remove request to master. */
	if (EGroup->EgiType == EG_MEMBER) {
		if ((RetVal = EvSendUnRegEventClass(EGroup, accessCode,
						    classID, &Flags)) < 0) {
			write_unlock_irqrestore(&EGroup->EgiLock, Flags);
			return RetVal;
		}
	}

	RetVal =  EvInternalUnRegisterEventClass(EGroup, classID, accessCode);
	write_unlock_irqrestore(&EGroup->EgiLock, Flags);
	return RetVal;
}

int
EvUnRegisterLocalEventClass(EvClassID_t classID, EvAccess_t accessCode)
{
	return EvUnRegisterEventClass(EG_LOCAL, classID, accessCode);
}

/*
 * EvGetEventClassID() finds the ID associated with an event class name
 * passed in className.  This function works only locally because
 * all event classes should be known locally.
 *
 * If the event group cound not be found -EEXIST will be returned.  If
 * the class identified by className is not registered the -ESRCH will
 * be returned.  Else "class" will be filled with the unique class
 * ID assiged to className and a zero will be returned to indicates 
 * success.
 */
int
EvGetEventClassID(EvGroupID_t groupID, char *className, EvClassID_t *classID)
{
	EvGroupInfo_t *EGroup;
	unsigned long Flags;
	int RetVal;

	/* Get the group info pointer and return with it locked. */
	if ((RetVal = EvLockAndGetGroup(groupID, EV_ACCESS_IGNORE,
					EAC_NONE, &Flags,
					EV_LOCK_GROUP_READ, &EGroup))) {
		return RetVal;
	}

	RetVal = EvCheckEventClass(EGroup, className, classID);

	read_unlock_irqrestore(&EGroup->EgiLock, Flags);
	return RetVal;
}

int
EvGetLocalEventClassID(char *className, EvClassID_t *classID)
{
	return EvGetEventClassID(EG_LOCAL, className, classID);
}

/*
 * EvGetEventClassName() finds the class name string  associated with an event
 * class ID.  This function works only locally because all event classes
 * should be known locally.
 *
 * If the event group cound not be found -EEXIST will be returned.  If
 * the class identified by className is not registered the -ESRCH will
 * be returned.  Else "class" will be filled with the unique class
 * ID assiged to className and a zero will be returned to indicates 
 * success.
 */
int
EvGetEventClassName(EvGroupID_t groupID, EvClassID_t classID, char *className)
{
	EvGroupInfo_t *EGroup;
	EvClassInfo_t *HashBase;
	unsigned long Flags;
	int RetVal;

	/* Get the group info pointer and return with it locked. */
	if ((RetVal = EvLockAndGetGroup(groupID, EV_ACCESS_IGNORE,
					EAC_NONE, &Flags,
					EV_LOCK_GROUP_READ, &EGroup))) {
		return RetVal;
	}

	HashBase = EvGetHashBase(EGroup, classID);

	while (HashBase->EciNext) {
		if (HashBase->EciClass == classID) {
			strncpy(className, HashBase->EciName, 16);
			read_unlock_irqrestore(&EGroup->EgiLock, Flags);
			return EV_NOERR;
		}
		HashBase = HashBase->EciNext;
	}

	read_unlock_irqrestore(&EGroup->EgiLock, Flags);
	return -EV_ERROR_CLASS_EXISTS;
}

int
EvGetLocalEventClassName(EvClassID_t classID, char *className)
{
	return EvGetEventClassName(EG_LOCAL, classID, className);
}

/*
 * EvSubscribeEvent() subscribes a kernel component to recieve
 * an event.  If the event call back function kernelCB is specified
 * events will be delivered by calling this function.  Otherwise 
 * events will be placed on an internal event queue and the queue must
 * be polled using the EvGetEvent function.
 *
 * This function subscribes to an event localy.  It does not attempt to
 * inform a remote master that a subscriber has been attached.  This
 * should be added in a later version of the function.
 */
int
EvSubscribeEvent(EvUserID_t userID, EvPri_t pri,
		 EvGroupID_t groupID, EvClassID_t classID,
		 EvEventID_t eventID, EvAccess_t accessCode,
		 int (*kernelCB)(EvUserID_t, EvPri_t, EvGroupID_t,
				 EvGroupID_t, EvClassID_t, EvEventID_t,
				 int, int, int, int, int, void *))
{
	EvDest_t *EventDestP = NULL;
	EvDest_t *TmpDestP = NULL;
	EvDest_t *LastEventDestP = NULL;
	EvDestBase_t *EventBase;
	EvKernelInfo_t *EventUser;
	EvGroupInfo_t *EGroup;
	EvClassInfo_t *HashBase;
	EvClassInfo_t *ClassBase;
	unsigned long Flags;

	/* Check the user ID for validity. */
	read_lock_irqsave(&EvUsersLock, Flags);
	if ((EventUser = EvCheckUser(userID)) == NULL) {
		read_unlock_irqrestore(&EvUsersLock, Flags);
		return -EV_ERROR_USER_EXISTS;
	}

	/* Assume the Event user returned status will no change during
	 * the life of this request.
	 */
	read_lock(&EvGroupLock);
	read_unlock(&EvUsersLock);

	/* Get the event group pointer. */
	if ((EGroup = EvGetGroupBase(groupID)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	HashBase = EvGetHashBase(EGroup, classID);

	/* Find the top level entry for this class of events. */
	if ((EventBase = EvFindEventBase(classID,HashBase,&ClassBase)) == NULL){
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_CLASS_EXISTS;
	}

	spin_lock(&ClassBase->EciLock);
	read_unlock(&EvGroupLock);

	/* Check permissions. */
	if (EvCheckAccessCode(ClassBase->EciAccessCode, accessCode)) {
		spin_unlock_irqrestore(&ClassBase->EciLock, Flags);
		return -EV_ERROR_CLASS_ACCESS;
	}

	/* search until the event is found in this catagory or until the
	 * last blank element on the list if found.
	 */
	while (EventBase->EdbNext != NULL) {
		if (EventBase->EdbEvent == eventID) {
			EventDestP = &EventBase->EdbDestQ;
			break;
		}
		EventBase = EventBase->EdbNext;
	}

	/* If no destination pointer has been identified for a chain
	 * search then this event type has not yet been registered by anybody
	 * so fill in the last empty list element and create a new empty one
	 * to inidcate end of list.
	 */
	if (EventDestP == NULL) {
		EventBase->EdbEvent = eventID;
		EventBase->EdbUseCount = 0;

		/* Create the next empty element to indicate end of list. */
		if ((EventBase->EdbNext = kmalloc(sizeof(EvDestBase_t),
							GFP_ATOMIC)) == NULL) {
			spin_unlock_irqrestore(&ClassBase->EciLock, Flags);
			return -EV_ERROR_MEM_ALLOC;
		}

		EventBase->EdbNext->EdbNext = NULL;

		EventDestP = &EventBase->EdbDestQ;
		EventDestP->EdNext = kmalloc(sizeof(EvDest_t), GFP_ATOMIC);
		EventDestP->EdNext->EdNext = NULL;

		 goto EvFillInKernelRequestPacket;
	}

	/* Now search to see if this file descriptor already has registered
	 * for this event type.
	 */
	while (EventDestP->EdNext != NULL) {
		if (EventDestP->EdID == userID) {
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
	if ((LastEventDestP != NULL) && (LastEventDestP->EdPri >= pri)) {
		if ((EventDestP->EdNext = kmalloc(sizeof(EvDest_t),
							GFP_ATOMIC)) == NULL) {
			spin_unlock_irqrestore(&ClassBase->EciLock, Flags);
			return -EV_ERROR_MEM_ALLOC;
		}

		EventDestP->EdNext->EdNext = NULL;
		goto EvFillInKernelRequestPacket;
	}

	EventDestP = &EventBase->EdbDestQ;

	/* Check the priority against the top element */
	if (EventDestP->EdPri >= pri) {
		/* Priority of event places it somewhere in the middle */
		while (EventDestP->EdNext->EdPri >= pri) {
			EventDestP = EventDestP->EdNext;
		}
	}

	if ((TmpDestP = kmalloc(sizeof(EvDest_t), GFP_ATOMIC)) == NULL) {
		spin_unlock_irqrestore(&ClassBase->EciLock, Flags);
		return -EV_ERROR_MEM_ALLOC;
	}

	TmpDestP->EdPri = EventDestP->EdPri;
	TmpDestP->EdUinfo = EventDestP->EdUinfo;
	TmpDestP->EdCB = EventDestP->EdCB;
	TmpDestP->EdID = EventDestP->EdID;
	TmpDestP->EdKinfo = EventDestP->EdKinfo;
	TmpDestP->EdNext = EventDestP->EdNext;
	EventDestP->EdNext = TmpDestP;

EvFillInKernelRequestPacket:
	EventBase->EdbUseCount++;
	EventDestP->EdPri = pri;
	EventDestP->EdUinfo = NULL;
	EventDestP->EdID = userID;
	EventDestP->EdCB = kernelCB;

	EventDestP->EdKinfo = EventUser;

	ClassBase->EciUseCount++;

	spin_unlock_irqrestore(&ClassBase->EciLock, Flags);
	return EV_NOERR;
}

int
EvLocalSubscribeEvent(EvUserID_t userID, EvPri_t pri,
		      EvClassID_t classID, EvEventID_t eventID,
		      EvAccess_t accessCode,
		      int (*kernelCB)(EvUserID_t, EvPri_t, EvGroupID_t,
				      EvGroupID_t, EvClassID_t, EvEventID_t,
				      int, int, int, int, int, void *))
{
	return EvSubscribeEvent(userID, pri, EG_LOCAL, classID, eventID,
			        accessCode, kernelCB);
}

/*
 * EvUnSubscribeEvent() removes the user id from the list of subscribers
 * waiting for the particular event to occur.
 *
 * If the user ID does not indicate a currently registered user then
 * -ENOENT is returned.  If the event group is not a currently registered
 * group then -EEXIST is returned.  If the class in not found in the this
 * of currently know classes then -EINVAL is returned.  If the user is
 * not currently subscribed to recieve the event then -EBUSY is returned.
 *
 * Otherwise the subscription is removed and a zero is returned to
 * indicate susccess.
 */
int
EvUnSubscribeEvent(EvUserID_t userID, EvGroupID_t groupID,
		   EvClassID_t classID, EvEventID_t eventID)
{
	EvDest_t *EventDestP = NULL;
	EvDest_t *FreeDest = NULL;
	EvDestBase_t *EventBase;
	EvDestBase_t *LastEventBase = NULL;
	EvKernelInfo_t *EventUser;
	EvGroupInfo_t *EGroup;
	EvClassInfo_t *HashBase;
	EvClassInfo_t *ClassBase;
	unsigned long Flags;

	/* Check the user id. */
	read_lock_irqsave(&EvUsersLock, Flags);
	if ((EventUser = EvCheckUser(userID)) == NULL) {
		read_unlock_irqrestore(&EvUsersLock, Flags);
		return -EV_ERROR_USER_EXISTS;
	}

	read_lock(&EvGroupLock);
	read_unlock(&EvUsersLock);

	/* Get the base event group information. */
	if ((EGroup = EvGetGroupBase(groupID)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	HashBase = EvGetHashBase(EGroup, classID);

	/* Find the top level entry for this class of events. */
	if ((EventBase = EvFindEventBase(classID,HashBase,&ClassBase)) == NULL){
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_CLASS_EXISTS;
	}

	spin_lock(&ClassBase->EciLock);
	read_unlock(&EvGroupLock);

	/* search until the event is found in this catagory or until the
	 * last blank element on the list if found.
	 */
	while (EventBase->EdbNext != NULL) {
		if (EventBase->EdbEvent == eventID) {
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

	if (EventDestP->EdID == userID) {
		if (EventDestP->EdNext == NULL) {
			/* This is the only element on the list so it
			 * will be removed below. */
			goto EvUnRegFreeBase;
		}

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
		if (EventDestP->EdNext->EdID == userID) {
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
EvLocalUnSubscribeEvent(EvUserID_t userID,
			EvClassID_t classID, EvEventID_t eventID)
{
	return EvUnSubscribeEvent(userID, EG_LOCAL, classID, eventID);
}

/*
 * EvGetEvent() retrieves an event from the event queue assigned to
 * the user id.
 *
 * If the user ID passed in is not a registed user then -ENOENT is
 * returned.  If there are no pending events and waitflag is not
 * set then -EAGAIN is returned.  If there are no events and waitflag
 * is set then the kernel thread associated with the user ID is
 * put to sleep.
 *
 * Otherwise the event packet pointer is placed in packet and a zero
 * is returned to indicate success.
 */
int
EvGetEvent(EvUserID_t userID, int waitFlag, EvPacket_t **packet)
{
	EvPacket_t *TmpPacket;
	EvKernelInfo_t *EventUser;
	unsigned long Flags;

	read_lock_irqsave(&EvUsersLock, Flags);

	/* Make sure this is a valile user ID. */
	if ((EventUser = EvCheckUser(userID)) == NULL) {
		read_unlock_irqrestore(&EvUsersLock, Flags);
		*packet = NULL;
		return -EV_ERROR_USER_EXISTS;
	}

	spin_lock(&EventUser->EkiLock);
	read_unlock(&EvUsersLock);

	for (;;) {
		TmpPacket = EventUser->EkiHead;

		/* If there is a queued packet then get it and return. */
		if (TmpPacket) {
			EventUser->EkiHead = TmpPacket->EpNext;
			if (EventUser->EkiHead == NULL) {
				EventUser->EkiTail = NULL;
			}
			EventUser->EkiQCount--;
			spin_unlock_irqrestore(&EventUser->EkiLock, Flags);
			*packet = TmpPacket;
			return EV_NOERR;
		}

		/* No queued packet and wait flag is not set then return
		 * with a try again later indication.
		 */
		if (!waitFlag) {
			spin_unlock_irqrestore(&EventUser->EkiLock, Flags);
			*packet = NULL;
			return -EV_ERROR_NO_EVENT;
		}

		/* Other wise sleep waiting for ane event to arrive. */
		spin_unlock_irqrestore(&EventUser->EkiLock, Flags);
		interruptible_sleep_on(&EventUser->EkiWaitQ);
		spin_lock_irqsave(&EventUser->EkiLock, Flags);
	}
}

EXPORT_SYMBOL(EvRegisterUser);
EXPORT_SYMBOL(EvUnRegisterUser);
EXPORT_SYMBOL(EvSetQLimit);
EXPORT_SYMBOL(EvSetSysQLimit);
EXPORT_SYMBOL(EvGetUserID);
EXPORT_SYMBOL(EvGetUserName);
EXPORT_SYMBOL(EvRegisterEventClass);
EXPORT_SYMBOL(EvUnRegisterEventClass);
EXPORT_SYMBOL(EvGetEventClassID);
EXPORT_SYMBOL(EvGetEventClassName);
EXPORT_SYMBOL(EvSubscribeEvent);
EXPORT_SYMBOL(EvUnSubscribeEvent);
EXPORT_SYMBOL(EvGetEvent);
