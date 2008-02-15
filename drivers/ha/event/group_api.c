/*
 * group_api.c
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

EvGroupInfo_t *_EvGetGroup(char *);
int EvGroupSendEvent(EvUserID_t, EvPri_t, EvGroupID_t, EvGroupID_t,
		      EvAccess_t, EvClassID_t, EvEventID_t,
		      int, int, int, int, EvDest_t *, int, void *);

EvGroupInfo_t *EvGroupHead = NULL;
unsigned short KernelNextGroup = 0;	/* Local events are ID zero */
rwlock_t EvGroupLock = RW_LOCK_UNLOCKED;

/*
 * EventCreateGroup creates a new instance of a locality on the local
 * machine.  Calling it has different consequences for different types
 * of registerations.
 *
 * Calling this functions with a type of EG_LOCAL is done by the event
 * broker system itself at init time.  This "local" instance of locality
 * is for events that will be addressed only to recipients on the local
 * system.  It is possible to create more local locality instances but
 * it is questionable if this is necessary.
 *
 * Calling it with a type of EG_MASTER creates the control structures
 * for the master ID controller on a distributed eventing network.  The
 * local system will be responsible for assigning event class and
 * member system ID's withing the locality being defined.
 *
 * Calling it with a type of EG_MEMBER creates a member instance to
 * control delivery of events on the local system and to send
 * generated events to the remote master where it can be echoed
 * to other members.
 *
 * A system using the event broker non locally would create a
 * communications connection between systems such as:
 *
 *    +-------------+                 +-------------+
 *    |             |                 |             |
 *    |             |   Comm Link     |             |
 *    |   Member    | <-------------> |   Master    |
 *    |             |                 |             |
 *    |             |                 |             |
 *    +-------------+                 +-------------+
 *
 * The boxes represent either a user level daemaon program or some
 * kernel component responsible for the comm link between member and
 * master.  For a member to use a locality, the master must first have
 * defined it and exported a comm interface to connect to.  When the
 * member wishes to participate it connects with the master, the master
 * registers a new member with its event broker using the EventAttachGroup()
 * function and returns the ID assigned to the member back to the member.
 * The member will then use EventCreateGroup() to register a member
 * control set with the ID assigned by the master;
 */

/*
 * EvCreateGroup() creates a new event group.  The user ID passed in is
 * used to register an event subscription for creater.  If the controCB
 * is passed in the subscription in delivered using it.  This subscription
 * is for control events to be sent to the creator.
 *
 * If the type field is set to local, the a local event group will be
 * created.  Since the system creates one local group at startup it is
 * questionable as to whether this is useful for more groups.
 *
 * If the type field is set to indicate a master then this system is
 * set up to serve an inter system event group as the master.  This means
 * this instance of the event broker assignes event class IDs and is used
 * as the communications path to member instances.
 *
 * If the type field indicates this is a member, the event group control
 * is set up on this system and this system will send events generated
 * to the master system to be echoed back to all other members.
 *
 * IF the user ID in not valid then -EPERM is returned.  If the event group
 * identifed by masterName has already been created then -EEXIST is returned.
 * If no kernel space can be allocated for the control data then -ENOSPC is
 * returned.
 *
 * Otherwise the event group control structures will be allocated and
 * a zero will be returned to indicate success.
 */
int
EvCreateGroup(char *groupName, char *memberName, EvType_t type,
	      EvAccess_t groupAccessCode, EvAccess_t controlAccessCode,
	      short hashSize, EvGroupID_t masterID, EvGroupID_t memberID,
	      EvGroupID_t *groupID)
{
	EvGroupInfo_t *EGroup;
	unsigned long Flags;
	unsigned short ControlClass;
	int Loop;
	int RetVal;

	write_lock_irqsave(&EvGroupLock, Flags);

	/* If event group exists then return error. */
	if (_EvGetGroup(groupName) != NULL) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	/* Create the event group control struct an link into list. */
	if (EvGroupHead == NULL) {
		if ((EvGroupHead = kmalloc (sizeof(EvGroupInfo_t), GFP_ATOMIC)) == NULL) {
			write_unlock_irqrestore(&EvGroupLock, Flags);
			return -EV_ERROR_MEM_ALLOC;
		}
		EGroup = EvGroupHead;
	} else {
		EGroup = EvGroupHead;

		while (EGroup->EgiNext) {
			EGroup = EGroup->EgiNext;
		}
		if ((EGroup->EgiNext = kmalloc(sizeof(EvGroupInfo_t), GFP_ATOMIC)) == NULL) {
			write_unlock_irqrestore(&EvGroupLock, Flags);
			return -EV_ERROR_MEM_ALLOC;
		}
		EGroup = EGroup->EgiNext;
	}

	strncpy(EGroup->EgiName, groupName, 16);
	EGroup->EgiID = KernelNextGroup++;

	strncpy(EGroup->EgiMemberName, memberName, 16);

	EGroup->EgiType = type;
	if (type == EG_MASTER) {
		/* If master prepare to assign members. */
		EGroup->EgiMemberID = 0;
		EGroup->EgiNextMemberID = 1;
		EGroup->EgiMasterID = EGroup->EgiID;
		EGroup->EgiMembers = NULL;
	} else {
		/* If not next member not used and member ID is from master */
		EGroup->EgiMemberID = memberID;
		EGroup->EgiNextMemberID = -1;
		EGroup->EgiMasterID = masterID;
		EGroup->EgiMembers = NULL;
	}

	/* Set returned ID value. */
	*groupID = EGroup->EgiID;

	/* Prepare the hash list to control event class assignments. */
	if (hashSize == 0) {
		EGroup->EgiHashSize = EV_NUM_LOCAL_HASHES;
	} else {
		EGroup->EgiHashSize = hashSize;
	}

	EGroup->EgiClassHash =
			kmalloc(sizeof(EvClassInfo_t) * hashSize, GFP_ATOMIC);

	/* Clear the group event control data. */
	EGroup->EgiGroupDest.EdID = 0;
	EGroup->EgiGroupDest.EdUinfo = NULL;
	EGroup->EgiGroupDest.EdCB = NULL;
	EGroup->EgiGroupDest.EdKinfo = NULL;

	EGroup->EgiUseCount = 0;
	EGroup->EgiNextClass = 0;  /* Zero is the control class. */
	EGroup->EgiLock = RW_LOCK_UNLOCKED;

	for (Loop = 0; Loop < hashSize; Loop++) {
		EGroup->EgiClassHash[Loop].EciLock = SPIN_LOCK_UNLOCKED;
		EGroup->EgiClassHash[Loop].EciNext = NULL;
	}

	EGroup->EgiPendRemoteList = NULL;
	EGroup->EgiNext = NULL;

	/* The first class if events created for a event group is a
	 * control class.  It is not needed to worry that that this
	 * is coordinated between the master and member because as
	 * the first it always gets the same ID.
 	 */
	EGroup->EgiAccessCode = groupAccessCode;
	write_unlock_irqrestore(&EvGroupLock, Flags);

	if ((RetVal = EvRegisterEventClass(EGroup->EgiID, "Control",
			     		   groupAccessCode,
					   controlAccessCode, &ControlClass))) {
		printk("Event Broker: %s Error %d\n",
		       "Add group failed to register control class", RetVal);
	}

	return EV_NOERR;
}

/* This function is as yet incomplete and is here for a particular use.  More
 * to follow.  DO NOT USE.
 */
int
EvTakeOverMaster(EvGroupID_t groupID, EvGroupID_t memberID,
		 EvAccess_t accessCode)
{
	EvGroupInfo_t *EGroup = EvGroupHead;
	unsigned long Flags;

	write_lock_irqsave(&EvGroupLock, Flags);

	/* Has the event group already been created or not? */
	while (EGroup && (EGroup->EgiID != groupID)) {
		EGroup = EGroup->EgiNext;
	}

	if (EGroup == NULL) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	/* If this is not a master group control the error */
	if (EGroup->EgiType != EG_MEMBER) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_TYPE;
	}

	/* Permissions? */
	if (EvCheckAccessCode(EGroup->EgiAccessCode, accessCode)) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_ACCESS;
	}

	if (memberID == EGroup->EgiMemberID) {
		/* This instance is taking over as master. */
		EGroup->EgiType = EG_MASTER;
		EGroup->EgiMasterID = EGroup->EgiMemberID;
		EGroup->EgiNextMemberID = memberID + 1;
	} else {
		/* Set the new master instance. */
	}

	write_unlock_irqrestore(&EvGroupLock, Flags);
	return EV_NOERR;
}

/*
 * EvDeleteGroup() removes and event group from the system.  This function
 * is not allowed to succed if there are event subscribers waiting on this
 * event group.  First the local structures for the event group are removed.
 * Once this is done if this is an events.
 *
 * Looks like this still needs some work.
 */
int
EvDeleteGroup(EvGroupID_t groupID, EvAccess_t accessCode)
{
	EvGroupInfo_t *TmpGroup;
	EvGroupInfo_t *FreeGroup;
	unsigned long Flags;

	write_lock_irqsave(&EvGroupLock, Flags);
	if (EvGroupHead == NULL) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	/* Check to see if this is at the head.  If so this is the local
	 * group and in not deletable.
	 */
	if (EvGroupHead->EgiID == groupID) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_BUSY;
	}

	TmpGroup = EvGroupHead;

	while(TmpGroup->EgiNext) {
		if (TmpGroup->EgiNext->EgiID == groupID) {
			FreeGroup = TmpGroup->EgiNext;

			if (FreeGroup->EgiUseCount > 1) {
				/* For now just do not allow unregisteration. */
				write_unlock_irqrestore(&EvGroupLock, Flags);
				return -EV_ERROR_GROUP_BUSY;
			}

			if (EvCheckAccessCode(FreeGroup->EgiAccessCode,
					      accessCode)) {
				write_unlock_irqrestore(&EvGroupLock, Flags);
				return -EV_ERROR_GROUP_ACCESS;
			}

			EvInternalUnRegisterEventClass(FreeGroup, 0,
						EvGetClassAccess(FreeGroup, 0));
			TmpGroup->EgiNext = FreeGroup->EgiNext;
			write_unlock_irqrestore(&EvGroupLock, Flags);
			kfree(FreeGroup->EgiClassHash);
			kfree(FreeGroup);

			return EV_NOERR;
		}

		TmpGroup = TmpGroup->EgiNext;
	}

	write_unlock_irqrestore(&EvGroupLock, Flags);
	return -EV_ERROR_GROUP_EXIST;
}

int
EvAttachGroup(EvGroupID_t groupID, char *memberName,
	      EvAccess_t accessCode, EvGroupID_t *memberID)
{
	EvGroupInfo_t *EGroup = EvGroupHead;
	unsigned long Flags;
	EvMemberList_t *Member;

	write_lock_irqsave(&EvGroupLock, Flags);

	/* Has the event group already been created or not? */
	while (EGroup && (EGroup->EgiID != groupID)) {
		EGroup = EGroup->EgiNext;
	}

	if (EGroup == NULL) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	/* If this is not a master group control the error */
	if (EGroup->EgiType != EG_MASTER) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_TYPE;
	}

	/* Permissions? */
	if (EvCheckAccessCode(EGroup->EgiAccessCode, accessCode)) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_ACCESS;
	}

	/* Has this member already been registered? */
	Member = EGroup->EgiMembers;
	while (Member) {
		if (!strncmp(Member->EslName, memberName, 16)) {
			return -EV_ERROR_MEMBER_EXIST;
		}
		Member = Member->EslNext;
	}

	/* Now allocate an element to store the member info into. */
	if (EGroup->EgiMembers == NULL) {
		if ((EGroup->EgiMembers = kmalloc(sizeof(EvMemberList_t),
							GFP_ATOMIC)) == NULL) {
			write_unlock_irqrestore(&EvGroupLock, Flags);
			return -EV_ERROR_MEM_ALLOC;
		}
		Member = EGroup->EgiMembers;
	} else {
		Member = EGroup->EgiMembers;
		while (Member->EslNext != NULL) {
			Member = Member->EslNext;
		}

		if ((Member->EslNext = kmalloc(sizeof(EvMemberList_t),
							GFP_ATOMIC)) == NULL) {
			write_unlock_irqrestore(&EvGroupLock, Flags);
			return -EV_ERROR_MEM_ALLOC;
		}
		Member = Member->EslNext;
	}

	/* Fill in the member information. */
	strncpy(Member->EslName, memberName, 16);
	Member->EslID = EGroup->EgiNextMemberID++;
	Member->EslNext = NULL;

	/* Incrementing use count prevents the master from deleting
	 * the event group control structures while remotes are trying
	 * to use it;
	 */
	EGroup->EgiUseCount++;
	/* Fill in returned member ID value */
	*memberID = Member->EslID;

	write_unlock_irqrestore(&EvGroupLock, Flags);
	return EV_NOERR;;
}

int
EvDetachGroup(EvGroupID_t groupID, EvGroupID_t memberID, EvAccess_t accessCode)
{
	EvGroupInfo_t *EGroup = EvGroupHead;
	unsigned long Flags;
	EvMemberList_t *Member;
	EvMemberList_t *FreeMember;

	write_lock_irqsave(&EvGroupLock, Flags);

	/* Has the event group already been created or not? */
	while (EGroup && (EGroup->EgiID != groupID)) {
		EGroup = EGroup->EgiNext;
	}

	if (EGroup == NULL) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	/* If this is not a master group control the error */
	if (EGroup->EgiType != EG_MASTER) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_TYPE;
	}

	/* Are there any members registered? */
	if (EGroup->EgiMembers == NULL) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_MEMBER_EXIST;
	}

	/* Permissions? */
	if (EvCheckAccessCode(EGroup->EgiAccessCode, accessCode)) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_ACCESS;
	}

	/* Find the member pointer */
	if (EGroup->EgiMembers->EslID == memberID) {
		FreeMember = EGroup->EgiMembers;
		EGroup->EgiMembers = EGroup->EgiMembers->EslNext;

		/* Do not forget to indicate the member is gone. */
		EGroup->EgiUseCount--;
		write_unlock_irqrestore(&EvGroupLock, Flags);
		kfree(FreeMember);
		return EV_NOERR;
	}

	Member = EGroup->EgiMembers;
	while (Member->EslNext) {
		if (Member->EslNext->EslID == memberID) {
			FreeMember = Member->EslNext;
			Member->EslNext = FreeMember->EslNext;

			/* Do not forget to indicate the member is gone. */
			EGroup->EgiUseCount--;
			write_unlock_irqrestore(&EvGroupLock, Flags);
			kfree(FreeMember);
			return EV_NOERR;
		}
		Member = Member->EslNext;
	}

	write_unlock_irqrestore(&EvGroupLock, Flags);
	return -EV_ERROR_MEMBER_EXIST;
}

/*
 * This function returns the id of the event group identified by the name
 * field.  This ID is only valid for the local machine and should not be
 * passed outside of it.
 */
int
EvGetGroupID(char *name, EvGroupID_t *groupID)
{
	EvGroupInfo_t *EGroup;
	unsigned long Flags;

	write_lock_irqsave(&EvGroupLock, Flags);
	EGroup = _EvGetGroup(name);
	write_unlock_irqrestore(&EvGroupLock, Flags);
	if (EGroup) {
		*groupID = EGroup->EgiID;
		return EV_NOERR;
	}

	return -EV_ERROR_GROUP_EXIST;
}

EvGroupInfo_t *
_EvGetGroup(char *name)
{
	EvGroupInfo_t *EGroup = EvGroupHead;

	while (EGroup) {
		if (!strcmp(EGroup->EgiName, name)) {
			return EGroup;
		}
		EGroup = EGroup->EgiNext;
	}
	return NULL;
}

/*
 * This function returns the name of the event group identified by the ID
 * field.
 */
int
EvGetGroupName(EvGroupID_t groupID, char *name)
{
	EvGroupInfo_t *EGroup;
	unsigned long Flags;

	write_lock_irqsave(&EvGroupLock, Flags);
	if ((EGroup = EvGetGroupBase(groupID)) == NULL) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	strncpy(name, EGroup->EgiName, 16);
	write_unlock_irqrestore(&EvGroupLock, Flags);
	return EV_NOERR;
}

/*
 * This function returns the id of the member element element from group
 * ID identified by the name * field.  This ID is valid within the group
 * domain.
 */
int
EvGetMemberID(EvGroupID_t groupID, char *memberName, EvGroupID_t *memberID)
{
	EvGroupInfo_t *EGroup;
	unsigned long Flags;
	EvMemberList_t *Member;

	write_lock_irqsave(&EvGroupLock, Flags);
	if ((EGroup = EvGetGroupBase(groupID)) == NULL) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	write_lock(&EGroup->EgiLock);
	write_unlock(&EvGroupLock);

	/* If this is a member and not a master group controler. */
	if (EGroup->EgiType == EG_MEMBER) {
		/* If the member is this group member then localize. */
		if (!strncmp(EGroup->EgiMemberName, memberName, 16)) {
			*memberID = EGroup->EgiMemberID;
			write_unlock_irqrestore(&EGroup->EgiLock, Flags);
			return EV_NOERR;
		}

		return EvSendGetMemberID(EGroup, memberName, memberID, Flags);
	}

	if (!strncmp(EGroup->EgiMemberName, memberName, 16)) {
		*memberID = EGroup->EgiMemberID;
		write_unlock_irqrestore(&EGroup->EgiLock, Flags);
		return EV_NOERR;
	}

	Member = EGroup->EgiMembers;

	while (Member) {
		if (!strncmp(Member->EslName, memberName, 16)) {
			*memberID = Member->EslID;
			write_unlock_irqrestore(&EGroup->EgiLock, Flags);
			return EV_NOERR;
		}
		Member = Member->EslNext;
	}

	write_unlock_irqrestore(&EGroup->EgiLock, Flags);

	return -EV_ERROR_MEMBER_EXIST;
}

int
EvGetMemberName(EvGroupID_t groupID, EvGroupID_t memberID, char *memberName)
{
	EvGroupInfo_t *EGroup;
	EvMemberList_t *Member;
	unsigned long Flags;

	write_lock_irqsave(&EvGroupLock, Flags);
	if ((EGroup = EvGetGroupBase(groupID)) == NULL) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	write_lock(&EGroup->EgiLock);
	write_unlock(&EvGroupLock);

	/* If this is a member and not a master group controler. */
	if (EGroup->EgiType == EG_MEMBER) {
		/* If the member is this group member then localize. */
		if (EGroup->EgiMemberID == memberID) {
			strncpy(memberName, EGroup->EgiMemberName, 16);
			write_unlock_irqrestore(&EGroup->EgiLock, Flags);
			return EV_NOERR;
		}

		return EvSendGetMemberName(EGroup, memberID, memberName, Flags);
	}

	if (EGroup->EgiMemberID == memberID) {
		strncpy(memberName, EGroup->EgiMemberName, 16);
		write_unlock_irqrestore(&EGroup->EgiLock, Flags);
		return EV_NOERR;
	}

	Member = EGroup->EgiMembers;

	while (Member) {
		if (Member->EslID == memberID) {
			strncpy(memberName, Member->EslName, 16);
			write_unlock_irqrestore(&EGroup->EgiLock, Flags);
			return EV_NOERR;
		}
		Member = Member->EslNext;
	}

	write_unlock_irqrestore(&EGroup->EgiLock, Flags);

	return -EV_ERROR_MEMBER_EXIST;
}

int
EvSubscribeGroupEvents(EvUserID_t userID,
		       EvGroupID_t groupID, EvAccess_t accessCode,
		       int (*kernelCB)(EvUserID_t, int, EvGroupID_t,
				       EvGroupID_t, EvClassID_t, EvEventID_t,
				       int, int, int, int, int, void *))
{
	EvKernelInfo_t *EventUser;
	EvGroupInfo_t *EGroup;
	unsigned long Flags;

	read_lock_irqsave(&EvUsersLock, Flags);
	if ((EventUser = EvCheckUser(userID)) == NULL) {
		read_unlock_irqrestore(&EvUsersLock, Flags);
		return -EV_ERROR_USER_EXISTS;
	}

	write_lock(&EvGroupLock);
	read_unlock(&EvUsersLock);

	if ((EGroup = EvGetGroupBase(groupID)) == NULL) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	/* Check the access code for the group. */
	if (EvCheckAccessCode(EGroup->EgiAccessCode, accessCode)) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_ACCESS;
	}

	/* Check that there are no current control processes. */
	if (EGroup->EgiGroupDest.EdID != 0) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_BUSY;
	}

	EGroup->EgiGroupDest.EdPri = 0;		/* Not used. */
	EGroup->EgiGroupDest.EdID = userID;
	EGroup->EgiGroupDest.EdUinfo = NULL;
	EGroup->EgiGroupDest.EdCB = kernelCB;
	EGroup->EgiGroupDest.EdKinfo = EventUser;

	EGroup->EgiUseCount++;

	write_unlock_irqrestore(&EvGroupLock, Flags);
	return EV_NOERR;
}

int
EvUnSubscribeGroupEvents(EvUserID_t userID, EvGroupID_t groupID)
{
	EvKernelInfo_t *EventUser;
	EvGroupInfo_t *EGroup;
	unsigned long Flags;

	read_lock_irqsave(&EvUsersLock, Flags);
	if ((EventUser = EvCheckUser(userID)) == NULL) {
		read_unlock_irqrestore(&EvUsersLock, Flags);
		return -EV_ERROR_USER_EXISTS;
	}

	write_lock(&EvGroupLock);
	read_unlock(&EvUsersLock);

	if ((EGroup = EvGetGroupBase(groupID)) == NULL) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	/* Check that there are no current control processes. */
	if (EGroup->EgiGroupDest.EdID != userID) {
		write_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_ACCESS;
	}

	EGroup->EgiGroupDest.EdID = 0;
	EGroup->EgiGroupDest.EdUinfo = NULL;
	EGroup->EgiGroupDest.EdCB = NULL;
	EGroup->EgiGroupDest.EdKinfo = NULL;

	EGroup->EgiUseCount--;

	write_unlock_irqrestore(&EvGroupLock, Flags);

	return EV_NOERR;
}

int
EvSendGetMemberID(EvGroupInfo_t *eGroup, char *memberName,
		  EvGroupID_t *memberID, unsigned long flags)
{
	EvPendRem_t *Pend;
	int RetVal;

	/* Check to see if there is a remote control entity registered. */
	if (eGroup->EgiGroupDest.EdID == 0) {
		write_unlock_irqrestore(&eGroup->EgiLock, flags);
		return -EV_ERROR_NO_REMOTE;
	}

	/* Allocate an entry to pend on while remote request is processed. */
	if ((Pend = EvGetPendEntry(eGroup, EV_REM_GET_MEMBER_ID, 0, 0, 0, 0,
				   strlen(memberName)+1, memberName)) == NULL) {
		write_unlock_irqrestore(&eGroup->EgiLock, flags);
		return -EV_ERROR_MEM_ALLOC;
	}

	/* Send the reqest to the waiting control agent. */
	RetVal = EvGroupSendEvent(0, 0, eGroup->EgiID, eGroup->EgiMemberID,
		   -1, EV_CONTROL_CLASS_ID, EV_REM_GET_MEMBER_ID,
		   0, 0, 0, 0,
		   &eGroup->EgiGroupDest, strlen(memberName) + 1, memberName);
	if (RetVal) {
		/* Free the Pend data structure. */
		EvDelPendEntry(eGroup, Pend);

		write_unlock_irqrestore(&eGroup->EgiLock, flags);
		return RetVal;
	}
		   
	/* Make sure group cannont be removed and Wait for information
	 * to be returned. */
	eGroup->EgiUseCount++;
	write_unlock_irqrestore(&eGroup->EgiLock, flags);

	interruptible_sleep_on(&Pend->PrWaitQ);

	/* Require the load and releas the group use count to allow other
	 * group activities. */
	write_lock_irqsave(&eGroup->EgiLock, flags);
	eGroup->EgiUseCount--;

	/* If an error has been indicated then return it. */
	RetVal = Pend->PrRetInfo.EpInfo[0];
	if (RetVal) {
		/* Free the Pend data structure. */
		EvDelPendEntry(eGroup, Pend);

		write_unlock_irqrestore(&eGroup->EgiLock, flags);
		return -RetVal;
	}

	/* fill in the memberID info. */
	*memberID = Pend->PrRetInfo.EpInfo[1];

	/* Free the Pend data structure. */
	EvDelPendEntry(eGroup, Pend);

	write_unlock_irqrestore(&eGroup->EgiLock, flags);
	return EV_NOERR;
}

int
EvCompGetMemberID(EvGroupID_t groupID, char *memberName,
		  int error, EvGroupID_t memberID)
{
	EvGroupInfo_t *EGroup;
	EvPendRem_t *Pend;
	unsigned long Flags;

	read_lock_irqsave(&EvGroupLock, Flags);
	if ((EGroup = EvGetGroupBase(groupID)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	write_lock(&EGroup->EgiLock);
	read_unlock(&EvGroupLock);

	/* Pend the pending control struct for this ID. */
	if ((Pend = EvFindPendEntry(EGroup, EV_REM_GET_MEMBER_ID,
				    0, 0, 0, 0, 
				    strlen(memberName)+1, memberName)) == NULL) {
		write_unlock_irqrestore(&EGroup->EgiLock, Flags);
		return -EV_ERROR_NO_REQUESTOR;
	}

	/* Fill in the return values. */
	Pend->PrRetInfo.EpInfo[0] = error;
	Pend->PrRetInfo.EpInfo[1] = memberID;

	/* Wake up the requester. */
	wake_up_interruptible(&Pend->PrWaitQ);

	write_unlock_irqrestore(&EGroup->EgiLock, Flags);
	return EV_NOERR;
}

int
EvSendGetMemberName(EvGroupInfo_t *eGroup, EvGroupID_t memberID,
		    char *memberName,  unsigned long flags)
{
	EvPendRem_t *Pend;
	int RetVal;

	/* Check to see if there is a remote control entity registered. */
	if (eGroup->EgiGroupDest.EdID == 0) {
		write_unlock_irqrestore(&eGroup->EgiLock, flags);
		return -EV_ERROR_NO_REMOTE;
	}

	/* Allocate an entry to pend on while remote request is processed. */
	if ((Pend = EvGetPendEntry(eGroup, EV_REM_GET_MEMBER_NAME,
				   memberID, 0, 0, 0, 0, NULL)) == NULL) {
		write_unlock_irqrestore(&eGroup->EgiLock, flags);
		return -EV_ERROR_MEM_ALLOC;
	}

	/* Send the reqest to the waiting control agent. */
	RetVal = EvGroupSendEvent(0, 0, eGroup->EgiID, eGroup->EgiMemberID,
		   -1, EV_CONTROL_CLASS_ID, EV_REM_GET_MEMBER_NAME,
		   memberID, 0, 0, 0,
		   &eGroup->EgiGroupDest, 0, NULL);
	if (RetVal) {
		/* Free the Pend data structure. */
		EvDelPendEntry(eGroup, Pend);
		write_unlock_irqrestore(&eGroup->EgiLock, flags);
		return RetVal;
	}
		   
	/* Make sure group cannont be removed and Wait for information
	 * to be returned. */
	eGroup->EgiUseCount++;
	write_unlock_irqrestore(&eGroup->EgiLock, flags);

	interruptible_sleep_on(&Pend->PrWaitQ);

	/* Require the load and releas the group use count to allow other
	 * group activities. */
	write_lock_irqsave(&eGroup->EgiLock, flags);
	eGroup->EgiUseCount--;

	/* If an error has been indicated then return it. */
	RetVal = Pend->PrRetInfo.EpInfo[0];
	if (RetVal) {
		/* Free the Pend data structure. */
		EvDelPendEntry(eGroup, Pend);
		write_unlock_irqrestore(&eGroup->EgiLock, flags);
		return -RetVal;
	}

	/* fill in the memberName. */
	strncpy(memberName, Pend->PrRetInfo.EpData, 16);

	/* Free the Pend data structure. */
	EvDelPendEntry(eGroup, Pend);

	write_unlock_irqrestore(&eGroup->EgiLock, flags);
	return EV_NOERR;
}

int
EvCompGetMemberName(EvGroupID_t groupID, EvGroupID_t memberID,
		    int error, char * memberName)
{
	EvGroupInfo_t *EGroup;
	EvPendRem_t *Pend;
	unsigned long Flags;

	read_lock_irqsave(&EvGroupLock, Flags);
	if ((EGroup = EvGetGroupBase(groupID)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	write_lock(&EGroup->EgiLock);
	read_unlock(&EvGroupLock);

	/* Pend the pending control struct for this ID. */
	if ((Pend = EvFindPendEntry(EGroup, EV_REM_GET_MEMBER_NAME,
				    memberID, 0, 0, 0, 0, NULL)) == NULL) {
		write_unlock_irqrestore(&EGroup->EgiLock, Flags);
		return -EV_ERROR_NO_REQUESTOR;
	}

	/* Fill in the return values. */
	Pend->PrRetInfo.EpInfo[0] = error;
	Pend->PrRetInfo.EpData = memberName;

	/* Wake up the requester. */
	wake_up_interruptible(&Pend->PrWaitQ);

	write_unlock_irqrestore(&EGroup->EgiLock, Flags);
	return EV_NOERR;
}

/*
 * User requests a new event class on a remote server.
 */
int
EvSendRegEventClass(EvGroupInfo_t *eGroup, char *className,
		    EvAccess_t groupAccessCode, EvAccess_t classAccessCode,
		    EvClassID_t *classID, unsigned long *flags)
{
	EvPendRem_t *Pend;
	int RetVal;

	/* Check to see if there is a remote control entity registered. */
	if (eGroup->EgiGroupDest.EdID == 0) {
		return -EV_ERROR_NO_REMOTE;
	}

	/* Allocate an entry to pend on while remote request is processed. */
	if ((Pend = EvGetPendEntry(eGroup, EV_REM_REGISTER_EVENT_CLASS,
				   groupAccessCode, classAccessCode, 0, 0,
				   strlen(className) + 1, className)) == NULL) {
		return -EV_ERROR_MEM_ALLOC;
	}

	/* Send the reqest to the waiting control agent. */
	RetVal = EvGroupSendEvent(0, 0, eGroup->EgiID, eGroup->EgiMemberID,
		   -1, EV_CONTROL_CLASS_ID, EV_REM_REGISTER_EVENT_CLASS,
		   groupAccessCode, classAccessCode, 0, 0,
		   &eGroup->EgiGroupDest, strlen(className) + 1, className);
	if (RetVal) {
		/* Free the Pend data structure. */
		EvDelPendEntry(eGroup, Pend);

		return RetVal;
	}
		   
	/* Make sure group cannont be removed and Wait for information
	 * to be returned. */
	eGroup->EgiUseCount++;
	write_unlock_irqrestore(&eGroup->EgiLock, *flags);

	interruptible_sleep_on(&Pend->PrWaitQ);

	/* Require the load and releas the group use count to allow other
	 * group activities. */
	write_lock_irqsave(&eGroup->EgiLock, *flags);
	eGroup->EgiUseCount--;


	/* If an error has been indicated then return it. */
	RetVal = Pend->PrRetInfo.EpInfo[0];
	if (RetVal) {
		/* Free the Pend data structure. */
		EvDelPendEntry(eGroup, Pend);

		return -RetVal;
	}

	/* fill in the classID. */
	*classID = Pend->PrRetInfo.EpInfo[1];

	/* Free the Pend data structure. */
	EvDelPendEntry(eGroup, Pend);

	return EV_NOERR;
}

/*
 * Connection daemon has recieved a message from the event master
 * that a new class is available for this member.  This function
 * registers that new class.
 */
int
EvRemoteRegisterEventClass(EvGroupID_t groupID, char *className,
			   EvAccess_t groupAccessCode,
			   EvAccess_t classAccessCode,
		   	   int error, EvClassID_t classID)
{
	EvGroupInfo_t *EGroup;
	EvPendRem_t *Pend;
	EvClassInfo_t *HashBase;
	unsigned long Flags;
	int RetVal;

	read_lock_irqsave(&EvGroupLock, Flags);
	if ((EGroup = EvGetGroupBase(groupID)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	write_lock(&EGroup->EgiLock);
	read_unlock(&EvGroupLock);

	/* Pend the pending control struct for this ID.
	 *
	 * If Pend is NULL then this is the master reporting a new event
	 * class.
	 */
	if ((Pend = EvFindPendEntry(EGroup, EV_REM_REGISTER_EVENT_CLASS,
				    groupAccessCode, classAccessCode, 0, 0,
				    strlen(className)+1, className)) != NULL) {
		/* Fill in the return values. */
		Pend->PrRetInfo.EpInfo[0] = error;
		Pend->PrRetInfo.EpInfo[1] = classID;

		/* Wake up the requester. */
		wake_up_interruptible(&Pend->PrWaitQ);

		write_unlock_irqrestore(&EGroup->EgiLock, Flags);
		return EV_NOERR;
	}

	/* Register a new event class on a group member for the master. */
	HashBase = EvGetHashBase(EGroup, classID);

	while (HashBase->EciNext != NULL) {
		HashBase = HashBase->EciNext;
	}

	RetVal=EvFillInHashInfo(HashBase, classAccessCode, className, &classID);

	write_unlock_irqrestore(&EGroup->EgiLock, Flags);
	return RetVal;
}

/*
 * Group master informs remote servers a new class has been registered.
 */
int
EvSendRegClassToMembers(EvGroupInfo_t *eGroup, char *className,
			EvAccess_t groupAccessCode,
			EvAccess_t classAccessCode, EvClassID_t classID)
{
	/* Check to see if there is a remote control entity registered. */
	if (eGroup->EgiGroupDest.EdID == 0) {
		return EV_NOERR;
	}

	/* Send the reqest to the waiting control agent. */
	return EvGroupSendEvent(0, 0, eGroup->EgiID, 0,
		   -1, EV_CONTROL_CLASS_ID, EV_MASTER_REG_EVENT_CLASS,
		   groupAccessCode, classAccessCode, classID, 0,
		   &eGroup->EgiGroupDest, strlen(className) + 1, className);
}

/*
 * User requests unregisters an event class on a remote server.
 */
int
EvSendUnRegEventClass(EvGroupInfo_t *eGroup, EvAccess_t accessCode,
		      EvClassID_t classID, unsigned long *flags)
{
	EvPendRem_t *Pend;
	int RetVal;

	/* Check to see if there is a remote control entity registered. */
	if (eGroup->EgiGroupDest.EdID == 0) {
		return -EV_ERROR_NO_REMOTE;
	}

	/* Allocate an entry to pend on while remote request is processed. */
	if ((Pend = EvGetPendEntry(eGroup, EV_REM_UNREGISTER_EVENT_CLASS,
				   accessCode, classID, 0, 0,
				   0, NULL)) == NULL) {
		return -EV_ERROR_MEM_ALLOC;
	}

	/* Send the reqest to the waiting control agent. */
	RetVal = EvGroupSendEvent(0, 0, eGroup->EgiID, eGroup->EgiMemberID,
		   -1, EV_CONTROL_CLASS_ID, EV_REM_UNREGISTER_EVENT_CLASS,
		   accessCode, classID, 0, 0,
		   &eGroup->EgiGroupDest, 0, NULL);
	if (RetVal) {
		return RetVal;
	}
		   
	/* Make sure group cannont be removed and Wait for information
	 * to be returned. */
	eGroup->EgiUseCount++;
	write_unlock_irqrestore(&eGroup->EgiLock, *flags);

	interruptible_sleep_on(&Pend->PrWaitQ);

	/* Require the load and releas the group use count to allow other
	 * group activities. */
	write_lock_irqsave(&eGroup->EgiLock, *flags);
	eGroup->EgiUseCount--;

	RetVal = Pend->PrRetInfo.EpInfo[0];

	/* Free the Pend data structure. */
	EvDelPendEntry(eGroup, Pend);
	return RetVal;
}

int 
EvRemoteUnRegisterEventClass(EvGroupID_t groupID, EvClassID_t classID,
			     EvAccess_t accessCode, int error)
{
	EvGroupInfo_t *EGroup;
	EvPendRem_t *Pend;
	unsigned long Flags;
	int RetVal;

	read_lock_irqsave(&EvGroupLock, Flags);
	if ((EGroup = EvGetGroupBase(groupID)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	write_lock(&EGroup->EgiLock);
	read_unlock(&EvGroupLock);

	/* Pend the pending control struct for this ID.
	 *
	 * If Pend is NULL then this is the master reporting a new event
	 * class.
	 */
	if ((Pend = EvFindPendEntry(EGroup, EV_REM_UNREGISTER_EVENT_CLASS,
				    accessCode, classID, 0, 0,
				    0, NULL)) != NULL) {
		/* Fill in the return values. */
		Pend->PrRetInfo.EpInfo[0] = error;

		/* Wake up the requester. */
		wake_up_interruptible(&Pend->PrWaitQ);

		write_unlock_irqrestore(&EGroup->EgiLock, Flags);
		return EV_NOERR;
	}

	RetVal =  EvInternalUnRegisterEventClass(EGroup, classID, accessCode);
	write_unlock_irqrestore(&EGroup->EgiLock, Flags);
	return RetVal;
}

int
EvSendUnRegClassToMembers(EvGroupInfo_t *eGroup, EvClassID_t classID,
			  EvAccess_t accessCode)
{
	/* Check to see if there is a remote control entity registered. */
	if (eGroup->EgiGroupDest.EdID == 0) {
		return EV_NOERR;
	}

	/* Send the reqest to the waiting control agent. */
	return EvGroupSendEvent(0, 0, eGroup->EgiID, eGroup->EgiMemberID,
		   -1, EV_CONTROL_CLASS_ID, EV_MASTER_UNREG_EVENT_CLASS,
		   accessCode, classID, 0, 0,
		   &eGroup->EgiGroupDest, 0, NULL);
}

int
EvRemoteSendEvent(EvUserID_t senderID, EvGroupID_t memberID, EvPri_t pri,
		  EvGroupID_t groupID, EvClassID_t classID, EvEventID_t eventID,
		  EvAccess_t accessCode,
		  int info0, int info1, int info2, int info3,
		  int dataLen, void *data)
{
	EvGroupInfo_t *EGroup;
	EvDestBase_t *EventBase;
	EvClassInfo_t *HashBase;
	EvClassInfo_t *ClassBase;
	unsigned long Flags;
	int RetVal;

	read_lock_irqsave(&EvGroupLock, Flags);
	if ((EGroup = EvGetGroupBase(groupID)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	/* If this a remote gen to the master then echo it back to
	 * all members.
	 */
	if (EGroup->EgiType == EG_MASTER) {
		if (EGroup->EgiGroupDest.EdID != 0) {
			RetVal = EvGroupSendEvent(senderID, pri,
				   EGroup->EgiID, EGroup->EgiMemberID,
				   accessCode, classID, eventID,
				   info0, info1, info2, info3,
				   &EGroup->EgiGroupDest, dataLen, data);
			if (RetVal) {
				read_unlock_irqrestore(&EvGroupLock, Flags);
				return RetVal;
			}
		}
	}

	/* If the memberID issuing the event is the same as the member
	 * id of this system then we have been echoed back an event 
	 * we sent tot he master.  Ignore it.
	 */
	if ((EGroup->EgiType == EG_MEMBER) &&
	    (EGroup->EgiMemberID == memberID)) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return EV_NOERR;
	}

	HashBase = EvGetHashBase(EGroup, classID);

	if ((EventBase = EvFindEventBase(classID, HashBase, &ClassBase))==NULL){
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_CLASS_EXISTS;
	}

	spin_lock(&ClassBase->EciLock);
	read_unlock(&EvGroupLock);

	RetVal = _EvSendEvent(EventBase, senderID, pri, groupID, memberID, classID,
		     eventID, info0, info1, info2, info3, dataLen, data);

	spin_unlock_irqrestore(&ClassBase->EciLock, Flags);
	return RetVal;
}

int
EvSyncToMember(EvGroupID_t groupID, EvAccess_t accessCode, EvGroupID_t memberID)
{
	EvGroupInfo_t *EGroup;
	EvClassInfo_t *HashBase;
	unsigned long Flags;
	int Hash;
	int RetVal;

	/* Get the group info pointer and return with it locked. */
	if ((RetVal = EvLockAndGetGroup(groupID, EV_ACCESS_CHECK,
					accessCode, &Flags,
					EV_LOCK_GROUP_READ, &EGroup))) {
		return RetVal;
	}

	/* If this a remote gen to the master then echo it back to
	 * all members.
	 */
	if (EGroup->EgiType != EG_MASTER) {
		read_unlock_irqrestore(&EGroup->EgiLock, Flags);
		return EV_ERROR_GROUP_TYPE;
	}

	for (Hash = 0; Hash < EGroup->EgiHashSize; Hash++) {
		HashBase = &EGroup->EgiClassHash[Hash];

		while (HashBase->EciNext != NULL) {
			if (HashBase->EciClass == EV_CONTROL_CLASS_ID) {
				HashBase = HashBase->EciNext;
				continue;
			}

			RetVal = EvGroupSendEvent(0, 0, EGroup->EgiID, memberID, -1,
				   EV_CONTROL_CLASS_ID,
				   EV_MASTER_REG_EVENT_CLASS,
		   		   accessCode, HashBase->EciAccessCode,
				   HashBase->EciClass, 0,
		   		   &EGroup->EgiGroupDest,
				   strlen(HashBase->EciName) + 1,
				   HashBase->EciName);
			if (RetVal) {
				read_unlock_irqrestore(&EGroup->EgiLock, Flags);
				return RetVal;
			}

			HashBase = HashBase->EciNext;
		}
	}

	read_unlock_irqrestore(&EGroup->EgiLock, Flags);
	return EV_NOERR;
}

int
EvSendGetUserID(EvGroupID_t groupID, EvGroupID_t memberID,
	      char *userName, EvUserID_t *userID)
{
	EvGroupInfo_t *EGroup;
	EvPendRem_t *Pend;
	unsigned long Flags;
	int RetVal;

	/* Get the group info pointer and return with it locked. */
	if ((RetVal = EvLockAndGetGroup(groupID, EV_ACCESS_IGNORE,
					EAC_NONE, &Flags,
					EV_LOCK_GROUP_WRITE, &EGroup))) {
		return RetVal;
	}

	
	/* Check to see if there is a remote control entity registered. */
	if (EGroup->EgiGroupDest.EdID == 0) {
		write_unlock_irqrestore(&EGroup->EgiLock, Flags);
		return -EV_ERROR_NO_REMOTE;
	}

	/* Allocate an entry to pend on while remote request is processed. */
	if ((Pend = EvGetPendEntry(EGroup, EV_REM_GET_USER_ID,
				   memberID, 0, 0, 0,
				   strlen(userName) + 1, userName)) == NULL) {
		write_unlock_irqrestore(&EGroup->EgiLock, Flags);
		return -EV_ERROR_MEM_ALLOC;
	}

	/* Send the reqest to the waiting control agent. */
	RetVal = EvGroupSendEvent(0, 0, EGroup->EgiID, EGroup->EgiMemberID,
		   -1, EV_CONTROL_CLASS_ID, EV_REM_GET_USER_ID,
		   memberID, 0, 0, 0, &EGroup->EgiGroupDest,
		   strlen(userName) + 1, userName);
	if (RetVal) {
		EvDelPendEntry(EGroup, Pend);
		write_unlock_irqrestore(&EGroup->EgiLock, Flags);
		return RetVal;
	}
		   
	/* Make sure group cannont be removed and Wait for information
	 * to be returned. */
	EGroup->EgiUseCount++;
	write_unlock_irqrestore(&EGroup->EgiLock, Flags);

	interruptible_sleep_on(&Pend->PrWaitQ);

	/* Reaquire the load and releas the group use count to allow other
	 * group activities. */
	write_lock_irqsave(&EGroup->EgiLock, Flags);
	EGroup->EgiUseCount--;

	/* If an error has been indicated then return it. */
	RetVal = Pend->PrRetInfo.EpInfo[0];
	if (RetVal) {
		/* Free the Pend data structure. */
		EvDelPendEntry(EGroup, Pend);

		write_unlock_irqrestore(&EGroup->EgiLock, Flags);
		return -RetVal;
	}
	
	/* Fill in user userID information. */
	*userID = Pend->PrRetInfo.EpInfo[1];

	/* Free the Pend data structure. */
	EvDelPendEntry(EGroup, Pend);

	write_unlock_irqrestore(&EGroup->EgiLock, Flags);
	return EV_NOERR;
}

int
EvCompGetUserID(EvGroupID_t groupID, EvGroupID_t memberID, char *userName,
		int error, EvUserID_t userID)
{
	EvGroupInfo_t *EGroup;
	EvPendRem_t *Pend;
	unsigned long Flags;

	read_lock_irqsave(&EvGroupLock, Flags);
	if ((EGroup = EvGetGroupBase(groupID)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	write_lock(&EGroup->EgiLock);
	read_unlock(&EvGroupLock);

	/* Pend the pending control struct for this ID. */
	if ((Pend = EvFindPendEntry(EGroup, EV_REM_GET_USER_ID,
				    memberID, 0, 0, 0,
				    strlen(userName) + 1, userName)) == NULL) {
		write_unlock_irqrestore(&EGroup->EgiLock, Flags);
		return -EV_ERROR_NO_REQUESTOR;
	}

	/* Fill in the return values. */
	Pend->PrRetInfo.EpInfo[0] = error;
	Pend->PrRetInfo.EpInfo[1] = userID;

	/* Wake up the requester. */
	wake_up_interruptible(&Pend->PrWaitQ);

	write_unlock_irqrestore(&EGroup->EgiLock, Flags);
	return EV_NOERR;
}

int
EvSendGetUserName(EvGroupInfo_t *eGroup, EvGroupID_t memberID,
	          EvUserID_t userID, char *userName, unsigned long *flags)
{
	EvPendRem_t *Pend;
	int RetVal;

	/* Check to see if there is a remote control entity registered. */
	if (eGroup->EgiGroupDest.EdID == 0) {
		return -EV_ERROR_NO_REMOTE;
	}

	/* Allocate an entry to pend on while remote request is processed. */
	if ((Pend = EvGetPendEntry(eGroup, EV_REM_GET_USER_NAME,
				   memberID, userID, 0, 0, 0, NULL)) == NULL) {
		return -EV_ERROR_MEM_ALLOC;
	}

	/* Send the reqest to the waiting control agent. */
	RetVal = EvGroupSendEvent(0, 0, eGroup->EgiID, eGroup->EgiMemberID,
		   -1, EV_CONTROL_CLASS_ID, EV_REM_GET_USER_NAME,
		   memberID, userID, 0, 0, &eGroup->EgiGroupDest, 0, NULL);
	if (RetVal) {
		EvDelPendEntry(eGroup, Pend);
		return RetVal;
	}
		   
	/* Make sure group cannont be removed and Wait for information
	 * to be returned. */
	eGroup->EgiUseCount++;
	write_unlock_irqrestore(&eGroup->EgiLock, *flags);

	interruptible_sleep_on(&Pend->PrWaitQ);

	/* Reaquire the load and releas the group use count to allow other
	 * group activities. */
	write_lock_irqsave(&eGroup->EgiLock, *flags);
	eGroup->EgiUseCount--;

	/* If an error has been indicated then return it. */
	RetVal = Pend->PrRetInfo.EpInfo[0];
	if (RetVal) {
		/* Free the Pend data structure. */
		EvDelPendEntry(eGroup, Pend);

		return -RetVal;
	}
	
	/* Copy user Name information. */
	strncpy(userName, Pend->PrRetInfo.EpData, 16);

	/* Free the Pend data structure. */
	EvDelPendEntry(eGroup, Pend);

	return EV_NOERR;
}

int
EvCompGetUserName(EvGroupID_t groupID, EvGroupID_t memberID, EvUserID_t userID,
		int error, char *userName)
{
	EvGroupInfo_t *EGroup;
	EvPendRem_t *Pend;
	unsigned long Flags;

	read_lock_irqsave(&EvGroupLock, Flags);
	if ((EGroup = EvGetGroupBase(groupID)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	write_lock(&EGroup->EgiLock);
	read_unlock(&EvGroupLock);

	/* Pend the pending control struct for this ID. */
	if ((Pend = EvFindPendEntry(EGroup, EV_REM_GET_USER_NAME,
				    memberID, userID, 0, 0,
				    strlen(userName) + 1, userName)) == NULL) {
		write_unlock_irqrestore(&EGroup->EgiLock, Flags);
		return -EV_ERROR_NO_REQUESTOR;
	}

	/* Fill in the return values. */
	Pend->PrRetInfo.EpInfo[0] = error;
	Pend->PrRetInfo.EpData = userName;

	/* Wake up the requester. */
	wake_up_interruptible(&Pend->PrWaitQ);

	write_unlock_irqrestore(&EGroup->EgiLock, Flags);
	return EV_NOERR;
}

int
EvGroupSendEvent(EvUserID_t senderID, EvPri_t pri,
		 EvGroupID_t groupID, EvGroupID_t memberID,
		 EvAccess_t accessCode,
		 EvClassID_t classID, EvEventID_t eventID,
		 int info0, int info1, int info2, int info3,
		 EvDest_t *EDest, int dataLen, void *data)
{
	int RetVal;

	if (EDest->EdCB != NULL) {
		RetVal = EV_NOERR;
		EDest->EdCB(senderID, pri, groupID, memberID, classID, eventID,
			    info0, info1, info2, info3, dataLen, data);
	} else {
		RetVal = EvGenEvent(senderID, pri, groupID, memberID, accessCode,
			   classID, eventID, info0, info1, info2, info3,
			   EDest, dataLen, data);
	}

	return RetVal;
}

EXPORT_SYMBOL(EvCreateGroup);
EXPORT_SYMBOL(EvDeleteGroup);
EXPORT_SYMBOL(EvAttachGroup);
EXPORT_SYMBOL(EvDetachGroup);
EXPORT_SYMBOL(EvGetGroupID);
EXPORT_SYMBOL(EvGetGroupName);
EXPORT_SYMBOL(EvGetMemberID);
EXPORT_SYMBOL(EvGetMemberName);
