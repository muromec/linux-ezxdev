/*
 * event_core.c
 *
 * Core intialization and driver access routines for the eventing mechanism.
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
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/event.h>
#include <linux/proc_fs.h>
#include <linux/devfs_fs_kernel.h>
#include <asm/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "event_core.h"

#define MODVERSIONS

/* Linux Driver prototypes. */
static int event_ioctl(struct inode *, struct file *,
		       unsigned int, unsigned long);
static int event_open(struct inode *, struct file *);
static int event_release(struct inode *, struct file *);
static int event_flush(struct file *);
static int event_fasync(int, struct file *, int);
static unsigned int event_poll(struct file *, poll_table *);

static struct  file_operations event_fops = {
	ioctl:    event_ioctl,
	open:     event_open,
	release:  event_release,
	flush:    event_flush,
	fasync:   event_fasync,
	poll:	  event_poll,
};

void EvFillFreeList(void);
spinlock_t EventAllocLock = SPIN_LOCK_UNLOCKED;

EvPacket_t *EvFreeList;

int EvMajor;

#ifdef CONFIG_PROC_FS
/* /proc entry routines. */
static int EvUserReadProc(char *, char **, off_t, int);
static int EvEventsReadProc(char *, char **, off_t, int);

struct proc_dir_entry *EvProcDir;
#endif

int             
event_init(void)
{
	int RetVal;
	int LocalUserID;
	unsigned short GroupID;

	printk("%s %s\n", "Event Broker Driver (C)",
	       "2001,2002 MontaVista Software (source@mvista.com)");

	/* On every system the first groupID assigned is local.  Other things
	 * may add others later but this is the first.
	 */
	if ((RetVal = EvRegisterUser("LocalControl", &LocalUserID)) < 0) {
		return -EACCES;
	}

	if ((RetVal = EvCreateGroup("Local", "Local", EG_LOCAL,
				    EAC_NONE, EAC_NONE,
				    EV_NUM_LOCAL_HASHES, 0, 0, &GroupID))) {
		EvUnRegisterUser(LocalUserID);
		return -ENOSPC;
	}

	/* Event packets will be often used and reused,  To prevent memory
	 * fragmentation a bucket of them will be allocated 1 page memory
	 * usage at a time and placed on a free list.  If the list gets empty
	 * a new bucket will be allocated in addition.
	 */
	EvFillFreeList();

	if ((EvMajor = devfs_register_chrdev(0, "Event Manager",
					     &event_fops)) < 0) {
		printk("Could not register the event manager\n");
		return -ENODEV;
	}

	devfs_register(NULL, "event", DEVFS_FL_AUTO_DEVNUM, 0, 0,
		       S_IFCHR | S_IRUGO | S_IWUGO, &event_fops, NULL);

#ifdef CONFIG_PROC_FS
	EvProcDir = proc_mkdir("events", NULL);
	EvProcDir->owner = THIS_MODULE;
	create_proc_info_entry("users", 0, EvProcDir, EvUserReadProc);
	create_proc_info_entry("events", 0, EvProcDir, EvEventsReadProc);
#endif
	return 0;
}

/*
 * Linux Driver ioctl access point.
 */
static int
event_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	EvReq_t EventInfo;
	EvUserInfo_t *UserInfo = (EvUserInfo_t *)file->private_data;
	EvGroup_t GroupID;
	EvClass_t ClassID;
	EvUser_t UserID;
	EvPacket_t Packet;
	EvPacket_t *TmpPacket;
	EvAccessCode_t Access;
	char *TmpString = NULL;
	int RetVal;
	void *FreeData;
	unsigned long Flags;

	switch (cmd) {
	case EV_REG_USER:
		/*
		 * Register a user process user ID.
		 */
		if (UserInfo->EuiID != 0) {
			return -EV_ERROR_USER_EXISTS;
		}

		if (copy_from_user((int *)&UserID,
				   (void *)arg, sizeof(UserID))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((RetVal = EvRegisterUser(UserID.EuName,
					     &UserID.EuID)) < 0) {
			return RetVal;
		}

		/* Save the ID in the userinfo structure. */
		UserInfo->EuiID = UserID.EuID;

		if (copy_to_user((void *)arg, (int *)&UserID, sizeof(UserID))) {
			return -EV_ERROR_INVALID_COPY_OUT;
		}
		return EV_NOERR;

	case EV_UNREG_USER:
		/*
		 * Unregister the user process from the event broker.
		 */

		/* Has this user been registered? */
		if (UserInfo->EuiID == 0) {
			return -EV_ERROR_USER_EXISTS;
		}

		if ((RetVal = EvUnRegisterUser(UserInfo->EuiID)) < 0) {
			return RetVal;
		}

		UserInfo->EuiID = 0;
		return EV_NOERR;

	case EV_SET_Q_LIMIT:
		/*
		 * Set maximum number of queued events for a userID.
		 */

		/* Has this user been registered? */
		if (UserInfo->EuiID == 0) {
			return -EV_ERROR_USER_EXISTS;
		}

		UserInfo->EuiQLimit = (unsigned int)arg;
		return EV_NOERR;


	case EV_SET_SYS_Q_LIMIT:
		/*
		 * Set system wide value of maximum number of queued events
		 * per userID.
		 */

		/* must be su (prevent denial of service attack) */
		if (current->uid != 0) {
			return -EV_ERROR_NOT_SU;
		}

		EvSysQLimit = (unsigned int)arg;
		return EV_NOERR;


	case EV_GET_USER_ID:
		/*
		 * Return the user ID for a user level process from the
		 * specified name string.
		 */
		if (copy_from_user((int *)&UserID,
				   (void *)arg, sizeof(UserID))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((RetVal = EvGetUserID(UserID.EuGroupID,
					  UserID.EuMemberID,
					  UserID.EuName,
					  &UserID.EuID)) < 0) {
			return RetVal;
		}

		if (copy_to_user((void *)arg, (int *)&UserID, sizeof(UserID))) {
			return -EV_ERROR_INVALID_COPY_OUT;
		}
		return EV_NOERR;

	case EV_COMP_USER_ID:
		/*
		 * Used by connection daemon to complete the get user
		 * ID request to the remote master.
		 */
		if (copy_from_user((int *)&Packet,
				   (void *)arg, sizeof(EvPacket_t))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((Packet.EpDataLen == 0) || (Packet.EpData == NULL)) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((TmpString =
			kmalloc(Packet.EpDataLen + 1, GFP_KERNEL)) == NULL) {
			return -EV_ERROR_MEM_ALLOC;
		}
       
		if (copy_from_user((int *)TmpString,
				   (void *)Packet.EpData, Packet.EpDataLen)) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		RetVal = EvCompGetUserID(Packet.EpGroupID, Packet.EpMemberID,
				         TmpString,
					 Packet.EpInfo[0], Packet.EpInfo[1]);
		
		kfree(TmpString);
		return RetVal;

	case EV_GET_USER_NAME:
		/*
		 * Return the name string used to register the user ID.
		 */
		if(copy_from_user((int *)&UserID, (void *)arg, sizeof(UserID))){
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((RetVal = EvGetUserName(UserID.EuGroupID,
					    UserID.EuMemberID,
					    UserID.EuID,
					    UserID.EuName)) <0) {
			return RetVal;
		}

		if (copy_to_user((void *)arg, (int *)&UserID, sizeof(UserID))) {
			return -EV_ERROR_INVALID_COPY_OUT;
		}
		return 0;

	case EV_COMP_USER_NAME:
		/*
		 * Used by connection daemon to complete the get user
		 * Name request to the remote master.
		 */
		if (copy_from_user((int *)&Packet,
				   (void *)arg, sizeof(EvPacket_t))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((Packet.EpDataLen == 0) || (Packet.EpData == NULL)) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((TmpString =
			kmalloc(Packet.EpDataLen + 1, GFP_KERNEL)) == NULL) {
			return -EV_ERROR_MEM_ALLOC;
		}
       
		if (copy_from_user((int *)TmpString,
				   (void *)Packet.EpData, Packet.EpDataLen)) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		RetVal = EvCompGetUserName(Packet.EpGroupID, Packet.EpMemberID,
					 Packet.EpInfo[0], Packet.EpInfo[1],
				         TmpString);
		
		kfree(TmpString);
		return RetVal;

	case EV_CREATE_GROUP:
		/*
		 * Access for a user process to create an event group.
		 */
		if (copy_from_user((int *)&GroupID,
				   (void *)arg, sizeof(GroupID))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if (GroupID.EgHashSize == 0) {
			GroupID.EgHashSize = EV_NUM_LOCAL_HASHES;
		}

		if ((RetVal = EvCreateGroup(GroupID.EgGroupName,
					    GroupID.EgMemberName,
					    GroupID.EgType,
					    GroupID.EgGroupAccessCode,
					    GroupID.EgControlAccessCode,
					    GroupID.EgHashSize,
					    GroupID.EgMasterID,
					    GroupID.EgMemberID,
					    &GroupID.EgGroupID)) <0) {
			return RetVal;
		}

		if(copy_to_user((void *)arg, (int *)&GroupID, sizeof(GroupID))){
			return -EV_ERROR_INVALID_COPY_OUT;
		}
		return 0;

	case EV_DELETE_GROUP:
		/*
		 * Access for a user process to remove an event group.
		 */
		if (copy_from_user((int *)&GroupID,
				   (void *)arg, sizeof(GroupID))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		return EvDeleteGroup(GroupID.EgGroupID,
				     GroupID.EgGroupAccessCode);

	case EV_ATTACH_GROUP:
		/*
		 * Access for a user level process to attach as a member to
		 * an event group master on this system.
		 */
		if (copy_from_user((int *)&GroupID,
				   (void *)arg, sizeof(GroupID))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((RetVal = EvAttachGroup(GroupID.EgMasterID,
					    GroupID.EgMemberName,
					    GroupID.EgGroupAccessCode,
					    &GroupID.EgMemberID)) < 0) {
			return RetVal;
		}

		if(copy_to_user((void *)arg, (int *)&GroupID, sizeof(GroupID))){
			return -EV_ERROR_INVALID_COPY_OUT;
		}
		return 0;

	case EV_DETACH_GROUP:
		/*
		 * Access for a user process to detach as a member from a
		 * local event group master.
		 */
		if (copy_from_user((int *)&GroupID,
				   (void *)arg, sizeof(GroupID))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		return EvDetachGroup(GroupID.EgGroupID, GroupID.EgMemberID,
				     GroupID.EgGroupAccessCode);

	case EV_GET_GROUP_ID:
		/*
		 * Access for a user process to query for the event group ID
		 * associeated with the event group indentifier string.
		 */
		if (copy_from_user((int *)&GroupID,
				   (void *)arg, sizeof(GroupID))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((RetVal = EvGetGroupID(GroupID.EgGroupName,
					   &(GroupID.EgGroupID))) < 0) {
			return RetVal;
		}

		if(copy_to_user((void *)arg, (int *)&GroupID, sizeof(GroupID))){
			return -EV_ERROR_INVALID_COPY_OUT;
		}
		return 0;

	case EV_GET_GROUP_NAME:
		/*
		 * Access for a user process to return the identifying name
		 * string for the request event group ID.
		 */
		if (copy_from_user((int *)&GroupID,
				   (void *)arg, sizeof(GroupID))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((RetVal = EvGetGroupName(GroupID.EgGroupID,
					      GroupID.EgGroupName)) < 0) {
			return RetVal;
		}

		if(copy_to_user((void *)arg, (int *)&GroupID, sizeof(GroupID))){
			return -EV_ERROR_INVALID_COPY_OUT;
		}
		return 0;

	case EV_GET_MEMBER_ID:
		/*
		 * Access for a user process to query for the event group ID
		 * associated with the member name sting as a part of an event
		 * group.
		 */
		if (copy_from_user((int *)&GroupID,
				   (void *)arg, sizeof(GroupID))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((RetVal = EvGetMemberID(GroupID.EgGroupID,
			       		    GroupID.EgMemberName,
			       		    &(GroupID.EgMemberID))) < 0) {
			return RetVal;
		}

		if((RetVal = copy_to_user((void *)arg,
					  (int *)&GroupID, sizeof(GroupID)))) {
			return -EV_ERROR_INVALID_COPY_OUT;
		}
		return 0;

	case EV_COMP_MEMBER_ID:
		/*
		 * Access for a user process to query for the event group ID
		 * associated with the member name sting as a part of an event
		 * group.
		 */
		if (copy_from_user((int *)&Packet,
				   (void *)arg, sizeof(EvPacket_t))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((Packet.EpDataLen == 0) || (Packet.EpData == NULL)) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((TmpString =
			kmalloc(Packet.EpDataLen + 1, GFP_KERNEL)) == NULL) {
			return -EV_ERROR_MEM_ALLOC;
		}
       
		if (copy_from_user((int *)TmpString,
				   (void *)Packet.EpData, Packet.EpDataLen)) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		RetVal = EvCompGetMemberID(Packet.EpGroupID, TmpString,
					   Packet.EpInfo[0], Packet.EpInfo[1]);
		
		kfree(TmpString);
		return RetVal;

	case EV_GET_MEMBER_NAME:
		/*
		 * Access for a user process to query for the event group name
		 * string associated with the member ID as a part of an event
		 * group.
		 */
		if (copy_from_user((int *)&GroupID,
				   (void *)arg, sizeof(GroupID))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((RetVal = EvGetMemberName(GroupID.EgGroupID,
					     GroupID.EgMemberID,
					     GroupID.EgMemberName)) < 0) {
			return RetVal;
		}

		if(copy_to_user((void *)arg, (int *)&GroupID, sizeof(GroupID))){
			return -EV_ERROR_INVALID_COPY_OUT;
		}
		return 0;

	case EV_COMP_MEMBER_NAME:
		/*
		 * Access for a user process to query for the event group ID
		 * associated with the member name sting as a part of an event
		 * group.
		 */
		if (copy_from_user((int *)&Packet,
				   (void *)arg, sizeof(EvPacket_t))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((Packet.EpDataLen == 0) || (Packet.EpData == NULL)) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((TmpString =
			kmalloc(Packet.EpDataLen + 1, GFP_KERNEL)) == NULL) {
			return -EV_ERROR_MEM_ALLOC;
		}
       
		if (copy_from_user((int *)TmpString,
				   (void *)Packet.EpData, Packet.EpDataLen)) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		RetVal = EvCompGetMemberName(Packet.EpGroupID,
					     (EvGroupID_t)Packet.EpInfo[0],
					      Packet.EpInfo[1], TmpString);
		kfree(TmpString);
		return RetVal;

	case EV_REG_EVENT_CLASS:
		/*
		 * User process event class registeration access.
		 */
		if (copy_from_user((int *)&ClassID,
				   (void *)arg, sizeof(ClassID))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((RetVal = EvRegisterEventClass(ClassID.EcGroupID,
						   ClassID.EcName,
						   ClassID.EcGroupAccessCode,
						   ClassID.EcClassAccessCode,
						   &ClassID.EcClassID)) < 0) {
			return RetVal;
		}

		if(copy_to_user((void *)arg, (int *)&ClassID, sizeof(ClassID))){
			return -EV_ERROR_INVALID_COPY_OUT;
		}
		return RetVal;

	case EV_REM_REG_EVENT_CLASS:
		/*
		 * Access for a user process to query for the event group ID
		 * associated with the member name sting as a part of an event
		 * group.
		 */
		if (copy_from_user((int *)&Packet,
				   (void *)arg, sizeof(EvPacket_t))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((Packet.EpDataLen == 0) || (Packet.EpData == NULL)) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((TmpString =
			kmalloc(Packet.EpDataLen + 1, GFP_KERNEL)) == NULL) {
			return -EV_ERROR_MEM_ALLOC;
		}
       
		if (copy_from_user((int *)TmpString,
				   (void *)Packet.EpData, Packet.EpDataLen)) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		RetVal = EvRemoteRegisterEventClass(Packet.EpGroupID,
						    TmpString,
						    Packet.EpInfo[0],
						    Packet.EpInfo[1],
						    Packet.EpInfo[2],
						    Packet.EpInfo[3]);
		kfree(TmpString);
		return RetVal;

	case EV_UNREG_EVENT_CLASS:
		/*
		 * Event calss unregisteration access for a user process.
		 */
		if (copy_from_user((int *)&ClassID,
				   (void *)arg, sizeof(ClassID))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		return EvUnRegisterEventClass(ClassID.EcGroupID,
					      ClassID.EcClassID,
					      ClassID.EcGroupAccessCode);

	case EV_REM_UNREG_EVENT_CLASS:
		/*
		 * Access for a user process to query for the event group ID
		 * associated with the member name sting as a part of an event
		 * group.
		 */
		if (copy_from_user((int *)&Packet,
				   (void *)arg, sizeof(EvPacket_t))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		return EvRemoteUnRegisterEventClass(Packet.EpGroupID,
						    Packet.EpInfo[0],
						    Packet.EpInfo[1],
						    Packet.EpInfo[2]);

	case EV_GET_EVENT_CLASS:
		/*
		 * User process access to return the class ID associated with
		 * the name string used to register it.
		 */
		if (copy_from_user((int *)&ClassID,
				   (void *)arg, sizeof(ClassID))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((RetVal = EvGetEventClassID(ClassID.EcGroupID,
					      ClassID.EcName,
					      &ClassID.EcClassID)) < 0) {
			return RetVal;
		}

		if(copy_to_user((void *)arg, (int *)&ClassID, sizeof(ClassID))){
			return -EV_ERROR_INVALID_COPY_OUT;
		}
		return 0;

	case EV_GET_EVENT_CLASS_NAME:
		/*
		 * User process access to return the name string associated with
		 * a registered class ID.
		 */
		if (copy_from_user((int *)&ClassID,
				   (void *)arg, sizeof(ClassID))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((RetVal = EvGetEventClassName(ClassID.EcGroupID,
						  ClassID.EcClassID,
						  ClassID.EcName)) < 0) {
			return RetVal;
		}

		if(copy_to_user((void *)arg, (int *)&ClassID, sizeof(ClassID))){
			return -EV_ERROR_INVALID_COPY_OUT;
		}
		return 0;

	case EV_SUBSCRIBE_EVENT:
		/*
		 * Access for a user process to subscribe to receieve an event.
		 */
		if (copy_from_user((int *)&EventInfo,
				   (void *)arg, sizeof(EvReq_t))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		return EvUserSubscribeEvent(UserInfo, &EventInfo);

	case EV_UNSUBSCRIBE_EVENT:
		/*
		 * Access for a user process to unsubscribe from receiving an
		 * event.
		 */
		if (copy_from_user((int *)&EventInfo,
				   (void *)arg, sizeof(EvReq_t))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		return EvUserUnSubscribeEvent(UserInfo, &EventInfo);

	case EV_SUBSCRIBE_GROUP_EV:
		/*
		 * Access for a user process to subscribe to receieve an event.
		 */
		if (copy_from_user((int *)&EventInfo,
				   (void *)arg, sizeof(EvReq_t))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		return EvUserSubscribeGroupEvents(UserInfo, &EventInfo);

	case EV_UNSUBSCRIBE_GROUP_EV:
		/*
		 * Access for a user process to unsubscribe from receiving an
		 * event.
		 */
		if (copy_from_user((int *)&EventInfo,
				   (void *)arg, sizeof(EvReq_t))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		return EvUnSubscribeGroupEvents(UserInfo->EuiID,
						EventInfo.ErGroupID);

	case EV_SEND_EVENT:
		/*
		 * Access for a user process to send an event.
		 */
		if (copy_from_user((int *)&Packet,
				   (void *)arg, sizeof(EvPacket_t))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((Packet.EpDataLen != 0) && (Packet.EpData != NULL)) {
			TmpString = kmalloc(Packet.EpDataLen + 1, GFP_KERNEL);

			if (copy_from_user((int *)TmpString,
					   (void *)Packet.EpData,
					   Packet.EpDataLen)) {
				return -EV_ERROR_INVALID_COPY_IN;
			}
		}

		RetVal = EvSendEvent(UserInfo->EuiID, Packet.EpPri,
				   Packet.EpGroupID, Packet.EpClassID,
				   Packet.EpEventID, Packet.EpAccessCode,
				   Packet.EpInfo[0], Packet.EpInfo[1],
				   Packet.EpInfo[2], Packet.EpInfo[3],
				   Packet.EpDataLen, TmpString);

		if (TmpString) {
			kfree(TmpString);
		}

		return RetVal;

	case EV_REM_SEND_EVENT:
		/*
		 * Access for a user process to send an event.
		 */
		if (copy_from_user((int *)&Packet,
				   (void *)arg, sizeof(EvPacket_t))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		if ((Packet.EpDataLen != 0) && (Packet.EpData != NULL)) {
			TmpString = kmalloc(Packet.EpDataLen + 1, GFP_KERNEL);

			if (copy_from_user((int *)TmpString,
					   (void *)Packet.EpData,
					   Packet.EpDataLen)) {
				return -EV_ERROR_INVALID_COPY_IN;
			}
		}

		RetVal =  EvRemoteSendEvent(UserInfo->EuiID, Packet.EpMemberID,
					    Packet.EpPri,
					    Packet.EpGroupID, Packet.EpClassID,
					    Packet.EpEventID,
					    Packet.EpAccessCode,
					    Packet.EpInfo[0], Packet.EpInfo[1],
					    Packet.EpInfo[2], Packet.EpInfo[3],
					    Packet.EpDataLen, TmpString);

		if (TmpString) {
			kfree(TmpString);
		}

		return RetVal;

	case EV_GET_EVENT:
		/*
		 * Access for a user level process to retrieve an event.
		 */
		if (copy_from_user((int *)&Packet,
				   (void *)arg, sizeof(EvPacket_t))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		/* Keep doing while wait for event is indicated and this
		 * is a polled device.
		 */
		spin_lock_irqsave(&UserInfo->EuiLock, Flags);
		do {
			TmpPacket = UserInfo->EuiHead;

			if (TmpPacket) {
			    if ((TmpPacket->EpDataLen != 0) &&
				(TmpPacket->EpData != NULL) &&
				(Packet.EpData != NULL)) {
				if (Packet.EpDataLen <
				    TmpPacket->EpDataLen) {
					TmpPacket->EpDataLen = Packet.EpDataLen;
				}

				if (copy_to_user((void *)Packet.EpData,
						 (int *)TmpPacket->EpData,
						 TmpPacket->EpDataLen)) {
				    spin_unlock_irqrestore(&UserInfo->EuiLock,
						           Flags);
				    return -EV_ERROR_INVALID_COPY_OUT;
				}
				FreeData = TmpPacket->EpData;
				TmpPacket->EpData = Packet.EpData;
	
				kfree(FreeData);
			    } else {
				/* Insure sanity if the receiver specified not
				 * data length.
				 */
				TmpPacket->EpDataLen = 0;
			    }

			    if (copy_to_user((void *)arg, (int *)TmpPacket,
							sizeof(EvPacket_t))) {
				spin_unlock_irqrestore(&UserInfo->EuiLock,
						       Flags);
				return -EV_ERROR_INVALID_COPY_OUT;
			    }

			    UserInfo->EuiQCount--;

			    UserInfo->EuiHead = TmpPacket->EpNext;
			    if (UserInfo->EuiHead == NULL) {
				UserInfo->EuiTail = NULL;
			    }

			    spin_unlock_irqrestore(&UserInfo->EuiLock, Flags);
			    EvFree(TmpPacket, 0);
			    return 0;
			} 
			if (!Packet.EpFlag) {
			    spin_unlock_irqrestore(&UserInfo->EuiLock, Flags);
			    return -EV_ERROR_NO_EVENT;
			}

			spin_unlock_irqrestore(&UserInfo->EuiLock, Flags);
			interruptible_sleep_on(&UserInfo->EuiWaitQ);
			spin_lock_irqsave(&UserInfo->EuiLock, Flags);

			if (UserInfo->EuiHead == NULL) {
			    spin_unlock_irqrestore(&UserInfo->EuiLock, Flags);
			    return -EV_ERROR_WAIT_INTERRUPTED;
			}
		} while (Packet.EpFlag);

	case EV_SET_GROUP_ACCESSCODE:
		/*
		 * Access for a user process to send an event.
		 */
		if (copy_from_user((int *)&Access,
				   (void *)arg, sizeof(EvAccessCode_t))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		return EvSetGroupAccessCode(Access.EaGroupID,
					    Access.EaOldAccessCode,
					    Access.EaNewAccessCode);

	case EV_SET_CLASS_ACCESSCODE:
		/*
		 * Access for a user process to send an event.
		 */
		if (copy_from_user((int *)&Access,
				   (void *)arg, sizeof(EvAccessCode_t))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		return EvSetClassAccessCode(Access.EaGroupID,
					    Access.EaClassID,
					    Access.EaOldAccessCode,
					    Access.EaNewAccessCode);

	case EV_SYNC_TO_MEMBER:
		if (copy_from_user((int *)&GroupID,
				   (void *)arg, sizeof(GroupID))) {
			return -EV_ERROR_INVALID_COPY_IN;
		}

		return  EvSyncToMember(GroupID.EgGroupID,
				       GroupID.EgGroupAccessCode,
				       GroupID.EgMemberID);
	default:
		return -EV_ERROR_INVALID_REQEST;
	}
}

/*
 * Linux Driver open access point.
 */
static int
event_open(struct inode *inode, struct file *file)
{
	EvUserInfo_t *UserInfo =
		(EvUserInfo_t *)kmalloc(sizeof(EvUserInfo_t), GFP_KERNEL);

	/* Fill in the user data and link it to the private data area
	 * for the user process.
	 */
	UserInfo->EuiID = 0;
	UserInfo->EuiPid = current->pid;
	UserInfo->EuiType = ED_USER_POLL;
	init_waitqueue_head(&UserInfo->EuiWaitQ);
	UserInfo->EuiFasyncPtr = (struct fasync_struct *)NULL;
	UserInfo->EuiHead = (EvPacket_t *)NULL;
	UserInfo->EuiTail = (EvPacket_t *)NULL;
	UserInfo->EuiLock = SPIN_LOCK_UNLOCKED;
	UserInfo->EuiQCount = 0;
	UserInfo->EuiQLimit = 0;

	file->private_data = (void *)UserInfo;
	return 0;
}

/*
 * Linux Driver close access point.
 */
static int
event_release(struct inode *inode, struct file *file)
{
	EvPacket_t *Tmp1EventPtr;
	EvPacket_t *Tmp2EventPtr;
	EvUserInfo_t *UserInfo = (EvUserInfo_t *)file->private_data;

	/* If UserInfo is null then a previous flush routine has removed it */
	if (UserInfo == NULL) {
		return 0;
	}

	/* It the user had registered an ID then get his information
	 * out of the system.
	 */
	if (UserInfo->EuiID != 0) {
		EvUnRegisterUser(UserInfo->EuiID);

		/* Free any pending events scheduled for this user. */
		Tmp1EventPtr = UserInfo->EuiHead;
		while (Tmp1EventPtr) {
			Tmp2EventPtr = Tmp1EventPtr;
			Tmp1EventPtr = Tmp1EventPtr->EpNext;
			EvFree(Tmp2EventPtr, 1);
		}
	}

	/* Make him gone. */
	kfree(UserInfo);
	return 0;
}

/*
 * Linux Driver flush access point.
 */
static int
event_flush(struct file *file)
{
	EvPacket_t *Tmp1EventPtr;
	EvPacket_t *Tmp2EventPtr;
	EvUserInfo_t *UserInfo = (EvUserInfo_t *)file->private_data;

	/* If UserInfo is null then a previous flush routine has removed it */
	if (UserInfo == NULL) {
		return 0;
	}

	/* It the user had registered an ID then get his information
	 * out of the system.
	 */
	if (UserInfo->EuiPid == current->pid) {
		if (UserInfo->EuiID != 0)  {
			EvUnRegisterUser(UserInfo->EuiID);

			/* Free any pending events scheduled for this user. */
			Tmp1EventPtr = UserInfo->EuiHead;
			while (Tmp1EventPtr) {
				Tmp2EventPtr = Tmp1EventPtr;
				Tmp1EventPtr = Tmp1EventPtr->EpNext;
				EvFree(Tmp2EventPtr, 1);
			}
		}

		/* Make him gone. */
		file->private_data = NULL;
		kfree(UserInfo);
	}

	return 0;
}

/*
 * Linux Driver fasync access point.
 */
static int
event_fasync(int fd, struct file *fp, int on)
{
	unsigned long Flags;
	EvUserInfo_t *UserInfo = (EvUserInfo_t *)fp->private_data;

	spin_lock_irqsave(&UserInfo->EuiLock, Flags);
	fasync_helper(fd, fp, on, &UserInfo->EuiFasyncPtr);

	/* If the on flag is set then indicate events are
	 * to be delivered asyncronouse by signalling the process
	 * an event is waiting.  Othersie indicate the process will
	 * poll for events.
	 */
	if (on) {
		UserInfo->EuiType = ED_USER_ASYNC;
	} else {
		UserInfo->EuiType = ED_USER_POLL;
	}

	spin_unlock_irqrestore(&UserInfo->EuiLock, Flags);
	return 0;
}

static unsigned int
event_poll(struct file *fp, poll_table *pp)
{
	EvUserInfo_t *UserInfo = (EvUserInfo_t *)fp->private_data;

	poll_wait(fp, &UserInfo->EuiWaitQ, pp);

	if (UserInfo->EuiHead != NULL) {
		return (POLLIN | POLLRDNORM);
	}

	return 0;
}

#ifdef CONFIG_PROC_FS
/*
 * The EvUserReadProc() routine displays the list of registered users
 * on this system in the /proc file system.
 */
static int
EvUserReadProc(char * buf, char ** start, off_t offset, int len)
{
	char *Page = buf;
	EvKernelInfo_t *TmpUser;
	unsigned long Flags;

	read_lock_irqsave(&EvUsersLock, Flags);
	TmpUser = EvUsersHead;
	Page += sprintf(Page, "\nEvent Users List\n");
	Page += sprintf(Page, "------------------------------\n");
	while (TmpUser) {
		Page += sprintf(Page, "%3d: %s\n", TmpUser->EkiID, TmpUser->EkiName);
		TmpUser = TmpUser->EkiNext;
	}
		
	read_unlock_irqrestore(&EvUsersLock, Flags);
	Page += sprintf(Page, "\n");

	return (int)(Page - buf);
}

char EvGroupType[] = {'L', 'M', 'm'};
/*
 * The EvEventsReadProc routine displays the list of registered event
 * classes and any pending events on this system in the /proc file system.
 */
static int
EvEventsReadProc(char * buf, char ** start, off_t offset, int len)
{
	EvGroupInfo_t *EGroup;
	int Hash;
	EvClassInfo_t *HashBase;
	EvDestBase_t *EventBase;
	EvDest_t *Dest;
	EvPacket_t *Packet;
	EvMemberList_t *Member;
	char *Page = buf;
	char Scratch[64];

	Page += sprintf(Page, "\nEvent Information:\n");
	Page += sprintf(Page, "------------------------------------------------------------------------\n");
	Page += sprintf(Page, "System Queue Limit %d\n", EvSysQLimit);
	for (EGroup = EvGroupHead; EGroup != NULL; EGroup = EGroup->EgiNext) {
	    Page += sprintf(Page, "%c %s(%d) %s(%d)\n",
			    EvGroupType[EGroup->EgiType],
			    EGroup->EgiName, EGroup->EgiID,
			    EGroup->EgiMemberName, EGroup->EgiMemberID);

	    if (EGroup->EgiGroupDest.EdID == 0) {
		    Page += sprintf(Page, "    No remote control agent\n");
	    } else if (EGroup->EgiGroupDest.EdUinfo == NULL) {
		    Page += sprintf(Page,
				    "    Kernel remote control agent ID: %d\n",
				    EGroup->EgiGroupDest.EdID);
	    } else {
		    Page += sprintf(Page,
				    "    User remote control agent ID: %d\n",
				    EGroup->EgiGroupDest.EdID);
	    }


	    for (Hash = 0; Hash < EGroup->EgiHashSize; Hash++) {
		HashBase = &EGroup->EgiClassHash[Hash];

		while (HashBase->EciNext) {
		    Page += sprintf(Page, "    Class %s(%d) Use Count %d\n",
				    HashBase->EciName, HashBase->EciClass,
				    HashBase->EciUseCount);

		    EventBase = HashBase->EciEventQ;
		    while (EventBase->EdbNext != NULL) {
			Page += sprintf(Page, "        Event %d count %d\n",
					EventBase->EdbEvent,
					EventBase->EdbUseCount);

			Dest = &EventBase->EdbDestQ;
			while (Dest->EdNext != NULL) {
			    EvGetUserName(EG_LOCAL, EG_LOCAL,
					  Dest->EdID, Scratch);
			    Page += sprintf(Page,
					"        User %d \"%s\" Priority %d ",
			    		Dest->EdID, Scratch, Dest->EdPri);
	
			    if (Dest->EdUinfo) {
				Page += sprintf(Page, "Q count/limit %d/%d\n",
						Dest->EdUinfo->EuiQCount,
						Dest->EdUinfo->EuiQLimit);
				Page += sprintf(Page, "User Application\n");
				Packet = Dest->EdUinfo->EuiHead;
	
				while (Packet) {
				    Page += sprintf(Page,
			"Sender %d info 0x%x:0x%x:0x%x:0x%x data len %d\n",
						    Packet->EpSenderID,
						    Packet->EpInfo[0],
						    Packet->EpInfo[1],
						    Packet->EpInfo[2],
						    Packet->EpInfo[3],
						    Packet->EpDataLen);
				    Packet = Packet->EpNext;
				}
			    } else if (Dest->EdCB) {
				Page += sprintf(Page, "Kernel Call Back\n");
			    } else {
				Page += sprintf(Page, "Q count/limit %d/%d\n",
						Dest->EdKinfo->EkiQCount,
						Dest->EdKinfo->EkiQLimit);
				Page += sprintf(Page, "Kernel Queued\n");
				Packet = Dest->EdKinfo->EkiHead;

				while (Packet) {
				    Page += sprintf(Page,
			"Sender %d info 0x%x:0x%x:0x%x:0x%x data len %d\n",
						    Packet->EpSenderID,
						    Packet->EpInfo[0],
						    Packet->EpInfo[1],
						    Packet->EpInfo[2],
						    Packet->EpInfo[3],
						    Packet->EpDataLen);
				    Packet = Packet->EpNext;
				}
			    }

			    Dest = Dest->EdNext;
			}

			EventBase = EventBase->EdbNext;
		    }

		    HashBase = HashBase->EciNext;
		}
	    }

	    Member = EGroup->EgiMembers;
	    while (Member) {
		Page += sprintf(Page, "    Member %s ID %d\n",
				Member->EslName, Member->EslID);
		Member = Member->EslNext;
	    }
	}
	return (int)(Page - buf);
}
#endif

/*
 * Used by init code to create free list of emergency packet buffers.
 */
void
EvFillFreeList(void)
{
	int Loop;
	EvPacket_t *TmpBase;

	EvFreeList = kmalloc(4096, GFP_KERNEL);
	if (EvFreeList == NULL)
		return;

	TmpBase = EvFreeList;

	for (Loop = 0; Loop < (4096 / sizeof(EvPacket_t))-1; Loop++) {
		TmpBase->EpEventID = 0;
		TmpBase->EpClassID = 0;
		TmpBase->EpGroupID = 0;
		TmpBase->EpMemberID = 0;
		TmpBase->EpTimeStamp.tv_sec = 0;
		TmpBase->EpTimeStamp.tv_usec = 0;
		TmpBase->EpInfo[0] = 0;
		TmpBase->EpInfo[1] = 0;
		TmpBase->EpInfo[2] = 0;
		TmpBase->EpInfo[3] = 0;
		TmpBase->EpPri = 0;
		TmpBase->EpDataLen = 0;
		TmpBase->EpData = NULL;
		TmpBase->EpNext = TmpBase + 1;
		TmpBase++;
	}
	TmpBase->EpEventID = 0;
	TmpBase->EpClassID = 0;
	TmpBase->EpGroupID = 0;
	TmpBase->EpMemberID = 0;
	TmpBase->EpTimeStamp.tv_sec = 0;
	TmpBase->EpTimeStamp.tv_usec = 0;
	TmpBase->EpInfo[0] = 0;
	TmpBase->EpInfo[1] = 0;
	TmpBase->EpInfo[2] = 0;
	TmpBase->EpInfo[3] = 0;
	TmpBase->EpPri = 0;
	TmpBase->EpDataLen = 0;
	TmpBase->EpData = NULL;
	TmpBase->EpNext = NULL;
}

/*
 * Used by EvGenEvent to allocate packet memory space for an event.
 */
EvPacket_t *
EvAllocate(void)
{
	EvPacket_t *TmpBase;

	spin_lock(&EventAllocLock);
	if ((TmpBase = kmalloc(sizeof(EvPacket_t), GFP_ATOMIC)) != NULL) {
		/* Still enough memory to not worry about using it. */
		TmpBase->EpFlag = 0;
		spin_unlock(&EventAllocLock);
		return  TmpBase;
	}

	/* Get a packet from the reserved list. */
	if (EvFreeList == NULL) {
		return NULL;
	}

	TmpBase = EvFreeList;
	EvFreeList = EvFreeList->EpNext;
	TmpBase->EpFlag = EF_EMERGENCY_PACKET;

	spin_unlock(&EventAllocLock);
	return TmpBase;
}

/*
 * Return the memory from an event packet back to the system.
 */
void
EvFree(EvPacket_t *eventPtr, int flag)
{
	unsigned long Flags;

	spin_lock_irqsave(&EventAllocLock, Flags);
	if (flag && (eventPtr->EpDataLen > 0) && (eventPtr->EpData != NULL)) {
		kfree(eventPtr->EpData);
	}

	if (eventPtr->EpFlag & EF_EMERGENCY_PACKET) {
		eventPtr->EpNext = EvFreeList;
		EvFreeList = eventPtr;
	} else {
		kfree(eventPtr);
	}

	spin_unlock_irqrestore(&EventAllocLock, Flags);
	return;
}

/*
 * Queue an event to a subscriber.
 */
int
EvGenEvent(EvUserID_t senderID, EvPri_t pri,
	   EvGroupID_t groupID, EvGroupID_t memberID,
	   EvAccess_t accessCode, EvClassID_t classID, EvEventID_t eventID,
	   int info0, int info1, int info2, int info3,
	   EvDest_t *EDest, int dataLen, void *data)
{
	EvPacket_t *Packet;
	EvPacket_t *PrevPacket;
	EvPacket_t *TmpPacket;
	EvPacket_t *EndPacket;

	Packet = EvAllocate();
	if (Packet == NULL) {
		return -EV_ERROR_MEM_ALLOC;
	}

	Packet->EpSenderID = senderID;
	Packet->EpPri = pri;
	Packet->EpGroupID = groupID;
	Packet->EpMemberID = memberID;
	Packet->EpAccessCode = accessCode;
	Packet->EpClassID = classID;
	Packet->EpEventID = eventID;
	do_gettimeofday(&Packet->EpTimeStamp);
	Packet->EpInfo[0] = info0;
	Packet->EpInfo[1] = info1;
	Packet->EpInfo[2] = info2;
	Packet->EpInfo[3] = info3;
	Packet->EpDataLen = dataLen;

	/* If there is extened data associated with the event then
	 * allocate space to transmit it.  If there is not enough space
	 * then the event is still sent but the Flags field of the event is
	 * set to EF_DATA_FAILURE to indicate the fact.
	 */
	if ((dataLen != 0) && (data != NULL)) {
		if ((Packet->EpData = kmalloc(dataLen + 1, GFP_ATOMIC)) == NULL) {
			Packet->EpDataLen = 0;
			Packet->EpData = NULL;
			Packet->EpFlag |= EF_DATA_FAILURE;
		} else {
			memcpy(Packet->EpData, data, dataLen);
			Packet->EpFlag |= EF_DATA_AVAIL;
		}
	} else {
		Packet->EpData = NULL;
		Packet->EpFlag &= ~(EF_DATA_AVAIL|EF_DATA_FAILURE);
	}

	if (EDest->EdUinfo == NULL) {
		/* Queue in kernel */
		spin_lock(&EDest->EdKinfo->EkiLock);

		if (
		    ((EDest->EdKinfo->EkiQLimit) &&
		     (EDest->EdKinfo->EkiQLimit <= EDest->EdKinfo->EkiQCount)) ||
		    ((EvSysQLimit) &&
		     (EvSysQLimit <= EDest->EdKinfo->EkiQCount))
		   ) {
			EvFree(Packet, 1);
			spin_unlock(&EDest->EdKinfo->EkiLock);
			return -EV_ERROR_QUEUE_FULL;
		}

		TmpPacket = EDest->EdKinfo->EkiHead;
		EndPacket = EDest->EdKinfo->EkiTail;

		if (TmpPacket == NULL) {
			Packet->EpNext          = NULL;
			EDest->EdKinfo->EkiHead = Packet;
			EDest->EdKinfo->EkiTail = Packet;

		} else if (TmpPacket->EpPri < pri) {
			/* Goes to head of the list. */
			Packet->EpNext          = TmpPacket;
			EDest->EdKinfo->EkiHead = Packet;

		} else if (EndPacket->EpPri >= pri) {
			/* Place on tail of list */
			Packet->EpNext          = NULL;
			EndPacket->EpNext       = Packet;
			EDest->EdKinfo->EkiTail = Packet;

		} else {
			/* Got to find the middle somewhere. */
			PrevPacket = TmpPacket;
			while (TmpPacket) {
				if (TmpPacket->EpPri < pri) {
					Packet->EpNext     = TmpPacket;
					PrevPacket->EpNext = Packet;
					break;
				}
				PrevPacket = TmpPacket;
				TmpPacket  = TmpPacket->EpNext;
			}
		}

		EDest->EdKinfo->EkiQCount++;

		wake_up_interruptible(&EDest->EdKinfo->EkiWaitQ);
		spin_unlock(&EDest->EdKinfo->EkiLock);
		return EV_NOERR;
	}

	/* Send up to a processes */
	spin_lock(&EDest->EdUinfo->EuiLock);

	if (
	    ((EDest->EdUinfo->EuiQLimit) &&
	     (EDest->EdUinfo->EuiQLimit <= EDest->EdUinfo->EuiQCount)) ||
	    ((EvSysQLimit) &&
	     (EvSysQLimit <= EDest->EdUinfo->EuiQCount))
	   ) {
		EvFree(Packet, 1);
		spin_unlock(&EDest->EdUinfo->EuiLock);
		return -EV_ERROR_QUEUE_FULL;
	}

	TmpPacket = EDest->EdUinfo->EuiHead;
	EndPacket = EDest->EdUinfo->EuiTail;

	if (TmpPacket == NULL) {
		Packet->EpNext          = NULL;
		EDest->EdUinfo->EuiHead = Packet;
		EDest->EdUinfo->EuiTail = Packet;

	} else if (TmpPacket->EpPri < pri) {
		/* Goes to head of the list. */
		Packet->EpNext          = TmpPacket;
		EDest->EdUinfo->EuiHead = Packet;

	} else if (EndPacket->EpPri >= pri) {
		/* Place on tail of list */
		Packet->EpNext          = NULL;
		EndPacket->EpNext       = Packet;
		EDest->EdUinfo->EuiTail = Packet;

	} else {
		/* Got to find the middle somewhere. */
		PrevPacket = TmpPacket;
		while (TmpPacket) {
			if (TmpPacket->EpPri < pri) {
				Packet->EpNext     = TmpPacket;
				PrevPacket->EpNext = Packet;
				break;
			}
			PrevPacket = TmpPacket;
			TmpPacket  = TmpPacket->EpNext;
		}
	}

	EDest->EdUinfo->EuiQCount++;

	/* First check to see if this is polled file descriptor. */
	if (EDest->EdUinfo->EuiType == ED_USER_POLL) {
		wake_up_interruptible(&EDest->EdUinfo->EuiWaitQ);
		spin_unlock(&EDest->EdUinfo->EuiLock);
		return EV_NOERR;
	}

	wake_up_interruptible(&EDest->EdUinfo->EuiWaitQ);
	kill_fasync(&EDest->EdUinfo->EuiFasyncPtr, SIGIO, POLL_IN);
	spin_unlock(&EDest->EdUinfo->EuiLock);
	return EV_NOERR;
}

int
_EvSendEvent(EvDestBase_t *eventBase, EvUserID_t senderID, EvPri_t pri,
	     EvGroupID_t groupID, EvGroupID_t memberID,
	     EvClassID_t classID, EvEventID_t eventID,
	     int info0, int info1, int info2, int info3,
	     int dataLen, void *data)
{
	EvDest_t *EDest = NULL;
	int RetVal;
	int TmpRetVal;

	RetVal = EV_NOERR;

	while (eventBase->EdbNext != NULL) {
		if ((eventBase->EdbEvent != eventID) &&
		    (eventBase->EdbEvent != 0)) {
			eventBase = eventBase->EdbNext;
			continue;
		}

		EDest = &eventBase->EdbDestQ;

		while (EDest->EdNext != NULL) {
			if (EDest->EdCB != NULL) {
				/*
				 * Things to document:
				 *
				 * Since the class base lock is held while
				 * the call back is being processed, the
				 * callback function is not allowed to do
				 * anything that will modify processin on
				 * this class of event.
				 */
				if ((*EDest->EdCB)(senderID, pri, groupID,
						   memberID, classID, eventID,
						   info0, info1, info2, info3,
						   dataLen, data) == 0) {
					kfree(data);
				}
			} else {
				if ((TmpRetVal = EvGenEvent(senderID, pri, groupID, memberID,
					   -1, classID, eventID,
					   info0, info1, info2, info3,
					   EDest, dataLen, data))) {
					/* indeterminate which error gets
					 * returned, if more than one EDest
					 * results in an error
					 */
					RetVal = TmpRetVal;
				}
			}

			EDest = EDest->EdNext;
		}

		eventBase = eventBase->EdbNext;
	}
	return RetVal;
}

/*
 * The EvSendEvent() function allows users of the event broker to
 * create an event and send it to all subscribers.
 *
 * If the event group indicated does not exist the -EEXIST is returned.  If
 * the class of the event has not been registered then -EINVAL is returned.
 *
 * Otherwise the event is sent to all subscribers and a zero is returned
 * to indicate success.
 */
int
EvSendEvent(EvUserID_t senderID, EvPri_t pri,
	    EvGroupID_t groupID, EvClassID_t classID, EvEventID_t eventID,
	    EvAccess_t accessCode,
	    int info0, int info1, int info2, int info3,
	    int dataLen, void *data)
{
	EvDestBase_t *EventBase;
	unsigned long Flags;
	EvGroupInfo_t *EGroup;
	EvClassInfo_t *HashBase;
	EvClassInfo_t *ClassBase;
	EvGroupID_t MemberID;
	int RetVal;

	read_lock_irqsave(&EvUsersLock, Flags);
	if (EvCheckUser(senderID) == NULL) {
		read_unlock_irqrestore(&EvUsersLock, Flags);
		return -EV_ERROR_USER_EXISTS;
	}

	read_lock(&EvGroupLock);
	read_unlock(&EvUsersLock);

	/* Find the event group information. */
	if ((EGroup = EvGetGroupBase(groupID)) == NULL) {
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_GROUP_EXIST;
	}

	MemberID = EGroup->EgiMemberID;
	HashBase = EvGetHashBase(EGroup, classID);

	/* Find the top level entry for this classID of events. */
	if ((EventBase = EvFindEventBase(classID, HashBase, &ClassBase))==NULL){
		read_unlock_irqrestore(&EvGroupLock, Flags);
		return -EV_ERROR_CLASS_EXISTS;
	}

	/* if the groupID indicates this is nat a local node then send
	 * off remote event information.
	 */
	if (EGroup->EgiType != EG_LOCAL) {
		if (EGroup->EgiGroupDest.EdID != 0) {
			if (EGroup->EgiGroupDest.EdCB) {
				EGroup->EgiGroupDest.EdCB(senderID, pri,
						EGroup->EgiID,
						EGroup->EgiMemberID, classID,
						eventID,
						info0, info1, info2, info3,
						dataLen, data);
			} else {
				if ((RetVal = EvGenEvent(senderID, pri,
					   EGroup->EgiID, EGroup->EgiMemberID,
					   accessCode, classID, eventID,
					   info0, info1, info2, info3,
					   &EGroup->EgiGroupDest, dataLen, data))) {
					read_unlock_irqrestore(&EvGroupLock, Flags);
					return RetVal;
				}
			}
		}
	}

	spin_lock(&ClassBase->EciLock);
	read_unlock(&EvGroupLock);

	RetVal = _EvSendEvent(EventBase, senderID, pri, groupID, MemberID, classID,
		     eventID, info0, info1, info2, info3, dataLen, data);

	spin_unlock_irqrestore(&ClassBase->EciLock, Flags);

	return RetVal;
}

/*
 * Helper function for other code.
 */
EvDestBase_t *
EvFindEventBase(EvClassID_t classID, EvClassInfo_t *hashBase,
		EvClassInfo_t **newBase)
{
	/* Find the top level entry for this classID of events. */
	while(hashBase->EciNext != NULL) {
		if (hashBase->EciClass == classID) {
			break;
		}
		hashBase = hashBase->EciNext;
	}

	while(hashBase->EciNext == NULL) {
		return NULL;
	}

	if (newBase != NULL)
		*newBase = hashBase;

	return hashBase->EciEventQ;
}

/*
 * Helper function for EvUnSubscribeAllEvents().
 */
void
EvUnSubscribeAllEventsByClass(EvUserID_t id, EvDestBase_t *eventBase, int *useCount)
{
	EvDest_t *EDest = NULL;
	EvDest_t *FreeDest = NULL;
	EvDestBase_t *SaveBase = eventBase;
	EvDestBase_t *LastEventBase = NULL;
	
	/* Search all event types in this classID. */
	while (eventBase->EdbNext != NULL) {
		EDest = &eventBase->EdbDestQ;

		/* Search the chain of destinations for this user. */
		while (EDest->EdNext != NULL) {
			if (EDest->EdNext->EdID == id) {
				FreeDest = EDest->EdNext;
				EDest->EdNext = EDest->EdNext->EdNext;
				eventBase->EdbUseCount--;
				kfree(FreeDest);
			} else {
				EDest = EDest->EdNext;
			}
		}

		/* Reset the top pointer */
		EDest = &eventBase->EdbDestQ;

		/* Top element treated special. */
		if (EDest->EdID == id) {
			if (EDest->EdNext != NULL) {
				EDest->EdUinfo = EDest->EdNext->EdUinfo;
				EDest->EdCB = EDest->EdNext->EdCB;
				EDest->EdID = EDest->EdNext->EdID;
				EDest->EdKinfo = EDest->EdNext->EdKinfo;
				FreeDest = EDest->EdNext;
				EDest->EdNext = EDest->EdNext->EdNext;
			}

			eventBase->EdbUseCount--;
			kfree(FreeDest);
		}

		if (eventBase->EdbUseCount == 0) {
			if (eventBase == SaveBase) {
				/* Free the top element */
				eventBase->EdbEvent = eventBase->EdbNext->EdbEvent;
				eventBase->EdbUseCount = eventBase->EdbNext->EdbUseCount;
				eventBase->EdbDestQ = eventBase->EdbNext->EdbDestQ;
				LastEventBase = eventBase->EdbNext;
				eventBase->EdbNext = LastEventBase->EdbNext;
				kfree(LastEventBase);
			} else {
				LastEventBase->EdbNext = eventBase->EdbNext;
				kfree(eventBase);
				eventBase = LastEventBase->EdbNext;
			}

			(*useCount)--;
		} else {
			/* Go to the next event type. */
			LastEventBase = eventBase;
			eventBase = eventBase->EdbNext;
		}
	}

	return;
}

/*
 * This is a helper function for EvUnRegisterUser().  It traverses the
 * undelivered event tree and removes all those ment for this user.
 */
void
EvUnSubscribeAllEvents(EvUserID_t userID)
{
	EvGroupInfo_t *EGroup;
	EvClassInfo_t *HashBase;
	int Hash;

	for (EGroup = EvGroupHead; EGroup != NULL; EGroup = EGroup->EgiNext) {
		if (EGroup->EgiGroupDest.EdID == userID) {
			EGroup->EgiGroupDest.EdID = 0;
			EGroup->EgiGroupDest.EdUinfo = NULL;
			EGroup->EgiGroupDest.EdCB = NULL;
			EGroup->EgiGroupDest.EdKinfo = NULL;

			EGroup->EgiUseCount--;
		}

		for (Hash = 0; Hash < EGroup->EgiHashSize; Hash++) {
			HashBase = &EGroup->EgiClassHash[Hash];

			while(HashBase->EciNext != NULL) {
				spin_lock(&HashBase->EciLock);
				EvUnSubscribeAllEventsByClass(userID,
						HashBase->EciEventQ,
						&HashBase->EciUseCount);
				spin_unlock(&HashBase->EciLock);
				HashBase = HashBase->EciNext;
			}
		}
	}

	return;
}

EXPORT_SYMBOL(EvFree);
EXPORT_SYMBOL(EvSendEvent);
