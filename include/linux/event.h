/*
 * event.h
 *
 * Kernel event access routines and data structures.
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * Copyright 2001 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef EVENT_H
#define EVENT_H

/* User request uses for the Flag in EvPacket_t */
#define EF_WAIT_FOR_EVENT	0x1

/* Flag values to indicate the status of the extra data field. */
#define EF_DATA_AVAIL		0x2
#define EF_DATA_FAILURE		0x4

/* Event class zero indicates send all events for that class. */
#define EC_ALL			0

/* All local events default to locality zero */
#define EG_LOCAL		0

/* Indication that not access code has been set. */
#define EAC_NONE		0

/* Types of non local event group types */
#define EG_MASTER		1
#define EG_MEMBER		2

/* Control class for groups is always zero */
#define EV_CONTROL_CLASS_ID 0

/* Events to be processed by control class. */
#define EV_REM_GET_MEMBER_ID		1
#define EV_REM_GET_MEMBER_NAME		2

#define EV_REM_REGISTER_EVENT_CLASS	3
#define EV_MASTER_REG_EVENT_CLASS	4

#define EV_REM_UNREGISTER_EVENT_CLASS	5
#define EV_MASTER_UNREG_EVENT_CLASS	6

#define EV_REM_GET_USER_ID		7
#define EV_REM_GET_USER_NAME		8


/* Error codes known by the event manager. */
#define EV_NOERR		  0
#define EV_ERROR_MIN		  400

#define EV_ERROR_GROUP_EXIST	  401
#define EV_ERROR_GROUP_ACCESS	  402
#define EV_ERROR_GROUP_BUSY	  403
#define EV_ERROR_GROUP_TYPE	  404

#define EV_ERROR_MEMBER_EXIST	  405

#define EV_ERROR_NO_REMOTE	  406
#define EV_ERROR_NO_REQUESTOR	  407
#define EV_ERROR_TIMEOUT	  408

#define EV_ERROR_CLASS_EXISTS	  409
#define EV_ERROR_CLASS_BUSY	  410
#define EV_ERROR_CLASS_ACCESS	  411
#define EV_ERROR_CLASS_NO_SUB	  412

#define EV_ERROR_USER_EXISTS	  413

#define EV_ERROR_NO_EVENT	  414

#define EV_ERROR_INVALID_COPY_IN  415
#define EV_ERROR_INVALID_COPY_OUT 416

#define EV_ERROR_MEM_ALLOC	  417

#define EV_ERROR_WAIT_INTERRUPTED 418
#define EV_ERROR_INVALID_REQEST   419

#define EV_ERROR_QUEUE_FULL       420
#define EV_ERROR_NOT_SU           421

#define EV_ERROR_MAX		  EV_ERROR_NOT_SU

typedef unsigned int	EvUserID_t;
typedef unsigned short	EvGroupID_t;
typedef unsigned short	EvClassID_t;
typedef unsigned short	EvEventID_t;
typedef short		EvType_t;
typedef int		EvPri_t;
typedef unsigned int	EvAccess_t;

/* Structure of an event packet. */
typedef struct ev_packet {
	int         EpFlag;	/* Used to indicate sleep wait or not. */
	EvUserID_t  EpSenderID;
	EvPri_t     EpPri;
	EvGroupID_t EpGroupID;
	EvGroupID_t EpMemberID;
	EvClassID_t EpClassID;
	EvEventID_t EpEventID;
	EvAccess_t  EpAccessCode;
	struct timeval EpTimeStamp;
	int         EpInfo[4];
	int         EpDataLen;
	void        *EpData;
	struct ev_packet *EpNext;
} EvPacket_t;

/* Structure used to request and event to be sent to a process or not. */
typedef struct ev_req {
	EvEventID_t ErUserID;
	EvPri_t     ErPri;
	EvGroupID_t ErGroupID;
	EvClassID_t ErClassID;
	EvEventID_t ErEventID;
	EvAccess_t  ErAccessCode;
} EvReq_t;

/* Data structure used by EV_REG_ID, EV_UNREG_ID and EV_GET_ID. */
typedef struct ev_class {
	EvGroupID_t EcGroupID;
	EvClassID_t EcClassID;
	EvAccess_t  EcGroupAccessCode;
	EvAccess_t  EcClassAccessCode;
	char        EcName[16];
} EvClass_t;

/* Data structure used by EV_REG_ID, EV_UNREG_ID and EV_GET_ID. */
typedef struct ev_group {
	char        EgGroupName[16];
	char        EgMemberName[16];
	EvGroupID_t EgMasterID;
	EvType_t    EgType;
	short       EgHashSize;
	EvAccess_t  EgGroupAccessCode;
	EvAccess_t  EgControlAccessCode;
	EvGroupID_t EgGroupID;
	EvGroupID_t EgMemberID;
} EvGroup_t;

/* Data structure used by EV_REG_ID, EV_UNREG_ID and EV_GET_ID. */
typedef struct ev_user {
	EvGroupID_t EuGroupID;
	EvGroupID_t EuMemberID;
	EvUserID_t  EuID;
	char        EuName[16];
} EvUser_t;

typedef struct ev_access {
	EvGroupID_t EaGroupID;
	EvClassID_t EaClassID;
	EvAccess_t  EaOldAccessCode;
	EvAccess_t  EaNewAccessCode;
} EvAccessCode_t;

/* User level ioctl cmd identifiers. */
#define EV_MAGIC 0xd0

#define EV_REG_USER		_IOWR(EV_MAGIC, 1, EvUser_t)
#define EV_UNREG_USER		_IO(EV_MAGIC, 2)

#define EV_GET_USER_ID		_IOWR(EV_MAGIC, 3, EvUser_t)
#define EV_COMP_USER_ID		_IOWR(EV_MAGIC, 4, EvUser_t)
#define EV_GET_USER_NAME	_IOWR(EV_MAGIC, 5, EvUser_t)
#define EV_COMP_USER_NAME	_IOWR(EV_MAGIC, 6, EvUser_t)

#define EV_CREATE_GROUP		_IOWR(EV_MAGIC, 7, EvGroup_t)
#define EV_DELETE_GROUP		_IOW(EV_MAGIC, 8, EvGroup_t)
#define EV_ATTACH_GROUP		_IOWR(EV_MAGIC, 9, EvGroup_t)
#define EV_DETACH_GROUP		_IOW(EV_MAGIC, 10, EvGroup_t)

#define EV_GET_GROUP_ID		_IOWR(EV_MAGIC, 11, EvGroup_t)
#define EV_GET_GROUP_NAME	_IOWR(EV_MAGIC, 12, EvGroup_t)
#define EV_GET_MEMBER_ID	_IOWR(EV_MAGIC, 13, EvGroup_t)
#define EV_COMP_MEMBER_ID	_IOW(EV_MAGIC, 14, EvPacket_t)
#define EV_GET_MEMBER_NAME	_IOWR(EV_MAGIC, 15, EvGroup_t)
#define EV_COMP_MEMBER_NAME	_IOW(EV_MAGIC, 16, EvPacket_t)

#define EV_REG_EVENT_CLASS	_IOWR(EV_MAGIC, 17, EvClass_t)
#define EV_REM_REG_EVENT_CLASS	_IOWR(EV_MAGIC, 18, EvPacket_t)

#define EV_UNREG_EVENT_CLASS	_IOW(EV_MAGIC, 19, EvClass_t)
#define EV_REM_UNREG_EVENT_CLASS _IOWR(EV_MAGIC, 20, EvPacket_t)

#define EV_GET_EVENT_CLASS	_IOWR(EV_MAGIC, 21, EvClass_t)
#define EV_GET_EVENT_CLASS_NAME	_IOWR(EV_MAGIC, 22, EvClass_t)

#define EV_SUBSCRIBE_EVENT	_IOW(EV_MAGIC, 23, EvReq_t)
#define EV_UNSUBSCRIBE_EVENT	_IOW(EV_MAGIC, 24, EvReq_t)

#define EV_SUBSCRIBE_GROUP_EV	_IOW(EV_MAGIC, 25, EvReq_t)
#define EV_UNSUBSCRIBE_GROUP_EV	_IOW(EV_MAGIC, 26, EvReq_t)

#define EV_GET_EVENT		_IOWR(EV_MAGIC, 27, EvPacket_t)
#define EV_SEND_EVENT		_IOW(EV_MAGIC, 28, EvPacket_t)
#define EV_REM_SEND_EVENT	_IOW(EV_MAGIC, 29, EvPacket_t)

#define EV_SYNC_TO_MEMBER	_IOW(EV_MAGIC, 30, EvReq_t)

#define EV_SET_GROUP_ACCESSCODE	_IOW(EV_MAGIC, 31, EvAccessCode_t)
#define EV_SET_CLASS_ACCESSCODE	_IOW(EV_MAGIC, 32, EvAccessCode_t)

#define EV_SET_Q_LIMIT		_IOW(EV_MAGIC, 33, unsigned int)
#define EV_SET_SYS_Q_LIMIT	_IOW(EV_MAGIC, 34, unsigned int)

#ifdef __KERNEL__
#ifdef CONFIG_EVENT_BROKER
/* Kernel interface access function prototypes. */
int  EvRegisterUser(char *, EvUserID_t *);
int  EvUnRegisterUser(EvUserID_t);

int  EvGetUserID(EvGroupID_t, EvGroupID_t, char *, EvUserID_t *);
int  EvGetLocalUserID(char *, EvUserID_t *);
int  EvCompGetUserID(EvGroupID_t, EvGroupID_t, char *, int, EvUserID_t);

int  EvGetUserName(EvGroupID_t, EvGroupID_t, EvUserID_t, char *);
int  EvGetLocalUserName(EvUserID_t, char *);
int  EvCompGetUserName(EvGroupID_t, EvGroupID_t, EvUserID_t, int, char *);

int  EvCreateGroup(char *, char *, EvType_t, EvAccess_t, EvAccess_t, short,
		   EvGroupID_t, EvGroupID_t, EvGroupID_t *);

int  EvTakeOverMaster(EvGroupID_t, EvGroupID_t, EvAccess_t);

int  EvDeleteGroup(EvGroupID_t, EvAccess_t);
int  EvAttachGroup(EvGroupID_t, char *, EvAccess_t, EvGroupID_t *);
int  EvDetachGroup(EvGroupID_t, EvGroupID_t, EvAccess_t);

int  EvGetGroupID(char *, EvGroupID_t *);
int  EvGetGroupName(EvGroupID_t, char *);

int  EvGetMemberID(EvGroupID_t, char *, EvGroupID_t *);
int  EvCompGetMemberID(EvGroupID_t, char *, int, EvGroupID_t);
int  EvGetMemberName(EvGroupID_t, EvGroupID_t, char *);
int  EvCompGetMemberName(EvGroupID_t, EvGroupID_t, int, char *);

int  EvSubscribeGroupEvents(EvUserID_t, EvGroupID_t, EvAccess_t,
		      	    int (*)(EvUserID_t, int, EvGroupID_t,
			            EvGroupID_t, EvClassID_t, EvEventID_t,
			            int, int, int, int, int, void *));
int  EvUnSubscribeGroupEvents(EvUserID_t, EvGroupID_t);

int  EvRegisterEventClass(EvGroupID_t, char *, EvAccess_t,
			  EvAccess_t, EvClassID_t *);
int  EvRegisterLocalEventClass(char *, EvAccess_t, EvClassID_t *); 
int  EvRemoteRegisterEventClass(EvGroupID_t, char *, EvAccess_t,
				EvAccess_t, int, EvClassID_t);

int  EvUnRegisterEventClass(EvGroupID_t, EvClassID_t, EvAccess_t);
int  EvUnRegisterLocalEventClass(EvClassID_t, EvAccess_t);
int  EvRemoteUnRegisterEventClass(EvGroupID_t, EvClassID_t, EvAccess_t, int);

int  EvGetEventClassID(EvGroupID_t, char *, EvClassID_t *);
int  EvGetLocalEventClassID(char *, EvClassID_t *);
int  EvGetEventClassName(EvGroupID_t, EvClassID_t, char *);
int  EvGetLocalEventClassName(EvClassID_t, char *);

int  EvSubscribeEvent(EvUserID_t, EvPri_t, EvGroupID_t, EvClassID_t,
		      EvEventID_t, EvAccess_t,
		      int (*)(EvUserID_t, EvPri_t, EvGroupID_t,
			      EvGroupID_t, EvClassID_t, EvEventID_t,
			      int, int, int, int, int, void *));
int  EvLocalSubscribeEvent(EvUserID_t, EvPri_t, EvClassID_t,
			   EvEventID_t, EvAccess_t,
			   int (*)(EvUserID_t, EvPri_t, EvGroupID_t,
				   EvGroupID_t, EvClassID_t, EvEventID_t,
				   int, int, int, int, int, void *));
int  EvUnSubscribeEvent(EvUserID_t, EvGroupID_t, EvClassID_t, EvEventID_t);
int  EvLocalUnSubscribeEvent(EvUserID_t, EvClassID_t, EvEventID_t);

int  EvGetEvent(EvUserID_t, int, EvPacket_t **);
void EvFree(EvPacket_t *, int);

int  EvSendEvent(EvUserID_t, EvPri_t, EvGroupID_t, EvClassID_t, EvEventID_t,
		 EvAccess_t, int, int, int, int, int, void *);
int  EvRemoteSendEvent(EvUserID_t, EvGroupID_t, EvPri_t, EvGroupID_t,
		       EvClassID_t, EvEventID_t, EvAccess_t,
		       int, int, int, int, int, void *);

int  EvSyncToMember(EvGroupID_t, EvAccess_t, EvGroupID_t);

int  EvSetGroupAccessCode(EvGroupID_t, EvAccess_t, EvAccess_t);
int  EvSetClassAccessCode(EvGroupID_t, EvClassID_t, EvAccess_t, EvAccess_t);
int  EvSetLocalClassAccessCode(EvClassID_t, EvAccess_t, EvAccess_t);

/* for this that do not like my function naming methods */
#define ev_reg_user		     EvRegisterUser
#define ev_unreg_user		     EvUnRegisterUser
#define ev_get_user_id		     EvGetUserID
#define ev_loc_get_user_id	     EvGetLocalUserID
#define ev_comp_get_user_id	     EvCompGetUserID
#define ev_get_user_name	     EvGetUserName
#define ev_loc_get_user_name	     EvGetLocalUserName
#define ev_comp_get_user_name	     EvCompGetUserName
#define ev_create_group		     EvCreateGroup
#define ev_takeover_master	     EvTakeOverMaster
#define ev_delete_group		     EvDeleteGroup
#define ev_attach_group		     EvAttachGroup
#define ev_detach_group		     EvDetachGroup
#define ev_get_group_id		     EvGetGroupID
#define ev_get_group_name	     EvGetGroupName
#define ev_get_mem_id		     EvGetMemberID
#define ev_comp_get_mem_id	     EvCompGetMemberID
#define ev_get_mem_name		     EvGetMemberName
#define ev_comp_get_mem_name	     EvCompGetMemberName
#define ev_sub_group_events	     EvSubscribeGroupEvents
#define ev_unsub_group_events	     EvUnSubscribeGroupEvents
#define ev_reg_class		     EvRegisterEventClass
#define ev_loc_reg_class	     EvRegisterLocalEventClass
#define ev_rem_reg_class	     EvRemoteRegisterEventClass
#define ev_unreg_class		     EvUnRegisterEventClass
#define ev_loc_unreg_class	     EvUnRegisterLocalEventClass
#define ev_rem_unreg_class	     EvRemoteUnRegisterEventClass
#define ev_get_class_id		     EvGetEventClassID
#define ev_get_loc_class_id	     EvGetLocalEventClassID
#define ev_get_class_name	     EvGetEventClassName
#define ev_get_loc_class_name	     EvGetLocalEventClassName
#define ev_sub_event		     EvSubscribeEvent
#define ev_loc_sub_event	     EvLocalSubscribeEvent
#define ev_unsub_event		     EvUnSubscribeEvent
#define ev_loc_unsub_event	     EvLocalUnSubscribeEvent
#define ev_get_event		     EvGetEvent
#define ev_free			     EvFree
#define ev_send_event		     EvSendEvent
#define ev_remote_send_event	     EvRemoteSendEvent
#define ev_sync_to_member	     EvSyncToMember
#define ev_set_group_access_code     EvSetGroupAccessCode
#define ev_set_class_access_code     EvSetClassAccessCode
#define ev_set_loc_class_access_code EvSetLocalClassAccessCode

#else /* CONFIG_EVENT_BROKER */

#define EvRegisterUser(x,y)
#define EvUnRegisterUser(x)

#define EvGetUserID(w,x,y,z)
#define EvGetLocalUserID(x,y)
#define EvCompGetUserID(v,w,x,y,z)

#define EvGetUserName(w,x,y,z)
#define EvGetLocalUserName(x,y)
#define EvCompGetUserName(v,w,x,y,z)

#define EvCreateGroup(r,s,t,u,v,w,x,y,z)

#define EvTakeOverMaster(x,y,z)

#define EvDeleteGroup(x,y)
#define EvAttachGroup(w,x,y,z)
#define EvDetachGroup(x,y,z)

#define EvGetGroupID(x,y)
#define EvGetGroupName(x,y)

#define EvGetMemberID(x,y,z)
#define EvCompGetMemberID(w,x,y,z)
#define EvGetMemberName(x,y,z)
#define EvCompGetMemberName(w,x,y,z)

#define EvSubscribeGroupEvents(w,x,y,z)
#define EvUnSubscribeGroupEvents(x,y)

#define EvRegisterEventClass(v,w,x,y,z)
#define EvRegisterLocalEventClass(x,y,z)
#define EvRemoteRegisterEventClass(u,v,w,x,y,z)

#define EvUnRegisterEventClass(x,y,z)
#define EvUnRegisterLocalEventClass(x,y)
#define EvRemoteUnRegisterEventClass(w,x,y,z)

#define EvGetEventClassID(x,y,z)
#define EvGetLocalEventClassID(x,y)
#define EvGetEventClassName(x,y)
#define EvGetLocalEventClassName(x)

#define EvSubscribeEvent(t,u,v,w,x,y,z)
#define EvLocalSubscribeEvent(u,v,w,x,y,z)
#define EvUnSubscribeEvent(w,x,y,z)
#define EvLocalUnSubscribeEvent(x,y,z)

#define EvGetEvent(x,y,z)
#define EvFree(x,y)

#define EvSendEvent(o,p,q,r,s,t,u,v,w,x,y,z)
#define EvRemoteSendEvent(n,o,p,q,r,s,t,u,v,w,x,y,z)

#define EvSyncToMember(x,y,z)
#define EvSetGroupAccessCode(x,y,z)
#define EvSetClassAccessCode(w,x,y,z)
#define EvSetLocalClassAccessCode(x,y,z)

/* for this that do not like my function naming methods */
#define ev_reg_user(x,y)
#define ev_unreg_user(x)
#define ev_get_user_id(v,w,x,y,z)
#define ev_loc_get_user_id(x,y)
#define ev_comp_get_user_id(w,x,y,z)
#define ev_get_user_name(v,w,x,y.z)
#define ev_loc_get_user_name(x,y)
#define ev_comp_get_user_name(w,x,y,z)
#define ev_create_group(r,s,t,u,v,w,x,y,z)
#define ev_takeover_master(x,y,z)
#define ev_delete_group(x,y)
#define ev_attach_group(w,x,y,z)
#define ev_detach_group(x,y,z)
#define ev_get_group_id(x,y)
#define ev_get_group_name(x,y)
#define ev_get_mem_id(x,y,z)
#define ev_comp_get_mem_id(w,x,y,z)
#define ev_get_mem_name(x,y,z)
#define ev_comp_get_mem_name(w,x,y,z)
#define ev_sub_group_events(w,x,y,z)
#define ev_unsub_group_events(x,y)
#define ev_reg_class(x,y,z)
#define ev_loc_reg_class(x,y,z)
#define ev_rem_reg_class(u,v,w,x,y,z)
#define ev_unreg_class(x,y,z)
#define ev_loc_unreg_class(x,y)
#define ev_rem_unreg_class(w,x,y,z)
#define ev_get_class_id(x,y,z)
#define ev_get_loc_class_id(x,y)
#define ev_get_class_name(x,y)
#define ev_get_loc_class_name(x)
#define ev_sub_event(t,u,v,w,x,y,z)
#define ev_loc_sub_event(u,v,w,x,y,z)
#define ev_unsub_event(w,x,y,z)
#define ev_loc_unsub_event(x,y,z)
#define ev_get_event(x,y,z)
#define ev_free(x,y)
#define ev_send_event(o,p,q,r,s,t,u,v,w,x,y,z)
#define ev_remote_send_event(n,o,p,q,r,s,t,u,v,w,x,y,z)
#define ev_sync_to_member(x,y,z)
#define ev_set_group_access_code(x,y,z)
#define ev_set_class_access_code(w,x,y,z)
#define ev_set_loc_class_access_code(x,y,z)

#endif /* CONFIG_EVENT_BROKER */
#endif /* __KERNEL__ */
#endif /* EVENT_H */
