/*
 * event_core.h
 *
 * Data structures and values needed internaly for the eventing mechanism.
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
#ifndef EVENT_CORE_H
#define EVENT_CORE_H

/* Currently know types of event accesses. */
#define ED_USER_ASYNC   1
#define ED_USER_POLL    2

#define EV_ACCESS_CHECK	 1
#define EV_ACCESS_IGNORE 0

typedef struct ev_kinfo_t {
	EvUserID_t        EkiID;
	char              EkiName[16];
        wait_queue_head_t EkiWaitQ;
        EvPacket_t	  *EkiHead;
        EvPacket_t	  *EkiTail;
	spinlock_t	  EkiLock;
	struct ev_kinfo_t *EkiNext;
	unsigned int	  EkiQCount;
	unsigned int	  EkiQLimit;
} EvKernelInfo_t;

typedef struct ev_user_info {
	EvType_t	     EuiType;
	EvUserID_t	     EuiID;
	pid_t		     EuiPid;
        wait_queue_head_t    EuiWaitQ;
        struct fasync_struct *EuiFasyncPtr;
        EvPacket_t	     *EuiHead;
        EvPacket_t	     *EuiTail;
	spinlock_t	     EuiLock;
	unsigned int	     EuiQCount;
	unsigned int	     EuiQLimit;
} EvUserInfo_t;

typedef struct ev_dest_t {
        EvPri_t	       EdPri;
	EvUserID_t     EdID;
        int	       (*EdCB)(EvUserID_t, int, EvGroupID_t,
			       EvGroupID_t, EvClassID_t, EvEventID_t,
			       int, int, int, int, int, void *);
        EvUserInfo_t   *EdUinfo;
        EvKernelInfo_t *EdKinfo;
        struct ev_dest_t *EdNext;
} EvDest_t;

typedef struct ev_dest_base_t {
        EvEventID_t EdbEvent;
        int         EdbUseCount;
        EvDest_t    EdbDestQ;
        struct ev_dest_base_t *EdbNext;
} EvDestBase_t;

typedef struct ev_class_info {
	EvClassID_t  EciClass;
	char         EciName[16];
	int	     EciUseCount;
	EvAccess_t   EciAccessCode;
	EvDestBase_t *EciEventQ;
	spinlock_t   EciLock;
	struct ev_class_info *EciNext;
} EvClassInfo_t;

typedef struct ev_sl_list {
	char	    EslName[16];
	EvGroupID_t EslID;
	struct ev_sl_list *EslNext;
} EvMemberList_t;

typedef struct ev_pend_rem {
	void		   *PrGroup;
	int		   PrRequest;
	int		   PrInfo[4];
	int		   PrUseCount;
	int		   PrDataLen;
	char *		   PrData;
        wait_queue_head_t  PrWaitQ;
	EvPacket_t	   PrRetInfo;
	struct ev_pend_rem *PrNext;
} EvPendRem_t;

typedef struct ev_group_info_t {
	char           EgiName[16];      /* Name of Locality Server */
	char           EgiMemberName[16];/* Name of element using Server */
	EvGroupID_t    EgiID;		 /* Local ID of group */
	EvGroupID_t    EgiMasterID;	 /* ID of group master */
	EvGroupID_t    EgiMemberID;	 /* ID of element using Server */
	EvType_t       EgiType;		 /* MASTER or MEMBER */
	EvGroupID_t    EgiNextMemberID;  /* Next member ID to assign */
	unsigned short EgiUseCount;
	short          EgiHashSize;      /* Size of the hash list */
	EvClassInfo_t  *EgiClassHash;    /* Class hash table for this Loc */
	rwlock_t       EgiLock;		 /* Class processing lock */
	EvClassID_t    EgiNextClass;     /* Next class ID to assign */
	EvDest_t       EgiGroupDest;	 /* Group remote event queue. */
	EvMemberList_t *EgiMembers;	 /* If this is MASTER */
	EvPendRem_t    *EgiPendRemoteList;/* List of pending remote reqests */
	EvAccess_t     EgiAccessCode;	 /* Permissons on the group */
	struct ev_group_info_t *EgiNext;
} EvGroupInfo_t;

/* Kernel uses for the Flag in KernelEventPacket. */
#define EF_EMERGENCY_PACKET     0x1

/* Number of hash entries for the local event group. */
#define EV_NUM_LOCAL_HASHES 13

/* Group locking modes. */
#define EV_LOCK_GROUP_WRITE   1
#define EV_LOCK_GROUP_READ    0

extern EvGroupInfo_t *EvGroupHead;
extern EvKernelInfo_t *KernelClassHead;
extern EvKernelInfo_t *EvUsersHead;

extern rwlock_t EvUsersLock;
extern rwlock_t EvGroupLock;

extern unsigned int EvSysQLimit;

EvDestBase_t * EvFindEventBase(EvClassID_t, EvClassInfo_t *, EvClassInfo_t **);
EvKernelInfo_t *EvCheckUser(EvUserID_t);
void EvUnSubscribeAllEvents(EvUserID_t);

EvGroupInfo_t *EvGetGroupBase(EvGroupID_t);
int  EvLockAndGetGroup(EvGroupID_t, int, EvAccess_t,
		       unsigned long *, int, EvGroupInfo_t **);
EvClassInfo_t * EvGetHashBase(EvGroupInfo_t *eGroup, EvClassID_t);

int  EvSendGetMemberID(EvGroupInfo_t *, char *, EvGroupID_t *, unsigned long);
int  EvSendGetMemberName(EvGroupInfo_t *, EvGroupID_t, char *, unsigned long);

int  EvSendRegEventClass(EvGroupInfo_t *, char *, EvAccess_t,
			 EvAccess_t, EvClassID_t *, unsigned long *);
int  EvSendRegClassToMembers(EvGroupInfo_t *, char *, EvAccess_t,
			     EvAccess_t, EvClassID_t);

int  EvSendUnRegEventClass(EvGroupInfo_t *, EvAccess_t,
			   EvClassID_t, unsigned long *);
int  EvSendUnRegClassToMembers(EvGroupInfo_t *, EvClassID_t, EvAccess_t);

int  EvSendGetUserID(EvGroupID_t, EvGroupID_t, char *, EvUserID_t *);
int  EvSendGetUserName(EvGroupInfo_t *, EvGroupID_t, EvUserID_t, char *, unsigned long *);

int  EvUserSubscribeEvent(EvUserInfo_t *, EvReq_t *);
int  EvUserUnSubscribeEvent(EvUserInfo_t *, EvReq_t *);
int  EvUserSubscribeGroupEvents(EvUserInfo_t *, EvReq_t *);
int  EvUserSendEvent(EvUserInfo_t *, EvReq_t *);

int  EvCheckAccessCode(EvAccess_t, EvAccess_t);

int  EvGenEvent(EvUserID_t, int, EvGroupID_t, EvGroupID_t,
		EvAccess_t, EvClassID_t, EvEventID_t,
		int, int, int, int, EvDest_t *, int, void *);

EvPendRem_t *EvGetPendEntry(EvGroupInfo_t *,
			    int, int, int, int, int, int, char *);
EvPendRem_t *EvFindPendEntry(EvGroupInfo_t *, 
			     int, int, int, int, int, int, char *);
void EvDelPendEntry(EvGroupInfo_t *, EvPendRem_t *);

int EvFillInHashInfo(EvClassInfo_t *, EvAccess_t, char *, unsigned short *);
int EvInternalUnRegisterEventClass(EvGroupInfo_t *, EvClassID_t, EvAccess_t);
EvAccess_t EvGetClassAccess(EvGroupInfo_t *, EvClassID_t);

int  _EvSendEvent(EvDestBase_t *, EvUserID_t, int, EvGroupID_t, EvGroupID_t,
		  EvClassID_t, EvEventID_t, int, int, int, int, int, void *);

/* Debug routines. */
void DumpEventReg(void);
void DumpClassList(void);

#endif /* EVENT_CORE_H */
