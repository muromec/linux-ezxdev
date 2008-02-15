/*
 *  NSA Security-Enhanced Linux (SELinux) security module
 *
 *  This file contains the hook function processing for the extended
 *  socket calls.  This processing is optional and can be omitted by
 *  disabling the Extended Socket Calls option.  Since the current 
 *  implementation has several unresolved issues, it is recommended that 
 *  you disable this option for normal use of SELinux.  Disabling this option 
 *  has no impact on the enforcement of the network policy by the kernel, and 
 *  no applications have yet been modified to use these calls.
 *
 *  If you want to experiment with the extended socket calls on INET
 *  sockets, then you should also enable the Labeled IP Networking Support
 *  and CIPSO/FIPS-188 IP Options options.
 *
 *  Authors:  Stephen Smalley, <sds@epoch.ncsc.mil>
 *            Chris Vance, <cvance@nai.com>
 *            Wayne Salamon, <wsalamon@nai.com>
 *
 *  Copyright (C) 2002 Networks Associates Technology, Inc.
 * 
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 */ 
#ifndef _EXTSOCKET_H
#define _EXTSOCKET_H

#ifdef CONFIG_SECURITY_SELINUX_EXTSOCKET

static LIST_HEAD(open_request_security_head);

static spinlock_t open_request_alloc_lock = SPIN_LOCK_UNLOCKED;

static inline int extsocket_open_request_alloc_security(struct open_request *req)
{
	struct open_request_security_struct *orsec, *new_orsec;
	unsigned long flags;

	new_orsec = kmalloc(sizeof(struct open_request_security_struct), SAFE_ALLOC);
	if (!new_orsec)
		return -ENOMEM;

	spin_lock_irqsave(&open_request_alloc_lock, flags);
	orsec = req->security;
	if (orsec && orsec->magic == SELINUX_MAGIC) {
		spin_unlock_irqrestore(&open_request_alloc_lock, flags);
		kfree(new_orsec);
		return 0;
	}
	orsec = new_orsec;

	memset(orsec, 0, sizeof(struct open_request_security_struct));
	orsec->magic = SELINUX_MAGIC;
	orsec->req = req;
	list_add(&orsec->list, &open_request_security_head);
	orsec->newconn_sid = SECINITSID_UNLABELED;
	req->security = orsec;

	spin_unlock_irqrestore(&open_request_alloc_lock, flags);
	return 0;
}

static inline void extsocket_open_request_free_security(struct open_request *req)
{
	struct open_request_security_struct *orsec;
	unsigned long flags;

	orsec = req->security;
	if (!orsec || orsec->magic != SELINUX_MAGIC)
		return;

	req->security = NULL;
	spin_lock_irqsave(&open_request_alloc_lock, flags);
	list_del(&orsec->list);
	spin_unlock_irqrestore(&open_request_alloc_lock, flags);
	kfree(orsec);
}

static inline void extsocket_init(struct inode_security_struct *isec)
{
	isec->peer_sid = SECINITSID_ANY_SOCKET;
}

static inline security_id_t extsocket_create(struct task_security_struct *tsec)
{
	if (tsec->in_sid[0])
		return tsec->in_sid[0];
	else
		return tsec->sid;
}

static inline int extsocket_connect(struct socket *sock,
				    struct sockaddr *address,
				    int addrlen,
				    struct task_security_struct *tsec,
				    struct inode_security_struct *isec,
				    struct avc_audit_data *adp)
{
	struct sock *sk = sock->sk;
	security_id_t node_sid;
	security_id_t dso_sid;
	__u32 daddr;
	int err;

	dso_sid = tsec->in_sid[0];
	if (dso_sid) {
		if (sk && sk->family == PF_INET) {
			/* 
			 * Verify that the destination is trusted to enforce
			 * the restriction on the peer socket.
			 */			
			if (addrlen != sizeof(struct sockaddr_in))
				return -EINVAL;
			daddr = ((struct sockaddr_in *)address)->sin_addr.s_addr;
			err = security_node_sid(PF_INET, &daddr, sizeof(daddr), &node_sid);
			if (err) 
				return err;
			err = avc_has_perm_audit(dso_sid, node_sid, SECCLASS_NODE, 
						 NODE__ENFORCE_DEST, adp);
			if (err) 
				return err;
		} 
		/* Save the peer SID so set_owner_w can send it out */
		if (dso_sid != SECINITSID_ANY_SOCKET)
			isec->dsid = dso_sid;
		isec->peer_sid = dso_sid;
	}

	return 0;
}

static inline int extsocket_listen(struct socket *sock,
	                           struct task_security_struct *tsec,
				   struct inode_security_struct *isec,
				   struct avc_audit_data *adp)
{
	security_id_t newconn_sid;
	int useclient;
	access_vector_t perm;
	int err;

	/*
	 * Use newconn_sid and useclient from the extended call.
	 * newconn_sid is in_sid[0], useclient is in_sid[1]
	 */
	newconn_sid = tsec->in_sid[0];
	useclient = tsec->in_sid[1];
	if (newconn_sid && useclient)
		return -EINVAL;

	switch (isec->sclass) {
	case SECCLASS_UNIX_STREAM_SOCKET:
		perm = UNIX_STREAM_SOCKET__NEWCONN;
		break;
	case SECCLASS_TCP_SOCKET:
		perm = TCP_SOCKET__NEWCONN;
		break;
	default:
		if (useclient || (newconn_sid && newconn_sid != isec->sid))
			return -EOPNOTSUPP;
		isec->useclient = 0;
		isec->newconn_sid = isec->sid;
		return 0;
	}

	/* newconn permission is implicitly granted if the SIDs are equal */
	if (newconn_sid && newconn_sid != isec->sid) {
		err = avc_has_perm_audit(isec->sid, newconn_sid,
				isec->sclass, perm, adp);
		if (err)
			return err;
	}
	isec->useclient = useclient;
	if (newconn_sid)
		isec->newconn_sid = newconn_sid;
	else
		isec->newconn_sid = isec->sid;

	return 0;
}

static inline void extsocket_accept(struct inode_security_struct *isec, 
				    struct inode_security_struct *newisec)
{
	switch (isec->sclass) {
	case SECCLASS_UNIX_STREAM_SOCKET:
		newisec->conn_perm = UNIX_STREAM_SOCKET__ACCEPTFROM;
		break;
	case SECCLASS_TCP_SOCKET:
		newisec->conn_perm = TCP_SOCKET__ACCEPTFROM;
		break;
	default:
		newisec->conn_perm = 0;
		break;
	}
}

static inline void extsocket_post_accept(struct socket *sock,
					 struct socket *newsock)
{
	struct task_security_struct *tsec;
	struct inode_security_struct *isec;
	struct inode_security_struct *newisec;
	struct sock_security_struct *sksec;
	struct sock *sk = newsock->sk;

	tsec = current->security;
	newisec = SOCK_INODE(newsock)->i_security;
	isec = SOCK_INODE(sock)->i_security;
	sksec = sk->security;

	/* Set the peer SID of the new socket to the SID set in the
	 * new sock structure. This SID was set when the sock object
	 * was created at final connection setup.
	 */
	newisec->peer_sid = sksec->peer_sid;

	/* If useclient was specified, then the new socket 
	   inherits the SID of the peer.  Otherwise, the 
	   SID originally assigned during accept is fine. */
	if (isec->useclient)
		newisec->sid = newisec->peer_sid;

	tsec->out_sid[0] = newisec->peer_sid;

	return;
}

static inline int extsocket_sendmsg(struct socket *sock,
				    struct msghdr *msg,
	                            struct task_security_struct *tsec,
				    struct inode_security_struct *isec,
				    struct avc_audit_data *adp)
{
	struct sock *sk = sock->sk;
	security_id_t node_sid;
	security_id_t dso_sid, msg_sid;
	__u32 daddr;
	int err;

	dso_sid = tsec->in_sid[0];
	msg_sid = tsec->in_sid[1];
	if (sk->type == SOCK_STREAM) {
		/* All messages sent on a stream socket have the same SID, 
		   inherited from the socket. */
		if (msg_sid && msg_sid != isec->sid) 
			return -EINVAL;
		/* The only possible destination is the connected peer. */
		if (dso_sid && dso_sid != isec->peer_sid) 
			return -EACCES;
		if (isec->sclass == SECCLASS_TCP_SOCKET && isec->conn_perm) {
			/* Revalidate the connection permission. */
			err = avc_has_perm_audit(isec->sid, isec->peer_sid,
						 isec->sclass, isec->conn_perm, 
						 adp);
			if (err)
				return err;
		}
	} else { /* non-stream sockets */
		if (sk->family == PF_INET) {
			/* UDP, raw IP, etc. */
			if (dso_sid) {
				/*
				 * Verify that the destination is trusted to enforce
				 * the restriction on the destination socket. The
				 * destination address is taken from the msg header 
				 * structure if present, else it is taken from the 
				 * socket if the socket is in the connected state.
				 */
				if (msg->msg_name) {
					if (msg->msg_namelen != sizeof(struct sockaddr_in))
						return -EINVAL;
					daddr = ((struct sockaddr_in *)msg->msg_name)->sin_addr.s_addr;
				} else {
					if (sk->state != TCP_ESTABLISHED) 
						return -ENOTCONN;
					daddr = sk->daddr;
				}
				
				err = security_node_sid(PF_INET, &daddr, 
							sizeof(daddr), &node_sid);
				if (err)
					return err;
			
				err = avc_has_perm_audit(dso_sid, node_sid,
							 SECCLASS_NODE, 
							 NODE__ENFORCE_DEST, adp);
				if (err)
					return err;
			} else {
				dso_sid = isec->peer_sid;
			}

			err = avc_has_perm_audit(isec->sid, dso_sid,
						 isec->sclass, SOCKET__SENDTO, 
						 adp);
			if (err)
				return err;
		}

		if (msg_sid  && msg_sid != isec->sid) {
			err = avc_has_perm_audit(isec->sid, msg_sid,
						 isec->sclass, SOCKET__SEND_MSG, adp);
			if (err) 
				return err;
		}
	}
	/* Copy the destination SID and message SID into the inode security
	 * structure so they can be copied into the SKB later.
	 * XXX Need to bind the (msid, dsid) pair to the particular message 
	 * in some manner so that extsocket_skb_set_owner_w can
	 * ensure that it is only applied to the corresponding network buffers.
	 */
	if ( (dso_sid) && (dso_sid != SECINITSID_ANY_SOCKET) )
		isec->dsid = dso_sid;

	if (sk->type != SOCK_STREAM)
		isec->msid = msg_sid;

	return 0;
}

static inline void extsocket_recvmsg(struct socket *sock,
	                             struct task_security_struct *tsec,
				     struct inode_security_struct *isec)
{
	if (sock->sk->type == SOCK_STREAM) {
		tsec->out_sid[0] = isec->peer_sid; /* SSO SID */
		tsec->out_sid[1] = isec->peer_sid; /* MSG SID */
	}
}

static inline void extsocket_getsockname(struct task_security_struct *tsec,
					 struct inode_security_struct *isec)
{
	tsec->out_sid[0] = isec->sid;
}

static inline void extsocket_getpeername(struct task_security_struct *tsec,
					 struct inode_security_struct *isec)
{
	tsec->out_sid[0] = isec->peer_sid;
}

#ifndef MODULE
extern struct or_calltable or_ipv4;
static inline void selinux_socket_reset(struct sock *sk, struct sk_buff *skb)
{
	if (sk->family != PF_INET)
		return;
	if (skb->nh.iph->version != 4)
		return;

	if (skb->nh.iph->protocol == IPPROTO_TCP) {
		or_ipv4.send_reset(skb);
	} else if (skb->nh.iph->protocol == IPPROTO_UDP) {
		icmp_send(skb, ICMP_DEST_UNREACH, ICMP_PORT_UNREACH, 0);
	}
}
#else
static inline void selinux_socket_reset(struct sock *sk, struct sk_buff *skb)
{
	return;
}
#endif

static inline int extsocket_sock_rcv_skb(struct sock *sk, 
					 struct sk_buff *skb,
					 struct inode_security_struct *isec,
					 struct skb_security_struct *ssec,
					 struct avc_audit_data *adp)
{
	int err;

	if (isec->sclass == SECCLASS_TCP_SOCKET) {
		struct tcphdr *th = skb->h.th;

		switch (sk->state) {
		case TCP_LISTEN:
			if (th->syn) {
				security_id_t newconn_sid = ssec->ssid;
				if (isec->useclient) {
					if (newconn_sid != isec->sid) {
						err = avc_has_perm_ref_audit(
							isec->sid,
							newconn_sid,
							isec->sclass,
							TCP_SOCKET__NEWCONN,
							&isec->avcr, adp);
						if (err) {
							return err;
						}
					}
				} else {
					newconn_sid = isec->newconn_sid;
				} 

				if (newconn_sid != isec->sid) {
					err = avc_has_perm_ref_audit(newconn_sid,
								     ssec->ssid,
								     isec->sclass, 
								     TCP_SOCKET__ACCEPTFROM, 
								     &isec->avcr, adp);
					if (err)
						return err;
				}

				if (ssec->dsid != SECINITSID_UNLABELED &&
				    ssec->dsid != isec->newconn_sid) {
					if (net_ratelimit()) {
						printk(KERN_WARNING
						       "TCP_LISTEN saddr=%d.%d.%d.%d:%d daddr=%d.%d.%d.%d:%d dsid=%d newconn_sid=%d\n",
						       NIPQUAD(skb->nh.iph->saddr),
						       ntohs(th->source),
						       NIPQUAD(skb->nh.iph->daddr),
						       ntohs(th->dest),
						       ssec->dsid, 
						       isec->newconn_sid);
					}
					selinux_socket_reset(sk, skb);
					return -EACCES;
				}

				isec->peer_sid = ssec->ssid;
	
			}
			break;
		case TCP_SYN_SENT:
			if (!th->rst && (th->ack || th->syn)) {
				if (isec->peer_sid != SECINITSID_ANY_SOCKET &&
				    ssec->ssid != isec->peer_sid) {
					if (net_ratelimit()) {
						printk(KERN_WARNING
						       "TCP_SYN_SENT saddr=%d.%d.%d.%d:%d daddr=%d.%d.%d.%d:%d ssid=%d dsid=%d msid=%d sid=%d peer_sid=%d\n",
						       NIPQUAD(skb->nh.iph->saddr),
						       ntohs(th->source),
						       NIPQUAD(skb->nh.iph->daddr),
						       ntohs(th->dest),
						       ssec->ssid, ssec->dsid,
						       ssec->msid,
						       isec->sid, isec->peer_sid);
					}
					/* behave like tcp_reset() */
#ifndef MODULE
					sk->err = ECONNREFUSED;
					if (!sk->dead)
						sk->error_report(sk);
					tcp_done(sk);
#endif
					return -EACCES;
				}

				isec->peer_sid = ssec->ssid;
				isec->conn_perm = TCP_SOCKET__CONNECTTO;
			}
			break;
		case TCP_ESTABLISHED:
			/* Revalidate the connection permission. */
			err = avc_has_perm_audit(isec->sid, ssec->ssid,
						  isec->sclass, isec->conn_perm, 
						  adp);
			if (err)
				return err;
			break;
		}
	} else if (sk->type != SOCK_STREAM) {
		if ((isec->peer_sid != SECINITSID_ANY_SOCKET &&
	 	     ssec->ssid != isec->peer_sid) ||
		    (ssec->dsid != SECINITSID_UNLABELED &&
		     ssec->dsid != isec->sid)) {
			if (net_ratelimit())
			    printk(KERN_WARNING
				"Sock rcv saddr=%d.%d.%d.%d daddr=%d.%d.%d.%d ssid=%d dsid=%d msid=%d sid=%d peer_sid=%d\n",
				NIPQUAD(skb->nh.iph->saddr),
				NIPQUAD(skb->nh.iph->daddr),
				ssec->ssid, ssec->dsid, ssec->msid,
				isec->sid, isec->peer_sid);
			selinux_socket_reset(sk, skb);
			return -EACCES;
        	}

		/* recv_msg is implicitly granted if the SIDs are equal */
		if (ssec->ssid != ssec->msid) {
			err = avc_has_perm_ref_audit(isec->sid, ssec->msid,
						isec->sclass, SOCKET__RECV_MSG,
						&isec->avcr, adp);
			if (err) {
				selinux_socket_reset(sk, skb);
				return err;
			}
		}
	}

	return 0;
}

static inline void extsocket_tcp_connection_request(struct sock *sk,
						    struct sk_buff *skb,
						    struct open_request *req)
{
        struct open_request_security_struct *orsec;
	struct inode_security_struct *isec;
	struct socket *sock;
	sock = sk->socket;
	if (sock == NULL)	/* This is not a user socket */
		return;

	orsec = req->security;

	isec = SOCK_INODE(sock)->i_security;

	if (isec->useclient) {
        	struct skb_security_struct *skbsec = skb->lsm_security;
		orsec->newconn_sid = skbsec->ssid;
	} else {
		orsec->newconn_sid = isec->newconn_sid;
	}
	return;
}

static inline void extsocket_tcp_synack(struct sock *sk, struct sk_buff *skb,
				   struct open_request *req)
{
	struct open_request_security_struct *orsec;
	struct skb_security_struct *skbsec;

	orsec = req->security;
	skbsec = skb->lsm_security;

	/*
	 * Although the SYN-ACK is sent from the listening socket,
	 * it is labeled with the SID of the server socket that will
	 * be created by the connection.  This permits the client
	 * to determine the server socket SID during connection
	 * establishment.  The server socket SID will be the same
	 * as the listening socket SID unless listen_secure was used.
	 */
	skbsec->ssid = orsec->newconn_sid;
	skbsec->msid = orsec->newconn_sid;
	
	return;
}

static inline void extsocket_tcp_create_openreq_child(
					struct sock_security_struct *newsksec,
					struct inode_security_struct *isec,
					struct sk_buff *skb,
					struct open_request *req)
{
	struct skb_security_struct *skbsec;

	skbsec = skb->lsm_security;

	if (isec && isec->useclient) {
        	struct open_request_security_struct *orsec = req->security;
		newsksec->sid = orsec->newconn_sid;
	}
	newsksec->peer_sid = skbsec->ssid;

	return;
}

static inline int extsocket_unix_stream_connect(struct inode_security_struct *isec,
						struct inode_security_struct *other_isec,
						struct sock *newsk,
						struct avc_audit_data *adp)
{
	int err;
	struct sock_security_struct *sksec;

	sksec = newsk->security;

	if (other_isec->useclient) {
		if (isec->sid != other_isec->sid) {
			err = avc_has_perm_ref_audit(
				other_isec->sid,
				isec->sid,
				isec->sclass,
				UNIX_STREAM_SOCKET__NEWCONN,
				&isec->avcr, adp);
			if (err)
				return err;
		}
		/* XXX Unreliable.  Need per-connection value. */
		other_isec->newconn_sid = isec->sid;
	}

	if (other_isec->newconn_sid != other_isec->sid) {
		err = avc_has_perm_ref_audit(isec->sid, other_isec->newconn_sid,
					     isec->sclass,
					     UNIX_STREAM_SOCKET__CONNECTTO,
					     &other_isec->avcr, adp);
		if (err)
			return err;
	}

	if (isec->dsid && isec->dsid != other_isec->newconn_sid) {
		if (net_ratelimit()) {
			printk(KERN_WARNING "UNIX CONNECT client dsid %d server sid %d\n",
			       isec->dsid, other_isec->newconn_sid);
		}
		return -EACCES;
	}
	isec->dsid = 0;

	isec->peer_sid = other_isec->newconn_sid;
	isec->conn_perm = UNIX_STREAM_SOCKET__CONNECTTO;

	/* Set the new sock peer SID to sending socket SID */
	sksec->peer_sid = isec->sid;

	return 0;
}

static inline int extsocket_unix_may_send(struct inode_security_struct *isec,
					  struct inode_security_struct *other_isec,
					  struct avc_audit_data *adp)
{
	if ((other_isec->peer_sid != SECINITSID_ANY_SOCKET &&
	     isec->sid != other_isec->peer_sid) ||
	    (isec->dsid && isec->dsid != other_isec->sid)) {
		if (net_ratelimit()) {
			printk(KERN_WARNING "UNIX SEND sender sid %d sender dsid %d receiver sid %d receiver peer_sid %d\n",
			       isec->sid, isec->dsid, other_isec->sid, other_isec->peer_sid);
		}
		return -EACCES;
	}
	isec->dsid = 0;
	
	return 0;
}


static inline void extsocket_skb_set_owner_w(struct sk_buff *skb,
					     struct sock *sk,
					     struct skb_security_struct *ssec,
					     struct inode_security_struct *isec)
{
	/* 
	 * XXX Unreliable, see comment in extsocket_sendmsg.
	 */

	if (sk->type != SOCK_STREAM && isec->msid) {
		/* Use the specified message SID for the datagram. */
		ssec->msid = isec->msid;
		isec->msid = 0;
	}

	if (isec->dsid) {
		/* Use the specified destination SID. */
	 	ssec->dsid = isec->dsid;
		isec->dsid = 0;
	}
}

static inline void extsocket_skb_recv_datagram(struct sk_buff *skb,
					       struct sock *sk)
{
	struct skb_security_struct *ssec;
	struct task_security_struct *tsec;

	ssec = skb->lsm_security;
	tsec = current->security;

	tsec->out_sid[0] = ssec->ssid; /* SSO SID */
	tsec->out_sid[1] = ssec->msid; /* MSG SID */
}



#else

static inline int extsocket_open_request_alloc_security(struct open_request *req)
{
	return 0;
}

static inline void extsocket_open_request_free_security(struct open_request *req)
{
	return;
}

static inline void extsocket_init(struct inode_security_struct *isec)
{
	return;
}

static inline security_id_t extsocket_create(struct task_security_struct *tsec)
{
	return tsec->sid;
}

static inline int extsocket_connect(struct socket *sock,
				    struct sockaddr *address,
				    int addrlen,
				    struct task_security_struct *tsec,
				    struct inode_security_struct *isec,
				    struct avc_audit_data *adp)
{
	return 0;
}

static inline int extsocket_listen(struct socket *sock,
	                           struct task_security_struct *tsec,
				   struct inode_security_struct *isec,
				   struct avc_audit_data *adp)
{
	return 0;
}

static inline void extsocket_accept(struct inode_security_struct *isec, 
				    struct inode_security_struct *newisec)
{
	return;
}

static inline void extsocket_post_accept(struct socket *sock,
					 struct socket *newsock)
{
	return;
}

static inline int extsocket_sendmsg(struct socket *sock,
				    struct msghdr *msg,
	                            struct task_security_struct *tsec,
				    struct inode_security_struct *isec,
				    struct avc_audit_data *adp)
{
	struct sock *sk = sock->sk;
	int err;

	if (sk->family == PF_INET && sk->type != SOCK_STREAM) {
		err = avc_has_perm_audit(isec->sid, SECINITSID_ANY_SOCKET,
					 isec->sclass, SOCKET__SENDTO, 
					 adp);
		if (err)
			return err;
	}
	return 0;
}

static inline void extsocket_recvmsg(struct socket *sock,
	                             struct task_security_struct *tsec,
				     struct inode_security_struct *isec)
{
	return;
}

static inline void extsocket_getsockname(struct task_security_struct *tsec,
					 struct inode_security_struct *isec)
{
	return;
}

static inline void extsocket_getpeername(struct task_security_struct *tsec,
					 struct inode_security_struct *isec)
{
	return;
}

static inline int extsocket_sock_rcv_skb(struct sock *sk, 
					 struct sk_buff *skb,
					 struct inode_security_struct *isec,
					 struct skb_security_struct *ssec,
					 struct avc_audit_data *adp)
{
	return 0;
}

static inline void extsocket_tcp_connection_request(struct sock *sk,
						    struct sk_buff *skb,
						    struct open_request *req)
{
	return;
}

static inline void extsocket_tcp_synack(struct sock *sk, struct sk_buff *skb,
					struct open_request *req)
{
	return;
}

static inline void extsocket_tcp_create_openreq_child(
					struct sock_security_struct *newsksec,
					struct inode_security_struct *isec,
					struct sk_buff *skb,
					struct open_request *req)
{
	return;
}

static inline int extsocket_unix_stream_connect(struct inode_security_struct *isec,
						struct inode_security_struct *other_isec,
						struct sock *newsk,
						struct avc_audit_data *adp)
{
	return 0;
}

static inline int extsocket_unix_may_send(struct inode_security_struct *isec,
					  struct inode_security_struct *other_isec,
					  struct avc_audit_data *adp)
{
	return 0;
}

static inline void extsocket_skb_set_owner_w(struct sk_buff *skb,
					     struct sock *sk,
					     struct skb_security_struct *ssec,
					     struct inode_security_struct *isec)
{
	return;
}

static inline void extsocket_skb_recv_datagram(struct sk_buff *skb,
					       struct sock *sk)

{
	return;
}

#endif

#endif
