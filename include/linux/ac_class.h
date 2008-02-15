#ifndef AC_CLASS_H
#define AC_CLASS_H
/*================================================================================
                                                                               
                      Header Name: ac_class.h

General Description: Common class type definitions
 
==================================================================================
 
   Copyright (C) 2005 - Motorola
 
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 2 as
   published by the Free Software Foundation.
 
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
 
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
  
Revision History:
   2005-Jan - Motorola - Created by Zhu Zhifu for Security Access Control Module
==================================================================================*/

#include <asm/errno.h>

/*================================================================================
                                   CONSTANTS
================================================================================*/
/*
 * Security object class definitions
 */
#define AC_CLASS_SECURITY                                1
#define AC_CLASS_PROCESS                                 2
#define AC_CLASS_SYSTEM                                  3
#define AC_CLASS_CAPABILITY                              4
#define AC_CLASS_FILESYSTEM                              5
#define AC_CLASS_FILE                                    6
#define AC_CLASS_DIR                                     7
#define AC_CLASS_FD                                      8
#define AC_CLASS_LNK_FILE                                9
#define AC_CLASS_CHR_FILE                                10
#define AC_CLASS_BLK_FILE                                11
#define AC_CLASS_SOCK_FILE                               12
#define AC_CLASS_FIFO_FILE                               13
#define AC_CLASS_SOCKET                                  14
#define AC_CLASS_TCP_SOCKET                              15
#define AC_CLASS_UDP_SOCKET                              16
#define AC_CLASS_RAWIP_SOCKET                            17
#define AC_CLASS_NODE                                    18
#define AC_CLASS_NETIF                                   19
#define AC_CLASS_NETLINK_SOCKET                          20
#define AC_CLASS_PACKET_SOCKET                           21
#define AC_CLASS_KEY_SOCKET                              22
#define AC_CLASS_UNIX_STREAM_SOCKET                      23
#define AC_CLASS_UNIX_DGRAM_SOCKET                       24
#define AC_CLASS_SEM                                     25
#define AC_CLASS_MSG                                     26
#define AC_CLASS_MSGQ                                    27
#define AC_CLASS_SHM                                     28
#define AC_CLASS_IPC                                     29

/* file operations */
#define AC_FILE_IOCTL                               0x00000001UL
#define AC_FILE_RENAME                              0x00000002UL
#define AC_FILE_WRITE                               0x00000004UL
#define AC_FILE_CREATE                              0x00000008UL
#define AC_FILE_GETATTR                             0x00000010UL
#define AC_FILE_SETATTR                             0x00000020UL
#define AC_FILE_LOCK                                0x00000040UL
#define AC_FILE_EXECUTE                             0x00000080UL
#define AC_FILE_UNLINK                              0x00000100UL
#define AC_FILE_QUOTAON                             0x00000200UL
#define AC_FILE_LINK                                0x00000400UL
#define AC_FILE_APPEND                              0x00000800UL
#define AC_FILE_SWAPON                              0x00001000UL
#define AC_FILE_MOUNTON                             0x00002000UL
#define AC_FILE_READ                                0x00004000UL

/* directory operations */
#define AC_DIR_EXECUTE                              0x00000001UL
#define AC_DIR_UNLINK                               0x00000002UL
#define AC_DIR_SETATTR                              0x00000004UL
#define AC_DIR_QUOTAON                              0x00000008UL
#define AC_DIR_LINK                                 0x00000010UL
#define AC_DIR_WRITE                                0x00000020UL
#define AC_DIR_IOCTL                                0x00000040UL
#define AC_DIR_READ                                 0x00000080UL
#define AC_DIR_RENAME                               0x00000100UL
#define AC_DIR_APPEND                               0x00000200UL
#define AC_DIR_LOCK                                 0x00000400UL
#define AC_DIR_SWAPON                               0x00000800UL
#define AC_DIR_GETATTR                              0x00001000UL
#define AC_DIR_MOUNTON                              0x00002000UL
#define AC_DIR_CREATE                               0x00004000UL
#define AC_DIR_ADD_NAME                             0x00008000UL
#define AC_DIR_REMOVE_NAME                          0x00010000UL
#define AC_DIR_REPARENT                             0x00020000UL
#define AC_DIR_SEARCH                               0x00040000UL
#define AC_DIR_RMDIR                                0x00080000UL

/* Filesystem Operations */
#define AC_FILESYSTEM_MOUNT                         0x00000001UL
#define AC_FILESYSTEM_REMOUNT                       0x00000002UL
#define AC_FILESYSTEM_UNMOUNT                       0x00000004UL
#define AC_FILESYSTEM_GETATTR                       0x00000008UL
#define AC_FILESYSTEM_RELABELFROM                   0x00000010UL
#define AC_FILESYSTEM_RELABELTO                     0x00000020UL
#define AC_FILESYSTEM_TRANSITION                    0x00000040UL
#define AC_FILESYSTEM_ASSOCIATE                     0x00000080UL
#define AC_FILESYSTEM_QUOTAMOD                      0x00000100UL
#define AC_FILESYSTEM_QUOTAGET                      0x00000200UL

/* IPC Operations */
#define AC_IPC_SETATTR                              0x00000001UL
#define AC_IPC_READ                                 0x00000002UL
#define AC_IPC_ASSOCIATE                            0x00000004UL
#define AC_IPC_DESTROY                              0x00000008UL
#define AC_IPC_UNIX_WRITE                           0x00000010UL
#define AC_IPC_CREATE                               0x00000020UL
#define AC_IPC_UNIX_READ                            0x00000040UL
#define AC_IPC_GETATTR                              0x00000080UL
#define AC_IPC_WRITE                                0x00000100UL

/* Process Operations */
#define AC_PROCESS_FORK                             0x00000001UL
#define AC_PROCESS_TRANSITION                       0x00000002UL
#define AC_PROCESS_SIGCHLD                          0x00000004UL
#define AC_PROCESS_SIGKILL                          0x00000008UL
#define AC_PROCESS_SIGSTOP                          0x00000010UL
#define AC_PROCESS_SIGNULL                          0x00000020UL
#define AC_PROCESS_SIGNAL                           0x00000040UL
#define AC_PROCESS_PTRACE                           0x00000080UL
#define AC_PROCESS_GETSCHED                         0x00000100UL
#define AC_PROCESS_SETSCHED                         0x00000200UL
#define AC_PROCESS_GETSESSION                       0x00000400UL
#define AC_PROCESS_GETPGID                          0x00000800UL
#define AC_PROCESS_SETPGID                          0x00001000UL
#define AC_PROCESS_GETCAP                           0x00002000UL
#define AC_PROCESS_SETCAP                           0x00004000UL
#define AC_PROCESS_SHARE                            0x00008000UL

/* system permissions */
#define AC_SYSTEM_IPC_INFO                          0x00000001UL
#define AC_SYSTEM_NFSD_CONTROL                      0x00000002UL
#define AC_SYSTEM_BDFLUSH                           0x00000004UL
#define AC_SYSTEM_SYSLOG_READ                       0x00000008UL
#define AC_SYSTEM_SYSLOG_MOD                        0x00000010UL
#define AC_SYSTEM_SYSLOG_CONSOLE                    0x00000020UL
#define AC_SYSTEM_REBOOT                            0x00000040UL

/* security operations */
#define AC_SECURITY_ACCESS                          0x00000001UL

/* is a default process type */
#define AC_PROCESS_ISDEFAULT                        0x00000001UL

/* Socket Operations */
#define AC_SOCKET_RECV_MSG                          0x00000001UL
#define AC_SOCKET_SETOPT                            0x00000002UL
#define AC_SOCKET_APPEND                            0x00000004UL
#define AC_SOCKET_SETATTR                           0x00000008UL
#define AC_SOCKET_SENDTO                            0x00000010UL
#define AC_SOCKET_GETOPT                            0x00000020UL
#define AC_SOCKET_READ                              0x00000040UL
#define AC_SOCKET_SHUTDOWN                          0x00000080UL
#define AC_SOCKET_LISTEN                            0x00000100UL
#define AC_SOCKET_BIND                              0x00000200UL
#define AC_SOCKET_WRITE                             0x00000400UL
#define AC_SOCKET_ACCEPT                            0x00000800UL
#define AC_SOCKET_CONNECT                           0x00001000UL
#define AC_SOCKET_LOCK                              0x00002000UL
#define AC_SOCKET_IOCTL                             0x00004000UL
#define AC_SOCKET_CREATE                            0x00008000UL
#define AC_SOCKET_NAME_BIND                         0x00010000UL
#define AC_SOCKET_SEND_MSG                          0x00020000UL
#define AC_SOCKET_RECVFROM                          0x00040000UL
#define AC_SOCKET_GETATTR                           0x00080000UL

/* default SID */
#define AC_DEFAULT_UNLABELED                        0
#define AC_CLASS_KERNEL                             1 

/* DENY and GRANT */
#define AC_CHECK_DENY                               (-EACCES)
#define AC_CHECK_GRANT                              0

/* "ANY" KEYWORD in policy file */
#define AC_FILETYPE_ANY                             0xFFFFFFFF


/* defined for flags */
#define AC_POLICY_MAGIC           0xF1E2D3C4

/*================================================================================
                                     ENUMS
================================================================================*/
/* for list type */
typedef enum {
    AC_LIST_DOMAIN = 0,
    AC_LIST_PROCESS,
    AC_LIST_RESOURCE,
    AC_LIST_CLASS,
    AC_LIST_DEFAULT_SID,
    AC_LIST_AP,
    AC_LIST_CP,
    AC_LIST_CO,
    AC_LIST_SP,
    AC_LIST_NUM,
} AC_LIST_TYPE;                       

/* for default sid type */
typedef enum {
    AC_DEFAULT_TYPE_PROCESS,
    AC_DEFAULT_TYPE_OBJECT,
} AC_CREATE_TYPE;

/* find a node by name? no? or element?  */
typedef enum {
    AC_FIND_BY_NO,
    AC_FIND_BY_NAME,
    AC_FIND_BY_ELEMENT,
} AC_FIND_TYPE;
	
/* defined for syscall */
typedef enum {
    AC_SYSCALL_GETDOMAIN = 1,
    AC_SYSCALL_GETPROCNO,
    AC_SYSCALL_SETDOMAINTYPE,
    AC_SYSCALL_EXECV,
    AC_SYSCALL_GETRESSID,
    AC_SYSCALL_ENABLE,
    AC_SYSCALL_NUM,
} AC_SYSCALL_TYPE;
/*================================================================================
                                     MACROS
================================================================================*/
#define AC_CLASS_OPERATIONS(class,operation)  AC_ ## class ## _ operation

#endif  /*AC_CLASS_H*/

