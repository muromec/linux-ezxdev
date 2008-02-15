#ifndef AC_POLICY_H
#define AC_POLICY_H
/*================================================================================
                                                                               
                      Header Name: ac_policy.h

General Description: Defines policy structures 
 
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
                            Modification     Tracking
Author(core ID)                 Date          Number      Description of Changes
------------------------    ------------    ----------   -----------------------------------
Zhu zhifu (w20598)           01/20/2005     LIBff54376   Init version   

==================================================================================*/
/*===============================================================================
                                 INCLUDE FILES
================================================================================*/
#include "ac_global.h"

/*================================================================================
                                   CONSTANTS
================================================================================*/
#define AC_PATH_MAX_CHAR          100 
#define AC_POLICY_STRING          "MOTOAC"
#define AC_POLICY_VERSION_1         0x00010000
#define AC_POLICY_VERSION_2         0x00020000
#define AC_POLICY_PATH            "/etc/acpolicy.cfg"

/*================================================================================
                                     ENUMS
================================================================================*/
enum {
    AC_SYM_DOMAIN = 0,
    AC_SYM_PROCESS,
    AC_SYM_RESOURCE,
    AC_SYM_CLASS,
    AC_SYM_DEFAULT_SID,
    AM_SYM_NUM,
};

/*================================================================================
                          STRUCTURES AND OTHER TYPEDEFS
================================================================================*/
/* the common list node */
typedef struct ac_list_node {
    void *elem;
    struct ac_list_node *pnext;
} list_node_t;

/* the common list head */
typedef struct ac_list_head {
    char *name;
    int elem_num;
    list_node_t *node;
} list_head_t;

typedef struct defdom_datum {
    int did;
    struct defdom_datum *pnext;
} defdom_datum_t;

/* the map of the policy file in RAM */
typedef struct policydb {
    list_head_t plist[AC_SYM_DEFAULT_SID + 1];
#define domain_elem   plist[AC_SYM_DOMAIN]
#define process_elem  plist[AC_SYM_PROCESS]
#define resource_elem plist[AC_SYM_RESOURCE]
#define class_elem    plist[AC_SYM_CLASS]
#define default_sid   plist[AC_SYM_DEFAULT_SID]
    int default_rec_rule;  /* 1:bypath; 0:not bypath */
    unsigned long policy_ver;
    defdom_datum_t *default_domain_no;
}policydb_t;

/* domain information */
typedef struct dom_datum {
    int dno;   /* the serial number of domain type */
    char *name; /* domain name */
    void *apolicy;   /* access policies */
    int def_proc_no; /* default process type */
} dom_datum_t;

/* process information */
typedef struct proc_datum {
    int pno;   /* the serial number of process type */
    char *name; 
    int dom_type_no; /* which domain that the process belong to */
    int spec_attr;   /* special privileges for itself. such as "default process" */
    void *sp_policy;  /* sepcial privileges for (security/filesystem/system/.. class) */
    void *cp_policy;  /* create policies of processes */
    void *co_policy;  /* create policies of resource objects */
} proc_datum_t;

/* resource information */
typedef struct reso_datum {
    int rno;
    char *name;
} reso_datum_t;

/* default process defines and resource defines */
typedef struct dsid_datum {
    int path_length;
    int sid_type;          /* process/object */
    int def_type_no;
    char *full_path;
} dsid_datum_t;

/* class information */
typedef struct class_datum {
    int class_no;
    char *name;
    int all_pri_bits; /* all support operations */
} class_datum_t;

/* create policy of resource objects */
typedef struct co_datum {
    int pare_type_no;
    int class_no;      /* resource class type */
    int reso_type_no;
} co_datum_t;   

/* create policy of processes */
typedef struct cp_datum {
    int pare_type_no;
    int file_type_no;
    int child_type_no;
} cp_datum_t;

/* specify policy */
typedef struct sp_datum {
    int class_no;
    int pri_bitmap;
} sp_datum_t;

/* access policies */
typedef struct ap_datum {
    int action;
    int reso_type_no;
    int class_no;
    int priv_bitmap;
} ap_datum_t;

typedef int (*fsort)(list_head_t *phead,list_node_t *pnode,list_node_t **pret);

#define AC_POLICY_READ(type) ac_policy_read_ ## type 

/*================================================================================
                           FUNCTIONS
================================================================================*/
extern list_node_t *ac_find_node(list_head_t *phead,int node_type,void *elem,int elem_type);
extern void *ac_malloc_datum(int elem_type);
extern int ac_policydb_init(policydb_t *policydb);
extern int ac_policydb_free(policydb_t *policydb);
extern list_node_t *ac_list_new_node(void *elem);
extern list_head_t *ac_list_new_head(const char *name);
extern void ac_list_free_head(list_head_t *phead,int type); 
extern int ac_list_add_node(list_head_t *phead,list_node_t *pnode,fsort fn);
extern int ac_sort_default_sid(list_head_t *phead,list_node_t *pnode,list_node_t **pret);

extern list_node_t *ac_get_defpt(void);
extern int ac_object_getdefsid(char *path,int type);
extern int ac_socket_getsid(int proc_no,int class_no);
extern int ac_process_getsid(int parent_no,int file_type_no);
extern int ac_task_has_perm(int proc_no,int tproc_no,int perm);
extern int ac_check_ac_perm(int dom_no,int reso_no,int reso_type,int oper_type);

extern int ac_proc_to_dom(int proc_no);
extern int ac_dom_default_proc(int dom_no);
extern int ac_dom_exist(int dom_no);	
extern int ac_load_policy(void);	
extern unsigned long ac_get_version(void);	
#endif  /*AC_POLICY_H*/

