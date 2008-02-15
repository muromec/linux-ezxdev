/*================================================================================
                                                                               
Module Name:  ac_policy.c

General Description: maintain all types of policy.

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
Author (core ID)                Date          Number      Description of Changes
------------------------    ------------    ----------   -------------------------
Zhu zhifu (w20598)           01/20/2005     LIBff54376   Init version     

==================================================================================*/
/*================================================================================
                                 INCLUDE FILES
================================================================================*/
#include "ac_policy.h"

/*================================================================================
                                LOCAL CONSTANTS
================================================================================*/

/*================================================================================
                   LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
================================================================================*/
typedef void (*list_compare)(int ssid,int tsid,int classid,int operationid);
typedef int (*fcompelem)(list_node_t *pnode, void *elem,int comp_type);

/*================================================================================
                                  LOCAL MACROS
================================================================================*/

/*================================================================================
                                  LOCAL STRUCTURES
================================================================================*/
/* element type compare functions */
typedef struct ac_compare_fun {
    int typeno;
    fcompelem fn;
    char *name;
} ac_compare_fun_t;

/*================================================================================
                            LOCAL FUNCTION PROTOTYPES
================================================================================*/
static int ac_list_compare_domain(list_node_t *pnode,void *elem,int comp_type);
static int ac_list_compare_process(list_node_t *pnode,void *elem,int comp_type);
static int ac_list_compare_resource(list_node_t *pnode,void *elem,int comp_type);
static int ac_list_compare_defsid(list_node_t *pnode,void *elem,int comp_type);
static int ac_list_compare_class(list_node_t *pnode,void *elem,int comp_type);
static int ac_list_compare_ap(list_node_t *pnode,void *elem,int comp_type);
static int ac_list_compare_cp(list_node_t *pnode,void *elem,int comp_type);
static int ac_list_compare_co(list_node_t *pnode,void *elem,int comp_type);  
static int ac_list_compare_sp(list_node_t *pnode,void *elem,int comp_type); 

static int ac_policy_read_domtype(policydb_t *pdb,FILE *fp); 
static int ac_policy_read_proctype(policydb_t *pdb,FILE *fp);
static int ac_policy_read_resotype(policydb_t *pdb,FILE *fp);
static int ac_policy_read_class(policydb_t *pdb,FILE *fp); 
static int ac_policy_read_dsid(policydb_t *pdb,FILE *fp,int type); 
static int ac_policy_read_cp(policydb_t *pdb,FILE *fp); 
static int ac_policy_read_co(policydb_t *pdb,FILE *fp); 
static int ac_policy_read_ap(policydb_t *pdb,FILE *fp);                                                   

static int ac_policy_read_sp(list_head_t *phead,FILE *fp);
/*================================================================================
                                LOCAL VARIABLES
================================================================================*/
int ac_policy_init = 0;
int ac_policy_enable = 1;

static ac_compare_fun_t ac_compare_fn[AC_LIST_NUM] =
 {{AC_LIST_DOMAIN,&ac_list_compare_domain,"dom"},
  {AC_LIST_PROCESS,&ac_list_compare_process,"proc"},
  {AC_LIST_RESOURCE,&ac_list_compare_resource,"reso"},
  {AC_LIST_CLASS,&ac_list_compare_class,"class"},
  {AC_LIST_DEFAULT_SID,&ac_list_compare_defsid,"dsid"},
  {AC_LIST_AP,&ac_list_compare_ap,"ap"},
  {AC_LIST_CP,&ac_list_compare_cp,"cp"},
  {AC_LIST_CO,&ac_list_compare_co,"co"},
  {AC_LIST_SP,&ac_list_compare_sp,"sp"},
 };
                                                            
/*================================================================================
                                GLOBAL VARIABLES
================================================================================*/
policydb_t policydb;

/*================================================================================
                            LOCAL POINTER DECLARATIONS
================================================================================*/
/*================================================================================
                                LOCAL FUNCTIONS
================================================================================*/
#if CONFIG_SECURITY_AC_DEVELOP
void ac_list_printk(list_head_t *phead,int type)                                                           
{
    list_node_t *pnode1,*pnode2;
    
    if (phead == NULL)
        return;
    
    pnode1 = pnode2 = phead->node;
    
    switch (type)
    {
    	case AC_LIST_DOMAIN:
    	{
    	    dom_datum_t *pnode_dom;
    	    
    	    printk("\n-----------------------------------\n");
    	    printk(" Domain defines:\n");
    	    while(pnode1)
    	    {
    	        printk("-------------------------\n");
    	        pnode_dom = (dom_datum_t *)pnode1->elem;
    	        if (pnode_dom->name) 
    	            printk("name: %s\n",pnode_dom->name);
    	        printk("No: %d\n",pnode_dom->dno);
    	        printk("Default process no: %d\n",pnode_dom->def_proc_no);
    	             	            
    	        ac_list_printk(pnode_dom->apolicy,AC_LIST_AP);
       	        pnode1 = pnode2->pnext;
    	        pnode2 = pnode1;
    	    }
    	    printk("\n-----------------------------------\n");
    	    break;   
    	}
    	case AC_LIST_PROCESS:
    	{
    	    proc_datum_t *pnode_proc;
    	    
    	    printk("\n-----------------------------------\n");
    	    printk(" Process defines:\n");
    	    while(pnode1)
    	    {
    	    	printk("-------------------------\n");
    	    	pnode_proc = (proc_datum_t *)pnode1->elem;
    	    	if (pnode_proc->name)
    	    	    printk("Name: %s\n",pnode_proc->name);
    	    	printk("No: %d\n",pnode_proc->pno);
    	    	printk("Domain no: %d\n",pnode_proc->dom_type_no);
    	    	printk("Spec_attr: 0x%x\n",pnode_proc->spec_attr);
    	    	
    	    	ac_list_printk(pnode_proc->co_policy,AC_LIST_CO);
    	    	ac_list_printk(pnode_proc->cp_policy,AC_LIST_CP);
    	    	ac_list_printk(pnode_proc->sp_policy,AC_LIST_SP);
    	    	pnode1 = pnode2->pnext;
    	    	pnode2 = pnode1;
    	    }
    	    printk("\n-----------------------------------\n");
    	    break;
    	}
    	case AC_LIST_RESOURCE:
    	{
    	    reso_datum_t *pnode_reso;
    	    
    	    printk("\n-----------------------------------\n");
    	    printk(" Resource defines:\n");
    	    while(pnode1)
    	    {
    	    	printk("-------------------------\n");
    	    	pnode_reso = (reso_datum_t *)pnode1->elem;
    	    	if (pnode_reso->name)
    	    	    printk("Name: %s\n",pnode_reso->name);
    	    	printk("No: %d\n",pnode_reso->rno);
    	    	pnode1 = pnode2->pnext;
    	    	pnode2 = pnode1;
    	    }
    	    break;
    	}
    	case AC_LIST_CLASS:
    	{
    	    class_datum_t *pnode_class;
    	    
    	    printk("\n-----------------------------------\n");
    	    printk(" Class defines:\n");
    	    while(pnode1)
    	    {
    	    	printk("\n-----------------------------------\n");
    	    	pnode_class = (class_datum_t *)pnode1->elem;
    	    	if (pnode_class->name)
    	    	    printk("Name: %s\n",pnode_class->name);
    	    	printk("No: %d\n",pnode_class->class_no);
    	    	printk("All_pri_bits: 0x%x\n",pnode_class->all_pri_bits);
    	    	pnode1 = pnode2->pnext;
    	    	pnode2 = pnode1;
    	    }
    	    break;
    	}
        case AC_LIST_DEFAULT_SID:
        {
            dsid_datum_t *pnode_dsid;
    	    
    	    printk("\n-----------------------------------\n");
    	    printk(" Default SID defines:\n");
    	    while(pnode1)
    	    {
    	    	printk("\n-----------------------------------\n");
    	    	pnode_dsid = (dsid_datum_t *)pnode1->elem;
    	    	printk("Full Path: %s\n",pnode_dsid->full_path);
    	    	printk("SID type: %d\n",pnode_dsid->sid_type);
    	    	printk("Default SID: %d\n",pnode_dsid->def_type_no);
    	    	pnode1 = pnode2->pnext;
    	    	pnode2 = pnode1;
    	    }
    	    break;
    	}
    	case AC_LIST_AP:
    	{
            ap_datum_t *pnode_ap;
    	    
    	    printk("\n-----------------------------------\n");
    	    printk(" Access Policy defines:\n");
    	    while(pnode1)
    	    {
    	    	printk("\n-----------------------------------\n");
    	    	pnode_ap = (ap_datum_t *)pnode1->elem;
		printk("action:");
                if (pnode_ap->action)
                    printk("ALLOW\n");
                else
                    printk("DENY\n");
    	    	printk("Resource Type No: %d\n",pnode_ap->reso_type_no);
    	    	printk("Class No: %d\n",pnode_ap->class_no);
    	    	printk("Perms: 0x%x\n",pnode_ap->priv_bitmap);
    	    	pnode1 = pnode2->pnext;
    	    	pnode2 = pnode1;
    	    }
    	    break;
    	}
    	case AC_LIST_CP:
    	{
            cp_datum_t *pnode_cp;
    	    
    	    printk("\n-----------------------------------\n");
    	    printk(" Create Process Policy defines:\n");
    	    while(pnode1)
    	    {
    	    	printk("\n-----------------------------------\n");
    	    	pnode_cp = (cp_datum_t *)pnode1->elem;
    	    	printk("Parent Type No: %d\n",pnode_cp->pare_type_no);
    	    	printk("File No: %d\n",pnode_cp->file_type_no);
    	    	printk("Child: %d\n",pnode_cp->child_type_no);
    	    	pnode1 = pnode2->pnext;
    	    	pnode2 = pnode1;
    	    }
    	    break;
    	}
    	case AC_LIST_CO:
    	{
            co_datum_t *pnode_co;
    	    
    	    printk("\n-----------------------------------\n");
    	    printk(" Create Object Policy defines:\n");
    	    while(pnode1)
    	    {
    	    	printk("\n-----------------------------------\n");
    	    	pnode_co = (co_datum_t *)pnode1->elem;
    	    	printk("Parent Type No: %d\n",pnode_co->pare_type_no);
    	    	printk("Class No: %d\n",pnode_co->class_no);
    	    	printk("Object Type No: %d\n",pnode_co->reso_type_no);
    	    	pnode1 = pnode2->pnext;
    	    	pnode2 = pnode1;
    	    }
    	    break;
    	}
        case AC_LIST_SP:
       {
            sp_datum_t *pnode_sp;
    	    
    	    printk("\n-----------------------------------\n");
    	    printk(" Sepc Policy defines:\n");
    	    while(pnode1)
    	    {
    	    	printk("\n-----------------------------------\n");
    	    	pnode_sp = (sp_datum_t *)pnode1->elem;
    	    	printk("Class No: %d\n",pnode_sp->class_no);
    	    	printk("Perms: 0x%x\n",pnode_sp->pri_bitmap);
    	    	pnode1 = pnode2->pnext;
    	    	pnode2 = pnode1;
    	    }
    	    break;
    	}
    	default:
    	    return;
    }    
}       
#else
void ac_list_printk(list_head_t *phead,int type)
{
}
#endif

#if CONFIG_SECURITY_AC_DEVELOP
int ac_policydb_printk(policydb_t *policydb)                                                           
{
    int i;
     
    if (policydb == NULL)
        return -EINVAL;
        
    for (i=0;i<(AC_LIST_DEFAULT_SID + 1);i++)
        ac_list_printk(&policydb->plist[i],i);

    return 0; 
}       
#else
int ac_policydb_printk(policydb_t *policydb)
{
    return 0;
}
#endif


/*================================================================================
FUNCTION: ac_list_compare_domain          
DESCRIPTION: 
   find a domain node according to the domain name;
ARGUMENTS PASSED:
   list_node_t *phead;
   void        *elem;
   int         comp_type   : by name/no/elem
RETURN VALUE:
   int;  1:equal; 0:no 
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
================================================================================*/
static int ac_list_compare_domain(list_node_t *pnode,void *elem,int comp_type)                                                           
{
    dom_datum_t *pnode_dom, *pnode_tmp;
    int no,ret;
    
    pnode_tmp = (dom_datum_t *)pnode->elem;
    switch (comp_type)
    {
        case AC_FIND_BY_NO:
             no = *(int *)elem;
             ret = (pnode_tmp->dno == no);
             break;
        case AC_FIND_BY_NAME:
             ret = (strcmp(pnode_tmp->name,(char *)elem));
             break;
        case AC_FIND_BY_ELEMENT:
             pnode_dom = (dom_datum_t *)elem;
             ret = (!strcmp(pnode_tmp->name,pnode_dom->name));
             break;
        default:
             ret = -1;
             break;
    }
    
    return ret;
} 

/*================================================================================
FUNCTION: ac_list_compare_process          
DESCRIPTION: 
   find a process node according to the process name;
ARGUMENTS PASSED:
   list_node_t *phead;
   void        *elem;
   int         comp_type   : by name/no/elem
RETURN VALUE:
   int;  1:equal; 0:no 
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
================================================================================*/
static int ac_list_compare_process(list_node_t *pnode,void *elem,int comp_type)                                                           
{
    proc_datum_t *pnode_proc, *pnode_tmp;
    int no,ret;
    
    pnode_tmp = (proc_datum_t *)pnode->elem;
    switch (comp_type)
    {
        case AC_FIND_BY_NO:
             no = *(int *)elem;
             ret = (pnode_tmp->pno == no);
             break;
        case AC_FIND_BY_NAME:
             ret = (strcmp(pnode_tmp->name,(char *)elem));
             break;
        case AC_FIND_BY_ELEMENT:
             pnode_proc = (proc_datum_t *)elem;
             ret = (!strcmp(pnode_tmp->name,pnode_proc->name));
             break;
        default:
             ret = -1;
             break;
    }
    
    return ret;
}          

/*================================================================================
FUNCTION: ac_list_compare_resource          
DESCRIPTION: 
   find a resource node according to the resource name;
ARGUMENTS PASSED:
   list_node_t *phead;
   void        *elem;
   int         comp_type   : by name/no/elem
RETURN VALUE:
   int;  1:equal; 0:no 
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
================================================================================*/
static int ac_list_compare_resource(list_node_t *pnode,void *elem,int comp_type)                                                           
{
    reso_datum_t *pnode_reso, *pnode_tmp;
    int no,ret;
    
    pnode_tmp = (reso_datum_t *)pnode->elem;
    switch (comp_type)
    {
        case AC_FIND_BY_NO:
             no = *(int *)elem;
             ret = (pnode_tmp->rno == no);
             break;
        case AC_FIND_BY_NAME:
             ret = (strcmp(pnode_tmp->name,(char *)elem));
             break;
        case AC_FIND_BY_ELEMENT:
             pnode_reso = (reso_datum_t *)elem;
             ret = (!strcmp(pnode_tmp->name,pnode_reso->name));
             break;
        default:
             ret = -1;
             break;
    }

    return ret;
}          

/*================================================================================
FUNCTION: ac_list_compare_class          
DESCRIPTION: 
   find a class node according to the class name;
ARGUMENTS PASSED:
   list_node_t *phead;
   void        *elem;
   int         comp_type   : by name/no/elem
RETURN VALUE:
   int;  1:equal; 0:no 
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
================================================================================*/
static int ac_list_compare_class(list_node_t *pnode,void *elem,int comp_type)                                                           
{
    class_datum_t *pnode_class, *pnode_tmp;
    int no,ret;
    
    pnode_tmp = (class_datum_t *)pnode->elem;
    switch (comp_type)
    {
        case AC_FIND_BY_NO:
             no = *(int *)elem;
             ret = (pnode_tmp->class_no == no);
             break;
        case AC_FIND_BY_NAME:
             ret = (strcmp(pnode_tmp->name,(char *)elem));
             break;
        case AC_FIND_BY_ELEMENT:
             pnode_class = (class_datum_t *)elem;
             ret = (!strcmp(pnode_tmp->name,pnode_class->name));
             break;
        default:
             ret = -1;
             break;
    }
         
    return ret;
}          

/*================================================================================
FUNCTION: ac_list_compare_defsid          
DESCRIPTION: 
   find a default sid according to the path name and class type;
ARGUMENTS PASSED:
   list_node_t *phead;
   void        *elem;
   int         comp_type   : by name/no/elem
RETURN VALUE:
   int;  1:equal; 0:no 
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
================================================================================*/
static int ac_list_compare_defsid(list_node_t *pnode,void *elem,int comp_type)                                                           
{
    dsid_datum_t *pnode_dsid, *pnode_tmp;
    int ret = 0;
    char *delimiter;
    unsigned long ver;
    
    pnode_tmp = (dsid_datum_t *)pnode->elem;
    if (pnode_tmp == NULL)
        return 0;
        
    switch (comp_type)
    {
        case AC_FIND_BY_ELEMENT:
             pnode_dsid = (dsid_datum_t *)elem;
             ver = ac_get_version();
             if (ver == AC_POLICY_VERSION_1)
             {
                 ret = (!strncmp(pnode_tmp->full_path,pnode_dsid->full_path,pnode_tmp->path_length) && (pnode_tmp->sid_type == pnode_dsid->sid_type));
             }
             else if (ver == AC_POLICY_VERSION_2)
             {
                 if ((strcmp(pnode_tmp->full_path,pnode_dsid->full_path) == 0) && (pnode_tmp->sid_type == pnode_dsid->sid_type))
                 {
                     ret = 1;
                 }
                 else if (pnode_tmp->sid_type == pnode_dsid->sid_type)
                 {
                     delimiter = strchr(pnode_tmp->full_path, '*');
                     if(delimiter != NULL)
                     {
                         if (strncmp(pnode_tmp->full_path,pnode_dsid->full_path,pnode_tmp->path_length-1) == 0)
                         {
                             ret = 1;
                         }
                     }
                 }
             }
             else
                 ret = -1;
             break;
        default:
             ret = -1;
             break;
    }
 
    return ret;
}

/*================================================================================
FUNCTION: ac_list_compare_ap          
DESCRIPTION: 
   find a access control policy;
ARGUMENTS PASSED:
   list_node_t *phead;
   void        *elem;
   int         comp_type   : by name/no/elem
RETURN VALUE:
   int;  1:equal; 0:no 
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
================================================================================*/
static int ac_list_compare_ap(list_node_t *pnode,void *elem,int comp_type)                                                           
{
    ap_datum_t *pnode_ap, *pnode_tmp;
    int ret;
    
    pnode_tmp = (ap_datum_t *)pnode->elem;
    switch (comp_type)
    {
        case AC_FIND_BY_ELEMENT:
             pnode_ap = (ap_datum_t *)elem;
             ret =  ((pnode_ap->reso_type_no == pnode_tmp->reso_type_no)&&
                    (pnode_ap->class_no == pnode_tmp->class_no));
             break;
        default:
             ret = -1;
             break;
    }
        
    return ret;
}    

/*================================================================================
FUNCTION: ac_list_compare_cp          
DESCRIPTION: 
   find a create process policy;
ARGUMENTS PASSED:
   list_node_t *phead;
   void        *elem;
   int         comp_type   : by name/no/elem
RETURN VALUE:
   int;  1:equal; 0:no 
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
================================================================================*/
static int ac_list_compare_cp(list_node_t *pnode,void *elem,int comp_type)                                                           
{
    cp_datum_t *pnode_cp, *pnode_tmp;
    int ret;
    
    pnode_tmp = (cp_datum_t *)pnode->elem;
    switch (comp_type)
    {
        case AC_FIND_BY_ELEMENT:
             pnode_cp = (cp_datum_t *)elem;
             ret =  ((pnode_tmp->pare_type_no == pnode_cp->pare_type_no)&&
                    ((pnode_tmp->file_type_no == pnode_cp->file_type_no) || 
		    (pnode_tmp->file_type_no == AC_FILETYPE_ANY)));
             break;
        default:
             ret = -1;
             break;
    }
    
    return ret;
}    

/*================================================================================
FUNCTION: ac_list_compare_co          
DESCRIPTION: 
   find a create object policy;
ARGUMENTS PASSED:
   list_node_t *phead;
   void        *elem;
   int         comp_type   : by name/no/elem
RETURN VALUE:
   int;  1:equal; 0:no 
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
================================================================================*/
static int ac_list_compare_co(list_node_t *pnode,void *elem,int comp_type)                                                           
{
    co_datum_t *pnode_co, *pnode_tmp;
    int ret;
    
    pnode_tmp = (co_datum_t *)pnode->elem;
    switch (comp_type)
    {
        case AC_FIND_BY_ELEMENT:
             pnode_co = (co_datum_t *)elem;
             ret = ((pnode_tmp->pare_type_no == pnode_co->pare_type_no)&&
                    (pnode_tmp->class_no == pnode_co->class_no));
             break;
        default:
             ret = -1;
             break;
    }
    
    return ret;
}    

/*================================================================================
FUNCTION: ac_list_compare_sp          
DESCRIPTION: 
   find a especial policy;
ARGUMENTS PASSED:
   list_node_t *phead;
   void        *elem;
   int         comp_type   : by name/no/elem
RETURN VALUE:
   int;  1:equal; 0:no 
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
================================================================================*/
static int ac_list_compare_sp(list_node_t *pnode,void *elem,int comp_type)                                                           
{
    sp_datum_t *pnode_tmp;
    int class_no,ret;
    
    pnode_tmp = (sp_datum_t *)pnode->elem;
    switch (comp_type)
    {
        case AC_FIND_BY_NO:
	     class_no = *(int *)elem;
             ret = (pnode_tmp->class_no == class_no );
             break;
        default:
             ret = -1;
             break;
    }
    
    return ret;
}    

/*================================================================================
FUNCTION: ac_list_lastnode          
DESCRIPTION: 
   find the last node in a list
ARGUMENTS PASSED:
   list_head_t *phead;
RETURN VALUE:
   list_node_t * 
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
static list_node_t *ac_list_lastnode(list_head_t *phead)                                                           
{
    list_node_t *pnode_tmp;
	
    pnode_tmp = phead->node;
    if (!pnode_tmp)
        return NULL;
        
    while (pnode_tmp->pnext)
        pnode_tmp = pnode_tmp->pnext;
            
    return pnode_tmp; 
}       
/*================================================================================
                                GLOBAL FUNCTIONS
================================================================================*/
/*================================================================================
FUNCTION: ac_find_node          
DESCRIPTION: 
   find a node in a list 
ARGUMENTS PASSED:
   list_head_t *phead;
   int         node_type;  which list?
   void        *elem;
   int         elem_type;  domain/process/resource/class/default/AP/CP/CO 
RETURN VALUE:
   list_node_t *;
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
list_node_t *ac_find_node(list_head_t *phead,int node_type,void *elem,int elem_type)
{
    list_node_t *pnode;
    fcompelem fn;
    int ret;
   
    if ((phead == NULL) || (elem == NULL))
        return NULL;
    
    if (elem_type >= AC_LIST_NUM)
        return NULL; 

    fn = ac_compare_fn[node_type].fn;
    if (fn == NULL)
       exit(-1);
	    
    pnode = phead->node;
    while (pnode)
    {
        ret = fn(pnode,elem,elem_type); 
        if (ret > 0) /* find it! */
            break;
        else if (ret < 0) /* elem_type error. */
            return NULL;        
        
        pnode = pnode->pnext;        
    }
 
    return pnode;
}       


/*================================================================================
FUNCTION: ac_compare_default_sid          
DESCRIPTION: 
   add a node and sort the default sid list using path length 
ARGUMENTS PASSED:
   list_head_t *phead;
   list_node_t *pnode;
   list_node_t **pret;
RETURN VALUE:
   int:      the position of the new node
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
int ac_sort_default_sid(list_head_t *phead,list_node_t *pnode,list_node_t **pret)
{
    list_node_t *pnode_tmp1,*pnode_tmp2;
    dsid_datum_t *elem_data;
    dsid_datum_t *elem_tmp;
    int position;
    
    *pret = NULL;
	
    pnode_tmp1 = phead->node;
    if (!pnode_tmp1)
        return 0; /* no node exists */
        
    elem_data = (dsid_datum_t *)pnode->elem;
    elem_tmp = (dsid_datum_t *)pnode_tmp1->elem;
    
    if (elem_tmp->path_length < elem_data->path_length)
        return 1; /* insert the new node into the head of the list */
    
    position = 2;
    pnode_tmp2 = pnode_tmp1-> pnext;
    while (pnode_tmp2)
    {
    	elem_tmp = (dsid_datum_t *)pnode_tmp2->elem;
    	if (elem_tmp->path_length < elem_data->path_length)
    	    break;
    	    
        pnode_tmp1 = pnode_tmp2;
        pnode_tmp2 = pnode_tmp2->pnext;
        position ++;
    }
    
    *pret = pnode_tmp1;
        
    return position; 
}       

/*================================================================================
FUNCTION: ac_list_add_node          
DESCRIPTION: 
   add a node into a list
ARGUMENTS PASSED:
   list_head_t *phead;
   list_node_t *pnode;
   fsort *fn;        use to decide how to insert a node according to the element
RETURN VALUE:
   int; 0:success,-1:error 
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
int ac_list_add_node(list_head_t *phead,list_node_t *pnode,fsort fn)                                                           
{
    list_node_t *pnode_tmp;
    int ret = 0;
	
    if ((phead == NULL) || (pnode == NULL))
        return -1;
    
    if (fn == NULL)
    {
        pnode_tmp = ac_list_lastnode(phead);
        if (pnode_tmp == NULL)
            phead->node = pnode;
        else
            pnode_tmp->pnext = pnode;
        pnode->pnext = NULL;
    }else
    {
        ret = fn(phead,pnode,&pnode_tmp);
        
        /* add the node as the first node */
        if ((ret < 2) && (pnode_tmp == NULL))
        {
            pnode->pnext = phead->node;
            phead->node = pnode;
        } 
        else 
        {
            pnode->pnext = pnode_tmp->pnext;
            pnode_tmp->pnext = pnode;
        }
               	    
    }
    phead->elem_num ++;
        
    return 0; 
}       

/*================================================================================
FUNCTION: ac_list_new_node          
DESCRIPTION: 
   initialize a list node
ARGUMENTS PASSED:
   None
RETURN VALUE:
   list_node_t ;
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
list_node_t *ac_list_new_node(void *elem)                                                           
{
    list_node_t *pnode_tmp;
    
    pnode_tmp = (list_node_t *)malloc_sleep(sizeof(list_node_t));
    if (pnode_tmp == NULL)
        return NULL;
        
    memset(pnode_tmp,0,sizeof(list_node_t));
    pnode_tmp->elem = elem;

    return pnode_tmp; 
}       

/*================================================================================
FUNCTION: ac_list_new_head          
DESCRIPTION: 
   initialize a list head
ARGUMENTS PASSED:
   const char *name; list name
RETURN VALUE:
   list_head_t ;
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
list_head_t *ac_list_new_head(const char *name)                                                           
{
    list_head_t *phead_tmp;
    
    phead_tmp = (list_head_t *)malloc_sleep(sizeof(list_head_t));
    if (phead_tmp == NULL)
        return NULL;
        
    memset(phead_tmp,0,sizeof(list_head_t));

    if (name != NULL) 
    {
    	phead_tmp->name = (char *)malloc_sleep(strlen(name)+1);
    	if (phead_tmp->name == NULL)
    	{
    	    free(phead_tmp);
    	    return NULL;
    	}   
        memset(phead_tmp->name,0,strlen(name)+1);
        memcpy(phead_tmp->name,name,strlen(name));
    }
       
    return phead_tmp; 
}       

/*================================================================================
FUNCTION: ac_list_free_head          
DESCRIPTION: 
   release a list
ARGUMENTS PASSED:
   list_head_t *phead
   int type
RETURN VALUE:
   None ;
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
void ac_list_free_head(list_head_t *phead,int type)                                                           
{
    list_node_t *pnode1,*pnode2;
    
    if (phead == NULL)
        return;
    
    pnode1 = pnode2 = phead->node;
    
    switch (type)
    {
    	case AC_LIST_DOMAIN:
    	{
    	    dom_datum_t *pnode_dom;
    	    while(pnode1)
    	    {
    	        pnode_dom = (dom_datum_t *)pnode1->elem;
    	        ac_list_free_head(pnode_dom->apolicy,AC_LIST_AP);
    	        free(pnode_dom->name);
    	        free(pnode_dom);
    	        pnode1 = pnode2->pnext;
    	        free(pnode2);
    	        pnode2 = pnode1;
    	    }
    	    break;   
    	}
    	case AC_LIST_PROCESS:
    	{
    	    proc_datum_t *pnode_proc;
    	    while(pnode1)
    	    {
    	    	pnode_proc = (proc_datum_t *)pnode1->elem;
    	    	ac_list_free_head(pnode_proc->co_policy,AC_LIST_CO);
    	    	ac_list_free_head(pnode_proc->cp_policy,AC_LIST_CP);
    	    	ac_list_free_head(pnode_proc->sp_policy,AC_LIST_SP);
    	    	free(pnode_proc->name);
    	    	free(pnode_proc);
    	    	pnode1 = pnode2->pnext;
    	    	free(pnode2);
    	    	pnode2 = pnode1;
    	    }
    	    break;
    	}
    	case AC_LIST_RESOURCE:
    	{
    	    reso_datum_t *pnode_reso;
    	    while(pnode1)
    	    {
    	    	pnode_reso = (reso_datum_t *)pnode1->elem;
    	    	free(pnode_reso->name);
    	    	pnode1 = pnode2->pnext;
    	    	free(pnode2);
    	    	pnode2 = pnode1;
    	    }
    	    break;
    	}
    	case AC_LIST_CLASS:
    	{
    	    class_datum_t *pnode_class;
    	    while(pnode1)
    	    {
    	    	pnode_class = (class_datum_t *)pnode1->elem;
    	    	free(pnode_class->name);
    	    	pnode1 = pnode2->pnext;
    	    	free(pnode2);
    	    	pnode2 = pnode1;
    	    }
    	    break;
    	}
        case AC_LIST_DEFAULT_SID:
        {
            dsid_datum_t *pnode_dsid;
            while(pnode1)
            {
                pnode_dsid = (dsid_datum_t *)pnode1->elem;
                free(pnode_dsid->full_path);
                pnode1 = pnode2->pnext;
                free(pnode2);
                pnode2 = pnode1;
            }            
        }
    	case AC_LIST_AP:
    	case AC_LIST_CP:
    	case AC_LIST_CO:
        case AC_LIST_SP:
        {
            while(pnode1)
            {
                free(pnode1->elem);
                pnode1 = pnode2->pnext;
                free(pnode2);
                pnode2 = pnode1;
                free(phead);
                phead = NULL;
            }
            break;
        }
    	default:
    	    return;
    }    
}       

/*================================================================================
FUNCTION: ac_policydb_init          
DESCRIPTION: 
   initialize policy structures
ARGUMENTS PASSED:
   policydb_t
RETURN VALUE:
   int; 0:success;
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
int ac_policydb_init(policydb_t *policydb)                                                           
{
    if (policydb == NULL)
        return -EINVAL;
        
    memset(policydb,0,sizeof(policydb_t));
    
    return 0; 
}       

/*================================================================================
FUNCTION: ac_policydb_free          
DESCRIPTION: 
   free policy structures
ARGUMENTS PASSED:
   policydb_t
RETURN VALUE:
   int; 0:success;
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
int ac_policydb_free(policydb_t *p)                                                           
{
    int i;
    defdom_datum_t *ptemp;
    defdom_datum_t *pdefdomain;
     
    if (p == NULL)
        return -EINVAL;
        
    for (i=0;i<(AC_LIST_DEFAULT_SID + 1);i++)
        ac_list_free_head(&p->plist[i],i);

    pdefdomain = p->default_domain_no;
    while (pdefdomain)
    {
        ptemp = pdefdomain;
        pdefdomain = pdefdomain->pnext;
        free(ptemp);
    }        

    return 0; 
}       

/*================================================================================
FUNCTION: ac_malloc_datum          
DESCRIPTION: 
   malloc a datum structure
ARGUMENTS PASSED:
   int elem_type
RETURN VALUE:
   void *
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
void *ac_malloc_datum(int elem_type)                                                           
{
    void *pnode;
    
    if ((elem_type < 0) || (elem_type >= AC_LIST_NUM))
        return NULL;

    switch (elem_type)
    {
        case AC_LIST_DOMAIN:
        {
            dom_datum_t *pnode_dom;
            pnode_dom = (dom_datum_t *)malloc_sleep(sizeof(dom_datum_t));
            pnode = (void *)pnode_dom;
            break;    
        }
        case AC_LIST_PROCESS:
        {
    	    proc_datum_t *pnode_proc;
    	    pnode_proc = (proc_datum_t *)malloc_sleep(sizeof(proc_datum_t));
            pnode = (void *)pnode_proc;
            break;    
        }
        case AC_LIST_RESOURCE:
        {
            reso_datum_t *pnode_reso;
            pnode_reso = (reso_datum_t *)malloc_sleep(sizeof(reso_datum_t));
            pnode = (void *)pnode_reso;
            break;   
        }
        case AC_LIST_CLASS:
        {
      	    class_datum_t *pnode_class;
    	    pnode_class = (class_datum_t *)malloc_sleep(sizeof(class_datum_t));
            pnode = (void *)pnode_class;
            break;   
        }
        case AC_LIST_DEFAULT_SID:
        {
    	    dsid_datum_t *pnode_dsid;
    	    pnode_dsid = (dsid_datum_t *)malloc_sleep(sizeof(dsid_datum_t));
            // memset(pnode_dsid->full_path,0,AC_PATH_MAX_CHAR);
            if (pnode_dsid == NULL)
                return;
            pnode_dsid->full_path = NULL;
            pnode = (void *)pnode_dsid;
            break;   
        }
        case AC_LIST_AP:
        {
    	    ap_datum_t *pnode_ap;
    	    pnode_ap = (ap_datum_t *)malloc_sleep(sizeof(ap_datum_t));
            pnode = (void *)pnode_ap;
            break;   
        }
        case AC_LIST_CP:
        {
    	    cp_datum_t *pnode_cp;
    	    pnode_cp = (cp_datum_t *)malloc_sleep(sizeof(cp_datum_t));
            pnode = (void *)pnode_cp;
            break;   
        }
        case AC_LIST_CO:
        {
    	    co_datum_t *pnode_co;
    	    pnode_co = (co_datum_t *)malloc_sleep(sizeof(co_datum_t));
            pnode = (void *)pnode_co;
            break;   
        }
        case AC_LIST_SP:
        {
            sp_datum_t *pnode_sp;
    	    pnode_sp = (sp_datum_t *)malloc_sleep(sizeof(sp_datum_t));
            pnode = (void *)pnode_sp;
            break; 
        }
        default:
            pnode = NULL;
            break;
    }
    
    return pnode; 
}

/*================================================================================
FUNCTION: ac_get_defpt          
DESCRIPTION: 
   find the default process type of this system
ARGUMENTS PASSED:
   None
RETURN VALUE:
   list_node_t  *
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
list_node_t *ac_get_defpt(void)
{
    list_node_t *pnode;
    proc_datum_t *proc_node;
    list_head_t *phead = &policydb.process_elem;

    pnode = phead->node;
    while (pnode)
    {
    	proc_node = (proc_datum_t *)pnode->elem;
    	if (proc_node->spec_attr & AC_PROCESS_ISDEFAULT)
    	    break;
    	pnode = pnode->pnext;
    }
    
    return pnode;
}

/*================================================================================
FUNCTION: ac_object_getdefsid          
DESCRIPTION: 
   assign a SID for a inode(file/dir/fs)or a process
ARGUMENTS PASSED:
   char *path
   int  type    :process or file (AC_DEFAULT_TYPE_PROCESS or AC_DEFAULT_TYPE_OBJECT)
RETURN VALUE:
   int  sid
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
int ac_object_getdefsid(char *path,int type)
{
    list_node_t *pnode;
    dsid_datum_t *dsid_node;
    int defsid = AC_DEFAULT_UNLABELED;
    int len;
   
    if (((type != AC_DEFAULT_TYPE_PROCESS) && (type != AC_DEFAULT_TYPE_OBJECT)) ||        
        (path == NULL))
        return -EINVAL; /* parameter error */

    if ((!ac_policy_init) && (type == AC_DEFAULT_TYPE_PROCESS))
    	return AC_CLASS_KERNEL;

    if ((!ac_policy_init) && (type == AC_DEFAULT_TYPE_OBJECT))
	return AC_DEFAULT_UNLABELED;

    dsid_node = (dsid_datum_t *)ac_malloc_datum(AC_LIST_DEFAULT_SID);
    if (dsid_node == NULL)
        return -ENOMEM; /* no memory */

    len = strlen(path);

    dsid_node->full_path = (char *)malloc_sleep(len + 1);
    if (dsid_node->full_path == NULL)
    {
        free(dsid_node);
        return -ENOMEM;
    }

    dsid_node->sid_type = type;

    memset(dsid_node->full_path,0,len + 1);
    memcpy(dsid_node->full_path,path,len);
    
    pnode = ac_find_node(&policydb.default_sid,AC_LIST_DEFAULT_SID,
                (void *)dsid_node,AC_FIND_BY_ELEMENT);
   
    free(dsid_node->full_path); 
    free(dsid_node);
    
    if (pnode != NULL)
    {
    	dsid_node = (dsid_datum_t *)pnode->elem;
    	defsid = dsid_node->def_type_no;
    }

    return defsid;
}

/*================================================================================
FUNCTION: ac_socket_getsid          
DESCRIPTION: 
   assign a SID for a socket
ARGUMENTS PASSED:
   int  proc_no    :process type;
   int  class_no   :class type
RETURN VALUE:
   int  sid
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
int ac_socket_getsid(int proc_no,int class_no)
{
    list_node_t *pnode;
    proc_datum_t *proc_node;
    co_datum_t *co_node;

    if (!ac_policy_init)
	return AC_DEFAULT_UNLABELED;

    printf("w20598: ac_socket_getsid. proc_no = %d,class_no = %d\n",proc_no,class_no);

    pnode = ac_find_node(&policydb.process_elem,AC_LIST_PROCESS,(void *)&proc_no,AC_FIND_BY_NO); 
    if (pnode == NULL)
        return  -EPERM;  /* process doesn't exist */
    
    proc_node = (proc_datum_t *)pnode->elem;
    co_node = (co_datum_t *)ac_malloc_datum(AC_LIST_CO);
    if (co_node == NULL)
        return -ENOMEM;   /* no memory */
    
    co_node->pare_type_no = proc_no;
    co_node->class_no = class_no;    
    pnode = ac_find_node(proc_node->co_policy,AC_LIST_CO,
                (void *)co_node,AC_FIND_BY_ELEMENT);
    
    free(co_node);
    
    if (pnode == NULL)
    {
	printf("use default label %d!\n",AC_DEFAULT_UNLABELED);
        return AC_DEFAULT_UNLABELED;
    }
    else
    {
    	co_node = (co_datum_t *)pnode->elem;
	printf("use defined label %d!\n",co_node->reso_type_no);
    	return co_node->reso_type_no;
    }   
}

/*================================================================================
FUNCTION: ac_process_getsid          
DESCRIPTION: 
   assign a SID for a process 
ARGUMENTS PASSED:
   int  parent_no    :process type;
   int  file_type_no :executable file type
RETURN VALUE:
   int  sid
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
int ac_process_getsid(int parent_no,int file_type_no)
{
    list_node_t *pnode;
    proc_datum_t *proc_node;
    cp_datum_t *cp_node;
   
    if (!ac_policy_init)
        return parent_no;

    printf("w20598: ac_process_getsid. parent_no = %d, file_type_no = %d\n",parent_no,file_type_no);

    pnode = ac_find_node(&policydb.process_elem,AC_LIST_PROCESS,(void *)&parent_no,AC_FIND_BY_NO); 
    if (pnode == NULL)
        return  -EPERM;  /* process doesn't exist */
    proc_node = (proc_datum_t *)pnode->elem;
    
    cp_node = (cp_datum_t *)ac_malloc_datum(AC_LIST_CP);
    if (cp_node == NULL)
        return -ENOMEM;   /* no memory */
    
    cp_node->pare_type_no = parent_no;
    cp_node->file_type_no = file_type_no;       
    
    pnode = ac_find_node(proc_node->cp_policy,AC_LIST_CP,
                (void *)cp_node,AC_FIND_BY_ELEMENT);
    
    free(cp_node);
    
    if (pnode == NULL)
#ifdef AC_SECURITY_HIGHLEVEL
        return -EPERM;            /* dont allow to execv this process */
#else
    {
    	pnode = ac_get_defpt();
    	if (pnode == NULL)
    	    return -EPERM;
    	
    	proc_node = (proc_datum_t *)pnode->elem;
	printf("cant find policy.use default = %d\n",proc_node->pno);
        return proc_node->pno;
    }
#endif
    else
    {
    	cp_node = (cp_datum_t *)pnode->elem;
	printf("transfer to %d\n",cp_node->child_type_no);
    	return cp_node->child_type_no;
    }   
}

/*================================================================================
  FUNCTION: ac_proc_to_dom          
  DESCRIPTION: 
      find the domain number of the process.
  ARGUMENTS PASSED:
      int  proc_no 
  RETURN VALUE:
      None
  PRE-CONDITIONS:
      None
  POST-CONDITIONS: 
      None
 IMPORTANT NOTES:
      None
 ================================================================================*/
int ac_proc_to_dom(int proc_no)
{
    list_node_t *pnode;
    proc_datum_t *proc_node;

    pnode = ac_find_node(&policydb.process_elem,AC_LIST_PROCESS,(void *)&proc_no,AC_FIND_BY_NO);
    if (pnode == NULL)
        return  -EPERM;  /* process doesn't exist */

    proc_node = (proc_datum_t *)pnode->elem;

    return proc_node->dom_type_no;
}

/*================================================================================
  FUNCTION: ac_dom_default_proc          
  DESCRIPTION: 
      find the default process defined of a domain.
  ARGUMENTS PASSED:
      int  dom_no 
  RETURN VALUE:
      None
  PRE-CONDITIONS:
      None
  POST-CONDITIONS: 
      None
 IMPORTANT NOTES:
      None
 ================================================================================*/
int ac_dom_default_proc(int dom_no)
{
    list_node_t *pnode;
    dom_datum_t *dom_node;

    pnode = ac_find_node(&policydb.domain_elem,AC_LIST_DOMAIN,(void *)&dom_no,AC_FIND_BY_NO);
    if (pnode == NULL)
        return  -EPERM;  /* process doesn't exist */

    dom_node = (dom_datum_t *)pnode->elem;

    return dom_node->def_proc_no;
}

/*================================================================================
  FUNCTION: ac_dom_exist          
  DESCRIPTION: 
      Is a domain type is defined in policy file.
  ARGUMENTS PASSED:
      int  dom_no 
  RETURN VALUE:
      int;  1: exist; 0: doesnt exist
  PRE-CONDITIONS:
      None
  POST-CONDITIONS: 
      None
 IMPORTANT NOTES:
      None
 ================================================================================*/
int ac_dom_exist(int dom_no)
{
    void *pnode;

    pnode =(void *)ac_find_node(&policydb.domain_elem,AC_LIST_DOMAIN,(void *)&dom_no,AC_FIND_BY_NO);

    return ((pnode == NULL)? 0:1);
}

/*================================================================================
  FUNCTION: ac_permission_isdefined          
  DESCRIPTION: 
     check whether class exist and the permission is defined 
  ARGUMENTS PASSED:
     int  class_no       :class no
     int  perm           :permission
  RETURN VALUE:
     int;  1: defined; 0: Not defined
  PRE-CONDITIONS:
     None
  POST-CONDITIONS: 
     None
  IMPORTANT NOTES:
     None
 ================================================================================*/
int ac_permission_isdefined(int class_no,int perm)
{
    list_node_t *pnode;
    class_datum_t *class_node;
    
    pnode = ac_find_node(&policydb.class_elem,AC_LIST_CLASS,(void *)&class_no,AC_FIND_BY_NO);
    if (pnode == NULL)
        return 0; /* Not defined, GRANT it */
  
    class_node = (class_datum_t *)pnode->elem;
    if (~class_node->all_pri_bits & perm)
        return 0; /* permission doesnt defined. GRANT */

    return 1;
}

/*================================================================================
FUNCTION: ac_task_has_perm          
DESCRIPTION: 
   check whether task has a permission.
ARGUMENTS PASSED:
   int  proc_no
   int  class_no      :especial policy class type.
   int  perm
RETURN VALUE:
   AC_CHECK_GRANT/AC_CHECK_DENY
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
int ac_task_has_perm(int proc_no,int class_no,int perm)
{
    list_node_t *pnode;
    proc_datum_t *proc_node;
    sp_datum_t  *sp_node;
       
    if (!ac_policy_init)
        return AC_CHECK_GRANT;

    if (!ac_policy_enable)
        return AC_CHECK_GRANT;
        
    if (!ac_permission_isdefined(class_no,perm))
        return AC_CHECK_GRANT; /* permission doesnt defined. GRANT */
    
    printf("w20598: ac_task_has_perm. proc_no = %d, class_no = %d, perm = 0x%x\n",proc_no,class_no,perm);
    pnode = ac_find_node(&policydb.process_elem,AC_LIST_PROCESS,(void *)&proc_no,AC_FIND_BY_NO); 
    if (pnode == NULL)
        return  -EPERM;  /* process doesn't exist */
    
    proc_node = (proc_datum_t *)pnode->elem;
    pnode = ac_find_node(proc_node->sp_policy,AC_LIST_SP,(void *)&class_no,AC_FIND_BY_NO);
    if (pnode == NULL)
        return AC_CHECK_DENY;
        
    sp_node = (sp_datum_t *)pnode->elem;
    printf("perm is %d\n",!(sp_node->pri_bitmap & perm));
    return ((sp_node->pri_bitmap & perm)?AC_CHECK_GRANT:AC_CHECK_DENY);
}

int ac_is_default_domain(int dom_no)
{
   defdom_datum_t *pdefdomain;

   pdefdomain = (&policydb)->default_domain_no;

   while (pdefdomain)
   {
       if (pdefdomain->did == dom_no)
           return 1;
       pdefdomain = pdefdomain->pnext;
   } 

   return 0;
}

/*================================================================================
FUNCTION: ac_check_ac_perm          
DESCRIPTION: 
   make an access decision for an access request.
ARGUMENTS PASSED:
   int  proc_no      :process type
   int  reso_no      :resource type
   int  reso_type    :resource class type
   int  oper_type    :operation type
RETURN VALUE:
   int     1:yes;0:no
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
int ac_check_ac_perm(int proc_no,int reso_no,int reso_type,int oper_type)
{
    list_node_t *pnode;
    dom_datum_t *dom_node;
    ap_datum_t *ap_node;
    int dom_no;
    
    if (!ac_policy_init)
	return AC_CHECK_GRANT;
   
    if (!ac_policy_enable)
	return AC_CHECK_GRANT;
    
    printf("w20598: ac_check_ac_perm. proc_no = %d, reso_no = %d,reso_type = %d,oper_type = 0x%x\n",
		    proc_no,reso_no,reso_type,oper_type);
    
    /* un-protected resource */
    if ((reso_no == AC_DEFAULT_UNLABELED) && policydb.default_rec_rule)
    {
	printf("w20598: undefined resource class bypass\n");
        return AC_CHECK_GRANT;
    }
   
    if (!ac_permission_isdefined(reso_type,oper_type))
        return AC_CHECK_GRANT; /* permission doesnt defined. GRANT */
   
    /* find domain no */
    dom_no = ac_proc_to_dom(proc_no);
    if (dom_no < 0)
	return -EPERM;
    
    pnode = ac_find_node(&policydb.domain_elem,AC_LIST_DOMAIN,(void *)&dom_no,AC_FIND_BY_NO); 
    if (pnode == NULL)
        return  -EPERM;  /* process doesn't exist */
    
    dom_node = (dom_datum_t *)pnode->elem;
    
    ap_node = (ap_datum_t *)ac_malloc_datum(AC_LIST_AP);
    if (ap_node == NULL)
        return -ENOMEM;   /* no memory */
    
    ap_node->reso_type_no = reso_no;
    ap_node->class_no = reso_type;       
    
    pnode = ac_find_node(dom_node->apolicy,AC_LIST_AP,
                (void *)ap_node,AC_FIND_BY_ELEMENT);
    
    free(ap_node);
    
    if (pnode == NULL)
    {
        /* default domain bypass */
          if (ac_is_default_domain(dom_no))
              return AC_CHECK_GRANT;

          return AC_CHECK_DENY;            /* DENY */
    }
    else
    {
    	ap_node = (ap_datum_t *)pnode->elem;
        if ((ap_node->priv_bitmap & oper_type) == oper_type)
            return ((ap_node->action)?AC_CHECK_GRANT:AC_CHECK_DENY);
        else
            return ((ap_node->action)?AC_CHECK_DENY:AC_CHECK_GRANT);
    }   
}

/*================================================================================
FUNCTION: ac_policydb_read          
DESCRIPTION: 
   load the binary policy file into RAM
ARGUMENTS PASSED:
   policydb_t *pdb;
   FILE       *fp;
RETURN VALUE:
   int     0:success; <0: error
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
int ac_policydb_read(policydb_t *pdb,FILE *fp)
{
    int i,ret,items;
    unsigned long buf[4],len;
    char *policy_str;

    printf("w20598: init policydb structure!\n");    
    if ((ret = ac_policydb_init(pdb)) != 0 )
        return ret;
    
    printf("w20598: read magic and string!\n");
    /* read policy file head */
    items = fread(buf,sizeof(unsigned long),2,fp);
    if (items != 2)
        goto bad;
        
    for (i=0;i<2;i++)
        buf[i] = le32_to_cpu(buf[i]);
    printf("w20598: magic = 0x%x\n",buf[0]);
    printf("w20598: string len = %d\n",buf[1]);
    if (buf[0] != AC_POLICY_MAGIC)
    	goto bad;
    
    len = buf[1];
    if (len != strlen(AC_POLICY_STRING))
    	goto bad;
    
    policy_str = (char *)malloc_sleep(len + 1);
    if (policy_str == NULL)
        goto bad;
        
    items = fread(policy_str,1,len,fp);
    if (items != len)
    {
    	free(policy_str);
    	goto bad;
    }
    
    policy_str[len] = 0;
    if (strcmp(policy_str,AC_POLICY_STRING))
    	goto bad;
    
    printf("w20598: string = %s\n",AC_POLICY_STRING);  
    free(policy_str);
    policy_str = NULL;
    
    items = fread(buf,sizeof(unsigned long),1,fp);
    if (items != 1)
        goto bad;
    
    buf[0] = le32_to_cpu(buf[0]);
    printf("w20598: version 0x%x \n",buf[0]);
    pdb->policy_ver = buf[0];
  //  if (buf[0] != AC_POLICY_VERSION)
    //	goto bad;

    printf("w20598: read types\n"); 
    /* types */
    if (AC_POLICY_READ(domtype)(pdb,fp))
        goto bad;
    
    printf("w20598: read classes\n");
    /* classes */
    if (AC_POLICY_READ(class)(pdb,fp))
        goto bad;
    
    printf("w20598: read default processes\n");
    /* default processes */
    if (AC_POLICY_READ(dsid)(pdb,fp,AC_DEFAULT_TYPE_PROCESS))
        goto bad;
    
    printf("w20598: read create policies\n");
    /* create policies */
    if (AC_POLICY_READ(cp)(pdb,fp))
        goto bad;
    
    printf("w20598: read access policies\n");
    /* access policies */
    if (AC_POLICY_READ(ap)(pdb,fp))
        goto bad;
    printf("w20598: policy read successful\n");

    ac_policydb_printk(pdb);
    ac_policy_init = 1;
    return 0;

bad:
    ac_policydb_free(pdb);
    return -1;    
}

/*================================================================================
FUNCTION: ac_load_policy          
DESCRIPTION: 
   open/read the policy file. 
ARGUMENTS PASSED:
   None
RETURN VALUE:
   int     0:success; <0: error
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
int ac_load_policy(void)
{
    int ret;
    FILE *policy_fp;

    printf("w20598: open policy file %s\n",AC_POLICY_PATH); 
    policy_fp = fopen(AC_POLICY_PATH, "r"); 
    if (!policy_fp) 
        return -EINVAL;

    printf("w20598: before read policy file\n");
    ret = ac_policydb_read(&policydb,policy_fp);
    fclose(policy_fp);
    
    return ret;
}

/*================================================================================
FUNCTION: ac_policy_read_domtype          
DESCRIPTION: 
   read type defines in policy file; 
ARGUMENTS PASSED:
   policydb_t *pdb;
   FILE       *fp
RETURN VALUE:
   int     0:success; <0: error
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
static int ac_policy_read_domtype(policydb_t *pdb,FILE *fp)
{
    int i,j,items;
    unsigned long buf[AC_PATH_MAX_CHAR - 1],num,len;
    dom_datum_t *pnode_dom = NULL;
    list_node_t *pnode = NULL;
   
    printf("w20598: ac_policy_read_domtype\n"); 
    items = fread(buf,sizeof(unsigned long),1,fp);
    if (items != 1)
        return -1;
    buf[0]=le32_to_cpu(buf[0]);
    num = buf[0];
    
    printf("w20598: domain type number = %d\n",buf[0]);

    /* domain type */
    for (i = 0; i < num; i++)
    {
	printf("w20598: i=%d",i);
        items = fread(buf,sizeof(unsigned long),2,fp);
        if (items != 2)
            goto bad;
        for (j=0;j<2;j++)
            buf[j] = le32_to_cpu(buf[j]);
        len = buf[1];
   
        printf("  No = %d len = %d",buf[0],buf[1]);
        pnode_dom = (dom_datum_t *)ac_malloc_datum(AC_LIST_DOMAIN);
    	if (pnode_dom == NULL)
    	    goto bad;

        pnode_dom->dno = buf[0];
        pnode_dom->name = (char *)malloc_sleep(len+1);
        if (pnode_dom->name == NULL)
            goto bad;
	
        items = fread(pnode_dom->name,1,len,fp);
        if (items != len)
            goto bad;
            
        pnode_dom->name[len] = 0;
	printf("  name = %s",pnode_dom->name); 
        
        items = fread(buf,sizeof(unsigned long),1,fp);
        if (items != 1)
            goto bad;
            
        buf[0]=le32_to_cpu(buf[0]);
	printf(" default proc = %d\n",buf[0]);
        pnode_dom->def_proc_no = buf[0];

        pnode_dom->apolicy = (list_head_t *)ac_list_new_head("ap");
        if (pnode_dom->apolicy == NULL)
            goto bad;
            
        pnode = ac_list_new_node((void *)pnode_dom);
        if (pnode == NULL)
            goto bad;
            
        if (ac_list_add_node(&(pdb->domain_elem),pnode,NULL) < 0)
            goto bad;
    }
    return ac_policy_read_proctype(pdb,fp);
    
bad:
    if (pnode) free(pnode);
    if (pnode_dom)
    {
    	if (pnode_dom->apolicy)
    	{
    	    list_head_t *phead_tmp = (list_head_t *)pnode_dom->apolicy;
    	    if (phead_tmp->name) free(phead_tmp->name);
    	    free(pnode_dom->apolicy);    
    	}
        if (pnode_dom->name) free(pnode_dom->name);
        free(pnode_dom);
    }
    return -1;
}

/*================================================================================
FUNCTION: ac_policy_read_proctype          
DESCRIPTION: 
   read process defines in policy file; 
ARGUMENTS PASSED:
   policydb_t *pdb;
   FILE       *fp
RETURN VALUE:
   int     0:success; <0: error
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
static int ac_policy_read_proctype(policydb_t *pdb,FILE *fp)
{
    int i,j,items;
    unsigned long buf[AC_PATH_MAX_CHAR - 1],num,len;
    proc_datum_t *pnode_proc = NULL;
    list_node_t *pnode = NULL;
    
    items = fread(buf,sizeof(unsigned long),1,fp);
    if (items != 1)
        return -1;
    buf[0]=le32_to_cpu(buf[0]);
    num = buf[0];
   
    printf("w20598: proctype number = %d\n",buf[0]); 
    /* process type */
    for (j = 0; j < num; j++)
    {
        items = fread(buf,sizeof(unsigned long),2,fp);
        if (items != 2)
            goto bad;
        for (i=0;i<2;i++)
            buf[i] = le32_to_cpu(buf[i]);
        len = buf[1];
    
        pnode_proc = (proc_datum_t *)ac_malloc_datum(AC_LIST_PROCESS);
    	if (pnode_proc == NULL)
    	    goto bad;
        
	printf(" NO = %d",buf[0]);
        pnode_proc->pno = buf[0];
        pnode_proc->name = (char *)malloc_sleep(len+1);
        if (pnode_proc->name == NULL)
            goto bad;
            
        items = fread(pnode_proc->name,1,len,fp);
        if (items != len)
            goto bad;
            
        pnode_proc->name[len] = 0;
        
	printf("  name = %s",pnode_proc->name);
        items = fread(buf,sizeof(unsigned long),1,fp);
        if (items != 1)
            goto bad;
        
        pnode_proc->spec_attr = le32_to_cpu(buf[0]);
        printf("  spec_attr = %d",pnode_proc->spec_attr);
        	
        pnode_proc->sp_policy = (list_head_t *)ac_list_new_head("sp"); 
        pnode_proc->cp_policy = (list_head_t *)ac_list_new_head("cp");
        pnode_proc->co_policy = (list_head_t *)ac_list_new_head("co");
        if ((pnode_proc->sp_policy == NULL) ||
            (pnode_proc->cp_policy == NULL) ||
            (pnode_proc->co_policy == NULL))
            goto bad;
       
	/* read especial policy */
        if ( AC_POLICY_READ(sp)(pnode_proc->sp_policy,fp) < 0)
            goto bad;
    
         items = fread(buf,sizeof(unsigned long),1,fp);
         if (items != 1)
	            goto bad;
        
        pnode_proc->dom_type_no = le32_to_cpu(buf[0]);
	printf(" dom_type = %d\n",pnode_proc->dom_type_no);
	
        pnode = ac_list_new_node((void *)pnode_proc);
        if (pnode == NULL)
            goto bad;
            
        if (ac_list_add_node(&(pdb->process_elem),pnode,NULL) < 0)
            goto bad;
    }
    return ac_policy_read_resotype(pdb,fp);
    
bad:
    if (pnode) free(pnode);    
    if (pnode_proc)
    {
	list_head_t *phead_tmp;
    	if (pnode_proc->name) free(pnode_proc->name);
    	if (pnode_proc->sp_policy) 
	{
	    phead_tmp = (list_head_t *)pnode_proc->sp_policy;
	    if (phead_tmp->name)
	        free(phead_tmp->name);
	    free(pnode_proc->sp_policy);
	}
    	if (pnode_proc->cp_policy) 
	{
	     phead_tmp = (list_head_t *)pnode_proc->cp_policy;
	     if (phead_tmp->name)
	         free(phead_tmp->name);
	     free(pnode_proc->cp_policy);
	 }
    	if (pnode_proc->co_policy) 
	{
	     phead_tmp = (list_head_t *)pnode_proc->co_policy;
             if (phead_tmp->name)
                 free(phead_tmp->name);
             free(pnode_proc->co_policy);
	}
    	free(pnode_proc);
    }
    return -1;
}


/*================================================================================
FUNCTION: ac_policy_read_resotype          
DESCRIPTION: 
   read resource defines in policy file; 
ARGUMENTS PASSED:
   policydb_t *pdb;
   FILE       *fp
RETURN VALUE:
   int     0:success; <0: error
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
static int ac_policy_read_resotype(policydb_t *pdb,FILE *fp)
{
    int i,j,items;
    unsigned long buf[AC_PATH_MAX_CHAR - 1],num,len;
    reso_datum_t *pnode_reso = NULL;
    list_node_t *pnode = NULL;
    
    items = fread(buf,sizeof(unsigned long),1,fp);
    if (items != 1)
        return -1;
    buf[0]=le32_to_cpu(buf[0]);
    num = buf[0];
   
    printf("w20598: resource type number = %d\n",buf[0]); 
    /* domain type */
    for (i = 0; i < num; i++)
    {
        items = fread(buf,sizeof(unsigned long),2,fp);
        if (items != 2)
            goto bad;
        for (j=0;j<2;j++)
            buf[j] = le32_to_cpu(buf[j]);
        len = buf[1];
    
        pnode_reso = (reso_datum_t *)ac_malloc_datum(AC_LIST_RESOURCE);
    	if (pnode_reso == NULL)
    	    goto bad;

        pnode_reso->rno = buf[0];
        pnode_reso->name = (char *)malloc_sleep(len+1);
        if (pnode_reso->name == NULL)
            goto bad;
            
        items = fread(pnode_reso->name,1,len,fp);
        if (items != len)
            goto bad;
            
        pnode_reso->name[len] = 0;
       
        printf(" NO = %d, Name = %s\n",pnode_reso->rno,pnode_reso->name);	
        pnode = ac_list_new_node((void *)pnode_reso);
        if (pnode == NULL)
            goto bad;
            
        if (ac_list_add_node(&(pdb->resource_elem),pnode,NULL) < 0)
            goto bad;
    }
    return 0;
    
bad:
    if (pnode) free(pnode);
    if (pnode_reso)
    {
        if (pnode_reso->name) free(pnode_reso->name);
        free(pnode_reso);
    }
    return -1;
}

/*================================================================================
FUNCTION: ac_policy_read_class          
DESCRIPTION: 
   read classes in policy file; 
ARGUMENTS PASSED:
   policydb_t *pdb;
   FILE       *fp
RETURN VALUE:
   int     0:success; <0: error
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
static int ac_policy_read_class(policydb_t *pdb,FILE *fp)
{
    int i,j,items;
    unsigned long buf[AC_PATH_MAX_CHAR - 1],num,len;
    class_datum_t *pnode_class = NULL;
    list_node_t *pnode = NULL;
    
    items = fread(buf,sizeof(unsigned long),1,fp);
    if (items != 1)
        return -1;
    buf[0]=le32_to_cpu(buf[0]);
    num = buf[0];
    
    printf("w20598: class number = %d\n",buf[0]);
    /* domain type */
    for (i = 0; i < num; i++)
    {
        items = fread(buf,sizeof(unsigned long),2,fp);
        if (items != 2)
            goto bad;
        for (j=0;j<2;j++)
            buf[j] = le32_to_cpu(buf[j]);
        len = buf[1];
   
        printf("No = %d",buf[0]);	
        pnode_class = (class_datum_t *)ac_malloc_datum(AC_LIST_CLASS);
    	if (pnode_class == NULL)
    	    goto bad;

        pnode_class->class_no = buf[0];
        pnode_class->name = (char *)malloc_sleep(len+1);
        if (pnode_class->name == NULL)
            goto bad;
            
        items = fread(pnode_class->name,1,len,fp);
        if (items != len)
            goto bad;
            
        pnode_class->name[len] = 0;
        
	printf("  Name = %s",pnode_class->name);
        items = fread(buf,sizeof(unsigned long),1,fp);
        if (items != 1)
            goto bad;
        buf[0] = le32_to_cpu(buf[0]);
        pnode_class->all_pri_bits = buf[0];   
        
        printf("  all_pri_bits = 0x%x\n",buf[0]);	
        pnode = ac_list_new_node((void *)pnode_class);
        if (pnode == NULL)
            goto bad;
            
        if (ac_list_add_node(&(pdb->class_elem),pnode,NULL) < 0)
            goto bad;
    }
    return 0;
    
bad:
    if (pnode) free(pnode);
    if (pnode_class)
    {
        if (pnode_class->name) free(pnode_class->name);
        free(pnode_class);
    }
    return -1;
}

/*================================================================================
FUNCTION: ac_policy_read_dsid         
DESCRIPTION: 
   read default process defines in policy file; 
ARGUMENTS PASSED:
   policydb_t *pdb;
   FILE       *fp
   int        type  process/object
RETURN VALUE:
   int     0:success; <0: error
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
static int ac_policy_read_dsid(policydb_t *pdb,FILE *fp,int type)
{
    int i,j,items;
    unsigned long buf[AC_PATH_MAX_CHAR - 1],num,len;
    dsid_datum_t *pnode_dsid = NULL;
    list_node_t *pnode = NULL;
    
    items = fread(buf,sizeof(unsigned long),1,fp);
    if (items != 1)
        return -1;
    buf[0]=le32_to_cpu(buf[0]);
    num = buf[0];
    
    printf("w20598: default sid(type = %d) number = %d\n",type,buf[0]);
    /* domain type */
    for (i = 0; i < num; i++)
    {
        items = fread(buf,sizeof(unsigned long),2,fp);
        if (items != 2)
            goto bad;
        for (j=0;j<2;j++)
            buf[j] = le32_to_cpu(buf[j]);
        len = buf[1];
        //len = (buf[1] > (AC_PATH_MAX_CHAR - 1))?(AC_PATH_MAX_CHAR - 1):buf[1];
    
        pnode_dsid = (dsid_datum_t *)ac_malloc_datum(AC_LIST_DEFAULT_SID);
    	if (pnode_dsid == NULL)
    	    goto bad;

        pnode_dsid->def_type_no = buf[0];
        pnode_dsid->sid_type = type;
        pnode_dsid->path_length = len;

        pnode_dsid->full_path = (char *)malloc_sleep(len + 1);	            
        if (pnode_dsid->full_path == NULL)
            goto bad;

        memset(pnode_dsid->full_path,0,len + 1);
        items = fread(pnode_dsid->full_path,1,len,fp);
        if (items != len)
            goto bad;
            
        printf("NO = %d, path length = %d, full path = %s\n",buf[0],buf[1],pnode_dsid->full_path);	
        pnode = ac_list_new_node((void *)pnode_dsid);
        if (pnode == NULL)
            goto bad;
            
        if (ac_list_add_node(&(pdb->default_sid),pnode,&ac_sort_default_sid) < 0)
            goto bad;
    }
    if (type == AC_DEFAULT_TYPE_OBJECT)
	return 0;
    else
	return ac_policy_read_dsid(pdb,fp,AC_DEFAULT_TYPE_OBJECT);
    
bad:
    if (pnode) free(pnode);
    if (pnode_dsid)
    {
        if (pnode_dsid->full_path)
            free(pnode_dsid->full_path);
        free(pnode_dsid);
    }
    return -1;
}

/*================================================================================
FUNCTION: ac_policy_read_cp          
DESCRIPTION: 
   read create policies in policy file; 
ARGUMENTS PASSED:
   policydb_t *pdb
   FILE       *fp
RETURN VALUE:
   int     0:success; <0: error
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
static int ac_policy_read_cp(policydb_t *pdb,FILE *fp)
{
    int i,j,items,proc_no;
    unsigned long buf[AC_PATH_MAX_CHAR - 1],num;
    cp_datum_t *pnode_cp = NULL;
    proc_datum_t *pnode_proc = NULL;
    list_node_t *pnode = NULL;
    
    items = fread(buf,sizeof(unsigned long),1,fp);
    if (items != 1)
        return -1;
    buf[0]=le32_to_cpu(buf[0]);
    num = buf[0];
    
    printf("w20598: create process policy number = %d\n",buf[0]);
    /* create process policies */
    for (i = 0; i < num; i++)
    {
    	items = fread(buf,sizeof(unsigned long),3,fp);
        if (items != 3)
            goto bad;
        for (j=0;j<3;j++)
            buf[j] = le32_to_cpu(buf[j]);

        proc_no = buf[0];
        pnode = ac_find_node(&pdb->process_elem,AC_LIST_PROCESS,(void *)&proc_no,AC_FIND_BY_NO);
        if (pnode == NULL)
            goto bad;

        pnode_proc = (proc_datum_t *)pnode->elem;	
        pnode_cp = (cp_datum_t *)ac_malloc_datum(AC_LIST_CP);
        if (pnode_cp == NULL)
            goto bad;
        
        pnode_cp->pare_type_no = buf[0];
        pnode_cp->file_type_no = buf[1];
        pnode_cp->child_type_no = buf[2];
        
	printf("parent(%d)--> file type(%d) ==== child(%d)\n",buf[0],buf[1],buf[2]);
        pnode = ac_list_new_node((void *)pnode_cp);
        if (pnode == NULL)
            goto bad;

	printf("w20598: insert into proctype(%d) list\n",pnode_proc->pno);
	if (ac_list_add_node((list_head_t *)pnode_proc->cp_policy,pnode,NULL) < 0)
            goto bad;
    }
    return ac_policy_read_co(pdb,fp);
    
bad:
    if (pnode) free(pnode);
    if (pnode_cp) free(pnode_cp);
    return -1;
}

/*================================================================================
FUNCTION: ac_policy_read_co          
DESCRIPTION: 
   read create objects policies in policy file; 
ARGUMENTS PASSED:
   policydb_t *pdb
   FILE       *fp
RETURN VALUE:
   int     0:success; <0: error
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
static int ac_policy_read_co(policydb_t *pdb,FILE *fp)
{
    int i,j,items,proc_no;
    unsigned long buf[AC_PATH_MAX_CHAR - 1],num;
    co_datum_t *pnode_co = NULL;
    proc_datum_t *pnode_proc = NULL;
    list_node_t *pnode = NULL;
    
    items = fread(buf,sizeof(unsigned long),1,fp);
    if (items != 1)
        return -1;
    buf[0]=le32_to_cpu(buf[0]);
    num = buf[0];
   
    printf("w20598: create object number = %d\n",buf[0]);
    /* create object  policies */
    for (i = 0; i < num; i++)
    {
    	items = fread(buf,sizeof(unsigned long),3,fp);
        if (items != 3)
            goto bad;
        for (j=0;j<3;j++)
            buf[j] = le32_to_cpu(buf[j]);
        
        proc_no = buf[0];
        pnode = ac_find_node(&pdb->process_elem,AC_LIST_PROCESS,(void *)&proc_no,AC_FIND_BY_NO);
        if (pnode == NULL)
            goto bad;
       
        pnode_proc = (proc_datum_t *)pnode->elem;	
        pnode_co = (co_datum_t *)ac_malloc_datum(AC_LIST_CO);
        if (pnode_co == NULL)
            goto bad;
        
        pnode_co->pare_type_no = buf[0];
        pnode_co->class_no = buf[1];
        pnode_co->reso_type_no = buf[2];
        
	printf("parent(%d)-->class type(%d) === resource type(%d)\n",buf[0],buf[1],buf[2]);
        pnode = ac_list_new_node((void *)pnode_co);
        if (pnode == NULL)
            goto bad;
        
        printf("w20598: insert into proctype(%d) list\n",pnode_proc->pno);	
        if (ac_list_add_node((list_head_t *)pnode_proc->co_policy,pnode,NULL) < 0)
            goto bad;
    }
    return 0;
    
bad:
    if (pnode) free(pnode);
    if (pnode_co) free(pnode_co);
    return -1;
}

/*================================================================================
FUNCTION: ac_policy_read_ap          
DESCRIPTION: 
   read access policies in policy file; 
ARGUMENTS PASSED:
   policydb_t *pdb
   FILE       *fp
RETURN VALUE:
   int     0:success; <0: error
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
static int ac_policy_read_ap(policydb_t *pdb,FILE *fp) 
{
    int i,j,items,dom_no;
    unsigned long buf[AC_PATH_MAX_CHAR - 1],num;
    ap_datum_t *pnode_ap = NULL;
    dom_datum_t *pnode_dom = NULL;
    list_node_t *pnode = NULL;
    defdom_datum_t *defdomain,*ptemp;
    
    items = fread(buf,sizeof(unsigned long),2,fp);
    if (items != 2)
        return -1;
    for (i=0;i<2;i++)
        buf[i]=le32_to_cpu(buf[i]);
    pdb->default_rec_rule = buf[0];
    num = buf[1];

    items = fread(buf,sizeof(unsigned long),num + 1,fp);
    if (items != (num + 1))
        return -1;

    if (num > 0)
    {
        if ( pdb->default_domain_no == NULL)
        {
            defdomain = (defdom_datum_t *)malloc_sleep(sizeof(defdom_datum_t));
            if (defdomain == NULL)
                goto bad;
            memset(defdomain,0,sizeof(defdom_datum_t));
            pdb->default_domain_no = ptemp = defdomain;
        }
        else
            defdomain = ptemp = pdb->default_domain_no;
        for (i=0;i<num;i++)
        {
            buf[i]=le32_to_cpu(buf[i]);
            if (defdomain == NULL)
            {
                defdomain = (defdom_datum_t *)malloc_sleep(sizeof(defdom_datum_t));
                if (defdomain == NULL)
                    goto bad;
                ptemp->pnext = defdomain;
                ptemp = defdomain;
                memset(defdomain,0,sizeof(defdom_datum_t));
            }
            defdomain->did = buf[i];
            defdomain = defdomain->pnext;
        }
    }
    
    num = buf[num];

    /* especial policies */
    for (i = 0; i < num; i++)
    {
    	items = fread(buf,sizeof(unsigned long),5,fp);
        if (items != 5)
            goto bad;
        for (j=0;j<5;j++)
            buf[j] = le32_to_cpu(buf[j]);
        
        dom_no = buf[1];
        pnode = ac_find_node(&pdb->domain_elem,AC_LIST_DOMAIN,(void *)&dom_no,AC_FIND_BY_NO);
        if (pnode == NULL)
            goto bad;
        
	pnode_dom = (dom_datum_t *)pnode->elem;
        pnode_ap = (ap_datum_t *)ac_malloc_datum(AC_LIST_AP);
        if (pnode_ap == NULL)
            goto bad;
        
        pnode_ap->action = buf[0];
        pnode_ap->reso_type_no = buf[2];
        pnode_ap->class_no = buf[3];
        pnode_ap->priv_bitmap = buf[4];    

	printf("domno(%d): resource type(%d)  class no(%d)  priv(0x%x)\n",buf[0],buf[1],buf[2],buf[3]);        
        pnode = ac_list_new_node((void *)pnode_ap);
        if (pnode == NULL)
            goto bad;
        
        printf("w20598: insert into domtype(%d) list\n",pnode_dom->dno);	
        if (ac_list_add_node((list_head_t *)pnode_dom->apolicy,pnode,NULL) < 0)
            goto bad;
    }
    return 0;
    
bad:
    if (pnode) free(pnode);
    if (pnode_ap) free(pnode_ap);
    return -1;
}

/*================================================================================
FUNCTION: ac_policy_read_sp          
DESCRIPTION: 
   read especial policies in policy file; 
ARGUMENTS PASSED:
   list_head_t *phead
   FILE       *fp
RETURN VALUE:
   int     0:success; <0: error
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
static int ac_policy_read_sp(list_head_t *phead,FILE *fp) 
{
    int i,j,items;
    unsigned long buf[AC_PATH_MAX_CHAR - 1],num;
    sp_datum_t *pnode_sp = NULL;
    list_node_t *pnode = NULL;
    
    items = fread(buf,sizeof(unsigned long),1,fp);
    if (items != 1)
        return -1;
    buf[0]=le32_to_cpu(buf[0]);
    num = buf[0];

    printf("  spec_policy number = %d",buf[0]);    
    /* especial policies */
    for (j = 0; j < num; j++)
    {
    	items = fread(buf,sizeof(unsigned long),2,fp);
        if (items != 2)
            return -1;
        for (i=0;i<2;i++)
            buf[i] = le32_to_cpu(buf[i]);
        
        pnode_sp = (sp_datum_t *)ac_malloc_datum(AC_LIST_SP);
        if (pnode_sp == NULL)
            goto bad;
        
        pnode_sp->class_no = buf[0];
        pnode_sp->pri_bitmap = buf[1];
        
        pnode = ac_list_new_node((void *)pnode_sp);
        if (pnode == NULL)
            goto bad;
            
        if (ac_list_add_node(phead,pnode,NULL) < 0)
            goto bad;
    }
    return 0;
    
bad:
    if (pnode) free(pnode);
    if (pnode_sp) free(pnode_sp);
    return -1;
}

unsigned long ac_get_version(void)
{
    return policydb.policy_ver;
}
