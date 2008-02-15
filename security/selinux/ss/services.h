
/* Author : Stephen Smalley, <sds@epoch.ncsc.mil> */

/* FLASK */

#ifndef _SERVICES_H_
#define _SERVICES_H_

#include "policydb.h"
#include "sidtab.h"

/*
 * The security server uses two global data structures
 * when providing its services:  the SID table (sidtab)
 * and the policy database (policydb).
 */
extern sidtab_t sidtab;
extern policydb_t policydb;

/*
 * The prototypes for the security services provided to
 * the kernel are declared in include/linux/flask/security.h.
 * Some security services are not used by the kernel, so their 
 * prototypes are here.
 */
int security_load_policy(FILE * fp);	/* IN */

int security_get_user_sids(security_id_t callsid,
	                   char *username,
			   security_id_t **sids,
			   __u32 *nel);

int security_get_sids(security_id_t **sids,
		      __u32 *nel);

#endif	/* _SERVICES_H_ */

/* FLASK */

