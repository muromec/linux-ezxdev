
/* FLASK */

/* 
 * Implementation of security server system calls
 */

#include <linux/config.h>
#include <linux/flask/security.h>
#include <linux/flask/avc.h>
#include <linux/flask/syscalls.h>
#include <asm/uaccess.h>	/* copy_to/from_user  */
#include <asm/errno.h>
#include "services.h"

/* hooks.c */
extern int task_has_security(struct task_struct *tsk,
			     access_vector_t perms);

#define CONTEXT_MAX  8192 
#define USERNAME_MAX  255 

/*
 * Compute access vectors based on a SID pair for
 * the permissions in a particular class.
 */
long sys_security_compute_av(struct security_query *query, 
			     struct security_response *response)
{				
	int ret_val;
	struct security_query q2;
	struct security_response r2;


	ret_val = task_has_security(current, SECURITY__COMPUTE_AV);
	if (ret_val)
		goto out;

	if (copy_from_user(&q2, query, sizeof(q2))) {
		ret_val = -EFAULT;
		goto out;
	}

	ret_val = security_compute_av(
			       q2.ssid, q2.tsid, q2.tclass, q2.requested,
					     &(r2.allowed), &(r2.decided),
					     &(r2.auditallow),
					     &(r2.auditdeny),
					     &(r2.seqno));

	if (ret_val)
		goto out;

	if (copy_to_user(response, &r2, sizeof(r2))) {
		ret_val = -EFAULT;
		goto out;
	}

      out:
	return ret_val;
}


/*
 * Write the security context string representation of 
 * the context associated with `sid' into the provided
 * buffer `scontext'.  On entry, `*scontext_len' should
 * be set to the length of the provided buffer.  On exit,
 * `*scontext_len' is set to the length of the string.
 * If the provided buffer is too small, `*scontext_len'
 * is set to the correct length and -ENOSPC is returned.
 */
long sys_security_sid_to_context(security_id_t sid,	
				 security_context_t scontext,	
				 __u32 *scontext_len)
{				
	int ret_val;
	security_context_t scontext2;
	__u32 in_len, out_len;


	ret_val = task_has_security(current, SECURITY__SID_TO_CONTEXT);
	if (ret_val)
		goto out2;

	ret_val = security_sid_to_context(sid, &scontext2, &out_len);
	if (ret_val)
		goto out2;

	if (copy_from_user(&in_len, scontext_len, sizeof(__u32))) {
		ret_val = -EFAULT;
		goto out;
	}
	if (copy_to_user(scontext_len, &out_len, sizeof(__u32))) {
		ret_val = -EFAULT;
		goto out;
	}
	if (in_len < out_len) {
		ret_val = -ENOSPC;
		goto out;
	}
	if (copy_to_user(scontext, scontext2, out_len)) {
		ret_val = -EFAULT;
		goto out;
	}

      out:
	kfree(scontext2);
      out2:
	return ret_val;
}


/*
 * Return a SID associated with the security context that
 * has the string representation specified by `scontext'.
 */
long sys_security_context_to_sid(security_context_t scontext,	
				 __u32 scontext_len,	
				 security_id_t * out_sid)
{				
	int ret_val;
	security_context_t scontext2;
	security_id_t out_sid_2;

	if (scontext_len == 0 || scontext_len > CONTEXT_MAX)
		return -ENAMETOOLONG;

	ret_val = task_has_security(current, SECURITY__CONTEXT_TO_SID);
	if (ret_val)
		goto out;

	scontext2 = kmalloc(scontext_len, GFP_USER);
	if (!scontext2) {
		ret_val = -ENOMEM;
		goto out;
	}
	if (copy_from_user(scontext2, scontext,(unsigned long) scontext_len)) {
		ret_val = -EFAULT;
		goto out;
	}
	ret_val = security_context_to_sid(scontext2, scontext_len, &out_sid_2);
	kfree(scontext2);
	if (ret_val)
		goto out;
	if (copy_to_user(out_sid, &out_sid_2, sizeof(out_sid_2))) {
		ret_val = -EFAULT;
		goto out;
	}

      out:
	return ret_val;
}


/*
 * Compute a SID to use for labeling a new object in the 
 * class `tclass' based on a SID pair.  
 */
long sys_security_transition_sid(security_id_t ssid,	
					   security_id_t tsid,	
					   security_class_t tclass, 
					   security_id_t * out_sid )
{
	int ret_val;
	security_id_t out_sid_2;


	ret_val = task_has_security(current, SECURITY__TRANSITION_SID);
	if (ret_val)
		goto out;

	ret_val = security_transition_sid(ssid, tsid, tclass, &out_sid_2);
	if (ret_val)
		goto out;
	if (copy_to_user(out_sid, &out_sid_2, sizeof(out_sid_2))) {
		ret_val = -EFAULT;
		goto out;
	}

      out:
	return ret_val;
}


/*
 * Compute a SID to use for relabeling an object in the 
 * class `tclass' based on a SID pair.  
 */
long sys_security_change_sid(security_id_t ssid,	
			     security_id_t tsid,	
			     security_class_t tclass, 
			     security_id_t * out_sid )
{
	int ret_val;
	security_id_t out_sid_2;


	ret_val = task_has_security(current, SECURITY__CHANGE_SID);
	if (ret_val)
		goto out;

	ret_val = security_change_sid(ssid, tsid, tclass, &out_sid_2);
	if (ret_val)
		goto out;
	if (copy_to_user(out_sid, &out_sid_2, sizeof(out_sid_2))) {
		ret_val = -EFAULT;
		goto out;
	}

      out:
	return ret_val;
}


/*
 * Compute a SID to use when selecting a member of a 
 * polyinstantiated object of class `tclass' based on 
 * a SID pair.
 */
long sys_security_member_sid(security_id_t ssid,	
			     security_id_t tsid,	
			     security_class_t tclass,	
			     security_id_t * out_sid)
{				
	int ret_val;
	security_id_t out_sid_2;


	ret_val = task_has_security(current, SECURITY__MEMBER_SID);
	if (ret_val)
		goto out;

	ret_val = security_member_sid(ssid, tsid, tclass, &out_sid_2);
	if (ret_val)
		goto out;
	if (copy_to_user(out_sid, &out_sid_2, sizeof(out_sid_2))) {
		ret_val = -EFAULT;
		goto out;
	}

      out:
	return ret_val;
}

char *policyfile;

/*
 * Load a new policy configuration from a file,
 * update the security server state, and reset
 * the access vector cache.  If no path is specified,
 * then the default policy configuration file is used.
 */
long sys_security_load_policy(char *path,
			      __u32 pathlen)
					
{				
	char *file;
	FILE *fp;
	int ret_val;
	unsigned int len;

	if (!policyfile) {
		len = strlen(POLICYDB_PATHPREFIX)+12;
		policyfile = malloc(len);
		if (!policyfile) 
			return -ENOMEM;
#ifdef CONFIG_SECURITY_SELINUX_MLS
		snprintf(policyfile, len, "%s.%d-mls", POLICYDB_PATHPREFIX, 
			 POLICYDB_VERSION);
#else
		snprintf(policyfile, len, "%s.%d", POLICYDB_PATHPREFIX, 
			 POLICYDB_VERSION);
#endif		
	}

	if (pathlen > PATH_MAX) 
		return -ENAMETOOLONG;

	if (pathlen) {
		file = kmalloc(pathlen, GFP_USER);
		if (!file) {
			return -ENOMEM;
		}
		if (copy_from_user(file, path, (unsigned long) pathlen)) {
			ret_val = -EFAULT;
			goto out2;
		}
		if (file[pathlen - 1]) {
			/* Path is not null-terminated. */
			ret_val = -EINVAL;
			goto out2;
		}
		if (file[0] == 0) {
			kfree(file);
			file = policyfile;
		}
	} else
		file = policyfile;

	fp = fopen(file, "r");
	if (!fp) {
		ret_val = -EINVAL;
		goto out2;
	}
	ret_val = task_has_security(current, SECURITY__LOAD_POLICY);
	if (ret_val) 
		goto out;
	printf("security:  loading policy configuration from %s\n", file);
	ret_val = security_load_policy(fp);

      out:
	fclose(fp);
      out2:
	if (file != policyfile)
		kfree(file);
	return ret_val;
}


/*
 * Write the set of active SIDs into the provided  
 * array `sids'. On entry, `*nel' should be set
 * to the number of elements in the provided array.  On 
 * exit, `*nel' is set to the number of active SIDs.
 * If the provided array is too small, `*nel' is set
 * to the number of active SIDs and -ENOSPC is returned.
 */
long sys_security_get_sids(security_id_t * sids,
			   __u32 *nel)
{
	security_id_t *sids2 = 0;
	__u32 in_nel, out_nel;
	int ret_val;

	ret_val = task_has_security(current, SECURITY__GET_SIDS);
	if (ret_val)
		goto out;

	if (copy_from_user(&in_nel, nel, sizeof(__u32))) {
		ret_val = -EFAULT;
		goto out;
	}

	ret_val = security_get_sids(&sids2, &out_nel);
	if (ret_val)
		goto out;

	if (copy_to_user(nel, &out_nel, sizeof(__u32))) {
		ret_val = -EFAULT;
		goto out;
	}
	if (in_nel < out_nel) {
		ret_val = -ENOSPC;
		goto out;
	}

	if (copy_to_user(sids, sids2, sizeof(security_id_t) * out_nel)) {
		ret_val = -EFAULT;
		goto out;
	}
      out:
	if (sids2)
		kfree(sids2);
	return ret_val;
}

/*
 * Generate the set of SIDs for legal security contexts
 * for a given user that can be reached by `fromsid'.
 * Write the set of SIDs into the provided array `sids'.  
 * On entry, `*nel' should be set to the number of elements in 
 * the provided array.  On exit, `*nel' is set to the number of SIDs.
 * If the provided array is too small, `*nel' is set to the number 
 * of SIDs and -ENOSPC is returned.
 */
long sys_security_get_user_sids(security_id_t fromsid,
					  char *username, 
					  __u32 namelen,
					  security_id_t * sids,
					  __u32 *nel)
{
	int ret_val = 0;
	char *username2 = 0;
	security_id_t *sids2 = 0;
	__u32 in_nel, out_nel;

	if (namelen == 0 || namelen > USERNAME_MAX)
		return -ENAMETOOLONG;


	ret_val = task_has_security(current, SECURITY__GET_USER_SIDS);
	if (ret_val)
		goto out;

	if (copy_from_user(&in_nel, nel, sizeof(__u32))) {
		ret_val = -EFAULT;
		goto out;
	}

	username2 = kmalloc(namelen, GFP_USER);
	if (!username2) {
		ret_val = -ENOMEM;
		goto out;
	}

	if (copy_from_user(username2, username, (unsigned long)namelen)) {
		ret_val = -EFAULT;
		goto out;
	}

	if (username2[namelen - 1]) {
		ret_val = -EINVAL;
		goto out;
	}

	ret_val = security_get_user_sids(fromsid, username2,
					 &sids2, &out_nel);
	if (ret_val) 
		goto out;

	if (copy_to_user(nel, &out_nel, sizeof(__u32))) {
		ret_val = -EFAULT;
		goto out;
	}
		
	if (in_nel < out_nel) {
		ret_val = -ENOSPC;
		goto out;
	}

	if (copy_to_user(sids, sids2, sizeof(security_id_t) * out_nel)) {
		ret_val = -EFAULT;
		goto out;
	}

out:
	if (username2)
		kfree(username2);
	if (sids2)
		kfree(sids2);
	return ret_val;
}

#ifdef CONFIG_SECURITY_SELINUX_MLS
long sys_security_mls(void)
{
	return 1;
}
#else
long sys_security_mls(void)
{
	return 0;
}
#endif

/* FLASK */

