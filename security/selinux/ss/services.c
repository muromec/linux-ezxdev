
/*
 * Author : Stephen Smalley, <sds@epoch.ncsc.mil> 
 */

/* FLASK */

/*
 * Implementation of the security services.
 */

#include "context.h"
#include "policydb.h"
#include "services.h"
#include "sidtab.h"
#include "mls.h"
#include "services_private.h"

sidtab_t sidtab;
policydb_t policydb;
int ss_initialized = 0;

/*
 * The largest sequence number that has been used when
 * providing an access decision to the access vector cache.
 * The sequence number only changes when a policy change
 * occurs.
 */
static __u32 latest_granting = 0;


/*
 * Return the boolean value of a constraint expression 
 * when it is applied to the specified source and target 
 * security contexts.
 */
static int constraint_expr_eval(context_struct_t * scontext,
				context_struct_t * tcontext,
				constraint_expr_t * expr)
{
	__u32 val1, val2;
	context_struct_t *c;
	role_datum_t *r1, *r2;

	switch (expr->expr_type) {
	case CEXPR_NOT:
		return !constraint_expr_eval(scontext, tcontext, expr->left);
	case CEXPR_AND:
		return constraint_expr_eval(scontext, tcontext, expr->left) &&
		    constraint_expr_eval(scontext, tcontext, expr->right);
	case CEXPR_OR:
		return constraint_expr_eval(scontext, tcontext, expr->left) ||
		    constraint_expr_eval(scontext, tcontext, expr->right);
	case CEXPR_ATTR:
		if (expr->attr & CEXPR_USER) {
			val1 = scontext->user;
			val2 = tcontext->user;
		} else if (expr->attr & CEXPR_ROLE) {
			val1 = scontext->role;
			val2 = tcontext->role;
			r1 = policydb.role_val_to_struct[val1 - 1];
			r2 = policydb.role_val_to_struct[val2 - 1];
			switch (expr->op) {
			case CEXPR_DOM:
				return ebitmap_get_bit(&r1->dominates,
						       val2 - 1);
			case CEXPR_DOMBY:
				return ebitmap_get_bit(&r2->dominates,
						       val1 - 1);
			case CEXPR_INCOMP:
				return ( !ebitmap_get_bit(&r1->dominates,
							  val2 - 1) &&
					 !ebitmap_get_bit(&r2->dominates,
							  val1 - 1) );
			}
		} else if (expr->attr & CEXPR_TYPE) {
			val1 = scontext->type;
			val2 = tcontext->type;
		} else
			return 0;
		
		switch (expr->op) {
		case CEXPR_EQ:
			return (val1 == val2);
		case CEXPR_NEQ:
			return (val1 != val2);
		default:
			return 0;
		}
		return 0;
	case CEXPR_NAMES:
		c = scontext;
		if (expr->attr & CEXPR_TARGET) 
			c = tcontext;
		if (expr->attr & CEXPR_USER) 
			val1 = c->user;
		else if (expr->attr & CEXPR_ROLE)
			val1 = c->role;
		else if (expr->attr & CEXPR_TYPE)
			val1 = c->type;
		else 
			return 0;

		switch (expr->op) {
		case CEXPR_EQ:
			return ebitmap_get_bit(&expr->names, val1 - 1);
		case CEXPR_NEQ:
			return !ebitmap_get_bit(&expr->names, val1 - 1);
		default:
			return 0;
		}
		return 0;
	default:
		return 0;
	}

	return 0;
}


/*
 * Compute access vectors based on a context structure pair for
 * the permissions in a particular class.
 */
static int context_struct_compute_av(context_struct_t *scontext,
				     context_struct_t *tcontext,
				     security_class_t tclass,
				     access_vector_t requested,
				     access_vector_t * allowed,
				     access_vector_t * decided,
				     access_vector_t * auditallow,
				     access_vector_t * auditdeny,
				     __u32 *seqno)
{
	constraint_node_t *constraint;
	struct role_allow *ra;
	avtab_key_t avkey;
	avtab_datum_t *avdatum;
	class_datum_t *tclass_datum;

	*seqno = latest_granting;
	*decided = 0xffffffff;

	if (!tclass || tclass > policydb.p_classes.nprim) {
		printf("security_compute_av:  unrecognized class %d\n",
		       tclass);
		return -EINVAL;
	}
	tclass_datum = policydb.class_val_to_struct[tclass - 1];

	/* 
	 * Initialize the access vectors to the default values.
	 */
	*allowed = 0;
	*auditallow = 0;
	*auditdeny = ~0;

	/*
	 * If a specific type enforcement rule was defined for
	 * this permission check, then use it.
	 */
	avkey.source_type = scontext->type;
	avkey.target_type = tcontext->type;
	avkey.target_class = tclass;
	avdatum = avtab_search(&policydb.te_avtab, &avkey, AVTAB_AV);
	if (avdatum) {
		if (avdatum->specified & AVTAB_ALLOWED)
			*allowed = avtab_allowed(avdatum);
		if (avdatum->specified & AVTAB_AUDITDENY)
			*auditdeny = avtab_auditdeny(avdatum);
		if (avdatum->specified & AVTAB_AUDITALLOW)
			*auditallow = avtab_auditallow(avdatum);
	}

	/*
	 * Remove any permissions prohibited by the MLS policy.
	 */
	mls_compute_av(scontext, tcontext, tclass_datum, allowed);

	/* 
	 * Remove any permissions prohibited by a constraint.
	 */
	constraint = tclass_datum->constraints;
	while (constraint) {
		if ((constraint->permissions & (*allowed)) &&
		    !constraint_expr_eval(scontext, tcontext,
					  constraint->expr)) {
			*allowed = (*allowed) & ~(constraint->permissions);
		}
		constraint = constraint->next;
	}

	/* 
	 * If checking process transition permission and the
	 * role is changing, then check the (current_role, new_role) 
	 * pair.
	 */
	if (tclass == SECCLASS_PROCESS &&
	    *allowed && PROCESS__TRANSITION &&
	    scontext->role != tcontext->role) {
		for (ra = policydb.role_allow; ra; ra = ra->next) {
			if (scontext->role == ra->role &&
			    tcontext->role == ra->new_role) 
				break;
		}		
		if (!ra)
			*allowed = (*allowed) & ~(PROCESS__TRANSITION);
	}	

	return 0;
}


int security_compute_av(security_id_t ssid,
			security_id_t tsid,
			security_class_t tclass,
			access_vector_t requested,
			access_vector_t * allowed,
			access_vector_t * decided,
			access_vector_t * auditallow,
			access_vector_t * auditdeny,
			__u32 *seqno)
{
	context_struct_t *scontext = 0, *tcontext = 0;
	int rc = 0;

	if (!ss_initialized) {
		*allowed = requested;
		*decided = requested;
		*auditallow = 0;
		*auditdeny = 0xffffffff;
		return 0;
	}

	POLICY_RDLOCK;

	scontext = sidtab_search(&sidtab, ssid);
	if (!scontext) {
		printf("security_compute_av:  unrecognized SID %d\n", ssid);
		rc = -EINVAL;
		goto out;
	}
	tcontext = sidtab_search(&sidtab, tsid);
	if (!tcontext) {
		printf("security_compute_av:  unrecognized SID %d\n", tsid);
		rc = -EINVAL;
		goto out;
	}

	rc = context_struct_compute_av(scontext, tcontext, tclass,
				       requested, allowed, decided,
				       auditallow, auditdeny,
				       seqno);
out:
	POLICY_RDUNLOCK;
	return rc;
}


/*
 * Write the security context string representation of 
 * the context structure `context' into a dynamically
 * allocated string of the correct size.  Set `*scontext'
 * to point to this string and set `*scontext_len' to
 * the length of the string.
 */
int context_struct_to_string(context_struct_t * context,
			     security_context_t * scontext,
			     __u32 *scontext_len)
{
	char *scontextp;

	*scontext = 0;
	*scontext_len = 0;

	/* Compute the size of the context. */
	*scontext_len += strlen(policydb.p_user_val_to_name[context->user - 1]) + 1;
	*scontext_len += strlen(policydb.p_role_val_to_name[context->role - 1]) + 1;
	*scontext_len += strlen(policydb.p_type_val_to_name[context->type - 1]) + 1;
	*scontext_len += mls_compute_context_len(context);

	/* Allocate space for the context; caller must free this space. */
	scontextp = (char *) malloc(*scontext_len+1);
	if (!scontextp) {
		return -ENOMEM;
	}
	*scontext = (security_context_t) scontextp;

	/*
	 * Copy the user name, role name and type name into the context.
	 */
	sprintf(scontextp, "%s:%s:%s:",
		policydb.p_user_val_to_name[context->user - 1],
		policydb.p_role_val_to_name[context->role - 1],
		policydb.p_type_val_to_name[context->type - 1]);
	scontextp += strlen(policydb.p_user_val_to_name[context->user - 1]) + 1 + strlen(policydb.p_role_val_to_name[context->role - 1]) + 1 + strlen(policydb.p_type_val_to_name[context->type - 1]) + 1;

	mls_sid_to_context(context, &scontextp);

	scontextp--;
	*scontextp = 0;

	return 0;
}


#include <linux/flask/initial_sid_to_string.h>

/*
 * Write the security context string representation of 
 * the context associated with `sid' into a dynamically
 * allocated string of the correct size.  Set `*scontext'
 * to point to this string and set `*scontext_len' to
 * the length of the string.
 */
int security_sid_to_context(security_id_t sid,
			    security_context_t * scontext,
			    __u32 *scontext_len)
{
	context_struct_t *context;
	int rc = 0;

	if (!ss_initialized) {
		if (sid <= SECINITSID_NUM) {
			char *scontextp;

			*scontext_len = strlen(initial_sid_to_string[sid]) + 1;
			scontextp = malloc(*scontext_len);
			strcpy(scontextp, initial_sid_to_string[sid]);
			*scontext = (security_context_t) scontextp;
			return 0;
		}
		printf("security_sid_to_context:  called before initial load_policy on unknown SID %d\n", sid);
		return -EINVAL;
	}
	POLICY_RDLOCK;
	context = sidtab_search(&sidtab, sid);
	if (!context) {
		printf("security_sid_to_context:  unrecognized SID %d\n", sid);
		rc = -EINVAL;
		goto out;
	}
	rc = context_struct_to_string(context, scontext, scontext_len);
out:
	POLICY_RDUNLOCK;
	return rc;
	
}

/*
 * Return a SID associated with the security context that
 * has the string representation specified by `scontext'.
 */
int security_context_to_sid(security_context_t scontext,
			    __u32 scontext_len,
			    security_id_t * sid)
{
	security_context_t scontext2;
	context_struct_t context;
	role_datum_t *role;
	type_datum_t *typdatum;
	user_datum_t *usrdatum;
	char *scontextp, *p, oldc;
	int rc = 0;

	if (!ss_initialized) {
		int i;

		for (i = 1; i < SECINITSID_NUM; i++) {
			if (!strcmp(initial_sid_to_string[i], scontext)) {
				*sid = i;
				return 0;
			}
		}
		printf("security_context_to_sid: called before initial load_policy on unknown context %s\n", scontext);
		return -EINVAL;
	}
	*sid = SECSID_NULL;

	/* copy the string so that we can modify the copy as we parse it */
	scontext2 = malloc(scontext_len);
	if (!scontext2) {
		return -ENOMEM;
	}
	memcpy(scontext2, scontext, scontext_len);

	context_init(&context);
	*sid = SECSID_NULL;

	POLICY_RDLOCK;

	/* Parse the security context. */

	rc = -EINVAL;
	scontextp = (char *) scontext2;
	if (scontextp[scontext_len - 1])
		/* Security context is not null-terminated. */
		goto out;

	/* Extract the user. */
	p = scontextp;
	while (*p && *p != ':')
		p++;

	if (*p == 0)
		goto out;

	*p++ = 0;

	usrdatum = (user_datum_t *) hashtab_search(policydb.p_users.table,
					      (hashtab_key_t) scontextp);
	if (!usrdatum)
		goto out;

	context.user = usrdatum->value;

	/* Extract role. */
	scontextp = p;
	while (*p && *p != ':')
		p++;

	if (*p == 0)
		goto out;

	*p++ = 0;

	role = (role_datum_t *) hashtab_search(policydb.p_roles.table,
					       (hashtab_key_t) scontextp);
	if (!role)
		goto out;
	context.role = role->value;

	/* Extract type. */
	scontextp = p;
	while (*p && *p != ':')
		p++;
	oldc = *p;
	*p++ = 0;

	typdatum = (type_datum_t *) hashtab_search(policydb.p_types.table,
					      (hashtab_key_t) scontextp);

	if (!typdatum)
		goto out;

	context.type = typdatum->value;

	rc = mls_context_to_sid(oldc, &p, &context);
	if (rc)
		goto out;

	/* Check the validity of the new context. */
	if (!policydb_context_isvalid(&policydb, &context)) {
		rc = -EINVAL;
		goto out;
	}
	/* Obtain the new sid. */
	rc = sidtab_context_to_sid(&sidtab, &context, sid);
out:
	POLICY_RDUNLOCK;
	context_destroy(&context);
	free(scontext2);
	return rc;
}

static int security_compute_sid(security_id_t ssid,
				security_id_t tsid,
				security_class_t tclass,
				__u32 specified,
				security_id_t * out_sid)
{
	context_struct_t *scontext = 0, *tcontext = 0, newcontext;
	struct role_trans *roletr = 0;
	avtab_key_t avkey;
	avtab_datum_t *avdatum;
	unsigned int type_change = 0;
	int rc = 0;

	if (!ss_initialized) {
		switch (tclass) {
		case SECCLASS_PROCESS:
			*out_sid = ssid;
			break;
		default:
			*out_sid = tsid;
			break;
		}
		return 0;
	}
	
	POLICY_RDLOCK;

	scontext = sidtab_search(&sidtab, ssid);
	if (!scontext) {
		printf("security_compute_sid:  unrecognized SID %d\n", ssid);
		rc = -EINVAL;
		goto out;
	}
	tcontext = sidtab_search(&sidtab, tsid);
	if (!tcontext) {
		printf("security_compute_sid:  unrecognized SID %d\n", tsid);
		rc = -EINVAL;
		goto out;
	}

	context_init(&newcontext);

	/* Set the user identity. */
	switch (specified) {
	case AVTAB_TRANSITION:
	case AVTAB_CHANGE:
		/* Use the process user identity. */
		newcontext.user = scontext->user;
		break;
	case AVTAB_MEMBER:
		/* Use the related object owner. */
		newcontext.user = tcontext->user;
		break;
	}

	/* Set the role and type to default values. */
	switch (tclass) {
	case SECCLASS_PROCESS:
		/* Use the current role and type of process. */
		newcontext.role = scontext->role;
		newcontext.type = scontext->type;
		break;
	default:
		/* Use the well-defined object role. */
		newcontext.role = OBJECT_R_VAL;
		/* Use the type of the related object. */
		newcontext.type = tcontext->type;
	}

	/* Look for a type transition/member/change rule. */
	avkey.source_type = scontext->type;
	avkey.target_type = tcontext->type;
	avkey.target_class = tclass;
	avdatum = avtab_search(&policydb.te_avtab, &avkey, AVTAB_TYPE);
	type_change = (avdatum && (avdatum->specified & specified));
	if (type_change) {
		/* Use the type from the type transition/member/change rule. */
		switch (specified) {
		case AVTAB_TRANSITION:
			newcontext.type = avtab_transition(avdatum);
			break;
		case AVTAB_MEMBER:
			newcontext.type = avtab_member(avdatum);
			break;
		case AVTAB_CHANGE:
			newcontext.type = avtab_change(avdatum);
			break;
		}
	}

	/* Check for class-specific changes. */
	switch (tclass) {
	case SECCLASS_PROCESS:
		if (specified & AVTAB_TRANSITION) {
			/* Look for a role transition rule. */
			for (roletr = policydb.role_tr; roletr; 
			     roletr = roletr->next) {
				if (roletr->role == scontext->role &&
				    roletr->type == tcontext->type) {
					/* Use the role transition rule. */
					newcontext.role = roletr->new_role;
					break;
				}
			}
		}

		if (!type_change && !roletr) {
			/* No change in process role or type. */
			*out_sid = ssid;
			goto out;

		}
		break;
	default:
		if (!type_change &&
		    (newcontext.user == tcontext->user) &&
		    mls_context_cmp(scontext, tcontext)) {
                        /* No change in object type, owner, 
			   or MLS attributes. */
			*out_sid = tsid;
			goto out;
		}
		break;
	}

	/* Set the MLS attributes.
	   This is done last because it may allocate memory. */
	rc = mls_compute_sid(scontext, tcontext, tclass, specified, &newcontext);
	if (rc) 
		goto out;

	/* Check the validity of the context. */
	if (!policydb_context_isvalid(&policydb, &newcontext)) {
		rc = compute_sid_handle_invalid_context(scontext, 
							tcontext, 
							tclass, 
							&newcontext);
		if (rc)
			goto out;
	}
	/* Obtain the sid for the context. */
	rc = sidtab_context_to_sid(&sidtab, &newcontext, out_sid);
out:
	POLICY_RDUNLOCK;
	context_destroy(&newcontext);
	return rc;
}

/*
 * Compute a SID to use for labeling a new object in the 
 * class `tclass' based on a SID pair.  
 */
int security_transition_sid(security_id_t ssid,
			    security_id_t tsid,
			    security_class_t tclass,
			    security_id_t * out_sid)
{
	return security_compute_sid(ssid, tsid, tclass, AVTAB_TRANSITION, out_sid);
}


/*
 * Compute a SID to use when selecting a member of a 
 * polyinstantiated object of class `tclass' based on 
 * a SID pair.
 */
int security_member_sid(security_id_t ssid,
			security_id_t tsid,
			security_class_t tclass,
			security_id_t * out_sid)
{
	return security_compute_sid(ssid, tsid, tclass, AVTAB_MEMBER, out_sid);
}


/*
 * Compute a SID to use for relabeling an object in the 
 * class `tclass' based on a SID pair.  
 */
int security_change_sid(security_id_t ssid,
			security_id_t tsid,
			security_class_t tclass,
			security_id_t * out_sid)
{
	return security_compute_sid(ssid, tsid, tclass, AVTAB_CHANGE, out_sid);
}


/*
 * Verify that each permission that is defined under the
 * existing policy is still defined with the same value
 * in the new policy.
 */
static int validate_perm(hashtab_key_t key, hashtab_datum_t datum, void *p)
{
	hashtab_t h;
	perm_datum_t *perdatum, *perdatum2;


	h = (hashtab_t) p;
	perdatum = (perm_datum_t *) datum;

	perdatum2 = (perm_datum_t *) hashtab_search(h, key);
	if (!perdatum2) {
		printf("security:  permission %s disappeared", key);
		return -1;
	}
	if (perdatum->value != perdatum2->value) {
		printf("security:  the value of permission %s changed", key);
		return -1;
	}
	return 0;
}


/*
 * Verify that each class that is defined under the
 * existing policy is still defined with the same 
 * attributes in the new policy.
 */
static int validate_class(hashtab_key_t key, hashtab_datum_t datum, void *p)
{
	policydb_t *newp;
	class_datum_t *cladatum, *cladatum2;

	newp = (policydb_t *) p;
	cladatum = (class_datum_t *) datum;

	cladatum2 = (class_datum_t *) hashtab_search(newp->p_classes.table, key);
	if (!cladatum2) {
		printf("security:  class %s disappeared\n", key);
		return -1;
	}
	if (cladatum->value != cladatum2->value) {
		printf("security:  the value of class %s changed\n", key);
		return -1;
	}
	if ((cladatum->comdatum && !cladatum2->comdatum) ||
	    (!cladatum->comdatum && cladatum2->comdatum)) {
		printf("security:  the inherits clause for the access vector definition for class %s changed\n", key);
		return -1;
	}
	if (cladatum->comdatum) {
		if (hashtab_map(cladatum->comdatum->permissions.table, validate_perm,
				cladatum2->comdatum->permissions.table)) {
			printf(" in the access vector definition for class %s\n", key);
			return -1;
		}
	}
	if (hashtab_map(cladatum->permissions.table, validate_perm,
			cladatum2->permissions.table)) {
		printf(" in access vector definition for class %s\n", key);
		return -1;
	}
	return 0;
}

/* Clone the SID into the new SID table. */
static int clone_sid(security_id_t sid,
		     context_struct_t *context,
		     void *arg)
{
	sidtab_t *s = arg;

	return sidtab_insert(s, sid, context);
}

typedef struct {
	policydb_t *oldp;
	policydb_t *newp;
} convert_context_args_t;

/*
 * Convert the values in the security context
 * structure `c' from the values specified
 * in the policy `p->oldp' to the values specified
 * in the policy `p->newp'.  Verify that the
 * context is valid under the new policy.
 */
static int convert_context(security_id_t key,
			   context_struct_t * c,
			   void *p)
{
	convert_context_args_t *args;
	context_struct_t oldc;
	role_datum_t *role;
	type_datum_t *typdatum;
	user_datum_t *usrdatum;
	security_context_t s;
	__u32 len;
	int rc = -EINVAL;
       
	args = (convert_context_args_t *) p;

	if (context_cpy(&oldc, c))
		return -ENOMEM;

	/* Convert the user. */
	usrdatum = (user_datum_t *) hashtab_search(args->newp->p_users.table,
			    args->oldp->p_user_val_to_name[c->user - 1]);

	if (!usrdatum) {
		goto bad;
	}
	c->user = usrdatum->value;

	/* Convert the role. */
	role = (role_datum_t *) hashtab_search(args->newp->p_roles.table,
			    args->oldp->p_role_val_to_name[c->role - 1]);
	if (!role) {
		goto bad;
	}
	c->role = role->value;

	/* Convert the type. */
	typdatum = (type_datum_t *)
	    hashtab_search(args->newp->p_types.table,
			   args->oldp->p_type_val_to_name[c->type - 1]);
	if (!typdatum) {
		goto bad;
	}
	c->type = typdatum->value;

	rc = mls_convert_context(args->oldp, args->newp, c);
	if (rc)
		goto bad;

	/* Check the validity of the new context. */
	if (!policydb_context_isvalid(args->newp, c)) {
		rc = convert_context_handle_invalid_context(&oldc);
		if (rc)
			goto bad;
	}

	context_destroy(&oldc);
	return 0;

      bad:
	context_struct_to_string(&oldc, &s, &len);
	context_destroy(&oldc);
	printf("security:  invalidating context %s\n", s);
	free(s);
	return rc;
}


/*
 * Read a new set of configuration data from 
 * a policy database binary representation file.
 *
 * Verify that each class that is defined under the
 * existing policy is still defined with the same 
 * attributes in the new policy.  
 *
 * Convert the context structures in the SID table to the
 * new representation and verify that all entries
 * in the SID table are valid under the new policy. 
 *
 * Change the active policy database to use the new 
 * configuration data.  
 *
 * Reset the access vector cache.
 */
int security_load_policy(FILE * fp)
{
	policydb_t oldpolicydb, newpolicydb;
	sidtab_t oldsidtab, newsidtab;
	convert_context_args_t args;
	__u32 seqno;
	int rc = 0;

	if (!ss_initialized) {
		if (policydb_read(&policydb, fp)) {
			return -EINVAL;
		}
		if (policydb_load_isids(&policydb, &sidtab)) {
			policydb_destroy(&policydb);
			return -EINVAL;
		}
		ss_initialized = 1;
		return 0;
	}

#if 0
	sidtab_hash_eval(&sidtab, "sids");
#endif 

	LOAD_LOCK;

	if (policydb_read(&newpolicydb, fp)) {
		LOAD_UNLOCK;
		return -EINVAL;
	}

	sidtab_init(&newsidtab);

	/* Verify that the existing classes did not change. */
	if (hashtab_map(policydb.p_classes.table, validate_class, &newpolicydb)) {
		printf("security:  the definition of an existing class changed\n");
		rc = -EINVAL;
		goto err;
	}

	/* Clone the SID table. */
	if (sidtab_map(&sidtab, clone_sid, &newsidtab)) {
		rc = -ENOMEM;
		goto err;
	}

	/* Convert the internal representations of contexts 
	   in the new SID table and remove invalid SIDs. */
	args.oldp = &policydb;
	args.newp = &newpolicydb;
	sidtab_map_remove_on_error(&newsidtab, convert_context, &args);

	/* Save the old policydb and SID table to free later. */
	memcpy(&oldpolicydb, &policydb, sizeof policydb);
	sidtab_set(&oldsidtab, &sidtab);

	/* Install the new policydb and SID table. */
	POLICY_WRLOCK;
	memcpy(&policydb, &newpolicydb, sizeof policydb);
	sidtab_set(&sidtab, &newsidtab);
	seqno = ++latest_granting;
	POLICY_WRUNLOCK;
	LOAD_UNLOCK;

	/* Free the old policydb and SID table. */
	policydb_destroy(&oldpolicydb);
	sidtab_destroy(&oldsidtab);

	avc_ss_reset(seqno);

	return 0;

err:
	LOAD_UNLOCK;
	sidtab_destroy(&newsidtab);
	policydb_destroy(&newpolicydb);
	return rc;

}

/*
 * Return the SIDs to use for an unlabeled file system
 * that is being mounted from the device with the
 * the kdevname `name'.  The `fs_sid' SID is returned for 
 * the file system and the `file_sid' SID is returned
 * for all files within that file system.
 */
int security_fs_sid(char *name,
		    security_id_t * fs_sid,
		    security_id_t * file_sid)
{
	int rc = 0;
	ocontext_t *c;

	POLICY_RDLOCK;

	c = policydb.ocontexts[OCON_FS];
	while (c) {
		if (strcmp(c->u.name, name) == 0)
			break;
		c = c->next;
	}

	if (c) {
		if (!c->sid[0] || !c->sid[1]) {
			rc = sidtab_context_to_sid(&sidtab,
						   &c->context[0],
						   &c->sid[0]);
			if (rc)
				goto out;
			rc = sidtab_context_to_sid(&sidtab,
						   &c->context[1],
						   &c->sid[1]);
			if (rc)
				goto out;
		}
		*fs_sid = c->sid[0];
		*file_sid = c->sid[1];
	} else {
		*fs_sid = SECINITSID_FS;
		*file_sid = SECINITSID_FILE;
	}

      out:
	POLICY_RDUNLOCK;
	return rc;
}


/*
 * Return the SID of the port specified by
 * `domain', `type', `protocol', and `port'.
 */
int security_port_sid(__u16 domain,
		      __u16 type,
		      __u8 protocol,
		      __u16 port,
		      security_id_t * out_sid)
{
	ocontext_t *c;
	int rc = 0;

	POLICY_RDLOCK;

	c = policydb.ocontexts[OCON_PORT];
	while (c) {
		if (c->u.port.protocol == protocol &&
		    c->u.port.low_port <= port &&
		    c->u.port.high_port >= port)
			break;
		c = c->next;
	}

	if (c) {
		if (!c->sid[0]) {
			rc = sidtab_context_to_sid(&sidtab,
						   &c->context[0],
						   &c->sid[0]);
			if (rc)
				goto out;
		}
		*out_sid = c->sid[0];
	} else {
		*out_sid = SECINITSID_PORT;
	}

      out:
	POLICY_RDUNLOCK;
	return rc;
}


/*
 * Return the SIDs to use for a network interface
 * with the name `name'.  The `if_sid' SID is returned for 
 * the interface and the `msg_sid' SID is returned as 
 * the default SID for messages received on the
 * interface.
 */
int security_netif_sid(char *name,
		       security_id_t * if_sid,
		       security_id_t * msg_sid)
{
	int rc = 0;
	ocontext_t *c;

	POLICY_RDLOCK;

	c = policydb.ocontexts[OCON_NETIF];
	while (c) {
		if (strcmp(name, c->u.name) == 0)
			break;
		c = c->next;
	}

	if (c) {
		if (!c->sid[0] || !c->sid[1]) {
			rc = sidtab_context_to_sid(&sidtab,
						  &c->context[0],
						  &c->sid[0]);
			if (rc)
				goto out;
			rc = sidtab_context_to_sid(&sidtab,
						   &c->context[1],
						   &c->sid[1]);
			if (rc)
				goto out;
		}
		*if_sid = c->sid[0];
		*msg_sid = c->sid[1];
	} else {
		*if_sid = SECINITSID_NETIF;
		*msg_sid = SECINITSID_NETMSG;
	}

      out:
	POLICY_RDUNLOCK;
	return rc;
}


/*
 * Return the SID of the node specified by the address
 * `addrp' where `addrlen' is the length of the address
 * in bytes and `domain' is the communications domain or
 * address family in which the address should be interpreted.
 */
int security_node_sid(__u16 domain,
		      void *addrp,
		      __u32 addrlen,
		      security_id_t *out_sid)
{
	int rc = 0;
	__u32 addr;
	ocontext_t *c;

	POLICY_RDLOCK; 

	if (domain != AF_INET || addrlen != sizeof(__u32)) {
		*out_sid = SECINITSID_NODE;
		goto out;
	}
	addr = *((__u32 *)addrp);

	c = policydb.ocontexts[OCON_NODE];
	while (c) {
		if (c->u.node.addr == (addr & c->u.node.mask))
			break;
		c = c->next;
	}

	if (c) {
		if (!c->sid[0]) {
			rc = sidtab_context_to_sid(&sidtab,
						   &c->context[0],
						   &c->sid[0]);
			if (rc)
				goto out;
		}
		*out_sid = c->sid[0];
	} else {
		*out_sid = SECINITSID_NODE;
	}

      out:
	POLICY_RDUNLOCK;
	return rc;
}

/*
 * Generate the set of active SIDs.
 * Set `*sids' to point to a dynamically allocated 
 * array containing the set of SIDs.  Set `*nel' to the
 * number of elements in the array.
 */
int security_get_sids(security_id_t **sids,
		      __u32 *nel)
{
	int rc;
	POLICY_RDLOCK; 
	rc = sidtab_get_sids(&sidtab, sids, nel);
	POLICY_RDUNLOCK;
	return rc;
}

/*
 * Generate the set of SIDs for legal security contexts
 * for a given user that can be reached by `fromsid'.
 * Set `*sids' to point to a dynamically allocated 
 * array containing the set of SIDs.  Set `*nel' to the
 * number of elements in the array.
 */
#define SIDS_NEL 25

int security_get_user_sids(security_id_t fromsid,
	                   char *username,
			   security_id_t **sids,
			   __u32 *nel)
{
	context_struct_t *fromcon, usercon;
	security_id_t *mysids, *mysids2, sid;
	__u32 mynel = 0, maxnel = SIDS_NEL;
	user_datum_t *user;
	role_datum_t *role;
	access_vector_t allowed, decided;
	access_vector_t auditallow, auditdeny;
	__u32 seqno;
	int rc = 0, i, j;

	if (!ss_initialized) {
		*sids = 0;
		*nel = 0;
		return 0;
	}

	POLICY_RDLOCK; 

	fromcon = sidtab_search(&sidtab, fromsid);
	if (!fromcon) {
		rc = -EINVAL;
		goto out;
	}

	user = (user_datum_t *) hashtab_search(policydb.p_users.table,
					       username);
	if (!user) {
		rc = -EINVAL;
		goto out;
	}
	usercon.user = user->value;

	mysids = malloc(maxnel*sizeof(security_id_t));
	if (!mysids) {
		rc = -ENOMEM;
		goto out;
	}
	memset(mysids, 0, maxnel*sizeof(security_id_t));

	for (i = ebitmap_startbit(&user->roles); i < ebitmap_length(&user->roles); i++) {
		if (!ebitmap_get_bit(&user->roles, i)) 
			continue;		
		role = policydb.role_val_to_struct[i];
		usercon.role = i+1;
		for (j = ebitmap_startbit(&role->types); j < ebitmap_length(&role->types); j++) {
			if (!ebitmap_get_bit(&role->types, j)) 
				continue;	
			usercon.type = j+1;
			if (usercon.type == fromcon->type)
				continue;
			mls_for_user_ranges(user,usercon) {
				rc = context_struct_compute_av(fromcon, &usercon, 
							       SECCLASS_PROCESS,
							       PROCESS__TRANSITION, 
							       &allowed, &decided,
							       &auditallow, &auditdeny,
							       &seqno);
				if (rc ||  !(allowed & PROCESS__TRANSITION)) 
					continue;
				rc = sidtab_context_to_sid(&sidtab, &usercon, &sid);
				if (rc) {
					free(mysids);
					goto out;
				}
				if (mynel < maxnel) {
					mysids[mynel++] = sid;
				} else {
					maxnel += SIDS_NEL;
					mysids2 = malloc(maxnel*sizeof(security_id_t));
					if (!mysids2) {
						rc = -ENOMEM;
						free(mysids);
						goto out;
					}
					memset(mysids2, 0, maxnel*sizeof(security_id_t));
					memcpy(mysids2, mysids, mynel * sizeof(security_id_t));
					free(mysids);
					mysids = mysids2;
					mysids[mynel++] = sid;
				}
			}
			mls_end_user_ranges;
		}
	}

	*sids = mysids;
	*nel = mynel;

out:	
	POLICY_RDUNLOCK;
	return rc;
}

/*
 * Return the SID to use for a file in a filesystem
 * that cannot support a persistent label mapping or use another
 * fixed labeling behavior like transition SIDs or task SIDs.
 */
int security_genfs_sid(const char *fstype,
	               char *path,
		       security_class_t sclass,
		       security_id_t *sid)
{
	int len;
	genfs_t *genfs;
	ocontext_t *c;
	int rc = 0, cmp = 0;

	POLICY_RDLOCK;

	for (genfs = policydb.genfs; genfs; genfs = genfs->next) {
		cmp = strcmp(fstype, genfs->fstype);
		if (cmp <= 0)
			break;
	}

	if (!genfs || cmp) {
		*sid = SECINITSID_UNLABELED;
		rc = -ENOENT;
		goto out;
	}

	for (c = genfs->head; c; c = c->next) {
		len = strlen(c->u.name);
		if ((!c->v.sclass || sclass == c->v.sclass) &&
		    (strncmp(c->u.name, path, len) == 0))
			break;
	}

	if (!c) {
		*sid = SECINITSID_UNLABELED;
		rc = -ENOENT;
		goto out;
	}

	if (!c->sid[0]) {
		rc = sidtab_context_to_sid(&sidtab,
					   &c->context[0],
					   &c->sid[0]);
		if (rc)
			goto out;
	}

	*sid = c->sid[0];
out:
	POLICY_RDUNLOCK;
	return rc;
}

int security_fs_use(
	const char *fstype,
	unsigned int *behavior,
	security_id_t *sid)
{
	int rc = 0;
	ocontext_t *c;

	POLICY_RDLOCK;

	c = policydb.ocontexts[OCON_FSUSE];
	while (c) {
		if (strcmp(fstype, c->u.name) == 0)
			break;
		c = c->next;
	}

	if (c) {
		*behavior = c->v.behavior;
		if (c->v.behavior != SECURITY_FS_USE_PSID) {
			if (!c->sid[0]) {
				rc = sidtab_context_to_sid(&sidtab,
							   &c->context[0],
							   &c->sid[0]);
				if (rc)
					goto out;
			}
			*sid = c->sid[0];
		}
	} else {
		rc = security_genfs_sid(fstype, "/", SECCLASS_DIR, sid);
		if (rc) {
			*behavior = SECURITY_FS_USE_NONE;
			rc = 0;
		} else {
			*behavior = SECURITY_FS_USE_GENFS;
		}
	}

      out:
	POLICY_RDUNLOCK;
	return rc;	
}


/* FLASK */

