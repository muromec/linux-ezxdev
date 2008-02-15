#ifdef CONFIG_SECURITY_SELINUX_DEVELOP
int context_struct_to_string(context_struct_t * context,
			     security_context_t * scontext,
			     __u32 *scontext_len);

extern int selinux_enforcing;

static inline int compute_sid_handle_invalid_context(
	context_struct_t *scontext,
	context_struct_t *tcontext,
	security_class_t tclass,
	context_struct_t *newcontext)
{
	security_context_t s, t, n;
	__u32 slen, tlen, nlen;

	if (selinux_enforcing) {
		return -EACCES;
	} else {
		context_struct_to_string(scontext, &s, &slen);
		context_struct_to_string(tcontext, &t, &tlen);
		context_struct_to_string(newcontext, &n, &nlen);
		printf("security_compute_sid:  invalid context %s", n);
		printf(" for scontext=%s", s);
		printf(" tcontext=%s", t);
		printf(" tclass=%s\n", policydb.p_class_val_to_name[tclass-1]);
		free(s);
		free(t);
		free(n);
		return 0;
	} 
}

static inline int convert_context_handle_invalid_context(
	context_struct_t *context)
{
	security_context_t s;
	__u32 len;

	if (selinux_enforcing) {
		return -EINVAL;
	} else {
		context_struct_to_string(context, &s, &len);
		printf("security:  context %s is invalid\n", s);
		free(s);
		return 0;
	}
}

#else
#define compute_sid_handle_invalid_context(scontext,tcontext,tclass,newcontext) -EACCES
#define convert_context_handle_invalid_context(context) -EINVAL
#endif

#ifdef __KERNEL__
static rwlock_t policy_rwlock = RW_LOCK_UNLOCKED;
#define POLICY_RDLOCK read_lock(&policy_rwlock)
#define POLICY_WRLOCK write_lock_irq(&policy_rwlock)
#define POLICY_RDUNLOCK read_unlock(&policy_rwlock)
#define POLICY_WRUNLOCK write_unlock_irq(&policy_rwlock)
#else
#define POLICY_RDLOCK 
#define POLICY_WRLOCK
#define POLICY_RDUNLOCK
#define POLICY_WRUNLOCK
#endif

#ifdef __KERNEL__
static DECLARE_MUTEX(load_sem);
#define LOAD_LOCK down(&load_sem)
#define LOAD_UNLOCK up(&load_sem)
#else
#define LOAD_LOCK 
#define LOAD_UNLOCK
#endif
