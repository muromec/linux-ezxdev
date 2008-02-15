
/* Author : Stephen Smalley, <sds@epoch.ncsc.mil> */

/* FLASK */

/*
 * A security context is a set of security attributes
 * associated with each subject and object controlled
 * by the security policy.  Security contexts are
 * externally represented as variable-length strings
 * that can be interpreted by a user or application
 * with an understanding of the security policy. 
 * Internally, the security server uses a simple
 * structure.  This structure is private to the
 * security server and can be changed without affecting
 * clients of the security server.
 */

#ifndef _CONTEXT_H_
#define _CONTEXT_H_

#include "ebitmap.h"

#include "mls_types.h"

/*
 * A security context consists of an authenticated user
 * identity, a role, a type and a MLS range.
 */
typedef struct context_struct {
	__u32 user;
	__u32 role;
	__u32 type;
#ifdef CONFIG_SECURITY_SELINUX_MLS
	mls_range_t range;
#endif
} context_struct_t;


#ifdef CONFIG_SECURITY_SELINUX_MLS

#define mls_context_init(c) memset(c, 0, sizeof(mls_range_t))

static inline int mls_context_cpy(context_struct_t * dst, 
				  context_struct_t * src)
{
	(dst)->range.level[0].sens = (src)->range.level[0].sens;
	if (!ebitmap_cpy(&(dst)->range.level[0].cat, &(src)->range.level[0].cat))
		return -ENOMEM;
	(dst)->range.level[1].sens = (src)->range.level[1].sens;
	if (!ebitmap_cpy(&(dst)->range.level[1].cat, &(src)->range.level[1].cat)) {
		ebitmap_destroy(&(dst)->range.level[0].cat);
		return -ENOMEM;
	}
	return 0;
}

#define mls_context_cmp(c1,c2) \
(((c1)->range.level[0].sens == (c2)->range.level[0].sens) && \
 ebitmap_cmp(&(c1)->range.level[0].cat,&(c2)->range.level[0].cat) && \
 ((c1)->range.level[1].sens == (c2)->range.level[1].sens) && \
 ebitmap_cmp(&(c1)->range.level[1].cat,&(c2)->range.level[1].cat))

#define mls_context_destroy(c) \
do { \
	ebitmap_destroy(&(c)->range.level[0].cat); \
	ebitmap_destroy(&(c)->range.level[1].cat); \
        memset(c, 0, sizeof(mls_range_t)); \
} while (0)

#else

#define mls_context_init(c)
#define mls_context_cpy(dst,src) 0
#define mls_context_destroy(c)
#define mls_context_cmp(c1,c2) 1

#endif


#define context_init(c) memset(c, 0, sizeof(context_struct_t))

static inline int context_cpy(context_struct_t * dst,
			      context_struct_t * src)
{
	(dst)->user = (src)->user;
	(dst)->role = (src)->role;
	(dst)->type = (src)->type;
	return mls_context_cpy(dst, src);
}


#define context_destroy(c) \
do { \
	(c)->user = 0; \
	(c)->role = 0; \
	(c)->type = 0; \
	mls_context_destroy(c); \
} while (0)

static inline int context_cmp(context_struct_t * c1,
			      context_struct_t * c2)
{
	return (((c1)->user == (c2)->user) &&
		((c1)->role == (c2)->role) &&
		((c1)->type == (c2)->type) &&
		mls_context_cmp(c1, c2));
}

#endif	/* _CONTEXT_H_ */

/* FLASK */

