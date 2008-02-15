
/* Author : Stephen Smalley, <sds@epoch.ncsc.mil> */

/* FLASK */

/*
 * An access vector table (avtab) is a hash table
 * of access vectors and transition types indexed 
 * by a type pair and a class.  An access vector
 * table is used to represent the type enforcement
 * tables.
 */

#ifndef _AVTAB_H_
#define _AVTAB_H_

typedef struct avtab_key {
	__u32 source_type;	/* source type */
	__u32 target_type;	/* target type */
	__u32 target_class;     /* target object class */
} avtab_key_t;

typedef struct avtab_datum {
#define AVTAB_ALLOWED     1
#define AVTAB_AUDITALLOW  2
#define AVTAB_AUDITDENY   4
#define AVTAB_AV         (AVTAB_ALLOWED | AVTAB_AUDITALLOW | AVTAB_AUDITDENY)
#define AVTAB_TRANSITION 16
#define AVTAB_MEMBER     32
#define AVTAB_CHANGE     64
#define AVTAB_TYPE       (AVTAB_TRANSITION | AVTAB_MEMBER | AVTAB_CHANGE)
	__u32 specified;	/* what fields are specified */
        __u32 data[3];          /* access vectors or types */
#define avtab_allowed(x) (x)->data[0]
#define avtab_auditdeny(x) (x)->data[1]
#define avtab_auditallow(x) (x)->data[2]
#define avtab_transition(x) (x)->data[0]
#define avtab_change(x) (x)->data[1]
#define avtab_member(x) (x)->data[2]
} avtab_datum_t;

typedef struct avtab_node *avtab_ptr_t;

struct avtab_node {
	avtab_key_t key;
	avtab_datum_t datum;
	avtab_ptr_t next;
};

typedef struct avtab {
	avtab_ptr_t *htable;
	__u32 nel;	/* number of elements */
} avtab_t;

int avtab_init(avtab_t *);

int avtab_insert(avtab_t * h, avtab_key_t * k, avtab_datum_t * d);

avtab_datum_t *avtab_search(avtab_t * h, avtab_key_t * k, int specified);

void avtab_destroy(avtab_t * h);

int avtab_map(avtab_t * h,
	      int (*apply) (avtab_key_t * k,
			    avtab_datum_t * d,
			    void *args),
	      void *args);

void avtab_hash_eval(avtab_t * h, char *tag);

int avtab_read(avtab_t * a, FILE * fp, __u32 config);

#ifndef __KERNEL__
int avtab_write(avtab_t * a, FILE * fp);
#endif

#endif	/* _AVTAB_H_ */

/* FLASK */

