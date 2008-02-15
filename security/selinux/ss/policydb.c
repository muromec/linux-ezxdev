
/* Author : Stephen Smalley, <sds@epoch.ncsc.mil> */

/* FLASK */

/*
 * Implementation of the policy database.
 */

#include "policydb.h"
#include "services.h"
#include "mls.h"

#if 0
static char *symtab_name[SYM_NUM] = {
	"common prefixes",
	"classes",
	"roles",
	"types",
	"users"
	mls_symtab_names
};
#endif

static unsigned int symtab_sizes[SYM_NUM] = {
	2,
	32,
	16,
	512,
	128
	mls_symtab_sizes
};

/* 
 * Initialize the role table.
 */
int roles_init(policydb_t *p)
{
	char *key = 0;
	role_datum_t *role;

	role = malloc_sleep(sizeof(role_datum_t));
	if (!role)
		return -1;
	memset(role, 0, sizeof(role_datum_t));
	role->value = ++p->p_roles.nprim;
	if (role->value != OBJECT_R_VAL)
		return -1;
	key = malloc_sleep(strlen(OBJECT_R)+1);
	if (!key)
		return -1;
	strcpy(key, OBJECT_R);

	if (hashtab_insert(p->p_roles.table, key, role))
		return -1;
	
	return 0;
}


/*
 * Initialize a policy database structure.
 */
int policydb_init(policydb_t * p)
{
	int i;

	memset(p, 0, sizeof(policydb_t));

	for (i = 0; i < SYM_NUM; i++) {
		if (symtab_init(&p->symtab[i], symtab_sizes[i]))
			return -1;
	}

	if (avtab_init(&p->te_avtab))
		return -1;

	if (roles_init(p))
		return -1;

	return 0;
}


/*
 * The following *_index functions are used to
 * define the val_to_name and val_to_struct arrays
 * in a policy database structure.  The val_to_name
 * arrays are used when converting security context
 * structures into string representations.  The
 * val_to_struct arrays are used when the attributes
 * of a class, role, or user are needed.
 */

static int common_index(hashtab_key_t key, hashtab_datum_t datum, void *datap)
{
	policydb_t *p;
	common_datum_t *comdatum;


	comdatum = (common_datum_t *) datum;
	p = (policydb_t *) datap;

	p->p_common_val_to_name[comdatum->value - 1] = (char *) key;

	return 0;
}


static int class_index(hashtab_key_t key, hashtab_datum_t datum, void *datap)
{
	policydb_t *p;
	class_datum_t *cladatum;


	cladatum = (class_datum_t *) datum;
	p = (policydb_t *) datap;

	p->p_class_val_to_name[cladatum->value - 1] = (char *) key;
	p->class_val_to_struct[cladatum->value - 1] = cladatum;

	return 0;
}


static int role_index(hashtab_key_t key, hashtab_datum_t datum, void *datap)
{
	policydb_t *p;
	role_datum_t *role;


	role = (role_datum_t *) datum;
	p = (policydb_t *) datap;

	p->p_role_val_to_name[role->value - 1] = (char *) key;
	p->role_val_to_struct[role->value - 1] = role;

	return 0;
}


static int type_index(hashtab_key_t key, hashtab_datum_t datum, void *datap)
{
	policydb_t *p;
	type_datum_t *typdatum;


	typdatum = (type_datum_t *) datum;
	p = (policydb_t *) datap;

	if (typdatum->primary)
		p->p_type_val_to_name[typdatum->value - 1] = (char *) key;

	return 0;
}

static int user_index(hashtab_key_t key, hashtab_datum_t datum, void *datap)
{
	policydb_t *p;
	user_datum_t *usrdatum;


	usrdatum = (user_datum_t *) datum;
	p = (policydb_t *) datap;

	p->p_user_val_to_name[usrdatum->value - 1] = (char *) key;
	p->user_val_to_struct[usrdatum->value - 1] = usrdatum;

	return 0;
}

static int (*index_f[SYM_NUM]) (hashtab_key_t key, hashtab_datum_t datum, void *datap) =
{
	common_index,
	class_index,
	role_index,
	type_index,
	user_index
	mls_index_f
};


/*
 * Define the common val_to_name array and the class
 * val_to_name and val_to_struct arrays in a policy
 * database structure.  
 */
int policydb_index_classes(policydb_t * p)
{
	p->p_common_val_to_name = (char **)
	    malloc_sleep(p->p_commons.nprim * sizeof(char *));
	if (!p->p_common_val_to_name)
		return -1;

	if (hashtab_map(p->p_commons.table, common_index, p))
		return -1;

	p->class_val_to_struct = (class_datum_t **)
	    malloc_sleep(p->p_classes.nprim * sizeof(class_datum_t *));
	if (!p->class_val_to_struct)
		return -1;

	p->p_class_val_to_name = (char **)
	    malloc_sleep(p->p_classes.nprim * sizeof(char *));
	if (!p->p_class_val_to_name)
		return -1;

	if (hashtab_map(p->p_classes.table, class_index, p))
		return -1;
	return 0;
}


/*
 * Define the other val_to_name and val_to_struct arrays
 * in a policy database structure.  
 */
int policydb_index_others(policydb_t * p)
{
	int i;


	printf("security:  %d users, %d roles, %d types",
	       p->p_users.nprim, p->p_roles.nprim, p->p_types.nprim);
	mls_policydb_index_others(p);
	printf("\n");

	printf("security:  %d classes, %d rules\n",
	       p->p_classes.nprim, p->te_avtab.nel);

#if 0
	avtab_hash_eval(&p->te_avtab, "rules");
	for (i = 0; i < SYM_NUM; i++) 
		hashtab_hash_eval(p->symtab[i].table, symtab_name[i]);
#endif

	p->role_val_to_struct = (role_datum_t **)
	    malloc_sleep(p->p_roles.nprim * sizeof(role_datum_t *));
	if (!p->role_val_to_struct)
		return -1;

	p->user_val_to_struct = (user_datum_t **)
	    malloc_sleep(p->p_users.nprim * sizeof(user_datum_t *));
	if (!p->user_val_to_struct)
		return -1;

	for (i = SYM_ROLES; i < SYM_NUM; i++) {
		p->sym_val_to_name[i] = (char **)
		    malloc_sleep(p->symtab[i].nprim * sizeof(char *));
		if (!p->sym_val_to_name[i])
			return -1;
		if (hashtab_map(p->symtab[i].table, index_f[i], p))
			return -1;
	}

	return 0;
}


/*
 * The following *_destroy functions are used to
 * free any memory allocated for each kind of
 * symbol data in the policy database.
 */

static int perm_destroy(hashtab_key_t key, hashtab_datum_t datum, void *p)
{
	if (key)
		free(key);
	free(datum);
	return 0;
}


static int common_destroy(hashtab_key_t key, hashtab_datum_t datum, void *p)
{
	common_datum_t *comdatum;

	if (key)
		free(key);
	comdatum = (common_datum_t *) datum;
	hashtab_map(comdatum->permissions.table, perm_destroy, 0);
	hashtab_destroy(comdatum->permissions.table);
	free(datum);
	return 0;
}


int constraint_expr_destroy(constraint_expr_t * expr)
{
	expr->count--;
	if (expr->count == 0) {
		ebitmap_destroy(&expr->names);
		if (expr->left)
			constraint_expr_destroy(expr->left);
		if (expr->right)
			constraint_expr_destroy(expr->right);
		free(expr);
	}
	return 0;
}


static int class_destroy(hashtab_key_t key, hashtab_datum_t datum, void *p)
{
	class_datum_t *cladatum;
	constraint_node_t *constraint, *ctemp;

	if (key)
		free(key);
	cladatum = (class_datum_t *) datum;
	hashtab_map(cladatum->permissions.table, perm_destroy, 0);
	hashtab_destroy(cladatum->permissions.table);
	constraint = cladatum->constraints;
	while (constraint) {
		constraint_expr_destroy(constraint->expr);
		ctemp = constraint;
		constraint = constraint->next;
		free(ctemp);
	}
	if (cladatum->comkey)
		free(cladatum->comkey);
	free(datum);
	return 0;
}

static int role_destroy(hashtab_key_t key, hashtab_datum_t datum, void *p)
{
	role_datum_t *role;

	if (key)
		free(key);
	role = (role_datum_t *) datum;
	ebitmap_destroy(&role->dominates);
	ebitmap_destroy(&role->types);
	free(datum);
	return 0;
}

static int type_destroy(hashtab_key_t key, hashtab_datum_t datum, void *p)
{
	if (key)
		free(key);
	free(datum);
	return 0;
}

static int user_destroy(hashtab_key_t key, hashtab_datum_t datum, void *p)
{
	user_datum_t *usrdatum;

	if (key)
		free(key);
	usrdatum = (user_datum_t *) datum;
	ebitmap_destroy(&usrdatum->roles);
	mls_user_destroy(usrdatum);
	free(datum);
	return 0;
}


static int (*destroy_f[SYM_NUM]) (hashtab_key_t key, hashtab_datum_t datum, void *datap) =
{
	common_destroy,
	class_destroy,
	role_destroy,
	type_destroy,
	user_destroy
	mls_destroy_f
};


/*
 * Free any memory allocated by a policy database structure.
 */
void policydb_destroy(policydb_t * p)
{
	ocontext_t *c, *ctmp;
	genfs_t *g, *gtmp;
	int i;

	for (i = 0; i < SYM_NUM; i++) {
		hashtab_map(p->symtab[i].table, destroy_f[i], 0);
		hashtab_destroy(p->symtab[i].table);
	}

	for (i = 0; i < SYM_NUM; i++) {
		if (p->sym_val_to_name[i])
			free(p->sym_val_to_name[i]);
	}

	if (p->class_val_to_struct)
		free(p->class_val_to_struct);
	if (p->role_val_to_struct)
		free(p->role_val_to_struct);
	if (p->user_val_to_struct)
		free(p->user_val_to_struct);

	avtab_destroy(&p->te_avtab);

	for (i = 0; i < OCON_NUM; i++) {
		c = p->ocontexts[i];
		while (c) {
			ctmp = c;
			c = c->next;
			context_destroy(&ctmp->context[0]);
			context_destroy(&ctmp->context[1]);
			if (i == OCON_ISID || i == OCON_FS || i == OCON_NETIF || i == OCON_FSUSE)
				free(ctmp->u.name);
			free(ctmp);
		}
	}

	g = p->genfs;
	while (g) {
		free(g->fstype);
		c = g->head;
		while (c) {
			ctmp = c;
			c = c->next;
			context_destroy(&ctmp->context[0]);
			free(ctmp->u.name);
			free(ctmp);
		}
		gtmp = g;
		g = g->next;
		free(gtmp);
	}

	return;
}


/*
 * Load the initial SIDs specified in a policy database
 * structure into a SID table.
 */
int policydb_load_isids(policydb_t *p, sidtab_t *s) 
{
	ocontext_t *head, *c;

	if (sidtab_init(s)) {
		printf("security:  out of memory on SID table init\n");
		return -1;
	}

	head = p->ocontexts[OCON_ISID];
	for (c = head; c; c = c->next) {
		if (!c->context[0].user) {
			printf("security:  SID %s was never defined.\n", 
			       c->u.name);
			return -1;
		}
		if (sidtab_insert(s, c->sid[0], &c->context[0])) {
			printf("security:  unable to load initial SID %s.\n", 
			       c->u.name);
			return -1;
		}
	}

	return 0;
}


/*
 * Return TRUE if the fields in the security context 
 * structure `c' are valid.  Return FALSE otherwise.
 */
int policydb_context_isvalid(policydb_t *p, context_struct_t *c)
{
	role_datum_t *role;
	user_datum_t *usrdatum;


	/*
	 * Role must be authorized for the type.
	 */
	if (!c->role || c->role > p->p_roles.nprim)
		return FALSE;

	if (c->role != OBJECT_R_VAL) {
		role = p->role_val_to_struct[c->role - 1];
		if (!ebitmap_get_bit(&role->types,
				     c->type - 1))
			/* role may not be associated with type */
			return FALSE;
		
		/*
		 * User must be authorized for the role.
		 */
		if (!c->user || c->user > p->p_users.nprim)
			return FALSE;
		usrdatum = p->user_val_to_struct[c->user - 1];
		if (!usrdatum)
			return FALSE;

		if (!ebitmap_get_bit(&usrdatum->roles,
				     c->role - 1))
			/* user may not be associated with role */
			return FALSE;
	}

	if (!mls_context_isvalid(p, c))
		return FALSE;

	return TRUE;
}


/*
 * Read and validate a security context structure
 * from a policydb binary representation file.
 */
static int context_read_and_validate(context_struct_t * c,
					policydb_t * p,
					FILE * fp)
{
	__u32 buf[32];
	size_t items;

	items = fread(buf, sizeof(__u32), 3, fp);
	if (items != 3) {
		printf("security: context truncated\n");
		return -1;
	}
	c->user = le32_to_cpu(buf[0]);
	c->role = le32_to_cpu(buf[1]);
	c->type = le32_to_cpu(buf[2]);
	if (mls_read_range(c, fp)) {
		printf("security: error reading MLS range of context\n");
		return -1;
	}

	if (!policydb_context_isvalid(p, c)) {
		printf("security:  invalid security context\n");
		context_destroy(c);
		return -1;
	}
	return 0;
}


/*
 * The following *_read functions are used to
 * read the symbol data from a policy database
 * binary representation file.
 */

static int perm_read(policydb_t * p, hashtab_t h, FILE * fp)
{
	char *key = 0;
	perm_datum_t *perdatum;
	__u32 buf[32], len;
	int items, items2;

	perdatum = malloc_sleep(sizeof(perm_datum_t));
	if (!perdatum)
		return -1;
	memset(perdatum, 0, sizeof(perm_datum_t));

	items = 2;
	items2 = fread(buf, sizeof(__u32), items, fp);
	if (items != items2)
		goto bad;

	len = le32_to_cpu(buf[0]);
	perdatum->value = le32_to_cpu(buf[1]);
	if (mls_read_perm(perdatum, fp))
		goto bad;

	key = malloc_sleep(len + 1);
	if (!key)
		goto bad;
	items = fread(key, 1, len, fp);
	if (items != len)
		goto bad;
	key[len] = 0;

	if (hashtab_insert(h, key, perdatum))
		goto bad;

	return 0;

      bad:
	perm_destroy(key, perdatum, NULL);
	return -1;
}


static int common_read(policydb_t * p, hashtab_t h, FILE * fp)
{
	char *key = 0;
	common_datum_t *comdatum;
	__u32 buf[32], len, nel;
	int items, i;

	comdatum = malloc_sleep(sizeof(common_datum_t));
	if (!comdatum)
		return -1;
	memset(comdatum, 0, sizeof(common_datum_t));

	items = fread(buf, sizeof(__u32), 4, fp);
	if (items != 4)
		goto bad;

	len = le32_to_cpu(buf[0]);
	comdatum->value = le32_to_cpu(buf[1]);

	if (symtab_init(&comdatum->permissions, PERM_SYMTAB_SIZE))
		goto bad;
	comdatum->permissions.nprim = le32_to_cpu(buf[2]);
	nel = le32_to_cpu(buf[3]);

	key = malloc_sleep(len + 1);
	if (!key)
		goto bad;
	items = fread(key, 1, len, fp);
	if (items != len)
		goto bad;
	key[len] = 0;

	for (i = 0; i < nel; i++) {
		if (perm_read(p, comdatum->permissions.table, fp))
			goto bad;
	}

	if (hashtab_insert(h, key, comdatum))
		goto bad;

	return 0;

      bad:
	common_destroy(key, comdatum, NULL);
	return -1;
}


static constraint_expr_t *
 constraint_expr_read(FILE * fp)
{
	constraint_expr_t *expr;
	__u32 buf[32];
	int items;

	expr = malloc_sleep(sizeof(constraint_expr_t));
	if (!expr)
		return NULL;
	memset(expr, 0, sizeof(constraint_expr_t));

	items = fread(buf, sizeof(__u32), 3, fp);
	if (items != 3)
		goto bad;

	expr->expr_type = le32_to_cpu(buf[0]);
	expr->attr = le32_to_cpu(buf[1]);
	expr->op = le32_to_cpu(buf[2]);
	expr->count = 1;
	items = 0;

	switch (expr->expr_type) {
	case CEXPR_NAMES:
		if (!ebitmap_read(&expr->names, fp))
			goto bad;
		break;
	case CEXPR_AND:
	case CEXPR_OR:
		expr->left = constraint_expr_read(fp);
		if (!expr->left)
			goto bad;
		expr->right = constraint_expr_read(fp);
		if (!expr->right)
			goto bad;
		break;
	case CEXPR_NOT:
		expr->left = constraint_expr_read(fp);
		if (!expr->left)
			goto bad;
		break;
	}

	return expr;

      bad:
	constraint_expr_destroy(expr);
	return NULL;
}


static int class_read(policydb_t * p, hashtab_t h, FILE * fp)
{
	char *key = 0;
	class_datum_t *cladatum;
	constraint_node_t *c, *l;
	__u32 buf[32], len, len2, ncons, nel;
	int items, i;

	cladatum = (class_datum_t *) malloc_sleep(sizeof(class_datum_t));
	if (!cladatum)
		return -1;
	memset(cladatum, 0, sizeof(class_datum_t));

	items = fread(buf, sizeof(__u32), 6, fp);
	if (items != 6)
		goto bad;

	len = le32_to_cpu(buf[0]);
	len2 = le32_to_cpu(buf[1]);
	cladatum->value = le32_to_cpu(buf[2]);

	if (symtab_init(&cladatum->permissions, PERM_SYMTAB_SIZE))
		goto bad;
	cladatum->permissions.nprim = le32_to_cpu(buf[3]);
	nel = le32_to_cpu(buf[4]);

	ncons = le32_to_cpu(buf[5]);

	key = malloc_sleep(len + 1);
	if (!key)
		goto bad;
	items = fread(key, 1, len, fp);
	if (items != len)
		goto bad;
	key[len] = 0;

	if (len2) {
		cladatum->comkey = malloc_sleep(len2 + 1);
		if (!cladatum->comkey)
			goto bad;
		items = fread(cladatum->comkey, 1, len2, fp);
		if (items != len2)
			goto bad;
		cladatum->comkey[len2] = 0;

		cladatum->comdatum = hashtab_search(p->p_commons.table,
						    cladatum->comkey);
		if (!cladatum->comdatum) {
			printf("security:  unknown common %s\n", cladatum->comkey);
			goto bad;
		}
	}
	for (i = 0; i < nel; i++) {
		if (perm_read(p, cladatum->permissions.table, fp))
			goto bad;
	}

	l = NULL;
	for (i = 0; i < ncons; i++) {
		c = malloc_sleep(sizeof(constraint_node_t));
		if (!c)
			goto bad;
		memset(c, 0, sizeof(constraint_node_t));
		items = fread(buf, sizeof(__u32), 1, fp);
		if (items != 1)
			goto bad;
		c->permissions = le32_to_cpu(buf[0]);
		c->expr = constraint_expr_read(fp);
		if (!c->expr)
			goto bad;
		if (l) {
			l->next = c;
		} else {
			cladatum->constraints = c;
		}
		l = c;

	}

	if (mls_read_class(cladatum, fp))
		goto bad;

	if (hashtab_insert(h, key, cladatum))
		goto bad;

	return 0;

      bad:
	class_destroy(key, cladatum, NULL);
	return -1;
}


static int role_read(policydb_t * p, hashtab_t h, FILE * fp)
{
	char *key = 0;
	role_datum_t *role;
	__u32 buf[32], len;
	int items;

	role = malloc_sleep(sizeof(role_datum_t));
	if (!role)
		return -1;
	memset(role, 0, sizeof(role_datum_t));

	items = fread(buf, sizeof(__u32), 2, fp);
	if (items != 2)
		goto bad;

	len = le32_to_cpu(buf[0]);
	role->value = le32_to_cpu(buf[1]);

	key = malloc_sleep(len + 1);
	if (!key)
		goto bad;
	items = fread(key, 1, len, fp);
	if (items != len)
		goto bad;
	key[len] = 0;

	if (!ebitmap_read(&role->dominates, fp))
		goto bad;

	if (!ebitmap_read(&role->types, fp))
		goto bad;

	if (strcmp(key, OBJECT_R) == 0) {
		if (role->value != OBJECT_R_VAL) {
			printf("Role %s has wrong value %d\n",
			       OBJECT_R, role->value);
			role_destroy(key, role, NULL);
			return -1;
		}
		role_destroy(key, role, NULL);
		return 0;
	}

	if (hashtab_insert(h, key, role))
		goto bad;

	return 0;

      bad:
	role_destroy(key, role, NULL);
	return -1;
}


static int type_read(policydb_t * p, hashtab_t h, FILE * fp)
{
	char *key = 0;
	type_datum_t *typdatum;
	__u32 buf[32], len;
	int items;

	typdatum = malloc_sleep(sizeof(type_datum_t));
	if (!typdatum)
		return -1;
	memset(typdatum, 0, sizeof(type_datum_t));

	items = fread(buf, sizeof(__u32), 3, fp);
	if (items != 3)
		goto bad;

	len = le32_to_cpu(buf[0]);
	typdatum->value = le32_to_cpu(buf[1]);
	typdatum->primary = le32_to_cpu(buf[2]);

	key = malloc_sleep(len + 1);
	if (!key)
		goto bad;
	items = fread(key, 1, len, fp);
	if (items != len)
		goto bad;
	key[len] = 0;

	if (hashtab_insert(h, key, typdatum))
		goto bad;

	return 0;

      bad:
	type_destroy(key, typdatum, NULL);
	return -1;
}

static int user_read(policydb_t * p, hashtab_t h, FILE * fp)
{
	char *key = 0;
	user_datum_t *usrdatum;
	__u32 buf[32], len;
	int items;


	usrdatum = malloc_sleep(sizeof(user_datum_t));
	if (!usrdatum)
		return -1;
	memset(usrdatum, 0, sizeof(user_datum_t));

	items = fread(buf, sizeof(__u32), 2, fp);
	if (items != 2)
		goto bad;

	len = le32_to_cpu(buf[0]);
	usrdatum->value = le32_to_cpu(buf[1]);

	key = malloc_sleep(len + 1);
	if (!key)
		goto bad;
	items = fread(key, 1, len, fp);
	if (items != len)
		goto bad;
	key[len] = 0;

	if (!ebitmap_read(&usrdatum->roles, fp))
		goto bad;

	if (mls_read_user(usrdatum, fp))
		goto bad;

	if (hashtab_insert(h, key, usrdatum))
		goto bad;

	return 0;

      bad:
	user_destroy(key, usrdatum, NULL);
	return -1;
}


static int (*read_f[SYM_NUM]) (policydb_t * p, hashtab_t h, FILE * fp) =
{
	common_read,
	class_read,
	role_read,
	type_read,
	user_read
	mls_read_f
};

#define mls_config(x) \
       ((x) & POLICYDB_CONFIG_MLS) ? "mls" : "no_mls"

/*
 * Read the configuration data from a policy database binary
 * representation file into a policy database structure.
 */

#ifndef __KERNEL__
int policydb_inflate(FILE *fp)
{
	unsigned char buf[2];
	size_t items;

	items = fread(buf, 1, 2, fp);
	if (items != 2)
		return -1;

	if (buf[0] != 037 || ((buf[1] != 0213) && (buf[1] != 0236))) {
		/* Not compressed. */
		rewind(fp);
		return 0;
	}

	fprintf(stderr, "checkpolicy:  no support yet for loading a gzipped policy directly into checkpolicy (just into the kernel).\n");
	
	return -1;
}

void policydb_inflate_release(FILE *fp)
{
	return;
}
#else
extern int policydb_inflate(FILE * fp); 
extern void policydb_inflate_release(struct file *fp);
#endif

int policydb_read(policydb_t * p, FILE * fp)
{
	struct role_allow *ra, *lra;
	struct role_trans *tr, *ltr;
	ocontext_t *l, *c, *newc;
	genfs_t *genfs_p, *genfs, *newgenfs;
	int i, j;
	__u32 buf[32], len, len2, config, nprim, nel, nel2;
	size_t items;
	char *policydb_str;

	config = 0;
	mls_set_config(config);

	if (policydb_init(p)) 
		return -1;

	/* Inflate the policydb, if compressed. */
	if (policydb_inflate(fp)) {
		policydb_destroy(p);
		return -1;
	}

	/* Read the magic number and string length. */
	items = fread(buf, sizeof(__u32), 2, fp);
	if (items != 2) 
		goto bad;
	for (i = 0; i < 2; i++)
		buf[i] = le32_to_cpu(buf[i]);

	if (buf[0] != POLICYDB_MAGIC) {
		printf("security:  policydb magic number 0x%x does not match expected magic number 0x%x\n", buf[0], POLICYDB_MAGIC);
		goto bad;
	}

	len = buf[1];
	if (len != strlen(POLICYDB_STRING)) {
		printf("security:  policydb string length %d does not match expected length %d\n", len, strlen(POLICYDB_STRING));
		goto bad;
	}
	policydb_str = malloc_sleep(len + 1);
	if (!policydb_str) {
		printf("security:  unable to allocate memory for policydb string of length %d\n", len);
		goto bad;
	}
	items = fread(policydb_str, 1, len, fp);
	if (items != len) {
		printf("security:  truncated policydb string identifier at %d\n", items);
		free(policydb_str);
		goto bad;
	}
	policydb_str[len] = 0;
	if (strcmp(policydb_str, POLICYDB_STRING)) {
		printf("security:  policydb string %s does not match my string %s\n", policydb_str, POLICYDB_STRING);
		free(policydb_str);
		goto bad;
	}
	/* Done with policydb_str. */
	free(policydb_str);
	policydb_str = NULL;

	/* Read the version, config, and table sizes. */
	items = fread(buf, sizeof(__u32), 4, fp);
	if (items != 4) 
		goto bad;
	for (i = 0; i < 4; i++)
		buf[i] = le32_to_cpu(buf[i]);

	if (buf[0] != POLICYDB_VERSION) {
		printf("security:  policydb version %d does not match my version %d\n", buf[0], POLICYDB_VERSION);
		goto bad;
	}
	if (buf[1] != config) {
		printf("security:  policydb configuration (%s) does not match my configuration (%s)\n",
		       mls_config(buf[1]),
		       mls_config(config));
		goto bad;
	}
	if (buf[2] != SYM_NUM || buf[3] != OCON_NUM) {
		printf("security:  policydb table sizes (%d,%d) do not match mine (%d,%d)\n", buf[2], buf[3], SYM_NUM, OCON_NUM);
		goto bad;
	}

	if (mls_read_nlevels(p, fp)) 
		goto bad;

	for (i = 0; i < SYM_NUM; i++) {
		items = fread(buf, sizeof(__u32), 2, fp);
		if (items != 2)
			goto bad;
		nprim = le32_to_cpu(buf[0]);
		nel = le32_to_cpu(buf[1]);
		for (j = 0; j < nel; j++) {
			if (read_f[i] (p, p->symtab[i].table, fp))
				goto bad;
		}

		p->symtab[i].nprim = nprim;
	}

	if (avtab_read(&p->te_avtab, fp, config))
		goto bad;

	items = fread(buf, sizeof(__u32), 1, fp);
	if (items != 1)
		goto bad;
	nel = le32_to_cpu(buf[0]);
	ltr = NULL;
	for (i = 0; i < nel; i++) {
		tr = malloc_sleep(sizeof(struct role_trans));
		if (!tr) {
			goto bad;
		}
		memset(tr, 0, sizeof(struct role_trans));
		if (ltr) {
			ltr->next = tr;
		} else {
			p->role_tr = tr;
		}
		items = fread(buf, sizeof(__u32), 3, fp);
		if (items != 3)
			goto bad;
		tr->role = le32_to_cpu(buf[0]);
		tr->type = le32_to_cpu(buf[1]);
		tr->new_role = le32_to_cpu(buf[2]);
		ltr = tr;
	}

	items = fread(buf, sizeof(__u32), 1, fp);
	if (items != 1)
		goto bad;
	nel = le32_to_cpu(buf[0]);
	lra = NULL;
	for (i = 0; i < nel; i++) {
		ra = malloc_sleep(sizeof(struct role_allow));
		if (!ra) {
			goto bad;
		}
		memset(ra, 0, sizeof(struct role_allow));
		if (lra) {
			lra->next = ra;
		} else {
			p->role_allow = ra;
		}
		items = fread(buf, sizeof(__u32), 2, fp);
		if (items != 2)
			goto bad;
		ra->role = le32_to_cpu(buf[0]);
		ra->new_role = le32_to_cpu(buf[1]);
		lra = ra;
	}

	if (policydb_index_classes(p))
		goto bad;

	if (policydb_index_others(p))
		goto bad;

	for (i = 0; i < OCON_NUM; i++) {
		items = fread(buf, sizeof(__u32), 1, fp);
		if (items != 1)
			goto bad;
		nel = le32_to_cpu(buf[0]);
		l = NULL;
		for (j = 0; j < nel; j++) {
			c = malloc_sleep(sizeof(ocontext_t));
			if (!c) {
				goto bad;
			}
			memset(c, 0, sizeof(ocontext_t));
			if (l) {
				l->next = c;
			} else {
				p->ocontexts[i] = c;
			}
			l = c;
			switch (i) {
			case OCON_ISID:
				items = fread(buf, sizeof(__u32), 1, fp);
				if (items != 1)
					goto bad;
				c->sid[0] = le32_to_cpu(buf[0]);
				if (context_read_and_validate(&c->context[0], p, fp))
					goto bad;
				break;
			case OCON_FS:
			case OCON_NETIF:
				items = fread(buf, sizeof(__u32), 1, fp);
				if (items != 1)
					goto bad;
				len = le32_to_cpu(buf[0]);
				c->u.name = malloc_sleep(len + 1);
				if (!c->u.name) {
					goto bad;
				}
				items = fread(c->u.name, 1, len, fp);
				if (items != len)
					goto bad;
				c->u.name[len] = 0;
				if (context_read_and_validate(&c->context[0], p, fp))
					goto bad;
				if (context_read_and_validate(&c->context[1], p, fp))
					goto bad;
				break;
			case OCON_PORT:
				items = fread(buf, sizeof(__u32), 3, fp);
				if (items != 3)
					goto bad;
				c->u.port.protocol = le32_to_cpu(buf[0]);
				c->u.port.low_port = le32_to_cpu(buf[1]);
				c->u.port.high_port = le32_to_cpu(buf[2]);
				if (context_read_and_validate(&c->context[0], p, fp))
					goto bad;
				break;
			case OCON_NODE:
				items = fread(buf, sizeof(__u32), 2, fp);
				if (items != 2)
					goto bad;
				c->u.node.addr = le32_to_cpu(buf[0]);
				c->u.node.mask = le32_to_cpu(buf[1]);
				if (context_read_and_validate(&c->context[0], p, fp))
					goto bad;
				break;
			case OCON_FSUSE:
				items = fread(buf, sizeof(__u32), 2, fp);
				if (items != 2)
					goto bad;
				c->v.behavior = le32_to_cpu(buf[0]);
				len = le32_to_cpu(buf[1]);
				c->u.name = malloc_sleep(len + 1);
				if (!c->u.name) {
					goto bad;
				}
				items = fread(c->u.name, 1, len, fp);
				if (items != len)
					goto bad;
				c->u.name[len] = 0;
				if (c->v.behavior != SECURITY_FS_USE_PSID) {
					if (context_read_and_validate(&c->context[0], p, fp))
						goto bad;
				}
				break;
			}
		}
	}

	items = fread(buf, sizeof(__u32), 1, fp);
	if (items != 1)
		goto bad;
	nel = le32_to_cpu(buf[0]);
	genfs_p = NULL;
	for (i = 0; i < nel; i++) {
		newgenfs = malloc_sleep(sizeof(genfs_t));
		if (!newgenfs) {
			goto bad;
		}
		memset(newgenfs, 0, sizeof(genfs_t));
		items = fread(buf, sizeof(__u32), 1, fp);
		if (items != 1)
			goto bad;
		len = le32_to_cpu(buf[0]);
		newgenfs->fstype = malloc_sleep(len + 1);
		if (!newgenfs->fstype) {
			goto bad;
		}
		items = fread(newgenfs->fstype, 1, len, fp);
		if (items != len)
			goto bad;
		newgenfs->fstype[len] = 0;
		for (genfs_p = NULL, genfs = p->genfs; genfs; 
		     genfs_p = genfs, genfs = genfs->next) {
			if (strcmp(newgenfs->fstype, genfs->fstype) == 0) {
				printf("security:  dup genfs fstype %s\n", newgenfs->fstype);
				goto bad;
			}
			if (strcmp(newgenfs->fstype, genfs->fstype) < 0)
				break;
		}
		newgenfs->next = genfs;
		if (genfs_p)
			genfs_p->next = newgenfs;
		else
			p->genfs = newgenfs;
		items = fread(buf, sizeof(__u32), 1, fp);
		if (items != 1)
			goto bad;
		nel2 = le32_to_cpu(buf[0]);
		for (j = 0; j < nel2; j++) {
			newc = malloc_sleep(sizeof(ocontext_t));
			if (!newc) {
				goto bad;
			}
			memset(newc, 0, sizeof(ocontext_t));
			items = fread(buf, sizeof(__u32), 1, fp);
			if (items != 1)
				goto bad;
			len = le32_to_cpu(buf[0]);
			newc->u.name = malloc_sleep(len + 1);
			if (!newc->u.name) {
				goto bad;
			}
			items = fread(newc->u.name, 1, len, fp);
			if (items != len)
				goto bad;
			newc->u.name[len] = 0;
			items = fread(buf, sizeof(__u32), 1, fp);
			if (items != 1)
				goto bad;
			newc->v.sclass = le32_to_cpu(buf[0]);
			if (context_read_and_validate(&newc->context[0], p, fp))
				goto bad;
			for (l = NULL, c = newgenfs->head; c; 
			     l = c, c = c->next) {
				if (!strcmp(newc->u.name, c->u.name) &&
				    (!c->v.sclass || !newc->v.sclass || newc->v.sclass == c->v.sclass)) {
					printf("security:  dup genfs entry (%s,%s)\n", newgenfs->fstype, c->u.name);
					goto bad;
				}
				len = strlen(newc->u.name);
				len2 = strlen(c->u.name);
				if (len > len2)
					break;
			}
			newc->next = c;
			if (l)
				l->next = newc;
			else
				newgenfs->head = newc;
		}
	}

	if (mls_read_trusted(p, fp))
		goto bad;

	policydb_inflate_release(fp);

	return 0;
bad:
	policydb_inflate_release(fp);
	policydb_destroy(p);
	return -1;
}


#ifndef __KERNEL__

/*
 * Write a security context structure
 * to a policydb binary representation file.
 */
static int context_write(context_struct_t * c, FILE * fp)
{
	__u32 buf[32];
	size_t items, items2;

	items = 0;
	buf[items++] = cpu_to_le32(c->user);
	buf[items++] = cpu_to_le32(c->role);
	buf[items++] = cpu_to_le32(c->type);
	items2 = fwrite(buf, sizeof(__u32), items, fp);
	if (items2 != items)
		return -1;
	if (mls_write_range(c, fp))
		return -1;

	return 0;
}


/*
 * The following *_write functions are used to
 * write the symbol data to a policy database
 * binary representation file.
 */

static int perm_write(hashtab_key_t key, hashtab_datum_t datum, void *p)
{
	perm_datum_t *perdatum;
	__u32 buf[32], len;
	int items, items2;
	FILE *fp = p;

	perdatum = (perm_datum_t *) datum;

	len = strlen(key);
	items = 0;
	buf[items++] = cpu_to_le32(len);
	buf[items++] = cpu_to_le32(perdatum->value);
	mls_write_perm(buf, items, perdatum);
	items2 = fwrite(buf, sizeof(__u32), items, fp);
	if (items != items2)
		return -1;

	items = fwrite(key, 1, len, fp);
	if (items != len)
		return -1;

	return 0;
}


static int common_write(hashtab_key_t key, hashtab_datum_t datum, void *p)
{
	common_datum_t *comdatum;
	__u32 buf[32], len;
	int items, items2;
	FILE *fp = p;

	comdatum = (common_datum_t *) datum;

	len = strlen(key);
	items = 0;
	buf[items++] = cpu_to_le32(len);
	buf[items++] = cpu_to_le32(comdatum->value);
	buf[items++] = cpu_to_le32(comdatum->permissions.nprim);
	buf[items++] = cpu_to_le32(comdatum->permissions.table->nel);
	items2 = fwrite(buf, sizeof(__u32), items, fp);
	if (items != items2)
		return -1;

	items = fwrite(key, 1, len, fp);
	if (items != len)
		return -1;

	if (hashtab_map(comdatum->permissions.table, perm_write, fp))
		return -1;

	return 0;
}


static int constraint_expr_write(constraint_expr_t * expr, FILE * fp)
{
	__u32 buf[32];
	int items, items2;

	items = 0;
	buf[items++] = cpu_to_le32(expr->expr_type);
	buf[items++] = cpu_to_le32(expr->attr);
	buf[items++] = cpu_to_le32(expr->op);
	items2 = fwrite(buf, sizeof(__u32), items, fp);
	if (items != items2)
		return -1;

	switch (expr->expr_type) {
	case CEXPR_NAMES:
		if (!ebitmap_write(&expr->names, fp))
			return -1;
		break;
	default:
		break;
	}

	if (expr->left) {
		if (constraint_expr_write(expr->left, fp))
			return -1;
	}
	if (expr->right) {
		if (constraint_expr_write(expr->right, fp))
			return -1;
	}
	return 0;
}


static int class_write(hashtab_key_t key, hashtab_datum_t datum, void *p)
{
	class_datum_t *cladatum;
	constraint_node_t *c;
	__u32 buf[32], len, len2, ncons;
	int items, items2;
	FILE *fp = p;

	cladatum = (class_datum_t *) datum;

	len = strlen(key);
	if (cladatum->comkey)
		len2 = strlen(cladatum->comkey);
	else
		len2 = 0;

	ncons = 0;
	for (c = cladatum->constraints; c; c = c->next) {
		ncons++;
	}

	items = 0;
	buf[items++] = cpu_to_le32(len);
	buf[items++] = cpu_to_le32(len2);
	buf[items++] = cpu_to_le32(cladatum->value);
	buf[items++] = cpu_to_le32(cladatum->permissions.nprim);
	if (cladatum->permissions.table) 
		buf[items++] = cpu_to_le32(cladatum->permissions.table->nel);
	else
		buf[items++] = 0;
	buf[items++] = cpu_to_le32(ncons);
	items2 = fwrite(buf, sizeof(__u32), items, fp);
	if (items != items2)
		return -1;

	items = fwrite(key, 1, len, fp);
	if (items != len)
		return -1;

	if (cladatum->comkey) {
		items = fwrite(cladatum->comkey, 1, len2, fp);
		if (items != len2)
			return -1;
	}
	if (hashtab_map(cladatum->permissions.table, perm_write, fp))
		return -1;

	for (c = cladatum->constraints; c; c = c->next) {
		buf[0] = cpu_to_le32(c->permissions);
		items = fwrite(buf, sizeof(__u32), 1, fp);
		if (items != 1)
			return -1;
		if (constraint_expr_write(c->expr, fp))
			return -1;
	}

	if (mls_write_class(cladatum, fp))
		return -1;

	return 0;
}

static int role_write(hashtab_key_t key, hashtab_datum_t datum, void *p)
{
	role_datum_t *role;
	__u32 buf[32], len;
	int items, items2;
	FILE *fp = p;

	role = (role_datum_t *) datum;

	len = strlen(key);
	items = 0;
	buf[items++] = cpu_to_le32(len);
	buf[items++] = cpu_to_le32(role->value);
	items2 = fwrite(buf, sizeof(__u32), items, fp);
	if (items != items2)
		return -1;

	items = fwrite(key, 1, len, fp);
	if (items != len)
		return -1;

	if (!ebitmap_write(&role->dominates, fp))
		return -1;

	if (!ebitmap_write(&role->types, fp))
		return -1;

	return 0;
}

static int type_write(hashtab_key_t key, hashtab_datum_t datum, void *p)
{
	type_datum_t *typdatum;
	__u32 buf[32], len;
	int items, items2;
	FILE *fp = p;

	typdatum = (type_datum_t *) datum;

	len = strlen(key);
	items = 0;
	buf[items++] = cpu_to_le32(len);
	buf[items++] = cpu_to_le32(typdatum->value);
	buf[items++] = cpu_to_le32(typdatum->primary);
	items2 = fwrite(buf, sizeof(__u32), items, fp);
	if (items != items2)
		return -1;

	items = fwrite(key, 1, len, fp);
	if (items != len)
		return -1;

	return 0;
}

static int user_write(hashtab_key_t key, hashtab_datum_t datum, void *p)
{
	user_datum_t *usrdatum;
	__u32 buf[32], len;
	int items, items2;
	FILE *fp = p;


	usrdatum = (user_datum_t *) datum;

	len = strlen(key);
	items = 0;
	buf[items++] = cpu_to_le32(len);
	buf[items++] = cpu_to_le32(usrdatum->value);
	items2 = fwrite(buf, sizeof(__u32), items, fp);
	if (items != items2)
		return -1;

	items = fwrite(key, 1, len, fp);
	if (items != len)
		return -1;

	if (!ebitmap_write(&usrdatum->roles, fp))
		return -1;

	return mls_write_user(usrdatum, fp);
}


static int (*write_f[SYM_NUM]) (hashtab_key_t key, hashtab_datum_t datum, void *datap) =
{
	common_write,
	class_write,
	role_write,
	type_write,
	user_write
	mls_write_f
};


/*
 * Write the configuration data in a policy database
 * structure to a policy database binary representation
 * file.
 */
int policydb_write(policydb_t * p, FILE * fp)
{
	struct role_allow *ra;
	struct role_trans *tr;
	ocontext_t *c;
	genfs_t *genfs;
	int i, j;
	__u32 buf[32], len, config, nel;
	size_t items, items2;
	char *policydb_str = POLICYDB_STRING;

	config = 0;
	mls_set_config(config);

	/* Write the magic number and string identifiers. */
	items = 0;
	buf[items++] = cpu_to_le32(POLICYDB_MAGIC);
	len = strlen(POLICYDB_STRING);
	buf[items++] = cpu_to_le32(len);
	items2 = fwrite(buf, sizeof(__u32), items, fp);
	if (items != items2)
		return -1;
	items = fwrite(policydb_str, 1, len, fp);
	if (items != len)
		return -1;

	/* Write the version, config, and table sizes. */
	items = 0;
	buf[items++] = cpu_to_le32(POLICYDB_VERSION);
	buf[items++] = cpu_to_le32(config);
	buf[items++] = cpu_to_le32(SYM_NUM);
	buf[items++] = cpu_to_le32(OCON_NUM);
	items2 = fwrite(buf, sizeof(__u32), items, fp);
	if (items != items2)
		return -1;

	if (mls_write_nlevels(p, fp))
		return -1;

	for (i = 0; i < SYM_NUM; i++) {
		buf[0] = cpu_to_le32(p->symtab[i].nprim);
		buf[1] = cpu_to_le32(p->symtab[i].table->nel);
		items = fwrite(buf, sizeof(__u32), 2, fp);
		if (items != 2)
			return -1;
		if (hashtab_map(p->symtab[i].table, write_f[i], fp))
			return -1;
	}

	if (avtab_write(&p->te_avtab, fp))
		return -1;

	nel = 0;
	for (tr = p->role_tr; tr; tr = tr->next) 
		nel++;
	buf[0] = cpu_to_le32(nel);
	items = fwrite(buf, sizeof(__u32), 1, fp);
	if (items != 1)
		return -1;
	for (tr = p->role_tr; tr; tr = tr->next) {
		buf[0] = cpu_to_le32(tr->role);
		buf[1] = cpu_to_le32(tr->type);
		buf[2] = cpu_to_le32(tr->new_role);
		items = fwrite(buf, sizeof(__u32), 3, fp);
		if (items != 3)
			return -1;		
	}

	nel = 0;
	for (ra = p->role_allow; ra; ra = ra->next) 
		nel++;
	buf[0] = cpu_to_le32(nel);
	items = fwrite(buf, sizeof(__u32), 1, fp);
	if (items != 1)
		return -1;
	for (ra = p->role_allow; ra; ra = ra->next) {
		buf[0] = cpu_to_le32(ra->role);
		buf[1] = cpu_to_le32(ra->new_role);
		items = fwrite(buf, sizeof(__u32), 2, fp);
		if (items != 2)
			return -1;		
	}

	for (i = 0; i < OCON_NUM; i++) {
		nel = 0;
		for (c = p->ocontexts[i]; c; c = c->next)
			nel++;
		buf[0] = cpu_to_le32(nel);
		items = fwrite(buf, sizeof(__u32), 1, fp);
		if (items != 1)
			return -1;
		for (c = p->ocontexts[i]; c; c = c->next) {
			switch (i) {
			case OCON_ISID:
				buf[0] = cpu_to_le32(c->sid[0]);
				items = fwrite(buf, sizeof(__u32), 1, fp);
				if (items != 1)
					return -1;
				if (context_write(&c->context[0], fp))
					return -1;
				break;
			case OCON_FS:
			case OCON_NETIF:
				len = strlen(c->u.name);
				buf[0] = cpu_to_le32(len);
				items = fwrite(buf, sizeof(__u32), 1, fp);
				if (items != 1)
					return -1;
				items = fwrite(c->u.name, 1, len, fp);
				if (items != len)
					return -1;
				if (context_write(&c->context[0], fp))
					return -1;
				if (context_write(&c->context[1], fp))
					return -1;
				break;
			case OCON_PORT:
				buf[0] = c->u.port.protocol;
				buf[1] = c->u.port.low_port;
				buf[2] = c->u.port.high_port;
				for (j = 0; j < 3; j++) {
					buf[j] = cpu_to_le32(buf[j]);
				}
				items = fwrite(buf, sizeof(__u32), 3, fp);
				if (items != 3)
					return -1;
				if (context_write(&c->context[0], fp))
					return -1;
				break;
			case OCON_NODE:
				buf[0] = cpu_to_le32(c->u.node.addr);
				buf[1] = cpu_to_le32(c->u.node.mask);
				items = fwrite(buf, sizeof(__u32), 2, fp);
				if (items != 2)
					return -1;
				if (context_write(&c->context[0], fp))
					return -1;
				break;
			case OCON_FSUSE:
				buf[0] = cpu_to_le32(c->v.behavior);
				len = strlen(c->u.name);
				buf[1] = cpu_to_le32(len);
				items = fwrite(buf, sizeof(__u32), 2, fp);
				if (items != 2)
					return -1;
				items = fwrite(c->u.name, 1, len, fp);
				if (items != len)
					return -1;
				if (c->v.behavior != SECURITY_FS_USE_PSID) {
					if (context_write(&c->context[0], fp))
						return -1;
				}
				break;
			}
		}
	}

	nel = 0;
	for (genfs = p->genfs; genfs; genfs = genfs->next) 
		nel++;
	buf[0] = cpu_to_le32(nel);
	items = fwrite(buf, sizeof(__u32), 1, fp);
	if (items != 1)
		return -1;	
	for (genfs = p->genfs; genfs; genfs = genfs->next) {
		len = strlen(genfs->fstype);
		buf[0] = cpu_to_le32(len);
		items = fwrite(buf, sizeof(__u32), 1, fp);
		if (items != 1)
			return -1;
		items = fwrite(genfs->fstype, 1, len, fp);
		if (items != len)
			return -1;
		nel = 0;
		for (c = genfs->head; c; c = c->next)
			nel++;
		buf[0] = cpu_to_le32(nel);
		items = fwrite(buf, sizeof(__u32), 1, fp);
		if (items != 1)
			return -1;
		for (c = genfs->head; c; c = c->next) {
			len = strlen(c->u.name);
			buf[0] = cpu_to_le32(len);
			items = fwrite(buf, sizeof(__u32), 1, fp);
			if (items != 1)
				return -1;
			items = fwrite(c->u.name, 1, len, fp);
			if (items != len)
				return -1;
			buf[0] = cpu_to_le32(c->v.sclass);
			items = fwrite(buf, sizeof(__u32), 1, fp);
			if (items != 1)
				return -1;
			if (context_write(&c->context[0], fp))
				return -1;
		}
	}

	if (mls_write_trusted(p, fp))
		return -1;

	return 0;
}

#endif

