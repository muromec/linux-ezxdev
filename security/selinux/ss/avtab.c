
/* Author : Stephen Smalley, <sds@epoch.ncsc.mil> */

/* FLASK */

/* 
 * Implementation of the access vector table type.
 */

#include "avtab.h"
#include "policydb.h"

#define HASH_BITS 15
#define HASH_BUCKETS (1 << HASH_BITS)
#define HASH_MASK (HASH_BUCKETS-1)

#define AVTAB_SIZE HASH_BUCKETS

#define AVTAB_HASH(keyp) \
((keyp->target_class + \
 (keyp->target_type << 2) + \
 (keyp->source_type << 9)) & \
 HASH_MASK)

int avtab_insert(avtab_t * h, avtab_key_t * key, avtab_datum_t * datum)
{
	int hvalue;
	avtab_ptr_t prev, cur, newnode;

	if (!h)
		return -ENOMEM;

	hvalue = AVTAB_HASH(key);
	for (prev = NULL, cur = h->htable[hvalue];
	     cur;
	     prev = cur, cur = cur->next) {
		if (key->source_type == cur->key.source_type && 
		    key->target_type == cur->key.target_type &&
		    key->target_class == cur->key.target_class &&
		    (datum->specified & cur->datum.specified))
			return -EEXIST;
		if (key->source_type < cur->key.source_type)
			break;
		if (key->source_type == cur->key.source_type && 
		    key->target_type < cur->key.target_type)
			break;
		if (key->source_type == cur->key.source_type && 
		    key->target_type == cur->key.target_type &&
		    key->target_class < cur->key.target_class)
			break;
	}

	newnode = (avtab_ptr_t) malloc_sleep(sizeof(struct avtab_node));
	if (newnode == NULL)
		return -ENOMEM;
	memset(newnode, 0, sizeof(struct avtab_node));
	newnode->key = *key;
	newnode->datum = *datum;
	if (prev) {
		newnode->next = prev->next;
		prev->next = newnode;
	} else {
		newnode->next = h->htable[hvalue];
		h->htable[hvalue] = newnode;
	}

	h->nel++;
	return 0;
}


avtab_datum_t *
 avtab_search(avtab_t * h, avtab_key_t * key, int specified)
{
	int hvalue;
	avtab_ptr_t cur;


	if (!h)
		return NULL;

	hvalue = AVTAB_HASH(key);
	for (cur = h->htable[hvalue]; cur; cur = cur->next) {
		if (key->source_type == cur->key.source_type && 
		    key->target_type == cur->key.target_type &&
		    key->target_class == cur->key.target_class &&
		    (specified & cur->datum.specified))
			return &cur->datum;

		if (key->source_type < cur->key.source_type)
			break;
		if (key->source_type == cur->key.source_type && 
		    key->target_type < cur->key.target_type)
			break;
		if (key->source_type == cur->key.source_type && 
		    key->target_type == cur->key.target_type &&
		    key->target_class < cur->key.target_class)
			break;
	}

	return NULL;
}


void avtab_destroy(avtab_t * h)
{
	int i;
	avtab_ptr_t cur, temp;


	if (!h)
		return;

	for (i = 0; i < AVTAB_SIZE; i++) {
		cur = h->htable[i];
		while (cur != NULL) {
			temp = cur;
			cur = cur->next;
			free(temp);
		}
		h->htable[i] = NULL;
	}
	free(h->htable);
}


int avtab_map(avtab_t * h,
	      int (*apply) (avtab_key_t * k,
			    avtab_datum_t * d,
			    void *args),
	      void *args)
{
	int i, ret;
	avtab_ptr_t cur;


	if (!h)
		return 0;

	for (i = 0; i < AVTAB_SIZE; i++) {
		cur = h->htable[i];
		while (cur != NULL) {
			ret = apply(&cur->key, &cur->datum, args);
			if (ret)
				return ret;
			cur = cur->next;
		}
	}
	return 0;
}


int avtab_init(avtab_t * h)
{
	int i;

	h->htable = malloc_sleep(sizeof(avtab_ptr_t)*AVTAB_SIZE);
	if (!h->htable)
		return -1;
	for (i = 0; i < AVTAB_SIZE; i++)
		h->htable[i] = (avtab_ptr_t) NULL;
	h->nel = 0;
	return 0;
}


void avtab_hash_eval(avtab_t * h, char *tag)
{
	int i, chain_len, slots_used, max_chain_len;
	avtab_ptr_t cur;


	slots_used = 0;
	max_chain_len = 0;
	for (i = 0; i < AVTAB_SIZE; i++) {
		cur = h->htable[i];
		if (cur) {
			slots_used++;
			chain_len = 0;
			while (cur) {
				chain_len++;
				cur = cur->next;
			}

			if (chain_len > max_chain_len)
				max_chain_len = chain_len;
		}
	}

	printf("%s:  %d entries and %d/%d buckets used, longest chain length %d\n",
	       tag, h->nel, slots_used, AVTAB_SIZE, max_chain_len);
}


int avtab_read(avtab_t * a, FILE * fp, __u32 config)
{
	int i, rc;
	avtab_key_t avkey;
	avtab_datum_t avdatum;
	__u32 buf[32];
	__u32 nel;
	size_t items, items2;


	items = fread(&nel, sizeof(__u32), 1, fp);
	if (items != 1) {
		printf("security: avtab: truncated table\n");
		goto bad;
	}
	nel = le32_to_cpu(nel);
	if (!nel) {
		printf("security: avtab: table is empty\n");
		goto bad;
	}
	for (i = 0; i < nel; i++) {
		memset(&avkey, 0, sizeof(avtab_key_t));
		memset(&avdatum, 0, sizeof(avtab_datum_t));

		items = fread(buf, sizeof(__u32), 1, fp);
		if (items != 1) {
			printf("security: avtab: truncated entry\n");
			goto bad;
		}
		items2 = le32_to_cpu(buf[0]);
		if (items2 > (sizeof(buf) / sizeof(__u32))) {
			printf("security: avtab: entry too large\n");
			goto bad;
		}
		items = fread(buf, sizeof(__u32), items2, fp);
		if (items != items2) {
			printf("security: avtab: truncated entry\n");
			goto bad;
		}
		items = 0;
		avkey.source_type = le32_to_cpu(buf[items++]);
		avkey.target_type = le32_to_cpu(buf[items++]);
		avkey.target_class = le32_to_cpu(buf[items++]);
		avdatum.specified = le32_to_cpu(buf[items++]);
		if (!(avdatum.specified & (AVTAB_AV | AVTAB_TYPE))) {
			printf("security: avtab: null entry\n");
			goto bad;
		}
		if ((avdatum.specified & AVTAB_AV) &&
		    (avdatum.specified & AVTAB_TYPE)) {
			printf("security: avtab: entry has both access vectors and types\n");
			goto bad;
		}
		if (avdatum.specified & AVTAB_AV) {
			if (avdatum.specified & AVTAB_ALLOWED)
				avtab_allowed(&avdatum) = le32_to_cpu(buf[items++]);
			if (avdatum.specified & AVTAB_AUDITDENY) 
				avtab_auditdeny(&avdatum) = le32_to_cpu(buf[items++]);
			if (avdatum.specified & AVTAB_AUDITALLOW) 
				avtab_auditallow(&avdatum) = le32_to_cpu(buf[items++]);
		} else {
			if (avdatum.specified & AVTAB_TRANSITION)
				avtab_transition(&avdatum) = le32_to_cpu(buf[items++]);
			if (avdatum.specified & AVTAB_CHANGE)
				avtab_change(&avdatum) = le32_to_cpu(buf[items++]);
			if (avdatum.specified & AVTAB_MEMBER)
				avtab_member(&avdatum) = le32_to_cpu(buf[items++]);
		}
		if (items != items2) {
			printf("security: avtab: entry only had %d items, expected %d\n", items2, items);
			goto bad;
		}
		rc = avtab_insert(a, &avkey, &avdatum);
		if (rc) {
			if (rc == -ENOMEM)
				printf("security: avtab: out of memory\n");
			if (rc == -EEXIST)
				printf("security: avtab: duplicate entry\n");
			goto bad;
		}
	}

	return 0;

      bad:
	avtab_destroy(a);
	return -1;
}


#ifndef __KERNEL__
int avtab_write(avtab_t * a, FILE * fp)
{
	int i;
	avtab_ptr_t cur;
	__u32 buf[32];
	__u32 nel;
	size_t items, items2;

	nel = cpu_to_le32(a->nel);
	items = fwrite(&nel, sizeof(__u32), 1, fp);
	if (items != 1)
		return -1;

	for (i = 0; i < AVTAB_SIZE; i++) {
		for (cur = a->htable[i]; cur; cur = cur->next) {
			items = 1;	/* item 0 is used for the item count */
			buf[items++] = cpu_to_le32(cur->key.source_type);
			buf[items++] = cpu_to_le32(cur->key.target_type);
			buf[items++] = cpu_to_le32(cur->key.target_class);
			buf[items++] = cpu_to_le32(cur->datum.specified);
			if (!(cur->datum.specified & (AVTAB_AV | AVTAB_TYPE))) {
				printf("security: avtab: null entry\n");
				return -1;
			}
			if ((cur->datum.specified & AVTAB_AV) &&
			    (cur->datum.specified & AVTAB_TYPE)) {
				printf("security: avtab: entry has both access vectors and types\n");
				return -1;
			}
			if (cur->datum.specified & AVTAB_AV) {
				if (cur->datum.specified & AVTAB_ALLOWED)
					buf[items++] = cpu_to_le32(avtab_allowed(&cur->datum));
				if (cur->datum.specified & AVTAB_AUDITDENY)
					buf[items++] = cpu_to_le32(avtab_auditdeny(&cur->datum));
				if (cur->datum.specified & AVTAB_AUDITALLOW)
					buf[items++] = cpu_to_le32(avtab_auditallow(&cur->datum));
			} else {
				if (cur->datum.specified & AVTAB_TRANSITION)
					buf[items++] = cpu_to_le32(avtab_transition(&cur->datum));
				if (cur->datum.specified & AVTAB_CHANGE)
					buf[items++] = cpu_to_le32(avtab_change(&cur->datum));
				if (cur->datum.specified & AVTAB_MEMBER)
					buf[items++] = cpu_to_le32(avtab_member(&cur->datum));
			}
			buf[0] = cpu_to_le32(items - 1);

			items2 = fwrite(buf, sizeof(__u32), items, fp);
			if (items != items2)
				return -1;
		}
	}

	return 0;
}
#endif

