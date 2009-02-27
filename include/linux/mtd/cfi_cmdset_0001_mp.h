#ifndef _MTD_CFI_CMDSET_0001_MP_H
#define _MTD_CFI_CMDSET_0001_MP_H

/* FLash partition */

struct flpartition{
	unsigned long offset;
	unsigned long size;

	flstate_t state;
	flstate_t oldstate;

	spinlock_t *mutex;
	spinlock_t _spinlock;
	wait_queue_head_t wq;

	struct cfi_intelext_regioninfo *info;
};

/* FLash private data */
struct flprivate {
	unsigned long num_writing;
	unsigned long num_erasing;

	int num_partitions;
	struct flpartition *partitions;

	int curpos;
	struct cfi_pri_intelext *extp;
};

#define PARTITION_SEL(chip, adr) ({	\
	int __num;	\
	unsigned long __start, __end; 	\
	struct flpartition *__p=NULL, *__p0;	\
	struct flprivate *__priv = (struct flprivate*) chip->priv; \
	for (__num=0; __num < __priv->num_partitions; __num++) {	\
		__p0 = __priv->partitions + __priv->curpos; \
		__start = __p0->offset; \
		__end = __start + __p0->size; \
		if ( (adr >= __start) && (adr < __end)) { \
			__p = __p0;\
			break; \
		}; \
		__priv->curpos = (__priv->curpos + 1) % __priv->num_partitions; \
	} \
	__p; \
})

#define CFI_READ_QUERY(map, base, adr, extp, size) \
do {  \
	struct cfi_private *__cfi = map->fldrv_priv; \
	int __ofs_factor = __cfi->interleave * __cfi->device_type; \
	int __i; \
	for (__i=0; __i<(size); __i++)  \
		((unsigned char*)(extp))[__i] =  \
			cfi_read_query(map, (base+(((adr)+__i)*__ofs_factor))); \
}while(0); 

#define CFI_QUERY_SIZE(map, base, adr) ({ \
	struct cfi_private *__cfi = map->fldrv_priv; \
	int __ofs_factor = __cfi->interleave * __cfi->device_type; \
	struct cfi_pri_intelext __ext; \
	struct cfi_intelext_regioninfo __rinfo; \
	int __ofs = 0, __num_r, __ri; \
	CFI_READ_QUERY(map, base, adr, (&__ext), sizeof(__ext)); \
	__ofs += sizeof(__ext); \
	if (__ext.MajorVersion == '1' && __ext.MinorVersion == '3') { \
		__ofs += (__ext.NumProtectionFields - 1) * (sizeof(__u32) + 6); \
		__ofs += 6; \
		__num_r = cfi_read_query(map, (base + ((adr+__ofs)*__ofs_factor))); \
		__ofs ++; \
		for (__ri=0; __ri < __num_r; __ri++) { \
			CFI_READ_QUERY(map, base, adr+__ofs, (&__rinfo), sizeof(__rinfo)); \
			__ofs += sizeof(__rinfo); \
			__ofs += (__rinfo.NumBlockTypes - 1) * sizeof(struct cfi_intelext_blockinfo); \
		} \
	} \
	__ofs; \
})


#endif
