
/*
 * Author:  Stephen Smalley, <sds@epoch.ncsc.mil>
 */

/* FLASK */

/* 
 * Global definitions that are included at the beginning
 * of every file using the -include directive.
 *
 * These definitions are used to permit the same
 * source code to be used to build both the security
 * server component of the kernel and the checkpolicy
 * program.
 */

#ifndef __SS_GLOBAL_H
#define __SS_GLOBAL_H
/*
 * This variable is set to one when the security server
 * has completed initialization.
 */
extern int ss_initialized;

#ifndef __KERNEL__

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>
#include <errno.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <byteswap.h>
#include <endian.h>
#include <linux/flask/flask.h>
#include <linux/flask/security.h>
#include <linux/flask/avc_ss.h>
#include <linux/flask/av_permissions.h>
#include <linux/flask/syscalls.h>

#define NIPQUAD(addr) \
	((unsigned char *)&addr)[0], \
	((unsigned char *)&addr)[1], \
	((unsigned char *)&addr)[2], \
	((unsigned char *)&addr)[3]

#if __BYTE_ORDER == __LITTLE_ENDIAN
#define cpu_to_le32(x) (x)
#define le32_to_cpu(x) (x)
#define cpu_to_le64(x) (x)
#define le64_to_cpu(x) (x)
#else
#define cpu_to_le32(x) bswap_32(x)
#define le32_to_cpu(x) bswap_32(x)
#define cpu_to_le64(x) bswap_64(x)
#define le64_to_cpu(x) bswap_64(x)
#endif

#define wmb() 

#define malloc_sleep(size) malloc(size)

#else

#include <linux/kernel.h>	/* printk */
#include <linux/slab.h>		/* kmalloc, kfree */
#include <linux/vmalloc.h>
#include <linux/fs.h>		/* read, write, open */
#include <linux/file.h>
#include <linux/string.h>	/* strcpy, strncpy, strcmp */
#include <linux/in.h>		/* IPPROTO_* */
#include <linux/ctype.h>
#include <linux/flask/flask.h>
#include <linux/flask/avc.h>
#include <linux/flask/avc_ss.h>
#include <linux/flask/security.h>
#include <linux/flask/syscalls.h>
#include <asm/system.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include "selinux_plug.h"

#define malloc(size) kmalloc(size, GFP_ATOMIC)
#define malloc_sleep(size) kmalloc(size, GFP_KERNEL)
#define free(ptr) kfree(ptr)

typedef struct file FILE;

static inline FILE *fopen(char *path, char *type)
{
	struct nameidata nd;
	struct file *file;
	int err = 0;

	if (strcmp(type, "r"))
		panic("fopen");

	if (path_init(path, LOOKUP_FOLLOW | LOOKUP_POSITIVE, &nd)) 
		err = path_walk(path, &nd);
	if (err)
		return NULL;

	if (!ss_initialized)
		inode_security_set_sid(nd.dentry->d_inode,SECINITSID_POLICY);

	if (!S_ISREG(nd.dentry->d_inode->i_mode)) 
		goto bad;

	file = dentry_open(nd.dentry, nd.mnt, O_RDONLY);
	if (IS_ERR(file))
		return NULL;
	else 
		return file;

bad:
	path_release(&nd);
	return NULL;
}


static inline int fclose(FILE * stream)
{
	fput(stream);
	return 0;
}

extern unsigned char *ss_fread_inbuf;
extern unsigned ss_fread_insize;
extern int ss_fread_use_inbuf;

static inline ssize_t fread(void *buf, size_t size, size_t nitems, FILE * fp)
{
	unsigned long bytes = nitems * size;

	if (ss_fread_use_inbuf) {
		if (fp->f_pos >= ss_fread_insize) 
			return 0;
		if ((fp->f_pos + bytes) > ss_fread_insize) 
			bytes = ss_fread_insize - fp->f_pos;
		memcpy(buf, &ss_fread_inbuf[fp->f_pos], bytes);
		fp->f_pos += bytes;
		return (bytes / size);
	} else {
		mm_segment_t old_fs;
		ssize_t rc;

		old_fs = get_fs();
		set_fs(KERNEL_DS);
		rc = (fp)->f_op->read((fp), (buf), bytes, &(fp)->f_pos);
		set_fs(old_fs);
		if (rc > 0)
			return (rc / size);
		return 0;
	}
}

#define printf printk

#define exit(error_code) panic("SS: exiting (%d)",error_code)

#endif

#endif /* __SS_GLOBAL_H */
