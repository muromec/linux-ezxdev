/* -*- linux-c -*- */

/*
 * Implementation of the persistent label mapping.
 *
 * Author:  Stephen Smalley, <sds@epoch.ncsc.mil>
 *
 * The mapping is currently implemented using
 * regular files in a fixed subdirectory of the
 * root directory of each file system.  The "contexts" 
 * file stores the security contexts associated with
 * objects in the file system.  The "index" file
 * stores (offset, length) pairs for the security 
 * contexts in the "contexts" file, indexed by 
 * persistent SID (PSID). The "inodes" file stores 
 * PSIDs for the files in the file system, indexed by
 * inode number.
 */

#include <linux/config.h>

#include <linux/types.h>
#include <linux/flask/flask.h>
#include <linux/flask/security.h>
#include <linux/flask/psid.h>
#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>
#include "selinux_plug.h"

#if 0
#define DPRINTF(args...) printk(KERN_ALERT args)
#else
#define DPRINTF(args...)
#endif

/* Read or write at an absolute offset.  The offset is only an
   input parameter. */
static inline ssize_t read_abs(struct file *fp, char *buf, size_t len, loff_t off) 
{
  return fp->f_op->read(fp, buf, len, &off);
}
static inline ssize_t write_abs(struct file *fp, const char *buf, size_t len, loff_t off) 
{
	unsigned long limit;
	ssize_t rc;

	limit = current->rlim[RLIMIT_FSIZE].rlim_cur;
	current->rlim[RLIMIT_FSIZE].rlim_cur = RLIM_INFINITY;
	rc = fp->f_op->write(fp, buf, len, &off);
	current->rlim[RLIMIT_FSIZE].rlim_cur = limit;
	return rc;
}


/*
 * Subdirectory for the PSID mapping files.
 */
#define PSEC_SECDIR "...security"
#define PSEC_SECDIR_MODE 0700

/*
 * The PSID mapping files
 */
#define PSEC_CONTEXTS   0	/* security contexts */
#define PSEC_INDEX      1	/* psid -> (offset, len) of context */
#define PSEC_INODES     2	/* ino -> psid */
#define PSEC_NFILES	3	/* total number of security files */

static char *psec_sfiles[PSEC_NFILES] = 
{
	"contexts",
	"index", 
	"inodes"
};

#define PSEC_SECFILE_MODE (S_IFREG | 0600)


/*
 * Record structure for entries in index file.
 */
typedef struct 
{
	loff_t ofs;	/* offset within file, in bytes */
	size_t len;	/* length of context, in bytes */
} s_index_t;

#define SINDEX_MASK (sizeof(s_index_t)-1)

/*
 * For each mounted file system, we maintain a
 * incore cache that maps between PSIDs and SIDs.
 * The current implementation loads the entire
 * PSID mapping into this "cache" at mount time,
 * but the intent is to change the implementation
 * to load from the mapping on demand.  
 */
typedef struct psidtab_node {
	psid_t psid;
	security_id_t sid;
	struct psidtab_node *next;
} psidtab_node_t;

#define PSIDTAB_SLOTS 32
#define BAD_PSIDS 32

struct psidtab {
	psidtab_node_t *slots[PSIDTAB_SLOTS];	/* PSID cache */
	int    initialized;	/* is the PSID mapping initialized? */
	psid_t next_psid;	/* next PSID to be allocated */
	loff_t next_context;    /* offset of next context */
	struct file files[PSEC_NFILES];  /* mapping files */
#define contexts_fp files[PSEC_CONTEXTS] /* contexts file */
#define index_fp files[PSEC_INDEX]	 /* index file */
#define inodes_fp files[PSEC_INODES]       /* inode file */
	psid_t bad_psids[BAD_PSIDS];     /* bad PSIDs */
	int n_bad;			 /* number of bad PSIDs */
	s_index_t raw_sindex;		 /* PSID 0 */
	spinlock_t lock;
	struct semaphore sem;
	unsigned int count;
};
typedef struct psidtab psidtab_t;

#define PSIDTAB_HASH(psid) (psid & (PSIDTAB_SLOTS - 1))

static spinlock_t psidtab_lock = SPIN_LOCK_UNLOCKED;

/*
 * Two PSIDs are reserved:  PSID 0
 * refers to the security context to
 * assign to unlabeled objects in the
 * file system and PSID 1 refers to the
 * security context of the file system itself.
 */
#define FILE_PSID 0
#define FS_PSID   1

/*
 * Create a PSID cache.
 */
static int psidtab_create(psidtab_t **tp)
{
	psidtab_t 	*t;


	t = (psidtab_t *) kmalloc(sizeof(psidtab_t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;
	memset(t, 0, sizeof(psidtab_t));
	t->lock = SPIN_LOCK_UNLOCKED;
	init_MUTEX(&t->sem);
	t->count = 1;
	*tp = t;
	return 0;
}

/* 
 * Release the PSID mapping files and
 * free the memory used by the PSID cache.
 */
static void psidtab_destroy(psidtab_t *t)
{
	psidtab_node_t     *cur, *tmp;
	int             hvalue, i;

	for (i = 0; i < PSEC_NFILES; i++) {
		if (t->files[i].f_dentry) {
			dput(t->files[i].f_dentry);
			close_private_file(&t->files[i]);
		}
	}

	for (hvalue = 0; hvalue < PSIDTAB_SLOTS; hvalue++) {
		cur = t->slots[hvalue];
		while (cur) {
			tmp = cur;
			cur = cur->next;
			kfree(tmp);
		}
	}

	kfree(t);
}

static inline psidtab_t *psidtab_get(struct superblock_security_struct *sbsec)
{
	psidtab_t *t;

	spin_lock(&psidtab_lock);
	t = sbsec->psidtab;
	if (!t || !t->count) {
		spin_unlock(&psidtab_lock);
		return NULL;
	}
	t->count++;
	spin_unlock(&psidtab_lock);

	return t;
}

static inline void psidtab_put(struct superblock_security_struct *sbsec,
			       psidtab_t *t)
{
	spin_lock(&psidtab_lock);
	if (!t || !t->count) {
		if (t)
			printk("psidtab_put:  zero count\n");
		spin_unlock(&psidtab_lock);
		return;
	}
	t->count--;
	if (t->count) {
		spin_unlock(&psidtab_lock);
		return;
	}
	if (sbsec->psidtab == t) {
		printk("psidtab_put:  psidtab not already cleared.\n");
		sbsec->psidtab = NULL;
	}
	spin_unlock(&psidtab_lock);

	psidtab_destroy(t);
}

/*
 * Insert an entry for the pair (`psid', `sid') in the PSID cache.
 */
static int psidtab_insert(psidtab_t *t, psid_t psid, security_id_t sid)
{
	psidtab_node_t     *new;
	int             hvalue;


	hvalue = PSIDTAB_HASH(psid);	
	new = (psidtab_node_t *) kmalloc(sizeof(psidtab_node_t), GFP_KERNEL);
	if (!new) {
		return -ENOMEM;
	}
	new->psid = psid;
	new->sid = sid;
	new->next = t->slots[hvalue];
	wmb();
	t->slots[hvalue] = new;
	return 0;
}


/*
 * Return the SID associated with the 
 * PSID `psid' in the PSID cache.
 * Return 0 if no match is found.
 */
static security_id_t psidtab_search_psid(psidtab_t *t, psid_t psid)
{
	psidtab_node_t     *cur;
	int             hvalue;


	hvalue = PSIDTAB_HASH(psid);
	for (cur = t->slots[hvalue]; cur ; cur = cur->next) {
		if (psid == cur->psid)
			return cur->sid;
	}

	return 0;
}


/*
 * Return the PSID associated with the 
 * SID `sid' in the PSID cache.
 * If 'reserved' is zero, then a reserved PSID is not 
 * returned even if it matches.
 * Return 0 if no match is found.
 */
static psid_t psidtab_search_sid(psidtab_t *t, security_id_t sid, int reserved)
{
	psidtab_node_t     *cur;
	int             hvalue;


	for (hvalue = 0; hvalue < PSIDTAB_SLOTS; hvalue++) {
		for (cur = t->slots[hvalue]; cur; cur = cur->next) {
			if (sid == cur->sid && 
			    ((cur->psid > FS_PSID) || reserved))
				return cur->psid;
		}
	}
	
	return 0;
}


/*
 * Change the SID associated with PSID `psid'
 * to the SID `sid' in the PSID cache.
 */
static void psidtab_change_psid(psidtab_t *t, 
				psid_t psid, security_id_t sid)
{
	psidtab_node_t     *cur;
	int             hvalue;


	hvalue = PSIDTAB_HASH(psid);
	for (cur = t->slots[hvalue]; cur ; cur = cur->next) {
		if (psid == cur->psid) {
			cur->sid = sid;
			return;
		}
	}

	return;
}

#if 0
static void psidtab_hash_eval(psidtab_t *t, char *tag)
{
	int             i, nel, chain_len, max_chain_len, slots_used;
	psidtab_node_t     *node;

	nel = 0;
	slots_used = 0;
	max_chain_len = 0;
	for (i = 0; i < PSIDTAB_SLOTS; i++) {
		node = t->slots[i];
		if (node) {
			slots_used++;
			chain_len = 0;
			while (node) {
				nel++;
				chain_len++;
				node = node->next;
			}
			if (chain_len > max_chain_len)
				max_chain_len = chain_len;
		}
	}

	printk("\n%s psidtab:  %d entries and %d/%d buckets used, longest chain length %d\n",
	       tag, nel, slots_used, PSIDTAB_SLOTS, max_chain_len);
}
#else
#define psidtab_hash_eval(t, tag) 
#endif


/*
 * Allocate and return a new PSID for the 
 * security context associated with the 
 * SID `newsid'.  
 */
static int newpsid(psidtab_t *t,
		   security_id_t newsid,
		   psid_t *out_psid)
{
	security_context_t context;
	s_index_t raw_sindex;
	psid_t psid;
	loff_t off;
	size_t len;
	int rc;


	DPRINTF("newpsid:  obtaining psid for sid %d\n", newsid);

	rc = security_sid_to_context(newsid, &context, &len);
	if (rc)
		return -EACCES;
	DPRINTF("newpsid:  sid %d -> context %s\n", newsid, context);

	/*
	 * Append the security context to the contexts file.
	 */
	spin_lock(&t->lock);
	off = t->next_context;
	t->next_context += len;
	spin_unlock(&t->lock);
	rc = write_abs(&t->contexts_fp, context, len, off);
	if (rc < 0) {
		printk("newpsid:  error %d in writing to contexts\n", -rc);
		kfree(context);
		return rc;
	}
	DPRINTF("newpsid:  added %s to contexts at %Ld\n", 
		context, off);
	kfree(context);

	/*
	 * Write the index record at the location
	 * for the next allocated PSID.
	 */
	raw_sindex.ofs = cpu_to_le64(off);
	raw_sindex.len = cpu_to_le32(len);
	spin_lock(&t->lock);
	psid = t->next_psid++;
	spin_unlock(&t->lock);
	rc = write_abs(&t->index_fp, (const char*)&raw_sindex, sizeof(s_index_t), psid * sizeof(s_index_t));
	if (rc < 0) {
		printk("newpsid:  error %d in writing to index\n", -rc);
		return rc;
	}
	DPRINTF("newpsid:  added new psid %d = (%Ld,%d) to index\n", 
		psid, off, len); 

	/*
	 * Add the (`psid', `newsid') pair to the PSID cache.
	 */
	rc = psidtab_insert(t, psid, newsid);
	if (rc)
		return rc;

	DPRINTF("newpsid:  added (%d, %d) to psidtab\n", psid, newsid);

	*out_psid = psid;
	return 0;
}


/*
 * Initialize the incore data in `t' for a PSID mapping
 * from the PSID mapping files in the  file system `sb'.
 * Set the SID of `sb' and the SIDs of the inodes for the
 * root directory, the mapping directory and the mapping
 * files.
 *
 * If the file system is unlabeled and the 
 * file system is being mounted read-write, then 
 * create a new PSID mapping on it.  
 */
static int psidfiles_init(struct super_block *sb, psidtab_t *t)
{
	struct superblock_security_struct *sbsec = sb->s_security;
	struct dentry *dir, *file;
	int index;
	int need_to_init = 0;
	s_index_t sindex, raw_sindex;
	security_id_t fs_sid, file_sid, sid;
	psid_t file_psid, fs_psid, raw_psid;
	loff_t pos;
	char *cbuf;
	int rc = 0;

	/* Initialize the root directory SID before the lookup.
	   This will be changed down below once we know the real SID. */
	inode_security_set_sid(sb->s_root->d_inode, SECINITSID_FILE);

	DPRINTF("psidfiles_init:  looking up %s\n", PSEC_SECDIR);
	dir = lookup_one_len(PSEC_SECDIR, sb->s_root, strlen(PSEC_SECDIR));
	rc = PTR_ERR(dir);
	if (IS_ERR(dir)) {
		printk("psidfiles_init:  lookup_one returned %d\n", -rc);
		goto bad;
	}

	if (!dir->d_inode) {
		if ((sb->s_flags & MS_RDONLY) == 0) {
			/*
			 * The mapping subdirectory did not exist.  
			 * Since the file system is mounted 
			 * read-write, create it.
			 */
			DPRINTF("psidfiles_init:  %s did not exist; creating\n", 
				PSEC_SECDIR);

			need_to_init = 1;

			if (!sb->s_root->d_inode->i_op || 
			    !sb->s_root->d_inode->i_op->mkdir) {
				printk("psidfiles_init:  no mkdir support\n");
				rc = -EPERM;
				dput(dir);
				goto bad;
			}

			rc = sb->s_root->d_inode->i_op->mkdir(
				sb->s_root->d_inode, dir, 
				PSEC_SECDIR_MODE);
			if (rc) {
				printk("psidfiles_init:  mkdir returned %d\n", -rc);
				dput(dir);	
				goto bad;
			}
		} else {
			/*
			 * The mapping subdirectory did not exist.  
			 * Since the file system is mounted 
			 * read-only, no mapping can be created.
			 * Obtain the SID of the file system and the
			 * default file SID from the security server
			 * and return without setting the initialized
			 * flag.  
			 */
			DPRINTF("psidfiles_init:  %s did not exist; read-only\n", PSEC_SECDIR);

			dput(dir);

			rc = security_fs_sid((char *)kdevname(to_kdev_t(sb->s_dev)), 
					     &fs_sid, &file_sid);
			if (rc) 
				goto bad;

			sbsec->sid = fs_sid;
			inode_security_set_sid(sb->s_root->d_inode, file_sid);

			return 0;
		}
	}

	/* Set the SID on the mapping subdirectory */
	inode_security_set_sid(dir->d_inode, SECINITSID_FILE_LABELS);
	
	/* Look up or create each mapping file */
	for (index = 0; index < PSEC_NFILES; index++) {
		DPRINTF("psidfiles_init:  checking for %s\n",
			psec_sfiles[index]);
		file = lookup_one_len(psec_sfiles[index], dir, strlen(psec_sfiles[index]));
		rc = PTR_ERR(file);
		if (IS_ERR(file)) {
			printk("psidfiles_init:  lookup_one returned %d\n", -rc);
			goto bad;
		}

		if (!file->d_inode) {
			/*
			 * The mapping file did not exist.  
			 * If the mapping subdirectory was just created,
			 * then create the mapping file.  Otherwise,
			 * fail.
			 */
			if (!need_to_init) {
				printk("psidfiles_init:  %s did not exist\n",
				       psec_sfiles[index]);
				rc = -ENOENT;
				goto bad_file;
			}

			DPRINTF("psidfiles_init:  %s did not exist; creating\n", 
				psec_sfiles[index]);

			if (!dir->d_inode->i_op || 
			    !dir->d_inode->i_op->create) {
				printk("psidfiles_init:  no create support!\n");
				rc = -ENOENT;
				goto bad_file;
			}

			rc = dir->d_inode->i_op->create(
				dir->d_inode, 
				file, 
				PSEC_SECFILE_MODE);
			if (rc) {
				printk("psidfiles_init:  create returned %d\n", -rc);
				goto bad_file;
			}
		}

		/* Set the SID on the mapping file */
		inode_security_set_sid(file->d_inode, SECINITSID_FILE_LABELS);

		/* "Open" the file and set it for synchronous writes */
		rc = open_private_file(&t->files[index], file, O_RDWR); 
		if (index == PSEC_CONTEXTS || index == PSEC_INDEX)
			t->files[index].f_flags |= O_SYNC;
		if (rc) {	
			printk("psidfiles_init:  open_private_file returned %d\n", -rc);
			goto bad_file;
		}

		if (!t->files[index].f_op || 
		    !t->files[index].f_op->read ||
		    !t->files[index].f_op->write) {
			printk("psidfiles_init:  no read/write support\n");
			dput(dir);
			rc = -ENOENT;
			goto bad;
		}
	}

	dput(dir);

	if (need_to_init) {
		/*
		 * The mapping subdirectory and files were just created.
		 * Obtain the file system and default file SIDs from
		 * the security server, define the corresponding
		 * reserved PSIDs, set the initialized flag and return.
		 */
		DPRINTF("psidfiles_init:  initializing labeling files\n");

		rc = security_fs_sid((char*)kdevname(to_kdev_t(sb->s_dev)), 
				     &fs_sid, &file_sid);
		if (rc)
			goto bad;

		sbsec->sid = fs_sid;
		inode_security_set_sid(sb->s_root->d_inode, file_sid);

		rc = newpsid(t, file_sid, &file_psid);
		if (rc)
			goto bad;

		if (file_psid) {
			printk("psidfiles_init:  default file psid should be zero, is %d\n", file_psid);
			goto bad;
		}
		
		rc = newpsid(t, fs_sid, &fs_psid);
		if (rc)
			goto bad;

		if (fs_psid != FS_PSID) {
			printk("psidfiles_init:  file system psid should be %d, is %d\n", FS_PSID, fs_psid);
			goto bad;
		}

		t->initialized = 1;

		return 0;
	}	

	/*	
	 * The mapping subdirectory and files already existed.
	 * Load the mapping files into the PSID cache.  Obtain
	 * the SIDs for the file system and the root directory
	 * from the mapping.  Set the initialized flag.
	 */
	DPRINTF("psidfiles_init:  reading index and contexts files\n");
	pos = 0;
	rc = read_abs(&t->index_fp, (char *)&raw_sindex, sizeof(s_index_t), pos);
	t->raw_sindex.ofs = raw_sindex.ofs;
	t->raw_sindex.len = raw_sindex.len;
	while (rc > 0) {	    
		sindex.ofs = le64_to_cpu(raw_sindex.ofs);
		sindex.len = le32_to_cpu(raw_sindex.len);

		DPRINTF("psidfiles_init:  read index (%Ld,%d) at %Ld\n",
			sindex.ofs, sindex.len, pos);

		cbuf = kmalloc(sindex.len, GFP_KERNEL);
		if (!cbuf) {
			rc = -ENOMEM;
			goto bad;
		}

		rc = read_abs(&t->contexts_fp, cbuf, sindex.len, sindex.ofs);
		if (rc < 0) {
			printk("psidfiles_init:  error %d in reading contexts\n", 
			       -rc);
			kfree(cbuf);
			goto bad;
		}
		
		rc = security_context_to_sid(cbuf, rc, &sid);
		if (rc) {
			DPRINTF("psidfiles_init:  invalid context %s (psid %d).\n", 
			       cbuf, t->next_psid);
#if 0
			if ((sb->s_flags & MS_RDONLY) == 0) {
				rc = write_abs(&t->index_fp, 
					       (const char*)&t->raw_sindex, 
					       sizeof(s_index_t),
					       pos);
				if (rc < 0) {
					printk("psidfiles_init:  error %d in writing to index for psid %d\n", -rc, t->next_psid);
				}
				DPRINTF("psidfiles_init:  remapped psid %d to psid 0\n", 
					t->next_psid);
			} else {
				if (t->n_bad < BAD_PSIDS)
					t->bad_psids[t->n_bad++] = t->next_psid;
			}
#endif
			t->next_psid++;
		}
		else
		{
			DPRINTF("psidfiles_init:  psid %d -> context %s -> sid %d\n",
				t->next_psid, cbuf, sid);

			rc = psidtab_insert(t, t->next_psid, sid);
			t->next_psid++;
			if (rc) {
				goto bad;
			}
		}

		kfree(cbuf);

		pos += sizeof(s_index_t);
		rc = read_abs(&t->index_fp, (char *)&raw_sindex, 
			      sizeof(s_index_t), pos);
	}

	if (rc < 0) {
		printk("psidfiles_init:  error %d in reading index\n", 
		       -rc);
		goto bad;
	}

	fs_sid = psidtab_search_psid(t, FS_PSID);
	if (!fs_sid) {
		printk("psidfiles_init:  no SID for fs psid\n");
		rc = -EINVAL;
		goto bad;
	}

	DPRINTF("psidfiles_init:  fs sid %d\n", fs_sid);

	sbsec->sid = fs_sid;

	DPRINTF("psidfiles_init:  reading inodes file for root inode\n");

	rc = read_abs(&t->inodes_fp, (char *)&raw_psid, sizeof(psid_t), 
		      sb->s_root->d_inode->i_ino * sizeof(psid_t));
	if (rc < 0) {
		printk("psidfiles_init:  error %d in reading inodes\n", 		
		       -rc);
		goto bad;
	}
	if (rc == 0)
		file_psid = 0;
	else
		file_psid = le32_to_cpu(raw_psid);

	sid = psidtab_search_psid(t, file_psid);
	if (!sid) {
		printk("psidfiles_init:  root inode is unlabeled\n");
		sid = SECINITSID_UNLABELED;
	}

	inode_security_set_sid(sb->s_root->d_inode, sid);

	{ 
		struct inode *inode = t->contexts_fp.f_dentry->d_inode;

		down(&inode->i_sem);
		t->next_context = inode->i_size;
		up(&inode->i_sem);
	}

	t->initialized = 1;

	return 0;

bad_file:
	dput(file);
	dput(dir);
bad:	
	inode_security_set_sid(sb->s_root->d_inode, SECINITSID_UNLABELED);
	printk("psidfiles_init:  initialization failed, error %d\n", -rc);
	return rc;
}


/*
 * Initialize the PSID mapping on the file system `sb'.
 * If `sb' is an unlabeled file system and it is being
 * mounted read-write, then create a new PSID mapping
 * on it.
 */
int psid_init(struct super_block *sb)
{
	struct superblock_security_struct *sbsec = sb->s_security;
	psidtab_t *t = psidtab_get(sbsec);
	mm_segment_t old_fs;
	int rc;


	if (t) {
		/* PSID mapping is already initialized. 
		   This can occur from umount_busy if umount_close 
		   was not called, i.e. this is not the last instance
		   and it is busy. Not an error. */
		psidtab_put(sbsec, t);
		return 0;
	}

	DPRINTF("psid_init:  creating psidtab\n");

	rc = psidtab_create(&t);
	if (rc)
		return rc;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	rc = psidfiles_init(sb, t);
	set_fs(old_fs);
	if (rc)	{
		psidtab_destroy(t);
		return rc;
	}
	if (t->initialized)
		psidtab_hash_eval(t, "init");

	sbsec->psidtab = t;
	return 0;
}


/*
 * If `sb' is an unlabeled file system that was 
 * originally mounted read-only and is now being 
 * remounted read-write, then create a new PSID mapping
 * on it.
 */
int psid_remount(struct super_block *sb)
{
	struct superblock_security_struct *sbsec = sb->s_security;
	psidtab_t *t = psidtab_get(sbsec);
	mm_segment_t old_fs;
	int i, rc;

	if (!t) {
		printk("psid_remount:  uninitialized super block\n");
		return -EACCES;
	}

	if (!t->initialized) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		rc = psidfiles_init(sb, t);
		set_fs(old_fs);
		if (rc)	{
			psidtab_put(sbsec, t);
			panic("VFS:  psid_remount failed (rc=%d)", rc);
			return rc;
		}
	} else if (t->n_bad) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		for (i = 0; i < t->n_bad; i++) {
			rc = write_abs(&t->index_fp, 
				       (const char*)&t->raw_sindex, 
				       sizeof(s_index_t), 
				       t->bad_psids[i] * sizeof(s_index_t));

			if (rc < 0) {
				printk("psid_remount:  error %d in writing to index for psid %d\n", -rc, t->bad_psids[i]);
			}
			DPRINTF("psid_remount:  remapped psid %d to psid 0\n", 
				t->bad_psids[i]);
		}
		set_fs(old_fs);
		t->n_bad = 0;
	}

	if (t->initialized)
		psidtab_hash_eval(t, "remount");

	psidtab_put(sbsec, t);

	return 0;
}


/*
 * Free any memory and release any files used for
 * the PSID mapping of `sb'.
 */
void psid_release(struct super_block *sb)
{
	struct superblock_security_struct *sbsec = sb->s_security;
	psidtab_t *t = psidtab_get(sbsec);

	if (!t) {
		/* PSID mapping was already released (or never initialized). 
		   This can occur from superblock_free_security, since 
		   the PSID mapping is typically released by umount_close.
		   Not an error. */
		return;
	}

	DPRINTF("psid_release:  destroying psidtab\n");

	if (t->initialized)
		psidtab_hash_eval(sb->s_security, "release");

	/* Forcibly clear sbsec->psidtab, rather than 
	   merely waiting for the final psidtab_put. 
	   This ensures that a subsequent psid_init will 
	   be able to succeed, e.g. for umount_busy or post_pivotroot. 
	   Take the semaphore to synchronize with in-progress writes. */
	down(&t->sem);
	sbsec->psidtab = NULL;
	up(&t->sem);

	/* Put twice, once for the get above and once to drop
	   the reference saved in sbsec.  Only transient references
	   will remain for in-progress PSID functions. */
	psidtab_put(sbsec, t);
	psidtab_put(sbsec, t);
}


/*
 * Look up the PSID of `inode' in the PSID mapping
 * and return the corresponding SID.
 */
int psid_to_sid(struct inode *inode, security_id_t *out_sid)
{
	struct super_block *sb = inode->i_sb;
	struct superblock_security_struct *sbsec = sb->s_security;
	struct inode_security_struct *isec;
	psidtab_t *t = psidtab_get(sbsec);
	psid_t psid, raw_psid;
	security_id_t sid;
	mm_segment_t old_fs;
	int rc;

	if (!t) {
		*out_sid = SECINITSID_UNLABELED;
		return 0;
	}

	if (!t->initialized) {
		isec = sb->s_root->d_inode->i_security;
		*out_sid = isec->sid;
		psidtab_put(sbsec, t);
		return 0;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	rc = read_abs(&t->inodes_fp, (char *)&raw_psid, sizeof(psid_t), inode->i_ino * sizeof(psid_t));
	set_fs(old_fs);
	if (rc < 0) {
		printk("psid_to_sid:  unable to read inodes for %ld, using unlabeled\n", 
		       inode->i_ino);
		*out_sid = SECINITSID_UNLABELED;
		psidtab_put(sbsec, t);
		return 0;
	}
	if (rc == 0)
		psid = 0;
	else
		psid = le32_to_cpu(raw_psid);

	sid = psidtab_search_psid(t, psid);
	if (!sid) {
		DPRINTF("psid_to_sid:  no SID for psid %d\n", psid);
		sid = psidtab_search_psid(t, 0);
		if (!sid) {
			inode_security_set_sid(inode, SECINITSID_UNLABELED);
			psidtab_put(sbsec, t);
			return 0;
		}
	}

	*out_sid = sid;
	psidtab_put(sbsec, t);
	return 0;
}


/*
 * Look up the security context associated with the 
 * SID `sid' in the PSID mapping and set the PSID
 * for the inode accordingly.  If no PSID exists in
 * the PSID mapping for the security context, then
 * allocate a new PSID and assign it to the security
 * context.
 */
int sid_to_psid(struct inode *inode,
		security_id_t sid)
{
	struct super_block *sb = inode->i_sb;
	struct superblock_security_struct *sbsec = sb->s_security;
	psidtab_t *t = psidtab_get(sbsec);
	psid_t          psid, raw_psid;
	int rc = 0;
	mm_segment_t old_fs;

	if (!t || !t->initialized) {
		psidtab_put(sbsec, t);
		return 0;
	}

	psid = psidtab_search_sid(t, sid, 0);
	if (!psid) {
		psidtab_put(sbsec, t);
		down(&t->sem);
		/* Rescan now that we hold the semaphore */
		t = psidtab_get(sbsec);
		if (!t || !t->initialized) {
			rc = -EACCES;
			goto up_out;
		}
		psid = psidtab_search_sid(t, sid, 0);
		if (psid) 
			goto up_out;
		/* No PSID for this SID - allocate a new one. */
		if (sb->s_flags & MS_RDONLY) {
			rc = -EACCES;
			goto up_out;
		}

		old_fs = get_fs();
		set_fs(KERNEL_DS);
		rc = newpsid(t, sid, &psid);
		set_fs(old_fs);
up_out:
		up(&t->sem);
	}

	if (rc) {
		psidtab_put(sbsec, t);
		return rc;
	}

	raw_psid = cpu_to_le32(psid);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	rc = write_abs(&t->inodes_fp, (char*)&raw_psid, sizeof(psid_t), 
		       inode->i_ino * sizeof(psid_t));
	set_fs(old_fs);
	if (rc < 0) {
		printk("sid_to_psid:  write_abs failed, rc=%d (dev=%s ino=%ld), offset %ld\n", -rc, kdevname(to_kdev_t(inode->i_dev)), inode->i_ino, inode->i_ino * sizeof(psid_t));
		psidtab_put(sbsec, t);
		return rc;
	}
	psidtab_put(sbsec, t);
	return 0;
}

int clear_psid(struct inode *inode)
{
	struct super_block *sb = inode->i_sb;
	struct superblock_security_struct *sbsec = sb->s_security;
	psidtab_t *t = psidtab_get(sbsec);
	psid_t          raw_psid;
	int rc;
	mm_segment_t old_fs;

	if (!t || !t->initialized) {
		psidtab_put(sbsec, t);
		return 0;
	}

	raw_psid = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	rc = write_abs(&t->inodes_fp, (char*)&raw_psid, sizeof(psid_t), 
		       inode->i_ino * sizeof(psid_t));
	set_fs(old_fs);
	if (rc < 0) {
		psidtab_put(sbsec, t);
		return rc;
	}
	psidtab_put(sbsec, t);
	return 0;
}


/*
 * Change the security context associated
 * with PSID `psid' in the PSID mapping to
 * the security context associated with `newsid'.
 */
static int chpsid(psidtab_t *t,
		  psid_t psid,
		  security_id_t newsid)
{
	security_context_t context;
	s_index_t raw_sindex;
	psid_t clone_psid;
	loff_t off;
	size_t len;
	int rc = 0;

	DPRINTF("chpsid:  changing psid %d to sid %d\n", psid, newsid);

	psid = psidtab_search_sid(t, newsid, 1);
	if (psid) {
		/*
		 * There is already a PSID for the security context
		 * associated with `newsid'.  The index record for 
		 * `psid' can simply be changed to be the same as 
		 * the index record for this PSID.
		 */
		clone_psid = psid;
		DPRINTF("chpsid:  cloning existing psid %d\n", clone_psid);
		rc = read_abs(&t->index_fp, 
			      (char *)&raw_sindex, sizeof(s_index_t), 
			      clone_psid * sizeof(s_index_t));
		if (rc < 0) 
			return rc;
		
		off = le64_to_cpu(raw_sindex.ofs);
		len = le32_to_cpu(raw_sindex.len);
	} else {
		/*
		 * There is no PSID for the security context 
		 * associated with `newsid'.  Add the security
		 * context to the contexts file and define a
		 * new index record for `psid'.
		 */
		DPRINTF("chpsid:  adding new entry to contexts\n");
		rc = security_sid_to_context(newsid, &context, &len);
		if (rc) 
			return rc;

		DPRINTF("chpsid:  sid %d -> context %s\n", newsid, context);

		spin_lock(&t->lock);
		off = t->next_context;
		t->next_context += len;
		spin_unlock(&t->lock);
		rc = write_abs(&t->contexts_fp, context, len, off);
		if (rc < 0) {
			kfree(context);
			return rc;
		}

		raw_sindex.ofs = cpu_to_le64(off);
		raw_sindex.len = cpu_to_le32(len);

		DPRINTF("chpsid:  added %s to contexts at %Ld\n", 
			context, off);

		kfree(context);
	}

	/*
	 * Change the index record for `psid'.
	 */
	rc = write_abs(&t->index_fp, (const char*)&raw_sindex, sizeof(s_index_t), psid * sizeof(s_index_t));
	if (rc < 0) 
		return rc;

	DPRINTF("chpsid:  changed psid %d to (%Ld, %d)\n", 
		psid, off, len); 

	/* 
	 * Change the SID for `psid' in the PSID cache.
	 */
	psidtab_change_psid(t, psid, newsid);

	DPRINTF("chpsid:  changed to (%d, %d) in psidtab\n", psid, newsid);

	return 0;
}


/*
 * Change the file system security and the 
 * default file security context in the PSID
 * mapping of `sb' to the security contexts
 * associated with `fs_sid' and `f_sid'.
 * If either `fs_sid' or `f_sid' is null,
 * then do not change the corresponding
 * security context.
 */
int psid_chsidfs(struct super_block *sb, 
		 security_id_t fs_sid,
		 security_id_t f_sid)
{
	struct superblock_security_struct *sbsec = sb->s_security;
	psidtab_t *t = psidtab_get(sbsec);
	mm_segment_t old_fs;
	int rc = 0;


	if (sb->s_flags & MS_RDONLY) {
		DPRINTF("psid_chsidfs:  file system is read-only\n");
		psidtab_put(sbsec, t);
		return -EACCES;
	}

	if (!t || !t->initialized) {
		psidtab_put(sbsec, t);
		return -EACCES;
	}	


	psidtab_put(sbsec, t);
	down(&t->sem);
	t = psidtab_get(sbsec);
	if (!t || !t->initialized) {
		rc = -EACCES;
		goto out;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	if (fs_sid) {
		rc = chpsid(t, FS_PSID, fs_sid);
	}
	if (f_sid && !rc) {
		rc = chpsid(t, FILE_PSID, f_sid);
	}
	set_fs(old_fs);

out:
	up(&t->sem);
	psidtab_put(sbsec, t);
	return rc;
}
