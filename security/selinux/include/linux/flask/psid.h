
/* -*- linux-c -*- */

/*
 * Author : Stephen Smalley, <sds@epoch.ncsc.mil> 
 */

#ifndef _LINUX_FLASK_PSID_H_
#define _LINUX_FLASK_PSID_H_

/*
 * Persistent label mappings. 
 */

#include <linux/types.h>
#include <linux/flask/flask.h>
#include <linux/fs.h>

/*
 * Initialize the PSID mapping on the file system `sb'.
 * If `sb' is an unlabeled file system and it is being
 * mounted read-write, then create a new PSID mapping
 * on it.
 */
int psid_init(struct super_block *sb);

/*
 * If `sb' is an unlabeled file system that was 
 * originally mounted read-only and is now being 
 * remounted read-write, then create a new PSID mapping
 * on it.
 */
int psid_remount(struct super_block *sb);

/*
 * Free any memory and release any files used for
 * the PSID mapping of `sb'.
 */
void psid_release(struct super_block *sb);

/*
 * Look up the PSID of `inode' in the PSID mapping
 * and return the SID for the corresponding 
 * security context.
 */
int psid_to_sid(
	struct inode *inode, security_id_t *out_sid);

/*
 * Look up the security context associated with the 
 * SID `sid' in the PSID mapping and set the PSID
 * for the inode accordingly.  If no PSID exists in
 * the PSID mapping for the security context, then
 * allocate a new PSID and assign it to the security
 * context.
 */
int sid_to_psid(
	struct inode *inode,
	security_id_t sid);

/*
 * Clear the PSID associated with `inode'.
 */
int clear_psid(struct inode *inode);

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
		 security_id_t f_sid);

#endif /* _LINUX_FLASK_PSID_H_ */

