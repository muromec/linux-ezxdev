/*
 * security/dte/read_policy.c
 * DTE security module functions.  These are inserted into the DTE
 * security plug in security/dte.c
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 * The functions which I don't redefine stay in security/dte/dte.c.
 *
 * author: Serge Hallyn  <hallyn@cs.wm.edu>
 */

/*
 * CONFIG_DTE_VERBOSE:  prints some helpful info, ie when access is denied
 * CONFIG_DTE_DEBUG:    prints painful amount of debugging info
 */
#include "dte.h"
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/security.h>
#include <linux/sysctl.h>

kmem_cache_t *dte_type_cache,  /* cache of typenames */
*dte_path_cache,  /* cache of pathnames */
*dte_domain_cache,/* cache of domains */
*dte_ep_cache,    /* entry point cache */
*dte_ea_cache,    /* entry access cache */
*dte_sa_cache,    /* signal access cache */
*dte_mntr_cache,  /* mount restricts */
*dte_map_cache;   /* dte_map_node cache */

/*
 * Secondary_ops to allow using capability, and hopefully the openwall
 * modules as secondary modules.  Needless to say, example gratefully
 * taken from selinux.
 */
extern struct security_operations *dte_secondary_ops;

/*
 * SECTION
 * dte_types pretends to be an array of typenames.
 *
 * we grab space for the actual names out of a kernel cache.  when
 * we grab a slab, dte_type_names points to the start of it.  As we
 * add typenames, dte_type_names move to point to the first free spot
 * in the slab, and sizeof_type_names lists the free space left in the
 * slab.
 *
 * num_dte_types gives the number of types.  It can't grow
 *  larger than pagesize (8192)/sizeof(char*), because
 *  the array sits on a 4096-byte kernel page
 *
 *  The same layout is used for dte pathnames.
 */
char **dte_types, **dte_paths; /* arrays pointing to actual names */
char *dte_type_names;  /* only used at boot, this is where dte_types \ */
char *dte_path_names;  /* and dte_paths point into */
int num_dte_types, num_dte_paths;
int sizeof_dte_types, sizeof_dte_paths;  /* space used in current 4096k block */

static char *dte_type_name_backup[100];
static char *dte_path_name_backup[100];
static int num_dte_type_name_slabs;
static int num_dte_path_name_slabs;

extern struct dte_map_node   *dte_root_mapnode;
extern int num_dte_map_nodes;

struct dte_mntr **dte_mount_r;
int num_dte_mount_r;

int dte_initialized;
int dte_debug;
struct dte_domain_t *dte_init_domain;
struct dte_domain_t **dte_domains;  /* 8192 bytes */
int num_dte_domains;
int dte_bitchmode;

/*
 * DTE really should not be compiled as a module.  The reason is that there is
 * no good way to determine domains for running processes, or types for loaded
 * files.
 *
 * Domains for running processes could be solved by running through the process
 * tree, and determining labels based upon the name of the executed file.  Two
 * problems are the determination of the exec'd filename, and the possibility
 * of a parent process dying before the child.  The child's parent will be
 * reassigned, and the result could be completely wrong.
 *
 * For now (perhaps forever), if you insist on using dte as a module,  all
 * running processes will be assigned a default domain.  My first solution, to
 * get this running, is to use init_domain as default_domain.  If it seems
 * worth it, I could make it separately specifiable in the config file.
 *
 * I'm afraid just starting at root and walking the whole path *is* the best
 * solution for setting up the inode->type and inode->mapnode pointers.  Since
 * "rootfs" is a loose concept these days, I will simply use the rootfs of the
 * process loading the module.  Who knows, maybe it'll allow for some cool
 * trickery of setting up a new namespace before setting up dte.
 */
struct dte_domain_t *default_domain;

#define DTE_DEBUG(x) printk(KERN_NOTICE x);
#define DTE_DEBUG2(x,y) printk(KERN_NOTICE x,y);
#define DTE_NO_MEM(x) { \
	printk(KERN_NOTICE "DTE: %s: out of memory", x); \
	return -ENOMEM; \
}

static int atoi(char *c, char *ce)
{
	int ret = 0;

	while (c<ce) {
		ret *= 10;
		ret += ((*c)-'0');
		c++;
	}
	return ret;
}

unsigned int dte_hash(const char *s, int n)
{
	unsigned int sum;

	if (!s) return 0;
	sum = *s++;
	while (*s) {
		sum = (sum << 4) | (sum >> (8*sizeof(unsigned long)-4));
		sum ^= *s++;
	}
	return (sum % (unsigned int)n);
}

unsigned int dte_hash_c(char *c, char *ce, int n)
{
	unsigned int sum;

	if (!c || !ce) return 0;
	sum = *c++;
	while (c < ce) {
		sum = (sum << 4) | (sum >> (8*sizeof(unsigned long)-4));
		sum ^= *c++;
	}
	return (sum % (unsigned int)n);
}

static int strcmp_c(char *s, char *c, char *ce)
{
	while (c<ce) {
		if (*s++!=*c++)
			return 1;
	}
	if (*s!='\0')
		return 1;
	return 0;
}

static int dte_add_typename(char *s, char *e)
{
	int len = e-s;

	if (sizeof_dte_types+len+1 > 4096) {
		/* grab a new slab to throw this onto */
		if (num_dte_type_name_slabs>99) {
			printk(KERN_ERR "DTE: too many typenames.\n");
			return -ENOMEM;
		}
		dte_type_name_backup[num_dte_type_name_slabs++] = dte_type_names;
		dte_type_names = kmem_cache_alloc(dte_type_cache,GFP_KERNEL);
		if (!dte_type_names) {
			printk(KERN_NOTICE "dte_add_typename: insufficient memory for names.\n");
			return -ENOMEM;
		}
		sizeof_dte_types = 0;
	}
	sizeof_dte_types += len+1;
	if (num_dte_types > 2048) {
		printk(KERN_NOTICE "2048 types?  MY LORD!  Sorry, the dte_types array\n");
		printk(KERN_NOTICE "doesn't expand (yet).  No more types possible.\n");
		return -1;
	}
	dte_types[num_dte_types++] = dte_type_names;
	while (s<e) {
		*dte_type_names = *s;
		dte_type_names++;
		s++;
	}
	sizeof_dte_types += len+1;
	*dte_type_names = '\0';
	dte_type_names++;
	return 0;
}

static char *dte_add_pathname(char *s, char *e)
{
	int len = e-s;
	int i=0;

	/* avoid duplicates */
	while (i<num_dte_paths) {
		if (strcmp_c(dte_paths[i],s,e)==0)
			return dte_paths[i];
		i++;
	}

	if (sizeof_dte_paths+len+1 > 4096) {
		/* grab a new slab to throw this onto */
		if (num_dte_path_name_slabs>99) {
			printk(KERN_ERR "DTE: too many pathnames.\n");
			return NULL;
		}
		dte_path_name_backup[num_dte_path_name_slabs++] = dte_path_names;
		dte_path_names = kmem_cache_alloc(dte_path_cache,GFP_KERNEL);
		if (!dte_path_names) {
			printk(KERN_NOTICE "dte_add_pathname: insufficient memory for names.\n");
			return NULL;
		}
		sizeof_dte_paths = 0;
	}
	sizeof_dte_paths += len+1;
	if (num_dte_paths > 2048) {
		printk(KERN_NOTICE "2048 paths?  MY LORD!  Sorry, the dte_paths array\n");
		printk(KERN_NOTICE "doesn't expand (yet).  No more paths possible.\n");
		return NULL;
	}
	dte_paths[num_dte_paths++] = dte_path_names;
	while (s<e) {
		*dte_path_names = *s;
		dte_path_names++;
		s++;
	}
	sizeof_dte_paths += len+1;
	*dte_path_names = '\0';
	dte_path_names++;
	return dte_paths[num_dte_paths-1];
}

static int dte_add_domain(char *s, char *e)
{
	struct dte_domain_t *d;
	char *c;

	d = dte_domains[num_dte_domains] = kmem_cache_alloc(dte_domain_cache,
			GFP_KERNEL);
	if (!d)
		DTE_NO_MEM("dte_add_domain");
	c = d->name = kmalloc(e-s+1,GFP_KERNEL);
	if (!c)
		DTE_NO_MEM("dte_add_domain (2)");
	d->namelen = (e-s);
	while (s<e)
		*c++ = *s++;
	*c = '\0';
	d->num_ep = d->num_ta = d->num_ea = d->num_sa = d->num_gw = 0;
	d->ep = NULL;
	d->ta = NULL;
	d->ea = NULL;
	d->sa = NULL;
	d->gw = NULL;
	d->hash_next = NULL;
	num_dte_domains++;
	return 0;
}

static struct dte_domain_t *dte_get_domain(char *c, char *ce)
{
	struct dte_domain_t *d;
	int len = ce-c;
	int h;

	h = dte_hash_c(c,ce,num_dte_domains);
	if (h<0 || h>num_dte_domains)
		return NULL;
	d = dte_domains[h];
	if (!d)
		return NULL;
	while (d->hash_next && (strncmp(d->name,c,len) || d->namelen!=len))
		d = d->hash_next;
	if (strncmp(d->name,c,len) || d->namelen!=len)
		return NULL;
	return d;
}

/* this is only called at boot, so not too worried about speed */
/* hey wait - that's no longer true! It's called from setup_eafile*/
/* 
 * TODO
 * will have to sort or hash these now!
 * */
char *dte_get_type(char *c, char *ce)
{
	int i;

	for (i=0; i<num_dte_types; i++) {
		if (!strcmp_c(dte_types[i], c, ce))
			return dte_types[i];
	}
	return 0;
}

static int sort_domains(void)
{
	int i, lo, h, nd=num_dte_domains;
	struct dte_domain_t **d, *t;

	d = (struct dte_domain_t **)kmalloc(nd*sizeof(struct dte_domain_t *),GFP_KERNEL);
	if (!d) 
		DTE_NO_MEM("sort_domains");

	for (i=0; i<nd; i++) {
		d[i] = dte_domains[i];
		dte_domains[i] = NULL;
	}
	/* lo keeps the lowest dte_domain which might be null */
	/* used only in case of hash collisions */
	lo = 0;
	for (i=0; i<nd; i++) {
		h = dte_hash(d[i]->name, nd);
		if (dte_domains[h]) {
			/* hash collision */
			while (dte_domains[lo])
				lo++;
			dte_domains[lo] = d[i];
			lo++;
			t = dte_domains[h];
			while (t->hash_next) {
				t = t->hash_next;
			}
			t->hash_next = d[i];
		} else
			dte_domains[h] = d[i];
	}
	kfree(d);
	return 0;
}

/*
 * set up the gateways for auto switches
 * in 2.3.28, we then removed the auto ea's from domain->ea.
 * since we've already set up hashes, and since the overhead
 * of having the ea's seems minimal, maybe we should just
 * keep them in there?
 *
 * The easiest alternative would be to switch the policy language
 * to separate autos from execs.  Maybe I should do that?
 */
int dte_setup_gateways(void)
{
	struct dte_domain_t *d;
	struct dte_ep *ep;
	struct dte_gateway *gw;
	int h, i, j, k, lo, n;

	for (i=0; i<num_dte_domains; i++) {
		d = dte_domains[i];
		/* calculate number of gateways */
		for (j=0; j<d->num_ea; j++)
			if (d->ea[j].access == DTE_AUTO)
				d->num_gw += d->ea[j].other_domain->num_ep;
		/* allocate the space */
		d->gw = kmalloc(d->num_gw * sizeof(struct dte_gateway), GFP_KERNEL);
		if (!d->gw)
			DTE_NO_MEM("dte_setup_gateways");
		memset(d->gw,0,d->num_gw * sizeof(struct dte_gateway));
		/* set up the hash table
		* note that auto access to two domains which share an entry point
		* amounts to undefined (well, defined by hash function behavior)
		* behavior.  This will be warned against by the policy setup GUI
		 */
		lo = 0;
		for (j=0; j<d->num_ea; j++)
			if (d->ea[j].access == DTE_AUTO) {
				ep = d->ea[j].other_domain->ep;
				n  = d->ea[j].other_domain->num_ep;
				for (k=0; k<n; k++) {
					h = dte_hash(ep[k].type, d->num_gw);
					gw = &d->gw[h];
					while (gw->type && gw->hash_next)
						gw = gw->hash_next;
					if (gw->type) {
						while (d->gw[lo].type)
							lo++;
						/*                  d->gw[lo] = kmem_cache_alloc();*/
						gw->hash_next = &d->gw[lo];
						/*                  lo++;*/
						d->gw[lo].type = ep[k].type;
						d->gw[lo].domain = d->ea[j].other_domain;
						d->gw[lo].hash_next = NULL;
						lo++;
					} else {
						d->gw[h].type = ep[k].type;
						d->gw[h].domain = d->ea[j].other_domain;
						d->gw[h].hash_next = NULL;
					}
				}
			}
	}
	return 0;
}

#ifdef CONFIG_DTE_VERBOSE
static void dte_show_assigns(struct dte_map_node *n, int tab)
{
	int i;
	char line[40];

	if (!n) return;
	for (i=0; i<tab && i<39; i++)
		line[i] = ' ';
	line[i]='\0';
	printk(KERN_NOTICE "%sname: %s len %d\n",line,n->name,n->namelen);
	printk(KERN_NOTICE "%setype: %s\n",line,n->etype ? n->etype : "");
	printk(KERN_NOTICE "%sutype: %s\n",line,n->utype ? n->utype : "");
	printk(KERN_NOTICE "%s%d children%c\n",line,n->num_kids,
			n->num_kids ? ':' : '.');
	for (i=0; i<n->num_kids; i++)
		dte_show_assigns(n->kids[i],tab+2);
}

void show_dte(void)
{
	int i, j;
	struct dte_domain_t *d;

	printk("types:\n");
	for (i=0; i<num_dte_types; i++)
		printk("%s\n", dte_types[i]);
	printk("paths:\n");
	for (i=0; i<num_dte_paths; i++)
		printk("%s\n", dte_paths[i]);
	printk("domains:\n");
	for (i=0; i<num_dte_domains; i++) {
		d = dte_domains[i];
		printk("domain %d is %s.\n", i, d->name);
		printk(" %d entry points: ", d->num_ep);
		for (j=0; j<d->num_ep; j++)
			printk("%s ", d->ep[j].type);
		printk("\n");
		printk(" %d type accesses: ", d->num_ta);
		for (j=0; j<d->num_ta; j++)
			printk("%d->%s ",(int)d->ta[j].access, d->ta[j].type);
		printk("\n");
		printk(" %d domain access: ",d->num_ea);
		for (j=0; j<d->num_ea; j++)
			printk("%d->%s ",(int)d->ea[j].access, d->ea[j].other_domain->name);
		printk("\n");
		printk(" %d gateways: ",d->num_gw);
		for (j=0; j<d->num_gw; j++) {
			printk("j is %d.\n", j);
			if (!d->gw) printk("gateway array null.\n");
			if (!d->gw[j].domain) printk("gateway's domain is null.\n");
			if (!d->gw[j].domain->name) printk("gw.domain->name is null.\n");
			printk("%s->%s ",d->gw[j].type, d->gw[j].domain->name);
		}
		printk("\n");
		printk(" %d signals: ",d->num_sa);
		for (j=0; j<d->num_sa; j++)
			printk("%d->%s ",d->sa[j].signal,
					(d->sa[j].recv_domain==d) ? "all" : d->sa[j].recv_domain->name);
		printk("\n\n");
	}
	printk("Type assignments:\n");
	dte_show_assigns(dte_root_mapnode,2);
}
#endif

/*
 * fill up the buffer from file.  We save the current data sitting
 * after stat->blin, so the first line will be complete.
 *
 * Since we might be in the middle of reading a line, we set
 * stat->mark to the previous end of buffer, so we can continue
 * at mark+1.
 * tmplen is the size from start of last line to end of buffer (before read)
 * newlen is numchars read in
 */
static int dte_fread_buf(struct dte_fdata *stat)
{
	mm_segment_t *fs;
	int i, tmplen, newlen;

	DTE_DEBUG("dte_fread_buf: starting.\n");
	if (stat->blin == stat->buffer) {
		printk(KERN_NOTICE "dte_fread_buf: error: current line is size of buffer.\n");
		/* probably not the safest way to handle it, but gotta think what is: */
		printk(KERN_NOTICE "skipping this line...\n");
		stat->blin = stat->buffer+stat->buflen;
	}
	if (stat->elin > stat->blin)
		tmplen = stat->elin-stat->blin;
	else
		tmplen=0;
	fs = &get_fs();
	set_fs(KERNEL_DS);
	/* we assume stat.blin points to the beginning of the unfinished line */
	/*   memset(stat.buffer,0,4096);*/
	for (i=0; i<tmplen; i++)
		*(stat->buffer+i) = *(stat->blin+i);
	stat->buflen = tmplen;
	newlen = stat->fin->f_op->read(stat->fin,stat->buffer+tmplen,
			4096-tmplen, &stat->fin->f_pos);
	stat->numbufs++;
	if (newlen <= 4096-tmplen) {
		stat->eof = 1;
		if (newlen<=0)
			return 0;
	}
	stat->buflen += newlen;
	set_fs(*fs);
	stat->blin = stat->buffer;
	stat->mark = stat->buffer+tmplen;
	DTE_DEBUG("dte_fread_buf: done.\n");
	return 1;
}

/*
 * puts stat.elin at the end of the line started with stat.blin
 * skips blank lines or comments
 * comments are not allowed to begin mid-line
 * returns 0 if the line is incomplete (iow more of the file needs
 * to be read into stat.buffer)
 */
static int dte_find_endline(struct dte_fdata *stat)
{
	char c;
	int reread = 0;

start:
	stat->elin = stat->blin;
	c = *(stat->elin);
	while (c==' ' || c=='\t') {
		if (stat->elin >= stat->buffer+stat->buflen) {
			if (reread)
				return 0;
			if (!dte_fread_buf(stat)) {
				printk(KERN_NOTICE "dte_find_endline: premature end of file.\n");
				return 0;
			}
			reread++;
			stat->elin = stat->mark;
		}
		c = *(++stat->elin);
	}
	if (c=='#' || c=='\n') {
		/* go to start of next line */
		while (c!='\n') {
			if (stat->elin >= stat->buffer+stat->buflen) {
				if (reread)
					return 0;
				if (!dte_fread_buf(stat)) {
					printk(KERN_NOTICE "dte_find_endline: premature end of file.\n");
					return 0;
				}
				reread++;
				stat->elin = stat->mark;
			}
			c = *(++stat->elin);
		}
		stat->blin = stat->elin+1;
		goto start;
	}

	while (c!='\n') {
		if (stat->elin >= stat->buffer+stat->buflen) {
			if (reread)
				return 0;
			if (!dte_fread_buf(stat)) {
				printk(KERN_NOTICE "dte_find_endline: premature end of file.\n");
				return 0;
			}
			reread++;
			stat->elin = stat->mark;
		}
		c = *(++stat->elin);
	}
	return 1;
}

/*
* places stat->elin at true eol, and makes sure the whole line is read
* into the buffer in the process.
*
* true eol here means we take into account '\'.
*
* also sets blin if the incoming blin was for a comment line
*/
static int dte_find_true_eol(struct dte_fdata *stat)
{
	char c;
	int reread = 0;

start:
	stat->elin = stat->blin;
	c = *(stat->elin);
	while (c==' ' || c=='\t') {
		if (stat->elin >= stat->buffer+stat->buflen) {
			if (reread)
				return 0;
			if (!dte_fread_buf(stat)) {
				printk(KERN_NOTICE "findtrueeol: premature end of file.\n");
				return 0;
			}
			reread++;
			stat->elin = stat->mark;
		}
		c = *(++stat->elin);
	}
	if (c=='#' || c=='\n') {
		/* go to start of next line */
		while (c!='\n') {
			if (stat->elin >= stat->buffer+stat->buflen) {
				if (reread)
					return 0;
				if (!dte_fread_buf(stat)) {
					printk(KERN_NOTICE "findtrueeol: premature end of file.\n");
					return 0;
				}
				reread++;
				stat->elin = stat->mark;
			}
			c = *(++stat->elin);
		}
		stat->blin = stat->elin+1;
		goto start;
	}

	/* no comments allowed between '\'-connected lines */
	while (1) {
		while (c!='\n' && c!='\\') {
			if (stat->elin >= stat->buffer+stat->buflen) {
				if (reread)
					return 0;
				if (!dte_fread_buf(stat)) {
					printk(KERN_NOTICE "findtrueeol: premature end of file.\n");
					return 0;
				}
				reread++;
				stat->elin = stat->mark;
			}
			c = *(++stat->elin);
		}
		if (c=='\n')
			return 1;
		/* find end of this line, go to first char on next line */
		while (c!='\n') {
			if (stat->elin >= stat->buffer+stat->buflen) {
				if (reread)
					return 0;
				if (!dte_fread_buf(stat)) {
					printk(KERN_NOTICE "findtrueeol: premature end of file.\n");
					return 0;
				}
				reread++;
				stat->elin = stat->mark;
			}
			c = *(++stat->elin);
		}
		c = *(++stat->elin);
	}
	BUG();
}


static int read_types(struct dte_fdata *stat)
{
	char *c, *ce;

	stat->state = DTE_STATE_TYPES;
	c = stat->blin;
	if (!dte_find_true_eol(stat)) {
		printk(KERN_ERR "DTE reading policy: error reading 'types' line.\n");
		return -1;
	}
	/* we know there are now no comments, full line is in buffer, and
	* stat->elin is eol after all '\'-connected lines. */
	while (*c==' ' || *c=='\t') c++;
	if (strncmp(c,"types ",6))
		return -1;
	c += 6;
	while (1) {
		while (*c==' ' || *c=='\t') c++;
		if (*c=='\n') {
			stat->blin = stat->elin+1;
			stat->state = DTE_STATE_DOMAINS;
			/* done reading types */
			return 0;
		}
		ce = c;
		while (*ce!='\\' && *ce!=' ' && *ce!='\t' && *ce!='\n')
			ce++;
		if (c!=ce) {
			/* regardless what ce is, there's a word between c and ce-1 */
			if (dte_add_typename(c,ce)) {
				*ce='\0';
				printk(KERN_ERR "Error adding typename %s.\n", c);
				return -1;
			}
		}
		if (*ce=='\\') {
			/* continue on the next line */
			do { ce++;} while (*ce!='\n');
		} else if (*ce=='\n') {
			/* end of line, end of types */
			if (ce!=stat->elin) {
				printk(KERN_ERR "DTE: bad input on types line.\n");
				return -1;
			}
			/* set up for the next stuff to be read in */
			stat->blin = stat->elin+1;
			stat->state = DTE_STATE_DOMAINS;
			/* done reading types */
			return 0;
		}
		c = ce+1;
	}
	/* not reached */
	BUG();
}

static int read_domains(struct dte_fdata *stat)
{
	char *c, *ce;

	if (stat->state != DTE_STATE_DOMAINS) {
		printk(KERN_ERR "DTE: domains line out of order in config file.\n");
		return -1;
	}

	c = stat->blin;
	if (!dte_find_true_eol(stat)) {
		printk(KERN_ERR "DTE: Error finding eol for domains line.\n");
		return -1;
	}
	/* we know there are now no comments, full line is in buffer, and
	* stat->elin is eol after all '\'-connected lines. */
	while (*c==' ' || *c=='\t') c++;
	if (strncmp(c,"domains ",6))
		return -1;
	c += 8;
	while (1) {
		while (*c==' ' || *c=='\t') c++;
		if (*c=='\n') {
			stat->blin = stat->elin+1;
			stat->state = DTE_STATE_DEFS;
			return 0;
		}
		ce = c;
		while (*ce!='\\' && *ce!=' ' && *ce!='\t' && *ce!='\n')
			ce++;
		if (c!=ce) {
			/* regardless what ce is, there's a word between c and ce-1 */
			if (dte_add_domain(c,ce)) {
				*ce = '\0';
				printk(KERN_ERR "DTE: Error adding domain name %s.\n", c);
				return -ENOMEM;
			}
		}
		if (*ce=='\\') {
			/* continue on the next line */
			do { ce++; } while (*ce!='\n');
		} else if (*ce=='\n') {
			/* end of line, end of types */
			if (ce!=stat->elin) {
				printk(KERN_ERR "DTE: Error: reached eol on domains line.\n");
				return -1;
			}
			/* set up for the next stuff to be read in */
			stat->blin = stat->elin+1;
			stat->state = DTE_STATE_DEFS;
			return 0;
		}
		c = ce+1;
	}
	/* not reached */
	BUG();
}

static int read_def_d(struct dte_fdata *stat)
{
	char *c, *ce;

	c = stat->blin;
	while (*c==' ' || *c=='\t')
		c++;
	if (strncmp(c,"default_d ",10)) {
		printk(KERN_ERR "DTE: read_def_d: no default domain here!\n");
		return -1;
	}

	c += 10;
	while (*c==' ' || *c=='\t')
		c++;
	ce = c;
	while (*ce!=' ' && *ce!='\t' && *ce!='\n')
		ce++;
	dte_init_domain = dte_get_domain(c,ce);
	if (!dte_init_domain) {
		*ce = '\0';
		printk(KERN_ERR "DTE: default domain %s does not exist.\n", c);
		return -1;
	}
	default_domain = dte_init_domain;
	stat->blin = stat->elin+1;
	return 0;
}

static int read_def_t(struct dte_fdata *stat, int w)
{
	char *c, *ce, *t;

	c = stat->blin;
	while (*c==' ' || *c=='\t')
		c++;
	if (strncmp(c,"default_",8) || *(c+9)!='t') {
		printk(KERN_ERR "DTE: read_def_t: no default type here!\n");
		return -1;
	}

	c += 11;
	while (*c==' ' || *c=='\t')
		c++;
	ce = c;
	while (*ce!=' ' && *ce!='\t' && *ce!='\n')
		ce++;
	if (!(t = dte_get_type(c,ce))) {
		*ce = '\0';
		printk(KERN_ERR "DTE: Bad default type %s, %d.\n", c, w);
		return -1;
	}

	if (!num_dte_map_nodes) {
		dte_root_mapnode = kmem_cache_alloc(dte_map_cache, GFP_KERNEL);
		num_dte_map_nodes++;
		dte_root_mapnode->name = kmalloc(2,GFP_KERNEL);
		sprintf(dte_root_mapnode->name, "/");
		dte_root_mapnode->namelen = 1;
		dte_root_mapnode->etype     = NULL;
		dte_root_mapnode->utype     = NULL;
		dte_root_mapnode->num_kids  = 0;
		dte_root_mapnode->hash_next = NULL;
		dte_root_mapnode->kids      = NULL;
	}

	switch (w) {
		case 1: dte_root_mapnode->etype =  t;
				  DTE_DEBUG2("dte_root_etype set to %s.\n",t);
				  break;
		case 2: dte_root_mapnode->utype =  t;
				  DTE_DEBUG2("dte_root_utype set to %s.\n",t);
				  break;
		case 3: dte_root_mapnode->etype = dte_root_mapnode->utype = t;
				  DTE_DEBUG2("dte_root_rtype set to %s.\n",t);
				  break;
		default: printk(KERN_NOTICE "read_def_t: this can't be. w!={1,2,3}.\n");
					return 1;
	}
	stat->blin = stat->elin+1;
	stat->state = DTE_STATE_SPECD;  /* might be one more def_t, that's ok */
	return 0;
}

static unsigned char dte_convert_ta(char c)
{
	switch(c) {
		default : return 0;
		case 'R':
		case 'r': return (unsigned char) DTE_FR;
		case 'W':
		case 'w': return (unsigned char) DTE_FW;
		case 'x':
		case 'X': return (unsigned char) DTE_FX;
		case 'a':
		case 'A': return (unsigned char) DTE_FA;
		case 'd': /* d: directory descend */
		case 'D': return (unsigned char) DTE_DX;
		case 'c':
		case 'C': return (unsigned char) DTE_DW;
		case 'l': /* l: as in ls - yeah, i'm reaching */
		case 'L': return (unsigned char) DTE_DR;
	}
}

#define DTE_P_ERR(which) printk(KERN_NOTICE \
		"read_specd: no opening paren for %s.\n",which);

#define MEM_ERROR(which) printk(KERN_NOTICE \
		"read_specd: insufficent memory for %s.\n",which);

#define DTE_W_ERR(wnum,wdom) printk(KERN_NOTICE \
		"read_specd: wrong number of %s specified for domain %s.\n",\
		wnum,wdom);

static int read_specd(struct dte_fdata *stat)
{
	char *c, *ce;
	int h, n, p, w;
	int lo, state=0;
	struct dte_domain_t *d;
	int have_num;

	if (!dte_find_true_eol(stat)) {
		printk(KERN_ERR "DTE: Error finding eol for specd line.\n");
		return -1;
	}
	c = stat->blin;
	while (*c==' ' || *c=='\t')
		c++;
	if (strncmp(c,"spec_domain ",12)) {
		/* debug */
		int i;
		printk(KERN_NOTICE "\nHere's the current line:\n");
		for (i=0; i<20; i++)
			printk(KERN_NOTICE "%c", *(c+i));
		printk(KERN_NOTICE "\n\n");
		printk(KERN_ERR "read_specd: no domain spec here...\n");
		return -1;
	}

	w = lo = p = 0;
	d = NULL;
	c += 12;
	have_num=0;
	while (state < 5) {
		/* first the domain name being defined */
		while (*c==' ' || *c=='\t')
			c++;
		if (*c=='\\') {
			do {c++;} while (*c!='\n');
			c++;
			continue;
		}
		if (state>0 && *c==')') {
			switch(state) {
				default: break;
				case 1: if (w!=d->num_ep) {
							  DTE_W_ERR("entry points",d->name);
							  return -1;
						  }
						  break;
				case 2: if (w!=d->num_ta) {
							  DTE_W_ERR("type accesses",d->name);
							  return -1;
						  }
						  break;
				case 3: if (w!=d->num_ea) {
							  DTE_W_ERR("domain accesses",d->name);
							  return -1;
						  }
						  break;
				case 4: if (w!=d->num_sa) {
							  DTE_W_ERR("signal accesses",d->name);
							  return -1;
						  }
						  break;
			}
			state++;
			c++;
			have_num = p = 0;
			continue;
		}
		if (*c=='(') {
			if (p) {
				printk(KERN_ERR "read_specd:  unmatched '('.\n");
				return -1;
			}
			if (state==0) {
				/* easier to have it always or never allowed, and unnecessary */
				printk(KERN_ERR "read_specd: no '(' for spec_domain name.\n");
				return -1;
			}
			p++;
			c++;
			continue;
		}
		switch(state) {
			/* We're sure now that c sits on first char of next valid word */
			case 0: ce = c;
					  while (*ce!=' '&&*ce!='\t'&&*ce!=')'&&*ce!='\n'&&*ce!='\\')
						  ce++;
					  d = dte_get_domain(c,ce);
					  if (!d) {
						  *ce = '\0';
						  printk(KERN_ERR "DTE: dte_specd: invalid domain %s.\n", c);
						  return -1;
					  }
					  c = ce;  /* ce is the char *after* word. */
					  state++;
					  break;

			case 1: ce = c;
					  if (!p) DTE_P_ERR("entry points");
					  while (*ce!=' '&&*ce!='\t'&&*ce!=')'&&*ce!='\n'&&*ce!='\\')
						  ce++;
					  if (!have_num) {
						  n = atoi(c,ce);
						  d->num_ep = n;
						  d->ep = (struct dte_ep*) kmalloc(n*sizeof(struct dte_ep),
								  GFP_KERNEL);
						  if (!d->ep) {
							  MEM_ERROR("entry points");
							  return -ENOMEM;
						  }
						  memset(d->ep,0,n*sizeof(struct dte_ep));
						  lo = w = 0;  /* w is which one is being defined */
						  have_num++;
						  c = ce;
					  } else {
						  struct dte_ep *e;
						  char *t;

						  if (w >= d->num_ep) {
							  DTE_W_ERR("entry points",d->name);
							  return -1;
						  }
						  t = dte_get_type(c,ce);
						  if (!t) {
							  *ce = '\0';
							  printk(KERN_NOTICE "at domain %s gateway %s.\n",
									  d->name, c);
							  return -1;
						  }
						  h = dte_hash(t,d->num_ep);
						  e = &d->ep[h];
						  while (e->type && e->hash_next)
							  e = e->hash_next;
						  if (e->type) {
							  while (d->ep[lo].type) lo++;
							  d->ep[lo].type = t;
							  e->hash_next = &d->ep[lo];
							  lo++;
						  } else
							  e->type = t;
						  w++;
						  c = ce;
					  }
					  break;
			case 2: ce = c;
					  if (!p) DTE_P_ERR("type access specs");
					  while (*ce!=' '&&*ce!='\t'&&*ce!=')'&&*ce!='\n'&&*ce!='\\')
						  ce++;
					  if (!have_num) {
						  n = atoi(c,ce);
						  d->num_ta = n;
						  d->ta = (struct dte_ta*) kmalloc(n*sizeof(struct dte_ta),
								  GFP_KERNEL);
						  if (!d->ta) {
							  MEM_ERROR("entry points");
							  return -ENOMEM;
						  }
						  memset(d->ta,0,n*sizeof(struct dte_ta));
						  have_num++;
						  lo = w = 0;  /* w is which one is being defined */
						  c = ce;
					  } else {
						  struct dte_ta *ta;
						  char *t;
						  unsigned char access = 0;

						  if (w >= d->num_ta) {
							  DTE_W_ERR("type access",d->name);
							  return -1;
						  }
						  /* must be exactly:
						  * rwx->typename (rwx can be any type access combo)
							*/
						  while (*c!='-') {
							  access |= dte_convert_ta(*c);
							  c++;
						  }
						  c += 2; /* get passed '->' */
						  t = dte_get_type(c,ce);
						  if (!t) {
							  printk(KERN_NOTICE "bad type in domain spec %s.\n",
									  d->name);
							  return -1;
						  }
						  h = dte_hash(t,d->num_ta);
						  ta = &d->ta[h];
						  while (ta->type && ta->hash_next)
							  ta = ta->hash_next;
						  if (ta->type) {
							  while (d->ta[lo].type) lo++;
							  d->ta[lo].type = t;
							  d->ta[lo].access = access;
							  ta->hash_next = &d->ta[lo];
							  lo++;
						  } else {
							  ta->type = t;
							  ta->access = access;
						  }
						  w++;
						  c = ce;
					  }
					  break;
			case 3: ce = c;
					  if (!p) DTE_P_ERR("domain access specs");
					  if (!have_num) {
						  while (*ce!=' '&&*ce!='\t'&&*ce!=')'&&*ce!='\n'&&*ce!='\\')
							  ce++;
						  n = atoi(c,ce);
						  d->num_ea = n;
						  d->ea = (struct dte_ea*) kmalloc(n*sizeof(struct dte_ea)
								  ,GFP_KERNEL);
						  if (!d->ea) {
							  MEM_ERROR("domain access");
							  return -ENOMEM;
						  }
						  memset(d->ea,0,n*sizeof(struct dte_ea));
						  have_num++;
						  c = ce;
						  lo = w = 0;
					  } else {
						  struct dte_ea *ea;
						  struct dte_domain_t *dest;
						  unsigned char access;

						  if (w >= d->num_ea) {
							  DTE_W_ERR("domain access",d->name);
							  return -1;
						  }
						  /* first comes 'exec' or 'auto' */
						  ce = c;
						  while (*ce!='-') ce++;
						  if (!strncmp(c,"exec",4) && (ce-c==4))
							  access = DTE_EXEC;
						  else access = DTE_AUTO;
						  c = ce += 2;
						  while (*ce!=' '&&*ce!='\t'&&*ce!=')'&&*ce!='\n'&&*ce!='\\')
							  ce++;
						  dest = dte_get_domain(c,ce);
						  if (!dest) {
							  printk(KERN_NOTICE "bad d in domain accesses for %s.\n",
									  d->name);
							  return -1;
						  }
						  h = dte_hash(dest->name,d->num_ea);
						  ea = &d->ea[h];
						  while (ea->other_domain && ea->hash_next)
							  ea = ea->hash_next;
						  if (ea->other_domain) {
							  while (d->ea[lo].other_domain) lo++;
							  d->ea[lo].other_domain = dest;
							  d->ea[lo].access = access;
							  ea->hash_next = &d->ea[lo];
							  lo++;
						  } else {
							  ea->other_domain = dest;
							  ea->access = access;
						  }
						  w++;
						  c = ce;
					  }
					  break;
			case 4: ce = c;
					  if (!p) DTE_P_ERR("signal access specs");
					  if (!have_num) {
						  while (*ce!=' '&&*ce!='\t'&&*ce!=')'&&*ce!='\n'&&*ce!='\\')
							  ce++;
						  n = atoi(c,ce);
						  d->num_sa = n;
						  d->sa = (struct dte_sa*) kmalloc(n*sizeof(struct dte_sa),
								  GFP_KERNEL);
						  if (!d->sa) {
							  MEM_ERROR("signal access");
							  return -ENOMEM;
						  }
						  memset(d->sa,0,n*sizeof(struct dte_sa));
						  lo = w = 0;
						  c = ce;
						  have_num++;
					  } else {
						  struct dte_sa *sa;
						  struct dte_domain_t *dest;
						  int signal;

						  if (w >= d->num_sa) {
							  DTE_W_ERR("signal access",d->name);
							  return -1;
						  }
						  ce = c;
						  while (*ce!='-') ce++;
						  signal = atoi(c,ce);
						  c = ce +=2;
						  while (*ce!=' '&&*ce!='\t'&&*ce!=')'&&*ce!='\n'&&*ce!='\\')
							  ce++;
						  if (*c!='0' || ce-c>1) {
							  dest = dte_get_domain(c,ce);
							  if (!dest || dest==d) {
								  printk(KERN_NOTICE "bad d in signal access for %s.\n",
										  d->name);
								  return -1;
							  }
						  } else
							  dest = d;
						  h = dte_hash(dest->name, d->num_sa);
						  sa = &d->sa[h];
						  while (sa->recv_domain && sa->hash_next)
							  sa = sa->hash_next;
						  if (sa->recv_domain) {
							  while (d->sa[lo].recv_domain) lo++;
							  d->sa[lo].recv_domain = dest;
							  d->sa[lo].signal = signal;
							  sa->hash_next = &d->sa[lo];
							  lo++;
						  } else {
							  sa->recv_domain = dest;
							  sa->signal = signal;
						  }
						  w++;
						  c = ce;
					  }
					break;
			default:	printk("bad case.\n");
					return -1;
					break;
		}
	}

	stat->blin = stat->elin+1;
	return 0;
}

/* format of ta:
* assign -X type path1 path2 ... pathn
* where x is e,r or u.  no combos.
* line may be split by \
 */
static int read_ta(struct dte_fdata *stat)
{
	char *t, *c, *ce;
	char *p;
	char assign;
	struct dte_map_node *tmpmap;

	if (!dte_find_true_eol(stat)) {
		printk(KERN_ERR "DTE: Error finding eol for ta line.\n");
		return -1;
	}
	c = stat->blin;
	while (*c==' ' || *c=='\t')
		c++;
	if (strncmp(c,"assign ",7)) {
		printk(KERN_ERR "read_ta: no type assign here...\n");
		return -1;
	}

	while (*c!='-') c++;
	c++;
	assign = *c;
	ce = c += 2;

	while (*ce!=' '&&*ce!='\t'&&*ce!=')'&&*ce!='\n'&&*ce!='\\')
		ce++;

	t = dte_get_type(c,ce);
	if (!t) {
		ce = '\0';
		printk(KERN_ERR "dte: bad type %s in type assignment.\n", c);
		return -1;
	}

	while (1) {
		c = ce;
		while (*c==' ' || *c == '\t') c++;
		if (*c=='\n') {
			stat->blin = stat->elin+1;
			return 0;
		}
		if (*c=='\\') {
			do {c++;} while (*c!='\n');
			c++;
			continue;
		}
		ce = c;

		while (*ce!=' '&&*ce!='\t'&&*ce!=')'&&*ce!='\n'&&*ce!='\\')
			ce++;
		p = dte_add_pathname(c,ce);

		tmpmap = dte_find_map_node_create(p);
		if (!tmpmap) {
			printk(KERN_ERR "DTE: couldn't creat map node at read_ta %s.\n",
					p);
			return -1;
		}

		switch(assign) {
			case 'e': tmpmap->etype = t; break;
			case 'u': tmpmap->utype = t; break;
			default : tmpmap->etype = tmpmap->utype = t;
						 break;
		}

		c = ce;
	}
	BUG();
}

int dte_sort_mntrs(void)
{
	struct dte_mntr **tmpr, *r, *dest;
	int i, h, lo;

	tmpr = kmalloc(num_dte_mount_r*sizeof(struct dte_mntr **), GFP_KERNEL);
	if (!tmpr) {
		printk(KERN_ERR "DTE: not enough memory for mount restrictions.\n");
		return -ENOMEM;
	}
	memset(tmpr,0,num_dte_mount_r*sizeof(struct dte_mntr **));
	r = dte_mount_r[0];
	lo = 0;
	for (i=0; i<num_dte_mount_r; i++) {
		h = dte_hash(r->summ, num_dte_mount_r);
		dest = tmpr[h];
		if (!dest) {
			tmpr[h] = r;
		} else {
			while (dest->hash_next)
				dest = dest->hash_next;
			while (tmpr[lo])
				lo++;
			tmpr[lo] = r;
			dest->hash_next = r;
			lo++;
		}

		dest = r->hash_next;
		r->hash_next = NULL;
		r = dest;
	}
	kfree(dte_mount_r);
	dte_mount_r = tmpr;
	return 0;
}

int read_mntr(struct dte_fdata *stat)
{
	char *c, *ce;
	char how;
	int maj, min;
	struct dte_mntr *r;
	int len;

	DTE_DEBUG("read_mntr: starting.\n");
	if (!dte_find_true_eol(stat)) {
		printk(KERN_ERR "DTE: Error finding eol for mnt restrict line.\n");
		return -1;
	}
	c = stat->blin;
	while (*c==' ' || *c=='\t')
		c++;
	if (!strncmp(c,"pretend ",8))
		how = 'p';
	else if (!strncmp(c,"restrict ",9))
		how = 'r';
	else {
		printk(KERN_NOTICE "read_mntr: no mount restriction here...\n");
		return -1;
	}
	while (*c!=' ' && *c!='\t')
		c++;
	while (*c==' ' || *c=='\t')
		c++;
	/* 
	 * in this case, we'll always be comparing copies of path/dev names,
	 * so we'll never be able to compare by array offsets.  So we just
	 * allocate mem for device names right here
	 *
	 * we don't allow '\' to split lines in this case - shouldn't really be
	 * necessary...
	 *
	 * First, get device:
	 */
	while (*c==' ' && *c=='\t') c++;
	ce = c;
	while (*ce!=' ' && *ce!='\t') ce++;
	maj = atoi(c,ce);
	c=++ce;
	while (*c==' ' && *c=='\t') c++;
	ce = c;
	while (*ce!=' ' && *ce!='\t') ce++;
	min = atoi(c,ce);

	/* grab a mntr struct from cache */
	r = kmem_cache_alloc(dte_mntr_cache, GFP_KERNEL);
	if (!r)
		DTE_NO_MEM("dte_read_mntr");
	r->maj   = (unsigned char)maj;
	r->min   = (unsigned char)min;
	sprintf(r->summ, "%3d,%3d",maj,min);

	c = ++ce;
	while (*c==' ' || *c=='\t')
		c++;
	ce = c;
	while (*ce!=' ' && *ce!='\t' && *ce!='\n' && *ce!='\0')
		ce++;
	len = ce-c;
	r->path = (char *)kmalloc(NAME_ALLOC_LEN(len), GFP_KERNEL);
	if (!r->path)
		DTE_NO_MEM("dte_read_mntr");
	memcpy(r->path,c,len);
	r->path[len] = 0;

	r->how = how;
	num_dte_mount_r++;
	r->hash_next = dte_mount_r[0];
	dte_mount_r[0]  = r;

	stat->blin = stat->elin+1;

	DTE_DEBUG("read_mntr: returning.\n");
	return 0;
}

static void free_map_nodes(struct dte_map_node *r) {
	int i;

	if (!r)
		return;
	
	for (i=0; i<r->num_kids; i++) {
		free_map_nodes(r->kids[i]);
	}
	kfree(r->kids);
	kfree(r->name);
	kmem_cache_free(dte_map_cache, r);
}

static void free_unhashed_map_nodes(struct dte_map_node *r) {
	struct dte_map_node *tmp, *tmp2;

	if (!r)
		return;
	
	if (r->num_kids) {
		tmp = r->kids[0];
		while (tmp) {
			tmp2 = tmp->hash_next;
			free_unhashed_map_nodes(tmp);
			tmp = tmp2;
		}
	}
	kfree(r->kids);
	kfree(r->name);
	kmem_cache_free(dte_map_cache, r);
}

/*
 * Free dte memory in case of failure during setup.
 * The depth bit may seem confusing, but it just has to do with how
 * far we've gotten into the setup process.
 * If depth > 3, then we're past read_dte_config, and called from
 * outside this file!
 *
 * If depth <= 5, we haven't hashed mapnodes yet, so, at each level,
 * mapnodes are actually a linked list, with hash_next being next.
 */
void free_dte_memory(int depth) {
	int i;
	struct dte_mntr *tmpr, *tmpr2;
	struct dte_domain_t *d;

	if (depth>=5)
		/* we've passed dte_setup_gateways: free gateways */
		for (i=0; i<num_dte_domains; i++)
			if (dte_domains[i]->gw)
				kfree(dte_domains[i]->gw);

	if (depth >= 3) {
		if (dte_mount_r) {
			/* the mntrs are not yet sorted */
			tmpr = dte_mount_r[0];
			while (tmpr) {
				tmpr2 = tmpr->hash_next;
				kfree(tmpr->path);
				kmem_cache_free(dte_mntr_cache, tmpr);
				tmpr = tmpr2;
			}
			kfree(dte_mount_r);
		}

		/* free domain specifications */
		for (i=0; i<num_dte_domains; i++) {
			d = dte_domains[i];
			if (d->ep)
				kfree(d->ep);
			if (d->ta)
				kfree(d->ta);
			if (d->ea)
				kfree(d->ea);
			if (d->sa)
				kfree(d->sa);
			if (d->name)
				kfree(d->name);
			kmem_cache_free(dte_domain_cache, dte_domains[i]);
		}

		/* free the typename and pathname slabs */
		for (i=0; i<num_dte_type_name_slabs; i++) {
			kmem_cache_free(dte_type_cache, dte_type_name_backup[i]);
		}
		for (i=0; i<num_dte_path_name_slabs; i++) {
			kmem_cache_free(dte_path_cache, dte_path_name_backup[i]);
		}

		/* free the mapnodes */
		if (depth <= 5)
			free_unhashed_map_nodes(dte_root_mapnode);
		else
			free_map_nodes(dte_root_mapnode);
	}

	if (depth >= 2) {
		if (dte_types) kfree(dte_types);
		if (dte_paths) kfree(dte_paths);
		if (dte_type_names) kmem_cache_free(dte_type_cache,dte_type_names);
		if (dte_path_names) kmem_cache_free(dte_path_cache,dte_path_names);
	}

	if (dte_mntr_cache) kmem_cache_destroy(dte_mntr_cache);
	if (dte_map_cache) kmem_cache_destroy(dte_map_cache);
	if (dte_sa_cache) kmem_cache_destroy(dte_sa_cache);
	if (dte_ea_cache) kmem_cache_destroy(dte_ea_cache);
	if (dte_ep_cache) kmem_cache_destroy(dte_ep_cache);
	if (dte_domain_cache) kmem_cache_destroy(dte_domain_cache);
	if (dte_path_cache) kmem_cache_destroy(dte_path_cache);
	if (dte_type_cache) kmem_cache_destroy(dte_type_cache);
	if (dte_domains) kfree(dte_domains);
}

/*
 * read_dte_config: read the config file.
 */
int read_dte_config(void)
{
	struct dte_fdata stat;
	char *c;
	int line = 0;
	int err  = 0;

	/* initialization of variables and memcaches */
	num_dte_domains = 0;
	dte_domains = kmalloc(8192,GFP_KERNEL);
	dte_type_cache = kmem_cache_create("dte_type_names",4096,0,0,NULL,NULL);
	dte_path_cache = kmem_cache_create("dte_path_names",4096,0,0,NULL,NULL);
	dte_domain_cache = kmem_cache_create("dte_domains",sizeof(struct dte_domain_t),
			0,0,NULL,NULL);
	dte_ep_cache   = kmem_cache_create("dte_ep_cache",sizeof(struct dte_ep),
			0,0,NULL,NULL);
	dte_ea_cache   = kmem_cache_create("dte_ea_cache",sizeof(struct dte_ea),
			0,0,NULL,NULL);
	dte_sa_cache   = kmem_cache_create("dte_sa_cache",sizeof(struct dte_sa),
			0,0,NULL,NULL);
	dte_map_cache  = kmem_cache_create("dte_map_cache",sizeof(struct dte_map_node),
			0,0,NULL,NULL);
	dte_mntr_cache = kmem_cache_create("dte_mount_r",sizeof(struct dte_mntr),
			0,0,NULL,NULL);
	if (!dte_type_cache || !dte_path_cache || !dte_domain_cache || !dte_ep_cache
			|| !dte_ea_cache || !dte_sa_cache || !dte_map_cache
			|| !dte_mntr_cache || !dte_domains) {
		printk(KERN_NOTICE "read_dte_config: insufficient memory for caches.\n");
		err = -ENOMEM;
		goto out_mem1;
	}

	num_dte_type_name_slabs = 0;
	num_dte_path_name_slabs = 0;
	num_dte_map_nodes = 0;
	dte_root_mapnode  = NULL;
	num_dte_types   = sizeof_dte_types = 0;
	num_dte_paths   = sizeof_dte_paths = 0;
	dte_types = kmalloc(8192,GFP_KERNEL);/* this is just an array of pointers */
	dte_paths = kmalloc(8192,GFP_KERNEL);/* into slabs coming from next two */
	dte_type_names = kmem_cache_alloc(dte_type_cache,GFP_KERNEL);
	dte_path_names = kmem_cache_alloc(dte_path_cache,GFP_KERNEL);
	if (!dte_types || !dte_paths || !dte_type_names || !dte_path_names) {
		printk(KERN_NOTICE "read_dte_config: insufficient memory for names.\n");
		err = -ENOMEM;
		goto out_mem2;
	}

	num_dte_mount_r = 0;
	dte_mount_r = kmalloc(sizeof(struct dte_mntr **), GFP_KERNEL);
	dte_mount_r[0] = NULL;

	stat.fin = filp_open("/.dte/dte.conf",00,O_RDONLY);
	if (IS_ERR(stat.fin) || stat.fin==NULL) {
		printk(KERN_ERR "Error opening /.dte/dte.conf for reading.\n");
		err = -EINVAL;
		goto out_mem2;
	}

	if (stat.fin->f_op==NULL || stat.fin->f_op->read==NULL) {
		
		printk(KERN_ERR "/.dte/dte.conf or root fs not readable.\n");
		err = -EINVAL;
		goto out_mem2;
	}

	stat.numbufs=0;
	stat.fin->f_pos = 0;
	stat.blin = stat.elin = stat.buffer+4095;
	stat.buflen = stat.eof = 0;
	dte_bitchmode = 0;

	if (dte_fread_buf(&stat)!=1) {
		printk(KERN_ERR "Error reading dte policy file.\n");
		err = -EINVAL;
		goto out_bad_fput;
	}

	while (stat.state != DTE_STATE_DONE &&
			!(stat.eof && stat.blin >= stat.buffer+stat.buflen)) {
		if (stat.blin >= stat.buffer+stat.buflen) {
			/* end of file */
			printk(KERN_ERR "Error reading dte policy file.\n");
			err = -EINVAL;
			goto out_bad_fput;
		}

		if (!dte_find_endline(&stat)) {
			err = -EINVAL;
			goto out_bad_fput;
		}
		c = stat.blin;
		while (*c==' ') c++;
		if (strncmp(c,"types ",6)==0)
			err = read_types(&stat);
		else if (strncmp(c,"domains ",8)==0) {
			err = read_domains(&stat);
			if (!err)
				err = sort_domains();
		} else if (strncmp(c,"default_d ",10)==0)
			err = read_def_d(&stat);
		else if (strncmp(c,"default_et ",11)==0)
			err = read_def_t(&stat,1);
		else if (strncmp(c,"default_ut ",11)==0)
			err = read_def_t(&stat,2);
		else if (strncmp(c,"default_rt ",11)==0)
			err = read_def_t(&stat,3);
		else if (strncmp(c,"spec_domain ",12)==0)
			err = read_specd(&stat);
		else if (strncmp(c,"assign ",7)==0)
			err = read_ta(&stat);
		else if (strncmp(c,"pretend ",8)==0)
			err = read_mntr(&stat);
		else if (strncmp(c,"restrict ",9)==0)
			err = read_mntr(&stat);
		else if (strncmp(c,"bitch\n",6)==0) {
			dte_bitchmode = 1;
			stat.blin = stat.elin+1;
		} else {
			printk(KERN_ERR "Error reading dte policy file:\n");
			printk(KERN_ERR "bad input line, line %d.\n",line);
			err = -EINVAL;
			goto out_bad_fput;
		}
		if (err) {
			*(c+12) = '\0';
			printk(KERN_ERR "DTE: error at line %d, beginning :\n%s\n",
					line, c);
			goto out_bad_fput;
		} else
			line++;
	}

	fput(stat.fin);
	
	return 0;

out_bad_fput:
	fput(stat.fin);

	free_dte_memory(3);
	return err;

out_mem2:
	free_dte_memory(2);
	return err;

out_mem1:
	free_dte_memory(1);
	return err;
}
