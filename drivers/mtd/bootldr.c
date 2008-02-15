/*
 * Read flash partition table from Compaq Bootloader
 *
 * Copyright 2001 Compaq Computer Corporation.
 *
 * $Id: bootldr.c,v 1.7 2001/11/20 19:31:37 jamey Exp $
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 * COMPAQ COMPUTER CORPORATION MAKES NO WARRANTIES, EXPRESSED OR IMPLIED,
 * AS TO THE USEFULNESS OR CORRECTNESS OF THIS CODE OR ITS
 * FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 */

/*
 * Maintainer: Jamey Hicks (jamey.hicks@compaq.com)
 */

#include <linux/kernel.h>
#include <linux/slab.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/bootmem.h>

static char *cmdline = NULL;

/*
 * Format suggested by Nicolas Pitre: mtdpart=<name1>@<size1>:<offset1>[,<name2>@<size2>:<offset2>[,...]]
 */
static int parse_mtdpart(char *cmdline, struct mtd_info *master, struct mtd_partition **pparts)
{
        int npartitions = 0;
        int i;
        char *start = cmdline;

	struct mtd_partition *parts;
	int namelen = 0;
	char *names; 
        int ret = 0;

        printk(__FUNCTION__ "{%s}\n", cmdline);
        /* first, tokenize the line, validate format, and count partitions */
        while (start) {
                char *comma_pos = strchr(start, ',');
                char *at_pos = strchr(start, '@');
                char *colon_pos = strchr(start, ':');

                if (at_pos) { 
                        *at_pos = 0;
                } else {
                        ret = -EINVAL;
                        goto out;
                }
                if (colon_pos) { 
                        *colon_pos = 0;
                } else {
                        ret = -EINVAL;
                        goto out;
                }

                if (*start) {
                        namelen += strlen(start) + 1;
                } else {
                        ret = -EINVAL;
                        goto out;
                }

                npartitions++;

                printk("\t name=%s\n", start);
                printk("\t at_pos=%s\n", at_pos+1);
                printk("\t colon_pos=%s\n", colon_pos+1);
                printk("\t comma_pos=%s\n", comma_pos);

                if (comma_pos) {
                        *comma_pos = 0;
                        start = comma_pos + 1;
                } else {
                        break;
                }


        }

        if (npartitions == 0) 
                return 0;

        /* rewind */
        start = cmdline;

        printk("npartitions=%d\n", npartitions);

	parts = kmalloc(sizeof(struct mtd_partition)*npartitions + namelen, GFP_KERNEL);
	if (!parts) {
		ret = -ENOMEM;
		goto out;
	}
	names = (char *)&parts[npartitions];
	memset(parts, 0, sizeof(struct mtd_partition)*npartitions + namelen);

        // now fill in the partition info
	for (i = 0; i < npartitions; i++) {
                char *name = start;
                int namelen = strlen(name);
                char *offset = name + namelen + 1;
                char *size = offset + strlen(offset) + 1;

		parts[i].name = names;
		names += namelen + 1;
		strcpy(parts[i].name, name);

		parts[i].size = simple_strtoul(size, NULL, 0);
		parts[i].offset = simple_strtoul(offset, NULL, 0);
		parts[i].mask_flags = 0;
		
                printk("        partition %s o=%x s=%x\n", 
                       parts[i].name, parts[i].offset, parts[i].size);

                start = size + strlen(size) + 1;
	}

	ret = npartitions;
        if (pparts)
                *pparts = parts;

 out:
        printk(__FUNCTION__ ": ret=%d\n", ret);
        return ret;
}

int parse_bootldr_partitions(struct mtd_info *master, struct mtd_partition **pparts)
{
        if (cmdline) {
                return parse_mtdpart(cmdline, master, pparts);
        } else {
                if (pparts)
                        *pparts = NULL;
                return 0;
        }
}

/*
 * Format suggested by Nicolas Pitre: mtdpart=<name1>@<size1>:<offset1>[,<name2>@<size2>:<offset2>[,...]]
 */
static int handle_mtdpart(char *mtdpart)
{
        cmdline = mtdpart;
        return 1;
}

__setup("mtdpart=", handle_mtdpart);

EXPORT_SYMBOL(parse_bootldr_partitions);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Compaq Computer Corporation");
MODULE_DESCRIPTION("Parsing code for Compaq bootldr partitions");
