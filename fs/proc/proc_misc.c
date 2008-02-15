/*
 *  linux/fs/proc/proc_misc.c
 *
 *  linux/fs/proc/array.c
 *  Copyright (C) 1992  by Linus Torvalds
 *  Copyright (c) 2005 Motorola Inc.
 *
 *  based on ideas by Darren Senn
 *
 *  This used to be the part of array.c. See the rest of history and credits
 *  there. I took this into a separate file and switched the thing to generic
 *  proc_file_inode_operations, leaving in array.c only per-process stuff.
 *  Inumbers allocation made dynamic (via create_proc_entry()).  AV, May 1999.
 *
 * Changes:
 * Fulton Green      :  Encapsulated position metric calculations.
 *			<kernel@FultonGreen.com>
 * Susan Gu          :  Calculate the slab free space for EzX
 *                      <w15879@motorola.com> 
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/tty.h>
#include <linux/string.h>
#include <linux/mman.h>
#include <linux/proc_fs.h>
#include <linux/ioport.h>
#include <linux/config.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/swap.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/signal.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/smp_lock.h>
#include <linux/seq_file.h>
#include <linux/vmalloc.h>

#include <linux/dcache.h>
#include <linux/fs.h>


#ifdef CONFIG_MEMORY_ACCOUNTING
#include <asm/page.h>
#include <asm/memory.h>
#endif
#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <asm/io.h>

#define LOAD_INT(x) ((x) >> FSHIFT)
#define LOAD_FRAC(x) LOAD_INT(((x) & (FIXED_1-1)) * 100)
/*
 * Warning: stuff below (imported functions) assumes that its output will fit
 * into one page. For some of those functions it may be wrong. Moreover, we
 * have a way to deal with that gracefully. Right now I used straightforward
 * wrappers, but this needs further analysis wrt potential overflows.
 */
extern int get_hardware_list(char *);
extern int get_stram_list(char *);
#ifdef CONFIG_MODULES
extern int get_module_list(char *);
#endif
extern int get_device_list(char *);
extern int get_filesystem_list(char *);
extern int get_exec_domain_list(char *);
extern int get_irq_list(char *);
extern int get_dma_list(char *);
extern int get_locks_status (char *, char **, off_t, int);
extern int get_swaparea_info (char *);
#ifdef CONFIG_SGI_DS1286
extern int get_ds1286_status(char *);
#endif

void proc_sprintf(char *page, off_t *off, int *lenp, const char *format, ...)
{
	int len = *lenp;
	va_list args;

	/* try to only print whole lines */
	if (len > PAGE_SIZE-512)
		return;

	va_start(args, format);
	len += vsnprintf(page + len, PAGE_SIZE-len, format, args);
	va_end(args);

	if (len <= *off) {
		*off -= len;
		len = 0;
	}

	*lenp = len;
}

static int proc_calc_metrics(char *page, char **start, off_t off,
				 int count, int *eof, int len)
{
	if (len <= off+count) *eof = 1;
	*start = page + off;
	len -= off;
	if (len>count) len = count;
	if (len<0) len = 0;
	return len;
}

static int loadavg_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int a, b, c;
	int len;

	a = avenrun[0] + (FIXED_1/200);
	b = avenrun[1] + (FIXED_1/200);
	c = avenrun[2] + (FIXED_1/200);
	len = sprintf(page,"%d.%02d %d.%02d %d.%02d %ld/%d %d\n",
		LOAD_INT(a), LOAD_FRAC(a),
		LOAD_INT(b), LOAD_FRAC(b),
		LOAD_INT(c), LOAD_FRAC(c),
		nr_running(), nr_threads, last_pid);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int uptime_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	unsigned long uptime;
	unsigned long idle;
	int len;

	uptime = jiffies;
	idle = init_task.times.tms_utime + init_task.times.tms_stime;

	/* The formula for the fraction parts really is ((t * 100) / HZ) % 100, but
	   that would overflow about every five days at HZ == 100.
	   Therefore the identity a = (a / b) * b + a % b is used so that it is
	   calculated as (((t / HZ) * 100) + ((t % HZ) * 100) / HZ) % 100.
	   The part in front of the '+' always evaluates as 0 (mod 100). All divisions
	   in the above formulas are truncating. For HZ being a power of 10, the
	   calculations simplify to the version in the #else part (if the printf
	   format is adapted to the same number of digits as zeroes in HZ.
	 */
#if HZ!=100
	len = sprintf(page,"%lu.%02lu %lu.%02lu\n",
		uptime / HZ,
		(((uptime % HZ) * 100) / HZ) % 100,
		idle / HZ,
		(((idle % HZ) * 100) / HZ) % 100);
#else
	len = sprintf(page,"%lu.%02lu %lu.%02lu\n",
		uptime / HZ,
		uptime % HZ,
		idle / HZ,
		idle % HZ);
#endif
	return proc_calc_metrics(page, start, off, count, eof, len);
}

#if defined(CONFIG_MEMORY_ACCOUNTING) && !defined(CONFIG_DISCONTIGMEM)
/*
 * We walk all the page structs and translate them to pfns
 * We get the free pages by the page count, count == 0.
 *
 * We have an array of unsigned ints of which each bit in an int 
 * represents one page in memory.
 *
 * The pfn is the index into the array, pfn 0 == index 0 (index = pfn / 32).  
 * We figure out which bit of the unsigned int to set by the following:
 * bit = pfn % 32, we then set that bit to zero to denote it's free via:
 * array[index] &= ~(1<<bit);   then we pump the data to /proc 
 */

#define BITS_PER_INT 	32
#define ONE_LINE	65
static int highwater = 0;
static int lowwater = 0;
static int mem_acc_debug = 0;
static int index = 0;

static int memmap_read_proc(char *page, char **start, off_t off,
    int count, int *eof, void *data)
{
	static  int *bitmap = NULL;
	struct page *p, *end;
	int mark;
	int len = 0;
	int free = 0;
	int total = 0;
	int i, bit, start_pfn, end_pfn, array_size = 0;
	int array_bounds, pfn = 0;

        p = NODE_MEM_MAP(0);
        start_pfn = (int)page_to_pfn(p);
	end_pfn = start_pfn + num_physpages;
        end = pfn_to_page(end_pfn);
	array_size = end_pfn - start_pfn;
	array_bounds = array_size / BITS_PER_INT;

	if (bitmap == NULL) {
		bitmap = (unsigned int *)kmalloc(array_size / sizeof (int),
	    	    GFP_KERNEL);
	}

	/*
	 * one means used, zero means free, set them all to 1,
	 * clear them as we find them free.
	 */

	for (i = 0; i < array_bounds; i++) {
		bitmap[i] = 0xffffffff;
	}

	total = free = 0;
	do {
		total++;
		pfn = page_to_pfn(p);
		if (!PageReserved(p) && page_count(p) == 0 && pfn_valid(pfn)) {
			i = pfn - start_pfn;
			bit = i % BITS_PER_INT;
			i /= BITS_PER_INT;
			free++;
			bitmap[i] &= ~(1<<bit);
			if (mem_acc_debug)
				printk("i %d bit %d pfn %d pa 0x%x c %d\n",
			    	    index, bit, pfn, page_to_phys(p),
				    page_count(p));
		}
		p++;
	} while (p < end);

	mark = total - free;
	if (mark < lowwater) {
		lowwater = mark;
	}
	if (mark > highwater) {
		highwater = mark;
	}
	if (off == 0) {
		index  = 0;
		len = sprintf(page,
	    "High water used pages %d Low water used pages %d\n",
		       highwater, lowwater);
	} else if (index == -1) {
		index = 0;
		*eof = 1;
		return 0;
	}

	for (index; index < array_bounds && len+ONE_LINE < count; index += 8) {
		len += sprintf(page + len, "%08x%08x%08x%08x%08x%08x%08x%08x\n",
		    bitmap[index], bitmap[index + 1], bitmap[index + 2],
		    bitmap[index + 3], bitmap[index + 4], bitmap[index + 5],
		    bitmap[index + 6], bitmap[index + 7]);
	}
	if (index >= array_bounds || len > count) {
		index == -1;
		kfree(bitmap);
		bitmap = NULL;
	}
	*start = page;
	return len;
}
#endif

extern atomic_t vm_committed_space;

static int meminfo_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	struct sysinfo i;
	int len;
	int pg_size, committed;

/*
 * display in kilobytes.
 */
#define K(x) ((x) << (PAGE_SHIFT - 10))
#define B(x) ((unsigned long long)(x) << PAGE_SHIFT)
	si_meminfo(&i);
	si_swapinfo(&i);
	pg_size = atomic_read(&page_cache_size) - i.bufferram ;
	committed = atomic_read(&vm_committed_space);

	len = sprintf(page, "        total:    used:    free:  shared: buffers:  cached:\n"
		"Mem:  %8Lu %8Lu %8Lu %8Lu %8Lu %8Lu\n"
		"Swap: %8Lu %8Lu %8Lu\n",
		B(i.totalram), B(i.totalram-i.freeram), B(i.freeram),
		B(i.sharedram), B(i.bufferram),
		B(pg_size), B(i.totalswap),
		B(i.totalswap-i.freeswap), B(i.freeswap));
	/*
	 * Tagged format, for easy grepping and expansion.
	 * The above will go away eventually, once the tools
	 * have been updated.
	 */
	len += sprintf(page+len,
		"MemTotal:     %8lu kB\n"
		"MemFree:      %8lu kB\n"
		"MemShared:    %8lu kB\n"
		"Buffers:      %8lu kB\n"
		"Cached:       %8lu kB\n"
		"SwapCached:   %8lu kB\n"
		"Active:       %8u kB\n"
		"Inactive:     %8u kB\n"
		"HighTotal:    %8lu kB\n"
		"HighFree:     %8lu kB\n"
		"LowTotal:     %8lu kB\n"
		"LowFree:      %8lu kB\n"
		"SwapTotal:    %8lu kB\n"
		"SwapFree:     %8lu kB\n"
		"Committed_AS: %8u kB\n",
		K(i.totalram),
		K(i.freeram),
		K(i.sharedram),
		K(i.bufferram),
		K(pg_size - swapper_space.nrpages),
		K(swapper_space.nrpages),
		K(nr_active_pages),
		K(nr_inactive_pages),
		K(i.totalhigh),
		K(i.freehigh),
		K(i.totalram-i.totalhigh),
		K(i.freeram-i.freehigh),
		K(i.totalswap),
		K(i.freeswap),
		K(committed));

	return proc_calc_metrics(page, start, off, count, eof, len);
#undef B
#undef K
}

static int version_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	extern char *linux_banner;
	int len;

	strcpy(page, linux_banner);
	len = strlen(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

extern struct seq_operations cpuinfo_op;
static int cpuinfo_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &cpuinfo_op);
}
static struct file_operations proc_cpuinfo_operations = {
	open:		cpuinfo_open,
	read:		seq_read,
	llseek:		seq_lseek,
	release:	seq_release,
};

#ifdef CONFIG_PROC_HARDWARE
static int hardware_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_hardware_list(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}
#endif

#ifdef CONFIG_STRAM_PROC
static int stram_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_stram_list(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}
#endif

extern struct seq_operations partitions_op;
static int partitions_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &partitions_op);
}
static struct file_operations proc_partitions_operations = {
	open:		partitions_open,
	read:		seq_read,
	llseek:		seq_lseek,
	release:	seq_release,
};

#ifdef CONFIG_MODULES
static int modules_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_module_list(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

extern struct seq_operations ksyms_op;
static int ksyms_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &ksyms_op);
}
static struct file_operations proc_ksyms_operations = {
	open:		ksyms_open,
	read:		seq_read,
	llseek:		seq_lseek,
	release:	seq_release,
};
#endif

extern struct seq_operations slabinfo_op;
extern ssize_t slabinfo_write(struct file *, const char *, size_t, loff_t *);
static int slabinfo_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &slabinfo_op);
}
static struct file_operations proc_slabinfo_operations = {
	open:		slabinfo_open,
	read:		seq_read,
	write:		slabinfo_write,
	llseek:		seq_lseek,
	release:	seq_release,
};

static int kstat_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int i, len = 0;
	extern unsigned long total_forks;
	unsigned long jif = jiffies;
	unsigned int sum = 0, user = 0, nice = 0, system = 0;
	int major, disk;

	for (i = 0 ; i < smp_num_cpus; i++) {
		int cpu = cpu_logical_map(i), j;

		user += kstat.per_cpu_user[cpu];
		nice += kstat.per_cpu_nice[cpu];
		system += kstat.per_cpu_system[cpu];
#if !defined(CONFIG_ARCH_S390)
		for (j = 0 ; j < NR_IRQS ; j++)
			sum += kstat.irqs[cpu][j];
#endif
	}

	proc_sprintf(page, &off, &len,
		      "cpu  %u %u %u %lu\n", user, nice, system,
		      jif * smp_num_cpus - (user + nice + system));
	for (i = 0 ; i < smp_num_cpus; i++)
		proc_sprintf(page, &off, &len,
			"cpu%d %u %u %u %lu\n",
			i,
			kstat.per_cpu_user[cpu_logical_map(i)],
			kstat.per_cpu_nice[cpu_logical_map(i)],
			kstat.per_cpu_system[cpu_logical_map(i)],
			jif - (  kstat.per_cpu_user[cpu_logical_map(i)] \
				   + kstat.per_cpu_nice[cpu_logical_map(i)] \
				   + kstat.per_cpu_system[cpu_logical_map(i)]));
	proc_sprintf(page, &off, &len,
		"page %u %u\n"
		"swap %u %u\n"
		"intr %u",
			kstat.pgpgin >> 1,
			kstat.pgpgout >> 1,
			kstat.pswpin,
			kstat.pswpout,
			sum
	);
#if !defined(CONFIG_ARCH_S390)
	for (i = 0 ; i < NR_IRQS ; i++)
		proc_sprintf(page, &off, &len,
			     " %u", kstat_irqs(i));
#endif

	proc_sprintf(page, &off, &len, "\ndisk_io: ");

	for (major = 0; major < DK_MAX_MAJOR; major++) {
		for (disk = 0; disk < DK_MAX_DISK; disk++) {
			int active = kstat.dk_drive[major][disk] +
				kstat.dk_drive_rblk[major][disk] +
				kstat.dk_drive_wblk[major][disk];
			if (active)
				proc_sprintf(page, &off, &len,
					"(%u,%u):(%u,%u,%u,%u,%u) ",
					major, disk,
					kstat.dk_drive[major][disk],
					kstat.dk_drive_rio[major][disk],
					kstat.dk_drive_rblk[major][disk],
					kstat.dk_drive_wio[major][disk],
					kstat.dk_drive_wblk[major][disk]
			);
		}
	}

	proc_sprintf(page, &off, &len,
		"\nctxt %lu\n"
		"btime %lu\n"
		"processes %lu\n",
		nr_context_switches(),
		xtime.tv_sec - jif / HZ,
		total_forks);

	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int devices_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_device_list(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

#if !defined(CONFIG_ARCH_S390)
static int interrupts_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_irq_list(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}
#endif

static int filesystems_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_filesystem_list(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

#ifdef CONFIG_GENERIC_ISA_DMA
static int dma_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_dma_list(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}
#endif

static int ioports_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_ioport_list(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int cmdline_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	extern char saved_command_line[];
	int len;

	len = snprintf(page, count, "%s\n", saved_command_line);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

#ifdef CONFIG_SGI_DS1286
static int ds1286_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_ds1286_status(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}
#endif

static int locks_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	lock_kernel();
	len = get_locks_status(page, start, off, count);
	unlock_kernel();
	if (len < count) *eof = 1;
	return len;
}

static int execdomains_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_exec_domain_list(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int swaps_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_swaparea_info(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

extern struct list_head active_list;
extern struct list_head inactive_list;

static int mmlist_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	struct list_head *active_head;
	struct list_head *inactive_head;
	struct list_head *active_next;
	struct list_head *inactive_next;
	
	active_head = &(active_list);
	active_next = active_list.next;

	inactive_head = &(inactive_list);
	inactive_next = inactive_list.next;

	printk(KERN_NOTICE "ACTIVE list(%d)\n",nr_active_pages);
	while (active_next != active_head)
	{
		struct page *this_page = list_entry(active_next,struct page,lru);
		struct dentry *this_dentry;
		struct inode *this_inode;
		struct list_head *tmp;
		struct list_head *tmp_head;

		if (!this_page->mapping)
			printk(KERN_NOTICE "PAGE(0x%x) is from malloc\n", (unsigned int)this_page);
		else
		{
			this_inode = this_page->mapping->host;
			tmp_head = &(this_inode->i_dentry);
			tmp = tmp_head->next;
			while(tmp != tmp_head)
			{
				this_dentry = list_entry(tmp,struct dentry,d_alias);
				printk(KERN_NOTICE "PAGE(0x%x)--(%s)\n",(unsigned int)this_page,this_dentry->d_name.name);
				tmp = tmp->next;
			}
		}
		active_next = active_next->next;
	}

	printk(KERN_NOTICE "INACTIVE list(%d)\n",nr_inactive_pages);
	while (inactive_next != inactive_head)
	{
		struct page *this_page = list_entry(inactive_next,struct page,lru);
		struct dentry *this_dentry;
		struct inode *this_inode;
		struct list_head *tmp;
		struct list_head *tmp_head;

		if (!this_page->mapping)
			printk(KERN_NOTICE "PAGE(0x%x) is from malloc\n", (unsigned int)this_page);
		else
		{
			this_inode = this_page->mapping->host;
			tmp_head = &(this_inode->i_dentry);
			tmp = tmp_head->next;
			while (tmp != tmp_head)
			{
				this_dentry = list_entry(tmp,struct dentry,d_alias);
				printk(KERN_NOTICE "PAGE(0x%x)--(%s)\n",(unsigned int)this_page,this_dentry->d_name.name);
				tmp = tmp->next;
			}
		}
		inactive_next = inactive_next->next;
	}
	return 0;
}
static int show_vma_pages(pmd_t *dir, unsigned long start, unsigned long end, unsigned long *total_pages, unsigned long *total_fpages, struct task_struct *ptask)
{
	pte_t *pte;
	struct page *page;
	unsigned long address = start;

	////printk(KERN_NOTICE "SHOW_VMA_PAGES(*PMD(0x%x))\n", pmd_val(*dir));////
	if (pmd_none(*dir))
		return 0;

	while (address < end)
	{
		pte = pte_offset(dir,address);
		////printk(KERN_NOTICE "PTE(0x%x)\n",pte_val(*pte)); ////
		if (pte_present(*pte))
		{
			*total_pages = (*total_pages) + 1;
			page = pte_page(*pte);
			////printk(KERN_NOTICE "PAGE(0x%x)\n", page);////
			if (page->mapping)
			{
				struct inode *inode;
				struct dentry *dentry;
				struct list_head *head;
				struct list_head *next;

				*total_fpages = (*total_fpages) + 1;
				inode = page->mapping->host;
				head = &(inode->i_dentry);
				next = head->next;
				while (next != head)
				{
					dentry = list_entry(next, struct dentry, d_alias);
					next = next->next;
					printk(KERN_NOTICE "(%s):PAGE(0x%x) of FILE(%s)\n", ptask->comm, (unsigned int)page, dentry->d_name.name);
				}
			}
			else
				printk(KERN_NOTICE "(%s):PAGE(0x%x)\n", ptask->comm, (unsigned int)page);
		}

		address += PAGE_SIZE;
	}
	return 0;
}
static int ppages_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	struct task_struct *ptask;
	struct vm_area_struct *pvma;
	pgd_t *pgd;
	pmd_t *pmd;
	unsigned long total_pages;
	unsigned long total_file_pages;

	unsigned long address,end,tmp_addr;
	unsigned long length;

	unsigned char str[6];

	printk(KERN_NOTICE "PROCESS MEMORY REGIONS (start)\n");

	for_each_task(ptask)
	{
		printk(KERN_NOTICE "TASK-TASK-TASK(%s--%d)\n",ptask->comm,ptask->pid);
		
		if (!ptask->mm)
		{
			printk(KERN_NOTICE "TASK-TASK-TASK(%s--%d) is a kernel thread\n", ptask->comm, ptask->pid);
			continue;
		}
		
		total_pages = 0;
		total_file_pages = 0;

		pvma = ptask->mm->mmap;
		if (!pvma)
		{
			printk(KERN_NOTICE "TASK-TASK-TASK(%s--%d), VMA is NULL\n", ptask->comm, ptask->pid);
			continue;
		}

		while (pvma)
		{
			int flags = pvma->vm_flags;

			str[0] = (flags & VM_READ)? 'r':'-';
			str[1] = (flags & VM_WRITE)? 'w':'-';
			str[2] = (flags & VM_EXEC)? 'x':'-';
			str[3] = (flags & VM_SHARED)? 's':'p';
			str[4] = (flags & VM_GROWSDOWN)? 'k':0;
			str[5] = 0;
			
			printk(KERN_NOTICE "(%s) VMA (0x%x -- 0x%x)(%s)\n",ptask->comm,(unsigned int)pvma->vm_start,(unsigned int)pvma->vm_end,str);

			/* Check if this vma belongs to pxafb framebuffer, if so, just go to next */
			if ((pvma->vm_file) && (MAJOR(pvma->vm_file->f_dentry->d_inode->i_rdev) == 29))
			{
				printk(KERN_NOTICE "(%s) This VMA is framebuffer mapping\n", ptask->comm);
				pvma = pvma->vm_next;
				continue;
			}

			address = pvma->vm_start;
			end = pvma->vm_end;
	
			pgd = pgd_offset(ptask->mm,address);
			pmd = (pmd_t *)pgd;
	
			do
			{
				tmp_addr = (address + PMD_SIZE) & PMD_MASK;
				if (tmp_addr < end)
					length = tmp_addr - address;
				else
					length = end - address;
				
				////printk(KERN_NOTICE "SHOW_VMA_PAGES(0x%x -- 0x%x)\n",address,address+length);////
				show_vma_pages(pmd, address, address + length, &total_pages,&total_file_pages, ptask);
				////printk(KERN_NOTICE "SHOW_VMA_PAGES(END 0x%x -- 0x%x)\n",address, address+length);////
				
				pmd ++;
				address += length;
			}while (address < end);
		
			//Susan for APLOGGER to print out
			schedule();

			pvma = pvma->vm_next;
		}

		printk(KERN_NOTICE "TASK-TASK-TASK(%s--%d),total_pages(%d),total_fpages(%d),s_code(0x%x),code(0x%x),s_data(0x%x),data(0x%x),s_brk(0x%x),brk(0x%x)\n",ptask->comm,(unsigned int)ptask->pid,(unsigned int)total_pages,(unsigned int)total_file_pages,(unsigned int)current->mm->start_code,(unsigned int)current->mm->end_code,(unsigned int)current->mm->start_data,(unsigned int)current->mm->end_data,(unsigned int)current->mm->start_brk,(unsigned int)current->mm->brk);

	}

	printk(KERN_NOTICE "PROCESS MEMORY REGIONS (end)\n");

	return 0;
}

static int nrlocals_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	struct task_struct *ptask;

	for_each_task(ptask)
	{
		printk(KERN_NOTICE "TASK-TASK-TASK(%s--%d), nr_local_pages(%d)\n",ptask->comm,ptask->pid,ptask->nr_local_pages);
	}

	return 0;
}


static int memory_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_mem_list(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}
//Susan for reporting potential free pages
extern int slab_cache_freeable_info(void);
static int slabfree_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int ret = 0;
	int len = 0;

	ret = slab_cache_freeable_info();
	if (ret == -EAGAIN)
		len = sprintf(page, "BUSY: %d\n", -1);
	else
		len = sprintf(page, "SLABFREE: %d\n", (unsigned int)(ret * PAGE_SIZE));

	return proc_calc_metrics(page, start, off, count, eof, len);
}

/* For pxafb LCD status checking */
static int pxafbs_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
#if (0)
	extern struct global_state pxafb_global_state;
	
	unsigned long pxafb_data[16];
	pxafb_db_proc(pxafb_data);
	
	printk(KERN_NOTICE "LCD active panel(%d)\n",pxafb_global_state.active_panel);
	printk(KERN_NOTICE "LCD main panel state(0x%x)\n",pxafb_global_state.main_state);
	printk(KERN_NOTICE "LCD cli panel state(0x%x)\n",pxafb_global_state.cli_state);
	printk(KERN_NOTICE "LCD backlight for main dutycycle (%d)\n", pxafb_global_state.bklight_main_dutycycle);
	printk(KERN_NOTICE "LCD backlight for cli dutycycle (%d)\n", pxafb_global_state.bklight_cli_dutycycle);
	printk(KERN_NOTICE "LCD main base frame is exported?(%x)\n",pxafb_data[0]);
	printk(KERN_NOTICE "pxafb_main: FIX.smem_len(%d)\n",pxafb_data[1]);
	printk(KERN_NOTICE "pxafb_main: VAR.yres_virtual(%d)\n",pxafb_data[1]);
	printk(KERN_NOTICE "pxafb_main: VAR.yoffset(%d)\n",pxafb_data[2]);
	printk(KERN_NOTICE "pxafb_main: FIRST_FB.dmsdesc_fb_dma(0x%x)\n",pxafb_data[3]);
	printk(KERN_NOTICE "pxafb_main: FIRST_FB.screen_dma(0x%x)\n",pxafb_data[1]);
	printk(KERN_NOTICE "pxafb_main: SECOND_FB.fmadesc_fb_dma(0x%x)\n",pxafb_data[1]);
	printk(KERN_NOTICE "pxafb_main: SECOND_FB.screen_dma(0x%x)\n",pxafb_data[2]);
#ifdef CONFIG_PXAFB_CLI
	printk(KERN_NOTICE "LCD CLI base frame is exported?(%x)\n",pxafb_data[0]);
	printk(KERN_NOTICE "pxafb_main: FIX.smem_len(%d)\n",pxafb_data[1]);
	printk(KERN_NOTICE "pxafb_main: VAR.yres_virtual(%d)\n",pxafb_data[1]);
	printk(KERN_NOTICE "pxafb_main: VAR.yoffset(%d)\n",pxafb_data[2]);
	printk(KERN_NOTICE "pxafb_main: FIRST_FB.dmsdesc_fb_dma(0x%x)\n",pxafb_data[3]);
	printk(KERN_NOTICE "pxafb_main: FIRST_FB.screen_dma(0x%x)\n",pxafb_data[1]);
	printk(KERN_NOTICE "pxafb_main: SECOND_FB.fmadesc_fb_dma(0x%x)\n",pxafb_data[1]);
	printk(KERN_NOTICE "pxafb_main: SECOND_FB.screen_dma(0x%x)\n",pxafb_data[2]);
#endif
#endif
	return 0;
}
/*
 * This function accesses profiling information. The returned data is
 * binary: the sampling step and the actual contents of the profile
 * buffer. Use of the program readprofile is recommended in order to
 * get meaningful info out of these data.
 */
static ssize_t read_profile(struct file *file, char *buf,
			    size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	ssize_t read;
	char * pnt;
	unsigned int sample_step = 1 << prof_shift;

	if (p >= (prof_len+1)*sizeof(unsigned int))
		return 0;
	if (count > (prof_len+1)*sizeof(unsigned int) - p)
		count = (prof_len+1)*sizeof(unsigned int) - p;
	read = 0;

	while (p < sizeof(unsigned int) && count > 0) {
		put_user(*((char *)(&sample_step)+p),buf);
		buf++; p++; count--; read++;
	}
	pnt = (char *)prof_buffer + p - sizeof(unsigned int);
	if (copy_to_user(buf,(void *)pnt,count))
		return -EFAULT;
	read += count;
	*ppos += read;
	return read;
}

/*
 * Writing to /proc/profile resets the counters
 *
 * Writing a 'profiling multiplier' value into it also re-sets the profiling
 * interrupt frequency, on architectures that support this.
 */
static ssize_t write_profile(struct file * file, const char * buf,
			     size_t count, loff_t *ppos)
{
#ifdef CONFIG_SMP
	extern int setup_profiling_timer (unsigned int multiplier);

	if (count==sizeof(int)) {
		unsigned int multiplier;

		if (copy_from_user(&multiplier, buf, sizeof(int)))
			return -EFAULT;

		if (setup_profiling_timer(multiplier))
			return -EINVAL;
	}
#endif

	memset(prof_buffer, 0, prof_len * sizeof(*prof_buffer));
	return count;
}

static struct file_operations proc_profile_operations = {
	read:		read_profile,
	write:		write_profile,
};

struct proc_dir_entry *proc_root_kcore;

static void create_seq_entry(char *name, mode_t mode, struct file_operations *f)
{
	struct proc_dir_entry *entry;
	entry = create_proc_entry(name, mode, NULL);
	if (entry)
		entry->proc_fops = f;
}

void __init proc_misc_init(void)
{
	struct proc_dir_entry *entry;
	static struct {
		char *name;
		int (*read_proc)(char*,char**,off_t,int,int*,void*);
	} *p, simple_ones[] = {
		{"loadavg",     loadavg_read_proc},
		{"uptime",	uptime_read_proc},
		{"meminfo",	meminfo_read_proc},
		{"version",	version_read_proc},
#ifdef CONFIG_PROC_HARDWARE
		{"hardware",	hardware_read_proc},
#endif
#ifdef CONFIG_STRAM_PROC
		{"stram",	stram_read_proc},
#endif
#ifdef CONFIG_MODULES
		{"modules",	modules_read_proc},
#endif
		{"stat",	kstat_read_proc},
		{"devices",	devices_read_proc},
#if !defined(CONFIG_ARCH_S390)
		{"interrupts",	interrupts_read_proc},
#endif
		{"filesystems",	filesystems_read_proc},
#ifdef CONFIG_GENERIC_ISA_DMA
		{"dma",		dma_read_proc},
#endif		
		{"ioports",	ioports_read_proc},
		{"cmdline",	cmdline_read_proc},
#ifdef CONFIG_SGI_DS1286
		{"rtc",		ds1286_read_proc},
#endif
		{"locks",	locks_read_proc},
		{"swaps",	swaps_read_proc},
		{"iomem",	memory_read_proc},
		{"execdomains",	execdomains_read_proc},
#if defined(CONFIG_MEMORY_ACCOUNTING) && !defined(CONFIG_DISCONTIGMEM)
		{"memmap",	memmap_read_proc},
#endif
		{"mmlist",  mmlist_read_proc},
		{"ppages",  ppages_read_proc},
		{"nrlocals", nrlocals_read_proc},
	//Susan for reporting potential free pages
		{"slabfree", slabfree_read_proc},
		{"pxafbs",	pxafbs_read_proc},

		{NULL,}
	};
	for (p = simple_ones; p->name; p++)
		create_proc_read_entry(p->name, 0, NULL, p->read_proc, NULL);

#if defined(CONFIG_MEMORY_ACCOUNTING) && !defined(CONFIG_DISCONTIGMEM)
	/*
	 * get the amount of free pages right here, as this
	 * should be the lowest amount of allocated pages
	 * the system ever sees.  After log in there
	 * should just be more allocated pages.
	 */
	lowwater = num_physpages - nr_free_pages();
#endif

	proc_symlink("mounts", NULL, "self/mounts");

	/* And now for trickier ones */
	entry = create_proc_entry("kmsg", S_IRUSR, &proc_root);
	if (entry)
		entry->proc_fops = &proc_kmsg_operations;
	create_seq_entry("cpuinfo", 0, &proc_cpuinfo_operations);
	create_seq_entry("partitions", 0, &proc_partitions_operations);
	create_seq_entry("slabinfo",S_IWUSR|S_IRUGO,&proc_slabinfo_operations);
#ifdef CONFIG_MODULES
	create_seq_entry("ksyms", 0, &proc_ksyms_operations);
#endif
	proc_root_kcore = create_proc_entry("kcore", S_IRUSR, NULL);
	if (proc_root_kcore) {
		proc_root_kcore->proc_fops = &proc_kcore_operations;
		proc_root_kcore->size =
				(size_t)high_memory - PAGE_OFFSET + PAGE_SIZE;
	}
	if (prof_shift) {
		entry = create_proc_entry("profile", S_IWUSR | S_IRUGO, NULL);
		if (entry) {
			entry->proc_fops = &proc_profile_operations;
			entry->size = (1+prof_len) * sizeof(unsigned int);
		}
	}
#ifdef CONFIG_PPC32
	{
		extern struct file_operations ppc_htab_operations;
		entry = create_proc_entry("ppc_htab", S_IRUGO|S_IWUSR, NULL);
		if (entry)
			entry->proc_fops = &ppc_htab_operations;
	}
#endif
}
