#ifndef __ASM_SH_BUGS_H
#define __ASM_SH_BUGS_H

/*
 * This is included by init/main.c to check for architecture-dependent bugs.
 *
 * Needs:
 *	void check_bugs(void);
 */

/*
 * I don't know of any Super-H bugs yet.
 */

#include <asm/processor.h>
#include <asm/pci-sh7751.h>

static void __init check_bugs(void)
{
	extern unsigned long loops_per_jiffy;
	char *p= &system_utsname.machine[2]; /* "sh" */
	unsigned int id_51, id_51r;

	id_51 = (SH7751_DEVICE_ID << 16) | SH7751_VENDOR_ID;
	id_51r = (SH7751R_DEVICE_ID << 16) | SH7751_VENDOR_ID;

	cpu_data->loops_per_jiffy = loops_per_jiffy;
	
	switch (cpu_data->type) {
	case CPU_SH7708:
		*p++ = '3';
		printk("CPU: SH7707/SH7708/SH7709\n");
		break;
	case CPU_SH7729:
		*p++ = '3';
		printk("CPU: SH7709A/SH7729/SH7709S/SH7727\n");
		break;
	case CPU_SH7750:
		*p++ = '4';
		printk("CPU: SH7750/SH7751\n");
		break;
        case CPU_SH7750S:
               *p++ = '4';
               printk("CPU: SH7750S\n");
               break;
        case CPU_SH7750R:
               *p++ = '4';
               printk("CPU: SH7750R\n");
               break;
        case CPU_SH7751:
               *p++ = '4';
               printk("CPU: SH7751\n");
               break;
        case CPU_SH7751R:
               *p++ = '4';
               printk("CPU: SH7751R\n");
               break;
	case CPU_ST40STB1:
		*p++ = '4';
		printk("CPU: ST40STB1\n");
		break;
	case CPU_SH73180:
		*p++ = '4';
		printk("CPU: SH73180\n");
		break;
	default:
		printk("CPU: ??????\n");
		break;
	}

#ifndef __LITTLE_ENDIAN__
	/* 'eb' means 'Endian Big' */
	*p++ = 'e';
	*p++ = 'b';
#endif
	*p = '\0';
}
#endif /* __ASM_SH_BUGS_H */
