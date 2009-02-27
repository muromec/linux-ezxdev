/*
 * firmware.c - firmware subsystem hoohaw.
 */

#if 1 /* linux-pm */
#include <linux/device.h>
#endif /* linux-pm */
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/init.h>

static struct subsystem firmware_subsys = {
	.kobj	= { .name = "firmware" },
};

int firmware_register(struct subsystem * s)
{
	s->parent = &firmware_subsys;
	return subsystem_register(s);
}

void firmware_unregister(struct subsystem * s)
{
	subsystem_unregister(s);
}

#if 1 /* linux-pm */
int __init firmware_init(void)
#else
static int __init firmware_init(void)
#endif
{
	return subsystem_register(&firmware_subsys);
}

#if 0 /* linux-pm */
core_initcall(firmware_init);
#endif

EXPORT_SYMBOL(firmware_register);
EXPORT_SYMBOL(firmware_unregister);
