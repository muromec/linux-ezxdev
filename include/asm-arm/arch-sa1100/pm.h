/*
 *
 * Declarations for ARM Linux Power Management
 *
 * Copyright 2002 Compaq Computer Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Author: Jamey Hicks.
 *
 */


extern int (*pm_suggest_suspend_hook)(int state);
extern int (*pm_sysctl_suspend_hook)(int state);
extern int pm_use_sbin_pm_helper; 
extern int pm_suspend(void);
extern int pm_suggest_suspend(void); /* triggers /sbin/pm_helper or queueing event to apmd */
