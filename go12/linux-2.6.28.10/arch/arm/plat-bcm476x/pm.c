/*****************************************************************************
* Copyright 2008 - 2009 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/
/*
 * linux/arch/arm/mach-bcm47xx/pm.c
 *
 * Power Management Routines
 *
 *****************************************************************************/

#include <linux/version.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/atomic.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>
#include <asm/mach-types.h>
#include <asm/arch/reg_pwrseq.h>

static int bcm47xx_suspend_valid(suspend_state_t state);
static int bcm47xx_suspend_prepare(void);
static int bcm47xx_suspend_enter(suspend_state_t state);
static void bcm47xx_suspend_finish(void);
static void bcm47xx_suspend_recover(void);

static int bcm47xx_suspend_valid(suspend_state_t state)
{
	return bcm4760_pm_suspend_valid(state);
}

static int bcm47xx_suspend_prepare(void)
{
	return bcm4760_pm_suspend_prepare();
}

static int bcm47xx_suspend_enter(suspend_state_t state)
{
    int ret = 0;		// zero means no good on suspend state entry

	switch (state)
	{
	case PM_SUSPEND_STANDBY:
        {
            if(bcm4760_pm_suspend_standby())
                printk(KERN_INFO "bcm4760_pm_suspend_standby() failed \n") ;
			else
				ret=1;
		    break ;
        }
	case PM_SUSPEND_MEM:
            if(bcm4760_pm_suspend_mem())
                printk(KERN_INFO "bcm4760_pm_suspend_mem() failed \n") ;
            else
            	ret=1;
		break;
	}

	return ret;
}

static void bcm47xx_suspend_finish(void)
{
	bcm4760_pm_suspend_finish();

	return;
}
static void bcm47xx_suspend_recover(void)
{
	return;
}

static struct platform_suspend_ops bcm4760_suspend_ops ={
	.valid		= bcm47xx_suspend_valid,
	.prepare	= bcm47xx_suspend_prepare,
	.enter		= bcm47xx_suspend_enter,
	.finish		= bcm47xx_suspend_finish,
	.recover	= bcm47xx_suspend_recover
};

int __init bcm47xx_pm_init(void)
{
	suspend_set_ops(&bcm4760_suspend_ops);
	return 0;
}
__initcall(bcm47xx_pm_init);
