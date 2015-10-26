/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
/*  derived from linux/arch/arm/mach-versatile/core.c
 *  linux/arch/arm/mach-bcm476x/core.c
 *
 *  Copyright (C) 1999 - 2003 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <asm/system.h>
#include <asm/mach/arch.h>

#include <plat/core.h>

AMBA_DEVICE(uartA, "dev:f0",  URT0, NULL);
AMBA_DEVICE(uartB, "dev:f1",  URT1, NULL);
AMBA_DEVICE(uartC, "dev:f2",  URT2, NULL);
AMBA_DEVICE(uartD, "dev:f3",  URTG, NULL);

static struct amba_device *amba_devs[] __initdata = {
	&uartA_device,
	&uartB_device,
	&uartC_device,
	&uartD_device,
};

void __init bcm476x_amba_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(amba_devs); i++) {
		struct amba_device *d = amba_devs[i];
		amba_device_register(d, &iomem_resource);
	}
}

