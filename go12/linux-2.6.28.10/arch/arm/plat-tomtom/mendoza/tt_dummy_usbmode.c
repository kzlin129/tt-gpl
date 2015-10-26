/*
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Benoit Leffray <benoit.leffray@tomtom.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <asm/mach/arch.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/platform_device.h>

struct platform_device dummy_usbmode = {
	.name	= "usbmode-impl-dummy",
	.id	= -1,
};

static int __init dummy_usbmode_init(void)
{
	platform_device_register(&dummy_usbmode);
	return 0;
}
arch_initcall(dummy_usbmode_init);
