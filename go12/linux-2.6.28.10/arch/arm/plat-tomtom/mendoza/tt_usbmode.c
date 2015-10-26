/*
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Benoit Leffray <benoit.leffray@tomtom.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/kernel.h>
#include <linux/types.h>

#include <linux/init.h>
#include <linux/platform_device.h>

struct platform_device  usbmode_pdev =
{
    .name   =   "tomtom-usbmode",
	.id	= 1,
};

int __init usbmode_init(void)
{
	platform_device_register(&usbmode_pdev);
	return 0;
}
arch_initcall(usbmode_init);
