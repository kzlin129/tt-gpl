/*
 * Copyright (C) 2008 TomTom BV <http://www.tomtom.com/>
 * Author: Benoit Leffray <benoit.leffray@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/
 
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <asm/delay.h>
#include <plat/map.h>
#include <plat/mendoza.h>
#include <plat/gpio-cfg.h>
#include <plat/flipflop.h>


#define PFX		"tomtom_flipflop: "	

static void flipflop_dev_release(struct device *dev)
{
}

static int init_flipflop(struct platform_device *pdev)
{
#ifdef CONFIG_TOMTOM_FLIPFLOP_HW
	int err=0;

	if (gpio_is_valid(TT_VGPIO_RFS_BOOT_CLK)) {
		err = gpio_request(TT_VGPIO_RFS_BOOT_CLK, "RFS_BOOT_CLK");

		if (err) {
			printk(KERN_ERR "error requesting RFS_BOOT_CLK %d\n",TT_VGPIO_RFS_BOOT_CLK);
			return err;
		}
	}
	if (gpio_is_valid(TT_VGPIO_RFS_BOOT_Q)) {
		err = gpio_request(TT_VGPIO_RFS_BOOT_Q, "RFS_BOOT_Q");

		if (err) {
			printk(KERN_ERR "error requesting RFS_BOOT_Q %d\n",TT_VGPIO_RFS_BOOT_Q);
			return err;
		}
	}
	gpio_direction_input(TT_VGPIO_RFS_BOOT_Q);
	gpio_direction_output(TT_VGPIO_RFS_BOOT_CLK, 0);
	s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_RFS_BOOT_Q), 	S3C_GPIO_PULL_DOWN);
	return 0;
#else
	return 0;
#endif
}

static void set_flipflop(int v)
{
#ifdef CONFIG_TOMTOM_FLIPFLOP_HW
	int v2;

	v = !!v;
	v2 = !!gpio_get_value(TT_VGPIO_RFS_BOOT_Q);
	if (v2 != v) {
		gpio_set_value(TT_VGPIO_RFS_BOOT_CLK, 1);
		udelay(10);
		gpio_set_value(TT_VGPIO_RFS_BOOT_CLK, 0);
	}
	v2 = !!gpio_get_value(TT_VGPIO_RFS_BOOT_Q);
#else
	volatile unsigned long *failflg =
		(unsigned long *)phys_to_virt(0x52000000);
	*failflg = v ? 0xdeadbeef : 0x0;
#endif
}

static int get_flipflop(void)
{
#ifdef CONFIG_TOMTOM_FLIPFLOP_HW
	int v = gpio_get_value(TT_VGPIO_RFS_BOOT_Q);
	return !!v;
#else
	volatile unsigned long *failflg =
		(unsigned long *)phys_to_virt(0x52000000);
	return *failflg;
#endif
}

static struct flipflop_info flipflop_info = {
	.init = init_flipflop,
	.set_value = set_flipflop,
	.get_value = get_flipflop
};

static struct platform_device flipflop_pdev =
{
	.name   = "flipflop",
	.id		= -1,
	.dev = {
		.release = flipflop_dev_release,
		.platform_data = &flipflop_info,
	},
};

static int __init flipflop_init(void)
{
	platform_device_register(&flipflop_pdev);

	printk(KERN_INFO PFX " Registered\n");
	return 0;
};

arch_initcall(flipflop_init);

