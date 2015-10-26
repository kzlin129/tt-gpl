/* linux/arch/arm/plat-s5pc1xx/setup-fb.c
 *
 * Copyright 2009 Samsung Electronics
 *	Jinsung Yang <jsgood.yang@samsung.com>
 *	http://samsungsemi.com/
 *
 * Base S5PC1XX FIMD gpio configuration
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <asm/io.h>
#include <plat/gpio-cfg.h>
#include <plat/map.h>
#include <plat/regs-clock.h>

struct platform_device; /* don't need the contents */

#ifdef CONFIG_CPU_S5PC100
void s3cfb_cfg_gpio(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < 8; i++)
		s3c_gpio_cfgpin(S5PC1XX_GPF0(i), S3C_GPIO_SFN(2));

	for (i = 0; i < 8; i++)
		s3c_gpio_cfgpin(S5PC1XX_GPF1(i), S3C_GPIO_SFN(2));

	for (i = 0; i < 8; i++)
		s3c_gpio_cfgpin(S5PC1XX_GPF2(i), S3C_GPIO_SFN(2));

	for (i = 0; i < 4; i++)
		s3c_gpio_cfgpin(S5PC1XX_GPF3(i), S3C_GPIO_SFN(2));
}

int s3cfb_backlight_on(struct platform_device *pdev)
{
	int err;

	err = gpio_request(S5PC1XX_GPD(0), "GPD");

	if (err) {
		printk(KERN_ERR "failed to request GPD for "
			"lcd backlight control\n");
		return err;
	}

	gpio_direction_output(S5PC1XX_GPD(0), 1);
	gpio_free(S5PC1XX_GPD(0));

	return 0;
}
#else
void s3cfb_cfg_gpio(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < 8; i++)
		s3c_gpio_cfgpin(S5PC1XX_GPF0(i), S3C_GPIO_SFN(2));

	for (i = 0; i < 8; i++)
		s3c_gpio_cfgpin(S5PC1XX_GPF1(i), S3C_GPIO_SFN(2));

	for (i = 0; i < 8; i++)
		s3c_gpio_cfgpin(S5PC1XX_GPF2(i), S3C_GPIO_SFN(2));

	for (i = 0; i < 4; i++)
		s3c_gpio_cfgpin(S5PC1XX_GPF3(i), S3C_GPIO_SFN(2));

	/* mDNIe SEL: why we shall write 0x2 ? */
	writel(0x2, S5P_MDNIE_SEL);
}

int s3cfb_backlight_on(struct platform_device *pdev)
{
	int err;

	err = gpio_request(S5PC1XX_GPD0(3), "GPD0");

	if (err) {
		printk(KERN_ERR "failed to request GPD0 for "
			"lcd backlight control\n");
		return err;
	}

	gpio_direction_output(S5PC1XX_GPD0(3), 1);
	gpio_free(S5PC1XX_GPD0(3));

	return 0;
}
#endif

int s3cfb_reset_lcd(struct platform_device *pdev)
{
	int err;

	err = gpio_request(S5PC1XX_GPH0(6), "GPH0");

	if (err) {
		printk(KERN_ERR "failed to request GPH0 for "
			"lcd reset control\n");
		return err;
	}

	gpio_direction_output(S5PC1XX_GPH0(6), 1);

	mdelay(100);

	gpio_set_value(S5PC1XX_GPH0(6), 0);
	mdelay(10);

	gpio_set_value(S5PC1XX_GPH0(6), 1);
	mdelay(10);

	gpio_free(S5PC1XX_GPH0(6));

	return 0;
}

