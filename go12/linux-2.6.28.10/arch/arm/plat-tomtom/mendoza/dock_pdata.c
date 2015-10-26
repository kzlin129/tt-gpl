/*
 *  Copyright (C) 2008 by TomTom International BV. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation.
 *
 *
 * ** See arch/arm/plat-tomtom/mendoza/include/plat/dock_pdata.h for documentation **
 *
 */

#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <plat/dock.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <plat/mendoza.h>
#include <plat/gpio-cfg.h>

/* Separate resources in case of non-consecutive IRQ lines */

static int mendoza_dock_machine_init(void)
{
	int err = 0;

	if (gpio_is_valid(TT_VGPIO_DOCK_RESET)) {
		if ((err = gpio_request(TT_VGPIO_DOCK_RESET, "TT_VGPIO_DOCK_RESET"))) {
			printk("%s: Can't request TT_VGPIO_DOCK_RESET\n", __func__);
			goto err_free1;
		}
	}

	if (gpio_is_valid(TT_VGPIO_ACCESSORY_PWR_EN)) {
		if ((err = gpio_request(TT_VGPIO_ACCESSORY_PWR_EN, 
				"TT_VGPIO_ACCESSORY_PWR_EN"))) {
			printk("%s: Can't request TT_VGPIO_ACCESSORY_PWR_EN\n", __func__);
			goto err_free2;
		}
	}

	if (gpio_is_valid(TT_VGPIO_DOCK_DET_PWR)) {
		if ((err = gpio_request(TT_VGPIO_DOCK_DET_PWR, "TT_VGPIO_DOCK_DET_PWR"))) {
			printk("%s: Can't request TT_VGPIO_DOCK_DET_PWR\n", __func__);
			goto err_free3;
		}
	}

	if (gpio_is_valid(TT_VGPIO_DOCK_DET0)) {
		if ((err = gpio_request(TT_VGPIO_DOCK_DET0, "TT_VGPIO_DOCK_DET0"))) {
			printk("%s: Can't request TT_VGPIO_DOCK_DET0\n", __func__);
			goto err_free4;
		}
	}

	if (gpio_is_valid(TT_VGPIO_DOCK_DET1)) {
		if ((err = gpio_request(TT_VGPIO_DOCK_DET1, "TT_VGPIO_DOCK_DET1"))) {
			printk("%s: Can't request TT_VGPIO_DOCK_DET1\n", __func__);
			goto err_free5;
		}
	}
	return 0;

err_free5:
	gpio_free(TT_VGPIO_DOCK_DET0);
err_free4:
	gpio_free(TT_VGPIO_DOCK_DET_PWR);
err_free3:
	gpio_free(TT_VGPIO_ACCESSORY_PWR_EN);
err_free2:
	gpio_free(TT_VGPIO_DOCK_RESET);
err_free1:
	return err;
}

static void mendoza_dock_machine_release(void)
{
	gpio_free(TT_VGPIO_DOCK_DET1);
	gpio_free(TT_VGPIO_DOCK_DET0);
	gpio_free(TT_VGPIO_DOCK_DET_PWR);
	gpio_free(TT_VGPIO_ACCESSORY_PWR_EN);
	gpio_free(TT_VGPIO_DOCK_RESET);
}

static int mendoza_dock_line_config(struct dock_platform_data *pd, dock_connector_line_t line, int default_value, dock_line_flag_t flags)
{
	switch(line) {
	case RDS_nRST:
		BUG_ON(flags & (DOCK_LINE_INPUT | DOCK_LINE_INTERRUPT));
		s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_DOCK_RESET), S3C_GPIO_PULL_NONE);
		gpio_direction_output(TT_VGPIO_DOCK_RESET, default_value);
		break;
	case DOCK_DET0:
#if defined (CONFIG_MACH_SEOUL)
		s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_DOCK_DET0), S3C_GPIO_PULL_UP);
#else
		s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_DOCK_DET0), S3C_GPIO_PULL_NONE);
#endif
		gpio_direction_input(TT_VGPIO_DOCK_DET0);
		break;
	case DOCK_DET1:
#if defined (CONFIG_MACH_SEOUL)
		s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_DOCK_DET1), S3C_GPIO_PULL_UP);
#else
		s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_DOCK_DET1), S3C_GPIO_PULL_NONE);
#endif
		gpio_direction_input(TT_VGPIO_DOCK_DET1);
		break;
	case DC_OUT:
		s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_ACCESSORY_PWR_EN), S3C_GPIO_PULL_NONE);
		gpio_direction_output(TT_VGPIO_ACCESSORY_PWR_EN, default_value);
		break;
	case DOCK_I2C_PWR:
		s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_DOCK_I2C_EN), S3C_GPIO_PULL_NONE);
		gpio_direction_output(TT_VGPIO_DOCK_I2C_EN, default_value);
		break;
	case DOCK_PWR_DET:
		s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_DOCK_DET_PWR), S3C_GPIO_PULL_NONE);
		gpio_direction_output(TT_VGPIO_DOCK_DET_PWR, default_value);
		break;
	default:
		break;
	}
}

static int mendoza_dock_line_get(struct dock_platform_data *pd, dock_connector_line_t line)
{
 int			ret = 0;

	switch(line) {
	case DOCK_DET0:
		ret = gpio_get_value(TT_VGPIO_DOCK_DET0);
		break;
	case DOCK_DET1:
		ret = gpio_get_value(TT_VGPIO_DOCK_DET1);
		break;
	case DOCK_PWR_DET:
		ret	= gpio_get_value(TT_VGPIO_DOCK_DET_PWR);
		break;
	case DOCK_I2C_PWR:
		ret = gpio_get_value(TT_VGPIO_DOCK_I2C_EN);
		break;
	default:
		/* The driver logic asked for something that is not supported. WARN, because you don't want */
		/* stall the boot process for something as menial as the dock.                              */
		WARN_ON(1);
		break;
	}

	return ret;	
}

static int mendoza_dock_line_set(struct dock_platform_data *pd, dock_connector_line_t line, int activate)
{
 int			ret = 0;

	switch(line) {
	case RDS_nRST:
		gpio_set_value(TT_VGPIO_DOCK_RESET, activate);
		break;
	case DC_OUT:
		gpio_set_value(TT_VGPIO_ACCESSORY_PWR_EN, activate);
		break;
	case DOCK_PWR_DET:
		gpio_set_value(TT_VGPIO_DOCK_DET_PWR, activate);
		break;
	case DOCK_I2C_PWR:
		gpio_set_value(TT_VGPIO_DOCK_I2C_EN, activate);
		break;
	default:
		printk(KERN_ERR "Pin number %d has no set implementation\n", line);
		WARN_ON(1);
		break;
	}

	return ret;
}


static int mendoza_dock_line_get_irq(struct dock_platform_data *pd, dock_connector_line_t line)
{
 int ret = -1;

	switch(line) {
	case DOCK_DET0:
		ret	= gpio_to_irq(TT_VGPIO_DOCK_DET0);
		break;
	case DOCK_DET1:
		ret	= gpio_to_irq(TT_VGPIO_DOCK_DET1);
		break;
	case FM_INT:
		ret	= gpio_to_irq(TT_VGPIO_DOCK_FM_INT);
		break;
	default:
		break;
	}

	return ret;
}

static inline void dock_suspend (void)
{
	printk(KERN_INFO "TT_Dock: Suspend\n");

#ifdef CONFIG_MACH_SEOUL 
	gpio_direction_output(S3C_GPN11, 1);/* TT_DOCK_DETPWR */
	gpio_direction_input_pd(S3C_GPL14);	/* TT_DOCK_SPARE */
	gpio_direction_input_pd(S3C_GPM0);	/* TT_nHPDETECT */
	gpio_direction_output(S3C_GPB4, 0);	/* TT_RDS_RST */
	gpio_direction_output(S3C_GPN9, 0);	/* TT_DOCK_PWREN */
	gpio_direction_input_pd(S3C_GPN8);	/* TT_RDS_IRQ */
#endif
}

static inline void dock_resume (void)
{
	printk(KERN_INFO "TT_Dock: Resume\n");
}

struct resource dock_resources[] = {
	{
		.name = "dock_sense0",
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "dock_sense1",
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "dock_i2c",
		.flags = IORESOURCE_IRQ,
	},
};

static struct dock_platform_data dock_pdata = {
	.i2cbusnumber		= 0,
	.dock_machine_init	= mendoza_dock_machine_init,
	.dock_machine_release	= mendoza_dock_machine_release,
	.dock_line_config	= mendoza_dock_line_config,
	.dock_line_set		= mendoza_dock_line_set,
	.dock_line_get		= mendoza_dock_line_get,
	.dock_line_get_irq	= mendoza_dock_line_get_irq,
	.suspend		= dock_suspend,
	.resume			= dock_resume,
};

static struct platform_device dock_pdev = {
	.name = "dock",
	.id = -1,
	.num_resources = ARRAY_SIZE(dock_resources),
	.resource = dock_resources,
	.dev = {
		.platform_data = (void *) &dock_pdata,
	},
};

static int __init setup_dock(char *identifier)
{
	BUG_ON(TT_VGPIO_DOCK_DET0 == 0);

	dock_resources[DOCK_IRQ_SENSE0].start =
		gpio_to_irq(TT_VGPIO_DOCK_DET0);
	dock_resources[DOCK_IRQ_SENSE0].end =
		dock_resources[DOCK_IRQ_SENSE0].start;

	BUG_ON(TT_VGPIO_DOCK_DET1 == 0);
	dock_resources[DOCK_IRQ_SENSE1].start =
		gpio_to_irq(TT_VGPIO_DOCK_DET1);
	dock_resources[DOCK_IRQ_SENSE1].end =
		dock_resources[DOCK_IRQ_SENSE1].start;

	return platform_device_register(&dock_pdev);
};

arch_initcall(setup_dock);
