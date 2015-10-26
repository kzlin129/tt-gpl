/*
 *  Copyright (C) 2008 by TomTom International BV. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation.
 *
 *
 *
 */

#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/vgpio.h>
#include <linux/gpio.h>

#include <plat/tt_setup_handler.h>
#include <plat/irvine.h>
#include <plat/low-dc-vcc.h>

#include <mach/pinmux.h>


void detect_low_dc_mach_release (struct detect_low_dc_platform_data* pdata)
{
	gpio_free(TT_VGPIO_LOW_DC_VCC);
}

int detect_low_dc_config_gpio (void)
{
	int err=0;

	if (gpio_is_valid(TT_VGPIO_LOW_DC_VCC)) {
		if ((err = gpio_request(TT_VGPIO_LOW_DC_VCC, "TT_VGPIO_LOW_DC_VCC"))) {
			printk("%s: Can't request TT_VGPIO_LOW_DC_VCC\n", __func__);
			
		} else {
			err = gpio_direction_input(TT_VGPIO_LOW_DC_VCC);
		}
	}

	return err;
}

int detect_low_dc_get_value (void)
{
	int retval;
	
	retval = gpio_get_value(TT_VGPIO_LOW_DC_VCC);

	return retval;
}

void detect_low_dc_suspend (void)
{

}

void detect_low_dc_resume (void)
{

}


struct resource detect_low_dc_resources[] = 
{
	{
		.name = "low_dc_vcc",
		.flags = IORESOURCE_IRQ,
	},
};

static struct detect_low_dc_platform_data detect_low_dc_pdata = 
{
	.mach_release	= detect_low_dc_mach_release,

	.config_gpio	= detect_low_dc_config_gpio,
	.get_gpio_value	= detect_low_dc_get_value,

	.suspend	= detect_low_dc_suspend,
	.resume		= detect_low_dc_resume,

};

static struct platform_device detect_low_dc_pdev = 
{
	.name 		= "low_dc_vcc",
	.id 		= -1,
	.num_resources 	= ARRAY_SIZE(detect_low_dc_resources),
	.resource 	= detect_low_dc_resources,
	.dev = 
	{
		.platform_data = (void *) &detect_low_dc_pdata,
	},
};

static int __init irvine_low_dc_vcc (void)
{
	printk (KERN_INFO"Initializing LOW_DC_VCC\n");

	BUG_ON(TT_VGPIO_LOW_DC_VCC == 0);
	detect_low_dc_resources[0].start = gpio_to_irq(TT_VGPIO_LOW_DC_VCC);
	detect_low_dc_resources[0].end   = detect_low_dc_resources[0].start;

	bcm4760_set_pin_mux (vgpio_to_gpio (TT_VGPIO_LOW_DC_VCC), BCM4760_PIN_MUX_GPIO);

	return platform_device_register(&detect_low_dc_pdev);
};

arch_initcall (irvine_low_dc_vcc);
