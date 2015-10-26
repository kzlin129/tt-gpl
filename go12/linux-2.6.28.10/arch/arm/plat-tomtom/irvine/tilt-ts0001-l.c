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
#include <linux/delay.h>

#include <plat/tt_setup_handler.h>
#include <plat/irvine.h>
#include <plat/tilt_sensor.h>

#include <mach/pinmux.h>

static int tilt_power = 0;

struct resource tilt_resources[] = {
	{
	 .name = "tilt_sensor",
	 .flags = IORESOURCE_IRQ,
	 },
};

static int tilt_power_get(void)
{
	return tilt_power;
}

static int tilt_power_set(int power_on)
{
	if (power_on) {
		gpio_direction_output(TT_VGPIO_TILT_PWR, 1);
		tilt_power = 1;

		/* don't have the documentation, so let's hope that 20ms is enough */
		msleep(20);
	} else {
		gpio_direction_output(TT_VGPIO_TILT_PWR, 0);
		tilt_power = 0;
	}
	return 0;
}

static int tilt_value(void)
{
	return gpio_get_value(TT_VGPIO_TILT_OUT);
}

static unsigned int tilt_raw(void)
{
	unsigned int value = 0;

	value |= (gpio_get_value(TT_VGPIO_TILT_OUT)<<0);

	return value;
}

static int config_gpio(void)
{
	/* config TILT_OUT pin as input */
	bcm4760_set_pin_mux(vgpio_to_gpio(TT_VGPIO_TILT_OUT),
			    BCM4760_PIN_MUX_GPIO);
	gpio_request(TT_VGPIO_TILT_OUT, "TT_VGPIO_TILT_OUT");
	gpio_direction_input(TT_VGPIO_TILT_OUT);

	tilt_resources[0].start = gpio_to_irq(TT_VGPIO_TILT_OUT);
	tilt_resources[0].end = tilt_resources[0].start;

	/* config TILT_PWR pin as output */
	bcm4760_set_pin_mux(vgpio_to_gpio(TT_VGPIO_TILT_PWR),
			    BCM4760_PIN_MUX_GPIO);
	gpio_request(TT_VGPIO_TILT_PWR, "TT_VGPIO_TILT_PWR");

	/* power off tilt first*/
	tilt_power_set(0);

	return 0;
}

static struct tilt_sensor_platform_data tilt_sensor_pdata = {
	.tilt_power_set = tilt_power_set,
	.tilt_power_get = tilt_power_get,
	.tilt_value = tilt_value,
	.tilt_raw = tilt_raw,
	.config_gpio = config_gpio,
};

static struct platform_device tilt_sensor_pdev = {
	.name = "tilt_sensor",
	.id = -1,
	.num_resources = ARRAY_SIZE(tilt_resources),
	.resource = tilt_resources,
	.dev = {
		.platform_data = (void *)&tilt_sensor_pdata,
		},
};

static int __init tt_tilt_setup(char *identifier)
{
	printk(KERN_INFO "Initializing Tilt sensor\n");

	BUG_ON(TT_VGPIO_TILT_PWR == 0);
	BUG_ON(TT_VGPIO_TILT_OUT == 0);

	return platform_device_register(&tilt_sensor_pdev);
};

TT_SETUP_CB(tt_tilt_setup, "tomtom-bcm-tilt-ts0001-l");
