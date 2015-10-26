/*
 *  Copyright (C) 2008-2011 by TomTom International BV. All rights reserved.
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

struct resource ts1003_resources[] = {
	{
	 .name = "tilt_sensor",
	 .flags = IORESOURCE_IRQ,
	 },
	{
	 .name = "tilt_out2",
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
    int E1, E2;
    E1 = gpio_get_value(TT_VGPIO_TILT_OUT);
    E2 = gpio_get_value(TT_VGPIO_TILT_OUT2);
	//printk("ts1003-al: E1=%d, E2=%d \n", E1, E2);

	if(E1&&E2)
        return 0;
    else
        return 1;
}

static unsigned int tilt_raw(void)
{
	unsigned int value = 0;
	
	value |= (gpio_get_value(TT_VGPIO_TILT_OUT)<<0);
	value |= (gpio_get_value(TT_VGPIO_TILT_OUT2)<<1);	
		
	return value;
}

static int config_gpio(void)
{
	/* config TILT_OUT pin as input */
	bcm4760_set_pin_mux(vgpio_to_gpio(TT_VGPIO_TILT_OUT),
			    BCM4760_PIN_MUX_GPIO);
	gpio_request(TT_VGPIO_TILT_OUT, "TT_VGPIO_TILT_OUT");
	gpio_direction_input(TT_VGPIO_TILT_OUT);

	ts1003_resources[0].start = gpio_to_irq(TT_VGPIO_TILT_OUT);
	ts1003_resources[0].end = ts1003_resources[0].start;

	/* config TILT_OUT2 pin as input */
	bcm4760_set_pin_mux(vgpio_to_gpio(TT_VGPIO_TILT_OUT2),
			    BCM4760_PIN_MUX_GPIO);
	gpio_request(TT_VGPIO_TILT_OUT2, "TT_VGPIO_TILT_OUT2");
	gpio_direction_input(TT_VGPIO_TILT_OUT2);

	ts1003_resources[1].start = gpio_to_irq(TT_VGPIO_TILT_OUT2);
	ts1003_resources[1].end = ts1003_resources[1].start;
	
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
	.num_resources = ARRAY_SIZE(ts1003_resources),
	.resource = ts1003_resources,
	.dev = {
		.platform_data = (void *)&tilt_sensor_pdata,
		},
};

static int __init tt_tilt_setup(char *identifier)
{
	printk(KERN_INFO "Initializing Tilt sensor (ts1003-al) \n");

	BUG_ON(TT_VGPIO_TILT_PWR == 0);
	BUG_ON(TT_VGPIO_TILT_OUT == 0);
    BUG_ON(TT_VGPIO_TILT_OUT2 == 0);

	return platform_device_register(&tilt_sensor_pdev);
};

TT_SETUP_CB(tt_tilt_setup, "tomtom-bcm-tilt-ts1003-al");
