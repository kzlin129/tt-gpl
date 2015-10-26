/* 
 * Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 * Author: Rogier Stam <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/powerbutton.h>
#include <linux/gpio.h>
#include <linux/vgpio.h>

#include <plat/mendoza.h>
#include <plat/gpio-cfg.h>

#define DEBOUNCE_POLL (HZ/5)	/* Very 1/5th of a second */

static struct timer_list debounce_poll;
static atomic_t btnstate = ATOMIC_INIT(1);

static int mendoza_powerbutton_workaround_getvalue( void )
{ 
        unsigned long int flags;
	int value;

        local_irq_save( flags );
	value = gpio_get_value(TT_VGPIO_PWR_BUTTON);
        local_irq_restore( flags );

	return !value;				/* Normaly active low */
}

static int mendoza_powerbutton_get_timed_value ( void )
{
	unsigned int tmp;

	tmp = atomic_read (&btnstate);

	/* We only need to remove the timer if we are still checking for its value, this is
         * only if it is still being tested */
	if (tmp) {
		del_timer_sync (&debounce_poll);
		tmp = mendoza_powerbutton_workaround_getvalue ();
	}  

	return tmp;
}

static void debounce_timeout_func (unsigned long data_pointer)
{
	unsigned int tmp;

	tmp = atomic_read (&btnstate);

	if (tmp) {
		tmp = mendoza_powerbutton_workaround_getvalue ();
		atomic_set (&btnstate, tmp);

		mod_timer(&debounce_poll, jiffies + DEBOUNCE_POLL);
	}
}

static void mendoza_pb_dev_release(struct device *dev) {}

static struct powerbutton_pdata mendoza_pb_pdata=
{
	.get_value         = mendoza_powerbutton_workaround_getvalue,
	.get_timed_value   = mendoza_powerbutton_get_timed_value,
};

static struct resource mendoza_pb_resources[]=
{
	{
		.name = "powerbutton",
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device pb = {
	.name = "powerbutton",
	.id = -1,
	.num_resources 	= ARRAY_SIZE(mendoza_pb_resources),
	.resource = mendoza_pb_resources,
	.dev = {
		.platform_data = &mendoza_pb_pdata,
		.release = mendoza_pb_dev_release,
	},
};

static int __init mendoza_pb_init(void)
{
	int irqnr, ret;

	irqnr  = gpio_to_irq(vgpio_to_gpio(TT_VGPIO_PWR_BUTTON));
	mendoza_pb_resources[0].start = irqnr;
	mendoza_pb_resources[0].end = irqnr;

	ret = gpio_request (TT_VGPIO_PWR_BUTTON, "wake");
	if (ret) {
		printk ("Could not get gpio number: %d\n", TT_VGPIO_PWR_BUTTON);
		return -1;
	}

	s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_PWR_BUTTON), S3C_GPIO_PULL_NONE);
	gpio_direction_input(TT_VGPIO_PWR_BUTTON);

	/* Start the timer that checks the button state for debounce */
	init_timer(&debounce_poll);
	debounce_poll.function = debounce_timeout_func;
	mod_timer (&debounce_poll, jiffies + 1);

	return platform_device_register(&pb);
}
arch_initcall(mendoza_pb_init);
