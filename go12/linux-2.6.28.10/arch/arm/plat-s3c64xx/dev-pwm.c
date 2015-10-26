/*
 * Copyright (C) 2008 TomTom BV <http://www.tomtom.com/>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/
 
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <asm/delay.h>
#include <plat/map.h>
#include <mach/gpio.h>
#include <plat/mendoza.h>
#include <plat/pwm_pdata.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>
#include <plat/gpio-bank-f.h>

static int s3c_pwm_config_gpio (int id)
{
	int ret = -1;

	if (id == 0) {
		if(gpio_is_valid(S3C64XX_GPF(14))) {
			ret = gpio_request(S3C64XX_GPF(14), "GPF");

			if (ret) {
				printk(KERN_ERR "failed to request GPF for PWM-OUT 0\n");
			}
			s3c_gpio_cfgpin(S3C64XX_GPF(14),S3C64XX_GPF14_PWM_TOUT0);			 
		}
	} else if(id == 1) {
		if(gpio_is_valid(S3C64XX_GPF(15))) {
			ret = gpio_request(S3C64XX_GPF(15), "GPF");

			if (ret) {
				printk(KERN_ERR "failed to request GPF for PWM-OUT 1\n");
			}
			s3c_gpio_cfgpin(S3C64XX_GPF(15),S3C64XX_GPF15_PWM_TOUT1);			 
		}
	
	} else {
		printk(KERN_ERR "This PWM dosen't support PWM out\n");
	}

	if (id == 4) {
		printk(KERN_ERR "TIMER4 is currently not supported\n");
		return -ENXIO;
	}
	return ret;
}

static struct pwm_pdata pdata = {
	.pwm_config_gpio  = s3c_pwm_config_gpio,
};

/* Standard setup for a timer block. */

#define TIMER_RESOURCE_SIZE (1)

#define TIMER_RESOURCE(_tmr, _irq)			\
	(struct resource [TIMER_RESOURCE_SIZE]) {	\
		[0] = {					\
			.start	= _irq,			\
			.end	= _irq,			\
			.flags	= IORESOURCE_IRQ	\
		}					\
	}

#define DEFINE_S3C_TIMER(_tmr_no, _irq)			\
	.name		= "s3c24xx-pwm",		\
	.id		= _tmr_no,			\
	.num_resources	= TIMER_RESOURCE_SIZE,		\
	.resource	= TIMER_RESOURCE(_tmr_no, _irq),\
	.dev = {					\
		.platform_data = &pdata, 		\
	},						\

/* since we already have an static mapping for the timer, we do not
 * bother setting any IO resource for the base.
 */

struct platform_device s3c_device_timer[] = {
	[0] = { DEFINE_S3C_TIMER(0, IRQ_TIMER0) },
	[1] = { DEFINE_S3C_TIMER(1, IRQ_TIMER1) },
	[2] = { DEFINE_S3C_TIMER(2, IRQ_TIMER2) },
	[3] = { DEFINE_S3C_TIMER(3, IRQ_TIMER3) },
	[4] = { DEFINE_S3C_TIMER(4, IRQ_TIMER4) },
};

