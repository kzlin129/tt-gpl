/*
 *  Backlight PLATFORM DEVICE using the s5p6440 PWM interface 
 *
 *  Copyright (c) 2004-2009 TomTom
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/pwm.h>
#include <linux/vgpio.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <mach/gpio.h>
#include <linux/pwm_backlight.h>
#include <plat/mendoza.h>


#define PWM_BL_NAME		"pwm-backlight"
#define PWM_BL_PFX		PWM_BL_NAME " Dev: "

#define PWM_BL_INVERTED			1
#define PWM_BL_MAX_INTENSITY		100
#define PWM_BL_DEFAULT_INTENSITY	35 
#define	PWM_BL_PWM_CHANNEL		1


/* Make the brightness behave linearly from user space, 
   we cannot go below 18% (it is impossible to see anything,
   and neither above 50% because it draws to much current*/

/* This table is calculated as:
     x=5   => y=18 
     x=100 => y=50

     y=ROUND(x*(32/95)+16.32)
*/

#ifdef CONFIG_MACH_VENICE
static int pwm_curve_lut[]={ 
	 14 ,		/*   0 */
	 17 ,		/*   5 */
	 19 ,		/*  10 */
	 22 ,		/*  15 */
	 24 ,		/*  20 */
	 27 ,		/*  25 */
	 29 ,		/*  30 */
	 32 ,		/*  35 */
	 34 ,		/*  40 */
	 37 ,		/*  45 */
	 39 ,		/*  50 */
	 42 ,		/*  55 */
	 44 ,		/*  60 */
	 47 ,		/*  65 */
	 49 ,		/*  70 */
	 52 ,		/*  75 */
	 54 ,		/*  80 */
	 57 ,		/*  85 */
	 59 ,		/*  90 */
	 62 ,		/*  95 */
	 65 ,		/* 100 */
};
#else
static int pwm_curve_lut[]={ 
	 1 ,		/*   0 */
	 18 ,		/*   5 */
	 20 ,		/*  10 */
	 21 ,		/*  15 */
	 23 ,		/*  20 */
	 25 ,		/*  25 */
	 26 ,		/*  30 */
	 28 ,		/*  35 */
	 30 ,		/*  40 */
	 31 ,		/*  45 */
	 33 ,		/*  50 */
	 35 ,		/*  55 */
	 37 ,		/*  60 */
	 38 ,		/*  65 */
	 40 ,		/*  70 */
	 42 ,		/*  75 */
	 43 ,		/*  80 */
	 45 ,		/*  85 */
	 47 ,		/*  90 */
	 48 ,		/*  95 */
	 50 ,		/* 100 */
};
#endif

static void tomtom_bl_activate(int activation)
{
	switch(activation) {
	  case 0:
		printk(KERN_INFO PWM_BL_PFX "Deactivated\n");
		break;

	  case 1:
		printk(KERN_INFO PWM_BL_PFX "Activated\n");
		break;

	  default:
		printk(KERN_ERR PWM_BL_PFX "Unknown activation!\n");
		return;
	}

	/* set backlight enable pin */
	gpio_set_value(TT_VGPIO_BACKLIGHT_EN, activation);
}

static int tomtom_convert_brightness(int intensity)
{
	int real_intensity;

	if (intensity%5 > 2)
		real_intensity = pwm_curve_lut[(intensity/5)+1];
	else
		real_intensity = pwm_curve_lut[intensity/5];

	if(PWM_BL_INVERTED)
		real_intensity = 100 - real_intensity;

	return real_intensity; 
}

static int tomtom_backlight_notify(int brightness)
{
	static int current_state = 0;

	if (!brightness) {
		if (current_state) {
			tomtom_bl_activate(0);			
			current_state = 0;
		}
	} else {
		if (!current_state) {			
			tomtom_bl_activate(1);
			current_state = 1;
				
		}
	}
	
	return tomtom_convert_brightness(brightness);
}

static int tomtom_backlight_init (struct device *dev)
{
	int err=0;

	err = gpio_request(TT_VGPIO_BACKLIGHT_EN, "BACKLIGHT_EN");

	if (err) {
		printk(KERN_ERR "error requesting BACKLIGHT_EN %d\n",TT_VGPIO_BACKLIGHT_EN);
		
	} else {
		s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_BACKLIGHT_EN), S3C_GPIO_PULL_NONE);
		gpio_direction_output(TT_VGPIO_BACKLIGHT_EN, 0);
	}

	return err;
}

static void tomtom_backlight_exit(struct device *dev)
{
	gpio_free(TT_VGPIO_BACKLIGHT_EN);
}

static struct platform_pwm_backlight_data tomtom_backlight_data = {
	.pwm_id		= PWM_BL_PWM_CHANNEL,
	.max_brightness	= PWM_BL_MAX_INTENSITY,
	.dft_brightness	= PWM_BL_DEFAULT_INTENSITY,
	.pwm_period_ns	= 40000, /*frequency of 50kHz*/
	.init		= tomtom_backlight_init,
	.notify		= tomtom_backlight_notify,
	.exit		= tomtom_backlight_exit,
};

static struct platform_device tomtom_backlight_device = {
	.name		= "pwm-backlight",
	.id		= -1,
	.dev		= {
		.parent		= &s3c_device_timer[1].dev,
		.platform_data	= &tomtom_backlight_data,
	},
};


int tomtom_bl_setup(void)
{
	printk("TomTom backlight device registered\n");
	return platform_device_register(&tomtom_backlight_device);
}
//arch_initcall(tomtom_bl_setup);
