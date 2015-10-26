/*
 *  LTC3577 PMIC PLATFORM DEVICE
 *
 *  Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 *  Author: Andrzej Zukowski <andrzej.zukowski@tomtom.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/mfd/ltc3577/pmic.h>
#include <plat/mendoza.h>
#include <plat/regs-gpio.h>
#include <mach/gpio.h>

#define LTC3577_BK1BRST_EN	(0x1 << 2)
#define LTC3577_BK2BRST_EN	(0x1 << 3)
#define LTC3577_BK3BRST_EN	(0x1 << 4)
#define LTC3577_BK_MASK		(0x7 << 2)

#define LTC3577_SLEW_1NS	(0x0 << 5)
#define LTC3577_SLEW_2NS	(0x1 << 5)
#define LTC3577_SLEW_4NS	(0x2 << 5)
#define LTC3577_SLEW_8NS	(0x3 << 5)
#define LTC3577_SLEW_MASK	(0x3 << 5)

static void ltc3577_set_charge(charge_e charge)
{		
	printk(KERN_INFO LTC3577_DEVNAME ": Drawing ");
	switch(charge) {
	  case eCHARGING_500mA:
		printk("500mA\n");
		break;	
	  case eCHARGING_1A:
		printk("1A\n");
		break;	
	}

	gpio_set_value(TT_VGPIO_WALL_ON, charge);
}

static struct ltc3577_pmic_data pdata = {
	.burst_mode = LTC3577_BK1BRST_EN |
		      LTC3577_BK2BRST_EN |
		      LTC3577_BK3BRST_EN,
	.slew_rate  = LTC3577_SLEW_1NS,
	.set_charge	= ltc3577_set_charge,
};

struct platform_device mendoza_ltc3577_pmic = {
	.name		= LTC3577_DEVNAME,
	.id		= -1,
	.dev		= {
		.platform_data	= &pdata,
	},
};

int __init ltc3577_gpio_init( void )
{
	int err =0;
	
	if ((err = gpio_request(TT_VGPIO_WALL_ON, "WALL_ON"))) {
				printk ("Could not get gpio number: %d\n", TT_VGPIO_WALL_ON);	
	}
	gpio_direction_output(TT_VGPIO_WALL_ON,0);
	
}
arch_initcall( ltc3577_gpio_init );
