/*
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Benoit Leffray <benoit.leffray@tomtom.com>
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/platform_device.h>
#include <linux/ltc3577-pmic.h>
#include <linux/i2c.h>
#include <plat/regs-gpio.h>
#include <plat/mendoza.h>
#include <plat/tt_setup_handler.h>
#include <mach/gpio.h>
#include <mach/hardware.h>

static void ltc3577_init(void)
{
	printk(KERN_INFO LTC3577_DEVNAME ": Initialized\n");
	gpio_direction_output(TT_VGPIO_WALL_ON, 0);
}

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

static ltc3577_pdata_t  ltc3577_pdata = {
	.init		= ltc3577_init,
	.set_charge	= ltc3577_set_charge,
	.i2c_addr	= 0x13,
};

static struct i2c_board_info	ltc3577_i2c_info = {
	I2C_BOARD_INFO(LTC3577_DEVNAME, 9),
	&ltc3577_pdata,
};

static int __init ltc3577_register(void)
{
	int i2c_bus = 0;

	if(i2c_register_board_info(i2c_bus, &ltc3577_i2c_info, 1) < 0) {
		printk(KERN_INFO LTC3577_DEVNAME " I2C Board Registration Failure!\n" );
		return -3;
	}

	printk(KERN_ERR LTC3577_DEVNAME ": I2C Board Registered\n");
	return 0;
}

static int __init ltc3577_setup_cb(char * identifier)
{
	return ltc3577_register();
}

TT_SETUP_CB(ltc3577_setup_cb, LTC3577_DEVNAME);
