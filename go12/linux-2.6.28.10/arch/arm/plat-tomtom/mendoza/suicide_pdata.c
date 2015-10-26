/* 
 * Copyright (C) 2010 TomTom BV <http://www.tomtom.com/>
 * Author: Niels Langendorff niels.langendorff@tomtom.com>
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
#include <linux/io.h>
#include <plat/mendoza.h>
#include <plat/gpio-cfg.h>
#include <plat/tt_setup_handler.h>
#include <linux/pm.h>
#include <mach/gpio.h>

static int suicide_gpio_request(void)
{
	return gpio_request(TT_VGPIO_L3_MODE, "TT_VGPIO_PWR_KILL");
}

static void suicide_gpio_free(void)
{
	gpio_free(TT_VGPIO_PWR_KILL);
}

static void suicide_config_gpio(void)
{
	gpio_direction_output(TT_VGPIO_PWR_KILL, 1);
	gpio_set_value(TT_VGPIO_PWR_KILL, 1);	
}

static void suicide_power_off(void)
{
	gpio_set_value(TT_VGPIO_PWR_KILL, 0);	
}

static int __init suicide_register( void )
{
	int err;

	if (!gpio_is_valid(TT_VGPIO_PWR_KILL)) {
		return -1;
	}

	if ((err = suicide_gpio_request())) {
		printk("%s: Can't request TT_VGPIO_PWR_KILL GPIO\n", __func__);
		return err;
	}
	suicide_config_gpio();
	pm_power_off = suicide_power_off;

	return 0;
}
arch_initcall( suicide_register );
