/*
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Vincent Dejouy <vincent.dejouy@tomtom.com>
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
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <plat/mendoza.h>
#include <plat/gpio-cfg.h>
#include <plat/tt_setup_handler.h>
#include <plat/kxr94_pdata.h>
#include <mach/gpio.h>


static int kxr94_gpio_request(void)
{

	return gpio_request(TT_VGPIO_DR_PWR_EN, "TT_VGPIO_DR_PWR_EN");

}

static void kxr94_gpio_free(void)
{
	gpio_free(TT_VGPIO_DR_PWR_EN);
}

static void kxr94_config_gpio(void)
{
	gpio_direction_output(TT_VGPIO_DR_PWR_EN, 0);
}

static void kxr94_pwr_en(void)
{
	gpio_set_value(TT_VGPIO_DR_PWR_EN, 1);	
}


static void kxr94_init(void)
{
	if (!kxr94_gpio_request()) {
		kxr94_config_gpio();
		kxr94_pwr_en();
	}
	printk(KERN_ERR KXR94_DEVNAME ": Initialized\n");
}

static kxr94_pdata_t  kxr94_pdata = {
	.init		= kxr94_init,
};


kxr94_pdata_t* setup_kxr94_pdata(void)
{
	return &kxr94_pdata;
}

