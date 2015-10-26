/*
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Chris Liu <chris.liu@tomtom.com>
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
#include <linux/broadcom/gpio.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/vgpio.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <asm/io.h>
#include <linux/himax_hx8520.h>
#include <mach/pinmux.h>
#include <plat/tt_setup_handler.h>
#include <plat/irvine.h>
#include <plat/fdt.h>

static int hx_gpio_request(void)
{
	int err = 0;

	if ((err = gpio_request(TT_VGPIO_TSP_PWR, "TT_VGPIO_TSP_PWR"))) {
		printk(KERN_ERR "Can't request TT_VGPIO_TSP_PWR GPIO\n");
		gpio_free(TT_VGPIO_TSP_PWR);
		return err;
	}

	if ((err = gpio_request(TT_VGPIO_TSP_CE, "TT_VGPIO_TSP_CE"))) {
		printk(KERN_ERR "Can't request TT_VGPIO_TSP_CE GPIO\n");
		gpio_free(TT_VGPIO_TSP_PWR);
		gpio_free(TT_VGPIO_TSP_CE);
		return err;
	}

	if ((err = gpio_request(TT_VGPIO_TSP_ATTN, "TT_VGPIO_TSP_ATTN"))) {
		printk(KERN_ERR "Can't request TT_VGPIO_TSP_ATTN GPIO\n");
		gpio_free(TT_VGPIO_TSP_PWR);
		gpio_free(TT_VGPIO_TSP_CE);
		gpio_free(TT_VGPIO_TSP_ATTN);
		return err;
	}

	return err;
}

static void hx_config_gpio()
{
	gpio_direction_input(TT_VGPIO_TSP_ATTN);
	reg_gpio_set_pull_up_down(vgpio_to_gpio(TT_VGPIO_TSP_ATTN), 1);
	reg_gpio_set_pull_up_down_enable(vgpio_to_gpio(TT_VGPIO_TSP_ATTN));
	gpio_direction_output(TT_VGPIO_TSP_CE, 0);
}

static void hx_reset_ts(void)
{
	gpio_direction_output(TT_VGPIO_TSP_CE, 1);
	msleep(10);
	gpio_direction_output(TT_VGPIO_TSP_CE, 0);
	msleep(1);
	gpio_direction_output(TT_VGPIO_TSP_CE, 1);
}

static void hx_init()
{
	hx_gpio_request();
	hx_config_gpio();
	hx_reset_ts();
}

static int hx_get_intr_line()
{
	return gpio_get_value(TT_VGPIO_TSP_ATTN);
}

static struct hx_pdata_t hx_pdata;
static struct i2c_board_info hx_i2c_info = 
{
	.type = HX_DEVNAME,
	.platform_data = &hx_pdata,
};

int __init irvine_hx_init (void)
{
	printk (KERN_INFO "Initializing Himax HX8520\n");
	
	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_TSP_ATTN), BCM4760_PIN_MUX_GPIO);
	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_TSP_CE), BCM4760_PIN_MUX_GPIO);
	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_TSP_PWR), BCM4760_PIN_MUX_GPIO);
	
	return 0;
}

static int __init tt_himax_setup(char *str)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int i2c_bus = 0; /* HX8520 is on I2C bus 0 */

	hx_pdata.reset_ts = hx_reset_ts;
	hx_pdata.get_intr_line = hx_get_intr_line;
	hx_i2c_info.irq = gpio_to_irq(TT_VGPIO_TSP_ATTN);
	hx_i2c_info.addr = 0x4B;

	hx_init();

	adapter = i2c_get_adapter(i2c_bus);
	if (adapter) 
	{
		client = i2c_new_device(adapter, &hx_i2c_info);
		if (!client) {
			printk(KERN_ERR HX_DEVNAME ": Can't add new I2C device\n" );
			return -ENODEV;
		}
	}
	
	printk(KERN_INFO "Himax HX8520 initialized\n");

	return 0;
}

TT_SETUP_CB(tt_himax_setup, "tomtom-bcm-touchscreen-himax");

