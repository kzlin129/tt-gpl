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

#include <linux/platform_device.h>
#include <linux/broadcom/gpio.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/vgpio.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/atmel_maxtouch.h>
#include <mach/pinmux.h>
#include <plat/tt_setup_handler.h>
#include <plat/irvine.h>

static int atmel_gpio_request(void)
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

static void atmel_config_gpio(void)
{
	gpio_direction_output(TT_VGPIO_TSP_PWR, 0);
	gpio_direction_input(TT_VGPIO_TSP_ATTN);
	reg_gpio_set_pull_up_down( vgpio_to_gpio(TT_VGPIO_TSP_ATTN), 1);
	reg_gpio_set_pull_up_down_enable(vgpio_to_gpio(TT_VGPIO_TSP_ATTN));
	gpio_direction_output(TT_VGPIO_TSP_CE, 0);

}

static void atmel_reset_ts(void)
{
	//Reset active low
	gpio_set_value(TT_VGPIO_TSP_CE, 0);
	msleep(100);
	gpio_set_value(TT_VGPIO_TSP_CE, 1);
}

static void atmel_init(void)
{
	if (atmel_gpio_request())
	{
		printk(KERN_ERR "Couldn't initialize Atmel Capacitive Touch Screen Controller\n");
		return;
	}

	atmel_config_gpio();
	atmel_reset_ts();

}

static struct mxt_platform_data atmel_pdata;

static u8 read_attn_line(void)
{
	return gpio_get_value(TT_VGPIO_TSP_ATTN);
}

static struct i2c_board_info atmel_i2c_info = 
{
	.type = ATMEL_DEVNAME,
	.addr = 0x4A,
	.platform_data = &atmel_pdata,
};

int __init irvine_atmel_init (void)
{
	printk (KERN_INFO "Initializing Atmel MaxTouch\n");

	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_TSP_ATTN), BCM4760_PIN_MUX_GPIO);
	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_TSP_CE), BCM4760_PIN_MUX_GPIO);
	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_TSP_PWR), BCM4760_PIN_MUX_GPIO);

	return 0;
}

static int __init tt_atmel_setup(void)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int i2c_bus = 0; /* Atmel MaxTouch is on I2C bus 0 */

	printk("Enter tt_atmel_setup\n");
	
	irvine_atmel_init();

	atmel_pdata.init_platform_hw = NULL;
	atmel_pdata.exit_platform_hw = NULL;
	atmel_pdata.valid_interrupt  = NULL;
	atmel_pdata.read_chg	     = read_attn_line;
	atmel_pdata.max_x	     = 1023;
	atmel_pdata.max_y	     = 767;

	atmel_init();

        atmel_i2c_info.irq = gpio_to_irq(TT_VGPIO_TSP_ATTN);

	adapter = i2c_get_adapter(i2c_bus);
	if (adapter)
	{
		printk("Atmel TS: just before adding new i2c device\n");
		client = i2c_new_device(adapter, &atmel_i2c_info);
		if (!client) {
			printk(KERN_ERR ATMEL_DEVNAME ": Can't add new I2C device\n" );
			return -ENODEV;
		}

	} else {
		printk("Atmel TS: just before registering i2c board info\n");
		if(i2c_register_board_info(i2c_bus, &atmel_i2c_info, 1) < 0) {
			printk(KERN_ERR ATMEL_DEVNAME  ": Can't register Atmel Touch Screen I2C Board\n" );
			return -ENODEV;
		}
	}

	printk(KERN_INFO "Atmel MaxTouch initialized\n");

	return 0;
}

TT_SETUP_CB(tt_atmel_setup, "tomtom-bcm-touchscreen-atmel");

