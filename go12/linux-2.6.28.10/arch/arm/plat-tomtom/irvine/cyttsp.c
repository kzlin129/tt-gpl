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

#include <linux/cyttsp.h>
#include <mach/pinmux.h>

#include <plat/tt_setup_handler.h>
#include <plat/irvine.h>
#include <plat/fdt.h>

static unsigned char bootloader_key_fw_v2[] = {0x4E, 0x69, 0x4C, 0x61, 0x56, 0x69, 0x44, 0x65};

#define I2C_BUS		0
#define I2C_ADDR	0x0A

static int cyttsp_gpio_request(void)
{
	int err = 0;

	if ((err = gpio_request(TT_VGPIO_TSP_PWR, "TT_VGPIO_TSP_PWR"))) {
		printk (KERN_ERR "Can't request TT_VGPIO_TSP_PWR GPIO\n");
		gpio_free(TT_VGPIO_TSP_PWR);
		return err;
	}

	if ((err = gpio_request(TT_VGPIO_TSP_CE, "TT_VGPIO_TSP_CE"))) {
		printk (KERN_ERR "Can't request TT_VGPIO_TSP_CE GPIO\n");
		gpio_free(TT_VGPIO_TSP_PWR);
		gpio_free(TT_VGPIO_TSP_CE);
		return err;
	}

	if ((err = gpio_request(TT_VGPIO_TSP_ATTN, "TT_VGPIO_TSP_ATTN"))) {
		printk (KERN_ERR "Can't request TT_VGPIO_TSP_ATTN GPIO\n");
		gpio_free(TT_VGPIO_TSP_PWR);
		gpio_free(TT_VGPIO_TSP_CE);
		gpio_free(TT_VGPIO_TSP_ATTN);
		return err;
	}

	return err;
}

static void cyttsp_config_gpio(void)
{
	gpio_direction_output(TT_VGPIO_TSP_PWR, 0);

	gpio_direction_input(TT_VGPIO_TSP_ATTN);
	reg_gpio_set_pull_up_down( vgpio_to_gpio(TT_VGPIO_TSP_ATTN), 1);
	reg_gpio_set_pull_up_down_enable(vgpio_to_gpio(TT_VGPIO_TSP_ATTN));

	gpio_direction_output(TT_VGPIO_TSP_CE, 0);

}

static void cyttsp_reset(void)
{
	// Reset active low
	gpio_set_value(TT_VGPIO_TSP_CE, 0);
	msleep(100);
	gpio_set_value(TT_VGPIO_TSP_CE, 1);
}

static void cyttsp_init(void)
{
	if (cyttsp_gpio_request()) {
		printk (KERN_ERR "Couldn't initialize Capacitive Touch Screen Controller\n");
		return;
	}

	cyttsp_config_gpio();

	cyttsp_reset();
}

static struct cyttsp_pdata cyttsp_pdata = 
{
	.reset_ts	= cyttsp_reset,
};

static struct i2c_board_info cyttsp_i2c_info = 
{
	.type			= CYTTSP_DEVNAME,
	.platform_data	= &cyttsp_pdata,
};

int __init irvine_cyttsp_init (void)
{
	printk (KERN_INFO "Initializing Cypress TTSP\n");

	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_TSP_ATTN), BCM4760_PIN_MUX_GPIO);
	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_TSP_CE), BCM4760_PIN_MUX_GPIO);
	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_TSP_PWR), BCM4760_PIN_MUX_GPIO);

	return 0;
}

static int __init tt_cyttsp_setup(char *str)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;

	printk("Enter tt_cyttsp_setup\n");
	
	irvine_cyttsp_init();

	cyttsp_init();

	cyttsp_i2c_info.irq = gpio_to_irq(TT_VGPIO_TSP_ATTN);

	memcpy(cyttsp_pdata.key, bootloader_key_fw_v2, SIZE_BOOTLOADER_KEY);

	cyttsp_i2c_info.addr = I2C_ADDR;

	adapter = i2c_get_adapter(I2C_BUS);
	if (adapter) {
		client = i2c_new_device(adapter, &cyttsp_i2c_info);
		if (!client) {
			printk(KERN_ERR CYTTSP_DEVNAME ": Can't add new I2C device\n" );
			kfree(cyttsp_pdata.key);
			return -ENODEV;
		}
	}
	
	printk(KERN_INFO "Cypress TTSP initialized\n");

	return 0;
}

TT_SETUP_CB(tt_cyttsp_setup, "tomtom-bcm-touchscreen-cypress");

