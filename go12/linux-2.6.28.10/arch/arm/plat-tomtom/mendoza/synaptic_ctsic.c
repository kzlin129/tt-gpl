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


//#include <asm/arch/regs-gpio.h>
//#include <asm/arch/gpio.h>
//#include <asm/hardware.h>
//#include <asm/irq.h>
#include <linux/i2c.h>
#include <linux/platform_device.h> 
#include <linux/rmi_i2c.h>
#include <linux/io.h>
#include <plat/mendoza.h>
#include <plat/gpio-cfg.h>
#include <plat/tt_setup_handler.h>
#include <mach/gpio.h>


static int synaptic_gpio_request(void)
{
	int err = 0;

	if ((err = gpio_request(TT_VGPIO_DOCK_I2C_EN, "TT_VGPIO_DOCK_I2C_EN"))) {
		printk(KERN_INFO SYNAPTIC_DEVNAME ":Can't request TT_VGPIO_DOCK_I2C_EN GPIO\n");
		gpio_free(TT_VGPIO_DOCK_I2C_EN);
		return err;
		
	}

	if ((err = gpio_request(TT_VGPIO_ACCESSORY_PWR_EN, "TT_VGPIO_ACCESSORY_PWR_EN"))) {
		printk(KERN_INFO SYNAPTIC_DEVNAME ":Can't request TT_VGPIO_ACCESSORY_PWR_EN GPIO\n");
		gpio_free(TT_VGPIO_DOCK_I2C_EN);
		gpio_free(TT_VGPIO_ACCESSORY_PWR_EN);
		return err;
	}

	return err;

}

static void synaptic_gpio_free(void)
{
	gpio_free(TT_VGPIO_DOCK_I2C_EN);
	gpio_free(TT_VGPIO_ACCESSORY_PWR_EN);	
}

static void synaptic_config_gpio(void)
{
	// switch on i2c on dock connector (needed for temporary wire up with torinos)
	gpio_direction_output(TT_VGPIO_DOCK_I2C_EN, 0);
	// power of synaptic chip
	gpio_direction_output(TT_VGPIO_ACCESSORY_PWR_EN, 0);
	//pin_setirq(&pins.pHPDETECT);
	//s3c_gpio_pullup(S3C_GPM1, 0);   /* Pull-up/down disable */
}

static void synaptic_pwr_en(void)
{
	//pin_activate(&pins.pI2C0_3V3_ON);
	gpio_set_value(TT_VGPIO_DOCK_I2C_EN, 1);
	gpio_set_value(TT_VGPIO_ACCESSORY_PWR_EN, 1);
}

static int synaptic_get_attention(void)
{
        //return pin_getinput(&pins.pHPDETECT);
	return 1;
}

static void synaptic_init(void)
{
	synaptic_gpio_request();
	synaptic_config_gpio();
	synaptic_pwr_en();
	printk(KERN_INFO SYNAPTIC_DEVNAME ": Initialized\n");
}


// contains i2c bus number and low level hardware init function for each rmi peripheral
// platform data for each i2c board
static rmi_pdata_t  rmi_plat_data[] = {
	[0] = {
		.init		= synaptic_init,
		.get_attention  = synaptic_get_attention,
		.i2c_bus	= 0,
	}
};


static struct i2c_board_info	rmi_i2c_boards_info[] = {
	[0] = {
		I2C_BOARD_INFO(SYNAPTIC_DEVNAME, 0x20 /*real i2c adress */),
	}
};

static int __init register_rmi_i2c_peripherals(void)
{
	int i;
	int i2c_bus = 0;
	int err;
	struct i2c_adapter * adapter;
	struct i2c_client * client;

        printk(KERN_INFO SYNAPTIC_DEVNAME ": Enter Register function\n");

	synaptic_init();

	for (i=0;i<ARRAY_SIZE(rmi_i2c_boards_info);i++){

		rmi_i2c_boards_info[i].platform_data = &(rmi_plat_data[i]);
		if (i == 0){
			//rmi_i2c_boards_info[i].irq = pin_getirq(&pins.pHPDETECT);
			rmi_i2c_boards_info[i].irq = 0;
		}
		
		adapter = i2c_get_adapter(rmi_plat_data[i].i2c_bus);

		if (adapter) {
			client = i2c_new_device(adapter, &rmi_i2c_boards_info[i]);
			if (!client) {
				printk(KERN_INFO SYNAPTIC_DEVNAME ": Can't add new device Synaptic Cap Touch Screen I2C Board\n" );
				return -ENODEV;
			}
		} else {
			if((err = i2c_register_board_info(rmi_plat_data[i].i2c_bus, &rmi_i2c_boards_info[i], 1)) < 0) {
				printk(KERN_INFO SYNAPTIC_DEVNAME  ": Can't register Synaptic Touch Screen I2C Board\n" );
				return err;
			}
		}

		printk(KERN_INFO SYNAPTIC_DEVNAME ": I2C Board Registered, i2c addr: 0x%x i2c bus: %d\n", rmi_i2c_boards_info[i].addr, rmi_plat_data[i].i2c_bus);
	
	}
        
	return 0;
}

static int __init synaptic_setup_cb(char * identifier)
{
        printk(KERN_INFO SYNAPTIC_DEVNAME ": Enter Init function\n");
	return register_rmi_i2c_peripherals();
}

TT_SETUP_CB(synaptic_setup_cb, SYNAPTIC_DEVNAME);
