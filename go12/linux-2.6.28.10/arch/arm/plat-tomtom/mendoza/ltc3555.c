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
#include <plat/regs-gpio.h>
#include <mach/hardware.h>
#include <linux/i2c.h>
#include <linux/ltc3555-pmic.h>
#include <plat/mendoza.h>
#include <plat/tt_setup_handler.h>

static struct ltc3555_volt      mendoza_ltc3555_sw3_power[]=
{
#if defined(CONFIG_TOMTOM_PMIC_LTC3555_CPU_FREQ)
#if 0
	/* Standard Samsung values. Don't use them on Seoul. */
	{ .frequency = 66000,  .voltage = 1000 },
	{ .frequency = 133000, .voltage = 1000 },
	{ .frequency = 222000, .voltage = 1050 },
	{ .frequency = 333000, .voltage = 1100 },
	{ .frequency = 667000, .voltage = 1200 },
#else
	{ .frequency = 133000, .voltage = 1100 },
	{ .frequency = 266000, .voltage = 1150 },
	{ .frequency = 533000, .voltage = 1215 },
	{ .frequency = 667000, .voltage = 1260 },
#endif
#else
        {533000, 1215}, {666000, 1260},
#endif
	{0, 0}
};

static struct ltc3555_volt      mendoza_ltc3555_sw2_power[]=
{
#if defined(CONFIG_TOMTOM_PMIC_LTC3555_CPU_FREQ)
	{ .frequency = 66000,  .voltage = 3328 },
	{ .frequency = 133000, .voltage = 3328 },
	{ .frequency = 222000, .voltage = 3328 },
	{ .frequency = 333000, .voltage = 3328 },
	{ .frequency = 667000, .voltage = 3328 },
#else
	{533000, 3328},
#endif
	{0, 0}
/*        {600000, 3328}, {0, 0}*/
};

static struct ltc3555_platform  mendoza_ltc3555_pdata=
{
        .sw2 = {316000, 100000, mendoza_ltc3555_sw2_power},
        .sw3 = {62000,  100000, mendoza_ltc3555_sw3_power},
        .swmode = {LTC3555_BURST, LTC3555_PULSE_SKIP, LTC3555_PULSE_SKIP},

	.cpu_clk = "fclk",

	.wall_pwr_pin	= {
		.pin	= TT_VGPIO_WALL_ON,
	},

	/* Although we don't explicitly enable Regulator-3 in the LTC, it is actually
	 * controled by the EN3 pin, which is logically or-ed with this register.
	 * That pin will be set to low level when the core actually suspends, hence
	 * cutting the power.
	 * We do not enable the other regulators, due to deasserting ENABLE pins would
   	 * not work (used by 'suicide' circuit).
	 */
	.initial_state = (LTC3555_DISABLE_BATTCHARGE | LTC3555_INPUT_CURRENT_DEFAULT),
};

static struct i2c_board_info ltc3555_i2c_info = {
	I2C_BOARD_INFO(LTC3555_DEVNAME, 9),
	.platform_data = &mendoza_ltc3555_pdata,
};

static int __init ltc3555_register(void)
{
	struct i2c_adapter *ltc3555;
	struct i2c_client *pmic;
	int busnr = 0;

	printk(KERN_INFO LTC3555_DEVNAME" I2C Board.. ");

	if( (ltc3555 = i2c_get_adapter(busnr) ) !=NULL ){
		if ( (pmic = i2c_new_device(ltc3555, &ltc3555_i2c_info) ) == NULL){
			printk(KERN_INFO"can't register LTC3555 I2C Board.\n");
			return -3;
		}
	} else { 
		printk(KERN_INFO"cannot get adapter trying to register boardinfo.. \n");
		if(i2c_register_board_info(0, &ltc3555_i2c_info, 1) < 0) {
			printk(KERN_INFO"can't register LTC3555 I2C Board.\n" );
			return -3;
		}
	}
	printk(KERN_INFO"registered.\n");
	return 0;
};

static int __init ltc3555_setup_cb(char * identifier)
{
	return ltc3555_register();
};

TT_SETUP_CB(ltc3555_setup_cb , LTC3555_DEVNAME);
