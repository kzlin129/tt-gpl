/*
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
#include <linux/i2c/at24.h>
#include <linux/i2c.h>

static struct i2c_board_info at24_eeprom_i2c = 
{
        I2C_BOARD_INFO("24c01", 0x50),
};

static int __init at24_eeprom_register(void)
{
	int i2c_bus = 0;

	if(i2c_register_board_info(i2c_bus, &at24_eeprom_i2c, 1) < 0) {
		printk( KERN_ERR "Failed to register AT24 EEPROM.\n" );
		return -1;
	}

	printk(KERN_INFO "AT24xxxx I2C Board Registered\n");
	return 0;
}

arch_initcall(at24_eeprom_register);
