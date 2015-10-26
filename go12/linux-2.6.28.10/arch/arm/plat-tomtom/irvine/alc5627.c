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

#include <linux/i2c.h>

#include <plat/tt_setup_handler.h>

#define ALC5627_DEVNAME "alc5627"
#define PFX ALC5627_DEVNAME ": "

static struct i2c_board_info alc5627_i2c_info = 
{
	I2C_BOARD_INFO(ALC5627_DEVNAME, 0x18),
};

static int __init irvine_alc5627_setup (char *str)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int i2c_bus;

	i2c_bus = 0;		/* ALC5627 is on I2C bus 0 */

	adapter = i2c_get_adapter(i2c_bus);
	if (adapter)
	{
		client = i2c_new_device(adapter, &alc5627_i2c_info);
		if (!client) {
			printk(KERN_ERR ALC5627_DEVNAME ": Can't add new I2C device\n" );
			return -ENODEV;
		}
	}

	return 0;
}

TT_SETUP_CB(irvine_alc5627_setup, "tomtom-bcm-amplifier-alc5627");

