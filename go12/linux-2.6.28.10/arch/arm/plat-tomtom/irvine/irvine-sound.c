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
#include <linux/gpio.h>
#include <linux/vgpio.h>
#include <mach/pinmux.h>
#include <plat/irvine.h>

static struct platform_device irvine_device_asoc =
{
	.name	= "bcm476x-soc-0",
	.id	= -1,
};

int __init irvine_sound_init (void)
{
	platform_device_register(&irvine_device_asoc);

	return 0;
}

static int __init irvine_sound_amplifier_init( void )
{
	bcm4760_set_pin_mux( vgpio_to_gpio(TT_VGPIO_AMP_PWR_EN), BCM4760_PIN_MUX_GPIO );
	gpio_direction_output( TT_VGPIO_AMP_PWR_EN, 1 );

	return 0;
}
arch_initcall( irvine_sound_amplifier_init );

