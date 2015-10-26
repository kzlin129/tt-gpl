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
#include <linux/pmu_device.h>
#include <linux/gpio.h>
#include <linux/vgpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <asm/arch/hardware.h>
#include <asm/arch/bcm4760_reg.h>

#include <plat/irvine.h>
//#include <plat/irvine-pmu.h>

#include <linux/broadcom/pmu_bcm59040.h>
#include <asm/arch/pmu_device_bcm59040.h>
#include <linux/broadcom/bcm59040_usbstat.h>

bcm59040_client_platform_data_t bcm59040_battery_platdata =
{
	.base = {
		.client_type	= BCMPMU_BATTERY,
		.index		= -1,
	},
	
};

bcm59040_client_platform_data_t bcm59040_control_platdata =
{
	.base = {
		.client_type	= BCMPMU_CONTROL,
		.index		= -1,
	},
};

bcm59040_client_platform_data_t bcm59040_charger_platdata =
{
	.base = {
		.client_type	= BCMPMU_CHARGER,
		.index		= -1,
	},
};

bcm59040_client_platform_data_t bcm59040_regulator_platdata =
{
	.base = {
		.client_type	= BCMPMU_REGULATOR,
		.index		= -1,
	},
};

bcm59040_client_platform_data_t bcm59040_rtc_platdata =
{
	.base = {
		.client_type	= BCMPMU_RTC,
		.index		= -1,
	},
};

bcm59040_client_platform_data_t bcm59040_adc_platdata =
{
	.base = {
		.client_type	= BCMPMU_ADC,
		.index		= -1,
	},
};

struct bcm59040_usbstat_pdata bcm59040_usbstat_priv_platdata=
{
	.bcm4760_cmu_base= IO_ADDRESS( CMU_R_CMU_CTL0_MEMADDR ),
	.bcm4760_usb_base= IO_ADDRESS( USB_R_GOTGCTL_MEMADDR ),
	.bcm59040_usb_irq= BCM59040_IRQ_USB,
};

bcm59040_client_platform_data_t bcm59040_usbstat_platdata =
{
	.base = {
		.client_type	= BCMPMU_USBSTAT,
		.index		= -1,
	},
	.extra			= &bcm59040_usbstat_priv_platdata,
};

/* GPIO init table - change this as necessary */
static BCM_PMU_gpioInit_t gpioInitTable[BCM59040_MAX_GPIO] = {
	{
		gpioDir: GPIO_INPUT,
		gpioData: 0,
		gpioMode: BCM59040_GPIOX_MODE_NORMAL,
	},
	{
		gpioDir: GPIO_INPUT,
		gpioData: 0,
		gpioMode: BCM59040_GPIOX_MODE_NORMAL,
	},
	{
		gpioDir: GPIO_INPUT,
		gpioData: 0,
		gpioMode: BCM59040_GPIOX_MODE_NORMAL,
	},
	{
		gpioDir: GPIO_INPUT,
		gpioData: 0,
		gpioMode: BCM59040_GPIOX_MODE_NORMAL,
	},
	{
		gpioDir: GPIO_INPUT,
		gpioData: 0,
		gpioMode: BCM59040_GPIOX_MODE_NORMAL,
	},
};

static BCM_PMU_gpioPlatformData_t gpioInitData = {
	.numGPIO		= ARRAY_SIZE(gpioInitTable),
	.gpioInitTable		= gpioInitTable,
};

bcm59040_client_platform_data_t bcm59040_gpio_platdata =
{
	.base = {
		.client_type	= BCMPMU_GPIO,
		.index		= -1,
	},
	.extra		=&gpioInitData,
};

/** bcm59040_pmu_boards - PMU boards structure containing all BCM59040 subsystem subdevices.
 *
 * This list contains a complete list of all of the related BCM59040 subsystem devices.
 * This "core" driver will register this list of boards with the PMU Device framework
 * after it finishes its own initialization. The subsystem drivers will then register
 * against these boards.
 *
 */
static struct pmu_board_info bcm59040_pmu_boards[] =
{
	[0] =   /* BCM59040 control device registration data */
	{
		.name       = "control",
		.platform_data   = &bcm59040_control_platdata,
	},
	[1] =   /* BCM59040 GPIO device registration data */
	{
		.name       = "gpio",
		.platform_data  = &bcm59040_gpio_platdata,
	},
	[2] =   /* BCM59040 charger device registration data */
	{
		.name       = "charger",
		.platform_data  = &bcm59040_charger_platdata,
	},
	[3] =   /* BCM59040 ADC device registration data */
	{
		.name       = "adc",
		.platform_data  = &bcm59040_adc_platdata,
	},
	[4] =   /* BCM59040 regulator device registration data */
	{
		.name       = "regulator",
		.platform_data  = &bcm59040_regulator_platdata,
	},
	[5] =   /* BCM59040 RTC device registration data */
	{
		.name       = "rtc",
		.platform_data  = &bcm59040_rtc_platdata,
	},
	[6] =   /* BCM59040 battery maintenance device registration data */
	{
		.name       = "battery",
		.platform_data  = &bcm59040_battery_platdata,
	},
	[7] =   /* BCM59040 USBSTAT Emulation data. */
	{
		.name		= "usbstat",
		.platform_data	= &bcm59040_usbstat_platdata,
	},
};

static struct pmu_bcm59040_platform_data bcm59040_pdata =
{
	.pmu_board_info         = bcm59040_pmu_boards,
	.pmu_board_info_entries = ARRAY_SIZE(bcm59040_pmu_boards),
};

static struct i2c_board_info bcm59040_i2c_info = 
{
	I2C_BOARD_INFO(BCM59040_DEVNAME, 0x08),
	.platform_data = &bcm59040_pdata,
};

void irvine_pmu_register_defaults(char *board, void *defaults)
{
	bcm59040_client_platform_data_t *pdata;
	int i;

	if (defaults == NULL)
		return;

	for (i = 0; i < sizeof(bcm59040_pmu_boards)/sizeof(struct pmu_board_info); i++) {
		if (strcmp(bcm59040_pmu_boards[i].name, board) == 0) {
			printk ("Registering defaults for: %s\n", board);
			pdata = (bcm59040_client_platform_data_t*) bcm59040_pmu_boards[i].platform_data;
			pdata->defaults = defaults;
		}
	}
}


static int __init irvine_pmu_init(void)
{
	int i2c_bus = 0;

	bcm59040_i2c_info.irq = gpio_to_irq(TT_VGPIO_PMU_IRQ);

	if (i2c_register_board_info(i2c_bus, &bcm59040_i2c_info, 1) < 0) {
		printk(KERN_ERR PFX "I2C Board Registration Failure!\n" );
		return -3;
	}

	printk(KERN_INFO PFX "I2C Board Registered (gpio = %d, irq = %d !!!)\n", vgpio_to_gpio(TT_VGPIO_PMU_IRQ), bcm59040_i2c_info.irq);

	return 0;
}

arch_initcall(irvine_pmu_init);
