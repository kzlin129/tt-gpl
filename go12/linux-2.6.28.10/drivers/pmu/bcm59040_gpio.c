/*****************************************************************************
 * Copyright 2003 - 2010 Broadcom Corporation.  All rights reserved.
 *
 * Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2, available at
 * http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
 *
 * Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a
 * license other than the GPL, without Broadcom's express prior written
 * consent.
 *****************************************************************************/

/**
 * BCM59040 PMU GPIO Driver
 *****************************************************************************
 *
 *     This implements the BCM59040 PMU GPIO driver. The BCM59040 includes
 *     5 general purpose I/O pins and this driver makes them available via
 *     the standard Linux GPIO driver and user level libraries. Not all 5
 *     pins may be available as GPIOs because GPIO3-5 are dual purpose and
 *     may be programmed for special (non-GPIO) functions. This is done via
 *     the gpioInitTable structure which controls the initial configuration
 *     of the GPIO pins including special function and initial direction.
 *     This gpioInitTable structure is generally found in pmu_bcm59040.c but
 *     may be in another architecture specific location.
 *
 *
 ****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>

#include <linux/platform_device.h>
#include <linux/pmu_device.h>
#include <asm/arch/pmu_device_bcm59040.h>

#include <linux/broadcom/pmu_bcm59040.h>

/* Include external interfaces */

#include <asm/gpio.h>
#include <asm/arch/bcm59040_irqs.h>
#include <asm/arch/gpio.h>

//#include <linux/pmu/bcm59040_core.h> // not sure if this is needed yet

/* Include internal interfaces */


/*
* Structure and macro definitions.
*/

#define BCM59040_GPIO_DRIVER				"bcm59040_gpio"
#define BCM59040_GPIO_MOD_DESCRIPTION		"BCM59040 GPIO Driver"
#define BCM59040_GPIO_MOD_VERSION			1.0

#undef BCM59040_DEVNAME
#define BCM59040_DEVNAME					BCM59040_GPIO_DRIVER
#define PFX									BCM59040_DEVNAME ": "

/* Debug logging */
#ifdef DEBUG
#undef DEBUG
#endif
//#define DEBUG 1

#define DBG_ERROR	0x01
#define DBG_INFO	0x02
#define DBG_TRACE	0x04
#define DBG_TRACE2	0x08
#define DBG_DATA	0x10
#define DBG_DATA2	0x20

#ifdef DEBUG
//#define DBG_DEFAULT_LEVEL	(DBG_ERROR | DBG_INFO | DBG_TRACE)
#define DBG_DEFAULT_LEVEL	(DBG_ERROR | DBG_INFO)

static int 		gLevel = DBG_DEFAULT_LEVEL;

#	define PMU_DEBUG(level,x) {if (level & gLevel) printk x;}
#else
#	define PMU_DEBUG(level,x)
#endif

typedef struct bcm59040_gpio_config_struct
{
	struct gpio_chip			chip;
	struct pmu_client			*pclient;
} bcm59040_gpio_config_t;

/*
* Local variables.
*/



/*
 * Function definitions.
 */

static int bcm59040_gpio_set_mode(struct pmu_client *pclient, int gpioNum, int ioMode)
{
	int			rc=0;
	uint8_t		data;
	u8			regAddr;
	
	if((gpioNum < 1) && (ioMode != BCM59040_GPIOX_MODE_NORMAL)) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_gpio_set_mode: gpio %d is not available for special mode.\n", gpioNum));
		return -EINVAL;
	}
	
	regAddr = BCM59040_REG_GPIOCTRL1 + gpioNum;
	if((rc = pmu_bus_seqread(pclient, regAddr, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error %d reading BCM59040_REG_GPIOCTRL%d register.\n", rc, gpioNum));
		return -EINVAL;
	}
	
	data = (~BCM59040_GPIOCTRLX_GPIO_MODE_MASK & (unsigned char)data) | (ioMode << 3);
	if((rc = pmu_bus_seqwrite(pclient, regAddr, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error %d writing BCM59040_REG_GPIOCTRL%d register.\n", rc, gpioNum));
		return -EINVAL;
	}

	return 0;
}

static int bcm59040_gpio_set_direction(struct pmu_client *pclient, int gpioNum, BCM_PMU_gpio_dir_t ioDir, int ioData)
{
	int			rc=0;
	uint8_t		data;
	u8			regAddr;

	if((gpioNum > 4) || (gpioNum < 0))
	{
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_gpio_set_direction: gpio %d not available.\n", gpioNum));
		return -EINVAL;
	}

	/* If HW based OTG is used, GPIO0 and GPIO1 cannot be configured as output */
	if((rc = pmu_bus_seqread(pclient, BCM59040_REG_OTGCTRL2, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_OTGCTRL2 register.\n"));
		return -EINVAL;
	}

	if((BCM59040_OTGCTRL2_HWVSSW_MASK & data) >> 4) {
		if(gpioNum < 2) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_gpio_set_direction: gpio %d cannot be configured in HW OTG CTRL mode\n",
gpioNum));
			return -EINVAL;
		}
	}

	regAddr = BCM59040_REG_GPIOCTRL1 + gpioNum;
	if((rc = pmu_bus_seqread(pclient, regAddr, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_GPIOCTRL%d register.\n",gpioNum));
		return -EINVAL;
	}

	data = (~BCM59040_GPIOCTRLX_GPIODIR_MASK & data) | ioDir;
	if(ioDir == GPIO_OUTPUT) {
		data &= ~BCM59040_GPIOCTRLX_GPIO_DATA;
		data |= (ioData << 2);
	}

	if((rc = pmu_bus_seqwrite(pclient, regAddr, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_GPIOCTRL%d register.\n",gpioNum));
		return -EINVAL;
	}

	return 0;
}

static int bcm59040_gpio_read_data(struct pmu_client *pclient, int gpioNum, int *ioData)
{
	int			rc=0;
	uint8_t		data;
	u8			regAddr;

	if((gpioNum > 4) || (gpioNum < 0)) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_gpio_read_data: gpio %d not available.\n", gpioNum));
		return -EINVAL;
	}

	regAddr = BCM59040_REG_GPIOCTRL1 + gpioNum;
	if((rc = pmu_bus_seqread(pclient, regAddr, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_GPIOCTRL%d register.\n",gpioNum));
		return -EINVAL;
	}

	*ioData = (BCM59040_GPIOCTRLX_GPIO_DATA & data) >> 2;

	return 0;
}

static int bcm59040_gpio_write_data(struct pmu_client *pclient, int gpioNum, int ioData)
{
	int			rc=0;
	uint8_t		data;
	u8			regAddr;

	if ((gpioNum > 4) || (gpioNum < 0)) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_gpio_write_data: gpio %d not available.\n", gpioNum));
		return -EINVAL;
	}

	regAddr = BCM59040_REG_GPIOCTRL1 + gpioNum;
	if((rc = pmu_bus_seqread(pclient, regAddr, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_GPIOCTRL%d register.\n",gpioNum));
		return -EINVAL;
	}

	/* clear it */
	data = (~BCM59040_GPIOCTRLX_GPIO_DATA & data);

	data = ((ioData << 2) | data);

	if((rc = pmu_bus_seqwrite(pclient, regAddr, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_GPIOCTRL%d register.\n",gpioNum));
		return -EINVAL;
	}

	return 0;
}

static int bcm59040_gpio_config(struct pmu_client *pclient)
{
	int							i = 0;
	bcm59040_client_platform_data_t		*plat_data = pclient->dev.platform_data;
	BCM_PMU_gpioPlatformData_t			*gpio_plat_data = plat_data->extra;

	for(i=0; i < gpio_plat_data->numGPIO; i++) {
		bcm59040_gpio_set_mode(pclient, i, gpio_plat_data->gpioInitTable[i].gpioMode);
		if(gpio_plat_data->gpioInitTable[i].gpioMode == BCM59040_GPIOX_MODE_NORMAL) {
			if(gpio_plat_data->gpioInitTable[i].gpioDir == GPIO_INPUT) {
				bcm59040_gpio_set_direction(pclient, i,
											gpio_plat_data->gpioInitTable[i].gpioDir, 0);
				bcm59040_gpio_read_data(pclient, i,
										&gpio_plat_data->gpioInitTable[i].gpioData);
			}
			else if(gpio_plat_data->gpioInitTable[i].gpioDir == GPIO_OUTPUT) {
				bcm59040_gpio_set_direction(pclient, i,
											gpio_plat_data->gpioInitTable[i].gpioDir,
											gpio_plat_data->gpioInitTable[i].gpioData);
			}
		}
	}

	return 0;
}

static int gpio59040_direction_input(struct gpio_chip *chip,
									 unsigned offset)
{
	bcm59040_gpio_config_t	*gpio_config;

	gpio_config = container_of(chip, bcm59040_gpio_config_t, chip);

	PMU_DEBUG(DBG_TRACE,("GPIO: Set input direction %d\n",offset));
	if(bcm59040_gpio_set_direction(gpio_config->pclient, offset,
								   GPIO_INPUT, 0) < 0)
		return -EINVAL;

	return 0;
} // gpio59040_direction_input

static int gpio59040_direction_output(struct gpio_chip *chip,
									  unsigned offset,
									  int value)
{
	bcm59040_gpio_config_t	*gpio_config;

	gpio_config = container_of(chip, bcm59040_gpio_config_t, chip);

	PMU_DEBUG(DBG_TRACE,("GPIO: Set output direction %d\n",offset));
	if(bcm59040_gpio_set_direction(gpio_config->pclient, offset,
								   GPIO_OUTPUT, value) < 0)
		return -EINVAL;

	return 0;
} // gpio59040_direction_output

static int gpio59040_get(struct gpio_chip *chip,
									  unsigned offset)
{
	int					value;
	bcm59040_gpio_config_t	*gpio_config;

	gpio_config = container_of(chip, bcm59040_gpio_config_t, chip);

	PMU_DEBUG(DBG_TRACE,("GPIO: Set output direction %d\n",offset));
	bcm59040_gpio_read_data(gpio_config->pclient, offset, &value);
	return value;
} // gpio59040_get

static void gpio59040_set(struct gpio_chip *chip,
									  unsigned offset,
									  int value)
{
	bcm59040_gpio_config_t	*gpio_config;

	gpio_config = container_of(chip, bcm59040_gpio_config_t, chip);

	PMU_DEBUG(DBG_TRACE,("GPIO: Set output direction %d\n",offset));
	bcm59040_gpio_write_data(gpio_config->pclient, offset, value);
} // gpio59040_set

static bcm59040_gpio_config_t bcm59040_gpio_chip = {
	.chip = {
		.label				= "BCM59040",
		.direction_input	= gpio59040_direction_input,
		.direction_output	= gpio59040_direction_output,
		.get				= gpio59040_get,
		.set				= gpio59040_set,
		.base				= ARCH_NR_GPIOS-5,
		.ngpio				= BCM59040_NUM_OF_GPIOS,
	}
};

/** bcm59040_gpio_probe - BCM59040 GPIO probe function.
 *
 * Initialize BCM59040 GPIO pins and link into appropriate Linux
 * services for GPIOs.
 *
 * @client:	Pointer to client structure for this device.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 */

static int bcm59040_gpio_probe(struct pmu_client *pclient,
							   const struct pmu_device_id *id)
{
	int							rc=0;

	if((rc=bcm59040_gpio_config(pclient)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "Failed to create BCM59040 gpiochip!\n"));
		return rc;
	}

	bcm59040_gpio_chip.pclient = pclient;
	if((rc=gpiochip_add(&bcm59040_gpio_chip.chip)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "Failed to create BCM59040 gpiochip, error %d!\n", rc));
		return rc;
	}
	pmu_set_drvdata(pclient, &bcm59040_gpio_chip);
	
	printk(KERN_INFO PFX "gpiochip installed for %d:%d.\n", ARCH_NR_GPIOS-5, ARCH_NR_GPIOS-1);

	return 0;
}

/** bcm59040_gpio_remove - BCM59040 control remove
 *
 * Need to remove BCM59040 control driver.
 *
 * @client:	Pointer to client structure for this device.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 * @todo	Need to determine what suspend does for this driver still.
 *
 */

static int bcm59040_gpio_remove(struct pmu_client *pclient)
{
	int ret;

	ret = gpiochip_remove(&bcm59040_gpio_chip.chip);
	if (ret < 0) {
		printk ("Unable to remove gpio chip: %d\n", ret);
	}

	return 0;
}

#ifdef CONFIG_PM

/** bcm59040_gpio_suspend - BCM59040 control module Initialization
 *
 * Core suspend handler that is called when sysem is entering
 * a suspend state.
 *
 * @client:	Pointer to client structure for this device.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 * @todo	Need to determine what suspend does for this driver still.
 *
 */

static int bcm59040_gpio_suspend(struct pmu_client *pclient, pm_message_t state)
{
	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Suspend processed for %s.\n",
						  pclient->name));

	return 0;
} /* bcm59040_gpio_suspend */

/** bcm59040_gpio_resume - BCM59040 control resume
 *
 * Core resume handler that is called to restore BCM59040 control device
 * to state it was in when we entered suspend.
 *
 * @client:	Pointer to client structure for this device.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 * @todo	Need to determine what suspend does for this driver still.
 *
 */

static int bcm59040_gpio_resume(struct pmu_client *pclient)
{
	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Suspend processed for %s.\n",
						  pclient->name ));

	return 0;
} /* bcm59040_gpio_resume */

#endif

struct pmu_device_id bcm59040_gpio_idtable[] = {
	{ "gpio" },
	{ }
};

/** bcm59040_gpio_driver - BCM59040 control driver registration data
 *
 */
struct pmu_driver bcm59040_gpio_driver =
{
	.driver =
	{
		.name	= BCM59040_GPIO_DRIVER,
	},
	.probe		= &bcm59040_gpio_probe,
	.remove		= &bcm59040_gpio_remove,
#ifdef CONFIG_PM
	.suspend    = &bcm59040_gpio_suspend,
	.resume     = &bcm59040_gpio_resume,
#endif
	.id_table	= bcm59040_gpio_idtable,
};

/** bcm59040_gpio_init - BCM59040 control module Initialization
*
* Registers the BCM59040 control driver with the PMU device
* framework.
*
* Returns 0 if successful otherwise standard Linux error codes.
*
*/
static int __init bcm59040_gpio_init(void)
{
	return pmu_register_driver(&bcm59040_gpio_driver);
}

/** bcm59040_gpio_exit - BCM59040 control module Exit
*
* Deregisters the BCM59040 control driver with the PMU device
* framework.
*
* Returns 0 if successful otherwise standard Linux error codes.
*
*/

static void __exit bcm59040_gpio_exit(void)
{
	pmu_unregister_driver(&bcm59040_gpio_driver);
}


device_initcall(bcm59040_gpio_init);
module_exit(bcm59040_gpio_exit);

MODULE_DESCRIPTION(BCM59040_GPIO_MOD_DESCRIPTION);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
MODULE_VERSION(BCM59040_GPIO_MOD_VERSION);
