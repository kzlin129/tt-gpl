/* drivers/barcelona/gprs/faro/faro.c
 *
 * GPRS driver for Foxlink FARO
 *
 * Copyright (C) 2008 TomTom BV <http://www.tomtom.com/>
 * Author: Niels Langendorff <niels.langendorff@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


/* Normal kernel includes */
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <asm/gpio.h>
#include <plat/gprs.h>
#include <plat/gprs_types.h>

/* Defines */
#define PFX "faro:"
#define PK_DBG PK_DBG_FUNC
#define FARO_DEVNAME	"faro"
#define DRIVER_DESC_LONG "TomTom GPRS Driver for Foxlink FARO, (C) 2008 TomTom BV "


typedef struct
{
	gprs_state_e			state;
} faro_data_t;


static faro_data_t *faro_data = 0;

int faro_reset(struct gprs_data *data)
{
	faro_data_t * faro_data;

	BUG_ON(!data);
	faro_data = gprs_get_drvdata(data);

	if (faro_data->state == GPRS_STATUS_ON) {
		data->pd->gprs_power( 1 );
		data->pd->gprs_reset( 0 );
		msleep(1);
		data->pd->gprs_reset( 1 );
		msleep(100);
		data->pd->gprs_reset( 0 );
		msleep(1000);
		data->pd->gprs_power( 0 );

		printk(KERN_INFO PFX " reset\n");
		return 0;
	} else
		return -1;
}

int faro_off(struct gprs_data *data)
{
	faro_data_t * faro_data;

	BUG_ON(!data);
	faro_data = gprs_get_drvdata(data);
	BUG_ON(!faro_data);

	if (faro_data->state == GPRS_STATUS_ON) {
		data->pd->gprs_power( 0 );
		data->pd->gprs_reset( 0 );
		msleep(1);
		data->pd->gprs_reset( 1 );
		msleep(100);
		data->pd->gprs_reset( 0 );

		printk(KERN_INFO PFX " switched off\n");

		faro_data->state = GPRS_STATUS_OFF;
		return 0;
	} else
		return -1;
}

int faro_on(struct gprs_data *data)
{
	faro_data_t * faro_data;
	BUG_ON(!data);
	faro_data = gprs_get_drvdata(data);
	BUG_ON(!faro_data);

	/* Set faro_data->stat=ON first, otherwise /sys/.../power_on will return 0 until
	   this function completes. If we take a long time to return, kernel will
	   report modem as off when we're in the process of turning on.
	*/
	faro_data->state = GPRS_STATUS_ON;

	/* First thing is to do whatever we have to setup the port the right way */
	if (data->pd->gprs_setup_port)
		data->pd->gprs_setup_port();

	/* start with a "power off reset", otherwise Faro 1.1 and 2.0 might
	   start outputting debug information. */
	data->pd->gprs_power( 0 );
	data->pd->gprs_reset( 0 );
	msleep(1);
	data->pd->gprs_reset( 1 );
	msleep(100);
	data->pd->gprs_reset( 0 );
	msleep(2000);

	/* power on sequence */
	data->pd->gprs_power( 1 );
	msleep(2000);
	data->pd->gprs_power( 0 );
	msleep(1200);

	printk(KERN_INFO PFX " switched on\n");

	return 0;
}

#ifdef CONFIG_PM

int faro_suspend(struct gprs_data *data)
{
	faro_data_t * faro_data;

	BUG_ON(!data);
	faro_data = gprs_get_drvdata(data);
	BUG_ON(!faro_data);
	
	if (faro_data->state == GPRS_STATUS_ON)
		faro_off(data);

	if (data->pd->gprs_suspend)
		data->pd->gprs_suspend();

	return 0;
}

#endif

/* probe method only to be called from gprs driver */
static int faro_probe(struct gprs_data *data)
{
	printk(KERN_INFO PFX " probed\n");

	if (!
	    (faro_data =
	     kmalloc(sizeof(faro_data_t), GFP_KERNEL))) {
		printk(KERN_ERR PFX
		       " failed to allocate memory for the faro_data struct\n");
		return -ENOMEM;
	}

	memset(faro_data, 0x00, sizeof(faro_data_t));
	faro_data->state		= GPRS_STATUS_OFF;

	gprs_set_drvdata(data, faro_data);

	return 0;
}

/* remove method only to be called from gprs driver */
static int faro_remove(struct gprs_data *data)
{
	faro_data_t * faro_data;

	BUG_ON(!data);
	faro_data = gprs_get_drvdata(data);
	BUG_ON(!faro_data);

	kfree(faro_data);
	printk(KERN_INFO PFX " removed\n");
		
	return 0;
}

static gprs_state_e faro_state(struct gprs_data * data)
{
	faro_data_t * faro_data;
	BUG_ON(!data);
	faro_data = gprs_get_drvdata(data);
	BUG_ON(!faro_data);

	return faro_data->state;
}


struct gprs_operations faro_gprs_ops = 
{
	.gprs_reset			= faro_reset,
	.gprs_on			= faro_on,
	.gprs_off			= faro_off,
	.gprs_state			= faro_state,
	.gprs_probe			= faro_probe,
	.gprs_remove		= faro_remove,
#ifdef CONFIG_PM
	.gprs_suspend		= faro_suspend,
#endif
};

static int __init faro_init_module(void)
{
	int ret;

	/* display driver information when the driver is started */
	printk(KERN_INFO DRIVER_DESC_LONG "(" PFX ")\n");

	ret = register_gprs_driver(&faro_gprs_ops, FARO_DEVNAME, GOGPRS_FARO);
	if (ret) {
		printk(KERN_ERR PFX " unable to register gprs driver (%d)\n", ret);
		return ret;
	}

	return 0;
};


static void __exit faro_exit_module(void)
{
	unregister_gprs_driver();
};

module_init(faro_init_module);
module_exit(faro_exit_module);

MODULE_DESCRIPTION(DRIVER_DESC_LONG);
MODULE_AUTHOR ("Niels Langendorff "    "(niels.langendorff@tomtom.com)");
MODULE_LICENSE("GPL");
