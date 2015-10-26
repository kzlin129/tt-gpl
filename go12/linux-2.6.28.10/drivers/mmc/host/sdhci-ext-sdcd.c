/* drivers/mmc/host/sdhci-ext-sdcd.c

Copyright (C) 2008 TomTom BV <http://www.tomtom.com/>
Author:	Rogier Stam <rogier.stam@tomtom.com>

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 as
published by the Free Software Foundation.

*/

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include "sdhci.h"

static int mmc_sd_cd_set_cd( struct platform_device *pdev )
{
	struct resource		*card_detect_gpio;
	struct resource		*sdhci_controller;

	card_detect_gpio=platform_get_resource( pdev, IORESOURCE_IO, 0 );
	sdhci_controller=platform_get_resource( pdev, IORESOURCE_MEM, 0 );
	if( (card_detect_gpio != NULL) && (sdhci_controller != NULL) )
	{
		/* Set test mode (e.g force card detect), and read the gpio to determine the state. */
		if( !gpio_get_value( card_detect_gpio->start ) )
			writeb( readb( sdhci_controller->start + SDHCI_HOST_CONTROL ) | SDHCI_CTRL_CD_TESTLVL | SDHCI_CTRL_CD_SIGDET,
				sdhci_controller->start + SDHCI_HOST_CONTROL );
		else
			writeb( (readb( sdhci_controller->start + SDHCI_HOST_CONTROL ) & ~(SDHCI_CTRL_CD_TESTLVL)) | SDHCI_CTRL_CD_SIGDET,
				sdhci_controller->start + SDHCI_HOST_CONTROL );

		/* Force a vendor specific interrupt. This replaces the card insertion/removal event. */
		writel( SDHCI_FINT_CEATAERR_SIG, sdhci_controller->start + SDHCI_FORCE_EVENT_INT_STATUS );
		return 0;
	}

	
	return -1;
}

static irqreturn_t mmc_sd_cd_irq(int irq, void *data)
{
	struct platform_device	*pdev=(struct platform_device *) data;

	mmc_sd_cd_set_cd( pdev );
	return IRQ_HANDLED;
}

static int mmc_sd_cd_probe( struct platform_device *pdev )
{
	struct resource		*card_detect_gpio;
	struct resource		*sdhci_controller;

	printk( "MMC SD Card Detect workaround - v1.0\n" );
	card_detect_gpio=platform_get_resource( pdev, IORESOURCE_IO, 0 );
	sdhci_controller=platform_get_resource( pdev, IORESOURCE_MEM, 0 );
	if( (card_detect_gpio != NULL) && (sdhci_controller != NULL) )
	{
		/* Set the right state. */
		gpio_direction_input( card_detect_gpio->start );
		mmc_sd_cd_set_cd( pdev );

		/* Now request an interrupt to deal with this */
		if( request_irq( gpio_to_irq( card_detect_gpio->start ), mmc_sd_cd_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, pdev->name, pdev ) )
			return -ENXIO;

		/* Done. */
		return 0;
	}
	return -ENODEV;
}

static int mmc_sd_cd_remove( struct platform_device *pdev )
{
	struct resource		*card_detect_gpio;
	
	card_detect_gpio=platform_get_resource( pdev, IORESOURCE_IO, 0 );
	if( card_detect_gpio != NULL )
		free_irq( gpio_to_irq( card_detect_gpio->start ), pdev );
	return 0;
}

#ifdef CONFIG_SUSPEND
static int mmc_sd_cd_suspend(struct platform_device *pdev, pm_message_t msg)
{
	/* Do nothing. Only resume handles. */
	return 0;
}

static int mmc_sd_cd_resume(struct platform_device *pdev)
{
	return mmc_sd_cd_set_cd( pdev );
}
#else
#define mmc_sd_cd_suspend	NULL
#define mmc_sd_cd_resume	NULL
#endif

static struct platform_driver mmc_sd_cd_driver =
{
	.probe   = mmc_sd_cd_probe,
	.remove  = mmc_sd_cd_remove,
	.suspend = mmc_sd_cd_suspend,
	.resume  = mmc_sd_cd_resume,
	.driver  =
	{
		.owner = THIS_MODULE,
		.name = "mmc_sd_cd",
	},
};

static int __init mmc_sd_cd_init(void)
{
	if( platform_driver_register( &mmc_sd_cd_driver ) )
	{
		printk( KERN_ERR "Unable to register external GPIO SDHCI Card Detect driver!\n" );
		return -ENODEV;
	}
	return 0;
}

static void __exit mmc_sd_cd_exit(void)
{
	platform_driver_unregister( &mmc_sd_cd_driver );
	return;
}

late_initcall( mmc_sd_cd_init );
module_exit( mmc_sd_cd_exit );

MODULE_AUTHOR( "Rogier Stam (rogier.stam@tomtom.com)" );
MODULE_LICENSE("GPL");
