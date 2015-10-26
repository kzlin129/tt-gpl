/* drivers/barcelona/gprs/gprs.c
 *
 * GPRS driver for Murcia/Knock
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Author: Rogier Stam <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uio.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <asm/uaccess.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <barcelona/gopins.h>
#include <barcelona/gotype.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/string.h>
#include "mc55.h"
#include "adi6720.h"
#include "faro.h"
#include "gprs.h"

static ssize_t gprs_power_show( struct device *dev, struct device_attribute *attr, char *buffer );
static ssize_t gprs_modem_name( struct device *dev, struct device_attribute *attr, char *buffer );
static ssize_t gprs_reset_store( struct device *dev, struct device_attribute *attr, const char *buffer, size_t size );
static ssize_t gprs_power_store( struct device *dev, struct device_attribute *attr, const char *buffer, size_t size );

int gprs_modem_status( struct gprs_modem *device )
{
	return device->stat;
}

/* List with supported modems. */
struct gprs_modem	gprs_modem_list[]=
{
	{GPRS_MODEL_ADI6720, GPRS_STATUS_OFF, adi6720_detect, adi6720_reset, adi6720_off, adi6720_on, gprs_modem_status, adi6720_suspend, adi6720_resume},
	{GPRS_MODEL_FARO, GPRS_STATUS_OFF, faro_detect, faro_reset, faro_off, faro_on, gprs_modem_status,
	 faro_suspend, faro_resume},
	{GPRS_MODEL_MC55, GPRS_STATUS_OFF, siemens_mc55_detect, siemens_mc55_reset, siemens_mc55_off, siemens_mc55_on,
	 gprs_modem_status, siemens_mc55_suspend, siemens_mc55_resume},
};

/* List with 'files' in sysfs and their handlers. Just add one (including handlers) to add it. */
struct device_attribute	gprs_attributes[]=
{
	{{"power_on", THIS_MODULE, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP}, gprs_power_show, gprs_power_store},
	{{"reset", THIS_MODULE, S_IWUSR | S_IWGRP}, NULL, gprs_reset_store}, 
	{{"name", THIS_MODULE, S_IRUSR | S_IRGRP}, gprs_modem_name, NULL}, 
};

static ssize_t gprs_power_show( struct device *dev, struct device_attribute *attr, char *buffer )
{
	struct gprs_modem	*device=(struct gprs_modem *) dev_get_drvdata( dev );

	/* Power. Should match the entry power in the gprs attributes structure. */
	snprintf( buffer, PAGE_SIZE, "%i", device->status( device ) );

	return strlen( buffer );
}

static ssize_t gprs_modem_name( struct device *dev, struct device_attribute *attr, char *buffer )
{
	struct gprs_modem	*device=(struct gprs_modem *) dev_get_drvdata( dev );

	snprintf( buffer, PAGE_SIZE, "%s\n", device->name );

	return strlen( buffer );
}

static ssize_t gprs_reset_store( struct device *dev, struct device_attribute *attr, const char *buffer, size_t size )
{
	struct gprs_modem	*device=(struct gprs_modem *) dev_get_drvdata( dev );

	/* Ensure the data written is valid. */
	if( buffer[0] != '1' )
		return -EINVAL;

	switch( size )
	{
		case 1 :
			break;

		case 2 :
			if( buffer[1] != '\n' )
				return -EINVAL;
			break;

		default :
			return -EINVAL;
	}

	/* Reset. Should match  the entry reset in the gprs attributes structure. */
	return (device->reset( device ) != 0 ? -EINVAL : 1);
}

static ssize_t gprs_power_store( struct device *dev, struct device_attribute *attr, const char *buffer, size_t size )
{
	struct gprs_modem	*device=(struct gprs_modem *) dev_get_drvdata( dev );

	/* Ensure the data written is valid. */
	if( (buffer[0] != '1') && (buffer[0] != '0') )
		return -EINVAL;

	switch( size )
	{
		case 1 :
			break;

		case 2 :
			if( buffer[1] != '\n' )
				return -EINVAL;
			break;

		default :
			return -EINVAL;
	}

	/* Power. Should match the entry power in the gprs attributes structure. */
	if( buffer[0] == '1' )
		return (device->on( device ) != 0 ? -EINVAL : 1);
	else
		return (device->off( device ) != 0 ? -EINVAL : 1);
}
			
static int gprs_probe( struct device *dev )
{
	int			index=0;
	struct gprs_modem	*device=NULL;

	/* Run the detect for all GPRS modems in the list. First one to be detected is going to be used. */
	for( index=0; index < GPRS_MODEM_ENTRIES; index++ )
	{
		if( gprs_modem_list[index].detect( &(gprs_modem_list[index]) ) != 0 )
			break;
	}

	if( index == GPRS_MODEM_ENTRIES )
		return -ENODEV;
	else device=&(gprs_modem_list[index]);

	/* We're registered. Initialize. */
	printk( "GPRS Modem driver (\"%s\")\n", device->name ); 
	dev_set_drvdata( dev, device );

	/* Register each 'file' in sysfs seperately. */
	for( index=0; index < (sizeof( gprs_attributes )/sizeof( gprs_attributes[0] )); index++ )
	{
		if( device_create_file( dev, &(gprs_attributes[index]) ) != 0 )
		{
			for( index--; index >=0; index-- )
				device_remove_file( dev, &(gprs_attributes[index]) );
			return -ENOMEM;
		}
	}

	/* Done. */ 
	return 0;
}

static int gprs_remove( struct device *dev )
{
	int			index;

	/* Make sure the device exists. */
	if( dev == NULL )
	{
		printk( KERN_ERR MODPFX "Can't get device data for device removal!!!\n" );
		return -ENODEV;
	}

	/* Remove each sysfs 'file'. */
	for( index=0; index < (sizeof( gprs_attributes )/sizeof( gprs_attributes[0] )); index++ )
		device_remove_file( dev, &(gprs_attributes[index]) );

	return 0;
}

static void gprs_shutdown( struct device *dev )
{
	gprs_remove( dev );
	return;
}

static int gprs_suspend(struct device *dev, u32 state, u32 level)
{
	struct gprs_modem	*device=dev_get_drvdata( dev );
	int			rc=0;

	if( level == SUSPEND_POWER_DOWN )
		rc=device->suspend( device );

	return rc;
}

static int gprs_resume(struct device *dev, u32 level)
{
	struct gprs_modem	*device=dev_get_drvdata( dev );
	int			rc=0;

	if( level == RESUME_POWER_ON )
		rc=device->resume( device );

	return rc;
}

static struct device_driver gprs_driver = {
	.owner		= THIS_MODULE,
	.name		= "gprs-interface",
	.bus		= &platform_bus_type,
	.probe		= gprs_probe,
	.remove		= gprs_remove,
	.shutdown	= gprs_shutdown,
	.suspend	= gprs_suspend,
	.resume		= gprs_resume,
};

static int __init gprs_init( void )
{
        /* Signon message. */
        return driver_register( &gprs_driver );
}

static void __exit gprs_exit( void )
{
        driver_unregister( &gprs_driver );
}

module_init( gprs_init );
module_exit( gprs_exit );

MODULE_DESCRIPTION( "GPRS driver for Knock and Murcia" );
MODULE_LICENSE( "GPL" );
MODULE_AUTHOR( "Rogier Stam <rogier.stam@tomtom.com>" );
