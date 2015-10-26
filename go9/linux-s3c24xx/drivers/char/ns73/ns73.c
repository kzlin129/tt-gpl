/* ns73.c
 *
 * Control driver for Niigata Seimitsu NS73 chip.
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Authors: Rogier Stam <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/ns73.h>
#include <barcelona/gopins.h>
#include <linux/delay.h>
#include <asm-arm/arch-s3c2410/hardware.h>

#define BARCELONA_DEBUG	__FILE__
#define PFX "NS73: "
#define PK_DBG PK_DBG_FUNC
#include <barcelona/debug.h>

struct ns73dev {
	struct cdev cdev;
	struct i2c_client* client;
};

/* Forward declarations */
static struct i2c_client client_template;

/* Magic definition of all other variables and things */
static unsigned short normal_i2c[] = { NS73_I2C_SLAVE_ADDR, I2C_CLIENT_END };
I2C_CLIENT_INSMOD;

static int ns73_attach(struct i2c_adapter* adapter);
static int ns73_detach(struct i2c_client* client);
static int ns73_i2c_probe(struct device *dev);
static int ns73_powerdown( struct i2c_client *c);
static int ns73_SetFreq( struct i2c_client *c, unsigned int u32Freq);
static int ns73_i2c_remove( struct device *dev );

/* If the dock device is present, it'll unregister the platform devices before suspending, */
/* causing this driver to have problems upon resume. Therefore, let the unregister handle  */
/* the resume action. */
#if !defined( CONFIG_BARCELONA_DOCK ) && defined( CONFIG_PM )
static int ns73_i2c_suspend(struct device * dev, pm_message_t state, u32 level);
static int ns73_i2c_resume(struct device * dev, u32 level);
#else
#define ns73_i2c_suspend	(NULL)
#define ns73_i2c_resume 	(NULL)
#endif /* !defined( CONFIG_BARCELONA_DOCK ) && defined( CONFIG_PM ) */
static int ns73_powersuspend( struct i2c_client *client );
static int ns73_powerresume( struct i2c_client *client );
static dev_t fmX_devno;

static struct i2c_driver driver = {
	.id			= I2C_DRIVERID_NS73,
	.owner			= THIS_MODULE,
	.name			= "ns73_i2c",
	.flags			= I2C_DF_NOTIFY,
	.attach_adapter		= ns73_attach,
	.detach_client		= ns73_detach,
	.driver = {
		.name		= "ns73_i2c",
	},
};

struct device_driver ns73_driver = {
	.name			= NS73_DEVNAME,
	.probe			= ns73_i2c_probe,
	.suspend		= ns73_i2c_suspend,
	.remove			= ns73_i2c_remove,
	.resume			= ns73_i2c_resume,
	.owner			= THIS_MODULE,
	.bus			= &platform_bus_type,
};

static struct i2c_client client_template = {
        .name 			= "ns73_i2c_client",
        .flags			= I2C_CLIENT_ALLOW_USE,
        .usage_count		= 0,
        .driver			= &driver,
        .addr			= NS73_I2C_SLAVE_ADDR,
};

static int
ns73_startup( struct i2c_client *c )
{
	unsigned char	soft_reset[]	= { 0x0E, 0x05 };
	unsigned char	init_array[][2]	= {
				{ 0x01, 0xB4 },		/* Pilot on, Forced Subcarrier off */
				{ 0x02, 0x03 },		/* Unlock Detect Off, TX power=2mW */
				{ 0x05, 0x00 },		/* Reserved. */
				{ 0x06, 0x1A },		/* CIB=320uA, CIA=1.25uA */
				{ 0x07, 0x00 },		/* Reserved. */
				{ 0x09, 0x00 },		/* Reserved. */
				{ 0x0A, 0x00 },		/* Reserved. */
				{ 0x0B, 0x00 },		/* Reserved. */
				{ 0x0D, 0x00 } };	/* Reserved. */
	int		count=0;
	int		rc=0;

	/* Reset the NS73. */
	rc=i2c_smbus_write_byte_data( c, soft_reset[0], soft_reset[1] );
	if( rc < 0 ) return rc;

	/* Bring the NS73 in a default state. */
	rc=ns73_powerdown( c );
	if( rc < 0 ) return rc;

	for( count=0; count < (sizeof( init_array )/sizeof( init_array[0] )); count++ )
	{
		rc=i2c_smbus_write_byte_data( c, init_array[count][0], init_array[count][1] );
		if( rc < 0 ) return rc;
	}

	mdelay( 3 );

	/* Since the USB dock for valencia/murcia pulls down the SDA pin to ground, the data received will */
	/* be all 0. This means that whatever we wrote before can't match to what we read back now if the  */
	/* usb dock is connected. */
	for( count=0; count < (sizeof( init_array )/sizeof( init_array[0] )); count++ )
	{
		rc=i2c_smbus_read_byte_data( c, init_array[count][0] );
		if( rc < 0 ) return rc;
		if( rc != 0 ) break;
	}

	mdelay( 3 );
	if( count != (sizeof( init_array )/sizeof( init_array[0] )) )
		return 0;
	else
		return -1;
}
		
static int
ns73_powerdown( struct i2c_client *c)
{
	unsigned char	pwroff[] = { 0x00, 0x84 };
	return i2c_smbus_write_byte_data( c, pwroff[0], pwroff[1] );
}

static int
ns73_SetFreq( struct i2c_client *c, unsigned int u32Freq)
{
	unsigned char	pwroff[] = { 0x00, 0x84 };
	unsigned char	setfreq[][2] = { {0x03, 0x00}, {0x04, 0x00} };;
	unsigned char	cexband[][2] = { {0x08, 0x18}, {0x08, 0x19}, {0x08, 0x1A}, {0x08, 0x1B}};
	unsigned int	RegFreq=0;
	int		rc = 0;
	int		cexidx=0;

	if( (u32Freq < 87000000) || (u32Freq > 108000000) )
	{
		rc=i2c_smbus_write_byte_data( c, pwroff[0], pwroff[1] );
		if( rc < 0 ) return rc;
	}
	else
	{
		RegFreq=(u32Freq + 304000 + 4096)/8192;
		setfreq[0][1]=((unsigned char) ((RegFreq & 0x000000FF) >> 0));
		setfreq[1][1]=((unsigned char) ((RegFreq & 0x0000FF00) >> 8));
		rc=i2c_smbus_write_byte_data( c, setfreq[0][0], setfreq[0][1] );
		if( rc < 0 ) return rc;
		rc=i2c_smbus_write_byte_data( c, setfreq[1][0], setfreq[1][1] );
		if( rc < 0 ) return rc;

		if( u32Freq <= 90500000 )
			cexidx=3;
		else if( (u32Freq > 90500000) && (u32Freq <= 96000000) )
			cexidx=2;
		else if( (u32Freq > 96000000) && (u32Freq <= 102000000) )
			cexidx=1;
		else if( (u32Freq > 102000000) && (u32Freq <= 108000000) )
			cexidx=0;

		rc=i2c_smbus_write_byte_data( c, cexband[cexidx][0], cexband[cexidx][1] );
		if( rc < 0 ) return rc;
	}
	return rc;
}

/* The suspend array contains default working values to ensure the device can work if someone uses an ioctl to enable it. */
unsigned char ns73_suspend_array[][2]=
{{0x01, 0xB4}, {0x02, 0x03}, {0x03, 0x31}, {0x04, 0x00},
 {0x05, 0x00}, {0x06, 0x1A}, {0x07, 0x00}, {0x08, 0x18},
 {0x09, 0x00}, {0x0A, 0x00}, {0x0B, 0x00}, {0x0D, 0x00},
 {0x00, 0x85}, {0xFF, 0xFF}};

static int ns73_powersuspend( struct i2c_client *client )
{
	int	rc=0;
	int	index=0;

	for( index=0; ns73_suspend_array[index][0] != 0xFF; index++ )
	{
		rc=i2c_smbus_read_byte_data( client, ns73_suspend_array[index][0] );
		if( rc < 0 ) return rc;
		ns73_suspend_array[index][1]=(unsigned char) rc;
	}
	return ns73_powerdown( client );
}

static int ns73_powerresume( struct i2c_client *client )
{
	int	rc=0;
	int	index=0;

	/* Initialize to default state. */
	rc=ns73_startup( client );
	if( rc ) return rc;

	/* Restore settings. */
	for( index=0; ns73_suspend_array[index][0] != 0xFF; index++ )
	{
		rc=i2c_smbus_write_byte_data( client, ns73_suspend_array[index][0], ns73_suspend_array[index][1] );
		if( rc < 0 ) return rc;
	}
	return rc;
}

static int
ns73_i2c_remove( struct device *dev )
{
	return i2c_del_driver( &driver );
}

static int
ns73_i2c_probe(struct device *dev)
{	
	struct fm_transmitter_info	*pdata = dev->platform_data;

	fmX_devno = pdata->device_nr;

	return i2c_add_driver(&driver);
}

#if !defined( CONFIG_BARCELONA_DOCK ) && defined( CONFIG_PM )
static int
ns73_i2c_suspend(struct device * dev, pm_message_t state, u32 level)
{
	struct i2c_client *c = NULL;
	int rc = 0;

	c=i2c_get_client(I2C_DRIVERID_NS73, 0, NULL);
	if( c != NULL )
	{
		if (level == SUSPEND_POWER_DOWN)
		{
			rc = ns73_powersuspend( c );
			if( rc < 0 )
				printk( KERN_ERR PFX"Error while doing powersuspend: rc = %d\n", rc );
		}
	}

	/* Never return an error here. It will block the kernel from going into suspend further
	 * and the FM transmitter is not important enough to block that!
	 */
	return 0;
}

static int
ns73_i2c_resume(struct device * dev, u32 level)
{
	struct i2c_client *c=NULL;
	int rc = 0;

	c=i2c_get_client(I2C_DRIVERID_NS73, 0, NULL);
	if( c != NULL )
	{
		if (level == RESUME_POWER_ON)
			rc = ns73_powerresume( c);
	}

	return (rc < 0 ? rc : 0);
}
#endif /* !defined( CONFIG_BARCELONA_DOCK ) && defined( CONFIG_PM ) */

static int
ns73_open(struct inode* nodep, struct file* filep)
{
	struct ns73dev *dev = container_of(nodep->i_cdev, struct ns73dev, cdev);
	filep->private_data = dev;
	return 0;
}

static int
ns73_release(struct inode* nodep, struct file* filep)
{
	if (filep && filep->private_data) {
		filep->private_data = NULL;
	}
	return 0;
}

static int
ns73_ioctl(struct inode* nodep, struct file* filep, unsigned int cmd, unsigned long arg)
{
	struct ns73dev	*dev = (filep ? filep->private_data : NULL);
	unsigned char	stereo_on[] 	= { 0x01, 0xB4 };
	unsigned char	stereo_off[] 	= { 0x01, 0xBC };
	unsigned char	read_power[]	= { 0x00, 0x00 };
	unsigned char	power_off[]	= { 0x00, 0x84 };
	unsigned char	power_on[]	= { 0x00, 0x81 };

	int rc = -ENOTTY;

	if (_IOC_TYPE(cmd) != FMTRANSMITTER_DRIVER_MAGIC)
	{
		PK_ERR("Wrong IOC type! Failing command");
		return -ENOTTY;
	}

	switch( cmd)
	{
		case IOW_SET_FM_FREQUENCY:
		{
			unsigned int u32Freq = (unsigned int) arg;
			rc = ns73_SetFreq( dev->client, u32Freq);
   		}
		break;

		case IOW_ENABLE:
		{
			unsigned char bEnable = (unsigned char) arg;
			if( bEnable)
			{
				rc = ns73_powerresume( dev->client );
				if( rc < 0 ) return rc;
				rc=i2c_smbus_write_byte_data( dev->client, power_on[0], power_on[1] );
			}
			else
			{
				rc = ns73_powersuspend( dev->client );
				if( rc < 0 ) return rc;
				rc=i2c_smbus_write_byte_data( dev->client, power_off[0], power_off[1] );
			}
			break;
		}

		case IOW_FMTRX_SET_MONO_STEREO:
		{
			unsigned char bEnable = (unsigned char) arg;
			if( bEnable )
				rc=i2c_smbus_write_byte_data( dev->client, stereo_on[0], stereo_on[1] );
			else
				rc=i2c_smbus_write_byte_data( dev->client, stereo_off[0], stereo_off[1] );
			break;
		}

		case IOR_FM_GET_STATE:
		{
			/* Get the state. */
			rc=i2c_smbus_read_byte_data( dev->client, read_power[0] );
			if( rc < 0 ) return rc;
			read_power[1]=(unsigned char) rc;

			/* Check the power bit state. */
			if( read_power[1] & 0x01 )
				*((int *) arg)=fmtrx_on;
			else
				*((int *) arg)=fmtrx_off;
			break;
		}
		
		default:
		{
			break;
		}
	}
	return (rc < 0 ? rc : 9);
}

static unsigned int
ns73_poll(struct file *filep, struct poll_table_struct *pq)
{
	// FM transmitter can neither be written nor read, 
	// the poll() routine should always indicate 
	// that it is neither possible to read nor to write to the driver.
	return 0;
}

struct file_operations ns73_fops = {
        .owner		= THIS_MODULE,
        .open		= ns73_open,
        .release	= ns73_release,
        .ioctl		= ns73_ioctl,
        .poll		= ns73_poll,
};


static int
ns73_detect_client(struct i2c_adapter *adapter, int address, int kind)
{
	struct ns73dev *dev;
	struct i2c_client *c;
	int rc = 0;

	PK_DBG("----------------------%s-------------------\n", __func__);
	c = kmalloc(sizeof *c, GFP_KERNEL);
	if (!c) {
		rc = -ENOMEM;
		goto no_client;
	}
	memcpy(c, &client_template, sizeof *c);
	c->adapter = adapter;
	strcpy(c->name, NS73_DEVNAME);

	dev = kmalloc(sizeof *dev, GFP_KERNEL);
	if (!dev) {
		rc = -ENOMEM;
		goto no_dev;
	}
	memset(dev, 0, sizeof *dev);
	i2c_set_clientdata(c, dev);

	i2c_attach_client(c);

	dev->client = c;
	
	/* Now setup our character device to the fm */
	cdev_init( &dev->cdev, &ns73_fops );
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &ns73_fops;

	rc=ns73_startup( c );
	if( rc < 0 ) goto no_cdev;

	rc=ns73_powerdown( c );
	if( rc < 0 ) goto no_cdev;

	if( (rc = cdev_add(&dev->cdev, fmX_devno, 1) ) )
		goto no_cdev;
	printk( "Niigata Seimitsu NS73 FM Transmitter v1.0 driver loaded\n" );
	return 0;

no_cdev:
	i2c_detach_client(c);
no_dev:
	kfree(c);
no_client:
	PK_DBG("%s: ERROR %d!\n", __func__, rc);

	return rc;

}

/* ----------------------------------------------------------------------- */

static int
ns73_attach(struct i2c_adapter *adap)
{
	int ret= 0;

	ret=i2c_probe( adap, &addr_data, ns73_detect_client);
	udelay( 100 );
	
	return ret;
}

static int
ns73_detach(struct i2c_client *c)
{
	struct ns73dev* fm = i2c_get_clientdata(c);
	
	i2c_detach_client(c);

	if( fm != NULL )
	{
		printk( "Niigata Seimitsu NS73 FM Transmitter v1.0 driver unloaded\n" );
		cdev_del( &fm->cdev );
		
		kfree(fm);
		kfree(c);
	}

	return 0;
}


/* ----------------------------------------------------------------------- */

static int __init 
ns73_init(void)
{
	return driver_register(&ns73_driver);
}

static void __exit
ns73_exit(void)
{
	driver_unregister(&ns73_driver);
}

MODULE_AUTHOR("Rogier Stam <rogier.stam@tomtom.com>");
MODULE_DESCRIPTION("Driver for I2C connected Niigata Seimitsu NS73 FM Transmitter");

module_init(ns73_init);
module_exit(ns73_exit);

