/*
 * drivers/video/samsung/lms480wv.c
 *
 * Copyright (C) 2008 Jeroen Taverne <jeroen.taverne@samsung.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 */


#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/module.h>

#include <linux/cordoba_lms480wv.h>

#define BUF_LEN 2

static void lms480wv_spi_config( struct spi_device *spi )
{
	int len=0;
	int i,j=0;
	unsigned char write_buf[BUF_LEN];
	unsigned char data[] = {0x00, 0x63,0x01, 0x55,0x02, 0x00,0x03, 0x0B,0x04, 0x00,0x05, 0x11,0x06, 0x00,
				0x07, 0x00,0x08, 0x40,0x09, 0x01,0x0A, 0x20,0x0B, 0x29,0x0C, 0x10,0x0D, 0x30,
				0x0E, 0x20,0x0F, 0x20,0x10, 0xCA,0x11, 0xCA,0x12, 0x0B,0x13, 0x20,0x14, 0x20,
				0x15, 0x20,0x16, 0x80,0x17, 0x80,0x18, 0x80,0x20, 0x00,0x21, 0x03,0x22, 0xEF,
				0x23, 0x2D,0x24, 0x6B,0x25, 0xA1,0x26, 0xD8,0x27, 0x6D,0x28, 0xEB,0x29, 0x51,
				0x2A, 0x97,0x2B, 0xFF,0x2C, 0x40,0x2D, 0x95,0x2E, 0xFE,0x50, 0x00,0x51, 0x03,
				0x52, 0xEF,0x53, 0x2D,0x54, 0x6B,0x55, 0xA1,0x56, 0xD8,0x57, 0x6D,0x58, 0xEB,
				0x59, 0x51,0x5A, 0x97,0x5B, 0xFF,0x5C, 0x40,0x5D, 0x95,0x5E, 0xFE,0x2F, 0x21};

	len = sizeof(data);

	for (i=0; i<(len/2); i++){
		write_buf[0] = data[j++];
		write_buf[1] = data[j++];
  		spi_write(spi, write_buf, BUF_LEN);
	}
}


static int lms480wv_spi_probe( struct spi_device *spi )
{
	int err=0;
	struct lms480wv_platform_data 	*pdata=spi->dev.platform_data;
	
	/* Signon. */
	printk(KERN_INFO"TomTom GO lms480wv SPI Driver, (c) 2008 TomTom B.V\n" );
	
	err =	pdata->request_gpio();
	if(err) printk(KERN_ERR"lms480wv err: %d\n",err);
	pdata->config_gpio();
	pdata->enable_lcm();
	pdata->reset_lcm();
	/*pdata->simple_backlight_on();*/

	lms480wv_spi_config( spi );

	pdata->free_gpio();
	return err;
}

static int lms480wv_spi_remove( struct spi_device *spi )
{
	return 0;
}

static int lms480wv_suspend( struct spi_device *spi, pm_message_t msg)
{
	return 0;
}

static int lms480wv_resume( struct spi_device *spi )
{
	lms480wv_spi_config( spi );
	return 0;
}

static struct spi_driver	lms480wv_spi_driver=
{
	.driver=
	{
		.name		= "lms480wv_spi",
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
	},
	.probe			= lms480wv_spi_probe,
	.remove			= lms480wv_spi_remove,
	.suspend		= lms480wv_suspend,
	.resume			= lms480wv_resume,
};

static int __init lms480wv_spi_init( void )
{
	return spi_register_driver( &lms480wv_spi_driver );
}

static void __exit lms480wv_spi_exit( void )
{
	spi_unregister_driver( &lms480wv_spi_driver );
	return;
}

module_init( lms480wv_spi_init );
module_exit( lms480wv_spi_exit );

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("lms480wv SPI configuration driver");
