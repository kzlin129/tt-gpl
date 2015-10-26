/* lms606kf01.c
 *
 * SPI protocol driver for LMS606KF01 LCM
 *
 * Copyright (C) 2012 TomTom BV <http://www.tomtom.com/>
 * Authors: Will Lin <Will.Lin@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/firmware.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/vgpio.h>

#include <plat/irvine.h>
#include <plat/lcd.h>

#include <asm/uaccess.h>

#include <mach/pinmux.h>

#define LMS606KF01_DEVNAME "lcm-lms606kf01"

#define LMS606KF01_SET(x) \
do{\
	lms606kf01_write(spi, x[0], &x[1], sizeof(x)/sizeof(x[0])-1);\
}while(0)


static u16 password[] = 
{ 
	0xF0, 0x5A, 0x5A
};

static u16 vcom[] = 
{ 
	0xF4, 0x30, 0x3F, 0x5E
};

static u16 smps_posi[] = 
{ 
	0xBD, 0x06, 0x0B, 0x7E, 0x03,
	0x12, 0x37
};

static u16 smps_nega[] = 
{ 
	0xBE, 0x06, 0x0B, 0x7E, 0x03,
	0x15, 0x37
};

static u16 smps[] = 
{ 
	0xBF, 0x02, 0x0F
};

static u16 power_control[] = 
{ 
	0xF3, 0x10, 0xA9, 0x00, 0x01, 
	0x44, 0xA4, 0x83, 0x83, 0x00,
	0x50
};

static u16 source_control[] = 
{ 
	0xF5, 0x03, 0x09, 0x80, 0x01,
	0x01, 0x00, 0x00, 0x05, 0x03
};

static u16 positive_r_gamma[] = 
{ 
	0xE7, 0x05, 0x35, 0x1E, 0x1C,
	0x1B, 0x1D, 0x18, 0x1F, 0x1F,
	0x24, 0x2C, 0x36, 0x36, 0x37,
	0x28, 0x16, 0x00
};

static u16 positive_g_gamma[] = 
{ 
	0xE8, 0x05, 0x35, 0x1E, 0x1C,
	0x1B, 0x1D, 0x18, 0x22, 0x1F,
	0x27, 0x2E, 0x39, 0x3C, 0x2D,
	0x23, 0x08, 0x0C
};

static u16 positive_b_gamma[] = 
{ 
	0xE9, 0x05, 0x35, 0x1E, 0x1C,
	0x1B, 0x1D, 0x18, 0x1F, 0x21,
	0x28, 0x2E, 0x3A, 0x34, 0x2D,
	0x28, 0x0A, 0x00
};

static u16 negative_r_gamma[] = 
{ 
	0xEA, 0x05, 0x23, 0x0A, 0x0A,
	0x07, 0x02, 0x04, 0x09, 0x0A,
	0x0E, 0x15, 0x10, 0x0E, 0x12,
	0x12, 0x04, 0x00
};

static u16 negative_g_gamma[] = 
{ 
	0xEB, 0x05, 0x10, 0x0A, 0x0A,
	0x07, 0x02, 0x04, 0x06, 0x0A,
	0x11, 0x19, 0x15, 0x16, 0x1C,
	0x1D, 0x08, 0x0C
};

static u16 negative_b_gamma[] = 
{ 
	0xEC, 0x05, 0x23, 0x0A, 0x0A,
	0x07, 0x02, 0x04, 0x09, 0x0C,
	0x12, 0x1B, 0x18, 0x26, 0x23,
	0x26, 0x14, 0x00
};

static u16 display_control[] = 
{ 
	0xF2, 0x01, 0x80, 0x00, 0x04,
	0x08, 0x08
};

static u16 resolution[] = 
{ 
	0xB8, 0x12
};

static u16 pixel_format[] = 
{ 
	0x3A, 0x50
};

static u16 sleep_out[] = 
{ 
	0x11
};

static u16 disp_on[] = 
{ 
	0x29
};

static u16 disp_off[] = 
{ 
	0x28
};

static u16 sleep_in[] = 
{ 
	0x10
};

static int lms606kf01_write(struct spi_device *spi, u16 cmd, const void *buf, size_t len)
{
	int i = 0;
	struct spi_message	message;
	struct spi_transfer	x[2];

	spi_message_init(&message);
	memset(x, 0, sizeof x);
       
	x[0].tx_buf	= &cmd;
	x[0].len	= 1 << 1; /* byte count */
	x[0].bits_per_word = 9;
	spi_message_add_tail(&x[0], &message);

	if((len > 0) && buf)
	{
		/* The 9th bit of data is 1 */
		u16* data = (u16*)buf;
		for( i = 0; i < len; i++)
		{
			*data |= 0x100;			
			data++;
		}

		x[1].tx_buf	= buf;
		x[1].len	= len << 1; /* byte count */
		x[1].bits_per_word = 9;
		spi_message_add_tail(&x[1], &message);
	}
	/* do the i/o */
	return spi_sync(spi, &message);
}

static int lms606kf01_power_on(struct spi_device *spi)
{
	int ret = 0;
	struct lcd_info *lms606kf_info = spi->dev.platform_data;

	mdelay(10);

	if(lms606kf_info->lcd_on)
		lms606kf_info->lcd_on(1);

	mdelay(120);

	LMS606KF01_SET(password);
	LMS606KF01_SET(vcom);
	LMS606KF01_SET(sleep_out);
	
	mdelay(1);
	
	LMS606KF01_SET(smps_posi);
	LMS606KF01_SET(smps_nega);
	LMS606KF01_SET(smps);
	LMS606KF01_SET(power_control);
	LMS606KF01_SET(source_control);
	LMS606KF01_SET(positive_r_gamma);
	LMS606KF01_SET(positive_g_gamma);
	LMS606KF01_SET(positive_b_gamma);
	LMS606KF01_SET(negative_r_gamma);
	LMS606KF01_SET(negative_g_gamma);
	LMS606KF01_SET(negative_b_gamma);
	LMS606KF01_SET(display_control);
	LMS606KF01_SET(resolution);
	LMS606KF01_SET(pixel_format);

	mdelay(120);
	
	LMS606KF01_SET(disp_on);
	
	mdelay(120);

	return ret;	
}

static int lms606kf01_power_off(struct spi_device *spi)
{
	int ret = 0;
	struct lcd_info *lms606kf_info = spi->dev.platform_data;;	
	
	LMS606KF01_SET(disp_off);
	LMS606KF01_SET(sleep_in);

	mdelay(120);

	if(lms606kf_info->lcd_on)
		lms606kf_info->lcd_on(0);	

	return ret;	
}

static int __devinit lms606kf01_probe(struct spi_device *spi)
{

	printk(LMS606KF01_DEVNAME " probing...\n");
	
	spi->bits_per_word = 9;
	spi_setup(spi);

	lms606kf01_power_on(spi);
	
	printk(LMS606KF01_DEVNAME " probed.\n");

	return 0;
}

static int __devexit lms606kf01_remove(struct spi_device *spi)
{
	lms606kf01_power_off(spi);
	return 0;
}

static int lms606kf01_shutdown(struct spi_device *spi)
{
	lms606kf01_power_off(spi);
	return 0;
}

static int lms606kf01_suspend(struct spi_device *spi, pm_message_t mesg)
{
	lms606kf01_power_off(spi);
	return 0;
}

static int lms606kf01_resume(struct spi_device *spi)
{
	lms606kf01_power_on(spi);
	return 0;
}
static struct spi_driver lms606kf01_driver = {
	.driver = {
		.name   = LMS606KF01_DEVNAME,
		.bus    = &spi_bus_type,
		.owner  = THIS_MODULE,
	},
	.probe  = lms606kf01_probe,
	.remove = __devexit_p(lms606kf01_remove),
	.shutdown  = lms606kf01_shutdown,
	.suspend  = lms606kf01_suspend,
	.resume  = lms606kf01_resume,
};

static int __init lms606kf01_init(void)
{
	int ret = spi_register_driver(&lms606kf01_driver);
	printk(KERN_INFO LMS606KF01_DEVNAME	" is  rgistered. [%i]\n", ret);
	return ret;
}

static void __exit lms606kf01_exit(void)
{
	spi_unregister_driver(&lms606kf01_driver);
}

module_init(lms606kf01_init);
module_exit(lms606kf01_exit);


MODULE_AUTHOR("Will Lin <Will.Lin@tomtom.com>");
MODULE_DESCRIPTION("Driver for SPI connected LMS606KF01.");
MODULE_LICENSE("GPL");

