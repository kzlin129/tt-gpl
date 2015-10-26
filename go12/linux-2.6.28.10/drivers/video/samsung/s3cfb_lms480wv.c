/*
 * drivers/video/s3c/s3cfb_lms480wv.c
 *
 * Copyright (C) 2008 Jeroen Taverne <jeroen.taverne@samsung.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	S3C Frame Buffer Driver
 *	based on skeletonfb.c, sa1100fb.h, s3c2410fb.c
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/sysrq.h>
#include <linux/console.h>

#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/kthread.h>

#include <linux/cordoba_lms480wv.h>

#include <asm/io.h>
#include <asm/irq.h>


#include <asm/mach-types.h>

#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include <plat/regs-gpio.h>
#include <plat/regs-lcd.h>

#include "s3cfb.h"


#define BUF_LEN 2

#define S3C_FB_HFP		16	/* front porch */
#define S3C_FB_HSW		13	/* hsync width */
#define S3C_FB_HBP		86	/* back porch */

#define S3C_FB_VFP		8	/* front porch */
#define S3C_FB_VSW		1	/* vsync width */
#define S3C_FB_VBP		19	/* back porch */

#define S3C_FB_HRES		800	/* horizon pixel  x resolution */
#define S3C_FB_VRES		480	/* line cnt       y resolution */

#define S3C_FB_HRES_VIRTUAL	S3C_FB_HRES	/* horizon pixel  x resolution */
#define S3C_FB_VRES_VIRTUAL	S3C_FB_VRES*2	/* line cnt       y resolution */

#define S3C_FB_HRES_OSD		S3C_FB_HRES	/* horizon pixel  x resolution */
#define S3C_FB_VRES_OSD		S3C_FB_VRES	/* line cnt       y resolution */

#define S3C_FB_VFRAME_FREQ     	30	/* frame rate freq */

#define S3C_FB_PIXEL_CLOCK	(S3C_FB_VFRAME_FREQ * (S3C_FB_HFP + S3C_FB_HSW + S3C_FB_HBP + S3C_FB_HRES) * (S3C_FB_VFP + S3C_FB_VSW + S3C_FB_VBP + S3C_FB_VRES))

#define TT_LCM_PWR_EN      S3C_GPL2
#define TT_LCD43_PON       S3C_GPC3

static void s3cfb_set_fimd_info(void)
{
	s3cfb_fimd.vidcon1 = S3C_VIDCON1_IHSYNC_INVERT | S3C_VIDCON1_IVSYNC_INVERT | S3C_VIDCON1_IVDEN_NORMAL;
	s3cfb_fimd.vidtcon0 = S3C_VIDTCON0_VBPD(S3C_FB_VBP - 1) | S3C_VIDTCON0_VFPD(S3C_FB_VFP - 1) | S3C_VIDTCON0_VSPW(S3C_FB_VSW - 1);
	s3cfb_fimd.vidtcon1 = S3C_VIDTCON1_HBPD(S3C_FB_HBP - 1) | S3C_VIDTCON1_HFPD(S3C_FB_HFP - 1) | S3C_VIDTCON1_HSPW(S3C_FB_HSW - 1);
	s3cfb_fimd.vidtcon2 = S3C_VIDTCON2_LINEVAL(S3C_FB_VRES - 1) | S3C_VIDTCON2_HOZVAL(S3C_FB_HRES - 1);

	s3cfb_fimd.vidosd0a = S3C_VIDOSDxA_OSD_LTX_F(0) | S3C_VIDOSDxA_OSD_LTY_F(0);
	s3cfb_fimd.vidosd0b = S3C_VIDOSDxB_OSD_RBX_F(S3C_FB_HRES - 1) | S3C_VIDOSDxB_OSD_RBY_F(S3C_FB_VRES - 1);

	s3cfb_fimd.vidosd1a = S3C_VIDOSDxA_OSD_LTX_F(0) | S3C_VIDOSDxA_OSD_LTY_F(0);
	s3cfb_fimd.vidosd1b = S3C_VIDOSDxB_OSD_RBX_F(S3C_FB_HRES_OSD - 1) | S3C_VIDOSDxB_OSD_RBY_F(S3C_FB_VRES_OSD - 1);

	s3cfb_fimd.width = S3C_FB_HRES;
	s3cfb_fimd.height = S3C_FB_VRES;
	s3cfb_fimd.xres = S3C_FB_HRES;
	s3cfb_fimd.yres = S3C_FB_VRES;

#if defined(CONFIG_FB_S3C_VIRTUAL_SCREEN)
	s3cfb_fimd.xres_virtual = S3C_FB_HRES_VIRTUAL;
	s3cfb_fimd.yres_virtual = S3C_FB_VRES_VIRTUAL;
#else
	s3cfb_fimd.xres_virtual = S3C_FB_HRES;
	s3cfb_fimd.yres_virtual = S3C_FB_VRES;
#endif

	s3cfb_fimd.osd_width = S3C_FB_HRES_OSD;
	s3cfb_fimd.osd_height = S3C_FB_VRES_OSD;
	s3cfb_fimd.osd_xres = S3C_FB_HRES_OSD;
	s3cfb_fimd.osd_yres = S3C_FB_VRES_OSD;

	s3cfb_fimd.osd_xres_virtual = S3C_FB_HRES_OSD;
	s3cfb_fimd.osd_yres_virtual = S3C_FB_VRES_OSD;

  	s3cfb_fimd.pixclock = S3C_FB_PIXEL_CLOCK;

	s3cfb_fimd.hsync_len = S3C_FB_HSW;
	s3cfb_fimd.vsync_len = S3C_FB_VSW;
	s3cfb_fimd.left_margin = S3C_FB_HFP;
	s3cfb_fimd.upper_margin = S3C_FB_VFP;
	s3cfb_fimd.right_margin = S3C_FB_HBP;
	s3cfb_fimd.lower_margin = S3C_FB_VBP;
}


void s3cfb_init_hw(void)
{
	printk(KERN_INFO "LCD TYPE :: LMS480WV will be initialized\n");
	
	s3cfb_set_fimd_info();
	s3cfb_set_gpio();
}

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
