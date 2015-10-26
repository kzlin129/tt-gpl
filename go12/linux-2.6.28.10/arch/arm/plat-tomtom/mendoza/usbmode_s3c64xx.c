/*
 * Copyright (C) 2008 TomTom BV <http://www.tomtom.com/> 
 * Author: Rogier Stam <rogier.stam@tomtom.com> 
 *    Andrzej Zukowski <andrzej.zukowski@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation. 
 *
*/
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <plat/regs-otg.h>
#include <mach/hardware.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <mach/map.h>
#include <plat/usbmode.h>
#include <plat/synap_ll_usbmode_light_plat.h>
#include <asm/io.h>

static struct resource s3c64xx_usbmode_resources[]=
{
	{
		.name		=	"interrupt",
		.start		=	IRQ_OTG,
		.end		=	IRQ_OTG,
		.flags		=	IORESOURCE_IRQ,
		.parent		=	NULL,
		.child		=	NULL,
		.sibling	=	NULL,
	}
};

static inline void s3c64xx_otg_phy_quirk(void)
{
	/* Set some undocumented registers to disable overcurrent detection
	   in the USB controller.

	   Instructions from Samsung (for 6410/6431):
	   Set some undocumented registers:
	   - Phy.Addr 7C10_0018:
	     [9] Vbus_vld_ext = 1 (for OTGPHY)
	       0: VBUS signal inactive (default))
	       1: VBUS signal active
	     [10] Vbus_vld_ext_sel
	       0: internal (default)
	       1: external

	   - Phy.Addr 7C10_0080:
	     [1:0] VBUS Valid
	       00 : VBUS signal inactive
	       10 : VBUS signal inactive (?)
	       11 : VBUS signal active
	*/

	writel(readl(S3C_USBOTG_PHYREG(0x18)) | (3 << 9), S3C_USBOTG_PHYREG(0x18));
	writel(readl(S3C_USBOTG_PHYREG(0x80)) | 3, S3C_USBOTG_PHYREG(0x80));
}

static int s3c64xx_usbmode_setup( struct platform_device *pdev )
{
	return 0;
}

static irqreturn_t s3c64xx_usbmode_irq_handler( struct platform_device *pdev )
{
	uint32_t gintsts;

	gintsts = readl( S3C_UDC_OTG_GINTSTS );
	if( gintsts & INT_RESET )
	{
		/* Generate the reset event. */
		s3c64xx_otg_phy_quirk();
		usbmode_push_event(INPUT_DEVICE_RESET);
	}
	return IRQ_NONE;
}

static struct synap_ll_usbmode_platform_data s3c64xx_usbmode_plat_data =
{
	.irqhandler		=	s3c64xx_usbmode_irq_handler,
	.setup			=	s3c64xx_usbmode_setup,
	.device_detect_timeout	=	DEVICE_DETECT_TIMEOUT_SECONDS,
};

static struct platform_device s3c64xx_usbmode_pdev=
{
	.name			=	"synap-ll-usbmode",
	.id			=	0,
	.num_resources		=	ARRAY_SIZE( s3c64xx_usbmode_resources ),
	.resource		=	s3c64xx_usbmode_resources,
	.dev.platform_data	=	&s3c64xx_usbmode_plat_data,
};


static int __init s3c64xx_usbmode_init(void)
{
	int ret = platform_device_register(&s3c64xx_usbmode_pdev);
	return ret;
}
arch_initcall( s3c64xx_usbmode_init );

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("S3C64XX USBMode Arch code");
