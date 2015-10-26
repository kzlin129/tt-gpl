/*
 * Copyright (C) 2008 TomTom BV <http://www.tomtom.com/> 
 * Author: Rogier Stam <rogier.stam@tomtom.com> 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation. 
 *
*/
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <mach/hardware.h>
#include <linux/gpio.h>
#include <asm/arch/bcm4760_reg.h>
#include <plat/usbmode.h>
#include <plat/synap_ll_usbmode_light_plat.h>
#include <plat/irvine.h>
#include <linux/vgpio.h>
#include <asm/io.h>

static struct platform_device usbmode_pdev=
{
	.name			=	"tomtom-usbmode",
	.id			=	1,
};

static struct resource bcm4760_usbmode_resources[]=
{
	{
		.name		=	"USB-core-interrupt",
		.start		=	BCM4760_INTR_USB,
		.end		=	BCM4760_INTR_USB,
		.flags		=	IORESOURCE_IRQ,
		.parent		=	NULL,
		.child		=	NULL,
		.sibling	=	NULL,
	},
	{
		.name		=	"base-address",
		.start		=	IO_ADDRESS( USB_R_GOTGCTL_MEMADDR ),
		.end		=	IO_ADDRESS( USB_R_DTXFSTS12_MEMADDR ) + 4,
		.flags		=	IORESOURCE_MEM,
		.parent		=	NULL,
		.child		=	NULL,
		.sibling	=	NULL,
	},
	{
		.name		=	"block-reset",
		.start		=	IO_ADDRESS( CMU_R_BLOCK_RESET0_MEMADDR ),
		.end		=	IO_ADDRESS( CMU_R_BLOCK_RESET0_MEMADDR ) + 4,
		.flags		=	IORESOURCE_MEM,
		.parent		=	NULL,
		.child		=	NULL,
		.sibling	=	NULL,
	},
};

static irqreturn_t bcm4760_usbmode_irq_handler( struct platform_device *pdev )
{
	struct resource			*base=platform_get_resource( pdev, IORESOURCE_MEM, 0 );
	uint32_t			gintsts=readl( base->start + USB_R_GINTSTS_SEL );

	if( gintsts & USB_F_USBRSTMSK_MASK )
	{
		/* Generate the reset event. */
		usbmode_push_event( INPUT_DEVICE_RESET );
	}
	return IRQ_NONE;
}

static int bcm4760_usbmode_setup( struct platform_device *pdev )
{
	struct resource			*otg_base=platform_get_resource( pdev, IORESOURCE_MEM, 0 );
	struct resource			*block_reset_base=platform_get_resource( pdev, IORESOURCE_MEM, 1 );
	uint32_t			gintmsk;

	/* Reset the controller. */
	writel( readl( block_reset_base->start ) & ~(CMU_F_CMU_USB_HRESETN_MASK), block_reset_base->start );
	writel( readl( block_reset_base->start ) | CMU_F_CMU_USB_HRESETN_MASK, block_reset_base->start );

	/* Ack any outstanding interrupts, as we don't want to be interrupted by old stuff. */
	writel( readl( otg_base->start + USB_R_GOTGINT_SEL ), otg_base->start + USB_R_GOTGINT_SEL );
	writel( readl( otg_base->start + USB_R_GINTSTS_SEL ), otg_base->start + USB_R_GINTSTS_SEL );

        /* Enable the IRQs we're interested in. Leave the Global Interrupt Mask masked. DWC_OTG will take care of that. */
        writel( readl( otg_base->start + USB_R_GAHBCFG_SEL ) & ~(USB_F_GLBLINTRMSK_MASK), otg_base->start + USB_R_GAHBCFG_SEL );
        gintmsk=readl( otg_base->start + USB_R_GINTMSK_SEL );
        gintmsk|=USB_F_SESSREQINTMSK_MASK | USB_F_DISCONNINTMSK_MASK | USB_F_OTGINTMSK_MASK | USB_F_USBRSTMSK_MASK | USB_F_CONIDSTSCHNGMSK_MASK;
        writel( gintmsk, otg_base->start + USB_R_GINTMSK_SEL );
	return 0;
}

static int bcm4760_usbmode_is_usbhost( struct platform_device *pdev )
{
	struct resource			*otg_base=platform_get_resource( pdev, IORESOURCE_MEM, 0 );

	return (readl( otg_base->start + USB_R_GOTGCTL_SEL ) & USB_F_ASESVLD_MASK ? 1 : 0);
}

static struct synap_ll_usbmode_platform_data bcm4760_usbmode_plat_data =
{
	.irqhandler		=	bcm4760_usbmode_irq_handler,
	.setup			=	bcm4760_usbmode_setup,
	.device_detect_timeout	=	DEVICE_DETECT_TIMEOUT_SECONDS,
	.is_usbhost		=	bcm4760_usbmode_is_usbhost,
};
	
static struct platform_device bcm4760_usbmode_pdev=
{
	.name			=	"synap-ll-usbmode",
	.id			=	0,
	.num_resources		=	ARRAY_SIZE( bcm4760_usbmode_resources ),
	.resource		=	bcm4760_usbmode_resources,
	.dev.platform_data	=	&bcm4760_usbmode_plat_data,
};

static int __init bcm4760_usbmode_init(void)
{
	/* Register the platform devices. */
	platform_device_register( &usbmode_pdev );
	platform_device_register( &bcm4760_usbmode_pdev );
	return 0;
}
arch_initcall( bcm4760_usbmode_init );

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BCM4760 USBMode Arch code");
