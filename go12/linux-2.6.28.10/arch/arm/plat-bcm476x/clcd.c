/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
/*  derived from linux/arch/arm/mach-versatile/core.c
 *  linux/arch/arm/mach-bcm476x/core.c
 *
 *  Copyright (C) 1999 - 2003 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/amba/bus.h>
#include <linux/amba/bcmring_clcd.h>

#include <plat/core.h>
#include <plat/clcd.h>

#include <linux/gpio.h>
#include <linux/vgpio.h>
#include <plat/irvine.h>

static clcd_lookup_t clcd_lookup_panel;

static int bcm476x_clcd_setup	(struct clcd_fb *fb);
static int bcm476x_clcd_mmap	(struct clcd_fb *fb, struct vm_area_struct *vma);
static void bcm476x_clcd_remove	(struct clcd_fb *fb);

static struct clcd_board clcd_plat_data = 
{
	.name		= "BCM476X",

	/* Panel management methods */
	.panel_init	= NULL,
	.panel_enable	= NULL,
	.panel_disable	= NULL,
	.panel_suspend 	= NULL,
	.panel_resume	= NULL,
	.panel_setup	= NULL,

	/* Mode management methods */
	.check		= clcdfb_check,
	.decode		= clcdfb_decode,

	/* CLCD management */
	.setup		= bcm476x_clcd_setup,
	.mmap		= bcm476x_clcd_mmap,
	.remove		= bcm476x_clcd_remove,
};

static int bcm476x_clcd_setup(struct clcd_fb *fb)
{
	unsigned long framesize;
	dma_addr_t dma;

	/* Find the panel type that has been registered and set it up */
	if (clcd_lookup_panel != NULL) {
		struct clcd_device *dev;

		dev = clcd_lookup_panel (fb->panel_index);
		if (dev == NULL) {
			printk (KERN_ERR "CLCD: Unable to find panel with index: %d\n", fb->panel_index);
			return -1;
		}

		fb->panel = dev->timings;

		clcd_plat_data.panel_enable  	= dev->cntrl->enable;
		clcd_plat_data.panel_disable 	= dev->cntrl->disable;
		clcd_plat_data.panel_suspend 	= dev->cntrl->suspend;
		clcd_plat_data.panel_resume	= dev->cntrl->resume;
		clcd_plat_data.panel_init	= dev->cntrl->init;
		clcd_plat_data.panel_setup	= dev->cntrl->setup;
	}

	framesize = ((fb->panel->mode.xres * fb->panel->mode.yres) *
	             (fb->panel->bpp/8)) * 2; 

	/* As we would like to do direct rendering, we need to pass Mali a Page Size aligned buffer, also a multiple of Page_Size. */
	framesize= (framesize + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);

	fb->fb.screen_base = dma_alloc_writecombine(&fb->dev->dev, framesize,
						    &dma, GFP_KERNEL);
	if (!fb->fb.screen_base) {
		printk(KERN_ERR "CLCD: unable to map framebuffer\n");
		return -ENOMEM;
	}

	fb->fb.fix.smem_start	= dma;
	fb->fb.fix.smem_len	= framesize;

	return 0;
}

static int bcm476x_clcd_mmap(struct clcd_fb *fb, struct vm_area_struct *vma)
{
	return dma_mmap_writecombine(&fb->dev->dev, vma,
				     fb->fb.screen_base,
				     fb->fb.fix.smem_start,
				     fb->fb.fix.smem_len);
}

static void bcm476x_clcd_remove(struct clcd_fb *fb)
{
	dma_free_writecombine(&fb->dev->dev, fb->fb.fix.smem_len,
			      fb->fb.screen_base, fb->fb.fix.smem_start);
}

void bcm476x_clcd_lookup_register(clcd_lookup_t lookup_func)
{
	clcd_lookup_panel = lookup_func;
}

AMBA_DEVICE(clcd,  "dev:20",  LCD,   &clcd_plat_data);

void __init bcm476x_amba_lcd_init(void)
{
	amba_device_register(&clcd_device, &iomem_resource);
}

