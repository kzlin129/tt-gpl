/* drivers/video/tomtom/s3c24xx_lcd.c
 *
 * TomTom GO S3C24xx LCD Controller
 * based on skeletonfb.c, sa1100fb.c, tomtomgofb.c
 *
 * Copyright (C) 2005,2006,2007 TomTom BV <http://www.tomtom.com/>
 * Author: Christian Daniel <cd@cdaniel.de>
 * Author: Thomas Kleffel <tk@maintech.de>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 * Author: Mark-Jan Bastian <mark-jan.bastian@tomtom.com>
 * Author: Martin Jackson <martin.jackson@tomtom.com>
 * Author: Onno Hovers <onno.hovers@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/tomtomgofb.h>
#include <linux/dma-mapping.h>
#include <linux/cpufreq.h>

#include <asm/hardware/clock.h>
#include <asm/arch/regs-gpio.h>
#include <asm/uaccess.h>

#include <barcelona/gopins.h>
#include <barcelona/gotype.h>
#include <barcelona/debug.h>
#include <barcelona/Barc_pwm.h>
#include <barcelona/ttgfb.h>

#include "lcd_controller.h"
#include "screeninfo.h"
#include "lcdregs.h"

#define BYTE_PER_PIXEL 2

static void s3c24xx_lcd_initialize(struct lcd_screen_info *screeninfo, struct lcd_additional_info *addinfo)
{
	unsigned long flags;
	
	local_irq_save(flags);
	
	/* Start display controller */
	rLCDCON1 &= ~1;
	rTCONSEL = 0;
	rLCDCON1 = (addinfo->divider << 8) | (((u32) addinfo->mmode) << 7) | (((u32) screeninfo->pnr_mode) << 5) |
			(((u32) screeninfo->bpp_mode) << 1) | ((u32)addinfo->envid << 0);
	rLCDCON2 = (((u32) screeninfo->screen.tft.vbpd) << 24) |
			(((u32) addinfo->lineval) << 14) |
			(((u32) screeninfo->screen.tft.vfpd) << 6) |
			(((u32) screeninfo->screen.tft.vspw) << 0);
	rLCDCON3 = (((u32) screeninfo->screen.tft.hbpd) << 19) |
			(((u32) addinfo->hozval) << 8) |
			(((u32) screeninfo->screen.tft.hfpd) << 0);
	rLCDCON4 = (((u32)addinfo->mval) << 8) | (((u32) screeninfo->screen.tft.hspw) << 0);
	rLCDCON5 =	(((u32) addinfo->bpp24bl) << 12) | (((u32) addinfo->frm565) << 11) |
			(((u32) addinfo->invvclk) << 10) | (((u32) addinfo->invvline) << 9) | (((u32) addinfo->invvframe) << 8) |
			(((u32) addinfo->invvd) << 7)    | (((u32) addinfo->invvden) << 6)  | (((u32) addinfo->invpwren) << 5) |
			(((u32) addinfo->invlend) << 4)  | (((u32) addinfo->pwren) << 3)    | (((u32) addinfo->enlend) << 2) |
			(((u32) addinfo->bswp) << 1)     | (((u32) addinfo->hwswp) << 0);
	
	local_irq_restore(flags);
}

static void s3c24xx_lcd_update_base(int layer, struct lcd_screen_info *screeninfo, struct fb_info *fbi, struct ttgfb_window *winptr )
{
	unsigned long flags;

	struct ttgfb_window win        = *winptr;
	unsigned long       start      = fbi->fix.smem_start + (fbi->var.yoffset * win.layer_width * BYTE_PER_PIXEL);
	unsigned long       winstart   = start    + (win.src_top    *  win.layer_width + win.src_left) * BYTE_PER_PIXEL;
	unsigned long       winstop    = winstart + ((win.height-1) *  win.layer_width  + win.width) * BYTE_PER_PIXEL;
	unsigned long       winoffsize =  win.layer_width - win.width;

	local_irq_save(flags);
	
	if (rLCDCON1 & 1)
		while ((rLCDCON1 >> 18) == 0)
			/* nothing */;
	
	rLCDSADDR1 = S3C2410_LCDBANK(winstart >> 22)|S3C2410_LCDBASEU(winstart >> 1);
	rLCDSADDR2 = (winstop) >> 1;
	rLCDSADDR3 = S3C2410_OFFSIZE(winoffsize)|S3C2410_PAGEWIDTH(win.width);
	
	local_irq_restore(flags);
}

static void s3c24xx_lcd_show_color_screen(unsigned color)
{
	// Show black screen
	rTPAL = (1<<24)|color;
}

static void s3c24xx_lcd_show_normal_screen(void)
{
	// Show normal screen
	rTPAL = (0<<24);
}

static int s3c24xx_lcd_get_number_of_layers(void)
{
	return 1;
}

static unsigned s3c24xx_lcd_get_base(int layer)
{
	return rLCDSADDR1 << 1;
}

static void s3c24xx_lcd_start_lcd(void)
{	
	// Show black screen
	rTPAL = (1<<24)|0x000000;
	
	// Start display controller
	rLCDCON1 |= (1 << 0);
		
	mdelay(200);
	// Show normal screen
	rTPAL = (0<<24);
}

static void s3c24xx_lcd_stop_lcd(void)
{
	// Stop display controller
	rLCDCON1 &= ~1;
}

static const char *s3c24xx_lcd_get_id(void)
{
	return "s3c24xx-lcd";
}

/* Returns the actual set clock divider value. */
static unsigned short s3c24xx_lcd_set_clkval( struct lcd_screen_info *screeninfo, unsigned short clkval )
{
	unsigned long int	regval;

	/* Is this a legal value? */
	if( clkval > 1023 )
		clkval=1023;

	/* Set the CLK value only if the divider is within bounds. */
	regval=rLCDCON1 & 0xFFFC00FF;
	rLCDCON1=regval | (((unsigned long int) clkval) << 8);
	
	/* Return the actual divider calculated. */
	return clkval;
}

static int s3c24xx_lcd_ioctl(unsigned int cmd, unsigned long arg, struct fb_info *info)
{
	return -EINVAL;	
}

static void s3244xx_lcd_resume(void)
{
}

struct lcd_controller s3c24xx_lcd_controller = {
	.initialize           = s3c24xx_lcd_initialize,
	.update_base          = s3c24xx_lcd_update_base,
	.show_color_screen    = s3c24xx_lcd_show_color_screen,
	.show_normal_screen   = s3c24xx_lcd_show_normal_screen,
	.get_number_of_layers = s3c24xx_lcd_get_number_of_layers,
	.get_base             = s3c24xx_lcd_get_base,
	.start_lcd            = s3c24xx_lcd_start_lcd,
	.stop_lcd             = s3c24xx_lcd_stop_lcd,
	.get_id               = s3c24xx_lcd_get_id,
	.set_clkval           = s3c24xx_lcd_set_clkval,
	.ioctl                = s3c24xx_lcd_ioctl,
	.resume               = s3244xx_lcd_resume
};

/* EOF */
