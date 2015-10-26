/* drivers/video/tomtom/s3c2443_tft.c
 *
 * TomTom GO S3C2443 TFT LCD Controller
 * based on ttgfb.c
 *
 * Copyright (C) 2005,2006,2007 TomTom BV <http://www.tomtom.com/>
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
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/tomtomgofb.h>
#include <linux/dma-mapping.h>
#include <linux/cpufreq.h>
#include <linux/bootmem.h>

#include <asm/hardware/clock.h>
#include <asm/arch/regs-gpio.h>
#include <asm/uaccess.h>

#include <barcelona/gopins.h>
#include <barcelona/gotype.h>
#include <barcelona/debug.h>
#include <barcelona/ttgfb.h>

#include "lcd_controller.h"
#include "screeninfo.h"
#include "lcdregs.h"

#define BYTE_PER_PIXEL 2

static unsigned long resume_colorkey;
static unsigned long resume_alpha;
static int resume_colorkey_defined = 0;

#define _IGNORE_INPUTSYNC	0
#define _FOLLOW_INPUTSYNC	1

static int last_set_framerate;
static int last_requested_framerate;

/*DEBUG_GRAPHICS  bit masks      1:sync location flow       2:sync location    */
#define DEBUG_GRAPHICS 0

/* Enable when also horizontal settings should be changed for setting framerate: */
//#define S3C2443_TFT_TWEAK


static unsigned int	lineval;	/*original LINEVAL*/
static unsigned int	vfpd;	/*original VFPD */
static int		vsync_adj_state;	/*state of sync-follow system.*/

#if  (DEBUG_GRAPHICS)
static unsigned int 	cpubase0	= 0;
static unsigned int 	cpubase1	= 0;
static unsigned int 	idxy		= 0;
static int 		flags		= 0x6;
#endif

/*
framerate is calculated as: dotclock/(total image+boundary area)
That area is calculated as: (hspw+hfpd+hbpd+hozval)*(vspw,vbpd,vfpd,lineval)
Keeping the same display picture, the framerate can by changed by adjusting the hspw,hfpd,hbpd,vspw,vbpd and vfpd.
The vfpd part is the one that will be used for this. If framerate is still to low for that, then (when enabled
with S3C2443_TFT_TWEAK)  the values of hspw, hbpd and hfpd will be changed.
Possible addition if vfpd is not big enough to decrease the framerate:
	  - increase liveval (but do not decrease to under what we want to view)
 */
static int s3c2443_set_framerate(int framerate)
{
	long clksel = 133000000;
	long clkdiv = ((rVIDCON0>>6)&0x3f)+1;
	int vspw,vbpd,_vfpd,_lineval,vtotal;
	int hspw,hfpd,hbpd,hozval,htotal;
	unsigned long vidcon=rVIDTCON0;
	
	last_requested_framerate = framerate;
	
	/*collect all register values*/
	vspw = 1+(vidcon&255);
	_vfpd = 1+((vidcon>>8)&255);
	vbpd = 1+((vidcon>>16)&255);
	vidcon=rVIDTCON1;
	hspw = 1+(vidcon&255);
	hfpd = 1+((vidcon>>8)&255);
	hbpd = 1+((vidcon>>16)&255);
	vidcon=rVIDTCON2;
	hozval = 1+(vidcon&0x7ff);
	_lineval = 1+(vidcon>>11);

	/* use shadow values in case we are following the sync. Because in that case, the register values will change all the time */
	if (lineval) _lineval=lineval;
	if (vfpd) _vfpd=vfpd;
	
	/*total line length*/
	htotal = hspw+hfpd+hbpd+hozval;
	
	/* calc new vtotal & vfpd  needed for requested framerate*/
    vtotal = clksel/(clkdiv*framerate*htotal);
	_vfpd = vtotal-vspw-vbpd-_lineval;

#ifdef 	S3C2443_TFT_TWEAK
	printk(KERN_ERR "Trying to set tft refresh rate by tweaking horizontal values.\n");
	/* change value till we get closest to needed vfpd*/
	while ((_vfpd<3)&&(hspw>4)&&(hbpd>4)&&(hfpd>2))
	{
		if (hfpd>hspw)
		{
			hfpd--;
		}
		else
		{
			if (hspw>hbpd)
			{
				hspw--;
			}
			else
			{
			    if (hbpd>4) hbpd--;
			}
		}
		
		htotal = hspw+hfpd+hbpd+hozval;
		vtotal = clksel/(clkdiv*framerate*htotal);
		_vfpd = vtotal-vspw-vbpd-_lineval;
	}
#endif	
	/*calc new htotal and hfpd*/
	htotal = clksel/(clkdiv*framerate*vtotal);
	hfpd = htotal-hspw-hbpd-hozval;

	if ((last_set_framerate != framerate)&&(vsync_adj_state))
	{
		
		if (_vfpd>0)
		{
			vfpd=_vfpd;
			rVIDTCON0 = (0 << 24) |
				(((u32) vbpd-1) << 16) |
				(((u32) _vfpd-1) << 8) |
				(((u32) vspw-1) << 0);
			rVIDTCON1 = (0 << 24) |
				(((u32) hbpd-1) << 16) |
				(((u32) hfpd-1) << 8) |
				(((u32) hspw-1) << 0);

				printk(KERN_ERR "Set tft refresh rate to %ldHz(%dHz requested). (clksel=%ld,dclk=%ld).\n",clksel/(htotal*vtotal*clkdiv),framerate,clksel,clksel/clkdiv);
				if (lineval) lineval=_lineval;
				if (vfpd) vfpd=_vfpd;
		}
		else
		{
			printk(KERN_ERR "Setting tft refresh rate to %dHz failed. (clksel=%ld,dclk=%ld).\n",framerate,clksel,clksel/clkdiv);
		}
		last_set_framerate=framerate;
	}	

	
	return(clksel/(htotal*vtotal*clkdiv));
	
}

/* Update register only if values differs: some could have been
   already initialized by the bootloader, and setting them with
   the same value again will make the screen flicker for no reason
   (rVIDCON0 for eg). */
#define UPD_REGISTER(reg, new_value) \
	{	unsigned long old_value = reg; \
		if ((new_value) != (old_value)) \
			reg = (new_value); \
	}

static void s3c2443_tft_initialize(struct lcd_screen_info *screeninfo, struct lcd_additional_info *addinfo)
{
	unsigned long int new_divider = 2 * addinfo->divider + 1;
	
	UPD_REGISTER(rVIDCON0, (new_divider << 6) |
			(1 << 5) | (1 << 4) | (1 << 1) | (1 << 0));
	UPD_REGISTER(rVIDCON1, ((u32)addinfo->invvclk << 7) | 
			((u32)addinfo->invvline << 6) | 
			((u32)addinfo->invvframe << 5) |
			((u32)addinfo->invvden << 4));
	vfpd = screeninfo->screen.tft.vfpd;
	UPD_REGISTER(rVIDTCON0, (0 << 24) |
			(((u32) screeninfo->screen.tft.vbpd) << 16) |
			(((u32) screeninfo->screen.tft.vfpd) << 8) |
			(((u32) screeninfo->screen.tft.vspw) << 0));
	UPD_REGISTER(rVIDTCON1, (0 << 24) |
			(((u32) screeninfo->screen.tft.hbpd) << 16) |
			(((u32) screeninfo->screen.tft.hfpd) << 8) |
			(((u32) screeninfo->screen.tft.hspw) << 0));
	lineval = (screeninfo->y_res -1);
	UPD_REGISTER(rVIDTCON2, (lineval << 11) |
			((screeninfo->x_res -1) << 0));

	/* window control */
	UPD_REGISTER(rWINCON0, ((u32)addinfo->bswp << 17) |
			((u32)addinfo->hwswp << 16) |
			(0 << 9) | (5 << 2) | (1 << 0));
	UPD_REGISTER(rWINCON1, ((u32)addinfo->bswp << 17) |
			((u32)addinfo->hwswp << 16) |
			(0 << 9) | (1 << 6) | (5 << 2) | (0 << 1) |(1 << 0));

	/* RGB 565, no dithering */
	UPD_REGISTER(rDITHMODE, (0 << 5) | (1 << 3) | (0 << 1) | (0 << 0));

	s3c2443_set_framerate(60);
}

static void s3c2443_tft_update_base(int layer, struct lcd_screen_info *screeninfo, struct fb_info *fbi, struct ttgfb_window *winptr )
{
	unsigned long flags;

	struct ttgfb_window win        =  *winptr;
	unsigned long       start      = fbi->fix.smem_start + fbi->var.yoffset * win.layer_width  * BYTE_PER_PIXEL;
	unsigned long       winstart   = start    + (win.src_top    * win.layer_width  + win.src_left) * BYTE_PER_PIXEL; 
	unsigned long       winstop    = winstart + ((win.height-1) * win.layer_width + win.width) * BYTE_PER_PIXEL;
	unsigned long       winoffsize = win.layer_width - win.width;

	local_irq_save(flags);
	
	if ((rVIDCON0 & 3) == 3)
		while ((rVIDCON1 >> 16) == 0)
			/* nothing */;
	
	if(layer != 0) {
		rVIDOSD1A = win.dst_left  << 11 | win.dst_top;
		rVIDOSD1B = (win.dst_left + win.width - 1) << 11 | (win.dst_top + win.height - 1);
		
		rVIDW01ADD0 = winstart;
		rVIDW01ADD1 = winstop;
		rVIDW01ADD2 = ((winoffsize * BYTE_PER_PIXEL) <<13) | ((win.width * BYTE_PER_PIXEL) << 0);
#if (DEBUG_GRAPHICS	& 3)
		cpubase0=fbi->screen_base;
#endif		
	} else {
		rVIDOSD0A = win.dst_left  << 11 | win.dst_top;
		rVIDOSD0B = (win.dst_left + win.width - 1) << 11 | (win.dst_top + win.height - 1);
		
		rVIDW00ADD0B0 = winstart;
		rVIDW00ADD1B0 = winstop;
		rVIDW00ADD2B0 = ((winoffsize * BYTE_PER_PIXEL) <<13) | ((win.width * BYTE_PER_PIXEL) << 0);
#if (DEBUG_GRAPHICS	& 3)
		cpubase1=fbi->screen_base;
#endif		

		/* only use two different buffers when not panning */
		if (fbi->var.yoffset == 0)
		{
			winstart += screeninfo->y_res * screeninfo->x_res * BYTE_PER_PIXEL;
			winstop  += screeninfo->y_res * screeninfo->x_res * BYTE_PER_PIXEL;
		}

		rVIDW00ADD0B1 = winstart;
		rVIDW00ADD1B1 = winstop;
		rVIDW00ADD2B1 = ((winoffsize * BYTE_PER_PIXEL) <<13) | ((win.width * BYTE_PER_PIXEL) << 0);
	}
	
	local_irq_restore(flags);
}

static void s3c2443_tft_show_color_screen(unsigned color)
{
	// Show black screen
	rWIN0MAP = (1<<24)|color;
	rWIN1MAP = (1<<24)|color;
}

static void s3c2443_tft_show_normal_screen(void)
{
	// Show normal screen
	rWIN0MAP = (0<<24);
	rWIN1MAP = (0<<24);
}

static int s3c2443_tft_get_number_of_layers(void)
{
	return 2;
}

static unsigned s3c2443_tft_get_base(int layer)
{
	return (layer!=0) ? rVIDW01ADD0 : rVIDW00ADD0B0;
}

static void s3c2443_tft_start_lcd(void)
{
	// Show black screen
	rWIN0MAP = (1<<24)|0x000000;
	rWIN1MAP = (1<<24)|0x000000;
	
	// Start display controller

	/* only 2443 has both LCD and TFT controller */
	if (IO_GetCpuType() == GOCPU_S3C2443 ) 
	{
		/* Wait for end-of-frame from LCD controller before switching over to TFT controller */
		if ((rLCDCON1 & 1))			/* If LCD controller is enabled */
		{
			while ((rLCDCON1 & 0x0FFC0000))	/* Wait until linecount is 0 */
				;
			rLCDCON1 &= ~1;			/* Disable LCD controller */
		}
		rMISCCR |= (1<<28);			/* Switch output to TFT controller */
	}

	rVIDCON0 |= 3;					/* Enable TFT controller output */

	// Show normal screen
	rWIN0MAP = (0<<24);
	rWIN1MAP = (0<<24);
}

static void s3c2443_tft_stop_lcd(void)
{
	rVIDCON0 &= ~3;
}

static void s3c2443_tft_set_colorkey(int enable, unsigned key)
{
	if (!enable) {
		rW1KEYCON0 = 0;
		rW1KEYCON1 = 0;
	} else {
		rW1KEYCON0 = (1 << 26) | (1 << 25) | (0x7 << 16) | (0x3 << 8) | (0x7 << 0);
		rW1KEYCON1 = key & 0x00ffffff;
	}
}

static void s3c2443_tft_set_alpha(unsigned alpha)
{
	rVIDOSD1C = alpha;
}

static const char *s3c2443_tft_get_id(void)
{
	return "s3c2443-tft-lcd";
}

/* Returns the actual set clock divider value. */
static unsigned short s3c2443_tft_set_clkval( struct lcd_screen_info *screeninfo, unsigned short clkval )
{
	unsigned long int regval;
	unsigned long int new_clkval = 1 + 2 * clkval;   /* new_clkval + 1 = (old_clkval + 1) * 2  */
	
	/* Is this a legal value? */
	if( clkval > 31 )
		clkval= 31;
	
	/* Set the CLK value only if the divider is within bounds. */
	regval=rVIDCON0 & 0xFFFFF03F;
	rVIDCON0 =regval | (((unsigned long int) new_clkval) << 6);	
	
	/* Return the actual divider calculated. */
	return clkval;
}

static int s3c2443_tft_ioctl(unsigned int cmd, unsigned long arg, struct fb_info *info)
{
	switch(cmd) {
	case TTGFB_IOCTL_BLEND_DISABLE:
		resume_colorkey_defined = 0;
		s3c2443_tft_set_colorkey(0, 0);
		break;
	case TTGFB_IOCTL_BLEND_COLOR_KEY:
		s3c2443_tft_set_colorkey(1, arg);
		resume_colorkey = arg;
		resume_colorkey_defined = 1;
		break;
	case TTGFB_IOCTL_SET_ALPHA:
		resume_alpha = arg;
		s3c2443_tft_set_alpha(arg);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void s3c2443_tft_resume(void)
{
	if (resume_colorkey_defined)
	{
		s3c2443_tft_set_colorkey(1, resume_colorkey);
		s3c2443_tft_set_alpha(resume_alpha);
	}
	else
	{
		s3c2443_tft_set_colorkey(0, 0);
	}
}

struct lcd_controller s3c2443_tft_controller = {
	.initialize           = s3c2443_tft_initialize,
	.update_base          = s3c2443_tft_update_base,
	.show_color_screen    = s3c2443_tft_show_color_screen,
	.show_normal_screen   = s3c2443_tft_show_normal_screen,
	.get_number_of_layers = s3c2443_tft_get_number_of_layers,
	.get_base             = s3c2443_tft_get_base,
	.start_lcd            = s3c2443_tft_start_lcd,
	.stop_lcd             = s3c2443_tft_stop_lcd,
	.get_id               = s3c2443_tft_get_id,
	.set_clkval           = s3c2443_tft_set_clkval,
	.ioctl                = s3c2443_tft_ioctl,
	.resume               = s3c2443_tft_resume
};

/* EOF */
