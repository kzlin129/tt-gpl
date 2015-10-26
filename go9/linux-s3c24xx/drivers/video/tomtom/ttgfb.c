/* drivers/video/tomtom/ttgfb.c
 *
 * TomTom GO LCD Controller Frame Buffer Driver
 * based on skeletonfb.c, sa1100fb.c, tomtomgofb.c
 *
 * Copyright (C) 2005,2006,2007 TomTom BV <http://www.tomtom.com/>
 * Author: Christian Daniel <cd@cdaniel.de>
 * Author: Thomas Kleffel <tk@maintech.de>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 * Author: Mark-Jan Bastian <mark-jan.bastian@tomtom.com>
 * Author: Martin Jackson <martin.jackson@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
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
#include <barcelona/ttgfb.h>

#define PFX "ttgfb: "
#define PK_DBG PK_DBG_FUNC

#include "screeninfo.h"
#include "framerate.h"
#include "ttgfb.h"
#include "lcdregs.h"
#include "lcd_controller.h"
#include "nec.h"
#include "ltv350qv.h"
#include "lms350gf.h"
#include "lms350gf20.h"
#include "lte246qv.h"
#include "ltp400wq.h"
#include "lte430wq.h"
#include "lq043t1.h"
#include "lq035q1dg.h"
#include "lq035q1dg04.h"
#include "lms430hf12.h"
#include "a035qn02.h"
#include "a043fw03.h"
#include "a050fw02v2.h"
#include "lq043t3dw02.h"
#include "lq043t3dw01.h"
#include "lms500hf01.h"
#include "lb043wq3.h"
#include "a050fw03v2.h"
#include "ld050wq1.h"
#include "t35qta530.h"

#define TTGL_XRES ttgfb_lcdspecs->x_res
#define TTGL_YRES ttgfb_lcdspecs->y_res
#define TTGL_XRESV TTGL_XRES
#define TTGL_YRESV (TTGL_YRES * TTGL_PAGES) /* For page flipping */
#define TTGL_BPP    16
#define TTGL_BYPP   2
#define TTGL_LINE   (TTGL_XRES * TTGL_BYPP)
#define TTGL_SIZE   (TTGL_YRES * TTGL_XRES * TTGL_BYPP)
#define TTGL_WIDTH  53
#define TTGL_HEIGHT 71
#define TTGL_R_OFS  11
#define TTGL_R_LEN  5
#define TTGL_G_OFS  5
#define TTGL_G_LEN  6
#define TTGL_B_OFS  0
#define TTGL_B_LEN  5
#define TTGL_T_OFS  0
#define TTGL_T_LEN  0
#define TTGL_PAGES  2
#define TTGL_MAX_LAYERS 2

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
#include <barcelona/cpufreq_order.h>

static struct lcd_screen_info ttgfb_lcdspecs_unknown = {"UNK", "UNKNWN320_240", 0, 0, 240, 320, 25, 60, NULL, NULL, NULL, PNR_TFT, BPP_16BPP_TFT, 133000000, {{0, 0, 0, 0, 0, 0}}, lcd_def_freq_policy, lcd_def_freq_trans};
#else
static struct lcd_screen_info ttgfb_lcdspecs_unknown = {"UNK", "UNKNWN320_240", 0, 0, 240, 320, 25, 60, NULL, NULL, NULL, PNR_TFT, BPP_16BPP_TFT, 133000000, {{0, 0, 0, 0, 0, 0}}};
#endif
static struct lcd_screen_info *ttgfb_lcdspecs = &ttgfb_lcdspecs_unknown;
static int    ttgfb_lcdon = 1;

static struct ttgfb_slot fbs[TTGL_MAX_LAYERS];
static struct resource *lcd_mem;
static struct clk *lcd_clock;

/* forward declarations, needed for fb_blank() hook */
static void tomtomgo_lcdon(void);
static void tomtomgo_lcdoff(void);

struct lcd_controller *ttgfb_lcd_controller = &s3c24xx_lcd_controller;

unsigned ttgfb_get_base(void)
{
	return ttgfb_lcd_controller->get_base(0);
}

void ttgfb_startlcd(void)
{
	// Enable timing signals
	IO_Activate(LCD_OEN);
	IO_SetFunction(VDEN);
	IO_SetFunction(VCLK);
	IO_SetFunction(HSYNC);
	IO_SetFunction(VSYNC);

	// Start driving LCD data
	IO_SetFunction(VD0);
	IO_SetFunction(VD1);
	IO_SetFunction(VD2);
	IO_SetFunction(VD3);
	IO_SetFunction(VD4);
	IO_SetFunction(VD5);
	IO_SetFunction(VD6);
	IO_SetFunction(VD7);
	IO_SetFunction(VD8);
	IO_SetFunction(VD9);
	IO_SetFunction(VD10);
	IO_SetFunction(VD11);
	IO_SetFunction(VD12);
	IO_SetFunction(VD13);
	IO_SetFunction(VD14);
	IO_SetFunction(VD15);
	IO_SetFunction(VD18);
	IO_SetFunction(VD19);
	IO_SetFunction(VD20);
	IO_SetFunction(VD21);
	IO_SetFunction(VD22);
	IO_SetFunction(VD23);
}

void ttgfb_stoplcd(void)
{
	// Disable timing signals
	IO_Deactivate(LCD_OEN);
	IO_Deactivate(VDEN);
	IO_Deactivate(VCLK);
	IO_Deactivate(HSYNC);
	IO_Deactivate(VSYNC);

	// Stop driving LCD data
	IO_Deactivate(VD0);
	IO_Deactivate(VD1);
	IO_Deactivate(VD2);
	IO_Deactivate(VD3);
	IO_Deactivate(VD4);
	IO_Deactivate(VD5);
	IO_Deactivate(VD6);
	IO_Deactivate(VD7);
	IO_Deactivate(VD8);
	IO_Deactivate(VD9);
	IO_Deactivate(VD10);
	IO_Deactivate(VD11);
	IO_Deactivate(VD12);
	IO_Deactivate(VD13);
	IO_Deactivate(VD14);
	IO_Deactivate(VD15);
	IO_Deactivate(VD18);
	IO_Deactivate(VD19);
	IO_Deactivate(VD20);
	IO_Deactivate(VD21);
	IO_Deactivate(VD22);
	IO_Deactivate(VD23);
}

/*
 *	s3c2410fb_check_var():
 *	Get the video params out of 'var'. If a value doesn't fit, round it up,
 *	if it's too big, return -EINVAL.
 *
 *	Round up in the following order: bits_per_pixel, xres,#define TTGL_PAGES  2
 *	yres, xres_virtual, yres_virtual, xoffset, yoffset, grayscale,
 *	bitfields, horizontal timing, vertical timing.
 */
static int ttgfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	PK_DBG("check_var(var=%p, info=%p)\n", var, info);

	if (var->xres > TTGL_XRES)		return -EINVAL;
	if (var->yres > TTGL_YRES)		return -EINVAL;

	if (var->xres_virtual > TTGL_XRESV)	return -EINVAL;
	if (var->yres_virtual > TTGL_YRESV)	return -EINVAL;

	if (var->xoffset != 0)			return -EINVAL;
	if (var->yoffset > TTGL_YRES)		return -EINVAL;

	if (var->bits_per_pixel != TTGL_BPP)	return -EINVAL;
	if (var->grayscale != 0)		return -EINVAL;

	if (var->red.offset   != TTGL_R_OFS)	return -EINVAL;
	if (var->green.offset != TTGL_G_OFS)	return -EINVAL;
	if (var->blue.offset  != TTGL_B_OFS)	return -EINVAL;

	if (var->red.length   != TTGL_R_LEN)	return -EINVAL;
	if (var->green.length != TTGL_G_LEN)	return -EINVAL;
	if (var->blue.length  != TTGL_B_LEN)	return -EINVAL;

	return 0;
}

/**
 *      ttgfb_set_par - Optional function. Alters the hardware state.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 */
static int ttgfb_set_par(struct fb_info *info)
{
	PK_DBG("set_par(info=%p)\n", info);
	return 0;
}


static int ttgfb_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue, unsigned transp, struct fb_info *info)
{
	struct ttgfb_slot *slot = (struct ttgfb_slot *) info;
	unsigned bpp, m;

	bpp = info->var.bits_per_pixel;
	m = 1 << bpp;
	if (regno >= m) {
		return -EINVAL;
	}

	switch (bpp) {
	case 16:
		/* RGB 565 */
		slot->pseudo_pal[regno] = ((red & 0xF800)
			| ((green & 0xFC00) >> 5)
			| ((blue & 0xF800) >> 11));
		break;
	}

	return 0;
}


/**
 *	ttgfb_pan_display - NOT a required function. Pans the display.
 *	@var: frame buffer variable screen structure
 *	@info: frame buffer structure that represents a single frame buffer
 *
 *	Pan (or wrap, depending on the `vmode' field) the display using the
 *	`xoffset' and `yoffset' fields of the `var' structure.
 *	If the values don't fit, return -EINVAL.
 *
 *	Returns negative errno on error, or zero on success.
 */
static int ttgfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	PK_DBG("xoffset=%u, yoffset=%u\n", var->xoffset, var->yoffset);
	if (var->xoffset != 0)
		return -EINVAL;

	if(info == &(fbs[0].fbi)) {
		fbs[0].fbi.var.yoffset = var->yoffset;

		ttgfb_lcd_controller->update_base(0, ttgfb_lcdspecs, &(fbs[0].fbi), &(fbs[0].window));
	} else {
		fbs[1].fbi.var.yoffset = var->yoffset;

		ttgfb_lcd_controller->update_base(1, ttgfb_lcdspecs, &(fbs[1].fbi), &(fbs[1].window));
	}
	return 0;
}


/**
 *      xxxfb_blank - NOT a required function. Blanks the display.
 *	@blank_mode: the blank mode we want.
 *	@info: frame buffer structure that represents a single frame buffer
 *
 *	Blank the screen if blank_mode != 0, else unblank. Return 0 if
 *	blanking succeeded, != 0 if un-/blanking failed due to e.g. a
 *	video mode which doesn't support it. Implements VESA suspend
 *	and powerdown modes on hardware that supports disabling hsync/vsync:
 *	blank_mode == 2: suspend vsync
 *	blank_mode == 3: suspend hsync
 *	blank_mode == 4: powerdown
 *
 *	Returns negative errno on error, or zero on success.
 *
 */
static int ttgfb_blank(int blank_mode, struct fb_info *info)
{
	/* For now, we only support unblank & powerdown... */
	switch(blank_mode) {
		/* screen: unblanked, hsync: on,  vsync: on */
		case FB_BLANK_UNBLANK:
			tomtomgo_lcdon();
			break;

		/* screen: blanked,   hsync: on,  vsync: on */
		case FB_BLANK_NORMAL:
		/* screen: blanked,   hsync: on,  vsync: off */
		case FB_BLANK_VSYNC_SUSPEND:
		/* screen: blanked,   hsync: off, vsync: on */
		case FB_BLANK_HSYNC_SUSPEND:
			return -EINVAL;

		/* screen: blanked,   hsync: off, vsync: off */
		case FB_BLANK_POWERDOWN:
			tomtomgo_lcdoff();
			break;
	}

	return 0;
}

/**
 *	xxxfb_screensave - NOT a required function. Selects TFT powerdown mode while retaining display content
 *	@info: the blank mode we want.
 *	@info: frame buffer structure that represents a single frame buffer
 *
 *	Returns negative errno on error, or zero on success.
 *
 */

static int 
ttgfb_screensave(int mode, struct fb_info *info)
{	
	switch(mode) {
		/* screensave mode for some LCD panels */
		case FB_SCREENSAVE_START:
			if (ttgfb_lcdspecs->Screensave) {
				ttgfb_lcdspecs->Screensave(1);
			}
			break;
		case FB_SCREENSAVE_STOP:
			if (ttgfb_lcdspecs->Screensave) {
				ttgfb_lcdspecs->Screensave(0);
			}
			break;
	}
	return 0;
}

void ttgfb_set_window(int layer, struct ttgfb_window *win)
{
	win->src_left     = win->src_left & 0xfffc;
	win->src_top      = win->src_top  & 0xfffc;
	win->dst_left     = win->dst_left & 0xfffc;
	win->dst_top      = win->dst_top  & 0xfffc;
	win->height       = (win->height + 3) & 0xffc;
	win->width        = (win->width + 3) & 0xffc;
	win->layer_width  = (win->layer_width + 3) & 0xffc;	

	fbs[layer].window = *win;
	
	ttgfb_lcd_controller->update_base(layer, ttgfb_lcdspecs, &(fbs[layer].fbi), win);
}

static int ttgfb_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg, struct fb_info *info)
{
	switch(cmd) {
	case TTGFB_IOCTL_GET_WIN:
		{
			int layer;
			
			if(info == &(fbs[0].fbi))
				layer = 0;
			else
				layer = 1;
			
			if(copy_to_user((void *)arg, &(fbs[layer].window) , sizeof(struct ttgfb_window)))
				return -EFAULT;
		}
		break;
	case TTGFB_IOCTL_SET_WIN:
		{
			struct ttgfb_window win;
			
			if(copy_from_user(&win, (void *)arg, sizeof(struct ttgfb_window)))
				return -EFAULT;
			
			if(win.layer_width == 0)
				win.layer_width = TTGL_XRES;
			
			if ((win.src_left >= TTGL_XRES)|| (win.src_left + win.width > TTGL_XRES)  || (win.dst_left >= TTGL_XRES)|| (win.dst_left + win.width > TTGL_XRES) || (win.width  > TTGL_XRES) ||
			    (win.src_top  >= TTGL_YRES)|| (win.src_top + win.height > TTGL_YRES)  || (win.dst_top  >= TTGL_YRES)|| (win.dst_top + win.height > TTGL_YRES) || (win.height > TTGL_YRES) ||
			    (win.width > win.layer_width) || (win.layer_width > TTGL_XRES))
				return -EINVAL;
			
			if(info == &(fbs[0].fbi)) {
				ttgfb_set_window(0, &win);
			} else {
				ttgfb_set_window(1, &win);
			}
		}
		break;

	case TTGFB_IOCTL_GET_LCDON:
		{
			if ( put_user(ttgfb_lcdon, (int __user *)arg) )
			return -EFAULT;
		}
		break;

	default:
		return ttgfb_lcd_controller->ioctl(cmd, arg, info);
	}
	return 0;
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
#define TTGFB_FB_OPEN		0x00000001
static unsigned long int	ttgfb_flags = 0;

/* Open and close routine to keep track of when the framebuffer is opened and when not. */
static int ttgfb_open( struct fb_info *info, int user )
{
	unsigned short		max_prescaler=(ttgfb_lcdspecs->pnr_mode == PNR_TFT ? 2048 : 1024);
	unsigned short		min_prescaler=2;
	__u32			hclk_min=get_hclkrate( ttgfb_lcdspecs, ttgfb_lcdspecs->refresh_min, min_prescaler );
	__u32			hclk_max=get_hclkrate( ttgfb_lcdspecs, ttgfb_lcdspecs->refresh_max, max_prescaler );

	/* Check if device is opened. */
	if( (ttgfb_flags & TTGFB_FB_OPEN) == 0 )
	{
		/* Check if the HCLK is within legal limits. */
		if( (ttgfb_lcdspecs->curr_hclk < hclk_min) || (ttgfb_lcdspecs->curr_hclk > hclk_max) )
			return -ENODEV;

		/* Init the display. This will set the prescaler also. */
		tomtomgo_lcdon( );

		/* Flag opened. */
		ttgfb_flags|=TTGFB_FB_OPEN;
	}
	else
	{
		/* Already opened. */
		return -EACCES;
	}

	/* No problems. */
	return 0;
}

static int ttgfb_close( struct fb_info *info, int user )
{
	if( ttgfb_flags & TTGFB_FB_OPEN )
	{
		/* Flag closed. */
		ttgfb_flags&=~(TTGFB_FB_OPEN);

		/* Stop the display */
		tomtomgo_lcdoff();
		return 0;
	}
	else
	{
		/* Already closed. */
		return -EACCES;
	}
}

#endif
static struct fb_ops ttgfb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= ttgfb_check_var,
	.fb_set_par	= ttgfb_set_par,
	.fb_screensave	= ttgfb_screensave,
	.fb_blank	= ttgfb_blank,
	.fb_pan_display	= ttgfb_pan_display,
	.fb_setcolreg	= ttgfb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_cursor	= soft_cursor,
	.fb_ioctl       = ttgfb_ioctl,
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	.fb_open	= ttgfb_open,		/* Open routine. */
	.fb_release	= ttgfb_close,		/* Close routine. */
#endif
};

static void ttgfb_setup(void)
{
	switch (IO_GetTftType()) {
	case GOTFT_NEC_NL2432HC22:
		ttgfb_lcdspecs=&ttgfb_nl2432hc22_lcdspecs;
		break;
	case GOTFT_SAMSUNG_LTV350:
		ttgfb_lcdspecs=&ttgfb_ltv350qv_lcdspecs;
		break;
	case GOTFT_SAMSUNG_LTE246QV:
		ttgfb_lcdspecs=&ttgfb_lte246qv_lcdspecs;
		break;
	case GOTFT_SAMSUNG_LTP400:
		ttgfb_lcdspecs=&ttgfb_ltp400wq_lcdspecs;
		break;
	case GOTFT_SAMSUNG_LTE430WQ:
		ttgfb_lcdspecs=&ttgfb_lte430wq_lcdspecs;
		break;
	case GOTFT_SHARP_LQ043T1:
		ttgfb_lcdspecs=&ttgfb_lq043t1_lcdspecs;
		break;
	case GOTFT_SAMSUNG_LMS350GF:
		ttgfb_lcdspecs=&ttgfb_lms350gf_lcdspecs;
		break;
	case GOTFT_SAMSUNG_LMS350GF20:
		ttgfb_lcdspecs=&ttgfb_lms350gf20_lcdspecs;
		break;
	case GOTFT_SHARP_LQ035Q1DG:
		ttgfb_lcdspecs=&ttgfb_lq035q1dg_lcdspecs;
		break;
	case GOTFT_SAMSUNG_LMS430HF12:
	case GOTFT_SAMSUNG_LMS430HF19: /* compatible wit HF12 */
	case GOTFT_SAMSUNG_LMS430HF29:
		ttgfb_lcdspecs=&ttgfb_lms430hf12_lcdspecs; /* made compatible with HF29 */
		break;	
	case GOTFT_SHARP_LQ035Q1DG04:
		ttgfb_lcdspecs=&ttgfb_lq035q1dg04_lcdspecs;
		break;		
	case GOTFT_AUO_A035QN02:
		ttgfb_lcdspecs=&ttgfb_a035qn02_lcdspecs;
		break;
	case GOTFT_AUO_A043FW03V0:
	case GOTFT_AUO_A043FW03V1:
	case GOTFT_AUO_A043FW05V1:
		ttgfb_lcdspecs=&ttgfb_a043fw03_lcdspecs; /* made compatible with FW03V1 */
		break;
	case GOTFT_SHARP_LQ043T3DW01:
		ttgfb_lcdspecs=&ttgfb_lq043t3dw01_lcdspecs;
		break;			
	case GOTFT_SHARP_LQ043T3DW02:
		ttgfb_lcdspecs=&ttgfb_lq043t3dw02_lcdspecs;
		break;
	case GOTFT_AUO_A050FW02V2:    	        
		ttgfb_lcdspecs=&ttgfb_a050fw02v2_lcdspecs;
		break;		
	case GOTFT_AUO_A050FW03V2:    	        
		ttgfb_lcdspecs=&ttgfb_a050fw03v2_lcdspecs;
		break;				
	case GOTFT_SAMSUNG_LMS500HF01:
	case GOTFT_SAMSUNG_LMS500HF05:
		ttgfb_lcdspecs=&ttgfb_lms500hf01_lcdspecs;
		break;
	case GOTFT_LG_LD050WQ1:
		ttgfb_lcdspecs=&ttgfb_ld050wq1_lcdspecs;
		break;
	case GOTFT_LG_LB043WQ3:
		ttgfb_lcdspecs=&ttgfb_lb043wq3_lcdspecs;
		break;		
	case GOTFT_WISTRON_T35QTA530:
		ttgfb_lcdspecs=&ttgfb_t35qta530_lcdspecs;
		break;		
	default:
		PK_WARN("Unknown TFT type %u, assuming 240x320 resolution.\n", IO_GetTftType());
		ttgfb_lcdspecs = &ttgfb_lcdspecs_unknown;
		break;
	}

	/* Fill in the current clock rate. This since the correct clockrate is not known upon structure initialization. */
	ttgfb_lcdspecs->curr_hclk=get_hclkfreq( );
	if( ttgfb_lcdspecs->curr_hclk == 0 )
	{
		PK_WARN( "Could not get current hclk rate. Assuming 133MHz.\n" );
		ttgfb_lcdspecs->curr_hclk=133000000;
	}
}

static void ttgfb_setup_fb(struct ttgfb_slot *fbs, int layer)
{
	struct fb_info *fbi= &fbs->fbi;
	struct fb_monspecs ttgfb_monspecs;
	const char *id = ttgfb_lcd_controller->get_id();
	
	fbs->window.src_left     = 0;
	fbs->window.src_top      = 0;
	fbs->window.dst_left     = 0;
	fbs->window.dst_top      = 0;
	fbs->window.width        = TTGL_XRES;
	fbs->window.height       = TTGL_YRES;
	fbs->window.layer_width  = TTGL_XRES;

	memset(&fbi->var, 0, sizeof fbi->var);
	fbi->var.xres           = TTGL_XRES;
	fbi->var.yres           = TTGL_YRES;
	fbi->var.xres_virtual   = TTGL_XRESV;
	fbi->var.yres_virtual   = TTGL_YRESV;
	fbi->var.bits_per_pixel = TTGL_BPP;
	fbi->var.red.offset     = TTGL_R_OFS;
	fbi->var.red.length     = TTGL_R_LEN;
	fbi->var.green.offset   = TTGL_G_OFS;
	fbi->var.green.length   = TTGL_G_LEN;
	fbi->var.blue.offset    = TTGL_B_OFS;
	fbi->var.blue.length    = TTGL_B_LEN;
	fbi->var.transp.offset  = TTGL_T_OFS;
	fbi->var.transp.length  = TTGL_T_LEN;
	fbi->var.activate       = FB_ACTIVATE_NOW;
	fbi->var.height         = TTGL_HEIGHT;
	fbi->var.width          = TTGL_WIDTH;
	fbi->var.vmode          = FB_VMODE_NONINTERLACED;

	memset(&fbi->fix, 0, sizeof fbi->fix);
	strcpy(fbi->fix.id, id);

	fbi->fix.smem_len       = TTGL_YRESV * TTGL_XRESV * TTGL_BYPP;
	fbi->fix.type           = FB_TYPE_PACKED_PIXELS;
	fbi->fix.visual         = FB_VISUAL_TRUECOLOR;
	fbi->fix.ypanstep       = 1;
	fbi->fix.line_length    = TTGL_LINE;
	fbi->fix.accel          = FB_ACCEL_NONE;

	fbi->flags              = FBINFO_FLAG_DEFAULT;
	
	fbi->fbops              = &ttgfb_ops;
	fbi->pseudo_palette     = fbs->pseudo_pal;
	
	/* Fill in the monitorspecs fields. */
	strcpy( ttgfb_monspecs.manufacturer, ttgfb_lcdspecs->manufacturer );
	strcpy( ttgfb_monspecs.monitor, ttgfb_lcdspecs->lcd );
	ttgfb_monspecs.hfmin=ttgfb_lcdspecs->x_res * ttgfb_lcdspecs->refresh_min;
	ttgfb_monspecs.hfmax=ttgfb_lcdspecs->x_res * ttgfb_lcdspecs->refresh_max;
	ttgfb_monspecs.dclkmin=ttgfb_lcdspecs->x_res * ttgfb_lcdspecs->y_res * ttgfb_lcdspecs->refresh_min;
	ttgfb_monspecs.dclkmax=ttgfb_lcdspecs->x_res * ttgfb_lcdspecs->y_res * ttgfb_lcdspecs->refresh_max;
	ttgfb_monspecs.vfmin=ttgfb_lcdspecs->refresh_min;
	ttgfb_monspecs.vfmax=ttgfb_lcdspecs->refresh_max;
	ttgfb_monspecs.max_x=ttgfb_lcdspecs->x_width/10;
	ttgfb_monspecs.max_y=ttgfb_lcdspecs->y_width/10;
	
	fbi->monspecs           = ttgfb_monspecs;
}

static int ttgfb_alloc_fb(struct ttgfb_slot *fbs, int layer)
{
	fbs->map_size = PAGE_ALIGN(fbs->fbi.fix.smem_len);

	/* Check if the lfb_size is specified and the layer is 0. If so, we can just map this range. It's reserved. */
	if ((layer != 0) || (screen_info.lfb_size == 0)) {
		PK_DBG("layer %d, lfb_size %u, using dma_alloc_writecombine(%u)\n",
		        layer, screen_info.lfb_size, fbs->map_size);
		
		fbs->map_cpu = dma_alloc_writecombine(fbs->dev, fbs->map_size, &fbs->map_dma, GFP_KERNEL);
	} else {
		PK_DBG("layer %d, lfb_size %u, using ioremap(%#x, %u)\n", 
		       layer, screen_info.lfb_size, screen_info.lfb_base, fbs->map_size);
		
		fbs->map_cpu = ioremap(screen_info.lfb_base, fbs->map_size);
		fbs->map_dma = screen_info.lfb_base;
	}
	PK_DBG("map_cpu=%#x, map_dma=%#x\n", (unsigned) fbs->map_cpu, fbs->map_dma);

	if (!fbs->map_cpu)
		return -ENOMEM;

	fbs->fbi.fix.smem_start = fbs->map_dma;
	fbs->fbi.screen_base    = fbs->map_cpu;
	fbs->fbi.screen_size    = fbs->map_size;

	if (!screen_info.lfb_base) {
		/* If no LFB from bootloader, just clear it out. */
		PK_INFO("No LFB from bootloader, clearing screen\n");
		memset(fbs->map_cpu, 0, fbs->map_size);
	} else {
		/* Check whether or not the lfb_size is specified. If so, we can just use the buffer as it has been reserved. */
		if ((layer != 0) || (screen_info.lfb_size == 0)) {
			/* lfb_size is not specified. Do things the old way. */
			/* If bootloader supplies LFB, try to copy it. */
			void __iomem *orig_lfb = ioremap(screen_info.lfb_base, fbs->map_size);
			if (!orig_lfb) {
				PK_INFO("Unable to map bootloader LFB, clearing screen\n");
				memset(fbs->map_cpu, 0, fbs->map_size);
			} else {
				u_char *temp;
				PK_INFO("Copying bootloader LFB to all pages\n");
				for (temp = fbs->map_cpu; temp < fbs->map_cpu + fbs->map_size; temp += fbs->map_size / TTGL_PAGES)
					memcpy(temp, orig_lfb, fbs->map_size / TTGL_PAGES);
				iounmap(orig_lfb);
			}
		} else {
			/* Memory is reserved. We don't have to copy we can just use. */
			PK_INFO("Using LFB from bootloader (reserved memory).\n");
			
		}
	}
	return 0;
}

static void ttgfb_free_fb(struct ttgfb_slot *fbs, int layer)
{
	if (fbs->map_cpu) {
		/* Check whether the memory is mapped with ioremap, or if it's allocated. */
		if ((layer != 0) || (screen_info.lfb_size <= fbs->map_size))
			dma_free_writecombine(fbs->dev, fbs->map_size, fbs->map_cpu, fbs->map_dma);
		else
			iounmap(fbs->map_cpu);
		fbs->map_dma = 0;
		fbs->map_cpu = NULL;
	}
}

static void tomtomgo_lcdoff(void)
{
	// Only turn off LCD when it's on
	if (ttgfb_lcdon != 0)
	{
		ttgfb_lcdon = 0;

		// Disable backlight here now to avoid white FLASH
#if defined CONFIG_BARCELONA_PWM || defined CONFIG_BARCELONA_PWM_MODULE
		pwm_disable();
#endif
		IO_Deactivate(BACKLIGHT_EN);
		IO_Deactivate(BACKLIGHT_PWM);

		// Wait 10 ms to let the backlight chip switch off
		mdelay(10);

		// Show black screen
		ttgfb_lcd_controller->show_color_screen(0);

		mdelay(100);

		if (ttgfb_lcdspecs->Off) ttgfb_lcdspecs->Off();
			
		// Disable timing signals + stop driving LCD data
		ttgfb_stoplcd();

		// Turn charge pump off
		IO_Deactivate(LCD_BIAS_PWREN);

		// Wait 50 ms
		mdelay(50);

		// Disable LCD power
		IO_Deactivate(LCD_VCC_PWREN);

		// Wait 50 ms
		mdelay(50);

		clk_disable(lcd_clock);
	}
}

void tomtomgo_lcdclearcolor(unsigned short color)
{	
	/* assume 16 bit color */
	int i;
	unsigned short *p = (unsigned short*)fbs[0].map_cpu;
	unsigned int size_short = fbs[0].map_size / sizeof(unsigned short);
	for (i=0; i<size_short; i++) {
		*p++ = color;
	}
}

static void tomtomgo_lcdon(void)
{
	int i, layers;
	clk_enable(lcd_clock);

	// Set base frameaddress
	layers = ttgfb_lcd_controller->get_number_of_layers();
	
	for(i = 0; i < layers; i++)
	{
		ttgfb_lcd_controller->update_base(i, ttgfb_lcdspecs, &(fbs[i].fbi), &fbs[i].window);
	}

	if (ttgfb_lcdon == 0)
	{
		ttgfb_lcd_controller->show_color_screen(0x000000);
	
		// Do softstart on certain LCD's
		if (IO_HaveTftSoftStart()) IO_GeneratePWM(LCD_VCC_PWREN);
		
		if (IO_GetTftType() == GOTFT_SHARP_LQ035Q1DG) {
			IO_Deactivate(LCD_RESET);	// set SHUT to high
			mdelay(1);
		}
		
		/* Enable LCD power */
		IO_Activate(LCD_VCC_PWREN);

		if (IO_HasPin(LCD_BIAS_PWREN)) {
			// Wait 50 ms till power is stable
			mdelay(50);

			// Turn charge pump on
			IO_Activate(LCD_BIAS_PWREN);
		
			// Wait 50 ms till bias power is stable
			mdelay(50);
		}
		
		// Enable timing signals
		ttgfb_startlcd();

		// Do panel dep. controller init
		if (ttgfb_lcdspecs->Init) ttgfb_lcdspecs->Init();

		mdelay(100);
		// Show normal screen
		ttgfb_lcd_controller->show_normal_screen();
	}
	ttgfb_lcdon = 1;
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
/* Calculate the prescaler value needed for the frequency specified in the lcd_screen_info structure. */
unsigned short find_prescaler( struct lcd_screen_info *lcd_info, unsigned long int hclk_rate )
{
	unsigned short		count=0;

	/* Search until we find the closest match. */
	while( get_framerate( lcd_info, count, hclk_rate ) > lcd_info->refresh_max )
	{
		count+=1;
	}

	/* Return the result. */
	return count;
}

/*
 * CPU clock speed change handler. Change LCD timing / framerate.
 */

int lcd_def_freq_trans(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs	*f = data;
	struct lcd_screen_info	*lcd_info=container_of( nb, struct lcd_screen_info, freq_transition );  
	unsigned short		max_prescaler=(lcd_info->pnr_mode == PNR_TFT ? 2048 : 1024);
	unsigned short		min_prescaler=2;
	__u32			hclk_min=get_hclkrate( lcd_info, lcd_info->refresh_min, min_prescaler );
	__u32			hclk_max=get_hclkrate( lcd_info, lcd_info->refresh_max, max_prescaler );
	unsigned long int	hclk_old;
	unsigned long int	hclk_new;
	unsigned short		new_prescaler;

	f->trans2hclk( f, &hclk_old, &hclk_new );

	new_prescaler=find_prescaler( lcd_info, hclk_new*1000 );

	/* If the device ain't open, no need to worry. */
	switch (val)
	{
		case CPUFREQ_PRECHANGE:
		{
			/* Make sure that the display is turned off if it goes out of bounds. */
			if( (ttgfb_flags & TTGFB_FB_OPEN) && (((hclk_new * 1000) > hclk_max) || ((hclk_new * 1000) < hclk_min)) )
			{
				/* If the old clock was within bounds.... */
				if( ((hclk_old * 1000) < hclk_max) && ((hclk_old * 1000) > hclk_min))
				{
					/* Clock is going out of bounds. Shut down the display. */
					tomtomgo_lcdoff( );
				}
			}
			else
			{
				/* Ensure we don't overclock the LCD display. We do this by making sure it can never exceed */
				/* the maximum clock. */
				if( hclk_new > hclk_old )
				{
					/* Try to find the prescaler and set it. */
					if( new_prescaler > 1023 )
					{
						printk( "ttgfb: WARNING! Can't go above maximum clock. Disabling LCD.\n" );
						if( ttgfb_flags & TTGFB_FB_OPEN ) tomtomgo_lcdoff( );
					}
					else
					{
						/* Check if the framerate goes below the minimum. */
						if( get_framerate( lcd_info, new_prescaler, hclk_new*1000 ) < lcd_info->refresh_min )
							printk( "ttgfb: WARNING! Selected clock is below minimum!\n" );

						/* Set the prescaler. */
						set_clkval( lcd_info, new_prescaler );
					}
				}
			}
			break;
		}

		case CPUFREQ_POSTCHANGE:
		{
			/* Check if the old clock was invalid but the new one is valid. If so, we need to restart the LCD. */
			if( (ttgfb_flags & TTGFB_FB_OPEN) && (((hclk_old * 1000) > hclk_max) || ((hclk_old * 1000) < hclk_min)) )
			{
				/* Is the next clock valid ? */
				if( ((hclk_new * 1000) < hclk_max) && ((hclk_new * 1000) > hclk_min) )
				{
					tomtomgo_lcdon( );
				}
			}
			else
			{
				/* Clock changed. Check if it goes from high to low. If so, we need to set it here. */
				if( hclk_old > hclk_new )
				{
					/* Try to find the prescaler and set it. */
					if( new_prescaler > 1023 )
					{
						printk( "ttgfb: WARNING! Can't go above maximum clock. Disabling LCD.\n" );
						if( ttgfb_flags & TTGFB_FB_OPEN ) tomtomgo_lcdoff( );
					}
					else
					{
						/* Check if the framerate goes below the minimum. */
						if( get_framerate( lcd_info, new_prescaler, hclk_new*1000 ) < lcd_info->refresh_min )
							printk( "ttgfb: WARNING! Selected clock is below minimum!\n" );

						/* Set the prescaler. */
						set_clkval( lcd_info, new_prescaler );
					}
				}
			}

			/* Save the current HCLK. */
			lcd_info->curr_hclk=hclk_new * 1000;
			break;
		}
	}
	return 0;
}

int lcd_def_freq_policy( struct notifier_block *nb, unsigned long val, void *data )
{
	struct cpufreq_policy	*policy = data;
	struct lcd_screen_info	*lcd_info=container_of( nb, struct lcd_screen_info, freq_policy );  
	unsigned short		max_prescaler=(lcd_info->pnr_mode == PNR_TFT ? 2048 : 1024);
	unsigned short		min_prescaler=2;
	__u32			hclk_min=get_hclkrate( lcd_info, lcd_info->refresh_min, min_prescaler );
	__u32			hclk_max=get_hclkrate( lcd_info, lcd_info->refresh_max, max_prescaler );
	unsigned long int	low_hclk;
	unsigned long int	high_hclk;

	policy->policy2hclk( policy, &low_hclk, &high_hclk );

	switch (val) {
	case CPUFREQ_ADJUST:
		if( ttgfb_flags & TTGFB_FB_OPEN )
			policy->hclk2policy( policy, hclk_min, hclk_max );
		break;
	case CPUFREQ_INCOMPATIBLE:
		if( ttgfb_flags & TTGFB_FB_OPEN )
		{
			if( (low_hclk >= hclk_min) && (low_hclk <= hclk_max) )
				hclk_min=low_hclk;

			if( (high_hclk >= hclk_min) && (high_hclk <= hclk_max) )
				hclk_max=high_hclk;

			if( (high_hclk != hclk_max) || (low_hclk != hclk_min) )
				policy->hclk2policy( policy, hclk_min, hclk_max );
		}
		break;
	case CPUFREQ_NOTIFY:
		/* Illegal values are handled in the transition notifier. */
		break;
	}
	return 0;
}
#endif

static int ttgfb_probe(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct resource		*res;
	int			size;
	int 			layers, i;

	PK_DBG("probe=%p, device=%p\n", pdev, dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		PK_ERR("failed to get memory region resouce\n");
		return -ENOENT;
	}

	size = (res->end-res->start)+1;
	lcd_mem = request_mem_region(res->start, size, pdev->name);
	if (lcd_mem == NULL) {
		PK_ERR("failed to get memory region\n");
		return -ENOENT;
	}

	lcd_clock = clk_get(dev, "lcd");
	if (IS_ERR(lcd_clock)) {
		PK_ERR("failed to find lcd clock source\n");
		return -ENOENT;
	}

	// make sure dotclk change functionpointers are initialised before getting clock
	clk_use(lcd_clock);
	clk_enable(lcd_clock);

	ttgfb_setup();
	
	layers = ttgfb_lcd_controller->get_number_of_layers();
	
	for(i = 0; i< layers; i++) {
		ttgfb_setup_fb(&(fbs[i]), i);
		ttgfb_alloc_fb(&(fbs[i]), i);
		register_framebuffer(&(fbs[i].fbi));
	}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	/* Register the cpu notifier (for scaling) */
	memset( &ttgfb_lcdspecs->freq_transition, 0, sizeof( ttgfb_lcdspecs->freq_transition ) );
	memset( &ttgfb_lcdspecs->freq_policy, 0, sizeof( ttgfb_lcdspecs->freq_policy ) );
	ttgfb_lcdspecs->freq_transition.notifier_call = ttgfb_lcdspecs->lcd_freq_transition_handler;
	ttgfb_lcdspecs->freq_transition.priority = CPUFREQ_ORDER_S3C24XX_TTGFB_PRIO;
	ttgfb_lcdspecs->freq_policy.notifier_call = ttgfb_lcdspecs->lcd_freq_policy_handler;
	cpufreq_register_notifier(&ttgfb_lcdspecs->freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_register_notifier(&ttgfb_lcdspecs->freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

	/* Init the display */
	tomtomgo_lcdon();

	return 0;
}

static int ttgfb_remove(struct device *dev)
{
	int layers, i;
	
	PK_DBG("device=%p\n", dev);

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	cpufreq_unregister_notifier(&ttgfb_lcdspecs->freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_unregister_notifier(&ttgfb_lcdspecs->freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

	tomtomgo_lcdoff();
	
	layers = ttgfb_lcd_controller->get_number_of_layers();
	
	for(i = 0; i < layers; i++)
	{
		unregister_framebuffer(&(fbs[i].fbi));
		ttgfb_free_fb(&(fbs[i]), i);
	}
	
	if (lcd_clock != NULL) {
		clk_disable(lcd_clock);
		clk_unuse(lcd_clock);
		clk_put(lcd_clock);
		lcd_clock = NULL;
	}
	if (lcd_mem != NULL) {
		release_resource(lcd_mem);
		kfree(lcd_mem);
		lcd_mem = NULL;
	}

	return 0;
}

static void ttgfb_shutdown(struct device *dev)
{
	tomtomgo_lcdoff();
}

#ifdef CONFIG_PM

static int ttgfb_suspend(struct device *dev, u32 state, u32 level)
{
	PK_DBG("%s: dev = %p, state = %u, level = %u\n", __func__, dev, state, level);
	if (level == SUSPEND_POWER_DOWN) {
		tomtomgo_lcdoff();
	}
	return 0;
}

static int ttgfb_resume(struct device *dev, u32 level)
{
	PK_DBG("%s: dev = %p, level = %u\n", __func__, dev, level);
	if (level == RESUME_POWER_ON) 
	{
		tomtomgo_lcdon();
		ttgfb_lcd_controller->resume();
	}
	return 0;
}

#else /* CONFIG_PM */
#define ttgfb_suspend NULL
#define ttgfb_resume  NULL
#endif /* CONFIG_PM */

static struct device_driver ttgfb_s3c2410_driver = {
	.name		= "s3c2410-lcd",
	.bus		= &platform_bus_type,
	.probe		= ttgfb_probe,
	.remove		= ttgfb_remove,
	.shutdown	= ttgfb_shutdown,
	.suspend	= ttgfb_suspend,
	.resume		= ttgfb_resume,
};

static struct device_driver ttgfb_s3c2443_tft_driver = {
	.name		= "s3c2443-tft-lcd",
	.bus		= &platform_bus_type,
	.probe		= ttgfb_probe,
	.remove		= ttgfb_remove,
	.shutdown	= ttgfb_shutdown,
	.suspend	= ttgfb_suspend,
	.resume		= ttgfb_resume,
};

static int __init ttgfb_init(void) {
	int result=0;

	if (IO_GetCpuType() == GOCPU_S3C2450 ) {
		ttgfb_lcd_controller = &s3c2443_tft_controller;
		PK_INFO("Using s3c2443 tft lcd controller\n");
		result = driver_register(&ttgfb_s3c2443_tft_driver);
	} else {
		ttgfb_lcd_controller = &s3c24xx_lcd_controller;
		PK_INFO("Using s3c2410 lcd controller\n");
		result = driver_register(&ttgfb_s3c2410_driver);
	}
	return result;
}

static void __exit ttgfb_exit(void) {
	if(ttgfb_lcd_controller == &s3c24xx_lcd_controller) {
		driver_unregister(&ttgfb_s3c2410_driver);
	} else {
		driver_unregister(&ttgfb_s3c2443_tft_driver);
	}
}

module_init(ttgfb_init);
module_exit(ttgfb_exit);

MODULE_AUTHOR("Christian Daniel <cd@cdaniel.de>, Thomas Kleffel <tk@maintech.de>, Jeroen Taverne <jeroen.taverne@tomtom.com>, Dimitry Andric <dimitry.andric@tomtom.com>, Mark-Jan Bastian <mark-jan.bastian@tomtom.com>, Rogier Stam <rogier.stam@tomtom.com>, Martin Jackson <martin.jackson@tomtom.com> ((c) 2005 TomTom BV)");
MODULE_DESCRIPTION("TomTom GO Framebuffer Driver");
MODULE_LICENSE("GPL");

/* EOF */
