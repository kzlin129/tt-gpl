/* drivers/video/tomtom/lq043t1dg01.c
 *
 * Specific functionality for the Sharp LQ043T1DG01 screen.
 *
 * Copyright (C) 2005,2006 TomTom BV <http://www.tomtom.com/>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef __KERNEL__
#include <linux/delay.h>
#include <barcelona/gopins.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <asm/hardware/clock.h>
#endif /* __KERNEL */
#ifdef __BOOTLOADER__
#include "timer.h"
#include "gopins.h"
#endif /* __BOOTLOADER__ */
#include "lcdregs.h"
#include "screeninfo.h"
#include "framerate.h"
#include "lq043t1.h"
#include "lcd_controller.h"

// LCDCON1
#define MMODE		0
#define ENVID		0

// LCDCON2
#define VBPD		1
#define VFPD		1
#define VSPW		10

// LCDCON3
#define HBPD		1
#define HFPD		1

// LCDCON4
#define MVAL		13
#define HSPW		41

// LCDCON5
#define BPP24BL		0
#define FRM565		1
#define INVVCLK		0
#define INVVLINE	1
#define INVVFRAME	1
#define INVVD		0
#define INVVDEN		0
#define INVPWREN	0
#define INVLEND		1
#define PWREN		1
#define ENLEND		0
#define BSWP		0
#define HWSWP		1

static struct lcd_additional_info add_lcdspecs = {
	.bswp = BSWP, .hwswp = HWSWP, .invvclk = INVVCLK, .invvline = INVVLINE, .invvframe = INVVFRAME, .invvd = INVVD, .invvden = INVVDEN,
	.pwren = PWREN, .invpwren = INVPWREN, .enlend = ENLEND, .invlend = INVLEND,
	.envid = ENVID, .mmode = MMODE, .mval = MVAL, .bpp24bl = BPP24BL, .frm565 = FRM565,
	.hozval = 0, .lineval = 0, .divider = 0
};

void LQ043T1_Init(void)
{
	add_lcdspecs.divider = (unsigned long int) get_clkval( &ttgfb_lq043t1_lcdspecs,
								ttgfb_lq043t1_lcdspecs.refresh_max,
								ttgfb_lq043t1_lcdspecs.curr_hclk );

	get_hozlineval( &ttgfb_lq043t1_lcdspecs, &add_lcdspecs.hozval, &add_lcdspecs.lineval );

	ttgfb_lcd_controller->initialize( &ttgfb_lq043t1_lcdspecs, &add_lcdspecs);
	ttgfb_lcd_controller->start_lcd();

	mdelay(100);
	IO_Deactivate(LCD_RESET);
}

void LQ043T1_Off(void)
{
	IO_Activate(LCD_RESET);
	mdelay(100);
	// Stop display controller
	ttgfb_lcd_controller->stop_lcd();
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
struct lcd_screen_info ttgfb_lq043t1_lcdspecs = {"SHP", "LQ043T1", 95, 54, 480, 272, 25, 75, LQ043T1_Init, LQ043T1_Off, NULL,
						 PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}},
						 lcd_def_freq_policy, lcd_def_freq_trans};
#else
struct lcd_screen_info ttgfb_lq043t1_lcdspecs = {"SHP", "LQ043T1", 95, 54, 480, 272, 25, 75, LQ043T1_Init, LQ043T1_Off, NULL,
						 PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}}};
#endif

/* EOF */
