/* drivers/video/tomtom/lq043t3dw01.c
 *
 * Specific functionality for the Sharp LQ043T3DW01 screen. See:
 * http://www.samsung.com/Products/TFTLCD/Small_n_Medium/
 *
 * Copyright (C) 2005 TomTom BV <http://www.tomtom.com/>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef __KERNEL__
#include <linux/delay.h>
#include <linux/cpufreq.h>
#include <barcelona/gopins.h>
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
#include "lq043t3dw01.h"
#include "lcd_controller.h"

// LCDCON1
#define MMODE		0
#define PNR_MODE	3
#define BPP_MODE	12
#define ENVID		0

// LCDCON2
#define VBPD		((2-1)&0x1ff)
#define VFPD		((2-1)&0xff)
#define VSPW		((2-1)&0x3f)

// LCDCON3
#define HBPD		((3-1)&0x7f)
#define HFPD		((2-1)&0xff)

// LCDCON4
#define MVAL		13
#define HSPW		((13-1)&0xff)

// LCDCON5
#define BPP24BL		0
#define FRM565		1
#define INVVCLK		1
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

static void delay_1us_more(void)
{
#ifdef __KERNEL__
    udelay(1);
#else
    mdelay(1);
#endif	
}

void LQ043T3DW01_Init(void)
{
	add_lcdspecs.divider = (unsigned long int) get_clkval( &ttgfb_lq043t3dw01_lcdspecs,
								ttgfb_lq043t3dw01_lcdspecs.refresh_max,
								ttgfb_lq043t3dw01_lcdspecs.curr_hclk );

	get_hozlineval( &ttgfb_lq043t3dw01_lcdspecs, &add_lcdspecs.hozval, &add_lcdspecs.lineval );

  	delay_1us_more();
	ttgfb_lcd_controller->initialize( &ttgfb_lq043t3dw01_lcdspecs, &add_lcdspecs);
	delay_1us_more();

	// Start display controller
	IO_Deactivate(VDEN); // do not use VDEN
	mdelay(1);
	IO_Deactivate(LCD_CS); //SHUT as VCI
	delay_1us_more();
  	
	ttgfb_lcd_controller->start_lcd();

	delay_1us_more();

	IO_Activate(LCD_CS); //SHUT: VCI->GND
	mdelay(300); // at least wait for 15 frames
  
}

void LQ043T3DW01_Off(void)
{
	IO_Deactivate(LCD_CS); //SHUT : GND-> VCI
	mdelay(60); // at least wait for 3 frames

	// Stop display controller
	ttgfb_lcd_controller->stop_lcd();

}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ 
struct lcd_screen_info ttgfb_lq043t3dw01_lcdspecs = {"SHP", "LQ043T3DW01", 95, 54, 480, 272, 25, 62, LQ043T3DW01_Init, LQ043T3DW01_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}},
						  lcd_def_freq_policy, lcd_def_freq_trans};
#else
struct lcd_screen_info ttgfb_lq043t3dw01_lcdspecs = {"SHP", "LQ043T3DW01", 95, 54, 480, 272, 25, 62, LQ043T3DW01_Init, LQ043T3DW01_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}}};
#endif
/* EOF */
