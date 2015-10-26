/* drivers/video/tomtom/lms430hf12.c
 *
 * Specific functionality for the Samsung LMS430HF12 screen. See:
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
#include "lms430hf12.h"
#include "lcd_controller.h"

// LCDCON1
#define MMODE		0
#define PNR_MODE	3
#define BPP_MODE	12
#define ENVID		0

// LCDCON2
#define VBPD		0
#define VFPD		2
#define VSPW		10

// LCDCON3
#define HBPD		45
#define HFPD		7

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

void LMS430HF12_Init(void)
{
	add_lcdspecs.divider = (unsigned long int) get_clkval( &ttgfb_lms430hf12_lcdspecs,
								ttgfb_lms430hf12_lcdspecs.refresh_max,
								ttgfb_lms430hf12_lcdspecs.curr_hclk );

	get_hozlineval( &ttgfb_lms430hf12_lcdspecs, &add_lcdspecs.hozval, &add_lcdspecs.lineval );

	mdelay(10); 		//5ms < T1, this delay is added since LMS430HF29

	ttgfb_lcd_controller->initialize(&ttgfb_lms430hf12_lcdspecs, &add_lcdspecs);
	ttgfb_lcd_controller->start_lcd();

	mdelay(12); 		//T2: 10msec
	IO_Deactivate(LCD_CS);	//this is actually PON pin for this LCM
	mdelay(250);		// T3:Since LMS430HF29, T3 is increased to 12 frames before power on backlight
}

void LMS430HF12_Off(void)
{
	mdelay(200); // T3:wait for at least 10 frames after power off backlight
	IO_Activate(LCD_CS);//this is actually PON pin for this LCM
	mdelay(120); // T5:at lease 5 frames+T4

	ttgfb_lcd_controller->stop_lcd();
	mdelay(12);//T4: at lease 10msec
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
struct lcd_screen_info ttgfb_lms430hf12_lcdspecs = {"SAM", "LMS430HF12", 95, 54, 480, 272, 25, 75, LMS430HF12_Init, LMS430HF12_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}},
						  lcd_def_freq_policy, lcd_def_freq_trans};
#else
struct lcd_screen_info ttgfb_lms430hf12_lcdspecs = {"SAM", "LMS430HF12", 95, 54, 480, 272, 25, 75, LMS430HF12_Init, LMS430HF12_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}}};
#endif
/* EOF */
