/* drivers/video/tomtom/lb043wq3.c
 *
 * Based on preliminary specification LB043WQ3-TD03 Ver0.2 (16-03-2010)
 *
 * Copyright (C) 2010 TomTom BV <http://www.tomtom.com/>
 * Author: Will Lin       <Will.Lin@tomtom.com>
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
#include "lb043wq3.h"
#include "lcd_controller.h"

// LCDCON1
#define MMODE		0
#define PNR_MODE	3
#define BPP_MODE	12
#define ENVID		0

// LCDCON2
#define VBPD		4
#define VFPD		4
#define VSPW		16

// LCDCON3
#define HBPD		14
#define HFPD		14

// LCDCON4
#define MVAL		13
#define HSPW		14

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

void LB043WQ3_Init(void)
{
	add_lcdspecs.divider = (unsigned long int) get_clkval( &ttgfb_lb043wq3_lcdspecs,
								ttgfb_lb043wq3_lcdspecs.refresh_max,
								ttgfb_lb043wq3_lcdspecs.curr_hclk );

	get_hozlineval( &ttgfb_lb043wq3_lcdspecs, &add_lcdspecs.hozval, &add_lcdspecs.lineval );

    mdelay(12); 		//10ms < T1
	ttgfb_lcd_controller->initialize(&ttgfb_lb043wq3_lcdspecs, &add_lcdspecs);
	ttgfb_lcd_controller->start_lcd();

	mdelay(12); 		//10ms < T2 < 100ms
	IO_Deactivate(LCD_CS);	//this is actually PON pin for this LCM
	mdelay(200); // 150ms < T3:wait for at least 10 frames before power on backlight
}

void LB043WQ3_Off(void)
{
	mdelay(200); // 150ms < T6:wait for at least 10 frames after power off backlight
	IO_Activate(LCD_CS);//this is actually PON pin for this LCM
	mdelay(120); // T5:at lease 5 frames+T4

	ttgfb_lcd_controller->stop_lcd();
	mdelay(12);//T4: at lease 10msec
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
struct lcd_screen_info ttgfb_lb043wq3_lcdspecs = {"LGD", "LB043WQ3", 95, 54, 480, 272, 60, 62, LB043WQ3_Init, LB043WQ3_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}},
						  lcd_def_freq_policy, lcd_def_freq_trans};
#else
struct lcd_screen_info ttgfb_lb043wq3_lcdspecs = {"LGD", "LB043WQ3", 95, 54, 480, 272, 60, 62, LB043WQ3_Init, LB043WQ3_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}}};
#endif
/* EOF */
