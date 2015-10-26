/* drivers/video/tomtom/lte430wq.c
 *
 * Specific functionality for the Samsung LTE430WQ screen. See:
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
#include "lte430wq.h"
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

void LTE430WQ_Init(void)
{
	 add_lcdspecs.divider = (unsigned long int) get_clkval( &ttgfb_lte430wq_lcdspecs,
								ttgfb_lte430wq_lcdspecs.refresh_max,
								ttgfb_lte430wq_lcdspecs.curr_hclk );

	get_hozlineval( &ttgfb_lte430wq_lcdspecs, &add_lcdspecs.hozval, &add_lcdspecs.lineval );

	ttgfb_lcd_controller->initialize(&ttgfb_lte430wq_lcdspecs, &add_lcdspecs);
	ttgfb_lcd_controller->start_lcd();

	mdelay(100);
	IO_Deactivate(LCD_RESET);
}

void LTE430WQ_Off(void)
{
	IO_Activate(LCD_RESET);
	mdelay(100);
	ttgfb_lcd_controller->stop_lcd();
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
struct lcd_screen_info ttgfb_lte430wq_lcdspecs = {"SAM", "LTE430WQ", 95, 54, 480, 272, 25, 75, LTE430WQ_Init, LTE430WQ_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}},
						  lcd_def_freq_policy, lcd_def_freq_trans};
#else
struct lcd_screen_info ttgfb_lte430wq_lcdspecs = {"SAM", "LTE430WQ", 95, 54, 480, 272, 25, 75, LTE430WQ_Init, LTE430WQ_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}}};
#endif
/* EOF */
