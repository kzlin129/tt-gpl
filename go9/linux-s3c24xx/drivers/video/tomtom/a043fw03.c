/* drivers/video/tomtom/a043fw03.c
 *
 * Specific functionality for the AUO A043FW03 screen. 
 *
 * ######## NOTE #############
 * Based on preliminary datasheet A043FW03 V1 ver 0.1 (15-05-2009)
 * Modified by preliminary specification A043FW05 V1 ver 0.1 (16-04-2010)
 *
 * ###########################
 *
 * Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 * Author: Will Lin <Will.Lin@tomtom.com>
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
#include "a043fw03.h"
#include "lcd_controller.h"

#define USE_DATA_ENABLE

#define FRAME   17 //unit ms. By typical frame rate 60Hz

// LCDCON1
#define MMODE		0
#define PNR_MODE	3
#define BPP_MODE	12
#define ENVID		0

// LCDCON2
#define VBPD		1  //Typical Tvbp = 12 = (VSPW + 1) + (VBPD+1)
#define VFPD		3  //Typical Tvfp = 4 = VFPD + 1
#define VSPW		9  //Typical Tvw = 10 = VSPW + 1

// LCDCON3
#define HBPD		41 //Typical Thbp = 43 = (HSPW+1) + (HBPD+1)
#define HFPD		7  //Typical Thfp = 8 = HFPD+1

// LCDCON4
#define MVAL		0
#define HSPW		0  //Typical Thw = 1 = HSPW+1

// LCDCON5
#define BPP24BL		0
#define FRM565		1
#define INVVCLK		0 //The video data is fetched at VCLK falling edge
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
	.bswp = BSWP, .hwswp = HWSWP, .invvclk = INVVCLK, .invvline = INVVLINE,
	.invvframe = INVVFRAME, .invvd = INVVD, .invvden = INVVDEN, .pwren = PWREN,
	.invpwren = INVPWREN, .enlend = ENLEND, .invlend = INVLEND, .envid = ENVID, .mmode = MMODE,
	.mval = MVAL, .bpp24bl = BPP24BL, .frm565 = FRM565, .hozval = 0, .lineval = 0, .divider = 0 
};

static void A043FW03_Init(void)
{
	add_lcdspecs.divider = (unsigned long int) get_clkval( &ttgfb_a043fw03_lcdspecs,
								ttgfb_a043fw03_lcdspecs.refresh_max,
								ttgfb_a043fw03_lcdspecs.curr_hclk );
	/* Get HOZVAL and LINEVAL. */
	get_hozlineval( &ttgfb_a043fw03_lcdspecs, &add_lcdspecs.hozval, &add_lcdspecs.lineval );

	// Start display controller
	ttgfb_lcd_controller->initialize(&ttgfb_a043fw03_lcdspecs, &add_lcdspecs);
	ttgfb_lcd_controller->start_lcd();

#ifndef USE_DATA_ENABLE  
	IO_Deactivate(VDEN);    
#endif
           	
	mdelay(1*FRAME);
	IO_Deactivate(LCD_CS); //Pull High DISP
	mdelay(11*FRAME);  //Since A043FW05V1
}

void A043FW03_Off(void)
{
	IO_Activate(LCD_CS); //Pull Low DISP	
	mdelay(10*FRAME);  //Since A043FW05V1

	// Stop display controller
	ttgfb_lcd_controller->stop_lcd();
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ 
struct lcd_screen_info ttgfb_a043fw03_lcdspecs = {"AUO", "A043FW03", 95, 54, 480, 272, 60, 70, A043FW03_Init, A043FW03_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}},
						  lcd_def_freq_policy, lcd_def_freq_trans};
#else
struct lcd_screen_info ttgfb_a043fw03_lcdspecs = {"AUO", "A043FW03", 95, 54, 480, 272, 60, 70, A043FW03_Init, A043FW03_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}}};
#endif
/* EOF */
