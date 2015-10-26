/* drivers/video/tomtom/ld050wq1.c
 *
 * Based on preliminary specification LD050WQ1-TD01 Ver0.1 (01-06-2010)
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
#include "ld050wq1.h"

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

void LD050WQ1_Init(void)
{
	unsigned long int	divider=(unsigned long int) get_clkval( &ttgfb_ld050wq1_lcdspecs,
									ttgfb_ld050wq1_lcdspecs.refresh_max,
									ttgfb_ld050wq1_lcdspecs.curr_hclk );
	unsigned long int	hozval;
	unsigned long int	lineval;

	/* Get HOZVAL and LINEVAL. */
	get_hozlineval( &ttgfb_ld050wq1_lcdspecs, &hozval, &lineval );
    
    mdelay(12); 		//10ms < T1
    
	/* Start display controller */
	rLCDCON1 &= ~1;
	rTCONSEL = 0;
	rLCDCON1 = (divider << 8) | (MMODE << 7) | (((unsigned long int) ttgfb_ld050wq1_lcdspecs.pnr_mode) << 5) |
						   (((unsigned long int) ttgfb_ld050wq1_lcdspecs.bpp_mode) << 1) | (ENVID << 0);
	rLCDCON2 = (((unsigned long int) ttgfb_ld050wq1_lcdspecs.screen.tft.vbpd) << 24) |
		   (lineval << 14) |
		   (((unsigned long int) ttgfb_ld050wq1_lcdspecs.screen.tft.vfpd) << 6) |
		   (((unsigned long int) ttgfb_ld050wq1_lcdspecs.screen.tft.vspw) << 0);
	rLCDCON3 = (((unsigned long int) ttgfb_ld050wq1_lcdspecs.screen.tft.hbpd) << 19) |
		   (hozval << 8) |
		   (((unsigned long int) ttgfb_ld050wq1_lcdspecs.screen.tft.hfpd) << 0);
	rLCDCON4 = (MVAL << 8) | (((unsigned long int) ttgfb_ld050wq1_lcdspecs.screen.tft.hspw) << 0);
	rLCDCON5 = (BPP24BL << 12) | (FRM565 << 11) | (INVVCLK << 10) | (INVVLINE << 9) | (INVVFRAME << 8) | (INVVD << 7) |
		   (INVVDEN << 6) | (INVPWREN << 5) | (INVLEND << 4) | (PWREN << 3) | (ENLEND << 2) | (BSWP << 1) | (HWSWP << 0);

	// Start display controller
	rLCDCON1 |= 1;

  mdelay(12); //10ms < T2 < 100ms
  
  IO_Deactivate(LCD_CS);//this is actually PON pin for this LCM
  mdelay(200); // 150ms < T3:wait for at least 10 frames before power on backlight
}

void LD050WQ1_Off(void)
{
	mdelay(200); // 150ms < T6:wait for at least 10 frames after power off backlight
	IO_Activate(LCD_CS);//this is actually PON pin for this LCM
	mdelay(120); // T5:at lease 5 frames+T4
	// Stop display controller
	rLCDCON1 = 0;
	mdelay(12);//T4: at lease 10msec
}

#if defined CONFIG_CPU_FREQ  && defined CONFIG_S3C24XX_DFS_CPUFREQ 
struct lcd_screen_info ttgfb_ld050wq1_lcdspecs = {"LGD", "LD050WQ1", 95, 54, 480, 272, 60, 62, LD050WQ1_Init, LD050WQ1_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}},
						  lcd_def_freq_policy, lcd_def_freq_trans};
#else
struct lcd_screen_info ttgfb_ld050wq1_lcdspecs = {"LGD", "LD050WQ1", 95, 54, 480, 272, 60, 62, LD050WQ1_Init, LD050WQ1_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}}};
#endif
/* EOF */
