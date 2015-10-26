/* drivers/video/tomtom/lms500hf01.c
 *
 * Specific functionality for the LMS500HF01 screen. 
 *
 * ######## NOTE #############
 * Based on preliminary datasheet LMS500HF01 Rev. 000  (30-06-2009)
  *
 * ###########################
 *
 * Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 * Author: Kwok Wong <Kwok.Wong@tomtom.com>
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
#include "lms500hf01.h"

#define USE_DATA_ENABLE

#define FRAME   17 //unit ms. By typical frame rate 60Hz

// LCDCON1
#define MMODE		0
#define PNR_MODE	3
#define BPP_MODE	12
#define ENVID		0

// LCDCON2
#define VBPD		5  // was 1 Typical Tvbp = 8 = (VSPW + 1) + (VBPD+1)
#define VFPD		3  // was 7 Typical Tvfp = 8 = VFPD + 1
#define VSPW		5  // was 5 Typical Tvw = 6 = VSPW + 1

// LCDCON3
#define HBPD		38 // was: 41 Typical Thbp = 40 = (HSPW+1) + (HBPD+1)
#define HFPD		10 // was: 7 Typical Thfp = 11 = HFPD+1

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

#ifdef __KERNEL__
#define delay3usec()		udelay(3)
#endif
#ifdef __BOOTLOADER__
#define delay3usec()		mdelay(1)
#endif

void LMS500HF01_Init(void)
{
	unsigned long int	divider=(unsigned long int) get_clkval( &ttgfb_lms500hf01_lcdspecs,
									ttgfb_lms500hf01_lcdspecs.refresh_max,
									ttgfb_lms500hf01_lcdspecs.curr_hclk );
	unsigned long int	hozval;
	unsigned long int	lineval;

	/* Get HOZVAL and LINEVAL. */
	get_hozlineval( &ttgfb_lms500hf01_lcdspecs, &hozval, &lineval );

	mdelay(10); 		//5ms < T1, this delay is added since LMS500HF05

	/* Start display controller */
	rLCDCON1 &= ~1;
	rTCONSEL = 0;
	rLCDCON1 = (divider << 8) | (MMODE << 7) | (((unsigned long int) ttgfb_lms500hf01_lcdspecs.pnr_mode) << 5) |
						   (((unsigned long int) ttgfb_lms500hf01_lcdspecs.bpp_mode) << 1) | (ENVID << 0);
	rLCDCON2 = (((unsigned long int) ttgfb_lms500hf01_lcdspecs.screen.tft.vbpd) << 24) |
		   (lineval << 14) |
		   (((unsigned long int) ttgfb_lms500hf01_lcdspecs.screen.tft.vfpd) << 6) |
		   (((unsigned long int) ttgfb_lms500hf01_lcdspecs.screen.tft.vspw) << 0);
	rLCDCON3 = (((unsigned long int) ttgfb_lms500hf01_lcdspecs.screen.tft.hbpd) << 19) |
		   (hozval << 8) |
		   (((unsigned long int) ttgfb_lms500hf01_lcdspecs.screen.tft.hfpd) << 0);
	rLCDCON4 = (MVAL << 8) | (((unsigned long int) ttgfb_lms500hf01_lcdspecs.screen.tft.hspw) << 0);
	rLCDCON5 = (BPP24BL << 12) | (FRM565 << 11) | (INVVCLK << 10) | (INVVLINE << 9) | (INVVFRAME << 8) | (INVVD << 7) |
		   (INVVDEN << 6) | (INVPWREN << 5) | (INVLEND << 4) | (PWREN << 3) | (ENLEND << 2) | (BSWP << 1) | (HWSWP << 0);

	// Start display controller
	rLCDCON1 |= 1;

#ifndef USE_DATA_ENABLE  
    IO_Deactivate(VDEN);    
#endif

  mdelay(12); //T2: 10msec
  
  IO_Deactivate(LCD_CS);//this is actually PON pin for this LCM
  mdelay(250);		// T3:Since LMS500HF05, T3 is increased to 12 frames before power on backlight
}

void LMS500HF01_Off(void)
{
	mdelay(200); // T6:wait for at least 10 frames after power off backlight
	IO_Activate(LCD_CS);//this is actually PON pin for this LCM
	mdelay(120); // T5:at least 6 frames+T4
	// Stop display controller
	rLCDCON1 = 0;
	mdelay(12);//T4: at least 10msec
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
struct lcd_screen_info ttgfb_lms500hf01_lcdspecs = {"SAM", "LMS500HF01", 95, 54, 480, 272, 25, 75, LMS500HF01_Init, LMS500HF01_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}},
						  lcd_def_freq_policy, lcd_def_freq_trans};
#else
struct lcd_screen_info ttgfb_lms500hf01_lcdspecs = {"SAM", "LMS500HF01", 95, 54, 480, 272, 25, 75, LMS500HF01_Init, LMS500HF01_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}}};
#endif
/* EOF */
