/* drivers/video/tomtom/ltp400wq.c
 *
 * Specific functionality for the Samsung LTP400WQ screen. See:
 * http://www.samsung.com/Products/TFTLCD/Small_n_Medium/LTP400WQ_F01/LTP400WQ-F01.htm
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
#include "ltp400wq.h"

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
#define HBPD		7
#define HFPD		2

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

void LTP400WQ_Init(void)
{
        unsigned long int       divider=(unsigned long int) get_clkval( &ttgfb_ltp400wq_lcdspecs,
                                                                        ttgfb_ltp400wq_lcdspecs.refresh_max,
                                                                        ttgfb_ltp400wq_lcdspecs.curr_hclk );
        unsigned long int       hozval;
        unsigned long int       lineval;

        /* Get HOZVAL and LINEVAL. */
        get_hozlineval( &ttgfb_ltp400wq_lcdspecs, &hozval, &lineval );

        /* Start display controller */
        rLCDCON1 &= ~1;
        rTCONSEL = 0;
        rLCDCON1 = (divider << 8) | (MMODE << 7) | (((unsigned long int) ttgfb_ltp400wq_lcdspecs.pnr_mode) << 5) |
                   (((unsigned long int) ttgfb_ltp400wq_lcdspecs.bpp_mode) << 1) | (ENVID << 0);
        rLCDCON2 = (((unsigned long int) ttgfb_ltp400wq_lcdspecs.screen.tft.vbpd) << 24) |
                   (lineval << 14) |
                   (((unsigned long int) ttgfb_ltp400wq_lcdspecs.screen.tft.vfpd) << 6) |
                   (((unsigned long int) ttgfb_ltp400wq_lcdspecs.screen.tft.vspw) << 0);
        rLCDCON3 = (((unsigned long int) ttgfb_ltp400wq_lcdspecs.screen.tft.hbpd) << 19) |
                   (hozval << 8) |
                   (((unsigned long int) ttgfb_ltp400wq_lcdspecs.screen.tft.hfpd) << 0);
        rLCDCON4 = (MVAL << 8) | (((unsigned long int) ttgfb_ltp400wq_lcdspecs.screen.tft.hspw) << 0);
        rLCDCON5 = (BPP24BL << 12) | (FRM565 << 11) | (INVVCLK << 10) | (INVVLINE << 9) | (INVVFRAME << 8) | (INVVD << 7) |
                   (INVVDEN << 6) | (INVPWREN << 5) | (INVLEND << 4) | (PWREN << 3) | (ENLEND << 2) | (BSWP << 1) | (HWSWP << 0);
        rLCDCON1 |= 1;

        mdelay(100);
        IO_Deactivate(LCD_RESET);
}

void LTP400WQ_Off(void)
{
	IO_Activate(LCD_RESET);
	mdelay(100);
	// Stop display controller
	rLCDCON1 = 0;
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
struct lcd_screen_info ttgfb_ltp400wq_lcdspecs = {"SAM", "LTP400WQ", 88, 50, 480, 272, 25, 75, LTP400WQ_Init, LTP400WQ_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}},
						  lcd_def_freq_policy, lcd_def_freq_trans};
#else
struct lcd_screen_info ttgfb_ltp400wq_lcdspecs = {"SAM", "LTP400WQ", 88, 50, 480, 272, 25, 75, LTP400WQ_Init, LTP400WQ_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}}};
#endif
/* EOF */
