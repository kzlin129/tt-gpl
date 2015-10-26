/* drivers/video/tomtom/nec.c
 *
 * Specific functionality for the NEC screen. See:
 * http://www.nec-lcd.com/english/products/industries/nl2432hc22-22b.html
 *
 * Copyright (C) 2005 TomTom BV <http://www.tomtom.com/>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef __KERNEL__
#include <linux/cpufreq.h>
#include <barcelona/gopins.h>
#include <linux/err.h>
#include <asm/hardware/clock.h>
#include <linux/fb.h>
#endif
#include "lcdregs.h"
#include "screeninfo.h"
#include "framerate.h"
#include "nec.h"

#define MVAL_USED		(0)
#define MVAL			(0)

#define VFPD_240320		((5-1)&0xff)
#define VBPD_240320		((5-1)&0xff)
#define VSPW_240320		((2-1)&0x3f)
#define HFPD_240320		((8-1)&0xff)
#define HBPD_240320		((16-1)&0x7f)
#define HSPW_240320		((24-1)&0xff)

static void NEC_Init(void)
{
	unsigned long int	divider=(unsigned long int) get_clkval( &ttgfb_nl2432hc22_lcdspecs,
									ttgfb_nl2432hc22_lcdspecs.refresh_max,
									ttgfb_nl2432hc22_lcdspecs.curr_hclk );
	unsigned long int	hozval;
	unsigned long int	lineval;

	/* Get HOZVAL and LINEVAL. */
	get_hozlineval( &ttgfb_nl2432hc22_lcdspecs, &hozval, &lineval );

	/* Start S3C display controller, LCM needs dotclock to run */
	rLCDCON1 &= ~1;
	rTCONSEL = 2;
	rLCDCON1 = (divider << 8) | (MVAL_USED << 7) | (((unsigned long int) ttgfb_nl2432hc22_lcdspecs.pnr_mode) << 5) |
		   (((unsigned long int) ttgfb_nl2432hc22_lcdspecs.bpp_mode) << 1) | (0 << 0);
	rLCDCON2 = (((unsigned long int) ttgfb_nl2432hc22_lcdspecs.screen.tft.vbpd) << 24) |
		   (lineval << 14) |
		   (((unsigned long int) ttgfb_nl2432hc22_lcdspecs.screen.tft.vfpd) << 6) |
		   (((unsigned long int) ttgfb_nl2432hc22_lcdspecs.screen.tft.vspw) << 0);
	rLCDCON3 = (((unsigned long int) ttgfb_nl2432hc22_lcdspecs.screen.tft.hbpd) << 19) |
		   (hozval << 8) |
		   (((unsigned long int) ttgfb_nl2432hc22_lcdspecs.screen.tft.hfpd) << 0);
	rLCDCON4 = (MVAL << 8) | (((unsigned long int) ttgfb_nl2432hc22_lcdspecs.screen.tft.hspw) << 0);
	rLCDCON5 = (1<<11)|(1<<8)|(1<<4)|(1<<3)|(1<<2)|(1<<0);
	rLCDCON1 |= 1;
}

static void NEC_Off(void)
{
	// Stop display controller
	rLCDCON1 = 0;
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
struct lcd_screen_info ttgfb_nl2432hc22_lcdspecs = {"NEC", "NL2432HC22", 0, 0, 240, 320, 25, 75, NEC_Init, NEC_Off, NULL,
						    PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW_240320, VBPD_240320, VFPD_240320,
						    HSPW_240320, HBPD_240320, HFPD_240320}},
						    lcd_def_freq_policy, lcd_def_freq_trans};
#else
struct lcd_screen_info ttgfb_nl2432hc22_lcdspecs = {"NEC", "NL2432HC22", 0, 0, 240, 320, 25, 75, NEC_Init, NEC_Off, NULL,
						    PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW_240320, VBPD_240320, VFPD_240320,
						    HSPW_240320, HBPD_240320, HFPD_240320}}};
#endif
