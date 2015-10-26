/* drivers/video/tomtom/a050fw03v2.c
 *
 * Specific functionality for the AUO A050FW03 screen. 
 *
 * ######## NOTE ############# 
 * Based on preliminary datasheet A043FW03 V1 ver 0.1 (15-05-2009)
 * Based on preliminary datasheet A050FW02 V2 ver 0.2 (12-05-2009)
 * Based on preliminary datasheet A050FW02 V2 ver 0.9 (28-04-2010)
 * Based on preliminary datasheet A050FW03 V2 ver 0.2 (13-05-2010)
 *
 * ###########################
 *
 * Copyright (C) 2010 TomTom BV <http://www.tomtom.com/>
 * Author: Will Lin <Will.Lin@tomtom.com>
 * Author: Kwok Wong <Kwok.Wong@tomtom.com> 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef __KERNEL__
#include <linux/delay.h>
#include <asm/system.h>			/* for local_irq_save/restore */
#include <linux/cpufreq.h>
#include <barcelona/gopins.h>
#include <linux/err.h>
#include <asm/hardware/clock.h>
#endif /* __KERNEL */
#ifdef __BOOTLOADER__
#include "irqsave.h"
#include "timer.h"
#include "gopins.h"
#endif /* __BOOTLOADER__ */
#include "lcdregs.h"
#include "screeninfo.h"
#include "framerate.h"
#include "a050fw03v2.h"

#define USE_DATA_ENABLE

#define FRAME   17 //unit ms. By typical frame rate 60Hz

// LCDCON1
#define MMODE		0
#define PNR_MODE	3
#define BPP_MODE	12
#define ENVID		0

// LCDCON2
#define VBPD		6  // was 1, Typical Tvbp = 8 = (VSPW + 1) + (VBPD+1)
#define VFPD		7  // was 7, Typical Tvfp = 8 = VFPD + 1
#define VSPW		0  // was 5, Typical Tvw = 1 = VSPW + 1

// LCDCON3
#define HBPD		38 // was 38, Typical Thbp = 40 = (HSPW+1) + (HBPD+1)
#define HFPD		4  // was 10, Typical Thfp = 5 = HFPD+1

// LCDCON4
#define MVAL		0
#define HSPW		0  //Typical Thw = 1 = HSPW+1

// LCDCON5
#define BPP24BL		0
#define FRM565		1
#define INVVCLK		1 //The video data is fetched at VCLK rising edge
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

static void A050FW03V2_WriteHWord(unsigned short data)
{
	int bit;
	
	for ( bit = 0; bit < 16; bit++ )
	{
		IO_Deactivate(LCD_SCL);
		if (data & 0x8000) IO_Activate(LCD_SDI); else IO_Deactivate(LCD_SDI);
		data <<= 1;
		IO_Activate(LCD_SCL);
		IO_Activate(LCD_SCL); /* make high period longer */
	}
}

#ifdef __KERNEL__
#define delay3usec()		udelay(3)
#endif
#ifdef __BOOTLOADER__
#define delay3usec()		mdelay(1)
#endif

static void A050FW03V2_Tx_Data(unsigned char Index, unsigned char REG_DATA)
{
	unsigned long flags;
	unsigned short value;
	
	IO_Activate(LCD_SCL);
	IO_Deactivate(LCD_CS);

	value = (Index << 1) ;	             /* write to register[Index] */
	value = (value << 8) | REG_DATA;	/* Write register address 8 bit */
	
	local_irq_save(flags);
	IO_Activate(LCD_CS);
	A050FW03V2_WriteHWord(value);
	IO_Deactivate(LCD_CS);
	local_irq_restore(flags);
	delay3usec();
}

void A050FW03V2_Init(void)
{
	unsigned long int	divider=(unsigned long int) get_clkval( &ttgfb_a050fw03v2_lcdspecs,
									ttgfb_a050fw03v2_lcdspecs.refresh_max,
									ttgfb_a050fw03v2_lcdspecs.curr_hclk );
	unsigned long int	hozval;
	unsigned long int	lineval;

	IO_Deactivate(LCD_CS); /* Deactive serial chip select */
#ifndef USE_DATA_ENABLE
    IO_Deactivate(LCD_RESET); /* HVDSL=биHби: Set under HV mode */
#else
	IO_Activate(LCD_RESET); /* HVDSL=биLби: Set under DE mode */
#endif
	
	/* Get HOZVAL and LINEVAL. */
	get_hozlineval( &ttgfb_a050fw03v2_lcdspecs, &hozval, &lineval );

    mdelay(10);
    
    /* Start display controller */
	rLCDCON1 &= ~1;
	rTCONSEL = 0;
	rLCDCON1 = (divider << 8) | (MMODE << 7) | (((unsigned long int) ttgfb_a050fw03v2_lcdspecs.pnr_mode) << 5) |
						   (((unsigned long int) ttgfb_a050fw03v2_lcdspecs.bpp_mode) << 1) | (ENVID << 0);
	rLCDCON2 = (((unsigned long int) ttgfb_a050fw03v2_lcdspecs.screen.tft.vbpd) << 24) |
		   (lineval << 14) |
		   (((unsigned long int) ttgfb_a050fw03v2_lcdspecs.screen.tft.vfpd) << 6) |
		   (((unsigned long int) ttgfb_a050fw03v2_lcdspecs.screen.tft.vspw) << 0);
	rLCDCON3 = (((unsigned long int) ttgfb_a050fw03v2_lcdspecs.screen.tft.hbpd) << 19) |
		   (hozval << 8) |
		   (((unsigned long int) ttgfb_a050fw03v2_lcdspecs.screen.tft.hfpd) << 0);
	rLCDCON4 = (MVAL << 8) | (((unsigned long int) ttgfb_a050fw03v2_lcdspecs.screen.tft.hspw) << 0);
	rLCDCON5 = (BPP24BL << 12) | (FRM565 << 11) | (INVVCLK << 10) | (INVVLINE << 9) | (INVVFRAME << 8) | (INVVD << 7) |
		   (INVVDEN << 6) | (INVPWREN << 5) | (INVLEND << 4) | (PWREN << 3) | (ENLEND << 2) | (BSWP << 1) | (HWSWP << 0);

	// Start display controller
	rLCDCON1 |= 1;

#ifndef USE_DATA_ENABLE  
    IO_Deactivate(VDEN);    
#endif

	mdelay(50); 
	A050FW03V2_Tx_Data(5, 0x1e);
	A050FW03V2_Tx_Data(5, 0x5e);
    A050FW03V2_Tx_Data(5, 0x5f);
#ifndef USE_DATA_ENABLE
    A050FW03V2_Tx_Data(6, 0x8|(VBPD+2));
    A050FW03V2_Tx_Data(7, HBPD+2);
#endif

	mdelay(11*FRAME);	

}

void A050FW03V2_Off(void)
{
	A050FW03V2_Tx_Data(5, 0x5e); 
	mdelay(10*FRAME);

	// Stop display controller
	rLCDCON1 = 0;
	
    mdelay(10);
    /* Configure these PINs to resolve power leakage */
    IO_Activate(LCD_RESET);
	IO_Deactivate(LCD_SCL);
	IO_Deactivate(LCD_SDI);    
    IO_Activate(LCD_CS);
	mdelay(1);    
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ 
struct lcd_screen_info ttgfb_a050fw03v2_lcdspecs = {"AUO", "A050FW03V2", 95, 54, 480, 272, 60, 70, A050FW03V2_Init, A050FW03V2_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}},
						  lcd_def_freq_policy, lcd_def_freq_trans};
#else
struct lcd_screen_info ttgfb_a050fw03v2_lcdspecs = {"AUO", "A050FW03V2", 95, 54, 480, 272, 60, 70, A050FW03V2_Init, A050FW03V2_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}}};
#endif
/* EOF */
