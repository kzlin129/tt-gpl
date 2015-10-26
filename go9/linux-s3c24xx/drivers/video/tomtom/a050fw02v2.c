/* drivers/video/tomtom/a050fw02v2.c
 *
 * Specific functionality for the AUO A050FW02 screen. 
 *
 * ######## NOTE #############
 * Based on preliminary datasheet A043FW03 V1 ver 0.1 (15-05-2009)
 * Based on preliminary datasheet A050FW02 V2 ver 0.2 (12-05-2009)
 * Based on preliminary datasheet A050FW02 V2 ver 0.9 (28-04-2010)
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
#include "a050fw02v2.h"

#define USE_DATA_ENABLE

#define FRAME   17 //unit ms. By typical frame rate 60Hz

// LCDCON1
#define MMODE		0
#define PNR_MODE	3
#define BPP_MODE	12
#define ENVID		0

// LCDCON2
#define VBPD		1  // was 1 Typical Tvbp = 8 = (VSPW + 1) + (VBPD+1)
#define VFPD		7  // was 3 Typical Tvfp = 8 = VFPD + 1
#define VSPW		5  // was 9 Typical Tvw = 6 = VSPW + 1

// LCDCON3
#define HBPD		38 // was: 41 Typical Thbp = 40 = (HSPW+1) + (HBPD+1)
#define HFPD		10 // was: 7 Typical Thfp = 11 = HFPD+1

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

static void A050FW02V2_WriteByte(unsigned char data)
{
	int bit;

	for (bit=0;bit<8;bit++)
	{
		IO_Deactivate(LCD_SCL);
		if (data & 0x80) IO_Activate(LCD_SDI); else IO_Deactivate(LCD_SDI);
		IO_Activate(LCD_SCL);
		data <<= 1;
		udelay(1);
	}
}

#ifdef __KERNEL__
#define delay3usec()		udelay(3)
#endif
#ifdef __BOOTLOADER__
#define delay3usec()		mdelay(1)
#endif

static void A050FW02V2_Tx_Data(unsigned char Index, unsigned char REG_DATA)
{
	unsigned long flags;

	IO_Activate(LCD_SCL);
	IO_Deactivate(LCD_CS);

	local_irq_save(flags);
	IO_Activate(LCD_CS);
	A050FW02V2_WriteByte(Index << 2 | 0x0 << 1);	/* write to register[Index] */
	A050FW02V2_WriteByte(REG_DATA);     		/* Write register address 8 bit */
	IO_Deactivate(LCD_CS);
	local_irq_restore(flags);
	delay3usec();
}

void A050FW02V2_Init(void)
{
	unsigned long int	divider=(unsigned long int) get_clkval( &ttgfb_a050fw02v2_lcdspecs,
									ttgfb_a050fw02v2_lcdspecs.refresh_max,
									ttgfb_a050fw02v2_lcdspecs.curr_hclk );
	unsigned long int	hozval;
	unsigned long int	lineval;

	IO_Deactivate(LCD_CS); /* Deactive serial chip select */
	IO_Activate(LCD_RESET); /* HVDSL="L" ?? */
	
	/* Get HOZVAL and LINEVAL. */
	get_hozlineval( &ttgfb_a050fw02v2_lcdspecs, &hozval, &lineval );

	/* Start display controller */
	rLCDCON1 &= ~1;
	rTCONSEL = 0;
	rLCDCON1 = (divider << 8) | (MMODE << 7) | (((unsigned long int) ttgfb_a050fw02v2_lcdspecs.pnr_mode) << 5) |
						   (((unsigned long int) ttgfb_a050fw02v2_lcdspecs.bpp_mode) << 1) | (ENVID << 0);
	rLCDCON2 = (((unsigned long int) ttgfb_a050fw02v2_lcdspecs.screen.tft.vbpd) << 24) |
		   (lineval << 14) |
		   (((unsigned long int) ttgfb_a050fw02v2_lcdspecs.screen.tft.vfpd) << 6) |
		   (((unsigned long int) ttgfb_a050fw02v2_lcdspecs.screen.tft.vspw) << 0);
	rLCDCON3 = (((unsigned long int) ttgfb_a050fw02v2_lcdspecs.screen.tft.hbpd) << 19) |
		   (hozval << 8) |
		   (((unsigned long int) ttgfb_a050fw02v2_lcdspecs.screen.tft.hfpd) << 0);
	rLCDCON4 = (MVAL << 8) | (((unsigned long int) ttgfb_a050fw02v2_lcdspecs.screen.tft.hspw) << 0);
	rLCDCON5 = (BPP24BL << 12) | (FRM565 << 11) | (INVVCLK << 10) | (INVVLINE << 9) | (INVVFRAME << 8) | (INVVD << 7) |
		   (INVVDEN << 6) | (INVPWREN << 5) | (INVLEND << 4) | (PWREN << 3) | (ENLEND << 2) | (BSWP << 1) | (HWSWP << 0);

	// Start display controller
	rLCDCON1 |= 1;

#ifndef USE_DATA_ENABLE  
    IO_Deactivate(VDEN);    
#endif

	mdelay(50);
	A050FW02V2_Tx_Data(0x30, 0x00); /* R48: discharge voltage */
	A050FW02V2_Tx_Data(1, 0x1b); /* Timer controller in reset state */
	A050FW02V2_Tx_Data(1, 0x1f); /* STB=1 */
	A050FW02V2_Tx_Data(0, 0x00); /* falling edge */
	A050FW02V2_Tx_Data(4, 0x28);
	A050FW02V2_Tx_Data(5, 0x08);
	A050FW02V2_Tx_Data(6, 0x5c); /* new color gamma */
	A050FW02V2_Tx_Data(7, 0x00); /* new color gamma */
	A050FW02V2_Tx_Data(8, 0x40);
	A050FW02V2_Tx_Data(9, 0x40);
	A050FW02V2_Tx_Data(0xa, 0x40);
	A050FW02V2_Tx_Data(0xb, 0x40);
	A050FW02V2_Tx_Data(0xc, 0x40);
	A050FW02V2_Tx_Data(0xd, 0x40);	

	mdelay(1*FRAME);
//	IO_Deactivate(LCD_RESET); //Pull High DISP A043FW03V1 */
	mdelay(9*FRAME);
}

void A050FW02V2_Off(void)
{
	A050FW02V2_Tx_Data(1, 0x07); /* STB=0 */
//	IO_Activate(LCD_RESET); //Pull Low DISP
	mdelay(7*FRAME);
	// Stop display controller
	rLCDCON1 = 0;

}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ 
struct lcd_screen_info ttgfb_a050fw02v2_lcdspecs = {"AUO", "A050FW02V2", 95, 54, 480, 272, 60, 70, A050FW02V2_Init, A050FW02V2_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}},
						  lcd_def_freq_policy, lcd_def_freq_trans};
#else
struct lcd_screen_info ttgfb_a050fw02v2_lcdspecs = {"AUO", "A050FW02V2", 95, 54, 480, 272, 60, 70, A050FW02V2_Init, A050FW02V2_Off, NULL,
						  PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD, HSPW, HBPD, HFPD}}};
#endif
/* EOF */
