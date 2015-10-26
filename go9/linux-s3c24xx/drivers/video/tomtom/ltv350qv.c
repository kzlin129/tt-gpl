/* drivers/video/tomtom/ltv350qv.c
 *
 * Specific functionality for the Samsung LTV350QV screen. See:
 * http://www.samsung.com/Products/TFTLCD/Small_n_Medium/LTV350QV_F01/LTV350QV-F01.htm
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
#include "ltv350qv.h"

#define MVAL_USED		(0)
#define MVAL			(0)

#define VBPD_320240		((7-1)&0xff)
#define VFPD_320240		((7-1)&0xff)
#define VSPW_320240		((3-1) &0x3f)
#define HBPD_320240		((19-1)&0x7f)
#define HFPD_320240		((17-1)&0xff)
#define HSPW_320240		((14-1)&0xff)

static void LTV350QV_WriteByte(unsigned char data)
{
	int bit;

	for (bit=0;bit<8;bit++)
	{
		IO_Deactivate(LCD_SCL);
		if (data & 0x80) IO_Activate(LCD_SDI); else IO_Deactivate(LCD_SDI);
		IO_Activate(LCD_SCL);
		data <<= 1;
	}
}

static void LTV350QV_Tx_Data(unsigned char Index,unsigned short REG_DATA)
{
	unsigned char DeviceID = 0x74;
	unsigned char lcd_Index = (unsigned char)(Index & 0xff);
	unsigned char lcd_data0 = (unsigned char)((REG_DATA >> 8)& 0xff);
	unsigned char lcd_data1 = (unsigned char)( REG_DATA & 0xff);

	IO_Deactivate(LCD_SCL);
	IO_Deactivate(LCD_CS);

	IO_Activate(LCD_CS);
	LTV350QV_WriteByte(DeviceID|0x0);      //Send Device ID code
	LTV350QV_WriteByte(0x00);	     //Write register 8 bit
	LTV350QV_WriteByte(lcd_Index);     //Write register 8 bit
	IO_Deactivate(LCD_CS);
	mdelay(1);

	IO_Activate(LCD_CS);
	LTV350QV_WriteByte(DeviceID|0x2);      //Send Device ID code
	LTV350QV_WriteByte(lcd_data0);     //Write the data first 8 bit
	LTV350QV_WriteByte(lcd_data1);     //Write the data second 8 bit
	IO_Deactivate(LCD_CS);
	mdelay(1);                 //need to delay 50us
}

void LTV350QV_Init(void)
{
	unsigned long int	divider=(unsigned long int) get_clkval( &ttgfb_ltv350qv_lcdspecs,
									ttgfb_ltv350qv_lcdspecs.refresh_max,
									ttgfb_ltv350qv_lcdspecs.curr_hclk );
	unsigned long int	hozval;
	unsigned long int	lineval;

	/* Get HOZVAL and LINEVAL. */
	get_hozlineval( &ttgfb_ltv350qv_lcdspecs, &hozval, &lineval );

	IO_Deactivate(LCD_RESET);
	mdelay(10);
	IO_Activate(LCD_RESET);
	mdelay(10);
	IO_Deactivate(LCD_RESET);
	mdelay(10);

	// Start display controller
	rLCDCON1 &= ~1;
	rTCONSEL = 0;
	rLCDCON1 = (divider << 8) | (MVAL_USED << 7) | (((unsigned long int) ttgfb_ltv350qv_lcdspecs.pnr_mode) << 5) |
		   (((unsigned long int) ttgfb_ltv350qv_lcdspecs.bpp_mode) << 1) | (0 << 0);
	rLCDCON2 = (((unsigned long int) ttgfb_ltv350qv_lcdspecs.screen.tft.vbpd) << 24) |
		   (lineval << 14) |
		   (((unsigned long int) ttgfb_ltv350qv_lcdspecs.screen.tft.vfpd) << 6) |
		   (((unsigned long int) ttgfb_ltv350qv_lcdspecs.screen.tft.vspw) << 0);
	rLCDCON3=(((unsigned long int) ttgfb_ltv350qv_lcdspecs.screen.tft.hbpd) << 19) | (hozval << 8) |
		 (((unsigned long int) ttgfb_ltv350qv_lcdspecs.screen.tft.hfpd) << 0);
	rLCDCON4 = (MVAL << 8) | (((unsigned long int) ttgfb_ltv350qv_lcdspecs.screen.tft.hspw) << 0);
	rLCDCON5=(1<<11)|(1<<10)|(1<<9)|(1<<8)|(1<<6)|(1<<0);

	LTV350QV_Tx_Data(0x09,0x0000);
	mdelay(10);
	LTV350QV_Tx_Data(0x09,0x4000);
	LTV350QV_Tx_Data(0x0a,0x2000);
	LTV350QV_Tx_Data(0x09,0x4055);
	mdelay(50);
	LTV350QV_Tx_Data(0x01,0x409d);
	LTV350QV_Tx_Data(0x02,0x0204);    
	LTV350QV_Tx_Data(0x03,0x0100);
	LTV350QV_Tx_Data(0x04,0x3000);
	LTV350QV_Tx_Data(0x05,0x4003);
	LTV350QV_Tx_Data(0x06,0x000a);
	LTV350QV_Tx_Data(0x07,0x0021);
	LTV350QV_Tx_Data(0x08,0x0c00);
	LTV350QV_Tx_Data(0x10,0x0103);
	LTV350QV_Tx_Data(0x11,0x0301);
	LTV350QV_Tx_Data(0x12,0x1f0f);
	LTV350QV_Tx_Data(0x13,0x1f0f);
	LTV350QV_Tx_Data(0x14,0x0707);
	LTV350QV_Tx_Data(0x15,0x0307);
	LTV350QV_Tx_Data(0x16,0x0707);
	LTV350QV_Tx_Data(0x17,0x0000);
	LTV350QV_Tx_Data(0x18,0x0004);
	LTV350QV_Tx_Data(0x19,0x0000);
	rLCDCON1 |= 1;
	mdelay(100);
	LTV350QV_Tx_Data(0x09,0x4a55);
	LTV350QV_Tx_Data(0x05,0x5003);
}

void LTV350QV_Off(void)
{
	LTV350QV_Tx_Data(0x09,0x4055);
	LTV350QV_Tx_Data(0x05,0x4003);    
	mdelay(10);
	LTV350QV_Tx_Data(0x0a,0x0000);
	mdelay(10);
	LTV350QV_Tx_Data(0x09,0x4000);
	mdelay(10);

	// Stop display controller
	rLCDCON1 = 0;

	IO_Activate(LCD_RESET);
	IO_Deactivate(LCD_SCL);
	IO_Deactivate(LCD_SDI);
	IO_Activate(LCD_CS);
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
struct lcd_screen_info ttgfb_ltv350qv_lcdspecs = {"SAM", "LTV350QV", 70, 53, 320, 240, 25, 75, LTV350QV_Init, LTV350QV_Off, NULL, 
						   PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW_320240, VBPD_320240, VFPD_320240,
						   HSPW_320240, HBPD_320240, HFPD_320240}}, lcd_def_freq_policy, 
						   lcd_def_freq_trans};
#else
struct lcd_screen_info ttgfb_ltv350qv_lcdspecs = {"SAM", "LTV350QV", 70, 53, 320, 240, 25, 75, LTV350QV_Init, LTV350QV_Off, NULL,
						   PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW_320240, VBPD_320240, VFPD_320240,
						   HSPW_320240, HBPD_320240, HFPD_320240}}};
#endif

/* EOF */
