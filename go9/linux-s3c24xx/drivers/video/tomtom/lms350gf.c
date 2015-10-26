/* drivers/video/tomtom/lms350gf.c
 *
 * Specific functionality for the Samsung LMS350GF screen.
 *
 * ######## NOTE #############
 * Based on preliminary datasheet LMS350GF03-001 rev 001 (15-02-2007)
 *
 * ###########################
 *
 * Copyright (C) 2005 TomTom BV <http://www.tomtom.com/>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
 * Author: Martin Jackson <martin.jackson@tomtom.com>
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
#include "lms350gf.h"
#include "lcd_controller.h"

#define MVAL_USED		(0)
#define MVAL			(0)

#define VBPD_320240		((7-1)&0xff)
#define VFPD_320240		((7-1)&0xff)
#define VSPW_320240		((3-1) &0x3f)
#define HBPD_320240		((19-1)&0x7f)
#define HFPD_320240		((17-1)&0xff)
#define HSPW_320240		((14-1)&0xff)
#define DISABLE_DE		(1 << 6)
#define LMS350GF_HBP		(0x21)
#define LMS350GF_VBP		(0x8)

static void LMS350GF_WriteByte(unsigned char data)
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

static void LMS350GF_Tx_Data(unsigned char Index,unsigned short REG_DATA)
{
	unsigned char DeviceID = 0x74;
	unsigned char lcd_Index = (unsigned char)(Index & 0xff);
	unsigned char lcd_data0 = (unsigned char)((REG_DATA >> 8)& 0xff);
	unsigned char lcd_data1 = (unsigned char)( REG_DATA & 0xff);

	IO_Activate(LCD_SCL);
	IO_Deactivate(LCD_CS);

	IO_Activate(LCD_CS);
	LMS350GF_WriteByte(DeviceID|0x0);      //Send Device ID code
	LMS350GF_WriteByte(0x00);	     //Write register address 8 bit
	LMS350GF_WriteByte(lcd_Index);     //Write register address 8 bit
	IO_Deactivate(LCD_CS);
	mdelay(1);

	IO_Activate(LCD_CS);
	LMS350GF_WriteByte(DeviceID|0x2);      //Send Device ID code
	LMS350GF_WriteByte(lcd_data0);     //Write the data first 8 bit
	LMS350GF_WriteByte(lcd_data1);     //Write the data second 8 bit
	IO_Deactivate(LCD_CS);
	mdelay(1);                 //need to delay 50us
}

static struct lcd_additional_info add_lcdspecs = {
	.invvline = 1, .invvframe = 1, .hwswp = 1, .frm565 = 1
};

void LMS350GF_Init(void)
{
	add_lcdspecs.divider = (unsigned long int) get_clkval( &ttgfb_lms350gf_lcdspecs,
		ttgfb_lms350gf_lcdspecs.refresh_max, ttgfb_lms350gf_lcdspecs.curr_hclk ); 

	get_hozlineval( &ttgfb_lms350gf_lcdspecs, &add_lcdspecs.hozval, &add_lcdspecs.lineval );

	/* Start display controller */
	ttgfb_lcd_controller->initialize(&ttgfb_lms350gf_lcdspecs, &add_lcdspecs);

	/* Start of LCD Power On Sequence */
	IO_Deactivate(LCD_RESET);
	mdelay(20);
	IO_Activate(LCD_RESET);
	mdelay(10);
	IO_Deactivate(LCD_RESET);
	mdelay(10);

	LMS350GF_Tx_Data(0x07,0x0000);
	mdelay(15);
	LMS350GF_Tx_Data(0x12,0x1618);
	LMS350GF_Tx_Data(0x11,0x222f);
	LMS350GF_Tx_Data(0x13,0x43c9); // was 0x40ca
	LMS350GF_Tx_Data(0x10,0x3108);
	mdelay(100);
	LMS350GF_Tx_Data(0x12,0x1658);
	mdelay(80);
	LMS350GF_Tx_Data(0x01,0x2b1d);    
	LMS350GF_Tx_Data(0x02,0x0300);
	LMS350GF_Tx_Data(0x03,0xd000 /*| DISABLE_DE*/);//Data Enable used
	LMS350GF_Tx_Data(0x08,LMS350GF_VBP);	/* vporch is 10 */
	LMS350GF_Tx_Data(0x09,LMS350GF_HBP);	/* hporch is 33 */

	LMS350GF_Tx_Data(0x76,0x2213);
	LMS350GF_Tx_Data(0x0b,0x33e1);
	LMS350GF_Tx_Data(0x0c,0x0020);
	LMS350GF_Tx_Data(0x76,0x0000);
	LMS350GF_Tx_Data(0x0d,0x0000);
	LMS350GF_Tx_Data(0x0e,0x0000);
	LMS350GF_Tx_Data(0x14,0x0000);
	LMS350GF_Tx_Data(0x15,0x0803);
	LMS350GF_Tx_Data(0x16,0x0000);
	LMS350GF_Tx_Data(0x30,0x0200); // was 0x0209
	LMS350GF_Tx_Data(0x31,0x0404);
	LMS350GF_Tx_Data(0x32,0x0000); // was 0x0e07
	LMS350GF_Tx_Data(0x33,0x0602);
	LMS350GF_Tx_Data(0x34,0x0707);
	LMS350GF_Tx_Data(0x35,0x0707);
	LMS350GF_Tx_Data(0x36,0x0000); // was 0x0707
	LMS350GF_Tx_Data(0x37,0x0206);
	LMS350GF_Tx_Data(0x38,0x0f06);
	LMS350GF_Tx_Data(0x39,0x0611);
	/* End of LCD Power On Sequence
	Start of Display On Sequence */
	ttgfb_lcd_controller->start_lcd();
	LMS350GF_Tx_Data(0x07,0x0001);
	mdelay(30);
	LMS350GF_Tx_Data(0x07,0x0101);
	mdelay(30);
	LMS350GF_Tx_Data(0x76,0x2213);
	LMS350GF_Tx_Data(0x1c,0x6650);
	LMS350GF_Tx_Data(0x0b,0x33e1);
	LMS350GF_Tx_Data(0x76,0x0000);
	LMS350GF_Tx_Data(0x07,0x0103);
	/* End of display on sequence */
}

void LMS350GF_Off(void)
{
	/* Start of Display Off Sequence */
	LMS350GF_Tx_Data(0x76,0x2213);
	LMS350GF_Tx_Data(0x0b,0x33e1);    
	LMS350GF_Tx_Data(0x76,0x0000);
	LMS350GF_Tx_Data(0x07,0x0102);
	mdelay(30);
	LMS350GF_Tx_Data(0x07,0x0100);
	mdelay(30);
	LMS350GF_Tx_Data(0x12,0x0000);
	LMS350GF_Tx_Data(0x10,0x0100);
	/* End of Display Off Sequence
	Start of Power Off Sequence */
	LMS350GF_Tx_Data(0x07,0x0000);
	LMS350GF_Tx_Data(0x10,0x0000);
	LMS350GF_Tx_Data(0x11,0x0000);
	LMS350GF_Tx_Data(0x12,0x0000);
	/* End of Power Off Sequence */

	// Stop display controller
	ttgfb_lcd_controller->stop_lcd();

	IO_Activate(LCD_RESET);
	IO_Deactivate(LCD_SCL);
	IO_Deactivate(LCD_SDI);
	IO_Activate(LCD_CS);
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
struct lcd_screen_info ttgfb_lms350gf_lcdspecs = {"SAM", "LMS350GF", 70, 53, 320, 240, 25, 75, LMS350GF_Init, LMS350GF_Off, NULL, 
						   PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW_320240, VBPD_320240, VFPD_320240,
						   HSPW_320240, HBPD_320240, HFPD_320240}}, lcd_def_freq_policy, 
						   lcd_def_freq_trans};
#else
struct lcd_screen_info ttgfb_lms350gf_lcdspecs = {"SAM", "LMS350GF", 70, 53, 320, 240, 25, 75, LMS350GF_Init, LMS350GF_Off, NULL,
						   PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW_320240, VBPD_320240, VFPD_320240,
						   HSPW_320240, HBPD_320240, HFPD_320240}}};
#endif

/* EOF */

