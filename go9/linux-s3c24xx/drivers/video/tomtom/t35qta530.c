/* drivers/video/tomtom/t35qta530.c
 *
 * Specific functionality for the Wistron T35QTA530 screen.
 *
 * ######## NOTE #############
 * Based on preliminary specification T35QTA530 Rev 03 (2010-7-28)
 *
 * ###########################
 *
 * Copyright (C) 2010 TomTom BV <http://www.tomtom.com/>
 * Author: Will Lin <Will.Lin@tomtom.com>
 *         Kwok Wong <Kwok.Wong@tomtom.com>
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
#include "t35qta530.h"
#include "lcd_controller.h"

#define USE_DATA_ENABLE
#define FRAME 17 //60Hz yield 16.7ms/frame

//VIDCON0
#define VCLKEN 1
#define ENVID 1
#define ENVID_F 1

#define CLKDIR 1  //clock is divided using CLKVAL_F
#define PNRMODE 0 //RGB parallel format

//VIDCON1
#define INVVCLK		1 //The video data is fetched at VCLK rising edge
#define INVVLINE	1 //HSYNC pulse polarity is active low
#define INVVFRAME	1 //VSYNC pulse polarity is active low
#define INVVD		1 //VDEN signal polarity is active low

// VIDTCON0
#define VBPD		13
#define VFPD		3
#define VSPW		3

// VIDTCON1
#define HBPD		53
#define HFPD		19
#define HSPW		13

//WINCON0
#define BSWP		0
#define HWSWP		1
#define BURST_LEN 0 //16 words burst
#define BPPMODE_F 5 //16bpp:565
#define ENWIN_F 1 //enable video output and control signals

//WPALOCN
#define W1PAL 6 //16-bit(5:6:5)
#define W0PAL 6 //16-bit(5:6:5)

static struct lcd_additional_info add_lcdspecs = { 
	.bswp = BSWP, .hwswp = HWSWP, .invvclk = INVVCLK, .invvline = INVVLINE,
	.invvframe = INVVFRAME, .invvd = INVVD, .invvden = INVVD,
	.envid = ENVID,
	.frm565 = 1 
};


static void T35QTA530_ALT_TFT_WriteByte(unsigned char data)
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


#ifdef __KERNEL__
#define delay3usec()		udelay(3)
#endif
#ifdef __BOOTLOADER__
#define delay3usec()		mdelay(1)
#endif

static void T35QTA530_ALT_TFT_Tx_Data(unsigned char Index,unsigned short REG_DATA)
{
	unsigned char DeviceID = 0x70;
	unsigned char lcd_Index = (unsigned char)(Index & 0xff);
	unsigned char lcd_data0 = (unsigned char)((REG_DATA >> 8)& 0xff);
	unsigned char lcd_data1 = (unsigned char)( REG_DATA & 0xff);

	IO_Activate(LCD_SCL);
	IO_Deactivate(LCD_CS);

	IO_Activate(LCD_CS);
	T35QTA530_ALT_TFT_WriteByte(DeviceID|0x0);      //Send Device ID code
	T35QTA530_ALT_TFT_WriteByte(0x00);	     //Write register address 8 bit
	T35QTA530_ALT_TFT_WriteByte(lcd_Index);     //Write register address 8 bit
	IO_Deactivate(LCD_CS);
	delay3usec();

	IO_Activate(LCD_CS);
	T35QTA530_ALT_TFT_WriteByte(DeviceID|0x2);      //Send Device ID code
	T35QTA530_ALT_TFT_WriteByte(lcd_data0);     //Write the data first 8 bit
	T35QTA530_ALT_TFT_WriteByte(lcd_data1);     //Write the data second 8 bit
	IO_Deactivate(LCD_CS);
	delay3usec();                
} 

void T35QTA530_ALT_TFT_Init(void)
{
	add_lcdspecs.divider = (unsigned long int) get_clkval( &ttgfb_t35qta530_lcdspecs,
								ttgfb_t35qta530_lcdspecs.refresh_max,
								ttgfb_t35qta530_lcdspecs.curr_hclk );
	/* Get HOZVAL and LINEVAL. */
	get_hozlineval( &ttgfb_t35qta530_lcdspecs, &add_lcdspecs.hozval, &add_lcdspecs.lineval );

	/* Start of LCD Power On Sequence */
	IO_Deactivate(LCD_RESET);
	mdelay(1);
	IO_Activate(LCD_RESET);
	mdelay(1);
	IO_Deactivate(LCD_RESET);
	mdelay(1); // > 10us

	/* Start display controller */
	ttgfb_lcd_controller->initialize(&ttgfb_t35qta530_lcdspecs, &add_lcdspecs);
	ttgfb_lcd_controller->start_lcd(); /* Dotclk/sync turn on */

#ifndef USE_DATA_ENABLE
  IO_Activate(VDEN);
#endif
    
    mdelay(1); // > 10us

    T35QTA530_ALT_TFT_Tx_Data(0x01,0x6300);
    T35QTA530_ALT_TFT_Tx_Data(0x02,0x0200);
    T35QTA530_ALT_TFT_Tx_Data(0x03,0x6364); 
    T35QTA530_ALT_TFT_Tx_Data(0x04,0x0441); 
    T35QTA530_ALT_TFT_Tx_Data(0x05,0xB444);
    T35QTA530_ALT_TFT_Tx_Data(0x0A,0x4008);
    T35QTA530_ALT_TFT_Tx_Data(0x0B,0xD400);
    T35QTA530_ALT_TFT_Tx_Data(0x0D,0x3229);
    T35QTA530_ALT_TFT_Tx_Data(0x0E,0x3200);
    T35QTA530_ALT_TFT_Tx_Data(0x0F,0x0000);
	T35QTA530_ALT_TFT_Tx_Data(0x16,0x9f80); 
	T35QTA530_ALT_TFT_Tx_Data(0x17,0x2212); /* HBP=68, VBP=18  */
    T35QTA530_ALT_TFT_Tx_Data(0x1E,0x0052);
    
    //gamma
    T35QTA530_ALT_TFT_Tx_Data(0x30,0x0000);
    T35QTA530_ALT_TFT_Tx_Data(0x31,0x0407);
    T35QTA530_ALT_TFT_Tx_Data(0x32,0x0202);
    T35QTA530_ALT_TFT_Tx_Data(0x33,0x0000);
    T35QTA530_ALT_TFT_Tx_Data(0x34,0x0505);
    T35QTA530_ALT_TFT_Tx_Data(0x35,0x0003);
    T35QTA530_ALT_TFT_Tx_Data(0x36,0x0707);
    T35QTA530_ALT_TFT_Tx_Data(0x37,0x0000);
#if 0 /* Gamma 2.2 */
    T35QTA530_ALT_TFT_Tx_Data(0x3A,0x0400);
#else /* Gamma 2.8 */
    T35QTA530_ALT_TFT_Tx_Data(0x3A,0x090A);
#endif
    T35QTA530_ALT_TFT_Tx_Data(0x3B,0x0904);
    mdelay(4*FRAME);  //Delay 4 frames to show Black Pattern before backlight on
}

void T35QTA530_ALT_TFT_Off(void)
{
    T35QTA530_ALT_TFT_Tx_Data(0x03,0x6360); /* AP[2:0]=000 */
    mdelay(6*FRAME); //tshut-off about 6 frames
    
	// Stop display controller
	ttgfb_lcd_controller->stop_lcd();

    mdelay(FRAME);
	IO_Activate(LCD_RESET);
	IO_Deactivate(LCD_SCL);
	IO_Deactivate(LCD_SDI);
	IO_Activate(LCD_CS);
}
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ 
struct lcd_screen_info ttgfb_t35qta530_lcdspecs = {"WTN", "T35QTA530", 70, 53, 320, 240, 65, 85, T35QTA530_ALT_TFT_Init, T35QTA530_ALT_TFT_Off, NULL, 
						   PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD,
						   HSPW, HBPD, HFPD}}, lcd_def_freq_policy, 
						   lcd_def_freq_trans};
#else
struct lcd_screen_info ttgfb_t35qta530_lcdspecs = {"WTN", "T35QTA530", 70, 53, 320, 240, 65, 85, T35QTA530_ALT_TFT_Init, T35QTA530_ALT_TFT_Off, NULL,
						   PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW, VBPD, VFPD,
						   HSPW, HBPD, HFPD}}};
#endif

/* EOF */

