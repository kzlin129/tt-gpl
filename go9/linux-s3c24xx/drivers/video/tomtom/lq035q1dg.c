/* drivers/video/tomtom/lq035q1dg.c
 *
 * Specific functionality for the Sharp LQ035Q1DG screen.
 *
 * Copyright (C) 2005 TomTom BV <http://www.tomtom.com/>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
 * Author: Martin Jackson <martin.jackson@tomtom.com>
 * Author: Mark-Jan Bastian <mark-jan.bastian@tomtom.com>
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
#include "lq035q1dg.h"
#include "lcd_controller.h"

#define MVAL_USED		(0)
#define MVAL			(0)


#define VBPD_320240		((1-1)&0xff)
#define VFPD_320240		((2-1)&0xff) // vertical front porch always 2 for this panel: cannot be changed
#define VSPW_320240		((1-1) &0x3f)
#define HBPD_320240		((6-1)&0x3f)
#define HFPD_320240		((8-1)&0xff) // vertical front porch always 8 for this panel: cannot be changed
#define HSPW_320240		((2-1)&0xff)

#define USE_9BIT_SPI

#ifndef USE_9BIT_SPI
static void LQ035Q1DG_WriteByte(unsigned int data)
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
#endif

#ifdef USE_9BIT_SPI
static void LQ035Q1DG_Write9bits(unsigned int data)
{
	int bit;

	for (bit=0;bit<9;bit++)
	{
		IO_Deactivate(LCD_SCL);
		if (data & 0x100) IO_Activate(LCD_SDI); else IO_Deactivate(LCD_SDI);
		IO_Activate(LCD_SCL);
		data <<= 1;
	}
}
#endif

static void delay3usec(void)
{
	int i;
	
	for(i=0;i<75;i++)
	{
		;
	}
}


static void LQ035Q1DG_WriteReg(unsigned short Index,unsigned short REG_DATA)
{
	unsigned char lcd_Index0 = (unsigned char)( (Index>>8) & 0xff);
	unsigned char lcd_Index1 = (unsigned char)( Index & 0xff);	
	unsigned char lcd_data0 = (unsigned char)((REG_DATA >> 8)& 0xff);
	unsigned char lcd_data1 = (unsigned char)( REG_DATA & 0xff);

#ifdef USE_9BIT_SPI
	IO_Activate(LCD_SCL);
	IO_Deactivate(LCD_CS);

	IO_Activate(LCD_CS);
	LQ035Q1DG_Write9bits(lcd_Index0);
	IO_Deactivate(LCD_CS);
	delay3usec();
	IO_Activate(LCD_CS);
	LQ035Q1DG_Write9bits(lcd_Index1);
	IO_Deactivate(LCD_CS);
	delay3usec();

	IO_Activate(LCD_CS);
	LQ035Q1DG_Write9bits(lcd_data0|(1<<8));     //Write the data first 8 bit
	IO_Deactivate(LCD_CS);
	delay3usec();
	IO_Activate(LCD_CS);	
	LQ035Q1DG_Write9bits(lcd_data1|(1<<8));     //Write the data second 8 bit
	IO_Deactivate(LCD_CS);
	delay3usec();

#else //24bit SPI
	IO_Activate(LCD_SCL);
	IO_Deactivate(LCD_CS);

	IO_Activate(LCD_CS);
	LQ035Q1DG_WriteByte(0x70);
	LQ035Q1DG_WriteByte(lcd_Index0);     //Write register address 8 bit
	LQ035Q1DG_WriteByte(lcd_Index1);     //Write register address 8 bit	
	IO_Deactivate(LCD_CS);
	delay3usec();

	IO_Activate(LCD_CS);
	LQ035Q1DG_WriteByte(0x72);
	LQ035Q1DG_WriteByte(lcd_data0);     //Write the data first 8 bit
	LQ035Q1DG_WriteByte(lcd_data1);     //Write the data second 8 bit
	IO_Deactivate(LCD_CS);
	delay3usec();
#endif
}

static struct lcd_additional_info add_lcdspecs = {
	.invvclk = 1, .invvline = 1, .invvframe = 1, .hwswp = 1, .frm565 = 1
};

static void LQ035Q1DG_Init(void)
{
	unsigned int vspw_cpu,hspw_cpu, hbpd_cpu, vbpd_cpu, lcm_r16_hbpd, lcm_r17_vbpd;

	vspw_cpu = ttgfb_lq035q1dg_lcdspecs.screen.tft.vspw+1;//1
	vbpd_cpu = ttgfb_lq035q1dg_lcdspecs.screen.tft.vbpd+1;//1	
	hspw_cpu = ttgfb_lq035q1dg_lcdspecs.screen.tft.hspw+1;//2
	hbpd_cpu = ttgfb_lq035q1dg_lcdspecs.screen.tft.hbpd+1;//6
	lcm_r16_hbpd = hbpd_cpu+hspw_cpu-2; //-2 defined in spec
	lcm_r17_vbpd = vbpd_cpu+vspw_cpu;

	/* Start of LCD Power On Sequence */
	add_lcdspecs.divider = (unsigned long int) get_clkval( &ttgfb_lq035q1dg_lcdspecs,
								ttgfb_lq035q1dg_lcdspecs.refresh_max,
								ttgfb_lq035q1dg_lcdspecs.curr_hclk ); 

	get_hozlineval( &ttgfb_lq035q1dg_lcdspecs, &add_lcdspecs.hozval, &add_lcdspecs.lineval );

	/* Start display controller */
	ttgfb_lcd_controller->initialize(&ttgfb_lq035q1dg_lcdspecs, &add_lcdspecs);

	IO_Deactivate(LCD_RESET); //set SHUT to HIGH

	LQ035Q1DG_WriteReg(0x01, 0x2aef );
	LQ035Q1DG_WriteReg(0x02, 0x0300 );
	LQ035Q1DG_WriteReg(0x03, 0x787e );
	LQ035Q1DG_WriteReg(0x0c, 0x0005 );
	LQ035Q1DG_WriteReg(0x0d, 0x0002 );
	LQ035Q1DG_WriteReg(0x0e, 0x2900 );
	LQ035Q1DG_WriteReg(0x16, (0x9f80|lcm_r16_hbpd) ); 
	LQ035Q1DG_WriteReg(0x17, (0x0000|lcm_r17_vbpd) );
	LQ035Q1DG_WriteReg(0x2e, 0xb945 );
	LQ035Q1DG_WriteReg(0x30, 0x0000 );
	LQ035Q1DG_WriteReg(0x31, 0x0707 );
	LQ035Q1DG_WriteReg(0x32, 0x0003 );
	LQ035Q1DG_WriteReg(0x33, 0x0402 );
	LQ035Q1DG_WriteReg(0x34, 0x0307 );
	LQ035Q1DG_WriteReg(0x35, 0x0000 );
	LQ035Q1DG_WriteReg(0x36, 0x0707 );
	LQ035Q1DG_WriteReg(0x37, 0x0204 );
	LQ035Q1DG_WriteReg(0x3a, 0x0d0b );
	LQ035Q1DG_WriteReg(0x3b, 0x0d0b );  
	delay3usec();
	ttgfb_lcd_controller->start_lcd();
	delay3usec();
	IO_Activate(LCD_RESET);//set SHUT to LOW 
	mdelay(300);
	/* End of display on sequence */
}

static void LQ035Q1DG_Off(void)
{
	IO_Deactivate(LCD_RESET);// Shut high
	mdelay(50);// at least 2 frames

	// Stop display controller--> turn off hsync, vsync and dot_clk
	ttgfb_lcd_controller->stop_lcd();

	IO_Activate(LCD_RESET);	//shut low
		
	IO_Deactivate(LCD_SCL);
	IO_Deactivate(LCD_SDI);
	IO_Activate(LCD_CS);
	mdelay(50);// at least 2 frames until power off
}

struct lcd_screen_info ttgfb_lq035q1dg_lcdspecs = {"SHP", "LQ035Q1DG", 70, 53, 320, 240, 25, 75, LQ035Q1DG_Init, LQ035Q1DG_Off, NULL, 
						   PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW_320240, VBPD_320240, VFPD_320240,
						   HSPW_320240, HBPD_320240, HFPD_320240}}
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
						   ,lcd_def_freq_policy,lcd_def_freq_trans
#endif
};
/* EOF */

