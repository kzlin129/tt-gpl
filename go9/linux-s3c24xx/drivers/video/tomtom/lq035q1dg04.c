/* drivers/video/tomtom/lq035q1dg04.c
 *
 * Specific functionality for the Sharp LQ035Q1DG04 screen.
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
#include "lq035q1dg04.h"
#include "lcd_controller.h"

#define MVAL_USED		(0)
#define MVAL			(0)

#define USE_DATA_ENABLE

#define VBPD_320240		((17-1)&0xff)
#define VFPD_320240		((7-1)&0xff) 
#define VSPW_320240		((3-1) &0x3f)
#define HBPD_320240		((31-1)&0x3f)
#define HFPD_320240		((17-1)&0xff) 
#define HSPW_320240		((14-1)&0xff)

static void delay_1us_more(void)
{
#ifdef __KERNEL__
    udelay(1);
#else
    mdelay(1);
#endif	
}

static void LQ035Q1DG04_WriteByte(unsigned int data)
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

static void delay3usec(void)
{
	int i;
	
	for(i=0;i<75;i++)
	{
		;
	}
}


static void LQ035Q1DG04_WriteReg(unsigned short Index,unsigned short REG_DATA)
{
	unsigned char lcd_Index0 = (unsigned char)( (Index>>8) & 0xff);
	unsigned char lcd_Index1 = (unsigned char)( Index & 0xff);	
	unsigned char lcd_data0 = (unsigned char)((REG_DATA >> 8)& 0xff);
	unsigned char lcd_data1 = (unsigned char)( REG_DATA & 0xff);

	IO_Activate(LCD_SCL);
	IO_Deactivate(LCD_CS);

	IO_Activate(LCD_CS);
	LQ035Q1DG04_WriteByte(0x74);
	LQ035Q1DG04_WriteByte(lcd_Index0);     //Write register address 8 bit
	LQ035Q1DG04_WriteByte(lcd_Index1);     //Write register address 8 bit	
	IO_Deactivate(LCD_CS);
	delay3usec();

	IO_Activate(LCD_CS);
	LQ035Q1DG04_WriteByte(0x76);
	LQ035Q1DG04_WriteByte(lcd_data0);     //Write the data first 8 bit
	LQ035Q1DG04_WriteByte(lcd_data1);     //Write the data second 8 bit
	IO_Deactivate(LCD_CS);
	delay3usec();
}

static struct lcd_additional_info add_lcdspecs = {
	.invvclk = 1, .invvline = 1, .invvframe = 1, .hwswp = 1, .frm565 = 1
};

static void LQ035Q1DG04_Init(void)
{
	unsigned int vspw_cpu,hspw_cpu, hbpd_cpu, vbpd_cpu, lcm_r16_hbpd, lcm_r17_vbpd;

	vspw_cpu = ttgfb_lq035q1dg04_lcdspecs.screen.tft.vspw+1;//1
	vbpd_cpu = ttgfb_lq035q1dg04_lcdspecs.screen.tft.vbpd+1;//1	
	hspw_cpu = ttgfb_lq035q1dg04_lcdspecs.screen.tft.hspw+1;//2
	hbpd_cpu = ttgfb_lq035q1dg04_lcdspecs.screen.tft.hbpd+1;//6
	lcm_r16_hbpd = hbpd_cpu+hspw_cpu-2; //-2 defined in spec
	lcm_r17_vbpd = vbpd_cpu+vspw_cpu;
	/* Start of LCD Power On Sequence */
	
	/* Start of LCD Power On Sequence */
	add_lcdspecs.divider = (unsigned long int) get_clkval( &ttgfb_lq035q1dg04_lcdspecs,
								ttgfb_lq035q1dg04_lcdspecs.refresh_max,
								ttgfb_lq035q1dg04_lcdspecs.curr_hclk ); 

	get_hozlineval( &ttgfb_lq035q1dg04_lcdspecs, &add_lcdspecs.hozval, &add_lcdspecs.lineval );

	/* Start display controller */
	ttgfb_lcd_controller->initialize(&ttgfb_lq035q1dg04_lcdspecs, &add_lcdspecs);

#ifndef USE_DATA_ENABLE
	IO_Activate(VDEN);
#endif
	mdelay(50);							/* assure VCI meets 90% of 3.3V */
  
	IO_Activate(LCD_RESET);
	delay_1us_more();
	IO_Deactivate(LCD_RESET); //#36 REST PIN:GND->VCI
	delay_1us_more();
  
	LQ035Q1DG04_WriteReg(0x01, 0x2aef );
	LQ035Q1DG04_WriteReg(0x02, 0x0300 );
	LQ035Q1DG04_WriteReg(0x03, 0x080e );
	LQ035Q1DG04_WriteReg(0x0b, 0xd000 );
	LQ035Q1DG04_WriteReg(0x0c, 0x0005 ); /* was 3 */
	LQ035Q1DG04_WriteReg(0x0d, 0x000f );
	LQ035Q1DG04_WriteReg(0x0e, 0x2c00 );
	LQ035Q1DG04_WriteReg(0x11, 0x0001 );
#ifdef USE_DATA_ENABLE	
	LQ035Q1DG04_WriteReg(0x12, 0x0060 );//DEN mode!!
#else	
	LQ035Q1DG04_WriteReg(0x12, 0x0064 );//HS/VS mode!!
#endif	
	LQ035Q1DG04_WriteReg(0x16, (0x9f80|lcm_r16_hbpd) ); 
	LQ035Q1DG04_WriteReg(0x17, (0x0000|lcm_r17_vbpd) );
	LQ035Q1DG04_WriteReg(0x1e, 0x0000 );
	LQ035Q1DG04_WriteReg(0x28, 0x0006 );
	LQ035Q1DG04_WriteReg(0x2a, 0x0187 );
//	LQ035Q1DG04_WriteReg(0x2c, 0x888d );
	LQ035Q1DG04_WriteReg(0x30, 0x0000 );
	LQ035Q1DG04_WriteReg(0x31, 0x0103 );
	LQ035Q1DG04_WriteReg(0x32, 0x0001 );
	LQ035Q1DG04_WriteReg(0x33, 0x0501 );
	LQ035Q1DG04_WriteReg(0x34, 0x0607 );
	LQ035Q1DG04_WriteReg(0x35, 0x0406 );
	LQ035Q1DG04_WriteReg(0x36, 0x0707 );
	LQ035Q1DG04_WriteReg(0x37, 0x0305 );
	LQ035Q1DG04_WriteReg(0x3a, 0x0f0f );
	LQ035Q1DG04_WriteReg(0x3b, 0x0f02 );  
	delay_1us_more();
	ttgfb_lcd_controller->start_lcd();
	delay_1us_more();
	LQ035Q1DG04_WriteReg(0x11, 0x0000 );
	mdelay(400); //wait for more than 15 frames
	/* End of display on sequence */
}

static void LQ035Q1DG04_Off(void)
{
	LQ035Q1DG04_WriteReg(0x11, 0x0001 );
	mdelay(60); //wait for more than 3 frames
	// Stop display controller--> turn off hsync, vsync and dot_clk
	ttgfb_lcd_controller->stop_lcd();
		
	IO_Deactivate(LCD_SCL); 
	IO_Deactivate(LCD_SDI);
	IO_Activate(LCD_CS);
	//V1.1 page 17
	IO_Activate(LCD_RESET);
}

struct lcd_screen_info ttgfb_lq035q1dg04_lcdspecs = {"SHP", "LQ035Q1DG04", 70, 53, 320, 240, 25, 66, LQ035Q1DG04_Init, LQ035Q1DG04_Off, NULL, 
						   PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW_320240, VBPD_320240, VFPD_320240,
						   HSPW_320240, HBPD_320240, HFPD_320240}}
#if defined (CONFIG_CPU_FREQ)  && defined (CONFIG_S3C24XX_DFS_CPUFREQ)
						   ,lcd_def_freq_policy,lcd_def_freq_trans
#endif
};
/* EOF */

