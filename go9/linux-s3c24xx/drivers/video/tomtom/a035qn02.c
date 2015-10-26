/* drivers/video/tomtom/a035qn02.c
 *
 * Specific functionality for the AUO A035QN02 screen.
 *
 * ######## NOTE #############
 * Based on preliminary datasheet A035QN02 rev 0.4 (25-02-2008)
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
#include "a035qn02.h"
#include "lcd_controller.h"

#define MVAL_USED		(0)
#define MVAL			(0)

#define VBPD_320240		((7-1)&0xff)
#define VFPD_320240		((7-1)&0xff)
#define VSPW_320240		((3-1) &0x3f)
#define HBPD_320240		((19-1)&0x7f)
#define HFPD_320240		((17-1)&0xff)
#define HSPW_320240		((14-1)&0xff)

static void A035QN02_WriteByte(unsigned char data)
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

static void A035QN02_Tx_Data(unsigned char Index,unsigned short REG_DATA)
{
	unsigned char DeviceID = 0x70;
	unsigned char lcd_Index = (unsigned char)(Index & 0xff);
	unsigned char lcd_data0 = (unsigned char)((REG_DATA >> 8)& 0xff);
	unsigned char lcd_data1 = (unsigned char)( REG_DATA & 0xff);

	IO_Activate(LCD_SCL);
	IO_Deactivate(LCD_CS);

	IO_Activate(LCD_CS);
	A035QN02_WriteByte(DeviceID|0x0);      //Send Device ID code
	A035QN02_WriteByte(0x00);	     //Write register address 8 bit
	A035QN02_WriteByte(lcd_Index);     //Write register address 8 bit
	IO_Deactivate(LCD_CS);
	delay3usec();

	IO_Activate(LCD_CS);
	A035QN02_WriteByte(DeviceID|0x2);      //Send Device ID code
	A035QN02_WriteByte(lcd_data0);     //Write the data first 8 bit
	A035QN02_WriteByte(lcd_data1);     //Write the data second 8 bit
	IO_Deactivate(LCD_CS);
	delay3usec();                
}

static struct lcd_additional_info add_lcdspecs = {
	.invvclk = 1, .invvline = 1, .invvframe = 1, .hwswp = 1, .frm565 = 1
};

static void A035QN02_Init(void)
{
	unsigned int vspw_cpu,hspw_cpu, hbpd_cpu, vbpd_cpu, lcm_r16_hbpd, lcm_r17_vbpd;

    //Calculate vspw and 
	vspw_cpu = ttgfb_a035qn02_lcdspecs.screen.tft.vspw+1;//1
	vbpd_cpu = ttgfb_a035qn02_lcdspecs.screen.tft.vbpd+1;//1	
	hspw_cpu = ttgfb_a035qn02_lcdspecs.screen.tft.hspw+1;//2
	hbpd_cpu = ttgfb_a035qn02_lcdspecs.screen.tft.hbpd+1;//6
	lcm_r16_hbpd = hbpd_cpu+hspw_cpu-2; //-2 defined in spec
	lcm_r17_vbpd = vbpd_cpu+vspw_cpu;

	add_lcdspecs.divider = (unsigned long int) get_clkval( &ttgfb_a035qn02_lcdspecs,
		ttgfb_a035qn02_lcdspecs.refresh_max, ttgfb_a035qn02_lcdspecs.curr_hclk ); 

	get_hozlineval( &ttgfb_a035qn02_lcdspecs, &add_lcdspecs.hozval, &add_lcdspecs.lineval );

	/* Start of LCD Power On Sequence */
	IO_Deactivate(LCD_RESET);
	mdelay(20);
	IO_Activate(LCD_RESET);
	mdelay(10);
	IO_Deactivate(LCD_RESET);
	mdelay(10);

	/* Start display controller */
	ttgfb_lcd_controller->initialize(&ttgfb_a035qn02_lcdspecs, &add_lcdspecs);
	ttgfb_lcd_controller->start_lcd();

    mdelay(30); //Delay for sending SPI command after 2nd VSYNC

	A035QN02_Tx_Data(0x01,0x2AEF);
	A035QN02_Tx_Data(0x03,0x920E);
	A035QN02_Tx_Data(0x0C,0x0002);
	A035QN02_Tx_Data(0x0D,0x000C);
	A035QN02_Tx_Data(0x0E,0x3100);
 	A035QN02_Tx_Data(0x10,0x00DC);
 	A035QN02_Tx_Data(0x12,0x0064); //based one A035QN02 V7 ver08
	A035QN02_Tx_Data(0x16, (0x9f80|lcm_r16_hbpd) ); 
	A035QN02_Tx_Data(0x17, (0x0000|lcm_r17_vbpd) );

	/* old gamma settings (2.2) * /
	A035QN02_Tx_Data(0x1E,0x00A7);	
	A035QN02_Tx_Data(0x30,0x0304);
	A035QN02_Tx_Data(0x31,0x0507);
	A035QN02_Tx_Data(0x32,0x0405);
	A035QN02_Tx_Data(0x33,0x0007);
	A035QN02_Tx_Data(0x34,0x0507);
	A035QN02_Tx_Data(0x35,0x0004);
	A035QN02_Tx_Data(0x36,0x0605);
	A035QN02_Tx_Data(0x37,0x0103);
	//*/

	/* new gamma settings (2.8) */
	A035QN02_Tx_Data(0x1E, 0x00A5);
	A035QN02_Tx_Data(0x30, 0x0001);
	A035QN02_Tx_Data(0x31, 0x0503);
	A035QN02_Tx_Data(0x32, 0x0407);
	A035QN02_Tx_Data(0x33, 0x0007);
	A035QN02_Tx_Data(0x34, 0x0607);
	A035QN02_Tx_Data(0x35, 0x0305);
	A035QN02_Tx_Data(0x36, 0x0506);
	A035QN02_Tx_Data(0x37, 0x0004);
	//*/

	A035QN02_Tx_Data(0x3A,0x000F);
	A035QN02_Tx_Data(0x3B,0x000F);

    mdelay(164);  //tshut-on Typical is 164 ms
}

static void A035QN02_Off(void)
{
	A035QN02_Tx_Data(0x11,0x0001);

	mdelay(35);

	/* Stop display controller */
	ttgfb_lcd_controller->stop_lcd();

	IO_Activate(LCD_RESET);
	IO_Deactivate(LCD_SCL);
	IO_Deactivate(LCD_SDI);
	IO_Activate(LCD_CS);
}
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ 
struct lcd_screen_info ttgfb_a035qn02_lcdspecs = {"AUO", "A035QN02", 70, 53, 320, 240, 25, 75, A035QN02_Init, A035QN02_Off, NULL, 
						   PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW_320240, VBPD_320240, VFPD_320240,
						   HSPW_320240, HBPD_320240, HFPD_320240}}, lcd_def_freq_policy, 
						   lcd_def_freq_trans};
#else
struct lcd_screen_info ttgfb_a035qn02_lcdspecs = {"AUO", "A035QN02", 70, 53, 320, 240, 25, 75, A035QN02_Init, A035QN02_Off, NULL,
						   PNR_TFT, BPP_16BPP_TFT, 0, {{VSPW_320240, VBPD_320240, VFPD_320240,
						   HSPW_320240, HBPD_320240, HFPD_320240}}};
#endif

/* EOF */
