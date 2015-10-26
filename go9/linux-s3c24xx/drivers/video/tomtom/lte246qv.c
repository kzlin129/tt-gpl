/* drivers/video/tomtom/lte246qv.c
 *
 * Specific functionality for the Samsung LTE246QV screen. See:
 * http://www.samsung.com/Products/TFTLCD/Others/LTE246QV/LTE246QV.htm
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
#include "lte246qv.h"

/* HACK for framebuffer access without including ttgfb.h */
extern unsigned ttgfb_get_base(void);
void ttgfb_stoplcd(void);
void ttgfb_startlcd(void);

#define MVAL_USED		(0)
#define MVAL			(0)

#define VFPD_240320		7
#define VBPD_240320		6
#define VSPW_240320		0
#define HFPD_240320		23
#define HBPD_240320		7
#define HSPW_240320		3

static int cl = 0; /* 8 color mode */

/* internal_mode means display retains FB, LCD controller stopped */
/* once in a while, switch to RGB mode for a sec or so to prevent sticking of crystals */
static int internal_mode = 0;

static void LTE246QV_WriteByte(unsigned char data)
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

void LTE246QV_WriteWord(short data)
{
	LTE246QV_WriteByte(data >> 8);
	LTE246QV_WriteByte(data & 0xff);
}

void WriteLDI_LTE246_Index(int address)
{
	IO_Deactivate(LCD_CS);
	IO_Activate(LCD_SCL);
	IO_Activate(LCD_SDI);

	IO_Activate(LCD_CS);
		
	LTE246QV_WriteByte(0x1d << 2);
	LTE246QV_WriteWord(address);
		
	IO_Deactivate(LCD_CS); 
}

void WriteLDI_LTE246(int address, int data)
{
	WriteLDI_LTE246_Index(address);
	
	IO_Activate(LCD_CS);
	LTE246QV_WriteByte((0x1d << 2) | 2);
	LTE246QV_WriteWord(data);
	IO_Deactivate(LCD_CS);
}

static void S6D2101_PowerOn(void)
{
	/* Power Setting Sequence */
	WriteLDI_LTE246(0x11, 0x2000); // GVD=0x20 = 4.01V
	WriteLDI_LTE246(0x14, 0x1946); // VCM=0x1c=3.89V, OTP_SEL=1, VML=0x6=3.822V // 1946 gammatuning
	WriteLDI_LTE246(0x10, 0x4d00); // SAP=0x9, BT=0x5 
	WriteLDI_LTE246(0x13, 0x0040); // PON=1, PON1=0, AON=0
	mdelay(50);
	WriteLDI_LTE246(0x13, 0x0060); // PON=1, PON1=0, AON=1
	mdelay(20);
	WriteLDI_LTE246(0x13, 0x0062); // PON=1, PON1=0, AON=1, VGLON=1
	mdelay(15);
	WriteLDI_LTE246(0x13, 0x0063); // PON=1, PON1=0, AON=1, VGLON=1, VCLON=1
	mdelay(20);
	WriteLDI_LTE246(0x13, 0x0073); // PON=1, PON1=1, AON=1, VGLON=1, VCLON=1
	mdelay(150);
}

static void S6D2101_PowerOff(void)
{
	// Powerdown sequence
	WriteLDI_LTE246(0x10, 0x0500); // SAP3-0=0x0, BT2-0=0x5, SLP=0, STB=0
	WriteLDI_LTE246(0x13, 0x0063); // PON=1,PON1=1,AON=0,VGLON=1,VCLON=1
	mdelay(50);
	WriteLDI_LTE246(0x13, 0x0040); // PON=1,PON1=0,AON=0,VGLON=0,VCLON=0
	mdelay(50);
	WriteLDI_LTE246(0x13, 0x0000); // PON=0,PON1=0,AON=0,VGLON=0,VCLON=0
}

static void S6D2101_DisplayOn(void)
{
	//Display On Sequence
	WriteLDI_LTE246(0x07, 0x0016 | (cl << 3)); // BLK_S1-0=0 (float),GON=1,CL=[cl],REV=1,D1-0=0x2 (binary output)
	mdelay(100); // wait >2 frames
	WriteLDI_LTE246(0x07, 0x0017 | (cl << 3)); // BLK_s1-0=0 (float),GON=1,CL=0,REV=1,D1-0=0x3 (normal display)
	WriteLDI_LTE246(0x11, 0x2007); // VC2-0=0x7 (1.00 * VCI_R)
	WriteLDI_LTE246(0x20, 0x0000); // AD 7-0=0x00
	WriteLDI_LTE246(0x21, 0x0000); // AD16-9=0x00
	WriteLDI_LTE246_Index(0x22);
}

static void S6D2101_DisplayOff(void)
{
	// Display off sequence
	mdelay(100); // ??
	WriteLDI_LTE246(0x0b, 0x9c48); // NO2-0=0x4, SDT2-0=0x7, ECS2-0=0x0, DCR_EX=1, DCR2-0=0x1
	WriteLDI_LTE246(0x11, 0x2000); // GVD5-0=0x20 (4.01V), VC2-0=0x0 (VCI1=0.73*VCI_R)
	WriteLDI_LTE246(0x07, 0x0016 | (cl << 3)); // BLK1-0=0x0 (float), GON=1,CL=[cl],REV=1,D1-0=0x2 (Binary output)
	mdelay(100); // wait >2 frames
	WriteLDI_LTE246(0x07, 0x0004 | (cl << 3)); // BLK1-0=0x0 (float), GON=0,CL=[cl],REV=1,D1-0=0x0 (Halt)
}

static void S6D2101_InternalMode(void)
{
	WriteLDI_LTE246(0x0c, 0x0000);
	mdelay(20);
	WriteLDI_LTE246(0x20, 0x0000); // AD 7-0=0x00
	WriteLDI_LTE246(0x21, 0x0000); // AD16-9=0x00
	mdelay(20);

	/* disable LCD RGB i/f signal driving */
	ttgfb_stoplcd();
	
	internal_mode = 1;
}

static void S6D2101_ExitInternalMode(void)
{
	WriteLDI_LTE246(0x0c, 0x0110);
	mdelay(20);
	WriteLDI_LTE246(0x20, 0x0000); // AD 7-0=0x00
	WriteLDI_LTE246(0x21, 0x0000); // AD16-9=0x00
	WriteLDI_LTE246_Index(0x22);
	mdelay(20);

	/* enable LCD RGB i/f signal driving */
	ttgfb_startlcd();	

	internal_mode = 0;
}


static void S6D2101_EightColorMode(void)
{
	cl	= 1;	
	WriteLDI_LTE246(0x07, 0x0017 | (cl << 3));
	mdelay(40);
}

static void S6D2101_ExitEightColorMode(void)
{
	cl	= 0;	
	WriteLDI_LTE246(0x07, 0x0017 | (cl << 3));
}

static void S6D2101_WindowScreenInternalEightColorMode(unsigned char hsa, unsigned char hea)
{
	unsigned int addr = 0x0000;
	S6D2101_InternalMode();

	WriteLDI_LTE246(0x20, addr & 0xff);
	WriteLDI_LTE246(0x21, 0x0000); /* no addr allowed */
	WriteLDI_LTE246(0x44, (hea << 8) | hsa);

	S6D2101_EightColorMode();
}

static void S6D2101_ExitWindowScreenInternalEightColorMode(void)
{
	S6D2101_ExitEightColorMode();
	WriteLDI_LTE246(0x44, 0xef00);
	mdelay(40);

	S6D2101_ExitInternalMode();
}

static void S6D2101_Init(void)
{
	//Initializing Sequence for S6D2101
	WriteLDI_LTE246(0x01, 0x0127); // SS=1, NL=0x27 = 320 lines
	WriteLDI_LTE246(0x02, 0x0300); // INV=0x3 1-line inversion with frame inversion
	WriteLDI_LTE246(0x03, (1 << 12) | (0x3 << 4) | (0 << 3)); // BGR=1, ID10=0x3, AM=0
	WriteLDI_LTE246(0x07, 0x0004 | (cl << 3)); // BLK_s1-0=0 (float),GON=0,CL=[cl],REV=1,D1-0=0x0 (halt)
	WriteLDI_LTE246(0x08, 0x5b03); // VFP=0x5, VBP=0xb, HBP=0x003 (=8 DOTCLK)
	WriteLDI_LTE246(0x09, 0x0001); // ASG=dual
	WriteLDI_LTE246(0x0a, 0x0004); // internal refresh
	WriteLDI_LTE246(0x0b, 0x0cc8); // NO=0x0, SDT=0x3, ECS=0x1, DCR_EX=1 (dotclk), DCR=0x1
	WriteLDI_LTE246(0x0c, 0x0110); // RM=1 (RGB intf), DM=0x1 (RGB intf), RIM=0x0 (18 BPP) 
	WriteLDI_LTE246(0x30, 0x0000); // Gamma PKP12-10=0x00, PKP 2- 0=0x00 
	WriteLDI_LTE246(0x31, 0x0102); // Gamma PKP32-30=0x02, PKP22-20=0x02 // 0102 gammatuning
	WriteLDI_LTE246(0x32, 0x0101); // Gamma PKP52-50=0x01, PKP42-40=0x01
	WriteLDI_LTE246(0x33, 0x0001); // Gamma PRP12-10=0x00, PRP02-00=0x01
	WriteLDI_LTE246(0x34, 0x0505); // Gamma PKN12-10=0x05, PKN02-00=0x05
	WriteLDI_LTE246(0x35, 0x0605); // Gamma PKN32-30=0x05, PKN22-20=0x04 // 0605 gammatuning
	WriteLDI_LTE246(0x36, 0x0707); // Gamma PKN52-50=0x07, PKN42-40=0x07
	WriteLDI_LTE246(0x37, 0x0200); // Gamma PRN12-10=0x02, PRN02-00=0x00
	WriteLDI_LTE246(0x38, 0x0500); // Gamma VRP14-10=0x05, VRP04-00=0x00
	WriteLDI_LTE246(0x39, 0x0000); // Gamma VRN14-10=0x00, VRN04-00=0x00
	WriteLDI_LTE246(0x42, 0x013f); // PSE=0x13f
	WriteLDI_LTE246(0x43, 0x0000); // PSS=0x000
	WriteLDI_LTE246(0x44, 0xef00); // PSS=0x000
	WriteLDI_LTE246(0x45, 0x013f); // 
	WriteLDI_LTE246(0x46, 0x0000); // 
	WriteLDI_LTE246(0x72, 0x0402); // Testcommand2, Don't use this command
}

void LTE246QV_Screensave(int on)
{
	/* if display was already in sleep, update framebuffer contents by switching to RGB */
	if (internal_mode && on) {
		S6D2101_ExitWindowScreenInternalEightColorMode();
		mdelay(40); // dump ca two frames using RGB if
		S6D2101_WindowScreenInternalEightColorMode(120-16,120+16);
		return;
	}

	if (on) {
		S6D2101_WindowScreenInternalEightColorMode(120-16,120+16);
	} else {
		S6D2101_ExitWindowScreenInternalEightColorMode();
	}
}

void LTE246QV_Init(void)
{
	unsigned long int	divider=(unsigned long int) get_clkval( &ttgfb_lte246qv_lcdspecs,
                                                                        ttgfb_lte246qv_lcdspecs.refresh_max,
                                                                        ttgfb_lte246qv_lcdspecs.curr_hclk );
	unsigned long int	hozval;
	unsigned long int	lineval;

	/* Get HOZVAL and LINEVAL. */
	get_hozlineval( &ttgfb_lte246qv_lcdspecs, &hozval, &lineval );

	/* Start S3C display controller, LCM needs dotclock to run */
	rLCDCON1 &= ~1;
	rTCONSEL = 0;
	rLCDCON1 = (divider << 8) | (MVAL_USED << 7) | (((unsigned long int) ttgfb_lte246qv_lcdspecs.pnr_mode) << 5) |
		   (((unsigned long int) ttgfb_lte246qv_lcdspecs.bpp_mode) << 1) | (0 << 0);
        rLCDCON2 = (((unsigned long int) ttgfb_lte246qv_lcdspecs.screen.tft.vbpd) << 24) |
                   (lineval << 14) |
                   (((unsigned long int) ttgfb_lte246qv_lcdspecs.screen.tft.vfpd) << 6) |
                   (((unsigned long int) ttgfb_lte246qv_lcdspecs.screen.tft.vspw) << 0);
	rLCDCON3 = (((unsigned long int) ttgfb_lte246qv_lcdspecs.screen.tft.hbpd) << 19) |
		   (hozval << 8) |
		   (((unsigned long int) ttgfb_lte246qv_lcdspecs.screen.tft.hfpd) << 0);
	rLCDCON4 = (MVAL << 8) | (((unsigned long int) ttgfb_lte246qv_lcdspecs.screen.tft.hspw) << 0);
	rLCDCON5=(1<<11)|(1<<10)|(1<<9)|(1<<8)|(1<<6)|(1<<0);
	rLCDCON1 |= 1;

	IO_Deactivate(LCD_CS);
	IO_Activate(LCD_SCL);
	IO_Activate(LCD_SDI);

	mdelay(2);
	IO_Activate(LCD_RESET);
	mdelay(50);
	IO_Deactivate(LCD_RESET);
	mdelay(50);
	
	//Power Setting Sequence S6D2101
	S6D2101_PowerOn();
	
	mdelay(20);
	
	//Initializing Sequence for S6D2101
	S6D2101_Init();

	mdelay(200);

	//Display On Sequence
	S6D2101_DisplayOn();
}

void LTE246QV_Off(void)
{
	// Display off sequence
	S6D2101_DisplayOff();
	
	// Powerdown sequence
	S6D2101_PowerOff();
	
	// Stop display controller
	rLCDCON1 = 0;

	IO_Activate(LCD_RESET);
	IO_Deactivate(LCD_SCL);
	IO_Deactivate(LCD_SDI);
	IO_Activate(LCD_CS);
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
struct lcd_screen_info ttgfb_lte246qv_lcdspecs = {"SAM", "LTE246QV", 38, 50, 240, 320, 25, 75, LTE246QV_Init, LTE246QV_Off,
						   LTE246QV_Screensave, PNR_TFT, BPP_16BPP_TFT, 0,
						   {{VSPW_240320, VBPD_240320, VFPD_240320,
						    HSPW_240320, HBPD_240320, HFPD_240320}},
						   lcd_def_freq_policy, lcd_def_freq_trans};
#else
struct lcd_screen_info ttgfb_lte246qv_lcdspecs = {"SAM", "LTE246QV", 38, 50, 240, 320, 25, 75, LTE246QV_Init, LTE246QV_Off,
						   LTE246QV_Screensave, PNR_TFT, BPP_16BPP_TFT, 0,
						   {{VSPW_240320, VBPD_240320, VFPD_240320,
						    HSPW_240320, HBPD_240320, HFPD_240320}}};
#endif

/* EOF */
