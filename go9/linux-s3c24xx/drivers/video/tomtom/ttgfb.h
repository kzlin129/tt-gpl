/* drivers/video/tomtom/ttgfb.h
 *
 * TomTom GO LCD Controller Frame Buffer Driver
 * based on skeletonfb.c, sa1100fb.c, tomtomgofb.c
 *
 * Copyright (C) 2005 TomTom BV <http://www.tomtom.com/>
 * Author: Christian Daniel <cd@cdaniel.de>
 * Author: Thomas Kleffel <tk@maintech.de>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DRIVERS_VIDEO_TOMTOM_TTGFB_H
#define __DRIVERS_VIDEO_TOMTOM_TTGFB_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __KERNEL__
/* Shadows for LCD controller registers */

enum ttgfb_screen_type {
	STN_4BDS,
	STN_4BSS,
	STN_8BSS,
	TFT_LCD,
};

enum ttgfb_screen_bpp {
	STN_1BPP,
	STN_2BPP,
	STN_4BPP,
	STN_8BPP,
	STN_12BPP,
	TFT_1BPP,
	TFT_2BPP,
	TFT_4BPP,
	TFT_8BPP,
	TFT_16BPP_5551,
	TFT_16BPP_565,
	TFT_24BPP_LSB,
	TFT_24BPP_MSB,
};

struct ttgfb_screen_info {
	enum ttgfb_screen_type type;
	enum ttgfb_screen_bpp bpp;

	u32	vsize, vfpd, vbpd, vspw;
	u32	hsize, hfpd, hbpd, hspw;
	u32	refresh_max, refresh_min;

	u32 	clkval;

	u8	bswp, hwswp;
	u8 	invvclk, invvline, invvframe, invvd, invvden;
	u8	enpwren, invpwren;
	u8	enlend, invlend;
};

struct ttgfb_slot {
	struct fb_info		fbi;
	struct device		*dev;

	/* raw memory addresses */
	dma_addr_t		map_dma;	/* physical */
	u_char *		map_cpu;	/* virtual */
	u_int			map_size;

	struct ttgfb_window     window;

	u32 pseudo_pal[16];
};
#endif /* __KERNEL__ */

#ifdef __BOOTLOADER__
void TFT_Screensave(int on);
void TFT_Off(void);
void TFT_On(int memSize);
#endif

/* 
 * some LCD panel drivers want a pointer to framebuffer
 * to upload it over SPI instead of RGB interface 
 */
unsigned int ttgfb_get_base(void);
void ttgfb_stoplcd(void);
void ttgfb_startlcd(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DRIVERS_VIDEO_TOMTOM_TTGFB_H */

/* EOF */
