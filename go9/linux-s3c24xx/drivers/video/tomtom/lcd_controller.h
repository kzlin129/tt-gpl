/* drivers/video/tomtom/lcd_controller.h
 *
 * TomTom GO LCD Controller Frame Buffer Driver
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Author: Onno Hovers <onno.hovers@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DRIVERS_VIDEO_TOMTOM_LCD_CONTROLLER_H 
#define __DRIVERS_VIDEO_TOMTOM_LCD_CONTROLLER_H

struct lcd_screen_info;
struct fb_info;
struct ttgfb_window;

struct lcd_additional_info {
	u8  bswp, hwswp;
	u8  invvclk, invvline, invvframe, invvd, invvden;
	u8  pwren, invpwren;
	u8  enlend, invlend;
	u8  envid, mmode, mval, bpp24bl, frm565;
	
	unsigned long hozval, lineval, divider;
};

struct lcd_controller {
	void (*initialize)(struct lcd_screen_info *screeninfo, struct lcd_additional_info *addinfo);
	void (*update_base)(int layer, struct lcd_screen_info *screeninfo, struct fb_info *fbi, struct ttgfb_window *winptr);
	void (*show_color_screen)(unsigned color);
	void (*show_normal_screen)(void);
	int (*get_number_of_layers)(void);
	unsigned (*get_base)(int layer);
	void (*start_lcd)(void);
	void (*stop_lcd)(void);
	const char *(*get_id)(void);
	unsigned short (*set_clkval)( struct lcd_screen_info *screeninfo, unsigned short clkval );
	int (*ioctl)(unsigned int cmd, unsigned long arg, struct fb_info *info);
	void (*resume)(void);
};

extern struct lcd_controller s3c24xx_lcd_controller;
extern struct lcd_controller s3c2443_tft_controller;

extern struct lcd_controller *ttgfb_lcd_controller;

#ifdef __KERNEL__
#endif

#endif /* __DRIVERS_VIDEO_TOMTOM_LCD_CONTROLLER_H */
