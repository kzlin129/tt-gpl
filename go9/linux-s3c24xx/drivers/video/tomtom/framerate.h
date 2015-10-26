/* drivers/video/tomtom/framerate.h
 *
 * LCD screen support routines include file.
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Author: Rogier Stam <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __DRIVERS_VIDEO_TOMTOM_FRAMERATE_H
#define __DRIVERS_VIDEO_TOMTOM_FRAMERATE_H 
void get_hozlineval( struct lcd_screen_info *lcdinfo, unsigned long int *hozval, unsigned long int *lineval );
unsigned long int calc_nom( struct lcd_screen_info *screeninfo );
unsigned short get_clkval( struct lcd_screen_info *screeninfo, unsigned short framerate, unsigned long int hclk_rate );
unsigned short get_framerate( struct lcd_screen_info *screeninfo, unsigned short clkval, unsigned long int hclk_rate );
unsigned long get_hclkrate( struct lcd_screen_info *screeninfo, unsigned short clkval, unsigned short framerate );
unsigned short set_clkval( struct lcd_screen_info *screeninfo, unsigned short clkval );
unsigned short set_framerate( struct lcd_screen_info *screeninfo, unsigned short framerate, unsigned long hclk_rate );
unsigned long int get_hclkfreq( void );
#endif
