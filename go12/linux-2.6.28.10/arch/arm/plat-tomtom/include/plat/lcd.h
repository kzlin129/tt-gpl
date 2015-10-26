/*
 * Copyright (C) 2012 TomTom BV <http://www.tomtom.com/>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _LINUX_LCD_H
#define _LINUX_LCD_H

#include <linux/device.h>

struct lcd_info {
	void (*lcd_on)(int v);	
};

#endif
