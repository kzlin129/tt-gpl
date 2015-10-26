/* include/barcelona/ttgfb.h
 *
 * Public interface for the S3C2443 TFT framebuffer driver
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Author: Onno Hovers <onno.hovers@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __INCLUDE_BARCELONA_BARC_TTGFB_H
#define __INCLUDE_BARCELONA_BARC_TTGFB_H

#include <linux/ioctl.h>

struct ttgfb_window
{
	unsigned short src_left;
	unsigned short src_top;
	
	unsigned short dst_left;
	unsigned short dst_top;

	unsigned short width;
	unsigned short height;

	unsigned short layer_width;
};

#define TTGFB_MAGIC                         ('F')
#define TTGFB_IOCTL_BLEND_DISABLE            _IO ( TTGFB_MAGIC, 1)
#define TTGFB_IOCTL_BLEND_COLOR_KEY          _IOW( TTGFB_MAGIC, 2, int )
#define TTGFB_IOCTL_SET_WIN                  _IOW( TTGFB_MAGIC, 3, const struct ttgfb_window * )
#define TTGFB_IOCTL_SET_ALPHA                _IOW( TTGFB_MAGIC, 4, int )
#define TTGFB_IOCTL_GET_WIN                  _IOW( TTGFB_MAGIC, 5, struct ttgfb_window * )
#define TTGFB_IOCTL_GET_LCDON                _IOR( TTGFB_MAGIC, 6, int )

#endif
