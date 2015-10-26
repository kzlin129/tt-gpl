/*
 * drivers/video/samsung/a043fw05v4.c
 *
 * Copyright (C) 2008 Travis Kuo <travis.kuo@tomtom.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 */
#ifndef __A043FW05V4_H__
#define __A043FW05V4_H__

// 60Hz frame-rate. so each frame has max 17 ms
#define A043FW05V4_FRAME_PERIOD_MS   17 

struct a043fw05v4_platform_data 
{
	void (*config_gpio) (void);
	int (*request_gpio)  (void);
	void (*free_gpio) (void);
	void (*power_on) (void);
	void (*power_off) (void);
	void (*disp_on) (void);
	void (*disp_off) (void);
};

struct a043fw05v4_platform_data * setup_a043fw05v4_pdata(void);

#endif
