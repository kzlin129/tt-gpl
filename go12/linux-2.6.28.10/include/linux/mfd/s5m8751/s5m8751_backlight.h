/*
 * s5m8751_backlight.h  --  S5M8751 Backlight driver
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef __LINUX_MFD_S5M8751_BACKLIGHT_H_
#define __LINUX_MFD_S5M8751_BACKLIGHT_H_

/*
 * Register values
 */

#define S5M8751_WLED_CNTRL			0x14

/*
 * R20 (0x14) - WLED_CNTRL
 */
#define S5M8751_WLED_EN_MASK			0x80
#define S5M8751_WLED_EN_SHIFT			7

#define S5M8751_FREQ_MASK			0x20
#define S5M8751_FREQ_SHIFT			5

#define S5M8751_CCFDIM_MASK			0x1F
#define S5M8751_CCFDIM_SHIFT			0

struct s5m8751_backlight {
	struct platform_device *pdev;
};

struct platform_s5m8751_backlight_data {
	unsigned int max_brightness;
	unsigned int dft_brightness;
	unsigned int pwm_freq;
	int (*convert)(int brightness);
};

#endif
