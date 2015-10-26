/*
 *  pmic.h  -- Backlight driver for LTC3577
 *
 *  Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 *  Author: Andrzej Zukowski <andrzej.zukowski@tomtom.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef __LINUX_MFD_LTC3577_BL_H_
#define __LINUX_MFD_LTC3577_BL_H_

#define LTC3577_MAX_INTENSITY	100
#define LTC3577_DFT_INTENSITY	35

#define LTC3577_LED_SHIFT	(0)
#define LTC3577_DAC_SHIFT	(8)
#define LTC3577_PWM_SHIFT	(14)
#define LTC3577_LED_MASK	(0xff)
#define LTC3577_DAC_MASK	(0x3f)
#define LTC3577_PWM_MASK	(0xff)

struct ltc3577_bl_data {
	uint8_t  max_brightness;
	uint8_t  dft_brightness;
	uint8_t  pwm;
	uint8_t  gr;
	uint8_t  slew;
	uint32_t (*get_intensity)(int);
};

#endif /*__LINUX_MFD_LTC3577_BL_H_ */
