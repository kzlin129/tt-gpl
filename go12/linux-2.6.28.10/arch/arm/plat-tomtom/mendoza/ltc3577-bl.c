/*
 *  Backlight PLATFORM DEVICE using the ltc3577 chip interface
 *
 *  Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 *  Authors: Andrzej Zukowski <andrzej.zukowski@tomtom.com>
 *           Benoit Leffray <benoit.leffray@tomtom.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/mfd/ltc3577/bl.h>

#define LTC3577_EN		(0x1 << 0)
#define LTC3577_DIS		(0x0 << 0)
#define LTC3577_GR_15		(0x0 << 1)
#define LTC3577_GR_460		(0x1 << 1)
#define LTC3577_GR_930		(0x2 << 1)
#define LTC3577_GR_1850		(0x3 << 1)
#define LTC3577_MD_CC		(0x0 << 3)
#define LTC3577_MD_HV		(0x1 << 3)
#define LTC3577_MD_PWM		(0x2 << 3)
#define LTC3577_PWM_8770	(0x0 << 5)
#define LTC3577_PWM_4390	(0x1 << 5)
#define LTC3577_PWM_2920	(0x2 << 5)
#define LTC3577_PWM_2190	(0x3 << 5)
#define LTC3577_SLEW_FAST	(0x0 << 7)
#define LTC3577_SLEW_SLOW	(0x1 << 7)

#define LUT_ENTRY(c,d,p)	((c & LTC3577_LED_MASK) << LTC3577_LED_SHIFT)|\
				((d & LTC3577_DAC_MASK) << LTC3577_DAC_SHIFT)|\
				((p & LTC3577_PWM_MASK) << LTC3577_PWM_SHIFT)

#define LTC3577_GET_LED(r)	((r >> LTC3577_LED_SHIFT) & LTC3577_LED_MASK)
#define LTC3577_GET_DAC(r)	((r >> LTC3577_DAC_SHIFT) & LTC3577_DAC_MASK)
#define LTC3577_GET_PWM(r)	((r >> LTC3577_PWM_SHIFT) & LTC3577_PWM_MASK)

static uint32_t ltc3577_bl_lut[] =
{
	LUT_ENTRY ((LTC3577_DIS | LTC3577_MD_CC ), 0x19, 0xff), //   0 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_CC ), 0x24, 0xff), //   5 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_CC ), 0x2a, 0xff), //  10 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_CC ), 0x2d, 0xff), //  15 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_CC ), 0x30, 0xff), //  20 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_PWM), 0x3f, 0x4f), //  25 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_CC ), 0x33, 0xff), //  30 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_PWM), 0x3e, 0x6f), //  35 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_CC ), 0x36, 0xff), //  40 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_PWM), 0x3f, 0x7f), //  45 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_CC ), 0x38, 0xff), //  50 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_CC ), 0x39, 0xff), //  55 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_PWM), 0x3f, 0x9f), //  60 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_PWM), 0x3f, 0xaf), //  65 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_CC ), 0x3b, 0xff), //  70 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_PWM), 0x3f, 0xbf), //  75 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_CC ), 0x3c, 0xff), //  80 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_PWM), 0x3f, 0xcf), //  85 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_PWM), 0x3f, 0xdf), //  90 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_CC ), 0x3e, 0xff), //  95 %
	LUT_ENTRY ((LTC3577_EN  | LTC3577_MD_CC ), 0x3f, 0xff), // 100 %
};

static uint32_t ltc3577_get_intensity(int brightness)
{
	if (brightness > LTC3577_MAX_INTENSITY)
		brightness = LTC3577_MAX_INTENSITY;

	if (brightness < 0)
		brightness = 0;

	return ltc3577_bl_lut[brightness/5];
}

static struct ltc3577_bl_data pdata = {
	.max_brightness	= LTC3577_MAX_INTENSITY,
	.dft_brightness	= LTC3577_DFT_INTENSITY,
	.pwm		= LTC3577_PWM_8770,
	.gr		= LTC3577_GR_15,
	.slew		= LTC3577_SLEW_SLOW,
	.get_intensity	= ltc3577_get_intensity,
};

struct platform_device mendoza_ltc3577_bl = {
	.name		= "ltc3577-bl",
	.id		= -1,
	.dev		= {
		.platform_data	= &pdata,
	},
};
