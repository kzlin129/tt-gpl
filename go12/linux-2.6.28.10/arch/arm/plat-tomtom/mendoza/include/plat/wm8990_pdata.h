/* wm8990.h
 *
 * Control driver for wm8990 .
 *
 * Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 * Authors: Niels Langendorff <niels.langendorff@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef INCLUDE_LINUX_WM8990_H
#define INCLUDE_LINUX_WM8990_H

typedef void (* asset_pin_f)(int);

typedef struct
{
	asset_pin_f codec_pwr_en;
	asset_pin_f	amp_pwr_en;

	void (*suspend)(void);
	void (*resume)(void);
	void (*config_gpio) (void);
	int (*request_gpio)  (void);
	void (*free_gpio) (void);
} wm8990_platform_t;

struct wm8990_pdata {
	unsigned int sysclk;
	unsigned int in_source;
	unsigned int out_route;
	unsigned int pll_out;
	struct work_struct resume_work;
	struct snd_soc_codec *codec;
	int dapm_state_suspend;
	struct platform_device *pdev;
	u8 pll_enable;
	wm8990_platform_t *pdata;

	/* dapm */
	u8 dapm_lineL;
	u8 dapm_lineR;
	u8 dapm_hpL;
	u8 dapm_hpR;
};

#define WM8990_DEVNAME			"WM8990-Codec"

#endif /* INCLUDE_LINUX_WM8990_H */
