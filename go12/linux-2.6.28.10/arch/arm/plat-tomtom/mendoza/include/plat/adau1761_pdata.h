/* adau1761_pdata.h
 *
 * Control driver for adau1761 .
 *
 * Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 * Authors: Niels Langendorff <niels.langendorff@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef INCLUDE_PLAT_ADAU1761_PDATA_H
#define INCLUDE_PLAT_ADAU1761_PDATA_H

typedef void (* asset_pin_f)(int);

typedef struct
{
	asset_pin_f codec_pwr_en;
	asset_pin_f	amp_pwr_en;
	asset_pin_f	mic_stby_n;

	void (*suspend)(void);
	void (*resume)(void);
	void (*config_gpio) (void);
	int (*request_gpio)  (void);
	void (*free_gpio) (void);
} adau1761_platform_t;

struct adau1761_priv {
	unsigned int in_source;
	unsigned int out_route;
	struct work_struct resume_work;
	struct snd_soc_codec *codec;
	int dapm_state_suspend;
	struct platform_device *pdev;
	adau1761_platform_t *pdata;

	/* dapm */
	u8 dapm_lineL;
	u8 dapm_lineR;
	u8 dapm_hpL;
	u8 dapm_hpR;
};

#define ADAU1761_DEVNAME			"ADAU1761-Codec"

#endif /* INCLUDE_PLAT_ADAU1761_PDATA_H */
