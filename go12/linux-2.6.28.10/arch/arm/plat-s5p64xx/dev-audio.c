/* linux/arch/arm/plat-s3c/dev-audio.c
 *
 * Copyright 2009 Wolfson Microelectronics
 *      Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/map.h>

#include <plat/devs.h>
#include <asm/dma.h>
#include <mach/s3c-dma.h>

#include <plat/audio.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/gpio-bank-r.h>

#include <asm/mach-types.h>

static struct resource s5p64xx_iisv4_resource[] = {
	[0] = {
		.start = S5P64XX_PA_IIS_V40,
		.end   = S5P64XX_PA_IIS_V40 + 0x100 - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device s5p64xx_device_iisv4 = {
	.name		  = "s5p64xx-iis",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(s5p64xx_iisv4_resource),
	.resource	  = s5p64xx_iisv4_resource,
};
EXPORT_SYMBOL(s5p64xx_device_iisv4);

static int s5p6442_pcm_cfg_gpio(struct platform_device *pdev)
{

	/* Havana is special in sooooo many ways... */
	if (!(machine_is_havana() || machine_is_catania_s())) {
		s3c_gpio_cfgpin(S5P64XX_GPR(13), S5P64XX_GPR13_PCM_EXTCLK);
	}

	s3c_gpio_cfgpin(S5P64XX_GPR(6), S5P64XX_GPR6_PCM_SOUT);
	s3c_gpio_cfgpin(S5P64XX_GPR(7), S5P64XX_GPR7_PCM_DCLK);
	s3c_gpio_cfgpin(S5P64XX_GPR(8), S5P64XX_GPR8_PCM_SIN);
	s3c_gpio_cfgpin(S5P64XX_GPR(14), S5P64XX_GPR14_PCM_FSYNC);

	return 0;
}

struct s3c_audio_pdata s3c_pcm_pdata = {
	.cfg_gpio = s5p6442_pcm_cfg_gpio,
};

static struct resource s5p64xx_pcm_resource[] = {
	[0] = {
		.start	= S5P64XX_PA_PCM,
		.end	= S5P64XX_PA_PCM + 0x100 - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= DMACH_PCM_OUT,
		.end	= DMACH_PCM_OUT,
		.flags	= IORESOURCE_DMA,
	},
	[2] = {
		.start	= DMACH_PCM_IN,
		.end	= DMACH_PCM_IN,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device s5p64xx_device_pcm = {
	.name		= "samsung-pcm",
	.id		= 0,
	.resource	= s5p64xx_pcm_resource,
	.num_resources	= ARRAY_SIZE(s5p64xx_pcm_resource),
	.dev = {
		.platform_data = &s3c_pcm_pdata,
	},
};
EXPORT_SYMBOL(s5p64xx_device_pcm);
