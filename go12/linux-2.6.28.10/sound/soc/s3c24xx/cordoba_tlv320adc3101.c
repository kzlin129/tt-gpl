/*
 * cordoba_tlv320adc3101.c  --  SoC audio for S5P6440 Cordoba with TLV320ADC3101
 *
 * File based on smdk6410_wm8990.c
 *
 * Copyright 2009, Tom Tom International.
 * Author: Niels Langendorff
 *         niels.langendorff@tomtom.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    11th Nov 2009   Initial version.
 *
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "../codecs/tlv320adc3101.h"
#include "s3c24xx-pcm.h"
#include "s3c64xx-i2s.h"

#include <asm/gpio.h>
#include <mach/map.h>
#include <plat/regs-gpio.h> 
#include <plat/gpio-cfg.h> 

#include <asm/io.h>

#include <plat/map-base.h>
#include <plat/regs-clock.h>

#include <plat/tlv320adc3101_pdata.h>


extern int tomtom_add_nashville_controls(struct snd_card *, void *);

static int cordoba_socmst_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct s3c_i2sv2_rate_calc div;
	struct clk *clk;
	unsigned int epll_out;
	int ret;
	int bfs = 256;
	int rfs = 32;

	clk = clk_get(NULL, "fout_epll");
	if (IS_ERR(clk)) {
		printk("failed to get fout_epll\n");
		return -EBUSY;
	}

	/* Choose BFS and RFS values combination that is supported by
	 * both the TLV320ADC3101 codec as well as the S5P6440 AP
	 *
	 * TLV320ADC3101 codec supports only S8, S16_LE, S24_LE.
	 * S5P6440 AP supports only S8, S16_LE & S24_LE.
	 * We implement all for completeness but only S16_LE & S24_LE bit-lengths 
	 * are possible for this AP-Codec combination.
	 */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		bfs = 16;
		rfs = 256;		/* Can take any RFS value for AP */
 		break;
 	case SNDRV_PCM_FORMAT_S16_LE:
		bfs = 32;
		rfs = 256;		/* Can take any RFS value for AP */
 		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
	case SNDRV_PCM_FORMAT_S20_3LE:
 	case SNDRV_PCM_FORMAT_S24_LE:
		bfs = 48;
		rfs = 512;		/* See Table 41-1,2 of S5P6440 UserManual */
 		break;
	default:
		return -EINVAL;
	}
 
	switch (params_rate(params)) {
	case 8000:
	case 12000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		epll_out = 49152000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		epll_out = 67738000;
		break;
	default:
		printk("%s:%d Sampling Rate %u not supported!\n", __func__, __LINE__, params_rate(params));
		return -EINVAL;
	}

	if (clk_get_rate(clk) != epll_out)
		clk_set_rate(clk, epll_out);

	clk_put(clk);

	/* Set the Codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | \
		SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* Set the AP DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, 
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | \
		SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CLKSRC_CDCLK,
					0, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	/* We use SCLK_AUDIO for basic ops in SoC-Master mode */
	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CLKSRC_MUX,
					0, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* Set the Codec BCLK(no option to set the MCLK) */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, params_rate(params) * rfs, 
		SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	s3c_i2sv2_iis_calc_rate(&div, NULL, params_rate(params),
				s3c64xx_i2s_get_clock(cpu_dai));

	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_I2SV2_DIV_RCLK, div.fs_div);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_I2SV2_DIV_PRESCALER,
							div.clk_div - 1);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct snd_soc_dapm_widget tlv320adc3101_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("MicIn", NULL),
	SND_SOC_DAPM_MIC("DMicIn", NULL),
};

/* example machine audio_mapnections */
static const struct snd_soc_dapm_route audio_map[] = {
	{"IN1_L", NULL, "MicIn"},
	{"IN1_R", NULL, "MicIn"},
	{"IN2_L", NULL, "MicIn"},
	{"IN2_R", NULL, "MicIn"},
	{"IN3_R", NULL, "MicIn"},
	{"IN3_L", NULL, "MicIn"},

	{"DMDIN", NULL, "DMicIn"},
	{"DMCLK", NULL, "DMicIn"},
};

static int cordoba_tlv320adc3101_init(struct snd_soc_codec *codec)
{
	/* Add cordoba specific widgets */
	snd_soc_dapm_new_controls(codec, tlv320adc3101_dapm_widgets,
		ARRAY_SIZE(tlv320adc3101_dapm_widgets));

	tomtom_add_nashville_controls(codec->card, codec);

	/* set up cordoba specific audio paths */
	snd_soc_dapm_add_routes(codec, audio_map,ARRAY_SIZE(audio_map));

	/* always connected */
	snd_soc_dapm_enable_pin(codec, "MicIn");
	snd_soc_dapm_enable_pin(codec, "DMicIn");

	snd_soc_dapm_nc_pin(codec, "IN1_L");
	snd_soc_dapm_nc_pin(codec, "IN1_R");

	snd_soc_dapm_nc_pin(codec, "IN2_L");
	snd_soc_dapm_nc_pin(codec, "IN3_L");

	snd_soc_dapm_enable_pin(codec, "IN2_R");
	snd_soc_dapm_enable_pin(codec, "IN3_R");

	snd_soc_dapm_enable_pin(codec, "DMDIN");
	snd_soc_dapm_enable_pin(codec, "DMCLK");

	snd_soc_dapm_sync(codec);

	return 0;
}

/*
 * TLV320ADC3101 DAI operations.
 */
static struct snd_soc_ops cordoba_ops = {
	.hw_params = cordoba_socmst_hw_params,
};

static struct snd_soc_dai_link cordoba_dai[] = {
{
	.name = "TLV320ADC3101",
	.stream_name = "ADC3101",
	.cpu_dai = &s3c64xx_i2s_dai[0],
	.codec_dai = &adc3101_dai,
	.init = cordoba_tlv320adc3101_init,
	.ops = &cordoba_ops,
},
};

static struct snd_soc_card cordoba = {
	.name = "Cordoba Capture",
	.dai_link = cordoba_dai,
	.num_links = ARRAY_SIZE(cordoba_dai),
};

static struct snd_soc_device cordoba_snd_devdata = {
	.card = &cordoba,
	.platform = &s3c24xx_soc_platform,
	.codec_dev = &soc_codec_dev_adc3101,
};

static struct platform_device *cordoba_snd_device;

static int __init cordoba_audio_init(void)
{
	int ret;


	cordoba_snd_device = platform_device_alloc("soc-audio", 1);
	if (!cordoba_snd_device)
		return -ENOMEM;

	platform_set_drvdata(cordoba_snd_device, &cordoba_snd_devdata);
	cordoba_snd_devdata.dev = &cordoba_snd_device->dev;
	ret = platform_device_add(cordoba_snd_device);

	if (ret)
		platform_device_put(cordoba_snd_device);
	
	return ret;
}

static void __exit cordoba_audio_exit(void)
{
	platform_device_unregister(cordoba_snd_device);
}

module_init(cordoba_audio_init);
module_exit(cordoba_audio_exit);

/* Module information */
MODULE_AUTHOR("Niels Langendorff <niels.langendorff@tomtom.com>");
MODULE_DESCRIPTION("ALSA SoC Cordoba TLV320ADC3101");
MODULE_LICENSE("GPL");
