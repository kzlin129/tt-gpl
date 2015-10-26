/*
 * havana_s5m8751.c
 *
 * Copyright (C) 2009, Samsung Elect. Ltd. - Jaswinder Singh <jassisinghbrar@gmail.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "../codecs/s5m8751.h"
#include "s3c24xx-pcm.h"
#include "s3c64xx-i2s.h"

#include <asm/gpio.h>
#include <mach/map.h>
#include <plat/regs-gpio.h> 
#include <plat/gpio-cfg.h> 

#include <asm/io.h>

#include <plat/map-base.h>
#include <plat/regs-clock.h>

#ifdef CONFIG_SND_DEBUG
#define s3cdbg(x...) printk(x)
#else
#define s3cdbg(x...)
#endif

extern int tomtom_add_nashville_controls(struct snd_card *, void *);

struct snd_soc_dai *cpu_dai;
struct s3c_i2sv2_rate_calc div;

int havana_soc_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct clk *clk;
	unsigned int epll_out;
	int bfs, ret, rfs;

	cpu_dai = rtd->dai->cpu_dai;

	clk = clk_get(NULL, "fout_epll");
	if (IS_ERR(clk)) {
		printk(KERN_ERR "failed to get fout_epll\n");
		return -EBUSY;
	}

	/* Choose BFS and RFS values combination that is supported by
	 * both the S5M8751 codec as well as the S5P6440 AP
	 *
	 * S5M8751 codec supports only S16_LE, S18_3LE, S20_3LE & S24_LE.
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
		printk(KERN_ERR "%s:%d Sampling Rate %u not supported!\n",
				 __func__, __LINE__, params_rate(params));
		return -EINVAL;
	}

	if (clk_get_rate(clk) != epll_out)
		clk_set_rate(clk, epll_out);

	clk_put(clk);

	/* Set the AP DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;


	/* Set the Codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBS_CFS);
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

	ret = snd_soc_dai_set_clkdiv(codec_dai, S5M8751_BCLK, bfs);
	if (ret < 0)
		return ret;

	s3c_i2sv2_iis_calc_rate(&div, NULL, params_rate(params),
				s3c64xx_i2s_get_clock(cpu_dai));

	return 0;
}

/*
 * S5M8751 DAI operations.
 */
static struct snd_soc_ops havana_ops = {
	.hw_params = havana_soc_hw_params,
};

static const struct snd_soc_dapm_widget s5m8751_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_LINE("LineOut", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

/* machine audio_map connections */
static const struct snd_soc_dapm_route audio_map[] = {
	{"Headphones", NULL, "LH"},
	{"Headphones", NULL, "RH"},

	{"Speaker", NULL, "SPK"},

	{"LineOut", NULL, "LL"},
	{"LineOut", NULL, "RL"},
};

static int havana_s5m8751_init(struct snd_soc_codec *codec)
{
	/* Add havana specific widgets */
	snd_soc_dapm_new_controls(codec, s5m8751_dapm_widgets,ARRAY_SIZE(s5m8751_dapm_widgets));

	tomtom_add_nashville_controls(codec->card, codec);

	/* set up havana specific audio paths */
	snd_soc_dapm_add_routes(codec, audio_map,ARRAY_SIZE(audio_map));

	/* always connected */
	snd_soc_dapm_enable_pin(codec, "Headphones");
	snd_soc_dapm_enable_pin(codec, "Speaker");
	snd_soc_dapm_enable_pin(codec, "LineOut");

	snd_soc_dapm_enable_pin(codec, "RL");
	snd_soc_dapm_enable_pin(codec, "LL");
	snd_soc_dapm_enable_pin(codec, "RH");
	snd_soc_dapm_enable_pin(codec, "LH");
	snd_soc_dapm_enable_pin(codec, "SPK");

	/* signal a DAPM event */
	snd_soc_dapm_sync(codec);

	return 0;
}

static struct snd_soc_dai_link havana_dai[] = {
{
	.name = "S5M8751",
	.stream_name = "S5M8751 Playback",
	.cpu_dai = &s3c64xx_i2s_dai[0],
	.codec_dai = &s5m8751_dai,
	.init = havana_s5m8751_init,
	.ops = &havana_ops,
},
};

static struct snd_soc_card havana = {
	.name = "Havana Playback",
	.dai_link = havana_dai,
	.num_links = ARRAY_SIZE(havana_dai),
};

static struct snd_soc_device havana_snd_devdata = {
	.card = &havana,
	.platform = &s3c24xx_soc_platform,
	.codec_dev = &soc_codec_dev_s5m8751,
};

static struct platform_device *havana_snd_device;

static int __init havana_audio_init(void)
{
	int ret;


	havana_snd_device = platform_device_alloc("soc-audio", 1);
	if (!havana_snd_device)
		return -ENOMEM;

	platform_set_drvdata(havana_snd_device, &havana_snd_devdata);
	havana_snd_devdata.dev = &havana_snd_device->dev;
	ret = platform_device_add(havana_snd_device);

	if (ret)
		platform_device_put(havana_snd_device);
	
	return ret;
}

static void __exit havana_audio_exit(void)
{
	platform_device_unregister(havana_snd_device);
}

module_init(havana_audio_init);
module_exit(havana_audio_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC Havana S5M8751");
MODULE_LICENSE("GPL");
