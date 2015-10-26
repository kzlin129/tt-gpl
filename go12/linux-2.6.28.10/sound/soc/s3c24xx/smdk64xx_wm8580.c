/*
 *  smdk64xx_wm8580.c
 *
 *  Copyright (c) 2009 Samsung Electronics Co. Ltd
 *  Author: Jaswinder Singh <jassi.brar@samsung.com>
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

#include "../codecs/wm8580.h"
#include "s3c24xx-pcm.h"
#include "s3c64xx-i2s.h"

#include <linux/report.h>

#ifdef	CONFIG_PLAT_S3C64XX
#define S3C64XX_I2S_V4 2
#else
#define S3C64XX_I2S_V4 0
#endif

#ifdef CONFIG_SND_WM8580_MASTER

/* SMDK64XX has a 12MHZ crystal attached to WM8580 */
#define SMDK64XX_WM8580_FREQ 12000000

static int smdk64xx_socslv_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	unsigned int pll_out;
	int bfs, rfs, ret;

	/* The Fvco for WM8580 PLLs must fall within [90,100]MHz.
	 * This criterion can't be met if we request PLL output
	 * as {8000x256, 64000x256, 11025x256}Hz.
	 * As a wayout, we rather change rfs to a minimum value that
	 * results in (params_rate(params) * rfs), and itself, acceptable
	 * to both - the CODEC and the CPU.
	 */
	switch (params_rate(params)) {
	case 16000:
	case 22050:
	case 22025:
	case 32000:
	case 44100:
	case 48000:
	case 88200:
	case 96000:
	case 24000:
		rfs = 256;
		break;
	case 64000:
		rfs = 384;
		break;
	case 8000:
	case 11025:
	case 12000:
		rfs = 512;
		break;
	default:
		printk("%s:%d Sampling Rate %u not supported!\n", __func__, __LINE__, params_rate(params));
		EXIT(-EINVAL);
		return -EINVAL;
	}
	pll_out = params_rate(params) * rfs;

	/* Set the Codec DAI configuration */
	YELL("snd_soc_dai_set_fmt");
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		EXIT(ret);
		return ret;
	}

	/* Set the AP DAI configuration */
	YELL("snd_soc_dai_set_fmt");
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		EXIT(ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CLKSRC_CDCLK,
					0, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		EXIT(ret);
		return ret;
	}

	/* We use PCLK for basic ops in SoC-Slave mode */
	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CLKSRC_PCLK,
					0, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		EXIT(ret);
		return ret;
	}

	/* Set WM8580 to drive MCLK from it's PLLA */
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8580_MCLK,
					WM8580_CLKSRC_PLLA);
	if (ret < 0) {
		EXIT(ret);
		return ret;
	}

	/* Explicitly set WM8580-DAC to source from MCLK */
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8580_DAC_CLKSEL,
					WM8580_CLKSRC_MCLK);
	if (ret < 0) {
		EXIT(ret);
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8580_MCLKRATIO, rfs);
	if (ret < 0) {
		EXIT(ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai, WM8580_PLLA,
					SMDK64XX_WM8580_FREQ, pll_out);
	if (ret < 0) {
		EXIT(ret);
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_I2SV2_DIV_RCLK, rfs);
	if (ret < 0) {
		EXIT(ret);
		return ret;
	}

	return 0;
}
#else
static int smdk64xx_socmst_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct s3c_i2sv2_rate_calc div;
	struct clk *clk;
	unsigned int epll_out;
	int bfs, ret;

	clk = clk_get(NULL, "fout_epll");
	if (IS_ERR(clk)) {
		printk("failed to get fout_epll\n");
		EXIT(-EBUSY);
		return -EBUSY;
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
		EXIT(-EINVAL);
		return -EINVAL;
	}

	if (clk_get_rate(clk) != epll_out)
		clk_set_rate(clk, epll_out);

	clk_put(clk);

	/* Set the Codec DAI configuration */
	YELL("snd_soc_dai_set_fmt");
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0) {
		EXIT(ret);
		return ret;
	}

	/* Set the AP DAI configuration */
	YELL("snd_soc_dai_set_fmt");
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0) {
		EXIT(ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CLKSRC_CDCLK,
					0, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		EXIT(ret);
		return ret;
	}

	/* We use SCLK_AUDIO for basic ops in SoC-Master mode */
	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CLKSRC_MUX,
					0, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		EXIT(ret);
		return ret;
	}

	/* Set WM8580 to drive MCLK from MCLK Pin */
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8580_MCLK,
					WM8580_CLKSRC_MCLK);
	if (ret < 0) {
		EXIT(ret);
		return ret;
	}

	/* Explicitly set WM8580-DAC to source from MCLK */
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8580_DAC_CLKSEL,
					WM8580_CLKSRC_MCLK);
	if (ret < 0) {
		EXIT(ret);
		return ret;
	}

	s3c_i2sv2_iis_calc_rate(&div, NULL, params_rate(params),
				s3c64xx_i2s_get_clock(cpu_dai));

	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_I2SV2_DIV_RCLK, div.fs_div);
	if (ret < 0) {
		EXIT(ret);
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_I2SV2_DIV_PRESCALER,
							div.clk_div - 1);
	if (ret < 0) {
		EXIT(ret);
		return ret;
	}

	return 0;
}
#endif

/*
 * SMDK64XX WM8580 DAI operations.
 */
static struct snd_soc_ops smdk64xx_ops = {
#ifdef CONFIG_SND_WM8580_MASTER
	.hw_params = smdk64xx_socslv_hw_params,
#else
	.hw_params = smdk64xx_socmst_hw_params,
#endif
};

/* SMDK64xx Playback widgets */
static const struct snd_soc_dapm_widget wm8580_dapm_widgets_pbk[] = {
	SND_SOC_DAPM_HP("Front-L/R", NULL),
	SND_SOC_DAPM_HP("Center/Sub", NULL),
	SND_SOC_DAPM_HP("Rear-L/R", NULL),
};

/* SMDK64xx Capture widgets */
static const struct snd_soc_dapm_widget wm8580_dapm_widgets_cpt[] = {
	SND_SOC_DAPM_MIC("MicIn", NULL),
	SND_SOC_DAPM_LINE("LineIn", NULL),
};

/* SMDK-PAIFTX connections */
static const struct snd_soc_dapm_route audio_map_tx[] = {
	/* MicIn feeds AINL */
	{"AINL", NULL, "MicIn"},

	/* LineIn feeds AINL/R */
	{"AINL", NULL, "LineIn"},
	{"AINR", NULL, "LineIn"},
};

/* SMDK-PAIFRX connections */
static const struct snd_soc_dapm_route audio_map_rx[] = {
	/* Front Left/Right are fed VOUT1L/R */
	{"Front-L/R", NULL, "VOUT1L"},
	{"Front-L/R", NULL, "VOUT1R"},

	/* Center/Sub are fed VOUT2L/R */
	{"Center/Sub", NULL, "VOUT2L"},
	{"Center/Sub", NULL, "VOUT2R"},

	/* Rear Left/Right are fed VOUT3L/R */
	{"Rear-L/R", NULL, "VOUT3L"},
	{"Rear-L/R", NULL, "VOUT3R"},
};

static int smdk64xx_wm8580_init_paiftx(struct snd_soc_codec *codec)
{
	/* Add smdk64xx specific Capture widgets */
	snd_soc_dapm_new_controls(codec, wm8580_dapm_widgets_cpt,
				  ARRAY_SIZE(wm8580_dapm_widgets_cpt));

	/* Set up PAIFTX audio path */
	snd_soc_dapm_add_routes(codec, audio_map_tx, ARRAY_SIZE(audio_map_tx));

	/* All enabled by default */
	snd_soc_dapm_enable_pin(codec, "MicIn");
	snd_soc_dapm_enable_pin(codec, "LineIn");

	/* signal a DAPM event */
	snd_soc_dapm_sync(codec);

	return 0;
}

static int smdk64xx_wm8580_init_paifrx(struct snd_soc_codec *codec)
{
	/* Add smdk64xx specific Playback widgets */
	snd_soc_dapm_new_controls(codec, wm8580_dapm_widgets_pbk,
				  ARRAY_SIZE(wm8580_dapm_widgets_pbk));

	/* Set up PAIFRX audio path */
	snd_soc_dapm_add_routes(codec, audio_map_rx, ARRAY_SIZE(audio_map_rx));

	/* All enabled by default */
	snd_soc_dapm_enable_pin(codec, "Front-L/R");
	snd_soc_dapm_enable_pin(codec, "Center/Sub");
	snd_soc_dapm_enable_pin(codec, "Rear-L/R");

	/* signal a DAPM event */
	snd_soc_dapm_sync(codec);

	return 0;
}

static struct snd_soc_dai_link smdk64xx_dai[] = {
{ /* Primary Playback i/f */
	.name = "WM8580 PAIF RX",
	.stream_name = "Playback",
	.cpu_dai = &s3c64xx_i2s_dai[S3C64XX_I2S_V4],
	.codec_dai = &wm8580_dai[WM8580_DAI_PAIFRX],
	.init = smdk64xx_wm8580_init_paifrx,
	.ops = &smdk64xx_ops,
},
{ /* Primary Capture i/f */
	.name = "WM8580 PAIF TX",
	.stream_name = "Capture",
	.cpu_dai = &s3c64xx_i2s_dai[S3C64XX_I2S_V4],
	.codec_dai = &wm8580_dai[WM8580_DAI_PAIFTX],
	.init = smdk64xx_wm8580_init_paiftx,
	.ops = &smdk64xx_ops,
},
};

static struct snd_soc_machine smdk64xx = {
	.name = "smdk64xx",
	.dai_link = smdk64xx_dai,
	.num_links = ARRAY_SIZE(smdk64xx_dai),
};

static struct wm8580_setup_data smdk64xx_wm8580_setup = {
	.i2c_address = 0x1b,
};

static struct snd_soc_device smdk64xx_snd_devdata = {
	.machine = &smdk64xx,
	.platform = &s3c24xx_soc_platform,
	.codec_dev = &soc_codec_dev_wm8580,
	.codec_data = &smdk64xx_wm8580_setup,
};

static struct platform_device *smdk64xx_snd_device;

static int __init smdk64xx_audio_init(void)
{
	int ret;

	smdk64xx_snd_device = platform_device_alloc("soc-audio", -1);
	if (!smdk64xx_snd_device)
		return -ENOMEM;

	platform_set_drvdata(smdk64xx_snd_device, &smdk64xx_snd_devdata);
	smdk64xx_snd_devdata.dev = &smdk64xx_snd_device->dev;
	ret = platform_device_add(smdk64xx_snd_device);

	if (ret)
		platform_device_put(smdk64xx_snd_device);

	return ret;
}

static void __exit smdk64xx_audio_exit(void)
{
	platform_device_unregister(smdk64xx_snd_device);
}

module_init(smdk64xx_audio_init);
module_exit(smdk64xx_audio_exit);

MODULE_AUTHOR("Jaswinder Singh, jassi.brar@samsung.com");
MODULE_DESCRIPTION("ALSA SoC SMDK64XX WM8580");
MODULE_LICENSE("GPL");
