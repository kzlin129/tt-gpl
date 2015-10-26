/*
 * catania_tlv320adc3101.c  --  SoC audio for S5P6440 Catania with TLV320ADC3101
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

static int catania_linein_func = 0;
static int catania_micin_func = 1;
static int catania_digimicin_func = 0;

extern int tomtom_add_nashville_controls(struct snd_card *, void *);

static void catania_ext_control(struct snd_soc_codec *codec)
{
	if (catania_linein_func)
		snd_soc_dapm_enable_pin(codec, "Line Input");
	else
		snd_soc_dapm_disable_pin(codec, "Line Input");

	if (catania_micin_func)
		snd_soc_dapm_enable_pin(codec, "Mic Input");
	else
		snd_soc_dapm_disable_pin(codec, "Mic Input");

	if (catania_digimicin_func)
		snd_soc_dapm_enable_pin(codec, "Digital Mic Input");
	else
		snd_soc_dapm_disable_pin(codec, "Digital Mic Input");

	snd_soc_dapm_sync(codec);
}

static int catania_socmst_hw_params(struct snd_pcm_substream *substream,
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
	/* Line Input */
	SND_SOC_DAPM_MIC("Line Input", NULL),
	/* Mic Input */
	SND_SOC_DAPM_MIC("Mic Input", NULL),
	/* Digital Mic Input */
	SND_SOC_DAPM_MIC("Digital Mic Input", NULL),
};

/* example machine audio_mapnections */
static const struct snd_soc_dapm_route audio_map[] = {
/* Line Input to codec dapm Inputs */
	{"IN1_L", NULL, "Line Input"},
	{"IN1_R", NULL, "Line Input"},

/* Mic Input to codec dapm Inputs */
	{"IN1_L", NULL, "Mic Input"},
	{"IN1_R", NULL, "Mic Input"},

/* Digital Mic Input to codec dapm Inputs */
	{"DMic_L", NULL, "Digital Mic Input"},
	{"DMic_R", NULL, "Digital Mic Input"},
};

static const char *linein_function[] = { "Off", "On" };
static const char *micin_function[] = { "Off", "On" };
static const char *digimicin_function[] = { "Off", "On" };

static const struct soc_enum catania_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(linein_function), linein_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(micin_function), micin_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(digimicin_function), digimicin_function),
};

static int catania_get_linein(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = catania_linein_func;

	return 0;
}

static int catania_set_linein(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (catania_linein_func == ucontrol->value.integer.value[0])
		return 0;

	catania_linein_func = ucontrol->value.integer.value[0];
	catania_ext_control(codec);

	return 1;
}

static int catania_get_micin(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = catania_micin_func;

	return 0;
}

static int catania_set_micin(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (catania_micin_func == ucontrol->value.integer.value[0])
		return 0;

	catania_micin_func = ucontrol->value.integer.value[0];
	catania_ext_control(codec);

	return 1;
}

static int catania_get_digimicin(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = catania_digimicin_func;

	return 0;
}

static int catania_set_digimicin(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (catania_digimicin_func == ucontrol->value.integer.value[0])
		return 0;

	catania_digimicin_func = ucontrol->value.integer.value[0];
	catania_ext_control(codec);

	return 1;
}

static const struct snd_kcontrol_new adc3101_catania_controls[] = {
/* Line In Jack control */
	SOC_ENUM_EXT("Line In Jack", catania_enum[0], 
			catania_get_linein, catania_set_linein),
/* Mic In Jack control */
	SOC_ENUM_EXT("MIC In Jack", catania_enum[1], 
			catania_get_micin, catania_set_micin),
/* Digital Mic In control */
	SOC_ENUM_EXT("Digital MIC In", catania_enum[2], 
			catania_get_digimicin, catania_set_digimicin),
};

static int catania_tlv320adc3101_init(struct snd_soc_codec *codec)
{
	int i, err;

	/* Add adc3101 specific controls */
	for (i = 0; i < ARRAY_SIZE(adc3101_catania_controls); i++) {
		err = snd_ctl_add(codec->card, 
				snd_soc_cnew(&adc3101_catania_controls[i], codec, NULL));
	if (err < 0)
		return err;
	}

	/* Add catania specific widgets */
	snd_soc_dapm_new_controls(codec, tlv320adc3101_dapm_widgets,
		ARRAY_SIZE(tlv320adc3101_dapm_widgets));

	tomtom_add_nashville_controls(codec->card, codec);

	/* set up catania specific audio paths */
	snd_soc_dapm_add_routes(codec, audio_map,ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);

	return 0;
}

static int catania_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->codec;

	catania_ext_control(codec);
	return 0;
}

static void catania_shutdown(struct snd_pcm_substream *substream)
{
}

/*
 * TLV320ADC3101 DAI operations.
 */
static struct snd_soc_ops catania_ops = {
	.hw_params = catania_socmst_hw_params,
	.startup = catania_startup,
	.shutdown = catania_shutdown,
};

static struct snd_soc_dai_link catania_dai[] = {
{
	.name = "TLV320ADC3101",
	.stream_name = "ADC3101",
	.cpu_dai = &s3c64xx_i2s_dai[0],
	.codec_dai = &adc3101_dai,
	.init = catania_tlv320adc3101_init,
	.ops = &catania_ops,
},
};

static struct snd_soc_card catania = {
	.name = "Catania Capture",
	.dai_link = catania_dai,
	.num_links = ARRAY_SIZE(catania_dai),
};

static struct snd_soc_device catania_snd_devdata = {
	.card = &catania,
	.platform = &s3c24xx_soc_platform,
	.codec_dev = &soc_codec_dev_adc3101,
};

static struct platform_device *catania_snd_device;

static int __init catania_audio_init(void)
{
	int ret;


	catania_snd_device = platform_device_alloc("soc-audio", 0);
	if (!catania_snd_device)
		return -ENOMEM;

	platform_set_drvdata(catania_snd_device, &catania_snd_devdata);
	catania_snd_devdata.dev = &catania_snd_device->dev;
	ret = platform_device_add(catania_snd_device);

	if (ret)
		platform_device_put(catania_snd_device);
	
	return ret;
}

static void __exit catania_audio_exit(void)
{
	platform_device_unregister(catania_snd_device);
}

module_init(catania_audio_init);
module_exit(catania_audio_exit);

/* Module information */
MODULE_AUTHOR("Niels Langendorff <niels.langendorff@tomtom.com>");
MODULE_DESCRIPTION("ALSA SoC Catania TLV320ADC3101");
MODULE_LICENSE("GPL");
