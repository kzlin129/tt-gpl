/*
 * s5p6440_adau1761.c  --  SoC audio for S5P6440 Cordoba with ADAU1761
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
 *    24th July 2009   Initial version.
 *
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "../codecs/adau1761.h"
#include "../codecs/adau1761_fw.h"
#include "s3c24xx-pcm.h"
#include "s3c64xx-i2s.h"

#include <asm/gpio.h>
#include <mach/map.h>
#include <plat/regs-gpio.h> 
#include <plat/gpio-cfg.h> 

#include <asm/io.h>

#include <plat/map-base.h>
#include <plat/regs-clock.h>

#include <plat/adau1761_pdata.h>

/* define the scenarios */
#define S5P6440_AUDIO_OFF			0
#define S5P6440_STEREO_TO_HEADPHONES		1
#define S5P6440_STEREO_TO_SPEAKER		2
#define S5P6440_STEREO_TO_HP_AND_SPKR		3
#define S5P6440_CAPTURE_LINE_IN		4
#define S5P6440_CAPTURE_MIC			5
#define S5P6440_CAPTURE_MIC_SPKR_OUT		6
#define S5P6440_CAPTURE_MIC_HP_OUT		7
#define S5P6440_CAPTURE_MIC_HP_SPKR_OUT	8
#define S5P6440_CAPTURE_LINE_SPKR_OUT		9
#define S5P6440_CAPTURE_LINE_HP_OUT		10
#define S5P6440_CAPTURE_LINE_HP_SPKR_OUT	11

#ifdef CONFIG_SND_DEBUG
#define s3cdbg(x...) printk(x)
#else
#define s3cdbg(x...)
#endif

static int adau1761_rfs = CONFIG_SND_SOC_ADAU1761_INFREQ;

extern int tomtom_add_nashville_controls(struct snd_card *, void*);

static int s5p6440_socmst_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct s3c_i2sv2_rate_calc div;
	struct clk *clk;
	unsigned int epll_out;
	int bfs, rfs, psr, ret;

	/* Choose BFS and RFS values combination that is supported by
	 * both the ADAU1761 codec as well as the S5P6440 AP
	 *
	 * ADAU1761 codec supports only S8, S16_LE, S24_LE.
	 * S5P6440 AP supports only S8, S16_LE & S24_LE.
	 * We implement all for completeness but only S16_LE & S24_LE bit-lengths 
	 * are possible for this AP-Codec combination.
	 */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		bfs = 32;			/* Table 10.01-1 S5P6440 UM */
		rfs = adau1761_rfs;	/* Table 10.01-2 S5P6440 UM, 256, 384 or 768 */
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		bfs = 32;			/* Table 10.01-1 S5P6440 UM */
		rfs = adau1761_rfs;	/* Table 10.01-2 S5P6440 UM, 256, 384 or 768 */
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S24_LE:
		bfs = 48;			/* Table 10.01-1 S5P6440 UM */
		rfs = adau1761_rfs;	/* Table 10.01-2 S5P6440 UM, 256, 384 or 768 */
		break;
	default:
		return -EINVAL;
	}

	clk = clk_get(NULL, "fout_epll");
	if (IS_ERR(clk)) {
		printk("failed to get fout_epll\n");
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
		return -EINVAL;
	}

	if (clk_get_rate(clk) != epll_out)
		clk_set_rate(clk, epll_out);

	clk_put(clk);

	/* Set the Codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | \
		SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_LEFT_J);
	if (ret < 0) {
		return ret;
	}

	/* Set the AP DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, 
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | \
		SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_LEFT_J);
	if (ret < 0) {
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CLKSRC_CDCLK,
					0, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		return ret;
	}

	/* We use SCLK_AUDIO for basic ops in SoC-Master mode */
	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CLKSRC_MUX,
					0, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		return ret;
	}

	/* Set the Codec BCLK(no option to set the MCLK) */
	ret = snd_soc_dai_set_clkdiv(codec_dai, ADAU1761_BCLK_ID, bfs);
	if (ret < 0) {
		return ret;
	}

	s3c_i2sv2_iis_calc_rate(&div, NULL, params_rate(params),
				s3c64xx_i2s_get_clock(cpu_dai));

	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_I2SV2_DIV_RCLK, div.fs_div);
	if (ret < 0) {
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_I2SV2_DIV_PRESCALER,
							div.clk_div - 1);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/*
 * ADAU1761 HiFi DAI opserations.
 */
static struct snd_soc_ops s5p6440_hifi_ops = {
	.hw_params = s5p6440_socmst_hw_params,
};

static int s5p6440_scenario = 0;

static int s5p6440_get_scenario(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	s3cdbg("%s: %d , current dapm secenario %d\n", __FUNCTION__, __LINE__, 
		s5p6440_scenario);

	ucontrol->value.integer.value[0] = s5p6440_scenario;
	return 0;
}

static int set_scenario_endpoints(struct snd_soc_codec *codec, int scenario)
{
	s5p6440_scenario = scenario;

	s3cdbg("%s: %d , current dapm endpoints %d\n", __FUNCTION__, __LINE__, 
		s5p6440_scenario);
	switch (s5p6440_scenario) {
	case S5P6440_AUDIO_OFF:
		snd_soc_dapm_disable_pin(codec, "Headphones");
		snd_soc_dapm_disable_pin(codec, "Speaker");
		snd_soc_dapm_disable_pin(codec, "MonoOut");
		snd_soc_dapm_disable_pin(codec, "MicIn");
		snd_soc_dapm_disable_pin(codec, "LineIn");
		break;
	case S5P6440_STEREO_TO_HEADPHONES:
		snd_soc_dapm_enable_pin(codec, "Headphones");
		snd_soc_dapm_disable_pin(codec, "Speaker");
		snd_soc_dapm_disable_pin(codec, "MonoOut");
		snd_soc_dapm_disable_pin(codec, "MicIn");
		snd_soc_dapm_disable_pin(codec, "LineIn");
		break;
	case S5P6440_STEREO_TO_SPEAKER:
		snd_soc_dapm_disable_pin(codec, "Headphones");
		snd_soc_dapm_enable_pin(codec, "Speaker");
		snd_soc_dapm_disable_pin(codec, "MicIn");
		snd_soc_dapm_disable_pin(codec, "LineIn");
		break;
	case S5P6440_STEREO_TO_HP_AND_SPKR:
		snd_soc_dapm_enable_pin(codec, "Headphones");
		snd_soc_dapm_enable_pin(codec, "Speaker");
		snd_soc_dapm_disable_pin(codec, "MicIn");
		snd_soc_dapm_disable_pin(codec, "LineIn");
		break;
	case S5P6440_CAPTURE_MIC:
		snd_soc_dapm_disable_pin(codec, "Headphones");
		snd_soc_dapm_disable_pin(codec, "Speaker");
		snd_soc_dapm_enable_pin(codec, "MicIn");
		snd_soc_dapm_disable_pin(codec, "LineIn");
		break;
	case S5P6440_CAPTURE_LINE_IN:
		snd_soc_dapm_disable_pin(codec, "Headphones");
		snd_soc_dapm_disable_pin(codec, "Speaker");
		snd_soc_dapm_disable_pin(codec, "MicIn");
		snd_soc_dapm_enable_pin(codec, "LineIn");
		break;
	case S5P6440_CAPTURE_MIC_SPKR_OUT:
		snd_soc_dapm_disable_pin(codec, "Headphones");
		snd_soc_dapm_enable_pin(codec, "Speaker");
		snd_soc_dapm_enable_pin(codec, "MicIn");
		snd_soc_dapm_disable_pin(codec, "LineIn");
		break;
	case S5P6440_CAPTURE_MIC_HP_OUT:
		snd_soc_dapm_enable_pin(codec, "Headphones");
		snd_soc_dapm_disable_pin(codec, "Speaker");
		snd_soc_dapm_enable_pin(codec, "MicIn");
		snd_soc_dapm_disable_pin(codec, "LineIn");
		break;
	case S5P6440_CAPTURE_MIC_HP_SPKR_OUT:
		snd_soc_dapm_enable_pin(codec, "Headphones");
		snd_soc_dapm_enable_pin(codec, "Speaker");
		snd_soc_dapm_enable_pin(codec, "MicIn");
		snd_soc_dapm_disable_pin(codec, "LineIn");
		break;
	case S5P6440_CAPTURE_LINE_SPKR_OUT:
		snd_soc_dapm_disable_pin(codec, "Headphones");
		snd_soc_dapm_enable_pin(codec, "Speaker");
		snd_soc_dapm_disable_pin(codec, "MicIn");
		snd_soc_dapm_enable_pin(codec, "LineIn");
		break;
	case S5P6440_CAPTURE_LINE_HP_OUT:
		snd_soc_dapm_enable_pin(codec, "Headphones");
		snd_soc_dapm_disable_pin(codec, "Speaker");
		snd_soc_dapm_disable_pin(codec, "MicIn");
		snd_soc_dapm_enable_pin(codec, "LineIn");
		break;
	case S5P6440_CAPTURE_LINE_HP_SPKR_OUT:
		snd_soc_dapm_enable_pin(codec, "Headphones");
		snd_soc_dapm_enable_pin(codec, "Speaker");
		snd_soc_dapm_disable_pin(codec, "MicIn");
		snd_soc_dapm_enable_pin(codec, "LineIn");
		break;
	default:
		snd_soc_dapm_enable_pin(codec, "Headphones");
		snd_soc_dapm_enable_pin(codec, "Speaker");
		snd_soc_dapm_enable_pin(codec, "MicIn");
		snd_soc_dapm_enable_pin(codec, "LineIn");
		break;
	}

	snd_soc_dapm_sync(codec);
	return 0;
}

static int s5p6440_set_scenario(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (s5p6440_scenario == ucontrol->value.integer.value[0])
		return 0;

	set_scenario_endpoints(codec, ucontrol->value.integer.value[0]);
	return 1;
}

static int adau1761_dapm_speaker_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *k, int event)
{
	struct snd_soc_codec	*codec = w->codec;
	struct adau1761_priv	*cpriv = codec->private_data;
	adau1761_platform_t	*pdata = cpriv->pdata;
	s3cdbg("%s(%d) event=%d\n", __func__, __LINE__, event);
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pdata->amp_pwr_en( 1 );
	} else if (SND_SOC_DAPM_EVENT_OFF(event)) {
		pdata->amp_pwr_en( 0 );
	} else {
		printk("%s(%d): Unknown event #%d\n", __func__, __LINE__, event);
	}
	return 0;
}

static int adau1761_dapm_mic_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *k, int event)
{
	struct snd_soc_codec	*codec = w->codec;
	struct adau1761_priv	*cpriv = codec->private_data;
//	adau1761_platform_t	*pdata = cpriv->pdata;
	s3cdbg("%s(%d) event=%d\n", __func__, __LINE__, event);
	if (SND_SOC_DAPM_EVENT_ON(event)) {
//		pdata->mic_stdby_n( 1 );
	} else if (SND_SOC_DAPM_EVENT_OFF(event)) {
//		pdata->mic_stdby_n( 0 );
	} else {
		printk("%s(%d): Unknown event #%d\n", __func__, __LINE__, event);
	}
	return 0;
}

static const struct snd_soc_dapm_widget adau1761_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_MIC("MicIn", adau1761_dapm_mic_event),
	SND_SOC_DAPM_LINE("LineIn", NULL),
	SND_SOC_DAPM_SPK("Speaker", adau1761_dapm_speaker_event),
};

/* example machine audio_mapnections */
static const struct snd_soc_dapm_route audio_map[] = {
	{"Headphones", NULL, "LHPOUT"},
	{"Headphones", NULL, "RHPOUT"},

	{"Speaker", NULL, "LOUT"},
	{"Speaker", NULL, "ROUT"},

	{"LLINEIN", NULL, "LineIn"},
	{"RLINEIN", NULL, "LineIn"},

	{"LMICIN", NULL, "MicIn"},
	{"RMICIN", NULL, "MicIn"},
};

static const char *smdk_scenarios[] = {
	"Off",
	"Headphones",
	"Speaker",
	"Headphone+Speaker",
	"Capture Line In",
	"Capture Mic",
	"Capture Mic+SPK",
	"Capture Mic+HP",
	"Capture Mic+SPK+HP",
	"Capture Line+SPK",
	"Capture Line+HP",
	"Capture Line+SPK+HP",
};

static const struct soc_enum smdk_scenario_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(smdk_scenarios),smdk_scenarios),
};

static const struct snd_kcontrol_new adau1761_s5p6440_controls[] = {
	SOC_ENUM_EXT("SMDK Mode", smdk_scenario_enum[0],
		s5p6440_get_scenario, s5p6440_set_scenario),
};

/*
 * This is an example machine initialisation for a adau1761 connected to a
 * s5p6440. It is missing logic to detect hp/mic insertions and logic
 * to re-route the audio in such an event.
 */
static int s5p6440_adau1761_init(struct snd_soc_codec *codec)
{
	int i, err;

	/* Add s5p6440 specific widgets */
	snd_soc_dapm_new_controls(codec, adau1761_dapm_widgets, 
		ARRAY_SIZE(adau1761_dapm_widgets));

	/* add s5p6440 specific controls */
	for (i = 0; i < ARRAY_SIZE(adau1761_s5p6440_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&adau1761_s5p6440_controls[i],
				codec, NULL));
		if (err < 0)
			return err;
	}

	tomtom_add_nashville_controls(codec->card, codec);

	/* set up s5p6440 specific audio paths */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* always connected */
	snd_soc_dapm_enable_pin(codec, "Headphones");
	snd_soc_dapm_enable_pin(codec, "MicIn");
	snd_soc_dapm_enable_pin(codec, "LineIn");
	snd_soc_dapm_enable_pin(codec, "Speaker");

	/* not connected */
	snd_soc_dapm_nc_pin(codec, "LMICIN");
	snd_soc_dapm_nc_pin(codec, "RMICIN");
	snd_soc_dapm_nc_pin(codec, "LLINEIN");
	snd_soc_dapm_nc_pin(codec, "RLINEIN");
	snd_soc_dapm_nc_pin(codec, "LOUT");
	snd_soc_dapm_nc_pin(codec, "LHPOUT");
	snd_soc_dapm_nc_pin(codec, "ROUT");
	snd_soc_dapm_nc_pin(codec, "RHPOUT");
//	snd_soc_dapm_nc_pin(codec, "MONOOUT");

	/* set endpoints to default mode & sync with DAPM */
	set_scenario_endpoints(codec, S5P6440_CAPTURE_LINE_HP_SPKR_OUT);

	return 0;
}

static struct snd_soc_dai_link s5p6440_dai[] = {
{ /* Hifi Playback - for similatious use with voice below */
	.name = "ADAU1761",
	.stream_name = "ADAU1761 HiFi",
	.cpu_dai = &s3c64xx_i2s_dai[0],
	.codec_dai = &adau1761_dai,
	.init = s5p6440_adau1761_init,
	.ops = &s5p6440_hifi_ops,
},
};

static struct snd_soc_card s5p6440 = {
	.name = "s5p6440",
	.dai_link = s5p6440_dai,
	.num_links = ARRAY_SIZE(s5p6440_dai),
};

static struct adau1761_setup_data s5p6440_adau1761_setup = {
	.i2c_bus = 0,
	.i2c_address = 0x38, /* 0x70, addr0 = addr1 = '0' */
};

static struct snd_soc_device s5p6440_snd_devdata = {
	.card = &s5p6440,
	.platform = &s3c24xx_soc_platform,
	.codec_dev = &soc_codec_dev_adau1761,
	.codec_data = &s5p6440_adau1761_setup,
};

static struct platform_device *s5p6440_snd_device;

static int __init s5p6440_audio_init(void)
{
	int	ret;

	switch (adau1761_rfs) {
		case 256:
		case 512:
		case 768:
			break;
		default:
			return -EINVAL;
			break;
	}

	s5p6440_snd_device = platform_device_alloc("soc-audio", -1);
	if (!s5p6440_snd_device) {
		return -ENOMEM;
	}

	platform_set_drvdata(s5p6440_snd_device, &s5p6440_snd_devdata);
	s5p6440_snd_devdata.dev = &s5p6440_snd_device->dev;
	ret = platform_device_add(s5p6440_snd_device);

	if (ret)
		platform_device_put(s5p6440_snd_device);

	return ret;
}

static void __exit s5p6440_audio_exit(void)
{
	platform_device_unregister(s5p6440_snd_device);
}

module_init(s5p6440_audio_init);
module_exit(s5p6440_audio_exit);

/* Module information */
MODULE_AUTHOR("Niels Langendorff");
MODULE_DESCRIPTION("ALSA SoC ADAU1761 for S5P6440");
MODULE_LICENSE("GPL");
