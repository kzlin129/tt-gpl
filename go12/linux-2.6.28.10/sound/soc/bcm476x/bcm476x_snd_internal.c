/*
 * snd_internal.c  --  SoC audio for BCM476X chip to interface with internal codec
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
 * Copyright 2005 Openedhand Ltd.
 * Copyright 2009 Broadcom Corp.
 *
 * Authors: Liam Girdwood <liam.girdwood@wolfsonmicro.com>
 *          Richard Purdie <richard@openedhand.com>
 *          Dzanh Nguyen <dzanh@broadcom.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include "../codecs/bcm476x_codec.h"
#include "bcm476x_snd_pcm.h"


// Jack functions
#define BRCM_JACK_OFF     0
#define BRCM_JACK_STEREO  1
#define BRCM_JACK_VOICE   2
#define BRCM_JACK_BT      3
#define BRCM_JACK_BT_OFF  4

#define BRCM_SPK_OFF      0
#define BRCM_SPK_STEREO   1
#define BRCM_SPK_VOICE    2
#define BRCM_SPK_BT       3
#define BRCM_SPK_BT_OFF   4


extern int tomtom_add_nashville_controls(struct snd_card *, void*);
static int snd_probe (struct platform_device *);

static int brcm_jack_func = BRCM_JACK_OFF;
static int brcm_spk_func = BRCM_SPK_OFF;

static void brcm_snd_jack_control(struct snd_soc_codec *codec)
{
    //printk (KERN_INFO "brcm_snd_jack_control: jack_func =%d\n",brcm_jack_func);
	/* set up jack connection */
	switch (brcm_jack_func) {
	case BRCM_JACK_OFF:
		/* jack removed, everything off */
		snd_soc_dapm_disable_pin(codec, "Audio In");
		snd_soc_dapm_disable_pin(codec, "Voice In");
		break;
	case BRCM_JACK_STEREO:
		snd_soc_dapm_enable_pin (codec, "Audio In");
		break;
	case BRCM_JACK_VOICE:
		snd_soc_dapm_enable_pin (codec, "Voice In");
		break;
	case BRCM_JACK_BT:
		snd_soc_dapm_enable_pin (codec, "PCM In");
		break;
	case BRCM_JACK_BT_OFF:
		snd_soc_dapm_disable_pin (codec, "PCM In");
		break;
	}
	snd_soc_dapm_sync(codec);

}

static void brcm_snd_spk_control(struct snd_soc_codec *codec)
{
    //printk (KERN_INFO "brcm_snd_spk_control: spk_func =%d\n",brcm_spk_func);
	/* set up jack connection */
	switch (brcm_spk_func) {
	case BRCM_SPK_OFF:
		snd_soc_dapm_disable_pin(codec, "Audio Out");
		snd_soc_dapm_disable_pin(codec, "Voice Out");
		break;
	case BRCM_SPK_STEREO:
		snd_soc_dapm_enable_pin (codec, "Audio Out");
		break;
	case BRCM_SPK_VOICE:
		snd_soc_dapm_enable_pin (codec, "Voice Out");
		break;
	case BRCM_SPK_BT:
		snd_soc_dapm_enable_pin (codec, "PCM Out");
		break;
	case BRCM_SPK_BT_OFF:
		snd_soc_dapm_disable_pin (codec, "PCM Out");
		break;
	}
	snd_soc_dapm_sync(codec);
}

static int brcm_snd_internal_startup(struct snd_pcm_substream *substream)
{
	//struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//struct snd_soc_codec *codec = rtd->socdev->codec;
    //printk (KERN_INFO "brcm_snd_internal_startup:\n");
	/* check the jack status at stream startup */
	//brcm_snd_jack_control(codec);
	//brcm_snd_spk_control(codec);
	return 0;
}


static int brcm_snd_internal_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	//struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	//struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;
    
    //printk (KERN_INFO "brcm_snd_internal_hw_params:\n");

#if (0)

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;
#endif

	return ret;
}
    
static int brcm_snd_internal_prepare(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->codec;

    //printk (KERN_INFO "brcm_snd_internal_prepare:\n");

    if (SNDRV_PCM_STREAM_PLAYBACK == substream->stream) 
    {
        if (substream->pcm->device == BCM476X_BT_DAI) // Handfree subdevice
        {   
            if (runtime->rate != 8000)
               return -EINVAL;

            brcm_spk_func = BRCM_SPK_BT;
        }
        else if (runtime->rate <= 16000) // Voice subdevice
        {
            if (substream->pcm->device == BCM476X_STEREO_DAI)   /* Play thru stereo path */
                brcm_spk_func = BRCM_SPK_STEREO;
            else
                brcm_spk_func = BRCM_SPK_VOICE;                 /* Play thru mono path */
        }
        else // default to stereo path
        {
            brcm_spk_func = BRCM_SPK_STEREO;
        }
	    brcm_snd_spk_control(codec);
    }
    else
    {   // capture streams
        if (substream->pcm->device == BCM476X_BT_DAI) // BT subdevice
        {
            if (runtime->rate != 8000)
               return -EINVAL;
            brcm_jack_func = BRCM_JACK_BT;
        }
        else if (runtime->rate <= 16000) // Voice subdevice
        {
            brcm_jack_func = BRCM_JACK_VOICE;
        }
        else // default to Music subdevice
        { 
            brcm_jack_func = BRCM_JACK_STEREO;
        }
	    brcm_snd_jack_control(codec);
    }
    return 0;
}

static void brcm_snd_internal_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->codec;

    //printk (KERN_INFO "brcm_snd_internal_shutdown: pcm device=%d\n", substream->pcm->device);

    /* Shutdown the component of the audio block */
    if (SNDRV_PCM_STREAM_PLAYBACK == substream->stream)
    {
        if (2 == substream->pcm->device) 
        {
            brcm_spk_func = BRCM_SPK_BT_OFF;
        }
        else
        {
            brcm_spk_func = BRCM_SPK_OFF;
        }
	    brcm_snd_spk_control(codec);
    }
    else
    {
        if (2 == substream->pcm->device) 
        {
            brcm_jack_func = BRCM_JACK_BT_OFF;
        }
        else
        {
            brcm_jack_func = BRCM_JACK_OFF;
        }
	    brcm_snd_jack_control(codec);
    }
}

static struct snd_soc_ops brcm_ops = {
	.startup = brcm_snd_internal_startup,
	.shutdown = brcm_snd_internal_shutdown,
	.hw_params = brcm_snd_internal_hw_params,
	.prepare = brcm_snd_internal_prepare,
};


/* brcm machine dapm widgets */
static const struct snd_soc_dapm_widget bcm476x_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Audio Out", NULL),
	SND_SOC_DAPM_LINE("Voice Out", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
    SND_SOC_DAPM_LINE("Line Out", NULL), 
	SND_SOC_DAPM_MIC("Audio In", NULL), 
    SND_SOC_DAPM_MIC("Voice In", NULL), 
};

/* machine audio_map to bcm-internal codec
 * sink, control, source 
 */
static const struct snd_soc_dapm_route bcm476x_snd_audio_map[] = {
	{"Audio Out", NULL, "LOUT"},
	{"Audio Out", NULL, "ROUT"},

	/* Voice jack to mono Voice Out and Voice In */
	{"Voice Out", NULL, "VOUT"},

    {"Line Out", NULL, "PCMOUT"},
	{"PCMINPUT", NULL, "Line In"},
 
	/* voice is connected to voice input */
	{"VLINPUT", NULL, "Voice In"},
	{"VRINPUT", NULL, "Voice In"},

	/* audio is connected to stereo audio input */
    {"ALINPUT", NULL, "Audio In"},
	{"ARINPUT", NULL, "Audio In"},
};

static int snd_bcm476x_init(struct snd_soc_codec *codec)
{
	/* Add nashville controls */
	tomtom_add_nashville_controls (codec->card, codec);

	/* 476x codec pins */
	snd_soc_dapm_disable_pin(codec, "LOUT");
	snd_soc_dapm_disable_pin(codec, "ROUT");
	snd_soc_dapm_disable_pin(codec, "VOUT");
	snd_soc_dapm_disable_pin(codec, "PCMOUT");
	snd_soc_dapm_disable_pin(codec, "ALINPUT");
	snd_soc_dapm_disable_pin(codec, "ARINPUT");
	snd_soc_dapm_disable_pin(codec, "VLINPUT");
	snd_soc_dapm_disable_pin(codec, "VRINPUT");
	snd_soc_dapm_disable_pin(codec, "PCMINPUT");

	/* Add brcm specific widgets */
	snd_soc_dapm_new_controls(codec, bcm476x_dapm_widgets,
				  ARRAY_SIZE(bcm476x_dapm_widgets));

	/* Set up brcm specific audio paths */
	snd_soc_dapm_add_routes(codec, bcm476x_snd_audio_map, ARRAY_SIZE(bcm476x_snd_audio_map));

	snd_soc_dapm_sync(codec);
	return 0;
}

/*
 * Voice/Music Codec DAI (dummy DAI)
 */
static struct snd_soc_dai audio_dai = {
	.name = "Audio",
	.id = 0,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
        	.rates = (SNDRV_PCM_RATE_8000  | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | 
	                 SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 ),
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
        .rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000),
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
};

static struct snd_soc_dai voice_dai = {
    .name = "Voice",
    .id = 1,
    .playback = {
        .channels_min = 1,
        .channels_max = 2,
        .rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
        .formats = SNDRV_PCM_FMTBIT_S16_LE,},
    .capture = {
        .channels_min = 1,
        .channels_max = 2,
        .rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
        .formats = SNDRV_PCM_FMTBIT_S16_LE,},
};


/*
 * BT Codec DAI
 */
static struct snd_soc_dai bt_dai = {
	.name = "Bluetooth",
	.id = 2,
	.playback = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
};

/* brcm digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link brcm_snd_codec_dai[] = {
    {
	.name = "bcm476x-audio",
	.stream_name = "stereo",
	.cpu_dai = &audio_dai,  // soc driver requires a link to a cpu dai such as i2s_dai, we dont use i2s therefore link to a dummy one
	.codec_dai = &bcm476x_dai[BCM476X_STEREO_DAI],
	.init = snd_bcm476x_init,
	.ops = &brcm_ops,
    },
    {
	.name = "bcm476x-voice",
	.stream_name = "mono",
	.cpu_dai = &voice_dai, // soc driver requires a link to a cpu dai such as i2s_dai, we dont use i2s therefore link to a dummy one
	.codec_dai = &bcm476x_dai[BCM476X_VOICE_DAI],
	.ops = &brcm_ops,
    },
    {
	.name = "bcm476x-bt",
	.stream_name = "blue-tooth",
    .cpu_dai = &bt_dai, // soc driver requires a link to a cpu dai such as i2c_dai, we dont use i2s therefore link to a dummy one
	.codec_dai = &bcm476x_dai[BCM476X_BT_DAI],
	.ops = &brcm_ops,
    },
};

/* brcm audio machine driver */
static struct snd_soc_card brcm_snd_soc_codec = {
	.name = "BRCM",
	.dai_link = &brcm_snd_codec_dai[0],
	.num_links = ARRAY_SIZE(brcm_snd_codec_dai),
};

/* brcm audio subsystem */
static struct snd_soc_device brcm_snd_devdata = {
	.card = &brcm_snd_soc_codec,
	.platform = &brcm_soc_platform,
	.codec_dev = &soc_codec_dev_bcm476x,
	//.codec_data = &brcm_bcm476x_setup,
};

static struct platform_device *brcm_device_snd;

static struct platform_driver bcm4760_driver = {
	.driver	= {
		.name   = "bcm476x-soc-0", 
        },

	.probe = snd_probe,
};

static int snd_probe (struct platform_device *pdev)
{
	int ret;

	// Must use "soc-audio" for it to probe the sound soc device
	brcm_device_snd = platform_device_alloc("soc-audio", -1);  
	if (!brcm_device_snd)
		return -ENOMEM;

	platform_set_drvdata(brcm_device_snd, &brcm_snd_devdata);
	brcm_snd_devdata.dev = &brcm_device_snd->dev;
	ret = platform_device_add(brcm_device_snd);
	
	if (ret < 0)
		return ret;

	return 0;
}

static int __init brcm_snd_internal_init(void)
{
        platform_driver_register(&bcm4760_driver);

	return 0;
}

static void __exit brcm_snd_internal_exit(void)
{
	platform_device_unregister(brcm_device_snd);
}

module_init(brcm_snd_internal_init);
module_exit(brcm_snd_internal_exit);

MODULE_DESCRIPTION("ALSA SoC BCM476X");
MODULE_LICENSE("GPL");
