/*
 * bc6.c  --  SoC audio for S3C64XX with BC6
 *
 * Copyright 2010, Tom Tom International.
 * Author: Niels Langendorff
 *         niels.langendorff@tomtom.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    11th Aug 2010   Initial version.
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "../codecs/bc6.h"
#include "s3c24xx-pcm.h"
#include "s3c64xx-i2s.h"
#include "s3c-pcm.h"

#ifdef	CONFIG_PLAT_S3C64XX
#define S3C64XX_I2S_V4 2
#else
#define S3C64XX_I2S_V4 0
#endif

static int s3c64xx_socpcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct clk *clk;
	struct s3c_pcm_info *s3c_pcm = (struct s3c_pcm_info *)cpu_dai->private_data;
	unsigned int epll_out;
	int ret;

	clk = clk_get(NULL, "fout_epll");
	if (IS_ERR(clk)) {
		printk(KERN_ERR "failed to get fout_epll\n");
		return -EBUSY;
	}

	switch (params_rate(params)) {
	case 8000:
	case 16000:
		epll_out = 49152000;
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
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_DSP_B
					 | SND_SOC_DAIFMT_IB_NF
					 | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* We use SCLK_AUDIO for basic ops in SoC-Master mode */
	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CLKSRC_MUX,
					0, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* s3c-pcm sets this value to 4, but with this value half of the frames are dropped */
	s3c_pcm->dma_playback->dma_size = 2;
	s3c_pcm->dma_capture->dma_size = 2;

	return 0;
}
/*
 * BC6 DAI opserations.
 */
static struct snd_soc_ops s3c64xx_pcm_ops = {
	.hw_params = s3c64xx_socpcm_hw_params,
};

/*
 * This is an example machine initialisation for a BC6 connected to a
 * s3c64xx. It is missing logic to detect hp/mic insertions and logic
 * to re-route the audio in such an event.
 */
static int s3c64xx_bc6_init(struct snd_soc_codec *codec)
{
	return 0;
}


static struct snd_soc_dai_link s3c64xx_dai[] = {
{ 
	/* PCM i/f */
	.name = "BC6 PCM IF",
	.stream_name = "Tx/Rx",
	.cpu_dai = &s3c_pcm_dai[0],
	.codec_dai = &bc6_dai,
	.init = s3c64xx_bc6_init,
	.ops = &s3c64xx_pcm_ops,
},
};

static struct snd_soc_card s3c64xx = {
	.name = "Havana",
	.dai_link = s3c64xx_dai,
	.num_links = ARRAY_SIZE(s3c64xx_dai),
};

static struct snd_soc_device s3c64xx_snd_devdata = {
	.card = &s3c64xx,
	.platform = &s3c24xx_soc_platform,
	.codec_dev = &soc_codec_dev_bc6,
};

static struct platform_device *s3c64xx_snd_device;

static int __init s3c64xx_audio_init(void)
{
	int	ret;

	s3c64xx_snd_device = platform_device_alloc("soc-audio", 2);
	if (!s3c64xx_snd_device) {
	    printk("platform_device_alloc failed %s:%d\n", __FILE__, __LINE__);
		return -ENOMEM;
	}

	platform_set_drvdata(s3c64xx_snd_device, &s3c64xx_snd_devdata);
	s3c64xx_snd_devdata.dev = &s3c64xx_snd_device->dev;
	ret = platform_device_add(s3c64xx_snd_device);

	if (ret)
		platform_device_put(s3c64xx_snd_device);

	return ret;
}

static void __exit s3c64xx_audio_exit(void)
{
	platform_device_unregister(s3c64xx_snd_device);
}

module_init(s3c64xx_audio_init);
module_exit(s3c64xx_audio_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC BC6 for S3C64XX");
MODULE_AUTHOR("Niels Langendorff <niels.langendorff@tomtom.com>");
MODULE_LICENSE("GPL");
