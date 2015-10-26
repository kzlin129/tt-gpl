/*
 * linux/sound/arm/snd_pcm.c -- ALSA PCM interface for the BCM476X chip
 *
 * Author:	Dzanh Nguyen
 * Created:	Apr 10, 2009
 * Copyright:	(C) 2009 Broadcom Corp.

 * Author:	Nicolas Pitre
 * Created:	Nov 30, 2004
 * Copyright:	(C) 2004 MontaVista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <asm/arch/hw_cfg.h>
#include <asm/arch/hardware.h>                  // for IO_ADDRESS()
#include <asm/arch/bcm4760_reg.h>               // for I2S_REG_BASE_ADDR
#include "../codecs/bcm476x_codec.h"
#include "bcm476x_snd_pcm.h"

#ifndef MIN
#define MIN(a,b) (((a) > (b)) ? (b) : (a))
#endif

//
//  Module declarations.
//
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom sound interface");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA,Broadcom soundcard}}");

#if defined(CONFIG_PLAT_BCM476X)
#define MAX_PCM_DEVICES         3   // Map PCM device to HAL Audio Codec
#define PCM_MAX_BUFFER_SIZE     (64*1024)
#define MAX_PCM_SUBSTREAMS      1             
#define PCM_FORMATS             (SNDRV_PCM_FMTBIT_S16_LE)
#define PCM_PLAY_RATE           (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |  SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |SNDRV_PCM_RATE_48000)
#define PCM_PLAY_RATE_MIN       8000
#define PCM_PLAY_RATE_MAX       48000
#define PCM_CAPTURE_RATE        (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000)
#define PCM_CAPTURE_RATE_MIN    8000
#define PCM_CAPTURE_RATE_MAX    48000

#else 

#define MAX_PCM_DEVICES         3
#define PCM_MAX_BUFFER_SIZE     (64*1024)
#define MAX_PCM_SUBSTREAMS      1  
#define PCM_FORMATS             (SNDRV_PCM_FMTBIT_S16_LE)
#define PCM_PLAY_RATE           (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_22050)
#define PCM_PLAY_RATE_MIN       8000
#define PCM_PLAY_RATE_MAX       32000
#define PCM_CAPTURE_RATE        (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_22050)
#define PCM_CAPTURE_RATE_MIN    8000
#define PCM_CAPTURE_RATE_MAX    32000

#endif

#define MAX_PCM_STREAMS    (SNDRV_PCM_STREAM_LAST+1)
#define PCM_CHANNELS_MIN   1
#define PCM_CHANNELS_MAX   2
#define PCM_PERIODS_MIN    1
#define PCM_PERIODS_MAX    1024

typedef enum
{
   SND_CARD_IDLE,
   SND_CARD_OPENED,
   SND_CARD_ACTIVE,
   SND_CARD_CLOSING,
   SND_CARD_MAX_STATE
} SND_CARD_STATE;

typedef struct snd_soc_brcm_pcm
{
   spinlock_t   lock;
   int          bActive;
   unsigned int pcm_size;
   unsigned int pcm_count;
   unsigned int pcm_bps;      /* bytes per second */
   unsigned int pcm_jiffy;    /* bytes per one jiffy */
   unsigned int pcm_buf_pos;  /* position in buffer */
   struct snd_pcm_substream *substream;
} snd_soc_brcm_pcm_t;

static struct snd_pcm_hardware snd_soc_brcm_pcm_playback =
{
   .info =              (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_MMAP_VALID),
   .formats =           PCM_FORMATS,
   .rates =             PCM_PLAY_RATE,
   .rate_min =          PCM_PLAY_RATE_MIN,
   .rate_max =          PCM_PLAY_RATE_MAX,
   .channels_min =      PCM_CHANNELS_MIN,
   .channels_max =      PCM_CHANNELS_MAX,
   .buffer_bytes_max =  PCM_MAX_BUFFER_SIZE,
   .period_bytes_min =  64,
   .period_bytes_max =  PCM_MAX_BUFFER_SIZE,
   .periods_min =       PCM_PERIODS_MIN,
   .periods_max =       PCM_PERIODS_MAX,
   .fifo_size =         0,
};

static struct snd_pcm_hardware snd_soc_brcm_pcm_capture =
{
   .info =              (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_MMAP_VALID),
   .formats =           PCM_FORMATS,
   .rates =             PCM_CAPTURE_RATE,
   .rate_min =          PCM_CAPTURE_RATE_MIN,
   .rate_max =          PCM_CAPTURE_RATE_MAX,
   .channels_min =      PCM_CHANNELS_MIN,
   .channels_max =      PCM_CHANNELS_MAX,
   .buffer_bytes_max =  PCM_MAX_BUFFER_SIZE,
   .period_bytes_min =  64,
   .period_bytes_max =  PCM_MAX_BUFFER_SIZE,
   .periods_min =       PCM_PERIODS_MIN,
   .periods_max =       PCM_PERIODS_MAX,
   .fifo_size =         0,
};

/*************************
 * Function prototypes
 *************************/

static void bcm476x_pcm_enqueue(struct snd_pcm_substream *substream)
{
    snd_soc_brcm_pcm_t *dpcm  = substream->runtime->private_data;
    struct snd_soc_pcm_runtime *soc_rtd = substream->private_data;
    struct snd_soc_codec *codec = soc_rtd->socdev->codec;
    struct snd_pcm_runtime *runtime = substream->runtime;
    int len = dpcm->pcm_count; // default to send one period

    if ((dpcm->pcm_buf_pos + len) > dpcm->pcm_size) 
    	len  = dpcm->pcm_size - dpcm->pcm_buf_pos;

    if (SNDRV_PCM_STREAM_CAPTURE == substream->stream)
        codec->hw_read(substream, &(runtime->dma_area[dpcm->pcm_buf_pos]), len);
    else
        codec->hw_write(substream, &(runtime->dma_area[dpcm->pcm_buf_pos]), len);
}

static void pcm_xfer_done(void* params, int error, int xfer_len)
{
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *) params;
    struct snd_pcm_runtime *runtime = substream->runtime;
    snd_soc_brcm_pcm_t *dpcm  = runtime->private_data;
    //printk(KERN_INFO "pcm_xfer_done: stream=%p, error=%d,xfer_len=%d\n", substream, error, xfer_len);

	if (error || NULL == substream)
		return;

    spin_lock_irq(&dpcm->lock);
    if (dpcm->bActive) // still running?
    {
        /* Clear old samples if PCM buffer is more than 2 periods. */
        if (SNDRV_PCM_STREAM_PLAYBACK == substream->stream)
            memset(&(runtime->dma_area[dpcm->pcm_buf_pos]), 0, xfer_len);
        //  Update the position pointers.
        //
        dpcm->pcm_buf_pos += xfer_len;

       /* wrap buffer position if next position period exceeds buffer size */
        if (dpcm->pcm_buf_pos >= dpcm->pcm_size) 
            dpcm->pcm_buf_pos = 0;

        spin_unlock_irq(&dpcm->lock);

        snd_pcm_period_elapsed(substream); // A period is completed, let the middleware know.

        spin_lock_irq(&dpcm->lock);

        /* Need to check for active again here, because the snd_pcm_period_elapsed notification may trigger
         * a stop (SNDRV_PCM_TRIGGER_STOP) befor a return from this function.
         */
        if (dpcm->bActive) bcm476x_pcm_enqueue(substream);
    }
    spin_unlock_irq(&dpcm->lock);
}

/****************************************************************************
*
*  snd_soc_brcm_pcm_trigger
*
*  Sound trigger routine called when PCM is started, stopped, and paused.
*
***************************************************************************/
static int snd_soc_brcm_pcm_trigger(
   struct snd_pcm_substream * substream,
   int cmd
)
{
   struct snd_pcm_runtime *runtime = substream->runtime;
   snd_soc_brcm_pcm_t *dpcm = runtime->private_data;


   /*printk("snd_soc_brcm_pcm_trigger: device %d, cmd= %d\n", substream->pcm->device, cmd);*/


   switch ( cmd )
   {
      case SNDRV_PCM_TRIGGER_START:
      {
         spin_lock_irq(&dpcm->lock);
         dpcm->bActive = 1;
         bcm476x_pcm_enqueue(substream);
         spin_unlock_irq(&dpcm->lock);
      }
      break;
      case SNDRV_PCM_TRIGGER_STOP:
      {
         spin_lock_irq(&dpcm->lock);
         dpcm->bActive = 0;
         spin_unlock_irq(&dpcm->lock);
      }
      break;
      default:
      {
         return -EINVAL;
      }
   }
   return 0;
}


/****************************************************************************
*
*  snd_soc_brcm_pcm_runtime_free
*
*  Sound callback for freeing runtime resources.
*
***************************************************************************/
static void snd_soc_brcm_pcm_runtime_free(struct snd_pcm_runtime *runtime)
{
    snd_soc_brcm_pcm_t *dpcm = runtime->private_data;
    if (dpcm) kfree(dpcm);
}

static int snd_soc_brcm_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
    return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params)); 
}

static int snd_soc_brcm_pcm_hw_free(struct snd_pcm_substream *substream)
{
    return snd_pcm_lib_free_pages(substream);
}

static int snd_soc_brcm_pcm_prepare(struct snd_pcm_substream * substream)
{
    unsigned int bps;
    struct snd_pcm_runtime *runtime = substream->runtime;
    snd_soc_brcm_pcm_t *dpcm = runtime->private_data;

    bps = runtime->rate * runtime->channels;
    bps *= snd_pcm_format_width(runtime->format);
    bps /= 8;
    if (bps <= 0) {
      printk( KERN_ERR "snd_card_brcm_playback_prepare: Failed invalid pcm bps %d\n", bps );
      return -EINVAL;
    }
    dpcm->pcm_bps = bps;
    dpcm->pcm_jiffy = bps / HZ;
    dpcm->pcm_size = snd_pcm_lib_buffer_bytes(substream);
    dpcm->pcm_count = snd_pcm_lib_period_bytes(substream);
    dpcm->pcm_buf_pos = 0;
//    printk( KERN_INFO "snd_soc_brcm_pcm_prepare: device %d, pcm_size=%i pcm_count=%i\n",substream->pcm->device, dpcm->pcm_size, dpcm->pcm_count);
    return 0;
}
 
static snd_pcm_uframes_t snd_soc_brcm_pcm_pointer(struct snd_pcm_substream * substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    snd_soc_brcm_pcm_t *dpcm = runtime->private_data;

    return bytes_to_frames(runtime, dpcm->pcm_buf_pos);
}

static int snd_soc_brcm_pcm_open(struct snd_pcm_substream *substream)
{
   struct snd_pcm_runtime *runtime = substream->runtime;
   snd_soc_brcm_pcm_t *dpcm;

   dpcm = kcalloc(1, sizeof(*dpcm), GFP_KERNEL);
   if (dpcm == NULL)
      return -ENOMEM;

   spin_lock_init(&dpcm->lock);
   dpcm->substream = substream;
   runtime->private_data = dpcm;
   runtime->private_free = snd_soc_brcm_pcm_runtime_free;
   snd_soc_set_runtime_hwparams(substream, 
       (SNDRV_PCM_STREAM_CAPTURE == substream->stream) ? &snd_soc_brcm_pcm_capture : &snd_soc_brcm_pcm_playback);
   return 0;
}

static int snd_soc_brcm_pcm_close(struct snd_pcm_substream *substream)
{
    return 0;
}

static struct snd_pcm_ops snd_soc_brcm_ops = {
   .open =        snd_soc_brcm_pcm_open,
   .close =       snd_soc_brcm_pcm_close,
   .ioctl =       snd_pcm_lib_ioctl,
   .hw_params =   snd_soc_brcm_pcm_hw_params,
   .hw_free =     snd_soc_brcm_pcm_hw_free,
   .prepare =     snd_soc_brcm_pcm_prepare,
   .trigger =     snd_soc_brcm_pcm_trigger,
   .pointer =     snd_soc_brcm_pcm_pointer,
};

#if 0 // removed, not used
static int pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = substream->runtime->hw.buffer_bytes_max;
	buf->dev.type = SNDRV_DMA_TYPE_CONTINUOUS;
	//buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
					   &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	return 0;
}

static void snd_soc_brcm_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);
		buf->area = NULL;
	}
}
#endif

static int snd_soc_brcm_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,	struct snd_pcm *pcm)
{
    bcm476x_codec_register_stream_xfer_done_nofify(pcm_xfer_done);
    return snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
                                         snd_dma_continuous_data(GFP_KERNEL),
                                         0, 64*1024);

}

static int snd_soc_brcm_pcm_probe(struct platform_device *pdev)
{
	return 0;
}

static int snd_soc_brcm_pcm_remove(struct platform_device *pdev)
{
	return 0;
}

struct snd_soc_platform brcm_soc_platform = {
	.name		= "BRCM-SND",
	.probe 		= snd_soc_brcm_pcm_probe,
	.remove		= snd_soc_brcm_pcm_remove,
	//.suspend 	= snd_soc_brcm_pcm_suspend,
	//.resume 	= snd_soc_brcm_pcm_resume,

	// pcm creation & destruction
	.pcm_new 	= snd_soc_brcm_pcm_new,
	//.pcm_free	= snd_soc_brcm_pcm_free_dma_buffers,
	// platform stream ops
	.pcm_ops 	= &snd_soc_brcm_ops,
};
EXPORT_SYMBOL_GPL(brcm_soc_platform);

MODULE_AUTHOR("Dzanh Nguyen");
MODULE_DESCRIPTION("Broacom BCM476X PCM DMA module");
MODULE_LICENSE("GPL");
