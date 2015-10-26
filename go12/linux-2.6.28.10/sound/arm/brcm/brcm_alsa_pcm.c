/*
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/platform_device.h>
#include <sound/driver.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>

#include <asm/arch/hw_cfg.h>

#include "brcm_alsa.h"

/* hardware definition */
static struct snd_pcm_hardware brcm_playback_hw = 
{
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_BLOCK_TRANSFER |	SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_8000_48000,
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 32768,
	.period_bytes_min = 4096,
	.period_bytes_max = 32768,
	.periods_min = 1,
	.periods_max = 1024,
};

static struct snd_pcm_hardware brcm_capture_hw = 
{
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |	SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_8000_48000,
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 32768,
	.period_bytes_min = 4096,
	.period_bytes_max = 32768,
	.periods_min = 1,
	.periods_max = 1024,
};


//common

static int brcm_alsa_omx_pcm_hw_params(struct snd_pcm_substream * substream,
                                   struct snd_pcm_hw_params * hw_params)
{
	int index;
	
	DEBUG("\n %lx:hw_params %d\n",jiffies,(int)substream->stream);
	
	
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		index = 0;		
	}
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
	{
		index = 1;		
	}
	else
		return -EINVAL;
	
	DEBUG("\t params_access=%d\n",params_access(hw_params));
	DEBUG("\t params_format=%d\n",params_format(hw_params));
	DEBUG("\t params_subformat=%d\n",params_subformat(hw_params));
	DEBUG("\t params_channels=%d\n",params_channels(hw_params));
	DEBUG("\t params_rate=%d\n",params_rate(hw_params));
	DEBUG("\t params_period_size=%d\n",params_period_size(hw_params));
	DEBUG("\t params_period_bytes=%d\n",params_period_bytes(hw_params));
	DEBUG("\t params_periods=%d\n",params_periods(hw_params));
	DEBUG("\t params_buffer_size=%d\n",params_buffer_size(hw_params));
	DEBUG("\t params_buffer_bytes=%d\n",params_buffer_bytes(hw_params));
			
	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
}

static int brcm_alsa_omx_pcm_hw_free(struct snd_pcm_substream * substream)
{
	brcm_alsa_omx_vc03_close();
	
	DEBUG("\n %lx:hw_free \n",jiffies);
	
	return snd_pcm_lib_free_pages(substream);
}

static unsigned int buffer_size[]={8192};  //frames
static struct snd_pcm_hw_constraint_list constraints_buffer_size = 
{
	.count = ARRAY_SIZE(buffer_size),
	.list  = buffer_size,
	.mask  = 0,
};

static unsigned int period_size[]={1024};
static struct snd_pcm_hw_constraint_list constraints_period_size = 
{
	.count = ARRAY_SIZE(period_size),
	.list  = period_size,
	.mask  = 0,
};


//playback
static int brcm_alsa_omx_pcm_playback_open(struct snd_pcm_substream * substream)
{
	brcm_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err=0;
		
	runtime->hw = brcm_playback_hw;	
	chip->substream[0] = substream;

	//make sure buffer-size is multiple of period-size
	err = snd_pcm_hw_constraint_list(runtime,0,SNDRV_PCM_HW_PARAM_BUFFER_SIZE,
										&constraints_buffer_size);
	
	if (err<0)
		return err;
											
	err = snd_pcm_hw_constraint_list(runtime,0,SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
										&constraints_period_size);
										
										
	DEBUG("\n %lx:playback_open \n",jiffies);
		
	return err;
}

static int brcm_alsa_omx_pcm_playback_close(struct snd_pcm_substream * substream)
{
	brcm_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	
	DEBUG("\n %lx:playback_close \n",jiffies);
	
	chip->substream[0] = NULL;
	
	return 0;
}

static int brcm_alsa_omx_pcm_playback_prepare(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	DEBUG("\n %lx:playback_prepare \n",jiffies);
	
	
	DEBUG("\t rate=%d\n",runtime->rate);
	DEBUG("\t format=%d\n",runtime->format);
	DEBUG("\t channels=%d\n",runtime->channels);
	DEBUG("\t dma_area=%x\n",(unsigned int)runtime->dma_area);
	DEBUG("\t dma_bytes=%d\n",runtime->dma_bytes);
	DEBUG("\t period_bytes=%d\n",frames_to_bytes(runtime, runtime->period_size));
	DEBUG("\t avail_min=%d\n",frames_to_bytes(runtime, runtime->control->avail_min));

	g_brcm_alsa_chip->rate[0]         = runtime->rate;
	g_brcm_alsa_chip->buffer_bytes[0] = runtime->dma_bytes;
	g_brcm_alsa_chip->period_bytes[0] = frames_to_bytes(runtime, runtime->period_size);
	
	return 0;
}

static int brcm_alsa_omx_pcm_playback_trigger(struct snd_pcm_substream * substream, int cmd)
{
	DEBUG("\n %lx:playback_trigger cmd=%d\n",jiffies,cmd);
	
	switch (cmd) 
	{
		case SNDRV_PCM_TRIGGER_START:
			/* do something to start the PCM engine */
			startPlay();
			break;
		
		case SNDRV_PCM_TRIGGER_STOP:
			/* do something to stop the PCM engine */
			stopPlay();
			break;
		
		default:
		return -EINVAL;
	}	
	
	return 0;
}

static snd_pcm_uframes_t brcm_alsa_omx_pcm_playback_pointer(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t ret;

	ret = bytes_to_frames(runtime, g_brcm_alsa_chip->pcm_ptr[0]);
	DEBUG("\n %lx:playback_pointer %d\n",jiffies,(int)ret);
    
    return ret;
}

//need to check every VC OMX release
static unsigned int rate[]={16000};
static struct snd_pcm_hw_constraint_list constraints_rate = 
{
	.count = ARRAY_SIZE(rate),
	.list  = rate,
	.mask  = 0,
};


//capture
static int brcm_alsa_omx_pcm_capture_open(struct snd_pcm_substream * substream)
{
	brcm_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err=0;
	
	runtime->hw = brcm_capture_hw;	
	chip->substream[1] = substream;
	
	//make sure buffer-size is multiple of period-size
	err = snd_pcm_hw_constraint_list(runtime,0,SNDRV_PCM_HW_PARAM_BUFFER_SIZE,
										&constraints_buffer_size);
	
	if (err<0)
		return err;
											
	err = snd_pcm_hw_constraint_list(runtime,0,SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
										&constraints_period_size);
										
	if (err<0)
		return err;
											
	err = snd_pcm_hw_constraint_list(runtime,0,SNDRV_PCM_HW_PARAM_RATE,
										&constraints_rate);
										
	DEBUG("\n %lx:capture_open \n",jiffies);
	return 0;
}

static int brcm_alsa_omx_pcm_capture_close(struct snd_pcm_substream * substream)
{
	brcm_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	
	chip->substream[1] = NULL;
	
	DEBUG("\n %lx:capture_close \n",jiffies);
	return 0;
}

static int brcm_alsa_omx_pcm_capture_prepare(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	DEBUG("\n %lx:capture_prepare \n",jiffies);
	
	DEBUG("\t rate=%d\n",runtime->rate);
	DEBUG("\t format=%d\n",runtime->format);
	DEBUG("\t channels=%d\n",runtime->channels);
	DEBUG("\t dma_area=%x\n",(unsigned int)runtime->dma_area);
	DEBUG("\t dma_bytes=%d\n",runtime->dma_bytes);
	DEBUG("\t period_bytes=%d\n",frames_to_bytes(runtime, runtime->period_size));
	DEBUG("\t avail_min=%d\n",frames_to_bytes(runtime, runtime->control->avail_min));

	g_brcm_alsa_chip->rate[1]         = runtime->rate;
	g_brcm_alsa_chip->buffer_bytes[1] = runtime->dma_bytes;
	g_brcm_alsa_chip->period_bytes[1] = frames_to_bytes(runtime, runtime->period_size);
	
	return 0;
}

static int brcm_alsa_omx_pcm_capture_trigger(struct snd_pcm_substream * substream, int cmd)
{
	DEBUG("\n %lx:capture_trigger cmd=%d\n",jiffies,cmd);
	
	switch (cmd) 
	{
		case SNDRV_PCM_TRIGGER_START:
			/* do something to start the PCM engine */
			startRecd();
		break;
		
		case SNDRV_PCM_TRIGGER_STOP:
			/* do something to stop the PCM engine */
			stopRecd();
		break;
		
		default:
		return -EINVAL;
	}	
	return 0;
}

static snd_pcm_uframes_t brcm_alsa_omx_pcm_capture_pointer(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t ret;

	ret = bytes_to_frames(runtime, g_brcm_alsa_chip->pcm_ptr[1]);
	DEBUG("\n %lx:capture_pointer %d\n",jiffies,(int)ret);

	return ret;
}



//

static struct snd_pcm_ops brcm_alsa_omx_pcm_playback_ops = {
   .open =        brcm_alsa_omx_pcm_playback_open,
   .close =       brcm_alsa_omx_pcm_playback_close,
   .ioctl =       snd_pcm_lib_ioctl,
   .hw_params =   brcm_alsa_omx_pcm_hw_params,
   .hw_free =     brcm_alsa_omx_pcm_hw_free,
   .prepare =     brcm_alsa_omx_pcm_playback_prepare,
   .trigger =     brcm_alsa_omx_pcm_playback_trigger,
   .pointer =     brcm_alsa_omx_pcm_playback_pointer,
};

static struct snd_pcm_ops brcm_alsa_omx_pcm_capture_ops = {
   .open =        brcm_alsa_omx_pcm_capture_open,
   .close =       brcm_alsa_omx_pcm_capture_close,
   .ioctl =       snd_pcm_lib_ioctl,
   .hw_params =   brcm_alsa_omx_pcm_hw_params,
   .hw_free =     brcm_alsa_omx_pcm_hw_free,
   .prepare =     brcm_alsa_omx_pcm_capture_prepare,
   .trigger =     brcm_alsa_omx_pcm_capture_trigger,
   .pointer =     brcm_alsa_omx_pcm_capture_pointer,
};


int __devinit brcm_alsa_omx_pcm_new(struct snd_card *card)
{
	struct snd_pcm *pcm;
	int err;

	err = snd_pcm_new(card, "Broadcom ALSA PCM", 0, 1, 1, &pcm);
	if (err<0)
		return err;
    
    pcm->private_data = card->private_data;
	strcpy(pcm->name, "Broadcom ALSA PCM");		
    
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &brcm_alsa_omx_pcm_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &brcm_alsa_omx_pcm_capture_ops);


	pcm->info_flags = 0;
	err=snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
                                     snd_dma_continuous_data(GFP_KERNEL),
                                     0, 32*1024);                                     
	return err;
	
}

//0: playback
//1: record
void snd_brcm_int_handler(int index)
{
	struct snd_pcm_substream *substream = g_brcm_alsa_chip->substream[index];
	struct snd_pcm_runtime *runtime = substream->runtime;
	
	g_brcm_alsa_chip->pcm_ptr[index] += g_brcm_alsa_chip->period_bytes[index];
	g_brcm_alsa_chip->pcm_ptr[index] %= g_brcm_alsa_chip->buffer_bytes[index];		
	
	snd_pcm_period_elapsed(substream);
		
	DEBUG("\n %lx:elapsed  pcm_ptr=%d hw_ptr=%d\n",jiffies,
		g_brcm_alsa_chip->pcm_ptr[index],(int)runtime->status->hw_ptr);	
}
