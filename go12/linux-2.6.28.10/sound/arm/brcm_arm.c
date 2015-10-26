/*
 *  Broadcom sound API to the Endpoint driver.
 *    Copyright (c) 2005 Broadcom Corporation
 *
 *  Based on the template from the dummy soundcard driver:
 *
 *  Dummy soundcard
 *  Copyright (c) by Jaroslav Kysela <perex@suse.cz>
 *
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
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <linux/broadcom/halaudio.h>
#include <asm/arch/hw_cfg.h>
#include <linux/broadcom/halaudio_settings.h>
#if defined( CONFIG_BCM_SLEEP_MODE )
#include <linux/broadcom/cpu_sleep.h>
#endif

#include <plat/scenarii.h>
extern int tomtom_add_nashville_controls(struct snd_card *, void*);

#define SND_LOG_ENABLED       0        /* debugging */
#if SND_LOG_ENABLED
#include <linux/broadcom/knllog.h>
#define SND_LOG               KNLLOG
#else
#define SND_LOG(fmt,arg...)
#endif

#ifndef MIN
#define MIN(a,b) (((a) > (b)) ? (b) : (a))
#endif

//  Module declarations.
//
//  Module declarations.
//
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom sound interface");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA,Broadcom soundcard}}");
// For two or more concurrent PCM streams, we have to be in halduio superuser mode otherwise only one stream
// can be active at a time.
#define HALAUDIO_SUPER_USER_MODE

#if defined(CONFIG_PLAT_BCM476X)
#define AUDIO_MUTE_GAIN         HAL_AUDIO_GAIN_MUTE
#define MAX_PCM_DEVICES         3   // Map PCM device to HAL Audio Codec
#define MAX_MIDI_DEVICES        0
#define MAX_BUFFER_SIZE         (64*1024)
#define MAX_PCM_SUBSTREAMS      1             
#define USE_FORMATS             (SNDRV_PCM_FMTBIT_S16_LE)
#define USE_PLAY_RATE           (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |  SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |SNDRV_PCM_RATE_48000)
#define USE_PLAY_RATE_MIN       8000
#define USE_PLAY_RATE_MAX       48000
#define USE_CAPTURE_RATE        (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000)
#define USE_CAPTURE_RATE_MIN    8000
#define USE_CAPTURE_RATE_MAX    48000

static const char *pcm_dev_name[MAX_PCM_DEVICES] = {
    "Audio",
    "Voice",
    "PCM",
};

#else 

#define AUDIO_MUTE_GAIN         HAL_AUDIO_GAIN_SLEEP
#define MAX_PCM_DEVICES         3
#define MAX_MIDI_DEVICES        0
#define MAX_BUFFER_SIZE         (64*1024)
#define MAX_PCM_SUBSTREAMS      1  
#define USE_FORMATS             (SNDRV_PCM_FMTBIT_S16_BE)
#define USE_PLAY_RATE           (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_22050)
#define USE_PLAY_RATE_MIN       8000
#define USE_PLAY_RATE_MAX       32000
#define USE_CAPTURE_RATE        (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_22050)
#define USE_CAPTURE_RATE_MIN    8000
#define USE_CAPTURE_RATE_MAX    32000
static const char *pcm_dev_name[MAX_PCM_DEVICES] = {
    "Broadcom PCM"
};

#endif

#define MAX_PCM_STREAMS    (SNDRV_PCM_STREAM_LAST+1)
#define USE_CHANNELS_MIN   1
#define USE_CHANNELS_MAX   2
#define USE_PERIODS_MIN    1
#define USE_PERIODS_MAX    1024


#define ALSA_CARD_BRCM_NAME "alsa_card_brcm"


static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX; /* Index 0-MAX */
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;  /* ID for this card */
static int enable[SNDRV_CARDS] = {1, [1 ... (SNDRV_CARDS - 1)] = 0};
static int pcm_devs[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS - 1)] = MAX_PCM_DEVICES};
static int pcm_substreams[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS - 1)] = MAX_PCM_SUBSTREAMS};
static int gMyAlsaHookId = -1;

typedef enum
{
   SND_CARD_IDLE,
   SND_CARD_OPENED,
   SND_CARD_ACTIVE,
   SND_CARD_CLOSING,
   SND_CARD_MAX_STATE
} SND_CARD_STATE;

module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for Broadcom soundcard.");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for Broadcom soundcard.");
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable the Broadcom soundcard.");
module_param_array(pcm_devs, int, NULL, 0444);
MODULE_PARM_DESC(pcm_devs, "PCM devices # (0-1) for Broadcom driver.");
module_param_array(pcm_substreams, int, NULL, 0444);
MODULE_PARM_DESC(pcm_substreams, "PCM substreams # (1-16) for Broadcom driver.");

typedef enum 
{
    MIXER_ADDR_SPEAKER,
    MIXER_ADDR_AUX,
    MIXER_ADDR_SPEAKER_DIG,
    MIXER_ADDR_AUX_DIG,
#if defined(CONFIG_PLAT_BCM476X)
    MIXER_ADDR_MIC,
    MIXER_ADDR_AUX_MIC,
    MIXER_ADDR_HANDFREE,
#endif
    MIXER_ADDR_LAST,
} MIXER_ADDR_T;

#define MIXER_SOURCE_LAST  1

typedef struct snd_card_brcm
{
   struct snd_card *card;
   spinlock_t mixer_lock;
   int mixer_volume[MIXER_ADDR_LAST+1][2];
   int play_dst[MIXER_SOURCE_LAST+1][2];
   int hand_free;
}
snd_card_brcm_t;

typedef struct snd_card_brcm_pcm
{
   snd_card_brcm_t *bcmCard;
   spinlock_t lock;
   int bActive;
   int alsaHookId;
   int codec;
   unsigned int pcm_size;
   unsigned int pcm_count;
   unsigned int pcm_bps;      /* bytes per second */
   unsigned int pcm_jiffy;    /* bytes per one jiffy */
   unsigned int pcm_irq_pos;  /* IRQ position */
   unsigned int pcm_buf_pos;  /* position in buffer */
   struct snd_pcm_substream *substream;
}
snd_card_brcm_pcm_t;

typedef struct  {
    SND_CARD_STATE state;
    long    threadId;
    int     alsaHookId;
    int     keepRunning;
    struct completion threadExited;
    struct semaphore sem; 
    struct snd_card_brcm_pcm *dpcm;
} alsaThread_t;

static alsaThread_t gWorkerThread[MAX_PCM_STREAMS][MAX_PCM_DEVICES][MAX_PCM_SUBSTREAMS]; // indexed by pcm stream type
static unsigned char gStreamGainInit[MAX_PCM_STREAMS][MAX_PCM_DEVICES][MAX_PCM_SUBSTREAMS]={{{0}}};

static struct snd_card *snd_brcm_cards[SNDRV_CARDS] = SNDRV_DEFAULT_PTR;

#define BRCM_SND_MAX_RATES_CONSTRAINT     HAL_AUDIO_MAX_NUM_FREQ_SETTINGS
static unsigned int rates[BRCM_SND_MAX_RATES_CONSTRAINT] = {
   8000,  22050, 0, 0, 0, 0, 0, 0, 0, 0
};

static struct snd_pcm_hw_constraint_list hw_constraints_rates = {
   .count   = ARRAY_SIZE(rates),
   .list    = rates,
   .mask    = 0,
};

/*************************
 * Function prototypes
 *************************/
static int configure_notches(void);
#ifdef CONFIG_TOMTOM_NASHVILLE_SCENARI_BCM4760
extern int alc5627_i2c_init(void);
#endif

static alsaThread_t *getThread(int stream_type, int device, int stream_num)
{
    if (stream_type >= MAX_PCM_STREAMS || device >= MAX_PCM_DEVICES || stream_num >= MAX_PCM_SUBSTREAMS || stream_num < 0)
        return NULL;
    return &gWorkerThread[stream_type][device][stream_num];
}

static alsaThread_t *getSubstreamThread(struct snd_pcm_substream * substream)
{
    return getThread(substream->stream,substream->pcm->device,substream->number);
}

/****************************************************************************
*
*  Start playing the audio.
*
***************************************************************************/
static void startPlay( alsaThread_t *thread )
{
    thread->keepRunning = 1;
    thread->state = SND_CARD_ACTIVE;
    up( &thread->sem );
}

/****************************************************************************
*
*  Stop playing the audio.
*
***************************************************************************/
static void cancelPlay( alsaThread_t *thread )
{
    thread->keepRunning = 0;
}

/****************************************************************************
*
*  Start capturing the audio.
*
***************************************************************************/
static void startCapture( alsaThread_t *thread  )
{
    thread->keepRunning = 1;
    thread->state = SND_CARD_ACTIVE;
    up( &thread->sem );
}

/****************************************************************************
*
*  Stop capturing the audio.
*
***************************************************************************/
static void cancelCapture( alsaThread_t *thread  )
{
    thread->keepRunning = 0;
}

/****************************************************************************
*
*  playContinue
*
*  Send more data to the HW audio buffer.
*
***************************************************************************/
//static 
void brcm_alsa_playContinue( alsaThread_t *thread )
{
   int bytesXfer = 0;
   snd_card_brcm_pcm_t *dpcm = thread->dpcm;
   struct snd_pcm_runtime *runtime = dpcm->substream->runtime;

   //  Check if the user has aborted.
   //
   if (!dpcm->bActive)
   {
      thread->keepRunning = 0;
      return;
   }

   //printk(KERN_INFO "playContinue: started, pcm_count=%d, bufpos =%d\n",  (int) dpcm->pcm_count, dpcm->pcm_buf_pos );
   //  We will transfer exactly one period to the endpoint.
#if defined(CONFIG_PLAT_BCM476X)
   bytesXfer = halAudio_writeDev( dpcm->alsaHookId, dpcm->codec, runtime->channels,
                                     &(runtime->dma_area[dpcm->pcm_buf_pos]), dpcm->pcm_count );
#else
   bytesXfer = halAudio_writeDev( dpcm->alsaHookId, runtime->channels,
                                     &(runtime->dma_area[dpcm->pcm_buf_pos]), dpcm->pcm_count );

#endif
   //printk(KERN_INFO "dataContinue: started, bytesXfer=%d\n",  (int) bytesXfer );

   if (bytesXfer <= 0)
   {
      //  Error in writing to the endpoint.  End the playback.
      //
      thread->keepRunning = 0;
   }
   if (!thread->keepRunning)
   {
      //  While we were blocked on the call to halAudio_writeDev, someone cancelled us.  Just return.
      //
      return;
   }

   spin_lock_irq(&dpcm->lock);

   //  Update the position pointers.
   //
   dpcm->pcm_irq_pos += bytesXfer;
   dpcm->pcm_buf_pos += bytesXfer;
   /* wrap buffer position */
   dpcm->pcm_buf_pos %= dpcm->pcm_size;

   SND_LOG( "irq_pos=%i buf_pos=%i bytesXfer=%i\n",
         dpcm->pcm_irq_pos, dpcm->pcm_buf_pos, bytesXfer );

   //  Check if we have completed a period.  If so, let middleware know.
   //
   if (dpcm->pcm_irq_pos >= dpcm->pcm_count)
   {
      /* wrap irq position */
      dpcm->pcm_irq_pos %= dpcm->pcm_count;
      spin_unlock_irq(&dpcm->lock);
      snd_pcm_period_elapsed(dpcm->substream);
   }
   else
   {
      spin_unlock_irq(&dpcm->lock);
   }
}

/****************************************************************************
*
*  captureContinue
*
*  Capture more data from the HW to the audio buffer.
*
***************************************************************************/
//static 
void brcm_alsa_captureContinue( alsaThread_t *thread )
{
   int bytesXfer = 0;
   snd_card_brcm_pcm_t *dpcm = thread->dpcm;
   struct snd_pcm_runtime *runtime = dpcm->substream->runtime;

   //  Check if the user has aborted.
   //
   if (!dpcm->bActive)
   {
      thread->keepRunning = 0;
      return;
   }

   //printk(KERN_INFO "captureContinue: started, pcm_count=%d, bufpos =%d\n",  (int) dpcm->pcm_count, dpcm->pcm_buf_pos );
   //  We will transfer exactly one period to the endpoint.
   //
#if defined(CONFIG_PLAT_BCM476X)
   bytesXfer = halAudio_readDev( dpcm->alsaHookId, dpcm->codec, runtime->channels,
                                     &(runtime->dma_area[dpcm->pcm_buf_pos]), dpcm->pcm_count );
#else
   bytesXfer = halAudio_readDev( dpcm->alsaHookId, runtime->channels,
                                     &(runtime->dma_area[dpcm->pcm_buf_pos]), dpcm->pcm_count );
#endif
   //printk(KERN_INFO "dataContinue: started, bytesWritten=%d\n",  (int) bytesWritten );

   if (bytesXfer <= 0)
   {
      //  Error in writing to the endpoint.  End the playback.
      //
      thread->keepRunning = 0;
   }
   if (!thread->keepRunning)
   {
      //  While we were blocked on the call to halAudio_writeDev, someone cancelled us.  Just return.
      //
      return;
   }

   spin_lock_irq(&dpcm->lock);

   //  Update the position pointers.
   //
   dpcm->pcm_irq_pos += bytesXfer;
   dpcm->pcm_buf_pos += bytesXfer;
   /* wrap buffer position */
   dpcm->pcm_buf_pos %= dpcm->pcm_size;

   SND_LOG( "irq_pos=%i buf_pos=%i bytesXfer=%i\n",
         dpcm->pcm_irq_pos, dpcm->pcm_buf_pos, bytesXfer );

   //  Check if we have completed a period.  If so, let middleware know.
   //
   if (dpcm->pcm_irq_pos >= dpcm->pcm_count)
   {
      /* wrap irq position */
      dpcm->pcm_irq_pos %= dpcm->pcm_count;
      spin_unlock_irq(&dpcm->lock);
      snd_pcm_period_elapsed(dpcm->substream);
   }
   else
   {
      spin_unlock_irq(&dpcm->lock);
   }
}

/****************************************************************************
*
*  alsaWorkerThread
*
*  This is the worker thread main routine for processing buffers.
*
***************************************************************************/
#define WORKERTHRD_RT_PRIORITY      62
static void alsaWorkerThread( alsaThread_t *thread )
{
   struct sched_param sparm;
   int rc;

   daemonize("alsaWorker");

   /* Set real-time priority */
   sparm.sched_priority = WORKERTHRD_RT_PRIORITY;
   if (( rc = sched_setscheduler( current, SCHED_FIFO, &sparm )) < 0 )
   {
      printk( KERN_ERR "failed to set RT priority %i for thread\n", sparm.sched_priority );
   }

   while ( thread->state != SND_CARD_CLOSING )
   {
      if ( down_interruptible (&thread->sem) == 0 ) /* Wait for trigger starts */
      {
         snd_card_brcm_pcm_t *thread_dpcm = thread->dpcm;
         BUG_ON(NULL == thread_dpcm);
         //  As long as the user keeps feeding us data, keep playing.
         //
         while (thread->keepRunning)
         {
            if (SNDRV_PCM_STREAM_CAPTURE == thread_dpcm->substream->stream)
                brcm_alsa_captureContinue( thread );
            else if (SNDRV_PCM_STREAM_PLAYBACK == thread_dpcm->substream->stream)
                brcm_alsa_playContinue( thread );
            else 
            {
                printk( KERN_ERR "no handler for alsaWorker stream %d\n", thread_dpcm->substream->stream);
                thread->keepRunning = 0;
            }
         }
      }
   }
   complete_and_exit( &thread->threadExited, 0 );
}

/****************************************************************************
*
*  add_playback_constraints
*
*  Check the constraints we have upon playback.
*
***************************************************************************/
static int add_playback_constraints(struct snd_pcm_runtime *runtime)
{
   int err;

   if ((err = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS)) < 0)
      return err;
   if ((err = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE, &hw_constraints_rates)) < 0)
      return err;
   return 0;
}

/****************************************************************************
*
*  add_capture_constraints
*
*  Check the constraints we have upon capture.
*
***************************************************************************/
static int add_capture_constraints(struct snd_pcm_runtime *runtime)
{
   int err;

   if ((err = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS)) < 0)
      return err;
   if ((err = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE, &hw_constraints_rates)) < 0)
      return err;
   return 0;
}

/****************************************************************************
*
*  snd_card_brcm_playback_trigger
*
*  Sound trigger routine called when PCM is started, stopped, and paused.
*
***************************************************************************/
static int snd_card_brcm_playback_trigger(
   struct snd_pcm_substream * substream,
   int cmd
)
{
   struct snd_pcm_runtime *runtime = substream->runtime;
   snd_card_brcm_pcm_t *dpcm = runtime->private_data;
   alsaThread_t *thread = getSubstreamThread(substream);

   if (thread == NULL)
        return EINVAL;

   switch ( cmd )
   {
      case SNDRV_PCM_TRIGGER_START:
      {
         if (thread->keepRunning)
         {
            //  Already streaming.  Fail the start call.
            //
            return -EINVAL;
         }
         dpcm->bActive = 1;
         startPlay(thread);
      }
      break;
      case SNDRV_PCM_TRIGGER_STOP:
      {
         dpcm->bActive = 0;
         cancelPlay(thread);
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
*  snd_card_brcm_capture_trigger
*
*  Sound trigger routine (for record and cancel).
*
***************************************************************************/
static int snd_card_brcm_capture_trigger(struct snd_pcm_substream * substream,
                                         int cmd)
{
   struct snd_pcm_runtime *runtime = substream->runtime;
   snd_card_brcm_pcm_t *dpcm = runtime->private_data;
   alsaThread_t *thread = getSubstreamThread(substream);

   if (thread == NULL)
        return EINVAL;

   switch ( cmd )
   {
      case SNDRV_PCM_TRIGGER_START:
      {
         if (thread->keepRunning)
         {
            //  Already streaming.  Fail the start call.
            //
            return -EINVAL;
         }
         dpcm->bActive = 1;
         startCapture(thread);
      }
      break;
      case SNDRV_PCM_TRIGGER_STOP:
      {
         dpcm->bActive = 0;
         cancelCapture(thread);
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
*  snd_card_brcm_playback_prepare
*
*  Sound callback for preparing the playback stream.
*
***************************************************************************/
static int snd_card_brcm_playback_prepare(struct snd_pcm_substream * substream)
{
    unsigned int bps, codec;
    struct snd_pcm_runtime *runtime = substream->runtime;
    snd_card_brcm_pcm_t *dpcm = runtime->private_data;

    printk(KERN_INFO "snd_card_brcm_pcm_prepare:\n");

   //  Enable Audio component
    //
#ifndef HALAUDIO_SUPER_USER_MODE
    /* halAudio_getControl only sets the rate if not in super user mode */
    if ( halAudio_getControl( dpcm->alsaHookId, HAL_AUDIO_MODE_ALSA, runtime->rate ))
    {
      printk( KERN_ERR "ALSA: Failed to get control rate=%i\n", runtime->rate );

      /* Failed to change rate */
      return -1;
    }
#else
    /* set mode and user rate for super user mode */
    halAudio_selectMode( dpcm->alsaHookId, HAL_AUDIO_MODE_ALSA, runtime->rate ); 
#endif
 
#if 1
    /* FIXED ME. After the last call to snd_card_brcm_close, the halAudio_internal will put the device into DEEP_SLEEP which may also
     * cause the device to go into MUTE mode. We need to do this to get out of MUTE mode. We suppose to put the gain back to the
     * last set value.
     */
    if (!gStreamGainInit[substream->stream][substream->pcm->device][substream->number]) // Initialize default gain once
    {
        int db;
        db = halAudio_getAnaGain(dpcm->alsaHookId, HAL_AUDIO_EAR_SPKR); // get from hw
        halAudio_setAnaGain (dpcm->alsaHookId, HAL_AUDIO_EAR_SPKR, db); // set hal-audio gain to current value
        db = halAudio_getDigGain(dpcm->alsaHookId, HAL_AUDIO_EAR_SPKR); // get from hw
        halAudio_setDigGain (dpcm->alsaHookId, HAL_AUDIO_EAR_SPKR, db);
        db = halAudio_getAnaGain(dpcm->alsaHookId, HAL_AUDIO_AUX_SPKR); // get from hw
        halAudio_setAnaGain (dpcm->alsaHookId, HAL_AUDIO_AUX_SPKR, db); // set hal-audio gain to current value
        db = halAudio_getDigGain(dpcm->alsaHookId, HAL_AUDIO_AUX_SPKR); // get from hw
        halAudio_setDigGain (dpcm->alsaHookId, HAL_AUDIO_AUX_SPKR, db);
    #if defined(CONFIG_PLAT_BCM476X)
        if (2 == substream->pcm->device)
        {
            db = halAudio_getAnaGain(dpcm->alsaHookId, HAL_AUDIO_HANDFREE_SPKR); // get from hw
            halAudio_setAnaGain (dpcm->alsaHookId, HAL_AUDIO_HANDFREE_SPKR, db);
        }
    #endif
        gStreamGainInit[substream->stream][substream->pcm->device][substream->number] = 1;
    }
#endif 

    halAudio_enableAudio( dpcm->alsaHookId, 1 ); // Must enable audio first to power audio block on

#if !defined(CONFIG_PLAT_BCM476X) 
    // set the default mic and speaker source to EAR spkr 
    halAudio_blockDisable( dpcm->alsaHookId, HAL_AUDIO_AUX_SPKR );
    halAudio_blockEnable( dpcm->alsaHookId, HAL_AUDIO_EAR_SPKR );
    // set the default playout speaker to EAR spkr (not necessary if the phone phone support 1 codec)
    halAudio_setActiveCodec( dpcm->alsaHookId, (codec = HW_HANDSET_CODEC) );
#else // BCM476X
    if (substream->pcm->device == 2) // Handfree subdevice
    {
       if (runtime->rate != 8000) {
           printk(KERN_CRIT "AAAA\n");
           return -EINVAL;
       }

       halAudio_blockEnable( dpcm->alsaHookId, HAL_AUDIO_HANDFREE_SPKR );
       halAudio_setActiveCodec( dpcm->alsaHookId, (codec = HW_HANDFREE_CODEC) );
    }
    else if (/* (substream->pcm->device == 1 && */ runtime->rate <= 16000) // Voice subdevice
    {
       halAudio_blockDisable( dpcm->alsaHookId, HAL_AUDIO_EAR_SPKR );
       halAudio_blockEnable( dpcm->alsaHookId, HAL_AUDIO_AUX_SPKR );
       // set the default playout speaker to EAR spkr (not necessary if the phone phone support 1 codec)
       halAudio_setActiveCodec( dpcm->alsaHookId, (codec = HW_SPEAKERPHONE_CODEC) );
    }
    else // default to music path
    {
       // set the default mic and speaker source to EAR spkr 
       halAudio_blockDisable( dpcm->alsaHookId, HAL_AUDIO_AUX_SPKR );
       halAudio_blockEnable( dpcm->alsaHookId, HAL_AUDIO_EAR_SPKR );
       // set the default playout speaker to EAR spkr (not necessary if the phone phone support 1 codec)
       halAudio_setActiveCodec( dpcm->alsaHookId, (codec = HW_HANDSET_CODEC) );
    }
#endif
    dpcm->codec = codec;
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
    dpcm->pcm_irq_pos = 0;
    dpcm->pcm_buf_pos = 0;
    SND_LOG( "pcm_size=%i pcm_count=%i rate=%i bps=%i pcm_jiffy=%i\n",
         dpcm->pcm_size, dpcm->pcm_count, runtime->rate, bps, dpcm->pcm_jiffy  );

    return 0;
}

/****************************************************************************
*
*  snd_card_brcm_capture_prepare
*
*  Sound callback for preparing the capture stream.
*
***************************************************************************/
static int snd_card_brcm_capture_prepare(struct snd_pcm_substream * substream)
{
    unsigned int bps, codec;
    struct snd_pcm_runtime *runtime = substream->runtime;
    snd_card_brcm_pcm_t *dpcm = runtime->private_data;

    //printk(KERN_INFO "snd_card_brcm_pcm_prepare:\n");

   //  Enable Audio component
    //
#ifndef HALAUDIO_SUPER_USER_MODE
    /* halAudio_getControl only sets the rate if not in super user mode */
    if ( halAudio_getControl( dpcm->alsaHookId, HAL_AUDIO_MODE_ALSA, runtime->rate ))
    {
      printk( KERN_ERR "ALSA: Failed to get control rate=%i\n", runtime->rate );

      /* Failed to change rate */
      return -1;
    }
#else
    /* set mode and user rate for super user mode */
    halAudio_selectMode( dpcm->alsaHookId, HAL_AUDIO_MODE_ALSA, runtime->rate ); 
#endif
 
#if 1
    /* FIXED ME. After the last call to snd_card_brcm_close, the halAudio_internal will put the device into DEEP_SLEEP which may also
     * cause the device to go into MUTE mode. We need to do this to get out of MUTE mode. We suppose to put the gain back to the
     * last set value.
     */
    if (!gStreamGainInit[substream->stream][substream->pcm->device][substream->number]) // Initialize default gain once
    {
        int db;
        db = halAudio_getAnaGain(dpcm->alsaHookId, HAL_AUDIO_EAR_MIC); // get from hw
        halAudio_setAnaGain (dpcm->alsaHookId, HAL_AUDIO_EAR_MIC, db);
        db = halAudio_getDigGain(dpcm->alsaHookId, HAL_AUDIO_EAR_MIC); // get from hw
        halAudio_setDigGain (dpcm->alsaHookId, HAL_AUDIO_EAR_MIC, db);
        db = halAudio_getAnaGain(dpcm->alsaHookId, HAL_AUDIO_AUX_MIC); // get from hw
        halAudio_setAnaGain (dpcm->alsaHookId, HAL_AUDIO_AUX_MIC, db);
        db = halAudio_getDigGain(dpcm->alsaHookId, HAL_AUDIO_AUX_MIC); // get from hw
        halAudio_setDigGain (dpcm->alsaHookId, HAL_AUDIO_AUX_MIC, db);
    #if defined(CONFIG_PLAT_BCM476X)
        if (2 == substream->pcm->device)
        {
            db = halAudio_getAnaGain(dpcm->alsaHookId, HAL_AUDIO_HANDFREE_MIC); // get from hw
            halAudio_setAnaGain (dpcm->alsaHookId, HAL_AUDIO_HANDFREE_MIC, db);
        }
    #endif
        gStreamGainInit[substream->stream][substream->pcm->device][substream->number] = 1;
    }
#endif 

    halAudio_enableAudio( dpcm->alsaHookId, 1 ); // Must enable audio first to power audio block on
#if !defined(CONFIG_PLAT_BCM476X) 
    // set the default mic and speaker source to EAR mic
    halAudio_blockDisable( dpcm->alsaHookId, HAL_AUDIO_AUX_MIC );
    halAudio_blockEnable( dpcm->alsaHookId, HAL_AUDIO_EAR_MIC );
    halAudio_setActiveCodec( dpcm->alsaHookId, (codec = HW_HANDSET_CODEC) );
#else // BCM476X
    if (substream->pcm->device == 2) // BT subdevice
    {
       if (runtime->rate != 8000)
           return -EINVAL;

       halAudio_blockEnable( dpcm->alsaHookId, HAL_AUDIO_HANDFREE_MIC ); 
       halAudio_setActiveCodec( dpcm->alsaHookId, (codec = HW_HANDFREE_CODEC));
    }
    else if (/* substream->pcm->device == 1  && */ runtime->rate <= 16000) // Voice subdevice
    {
       halAudio_blockDisable( dpcm->alsaHookId, HAL_AUDIO_EAR_MIC );   
       halAudio_blockEnable( dpcm->alsaHookId, HAL_AUDIO_AUX_MIC ); // Use voice path for recording 
       halAudio_setActiveCodec( dpcm->alsaHookId, (codec = HW_SPEAKERPHONE_CODEC));
    }
    else // default to Music subdevice
    { 
       halAudio_blockDisable( dpcm->alsaHookId, HAL_AUDIO_AUX_MIC );
       halAudio_blockEnable( dpcm->alsaHookId, HAL_AUDIO_EAR_MIC );    // Use audio path for recording
       halAudio_setActiveCodec( dpcm->alsaHookId, (codec = HW_HANDSET_CODEC));
    }
#endif

    dpcm->codec = codec;

    bps = runtime->rate * runtime->channels;
    bps *= snd_pcm_format_width(runtime->format);
    bps /= 8;
    if (bps <= 0) {
      printk( KERN_ERR "snd_card_brcm_capture_prepare: Failed invalid pcm bps %d\n", bps );
      return -EINVAL;
    }

    dpcm->pcm_bps = bps;
    dpcm->pcm_jiffy = bps / HZ;
    dpcm->pcm_size = snd_pcm_lib_buffer_bytes(substream);
    dpcm->pcm_count = snd_pcm_lib_period_bytes(substream);
    dpcm->pcm_irq_pos = 0;
    dpcm->pcm_buf_pos = 0;
    SND_LOG( "pcm_size=%i pcm_count=%i rate=%i bps=%i pcm_jiffy=%i\n",
         dpcm->pcm_size, dpcm->pcm_count, runtime->rate, bps, dpcm->pcm_jiffy  );

    return 0;
}

/****************************************************************************
*
*  snd_card_brcm_playback_pointer
*
*  Sound callback for getting the current playback frame. This function is
*  called when PCM middle layer inquires the current hardware position on
*  the buffer.
*
***************************************************************************/
static snd_pcm_uframes_t snd_card_brcm_playback_pointer(struct snd_pcm_substream * substream)
{
   struct snd_pcm_runtime *runtime = substream->runtime;
   snd_card_brcm_pcm_t *dpcm = runtime->private_data;

   return bytes_to_frames(runtime, dpcm->pcm_buf_pos);
}

/****************************************************************************
*
*  snd_card_brcm_capture_pointer
*
*  Sound callback for getting the current capture frame.
*
***************************************************************************/
static snd_pcm_uframes_t snd_card_brcm_capture_pointer(struct snd_pcm_substream * substream)
{
   struct snd_pcm_runtime *runtime = substream->runtime;
   snd_card_brcm_pcm_t *dpcm = runtime->private_data;

   return bytes_to_frames(runtime, dpcm->pcm_buf_pos);
}

static struct snd_pcm_hardware snd_card_brcm_playback =
{
   .info =              (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
                         SNDRV_PCM_INFO_MMAP_VALID),
   .formats =           USE_FORMATS,
   .rates =             USE_PLAY_RATE,
   .rate_min =          USE_PLAY_RATE_MIN,
   .rate_max =          USE_PLAY_RATE_MAX,
   .channels_min =      USE_CHANNELS_MIN,
   .channels_max =      USE_CHANNELS_MAX,
   .buffer_bytes_max =  MAX_BUFFER_SIZE,
   .period_bytes_min =  64,
   .period_bytes_max =  MAX_BUFFER_SIZE,
   .periods_min =       USE_PERIODS_MIN,
   .periods_max =       USE_PERIODS_MAX,
   .fifo_size =         0,
};

static struct snd_pcm_hardware snd_card_brcm_capture =
{
   .info =              (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
                         SNDRV_PCM_INFO_MMAP_VALID),
   .formats =           USE_FORMATS,
   .rates =             USE_CAPTURE_RATE,
   .rate_min =          USE_CAPTURE_RATE_MIN,
   .rate_max =          USE_CAPTURE_RATE_MAX,
   .channels_min =      USE_CHANNELS_MIN,
   .channels_max =      USE_CHANNELS_MAX,
   .buffer_bytes_max =  MAX_BUFFER_SIZE,
   .period_bytes_min =  64,
   .period_bytes_max =  MAX_BUFFER_SIZE,
   .periods_min =       USE_PERIODS_MIN,
   .periods_max =       USE_PERIODS_MAX,
   .fifo_size =         0,
};

/****************************************************************************
*
*  snd_card_brcm_runtime_free
*
*  Sound callback for freeing runtime resources.
*
***************************************************************************/
static void snd_card_brcm_runtime_free(struct snd_pcm_runtime *runtime)
{
   snd_card_brcm_pcm_t *dpcm = runtime->private_data;
   if (dpcm) kfree(dpcm);
}

/****************************************************************************
*
*  snd_card_brcm_hw_params
*
*  Sound callback for allocating HW parameters.
*
***************************************************************************/
static int snd_card_brcm_hw_params(struct snd_pcm_substream * substream,
                                   struct snd_pcm_hw_params * hw_params)
{
   return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
}

/****************************************************************************
*
*  snd_card_brcm_hw_free
*
*  Sound callback for freeing HW parameters.
*
***************************************************************************/
static int snd_card_brcm_hw_free(struct snd_pcm_substream * substream)
{
   return snd_pcm_lib_free_pages(substream);
}

/****************************************************************************
*
*  Helper search routine to find supported rates. 
*
*  Returns 1 if found, otherwise 0
*
***************************************************************************/
static int findFreq( HAL_AUDIO_FREQUENCIES *freq, int rate )
{
   int i;
   for ( i = 0; i < freq->numSettings; i++ )
   {
      if ( rate == freq->freqSetting[i] )
         return 1;
   }
   return 0;
}

static void open_worker_thread(alsaThread_t *thread, snd_card_brcm_pcm_t *dpcm)
{
   //  Set up the kernel thread for processing audio
   //
   sema_init(&thread->sem, 0);
   init_completion(&thread->threadExited);
   thread->dpcm = dpcm;
   thread->keepRunning = 0;
   thread->state = SND_CARD_OPENED;
   thread->threadId = kernel_thread((void *)alsaWorkerThread, (void *) thread, 0);
}

static void close_worker_thread(alsaThread_t *thread)
{
   if (thread->threadId >= 0)
   {
      thread->state = SND_CARD_CLOSING;
      thread->keepRunning = 0;
      up(&thread->sem); /* Wake task up if sleeping */
      kill_proc_info(SIGTERM, SEND_SIG_PRIV, thread->threadId);
      if (!wait_for_completion_timeout(&thread->threadExited, msecs_to_jiffies(500)))
      {
            struct snd_pcm_substream *substream = thread->dpcm->substream;
            printk( KERN_ERR "ALSA worker completion for stream %d, device %d timed-out\n", substream->stream, substream->pcm->device);
      }
   }
   thread->threadId = -1;
   thread->state = SND_CARD_IDLE;
}


/****************************************************************************
*
*  snd_card_brcm_open
*
*  Sound callback for opening a stream.
*
***************************************************************************/
static int snd_card_brcm_open(struct snd_pcm_substream * substream)
{
   struct snd_pcm_runtime *runtime = substream->runtime;
   snd_card_brcm_pcm_t *dpcm;
   int err;
   HAL_AUDIO_FREQUENCIES freq;
   alsaThread_t *thread =  getSubstreamThread(substream);

   /* check if another user is using this sound card */
   if( thread->state != SND_CARD_IDLE )
   {
      printk( KERN_ERR "ALSA PCM substream %d being used by another user, failed to continue\n", substream->stream );
      return -EPERM;
   }

   dpcm = kcalloc(1, sizeof(*dpcm), GFP_KERNEL);
   if (dpcm == NULL)
      return -ENOMEM;

   /* Use preallocated hook instead of creating a new one because everytime you create a new hook, the halaudio-hook
    * reset the audio gain into DEEP-SLEEP (Mute), the last gain value that was set to the soundcard will be lost.
    */
   dpcm->alsaHookId = thread->alsaHookId; // use preallocated hook-id of the thread.

   /* Determine supported frequencies */
   if (halAudio_getFrequencies( HAL_AUDIO_MODE_ALSA, &freq ))
   {
      printk( KERN_ERR "ALSA: Failed to query supported frequencies\n" );
      kfree(dpcm);
      return -EFAULT;
   }

#ifdef HALAUDIO_SUPER_USER_MODE
    /* Put in superuser mode to have full control for multi-streams */
   halAudio_setSuperUser(dpcm->alsaHookId, 1, HAL_AUDIO_MODE_ALSA);
#endif

   spin_lock_init(&dpcm->lock);
   dpcm->substream = substream;
   runtime->private_data = dpcm;
   runtime->private_free = snd_card_brcm_runtime_free;
   runtime->hw = (SNDRV_PCM_STREAM_CAPTURE == substream->stream) ? snd_card_brcm_capture : snd_card_brcm_playback;
#if 0 // Removed, this is for testing non-interleaved device and mmap
   if (substream->pcm->device & 1)
   {
      runtime->hw.info &= ~SNDRV_PCM_INFO_INTERLEAVED;
      runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;
   }
   if (substream->pcm->device & 2)
   {
      runtime->hw.info &= ~(SNDRV_PCM_INFO_MMAP|SNDRV_PCM_INFO_MMAP_VALID);
   }
#endif
   SND_LOG( "numSettings=%i", freq.numSettings );

   if ( freq.numSettings > 0 ) 
   {
      unsigned int ratesflag = 0;
      int i, len;

      for ( i = 0; i < freq.numSettings; i++ )
      {
         SND_LOG( "   %i: %i Hz", i, freq.freqSetting[i] );
      }

      /* Update hardware configuration */

      /* Assume ordered frequency list */
      runtime->hw.rate_min = freq.freqSetting[0];
      runtime->hw.rate_max = freq.freqSetting[freq.numSettings-1];

      if ( findFreq( &freq, 8000 ))
      {
         ratesflag |= SNDRV_PCM_RATE_8000;
      }
      if ( findFreq( &freq, 16000 ))
      {
         ratesflag |= SNDRV_PCM_RATE_16000;
      }
      if ( findFreq( &freq, 22050 ))
      {
         ratesflag |= SNDRV_PCM_RATE_22050;
      }
      if ( findFreq( &freq, 32000 ))
      {
         ratesflag |= SNDRV_PCM_RATE_32000;
      }
      if ( findFreq( &freq, 44100 ))
      {
         ratesflag |= SNDRV_PCM_RATE_44100;
      }
      if ( findFreq( &freq, 48000 ))
      {
         ratesflag |= SNDRV_PCM_RATE_48000;
      }
      runtime->hw.rates = ratesflag;

      /* Update hardware constraints */

      len = MIN( BRCM_SND_MAX_RATES_CONSTRAINT, freq.numSettings );
      for ( i = 0; i < len; i++ )
      {
         rates[i] = freq.freqSetting[i];
      }
      hw_constraints_rates.count = len;

      SND_LOG( "rates=0x%x len=%i", ratesflag, len );
   }

   if (SNDRV_PCM_STREAM_CAPTURE == substream->stream)
        err = add_capture_constraints(runtime);
   else
        err = add_playback_constraints(runtime);

   if (err < 0)
   {
      kfree(dpcm);
      printk( KERN_ERR "Failed to add stream constraints\n" );
      return err;
   }
   /* Start transfer thread */
   open_worker_thread(thread, dpcm);
   return 0;
}

/****************************************************************************
*
*  snd_card_brcm_playback_open
*
*  Sound callback for opening a playback stream.
*
***************************************************************************/
static int snd_card_brcm_playback_open(struct snd_pcm_substream * substream)
{
   return snd_card_brcm_open(substream);
}

/****************************************************************************
*
*  snd_card_brcm_capture_open
*
*  Sound callback for opening a capture stream.
*
***************************************************************************/
static int snd_card_brcm_capture_open(struct snd_pcm_substream * substream)
{
   return snd_card_brcm_open(substream);
}

/****************************************************************************
*
*  snd_card_brcm_close
*
*  Closing the sound card
*
***************************************************************************/
static int snd_card_brcm_close(struct snd_pcm_substream * substream)
{
    snd_card_brcm_pcm_t *dpcm = substream->runtime->private_data;
    alsaThread_t *thread = getSubstreamThread(substream);

    if ( thread->state == SND_CARD_ACTIVE || thread->state == SND_CARD_OPENED)
    {
        printk( KERN_INFO "snd_card_brcm_close, stream-type %i, dev %i, stream %i\n", substream->stream, substream->pcm->device, substream->number);

#if defined(CONFIG_PLAT_BCM476X)
        /* Shutdown the component of the audio block */
        if (SNDRV_PCM_STREAM_CAPTURE == substream->stream)
        {
            if (2 == substream->pcm->device) 
            {
                halAudio_blockDisable( dpcm->alsaHookId, HAL_AUDIO_HANDFREE_MIC );
            }
            else
            {
                // set the default mic and speaker source to EAR mic
                halAudio_blockDisable( dpcm->alsaHookId, HAL_AUDIO_EAR_MIC );
                halAudio_blockDisable( dpcm->alsaHookId, HAL_AUDIO_AUX_MIC );
            }
        }
        else
        {
            if (2 == substream->pcm->device ) 
            {
                halAudio_blockDisable( dpcm->alsaHookId, HAL_AUDIO_HANDFREE_SPKR );
            }
            else
            {
               // set the default mic and speaker source to EAR spkr 
               halAudio_blockDisable( dpcm->alsaHookId, HAL_AUDIO_EAR_SPKR );
               halAudio_blockDisable( dpcm->alsaHookId, HAL_AUDIO_AUX_SPKR );
            }
        }
#else
        if (SNDRV_PCM_STREAM_CAPTURE == substream->stream)
        {
            // set the default mic and speaker source to EAR mic
            halAudio_blockDisable( dpcm->alsaHookId, HAL_AUDIO_EAR_MIC );
            halAudio_blockDisable( dpcm->alsaHookId, HAL_AUDIO_AUX_MIC );
        }
        else
        {
            // set the default mic and speaker source to EAR spkr 
            halAudio_blockDisable( dpcm->alsaHookId, HAL_AUDIO_EAR_SPKR );
            halAudio_blockDisable( dpcm->alsaHookId, HAL_AUDIO_AUX_SPKR );
        }
#endif
        close_worker_thread(thread);
        /* we have to release control halaudio so another user can use it */
        halAudio_disableAudio( dpcm->alsaHookId );
        halAudio_releaseControl( dpcm->alsaHookId );
    }

#ifdef HALAUDIO_SUPER_USER_MODE
    /* Remove superuser mode for other non-superuser application to use the halaudio. */
   halAudio_setSuperUser(dpcm->alsaHookId, 0, HAL_AUDIO_MODE_ALSA);
#endif
    return 0;
}
/****************************************************************************
*
*  snd_card_brcm_playback_close
*
*  Sound callback for closing a playback stream.
*
***************************************************************************/
static int snd_card_brcm_playback_close(struct snd_pcm_substream * substream)
{
    return snd_card_brcm_close(substream);
}

/****************************************************************************
*
*  snd_card_brcm_capture_close
*
*  Sound callback for closing a capture stream.
*
***************************************************************************/
static int snd_card_brcm_capture_close(struct snd_pcm_substream * substream)
{
    return snd_card_brcm_close(substream);
}

static struct snd_pcm_ops snd_card_brcm_playback_ops = {
   .open =        snd_card_brcm_playback_open,
   .close =       snd_card_brcm_playback_close,
   .ioctl =       snd_pcm_lib_ioctl,
   .hw_params =   snd_card_brcm_hw_params,
   .hw_free =     snd_card_brcm_hw_free,
   .prepare =     snd_card_brcm_playback_prepare,
   .trigger =     snd_card_brcm_playback_trigger,
   .pointer =     snd_card_brcm_playback_pointer,
};

static struct snd_pcm_ops snd_card_brcm_capture_ops = {
   .open =        snd_card_brcm_capture_open,
   .close =       snd_card_brcm_capture_close,
   .ioctl =       snd_pcm_lib_ioctl,
   .hw_params =   snd_card_brcm_hw_params,
   .hw_free =     snd_card_brcm_hw_free,
   .prepare =     snd_card_brcm_capture_prepare,
   .trigger =     snd_card_brcm_capture_trigger,
   .pointer =     snd_card_brcm_capture_pointer,
};

/****************************************************************************
*
*  snd_card_brcm_pcm
*
*  Sound callback for allocating a PCM sound card.
*
***************************************************************************/
static int __init snd_card_brcm_new_pcm(snd_card_brcm_t *bcmCard, int device, int substreams)
{
   struct snd_pcm *pcm;
   const char *dev_name;
   int err;

   if ((err = snd_pcm_new(bcmCard->card, "PCM", device, substreams, substreams, &pcm)) < 0)
      return err;
   snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_card_brcm_playback_ops);
   snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_card_brcm_capture_ops);
   pcm->private_data = bcmCard;
   pcm->info_flags = 0;
   if (device < MAX_PCM_DEVICES && pcm_dev_name[device])
       dev_name = pcm_dev_name[device];
   else
       dev_name = "Sound";
   strncpy(pcm->name, dev_name, sizeof(pcm->name));

   snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
                                         snd_dma_continuous_data(GFP_KERNEL),
                                         0, 64*1024);
   return 0;
}

#define BRCM_VOLUME(xname, xindex, addr) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, .index = xindex, \
  .info = snd_brcm_volume_info, \
  .get = snd_brcm_volume_get,   \
  .put = snd_brcm_volume_put,   \
  .private_value = addr }

/* Add 1 for the mute/sleep setting */
#define NUM_NOTCHES_ANA ((((cap->analog.maxDB - cap->analog.minDB) / cap->analog.range.fixedStepSize)+1) + 1)
#define NUM_NOTCHES_DIG ((((cap->digital.maxDB - cap->digital.minDB) / cap->digital.range.fixedStepSize)+1) + 1)
#define NUM_NOTCHES_MAX (HAL_AUDIO_MAX_NUM_DB_SETTINGS + 1)

//  These tables are used to map db values of the audio blocks to "notches".
//
static int gNotchesConfigured;


struct notch_info
{
   int   notches;                      /* number of notches supported */
   int   db[NUM_NOTCHES_MAX];          /* notch value to dB mapping */
};

struct block_info
{
   HAL_AUDIO_BLOCK   id;               /* Hal Audio block id */
   struct notch_info ana_info;         /* Analog gain notch info */
   struct notch_info dig_info;         /* Digital gain notch info */
};

//  Table of gain information for supported blocks
//
#define MAX_NUM_AUDIO_BLOCKS        4
static struct block_info gBlockInfo[MAX_NUM_AUDIO_BLOCKS] =
{
   {
      .id = HAL_AUDIO_EAR_SPKR,
   },
   {
      .id = HAL_AUDIO_AUX_SPKR,
   },
   {
      .id = HAL_AUDIO_EAR_MIC,
   },
   {
      .id = HAL_AUDIO_AUX_MIC,
   },
};

/****************************************************************************
*
*  setupNotches
*
*  This routine is used to fill in one notch table with given parameters.
*
***************************************************************************/
static int setupNotches(
   HAL_AUDIO_CAPABILITIES *cap,        //< Block capabilities
   struct notch_info      *ana_info,   //< Analog gain notch info
   struct notch_info      *dig_info    //< Analog gain notch info
)
{
   int i;
   int db;

   //  First notch (zero) is always sleep/mute
   //
   dig_info->db[0] = AUDIO_MUTE_GAIN;
   ana_info->db[0] = AUDIO_MUTE_GAIN;

   //  Digital notch information
   //
   if (cap->digital.rangeType == HAL_AUDIO_RANGETYPE_FIXED_STEPSIZE)
   {
      //  Fixed range.
      //
      if (NUM_NOTCHES_DIG > NUM_NOTCHES_MAX)
      {
         printk("Too many digital notches (%d, max=%d)\n", NUM_NOTCHES_DIG, NUM_NOTCHES_MAX );
         return -1;
      }
      dig_info->notches = NUM_NOTCHES_DIG;

      for (i = 1, db = cap->digital.minDB; i < dig_info->notches; 
            i++, db += cap->digital.range.fixedStepSize)
      {
         dig_info->db[i] = db;
      }
   }
   else
   {
      //  Listed range.
      //
      if (cap->digital.range.list.numSettings > NUM_NOTCHES_MAX)
      {
         printk("Too many EAR_SPKR notches\n");
         return -1;
      }
      dig_info->notches = cap->digital.range.list.numSettings;

      for (i = 1; i < cap->digital.range.list.numSettings; i++)
      {
         dig_info->db[i] = cap->digital.range.list.dbSetting[i-1];
      }
   }

   //  Analog notch information
   //
   if (cap->analog.rangeType == HAL_AUDIO_RANGETYPE_FIXED_STEPSIZE)
   {
      //  Fixed range.
      //
      if (NUM_NOTCHES_ANA > NUM_NOTCHES_MAX)
      {
         printk("Too many analog notches (%d, max=%d)\n", NUM_NOTCHES_ANA, NUM_NOTCHES_MAX );
         return -1;
      }
      ana_info->notches = NUM_NOTCHES_ANA;

      for (i = 1, db = cap->analog.minDB; i < ana_info->notches;
            i++, db += cap->analog.range.fixedStepSize)
      {
         ana_info->db[i] = db;
      }
   }
   else
   {
      //  Listed range.
      //
      if (cap->analog.range.list.numSettings > NUM_NOTCHES_MAX)
      {
         printk("Too many EAR_SPKR notches\n");
         return -1;
      }
      ana_info->notches = cap->analog.range.list.numSettings;

      for (i = 1; i < cap->analog.range.list.numSettings; i++)
      {
         ana_info->db[i] = cap->analog.range.list.dbSetting[i-1];
      }
   }

   return 0;
}

/****************************************************************************
* 
*  Helper routine to retrieve the block info structure
*
***************************************************************************/
static struct block_info *getBlockInfo(HAL_AUDIO_BLOCK block)
{
   int                i;
   struct block_info *bp;

   bp = &gBlockInfo[0];

   for ( i = 0; i < MAX_NUM_AUDIO_BLOCKS; i++, bp++ )
   {
      if ( bp->id == block )
      {
         return bp;
      }
   }

   return NULL;
}

/****************************************************************************
*
*  Helper routine to determine notch value from db
*
***************************************************************************/
static int getNotchValue(struct notch_info *info, int db)
{
   int i;
   for (i = 0; i < info->notches; i++)
   {
      if (info->db[i] >= db)
      {
         return i;
      }
   }
   return 0;
}

/****************************************************************************
*
*  Helper routine to retrieve db gain value from notch value.
*
***************************************************************************/
static inline int getNotchVolume(struct notch_info *info, int notch )
{
   return info->db[notch];
}

/****************************************************************************
*
*  configure_notches
*
*  Configure the notch to DB tables.
*
***************************************************************************/
static int configure_notches(void)
{
   HAL_AUDIO_CAPABILITIES  cap;
   struct block_info      *binfop;
   int                     i;

   //  Setup notch to db conversion structures.
   //
   for (i = 0; i < MAX_NUM_AUDIO_BLOCKS; i++)
   {
      binfop = &gBlockInfo[i];
      if ( binfop->id )
      {
         if (halAudio_getGains( binfop->id, &cap ))
         {
            return -1;
         }
         if (setupNotches(&cap, &binfop->ana_info, &binfop->dig_info) < 0)
         {
            return -1;
         }
      }
   }
   gNotchesConfigured = 1;
   return 0;
}

/****************************************************************************
*
*  snd_brcm_volume_info
*
*  Control callback for getting volume information.
*
***************************************************************************/
static int snd_brcm_volume_info(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_info * uinfo)
{
   int addr = kcontrol->private_value;
   struct block_info *binfop = NULL;

   if (!gNotchesConfigured)
   {
      if (configure_notches())
      {
         printk( KERN_ERR "BRCM ALSA: %s failed to configure gain notches\n", __FUNCTION__ );
         return -1;
      }
   }

   uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
   uinfo->count = 1;
   uinfo->value.integer.min = 0;

   if (addr == MIXER_ADDR_SPEAKER || addr == MIXER_ADDR_SPEAKER_DIG)
   {
      binfop = getBlockInfo( HAL_AUDIO_EAR_SPKR );
   }
   else if (addr == MIXER_ADDR_AUX || addr == MIXER_ADDR_AUX_DIG)
   {
      binfop = getBlockInfo( HAL_AUDIO_AUX_SPKR );
   }
#if defined(CONFIG_PLAT_BCM476X)
   else if (addr == MIXER_ADDR_MIC)
   {
      binfop = getBlockInfo( HAL_AUDIO_EAR_MIC );
   }
   else if (addr == MIXER_ADDR_AUX_MIC)
   {
      binfop = getBlockInfo( HAL_AUDIO_AUX_MIC );
   }
#endif

   if (!binfop)
   {
      return -EINVAL;
   }

   if (addr == MIXER_ADDR_SPEAKER_DIG || addr == MIXER_ADDR_AUX_DIG)
   {
      uinfo->value.integer.max = binfop->dig_info.notches - 1;;
   }
   else
   {
      uinfo->value.integer.max = binfop->ana_info.notches - 1;
   }

   return 0; 
}

/****************************************************************************
*
*  snd_brcm_volume_get
*
*  Control callback for getting volume level.
*
***************************************************************************/
static int snd_brcm_volume_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
   snd_card_brcm_t *bcmCard = snd_kcontrol_chip(kcontrol);
   unsigned long flags;
   int addr = kcontrol->private_value;
   int volume;
   int db;
   HAL_AUDIO_BLOCK block;
   struct block_info *binfop;

   if (!gNotchesConfigured)
   {
      if (configure_notches())
      {
         printk( KERN_ERR "BRCM ALSA: %s failed to configure gain notches\n", __FUNCTION__ );
         return -1;
      }
   }

   switch (addr)
   {
      case MIXER_ADDR_SPEAKER:
      case MIXER_ADDR_SPEAKER_DIG:
      default:
      {
         block = HAL_AUDIO_EAR_SPKR;
      }
      break;

      case MIXER_ADDR_AUX:
      case MIXER_ADDR_AUX_DIG:
      {
         block = HAL_AUDIO_AUX_SPKR;
      }
      break;

#if defined(CONFIG_PLAT_BCM476X)
      case MIXER_ADDR_MIC:
         block = HAL_AUDIO_EAR_MIC;
      break;

      case MIXER_ADDR_AUX_MIC:
         block = HAL_AUDIO_AUX_MIC;
      break;
#endif
   }

   binfop = getBlockInfo( block );
   if (!binfop)
   {
      printk( KERN_ERR "BRCM ALSA: %s block 0x%x is invalid\n", __FUNCTION__, block );
      return -EINVAL;
   }

   if (addr == MIXER_ADDR_SPEAKER_DIG || addr == MIXER_ADDR_AUX_DIG)
   {
      db = halAudio_getDigGain( gMyAlsaHookId, block );
      volume = getNotchValue( &binfop->dig_info, db );
   }
   else
   {
      db = halAudio_getAnaGain( gMyAlsaHookId, block );
      volume = getNotchValue( &binfop->ana_info, db );
   }
//	printk("ADDR=%d, addr=%d volume=%d\n", MIXER_ADDR_AUX, addr, volume);
   spin_lock_irqsave(&bcmCard->mixer_lock, flags);
   bcmCard->mixer_volume[addr][0] = volume;
   bcmCard->mixer_volume[addr][1] = volume;
   ucontrol->value.integer.value[0] = bcmCard->mixer_volume[addr][0];
   ucontrol->value.integer.value[1] = bcmCard->mixer_volume[addr][1];
   spin_unlock_irqrestore(&bcmCard->mixer_lock, flags);

   return 0;
}

/****************************************************************************
*
*  snd_brcm_volume_put
*
*  Control callback for setting volume level.
*
***************************************************************************/
#define BRCM_LIMIT(var,min,max)  do { if ((var)<(min)) (var) = (min); else if ((var)>(max)) (var) = (max); } while (0)
static int snd_brcm_volume_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
   snd_card_brcm_t *bcmCard = snd_kcontrol_chip(kcontrol);
   unsigned long flags;
   int change, addr = kcontrol->private_value;
   int left, right, digital;
   int max_vol;
   HAL_AUDIO_BLOCK block = -1;
   struct block_info *binfop;

   if (!gNotchesConfigured)
   {
      if (configure_notches())
      {
         printk( KERN_ERR "BRCM ALSA: %s failed to configure gain notches\n", __FUNCTION__ );
         return -1;
      }
   }

   switch (addr)
   {
      case MIXER_ADDR_SPEAKER:
      case MIXER_ADDR_SPEAKER_DIG:
      default:
      {
         block = HAL_AUDIO_EAR_SPKR;
      }
      break;

      case MIXER_ADDR_AUX:
      case MIXER_ADDR_AUX_DIG:
      {
         block = HAL_AUDIO_AUX_SPKR;
      }
      break;

#if defined(CONFIG_PLAT_BCM476X)
      case MIXER_ADDR_MIC:
         block = HAL_AUDIO_AUX_MIC; // map to voice path MICs
      break;
#endif
   }

   binfop = getBlockInfo( block );
   if (!binfop)
   {
      printk( KERN_ERR "BRCM ALSA: %s block 0x%x is invalid\n", __FUNCTION__, block );
      return -EINVAL;
   }
   digital = (addr == MIXER_ADDR_SPEAKER_DIG || addr == MIXER_ADDR_AUX_DIG);
   max_vol = digital ? binfop->dig_info.notches : binfop->ana_info.notches;
   left = ucontrol->value.integer.value[0];
   BRCM_LIMIT( left, 0, max_vol );
   right = ucontrol->value.integer.value[1];
   BRCM_LIMIT( right, 0, max_vol );

   spin_lock_irqsave(&bcmCard->mixer_lock, flags);
   change = bcmCard->mixer_volume[addr][0] != left ||
            bcmCard->mixer_volume[addr][1] != right;
   bcmCard->mixer_volume[addr][0] = left;
   bcmCard->mixer_volume[addr][1] = right;
   spin_unlock_irqrestore(&bcmCard->mixer_lock, flags);

   if (change)
   {
      //  Update the endpoint settings for the audio volume.
      //
      if (digital)
      {
         halAudio_setDigGain( gMyAlsaHookId, block, getNotchVolume( &binfop->dig_info, left ) );
      }
      else
      {
         halAudio_setAnaGain( gMyAlsaHookId, block, getNotchVolume( &binfop->ana_info, left ) );
      }
   }
   return change;
}

#define BRCM_PLAYDST(xname, xindex, addr) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, .index = xindex, \
  .info = snd_brcm_playdst_info, \
  .get = snd_brcm_playdst_get, .put = snd_brcm_playdst_put, \
  .private_value = addr }

/****************************************************************************
*
*  snd_brcm_playdst_info
*
*  Control callback for getting play destination information.
*
***************************************************************************/
static int snd_brcm_playdst_info(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_info * uinfo)
{
   uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
   uinfo->count = 1;
   uinfo->value.integer.min = 0;
   uinfo->value.integer.max = 1;
   return 0;
}

/****************************************************************************
*
*  snd_brcm_playdst_get
*
*  Control callback for getting play destination setting.
*
***************************************************************************/
static int snd_brcm_playdst_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
   snd_card_brcm_t *bcmCard = snd_kcontrol_chip(kcontrol);
   unsigned long flags;
   int addr = kcontrol->private_value;
   HAL_AUDIO_SPKRSOURCE audioSource;

   if( halAudio_blockQueryGain( gMyAlsaHookId, HAL_AUDIO_EAR_SPKR ) != AUDIO_MUTE_GAIN )
   {
      audioSource = HAL_AUDIO_SPKRSOURCE_EAR;
   }
   else
   {
      audioSource = HAL_AUDIO_SPKRSOURCE_AUX;
   }
   spin_lock_irqsave(&bcmCard->mixer_lock, flags);
   bcmCard->play_dst[addr][0] = (int) audioSource;
   bcmCard->play_dst[addr][1] = (int) audioSource;
   ucontrol->value.integer.value[0] = bcmCard->play_dst[addr][0];
   ucontrol->value.integer.value[1] = bcmCard->play_dst[addr][1];
   spin_unlock_irqrestore(&bcmCard->mixer_lock, flags);
   return 0;
}

/****************************************************************************
*
*  snd_brcm_playdst_put
*
*  Control callback for setting play destination setting.
*
***************************************************************************/
static int snd_brcm_playdst_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
   snd_card_brcm_t *bcmCard = snd_kcontrol_chip(kcontrol);
   unsigned long flags;
   int change, addr = kcontrol->private_value;
   int source;

   source = ucontrol->value.integer.value[0];
   spin_lock_irqsave(&bcmCard->mixer_lock, flags);
   change = bcmCard->play_dst[addr][0] != source;
   bcmCard->play_dst[addr][0] = source;
   spin_unlock_irqrestore(&bcmCard->mixer_lock, flags);
   if (change)
   {
      //  Update the endpoint settings for where the audio is going.
      //
      if( source == HAL_AUDIO_SPKRSOURCE_EAR )
      {
         halAudio_blockEnable( gMyAlsaHookId, HAL_AUDIO_EAR_SPKR );
         halAudio_blockDisable( gMyAlsaHookId, HAL_AUDIO_AUX_SPKR );
         halAudio_setActiveCodec( gMyAlsaHookId, HW_HANDSET_CODEC );
      }
      else if ( source == HAL_AUDIO_SPKRSOURCE_AUX )
      {
         halAudio_blockDisable( gMyAlsaHookId, HAL_AUDIO_EAR_SPKR );
         halAudio_blockEnable( gMyAlsaHookId, HAL_AUDIO_AUX_SPKR );
         halAudio_setActiveCodec( gMyAlsaHookId, HW_HEADSET_CODEC );
      }
   }
   return change;
}

#if 0 // defined(CONFIG_PLAT_BCM476X) ( Removed, can not use this anymore, have to go thru BT subdevice for echo cancellation)
#define BRCM_HANDFREE(xname, xindex, addr) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
  .name = xname, \
  .index = xindex, \
  .info = snd_brcm_handfree_info, \
  .get = snd_brcm_handfree_get, \
  .put = snd_brcm_handfree_put, \
  .private_value = addr }

/****************************************************************************
*
*  snd_brcm_handfree_info
*
*  Handfree configuration information
*
***************************************************************************/
static int snd_brcm_handfree_info(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_info * uinfo)
{
   uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
   uinfo->count = 1;
   uinfo->value.integer.min = 0;
   uinfo->value.integer.max = 1;
   return 0;
}

/****************************************************************************
*
*  snd_brcm_handfree_get
*
*  Control callback for getting handfree parameter.
*
***************************************************************************/
static int snd_brcm_handfree_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
   snd_card_brcm_t *bcmCard = snd_kcontrol_chip(kcontrol);
   unsigned long flags;

   spin_lock_irqsave(&bcmCard->mixer_lock, flags);
   ucontrol->value.integer.value[0] = bcmCard->hand_free;
   spin_unlock_irqrestore(&bcmCard->mixer_lock, flags);
   return 0;
}

/****************************************************************************
*
*  snd_brcm_handfree_put
*
*  Control callback for setting handfree parameter.
*
***************************************************************************/
static int snd_brcm_handfree_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
   snd_card_brcm_t *bcmCard = snd_kcontrol_chip(kcontrol);
   unsigned long flags;
   int set_value, change;

   set_value = ucontrol->value.integer.value[0];
   spin_lock_irqsave(&bcmCard->mixer_lock, flags);
   change = bcmCard->hand_free != set_value;
   spin_unlock_irqrestore(&bcmCard->mixer_lock, flags);
   if (change)
   {
        if (set_value)
        {
            int freq = 8000;
            if ( 0 == halAudio_getControl( gMyAlsaHookId, HAL_AUDIO_MODE_ALSA, freq ))
            {
               halAudio_enableAudio( gMyAlsaHookId, 1 ); // Must enable audio first to power audio block on
               halAudio_setAnaGain (gMyAlsaHookId, HAL_AUDIO_HANDFREE_SPKR, 0);
               halAudio_blockEnable(gMyAlsaHookId, HAL_AUDIO_HANDFREE_SPKR);
               spin_lock_irqsave(&bcmCard->mixer_lock, flags);
               bcmCard->hand_free = 1;
               spin_unlock_irqrestore(&bcmCard->mixer_lock, flags);
            }
        }
        else
        {
           halAudio_blockDisable( gMyAlsaHookId, HAL_AUDIO_HANDFREE_SPKR);
           halAudio_releaseControl( gMyAlsaHookId );
           spin_lock_irqsave(&bcmCard->mixer_lock, flags);
           bcmCard->hand_free = 0;
           spin_unlock_irqrestore(&bcmCard->mixer_lock, flags);
       }
   }
   return change;
}
#endif // defined(CONFIG_PLAT_BCM476X)

static struct snd_kcontrol_new snd_brcm_controls[] = {
   BRCM_VOLUME("Speaker Volume", 0, MIXER_ADDR_SPEAKER),
   BRCM_VOLUME("Aux Volume", 0, MIXER_ADDR_AUX),
   BRCM_VOLUME("SpeakerDig Volume", 0, MIXER_ADDR_SPEAKER_DIG),
   BRCM_VOLUME("AuxDig Volume", 0, MIXER_ADDR_AUX_DIG),
#if defined(CONFIG_PLAT_BCM476X)
   BRCM_VOLUME("Stereo-MIC", 0, MIXER_ADDR_MIC),
   BRCM_VOLUME("Mono-MIC", 0, MIXER_ADDR_AUX_MIC),
   // BRCM_HANDFREE("Handfree", 0, MIXER_ADDR_HANDFREE), // Removed, can not use this anymore, have to go thru BT subdevice for echo cancellation
#else
   BRCM_PLAYDST("Play Switch", 0, MIXER_ADDR_SPEAKER),
#endif
};

/****************************************************************************
*
*  snd_card_brcm_new_mixer
*
*  Set up the mixer controls for this card.
*
***************************************************************************/
static int __init snd_card_brcm_new_mixer(snd_card_brcm_t * bcmCard)
{
   struct snd_card *card = bcmCard->card;
   unsigned int idx;
   int err;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28)
   snd_assert(bcmCard != NULL, return -EINVAL);
#endif
   spin_lock_init(&bcmCard->mixer_lock);
   strcpy(card->mixername, "Broadcom Mixer");

   for (idx = 0; idx < ARRAY_SIZE(snd_brcm_controls); idx++)
   {
      if ((err = snd_ctl_add(card, snd_ctl_new1(&snd_brcm_controls[idx], bcmCard))) < 0)
         return err;
   }
   tomtom_add_nashville_controls(card, (void *) bcmCard);
   return 0;
}

/****************************************************************************
*
*  snd_card_brcm_probe
*
*  Configure the sound card.
*
***************************************************************************/
static int __init snd_card_brcm_probe(int dev)
{
   struct snd_card *card;
   struct snd_card_brcm *bcmCard;
   int idx, err;

   if (!enable[dev])
      return -ENODEV;
   card = snd_card_new(index[dev], id[dev], THIS_MODULE,
                       sizeof(struct snd_card_brcm));
   if (card == NULL)
      return -ENOMEM;
   bcmCard = (struct snd_card_brcm *)card->private_data;
   bcmCard->card = card;
   for (idx = 0; idx < MAX_PCM_DEVICES && idx < pcm_devs[dev]; idx++)
   {
      if (pcm_substreams[dev] < 1)
         pcm_substreams[dev] = 1;
      if (pcm_substreams[dev] > MAX_PCM_SUBSTREAMS)
         pcm_substreams[dev] = MAX_PCM_SUBSTREAMS;
      if ((err = snd_card_brcm_new_pcm(bcmCard, idx, pcm_substreams[dev])) < 0)
         goto __nodev;
   }
   /* Add mixer control */
   if ((err = snd_card_brcm_new_mixer(bcmCard)) < 0)
      goto __nodev;
   strcpy(card->driver, "Broadcom");
   strcpy(card->shortname, "Broadcom");
   sprintf(card->longname, "Broadcom PCM %i", dev + 1);
   if ((err = snd_card_register(card)) == 0)
   {
      snd_brcm_cards[dev] = card;
#ifdef CONFIG_TOMTOM_NASHVILLE_SCENARI_BCM4760
      alc5627_i2c_init();
#endif
      return 0;
   }
__nodev:
   snd_card_free(card);
   return err;
}

int bcm476x_scenari_set_volume_db(int target_db)
{
	struct block_info	*binfop;
	int			db_index;

	/* Broadcom still hasn't communicated on other values, so just assume HAL_AUDIO_EAR_SPKR for now */
	binfop		= getBlockInfo(HAL_AUDIO_EAR_SPKR);

	/* dB "gains" are passed by Nashville as positive values, when they are actually negative ones. */
	target_db	*= -1;	/* Fix that for the card driver. */

	/* Set the gain to the closest value we could find in the table. */
	halAudio_setAnaGain(gMyAlsaHookId, HAL_AUDIO_EAR_SPKR, target_db /*binfop->ana_info.db[db_index] */);
	return 1;
}
EXPORT_SYMBOL(bcm476x_scenari_set_volume_db);


static int alsa_card_brcm_probe(struct platform_device *pdev)
{
    int dev, cards;
    unsigned int idx, substream;

    for (dev = cards = 0; dev < SNDRV_CARDS && enable[dev]; dev++)
    {
        if (snd_card_brcm_probe(dev) < 0)
        {
            #ifdef MODULE
             printk(KERN_ERR "Broadcom sound interface #%i not found or device busy\n", dev + 1);
            #endif
             break;
        }
        cards++;
    }
    if (!cards)
    {
        #ifdef MODULE
          printk(KERN_ERR "Broadcom sound interface not found or device busy\n");
        #endif
          return -ENODEV;
    }

    memset (&gWorkerThread, 0, sizeof(gWorkerThread));
    for (idx = 0; idx <= SNDRV_PCM_STREAM_LAST; idx++)
        for (dev = 0; dev < MAX_PCM_DEVICES; dev++)
            for (substream = 0; substream < MAX_PCM_SUBSTREAMS; substream++)
            {
                int hookId;
                alsaThread_t *pThread = getThread(idx, dev, substream);
                if (pThread) 
                {
                    //  Register ourselves with the HAL Audio driver.
                    //
                    hookId = halAudio_allocate_client();
                    pThread->threadId = -1;
                    pThread->alsaHookId = hookId;
                }
            }
    /* Set global hook id to hook id of stream 0 */
    gMyAlsaHookId = getThread(0,0,0)->alsaHookId;
    return 0;
}

static int alsa_card_brcm_remove(struct platform_device *pdev)
{
	int idx, dev, substream;

	for (idx = 0; idx < SNDRV_CARDS; idx++)
		snd_card_free(snd_brcm_cards[idx]);

	for (idx = 0; idx <= SNDRV_PCM_STREAM_LAST; idx++) {
		for (dev = 0; dev < MAX_PCM_DEVICES; dev++) {
			for (substream = 0; substream < MAX_PCM_SUBSTREAMS; substream++) {
				alsaThread_t *pThread;

				if ((pThread = getThread(idx, dev, substream)) == NULL) continue;
				if (pThread->threadId >= 0)
					close_worker_thread(pThread);
				if (pThread->alsaHookId >= 0) {
					halAudio_free_client(pThread->alsaHookId);
					pThread->alsaHookId = -1;
				}
			}
		}
	}
	return 0;
}

static struct platform_driver alsa_card_brcm_driver = {
	.probe		= alsa_card_brcm_probe,
	.remove		= alsa_card_brcm_remove,
	.driver		= {
		.name	= ALSA_CARD_BRCM_NAME,
	},
};

static int __init alsa_card_brcm_init(void)
{
	return platform_driver_register(&alsa_card_brcm_driver);
}

static void __exit alsa_card_brcm_exit(void)
{
	platform_driver_unregister(&alsa_card_brcm_driver);
}

module_init(alsa_card_brcm_init)
module_exit(alsa_card_brcm_exit)
