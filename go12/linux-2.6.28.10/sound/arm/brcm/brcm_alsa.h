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
#ifndef __BRCM_ALSA_H__
#define __BRCM_ALSA_H__


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
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>

#include <linux/broadcom/hw_cfg.h>

#define DEBUG_ON

#if defined(DEBUG_ON)
//#define DEBUG(args...)  if (debug) snd_printk(args)
#define DEBUG(args...)  if (debug) printk(args)
#else
#define DEBUG(args...) 
#endif


#define AUDIO_MAX_OUTPUT_VOLUME 100

typedef enum 
{
	PCM_TYPE_PLAYBACK=0,
	PCM_TYPE_CAPTURE,
	PCM_TYPE_TOTAL,
	
	//tag it along
	PCM_TYPE_PLAYBACK_MUTE
} PCM_TYPE;

//event code
#define PCM_EVENT_PLAY_NONE  0x00000000
#define PCM_EVENT_PLAY_START (1<<PCM_TYPE_PLAYBACK)
#define PCM_EVENT_RECD_START (1<<PCM_TYPE_CAPTURE)

#define PCM_EVENT_PLAY_VOLM  (1<<PCM_TYPE_PLAYBACK)
#define PCM_EVENT_RECD_VOLM  (1<<PCM_TYPE_CAPTURE)
#define PCM_EVENT_PLAY_MUTE  (1<<PCM_TYPE_PLAYBACK_MUTE)

typedef struct brcm_alsa_chip
{
	struct snd_card *card;
	//for playback[0] and record[1]
	struct snd_pcm_substream *substream[PCM_TYPE_TOTAL];   //current   
	int rate[PCM_TYPE_TOTAL];
	int buffer_bytes[PCM_TYPE_TOTAL];
	int period_bytes[PCM_TYPE_TOTAL];
	int pcm_ptr[PCM_TYPE_TOTAL];	
	
	int pcm_param_changed;	
	int pcm_playback_volume;
	int pcm_capture_volume;
	int pcm_playback_mute;
} brcm_alsa_chip_t;


//variables
extern int debug;
extern brcm_alsa_chip_t *g_brcm_alsa_chip;


//functions
extern int __devinit brcm_alsa_omx_pcm_new(struct snd_card *card);
extern int brcm_alsa_omx_vc03_init(void);
extern int brcm_alsa_omx_vc03_exit(void);
extern int brcm_alsa_omx_vc03_close(void);


extern int __devinit brcm_alsa_omx_ctl_new(struct snd_card *card);

extern void startPlay(void);
extern void stopPlay(void);
extern void startRecd(void);
extern void stopRecd(void);
extern void snd_brcm_int_handler(int index);
#endif
