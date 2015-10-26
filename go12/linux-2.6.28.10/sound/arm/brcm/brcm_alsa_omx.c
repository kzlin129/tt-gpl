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

#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/kthread.h>

#include "brcm_alsa.h"

//for OMX
#include <linux/broadcom/vc.h>
#include <linux/broadcom/vc03/vcilcs.h>
#include <linux/broadcom/vc03/vcilcs_intern.h>
#include <linux/broadcom/omx/omx.h>
#include "vcilclient.h"


//MACRO
#define ERROR_RETURN(str, error) 	\
	if (error != OMX_ErrorNone) \
	{ \
		snd_printk("\n %s %x\n",str,error); \
		return -1; \
	}

//#define PCM_EVENT_PLAY_NONE  0x00000000
//#define PCM_EVENT_PLAY_START 0x00000001
//#define PCM_EVENT_RECD_START 0x00000002

//GLOBAL

//LOCAL
static int alsa_omx_task_running;
struct task_struct* alsa_omx_task_id;
struct semaphore alsa_omx_sema;
static int alsa_pcm_event = PCM_EVENT_PLAY_NONE;
static int play_init=0;
static int recd_init=0;
static OMX_BUFFERHEADERTYPE *play_buffer; 
static OMX_BUFFERHEADERTYPE *recd_buffer; 

static ILCLIENT_T *client=NULL;
static COMPONENT_T st_play;
static COMPONENT_T st_recd;
static COMPONENT_T *list_play[] = {&st_play, NULL};
static COMPONENT_T *list_recd[] = {&st_recd, NULL};

static int brcm_omx_vc03_start(void);
static int brcm_omx_vc03_stop(void);

/****************************************************************************
*	alsa_omx_task:
*		thread to connect ALSA driver to omx
***************************************************************************/
static int alsa_omx_task( void *unused )
{
	DEBUG("\n alsa_omx_task thread\n");

	while (alsa_omx_task_running)
	{
		//sleep wait for start trigger
		if (down_interruptible(&alsa_omx_sema) == 0)
		{	
			up(&alsa_omx_sema); //signal last period is done
			while ( (alsa_pcm_event & PCM_EVENT_PLAY_START) || 
					(alsa_pcm_event & PCM_EVENT_RECD_START) )
			{
				//wait for last period done from OMX
				if (down_interruptible(&alsa_omx_sema) == 0 )
				{
					brcm_omx_vc03_start(); 
				}
			}
		}
			
		//stop trigger
		brcm_omx_vc03_stop();		
	}

	alsa_omx_task_id = NULL;
	return 0;
}

static void empty_buffer_done_cb(void *data, COMPONENT_T *comp)
{
	DEBUG("\n %lx:empty_buffer_done_cb\n",jiffies);

	snd_brcm_int_handler(0);	//inform ALSA APP
	up(&alsa_omx_sema);
}

static void fill_buffer_done_cb(void *data, COMPONENT_T *comp)
{
	DEBUG("\n %lx:fill_buffer_done_cb \n",jiffies);

	snd_brcm_int_handler(1);	//inform ALSA APP
	up(&alsa_omx_sema);
}


static int brcm_omx_vc03_play_init(void)
{
	OMX_ERRORTYPE error;
	OMX_PARAM_PORTDEFINITIONTYPE param;
	OMX_AUDIO_PARAM_PCMMODETYPE param1;
	int size;
		
	DEBUG("\n %lx:brcm_omx_vc03_play_init\n",jiffies);
	g_brcm_alsa_chip->pcm_ptr[0] = 0;
	
	// playback stuff
	memset(&st_play,0,sizeof(COMPONENT_T));
	error = ilclient_create_component(client, &st_play, "audio_render", 0, 1, 1);
	ERROR_RETURN("ilclient_create_component",error);

	ilclient_disable_port(&st_play,101);
	
	//
	memset(&param, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
	param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
	param.nVersion.nVersion = OMX_VERSION;
	param.nPortIndex = 100;
	param.nBufferCountActual = 1; 
	param.nBufferSize = g_brcm_alsa_chip->period_bytes[0];//20*AUDIO_SAMPLE*2;
	param.eDir = OMX_DirInput;
	param.eDomain = OMX_PortDomainAudio;
	param.format.audio.eEncoding = OMX_AUDIO_CodingPCM;

	error = OMX_SetParameter(st_play.comp, OMX_IndexParamPortDefinition, &param);
	ERROR_RETURN("OMX_SetParameter",error);
	//
	memset(&param1, 0, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
	param1.nSize = sizeof(OMX_AUDIO_PARAM_PCMMODETYPE);
	param1.nVersion.nVersion = OMX_VERSION;
	param1.nPortIndex = 100;
	param1.nChannels = 2;
	param1.eEndian = OMX_EndianLittle;
	param1.bInterleaved = OMX_TRUE;
	param1.nBitPerSample = 16; //32
	param1.nSamplingRate = g_brcm_alsa_chip->rate[0];//8000;//44100;
	param1.eNumData = OMX_NumericalDataSigned;
	param1.ePCMMode = OMX_AUDIO_PCMModeLinear;

	error = OMX_SetParameter(st_play.comp, OMX_IndexParamAudioPcm, &param1);
	ERROR_RETURN("OMX_SetParameter",error);
	//
	error = OMX_SendCommand(st_play.comp, OMX_CommandStateSet, OMX_StateIdle, NULL);
	ERROR_RETURN("OMX_SendCommand",error);

	//
	size = g_brcm_alsa_chip->period_bytes[0];
	//for(i=0; i<param.nBufferCountActual; i++) //is one enough?
	{	
	  	error = OMX_UseBuffer(st_play.comp, &st_play.out_list, 100, st_play.out_list, size, NULL);
		ERROR_RETURN("OMX_UseBuffer",error);			
		
		DEBUG("\n playback %lx:st.out_list=%x st.out_list->private=%x\n",jiffies,
			(int)st_play.out_list,(int)st_play.out_list->pAppPrivate);
			
	}
		
	//					
	error = ilclient_wait_for_event(&st_play, OMX_EventCmdComplete, OMX_CommandStateSet, 
							0, OMX_StateIdle, 0,
	                       ILCLIENT_STATE_CHANGED, PLATFORM_EVENTGROUP_SUSPEND);
	ERROR_RETURN("ilclient_wait_for_event",error);

	//
	if (alsa_pcm_event & PCM_EVENT_PLAY_START )
	{
		error = ilclient_change_component_state(&st_play, OMX_StateExecuting);
		ERROR_RETURN("ilclient_change_component_state",error);
	}

	return 0;
}

static int brcm_omx_vc03_recd_init(void)
{
	OMX_ERRORTYPE error;
	OMX_PARAM_PORTDEFINITIONTYPE param;
	OMX_AUDIO_PARAM_PCMMODETYPE param1;
	int i,size;
		
	DEBUG("\n %lx:brcm_omx_vc03_recd_init\n",jiffies);
	g_brcm_alsa_chip->pcm_ptr[1] = 0;

	//recorder stuff
	memset(&st_recd,0,sizeof(COMPONENT_T));
	error = ilclient_create_component(client, &st_recd, "audio_record", 0, 1, 1);
	ERROR_RETURN("ilclient_create_component record",error);

	ilclient_disable_port(&st_recd,181);

	//
	memset(&param, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
	param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
	param.nVersion.nVersion = OMX_VERSION;
	param.nPortIndex = 180;
	
	error = OMX_GetParameter(st_recd.comp, OMX_IndexParamPortDefinition, &param);
	ERROR_RETURN("OMX_GetParameter record param",error);
	
	DEBUG("\n brcm_omx_vc03_recd_init:nBufferCountActual=%d\n",
		(int)param.nBufferCountActual);
	param.nBufferSize = g_brcm_alsa_chip->period_bytes[1];
	

	error = OMX_SetParameter(st_recd.comp, OMX_IndexParamPortDefinition, &param);
	ERROR_RETURN("OMX_SetParameter record param",error);

	//
	memset(&param1, 0, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
	param1.nSize = sizeof(OMX_AUDIO_PARAM_PCMMODETYPE);
	param1.nVersion.nVersion = OMX_VERSION;
	param1.nPortIndex = 180;
	
	error = OMX_GetParameter(st_recd.comp, OMX_IndexParamAudioPcm, &param1);
	ERROR_RETURN("OMX_GetParameter record param",error);

	param1.nSamplingRate = 16000;//support 16000 only?

	error = OMX_SetParameter(st_recd.comp, OMX_IndexParamAudioPcm, &param1);
	ERROR_RETURN("OMX_SetParameter record param1",error);

	//
	error = OMX_SendCommand(st_recd.comp, OMX_CommandStateSet, OMX_StateIdle, NULL);
	ERROR_RETURN("OMX_SendCommand",error);
	//
	size = g_brcm_alsa_chip->period_bytes[1];
	for(i=0; i<param.nBufferCountActual; i++) 
	{	
	  	error = OMX_UseBuffer(st_recd.comp, &st_recd.out_list, 180, st_recd.out_list, size, NULL);
		ERROR_RETURN("OMX_UseBuffer",error);			
		
		DEBUG("\n record %lx:st.out_list=%x st.out_list->private=%x\n",jiffies,
			(unsigned int)st_recd.out_list,(unsigned int)st_recd.out_list->pAppPrivate);
			
	}
	recd_buffer = st_recd.out_list;
							
	error = ilclient_wait_for_event(&st_recd, OMX_EventCmdComplete, OMX_CommandStateSet, 
							0, OMX_StateIdle, 0,
	                       ILCLIENT_STATE_CHANGED, PLATFORM_EVENTGROUP_SUSPEND);
	ERROR_RETURN("ilclient_wait_for_event record",error);
	
	if (alsa_pcm_event & PCM_EVENT_RECD_START)
	{
		error = ilclient_change_component_state(&st_recd, OMX_StateExecuting);
		ERROR_RETURN("ilclient_change_component_state record",error);
	}

	return 0;
}

static int brcm_omx_vc03_start(void)
{
	struct snd_pcm_runtime *runtime;
	OMX_ERRORTYPE error=OMX_ErrorNone;
		
	DEBUG("\n %lx:brcm_omx_vc03_start %d\n",jiffies,alsa_pcm_event);
	
	//return if stopped by ALSA
	if ( ((alsa_pcm_event & PCM_EVENT_PLAY_START) == 0) &&
		 ((alsa_pcm_event & PCM_EVENT_RECD_START) == 0) )
		return 0;
		
	//check if init?
	if (client==NULL)
	{
		client = ilclient_init();
		if (client==NULL)
		{
			DEBUG("\n %lx:ilclient_init failed\n",jiffies);
			return -1;
		}	
		
		//		
		error = OMX_Init();
		ERROR_RETURN("OMX_Init",error);
	}	
	
	//init playback
	if ( (alsa_pcm_event & PCM_EVENT_PLAY_START) && (play_init==0))
	{
		play_init=1;		 
		brcm_omx_vc03_play_init();
		ilclient_set_empty_buffer_done_callback(client,empty_buffer_done_cb,NULL);
	}
	
	//init record
	if ( (alsa_pcm_event & PCM_EVENT_RECD_START) && (recd_init==0))
	{
		recd_init=1;		 
		brcm_omx_vc03_recd_init();
		ilclient_set_fill_buffer_done_callback(client,fill_buffer_done_cb,NULL);
	}	
				
	if (alsa_pcm_event & PCM_EVENT_PLAY_START)
	{
		//do volume control
		if (g_brcm_alsa_chip->pcm_param_changed & PCM_EVENT_PLAY_VOLM)
		{
			OMX_AUDIO_CONFIG_VOLUMETYPE param;
			
			memset(&param, 0, sizeof(OMX_AUDIO_CONFIG_VOLUMETYPE));
			param.nSize = sizeof(OMX_AUDIO_CONFIG_VOLUMETYPE);
			param.nVersion.nVersion = OMX_VERSION;
			param.nPortIndex = 100;
			
			error = OMX_GetParameter(st_play.comp, OMX_IndexConfigAudioVolume, &param);
			ERROR_RETURN("OMX_GetParameter play volume param",error);
			
			param.bLinear = 1;
			param.sVolume.nValue = g_brcm_alsa_chip->pcm_playback_volume;
					
			error = OMX_SetParameter(st_play.comp, OMX_IndexConfigAudioVolume, &param);
			ERROR_RETURN("OMX_SetParameter play volume param",error);	
			
			error = OMX_GetParameter(st_play.comp, OMX_IndexConfigAudioVolume, &param);
			ERROR_RETURN("OMX_GetParameter play volume param",error);
			
			DEBUG("\n play volume=%d %d %d %d\n",g_brcm_alsa_chip->pcm_playback_volume,
				(int)param.sVolume.nValue,(int)param.sVolume.nMin,(int)param.sVolume.nMax);
			//reset: i don't think we need a lock here
			g_brcm_alsa_chip->pcm_param_changed &= ~PCM_EVENT_PLAY_VOLM;
		}
		
		//do mute
		if (g_brcm_alsa_chip->pcm_param_changed & PCM_EVENT_PLAY_MUTE)
		{
			OMX_AUDIO_CONFIG_MUTETYPE   param;
			
			memset(&param, 0, sizeof(OMX_AUDIO_CONFIG_MUTETYPE));
			param.nSize = sizeof(OMX_AUDIO_CONFIG_MUTETYPE);
			param.nVersion.nVersion = OMX_VERSION;
			param.nPortIndex = 100;			
			param.bMute = g_brcm_alsa_chip->pcm_playback_mute;
					
			error = OMX_SetParameter(st_play.comp, OMX_IndexConfigAudioMute, &param);
			ERROR_RETURN("OMX_SetParameter play mute param",error);	
					
			DEBUG("\n play mute = %d\n",g_brcm_alsa_chip->pcm_playback_mute);
			//reset: i don't think we need a lock here
			g_brcm_alsa_chip->pcm_param_changed &= ~PCM_EVENT_PLAY_MUTE;
		}
		
		runtime = g_brcm_alsa_chip->substream[0]->runtime;
		play_buffer = st_play.out_list;				
		play_buffer->nOffset = 0;
		play_buffer->nFilledLen = g_brcm_alsa_chip->period_bytes[0];
		play_buffer->pBuffer = &(runtime->dma_area[g_brcm_alsa_chip->pcm_ptr[0]]);
	
		error = OMX_EmptyThisBuffer(st_play.comp, play_buffer);
		ERROR_RETURN("OMX_EmptyThisBuffer",error);
				
		DEBUG("\n %lx:playback wait to be done\n",jiffies);
	}

	if (alsa_pcm_event & PCM_EVENT_RECD_START)
	{
		//do volume control
		//?
		
		runtime = g_brcm_alsa_chip->substream[1]->runtime;
			
		recd_buffer->nOffset = 0;		
		recd_buffer->nFilledLen = g_brcm_alsa_chip->period_bytes[1];
		recd_buffer->pBuffer = &(runtime->dma_area[g_brcm_alsa_chip->pcm_ptr[1]]);
		
		DEBUG("\n %lx:record do OMX_FillThisBuffer %x %x\n",jiffies,(int)recd_buffer,(int)recd_buffer);
		error = OMX_FillThisBuffer(st_recd.comp, recd_buffer);
		ERROR_RETURN("OMX_FillThisBuffer",error);
		
		DEBUG("\n %lx:record wait to be done OMX_FillThisBuffer %x\n",jiffies,(int)recd_buffer->pBuffer);
		
		//circular would be nicer
		recd_buffer = (OMX_BUFFERHEADERTYPE*)st_recd.out_list->pAppPrivate;
		if (recd_buffer==NULL)		
			recd_buffer = st_recd.out_list;				
	}
		
	return 0;
}

static int brcm_omx_vc03_stop(void)
{	
	OMX_ERRORTYPE error;
	
	//check if done?
	if (client==NULL) 
		return 0;
	
	DEBUG("\n %lx:brcm_omx_vc03_stop\n",jiffies);
	
	// cleanup
	// playback stuff
	if (play_init)
	{
		ilclient_change_component_state(&st_play, OMX_StateIdle);
		error = OMX_SendCommand(st_play.comp, OMX_CommandStateSet, OMX_StateLoaded, NULL);
		ERROR_RETURN("OMX_SendCommand",error);
	   
		if (st_play.out_list)
		{		
			DEBUG("\n OMX_FreeBuffer=%x %x\n",(int)st_play.out_list,(int)st_play.out_list->pAppPrivate);
			error = OMX_FreeBuffer(st_play.comp, 100, st_play.out_list);	  
			ERROR_RETURN("OMX_FreeBuffer",error);		
		}
		
		ilclient_cleanup_components(list_play);
		play_init=0;
	}
	

	// record stuff
	if (recd_init)
	{
		ilclient_change_component_state(&st_recd, OMX_StateIdle);
		error = OMX_SendCommand(st_recd.comp, OMX_CommandStateSet, OMX_StateLoaded, NULL);
		ERROR_RETURN("OMX_SendCommand",error);
   
		while (st_recd.out_list)
		{		
			OMX_BUFFERHEADERTYPE *next = st_recd.out_list->pAppPrivate;
			
			DEBUG("\n OMX_FreeBuffer=%x %x\n",(int)st_recd.out_list,(int)st_recd.out_list->pAppPrivate);
			error = OMX_FreeBuffer(st_recd.comp, 180, st_recd.out_list);	  
			ERROR_RETURN("OMX_FreeBuffer",error);	
			
			st_recd.out_list = next;	
		}
		
		ilclient_cleanup_components(list_recd);
		recd_init=0;
	}
	
	//
	//
	error = OMX_Deinit();
	ERROR_RETURN("OMX_Deinit",error);
	//
	ilclient_destroy(client);
	client=NULL;
	//
	
	return 0;	
}

int brcm_alsa_omx_vc03_close()
{
	
	//wait until brcm_omx_vc03_stop() is done
	//TODO: more error robust
	while (client)
		mdelay(5);	
	
	DEBUG("\n %lx: brcm_alsa_omx_vc03_close\n",jiffies);
	return 0;
}

int brcm_alsa_omx_vc03_init()
{

	//  Set up the kernel thread for processing frames
	//
	sema_init(&alsa_omx_sema, 0);

	alsa_omx_task_running = 1;
	alsa_omx_task_id = kthread_run(alsa_omx_task, 0, "alsa_omx_task");

	if(NULL==alsa_omx_task_id || IS_ERR(alsa_omx_task_id))
	{
		DEBUG("brcm_alsa_omx_vc03_init failed.\n");
		return -1;
	}
  	return 0;	
}

int brcm_alsa_omx_vc03_exit()
{
	//stop alsa_omx_task
	alsa_omx_task_running = 0;

	return 0;
	
}

void startPlay()
{	
	alsa_pcm_event |= PCM_EVENT_PLAY_START;	//set Play event
	up(&alsa_omx_sema);
	
	DEBUG("\n %lx:startPlay \n",jiffies);
}

void stopPlay()
{		
	alsa_pcm_event &= ~PCM_EVENT_PLAY_START;	//clear Play event
		
	DEBUG("\n %lx:stopPlay \n",jiffies);
}

void startRecd()
{	

	alsa_pcm_event |= PCM_EVENT_RECD_START;	//set Play event
	up(&alsa_omx_sema);
	
	DEBUG("\n %lx:startRecd \n",jiffies);
}

void stopRecd()
{		
	alsa_pcm_event &= ~PCM_EVENT_RECD_START;	//clear Play event
		
	DEBUG("\n %lx:stopRecd \n",jiffies);
}
