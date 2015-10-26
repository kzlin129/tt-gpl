/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/


#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/file.h>

#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <asm/io.h>
#include <asm/atomic.h>
#include <linux/dma-mapping.h>


#include <linux/broadcom/omx/OMX_Broadcom.h>
#include <linux/broadcom/vc03/host_ilcore.h>
#include <linux/broadcom/vc03/vcilclient.h>
#include <linux/broadcom/vc03/vcilplatform.h>
#include <linux/broadcom/vc03/vcos.h>

#if !defined __KERNEL__
#include <linux/broadcom/vc03/vcomx.h>
#define malloc libomx_malloc
#define free   libomx_free
#endif

//i hate it: it used to be in a header file, now it is gone.
#include <linux/broadcom/vc03/vcilplatform.h>
struct _COMPONENT_T {
   OMX_HANDLETYPE comp;
   int local_comp;
   PLATFORM_SEMAPHORE_T sema;
   PLATFORM_EVENTGROUP_T event;
   OMX_BUFFERHEADERTYPE *out_list;
   OMX_BUFFERHEADERTYPE *in_list;
   char name[32];
   unsigned int error_mask;
   unsigned int private;
   ILEVENT_T *list;
   ILCLIENT_T *client;
};

static char  *my_openfile(char *filepath,int *size)
{
    const int bufsize_max = 160 * 1024;
    int fd, nbytes, flags;
    mm_segment_t old_fs;
    char *buf;
    
    // open the file
    flags = O_RDONLY;
    old_fs = get_fs();
    set_fs(get_ds());
    fd = sys_open(filepath, flags, 0777);
    if(fd < 0)
    {
        set_fs(old_fs);
        return NULL;
    }

    if (( buf = vmalloc(bufsize_max)) == NULL )
        return NULL;

    
    memset(buf, 0, bufsize_max);
    nbytes = sys_read(fd, buf, bufsize_max);

    sys_close(fd);
    set_fs(old_fs);
  
    *size =nbytes;
    return buf;

}

#define INPUT_BUFFER_ARRIVED 1

#define ALSA_OMX_SUBSTREAM_PLAY_NUM (4) //3  //3 playback 
#define ALSA_OMX_SUBSTREAM_RECD_NUM (0) //3  //1 capture
#define ALSA_OMX_SUBSTREAM_NUM (ALSA_OMX_SUBSTREAM_PLAY_NUM+ALSA_OMX_SUBSTREAM_RECD_NUM) //3  

//debug only
int play_done[ALSA_OMX_SUBSTREAM_NUM];

typedef struct 
{

	ILCLIENT_T *client ;
	
	OS_EVENTGROUP_T vc_events[ALSA_OMX_SUBSTREAM_NUM];
	COMPONENT_T *list[ALSA_OMX_SUBSTREAM_NUM];
	struct task_struct* task_id[ALSA_OMX_SUBSTREAM_NUM];
	int alsa_task_running[ALSA_OMX_SUBSTREAM_NUM];

	spinlock_t lock;
	int notify_ap;
} alsa_struct_t;


typedef struct
{
	alsa_struct_t *alsa_struct_p;
	int task_num;
} alsa_single_t;


static void empty_buffer_done_callback(void *data, COMPONENT_T *comp)
{
	int ii;
	alsa_struct_t *play = (alsa_struct_t*)data;
	COMPONENT_T **list;

	list=play->list;
	for (ii=0;ii<ALSA_OMX_SUBSTREAM_NUM;ii++)
	{
		if (list[ii] == comp)
			break;
	}

	vc_assert(ii<ALSA_OMX_SUBSTREAM_NUM);

	os_eventgroup_signal( &play->vc_events[ii], 0 );
}

static int ERROR_RETURN(char *str, int error) 	
{	if (error != OMX_ErrorNone) 
	{ \
		printk(KERN_ERR "\n %s %x\n",str,error); 
		return -1; 
	}

  return 0;
}

#define ALSA_BUFFER_SIZE (32*1024)

COMPONENT_T *create_component_instance(ILCLIENT_T *client, int fs)
{
	OMX_ERRORTYPE error;
	OMX_PARAM_PORTDEFINITIONTYPE param;
	OMX_AUDIO_PARAM_PCMMODETYPE param1;
	COMPONENT_T *st=NULL;

	error = ilclient_create_component(client, &st, "audio_render", 0, 1, 1);
	ERROR_RETURN("ilclient_create_component",error);
	//
	ilclient_disable_port(st,101);	
	//
	
	memset(&param, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
	error = OMX_GetParameter(ILC_GET_HANDLE(st), OMX_IndexParamPortDefinition, &param);
	param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
	param.nVersion.nVersion = OMX_VERSION;
	param.nPortIndex = 100;
	param.nBufferCountActual = 1; 
	param.nBufferSize = ALSA_BUFFER_SIZE;
	param.eDir = OMX_DirInput;
	param.eDomain = OMX_PortDomainAudio;
	param.format.audio.eEncoding = OMX_AUDIO_CodingPCM;

	error = OMX_SetParameter(ILC_GET_HANDLE(st), OMX_IndexParamPortDefinition, &param);
	ERROR_RETURN("OMX_SetParameter",error);
	//
	memset(&param1, 0, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
	error = OMX_GetParameter(ILC_GET_HANDLE(st), OMX_IndexParamAudioPcm, &param1);
	param1.nSize = sizeof(OMX_AUDIO_PARAM_PCMMODETYPE);
	param1.nVersion.nVersion = OMX_VERSION;
	param1.nPortIndex = 100;
	param1.nChannels = 2;
	param1.eEndian = OMX_EndianLittle;
	param1.bInterleaved = OMX_TRUE;
	param1.nBitPerSample = 16; //32
	param1.nSamplingRate = fs;//8000;//44100;
	param1.eNumData = OMX_NumericalDataSigned;
	param1.ePCMMode = OMX_AUDIO_PCMModeLinear;
	param1.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
	param1.eChannelMapping[1] = OMX_AUDIO_ChannelRF;

	error = OMX_SetParameter(ILC_GET_HANDLE(st), OMX_IndexParamAudioPcm, &param1);
	ERROR_RETURN("OMX_SetParameter",error);
	//
	error = OMX_SendCommand(ILC_GET_HANDLE(st), OMX_CommandStateSet, OMX_StateIdle, NULL);
	ERROR_RETURN("OMX_SendCommand",error);

	//
	//for(i=0; i<param.nBufferCountActual; i++) //is one enough?
	{	
		unsigned char *buf = vmalloc(param.nBufferSize);
		error = OMX_UseBuffer(ILC_GET_HANDLE(st), &st->out_list,100, st->out_list, param.nBufferSize, buf);
		ERROR_RETURN("OMX_UseBuffer",error);						
	}
  
	error = ilclient_wait_for_event(st, OMX_EventCmdComplete, OMX_CommandStateSet, 
							0, OMX_StateIdle, 0,
	                       ILCLIENT_STATE_CHANGED, PLATFORM_EVENTGROUP_SUSPEND);
	ERROR_RETURN("ilclient_wait_for_event",error);
  

	//
	{
		error = ilclient_change_component_state(st, OMX_StateExecuting);
		ERROR_RETURN("ilclient_change_component_state",error);
	}

	return st;

}

int del_component_instance(COMPONENT_T *st)
{
	OMX_ERRORTYPE error;
	OMX_BUFFERHEADERTYPE *buffer;
	COMPONENT_T *dummy[2];

	if (!st)
		return 0;
	
	// cleanup
	error = ilclient_change_component_state(st, OMX_StateIdle);
	vc_assert(error == OMX_ErrorNone);

	error = OMX_SendCommand(ILC_GET_HANDLE(st), OMX_CommandStateSet, OMX_StateLoaded, NULL);
	vc_assert(error == OMX_ErrorNone);

	//
	buffer = st->out_list;
     {
		unsigned char *buf = buffer->pBuffer;

		error = OMX_FreeBuffer(ILC_GET_HANDLE(st), 100, buffer);
		vc_assert(error == OMX_ErrorNone);

		free(buf);
     }
     //
     

	ilclient_wait_for_event(st, OMX_EventCmdComplete, OMX_CommandStateSet, 0, OMX_StateLoaded, 0,
	                        ILCLIENT_STATE_CHANGED, PLATFORM_EVENTGROUP_SUSPEND);

	dummy[0] = st;
	dummy[1] = NULL;
	ilclient_cleanup_components(dummy);

	return 0;
}

#define THREAD_RT_PRIORITY  95


static int alsa_task_play(void *data)
{
	struct sched_param param;
 	int rc;	
	
	COMPONENT_T *st;
	ILCLIENT_T *client;

	alsa_single_t *single = (alsa_single_t *) data;
	alsa_struct_t *ptr = single->alsa_struct_p;

	int task_num = single->task_num;
	int task_running = ptr->alsa_task_running[task_num];
	OS_EVENTGROUP_T *vc_events = &ptr->vc_events[task_num];
	int event;
	
	//debug only
	unsigned char *buf;
	int fs;
	int alsa_ticks;  
	int total_byte;
	int bytes;
	char filename[64];
	char *file_buf;
	unsigned long   flags;

#if 0
	param.sched_priority = THREAD_RT_PRIORITY;
	if( ( rc = sched_setscheduler( current, SCHED_FIFO, &param ) ) == 0 )
		printk( KERN_ERR "[%s] pri changed to %d\n", __FUNCTION__, param.sched_priority);
	else
	{
	   printk( KERN_ERR "[%s] failed to set the RT priority %d\n", __FUNCTION__, param.sched_priority );
	   return rc;
	}
#endif

	snprintf( filename, sizeof( filename ), "/mnt/media0/alsa/alsa%d.wav", task_num );
	//read max 160KB wav content to playback
	file_buf = my_openfile(filename,&total_byte);
	if ((file_buf==NULL) || (total_byte==0))
		vc_assert(file_buf!=NULL);

	//read fs from wav header
	buf =  &file_buf[0x18];
	fs  = buf[0] + (buf[1]<<8) + (buf[2]<<16) + (buf[3]<<24);
	vc_assert( fs>=8000 && fs <= 96000);

	printk(KERN_ERR " alsa task running %d\n",task_num);
	client =ptr->client;
     //create component
	st = create_component_instance(client,fs);
	ptr->list[task_num] = st;
	vc_assert(st!=NULL);	

	
	spin_lock_irqsave(&ptr->lock, flags );
	ptr->notify_ap |= (1<<task_num);
	spin_unlock_irqrestore(&ptr->lock, flags );
	
	while (task_running)
	{
		OMX_BUFFERHEADERTYPE *buffer = st->out_list;
		OMX_ERRORTYPE error;
		int size;
		
		os_eventgroup_retrieve( vc_events,&event);
		task_running = ptr->alsa_task_running[task_num];

		if (event == 1)
		{
			//doing nothing: it is my empty buffer done event
		}
		else if (event == 2)
		{
			//i am told to start playing
			alsa_ticks = 0;
			bytes = total_byte-0x2c;
		}
		else if (event == 4)
		{
			//i am told to exit
			task_running = 0;
			continue;
		}
		
		if (bytes <= 0) 
		{
			//sent a signal i am done
			spin_lock_irqsave(&ptr->lock, flags );
			ptr->notify_ap |= (1<<task_num);
			spin_unlock_irqrestore(&ptr->lock, flags );
			
			continue;
		}

		size = bytes>ALSA_BUFFER_SIZE?ALSA_BUFFER_SIZE:bytes;
		buf = buffer->pBuffer;
		buffer->nFilledLen = size;
		buffer->nOffset = 0;
		buffer->pAppPrivate = NULL;

		//memset(buf,0,buffer->nFilledLen);
		memcpy(buf,file_buf+0x2c+alsa_ticks*ALSA_BUFFER_SIZE,size);

		error = OMX_EmptyThisBuffer(ILC_GET_HANDLE(st), buffer);
		vc_assert(error == OMX_ErrorNone);

		bytes -= ALSA_BUFFER_SIZE;
		alsa_ticks++;

	}

	del_component_instance(st);
	free(file_buf);
	
	spin_lock_irqsave(&ptr->lock, flags );
	ptr->notify_ap |= (1<<task_num);
	spin_unlock_irqrestore(&ptr->lock, flags );
	
	printk(KERN_ERR " alsa task stopped %d\n",task_num);

	return 0;
}

static int alsa_task_capture(void *data)
{
	COMPONENT_T *st;
	ILCLIENT_T *client;

	alsa_single_t *single = (alsa_single_t *) data;
	alsa_struct_t *ptr = single->alsa_struct_p;

	int task_num = single->task_num;
	int task_running = ptr->alsa_task_running[task_num];
	OS_EVENTGROUP_T *vc_events = &ptr->vc_events[task_num];
	int event;

	printk(KERN_ERR " alsa task running %d",task_num);
	
	while (task_running)
	{
		os_eventgroup_retrieve( vc_events,&event);

	}

	return 0;
	
}

int vc_do_wav(char *filename)
{
	COMPONENT_T *st;
	ILCLIENT_T *client;
	OMX_BUFFERHEADERTYPE *buffer;
	int size,n;
	OMX_ERRORTYPE error;
	int ii,wav_num;
	int success;
	int event;
	int flags;

	alsa_struct_t *alsa_ptr;
	alsa_single_t alsa_singles[ALSA_OMX_SUBSTREAM_NUM];

	wav_num = 0;
	size = strlen(filename);
	for (ii=0;ii<size;ii++)
	{
		wav_num *= 10;
		wav_num += filename[ii] - '0';
	}

	if (wav_num > ALSA_OMX_SUBSTREAM_PLAY_NUM)
		wav_num = ALSA_OMX_SUBSTREAM_PLAY_NUM;
	
	alsa_ptr = (alsa_struct_t *) malloc(sizeof(alsa_struct_t));
	vc_assert(alsa_ptr);
	spin_lock_init(&alsa_ptr->lock );
	alsa_ptr->notify_ap = 0;

	client = ilclient_init();
	vc_assert(client);
	alsa_ptr->client = client;
	
	ilclient_set_empty_buffer_done_callback(client, empty_buffer_done_callback, alsa_ptr);

	error = OMX_Init();
	vc_assert(error == OMX_ErrorNone);

	for (ii=0;ii<wav_num;ii++)
		play_done[ii] = -1;
	
	for (ii=0;ii<wav_num;ii++)
	{
		char task_name[32];

		snprintf( task_name, sizeof( task_name ), "alsa_playback_%d", ii );
		os_eventgroup_create(&alsa_ptr->vc_events[ii],task_name);
		alsa_ptr->list[ii] = NULL;
		alsa_ptr->alsa_task_running[ii] = 1;

		alsa_singles[ii].task_num = ii;
		alsa_singles[ii].alsa_struct_p = alsa_ptr;
		
		alsa_ptr->task_id[ii] = kthread_run(alsa_task_play, &alsa_singles[ii], task_name);
		vc_assert(alsa_ptr->task_id[ii]);
	}

	for (;ii<wav_num+ALSA_OMX_SUBSTREAM_RECD_NUM;ii++)
	{
		char task_name[32];

		snprintf( task_name, sizeof( task_name ), "alsa_capture_%d", ii-wav_num);
		os_eventgroup_create(&alsa_ptr->vc_events[ii],task_name);
		alsa_ptr->list[ii] = NULL;
		alsa_ptr->alsa_task_running[ii] = 1;

		alsa_singles[ii].task_num = ii;
		alsa_singles[ii].alsa_struct_p = alsa_ptr;
		
		alsa_ptr->task_id[ii] = kthread_run(alsa_task_capture, &alsa_singles[ii], task_name);
		vc_assert(alsa_ptr->task_id[ii]);

	}

	//waiting to be all inited
	while (alsa_ptr->notify_ap != ( (1<<wav_num)-1))
		msleep(10);

	spin_lock_irqsave(&alsa_ptr->lock, flags );
	alsa_ptr->notify_ap = 0;
	spin_unlock_irqrestore(&alsa_ptr->lock, flags );

	//repeat n times
	n=0x8000;
     while (n--)
     	{			
		for (ii=0;ii<wav_num;ii++)
			os_eventgroup_signal(&alsa_ptr->vc_events[ii],1);  //kick start

		//waiting until finishing playing for each task
		while (alsa_ptr->notify_ap != ( (1<<wav_num)-1))
			msleep(10);

		spin_lock_irqsave(&alsa_ptr->lock, flags );
		alsa_ptr->notify_ap = 0;
		spin_unlock_irqrestore(&alsa_ptr->lock, flags );	    
    	}
	  
	for (ii=0;ii<wav_num;ii++)
		os_eventgroup_signal(&alsa_ptr->vc_events[ii],2);  //kick stop
	
	//wait until everybody exits
	while (alsa_ptr->notify_ap != ( (1<<wav_num)-1))
		msleep(10);

	error = OMX_Deinit();
	vc_assert(error == OMX_ErrorNone);

	ilclient_destroy(client);

	free(alsa_ptr);
	return 0;
}

