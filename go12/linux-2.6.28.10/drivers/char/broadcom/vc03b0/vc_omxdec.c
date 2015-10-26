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




/**
*
*  @file    vc_omxdec.c
*
*  @brief   test code for omx decode path
*           Note this test code is developed for 2 ways video, no read_media component will be involved
*
****************************************************************************/
#include <linux/broadcom/omx/OMX_Broadcom.h>
#include <linux/broadcom/vc03/host_ilcore.h>
#include <linux/broadcom/vc03/vcilclient.h>
#include <linux/broadcom/vc03/vcilplatform.h>
#include <linux/broadcom/vc03/vcos.h>

#include <linux/broadcom/knllog.h>

#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/vmalloc.h>
#include <linux/syscalls.h>

#include "vc_omxencdec.h"

#if !defined __KERNEL__
#include <linux/broadcom/vc03/vcomx.h>
#define malloc libomx_malloc
#define free   libomx_free
#endif

#define MAX_DEC_PKT_SIZE   60000 /* maximum size of the packet we can handle */


/******************************************************************************
Static data.
******************************************************************************/
#define INPUT_BUFFER_ARRIVED 1

static int decThreadPid = 0;
struct semaphore pkt_available;
struct semaphore pkt_processed;
struct semaphore pkt_processCanStart;
struct completion decThreadExited;
static VC_DECPKT * pktptr;
static int firstPkt = 1;
static int thread_started = 0 ;

typedef struct 
{
   PLATFORM_EVENTGROUP_T events; /* event for buffer available */
   ILCLIENT_T *client;
   OMX_BUFFERHEADERTYPE *buffer_list;

   OMX_CONFIG_DISPLAYREGIONTYPE region;

   COMPONENT_T *video_decode;
   COMPONENT_T *video_scheduler;
   COMPONENT_T *video_render;

   COMPONENT_T *list[4];

   TUNNEL_T tunnels[2];

} DEC_STATE_T;

static DEC_STATE_T dec_state;

typedef struct 
{
   char * tmp_buf;
   int read_idx;
   int buffer_size;
   int file_handle;
   int bytes_available;

} READFILE_T;

static READFILE_T read_file;

/******************************************************************************
Static functions.
******************************************************************************/
static int dec_processThread( void *unused );
static int vc_create_decpath( void * unused );
int vc_delete_decpath( DEC_STATE_T *dec_state );

static void input_buffer_callback(void *data, COMPONENT_T *comp);

/*---------------------------------------------------------------------------*/
/***************************************************************************/
/**
*  start decoder processing thread.
*
*  @param1
*
*  @return
*
*  @remarks
*  the thread should be called only once
*
*/
void vc_start_decThd( void )
{
   if( thread_started == 0 )
   {
      sema_init( &pkt_available, 0 );
      sema_init( &pkt_processed, 0 );
      sema_init( &pkt_processCanStart, 0 );
      init_completion( &decThreadExited );
      decThreadPid = kernel_thread( dec_processThread, 0, 0 );

      thread_started = 1;
   }
   else
   {
      printk("thread already started!!!!\n");
   }
   return;
}

void vc_start_launch_dec( void )
{
   int threadPid = kernel_thread( vc_create_decpath, 0, 0 );
   firstPkt = 1;
   printk("decpath thread %d\n", threadPid );
}

int vc_decProcessPkt( VC_DECPKT * data )
{
   int result = 0;
   /* packet for the decoder arrive, tell the thread to process it */
   pktptr = data;
   up( &pkt_available );

   if( down_interruptible( &pkt_processed ) != 0 )
   {
      printk( KERN_ERR "vc dec pkt process failed\n");
      result = 1;
   }
   return  result;
}

#define DECTHREAD_RT_PRIORITY 85
static int dec_processThread( void *unused )
{
   int rc;
   uint32_t set;
   struct sched_param sparm;
   OMX_ERRORTYPE error;
   OMX_BUFFERHEADERTYPE *buffer;
   daemonize( "vc_dec_process" );

   sparm.sched_priority = DECTHREAD_RT_PRIORITY;

   if( ( rc = sched_setscheduler( current, SCHED_FIFO, &sparm ) ) < 0 )
   {
      printk( KERN_ERR "failed to set the RT priority %d for dec_processThread\n", sparm.sched_priority );
      return rc;
   }
   while( down_interruptible( &pkt_available ) == 0 )
   {
#if 0
      KNLLOG("pkt size %d: %x %x %x\n", pktptr->data_length, pktptr->flags, pktptr->data[0], 
            pktptr->data[1] );
#endif

      if( firstPkt )
      {
         // first packet, wait for the video_decoder component to be ready
         if( down_interruptible( &pkt_processCanStart ) != 0 )
         {
            printk("decode component ready for the first pkt\n\n");
         }
         firstPkt = 0;
      }
      if( dec_state.buffer_list )
      {
         buffer = dec_state.buffer_list;
         dec_state.buffer_list = dec_state.buffer_list->pAppPrivate;
      }
      else
      {
         while( (buffer = ilclient_get_input_buffer( dec_state.video_decode, 130 )) == NULL )
         {
            platform_eventgroup_get(&dec_state.events, INPUT_BUFFER_ARRIVED, PLATFORM_EVENTGROUP_OPERATION_OR_CONSUME, -1, &set);
         }
      }

      memcpy( buffer->pBuffer, pktptr->data, pktptr->data_length );
      buffer->nFilledLen = pktptr->data_length;
      buffer->nOffset = 0;
      buffer->pAppPrivate = NULL;
      buffer->nFlags = pktptr->flags;
      buffer->nInputPortIndex = 130;

      error = OMX_EmptyThisBuffer( ILC_GET_HANDLE( dec_state.video_decode ), buffer );
      vc_assert( error == OMX_ErrorNone );

      up( &pkt_processed );
   }  
   complete_and_exit( &decThreadExited, 0 );
}

static int vc_create_decpath( void * unused )
{
   OMX_ERRORTYPE error;
   OMX_PARAM_PORTDEFINITIONTYPE param;
   int status = 0;
   int i, n;
   DEC_STATE_T * dec_st = &dec_state;

   memset( dec_st, 0, sizeof( DEC_STATE_T ) );

   /* create OMX decoder path */
   error = OMX_Init();
   vc_assert( error == OMX_ErrorNone );

   dec_st->client = ilclient_init();

   /* setting display region */
   dec_st->region.nVersion.nVersion = OMX_VERSION;
   dec_st->region.nSize = sizeof( dec_st->region );
   dec_st->region.set = OMX_DISPLAY_SET_NUM | OMX_DISPLAY_SET_FULLSCREEN | 
  	                OMX_DISPLAY_SET_TRANSFORM | OMX_DISPLAY_SET_LAYER |
                        OMX_DISPLAY_SET_DEST_RECT | OMX_DISPLAY_SET_MODE | OMX_DISPLAY_SET_NOASPECT;
   dec_st->region.num = 0;
   dec_st->region.nPortIndex = 90;
   dec_st->region.transform = OMX_DISPLAY_ROT0;
   dec_st->region.fullscreen = 0;
   dec_st->region.layer = 2;
   dec_st->region.dest_rect.x_offset = 0;
   dec_st->region.dest_rect.y_offset = 0;
   dec_st->region.dest_rect.width = 800;
   dec_st->region.dest_rect.height = 480;
   dec_st->region.mode = OMX_DISPLAY_MODE_FILL;
   dec_st->region.noaspect = 1;

   /* create the video_decode component */
   ilclient_create_component(dec_st->client, &dec_st->video_decode, "video_decode", 0, 1, 1);
   dec_st->list[0] = dec_st->video_decode;
   ilclient_disable_port( dec_st->video_decode, 131 );

   /* create the video_render component */
   ilclient_create_component( dec_st->client, &dec_st->video_render, "video_render", 0, 1, 0 );
   dec_st->list[1] = dec_st->video_render;
   ilclient_disable_port( dec_st->video_render, 90 );

   /* set up the display region */
   error = OMX_SetConfig( ILC_GET_HANDLE( dec_st->video_render ), OMX_IndexConfigDisplayRegion, &dec_st->region );
   vc_assert( error == OMX_ErrorNone );   
   
   n = platform_eventgroup_create( &dec_st->events );
   vc_assert( n==0 );

   /* set up the callback for empty buffer */
   ilclient_set_empty_buffer_done_callback( dec_st->client, input_buffer_callback, &dec_st->events);

   error = OMX_SendCommand(ILC_GET_HANDLE(dec_st->video_decode), OMX_CommandStateSet, OMX_StateExecuting, NULL);
   vc_assert(error == OMX_ErrorNone);

   error = OMX_SendCommand(ILC_GET_HANDLE(dec_st->video_render), OMX_CommandStateSet, OMX_StateExecuting, NULL);
   vc_assert(error == OMX_ErrorNone);


   memset( &param, 0, sizeof( OMX_PARAM_PORTDEFINITIONTYPE ) );
   param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
   param.nVersion.nVersion = OMX_VERSION;

   /* configure the decoder to run H.264 decoder */

   /* ??? check if this is all u need */
   param.nPortIndex = 130;
   param.eDomain = OMX_PortDomainVideo;
   param.eDir = OMX_DirInput;
   param.format.video.eCompressionFormat = OMX_VIDEO_CodingAVC;
   param.format.video.eColorFormat = 0;

   /* configure the buffer chain to be used */
   param.nBufferCountActual = 2;
   param.nBufferSize = MAX_DEC_PKT_SIZE;

   error = OMX_SetParameter(ILC_GET_HANDLE(dec_st->video_decode), OMX_IndexParamPortDefinition, &param);
   vc_assert(error == OMX_ErrorNone);


   error = OMX_SendCommand(ILC_GET_HANDLE(dec_st->video_decode), OMX_CommandStateSet, OMX_StateIdle, NULL);
   vc_assert(error == OMX_ErrorNone);

   dec_st->buffer_list = NULL;

   /* register the buffers to be used with the video_decode component */
   for (i=0; i<param.nBufferCountActual; i++)
   {
      unsigned char *buf = malloc(param.nBufferSize);
      printk("buf[%d]=0x%08X\n", i, (int)buf);
      error = OMX_UseBuffer(ILC_GET_HANDLE(dec_st->video_decode), &dec_st->buffer_list, 130, 
            dec_st->buffer_list, param.nBufferSize, buf);
   }

   ilclient_wait_for_event(dec_st->video_decode, OMX_EventCmdComplete, OMX_CommandStateSet, 0, OMX_StateIdle, 0,
                           ILCLIENT_STATE_CHANGED, PLATFORM_EVENTGROUP_SUSPEND);


   set_tunnel( &dec_st->tunnels[0], dec_st->video_decode, 131, dec_st->video_render, 90 );

   
   memset( &param, 0, sizeof( OMX_PARAM_PORTDEFINITIONTYPE ) );
   param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
   param.nVersion.nVersion = OMX_VERSION;

   
   /* change the state of the video_decode to be active */
   ilclient_change_component_state(dec_st->video_decode, OMX_StateExecuting);
   /* tell the packet processing thread that we can start sending data packets to the decoder */
   up( &pkt_processCanStart );

   /* port status will be updated after the first packet is processed */
   i = ilclient_wait_for_event(dec_st->video_decode, OMX_EventPortSettingsChanged,
                                    131, 0, -1, 1, ILCLIENT_PARAMETER_CHANGED, PLATFORM_EVENTGROUP_SUSPEND);


#if 0
   {
      printk("setting up ilclient tunnel\n");
      memset( &param, 0, sizeof( OMX_PARAM_PORTDEFINITIONTYPE ) );
      param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
      param.nVersion.nVersion = OMX_VERSION;
      param.nPortIndex = 131;
      param.eDomain = OMX_PortDomainVideo;
      error = OMX_GetParameter(ILC_GET_HANDLE(dec_st->video_decode), OMX_IndexParamPortDefinition, &param);

      printk("/----------------- video_decode output port----------------------/\n");
         printk("nSize : %u\n", (unsigned int)param.nSize );
         printk("nVersion : 0x%x\n", (unsigned int)param.nVersion.nVersion );
         printk("nPortIndex : %u\n", (unsigned int)param.nPortIndex );
         printk("eDir : %d\n", param.eDir );
         printk("nBufferCountActial : %d\n", (int)param.nBufferCountActual );
         printk("nBufferCountMin : %d\n", (int)param.nBufferCountMin );
         printk("nBufferSize : %d\n", (int)param.nBufferSize );
         printk("bEnabled : %d\n", param.bEnabled );
         printk("bPopulated : %d\n", param.bPopulated );
         printk("eDomain : %d\n", param.eDomain );
         if( param.eDomain == OMX_PortDomainVideo )
         {
            printk("VideoDomain!\n");
            printk("cMIMEType: %s\n", param.format.video.cMIMEType );
            printk("pNativeRender: %u\n", (unsigned int)param.format.video.pNativeRender );
            printk("nFrameWidth: %u\n", (unsigned int)param.format.video.nFrameWidth );
            printk("nFrameHeight: %u\n", (unsigned int)param.format.video.nFrameHeight );
            printk("nStride: %u\n", (unsigned int)param.format.video.nStride );
            printk("nStliceHeight: %u\n", (unsigned int)param.format.video.nSliceHeight );
            printk("nBitRate: %u\n", (unsigned int)param.format.video.nBitrate );
            printk("nFramerate: %u\n", (unsigned int)param.format.video.xFramerate );
            printk("bFlagErrorConcealment: %d\n", param.format.video.bFlagErrorConcealment );
            printk("eCompressionFormat: %d\n", param.format.video.eCompressionFormat );
            printk("eColorFormat: %d\n", param.format.video.eColorFormat );
            printk("pNativeWindow: 0x%x\n", (unsigned int)param.format.video.pNativeWindow );
         }
         printk("bBufferContiguous : %d\n", param.bBuffersContiguous );
         printk("nBufferAlignment : %u\n", (unsigned int)param.nBufferAlignment );
      
      printk("/----------------- video_decode output port END----------------------/\n");

#if 0
      memset( &param, 0, sizeof( OMX_PARAM_PORTDEFINITIONTYPE ) );
      param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
      param.nVersion.nVersion = OMX_VERSION;
      param.nPortIndex = 90;
      error = OMX_GetParameter(ILC_GET_HANDLE(dec_st->video_render), OMX_IndexParamPortDefinition, &param);

      printk("/----------------- video_render input port----------------------/");
         printk("nSize : %u\n", (unsigned int)param.nSize );
         printk("nVersion : 0x%x\n", (unsigned int)param.nVersion.nVersion );
         printk("nPortIndex : %u\n", (unsigned int)param.nPortIndex );
         printk("eDir : %d\n", param.eDir );
         printk("nBufferCountActial : %d\n", (int)param.nBufferCountActual );
         printk("nBufferCountMin : %d\n", (int)param.nBufferCountMin );
         printk("nBufferSize : %d\n", (int)param.nBufferSize );
         printk("bEnabled : %d\n", param.bEnabled );
         printk("bPopulated : %d\n", param.bPopulated );
         printk("eDomain : %d\n", param.eDomain );
         if( param.eDomain == OMX_PortDomainVideo )
         {
            printk("VideoDomain!\n");
            printk("cMIMEType: %s\n", param.format.video.cMIMEType );
            printk("pNativeRender: %u\n", (unsigned int)param.format.video.pNativeRender );
            printk("nFrameWidth: %u\n", (unsigned int)param.format.video.nFrameWidth );
            printk("nFrameHeight: %u\n", (unsigned int)param.format.video.nFrameHeight );
            printk("nStride: %u\n", (unsigned int)param.format.video.nStride );
            printk("nStliceHeight: %u\n", (unsigned int)param.format.video.nSliceHeight );
            printk("nBitRate: %u\n", (unsigned int)param.format.video.nBitrate );
            printk("nFramerate: %u\n", (unsigned int)param.format.video.xFramerate );
            printk("bFlagErrorConcealment: %d\n", param.format.video.bFlagErrorConcealment );
            printk("eCompressionFormat: %d\n", param.format.video.eCompressionFormat );
            printk("eColorFormat: %d\n", param.format.video.eColorFormat );
            printk("pNativeWindow: 0x%x\n", (unsigned int)param.format.video.pNativeWindow );
         }
         printk("bBufferContiguous : %d\n", param.bBuffersContiguous );
         printk("nBufferAlignment : %u\n", (unsigned int)param.nBufferAlignment );
      
      printk("/----------------- video_render input port END----------------------/");
#endif
   }

#endif


   /* create the tunnel between components on the VC03 */
   if( ilclient_setup_tunnel( &dec_st->tunnels[0], 0, 0) < 0 )
   {
      printk("failed to setup tunnel from video_decode to video_scheduler\n");
      return -1;
   }


   /* start decoding */
   ilclient_change_component_state( dec_st->video_decode, OMX_StateExecuting );
   ilclient_change_component_state( dec_st->video_render, OMX_StateExecuting );

   /* print out the display setting */
      {
         OMX_CONFIG_DISPLAYREGIONTYPE current_region;
         
         memset( &current_region, 0, sizeof( OMX_CONFIG_DISPLAYREGIONTYPE ) );
         current_region.nSize = sizeof( OMX_CONFIG_DISPLAYREGIONTYPE );
         current_region.nVersion.nVersion = OMX_VERSION;
         current_region.nPortIndex = 90;
         error = OMX_GetConfig( ILC_GET_HANDLE( dec_st->video_render), OMX_IndexConfigDisplayRegion, 
               &current_region);
         vc_assert( error == OMX_ErrorNone );

         printk("--- Display Regsion---\n" );
         printk("port idx = %u\n", (unsigned int)current_region.nPortIndex );
         printk("VC03 size = %u (%d)\n", (unsigned int)current_region.nSize, sizeof( OMX_CONFIG_DISPLAYREGIONTYPE ) );
         printk("display_num = %u\n", (unsigned int)current_region.num );
         printk("fullscreen = %d\n", current_region.fullscreen );
         printk("dest_rect = %d, %d, %d, %d\n", current_region.dest_rect.x_offset, 
               current_region.dest_rect.y_offset, current_region.dest_rect.width, current_region.dest_rect.height);
         printk("src_rect = %d, %d, %d, %d\n", current_region.src_rect.x_offset, 
               current_region.src_rect.y_offset, current_region.src_rect.width, current_region.src_rect.height);
         printk("noaspect = %d\n", current_region.noaspect );
         printk("mode = %d\n", current_region.mode );
         printk("pixel_x = %u, pixel_y = %u\n", (unsigned int)current_region.pixel_x, 
               (unsigned int)current_region.pixel_y );
         printk("layer = %d\n", current_region.layer );
      }

   return status;
}

int vc_delete_decpath( DEC_STATE_T * dec_st )
{
   int status = 0, n;
   OMX_ERRORTYPE error;
   OMX_BUFFERHEADERTYPE *buffer;
   /* take down OMX decoder path */

   /* flush any buffers associated with the ports */
   error = OMX_SendCommand(ILC_GET_HANDLE(dec_st->video_decode), OMX_CommandFlush, -1, NULL);
   vc_assert(error == OMX_ErrorNone);
   error = OMX_SendCommand(ILC_GET_HANDLE(dec_st->video_render), OMX_CommandFlush, -1, NULL);
   vc_assert(error == OMX_ErrorNone);

   /* set the component state to IDLE */
	ilclient_state_transition(dec_st->list,OMX_StateIdle);

   /* disable all the output ports */
   ilclient_disable_port( dec_st->video_decode, 131 );

   /* change the component state to loaded */
   error = OMX_SendCommand(ILC_GET_HANDLE(dec_st->video_render), OMX_CommandStateSet, OMX_StateLoaded, NULL);
   vc_assert(error == OMX_ErrorNone);
   error = OMX_SendCommand(ILC_GET_HANDLE(dec_st->video_decode), OMX_CommandStateSet, OMX_StateLoaded, NULL);
   vc_assert(error == OMX_ErrorNone);

   /* free all the buffers previously used by the video_decode component */
   while((buffer = ilclient_get_input_buffer(dec_st->video_decode, 130)) != NULL)
   {
     unsigned char *buf = buffer->pBuffer;
       
     error = OMX_FreeBuffer(ILC_GET_HANDLE(dec_st->video_decode), 130, buffer);
     vc_assert(error == OMX_ErrorNone);
       
     free(buf);
   }

   ilclient_wait_for_event(dec_st->video_decode, OMX_EventCmdComplete, OMX_CommandStateSet, 0, OMX_StateLoaded, 0,
                           ILCLIENT_STATE_CHANGED, PLATFORM_EVENTGROUP_SUSPEND);
   

   ilclient_teardown_tunnels(dec_st->tunnels);
   
   ilclient_cleanup_components(dec_st->list);

   error = OMX_Deinit();

   n = platform_eventgroup_delete(&dec_st->events);
   vc_assert(n == 0);

   ilclient_destroy(dec_st->client);

   printk("done\n");

   return status;
}

int grep_a_frame( READFILE_T * file_idx, VC_DECPKT * pkt )
{
   char * dst, * buf;
   int nbytes, bytes_read, frame_available = 0;
   mm_segment_t old_fs;

   while( frame_available == 0 )
   {
      if( file_idx->buffer_size - file_idx->bytes_available )
      {
         /* space available, fill up the tmp buffer */
         old_fs = get_fs();
         set_fs( get_ds() );

         nbytes = sys_read( file_idx->file_handle, 
                            &file_idx->tmp_buf[file_idx->bytes_available], 
                            (file_idx->buffer_size-file_idx->bytes_available) );
         set_fs(old_fs);
         if(nbytes == 0)
         {
            printk("file done, exit\n");
            break;
         }
         file_idx->bytes_available += nbytes;
      }
      /* the first 4 bytes containus the Annex B header, we can skip that */
      buf = &file_idx->tmp_buf[4];
      dst = pkt->data;
      bytes_read = 0;

      /* default buffer flags (for normal data packets) */
      pkt->flags = (OMX_BUFFERFLAG_ENDOFFRAME | OMX_BUFFERFLAG_SYNCFRAME | OMX_BUFFERFLAG_STARTTIME);
      if( (buf[0] == 0x27) || (buf[0] == 0x28) )
      {
         /* sps / pps found, mark this for the VC03 */
         pkt->flags = (OMX_BUFFERFLAG_CODECCONFIG | OMX_BUFFERFLAG_ENDOFFRAME);
      }
      /* write the annex B header to the beginning of the bit stream */
      dst[0]=0;
      dst[1]=0;
      dst[2]=0;
      dst[3]=1;
      dst += 4;
      
      /* read the bit stream till the next NAL */
      while( !((buf[0]==0) && (buf[1]==0) && (buf[2]==0) && (buf[3]==1) ) )
      {
         *dst++ = *buf++;
         bytes_read ++;
         if( (bytes_read+4) >= MAX_DEC_PKT_SIZE )
         {
            printk("packet too big, we cannot handle!!!\n");
            vc_assert(0);
         }
      }
      pkt->data_length = bytes_read + 4;
      
      frame_available = 1;
      //printk("frame ready, size %d\n", pkt->data_length);
      file_idx->bytes_available -= (bytes_read + 4);
      memmove( file_idx->tmp_buf, &file_idx->tmp_buf[bytes_read+4], file_idx->bytes_available);
   }

   return frame_available;
}
void vc_do_dectest( int framenum, int offset, const char * path )
{
   uint8_t * buf;
   int fd, idx, flags, frame_there;
   mm_segment_t old_fs;
   VC_DECPKT pkt;

   (void) offset;

   printk("vc_do_dectest : framenum = %d, offset = %d, file = %s\n",
          framenum, offset, path );

   printk("start opening the file\n");
   memset( &read_file, 0, sizeof( READFILE_T ) );


   /* start file readying */
   flags = O_RDONLY;
   old_fs = get_fs();
   set_fs( get_ds() );
   fd = sys_open( path, flags, 0777 );
   if( fd < 0 )
   {
      VC_DEBUG( Trace, "invalid bitstream path %s\n", path );
      set_fs( old_fs );
      return;
   }
   buf = malloc( MAX_DEC_PKT_SIZE  );

   set_fs(old_fs);

   read_file.file_handle = fd;
   read_file.tmp_buf = malloc( MAX_DEC_PKT_SIZE  );
   read_file.buffer_size = MAX_DEC_PKT_SIZE ;

   idx = 0;

   vc_start_launch_dec( );

   while( idx < framenum )
   {
      {
         pkt.data = buf;

         frame_there = grep_a_frame( &read_file, &pkt );
         if( frame_there )
         {
            int nbytes;
            //printk("%d: frame_ava %d, size %d\n", idx, frame_there, pkt.data_length );
            //printk("data to be sent : 0x%2x %2x %2x %2x\n\n", buf[0], buf[1], buf[2], buf[3]);
            nbytes = pkt.data_length;

            if( nbytes & 0x0F )
            {
               memset( &buf[nbytes], 0, 16 );
               nbytes = (nbytes & ~(0x0F)) + 0x10;
               pkt.data_length = nbytes;
            }

#if 1
            if( vc_decProcessPkt( &pkt ) != 0 )
            {
               printk("video pkt process error, return now!!!!!!\n");
               idx = framenum;
            }
#endif
         }
         /* blindly sleep for 30 ms before processing the next frame */
         /* note that the frame rate might not be correct */
         set_current_state(  TASK_INTERRUPTIBLE );
         schedule_timeout( ((3 * HZ)/100) );
      }
      idx++;
   }

   free( buf );
   old_fs = get_fs();
   set_fs( get_ds() );
   sys_close(fd);
   set_fs(old_fs);
   free( read_file.tmp_buf );

   vc_delete_decpath( &dec_state );
}


static void input_buffer_callback(void *data, COMPONENT_T *comp)
{
   platform_eventgroup_set(data, INPUT_BUFFER_ARRIVED, PLATFORM_EVENTGROUP_OPERATION_OR);
}


