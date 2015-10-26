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
*  @file    vc03_frmfwd.c
*
*  @brief   driver code to handle video frames (video data for 2 way video)
*           between OMX driver (VC03) and the host application
*
****************************************************************************/
#include <linux/broadcom/omx/OMX_Broadcom.h>
#include <linux/broadcom/vc03/host_ilcore.h>
#include <linux/broadcom/vc03/vcilclient.h>
#include <linux/broadcom/vc03/vcilplatform.h>
#include <linux/broadcom/vc03/vcos.h>
#include <linux/broadcom/vc03/vc03_frmfwd_defs.h>
#include <linux/kthread.h>

#include <linux/broadcom/knllog.h>

#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/module.h>

/******************************************************************************
Static data.
******************************************************************************/
/* H.264 NAL type definitions */
#define H264_P_SLICE    1
#define H264_I_SLICE    5
#define H264_SEI        6
#define H264_SPS        7
#define H264_PPS        8

/* constants for camera pool configurations */
#define RECORD_IL_MAX_FRAMERATE             30
#define RECORD_IL_IP_WIDTH                1600  /* set to 1600 so low framerate for HD is supported on the ST camera */
#define RECORD_IL_IP_HEIGHT                900
#define RECORD_IL_NUM_HI_RES_POOL_FRAMES     3

/* maximum packet size handled by this application */
#define MAX_PKT_SIZE    60000

static volatile VC03_HOSTCALLBACK_T * vc03_frmfwd_hostg = NULL;

static struct task_struct *gFrameSendTask = NULL;
static struct task_struct *gFrameRecvTask = NULL;
static struct task_struct *gConfCmdTask = NULL;
static  VC03_FrameBufferList_t   gVC_FramesToVCList;
static struct semaphore cmd_available;
static struct semaphore cmd_returnready;
static struct semaphore encpkt_available;
static struct semaphore dec_setTunnel;
static int dec_active = 0;
static int enc_active = 0;
static int seq_num = 0;
static int encNum = 0;
static int first_pkt = 0;
static int noeos = 1;
static int thread_started = 0;
static int decNum = 0;
static unsigned short last_seqnum = 0;
static unsigned char * enc_tmpbuf;
static int tmp_len = 0;

int queueDepth;
int maxQueueDepth = 0;
int pktlost = -1;

int gVcPrioritySend     = 85;
int gVcPriorityRecv     = 80;
int gVcPriorityCmd      = 60;

VC03_VIDEOCONF_CMD * cmd_pending = NULL;

typedef struct 
{
   //PLATFORM_EVENTGROUP_T events; /* event for buffer available */
   ILCLIENT_T *client;
   OMX_BUFFERHEADERTYPE *buffer_list;

   OMX_CONFIG_DISPLAYREGIONTYPE region;

   COMPONENT_T *video_decode;
   COMPONENT_T *video_render;

   COMPONENT_T *list[3];

   TUNNEL_T tunnels[2];

   int status;
   OMX_VIDEO_CODINGTYPE decType;

} DEC_STATE_T;

typedef struct ENC_STATE_T
{
   PLATFORM_EVENTGROUP_T events;

   ILCLIENT_T *client;
   COMPONENT_T *list[5];

   OMX_BUFFERHEADERTYPE *buffer_list;

   OMX_CONFIG_DISPLAYREGIONTYPE region;

   COMPONENT_T *video_render2;
   COMPONENT_T *video_encode;
   COMPONENT_T *camera;
   COMPONENT_T *clock;

   OMX_CONFIG_DISPLAYREGIONTYPE region2;
   TUNNEL_T ca_di_tunnel[2];
   TUNNEL_T ca_ve_tunnel[2];
   
   OMX_S32 scale;
   OMX_U32 camplus_id;
   int camera_num;

   OMX_IMAGE_FOCUSCONTROLTYPE focus_mode;
   OMX_U32 focus_position;
   int capture_width;
   int capture_height;
   int bitrate;
   int framerate;
   OMX_VIDEO_CODINGTYPE encType;

   /* SPS for H.264, or header bytes for MPEG4 */
   unsigned char header_bytes[40];
   int header_size;
   /* pps for H.264 */
   unsigned char pps[20];
   int pps_size;
   unsigned char *hanging;

} ENC_STATE_T;

static DEC_STATE_T dec_state;
static ENC_STATE_T enc_state;

/******************************************************************************
Static functions.
******************************************************************************/
int vc03_framesend_thd( void * unused );
int vc03_framercv_thd( void * unused );
int vc03_confcmd_thd( void * unused );

void InitFrameBufferList( VC03_FrameBufferList_t *fList );
VC03_FrameBuffer_t *InternalDequeueFrameBuffer( VC03_FrameBufferList_t *fList );
void VC03_EnqueueFrameBuffer( VC03_FrameBufferList_t *fList, VC03_FrameBuffer_t *frameBuf );
VC03_FrameBuffer_t *VC03_DequeueFrameBuffer( VC03_FrameBufferList_t *fList );


/*---------------------------------------------------------------------------*/
/***************************************************************************/
/**
* vc03_host_set_cmd_callback
*
*  register the host callback for frame forwarding with the driver.
*
*  @param1
*
*  @return
*
*  @remarks
* 
*/
VC03_HOSTCALLBACK_T *vc03_host_set_cmd_callback(VC03_HOSTCALLBACK_T *callback)
{
   VC03_HOSTCALLBACK_T * result;

   printk("vc03_host_set_cmd_callback being called ~~~~ started = %d\n", thread_started);
   if(thread_started == 0 )
   {
      vc03_startfrmfwd_thds();
      thread_started = 1;
   }
   vc03_frmfwd_hostg = callback;
   result = callback;

   return( result );
}

/****************************************************************************
*
*  callbacks (expected to be called by the encoder path components.
*  This callbacks indicates the status of the pipeline
*
***************************************************************************/
static void output_buffer_callback(void *data, COMPONENT_T *comp)
{
   up( &encpkt_available );

}
static void eos_callback(void *userdata, COMPONENT_T *comp, OMX_U32 data)
{
   /* we have to make sure eos is received before ending the stream */
#if 0
   printk("eos_callback camera? %d encode? %d\n", 
         (comp==enc_state.camera), (comp==enc_state.video_encode) );
#endif
   noeos = 0;
}

/****************************************************************************
*
*  configre_camera
*
*   set up the camera for the encoder pipeline.  Note that the default camera
*   (compile time configurable in the VC03 binary) will be used
*
***************************************************************************/
static int configure_camera(ENC_STATE_T *st)//, OMX_BOOL want_display)
{
   OMX_ERRORTYPE error;

   ilclient_disable_port(st->camera, 70); // disable the preview port
   ilclient_disable_port(st->camera, 71); // disable the capture port
   ilclient_disable_port(st->camera, 72); // disable the time port

   ilclient_disable_port(st->video_render2, 90);

   {
      OMX_PARAM_CAMERAIMAGEPOOLTYPE camera_pools;

      // setup camera pool
      camera_pools.nSize = sizeof(OMX_PARAM_CAMERAIMAGEPOOLTYPE);
      camera_pools.nVersion.nVersion = OMX_VERSION;
      camera_pools.num_hi_res_frames = RECORD_IL_NUM_HI_RES_POOL_FRAMES;
      camera_pools.hi_res_width = st->capture_width;
      camera_pools.hi_res_height = st->capture_height;
      camera_pools.hi_res_type = 0;

      camera_pools.num_low_res_frames = camera_pools.num_hi_res_frames;
      camera_pools.low_res_width = st->capture_width >>1;
      camera_pools.low_res_height = st->capture_height >>1;
      camera_pools.low_res_type = 0;

      camera_pools.num_input_frames = 2;
      camera_pools.input_width = RECORD_IL_IP_WIDTH;
      camera_pools.input_height = RECORD_IL_IP_HEIGHT;
      camera_pools.input_type = 0;

      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexParamCameraImagePool, &camera_pools);
   }

   // setup the output format
   {
      //Only set the viewfinder port settings here - the capture port will be the same,
      //except for the framerate, which we don't know yet.
      OMX_PARAM_PORTDEFINITIONTYPE param;
      memset(&param, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
      param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
      param.nVersion.nVersion = OMX_VERSION;
      param.nPortIndex = 70;
      param.eDir = OMX_DirOutput;
      param.eDomain = OMX_PortDomainVideo;
      param.format.video.eCompressionFormat = OMX_IMAGE_CodingUnused;
      param.format.video.eColorFormat = OMX_COLOR_FormatYUV420PackedPlanar;
      param.format.video.nFrameWidth = st->capture_width;
      param.format.video.nFrameHeight = st->capture_height;
      param.format.video.nStride = (st->capture_width+15)&~15;
      param.format.video.nSliceHeight = 16;
      param.format.video.xFramerate = st->framerate << 16;
      param.nBufferCountActual = 1;

      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexParamPortDefinition, &param);
   }
   {
      OMX_IMAGE_FOCUSCONTROLTYPE focus_mode;
      OMX_U32 focus_position;
      OMX_ERRORTYPE error;
      OMX_IMAGE_CONFIG_FOCUSCONTROLTYPE focus_control;

      focus_mode = st->focus_mode;
      focus_position = st->focus_position;

      memset(&focus_control, 0, sizeof(OMX_IMAGE_CONFIG_FOCUSCONTROLTYPE));
      focus_control.nSize = sizeof(OMX_IMAGE_CONFIG_FOCUSCONTROLTYPE);
      focus_control.nVersion.nVersion = OMX_VERSION;
      focus_control.nPortIndex = OMX_ALL;
      error = OMX_GetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexConfigFocusControl, &focus_control);

      focus_control.eFocusControl = focus_mode;
      focus_control.nFocusStepIndex = focus_position;
      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexConfigFocusControl, &focus_control);
   }

   {
      OMX_PARAM_U32TYPE cameras_present;
      cameras_present.nSize = sizeof(OMX_PARAM_U32TYPE);
      cameras_present.nVersion.nVersion = OMX_VERSION;
      cameras_present.nPortIndex = OMX_ALL;

      error = OMX_GetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexParamCameraDevicesPresent, &cameras_present);

      if (error != OMX_ErrorNone || !(cameras_present.nU32&(1<<st->camera_num)))
         return -1;
   }

   {
      OMX_PARAM_U32TYPE omx_camplus_id;
      omx_camplus_id.nSize = sizeof(OMX_PARAM_U32TYPE);
      omx_camplus_id.nVersion.nVersion = OMX_VERSION;
      omx_camplus_id.nPortIndex = OMX_ALL;
      omx_camplus_id.nU32 = st->camplus_id;

      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexParamCameraCamplusId, &omx_camplus_id);
      if (error != OMX_ErrorNone)
         return -1;
   }

   {
      OMX_PARAM_U32TYPE camera_number;
      camera_number.nSize = sizeof(OMX_PARAM_U32TYPE);
      camera_number.nVersion.nVersion = OMX_VERSION;
      camera_number.nPortIndex = OMX_ALL;
      camera_number.nU32 = st->camera_num;

      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexParamCameraDeviceNumber, &camera_number);
      if (error != OMX_ErrorNone)
         return -1;
   }
#if 0
   /* NOTE : in theory you could run time pick another camera by changing 
    * the isp_tuner parameter */
   {
      OMX_PARAM_CAMERAISPTUNERTYPE isp_tuner;
      isp_tuner.nSize = sizeof(OMX_PARAM_CAMERAISPTUNERTYPE);
      isp_tuner.nVersion.nVersion = OMX_VERSION;
      strcpy((char*)isp_tuner.tuner_name, st->isp_tuner);
      
      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexParamISPTunerName, &isp_tuner);

      if (error != OMX_ErrorNone)
         return -1;
   }
#endif
   {
      COMPONENT_T *list[] = {st->camera, st->video_render2, NULL};

      ilclient_setup_tunnel(st->ca_di_tunnel, 0, PLATFORM_EVENTGROUP_SUSPEND);
      // transition to executing
      ilclient_state_transition(list, OMX_StateIdle);
      // transition to executing
      ilclient_state_transition(list, OMX_StateExecuting);
   }

   if (error == OMX_ErrorNone){
      return 0;
   }
   else{
      printk("Failed to load components\n");
      return -1;
   }
}
/****************************************************************************
*
*  start_encpath
*
*   start the encoder pipeline for 2 way video.  Path related components will be
*   started, tunnels will be setup and we will start the pipeline when everything is setup
*
***************************************************************************/

static int32_t start_encpath(ENC_STATE_T *st)
{
   OMX_PARAM_PORTDEFINITIONTYPE encode_output_param;
   OMX_CONFIG_DISPLAYREGIONTYPE region2;
   OMX_ERRORTYPE error;
   COMPONENT_T **comp;
   int i;

   region2 = st->region2;
   comp = st->list;

   i = platform_eventgroup_create( &st->events );
   vc_assert( i==0 );

   /* start the video_encode component */
   error = ilclient_create_component( st->client, &st->video_encode, "video_encode", 0, 1, 0 );
   vc_assert( error == OMX_ErrorNone );

   *comp++ = st->video_encode;

   ilclient_set_eos_callback(st->client, eos_callback, &st->events);
   ilclient_set_fill_buffer_done_callback(st->client, output_buffer_callback, &st->events);
   //ilclient_set_empty_buffer_done_callback(st->client, input_buffer_callback, &st->events);
   //ilclient_set_port_settings_callback(st->client, on_port_settings_changed, &st->events);

   {
      OMX_PARAM_U32TYPE omx_camplus_id;

      ilclient_create_component(st->client, &st->camera, "camera", 0, 0, 0);

      *comp++ = st->camera;

      //Set camplus ID to zero. Expect it to be set later.
      omx_camplus_id.nSize = sizeof(OMX_PARAM_U32TYPE);
      omx_camplus_id.nVersion.nVersion = OMX_VERSION;
      omx_camplus_id.nPortIndex = OMX_ALL;
      omx_camplus_id.nU32 = 0;

      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexParamCameraCamplusId, &omx_camplus_id);
   }

   ilclient_create_component(st->client, &st->video_render2, "video_render", 0, 1, 0);
   *comp++ = st->video_render2;
   ilclient_disable_port(st->video_render2, 90);

   // setup display region if already set
   if (region2.set)
   {
      error = OMX_SetConfig(ILC_GET_HANDLE(st->video_render2), OMX_IndexConfigDisplayRegion, &region2);
      vc_assert(error == OMX_ErrorNone);
   }

   ilclient_create_component(st->client, &st->clock, "clock", 0, 0, 0);
   *comp++ = st->clock;
   set_tunnel(st->ca_di_tunnel, st->camera, 70, st->video_render2, 90);

   set_tunnel(st->ca_ve_tunnel, st->camera, 71, st->video_encode, 200 );
   
   {
      OMX_PARAM_ILFIFOCONFIG encode_fifo_config;

      ilclient_disable_port(st->video_encode, 201);
      ilclient_disable_port(st->video_encode, 200);

      memset(&encode_output_param, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
      encode_output_param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
      encode_output_param.nVersion.nVersion = OMX_VERSION;
      encode_output_param.nPortIndex = 201;

      error = OMX_GetParameter(ILC_GET_HANDLE(st->video_encode), OMX_IndexParamPortDefinition, &encode_output_param);
      vc_assert(error == OMX_ErrorNone);

      encode_output_param.eDir = OMX_DirOutput;
      encode_output_param.eDomain = OMX_PortDomainVideo;
      encode_output_param.format.video.eCompressionFormat = st->encType;
      encode_output_param.format.video.nBitrate = st->bitrate;
      encode_output_param.nBufferSize = (400*1024);
      encode_output_param.nBufferCountActual = 2;
      error = OMX_SetParameter(ILC_GET_HANDLE(st->video_encode), OMX_IndexParamPortDefinition, &encode_output_param);
      vc_assert(error == OMX_ErrorNone);
            
      //Set the output fifo size
      memset(&encode_fifo_config, 0, sizeof(OMX_PARAM_ILFIFOCONFIG));
      encode_fifo_config.nSize = sizeof(OMX_PARAM_ILFIFOCONFIG);
      encode_fifo_config.nVersion.nVersion = OMX_VERSION;
      encode_fifo_config.nPortIndex = 201;
      encode_fifo_config.nDataSize = 2 * 1<<21;
      encode_fifo_config.nHeaderCount = 300;

      error = OMX_SetParameter(ILC_GET_HANDLE(st->video_encode), OMX_IndexParamILFifoConfig, &encode_fifo_config);
      vc_assert(error == OMX_ErrorNone);
   }
   /* allocate buffers to be used by the encoder */
   {
      OMX_STATETYPE state;
      error = OMX_GetState(ILC_GET_HANDLE(st->video_encode), &state);           //check whether the state is idle or not
      vc_assert(error == OMX_ErrorNone);
      if(state == OMX_StateLoaded)
      {
         ilclient_change_component_state(st->video_encode, OMX_StateWaitForResources);
      }
       
      st->buffer_list = NULL;
      for(i=0;i <encode_output_param.nBufferCountActual; i++)
      {
         unsigned char *buf = malloc( encode_output_param.nBufferSize );
         printk("buf[%d]=0x%08X\n", i, (int)buf);

         error = OMX_UseBuffer(ILC_GET_HANDLE(st->video_encode),
                                 &st->buffer_list, 201, st->buffer_list,
                                 encode_output_param.nBufferSize, buf );                 // supply buffer to vide encoder output port
          vc_assert(error == OMX_ErrorNone);
       }

       ilclient_change_component_state(st->video_encode, OMX_StateLoaded);

       ilclient_change_component_state(st->video_encode, OMX_StateIdle);  
   }

   configure_camera(st);

   // Configure clock
   {
      OMX_PORT_PARAM_TYPE param;
      OMX_TIME_CONFIG_ACTIVEREFCLOCKTYPE refclock;

      int i;

      //disable capture and clock ports on camera component
      ilclient_disable_port(st->camera, 71); // disable the capture port
      ilclient_disable_port(st->camera, 72); // disable the clock port

      // disable all ports apart from the first two

      param.nSize = sizeof(OMX_PORT_PARAM_TYPE);
      param.nVersion.nVersion = OMX_VERSION;
      error = OMX_GetParameter(ILC_GET_HANDLE(st->clock), OMX_IndexParamOtherInit, &param);
      vc_assert(error == OMX_ErrorNone);
      vc_assert(param.nPorts >= 2);

      for (i=param.nStartPortNumber; i<param.nStartPortNumber+param.nPorts; i++)
            ilclient_disable_port(st->clock, i);

      refclock.nSize = sizeof(OMX_TIME_CONFIG_ACTIVEREFCLOCKTYPE);
      refclock.nVersion.nVersion = OMX_VERSION;
      refclock.eClock = OMX_TIME_RefClockVideo;
      error = OMX_SetConfig(ILC_GET_HANDLE(st->clock), OMX_IndexConfigTimeActiveRefClock, &refclock);
      vc_assert(error == OMX_ErrorNone);

      ilclient_change_component_state(st->clock, OMX_StateIdle);

      {
         TUNNEL_T ctunnel;
         // connect up camera to clock
         set_tunnel(&ctunnel, st->clock, param.nStartPortNumber, st->camera, 72);

         error = OMX_SetupTunnel(ILC_GET_HANDLE(st->clock), param.nStartPortNumber, ILC_GET_HANDLE(st->camera), 72);
         vc_assert(error == OMX_ErrorNone);

         ilclient_enable_tunnel(&ctunnel);
      }
      {
         OMX_TIME_CONFIG_CLOCKSTATETYPE cstate;

         memset(&cstate, 0, sizeof(OMX_TIME_CONFIG_CLOCKSTATETYPE));
         cstate.nSize = sizeof(OMX_TIME_CONFIG_CLOCKSTATETYPE);
         cstate.nVersion.nVersion = OMX_VERSION;
         error = OMX_GetConfig(ILC_GET_HANDLE(st->clock), OMX_IndexConfigTimeClockState, &cstate);
         vc_assert(error == OMX_ErrorNone && cstate.eState == OMX_TIME_ClockStateStopped);
      }
   }
   {
      //Only set the viewfinder port settings here - the capture port will be the same,
      //except for the framerate, which we don't know yet.
      OMX_PARAM_PORTDEFINITIONTYPE param;
      memset(&param, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
      param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
      param.nVersion.nVersion = OMX_VERSION;
      param.nPortIndex = 71;
      param.eDir = OMX_DirOutput;
      param.eDomain = OMX_PortDomainVideo;
      param.format.video.eCompressionFormat = OMX_IMAGE_CodingUnused;
      param.format.video.eColorFormat = OMX_COLOR_FormatYUV420PackedPlanar;
      param.format.video.nFrameWidth = st->capture_width;
      param.format.video.nFrameHeight = st->capture_height;
      param.format.video.nStride = (st->capture_width+15)&~15;
      //param.format.video.nSliceHeight = 16;
      param.format.video.nSliceHeight = (st->capture_height+15)&~15;
      param.format.video.xFramerate = st->framerate << 16;
      //param.format.video.xFramerate = 0;
      param.nBufferCountActual = 1;

      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexParamPortDefinition, &param);
      vc_assert(error == OMX_ErrorNone);
   }

   // setup capture port to capture
   {
      OMX_CONFIG_BOOLEANTYPE capturing;
      memset(&capturing, 0, sizeof(OMX_CONFIG_BOOLEANTYPE));
      capturing.nSize = sizeof(OMX_CONFIG_BOOLEANTYPE);
      capturing.nVersion.nVersion = OMX_VERSION;
      capturing.bEnabled = OMX_TRUE;
      //capturing.bEnabled = OMX_FALSE;
      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexConfigCapturing, &capturing);
      vc_assert(error == OMX_ErrorNone);
   }
   i = ilclient_setup_tunnel( st->ca_ve_tunnel, 0, PLATFORM_EVENTGROUP_SUSPEND );

   ilclient_enable_port( st->camera, 71 );
   ilclient_enable_port(st->video_encode, 201); //enable video encode port
   ilclient_change_component_state(st->video_encode, OMX_StateExecuting);
#if 0
   {
      OMX_CONFIG_DISPLAYREGIONTYPE current_region;
       
      memset( &current_region, 0, sizeof( OMX_CONFIG_DISPLAYREGIONTYPE ) );
      current_region.nSize = sizeof( OMX_CONFIG_DISPLAYREGIONTYPE );
      current_region.nVersion.nVersion = OMX_VERSION;
      current_region.nPortIndex = 90;
      error = OMX_GetConfig( ILC_GET_HANDLE( st->video_render2 ), OMX_IndexConfigDisplayRegion, 
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
      printk("layer = %d\n", (int)current_region.layer );
   }
#endif
   {
      OMX_BUFFERHEADERTYPE *buffer;

      buffer = st->buffer_list;
      st->buffer_list = st->buffer_list->pAppPrivate;

      error = OMX_FillThisBuffer(ILC_GET_HANDLE(st->video_encode), buffer);         // signal video encoder to fill this buffer
      vc_assert(error == OMX_ErrorNone);
                  
      buffer = st->buffer_list;
      st->buffer_list = st->buffer_list->pAppPrivate;

      error = OMX_FillThisBuffer(ILC_GET_HANDLE(st->video_encode), buffer);         // signal video encoder to fill this buffer
      vc_assert(error == OMX_ErrorNone);
   }

   //Transition clock to executing, tell it to move to WaitForStartTime
   ilclient_change_component_state(st->clock, OMX_StateExecuting);

   {
      OMX_TIME_CONFIG_CLOCKSTATETYPE cstate;

      memset(&cstate, 0, sizeof(OMX_TIME_CONFIG_CLOCKSTATETYPE));
      cstate.nSize = sizeof(OMX_TIME_CONFIG_CLOCKSTATETYPE);
      cstate.nVersion.nVersion = OMX_VERSION;
      cstate.eState = OMX_TIME_ClockStateRunning;
      cstate.nStartTime = 0;
      cstate.nOffset = 0;
      cstate.nWaitMask = OMX_CLOCKPORT1;
      error = OMX_SetConfig(ILC_GET_HANDLE(st->clock), OMX_IndexConfigTimeClockState, &cstate);
      vc_assert(error == OMX_ErrorNone);
   }
   printk("start video encoding... \n");
   return 0;
}
/****************************************************************************
*
*  stop_encpath
*
*   stop the encoder pipeline for 2 way video.  Path related components will be
*   stopped, tunnels will be teardown, followed by a clean up of the related components
*
***************************************************************************/

static int32_t stop_encpath(ENC_STATE_T *st)
{
   OMX_ERRORTYPE error;
   OMX_BUFFERHEADERTYPE *buffer;
   OMX_TIME_CONFIG_CLOCKSTATETYPE cstate;
   OMX_PORT_PARAM_TYPE param;
   COMPONENT_T *vlist[] = {st->camera, st->clock, st->video_encode,NULL};
   COMPONENT_T *vlist2[] = {st->camera, st->clock,NULL};

  	printk("terminate encoder path\n");
  	//KNLLOG("terminate playing\n");

   {
      memset( &param, 0, sizeof( param ) );
      param.nSize = sizeof(OMX_PORT_PARAM_TYPE);
      param.nVersion.nVersion = OMX_VERSION;
      error = OMX_GetParameter(ILC_GET_HANDLE(st->clock), OMX_IndexParamOtherInit, &param);
      vc_assert(error == OMX_ErrorNone);

      memset(&cstate, 0, sizeof(OMX_TIME_CONFIG_CLOCKSTATETYPE));
      cstate.nSize = sizeof(OMX_TIME_CONFIG_CLOCKSTATETYPE);
      cstate.nVersion.nVersion = OMX_VERSION;
      cstate.eState = OMX_TIME_ClockStateStopped;
      cstate.nStartTime = 0;
      cstate.nOffset = 0;
      cstate.nWaitMask = 0;
      error = OMX_SetConfig(ILC_GET_HANDLE(st->clock), OMX_IndexConfigTimeClockState, &cstate);
      vc_assert(error == OMX_ErrorNone);
   }
#if 1
   while( noeos )
   {
      set_current_state(  TASK_INTERRUPTIBLE );
      schedule_timeout(10);
   }
   //printk("eos should have arrived\n");
#endif

   ilclient_change_component_state(st->video_render2, OMX_StateIdle);
   ilclient_change_component_state(st->video_render2, OMX_StateLoaded);

   ilclient_state_transition(vlist, OMX_StateIdle);
   ilclient_state_transition(vlist2, OMX_StateLoaded);

   error = OMX_SendCommand(ILC_GET_HANDLE(st->video_encode), OMX_CommandStateSet, OMX_StateLoaded, NULL);
   vc_assert(error == OMX_ErrorNone);   
   
#if 1
   while ( (buffer=ilclient_get_output_buffer(st->video_encode, 201)) != NULL )
   {
      unsigned char * buf = buffer->pBuffer;

      //KNLLOG("get buffer 0x%x\n", (unsigned int)buffer );

      error = OMX_FreeBuffer(ILC_GET_HANDLE(st->video_encode), 201, buffer);
      vc_assert(error == OMX_ErrorNone);
      //KNLLOG("freebuffer\n");
      
      free( buf );
   }
#endif

   //KNLLOG("now waiting for event change\n");
   ilclient_wait_for_event(st->video_encode, OMX_EventCmdComplete, OMX_CommandStateSet, 0, OMX_StateLoaded, 0,
                           ILCLIENT_STATE_CHANGED, PLATFORM_EVENTGROUP_SUSPEND);
   //KNLLOG("done waiting for event change\n");

   // teardown tunnels
   ilclient_teardown_tunnels(st->ca_di_tunnel);
   ilclient_teardown_tunnels(st->ca_ve_tunnel);
#if 1
   {// Destroy clock tunnel
      error = OMX_SetupTunnel(ILC_GET_HANDLE(st->clock), param.nStartPortNumber, NULL, 0);
      vc_assert(error == OMX_ErrorNone);

      error = OMX_SetupTunnel(ILC_GET_HANDLE(st->camera), 72, NULL, 0);
      vc_assert(error == OMX_ErrorNone);
   }
#endif
   ilclient_return_events(st->camera);


	// delete components
	ilclient_cleanup_components(st->list);

   if( st->hanging )
   {
      free( st->hanging );
      st->hanging = NULL;
   }

   return 0;
}

/****************************************************************************
*
*  start_decpath
*
*   start the decoder pipeline for 2 way video.  Path related components will be
*   started, tunnels will be setup and we will start the pipeline when everything is setup
*
*   Note that the decoder path setup is composed of 2 parts, the first part involve setting up
*   the related components
*
*   After the components are being setup, the decoder awaits the first few packets from the host
*
*   The decoder will respond after the first few packets have been received and knowing the decoder
*   characteristics.  Tunneling between components CANNOT continue until this stage is reached.
*   Second part of the decoder path setup involves setting up the related tunnel between components
*
***************************************************************************/
static int32_t start_decpath_components(DEC_STATE_T *dec_st)
{
   OMX_ERRORTYPE error;
   OMX_PARAM_PORTDEFINITIONTYPE param;
   int i;

   /* create the video_decode component */
   ilclient_create_component(dec_st->client, &dec_st->video_decode, "video_decode", 0, 1, 1);
   dec_st->list[0] = dec_st->video_decode;
   ilclient_disable_port( dec_st->video_decode, 131 );

   /* create the video_render component */
   ilclient_create_component( dec_st->client, &dec_st->video_render, "video_render", 0, 1, 0 );
   dec_st->list[1] = dec_st->video_render;
   ilclient_disable_port( dec_st->video_render, 90 );
   
#if 0
   n = platform_eventgroup_create( &dec_st->events );
   vc_assert( n==0 );
   /* set up the callback for empty buffer */
   /* disable because turns out we do not have to do so, we can wait..... */
   ilclient_set_empty_buffer_done_callback( dec_st->client, input_buffer_callback, &dec_st->events);
#endif
   
   error = OMX_SendCommand(ILC_GET_HANDLE(dec_st->video_decode), OMX_CommandStateSet, OMX_StateExecuting, NULL);
   vc_assert(error == OMX_ErrorNone);

   error = OMX_SendCommand(ILC_GET_HANDLE(dec_st->video_render), OMX_CommandStateSet, OMX_StateExecuting, NULL);
   vc_assert(error == OMX_ErrorNone);

   memset( &param, 0, sizeof( OMX_PARAM_PORTDEFINITIONTYPE ) );
   param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
   param.nVersion.nVersion = OMX_VERSION;

   /* configure the decoder to run H.264 decoder */
   param.nPortIndex = 130;
   param.eDomain = OMX_PortDomainVideo;
   param.eDir = OMX_DirInput;
   param.format.video.eCompressionFormat = dec_st->decType;
   param.format.video.eColorFormat = 0;

   /* configure the buffer chain to be used */
   param.nBufferCountActual = 4;
   param.nBufferSize = MAX_PKT_SIZE;

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

   /* set up the display region */
   /* note display region is being setup in separate frmfwd cmd */
   error = OMX_SetConfig( ILC_GET_HANDLE( dec_st->video_render ), OMX_IndexConfigDisplayRegion, &dec_st->region );
   vc_assert( error == OMX_ErrorNone );
   
   /* change the state of the video_decode to be active */
   ilclient_change_component_state(dec_st->video_decode, OMX_StateExecuting);
  
   return 0;
}

static int32_t start_decpath_tunnel( DEC_STATE_T *dec_st)
{
   int i;
   OMX_ERRORTYPE error;
   OMX_PARAM_PORTDEFINITIONTYPE param;

   /* have to continue on waiting to finish the second half of the setup process */
   //printk("start to wait\n");
   if( down_interruptible( &dec_setTunnel ) != 0 )
   {
      printk("dec set tunnel failed\n" );
      vc_assert(0);
   }
   //printk("done waiting\n");

   /* port status will be updated after the first few packet is processed */
   i = ilclient_wait_for_event(dec_st->video_decode, OMX_EventPortSettingsChanged,
                               131, 0, -1, 1, ILCLIENT_PARAMETER_CHANGED, PLATFORM_EVENTGROUP_SUSPEND);
   //printk("number of packets queued now = %d, num sent %d\n", queueDepth, decNum );
   {
      memset( &param, 0, sizeof( OMX_PARAM_PORTDEFINITIONTYPE ) );
      param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
      param.nVersion.nVersion = OMX_VERSION;
      param.nPortIndex = 131;
      param.eDomain = OMX_PortDomainVideo;
      error = OMX_GetParameter(ILC_GET_HANDLE(dec_st->video_decode), OMX_IndexParamPortDefinition, &param);

      printk("/----------------- video_decode output port----------------------/\n");
      printk("nPortIndex : %u\n", (unsigned int)param.nPortIndex );
      printk("eDir : %d\n", param.eDir );
      printk("nBufferCountActial : %d\n", (int)param.nBufferCountActual );
      printk("nBufferSize : %d\n", (int)param.nBufferSize );

      if( param.eDomain == OMX_PortDomainVideo )
      {
         printk("VideoDomain!\n");
         printk("nFrameWidth: %u\n", (unsigned int)param.format.video.nFrameWidth );
         printk("nFrameHeight: %u\n", (unsigned int)param.format.video.nFrameHeight );
         printk("nStride: %u\n", (unsigned int)param.format.video.nStride );
         printk("nStliceHeight: %u\n", (unsigned int)param.format.video.nSliceHeight );
      }
      printk("/----------------- video_decode output port END----------------------/\n");
   }

   /* ports are ready, create the tunnel between components on the VC03 */
   if( ilclient_setup_tunnel( &dec_st->tunnels[0], 0, 0) < 0 )
   {
      printk("failed to setup tunnel from video_decode to video_scheduler\n");
      vc_assert(0);
   }
   /* start decoding */
   ilclient_change_component_state( dec_st->video_decode, OMX_StateExecuting );
   ilclient_change_component_state( dec_st->video_render, OMX_StateExecuting );
   {
      OMX_CONFIG_DISPLAYREGIONTYPE current_region;

      memset( &current_region, 0, sizeof( OMX_CONFIG_DISPLAYREGIONTYPE ) );
      current_region.nSize = sizeof( OMX_CONFIG_DISPLAYREGIONTYPE );
      current_region.nVersion.nVersion = OMX_VERSION;
      current_region.nPortIndex = 90;
      error = OMX_GetConfig( ILC_GET_HANDLE( dec_st->video_render), OMX_IndexConfigDisplayRegion, 
                             &current_region);
      vc_assert( error == OMX_ErrorNone );
#if 0
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
      printk("layer = %d\n", (int)current_region.layer );
#endif
   }   
   return 0;
}

/****************************************************************************
*
*  stop_decpath
*
*   stop the decoder pipeline for 2 way video.  Path related components will be
*   stopped, tunnels will be teardown, followed by a clean up of the related components
*
***************************************************************************/
static int32_t stop_decpath(DEC_STATE_T *dec_st)
{
   OMX_BUFFERHEADERTYPE *buffer; 
   OMX_ERRORTYPE error;

   printk("decoder take down received, number of packets decoded = %d\n", decNum);

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
   /* wait for the state change to complete */
   ilclient_wait_for_event(dec_st->video_decode, OMX_EventCmdComplete, OMX_CommandStateSet, 0, OMX_StateLoaded, 0,
                           ILCLIENT_STATE_CHANGED, PLATFORM_EVENTGROUP_SUSPEND);
   

   ilclient_teardown_tunnels(dec_st->tunnels);   
   ilclient_cleanup_components(dec_st->list);
   
   return 0;
}

/****************************************************************************
*
*  VC03_EnqueueFrameBuffer
*
*   Adds a buffer to the queue.
*
***************************************************************************/
void VC03_EnqueueFrameBuffer( VC03_FrameBufferList_t *fList, VC03_FrameBuffer_t *frameBuf )
{
    //VC_DEBUG( Trace, "frameBuf: 0x%08x\n", (uint32_t)frameBuf );

    // Add the frame to the end of the list
    {
        unsigned long   flags;

        spin_lock_irqsave( &fList->lock, flags );

        list_add_tail( &frameBuf->hdr.node, &fList->list );

        spin_unlock_irqrestore( &fList->lock, flags );
    }

    // Wakeup anybody waiting for it.

    up( &fList->availSem );

} // VC03_EnqueueFrameBuffer

/****************************************************************************
*
*  VC_DequeueFrameBuffer
*
*   Removes a buffer from the queue. If the queue is empty, then it will
*   block.
*
***************************************************************************/
VC03_FrameBuffer_t *VC03_DequeueFrameBuffer( VC03_FrameBufferList_t *fList )
{
    // First of wait for a buffer to be available

    if ( down_interruptible( &fList->availSem ) != 0 )
    {
        return NULL;
    }

    return InternalDequeueFrameBuffer( fList );

} // VC_DequeueFrameBuffer


/****************************************************************************
*
*  InitFrameBufferList
*
*   Initialize a list of frame buffers.
*
***************************************************************************/
void InitFrameBufferList( VC03_FrameBufferList_t *fList )
{
    INIT_LIST_HEAD( &fList->list );

    spin_lock_init(  &fList->lock );
    sema_init( &fList->availSem, 0 );

} // InitFrameBufferList

/****************************************************************************
*
*  InternalDequeueFrameBuffer
*
*   Removes a buffer from the queue. This function assumes that the caller
*   has already downed the semaphore.
*
***************************************************************************/
VC03_FrameBuffer_t *InternalDequeueFrameBuffer( VC03_FrameBufferList_t *fList )
{
    VC03_FrameBuffer_t    *frameBuf;
    VC03_FrameBufferHdr_t    *frameBufHdr;

    {
        unsigned long   flags;

        spin_lock_irqsave( &fList->lock, flags );

        if ( list_empty( &fList->list ))
        {
            frameBuf = NULL;
        }
        else
        {
            struct list_head *deqNode = fList->list.next;
            list_del_init( deqNode );

            frameBufHdr = list_entry( deqNode, VC03_FrameBufferHdr_t, node );
            frameBuf = list_entry( frameBufHdr, VC03_FrameBuffer_t, hdr );
        }
        spin_unlock_irqrestore( &fList->lock, flags );
    }

    //VC_DEBUG( Trace, "returning frameBuf: 0x%08x\n", (uint32_t)frameBuf );

    return frameBuf;

} // InternalDequeueFrameBuffer


/***************************************************************************/
/**
* vc03_startfrmfwd_thds
*
*
*  Call this function to start off the packet processing threads
*
*  @param1
*
*  @return
*
*  @remarks
* 
*/
void vc03_startfrmfwd_thds( void )
{
   if (( gFrameSendTask == NULL ) || IS_ERR( gFrameSendTask ))
   {
      InitFrameBufferList( &gVC_FramesToVCList );
      gFrameSendTask = kthread_run( vc03_framesend_thd, NULL, "vcSend" );
      sema_init( &dec_setTunnel, 0 );
      if ( IS_ERR( gFrameSendTask ))
      {
         printk( KERN_ERR "Init: failed to start FrameSendThread: %ld\n", PTR_ERR( gFrameSendTask ));
         return;
      }
   }
   if (( gConfCmdTask == NULL ) || IS_ERR( gConfCmdTask ))
   {
      sema_init( &cmd_available, 0 );
      sema_init( &cmd_returnready, 0 );

      gConfCmdTask = kthread_run( vc03_confcmd_thd, NULL, "vcConfCmd" );
      if( IS_ERR( gConfCmdTask ) )
      {
         printk( KERN_ERR "Init: failed to start command thread: %ld\n", PTR_ERR( gConfCmdTask ));
         return;
      }
   }
   if (( gFrameRecvTask == NULL ) || IS_ERR( gFrameRecvTask ))
   {
      sema_init( &encpkt_available, 0 );
      gFrameRecvTask = kthread_run( vc03_framercv_thd, NULL, "vcRecv" );
      if( IS_ERR( gFrameRecvTask ))
      {
         printk( KERN_ERR "Init : failed to start FrameRecvThread: %ld\n", PTR_ERR( gFrameRecvTask ));
         return;
      }
   }
}

/***************************************************************************/
/**
* vc03_framesend_thd
*
*  This thread is responsible for processing the packets from the host to the VC03
*  (through OMX)
*
*  @param1
*
*  @return
*
*  @remarks
* 
*/
uint16_t last_seq_num = 0;
int vc03_framesend_thd( void * unused )
{
   int rc;
   VC03_FrameBuffer_t   *frameBuf;
   struct sched_param sparm;
   OMX_ERRORTYPE error;
   OMX_BUFFERHEADERTYPE *buffer;
   unsigned char * ptr;
   unsigned char nalType;
   int followed_I = 0;
#if 1
   sparm.sched_priority = gVcPrioritySend;
   if( ( rc = sched_setscheduler( current, SCHED_FIFO, &sparm ) ) < 0 )
   {
      printk( KERN_ERR "failed to set the RT priority %d for dec_processThread\n", sparm.sched_priority );
      return rc;
   }
#endif
   printk( KERN_INFO "*******************FRAMESENDTHREAD: STARTING*******************\n" );
   while ( (frameBuf = VC03_DequeueFrameBuffer( &gVC_FramesToVCList )) != NULL  )
   {
      //vc_adjust_thread_priority( &gVcPrioritySend );
      if( vc03_frmfwd_hostg == NULL )
      {
         printk( KERN_ERR "vc03_sendframe : cannot send the frame; host callback not registered\n");
         return -1;
      }
      else if( dec_active >= 1 )
      {
         buffer = NULL;
         if( dec_state.buffer_list )
         {
            buffer = dec_state.buffer_list;
            dec_state.buffer_list = dec_state.buffer_list->pAppPrivate;
         }
         else
         {
            int k=0;
            while( (buffer = ilclient_get_input_buffer( dec_state.video_decode, 130 )) == NULL )
            {
#if 0
               platform_eventgroup_get(&dec_state.events, INPUT_BUFFER_ARRIVED, PLATFORM_EVENTGROUP_OPERATION_OR_CONSUME, -1, &set);
#endif
               //KNLLOG("no buffer\n");
               /* wait for 10 ms and try to get a buffer again */
               //mdelay(10);
               set_current_state(  TASK_INTERRUPTIBLE );
               schedule_timeout( (HZ/100));
               k++;
               if( (k%1000) == 0 )
               {
                  printk("input buffer doesn't come back\n");
                  break;
               }
            }
         }
         if( buffer )
         {
            ptr = buffer->pBuffer;
#if 0
            /******** H264 specific code *****************/
            /* add the annex B header to the bitstream */
            ptr[0] = 0;
            ptr[1] = 0;
            ptr[2] = 0;
            ptr[3] = 1;
#endif
            /* copy the whole payload */
            memcpy( ptr, frameBuf->data, frameBuf->hdr.frame.data_len );
            /* determine the NAL type */
            /* we currently only support 1 frame per packet */
            buffer->nFlags = 0;
            if( frameBuf->hdr.frame.flags )
            {
               buffer->nFlags = (OMX_BUFFERFLAG_ENDOFFRAME);
            }
#if 0
            else
            {
               printk("not end of frame\n");
            }
#endif
            if( dec_state.decType == OMX_VIDEO_CodingAVC )
            {
               if( (ptr[0] == 0) && (ptr[1] == 0) && (ptr[2] == 0) && (ptr[3] == 1) )
               {
                  nalType = ( ptr[4] & 0x0F );
                  if( nalType == H264_SPS )
                  {
                     //printk("SPS\n");
                     buffer->nFlags = (OMX_BUFFERFLAG_CODECCONFIG | OMX_BUFFERFLAG_ENDOFFRAME);
                     followed_I = 1;
                     dec_state.status |= 0x01;
                  }
                  else if( nalType == H264_PPS )
                  {
                     //printk("PPS \n");
                     buffer->nFlags = (OMX_BUFFERFLAG_CODECCONFIG | OMX_BUFFERFLAG_ENDOFFRAME);
                     followed_I = 1;        
                     dec_state.status |= 0x02;
                  }
                  else if( nalType == H264_I_SLICE )
                  {
                     //printk("I slice\n");
                     buffer->nFlags |= (OMX_BUFFERFLAG_SYNCFRAME );
                  }
                  else if( followed_I == 1 )
                  {
                     buffer->nFlags |= ( OMX_BUFFERFLAG_SYNCFRAME );
                  }
                  followed_I = 0;
               }
            }
            else
            {
               dec_state.status = 0x03;
            }
            if( dec_state.decType == OMX_VIDEO_CodingMPEG4 )
            {
               /* this is a special case, no way to differentiate the header bytes from the bitstream */
               if( (buffer->nFlags & OMX_BUFFERFLAG_ENDOFFRAME) == 0 )
               {
                  buffer->nFlags = (OMX_BUFFERFLAG_CODECCONFIG | OMX_BUFFERFLAG_ENDOFFRAME);
               }
            }

            if( (buffer->nFlags & OMX_BUFFERFLAG_ENDOFFRAME) == 0 )
            {
               printk("not end of frame\n");
            }
            if( (dec_active == 1) && (dec_state.status == 0x03) )
            {
               //printk("second part can go\n");
               dec_active = 2;
               up( &dec_setTunnel );
            }
            /* patch the length to 16 bytes */
            //frameBuf->hdr.frame.data_len += 4;
            buffer->nFilledLen = frameBuf->hdr.frame.data_len;
            if( frameBuf->hdr.frame.data_len & 0x0F )
            {
               buffer->nFilledLen = (frameBuf->hdr.frame.data_len & ~(0x0F)) + 0x10;
               memset( &ptr[frameBuf->hdr.frame.data_len], 0, 
                     (buffer->nFilledLen-frameBuf->hdr.frame.data_len) );
            }
            buffer->nInputPortIndex = 130;
            buffer->nOffset = 0;
            buffer->pAppPrivate = NULL;
            buffer->nTimeStamp = 0; /* we are not using the timestamp on the VC03 yet */
            if( pktlost < 0 )
            {
               pktlost = 0;
               //last_seqnum = frameBuf->hdr.frame.seq_num;
            }
            else
            {
               last_seqnum++;

               if( last_seqnum != frameBuf->hdr.frame.seq_num )
               {
                  printk("lost : last_seqnum %u curr %u\n", last_seqnum, frameBuf->hdr.frame.seq_num );
                  pktlost++;
               }
            }
            last_seqnum = frameBuf->hdr.frame.seq_num;
#if 0 
            KNLLOG("len = %d addr 0x%x TS %u\(%d)n", frameBuf->hdr.frame.data_len, (unsigned int)buffer, 
                  frameBuf->hdr.frame.timestamp, frameBuf->hdr.frame.seq_num );
#endif
            error = OMX_EmptyThisBuffer( ILC_GET_HANDLE( dec_state.video_decode ), buffer );
            vc_assert( error == OMX_ErrorNone );
            decNum++;
         }
         else
         {
            printk("drop 1 pkt\n");
         }
      }      

      vc03_frmfwd_hostg->vc_free_framebuf( frameBuf );
   }
   return 1;
}

int vc03_framercv_thd( void * unused )
{
   //struct sched_param sparm;
   //int rc;
   OMX_ERRORTYPE error;
   OMX_BUFFERHEADERTYPE *buffer;

   COMPONENT_T *comp;
   VC03_FrameBuffer_t *frameBuf;
   unsigned char * ptr;
   unsigned int * ptr_32;
   //int data_len;
   unsigned int timestamp;
   unsigned int last_VC03timestamp;
   unsigned int last_timestamp;
   unsigned int VC03_timestamp;
#if 0
   sparm.sched_priority = gVcPriorityRecv;
   if( ( rc = sched_setscheduler( current, SCHED_FIFO, &sparm ) ) < 0 )
   {
      printk( KERN_ERR "failed to set the RT priority %d for dec_processThread\n", sparm.sched_priority );
      return rc;
   }
#endif
   printk( KERN_INFO "*******************FRAMERECVTHREAD: STARTING*******************\n" );

   while( down_interruptible( &encpkt_available ) == 0 )
   {
      comp = enc_state.video_encode;
      if( enc_active == 1 && ( vc03_frmfwd_hostg != NULL ) )
      {
         buffer = ilclient_get_output_buffer( comp, 201 );
         if( buffer )
         {
            ptr = (unsigned char *)buffer->pBuffer;

            if( buffer->nFlags & OMX_BUFFERFLAG_CODECCONFIG )
            {
               int config_len = buffer->nFilledLen;
               printk("config param: total len %d\n", config_len);

               if( (ptr[4] == 0x28) && (enc_state.encType == OMX_VIDEO_CodingAVC) )
               {
                  vc_assert( config_len <= 20 );
                  memcpy( enc_state.pps, ptr, config_len );
                  enc_state.pps_size = config_len;
               }
               else
               {
                  /* save the SPS */
                  vc_assert( config_len <= 40 );
                  memcpy( enc_state.header_bytes, ptr, config_len );
                  enc_state.header_size = config_len;
               }
            }
            else
            {
               if( first_pkt )
               {
                  last_timestamp = 0;
                  last_VC03timestamp = 0;
                  first_pkt = 0;
               }
               ptr_32 = (unsigned int *)(&ptr[4]);

               /* convert VC03 buffer timestamp from microsecond to 90 kHz clock */
               VC03_timestamp = (unsigned int)(buffer->nTimeStamp);
               timestamp = last_timestamp + 
                          ( ( (VC03_timestamp - last_VC03timestamp) / 1000 ) * 90 );

               last_timestamp = timestamp;
               last_VC03timestamp = VC03_timestamp;

               if( (buffer->nFlags & OMX_BUFFERFLAG_SYNCFRAME) &&
                   ((enc_state.encType == OMX_VIDEO_CodingAVC) || (enc_state.encType == OMX_VIDEO_CodingMPEG4)) )
               {
                  /* send out SPS */
                  frameBuf = vc03_frmfwd_hostg->vc_get_framebuf( 0, 
                              enc_state.header_size+sizeof(VC03_FrameBufferHdr_t) );
                  vc_assert( frameBuf != NULL );
                  frameBuf->hdr.frame.timestamp = timestamp;
                  frameBuf->hdr.frame.data_len = enc_state.header_size;
                  frameBuf->hdr.frame.seq_num = seq_num++;
                  memcpy( frameBuf->data, enc_state.header_bytes, enc_state.header_size );

                  vc03_frmfwd_hostg->vc_enqueue_framebuf( frameBuf, enc_state.header_size+sizeof(VC03_FrameBufferHdr_t), 0 );
               
                  if( enc_state.encType == OMX_VIDEO_CodingAVC )
                  {
                     /* send out PPS */
                     frameBuf = vc03_frmfwd_hostg->vc_get_framebuf( 0, 
                                 enc_state.pps_size+sizeof(VC03_FrameBufferHdr_t) );
                     vc_assert( frameBuf != NULL );
                     frameBuf->hdr.frame.timestamp = timestamp;
                     frameBuf->hdr.frame.data_len = enc_state.pps_size;
                     frameBuf->hdr.frame.seq_num = seq_num++;
                     memcpy( frameBuf->data, enc_state.pps, enc_state.pps_size );

                     vc03_frmfwd_hostg->vc_enqueue_framebuf( frameBuf, enc_state.pps_size+sizeof(VC03_FrameBufferHdr_t), 0 );
                     //KNLLOG("send SPS PPS 0x%x\n", timestamp);
                  }
               }
               if( buffer->nFlags & OMX_BUFFERFLAG_ENDOFFRAME )
               {
                  unsigned char * dest_ptr;

                  int pkt_len = buffer->nFilledLen + tmp_len;

                  frameBuf = vc03_frmfwd_hostg->vc_get_framebuf( 0, pkt_len+sizeof(VC03_FrameBufferHdr_t) );
                  vc_assert( frameBuf != NULL );
                  frameBuf->hdr.frame.timestamp = timestamp;
                  frameBuf->hdr.frame.data_len = pkt_len;
                  frameBuf->hdr.frame.seq_num = seq_num++;
                  if( buffer->nFlags & OMX_BUFFERFLAG_ENDOFFRAME )
                  {
                     frameBuf->hdr.frame.flags = 1;
                  }
                  else
                  {
                     frameBuf->hdr.frame.flags = 0;
                  }

                  dest_ptr = frameBuf->data;

                  if( tmp_len )
                  {
                     memcpy( frameBuf->data, enc_tmpbuf, tmp_len );
                     dest_ptr += tmp_len;

                     free( enc_tmpbuf );
                     tmp_len = 0;
                     enc_tmpbuf = NULL;
                  }

                  memcpy( dest_ptr, ptr, buffer->nFilledLen );

                  //KNLLOG("TS : %u, len %d, add 0x%x payload 0x%2x\n", frameBuf->hdr.frame.timestamp, pkt_len, (unsigned int)ptr, ptr_32[0]);

                  vc03_frmfwd_hostg->vc_enqueue_framebuf( frameBuf, pkt_len+sizeof(VC03_FrameBufferHdr_t), 0 );
                  encNum++;
               }
               else
               {
                  vc_assert( enc_tmpbuf == NULL );
                  enc_tmpbuf = malloc( MAX_PKT_SIZE );
                  tmp_len = buffer->nFilledLen;

                  memcpy( enc_tmpbuf, ptr, buffer->nFilledLen );   
               }
            }
#if 0
            if( (ptr[0] == 0) && (ptr[1] == 0) && (ptr[2] == 0) && (ptr[3] == 1) )
            {
            }
            else
            {
               //KNLLOG("BAD!!! TS %u len %d, add 0x%x 0x%x\n", frameBuf->hdr.frame.timestamp, buffer->nFilledLen, (unsigned int)ptr, ptr_32[0]);
               printk("bad packet 0x%2x 0x%2x 0x%2x 0x%2x\n", 
                     ptr[0], ptr[1], ptr[2], ptr[3] );
            }
#endif
            error = OMX_FillThisBuffer( ILC_GET_HANDLE( comp ), buffer );
            vc_assert( error == OMX_ErrorNone );
         }
      }
   }
   return 0;  
}
/***************************************************************************/
/**
* vc03_confcmd_thd
*
*  This thread is responsible for processing the packets from the host to the VC03
*  (through OMX)
*
*  @param1
*
*  @return
*
*  @remarks
* 
*/
int vc03_confcmd_thd( void * unused )
{
   //struct sched_param sparm;
   //int rc;
   OMX_ERRORTYPE error;
   DEC_STATE_T * dec_st = &dec_state;
   ENC_STATE_T * enc_st = &enc_state;

   memset( dec_st, 0, sizeof( DEC_STATE_T ) );
   memset( enc_st, 0, sizeof( ENC_STATE_T ) );
#if 0
   sparm.sched_priority = gVcPriorityCmd;
   if( ( rc = sched_setscheduler( current, SCHED_FIFO, &sparm ) ) < 0 )
   {
      printk( KERN_ERR "failed to set the RT priority %d for dec_processThread\n", sparm.sched_priority );
      return rc;
   }
#endif
   printk( KERN_INFO "*******************CONF CMD THD: STARTING*******************\n" );

   while( down_interruptible( &cmd_available ) == 0 )
   {
      printk("vc03 cmd %d received\n", cmd_pending->cmd);
      switch( cmd_pending->cmd )
      {
         case VC03_VIDEOCONF_ADD_DEC:
         {
            //printk("decoder set up now!\n");
            /* receive a start decoder command */
            //memset( dec_st, 0, sizeof( DEC_STATE_T ) );

            vc_assert( dec_active == 0 );
            dec_active = 1;
            decNum = 0;
            vc_assert( dec_st->status == 0 );
            pktlost = -1;

            dec_st->decType = cmd_pending->arg->arg.decParam.decType;
            printk("adding decoder type %d\n", dec_st->decType );

            /* initialize OMX */
            error = OMX_Init();
            vc_assert( error == OMX_ErrorNone );

            dec_st->client = ilclient_init();

            start_decpath_components( dec_st );
            /* tell the command caller (EPT) that we can start sending data packets to the decoder */
            
            cmd_pending = NULL;
            up( &cmd_returnready );

            start_decpath_tunnel( dec_st );
         }
         break;

         case VC03_VIDEOCONF_DEL_DEC:
         {
            dec_active = 0;
            stop_decpath( dec_st );
            error = OMX_Deinit();

            ilclient_destroy(dec_st->client);

            /* clean up the decoder state memory again */
            memset( dec_st, 0, sizeof( DEC_STATE_T ) );

            cmd_pending = NULL;
            up( &cmd_returnready );
         }
         break;

         case VC03_VIDEOCONF_DEC_DISPLAY_CONFIG:
         {
            /* configure the display region on the LCD screen */
            /* setting display region, hardcode to full screen for the time being */
            /* region not really configured until we send this config down to the component port */
            dec_st->region.nVersion.nVersion = OMX_VERSION;
            dec_st->region.nSize = sizeof( dec_st->region );
            dec_st->region.set = OMX_DISPLAY_SET_NUM | OMX_DISPLAY_SET_FULLSCREEN | 
  	                              OMX_DISPLAY_SET_TRANSFORM | OMX_DISPLAY_SET_LAYER |
                                 OMX_DISPLAY_SET_DEST_RECT | OMX_DISPLAY_SET_MODE | OMX_DISPLAY_SET_NOASPECT;

            dec_st->region.num = cmd_pending->arg->arg.display.device_num;
            dec_st->region.nPortIndex = 90;
            dec_st->region.transform = OMX_DISPLAY_ROT0;
            dec_st->region.fullscreen = 0;
            dec_st->region.layer = cmd_pending->arg->arg.display.layer;
            dec_st->region.dest_rect.x_offset = cmd_pending->arg->arg.display.x_offset;
            dec_st->region.dest_rect.y_offset = cmd_pending->arg->arg.display.y_offset;
            dec_st->region.dest_rect.width = cmd_pending->arg->arg.display.width;
            dec_st->region.dest_rect.height = cmd_pending->arg->arg.display.height;
            dec_st->region.mode = OMX_DISPLAY_MODE_FILL;
            dec_st->region.noaspect = 1;

            cmd_pending = NULL;
            up( &cmd_returnready );
            
         }
         break;
         case VC03_VIDEOCONF_ADD_ENC:
         {
            //printk("add encoder stream\n");

            /* set up a whole bunch of parameters, these should be done through a cofigration instead */
            enc_st->focus_mode = OMX_IMAGE_FocusControlHyperfocal;
            enc_st->focus_position = 215;
            /* save encoder configuration details */
            enc_st->capture_width = cmd_pending->arg->arg.encParam.resolutionWidth;
            enc_st->capture_height = cmd_pending->arg->arg.encParam.resolutionHeight;
            enc_st->bitrate = cmd_pending->arg->arg.encParam.bitRate;
            enc_st->framerate = cmd_pending->arg->arg.encParam.frameRate;
            enc_st->encType = cmd_pending->arg->arg.encParam.codecType;
            enc_st->camplus_id = 'CAMC';
            enc_st->camera_num = 0;
            enc_st->header_size = 0;

            printk("type %d, width %d height %d bitrate %d framerate %d\n",
                  enc_st->encType, enc_st->capture_width,
                  enc_st->capture_height, enc_st->bitrate,
                  enc_st->framerate );

            enc_st->scale = 1 << 16;

            error = OMX_Init();
            vc_assert( error == OMX_ErrorNone );

            enc_st->client = ilclient_init();
            seq_num = 0;
            encNum = 0;
            first_pkt = 1;
            enc_active = 1;

            start_encpath( enc_st );
            cmd_pending = NULL;
            up( &cmd_returnready );
         }
         break;

         case VC03_VIDEOCONF_DEL_ENC:
         {
            printk("del encoder stream, encNum %d seq %d\n", encNum, seq_num);
            enc_active = 0;
            stop_encpath( enc_st );

            error = OMX_Deinit();
            ilclient_destroy(enc_st->client);

            /* clean up the decoder state memory again */
            memset( enc_st, 0, sizeof( ENC_STATE_T ) );

            cmd_pending = NULL;
            up( &cmd_returnready );
         }
         break;

         case VC03_VIDEOCONF_ENC_DISPLAY_CONFIG:
         {
            printk("change encoder display\n");
            /* region not really configured until we send this config down to the component port */
            enc_st->region2.nVersion.nVersion = OMX_VERSION;
            enc_st->region2.nSize = sizeof( enc_st->region2 );
            enc_st->region2.set = OMX_DISPLAY_SET_NUM | OMX_DISPLAY_SET_FULLSCREEN | 
                                  OMX_DISPLAY_SET_TRANSFORM | OMX_DISPLAY_SET_LAYER | 
                                  OMX_DISPLAY_SET_DEST_RECT | OMX_DISPLAY_SET_MODE;

            enc_st->region2.num = cmd_pending->arg->arg.display.device_num;
            enc_st->region2.nPortIndex = 90;
            enc_st->region2.transform = OMX_DISPLAY_ROT0;
            enc_st->region2.fullscreen = 0;
            enc_st->region2.dest_rect.x_offset = cmd_pending->arg->arg.display.x_offset;
            enc_st->region2.dest_rect.y_offset = cmd_pending->arg->arg.display.y_offset;
            enc_st->region2.dest_rect.width = cmd_pending->arg->arg.display.width;
            enc_st->region2.dest_rect.height = cmd_pending->arg->arg.display.height;
            enc_st->region2.mode = OMX_DISPLAY_MODE_FILL;
            enc_st->region2.noaspect = 1;
            enc_st->region2.layer = cmd_pending->arg->arg.display.layer;
            cmd_pending = NULL;
            up( &cmd_returnready );
         }
         break;

         default:
         {
            printk("cmd not supported\n");
         }
         break;
      };
   }
   return 1;
}
/***************************************************************************/
/**
* vc03_sendframe
*
*  dequeues frames from the host application (EPT in this case) and sends 
*  the frames to the VC03.  Currently the data is enqueued in a list and a thread
*  (FrameSendThread) reads it out from this queue and frees the buffer after the
*  data has been sent to the VC03.
*
*  @param1
*
*  @return
*
*  @remarks
* 
*/
void vc03_sendframe ( int devhdl )
{
   VC03_FrameBuffer_t *frameBuf;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,25)
   /* FIXME: we should not access the structure semaphore member directly */
  queueDepth = gVC_FramesToVCList.availSem.count;
#else
  queueDepth = atomic_read(&gVC_FramesToVCList.availSem.count);
#endif
  if( maxQueueDepth < queueDepth )
  {
     maxQueueDepth = queueDepth;
  }

   if( vc03_frmfwd_hostg == NULL )
   {
      printk( KERN_ERR "vc03_sendframe : cannot send the frame; host callback not registered\n");
      return;
   }

   while( (frameBuf = vc03_frmfwd_hostg->vc_dequeue_framebuf( devhdl )) != NULL )
   {

      //KNLLOG("len = %d\n", frameBuf->hdr.frame.data_len);
      if( (frameBuf->hdr.frame.data_len > MAX_PKT_SIZE) ||
           (((frameBuf->hdr.frame.data_len & ~(0x0F)) + 0x10) > MAX_PKT_SIZE ))
      {
         printk("packet too large %d\n", frameBuf->hdr.frame.data_len);
         vc03_frmfwd_hostg->vc_free_framebuf( frameBuf );
      }
      else
      {
         VC03_EnqueueFrameBuffer( &gVC_FramesToVCList, frameBuf );
      }
   }
}

/***************************************************************************/
/**
* vc03_confcmd
*
*  Call this function (with appropriate arguments) when we need to perform the
*  following tasks for video conferencing:
*   a) start a video decoder path
*   b) start a video encoder path
*   c) change the display region
*   d) etc
*
*  @param1
*
*  @return
*
*  @remarks
* 
*/
void vc03_confcmd( VC03_VIDEOCONF_CMD * cmdp )
{
   /* we can only process 1 command at a time */
   vc_assert( cmd_pending == NULL );
   cmd_pending = cmdp;
   up( &cmd_available );
   if( down_interruptible( &cmd_returnready ) != 0 )
   {
      printk("cmd %d failed\n", cmdp->cmd );
      vc_assert(0);
   }
   printk("done cmd processing for %d\n", cmdp->cmd );
}

/***************************************************************************/
EXPORT_SYMBOL(vc03_host_set_cmd_callback);
EXPORT_SYMBOL(vc03_sendframe);
EXPORT_SYMBOL(vc03_confcmd);
