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




/*=============================================================================

Project  :  VMCS Host Apps
Module   :  IL Demos - Play
File     :  $RCSfile: play.c,v $
Revision :  $Revision: 1.1.2.10 $

FILE DESCRIPTION
Play demo
=============================================================================*/

#define PLAYING_LENGTH_MSEC      10000
#define CAMERA_PREVIEW_RENDERER  1


#include <stdarg.h>
#include <linux/broadcom/vc03/vcilcs_intern.h>
#include <linux/broadcom/vc03/vcomx.h>
#include <linux/broadcom/vc03/vcilclient.h>
#include <linux/broadcom/vc03/vcilplatform.h>
#include <linux/broadcom/vc03/dispmanx_types.h>

#include <linux/broadcom/knllog.h>
#include <linux/file.h>
#include <linux/fs.h>

#include <linux/kthread.h>
#include <linux/syscalls.h>

#if !defined(__KERNEL__)

#define malloc libomx_malloc
#define free   libomx_free
#include <string.h>
#include <stdio.h>
#include "omxctl.h"

#define local_printf printf

#else

#define local_printf ilclient_debug_output

#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,26)
#   include <linux/semaphore.h>
#else
#   include <asm/semaphore.h>
#endif



#define OMX_VERSION 0x10101


/* This component will handle URI's up to this maximum length */
#define ILTEST_CONTENTURI_MAXLEN 256

typedef struct {
   PLATFORM_EVENTGROUP_T events;

   char error_msg[64];
   OMX_S32 scale;

   // pointers to OpenMAX elements
   ILCLIENT_T *client;
   COMPONENT_T *clock;

#if CAMERA_PREVIEW_RENDERER
   COMPONENT_T *list[5];
//   TUNNEL_T tunnels[9];
   TUNNEL_T ctunnels[3];

   COMPONENT_T *camera;
   COMPONENT_T *video_render;
   COMPONENT_T *video_encode;

   OMX_CONFIG_DISPLAYREGIONTYPE region2;
   TUNNEL_T ca_di_tunnel[2];
   TUNNEL_T ca_ve_tunnel[2];

   OMX_U32 camplus_id;
   int camera_num;
   //char isp_tuner[32];
   OMX_IMAGE_FOCUSCONTROLTYPE focus_mode;
   OMX_U32 focus_position;
   int capture_width;
   int capture_height;

#else
   COMPONENT_T *list[11];
   TUNNEL_T tunnels[8];
   TUNNEL_T ctunnels[6];


#endif

   OMX_BUFFERHEADERTYPE *buffer_list;
   OMX_VIDEO_CODINGTYPE encType;
   int bitrate;
   unsigned char *header_ava;
   unsigned char *header_bytes;
   int header_size;

} CAM_STATE_T;
#if 0
static unsigned char config_prm[] =
{
   0x00,
   0x00,
   0x00,
   0x01,
   0x27,
   0x4d,
   0x40,
   0x1f,
   0x96,
   0x54,
#if 1 /* this if for HD */
   0x02, 
   0x80,
   0x2d,
   0xc8,
#endif
   
#if 0 /* VGA */
   0x05,
   0x01,
   0xec,
   0x80,
#endif
#if 0 /* QVGA */
   0x0a,
   0x0f,
   0xc8,
#endif
   0x00,
   0x00,
   0x00,
   0x01,
   0x28,
   0xee,
   0x02,
   0x58,
   0x80
};
#endif

#define ilclient_debug_output OMX_DEBUG

#define DISPMANX_FRAMEBUF_SCREEN_DISPLAY     0
CAM_STATE_T cam_st;

int vide_encoded_cnt = 0;

static int process_started = 0;
static int noeos = 1;
static struct file *save_hdl;

int defaultEnc = 7; /* default encoder is H.264 */
int testwidth = 1280;
int testheight = 720;


#define RECORD_IL_MAX_FRAMERATE             20
#define RECORD_IL_IP_WIDTH                1600
#define RECORD_IL_IP_HEIGHT                900
#define RECORD_IL_NUM_HI_RES_POOL_FRAMES     2

static int eventThreadPid = 0;
struct semaphore encpkt_available;
struct semaphore encfile_protect;
struct completion eventExited;

static int eventProcessThread( void * unused );

static void output_buffer_callback(void *data, COMPONENT_T *comp)
{
   up( &encpkt_available );

}
static void eos_callback(void *userdata, COMPONENT_T *comp, OMX_U32 data)
{
   //platform_eventgroup_set(userdata, EV_EOS, PLATFORM_EVENTGROUP_OPERATION_OR);
   //if( comp == cam_st.video_encode )
   {
      noeos = 0;
   }
   printk("eos_callback\n");
   //noeos = 0;
}

#if 0
static void input_buffer_callback(void *data, COMPONENT_T *comp)
{
   //platform_eventgroup_set(data, EV_INPUT_BUFFER_ARRIVED, PLATFORM_EVENTGROUP_OPERATION_OR);
   printk("input_buffer callback\n");
}

static void on_port_settings_changed(void *data, COMPONENT_T *comp, OMX_U32 port)
{
   printk("on port settings changed %d\n", port );
}
#endif

static int configure_camera(CAM_STATE_T *st)//, OMX_BOOL want_display)
{
   OMX_ERRORTYPE error;

   ilclient_disable_port(st->camera, 70); // disable the preview port
   ilclient_disable_port(st->camera, 71); // disable the capture port
   ilclient_disable_port(st->camera, 72); // disable the time port

   ilclient_disable_port(st->video_render, 90);

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

      printk("setting 2 POOLS!!!!!\n");
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
      param.format.video.xFramerate = RECORD_IL_MAX_FRAMERATE << 16;
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
      COMPONENT_T *list[] = {st->camera, st->video_render, NULL};

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
      strcpy(st->error_msg, "Failed to load components");
      return -1;
   }
}

static int32_t open_cam(CAM_STATE_T *st)
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

   ilclient_create_component(st->client, &st->video_render, "video_render", 0, 1, 0);
   *comp++ = st->video_render;
   ilclient_disable_port(st->video_render, 90);

   // setup display region if already set
   if (region2.set)
   {
      error = OMX_SetConfig(ILC_GET_HANDLE(st->video_render), OMX_IndexConfigDisplayRegion, &region2);
      vc_assert(error == OMX_ErrorNone);
   }

   ilclient_create_component(st->client, &st->clock, "clock", 0, 0, 0);
   *comp++ = st->clock;
   set_tunnel(st->ca_di_tunnel, st->camera, 70, st->video_render, 90);

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
            //buffers[i].enc_buf->pInputPortPrivate = NULL;
       }

       ilclient_change_component_state(st->video_encode, OMX_StateLoaded);

       ilclient_change_component_state(st->video_encode, OMX_StateIdle);  
   }

   configure_camera(st);

   // Configure clock
   {
      //disable capture and clock ports on camera component
      ilclient_disable_port(st->camera, 71); // disable the capture port
      ilclient_disable_port(st->camera, 72); // disable the clock port

      // disable all ports apart from the first two
      OMX_PORT_PARAM_TYPE param;
      OMX_TIME_CONFIG_ACTIVEREFCLOCKTYPE refclock;

      int i;

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
      printk("done setting clock????\n");
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
      param.format.video.xFramerate = RECORD_IL_MAX_FRAMERATE << 16;
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
   
   printk("now setting tunnel\n");
   i = ilclient_setup_tunnel( st->ca_ve_tunnel, 0, PLATFORM_EVENTGROUP_SUSPEND );
   if( i != 0 )
   {
      printk("setup tunnel retusn %d\n", i );
      //vc_assert(0);
   }
   printk("done setting tunnel\n");

   ilclient_enable_port( st->camera, 71 );
   ilclient_enable_port(st->video_encode, 201); //enable video encode port
   printk("done calling port enable\n");
   ilclient_change_component_state(st->video_encode, OMX_StateExecuting);
   //ilclient_state_transition(st->list, OMX_StateExecuting);


   {
      OMX_CONFIG_DISPLAYREGIONTYPE current_region;
       
      memset( &current_region, 0, sizeof( OMX_CONFIG_DISPLAYREGIONTYPE ) );
      current_region.nSize = sizeof( OMX_CONFIG_DISPLAYREGIONTYPE );
      current_region.nVersion.nVersion = OMX_VERSION;
      current_region.nPortIndex = 90;
      error = OMX_GetConfig( ILC_GET_HANDLE( st->video_render ), OMX_IndexConfigDisplayRegion, 
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

   
   local_printf("playing video/audio... \n");
   local_printf("hit return to terminate...\n");
   return 0;
}

static int32_t stop_cam(CAM_STATE_T *st)
{
   OMX_ERRORTYPE error;
   OMX_BUFFERHEADERTYPE *buffer;
   OMX_TIME_CONFIG_CLOCKSTATETYPE cstate;
   OMX_PORT_PARAM_TYPE param;
   COMPONENT_T *vlist[] = {st->camera, st->clock, st->video_encode,NULL};
   COMPONENT_T *vlist2[] = {st->camera, st->clock,NULL};
   int i=0;

  	ilclient_debug_output("terminate playing\n");
  	KNLLOG("terminate playing\n");

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
   //mdelay(500);
   while( noeos )
   {
      set_current_state(  TASK_INTERRUPTIBLE );
      schedule_timeout(10);
   }
   printk("eos should have arrived?\n");
#endif

   ilclient_change_component_state(st->video_render, OMX_StateIdle);
   ilclient_change_component_state(st->video_render, OMX_StateLoaded);

   ilclient_state_transition(vlist, OMX_StateIdle);
   KNLLOG("done IDLE state\n");
   ilclient_state_transition(vlist2, OMX_StateLoaded);
   KNLLOG("done Loaded state\n");

   error = OMX_SendCommand(ILC_GET_HANDLE(st->video_encode), OMX_CommandStateSet, OMX_StateLoaded, NULL);
   vc_assert(error == OMX_ErrorNone);   
   
   KNLLOG("sent encode loaded cmd\n");
#if 1
   while ( (buffer=ilclient_get_output_buffer(st->video_encode, 201)) != NULL )
   {
      unsigned char * buf = buffer->pBuffer;

      KNLLOG("get buffer 0x%x\n", (unsigned int)buffer );

      error = OMX_FreeBuffer(ILC_GET_HANDLE(st->video_encode), 201, buffer);
      vc_assert(error == OMX_ErrorNone);
      KNLLOG("freebuffer\n");
      
      free( buf );
   }
#endif

   KNLLOG("now waiting for event change\n");
   ilclient_wait_for_event(st->video_encode, OMX_EventCmdComplete, OMX_CommandStateSet, 0, OMX_StateLoaded, 0,
                           ILCLIENT_STATE_CHANGED, PLATFORM_EVENTGROUP_SUSPEND);
   KNLLOG("done waiting for event change\n");

   // teardown tunnels
   ilclient_teardown_tunnels(st->ca_di_tunnel);
   printk("done tunnel 1!!!!\n");
   ilclient_teardown_tunnels(st->ca_ve_tunnel);

   printk("done tunnel 2!!!!\n");
#if 1
   {// Destroy clock tunnel
      error = OMX_SetupTunnel(ILC_GET_HANDLE(st->clock), param.nStartPortNumber, NULL, 0);
      vc_assert(error == OMX_ErrorNone);

      error = OMX_SetupTunnel(ILC_GET_HANDLE(st->camera), 72, NULL, 0);
      vc_assert(error == OMX_ErrorNone);
   }
#endif
   ilclient_return_events(st->camera);

	ilclient_debug_output("ilclient_teardown_tunnels\n");

	// delete components
	ilclient_cleanup_components(st->list);
	ilclient_debug_output( "ilclient_cleanup_components\n");
   free( st->header_ava );
   st->header_ava = 0;
	
	error = OMX_Deinit();
	ilclient_debug_output( "OMX_Deinit() %d\n",error);

	ilclient_destroy(st->client);
 	local_printf("finish playing\n");

   return 0;
   
}

#define EVENTTHD_RT_PRIORITY 0

static int eventProcessThread( void * unused )
{
   int rc;
   OMX_ERRORTYPE error;
   OMX_BUFFERHEADERTYPE *buffer;
   mm_segment_t old_fs;
   int nbytes;
   unsigned int *ptr;
   unsigned char *ptr_8;
   //struct sched_param sparm;
   COMPONENT_T *comp = cam_st.video_encode;

   daemonize( "vc_enc_event" );
#if 0
   sparm.sched_priority = EVENTTHD_RT_PRIORITY ;

   if( ( rc = sched_setscheduler( current, SCHED_FIFO, &sparm ) ) < 0 )
   {
      printk( KERN_ERR "failed to set the RT priority %d for dec_processThread\n", sparm.sched_priority );
      return rc;
   }
#endif
   while( down_interruptible( &encpkt_available ) == 0 )
   {
      comp = cam_st.video_encode;
      if( (process_started == 1) && (save_hdl) )
      {
         down( &encfile_protect );
#if 0
         while( encfile_protect )
         {
            mdelay(10);
         }
#endif
         buffer=ilclient_get_output_buffer(comp, 201);
         if( buffer )
         {
            ptr = buffer->pBuffer;
            ptr_8 = buffer->pBuffer;
            if( cam_st.encType == OMX_VIDEO_CodingAVC )
            {
               if( (ptr_8[0] == 0) && (ptr_8[1] == 0) && (ptr_8[2] == 0) && (ptr_8[3] == 1) )
               {
                  if( !(buffer->nFlags & OMX_BUFFERFLAG_ENDOFFRAME) )
                  {
                     printk("not end of frame TS %u\n", (unsigned int)buffer->nTimeStamp);
                  }
               }
               else
               {
                  printk("bad pkt flags = 0x%x TS %u\n", buffer->nFlags, (unsigned int)buffer->nTimeStamp);
                  KNLLOG("bad!!!! 0x%x 0x%x 0x%x 0x%x\n", ptr[0], ptr[1], ptr[2], ptr[3] );
               }
            }

            old_fs = get_fs();
            set_fs( get_ds() );
            
            if( (cam_st.encType == OMX_VIDEO_CodingAVC) && (buffer->nFlags & OMX_BUFFERFLAG_SYNCFRAME) )
            {
               if( cam_st.header_ava == NULL )
               {
                  int i;
                  unsigned char *buf = malloc( 200 );
#if 0
                  OMX_PARAM_CODECCONFIGTYPE * cConfig = (OMX_PARAM_CODECCONFIGTYPE *)buf;

                  cConfig->nSize = 100;
                  cConfig->nVersion.nVersion = OMX_VERSION;
                  cConfig->nPortIndex = 201;

                  error = OMX_GetParameter(ILC_GET_HANDLE(comp), OMX_IndexParamCodecConfig, cConfig);
                  printk("error = 0x%x\n", error );
                  printk("nSize = %d\n", cConfig->nSize );
                  printk("nPortIndex = %d\n", cConfig->nPortIndex );
                  for( i=0; i < 40; i+=5 )
                  {
                     printk("nData 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x\n", cConfig->nData[i+0], cConfig->nData[i+1],
                     cConfig->nData[i+2], cConfig->nData[i+3],cConfig->nData[i+4] );
                  }
#endif
                  OMX_PARAM_CODECHEADERBYTES * cConfig = (OMX_PARAM_CODECHEADERBYTES *)buf;

                  cConfig->nSize = 100;
                  cConfig->nVersion.nVersion = OMX_VERSION;
                  cConfig->nPortIndex = 201;

                  error = OMX_GetParameter(ILC_GET_HANDLE(comp), OMX_IndexParamCodecHeader, cConfig);
                  printk("get header error = 0x%x\n", error );
                  printk("nSize = %d\n", cConfig->nSize );
                  printk("nPortIndex = %d\n", cConfig->nPortIndex );

                  cam_st.header_ava = buf;
                  cam_st.header_bytes = cConfig->nData;
                  cam_st.header_size = cConfig->nSize;
               }
               printk("writing sps and pps (ptr[1] = 0x%x)\n", ptr[1]);
               //save_hdl->f_op->write( save_hdl, config_prm, sizeof( config_prm ), &save_hdl->f_pos );
               save_hdl->f_op->write( save_hdl, cam_st.header_bytes, cam_st.header_size, &save_hdl->f_pos );
            }
            //nbytes = sys_write( save_hdl, ptr, buffer->nFilledLen );
            save_hdl->f_op->write( save_hdl, ptr, buffer->nFilledLen, &save_hdl->f_pos );
            assert( buffer->nFilledLen );

            set_fs( old_fs );
            //KNLLOG("0x%x, len = %d[ 0x%x 0x%x]\n", (unsigned int)buffer, buffer->nFilledLen, ptr[0], ptr[1] );

            error = OMX_FillThisBuffer(ILC_GET_HANDLE(comp), buffer);
            vc_assert(error == OMX_ErrorNone);
            vide_encoded_cnt++;
            if( (vide_encoded_cnt % 100) == 0 )
            {
               printk("100 messages received\n");
            }
         }
         up( &encfile_protect );
      }
      else
      {
#if 0
         while ( (buffer=ilclient_get_output_buffer(comp, 201)) != NULL )
         {
            unsigned char * buf = buffer->pBuffer;

            printk("get buffer 0x%x\n", (unsigned int)buffer );

            error = OMX_FreeBuffer(comp, 201, buffer);
            vc_assert(error == OMX_ErrorNone);
      
            vfree( buf );
         }
#endif
      }
   }
   complete_and_exit( &eventExited, 0 );
   
   return 0;
}

void vc_do_camenc(int display, int start)
{
  OMX_ERRORTYPE error;
  mm_segment_t old_fs;
  int flags;

  if( start == 1 )
  {
     process_started = 1;
     noeos = 1;
     flags = O_WRONLY | O_CREAT;
     old_fs = get_fs();
     set_fs ( get_ds() );
     //save_hdl  = sys_open( "/cap.bin", flags, 0777 );
     //printk("save_hdl = %d !!!! \n", save_hdl );
     save_hdl = filp_open( "/webroot/cap.bin", O_TRUNC | O_WRONLY | O_CREAT, 0644 );
     set_fs( old_fs );
     if( !save_hdl || !save_hdl->f_op || !save_hdl->f_op->write )
     {
        printk("failed to open file for storing bitstream\n");
        return;
     }
     sema_init( &encpkt_available, 0 );
     sema_init( &encfile_protect, 1 );
     init_completion( &eventExited );
     eventThreadPid = kernel_thread( eventProcessThread, 0, 0 );

      memset(&cam_st, 0, sizeof(cam_st));

      cam_st.region2.nVersion.nVersion = OMX_VERSION;
      cam_st.region2.nSize = sizeof(cam_st.region2);
      cam_st.region2.set = OMX_DISPLAY_SET_NUM | OMX_DISPLAY_SET_FULLSCREEN | 
                        OMX_DISPLAY_SET_TRANSFORM | OMX_DISPLAY_SET_LAYER | 
                        OMX_DISPLAY_SET_DEST_RECT | OMX_DISPLAY_SET_MODE;
      cam_st.region2.num = display;
      cam_st.region2.nPortIndex = 90;
      cam_st.region2.transform = OMX_DISPLAY_ROT0;
      cam_st.region2.fullscreen = 0;
      cam_st.region2.dest_rect.x_offset = 0;
      cam_st.region2.dest_rect.y_offset = 0;
      cam_st.region2.dest_rect.width = 800;
      cam_st.region2.dest_rect.height = 480;
      cam_st.region2.mode = OMX_DISPLAY_MODE_FILL;
      cam_st.region2.layer = 3;

      cam_st.focus_mode = OMX_IMAGE_FocusControlHyperfocal;
      cam_st.focus_position = 215;  // this is a "normal" hyperfccal position.
      cam_st.encType = (OMX_VIDEO_CODINGTYPE)defaultEnc;
      cam_st.capture_width = testwidth;
      cam_st.capture_height = testheight;
      cam_st.header_ava = NULL;
      cam_st.header_bytes = NULL;
      cam_st.bitrate = 500000;
      cam_st.camplus_id = 'CAMC';
      cam_st.camera_num = 0;
      //strcpy(cst.isp_tuner, "vb6850");

//   cst.newscale = 
      cam_st.scale = 1 << 16;
//   cst.newplaymode = cst.playmode = OMX_PLAYMODE_NORMAL;

      error = OMX_Init();
      cam_st.client = ilclient_init();
      open_cam(&cam_st);
  }
  else
  {
     process_started = 0;
     KNLLOG("process_started now 0\n");
     printk("waiting for protect\n");

     KNLLOG("down\n");
     down( &encfile_protect );
     old_fs = get_fs();
     set_fs (get_ds());
     filp_close( save_hdl, current->files );
     set_fs (old_fs);
     save_hdl =  NULL;
     up( &encfile_protect );
     KNLLOG("up\n");
     /* tear down */
     stop_cam( &cam_st );
  }
#if defined(__KERNEL__)
 // os_delay(10000);
#endif
}
